"""Latest-only LiDAR/IMU display mapper. No motor, odometry or planner output.

The bridge has display-decimated returns, not calibrated rolling-ray timings.
The estimate is explicitly unqualified. Missing attitude, scan gaps, ambiguous
registration and tilt freeze insertion rather than declare measured free space.
"""
import asyncio
import base64
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid
import re
import struct
import zlib
from collections import deque

import numpy as np
from dashboard_server.occupancy_map import atomic_json, read_json
from dashboard_server.tiled_occupancy import TiledOccupancyMap as OccupancyMap


def attitude(raw):
    if (raw.get('availability',{}).get('orientation') or {}).get('available') is False:
        raise ValueError('imu_orientation_unavailable')
    q = raw.get('orientation') or raw.get('quaternion') or {}
    x,y,z,w = (float(q[k]) for k in ('x','y','z','w'))
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not .9 < norm < 1.1: raise ValueError('invalid_imu_orientation')
    x,y,z,w = (v/norm for v in (x,y,z,w))
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1,min(1,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def translation(previous, current, delta_yaw):
    """Bounded translation-only ICP with the raw IMU heading difference.

    This provides a display estimate, not a qualified body-speed measurement.
    A geometrically rank-deficient match cannot extend the accumulated map.
    """
    c,s = math.cos(delta_yaw),math.sin(delta_yaw)
    rotated = current @ np.array([[c,s],[-s,c]])
    shift = np.zeros(2)
    for _ in range(8):
        shifted = rotated + shift
        distances = ((shifted[:,None,:]-previous[None,:,:])**2).sum(axis=2)
        nearest = distances.argmin(axis=1)
        residual = previous[nearest]-shifted
        lengths = np.sqrt(distances[np.arange(len(current)),nearest])
        good = lengths < min(.45,float(np.quantile(lengths,.7))+.03)
        if good.sum() < 90: raise ValueError('insufficient_scan_overlap')
        shift += np.median(residual[good],axis=0)
    support = previous[nearest[good]]
    eigen = np.linalg.eigvalsh(np.cov(support.T))
    rmse = float(np.sqrt(np.mean(lengths[good]**2)))
    if good.mean() < .55 or rmse > .12 or eigen[0] < .06 or np.linalg.norm(shift) > .5:
        raise ValueError('scan_registration_uncertain')
    return shift, dict(inlier_ratio=round(float(good.mean()),3),rmse_m=round(rmse,3))


class LiveMapping:
    def __init__(self, state_dir, start_worker=True):
        self.state_dir = Path(state_dir)
        self.lock = threading.RLock()
        self.inbox_lock = threading.Lock()
        self.imu_history = deque(maxlen=60)
        self.map = OccupancyMap(max_beams=450,laser_x=0.,laser_y=0.)
        self.map_epoch = uuid.uuid4().hex
        self.version = 0
        self.active = False
        self.tracking = False
        self.state = 'idle'
        self.reason = None
        self.latest = {}
        self.previous = None
        self.last_source_stamp = None
        self.pose = np.zeros(3)
        self.diagnostic = {}
        self.snapshot = {}
        self.stop_event = threading.Event()
        self.threads = []
        self.restored_snapshot = None
        self.persistence = {}
        self._publish()
        self._restore_saved_map()
        if start_worker:
            for kind,url in [('scan',os.environ.get('LIDAR_WS_UPSTREAM','ws://127.0.0.1:8765/ws/lidar')+'?max_points=900&profile=detail'),
                             ('imu',os.environ.get('IMU_WS_UPSTREAM','ws://127.0.0.1:8775/ws/imu')+'?profile=detail')]:
                thread = threading.Thread(target=lambda k=kind,u=url:asyncio.run(self._receive(k,u)),daemon=True)
                thread.start();self.threads.append(thread)
            thread=threading.Thread(target=self._run,daemon=True,name='display-occupancy-map')
            thread.start();self.threads.append(thread)

    async def _receive(self, kind, url):
        import websockets
        while not self.stop_event.is_set():
            try:
                async with websockets.connect(url,max_queue=1,max_size=2**20,open_timeout=3,close_timeout=1) as socket:
                    while not self.stop_event.is_set():
                        try: packet=json.loads(await asyncio.wait_for(socket.recv(),1))
                        except asyncio.TimeoutError: continue
                        if packet.get('type') != kind: continue
                        # Sensor receipt never waits for ICP/grid integration.
                        # A small history pairs an older scan with its actual
                        # attitude, rather than whichever IMU arrived last.
                        with self.inbox_lock:
                            row=(packet,time.monotonic());self.latest[kind]=row
                            if kind=='imu':self.imu_history.append(row)
            except (OSError,ValueError,websockets.exceptions.WebSocketException):
                await asyncio.sleep(1)

    def _run(self):
        while not self.stop_event.wait(.5):
            with self.lock:
                if not self.tracking: continue
            with self.inbox_lock:
                scan=self.latest.get('scan');history=list(self.imu_history)
            def distance(row):
                try:
                    stamp=row[0]['raw']['stamp']
                    return abs(float(stamp['sec'])+float(stamp['nanosec'])*1e-9-float(scan[0]['stamp']))
                except (KeyError,TypeError,ValueError):return math.inf
            imu=min(history,key=distance) if scan and history else None
            if scan and imu:
                try: self.ingest(scan,imu)
                except (KeyError,ValueError,TypeError,OverflowError) as error:
                    self.freeze(str(error))
            elif scan or imu:
                self.freeze('lidar_or_imu_unavailable')

    def freeze(self, reason):
        with self.lock:
            if not self.tracking:return
            # Sensor arrival order/temporary receipt delay is not a failed
            # geometric match. Keep listening without inserting any cells.
            # An established pose still has to pass the source-clock gap
            # check before drawing resumes; unknown travel is never filled.
            if reason in ('lidar_or_imu_unavailable','stale_lidar_or_imu',
                          'unaligned_lidar_imu','imu_orientation_unavailable'):
                self.state='waiting_for_sensors';self.reason=reason;self._publish()
                return
            self.active=self.tracking=False
            self.state='needs_reset';self.reason=reason;self._publish()

    def ingest(self, scan, imu, now=None):
        now=time.monotonic() if now is None else now
        frame,received=scan;imu_frame,imu_received=imu
        stamp=float(frame['stamp'])
        if not math.isfinite(stamp):raise ValueError('invalid_scan_stamp')
        if not 0 <= now-received <= 1.5 or not 0 <= now-imu_received <= 1.5 or imu_frame.get('imu_topic_alive') is not True:
            raise ValueError('stale_lidar_or_imu')
        raw=imu_frame.get('raw') or {}
        imu_stamp=raw.get('stamp') or {}
        source_imu=float(imu_stamp['sec'])+float(imu_stamp['nanosec'])*1e-9
        if not math.isfinite(source_imu) or abs(source_imu-stamp)>.2:raise ValueError('unaligned_lidar_imu')
        angles=[];ranges=[]
        for p in frame.get('points',[]):
            angle,distance=float(p['angle_deg']),float(p['distance_m'])
            if math.isfinite(angle) and math.isfinite(distance) and .05 <= distance < 30:
                angles.append(math.radians(angle));ranges.append(distance)
        if len(ranges)<120:raise ValueError('insufficient_lidar_returns')
        # ROS LaserScan +X is forward; the bridge's displayed X/Y is swapped.
        points=np.column_stack((np.cos(angles)*ranges,np.sin(angles)*ranges))
        points=points[np.linspace(0,len(points)-1,min(len(points),450),dtype=int)]
        orientation=attitude(raw)
        if max(abs(orientation[0]),abs(orientation[1]))>math.radians(12):raise ValueError('imu_tilt_exceeds_display_limit')
        with self.lock:
            if not self.tracking:return False
            if self.last_source_stamp == stamp:return False
            if self.last_source_stamp is not None:
                dt=stamp-self.last_source_stamp
                if not 0 < dt <= 2.:raise ValueError('scan_clock_gap_or_reset')
                prev_points,prev_attitude=self.previous
                if max(abs(orientation[i]-prev_attitude[i]) for i in (0,1))>math.radians(3):
                    raise ValueError('imu_tilt_changed')
                delta=math.atan2(math.sin(orientation[2]-prev_attitude[2]),math.cos(orientation[2]-prev_attitude[2]))
                if abs(delta)>math.radians(35):raise ValueError('imu_heading_gap')
                shift,self.diagnostic=translation(prev_points,points,delta)
                c,s=math.cos(self.pose[2]),math.sin(self.pose[2])
                self.pose[:2]+=np.array([[c,-s],[s,c]])@shift
                self.pose[2]+=delta
            self.previous=(points,orientation);self.last_source_stamp=stamp
            if self.active:
                # Preserve irregular real angles; absent returns stay unknown.
                # Integrate one complete scan so occupied endpoints win over
                # crossing free rays within that very same generation.
                sample=dict(pose=dict(x=float(self.pose[0]),y=float(self.pose[1]),yaw=float(self.pose[2]),stamp=stamp),
                            stamp=stamp,scan=dict(ranges=ranges,angles=angles,angle_min=0.,angle_increment=1.,range_min=.05,range_max=30.))
                self.map.update(sample)
                self.version+=1
                self.state='drawing';self.reason=None;self._publish()
            return True

    def _publish(self):
        self.snapshot={**(self.restored_snapshot or self.map.snapshot()),
                       'ok':True,'simulated':False,'active':self.active,'state':self.state,
                       'reason':self.reason,'version':self.version,'map_epoch':self.map_epoch}
        # Historical PNGs are retained only as displays. They must not replace
        # the current process's state/authority or provide a resume pose.
        self.snapshot.update(active=self.active,state=self.state,reason=self.reason,
                             version=self.version,map_epoch=self.map_epoch)
        self.snapshot.update(scan_count=self.version,coordinate_frame='unqualified_lidar_origin',
                             runtime_authority=False,autonomy_eligible=False,
                             localization='unqualified_lidar_imu_display_without_loop_closure',
                             pose_source='unqualified_lidar_imu_display',mount_height_m=.60,
                             diagnostic=dict(self.diagnostic),source='display_decimated_bridge',
                             note='표시용 LiDAR·IMU 위치 추정 · 장착 방향·rolling timing 미검증 · 주행 제어에 사용하지 않음')
        if self.persistence:
            self.snapshot['persistence']=dict(self.persistence)
            if self.persistence.get('native_cells') and self.restored_snapshot is None:
                self.snapshot['native_checkpoint']='last_native_map.json'

    def _restore_saved_map(self):
        path=self.state_dir/'last_map.json'
        if not path.is_file():return
        try:
            if path.is_symlink() or path.stat().st_size>8*1024*1024:
                raise ValueError('invalid_saved_map_size')
            saved=read_json(path)
            count=saved.get('scan_count',0)
            if type(count) is not int or not 0<=count<=1000000000:
                raise ValueError('invalid_saved_map_count')
            if not count:return
            epoch=saved.get('map_epoch','')
            if not re.fullmatch(r'[0-9a-f]{32}',epoch):
                raise ValueError('invalid_saved_map_epoch')
            if saved.get('native_checkpoint'):
                if saved['native_checkpoint']!='last_native_map.json':
                    raise ValueError('invalid_saved_map_checkpoint_path')
                from dashboard_server.tiled_checkpoint import digest
                native=self.state_dir/'last_native_map.json'
                if digest(native)!=(saved.get('persistence') or {}).get('manifest_sha256'):
                    raise ValueError('saved_map_generation_mismatch')
                self.map=OccupancyMap.load_checkpoint(native)
                if self.map.scan_count!=count:
                    raise ValueError('saved_map_generation_mismatch')
                self.persistence={**saved['persistence'],'restored':True,'native_cells':True}
            else:
                grid=saved.get('grid') or {}
                w,h=grid.get('width'),grid.get('height')
                if type(w) is not int or type(h) is not int or not 1<=w*h<=4194304 or min(w,h)<1:
                    raise ValueError('invalid_saved_map_grid_size')
                png=base64.b64decode(grid.get('image_base64',''),validate=True)
                if len(png)<24 or png[:8]!=b'\x89PNG\r\n\x1a\n' or struct.unpack('>II',png[16:24])!=(w,h):
                    raise ValueError('invalid_saved_map_png')
                origin=grid.get('origin') or {}
                if not all(type(v) in (int,float) and math.isfinite(v) for v in
                           (grid.get('resolution'),origin.get('x'),origin.get('y'))) or grid['resolution']<=0:
                    raise ValueError('invalid_saved_map_geometry')
                self.restored_snapshot=saved
                self.persistence={'restored':True,'native_cells':False,'legacy_PNG_only':True}
            self.map_epoch=epoch;self.version=count
            self.active=self.tracking=False
            self.state='needs_reset';self.reason='saved_map_requires_new_localization'
            self._publish()
        except (OSError,ValueError,TypeError,KeyError,EOFError,zlib.error) as error:
            # A corrupt file cannot partly install cells or create an authority.
            self.map=OccupancyMap(max_beams=450,laser_x=0.,laser_y=0.)
            self.restored_snapshot=None;self.version=0
            self.state='idle';self.reason=None
            self.persistence={'restored':False,'error':str(error)[:160]}
            self._publish()

    def status(self, since=None):
        value=self.snapshot
        if str(since)==str(value['version']) and value.get('grid'):
            return {**value,'grid':{k:v for k,v in value['grid'].items() if k!='image_base64'}}
        return value

    def action(self, operation):
        with self.lock:
            if operation=='reset':
                self.map=OccupancyMap(max_beams=450,laser_x=0.,laser_y=0.)
                self.pose=np.zeros(3);self.previous=None;self.last_source_stamp=None
                self.map_epoch=uuid.uuid4().hex;self.version=0;self.reason=None
                self.active=self.tracking=False;self.state='idle'
                self.restored_snapshot=None;self.persistence={}
            elif operation=='start':
                if self.state=='needs_reset':raise ValueError('새 지도를 눌러 위치 추정을 다시 시작하세요')
                self.active=self.tracking=True;self.state='waiting_for_sensors'
            elif operation=='stop':
                self.active=False
                if self.state!='needs_reset':self.state='paused'
            else:raise ValueError('unknown_map_operation')
            self._publish()
            if operation in ('stop','reset'):
                if self.restored_snapshot is None:
                    try:
                        proof=self.map.save_checkpoint(self.state_dir/'last_native_map.json')
                        self.persistence={'saved':True,'native_cells':True,**proof}
                    except (OSError,ValueError,TypeError) as error:
                        self.persistence={'saved':False,'native_cells':False,'error':str(error)[:160]}
                    self._publish()
                atomic_json(self.state_dir/'last_map.json',self.snapshot)
            return self.status()

    def close(self):
        self.stop_event.set()
        for thread in self.threads:thread.join(timeout=2)
        if self.version:
            self.action('stop')
