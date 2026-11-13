"""Single live sensor mapper: bounded inboxes and immutable HTTP generations.

Stop disconnects private raw subscriptions and stops registration. Existing
hardware owners/recorder continue. Resume needs observed re-localization before
inserting unknown travel; stopped-time motion is never invented.
"""
import asyncio
from collections import deque
import copy
import json
import math
import os
import threading
import time
import uuid
import bisect
import numpy as np
from .body_feedback import BodyFeedbackObserver, FeedbackContext, LaserPosePair, NativeImuYaw
from .live_mapping import LiveMapping
from .continuous_mapping import ContinuousMapping
from .tile_transport import StreamingOccupancyMap
from .occupancy_map import atomic_json


class SourceLiveMapping(LiveMapping):
    def __init__(self,state_dir,start_worker=True):
        self.engine=None;self.cell_snapshot=None
        self.body_feedback=BodyFeedbackObserver()
        self._feedback_previous=None
        self._feedback_sequence=0
        self._feedback_session='web_ws_subscriptions:'+uuid.uuid4().hex
        self._feedback_adapter={}
        super().__init__(state_dir,start_worker=False)
        self.engine=ContinuousMapping()
        if not self.version:self.map=self.engine.map
        elif self.restored_snapshot is None:
            self.map=StreamingOccupancyMap.load_checkpoint(self.state_dir/'last_native_map.json')
            for key,tile in self.map._tiles.items():
                states=self.map.cell_states(tile);self.map.tile_classes[key]=states
                self.map.tile_versions[key]=self.map.scan_count
                ys,xs=np.nonzero(states==2)
                if len(xs):self.map.object_bounds[key]=(key[0]*64+int(xs.min()),key[1]*64+int(ys.min()),
                    key[0]*64+int(xs.max())+1,key[1]*64+int(ys.max())+1)
        self.pending_scans=deque(maxlen=8);self.imu_history=deque(maxlen=100)
        self.wakeup=threading.Event();self.processing_lock=threading.RLock()
        self.command_lock=threading.RLock();self.work_generation=0
        self.dropped_source_scans=0;self.source_times={};self.connections={}
        self.last_progress=None;self.last_attempt=None;self.processing=False
        self.received_counts={'scan':0,'imu':0,'gps':0}
        self._publish()
        if start_worker:
            sources=[('scan',os.environ.get('LIDAR_WS_UPSTREAM','ws://127.0.0.1:8765/ws/lidar')+'?profile=mapping'),
                     ('imu',os.environ.get('IMU_WS_UPSTREAM','ws://127.0.0.1:8775/ws/imu')+'?profile=mapping'),
                     ('gps',os.environ.get('GPS_WS_UPSTREAM','ws://127.0.0.1:8778/ws/gps')+'?profile=detail')]
            for kind,url in sources:
                thread=threading.Thread(target=lambda k=kind,u=url:asyncio.run(self._receive(k,u)),daemon=True,name='map-source-'+kind)
                thread.start();self.threads.append(thread)
            thread=threading.Thread(target=self._run,daemon=True,name='source-rate-integrated-map')
            thread.start();self.threads.append(thread)

    async def _receive(self,kind,url):
        import websockets
        last_stamp=None
        while not self.stop_event.is_set():
            if not self.tracking:
                self.connections[kind]='idle';await asyncio.sleep(.2);continue
            try:
                async with websockets.connect(url,max_queue=2,max_size=2**20,open_timeout=3,close_timeout=1) as socket:
                    self.connections[kind]='connected'
                    if kind in ('scan','imu'):
                        with self.processing_lock,self.lock:
                            self._invalidate_feedback('sensor_subscription_connected',new_session=True)
                    while self.tracking and not self.stop_event.is_set():
                        try:
                            wire=await asyncio.wait_for(socket.recv(),.25)
                            receipt_ns=time.monotonic_ns()
                            packet=json.loads(wire)
                        except asyncio.TimeoutError:continue
                        if kind=='gps' and packet.get('stream_seq'):
                            await socket.send(json.dumps({'ack':packet['stream_seq']}))
                        if packet.get('type')!=kind:continue
                        if kind in ('scan','imu'):
                            packet={**packet,'_mapping_ws_receipt_ns':receipt_ns}
                        row=(packet,receipt_ns*1e-9)
                        with self.inbox_lock:
                            if not self.tracking:continue
                            self.latest[kind]=row;self.source_times[kind]=row[1];self.received_counts[kind]+=1
                            if kind=='imu':self.imu_history.append(row)
                            if kind=='scan' and packet.get('stamp')!=last_stamp:
                                if len(self.pending_scans)==self.pending_scans.maxlen:self.dropped_source_scans+=1
                                self.pending_scans.append(row);last_stamp=packet.get('stamp')
                        self.wakeup.set()
            except (OSError,ValueError,websockets.exceptions.WebSocketException):
                if kind in ('scan','imu'):
                    with self.processing_lock,self.lock:
                        self._invalidate_feedback('sensor_subscription_disconnected',new_session=True)
                self.connections[kind]='reconnecting';await asyncio.sleep(.5)

    def _run(self):
        while not self.stop_event.is_set():
            self.wakeup.wait(.1 if self.tracking else .5);self.wakeup.clear()
            if not self.tracking:continue
            with self.processing_lock,self.lock:
                if not self.tracking:continue
                source_identity=(self.engine,self.work_generation,self._feedback_session)
                with self.inbox_lock:
                    if not self.pending_scans:continue
                    scan=self.pending_scans[0];history=list(self.imu_history);gps=self.latest.get('gps')
            def delta(row):
                try:
                    stamp=row[0]['raw']['stamp'];return abs(stamp['sec']+stamp['nanosec']*1e-9-scan[0]['stamp'])
                except (KeyError,TypeError):return math.inf
            imu=min(history,key=delta) if history else None
            if (imu is None or delta(imu)>.1) and time.monotonic()-scan[1]<.15:
                self.wakeup.set();time.sleep(.015);continue
            with self.inbox_lock:
                if self.pending_scans and self.pending_scans[0] is scan:self.pending_scans.popleft()
            try:
                with self.processing_lock:
                    # Stop/reset may have happened after dequeue. Both GPS and
                    # scan admission share the same original worker identity.
                    with self.lock:
                        if source_identity!=(self.engine,self.work_generation,self._feedback_session):continue
                        if not self.tracking:continue
                    if gps and time.monotonic()-gps[1]<1:self.engine.gps(gps[0])
                    self.ingest(scan,imu,source_identity=source_identity)
            except (KeyError,ValueError,TypeError,OverflowError) as error:
                with self.processing_lock,self.lock:
                    if source_identity!=(self.engine,self.work_generation,self._feedback_session):continue
                    self._invalidate_feedback('source_input_or_localizer_error')
                    if self.tracking:self.state='waiting_for_sensors';self.reason=str(error);self._publish()
            except Exception as error:
                # A bad observation cannot silently kill the single worker.
                with self.processing_lock,self.lock:
                    if source_identity!=(self.engine,self.work_generation,self._feedback_session):continue
                    self._invalidate_feedback('source_input_or_localizer_error')
                    self.state='worker_error';self.reason=type(error).__name__+': '+str(error)[:160];self._publish()

    def ingest(self,scan,imu,now=None,*,source_identity=None):
        with self.processing_lock:
            with self.lock:
                identity=(self.engine,self.work_generation,self._feedback_session)
                if source_identity is not None and source_identity!=identity:return False
            try:return self._ingest(scan,imu,now)
            except Exception:
                # Keep admission and exception handling within one processing
                # fence. An old observation cannot erase a newer generation.
                with self.lock:
                    if identity==(self.engine,self.work_generation,self._feedback_session):
                        self._invalidate_feedback('source_input_or_localizer_error')
                raise

    def _ingest(self,scan,imu,now=None):
        now=time.monotonic() if now is None else now
        frame,received=scan;raw_scan=frame.get('raw_scan')
        if not raw_scan:raise ValueError('waiting_for_original_LaserScan_bridge')
        if not 0<=now-received<=1.5:raise ValueError('stale_original_LaserScan')
        raw=self.native_imu(imu[0]) if imu else None
        with self.inbox_lock:history=list(self.imu_history)
        # HTTP reads never wait for registration. Stop/reset waits for the
        # current scan so no rays arrive after its acknowledged response.
        with self.processing_lock:
            with self.lock:
                if not self.tracking:return False
                engine=self.engine;generation=self.work_generation;draw=self.active
                self.processing=True;self.last_attempt=time.monotonic()
            try:
                for packet,_ in history:
                    native=self.native_imu(packet)
                    if native:engine.imu(native)
                result=engine.add(raw_scan,raw,draw=draw)
                with self.lock:
                    if self.engine is not engine or generation!=self.work_generation:return False
                    self.map=engine.map;self.pose=engine.localizer.pose.copy();self.version+=1
                    eligible=result['accepted'] and result.get('map_eligible',True)
                    self.state=('drawing' if draw else 'paused') if eligible else 'waiting_for_geometry'
                    self.reason=None if eligible else result['reason']
                    if eligible:self.last_progress=time.monotonic()
                    try:self._observe_feedback(frame,received,result,engine,history)
                    except Exception as error:
                        # Diagnostic failure inhibits feedback, never interrupts
                        # an already localized/rasterized map or renews old data.
                        self._invalidate_feedback('feedback_observer_error')
                        self._feedback_adapter['error_class']=type(error).__name__
                    self.diagnostic={**engine.diagnostic,'source_queue_dropped':self.dropped_source_scans,
                        'receipt_to_map_ms':round((time.monotonic()-received)*1000,2)}
                    self._publish()
                return result['accepted']
            finally:self.processing=False

    def _invalidate_feedback(self,reason,new_session=False):
        # Caller holds the same processing fence as pose publication. This is
        # a WEB subscription session, never a DDS publisher or native RAW epoch.
        self._feedback_previous=None
        self.body_feedback.invalidate(reason)
        if new_session:
            self._feedback_session='web_ws_subscriptions:'+uuid.uuid4().hex
            self._feedback_sequence=0

    @staticmethod
    def _original_ns(message):
        try:
            stamp=message['header']['stamp'];sec=stamp['sec'];ns=stamp['nanosec']
            if type(sec) is not int or type(ns) is not int or sec<0 or not 0<=ns<1_000_000_000:
                return None
            value=sec*1_000_000_000+ns
            return value if value<=2**63-1 else None
        except (KeyError,TypeError):return None

    @staticmethod
    def _web_receipt_ns(packet,received):
        value=packet.get('_mapping_ws_receipt_ns')
        if type(value) is int and 0<=value<=2**63-1:return value
        # Direct/offline callers retain their original WEB tuple clock. This
        # conversion is explicitly not a native DDS callback timestamp.
        if type(received) in (int,float) and math.isfinite(received) and 0<=received<2**63/1e9:
            return int(received*1e9)
        return None

    def _cursor_imu_diagnostic(self,engine,scan_stamp_ns,context,history):
        cursor=engine.source
        # Reproduce source_stamp(scan)'s actual cursor key operation. Multiplying
        # the combined epoch nanoseconds rounds differently at modern epochs.
        scan_cursor_stamp=scan_stamp_ns//1_000_000_000+(scan_stamp_ns%1_000_000_000)*1e-9
        index=bisect.bisect_right(cursor.stamps,scan_cursor_stamp)-1
        if index<0:return None
        selected=cursor.imus[index]
        selected_ns=self._original_ns(selected)
        if selected_ns is None or selected_ns>scan_stamp_ns:return None
        # Join the actual cursor-selected original, not the front-end nearest
        # IMU (which may be in the future). Missing original frame stays missing.
        originals=[]
        for packet,received in history:
            native=self.native_imu(packet)
            if native is None or self._original_ns(native)!=selected_ns:continue
            # A timestamp is not payload identity. Conflicting originals or
            # duplicate receipts cannot prove which observation was selected.
            if native!=selected:return None
            originals.append((packet,received))
        if len(originals)!=1:return None
        for packet,received in originals:
            raw=packet.get('raw',{})
            native=self.native_imu(packet)
            if native is None or self._original_ns(native)!=selected_ns:continue
            frame=raw.get('frame_id')
            if type(frame) is not str or not frame.strip():return None
            receipt_ns=self._web_receipt_ns(packet,received)
            if receipt_ns is None:return None
            try:
                availability=raw.get('availability',{})
                if any((availability.get(k) or {}).get('available') is False for k in ('orientation','angular_velocity')):
                    return None
                covariance=raw.get('covariance',{})
                if any((covariance.get(k) or [0])[0]<0 for k in ('orientation','angular_velocity')):
                    return None
                q=[raw['orientation'][k] for k in 'xyzw']
                gyro=[raw['angular_velocity'][k] for k in 'xyz']
                if any(type(v) not in (int,float) or not math.isfinite(v) for v in q+gyro):return None
                norm=math.sqrt(sum(v*v for v in q))
                if not .9<norm<1.1:return None
                x,y,z,w=(v/norm for v in q)
                rate=2*(x*z-w*y)*gyro[0]+2*(y*z+w*x)*gyro[1]+(1-2*(x*x+y*y))*gyro[2]
                if not math.isfinite(rate):return None
            except (KeyError,TypeError,ValueError,OverflowError):return None
            return NativeImuYaw(context,selected_ns,receipt_ns,rate,
                native=False,gravity_projected=False,mount_verified=False,imu_frame=frame)
        return None

    def _observe_feedback(self,frame,received,result,engine,history):
        self._feedback_sequence+=1
        self._feedback_adapter=dict(mode='web_ws_sensor_plane_diagnostic',
            native_CDR_GID_RAW_context_available=False,source_clock_verified=False,
            receiver_session_is_publisher_epoch=False,body_control_eligible=False)
        if (result.get('accepted') is not True or result.get('map_eligible') is not True
                or result.get('motion_prior') is not False or result.get('path_break') is not False):
            self._invalidate_feedback('registration_rejected_prior_or_relocalized');return
        raw_scan=frame.get('raw_scan',{})
        stamp_ns=self._original_ns(raw_scan);receipt_ns=self._web_receipt_ns(frame,received)
        laser_frame=raw_scan.get('header',{}).get('frame_id')
        if stamp_ns is None or receipt_ns is None or type(laser_frame) is not str or not laser_frame.strip():
            self._invalidate_feedback('original_scan_clock_or_frame_missing');return
        try:pose=tuple(float(v) for v in result['pose'])
        except (KeyError,TypeError,ValueError,OverflowError):
            self._invalidate_feedback('invalid_geometric_pose');return
        if len(pose)!=3 or not all(math.isfinite(v) for v in pose):
            self._invalidate_feedback('invalid_geometric_pose');return
        context=FeedbackContext(self._feedback_session,self.map_epoch,self.work_generation,
            'ros_header_unverified','python_monotonic_web_ws',engine.frame,laser_frame)
        current=(self._feedback_sequence,stamp_ns,receipt_ns,pose,context)
        previous=self._feedback_previous
        if previous is None:
            self.body_feedback.invalidate('first_geometric_pose_priming')
            self._feedback_previous=current;return
        if (context!=previous[4] or stamp_ns<=previous[1] or receipt_ns<=previous[2]
                or stamp_ns-previous[1]>250_000_000 or abs(pose[2]-previous[3][2])>=math.pi):
            self._invalidate_feedback('sensor_pose_pair_discontinuous')
            self._feedback_previous=current;return
        pair=LaserPosePair(context,previous[0],current[0],previous[1],stamp_ns,
            previous[2],receipt_ns,previous[3],pose,accepted=True,map_eligible=True,
            independent_localization_validated=False,planar_supported=False,
            source_clock_verified=False,path_break=False,motion_prior=False)
        imu=self._cursor_imu_diagnostic(engine,stamp_ns,context,history)
        self.body_feedback.observe(pair,imu,source_now_ns=time.time_ns(),receipt_now_ns=time.monotonic_ns())
        self._feedback_adapter.update(selected_imu_stamp_ns=None if imu is None else imu.source_stamp_ns,
            selected_imu_receipt_ns=None if imu is None else imu.receipt_ns,
            selected_imu_frame=None if imu is None else imu.imu_frame,
            candidate_vertical_gyro_rad_s=None if imu is None else imu.yaw_rad_s)
        self._feedback_previous=current

    def feedback_status(self):
        with self.lock:
            return {**self.body_feedback.snapshot(),'adapter':dict(self._feedback_adapter)}

    def observation_input(self):
        """Read the existing WS source without starting/resetting map work.

        This fallback cannot prove original DDS/CDR/RAW provenance and therefore
        cannot produce a native-qualified autonomous proposal. Comparison now
        is separate from the unchanged original header and WS receipt clocks.
        """
        with self.lock:
            context=dict(source_session=self._feedback_session, epoch=self.map_epoch,
                generation=self.work_generation, source_clock_domain='ros_header_unverified',
                receipt_clock_domain='python_monotonic_web_ws')
            feedback=self.body_feedback.snapshot()
            adapter=dict(self._feedback_adapter)
            with self.inbox_lock:
                latest=self.latest.get('scan')
            if not self.tracking or latest is None:return None
            frame,received=latest
            raw=frame.get('raw_scan')
            if not raw:return None
            stamp=self._original_ns(raw);receipt=self._web_receipt_ns(frame,received)
            if stamp is None or receipt is None:return None
            try:
                scan=dict(stamp_ns=stamp,receipt_ns=receipt,sequence=self.received_counts['scan'],
                    frame_id=raw['header']['frame_id'],ranges=copy.deepcopy(raw['ranges']),
                    **{key:raw[key] for key in ('angle_min','angle_increment','range_min','range_max')},
                    original_native=False)
            except (KeyError,TypeError):return None
            laser=feedback.get('laser')
            motion=imu=None
            if (laser and feedback.get('source_stamp_ns')==stamp and
                    laser.get('registration_accepted') is True and laser.get('map_eligible') is True and
                    self._feedback_previous is not None and self._feedback_previous[1]==stamp):
                motion=dict(previous_scan_stamp_ns=laser['previous_source_stamp_ns'],
                    current_scan_stamp_ns=stamp,receipt_ns=feedback['receipt_ns'],
                    dx_m=laser['delta_xy_m'][0],dy_m=laser['delta_xy_m'][1],dyaw_rad=laser['delta_yaw_rad'],
                    accepted=True,map_eligible=True,path_break=False,motion_prior=False,
                    body_mount_verified=False,accuracy_verified=False)
                imu_stamp=adapter.get('selected_imu_stamp_ns')
                imu_receipt=adapter.get('selected_imu_receipt_ns')
                rate=adapter.get('candidate_vertical_gyro_rad_s')
                if imu_stamp is not None and imu_receipt is not None and rate is not None:
                    imu=dict(stamp_ns=imu_stamp,receipt_ns=imu_receipt,yaw_rate_rad_s=rate,original_native=False)
            return dict(context=context,scan=scan,motion=motion,imu=imu,
                source_now_ns=time.time_ns(),receipt_now_ns=time.monotonic_ns(),
                runtime_authority=False,motor_command_sent=False,physical_autonomy_qualified=False)

    @staticmethod
    def native_imu(packet):
        raw=packet.get('raw',{})
        if not raw or (raw.get('availability',{}).get('linear_acceleration') or {}).get('available') is False:return None
        return {**raw,'header':dict(stamp=raw['stamp'],frame_id=raw.get('frame_id','imu_link')),
            **{name+'_covariance':raw.get('covariance',{}).get(name,[0]*9)
               for name in ('orientation','angular_velocity','linear_acceleration')}}

    def _publish(self):
        if self.engine is None:return super()._publish()
        if self.restored_snapshot is not None:super()._publish();return
        info=self.engine.info()
        if self.state=='needs_reset':info.update(source='restored_native_cells_new_localization_required')
        self.snapshot=dict(ok=True,simulated=False,active=self.active,state=self.state,reason=self.reason,
            version=self.version,map_epoch=self.map_epoch,tile_epoch=f'{self.map_epoch}:0:0',view_epoch=f'{self.map_epoch}:0',
            scan_count=self.map.scan_count,**info)
        self.snapshot['diagnostic']=dict(self.diagnostic)
        self.snapshot['body_feedback']=self.feedback_status()
        if self.persistence:
            self.snapshot['persistence']=dict(self.persistence)
            if self.persistence.get('native_cells'):self.snapshot['native_checkpoint']='last_native_map.json'
        packet=self.map.tile_packet()
        packet['tiles']=[dict(t,revision=self.map.tile_versions.get((t['x'],t['y']),0)) for t in packet['tiles']]
        # The token changes whenever previously published path points can no
        # longer be addressed by index (new map epoch or 12k-point compaction).
        path=packet.get('path') or []
        self.cell_snapshot={**packet,**self.snapshot,
            'path_token':f"{self.map_epoch}:{int(getattr(self.map,'path_compacted_at',0) or 0)}",
            'path_count':len(path)}

    def task_status(self):
        now=time.monotonic()
        with self.inbox_lock:
            times=dict(self.source_times);depth=len(self.pending_scans);counts=dict(self.received_counts)
        return dict(id='live',kind='live',priority=0,epoch=self.map_epoch,state=self.snapshot.get('state'),
            active=self.active,tracking=self.tracking,processing=self.processing,
            stage='registering_scan' if self.processing else 'waiting_sensors' if self.tracking and not depth else 'idle' if not self.tracking else 'queued_scan',
            reason=self.snapshot.get('reason'),scan_count=self.map.scan_count,input_scans=self.engine.source.input_scans,
            queue_scans=depth,dropped_scans=self.dropped_source_scans,
            last_progress_age_s=None if self.last_progress is None else now-self.last_progress,
            last_attempt_age_s=None if self.last_attempt is None else now-self.last_attempt,
            source_age_s={k:max(0.,now-v) for k,v in times.items()},received=counts,
            connections=dict(self.connections),diagnostic=dict(self.diagnostic),
            body_feedback=self.feedback_status(),runtime_authority=False)

    # Clients that send path_token/path_count/path_stamp receive only the path
    # points after their last known point (path_from = its index), instead of
    # the whole retained path (up to 12,000 poses) on every 100 ms poll.
    supports_path_delta=True

    @staticmethod
    def path_delta(data,path_token,path_count,path_stamp):
        """Return (path_from, points) or None when the client must take the full path.

        The last point the client already holds is sent again so that a flag
        set on it after publication (break_before) is refreshed."""
        path=data.get('path') or []
        try:count=int(path_count);stamp=float(path_stamp)
        except (TypeError,ValueError):return None
        if (path_token is None or path_token!=data.get('path_token') or not 1<=count<=len(path)
                or not math.isfinite(stamp) or path[count-1].get('stamp')!=stamp):return None
        return count-1,path[count-1:]

    def status(self,since=None,transport=None,tile_since=None,tile_epoch=None,segment=None,
               path_token=None,path_count=None,path_stamp=None):
        data=self.cell_snapshot
        if self.restored_snapshot is not None:
            return {**super().status(since),'body_feedback':self.feedback_status()}
        if data is None:return dict(self.snapshot)
        try:revision=int(tile_since) if tile_epoch==data['tile_epoch'] else -1
        except (ValueError,TypeError):revision=-1
        full=not 0<=revision<=data['tile_version']
        result={**data,'tiles':data['tiles'] if full else [t for t in data['tiles'] if t['revision']>revision],
                'tiles_reset':full,'worker':self.task_status()}
        if not full and path_token is not None:
            delta=self.path_delta(data,path_token,path_count,path_stamp)
            if delta is not None:result.update(path_from=delta[0],path=delta[1],path_append=False)
        if self.tracking:
            now=time.monotonic()
            if now-self.source_times.get('scan',0)>1.5:result.update(state='waiting_for_sensors',reason='LiDAR_source_stale')
            elif now-self.source_times.get('imu',0)>1.5:result.update(state='waiting_for_sensors',reason='IMU_source_stale')
        return result

    def action(self,operation,expected_epoch=None):
        with self.command_lock:
            if expected_epoch is not None and expected_epoch!=self.map_epoch:raise ValueError('mapping_epoch_changed_refresh_before_action')
            if operation=='start' and self.active:return self.status(transport='tiles')
            with self.processing_lock,self.lock:
                if operation=='reset':
                    self._invalidate_feedback('mapping_reset',new_session=True)
                    if self.map.scan_count:self.map.save_checkpoint(self.state_dir/f'previous-{self.map_epoch}.json')
                    self.engine.closed=True;self.engine=ContinuousMapping();self.map=self.engine.map
                    self.pose=np.zeros(3);self.map_epoch=uuid.uuid4().hex;self.work_generation+=1
                    self.version=0;self.reason=None;self.active=self.tracking=False;self.state='idle'
                    self.restored_snapshot=None;self.persistence={};self.last_progress=None;self.last_attempt=None
                elif operation=='start':
                    if self.state=='needs_reset':raise ValueError('새 지도를 눌러 위치 추정을 다시 시작하세요')
                    self._invalidate_feedback('mapping_started',new_session=True)
                    self.active=self.tracking=True;self.state='waiting_for_sensors';self.wakeup.set()
                elif operation=='stop':
                    self._invalidate_feedback('mapping_stopped',new_session=True)
                    self.active=self.tracking=False
                    if self.state!='needs_reset':self.state='paused'
                else:raise ValueError('unknown_map_operation')
                with self.inbox_lock:
                    if operation in ('stop','reset'):
                        self.pending_scans.clear();self.imu_history.clear();self.latest.clear()
                self._publish()
                if operation in ('stop','reset'):
                    if self.restored_snapshot is None:
                        try:self.persistence={'saved':True,'native_cells':True,**self.map.save_checkpoint(self.state_dir/'last_native_map.json')}
                        except (OSError,ValueError,TypeError) as error:self.persistence={'saved':False,'error':str(error)[:160]}
                        self._publish()
                    atomic_json(self.state_dir/'last_map.json',self.snapshot)
            return self.status(transport='tiles')
