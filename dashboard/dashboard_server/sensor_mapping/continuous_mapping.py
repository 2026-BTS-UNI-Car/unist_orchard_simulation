"""Live adapter for the same causal, fixed-frame localizer used by replay.

Only source-timestamped LaserScan, native IMU and quality-gated GNSS enter.
An unresolved pose keeps the previous raster; a motion prior never casts rays.
"""
import json
import math
import threading
import time
from collections import deque
import numpy as np
from .cursor import SourceCursorLocalizer,integrate
from .raw_mapping.raw_localization import source_stamp
from .tile_transport import StreamingOccupancyMap


class ContinuousMapping:
    def __init__(self,max_records=36000):
        self.lock=threading.RLock();self.source=SourceCursorLocalizer()
        self.localizer=self.source.engine;self.gnss=self.source.gnss
        self.map=StreamingOccupancyMap(max_beams=450,laser_x=0.,laser_y=0.,
                                     conservative_free=True,endpoint_margin_cells=1)
        self.records=deque(maxlen=max_records);self.old_segments=[];self.segment=0;self.generation=0
        self.frame='continuous_sensor_local_frame';self.diagnostic={}
        self.max_records=max_records;self.closed=False;self.background_gnss=False
        self.pending={};self.last_imu=-float('inf');self.last_gps=-float('inf')

    def imu(self,message):
        stamp=source_stamp(message)
        if not math.isfinite(stamp):return
        with self.lock:
            if stamp>self.last_imu:
                self.pending[('/imu/data_raw',stamp)]=message
                self._bound_pending()

    def gps(self,sample):
        try:stamp=sample['stamp']['sec']+sample['stamp']['nanosec']*1e-9
        except (KeyError,TypeError,ValueError):return
        if not math.isfinite(stamp):return
        try:message={'data':json.dumps(sample,allow_nan=False)}
        except (ValueError,TypeError):return
        with self.lock:
            if stamp>self.last_gps:
                self.pending[('/gps/data',stamp)]=message
                self._bound_pending()

    def _bound_pending(self):
        # A disconnected/paused scan source must not create unbounded history.
        if len(self.pending)>256:
            for key in sorted(self.pending,key=lambda k:k[1])[:-256]:self.pending.pop(key)

    def add(self,scan,imu,draw=True,t_s=None,on_localized=None):
        started=time.perf_counter();stamp=source_stamp(scan)
        with self.lock:
            if self.closed:raise ValueError('mapping_engine_closed')
            if imu is not None:self.imu(imu)
            keys=sorted((k for k in self.pending if k[1]<=stamp),key=lambda k:(k[1],k[0]))
            batch=[]
            for topic,t in keys:
                batch.append(dict(topic=topic,message=self.pending.pop((topic,t))))
                if topic=='/imu/data_raw':self.last_imu=max(self.last_imu,t)
                else:self.last_gps=max(self.last_gps,t)
            batch.append(dict(topic='/scan',message=scan))
            result,counters,note=self.source.consume(batch)
            # Pose diagnostics are bounded without stopping a long live run.
            # Actual numerical keyframes are retained by the localizer.
            if len(self.source.pairs)>4096:self.source.pairs=self.source.pairs[-4096:]
            if len(self.gnss.accepted)>4096:
                self.gnss.accepted=self.gnss.accepted[-4096:]
                self.gnss.seen={factor.receiver_epoch for factor in self.gnss.accepted}
            localized_at=time.perf_counter()
            # Submit the original source/result before display raster work.
            # The callback may only enqueue bounded work; rejected/prior poses
            # must reach the observer too so it cannot retain old authority.
            if on_localized is not None:on_localized(result)
            raster_started=time.perf_counter()
            if result['accepted']:
                self.records.append(dict(stamp=stamp,pose=np.array(result['pose'],copy=True)))
                if draw and result.get('map_eligible',True):
                    integrate(self.map,scan,result['pose'],t_s)
                    if self.map.path and result.get('path_break'):self.map.path[-1]['break_before']=True
            self.diagnostic=dict(reason=result['reason'],accepted=result['accepted'],
                total_ms=(time.perf_counter()-started)*1000,
                localization_ms=(localized_at-started)*1000,
                observer_submit_ms=(raster_started-localized_at)*1000,
                raster_ms=(time.perf_counter()-raster_started)*1000,**counters)
            return result

    def info(self):
        from .native_raster import backend_status
        return dict(segment=0,segments=int(bool(self.records)),coordinate_frame=self.frame,
            raster_backend=backend_status(),
            localization='causal_fixed_frame_LiDAR_native_IMU_quality_GNSS',
            pose_source='encoderless_scan_registration',source='original_LaserScan_and_raw_IMU',
            runtime_authority=False,autonomy_eligible=False,pose_accuracy_validated=False,
            mount_height_m=.60,mount_rotation_calibrated=False,deskew_applied=False,
            gnss=dict(self.source.note),diagnostic=dict(self.diagnostic),no_encoder=True,
            PWM_used_as_odometry=False,camera_used_for_mapping=False,
            note='GPS·IMU·LiDAR 통합 · 같은 좌표계 유지 · 정합 불확실 시 이전 지도 보존')
