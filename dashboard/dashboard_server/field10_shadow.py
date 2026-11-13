"""C0 원 scan/IMU→최신 field10 ExecutorLoop의 관측용 연결. 출력 권한은 없다."""
import hashlib,json,math
from types import SimpleNamespace
def require(value,reason):
    if not value:raise ValueError(reason)
def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=True).encode()).hexdigest()
def message_header(ns,frame):
    require(type(ns)is int and ns>=0 and type(frame)is str,'original_header_required')
    return SimpleNamespace(stamp=SimpleNamespace(sec=ns//1000000000,nanosec=ns%1000000000),frame_id=frame)
class NoPayloadLog:
    def __init__(self):self.rows=0;self.dropped=0;self.ticks_suppressed=0
    def put(self,_record,tick=False):self.rows=min(9223372036854775807,self.rows+1)
    def health(self):return dict(state='ok',dropped=0,write_errors=0,last_error=None,failing_since=None,recoveries=0,
        pending=0,lines=self.rows,bytes=0,ticks_suppressed=0,writer_alive=True,file=None)
    def close(self,timeout=0.):pass
class Field10ShadowPlanner:
    """Each observation owner lazily creates one NoMotor loop, in its RAW clock.

    No C0 fitted motion/PWM/truth is injected as localization. The real raw
    inputs reach the unchanged row estimator; a non-test native gate remains
    WAIT until separately qualified map/pose/cost are supplied.
    """
    def __init__(self):
        self.context=None;self.loop=None;self._now=0.;self._stopped=False;self._scan_key=None;self._imu_key=None
        self._fingerprint=None;self.input_scans=0;self.input_imus=0;self._start=None
    def __call__(self,row,cruise_percent=0.):return self.plan(row,cruise_percent=cruise_percent)
    def reset(self,context):
        self.stop('new_observation_owner');self.context=dict(context);self.loop=None;self._stopped=False
        self._now=0.;self._scan_key=self._imu_key=self._fingerprint=None;self.input_scans=self.input_imus=0
    def _create(self,scan):
        from ._field10.row_executor import config as C,executor_main as E,navigation_observation as N
        cfg=C.ExecutorConfig()
        require(cfg.motor_output_enabled is False and cfg.calib_motor_output_enabled is False and cfg.require_native_navigation is True,
            'field10_native_required_motor_OFF_config')
        context_hash=digest(self.context)
        binding=N.NativeSourceBinding(context_hash,digest({'epoch':self.context.get('epoch'),'generation':self.context.get('generation')}),
            digest({'field10_config':cfg.digest(),'mount_qualified':False}),'c0_laser_diagnostic')
        gate=N.NativeNavigationGate(binding,min(cfg.lidar_max_age_s,.25),test_only=False)
        def forbidden(*_a,**_k):raise AssertionError('SHADOW_MUST_NOT_CREATE_BRIDGE')
        self.loop=E.ExecutorLoop(cfg,motor_output_flag=False,shadow_auto=False,bridge_factory=forbidden,
            log=NoPayloadLog(),clock=lambda:self._now,wall=lambda:0.,navigation_gate=gate)
        require(self.loop.motor_allowed is False and self.loop.bridge is None and self.loop._stop_transport is None,'NoMotor_loop_required')
        self._start=self.loop.submit('start',row_len_m=cfg.max_row_len_m)
        self._nav=N;self._gate=gate
    def reanchor_after_expiry(self,context):
        require(context==self.context,'same_C0_context_required')
        if self._stopped:return False
        if self.loop is not None:
            self.loop.hold_release();self.loop.runner.pause(self._now,'C0_input_expired')
        return True
    def plan(self,row,*,cruise_percent=0.):
        from .autonomy_observation import zero_decision
        if self._stopped:return zero_decision('FIELD10_OBSERVATION_STOPPED','STOP')
        require(type(row)is dict and row.get('context')==self.context,'same_owned_C0_input_context')
        require(str(self.context.get('receipt_clock_domain','')).startswith('CLOCK_MONOTONIC_RAW:'),'C0_RAW_receipt_clock_required')
        scan,imu=row['scan'],row['imu']
        require(scan.get('original_native')is True and imu.get('original_native')is True,'original_C0_scan_IMU_required')
        require(type(row['receipt_now_ns'])is int and row['receipt_now_ns']>=0,'original_C0_comparison_RAW_required')
        now=row['receipt_now_ns']*1e-9;require(now>=self._now,'C0_RAW_clock_rewind');self._now=now
        gyro,acc=imu.get('angular_velocity_xyz'),imu.get('linear_acceleration_xyz')
        if not(all(type(v)in(list,tuple)and len(v)==3 and all(type(x)in(int,float)and math.isfinite(x)for x in v)for v in(gyro,acc))):
            return zero_decision('FIELD10_RAW_IMU_VECTOR_UNAVAILABLE')
        require(imu['stamp_ns']<=scan['stamp_ns'],'original_causal_IMU_required')
        require(type(scan['ranges'])is list and 0<len(scan['ranges'])<=4096,'bounded_original_scan_required')
        if self.loop is None:self._create(scan)
        key=(scan['sequence'],scan['stamp_ns'],scan['receipt_ns']);fingerprint=digest({'scan':scan,'imu':imu})
        if key==self._scan_key:
            require(fingerprint==self._fingerprint,'same_C0_scan_changed')
        else:
            if self._scan_key is not None:require(all(a>b for a,b in zip(key,self._scan_key)),'C0_scan_rewind')
            imukey=(imu['stamp_ns'],imu['receipt_ns'])
            if imukey!=self._imu_key:
                if self._imu_key is not None:require(all(a>b for a,b in zip(imukey,self._imu_key)),'C0_IMU_rewind')
                g=SimpleNamespace(x=gyro[0],y=gyro[1],z=gyro[2]);a=SimpleNamespace(x=acc[0],y=acc[1],z=acc[2])
                msg=SimpleNamespace(header=message_header(imu['stamp_ns'],imu['frame_id']),angular_velocity=g,linear_acceleration=a)
                self.loop.on_imu_msg(imu['receipt_ns']*1e-9,msg);self._imu_key=imukey;self.input_imus+=1
            # None retains its original unknown ray slot; never a finite/free hit.
            msg=SimpleNamespace(header=message_header(scan['stamp_ns'],scan['frame_id']),
                ranges=[float('nan')if x is None else x for x in scan['ranges']],angle_min=scan['angle_min'],
                angle_increment=scan['angle_increment'],range_min=scan['range_min'],range_max=scan['range_max'])
            self.loop.on_scan_msg(scan['receipt_ns']*1e-9,msg);self.input_scans+=1
            age=(row['source_now_ns']-scan['stamp_ns'])*1e-9
            require(0<=age<=30,'original_C0_source_age')
            observation=self._nav.NativeNavigationObservation(self._gate.expected,scan['sequence'],age,'WAIT',None,False,0,'UNKNOWN',0)
            self._gate.offer(observation,now)
            self._scan_key,self._fingerprint=key,fingerprint
        self.loop.step(now)
        require(self.loop.motor_allowed is False and self.loop.bridge is None and self.loop._stop_transport is None,'SHADOW_output_inhibited')
        nav=dict(self.loop._native_navigation)
        # Native control admission is not qualified by C0 raw delivery alone.
        result=zero_decision('FIELD10_'+str(nav.get('reason','NOT_EVALUATED')))
        result['field10']=dict(planner='latest_field10_ExecutorLoop',input_provider='production_C0_original_commit',
            scans_fed=self.input_scans,imus_fed=self.input_imus,scans_processed=self.loop.counts['scans_processed'],
            imu_fed=self.loop.counts['imu_fed'],native_navigation=nav,bridge_created=False,stop_transport_created=False,
            motor_allowed=False,cruise_preference_applied=False,source_motion_used_as_localization=False,
            controller_state=self.loop.runner.state)
        return result
    def stop(self,reason='observation_stop'):
        self._stopped=True
        if self.loop is not None:self.loop.emergency_stop(reason)
    def manual_takeover(self):self.stop('manual_takeover')
def make_field10_shadow_planner():return Field10ShadowPlanner()
