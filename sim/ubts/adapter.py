"""Gazebo sensor contracts and manual control. No serial/hardware dependencies."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import socketserver
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from sensor_msgs.msg import Imu, LaserScan, JointState, NavSatFix
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import yaml
from scripts.rover_control_core import ControlError
from scripts.sim_camera_health import CameraHealthReader
from control import SimulationControl
from sim_telemetry import SensorTelemetry, GPS_HEIGHT_M
from scripts.gps_ros2_driver import make_navsat
from autonomy_control import AutonomousControl
from sim_telemetry import pose_of
from visualization import SensorVisualization
from planar_odometry import PlanarOdometry
from callback_timing import CallbackTiming
from snapshot_writer import LatestSnapshotWriter
from encoderless_runtime_contract import runtime_mode, strict_json, SOURCE as ENCODERLESS_SOURCE
from actuator_response import ActuatorResponse
from camera_profile import load_rover_config

ROOT=Path(__file__).resolve().parent

def save(name, payload):
    p=Path('/runtime')/name;tmp=p.with_suffix('.tmp')
    tmp.write_text(json.dumps(payload,allow_nan=False),encoding='utf-8');tmp.replace(p)

class SimMotor:
    firmware='SIMULATED_GAZEBO_SKID'
    error=None
    def __init__(self,node,config):
        self.node,self.config=node,config
        self.ready=False;self.target=[0.,0.];self.applied=[0.,0.];self.measured=[0.,0.]
        self.outputs={};self.last_sim=0.
        self.ever_enabled=False
        self.response=ActuatorResponse(config.get('actuator_response'),track_m=config.get('track_m',.56))
        self.effective=[0.,0.]
    @property
    def stopped(self):
        # In encoderless manual mode this means output ramp complete, not a
        # claim of measured wheel/body stationarity. Status names that limit.
        if getattr(self.node,'encoderless_mode',False):return max(map(abs,self.applied))<.001
        return max(map(abs,self.measured))<.02 and max(map(abs,self.applied))<.001
    def enable(self):self.ever_enabled=True
    def drive(self,left,right):
        self.target=[left/100*self.config['max_wheel_speed_m_s'],right/100*self.config['max_wheel_speed_m_s']]
    def release(self):self.target=[0.,0.]
    def hard_stop(self):
        self.target=[0.,0.];self.applied=[0.,0.];self.publish()
    def publish(self):
        msg=Twist();left,right=self.applied
        effective=self.response.wheel_targets(left,right,self.config['max_wheel_speed_m_s'],self.node.sim_time)
        self.effective=list(effective)
        msg.linear.x=sum(effective)/2;msg.angular.z=(effective[1]-effective[0])/self.config['track_m']
        self.node.command.publish(msg)
        self.outputs={pin:round(max(0,value)*255/self.config['max_wheel_speed_m_s']) for pin,value in zip(['D8','D5','D3','D0'],[left,-left,right,-right])}
    def step(self):
        elapsed=self.node.sim_time-self.last_sim
        if elapsed<0:self.last_sim=self.node.sim_time;return
        # Publish at most 50 Hz of simulation time, not once per ROS callback.
        # hard_stop() bypasses this cadence and publishes zero immediately.
        if elapsed<.0199:return
        dt=min(.05,elapsed);self.last_sim=self.node.sim_time
        ramped=self.response.ramp_step(self.applied,self.target,dt,self.config['max_wheel_speed_m_s'])
        if ramped is None:
            change=self.config['acceleration_m_s2']*dt
            for i in range(2): self.applied[i]+=max(-change,min(change,self.target[i]-self.applied[i]))
        else:self.applied=list(ramped)  # field profile: MEASURED MCU ramp + reversal break
        self.publish()

class Adapter(Node):
    @staticmethod
    def sensor_qos(topic):
        depth=PlanarOdometry.HISTORY_DEPTH if topic in ('/sim/imu','/sim/wheel_odom') else 1
        return QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=depth,reliability=ReliabilityPolicy.BEST_EFFORT)

    def create_publisher(self, *args, **kwargs):
        # These fixed-type local simulation endpoints already report missing
        # data through health/watchdogs. Avoid the default diagnostic-log
        # waitables on every high-rate message; data QoS is unchanged.
        kwargs.setdefault('event_callbacks',PublisherEventCallbacks(use_default_callbacks=False))
        return super().create_publisher(*args, **kwargs)

    def create_subscription(self, *args, **kwargs):
        kwargs.setdefault('event_callbacks',SubscriptionEventCallbacks(use_default_callbacks=False))
        return super().create_subscription(*args, **kwargs)

    def __init__(self):
        # The explicit /clock callback below owns simulation time and safety.
        # No ROS timer uses the node clock: avoid a second implicit subscription.
        super().__init__('ubts_sim_adapter',parameter_overrides=[Parameter('use_sim_time',value=False)])
        self.config=load_rover_config(ROOT/'rover.yaml')
        self.native_display_mode=runtime_mode()=='native_sensors_observe'
        self.encoderless_mode=runtime_mode() in ('encoderless_observe','native_sensors_observe')
        self.encoderless_status={'source':ENCODERLESS_SOURCE,'state':'waiting','fault':None,
                                 'reason':'waiting_for_localization','localization_valid':False,
                                 'autonomy_eligible':False,'runtime_authority':False,'last_fault':None}
        self.lock=threading.RLock();self.shutting_down=False
        self.sim_time=0.;self.clock_seen=0.;self.last={};self.count={};self.odom=None
        self.clock_stamp=Clock().clock  # Exact visualization stamp; zero before /clock.
        self.planar_odometry=None if self.encoderless_mode else PlanarOdometry()
        self.wheel_messages={}
        self.stopped_odometry_fault=None
        self.sensor_telemetry=SensorTelemetry(self.config['gps'],save)
        self.gps_last_stamp=-999.
        self.world_metadata=json.loads(Path('/runtime/world_metadata.json').read_text())
        self.camera_health=CameraHealthReader()
        self.snapshot_writer=None  # Started by main after construction.
        self.sensor_telemetry.world={k:self.world_metadata.get(k) for k in
                                     ('preset','quality','spawn','row_y','row_x_range','lane_centers_y')}
        self.command=self.create_publisher(Twist,'/sim/cmd_vel',10)
        self.scan_pub=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.imu_pub=self.create_publisher(Imu,'/imu/data_raw',qos_profile_sensor_data)
        self.odom_pub=None if self.encoderless_mode else self.create_publisher(Odometry,'/odom',10)
        self.status_pub=self.create_publisher(String,'/rover/status',10)
        self.gps_pub=self.create_publisher(String,'/gps/data',qos_profile_sensor_data)
        self.gps_fix_pub=self.create_publisher(NavSatFix,'/gps/fix',qos_profile_sensor_data)
        self.gps_driver_pub=self.create_publisher(String,'/gps/driver_status',10)
        self.tf=None if self.encoderless_mode else TransformBroadcaster(self)
        self.static=StaticTransformBroadcaster(self)
        self.motor=SimMotor(self,self.config['vehicle']);self.control=SimulationControl(self.motor)
        if self.encoderless_mode:self.motor.measured=[None,None]
        self.autonomy=AutonomousControl(self.control,self.config,self.world_metadata,sim_clock=lambda:self.sim_time)
        self.autonomy.feedback_check=self.update_sensor_wait_reason
        self.visualization=SensorVisualization(self,scan=False)
        self.callback_timing=CallbackTiming()
        self.callback_generation=0
        inputs=[(Clock,'/clock',self.clock_cb),(LaserScan,'/sim/scan',self.scan_cb),(Imu,'/sim/imu',self.imu_cb)]
        inputs+=([(String,'/encoderless/status',self.encoderless_cb)] if self.encoderless_mode else
                 [(Odometry,'/sim/wheel_odom',self.odom_cb),(JointState,'/joint_states',self.joint_cb)])
        for typ,topic,fn in inputs:
            # Camera liveness uses the existing bridge's small RAW-clock record;
            # this control executor does not receive full RGB/CameraInfo samples.
            # Integration needs measured increments across a short executor
            # stall. Only wheel/IMU get bounded history; control stays zero
            # until the paired pose has caught up to the current clock.
            self.create_subscription(typ,topic,self.observed_callback(topic,fn),self.sensor_qos(topic))
        self.static_frames()
        if self.config['gps'].get('enabled'):
            self.create_subscription(Odometry,'/ground_truth/odom',self.observed_callback('/ground_truth/odom',self.gps_cb),qos_profile_sensor_data)

    def observed_callback(self,topic,callback):
        observed=self.callback_timing.wrap(topic,callback)
        def received(message,info):
            try:return observed(message,info)
            finally:self.callback_generation+=1
        received._observed_callback=callback
        return received

    def touch(self,key):self.last[key]=time.monotonic();self.count[key]=self.count.get(key,0)+1
    def clock_cb(self,msg):
        t=msg.clock.sec+msg.clock.nanosec*1e-9
        with self.lock:
            if t<self.sim_time:
                self.autonomy.stop('clock_reset');self.control.stop('clock_reset');self.last.clear()
                if self.planar_odometry is not None:self.planar_odometry.reset()
                self.wheel_messages.clear();self.stopped_odometry_fault=None;self.odom=None;self.autonomy.odom=None;self.autonomy.yaw_measurement={};self.autonomy.sensor_wait_reason=None
            if t!=self.sim_time:self.clock_seen=time.monotonic()
            self.sim_time=t
            self.clock_stamp=msg.clock
    def scan_cb(self,msg):
        msg.header.frame_id='laser'
        # Gazebo renders all rays at the same instant. No invented rolling scan time.
        msg.scan_time=1/self.config['lidar']['hz'];msg.time_increment=0.
        self.callback_timing.measure_stage('scan_republish',self.scan_pub.publish,msg)
        self.callback_timing.measure_stage('scan_control_cache',self.cache_scan,msg)
    def cache_scan(self,msg):
        self.touch('scan')
        self.autonomy.scan={'ranges':list(msg.ranges),'angle_min':msg.angle_min,
                            'angle_increment':msg.angle_increment,'range_min':msg.range_min,
                            'range_max':msg.range_max,'wall_stamp':time.monotonic(),
                            'sim_stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9}
    def imu_cb(self,msg):
        if not getattr(self,'encoderless_mode',False):
            self.callback_timing.measure_stage('imu_estimator',self.integrate_imu,msg)
        self.callback_timing.measure_stage('imu_message_prepare',self.prepare_imu,msg)
        self.callback_timing.measure_stage('imu_publish_touch',self.publish_imu,msg)
    def integrate_imu(self,msg):
        q=msg.orientation;rate=msg.angular_velocity
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        with self.lock:
            try:
                if msg.orientation_covariance[0]<0:raise ValueError('imu_orientation_unavailable')
                accepted=self.planar_odometry.add_imu(stamp,(q.x,q.y,q.z,q.w),(rate.x,rate.y,rate.z),time.monotonic())
                if accepted:
                    imu_stamp,_,yaw_rate=self.planar_odometry.imu[-1]
                    self.autonomy.yaw_measurement={'sim_stamp':imu_stamp,'yaw_rate':yaw_rate,
                                                   'wall_stamp':time.monotonic()}
                    if self.planar_odometry.pending_wheel is not None:
                        self.odom_cb(retry=True)
            except ValueError as error:
                self.planar_odometry.fault=str(error)
                self.autonomy.yaw_measurement={}
                self.stop_odometry_fault()
    def prepare_imu(self,msg):
        msg.header.frame_id='imu_link'
        # Ideal simulated IMU: zero means covariance unknown, never hardware calibration.
        msg.orientation_covariance=[0.]*9;msg.angular_velocity_covariance=[0.]*9;msg.linear_acceleration_covariance=[0.]*9
    def publish_imu(self,msg):
        self.imu_pub.publish(msg);self.touch('imu')
    def encoderless_cb(self,msg):
        try:
            value=strict_json(msg.data,32768)
            if (not isinstance(value,dict) or value.get('source')!=ENCODERLESS_SOURCE
                    or value.get('autonomy_eligible') is not False or value.get('runtime_authority') is not False
                    or type(value.get('localization_valid')) is not bool):return
        except (ValueError,TypeError):return
        with self.lock:
            value.setdefault('last_fault',None)
            self.encoderless_status=value
            self.touch('localization_status')
            receipt=value.get('scan_receipt_ns')
            if value['localization_valid'] and type(receipt)is int and 0<=receipt<=time.monotonic_ns():
                original=receipt/1e9
                if original>self.last.get('odom',-999):self.count['odom']=self.count.get('odom',0)+1
                self.last['odom']=original  # status receipt cannot renew an old pose
    def odom_cb(self,msg=None,retry=False):
        # Encoder heading diverges on skid turns. Recover signed encoder
        # travel, then integrate it using IMU heading, never truth/GPS pose.
        with self.lock:
            try:
                if retry:estimate=self.planar_odometry.flush_wheel(time.monotonic())
                else:
                    p=msg.pose.pose.position;q=msg.pose.pose.orientation
                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                    # Duplicate stamps are ignored by the integrator; retain
                    # the same first source frame in the publication cache.
                    self.wheel_messages.setdefault(stamp,msg)
                    now=time.monotonic()
                    if self.planar_odometry.fault:
                        safe_startup=(not self.motor.ever_enabled and self.control.lease is None
                                      and not self.autonomy.active and self.motor.stopped
                                      and max(map(abs,self.motor.target))<.001
                                      and now-self.last.get('joints',-999)<.2
                                      and now-self.autonomy.yaw_measurement.get('wall_stamp',-999)<.2)
                        estimate=self.planar_odometry.recover_stationary_startup(
                            stamp,p.x,p.y,(q.x,q.y,q.z,q.w),now,allowed=safe_startup)
                    else:
                        estimate=self.planar_odometry.queue_wheel(stamp,p.x,p.y,(q.x,q.y,q.z,q.w),now)
            except ValueError as error:
                self.planar_odometry.fault=str(error);estimate=None
            if estimate is None:
                pending={sample[0] for sample in self.planar_odometry.pending_wheels}
                self.wheel_messages={stamp:value for stamp,value in self.wheel_messages.items() if stamp in pending}
                if self.planar_odometry.pending_wheel is not None:
                    self.autonomy.sensor_wait_reason='waiting_for_wheel_imu_pair'
                if self.planar_odometry.fault:
                    self.stop_odometry_fault()
                return
            # A partial flush may integrate an older frame while a newer
            # wheel still waits for IMU. Publish the exact accepted frame.
            msg=self.wheel_messages.get(estimate.stamp)
            if msg is None:
                self.planar_odometry._latch_fault('wheel_message_mismatch')
                self.stop_odometry_fault()
                return
            pending={sample[0] for sample in self.planar_odometry.pending_wheels}
            self.wheel_messages={stamp:value for stamp,value in self.wheel_messages.items() if stamp in pending}
            self.update_sensor_wait_reason(time.monotonic())
            self.stopped_odometry_fault=None
        msg=copy.deepcopy(msg)
        msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
        msg.pose.pose.position.x=estimate.x;msg.pose.pose.position.y=estimate.y;msg.pose.pose.position.z=0.
        msg.pose.pose.orientation.x=0.;msg.pose.pose.orientation.y=0.
        msg.pose.pose.orientation.z=math.sin(estimate.yaw/2);msg.pose.pose.orientation.w=math.cos(estimate.yaw/2)
        msg.twist.twist.linear.x=estimate.forward_velocity;msg.twist.twist.linear.y=0.;msg.twist.twist.linear.z=0.
        msg.twist.twist.angular.x=0.;msg.twist.twist.angular.y=0.;msg.twist.twist.angular.z=estimate.yaw_rate
        self.odom_pub.publish(msg);self.touch('odom');self.odom=msg
        self.autonomy.odom={**pose_of(msg),'wall_stamp':time.monotonic(),
                            'sim_stamp':estimate.stamp,'yaw_rate':estimate.yaw_rate}
        tr=TransformStamped();tr.header=msg.header;tr.child_frame_id='base_footprint';tr.transform.translation.x=msg.pose.pose.position.x;tr.transform.translation.y=msg.pose.pose.position.y;tr.transform.rotation=msg.pose.pose.orientation;self.tf.sendTransform(tr)
    def gps_cb(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp>=self.gps_last_stamp and stamp-self.gps_last_stamp<.19:return
        self.gps_last_stamp=stamp
        gps=self.sensor_telemetry.update_gps(msg,self.world_metadata.get('spawn',[-3,0,0]))
        self.gps_pub.publish(String(data=json.dumps(gps,allow_nan=False)));self.touch('gps')
        self.gps_fix_pub.publish(make_navsat(gps,NavSatFix))
        self.gps_driver_pub.publish(String(data=json.dumps(self.sensor_telemetry.gps_driver_status(),allow_nan=False)))
    def joint_cb(self,msg):
        v=self.config['vehicle'];m=dict(zip(msg.name,msg.velocity))
        with self.lock:
            for i,side in enumerate(['left','right']):
                vals=[m.get(f'{side}_{a}_wheel_joint') for a in ['front','rear']]
                vals=[x for x in vals if x is not None and math.isfinite(x)]
                if vals:self.motor.measured[i]=sum(vals)/len(vals)*v['wheel_radius_m']
        self.touch('joints')
    def static_frames(self):
        c=self.config;z=c['vehicle']['body_center_height_m'];l=c['lidar'];cam=c['camera']
        specs=[('base_footprint','base_link',(0,0,z),(0,0,0,1)),
               ('base_link','laser',(l['x_m'],l['y_m'],l['height_above_ground_m']-z),(0,0,0,1)),
               ('base_link','imu_link',(l['x_m'],l['y_m'],c['imu']['height_above_ground_m']-z),(0,0,0,1)),
               ('base_link','camera_link',(*cam['xyz_m'][:2],cam['xyz_m'][2]-z),(0,0,0,1)),
               ('base_link','gps_link',(0,0,GPS_HEIGHT_M-z),(0,0,0,1)),
               ('camera_link','camera_link_optical',(0,0,0),(-.5,.5,-.5,.5))]
        transforms=[]
        for parent,child,xyz,q in specs:
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q);transforms.append(t)
        self.static.sendTransform(transforms)
    def dispatch(self,payload):
        with self.lock:
            if self.shutting_down:return {'ok':False,'error':'adapter_shutting_down'}
            try:
                op=payload.get('op','')
                if getattr(self,'encoderless_mode',False) and op=='autonomy_start':
                    raise ControlError('encoderless_autonomy_disabled_pending_permit')
                if op.startswith('autonomy_'):return self.autonomy.dispatch(payload)
                if op in ('arm','stop'):
                    self.autonomy.stop('manual_override' if op=='arm' else payload.get('reason','operator_stop'))
                elif self.autonomy.active and op in ('drive','release','keepalive'):
                    raise ControlError('autonomy_active')
                return self.control.dispatch(payload)
            except (ControlError,ValueError,TypeError) as e:return {'ok':False,'error':str(e)}
    def stop_odometry_fault(self):
        fault=self.planar_odometry.fault
        if fault is None:return
        self.odom=None;self.autonomy.odom=None;self.motor.ready=False
        if self.stopped_odometry_fault==fault:return
        self.stopped_odometry_fault=fault
        # ROS may keep delivering hundreds of callbacks per second after a
        # fault. Publish the emergency stop once per transition, not per frame.
        autonomous=self.autonomy.active
        self.autonomy.stop('odometry_fault')
        if not autonomous:self.control.stop('odometry_fault')

    def update_sensor_wait_reason(self, now):
        if getattr(self,'encoderless_mode',False):
            self.autonomy.sensor_wait_reason='encoderless_autonomy_disabled_pending_permit'
            return
        waiting=not self.planar_odometry.control_pair_ready(self.sim_time)
        # Newly drained old messages get a recent callback receipt time.
        # A stale cached /clock must not make those old samples look current.
        if not 0<=now-self.clock_seen<=.2:waiting=True
        # Catch-up does not erase an acquisition window already expired while
        # callbacks were draining. Existing autonomy code performs the stop.
        since=self.autonomy.sensor_wait_since
        if since is not None and now-since>.2:waiting=True
        self.autonomy.sensor_wait_reason='waiting_for_wheel_imu_pair' if waiting else None

    def safety_tick(self):
        timing=getattr(self,'loop_timing',None)
        lock_started=timing.start_stage('safety_lock_wait') if timing else None
        with self.lock:
            if timing:timing.end_stage('safety_lock_wait',lock_started)
            body_started=timing.start_stage('safety_body') if timing else None
            try:
                self._safety_locked()
                self.autonomy.check_watchdogs()
            finally:
                if timing:timing.end_stage('safety_body',body_started)

    def _safety_locked(self):
        if getattr(self,'encoderless_mode',False):
            # Manual input ownership/expiry remain unchanged. Localization
            # readiness is separate and cannot authorize autonomous commands.
            self.motor.ready=self.clock_seen>0 and 0<=time.monotonic()-self.clock_seen<1.
            self.update_sensor_wait_reason(time.monotonic());self.control.tick()
            return
        if self.planar_odometry.expire_pending(time.monotonic()):
            self.stop_odometry_fault()
        # Wall-clock expiry remains live while simulation is paused.
        self.motor.ready=(time.monotonic()-self.clock_seen<1.
                          and self.planar_odometry.pose is not None
                          and self.planar_odometry.fault is None)
        self.update_sensor_wait_reason(time.monotonic())
        self.control.tick()

    def step(self, *, allow_planning=True):
        timing=getattr(self,'loop_timing',None)
        lock_started=timing.start_stage('step_lock_wait') if timing else None
        with self.lock:
            if timing:timing.end_stage('step_lock_wait',lock_started)
            self._safety_locked()
            if not getattr(self,'encoderless_mode',False):self.autonomy.step(allow_planning=allow_planning)
            self.motor.step()
    def snapshot(self):
        if self.snapshot_writer is None:raise RuntimeError('snapshot_writer_not_started')
        # Read-only health I/O is outside the control lock. Its original RAW
        # receipts remain the authority; reading never refreshes the sensor.
        camera_health=self.camera_health.read(os.environ.get('CAMERA_BRIDGE_STATUS_FILE','/runtime/camera_bridge_status.json'))
        timing=getattr(self,'loop_timing',None)
        lock_started=timing.start_stage('snapshot_lock_wait') if timing else None
        with self.lock:
            if timing:timing.end_stage('snapshot_lock_wait',lock_started)
            now=time.monotonic()
            # Gazebo can repeat JointState while paused; it is not a fresh sample.
            clock_alive=now-self.clock_seen<1.
            alive={key:clock_alive and now-self.last.get(key,-999)<2. for key in ['scan','imu','odom','joints']}
            if self.encoderless_mode:alive.pop('joints')
            if getattr(self,'native_display_mode',False):alive.pop('odom')
            for key in ('camera','camera_info'):
                alive[key]=clock_alive and camera_health[key]['alive']
                self.count[key]=camera_health[key]['count']
            captured_at=time.time()
            status={**self.control.status(),'updated_at':captured_at,'transport':'simulation','mode':'simulation','serial_open':False,'simulated':True,'measured_wheel_m_s':list(self.motor.measured),'commanded_wheel_m_s':list(self.motor.applied),
                    'effective_wheel_targets_m_s':list(self.motor.effective),'actuator_response':self.motor.response.status()}
            odometry=dict(self.encoderless_status) if self.encoderless_mode else self.planar_odometry.status()
            if self.encoderless_mode:
                if now-self.last.get('localization_status',-999)>1.:
                    odometry.update(state='waiting',reason='localization_status_stale',localization_valid=False)
                status.update(measured_wheels_available=False,stopped_semantics='command_output_zero_only',
                              manual_ready=self.motor.ready,autonomy_eligible=False)
                odometry.update(manual_ready=self.motor.ready,autonomy_eligible=False)
            self.status_pub.publish(String(data=json.dumps(status)))
            # This one small bounded fault record contains nested receipt lists.
            odometry['last_fault']=copy.deepcopy(odometry['last_fault'])
            payloads={'rover_bridge_status.json':status,
                      'simulation_status.json':{'updated_at':captured_at,'sim_time':self.sim_time,'clock_alive':clock_alive,'sensors_alive':alive,'counts':dict(self.count),'control':status,'odometry':odometry,
                                           'loop_timing':timing.writer_snapshot() if timing else None,'camera_health':camera_health,
                                           'callback_timing':self.callback_timing.snapshot(),'snapshot_writer':self.snapshot_writer.status(),
                                           'gc_timing':self.gc_timing.snapshot() if getattr(self,'gc_timing',None) else None}}
            # Reuse only an immutable path keyed by rounded XY content. Mutable
            # waypoints/diagnostics still detach before the writer owns them.
            autonomy_status=self.autonomy.status()
            if self.encoderless_mode:autonomy_status.update(autonomy_eligible=False,enabled=False,
                reason='encoderless_autonomy_disabled_pending_permit',message='encoderless_autonomy_disabled_pending_permit')
            autonomy_status['path']=self.autonomy.mission.path_snapshot(borrow=True)
            autonomy_status['waypoints']=[dict(point) for point in self.autonomy.mission.waypoints]
            autonomy_status['obstacle_diagnostic']=copy.deepcopy(autonomy_status.get('obstacle_diagnostic'))
            payloads['autonomy_status.json']=autonomy_status
            self.visualization.mission(autonomy_status,borrow_path=True)
            cam=self.config['camera']
            payloads['camera_driver_status.json']={'updated_at':captured_at,'state':'streaming' if alive['camera'] else 'waiting','running':alive['camera'],'simulated':True,'width':cam['width'],'height':cam['height'],'fps':cam['hz'],'pixel_format':'RGB8','rotate_180':False}
            payloads['imu_driver_status.json']={'updated_at':captured_at,'state':'streaming' if alive['imu'] else 'waiting','sensor':'BNO085 simulation','transport':'gazebo','orientation_convention':'ENU_FLU_simulated','rep145_ready':True,'simulated':True}
            if not self.config['gps'].get('enabled'):
                payloads['gps_bridge_status.json']={'updated_at':captured_at,'state':'not_connected','valid':False,'connected':False,'simulated':True}
        self.snapshot_writer.submit(payloads)

class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(.5)
        try:
            raw=self.rfile.readline(4097)
            if len(raw)>4096:raise ValueError('request_too_large')
            payload=json.loads(raw)
            if not isinstance(payload,dict):raise ValueError('object_required')
            reply=self.server.adapter.dispatch(payload)
        except (ValueError,OSError) as e:reply={'ok':False,'error':str(e)}
        try:self.wfile.write(json.dumps(reply,allow_nan=False).encode()+b'\n')
        except OSError:pass

class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads=True

def main():
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import ExternalShutdownException
    # Keep the context alive until the final zero has been published. The
    # default ROS signal handler shuts it down before our finally block.
    shutdown_requested=threading.Event()
    previous_handlers={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in previous_handlers:
        signal.signal(sig,lambda signum,frame:shutdown_requested.set())
    node=Adapter();path=Path(os.environ.get('ROVER_CONTROL_SOCKET','/run/ubts-sim/control.sock'))
    if path.parent!=Path('/run/ubts-sim'):raise RuntimeError('simulation control socket must be isolated')
    path.parent.mkdir(exist_ok=True);path.unlink(missing_ok=True)
    node.snapshot_writer=LatestSnapshotWriter(save)
    server=Server(str(path),Handler);server.adapter=node
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    next_status=0.
    try:
        from scripts.ros_executor import persistent_executor
        from loop_timing import LoopTiming,capture_context
        from callback_drain import drain_ready,idle_after_empty
        from gc_timing import GCTiming
        node.gc_timing=GCTiming();node.gc_timing.install()
        node.loop_timing=LoopTiming()
        node.autonomy.diagnostic_observer=node.loop_timing.measure_call
        context=lambda receipts:({'localization_mode':'encoderless_observe','odometry':dict(node.encoderless_status)}
                                 if node.encoderless_mode else capture_context(node,include_receipts=receipts))
        with persistent_executor(node) as executor:
            while rclpy.ok() and not shutdown_requested.is_set():
                node.loop_timing.begin(active=node.autonomy.active,wait_since=node.autonomy.sensor_wait_since,
                                       wait_reason=node.autonomy.sensor_wait_reason)
                drained=drain_ready(executor,lambda:node.callback_generation,node.safety_tick,
                                    observer=node.loop_timing.measure_call)
                idle_after_empty(drained,node.safety_tick,observer=node.loop_timing.measure_call)
                node.loop_timing.after_spin()
                node.step(allow_planning=drained['empty'])
                node.loop_timing.after_step(active=node.autonomy.active,context=context)
                had_snapshot=time.monotonic()>=next_status
                if had_snapshot:node.snapshot();next_status=time.monotonic()+.25
                node.loop_timing.finish(active=node.autonomy.active,had_snapshot=had_snapshot,context=context)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        if rclpy.ok(): raise
    finally:
        try:
            collector=getattr(node,'gc_timing',None)
            if collector is not None:collector.close()
            with node.lock:
                # Also reject already accepted handlers waiting on this lock.
                node.shutting_down=True
                node.control.stop('shutdown')
            server.shutdown();server.server_close()
            if not node.snapshot_writer.close():raise RuntimeError('snapshot_writer_flush_timeout')
        finally:
            path.unlink(missing_ok=True);node.destroy_node();rclpy.try_shutdown()
            for sig,handler in previous_handlers.items():signal.signal(sig,handler)

if __name__=='__main__':main()
