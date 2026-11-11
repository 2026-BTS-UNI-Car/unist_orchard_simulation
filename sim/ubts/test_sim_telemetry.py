"""Synthetic GPS + scan/odometry serialization checks without ROS or hardware."""
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parent))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'dashboard'))
from sim_telemetry import SensorTelemetry, serialized_range, GPS_HEIGHT_M, gps_stamp_seconds
from scripts import sim_clock


def test_mapping_sample_publishes_shared_monotonic_clock_contract(telemetry,monkeypatch):
    monkeypatch.setattr(sim_clock,'clock_domain',lambda:'test-kernel')
    monkeypatch.setattr(sim_clock,'raw_now',lambda:42.)
    obj,saved=telemetry
    obj.odom(odometry());obj.scan(laser())
    value=saved[-1][1]
    assert value['updated_monotonic_raw']==42. and value['clock_schema']==sim_clock.SCHEMA
    assert value['clock_source']==sim_clock.SOURCE and value['clock_domain']=='test-kernel' and 'updated_at' in value
    json.dumps(value,allow_nan=False)


def header(stamp):
    seconds=int(stamp)
    return NS(stamp=NS(sec=seconds,nanosec=round((stamp-seconds)*1e9)))

def vector(x=0.,y=0.,z=0.):return NS(x=x,y=y,z=z)

def odometry(stamp=10.,x=0.,y=0.,yaw=0.,z=0.,vx=0.,vy=0.):
    return NS(header=header(stamp),pose=NS(pose=NS(position=vector(x,y,z),orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))),twist=NS(twist=NS(linear=vector(vx,vy,0),angular=vector())))

def laser(stamp=10.,ranges=None):
    return NS(header=header(stamp),ranges=[5.]*720 if ranges is None else ranges,angle_min=-math.pi,angle_increment=2*math.pi/720,range_min=.05,range_max=30.)

def imu(stamp=10.):
    return NS(header=header(stamp),orientation=NS(x=0.,y=0.,z=0.,w=1.),angular_velocity=vector(),linear_acceleration=vector(0,0,9.8))

@pytest.fixture
def telemetry():
    saved=[]
    obj=SensorTelemetry({},lambda name,data:saved.append((name,data)))
    return obj,saved


def test_gps_is_explicitly_virtual_at_arbitrary_origin_without_rtk_claims(telemetry):
    obj,saved=telemetry
    sample=obj.update_gps(odometry(x=-3,y=0,z=.6),[-3,0,0])
    assert sample['simulated'] is True and sample['source']=='gazebo_virtual_gnss'
    assert sample['fix_type']=='simulated' and 'not real GNSS' in sample['description']
    assert sample['latitude']==37 and sample['longitude']==127
    assert sample['h_acc_m']==sample['position_accuracy_floor_m']==.5
    assert sample['v_acc_m'] is sample['satellites'] is None
    assert sample['time_source']=='simulation_clock_not_real_UTC'
    assert sample['position_noise_stddev_m']==0.
    assert sample['local_x']==sample['local_y']==0
    assert sample['altitude_msl_m'] is sample['altitude_ellipsoid_m'] is None
    assert sample['local_z']==pytest.approx(.6+GPS_HEIGHT_M)
    assert sample['fix_dimension']==2 and sample['course_deg'] is None
    assert sample['stamp']=={'sec':10,'nanosec':0}
    assert saved==[]


def test_virtual_gps_motion_units_and_course_use_truth_only_for_gps(telemetry):
    obj,saved=telemetry
    obj.odom(odometry(x=2,y=1))
    gps=obj.update_gps(odometry(x=97,y=50,yaw=math.pi/2,vx=3,vy=4),[-3,0,0])
    assert gps['local_x']==100 and gps['local_y']==50
    assert gps['speed_mps']==pytest.approx(5.,rel=0.,abs=1e-12)
    assert gps['course_deg']==pytest.approx(math.degrees(math.atan2(-4,3))%360)
    assert (gps['latitude']-37)*math.pi/180*6378137==pytest.approx(50)
    assert (gps['longitude']-127)*math.pi/180*6378137*math.cos(math.radians(37))==pytest.approx(100)
    obj.scan(laser())
    assert saved[-1][1]['pose']['x']==2 and saved[-1][1]['pose']['y']==1
    assert saved[-1][1]['gps']['local_x']==100,'mapping pose must not become ground truth GPS pose'


@pytest.mark.parametrize('value,expected',[(math.inf,None),(-math.inf,-1),(math.nan,-1),(None,-1),('invalid',-1),({},-1),([], -1),(1.234567,1.2346),(.049,.049),(31.,31.)])
def test_serialized_ranges_distinguish_no_return_from_invalid(value,expected):
    assert serialized_range(value)==expected


def test_scan_json_is_strict_and_invalid_does_not_become_free_space(telemetry):
    obj,saved=telemetry;obj.odom(odometry())
    obj.scan(laser(ranges=[math.inf,math.nan,-math.inf,None,'invalid',2.456789]))
    sample=saved[-1][1]
    assert sample['scan']['ranges']==[None,-1,-1,-1,-1,2.4568]
    assert sample['session_id'] and sample['seq']==1
    assert json.loads(json.dumps(sample,allow_nan=False))['stamp']==10.


def test_mapping_sample_declares_adapter_pose_source_independently_of_optional_observations(telemetry):
    obj,saved=telemetry
    obj.odom(odometry(x=2,y=1));obj.scan(laser())
    sample=saved[-1][1]
    assert not sample['imu'] and not sample['gps']
    assert sample['pose_source']=='wheel_distance_imu_heading'
    assert sample['localization']=='wheel_distance_imu_heading_without_loop_closure'
    assert sample['pose']['x']==2 and sample['pose']['y']==1


def test_scan_uses_nearest_time_aligned_odometry_and_rejects_far_pose(telemetry):
    obj,saved=telemetry
    obj.scan(laser(9.9));assert not saved
    obj.odom(odometry(9.9,x=1));obj.odom(odometry(10.,x=2))
    obj.scan(laser(9.98));assert saved[-1][1]['pose']['x']==2
    obj.scan(laser(10.5));assert len(saved)==1
    obj.odom(odometry(10.5,x=3));obj.scan(laser(10.5))
    assert len(saved)==2 and saved[-1][1]['pose']['x']==3


def test_scan_throttle_does_not_replay_duplicate_sample(telemetry):
    obj,saved=telemetry;obj.odom(odometry(10.));obj.scan(laser(10.))
    obj.scan(laser(10.));obj.scan(laser(10.05));assert len(saved)==1
    obj.odom(odometry(10.1));obj.scan(laser(10.1));assert len(saved)==2
    assert saved[1][1]['seq']>saved[0][1]['seq']


def test_clock_reset_changes_session_and_uses_only_new_epoch_pose(telemetry):
    obj,saved=telemetry;obj.odom(odometry(20.,x=100));obj.scan(laser(20.))
    before=saved[-1][1]['session_id']
    obj.odom(odometry(0.,x=0));obj.scan(laser(0.))
    after=saved[-1][1]
    assert after['session_id']!=before and after['stamp']==0 and after['pose']['x']==0
    assert len(obj.poses)==1


def test_clock_reset_does_not_attach_previous_epoch_gps_or_imu(telemetry):
    obj,saved=telemetry;obj.odom(odometry(20.));obj.update_imu(imu(20.));obj.update_gps(odometry(20.,x=100),[-3,0,0]);obj.scan(laser(20.))
    obj.odom(odometry(0.));obj.scan(laser(0.))
    assert not saved[-1][1]['gps'],'previous epoch GPS must not be archived as current'
    assert not saved[-1][1]['imu'],'previous epoch IMU must not be archived as current'


def test_nonfinite_pose_is_rejected_before_writing_invalid_json(telemetry):
    obj,saved=telemetry;obj.odom(odometry(x=math.nan));obj.scan(laser())
    assert not saved


def test_virtual_gps_survives_real_bridge_contract_and_expires(telemetry,monkeypatch):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'dashboard'))
    from scripts.gps_ws_bridge import GpsBridgeState
    import dashboard_server.gps as gps_contract
    clock=[1000.]
    monkeypatch.setattr(gps_contract.time,'time',lambda:clock[0])
    obj,_=telemetry
    sample=obj.update_gps(odometry(),[0,0,0])
    bridge=GpsBridgeState();bridge.update(NS(data=json.dumps(sample,allow_nan=False)))
    snapshot=bridge.snapshot()
    assert snapshot['valid'] and snapshot['simulated'] and snapshot['fix_type']=='simulated'
    assert snapshot['local_x']==snapshot['local_y']==0
    clock[0]+=4
    assert bridge.snapshot()['valid'] is False and bridge.snapshot()['state']=='stale'


@pytest.mark.parametrize('vx,vy,course',[(1.,0.,90.),(-1.,0.,270.),(0.,1.,0.),(0.,-1.,180.),(0.,0.,None)])
def test_gps_course_is_motion_direction_not_body_heading(telemetry,vx,vy,course):
    obj,_=telemetry
    gps=obj.update_gps(odometry(vx=vx,vy=vy),[0,0,0])
    assert gps['course_deg']==course
    assert gps['yaw']==0.


def test_antenna_position_and_velocity_use_existing_mount_and_child_frame(telemetry):
    obj,_=telemetry
    msg=odometry(z=.01)
    msg.pose.pose.orientation=NS(x=0.,y=math.sin(math.pi/4),z=0.,w=math.cos(math.pi/4))
    msg.twist.twist.linear.z=2.
    gps=obj.update_gps(msg,[0,0,0])
    assert gps['local_x']==pytest.approx(GPS_HEIGHT_M)
    assert gps['local_y']==pytest.approx(0)
    assert gps['local_z']==pytest.approx(.01)
    assert gps['speed_mps']==pytest.approx(2)
    assert gps['course_deg']==pytest.approx(90)
    msg=odometry();msg.twist.twist.angular.x=1.
    gps=obj.update_gps(msg,[0,0,0])
    assert gps['speed_mps']==pytest.approx(GPS_HEIGHT_M)
    assert gps['course_deg']==180.


def test_datum_names_are_explicit_and_legacy_altitude_is_not_ellipsoid(telemetry):
    obj,_=telemetry
    obj.config={'origin_altitude_m':999.}
    gps=obj.update_gps(odometry(z=.01),[0,0,0])
    assert gps['altitude_msl_m'] is gps['altitude_ellipsoid_m'] is None
    obj.config={'origin_altitude_msl_m':10.,'origin_altitude_ellipsoid_m':42.}
    gps=obj.update_gps(odometry(z=.01),[0,0,0])
    assert gps['altitude_msl_m']==pytest.approx(10.71)
    assert gps['altitude_ellipsoid_m']==pytest.approx(42.71)
    assert gps['fix_dimension']==3


@pytest.mark.parametrize('case',['no_fix','bad_position','bad_origin','pole','bad_quaternion','huge_quaternion'])
def test_gps_no_fix_clears_coordinates_with_strict_json(telemetry,case):
    obj,_=telemetry;msg=odometry()
    if case=='no_fix':obj.config['fix_available']=False
    if case=='bad_position':msg.pose.pose.position.x=math.nan
    if case=='bad_origin':obj.config['origin_longitude']=200.
    if case=='pole':obj.config['origin_latitude']=90.
    if case=='bad_quaternion':msg.pose.pose.orientation.w=0.
    if case=='huge_quaternion':msg.pose.pose.orientation.w=1e308
    gps=obj.update_gps(msg,[0,0,0])
    assert not gps['valid'] and gps['quality']==0 and gps['fix_type']=='no_fix'
    assert gps['latitude'] is gps['longitude'] is gps['altitude_ellipsoid_m'] is None
    assert obj.gps_driver_status()['state']=='no_fix'
    json.dumps(gps,allow_nan=False)


def test_virtual_gps_freshness_is_topic_receipt_not_a_physical_serial_port(telemetry,monkeypatch):
    import sim_telemetry
    from dashboard_server import gps as gps_module
    now=[100.]
    monkeypatch.setattr(sim_telemetry.time,'time',lambda:now[0])
    monkeypatch.setattr(gps_module.time,'time',lambda:now[0])
    obj,_=telemetry
    obj.update_gps(odometry(),[0,0,0])
    current=obj.gps_driver_status()
    assert current['connected'] and current['state']=='fix'
    assert current['driver']['serial_open'] is None
    assert current['driver']['serial_applicable'] is False
    now[0]=104.
    expired=obj.gps_driver_status()
    assert expired['state']=='stale' and not expired['connected'] and not expired['valid']
    obj.config['fix_available']=False
    obj.update_gps(odometry(11.),[0,0,0])
    assert obj.gps_driver_status()['state']=='no_fix'
    # The same recent sample cannot bypass the real disconnected USB guard.
    assert gps_module.fresh_snapshot(obj.gps,104.,driver={'serial_open':False})['state']=='stale'


def test_gps_exact_stamp_preserves_ns_and_reset_zero(telemetry):
    obj,_=telemetry;msg=odometry()
    msg.header.stamp.sec=16_777_216;msg.header.stamp.nanosec=123_456_789
    first=obj.update_gps(msg,[0,0,0])
    assert first['stamp']=={'sec':16_777_216,'nanosec':123_456_789}
    second=obj.update_gps(odometry(0.),[0,0,0])
    assert second['stamp']=={'sec':0,'nanosec':0} and second['sequence']>first['sequence']


@pytest.mark.parametrize('bad',[True,-1.,math.nan,math.inf,10**400,{'sec':True,'nanosec':0},{'sec':0,'nanosec':1e3},{'sec':0,'nanosec':10**9},{'sec':2**31,'nanosec':0}])
def test_gps_stamp_reader_rejects_malformed_values(bad):
    with pytest.raises(ValueError):gps_stamp_seconds(bad)
