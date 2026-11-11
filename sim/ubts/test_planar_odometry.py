"""Independent analytic path and timing checks for wheel/IMU dead reckoning."""
import math
from pathlib import Path
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parent))
from planar_odometry import PlanarOdometry


def q(yaw):
    return (0.,0.,math.sin(yaw/2),math.cos(yaw/2))


def test_skidding_encoder_yaw_cannot_invent_actual_turn():
    estimator=PlanarOdometry()
    # Wheels report an arc of radius 1, but IMU observes a straight heading.
    # A wheel-only pose would turn 90 degrees; output must advance pi/2 metres.
    for i in range(101):
        angle=i*math.pi/200
        stamp=i*.05
        estimator.add_imu(stamp,q(.4),(0.,0.,0.))
        pose=estimator.update_wheel(stamp,math.sin(angle),1-math.cos(angle),q(angle))
    assert pose.x == pytest.approx(math.pi/2,abs=1e-9)
    assert pose.y == pytest.approx(0.,abs=1e-9)
    assert pose.yaw == pytest.approx(0.,abs=1e-9)


def test_true_quarter_circle_uses_imu_heading_and_wheel_distance():
    estimator=PlanarOdometry()
    for i in range(101):
        angle=i*math.pi/200
        stamp=i*.05
        estimator.add_imu(stamp,q(angle),(0.,0.,math.pi/10))
        pose=estimator.update_wheel(stamp,math.sin(angle),1-math.cos(angle),q(angle))
    assert pose.x == pytest.approx(1.,abs=1e-9)
    assert pose.y == pytest.approx(1.,abs=1e-9)
    assert pose.yaw == pytest.approx(math.pi/2)


def test_reverse_and_in_place_turn_preserve_signed_distance():
    e=PlanarOdometry();e.add_imu(0,q(0),(0,0,0));e.update_wheel(0,0,0,q(0))
    e.add_imu(.1,q(0),(0,0,0));p=e.update_wheel(.1,-.1,0,q(0))
    assert p.x == pytest.approx(-.1) and p.forward_velocity == pytest.approx(-1.)
    e.add_imu(.2,q(.2),(0,0,2));p=e.update_wheel(.2,-.1,0,q(.8))
    assert p.x == pytest.approx(-.1) and p.y == pytest.approx(0.)
    assert p.yaw == pytest.approx(.2)


def test_heading_unwrap_interpolation_and_bounded_prediction():
    e=PlanarOdometry()
    e.add_imu(0,q(math.pi-.02),(0,0,.8))
    e.add_imu(.05,q(-math.pi+.02),(0,0,.8))
    assert e.heading_at(.025) == pytest.approx(.02)
    assert e.heading_at(.075) == pytest.approx(.06)
    assert e.heading_at(.151) is None


def test_missing_stale_duplicate_reset_contract():
    e=PlanarOdometry()
    assert e.update_wheel(0,0,0,q(0)) is None
    e.add_imu(0,q(1),(0,0,0));assert e.update_wheel(0,0,0,q(0)) is not None
    assert e.update_wheel(0,1,0,q(1)) is None
    assert e.update_wheel(.15,.1,0,q(1)) is None
    assert e.fault == 'imu_heading_stale'
    e.add_imu(.2,q(1),(0,0,0));assert e.update_wheel(.2,.2,0,q(1)) is None
    e.reset();e.add_imu(0,q(-1),(0,0,0));p=e.update_wheel(0,5,3,q(2))
    assert (p.x,p.y,p.yaw) == (0.,0.,0.)
    assert e.status()['uses_ground_truth'] is False


def test_wheel_gap_and_invalid_imu_fail_without_heading_fallback():
    e=PlanarOdometry();e.add_imu(0,q(0),(0,0,0));e.update_wheel(0,0,0,q(0))
    e.add_imu(1,q(0),(0,0,0));assert e.update_wheel(1,1,0,q(0)) is None
    assert e.fault == 'wheel_time_gap'
    with pytest.raises(ValueError,match='quaternion'):
        PlanarOdometry().add_imu(0,(0,0,0,0),(0,0,0))


def test_latest_gyro_yaw_derivative_accounts_for_body_tilt_and_rejects_duplicates():
    e=PlanarOdometry()
    roll,pitch,yaw=.3,-.2,.4
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    quaternion=(sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,
                cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy)
    # Pure world vertical yaw motion expressed in the tilted body frame.
    rate=.25
    body=(-rate*math.sin(pitch),rate*math.sin(roll)*math.cos(pitch),
          rate*math.cos(roll)*math.cos(pitch))
    assert e.add_imu(1.,quaternion,body)
    assert e.imu[-1][2]==pytest.approx(rate)
    assert not e.add_imu(1.,quaternion,(0,0,99))
    assert not e.add_imu(.9,quaternion,(0,0,99))
    assert e.imu[-1][0]==1. and e.imu[-1][2]==pytest.approx(rate)


def test_delayed_imu_callback_can_complete_pending_wheel_without_extra_prediction():
    e=PlanarOdometry();e.add_imu(0,q(0),(0,0,.2))
    e.queue_wheel(0,0,0,q(0),wall_time=1.)
    # Latest-only wheel arrives before IMU after a scheduling stall. Its
    # 150 ms horizon is deliberately outside the 100 ms prediction limit.
    assert e.queue_wheel(.15,.03,0,q(0),wall_time=1.15) is None
    assert e.fault is None and e.pose.stamp==0
    e.add_imu(.15,q(.03),(0,0,.2))
    pose=e.flush_wheel(1.16)
    assert pose.stamp==.15 and pose.yaw==pytest.approx(.03)
    assert pose.x==pytest.approx(.03*math.sin(.03)/.03)
    assert e.pending_wheel is None and e.fault is None


def test_missing_imu_latches_after_bounded_wait_not_refreshed_by_more_wheels():
    e=PlanarOdometry();e.add_imu(0,q(0),(0,0,0));e.queue_wheel(0,0,0,q(0),1.)
    assert e.queue_wheel(.15,.03,0,q(0),1.15) is None
    assert e.queue_wheel(.2,.04,0,q(0),1.25) is None
    assert e.expire_pending(1.351)=='imu_heading_stale'
    e.add_imu(.2,q(0),(0,0,0))
    assert e.flush_wheel(1.36) is None
    assert e.pose.stamp==0


def test_dropped_imu_frames_use_bounded_nearest_gyro_prediction():
    e=PlanarOdometry();e.add_imu(0,q(0),(0,0,.2));e.add_imu(.3,q(.06),(0,0,.2))
    assert e.heading_at(.29)==pytest.approx(.058)
    assert e.heading_at(.15) is None  # The unsupported middle cannot be invented.


def startup_fault():
    e=PlanarOdometry();e.add_imu(0,q(.4),(0,0,0));e.update_wheel(0,2,3,q(0))
    e.queue_wheel(.15,2,3,q(0),1.)
    e.expire_pending(1.201)
    return e


def test_stationary_never_armed_startup_recovery_preserves_original_frame():
    e=startup_fault();original_heading=e.initial_heading
    for i in range(25):
        stamp=2+i*.05;e.add_imu(stamp,q(.4),(0,0,0))
        pose=e.recover_stationary_startup(stamp,2,3,q(0),stamp,allowed=True)
        if pose is not None:break
    assert pose is not None and (pose.x,pose.y,pose.yaw)==(0,0,0)
    assert e.initial_heading==original_heading and e.fault is None
    assert e.startup_recoveries==1 and e.status()['last_fault']['pending_wait_s']>.2


@pytest.mark.parametrize('cause',['armed','wheel_motion','encoder_turn','imu_turn','gyro_motion'])
def test_startup_recovery_never_hides_possible_motion_or_prior_arm(cause):
    e=startup_fault()
    for i in range(30):
        stamp=2+i*.05
        e.add_imu(stamp,q(.42 if cause=='imu_turn' else .4),(0,0,.03 if cause=='gyro_motion' else 0))
        assert e.recover_stationary_startup(stamp,2.001 if cause=='wheel_motion' else 2,
                   3,q(.002 if cause=='encoder_turn' else 0),stamp,allowed=cause!='armed') is None
    assert e.fault=='imu_heading_stale' and e.startup_recoveries==0
    # A stopped sample later cannot erase the detected motion evidence.
    if cause!='armed':assert e.startup_motion_detected


def test_first_fault_reason_and_context_are_not_overwritten_by_later_callbacks():
    e=PlanarOdometry();e.add_imu(0,q(0),(0,0,0));e.update_wheel(0,0,0,q(0))
    e.add_imu(.4,q(0),(0,0,0));e.queue_wheel(.4,0,0,q(0),.4)
    assert e.fault=='wheel_time_gap'
    saved=e.status()['last_fault'].copy()
    e.queue_wheel(1,0,0,q(0),1);e.expire_pending(2)
    assert e.fault=='wheel_time_gap' and e.status()['last_fault']==saved


def test_repeated_paused_wheel_stamp_cannot_complete_startup_recovery():
    e=startup_fault();e.add_imu(2,q(.4),(0,0,0))
    for i in range(30):
        assert e.recover_stationary_startup(2,2,3,q(0),2+i*.05,allowed=True) is None
    assert e.fault and e.startup_recoveries==0


def test_received_intermediate_wheels_are_not_overwritten_into_a_false_gap():
    e=PlanarOdometry();e.add_imu(0,q(0),(0,0,0));e.queue_wheel(0,0,0,q(0),1.)
    # Reproducer for old latest-pending logic: it discarded .15 and .20,
    # then falsely rejected 0->.29 even though all three wheels arrived.
    for stamp,receipt in [(.15,1.01),(.20,1.02),(.29,1.03)]:
        assert e.queue_wheel(stamp,stamp,0,q(0),receipt) is None
    e.add_imu(.20,q(0),(0,0,0),1.04)
    pose=e.flush_wheel(1.04)
    assert pose.stamp==.29 and pose.x==pytest.approx(.29)
    assert e.distance==pytest.approx(.29) and e.fault is None
    assert not e.pending_wheels and e.control_pair_ready(.30)


def test_partial_flush_preserves_order_and_original_acquisition_deadline():
    e=PlanarOdometry();e.add_imu(0,q(0),(0,0,0));e.queue_wheel(0,0,0,q(0),1.)
    e.queue_wheel(.15,.15,0,q(0),1.01);e.queue_wheel(.29,.29,0,q(0),1.02)
    e.add_imu(.1,q(0),(0,0,0),1.03)
    pose=e.flush_wheel(1.03)
    assert pose.stamp==.15 and e.pending_wheel[0]==.29
    assert e.pending_since==1.01 and not e.control_pair_ready(.29)
    # Availability at the deadline does not authorize a late result.
    e.add_imu(.25,q(0),(0,0,0),1.22)
    assert e.flush_wheel(1.22) is None and e.fault=='imu_heading_stale'
    assert e.pose.stamp==.15 and e.status()['last_fault']['pending_wait_s']==pytest.approx(.21)


def test_bounded_history_saturates_closed_and_diagnostics_freeze_first_receipts():
    e=PlanarOdometry();e.add_imu(0,q(0),(0,0,0));e.queue_wheel(0,0,0,q(0),1.)
    for i in range(e.HISTORY_DEPTH):e.queue_wheel(.15+i*.01,i,0,q(0),1.01)
    assert len(e.pending_wheels)==8 and e.fault is None
    e.queue_wheel(.24,9,0,q(0),1.02)
    assert e.fault=='wheel_history_overflow' and len(e.pending_wheels)==8
    saved=e.status()['last_fault'].copy()
    assert len(saved['wheel_receipts'])==8 and len(saved['pending_stamps'])==8
    e.add_imu(1.,q(0),(0,0,0),2.)
    assert e.status()['last_fault']==saved


def test_genuine_missing_wheel_gap_still_faults_and_old_pose_cannot_drive():
    e=PlanarOdometry();e.add_imu(0,q(0),(0,0,0));e.queue_wheel(0,0,0,q(0),1.)
    assert not e.control_pair_ready(.11)
    assert not e.control_pair_ready(-.04)
    e.add_imu(.29,q(0),(0,0,0),1.01)
    assert e.queue_wheel(.29,.29,0,q(0),1.02) is None
    assert e.fault=='wheel_time_gap' and e.pose.stamp==0 and e.max_wheel_gap==.25


def test_duplicate_and_regressed_pending_wheels_do_not_reorder_or_renew_wait():
    e=PlanarOdometry();e.add_imu(0,q(0),(0,0,0));e.queue_wheel(0,0,0,q(0),1.)
    e.queue_wheel(.15,.15,0,q(0),1.01)
    e.queue_wheel(.15,999,0,q(0),1.1);e.queue_wheel(.14,999,0,q(0),1.11)
    assert len(e.pending_wheels)==1 and e.pending_wheel[1]==.15 and e.pending_since==1.01


def test_wheels_before_first_imu_cannot_pin_startup_or_invent_a_reference():
    e=PlanarOdometry()
    assert e.queue_wheel(0.,5.,3.,q(0),1.) is None
    e.add_imu(.05,q(.4),(0,0,0),2.)
    assert e.flush_wheel(2.) is None and e.pose is None and not e.pending_wheels
    pose=e.queue_wheel(.05,6.,3.,q(0),2.01)
    assert (pose.x,pose.y,pose.yaw)==(0.,0.,0.)
    assert e.fault is None and e.distance==0 and e.startup_unpaired_discarded==1


def test_startup_without_imu_is_bounded_and_remains_uninitialized():
    e=PlanarOdometry()
    for i in range(100):assert e.queue_wheel(i*.05,float(i),0,q(0),float(i)) is None
    assert len(e.pending_wheels)==8 and e.pose is None and not e.control_pair_ready(5.)
    assert e.startup_unpaired_discarded==92 and e.fault is None
    e.add_imu(5.,q(0),(0,0,0),101.)
    e.flush_wheel(101.)
    pose=e.queue_wheel(5.,100.,0,q(0),101.01)
    assert pose.x==0 and e.distance==0 and e.fault is None
