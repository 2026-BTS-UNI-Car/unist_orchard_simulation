import math
import json
from dataclasses import asdict
from pathlib import Path
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parent))
from yaw_rate_tracker import YawRateTracker
from planar_odometry import PlanarOdometry


def test_numpy_planner_values_produce_json_serializable_status():
    import numpy as np
    tracker=YawRateTracker()
    command=tracker.step(np.float64(.3),np.float64(.2),np.float64(.05),
                         np.float64(1),np.float64(1),authorized=True)
    serialized=json.loads(json.dumps(asdict(command),allow_nan=False))
    assert serialized['valid'] is True and isinstance(serialized['saturated'],bool)
    assert type(command.saturated) is bool


def test_feedback_tracks_rate_on_independent_skid_plant():
    tracker=YawRateTracker()
    measured=0.
    # Independent first-order plant: only 30% of kinematic yaw becomes motion.
    for i in range(601):
        command=tracker.step(.3,.16,measured,i*.05,i*.05,authorized=True)
        measured+=(.30*command.angular-measured)*.05/.35
        assert command.valid and max(abs(command.left_m_s),abs(command.right_m_s)) <= 1.
    assert measured == pytest.approx(.16,abs=.001)
    assert command.angular > .5


def test_saturated_wheels_do_not_accumulate_integral():
    tracker=YawRateTracker()
    for i in range(100):
        command=tracker.step(1.,.3,0.,i*.05,i*.05,authorized=True)
        assert command.saturated
        assert command.left_m_s == command.right_m_s == 1.
    assert tracker.integral == 0.
    command=tracker.step(.2,-.1,0.,5.,5.,authorized=True)
    assert command.angular < 0


def test_reverse_motion_and_turn_sign_change_reset_integral():
    tracker=YawRateTracker()
    for i in range(20):
        tracker.step(-.2,.15,.05,i*.05,i*.05,authorized=True)
    assert tracker.integral > .05
    command=tracker.step(-.2,-.15,-.05,1.,1.,authorized=True)
    assert command.linear == -.2 and command.angular < 0
    assert abs(tracker.integral) < .02
    assert (command.right_m_s-command.left_m_s)/.56 == pytest.approx(command.angular)


@pytest.mark.parametrize('kind',['stale','authorization','clock_reset','gap','invalid'])
def test_faults_latch_zero_until_explicit_reset(kind):
    tracker=YawRateTracker()
    tracker.step(.3,.15,.1,1.,1.,authorized=True)
    when,stamp,authorized,rate=1.05,1.05,True,.1
    if kind == 'stale':stamp=.8
    elif kind == 'authorization':authorized=False
    elif kind == 'clock_reset':when=stamp=0.
    elif kind == 'gap':when=stamp=2.
    elif kind == 'invalid':rate=math.nan
    command=tracker.step(.3,.15,rate,when,stamp,authorized=authorized)
    assert not command.valid and command.left_m_s == command.right_m_s == 0.
    assert not tracker.step(.3,.15,.1,3.,3.,authorized=True).valid
    tracker.reset()
    assert tracker.step(.3,.15,.1,3.,3.,authorized=True).valid


def test_zero_and_duplicate_time_do_not_create_extra_motion_or_integral():
    tracker=YawRateTracker()
    tracker.step(.2,.15,0.,0.,0.,authorized=True)
    tracker.step(.2,.15,0.,.05,.05,authorized=True)
    before=tracker.integral
    tracker.step(.2,.15,0.,.05,.05,authorized=True)
    assert tracker.integral == before
    stopped=tracker.step(0.,0.,.4,.1,.1,authorized=True)
    assert stopped.left_m_s == stopped.right_m_s == 0.
    assert tracker.integral == 0.


def test_20hz_imu_30hz_odometry_before_120hz_clock_interleaving():
    estimator=PlanarOdometry();tracker=YawRateTracker()
    delivered_clock=0.;pose=None;accepted=0
    quaternion=lambda yaw:(0.,0.,math.sin(yaw/2),math.cos(yaw/2))
    for tick in range(241):
        stamp=tick/120
        # IMU and wheel topics may precede the corresponding clock callback.
        # Encoder yaw deliberately overestimates the actual yaw by 4x.
        if tick%6 == 0:
            estimator.add_imu(stamp,quaternion(.2*stamp),(0.,0.,.2))
        if tick%4 == 0:
            yaw=.8*stamp;radius=.3/.8
            pose=estimator.update_wheel(stamp,radius*math.sin(yaw),radius*(1-math.cos(yaw)),quaternion(yaw))
            assert pose is not None
            command=tracker.step(.3,.2,pose.yaw_rate,delivered_clock,pose.stamp,authorized=True)
            assert command.valid,command.reason
            accepted+=1
        delivered_clock=stamp
        if pose is not None:
            command=tracker.step(.3,.2,pose.yaw_rate,delivered_clock,pose.stamp,authorized=True)
            assert command.valid,command.reason
    assert accepted == 61 and tracker.fault is None
    assert tracker.last_time == 2.
    assert pose.yaw == pytest.approx(.4)


def test_bounded_future_delivery_does_not_relax_large_future_or_stale_rejection():
    tracker=YawRateTracker()
    command=tracker.step(.3,.2,.1,.491666647,.499999980,authorized=True)
    assert command.valid
    before=tracker.integral
    assert tracker.step(.3,.2,.1,.499999980,.499999980,authorized=True).valid
    assert tracker.integral == before
    assert not tracker.step(.3,.2,.1,.5,.54,authorized=True).valid
    assert tracker.fault == 'future_yaw_rate'
    tracker.reset()
    assert not tracker.step(.3,.2,.1,.601,.5,authorized=True).valid
    assert tracker.fault == 'stale_yaw_rate'
