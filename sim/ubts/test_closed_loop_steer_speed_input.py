"""The steering controller's speed input options (closed_loop/steer_control.py speed_window_s, speed_prior_floor; 2026-11-19
corridor field data): synthetic snapshots, no plant. Every number is a test value.
"""
import math

from closed_loop.steer_control import RowFollowController, SteerFollowConfig

TABLE = ((20., .2), (40., .4), (60., .6))            # test table: 1 % duty = 0.01 m/s


def config(**changes):
    return SteerFollowConfig(**dict(dict(speed_table=TABLE), **changes))


def snap(k, speed, distance, moving=True, prior=None):
    s = dict(lidar_age_s=.08 + .05 * (k % 2), imu_age_s=.02, scan_count=10 + k // 2, speed=speed, still=not moving,
             still_s=0. if moving else 2., corridor_valid=True, corridor_reason='tracked_full_candidate',
             corridor_age_s=.1, heading_error=0., lateral_offset=0., row_end_ahead_m=None, front_clearance_m=math.inf,
             yaw_rate=0., steer_command_mrad=0, steer_measured_mrad=0., distance_m=distance)
    if prior is not None:
        s['actuator_prior'] = prior
    return s


def drive(cfg, seconds, lidar_speed, true_speed, prior=None):
    """The controller over `seconds` at 20 Hz: LiDAR speed `lidar_speed`, path distance growing at `true_speed`."""
    ctl = RowFollowController(cfg)
    out = []
    for k in range(int(seconds * 20)):
        t = k * .05
        out.append((ctl.step(t, snap(k, lidar_speed, true_speed * t, prior=prior)), ctl.telemetry()))
    return ctl, out


def test_default_is_the_lidar_speed():
    a = drive(config(), 8., .1, .35)[1]
    b = drive(config(speed_window_s=0.), 8., .1, .35)[1]
    assert [c for c, _ in a] == [c for c, _ in b]
    assert 'speed_used_m_s' not in a[-1][1]


def test_path_rate_lowers_the_duty_of_a_low_lidar_speed():
    _, low = drive(config(), 8., .1, .35)
    _, win = drive(config(speed_window_s=1.5), 8., .1, .35)
    assert win[-1][1]['speed_path_m_s'] == 0.35
    assert win[-1][1]['speed_used_m_s'] == 0.35
    assert win[-1][1]['speed_lidar_m_s'] == 0.1
    assert win[-1][0].throttle < low[-1][0].throttle        # the PI no longer winds up on the low LiDAR speed


def test_never_below_the_lidar_speed():
    _, out = drive(config(speed_window_s=1.5), 6., .3, 0.)   # the path distance stalls (a match gap)
    assert out[-1][1]['speed_path_m_s'] == 0.
    assert out[-1][1]['speed_used_m_s'] == 0.3


def test_overspeed_seen_through_the_path_rate():
    cfg = config(speed_window_s=1.5)
    _, lidar_only = drive(config(), 6., .1, .7)
    _, win = drive(cfg, 6., .1, .7)
    assert not any(c.state == 'STOP_REQUESTED' for c, _ in lidar_only)
    assert any(c.state == 'STOP_REQUESTED' and c.reason == 'stopping_overspeed' for c, _ in win)


def test_prior_floor():
    fresh = dict(fresh=True, speed_m_s=.5, predicting=False)
    _, out = drive(config(speed_prior_floor=.8), 4., .1, 0., prior=fresh)
    assert out[-1][1]['speed_prior_floor_m_s'] == 0.4
    assert out[-1][1]['speed_used_m_s'] == 0.4
    _, stale = drive(config(speed_prior_floor=.8), 4., .1, 0., prior=dict(fresh, fresh=False))
    assert stale[-1][1]['speed_used_m_s'] == 0.1


def test_limits():
    for bad in (dict(speed_window_s=-1.), dict(speed_window_s=6.), dict(speed_prior_floor=1.5)):
        try:
            config(**bad)
        except ValueError:
            continue
        raise AssertionError(bad)
