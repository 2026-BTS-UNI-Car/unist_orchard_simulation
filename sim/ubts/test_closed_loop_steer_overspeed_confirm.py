"""The overspeed duty cut confirmation (closed_loop/steer_control.py overspeed_confirm_s; 2026-11-18/19 corridor tick logs):
synthetic snapshots, no plant. Every number is a test value.
"""
import math

from closed_loop.steer_control import RowFollowController, SteerFollowConfig

TABLE = ((20., .2), (40., .4), (60., .6))            # test table: 1 % duty = 0.01 m/s; the minimum speed 0.2 -> 20 %


def config(**changes):
    return SteerFollowConfig(**dict(dict(speed_table=TABLE), **changes))


def snap(k, speed, distance):
    return dict(lidar_age_s=.08 + .05 * (k % 2), imu_age_s=.02, scan_count=10 + k // 2, speed=speed, still=False,
                still_s=0., corridor_valid=True, corridor_reason='tracked_full_candidate', corridor_age_s=.1,
                heading_error=0., lateral_offset=0., row_end_ahead_m=None, front_clearance_m=math.inf, yaw_rate=0.,
                steer_command_mrad=0, steer_measured_mrad=0., distance_m=distance)


def drive(cfg, speeds):
    """The controller at 20 Hz; speeds(t) -> LiDAR speed. Returns [(t, command, telemetry)]."""
    ctl = RowFollowController(cfg)
    out, distance = [], 0.
    for k in range(400):
        t = k * .05
        v = speeds(t)
        if v is None:
            break
        distance += v * .05
        command = ctl.step(t, snap(k, v, distance))
        out.append((t, command, ctl.telemetry()))
    return out


def cruise_then(spike_at, spike_s, spike_v, end=12.):
    def speeds(t):
        if t >= end:
            return None
        return spike_v if spike_at <= t < spike_at + spike_s else .3
    return speeds


def duty_at(out, t):
    return next(c.throttle for tt, c, _ in out if tt >= t - 1e-9)


def test_default_is_off_and_cuts_at_once():
    assert SteerFollowConfig.__dataclass_fields__['overspeed_confirm_s'].default == 0.
    a = drive(config(), cruise_then(8., .1, .65))
    b = drive(config(overspeed_confirm_s=0.), cruise_then(8., .1, .65))
    assert [c for _, c, _ in a] == [c for _, c, _ in b]
    before = duty_at(a, 7.95)
    assert before > 25
    assert duty_at(a, 8.0) == 20                       # the table duty of the minimum speed, no slew


def test_a_one_scan_spike_is_not_cut_with_confirmation():
    out = drive(config(overspeed_confirm_s=.3), cruise_then(8., .1, .65))
    before = duty_at(out, 7.95)
    during = min(c.throttle for t, c, _ in out if 8. <= t < 8.3)
    assert during >= before - 2                        # the PI fall rate only (15 %/s x 0.1 s), no cut to 20
    assert not any(c.state == 'STOP_REQUESTED' for _, c, _ in out)


def test_a_lasting_overspeed_is_cut_after_the_confirmation_and_stops_as_before():
    lasting = cruise_then(8., 10., .7)
    plain = drive(config(), lasting)
    confirm = drive(config(overspeed_confirm_s=.3), lasting)
    assert duty_at(plain, 8.0) == 20
    assert duty_at(confirm, 8.2) > 20                  # not yet confirmed
    assert duty_at(confirm, 8.35) == 20                # cut once 0.3 s above the line
    stop_plain = next(t for t, c, _ in plain if c.state == 'STOP_REQUESTED')
    stop_confirm = next(t for t, c, _ in confirm if c.state == 'STOP_REQUESTED')
    assert stop_plain == stop_confirm                  # the overspeed stop (overspeed_s from the first tick) unchanged
    assert next(c.reason for t, c, _ in confirm if c.state == 'STOP_REQUESTED') == 'stopping_overspeed'


def test_limits():
    for bad in (dict(overspeed_confirm_s=-.1), dict(overspeed_confirm_s=1.), dict(overspeed_confirm_s=1.5)):
        try:
            config(**bad)
        except ValueError:
            continue
        raise AssertionError(bad)
    config(overspeed_confirm_s=.5)
