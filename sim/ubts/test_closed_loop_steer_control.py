"""Row-following steering controller (closed_loop/steer_control.py): unit tests on synthetic snapshots.

No plant, no SIM truth, no closed loop: every snapshot is written by the test. The closed-loop runs are the
pre-registered matrix (docs/steering/PREREG_STEER_FOLLOW_KO.md). Every number is a test value.
"""
import math
import types

import pytest

from closed_loop import steer_control as SC
from closed_loop.steer_control import RowFollowController, SteerFollowConfig

READ_KEYS = {'lidar_age_s', 'imu_age_s', 'scan_count', 'speed', 'still', 'still_s', 'corridor_valid', 'corridor_reason',
             'corridor_age_s', 'heading_error', 'lateral_offset', 'row_end_ahead_m', 'front_clearance_m', 'yaw_rate',
             'steer_command_mrad', 'steer_measured_mrad',
             # speed governor (PC-10): the estimator's actuator prior block, the odometry flag and the map age hook
             'actuator_prior', 'odom_valid', 'map_age_s'}
TABLE = ((20., .2), (40., .4), (60., .6))            # test table: 1 % duty = 0.01 m/s


def config(**changes):
    values = dict(speed_table=TABLE)
    values.update(changes)
    return SteerFollowConfig(**values)


class Recording(dict):
    """A snapshot that records which keys the controller reads."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.read = set()

    def get(self, key, default=None):
        self.read.add(key)
        return super().get(key, default)

    def __getitem__(self, key):
        self.read.add(key)
        return super().__getitem__(key)


def snap(k=0, **changes):
    """A fresh, valid, centred snapshot at tick k (scans every second tick, 0.08 s old at their first tick)."""
    values = dict(lidar_age_s=.08 + .05 * (k % 2), imu_age_s=.02, scan_count=10 + k // 2, speed=0., still=True,
                  still_s=2., corridor_valid=True, corridor_reason='tracked_full_candidate', corridor_age_s=.1,
                  heading_error=0., lateral_offset=0., row_end_ahead_m=None, front_clearance_m=math.inf, yaw_rate=0.,
                  steer_command_mrad=0, steer_measured_mrad=0.)
    values.update(changes)
    return Recording(values)


def run(ctl, ticks, start=0, **changes):
    """Step the controller for ticks at 20 Hz with the same snapshot values; returns the commands."""
    out = []
    for k in range(start, start + ticks):
        out.append(ctl.step(k * .05, snap(k, **changes)))
    return out


def driving(ctl, speed=.4, **changes):
    """A controller that has finished the alignment and drives (returns the next tick index)."""
    run(ctl, 3, **changes)
    assert ctl.drive_t0 is not None
    return 3


# --- configuration ----------------------------------------------------------------------------------------------------

def test_defaults_are_the_design_placeholders():
    cfg = SteerFollowConfig()
    assert (cfg.cruise_speed_m_s, cfg.min_speed_m_s, cfg.max_speed_m_s) == (.40, .20, .50)        # design v2 V2-7
    assert (cfg.lookahead_base_m, cfg.lookahead_per_speed_s, cfg.lookahead_max_m) == (1.5, 1.25, 3.)
    assert (cfg.k_d, cfg.k_psi, cfg.k_r) == (1., 1., 0.) and cfg.trim_limit_deg == 3.
    assert (cfg.d_scan_m, cfg.d_blind_m, cfg.front_margin_m, cfg.stop_decel_m_s2) == (.05, .15, .35, 1.07)
    assert cfg.end_stop_offset_m == .61 and cfg.steer_follow_limit_deg == 12.
    assert (cfg.duty_rise_pct_s, cfg.duty_fall_pct_s) == (5., 15.)


@pytest.mark.parametrize('changes', (
    dict(min_speed_m_s=.5), dict(cruise_speed_m_s=.6), dict(approach_speed_m_s=.1), dict(lookahead_base_m=4.),
    dict(rate_margin=1.2), dict(steer_follow_limit_deg=30.), dict(wheelbase_m=0.), dict(d_scan_m=0.),
    dict(lateral_slow_full_m=.1), dict(duty_max_pct=90), dict(speed_table=((1., 1.),)), dict(k_d=math.nan),
    dict(rate_standstill_deg_s=40.),
))
def test_config_refuses_bad_values(changes):
    with pytest.raises(ValueError):
        config(**changes)


def test_factory_takes_the_limits_of_the_executor_configuration():
    executor = types.SimpleNamespace(steer_rate_limit_deg_s=20., steer_rate_limit_standstill_deg_s=10.,
                                     steer_limit_left_mrad=400, steer_limit_right_mrad=380, lidar_max_age_s=.45,
                                     imu_max_age_s=.4, duty_cap_pct=75)
    factory = SC.make_factory({'kind': 'row_follow', 'cruise_speed_m_s': .3, 'speed_table': [[20., .2], [60., .6]],
                               'rate_deg_s': 99.}, executor)
    a, b = factory(), factory()
    assert isinstance(a, RowFollowController) and a is not b
    assert (a.config.rate_deg_s, a.config.rate_standstill_deg_s) == (20., 10.)        # never the spec's own limit
    assert (a.config.limit_left_mrad, a.config.limit_right_mrad, a.config.cruise_speed_m_s) == (400., 380., .3)
    assert a.config.speed_table == ((20., .2), (60., .6))
    with pytest.raises(ValueError, match='unknown row_follow settings'):
        SC.make_factory({'kind': 'row_follow', 'gain': 1.}, executor)
    from steering import scripted_control
    assert isinstance(scripted_control.make_factory({'kind': 'row_follow'}, executor)(), RowFollowController)
    assert 'row_follow' in scripted_control.KINDS


# --- start ------------------------------------------------------------------------------------------------------------

def test_alignment_waits_for_the_corridor_then_gives_up_with_a_reason():
    ctl = RowFollowController(config())
    out = run(ctl, 122, corridor_valid=False)             # acquire_timeout_s 6 s: the first tick after it
    assert all(c.state == 'FOLLOW' and c.throttle == 0 and c.steer_mrad == 0 for c in out[:121])
    assert out[60].reason == 'align_wait_corridor'
    assert (out[121].state, out[121].reason, out[121].steer_mrad) == ('HOLD', 'corridor_not_acquired', None)
    assert ctl.pending == ('hold', 'corridor_not_acquired')


def test_alignment_moves_the_steering_at_standstill_before_driving():
    ctl = RowFollowController(config())
    out = run(ctl, 60, lateral_offset=-.4, steer_measured_mrad=None)     # right of the centre: steer left
    throttles = [c.throttle for c in out]
    first_drive = next(i for i, v in enumerate(throttles) if v > 0)
    assert first_drive >= 5 and all(v == 0 for v in throttles[:first_drive])
    assert out[first_drive - 1].steer_mrad > 60           # the steering moved before the drive started
    steps = [abs(b.steer_mrad - a.steer_mrad) for a, b in zip(out[:first_drive - 1], out[1:first_drive])]
    assert max(steps) <= math.radians(12.) * 1000 * .75 * .05 + 1       # standstill slew x margin
    assert ctl.phase_events[0][1] == 'ALIGN' and ctl.phase_events[1][1] == 'FOLLOW'
    stuck = RowFollowController(config())                 # the measured steering never follows: align_timeout_s
    out = run(stuck, 100, lateral_offset=-.4, steer_measured_mrad=0.)
    first_drive = next(i for i, c in enumerate(out) if c.throttle > 0)
    assert first_drive * .05 == pytest.approx(4.05, abs=.06)


# --- steering ---------------------------------------------------------------------------------------------------------

def target(ctl, **changes):
    return ctl._steer_target(snap(**changes), .4, .05, False)[0]


def test_steering_law_is_the_lookahead_point_of_the_centre_line():
    ctl = RowFollowController(config(trim_gain_rad_per_m_s=0.))
    lookahead = 1.5 + 1.25 * .4                           # 2.0 m at 0.40 m/s
    for lateral, heading in ((.2, 0.), (-.2, 0.), (0., .05), (0., -.05), (.1, .03)):
        y = -lateral + lookahead * math.sin(heading)
        want = math.atan(.76 * 2. * y / lookahead ** 2) * 1000.
        assert target(ctl, lateral_offset=lateral, heading_error=heading) == pytest.approx(want, abs=1e-9)
    assert target(ctl, lateral_offset=.2) < 0 < target(ctl, lateral_offset=-.2)       # left of centre: steer right
    assert target(ctl, heading_error=.05) > 0                                         # corridor axis to the left
    assert target(ctl, lateral_offset=3.) == pytest.approx(-math.radians(12.) * 1000.)     # follow limit
    assert target(ctl, corridor_valid=False) is None
    gains = RowFollowController(config(trim_gain_rad_per_m_s=0., k_d=2., k_psi=.5))
    y = -2. * .1 + .5 * lookahead * math.sin(.03)
    assert target(gains, lateral_offset=.1, heading_error=.03) == pytest.approx(
        math.atan(.76 * 2. * y / lookahead ** 2) * 1000., abs=1e-9)


def test_steering_command_is_slew_limited_inside_the_soft_limit_and_an_int():
    ctl = RowFollowController(config(steer_filter_s=0.))
    k = driving(ctl)
    out = run(ctl, 40, start=k, lateral_offset=-2., speed=.4, still=False)
    values = [c.steer_mrad for c in out]
    assert all(type(v) is int for v in values)
    steps = [b - a for a, b in zip(values, values[1:])]
    assert max(steps) <= math.radians(24.) * 1000 * .75 * .05 + 1       # moving slew x margin per 50 ms
    assert max(values) == pytest.approx(math.radians(12.) * 1000, abs=1.)
    assert max(values) < .9 * 409


def test_trim_integrates_slowly_and_is_bounded():
    ctl = RowFollowController(config())
    k = driving(ctl)
    run(ctl, 40, start=k, lateral_offset=.1, speed=.4, still=False)        # 2 s with y_L = -0.1 m
    assert ctl.trim == pytest.approx(-.05 * .1 * 2., rel=.1)
    run(ctl, 4000, start=k + 40, lateral_offset=.1, speed=.4, still=False)
    assert ctl.trim == pytest.approx(-math.radians(3.))
    still = RowFollowController(config())
    run(still, 2, lateral_offset=.3)
    assert still.trim == 0.                                # no trim during the alignment


# --- speed ------------------------------------------------------------------------------------------------------------

def reference(ctl, t_scan=.1, tau=.3, remaining=None, **changes):
    return ctl._speed_reference(snap(**changes), t_scan, tau, remaining)


def test_speed_caps():
    ctl = RowFollowController(config())
    v, bound, caps = reference(ctl)
    assert (v, bound) == (.40, 'cruise') and caps['scan'] == pytest.approx(.5) and caps['blind'] == pytest.approx(.5)
    assert reference(ctl, t_scan=.2)[:2] == (pytest.approx(.25), 'scan')              # a scan missing: 0.05 m / 0.2 s
    assert reference(ctl, tau=.5)[:2] == (pytest.approx(.30), 'blind')                # 0.15 m / 0.5 s
    assert reference(ctl, lateral_offset=.4)[:2] == (pytest.approx(.20), 'lateral')
    assert reference(ctl, lateral_offset=.275)[0] == pytest.approx(.30)
    assert reference(ctl, corridor_reason='tracked_one_side_guided')[:2] == (.20, 'quality')
    assert reference(ctl, corridor_age_s=.8)[:2] == (.20, 'quality')
    assert reference(ctl, corridor_valid=False)[:2] == (.20, 'quality')
    assert reference(ctl, remaining=1.6)[:2] == (.20, 'end')
    assert reference(ctl, remaining=3.)[1] == 'cruise'
    v, bound, _ = reference(ctl, front_clearance_m=.6)                                # room 0.25 m
    assert bound == 'stop' and v * (.3 + .25) + v * v / (2 * 1.07) == pytest.approx(.25)
    assert reference(ctl, front_clearance_m=.3)[:2] == (0., 'stop')
    fast = RowFollowController(config(cruise_speed_m_s=.5))
    assert reference(fast)[0] == .5 and RowFollowController(config()).stopping_distance(.4) == pytest.approx(
        .4 * .25 + .16 / 2.14)


def test_governor_caps_of_pc10():
    """Every new cap of the speed governor (design v2 V2-4, PC-10) on its own, and its name as the limit reason."""
    ctl = RowFollowController(config())
    assert 'map' not in reference(ctl)[2]                                              # hook off without a map age
    assert reference(ctl, map_age_s=.8)[:2] == (pytest.approx(.25), 'map')            # 0.20 m / 0.8 s
    ctl.curvature = .3 / .35 ** 2                                                     # sqrt(0.3 / k) = 0.35 m/s
    assert reference(ctl)[:2] == (pytest.approx(.35), 'curvature')
    ctl.curvature = 0.
    ctl.demand = .9                                                                   # half way from 0.8 to 1.0
    assert reference(ctl)[:2] == (pytest.approx(.30), 'steer')
    ctl.demand = 1.3
    assert reference(ctl)[:2] == (pytest.approx(.20), 'steer')
    ctl.demand = .8
    assert reference(ctl)[1] == 'cruise'
    ctl.demand = 0.
    assert reference(ctl, lidar_age_s=.31)[:2] == (.20, 'fresh')
    assert reference(ctl, lidar_age_s=.29)[1] == 'cruise'
    assert reference(ctl, actuator_prior=dict(predicting=True))[:2] == (.20, 'quality')
    assert reference(ctl, actuator_prior=dict(predicting=False))[1] == 'cruise'


def test_steering_demand_and_curvature_come_from_the_steering_target():
    ctl = RowFollowController(config())
    k = driving(ctl)
    run(ctl, 20, start=k, speed=.4, still=False, heading_error=.3)          # no lateral offset: no lateral cap
    assert ctl.demand > 1. and ctl.curvature == pytest.approx(math.tan(math.radians(12.)) / .76)
    assert ctl._telemetry['steer_demand'] > 1. and ctl._telemetry['cap'] == 'steer'
    assert ctl._telemetry['caps']['curvature'] == pytest.approx(math.sqrt(.3 / ctl.curvature), abs=1e-3)


def test_late_ticks_lengthen_tau_loop():
    ctl = RowFollowController(config())
    k = driving(ctl)
    t = k * .05
    for i in range(40):                                    # the controller steps every 0.1 s (every second tick lost)
        t += .1
        ctl.step(t, snap(0, scan_count=20 + i, lidar_age_s=.08, speed=.4, still=False))
    assert ctl._telemetry['tau_loop_s'] == pytest.approx(.08 + .24 + .05)
    assert ctl.caps['blind'] == pytest.approx(.15 / (.08 + .24 + .05), abs=1e-3)


def test_overspeed_cuts_the_duty_then_pauses():
    ctl = RowFollowController(config())
    k = driving(ctl)
    run(ctl, 40, start=k, speed=.4, still=False)
    c = ctl.step((k + 40) * .05, snap(0, speed=.6, still=False))
    assert c.state == 'FOLLOW' and c.throttle == 20 and ctl._telemetry['overspeed'] is True    # table^-1(0.2) = 20 %
    out = run(ctl, 25, start=k + 41, speed=.6, still=False)
    assert ('STOP_REQUESTED', 'stopping_overspeed') in [(c.state, c.reason) for c in out]
    assert ctl.pending == ('hold', 'overspeed')
    c = ctl.step(10., snap(0, speed=0., still=True, still_s=1.))
    assert (c.state, c.reason, c.recoverable) == ('HOLD', 'overspeed', True)
    again = RowFollowController(config())
    k = driving(again)
    out = run(again, 10, start=k, speed=.6, still=False) + run(again, 30, start=k + 10, speed=.4, still=False)
    assert again.pending is None and again.over_since is None                         # a short excess only slows


def test_recoverable_flag_of_the_holds():
    assert set(SC.RECOVERABLE_HOLDS) == {'speed_cap_below_min', 'overspeed', 'stale_lidar', 'stale_imu'}
    ctl = RowFollowController(config())
    k = driving(ctl)
    c = ctl.step(k * .05, snap(k, lidar_age_s=.5, speed=.4, still=False))
    assert (c.state, c.reason, c.recoverable) == ('HOLD', 'stale_lidar', True)
    end = RowFollowController(config())
    k = driving(end)
    run(end, 5, start=k, speed=.4, still=False, row_end_ahead_m=.7)
    c = end.step(9., snap(0, speed=0., still=True, still_s=1., row_end_ahead_m=.7))
    assert (c.state, c.reason, c.recoverable) == ('HOLD', 'row_end', False)


def test_loop_timing_is_measured_from_the_scan_stamps():
    ctl = RowFollowController(config())
    k = driving(ctl)
    run(ctl, 40, start=k, speed=.4, still=False)
    assert ctl._telemetry['t_scan_s'] == pytest.approx(.1) and ctl._telemetry['tau_loop_s'] == pytest.approx(.32)
    assert ctl._telemetry['cap'] == 'cruise'
    slow = RowFollowController(config())
    for i in range(80):                                    # scans every 4th tick (0.2 s apart)
        slow.step(i * .05, snap(0, scan_count=10 + i // 4, lidar_age_s=.08 + .05 * (i % 4), speed=.25, still=False))
    assert slow._telemetry['t_scan_s'] == pytest.approx(.2) and slow._telemetry['cap'] == 'scan'
    assert slow._telemetry['target_speed_m_s'] == pytest.approx(.25)


def test_one_missing_scan_does_not_lower_the_speed_but_a_run_of_them_stops():
    ctl = RowFollowController(config())
    k = driving(ctl)
    run(ctl, 40, start=k, speed=.4, still=False)
    ages = [.08, .13, .18, .23, .28, .33]                  # one gap of 0.3 s, then scans again
    for i, age in enumerate(ages):
        ctl.step((k + 40 + i) * .05, snap(0, scan_count=10 + (k + 39) // 2, lidar_age_s=age, speed=.4, still=False))
    out = run(ctl, 10, start=k + 46, speed=.4, still=False)
    assert all(c.state == 'FOLLOW' and c.throttle > 0 for c in out) and ctl._telemetry['cap'] == 'cruise'
    slow = RowFollowController(config())
    reasons = []
    for i in range(120):                                   # every scan 0.3 s apart: 0.05 / 0.3 < the minimum speed
        c = slow.step(i * .05, snap(0, scan_count=10 + i // 6, lidar_age_s=.08 + .05 * (i % 6), speed=.2, still=False))
        reasons.append(c.reason)
    assert 'stopping_speed_cap_below_min' in reasons and slow.pending == ('hold', 'speed_cap_below_min')


def test_throttle_starts_at_the_minimum_speed_duty_and_slews():
    ctl = RowFollowController(config())
    k = driving(ctl)
    out = run(ctl, 100, start=k, speed=0., still=False)    # no speed measured: the PI asks for more
    duties = [c.throttle for c in out]
    assert duties[0] == 20                                 # table^-1(0.20 m/s)
    assert all(type(v) is int for v in duties)
    assert max(b - a for a, b in zip(duties, duties[1:])) <= 1          # 5 %/s at 20 Hz
    assert duties[-1] == pytest.approx(20 + 5 * 5, abs=1)               # 5 s of rise
    run(ctl, 2000, start=k + 100, speed=0., still=False)
    assert ctl.last_throttle == 40 + 12 + 15               # feedforward + kp x 0.4 m/s + the integral limit (<= 70)
    capped = RowFollowController(config(speed_kp_pct_s_per_m=200.))
    k = driving(capped)
    run(capped, 2000, start=k, speed=0., still=False)
    assert capped.last_throttle == 70                      # duty_max_pct
    fast = RowFollowController(config())
    k = driving(fast)
    run(fast, 100, start=k, speed=.4, still=False)
    high = fast.last_throttle
    run(fast, 40, start=k + 100, speed=.7, still=False)    # too fast: the duty falls (up to 15 %/s)
    assert fast.last_throttle < high - 10


def test_the_duty_never_rises_while_faster_than_the_reference():
    # S1c: the start ramp of a vehicle faster than the table (table x 1.3) must stop rising once the speed passes v_ref
    ctl = RowFollowController(config())
    k = driving(ctl)
    run(ctl, 20, start=k, speed=.1, still=False)                  # still slow: the ramp rises
    rising = ctl.last_throttle
    out = run(ctl, 60, start=k + 20, speed=.45, still=False)      # faster than 0.40: no rise any more
    duties = [c.throttle for c in out]
    assert max(duties) <= rising                                  # held (it falls once the PI asks for less)
    assert ctl.integral < 0.                                      # the integral moves down while the rise is held


# --- stops ------------------------------------------------------------------------------------------------------------

def test_row_end_stop_and_hold():
    ctl = RowFollowController(config())
    k = driving(ctl)
    out = run(ctl, 20, start=k, speed=.2, still=False, row_end_ahead_m=2.2)
    assert out[-1].reason == 'follow_end' and ctl.current_phase() == 'APPROACH_END'
    assert ctl._telemetry['end_remaining_m'] == pytest.approx(2.2 - .61)
    c = ctl.step((k + 20) * .05, snap(k, speed=.2, still=False, row_end_ahead_m=.61 + .06))
    assert (c.state, c.reason, c.throttle, c.steer_mrad) == ('STOP_REQUESTED', 'stopping_row_end', 0, None)
    assert ctl.pending == ('hold', 'row_end')
    c = ctl.step((k + 21) * .05, snap(k, speed=.1, still=False))
    assert c.state == 'STOP_REQUESTED'                     # the decision is kept although the end is not posted now
    c = ctl.step((k + 40) * .05, snap(k, speed=0., still=True, still_s=.5))
    assert (c.state, c.reason, c.throttle, c.steer_mrad) == ('HOLD', 'row_end', 0, None)
    assert ctl.counts == dict(stops=1, holds=1) and ctl.current_phase() == 'STOPPED'


def test_row_end_far_ahead_does_not_stop():
    ctl = RowFollowController(config())
    k = driving(ctl)
    out = run(ctl, 40, start=k, speed=.4, still=False, row_end_ahead_m=6.)
    assert all(c.state == 'FOLLOW' and c.throttle > 0 for c in out) and ctl._telemetry['cap'] == 'cruise'


def test_obstacle_ahead_stops_before_the_margin():
    ctl = RowFollowController(config())
    k = driving(ctl)
    out = run(ctl, 5, start=k, speed=.3, still=False, front_clearance_m=2.)
    assert all(c.state == 'FOLLOW' for c in out)
    c = ctl.step((k + 5) * .05, snap(k, speed=.3, still=False, front_clearance_m=.45))
    assert (c.state, c.reason) == ('STOP_REQUESTED', 'stopping_obstacle_ahead')


def test_corridor_lost_only_after_the_allowed_time():
    ctl = RowFollowController(config())
    k = driving(ctl)
    run(ctl, 20, start=k, speed=.4, still=False, lateral_offset=-.3)
    held = ctl.last_throttle
    steer = None
    out = run(ctl, 20, start=k + 20, speed=.4, still=False, corridor_valid=False)      # 1.0 s without a corridor
    assert all(c.state == 'FOLLOW' and c.throttle > 0 for c in out)
    assert out[-1].reason == 'follow_quality' and out[-1].throttle <= held
    steer = out[0].steer_mrad
    assert all(abs(c.steer_mrad - steer) <= 1 for c in out)                            # the steering is kept
    c = ctl.step((k + 41) * .05, snap(k, speed=.4, still=False, corridor_valid=False))
    assert (c.state, c.reason) == ('STOP_REQUESTED', 'stopping_corridor_lost')
    again = RowFollowController(config())
    k = driving(again)
    run(again, 10, start=k, speed=.4, still=False, corridor_valid=False)
    out = run(again, 30, start=k + 10, speed=.4, still=False)                          # back within the time
    assert all(c.state == 'FOLLOW' for c in out) and again.pending is None


def test_stale_input_is_a_hold_at_once():
    for changes, reason in ((dict(lidar_age_s=.5), 'stale_lidar'), (dict(imu_age_s=.45), 'stale_imu'),
                            (dict(lidar_age_s=None), 'stale_lidar'), (dict(imu_age_s=math.inf), 'stale_imu')):
        ctl = RowFollowController(config())
        k = driving(ctl)
        run(ctl, 5, start=k, speed=.4, still=False)
        c = ctl.step((k + 5) * .05, snap(k, speed=.4, still=False, **changes))
        assert (c.state, c.reason, c.throttle, c.steer_mrad) == ('HOLD', reason, 0, None)


def test_only_the_documented_snapshot_keys_are_read_and_commands_are_valid():
    ctl = RowFollowController(config())
    read = set()
    for k in range(200):
        s = snap(k, speed=.3, still=False, lateral_offset=.1 * math.sin(k / 20.), heading_error=.02,
                 row_end_ahead_m=None if k < 150 else 2.5 - .02 * (k - 150), front_clearance_m=5.,
                 plant_x=1., truth_y=2.)
        c = ctl.step(k * .05, s)
        read |= s.read
        assert c.state in ('FOLLOW', 'STOP_REQUESTED', 'HOLD') and c.left == c.right == c.throttle
        assert 0 <= c.throttle <= 70 and (c.steer_mrad is None or type(c.steer_mrad) is int)
        if c.state != 'FOLLOW':
            assert c.throttle == 0 and c.steer_mrad is None
    assert read <= READ_KEYS
    assert set(SC.HOLD_REASONS) >= {'row_end', 'obstacle_ahead', 'corridor_lost', 'corridor_not_acquired'}
    assert set(ctl.telemetry()) >= {'phase'}
