"""Contract of the single-row controller (deterministic synthetic snapshots; SIM evaluation only).

Snapshots stand in for SensorEstimator output. The closed-loop tests use a tiny kinematic stand-in
(_Kinematic: yaw rate = 2 g d + drift through a first-order lag, speed = slope x (mean duty - deadband)),
NOT the SIM FieldPlant; one smoke test runs the real 2D harness. Run from the repository root:
    python -m pytest sim/ubts/closed_loop/test_single_row.py
"""
import math
import random

import pytest

from closed_loop.controllers import TERMINAL_HOLDS as ROW_TERMINAL_HOLDS, ControlConfig, RowController, config_for
from closed_loop.single_row import (CATEGORIES, STOP_CATEGORIES, TERMINAL_HOLDS, SingleRowConfig,
                                    SingleRowController, make_single_row_config, single_row_config_from,
                                    stop_category)

TICK = .05


def deg(value):
    return math.radians(value)


def snap(**overrides):
    s = dict(lidar_age_s=.05, imu_age_s=.02, still=False, still_s=0., corridor_valid=True, corridor_age_s=.1,
             corridor_updates=10, heading_error=0., lateral_offset=0., corridor_width=3.5, front_clearance_m=10.,
             speed=.4, odom_valid=True, row_end_ahead_m=None, side_ends_ahead_m=[None, None],
             side_end_free=[False, False], end_age_s=None, yaw_rate=0., distance_m=0., pivot_clearance_m=3.,
             pivot_radius_required_m=1.2, gyro_heading=0., vibration_rms=.5)
    s.update(overrides)
    if overrides.get('still') and 'still_s' not in overrides:
        s['still_s'] = 2.
    return s


def run(controller, start, seconds, **overrides):
    commands, t = [], start
    for _ in range(int(round(seconds / TICK))):
        t = round(t + TICK, 6)
        commands.append(controller.step(t, snap(**overrides)))
    return t, commands


def following(**config):
    controller = SingleRowController(make_single_row_config(**config))
    command = controller.step(0., snap(still=True, speed=0.))
    assert controller.state == 'FOLLOW', controller.events
    return controller, command


class _Kinematic:
    """Kinematic stand-in for closed-loop contract tests (not the SIM plant; no noise).

    State: along x, lateral y (left +), heading psi relative to the row (CCW +), speed v, yaw rate r
    [deg/s]. r -> 2 g (right - left) / 2 + drift(t) with lag tau_yaw; v -> slope x scale x (mean - deadband)
    with lag tau_v. The snapshot reports y, -psi, r (rad/s), v, still and the row end (end_x - x)."""

    def __init__(self, g=.12, drift=lambda t: 0., slope=.018, deadband=50., scale=1., y=0., psi_deg=0.,
                 end_x=None, end_visible_m=8., tau_yaw=.12, tau_v=.25):
        self.g, self.drift, self.slope, self.deadband, self.scale = g, drift, slope, deadband, scale
        self.x, self.y, self.psi, self.v, self.r = 0., y, deg(psi_deg), 0., 0.
        self.end_x, self.end_visible_m, self.tau_yaw, self.tau_v = end_x, end_visible_m, tau_yaw, tau_v
        self.still_s = 1.
        self.command = (0, 0)
        self.track = []               # along position after each recorded tick

    def step(self, t, dt=TICK):
        left, right = self.command
        mean, diff = (left + right) / 2., (right - left) / 2.
        v_target = max(0., self.slope * self.scale * (mean - self.deadband))
        r_target = 2 * self.g * diff if v_target > 0 else 0.
        r_target += self.drift(t) if v_target > 0 else 0.
        self.v += (v_target - self.v) * (1 - math.exp(-dt / self.tau_v))
        if v_target == 0 and self.v < .003:
            self.v = 0.
        self.r += (r_target - self.r) * (1 - math.exp(-dt / self.tau_yaw))
        self.psi += math.radians(self.r) * dt
        self.x += self.v * math.cos(self.psi) * dt
        self.y += self.v * math.sin(self.psi) * dt
        self.still_s = self.still_s + dt if self.v < .005 and abs(self.r) < .2 else 0.

    def snapshot(self, **extra):
        end = None
        if self.end_x is not None and self.end_x - self.x <= self.end_visible_m:
            end = self.end_x - self.x
        values = dict(lateral_offset=self.y, heading_error=-self.psi, yaw_rate=math.radians(self.r), speed=self.v,
                      still=self.still_s > 0, still_s=self.still_s, row_end_ahead_m=end, distance_m=self.x)
        values.update(extra)
        return snap(**values)


def closed_loop(controller, plant, seconds, start=0., record=None):
    t = start
    for _ in range(int(round(seconds / TICK))):
        t = round(t + TICK, 6)
        command = controller.step(t, plant.snapshot())
        plant.command = (command.left, command.right)
        plant.step(t)
        if record is not None:
            record.append((t, command, dict(controller.telemetry()), plant.y))
            plant.track.append(plant.x)
    return t


# --- configuration, categories ------------------------------------------------------------------------------
def test_config_defaults_and_validation():
    c = SingleRowConfig()
    assert c.mode == 'continuous' and c.side_clearance_check and not c.pivot_failure_fallback
    assert c.max_yaw_rate_deg_s <= .6 * 2 * c.arc_yaw_deg_s_per_pct_diff * c.max_differential_pct + 1e-9
    assert c.base_max_pct + c.max_differential_pct <= 100 and c.base_min_pct - c.max_differential_pct >= 0
    assert c.start_max_deg < c.heading_limit_deg
    for bad in (dict(mode='stop_turn'), dict(mode='hybrid'), dict(pivot_failure_fallback=True),
                dict(row_end_stop_m=.5), dict(base_min_pct=74, base_duty_pct=72), dict(base_max_pct=80),
                dict(max_yaw_rate_deg_s=4.5), dict(bias_max_pct=30.), dict(start_max_deg=30.),
                dict(cruise_speed_m_s=0.), dict(output_deadband_pct=.5), dict(sign_hysteresis_pct=20.)):
        with pytest.raises(ValueError):
            make_single_row_config(**bad)


def test_conversion_from_a_control_config_carries_only_changed_fields():
    converted = single_row_config_from(config_for('continuous'))
    assert converted == SingleRowConfig()                   # RowController tuning (base 75, yaw 4.5) not carried
    converted = single_row_config_from(config_for('continuous_side', base_duty_pct=70, lidar_max_age_s=.4))
    assert converted.base_duty_pct == 70 and converted.lidar_max_age_s == .4 and converted.side_clearance_check
    assert SingleRowController(config_for('continuous')).config == SingleRowConfig()   # harness factory path
    with pytest.raises(ValueError):
        single_row_config_from(config_for('hybrid'))
    with pytest.raises(TypeError):
        single_row_config_from(dict(mode='continuous'))


def test_every_terminal_reason_maps_to_one_category():
    for reason in TERMINAL_HOLDS + ROW_TERMINAL_HOLDS + ('stale_lidar', 'stale_imu'):
        assert stop_category(reason) in CATEGORIES, reason
    assert set(STOP_CATEGORIES.values()) <= set(CATEGORIES)
    assert stop_category('row_end') == 'ROW_END' and stop_category('obstacle_ahead') == 'OBSTACLE'
    assert stop_category('end_structure_ahead') == 'END_STRUCTURE'
    assert stop_category('steering_authority_insufficient') == 'CONTROL_AUTHORITY'
    assert stop_category('corridor_not_reacquired') == 'PERCEPTION_OR_LOCALISATION'
    assert stop_category('drive_no_response') == 'DRIVE' and stop_category('sensor_missing') == 'SENSOR'
    assert stop_category('row_distance_backstop') == 'OPERATOR_OR_BACKSTOP'
    assert stop_category('guard_violation:duty_above_cap') == 'CONTROL_AUTHORITY'
    assert stop_category('FOLLOW') is None and stop_category(None) is None


# --- start ----------------------------------------------------------------------------------------------------
def test_start_waits_like_reacquire_then_follows_forward():
    controller = SingleRowController()
    command = controller.step(0., snap(lidar_age_s=math.inf, imu_age_s=math.inf, corridor_valid=False))
    assert (command.left, command.right, controller.state) == (0, 0, 'REACQUIRE')
    assert controller.telemetry()['phase'] == 'START'
    command = controller.step(.05, snap(still=False))             # not seen still yet
    assert (command.left, command.right) == (0, 0)
    command = controller.step(.1, snap(still=True, speed=0.))
    assert controller.state == 'FOLLOW' and command.left > 0 and command.right > 0
    assert abs(command.left - command.right) <= 2


def test_initial_start_needs_a_longer_standstill_than_a_resume():
    controller = SingleRowController()
    run(controller, 0., .5, still=True, still_s=1., speed=0.)          # still 1.0 s < start_still_s 1.5 s
    assert controller.state == 'REACQUIRE'
    controller.step(.55, snap(still=True, still_s=1.5, speed=0.))
    assert controller.state == 'FOLLOW'


def test_rolling_start_equal_duty_until_the_sensors_see_motion():
    controller = SingleRowController()
    command = controller.step(0., snap(still=True, speed=0., heading_error=deg(10.)))
    assert controller.state == 'FOLLOW' and command.left == command.right and command.reason == 'rolling_start'
    t, commands = run(controller, 0., .5, still=True, still_s=1., speed=0., heading_error=deg(10.))
    assert all(c.left == c.right for c in commands) and controller.bias_pct == 0.
    t, commands = run(controller, t, .5, speed=.02, heading_error=deg(10.))       # not still, but not rolling yet
    assert all(c.left == c.right for c in commands)
    t, commands = run(controller, t, .5, speed=.2, heading_error=deg(10.))
    assert commands[-1].right > commands[-1].left and commands[-1].reason == 'single_row_follow'


@pytest.mark.parametrize('heading, lateral, refused', [(30., 0., True), (-26., 0., True), (20., -.4, True),
                                                      (20., .4, False), (24., 0., False), (5., .75, True)])
def test_start_misaligned_is_a_terminal_refusal(heading, lateral, refused):
    # lookahead error at the 3 m start lookahead: 20 deg - atan(-0.4 / 3) = 27.6 deg (refused), 20 - 7.6 = 12.4 (ok);
    # 0.75 m is outside the lateral band (0.5 x (3.5 / 2 - 0.35) = 0.70 m)
    for lookahead in (1.5, 3.):                                   # independent of the steering lookahead
        controller = SingleRowController(make_single_row_config(lookahead_m=lookahead))
        command = controller.step(0., snap(still=True, speed=0., heading_error=deg(heading), lateral_offset=lateral))
        if refused:
            assert controller.state == 'HOLD' and controller.reason == 'start_misaligned' and controller.hold_terminal
            assert controller.stop_category == 'CONTROL_AUTHORITY' and (command.left, command.right) == (0, 0)
            t, commands = run(controller, 0., 2., still=True, speed=0.)
            assert all(c.left == c.right == 0 for c in commands)
        else:
            assert controller.state == 'FOLLOW'


def test_start_refused_for_a_blocked_front_or_side_or_a_reached_end():
    controller = SingleRowController()
    controller.step(0., snap(still=True, speed=0., front_clearance_m=.2))
    assert controller.reason == 'obstacle_ahead' and controller.hold_terminal
    controller = SingleRowController()
    controller.step(0., snap(still=True, speed=0., row_end_ahead_m=.4))   # remaining 0.4 - 0.61 < 0
    assert controller.reason == 'row_end' and controller.stop_category == 'ROW_END'
    side = dict(speed=0., side_clearance_left_m=.01, side_nearest_x_left_m=.2)
    controller = SingleRowController()                           # hard side margin on 2 distinct scans before the start
    controller.step(0., snap(still=False, scan_count=1, **side))
    controller.step(.05, snap(still=True, scan_count=2, **side))
    assert controller.reason == 'side_blocked' and controller.hold_terminal
    controller = SingleRowController()                           # confirmed only after the start: stop, then hold
    controller.step(0., snap(still=True, scan_count=1, **side))
    assert controller.state == 'FOLLOW'
    controller.step(.05, snap(still=True, scan_count=2, **side))
    assert controller.state == 'STOP_REQUESTED' and controller.pending == ('hold', 'side_blocked')
    assert controller.stop_category == 'OBSTACLE'


# --- steering: drift bias, two-sided, anti-windup, slew --------------------------------------------------
def test_bias_converges_for_a_constant_drift_and_holds_the_centreline():
    controller, _ = following()
    plant = _Kinematic(drift=lambda t: -.6, y=.05)                  # 0.6 deg/s right drift (1.5 deg/m at 0.4 m/s)
    record = []
    closed_loop(controller, plant, 60., record=record)
    expected = .6 / (2 * .12)                                    # +2.5 % differential cancels it
    assert controller.bias_pct == pytest.approx(expected, abs=.6)
    late = [r for r in record if r[0] > 40.]
    assert max(abs(y) for *_, y in late) < .03
    assert all(c.right > c.left for _, c, _, _ in late[-100:])    # persistent correction, no equal-duty gaps
    assert controller.state == 'FOLLOW'


def test_bias_follows_a_slowly_varying_drift():
    controller, _ = following()
    def drift(t):                                                # reverses every 20 s (8 m at 0.4 m/s)
        return .8 * math.sin(2 * math.pi * t / 40.)
    plant = _Kinematic(drift=drift)
    record = []
    closed_loop(controller, plant, 80., record=record)
    late = [r for r in record if r[0] > 20.]
    assert max(abs(y) for *_, y in late) < .06
    # the estimate follows -drift / (2 g) [%] with the integrator lag (~20 deg at this 40 s period)
    bias = [tel['bias_pct'] for _, _, tel, _ in late]
    ideal = [-drift(t) / (2 * .12) for t, *_ in late]
    mb, mi = sum(bias) / len(bias), sum(ideal) / len(ideal)
    cov = sum((b - mb) * (i - mi) for b, i in zip(bias, ideal))
    corr = cov / math.sqrt(sum((b - mb) ** 2 for b in bias) * sum((i - mi) ** 2 for i in ideal))
    assert corr > .85


def test_correction_is_two_sided_and_never_reset_by_an_error_sign_change():
    controller, _ = following()
    plant = _Kinematic(drift=lambda t: 1.2, y=-.15)               # strong LEFT drift, vehicle right of centre
    record = []
    closed_loop(controller, plant, 40., record=record)
    assert controller.bias_pct < -3.                              # 1.2 / 0.24 = 5 % opposite to a left turn
    # The aim (lookahead error) asks for a LEFT yaw while the output turns RIGHT: no one-sided restriction.
    opposed = [c for t, c, tel, _ in record if tel['target_yaw_rate_deg_s'] > .05 and c.left > c.right]
    assert opposed
    bias_before = controller.bias_pct
    # A 10 deg heading kick to the right makes the OUTPUT change sign (a strong left correction); the drift
    # estimate is kept through it (not reset on a sign change; only a new controller starts from 0).
    plant.psi -= deg(10.)
    record = []
    closed_loop(controller, plant, 1.5, start=40., record=record)
    assert any(c.right > c.left + 4 for _, c, _, _ in record)
    assert controller.bias_pct < .6 * bias_before


def test_anti_windup_when_the_differential_is_saturated():
    controller, _ = following()
    plant = _Kinematic(g=0., y=.4)                                # no yaw response at all
    record = []
    closed_loop(controller, plant, 20., record=record)
    c = controller.config
    limit = min(c.max_differential_pct, 100 - controller.base_duty, controller.base_duty)
    assert all(abs(tel['differential_pct']) <= limit + 1e-9 for _, _, tel, _ in record)
    assert any(tel['saturated'] for _, _, tel, _ in record)
    assert abs(controller.bias_pct) < c.bias_max_pct              # frozen once saturated: not at its bound
    # The demand reverses: the output leaves the saturated side quickly (no stored wind-up).
    plant.y, plant.g = -.4, .12
    t, record = 20., []
    closed_loop(controller, plant, 3., start=t, record=record)
    assert any(cmd.right > cmd.left for _, cmd, _, _ in record)


def test_differential_slew_limit_and_forward_only_outputs():
    controller, _ = following()
    run(controller, 0., .5)
    step = controller.config.diff_slew_pct_s * TICK
    diffs, commands, t = [controller.differential], [], .5
    for rate in [-20.] * 6 + [0.] * 20:                           # a gyro spike: the P term demand jumps ~10 %
        t = round(t + TICK, 6)
        commands.append(controller.step(t, snap(yaw_rate=deg(rate))))
        diffs.append(controller.differential)
    assert max(abs(b - a) for a, b in zip(diffs, diffs[1:])) <= step + 1e-9
    assert max(diffs) >= 3 * step                                 # it did move, at the slew rate
    assert all(0 <= c.left <= 100 and 0 <= c.right <= 100 for c in commands)


def test_small_opposite_demand_does_not_cross_zero():
    controller, _ = following()
    controller.differential, controller._out_sign = 3., 1
    diffs, t = [], 0.
    for _ in range(16):              # 0.8 s: a tiny RIGHT demand (about -0.8 %), before the bias integrates
        t = round(t + TICK, 6)
        controller.step(t, snap(heading_error=deg(-.15)))
        diffs.append(controller.differential)
    assert min(diffs) >= 0. and diffs[-1] == 0.                  # eased to equal duty, not across zero
    t, commands = run(controller, t, 3., heading_error=deg(-6.), lateral_offset=.1)   # a real one crosses
    assert commands[-1].left > commands[-1].right


def test_plausibility_limited_lateral_reference():
    controller, _ = following()
    run(controller, 0., 1.)
    controller.step(1.05, snap(lateral_offset=.5))                 # estimate jumps 0.5 m in one tick
    reference = controller.telemetry()
    assert controller._lateral_ref == pytest.approx(controller.config.lateral_ref_slack_m_s * TICK, abs=1e-6)
    t, _ = run(controller, 1.05, 2., lateral_offset=.5)
    assert controller._lateral_ref < .25                            # followed only gradually
    assert reference['lateral_error_m'] == .5                      # telemetry reports the raw estimate too


# --- speed loop ----------------------------------------------------------------------------------------------
@pytest.mark.parametrize('scale, base_final', [(.75, 75.), (1.2, 68.5)])
def test_speed_loop_adapts_the_base_duty_to_the_observed_speed(scale, base_final):
    controller, _ = following()
    plant = _Kinematic(scale=scale)
    record = []
    closed_loop(controller, plant, 30., record=record)
    bases = [tel['base_duty_pct'] for _, _, tel, _ in record]
    c = controller.config
    assert all(c.base_min_pct <= b <= c.base_max_pct for b in bases)
    rates = [(b - a) / TICK for a, b in zip(bases, bases[1:])]
    assert max(rates) <= c.base_slew_up_pct_s + 1e-6 and min(rates) >= -c.base_slew_down_pct_s - 1e-6
    assert bases[-1] == pytest.approx(base_final, abs=1.)
    if base_final < c.base_max_pct:
        assert plant.v == pytest.approx(c.cruise_speed_m_s, abs=.03)


def test_target_speed_drops_for_a_large_steering_demand():
    controller, _ = following()
    run(controller, 0., 1.)
    assert controller.target_speed == pytest.approx(controller.config.cruise_speed_m_s)
    run(controller, 1., .5, heading_error=deg(14.))
    assert controller.target_speed < controller.config.cruise_speed_m_s - .1


# --- authority ---------------------------------------------------------------------------------------------------
def test_authority_slow_when_saturated_without_progress_then_released():
    controller, _ = following()
    plant = _Kinematic(g=.01, y=.4)                              # ~8 % of the nominal arc gain
    closed_loop(controller, plant, 12.)
    assert controller.authority_state == 'SLOW' and controller.counts['authority_slow_entries'] == 1
    assert controller.target_speed <= controller.config.authority_slow_speed_m_s + 1e-9
    plant.g, plant.y, plant.psi = .12, 0., 0.
    closed_loop(controller, plant, 8., start=12.)
    assert controller.authority_state == 'OK'


def test_authority_stop_for_lateral_or_heading_beyond_the_limits():
    controller, _ = following()
    lateral = .6
    t = 0.
    for _ in range(100):                                         # moving away from the centre at 0.4 sin 10 deg
        t = round(t + TICK, 6)
        lateral += .4 * math.sin(deg(10.)) * TICK
        controller.step(t, snap(lateral_offset=lateral, heading_error=deg(-10.)))
        if controller.state != 'FOLLOW':
            break
    assert controller.state == 'STOP_REQUESTED' and controller.pending == ('hold', 'steering_authority_insufficient')
    assert lateral > .7 and controller.stop_category == 'CONTROL_AUTHORITY'
    controller, _ = following()
    run(controller, 0., .2, heading_error=deg(31.))               # shorter than the 0.3 s confirmation
    assert controller.state == 'FOLLOW'
    run(controller, .2, .5, heading_error=deg(31.))
    assert controller.pending == ('hold', 'steering_authority_insufficient')


# --- row end and the front --------------------------------------------------------------------------------
def test_row_end_slow_zone_and_stop_at_the_observed_end_without_restart():
    controller, _ = following()
    plant = _Kinematic(end_x=12.)
    record = []
    closed_loop(controller, plant, 60., record=record)
    c = controller.config
    assert controller.state == 'HOLD' and controller.reason == 'row_end' and controller.hold_terminal
    assert controller.stop_category == 'ROW_END' and controller.counts['approach_entries'] == 1
    phases = [tel['phase'] for *_, tel, _ in record]
    first = phases.index('APPROACH_END')
    t_entry = record[first][0]
    remaining = 12. - c.end_stop_offset_m - plant.track[first - 1]   # to the stop point at the zone entry
    assert c.approach_start_m < remaining <= c.approach_start_m + controller.stopping_distance(.45) + .05
    assert all(tel['target_speed_m_s'] <= c.approach_speed_m_s + 1e-9 for t, _, tel, _ in record
               if t > t_entry and tel['phase'] == 'APPROACH_END')
    assert 12. - c.end_stop_offset_m - .25 <= plant.x <= 12. - c.end_stop_offset_m + .05    # front at the end
    # no restart and no turn after the terminal stop, whatever the snapshots say
    t, commands = run(controller, 60., 5., still=True, speed=0.)
    assert all(cmd.left == cmd.right == 0 for cmd in commands)
    assert all(state not in ('TURN', 'REDUCE', 'SETTLE') for _, state, _ in controller.events)


def test_end_evidence_never_comes_from_time_or_distance():
    controller, _ = following()
    t, commands = run(controller, 0., 30., distance_m=40.)        # long time / distance, no end evidence
    assert controller.state == 'FOLLOW' and controller.phase == 'FOLLOW'
    run(controller, t, .5, row_end_ahead_m=1.5)                   # in the slow zone
    assert controller.phase == 'APPROACH_END'
    t, _ = run(controller, t + .5, 2.5, row_end_ahead_m=None)     # evidence gone < release time: kept
    assert controller.phase == 'APPROACH_END'
    run(controller, t, 1., row_end_ahead_m=None)                  # gone > 3 s: released (a false end)
    assert controller.phase == 'FOLLOW' and controller.counts['approach_releases'] == 1


@pytest.mark.parametrize('extra, reason', [
    (dict(), 'obstacle_ahead'),                                                       # nothing ends
    (dict(row_end_ahead_m=1.), 'end_structure_ahead'),                                # confirmed end at the return
    (dict(side_ends_ahead_m=[.6, .4], end_age_s=.5), 'end_structure_ahead'),          # both lines end at it
    (dict(side_ends_ahead_m=[8., 7.5], end_age_s=.5), 'obstacle_ahead'),              # rows continue past it
    (dict(side_ends_ahead_m=[.6, .4], end_age_s=5.), 'obstacle_ahead'),               # stale end evidence
    (dict(side_ends_ahead_m=[-3., -2.5], end_age_s=.5, side_end_free=[True, True]), 'end_structure_ahead'),
    (dict(side_ends_ahead_m=[-3., -2.5], end_age_s=.5), 'obstacle_ahead'),
    (dict(side_ends_ahead_m=[.6, None], end_age_s=.5), 'obstacle_ahead'),            # one side only
    # staggered ends: the left row ended 5 m before in the open, the right runs up to the structure (dev round 1)
    (dict(side_ends_ahead_m=[-5., .4], end_age_s=.5, side_end_free=[True, False]), 'end_structure_ahead'),
    (dict(side_ends_ahead_m=[-5., .4], end_age_s=.5, side_end_free=[False, False]), 'obstacle_ahead'),
    (dict(side_ends_ahead_m=[-5., 8.], end_age_s=.5, side_end_free=[True, False]), 'obstacle_ahead'),  # right continues
])
def test_front_block_is_classified_by_end_evidence(extra, reason):
    controller, _ = following()
    run(controller, 0., 1.)
    controller.step(1.05, snap(front_clearance_m=.2, **extra))       # obstacle 0.2 + 0.61 m from the base
    assert controller.state == 'STOP_REQUESTED' and controller.pending == ('hold', reason)
    run(controller, 1.05, 2., still=True, speed=0., front_clearance_m=.2, **extra)
    assert controller.state == 'HOLD' and controller.reason == reason and controller.hold_terminal
    assert controller.stop_category == stop_category(reason)


# --- kept RowController semantics ----------------------------------------------------------------------
def test_stale_input_pauses_and_resumes_only_after_fresh_stillness():
    controller, _ = following()
    run(controller, 0., 2.)
    t, commands = run(controller, 2., .5, lidar_age_s=1.)
    assert controller.state == 'HOLD' and controller.reason == 'stale_lidar' and not controller.hold_terminal
    assert all(c.left == c.right == 0 for c in commands)
    t, _ = run(controller, t, .5, still=True, speed=0.)
    assert controller.state == 'HOLD'
    t, _ = run(controller, t, 1.2, still=True, speed=0.)
    assert controller.state == 'REACQUIRE' and controller.reason == 'fresh_input_and_still'
    t, _ = run(controller, t, .2, still=True, speed=0.)            # no NEW corridor update yet: waits
    assert controller.state == 'REACQUIRE'
    run(controller, t, .1, still=True, speed=0., corridor_updates=12)
    assert controller.state == 'FOLLOW' and controller.telemetry()['phase'] == 'FOLLOW'


def test_missing_sensors_at_start_and_drive_no_response_are_terminal():
    controller = SingleRowController()
    run(controller, 0., 3.5, imu_age_s=math.inf, still=True)
    assert controller.reason == 'sensor_missing' and controller.hold_terminal and controller.stop_category == 'SENSOR'
    controller, _ = following()
    t, _ = run(controller, 0., 4., still=True, still_s=1., speed=0.)   # commanded, never seen moving
    assert controller.pending == ('hold', 'drive_no_response') or controller.reason == 'drive_no_response'
    run(controller, t, 2., still=True, speed=0.)
    assert controller.reason == 'drive_no_response' and controller.stop_category == 'DRIVE'


def test_stop_not_observed_and_corridor_lost_wait():
    controller, _ = following()
    run(controller, 0., 1.)
    controller.step(1.05, snap(front_clearance_m=.1))
    t, _ = run(controller, 1.05, 5., front_clearance_m=.1)        # never seen still
    assert controller.reason == 'stop_not_observed' and controller.stop_category == 'DRIVE'
    controller, _ = following()
    run(controller, 0., 1.)
    t, _ = run(controller, 1., .3, corridor_valid=False)
    assert controller.state == 'STOP_REQUESTED' and controller.pending == ('wait_corridor', 'corridor_lost')
    assert controller.telemetry()['phase'] == 'WAIT'
    t, _ = run(controller, t, 1., corridor_valid=False, still=True, speed=0.)
    assert controller.state == 'REACQUIRE'
    t, _ = run(controller, t, .2, still=True, speed=0., corridor_updates=20, heading_error=deg(28.))
    assert controller.reason == 'reacquire_misaligned' and controller.hold_terminal


def test_never_pivots_or_reverses_on_arbitrary_snapshots():
    rng = random.Random(7)
    controller = SingleRowController()
    t = 0.
    states = set()
    for _ in range(4000):
        t = round(t + TICK, 6)
        s = snap(heading_error=deg(rng.uniform(-35, 35)), lateral_offset=rng.uniform(-.9, .9),
                 yaw_rate=deg(rng.uniform(-8, 8)), speed=rng.uniform(0, .6), still=rng.random() < .2,
                 front_clearance_m=rng.choice((10., 10., 10., .3)), corridor_valid=rng.random() < .97,
                 row_end_ahead_m=rng.choice((None, None, None, 3., 1.)), lidar_age_s=rng.choice((.05, .05, .6)))
        command = controller.step(t, s)
        assert isinstance(command.left, int) and isinstance(command.right, int)
        assert 0 <= command.left <= 100 and 0 <= command.right <= 100
        states.add(controller.state)
        if controller.state == 'HOLD' and controller.hold_terminal:
            controller = SingleRowController()                     # a terminal stop never drives on: new run
    assert not states & {'TURN', 'REDUCE', 'SETTLE'}


def test_telemetry_reports_the_control_quantities():
    controller, _ = following()
    run(controller, 0., 1., heading_error=deg(3.), lateral_offset=.1, row_end_ahead_m=6.)
    tel = controller.telemetry()
    for key in ('phase', 'lateral_error_m', 'heading_error_deg', 'lookahead_error_deg', 'target_yaw_rate_deg_s',
                'measured_yaw_rate_deg_s', 'differential_pct', 'bias_pct', 'base_duty_pct', 'target_speed_m_s',
                'measured_speed_m_s', 'saturated', 'saturation_fraction', 'authority_state', 'row_end_ahead_m',
                'stop_category'):
        assert key in tel, key
    assert tel['phase'] == 'FOLLOW' and tel['row_end_ahead_m'] == 6. and tel['stop_category'] is None


def test_row_controller_is_unchanged_and_independent():
    plain = RowController(config_for('continuous'))
    plain.step(0., snap(still=True))
    assert plain.state == 'FOLLOW' and not hasattr(plain, 'bias_pct')
    assert ControlConfig().mode == 'hybrid' and ControlConfig().lookahead_m == 3.


def test_closed_loop_smoke_in_the_2d_harness():
    from closed_loop.harness import run_episode
    from closed_loop.matrix import episodes
    episode = episodes(seeds=(0,), modes=('continuous',), names=('nominal_open_end',))[0]
    result = run_episode(episode, controller_factory=SingleRowController)
    assert result['final_reason'] == 'row_end' and result['collisions'] == 0 and result['end'] == 'terminal_hold'
    assert -1.2 <= result['final_x_rel_row_end_m'] <= 1.
    assert result['lateral_max_m'] < .3 and result['turns'] == 0
