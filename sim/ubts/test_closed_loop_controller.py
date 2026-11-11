"""State-machine contract of the encoderless row controllers (synthetic inputs).

Snapshots stand in for SensorEstimator output; no plant, scene or truth is
involved. Timing follows the 20 Hz controller tick. Each behavioural test is
written so that removing the feature it names makes it fail (demonstrated by
mutation runs on a scratch copy; see docs/FIELD_RESPONSE_SIM_CONTROL_20261013.md).
"""
import math

import pytest

from closed_loop.controllers import ControlConfig, RowController, TERMINAL_HOLDS, config_for
from closed_loop.estimator import SensorEstimator

TICK = .05
PIVOT_RADIUS = SensorEstimator().pivot_radius_required()


def snap(**overrides):
    s = dict(lidar_age_s=.05, imu_age_s=.02, still=False, still_s=0., corridor_valid=True, corridor_age_s=.1,
             corridor_updates=10, heading_error=0., lateral_offset=0., front_clearance_m=10.,
             speed=.4, odom_valid=True, row_end_ahead_m=None, yaw_rate=0., distance_m=0., pivot_clearance_m=3.,
             pivot_radius_required_m=PIVOT_RADIUS, gyro_heading=0., vibration_rms=.1)
    s.update(overrides)
    if overrides.get('still') and 'still_s' not in overrides:
        s['still_s'] = 1.
    return s


def deg(value):
    return math.radians(value)


def run(controller, start, seconds, **overrides):
    commands = []
    t = start
    for _ in range(int(round(seconds / TICK))):
        t = round(t + TICK, 6)
        commands.append(controller.step(t, snap(**overrides)))
    return t, commands


def following(mode='hybrid', **config):
    controller = RowController(config_for(mode, **config))
    command = controller.step(0., snap(still=True))
    assert controller.state == 'FOLLOW', controller.events
    return controller, command


def stopped_turning(heading=25., **config):
    """Hybrid controller that has just started a pivot from REACQUIRE."""
    controller = RowController(config_for('hybrid', **config))
    controller.step(0., snap(still=True, heading_error=deg(heading), speed=0.))
    assert controller.state == 'TURN', controller.events
    return controller


def test_pivot_radius_fixture_follows_the_estimator_geometry():
    # hypot(0.61 + 0.25, 0.35) + 0.10 with the nominal calibration (not a hard-coded 1.08)
    assert PIVOT_RADIUS == pytest.approx(math.hypot(.61 + .25, .35) + .10, abs=1e-9)


def test_waits_for_data_and_stillness_before_follow():
    controller = RowController(config_for('hybrid'))
    command = controller.step(0., snap(lidar_age_s=math.inf, imu_age_s=math.inf, corridor_valid=False))
    assert (command.left, command.right, controller.state) == (0, 0, 'REACQUIRE')
    command = controller.step(.05, snap(still=False))
    assert (command.left, command.right, controller.state) == (0, 0, 'REACQUIRE')
    command = controller.step(.1, snap(still=True))
    assert controller.state == 'FOLLOW' and command.left == command.right == 75


def test_missing_sensor_at_start_becomes_a_terminal_hold():
    controller = RowController(config_for('hybrid'))
    t, commands = run(controller, 0., 3.5, imu_age_s=math.inf, still=True)
    assert controller.state == 'HOLD' and controller.reason == 'sensor_missing' and controller.hold_terminal
    assert all(c.left == c.right == 0 for c in commands)


def test_unknown_hold_reasons_fail_closed():
    controller, _ = following()
    controller._hold('some_new_reason')
    assert controller.hold_terminal
    t, commands = run(controller, 0., 3., still=True)
    assert controller.state == 'HOLD' and all(c.left == c.right == 0 for c in commands)
    assert set(('stale_lidar', 'stale_imu')).isdisjoint(TERMINAL_HOLDS)


def test_still_confirmation_time_is_enforced():
    controller = RowController(config_for('hybrid'))
    run(controller, 0., 1., still=True, still_s=.2)                  # steady, but only 0.2 s
    assert controller.state == 'REACQUIRE'
    controller.step(1.05, snap(still=True, still_s=.45))
    assert controller.state == 'FOLLOW'
    relaxed = RowController(config_for('hybrid', still_confirm_s=.1))
    relaxed.step(0., snap(still=True, still_s=.2))
    assert relaxed.state == 'FOLLOW'


def test_deadband_entry_exit_hysteresis_and_min_hold():
    controller, _ = following()
    t, commands = run(controller, 0., .5, heading_error=deg(1.5))
    assert all(c.left == c.right == 75 for c in commands)             # inside deadband
    t, commands = run(controller, t, .2, heading_error=deg(3.))
    assert commands[-1].right > commands[-1].left                     # left turn correction
    t, commands = run(controller, t, .2, heading_error=deg(.5))
    assert controller.correcting                                      # min hold (>= MCU ramp) not over
    t, commands = run(controller, t, .8, heading_error=deg(.5))
    assert not controller.correcting and commands[-1].left == commands[-1].right


def test_exit_hysteresis_after_min_hold():
    controller, _ = following()
    t, _ = run(controller, 0., .2, heading_error=deg(3.))             # enter (>= 2.5 deg)
    assert controller.correcting
    t, _ = run(controller, t, 1.5, heading_error=deg(2.))             # between exit 1.0 and entry 2.5
    assert controller.correcting                                      # min hold long over: hysteresis keeps it
    t, _ = run(controller, t, .2, heading_error=deg(.8))              # below exit
    assert not controller.correcting


def test_recorrection_interval_with_urgent_bypass():
    controller, _ = following()
    t, _ = run(controller, 0., 1.2, heading_error=deg(3.))
    t, _ = run(controller, t, .1, heading_error=deg(.5))
    assert not controller.correcting
    t, commands = run(controller, t, .5, heading_error=deg(3.))
    assert not controller.correcting and commands[-1].left == commands[-1].right
    t, commands = run(controller, t, .1, heading_error=deg(7.))      # urgent error bypasses
    assert controller.correcting


def test_no_quick_reversal_of_the_correction_sign():
    controller, _ = following()
    t, commands = run(controller, 0., .3, heading_error=deg(4.))
    assert commands[-1].right > commands[-1].left
    t, commands = run(controller, t, .4, heading_error=deg(-4.))
    assert all(c.right >= c.left for c in commands)                  # held, never flipped
    t, commands = run(controller, t, 1., heading_error=deg(-4.))
    assert commands[-1].left > commands[-1].right


def test_outputs_stay_bounded_integer_and_integral_does_not_wind_up():
    controller, _ = following('continuous')
    t, commands = run(controller, 0., 12., heading_error=deg(30.), yaw_rate=0.)
    assert controller.state == 'FOLLOW'                              # A never stops to turn
    for c in commands:
        assert isinstance(c.left, int) and isinstance(c.right, int)
        assert 50 <= c.left <= 100 and 50 <= c.right <= 100
        assert c.right >= c.left
    assert abs(controller.differential) == 25
    # Conditional integration: saturated while the rate error pushes further -> no accumulation.
    assert abs(controller.integral) <= 1.
    # And the output leaves saturation as soon as the demand drops (no stored wind-up).
    t, commands = run(controller, t, .3, heading_error=deg(3.), yaw_rate=deg(2.))
    assert commands[-1].right - commands[-1].left < 40


def test_large_lookahead_error_reduce_stop_turn_settle_reacquire():
    controller, _ = following()
    t, commands = run(controller, 0., .1, heading_error=deg(20.))
    assert controller.state == 'REDUCE' and max(commands[-1].left, commands[-1].right) <= 62 + 25
    t, commands = run(controller, t, .6, heading_error=deg(20.), speed=.2)
    assert controller.state == 'STOP_REQUESTED' and (commands[-1].left, commands[-1].right) == (0, 0)
    t, commands = run(controller, t, 1.2, heading_error=deg(20.), still=True, speed=0.)
    assert controller.state == 'TURN' and (commands[-1].left, commands[-1].right) == (-100, 100)
    heading = 0.
    while controller.state == 'TURN':
        t = round(t + TICK, 6)
        heading += deg(20.) * TICK
        controller.step(t, snap(heading_error=deg(20.) - heading, yaw_rate=deg(20.), gyro_heading=heading,
                                speed=0., vibration_rms=4.))
        assert t < 8.
    assert controller.state == 'SETTLE' and math.degrees(heading) < 20.
    t, commands = run(controller, t, .6, heading_error=deg(2.), still=True, speed=0., gyro_heading=heading,
                      vibration_rms=.1)
    assert controller.state == 'REACQUIRE' and all(c.left == c.right == 0 for c in commands)
    t, commands = run(controller, t, .1, heading_error=deg(2.), still=True, corridor_updates=13, speed=0.)
    assert controller.state == 'FOLLOW'
    assert controller.counts['turns'] == 1 and controller.counts['stops'] == 1


def test_hybrid_trigger_uses_the_lookahead_error_the_pivot_aims_at():
    # 16 deg heading but already pointing back at the centreline from 0.7 m: target ~2.9 deg.
    controller, _ = following()
    t, commands = run(controller, 0., 30., heading_error=deg(16.), lateral_offset=.7)
    assert controller.counts['stops'] == 0 and controller.state == 'FOLLOW'
    assert not any(state == 'REDUCE' for _, state, _ in controller.events)    # no slow-down either
    # Same heading pointing away from the centreline: target ~29 deg -> REDUCE.
    controller, _ = following()
    run(controller, 0., .1, heading_error=deg(16.), lateral_offset=-.7)
    assert controller.state == 'REDUCE' and controller.reason == 'large_heading_error'


def test_reduce_returns_to_follow_when_the_error_recovers():
    controller, _ = following()
    t, _ = run(controller, 0., .1, heading_error=deg(20.))
    assert controller.state == 'REDUCE'
    t, _ = run(controller, t, .1, heading_error=deg(8.))
    assert controller.state == 'FOLLOW' and controller.counts['stops'] == 0


def test_ineffective_steering_triggers_stop_turn_only_after_hold_time():
    controller, _ = following()
    t, _ = run(controller, 0., 2.5, heading_error=deg(12.), yaw_rate=0.)
    assert controller.state == 'FOLLOW'                              # ramp + hold not over
    t, _ = run(controller, t, 1., heading_error=deg(12.), yaw_rate=0.)
    assert controller.state == 'REDUCE' and controller.reason == 'steering_ineffective'
    good, _ = following()
    run(good, 0., 4., heading_error=deg(12.), yaw_rate=deg(4.))      # responding: no stop
    assert good.state == 'FOLLOW'


def test_weak_steering_with_a_small_error_never_stops():
    controller, _ = following()
    run(controller, 0., 60., heading_error=deg(2.8), yaw_rate=0.)
    assert controller.counts['stops'] == 0 and controller.state == 'FOLLOW'


def test_weak_steering_does_not_stop_while_the_error_shrinks():
    controller, _ = following()
    t, error = 0., 12.
    for _ in range(int(6. / TICK)):
        t = round(t + TICK, 6)
        error -= .6 * TICK                                           # converging at 0.6 deg/s
        controller.step(t, snap(heading_error=deg(error), yaw_rate=0.))
    assert controller.counts['stops'] == 0


def test_stops_without_a_turn_are_capped():
    controller, _ = following()
    t = 0.
    for _ in range(4):                                               # REDUCE -> stop -> no turn needed
        t, _ = run(controller, t, .7, heading_error=deg(20.), speed=.2)
        if controller.state == 'HOLD':
            break
        t, _ = run(controller, t, 1.5, heading_error=deg(2.), still=True, speed=0.)
    assert controller.state == 'HOLD' and controller.reason == 'stop_repeat_limit' and controller.hold_terminal
    assert controller.counts['stops_without_turn'] == 3


def test_forward_drive_without_observed_motion_holds():
    controller, _ = following()
    t, commands = run(controller, 0., 1.9, still=True, still_s=1., speed=0.)   # ramp + grace: no verdict yet
    assert controller.state == 'FOLLOW'
    t, commands = run(controller, t, 1.2, still=True, still_s=1., speed=0.)
    assert controller.state == 'STOP_REQUESTED' and controller.reason == 'drive_no_response'
    assert commands[-1].left == commands[-1].right == 0
    t, _ = run(controller, t, 1.2, still=True, speed=0.)
    assert controller.state == 'HOLD' and controller.reason == 'drive_no_response' and controller.hold_terminal
    moving, _ = following()
    run(moving, 0., 10., speed=.4)
    assert moving.state == 'FOLLOW'


def test_stale_input_holds_then_recovers_after_fresh_stillness():
    controller, _ = following()
    command = controller.step(.05, snap(lidar_age_s=.6))
    assert controller.state == 'HOLD' and controller.reason == 'stale_lidar'
    assert (command.left, command.right) == (0, 0) and not controller.hold_terminal
    command = controller.step(.1, snap(imu_age_s=.5))
    assert controller.state == 'HOLD' and (command.left, command.right) == (0, 0)
    t, _ = run(controller, .1, 1.5, still=False)
    assert controller.state == 'HOLD'                                # fresh but not still
    t, _ = run(controller, t, 1.1, still=True)
    assert controller.state == 'REACQUIRE'


@pytest.mark.parametrize('stale', [dict(lidar_age_s=.6), dict(imu_age_s=.5)])
def test_stale_input_never_downgrades_a_pending_row_end(stale):
    controller, _ = following()
    controller.step(.05, snap(row_end_ahead_m=.05))
    assert controller.state == 'STOP_REQUESTED' and controller.reason == 'row_end_candidate'
    controller.step(.1, snap(**stale))
    assert controller.state == 'HOLD' and controller.reason == 'row_end_candidate' and controller.hold_terminal
    t, commands = run(controller, .1, 4., still=True, speed=0.)      # fresh, still: must stay held
    assert controller.state == 'HOLD' and all(c.left == c.right == 0 for c in commands)


@pytest.mark.parametrize('reason', ['pivot_stall_abort', 'turn_timeout'])
def test_stale_input_never_downgrades_a_pending_turn_abort(reason):
    controller = stopped_turning()
    controller._abort_turn(reason)
    controller.step(.1, snap(lidar_age_s=.6, still=True, speed=0.))
    assert controller.state == 'HOLD' and controller.reason == reason and controller.hold_terminal
    t, commands = run(controller, .1, 5., still=True, speed=0., heading_error=deg(25.))
    assert controller.state == 'HOLD' and controller.counts['turns'] == 1


def test_stale_input_during_a_stall_suspect_pivot_is_terminal():
    controller = stopped_turning()
    t, heading = 0., 0.
    while t < 2.05:
        t = round(t + TICK, 6)
        rate = 2.2 if t > 1.3 else 0.                                # barely turning (< 3 deg so far)
        heading += deg(rate) * TICK
        controller.step(t, snap(heading_error=deg(25.) - heading, yaw_rate=deg(rate), gyro_heading=heading,
                                speed=0.))
    assert controller.state == 'TURN' and math.degrees(heading) < 3.  # slow, not yet aborted
    controller.step(t + .05, snap(lidar_age_s=.6))
    assert controller.state == 'HOLD' and controller.reason == 'pivot_stall_abort' and controller.hold_terminal


def test_pivot_stall_is_aborted_and_never_pushed():
    controller = stopped_turning()
    t, commands = run(controller, 0., 2.2, heading_error=deg(25.), yaw_rate=0., gyro_heading=0., speed=0.)
    assert controller.state == 'STOP_REQUESTED' and controller.reason == 'pivot_stall_abort'
    assert commands[-1].left == commands[-1].right == 0
    assert sum(abs(c.left) == 100 for c in commands) <= 2.05 / TICK
    t, commands = run(controller, t, 5., heading_error=deg(25.), still=True, speed=0.)
    assert controller.state == 'HOLD' and controller.hold_terminal
    assert all(c.left == c.right == 0 for c in commands) and controller.counts['stalls'] == 1
    assert controller.counts['stops'] == 1                           # one stop, counted once


def test_slow_but_rotating_pivot_is_not_a_stall():
    controller = stopped_turning(heading=25.)
    t, heading = 0., 0.
    while controller.state == 'TURN' and t < 12.:
        t = round(t + TICK, 6)
        heading += deg(5.) * TICK if t > 1. else 0.                  # measured slow tail ~5 deg/s
        controller.step(t, snap(heading_error=deg(25.) - heading, yaw_rate=deg(5.) if t > 1. else 0.,
                                gyro_heading=heading, speed=0.))
    assert controller.state == 'SETTLE' and controller.counts['stalls'] == 0


def test_stop_observed_waits_for_the_mcu_fade():
    controller = stopped_turning()
    t, _ = run(controller, 0., 2.2, heading_error=deg(25.), yaw_rate=0., gyro_heading=0., speed=0., still=True)
    assert controller.state == 'STOP_REQUESTED'                      # stall abort while already still
    requested = controller.since
    t, _ = run(controller, t, .5, heading_error=deg(25.), still=True, speed=0.)
    assert controller.state == 'STOP_REQUESTED'                      # the +-100 duty is still fading
    t, _ = run(controller, t, .8, heading_error=deg(25.), still=True, speed=0.)
    assert controller.state == 'HOLD'
    observed = [e for e in controller.events if e[1] == 'STOP_OBSERVED'][-1][0]
    assert observed - requested >= 1.0                               # |100| / 100 %/s ramp


def test_pivot_that_sticks_mid_turn_is_aborted():
    controller = stopped_turning(heading=40.)
    t, heading = 0., 0.
    while controller.state == 'TURN' and t < 15.:
        t = round(t + TICK, 6)
        rate = 20. if 1. < t < 1.5 else 0.                           # 10 deg, then stuck
        heading += deg(rate) * TICK
        controller.step(t, snap(heading_error=deg(40.) - heading, yaw_rate=deg(rate), gyro_heading=heading,
                                speed=0.))
    assert controller.state == 'STOP_REQUESTED' and controller.reason == 'pivot_stall_abort'
    assert t < 3.5


def test_turn_timeout_bounds_a_very_slow_pivot():
    controller = stopped_turning(heading=25.)
    t, heading = 0., 0.
    while controller.state == 'TURN' and t < 20.:
        t = round(t + TICK, 6)
        heading += deg(1.6) * TICK                                   # above the progress floor, too slow
        controller.step(t, snap(heading_error=deg(25.) - heading, yaw_rate=deg(1.6) if t > 2.1 else deg(2.5),
                                gyro_heading=heading + deg(3.), speed=0.))
    limit = 1. + 1.5 * 25. / 5. + 1.5
    assert controller.state == 'STOP_REQUESTED' and controller.reason == 'turn_timeout'
    assert limit < t <= limit + .1


def _settling(**config):
    controller = stopped_turning(**config)
    controller.turn['released'] = None
    t, heading = 0., 0.
    while controller.state == 'TURN':
        t = round(t + TICK, 6)
        heading += deg(20.) * TICK
        controller.step(t, snap(heading_error=deg(25.) - heading, yaw_rate=deg(20.), gyro_heading=heading,
                                speed=0., vibration_rms=4.))
    assert controller.state == 'SETTLE'
    return controller, t, heading


def test_settle_requires_a_continuous_dwell_of_stillness_and_low_vibration():
    controller, t, heading = _settling()
    t, _ = run(controller, t, .4, still=True, gyro_heading=heading, speed=0., vibration_rms=.1)
    assert controller.state == 'SETTLE'                              # 0.4 s < settle_s 0.5 s
    t, _ = run(controller, t, .05, still=False, gyro_heading=heading, speed=0., vibration_rms=.1)
    t, _ = run(controller, t, .4, still=True, gyro_heading=heading, speed=0., vibration_rms=.1)
    assert controller.state == 'SETTLE'                              # dwell restarted after the break
    t, _ = run(controller, t, .2, still=True, gyro_heading=heading, speed=0., vibration_rms=.1)
    assert controller.state == 'REACQUIRE'


def test_settle_vibration_gate_blocks_reacquire():
    controller, t, heading = _settling()
    t, _ = run(controller, t, 2., still=True, gyro_heading=heading, speed=0., vibration_rms=4.)
    assert controller.state == 'SETTLE'
    t, _ = run(controller, t, .6, still=True, gyro_heading=heading, speed=0., vibration_rms=.1)
    assert controller.state == 'REACQUIRE'


def test_settle_timeout_is_a_terminal_hold():
    controller, t, heading = _settling()
    t, commands = run(controller, t, 4.5, still=False, gyro_heading=heading, speed=0., vibration_rms=.1)
    assert controller.state == 'HOLD' and controller.reason == 'settle_timeout' and controller.hold_terminal
    assert all(c.left == c.right == 0 for c in commands)


def test_corridor_loss_waits_then_holds_when_not_reacquired():
    controller, _ = following()
    controller.step(.05, snap(corridor_valid=False))
    assert controller.state == 'STOP_REQUESTED' and controller.reason == 'corridor_lost'
    t, _ = run(controller, .05, 1.2, corridor_valid=False, still=True, speed=0.)
    assert controller.state == 'REACQUIRE' and controller.reason == 'corridor_lost_wait'
    t, commands = run(controller, t, 5.5, corridor_valid=False, still=True, speed=0.)
    assert controller.state == 'HOLD' and controller.reason == 'corridor_not_reacquired'
    assert all(c.left == c.right == 0 for c in commands)
    # A corridor that comes back within the wait is followed again.
    controller, _ = following()
    controller.step(.05, snap(corridor_valid=False))
    t, _ = run(controller, .05, 1.2, corridor_valid=False, still=True, speed=0.)
    t, _ = run(controller, t, .5, still=True, speed=0., corridor_updates=13)
    assert controller.state == 'FOLLOW'


def test_repeat_cap_limits_stop_turns_per_distance():
    controller = RowController(config_for('hybrid'))
    controller.turn_starts = [0., 1.]
    controller.step(0., snap(still=True, heading_error=deg(25.), distance_m=2.))
    assert controller.state == 'HOLD' and controller.reason == 'stop_turn_repeat_limit'
    controller = RowController(config_for('hybrid'))
    controller.turn_starts = [0., 1.]
    controller.step(0., snap(still=True, heading_error=deg(25.), distance_m=6.5))
    assert controller.state == 'TURN'                                # outside the 5 m window


def test_front_wall_and_row_end_holds_are_terminal():
    controller, _ = following()
    controller.step(.05, snap(front_clearance_m=.3))
    assert controller.state == 'STOP_REQUESTED'
    t, commands = run(controller, .05, 1., front_clearance_m=.3, still=True, speed=0.)
    assert controller.state == 'HOLD' and controller.reason == 'front_blocked' and controller.hold_terminal
    t, commands = run(controller, t, 2., still=True, speed=0.)       # wall gone: still holds
    assert controller.state == 'HOLD' and all(c.left == c.right == 0 for c in commands)
    controller, _ = following()
    controller.step(.05, snap(row_end_ahead_m=.05))
    run(controller, .05, 1., still=True, speed=0.)
    assert controller.state == 'HOLD' and controller.reason == 'row_end_candidate'


def test_stopping_distance_uses_the_refitted_stop_model():
    controller = RowController(config_for('hybrid'))
    # 0.88 m/s: hold 0.15 + 0.25 s, then the conservative p10 deceleration 1.07 m/s^2
    assert controller.stopping_distance(.88) == pytest.approx(.88 * .4 + .88 ** 2 / (2 * 1.07))
    assert controller.required_front(0.) == pytest.approx(.35)


def test_missing_pivot_clearance_and_stop_timeout_hold():
    controller = RowController(config_for('hybrid'))
    controller.step(0., snap(still=True, heading_error=deg(25.), pivot_clearance_m=.9))
    assert controller.state == 'HOLD' and controller.reason == 'pivot_clearance_insufficient'
    controller, _ = following()
    controller.step(.05, snap(front_clearance_m=.3))
    run(controller, .05, 4., front_clearance_m=.3, still=False)
    assert controller.state == 'HOLD' and controller.reason == 'stop_not_observed'


def test_stop_turn_baseline_stops_for_small_errors():
    controller, command = following('stop_turn')
    assert command.left == command.right
    controller.step(.05, snap(heading_error=deg(5.)))
    assert controller.state == 'STOP_REQUESTED' and controller.reason == 'stop_turn_correction'


def test_configuration_guards():
    with pytest.raises(ValueError):
        ControlConfig(min_hold_s=.5)                                  # shorter than the MCU ramp
    with pytest.raises(ValueError):
        ControlConfig(base_duty_pct=90, max_differential_pct=25)      # no differential headroom
    with pytest.raises(ValueError):
        ControlConfig(enter_error_deg=1., exit_error_deg=1.)          # no hysteresis
    with pytest.raises(ValueError):
        ControlConfig(turn_enter_deg=10., turn_exit_deg=10.)          # no REDUCE hysteresis
    with pytest.raises(ValueError):
        ControlConfig(mode='encoder')


def test_fallback_variant_corrects_continuously_when_no_pivot_clearance():
    controller = RowController(config_for('hybrid_fallback'))
    controller.step(0., snap(still=True, heading_error=deg(25.), pivot_clearance_m=.9))
    assert controller.state == 'REACQUIRE' and controller.reason.startswith('continuous_fallback')
    t, commands = run(controller, 0., .2, still=True, heading_error=deg(25.), pivot_clearance_m=.9, corridor_updates=13)
    assert controller.state == 'FOLLOW' and commands[-1].right > commands[-1].left     # continuous, no pivot
    t, _ = run(controller, t, 2., heading_error=deg(25.), pivot_clearance_m=.9, corridor_updates=13, distance_m=1.)
    assert controller.state == 'FOLLOW' and controller.counts['turns'] == 0
    # once the fallback distance is driven, a pivot can be requested again (and is capped at one fallback)
    t, _ = run(controller, t, .1, heading_error=deg(25.), pivot_clearance_m=.9, distance_m=7.)
    assert controller.state in ('REDUCE', 'STOP_REQUESTED')
    default = RowController(config_for('hybrid'))
    default.step(0., snap(still=True, heading_error=deg(25.), pivot_clearance_m=.9))
    assert default.state == 'HOLD' and default.reason == 'pivot_clearance_insufficient'


def test_fallback_variant_after_a_pivot_stall_never_pivots_again_in_the_window():
    controller = RowController(config_for('hybrid_fallback'))
    controller.step(0., snap(still=True, heading_error=deg(25.), speed=0.))
    t, _ = run(controller, 0., 2.2, heading_error=deg(25.), yaw_rate=0., gyro_heading=0., speed=0.)
    assert controller.reason == 'pivot_stall_abort'
    t, _ = run(controller, t, 1.3, heading_error=deg(25.), still=True, speed=0.)
    assert controller.state == 'REACQUIRE' and controller.reason == 'continuous_fallback_after_pivot_stall_abort'
    t, commands = run(controller, t, 1., heading_error=deg(25.), still=True, speed=0., corridor_updates=13)
    assert controller.state == 'FOLLOW' and controller.counts['turns'] == 1 and controller.counts['fallbacks'] == 1


def test_stall_check_uses_scan_matching_rotation_when_available():
    """A slow pivot with a vibration-noisy gyro is not a stall; a flat LiDAR heading is."""
    controller = RowController(config_for('hybrid'))
    controller.step(0., snap(still=True, heading_error=deg(25.), speed=0., lidar_heading=0.))
    t, lidar = 0., 0.
    while controller.state == 'TURN' and t < 3.:
        t = round(t + TICK, 6)
        lidar += deg(4.5) * TICK if t > 1.3 else 0.                   # slow 4.5 deg/s after breakaway
        noisy = -3. if int(t / TICK) % 2 else 3.                      # gyro rate dominated by vibration
        controller.step(t, snap(heading_error=deg(25.) - lidar, yaw_rate=deg(noisy), gyro_heading=0.,
                                lidar_heading=lidar, speed=0.))
    assert controller.state == 'TURN' and controller.counts['stalls'] == 0
    stalled = RowController(config_for('hybrid'))
    stalled.step(0., snap(still=True, heading_error=deg(25.), speed=0., lidar_heading=0.))
    t, _ = run(stalled, 0., 2.2, heading_error=deg(25.), yaw_rate=deg(3.), gyro_heading=deg(4.), lidar_heading=0.,
               speed=0.)
    assert stalled.state == 'STOP_REQUESTED' and stalled.reason == 'pivot_stall_abort'
