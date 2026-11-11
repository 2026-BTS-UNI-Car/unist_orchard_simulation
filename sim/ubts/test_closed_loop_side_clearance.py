"""Opt-in side-clearance rule (variants *_side): estimator split, controller contract, one episode.

Synthetic snapshots stand in for SensorEstimator output (no plant, scene or
truth) except in the episode test, where truth is used only by the harness
metrics. Each behavioural test fails when the feature it names is removed:
swing inhibit (5), distinct-scan confirmation (3), yaw compensation (9), the
hard rule (3, 10). Test 10 does NOT detect a removed swing inhibit: canopy seed 6
spawns 4 mm inside a branch, so the side variants refuse the start (hard rule)
before any steering happens. SIM evaluation only; not a real-vehicle qualification.
"""
import hashlib
import math
import struct

import numpy as np
import pytest

from closed_loop.controllers import SIDE_VARIANTS, TERMINAL_HOLDS, VARIANTS, ControlConfig, RowController, config_for
from closed_loop.estimator import EstimatorConfig, SensorEstimator
from closed_loop.harness import run_episode
from closed_loop.matrix import ALL_MODES, MODE_LABEL, SCENARIOS, episodes
from closed_loop.world import WorldConfig, build_world

TICK = .05
PIVOT_RADIUS = SensorEstimator().pivot_radius_required()
CFG = EstimatorConfig()
XC = CFG.lidar_x_in_base_m - CFG.pivot_icr_behind_lidar_m        # nominal rotation centre (0.0)
SD = CFG.pivot_icr_uncertainty_m                                  # 0.25


def snap(**overrides):
    s = dict(lidar_age_s=.05, imu_age_s=.02, still=False, still_s=0., corridor_valid=True, corridor_age_s=.1,
             corridor_updates=10, heading_error=0., lateral_offset=0., front_clearance_m=10.,
             speed=.4, odom_valid=True, row_end_ahead_m=None, yaw_rate=0., distance_m=0., pivot_clearance_m=3.,
             pivot_radius_required_m=PIVOT_RADIUS, gyro_heading=0., vibration_rms=.1)
    s.update(overrides)
    if overrides.get('still') and 'still_s' not in overrides:
        s['still_s'] = 1.
    return s


def side(left=math.inf, right=math.inf, left_x=None, right_x=None, scan=1, **overrides):
    """Snapshot with the estimator's side keys (x None when that side has no return)."""
    if left_x is None and math.isfinite(left):
        left_x = .4
    if right_x is None and math.isfinite(right):
        right_x = .4
    return snap(side_clearance_left_m=left, side_clearance_right_m=right, side_nearest_x_left_m=left_x,
                side_nearest_x_right_m=right_x, rotation_centre_x_m=XC, rotation_centre_sd_m=SD,
                scan_count=scan, side_clearance_m=min(left, right), **overrides)


def deg(value):
    return math.radians(value)


def following(mode='hybrid_side', **config):
    controller = RowController(config_for(mode, **config))
    controller.step(0., snap(still=True))
    assert controller.state == 'FOLLOW', controller.events
    return controller


def bits(value):
    return struct.pack('<d', value)


# 1 -------------------------------------------------------------------------------
def test_side_split_matches_combined_band():
    half_l, half_w = CFG.footprint_length_m / 2, CFG.footprint_width_m / 2
    rng = np.random.default_rng(7)
    cases = [rng.uniform(-2., 2., (n, 2)) for n in (1, 5, 50, 2000) for _ in range(20)]
    cases += [c[c[:, 1] > 0] for c in cases[:30]] + [c[c[:, 1] < 0] for c in cases[:30]]   # one side only
    edge = np.array(((.2, .35), (.2, -.35), (.43, .5), (-.79, -.5), (.2, .30000001), (.2, -.30000001)))
    cases += [edge, np.repeat(edge, 3, axis=0), np.zeros((0, 2))]                  # ties, band edges, empty
    for points in cases:
        estimator = SensorEstimator()
        estimator._clearances(points)
        assert bits(min(estimator.side_left, estimator.side_right)) == bits(estimator.side_clearance)
        bx, by = points[:, 0] + CFG.lidar_x_in_base_m, points[:, 1]
        beside = (np.abs(bx) <= half_l) & (np.abs(by) > half_w - .05)
        for value, x, mask, lateral in ((estimator.side_left, estimator.side_left_x, beside & (by > 0), by),
                                        (estimator.side_right, estimator.side_right_x, beside & (by < 0), -by)):
            if not mask.any():
                assert value == math.inf and x is None
            else:
                i = int(np.argmin(lateral[mask]))
                assert value == float(lateral[mask][i] - half_w) and x == float(bx[mask][i])
                assert -half_l <= x <= half_l
    estimator = SensorEstimator()
    estimator._clearances(np.array(((.1, .5), (.0, -.9))))
    s = estimator.snapshot(0.)
    assert s['side_clearance_left_m'] == pytest.approx(.15) and s['side_clearance_right_m'] == pytest.approx(.55)
    assert s['side_nearest_x_left_m'] == pytest.approx(.28) and s['side_nearest_x_right_m'] == pytest.approx(.18)
    assert s['side_clearance_m'] == min(s['side_clearance_left_m'], s['side_clearance_right_m'])
    assert s['rotation_centre_x_m'] == 0. and s['rotation_centre_sd_m'] == .25 and s['scan_count'] == 0


# 2 -------------------------------------------------------------------------------
SCRIPT = ([dict(still=True, speed=0.)] * 4 + [dict(heading_error=deg(3.))] * 20
          + [dict(heading_error=deg(-5.), lateral_offset=.2)] * 30 + [dict(heading_error=deg(1.))] * 10
          + [dict(heading_error=deg(20.))] * 20 + [dict(heading_error=deg(20.), still=True, speed=0.)] * 30
          + [dict(heading_error=deg(12.), still=True, speed=0., yaw_rate=deg(6.), gyro_heading=deg(g))
             for g in range(0, 40, 2)]
          + [dict(still=True, speed=0., gyro_heading=deg(40.), vibration_rms=.1)] * 40
          + [dict(heading_error=deg(-3.), lateral_offset=-.3)] * 40 + [dict(row_end_ahead_m=.1)] * 5
          + [dict(still=True, speed=0.)] * 10)


@pytest.mark.parametrize('mode', ['continuous', 'stop_turn', 'hybrid', 'hybrid_fallback'])
def test_side_check_off_is_inert(mode):
    plain, extreme = RowController(config_for(mode)), RowController(config_for(mode))
    t = 0.
    for k, overrides in enumerate(SCRIPT):
        t = round(t + TICK, 6)
        a = plain.step(t, snap(**overrides))
        b = extreme.step(t, side(left=-1., right=-1., left_x=.1, right_x=-.5, scan=k // 2,
                                 **{**overrides, 'yaw_rate': overrides.get('yaw_rate', 0.)}))
        assert (a.left, a.right, a.state, a.reason) == (b.left, b.right, b.state, b.reason), (k, a, b)
    assert plain.events == extreme.events and plain.counts == extreme.counts
    assert extreme.counts['side_stops'] == extreme.counts['side_inhibit'] == 0
    assert extreme.side_streak == 0 and extreme.side_scan is None          # never evaluated
    assert len(plain.events) >= 4                                         # the script exercised the machine


# 3 -------------------------------------------------------------------------------
def test_side_hard_requires_distinct_scans():
    controller = following()
    t = 0.
    for _ in range(5):                                   # the same scan repeated on 5 ticks: one confirmation
        t = round(t + TICK, 6)
        command = controller.step(t, side(left=.0, left_x=.3, scan=5))
        assert controller.state == 'FOLLOW' and command.left > 0 and command.right > 0
    assert controller.side_streak == 1
    t = round(t + TICK, 6)
    command = controller.step(t, side(left=.0, left_x=.3, scan=6))      # second DISTINCT scan
    assert controller.state == 'STOP_REQUESTED' and controller.reason == 'side_blocked'
    assert (command.left, command.right) == (0, 0) and controller.counts['side_stops'] == 1
    t = round(t + TICK, 6)
    controller.step(t, side(left=.4, scan=7))            # a clear scan resets the streak ...
    assert controller.side_streak == 0
    for k in range(40):                                  # ... but the pending terminal decision stands
        t = round(t + TICK, 6)
        controller.step(t, side(left=.4, scan=8 + k, still=True, speed=0.))
    assert controller.state == 'HOLD' and controller.reason == 'side_blocked' and controller.hold_terminal
    assert 'side_blocked' in TERMINAL_HOLDS


# 4 -------------------------------------------------------------------------------
def test_side_hard_refuses_start_and_stops_terminal():
    controller = RowController(config_for('hybrid_side'))
    controller.step(.05, side(left=.01, scan=1))                  # not still yet: waits in REACQUIRE
    controller.step(.10, side(left=.01, scan=2))
    assert controller.state == 'REACQUIRE'
    command = controller.step(.15, side(left=.01, scan=3, still=True))
    assert controller.state == 'HOLD' and controller.reason == 'side_blocked' and controller.hold_terminal
    assert (command.left, command.right) == (0, 0)
    t = .15
    for k in range(40):                                           # terminal: clear input never resumes
        t = round(t + TICK, 6)
        command = controller.step(t, side(scan=4 + k, still=True))
        assert (command.left, command.right) == (0, 0)
    assert controller.state == 'HOLD' and controller.reason == 'side_blocked'
    # Priority front -> row end -> side (keeps the wall_stop verdicts).
    blocked = RowController(config_for('hybrid_side'))
    blocked.step(.05, side(left=.01, scan=1))
    blocked.step(.10, side(left=.01, scan=2, still=True, front_clearance_m=.1))
    assert blocked.reason == 'front_blocked'
    for scenario, expected in ((dict(front_clearance_m=.2), 'front_blocked'),
                               (dict(row_end_ahead_m=.05), 'row_end_candidate'), (dict(), 'side_blocked')):
        controller = following()
        controller.step(.05, side(right=.0, right_x=-.2, scan=1))                 # first hard scan: pending
        assert controller.state == 'FOLLOW'
        controller.step(.10, side(right=.0, right_x=-.2, scan=2, **scenario))     # confirmed on the same tick
        assert controller.state == 'STOP_REQUESTED' and controller.reason == expected, scenario
    # REDUCE stops for a hard side violation too.
    controller = following()
    controller.step(.05, snap(heading_error=deg(20.)))
    assert controller.state == 'REDUCE'
    controller.step(.10, side(left=.0, scan=1, heading_error=deg(20.)))
    controller.step(.15, side(left=.0, scan=2, heading_error=deg(20.)))
    assert controller.state == 'STOP_REQUESTED' and controller.reason == 'side_blocked'


# 5 -------------------------------------------------------------------------------
@pytest.mark.parametrize('where,x', [('front', .5), ('rear', -.5), ('middle', .1)])
@pytest.mark.parametrize('which', ['left', 'right'])
@pytest.mark.parametrize('w_sign', [1, -1])
def test_side_swing_inhibit_front_rear_middle(where, x, which, w_sign):
    side_sign = 1 if which == 'left' else -1
    refused = {'front': (side_sign,), 'rear': (-side_sign,), 'middle': (1, -1)}[where]
    controller = following()
    keys = dict(left=.10, left_x=x) if which == 'left' else dict(right=.10, right_x=x)
    t, commands = 0., []
    for _ in range(10):                                  # soft (0.10 > hard 0.03): never a stop
        t = round(t + TICK, 6)
        commands.append(controller.step(t, side(scan=1, heading_error=deg(4. * w_sign), **keys)))
    assert controller.state == 'FOLLOW' and controller.side_soft[which]
    if w_sign in refused:
        assert all(c.reason == 'side_swing_inhibit' and c.left == c.right == 75 for c in commands)
        assert controller.counts['side_inhibit'] == 10 and controller.differential == 0.
    else:
        assert all(c.reason == 'continuous_correction' for c in commands)
        assert w_sign * (commands[-1].right - commands[-1].left) > 0 and controller.counts['side_inhibit'] == 0
    # Never the opposite differential (no braking reversal).
    assert all(w_sign * (c.right - c.left) >= 0 for c in commands)


def test_swing_inhibit_needs_a_soft_side():
    controller = following()
    t = 0.
    for _ in range(10):
        t = round(t + TICK, 6)
        command = controller.step(t, side(left=.25, left_x=.5, scan=1, heading_error=deg(4.)))
    assert command.reason == 'continuous_correction' and command.right > command.left


# 6 -------------------------------------------------------------------------------
def test_side_soft_hysteresis():
    controller = following()
    expected = [(.25, False), (.15, True), (.18, True), (.199, True), (.20, False), (.18, False), (.149, True),
                (math.inf, False)]
    for k, (clearance, soft) in enumerate(expected):
        controller.step(round(.05 * (k + 1), 6), side(left=clearance, left_x=.5, scan=k + 1))
        assert controller.side_soft['left'] is soft, (clearance, soft)
        assert controller.side_soft['right'] is False
    # Hysteresis is evaluated per DISTINCT scan: a repeated scan does not re-evaluate.
    controller.step(1., side(left=.10, left_x=.5, scan=len(expected)))
    assert controller.side_soft['left'] is False


# 7 -------------------------------------------------------------------------------
def test_side_bias_bounded_no_pivot():
    controller = RowController(config_for('hybrid'))
    values = (0., -0., 1e-300, -1e-300, .2, -.2, 3., -3., 1e9, math.inf, -math.inf)
    for lateral in values:
        for heading in (0., -0., .1, -.1, 3.):
            s = snap(lateral_offset=lateral, heading_error=heading)
            assert bits(controller.target_error(s)) == bits(controller.target_error(s, 0.))
            reference = heading - math.atan2(lateral, 3.)
            assert bits(controller.target_error(s)) == bits(math.atan2(math.sin(reference), math.cos(reference)))
    assert config_for('hybrid_side').side_bias_max_m == 0. and config_for('hybrid_side_bias').side_bias_max_m == .15
    # Left obstacle ahead, nearly hard: bias saturates at 0.15 m to the right; steering only.
    biased = following('hybrid_side_bias')
    plain = following('hybrid_side')
    t, cb, cp = 0., [], []
    for k in range(100):
        t = round(t + TICK, 6)
        cb.append(biased.step(t, side(left=.04, left_x=.5, scan=k // 2 + 1)))
        cp.append(plain.step(t, side(left=.04, left_x=.5, scan=k // 2 + 1)))
    assert biased.side_bias_m == pytest.approx(-.15)
    assert any(c.left > c.right for c in cb) and all(c.left >= c.right for c in cb)   # steers right (away)
    assert all(c.left == c.right == 75 for c in cp)                                    # no bias: deadband
    assert biased.state == 'FOLLOW' and biased.counts['turns'] == 0 and biased.counts['stops'] == 0
    assert math.degrees(math.atan2(.15, 3.)) < config_for('hybrid').turn_exit_deg
    # No bias for a return in the uncertain middle zone, or when the other side has no room.
    for keys in (dict(left=.04, left_x=.0), dict(left=.04, left_x=.5, right=.25, right_x=.5)):
        controller = following('hybrid_side_bias')
        controller.step(.05, side(scan=1, **keys))
        assert controller.side_bias_m == 0.
    # The pivot aims at the UNBIASED centreline target.
    controller = RowController(config_for('hybrid_side_bias'))
    s = side(left=.04, left_x=.5, scan=1, still=True, speed=0., heading_error=deg(25.), lateral_offset=.1)
    controller.step(.05, s)
    assert controller.state == 'TURN' and controller.side_bias_m == pytest.approx(-.15)
    assert controller.turn['target'] == math.degrees(controller.target_error(s))


# 8 -------------------------------------------------------------------------------
def test_stop_turn_side_uses_hard_only():
    config = config_for('stop_turn_side')
    assert config.mode == 'stop_turn' and config.side_clearance_check
    assert (config.turn_enter_deg, config.max_turns_per_window) == (4., 8)       # base leniency kept
    controller = following('stop_turn_side')
    t = 0.
    for k in range(20):                                     # soft side + small error: equal duty, no inhibit
        t = round(t + TICK, 6)
        command = controller.step(t, side(left=.08, left_x=.5, scan=k + 1, heading_error=deg(3.)))
        assert command.reason == 'straight_equal_duty' and command.left == command.right
    assert controller.counts['side_inhibit'] == 0
    controller.step(round(t + TICK, 6), side(left=.02, scan=100))
    controller.step(round(t + 2 * TICK, 6), side(left=.02, scan=101))
    assert controller.state == 'STOP_REQUESTED' and controller.reason == 'side_blocked'


# 9 -------------------------------------------------------------------------------
def test_side_yaw_compensation():
    # raw 0.06 m > hard 0.03 m; sweep of the part at x = 0.6 m: 0.2 rad/s * (0.6 + 0.25) m * 0.2 s = 0.034 m
    for yaw_rate, blocked in ((.2, True), (-.2, True), (0., False), (.1, False)):
        controller = following()
        for k in range(3):
            controller.step(round(.05 * (k + 1), 6), side(left=.06, left_x=.6, scan=k + 1, yaw_rate=yaw_rate,
                                                          lidar_age_s=.2))
        assert (controller.reason == 'side_blocked') is blocked, yaw_rate
    controller = following()
    controller.step(.05, side(left=.06, left_x=XC, scan=1, yaw_rate=.2, lidar_age_s=.2))
    assert controller.side_comp['left'] == pytest.approx(.06 - .2 * SD * .2)     # sd always counted


def test_side_rule_configuration_guards():
    assert not ControlConfig().side_clearance_check
    for bad in (dict(side_hard_margin_m=0.), dict(side_hard_margin_m=.2), dict(side_soft_exit_m=.15),
                dict(side_confirm_scans=0), dict(side_bias_max_m=-.1), dict(side_bias_max_m=.31)):
        with pytest.raises(ValueError, match='side_rule_invalid'):
            ControlConfig(**bad)
    assert set(SIDE_VARIANTS) <= set(VARIANTS) and set(SIDE_VARIANTS) <= set(ALL_MODES)
    assert {MODE_LABEL[m] for m in SIDE_VARIANTS} == {'A_side', 'B_side', 'C_side', 'C_side_bias'}
    assert 'side_blocked' in SCENARIOS['canopy_narrow_aisle'].acceptable_holds


# 10 ------------------------------------------------------------------------------
def test_canopy_seed6_side_variants_do_not_deepen_the_spawn_contact():
    """canopy_narrow_aisle seed 6 starts 4 mm inside a branch; base A/C deepen it while turning."""
    runs = {c.mode: run_episode(c) for c in episodes(seeds=(6,), names=['canopy_narrow_aisle'],
                                                     modes=('continuous', 'hybrid', 'continuous_side', 'hybrid_side'))}
    for mode in ('continuous', 'hybrid'):
        base = runs[mode]
        assert base['start_branch_clearance_m'] < 0                       # spawn overlap (truth)
        assert base['branch_intrusion_increase_m'] > 0 and base['branch_contact_samples_moving'] > 0
        assert base['side_stops'] == base['side_inhibit_ticks'] == 0 and base['side_trigger'] is None
    for mode in ('continuous_side', 'hybrid_side'):
        result = runs[mode]
        assert result['branch_intrusion_increase_m'] == 0 and result['branch_contact_samples_moving'] == 0
        assert result['collisions'] == 0
        assert result['side_trigger'] is not None and result['side_trigger_truth_m'] is not None


# 11 ------------------------------------------------------------------------------
DEFAULT_SCENE_SHA256 = '9c88d54c5e5bb07823897723c86bbcfdeb5321f1e4b9f2e0a155b1494f1910ea'   # main 999a863


def scene_digest(world):
    h = hashlib.sha256()
    for group in (world.trunks, world.branches):
        h.update(struct.pack('<q', len(group)))
        for c in group:
            h.update(struct.pack('<3d', c.x, c.y, c.radius))
    h.update(struct.pack('<q', len(world.chunks)))
    for chunk in world.chunks:
        h.update(struct.pack('<qq', len(chunk.circles), len(chunk.segments)))
        for c in chunk.circles:
            h.update(struct.pack('<3d', c.x, c.y, c.radius))
        for seg in chunk.segments:
            h.update(struct.pack('<4d', *seg.start, *seg.end))
    for seg in world.walls:
        h.update(struct.pack('<4d', *seg.start, *seg.end))
    return h.hexdigest()


def test_default_scene_is_unchanged_by_the_world_extras():
    world = build_world(WorldConfig())
    assert scene_digest(world) == DEFAULT_SCENE_SHA256 and world.extras == ()
