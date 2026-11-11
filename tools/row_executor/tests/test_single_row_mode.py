"""Executor mode 'single_row' (2026-11-02): config rules, vendored SingleRowController in the Runner, optional row length
and its backstop, terminal reasons and stop categories, status / log telemetry, the native cross-check and the operator
requests. Vendor path only (core.load_vendor); the SIM-tree path and the closed loop are in test_single_row_sim.py.

The synthetic vehicle never moves (static orchard scan), so a real drive ends with drive_no_response; stub controllers
produce the other terminal reasons. No hardware, no ROS, no network.
"""
import json
import math
import os
import types
import unittest
from dataclasses import replace

import rowexec_testlib as L
from row_executor import config as C
from row_executor import core
from row_executor import executor_main as M
from row_executor.bridge_client import ARMED, LATCHED
from row_executor.config import ExecutorConfig
from row_executor.core import ACTIVE, ENDED, PAUSED, Runner
from row_executor.navigation_observation import (NativeNavigationGate, NativeNavigationObservation, NativePose2D,
                                                 NativeSourceBinding)
from test_executor_loop import Rig

SCENE = L.raw_scan(L.orchard_trunks())
BINDING = NativeSourceBinding('a' * 64, 'b' * 64, 'c' * 64, 'map')


def sr_config(**over):
    """A SHADOW single-row config (the max yaw rate 3.6 = 0.6 x nominal 2 x 0.15 x 20 is the SingleRowConfig rule)."""
    base = dict(mode='single_row', require_native_navigation=False, max_yaw_rate_deg_s=3.6)
    base.update(over)
    return ExecutorConfig(**base).validate()


def sr_motor_config(**over):
    """A feasible single-row MOTOR config (example numbers for the tests, not field values): breakaway 20 + margin 10 =
    30 = base_min 50 - differential 20; authority 6.0 deg/s at +-20, max yaw 3.6; base 50-60, duty cap 80."""
    base = dict(mode='single_row', require_native_navigation=False, motor_output_enabled=True, base_duty_pct=50,
                reduce_duty_pct=50, max_differential_pct=20, duty_cap_pct=80, diff_cap_pct=40,
                single_row_base_min_pct=50, single_row_base_max_pct=60, breakaway_duty_pct=20, inner_margin_pct=10,
                v_measured_m_s=.42, measured_yaw_at_max_diff_deg_s=6., max_yaw_rate_deg_s=3.6,
                measured_stop_distance_m=.24)
    base.update(over)
    return ExecutorConfig(**base).validate()


def problems(**over):
    base = dict(mode='single_row', require_native_navigation=False, max_yaw_rate_deg_s=3.6)
    base.update(over)
    try:
        ExecutorConfig(**base).validate()
    except C.ConfigError as exc:
        return exc.problems
    return []


def motor_problems(**over):
    try:
        sr_motor_config(**over)
    except C.ConfigError as exc:
        return exc.problems
    return []


class StubController:
    """Single-row stand-in: FOLLOW at the given duty, or a terminal HOLD / pending stop with the given reason."""

    def __init__(self, left=55, right=55, state='FOLLOW', reason='single_row_follow', pending=None, phase='FOLLOW'):
        self.out = (left, right, state, reason)
        self.state, self.reason = 'REACQUIRE', 'start'
        self.pending = pending
        self.phase = phase
        self.phase_events = [(0., 'APPROACH_END', 'remaining_1.00_m')]

    def step(self, now, snapshot):
        left, right, state, reason = self.out
        self.state, self.reason = state, reason
        return types.SimpleNamespace(left=left, right=right, state=state, reason=reason)

    def current_phase(self):
        return self.phase

    def telemetry(self):
        return dict(phase=self.phase, lateral_reference_m=.05, lookahead_error_deg=1.5, differential_pct=3.,
                    raw_differential_pct=3.2, differential_limit_pct=20., saturated=False, saturation_fraction=.1,
                    authority_state='OK', bias_pct=1.25, base_duty_pct=55., target_speed_m_s=.4, gyro_trust=1.,
                    target_yaw_rate_deg_s=.8)


def run_static(cfg, controller=None, before=3., hold_for=6., row_len_m=None):
    runner = Runner(cfg, controller_factory=controller)
    driver = L.FakeTimeDriver(runner, lambda t: SCENE, t0=2000.)
    runner.start_run(driver.t, row_len_m)
    out = driver.run(before)
    out += driver.run(hold_for, hold=True)
    return runner, driver, out


class ConfigTest(unittest.TestCase):
    def test_untouched_block_keeps_the_existing_digest_and_dump(self):
        cfg = ExecutorConfig()
        data = cfg.to_dict()
        self.assertFalse(set(C.SINGLE_ROW_KEYS) & set(data))
        self.assertEqual(len(C.SINGLE_ROW_KEYS), 16)     # 9 + 4 estimator / first-press keys (dev round 1) + 3 (round 2)
        self.assertTrue(all(k.startswith('single_row_') for k in C.SINGLE_ROW_KEYS))
        self.assertFalse(set(C.SINGLE_ROW_KEYS) & set(C.TURN_KEYS))       # not counted as turn keys
        import hashlib
        text = json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False)
        self.assertEqual(cfg.digest(), hashlib.sha256(text.encode()).hexdigest())

    def test_touched_block_round_trip_and_digest(self):
        cfg = sr_config(single_row_target_speed_m_s=.35, single_row_max_row_length_m=90.)
        data = cfg.to_dict()
        self.assertTrue(set(C.SINGLE_ROW_KEYS) <= set(data))
        self.assertEqual(C.loads(C.dumps(cfg)), cfg)
        self.assertNotEqual(cfg.digest(), sr_config().digest())
        self.assertNotEqual(sr_config().digest(), ExecutorConfig().digest())       # the mode is in the digest

    def test_example_config_loads_as_shadow_single_row(self):
        path = os.path.join(L.TOOLS, 'row_executor', 'config.single_row_shadow.example.json')
        cfg = C.load_config(path)
        self.assertEqual(cfg.mode, 'single_row')
        self.assertFalse(cfg.motor_output_enabled)
        self.assertFalse(cfg.require_native_navigation)
        self.assertEqual(cfg.feasibility()['status'], 'unknown')

    def test_native_provider_requirement_is_never_silently_ignored(self):
        found = problems(require_native_navigation=True)
        self.assertTrue(any('require_native_navigation false explicitly' in p for p in found))
        # the other modes keep the existing behaviour (required by default, valid)
        self.assertTrue(ExecutorConfig().require_native_navigation)
        ExecutorConfig().validate()

    def test_turn_and_row_end_stop_override_are_refused(self):
        self.assertTrue(any('turn_assist_enabled is refused' in p for p in problems(turn_assist_enabled=True)))
        self.assertTrue(any('row_end_stop_m' in p for p in problems(controller_overrides={'row_end_stop_m': .5})))

    def test_single_row_estimator_options_reach_the_estimator_only_in_single_row(self):
        cfg = sr_config()
        est = cfg.estimator_overrides()
        self.assertEqual((est['acquire_behind_m'], est['acquire_axis_window_deg'], est['row_end_segment_span_m'],
                          est['band_rotate_about_vehicle'], est['row_end_stagger_gap_m'], est['row_end_last_trunk']),
                         (2.5, 30., 2., True, 4., True))
        self.assertIsNone(sr_config(single_row_end_stagger_gap_m=None).estimator_overrides()['row_end_stagger_gap_m'])
        self.assertIs(sr_config(single_row_band_about_vehicle=False).estimator_overrides()['band_rotate_about_vehicle'],
                      False)
        off = sr_config(single_row_acquire_behind_m=None, single_row_end_segment_span_m=None,
                        single_row_end_stagger_gap_m=None).estimator_overrides()
        self.assertEqual((off['acquire_behind_m'], off['row_end_segment_span_m']), (None, None))
        for mode in ('continuous_side', 'continuous'):
            other = ExecutorConfig(mode=mode).estimator_overrides()
            self.assertFalse({'acquire_behind_m', 'acquire_axis_window_deg', 'row_end_segment_span_m',
                              'band_rotate_about_vehicle', 'row_end_stagger_gap_m', 'row_end_last_trunk'} & set(other))
        runner = Runner(cfg)
        ec = runner.estimator_config
        self.assertEqual((ec.acquire_behind_m, ec.acquire_axis_window_deg, ec.row_end_segment_span_m,
                          ec.band_rotate_about_vehicle, ec.row_end_stagger_gap_m, ec.row_end_last_trunk),
                         (2.5, 30., 2., True, 4., True))
        self.assertIsNone(Runner(ExecutorConfig()).estimator_config.row_end_segment_span_m)
        self.assertTrue(any('single_row_acquire_window_deg' in p for p in problems(single_row_acquire_window_deg=20.)))
        self.assertTrue(problems(single_row_initial_press_grace_s=4.))
        self.assertTrue(any('single_row_end_stagger_gap_m needs' in p
                            for p in problems(single_row_end_segment_span_m=None)))

    def test_base_range_rules(self):
        self.assertTrue(any('outside [single_row_base_min_pct' in p for p in problems(base_duty_pct=60)))
        self.assertTrue(any('outside [single_row_base_min_pct' in p for p in problems(single_row_base_min_pct=56)))
        self.assertTrue(any('duty_cap_pct 75 < single_row_base_max_pct' in p
                            for p in problems(single_row_base_max_pct=60)))
        self.assertTrue(any('inner wheel driving' in p for p in problems(single_row_base_min_pct=20)))
        self.assertEqual(problems(single_row_base_max_pct=60, duty_cap_pct=80), [])

    def test_speed_bias_start_yaw_and_cap_rules(self):
        self.assertTrue(any('min / approach speed' in p for p in problems(single_row_approach_speed_m_s=.5)))
        self.assertTrue(any('min / approach speed' in p for p in problems(single_row_min_speed_m_s=.45)))
        self.assertTrue(any('above max_speed_m_s' in p for p in problems(single_row_target_speed_m_s=.6)))
        self.assertTrue(any('single_row_bias_max_pct' in p for p in problems(single_row_bias_max_pct=21.)))
        self.assertTrue(any('guard_heading_deg' in p for p in problems(single_row_start_max_deg=26.)))
        self.assertTrue(any('nominal authority' in p for p in problems(max_yaw_rate_deg_s=3.7)))
        self.assertTrue(any('max_row_len_m' in p for p in problems(single_row_max_row_length_m=301.)))
        self.assertTrue(any('outside' in p for p in problems(single_row_start_max_deg=30.)))      # < heading limit 30

    def test_inner_wheel_rules_hold_at_the_lowest_base(self):
        self.assertEqual(motor_problems(), [])
        found = motor_problems(single_row_base_min_pct=49, base_duty_pct=50)
        self.assertTrue(any('single_row inner wheel single_row_base_min_pct-max_differential = 29' in p for p in found))
        # rolling threshold measured: the moving rule uses it, the standstill start still needs breakaway + margin -
        # at the LOWEST base too (the start base 55 passes the existing rules, base_min 50 does not)
        self.assertEqual(motor_problems(rolling_breakaway_duty_pct=20), [])
        found = problems(breakaway_duty_pct=45, rolling_breakaway_duty_pct=20, base_duty_pct=55,
                         single_row_base_min_pct=50, single_row_base_max_pct=55, duty_cap_pct=75)
        self.assertEqual(len(found), 1, found)            # a known breakaway makes it a config problem (any profile)
        self.assertIn('single_row_base_min_pct 50 < breakaway+margin = 55 (a start from standstill', found[0])
        self.assertEqual(problems(breakaway_duty_pct=45, rolling_breakaway_duty_pct=20, base_duty_pct=55,
                                  single_row_base_min_pct=55, single_row_base_max_pct=55, duty_cap_pct=75), [])

    def test_motor_rules_stay(self):
        found = motor_problems(single_row_end_stop_offset_m=.3)
        self.assertTrue(any('front would stop beyond the observed row end' in p for p in found))
        # steering GO rule of the continuous modes applies to single_row
        found = motor_problems(measured_yaw_at_max_diff_deg_s=3.9, max_yaw_rate_deg_s=2.3)
        self.assertTrue(any('steering GO rule' in p for p in found))
        found = motor_problems(max_differential_pct=21, diff_cap_pct=42, duty_cap_pct=81, base_duty_pct=51,
                               reduce_duty_pct=51, single_row_base_min_pct=51)
        self.assertTrue(any('max_differential_pct 21 outside [10, 20]' in p for p in found))
        # day-2 N1 limits unchanged
        self.assertTrue(any('max_speed_m_s' in p for p in motor_problems(max_speed_m_s=.6)))

    def test_controller_settings_map_to_single_row_config(self):
        cfg = sr_config(single_row_target_speed_m_s=.35, single_row_min_speed_m_s=.15, single_row_approach_speed_m_s=.18,
                        single_row_bias_max_pct=10., single_row_end_stop_offset_m=.7,
                        controller_overrides={'lookahead_m': 2.})
        mode, settings = cfg.controller_settings()
        self.assertEqual(mode, 'single_row')
        self.assertEqual((settings['cruise_speed_m_s'], settings['min_speed_m_s'], settings['authority_slow_speed_m_s'],
                          settings['approach_speed_m_s'], settings['bias_max_pct'], settings['end_stop_offset_m'],
                          settings['lookahead_m'], settings['base_min_pct'], settings['base_max_pct']),
                         (.35, .15, .15, .18, 10., .7, 2., 45, 55))
        self.assertAlmostEqual(C.single_row_time_cap_speed(cfg), .15)
        # the other modes keep their settings (no single-row key leaks into config_for)
        self.assertNotIn('cruise_speed_m_s', ExecutorConfig().controller_settings()[1])


class RunnerTest(unittest.TestCase):
    def test_runner_builds_the_vendored_single_row_controller(self):
        cfg = sr_config(single_row_target_speed_m_s=.35, single_row_bias_max_pct=10.)
        runner = Runner(cfg)
        self.assertIsNotNone(runner.single_row)
        self.assertTrue(core._inside(runner.single_row.__file__, core.VENDOR_DIR))
        cc = runner.controller_config
        self.assertEqual(type(cc).__name__, 'SingleRowConfig')
        self.assertEqual((cc.mode, cc.side_clearance_check, cc.pivot_failure_fallback), ('continuous', True, False))
        self.assertEqual((cc.base_duty_pct, cc.max_differential_pct, cc.cruise_speed_m_s, cc.bias_max_pct,
                          cc.lidar_max_age_s, cc.imu_max_age_s), (55, 20, .35, 10., .45, .4))
        controller = runner._new_controller()
        self.assertEqual(type(controller).__name__, 'SingleRowController')
        self.assertIn('row_end', runner.terminal_holds)
        self.assertIn('steering_authority_insufficient', runner.terminal_holds)
        # the other modes are unchanged
        self.assertIsNone(Runner(ExecutorConfig()).single_row)

    def test_optional_row_length_and_backstop_bounds(self):
        runner = Runner(sr_config(single_row_max_row_length_m=100.))
        runner.start_run(10.)
        bs = runner.backstop.summary()
        self.assertEqual(bs['limit_m'], 100.)
        self.assertAlmostEqual(bs['time_limit_s'], 1.15 * 100. / .2, places=6)
        self.assertEqual(bs['single_row'], dict(cap_m=100., ref_speed_m_s=.2, row_len_m=None, bound='cap'))
        runner.end_run(11.)
        runner.start_run(12., 40.)                                # an entered length adds L - 2 m
        bs = runner.backstop.summary()
        self.assertEqual((bs['limit_m'], bs['single_row']['bound']), (38., 'row_length'))
        self.assertAlmostEqual(bs['time_limit_s'], 1.15 * 38. / .2, places=6)
        runner.end_run(13.)
        runner.start_run(14., 150.)                               # longer than the cap: the cap bounds it
        self.assertEqual(runner.backstop.summary()['limit_m'], 100.)
        runner.end_run(15.)
        for bad in (2.5, 301., float('nan'), True, 'x'):
            with self.assertRaises(ValueError):
                runner.start_run(16., bad)
        self.assertEqual(runner.state, ENDED)                     # a refused Start changes nothing
        with self.assertRaises(ValueError):                       # the other modes still need a length
            Runner(ExecutorConfig()).start_run(1., None)

    def test_backstop_end_is_never_a_row_end(self):
        cfg = sr_config(single_row_max_row_length_m=3.)
        runner = Runner(cfg, estimator_factory=lambda: MovingEstimator(.5), controller_factory=StubController)
        driver = L.FakeTimeDriver(runner, lambda t: SCENE, t0=100.)
        runner.start_run(driver.t)
        out = driver.run(1.)
        out += driver.run(12., hold=True)
        end = [d for _, d in out if d.state == ENDED][0]
        self.assertEqual(end.reason, 'row_distance_backstop')
        sr = end.summary['single_row']
        self.assertEqual((sr['stop_category'], sr['stop_label']), ('OPERATOR_OR_BACKSTOP', 'OPERATOR_OR_BACKSTOP'))

    def test_controller_terminal_reasons_end_the_run_with_their_category(self):
        for reason, category in (('row_end', 'ROW_END'), ('obstacle_ahead', 'OBSTACLE'),
                                 ('end_structure_ahead', 'END_STRUCTURE'),
                                 ('steering_authority_insufficient', 'CONTROL_AUTHORITY'),
                                 ('start_misaligned', 'CONTROL_AUTHORITY'), ('stop_not_observed', 'CONTROL_AUTHORITY'),
                                 ('corridor_not_acquired', 'PERCEPTION_OR_LOCALISATION')):
            with self.subTest(reason=reason):
                runner, _, out = run_static(sr_config(), lambda r=reason: StubController(0, 0, 'HOLD', r))
                self.assertEqual(runner.state, ENDED)
                self.assertEqual(runner.end_reason, reason)
                sr = out[-1][1].summary['single_row']
                self.assertEqual((sr['stop_reason'], sr['stop_category']), (reason, category))
                self.assertEqual(sr['first_stop']['reason'], reason)

    def test_initial_press_is_kept_while_held_through_the_input_warm_up(self):
        # SIM dev round 1 (lidar_latency): a first press made before the inputs were fresh for fresh_recover_s starts
        # the run once they are, as long as it stays held (single_row_initial_press_grace_s, default 1 s)
        cfg = sr_config()
        runner = Runner(cfg, controller_factory=StubController)
        driver = L.FakeTimeDriver(runner, lambda t: SCENE, t0=400.)
        runner.start_run(driver.t)
        out = driver.run(.2) + driver.run(1.6, hold=True)
        pending = [d for _, d in out if d.summary.get('press_pending')]
        self.assertTrue(pending)
        self.assertTrue(all(d.state == PAUSED and d.reason == 'await_hold' and d.left == d.right == 0 for d in pending))
        t, dec = L.first(out, lambda d: d.state == ACTIVE)
        self.assertIsNotNone(t)
        fresh_since = 400. + .05                               # the first tick with fresh inputs
        self.assertGreaterEqual(t, fresh_since + cfg.fresh_recover_s - 1e-6)
        self.assertLessEqual(t, 400.2 + cfg.single_row_initial_press_grace_s + 1e-6)
        self.assertEqual(runner.resumes, 1)

    def test_initial_press_lapses_on_release_or_after_the_grace(self):
        runner = Runner(sr_config(), controller_factory=StubController)
        driver = L.FakeTimeDriver(runner, lambda t: SCENE, t0=500.)
        runner.start_run(driver.t)
        out = driver.run(.2) + driver.run(.3, hold=True) + driver.run(1.5, hold=False)
        self.assertTrue(all(d.state == PAUSED for _, d in out))          # released before the inputs were fresh
        self.assertEqual((runner.reason, runner.resumes), ('await_hold', 0))
        out = driver.run(.3, hold=True)                                   # a NEW press after the warm-up: normal start
        self.assertEqual(runner.state, ACTIVE)
        runner = Runner(sr_config(single_row_initial_press_grace_s=.3), controller_factory=StubController)
        driver = L.FakeTimeDriver(runner, lambda t: SCENE, t0=600.)
        runner.start_run(driver.t)
        out = driver.run(.2) + driver.run(2., hold=True)                  # held, but the grace runs out first
        self.assertTrue(all(d.state == PAUSED for _, d in out))
        self.assertEqual(runner.reason, 'press_ignored_inputs_not_fresh')
        runner = Runner(sr_config(single_row_initial_press_grace_s=0.), controller_factory=StubController)
        driver = L.FakeTimeDriver(runner, lambda t: SCENE, t0=700.)
        runner.start_run(driver.t)
        driver.run(.2)
        driver.run(2., hold=True)                                         # 0 = the original rule: ignored at once
        self.assertEqual((runner.state, runner.reason), (PAUSED, 'press_ignored_inputs_not_fresh'))

    def test_a_resume_after_a_stale_pause_never_uses_the_initial_grace(self):
        runner = Runner(sr_config(), controller_factory=StubController)
        driver = L.FakeTimeDriver(runner, lambda t: SCENE, t0=800.)
        runner.start_run(driver.t)
        driver.run(1.5)
        driver.run(.5, hold=True)
        self.assertEqual(runner.state, ACTIVE)
        driver.scan_on = False
        driver.run(1., hold=True)
        self.assertEqual((runner.state, runner.reason), (PAUSED, 'stale_lidar'))
        driver.scan_on = True
        driver.run(.1, hold=False)
        out = driver.run(1.5, hold=True)                                  # pressed before fresh again: ignored
        self.assertTrue(all(d.state == PAUSED and not d.summary.get('press_pending') for _, d in out))
        self.assertEqual(runner.reason, 'press_ignored_inputs_not_fresh')

    def test_pending_terminal_stop_outlives_a_released_hold(self):
        runner = Runner(sr_config(), controller_factory=lambda: StubController(
            0, 0, 'STOP_REQUESTED', 'row_end', pending=('hold', 'row_end'), phase='STOPPING'))
        driver = L.FakeTimeDriver(runner, lambda t: SCENE, t0=300.)
        runner.start_run(driver.t)
        driver.run(3.)
        driver.run(.5, hold=True)
        out = driver.run(.2, hold=False)                       # released during the terminal stop
        self.assertEqual(runner.state, ENDED)
        self.assertEqual(runner.end_reason, 'row_end')
        self.assertEqual(runner.end_note, 'terminal_stop_pending_at_hold_released')
        self.assertEqual(out[-1][1].summary['single_row']['stop_category'], 'ROW_END')

    def test_real_controller_static_scene_forward_only_then_drive_no_response(self):
        runner, _, out = run_static(sr_config(), before=3., hold_for=8.)
        driving = [d for _, d in out if d.left or d.right]
        self.assertTrue(driving)
        for d in driving:                                      # forward only, equal duty (rolling start: no roll)
            self.assertGreater(min(d.left, d.right), 0)
            self.assertEqual(d.left, d.right)
            self.assertEqual(d.summary['controller_state'], 'FOLLOW')
            self.assertEqual(d.summary['single_row']['phase'], 'FOLLOW')
        self.assertEqual(runner.end_reason, 'drive_no_response')
        sr = out[-1][1].summary['single_row']
        self.assertEqual((sr['stop_category'], sr['stop_label'], sr['phase']), ('CONTROL_AUTHORITY', 'DRIVE', 'STOPPED'))
        for key in ('lateral_error_m', 'heading_error_deg', 'confidence', 'left_pct', 'right_pct', 'differential_pct',
                    'saturated', 'authority_state', 'bias_pct', 'base_duty_pct', 'row_end_ahead_m', 'end_remaining_m',
                    'distance_m', 'speed_m_s', 'stop_reason', 'phase_events', 'localisation'):
            self.assertIn(key, sr)
        self.assertEqual(sr['confidence']['level'], 'OK')
        self.assertEqual(sr['localisation'], 'SENSOR_ONLY_SINGLE_ROW')
        geo = runner.single_row_geometry()
        self.assertEqual(geo['frame'], 'estimator_odometry_run_start')
        self.assertAlmostEqual(geo['base'][0], -L.LIDAR_X, places=2)   # base 0.18 m behind the LiDAR at the start
        self.assertAlmostEqual(geo['band']['width_m'], 3.5, delta=.4)
        self.assertLessEqual(len(geo['path']), core.SR_PATH_POINTS)

    def test_stop_categories(self):
        mod = Runner(sr_config()).single_row
        cat = core.single_row_stop_category
        self.assertEqual(cat(mod, 'await_hold'), (None, None, None))
        self.assertEqual(cat(mod, 'press_ignored_inputs_not_fresh'), (None, None, None))
        self.assertEqual(cat(mod, 'stale_lidar')[:2], ('PERCEPTION_OR_LOCALISATION', 'SENSOR'))
        self.assertEqual(cat(mod, 'hold_released')[0], 'OPERATOR_OR_BACKSTOP')
        # in the vendored table since 2026-11-02 (new entries only, the same categories as the executor display table)
        self.assertEqual(cat(mod, 'operator_stop'), ('OPERATOR_OR_BACKSTOP', 'OPERATOR_OR_BACKSTOP',
                                                     'single_row.stop_category'))
        self.assertEqual(cat(mod, 'reacquire_needs_press')[0], 'PERCEPTION_OR_LOCALISATION')
        self.assertEqual(cat(mod, 'stall_guard')[:2], ('PERCEPTION_OR_LOCALISATION', 'SENSOR'))
        for reason, label in core.SINGLE_ROW_EXECUTOR_CATEGORIES.items():      # never re-mapped by the table
            self.assertEqual(cat(mod, reason)[0], core.SINGLE_ROW_SUB_LABELS.get(label, label), reason)
        self.assertEqual(cat(mod, 'bridge_error')[0], 'SYSTEM_FAULT')
        self.assertEqual(cat(mod, 'guard_violation:pivot_command')[0], 'CONTROL_AUTHORITY')
        self.assertEqual(cat(mod, 'native_cross_check_cost_occupied')[0], 'OBSTACLE')
        self.assertEqual(cat(mod, 'something_new'), ('UNCLASSIFIED', None, 'none'))
        # every vendored table entry passes through (no re-mapping); only DRIVE / SENSOR become the PREREG enum
        for reason, label in mod.STOP_CATEGORIES.items():
            self.assertEqual(cat(mod, reason)[1], label)
            self.assertEqual(cat(mod, reason)[0], core.SINGLE_ROW_SUB_LABELS.get(label, label))


class MovingEstimator:
    """Snapshot of a vehicle moving straight at `speed` in a valid corridor (backstop test)."""

    def __init__(self, speed):
        self.speed, self.t0 = speed, None

    def on_imu(self, *a):
        pass

    def on_scan(self, *a):
        pass

    def snapshot(self, now):
        if self.t0 is None:
            self.t0 = now
        return dict(lidar_age_s=.05, imu_age_s=.02, still=False, still_s=0., speed=self.speed, odom_valid=True,
                    distance_m=self.speed * (now - self.t0), yaw_rate=0., corridor_valid=True, corridor_reason='fake',
                    heading_error=0., lateral_offset=0., corridor_width=3.6, corridor_age_s=.1, row_end_ahead_m=None)


def observation(sequence=1, **kwargs):
    value = NativeNavigationObservation(BINDING, sequence, 0., 'TRACKING',
                                        NativePose2D(0., 0., 0., age_s=0., quality='QUALIFIED_LOCAL'), True, 100,
                                        'CLEAR', sequence)
    return replace(value, **kwargs)


class LoopTest(unittest.TestCase):
    def begin(self, rig, row_len_m=None):
        fut = rig.loop.submit('start', row_len_m=row_len_m)
        rig.run(3.)
        self.assertTrue(fut.result()['ok'], fut.result())
        if rig.loop.motor_allowed:
            arm = rig.loop.submit('arm')
            rig.run(.1)
            self.assertTrue(arm.result()['ok'], arm.result())
        rig.press('sr-press-1')

    def test_start_request_row_length_is_optional_only_in_single_row(self):
        rig = Rig(cfg=sr_config())
        for value, ok in ((None, True), ('30', False), (True, False), (float('inf'), False)):
            fut = rig.loop.submit('start', row_len_m=value)
            rig.run(.1)
            self.assertEqual(fut.result()['ok'], ok, (value, fut.result()))
            if ok:
                rig.loop.submit('end')
                rig.run(.1)
        fut = rig.loop.submit('start', row_len_m=45.)
        rig.run(.1)
        self.assertTrue(fut.result()['ok'])
        self.assertEqual(rig.loop.runner.backstop.summary()['limit_m'], 43.)
        other = Rig()                                            # legacy continuous_side: still required
        fut = other.loop.submit('start', row_len_m=None)
        other.run(.1)
        self.assertEqual(fut.result()['error'], 'row_len_m_required')

    def test_shadow_status_log_and_sensor_only_localisation(self):
        rig = Rig(cfg=sr_config())
        self.begin(rig)
        rig.run(2., hold=True)
        status = rig.loop.status()
        self.assertEqual(status['controller_mode'], 'single_row')
        self.assertEqual(status['native_navigation']['state'], 'SENSOR_ONLY_SINGLE_ROW')
        self.assertTrue(status['native_navigation']['allow_motion'])
        self.assertFalse(status['native_navigation']['native_required'])
        self.assertIsNone(status['native_navigation']['native_cross_check'])
        self.assertEqual(status['single_row']['localisation'], 'SENSOR_ONLY_SINGLE_ROW')
        self.assertEqual(status['single_row']['phase'], 'FOLLOW')
        self.assertIsNotNone(status['single_row_geometry']['band'])
        self.assertEqual(status['single_row_settings']['localisation'], 'SENSOR_ONLY_SINGLE_ROW')
        self.assertFalse(status['turn_observe_available'])
        ticks = [r for r in rig.loop.log.records if r['k'] == 'tick']
        self.assertTrue(all('sr' in r for r in ticks if r['state'] in ('ACTIVE', 'PAUSED')))
        self.assertEqual(ticks[-1]['sr']['ph'], 'FOLLOW')
        self.assertTrue(any(r['L'] > 0 for r in ticks))
        fut = rig.loop.submit('turn_observe', params={})
        rig.run(.1)
        self.assertEqual(fut.result()['error'], 'turn_observe_not_in_single_row_mode')

    def test_native_cross_check_is_reported_and_only_a_valid_occupied_cost_pauses(self):
        gate = NativeNavigationGate(BINDING, .45, test_only=True)
        rig = Rig(cfg=sr_config(), navigation_gate=gate, navigation_test_mode=True)
        self.begin(rig)
        out = rig.run(1., hold=True)                            # no observation: WAIT, reported only
        self.assertTrue(any(d.state == ACTIVE and d.left > 0 for d in out))
        cross = rig.loop.status()['native_navigation']['native_cross_check']
        self.assertEqual(cross['reason'], 'NO_OBSERVATION')
        gate.offer(observation(1, forward_hazard='UNKNOWN'), rig.clock.t)
        out = rig.run(.2, hold=True)
        self.assertEqual(rig.loop.runner.state, ACTIVE)          # COST_UNKNOWN is never free, but not a veto either
        self.assertEqual(rig.loop.status()['native_navigation']['native_cross_check']['reason'], 'COST_UNKNOWN')
        gate.offer(observation(2, forward_hazard='OCCUPIED'), rig.clock.t)
        rig.run(.1, hold=True)
        self.assertEqual(rig.loop.runner.state, PAUSED)
        self.assertEqual(rig.loop.runner.reason, 'native_cross_check_cost_occupied')
        self.assertEqual(rig.loop.status()['single_row']['stop_category'], 'OBSTACLE')
        self.assertFalse(rig.loop.hold.active(rig.clock.t))      # the press was retired (a new press is needed)

    def test_untrusted_generated_provider_never_vetoes(self):
        gate = NativeNavigationGate(BINDING, .45, test_only=True)
        rig = Rig(cfg=sr_config(), navigation_gate=gate, navigation_test_mode=False)
        self.begin(rig)
        gate.offer(observation(1, forward_hazard='OCCUPIED'), rig.clock.t)
        rig.run(.5, hold=True)
        self.assertEqual(rig.loop.runner.state, ACTIVE)
        self.assertEqual(rig.loop.status()['native_navigation']['native_cross_check']['reason'],
                         'GENERATED_PROVIDER_FORBIDDEN')

    def test_motor_single_row_through_the_real_bridge(self):
        rig = Rig(cfg=sr_motor_config(), motor=True)
        self.begin(rig)
        out = rig.run(8., hold=True)
        sent = [call for call in rig.motor.calls if isinstance(call, tuple)]
        self.assertTrue(any(left > 0 for left, right in sent))
        self.assertTrue(all(left >= 0 and right >= 0 for left, right in sent))      # forward only, never a pivot
        self.assertTrue(all(left == 0 or min(left, right) >= 30 for left, right in sent))   # inner wheel rule
        self.assertEqual(rig.loop.runner.end_reason, 'drive_no_response')
        self.assertEqual(out[-1].summary['single_row']['stop_category'], 'CONTROL_AUTHORITY')
        self.assertNotEqual(rig.loop.bridge.state, ARMED)          # ended normally: op stop after stillness
        result = rig.loop.emergency_stop('test')                    # STOP still works and latches
        rig.run(.1)
        self.assertTrue(result['stop_sent'])

    def test_stop_latches_in_single_row(self):
        rig = Rig(cfg=sr_motor_config(), motor=True)
        self.begin(rig)
        rig.run(1.5, hold=True)
        result = rig.loop.emergency_stop('test')
        rig.run(.1)
        self.assertTrue(result['stop_sent'])
        self.assertEqual(rig.loop.bridge.state, LATCHED)
        self.assertEqual(rig.loop.runner.end_reason, 'operator_stop')
        self.assertEqual(rig.loop.status()['single_row']['stop_category'], 'OPERATOR_OR_BACKSTOP')


if __name__ == '__main__':
    unittest.main()
