"""Runner with the REAL vendored estimator/controller on synthetic static orchard scans.

The synthetic vehicle never moves (as on a bench with the wheels off the ground), so the real
controller ends every uninterrupted drive with drive_no_response; that is used on purpose.
"""
import json
import math
import os
import shutil
import tempfile
import types
import unittest

import numpy as np

import rowexec_testlib as L
from row_executor import config as C
from row_executor import core
from row_executor.config import ExecutorConfig
from row_executor.core import ACTIVE, ENDED, IDLE, PAUSED, Runner, ScanAdapter

SCENE = L.raw_scan(L.orchard_trunks())


def make(cfg=None, scene=SCENE, t0=1000., **kw):
    runner = Runner(cfg or ExecutorConfig(), **kw)
    driver = L.FakeTimeDriver(runner, lambda t: scene, t0=t0)
    runner.start_run(driver.t, 30.)
    return runner, driver


def states(decisions):
    return [(round(t, 2), d.state, d.reason, d.left, d.right) for t, d in decisions]


class FakeEstimator:
    """Stands in for SensorEstimator where only ages/distance/stillness matter."""

    def __init__(self, speed=0., still=False):
        self.speed, self.still = speed, still
        self.lidar_age, self.imu_age = .05, .02
        self.t0 = None

    def on_imu(self, stamp, gyro, accel):
        pass

    def on_scan(self, stamp, ranges, angle_min, increment, range_max):
        pass

    def snapshot(self, now):
        if self.t0 is None:
            self.t0 = now
        return dict(lidar_age_s=self.lidar_age, imu_age_s=self.imu_age, still=self.still, still_s=0.,
                    speed=self.speed, distance_m=self.speed * (now - self.t0), yaw_rate=0.,
                    corridor_valid=True, corridor_reason='fake')


class StubController:
    def __init__(self, left=55, right=55, state='FOLLOW'):
        self.out = (left, right, state)
        self.state, self.reason = 'REACQUIRE', 'start'
        self.times = []

    def step(self, now, snapshot):
        self.times.append(now)
        left, right, state = self.out
        self.state, self.reason = state, 'stub'
        return types.SimpleNamespace(left=left, right=right, state=state, reason='stub')


class VendorTest(unittest.TestCase):
    def test_vendor_is_hash_verified_and_imported_from_vendor_dir(self):
        estimator, controllers = core.load_vendor()
        self.assertTrue(core._inside(estimator.__file__, core.VENDOR_DIR))
        self.assertTrue(core._inside(controllers.__file__, core.VENDOR_DIR))
        manifest = core.verify_vendor()
        # single-row integration (2026-11-02): + closed_loop/single_row.py, vendored from the single-row branch
        # (854a6d7); single-row DEV round 1 fixes (a7e9785): opt-in estimator options + stop-table entries; DEV round 2
        # fixes (c35126025d9ea1d4c939312b64e09c57e90a70b6): band about the vehicle, staggered row end, last trunk, per-side END_STRUCTURE.
        # Steering branch (2026-11-09, docs/steering/EXECUTOR_IMPORT_KO.md): tools/steer_sim/regen_vendor.py rewrites the
        # vendor from this repository's SIM tree (--worktree: source_commit WORKTREE-UNCOMMITTED; --commit: that commit),
        # so the source commit is no longer the single-row one; every vendored file must equal its SIM-tree source.
        commit = manifest['source_commit']
        self.assertTrue(commit == 'WORKTREE-UNCOMMITTED'
                        or (len(commit) == 40 and all(c in '0123456789abcdef' for c in commit)), commit)
        self.assertFalse(manifest['modified'])
        files = {e['vendored_path']: e['source_path'] for e in manifest['files']}
        # review P4R-02: the vendored packages are closed_loop_copy / orchard_perception_copy (no SIM-tree names)
        self.assertLessEqual({'closed_loop_copy/__init__.py', 'closed_loop_copy/estimator.py',
                              'closed_loop_copy/controllers.py', 'closed_loop_copy/single_row.py',
                              'orchard_perception_copy/__init__.py', 'orchard_perception_copy/scan_corridor.py'},
                             set(files))
        self.assertEqual(estimator.__name__.split('.')[-3:], ['vendor', 'closed_loop_copy', 'estimator'])
        repo = os.path.dirname(L.TOOLS)
        for vendored, source in files.items():
            with open(os.path.join(core.VENDOR_DIR, *vendored.split('/')), 'rb') as fh:
                copy = fh.read()
            with open(os.path.join(repo, *source.split('/')), 'rb') as fh:
                self.assertEqual(copy, fh.read(), vendored)

    def test_tampered_vendor_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            copy = os.path.join(tmp, 'vendor')
            shutil.copytree(core.VENDOR_DIR, copy, ignore=shutil.ignore_patterns('__pycache__'))
            core.verify_vendor(copy)
            path = os.path.join(copy, 'closed_loop_copy', 'controllers.py')
            with open(path, 'ab') as fh:
                fh.write(b'\n# edited\n')
            with self.assertRaisesRegex(core.VendorError, 'modified: closed_loop_copy/controllers.py'):
                core.verify_vendor(copy)

    def test_vendor_and_sim_tree_live_in_one_process(self):
        # review P4R-02: load_vendor puts nothing on sys.path and registers no SIM-tree name, so the SIM tree's
        # closed_loop / orchard_perception import next to the vendored copies (before, in one pytest process the two
        # copies clashed and load_vendor refused); the vendored estimator binds the vendored scan corridor detector
        import sys
        names_before = {n for n in sys.modules if n.split('.')[0] in ('closed_loop', 'orchard_perception')}
        estimator, controllers = core.load_vendor()
        self.assertNotIn(core.VENDOR_DIR, sys.path)
        names_after = {n for n in sys.modules if n.split('.')[0] in ('closed_loop', 'orchard_perception')}
        self.assertEqual(names_after, names_before)
        corridor = sys.modules[estimator.observe_scan_corridor.__module__]
        self.assertTrue(core._inside(corridor.__file__, core.VENDOR_DIR))
        self.assertEqual(core.load_vendor()[0], estimator)                    # cached, same module
        self.assertEqual(core.single_row_module(controllers).__name__.rsplit('.', 1)[0],
                         controllers.__name__.rsplit('.', 1)[0])

    def test_runner_uses_forward_only_controller(self):
        runner = Runner(ExecutorConfig())
        cfg = runner.controller_config
        self.assertEqual(cfg.mode, 'continuous')
        self.assertTrue(cfg.side_clearance_check)
        self.assertFalse(cfg.pivot_failure_fallback)
        # imu_max_age_s 0.4: C1 gate P2 (one value for the Runner rule and the controller)
        self.assertEqual((cfg.base_duty_pct, cfg.max_differential_pct, cfg.lidar_max_age_s, cfg.imu_max_age_s),
                         (55, 20, .45, .4))
        self.assertEqual(runner.estimator_config.corridor_every_n_scans, 3)     # C1 gate P5
        self.assertFalse(Runner(ExecutorConfig(mode='continuous')).controller_config.side_clearance_check)

    def test_trimmed_scan_is_accepted_by_vendored_scan_corridor(self):
        observe_scan_corridor = core.load_vendor()[0].observe_scan_corridor     # the vendored detector
        padded = np.concatenate([SCENE, SCENE[:1]])          # 3241 beams: refused as is
        raw = types.SimpleNamespace(ranges=np.where(padded > 0, padded, np.inf), angle_min=L.RAW_AMIN,
                                    angle_increment=L.RAW_INC, range_min=.05, range_max=30.)
        self.assertEqual(observe_scan_corridor(raw).reason, 'invalid_scan_geometry')
        r, amin, inc, rmax, _ = ScanAdapter().convert(padded, L.RAW_AMIN, L.RAW_INC, 30., .05)
        fixed = types.SimpleNamespace(ranges=r, angle_min=amin, angle_increment=inc, range_min=.05, range_max=rmax)
        self.assertTrue(observe_scan_corridor(fixed).valid)


class HoldRuleTest(unittest.TestCase):
    def test_no_output_before_a_press_and_follow_after(self):
        runner, d = make()
        out = d.run(2.5, hold=True)              # held from before Start: never counts as a press
        self.assertTrue(all(dec.left == dec.right == 0 and dec.state == PAUSED for _, dec in out))
        out = d.run(.1, hold=False) + d.run(.5, hold=True)
        t, dec = L.first(out, lambda x: x.state == ACTIVE and x.left)
        self.assertIsNotNone(t)
        self.assertEqual((dec.left, dec.right, dec.summary['controller_state']), (55, 55, 'FOLLOW'))
        json.dumps(dec.summary, allow_nan=False)

    def test_press_before_inputs_fresh_for_1s_is_ignored(self):
        runner, d = make()
        out = d.run(.2, hold=False) + d.run(3., hold=True)
        self.assertTrue(all(dec.left == dec.right == 0 for _, dec in out))
        self.assertEqual(runner.state, PAUSED)
        self.assertEqual(runner.reason, 'press_ignored_inputs_not_fresh')
        out = d.run(.1, hold=False) + d.run(.3, hold=True)
        self.assertEqual(runner.state, ACTIVE)

    def test_release_pauses_and_resume_builds_new_controller(self):
        runner, d = make()
        d.run(1.5, hold=False)
        d.run(.5, hold=True)
        first_controller = runner.controller
        self.assertIsNotNone(first_controller)
        out = d.run(.3, hold=False)
        self.assertEqual((out[0][1].state, out[0][1].reason, out[0][1].left), (PAUSED, 'hold_released', 0))
        self.assertIsNone(runner.controller)
        d.run(.5, hold=True)
        self.assertEqual(runner.state, ACTIVE)
        self.assertIsNot(runner.controller, first_controller)
        self.assertEqual(runner.controllers_built, 2)

    def test_pause_prevents_false_drive_no_response_which_fires_after_about_3s(self):
        runner, d = make()
        d.run(1.5, hold=False)
        out = d.run(2.5, hold=True)
        t_follow, _ = L.first(out, lambda x: x.left > 0)
        self.assertTrue(all(dec.state == ACTIVE for t, dec in out if t >= t_follow))
        d.run(.5, hold=False)
        out = d.run(5., hold=True)
        t_follow, _ = L.first(out, lambda x: x.left > 0)
        t_zero, dec_zero = L.first(out, lambda x: x.left == 0 and x.state == ACTIVE and x.reason == 'drive_no_response')
        t_end, dec_end = L.first(out, lambda x: x.state == ENDED)
        self.assertIsNotNone(t_follow)
        # MCU ramp 1 s + grace 1 s + no-response 1 s (+ one scan) -> ~3 s, then the stop is confirmed.
        self.assertTrue(2.95 <= t_zero - t_follow <= 3.3, t_zero - t_follow)
        self.assertEqual(dec_end.reason, 'drive_no_response')
        self.assertTrue(t_end - t_follow <= 4.2, t_end - t_follow)
        self.assertTrue(all(dec.left == dec.right == 0 for t, dec in out if t >= t_zero))

    def test_non_bool_hold_is_not_a_press(self):
        runner, d = make()
        d.run(1.5, hold=False)
        d.run(.5, hold=lambda t: 1)
        self.assertEqual(runner.state, PAUSED)


class StaleRuleTest(unittest.TestCase):
    def follow(self, cfg=None):
        runner, d = make(cfg)
        d.run(1.5, hold=False)
        d.run(.6, hold=True)
        self.assertEqual(runner.state, ACTIVE)
        return runner, d

    def test_stale_lidar_pauses_and_never_restarts_while_held(self):
        runner, d = self.follow()
        d.scan_on = False
        t_stop = d.t
        out = d.run(.8, hold=True)
        t_pause, dec = L.first(out, lambda x: x.state == PAUSED)
        self.assertEqual((dec.reason, dec.left, dec.right), ('stale_lidar', 0, 0))
        self.assertLess(t_pause - t_stop, .5)
        d.scan_on = True
        out = d.run(3., hold=True)                       # fresh again, operator still holding
        self.assertTrue(all(x.state == PAUSED and x.left == x.right == 0 for _, x in out))
        out = d.run(.1, hold=False) + d.run(.8, hold=True)   # NEW press after >= 1 s fresh
        self.assertEqual(runner.state, ACTIVE)
        self.assertIsNotNone(L.first(out, lambda x: x.left > 0)[0])
        self.assertEqual(runner.stale_events, 1)

    def test_press_within_fresh_recover_window_is_ignored(self):
        runner, d = self.follow()
        d.scan_on = False
        d.run(.6, hold=False)                            # released during the outage
        self.assertEqual(runner.state, PAUSED)
        d.scan_on = True
        d.run(.3, hold=False)
        out = d.run(2.5, hold=True)                      # press 0.3 s after recovery: ignored, keep holding
        self.assertTrue(all(x.state == PAUSED and x.left == 0 for _, x in out))
        self.assertEqual(runner.reason, 'press_ignored_inputs_not_fresh')
        d.run(.1, hold=False)
        d.run(.3, hold=True)
        self.assertEqual(runner.state, ACTIVE)

    def test_stale_imu(self):
        runner, d = self.follow()
        d.imu_on = False
        out = d.run(.6, hold=True)
        t_pause, dec = L.first(out, lambda x: x.state == PAUSED)
        self.assertEqual(dec.reason, 'stale_imu')

    def test_stale_repeat_limit_ends_run(self):
        runner, d = self.follow(ExecutorConfig(max_stale_events=1))
        for k in range(2):
            d.scan_on = False
            d.run(.8, hold=True)
            d.scan_on = True
            if k == 0:
                d.run(1.5, hold=False)
                d.run(.5, hold=True)
                self.assertEqual(runner.state, ACTIVE)
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'stale_repeat_limit'))

    def test_recoverable_controller_hold_uses_the_same_rule(self):
        stub = StubController()
        runner = Runner(ExecutorConfig(), estimator_factory=FakeEstimator, controller_factory=lambda: stub)
        d = L.FakeTimeDriver(runner, lambda t: SCENE)
        runner.start_run(d.t, 30.)
        d.run(1.2, hold=False)
        stub.step = lambda now, s: types.SimpleNamespace(left=0, right=0, state='HOLD', reason='stale_lidar')
        d.run(.2, hold=True)
        self.assertEqual((runner.state, runner.reason, runner.stale_events), (PAUSED, 'stale_lidar', 1))


class TerminalTest(unittest.TestCase):
    def test_front_blocked_ends_run_and_a_press_does_not_restart(self):
        scene = L.raw_scan(L.orchard_trunks(extra=[(.7, 0., .12)]))
        runner, d = make(scene=scene)
        d.run(1.5, hold=False)
        out = d.run(1., hold=True)
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'front_blocked'))
        self.assertTrue(all(x.left == x.right == 0 for _, x in out))
        out = d.run(.2, hold=False) + d.run(1., hold=True)
        self.assertTrue(all(x.state == ENDED and x.terminal and x.left == 0 for _, x in out))
        old = runner.run_id
        runner.start_run(d.t, 20.)                       # a new explicit Start is the only way on
        self.assertEqual((runner.state, runner.run_id), (PAUSED, old + 1))

    def test_side_self_return_needs_the_body_mask(self):
        bracket = ((math.radians(100), .36, 3), (math.radians(-100), .36, 3))
        scene = L.raw_scan(L.orchard_trunks(), self_returns=bracket)
        runner, d = make(scene=scene)
        d.run(1.5, hold=False)
        d.run(1.5, hold=True)
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'side_blocked'))
        # 0.40 m: at +-100 deg the footprint side edge is 0.355 m from the LiDAR (+ 0.05 m, review S5)
        segments = tuple((L.body_bearing_to_raw_index(b) - h - 2, L.body_bearing_to_raw_index(b) + h + 2, .40)
                         for b, _, h in bracket)
        runner, d = make(ExecutorConfig(body_mask=segments), scene=scene)
        d.run(1.5, hold=False)
        out = d.run(1.5, hold=True)
        self.assertEqual(runner.state, ACTIVE)
        self.assertIsNotNone(L.first(out, lambda x: x.left == 55 and x.right == 55)[0])

    def test_guard_violations_end_the_run(self):
        cases = (((-55, 55, 'FOLLOW'), 'guard_violation:pivot_command'),
                 ((-40, -40, 'FOLLOW'), 'guard_violation:reverse_command'),
                 ((80, 80, 'FOLLOW'), 'guard_violation:duty_above_cap'),
                 ((20, 61, 'FOLLOW'), 'guard_violation:differential_above_cap'),
                 ((55., 55, 'FOLLOW'), 'guard_violation:non_integer_duty'),
                 ((40, 40, 'REACQUIRE'), 'guard_violation:output_in_non_driving_state'))
        for (left, right, state), why in cases:
            with self.subTest(why=why):
                runner = Runner(ExecutorConfig(), estimator_factory=FakeEstimator,
                                controller_factory=lambda: StubController(left, right, state))
                d = L.FakeTimeDriver(runner, lambda t: SCENE)
                runner.start_run(d.t, 30.)
                d.run(1.2, hold=False)
                out = d.run(.3, hold=True)
                self.assertEqual((runner.state, runner.end_reason), (ENDED, why))
                self.assertTrue(all(x.left == x.right == 0 for _, x in out))

    def test_inner_wheel_rule_in_guard(self):
        cfg = ExecutorConfig(breakaway_duty_pct=25, inner_margin_pct=10)      # inner >= 35
        runner = Runner(cfg, estimator_factory=FakeEstimator, controller_factory=lambda: StubController(34, 54))
        d = L.FakeTimeDriver(runner, lambda t: SCENE)
        runner.start_run(d.t, 30.)
        d.run(1.2, hold=False)
        d.run(.2, hold=True)
        self.assertEqual(runner.end_reason, 'guard_violation:inner_below_breakaway')

    def test_controller_runs_on_a_clock_starting_at_zero(self):
        stub = StubController()
        runner = Runner(ExecutorConfig(), estimator_factory=FakeEstimator, controller_factory=lambda: stub)
        d = L.FakeTimeDriver(runner, lambda t: SCENE, t0=98765.)
        runner.start_run(d.t, 30.)
        d.run(1.2, hold=False)
        d.run(.5, hold=True)
        self.assertAlmostEqual(stub.times[0], 0.)
        self.assertAlmostEqual(stub.times[-1], .45, places=6)

    def test_real_controller_follows_at_a_large_monotonic_time(self):
        runner, d = make(t0=987654.)
        d.run(1.5, hold=False)
        out = d.run(.5, hold=True)
        self.assertIsNotNone(L.first(out, lambda x: x.left == 55)[0])
        self.assertEqual(runner.state, ACTIVE)

    def test_clock_regression_ends_run(self):
        runner = Runner(ExecutorConfig(), estimator_factory=FakeEstimator, controller_factory=StubController)
        runner.start_run(10., 30.)
        runner.tick(11., False)
        self.assertEqual(runner.tick(10.5, False).reason, 'clock_regression')
        self.assertEqual(runner.state, ENDED)


class BackstopRunnerTest(unittest.TestCase):
    def test_distance_backstop(self):
        runner = Runner(ExecutorConfig(), estimator_factory=lambda: FakeEstimator(speed=1.),
                        controller_factory=StubController)
        d = L.FakeTimeDriver(runner, lambda t: SCENE)
        runner.start_run(d.t, 10.)                       # stop at 8 m
        out = d.run(1.2, hold=False) + d.run(10., hold=True)
        t_end, dec = L.first(out, lambda x: x.state == ENDED)
        self.assertEqual(dec.reason, 'row_distance_backstop')
        self.assertTrue(8. <= dec.summary['backstop']['travelled_m'] < 8.06)
        self.assertTrue(all(x.left == 0 for t, x in out if t >= t_end))

    def test_time_backstop_when_odometry_reads_nothing(self):
        cfg = ExecutorConfig(v_measured_m_s=.5)
        runner = Runner(cfg, estimator_factory=lambda: FakeEstimator(speed=0., still=False),
                        controller_factory=StubController)
        d = L.FakeTimeDriver(runner, lambda t: SCENE)
        runner.start_run(d.t, 10.)                       # 1.15 * 8 / 0.5 = 18.4 s of motion time
        out = d.run(20., hold=True)
        t_end, dec = L.first(out, lambda x: x.state == ENDED)
        self.assertEqual(dec.reason, 'row_time_backstop')
        self.assertAlmostEqual(t_end - 1000., 18.4, delta=.06)

    def test_paused_and_still_does_not_count_time(self):
        cfg = ExecutorConfig(v_measured_m_s=.5)
        est = FakeEstimator(speed=0., still=True)
        runner = Runner(cfg, estimator_factory=lambda: est, controller_factory=StubController)
        d = L.FakeTimeDriver(runner, lambda t: SCENE)
        runner.start_run(d.t, 10.)
        d.run(60., hold=False)                           # waiting, standing still
        self.assertEqual(runner.state, PAUSED)
        self.assertLess(runner.backstop.motion_time_s, .2)

    def test_invalid_row_length_does_not_start(self):
        runner = Runner(ExecutorConfig(), estimator_factory=FakeEstimator, controller_factory=StubController)
        for bad in (2.5, float('nan'), -1., 1e4):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                runner.start_run(0., bad)
        self.assertEqual((runner.state, runner.run_id), (IDLE, 0))
        runner.start_run(0., 20.)
        with self.assertRaisesRegex(RuntimeError, 'run_in_progress'):
            runner.start_run(1., 20.)


class StallGuardTest(unittest.TestCase):
    def test_tick_gap_pauses_and_repeats_end_the_run(self):
        runner = Runner(ExecutorConfig(), estimator_factory=FakeEstimator, controller_factory=StubController)
        d = L.FakeTimeDriver(runner, lambda t: SCENE)
        runner.start_run(d.t, 30.)
        d.run(1.2, hold=False)
        d.run(.2, hold=True)
        self.assertEqual(runner.state, ACTIVE)
        decision = runner.tick(d.t + .4, True)           # loop stalled 0.4 s > 0.35 s
        self.assertEqual((decision.state, decision.reason, decision.left), (PAUSED, 'stall_guard', 0))
        d.t += .4
        d.next_imu = d.next_scan = d.t
        for k in range(3):
            d.run(.1, hold=False)
            d.run(.1, hold=True)
            self.assertEqual(runner.state, ACTIVE)
            runner.pause(d.t, 'stall_guard')             # the watchdog thread's pause
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'stall_guard_repeat_limit'))


class BenchTest(unittest.TestCase):
    def bench_cfg(self):
        return ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK,
                              bench_pattern=((1., 40, 40), (.5, 30, 50)))

    def test_field_profile_cannot_run_a_bench_pattern(self):
        runner = Runner(ExecutorConfig(), estimator_factory=FakeEstimator)
        with self.assertRaisesRegex(RuntimeError, 'bench_pattern_requires_bench_profile'):
            runner.start_bench(0.)

    def test_pattern_through_hold_freshness_and_guard(self):
        est = FakeEstimator()
        runner = Runner(self.bench_cfg(), estimator_factory=lambda: est)
        d = L.FakeTimeDriver(runner, lambda t: SCENE)
        runner.start_bench(d.t)
        d.run(1.2, hold=False)
        out = d.run(.6, hold=True)
        self.assertEqual({(x.left, x.right) for _, x in out}, {(40, 40)})
        driven = sum(1 for _, x in out if x.left)
        out = d.run(.5, hold=False)                       # release: pause, pattern position kept
        self.assertTrue(all(x.left == 0 and x.state == PAUSED for _, x in out))
        est.imu_age = 1.                                  # stale while held: pause, no auto restart
        d.run(.1, hold=True)
        out = d.run(.3, hold=True)
        self.assertTrue(all(x.left == 0 for _, x in out))
        est.imu_age = .02
        out = d.run(1.5, hold=True)
        self.assertTrue(all(x.left == 0 for _, x in out))
        out = d.run(.1, hold=False) + d.run(2., hold=True)
        seen = [(x.left, x.right) for _, x in out if x.left]
        self.assertEqual(seen[0], (40, 40))                # continues where it paused
        self.assertEqual(seen.count((30, 50)), 10)          # 0.5 s step at 20 Hz
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'bench_pattern_complete'))
        # 1.5 s of pattern time at 20 Hz; each resume tick repeats the paused position once.
        self.assertIn(driven + len(seen), (30, 31, 32))


if __name__ == '__main__':
    unittest.main()
