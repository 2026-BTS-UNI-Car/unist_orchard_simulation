"""Regression tests for the day-2 review fixes (field autonomy work folder of 2026-10-22, review_safety_day2 N1-N9,
review_correctness_day2 R1-R6).

  N1  motor output (row and calibration) refuses every loosened safety-relevant setting the day-1 S6 check missed
  N2  calibration configs: base <= 60, duty cap <= 75, max speed <= 0.5
  N3  bench interlock ends and calibration yaw / overspeed / heading ends stop the motors at once (op stop, LATCHED)
  N4  an arc starts at (base, base) and switches to its pair only once the vehicle rolls
  N5  body_mask_static: forward-sector segments need --confirm-forward-sector (test_body_mask_static + here)
  N6  bench: lost scan matching while ACTIVE ends the run
  N7  calib_analysis: one field day, no silent fallback to an older run (make_field_config refuses it, exit 4)
  N8  C7: calibration kind 'stop' (executor 0/0 stop distance), row motor output needs measured_stop_distance_m
  N9  repeated short presses can not push a stuck vehicle without end (cumulative no-response budget)
  R1  rolldown floor independent of the static breakaway (C3b below C2 measurable)
  R2  on-site differential scales the measured yaw authority and max_yaw_rate_deg_s
  R5  the calibration tick tags let calib_analysis recompute exactly the runtime's measurement window
  R6  JSONL: end_note on ENDED ticks, one run_end event per run
On the code before the fixes (6693938) 24 of these tests fail and 35 subtests fail; the passing ones there are
controls of unchanged behaviour (SHADOW keeps every setting, filter off allowed, a normal calibration end still
fades, a PAUSED bench with invalid matching does not end, on-site max_yaw never above the file value).
"""
import dataclasses
import json
import math
import os
import tempfile
import unittest

import rowexec_testlib as L
from row_executor import body_mask_static as B
from row_executor import calib_analysis as CA
from row_executor import calib_runs as CR
from row_executor import config as C
from row_executor import core
from row_executor import executor_main as M
from row_executor.config import ConfigError, ExecutorConfig
from row_executor.core import ACTIVE, ENDED, PAUSED, Runner

import test_calib_runs as TC
import test_review_fixes_core as TF
from test_executor_loop import Rig as LoopRig
from test_executor_loop import motor_config

DT = .05


def row_motor(**over):
    """A row motor config that passes every rule (example numbers, not field values)."""
    base = dict(motor_output_enabled=True, base_duty_pct=55, reduce_duty_pct=55, max_differential_pct=20,
                breakaway_duty_pct=25, v_measured_m_s=.45, measured_yaw_at_max_diff_deg_s=6., max_yaw_rate_deg_s=3.6,
                measured_stop_distance_m=.25)
    base.update(over)
    return ExecutorConfig(**base)


def loop_rig(cfg, plant=None, est=None, motor=True):
    """ExecutorLoop + real RoverControl core with a fake estimator (calibration Plant or a fixed estimator)."""
    box = {}

    def runner_factory(c):
        runner = Runner(c, estimator_factory=(lambda: est) if est is not None else
                        (lambda: TC.Plant(box, **(plant or {}))))
        box['runner'] = runner
        return runner
    return LoopRig(cfg, motor=motor, runner_factory=runner_factory)


def calib_through_loop(test, cfg, kind, params=None, plant=None, max_s=60., press='cal-1'):
    rig = loop_rig(cfg, plant)
    loop = rig.loop
    fut = loop.submit('calib_start', calib_kind=kind, params=params or {})
    rig.run(1.5)
    test.assertTrue(fut.result()['ok'], fut.result())
    fut = loop.submit('arm')
    rig.run(.05)
    test.assertTrue(fut.result()['ok'], fut.result())
    rig.press(press)
    n_end = None
    for _ in range(int(max_s / .05)):
        n = len(rig.transport.requests)
        rig.run(.05, hold=True)
        if loop.runner.state == ENDED:
            n_end = n
            break
    test.assertIsNotNone(n_end, 'calibration run did not end')
    return rig, n_end


# --- N1 -----------------------------------------------------------------------------------------------------
class MotorHardeningTest(unittest.TestCase):
    LOOSENED = dict(near_confirm_scans=4, near_confirm_range_m=5., near_confirm_tol_m=.05, near_confirm_tol_step_m=0.,
                    range_min_floor_m=.5, backstop_margin_m=0., backstop_time_factor=3., max_speed_m_s=1.,
                    stall_guard_s=2., bridge_timeout_s=5., max_stale_events=20, max_stall_guard_pauses=20,
                    hold_timeout_s=.7, fresh_recover_s=.5, inner_margin_pct=0, yaw_authority_fraction=1.,
                    min_yaw_authority_deg_s=0., footprint_length_m=.3, footprint_width_m=.3)
    OVERRIDES = dict(corridor_lost_s=5., drive_grace_s=3., drive_no_response_s=3., drive_min_speed_m_s=.01,
                     still_confirm_s=.2, max_stops_per_window=10, reacquire_updates=1)

    def test_baseline_ok_and_every_loosened_key_refused(self):
        self.assertEqual(row_motor().validate().feasibility()['status'], 'ok')
        for key, value in self.LOOSENED.items():
            over = {key: value}
            if key == 'max_speed_m_s':
                over['v_measured_m_s'] = .9                       # the review probe: max 1.0 with v 0.9
            if key == 'inner_margin_pct':
                over['breakaway_duty_pct'] = 35                   # margin 0 would otherwise pass the static rule
            if key in ('footprint_length_m', 'footprint_width_m'):
                over['lidar_x_in_base_m'] = 0.
            with self.subTest(key=key), self.assertRaisesRegex(ConfigError, key):
                row_motor(**over).validate()

    def test_loosened_controller_overrides_refused_tightened_allowed(self):
        for key, value in self.OVERRIDES.items():
            with self.subTest(key=key), self.assertRaisesRegex(ConfigError, 'controller_overrides.' + key):
                row_motor(controller_overrides={key: value}).validate()
        tight = dict(corridor_lost_s=1.5, drive_grace_s=.8, drive_no_response_s=.8, drive_min_speed_m_s=.05,
                     still_confirm_s=.6, max_stops_per_window=2, reacquire_updates=3)
        row_motor(controller_overrides=tight).validate()

    def test_limits_equal_the_vendored_defaults(self):
        estimator, controllers = core.load_vendor()
        cc = controllers.ControlConfig()
        for name, _, bound in C.DAY2_CONTROLLER_LIMITS:
            self.assertEqual(getattr(cc, name), bound, name)
        d = ExecutorConfig()
        for name, _, bound in C.DAY2_MOTOR_LIMITS:
            if name in ('max_speed_m_s', 'footprint_length_m', 'footprint_width_m'):
                continue                                          # bounds around, not equal to, the default
            self.assertEqual(getattr(d, name), bound, name)

    def test_mount_sign_lidar_position_and_thresholds(self):
        for over, text in ((dict(lidar_yaw_offset_deg=-.92), 'lidar_yaw_offset_deg'),
                           (dict(lidar_yaw_offset_deg=165.), 'lidar_yaw_offset_deg'),
                           (dict(imu_yaw_sign=-1), 'imu_yaw_sign'),
                           (dict(lidar_x_in_base_m=1.), 'outside the footprint'),
                           (dict(breakaway_duty_pct=4), 'breakaway_duty_pct 4 below 20'),
                           (dict(breakaway_duty_pct=34, rolling_breakaway_duty_pct=10, max_differential_pct=10,
                                 measured_yaw_at_max_diff_deg_s=3., max_yaw_rate_deg_s=1.8),
                            'rolling_breakaway_duty_pct 10 below 20')):
            with self.subTest(over=over), self.assertRaisesRegex(ConfigError, text):
                row_motor(**over).validate()
        for offset in (172., 179.08, 180., -179.5, -171.):           # within 10 deg of 179.08 (wrapped)
            row_motor(lidar_yaw_offset_deg=offset).validate()

    def test_near_confirm_off_is_allowed(self):
        # filter off: every near return reaches the estimator (more conservative); its tolerances do not matter
        row_motor(near_confirm_scans=0, near_confirm_tol_m=.05, near_confirm_tol_step_m=0.).validate()

    def test_shadow_keeps_every_setting(self):
        ExecutorConfig(**self.LOOSENED, controller_overrides=self.OVERRIDES, lidar_x_in_base_m=0.).validate()

    def test_calibration_config_uses_the_same_rules(self):
        with self.assertRaisesRegex(ConfigError, 'calib_motor_output_enabled.*near_confirm_tol_m'):
            TC.calib_cfg(near_confirm_tol_m=.05, near_confirm_tol_step_m=0.)
        with self.assertRaisesRegex(ConfigError, 'controller_overrides.drive_no_response_s'):
            TC.calib_cfg(controller_overrides=dict(drive_no_response_s=3.))


# --- N2 -----------------------------------------------------------------------------------------------------
class CalibrationBoundsTest(unittest.TestCase):
    def test_base_cap_and_speed_bounded(self):
        TC.calib_cfg(base_duty_pct=60, reduce_duty_pct=60, max_differential_pct=10, duty_cap_pct=75)
        for over, text in ((dict(base_duty_pct=90, reduce_duty_pct=90, max_differential_pct=10, duty_cap_pct=100),
                            'base_duty_pct 90 above 60'),
                           (dict(duty_cap_pct=80), 'duty_cap_pct 80 above 75'),
                           (dict(max_speed_m_s=.55), 'max_speed_m_s 0.55 above 0.5')):
            with self.subTest(over=over), self.assertRaisesRegex(ConfigError, text):
                TC.calib_cfg(**over)
        # the overspeed end of a calibration run is therefore at most 0.5 + 0.15 m/s
        self.assertLessEqual(CR.make_plan(TC.cfg34(), 'arc').overspeed_m_s, .65 + 1e-9)


# --- N3 -----------------------------------------------------------------------------------------------------
class EmergencyEndsTest(unittest.TestCase):
    def test_calibration_yaw_end_stops_at_once(self):
        rig, n = calib_through_loop(self, TC.cfg34(), 'arc', dict(step_s=3.), plant=dict(gain=.7))
        self.assertEqual(rig.loop.runner.end_reason, 'calib_yaw_rate')
        ops = [r['op'] for r in rig.transport.requests[n:]]
        self.assertEqual(ops[0], 'stop')                         # no 0/0 fade first
        self.assertEqual(rig.loop.bridge.state, 'LATCHED')

    def test_normal_calibration_end_still_fades(self):
        rig, n = calib_through_loop(self, TC.cfg34(), 'arc', dict(step_s=3.))
        self.assertEqual(rig.loop.runner.end_reason, 'calib_complete')
        first = rig.transport.requests[n]
        self.assertEqual((first['op'], first['left'], first['right']), ('drive', 0, 0))

    def test_bench_motion_stops_at_once(self):
        est = TF.BenchEstimator()
        cfg = ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK, motor_output_enabled=True,
                             bench_pattern=((10., 40, 40),)).validate()
        rig = loop_rig(cfg, est=est)
        loop = rig.loop
        fut = loop.submit('start')
        rig.run(1.5)
        self.assertTrue(fut.result()['ok'])
        fut = loop.submit('arm')
        rig.run(.05)
        self.assertTrue(fut.result()['ok'])
        rig.press('bench')
        rig.run(.5, hold=True)
        self.assertIn((40, 40), [(r.get('left'), r.get('right')) for r in rig.transport.requests])
        est.s.update(still=False, speed=.15)                    # the wheels touch the ground
        n = len(rig.transport.requests)
        rig.run(.05, hold=True)
        self.assertEqual(loop.runner.end_reason, 'bench_motion_detected')
        self.assertEqual([r['op'] for r in rig.transport.requests[n:]], ['stop'])
        self.assertEqual(loop.bridge.state, 'LATCHED')

    def test_prefixes(self):
        for reason in ('bench_motion_detected', 'bench_front_blocked', 'bench_odometry_invalid', 'bench_odometry_lost',
                       'calib_yaw_rate', 'calib_overspeed', 'calib_heading_change'):
            self.assertTrue(reason.startswith(M.EMERGENCY_PREFIXES), reason)
        for reason in ('bench_pattern_complete', 'calib_complete', 'calib_front_blocked', 'calib_hold_released',
                       'calib_stop_test', 'row_end_candidate'):
            self.assertFalse(reason.startswith(M.EMERGENCY_PREFIXES), reason)


# --- N4 -----------------------------------------------------------------------------------------------------
class ArcStartTest(unittest.TestCase):
    def test_pair_only_after_the_vehicle_rolls(self):
        # static 44 / rolling 30 measured: the d20 pair (55, 35) could not start from standstill (35 < 44), but
        # keeps rolling (35 >= 30). The arc starts at (55, 55) and measures the pair while rolling.
        cfg = TC.calib_cfg(breakaway_duty_pct=44, rolling_breakaway_duty_pct=30, max_differential_pct=1)
        plan = CR.make_plan(cfg, 'arc', dict(arc_set='d20'))
        self.assertEqual((plan.floor_pct, plan.floor_source), (30, 'rolling_breakaway_duty_pct'))
        self.assertEqual((plan.start_floor_pct, plan.start_floor_source), (44, 'breakaway_duty_pct'))
        rig = TC.Rig(cfg, plan=plan, static=44, rolling=30)
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_complete')
        driving = [(d.left, d.right) for d in rig.log if d.left or d.right]
        self.assertEqual(driving[0], (55, 55))
        first_pair = next(i for i, p in enumerate(driving) if p != (55, 55))
        self.assertTrue(all(p == (55, 55) for p in driving[:first_pair]))
        self.assertTrue(all(r['valid'] for r in rig.runner.calib.results))
        self.assertAlmostEqual(rig.runner.calib.outcome['sides']['right']['gain'], .15, delta=.01)

    def test_moved_then_stalled_before_the_pair(self):
        rig = TC.Rig(stall_after_s=.05, v_per_pct=.0015)          # rolls slower than 0.10 m/s and then stops
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_start_stalled')
        self.assertEqual({(d.left, d.right) for d in rig.log if d.left or d.right}, {(55, 55)})


# --- N6 -----------------------------------------------------------------------------------------------------
class BenchOdometryTest(unittest.TestCase):
    def test_lost_matching_while_active_ends_the_run(self):
        rig = TF.BenchInterlockTest.rig(self)
        rig.est.s['odom_valid'] = False
        out = rig.run(1., True)
        self.assertEqual(rig.runner.state, ACTIVE)                 # up to 1 s
        out = rig.run(.15, True)
        self.assertEqual((rig.runner.state, rig.runner.end_reason), (ENDED, 'bench_odometry_lost'))
        self.assertEqual(TF.nonzero(rig.run(1., True)), [])

    def test_paused_with_invalid_matching_does_not_end(self):
        est = TF.BenchEstimator()
        est.s['odom_valid'] = False
        cfg = ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK, bench_pattern=((10., 40, 40),))
        runner = Runner(cfg, estimator_factory=lambda: est)
        runner.start_bench(0.)
        t = 0.
        for _ in range(60):
            t += DT
            runner.tick(t, False)
        self.assertEqual(runner.state, PAUSED)


# --- N7 -----------------------------------------------------------------------------------------------------
def write_log(path, records):
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        for r in records:
            fh.write(json.dumps(r) + '\n')


def arc_records(run_id, wall, valid, session_config=True):
    """Minimal JSONL of one arc calibration run (as the executor writes it)."""
    out = [dict(k='event', event='config', config={}, t=1., wall=wall)] if session_config else []
    plan = dict(kind='arc', floor_pct=34, max_differential_pct=10, steps=[[55, 45], [45, 55]])
    out.append(dict(k='event', event='calib_start', run_id=run_id, plan=plan, t=2., wall=wall))
    steps = [dict(i=0, L=55, R=45, diff=-10, seconds=4., gyro_yaw_dps=-1.5, lidar_yaw_dps=-1.5, valid=valid),
             dict(i=1, L=45, R=55, diff=10, seconds=4., gyro_yaw_dps=1.5, lidar_yaw_dps=1.5, valid=valid)]
    out.append(dict(k='event', event='calib_result', run_id=run_id, end_reason='calib_complete' if valid else
                    'calib_hold_released', results=steps, plan=plan, t=20., wall=wall + 20.))
    return out


class FieldDayTest(unittest.TestCase):
    def test_no_silent_fallback_to_an_older_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'row_executor_a_1_001.jsonl')
            write_log(path, arc_records(1, 1.7e9, True) + arc_records(2, 1.7e9 + 600., False, session_config=False))
            cfg_path = os.path.join(tmp, 'config.json')
            with open(cfg_path, 'w', encoding='utf-8', newline='\n') as fh:
                fh.write(C.dumps(TC.cfg34()))
            summaries = [CA.summarize(r) for r in CA.load_runs([path])]
            frag = CA.config_fragment(summaries)
            self.assertEqual(frag['fallback'], ['measured_yaw_at_max_diff_deg_s'])
            self.assertTrue(any('WARNING' in n and 'OLDER' in n for n in frag['notes']))
            # unified field procedure: the refusal (exit 4 / --allow-older) moved to tools/field_calib/make_field_config.py
            # (tests/test_make_field_config.py); calib_analysis only reports the fallback and writes no config
            out = os.path.join(tmp, 'merged.json')
            self.assertEqual(CA.main([path, '--config', cfg_path, '--out-config', out]), 2)
            self.assertFalse(os.path.exists(out))
            self.assertEqual(CA.main([path, '--config', cfg_path]), 0)

    def test_older_field_day_left_out(self):
        with tempfile.TemporaryDirectory() as tmp:
            old = os.path.join(tmp, 'row_executor_a_1_001.jsonl')
            new = os.path.join(tmp, 'row_executor_b_2_001.jsonl')
            write_log(old, arc_records(1, 1.7e9 - 2 * 86400., True))
            write_log(new, arc_records(1, 1.7e9, False))
            runs = CA.load_runs([old, new])
            kept, dropped = CA.field_day(runs)
            self.assertEqual((len(kept), dropped), (1, 1))
            frag = CA.config_fragment([CA.summarize(r) for r in kept])
            self.assertNotIn('measured_yaw_at_max_diff_deg_s', frag['fragment'])     # the other day is not used
            frag_all = CA.config_fragment([CA.summarize(r) for r in runs])
            self.assertIn('measured_yaw_at_max_diff_deg_s', frag_all['fragment'])     # (only with --all-days)
            self.assertEqual(frag_all['fallback'], ['measured_yaw_at_max_diff_deg_s'])


# --- N8 (C7) ------------------------------------------------------------------------------------------------
class StopKindTest(unittest.TestCase):
    def test_row_motor_needs_c7(self):
        with self.assertRaisesRegex(ConfigError, 'needs measured_stop_distance_m'):
            row_motor(measured_stop_distance_m=None).validate()
        self.assertIn('measured_stop_distance_m unknown (C7: executor 0/0 stop, calibration run kind stop)',
                      ExecutorConfig().feasibility()['unknown'])
        TC.calib_cfg()                                             # calibration output does not need it

    def test_plan_and_options(self):
        plan = CR.make_plan(TC.cfg34(), 'stop', dict(steady_s=3.))
        self.assertEqual(plan.steps, ((55, 55),))
        self.assertEqual(plan.floor_pct, 34)
        o = CR.plan_options(TC.cfg34())
        self.assertEqual(o['stop'], dict(duty=55, ok=True, why=None))
        for bad in (dict(steady_s=1.), dict(steady_s=7.), dict(arc_set='d10')):
            with self.subTest(bad=bad), self.assertRaises(CR.CalibPlanError):
                CR.make_plan(TC.cfg34(), 'stop', bad)
        with self.assertRaisesRegex(CR.CalibPlanError, 'base 40 below the start minimum 44'):
            CR.make_plan(TC.calib_cfg(breakaway_duty_pct=44, base_duty_pct=40, reduce_duty_pct=40,
                                      max_differential_pct=0, inner_rule_enabled=False), 'stop')

    def test_stop_run_observes_the_fade_stop(self):
        rig = TC.Rig(TC.cfg34(), kind='stop', decel=1.)
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_stop_test')
        self.assertTrue(rig.runner.calib.outcome['steady'])
        v = rig.runner.calib.outcome['speed_m_s']
        self.assertAlmostEqual(v, .0085 * 55, delta=.01)
        out = rig.run(3., False)
        stop = rig.runner.calib.stop
        self.assertTrue(stop['done'] and stop['ok'], stop)
        self.assertAlmostEqual(stop['stop_distance_m'], v * v / 2., delta=.03)     # plant: decel 1 m/s^2
        self.assertAlmostEqual(stop['v_at_zero_m_s'], v, delta=.01)
        self.assertLess(stop['time_to_rest_s'], 1.)
        self.assertEqual(out[-1].summary['calib']['stop']['stop_distance_m'], stop['stop_distance_m'])

    def test_through_the_loop_fade_wait_and_fragment(self):
        records = []
        for k in range(3):
            # slow fade (0.25 m/s^2): rest after ~1.9 s, beyond the normal 1.5 s op stop
            rig, n = calib_through_loop(self, TC.cfg34(), 'stop', plant=dict(decel=.25), press='stop-%d' % k)
            loop = rig.loop
            t_end = rig.clock.t
            rig.run(5., hold=True)
            self.assertEqual(loop.runner.end_reason, 'calib_stop_test')
            ops = rig.transport.requests[n:]
            stop_i = next(i for i, r in enumerate(ops) if r['op'] == 'stop')
            self.assertTrue(all(r['op'] == 'drive' and (r['left'], r['right']) == (0, 0) for r in ops[:stop_i]))
            self.assertGreater(stop_i, 15)                        # 0/0 kept well past the usual 1.5 s
            self.assertIsNone(rig.control.lease)
            events = [r for r in loop.log.records if r['k'] == 'event' and r['event'] == 'calib_stop']
            self.assertEqual(len(events), 1)
            ev = events[0]
            self.assertTrue(ev['stop']['ok'], ev)
            self.assertFalse(ev['emergency'])
            self.assertEqual(ev['terminal_stop_wait_s'], CR.STOP_TEST_WAIT_S)
            v = ev['stop']['v_at_zero_m_s']
            self.assertAlmostEqual(ev['stop']['stop_distance_m'], v * v / (2 * .25), delta=.05)
            self.assertGreater(ev['stop']['time_to_rest_s'], 1.5)
            self.assertLess(t_end, rig.clock.t)
            records += [dict(r, wall=1.7e9 + 100. * k) if r.get('k') == 'event' else r for r in loop.log.records]
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'row_executor_c7_1_001.jsonl')
            write_log(path, records)
            summaries = [CA.summarize(r) for r in CA.load_runs([path])]
            self.assertEqual([s['kind'] for s in summaries], ['stop', 'stop', 'stop'])
            frag = CA.config_fragment(summaries)
            value = frag['fragment']['measured_stop_distance_m']
            largest = max(s['stop']['stop_distance_m'] for s in summaries)
            self.assertAlmostEqual(value, math.ceil(largest * 100 - 1e-9) / 100)
            self.assertEqual(frag['fallback'], [])
            text = '\n'.join(CA.report_lines(summaries, frag))
            self.assertIn('0/0 뒤 정지', text)
            two = CA.config_fragment(summaries[:2])
            self.assertNotIn('measured_stop_distance_m', two['fragment'])
            self.assertTrue(any('need 3' in n for n in two['notes']))

    def test_c7_rule_against_the_model(self):
        frag = dict(fragment=dict(measured_stop_distance_m=.5), notes=[])
        out = CA.fit_fragment(frag, row_motor())
        self.assertTrue(any('exceeds the stop model' in n for n in out['notes']))
        merged, problems = CA.merge_config(row_motor(), dict(measured_stop_distance_m=.5))
        self.assertIsNone(merged)
        self.assertTrue(any('measured C7 stop distance' in p for p in problems))


# --- N9 -----------------------------------------------------------------------------------------------------
class RepeatedPressNoResponseTest(unittest.TestCase):
    def test_short_presses_can_not_push_a_stuck_vehicle_forever(self):
        rig = TF.Rig(stuck=True)
        rig.start_driving()
        rig.run(2.2, True)                                         # 2.5 s press: below the controller's 3 s test
        presses = 1
        while rig.runner.state != ENDED and presses < 6:
            rig.run(.3, False)
            rig.run(2.5, True)
            presses += 1
        self.assertEqual(rig.runner.end_reason, 'drive_no_response_repeat')
        self.assertEqual(presses, 3)                               # 3 x ~2.4 s > 6 s budget
        self.assertEqual(TF.nonzero(rig.run(.3, False) + rig.run(2., True)), [])

    def test_motion_resets_the_budget(self):
        rig = TF.Rig()
        rig.start_driving()
        for _ in range(4):
            rig.run(2.5, True)
            rig.run(.6, False)
        rig.run(2.5, True)
        self.assertEqual(rig.runner.state, ACTIVE)
        self.assertLess(rig.runner.no_motion_drive_s, .2)


# --- R1 -----------------------------------------------------------------------------------------------------
class RolldownFloorTest(unittest.TestCase):
    def test_floor_range_and_default(self):
        cfg = TC.cfg34()
        plan = CR.make_plan(cfg, 'rolldown')
        self.assertEqual(plan.floor_pct, CR.ROLLDOWN_DEFAULT_FLOOR_PCT)
        self.assertEqual(plan.steps[-1], (25, 25))
        self.assertLess(plan.floor_pct, cfg.breakaway_duty_pct)      # below the static breakaway
        self.assertEqual(CR.make_plan(cfg, 'rolldown', dict(floor_pct=20)).steps[-1], (20, 20))
        for bad in (19, 53):
            with self.subTest(floor=bad), self.assertRaises(CR.CalibPlanError):
                CR.make_plan(cfg, 'rolldown', dict(floor_pct=bad))

    def test_measured_rolling_below_static_feeds_the_row_rules(self):
        rig = TC.Rig(TC.calib_cfg(breakaway_duty_pct=40, max_differential_pct=5), kind='rolldown', static=40,
                     rolling=27)
        rig.go()
        rig.until_end()
        r = rig.runner.calib.outcome['rolling_breakaway_duty_pct']
        self.assertEqual(r, 28)                                     # 55 52 ... 31 28 kept, 25 stalls
        # decision table O4: diff = min(20, 45 - R) = 17 is now a valid row config (static rule alone: 5)
        row_motor(breakaway_duty_pct=40, rolling_breakaway_duty_pct=r, max_differential_pct=17,
                  measured_yaw_at_max_diff_deg_s=5.1, max_yaw_rate_deg_s=3.0).validate()
        with self.assertRaises(ConfigError):
            row_motor(breakaway_duty_pct=40, max_differential_pct=17, measured_yaw_at_max_diff_deg_s=5.1,
                      max_yaw_rate_deg_s=3.0).validate()


# --- R2 -----------------------------------------------------------------------------------------------------
class OnsiteYawAuthorityTest(unittest.TestCase):
    def test_lower_differential_scales_the_authority(self):
        f = row_motor().validate()                                  # diff 20, measured 6.0, max_yaw 3.6
        # steering GO rule (field decision rules 2026-10-22): row motor output keeps the differential at >= 10 and the
        # scaled authority at >= 4.0 deg/s; so diff 14 (4.2) is the lowest allowed here
        expect = {14: (4.2, 2.5), 20: (6.0, 3.6)}
        for diff, (yaw, max_yaw) in expect.items():
            new = C.onsite_update(f, f, dict(max_differential_pct=diff))
            self.assertEqual((new.measured_yaw_at_max_diff_deg_s, new.max_yaw_rate_deg_s), (yaw, max_yaw), diff)
            self.assertEqual(new.feasibility()['status'], 'ok')
        for diff in (13, 10, 8, 6, 4):                              # authority 3.9 ... 1.2 deg/s < 4.0: refused
            with self.subTest(diff=diff), self.assertRaisesRegex(ConfigError, 'yaw authority|max_differential_pct'):
                C.onsite_update(f, f, dict(max_differential_pct=diff))
        lowered = C.onsite_update(f, f, dict(max_differential_pct=14))
        back = C.onsite_update(f, lowered, dict(max_differential_pct=20))
        self.assertEqual((back.measured_yaw_at_max_diff_deg_s, back.max_yaw_rate_deg_s), (6.0, 3.6))

    def test_never_above_the_file_max_yaw_and_shadow_unknown(self):
        f = row_motor(max_yaw_rate_deg_s=1.5).validate()
        self.assertEqual(C.onsite_update(f, f, dict(max_differential_pct=15)).max_yaw_rate_deg_s, 1.5)
        shadow = ExecutorConfig()
        self.assertIsNone(C.onsite_update(shadow, shadow, dict(max_differential_pct=10)).measured_yaw_at_max_diff_deg_s)

    def test_through_the_loop_params(self):
        rig = LoopRig(row_motor().validate(), motor=True)
        fut = rig.loop.submit('params', params=dict(max_differential_pct=14))
        rig.run(.05)
        result = fut.result()
        self.assertTrue(result['ok'], result)
        self.assertEqual(result['derived'], dict(measured_yaw_at_max_diff_deg_s=4.2, max_yaw_rate_deg_s=2.5))
        self.assertEqual(rig.loop.status()['yaw_params']['max_yaw_rate_deg_s'], 2.5)
        fut = rig.loop.submit('params', params=dict(max_differential_pct=10))      # 3.0 deg/s < 4.0
        rig.run(.05)
        self.assertEqual(fut.result()['error'], 'invalid_params')


# --- R5 -----------------------------------------------------------------------------------------------------
class LagPlant(TC.Plant):
    """Yaw follows gain x (R - L) with a first-order lag (the review probe's plant)."""
    TAU = .6

    def snapshot(self, now):
        s = super().snapshot(now)
        if not hasattr(self, '_yaw'):
            self._yaw, self._h, self._tl = 0., 0., now
        dt, self._tl = now - self._tl, now
        if dt > 0:
            self._yaw += (s['yaw_rate'] - self._yaw) * (1 - math.exp(-dt / self.TAU))
        self._h += self._yaw * dt
        s.update(yaw_rate=self._yaw, gyro_heading=self._h, lidar_heading=math.atan2(math.sin(self._h), math.cos(self._h)))
        return s


class CrossCheckTest(unittest.TestCase):
    def test_recomputation_matches_the_runtime_window(self):
        box = {}

        def runner_factory(c):
            runner = Runner(c, estimator_factory=lambda: LagPlant(box))
            box['runner'] = runner
            return runner
        rig = LoopRig(TC.cfg34(), motor=True, runner_factory=runner_factory)
        loop = rig.loop
        fut = loop.submit('calib_start', calib_kind='arc', params=dict(step_s=4.))
        rig.run(1.5)
        self.assertTrue(fut.result()['ok'])
        loop.submit('arm')
        rig.run(.05)
        rig.press('lag-1')
        for _ in range(900):
            rig.run(.05, hold=True)
            if loop.runner.state == ENDED:
                break
        self.assertEqual(loop.runner.end_reason, 'calib_complete')
        ticks = [r for r in loop.log.records if r['k'] == 'tick' and 'calib' in r]
        for a, b in zip(ticks, ticks[1:]):
            if a['calib']['i'] != b['calib']['i']:
                self.assertEqual(b['calib']['ph'], 'settle')        # a new step never starts tagged 'measure'
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'row_executor_lag_1_001.jsonl')
            write_log(path, loop.log.records)
            s = CA.summarize(CA.load_runs([path])[0])
        self.assertTrue(s['cross_check']['ok'], s['cross_check'])
        self.assertLess(s['cross_check']['gyro_dps'], .1)
        for step, rec in zip(s['steps'], s['recomputed']):
            self.assertAlmostEqual(step['seconds'], rec['seconds'], delta=.06)


# --- R6 -----------------------------------------------------------------------------------------------------
class RunEndLogTest(unittest.TestCase):
    def test_end_note_in_ticks_and_run_end_event(self):
        rig = LoopRig(row_motor().validate(), motor=True)
        loop = rig.loop
        fut = loop.submit('start', row_len_m=40.)
        rig.run(1.5)
        self.assertTrue(fut.result()['ok'])
        loop.submit('arm')
        rig.run(.05)
        loop.request_end('test_end')                              # any end: one run_end event
        rig.run(.2)
        events = [r for r in loop.log.records if r['k'] == 'event' and r['event'] == 'run_end']
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['end_reason'], 'test_end')
        loop.runner.end_note = 'terminal_stop_pending_at_hold_released'          # as S1 sets it
        rig.run(.1)
        ticks = [r for r in loop.log.records if r['k'] == 'tick' and r.get('state') == 'ENDED']
        self.assertEqual(ticks[-1]['end_note'], 'terminal_stop_pending_at_hold_released')
        self.assertEqual(len([r for r in loop.log.records if r.get('event') == 'run_end']), 1)


# --- N5 -----------------------------------------------------------------------------------------------------
class ForwardSectorTest(unittest.TestCase):
    def test_forward_segments(self):
        result = dict(segments=[dict(bearing_deg=[-0.4, 0.3]), dict(bearing_deg=[34., 36.]),
                                dict(bearing_deg=[36., 40.]), dict(bearing_deg=[-120., -100.])])
        self.assertEqual([s['bearing_deg'] for s in B.forward_segments(result)], [[-0.4, 0.3], [34., 36.]])


if __name__ == '__main__':
    unittest.main()
