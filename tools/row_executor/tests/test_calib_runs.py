"""Calibration run kinds 'arc' (C5) and 'rolldown' (C3b): plan bounds, config keys, Runner behaviour, ExecutorLoop
(real dashboard RoverControl behind the real BridgeClient), JSONL adapter (calib_analysis) and the operator route.

The Runner tests use a kinematic fake estimator (Plant) driven by the Runner's last output: the vehicle starts when
both wheels reach the static breakaway, keeps rolling while both stay at or above the rolling threshold, speed is
proportional to the mean duty and the yaw rate to R - L. Numbers are test values, not field values.
"""
import json
import math
import os
import tempfile
import unittest

import rowexec_testlib as L  # noqa: F401  (path setup)
from row_executor import calib_analysis as CA
from row_executor import calib_runs as CR
from row_executor import config as C
from row_executor.config import ConfigError, ExecutorConfig
from row_executor.core import ACTIVE, ENDED, PAUSED, Runner

DT = .05


def calib_cfg(**over):
    base = dict(calib_motor_output_enabled=True)
    base.update(over)
    return ExecutorConfig(**base).validate()


def cfg34(**over):
    """Measured breakaway 34: the row rules still apply to the file (base - diff >= 34 + 10), so diff 10."""
    return calib_cfg(breakaway_duty_pct=34, max_differential_pct=10, **over)


class Plant:
    """Kinematic fake estimator (see the module docstring)."""

    def __init__(self, box, static=34, rolling=30, v_per_pct=.0085, gain=.15, ramp_s=.3, decel=1.,
                 front_at=None, side_left_at=None, odom_bad=None, stuck=False, yaw_bias_dps=0., lidar_age=None,
                 stall_after_s=None):
        self.box = box
        self.static, self.rolling, self.k, self.gain = static, rolling, v_per_pct, gain
        self.ramp_s, self.decel = ramp_s, decel
        self.front_at, self.side_left_at, self.odom_bad, self.stuck = front_at, side_left_at, odom_bad, stuck
        self.yaw_bias = yaw_bias_dps
        self.lidar_age = lidar_age
        self.stall_after_s = stall_after_s      # after this much motion the ground stops every wheel (rolling 100)
        self.moving_s = 0.
        self.t = self.t0 = None
        self.moving = False
        self.go_since = None
        self.speed = 0.
        self.dist = 0.
        self.heading = 0.
        self.still_since = None
        self.scans = 0
        self.last_scan_t = None

    def on_imu(self, *a):
        pass

    def on_scan(self, *a):
        pass

    def snapshot(self, now):
        if self.t is None:
            self.t = self.t0 = now
            self.still_since = now - 10.
            self.last_scan_t = now
        dt, self.t = now - self.t, now
        left, right = self.box['runner']._last_output
        inner = min(left, right)
        rolling = 101 if self.stall_after_s is not None and self.moving_s >= self.stall_after_s else self.rolling
        keep = left > 0 and right > 0 and inner >= rolling
        if not self.moving:
            if not self.stuck and left > 0 and right > 0 and inner >= self.static:
                self.go_since = now if self.go_since is None else self.go_since
                if now - self.go_since >= self.ramp_s:
                    self.moving = True
            else:
                self.go_since = None
        yaw_dps = 0.
        if self.moving and keep:
            self.speed = self.k * .5 * (left + right)
            yaw_dps = self.gain * (right - left) + self.yaw_bias
        elif self.moving:
            self.speed = max(0., self.speed - self.decel * dt)
            if self.speed <= 0.:
                self.moving = False
                self.go_since = None
        if self.moving:
            self.still_since = None
            self.moving_s += dt
        elif self.still_since is None:
            self.still_since = now
        self.dist += self.speed * dt
        self.heading += math.radians(yaw_dps) * dt
        while now - self.last_scan_t >= .1:
            self.last_scan_t += .1
            self.scans += 1
        rel = now - self.t0
        odom = not (self.odom_bad and self.odom_bad[0] <= rel < self.odom_bad[1])
        side_left = .01 if self.side_left_at is not None and self.dist >= self.side_left_at else 1.2
        still_s = 0. if self.moving else now - self.still_since
        return dict(lidar_age_s=(now - self.last_scan_t) if self.lidar_age is None else self.lidar_age(rel),
                    imu_age_s=.02, still=still_s > .45, still_s=still_s if still_s > .45 else 0.,
                    speed=self.speed, odom_valid=odom, distance_m=self.dist, yaw_rate=math.radians(yaw_dps),
                    gyro_heading=self.heading, lidar_heading=math.atan2(math.sin(self.heading), math.cos(self.heading)),
                    gyro_bias=0., scan_count=self.scans, corridor_valid=False, corridor_reason='waiting',
                    front_clearance_m=30. if self.front_at is None else self.front_at - self.dist,
                    side_clearance_left_m=side_left, side_clearance_right_m=1.2,
                    side_nearest_x_left_m=.3 if side_left < 1. else None, side_nearest_x_right_m=None,
                    rotation_centre_x_m=0., rotation_centre_sd_m=0., vibration_rms=.1)


class Rig:
    def __init__(self, cfg=None, kind='arc', params=None, plan=None, **plant):
        self.box = {}
        self.plant = None
        cfg = cfg or cfg34()

        def factory():
            self.plant = Plant(self.box, **plant)
            return self.plant
        self.runner = Runner(cfg, estimator_factory=factory)
        self.box['runner'] = self.runner
        self.t = 1000.
        self.plan = plan or CR.make_plan(cfg, kind, params or {})
        self.runner.start_calibration(self.t, self.plan)
        self.log = []

    def run(self, seconds, hold):
        out = []
        for _ in range(int(round(seconds / DT))):
            self.t += DT
            out.append(self.runner.tick(self.t, hold))
        self.log += out
        return out

    def go(self, expect_active=True):
        self.run(1.5, False)
        out = self.run(DT, True)
        if expect_active:
            assert self.runner.state == ACTIVE, self.runner.reason
        return out

    def until_end(self, max_s=90.):
        out = []
        for _ in range(int(round(max_s / DT))):
            out += self.run(DT, True)
            if self.runner.state == ENDED:
                return out
        raise AssertionError('calibration run did not end within %g s' % max_s)


# --- plans ------------------------------------------------------------------------------------------------------
class PlanTest(unittest.TestCase):
    def test_arc_default_is_the_task_table_with_measured_breakaway(self):
        plan = CR.make_plan(cfg34(), 'arc')
        self.assertEqual(plan.steps, ((55, 45), (45, 55), (55, 35), (35, 55)))
        self.assertEqual((plan.floor_pct, plan.floor_source), (34, 'breakaway_duty_pct'))
        self.assertAlmostEqual(plan.step_m, .9 * 15. / 4)

    def test_unknown_breakaway_minimum_is_40(self):
        cfg = calib_cfg()
        self.assertEqual(CR.floor_pct(cfg), (40, 'unknown_breakaway_default'))
        with self.assertRaises(CR.CalibPlanError) as ctx:
            CR.make_plan(cfg, 'arc')                                # (55, 35): inner 35 < 40
        self.assertTrue(any('inner 35 below the minimum duty 40' in p for p in ctx.exception.problems))
        self.assertEqual(CR.make_plan(cfg, 'arc', dict(arc_set='d10')).steps, ((55, 45), (45, 55)))

    def test_minimum_is_the_measured_breakaway_itself(self):
        cfg = calib_cfg(breakaway_duty_pct=46, base_duty_pct=60, reduce_duty_pct=60, max_differential_pct=4)
        with self.assertRaises(CR.CalibPlanError):
            CR.make_plan(cfg, 'arc', dict(arc_set='d20'))          # inner 40 < 46: refused, never clipped
        self.assertEqual(CR.make_plan(cfg, 'arc', dict(arc_set='d10')).steps, ((60, 50), (50, 60)))
        plan = CR.make_plan(cfg, 'rolldown')
        # day-2 R1: the rolldown floor (default 25) is independent of the static breakaway, so a rolling threshold
        # below it is measurable; the start (both wheels from standstill) still needs the breakaway
        self.assertEqual([d for d, _ in plan.steps], [60, 57, 54, 51, 48, 45, 42, 39, 36, 33, 30, 27, 25])
        self.assertEqual((plan.floor_pct, plan.floor_source), (25, 'rolldown_floor'))
        self.assertEqual((plan.start_floor_pct, plan.start_floor_source), (46, 'breakaway_duty_pct'))
        self.assertTrue(all(left == right for left, right in plan.steps))

    def test_caps_and_bounded_inputs(self):
        cfg = cfg34()
        bad = [dict(arc_set='d99'), dict(step_s=2.), dict(step_s=8.5), dict(distance_m=16.), dict(distance_m=1.),
               dict(time_s=46.), dict(time_s=True), dict(turbo=1), dict(step_pct=3)]
        for params in bad:
            with self.assertRaises(CR.CalibPlanError, msg=str(params)):
                CR.make_plan(cfg, 'arc', params)
        for params in (dict(step_pct=6), dict(step_pct=3.), dict(hold_s=1.5), dict(floor_pct=19), dict(floor_pct=55),
                       dict(arc_set='d10')):
            with self.assertRaises(CR.CalibPlanError, msg=str(params)):
                CR.make_plan(cfg, 'rolldown', params)
        with self.assertRaises(CR.CalibPlanError):
            CR.make_plan(cfg, 'pivot')
        with self.assertRaisesRegex(CR.CalibPlanError, 'outer 68 above duty_cap_pct 66'):
            CR.make_plan(calib_cfg(duty_cap_pct=66, base_duty_pct=58, reduce_duty_pct=58, max_differential_pct=5,
                                   diff_cap_pct=20), 'arc', dict(arc_set='sym20', distance_m=10.))
        with self.assertRaisesRegex(CR.CalibPlanError, r'\|L-R\| 20 above diff_cap_pct 15'):
            CR.make_plan(calib_cfg(max_differential_pct=7, diff_cap_pct=15), 'arc', dict(arc_set='sym20'))
        plan = CR.make_plan(cfg, 'arc', dict(arc_set='d20', step_s=6., distance_m=8., time_s=30))
        self.assertEqual(plan.params, dict(arc_set='d20', step_s=6., distance_m=8., time_s=30.))
        self.assertAlmostEqual(plan.step_m, 3.6)
        with self.assertRaisesRegex(CR.CalibPlanError, 'per arc step'):
            CR.make_plan(cfg, 'arc', dict(distance_m=8.))            # 4 steps: 1.8 m each

    def test_rolldown_duties_and_raised_floor(self):
        self.assertEqual(CR.rolldown_duties(55, 3, 40), (55, 52, 49, 46, 43, 40))
        self.assertEqual(CR.rolldown_duties(55, 4, 40), (55, 51, 47, 43, 40))
        self.assertEqual(len(CR.rolldown_duties(70, 2, 40)), 16)
        plan = CR.make_plan(calib_cfg(), 'rolldown', dict(step_pct=5, floor_pct=45))
        self.assertEqual([d for d, _ in plan.steps], [55, 50, 45])
        self.assertEqual(plan.floor_pct, 45)
        with self.assertRaises(CR.CalibPlanError):           # base 55 - 5 below a floor 52
            CR.make_plan(calib_cfg(), 'rolldown', dict(step_pct=5, floor_pct=52))

    def test_bench_profile_refused(self):
        bench = ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK, bench_pattern=((2., 40, 40),))
        with self.assertRaises(CR.CalibPlanError):
            CR.make_plan(bench, 'arc', dict(arc_set='d10'))

    def test_options_list_every_set(self):
        o = CR.plan_options(calib_cfg())
        self.assertEqual(o['floor_pct'], 40)
        ids = {a['id']: a for a in o['arc_sets']}
        self.assertEqual(set(ids), set(CR.ARC_SETS))
        self.assertFalse(ids['d10_d20']['ok'])
        self.assertTrue(ids['d10']['ok'])
        self.assertEqual([r['step_pct'] for r in o['rolldown']], list(CR.ROLL_STEP_PCT))
        json.dumps(o, allow_nan=False)


class ConfigTest(unittest.TestCase):
    def test_keys_validation(self):
        self.assertFalse(ExecutorConfig().calib_motor_output_enabled)
        cfg = calib_cfg()
        self.assertEqual(C.calib_output_problems(cfg), [])
        self.assertEqual(cfg.feasibility()['status'], 'unknown')        # row output still needs the measurements
        self.assertTrue(C.calib_output_problems(ExecutorConfig()))
        with self.assertRaisesRegex(ConfigError, 'field profile only'):
            ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK, bench_pattern=((2., 40, 40),),
                           calib_motor_output_enabled=True).validate()
        with self.assertRaisesRegex(ConfigError, 'guard_heading_deg'):
            calib_cfg(guard_heading_deg=None)
        with self.assertRaisesRegex(ConfigError, 'stop model'):
            calib_cfg(controller_overrides={'stop_decel_m_s2': 3.})
        with self.assertRaisesRegex(ConfigError, 'lidar_max_age_s'):
            calib_cfg(lidar_max_age_s=.6)
        for key, value in (('calib_distance_cap_m', 21.), ('calib_distance_cap_m', 1.), ('calib_time_cap_s', 100.),
                           ('calib_motor_output_enabled', 1)):
            with self.assertRaises(ConfigError, msg=key):
                C.from_dict({key: value})

    def test_json_round_trip_and_shipped_example(self):
        cfg = calib_cfg(calib_distance_cap_m=10., calib_time_cap_s=30.)
        self.assertEqual(C.loads(C.dumps(cfg)), cfg)
        path = os.path.join(L.TOOLS, 'row_executor', 'config.field_shadow.example.json')
        self.assertFalse(C.load_config(path).calib_motor_output_enabled)

    def test_onsite_only_lowers_with_calibration_output(self):
        cfg = calib_cfg()
        self.assertEqual(C.onsite_update(cfg, cfg, dict(base_duty_pct=50)).base_duty_pct, 50)
        with self.assertRaisesRegex(ConfigError, 'only <= the measured file value'):
            C.onsite_update(cfg, cfg, dict(base_duty_pct=58))


# --- Runner ---------------------------------------------------------------------------------------------------
class ArcRunTest(unittest.TestCase):
    def test_complete_arc_measures_both_sides(self):
        rig = Rig(params=dict(step_s=3.))
        before = rig.go()
        self.assertEqual((before[-1].left, before[-1].right), (55, 55))  # day-2 N4: (base, base) until it rolls
        out = rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_complete')
        driving = {(d.left, d.right) for d in rig.log if d.left or d.right}
        self.assertEqual(driving, set(rig.plan.steps) | {(55, 55)})     # only the start and the table
        self.assertTrue(all(d.left >= 0 and d.right >= 0 for d in rig.log))
        c = rig.runner.calib
        self.assertEqual(len(c.results), 4)
        self.assertTrue(all(r['valid'] for r in c.results))
        for r in c.results:
            self.assertAlmostEqual(r['gain_dps_per_pct'], .15, delta=.01)
            self.assertAlmostEqual(r['gyro_yaw_dps'], .15 * r['diff'], delta=.1)
            self.assertAlmostEqual(r['lidar_yaw_dps'], .15 * r['diff'], delta=.1)
            self.assertAlmostEqual(r['speed_m_s'], .0085 * .5 * (r['L'] + r['R']), delta=.01)
        o = c.outcome
        self.assertAlmostEqual(o['sides']['left']['gain'], .15, delta=.005)
        self.assertAlmostEqual(o['sides']['right']['gain'], .15, delta=.005)
        self.assertAlmostEqual(o['yaw_at_full_diff_dps'], .15 * 20, delta=.1)     # 2 x max_differential 10
        self.assertFalse(o['extrapolated'])                                        # |L-R| 20 was measured
        self.assertEqual(out[-1].state, ENDED)
        self.assertEqual((out[-1].left, out[-1].right), (0, 0))
        self.assertEqual(out[-1].summary['calib']['end_reason'], 'calib_complete')

    def test_steps_end_by_distance(self):
        rig = Rig(params=dict(arc_set='d10', step_s=8., distance_m=5.))       # 2.25 m per step at ~0.43 m/s
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_complete')
        for r in rig.runner.calib.results:
            self.assertTrue(r['valid'])
            self.assertLess(r['seconds'], 8.)
        self.assertLess(rig.runner.calib.counted, 5.)

    def test_release_ends_the_run_and_a_new_press_does_nothing(self):
        rig = Rig()
        rig.go()
        rig.run(4., True)
        rig.run(DT, False)
        self.assertEqual(rig.runner.state, ENDED)
        self.assertEqual(rig.runner.end_reason, 'calib_hold_released')
        rig.run(1., False)
        out = rig.run(2., True)
        self.assertTrue(all((d.left, d.right) == (0, 0) and d.state == ENDED for d in out))
        self.assertIsNotNone(rig.runner.calib.outcome)

    def test_press_before_fresh_inputs_is_ignored_like_a_row_run(self):
        rig = Rig()
        rig.run(.3, False)
        rig.run(.2, True)
        self.assertEqual(rig.runner.state, PAUSED)
        self.assertEqual(rig.runner.reason, 'press_ignored_inputs_not_fresh')
        self.assertFalse(rig.runner.calib.started)

    def test_stale_lidar_ends(self):
        rig = Rig(lidar_age=lambda rel: .05 if rel < 4. else .6)
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_stale_lidar')

    def test_stall_watchdog_pause_ends(self):
        rig = Rig()
        rig.go()
        rig.run(1., True)
        rig.runner.pause(rig.t, 'stall_guard')
        self.assertEqual(rig.runner.end_reason, 'calib_stall_guard')

    def test_front_obstacle_refuses_the_start_and_stops_a_running_arc(self):
        rig = Rig(front_at=.3)
        rig.go(expect_active=False)
        self.assertEqual(rig.runner.end_reason, 'calib_front_blocked')
        self.assertTrue(all((d.left, d.right) == (0, 0) for d in rig.log))
        rig = Rig(front_at=2.)
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_front_blocked')
        moving = [d for d in rig.log if d.left or d.right]
        self.assertTrue(moving)
        stop_at = rig.plant.dist
        self.assertLess(stop_at, 2. - .35)                               # stop model + 0.35 m margin respected

    def test_side_blocked_needs_two_distinct_scans(self):
        rig = Rig(side_left_at=1.)
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_side_blocked')

    def test_no_response_overspeed_yaw_heading(self):
        rig = Rig(stuck=True)
        rig.go()
        out = rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_no_response')
        self.assertAlmostEqual(rig.t - (1000. + 1.55), 3., delta=.15)
        rig = Rig(v_per_pct=.015)                                        # mean 50 % -> 0.75 m/s > 0.5 + 0.15
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_overspeed')
        rig = Rig(gain=.7)                                               # 0.7 x 20 = 14 deg/s > 12
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_yaw_rate')
        rig = Rig(gain=1.1, params=dict(arc_set='d10', step_s=8.))       # 11 deg/s for > 5.5 s: > 60 deg per step
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_heading_change')

    def test_caps(self):
        rig = Rig(kind='rolldown', params=dict(distance_m=2.), static=34, rolling=30)
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_distance_cap')
        self.assertLess(rig.plant.dist, 2.5)
        rig = Rig(params=dict(time_s=6.))
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_time_cap')

    def test_odometry_lost_ends(self):
        rig = Rig(odom_bad=(4., 99.))
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_odometry_lost')

    def test_inner_wheel_that_can_not_roll_ends_the_arc(self):
        rig = Rig(rolling=38)                                            # (55, 35): 35 < 38 -> the vehicle stops
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_arc_stalled')
        self.assertEqual([r['valid'] for r in rig.runner.calib.results], [True, True, False])

    def test_explicit_minimum_guard(self):
        import dataclasses
        cfg = cfg34()
        bad = dataclasses.replace(CR.make_plan(cfg, 'arc'), steps=((55, 30), (55, 45)))   # 30 < floor 34: a bug
        rig = Rig(cfg, plan=bad)
        rig.go()
        rig.until_end()                                                  # the pair starts once the vehicle rolls
        rig.run(.15, True)
        self.assertEqual(rig.runner.end_reason, 'guard_violation:inner_below_breakaway')
        self.assertEqual(rig.runner.guard.violations, 1)
        self.assertTrue(all((d.left, d.right) == (0, 0) for d in rig.log[-3:]))

    def test_needs_field_profile_and_a_plan(self):
        bench = ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK, bench_pattern=((2., 40, 40),))
        runner = Runner(bench, estimator_factory=lambda: Plant({}))
        with self.assertRaisesRegex(RuntimeError, 'field_profile'):
            runner.start_calibration(1., CR.make_plan(cfg34(), 'arc'))
        runner = Runner(calib_cfg(), estimator_factory=lambda: Plant({}))
        with self.assertRaisesRegex(RuntimeError, 'plan_required'):
            runner.start_calibration(1., dict(kind='arc'))


class RolldownRunTest(unittest.TestCase):
    def test_floor_reached_unknown_breakaway(self):
        rig = Rig(calib_cfg(), kind='rolldown', params=dict(floor_pct=40), static=44, rolling=34)
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'rolldown_floor_reached')
        o = rig.runner.calib.outcome
        self.assertEqual((o['lowest_sustained_pct'], o['floor_reached'], o['rolling_breakaway_duty_pct']), (40, True, 40))
        duties = [d.left for d in rig.log if d.left]
        self.assertEqual(sorted(set(duties), reverse=True), [55, 52, 49, 46, 43, 40])
        self.assertTrue(all(d.left == d.right for d in rig.log))

    def test_stall_gives_the_threshold(self):
        rig = Rig(calib_cfg(), kind='rolldown', static=44, rolling=45)
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'rolldown_stalled')
        o = rig.runner.calib.outcome
        self.assertEqual((o['lowest_sustained_pct'], o['stalled_at_pct']), (46, 43))
        self.assertFalse(o['floor_reached'])
        self.assertEqual(rig.runner.calib.results[-1]['stalled'], True)

    def test_rolling_threshold_below_the_measured_breakaway(self):
        # review_correctness_day2 R1: with C2 (44) in the config the rolldown goes below it and finds the rolling
        # threshold 34 (before the fix the floor was 44 and C3b could never be below C2)
        rig = Rig(calib_cfg(breakaway_duty_pct=44, max_differential_pct=1), kind='rolldown', static=44, rolling=34)
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'rolldown_stalled')
        o = rig.runner.calib.outcome
        self.assertEqual((o['rolling_breakaway_duty_pct'], o['stalled_at_pct']), (34, 31))
        self.assertLess(min(d.left for d in rig.log if d.left), 44)
        self.assertTrue(all(d.left == d.right for d in rig.log))

    def test_base_that_does_not_keep_rolling(self):
        rig = Rig(calib_cfg(), kind='rolldown', static=44, rolling=34, stall_after_s=.5)
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'rolldown_stalled_at_base')
        self.assertIsNone(rig.runner.calib.outcome['rolling_breakaway_duty_pct'])

    def test_yaw_cap_is_tighter(self):
        rig = Rig(calib_cfg(), kind='rolldown', static=40, rolling=30, yaw_bias_dps=7.)
        rig.go()
        rig.until_end()
        self.assertEqual(rig.runner.end_reason, 'calib_yaw_rate')


# --- ExecutorLoop + real bridge core ----------------------------------------------------------------------------
class LoopTest(unittest.TestCase):
    def rig(self, cfg, motor=True, **plant):
        from test_executor_loop import Rig as LoopRig
        box = {}

        def runner_factory(c):
            runner = Runner(c, estimator_factory=lambda: Plant(box, **plant))
            box['runner'] = runner
            return runner
        return LoopRig(cfg, motor=motor, runner_factory=runner_factory)

    def test_calibration_through_the_bridge(self):
        rig = self.rig(cfg34())
        loop = rig.loop
        fut = loop.submit('start', row_len_m=30.)            # a row run in a calibration-only config ...
        rig.run(1.5)
        self.assertTrue(fut.result()['ok'])
        fut = loop.submit('arm')
        rig.run(.05)
        self.assertEqual(fut.result()['error'], 'row_motor_output_not_enabled')     # ... is never armed
        fut = loop.submit('end')
        rig.run(.05)
        fut = loop.submit('calib_start', calib_kind='arc', params=dict(step_s=3.))
        rig.run(1.5)
        result = fut.result()
        self.assertTrue(result['ok'], result)
        self.assertEqual(result['plan']['steps'], [[55, 45], [45, 55], [55, 35], [35, 55]])
        fut = loop.submit('arm')
        rig.run(.05)
        self.assertTrue(fut.result()['ok'], fut.result())
        rig.press('cal-1')
        for _ in range(600):
            rig.run(.05, hold=True)
            if loop.runner.state == ENDED:
                break
        self.assertEqual(loop.runner.end_reason, 'calib_complete')
        rig.run(3., hold=True)
        drives = [r for r in rig.transport.requests if r['op'] == 'drive']
        self.assertTrue(all(r['deadman'] is True for r in drives))
        sent = {(r['left'], r['right']) for r in drives if r['left'] or r['right']}
        self.assertEqual(sent, {(55, 55), (55, 45), (45, 55), (55, 35), (35, 55)})     # (base, base) start: N4
        self.assertEqual(rig.transport.ops()[-1], 'stop')                 # normal end: 0/0, still, lease released
        self.assertIsNone(rig.control.lease)
        records = loop.log.records
        events = [r['event'] for r in records if r['k'] == 'event']
        self.assertEqual(events.count('calib_start'), 1)
        self.assertEqual(events.count('calib_step'), 4)
        self.assertEqual(events.count('calib_result'), 1)
        ticks = [r for r in records if r['k'] == 'tick' and 'calib' in r]
        self.assertTrue(ticks and all(set(r['calib']) >= {'k', 'i', 'ph', 'sL', 'sR', 'gyro', 'lidar', 'cd'}
                                      for r in ticks))
        status = loop.status()
        self.assertTrue(status['calib_allowed'])
        self.assertFalse(status['row_motor'])
        self.assertEqual(status['calib']['end_reason'], 'calib_complete')
        self.assertTrue(status['calib_options']['arc_sets'])
        # the JSONL adapter reads the same run back
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'row_executor_test_1_001.jsonl')
            with open(path, 'w', encoding='utf-8', newline='\n') as fh:
                for record in records:
                    fh.write(json.dumps(record) + '\n')
                fh.write('{"half a line\n\n')
            counts = {}
            runs = CA.load_runs([path], counts)
            self.assertEqual(counts['malformed'], 1)
            self.assertEqual(len(runs), 1)
            s = CA.summarize(runs[0])
            self.assertEqual(s['end_reason'], 'calib_complete')
            self.assertEqual(len(s['recomputed']), 4)
            self.assertTrue(s['cross_check']['ok'], s['cross_check'])
            frag = CA.config_fragment([s])
            self.assertAlmostEqual(frag['fragment']['measured_yaw_at_max_diff_deg_s'], 3., delta=.15)
            segs = CA.c5_segments(runs[0])
            self.assertEqual(len(segs), 4)
            self.assertEqual({x['side'] for x in segs}, {'left', 'right'})
            self.assertTrue(all(abs(x['gain_dps_per_pct'] - .15) < .01 for x in segs))
            for key in ('rid', 'label', 'req', 'diff', 't_rel', 'seconds', 'samples', 'method', 'yaw_lidar_dps',
                        'yaw_imu_dps', 'yaw_net_dps', 'yaw_imu_net_dps', 'speed_m_s', 'gain_dps_per_pct', 'radius_m',
                        'side'):
                self.assertIn(key, segs[0])
            cfg_path = os.path.join(tmp, 'config.json')
            with open(cfg_path, 'w', encoding='utf-8', newline='\n') as fh:
                fh.write(C.dumps(cfg34()))
            out_cfg = os.path.join(tmp, 'merged.json')
            rc = CA.main([path, '--config', cfg_path, '--out-fragment', os.path.join(tmp, 'frag.json'),
                          '--json', os.path.join(tmp, 's.json')])
            self.assertEqual(rc, 0)
            with open(os.path.join(tmp, 's.json'), 'r', encoding='utf-8') as fh:
                self.assertEqual(len(json.load(fh)['c5_segments']), 4)
            # unified field procedure: calib_analysis no longer writes configs (make_field_config.py does)
            self.assertEqual(CA.main([path, '--config', cfg_path, '--out-config', out_cfg]), 2)
            self.assertFalse(os.path.exists(out_cfg))
            merged, problems = CA.merge_config(cfg34(), CA.fit_fragment(frag, cfg34())['fragment'])
            self.assertIsNone(problems)
            self.assertAlmostEqual(merged.measured_yaw_at_max_diff_deg_s, 3., delta=.15)
            self.assertFalse(merged.motor_output_enabled)

    def test_pause_ends_and_needs_a_new_start(self):
        rig = self.rig(calib_cfg(), static=44, rolling=34)
        loop = rig.loop
        fut = loop.submit('calib_start', calib_kind='rolldown', params={})
        rig.run(1.5)
        self.assertTrue(fut.result()['ok'])
        loop.submit('arm')
        rig.run(.05)
        rig.press('cal-2')
        rig.run(2., hold=True)
        rig.release()
        rig.run(3.)
        self.assertEqual(loop.runner.end_reason, 'calib_hold_released')
        n = len([r for r in rig.transport.requests if r['op'] == 'drive' and (r['left'] or r['right'])])
        rig.press('cal-3')
        rig.run(2., hold=True)
        self.assertEqual(n, len([r for r in rig.transport.requests if r['op'] == 'drive' and (r['left'] or r['right'])]))
        self.assertIsNone(rig.control.lease)

    def test_refusals(self):
        rig = self.rig(calib_cfg(), motor=False)                        # SHADOW: never a calibration
        fut = rig.loop.submit('calib_start', calib_kind='arc', params=dict(arc_set='d10'))
        rig.run(.05)
        self.assertEqual(fut.result()['error'], 'calibration_needs_motor_mode')
        self.assertIsNone(rig.loop._stop_transport)
        self.assertIsNone(rig.loop.status()['calib_options'])           # no calibration card in SHADOW
        self.assertFalse(rig.loop.status()['calib_allowed'])
        self.assertEqual(rig.transport.requests, [])
        from test_executor_loop import motor_config
        rig = self.rig(motor_config())                                  # row output only: no calibration
        fut = rig.loop.submit('calib_start', calib_kind='arc', params=dict(arc_set='d10'))
        rig.run(.05)
        self.assertEqual(fut.result()['error'], 'calibration_output_not_allowed')
        rig = self.rig(calib_cfg())
        fut = rig.loop.submit('calib_start', calib_kind='arc', params={})     # (55, 35) below 40: refused
        rig.run(.05)
        self.assertEqual(fut.result()['error'], 'invalid_calibration')
        fut = rig.loop.submit('calib_start', calib_kind='arc', params=dict(arc_set='d10'))
        rig.run(.05)
        self.assertTrue(fut.result()['ok'])
        fut = rig.loop.submit('calib_start', calib_kind='rolldown', params={})
        rig.run(.05)
        self.assertEqual(fut.result()['error'], 'run_in_progress')


class HttpTest(unittest.TestCase):
    def test_calib_route(self):
        import threading
        import http.client
        from row_executor import operator_http as H

        class FakeLoop:
            def __init__(self):
                self.calls = []

            def request(self, kind, **kw):
                self.calls.append((kind, kw))
                return dict(ok=True, run_id=1)

            def status(self):
                return {}

        loop = FakeLoop()
        token = 'tok-0123456789abcdef'
        server = H.make_server(loop, '127.0.0.1', 0, token)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            host = '127.0.0.1:%d' % server.server_address[1]

            def post(body):
                conn = http.client.HTTPConnection(host, timeout=5)
                data = json.dumps(body).encode()
                conn.request('POST', '/api/calib_start', data, {'Host': host, 'Origin': 'http://' + host,
                                                                 'X-Row-Token': token, 'Content-Type': 'application/json',
                                                                 'Content-Length': str(len(data))})
                r = conn.getresponse()
                return r.status, json.loads(r.read())
            self.assertEqual(post(dict(kind='arc', params=dict(arc_set='d10')))[0], 200)
            self.assertEqual(loop.calls[-1], ('calib_start', dict(calib_kind='arc', params=dict(arc_set='d10'))))
            self.assertEqual(post(dict(kind='arc', turbo=1))[0], 400)
            self.assertEqual(post(dict(kind='arc', params=[1]))[0], 400)
            self.assertEqual(len(loop.calls), 1)
        finally:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    unittest.main()
