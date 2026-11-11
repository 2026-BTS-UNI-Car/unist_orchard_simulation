"""Regression tests for the day-1 review fixes in core / config / guard ("Fix A").

  S1 / F1  a terminal decision pending in the controller survives any pause (ENDED, never PAUSED)
  S2 / F7  corridor lost: PAUSED, a NEW press after standstill resumes, repeat limit 3 per 5 m
  F4       Backstop counts distance only while the estimator reports motion; expected rest position
  S4       bench interlock: motion / front clearance end a bench run
  S5       body mask bounded by the footprint (+ 0.05 m), forward sector by the front edge
  S6       motor output: geometry guard on, stop model / side / freshness not looser, bind_host literal
  F2       rolling inner-wheel threshold (static rule at standstill, rolling rule once moving)

The Runner tests drive the REAL vendored RowController with a kinematic fake estimator (the vehicle moves
while the Runner's last output is non-zero, coasts after a 0/0, then stands still), as the review probes did.
Each test here fails on the code before the fixes (missing behaviour or missing API).
"""
import math
import types
import unittest

import rowexec_testlib as L
from row_executor import config as C
from row_executor import core
from row_executor.config import ConfigError, ExecutorConfig
from row_executor.core import ACTIVE, ENDED, PAUSED, Runner
from row_executor import guard as G
from row_executor.guard import Backstop, OutputGuard

DT = .05


def motor_cfg(**over):
    """Feasible motor configuration (example numbers, not field values; same as test_config.feasible)."""
    # steering GO rule (field decision rules, 2026-10-22): row motor output needs max_differential 10-20 and a measured
    # authority >= 4.0 deg/s; base 60 keeps the inner wheel at breakaway 40 + margin 10 with differential 10
    base = dict(motor_output_enabled=True, base_duty_pct=60, reduce_duty_pct=60, max_differential_pct=10,
                duty_cap_pct=70, diff_cap_pct=20, breakaway_duty_pct=40, inner_margin_pct=10,
                v_measured_m_s=.48, measured_yaw_at_max_diff_deg_s=4.2, max_yaw_rate_deg_s=2.5,
                measured_stop_distance_m=.25)       # C7 (day-2 N8: required for row motor output)
    base.update(over)
    return ExecutorConfig(**base)


class Kinematic:
    """Fake estimator with simple kinematics (see the module docstring).

    front_at / end_at: front obstacle / row end at this distance from the Start; gaps: ((from_m, to_m), ...) where
    the corridor band is invalid; stuck: the vehicle never moves (drive_no_response); side_left_at: from this
    distance a return 1 cm beside the footprint on the left (side_blocked); phantom_m_s: distance_m growth while
    standing still (scan-matching jitter, review F4).
    """

    def __init__(self, box, speed=.45, coast_s=.45, front_at=None, end_at=None, gaps=(), stuck=False,
                 side_left_at=None, phantom_m_s=0.):
        self.box, self.v, self.coast_s = box, speed, coast_s
        self.front_at, self.end_at, self.gaps, self.stuck = front_at, end_at, tuple(gaps), stuck
        self.side_left_at, self.phantom = side_left_at, phantom_m_s
        self.t_last = self.zero_since = self.still_since = None
        self.dist = self.reported = 0.
        self.updates = self.scans = 0
        self.lidar_age, self.imu_age = .05, .02
        self.heading_error = 0.

    def on_imu(self, *a):
        pass

    def on_scan(self, *a):
        pass

    def corridor_valid(self):
        return not any(a <= self.dist < b for a, b in self.gaps)

    def snapshot(self, now):
        if self.t_last is None:
            self.t_last, self.zero_since, self.still_since = now, now - 10., now - 10.
        dt, self.t_last = now - self.t_last, now
        commanded = self.box['runner']._last_output != (0, 0)
        if commanded:
            self.zero_since = None
        elif self.zero_since is None:
            self.zero_since = now
        moving = not self.stuck and (commanded or now - self.zero_since < self.coast_s)
        speed = self.v if moving else 0.
        if moving:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now
        self.dist += speed * dt
        self.reported += speed * dt + (0. if moving else self.phantom * dt)
        valid = self.corridor_valid()
        self.scans += 1
        self.updates += valid
        side_left = .01 if self.side_left_at is not None and self.dist >= self.side_left_at else 1.2
        return dict(lidar_age_s=self.lidar_age, imu_age_s=self.imu_age, still=not moving,
                    still_s=0. if moving else now - self.still_since, speed=speed, odom_valid=True,
                    distance_m=self.reported, yaw_rate=0., corridor_valid=valid, corridor_age_s=.05,
                    corridor_reason='tracking' if valid else 'corridor_lost', heading_error=self.heading_error,
                    lateral_offset=0., corridor_width=3.5, corridor_updates=self.updates, scan_count=self.scans,
                    front_clearance_m=30. if self.front_at is None else self.front_at - self.dist,
                    row_end_ahead_m=None if self.end_at is None else self.end_at - self.dist,
                    side_clearance_left_m=side_left, side_clearance_right_m=1.2,
                    side_nearest_x_left_m=.3 if side_left < 1. else None, side_nearest_x_right_m=None,
                    rotation_centre_x_m=0., rotation_centre_sd_m=0.)


class Rig:
    """Real Runner + real vendored controller + Kinematic estimator on a fake 20 Hz clock."""

    def __init__(self, cfg=None, row_len_m=60., **world):
        self.box = {}
        self.est = None

        def factory():
            self.est = Kinematic(self.box, **world)
            return self.est
        self.runner = Runner(cfg or ExecutorConfig(), estimator_factory=factory)
        self.box['runner'] = self.runner
        self.t = 1000.
        self.runner.start_run(self.t, row_len_m)
        self.log = []

    def run(self, seconds, hold):
        out = []
        for _ in range(int(round(seconds / DT))):
            self.t += DT
            out.append(self.runner.tick(self.t, hold))
        self.log += out
        return out

    def hold_until(self, predicate, max_s=30.):
        """Keep holding until predicate(runner) (checked after every tick); returns the decisions."""
        out = []
        for _ in range(int(round(max_s / DT))):
            out += self.run(DT, True)
            if predicate(self.runner):
                return out
        raise AssertionError('condition not reached within %g s' % max_s)

    def start_driving(self):
        self.run(1.5, False)                     # inputs fresh >= 1 s, vehicle at rest
        out = self.run(.3, True)
        assert self.runner.state == ACTIVE, self.runner.reason
        return out


def ctl_state(runner):
    return None if runner.controller is None else runner.controller.state


def lr(decision):
    return decision.left, decision.right


def nonzero(decisions):
    return [(d.left, d.right) for d in decisions if d.left or d.right]


# --- S1 / F1 -------------------------------------------------------------------------------------------
class TerminalPendingSurvivesPauseTest(unittest.TestCase):
    def assert_terminal_kept(self, rig, reason, note):
        r = rig.runner
        self.assertEqual((r.state, r.end_reason, r.reason), (ENDED, reason, reason))
        self.assertEqual(r.end_note, note)
        last = rig.log[-1]
        self.assertTrue(last.terminal)
        self.assertEqual((last.state, last.reason, last.left, last.right), (ENDED, reason, 0, 0))
        self.assertEqual(last.summary.get('end_note'), note)
        self.assertEqual(r.status()['end_note'], note)
        # the vehicle stops; a new press (or keeping it) never drives again
        out = rig.run(1.5, False) + rig.run(2., True) + rig.run(.2, False) + rig.run(2., True)
        self.assertEqual(nonzero(out), [])
        self.assertTrue(all(d.state == ENDED and d.reason == reason for d in out))

    def test_row_end_candidate_then_hold_released(self):
        rig = Rig(end_at=2.)
        rig.start_driving()
        rig.hold_until(lambda r: ctl_state(r) == 'STOP_REQUESTED')
        self.assertEqual(rig.runner.controller.pending, ('hold', 'row_end_candidate'))
        rig.run(.1, True)
        rig.run(DT, False)                                    # operator lets go while the vehicle fades
        self.assert_terminal_kept(rig, 'row_end_candidate', 'terminal_stop_pending_at_hold_released')

    def test_front_blocked_then_stale_lidar(self):
        rig = Rig(front_at=2.)
        rig.start_driving()
        rig.hold_until(lambda r: ctl_state(r) == 'STOP_REQUESTED')
        self.assertEqual(rig.runner.controller.pending, ('front', 'front_blocked'))
        rig.est.lidar_age = .6                                # LiDAR goes stale during the stop
        rig.run(DT, True)
        rig.est.lidar_age = .05
        self.assert_terminal_kept(rig, 'front_blocked', 'terminal_stop_pending_at_stale_lidar')

    def test_drive_no_response_then_loop_stall(self):
        rig = Rig(stuck=True)
        rig.start_driving()
        rig.hold_until(lambda r: ctl_state(r) == 'STOP_REQUESTED', max_s=6.)
        self.assertEqual(rig.runner.controller.pending, ('hold', 'drive_no_response'))
        rig.t += .4                                           # main loop stalled 0.4 s > stall_guard_s
        rig.log.append(rig.runner.tick(rig.t, True))
        self.assert_terminal_kept(rig, 'drive_no_response', 'terminal_stop_pending_at_stall_guard')

    def test_side_blocked_then_watchdog_pause(self):
        rig = Rig(side_left_at=1.)
        rig.start_driving()
        rig.hold_until(lambda r: ctl_state(r) == 'STOP_REQUESTED')
        self.assertEqual(rig.runner.controller.pending, ('hold', 'side_blocked'))
        rig.runner.pause(rig.t, 'stall_guard')                # the watchdog thread's pause
        rig.log.append(rig.runner.tick(rig.t + DT, True))
        rig.t += DT
        self.assert_terminal_kept(rig, 'side_blocked', 'terminal_stop_pending_at_stall_guard')

    def test_uninterrupted_terminal_stop_unchanged(self):
        rig = Rig(end_at=2.)
        rig.start_driving()
        rig.hold_until(lambda r: r.state == ENDED)
        self.assertEqual((rig.runner.end_reason, rig.runner.end_note), ('row_end_candidate', None))

    def test_guard_stops_stay_terminal(self):
        rig = Rig()
        rig.start_driving()
        rig.est.heading_error = math.radians(30.)            # geometry guard (C1 gate P3)
        rig.run(DT, True)
        self.assertEqual((rig.runner.state, rig.runner.end_reason), (ENDED, 'guard_heading'))
        rig.run(.2, False)
        rig.runner.pause(rig.t, 'stall_guard')
        rig.est.lidar_age = .6
        rig.run(.2, True)
        rig.est.lidar_age = .05
        out = rig.run(1.5, False) + rig.run(1., True)
        self.assertEqual(nonzero(out), [])
        self.assertEqual((rig.runner.state, rig.runner.end_reason), (ENDED, 'guard_heading'))

    def test_pending_other_than_wait_corridor_is_terminal(self):
        runner = Runner(ExecutorConfig(), estimator_factory=lambda: Kinematic({'runner': None}),
                        controller_factory=lambda: None)
        runner.controller = types.SimpleNamespace(state='STOP_REQUESTED', pending=None)
        self.assertEqual(runner._pending_terminal(), 'stop_without_decision')
        runner.controller = types.SimpleNamespace(state='STOP_REQUESTED', pending=('turn', 'large_heading_error'))
        self.assertEqual(runner._pending_terminal(), 'large_heading_error')
        runner.controller = types.SimpleNamespace(state='STOP_REQUESTED', pending=('wait_corridor', 'corridor_lost'))
        self.assertIsNone(runner._pending_terminal())
        runner.controller = types.SimpleNamespace(state='FOLLOW', pending=('hold', 'row_end_candidate'))
        self.assertIsNone(runner._pending_terminal())


# --- S2 / F7 -------------------------------------------------------------------------------------------
class CorridorLostTest(unittest.TestCase):
    def test_holding_through_the_recovery_never_restarts(self):
        rig = Rig(gaps=((1., 1.1),))
        rig.start_driving()
        out = rig.hold_until(lambda r: r.state != ACTIVE)
        d = out[-1]
        self.assertEqual((d.state, d.reason, d.left, d.right), (PAUSED, 'corridor_lost', 0, 0))
        self.assertEqual(d.summary['controller_reason'], 'corridor_lost')
        self.assertIsNone(rig.runner.controller)
        out = rig.run(6., True)                               # corridor back, vehicle at rest, button held
        self.assertTrue(rig.est.corridor_valid())
        self.assertEqual(nonzero(out), [])
        self.assertTrue(all(x.state == PAUSED for x in out))
        self.assertEqual((rig.runner.resumes, rig.runner.corridor_lost_pauses), (1, 1))

    def test_new_press_after_standstill_resumes(self):
        rig = Rig(gaps=((1., 1.1),))
        rig.start_driving()
        rig.hold_until(lambda r: r.state != ACTIVE)
        rig.run(1.5, False)                                   # vehicle at rest
        out = rig.run(1., True)
        self.assertEqual(rig.runner.state, ACTIVE)
        self.assertTrue(nonzero(out))
        self.assertEqual(rig.runner.controllers_built, 2)

    def test_press_while_coasting_is_ignored(self):
        rig = Rig(gaps=((1., 1.1),))
        rig.start_driving()
        rig.hold_until(lambda r: r.state != ACTIVE)
        rig.run(DT, False)
        out = rig.run(DT, True)                               # pressed again at once: vehicle still coasting
        self.assertEqual((rig.runner.state, rig.runner.reason), (PAUSED, 'press_ignored_not_standstill'))
        out += rig.run(3., True)                              # keeps holding while it comes to rest: no restart
        self.assertEqual(nonzero(out), [])
        self.assertEqual(rig.runner.state, PAUSED)
        rig.run(DT, False)
        rig.run(.5, True)                                     # a new press now that it stands
        self.assertEqual(rig.runner.state, ACTIVE)

    def test_fourth_loss_within_5_m_ends_the_run(self):
        rig = Rig(gaps=((1., 1.1), (2., 2.1), (3., 3.1), (4., 4.1)))
        rig.start_driving()
        for k in range(3):
            rig.hold_until(lambda r: r.state != ACTIVE)
            self.assertEqual((rig.runner.state, rig.runner.reason), (PAUSED, 'corridor_lost'))
            rig.run(1.5, False)
            rig.run(.3, True)
            self.assertEqual(rig.runner.state, ACTIVE, k)
        rig.hold_until(lambda r: r.state != ACTIVE)
        r = rig.runner
        self.assertEqual((r.state, r.end_reason, r.corridor_lost_pauses), (ENDED, 'corridor_lost_repeat_limit', 4))
        self.assertIn('4 corridor losses within 5 m', r.end_note)

    def test_losses_further_apart_than_the_window_do_not_end(self):
        rig = Rig(gaps=((1., 1.1), (4., 4.1), (7., 7.1), (10., 10.1)))
        rig.start_driving()
        for k in range(4):
            rig.hold_until(lambda r: r.state != ACTIVE)
            self.assertEqual((rig.runner.state, rig.runner.reason), (PAUSED, 'corridor_lost'), k)
            rig.run(1.5, False)
            rig.run(.3, True)
        self.assertEqual(rig.runner.state, ACTIVE)

    def test_any_other_reacquire_needs_a_press(self):
        class Reacquire:
            state, reason, pending = 'REACQUIRE', 'fresh_input_and_still', None

            def step(self, now, s):
                return types.SimpleNamespace(left=0, right=0, state='REACQUIRE', reason='fresh_input_and_still')
        box = {}
        runner = Runner(ExecutorConfig(), estimator_factory=lambda: Kinematic(box), controller_factory=Reacquire)
        box['runner'] = runner
        runner.start_run(10., 30.)
        t = 10.
        for k in range(40):
            t += DT
            runner.tick(t, k >= 25)
        self.assertEqual((runner.state, runner.reason), (PAUSED, 'reacquire_needs_press'))


# --- F4 ------------------------------------------------------------------------------------------------
class BackstopMotionDistanceTest(unittest.TestCase):
    def test_sensor_reports_motion(self):
        self.assertFalse(G.sensor_reports_motion(dict(still=True, speed=0.)))
        self.assertFalse(G.sensor_reports_motion(dict(still=True, speed=-.04)))
        self.assertTrue(G.sensor_reports_motion(dict(still=True, speed=.06)))
        self.assertTrue(G.sensor_reports_motion(dict(still=False, speed=0.)))
        self.assertTrue(G.sensor_reports_motion(dict(speed=0.)))                 # no stillness report
        self.assertTrue(G.sensor_reports_motion(dict(still=True, speed=float('nan'))))
        self.assertTrue(G.sensor_reports_motion(dict(still=1, speed=0.)))         # not a bool True

    def test_still_increments_are_not_counted(self):
        b = Backstop()
        b.start(0., 0., 10., None)                                           # limit 8 m
        t, d = 0., 0.
        for _ in range(200):                                                 # 10 s standing: 0.26 m of jitter
            t += DT
            d += .026 * DT
            self.assertIsNone(b.check(t, d, False, False))
        self.assertAlmostEqual(b.travelled_m, 0., places=9)
        self.assertAlmostEqual(b.summary()['still_excluded_m'], .26, places=3)
        while True:                                                          # then 0.45 m/s
            t += DT
            d += .45 * DT
            reason = b.check(t, d, True, True)
            if reason:
                break
        self.assertEqual(reason, Backstop.DISTANCE)
        self.assertAlmostEqual(b.travelled_m, 8., delta=.0226)
        self.assertAlmostEqual(d, 8.26, delta=.03)                           # raw path length incl. jitter

    def test_motion_odometer(self):
        o = G.MotionOdometer(5.)
        self.assertTrue(o.update(5.5, True))
        self.assertTrue(o.update(5.6, False))
        self.assertTrue(o.update(5.595, True))                               # 5 mm back: tolerated
        self.assertFalse(o.update(5.5, True))                                # > 1 cm back: invalid
        self.assertFalse(o.update(float('inf'), True))
        self.assertAlmostEqual(o.counted_m, .5)
        self.assertAlmostEqual(o.excluded_m, .1)

    def test_runner_does_not_count_jitter_while_waiting(self):
        rig = Rig(cfg=ExecutorConfig(v_measured_m_s=.45), row_len_m=10., phantom_m_s=.03)
        rig.run(60., False)                                                  # standing at the row entry
        self.assertEqual(rig.runner.state, PAUSED)
        backstop = rig.runner.backstop
        self.assertLess(backstop.travelled_m, 1e-9)
        self.assertAlmostEqual(backstop.odometer.excluded_m, 1.8, delta=.01)
        rig.run(DT, False)
        rig.run(.3, True)
        out = rig.hold_until(lambda r: r.state == ENDED, max_s=30.)
        self.assertEqual(rig.runner.end_reason, 'row_distance_backstop')
        self.assertAlmostEqual(rig.est.dist, 8., delta=.03)                  # true distance at the end decision
        summary = out[-1].summary['backstop']
        self.assertAlmostEqual(summary['expected_rest_m'], 8. + rig.runner.stop_model.stopping_distance(.45),
                               places=3)

    def test_expected_rest_position_documented_value(self):
        runner = Runner(ExecutorConfig(v_measured_m_s=.45), estimator_factory=lambda: Kinematic({'runner': None}))
        runner.start_run(0., 30.)
        rest = runner.backstop_summary()['expected_rest_m']
        self.assertAlmostEqual(rest, 28. + .45 * .4 + .45 ** 2 / (2 * 1.07), places=3)   # L - 2 m + 0.275 m
        runner = Runner(ExecutorConfig(), estimator_factory=lambda: Kinematic({'runner': None}))
        runner.start_run(0., 30.)
        self.assertIsNone(runner.backstop_summary()['expected_rest_m'])


# --- S4 ------------------------------------------------------------------------------------------------
class BenchEstimator:
    def __init__(self):
        self.s = dict(lidar_age_s=.05, imu_age_s=.02, still=True, still_s=5., speed=0., odom_valid=True,
                      distance_m=0., yaw_rate=0., corridor_valid=False, corridor_reason='no_band',
                      front_clearance_m=5.)

    def on_imu(self, *a):
        pass

    def on_scan(self, *a):
        pass

    def snapshot(self, now):
        return dict(self.s)


class BenchInterlockTest(unittest.TestCase):
    def rig(self):
        est = BenchEstimator()
        cfg = ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK, bench_pattern=((10., 40, 40),))
        runner = Runner(cfg, estimator_factory=lambda: est)
        runner.start_bench(0.)
        rig = types.SimpleNamespace(runner=runner, est=est, t=0.)

        def run(seconds, hold):
            out = []
            for _ in range(int(round(seconds / DT))):
                rig.t += DT
                out.append(runner.tick(rig.t, hold))
            return out
        rig.run = run
        run(1.2, False)
        out = run(.3, True)
        self.assertEqual((runner.state, out[-1].left), (ACTIVE, 40))
        return rig

    def test_speed_ends_the_bench_run(self):
        rig = self.rig()
        rig.est.s.update(still=False, speed=.15)
        d = rig.run(DT, True)[-1]
        self.assertEqual((d.state, d.reason, d.left, d.right, d.terminal), (ENDED, 'bench_motion_detected', 0, 0, True))
        self.assertEqual(nonzero(rig.run(1., True)), [])

    def test_counted_distance_ends_the_bench_run(self):
        rig = self.rig()
        rig.est.s.update(still=False, speed=.08)
        for k in range(10):
            rig.est.s['distance_m'] += .03
            d = rig.run(DT, True)[-1]
            if d.state == ENDED:
                break
        self.assertEqual(d.reason, 'bench_motion_detected')
        self.assertGreater(rig.est.s['distance_m'], .2)

    def test_still_jitter_does_not_end_the_bench_run(self):
        rig = self.rig()
        for _ in range(100):                                  # 5 s, 0.5 m of path length while 'still'
            rig.est.s['distance_m'] += .005
            rig.run(DT, True)
        self.assertEqual(rig.runner.state, ACTIVE)

    def test_motion_while_paused_ends_the_bench_run(self):
        rig = self.rig()
        rig.run(.2, False)
        self.assertEqual(rig.runner.state, PAUSED)
        rig.est.s.update(still=False, speed=-.2)              # pushed backwards
        rig.run(DT, False)
        self.assertEqual((rig.runner.state, rig.runner.end_reason), (ENDED, 'bench_motion_detected'))

    def test_front_clearance_below_the_stop_model_ends_the_bench_run(self):
        rig = self.rig()
        need = rig.runner.stop_model.required_front(0.)
        self.assertAlmostEqual(need, .35)
        rig.est.s['front_clearance_m'] = need + .01
        rig.run(.2, True)
        self.assertEqual(rig.runner.state, ACTIVE)
        rig.est.s['front_clearance_m'] = .15                  # e.g. LiDAR 0.15 m from an obstacle (review S4 probe)
        rig.run(DT, True)
        self.assertEqual((rig.runner.state, rig.runner.end_reason), (ENDED, 'bench_front_blocked'))

    def test_bench_is_impossible_in_the_field_profile(self):
        with self.assertRaisesRegex(ConfigError, 'bench'):
            ExecutorConfig(profile='field', bench_pattern=((1., 40, 40),), bench_ack=C.BENCH_ACK).validate()
        runner = Runner(ExecutorConfig(), estimator_factory=BenchEstimator)
        with self.assertRaisesRegex(RuntimeError, 'bench_pattern_requires_bench_profile'):
            runner.start_bench(0.)


# --- S5 ------------------------------------------------------------------------------------------------
def beam(bearing_deg):
    return L.body_bearing_to_raw_index(math.radians(bearing_deg))


class BodyMaskFootprintTest(unittest.TestCase):
    def test_footprint_exit_range(self):
        # LiDAR 0.18 m ahead of the centre of a 1.22 x 0.70 m footprint
        self.assertAlmostEqual(C.footprint_exit_range(0., .18, 1.22, .7), .43)
        self.assertAlmostEqual(C.footprint_exit_range(math.pi, .18, 1.22, .7), .79)
        self.assertAlmostEqual(C.footprint_exit_range(math.pi / 2, .18, 1.22, .7), .35)
        self.assertAlmostEqual(C.footprint_exit_range(math.radians(30.), .18, 1.22, .7), .43 / math.cos(math.radians(30.)))
        self.assertEqual(C.footprint_exit_range(0., 1., 1.22, .7), 0.)          # LiDAR ahead of the footprint

    def test_limits_keep_every_masked_return_within_5_cm_of_the_footprint(self):
        cfg = ExecutorConfig()
        boundary, forward, bearings = C.body_mask_beam_limits(cfg)
        offset = math.radians(cfg.lidar_yaw_offset_deg)
        for i in range(0, cfg.beam_grid_count, 7):
            a = cfg.beam_grid_angle_min_rad + i * cfg.beam_grid_increment_rad + offset
            r = min(boundary[i], forward[i])
            x, y = cfg.lidar_x_in_base_m + r * math.cos(a), r * math.sin(a)       # estimator base frame
            outside = max(abs(x) - cfg.footprint_length_m / 2, abs(y) - cfg.footprint_width_m / 2)
            self.assertLessEqual(outside, .05 + 1e-9, (i, bearings[i]))

    def test_review_probe_mask_is_refused(self):
        # S5 probe: +-6.7 deg around the travel axis masked to 1.2 m hid a trunk 0.75 m ahead
        with self.assertRaisesRegex(ConfigError, 'forward sector'):
            # bearing 0 is raw -179.08 deg, next to the raw array wrap: the sector needs two segments
            ExecutorConfig(body_mask=((beam(-6.7), 3239, 1.2), (0, beam(6.7), 1.2))).validate()
        with self.assertRaisesRegex(ConfigError, 'footprint boundary'):
            ExecutorConfig(body_mask=((0, 3239, .5),)).validate()

    def test_bounds_per_direction(self):
        cases = ((0., .48, .49), (90., .40, .41), (-90., .40, .41), (180., .84, .85),
                 (30., .43 / math.cos(math.radians(30.)) + .05, .58))
        for bearing, ok, bad in cases:
            with self.subTest(bearing=bearing):
                i = beam(bearing)
                ExecutorConfig(body_mask=((i, i, round(ok - .001, 3)),)).validate()
                with self.assertRaises(ConfigError):
                    ExecutorConfig(body_mask=((i, i, bad),)).validate()

    def test_forward_sector_bound_uses_the_front_edge(self):
        # with the LiDAR 0.4 m behind the centre the front edge is 1.01 m ahead; a wide footprint keeps the side
        # edge out of the way, so only the front-edge rule bounds a forward beam
        cfg = dict(lidar_x_in_base_m=-.4, footprint_width_m=2.)
        i = beam(20.)
        front = 1.01 / math.cos(math.radians(20.)) + .05
        ExecutorConfig(body_mask=((i, i, 1.1),), body_mask_limit_m=2., **cfg).validate()
        self.assertLess(front, 1.2)
        with self.assertRaisesRegex(ConfigError, 'forward sector'):
            ExecutorConfig(body_mask=((i, i, round(front + .01, 3)),), body_mask_limit_m=2., **cfg).validate()


# --- S6 ------------------------------------------------------------------------------------------------
class MotorSafetySettingsTest(unittest.TestCase):
    def test_baseline_motor_config_is_ok(self):
        self.assertEqual(motor_cfg().validate().feasibility()['status'], 'ok')

    def test_geometry_guard_required_and_bounded(self):
        for key, value in (('guard_heading_deg', None), ('guard_heading_deg', 35.), ('guard_heading_deg', 8.),
                           ('guard_lateral_margin_m', None), ('guard_lateral_margin_m', -.05),
                           ('guard_lateral_margin_m', .4)):
            with self.subTest(key=key, value=value), self.assertRaisesRegex(ConfigError, key):
                motor_cfg(**{key: value}).validate()
        motor_cfg(guard_heading_deg=10., guard_lateral_margin_m=.3).validate()
        ExecutorConfig(guard_heading_deg=None, guard_lateral_margin_m=None).validate()      # shadow: allowed

    def test_stop_model_not_looser_without_c7(self):
        for name, value in (('front_margin_m', .2), ('stop_decel_m_s2', 5.), ('reaction_s', 0.),
                            ('stop_onset_s', 0.)):
            with self.subTest(name=name), self.assertRaisesRegex(ConfigError, 'needs measured_stop_distance_m'):
                motor_cfg(controller_overrides={name: value}, measured_stop_distance_m=None).validate()
        motor_cfg(controller_overrides=dict(front_margin_m=.5, stop_decel_m_s2=.8, reaction_s=.3)).validate()
        ExecutorConfig(controller_overrides={'front_margin_m': .2}).validate()             # shadow: allowed

    def test_stop_model_with_c7(self):
        # v 0.48 m/s: default model 0.48 * 0.4 + 0.48^2 / 2.14 = 0.300 m
        model = C.stopping_distance_model(.48)
        self.assertAlmostEqual(model, .48 * .4 + .48 ** 2 / 2.14)
        motor_cfg(measured_stop_distance_m=.29).validate()
        with self.assertRaisesRegex(ConfigError, 'shorter than the measured C7'):
            motor_cfg(measured_stop_distance_m=.35).validate()
        # a looser parameter is fine when the configured model still covers the measurement
        motor_cfg(measured_stop_distance_m=.2, controller_overrides={'stop_decel_m_s2': 2.}).validate()
        with self.assertRaisesRegex(ConfigError, 'shorter than the measured C7'):
            motor_cfg(measured_stop_distance_m=.29, controller_overrides={'stop_decel_m_s2': 2.}).validate()
        # a smaller front margin needs a model that exceeds the measurement by the difference
        with self.assertRaisesRegex(ConfigError, 'front_margin_m'):
            motor_cfg(measured_stop_distance_m=.29, controller_overrides={'front_margin_m': .3}).validate()
        motor_cfg(measured_stop_distance_m=.24, controller_overrides={'front_margin_m': .3}).validate()

    def test_side_and_freshness_not_looser(self):
        for over in (dict(controller_overrides={'side_hard_margin_m': .02}),
                     dict(controller_overrides={'side_soft_margin_m': .1, 'side_soft_exit_m': .2}),
                     dict(controller_overrides={'side_soft_exit_m': .18}),
                     dict(controller_overrides={'side_confirm_scans': 3}),
                     dict(lidar_max_age_s=.5), dict(imu_max_age_s=.45)):
            with self.subTest(over=over), self.assertRaisesRegex(ConfigError, 'looser than the default'):
                motor_cfg(**over).validate()
        motor_cfg(controller_overrides={'side_hard_margin_m': .05, 'side_confirm_scans': 1},
                  lidar_max_age_s=.3, imu_max_age_s=.3).validate()
        ExecutorConfig(lidar_max_age_s=1.).validate()                                        # shadow: allowed

    def test_defaults_match_the_vendored_controller(self):
        _, controllers = core.load_vendor()
        vendored = controllers.ControlConfig()
        for name, value in list(C.STOP_MODEL_DEFAULTS.items()) + list(C.SIDE_RULE_DEFAULTS.items()):
            self.assertEqual(getattr(vendored, name), value, name)
        model = controllers.RowController(vendored)
        for v in (0., .3, .48, .9):
            self.assertAlmostEqual(C.stopping_distance_model(v), model.stopping_distance(v))

    def test_bind_host(self):
        for host in ('', ' ', '0', '0.0.0.0', '::', '::ffff:0.0.0.0', '0x0', 'localhost', 'jetson.local',
                     ' 10.77.0.1', '224.0.0.1', '255.255.255.255', 'fe80::1', '010.77.0.1'):
            with self.subTest(host=host), self.assertRaisesRegex(ConfigError, 'bind_host'):
                ExecutorConfig(bind_host=host).validate()
        for host in ('', '0', '0.0.0.0', '::'):
            with self.subTest(host=host), self.assertRaisesRegex(ConfigError, 'wildcard'):
                ExecutorConfig(bind_host=host).validate()
        for host in ('10.77.0.1', '127.0.0.1', '192.168.219.50'):
            ExecutorConfig(bind_host=host).validate()


# --- F2 ------------------------------------------------------------------------------------------------
class RollingThresholdTest(unittest.TestCase):
    def rolling_cfg(self, **over):
        base = dict(base_duty_pct=55, reduce_duty_pct=55, max_differential_pct=20, breakaway_duty_pct=40,
                    rolling_breakaway_duty_pct=22, inner_margin_pct=10)
        base.update(over)
        return ExecutorConfig(**base)

    def test_feasibility(self):
        cfg = self.rolling_cfg().validate()                  # 35 >= 22 + 10 moving, 55 >= 40 + 10 at a start
        self.assertEqual((cfg.guard_inner_min_pct(), cfg.guard_rolling_inner_min_pct()), (50, 32))
        with self.assertRaisesRegex(ConfigError, 'base-max_differential = 35 < rolling_breakaway\\+margin = 36'):
            self.rolling_cfg(rolling_breakaway_duty_pct=26).validate()
        with self.assertRaisesRegex(ConfigError, 'base_duty_pct 48 < breakaway\\+margin = 50'):
            self.rolling_cfg(base_duty_pct=48, reduce_duty_pct=48, rolling_breakaway_duty_pct=10).validate()
        with self.assertRaisesRegex(ConfigError, 'above breakaway_duty_pct'):
            self.rolling_cfg(rolling_breakaway_duty_pct=41).validate()
        # not measured: the old rule (base - diff >= breakaway + margin) is unchanged
        with self.assertRaisesRegex(ConfigError, 'inner wheel base-max_differential = 35 < breakaway\\+margin = 50'):
            self.rolling_cfg(rolling_breakaway_duty_pct=None).validate()
        self.assertIsNone(ExecutorConfig(rolling_breakaway_duty_pct=20).guard_rolling_inner_min_pct())

    def test_motor_config_with_rolling_threshold(self):
        cfg = motor_cfg(base_duty_pct=55, reduce_duty_pct=55, max_differential_pct=12, duty_cap_pct=67,
                        diff_cap_pct=24, rolling_breakaway_duty_pct=30).validate()
        self.assertEqual(cfg.feasibility()['status'], 'ok')
        with self.assertRaises(ConfigError):
            motor_cfg(base_duty_pct=55, reduce_duty_pct=55, max_differential_pct=12, duty_cap_pct=67,
                      diff_cap_pct=24).validate()

    def test_guard_minimum_by_motion_state(self):
        guard = OutputGuard(75, 40, inner_min_pct=50, rolling_inner_min_pct=32)
        self.assertEqual(guard.apply(35, 75), (0, 0, 'inner_below_breakaway'))
        self.assertEqual(guard.apply(35, 75, rolling=True), (35, 75, None))
        self.assertEqual(guard.apply(31, 70, rolling=True), (0, 0, 'inner_below_breakaway'))
        self.assertEqual(guard.apply(35, 75, rolling=1), (0, 0, 'inner_below_breakaway'))   # only True
        with self.assertRaises(ValueError):
            OutputGuard(75, 40, inner_min_pct=30, rolling_inner_min_pct=32)
        guard = OutputGuard.from_config(self.rolling_cfg())
        self.assertEqual((guard.inner_min_pct, guard.rolling_inner_min_pct), (50, 32))

    def runner_rig(self, cfg, out=(35, 75)):
        est = BenchEstimator()
        est.s.update(corridor_valid=False)

        class Steer:
            state, reason, pending = 'REACQUIRE', 'start', None

            def step(self, now, s):
                self.state = 'FOLLOW'
                return types.SimpleNamespace(left=out[0], right=out[1], state='FOLLOW', reason='continuous_correction')
        runner = Runner(cfg, estimator_factory=lambda: est, controller_factory=Steer)
        runner.start_run(0., 30.)
        rig = types.SimpleNamespace(runner=runner, est=est, t=0.)

        def run(seconds, hold):
            res = []
            for _ in range(int(round(seconds / DT))):
                rig.t += DT
                res.append(runner.tick(rig.t, hold))
            return res
        rig.run = run
        run(1.2, False)
        return rig

    def test_runner_limits_the_differential_until_rolling(self):
        rig = self.runner_rig(self.rolling_cfg())
        d = rig.run(.2, True)[-1]
        self.assertEqual((d.state, d.left, d.right, d.summary['rolling']), (ACTIVE, 50, 60, False))   # mean 55 kept
        rig.est.s.update(still=False, still_s=0., speed=.08)
        self.assertEqual(lr(rig.run(DT, True)[-1]), (50, 60))          # below 0.10 m/s: not yet rolling
        rig.est.s.update(speed=.3)
        d = rig.run(DT, True)[-1]
        self.assertEqual((d.left, d.right, d.summary['rolling']), (35, 75, True))
        rig.est.s.update(speed=.07)                                      # hysteresis: still rolling
        self.assertEqual(lr(rig.run(DT, True)[-1]), (35, 75))
        rig.est.s.update(odom_valid=False)                               # odometry lost: static rule again
        self.assertEqual(lr(rig.run(DT, True)[-1]), (50, 60))
        rig.est.s.update(odom_valid=True, speed=.3)
        rig.run(DT, True)
        rig.est.s.update(still=True, still_s=.5, speed=0.)              # stalled: static rule again
        self.assertEqual(lr(rig.run(DT, True)[-1]), (50, 60))
        self.assertEqual(rig.runner.state, ACTIVE)
        self.assertGreater(rig.runner.inner_limited_ticks, 0)

    def test_new_resume_starts_with_the_static_rule(self):
        rig = self.runner_rig(self.rolling_cfg())
        rig.est.s.update(still=False, still_s=0., speed=.3)
        rig.run(.3, True)
        self.assertEqual(lr(rig.run(DT, True)[-1]), (35, 75))
        rig.run(.2, False)
        rig.est.s.update(still=True, still_s=2., speed=0.)
        rig.run(1.2, False)
        self.assertEqual(lr(rig.run(.2, True)[-1]), (50, 60))

    def test_without_rolling_threshold_nothing_is_limited(self):
        cfg = ExecutorConfig(breakaway_duty_pct=25, inner_margin_pct=10)        # inner >= 35, old behaviour
        rig = self.runner_rig(cfg, out=(34, 54))
        rig.run(.2, True)
        self.assertEqual(rig.runner.end_reason, 'guard_violation:inner_below_breakaway')
        self.assertEqual(rig.runner.inner_limited_ticks, 0)


if __name__ == '__main__':
    unittest.main()
