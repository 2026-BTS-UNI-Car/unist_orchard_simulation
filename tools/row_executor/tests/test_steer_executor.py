"""Front-wheel steering executor (design 8.3 P4, docs/steering/EXECUTOR_STEER_KO.md): config block and GO rules, the
stop reason table, SteerOutputGuard, SteerMonitor, the front_steer Runner (decision (throttle, steer_mrad), no inner
wheel rule, rear-axle rotation centre, pivot values off, no steering change after a pause or an end, steering status
freshness), steer_arm_precheck and SteerBridgeClient on a fake transport.

Synthetic inputs only (static orchard scans, fake bridge replies); the T1 executor-in-the-loop runs are in
test_sim_in_loop_steer.py. No hardware, no ROS, no network. Every number is a test value, not a field value.
"""
import math
import types
import unittest
from dataclasses import replace

import rowexec_testlib as L
from row_executor import config as C
from row_executor import core
from row_executor.bridge_client import (ARMED, DISARMED, LATCHED, BridgeError, SteerBridgeClient,
                                        steer_arm_precheck)
from row_executor.config import ExecutorConfig
from row_executor.core import ACTIVE, ENDED, PAUSED, Runner, SteerMonitor
from row_executor.guard import SteerOutputGuard

SCENE = L.raw_scan(L.orchard_trunks())

def scipy_problem():
    """None when scipy (the estimator's cKDTree / sparse graph) imports, else the reason (DECISIONS R10: the WSL
    Python 3.10 environment has a scipy built against another numpy)."""
    try:
        from scipy.sparse.csgraph import connected_components  # noqa: F401
        from scipy.spatial import cKDTree  # noqa: F401
    except Exception as exc:          # ImportError, or AttributeError from a numpy ABI mismatch
        return 'scipy not usable here (decision R10): %s: %s' % (type(exc).__name__, exc)
    return None


SCIPY_PROBLEM = scipy_problem()
# ExecutorConfig().digest() and the single-row SHADOW digest of the differential executor at 546fa8c (the commit the
# steering branch imported, docs/steering/EXECUTOR_IMPORT_KO.md): an untouched steering block must keep them.
DIGEST_DEFAULT_546FA8C = '700ee41c0188dffa058a2e1ce1f85411aee3a2ca38646d09451de79620df4340'
DIGEST_SINGLE_ROW_546FA8C = '594143cab2ba608bb60f18ac9a0999e0ca427ec03bff832aeaccead54a8e470d'


def steer_config(**over):
    """A SHADOW front_steer single-row config (test numbers)."""
    base = dict(vehicle_profile='front_steer', mode='single_row', require_native_navigation=False,
                max_yaw_rate_deg_s=3.6, steer_limit_left_mrad=409, steer_limit_right_mrad=409,
                steer_rate_limit_deg_s=24., steer_rate_limit_standstill_deg_s=12.)
    base.update(over)
    return ExecutorConfig(**base).validate()


MOTOR = dict(vehicle_profile='front_steer', mode='single_row', require_native_navigation=False,
             motor_output_enabled=True, base_duty_pct=55, max_yaw_rate_deg_s=3.6, breakaway_duty_pct=20,
             v_measured_m_s=.42, max_speed_m_s=.55, measured_stop_distance_m=.2, steer_limit_left_mrad=409,
             steer_limit_right_mrad=409, steer_rate_limit_deg_s=24., steer_rate_limit_standstill_deg_s=12.,
             steer_measured_rate_deg_s=30., steer_measured_rate_standstill_deg_s=15., steer_range_measured=True,
             steer_wheel_soft_margin_deg=3., steer_min_pwm_pct=25., steer_zero_offset_deg=0.,
             steer_servo_deadband_deg=.5, steer_polarity_checked=True, steer_cal_id=1, steer_allow_sim=True,
             lidar_yaw_offset_deg=179.08, lidar_x_in_base_m=.18)


def motor_config(**over):
    base = dict(MOTOR)
    base.update(over)
    return ExecutorConfig(**base).validate()


def config_problems(**over):
    base = dict(MOTOR)
    base.update(over)
    try:
        ExecutorConfig(**base).validate()
    except C.ConfigError as exc:
        return exc.problems
    return []


class StubSteer:
    """A steering controller stand-in: FOLLOW at a throttle with a steering ramp (mrad per step), or a fixed command."""

    def __init__(self, throttle=40, start=0, step=2, state='FOLLOW', reason='stub', hold_after=None):
        self.throttle, self.value, self.step_mrad = throttle, start, step
        self.state, self.reason = 'FOLLOW', 'start'
        self.out_state, self.out_reason = state, reason
        self.pending = None
        self.phase_events = []
        self.snapshots = []
        self.hold_after = hold_after
        self.steps = 0

    def step(self, t, s):
        self.snapshots.append(dict(s))
        self.steps += 1
        if self.hold_after is not None and self.steps > self.hold_after:
            self.state, self.reason = 'HOLD', 'script_complete'
            return types.SimpleNamespace(state='HOLD', reason='script_complete', throttle=0, steer_mrad=None,
                                         left=0, right=0)
        self.value += self.step_mrad
        self.state, self.reason = self.out_state, self.out_reason
        return types.SimpleNamespace(state=self.out_state, reason=self.out_reason, throttle=self.throttle,
                                     steer_mrad=int(self.value), left=self.throttle, right=self.throttle)

    def current_phase(self):
        return 'FOLLOW'

    def telemetry(self):
        return {}


def block(measured=12, age_ms=20, faults=(), calibrated=True):
    return dict(measured_mrad=measured, age_ms=age_ms, faults=list(faults), calibrated=calibrated,
                servo_state='tracking')


def host(arm_state='confirmed', stop_state='none'):
    return dict(arm_state=arm_state, stop_state=stop_state, stale=False, latches={})


def run_static(cfg, controller, before=3., hold_for=3., feed=None, hold=True, estimator_factory=None):
    """A front_steer Runner on a static orchard scan; feed(t) -> (block, host) or None per tick before the tick."""
    holder = {}

    def factory():
        holder['c'] = controller
        return controller

    runner = Runner(cfg, controller_factory=factory, estimator_factory=estimator_factory)
    driver = L.FakeTimeDriver(runner, lambda t: SCENE, t0=2000.)
    if feed is not None:
        runner.steer_required = True
        original = runner.tick

        def tick(now, hold_active):
            item = feed(now)
            if item is not None:
                runner.feed_steering(now, *item)
            return original(now, hold_active)
        runner.tick = tick
    runner.start_run(driver.t, None)
    out = driver.run(before)
    out += driver.run(hold_for, hold=hold)
    return runner, driver, out


# --- config ---------------------------------------------------------------------------------------------------------
class ConfigTest(unittest.TestCase):
    def test_untouched_block_keeps_the_546fa8c_digests_and_dump(self):
        cfg = ExecutorConfig()
        self.assertFalse(set(C.STEER_KEYS) & set(cfg.to_dict()))
        self.assertEqual(cfg.digest(), DIGEST_DEFAULT_546FA8C)
        sr = ExecutorConfig(mode='single_row', require_native_navigation=False, max_yaw_rate_deg_s=3.6).validate()
        self.assertEqual(sr.digest(), DIGEST_SINGLE_ROW_546FA8C)
        self.assertEqual(C.loads(C.dumps(cfg)), cfg)

    def test_touched_block_round_trip_and_digest(self):
        cfg = steer_config()
        data = cfg.to_dict()
        self.assertTrue(set(C.STEER_KEYS) <= set(data))
        self.assertEqual(C.loads(C.dumps(cfg)), cfg)
        self.assertNotEqual(cfg.digest(), DIGEST_SINGLE_ROW_546FA8C)
        self.assertEqual(data['vehicle_profile'], 'front_steer')

    def test_profile_and_static_rules(self):
        self.assertIn('vehicle_profile', ' '.join(config_problems(vehicle_profile='tank')))
        text = ' '.join(config_problems(steer_rate_limit_standstill_deg_s=30.))
        self.assertIn('not faster at standstill', text)
        text = ' '.join(config_problems(steer_status_max_age_s=.05))
        self.assertIn('never be fresh', text)
        text = ' '.join(config_problems(steer_rotation_centre_x_m=-.9))
        self.assertIn('outside the footprint', text)
        self.assertIn('turn_assist_enabled is refused', ' '.join(config_problems(turn_assist_enabled=True)))

    def test_go_rules_shadow_unknown_and_motor_problems(self):
        shadow = C.feasibility(steer_config())
        self.assertEqual(shadow['status'], 'unknown')
        joined = ' '.join(shadow['unknown'])
        for word in ('B5', 'steer_range_measured', 'B4', 'F1', 'deadband', 'B0', 'steer_cal_id'):
            self.assertIn(word, joined)
        self.assertEqual(C.feasibility(motor_config())['status'], 'ok')
        self.assertIn('above 0.8 x the measured', ' '.join(config_problems(steer_measured_rate_deg_s=20.)))
        self.assertIn('below 8.0 deg/s', ' '.join(config_problems(steer_measured_rate_deg_s=7.,
                                                                  steer_rate_limit_deg_s=5.,
                                                                  steer_rate_limit_standstill_deg_s=5.)))
        self.assertIn('inside the mechanical limit', ' '.join(config_problems(steer_wheel_soft_margin_deg=1.)))
        self.assertIn('zero offset', ' '.join(config_problems(steer_zero_offset_deg=2.)))
        self.assertIn('servo deadband', ' '.join(config_problems(steer_servo_deadband_deg=1.5)))
        self.assertIn('steer_polarity_checked', ' '.join(config_problems(steer_polarity_checked=False)))
        self.assertIn('steer_cal_id', ' '.join(config_problems(steer_cal_id=None)))

    def test_no_inner_wheel_or_yaw_authority_rule_for_front_steer(self):
        # the differential inner-wheel / arc-authority values do not matter for front_steer
        a = C.feasibility(motor_config())
        b = C.feasibility(motor_config(inner_margin_pct=50, measured_yaw_at_max_diff_deg_s=None))
        self.assertEqual((a['status'], a['problems']), (b['status'], b['problems']))

    def test_fault_reason_table(self):
        if SCIPY_PROBLEM:
            self.skipTest(SCIPY_PROBLEM)          # core.load_vendor imports the vendored estimator
        cases = {('POLARITY',): 'steering_polarity_fault', ('DRV_STUCK',): 'steering_driver_fault',
                 ('THERMAL',): 'steering_driver_fault', ('STALL',): 'steering_stalled',
                 ('TRACK',): 'steering_not_following', ('steering_not_following',): 'steering_not_following',
                 ('steering_ineffective',): 'steering_ineffective', ('POT_OPEN',): 'steering_feedback_invalid',
                 ('POT_SHORT',): 'steering_feedback_invalid', ('POT_JUMP',): 'steering_feedback_invalid',
                 ('POT_NOISE',): 'steering_feedback_invalid', ('NOCAL',): 'steering_uncalibrated',
                 ('calibration_mismatch',): 'steering_uncalibrated', ('steering_status_stale',): 'steering_feedback_stale',
                 ('status_seq_gap',): 'steering_feedback_stale', ('stop_unconfirmed',): 'stop_unconfirmed',
                 ('TTL',): 'link_ttl_expired', ('WATCHDOG',): 'link_watchdog', ('CRC',): 'link_bad_lines',
                 ('mcu_reset_detected',): 'mcu_reset', ('c_rejected',): 'link_command_rejected',
                 ('hardware_fault',): 'link_hardware_fault', ('mcu_disarmed',): 'mcu_disarmed',
                 ('BOOT',): None, ('SOFTLIMIT',): None, (): None}
        for names, reason in cases.items():
            self.assertEqual(C.steer_fault_reason(names), reason, names)
        # steering faults before link faults, table order inside one table
        self.assertEqual(C.steer_fault_reason(['TTL', 'STALL']), 'steering_stalled')
        self.assertEqual(C.steer_fault_reason(['TRACK', 'POLARITY']), 'steering_polarity_fault')
        module = core.single_row_module(core.load_vendor()[1])
        for reason in C.STEER_STOP_REASONS + C.STEER_LINK_STOP_REASONS:
            category, _, _ = core.single_row_stop_category(module, reason)
            self.assertIn(category, ('CONTROL_AUTHORITY', 'SYSTEM_FAULT'), reason)
        self.assertEqual(core.single_row_stop_category(module, 'stop_unconfirmed')[0], 'SYSTEM_FAULT')
        self.assertEqual(core.single_row_stop_category(module, 'steering_stalled')[0], 'CONTROL_AUTHORITY')


# --- guard ----------------------------------------------------------------------------------------------------------
class SteerOutputGuardTest(unittest.TestCase):
    def guard(self):
        return SteerOutputGuard(60, 409, 400, 24., 12.)

    def test_values(self):
        g = self.guard()
        self.assertEqual(g.check(0., 40, 100), None)
        for throttle, steer, why in ((40., 0, 'non_integer_throttle'), (True, 0, 'non_integer_throttle'),
                                     (-1, 0, 'reverse_command'), (61, 0, 'duty_above_cap'),
                                     (40, 1.5, 'non_integer_steer'), (40, float('nan'), 'non_integer_steer'),
                                     (40, True, 'non_integer_steer'), (40, 410, 'steer_outside_soft_limit'),
                                     (40, -401, 'steer_outside_soft_limit')):
            self.assertEqual(g.check(0., throttle, steer), why, (throttle, steer))
        self.assertIsNone(g.check(0., 40, 409))
        self.assertIsNone(g.check(0., 40, -400))

    def test_rate_moving_and_standstill(self):
        g = self.guard()
        self.assertEqual(g.apply(0., 40, 0), (40, 0, None))
        moving = math.radians(24.) * 1000. * .05            # 20.9 mrad per 50 ms
        self.assertEqual(g.apply(.05, 40, int(moving))[2], None)
        self.assertEqual(g.apply(.1, 40, int(moving) + int(moving) + 3)[2], 'steer_rate_above_limit')
        g = self.guard()
        g.apply(0., 0, 0)
        standstill = math.radians(12.) * 1000. * .05        # 10.5 mrad per 50 ms at throttle 0 after throttle 0
        self.assertEqual(g.check(.05, 0, int(standstill)), None)
        self.assertEqual(g.check(.05, 0, int(standstill) + 2), 'steer_rate_above_limit')
        self.assertEqual(g.check(.05, 40, int(standstill) + 2), None)     # this command moves: the moving rate
        self.assertEqual(g.check(-.01, 0, 0), 'clock_regression')

    def test_violation_keeps_the_previous_steer_and_hold_records(self):
        g = self.guard()
        g.apply(0., 40, 30)
        self.assertEqual(g.apply(.05, 40, 300), (0, 30, 'steer_rate_above_limit'))
        g.hold(.1, 30)
        self.assertEqual((g.last_throttle, g.last_steer), (0, 30))
        self.assertEqual(g.check(.15, 0, 30), None)


# --- monitor ----------------------------------------------------------------------------------------------------------
class SteerMonitorTest(unittest.TestCase):
    def test_freshness_calibration_validity(self):
        m = SteerMonitor(steer_config())
        self.assertEqual(m.check(0.), 'steering_feedback_stale')
        m.feed(10., block(age_ms=40), host())
        self.assertIsNone(m.check(10.1))
        self.assertAlmostEqual(m.age(10.1), .14)
        self.assertEqual(m.check(10.27), 'steering_feedback_stale')          # .3 s
        m.feed(11., block(age_ms=-1), host())
        self.assertEqual(m.check(11.), 'steering_feedback_stale')
        m.feed(12., block(calibrated=False), host())
        self.assertEqual(m.check(12.), 'steering_uncalibrated')
        m.feed(13., block(measured=None), host())
        self.assertEqual(m.check(13.), 'steering_feedback_invalid')
        m.feed(14., block(measured=700), host())                             # beyond 1.5 x 409
        self.assertEqual(m.check(14.), 'steering_feedback_invalid')
        self.assertFalse(m.feed(15., 'not a dict'))

    def test_faults_count_after_the_arm_confirmation(self):
        m = SteerMonitor(steer_config())
        m.feed(1., block(faults=['STALL']), host(arm_state='none'))
        self.assertIsNone(m.fault())
        m.feed(2., block(faults=['STALL']), host())
        self.assertEqual(m.fault(), 'steering_stalled')
        self.assertFalse(m.usable(2.))
        m.feed(3., block(faults=['BOOT', 'SOFTLIMIT']), host())
        self.assertIsNone(m.fault())
        self.assertTrue(m.usable(3.))

    def test_ttl_event_counted_by_the_host_ends_the_run(self):
        # review RS-01: an F TTL line the host counted (steering_host counters mcu_fault_TTL) after the arm confirmation
        # is the TTL event even when no status line carried the TTL bit (a C line cleared it first)
        m = SteerMonitor(steer_config())

        def counted(n, arm_state='confirmed'):
            return dict(host(arm_state=arm_state), counters={} if n is None else dict(mcu_fault_TTL=n, c_sent=5))
        m.feed(1., block(), counted(None, 'pending'))          # before the confirmation: no TTL yet (no key = 0)
        m.feed(2., block(), counted(None))
        self.assertEqual((m.link_events(), m.fault()), ([], None))
        m.feed(3., block(), counted(1))                        # the bit never shows in the block faults
        self.assertEqual(m.link_events(), ['TTL'])
        self.assertEqual(m.fault(), 'link_ttl_expired')
        self.assertEqual(m.summary(3.)['link_events'], ['TTL'])
        # a new arm: the count before its confirmation is the new base
        m.feed(4., block(), counted(1, 'none'))
        m.feed(5., block(), counted(1))
        self.assertEqual((m.link_events(), m.fault()), ([], None))
        # a steering fault still comes first (config table order)
        m.feed(6., block(faults=['STALL']), counted(2))
        self.assertEqual(m.fault(), 'steering_stalled')
        # a host without counters: the status bits only
        m = SteerMonitor(steer_config())
        m.feed(1., block(), host())
        self.assertEqual(m.link_events(), [])

    def test_authority_rule(self):
        cfg = steer_config(steer_authority_s=1.)
        m = SteerMonitor(cfg)
        sat = int(.96 * 409)
        s = dict(corridor_valid=True, lateral_offset=.30)
        self.assertIsNone(m.authority(0., sat, 40, s))
        self.assertIsNone(m.authority(.9, sat, 40, s))
        self.assertEqual(m.authority(1.05, sat, 40, dict(s, lateral_offset=.31)), 'steering_authority_insufficient')
        m = SteerMonitor(cfg)
        m.authority(0., sat, 40, s)
        self.assertIsNone(m.authority(1.05, sat, 40, dict(s, lateral_offset=.2)))   # the error falls: authority there
        m = SteerMonitor(cfg)
        m.authority(0., sat, 40, s)
        self.assertIsNone(m.authority(1.05, 100, 40, s))                            # not saturated
        self.assertIsNone(m.authority(2.5, sat, 0, s))                              # not driving
        self.assertIsNone(m.authority(4., sat, 40, dict(s, corridor_valid=False)))  # no corridor: not evaluated


# --- runner ------------------------------------------------------------------------------------------------------------
@unittest.skipIf(SCIPY_PROBLEM, SCIPY_PROBLEM or '')
class SteerRunnerTest(unittest.TestCase):
    def test_needs_a_steering_controller(self):
        with self.assertRaises(ValueError):
            Runner(steer_config())

    def test_decisions_are_throttle_and_steer_with_equal_duty(self):
        controller = StubSteer(throttle=40, step=2)
        runner, _, out = run_static(steer_config(), controller)
        active = [d for _, d in out if d.state == ACTIVE and d.throttle]
        self.assertTrue(active)
        for d in active:
            self.assertEqual((d.left, d.right), (d.throttle, d.throttle))
            self.assertEqual(type(d.steer_mrad), int)
        self.assertEqual(runner.guard.__class__, SteerOutputGuard)

    def test_snapshot_rear_axle_centre_and_no_pivot_values(self):
        controller = StubSteer()
        cfg = steer_config()
        run_static(cfg, controller)
        self.assertTrue(controller.snapshots)
        for s in controller.snapshots:
            for key in core.PIVOT_SNAPSHOT_KEYS:
                self.assertNotIn(key, s)
            self.assertEqual(s['rotation_centre_x_m'], cfg.steer_rotation_centre_x_m)
            self.assertIn('steer_measured_mrad', s)
            self.assertIn('steer_command_mrad', s)

    def test_pivot_settings_do_not_change_the_results(self):
        # design 6.7: the estimator's differential pivot (the ICR behind the LiDAR) is off for front_steer. Review P4R-09:
        # change the values the estimator really uses (EstimatorConfig pivot_icr_behind_lidar_m, pivot_icr_uncertainty_m:
        # rotation_centre, pivot_clearance, pivot_radius_required) through estimator_factory; decisions and controller
        # snapshots stay the same
        estimator_module, _ = core.load_vendor()
        cfg = steer_config()
        base = estimator_module.EstimatorConfig(**cfg.estimator_overrides())
        other = replace(base, pivot_icr_behind_lidar_m=base.pivot_icr_behind_lidar_m + .5,
                        pivot_icr_uncertainty_m=base.pivot_icr_uncertainty_m + .4)
        a, b = StubSteer(), StubSteer()
        runner_a, driver_a, out_a = run_static(cfg, a)
        runner_b, driver_b, out_b = run_static(cfg, b,
                                               estimator_factory=lambda: estimator_module.SensorEstimator(other))

        def trace(out):
            return [(round(t, 3), d.state, d.reason, d.throttle, d.steer_mrad) for t, d in out]
        self.assertEqual(trace(out_a), trace(out_b))
        self.assertTrue(a.snapshots)
        self.assertEqual(a.snapshots, b.snapshots)
        # the estimators themselves differ in these values (the comparison above can fail)
        raw_a, raw_b = runner_a.estimator.snapshot(driver_a.t), runner_b.estimator.snapshot(driver_b.t)
        self.assertNotEqual((raw_a['rotation_centre_x_m'], raw_a['rotation_centre_sd_m']),
                            (raw_b['rotation_centre_x_m'], raw_b['rotation_centre_sd_m']))
        self.assertNotEqual(raw_a.get('pivot_radius_required_m'), raw_b.get('pivot_radius_required_m'))

    def test_no_steering_change_after_pause_and_end(self):
        controller = StubSteer(step=3)

        def hold(t):
            return t < 2004.5
        runner = Runner(steer_config(), controller_factory=lambda: controller)
        driver = L.FakeTimeDriver(runner, lambda t: SCENE, t0=2000.)
        runner.start_run(driver.t, None)
        out = driver.run(3.)
        out += driver.run(3., hold=hold)
        paused = [(t, d) for t, d in out if d.state == PAUSED and t > 2004.5]
        self.assertTrue(paused)
        last_active = [d for t, d in out if d.state == ACTIVE][-1]
        self.assertTrue(all(d.steer_mrad == last_active.steer_mrad and d.throttle == 0 for _, d in paused))
        runner.end_run(driver.t, 'operator_stop')
        d = runner.tick(driver.t + .05, False)
        self.assertEqual((d.state, d.throttle, d.steer_mrad), (ENDED, 0, last_active.steer_mrad))

    def test_controller_steer_change_in_a_stop_state_ends_the_run(self):
        controller = StubSteer(throttle=0, state='STOP_REQUESTED', step=5)
        runner, _, out = run_static(steer_config(), controller)
        ended = [d for _, d in out if d.state == ENDED]
        self.assertTrue(ended)
        self.assertEqual(ended[0].reason, 'guard_violation:steer_in_non_driving_state')

    def test_rate_violation_ends_the_run(self):
        controller = StubSteer(step=60)
        runner, _, out = run_static(steer_config(), controller)
        ended = [d for _, d in out if d.state == ENDED]
        self.assertTrue(ended)
        self.assertEqual(runner.end_reason, 'guard_violation:steer_rate_above_limit')

    def test_required_steering_feedback(self):
        # armed (steer_required): a press without usable steering feedback is not taken
        controller = StubSteer()
        runner, _, out = run_static(steer_config(), controller, feed=lambda t: None)
        self.assertFalse([d for _, d in out if d.state == ACTIVE])
        self.assertIn('press_ignored_steering_feedback', [d.reason for _, d in out])
        # fresh, confirmed, calibrated: the run goes ACTIVE; the first measured angle seeds the held steering
        controller = StubSteer(start=37, step=0)
        runner, _, out = run_static(steer_config(), controller, feed=lambda t: (block(measured=37), host()))
        self.assertTrue([d for _, d in out if d.state == ACTIVE])
        first = [d for _, d in out if d.state == ACTIVE][0]
        self.assertEqual(first.steer_mrad, 37)
        self.assertEqual(controller.snapshots[0]['steer_measured_mrad'], 37.)
        self.assertEqual(controller.snapshots[0]['steer_command_mrad'], 37)
        # a controller that jumps away from the held (measured) steering is stopped by the slew rule (design 6.5)
        runner, _, out = run_static(steer_config(), StubSteer(start=0, step=0),
                                    feed=lambda t: (block(measured=37), host()))
        self.assertEqual(runner.end_reason, 'guard_violation:steer_rate_above_limit')

    def test_steering_fault_and_stale_status_end_the_run(self):
        def feed(t):
            if t > 2004.:
                return block(faults=['STALL']), host()
            return block(), host()
        controller = StubSteer(step=0)
        runner, _, out = run_static(steer_config(), controller, feed=feed)
        self.assertEqual(runner.end_reason, 'steering_stalled')
        self.assertIn('STALL', runner.end_note or '')

        def stale(t):
            return (block(), host()) if t < 2004. else None
        runner, _, out = run_static(steer_config(), StubSteer(step=0), feed=stale)
        self.assertEqual(runner.end_reason, 'steering_feedback_stale')

    def test_status_reports_the_steering_block(self):
        controller = StubSteer(step=0)
        runner, driver, _ = run_static(steer_config(), controller, feed=lambda t: (block(), host()))
        status = runner.status()
        self.assertEqual(status['vehicle_profile'], 'front_steer')
        self.assertGreater(status['steer_feedback_fed'], 0)


# --- precheck and client --------------------------------------------------------------------------------------------
def good_status(**over):
    status = dict(ok=True, connected=True, firmware='UBTS_STEER_V3', protocol=3, vehicle_profile='front_steer',
                  profile='front_steer', simulated=True, steer_limit_mrad=dict(left=409, right=409), cal_id=1,
                  steering=dict(block(), faults=[]), steering_host=dict(host(stop_state='confirmed')),
                  armed=False, state='disarmed', moving=False, updated_at=100.)
    status.update(over)
    return status


def precheck(status, **over):
    kw = dict(expect_cal_id=1, allow_sim=True, limit_left_mrad=409, limit_right_mrad=409, now=100.5)
    kw.update(over)
    return steer_arm_precheck(status, **kw)


class PrecheckTest(unittest.TestCase):
    def test_good_status_passes(self):
        self.assertIsNone(precheck(good_status()))

    def test_each_rule(self):
        cases = [
            (dict(ok=False), {}, 'status_not_ok'),
            (dict(connected=False), {}, 'esp_not_connected'),
            (dict(firmware='UBTS_ESP_V2'), {}, 'firmware_v3_required'),
            (dict(protocol=2), {}, 'firmware_v3_required'),
            (dict(vehicle_profile='differential'), {}, 'vehicle_profile_mismatch'),
            (dict(profile='differential'), {}, 'vehicle_profile_mismatch'),
            (dict(simulated=False), {}, 'sim_identity_mismatch'),               # SIM executor, real device
            ({}, dict(allow_sim=False), 'sim_identity_mismatch'),               # field executor, SIM device
            (dict(steer_limit_mrad=dict(left=450, right=409)), {}, 'steer_limit_mismatch'),
            (dict(steering=None), {}, 'steering_status_missing'),
            (dict(steering=dict(block(calibrated=False))), {}, 'steering_uncalibrated'),
            ({}, dict(expect_cal_id=None), 'calibration_id_not_configured'),
            (dict(cal_id=2), {}, 'calibration_mismatch'),
            (dict(steering=dict(block(faults=['POLARITY']))), {}, 'polarity_check_missing'),
            (dict(steering=dict(block(faults=['DRV_STUCK']))), {}, 'polarity_check_missing'),
            (dict(steering=dict(block(faults=['STALL']))), {}, 'steering_fault:STALL'),
            (dict(steering_host=dict(host(stop_state='confirmed'), stale=True)), {}, 'steering_status_stale'),
            (dict(steering=dict(block(age_ms=400))), {}, 'steering_status_stale'),
            (dict(steering=dict(block(measured=float('nan')))), {}, 'steering_feedback_invalid'),
            (dict(steering_host=dict(host(stop_state='pending'))), {}, 'stop_unconfirmed'),
            (dict(armed=True), {}, 'bridge_armed_by_other_client'),
            (dict(state='fault'), {}, 'bridge_state_fault'),
            (dict(moving=True), {}, 'bridge_reports_moving'),
            ({}, dict(now=110.), 'bridge_snapshot_stale'),
        ]
        for change, kw, want in cases:
            self.assertEqual(precheck(good_status(**change), **kw), want, (change, kw))

    def test_ignored_bits_and_rearm(self):
        self.assertIsNone(precheck(good_status(steering=dict(block(faults=['BOOT', 'TTL', 'SOFTLIMIT'])))))
        status = good_status(steering=dict(block(faults=['STALL', 'mcu_disarmed'])), state='fault')
        self.assertEqual(precheck(status), 'steering_fault:STALL,mcu_disarmed')
        self.assertIsNone(precheck(status, rearm=True))       # the operator re-arm: the bridge decides
        self.assertEqual(precheck(good_status(steering=dict(block(faults=['POLARITY']))), rearm=True),
                         'polarity_check_missing')


class FakeSteerBridge:
    """A transport answering like the steer bridge's control API (lease, ticket, seq, the status in every reply)."""

    def __init__(self, status=None):
        self.status = status or good_status()
        self.requests = []
        self.lease = None
        self.drive_error = None

    def __call__(self, request):
        self.requests.append(dict(request))
        op = request.get('op')
        if op == 'status':
            return dict(self.status)
        if op == 'arm':
            self.lease = 'L1'
            return dict(ok=True, lease='L1', ticket='T0', seq=0, status=dict(self.status))
        if op in ('drive', 'keepalive'):
            if self.drive_error:
                return dict(ok=False, error=self.drive_error)
            return dict(ok=True, lease=self.lease, ticket='T%d' % request['seq'], seq=request['seq'],
                        status=dict(self.status))
        if op == 'motion':
            return dict(ok=True)
        if op == 'stop':
            self.lease = None
            return dict(ok=True, state='stop_requested')
        if op == 'release':
            return dict(ok=True)
        return dict(ok=False, error='unknown_op')


def client(transport, **over):
    kw = dict(transport=transport, owner='test', limit_left_mrad=409, limit_right_mrad=409, expect_cal_id=1,
              allow_sim=True, clock=lambda: 50., wall=lambda: 100.5)
    kw.update(over)
    return SteerBridgeClient(**kw)


class SteerBridgeClientTest(unittest.TestCase):
    def test_arm_drive_keepalive_motion(self):
        t = FakeSteerBridge()
        c = client(t)
        c.arm()
        self.assertEqual(c.state, ARMED)
        self.assertEqual(t.requests[-1], dict(op='arm', owner='test', vehicle_profile='front_steer'))
        c.drive(40, -25)
        sent = t.requests[-1]
        self.assertEqual((sent['op'], sent['throttle'], sent['steer_mrad'], sent['deadman'], sent['seq']),
                         ('drive', 40, -25, True, 1))
        c.keepalive()
        self.assertEqual((t.requests[-1]['op'], t.requests[-1]['seq']), ('keepalive', 2))
        self.assertTrue(c.motion(.01, .4))
        self.assertEqual(c.steering()[1]['measured_mrad'], 12)

    def test_precheck_refusal_sends_no_arm(self):
        t = FakeSteerBridge(good_status(cal_id=2))
        c = client(t)
        with self.assertRaises(BridgeError) as ctx:
            c.arm()
        self.assertIn('arm_precheck:calibration_mismatch', str(ctx.exception))
        self.assertNotIn('arm', [r['op'] for r in t.requests])
        self.assertEqual(c.state, LATCHED)

    def test_invalid_drive_values_latch(self):
        t = FakeSteerBridge()
        c = client(t)
        c.arm()
        for throttle, steer in ((40, 410), (40, 1.), (101, 0), (-1, 0), (True, 0)):
            c2 = client(FakeSteerBridge())
            c2.arm()
            with self.assertRaises(BridgeError):
                c2.drive(throttle, steer)
            self.assertEqual(c2.state, LATCHED, (throttle, steer))

    def test_fault_reason(self):
        t = FakeSteerBridge()
        c = client(t)
        t.status = good_status(steering=dict(block(faults=['TRACK'])),
                               steering_host=dict(host(), latches={'steering_ineffective': {}}))
        self.assertEqual(c.fault_reason(), 'steering_not_following')        # a fresh status request
        self.assertEqual(c.fault_reason(good_status(steering_host=dict(host(), latches={'stop_unconfirmed': {}}))),
                         'stop_unconfirmed')
        self.assertIsNone(c.fault_reason(good_status()))
        self.assertIsNone(c.fault_reason(dict(ok=True)))

    def test_motion_never_latches(self):
        class Refusing(FakeSteerBridge):
            def __call__(self, request):
                if request.get('op') == 'motion':
                    return dict(ok=False, error='control_not_owned')
                return FakeSteerBridge.__call__(self, request)
        c = client(Refusing())
        self.assertFalse(c.motion(0., 0.))            # not armed: nothing sent
        c.arm()
        self.assertFalse(c.motion(.01, .3))
        self.assertEqual((c.state, c.counts['motion_refused']), (ARMED, 1))

    def test_rearm_after_acknowledged_latch(self):
        status = good_status(steering=dict(block(faults=['mcu_disarmed'])), state='fault')
        t = FakeSteerBridge(status)
        c = client(t)
        with self.assertRaises(BridgeError):
            c.arm()
        self.assertEqual(c.state, LATCHED)
        c.acknowledge_latch()
        self.assertEqual(c.state, DISARMED)
        c.arm()
        self.assertEqual(c.state, ARMED)
        self.assertEqual(c.last_precheck, dict(problem=None, rearm=True))
        self.assertFalse(c.operator_ack)


if __name__ == '__main__':
    unittest.main()
