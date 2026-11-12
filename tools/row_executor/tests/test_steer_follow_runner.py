"""The row-following controller (sim/ubts/closed_loop/steer_control.py) in the front_steer Runner's controller slot.

A static orchard scan and fake steering status replies: no plant, no closed loop (the vehicle never moves in the
scans). What is checked is the slot contract: the factory of scripted_control.make_factory({'kind': 'row_follow'}), the
alignment at standstill, a drive command the SteerOutputGuard accepts tick after tick, and the stale-input pause.
Every number is a test value.
"""
import os
import sys
import unittest

import rowexec_testlib as L
from row_executor.core import ACTIVE, ENDED, PAUSED, Runner
from test_steer_executor import MOTOR, SCIPY_PROBLEM, block, host
from row_executor.config import ExecutorConfig

SIM_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
                       'sim', 'ubts')
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)
from steering import scripted_control  # noqa: E402

SCENE = L.raw_scan(L.orchard_trunks())


@unittest.skipIf(SCIPY_PROBLEM, SCIPY_PROBLEM)
class RowFollowSlotTest(unittest.TestCase):
    def drive(self, seconds, scan_off_at=None):
        cfg = ExecutorConfig(**MOTOR).validate()
        factory = scripted_control.make_factory({'kind': 'row_follow'}, cfg)
        runner = Runner(cfg, controller_factory=factory)
        runner.steer_required = True
        driver = L.FakeTimeDriver(runner, lambda t: SCENE, t0=2000.)
        original = runner.tick
        measured = [0.]

        def tick(now, hold_active):
            command = runner._held_steer
            if command is not None:
                measured[0] = float(command)              # the steering follows the command (test stand-in)
            runner.feed_steering(now, block(measured=measured[0], age_ms=10), host())
            return original(now, hold_active)
        runner.tick = tick
        runner.start_run(driver.t, None)
        driver.run(1.5, hold=False)
        if scan_off_at is None:
            return runner, driver.run(seconds, hold=True)
        out = driver.run(scan_off_at, hold=True)
        driver.scan_on = False
        return runner, out + driver.run(seconds - scan_off_at, hold=True)

    def test_aligns_then_drives_inside_the_guard(self):
        runner, out = self.drive(4.)
        states = [d.state for _, d in out]
        self.assertEqual(states[-1], ACTIVE, out[-1][1].reason)
        self.assertNotIn(ENDED, states)
        self.assertIsNone(runner.last_violation)
        throttles = [d.throttle for _, d in out if d.state == ACTIVE]
        self.assertEqual(throttles[0], 0)                  # ALIGN: standstill first
        self.assertGreater(max(throttles), 15)             # then the start duty of the minimum speed and the slew
        self.assertLessEqual(max(throttles), 70)
        self.assertTrue(all(type(d.steer_mrad) is int for _, d in out if d.state == ACTIVE))
        self.assertTrue(out[-1][1].summary['controller_reason'].startswith('follow_'))
        self.assertEqual(type(runner.controller).__name__, 'RowFollowController')

    def test_stale_lidar_pauses_with_its_reason(self):
        runner, out = self.drive(4., scan_off_at=2.5)
        t_pause, decision = L.first(out, lambda d: d.state == PAUSED and d.reason != 'await_hold')
        self.assertIsNotNone(decision)
        self.assertEqual(decision.reason, 'stale_lidar')
        self.assertEqual((decision.throttle, out[-1][1].throttle), (0, 0))
        self.assertEqual(out[-1][1].state, PAUSED)         # no restart without a new press


class HoldOnce:
    """A steering controller stand-in: FOLLOW at standstill, then one HOLD with the given reason / recoverable flag."""

    def __init__(self, reason, recoverable, after=5):
        from closed_loop.steer_control import SteerCommand
        self.command, self.reason, self.recoverable, self.after = SteerCommand, reason, recoverable, after
        self.steps, self.state, self.pending, self.phase_events = 0, 'FOLLOW', None, []

    def step(self, t, s):
        self.steps += 1
        if self.steps > self.after:
            self.state = 'HOLD'
            return self.command('HOLD', self.reason, 0, None, self.recoverable)
        return self.command('FOLLOW', 'follow_cruise', 0, 0)

    def current_phase(self):
        return 'FOLLOW'

    def telemetry(self):
        return dict(cap='scan', caps=dict(cruise=.4, scan=.25), target_speed_m_s=.25, tau_loop_s=.33)


@unittest.skipIf(SCIPY_PROBLEM, SCIPY_PROBLEM)
class GovernorHoldTest(unittest.TestCase):
    """The steering controller's governor holds (PC-10): recoverable ones pause the run and count with the stale pauses,
    the others end it; the speed telemetry reaches the single-row block (the status 'speed' of the contract)."""

    def run_hold(self, reason, recoverable):
        cfg = ExecutorConfig(**MOTOR).validate()
        runner = Runner(cfg, controller_factory=lambda: HoldOnce(reason, recoverable))
        driver = L.FakeTimeDriver(runner, lambda t: SCENE, t0=2000.)
        runner.start_run(driver.t, None)
        driver.run(1.5, hold=False)
        return runner, driver.run(1., hold=True)

    def test_recoverable_hold_pauses_and_counts(self):
        for reason in ('speed_cap_below_min', 'overspeed'):
            runner, out = self.run_hold(reason, True)
            self.assertEqual((runner.state, runner.reason), (PAUSED, reason))
            self.assertEqual(runner.stale_events, 1)
            sr = [d.summary.get('single_row') for _, d in out if d.state == ACTIVE][-1]
            self.assertEqual((sr['speed_cap'], sr['cruise_speed_m_s'], sr['tau_loop_s']), ('scan', .4, .33))

    def test_terminal_hold_ends(self):
        runner, out = self.run_hold('row_end', False)
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'row_end'))


if __name__ == '__main__':
    unittest.main()
