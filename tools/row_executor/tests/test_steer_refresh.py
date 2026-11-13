"""A fresh steering status before a late tick (front_steer MOTOR; executor_main._refresh_steering, docs/steering/EXECUTOR_PERF_KO.md).

G-SIM stage 3 F (2026-11-13): three runs ended steering_feedback_stale because a long scan processing (100-270 ms) delayed
the tick: the steering status of the last drive reply was then 0.31-0.37 s old although the bridge and the MCU reported
on time. On the virtual clock of test_stop_lease (fake steer bridge, real SteerBridgeClient) a scan processing of 0.25 s is
emulated by moving the clock inside Runner.feed_scan:
  * the run goes on: one bridge status request before the tick, no drive, the steering status is fresh again;
  * the same delay with the MCU status line itself 0.4 s old (age_ms of the fresh status) still ends steering_feedback_stale;
  * a bridge that does not answer the status request still ends it;
  * a tick that is not late sends no status request (no change of the usual traffic);
  * a delay beyond the stall guard (0.35 s) still pauses the run (stall_guard), the refresh does not hide the executor's
    own lateness.
Test numbers only.
"""
import unittest

import rowexec_testlib as L  # noqa: F401  (path setup)
from row_executor import executor_main as M
from row_executor.bridge_client import ARMED
from row_executor.core import ACTIVE, ENDED, PAUSED
from test_stop_lease import SESSION, SteerRig


class SlowScanRig(SteerRig):
    """SteerRig whose next feed_scan (after slow_after()) moves the virtual clock by delay_s (a long scan processing)."""

    def __init__(self):
        super().__init__()
        self.delay = None
        runner_feed = None

        def install():
            nonlocal runner_feed
            runner = self.loop.runner
            original = runner.feed_scan

            def feed_scan(*a, **k):
                out = original(*a, **k)
                if self.delay is not None:
                    self.clock.t += self.delay
                    self.delay = None
                return out
            runner.feed_scan = feed_scan
        self.install = install

    def status_requests(self):
        return sum(1 for op in self.bridge.ops if op == 'status')


class CountingBridge:
    """Wraps the fake steer bridge: records the ops; status answers can be refused or given an old MCU line."""

    def __init__(self, bridge):
        self.bridge, self.ops = bridge, []
        self.refuse_status = False
        self.mcu_age_ms = None

    def __getattr__(self, name):
        return getattr(self.bridge, name)

    def __call__(self, request):
        op = request.get('op')
        self.ops.append(op)
        if op == 'status' and self.refuse_status:
            return dict(ok=False, error='bridge_not_answering')
        reply = self.bridge(request)
        if self.mcu_age_ms is not None:
            status = reply if op == 'status' else reply.get('status')
            if isinstance(status, dict) and isinstance(status.get('steering'), dict):
                status['steering'] = dict(status['steering'], age_ms=self.mcu_age_ms)
        return reply


def running():
    rig = SlowScanRig()
    counting = CountingBridge(rig.bridge)
    rig.bridge = counting
    rig.loop._stop_transport = counting               # the client factory reads rig.bridge at the Arm
    rig.armed()
    reply = rig.request('go', session=SESSION)
    assert reply['ok'], reply
    rig.run(.5)
    assert rig.loop.runner.state == ACTIVE
    rig.install()
    return rig


class RefreshTest(unittest.TestCase):
    def test_a_late_tick_reads_a_fresh_status_and_the_run_goes_on(self):
        rig = running()
        before = rig.status_requests()
        drives = rig.bridge.ops.count('drive')
        rig.run(1.)                                   # not late: no status request
        self.assertEqual(rig.status_requests(), before)
        self.assertGreater(rig.bridge.ops.count('drive'), drives)
        rig.delay = .25                               # the next scan processing takes 0.25 s
        rig.run(.5)
        self.assertEqual(rig.loop.runner.state, ACTIVE, rig.loop.runner.end_reason)
        self.assertEqual(rig.status_requests(), before + 1)
        self.assertEqual(rig.loop.counts['steer_refreshes'], 1)
        self.assertEqual(rig.loop.bridge.state, ARMED)

    def test_without_the_refresh_the_same_delay_ends_the_run(self):
        # the reference of the test above: the rule itself is unchanged (0.3 s)
        rig = running()
        rig.loop._refresh_steering = lambda now: False
        rig.delay = .25
        rig.run(.5)
        self.assertEqual((rig.loop.runner.state, rig.loop.runner.end_reason), (ENDED, 'steering_feedback_stale'))

    def test_an_old_mcu_line_is_still_stale(self):
        rig = running()
        rig.bridge.mcu_age_ms = 400                   # the bridge answers, the MCU status line is 0.4 s old
        rig.delay = .25
        rig.run(.5)
        self.assertEqual((rig.loop.runner.state, rig.loop.runner.end_reason), (ENDED, 'steering_feedback_stale'))

    def test_a_bridge_that_does_not_answer_is_still_stale(self):
        rig = running()
        rig.bridge.refuse_status = True
        rig.delay = .25
        rig.run(.5)
        self.assertEqual((rig.loop.runner.state, rig.loop.runner.end_reason), (ENDED, 'steering_feedback_stale'))
        self.assertGreaterEqual(rig.loop.counts['steer_refresh_failures'], 1)

    def test_a_delay_beyond_the_stall_guard_still_pauses(self):
        rig = running()
        rig.delay = .45                               # beyond stall_guard_s 0.35 s
        rig.run(.3)
        runner = rig.loop.runner
        self.assertIn(runner.state, (PAUSED, ENDED))
        self.assertNotEqual(runner.state, ACTIVE)
        self.assertEqual(rig.loop.config.stall_guard_s, .35)

    def test_the_fraction_and_the_limit(self):
        self.assertEqual(M.STEER_REFRESH_FRACTION, .5)
        rig = SteerRig()
        self.assertEqual(rig.loop.config.steer_status_max_age_s, .3)   # the vehicle limit is not changed


if __name__ == '__main__':
    unittest.main()
