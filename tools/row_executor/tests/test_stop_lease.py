"""Operator STOP during a pause and the lease of a waiting run (front_steer MOTOR, one_press; docs/steering/OPERATOR_API_KO.md
'STOP' and 'lease'; SIM dashboard sessions s2..s4 of 2026-11-13).

The ExecutorLoop on a virtual clock with scripted orchard scans and a fake steer bridge (lease, ticket and owner expiry,
op stop with the control reason operator_stop, a status-line latch) behind the real SteerBridgeClient:
  * a page STOP whose op stop refuses the drive of the same tick (stop_barrier) ends the run operator_stop, not
    bridge_error / SYSTEM_FAULT (session s2: run_end bridge_error drive_refused:stop_barrier 7 ms after the STOP);
  * a STOP that reached the bridge first (the dashboard's own bridge STOP, the CLI) ends it operator_stop too;
  * a lease taken by a steering fault while driving still ends the run with the steering reason;
  * an armed run that waits for its go keeps the lease (zero drives every send period, beyond the bridge's 1.5 s ticket
    and 30 s owner expiry);
  * a lease the bridge takes while the run waits (an idle-clearable status-line latch) keeps the run open: run_state
    'ready', go blockers lease_lost + arm_first, stop detail lease_lost; the next Arm re-arms and a go starts the run;
  * a STOP or a steering fault while the run waits still ends it.
Test numbers only (자리값 기반 SIM 결과 is not involved: no plant).
"""
import unittest

import rowexec_testlib as L  # noqa: F401  (path setup)
from row_executor import executor_main as M
from row_executor.bridge_client import ARMED, DISARMED, LATCHED, SteerBridgeClient
from row_executor.core import ACTIVE, ENDED, PAUSED, Runner, single_row_stop_category
from test_executor_loop import Clock, imu_msg, scan_msg
from test_steer_executor import SCENE, StubSteer, block, good_status, host, motor_config

TICKET_TTL_S, OWNER_TTL_S = 1.5, 30.          # rover_control_core.RoverControl (the bridge's control rules)
SESSION = 'screen-stop-0001'


class FakeSteerBridge:
    """The steer bridge's control API for the executor: arm / drive / keepalive / motion / stop / status. An op stop
    takes the lease with the control reason operator_stop (steer_bridge_core._step); a request after the ticket or the
    owner expiry takes it with stale_input / operator_expired; latch() takes it with a host latch. inject (callable) runs
    inside the next drive / keepalive, before its answer (a STOP arriving while the request is in flight)."""

    def __init__(self, clock):
        self.clock = clock
        self.lease, self.ticket, self.seq = None, None, 0
        self.reason = 'startup'
        self.faults, self.latches = [], {}
        self.ticket_until = self.owner_until = 0.
        self.inject, self.inject_error = None, None
        self.owned = []                              # (time, op) of every accepted drive / keepalive
        self.stops = 0

    def _take(self, reason):
        self.lease, self.ticket = None, None
        self.reason = reason

    def latch(self, name='steering_status_stale', faults=None):
        """A host latch (e.g. a status-line gap): STOP, the lease goes, state 'fault'."""
        self.latches[name] = dict(detail='test')
        self.faults = list(faults if faults is not None else [name])
        self._take(name)

    def status(self):
        armed = self.lease is not None
        state = 'armed_manual' if armed else ('fault' if self.latches else 'disarmed')
        return good_status(armed=armed, state=state, reason=self.reason,
                           steering=dict(block(faults=self.faults), faults=list(self.faults)),
                           steering_host=dict(host(arm_state='confirmed' if armed else 'none', stop_state='confirmed'),
                                              latches=dict(self.latches),
                                              counters=dict(stop_sent=self.stops, stop_confirmed=self.stops)))

    def __call__(self, request):
        op = request.get('op')
        now = self.clock()
        if op == 'status':
            return self.status()
        if op == 'stop':
            self.stops += 1
            self._take('operator_stop')
            return dict(ok=True, state='stop_requested')
        if op == 'arm':
            if self.lease is not None:
                return dict(ok=False, error='control_in_use')
            self.latches, self.faults = {}, []         # the re-arm clears the host latches (condition gone)
            self.lease, self.ticket, self.seq = 'L%d' % (len(self.owned) + 1), 'T0', 0
            self.reason = 'operator_armed'
            self.ticket_until, self.owner_until = now + TICKET_TTL_S, now + OWNER_TTL_S
            return dict(ok=True, lease=self.lease, ticket=self.ticket, seq=0, status=self.status())
        if op in ('drive', 'keepalive', 'motion', 'release'):
            if op in ('drive', 'keepalive') and self.inject is not None:
                inject, self.inject = self.inject, None
                inject()
                if self.inject_error:
                    error, self.inject_error = self.inject_error, None
                    return dict(ok=False, error=error)
            if self.lease is not None and now >= self.owner_until:
                self._take('operator_expired')
            if not self.lease or request.get('lease') != self.lease:
                return dict(ok=False, error='control_not_owned')
            if op == 'motion':
                return dict(ok=True)
            if request.get('ticket') != self.ticket or now >= self.ticket_until:
                self._take('stale_input')
                return dict(ok=False, error='stale_input_rearm_required')
            self.seq = request['seq']
            self.ticket = 'T%d' % self.seq
            self.ticket_until, self.owner_until = now + TICKET_TTL_S, now + OWNER_TTL_S
            self.owned.append((now, op))
            return dict(ok=True, lease=self.lease, ticket=self.ticket, seq=self.seq, status=self.status())
        return dict(ok=False, error='unknown_operation')


class SteerRig:
    """ExecutorLoop (front_steer MOTOR, one_press) + virtual clock + scan / IMU feeder + the operator screen."""

    def __init__(self, throttle=40):
        self.clock = Clock()
        self.bridge = FakeSteerBridge(self.clock)
        self.cfg = motor_config(start_mode='one_press')
        self.log = M._NullLog()

        def factory():
            return SteerBridgeClient(transport=self.bridge, owner='row-executor:test', limit_left_mrad=409,
                                     limit_right_mrad=409, expect_cal_id=1, allow_sim=True, clock=self.clock,
                                     wall=lambda: 100.5)
        self.loop = M.ExecutorLoop(self.cfg, motor_output_flag=True, clock=self.clock, wall=lambda: 1.7e9,
                                   bridge_factory=factory, stop_transport=self.bridge, log=self.log,
                                   runner_factory=lambda c: Runner(c, controller_factory=lambda: StubSteer(
                                       throttle=throttle, step=0)))
        self.next_imu = self.clock.t
        self.next_scan = self.clock.t + .013
        self.visible = True

    def run(self, seconds):
        out = []
        for _ in range(int(round(seconds / .05))):
            target = self.clock.t + .05
            while min(self.next_imu, self.next_scan) <= target:
                if self.next_imu <= self.next_scan:
                    self.clock.t = max(self.clock.t, self.next_imu)
                    self.loop.on_imu_msg(self.next_imu, imu_msg(self.next_imu - .01))
                    self.next_imu += 1 / 33.
                else:
                    self.clock.t = max(self.clock.t, self.next_scan)
                    self.loop.on_scan_msg(self.next_scan, scan_msg(self.next_scan - .1, SCENE))
                    self.next_scan += .1
            self.clock.t = target
            if self.visible:
                self.loop.operator_heartbeat(SESSION)
            out.append(self.loop.step())
        return out

    def request(self, kind, **kw):
        future = self.loop.submit(kind, **kw)
        self.run(.05)
        return future.result()

    def events(self, name):
        return [r for r in self.log.records if r.get('event') == name]

    def armed(self):
        assert self.request('start', row_len_m=None)['ok']
        reply = self.request('arm')
        assert reply['ok'], reply
        self.run(2.)                                  # inputs fresh for fresh_recover_s, corridor acquired
        return reply

    def running(self, test):
        self.armed()
        reply = self.request('go', session=SESSION)
        test.assertTrue(reply['ok'], reply)
        self.run(.5)
        test.assertEqual(self.loop.runner.state, ACTIVE)

    def category(self):
        runner = self.loop.runner
        return single_row_stop_category(getattr(runner, 'single_row', None), runner.end_reason)[0]


class OperatorStopTest(unittest.TestCase):
    def paused(self):
        rig = SteerRig()
        rig.running(self)
        rig.visible = False                           # the tab is hidden: the link expires, the run pauses
        rig.run(1.)
        self.assertEqual((rig.loop.runner.state, rig.loop.runner.reason), (PAUSED, 'operator_link_lost'))
        self.assertEqual(rig.loop.bridge.state, ARMED)            # zero drives with the steering held keep the lease
        return rig

    def test_page_stop_whose_barrier_refuses_the_drive_in_flight_is_an_operator_stop(self):
        rig = self.paused()
        # the page STOP arrives while the zero drive of a tick is in flight: emergency_stop (HTTP thread) sends its op
        # stop, the bridge answers the queued drive with stop_barrier (session s2 of 2026-11-13)
        rig.bridge.inject = lambda: rig.loop.emergency_stop('operator_page')
        rig.bridge.inject_error = 'stop_barrier'
        rig.run(.3)
        runner = rig.loop.runner
        self.assertEqual(runner.state, ENDED)
        self.assertEqual(runner.end_reason, 'operator_stop')
        self.assertEqual(rig.category(), 'OPERATOR_OR_BACKSTOP')
        self.assertIn('stop_barrier', runner.end_note)
        status = rig.loop.status()
        self.assertEqual((status['run_state'], status['stop_reason'], status['stop_detail']),
                         ('latched', 'OPERATOR_OR_BACKSTOP', 'operator_stop'))
        self.assertEqual(rig.events('run_end')[-1]['end_reason'], 'operator_stop')

    def test_a_stop_that_reached_the_bridge_first_is_an_operator_stop(self):
        rig = self.paused()
        rig.bridge({'op': 'stop', 'reason': 'dashboard_stop'})     # the dashboard's own bridge STOP comes first
        rig.run(.2)
        runner = rig.loop.runner
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'operator_stop'))
        self.assertEqual(runner.end_note, 'bridge_operator_stop:drive_refused:control_not_owned')
        self.assertEqual(rig.category(), 'OPERATOR_OR_BACKSTOP')
        rig.loop.emergency_stop('operator_page')                   # the executor STOP arrives after it: no change
        rig.run(.2)
        self.assertEqual(runner.end_reason, 'operator_stop')

    def test_a_steering_fault_that_took_the_lease_while_driving_keeps_its_reason(self):
        rig = SteerRig()
        rig.running(self)
        rig.bridge.latch('steering_status_stale')
        rig.run(.3)
        runner = rig.loop.runner
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'steering_feedback_stale'))
        self.assertEqual(rig.loop.status()['run_state'], 'latched')
        self.assertEqual(rig.events('lease_lost'), [])


class ArmedWaitTest(unittest.TestCase):
    def test_an_armed_run_keeps_the_lease_while_it_waits(self):
        rig = SteerRig()
        rig.armed()
        t0 = rig.clock.t
        rig.run(45.)                                  # longer than the bridge's 30 s owner expiry
        loop = rig.loop
        self.assertEqual((loop.runner.state, loop.runner.reason, loop.bridge.state), (PAUSED, 'await_hold', ARMED))
        self.assertEqual(loop.status()['run_state'], 'armed')
        times = [t for t, _ in rig.bridge.owned if t >= t0]
        gaps = [b - a for a, b in zip(times, times[1:])]
        self.assertGreater(len(times), 400)
        self.assertLessEqual(max(gaps), 1. / loop.config.send_hz + .05 + 1e-9)   # one send period + one tick
        self.assertEqual(rig.bridge.reason, 'operator_armed')

    def test_a_lease_the_bridge_takes_while_the_run_waits_needs_a_new_arm_only(self):
        rig = SteerRig()
        rig.armed()
        rig.bridge.latch('steering_status_stale')     # a status-line gap at the bridge (sessions s2, s4)
        rig.run(.3)
        loop = rig.loop
        self.assertEqual((loop.runner.state, loop.runner.reason), (PAUSED, 'await_hold'))       # the run stays open
        status = loop.status()
        self.assertEqual(status['run_state'], 'ready')
        self.assertIn('lease_lost', status['go_blockers'])
        self.assertIn('arm_first', status['go_blockers'])
        self.assertEqual((status['stop_reason'], status['stop_detail']), ('CONTROL_AUTHORITY', 'lease_lost'))
        lost = status['lease_lost']
        self.assertEqual((lost['bridge_reason'], lost['bridge_latches'], lost['error']),
                         ('steering_status_stale', ['steering_status_stale'], 'drive_refused:control_not_owned'))
        self.assertEqual(len(rig.events('lease_lost')), 1)
        self.assertEqual(rig.events('run_end'), [])
        reply = rig.request('go', session=SESSION)
        self.assertEqual((reply['ok'], reply['error']), (False, 'lease_lost'))
        rig.run(1.)                                   # nothing more is sent while the client is latched
        sent = len(rig.bridge.owned)
        rig.run(.5)
        self.assertEqual(len(rig.bridge.owned), sent)
        # the next Arm press: the operator's acknowledgement of that latch, a re-arm, then one go starts the run
        reply = rig.request('arm')
        self.assertTrue(reply['ok'], reply)
        self.assertTrue(reply['rearm_after_lease_lost'])
        self.assertEqual(loop.bridge.state, ARMED)
        self.assertEqual(len(rig.events('lease_lost_rearm')), 1)
        rig.run(.5)
        status = loop.status()
        self.assertEqual((status['run_state'], status['lease_lost'], status['stop_reason']), ('armed', None, None))
        reply = rig.request('go', session=SESSION)
        self.assertTrue(reply['ok'], reply)
        rig.run(.5)
        self.assertEqual(loop.runner.state, ACTIVE)

    def test_an_acknowledged_lease_loss_still_needs_the_arm(self):
        rig = SteerRig()
        rig.armed()
        rig.bridge.latch('steering_status_stale')
        rig.run(.3)
        self.assertTrue(rig.request('ack')['ok'])
        loop = rig.loop
        self.assertEqual(loop.bridge.state, DISARMED)
        status = loop.status()
        self.assertEqual(status['run_state'], 'ready')
        self.assertIn('lease_lost', status['go_blockers'])
        self.assertTrue(rig.request('arm')['ok'])
        self.assertEqual(loop.status()['run_state'], 'armed')

    def test_a_stop_while_the_run_waits_ends_it(self):
        rig = SteerRig()
        rig.armed()
        rig.bridge({'op': 'stop', 'reason': 'cli_stop'})          # somebody's STOP at the bridge
        rig.run(.3)
        runner = rig.loop.runner
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'operator_stop'))
        self.assertEqual(rig.loop.status()['run_state'], 'latched')
        self.assertEqual(rig.events('lease_lost'), [])

    def test_a_page_stop_while_the_run_waits_ends_it(self):
        rig = SteerRig()
        rig.armed()
        rig.bridge.inject = lambda: rig.loop.emergency_stop('operator_page')
        rig.bridge.inject_error = 'stop_barrier'
        rig.run(.3)
        self.assertEqual(rig.loop.runner.end_reason, 'operator_stop')

    def test_a_steering_fault_while_the_run_waits_ends_it(self):
        rig = SteerRig()
        rig.armed()
        rig.bridge.latch('steering_ineffective', faults=['TRACK'])
        rig.run(.3)
        runner = rig.loop.runner
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'steering_not_following'))
        self.assertEqual(rig.loop.bridge.state, LATCHED)
        self.assertEqual(rig.events('lease_lost'), [])

    def test_an_executor_stall_beyond_the_ticket_while_the_run_waits_ends_it(self):
        # the executor's own outage (no request for longer than the bridge ticket): a system fault, not a lease loss
        rig = SteerRig()
        rig.armed()
        rig.clock.t += TICKET_TTL_S + .2
        rig.next_imu, rig.next_scan = rig.clock.t, rig.clock.t + .013
        rig.run(.3)
        runner = rig.loop.runner
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'bridge_error'))
        self.assertEqual(rig.events('lease_lost'), [])


if __name__ == '__main__':
    unittest.main()
