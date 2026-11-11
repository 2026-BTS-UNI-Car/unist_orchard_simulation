"""BridgeClient against the REAL dashboard RoverControl (scripts/rover_control_core.py), in-process.

rover_control_core imports only secrets/time, so this runs on every platform. The full bridge
(Bridge + Handler + Server over a Unix socket) is exercised in test_bridge_protocol_linux.py.
"""
import json
import time
import unittest

import rowexec_testlib as L
from row_executor import bridge_client as B
from row_executor.bridge_client import ARMED, DISARMED, LATCHED, BridgeClient, BridgeError

core_mod = L.import_dash('rover_control_core')
RoverControl, ControlError = core_mod.RoverControl, core_mod.ControlError


class FakeMotor:
    """Never touches hardware; records what the bridge core asked for."""
    firmware = 'UBTS_WHEEL_PWM_V2'

    def __init__(self):
        self.calls = []
        self.error = None
        self.outputs = {}
        self.stopped = True

    @property
    def ready(self):
        return not self.error

    def hard_stop(self):
        self.calls.append('STOP')
        self.stopped = True

    def enable(self):
        self.calls.append('enable')

    def drive(self, left, right):
        self.calls.append((left, right))
        self.stopped = not (left or right)

    def release(self):
        self.calls.append('release')


class CoreTransport:
    """Mimics Bridge.request on top of the real RoverControl (status gets updated_at like the snapshot)."""

    def __init__(self, control):
        self.control = control
        self.requests = []
        self.on_request = None          # optional hook(request) run before the bridge sees it

    def __call__(self, request):
        self.requests.append(json.loads(json.dumps(request)))
        if self.on_request is not None:
            self.on_request(request)
        if request.get('op') == 'status':
            self.control.tick()
            return dict(self.control.status(), updated_at=time.time(), mode='manual')
        if request.get('op') == 'stop':
            self.control.stop(str(request.get('reason', 'operator_stop'))[:80])
            return {'ok': True, 'state': 'stop_requested'}
        try:
            return self.control.dispatch(request)
        except (ControlError, ValueError) as exc:
            return {'ok': False, 'error': str(exc)}

    def ops(self):
        return [r['op'] for r in self.requests]


class ClientTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.motor = FakeMotor()
        self.control = RoverControl(self.motor, lambda: self.now)
        self.transport = CoreTransport(self.control)
        self.client = BridgeClient(owner='row-executor:test', transport=self.transport)

    def other_arms(self):
        return self.control.dispatch({'op': 'arm', 'owner': 'dashboard-manual'})

    def test_arm_then_drives_carry_deadman_and_sequence(self):
        self.client.arm()
        self.assertEqual(self.client.state, ARMED)
        for k, (left, right) in enumerate(((0, 0), (0, 0), (40, 50), (0, 0))):
            response = self.client.drive(left, right)
            self.assertEqual(response['seq'], k + 1)
            self.assertEqual(self.control.seq, k + 1)
        drives = [r for r in self.transport.requests if r['op'] == 'drive']
        self.assertEqual(len(drives), 4)
        self.assertTrue(all(r['deadman'] is True for r in drives))      # also on 0/0
        self.assertEqual([r['seq'] for r in drives], [1, 2, 3, 4])
        self.assertEqual(self.motor.calls[-3:], [(0, 0), (40, 50), (0, 0)])
        self.assertEqual(self.transport.ops()[:2], ['status', 'arm'])

    def test_zero_drives_keep_the_lease_beyond_ticket_and_input_ttl(self):
        self.client.arm()
        for _ in range(40):                     # 4 s of 0/0 at 10 Hz (> TICKET_TTL 1.5 s, > INPUT_TTL 0.75 s)
            self.client.drive(0, 0)
            self.now += .1
            self.control.tick()
        self.assertIsNotNone(self.control.lease)
        self.now += 1.                          # 1 s without sends while not moving: lease kept
        self.control.tick()
        self.client.drive(0, 0)
        self.assertEqual(self.client.state, ARMED)
        self.assertIsNotNone(self.control.lease)

    def test_drive_without_deadman_would_drop_the_lease(self):
        # Why deadman must be true on 0/0 too (FEASIBILITY_CRITIQUE H4a): the core rejects it.
        reply = self.control.dispatch({'op': 'arm', 'owner': 'x'})
        with self.assertRaisesRegex(ControlError, 'invalid_drive'):
            self.control.dispatch({'op': 'drive', 'owner': 'x', 'lease': reply['lease'], 'ticket': reply['ticket'],
                                   'seq': 1, 'deadman': False, 'left': 0, 'right': 0})
        self.assertIsNone(self.control.lease)

    def test_other_clients_stop_latches_and_never_rearms(self):
        self.client.arm()
        self.client.drive(30, 30)
        self.control.stop('dashboard_stop')                  # e.g. observer phone STOP
        with self.assertRaisesRegex(BridgeError, 'drive_refused:control_not_owned'):
            self.client.drive(30, 30)
        self.assertEqual(self.client.state, LATCHED)
        sent = len(self.transport.requests)
        with self.assertRaisesRegex(BridgeError, 'not_armed:LATCHED'):
            self.client.drive(0, 0)
        with self.assertRaisesRegex(BridgeError, 'latched'):
            self.client.arm()
        self.assertEqual(len(self.transport.requests), sent)  # nothing sent while latched
        self.assertEqual(self.transport.ops().count('arm'), 1)
        self.assertIsNone(self.control.lease)
        self.assertEqual(self.motor.calls.count('enable'), 1)
        self.assertEqual(self.client.acknowledge_latch(), DISARMED)   # explicit operator action
        self.client.arm()
        self.assertEqual(self.client.state, ARMED)

    def test_lost_lease_latches_without_stopping_the_next_owner(self):        # review S7
        self.client.arm()
        self.client.drive(30, 30)
        self.control.stop('dashboard_stop')                  # dashboard STOP removes our lease ...
        human = self.other_arms()                            # ... and the human arms manual driving
        n = len(self.transport.requests)
        with self.assertRaisesRegex(BridgeError, 'drive_refused:control_not_owned'):
            self.client.drive(30, 30)
        self.assertEqual(self.client.state, LATCHED)
        self.assertEqual(self.transport.ops()[n:], ['drive'])           # no op stop after the refusal
        self.assertEqual(self.control.lease, human['lease'])
        self.assertEqual(self.client.counts['latch_stop_skipped'], 1)

    def test_lease_gone_answers_never_stop_other_errors_still_do(self):
        cases = (('stop_barrier', False), ('control_not_owned', False), ('request_expired', True),
                 ('control_busy_stopped', True), ('stale_input_rearm_required', True), ('invalid_sequence', True))
        for error, stop_expected in cases:
            with self.subTest(error=error):
                self.setUp()
                self.client.arm()
                inner = self.transport

                def answer(request, error=error, inner=inner):
                    if request['op'] == 'drive':
                        inner.requests.append(dict(request))
                        return {'ok': False, 'error': error}
                    return inner(request)
                self.client.transport = answer
                with self.assertRaisesRegex(BridgeError, 'drive_refused:' + error):
                    self.client.drive(30, 30)
                self.assertEqual(self.client.state, LATCHED)
                self.assertEqual(self.transport.ops()[-1] == 'stop', stop_expected)
                self.assertEqual(self.client.counts['latch_stop_skipped'], 0 if stop_expected else 1)

    def test_input_expiry_after_0_75_s_without_sends(self):
        self.client.arm()
        self.client.drive(30, 30)
        self.now += .76
        self.control.tick()
        self.assertEqual(self.control.reason, 'input_expired')
        self.assertIsNone(self.control.lease)
        self.assertEqual(self.motor.calls[-1], 'STOP')
        with self.assertRaisesRegex(BridgeError, 'control_not_owned'):
            self.client.drive(30, 30)
        self.assertEqual(self.client.state, LATCHED)

    def test_control_in_use_refusal_does_not_stop_the_other_owner(self):
        other = self.other_arms()
        with self.assertRaisesRegex(BridgeError, 'arm_precheck:bridge_armed_by_other_client'):
            self.client.arm()
        self.assertEqual(self.client.state, LATCHED)
        self.assertNotIn('arm', self.transport.ops())
        self.assertNotIn('stop', self.transport.ops())
        self.assertEqual(self.control.lease, other['lease'])

    def test_control_in_use_race_between_status_and_arm(self):
        # The dashboard arms after our status precheck but before our arm request arrives.
        self.transport.on_request = lambda request: self.other_arms() if request['op'] == 'arm' else None
        with self.assertRaisesRegex(BridgeError, 'arm_refused:control_in_use'):
            self.client.arm()
        self.assertEqual(self.client.state, LATCHED)
        self.assertEqual(self.transport.ops(), ['status', 'arm'])     # no stop to the other owner
        self.assertEqual(self.control.owner, 'dashboard-manual')

    def test_invalid_duties_are_never_sent(self):
        for bad in ((-10, 30), (30, -10), (30., 30), (True, 30), (30, 101), (None, 0)):
            with self.subTest(bad=bad):
                self.setUp()
                self.client.arm()
                with self.assertRaisesRegex(BridgeError, 'invalid_drive_values'):
                    self.client.drive(*bad)
                self.assertEqual(self.client.state, LATCHED)
                self.assertNotIn('drive', self.transport.ops())
                self.assertEqual(self.transport.ops()[-1], 'stop')    # held a lease: stopped
                self.assertIsNone(self.control.lease)

    def test_transport_failure_latches_and_stops(self):
        self.client.arm()
        calls = []

        def broken(request):
            calls.append(request['op'])
            if request['op'] == 'drive':
                raise B.TransportError('timeout')
            return self.transport(request)
        self.client.transport = broken
        with self.assertRaisesRegex(BridgeError, 'drive_failed:timeout'):
            self.client.drive(0, 0)
        self.assertEqual(calls, ['drive', 'stop'])
        self.assertEqual(self.client.state, LATCHED)
        self.assertIsNone(self.control.lease)

    def test_malformed_drive_answer_latches(self):
        self.client.arm()

        def wrong_seq(request):
            response = self.transport(request)
            if request['op'] == 'drive':
                response = dict(response, seq=response['seq'] + 5)
            return response
        self.client.transport = wrong_seq
        with self.assertRaisesRegex(BridgeError, 'drive_malformed_response'):
            self.client.drive(0, 0)
        self.assertEqual(self.client.state, LATCHED)

    def test_status_not_ok_latches(self):
        self.client.transport = lambda request: {'ok': False, 'state': 'fault', 'last_error': 'boom'}
        with self.assertRaisesRegex(BridgeError, 'bridge_status_not_ok:fault:boom'):
            self.client.status()
        self.assertEqual(self.client.state, LATCHED)

    def test_normal_stop_and_operator_stop(self):
        self.client.arm()
        self.client.drive(0, 0)
        self.assertTrue(self.client.stop('row_end'))
        self.assertEqual(self.client.state, DISARMED)
        self.assertIsNone(self.control.lease)
        self.client.arm()                                     # explicit new Start is allowed
        self.assertTrue(self.client.stop('operator_button', latch=True))
        self.assertEqual(self.client.state, LATCHED)
        def unreachable(request):
            raise B.TransportError('socket_error:x')
        self.client.transport = unreachable
        self.client.acknowledge_latch()
        self.assertFalse(self.client.stop('x'))
        self.assertEqual(self.client.state, LATCHED)

    def test_drive_requires_armed(self):
        with self.assertRaisesRegex(BridgeError, 'not_armed:DISARMED'):
            self.client.drive(0, 0)
        self.assertEqual(self.transport.requests, [])


class PrecheckAndHintTest(unittest.TestCase):
    def good(self, **over):
        status = dict(ok=True, state='disarmed', connected=True, firmware=B.FIRMWARE, armed=False, moving=False,
                      last_error=None, updated_at=1000.)
        status.update(over)
        return status

    def test_precheck(self):
        self.assertIsNone(B.arm_precheck(self.good(), now=1000.5))
        cases = ((dict(ok=False), 'status_not_ok'), (dict(connected=False), 'esp_not_connected'),
                 (dict(firmware='UBTS_WHEEL_PWM_V1'), 'firmware_not_v2'),
                 (dict(last_error='mcu_watchdog_rearm_required'), 'bridge_error:mcu_watchdog_rearm_required'),
                 (dict(armed=True, state='armed_manual'), 'bridge_armed_by_other_client'),
                 (dict(state='starting'), 'bridge_armed_by_other_client'),
                 (dict(moving=True), 'bridge_reports_moving'), (dict(updated_at=990.), 'bridge_snapshot_stale'))
        for over, why in cases:
            with self.subTest(why=why):
                self.assertEqual(B.arm_precheck(self.good(**over), now=1000.5), why)

    def test_recovery_hints_name_docker_restart_never_compose(self):
        for status in (dict(ok=False, state='fault', last_error='x'),
                       self.good(last_error='mcu_watchdog_rearm_required', connected=False),
                       self.good(connected=False), None):
            with self.subTest(status=status):
                hint = B.recovery_hint(status)
                self.assertIn('docker restart unist-orachard-ros2-esp-rover-bridge', hint)
                self.assertIn('never docker compose', hint)
                self.assertIn('docker ps -a --filter name=esp-rover-bridge', hint)   # check the actual name (F8)
        self.assertIn('dashboard', B.recovery_hint(self.good(armed=True, state='armed_manual')))
        self.assertIn('docker restart', B.recovery_hint(None, error='drive_failed:timeout'))
        self.assertIn('NEVER use docker compose', B.RECOVERY_PROCEDURE)
        self.assertIn("docker ps -a --filter name=esp-rover-bridge --format '{{.Names}} {{.Status}}'",
                      B.RECOVERY_PROCEDURE)


if __name__ == '__main__':
    unittest.main()
