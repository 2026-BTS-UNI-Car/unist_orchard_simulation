"""Protocol test: BridgeClient (real UnixSocketTransport) against the REAL dashboard ESP bridge.

Uses scripts/rover_control_bridge.py Bridge + Handler + Server and scripts/rover_control_core.py
RoverControl from the read-only dashboard checkout, in-process, on a temporary Unix socket, with a
fake motor object in place of EspMotor (no serial port, no hardware, no motor output anywhere).
The bridge loop below mirrors rover_control_bridge.main() without rclpy and the status file.
Linux only: rover_control_bridge imports scripts.esp_motor, which imports fcntl and termios.

ReadOnlyBindTest (review S10) runs only when UBTS_RO_BIND_RW and UBTS_RO_BIND_RO name the SAME directory, the
second one mounted read-only, e.g. in a container:  -v <dir>:/rw -v <dir>:/run/ubts-rover:ro  (exactly the
run_executor.sh motor/bench mount); the bridge creates its socket on the writable side, the client connects
through the read-only one.
"""
import errno
import inspect
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest

import rowexec_testlib as L
from row_executor import bridge_client as B
from row_executor.bridge_client import ARMED, DISARMED, LATCHED, BridgeClient, BridgeError, UnixSocketTransport

LINUX = sys.platform.startswith('linux')
bridge_mod = L.import_dash('rover_control_bridge') if LINUX else None


class FakeEspMotor:
    """The attributes/methods of scripts/esp_motor.EspMotor that Bridge and RoverControl use."""

    def __init__(self):
        self.port, self.identity, self.fd = '/fake/ubts-motor', 'fake-identity', 3
        self.firmware = B.FIRMWARE
        self.error = None
        self.outputs = {p: 0 for p in ('D8', 'D5', 'D3', 'D0')}
        self.calls = []
        self.drive_error = None
        self.lock = threading.Lock()

    @property
    def ready(self):
        return self.fd is not None and self.firmware == B.FIRMWARE and not self.error

    @property
    def stopped(self):
        return all(v == 0 for v in self.outputs.values())

    def poll(self):
        pass

    def enable(self):
        if not self.ready:
            raise RuntimeError('verified ESP V2 required')
        self.calls.append('enable')

    def drive(self, left, right):
        if self.drive_error:
            raise RuntimeError(self.drive_error)          # e.g. WheelController 'PC heartbeat expired'
        self.calls.append((left, right))
        self.outputs.update(D8=max(left, 0), D5=max(-left, 0), D3=max(right, 0), D0=max(-right, 0))

    def release(self):
        self.calls.append('release')

    def hard_stop(self):
        self.calls.append('STOP')
        for pin in self.outputs:
            self.outputs[pin] = 0

    def close(self):
        self.calls.append('close')
        self.firmware = None
        self.fd = None


class Recording:
    def __init__(self, inner):
        self.inner, self.requests = inner, []

    def __call__(self, request):
        self.requests.append(dict(request))
        return self.inner(request)

    def ops(self):
        return [r['op'] for r in self.requests]


class BridgeHarness:
    def __init__(self, directory=None):
        self.own_tmp = directory is None
        self.tmp = tempfile.mkdtemp(prefix='ubts-rowexec-') if directory is None else directory
        self.path = os.path.join(self.tmp, 'control.sock')
        if not self.own_tmp and os.path.exists(self.path):
            os.unlink(self.path)
        self.motor = FakeEspMotor()
        self.bridge = bridge_mod.Bridge(self.motor)
        self.server = bridge_mod.Server(self.path, bridge_mod.Handler)
        os.chmod(self.path, 0o660)
        self.server.bridge = self.bridge
        self.accepted = 0
        original = self.server.process_request

        def counting(request, client_address):
            self.accepted += 1
            return original(request, client_address)
        self.server.process_request = counting
        self.stop_event = threading.Event()
        self.fault = None
        self.threads = [threading.Thread(target=self.server.serve_forever, kwargs=dict(poll_interval=.05), daemon=True),
                        threading.Thread(target=self.loop, daemon=True)]
        for thread in self.threads:
            thread.start()
        deadline = time.monotonic() + 2.
        while self.bridge.snapshot.get('state') != 'disarmed':
            if time.monotonic() > deadline:
                raise RuntimeError('bridge did not start')
            time.sleep(.01)

    def loop(self):
        # Mirrors scripts/rover_control_bridge.py main(): bridge.step() every 20 ms; an exception
        # escaping Bridge.step puts the loop into a permanent 'fault' state (see test below).
        fault = None
        while not self.stop_event.wait(0.02):
            try:
                if not fault:
                    self.bridge.step()
                else:
                    self.bridge.snapshot = {**self.bridge.control.status(), "ok": False, "state": "fault",
                                            "last_error": fault, "updated_at": time.time()}
            except Exception as exc:
                fault = str(exc)
                self.fault = fault
                try:
                    self.bridge.control.stop("hardware_fault")
                except OSError:
                    pass
                finally:
                    self.motor.close()

    def client(self, owner='row-executor:test'):
        transport = Recording(UnixSocketTransport(self.path, .6))
        return BridgeClient(self.path, owner=owner, transport=transport), transport

    def raw(self, request):
        return UnixSocketTransport(self.path, .6)(request)

    def close(self):
        self.stop_event.set()
        self.server.shutdown()
        self.server.server_close()
        for thread in self.threads:
            thread.join(2.)
        if self.own_tmp:
            shutil.rmtree(self.tmp, ignore_errors=True)
        elif os.path.exists(self.path):
            os.unlink(self.path)


@unittest.skipUnless(LINUX, 'real ESP bridge imports fcntl/termios (Linux only)')
class BridgeProtocolTest(unittest.TestCase):
    def setUp(self):
        self.h = BridgeHarness()
        self.addCleanup(self.h.close)
        self.client, self.transport = self.h.client()

    def test_loop_mirror_matches_dashboard_main(self):
        source = inspect.getsource(bridge_mod.main)
        for fragment in ('stopped.wait(0.02)', 'bridge.step()', '"state": "fault"', 'fault = str(exc)',
                         'bridge.control.stop("hardware_fault")', 'motor.close()'):
            self.assertIn(fragment, source)
        handle = inspect.getsource(bridge_mod.Handler.handle)
        self.assertIn('self.rfile.readline(4097)', handle)     # one line = one request per connection

    def test_arm_sequence_ticket_and_one_request_per_connection(self):
        arm = self.client.arm()
        self.assertEqual(self.client.state, ARMED)
        self.assertEqual(arm['seq'], 0)
        tickets = [self.client.ticket]
        for left, right in ((0, 0), (0, 0), (40, 50), (0, 0)):
            self.client.drive(left, right)
            tickets.append(self.client.ticket)
            time.sleep(.05)
        self.assertEqual(len(set(tickets)), 5)                         # a new ticket per drive
        self.assertEqual(self.client.seq, 4)
        self.assertEqual(self.h.bridge.control.seq, 4)
        self.assertEqual(self.h.bridge.control.lease, self.client.lease)
        self.assertEqual(self.h.motor.calls[-3:], [(0, 0), (40, 50), (0, 0)])
        drives = [r for r in self.transport.requests if r['op'] == 'drive']
        self.assertTrue(all(r['deadman'] is True for r in drives))
        self.assertEqual([r['seq'] for r in drives], [1, 2, 3, 4])
        self.assertEqual(self.transport.inner.connections, len(self.transport.requests))
        self.assertEqual(self.h.accepted, len(self.transport.requests))

    def test_zero_drive_with_deadman_keeps_the_lease(self):
        self.client.arm()
        lease = self.h.bridge.control.lease
        end = time.monotonic() + 2.                                     # > TICKET_TTL 1.5 s, > INPUT_TTL 0.75 s
        while time.monotonic() < end:
            self.client.drive(0, 0)
            time.sleep(.1)
        time.sleep(1.)                                                  # idle, not moving: no input expiry
        self.client.drive(0, 0)
        self.assertEqual(self.client.state, ARMED)
        self.assertEqual(self.h.bridge.control.lease, lease)
        self.assertNotIn('STOP', self.h.motor.calls[1:])
        # The same 0/0 drive without deadman is invalid_drive: hard stop + lease removed.
        response = self.h.raw({'op': 'drive', 'owner': self.client.owner, 'lease': self.client.lease,
                               'ticket': self.client.ticket, 'seq': self.client.seq + 1, 'deadman': False,
                               'left': 0, 'right': 0})
        self.assertEqual(response, {'ok': False, 'error': 'invalid_drive'})
        self.assertIsNone(self.h.bridge.control.lease)
        self.assertEqual(self.h.bridge.control.reason, 'invalid_drive')

    def test_other_clients_stop_latches_and_no_auto_rearm(self):
        self.client.arm()
        self.client.drive(30, 30)
        observer, _ = self.h.client('observer-phone')
        self.assertTrue(observer.stop('dashboard_stop'))              # STOP needs no lease
        time.sleep(.1)
        with self.assertRaisesRegex(BridgeError, 'drive_refused:control_not_owned'):
            self.client.drive(30, 30)
        self.assertEqual(self.client.state, LATCHED)
        sent = len(self.transport.requests)
        for action in (lambda: self.client.drive(0, 0), self.client.arm):
            with self.assertRaises(BridgeError):
                action()
        time.sleep(.5)
        self.assertEqual(len(self.transport.requests), sent)           # nothing sent while latched
        self.assertEqual(self.transport.ops().count('arm'), 1)
        self.assertIsNone(self.h.bridge.control.lease)
        self.assertEqual(self.h.motor.calls.count('enable'), 1)
        self.assertFalse(self.h.bridge.snapshot['armed'])

    def test_lost_lease_latches_without_stopping_the_human(self):           # review S7, real bridge
        self.client.arm()
        self.client.drive(30, 30)
        observer, _ = self.h.client('observer-phone')
        self.assertTrue(observer.stop('dashboard_stop'))
        time.sleep(.1)
        dashboard, _ = self.h.client('dashboard-session')
        dashboard.arm()                                                 # the human takes over at once
        n = len(self.transport.requests)
        with self.assertRaisesRegex(BridgeError, 'drive_refused:control_not_owned'):
            self.client.drive(30, 30)
        self.assertEqual(self.client.state, LATCHED)
        self.assertEqual(self.transport.ops()[n:], ['drive'])          # no op stop from the executor
        time.sleep(.1)
        self.assertEqual(self.h.bridge.control.owner, 'dashboard-session')
        dashboard.drive(0, 0)                                           # the human's lease still works
        self.assertEqual(dashboard.state, ARMED)

    def test_input_expiry_after_0_75_s_without_sends(self):
        self.client.arm()
        self.client.drive(30, 30)
        t0 = time.monotonic()
        while self.h.bridge.control.lease is not None and time.monotonic() - t0 < 2.:
            time.sleep(.01)
        expired_after = time.monotonic() - t0
        self.assertEqual(self.h.bridge.control.reason, 'input_expired')
        self.assertTrue(.7 <= expired_after <= 1.0, expired_after)
        self.assertEqual(self.h.motor.calls[-1], 'STOP')
        self.assertTrue(self.h.motor.stopped)
        with self.assertRaisesRegex(BridgeError, 'control_not_owned'):
            self.client.drive(30, 30)
        self.assertEqual(self.client.state, LATCHED)

    def test_control_in_use_refusal(self):
        dashboard, _ = self.h.client('dashboard-session')
        dashboard.arm()
        with self.assertRaisesRegex(BridgeError, 'arm_precheck:bridge_armed_by_other_client'):
            self.client.arm()
        self.assertEqual(self.client.state, LATCHED)
        self.assertEqual(self.transport.ops(), ['status'])              # no arm, no stop sent
        self.assertEqual(self.h.bridge.control.owner, 'dashboard-session')
        dashboard.drive(0, 0)                                           # the other owner is untouched

    def test_control_in_use_race_between_status_and_arm(self):
        dashboard, _ = self.h.client('dashboard-session')
        inner = self.transport.inner

        def racing(request):
            if request['op'] == 'arm':
                dashboard.arm()
            return inner(request)
        self.transport.inner = racing
        with self.assertRaisesRegex(BridgeError, 'arm_refused:control_in_use'):
            self.client.arm()
        self.assertEqual(self.client.state, LATCHED)
        self.assertEqual(self.transport.ops(), ['status', 'arm'])
        self.assertEqual(self.h.bridge.control.owner, 'dashboard-session')
        dashboard.drive(0, 0)

    def test_mcu_watchdog_error_needs_a_bridge_restart(self):
        self.client.arm()
        self.client.drive(0, 0)
        self.h.motor.error = 'mcu_watchdog_rearm_required'             # EspMotor.event('watchdog'); never cleared
        time.sleep(.1)
        self.assertEqual(self.h.bridge.control.reason, 'device_unavailable')
        with self.assertRaisesRegex(BridgeError, 'control_not_owned'):
            self.client.drive(0, 0)
        self.client.acknowledge_latch()
        with self.assertRaisesRegex(BridgeError, 'arm_precheck:esp_not_connected'):
            self.client.arm()
        status = self.h.raw({'op': 'status'})
        self.assertEqual(status['last_error'], 'mcu_watchdog_rearm_required')
        hint = B.recovery_hint(status)
        self.assertIn('docker restart unist-orachard-ros2-esp-rover-bridge', hint)
        self.assertIn('never docker compose', hint)
        # Even a raw arm is refused by the bridge itself until it is restarted (after the latch's
        # own stop has been processed: a request queued before that step gets 'stop_barrier').
        time.sleep(.1)
        self.assertEqual(self.h.raw({'op': 'arm', 'owner': 'x'}), {'ok': False, 'error': 'verified_esp_v2_required'})

    def test_wheel_runtime_error_faults_the_bridge_loop(self):
        self.client.arm()
        self.h.motor.drive_error = 'PC heartbeat expired; wheels disabled'
        with self.assertRaisesRegex(BridgeError, 'drive_refused:control_busy_stopped'):
            self.client.drive(30, 30)
        self.assertEqual(self.client.state, LATCHED)
        self.assertEqual(self.h.fault, 'PC heartbeat expired; wheels disabled')
        self.assertIn('close', self.h.motor.calls)
        time.sleep(.1)
        self.client.acknowledge_latch()
        with self.assertRaisesRegex(BridgeError, 'bridge_status_not_ok:fault'):
            self.client.status()
        self.assertIn('docker restart', B.recovery_hint(self.client.last_status))
        self.assertEqual(self.client.state, LATCHED)
        self.assertNotIn((30, 30), self.h.motor.calls)

    def test_normal_stop_then_explicit_rearm(self):
        self.client.arm()
        self.client.drive(0, 0)
        self.assertTrue(self.client.stop('row_end_candidate'))
        self.assertEqual(self.client.state, DISARMED)
        time.sleep(.1)
        self.assertIsNone(self.h.bridge.control.lease)
        self.client.arm()
        self.assertEqual(self.client.state, ARMED)
        self.client.drive(0, 0)


RO_RW, RO_RO = os.environ.get('UBTS_RO_BIND_RW'), os.environ.get('UBTS_RO_BIND_RO')


@unittest.skipUnless(LINUX and RO_RW and RO_RO, 'needs UBTS_RO_BIND_RW/RO (same directory, second bound :ro)')
class ReadOnlyBindTest(unittest.TestCase):
    """run_executor.sh motor/bench mount the socket directory :ro; connect still works (review S10)."""

    def test_connect_arm_drive_stop_through_a_read_only_bind(self):
        try:
            with open(os.path.join(RO_RO, 'probe-write'), 'w') as fh:
                fh.write('x')
            self.fail('the RO side is writable: not a read-only bind')
        except OSError as exc:
            self.assertEqual(exc.errno, errno.EROFS)
        for restart in range(2):                       # a bridge restart re-creates the socket (new inode)
            h = BridgeHarness(RO_RW)
            try:
                path = os.path.join(RO_RO, 'control.sock')
                client = BridgeClient(path, owner='row-executor:ro-bind', transport=UnixSocketTransport(path, .6))
                client.arm()
                client.drive(0, 0)
                client.drive(30, 30)
                client.drive(0, 0)
                self.assertEqual(client.seq, 3)
                self.assertTrue(client.stop('ro_bind_test'))
                time.sleep(.1)
                self.assertIsNone(h.bridge.control.lease)
                self.assertIn((30, 30), h.motor.calls)
            finally:
                h.close()


if __name__ == '__main__':
    unittest.main()
