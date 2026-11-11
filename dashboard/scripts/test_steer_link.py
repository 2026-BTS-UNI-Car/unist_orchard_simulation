"""V3 link (handshake, identity rule, status bookkeeping) and the three transports. No real port is opened."""
import ast
import os
import sys
import tempfile
import unittest

try:
    import test_steer_support as ts      # unittest discover -s dashboard/scripts
except ImportError:                     # pytest imports these modules as scripts.test_steer_*
    from scripts import test_steer_support as ts
from scripts import esp_motor_v3 as v3

POSIX = os.name == 'posix' and sys.platform.startswith('linux')
SCRIPTS = os.path.dirname(os.path.abspath(__file__))


class Pair:
    """A link on a MemoryTransport with a fake MCU on the device end (no bridge core)."""

    def __init__(self, simulated=True, **mcu):
        self.clock = ts.Clock()
        self.transport = v3.MemoryTransport(clock=self.clock)
        self.mcu = ts.FakeMcu(self.transport, **mcu)
        self.link = v3.EspSteerV3(self.transport, self.clock, simulated=simulated, expect_hw='front_steer_v0')
        self.events = []

    def step(self, seconds=0.01, n=1):
        for _ in range(n):
            self.clock.t = round(self.clock.t + seconds, 9)
            self.mcu.tick(self.clock.t)
            self.events.extend(self.link.poll())


class HandshakeTest(unittest.TestCase):
    def test_port_open_is_not_ready_until_a_banner(self):
        pair = Pair(reset_on_open=False)
        pair.mcu._boot(pair.clock.t)   # a running MCU: its boot banner went out while the port was closed (lost)
        pair.link.open()
        self.assertEqual(pair.link.state, 'awaiting_banner')
        self.assertFalse(pair.link.ready)
        # garbage, a half line and the status lines before the banner are discarded
        pair.transport.device.send(b'\x00\xff boot rom junk\nS 1 2*')
        pair.step(n=20)
        self.assertFalse(pair.link.ready)
        self.assertGreaterEqual(pair.link.counters['discarded_before_banner'], 2)
        self.assertEqual(pair.events, [])
        pair.step(n=128)                # 1.48 s after open: still no INFO
        self.assertNotIn(b'INFO', [line for _, line in pair.mcu.rx_lines])
        pair.step(n=3)                  # INFO at 1.5 s, the MCU answers with banner + cal line
        self.assertIn(b'INFO', [line for _, line in pair.mcu.rx_lines])
        self.assertTrue(pair.link.ready)
        self.assertEqual(pair.events[0]['type'], 'banner')
        self.assertTrue(pair.events[0]['new_session'])
        self.assertEqual(pair.link.boot_id, ts.BOOT_IDS[0])

    def test_info_repeats_while_no_banner(self):
        pair = Pair(reset_on_open=False)   # the device never answers
        pair.link.open()
        pair.step(n=320)
        self.assertEqual([label for _, label, _ in pair.link.tx_log].count('INFO'), 2)
        self.assertFalse(pair.link.ready)

    def test_new_boot_banner_on_open_and_reopen(self):
        pair = Pair()
        pair.link.open()
        pair.step()
        self.assertTrue(pair.link.ready)
        first = pair.link.boot_id
        self.assertEqual(first, ts.BOOT_IDS[0])
        pair.link.close()
        pair.link.open()        # port-open reset: a new boot id, a new session
        pair.step()
        self.assertTrue(pair.link.ready)
        self.assertNotEqual(pair.link.boot_id, first)
        self.assertEqual(pair.link.sessions, 2)

    def test_boot_change_mid_session(self):
        pair = Pair()
        pair.link.open()
        pair.step(n=12)
        pair.link.next_q = 77
        pair.mcu.reset(pair.clock.t)
        pair.events.clear()
        pair.step()
        banner = [e for e in pair.events if e['type'] == 'banner'][0]
        self.assertTrue(banner['boot_changed'])
        self.assertFalse(banner['new_session'])
        self.assertEqual(pair.link.next_q, 1)
        self.assertIsNone(pair.link.status)

    def test_q_continues_from_the_mcu_last_q_in_the_same_boot(self):
        pair = Pair()
        pair.link.open()
        pair.step()
        pair.mcu.last_q = 500   # the same boot already took C lines from an earlier host session
        pair.step(n=6)
        self.assertEqual(pair.link.next_q, 501)

    def test_status_bookkeeping(self):
        pair = Pair()
        pair.link.open()
        pair.step(n=12)
        pair.mcu.drop_status = {2, 3}
        pair.events.clear()
        pair.step(n=20)
        missing = [e['n_missing'] for e in pair.events if e['type'] == 'status']
        self.assertIn(2, missing)
        pair.mcu.ms_jump = -100000
        pair.events.clear()
        pair.step(n=6)
        self.assertTrue(any(e.get('ms_regressed') for e in pair.events if e['type'] == 'status'))

    def test_bad_lines_after_the_handshake_are_reported(self):
        pair = Pair()
        pair.link.open()
        pair.step()
        pair.transport.device.send(b'S 1 2 3*00\nZZZ*00\n' + v3.mcu_line('Q 1'))
        pair.events.clear()
        pair.step()
        codes = sorted(e['code'] for e in pair.events if e['type'] == 'bad_line')
        self.assertEqual(codes, ['crc', 'crc', 'kind'])


class IdentityTest(unittest.TestCase):
    def test_sim_wrapper_refuses_a_real_device(self):
        pair = Pair(simulated=True, sim=False)
        pair.link.open()
        pair.step()
        self.assertEqual(pair.link.state, 'refused')
        self.assertFalse(pair.link.ready)
        self.assertEqual(pair.link.identity_error, 'sim_identity_mismatch')
        with self.assertRaises(OSError):
            pair.link.send_ping()
        pair.link.send_stop()           # STOP is always allowed
        pair.step()
        self.assertIn(b'STOP', [line for _, line in pair.mcu.rx_lines])

    def test_real_wrapper_refuses_a_sim_device(self):
        pair = Pair(simulated=False, sim=True)
        pair.link.open()
        pair.step()
        self.assertEqual(pair.link.identity_error, 'sim_identity_mismatch')
        with self.assertRaises(OSError):
            pair.link.send_arm()

    def test_real_wrapper_accepts_a_real_device(self):
        pair = Pair(simulated=False, sim=False)
        pair.link.open()
        pair.step()
        self.assertTrue(pair.link.ready)

    def test_hardware_profile_mismatch_is_flagged(self):
        pair = Pair(hw='front_steer_v9')
        pair.link.open()
        pair.step()
        self.assertTrue(pair.link.ready)
        self.assertTrue(pair.link.hw_mismatch)

    def test_role_must_be_explicit(self):
        with self.assertRaises(TypeError):
            v3.EspSteerV3(v3.MemoryTransport(), simulated=None)


class MemoryTransportTest(unittest.TestCase):
    def test_latency_and_errors(self):
        clock = ts.Clock(10.0)
        transport = v3.MemoryTransport(clock=clock, latency_s=0.005)
        with self.assertRaises(OSError):
            transport.read()
        transport.device.send(b'lost')
        self.assertEqual(transport.lost_while_closed, 4)
        transport.open()
        transport.write(b'PING\n')
        self.assertEqual(transport.device.take_timed(), [(10.0, b'PING\n')])
        transport.device.send(b'abc')
        self.assertEqual(transport.read(), b'')
        clock.t = 10.005
        self.assertEqual(transport.read(2), b'ab')
        self.assertEqual(transport.read(), b'c')
        transport.device.unplug()
        with self.assertRaises(OSError):
            transport.read()
        with self.assertRaises(OSError):
            transport.write(b'x')
        transport.close()
        with self.assertRaises(OSError):
            transport.open()
        transport.device.replug()
        transport.open()
        self.assertEqual(transport.open_count, 2)


@unittest.skipUnless(POSIX, 'pty transport needs Linux ptys')
class PtyTransportTest(unittest.TestCase):
    def test_path_outside_the_root_is_refused_before_open(self):
        with self.assertRaises(OSError):
            v3.PtyTransport('/dev/ttyUSB0').open()
        with self.assertRaises(OSError):
            v3.PtyTransport('/run/ubts-steer/../ubts-rover/esp-tty').open()

    def test_link_to_another_device_is_refused_before_open(self):
        # 2026-11-09 review: a link under the allowed root to another device was opened before the tty / major
        # check. Opening a USB serial port can reset the NodeMCU (spec 10.7), so the check comes first (stat).
        from unittest import mock
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, 'esp-tty')
            os.symlink('/dev/null', path)                 # a character device, not a pty slave
            with mock.patch.object(v3.os, 'open', side_effect=AssertionError('opened before the check')):
                with self.assertRaises(OSError) as caught:
                    v3.PtyTransport(path, allowed_root=root).open()
            self.assertIn('not a pty slave', str(caught.exception))
            missing = os.path.join(root, 'none')
            os.symlink(os.path.join(root, 'absent'), missing)
            with self.assertRaises(OSError):
                v3.PtyTransport(missing, allowed_root=root).open()

    def test_pty_round_trip_and_refusals(self):
        master, slave = os.openpty()
        slave_name = os.ttyname(slave)
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, 'esp-tty')
            os.symlink(slave_name, path)
            transport = v3.PtyTransport(path, allowed_root=root)
            transport.open()
            try:
                transport.write(b'PING\n')
                self.assertEqual(os.read(master, 64), b'PING\n')
                os.write(master, v3.mcu_line('W 1'))
                data = b''
                for _ in range(50):
                    data += transport.read()
                    if data.endswith(b'\n'):
                        break
                self.assertEqual(data, b'W 1*%02X\n' % v3.crc8(b'W 1'))
                os.close(master)
                master = None
                with self.assertRaises(OSError):
                    for _ in range(50):
                        transport.read()
            finally:
                transport.close()
                if master is not None:
                    os.close(master)
                os.close(slave)
            plain = os.path.join(root, 'plain')
            with open(plain, 'w') as handle:
                handle.write('x')
            with self.assertRaises(OSError):
                v3.PtyTransport(plain, allowed_root=root).open()


class SerialTransportConfigTest(unittest.TestCase):
    def test_config_checks_need_no_device(self):
        for path, identity in (('', 'id'), ('auto', 'id'), ('/dev/serial/by-id/x', ''),
                               ('/run/ubts-steer/esp-tty', 'id'), ('/run/ubts-steer', 'id')):
            with self.assertRaises(RuntimeError):
                v3.SerialTransport(path, identity).check_config()
        v3.SerialTransport('/dev/serial/by-id/usb-x', 'Silicon_Labs_X').check_config()
        with self.assertRaises(RuntimeError):
            v3.SerialTransport('/dev/serial/by-id/usb-x', 'x', baud=9600).check_config()


class ImportRuleTest(unittest.TestCase):
    """fcntl/termios only inside the POSIX transports; no ROS, socket or sim imports in the core modules."""

    def imports(self, name, top_level_only):
        with open(os.path.join(SCRIPTS, name), encoding='utf-8') as handle:
            tree = ast.parse(handle.read())
        nodes = tree.body if top_level_only else list(ast.walk(tree))
        out = set()
        for node in nodes:
            if isinstance(node, ast.Import):
                out.update(alias.name.split('.')[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                out.add(node.module.split('.')[0])
        return out

    def test_rules(self):
        forbidden_everywhere = {'rclpy', 'numpy', 'scipy', 'steering', 'closed_loop', 'sim', 'serial'}
        for name in ('esp_motor_v3.py', 'steer_bridge_core.py', 'steer_control_core.py', 'steer_rover_bridge.py'):
            with self.subTest(name=name):
                self.assertFalse(self.imports(name, False) & (forbidden_everywhere - (
                    {'rclpy'} if name == 'steer_rover_bridge.py' else set())))
                self.assertFalse(self.imports(name, True) & {'fcntl', 'termios', 'rclpy', 'select'})
        for name in ('steer_bridge_core.py', 'steer_control_core.py'):
            self.assertFalse(self.imports(name, False) & {'fcntl', 'termios', 'socket', 'socketserver', 'serial',
                                                          'select', 'os'})


if __name__ == '__main__':
    unittest.main()
