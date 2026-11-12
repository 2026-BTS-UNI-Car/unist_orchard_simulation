"""S2 format node and the supervisor hooks of WF-4 (pure parts: no ROS, no Gazebo, no files in the work tree).

  * steering.s2_format_node.S2Converter: the SIM convention check (3200 beams over 2 pi from -pi, frame laser), the
    conversion = steering.s2_format.RealFormat byte for byte (the F-SIM code), one input scan -> one output scan, the
    counters of rejected scans, stamp gaps (missing upstream scans) and stamps that do not increase, the status file,
    the yaw offset from the profile, the command line
  * steering.supervisor_steer: the S2 node is on by default (--s2-node none: off), its command is the node's command
    line (parsed by the node's own parser), its readiness needs the status of this run token, the sensor contract gets
    --s2-present; the stale status files of a role are moved to retired/<run token>/ (audit G06); the steer role's
    supervisor hooks (signal the bridge child: KILL / TERM / STOP / CONT, refusals)

unittest style (pytest on 3.12 / 3.13, python3 -m unittest on 3.10). Placeholder-based SIM results only.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np

from steering import gz_model
from steering import geometry as geo
from steering import s2_format as S2
from steering import s2_format_node as node
from steering import supervisor_steer as sup
from steering.vehicle_profile import load_profile

SIM_ROOT = Path(__file__).resolve().parent


def sim_scan(value=math.inf):
    return [value] * S2.SIM_BEAMS


def manifest(camera=True):
    return dict(scene='w0_force', sdf='world.sdf', sdf_sha256='0' * 64,
                model=dict(topics=dict(steer=gz_model.steer_topics(geo.LINKAGE), drive=gz_model.wheel_topics(),
                                       truth_odometry_gz=gz_model.TRUTH_ODOM_TOPIC,
                                       truth_joint_states_gz=gz_model.TRUTH_JOINT_STATE_TOPIC),
                           steer_type=geo.LINKAGE, drive_mode='force_dc'),
                renderer=dict(camera=camera, engine='ogre', headless=False))


class ConverterTest(unittest.TestCase):
    def setUp(self):
        self.conv = node.S2Converter(179.08)
        self.amin, self.inc = node.SIM_ANGLE_MIN, node.SIM_INCREMENT

    def test_conversion_is_realformat_byte_for_byte(self):
        rng = np.random.default_rng(3)
        ranges = rng.uniform(.04, 35., S2.SIM_BEAMS)
        ranges[::17] = np.inf
        ranges[5] = np.nan
        out, info = self.conv.convert(10_000_000_000, 'laser', self.amin, self.inc, ranges)
        ref = S2.RealFormat(179.08).convert(ranges)
        self.assertEqual(out.tobytes(), ref.tobytes())
        self.assertEqual(out.dtype, np.float32)
        self.assertEqual(out.shape, (S2.BEAMS,))
        self.assertIsNone(info['rejected'])
        self.assertTrue(np.all(np.isfinite(out)) and np.all(out >= 0.) and np.all(out < 30.))
        self.assertEqual(int(np.count_nonzero(out == 0.)), self.conv.counts['no_return_out'])

    def test_float32_input_of_the_ros_message(self):
        ranges = np.full(S2.SIM_BEAMS, 7.25, dtype=np.float32)
        out, _ = self.conv.convert(1, 'laser', np.float32(self.amin), np.float32(self.inc), ranges)
        self.assertEqual(out.tobytes(), S2.RealFormat(179.08).convert(ranges).tobytes())

    def test_yaw_convention_straight_ahead(self):
        # the SIM beam at body angle 0 (index 1600) appears at the raw beam whose angle + offset is 0
        ranges = sim_scan()
        ranges[S2.SIM_BEAMS // 2] = 5.
        out, _ = self.conv.convert(1, 'laser', self.amin, self.inc, ranges)
        j = int(np.flatnonzero(out > 0.)[0])
        body = math.atan2(math.sin(S2.RAW_AMIN + j * S2.RAW_INC + math.radians(179.08)),
                          math.cos(S2.RAW_AMIN + j * S2.RAW_INC + math.radians(179.08)))
        self.assertLess(abs(body), 2 * math.pi / S2.SIM_BEAMS)
        self.assertEqual(float(out[j]), 5.)

    def test_wrong_geometry_is_rejected_and_counted(self):
        cases = [('beam_count', dict(ranges=[1.] * 3240)), ('angle_min', dict(amin=0.)),
                 ('angle_increment', dict(inc=2 * math.pi / 3240)), ('frame_id', dict(frame='base_link'))]
        for reason, change in cases:
            with self.subTest(reason=reason):
                conv = node.S2Converter()
                out, info = conv.convert(5, change.get('frame', 'laser'), change.get('amin', self.amin),
                                         change.get('inc', self.inc), change.get('ranges', sim_scan(3.)))
                self.assertIsNone(out)
                self.assertEqual(info['rejected'], reason)
                self.assertEqual(conv.counts['rejected'], 1)
                self.assertEqual(conv.rejected[reason], 1)
                self.assertEqual(conv.counts['scans_out'], 0)
                self.assertEqual(conv.status('input_rejected')['last_reject']['reason'], reason)
        conv = node.S2Converter()
        out, info = conv.convert(None, 'laser', self.amin, self.inc, sim_scan())
        self.assertIsNone(out)
        self.assertEqual(info['rejected'], 'stamp_invalid')

    def test_one_in_one_out_and_stamp_gaps(self):
        stamps = [0, 100, 200, 400, 500, 500, 450, 800]                 # ms: a gap of 1, a repeat, a step back, 2 gone
        for ms in stamps:
            out, _ = self.conv.convert(ms * 1_000_000, 'laser', self.amin, self.inc, sim_scan(2.))
            self.assertIsNotNone(out)
        counts = self.conv.counts
        self.assertEqual(counts['scans_in'], len(stamps))
        self.assertEqual(counts['scans_out'], len(stamps))
        self.assertEqual(counts['stamp_gaps'], 2)                     # 200 -> 400 and 450 -> 800
        self.assertEqual(counts['missing_scans'], 1 + 2)
        self.assertEqual(counts['stamps_not_increasing'], 2)          # 500 -> 500 and 500 -> 450

    def test_status_payload(self):
        self.conv.convert(1, 'laser', self.amin, self.inc, sim_scan(4.))
        self.conv.note_callback_ms(2.5)
        status = self.conv.status('running', run_token='tok', input_topic='/sim/scan')
        self.assertEqual(status['schema'], node.SCHEMA)
        self.assertEqual(status['node'], 'ubts_steer_s2_format')
        self.assertEqual(status['run_token'], 'tok')
        self.assertEqual(status['output']['beams'], 3240)
        self.assertEqual(status['output']['angle_min'], S2.RAW_AMIN)
        self.assertEqual(status['output']['angle_increment'], S2.RAW_INC)
        self.assertEqual(status['output']['frame_id'], 'laser')
        self.assertEqual(status['callback_ms']['last'], 2.5)
        self.assertAlmostEqual(node.RAW_ANGLE_MAX, float(np.float32(S2.RAW_AMIN + 3239 * S2.RAW_INC)))
        json.dumps(status, allow_nan=False)
        with tempfile.TemporaryDirectory() as tmp:
            node.write_status(Path(tmp) / 'a' / 's2.json', status)
            raw = (Path(tmp) / 'a' / 's2.json').read_bytes()
            self.assertNotIn(b'\r\n', raw)
            self.assertEqual(json.loads(raw.decode('utf-8'))['state'], 'running')

    def test_profile_offset_and_parser(self):
        offset, profile_id, sha = node.profile_yaw_offset()
        self.assertEqual(offset, float(load_profile().value('lidar_yaw_offset_deg')))
        self.assertTrue(profile_id)
        args = node.build_parser().parse_args([])
        self.assertEqual((args.input, args.output), ('/sim/scan', '/scan'))
        self.assertIsNone(args.yaw_offset_deg)
        with self.assertRaises(ValueError):
            node.S2Converter(float('nan'))

    def test_node_name_is_the_graph_rule_name(self):
        from steering import p6a_graph
        self.assertEqual(node.NODE_NAME, p6a_graph.S2_NODE)
        self.assertIn(p6a_graph.S2_NODE, p6a_graph.SIM_SCAN_CONSUMERS)
        self.assertIn(p6a_graph.S2_NODE, p6a_graph.CLIPPED_SCAN_CONSUMERS)
        self.assertNotIn(p6a_graph.S2_NODE, p6a_graph.TRUTH_CONSUMERS)     # the S2 node reads no truth

    def test_node_source_reads_no_truth_topic(self):
        text = (SIM_ROOT / 'steering' / 's2_format_node.py').read_bytes().decode('utf-8')
        code = text.split('"""', 2)[2]                                   # without the module docstring
        for word in ('/ground_truth', '/sim/truth', '/odom', '/tf', 'Odometry', 'create_subscription(Odometry'):
            self.assertNotIn(word, code)


class SupervisorS2HookTest(unittest.TestCase):
    def test_default_and_off(self):
        args = sup.build_parser().parse_args(['--role', 'sim'])
        if not os.environ.get('UBTS_STEER_S2_NODE'):
            self.assertEqual(sup.s2_node_of(args), sup.DEFAULT_S2_NODE)
        for off in ('none', 'off', 'NONE', '0'):
            args = sup.build_parser().parse_args(['--role', 'sim', '--s2-node', off])
            self.assertIsNone(sup.s2_node_of(args))
        args = sup.build_parser().parse_args(['--role', 'sim', '--s2-node', 'x.y'])
        self.assertEqual(sup.s2_node_of(args), 'x.y')

    def test_command_parses_with_the_nodes_parser(self):
        command = sup.s2_node_command(sup.DEFAULT_S2_NODE, '/sim/scan_clipped', Path('/runtime'), 'tok', 'p.yaml')
        self.assertEqual(command[1:4], ['-B', '-m', sup.DEFAULT_S2_NODE])
        args = node.build_parser().parse_args(command[4:])
        self.assertEqual((args.input, args.output, args.run_token, args.profile),
                         ('/sim/scan_clipped', '/scan', 'tok', 'p.yaml'))
        self.assertEqual(Path(args.status_file).as_posix(), '/runtime/s2_node_status.json')

    def test_readiness_needs_this_run_token(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 's2.json'
            probe = sup.s2_node_ready(path, 'tok')
            path.write_text(json.dumps(dict(state='running', run_token='old', updated_at=time.time())))
            self.assertFalse(probe())
            path.write_text(json.dumps(dict(state='waiting', run_token='tok', updated_at=time.time())))
            self.assertTrue(probe())
            path.write_text(json.dumps(dict(state='input_rejected', run_token='tok', updated_at=time.time())))
            self.assertFalse(probe())
            path.write_text(json.dumps(dict(state='running', run_token='tok', updated_at=time.time() - 60.)))
            self.assertFalse(probe())

    def test_sim_children_run_the_s2_node_and_tell_the_contract(self):
        with tempfile.TemporaryDirectory() as tmp:
            for extra, present in (([], True), (['--s2-node', 'none'], False)):
                args = sup.build_parser().parse_args(['--role', 'sim', '--runtime-dir', tmp] + extra)
                args.run_token = 'tok'
                children = sup.sim_children(args, Path(tmp) / 'scene', manifest())
                names = [c.name for c in children]
                contract = [c for c in children if c.name == 'sensor_contract'][0]
                self.assertEqual('s2_node' in names, present)
                self.assertEqual('--s2-present' in [str(a) for a in contract.command], present)
                if present:
                    s2 = [c for c in children if c.name == 's2_node'][0]
                    self.assertLess(names.index('cpu_lidar'), names.index('s2_node'))
                    self.assertLess(names.index('s2_node'), names.index('sensor_contract'))
                    self.assertIn('/sim/scan', [str(a) for a in s2.command])
                    self.assertIn('tok', [str(a) for a in s2.command])


class RetireStatusTest(unittest.TestCase):
    def test_old_status_files_move_to_the_run_token_folder(self):
        with tempfile.TemporaryDirectory() as tmp:
            runtime = Path(tmp)
            for name in ('scan_clip_status.json', 's2_node_status.json', 'cpu_lidar_status.json',
                         'steering_status.json', 'telemetry_snapshot.json'):
                (runtime / name).write_text('{"old": true}')
            moved = sup.retire_stale_status(runtime, 'sim', 'tok1')
            self.assertEqual(sorted(moved), ['cpu_lidar_status.json', 's2_node_status.json', 'scan_clip_status.json'])
            for name in moved:
                self.assertFalse((runtime / name).exists())
                self.assertTrue((runtime / 'steer' / 'retired' / 'tok1' / name).is_file())
            self.assertTrue((runtime / 'steering_status.json').is_file())     # the steer role's own file
            self.assertTrue((runtime / 'telemetry_snapshot.json').is_file())  # not a node status file
            self.assertEqual(sup.retire_stale_status(runtime, 'steer', 'tok2'), ['steering_status.json'])
            self.assertEqual(sup.retire_stale_status(runtime, 'sim', 'tok3'), [])

    def test_only_the_newest_folders_stay(self):
        with tempfile.TemporaryDirectory() as tmp:
            runtime = Path(tmp)
            for k in range(8):
                (runtime / 'cpu_lidar_status.json').write_text('{}')
                sup.retire_stale_status(runtime, 'sim', 'tok%d' % k, keep=3)
                folder = runtime / 'steer' / 'retired' / ('tok%d' % k)
                os.utime(folder, (1000. + k, 1000. + k))
            left = sorted(p.name for p in (runtime / 'steer' / 'retired').iterdir())
            self.assertLessEqual(len(left), 3)
            self.assertIn('tok7', left)

    def test_every_role_lists_its_files(self):
        self.assertEqual(set(sup.ROLE_STATUS_FILES), set(sup.ROLES))
        self.assertIn(sup.S2_STATUS, sup.ROLE_STATUS_FILES['sim'])
        self.assertIn('scan_clip_status.json', sup.ROLE_STATUS_FILES['sim'])


class FakeChild:
    def __init__(self, name, process):
        self.name, self.process = name, process

    def describe(self):
        return dict(name=self.name, pid=None if self.process is None else self.process.pid)


class FakeSupervisor:
    role = 'steer'
    run_token = 'tok'

    def __init__(self, children):
        self.children = children
        self.notes = []

    def note(self, text, **fields):
        self.notes.append((text, fields))


class SupervisorHooksTest(unittest.TestCase):
    def test_refusals(self):
        hooks = sup.SupervisorHooks(FakeSupervisor([FakeChild('bridge', None)]), '/nowhere',
                                    signals={'KILL': 9, 'STOP': 19, 'CONT': 18, 'TERM': 15})
        self.assertEqual(hooks.handle({'op': 'nope'})['error'], 'unknown_op')
        self.assertEqual(hooks.handle({'op': 'signal', 'child': 'emulator', 'signal': 'KILL'})['error'],
                         'child_not_allowed')
        self.assertEqual(hooks.handle({'op': 'signal', 'child': 'bridge', 'signal': 'HUP'})['error'],
                         'signal_not_allowed')
        self.assertIn('resume_after_s', hooks.handle({'op': 'signal', 'child': 'bridge', 'signal': 'KILL',
                                                      'resume_after_s': 1.})['error'])
        self.assertIn('resume_after_s', hooks.handle({'op': 'signal', 'child': 'bridge', 'signal': 'STOP',
                                                      'resume_after_s': 99.})['error'])
        self.assertEqual(hooks.handle({'op': 'signal', 'child': 'bridge', 'signal': 'KILL'})['error'],
                         'child_not_running')
        status = hooks.handle({'op': 'status'})
        self.assertTrue(status['ok'])
        self.assertEqual(status['children'][0]['name'], 'bridge')

    @unittest.skipUnless(hasattr(signal, 'SIGSTOP') and Path('/proc').is_dir(), 'POSIX signals and /proc (Linux)')
    def test_stop_resume_and_kill_a_real_child(self):
        child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
        try:
            fake = FakeSupervisor([FakeChild('bridge', child)])
            hooks = sup.SupervisorHooks(fake, '/nowhere')
            reply = hooks.handle({'op': 'signal', 'child': 'bridge', 'signal': 'STOP', 'resume_after_s': .3})
            self.assertTrue(reply['ok'], reply)
            time.sleep(.1)
            self.assertEqual(proc_state(child.pid), 'T')
            time.sleep(.6)
            self.assertNotEqual(proc_state(child.pid), 'T')
            reply = hooks.handle({'op': 'signal', 'child': 'bridge', 'signal': 'KILL'})
            self.assertTrue(reply['ok'])
            self.assertEqual(child.wait(timeout=5.), -signal.SIGKILL)
            self.assertIn('HOOK_SIGNAL', [n[0] for n in fake.notes])
        finally:
            if child.poll() is None:
                child.kill()
                child.wait()

    @unittest.skipUnless(hasattr(__import__('socket'), 'AF_UNIX') and sys.platform != 'win32', 'Unix sockets')
    def test_socket_server(self):
        from steering.p6a_client import unix_request
        with tempfile.TemporaryDirectory(dir='/tmp') as tmp:
            path = os.path.join(tmp, 'sup.sock')
            hooks = sup.SupervisorHooks(FakeSupervisor([]), path).start()
            try:
                self.assertTrue(unix_request(path, {'op': 'status'})['ok'])
                self.assertEqual(unix_request(path, {'op': 'signal', 'child': 'bridge', 'signal': 'KILL'})['error'],
                                 'child_not_running')
            finally:
                hooks.close()
            self.assertFalse(os.path.exists(path))


def proc_state(pid):
    text = Path('/proc/%d/stat' % pid).read_text()
    return text.rsplit(')', 1)[1].split()[0]


if __name__ == '__main__':
    unittest.main()
