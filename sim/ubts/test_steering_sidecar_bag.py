"""Separate bag process of the steering sidecar (design 4.5, A23): steering_bag_record.BagRecord with a fake 'ros2'.

The fake (written into a temporary folder) parses 'bag record -o OUT ... --topics T ...', creates OUT and a data
file, and on SIGINT (or SIGTERM) writes a rosbag2-like metadata.yaml and exits 0. Switches by environment:
FAKE_IGNORE=INT or INT,TERM (escalation), FAKE_FAIL=1 (exits at once), FAKE_COUNT (message count).

  * clean start / SIGINT stop: metadata finalized, topics and count, exit 0, signal list
  * escalation: SIGINT ignored -> SIGTERM after stop_timeout_s; both ignored -> SIGKILL, not finalized
  * a process that exits by itself (start failure) and a stop right after the start
  * the existing output folder is refused (rosbag2 refuses it as well)
  * orphan guard: a parent that dies without stopping the bag (os._exit) -> the bag gets SIGINT through
    'setpriv --pdeathsig INT' and still writes metadata.yaml (Linux with util-linux setpriv only)
  * the SidecarController with the real BagRecord and the fake ros2: a full recording, check_recording passes

POSIX only (signals, process groups); on Windows every test here is skipped with the reason. unittest style.
Temporary folders only. Placeholder-based SIM results only.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import textwrap
import time
import unittest

from steering import steering_bag_record as bag_mod
from steering import steering_sidecar as sc

FAKE_ROS2 = textwrap.dedent('''
    import os, signal, sys, time
    args = sys.argv[1:]
    assert args[:2] == ['bag', 'record'], args
    out = args[args.index('-o') + 1]
    topics = []
    i = args.index('--topics') + 1
    while i < len(args) and not args[i].startswith('-'):
        topics.append(args[i])
        i += 1
    ignore = set(filter(None, os.environ.get('FAKE_IGNORE', '').split(',')))
    count = int(os.environ.get('FAKE_COUNT', '7'))
    def finish(signum, frame):
        lines = ['rosbag2_bagfile_information:', '  version: 9', '  storage_identifier: mcap',
                 '  duration:', '    nanoseconds: 2000000000', '  starting_time:',
                 '    nanoseconds_since_epoch: 1700000000000000000', '  message_count: %d' % (count * len(topics)),
                 '  topics_with_message_count:']
        for t in topics:
            lines += ['    - topic_metadata:', '        name: %s' % t, '        type: std_msgs/msg/String',
                      '      message_count: %d' % count]
        lines += ['  relative_file_paths:', '    - bag_0.mcap']
        with open(os.path.join(out, 'metadata.yaml'), 'w') as handle:
            handle.write('\\n'.join(lines) + '\\n')
        sys.exit(0)
    for name in ('INT', 'TERM'):
        sig = getattr(signal, 'SIG' + name)
        signal.signal(sig, signal.SIG_IGN if name in ignore else finish)
    if os.environ.get('FAKE_FAIL') == '1':
        sys.exit(2)
    os.makedirs(out)
    with open(os.path.join(out, 'bag_0.mcap'), 'wb') as handle:
        handle.write(b'fake')
    with open(os.path.join(os.path.dirname(out), 'fake_started'), 'w') as handle:
        handle.write(str(os.getpid()))
    while True:
        time.sleep(0.05)
''')

PARENT = textwrap.dedent('''
    import os, sys, time
    sys.path.insert(0, sys.argv[1])
    from steering import steering_bag_record as bag_mod
    bag = bag_mod.BagRecord(sys.argv[2], ros2=[sys.executable, sys.argv[3]], guard=None)
    bag.start()
    deadline = time.monotonic() + 10
    while not os.path.exists(os.path.join(os.path.dirname(sys.argv[2]), 'fake_started')):
        if time.monotonic() > deadline:
            os._exit(3)
        time.sleep(0.02)
    print(bag.result()['orphan_guard'], flush=True)
    os._exit(0)          # no stop, no clean-up: the parent dies
''')

SIM_ROOT = Path(__file__).resolve().parent


def posix_only(cls):
    if os.name != 'posix':
        return unittest.skip('POSIX signals and process groups (ros2 bag record runs on Linux only)')(cls)
    return cls


@posix_only
class BagRecordTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.fake = self.base / 'fake_ros2.py'
        self.fake.write_text(FAKE_ROS2, encoding='utf-8')
        self.segment = self.base / 'steering'
        self.segment.mkdir()
        self.env = dict(os.environ)

    def bag(self, guard=(), **kw):
        kw.setdefault('stop_timeout_s', 0.5)
        kw.setdefault('kill_after_s', 0.5)
        return bag_mod.BagRecord(self.segment / 'bag', ros2=[sys.executable, str(self.fake)], guard=list(guard),
                                 log_path=self.segment / 'bag.log', env=self.env, **kw)

    def wait_started(self, limit=10.0):
        deadline = time.monotonic() + limit
        while not (self.segment / 'fake_started').exists():
            self.assertLess(time.monotonic(), deadline, 'fake ros2 did not start')
            time.sleep(0.02)

    def test_clean_start_and_stop(self):
        bag = self.bag(use_sim_time=True)
        started = bag.start()
        self.assertTrue(started['started'])
        self.assertIn('--use-sim-time', started['command'])
        self.wait_started()
        self.assertTrue(bag.running())
        result = bag.stop()
        self.assertEqual(result['exit_code'], 0)
        self.assertEqual(result['signals'][0][0], 'SIGINT')
        self.assertEqual(len(result['signals']), 1)
        meta = result['metadata']
        self.assertTrue(meta['finalized'])
        self.assertEqual(meta['topics'], {'/rover/steering/state': {'type': 'std_msgs/msg/String',
                                                                    'message_count': 7}})
        self.assertEqual(result['time_base'], 'ros_simulation_time')
        self.assertTrue((self.segment / 'bag.log').exists())

    def test_sigterm_after_ignored_sigint(self):
        self.env['FAKE_IGNORE'] = 'INT'
        bag = self.bag()
        bag.start()
        self.wait_started()
        result = bag.stop()
        self.assertEqual([s[0] for s in result['signals']], ['SIGINT', 'SIGTERM'])
        self.assertGreaterEqual(result['signals'][1][1], 0.5)
        self.assertTrue(result['metadata']['finalized'])

    def test_sigkill_when_both_are_ignored(self):
        self.env['FAKE_IGNORE'] = 'INT,TERM'
        bag = self.bag()
        bag.start()
        self.wait_started()
        result = bag.stop()
        self.assertEqual([s[0] for s in result['signals']], ['SIGINT', 'SIGTERM', 'SIGKILL'])
        self.assertLess(result['exit_code'], 0)
        self.assertFalse(result['metadata']['finalized'])
        self.assertEqual(result['metadata']['error'], 'metadata_missing')

    def test_process_that_exits_by_itself(self):
        self.env['FAKE_FAIL'] = '1'
        bag = self.bag()
        bag.start()
        deadline = time.monotonic() + 10
        while not bag.poll():
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.02)
        result = bag.stop()
        self.assertEqual((result['exit_code'], result['signals'], result['metadata']['finalized']), (2, [], False))

    def test_stop_right_after_start(self):
        bag = self.bag()
        bag.start()
        result = bag.stop()                      # possibly before the fake installed its handlers
        self.assertIsNotNone(result['exit_code'])
        self.assertFalse(bag.running())

    def test_existing_output_folder_is_refused(self):
        (self.segment / 'bag').mkdir()
        with self.assertRaises(bag_mod.BagRecordError):
            self.bag().start()

    @unittest.skipUnless(bag_mod.pdeathsig_prefix(), 'util-linux setpriv with --pdeathsig not found')
    def test_orphan_guard_stops_the_bag_when_the_parent_dies(self):
        parent = self.base / 'parent.py'
        parent.write_text(PARENT, encoding='utf-8')
        out = subprocess.run([sys.executable, '-B', str(parent), str(SIM_ROOT), str(self.segment / 'bag'),
                              str(self.fake)], capture_output=True, text=True, timeout=30)
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertEqual(out.stdout.strip(), 'setpriv --pdeathsig INT')
        deadline = time.monotonic() + 10
        while not (self.segment / 'bag' / 'metadata.yaml').exists():
            self.assertLess(time.monotonic(), deadline, 'the orphaned bag process did not get SIGINT')
            time.sleep(0.05)
        self.assertTrue(bag_mod.read_metadata(self.segment / 'bag')['finalized'])
        pid = int((self.segment / 'fake_started').read_text())
        deadline = time.monotonic() + 10
        while os.path.exists('/proc/%d' % pid) and 'Z' not in open('/proc/%d/stat' % pid).read().split()[2]:
            self.assertLess(time.monotonic(), deadline, 'orphaned bag still running')
            time.sleep(0.05)


@posix_only
class ControllerWithRealBagTest(unittest.TestCase):
    def test_recording_with_the_bag_process(self):
        from test_steering_sidecar import FakeClock, FakeRecorder, host_profile_bytes, message
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            fake = base / 'fake_ros2.py'
            fake.write_text(FAKE_ROS2, encoding='utf-8')
            profile = base / 'host_profile.json'
            profile.write_bytes(host_profile_bytes())
            sha = sc.HostCalibration(host_profile_bytes()).sha256
            recorder = FakeRecorder(base / 'recordings')
            clock = FakeClock(wall=time.time(), steady=time.monotonic())

            def factory(output_dir, log_path):
                return bag_mod.BagRecord(output_dir, ros2=[sys.executable, str(fake)], guard=[], log_path=log_path,
                                         stop_timeout_s=2.0, kill_after_s=1.0)
            controller = sc.SidecarController(base / 'recordings', host_profile_path=str(profile),
                                              bag_factory=factory, clock=clock.clock, steady=time.monotonic,
                                              sleep=time.sleep)
            rid = '20261109T010203Z_both'
            recorder.start(rid, clock.wall)
            controller.on_status(recorder.status('recording', rid))
            self.assertIsNotNone(controller.segment.bag.pid)
            for i in range(10):
                clock.advance(0.05)
                controller.on_message(json.dumps(message(i, 50 * i, received_at=clock.wall - 0.01, sha=sha)))
            deadline = time.monotonic() + 10
            while not (controller.segment.dir / 'fake_started').exists():
                self.assertLess(time.monotonic(), deadline)
                time.sleep(0.02)
            controller.on_status(recorder.status('finalizing', rid))
            recorder.archive_rec(rid, 0.6)
            controller.on_status(recorder.status('idle'))
            deadline = time.monotonic() + 10
            while controller.closing:
                self.assertLess(time.monotonic(), deadline)
                controller.tick()
                time.sleep(0.02)
            check = sc.check_recording(base / 'recordings' / rid)
            self.assertTrue(check['passed'], check['segments'][0]['problems'])
            seg = check['segments'][0]
            self.assertEqual((seg['frame_count'], seg['bag']['message_count'], seg['bag']['exit_code']), (10, 7, 0))
            pid_file = json.loads((base / 'recordings' / rid / 'steering' / sc.BAG_PID).read_bytes())
            self.assertIn(str(base / 'recordings' / rid / 'steering' / 'bag'), pid_file['command'])


if __name__ == '__main__':
    unittest.main()
