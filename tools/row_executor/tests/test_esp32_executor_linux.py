"""End to end on Linux in real time: the row executor (ExecutorLoop + Esp32SteerBridgeClient + Esp32AbsHoldAdapter over a
real Unix socket) -> the dashboard's ESP32 bridge PROCESS (python -m scripts.esp32_steer_bridge, unchanged: real USB
transport code with a fake udev identity, flock, TIOCEXCL, termios) -> a pty whose other end is the fake ESP32-S3
firmware (dashboard tests/fake_esp32.py) in a thread (docs/steering/EXECUTOR_ESP32_KO.md 6).

Needs Linux (pty, Unix sockets) and UBTS_DASH_DIR=<dashboard checkout with scripts/esp32_steer_bridge.py>; skipped
otherwise. The fake board is the vehicle's board: the runtime-calibration SMOOTH firmware with the configured CAL marks
in its NVS (test_esp32_executor_e2e.board_options; dashboard da39e8a or later). Checked: arm -> go drives the board and steers to the left with a
negative ANGLE; the page STOP zeroes the drive and ends the run operator_stop; when the executor stops stepping
(killed) the bridge's input deadline zeroes the drive; SIGTERM of the bridge zeroes the drive and the executor ends
the run on the lost socket. Test numbers only; static scene.
Run alone (it imports the dashboard's 'scripts' package):
    UBTS_DASH_DIR=<dashboard> python3 -B -m pytest -q -p no:cacheprovider tools/row_executor/tests/test_esp32_executor_linux.py
"""
import importlib.util
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from dataclasses import replace

import rowexec_testlib as L
from row_executor import executor_main as M
from row_executor.bridge_client import LATCHED
from row_executor.core import ACTIVE, ENDED, Runner
from test_esp32_executor_e2e import SESSION, Steady, board_options, fake_board_problem, vehicle_config
from test_executor_loop import imu_msg, scan_msg
from test_steer_executor import SCENE

LINUX = sys.platform.startswith('linux')


def dashboard():
    if not LINUX:
        return None, None, 'Linux only: pty, fcntl, termios, Unix sockets'
    try:
        root = L.dash_dir()
    except RuntimeError as exc:
        return None, None, str(exc)
    if not os.path.isfile(os.path.join(root, 'scripts', 'esp32_steer_bridge.py')):
        return None, None, '%s has no scripts/esp32_steer_bridge.py (UBTS_DASH_DIR)' % root
    spec = importlib.util.spec_from_file_location('ubts_dash_fake_esp32_rt', os.path.join(root, 'tests', 'fake_esp32.py'))
    fake = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fake)
    problem = fake_board_problem(fake, root)
    if problem:
        return None, None, problem
    return root, fake, None


ROOT, FAKE, SKIP = dashboard()


class PtyBoard:
    """The fake firmware on a pty master, in real time (a thread); as the dashboard's tests/test_esp32_bridge_linux."""

    def __init__(self):
        import pty
        import tty
        self.master, self.slave = pty.openpty()
        tty.setraw(self.master)
        self.path = os.ttyname(self.slave)
        self.fake = FAKE.FakeEsp32(time.monotonic, reset_on_open=False, **board_options())
        self.fake.boot()
        self.fake.out.clear()
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        import select
        while self.running:
            ready = select.select([self.master], [], [], .002)[0]
            with self.lock:
                if ready:
                    try:
                        data = os.read(self.master, 4096)
                    except OSError:
                        data = b''
                    if data:
                        self.fake.feed(data)
                self.fake.advance_to_now()
                out = self.fake.read_output()
            if out:
                try:
                    os.write(self.master, out)
                except OSError:
                    pass

    def drive(self):
        with self.lock:
            self.fake.advance_to_now()
            return self.fake.drive()

    def received(self):
        with self.lock:
            return list(self.fake.received)

    def close(self):
        self.running = False
        self.thread.join(1)
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


class Executor:
    """ExecutorLoop at 20 Hz in a thread with synthetic scans (10 Hz) and IMU (33 Hz) on the monotonic clock and the
    operator page's heartbeats; step() can be suspended (a killed executor sends nothing more)."""

    def __init__(self, cfg, controller):
        self.log = M._NullLog()
        self.loop = M.ExecutorLoop(cfg, motor_output_flag=True, log=self.log,
                                   runner_factory=lambda c: Runner(c, controller_factory=lambda: controller))
        self.stepping = True
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        now = time.monotonic()
        next_imu, next_scan, next_step = now, now + .013, now
        while not self.stop.is_set():
            now = time.monotonic()
            while min(next_imu, next_scan) <= now:
                if next_imu <= next_scan:
                    self.loop.on_imu_msg(next_imu, imu_msg(next_imu - .01))
                    next_imu += 1 / 33.
                else:
                    self.loop.on_scan_msg(next_scan, scan_msg(next_scan - .1, SCENE))
                    next_scan += .1
            if now >= next_step:
                if self.stepping:
                    self.loop.operator_heartbeat(SESSION)
                    self.loop.step()
                next_step += .05
            time.sleep(.005)

    def request(self, kind, **kw):
        return self.loop.submit(kind, **kw).result(timeout=5)

    def close(self):
        self.stop.set()
        self.thread.join(2)
        self.loop.shutdown('test_end')


@unittest.skipIf(ROOT is None, SKIP or '')
class RealTime(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = self.tmp.name
        self.board = PtyBoard()
        info = os.stat(self.board.path)
        udev = os.path.join(self.dir, 'udev')
        os.makedirs(udev)
        with open(os.path.join(udev, 'c%d:%d' % (os.major(info.st_rdev), os.minor(info.st_rdev))), 'w') as handle:
            handle.write('E:ID_SERIAL=Fake_ESP32\n')
        self.sock = os.path.join(self.dir, 'run', 'control.sock')
        env = dict(os.environ, PYTHONPATH=ROOT, PYTHONDONTWRITEBYTECODE='1', ROVER_ESP32_DIRECTION_VERIFIED='1')
        self.proc = subprocess.Popen(
            [sys.executable, '-m', 'scripts.esp32_steer_bridge', '--device', self.board.path, '--identity', 'Fake_ESP32',
             '--udev-root', udev, '--lock-dir', self.dir, '--socket', self.sock, '--no-ros', '--boot-wait-s', '0.3'],
            cwd=ROOT, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.executor = None
        from row_executor.bridge_client import UnixSocketTransport
        probe = UnixSocketTransport(self.sock, .5)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if os.path.exists(self.sock):
                try:
                    if probe({'op': 'status'}).get('connected'):
                        break
                except Exception:
                    pass
            time.sleep(.05)
        else:
            self.fail('bridge not ready')

    def tearDown(self):
        if self.executor is not None:
            self.executor.close()
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
            try:
                self.proc.wait(5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(5)
        self.proc.stderr.close()
        self.board.close()
        self.tmp.cleanup()

    def start(self, goal_mrad=100):
        cfg = vehicle_config(socket_path=self.sock)
        self.executor = Executor(cfg, Steady(throttle=30, goal_mrad=goal_mrad, rate_mrad_s=40.))
        ex = self.executor
        self.assertTrue(ex.request('start', row_len_m=None)['ok'])
        reply = ex.request('arm')
        self.assertTrue(reply['ok'], reply)
        time.sleep(2.2)
        self.assertEqual(self.board.drive(), {2: 0, 3: 0})          # armed, no go: nothing drives
        reply = ex.request('go', session=SESSION)
        self.assertTrue(reply['ok'], reply)
        time.sleep(1.)
        self.assertEqual(ex.loop.runner.state, ACTIVE)
        self.assertNotEqual(self.board.drive(), {2: 0, 3: 0})
        return ex

    def until(self, predicate, limit):
        t0 = time.monotonic()
        while time.monotonic() - t0 < limit:
            if predicate():
                return time.monotonic() - t0
            time.sleep(.005)
        return None

    def test_drive_steer_and_page_stop(self):
        ex = self.start(goal_mrad=100)
        time.sleep(2.)
        angles = [line for _, line in self.board.received() if line.startswith('ANGLE')]
        self.assertTrue(angles and all(float(a.split()[1]) <= 0 for a in angles), angles)   # left = firmware minus
        self.assertGreater(self.board.fake.pot, vehicle_config().steer_esp32_cal[1] + 20)
        t0 = time.monotonic()
        reply = ex.loop.emergency_stop('operator_page')
        self.assertTrue(reply['stop_sent'])
        waited = self.until(lambda: self.board.drive() == {2: 0, 3: 0}, 1.)
        self.assertIsNotNone(waited)
        self.assertLess(waited, .2)
        time.sleep(1.2)
        runner = ex.loop.runner
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'operator_stop'))
        self.assertEqual(ex.loop.bridge.state, LATCHED)
        self.assertEqual(self.board.drive(), {2: 0, 3: 0})
        self.assertLess(time.monotonic() - t0, 5.)

    def test_killed_executor_input_deadline(self):
        ex = self.start(goal_mrad=0)
        ex.stepping = False                                          # no drive, no STOP, no heartbeat any more
        waited = self.until(lambda: self.board.drive() == {2: 0, 3: 0}, 2.)
        self.assertIsNotNone(waited)
        self.assertLess(waited, 1.0)                                 # bridge input deadline 0.75 s (+ loop and USB)

    def test_bridge_sigterm_stops_the_drive_and_ends_the_run(self):
        ex = self.start(goal_mrad=0)
        self.proc.send_signal(signal.SIGTERM)
        waited = self.until(lambda: self.board.drive() == {2: 0, 3: 0}, 2.)
        self.assertIsNotNone(waited)
        self.proc.wait(5)
        time.sleep(.6)
        runner = ex.loop.runner
        self.assertEqual(runner.state, ENDED)
        self.assertTrue(runner.end_reason.startswith('bridge'), runner.end_reason)


if __name__ == '__main__':
    unittest.main()
