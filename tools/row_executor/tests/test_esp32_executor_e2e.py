"""End to end in one process: the row executor (ExecutorLoop, SteerBridgeClient rules, Esp32AbsHoldAdapter) -> the
dashboard's ESP32-S3 bridge (scripts/esp32_steer_bridge.Esp32Bridge + scripts/esp32_abs_hold.Esp32SteerMotor, unchanged)
-> the fake ESP32-S3 firmware (dashboard tests/fake_esp32.py, written from BTS7960_Serial_Control.ino), on one virtual
clock (docs/steering/EXECUTOR_ESP32_KO.md 6).

Needs the dashboard repository with the ESP32 bridge: UBTS_DASH_DIR=<dashboard checkout> (rowexec_testlib.dash_dir);
skipped otherwise. Every request and reply goes through JSON as on the control socket. The bridge handles queued
requests in its 20 ms step like its loop thread. Run alone (it imports the dashboard's 'scripts' package):
    UBTS_DASH_DIR=<dashboard> python -B -m pytest -q -p no:cacheprovider tools/row_executor/tests/test_esp32_executor_e2e.py
Checked: no drive line before an armed run's go, the arm precheck (drive direction verified, firmware drive watchdog),
the steering sign and scale (executor left + -> firmware ANGLE -, measured back), the ANGLE policy, the page STOP, the
operator heartbeat loss (drive 0, no restart), the executor stopping (the bridge input deadline), the steering
not-following latch, an invalid potentiometer and that op motion never reaches the bridge.
Test numbers only (the motor values are test values, not field measurements); no plant: the scene is static.
"""
import concurrent.futures
import importlib.util
import json
import math
import os
import types
import unittest
from dataclasses import replace

import rowexec_testlib as L
from row_executor import config as C
from row_executor import executor_main as M
from row_executor.bridge_client import ARMED, LATCHED, Esp32AbsHoldAdapter, Esp32SteerBridgeClient
from row_executor.core import ACTIVE, ENDED, PAUSED, Runner
from test_executor_loop import imu_msg, scan_msg
from test_steer_executor import SCENE

SESSION = 'esp32-screen-0001'
WALL0 = 1.7e9
PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Test numbers for the motor side (not field values): the vehicle config of the kit plus feasible motor values.
MOTOR_TEST = dict(motor_output_enabled=True, tuning_file=None, breakaway_duty_pct=20, v_measured_m_s=.42,
                  measured_stop_distance_m=.2, steer_measured_rate_deg_s=5.094,
                  steer_measured_rate_standstill_deg_s=5.094, steer_rate_min_deg_s=4.0, steer_range_measured=True,
                  steer_wheel_soft_margin_deg=3., steer_min_pwm_pct=10., steer_zero_offset_deg=0.,
                  steer_polarity_checked=True, steer_esp32_require_fw_watchdog=False)


def dashboard():
    """(esp32_abs_hold, esp32_steer_bridge, fake_esp32) of the dashboard checkout, or a skip reason."""
    try:
        root = L.dash_dir()
    except RuntimeError as exc:
        return None, str(exc)
    if not os.path.isfile(os.path.join(root, 'scripts', 'esp32_steer_bridge.py')):
        return None, '%s has no scripts/esp32_steer_bridge.py (UBTS_DASH_DIR: the dashboard repository)' % root
    esp = L.import_dash('esp32_abs_hold')
    bridge = L.import_dash('esp32_steer_bridge')
    spec = importlib.util.spec_from_file_location('ubts_dash_fake_esp32', os.path.join(root, 'tests', 'fake_esp32.py'))
    fake = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fake)
    return (esp, bridge, fake), None


DASH, SKIP = dashboard()


def vehicle_config(**over):
    cfg = C.load_config(os.path.join(PACKAGE, 'config.vehicle_esp32_shadow.json'))
    values = dict(MOTOR_TEST)
    values.update(over)
    return replace(cfg, **values).validate()


class Clock:
    def __init__(self, t=5000.):
        self.t = t

    def __call__(self):
        return self.t


class Steady:
    """Controller stand-in: FOLLOW at a throttle while the steering ramps from the measured angle to goal_mrad at
    rate_mrad_s (inside the executor's slew limit). goal None: hold the first command."""

    def __init__(self, throttle=30, goal_mrad=0, rate_mrad_s=40.):
        self.throttle, self.goal, self.rate = throttle, goal_mrad, rate_mrad_s
        self.value = None
        self.t = None
        self.state, self.reason = 'FOLLOW', 'start'

    def step(self, t, s):
        if self.value is None:
            start = s.get('steer_command_mrad')
            if start is None:
                start = s.get('steer_measured_mrad') or 0.
            self.value = float(start)
            self.t = t
        dt, self.t = t - self.t, t
        if self.goal is not None:
            delta = self.goal - self.value
            self.value += max(-self.rate * dt, min(self.rate * dt, delta))
        steer = int(round(self.value))
        return types.SimpleNamespace(state='FOLLOW', reason='stub', throttle=self.throttle, steer_mrad=steer,
                                     left=self.throttle, right=self.throttle)

    def current_phase(self):
        return 'FOLLOW'

    def telemetry(self):
        return {}


class Lockstep:
    """The control socket in one process: JSON in and out; status / stop answered at once (they bypass the bridge
    queue), every other request queued and handled by one bridge step (one request per 20 ms step, as the loop)."""

    def __init__(self, rig):
        self.rig = rig
        self.ops = []

    def __call__(self, request):
        request = json.loads(json.dumps(request, allow_nan=False))
        self.ops.append(request.get('op'))
        bridge = self.rig.bridge
        if request.get('op') in ('status', 'stop'):
            reply = bridge.request(request)
        else:
            future = concurrent.futures.Future()
            bridge.pending.put_nowait((self.rig.clock(), request, future))
            bridge.step()
            reply = future.result(timeout=0)
        return json.loads(json.dumps(reply, allow_nan=False, default=str))


class Esp32Rig:
    """ExecutorLoop (front_steer MOTOR, one_press, ESP32 bridge) + the dashboard ESP32 bridge + the fake board."""

    def __init__(self, cfg=None, controller=None, direction_verified=True, fake_options=None, **motor_options):
        esp, bridge_mod, fake_mod = DASH
        self.fake_mod = fake_mod
        self.clock = Clock()
        options = dict(pot=float(fake_mod.CENTER))
        options.update(fake_options or {})
        self.fake = fake_mod.FakeEsp32(self.clock, **options)
        self.tr = fake_mod.FakeTransport(self.fake)
        self.motor = esp.Esp32SteerMotor(self.tr, self.clock, direction_verified=direction_verified, **motor_options)
        self.bridge = bridge_mod.Esp32Bridge(self.motor, self.clock, wall_clock=lambda: WALL0 + self.clock.t)
        self.socket = Lockstep(self)
        self.cfg = cfg or vehicle_config()
        self.log = M._NullLog()
        self.controller = controller or Steady()
        self.loop = M.ExecutorLoop(self.cfg, motor_output_flag=True, clock=self.clock, wall=lambda: WALL0 + self.clock.t,
                                   stop_transport=self.socket, log=self.log,
                                   runner_factory=lambda c: Runner(c, controller_factory=lambda: self.controller))
        self.next_imu = self.clock.t
        self.next_scan = self.clock.t + .013
        self.next_bridge = self.clock.t
        self.next_exec = self.clock.t
        self.visible = True
        self.executor_running = True
        self.run(2.5)                      # the bridge opens the port and confirms the board

    def run(self, seconds):
        end = self.clock.t + seconds - 1e-9
        out = []
        while self.clock.t < end:
            self.clock.t = round(self.clock.t + .01, 6)
            if self.clock.t + 1e-9 >= self.next_bridge:
                self.bridge.step()
                self.next_bridge += .02
            while min(self.next_imu, self.next_scan) <= self.clock.t + 1e-9:
                if self.next_imu <= self.next_scan:
                    self.loop.on_imu_msg(self.next_imu, imu_msg(self.next_imu - .01))
                    self.next_imu += 1 / 33.
                else:
                    self.loop.on_scan_msg(self.next_scan, scan_msg(self.next_scan - .1, SCENE))
                    self.next_scan += .1
            if self.executor_running and self.clock.t + 1e-9 >= self.next_exec:
                if self.visible:
                    self.loop.operator_heartbeat(SESSION)
                out.append(self.loop.step())
                self.next_exec += .05
        return out

    def request(self, kind, **kw):
        future = self.loop.submit(kind, **kw)
        self.run(.06)
        return future.result()

    def armed(self, test):
        test.assertTrue(self.request('start', row_len_m=None)['ok'])
        reply = self.request('arm')
        test.assertTrue(reply['ok'], reply)
        self.run(2.)                       # inputs fresh for fresh_recover_s, corridor acquired
        return reply

    def running(self, test):
        self.armed(test)
        reply = self.request('go', session=SESSION)
        test.assertTrue(reply['ok'], reply)
        self.run(.5)
        test.assertEqual(self.loop.runner.state, ACTIVE)

    # --- views ---------------------------------------------------------------------------------------------------
    def board_lines(self, since_ms=None, prefix=''):
        return [(ms, line) for ms, line in self.fake.received
                if (since_ms is None or ms >= since_ms) and line.startswith(prefix)]

    def drive_lines(self, since_ms=None):
        """Drive commands with a non-zero PWM ('2 F 77', '3 R 50')."""
        return [(ms, line) for ms, line in self.board_lines(since_ms)
                if line[:2] in ('2 ', '3 ') and line.split()[1] in ('F', 'R') and int(line.split()[2]) > 0]

    def now_ms(self):
        return int(self.clock.t * 1000)

    def events(self, name):
        return [r for r in self.log.records if r.get('event') == name]


@unittest.skipIf(DASH is None, SKIP or '')
class ArmAndStart(unittest.TestCase):
    def test_no_drive_line_before_the_go_and_none_in_shadow(self):
        rig = Esp32Rig()
        self.assertEqual(rig.motor.link, 'ready')
        self.assertTrue(rig.request('start', row_len_m=None)['ok'])
        rig.run(1.)
        self.assertEqual(rig.drive_lines(), [])          # a run without an arm sends nothing
        rig.request('arm')
        rig.run(3.)                                      # armed and waiting for the go: zero drives only
        self.assertEqual(rig.loop.bridge.state, ARMED)
        self.assertEqual(rig.drive_lines(), [])
        self.assertEqual(rig.fake.drive(), {2: 0, 3: 0})
        self.assertEqual(rig.loop.status()['run_state'], 'armed')
        # SHADOW: the same configuration without the flag builds no transport at all
        shadow = M.ExecutorLoop(vehicle_config(), motor_output_flag=False, clock=rig.clock)
        self.assertIsNone(shadow._stop_transport)
        self.assertIsNone(shadow.esp32)

    def test_arm_needs_the_verified_drive_direction(self):
        rig = Esp32Rig(direction_verified=False)
        rig.request('start', row_len_m=None)
        reply = rig.request('arm')
        self.assertFalse(reply['ok'])
        self.assertEqual(reply['error'], 'arm_precheck:drive_direction_unverified')
        self.assertNotIn('arm', rig.socket.ops)          # the precheck refusal sends no arm
        self.assertEqual(rig.drive_lines(), [])
        self.assertEqual(rig.board_lines(prefix='ANGLE'), [])

    def test_arm_needs_a_firmware_drive_watchdog_when_required(self):
        rig = Esp32Rig(cfg=vehicle_config(steer_esp32_require_fw_watchdog=True))
        rig.request('start', row_len_m=None)
        reply = rig.request('arm')
        self.assertEqual(reply['error'], 'arm_precheck:firmware_drive_watchdog_missing')
        # the proposed firmware watchdog (FW line DRIVE_WD=300): the same arm passes
        rig = Esp32Rig(cfg=vehicle_config(steer_esp32_require_fw_watchdog=True),
                       fake_options=dict(drive_watchdog_ms=300))
        rig.request('start', row_len_m=None)
        self.assertTrue(rig.request('arm')['ok'])

    def test_arm_needs_the_configured_cal_line(self):
        rig = Esp32Rig(cfg=vehicle_config(steer_esp32_cal=(1680, 1900, 2180, 20)))
        rig.request('start', row_len_m=None)
        self.assertEqual(rig.request('arm')['error'], 'arm_precheck:calibration_mismatch')


@unittest.skipIf(DASH is None, SKIP or '')
class Driving(unittest.TestCase):
    def test_drive_steering_sign_and_scale(self):
        goal = 100                                        # equivalent mrad, + left (5.73 deg)
        rig = Esp32Rig(controller=Steady(throttle=30, goal_mrad=goal, rate_mrad_s=40.))
        rig.running(self)
        rig.run(3.5)                                      # (the static scene ends a run after about 6 s: no response)
        drives = rig.drive_lines()                        # a drive value is written when it changes
        pwm = (30 * 255 + 50) // 100
        self.assertIn('2 F %d' % pwm, [line for _, line in drives])
        self.assertIn('3 F %d' % pwm, [line for _, line in drives])
        angles = [float(line.split()[1]) for _, line in rig.board_lines(None, 'ANGLE')]
        self.assertTrue(angles and all(a <= 0 for a in angles), angles)        # left = firmware minus
        scale = 20. / rig.cfg.steer_esp32_mark_deg
        expected = -math.degrees(goal / 1000.) * scale                          # -6.43 command deg
        self.assertAlmostEqual(angles[-1], round(expected), delta=.01)
        self.assertGreater(rig.fake.pot, rig.fake_mod.CENTER + 40)              # the pot moved to the left
        measured = rig.loop.runner.steer_monitor.measured_mrad()
        self.assertAlmostEqual(measured, goal, delta=25.)                        # back in equivalent mrad, + left
        adapter = rig.loop.esp32
        self.assertLessEqual(adapter.counts['target_changes'], 4 + 2)            # about one target per second
        self.assertEqual(rig.loop.runner.state, ACTIVE)
        self.assertNotIn('motion', rig.socket.ops)                               # op motion stays in the executor

    def test_page_stop(self):
        rig = Esp32Rig()
        rig.running(self)
        self.assertNotEqual(rig.fake.drive(), {2: 0, 3: 0})
        t_stop = rig.now_ms()
        reply = rig.loop.emergency_stop('operator_page')
        self.assertTrue(reply['stop_sent'])
        rig.run(.04)
        self.assertEqual(rig.fake.drive(), {2: 0, 3: 0})
        self.assertTrue(rig.board_lines(t_stop, 'STOP'))
        rig.run(1.5)
        runner = rig.loop.runner
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'operator_stop'))
        self.assertEqual(rig.loop.bridge.state, LATCHED)
        self.assertEqual(rig.drive_lines(t_stop + 30), [])
        self.assertEqual(len(rig.events('stop_confirmed')), 1)
        # no automatic restart: Start without an acknowledgement and a new Arm sends nothing
        rig.request('start', row_len_m=None)
        rig.request('go', session=SESSION)
        rig.run(1.)
        self.assertEqual(rig.drive_lines(t_stop + 30), [])

    def test_operator_heartbeat_loss_pauses_without_restart(self):
        rig = Esp32Rig()
        rig.running(self)
        rig.visible = False                               # the page stops its heartbeats
        t_lost = rig.now_ms()
        rig.run(1.4)
        runner = rig.loop.runner
        self.assertEqual((runner.state, runner.reason), (PAUSED, 'operator_link_lost'))
        self.assertEqual(rig.fake.drive(), {2: 0, 3: 0})
        zero = [ms for ms, line in rig.board_lines(t_lost) if line in ('2 S', '3 S')]
        self.assertTrue(zero)
        self.assertLessEqual(zero[0] - t_lost, int((rig.cfg.operator_timeout_s + .1 + .05) * 1000))
        rig.visible = True                                # the heartbeats come back: still paused
        t_back = rig.now_ms()
        rig.run(2.)
        self.assertEqual(runner.state, PAUSED)
        self.assertEqual(rig.drive_lines(t_back), [])
        self.assertEqual(rig.loop.bridge.state, ARMED)    # zero drives keep the lease (a new go may resume)

    def test_executor_stops_and_the_bridge_input_deadline_stops_the_drive(self):
        rig = Esp32Rig()
        rig.running(self)
        rig.executor_running = False                      # the executor process is gone (no drive, no STOP)
        t_dead = rig.now_ms()
        rig.run(1.2)
        self.assertEqual(rig.fake.drive(), {2: 0, 3: 0})
        zero = [ms for ms, line in rig.board_lines(t_dead) if line in ('STOP', '2 S', '3 S')]
        self.assertTrue(zero)
        self.assertLessEqual(zero[0] - t_dead, 850)       # 0.75 s input deadline + one bridge step + one drive period
        self.assertFalse(rig.bridge.control.lease)

    def test_steering_that_does_not_follow_latches_and_stops(self):
        rig = Esp32Rig(controller=Steady(throttle=30, goal_mrad=160, rate_mrad_s=60.), fake_options=dict(stuck=True))
        rig.running(self)
        rig.run(6.)
        runner = rig.loop.runner
        self.assertEqual((runner.state, runner.end_reason), (ENDED, 'steering_not_following'))
        self.assertIn('steering_not_following', rig.loop.esp32.latches)
        self.assertEqual(rig.fake.drive(), {2: 0, 3: 0})
        self.assertFalse(rig.bridge.control.lease)

    def test_invalid_potentiometer_ends_the_run(self):
        rig = Esp32Rig()
        rig.running(self)
        rig.fake.pot_invalid = True
        rig.run(1.)
        runner = rig.loop.runner
        self.assertEqual(runner.state, ENDED)
        self.assertIn(runner.end_reason, ('steering_feedback_invalid', 'steering_feedback_stale'))
        self.assertEqual(rig.fake.drive(), {2: 0, 3: 0})


class AdapterOnly(unittest.TestCase):
    def test_client_type(self):
        cfg = vehicle_config()
        adapter = Esp32AbsHoldAdapter.from_config(cfg, lambda request: {'ok': True})
        client = Esp32SteerBridgeClient.from_config(cfg, adapter)
        self.assertEqual((client.limit_left_mrad, client.expect_cal), (281, (1681, 1900, 2180, 20)))


if __name__ == '__main__':
    unittest.main()
