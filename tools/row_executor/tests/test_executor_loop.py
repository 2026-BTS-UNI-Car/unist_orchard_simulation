"""ExecutorLoop (executor_main) on a virtual clock: shadow isolation, hold/STOP/arm rules, bridge output
cadence, stall watchdog, latest-scan-only feeding, scan stamps, near-return confirmation, geometry guard,
on-site parameters and the source manifest. The bridge side is the REAL dashboard RoverControl
(scripts/rover_control_core.py) behind the real BridgeClient (CoreTransport, in-process, no socket).

Day-1 review fixes (field autonomy work folder of 2026-10-22, review_*_day1): S3/F8 press ids (HoldAliasTest,
HoldStateTest), S7 no stop after a lost lease (MotorTest), S8 log writer failures and loop exceptions
(LogWriterTest, LoopExceptionTest), S9 status says what STOP does (StopMeaningTest).

Steering branch import (docs/steering/EXECUTOR_IMPORT_KO.md): replay_adapters.py is not imported; the near-return
filter cross-check compares with a frozen copy of its NearTransientFilter (546fa8c) below.
"""
import errno
import math
import os
import shutil
import sys
import tempfile
import threading
import time
import types
import unittest

import numpy as np

import rowexec_testlib as L
from row_executor import config as C
from row_executor import core
from row_executor import executor_main as M
from row_executor.bridge_client import ARMED, DISARMED, LATCHED, BridgeClient
from row_executor.config import ExecutorConfig
from row_executor.core import ACTIVE, ENDED, PAUSED, NearConfirmFilter, Runner, ScanStamper
from test_bridge_client import CoreTransport, FakeMotor, RoverControl

SCENE = L.raw_scan(L.orchard_trunks())


class ReplayNearTransientFilter:
    """Frozen copy of replay_adapters.NearTransientFilter at 546fa8c (the evaluated replay variant; the replay tools are
    not part of this import): the reference of test_near_filter_matches_the_evaluated_replay_filter."""

    def __init__(self, near_m=2., tol_m=.35, confirm_scans=1, tol_step_m=.1):
        from scipy.spatial import cKDTree
        self._tree = cKDTree
        self.near_m, self.tol_m = float(near_m), float(tol_m)
        self.confirm_scans, self.tol_step_m = int(confirm_scans), float(tol_step_m)
        self.history = []

    def apply(self, r, angle_min, increment):
        idx = np.nonzero(np.isfinite(r) & (r < self.near_m))[0]
        a = angle_min + idx * increment
        pts = np.column_stack((r[idx] * np.cos(a), r[idx] * np.sin(a)))
        removed = 0
        if len(self.history) >= self.confirm_scans and len(idx):
            keep = np.ones(len(idx), dtype=bool)
            for age, prev in enumerate(self.history[:self.confirm_scans]):
                if not len(prev):
                    keep[:] = False
                    break
                d, _ = self._tree(prev).query(pts, distance_upper_bound=self.tol_m + age * self.tol_step_m)
                keep &= np.isfinite(d)
            removed = int((~keep).sum())
            r[idx[~keep]] = np.inf
        self.history.insert(0, pts)
        del self.history[self.confirm_scans:]
        return removed


def header(t):
    sec = int(math.floor(t))
    return types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec, nanosec=int(round((t - sec) * 1e9))))


def scan_msg(t_header, ranges=SCENE):
    return types.SimpleNamespace(header=header(t_header), ranges=ranges, angle_min=L.RAW_AMIN,
                                 angle_increment=L.RAW_INC, range_min=.05, range_max=30.)


def imu_msg(t, gz=0.):
    v = types.SimpleNamespace
    return v(header=header(t), angular_velocity=v(x=0., y=0., z=gz), linear_acceleration=v(x=0., y=0., z=9.81))


def legacy_sensor_config(**kwargs):
    """Explicit historical sensor-only fixture; newest production requires native admission."""
    kwargs.setdefault('require_native_navigation', False)
    kwargs.setdefault('distance_mode', 'scan_norm_legacy')
    return C.ExecutorConfig(**kwargs)


def motor_config(**over):
    """A feasible motor configuration (example numbers for the tests, not field values)."""
    # steering GO rule (field decision rules, 2026-10-22): row motor output needs max_differential 10-20 and a measured
    # authority >= 4.0 deg/s; base 60 keeps the inner wheel at breakaway 40 + margin 10 with differential 10
    base = dict(motor_output_enabled=True, base_duty_pct=60, reduce_duty_pct=60, max_differential_pct=10,
                duty_cap_pct=70, diff_cap_pct=20, breakaway_duty_pct=40, inner_margin_pct=10,
                v_measured_m_s=.48, measured_yaw_at_max_diff_deg_s=4.2, max_yaw_rate_deg_s=2.5,
                measured_stop_distance_m=.25)       # C7 (day-2 N8: required for row motor output)
    base.update(over)
    return legacy_sensor_config(**base).validate()


class Clock:
    def __init__(self, t=5000.):
        self.t = t

    def __call__(self):
        return self.t


class Rig:
    """ExecutorLoop + virtual clock + sensor feeder (+ the real bridge core when MOTOR)."""

    def __init__(self, cfg=None, motor=False, scene=SCENE, **kw):
        self.clock = Clock()
        self.cfg = cfg or legacy_sensor_config()
        self.motor = FakeMotor()
        self.control = RoverControl(self.motor, self.clock)
        self.transport = CoreTransport(self.control)
        self.clients = []

        def factory():
            client = BridgeClient(owner='row-executor:test', transport=self.transport)
            self.clients.append(client)
            return client
        self.loop = M.ExecutorLoop(self.cfg, motor_output_flag=motor, clock=self.clock, wall=lambda: 1.7e9,
                                   bridge_factory=factory, stop_transport=self.transport, **kw)
        self.scene = scene
        self.next_imu = self.clock.t
        self.next_scan = self.clock.t + .013
        self.scan_on = True
        self.hold_press = None

    def run(self, seconds, hold=None):
        """Advance; hold: None (no change), True (heartbeat each tick with the current press), False."""
        out = []
        steps = int(round(seconds / .05))
        for _ in range(steps):
            target = self.clock.t + .05
            while min(self.next_imu, self.next_scan) <= target:
                if self.next_imu <= self.next_scan:
                    self.clock.t = max(self.clock.t, self.next_imu)        # the clock never goes back
                    self.loop.on_imu_msg(self.next_imu, imu_msg(self.next_imu - .01))
                    self.next_imu += 1 / 33.
                else:
                    self.clock.t = max(self.clock.t, self.next_scan)
                    if self.scan_on and self.scene is not None:
                        self.loop.on_scan_msg(self.next_scan, scan_msg(self.next_scan - .1, self.scene))
                    self.next_scan += .1
            self.clock.t = target
            if hold is True:
                self.loop.hold_beat(self.hold_press)
            out.append(self.loop.step())
            self.control.tick()
        return out

    def press(self, name):
        self.hold_press = name
        self.loop.hold_beat(name)

    def release(self):
        self.loop.hold_release(self.hold_press)
        self.hold_press = None


class ShadowTest(unittest.TestCase):
    def test_shadow_has_no_bridge_and_no_transport(self):
        rig = Rig()
        self.assertFalse(rig.loop.motor_allowed)
        self.assertIsNone(rig.loop._stop_transport)
        self.assertEqual(rig.loop.request('start', timeout=0, row_len_m=30.)['error'], 'loop_not_responding')
        fut = rig.loop.submit('start', row_len_m=30.)
        rig.run(1.5)
        self.assertTrue(fut.result()['ok'])
        rig.press('press-1')
        out = rig.run(3., hold=True)
        self.assertTrue(any(d.state == ACTIVE and (d.left or d.right) for d in out))   # proposals exist
        self.assertIsNone(rig.loop.bridge)
        self.assertEqual(rig.transport.requests, [])           # nothing ever reached the bridge
        self.assertEqual(rig.loop.request('arm', timeout=0), dict(ok=False, error='loop_not_responding'))
        fut = rig.loop.submit('arm')
        rig.run(.05)
        self.assertEqual(fut.result()['error'], 'motor_output_not_allowed')
        result = rig.loop.emergency_stop('test')
        self.assertTrue(result['shadow'])
        self.assertIsNone(result['stop_sent'])
        rig.run(.1)
        self.assertEqual(rig.loop.runner.state, ENDED)
        self.assertEqual(rig.loop.runner.end_reason, 'operator_stop')
        self.assertEqual(rig.transport.requests, [])

    def test_motor_flag_needs_config_and_shadow_auto_needs_shadow(self):
        with self.assertRaisesRegex(ValueError, 'does not enable motor output'):
            M.ExecutorLoop(legacy_sensor_config(), motor_output_flag=True)
        with self.assertRaisesRegex(ValueError, 'shadow_auto is refused'):
            M.ExecutorLoop(motor_config(), motor_output_flag=True, shadow_auto=True)
        loop = M.ExecutorLoop(motor_config(), motor_output_flag=False)      # enabled config without the flag
        self.assertFalse(loop.motor_allowed)
        self.assertIsNone(loop._stop_transport)

    def test_shadow_auto_runs_and_restarts(self):
        rig = Rig(shadow_auto=True)
        out = rig.run(12.)
        runs = {d.summary.get('run_id') for d in out}
        self.assertGreaterEqual(len(runs), 2)                  # static vehicle: drive_no_response, then restart
        self.assertTrue(any(d.state == ACTIVE and d.left > 0 for d in out))
        self.assertIn('drive_no_response', {d.reason for d in out if d.state == ENDED})
        self.assertIsNone(rig.loop.bridge)
        ticks = [r for r in rig.loop.log.records if r['k'] == 'tick']
        ended = [r for r in ticks if r['state'] == 'ENDED']
        self.assertTrue(ended and all(r['end_reason'] for r in ended))
        for prev, cur in zip(ticks, ticks[1:]):       # a restart never relabels the old run's end ticks
            if cur['run'] != prev['run']:
                self.assertNotEqual(cur['state'], 'ENDED')


class InputTest(unittest.TestCase):
    def test_only_newest_scan_is_processed_but_all_are_stamped(self):
        rig = Rig()
        rig.loop.submit('start', row_len_m=30.)
        rig.loop.step(rig.clock.t)
        for k in range(3):
            rig.loop.on_scan_msg(rig.clock.t + .001 * k, scan_msg(rig.clock.t - .1 + .1 * k))
        rig.clock.t += .05
        rig.loop.step()
        self.assertEqual(rig.loop.counts['scans_processed'], 1)
        self.assertEqual(rig.loop.counts['scans_dropped'], 2)
        self.assertEqual(rig.loop.stamper.counts['stamped'], 3)

    def test_message_field_conversion(self):
        f = M.scan_fields(scan_msg(12.25))
        self.assertAlmostEqual(f['header'], 12.25, places=6)
        self.assertEqual(f['ranges'].dtype, np.float32)
        gyro, acc, h = M.imu_fields(imu_msg(3.5, gz=.1))
        self.assertEqual(gyro, (0., 0., .1))
        self.assertAlmostEqual(h, 3.5, places=6)
        self.assertEqual(M.human_command('{"left": 40, "right": 55}'), (40, 55))
        self.assertIsNone(M.human_command('{"left": 4.0, "right": 55}'))
        self.assertIsNone(M.human_command('x'))


class HoldStateTest(unittest.TestCase):
    def test_expired_press_is_never_revived(self):
        h = M.HoldState(.5)
        self.assertTrue(h.beat('aaaa', 0.))
        self.assertTrue(h.active(.4))
        self.assertFalse(h.active(.61))                 # heartbeat lost
        self.assertFalse(h.beat('aaaa', .7))            # the same press can not come back
        self.assertFalse(h.active(.7))
        self.assertTrue(h.beat('bbbb', .8))             # a NEW press is needed
        self.assertTrue(h.active(.8))
        h.release('bbbb')
        self.assertFalse(h.active(.81))
        self.assertFalse(h.beat('bbbb', .82))
        self.assertFalse(h.beat('x', .9))               # malformed id

    def test_second_device_press_releases_both_for_good(self):          # review S3
        h = M.HoldState(.5)
        self.assertTrue(h.beat('aaaa', 0.))
        serial = h.current(0.)[1]
        self.assertFalse(h.beat('bbbb', .1))            # a second device while aaaa is held: conflict
        self.assertEqual(h.current(.1), (False, None))  # the level drops
        self.assertFalse(h.beat('aaaa', .15))           # the first device can not continue ...
        self.assertFalse(h.beat('bbbb', .2))            # ... and the second can not take over
        h.release('bbbb')
        self.assertFalse(h.active(.25))
        self.assertTrue(h.beat('cccc', .3))             # only a genuinely new press holds again
        self.assertEqual(h.current(.3), (True, serial + 1))
        self.assertEqual((h.counts['conflicts'], h.counts['ignored']), (1, 2))

    def test_release_retires_ids_that_are_not_current(self):           # review F8 (late beats)
        h = M.HoldState(.5)
        h.release('late-press')                         # the release overtook its own last beat
        self.assertFalse(h.beat('late-press', 0.))
        self.assertTrue(h.beat('aaaa', .1))
        h.release('zzzz')                               # another id's release does not release aaaa
        self.assertTrue(h.active(.15))
        self.assertFalse(h.beat('zzzz', .16))
        h.release(None)                                 # malformed release body: releases the current press
        self.assertFalse(h.active(.2))
        self.assertFalse(h.beat('aaaa', .25))
        h.release(12345)                                # not a press id: ignored
        self.assertTrue(h.beat('bbbb', .3))


class MotorTest(unittest.TestCase):
    def setUp(self):
        self.rig = Rig(motor_config(), motor=True)
        self.loop = self.rig.loop

    def start_and_arm(self):
        fut = self.loop.submit('start', row_len_m=30.)
        self.rig.run(1.5)
        self.assertTrue(fut.result()['ok'])
        fut = self.loop.submit('arm')
        self.rig.run(.05)
        self.assertTrue(fut.result()['ok'], fut.result())
        self.assertEqual(self.loop.bridge.state, ARMED)

    def drives(self):
        return [r for r in self.rig.transport.requests if r['op'] == 'drive']

    def test_hold_before_arm_never_resumes(self):
        fut = self.loop.submit('start', row_len_m=30.)
        self.rig.run(1.5)
        self.assertTrue(fut.result()['ok'])
        self.rig.press('early')
        self.rig.run(2., hold=True)
        self.assertEqual(self.loop.runner.state, PAUSED)
        fut = self.loop.submit('arm')
        self.rig.run(.05, hold=True)
        self.assertEqual(fut.result()['error'], 'release_hold_first')
        self.assertEqual(self.rig.transport.requests, [])

    def test_every_drive_has_deadman_true_and_cadence(self):
        self.start_and_arm()
        self.rig.run(1.)                                   # armed, paused: 0/0 drives keep the lease
        paused = self.drives()
        self.assertTrue(paused and all(r['left'] == r['right'] == 0 for r in paused))
        self.assertTrue(8 <= len(paused) <= 12)            # 10 Hz
        self.rig.press('press-go')
        self.rig.run(2., hold=True)
        driving = [r for r in self.drives() if r['left'] or r['right']]
        self.assertTrue(driving)
        self.assertTrue(all(r['deadman'] is True for r in self.drives()))
        self.assertEqual([r['seq'] for r in self.drives()], list(range(1, len(self.drives()) + 1)))
        self.assertTrue(all(min(r['left'], r['right']) >= 50 for r in driving))   # inner >= breakaway + margin
        n = len(self.drives())
        self.rig.release()
        self.rig.run(.05)                                  # release -> 0/0 sent at once, not at the cadence
        self.assertEqual((self.drives()[n]['left'], self.drives()[n]['right']), (0, 0))
        self.assertEqual(self.loop.runner.state, PAUSED)
        self.assertEqual(self.loop.bridge.state, ARMED)

    def test_terminal_end_sends_zero_then_releases_the_lease(self):
        self.start_and_arm()
        self.rig.press('press-go')
        out = self.rig.run(7., hold=True)                  # the static vehicle: drive_no_response
        self.assertEqual(self.loop.runner.end_reason, 'drive_no_response')
        self.assertEqual(self.loop.bridge.state, DISARMED)
        self.assertEqual(self.rig.transport.ops()[-1], 'stop')
        self.assertIsNone(self.rig.control.lease)
        self.assertFalse(self.rig.control.moving)
        fut = self.loop.submit('start', row_len_m=30.)
        self.rig.run(.05)
        self.assertTrue(fut.result()['ok'])                # a new run needs a new Start + Arm
        self.assertEqual(self.loop.bridge.state, DISARMED)

    def test_operator_stop_is_immediate_and_latches(self):
        self.start_and_arm()
        self.rig.press('press-go')
        self.rig.run(1.5, hold=True)
        before = len(self.rig.transport.requests)
        result = self.loop.emergency_stop('page')           # from the HTTP thread: no loop step needed
        self.assertTrue(result['stop_sent'])
        self.assertEqual(self.rig.transport.requests[before]['op'], 'stop')
        self.assertIsNone(self.rig.control.lease)
        self.rig.run(.1, hold=True)
        self.assertEqual(self.loop.bridge.state, LATCHED)
        self.assertEqual(self.loop.runner.end_reason, 'operator_stop')
        fut = self.loop.submit('start', row_len_m=30.)
        self.rig.run(.05)
        self.assertEqual(fut.result()['error'], 'acknowledge_latch_first')
        fut = self.loop.submit('ack')
        self.rig.run(.05)
        fut = self.loop.submit('start', row_len_m=30.)
        self.rig.run(.05)
        self.assertTrue(fut.result()['ok'])

    def test_stop_before_any_arm_still_reaches_the_bridge(self):
        result = self.loop.emergency_stop('page')
        self.assertTrue(result['stop_sent'])
        self.assertEqual(self.rig.transport.ops(), ['stop'])

    def test_other_client_stop_latches_and_ends_the_run(self):
        self.start_and_arm()
        self.rig.press('press-go')
        self.rig.run(1.5, hold=True)
        self.rig.control.stop('dashboard_stop')             # e.g. the observer's phone STOP
        self.rig.run(.3, hold=True)
        self.assertEqual(self.loop.bridge.state, LATCHED)
        self.assertEqual(self.loop.runner.state, ENDED)
        self.assertIn(self.loop.runner.end_reason, ('bridge_error', 'bridge_latched'))
        n = len(self.rig.transport.requests)
        self.rig.run(1., hold=True)
        self.assertEqual(len(self.rig.transport.requests), n)    # nothing more is sent: no automatic re-arm

    def test_lost_lease_latches_without_stopping_the_humans_new_lease(self):      # review S7
        self.start_and_arm()
        self.rig.press('press-go')
        self.rig.run(1.5, hold=True)
        self.assertTrue(any(r['left'] > 0 for r in self.drives()))
        self.rig.control.stop('dashboard_stop')             # dashboard STOP ...
        human = self.rig.control.dispatch({'op': 'arm', 'owner': 'dashboard-manual'})   # ... then manual arm
        n = len(self.rig.transport.requests)
        self.rig.run(.5, hold=True)
        self.assertEqual(self.loop.bridge.state, LATCHED)
        self.assertEqual(self.loop.runner.state, ENDED)
        after = self.rig.transport.requests[n:]
        self.assertEqual([r['op'] for r in after], ['drive'])     # the refused drive, NO op stop afterwards
        self.assertEqual(self.rig.control.lease, human['lease'])   # the human keeps the lease
        self.assertEqual(self.rig.control.owner, 'dashboard-manual')
        self.assertEqual(self.loop.bridge.counts['latch_stop_skipped'], 1)
        self.loop.shutdown()                                       # shutdown does not stop it either
        self.assertEqual(self.rig.control.owner, 'dashboard-manual')
        self.assertNotIn('stop', [r['op'] for r in self.rig.transport.requests[n:]])

    def test_watchdog_sends_zero_once_and_pauses(self):
        self.start_and_arm()
        self.rig.press('press-go')
        self.rig.run(2., hold=True)
        self.assertEqual(self.loop.runner.state, ACTIVE)
        self.assertNotEqual(self.loop._last_sent, (0, 0))
        self.rig.clock.t += .4                               # the main loop is stuck (e.g. a long on_scan)
        self.assertTrue(self.loop.watchdog_check())
        self.assertFalse(self.loop.watchdog_check())         # once per stall
        last = self.drives()[-1]
        self.assertEqual((last['left'], last['right']), (0, 0))
        self.loop.hold_beat('press-go')
        self.loop.step()
        self.assertEqual(self.loop.runner.state, PAUSED)
        self.assertEqual(self.loop.runner.reason, 'stall_guard')
        self.rig.run(1.5, hold=True)                         # still held: no automatic restart
        self.assertEqual(self.loop.runner.state, PAUSED)

    def test_slow_scan_inside_a_live_step_never_resends_motion(self):
        self.start_and_arm()
        self.rig.press('press-go')
        self.rig.run(2., hold=True)
        self.assertNotEqual(self.loop._last_sent, (0, 0))
        runner, clock, loop = self.loop.runner, self.rig.clock, self.loop
        original = runner.feed_scan

        def slow_feed(*args, **kwargs):          # an on_scan that takes 0.4 s; the watchdog thread fires meanwhile
            clock.t += .2
            loop.watchdog_check()
            clock.t += .2
            return original(*args, **kwargs)
        runner.feed_scan = slow_feed
        clock.t += .05
        loop.on_scan_msg(clock.t, scan_msg(clock.t - .1))
        loop.on_imu_msg(clock.t, imu_msg(clock.t))
        loop.hold_beat('press-go')
        n = len(self.drives())
        d = loop.step()                           # live mode: the clock is read again after the inputs
        runner.feed_scan = original
        self.assertEqual((d.state, d.left, d.right), (PAUSED, 0, 0))
        self.assertEqual(runner.reason, 'stall_guard')
        after = [(r['left'], r['right']) for r in self.drives()[n:]]
        self.assertTrue(after and all(x == (0, 0) for x in after), after)

    def test_params_lower_only_with_motor_and_not_during_a_run(self):
        # breakaway 35: base 60 / differential 10 leaves room to lower the base (the steering GO rule keeps the
        # differential at >= 10 in motor configs, field decision rules 2026-10-22)
        self.rig = Rig(motor_config(breakaway_duty_pct=35), motor=True)
        self.loop = self.rig.loop
        fut = self.loop.submit('params', params=dict(base_duty_pct=61))
        self.rig.run(.05)
        self.assertIn('allows only <=', ' '.join(fut.result()['problems']))
        fut = self.loop.submit('params', params=dict(base_duty_pct=55, reduce_duty_pct=55, max_differential_pct=10))
        self.rig.run(.05)
        self.assertTrue(fut.result()['ok'], fut.result())
        self.assertEqual(self.loop.config.base_duty_pct, 55)
        self.assertEqual(self.loop.runner.config.max_differential_pct, 10)
        fut = self.loop.submit('params', params=dict(base_duty_pct=54))   # inner 44 < breakaway 35 + 10
        self.rig.run(.05)
        self.assertFalse(fut.result()['ok'])
        fut = self.loop.submit('params', params=dict(max_differential_pct=8))   # steering GO rule: >= 10
        self.rig.run(.05)
        self.assertFalse(fut.result()['ok'])
        self.loop.submit('start', row_len_m=30.)
        self.rig.run(.05)
        fut = self.loop.submit('params', params=dict(base_duty_pct=55))
        self.rig.run(.05)
        self.assertEqual(fut.result()['error'], 'end_the_run_first')


class BenchTest(unittest.TestCase):
    def test_bench_pattern_through_hold_and_guard(self):
        cfg = legacy_sensor_config(profile='bench', bench_ack=C.BENCH_ACK, motor_output_enabled=True,
                             bench_pattern=((1., 40, 40), (1., 30, 50))).validate()
        rig = Rig(cfg, motor=True)
        fut = rig.loop.submit('start')                     # no row length in the bench profile
        rig.run(1.5)
        self.assertTrue(fut.result()['ok'], fut.result())
        fut = rig.loop.submit('arm')
        rig.run(.05)
        self.assertTrue(fut.result()['ok'], fut.result())
        rig.press('bench-press')
        rig.run(3., hold=True)
        drives = [(r['left'], r['right']) for r in rig.transport.requests if r['op'] == 'drive']
        self.assertIn((40, 40), drives)
        self.assertIn((30, 50), drives)
        self.assertTrue(all(r['deadman'] is True for r in rig.transport.requests if r['op'] == 'drive'))
        self.assertEqual(rig.loop.runner.end_reason, 'bench_pattern_complete')
        self.assertEqual(rig.loop.bridge.state, DISARMED)

    def test_field_start_needs_a_row_length(self):
        rig = Rig()
        fut = rig.loop.submit('start')
        rig.run(.05)
        self.assertEqual(fut.result()['error'], 'row_len_m_required')


class LoopExceptionTest(unittest.TestCase):
    def test_exception_stops_then_limit_ends_the_process(self):
        class Boom:
            def __init__(self):
                self.calls, self.stops, self.events = 0, [], []

            def step(self):
                self.calls += 1
                raise RuntimeError('bug')

            def emergency_stop(self, source):
                self.stops.append(source)

            def _event(self, name, **kw):
                self.events.append(name)

            counts = {}
        loop = Boom()
        stop = M.threading.Event()
        with self.assertRaises(RuntimeError):
            M.run_loop(loop, stop, hz=1000., max_exceptions=3, exception_window_s=60.)
        self.assertEqual(loop.calls, 4)
        self.assertEqual(loop.stops, ['loop_exception'] * 4)
        self.assertEqual(loop.events, ['loop_exception'] * 4)

    @staticmethod
    def drive_steps(rig, stop, broken, fix_at, stop_at):
        """Wrap loop.step: advance the virtual clock 50 ms per step; _feed_inputs raises while broken."""
        loop = rig.loop
        original_feed, original_step = loop._feed_inputs, loop.step
        calls = {'n': 0}

        def feed():
            if broken['on']:
                raise RuntimeError('bug in feed')
            return original_feed()

        def step():
            calls['n'] += 1
            rig.clock.t += .05
            if calls['n'] == fix_at:
                broken['on'] = False
            if calls['n'] >= stop_at:
                stop.set()
            return original_step()
        loop._feed_inputs, loop.step = feed, step
        return calls

    def test_shadow_exceptions_never_end_the_process_and_logging_continues(self):      # review S8
        rig = Rig()
        fut = rig.loop.submit('start', row_len_m=30.)
        rig.run(1.5)
        self.assertTrue(fut.result()['ok'])
        stop, broken = threading.Event(), {'on': True}
        self.drive_steps(rig, stop, broken, fix_at=31, stop_at=60)
        M.run_loop(rig.loop, stop, hz=2000., shadow_trace_limit=3, shadow_summary_s=60., shadow_reset_after=1000)
        events = [r['event'] for r in rig.loop.log.records if r['k'] == 'event']
        self.assertEqual(events.count('loop_exception'), 3)          # full traces only for the first ones
        self.assertEqual(events.count('loop_exception_summary'), 1)  # then rate limited
        self.assertEqual(rig.loop.counts['loop_exceptions'], 30)
        self.assertEqual(rig.loop.runner.end_reason, 'loop_exception')
        ticks = [r for r in rig.loop.log.records if r['k'] == 'tick']
        self.assertEqual(len(ticks), 30 + 30)                        # the loop logs normally again after the fix
        st = rig.loop.status()
        self.assertEqual((st['loop_exceptions'], st['loop_error']), (30, "RuntimeError('bug in feed')"))

    def test_shadow_runner_rebuild_is_bounded(self):
        rig = Rig()
        stop, broken = threading.Event(), {'on': True}
        self.drive_steps(rig, stop, broken, fix_at=10 ** 9, stop_at=100)
        M.run_loop(rig.loop, stop, hz=2000., shadow_trace_limit=1, shadow_summary_s=60., shadow_reset_after=10,
                   shadow_max_resets=2)
        events = [r['event'] for r in rig.loop.log.records if r['k'] == 'event']
        self.assertEqual(events.count('runner_reset'), 2)
        self.assertEqual(rig.loop.counts['runner_resets'], 2)
        self.assertEqual(rig.loop.counts['loop_exceptions'], 100)

    def test_motor_exception_stops_and_latches_as_before(self):
        rig = Rig(motor_config(), motor=True)
        holder = types.SimpleNamespace(rig=rig, loop=rig.loop, assertTrue=self.assertTrue,
                                       assertEqual=self.assertEqual)
        MotorTest.start_and_arm(holder)
        rig.press('press-go')
        rig.run(2., hold=True)
        self.assertEqual(rig.loop.runner.state, ACTIVE)
        n = len(rig.transport.requests)
        stop, broken = threading.Event(), {'on': True}
        self.drive_steps(rig, stop, broken, fix_at=2, stop_at=6)
        M.run_loop(rig.loop, stop, hz=2000.)
        after = rig.transport.requests[n:]
        self.assertEqual(after[0]['op'], 'stop')                       # op stop at once from the exception path
        self.assertFalse([r for r in after if r['op'] == 'drive' and (r['left'] or r['right'])])
        self.assertEqual(rig.loop.bridge.state, LATCHED)
        self.assertEqual(rig.loop.runner.end_reason, 'operator_stop')
        self.assertIsNone(rig.control.lease)


class HoldAliasTest(unittest.TestCase):
    """Review S3 at the loop level (MOTOR, real bridge core): only a genuinely new press is a press edge."""

    def setUp(self):
        self.rig = Rig(motor_config(), motor=True)
        self.loop = self.rig.loop
        MotorTest.start_and_arm(self)

    def nonzero(self, since=0):
        return [r for r in self.rig.transport.requests[since:] if r['op'] == 'drive' and (r['left'] or r['right'])]

    def drive_then_stale_pause(self, press='device-a-1'):
        self.rig.press(press)
        self.rig.run(2., hold=True)
        self.assertEqual(self.loop.runner.state, ACTIVE)
        self.assertTrue(self.nonzero())
        self.rig.scan_on = False                          # LiDAR gap -> stale pause while A keeps holding
        self.rig.run(.7, hold=True)
        self.assertEqual((self.loop.runner.state, self.loop.runner.reason), (PAUSED, 'stale_lidar'))
        self.rig.scan_on = True
        self.rig.run(1.6, hold=True)                      # fresh again for > 1 s; holding does not restart
        self.assertEqual(self.loop.runner.state, PAUSED)

    def test_second_device_tap_never_restarts_a_held_stale_pause(self):
        self.drive_then_stale_pause()
        n = len(self.rig.transport.requests)
        self.assertFalse(self.loop.hold_beat('device-b-1'))       # B taps the button on another phone
        self.loop.hold_release('device-b-1')
        out = self.rig.run(2., hold=True)                          # A never lifted the finger
        self.assertTrue(all(d.state == PAUSED for d in out))
        self.assertEqual(self.nonzero(n), [])
        self.assertEqual(self.loop.hold.counts['conflicts'], 1)
        self.rig.release()                                         # A lifts and presses again: a new press
        self.rig.press('device-a-2')
        self.rig.run(2., hold=True)
        self.assertEqual(self.loop.runner.state, ACTIVE)
        self.assertTrue(self.nonzero(n))

    def test_second_device_press_while_driving_pauses_and_nobody_continues(self):
        self.rig.press('device-a-1')
        self.rig.run(2., hold=True)
        self.assertEqual(self.loop.runner.state, ACTIVE)
        self.assertFalse(self.loop.hold_beat('device-b-1'))       # conflict
        n = len(self.rig.transport.requests)
        self.rig.run(1.5, hold=True)                               # A keeps sending beats: ignored
        self.assertEqual((self.loop.runner.state, self.loop.runner.reason), (PAUSED, 'hold_released'))
        self.assertEqual(self.nonzero(n), [])
        first = self.rig.transport.requests[n]
        self.assertEqual((first['op'], first['left'], first['right']), ('drive', 0, 0))   # 0/0 at once
        self.assertEqual(self.loop.bridge.state, ARMED)

    def test_quick_repress_between_two_ticks_is_its_own_edge(self):
        self.drive_then_stale_pause()
        resumes = self.loop.runner.resumes
        self.loop.hold_release('device-a-1')                       # release and new press inside one tick
        self.rig.press('device-a-2')
        d1 = self.rig.run(.05, hold=True)[-1]
        self.assertEqual(d1.state, PAUSED)                         # the release is shown for one tick
        self.rig.run(.05, hold=True)
        self.assertEqual(self.loop.runner.state, ACTIVE)           # the new press resumes (the old code missed it)
        self.assertEqual(self.loop.runner.resumes, resumes + 1)
        self.assertEqual(self.loop.counts['hold_edges_split'], 1)

    def test_quick_repress_while_driving_pauses_then_resumes_with_a_new_controller(self):
        self.rig.press('device-a-1')
        self.rig.run(2., hold=True)
        built = self.loop.runner.controllers_built
        self.loop.hold_release('device-a-1')
        self.rig.press('device-a-2')
        self.assertEqual(self.rig.run(.05, hold=True)[-1].state, PAUSED)
        self.rig.run(.05, hold=True)
        self.assertEqual(self.loop.runner.state, ACTIVE)
        self.assertEqual(self.loop.runner.controllers_built, built + 1)


class LogWriterTest(unittest.TestCase):
    """Review S8: write errors never kill the writer; dropped lines are counted from the first failure."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='rowexec-log-')
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def wait(self, cond, timeout=5.):
        end = time.monotonic() + timeout
        while not cond():
            if time.monotonic() > end:
                self.fail('condition not reached')
            time.sleep(.01)

    def lines(self, log):
        out = []
        for path in log.paths:
            with open(path, 'rb') as fh:
                data = fh.read()
            self.assertNotIn(b'\r\n', data)
            for raw in data.split(b'\n'):
                if raw.strip():
                    out.append(M.json.loads(raw))
        return out

    def test_disk_errors_are_counted_and_writing_resumes(self):
        failing = {'on': False}

        def write(fd, data):
            if failing['on']:
                raise OSError(errno.ENOSPC, 'No space left on device')
            return os.write(fd, data)
        log = M.JsonlLog(self.tmp, write=write, retry_s=.2, wall=lambda: 1.7e9)
        self.addCleanup(log.close)
        for k in range(5):
            log.put({'k': k})
        self.wait(lambda: log.lines_written == 5)
        self.assertEqual(log.health()['state'], 'ok')
        failing['on'] = True
        for k in range(5, 15):
            log.put({'k': k})
        self.wait(lambda: log.dropped == 10)                 # counted at once, not after 4000 queued lines
        h = log.health()
        self.assertEqual(h['state'], 'failing')
        self.assertTrue(h['writer_alive'])
        self.assertIn('No space', h['last_error'])
        failing['on'] = False
        time.sleep(.3)                                       # retry interval
        for k in range(15, 18):
            log.put({'k': k})
        self.wait(lambda: log.lines_written == 8)
        h = log.health()
        self.assertEqual((h['state'], h['dropped'], h['recoveries']), ('ok', 10, 1))
        log.close()
        self.assertEqual([r['k'] for r in self.lines(log)], [0, 1, 2, 3, 4, 15, 16, 17])
        self.assertEqual(len(log.paths), 1)                  # the same part file is reopened, not one per retry

    def test_partial_line_stays_separate(self):
        calls = {'n': 0}

        def write(fd, data):
            calls['n'] += 1
            if calls['n'] == 2:                              # second line: half written, then the disk fails
                os.write(fd, bytes(data[:7]))
                raise OSError(errno.EIO, 'I/O error')
            return os.write(fd, data)
        log = M.JsonlLog(self.tmp, write=write, retry_s=.05, wall=lambda: 1.7e9)
        log.put({'k': 0})
        log.put({'k': 1})
        self.wait(lambda: log.dropped == 1)
        time.sleep(.1)
        log.put({'k': 2})
        self.wait(lambda: log.lines_written == 2)
        log.close()
        with open(log.paths[0], 'rb') as fh:
            rows = [line for line in fh.read().split(b'\n') if line]
        self.assertEqual(M.json.loads(rows[0]), {'k': 0})
        self.assertEqual(M.json.loads(rows[-1]), {'k': 2})   # not glued to the partial line
        self.assertEqual(len(rows), 3)

    def test_hung_disk_is_reported_as_stalled(self):
        gate = threading.Event()

        def write(fd, data):
            gate.wait(5.)
            return os.write(fd, data)
        log = M.JsonlLog(self.tmp, write=write, stall_s=.2, wall=lambda: 1.7e9)
        self.addCleanup(gate.set)
        for k in range(3):
            log.put({'k': k})
        self.wait(lambda: log.health()['state'] == 'stalled')
        self.assertGreater(log.health()['pending'], 0)
        gate.set()
        self.wait(lambda: log.lines_written == 3)
        self.assertEqual(log.health()['state'], 'ok')
        log.close()

    def test_dead_writer_counts_every_line(self):
        class Kill(BaseException):
            pass

        def write(fd, data):
            raise Kill()
        hook = threading.excepthook
        threading.excepthook = lambda args: None             # the deliberate thread death is not test noise
        self.addCleanup(setattr, threading, 'excepthook', hook)
        log = M.JsonlLog(self.tmp, write=write, wall=lambda: 1.7e9)
        log.put({'k': 0})
        log.thread.join(2.)
        self.assertFalse(log.thread.is_alive())
        before = log.dropped
        for k in range(4):
            log.put({'k': k})
        self.assertEqual(log.dropped, before + 4)
        self.assertEqual(log.health()['state'], 'failing')

    @unittest.skipUnless(sys.platform.startswith('linux') and os.path.exists('/dev/full'), 'needs /dev/full')
    def test_real_enospc_from_dev_full(self):
        log = M.JsonlLog(self.tmp, open_fd=lambda path: os.open('/dev/full', os.O_WRONLY), retry_s=.05,
                         wall=lambda: 1.7e9)
        for k in range(20):
            log.put({'k': k})
        self.wait(lambda: log.dropped == 20)
        h = log.health()
        self.assertEqual(h['state'], 'failing')
        self.assertIn('No space', h['last_error'])
        log.close()

    def test_status_shows_the_log_state(self):
        failing = {'on': True}

        def write(fd, data):
            if failing['on']:
                raise OSError(errno.ENOSPC, 'No space left on device')
            return os.write(fd, data)
        log = M.JsonlLog(self.tmp, write=write, retry_s=60., wall=lambda: 1.7e9)
        self.addCleanup(log.close)
        rig = Rig(log=log)
        self.wait(lambda: log.dropped >= 1)                  # the config event already failed
        rig.run(.2)
        status = rig.loop.status()
        self.assertEqual(status['log']['state'], 'failing')
        self.assertGreaterEqual(status['log']['dropped'], 1)


class StopMeaningTest(unittest.TestCase):
    """Review S9: the status says what the executor does to the vehicle and what the page STOP does."""

    def test_shadow(self):
        rig = Rig()
        rig.run(.1)
        st = rig.loop.status()
        self.assertEqual((st['mode'], st['vehicle_control'], st['stop_button']),
                         ('SHADOW', 'none', 'ends_shadow_run_only'))
        result = rig.loop.emergency_stop('page')
        self.assertIsNone(result['stop_sent'])
        self.assertIn('NOT stopped', result['note'])

    def test_motor(self):
        rig = Rig(motor_config(), motor=True)
        rig.run(.1)
        st = rig.loop.status()
        self.assertEqual((st['vehicle_control'], st['stop_button']), ('not_armed', 'vehicle_stop'))
        holder = types.SimpleNamespace(rig=rig, loop=rig.loop, assertTrue=self.assertTrue,
                                       assertEqual=self.assertEqual)
        MotorTest.start_and_arm(holder)
        rig.run(.1)
        self.assertEqual(rig.loop.status()['vehicle_control'], 'armed')
        result = rig.loop.emergency_stop('page')
        self.assertTrue(result['stop_sent'])
        self.assertIn('confirmed', result['note'])


class StampAndFilterTest(unittest.TestCase):
    def test_header_offset_stamps(self):
        st = ScanStamper('header_offset', .08, window=50, reset_s=.5)
        stamps = [st.stamp(10. + .1 * k + .1 + (.03 if k % 7 == 3 else 0.), 10. + .1 * k) for k in range(30)]
        self.assertTrue(all(abs(b - a - .1) < 1e-9 for a, b in zip(stamps, stamps[1:])))   # header spacing kept
        burst = [st.stamp(20.5 + .001 * k, 20.2 + .1 * k) for k in range(3)]     # 3 scans received in 2 ms
        self.assertTrue(all(b - a > .09 for a, b in zip(burst, burst[1:])))
        self.assertTrue(all(s <= r for s, r in zip(burst, (20.5, 20.501, 20.502))))
        resets = st.counts['resets']
        st.stamp(30., 30. - .1 - 3600.)                                        # system clock stepped
        self.assertEqual(st.counts['resets'], resets + 1)
        self.assertAlmostEqual(st.stamp(30.1, 30.1 - .1 - 3600.), 30.1 - .08, places=6)
        self.assertAlmostEqual(st.stamp(31., None), 31. - .08)
        self.assertEqual(ScanStamper('receipt', .08).stamp(5., 1.), 5. - .08)

    def test_near_filter_matches_the_evaluated_replay_filter(self):
        rng = np.random.default_rng(3)
        a = NearConfirmFilter(2, 2., .35, .1)
        b = ReplayNearTransientFilter(near_m=2., tol_m=.35, confirm_scans=2, tol_step_m=.1)
        for k in range(30):
            r = rng.uniform(.2, 6., 400)
            r[rng.random(400) < .3] = np.inf
            ra, rb = r.copy(), r.copy()
            self.assertEqual(a.apply(ra, -math.pi, 2 * math.pi / 400), b.apply(rb, -math.pi, 2 * math.pi / 400))
            np.testing.assert_array_equal(ra, rb)

    def test_one_scan_cluster_removed_persistent_object_kept(self):
        f = NearConfirmFilter(2, 2., .35, .1)
        inc = 2 * math.pi / 360
        base = np.full(360, np.inf)
        base[100:105] = 1.0                       # a real object, every scan
        for _ in range(3):
            f.apply(base.copy(), -math.pi, inc)
        speck = base.copy()
        speck[200:210] = .6                       # one-scan cluster
        removed = f.apply(speck, -math.pi, inc)
        self.assertEqual(removed, 10)
        self.assertTrue(np.all(np.isinf(speck[200:210])))
        self.assertTrue(np.all(speck[100:105] == 1.0))


class GeometryGuardTest(unittest.TestCase):
    def snapshot(self, **over):
        s = dict(lidar_age_s=.05, imu_age_s=.02, still=False, still_s=0., speed=.4, distance_m=1., yaw_rate=0.,
                 corridor_valid=True, corridor_reason='tracked', heading_error=0., lateral_offset=0.,
                 corridor_width=3.5)
        s.update(over)
        return s

    def test_heading_and_lateral_limits(self):
        runner = Runner(legacy_sensor_config())
        self.assertIsNone(runner.geometry_guard(self.snapshot()))
        self.assertEqual(runner.geometry_guard(self.snapshot(heading_error=math.radians(26))), 'guard_heading')
        self.assertIsNone(runner.geometry_guard(self.snapshot(heading_error=math.radians(24))))
        self.assertEqual(runner.geometry_guard(self.snapshot(lateral_offset=1.41)), 'guard_lateral')   # 1.75-0.35
        self.assertIsNone(runner.geometry_guard(self.snapshot(lateral_offset=-1.39)))
        self.assertIsNone(runner.geometry_guard(self.snapshot(corridor_valid=False, heading_error=3.)))
        off = Runner(legacy_sensor_config(guard_heading_deg=None, guard_lateral_margin_m=None))
        self.assertIsNone(off.geometry_guard(self.snapshot(heading_error=3., lateral_offset=5.)))

    def test_guard_ends_a_driving_run(self):
        class Est:
            def __init__(self):
                self.heading = 0.

            def on_imu(self, *a):
                pass

            def on_scan(self, *a):
                pass

            def snapshot(self, now):
                return GeometryGuardTest.snapshot(None, heading_error=self.heading)

        class Ctl:
            state, reason = 'FOLLOW', 'stub'

            def step(self, now, s):
                return types.SimpleNamespace(left=55, right=55, state='FOLLOW', reason='stub')

        est = Est()
        runner = Runner(legacy_sensor_config(), estimator_factory=lambda: est, controller_factory=Ctl)
        runner.start_run(0., 30.)
        runner.tick(1.1, False)
        runner._fresh_since = 0.
        d = runner.tick(1.2, True)
        self.assertEqual((d.state, d.left), (ACTIVE, 55))
        est.heading = math.radians(30)
        d = runner.tick(1.25, True)
        self.assertEqual((d.state, d.reason, d.left, d.right), (ENDED, 'guard_heading', 0, 0))


class SourceManifestTest(unittest.TestCase):
    def test_committed_sums_match_and_changes_are_detected(self):
        digest = M.verify_sources()
        self.assertEqual(len(digest), 64)
        with tempfile.TemporaryDirectory() as tmp:
            copy = os.path.join(tmp, 'row_executor')
            shutil.copytree(M.PACKAGE_DIR, copy, ignore=shutil.ignore_patterns('__pycache__'))
            M.verify_sources(copy)
            with open(os.path.join(copy, 'guard.py'), 'ab') as fh:
                fh.write(b'# edited\n')
            with self.assertRaisesRegex(RuntimeError, 'does not match'):
                M.verify_sources(copy)
            os.remove(os.path.join(copy, 'guard.py'))
            with self.assertRaisesRegex(RuntimeError, 'guard.py'):
                M.verify_sources(copy)


class OnsiteParamsTest(unittest.TestCase):
    def test_bounds_and_unknown_keys(self):
        cfg = legacy_sensor_config()
        with self.assertRaisesRegex(C.ConfigError, 'outside'):
            C.onsite_update(cfg, cfg, dict(base_duty_pct=80))
        with self.assertRaisesRegex(C.ConfigError, 'not an on-site parameter'):
            C.onsite_update(cfg, cfg, dict(duty_cap_pct=90))
        with self.assertRaisesRegex(C.ConfigError, 'duty_cap_pct 75 < base'):
            C.onsite_update(cfg, cfg, dict(base_duty_pct=60))     # caps are never raised on site
        new = C.onsite_update(cfg, cfg, dict(base_duty_pct=50, max_differential_pct=15, reduce_duty_pct=50))
        self.assertEqual((new.base_duty_pct, new.max_differential_pct), (50, 15))


if __name__ == '__main__':
    unittest.main()
