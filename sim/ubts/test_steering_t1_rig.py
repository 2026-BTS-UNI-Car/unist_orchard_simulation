"""T1 rig (steering/t1_rig.py): host bridge + V3 memory link + firmware-mode MCU + SteeredPlant + world, one SIM clock.

Checks the clock and the order of the rig (bridge grid, UART arrival, line delivery, request answers), the control-API
client handle, the A17 STOP / TTL / watchdog times in T1, the hold_measured target in firmware mode (the MCU filtered
position P[t0], decision R13-2), every fault hook, the host latch mcu_state_mismatch (open P3 item 1, A19 host part),
the op motion source of steering_ineffective (open P3 item 4), determinism (A8 for the rig: same seed -> same serial
bytes and trace SHA-256, also in a fresh process; another seed -> different) and close().

All numbers are placeholders: the results are placeholder-based SIM results (자리값 기반 SIM 결과), not field validation.
unittest style: pytest (3.12, 3.13) and `python3 -m unittest test_steering_t1_rig` (3.10). The native program is built
into UBTS_STEER_BUILD_DIR or a temporary folder (never into the work tree); no file is written inside the work tree.
"""
import atexit
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

from steering import flat_world as fw
from steering import t1_rig as tr
from steering.native import build as native_build
from steering.native import lockstep_v2 as v2
from steering.scene_spec import WorldSpec

UBTS_DIR = os.path.dirname(os.path.abspath(__file__))
_STATE = {}
STATUS_MS = 50          # 1000 / sthz (placeholder 20 Hz)
POSTS = WorldSpec.from_dict({'kind': 'W0', 'markers': [list(p) for p in fw.registration_posts()]})


def _cleanup(path):
    shutil.rmtree(path, ignore_errors=True)


def native_path():
    """The native program with firmware mode (built once per process), or SkipTest with the reason."""
    if 'native' not in _STATE:
        try:
            if not native_build.firmware_present():
                raise native_build.NativeBuildError(native_build.missing_firmware_message())
            native_build.find_compiler()
            folder = os.environ.get(native_build.ENV_BUILD_DIR)
            if not folder:
                folder = tempfile.mkdtemp(prefix='steer_t1_rig_build_')
                atexit.register(_cleanup, folder)
            _STATE['native'] = native_build.ensure_built(build_dir=folder).path
        except native_build.NativeBuildError as exc:
            _STATE['native'] = 'native steering program not available: %s' % exc
    value = _STATE['native']
    if not os.path.isfile(value):
        if native_build.firmware_required():
            raise AssertionError('%s is set: %s' % (native_build.ENV_REQUIRE_FIRMWARE, value))
        raise unittest.SkipTest(value)
    return value


def host_or_skip():
    try:
        return tr.host_modules()
    except tr.RigError as exc:
        raise unittest.SkipTest(str(exc))


class RigCase(unittest.TestCase):
    def setUp(self):
        host_or_skip()
        native_path()

    def rig(self, **config):
        config.setdefault('world', POSTS)
        rig = tr.T1Rig(tr.RigConfig(**config), native_executable=native_path())
        self.addCleanup(rig.close)
        return rig

    def ready(self, rig, limit_s=3.):
        ok = rig.run_until_true(lambda: rig.core.steering_host()['stop_state'] == 'confirmed', limit_s)
        self.assertTrue(ok, rig.core.steering_host())

    def armed(self, rig, owner='rig-test'):
        self.ready(rig)
        client = rig.client(owner)
        self.assertIsNone(client.arm_precheck())
        reply = client.arm()
        self.assertTrue(reply.get('ok'), reply)
        self.assertTrue(client.wait_armed(), rig.core.steering_host())
        return client

    @staticmethod
    def hold(rig, client, seconds, throttle, steer_mrad, every_s=.1):
        """Keep driving: a drive request every every_s for seconds (an executor at 10 Hz)."""
        end = rig.ms + int(round(seconds * 1000))
        reply = None
        while rig.ms < end:
            reply = client.drive(throttle, steer_mrad)
            if not reply.get('ok'):
                return reply
            rig.run_until(min(end, rig.ms + int(round(every_s * 1000)) - 0) / 1000.)
        return reply

    @staticmethod
    def first_change(rig, channel, value, since):
        for tick, v in rig.mcu_channel(channel):
            if tick >= since and v == value:
                return tick
        return None


class HandshakeTest(RigCase):
    def test_ready_banner_identity_and_session_stop(self):
        rig = self.rig(seed=1)
        self.ready(rig, .3)
        self.assertEqual(rig.link.state, 'ready')
        self.assertEqual(rig.link.boot_id, rig.fw.boot_id(0))
        banner = rig.link.banner
        self.assertEqual((banner['sim'], banner['hw'], banner['cal_id']), (True, rig.hp.hw_profile, 1))
        self.assertEqual(rig.link.cal['cal_id'], rig.hp.cal_id)
        heads = [w[3].split(b'*')[0].split(b' ')[0].strip() for w in rig.host_writes()]
        self.assertEqual([h for h in heads if h != b'PING'][:1], [b'STOP'])
        self.assertNotIn(b'ARM', heads)
        self.assertNotIn(b'C', heads)
        self.assertEqual(rig.core.snapshot['state'], 'disarmed')
        self.assertEqual(rig.mcu_lines(('N', 'F', 'W')), [])
        identity = rig.identity()
        self.assertEqual(identity['firmware_build'], rig.fw.firmware_build)
        self.assertEqual(identity['result_kind'], tr.RESULT_KIND)

    def test_port_open_reset_with_boot_delay_and_garbage(self):
        rig = self.rig(port_open_reset=True, boot_delay_ticks=70, boot_garbage_hex='00FF0A41420D0A')
        self.ready(rig, .5)
        self.assertEqual(rig.link.banner['rst'], 'ext_sys')
        # the host opens the port at t = 0, before the MCU's first tick: the port-open reset replaces the RESET
        # boot inside its boot delay, so there is one setup() (LOCKSTEP 11.5) and the boot id is the first one
        self.assertEqual(rig.link.boot_id, rig.fw.boot_id(0))
        self.assertEqual(rig.fw.last.boot_count, 1)

    def test_config_from_a_scene(self):
        from steering import scene_spec as ss
        scene = ss.scene('rig_scene', world={'kind': 'W0', 'markers': [[6., 4., .1]]},
                         start={'x_m': 1., 'y_m': .2, 'yaw_deg': 5., 'steer_deg': 3.}, seeds=[4, 5],
                         maneuver=dict(kind='step', params={}))
        config = tr.rig_config_from_scene(scene, 5)
        self.assertEqual((config.seed, config.stream_key), (5, 'rig_scene'))
        self.assertEqual(config.world.markers, ((6., 4., .1),))
        rig = tr.T1Rig(config, native_executable=native_path())
        self.addCleanup(rig.close)
        self.assertAlmostEqual(rig.plant.x, 1.)
        self.assertAlmostEqual(rig.plant.yaw, math.radians(5.))
        self.assertAlmostEqual(rig.plant.truth()['delta_eq_rad'], math.radians(3.), places=6)
        self.ready(rig)
        with self.assertRaises(tr.RigError):
            tr.rig_config_from_scene(scene.with_changes(backend='L0'), 5)

    def test_profile_level_and_sealed_corridors_are_refused(self):
        from steering import vehicle_profile as vp
        l0 = vp.load_profile().with_overrides({'steer_actuator_level': 'L0'}, label='l0')
        with self.assertRaises(tr.RigError):
            tr.T1Rig(tr.RigConfig(profile=l0), native_executable=native_path())
        sealed = WorldSpec(kind='W2', record='042301', corridor_id='c01', direction='fwd', allow_sealed=True)
        with self.assertRaises(tr.RigError):
            tr.T1Rig(tr.RigConfig(world=sealed), native_executable=native_path())


class OrderingTest(RigCase):
    def test_bridge_grid_uart_arrival_and_line_delivery(self):
        rig = self.rig(seed=2)
        client = self.armed(rig)
        self.hold(rig, client, 1., 40, 100)
        writes = rig.host_writes()
        self.assertTrue(writes)
        unit = v2.byte_units(115200)
        for t_ms, first, last, data, note in writes:
            self.assertEqual(t_ms % tr.BRIDGE_PERIOD_MS, 0, (t_ms, data))
            self.assertIsNotNone(first)
            self.assertGreaterEqual(first, t_ms + 1)
        # a C line written alone in a bridge step: first byte at t + 1, LF at ceil((576 t + 50 n) / 576)
        lone = [w for w in writes if w[3].startswith(b'C ') and not [x for x in writes if x[0] == w[0] and x is not w]]
        self.assertTrue(lone)
        t_ms, first, last, data, _ = lone[0]
        self.assertEqual(first, t_ms + 1)
        self.assertEqual(last, -(-(t_ms * v2.TICK_UNITS + unit * len(data)) // v2.TICK_UNITS))
        for item in rig.serial_log():
            if item['dir'] == 'm2h' and not item['note']:
                self.assertEqual(item['t_ms'], item['last_tick'] + 1)     # LF done tick + 1
        # the host read every status line at a bridge step at or after its delivery time
        for line, received in rig.link.take_status_history():
            self.assertEqual(round(received * 1000) % tr.BRIDGE_PERIOD_MS, 0)

    def test_request_is_answered_by_the_next_bridge_step(self):
        rig = self.rig()
        self.ready(rig)
        rig.run_until((rig.ms // 20 + 1) * 20 / 1000.)    # on the grid, its bridge step not run yet
        start = rig.ms
        reply = rig.request({'op': 'arm', 'owner': 'a', 'vehicle_profile': 'front_steer'})
        self.assertTrue(reply['ok'])
        self.assertEqual(rig.ms, start)                    # answered by the bridge step at this very time
        rig.step(.01)                                      # now 10 ms off the grid
        start = rig.ms
        reply = rig.request({'op': 'status'})
        self.assertEqual(rig.ms, start)                    # status bypasses the queue
        client = rig.client('a')
        client.lease, client.ticket, client.seq = rig.core.control.lease, rig.core.control.ticket, 0
        reply = client.drive(0, 0)
        self.assertTrue(reply['ok'], reply)
        self.assertEqual(rig.ms, start + 10)               # the next bridge step (10 ms later)
        self.assertEqual(rig.ms % 20, 0)

    def test_bridge_phase_and_period_options(self):
        rig = self.rig(bridge_phase_ms=10)
        self.ready(rig)
        self.assertTrue(all(w[0] % 20 == 10 for w in rig.host_writes()))
        with self.assertRaises(tr.RigError):
            tr.RigConfig(bridge_phase_ms=5)


class DriveTest(RigCase):
    def test_arm_drive_steer_sensors_and_trace(self):
        rig = self.rig(seed=4)
        client = self.armed(rig)
        t0 = rig.ms
        reply = self.hold(rig, client, 3., 40, 150)
        self.assertTrue(reply['ok'], reply)
        self.assertGreater(rig.plant.v, .2)
        status = client.status()
        steering = status['steering']
        target = rig.hp.permille_from_mrad(150)
        self.assertEqual(steering['target_permille'], target)
        self.assertLessEqual(abs(steering['measured_permille'] - target), 2 * 20 + 2, steering)
        self.assertEqual(status['requested']['throttle'], 40)
        cs = rig.host_writes(t0, head='C')
        qs = [int(w[3].split(b' ')[1]) for w in cs]
        self.assertEqual(qs, list(range(qs[0], qs[0] + len(qs))))
        packets = rig.poll_sensors()
        scans = [p for p in packets if p['kind'] == 'scan']
        imus = [p for p in packets if p['kind'] == 'imu']
        self.assertEqual(len(rig.poll_sensors()), 0)                    # each packet once
        span = rig.now
        self.assertLessEqual(abs(len(scans) - span * rig.lidar_hz), 1)
        self.assertLessEqual(abs(len(imus) - span * rig.imu_rate_hz), 2)
        self.assertTrue(all(round(p['stamp_s'] * 1000) % 10 == 0 for p in packets))
        self.assertEqual(len(scans[0]['ranges']), fw.LIDAR_RAYS)
        self.assertTrue(any(math.isfinite(r) for r in scans[-1]['ranges']))   # the posts reflect
        self.assertTrue(all(set(p) <= {'kind', 'seq', 'stamp_s', 'arrival_s', 'ranges', 'angle_min', 'angle_increment',
                                       'range_max', 'gyro_z_rad_s', 'accel_m_s2'} for p in packets))
        arrays = rig.trace_arrays()
        n = len(arrays['t_s'])
        self.assertEqual(n, rig.ms // 10)
        for name, array in arrays.items():
            if name not in ('phase_names', 'meta_json'):
                self.assertEqual(len(array), n, name)
        meta = json.loads(bytes(arrays['meta_json']).decode('utf-8'))
        self.assertEqual(meta['schema'], tr.TRACE_SCHEMA)
        self.assertEqual(sorted(meta['truth_keys']), sorted(k for k in arrays if k.startswith('truth_')))
        self.assertIn('hold_measured', meta['steer_target_note'])
        self.assertEqual(float(arrays['cmd_throttle_pct'][-1]), 40.)
        self.assertAlmostEqual(float(arrays['cmd_steer_target_mrad'][-1]), 150.)
        self.assertEqual(int(arrays['host_lease'][-1]), 1)
        self.assertEqual(int(arrays['truth_mcu_state'][-1]), 2)
        self.assertEqual(int(arrays['truth_steer_mode'][-1]), 1)       # ACTIVE
        for key in status:                                              # no truth in what a client reads
            self.assertNotIn('truth', key)

    def test_trace_files_follow_the_fast_sim_format(self):
        rig = self.rig(seed=4)
        self.ready(rig)
        rig.step(.3)
        folder = tempfile.mkdtemp(prefix='steer_t1_rig_trace_')
        self.addCleanup(_cleanup, folder)
        path, sha = rig.write_trace(os.path.join(folder, 'trace.npz'))
        import numpy as np
        with np.load(path, allow_pickle=False) as data:
            arrays = {name: data[name] for name in data.files}
        self.assertEqual(tr.content_sha256({k: v for k, v in arrays.items() if k != 'meta_json'}), sha)
        try:
            from steering import fast_sim
        except Exception as exc:          # fast_sim needs the harness imports (not on Python 3.10, decision R10)
            self.skipTest('fast_sim not importable here: %s' % exc)
        self.assertEqual(fast_sim.content_sha256(arrays), tr.content_sha256(arrays))
        other = os.path.join(folder, 'fast_sim.npz')
        fast_sim.write_npz(other, arrays)
        tr.write_npz(os.path.join(folder, 'rig.npz'), arrays)
        with open(other, 'rb') as a, open(os.path.join(folder, 'rig.npz'), 'rb') as b:
            self.assertEqual(a.read(), b.read())

    def test_client_arm_precheck_and_refusals(self):
        rig = self.rig(eeprom_blank=True)
        rig.run_until_true(lambda: rig.link.ready and rig.link.cal is not None, 1.)
        rig.step(.3)
        client = rig.client('c')
        self.assertEqual(client.arm_precheck(), 'steering_uncalibrated')
        self.assertEqual(client.arm(), {'ok': False, 'error': 'arm_precheck:steering_uncalibrated'})
        self.assertEqual(client.arm(precheck=False), {'ok': False, 'error': 'steering_uncalibrated'})
        rig2 = self.rig(cal_id=2, host_cal_id=1)
        rig2.step(.3)
        self.assertEqual(rig2.client('c').arm(precheck=False), {'ok': False, 'error': 'calibration_mismatch'})
        rig3 = self.rig()
        client3 = self.armed(rig3, 'first')
        self.assertTrue(client3.drive(0, 0)['ok'])
        self.assertEqual(rig3.client('second').arm_precheck(), 'bridge_armed_by_other_client')
        self.assertEqual(rig3.client('second').arm(precheck=False), {'ok': False, 'error': 'control_in_use'})
        self.assertEqual(client3.stop('brake')['ok'], True)
        self.assertIsNone(client3.lease)
        self.assertEqual(client3.drive(10, 0), {'ok': False, 'error': 'client_has_no_lease'})


class StopTimingTest(RigCase):
    """A17 in T1 (SIM time, deterministic): SIM_ACCEPTANCE_KO.md 2 with amendment 7."""

    def test_stop_send_parse_confirm_hold_and_no_restart(self):
        rig = self.rig(seed=7)
        client = self.armed(rig)
        self.hold(rig, client, 2., 40, 200)
        rig.step(.01)                                      # the request comes between two bridge steps
        t_req = rig.ms
        client.stop('brake')
        rig.step(.1)
        stops = rig.host_writes(t_req, head='STOP')
        self.assertEqual(len(stops), 1)
        t_send, arrival = stops[0][0], stops[0][2]
        self.assertLessEqual(t_send - t_req, 20)                                    # <= one bridge tick
        self.assertIs(rig.host_writes(t_send)[0][3], stops[0][3])                   # first line of that step
        self.assertEqual(self.first_change(rig, 'drive_l', 0, t_send), arrival)     # 0 servo ticks after parsing
        self.assertEqual(self.first_change(rig, 'mcu_state', 1, t_send), arrival)
        confirmed = rig.run_until_true(lambda: rig.core.steering_host()['stop_state'] == 'confirmed', .2)
        self.assertTrue(confirmed)
        self.assertLessEqual(rig.ms - t_send, 2 * STATUS_MS + 20)                   # <= 2 status periods (+ read)
        hstop = rig.cal['params']['hstop']
        rig.run_until((arrival + hstop + 200) / 1000.)
        self.assertEqual(self.first_change(rig, 'steer_en', 0, arrival), arrival + hstop)
        self.assertEqual(self.first_change(rig, 'steer_mode', 2, t_send), arrival)  # HOLD at the STOP tick
        self.assertEqual([w for w in rig.host_writes(t_send) if w[3].startswith((b'C ', b'ARM'))], [])
        self.assertEqual(rig.mcu_channel_at('drive_l', rig.ms - 1), 0)
        self.assertEqual(rig.core.control.reason, 'operator_stop')

    def test_stop_unconfirmed_resend_grid_and_latch(self):
        rig = self.rig(seed=8)
        client = self.armed(rig)
        self.hold(rig, client, .5, 40, 0)
        rig.fault('mcu_stall', duration_s=.8)
        rig.step(.02)
        client.stop('brake')
        rig.step(.6)
        stops = [w[0] for w in rig.host_writes(head='STOP')][-6:]
        t0 = stops[0]
        grid = [t0 + 100 + 50 * k for k in range(5)]
        self.assertEqual(stops[1:], [g + (-g) % 20 for g in grid])                   # amendment 7
        latch = rig.core.latches['stop_unconfirmed']
        self.assertEqual(int(round(latch['since'] * 1000)), t0 + 400)

    def test_ttl_and_watchdog_when_the_bridge_stalls(self):
        rig = self.rig(seed=9)
        client = self.armed(rig)
        self.hold(rig, client, .6, 40, 0)
        rig.fault('bridge_stall', duration_s=1.)
        stall = rig.ms
        rig.step(1.)
        last_c = rig.host_writes(head='C')[-1]
        ttl = int(last_c[3].split(b' ')[2])
        expiry = last_c[2] + ttl
        self.assertEqual(self.first_change(rig, 'drive_l', 0, stall), expiry)       # A17: TTL -> drive 0
        f_ttl = [line for line in rig.mcu_lines(('F',), stall) if line.text.startswith('F TTL')]
        self.assertEqual([line.queued_tick for line in f_ttl], [expiry])
        last_input = max(w[2] for w in rig.host_writes() if w[2] is not None)
        wd = rig.cal['params']['wd']
        self.assertEqual([line.queued_tick for line in rig.mcu_lines(('W',), stall)], [last_input + wd + 1])
        self.assertEqual(self.first_change(rig, 'mcu_state', 1, stall), last_input + wd + 1)
        back = rig.ms
        rig.step(.3)                                                                  # the bridge runs again
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'input_expired')                    # the 0.75 s input deadline
        self.assertIn('WATCHDOG', rig.core.steering_block()['faults'])                # the MCU latch (A17)
        self.assertEqual([w for w in rig.host_writes(back) if w[3].startswith((b'C ', b'ARM'))], [])   # no restart


class HoldMeasuredTest(RigCase):
    """Decision R13-2 in firmware mode: a hold_measured stop holds the MCU filtered position P[t0]."""

    def test_stop_while_slewing_holds_the_filtered_position(self):
        rig = self.rig(seed=11)
        client = self.armed(rig)
        self.hold(rig, client, 1., 30, 0)
        client.drive(30, 350)                               # a large step: the servo is still slewing at the STOP
        rig.step(.15)
        client.stop('brake')
        rig.step(.1)
        stop = rig.host_writes(head='STOP')[-1]
        t0 = stop[2]                                        # the STOP tick at the MCU
        p0 = rig.mcu_channel_at('servo_pos', t0)
        tgt_before = rig.mcu_channel_at('servo_tgt', t0 - 1)
        self.assertEqual(rig.mcu_channel_at('servo_tgt', t0), p0)
        self.assertEqual(rig.mcu_channel_at('servo_cmd', t0), p0)
        self.assertGreater(abs(tgt_before - p0), 40)        # the target moved back to where the steering was
        rig.step(1.)
        self.assertTrue(all(v == p0 for t, v in rig.mcu_channel('servo_tgt') if t0 <= t < t0 + 1000))
        lines = [s for tick, s in ((line.queued_tick, line) for line in rig.mcu_lines(('S',), t0 + 1))]
        status = [line for line in lines if line.status()['st'] == 1 and line.queued_tick < t0 + 1000]
        self.assertTrue(status)
        self.assertTrue(all(line.status()['tgt'] == p0 for line in status))
        arrays = rig.trace_arrays()
        import numpy as np
        t = np.round(arrays['t_s'] * 1000).astype(int)
        row = int(np.flatnonzero(t >= t0 + 10)[0])          # first row after the STOP tick
        self.assertAlmostEqual(float(arrays['steer_target_rad'][row]), rig.permille_to_rad(p0), places=12)
        self.assertAlmostEqual(float(arrays['truth_mcu_pos_rad'][row - 1]), rig.permille_to_rad(
            rig.mcu_channel_at('servo_pos', t[row - 1] - 1)), places=12)
        # review P4R-16: the rig writes the hold reference itself (same values as the MCU position column); there is
        # no column named like the fast_sim servo view (a different quantity)
        self.assertTrue(np.array_equal(arrays['truth_hold_position_rad'], arrays['truth_mcu_pos_rad']))
        self.assertNotIn('truth_servo_view_rad', arrays)
        hold_err = max(abs(v - p0) for tk, v in rig.mcu_channel('servo_pos') if t0 + 300 <= tk < t0 + 1000)
        self.assertLessEqual(hold_err, 2 * rig.cal['params']['db'])          # A17 hold error in MCU terms


class FaultHookTest(RigCase):
    def assert_mcu_fault(self, rig, codes_expected, client, step_at_s=None, seconds=3.):
        """Drive at 30 % with steering 0, from step_at_s with a large steering step (the servo pushes), until the MCU
        fault stops it. codes_expected: the MCU fault codes any of which shows the injected fault."""
        end = rig.ms + int(seconds * 1000)
        while rig.ms < end and client.lease is not None:
            steer = 350 if step_at_s is not None and rig.now >= step_at_s else 0
            reply = client.drive(30, steer)
            if not reply.get('ok'):
                break
            rig.step(.1)
        rig.step(.2)
        codes = [line.text.split(' ')[1] for line in rig.mcu_lines(('F',))]
        self.assertTrue(set(codes_expected) & set(codes), (codes_expected, codes))
        self.assertIn('mcu_disarmed', rig.core.latches)
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.mcu_channel_at('drive_l', rig.ms - 1), 0)
        return codes

    def test_model_faults(self):
        # (hook, params, MCU codes that show it, steering step time): a stuck driver shows as DRV_STUCK or POLARITY
        # (both are the hardware latches that need CAL POL, spec 12.1-71); a jammed actuator as STALL or TRACK
        cases = (('pot_open', {}, ('POT_OPEN',), None), ('pot_short', {}, ('POT_SHORT',), None),
                 ('pot_jump', {'counts': 40, 'duration_s': .2}, ('POT_JUMP',), None),
                 ('pot_noise', {}, ('POT_NOISE',), None), ('polarity_reversed', {}, ('POLARITY',), 1.3),
                 ('driver_stuck_on', {'pwm': .8}, ('DRV_STUCK', 'POLARITY'), None),
                 ('stall', {}, ('STALL', 'TRACK'), 1.3))
        for kind, params, codes, step_at in cases:
            with self.subTest(kind=kind):
                rig = self.rig(seed=21, faults=(tr.RigFault(kind, at_s=1.2, params=params),))
                client = self.armed(rig)
                self.assert_mcu_fault(rig, codes, client, step_at_s=step_at)

    def test_thermal_with_a_slow_actuator(self):
        # The host's steering_not_following (3 deg for 1.5 s) stops a slow actuator first with the placeholder host
        # profile; with that host rule relaxed here the MCU's own THERMAL proxy (5 s of >= 80 % output in 30 s) trips.
        rig = self.rig(seed=22, faults=(tr.RigFault('thermal', at_s=.3),),
                       host_overrides={'host_faults': {'track_duration_s': 30.}})
        client = self.armed(rig)
        for steer in (380, -380, 380, -380):
            for _ in range(30):
                if client.lease is None:
                    break
                client.drive(0, steer)
                rig.step(.1)
        rig.step(.2)
        self.assertIn('THERMAL', [line.text.split(' ')[1] for line in rig.mcu_lines(('F',))])

    def test_link_loss_trips_the_mcu_watchdog(self):
        rig = self.rig(seed=23)
        client = self.armed(rig)
        self.hold(rig, client, .5, 40, 0)
        rig.fault('link_loss', duration_s=1., direction='to_mcu')
        cut = rig.ms
        rig.step(1.2)
        self.assertEqual(len(rig.mcu_lines(('W',), cut)), 1)
        self.assertIn('mcu_disarmed', rig.core.latches)
        self.assertTrue(any(item['note'].endswith(':lost') for item in rig.serial_log() if item['dir'] == 'h2m'))

    def test_half_frame_crc_and_range_errors(self):
        rig0 = self.rig(seed=24)
        client0 = self.armed(rig0)
        self.hold(rig0, client0, .3, 30, 0)
        q_before = rig0.link.status['q']
        rig0.fault('half_frame')                             # 'C 1 30' without LF, then the bridge's next line
        rig0.step(.2)
        rejected = [line.text.split('*')[0] for line in rig0.mcu_lines(('N',))]
        self.assertTrue(rejected and rejected[0].split(' ')[2] in ('nocrc', 'syntax', 'crc'), rejected)
        self.assertNotEqual(rig0.link.status['q'], 1)        # the half line was never applied
        self.assertGreater(rig0.link.status['q'], 0)
        self.assertGreaterEqual(rig0.link.status['q'], q_before)
        self.assertIn('c_rejected', rig0.core.latches)       # a rejected C with a q stops the bridge
        self.assertIsNone(rig0.core.control.lease)
        rig = self.rig(seed=24)
        client = self.armed(rig)
        self.hold(rig, client, .3, 30, 0)
        rig.fault('crc_error')
        self.hold(rig, client, .2, 30, 0)
        self.assertTrue(any(line.text.startswith('N - crc') for line in rig.mcu_lines(('N',))))
        self.assertIsNotNone(client.lease)
        rig.fault('crc_error', count=5)                     # a burst of crcn bad lines: the MCU latches CRC
        self.hold(rig, client, .4, 30, 0)
        rig.step(.2)
        self.assertIn('CRC', [line.text.split(' ')[1] for line in rig.mcu_lines(('F',))])
        self.assertIn('mcu_disarmed', rig.core.latches)
        rig2 = self.rig(seed=25)
        client2 = self.armed(rig2)
        self.hold(rig2, client2, .2, 30, 0)
        rig2.fault('range_error')
        self.hold(rig2, client2, .3, 30, 0)
        rig2.step(.1)
        self.assertTrue(any(line.text.split('*')[0].endswith(' range') for line in rig2.mcu_lines(('N',))))
        self.assertIn('c_rejected', rig2.core.latches)
        self.assertIsNone(rig2.core.control.lease)

    def test_status_line_loss_latches_stale(self):
        rig = self.rig(seed=26)
        client = self.armed(rig)
        self.hold(rig, client, .3, 30, 0)
        rig.fault('status_loss', duration_s=.4)
        start = rig.ms
        self.hold(rig, client, .6, 30, 0)
        rig.run_until((start + 600) / 1000.)
        self.assertIn('steering_status_stale', rig.core.latches)
        self.assertGreaterEqual(rig.counters['status_lines_lost'], 7)              # 0.4 s of 20 Hz lines

    def test_mcu_reset_and_power_cut(self):
        for kind, params in (('mcu_reset', {'reason': 4}), ('power_cut', {'off_s': .3})):
            with self.subTest(kind=kind):
                rig = self.rig(seed=27)
                client = self.armed(rig)
                self.hold(rig, client, .8, 40, 0)
                rig.fault(kind, **params)
                at = rig.ms
                self.hold(rig, client, 1.5, 40, 0)
                rig.step(1.)
                self.assertEqual(self.first_change(rig, 'drive_en', 0, at), at)       # pins float: coast
                events = rig.trace_arrays()['truth_drive_event']
                self.assertIn(tr.DRIVE_EVENT_CODES['coast'], events.tolist())         # the plant coasted
                self.assertIn('mcu_reset_detected', rig.core.latches)
                self.assertIsNone(rig.core.control.lease)
                self.assertEqual(rig.link.boot_id, rig.fw.boot_id(1))

    def test_late_drive_after_stop_and_wall_clock_jump(self):
        rig = self.rig(seed=28)
        client = self.armed(rig)
        self.hold(rig, client, .4, 30, 0)
        reply = client.late_drive_after_stop(30, 0)
        self.assertEqual(reply, {'ok': False, 'error': 'stop_barrier'})
        rig2 = self.rig(seed=28)
        client2 = self.armed(rig2)
        self.hold(rig2, client2, .4, 30, 0)
        updated = rig2.core.snapshot['updated_at']
        rig2.fault('wall_clock_jump', jump_s=-3600.)
        before = len(rig2.host_writes(head='C'))
        self.hold(rig2, client2, .5, 30, 0)
        self.assertLess(rig2.core.snapshot['updated_at'], updated)              # the wall clock went back
        self.assertGreaterEqual(len(rig2.host_writes(head='C')) - before, 9)    # C at 20 Hz continued
        self.assertIsNotNone(client2.lease)

    def test_usb_unplug_motor_power_and_sensor_blackout(self):
        rig = self.rig(seed=29)
        client = self.armed(rig)
        self.hold(rig, client, .3, 30, 0)
        rig.fault('usb_unplug', duration_s=.5)
        rig.step(3.5)
        self.assertIn('hardware_fault', rig.core.latches)
        self.assertEqual(rig.link.state, 'ready')            # reopened; the banner came after INFO
        rig2 = self.rig(seed=30)
        client2 = self.armed(rig2)
        rig2.fault('motor_power_off')
        reply = None
        for _ in range(30):
            reply = client2.drive(0, 380)
            if not reply.get('ok'):
                break
            rig2.step(.1)
        self.assertTrue({'STALL', 'TRACK'} & {line.text.split(' ')[1] for line in rig2.mcu_lines(('F',))})
        rig3 = self.rig(seed=31)
        self.ready(rig3)
        rig3.poll_sensors()
        rig3.fault('sensor_blackout', duration_s=.5, which='lidar')
        t0 = rig3.now
        rig3.step(.5)
        packets = rig3.poll_sensors()
        self.assertFalse([p for p in packets if p['kind'] == 'scan' and t0 <= p['stamp_s'] < t0 + .5])
        self.assertTrue([p for p in packets if p['kind'] == 'imu'])

    def test_unknown_and_model_faults_at_runtime_are_refused(self):
        rig = self.rig()
        with self.assertRaises(tr.RigError):
            rig.fault('pot_open')
        with self.assertRaises(tr.RigError):
            rig.fault('no_such_fault')
        with self.assertRaises(tr.RigError):
            tr.RigConfig(faults=(tr.RigFault('no_such_fault'),))


class ReviewFixRigTest(RigCase):
    """Review P4R-05 / RS-04 (bridge process restart), RS-13 (reset when the port is opened again), P4R-15 (host link
    settings and an injected host defect), RS-05 (the client keeps the rig private)."""

    def test_bridge_restart_is_a_new_process(self):
        rig = self.rig(seed=31)
        client = self.armed(rig)
        self.hold(rig, client, .5, 30, 0)
        old_core = rig.core
        rig.fault('bridge_restart', down_s=1.)
        rig.step(.02)
        self.assertTrue(rig.bridge_down())
        with self.assertRaises(tr.BridgeUnavailable):
            client.drive(30, 0)
        with self.assertRaises(OSError):                    # a socket-like error
            client.status()
        rig.run_until(rig.now + 1.2)
        self.assertFalse(rig.bridge_down())
        self.assertIsNot(rig.core, old_core)
        self.assertEqual(rig.bridge_processes, 2)
        self.ready(rig)
        self.assertIsNone(rig.core.control.lease)           # the new process holds no lease
        reply = client.drive(30, 0)                         # the old lease is unknown to the new process
        self.assertFalse(reply.get('ok'))
        self.assertEqual(reply.get('error'), 'control_not_owned')
        self.assertEqual(rig.mcu_channel_at('mcu_state', rig.ms - 1), 1)      # DISARMED (link watchdog)
        stop_after = rig.host_writes(since_ms=[t for kind, t in rig.bridge_events if kind == 'start'][0], head='STOP')
        self.assertTrue(stop_after)                         # the session STOP of the new process

    def test_usb_replug_with_a_port_open_reset_boots_again(self):
        rig = self.rig(seed=32, port_open_reset=True)
        client = self.armed(rig)
        self.hold(rig, client, .3, 30, 0)
        boot_before = rig.link.boot_id
        rig.fault('usb_unplug', duration_s=.5)
        rig.run_until(rig.now + 3.)
        self.ready(rig, 3.)
        self.assertNotEqual(rig.link.boot_id, boot_before)  # the reopen reset the MCU (spec 10.7)
        self.assertIsNone(rig.core.control.lease)

    def test_host_link_settings_and_host_defect(self):
        rig = self.rig(seed=33, host_hw_profile='ubts_steer_other_board')
        rig.run_until(.5)
        self.assertIn('hardware_profile_mismatch', rig.core.latches)
        self.assertFalse(rig.client('x').arm(precheck=False).get('ok'))
        rig = self.rig(seed=34, host_simulated=False)
        rig.run_until(.5)
        self.assertIn('sim_identity_mismatch', rig.core.latches)
        rig = self.rig(seed=35)
        client = self.armed(rig)
        self.hold(rig, client, .3, 30, 0)
        rig.fault('host_defect')
        rig.step(.05)
        self.assertIn('internal_error', rig.core.latches)
        self.assertIsNone(rig.core.control.lease)
        with self.assertRaises(tr.RigError):
            tr.RigConfig(host_hw_profile='')

    def test_client_keeps_the_rig_private(self):
        rig = self.rig(seed=36)
        client = rig.client('private')
        self.assertFalse(hasattr(client, 'rig'))


class McuStateMismatchTest(RigCase):
    """Open P3 item 1 (A19 host part): an MCU ARMED without a host lease latches mcu_state_mismatch and stops."""

    def test_foreign_arm_on_the_wire(self):
        rig = self.rig(seed=41)
        self.ready(rig)
        rig.step(.2)
        t = rig.ms
        rig.fault('foreign_write', text='ARM ' + rig.link.boot_id, crc=True)   # a second writer (CLI) arms the MCU
        armed = rig.run_until_true(lambda: rig.mcu_channel_at('mcu_state', rig.ms - 1) == 2, .1)
        self.assertTrue(armed)
        rig.step(.3)
        latch = rig.core.latches['mcu_state_mismatch']
        self.assertIn('ARMED', latch['detail'])
        self.assertLessEqual(int(round(latch['since'] * 1000)) - t, STATUS_MS + 2 * 20 + 10)
        stop = [w for w in rig.host_writes(t, head='STOP')]
        self.assertTrue(stop)
        self.assertEqual(rig.mcu_channel_at('mcu_state', rig.ms - 1), 1)        # the bridge's STOP disarmed it
        self.assertEqual(rig.core.snapshot['state'], 'fault')
        self.assertIsNone(rig.core.control.lease)
        client = rig.client('operator')
        self.assertTrue(client.arm(precheck=False)['ok'])
        self.assertTrue(client.wait_armed())
        self.assertNotIn('mcu_state_mismatch', rig.core.latches)                  # operator re-arm clears it


class MotionSourceTest(RigCase):
    """Open P3 item 4: the executor side gives steering_ineffective its motion source through op motion."""

    def test_motion_op_from_the_lease_holder(self):
        from field_response import forward_curve
        for name, gyro_scale, expect in (('consistent', 1., False), ('no_yaw', 0., True)):
            with self.subTest(name):
                rig = self.rig(seed=51)
                client = self.armed(rig)
                # an open-loop speed estimate from the drive curve at the commanded duty (a model value, not truth)
                speed = forward_curve(50. * rig.params.battery_voltage_scale, rig.params.curve_params)
                end = rig.ms + 3000
                while rig.ms < end and client.lease is not None:
                    client.drive(50, 200)
                    for _ in range(5):
                        for packet in rig.poll_sensors():
                            if packet['kind'] == 'imu' and client.lease is not None:
                                reply = client.observe_motion(gyro_scale * packet['gyro_z_rad_s'], speed)
                                self.assertTrue(reply.get('ok') or rig.core.control.lease is None, reply)
                        rig.step(.02)
                self.assertEqual('steering_ineffective' in rig.core.latches, expect)
                self.assertEqual(client.lease is None, expect)
        rig = self.rig()
        self.ready(rig)
        other = rig.client('nobody')
        other.lease = 'not-a-lease'
        self.assertEqual(other.observe_motion(0., 0.), {'ok': False, 'error': 'control_not_owned'})


class DeterminismTest(RigCase):
    """A8 for the rig: same scene and seed -> identical serial bytes and trace SHA-256; another seed differs."""

    def test_same_seed_same_bytes_other_seed_differs(self):
        first = scenario(seed=61, executable=native_path())
        second = scenario(seed=61, executable=native_path())
        other = scenario(seed=62, executable=native_path())
        self.assertEqual(first, second)
        self.assertNotEqual(first[0], other[0])
        self.assertNotEqual(first[1], other[1])
        env = dict(os.environ, PYTHONHASHSEED='12345', PYTHONDONTWRITEBYTECODE='1')
        code = ('import json, sys; sys.dont_write_bytecode = True; import test_steering_t1_rig as t; '
                'print(json.dumps(t.scenario(seed=61, executable=sys.argv[1])))')
        out = subprocess.run([sys.executable, '-B', '-c', code, native_path()], cwd=UBTS_DIR, env=env,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=300)
        self.assertEqual(out.returncode, 0, out.stderr.decode('utf-8', 'replace')[-2000:])
        self.assertEqual(tuple(json.loads(out.stdout.decode('ascii').strip().splitlines()[-1])), first)


def scenario(seed, executable):
    """A short deterministic run (arm, drive with steering changes, a CRC error, STOP); (serial SHA, trace SHA)."""
    rig = tr.T1Rig(tr.RigConfig(seed=seed, world=POSTS), native_executable=executable)
    try:
        rig.run_until_true(lambda: rig.core.steering_host()['stop_state'] == 'confirmed', 3.)
        client = rig.client('det')
        client.arm()
        client.wait_armed()
        for steer in (100, -150, 50):
            end = rig.ms + 600
            while rig.ms < end:
                client.drive(40, steer)
                rig.step(.1)
        rig.fault('crc_error')
        client.drive(40, 0)
        rig.step(.2)
        client.stop('brake')
        rig.step(.5)
        return rig.serial_log_sha256(), rig.trace_sha256()
    finally:
        rig.close()


class CloseTest(RigCase):
    def test_close_ends_the_native_process(self):
        rig = self.rig()
        self.ready(rig)
        backend = rig.fw
        self.assertTrue(backend.running)
        rig.close()
        self.assertFalse(backend.running)
        self.assertEqual(backend.exit_code, 0)
        rig.close()                                          # idempotent
        with self.assertRaises(tr.RigError):
            rig.step(.01)

    def test_failed_constructor_ends_the_native_process(self):
        made = []

        class Recorded(tr.FirmwareLink):
            def __init__(self, options):
                super().__init__(options)
                made.append(self)

        original = tr.FirmwareLink
        tr.FirmwareLink = Recorded
        try:
            with self.assertRaisesRegex(tr.RigError, 'not on the 10 ms plant grid'):
                tr.T1Rig(tr.RigConfig(world=POSTS, lidar_hz=7.), native_executable=native_path())
        finally:
            tr.FirmwareLink = original
        self.assertEqual(len(made), 1)
        backend = made[0].fw
        self.assertIsNotNone(backend)                     # the native process had started
        self.assertFalse(backend.running)


class RasterWorldTest(RigCase):
    """W2: a raster corridor of the development set (needs the external maps and the raster LiDAR's library,
    which is broken on the WSL 3.10 (R10); never a sealed one). Skips only on missing external data or a
    RigDependencyError; any other error of the rig fails the test."""

    def test_rig_on_a_development_corridor(self):
        from closed_loop import raster_corridor as rc
        try:
            catalogue = rc.load_catalogue()
        except OSError as exc:
            self.skipTest('W2 needs the corridor catalogue (external data): %s' % exc)
        from steering.scene_spec import sealed_status
        pick = None
        for entry in catalogue.get('corridors') or ():
            record, cid = entry.get('record'), entry.get('corridor_id')
            status = (entry.get('directions') or {}).get('fwd', {}).get('status')
            if record and cid and status == 'included' and sealed_status(record, cid, 'fwd') is None:
                pick = (record, cid)
                break
        if pick is None:
            self.skipTest('no development corridor found in the catalogue')
        world = WorldSpec.from_dict({'kind': 'W2', 'record': pick[0], 'corridor_id': pick[1], 'direction': 'fwd'})
        try:
            rig = self.rig(world=world, start_pose=(1., 0., 0.))
        except tr.RigDependencyError as exc:
            self.skipTest(str(exc))
        client = self.armed(rig)
        self.hold(rig, client, 1., 30, 0)
        scans = [p for p in rig.poll_sensors() if p['kind'] == 'scan']
        self.assertTrue(scans)
        self.assertTrue(any(math.isfinite(r) for r in scans[-1]['ranges']))
        arrays = rig.trace_arrays()
        self.assertGreater(float(arrays['truth_field_x_m'][-1]), 1.)


if __name__ == '__main__':
    unittest.main()
