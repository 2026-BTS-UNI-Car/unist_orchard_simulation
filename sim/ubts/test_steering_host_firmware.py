"""Host bridge <-> steering firmware V3 end to end in T1 (design 8.3 P3, SIM_ACCEPTANCE_KO.md A17 P3 part).

The real host code (dashboard/scripts: steer_bridge_core + steer_control_core + esp_motor_v3 with its
MemoryTransport) talks to the real firmware (firmware/ubts_steer_v3 in the native lockstep v2 process,
FirmwareSteeringBackend) with one injected SIM clock:

  * the bridge core steps every 20 ms (the wrapper loop); its writes go to the MCU with the UART arrival rule of
    LOCKSTEP_PROTOCOL_KO.md 11.7 (115200 baud), written at the step time;
  * the MCU runs 1 ms ticks between two bridge steps; every MCU byte reaches the host at the end of the tick in
    which its last bit left (done tick + 1 ms);
  * the vehicle speed and the stopped flag given to the actuator model come from a small open-loop stub (speed
    follows 0.9 m/s x drive duty with 1 m/s^2 up and 2 m/s^2 down). It is not the plant: the full A17(F) with
    the steered plant, the estimator and the executor is P4.

Change channels, truth and MCU line ticks are read only by this test (evaluation). The host sees serial bytes.
All numbers are placeholders: the results are placeholder-based SIM results, not field validation.
unittest style: pytest (3.12, 3.13) and `python3 -m unittest` (3.10). No file is written inside the work tree
(the native program is built into UBTS_STEER_BUILD_DIR or a temporary folder).
"""
import atexit
import importlib
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

from steering import host_profile_export as hpe
from steering import vehicle_profile as vp
from steering.actuator_link import FirmwareSteeringBackend
from steering.native import build as native_build
from steering.native import fw_eeprom
from steering.native import lockstep_v2 as v2

DASHBOARD = Path(__file__).resolve().parents[2] / 'dashboard'
BRIDGE_MS = 20                       # host bridge loop period (steer_rover_bridge.py)
STATUS_MS = 50                       # 1000 / sthz (placeholder 20 Hz)
BOOT_SEED = 1
CHIP_ID = 0x5A1B2C
OWNER = 'operator-a'
ARM_REQUEST = {'op': 'arm', 'owner': OWNER, 'vehicle_profile': 'front_steer'}

_STATE = {}


def _cleanup_temp(path):
    shutil.rmtree(path, ignore_errors=True)


def host_modules():
    """(esp_motor_v3, steer_bridge_core) of this work tree's dashboard, or SkipTest with the reason.

    The host never imports sim code; this test imports the host. Another 'scripts' package may already be on the
    path (the SIM image carries an older dashboard copy): the host modules are imported with this tree's dashboard
    first, then sys.path and the earlier 'scripts*' modules are put back, so other tests keep what they had.
    """
    if 'host' in _STATE:
        if isinstance(_STATE['host'], str):
            raise unittest.SkipTest(_STATE['host'])
        return _STATE['host']
    if not (DASHBOARD / 'scripts' / 'steer_bridge_core.py').is_file():
        _STATE['host'] = 'dashboard/scripts/steer_bridge_core.py not found next to sim/ubts (%s)' % DASHBOARD
        raise unittest.SkipTest(_STATE['host'])
    saved_path = list(sys.path)
    saved = {name: module for name, module in sys.modules.items() if name == 'scripts' or name.startswith('scripts.')}
    for name in saved:
        del sys.modules[name]
    try:
        sys.path.insert(0, str(DASHBOARD))
        v3 = importlib.import_module('scripts.esp_motor_v3')
        bc = importlib.import_module('scripts.steer_bridge_core')
        _STATE['host'] = (v3, bc)
    except Exception as exc:   # any import failure is a skip reason, not an error of this test
        _STATE['host'] = 'host code from %s does not import: %s: %s' % (DASHBOARD, type(exc).__name__, exc)
    finally:
        sys.path[:] = saved_path
        for name in [n for n in sys.modules if n == 'scripts' or n.startswith('scripts.')]:
            del sys.modules[name]
        sys.modules.update(saved)
    if isinstance(_STATE['host'], str):
        raise unittest.SkipTest(_STATE['host'])
    return _STATE['host']


def native_path():
    """Path of the native program with firmware mode; SkipTest without firmware sources or a C++ compiler."""
    if 'native' not in _STATE:
        try:
            present = native_build.firmware_present()
        except native_build.NativeBuildError as exc:
            _STATE['native'] = str(exc)
        else:
            if not present:
                _STATE['native'] = 'firmware sources missing: %s (set %s)' % (
                    native_build.firmware_dir(), native_build.ENV_FIRMWARE_DIR)
            else:
                try:
                    native_build.find_compiler()
                    build_dir = os.environ.get(native_build.ENV_BUILD_DIR)
                    if not build_dir:
                        build_dir = tempfile.mkdtemp(prefix='steer_host_fw_build_')
                        atexit.register(_cleanup_temp, build_dir)
                    _STATE['native'] = native_build.ensure_built(build_dir=build_dir).path
                except native_build.NativeBuildError as exc:
                    _STATE['native'] = 'native steering program not available: %s' % exc
    value = _STATE['native']
    if not os.path.isfile(value):
        if native_build.firmware_required():   # a checkout that tracks the firmware must not skip these tests
            raise AssertionError('%s is set: %s' % (native_build.ENV_REQUIRE_FIRMWARE, value))
        raise unittest.SkipTest(value)
    return value


def placeholder_profile():
    if 'profile' not in _STATE:
        _STATE['profile'] = vp.load_profile()
    return _STATE['profile']


class Write:
    """One host write as the MCU receives it: bridge step ms, LF arrival tick (11.7), bytes."""

    __slots__ = ('ms', 'arrival', 'data')

    def __init__(self, ms, arrival, data):
        self.ms, self.arrival, self.data = ms, arrival, data

    @property
    def head(self):
        return self.data.split(b'*')[0].split(b' ')[0].strip().decode('ascii')

    def c_fields(self):
        """(q, ttl, L, R, S) of a C line."""
        body = self.data.rstrip(b'\n').split(b'*')[0].decode('ascii')
        _, q, ttl, left, right, steer = body.split(' ')
        return int(q), int(ttl), int(left), int(right), int(steer)


class Rig:
    """Bridge core (host) + V3 firmware (native lockstep v2) with one SIM clock in integer milliseconds."""

    def __init__(self, test, *, seed=0, eeprom_cal_id=1, host_cal_id=1, boot_delay_ticks=0, boot_garbage_hex='',
                 port_open_reset=False, injections=(), record_overrides=None, host_overrides=None):
        self.v3, self.bc = host_modules()
        self.profile = placeholder_profile()
        self.seed = seed
        startup = v2.firmware_startup_config(self.profile, seed)   # the calibration marks of this run (11.11)
        image, self.cal = fw_eeprom.calibrated_eeprom(startup, self.profile, cal_id=eeprom_cal_id)
        if record_overrides:
            params = dict(self.cal['params'], **record_overrides)
            self.cal = dict(self.cal, params=params)
            image = fw_eeprom.image_hex(fw_eeprom.encode_record(
                eeprom_cal_id, self.cal['raw_l'], self.cal['raw_c'], self.cal['raw_r'], self.cal['soft_l'],
                self.cal['soft_r'], params=params))
        self.backend = FirmwareSteeringBackend(
            self.profile, seed, injections=injections, executable=native_path(),
            firmware=dict(boot_seed=BOOT_SEED, chip_id=CHIP_ID, eeprom_hex=image, adc_mode='model',
                          boot_delay_ticks=boot_delay_ticks, boot_garbage_hex=boot_garbage_hex))
        test.addCleanup(self.backend.close)
        self.backend.reset()
        data = hpe.build(self.profile, cal_id=host_cal_id)
        for section, values in (host_overrides or {}).items():
            data[section] = dict(data[section], **values)
        self.hp = self.bc.HostProfile.from_bytes(hpe.to_bytes(data), origin='host firmware test')
        self.ms = 0
        self.transport = self.v3.MemoryTransport(clock=self.clock)
        if port_open_reset:   # NodeMCU DTR/RTS reset when the host opens the port (spec 10.7)
            self.transport.on_open.append(lambda: self.backend.event('port_open_reset', tick=self.ms))
        self.link = self.v3.EspSteerV3(self.transport, self.clock, simulated=True, expect_hw=self.hp.hw_profile,
                                       banner_timeout_s=self.hp.banner_timeout_s)
        self.core = self.bc.SteerBridgeCore(self.link, self.hp, clock=self.clock, wall_clock=lambda: 1.0e9)
        self.writes = []         # Write, every host line that reached the wire towards the MCU
        self.lost_writes = []    # (ms, bytes) dropped by drop_to_mcu
        self.dispatched = []     # (ms, request op, reply) of requests answered by a bridge step
        self.truth = []          # (ms at the end of the step, delta_eq_rad)
        self.drop_to_mcu = False
        self.host_frozen = False
        self.speed = 0.0
        self._tx_pos = 0

    # -- clock and loop
    def clock(self):
        return self.ms / 1000.0

    def _vehicle(self):
        last = self.backend.last
        duty = 0.0 if last is None else (abs(last.base.drive_duty_left) + abs(last.base.drive_duty_right)) / 2.0
        target = 0.9 * duty
        dt = BRIDGE_MS / 1000.0
        if self.speed < target:
            self.speed = min(target, self.speed + 1.0 * dt)
        else:
            self.speed = max(target, self.speed - 2.0 * dt)
        return self.speed, self.speed == 0.0 and duty == 0.0

    def tick(self):
        start = self.ms
        if not self.host_frozen:
            self.core.step()
        for _, data in self.transport.device.take_timed():
            if self.drop_to_mcu:
                self.lost_writes.append((start, data))
                continue
            arrivals = self.backend.send(data, tick=start, pace='uart')
            self.writes.append(Write(start, arrivals[-1], data))
        speed, stopped = self._vehicle()
        self.backend.run(BRIDGE_MS, step_ticks=BRIDGE_MS, vehicle_speed_m_s=speed, vehicle_stopped=stopped)
        self.ms += BRIDGE_MS
        self.truth.append((self.ms, self.backend.last.base.delta_eq_rad))
        new = self.backend.tx_bytes[self._tx_pos:]
        self._tx_pos = len(self.backend.tx_bytes)
        chunk, at = bytearray(), None
        for entry in new:
            t = (entry.done_tick + 1) / 1000.0
            if at is not None and t != at:
                self.transport.device.send(bytes(chunk), at=at)
                chunk = bytearray()
            chunk.append(entry.byte)
            at = t
        if chunk:
            self.transport.device.send(bytes(chunk), at=at)

    def run(self, ms):
        end = self.ms + ms
        while self.ms < end:
            self.tick()

    def run_until(self, predicate, limit_ms):
        end = self.ms + limit_ms
        while self.ms < end:
            if predicate():
                return True
            self.tick()
        return predicate()

    def submit(self, request):
        """Queue a request; the next bridge step answers it. Returns the reply."""
        future = self.core.submit(request)
        start = self.ms
        self.tick()
        if not future.done():
            raise AssertionError('request not answered by one bridge step: %r' % (request,))
        reply = future.result()
        self.dispatched.append((start, request.get('op'), reply))
        return reply

    # -- views (evaluation only)
    def ready(self, limit_ms=3000):
        ok = self.run_until(lambda: self.core.steering_host()['stop_state'] == 'confirmed', limit_ms)
        if not ok:
            raise AssertionError('no confirmed session STOP: %r' % (self.core.steering_host(),))

    def host_heads(self, since=0, until=None):
        return [w.head for w in self.writes if w.ms >= since and (until is None or w.ms < until)]

    def c_writes(self, since=0, until=None):
        return [w for w in self.writes if w.head == 'C' and w.ms >= since and (until is None or w.ms < until)]

    def mcu_lines(self, kinds=None, since=0):
        return [line for line in self.backend.lines()
                if line.queued_tick >= since and (kinds is None or line.kind in kinds)]

    def status_lines(self, since=0, until=None):
        return [(line.queued_tick, line.status()) for line in self.backend.lines()
                if line.kind == 'S' and line.queued_tick >= since and (until is None or line.queued_tick < until)]

    def changes(self, channel, since=0):
        return [(t, v) for t, v in self.backend.channel(channel) if t >= since]

    def first_change_to(self, channel, value, since):
        for t, v in self.backend.channel(channel):
            if t >= since and v == value:
                return t
        return None

    def latch_ms(self, name):
        entry = self.core.latches.get(name)
        return None if entry is None else int(round(entry['since'] * 1000))


class Operator:
    """A dashboard client: lease, one-use ticket, seq + 1, deadman (RoverControl rules)."""

    def __init__(self, rig, owner=OWNER):
        self.rig, self.owner = rig, owner
        self.reply = None

    def arm(self):
        reply = self.rig.submit(dict(ARM_REQUEST, owner=self.owner))
        if reply.get('ok'):
            self.reply = reply
        return reply

    def arm_confirmed(self, limit_ms=200):
        reply = self.arm()
        if not reply.get('ok'):
            raise AssertionError('arm refused: %r' % (reply,))
        if not self.rig.run_until(lambda: self.rig.core.steering_host()['arm_state'] == 'confirmed', limit_ms):
            raise AssertionError('arm not confirmed: %r' % (self.rig.core.steering_host(),))
        return reply

    def _owned(self, op, **extra):
        r = self.reply
        reply = self.rig.submit(dict(op=op, owner=self.owner, lease=r['lease'], ticket=r['ticket'], seq=r['seq'] + 1,
                                     **extra))
        if reply.get('ok'):
            self.reply = reply
        return reply

    def drive(self, throttle, steer_mrad):
        return self._owned('drive', deadman=True, throttle=throttle, steer_mrad=steer_mrad)

    def release(self):
        return self._owned('release')

    def hold(self, ms, throttle, steer_mrad, every_ms=100):
        """Keep the key pressed: a drive request every `every_ms` for `ms`."""
        end = self.rig.ms + ms
        while self.rig.ms < end:
            reply = self.drive(throttle, steer_mrad)
            if not reply.get('ok'):
                return reply
            self.rig.run(min(every_ms - BRIDGE_MS, end - self.rig.ms))
        return self.reply


class HostFirmwareCase(unittest.TestCase):
    def setUp(self):
        host_modules()
        native_path()

    def rig(self, **kwargs):
        return Rig(self, **kwargs)

    def driving(self, throttle=40, steer_mrad=150, ms=1500, **kwargs):
        rig = self.rig(**kwargs)
        rig.ready()
        op = Operator(rig)
        op.arm_confirmed()
        op.hold(ms, throttle, steer_mrad)
        return rig, op

    def assert_no_restart(self, rig, since, until=None, mcu_since=None):
        """No C or ARM from the host in [since, until) and MCU drive outputs 0 from mcu_since (default since) to
        the end: no automatic restart (A17). since is the host's own detection time (lines it wrote before it
        knew are not a restart)."""
        self.assertEqual([h for h in rig.host_heads(since, until) if h in ('C', 'ARM')], [])
        mcu_since = since if mcu_since is None else mcu_since
        for channel in ('drive_l', 'drive_r'):
            self.assertEqual(rig.backend.channel_at(channel, mcu_since), 0, channel)
            self.assertTrue(all(v == 0 for _, v in rig.changes(channel, mcu_since)), channel)
        self.assertIn(rig.backend.channel_at('mcu_state', rig.ms - 1), (1, 255))


class HandshakeTest(HostFirmwareCase):
    def test_banner_boot_id_and_session_stop(self):
        # Port-open reset with a 70 ms boot delay. Boot ROM bytes with an LF are dropped by the line splitter;
        # without an LF they swallow the banner and the host gets it with INFO after banner_timeout_s (spec 10.7).
        for garbage, limit_ms in (('00FF0A41420D0A', 400), ('00FF4142', 2200)):
            with self.subTest(garbage=garbage):
                rig = self.rig(port_open_reset=True, boot_delay_ticks=70, boot_garbage_hex=garbage)
                rig.ready(limit_ms)
                self.assertEqual(rig.link.state, 'ready')
                self.assertEqual(rig.link.boot_id, rig.backend.boot_id(0))
                banner = rig.link.banner
                self.assertEqual((banner['rst'], banner['sim'], banner['hw']), ('ext_sys', True, rig.hp.hw_profile))
                self.assertEqual((banner['wd'], banner['ttl'], banner['st']),
                                 (rig.hp.watchdog_ms, rig.hp.c_ttl_max_ms, rig.hp.status_hz))
                cal = rig.link.cal
                self.assertEqual(cal['cal_id'], rig.hp.cal_id)
                self.assertEqual((cal['soft_l'], cal['soft_r']), rig.hp.expected_soft)
                self.assertTrue(cal['pol_ok'] and not cal['caldirty'])
                heads = rig.host_heads()
                expected = ['INFO', 'STOP'] if garbage == '00FF4142' else ['STOP']
                self.assertEqual([h for h in heads if h != 'PING'][:len(expected)], expected)
                self.assertNotIn('CAL', heads)            # the cal line came with the banner, no CAL GET
                self.assertEqual([h for h in heads if h in ('ARM', 'C', 'BENCH')], [])
                t_ready = rig.ms
                rig.run(1000)
                pings = rig.host_heads(t_ready).count('PING')
                self.assertTrue(9 <= pings <= 11, pings)  # 10 Hz
                self.assertEqual(rig.mcu_lines(('N', 'F', 'W')), [])
                self.assertEqual(rig.backend.channel_at('mcu_state', rig.ms - 1), 1)
                self.assertEqual(rig.core.snapshot['state'], 'disarmed')

    def test_host_profile_and_eeprom_agree(self):
        # One calibration bundle (spec 12.1-64): the host profile export and the section 11.11 EEPROM image.
        rig = self.rig()
        data = hpe.build(rig.profile, cal_id=1)
        cal = data['calibration']
        self.assertEqual(cal['cal_id'], rig.cal['cal_id'])
        self.assertEqual((cal['expected_soft_permille']['left'], cal['expected_soft_permille']['right']),
                         (rig.cal['soft_l'], rig.cal['soft_r']))
        self.assertAlmostEqual(cal['permille_per_deg'], rig.cal['scale_permille_per_deg'], places=9)
        left, _, right = rig.cal['mark_angles_rad']
        self.assertAlmostEqual(cal['mark_left_mrad'], left * 1000.0, places=6)
        self.assertAlmostEqual(cal['mark_right_mrad'], -right * 1000.0, places=6)
        self.assertEqual(data['link']['watchdog_ms'], rig.cal['params']['wd'])
        self.assertEqual(data['link']['c_ttl_max_ms'], rig.cal['params']['ttlmax'])
        self.assertEqual(data['link']['status_hz'], rig.cal['params']['sthz'])

    def test_cal_id_mismatch_refuses_arm(self):
        rig = self.rig(eeprom_cal_id=2, host_cal_id=1)
        rig.run_until(lambda: rig.link.ready and rig.link.cal is not None, 1000)
        rig.run(200)
        self.assertIn('calibration_mismatch', rig.core.latches)
        self.assertEqual(rig.link.cal['cal_id'], 2)
        self.assertEqual(rig.submit(ARM_REQUEST), {'ok': False, 'error': 'calibration_mismatch'})
        rig.run(300)
        self.assertIsNone(rig.core.control.lease)
        self.assertNotIn('ARM', rig.host_heads())
        self.assertEqual(rig.core.snapshot['state'], 'fault')
        self.assertTrue(all(v == 1 for _, v in rig.changes('mcu_state')[1:]))


class DriveTest(HostFirmwareCase):
    def test_arm_drive_with_steering_c_resend_and_ttl_rule(self):
        rig = self.rig()
        rig.ready()
        op = Operator(rig)
        t_arm = rig.ms
        op.arm_confirmed()
        arm = [w for w in rig.writes if w.head == 'ARM']
        self.assertEqual(len(arm), 1)
        self.assertEqual(arm[0].data, b'ARM %s*%02X\n' % (rig.backend.boot_id(0).encode(),
                                                          rig.v3.crc8(b'ARM ' + rig.backend.boot_id(0).encode())))
        self.assertLessEqual(rig.ms - t_arm, rig.hp.arm_confirm_s * 1000)
        self.assertEqual(rig.backend.channel_at('mcu_state', arm[0].arrival), 2)
        segments = ((150, 2000), (-100, 2000))
        t_drive = rig.ms
        for steer, ms in segments:
            op.hold(ms, 40, steer)
        t_release = rig.ms
        op.release()
        rig.run(1600)
        cs = rig.c_writes(t_drive)
        self.assertGreater(len(cs), 70)
        # q + 1 for every C line (resends included), L = R = 102 (40 %), S from the calibration table.
        qs = [w.c_fields()[0] for w in cs]
        self.assertEqual(qs, list(range(qs[0], qs[0] + len(qs))))
        self.assertEqual(qs[0], 1)                              # first C of this boot
        for w in cs:
            q, ttl, left, right, steer = w.c_fields()
            input_until, throttle, mrad = self.input_deadline(rig, w.ms)
            remaining = int(math.floor((input_until - w.ms / 1000.0) * 1000.0))
            self.assertEqual(ttl, min(rig.hp.c_ttl_max_ms, rig.link.banner['ttl'], remaining), (w.ms, w.data))
            self.assertEqual((left, right), (self.duty(throttle),) * 2)
            self.assertEqual(steer, rig.hp.permille_from_mrad(mrad))
        # 20 Hz resend while the input is valid (one bridge step quantises 50 ms to 40 / 60 ms).
        steady = [w.ms for w in cs if t_drive + 500 <= w.ms < t_drive + 1500]
        self.assertTrue(19 <= len(steady) <= 21, len(steady))
        self.assertTrue(all(40 <= b - a <= 60 for a, b in zip(steady, steady[1:])), steady)
        # The MCU accepted every C: no N line, its status q follows the last C that arrived. The only F line is
        # the TTL expiry after the release deadline (1.1 s): the C stream ends there and the MCU brakes (already 0).
        self.assertEqual(rig.mcu_lines(('N', 'W', 'A', 'P'), t_arm), [])
        last_c = cs[-1]
        expiry = last_c.arrival + last_c.c_fields()[1]
        self.assertEqual([(line.queued_tick, line.text.split('*')[0]) for line in rig.mcu_lines(('F',), t_arm)],
                         [(expiry, 'F TTL 1 %d' % expiry)])
        self.assertGreaterEqual(expiry, t_release + 1100 - BRIDGE_MS)
        arrivals = [(w.arrival, w.c_fields()[0]) for w in cs]
        for tick, s in rig.status_lines(t_drive, t_release):
            last = [q for a, q in arrivals if a <= tick]
            if last:
                self.assertEqual(s['q'], last[-1], tick)
        # Drive duty and steering follow; the servo settles inside the re-push threshold 2 * db (spec 6.5, 7.6).
        db = rig.cal['params']['db']
        t = t_drive
        for steer, ms in segments:
            target = rig.hp.permille_from_mrad(steer)
            late = [s for tick, s in rig.status_lines(t + ms - 600, t + ms)]
            self.assertTrue(late)
            for s in late:
                self.assertEqual((s['st'], s['L'], s['R'], s['tgt']), (2, 102, 102, target))
                self.assertLessEqual(abs(s['pos'] - target), 2 * db, s)
            t += ms
        block = rig.core.steering_block()
        self.assertEqual(block['faults'], ['TTL'])               # the TTL condition bit after the release deadline
        self.assertEqual(rig.core.latches, {})
        # release: L = R = 0 with the same S, the MCU ramps down (release), the steering target stays.
        rel = rig.c_writes(t_release)
        self.assertTrue(rel)
        self.assertEqual({w.c_fields()[2:] for w in rel}, {(0, 0, rig.hp.permille_from_mrad(-100))})
        zero = rig.first_change_to('drive_l', 0, t_release)
        self.assertIsNotNone(zero)
        self.assertGreater(zero - t_release, 300)              # ramp 255/s from 102: about 400 ms, not a brake
        # No INFO or CAL GET while the lease was held (spec 12.1-60).
        self.assertEqual([h for h in rig.host_heads(t_arm) if h in ('INFO', 'CAL')], [])

    @staticmethod
    def duty(throttle):
        return (abs(throttle) * 255 + 50) // 100 * (1 if throttle >= 0 else -1)

    @staticmethod
    def input_deadline(rig, ms):
        """(input deadline in s, throttle, steer_mrad) of the last drive or release answered before `ms`."""
        found = None
        steer = None
        for start, op, reply in rig.dispatched:
            if start > ms or not reply.get('ok'):
                continue
            if op == 'drive':
                requested = reply['status']['requested']
                steer = requested['steer_mrad']
                found = (start / 1000.0 + rig.core.control.INPUT_TTL, requested['throttle'], steer)
            elif op == 'release':
                found = (start / 1000.0 + rig.core.control.RELEASE_TTL, 0, steer)
        return found


class StopTest(HostFirmwareCase):
    def test_stop_confirmed_hold_and_no_restart(self):
        rig, op = self.driving(40, 300, 2000)
        t_req = rig.ms - BRIDGE_MS + 1            # the request comes 1 ms after the previous bridge step
        rig.core.submit({'op': 'stop'})
        confirmed_ms = None
        while rig.ms < t_req + 2600:
            rig.tick()
            state = rig.core.steering_host()['stop_state']
            if confirmed_ms is None and state == 'confirmed' and rig.core.counters['stop_confirmed'] == 2:
                confirmed_ms = rig.ms - BRIDGE_MS  # the bridge step that read the confirming status line
        stop = [w for w in rig.writes if w.ms >= t_req and w.head == 'STOP']
        self.assertEqual(len(stop), 1)                                      # confirmed, no resend
        stop = stop[0]
        first_in_step = [w for w in rig.writes if w.ms == stop.ms][0]
        self.assertIs(first_in_step, stop)                                  # STOP is the first line of that step
        self.assertLessEqual(stop.ms - t_req, BRIDGE_MS)                    # A17: <= one bridge tick
        self.assertEqual(rig.first_change_to('drive_l', 0, stop.ms), stop.arrival)   # A17: 0 servo ticks
        self.assertEqual(rig.first_change_to('drive_r', 0, stop.ms), stop.arrival)
        after = rig.status_lines(stop.arrival)
        tick, s = after[0]
        self.assertLessEqual(tick - stop.arrival, STATUS_MS)
        self.assertEqual((s['st'], s['L'], s['R']), (1, 0, 0))
        self.assertEqual(rig.core.counters['stop_confirmed'], 2)            # session STOP and this one
        self.assertIsNotNone(confirmed_ms)
        self.assertLessEqual(confirmed_ms - stop.ms, rig.hp.stop_confirm_s * 1000)   # A17: <= 2 status periods
        # HOLD hold_measured for hstop, then OFF (spec 4.5, 6.5); hold error <= max(1 deg, 2 db) (A17).
        hstop = rig.cal['params']['hstop']
        self.assertEqual(rig.first_change_to('steer_en', 0, stop.arrival), stop.arrival + hstop)
        scale = rig.cal['scale_permille_per_deg']
        db_deg = rig.cal['params']['db'] / scale
        hold = [s for tick, s in rig.status_lines(stop.arrival, stop.arrival + hstop) if s['en'] & 2]
        self.assertGreater(len(hold), 30)
        err_deg = max(abs(s['tgt'] - s['pos']) for s in hold) / scale
        self.assertLessEqual(err_deg, max(1.0, 2 * db_deg) + 1e-9)
        d0 = [d for ms, d in rig.truth if ms <= stop.arrival + BRIDGE_MS][-1]
        drift = max(abs(d - d0) for ms, d in rig.truth if stop.arrival + BRIDGE_MS <= ms <= stop.arrival + hstop)
        self.assertLessEqual(math.degrees(drift), max(1.0, 2 * db_deg))
        # No automatic restart; the operator re-arm and a new drive are needed.
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'operator_stop')
        self.assert_no_restart(rig, stop.ms + 1)
        op2 = Operator(rig)
        t_rearm = rig.ms
        op2.arm_confirmed()
        rig.run(300)
        self.assertEqual(rig.c_writes(t_rearm), [])                         # armed, but no C before a drive
        op2.drive(30, 0)
        rig.run(100)
        self.assertTrue(rig.c_writes(t_rearm))

    def test_stop_unconfirmed_when_the_mcu_is_stuck(self):
        rig, op = self.driving(40, 100, 800)
        stall_at = rig.ms
        rig.backend.event('mcu_stall', tick=stall_at, arg=800)
        rig.run(BRIDGE_MS)
        rig.core.submit({'op': 'stop'})
        rig.run(700)
        stops = [w.ms for w in rig.writes if w.head == 'STOP' and w.ms >= stall_at]
        t0 = stops[0]
        # 5 resends on the fixed 50 ms grid from t0 + 100 ms (A17), each at the first 20 ms step at or after it.
        grid = [t0 + 100 + 50 * k for k in range(5)]
        self.assertEqual(stops[1:], [g + (-g) % BRIDGE_MS for g in grid])
        self.assertEqual(rig.latch_ms('stop_unconfirmed'), t0 + 400)
        self.assertEqual(rig.core.snapshot['state'], 'fault')
        self.assertIn('stop_unconfirmed', rig.core.snapshot['steering']['faults'])
        # The frozen MCU keeps its pins (a stuck MCU is the case the latch reports; physical stop is the E-stop).
        self.assertEqual(rig.backend.channel_at('drive_l', stall_at + 700), rig.backend.channel_at('drive_l',
                                                                                                    stall_at - 1))
        # After the stall the MCU runs again: its watchdog trips first (no input for > wd), then DISARMED lines.
        rig.run(600)
        w_lines = rig.mcu_lines(('W',), stall_at)
        self.assertEqual(len(w_lines), 1)
        self.assertEqual(w_lines[0].queued_tick, stall_at + 800)
        self.assertEqual(rig.core.steering_host()['stop_state'], 'confirmed')
        self.assertIn('stop_unconfirmed', rig.core.latches)                # kept until the operator re-arm
        self.assert_no_restart(rig, stall_at + 800)
        Operator(rig).arm_confirmed()
        self.assertEqual(rig.core.latches, {})

    def test_stop_unconfirmed_when_the_serial_line_drops(self):
        rig, op = self.driving(40, 100, 800)
        rig.drop_to_mcu = True
        cut = rig.ms
        last_c = rig.c_writes()[-1]
        last_input = max(w.arrival for w in rig.writes)
        rig.core.submit({'op': 'stop'})
        rig.run(900)
        lost = [ms for ms, data in rig.lost_writes if data == b'STOP\n']
        self.assertEqual(len(lost), 6)
        t0 = lost[0]
        self.assertEqual(rig.latch_ms('stop_unconfirmed'), t0 + 400)
        # The MCU on its own: TTL brake (ARMED), then the watchdog (DISARMED, W) at last input + wd + 1.
        q, ttl = last_c.c_fields()[:2]
        self.assertEqual(rig.first_change_to('drive_l', 0, cut), last_c.arrival + ttl)
        self.assertEqual(rig.backend.channel_at('mcu_state', last_c.arrival + ttl), 2)
        wd = rig.cal['params']['wd']
        self.assertEqual([line.queued_tick for line in rig.mcu_lines(('W',), cut)], [last_input + wd + 1])
        self.assertEqual(rig.first_change_to('mcu_state', 1, cut), last_input + wd + 1)
        self.assertEqual(rig.core.steering_host()['stop_state'], 'confirmed')   # by the watchdog DISARMED line
        self.assertIn('stop_unconfirmed', rig.core.latches)
        self.assert_no_restart(rig, last_input + wd + 1)


class LinkLossTest(HostFirmwareCase):
    def test_ttl_expiry_when_the_host_stops_sending(self):
        rig, op = self.driving(40, 100, 800)
        rig.host_frozen = True                 # the bridge process hangs: no C, no PING (shorter than wd)
        frozen = rig.ms
        rig.run(400)
        last_c = rig.c_writes()[-1]
        q, ttl = last_c.c_fields()[:2]
        expiry = last_c.arrival + ttl
        self.assertEqual(rig.first_change_to('drive_l', 0, frozen), expiry)          # A17: ttl, <= ttl + 1 tick
        self.assertEqual(rig.backend.channel_at('mcu_state', expiry), 2)              # ARMED, drive stopped
        f_lines = [(line.queued_tick, line.text.split('*')[0]) for line in rig.mcu_lines(('F', 'W'), frozen)]
        self.assertEqual(f_lines, [(expiry, 'F TTL 1 %d' % expiry)])
        fl = [s['fl'] for tick, s in rig.status_lines(expiry, frozen + 400)]
        self.assertTrue(fl and all(v & 0x0800 for v in fl))
        # The operator input is still valid (0.75 s deadline). On the MCU the TTL is a condition, not a latch (spec
        # 6.3: a new C would start the drive again after the 120 ms zero dwell). The host latch of DECISIONS R18
        # decision 1 closes that path: the first step after the hang reads the F TTL line while the input was still
        # valid, latches mcu_ttl_expired, drops the lease and sends STOP; no C with a new q, no restart until an
        # operator re-arm.
        rig.host_frozen = False
        resume = rig.ms
        rig.run(300)
        self.assertEqual(rig.c_writes(resume), [])
        self.assertEqual(rig.host_heads(resume)[0], 'STOP')
        self.assertEqual(list(rig.core.latches), ['mcu_ttl_expired'])
        self.assertIn('F TTL line', rig.core.latches['mcu_ttl_expired']['detail'])
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'mcu_ttl_expired')
        self.assert_no_restart(rig, resume, mcu_since=expiry)
        Operator(rig).arm_confirmed()
        self.assertEqual(rig.core.latches, {})

    def test_watchdog_on_link_loss_latches_the_host(self):
        # wd 200 ms with C ttl 300 ms: the watchdog, not the TTL, stops the drive (A17 watchdog item).
        rig, op = self.driving(40, 100, 800, record_overrides=dict(wd=200), host_overrides={'link': {'watchdog_ms': 200}})
        rig.drop_to_mcu = True                 # host -> MCU line lost, the host keeps running
        cut = rig.ms
        last_input = max(w.arrival for w in rig.writes)
        rig.run(600)
        trip = last_input + 200 + 1
        self.assertEqual([line.queued_tick for line in rig.mcu_lines(('W',), cut)], [trip])
        self.assertEqual(rig.first_change_to('drive_l', 0, cut), trip)        # A17: <= wd + 1 tick
        self.assertEqual(rig.first_change_to('mcu_state', 1, cut), trip)      # DISARMED
        self.assertTrue(rig.backend.channel_at('fault_bits', trip) & 0x1000)  # WATCHDOG latch
        self.assertEqual(rig.mcu_lines(('F',), cut), [])                        # before any TTL expiry
        self.assertIn('mcu_disarmed', rig.core.latches)
        self.assertEqual(rig.core.latches['mcu_disarmed']['detail'], 'MCU watchdog')
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'mcu_disarmed')
        self.assert_no_restart(rig, trip)
        rig.drop_to_mcu = False
        rig.run(200)
        Operator(rig).arm_confirmed()
        self.assertEqual(rig.core.latches, {})
        self.assertEqual(rig.backend.channel_at('fault_bits', rig.ms - 1) & 0x1000, 0)   # ARM cleared it

    def test_default_watchdog_after_the_ttl(self):
        rig, op = self.driving(40, 100, 800)
        rig.drop_to_mcu = True
        cut = rig.ms
        last_input = max(w.arrival for w in rig.writes)
        last_c = rig.c_writes()[-1]
        rig.run(800)
        ttl = last_c.c_fields()[1]
        wd = rig.cal['params']['wd']
        self.assertEqual(rig.first_change_to('drive_l', 0, cut), last_c.arrival + ttl)
        self.assertEqual([line.queued_tick for line in rig.mcu_lines(('W',), cut)], [last_input + wd + 1])
        self.assertEqual(rig.first_change_to('mcu_state', 1, cut), last_input + wd + 1)
        # host side (DECISIONS R18 decision 1): the F TTL line reaches the host (MCU -> host still works) while the
        # operator input is valid, so the host latches mcu_ttl_expired and drops the lease before the watchdog; the
        # later MCU DISARMED is then the expected end of its STOP, not mcu_disarmed
        self.assertEqual(list(rig.core.latches), ['mcu_ttl_expired'])
        self.assertLess(rig.core.latches['mcu_ttl_expired']['since'] * 1000., last_input + wd + 1)
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'mcu_ttl_expired')


class ResetAndFaultTest(HostFirmwareCase):
    def test_mcu_reset_changes_boot_id_and_latches(self):
        rig, op = self.driving(40, 100, 1000, boot_delay_ticks=70, boot_garbage_hex='00FF4142')
        t_reset = (rig.ms // STATUS_MS + 1) * STATUS_MS + 3      # 3 ms into a status line: the line is cut
        rig.backend.event('mcu_reset', tick=t_reset, arg=4)
        before_banner = None
        while rig.ms < t_reset + 800:
            new_boot = rig.mcu_lines(('S',), t_reset)
            if before_banner is None and new_boot and new_boot[0].done_tick + 1 <= rig.ms:
                # The next step reads the first status line of the new boot (ms back): an arm queued for that
                # step is refused, the boot id is not known yet (spec 12.1-61).
                n_arm = rig.host_heads().count('ARM')
                future = rig.core.submit(ARM_REQUEST)
                rig.tick()
                before_banner = future.result()
                self.assertEqual(rig.host_heads().count('ARM'), n_arm)
                continue
            rig.tick()
        # Pins float at the reset tick (coast), the new boot sets them up after the boot delay.
        self.assertEqual(rig.first_change_to('drive_l', 0, t_reset), t_reset)
        self.assertEqual(rig.first_change_to('drive_en', 0, t_reset), t_reset)
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'mcu_reset_detected')
        self.assertIn('mcu_reset_detected', rig.core.latches)
        self.assertIn('INFO', rig.host_heads(t_reset))                       # the banner was swallowed: ask
        self.assertEqual(rig.link.boot_id, rig.backend.boot_id(1))
        self.assertEqual(rig.link.banner['rst'], 'soft_restart')
        self.assertEqual(before_banner, {'ok': False, 'error': 'mcu_reset_detected'})
        detected = rig.latch_ms('mcu_reset_detected')
        self.assertLessEqual(detected - t_reset, 70 + STATUS_MS + 30)    # boot delay + first status line + step
        # C lines written before the host knew reach a booting or DISARMED MCU: lost or refused with N <q> state.
        refused = [line.text.split('*')[0].split(' ')[2] for line in rig.mcu_lines(('N',), t_reset)]
        self.assertTrue(set(refused) <= {'state'}, refused)
        self.assert_no_restart(rig, detected, mcu_since=t_reset)
        # Operator re-arm with the new boot id; q starts at 1 again.
        t_rearm = rig.ms
        op2 = Operator(rig)
        op2.arm_confirmed()
        self.assertEqual(rig.core.latches, {})
        arm = [w for w in rig.writes if w.head == 'ARM' and w.ms >= t_rearm]
        self.assertTrue(arm[0].data.startswith(b'ARM ' + rig.backend.boot_id(1).encode() + b'*'))
        op2.drive(30, 0)
        rig.run(60)
        self.assertEqual(rig.c_writes(t_rearm)[0].c_fields()[0], 1)

    def test_pot_fault_f_line_latches_the_host(self):
        rig, op = self.driving(40, 100, 600, injections=[{'kind': 'pot_open', 'at_s': 2.0, 'params': {}}])
        op.hold(2600 - rig.ms, 40, 100)          # returns early when the lease is gone
        rig.run(2600 - rig.ms)
        lines =[(line.queued_tick, line.text.split('*')[0].split(' ')[:2]) for line in rig.mcu_lines(('F',), 1900)]
        codes = [code for _, (_, code) in lines]
        self.assertIn('POT_OPEN', codes)
        first = lines[0][0]
        self.assertGreaterEqual(first, 2000)
        self.assertLessEqual(first, 2002)
        open_tick = [tick for tick, (_, code) in lines if code == 'POT_OPEN'][0]
        self.assertLessEqual(open_tick - 2000, 2)                            # med 5: f < pvmin 2 ticks later
        # Fault stop: steering output and drive 0 in the judging tick (A17: <= 1 tick), DISARMED.
        self.assertEqual(rig.first_change_to('steer_en', 0, 1990), first)
        self.assertEqual(rig.backend.channel_at('steer_pwm', first), 0)
        self.assertEqual(rig.first_change_to('drive_l', 0, 1990), first)
        self.assertEqual(rig.first_change_to('mcu_state', 1, 1990), first)
        self.assertIn('mcu_disarmed', rig.core.latches)
        self.assertTrue(rig.core.latches['mcu_disarmed']['detail'].startswith('MCU fault '))
        self.assertIn('POT_OPEN', rig.core.steering_block()['faults'])
        self.assertIsNone(rig.core.control.lease)
        detected = rig.latch_ms('mcu_disarmed')
        self.assertLessEqual(detected - first, STATUS_MS + 2 * BRIDGE_MS)   # next status line, read by a step
        t = rig.ms
        self.assert_no_restart(rig, detected, until=t, mcu_since=first)
        # Re-arm while the pot is still open: the MCU refuses (active condition), no lease remains.
        reply = Operator(rig).arm()
        rig.run(200)
        self.assertTrue(reply['ok'])                 # the host cannot see the MCU condition; the MCU decides
        self.assertEqual([line.text.split('*')[0] for line in rig.mcu_lines(('N',), t)], ['N - fault'])
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'steering_fault_latched')
        # The refused re-arm keeps the host latches and adds its own (spec 12.1-75, 2026-11-09 review): the state
        # stays fault while the MCU fault is there.
        self.assertEqual(set(rig.core.latches), {'mcu_disarmed', 'arm_rejected'})
        self.assertEqual(rig.core.snapshot['state'], 'fault')
        self.assertIn('POT_OPEN', rig.core.steering_block()['faults'])
        self.assert_no_restart(rig, t + BRIDGE_MS, mcu_since=first)

    def test_arm_refused_with_n_boot_gets_the_new_banner(self):
        # Spec 12.1-76 (2026-11-09 review): boot ROM bytes without LF hide every boot banner. The MCU resets after
        # the INFO banner and before any status line, so nothing tells the host: ARM with the old boot id gets
        # N - boot. That refusal latches mcu_reset_detected and asks INFO; the re-arm then uses the new boot id.
        # Before, every arm got N - boot again until the link was reopened.
        rig = self.rig(boot_garbage_hex='00FF4142')
        self.assertTrue(rig.run_until(lambda: rig.link.ready, 3000))
        self.assertIsNone(rig.link.status)
        rig.backend.event('mcu_reset', tick=rig.ms, arg=4)
        rig.run(400)
        self.assertEqual(rig.link.boot_id, rig.backend.boot_id(0))          # stale: nothing showed the reset
        self.assertEqual(rig.core.steering_host()['stop_state'], 'confirmed')
        t_arm = rig.ms
        self.assertTrue(rig.submit(ARM_REQUEST)['ok'])
        rig.run(200)
        self.assertEqual([line.text.split('*')[0] for line in rig.mcu_lines(('N',), t_arm)], ['N - boot'])
        self.assertTrue({'mcu_reset_detected', 'arm_rejected'} <= set(rig.core.latches))
        self.assertIsNone(rig.core.control.lease)
        self.assertIn('INFO', rig.host_heads(t_arm))
        self.assertTrue(rig.run_until(lambda: rig.link.boot_id == rig.backend.boot_id(1), 2000))
        rig.ready()
        Operator(rig).arm_confirmed()
        self.assertEqual(rig.core.latches, {})
        arm = [w for w in rig.writes if w.head == 'ARM' and w.ms >= t_arm]
        self.assertEqual(len(arm), 2)
        self.assertTrue(arm[1].data.startswith(b'ARM ' + rig.backend.boot_id(1).encode() + b'*'))
        self.assertEqual(rig.backend.channel_at('mcu_state', rig.ms - 1), 2)


if __name__ == '__main__':
    unittest.main()
