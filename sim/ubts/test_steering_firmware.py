"""Steering firmware V3 in the SIM: lockstep v2 firmware mode (design 8.3 P3, decision R1).

The unchanged firmware sources (firmware/ubts_steer_v3) run with the actuator and potentiometer model in the
native lockstep process; FirmwareSteeringBackend drives it. Expected values are written from the specification
(docs/steering/ESP_PROTOCOL_V3_DRAFT_KO.md, LOCKSTEP_PROTOCOL_KO.md section 11) by hand, not taken from another
implementation. unittest style: runs with pytest (Python 3.12, 3.13) and with `python3 -m unittest` (3.10).
The native binary is built into UBTS_STEER_BUILD_DIR or a temporary folder outside the work tree. Without the
firmware sources (a copy of sim/ubts only) the firmware tests skip with the reason; AbsentFirmwareTest then checks
that the program still builds without firmware mode.
All values are placeholders: these are placeholder-based SIM checks, not field validation.
"""
import atexit
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest

from steering import backend_api as ba
from steering import vehicle_profile as vp
from steering.actuator_link import (FirmwareSteeringBackend, NativeBackendError, NativeSteeringBackend, host_line,
                                    assemble_lines)
from steering.native import build as native_build
from steering.native import fw_eeprom
from steering.native import lockstep_v2 as v2

CLEAN = {
    'steer_zero_offset_deg': {'dist': 'uniform', 'low': 0.0, 'high': 0.0},
    'pot_cal_residual_deg': {'dist': 'uniform', 'low': 0.0, 'high': 0.0},
    'pot_noise_sd_counts': 0.0,
    'pot_supply_sag_per_pwm': 0.0,
    'steer_backlash_wander': {'sd_deg': 0.0, 'corr_length_m': 5.0},
    'steer_release_restoring': {'scale': 1.0, 'centering_per_s_per_m_s': 1.0, 'noise_sd_deg': 0.0,
                                'noise_corr_length_m': 1.0},
}
RECORD7 = fw_eeprom.encode_record(7, 842, 512, 181, 935, -935, pol_ok=1, mdir=1)   # spec 9.6
IMAGE7 = fw_eeprom.image_hex(RECORD7)
BOOT_IDS_SEED1 = ('89025CC1', '658EEC67', 'FB32555E')                             # spec 10.1
CAL7_LINE = ('UBTS_STEER_V3 cal id=7 crc=61AEB1A4 left=842 center=512 right=181 soft=935,-935 pol=ok mdir=1 '
             'caldirty=0 cfgdirty=0*D6')                                         # spec 5.9
BLANK_CAL_LINE = ('UBTS_STEER_V3 cal id=none crc=00000000 left=- center=- right=- soft=0,0 pol=no mdir=1 caldirty=0 '
                  'cfgdirty=0*3F')                                               # spec 5.9

_STATE = {}


def _cleanup_temp(path):
    shutil.rmtree(path, ignore_errors=True)


def compiler_or_skip():
    try:
        native_build.find_compiler()
    except native_build.NativeBuildError as exc:
        raise unittest.SkipTest('no C++ compiler for the native steering program: %s' % exc)


def firmware_dir_or_skip():
    """The firmware source folder; SkipTest when it is missing (then the program has no firmware mode), or a
    failure when UBTS_STEER_REQUIRE_FIRMWARE is set (a checkout that tracks the firmware must not skip)."""
    try:
        present = native_build.firmware_present()
    except native_build.NativeBuildError as exc:
        if native_build.firmware_required():
            raise AssertionError('%s is set: %s' % (native_build.ENV_REQUIRE_FIRMWARE, exc))
        raise unittest.SkipTest(str(exc))
    if not present:
        if native_build.firmware_required():
            raise AssertionError('%s is set: %s' % (native_build.ENV_REQUIRE_FIRMWARE,
                                                    native_build.missing_firmware_message()))
        raise unittest.SkipTest(native_build.missing_firmware_message())
    return native_build.firmware_dir()


def manifest_of(path):
    return json.loads(Path(path[:-4] + '.json' if path.endswith('.exe') else path + '.json').read_bytes())


def native():
    """The native program (built once per test process); SkipTest without a C++ compiler or firmware sources."""
    if 'build' not in _STATE:
        firmware_dir_or_skip()
        compiler_or_skip()
        build_dir = os.environ.get(native_build.ENV_BUILD_DIR)
        if not build_dir:
            build_dir = tempfile.mkdtemp(prefix='steer_fw_build_')
            atexit.register(_cleanup_temp, build_dir)
        _STATE['build_dir'] = build_dir
        _STATE['build'] = native_build.ensure_built(build_dir=build_dir)
    return _STATE['build']


def profile(clean=False):
    key = 'profile_clean' if clean else 'profile'
    if key not in _STATE:
        base = vp.load_profile()
        _STATE[key] = base.with_overrides(dict(CLEAN), label='fw_test', allow_out_of_range=True) if clean else base
    return _STATE[key]


def crcline(body):
    return host_line(body)


def body_crc(body):
    return '%s*%02X' % (body, fw_eeprom.crc8(body.encode('ascii')))


def script_firmware(raw=512, image=IMAGE7, **extra):
    fw = dict(boot_seed=1, chip_id=0x5A1B2C, eeprom_hex=image, adc_mode='script', adc_script_raw=raw)
    fw.update(extra)
    return fw


class FirmwareCase(unittest.TestCase):
    """Helpers: start a backend, read lines and status fields at ticks."""

    def make(self, firmware=None, injections=(), seed=0, clean=False, startup_overrides=None, prof=None):
        backend = FirmwareSteeringBackend(prof or profile(clean), seed, firmware=firmware, injections=injections,
                                          startup_overrides=startup_overrides, executable=native().path)
        self.addCleanup(backend.close)
        return backend

    def calibrated(self, seed=0, clean=True, injections=(), **firmware):
        """A backend with the section 11.11 EEPROM image of a calibrated car and the potentiometer model."""
        prof = profile(clean)
        startup = v2.firmware_startup_config(prof, seed, injections=injections)
        image, details = fw_eeprom.calibrated_eeprom(startup, prof)
        fw = dict(boot_seed=1, chip_id=0x5A1B2C, eeprom_hex=image, adc_mode='model')
        fw.update(firmware)
        return self.make(fw, injections=injections, seed=seed, clean=clean), details

    @staticmethod
    def events(backend, since=0):
        """Lines other than status lines queued at or after tick `since`, as (queued tick, text)."""
        return [(line.queued_tick, line.text) for line in backend.lines()
                if line.kind != 'S' and line.queued_tick >= since]

    @staticmethod
    def faults(backend, since=0):
        return [(tick, text) for tick, text in FirmwareCase.events(backend, since) if text.startswith('F ')]

    def status(self, backend, tick):
        found = [line for line in backend.lines() if line.kind == 'S' and line.queued_tick == tick]
        self.assertEqual(len(found), 1, 'no status line at tick %d' % tick)
        return found[0].status()

    def assertAllCrc(self, backend):
        for line in backend.lines():
            self.assertTrue(line.crc_ok, line.text)


def keep_driving(backend, start, stop, q=1, left=100, right=100, steer=0, every=100, ttl=300):
    """C every `every` ticks with a new q (20 Hz style resend keeps TTL and watchdog alive); returns the next q."""
    for tick in range(start, stop, every):
        backend.send(crcline('C %d %d %d %d %d' % (q, ttl, left, right, steer)), tick=tick)
        q += 1
    return q


# --- build, unit test, identity ---------------------------------------------------------------------------------------
class BuildTest(unittest.TestCase):
    def test_native_unit_test_program(self):
        """Parser, CRC, formatter, EEPROM record, raw -> permille, drive ramp and state machine checks in C++."""
        native()
        unit = native_build.ensure_unit_test_built(build_dir=_STATE['build_dir'])
        proc = subprocess.run([unit.path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
        text = proc.stdout.decode('ascii', 'replace')
        self.assertEqual(proc.returncode, 0, text)
        self.assertTrue(text.strip().splitlines()[-1].startswith('PASS'), text)

    def test_firmware_build_id_rule(self):
        """Spec 10.5 written again here: SHA-256 over the sorted firmware sources -> first 8 hex, uppercase."""
        fw_dir = firmware_dir_or_skip()
        names = sorted((p.relative_to(fw_dir).as_posix() for p in fw_dir.rglob('*')
                        if p.is_file() and p.suffix in ('.ino', '.h', '.cpp', '.c')), key=lambda n: n.encode())
        digest = hashlib.sha256()
        for name in names:
            data = (fw_dir / name).read_bytes()
            digest.update(name.encode() + b'\0' + struct.pack('<Q', len(data)) + data)
        expected = digest.hexdigest()[:8].upper()
        self.assertEqual(native_build.firmware_build_id(), expected)
        self.assertIn('ubts_steer_v3.ino', names)
        manifest = manifest_of(native().path)
        self.assertEqual(manifest['firmware_build_id'], expected)
        self.assertEqual(native().firmware_build_id, expected)

    def test_firmware_sources_are_plain_ascii_lf(self):
        fw_dir = firmware_dir_or_skip()
        for path in fw_dir.iterdir():
            data = path.read_bytes()
            self.assertNotIn(b'\r\n', data, path.name)
            self.assertFalse(data.startswith(b'\xef\xbb\xbf'), path.name)
            data.decode('ascii')
            self.assertNotIn(b'#if UBTS_STEER_SIM', data, path.name)      # no SIM branch in the firmware logic
            self.assertNotIn(b'#ifdef UBTS_STEER_SIM', data, path.name)

    def test_eeprom_record_and_defaults(self):
        self.assertEqual(RECORD7.hex().upper(), (
            '55534333016A07004A030002B500A70359FC0101F50014006600FF00C8001400C003E0012C01F40100000000D007D007D0070000'
            '140005001400EB033C00C3030800020032003200280064009900F40114007800E80350002C01CC001E00881305002C01F401'
            'A4B1AE61'))
        tomb = fw_eeprom.encode_record(7, 842, 512, 181, 935, -935, magic='USCX')
        self.assertEqual(struct.unpack_from('<I', tomb, 102)[0], 0x4F1B5580)
        self.assertEqual(fw_eeprom.params_from_profile(profile()), fw_eeprom.cfg_defaults())   # spec 4.10, s = 40
        self.assertEqual(fw_eeprom.raw_to_permille(842, 512, 181, 611), 300)
        self.assertEqual(fw_eeprom.raw_to_permille(842, 512, 181, 400), -338)
        startup = v2.firmware_startup_config(profile(), 1)
        image, details = fw_eeprom.calibrated_eeprom(startup, profile())
        self.assertAlmostEqual(details['scale_permille_per_deg'], 39.8, delta=0.1)              # section 11.11
        decoded = fw_eeprom.decode_record(bytes.fromhex(image)[:fw_eeprom.RECORD_BYTES])
        self.assertIsNotNone(decoded)
        self.assertEqual(bytes.fromhex(image)[fw_eeprom.RECORD_BYTES:], b'\xff' * 150)

    def test_boot_ids_follow_splitmix64(self):
        self.assertEqual(['%08X' % x for x in v2.sim_boot_ids(0, 3)], ['7B1DCDAF', 'A1B965F4', '8009454F'])
        self.assertEqual(['%08X' % x for x in v2.sim_boot_ids(1, 3)], list(BOOT_IDS_SEED1))
        self.assertEqual(['%08X' % x for x in v2.sim_boot_ids(42, 3)], ['2FEB6E95', 'B266F103', '130F9F52'])


# --- boot, banner, INFO, PING -----------------------------------------------------------------------------------------
class BootTest(FirmwareCase):
    def test_boot_banner_cal_line_and_first_status(self):
        b = self.make(script_firmware())
        self.assertTrue(b.build_id.startswith('ubts-steer-native/2 src=h'))
        self.assertEqual(b.firmware_build, native_build.firmware_build_id())
        b.run_until(110)
        lines = b.lines()
        self.assertEqual(lines[0].text, body_crc(
            'UBTS_STEER_V3 hello proto=3 build=%s dev=5A1B2C boot=89025CC1 rst=power_on hw=front_steer_v0 '
            'out=drive2,steer1 cal=7 sim=1 wd=500 ttl=300 st=20 drop=0' % b.firmware_build))
        self.assertEqual(lines[1].text, CAL7_LINE)
        self.assertEqual((lines[0].queued_tick, lines[1].queued_tick), (0, 0))
        self.assertEqual(lines[2].text, body_crc('S 50 0 1 0 0 0 1 512 0 0 0 0 4000 50 0'))
        self.assertEqual(lines[3].text, body_crc('S 100 1 1 0 0 0 1 512 0 0 0 0 4000 100 0'))
        self.assertEqual((lines[2].queued_tick, lines[3].queued_tick), (50, 100))
        self.assertEqual(lines[2].done_tick, v2.mcu_done_tick(50 * 576 + 50 * (len(lines[2].text) + 1)))
        self.assertAllCrc(b)
        # pins and state at the boot tick (spec 8.3 setup order: 14 GPIO writes)
        self.assertEqual(b.channel('drive_en'), [(0, 1)])
        self.assertEqual(b.channel('mcu_state'), [(0, 1)])
        self.assertEqual(b.channel('gpio_writes'), [(0, 14)])
        self.assertEqual(b.channel('fault_bits'), [(0, 0x4000)])
        self.assertEqual(b.channel('steer_en'), [])
        self.assertEqual(b.last.boot_count, 1)
        self.assertEqual(b.last.mcu_millis, 109)

    def test_blank_eeprom_and_boot_delay(self):
        b = self.make(script_firmware(image=None, boot_delay_ticks=70))
        b.send(b'PING\n', tick=10)        # before setup(): lost
        b.run_until(180)                  # the status line queued at 170 is sent by tick 174
        lines = b.lines()
        self.assertEqual(lines[0].queued_tick, 70)
        self.assertIn(' cal=none ', lines[0].text)
        self.assertEqual(lines[1].text, BLANK_CAL_LINE)
        self.assertEqual(lines[2].text, body_crc('S 120 0 1 0 0 0 1 512 0 0 0 0 4001 50 0'))
        self.assertEqual(lines[3].text, body_crc('S 170 1 1 0 0 0 1 512 0 0 0 0 4001 100 0'))
        self.assertEqual(b.last.rx_lost_total, 5)
        self.assertEqual(b.channel('mcu_state'), [(70, 1)])
        self.assertEqual(b.channel_at('mcu_state', 69), 255)

    def test_boot_garbage_bytes_come_first_and_are_flagged(self):
        b = self.make(script_firmware(boot_garbage_hex='00FF55'))
        b.run_until(20)
        self.assertEqual([(e.byte, e.boot_garbage, e.queued_tick) for e in b.tx_bytes[:3]],
                         [(0x00, True, 0), (0xFF, True, 0), (0x55, True, 0)])
        self.assertFalse(b.tx_bytes[3].boot_garbage)
        self.assertEqual(b.tx_bytes[0].done_tick, 0)     # 50 u -> tick 0
        with_garbage = b.lines(include_garbage=True)
        self.assertEqual(with_garbage[0].text, '\x00�U' + b.lines()[0].text)

    def test_info_and_ping(self):
        b = self.make(script_firmware())
        b.send(b'PING\n', tick=120)
        b.send(b'INFO\n', tick=160)
        b.run_until(210)
        self.assertEqual(self.status(b, 150)['wd'], 30)     # PING refreshed the link at 120
        self.assertEqual(self.status(b, 200)['wd'], 80)     # INFO did not
        info = [line for line in b.lines() if line.queued_tick == 160]
        self.assertEqual(len(info), 2)
        self.assertTrue(info[0].text.startswith('UBTS_STEER_V3 hello proto=3 '))
        self.assertEqual(info[1].text, CAL7_LINE)
        self.assertEqual(b.channel('mcu_state'), [(0, 1)])


# --- parser and rejections ---------------------------------------------------------------------------------------------
class ParserTest(FirmwareCase):
    def test_reject_reasons_and_order(self):
        b = self.make(script_firmware(raw=512))
        sends = [
            (100, b'ping\n'), (101, b'D8 0\n'), (102, b'C 1 300 0 0 0\n'), (103, crcline('ARM 00000001')),
            (104, b'INFO junk\n'), (105, b'PING*00\n'), (106, b'X' * 80 + b'\n'), (107, b'PING\x00junk\n'),
            (108, b'D8 2\r55\n'), (109, crcline('CFG zz 1')), (110, crcline('CFG kp 9999')),
            (111, crcline('CFG rstd 1000')), (112, crcline('CAL JOG 0 100')), (113, crcline('CAL MARK X')),
            (114, b'\n\r\n'), (115, crcline('ARM 89025CC1')),
        ]
        for tick, data in sends:
            b.send(data, tick=tick)
        b.run_until(130)
        self.assertEqual(self.events(b, since=100), [
            (100, body_crc('N - cmd')), (101, body_crc('N - state')), (102, body_crc('N 1 nocrc')),
            (103, body_crc('N - boot')), (104, body_crc('N - syntax')), (105, body_crc('N - crc')),
            (107, body_crc('N - long')),       # 81 bytes: at most 64 are read per tick, the LF in tick 107
            (107, body_crc('N - ctrl')), (108, body_crc('N - ctrl')), (109, body_crc('N - key')),
            (110, body_crc('N - range')), (111, body_crc('N - range')), (112, body_crc('N - range')),
            (113, body_crc('N - syntax'))])
        self.assertEqual(b.channel_at('mcu_state', 129), 2)          # empty lines ignored, ARM accepted

    def test_v2_strict_parser_cases_in_bench(self):
        """V2 native test 10 mapped to V3 (spec 14 row 10): syntax / range / cmd, outputs and watchdog unchanged."""
        cases = [(b'D8 -1', 'syntax'), (b'D8 256', 'range'), (b'D8 999999999', 'range'),
                 (b'D8 999999999999999999999', 'syntax'), (b'D8 1.5', 'syntax'), (b'D8 +1', 'syntax'),
                 (b'D8  1', 'syntax'), (b'D8 1 ', 'syntax'), (b'D8 1junk', 'syntax'), (b'D9 255', 'cmd'),
                 (b'D10 255', 'cmd'), (b'GPIO15 255', 'cmd'), (b'd8 255', 'cmd'), (b'D8', 'syntax'),
                 (b'D8 ', 'syntax'), (b' PING', 'cmd'), (b'PING junk', 'syntax')]
        runs = []
        for with_cases in (True, False):
            b = self.make(script_firmware())
            b.send(crcline('BENCH 89025CC1'), tick=150)
            b.send(b'D8 128\n', tick=150)
            if with_cases:
                for i, (text, _) in enumerate(cases):
                    b.send(text + b'\n', tick=400 + i)
            b.run_until(460)
            runs.append(b)
        got = [text for tick, text in self.events(runs[0], since=400)]
        self.assertEqual(got, [body_crc('N - %s' % reason) for _, reason in cases])
        self.assertEqual(self.status(runs[0], 450)['wd'], 300)          # last refresh: D8 128 at 150
        self.assertEqual(runs[0].channel('drive_l'), runs[1].channel('drive_l'))
        self.assertEqual(runs[0].channel_at('mcu_state', 459), 3)

    def test_stop_special_and_bad_line_burst(self):
        b = self.make(script_firmware())
        b.send(crcline('ARM 89025CC1'), tick=10)
        b.send(crcline('C 1 300 60 60 0'), tick=20)
        for i in range(4):
            b.send(b'PING*00\n', tick=200 + i)
        b.send(b'PING\n', tick=204)                     # a good line resets the run
        for i in range(4):
            b.send(b'PING*00\n', tick=205 + i)
        b.send(b'STOP*ZZ\n', tick=209)                  # special STOP: never a bad line
        b.run_until(215)
        self.assertEqual(b.channel_at('mcu_state', 208), 2)
        self.assertEqual(b.channel_at('mcu_state', 209), 1)
        self.assertEqual(b.channel_at('drive_l', 209), 0)
        self.assertEqual([t for t, text in self.events(b, since=200) if not text.startswith('N - crc')], [])
        b2 = self.make(script_firmware())
        b2.send(crcline('ARM 89025CC1'), tick=10)
        b2.send(crcline('C 1 300 60 60 0'), tick=20)
        for i in range(5):
            b2.send(b'C 2 300 60 60 0*00\n', tick=200 + i)
        b2.run_until(2300)
        ev = self.events(b2, since=200)
        self.assertEqual(ev, [(200 + i, body_crc('N - crc')) for i in range(5)] +
                         [(204, body_crc('F CRC 5 204'))])
        self.assertEqual(b2.channel_at('mcu_state', 204), 1)
        self.assertEqual(b2.channel_at('drive_l', 203) > 0, True)
        self.assertEqual(b2.channel_at('drive_l', 204), 0)                       # brake
        self.assertEqual(b2.channel_at('fault_bits', 204) & 0x2000, 0x2000)
        self.assertEqual(b2.channel_at('steer_en', 2203), 1)                     # HOLD measured for hloss 2000
        self.assertEqual(b2.channel_at('steer_en', 2204), 0)


# --- STOP, DISARM -------------------------------------------------------------------------------------------------------
class StopTest(FirmwareCase):
    def test_stop_in_armed_brakes_and_holds_steering_then_off(self):
        b = self.make(script_firmware())
        b.send(crcline('ARM 89025CC1'), tick=10)
        keep_driving(b, 20, 600)
        b.send(b'STOP\n', tick=600)
        b.run_until(2700)
        self.assertEqual(b.channel_at('drive_l', 599), 100)
        self.assertEqual(b.channel_at('drive_l', 600), 0)              # same tick (0 ticks)
        self.assertEqual(b.channel_at('drive_r', 600), 0)
        self.assertEqual(b.channel_at('mcu_state', 600), 1)
        self.assertEqual(b.channel_at('steer_en', 2599), 1)            # HOLD (measured) for hstop 2000
        self.assertEqual(b.channel_at('steer_en', 2600), 0)            # then OFF
        st = self.status(b, 650)
        self.assertEqual((st['st'], st['L'], st['R'], st['fl'], st['tl']), (1, 0, 0, 0, 0))
        self.assertEqual(self.events(b, since=11), [])                 # no reply lines

    def test_stop_in_bench_and_disarmed(self):
        b = self.make(script_firmware())
        b.send(crcline('BENCH 89025CC1'), tick=200)
        b.send(b'D8 200\n', tick=200)
        b.send(b'STOP\n', tick=500)
        b.run_until(600)
        self.assertGreater(b.channel_at('drive_l', 499), 0)
        self.assertEqual(b.channel_at('drive_l', 500), 0)
        self.assertEqual(b.channel_at('mcu_state', 500), 1)
        # DISARMED: DISARM from ARMED releases (ramp down), a later STOP cuts the ramp at once
        b2 = self.make(script_firmware())
        b2.send(crcline('ARM 89025CC1'), tick=10)
        keep_driving(b2, 20, 600)
        b2.send(crcline('DISARM'), tick=600)
        b2.send(b'STOP\n', tick=650)
        b2.run_until(700)
        self.assertEqual(b2.channel_at('mcu_state', 600), 1)
        self.assertTrue(0 < b2.channel_at('drive_l', 649) < 100)       # still ramping down (release)
        self.assertEqual(b2.channel_at('drive_l', 650), 0)            # STOP in DISARMED brakes
        self.assertEqual(b2.channel_at('steer_en', 699), 1)           # the DISARM HOLD continues


# --- ARM ------------------------------------------------------------------------------------------------------------------
class ArmTest(FirmwareCase):
    def test_arm_rejections_in_spec_order(self):
        blank = self.make(script_firmware(image=None))
        blank.send(crcline('ARM 89025CC1'), tick=10)
        blank.run_until(60)       # behind the boot lines on the 115200 wire
        self.assertEqual(self.events(blank, since=10), [(10, body_crc('N - nocal'))])
        nopol = self.make(script_firmware(image=fw_eeprom.image_hex(
            fw_eeprom.encode_record(7, 842, 512, 181, 935, -935, pol_ok=0))))
        nopol.send(crcline('ARM 89025CC1'), tick=10)
        nopol.run_until(60)
        self.assertEqual(self.events(nopol, since=10), [(10, body_crc('N - pol'))])
        dirty = self.make(script_firmware(raw=850))
        dirty.send(crcline('CAL MARK L'), tick=10)
        dirty.send(crcline('ARM 89025CC1'), tick=11)
        dirty.run_until(80)
        ev = self.events(dirty, since=10)
        self.assertIn(' left=850 ', ev[0][1])
        self.assertIn(' caldirty=1 ', ev[0][1])
        self.assertEqual(ev[1], (11, body_crc('N - cal')))
        open_pot = self.make(script_firmware(raw=10))      # f < pvmin 20: POT_OPEN and the ARM 'fault' condition
        open_pot.send(crcline('ARM 89025CC1'), tick=100)
        open_pot.run_until(140)
        ev = self.events(open_pot, since=0)
        self.assertEqual(ev[2], (0, body_crc('F POT_OPEN 10 0')))
        self.assertEqual(ev[3], (100, body_crc('N - fault')))

    def test_arm_right_and_wrong_boot_id(self):
        b = self.make(script_firmware())
        b.send(crcline('ARM 00000000'), tick=10)
        b.send(crcline('ARM 89025cc1'), tick=20)          # lower-case hex accepted (spec 2.3)
        b.run_until(110)
        self.assertEqual(self.events(b, since=10), [(10, body_crc('N - boot'))])
        self.assertEqual(b.channel('mcu_state'), [(0, 1), (20, 2)])
        self.assertEqual(b.channel('steer_en'), [(20, 1)])  # tick 20 is a servo tick (500 Hz from boot 0)
        st = self.status(b, 50)
        self.assertEqual((st['st'], st['en'], st['fl'], st['wd']), (2, 3, 0, 30))


# --- C: drive ramp, dwell, steering ---------------------------------------------------------------------------------------
class CommandTest(FirmwareCase):
    def test_ttl_expiry_example_of_spec_13_11(self):
        b = self.make(script_firmware())
        b.send(crcline('ARM 89025CC1'), tick=10)
        b.send(crcline('C 1 300 100 100 0'), tick=20)
        b.run_until(401)
        self.assertEqual(b.channel('steer_en'), [(10, 1)])
        self.assertEqual(b.channel_at('mcu_state', 10), 2)
        self.assertEqual(self.status(b, 50), dict(ms=50, n=0, st=2, q=1, L=0, R=0, en=3, raw=512, pos=0, tgt=0,
                                                  cmd=0, sp=0, fl=0, wd=30, tl=270))
        self.assertEqual(b.channel('drive_l')[0], (120, 2))
        st = self.status(b, 150)
        self.assertEqual((st['L'], st['R'], st['wd'], st['tl']), (10, 10, 130, 170))
        self.assertEqual([line.text for line in b.lines() if line.queued_tick == 300 and line.kind == 'S'],
                         ['S 300 5 2 1 48 48 3 512 0 0 0 0 0000 280 20*AA'])
        self.assertEqual(b.channel_at('drive_l', 319), 51)
        self.assertEqual((b.channel_at('drive_l', 320), b.channel_at('drive_r', 320)), (0, 0))
        self.assertEqual(self.events(b, since=11), [(320, 'F TTL 1 320*6B')])
        self.assertEqual(b.channel_at('fault_bits', 320), 0x0800)
        st = self.status(b, 350)
        self.assertEqual((st['st'], st['L'], st['R'], st['fl'], st['wd'], st['tl']), (2, 0, 0, 0x0800, 330, 0))
        self.assertAllCrc(b)

    def test_ramp_values_and_reversal_dwell(self):
        b = self.make(script_firmware())
        b.send(crcline('ARM 89025CC1'), tick=10)
        q = keep_driving(b, 20, 600)
        keep_driving(b, 620, 2000, q=q, left=-50, right=-50)
        b.run_until(2000)
        # d after k updates of the 10 ms grid (first at tick 120) is floor(2.55 k), up to the target
        expect = {119: 0, 120: 2, 130: 5, 140: 7, 150: 10, 300: 48, 310: 51, 509: 99, 510: 100, 619: 100}
        self.assertEqual({t: b.channel_at('drive_l', t) for t in expect}, expect)
        values = [b.channel_at('drive_l', t) for t in range(620, 2000)]
        zero_at = 620 + values.index(0)
        first_neg = 620 + next(i for i, v in enumerate(values) if v < 0)
        self.assertTrue(all(v >= 0 for v in values[:zero_at - 620]))
        self.assertGreaterEqual(first_neg - zero_at, 120)                     # 120 ms zero dwell
        self.assertIn(b.channel_at('drive_l', first_neg), (-2, -3))
        self.assertEqual(b.channel_at('drive_l', 1999), -50)
        self.assertEqual(self.events(b, since=11), [])

    def test_ping_does_not_extend_ttl(self):
        b = self.make(script_firmware())
        b.send(crcline('ARM 89025CC1'), tick=10)
        b.send(crcline('C 1 300 100 100 0'), tick=20)
        for tick in range(70, 700, 50):
            b.send(b'PING\n', tick=tick)
        b.send(crcline('C 2 300 100 100 0'), tick=500)
        b.run_until(700)
        self.assertEqual(self.events(b, since=11), [(320, 'F TTL 1 320*6B')])
        self.assertEqual(b.channel_at('mcu_state', 699), 2)                  # no watchdog: PINGs keep the link
        self.assertEqual([b.channel_at('drive_l', t) for t in (320, 400, 499)], [0, 0, 0])
        self.assertEqual(b.channel_at('fault_bits', 499), 0x0800)
        self.assertEqual(b.channel_at('fault_bits', 500), 0)                 # the next C clears the TTL bit
        self.assertEqual(b.channel_at('drive_l', 500), 2)                    # dwell since the brake at 320 is over

    def test_q_window_and_old_commands(self):
        b = self.make(script_firmware())
        b.send(crcline('ARM 89025CC1'), tick=10)
        b.send(crcline('C 5 300 0 0 0'), tick=20)
        b.send(crcline('C 5 300 0 0 0'), tick=21)        # same q
        b.send(crcline('C 4 300 0 0 0'), tick=22)        # older
        b.send(crcline('C 32772 300 0 0 0'), tick=23)    # d = 32767: new
        b.send(crcline('C 5 300 0 0 0'), tick=24)        # d = 32769: old
        b.send(crcline('C 70000 300 0 0 0'), tick=25)    # q > 65535: range, q printed as '-'
        b.send(crcline('C 32773 301 0 0 0'), tick=26)    # ttl > ttlmax: range
        b.send(crcline('C 32773 300 0 0 1001'), tick=27)  # |S| > 1000: range
        b.run_until(60)
        self.assertEqual(self.events(b, since=11), [
            (21, body_crc('N 5 old')), (22, body_crc('N 4 old')), (24, body_crc('N 5 old')),
            (25, body_crc('N - range')), (26, body_crc('N 32773 range')), (27, body_crc('N 32773 range'))])
        self.assertEqual(self.status(b, 50)['q'], 32772)

    def test_steering_rate_limit_moving_and_standstill(self):
        b = self.make(script_firmware(raw=512))
        b.send(crcline('ARM 89025CC1'), tick=10)
        keep_driving(b, 20, 200, left=0, right=0, steer=600)
        b.run_until(210)
        # standstill: rstd 480 permille/s; servo updates at ticks 20, 22, ...: cmd = floor(0.96 m) after m updates
        cmds = [self.status(b, t)['cmd'] for t in (50, 100, 150, 200)]
        self.assertEqual(cmds, [math.floor(0.96 * 16), math.floor(0.96 * 41), math.floor(0.96 * 66),
                                math.floor(0.96 * 91)])
        self.assertEqual([self.status(b, t)['tgt'] for t in (50, 200)], [600, 600])
        moving = self.make(script_firmware(raw=512))
        moving.send(crcline('ARM 89025CC1'), tick=10)
        keep_driving(moving, 20, 600, left=50, right=50, steer=900)
        moving.run_until(510)
        step = [self.status(moving, t + 50)['cmd'] - self.status(moving, t)['cmd'] for t in (250, 300, 350)]
        self.assertEqual(step, [48, 48, 48])                                 # rmov 960 permille/s while moving

    def test_soft_limit_blocks_outward_output(self):
        b = self.make(script_firmware(raw=842))            # pos 1000 > soft_l 935
        # script ADC steps of 17 and 25 counts would trip POT_JUMP (> jmp 8) and POT_NOISE (step variance > 25)
        b.send(crcline('CFG jmp 1023'), tick=2)
        b.send(crcline('CFG nsd 1000'), tick=3)
        b.send(crcline('ARM 89025CC1'), tick=10)
        b.send(crcline('C 1 300 0 0 1000'), tick=20)
        b.event('adc_set', tick=30, arg=825)               # pos 948 >= 935, cmd ~ 990: outward push wanted
        b.event('adc_set', tick=50, arg=800)               # pos 873 < 935: allowed
        b.run_until(110)
        st = self.status(b, 50)
        self.assertEqual((st['pos'], st['tgt']), (fw_eeprom.raw_to_permille(842, 512, 181, 825), 1000))
        self.assertEqual(st['fl'] & 0x0080, 0x0080)          # SOFTLIMIT condition bit
        self.assertEqual([b.channel_at('steer_pwm', t) for t in (40, 45, 49)], [0, 0, 0])
        self.assertEqual([b.channel_at('steer_en', t) for t in (40, 45, 49)], [1, 1, 1])   # BRAKE
        self.assertGreater(b.channel_at('steer_pwm', 65), 0)                   # D6 (mdir +1), inside the limit
        st = self.status(b, 100)
        self.assertEqual((st['pos'], st['fl'] & 0x0080), (873, 0))

    def test_model_steering_tracks_target_and_holds(self):
        b, details = self.calibrated()
        b.send(crcline('ARM ' + b.boot_id(0)), tick=10)
        keep_driving(b, 20, 3000, left=0, right=0, steer=200)
        b.send(b'STOP\n', tick=3000)
        b.run_until(3000)
        st = self.status(b, 2950)
        self.assertEqual(st['cmd'], 200)
        self.assertLessEqual(abs(st['pos'] - 200), 40)                      # hold tolerance 2 db (spec 6.5)
        self.assertEqual([t for t in self.events(b, since=11)], [])
        target_motor = math.radians(200. / details['scale_permille_per_deg'])
        self.assertLessEqual(abs(b.last.base.motor_side_angle_rad - target_motor), math.radians(1.2))
        delta_at_stop = b.last.base.delta_eq_rad
        worst = 0.
        for out in b.run(1990, step_ticks=10):
            worst = max(worst, abs(out.base.delta_eq_rad - delta_at_stop))
        self.assertLessEqual(worst, math.radians(1.0))                     # held at the measured position
        b.run(20)
        self.assertEqual(b.channel_at('steer_en', 4999), 1)
        self.assertEqual(b.channel_at('steer_en', 5000), 0)


# --- watchdog, faults -----------------------------------------------------------------------------------------------------
class FaultTest(FirmwareCase):
    def test_link_watchdog_latch_and_late_ping(self):
        b = self.make(script_firmware())
        b.send(crcline('ARM 89025CC1'), tick=10)
        b.send(crcline('C 1 300 100 100 0'), tick=20)
        b.send(b'PING\n', tick=521)                       # arrives in the watchdog tick: too late (step 3 < step 4)
        b.send(b'PING\n', tick=600)
        b.run_until(700)
        self.assertEqual(self.events(b, since=11), [(320, 'F TTL 1 320*6B'), (521, body_crc('W 521'))])
        self.assertEqual(b.channel_at('mcu_state', 520), 2)
        self.assertEqual(b.channel_at('mcu_state', 521), 1)
        self.assertEqual(b.channel_at('fault_bits', 521), 0x1000)    # WATCHDOG latched; TTL bit gone with ARMED
        self.assertEqual(b.channel_at('fault_bits', 699), 0x1000)    # a later PING does not clear it
        self.assertEqual(b.channel_at('steer_en', 699), 1)           # wdmode hold: HOLD for hloss
        b.send(crcline('ARM 89025CC1'), tick=700)
        b.run_until(720)
        self.assertEqual(b.channel_at('fault_bits', 700), 0)
        self.assertEqual(b.channel_at('mcu_state', 700), 2)

    def test_watchdog_in_bench(self):
        b = self.make(script_firmware())
        b.send(crcline('BENCH 89025CC1'), tick=200)
        b.send(b'D8 100\n', tick=210)
        b.run_until(800)
        self.assertEqual(self.events(b, since=200), [(210, body_crc('P D8 0 100')), (711, body_crc('W 711'))])
        self.assertEqual(b.channel_at('drive_l', 711), 0)
        self.assertEqual(b.channel_at('mcu_state', 711), 1)

    def pot_fault(self, kind, params):
        b, _ = self.calibrated(injections=[dict(kind=kind, at_s=0.2, params=params)])
        b.send(crcline('ARM ' + b.boot_id(0)), tick=10)
        keep_driving(b, 20, 400, left=0, right=0)
        b.run_until(400)
        faults = self.faults(b, since=11)
        tick = faults[0][0]
        self.assertEqual(b.channel_at('mcu_state', tick - 1), 2)
        self.assertEqual(b.channel_at('mcu_state', tick), 1)
        self.assertEqual(b.channel_at('steer_en', tick), 0)               # steering output 0 in the fault tick
        return faults

    @staticmethod
    def noise_arg(values):
        """Spec 6.7 POT_NOISE argument floor(100 * var) of a raw window, var = (W sum x^2 - (sum x)^2) / W^2."""
        w = len(values)
        return 100 * (w * sum(x * x for x in values) - sum(values) ** 2) // (w * w)

    def test_pot_open_short_jump(self):
        """The model raw changes at sample 200 (read at tick 200). POT_NOISE looks at the raw window (spec 6.7,
        12.1 item 30), so the single step already trips it at tick 200; the median of 5 follows at tick 202 and
        the code of the fault latches then (OPEN, SHORT: condition on f; JUMP: f[n] - f[n - 2])."""
        before = [512] * 49
        self.assertEqual(self.pot_fault('pot_open', {}), [
            (200, body_crc('F POT_NOISE %d 200' % self.noise_arg(before + [0]))),
            (202, body_crc('F POT_OPEN 0 202'))])
        self.assertEqual(self.pot_fault('pot_short', {}), [
            (200, body_crc('F POT_NOISE %d 200' % self.noise_arg(before + [1023]))),
            (202, body_crc('F POT_SHORT 1023 202'))])
        self.assertEqual(self.pot_fault('pot_jump', dict(counts=60, duration_s=0.1)), [
            (200, body_crc('F POT_NOISE %d 200' % self.noise_arg(before + [572]))),
            (202, body_crc('F POT_JUMP 60 202'))])

    def test_pot_noise(self):
        b = self.make(dict(boot_seed=1, chip_id=1, adc_mode='model', eeprom_hex=None),
                      injections=[dict(kind='pot_noise_x10', at_s=0.5, params=dict(scale=20.))])
        b.send(crcline('CFG jmp 1023'), tick=10)                         # isolate the noise detector
        b.run_until(700)
        faults = self.faults(b, since=11)
        self.assertEqual(len(faults), 1, faults)
        tick, text = faults[0]
        self.assertTrue(text.startswith('F POT_NOISE '), text)
        self.assertTrue(500 <= tick <= 560, tick)
        self.assertGreater(int(text.split(' ')[2]), 2500)                # floor(100 var) > (nsd / 10)^2 * 100

    def test_polarity_fault_and_no_false_trip(self):
        good, _ = self.calibrated()
        bad, _ = self.calibrated(injections=[dict(kind='polarity_reversed', at_s=0., params={})])
        for b in (good, bad):
            b.send(crcline('ARM ' + b.boot_id(0)), tick=10)
            keep_driving(b, 20, 1500, left=0, right=0, steer=200)
            b.run_until(1500)
        self.assertEqual(self.faults(good, since=11), [])
        faults = self.faults(bad, since=11)
        self.assertTrue(faults and faults[0][1].split(' ')[1] == 'POLARITY', faults)
        self.assertLess(int(faults[0][1].split(' ')[2]), -40)            # moved away from the output by > polp
        self.assertGreaterEqual(faults[0][0], 20 + 100)
        self.assertEqual(bad.channel_at('steer_en', faults[0][0]), 0)

    def test_driver_stuck_fault(self):
        b, _ = self.calibrated(injections=[dict(kind='driver_stuck_on', at_s=1.0, params=dict(pwm=0.6))])
        b.send(crcline('CFG db 200'), tick=5)            # wide deadband: the servo keeps BRAKE (u = 0, EN 1)
        b.send(crcline('ARM ' + b.boot_id(0)), tick=10)
        for tick in range(100, 2000, 100):
            b.send(b'PING\n', tick=tick)
        b.run_until(2000, vehicle_stopped=False)
        faults = self.faults(b, since=11)
        self.assertTrue(faults and faults[0][1].split(' ')[1] == 'DRV_STUCK', faults)
        self.assertTrue(1150 <= faults[0][0] <= 1400, faults)          # about 2 deg of motion within 300 ms
        self.assertGreater(abs(int(faults[0][1].split(' ')[2])), 80)
        self.assertEqual(b.channel_at('steer_en', faults[0][0]), 0)     # EN 0 cuts a stuck driver

    def test_driver_stuck_with_the_default_record(self):
        """Decision 12.1-70: default servo (db 20, re-push at 2 db) and dsm 80. A driver stuck on moves the steering
        while the MCU brakes, and on while the servo pushes back: DRV_STUCK (not POLARITY). BRAKE ticks alone never
        held more than about 3 db of motion, so DRV_STUCK could not trip with these values before. The latch then
        refuses ARM with pol until a passing CAL POL (decision 12.1-71)."""
        for pwm in (0.6, 1.0):
            with self.subTest(pwm=pwm):
                b, _ = self.calibrated(clean=False,
                                       injections=[dict(kind='driver_stuck_on', at_s=1.0, params=dict(pwm=pwm))])
                b.send(crcline('ARM ' + b.boot_id(0)), tick=10)
                for tick in range(100, 2000, 100):
                    b.send(b'PING\n', tick=tick)
                b.send(crcline('ARM ' + b.boot_id(0)), tick=2000)
                b.run_until(2050, vehicle_stopped=False, vehicle_speed_m_s=0.4)
                faults = self.faults(b, since=11)
                self.assertTrue(faults and faults[0][1].split(' ')[1] == 'DRV_STUCK', faults)
                self.assertEqual(len(faults), 1, faults)
                self.assertTrue(1000 < faults[0][0] <= 1400, faults)           # 2 deg (dsm) within dst 300 ms
                self.assertGreater(abs(int(faults[0][1].split(' ')[2])), 80)
                self.assertEqual(b.channel_at('steer_en', faults[0][0]), 0)
                self.assertEqual(b.channel_at('mcu_state', faults[0][0]), 1)
                self.assertIn((2000, body_crc('N - pol')), self.events(b, since=2000))
                self.assertEqual(b.channel_at('mcu_state', 2049), 1)

    def test_large_steps_and_sweeps_do_not_trip_track(self):
        """Decision 12.1-69: a servo that lags behind the rate-limited cmd but follows it is not a TRACK fault. With
        the placeholder P servo the lag at the rate limit is about 6 deg (> trk 3 deg) for longer than trt: standstill
        steps of 22 deg (seeds 0..2) and a full sweep while moving latched TRACK before (2026-11-09 review)."""
        cases = [(seed, ((4000, s),), 0, False) for seed in (0, 1, 2) for s in (900, -900)]
        cases.append((0, ((1500, -900), (4500, 900)), 102, True))
        for seed, steps, duty, moving in cases:
            with self.subTest(seed=seed, steps=steps, moving=moving):
                b, _ = self.calibrated(seed=seed, clean=False)
                b.send(crcline('ARM ' + b.boot_id(0)), tick=10)
                q, tick = 1, 20
                for end, steer in steps:
                    q = keep_driving(b, tick, end, q=q, left=duty, right=duty, steer=steer, every=50)
                    tick += 50 * ((end - tick + 49) // 50)
                end = steps[-1][0]
                b.run_until(end, vehicle_stopped=not moving, vehicle_speed_m_s=0.4 if moving else 0.0)
                self.assertEqual(self.faults(b, since=11), [])
                self.assertEqual(b.channel_at('mcu_state', end - 1), 2)
                final = self.status(b, end - 50)
                self.assertLessEqual(abs(final['pos'] - final['cmd']), 2 * 20 + 10, final)   # it got there

    def stall_setup(self, extra=()):
        b = self.make(script_firmware(raw=512))           # pos fixed: the MCU sees a jammed actuator
        for i, text in enumerate(extra):
            b.send(crcline(text), tick=2 + i)
        b.send(crcline('ARM 89025CC1'), tick=10)
        keep_driving(b, 20, 1400, left=0, right=0, steer=1000)
        return b

    def test_stall_fault_exact_tick(self):
        """cmd = floor(0.96 m) after m servo updates (ticks 20, 22, ...); |u| >= stp 153 from cmd 160
        (m 167, tick 352); runStall reaches stt 500 at tick 852; |P[852] - P[352]| = 0 < stm."""
        b = self.stall_setup()
        b.run_until(900)
        self.assertEqual(self.faults(b, since=11), [(852, body_crc('F STALL 0 852'))])
        self.assertEqual((b.channel_at('steer_pwm', 351), b.channel_at('steer_pwm', 352)), (152, 153))
        self.assertEqual(b.channel_at('steer_en', 852), 0)
        self.assertEqual(b.channel_at('mcu_state', 852), 1)

    def test_track_fault_exact_tick(self):
        """With stm 0 (no STALL): |cmd - P| > trk 120 from tick 273 (cmd 121 after the update at 272);
        runTrack reaches trt 1000 at tick 1272; arg cmd - P = floor(0.96 * 626) = 600."""
        b = self.stall_setup(extra=('CFG stm 0',))
        b.run_until(1300)
        self.assertEqual(self.faults(b, since=11), [(1272, body_crc('F TRACK 600 1272'))])

    def test_thermal_fault_from_jog(self):
        # While the THERMAL condition holds, CAL JOG and CAL POL are refused like ARM (decision 12.1-68).
        b = self.make(script_firmware(raw=512))
        b.send(crcline('CFG tha 300'), tick=50)
        b.send(crcline('CAL JOG 255 500'), tick=100)
        b.send(crcline('ARM 89025CC1'), tick=450)
        b.send(crcline('CAL JOG 255 500'), tick=452)
        b.send(crcline('CAL POL'), tick=454)
        b.run_until(600)
        self.assertEqual(self.events(b, since=50), [
            (50, body_crc('A CFG tha 300')), (100, body_crc('A JOG 255 500')), (401, body_crc('F THERMAL 301 401')),
            (450, body_crc('N - fault')), (452, body_crc('N - fault')), (454, body_crc('N - fault'))])
        self.assertEqual(b.channel('steer_pwm'), [(100, 255), (401, 0)])
        self.assertEqual(b.channel('steer_en'), [(100, 1), (401, 0)])


# --- calibration --------------------------------------------------------------------------------------------------------
class CalibrationTest(FirmwareCase):
    @staticmethod
    def jog(b, pwm, times, at, gap=520):
        for _ in range(times):
            b.send(crcline('CAL JOG %d 500' % pwm), tick=at)
            at += gap
        return at

    def test_cal_flow_with_polarity_check(self):
        """Bench (wheels lifted: moving actuator values). JOG at 200 (< thp 204: no thermal count) to the ends."""
        b = self.make(dict(boot_seed=1, chip_id=0x5A1B2C, adc_mode='model', eeprom_hex=None), clean=True)
        t = self.jog(b, 200, 4, at=100)               # to the left end stop
        b.send(crcline('CAL MARK L'), tick=t)
        t = self.jog(b, -200, 8, at=t + 10)           # to the right end stop
        b.send(crcline('CAL MARK R'), tick=t)
        t = self.jog(b, 200, 3, at=t + 10)            # back near the centre
        b.send(crcline('CAL MARK C'), tick=t)
        b.send(crcline('CAL SOFT 935 -935'), tick=t + 1)
        b.send(crcline('CAL POL'), tick=t + 10)
        b.send(crcline('CAL SAVE'), tick=t + 1100)
        b.send(crcline('ARM ' + b.boot_id(0)), tick=t + 1200)
        b.run_until(t + 1300, vehicle_stopped=False)
        self.assertEqual(self.faults(b, since=1), [])
        cal_lines = [line for line in b.lines() if line.kind == 'CAL'][1:]
        fields = dict(part.split('=', 1) for part in cal_lines[-1].body.split(' ')[2:])
        raw_l, raw_c, raw_r = int(fields['left']), int(fields['center']), int(fields['right'])
        self.assertTrue(raw_l - raw_c >= 50 and raw_c - raw_r >= 50, fields)
        pol = [line.text for line in b.lines() if line.text.startswith('A POL ')]
        self.assertEqual(len(pol), 1)
        _, _, verdict, d1, d2 = pol[0][:-3].split(' ')
        self.assertEqual(verdict, 'ok')
        self.assertTrue(int(d1) >= 4 and int(d2) <= -4, pol)
        expected = fw_eeprom.encode_record(1, raw_l, raw_c, raw_r, 935, -935, pol_ok=1, mdir=1)
        self.assertEqual(fields['id'], '1')                                    # blank EEPROM: cal_id 1
        self.assertEqual(fields['crc'], '%08X' % struct.unpack_from('<I', expected, 102)[0])
        self.assertEqual((fields['pol'], fields['mdir'], fields['caldirty'], fields['cfgdirty']), ('ok', '1', '0', '0'))
        self.assertEqual(b.channel('eeprom_commits')[-1][1], 1)
        self.assertEqual(b.channel_at('mcu_state', t + 1250), 2)              # ARM accepted after SAVE

    def test_save_survives_reset_and_cal_id_increments(self):
        b = self.make(script_firmware(raw=512))
        b.send(crcline('CAL SAVE'), tick=100)
        b.event('mcu_reset', tick=200, arg=4)
        b.send(crcline('CAL SAVE'), tick=300)
        b.send(crcline('CAL ERASE'), tick=310)
        b.event('mcu_reset', tick=400, arg=1)
        b.run_until(500)
        cal = [(line.queued_tick, line.text) for line in b.lines() if line.kind in ('CAL', 'BANNER')]
        self.assertIn(' id=8 ', cal[2][1])                                     # SAVE: 7 -> 8
        self.assertEqual(cal[3][0], 200)
        self.assertIn(' boot=%s rst=soft_restart ' % BOOT_IDS_SEED1[1], cal[3][1])
        self.assertIn(' cal=8 ', cal[3][1])                                     # kept across the reset
        self.assertIn(' id=9 ', cal[5][1])
        erase = [text for tick, text in self.events(b, since=310) if tick == 310]
        self.assertEqual(erase[0], body_crc('F NOCAL 0 110'))                   # millis 110 after the reset at 200
        self.assertIn(' id=none crc=00000000 left=- center=- right=- soft=0,0 pol=no ', erase[1])
        self.assertIn(' boot=%s rst=hw_wdt ' % BOOT_IDS_SEED1[2], cal[7][1])
        self.assertIn(' cal=none ', cal[7][1])
        # the tombstone keeps the id: mark again and save -> 10 (spec 9.5)
        for tick, raw, mark in ((600, 842, 'L'), (620, 512, 'C'), (640, 181, 'R')):
            b.event('adc_set', tick=tick, arg=raw)
            b.send(crcline('CAL MARK %s' % mark), tick=tick + 10)
        b.send(crcline('CAL SOFT 935 -935'), tick=660)
        b.send(crcline('CAL SAVE'), tick=670)
        b.run_until(700)
        last = [line.text for line in b.lines() if line.kind == 'CAL'][-1]
        self.assertIn(' id=10 ', last)
        self.assertIn(' pol=no ', last)
        self.assertEqual(b.channel('eeprom_commits')[-1][1], 4)

    def test_wear_limit_eight_commits_per_boot(self):
        b = self.make(script_firmware())
        for i in range(9):
            b.send(crcline('CAL SAVE'), tick=100 + i)
        b.run_until(260)          # nine cal lines need about 95 ms of wire time
        ev = self.events(b, since=100)
        self.assertEqual(len(ev), 9)
        self.assertIn(' id=15 ', ev[7][1])
        self.assertEqual(ev[8], (108, body_crc('N - wear')))


# --- BENCH --------------------------------------------------------------------------------------------------------------
class BenchTest(FirmwareCase):
    def test_bench_d_pins_ramp_and_rejections(self):
        b = self.make(script_firmware())
        b.send(b'D8 100\n', tick=50)                          # DISARMED: state
        b.send(crcline('BENCH 89025CC1'), tick=195)
        b.send(b'D8 100\n', tick=195)
        for i, text in enumerate((b'D1 0', b'D1 128', b'D2 1', b'D4 255', b'D6 1', b'D7 1', b'D2 300')):
            b.send(text + b'\n', tick=196 + i)
        b.send(crcline('ARM 89025CC1'), tick=203)             # BENCH -> ARMED directly: state
        for tick in range(400, 1600, 200):
            b.send(b'PING\n', tick=tick)                      # BENCH watchdog
        b.send(b'D5 60\n', tick=700)                          # reversal: decelerate, 120 ms, then reverse
        b.run_until(1600)
        self.assertEqual(self.events(b, since=50), [
            (50, body_crc('N - state')), (195, body_crc('P D8 0 100')), (196, body_crc('P D1 255 255')),
            (197, body_crc('P D1 255 255')), (198, body_crc('N - pin')), (199, body_crc('N - pin')),
            (200, body_crc('N - pin')), (201, body_crc('N - pin')), (202, body_crc('N - range')),
            (203, body_crc('N - state')), (700, body_crc('P D5 0 60'))])
        expect = {199: 0, 200: 2, 210: 5, 589: 99, 590: 100}  # grid from the boot tick 0, every 10 ms
        self.assertEqual({t: b.channel_at('drive_l', t) for t in expect}, expect)
        values = [b.channel_at('drive_l', t) for t in range(700, 1600)]
        zero_at = 700 + values.index(0)
        first_neg = 700 + next(i for i, v in enumerate(values) if v < 0)
        self.assertGreaterEqual(first_neg - zero_at, 120)
        self.assertEqual(b.channel_at('drive_l', 1599), -60)
        self.assertEqual(b.channel_at('drive_r', 1599), 0)
        self.assertEqual(b.channel('drive_en'), [(0, 1)])     # D1 never written again

    def test_bench_framing_error_brakes_and_repeated_zero_has_no_gpio_write(self):
        b = self.make(script_firmware())
        b.send(crcline('BENCH 89025CC1'), tick=200)
        b.send(b'D8 128\n', tick=200)
        b.send(b'D8 128\r\n', tick=500)                       # CR LF accepted
        b.send(b'D8 2\r55\n', tick=600)                       # embedded CR: ctrl, brake at once
        b.send(b'D8 64\r\n', tick=650)                        # recovers (after the 120 ms dwell)
        b.send(b'D8 0\n', tick=1010)                          # release
        b.send(b'D8 0\nD8 0\nD8 0\n', tick=1400)              # repeated zero
        b.run_until(1410)
        self.assertGreater(b.channel_at('drive_l', 599), 0)
        self.assertEqual(b.channel_at('drive_l', 600), 0)
        self.assertEqual(b.channel_at('mcu_state', 600), 3)   # stays BENCH
        self.assertEqual(b.channel_at('drive_l', 1000), 64)
        self.assertEqual(b.channel_at('drive_l', 1399), 0)
        self.assertEqual([text for tick, text in self.events(b, since=1400)], [body_crc('P D8 0 0')] * 3)
        self.assertEqual(b.channel_at('gpio_writes', 1409), b.channel_at('gpio_writes', 1399))


# --- resets, events, determinism, protocol --------------------------------------------------------------------------------
class SessionTest(FirmwareCase):
    def test_port_open_reset_new_banner_and_boot_id(self):
        b = self.make(script_firmware(boot_delay_ticks=70, boot_garbage_hex='F0'))
        b.send(crcline('ARM 89025CC1'), tick=80)
        b.send(crcline('C 1 300 80 80 0'), tick=100)
        b.send(crcline('C 2 300 80 80 0'), tick=200)
        b.event('port_open_reset', tick=250)
        b.send(b'PING\n', tick=260)                           # during the boot delay: lost
        b.send(crcline('ARM 89025CC1'), tick=400)             # the old boot_id
        b.send(crcline('ARM 658EEC67'), tick=410)
        b.run_until(500)
        self.assertGreater(b.channel_at('drive_l', 249), 0)
        self.assertEqual([b.channel_at(name, 250) for name in ('drive_l', 'drive_en', 'steer_en', 'mcu_state')],
                         [0, 0, 0, 255])
        banners = [line for line in b.lines() if line.kind == 'BANNER']
        self.assertEqual([line.queued_tick for line in banners], [70, 320])
        self.assertIn(' boot=658EEC67 rst=ext_sys ', banners[1].text)
        self.assertEqual(self.events(b, since=400), [(400, body_crc('N - boot'))])
        self.assertEqual(b.channel_at('mcu_state', 410), 2)
        self.assertEqual(b.last.rx_lost_total, 5)
        self.assertEqual(b.last.boot_count, 2)
        garbage = [e for e in b.tx_bytes if e.boot_garbage]
        self.assertEqual([(e.byte, e.queued_tick) for e in garbage], [(0xF0, 0), (0xF0, 250)])
        status = self.status(b, 370)
        self.assertEqual((status['st'], status['fl'], status['ms']), (1, 0x4000, 120))   # millis since the reset

    def test_power_cycle_and_motor_power(self):
        b, _ = self.calibrated()
        b.event('power_cycle', tick=100, arg=50)
        b.send(b'PING\n', tick=120)
        b.run_until(200)
        self.assertEqual(b.channel_at('mcu_state', 99), 1)
        self.assertEqual(b.channel_at('mcu_state', 100), 255)
        self.assertEqual(b.channel_at('mcu_state', 150), 1)                   # power-on reset at 150, delay 0
        banners = [line for line in b.lines() if line.kind == 'BANNER']
        self.assertIn(' rst=power_on ', banners[1].text)
        self.assertEqual(banners[1].queued_tick, 150)
        self.assertEqual(b.last.rx_lost_total, 5)
        m = self.make(script_firmware(motor_power=False))
        m.send(crcline('CAL JOG 255 100'), tick=10)
        m.event('motor_power', tick=50, arg=1)
        m.run_until(80)
        self.assertEqual(m.channel_at('steer_pwm', 20), 255)
        self.assertEqual(m.last.run_flags & v2.RUN_MOTOR_POWER, v2.RUN_MOTOR_POWER)

    def test_mcu_stall_ramp_catch_up_is_capped(self):
        b = self.make(script_firmware())
        b.send(crcline('CFG wd 5000'), tick=2)
        b.send(crcline('CFG ttlmax 1000'), tick=3)
        b.send(crcline('ARM 89025CC1'), tick=10)
        b.send(crcline('C 1 1000 255 255 0'), tick=20)
        b.event('mcu_stall', tick=300, arg=600)
        b.run_until(950)
        before = b.channel_at('drive_l', 299)
        self.assertEqual(b.channel_at('drive_l', 899), before)                 # pins hold during the stall
        jump = b.channel_at('drive_l', 900) - before
        self.assertTrue(0 < jump <= 13, jump)                                   # 50 ms worth at most
        self.assertIn(b.channel_at('drive_l', 910) - b.channel_at('drive_l', 900), (2, 3))
        self.assertEqual(self.events(b, since=1)[:2], [(2, body_crc('A CFG wd 5000')),
                                                       (3, body_crc('A CFG ttlmax 1000'))])

    def test_millis_rollover(self):
        start = 2 ** 32 - 300
        b = self.make(script_firmware(millis_start=start))
        b.send(crcline('ARM 89025CC1'), tick=10)
        b.send(crcline('C 1 300 100 100 0'), tick=20)
        b.run_until(401)
        self.assertEqual(self.status(b, 250)['ms'], (start + 250) % 2 ** 32)
        self.assertEqual(self.status(b, 350)['ms'], 50)
        self.assertEqual(self.events(b, since=11), [(320, body_crc('F TTL 1 20'))])
        self.assertEqual(b.channel_at('drive_l', 319), 51)
        self.assertEqual(b.channel_at('drive_l', 320), 0)

    def test_reset_frame_restarts_the_session(self):
        b = self.make(script_firmware())
        b.send(crcline('CAL SAVE'), tick=100)
        b.run_until(200)
        self.assertIn(' id=8 ', [line.text for line in b.lines() if line.kind == 'CAL'][-1])
        out = b.reset()
        self.assertEqual((out.ticks, out.fault_bits, out.drive_en, out.steer_en, b.ticks), (0, 0, False, False, 0))
        self.assertEqual(out.pot_raw, 512)
        b.run_until(60)
        lines = b.lines()
        self.assertIn(' boot=89025CC1 ', lines[0].text)                       # RNG restarted from boot_seed
        self.assertEqual(lines[1].text, CAL7_LINE)                             # EEPROM back to the startup image

    def test_determinism_and_step_partition(self):
        def scenario(step_ticks):
            b, _ = self.calibrated(seed=3, clean=False)
            b.send(crcline('ARM ' + b.boot_id(0)), tick=10, pace='uart')
            q = 1
            for tick in range(20, 1500, 100):
                b.send(crcline('C %d 300 60 60 %d' % (q, 300 if tick < 800 else -200)), tick=tick, pace='uart')
                q += 1
            b.event('adc_model', tick=900)
            b.run_until(1500, step_ticks=step_ticks, vehicle_stopped=False, vehicle_speed_m_s=0.3)
            return ([(e.queued_tick, e.done_tick, e.byte, e.boot_garbage) for e in b.tx_bytes],
                    [(c.tick, c.channel, c.value) for c in b.changes], b.last.base)
        first = scenario(10)
        self.assertEqual(first, scenario(10))
        self.assertEqual(first[:2], scenario(7)[:2])
        self.assertEqual(first[:2], scenario(1)[:2])
        self.assertTrue(any(name == 'steer_pwm' for _, name, _ in first[1]))

    def test_report_servo_channels(self):
        """firmware.report_servo (LOCKSTEP 11.6, SIM evaluation only): channels 11..14 carry the firmware's P[n], tgt,
        cmd and steering mode after every tick; they agree with the status line fields at its tick; without the
        option nothing else in the response changes (same bytes, same other channels)."""
        def scenario(report):
            b, _ = self.calibrated(seed=3, clean=False, report_servo=report)
            b.send(crcline('ARM ' + b.boot_id(0)), tick=10, pace='uart')
            q = 1
            for tick in range(20, 1500, 50):
                b.send(crcline('C %d 300 60 60 %d' % (q, 400 if tick < 700 else -300)), tick=tick, pace='uart')
                q += 1
            b.send(crcline('STOP'), tick=1500, pace='uart')
            b.run_until(3700, vehicle_stopped=False, vehicle_speed_m_s=0.3)   # STOP hold 2 s, then OFF
            return b
        with_servo, without = scenario(True), scenario(False)
        self.assertEqual([(e.queued_tick, e.done_tick, e.byte) for e in with_servo.tx_bytes],
                         [(e.queued_tick, e.done_tick, e.byte) for e in without.tx_bytes])
        others = [(c.tick, c.channel, c.value) for c in with_servo.changes if c.channel not in v2.SERVO_CHANNELS]
        self.assertEqual(others, [(c.tick, c.channel, c.value) for c in without.changes])
        self.assertFalse([c for c in without.changes if c.channel in v2.SERVO_CHANNELS])
        self.assertTrue(all(with_servo.channel(name) for name in v2.SERVO_CHANNELS))
        checked = 0
        for line in with_servo.lines():
            if line.kind != 'S':
                continue
            s = line.status()
            tick = line.queued_tick
            self.assertEqual(with_servo.channel_at('servo_pos', tick), s['pos'], tick)
            self.assertEqual(with_servo.channel_at('servo_tgt', tick), s['tgt'], tick)
            self.assertEqual(with_servo.channel_at('servo_cmd', tick), s['cmd'], tick)
            checked += 1
        self.assertGreater(checked, 30)
        modes = [v for _, v in with_servo.channel('steer_mode')]
        self.assertEqual(modes[:3], [1, 2, 0])          # ARM -> ACTIVE, STOP -> HOLD, hstop later -> OFF
        stop_tick = with_servo.channel('steer_mode')[1][0]
        # hold_measured (decision R13-2, spec 6.5): tgt = cmd = P[t0] at the STOP tick, the filtered position
        self.assertEqual(with_servo.channel_at('servo_tgt', stop_tick), with_servo.channel_at('servo_pos', stop_tick))
        self.assertEqual(with_servo.channel_at('servo_cmd', stop_tick), with_servo.channel_at('servo_pos', stop_tick))
        self.assertNotEqual(with_servo.channel_at('servo_tgt', stop_tick - 1), with_servo.channel_at('servo_pos',
                                                                                                     stop_tick))
        self.assertEqual(v2.CHANNEL_CODES['servo_pos'], 11)
        self.assertEqual(v2.CHANNEL_CODES['steer_mode'], 14)
        self.assertIn('report_servo', v2.FIRMWARE_KEYS)

    def test_uart_arrival_rule(self):
        ticks, end = v2.host_arrival_ticks(32, 0)              # protocol 11.7 example
        self.assertEqual(ticks[:11], [1] * 11)
        self.assertEqual(ticks[11:23], [2] * 12)
        self.assertEqual(ticks[23:], [3] * 9)
        self.assertEqual(end, 32 * 50)
        self.assertEqual(v2.mcu_done_tick(100 * 576 + 50), 100)
        self.assertEqual(v2.mcu_done_tick(100 * 576 + 85 * 50), 107)
        b = self.make(script_firmware())
        b.run_until(120)
        status = [e for e in b.tx_bytes if e.queued_tick == 100]
        self.assertEqual(status[0].done_tick, 100)
        self.assertEqual(status[-1].done_tick, v2.mcu_done_tick(100 * 576 + 50 * len(status)))

    def test_step_contract_of_the_backend(self):
        b = self.make(script_firmware())
        out = b.step(ba.SteerStepInput(dt_s=0.05, mode='firmware'))
        self.assertIsInstance(out, ba.SteerStepOutput)
        self.assertEqual(out.ticks, 50)
        self.assertTrue(out.serial_out.startswith(b'UBTS_STEER_V3 hello '))
        self.assertTrue(out.drive_en)
        out = b.step(ba.SteerStepInput(dt_s=0.05, mode='firmware', serial_in=b'INFO\n'))
        self.assertIn(b'UBTS_STEER_V3 hello ', out.serial_out)
        lines, rest = assemble_lines(b.tx_bytes)
        self.assertEqual([line.kind for line in lines[:2]], ['BANNER', 'CAL'])
        with self.assertRaises(ValueError):
            b.step(ba.SteerStepInput(dt_s=0.2, mode='firmware'))
        label = b.result_label()
        self.assertEqual(label['backend'], 'native_lockstep_v2_firmware')
        self.assertEqual(label['firmware_build'], native_build.firmware_build_id())
        self.assertFalse(label['field_validated'])


class ProtocolTest(FirmwareCase):
    def startup(self, edit=None, mode='firmware'):
        if mode == 'firmware':
            cfg = v2.firmware_startup_config(profile(), 1, firmware=script_firmware())
        else:
            cfg = v2.v2_startup_config(profile(), 1, mode=mode)
        if edit:
            edit(cfg)
        return json.dumps(cfg, separators=(',', ':')).encode('ascii') + b'\n'

    @staticmethod
    def run_native(data):
        proc = subprocess.run([native().path], input=data, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
        return proc.returncode, proc.stdout, proc.stderr.decode('ascii', 'replace')

    def assertExit(self, data, code):
        returncode, _, stderr = self.run_native(data)
        self.assertEqual(returncode, code, stderr)
        if code:
            self.assertTrue(stderr.startswith('steer_native: %s: ' % ba.EXIT_CODES[code]), stderr)

    def test_startup_errors(self):
        def setter(path, value):
            def edit(cfg):
                node = cfg
                for key in path[:-1]:
                    node = node[key]
                node[path[-1]] = value
            return edit
        for edit in (setter(('mcu_tick_hz',), 2000), setter(('mcu', 'baud'), 230400),
                     setter(('firmware', 'expect_build'), 'ZZZZZZZZ'), setter(('firmware', 'eeprom_hex'), 'FF' * 255),
                     setter(('firmware', 'boot_garbage_hex'), 'F'), setter(('firmware', 'chip_id'), 2 ** 24),
                     setter(('firmware', 'adc_mode'), 'none'), setter(('firmware', 'initial_delta_eq_rad'), 1.0),
                     setter(('protocol',), ba.PROTOCOL)):
            self.assertExit(self.startup(edit), ba.EXIT_BAD_STARTUP)
        wrong = '00000000' if native_build.firmware_build_id() != '00000000' else '11111111'
        self.assertExit(self.startup(setter(('firmware', 'expect_build'), wrong)), ba.EXIT_BAD_STARTUP)
        right = self.startup(setter(('firmware', 'expect_build'), native_build.firmware_build_id()))
        self.assertEqual(self.run_native(right)[0], 0)
        self.assertExit(self.startup(lambda cfg: cfg.pop('firmware')), ba.EXIT_BAD_STARTUP)
        self.assertExit(self.startup(setter(('firmware',), {}), mode='direct_pwm'), ba.EXIT_BAD_STARTUP)
        code, out, err = self.run_native(self.startup())
        self.assertEqual((code, err), (0, ''))
        tick, build_id = v2.decode_ready_v2(out, expected_tick_hz=1000)
        self.assertRegex(build_id, r'^ubts-steer-native/2 src=h[0-9a-f]{12} fw=[0-9A-F]{8} cxx=\S+ bits=(32|64)$')

    def test_frame_errors(self):
        line = self.startup()
        good = v2.encode_fw_step_request(1, 0.01)
        step_v1 = ba.encode_step_request(1, ba.SteerStepInput(dt_s=0.01))
        cases = [
            (ba.HEADER.pack(b'USTP', 1, v2.FRAME_FW_STEP_REQUEST, 1, len(good) - 16) + good[16:], ba.EXIT_BAD_HEADER),
            (step_v1[:4] + struct.pack('<H', 2) + step_v1[6:], ba.EXIT_BAD_HEADER),   # 0x0001 in firmware mode
            (v2.encode_fw_step_request(2, 0.01), ba.EXIT_SEQUENCE),
            (v2.encode_fw_step_request(1, 0.2, check=False), ba.EXIT_BAD_VALUE),
            (v2.encode_fw_step_request(1, 0.01, events=[(10, 1, 1)], check=False), ba.EXIT_BAD_VALUE),
            (v2.encode_fw_step_request(1, 0.01, events=[(0, 9, 0)], check=False), ba.EXIT_BAD_VALUE),
            (v2.encode_fw_step_request(1, 0.01, events=[(0, 1, 7)], check=False), ba.EXIT_BAD_VALUE),
            (v2.encode_fw_step_request(1, 0.01, rx=[(5, 1), (4, 1)], check=False), ba.EXIT_BAD_VALUE),
            (good[:16 + 26] + struct.pack('<H', 65) + good[16 + 28:], ba.EXIT_BAD_LENGTH),
            (good[:12] + struct.pack('<I', 33) + good[16:] + b'\0', ba.EXIT_BAD_LENGTH),
            (good[:20], ba.EXIT_TRUNCATED),
        ]
        for frames, code in cases:
            self.assertExit(line + frames, code)
        code, out, err = self.run_native(line + good)
        self.assertEqual((code, err), (0, ''))

    def test_v2_direct_pwm_matches_v1(self):
        inp = ba.SteerStepInput(dt_s=0.01, steer_pwm=0.6, steer_en=True, vehicle_speed_m_s=0.3, vehicle_stopped=False)
        frames_v1 = b''.join(ba.encode_step_request(i, inp) for i in range(1, 21))
        frames_v2 = b''.join(v2.encode_step_request_v2(i, inp) for i in range(1, 21))
        code1, out1, _ = self.run_native(ba.startup_line(ba.native_startup_config(profile(), 4)) + frames_v1)
        code2, out2, _ = self.run_native(ba.startup_line(v2.v2_startup_config(profile(), 4)) + frames_v2)
        self.assertEqual((code1, code2), (0, 0))
        r1 = 16 + 8 + struct.unpack_from('<I', out1, 20)[0]
        r2 = 16 + 8 + struct.unpack_from('<I', out2, 20)[0]
        self.assertTrue(out1[24:r1].startswith(b'ubts-steer-native/1 '))
        self.assertTrue(out2[24:r2].startswith(b'ubts-steer-native/2 '))
        body1, body2 = out1[r1:], out2[r2:]
        self.assertEqual(len(body1), 20 * 96)
        self.assertEqual(len(body1), len(body2))
        for k in range(0, len(body1), 96):
            self.assertEqual(body1[k + 6:k + 96], body2[k + 6:k + 96])      # all but the header version
            self.assertEqual((body1[k + 4], body2[k + 4]), (1, 2))

    def test_v1_backend_still_refuses_firmware_mode(self):
        with self.assertRaises(NativeBackendError) as info:
            NativeSteeringBackend(profile(), 1, mode='firmware', executable=native().path)
        self.assertTrue(info.exception.stderr_line.startswith('steer_native: bad_startup: mode: firmware'))


# --- a checkout without the firmware sources --------------------------------------------------------------------------
class AbsentFirmwareTest(unittest.TestCase):
    """An image that copies only sim/ubts: the program builds without firmware mode, the v1 modes and the v2
    direct_pwm / test_servo modes give the same bytes, and firmware mode is refused with a clear error."""

    def test_build_without_firmware_sources(self):
        compiler_or_skip()
        temp = tempfile.mkdtemp(prefix='steer_fw_absent_')
        self.addCleanup(_cleanup_temp, temp)
        build_dir = os.path.join(temp, 'build')
        saved_dir, saved_env = native_build.FIRMWARE_DIR, os.environ.pop(native_build.ENV_FIRMWARE_DIR, None)
        native_build.FIRMWARE_DIR = Path(temp) / 'firmware' / 'ubts_steer_v3'          # not there
        try:
            self.assertFalse(native_build.firmware_present())
            result = native_build.ensure_built(build_dir=build_dir)
            with self.assertRaises(native_build.NativeBuildError):
                native_build.ensure_unit_test_built(build_dir=build_dir)
            with self.assertRaises(native_build.NativeBuildError) as info:
                FirmwareSteeringBackend(profile(), 1, firmware=script_firmware(), build_dir=build_dir)
            self.assertIn('firmware mode needs the firmware sources', str(info.exception))
        finally:
            native_build.FIRMWARE_DIR = saved_dir
            if saved_env is not None:
                os.environ[native_build.ENV_FIRMWARE_DIR] = saved_env
        self.assertIsNone(result.firmware_build_id)
        self.assertEqual(result.compiler_output.strip(), '')                               # -Wall -Wextra clean
        manifest = manifest_of(result.path)
        self.assertIsNone(manifest['firmware_build_id'])
        self.assertEqual(manifest['firmware_files'], [])

        def run(executable, data):
            proc = subprocess.run([executable], input=data, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                  timeout=60)
            return proc.returncode, proc.stdout, proc.stderr.decode('ascii', 'replace')

        def ready_end(out):
            return 16 + 8 + struct.unpack_from('<I', out, 20)[0]

        inp = ba.SteerStepInput(dt_s=0.01, steer_pwm=0.6, steer_en=True, vehicle_speed_m_s=0.3, vehicle_stopped=False)
        v1 = ba.startup_line(ba.native_startup_config(profile(), 4)) + b''.join(
            ba.encode_step_request(i, inp) for i in range(1, 21))
        v2_direct = ba.startup_line(v2.v2_startup_config(profile(), 4)) + b''.join(
            v2.encode_step_request_v2(i, inp) for i in range(1, 21))
        code1, out1, err1 = run(result.path, v1)
        code2, out2, err2 = run(result.path, v2_direct)
        self.assertEqual((code1, err1, code2, err2), (0, '', 0, ''))
        self.assertRegex(out1[24:ready_end(out1)].decode('ascii'),
                         r'^ubts-steer-native/1 src=h[0-9a-f]{12} cxx=\S+ bits=(32|64)$')
        self.assertRegex(out2[24:ready_end(out2)].decode('ascii'),
                         r'^ubts-steer-native/2 src=h[0-9a-f]{12} fw=none cxx=\S+ bits=(32|64)$')
        self.assertEqual(len(out1) - ready_end(out1), 20 * 96)
        code, out, err = run(result.path, ba.startup_line(
            v2.firmware_startup_config(profile(), 1, firmware=script_firmware())))
        self.assertEqual((code, out), (ba.EXIT_BAD_STARTUP, b''))
        self.assertTrue(err.startswith('steer_native: bad_startup: mode firmware: this program was built without '
                                       'the firmware sources'), err)
        backend = NativeSteeringBackend(profile(), 4, mode='direct_pwm', executable=result.path)
        self.addCleanup(backend.close)
        backend.step(inp)
        try:
            full = native()
        except unittest.SkipTest:
            return
        for data, out in ((v1, out1), (v2_direct, out2)):          # same step bytes as the full program
            code, full_out, _ = run(full.path, data)
            self.assertEqual(code, 0)
            self.assertEqual(full_out[ready_end(full_out):], out[ready_end(out):])


class RequireFirmwareTest(unittest.TestCase):
    """Review RF-05 (2026-11-09): a checkout whose .gitignore dropped the firmware sources skipped every firmware
    test without a failure. With UBTS_STEER_REQUIRE_FIRMWARE set (not '' or '0') missing sources fail the tests;
    without it they still skip with the reason. Both ways of missing: a UBTS_STEER_FIRMWARE_DIR without the sketch
    (an error) and no default firmware folder (absent)."""

    def outcome(self, require, how):
        saved = {name: os.environ.pop(name, None) for name in (native_build.ENV_FIRMWARE_DIR,
                                                                native_build.ENV_REQUIRE_FIRMWARE)}
        saved_dir = native_build.FIRMWARE_DIR
        temp = tempfile.mkdtemp(prefix='steer_fw_require_')
        try:
            if require is not None:
                os.environ[native_build.ENV_REQUIRE_FIRMWARE] = require
            if how == 'env_dir_without_sketch':
                os.environ[native_build.ENV_FIRMWARE_DIR] = temp
            else:
                native_build.FIRMWARE_DIR = Path(temp) / 'firmware' / 'ubts_steer_v3'     # not there
            try:
                firmware_dir_or_skip()
            except unittest.SkipTest as exc:
                return 'skip', str(exc)
            except AssertionError as exc:
                return 'fail', str(exc)
            return 'present', ''
        finally:
            native_build.FIRMWARE_DIR = saved_dir
            for name, value in saved.items():
                os.environ.pop(name, None)
                if value is not None:
                    os.environ[name] = value
            _cleanup_temp(temp)

    def test_missing_firmware_fails_only_when_required(self):
        for how in ('env_dir_without_sketch', 'default_folder_absent'):
            for require, expected in ((None, 'skip'), ('', 'skip'), ('0', 'skip'), ('1', 'fail')):
                with self.subTest(how=how, require=require):
                    kind, text = self.outcome(require, how)
                    self.assertEqual(kind, expected, text)
                    if expected == 'fail':
                        self.assertIn(native_build.ENV_REQUIRE_FIRMWARE, text)

    def test_present_firmware_passes_when_required(self):
        expected = firmware_dir_or_skip()     # a copy without the sources skips here (or fails when required)
        saved = os.environ.get(native_build.ENV_REQUIRE_FIRMWARE)
        os.environ[native_build.ENV_REQUIRE_FIRMWARE] = '1'
        try:
            self.assertTrue(native_build.firmware_required())
            self.assertEqual(firmware_dir_or_skip(), expected)
        finally:
            if saved is None:
                os.environ.pop(native_build.ENV_REQUIRE_FIRMWARE, None)
            else:
                os.environ[native_build.ENV_REQUIRE_FIRMWARE] = saved


if __name__ == '__main__':
    unittest.main()
