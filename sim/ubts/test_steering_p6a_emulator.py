"""P6a ESP emulator core with the real firmware (native lockstep v2 firmware mode), without ROS or Gazebo.

EmulatorCore runs on a synthetic 120 Hz /clock with an in-process MemoryPort (the pty is Linux only and has its own
test below): the tick bundling of 1/120 s steps (8 or 9 ticks, the accumulated count exact), gaps above 0.1 s, the
port open reset and the banner, host bytes with the UART arrival rule, ARM + C driving the outputs (steering angles,
drive velocity targets duty x omega0), the fault hooks (link loss, status loss, corruption, banner rewrite, restart,
MCU reset), the A3(G) record / replay, the bounded history of the firmware backend, and the A19(G) rule ticks of
p6a_fw_rules computed from a record (report_adc per session) against the firmware's own F lines and status bits.

The native program is built into UBTS_STEER_BUILD_DIR (or a temporary folder); without a compiler or the firmware
sources the tests skip (UBTS_STEER_REQUIRE_FIRMWARE turns the skip into a failure). No file in the work tree.
unittest style (pytest on 3.12 / 3.13, python3 -m unittest on 3.10). Placeholder-based SIM results only.
"""
import atexit
import json
import os
from pathlib import Path
import shutil
import stat
import sys
import tempfile
import unittest

from steering import gz_esp_emulator as emu
from steering import p6a_fw_rules as rules
from steering.actuator_link import crc8
from steering.native import build as native_build
from steering.native import lockstep_v2 as v2

STEP_NS = 8333333
T0_NS = 5_000_000_000
_STATE = {}


def native_path():
    if 'native' not in _STATE:
        try:
            if not native_build.firmware_present():
                _STATE['native'] = native_build.missing_firmware_message()
            else:
                native_build.find_compiler()
                folder = os.environ.get(native_build.ENV_BUILD_DIR)
                if not folder:
                    folder = tempfile.mkdtemp(prefix='steer_p6a_build_')
                    atexit.register(shutil.rmtree, folder, True)
                _STATE['native'] = native_build.ensure_built(build_dir=folder).path
        except native_build.NativeBuildError as exc:
            _STATE['native'] = 'native steering program not available: %s' % exc
    value = _STATE['native']
    if not os.path.isfile(value):
        if native_build.firmware_required():
            raise AssertionError('%s is set: %s' % (native_build.ENV_REQUIRE_FIRMWARE, value))
        raise unittest.SkipTest(value)
    return value


def host_line(text):
    data = text.encode('ascii')
    return data + b'*%02X\n' % crc8(data)


class CoreRig:
    def __init__(self, tmp, host_open=True, **kwargs):
        self.port = emu.MemoryPort(host_open=host_open)
        self.log_path = Path(tmp) / 'emulator_log.jsonl'
        self.log = emu.EventLog(self.log_path)
        self.core = emu.EmulatorCore(seed=7, port=self.port, log=self.log, executable=native_path(), **kwargs)
        self.t = T0_NS
        self.outputs = []

    def run(self, seconds, speed=0.):
        for _ in range(int(round(seconds * 1e9 / STEP_NS))):
            out = self.core.on_clock(self.t, speed)
            if out is not None:
                self.outputs.append(out)
            self.t += STEP_NS

    def lines(self):
        return self.port.host_read().decode('ascii', 'replace').splitlines()

    def arm(self):
        """ARM with the boot id of the last banner (the session must have booted)."""
        banner = self.core.last_banner
        boot = [f for f in banner.split(' ') if f.startswith('boot=')][0][5:]
        self.port.host_write(host_line('ARM ' + boot))
        self.run(.1)
        self.q = 0

    def drive(self, seconds, steer_permille, duty=0, speed=0.):
        """C lines every 50 ms (TTL 300 ms) with the steering target in permille."""
        for _ in range(int(round(seconds / .05))):
            self.q = getattr(self, 'q', 0) + 1
            self.port.host_write(host_line('C %d 300 %d %d %d' % (self.q, duty, duty, steer_permille)))
            self.run(.05, speed)

    def f_ticks(self, code):
        return [e['tick'] for e in self.entries('m2h') if e['text'].split(' ')[:2] == ['F', code]]

    def entries(self, kind=None):
        self.log.flush()
        rows = [json.loads(l) for l in self.log_path.read_text(encoding='utf-8').splitlines() if l]
        return [r for r in rows if kind is None or r['kind'] == kind]

    def close(self):
        self.core.close()


class EmulatorCoreTest(unittest.TestCase):
    def setUp(self):
        native_path()
        self.tmp = tempfile.mkdtemp(prefix='steer_p6a_emu_')
        self.rigs = []

    def tearDown(self):
        for rig in self.rigs:
            rig.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def rig(self, **kwargs):
        rig = CoreRig(self.tmp, **kwargs)
        self.rigs.append(rig)
        return rig

    def test_tick_bundling_is_exact(self):
        rig = self.rig()
        counts = []
        for k in range(360):              # 3 s of 120 Hz steps
            before = rig.core.now_tick()
            rig.core.on_clock(T0_NS + k * STEP_NS, 0.)
            counts.append(rig.core.now_tick() - before)
        self.assertEqual(set(counts[1:]), {8, 9})
        lead = rig.core.lead_steps * STEP_NS
        self.assertEqual(rig.core.now_tick(), round((359 * STEP_NS + lead) / 1e6))
        # a gap of 0.35 s: FW_STEPs of at most 0.1 s, the count still exact
        steps = rig.core.counters['fw_steps']
        rig.core.on_clock(T0_NS + 359 * STEP_NS + 350000000, 0.)
        self.assertGreaterEqual(rig.core.counters['fw_steps'] - steps, 4)
        self.assertEqual(rig.core.now_tick(), round((359 * STEP_NS + 350000000 + lead) / 1e6))
        self.assertEqual(rig.core.status()['max_clock_gap_ms'], 350.)

    def test_port_open_reset_banner_and_status(self):
        rig = self.rig(host_open=False)
        rig.run(.5)
        self.assertEqual(rig.lines(), [])
        self.assertGreater(rig.core.counters['m2h_lines_port_closed'], 0)
        rig.port.host_open = True
        rig.run(.5)
        lines = rig.lines()
        self.assertTrue(lines[0].startswith('UBTS_STEER_V3 hello ') and ' rst=ext_sys ' in lines[0], lines[:2])
        self.assertIn(' sim=1 ', lines[0])
        self.assertTrue(lines[1].startswith('UBTS_STEER_V3 cal id=1 '))
        self.assertTrue(any(l.startswith('S ') for l in lines))
        self.assertEqual(rig.core.counters['host_opens'], 1)

    def test_arm_and_drive_reach_the_outputs(self):
        rig = self.rig()
        rig.run(.5)
        banner = [l for l in rig.lines() if l.startswith('UBTS_STEER_V3 hello')][-1]
        boot = [f for f in banner.split(' ') if f.startswith('boot=')][0][5:]
        rig.port.host_write(host_line('ARM ' + boot))
        rig.run(.1)
        for k in range(20):
            rig.port.host_write(host_line('C %d 300 102 102 300' % (k + 1)))
            rig.run(.05, speed=.2)
        out = rig.outputs[-1]
        self.assertGreater(out['truth']['delta_eq_rad'], .02)
        self.assertEqual(set(out['steer']), {'left', 'right'})
        omega0 = rig.core.line.omega0_rad_s
        for value in out['drive'].values():
            self.assertAlmostEqual(value, out['truth']['drive_duty_left'] * omega0)
        self.assertGreater(out['truth']['drive_duty_left'], .3)
        h2m = [e for e in rig.entries('h2m') if e['text'].startswith('C 1 ')][0]
        self.assertEqual(h2m['first'], h2m['tick'] + 1)           # UART rule: the first byte one tick later
        self.assertGreater(h2m['last'], h2m['first'])
        # STOP: drive pins to 0 at the arrival tick
        rig.port.host_write(b'STOP\n')
        rig.run(.2)
        stop = [e for e in rig.entries('h2m') if e['text'] == 'STOP'][-1]
        zero = [e for e in rig.entries('pin') if e['channel'] == 'drive_l' and e['value'] == 0 and
                e['tick'] >= stop['last']]
        self.assertTrue(zero and zero[0]['tick'] == stop['last'])
        self.assertEqual(rig.outputs[-1]['truth']['mcu_state'], 1)

    def test_hooks(self):
        rig = self.rig()
        rig.run(.4)
        rig.lines()
        reply = rig.core.apply_hook(dict(op='status_loss', duration_ms=200))
        self.assertTrue(reply['ok'])
        rig.run(.3)
        self.assertGreater(rig.core.counters['status_lines_lost'], 0)
        rig.core.apply_hook(dict(op='link', direction='to_mcu', duration_ms=100))
        rig.port.host_write(b'PING\n')
        rig.run(.05)
        self.assertEqual(rig.core.counters['h2m_bytes_lost'], 5)
        self.assertFalse(rig.core.apply_hook(dict(op='nope'))['ok'])
        rig.core.apply_hook(dict(op='banner_rewrite', fields={'sim': '0'}))
        rig.core.apply_hook(dict(op='event', kind='mcu_reset', arg=4))
        rig.run(.3)
        banners = [l for l in rig.lines() if l.startswith('UBTS_STEER_V3 hello')]
        self.assertTrue(banners and ' sim=0 ' in banners[-1] and ' rst=soft_restart ' in banners[-1])
        body, crc = banners[-1].rsplit('*', 1)
        self.assertEqual(crc, '%02X' % crc8(body.encode('ascii')))
        session = rig.core.session
        reply = rig.core.apply_hook(dict(op='restart', reason='test', eeprom='blank'))
        self.assertEqual(reply['session'], session + 1)
        rig.run(.3)
        banner = [l for l in rig.lines() if l.startswith('UBTS_STEER_V3 hello')][-1]
        self.assertIn(' cal=none ', banner)
        rig.core.apply_hook(dict(op='corrupt', kind='crc', count=1))
        rig.port.host_write(b'PING\n')
        rig.run(.1)
        self.assertIn('N - crc', ' '.join(rig.lines()))
        rig.core.apply_hook(dict(op='adc_pattern', values=[500, 520], duration_ms=40))
        rig.run(.1)
        self.assertEqual(rig.core.adc_programs, [])

    def test_record_and_replay(self):
        rig = self.rig(report_adc=True)
        rig.run(.2)
        path = Path(self.tmp) / 'record.json'
        reply = rig.core.apply_hook(dict(op='record', seconds=1., path=str(path)))
        self.assertTrue(reply['ok'])
        rig.run(1.3)
        self.assertTrue(path.exists())
        record = json.loads(path.read_text(encoding='utf-8'))
        result = emu.replay_record(record, executable=native_path())
        self.assertTrue(result['passed'], result['first_mismatches'])
        self.assertGreater(result['adc_read_changes_compared'], 100)
        # a changed response is found
        record['frames'][3]['out']['pot_raw'] += 1
        self.assertFalse(emu.replay_record(record, executable=native_path())['passed'])

    def record_case(self, name, **session):
        """A rig whose next session is recorded with report_adc (record hook), booted and ready."""
        rig = self.rig()
        rig.run(.2)
        path = Path(self.tmp) / ('record_%s.json' % name)
        reply = rig.core.apply_hook(dict(op='record', seconds=30., path=str(path), report_adc=True, **session))
        self.assertTrue(reply['ok'], reply)
        self.assertTrue(reply['session_info']['report_adc'])
        rig.run(.6)
        rig.lines()
        return rig, path

    def trace_of(self, rig, path, at_tick):
        reply = rig.core.apply_hook(dict(op='record_finish'))
        self.assertTrue(reply['ok'] and reply['finished'], reply)
        record = json.loads(path.read_text(encoding='utf-8'))
        return rules.FirmwareTrace(record, at_tick=at_tick)

    def test_record_hook_report_adc_and_finish(self):
        rig, path = self.record_case('adc')
        self.assertFalse(rig.core.report_adc)                    # the emulator default stays off
        rig.run(.3)
        trace = self.trace_of(rig, path, None)
        self.assertGreater(len(trace.raw), 300)
        self.assertTrue(400 <= trace.raw[-1] <= 650)
        self.assertEqual(trace.setup_tick, trace.boots[0])
        self.assertFalse(rig.core.apply_hook(dict(op='record_finish'))['finished'])
        with self.assertRaises(rules.TraceError):                # a record without analogRead values
            rules.FirmwareTrace(dict(session=dict(firmware=dict(report_adc=False)), frames=[]))

    def test_rule_tick_pot_noise_and_softlimit_disarmed(self):
        rig, path = self.record_case('noise')
        raw = rig.core.last_out.base.pot_raw
        start = rig.core.apply_hook(dict(op='adc_pattern', values=[raw - 6, raw + 6], duration_ms=300))['start_tick']
        rig.run(.6)
        f = rig.f_ticks('POT_NOISE')
        trace = self.trace_of(rig, path, start)
        rule = trace.first_trip('POT_NOISE', start)
        self.assertTrue(f and rule is not None, (f, rule))
        self.assertEqual(f[0], rule)
        # SOFTLIMIT: a slow ramp past the left soft limit (condition bit in the status lines, no latch)
        rig, path = self.record_case('soft')
        cal = rig.core.session_info['calibration']
        target = int(round(cal['raw_c'] + (cal['raw_l'] - cal['raw_c']) * (cal['soft_l'] + 30) / 1000.))
        start = rig.core.apply_hook(dict(op='adc_ramp', to=target, counts_per_tick=.2))['start_tick']
        rig.run(2.5)
        trace = self.trace_of(rig, path, start)
        rule = trace.first_change('SOFTLIMIT', start)
        lines = [e for e in rig.entries('m2h') if e['text'].startswith('S ') and e['tick'] >= start]
        first_bit = [e['tick'] for e in lines if int(e['text'].split(' ')[13], 16) & 0x0080]
        due = [e['tick'] for e in lines if e['tick'] >= rule]
        self.assertTrue(first_bit and due)
        self.assertEqual(first_bit[0], due[0])

    def test_rule_tick_polarity_drv_stuck_stall_thermal_armed(self):
        # POLARITY: the model's motor turns the wrong way after the injection
        rig, path = self.record_case('pol', injections=[dict(kind='polarity_reversed', at_s=.5, params={})])
        rig.arm()
        rig.drive(.5, 0)
        start = rig.core.now_tick()
        rig.drive(1.5, 250)
        f = rig.f_ticks('POLARITY')
        trace = self.trace_of(rig, path, start)
        self.assertTrue(f, 'no POLARITY line')
        self.assertLessEqual(abs(f[0] - trace.first_trip('POLARITY', start)), 1)
        # DRV_STUCK: the position drifts while the servo brakes in its deadband (scripted ADC, 1 count per 9 ticks)
        rig, path = self.record_case('stuck')
        rig.arm()
        rig.drive(1., 0)
        raw = rig.core.last_out.base.pot_raw
        start = rig.core.apply_hook(dict(op='adc_ramp', to=raw + 40, counts_per_tick=1. / 9.))['start_tick']
        rig.drive(2.5, 0)
        f = rig.f_ticks('DRV_STUCK')
        trace = self.trace_of(rig, path, start)
        self.assertTrue(f, 'no DRV_STUCK line')
        self.assertLessEqual(abs(f[0] - trace.first_trip('DRV_STUCK', start)), 1)
        # STALL: motor power off, ADC frozen, a steering step
        rig, path = self.record_case('stall')
        rig.arm()
        rig.drive(.5, 0)
        rig.core.apply_hook(dict(op='event', kind='motor_power', arg=0))
        rig.core.apply_hook(dict(op='event', kind='adc_set', arg=rig.core.last_out.base.pot_raw))
        start = rig.core.now_tick()
        rig.drive(1.5, 350)
        f = rig.f_ticks('STALL')
        trace = self.trace_of(rig, path, start)
        self.assertTrue(f, 'no STALL line')
        self.assertLessEqual(abs(f[0] - trace.first_trip('STALL', start)), 1)
        # THERMAL: CFG tha 50, thw 1; a large step saturates the servo output
        rig, path = self.record_case('thermal', record_overrides=dict(tha=50, thw=1))
        rig.arm()
        rig.drive(.5, 0)
        start = rig.core.now_tick()
        rig.drive(1., 380)
        f = rig.f_ticks('THERMAL')
        trace = self.trace_of(rig, path, start)
        self.assertTrue(f, 'no THERMAL line')
        self.assertLessEqual(abs(f[0] - trace.first_trip('THERMAL', start)), 1)

    def test_history_stays_bounded(self):
        rig = self.rig()
        rig.run(3.)
        backend = rig.core.backend
        self.assertEqual(backend.tx_bytes, [])
        self.assertEqual(backend.changes, [])
        self.assertEqual(backend.channel_values['mcu_state'], 1)


@unittest.skipUnless(sys.platform.startswith('linux'), 'pty needs Linux')
class PtyPortTest(unittest.TestCase):
    def test_link_open_close_and_unplug(self):
        tmp = tempfile.mkdtemp(prefix='steer_p6a_pty_')
        try:
            link = os.path.join(tmp, 'esp-tty')
            port = emu.PtyPort(link, allowed_root=tmp)
            port.create()
            info = os.stat(link)
            self.assertTrue(stat.S_ISCHR(info.st_mode) and 136 <= os.major(info.st_rdev) <= 143)
            self.assertFalse(port.poll_host_open())
            fd = os.open(link, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            try:
                self.assertTrue(port.poll_host_open())
                self.assertTrue(port.write(b'S 1\n'))
                import select
                select.select([fd], [], [], 1.)
                self.assertEqual(os.read(fd, 64), b'S 1\n')       # raw: no echo, no CR
                os.write(fd, b'PING\n')
                select.select([port.master], [], [], 1.)
                self.assertEqual(port.read(), b'PING\n')
            finally:
                os.close(fd)
            self.assertFalse(port.poll_host_open())
            port.unplug()
            self.assertFalse(os.path.lexists(link))
            port.replug()
            self.assertTrue(os.path.lexists(link))
            port.close()
            with self.assertRaises(emu.EmulatorError):
                emu.PtyPort('/tmp/elsewhere/esp-tty', allowed_root=tmp)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    unittest.main()
