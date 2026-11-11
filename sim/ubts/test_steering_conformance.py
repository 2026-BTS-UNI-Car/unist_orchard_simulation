"""Conformance scripts and runner for the ESP protocol V3 steering firmware (design 8.3 P3, A19 MCU part, A20).

Always: every script under steering/conformance/scripts validates against ubts.steer_conformance.v1
(docs/steering/ESP_PROTOCOL_V3_DRAFT_KO.md section 13), the CRC-8 / CRC-32 / record / boot id code matches the
specification's vectors, the coverage lists (fault codes, reject reasons, V2 scenarios) are complete, and the
matcher logic is checked against a small scripted fake backend defined here.

With the native firmware build: every script runs through the lockstep v2 firmware mode. The test is skipped
with the reason while the firmware mode is unavailable (no compiler, build failure, or a native program that
refuses the v2 firmware startup line). Builds go to UBTS_STEER_BUILD_DIR or a temporary folder, never into the
work tree. unittest style, so it also runs with `python3 -m unittest` (Python 3.10 has no pytest here).
All values are placeholders: passing is a placeholder-based SIM result, not field validation.
"""
import copy
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import unittest

from steering.conformance import run_conformance as rc
from steering.native import build as native_build

R7 = dict(cal_id=7, raw_l=842, raw_c=512, raw_r=181, soft_l=935, soft_r=-935, pol_ok=1, mdir=1, params={})
BANNER = ('UBTS_STEER_V3 hello proto=3 build=0A1B2C3D dev=5A1B2C boot=89025CC1 rst=power_on hw=front_steer_v0 '
          'out=drive2,steer1 cal=7 sim=1 wd=500 ttl=300 st=20 drop=0')
CAL = ('UBTS_STEER_V3 cal id=7 crc=61AEB1A4 left=842 center=512 right=181 soft=935,-935 pol=ok mdir=1 caldirty=0 '
       'cfgdirty=0')
A19_CODES = ('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'SOFTLIMIT', 'STALL',
             'TRACK', 'THERMAL', 'TTL', 'WATCHDOG', 'CRC', 'BOOT')
FAULT_BIT = {name: 1 << i for i, name in enumerate(('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE',
                                                    'POLARITY', 'DRV_STUCK', 'SOFTLIMIT', 'STALL', 'TRACK', 'THERMAL',
                                                    'TTL', 'WATCHDOG', 'CRC', 'BOOT'))}
V2_SCENARIOS = ('boot_order', 'mapping_and_full_range', 'reversal_interlock', 'pending_stop', 'watchdog_active',
                'watchdog_pending', 'late_ping_no_resurrection', 'timely_ping', 'locked_power_and_d4', 'strict_parser',
                'nul_rejected', 'crlf_and_embedded_cr', 'overflow_and_recovery', 'bounded_serial_work',
                'info_does_not_renew_watchdog', 'millis_rollover', 'gradual_acceleration_and_soft_release',
                'latest_request_wins', 'stall_catchup_capped', 'emergency_stop_immediate', 'bounded_actual_reports',
                'repeated_zero_ack_without_gpio_write')


def mcu(body):
    return '%s*%s' % (body, rc.crc8_hex(body))


def base_script(**changes):
    data = dict(schema=rc.SCHEMA, name='fake', spec=['13'],
                setup=dict(boot_seed=1, chip_id='5A1B2C', eeprom={'record': dict(R7)}),
                inputs=[], expect=[], end=dict(tick=200))
    data.update(changes)
    return data


def make(data):
    return rc.Script(copy.deepcopy(data), data['name'])


class FakeTransport(rc.FirmwareTransport):
    """Scripted MCU output: lines queued at given ticks (UART timing of lockstep 11.7) and channel changes.

    `echo` maps a received line to the line the fake answers in the same tick (a tiny stand-in for a parser).
    `skew` adds that many ticks to every queued tick when the step size equals skew_step (partition tests).
    """

    def __init__(self, lines=(), changes=(), echo=None, garbage=(), skew=0, skew_step=None, bad_crc_at=None):
        self.ready = dict(build_id='fake fw=0A1B2C3D', fw_build='0A1B2C3D', executable='fake', executable_sha256=None)
        self.lines = {}
        for tick, text in lines:
            self.lines.setdefault(tick, []).append(text)
        self.changes = sorted(changes)
        self.echo = dict(echo or {})
        self.garbage = list(garbage)
        self.skew, self.skew_step, self.bad_crc_at = skew, skew_step, bad_crc_at
        self.reset(0.)

    def reset(self, delta_eq_rad):
        self.ticks_total = 0
        self.uart_end = 0
        self.pending = []
        self.rx_line = bytearray()
        self.values = dict(rc.CHANNEL_INITIAL)
        return {}

    def _queue(self, tick, data, flags=0):
        for byte in data:
            self.uart_end = max(tick * 576, self.uart_end) + 50
            done = (self.uart_end + 575) // 576 - 1
            self.pending.append((tick, done, byte, flags))

    def step(self, first_tick, ticks, speed, supply, stopped, events, rx):
        changes, skew = [], self.skew if ticks == self.skew_step else 0
        for tick in range(first_tick, first_tick + ticks):
            if tick == 0 and self.garbage:
                self._queue(0, bytes(self.garbage), flags=1)
            for offset, byte in rx:
                if first_tick + offset != tick:
                    continue
                if byte == 0x0A:
                    reply = self.echo.get(self.rx_line.decode('latin-1'))
                    if reply is not None:
                        self._queue(tick, (reply + '\n').encode('ascii'))
                    self.rx_line = bytearray()
                else:
                    self.rx_line.append(byte)
            for text in self.lines.get(tick - skew, []):
                data = (text + '\n').encode('ascii')
                if self.bad_crc_at == tick:
                    data = data[:-3] + b'00\n'
                self._queue(tick, data)
            for t, name, value in self.changes:
                if t == tick and self.values[name] != value:
                    self.values[name] = value
                    changes.append((tick, name, value))
        end = first_tick + ticks
        tx = [item for item in self.pending if first_tick <= item[1] < end]
        self.pending = [item for item in self.pending if item[1] >= end]
        self.ticks_total = end
        base = {key: 0 for key in rc.BASE_FIELDS}
        base['ticks'] = ticks
        return rc.FwStep(first_tick, ticks, base, changes, tx, 1, 3, 1, end - 1, 0)


BOOT_CHANGES = [(0, 'mcu_state', 1), (0, 'drive_en', 1), (0, 'fault_bits', 0x4000), (0, 'gpio_writes', 14)]


def run_fake(data, step_ticks=10, check_step_ticks=7, **fake):
    script = make(data)
    fake.setdefault('lines', [(0, mcu(BANNER)), (0, mcu(CAL))])
    fake.setdefault('changes', BOOT_CHANGES)
    return rc.run_script(script, lambda: FakeTransport(**fake), step_ticks, check_step_ticks)


def by_id(result):
    return {item['id']: item for item in result['expectations'] + result['checks']}


BOOT_EXPECT = [
    dict(type='line', tick=0, text='UBTS_STEER_V3 hello proto=3 build={any} dev=5A1B2C boot={boot} rst=power_on '
                                   'hw=front_steer_v0 out=drive2,steer1 cal=7 sim=1 wd=500 ttl=300 st=20 drop=0*{crc}'),
    dict(type='line', tick=0, text=CAL + '*{crc}')]


# --- specification vectors ---------------------------------------------------------------------------------------------
class SpecificationVectors(unittest.TestCase):
    def test_crc8_vectors_and_table(self):
        for covered, expected in rc.CRC8_VECTORS:
            self.assertEqual(rc.crc8_hex(covered), expected, covered)
        self.assertEqual(rc.CRC8_TABLE[:16], rc.CRC8_TABLE_FIRST16)
        # Design 4.1 example values are wrong (spec 3); the corrected ones:
        self.assertEqual(rc.crc8_hex('ARM 5f3a19c2'), 'A7')
        self.assertEqual(rc.crc8_hex('DISARM'), '88')
        self.assertEqual(rc.crc8_hex('C 1842 250 102 102 -310'), '8E')
        self.assertEqual(rc.crc8_hex('S 123456 2210 2 1842 102 102 3 611 -310 -323 -318 -64 0000 40 260'), 'CB')

    def test_spec_example_lines_carry_their_crc(self):
        for line in ('S 300 5 2 1 48 48 3 512 0 0 0 0 0000 280 20*AA', 'F TTL 1 320*6B', 'W 521*78', 'N - long*3C',
                     'N - crc*D8', 'A CFG kp 245*58', 'P D8 0 0*C6', 'P D1 255 255*EB', BANNER + '*01', CAL + '*D6',
                     'UBTS_STEER_V3 cal id=none crc=00000000 left=- center=- right=- soft=0,0 pol=no mdir=1 '
                     'caldirty=0 cfgdirty=0*3F'):
            body, given = line.rsplit('*', 1)
            self.assertEqual(rc.crc8_hex(body), given, line)

    def test_self_check(self):
        self.assertTrue(rc.self_check())

    def test_crc32_record_and_tombstone(self):
        image = rc.eeprom_image({'record': dict(R7)})
        self.assertEqual(len(image), 256)
        self.assertEqual(image[:106].hex().upper(), rc.EXAMPLE_RECORD_HEX)
        self.assertEqual(image[106:], b'\xff' * 150)
        self.assertEqual(rc.crc32_iso_hdlc(b'123456789'), 0xCBF43926)
        tomb = rc.record_bytes(dict(R7, magic='USCX'))
        self.assertEqual(tomb[:4], b'USCX')
        self.assertEqual(int.from_bytes(tomb[102:], 'little'), 0x4F1B5580)
        self.assertEqual(rc.eeprom_image('blank'), b'\xff' * 256)

    def test_boot_ids(self):
        for seed, ids in rc.BOOT_ID_VECTORS:
            self.assertEqual(rc.boot_ids(seed, 3), list(ids))

    def test_crc_macro_covers_its_own_line(self):
        script = make(base_script())
        data = rc.expand_input_text('PING\nCFG kp*{crc}\nARM {boot}*{crc}\n', 10, script.plan, 'x')
        self.assertEqual(data, ('PING\nCFG kp*%s\nARM 89025CC1*%s\n' % (rc.crc8_hex('CFG kp'),
                                                                        rc.crc8_hex('ARM 89025CC1'))).encode())

    def test_boot_macro_follows_resets(self):
        data = base_script(events=[dict(tick=100, kind='mcu_reset', reason=4),
                                   dict(tick=150, kind='power_cycle', off_ticks=20)],
                           end=dict(tick=400))
        data['setup']['boot_delay_ticks'] = 5
        plan = make(data).plan
        self.assertEqual(plan.alive(0), None)
        self.assertEqual(plan.alive(5), '89025CC1')
        self.assertEqual(plan.alive(104), None)
        self.assertEqual(plan.alive(105), '658EEC67')
        self.assertEqual(plan.alive(160), None)          # power off 150..169, power-on reset 170, setup 175
        self.assertEqual(plan.alive(175), 'FB32555E')
        self.assertEqual(plan.cuts, [100, 150, 170])


# --- script format -------------------------------------------------------------------------------------------------------
class ScriptFiles(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.paths = rc.script_paths()
        cls.scripts = [rc.load_script(path) for path in cls.paths]

    def test_there_are_scripts(self):
        self.assertGreaterEqual(len(self.paths), 60)

    def test_files_are_utf8_lf_without_bom_and_forbidden_words(self):
        pattern = re.compile(br'(?i)cl' br'aude|anth' br'ropic|cod' br'ex|chat' br'gpt')
        for path in self.paths:
            data = path.read_bytes()
            self.assertFalse(data.startswith(b'\xef\xbb\xbf'), path.name)
            self.assertNotIn(b'\r', data, path.name)
            data.decode('utf-8')
            self.assertIsNone(pattern.search(data), path.name)
            self.assertTrue(rc.NAME_RE.match(path.stem), path.name)

    def test_every_script_has_spec_refs_and_description(self):
        for script in self.scripts:
            self.assertTrue(script.spec, script.name)
            self.assertTrue(script.description, script.name)

    def test_v2_scenarios_are_all_mapped(self):
        mapped = {script.v2_map for script in self.scripts if script.v2_map}
        self.assertEqual(set(V2_SCENARIOS) - mapped, set())
        self.assertEqual(mapped - set(V2_SCENARIOS), set())
        names = {script.name for script in self.scripts}
        self.assertEqual(set(V2_SCENARIOS) - names, set(), 'a script named after every V2 scenario')

    def test_every_reject_reason_is_expected_somewhere(self):
        seen = set()
        for script in self.scripts:
            for item in script.expect + script.checks:
                if item['type'] == 'line' and item['text'].startswith('N '):
                    seen.add(item['text'].split(' ')[2].split('*')[0])
        self.assertEqual(set(rc.N_REASONS) - seen, set())

    def test_every_a19_fault_code_is_raised_somewhere(self):
        lines, bits = set(), 0
        for script in self.scripts:
            for item in script.expect + script.checks:
                if item['type'] == 'line' and item['text'][:2] in ('F ', 'W '):
                    lines.add('WATCHDOG' if item['text'].startswith('W ') else item['text'].split(' ')[1])
                if item['type'] == 'change' and item['channel'] == 'fault_bits':
                    comp = item['to']
                    bits |= comp.a if comp.kind == 'hex' else (comp.b if comp.kind == 'mask' else 0)
                if item['type'] == 'status' and 'fl' in item['fields']:
                    comp = item['fields']['fl']
                    bits |= comp.a if comp.kind == 'hex' else (comp.b if comp.kind == 'mask' else 0)
        missing = [code for code in A19_CODES if code not in lines and not bits & FAULT_BIT[code]]
        self.assertEqual(missing, [])
        for code in ('POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'STALL', 'TRACK',
                     'THERMAL', 'TTL', 'CRC', 'NOCAL', 'WATCHDOG'):
            self.assertIn(code, lines, 'an F (or W) line for %s' % code)

    def test_event_kinds_and_commands_used(self):
        kinds, keywords = set(), set()
        for script in self.scripts:
            kinds.update(ev['kind'] for ev in script.events)
            for item in script.inputs:
                for raw in item['data'].split(b'\n'):
                    token = raw.split(b' ')[0].split(b'*')[0]
                    if token:
                        keywords.add(token.decode('latin-1'))
        self.assertGreaterEqual(kinds, {'mcu_reset', 'power_cycle', 'port_open_reset', 'mcu_stall', 'adc_set'})
        self.assertGreaterEqual(keywords, {'INFO', 'PING', 'STOP', 'ARM', 'DISARM', 'C', 'CAL', 'CFG', 'BENCH', 'D8',
                                           'D5', 'D3', 'D0', 'D1'})

    def test_list_and_validate_cli(self):
        self.assertEqual(rc.main(['--validate-only']), rc.EXIT_PASS)
        self.assertEqual(rc.main(['--list', 'boot_*']), rc.EXIT_PASS)
        self.assertEqual(rc.main(['--validate-only', 'no_such_script']), rc.EXIT_INVALID)


class FormatRejection(unittest.TestCase):
    def bad(self, data, fragment):
        with self.assertRaises(rc.ScriptError) as ctx:
            make(data)
        self.assertIn(fragment, str(ctx.exception))

    def test_unknown_and_missing_keys(self):
        self.bad(base_script(extra=1), 'unknown key')
        data = base_script()
        del data['end']
        self.bad(data, 'missing key')
        data = base_script()
        data['setup']['colour'] = 'red'
        self.bad(data, 'unknown key')

    def test_name_schema_and_setup(self):
        self.bad(base_script(schema='x'), 'schema')
        data = base_script()
        data['setup']['mcu_tick_hz'] = 2000
        self.bad(data, 'only 1000')
        data = base_script()
        data['setup']['chip_id'] = '5a1b2c'
        self.bad(data, 'upper-case')
        data = base_script()
        data['setup']['eeprom'] = {'hex': 'FF' * 10}
        self.bad(data, '512')

    def test_tolerance_rules(self):
        self.bad(base_script(expect=[dict(type='line', tick=5, tol=1, text='N - crc*{crc}')]), 'tol 1 only for F')
        self.bad(base_script(expect=[dict(type='change', channel='drive_l', tick=5, tol=1, to=0)]), 'fault_bits')
        self.bad(base_script(expect=[dict(type='line', tick=5, tol=2, text='F TTL 1 5*{crc}')]), '[0, 1]')
        self.bad(base_script(expect=[dict(type='pins', tick=5, tol=1, drive_l=0)]), 'tol')
        make(base_script(expect=[dict(type='line', tick=5, tol=1, text='F TTL 1 5*{crc}'),
                                 dict(type='change', channel='fault_bits', tick=5, tol=1, to='0800')]))

    def test_macros_and_inputs(self):
        self.bad(base_script(inputs=[dict(tick=1, text='PING{crc}\n')]), 'follow')
        self.bad(base_script(inputs=[dict(tick=1, text='PING {x}\n')]), 'macro')
        self.bad(base_script(inputs=[dict(tick=5, text='A\n'), dict(tick=1, text='B\n')]), 'tick order')
        self.bad(base_script(inputs=[dict(tick=1, text='é\n')]), 'ASCII')
        self.bad(base_script(inputs=[dict(tick=1, hex='0A0')]), 'hex')
        self.bad(base_script(inputs=[dict(tick=300, text='PING\n')]), '[0, 200]')
        self.bad(base_script(expect=[dict(type='line', tick=1, text='N {crc}')]), 'follow')

    def test_comparators(self):
        self.bad(base_script(expect=[dict(type='status', tick=50, fields=dict(fl='800'))]), 'hex')
        self.bad(base_script(expect=[dict(type='status', tick=50, fields=dict(L='pos'))]), 'integer')
        self.bad(base_script(expect=[dict(type='pins', tick=5, fault_bits={'mask': '0001', 'value': '0002'})]),
                 'outside the mask')
        self.bad(base_script(expect=[dict(type='status', tick=50, fields=dict(zz=1))]), 'unknown key')
        make(base_script(expect=[dict(type='pins', tick=5, drive_l='pos', steer_pwm={'min': -3}),
                                 dict(type='status', tick=50, fields=dict(L={'min': 0, 'max': 3}, fl='4000'))]))

    def test_events_and_plant(self):
        self.bad(base_script(events=[dict(tick=1, kind='mcu_reset', reason=9)]), '[1, 6]')
        self.bad(base_script(events=[dict(tick=1, kind='explode')]), 'kind')
        self.bad(base_script(plant=dict(speed_m_s=[[5, 0.1]])), 'tick 0')
        self.bad(base_script(plant=dict(supply_scale=[[0, 3.0]])), '(0, 2]')
        self.bad(base_script(events=[dict(tick=10, kind='power_cycle', off_ticks=50),
                                     dict(tick=20, kind='mcu_reset', reason=1)]), 'power-off')


# --- matcher logic against the fake ---------------------------------------------------------------------------------------
class MatcherLogic(unittest.TestCase):
    def test_passing_script(self):
        data = base_script(
            inputs=[dict(tick=20, text='CFG kp*{crc}\n')],
            expect=BOOT_EXPECT + [
                dict(type='line', tick=20, text='A CFG kp 245*{crc}', id='reply'),
                dict(type='line', tick=25, at='done', text='A CFG kp 245*{crc}'),  # UART busy with boot lines
                dict(type='pins', tick=0, mcu_state=1, drive_en=1, fault_bits='4000', gpio_writes=14),
                dict(type='pins_window', window=[0, 200], drive_l='zero', steer_en=0, fault_bits={
                    'mask': '4000', 'value': '4000'}),
                dict(type='change', channel='mcu_state', tick=0, to=1),
                dict(type='first_change', channel='drive_en', after=0, tick=0, to=1),
                dict(type='no_line', window=[1, 200], kinds=['N', 'F']),
                dict(type='count', window=[0, 0], kinds=['BANNER', 'CAL'], min=2, max=2)],
            end=dict(tick=200))
        result = run_fake(data, echo={'CFG kp*' + rc.crc8_hex('CFG kp'): mcu('A CFG kp 245')})
        self.assertTrue(result['pass'], rc.format_result(result, verbose=True))
        self.assertEqual(result['partition']['same'], True)
        self.assertEqual(result['unexpected_lines'], [])

    def test_line_tick_and_pattern_failures(self):
        data = base_script(expect=BOOT_EXPECT + [
            dict(type='line', tick=1, text=CAL + '*{crc}', id='late'),
            dict(type='line', tick=0, text=CAL.replace('id=7', 'id=8') + '*{crc}', id='wrong')])
        result = run_fake(data)
        found = by_id(result)
        self.assertFalse(found['late']['pass'])
        self.assertIn('same pattern at 0', found['late']['detail'])
        self.assertFalse(found['wrong']['pass'])
        self.assertFalse(result['pass'])

    def test_boot_macro_must_be_the_running_boot(self):
        wrong = BANNER.replace('89025CC1', '658EEC67')
        data = base_script(expect=[dict(BOOT_EXPECT[0], id='banner'), BOOT_EXPECT[1]],
                           end=dict(tick=50, unexpected='ignore'))
        result = run_fake(data, lines=[(0, mcu(wrong)), (0, mcu(CAL))])
        self.assertFalse(by_id(result)['banner']['pass'])

    def test_unexpected_lines_and_ignore_kinds(self):
        lines = [(0, mcu(BANNER)), (0, mcu(CAL)), (50, mcu('S 50 0 1 0 0 0 1 512 0 0 0 0 4000 50 0')),
                 (60, mcu('N - cmd'))]
        result = run_fake(base_script(expect=list(BOOT_EXPECT)), lines=lines)
        self.assertFalse(result['pass'])
        self.assertEqual([item['text'] for item in result['unexpected_lines']], [mcu('N - cmd')])
        data = base_script(expect=list(BOOT_EXPECT), end=dict(tick=200, ignore_kinds=['S', 'N']))
        self.assertTrue(run_fake(data, lines=lines)['pass'])
        data = base_script(expect=list(BOOT_EXPECT), end=dict(tick=200, unexpected='ignore'))
        self.assertTrue(run_fake(data, lines=lines)['pass'])
        data = base_script(expect=list(BOOT_EXPECT), end=dict(tick=200, checks=[
            dict(type='count', kinds=['N'], min=1, max=1)]))
        self.assertTrue(run_fake(data, lines=lines)['pass'])

    def test_every_mcu_line_crc_is_checked(self):
        lines = [(0, mcu(BANNER)), (0, mcu(CAL)), (60, mcu('N - cmd'))]
        data = base_script(expect=BOOT_EXPECT + [dict(type='line', tick=60, text='N - cmd*{any}')])
        result = run_fake(data, lines=lines, bad_crc_at=60)
        self.assertFalse(result['pass'])
        self.assertIn('CRC-8', result['format_failures'][0]['problems'][0])

    def test_line_formats(self):
        def problems(text):
            return rc.check_line_format(rc.Line(text, 0, 0, 0))
        self.assertEqual(problems(mcu('S 300 5 2 1 48 48 3 512 0 0 0 0 0000 280 20')), [])
        self.assertTrue(problems(mcu('S 300 5 2 1 48 48 3 512 0 0 0 0 000 280 20')))
        self.assertTrue(problems(mcu('S 300 05 2 1 48 48 3 512 0 0 0 0 0000 280 20')))
        self.assertTrue(problems(mcu('N 5 nonsense')))
        self.assertEqual(problems(mcu('N 65535 old')), [])
        self.assertEqual(problems(mcu('F POT_NOISE 4515 250')), [])
        self.assertTrue(problems(mcu('F SOFTLIMIT 1 2')))
        self.assertEqual(problems(mcu('A POL ok 6 -6')), [])
        self.assertEqual(problems(mcu('P D1 255 255')), [])
        self.assertTrue(problems(mcu('P D2 0 0')))
        self.assertTrue(problems(mcu('DEBUG hello')))
        self.assertTrue(problems('S 1 2*ab'))
        self.assertTrue(problems(mcu('W 5') + '\r'))
        self.assertTrue(problems(mcu('N - ' + 'x' * 20)))

    def test_status_fields(self):
        status = mcu('S 50 0 1 0 0 0 1 512 0 0 0 0 4000 50 0')
        data = base_script(expect=BOOT_EXPECT + [
            dict(type='status', tick=50, fields=dict(ms=50, st=1, fl='4000', wd={'min': 40, 'max': 60}), id='ok'),
            dict(type='status', tick=50, fields=dict(fl={'mask': '4000', 'value': '0000'}), id='mask'),
            dict(type='status', tick=100, fields=dict(st=1), id='missing')])
        result = run_fake(data, lines=[(0, mcu(BANNER)), (0, mcu(CAL)), (50, status)])
        found = by_id(result)
        self.assertTrue(found['ok']['pass'])
        self.assertFalse(found['mask']['pass'])
        self.assertIn('fl=4000', found['mask']['detail'])
        self.assertFalse(found['missing']['pass'])

    def test_change_semantics(self):
        changes = BOOT_CHANGES + [(30, 'drive_l', 2), (40, 'drive_l', 5), (50, 'fault_bits', 0x4800),
                                  (60, 'fault_bits', 0x4801)]
        data = base_script(expect=BOOT_EXPECT + [
            dict(type='change', channel='drive_l', tick=30, to=2, id='exact'),
            dict(type='change', channel='drive_l', tick=40, to='pos', id='still_pos'),
            dict(type='change', channel='drive_l', tick=40, to='any', id='any'),
            dict(type='change', channel='fault_bits', tick=50, to={'mask': '0800', 'value': '0800'}, id='ttl_on'),
            dict(type='change', channel='fault_bits', tick=60, to={'mask': '0800', 'value': '0800'}, id='ttl_again'),
            dict(type='change', channel='fault_bits', tick=59, tol=1, to='4801', id='tol'),
            dict(type='first_change', channel='drive_l', after=31, tick=40, to=5, id='first'),
            dict(type='first_change', channel='drive_l', after=30, tick=40, to=5, id='inclusive'),
            dict(type='pins', tick=35, drive_l=2, id='pins'),
            dict(type='pins_window', window=[30, 45], drive_l='pos', id='window'),
            dict(type='pins_window', window=[30, 45], drive_l=2, id='window_bad')])
        result = run_fake(data, changes=changes)
        found = by_id(result)
        for key, want in dict(exact=True, still_pos=False, any=True, ttl_on=True, ttl_again=False, tol=True,
                              first=True, inclusive=False, pins=True, window=True, window_bad=False).items():
            self.assertEqual(found[key]['pass'], want, '%s: %s' % (key, found[key]['detail']))

    def test_partition_difference_is_reported(self):
        lines = [(0, mcu(BANNER)), (0, mcu(CAL)), (60, mcu('N - cmd'))]
        data = base_script(expect=BOOT_EXPECT + [dict(type='line', window=[55, 70], text='N - cmd*{crc}')])
        result = run_fake(data, lines=lines, skew=3, skew_step=7)
        self.assertFalse(result['pass'])
        self.assertFalse(result['partition']['same'])
        self.assertIn('step partition', result['partition']['difference'])
        self.assertTrue(run_fake(data, lines=lines, check_step_ticks=0)['pass'])

    def test_garbage_bytes_are_not_lines_and_incomplete_tail_fails(self):
        result = run_fake(base_script(expect=list(BOOT_EXPECT)), garbage=b'\x00\xffjunk\n')
        self.assertTrue(result['pass'], rc.format_result(result))
        self.assertEqual(result['boot_garbage_bytes'], 7)
        data = base_script(expect=list(BOOT_EXPECT), end=dict(tick=200, drain_ticks=0))
        status = mcu('S 200 3 1 0 0 0 1 512 0 0 0 0 4000 200 0')     # 44 B: still sending after tick 200
        result = run_fake(data, lines=[(0, mcu(BANNER)), (0, mcu(CAL)), (200, status)])
        self.assertFalse(result['pass'])
        self.assertTrue(result['incomplete_lines'])

    def test_fragment_cut_by_a_reset(self):
        tx = [(0, 0, ord(ch), 0) for ch in 'S 1'] + [(5, 5, ord(ch), 0) for ch in mcu('N - cmd') + '\n']
        lines, fragments, _, tail = rc.assemble_lines(tx, [5])
        self.assertEqual([line.text for line in lines], [mcu('N - cmd')])
        self.assertEqual(fragments, [dict(queued=0, text='S 1')])
        self.assertIsNone(tail)

    def test_uart_input_schedule(self):
        data = base_script(inputs=[dict(tick=0, text='C 65535 1000 -255 -255 -1000*{crc}\n', pace='uart')])
        rx = rc.schedule_inputs(make(data))
        self.assertEqual([len(rx[t]) for t in sorted(rx)], [11, 12, 9])
        self.assertEqual(sorted(rx), [1, 2, 3])


# --- the real firmware build ---------------------------------------------------------------------------------------------------
class RealFirmware(unittest.TestCase):
    """Every script against the native lockstep program in firmware mode (skipped while it is not available)."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = None
        build_dir = os.environ.get('UBTS_STEER_BUILD_DIR')
        if not build_dir:
            cls.tmp = tempfile.mkdtemp(prefix='ubts_conformance_build_')
            build_dir = cls.tmp
        try:
            cls.executable = rc.resolve_executable(build_dir=build_dir)
            cls.available, cls.reason = rc.probe_firmware_mode(cls.executable)
        except rc.TransportError as exc:
            cls.executable, cls.available, cls.reason = None, False, str(exc)

    @classmethod
    def tearDownClass(cls):
        if cls.tmp:
            shutil.rmtree(cls.tmp, ignore_errors=True)

    def skip_or_fail(self):
        """Skip without the native firmware mode, or fail when UBTS_STEER_REQUIRE_FIRMWARE is set."""
        if native_build.firmware_required():
            self.fail('%s is set but the native firmware mode is unavailable: %s' % (
                native_build.ENV_REQUIRE_FIRMWARE, self.reason))
        self.skipTest('native firmware mode unavailable: %s' % self.reason)

    def test_all_scripts_pass_on_the_native_firmware(self):
        if not self.available:
            self.skip_or_fail()
        failures = []
        for path in rc.script_paths():
            script = rc.load_script(path)
            factory = rc.native_transport_factory(script, self.executable)
            result = rc.run_script(script, factory)
            if not result['pass']:
                failures.append(rc.format_result(result))
        self.assertEqual(failures, [], '\n'.join(failures))

    def test_the_package_codec_gives_the_same_trace(self):
        # The same program through actuator_link.FirmwareSteeringBackend (native/lockstep_v2.py): the startup
        # object must equal startup_config(), every script passes, and the trace equals the runner codec's trace.
        if not self.available:
            self.skip_or_fail()
        failures = []
        for path in rc.script_paths():
            script = rc.load_script(path)
            schedule = rc.schedule_inputs(script)
            traces = []
            for factory in (rc.native_transport_factory, rc.backend_transport_factory):
                transport = factory(script, self.executable)()
                try:
                    traces.append(rc.execute(script, transport, rc.DEFAULT_STEP_TICKS, schedule))
                finally:
                    transport.close()
            difference = rc.compare_traces(*traces)
            if difference is not None:
                failures.append('%s: %s' % (script.name, difference))
            elif not rc.evaluate(script, traces[1])['pass_']:
                failures.append('%s: fails through the backend transport' % script.name)
        self.assertEqual(failures, [], '\n'.join(failures))

    def test_a_wrong_expectation_fails_on_the_native_firmware(self):
        if not self.available:
            self.skip_or_fail()
        path = [p for p in rc.script_paths() if p.stem == 'c_ttl_expiry_brake'][0]
        data = json.loads(path.read_bytes().decode('utf-8'))
        for item in data['expect']:
            if item.get('text') == 'F TTL 1 320*{crc}':
                item['tick'] = 321
        script = rc.Script(data, data['name'])
        result = rc.run_script(script, rc.native_transport_factory(script, self.executable))
        self.assertFalse(result['pass'])
        self.assertEqual([item['id'] for item in result['expectations'] if not item['pass']], ['expect[10]'])


if __name__ == '__main__':
    unittest.main()
