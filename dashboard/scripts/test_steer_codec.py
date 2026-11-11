"""ESP V3 codec: CRC-8 vectors of the spec, command builders and strict MCU line parsers. No hardware."""
import unittest

try:
    import test_steer_support as ts  # noqa: F401  (unittest discover -s dashboard/scripts; sets the import path)
except ImportError:                     # pytest imports these modules as scripts.test_steer_*
    from scripts import test_steer_support as ts  # noqa: F401
from scripts import esp_motor_v3 as v3

# ESP_PROTOCOL_V3_DRAFT_KO.md 3 (computed vectors) plus the vectors of the task digest.
CRC_VECTORS = {
    '': 0x00, '123456789': 0xF4, 'A': 0xC0, 'STOP': 0x66, 'PING': 0x1F, 'INFO': 0x58, 'DISARM': 0x88,
    'ARM 5F3A19C2': 0xCC, 'ARM 5f3a19c2': 0xA7, 'BENCH 5F3A19C2': 0x8A, 'C 1842 250 102 102 -310': 0x8E,
    'C 1 300 0 0 0': 0xA7, 'C 2 300 128 128 -250': 0xE2, 'CAL MARK L': 0xDD, 'CAL SAVE': 0x0F, 'CFG kp 245': 0xC9,
    'S 123456 2210 2 1842 102 102 3 611 -310 -323 -318 -64 0000 40 260': 0xCB, 'N 1842 crc': 0x24,
    'N - long': 0x3C, 'N - crc': 0xD8, 'F POT_OPEN 3 123456': 0xC7, 'F TTL 1 320': 0x6B, 'W 123456': 0xCE,
    'W 521': 0x78, 'P D8 0 0': 0xC6, 'P D1 255 255': 0xEB, 'A CFG kp 245': 0x58,
}
BANNER = (b'UBTS_STEER_V3 hello proto=3 build=0A1B2C3D dev=5A1B2C boot=89025CC1 rst=power_on hw=front_steer_v0 '
          b'out=drive2,steer1 cal=7 sim=1 wd=500 ttl=300 st=20 drop=0*01')
CAL = (b'UBTS_STEER_V3 cal id=7 crc=61AEB1A4 left=842 center=512 right=181 soft=935,-935 pol=ok mdir=1 '
       b'caldirty=0 cfgdirty=0*D6')
CAL_BLANK = (b'UBTS_STEER_V3 cal id=none crc=00000000 left=- center=- right=- soft=0,0 pol=no mdir=1 caldirty=0 '
             b'cfgdirty=0*3F')
STATUS = b'S 300 5 2 1 48 48 3 512 0 0 0 0 0000 280 20*AA'


def line(body):
    """MCU line without LF, with a correct CRC."""
    return v3.mcu_line(body)[:-1]


class Crc8Test(unittest.TestCase):
    def test_vectors(self):
        for text, expected in CRC_VECTORS.items():
            with self.subTest(text=text):
                self.assertEqual(v3.crc8(text.encode('ascii')), expected)

    def test_table_and_design_examples_are_wrong(self):
        self.assertEqual(v3.CRC8_TABLE[:16], (0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31,
                                              0x24, 0x23, 0x2A, 0x2D))
        for text, wrong in (('ARM 5f3a19c2', 0x7D), ('DISARM', 0x1C), ('C 1842 250 102 102 -310', 0x4A),
                            ('S 123456 2210 2 1842 102 102 3 611 -310 -323 -318 -64 0000 40 260', 0x5C)):
            self.assertNotEqual(v3.crc8(text.encode('ascii')), wrong)


class BuilderTest(unittest.TestCase):
    def test_bare_lines(self):
        self.assertEqual(v3.stop_line(), b'STOP\n')
        self.assertEqual(v3.ping_line(), b'PING\n')
        self.assertEqual(v3.info_line(), b'INFO\n')
        self.assertEqual(v3.pin_line(8, 77), b'D8 77\n')

    def test_crc_lines(self):
        self.assertEqual(v3.arm_line('5F3A19C2'), b'ARM 5F3A19C2*CC\n')
        self.assertEqual(v3.disarm_line(), b'DISARM*88\n')
        self.assertEqual(v3.bench_line('5F3A19C2'), b'BENCH 5F3A19C2*8A\n')
        self.assertEqual(v3.c_line(1842, 250, 102, 102, -310), b'C 1842 250 102 102 -310*8E\n')
        self.assertEqual(v3.c_line(1, 300, 0, 0, 0), b'C 1 300 0 0 0*A7\n')
        self.assertEqual(v3.cal_mark_line('L'), b'CAL MARK L*DD\n')
        self.assertEqual(v3.cal_save_line(), b'CAL SAVE*0F\n')
        self.assertEqual(v3.cfg_set_line('kp', 245), b'CFG kp 245*C9\n')

    def test_every_builder_frames_a_valid_host_line(self):
        lines = [v3.info_line(), v3.ping_line(), v3.stop_line(), v3.arm_line('89025CC1'), v3.disarm_line(),
                 v3.bench_line('89025CC1'), v3.c_line(65535, 1000, -255, -255, -1000), v3.cal_jog_line(-255, 500),
                 v3.cal_pol_line(), v3.cal_mark_line('C'), v3.cal_mark_line('R'), v3.cal_soft_line(1000, -1000),
                 v3.cal_save_line(), v3.cal_erase_line(), v3.cal_get_line(), v3.cfg_set_line('svhz', 1000),
                 v3.cfg_get_line('tha'), v3.pin_line(0, 255)]
        for data in lines:
            with self.subTest(data=data):
                self.assertTrue(data.endswith(b'\n'))
                self.assertNotIn(b'\r', data)
                content = data[:-1]
                self.assertLessEqual(len(content), 63)
                self.assertTrue(all(0x20 <= b <= 0x7E for b in content))
                if b'*' in content:
                    body, crc = content.split(b'*')
                    self.assertEqual(crc, b'%02X' % v3.crc8(body))
        self.assertEqual(len(v3.c_line(65535, 1000, -255, -255, -1000)), 32)   # spec 11.2 maximum host C line

    def test_builder_ranges(self):
        bad = [lambda: v3.c_line(65536, 300, 0, 0, 0), lambda: v3.c_line(-1, 300, 0, 0, 0),
               lambda: v3.c_line(1, 0, 0, 0, 0), lambda: v3.c_line(1, 1001, 0, 0, 0),
               lambda: v3.c_line(1, 300, 256, 0, 0), lambda: v3.c_line(1, 300, 0, -256, 0),
               lambda: v3.c_line(1, 300, 0, 0, 1001), lambda: v3.c_line(True, 300, 0, 0, 0),
               lambda: v3.c_line(1, 300.0, 0, 0, 0), lambda: v3.arm_line('5f3a19c2'), lambda: v3.arm_line('00000000'),
               lambda: v3.arm_line('5F3A19C'), lambda: v3.arm_line(0x5F3A19C2), lambda: v3.cal_jog_line(0, 10),
               lambda: v3.cal_jog_line(10, 501), lambda: v3.cal_mark_line('X'), lambda: v3.cal_soft_line(0, -1),
               lambda: v3.cal_soft_line(1, 0), lambda: v3.cfg_set_line('nokey', 1), lambda: v3.cfg_set_line('kp', 0),
               lambda: v3.cfg_set_line('sthz', 30), lambda: v3.cfg_set_line('med', 4), lambda: v3.cfg_get_line('x'),
               lambda: v3.pin_line(9, 1), lambda: v3.pin_line(8, 256)]
        for make in bad:
            with self.assertRaises(ValueError):
                make()

    def test_frame_rules(self):
        for body in ('', ' PING', 'PING ', 'C  1', 'A*B', 'café', 'X\x01'):
            with self.assertRaises(ValueError):
                v3.frame(body)
        with self.assertRaises(ValueError):
            v3.frame('X' * 61)          # 61 + '*HH' = 64 content bytes
        self.assertEqual(len(v3.frame('X' * 60)), 64)
        self.assertEqual(len(v3.frame('X' * 63, crc=False)), 64)

    def test_cfg_table(self):
        self.assertEqual(len(v3.CFG_KEYS), 41)
        self.assertEqual(v3.CFG_KEY_NAMES[0], 'kp')
        self.assertEqual(v3.CFG_KEY_NAMES[-2:], ('movms', 'svhz'))
        for key, default, lo, hi, choices in v3.CFG_KEYS:
            self.assertTrue(lo <= default <= hi, key)
            if choices:
                self.assertIn(default, choices)


class ParserTest(unittest.TestCase):
    def test_spec_examples(self):
        banner = v3.parse_mcu_line(BANNER)
        self.assertEqual(banner, dict(kind='banner', proto=3, build='0A1B2C3D', dev='5A1B2C', boot='89025CC1',
                                      rst='power_on', hw='front_steer_v0', out='drive2,steer1', cal_id=7, sim=True,
                                      wd=500, ttl=300, st=20, drop=0))
        status = v3.parse_mcu_line(STATUS)
        self.assertEqual(status, dict(kind='status', ms=300, n=5, st=2, state='ARMED', q=1, L=48, R=48, en=3,
                                      raw=512, pos=0, tgt=0, cmd=0, sp=0, fl=0, wd=280, tl=20))
        cal = v3.parse_mcu_line(CAL)
        self.assertEqual((cal['cal_id'], cal['crc32'], cal['left'], cal['center'], cal['right'], cal['soft_l'],
                          cal['soft_r'], cal['pol_ok'], cal['mdir'], cal['caldirty']),
                         (7, '61AEB1A4', 842, 512, 181, 935, -935, True, 1, False))
        blank = v3.parse_mcu_line(CAL_BLANK)
        self.assertEqual((blank['cal_id'], blank['left'], blank['soft_set'], blank['pol_ok']), (None, None, False, False))
        self.assertEqual(v3.parse_mcu_line(b'N 1842 old*4A'), dict(kind='reject', q=1842, reason='old'))
        self.assertEqual(v3.parse_mcu_line(b'N - crc*D8'), dict(kind='reject', q=None, reason='crc'))
        self.assertEqual(v3.parse_mcu_line(b'F TTL 1 320*6B'), dict(kind='fault', code='TTL', arg=1, ms=320))
        self.assertEqual(v3.parse_mcu_line(b'F POT_OPEN 3 123456*C7')['code'], 'POT_OPEN')
        self.assertEqual(v3.parse_mcu_line(b'W 521*78'), dict(kind='watchdog', ms=521))
        self.assertEqual(v3.parse_mcu_line(b'A CFG kp 245*58'), dict(kind='ack', what='CFG', key='kp', value=245))
        self.assertEqual(v3.parse_mcu_line(b'P D1 255 255*EB'), dict(kind='pin', pin=1, duty=255, target=255))
        self.assertEqual(v3.parse_mcu_line(line('A JOG -60 200'))['pwm'], -60)
        pol = v3.parse_mcu_line(line('A POL fail 5 -3'))
        self.assertEqual((pol['ok'], pol['d1'], pol['d2']), (False, 5, -3))

    def assertCode(self, raw, code):
        with self.assertRaises(v3.V3LineError) as caught:
            v3.parse_mcu_line(raw)
        self.assertEqual(caught.exception.code, code, raw)

    def test_crc_and_framing_errors(self):
        self.assertCode(STATUS[:-2] + b'aa', 'crc')               # MCU CRC digits are uppercase
        self.assertCode(STATUS[:-3], 'crc')                       # no CRC field
        self.assertCode(STATUS[:-2] + b'AB', 'crc')               # wrong CRC
        self.assertCode(STATUS + b'\r', 'framing')                # the MCU never sends CR
        self.assertCode(b'S 300\x00 5*00', 'framing')
        self.assertCode(b'S \xff*00', 'framing')
        self.assertCode(b'', 'crc')
        self.assertCode(line('S 300 5 2 1 48 48 3 512 0 0 0 0 0000 280 20*'), 'syntax')

    def test_status_strictness(self):
        good = 'S 300 5 2 1 48 48 3 512 0 0 0 0 0000 280 20'
        self.assertEqual(v3.parse_mcu_line(line(good))['tl'], 20)
        cases = {
            'S 300 5 2 1 48 48 3 512 0 0 0 0 0000 280': 'syntax',          # 14 fields
            'S 300 5 2 1 48 48 3 512 0 0 0 0 0000 280 20 1': 'syntax',     # 16 fields
            'S  300 5 2 1 48 48 3 512 0 0 0 0 0000 280 20': 'syntax',      # double space
            'S 0300 5 2 1 48 48 3 512 0 0 0 0 0000 280 20': 'syntax',      # leading zero
            'S 300 5 2 1 -0 48 3 512 0 0 0 0 0000 280 20': 'syntax',       # -0
            'S 300 5 2 1 +4 48 3 512 0 0 0 0 0000 280 20': 'syntax',       # plus sign
            'S 300 5 0 1 48 48 3 512 0 0 0 0 0000 280 20': 'range',        # st 0 never appears
            'S 300 5 4 1 48 48 3 512 0 0 0 0 0000 280 20': 'range',
            'S 300 5 2 1 256 48 3 512 0 0 0 0 0000 280 20': 'range',
            'S 300 5 2 1 48 48 4 512 0 0 0 0 0000 280 20': 'range',
            'S 300 5 2 1 48 48 3 1024 0 0 0 0 0000 280 20': 'range',
            'S 300 5 2 1 48 48 3 512 10000 0 0 0 0000 280 20': 'range',
            'S 300 5 2 1 48 48 3 512 0 0 0 0 8000 280 20': 'range',        # reserved bit 15
            'S 300 5 2 1 48 48 3 512 0 0 0 0 00a0 280 20': 'syntax',       # fl lowercase
            'S 300 5 2 1 48 48 3 512 0 0 0 0 000 280 20': 'syntax',
            'S 300 5 2 1 48 48 3 512 0 0 0 0 0000 10000 20': 'range',
            'S 300 5 2 1 48 48 3 512 0 0 0 0 0000 280 1001': 'range',
            'S 4294967296 5 2 1 48 48 3 512 0 0 0 0 0000 280 20': 'range',
            'S 300 65536 2 1 48 48 3 512 0 0 0 0 0000 280 20': 'range',
        }
        for body, code in cases.items():
            with self.subTest(body=body):
                self.assertCode(line(body), code)

    def test_banner_and_cal_strictness(self):
        base = BANNER[:-3].decode('ascii')
        cases = {
            base.replace('proto=3', 'proto=2'): 'range',
            base.replace('boot=89025CC1', 'boot=89025cc1'): 'syntax',
            base.replace('boot=89025CC1', 'boot=00000000'): 'range',
            base.replace('rst=power_on', 'rst=brownout'): 'range',
            base.replace('out=drive2,steer1', 'out=drive2'): 'range',
            base.replace('sim=1', 'sim=2'): 'syntax',
            base.replace('st=20', 'st=30'): 'range',
            base.replace('wd=500', 'wd=50'): 'range',
            base.replace('cal=7', 'cal=0'): 'range',
            base.replace(' drop=0', ''): 'syntax',
            base.replace('wd=500 ttl=300', 'ttl=300 wd=500'): 'syntax',
            base.replace('dev=5A1B2C', 'dev=5A1B2'): 'syntax',
        }
        for body, code in cases.items():
            with self.subTest(body=body):
                self.assertCode(line(body), code)
        self.assertFalse(v3.parse_mcu_line(line(base.replace('sim=1', 'sim=0')))['sim'])
        self.assertIsNone(v3.parse_mcu_line(line(base.replace('cal=7', 'cal=none')))['cal_id'])
        too_long = base.replace('hw=front_steer_v0', 'hw=' + 'h' * 32).replace('drop=0', 'drop=65535')
        too_long = too_long.replace('build=0A1B2C3D', 'build=0A1B2C3D').replace('rst=power_on', 'rst=soft_restart')
        too_long = too_long.replace('cal=7', 'cal=65535').replace('wd=500', 'wd=5000').replace('ttl=300', 'ttl=1000')
        if len(too_long) + 4 > 174:
            self.assertCode(line(too_long), 'syntax')
        cal = CAL[:-3].decode('ascii')
        for body, code in {cal.replace('id=7', 'id=none'): 'range',                       # none needs crc 0
                           cal.replace('soft=935,-935', 'soft=935,0'): 'range',
                           cal.replace('soft=935,-935', 'soft=935'): 'syntax',
                           cal.replace('pol=ok', 'pol=yes'): 'syntax',
                           cal.replace('mdir=1', 'mdir=0'): 'syntax',
                           cal.replace('left=842', 'left=1024'): 'range',
                           cal.replace('crc=61AEB1A4', 'crc=61aeb1a4'): 'syntax'}.items():
            with self.subTest(body=body):
                self.assertCode(line(body), code)

    def test_other_lines(self):
        for body, code in {'N - nope': 'range', 'N 65536 old': 'range', 'N - old x': 'syntax',
                           'F BOOT 0 1': 'range', 'F POT_OPEN 2000 1': 'range', 'F CRC 0 1': 'range',
                           'F TTL 1': 'syntax', 'W': 'syntax', 'W 1 2': 'syntax', 'A CFG nokey 1': 'syntax',
                           'A JOG 0 10': 'range', 'A POL maybe 1 1': 'syntax', 'P D2 0 0': 'syntax',
                           'P D8 256 0': 'range', 'X 1': 'kind', 'UBTS_WHEEL_PWM_V2 ready': 'kind',
                           'UBTS_STEER_V3 bye': 'kind'}.items():
            with self.subTest(body=body):
                self.assertCode(line(body), code)

    def test_fault_names(self):
        self.assertEqual(v3.fault_names(0), [])
        self.assertEqual(v3.fault_names(0x4801), ['NOCAL', 'TTL', 'BOOT'])
        self.assertEqual(len(v3.FAULT_BIT_NAMES), 15)


class SplitterTest(unittest.TestCase):
    def test_lines_and_overlong(self):
        splitter = v3.LineSplitter(max_line=10)
        self.assertEqual(splitter.feed(b'ab\ncd'), [b'ab'])
        self.assertEqual(splitter.feed(b'e\n'), [b'cde'])
        self.assertEqual(splitter.feed(b'x' * 30 + b'\nok\n'), [b'ok'])
        self.assertEqual(splitter.overlong, 1)
        self.assertEqual(splitter.feed(b'\n'), [b''])


if __name__ == '__main__':
    unittest.main()
