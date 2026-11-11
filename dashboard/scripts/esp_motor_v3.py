"""ESP protocol V3 (UBTS_STEER_V3) on the host: codec, link state and transports.

Normative text: docs/steering/ESP_PROTOCOL_V3_DRAFT_KO.md (sections 2-5, 10, 11). Every
number is a placeholder value; results are placeholder-based SIM results, not field data.

Layers (design 2.4 "same code" range):
  * codec      crc8(), frame(), the *_line() command builders, parse_mcu_line()
  * link       EspSteerV3: port-open handshake (wait for a new banner), identity rule
               (sim flag, hardware profile), status/seq/ms bookkeeping, q numbering. The
               clock is injected; no wall clock is read here.
  * transports MemoryTransport (in-process byte pipes, injected clock; tests and T1),
               PtyTransport (Linux pty under /run/ubts-steer, T2) and SerialTransport
               (USB by-id device with udev ID_SERIAL check, T3). fcntl and termios are
               imported only inside PtyTransport and SerialTransport methods.

No ROS, numpy or sim imports. Runs on Python 3.10, 3.12 and 3.13.
"""
from __future__ import annotations

import collections
import os
import posixpath
import re
import time

FIRMWARE = 'UBTS_STEER_V3'
PROTOCOL = 3
MAX_HOST_CONTENT = 63           # spec 2.2: content bytes of one host line (CR/LF excluded)
MAX_RX_LINE = 256               # host receive splitter: longer MCU lines are dropped (max real line 174 B)
DEFAULT_BAUD = 115200
DEFAULT_HW = 'front_steer_v0'
SIM_DEVICE_ROOT = '/run/ubts-steer'
SIM_DEVICE_PATH = '/run/ubts-steer/esp-tty'

# MCU line kinds and their maximum length including LF (spec 5.1).
MAX_MCU_LINE = {'banner': 174, 'cal': 129, 'status': 85, 'reject': 18, 'fault': 35, 'watchdog': 16,
                'ack': 26, 'pin': 16}

STATE_NAMES = {1: 'DISARMED', 2: 'ARMED', 3: 'BENCH'}
RESET_NAMES = ('power_on', 'hw_wdt', 'exception', 'soft_wdt', 'soft_restart', 'deep_sleep', 'ext_sys', 'unknown')
REJECT_REASONS = ('long', 'ctrl', 'crc', 'cmd', 'nocrc', 'state', 'syntax', 'range', 'boot', 'nocal', 'cal', 'pol',
                  'fault', 'old', 'busy', 'wear', 'key', 'pin')
# fl bits (spec 5.3); bit 15 is reserved and always 0.
FAULT_BIT_NAMES = ('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'SOFTLIMIT',
                   'STALL', 'TRACK', 'THERMAL', 'TTL', 'WATCHDOG', 'CRC', 'BOOT')
LATCHED_FAULT_BITS = ('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'STALL',
                      'TRACK', 'THERMAL', 'WATCHDOG', 'CRC')
# F line codes and the range of their argument (spec 5.6; bounds follow from the field definitions).
FAULT_CODE_ARG = {'NOCAL': (0, 0), 'POT_OPEN': (0, 1023), 'POT_SHORT': (0, 1023), 'POT_JUMP': (-1023, 1023),
                  'POT_NOISE': (0, 2147483647), 'POLARITY': (-19998, 19998), 'DRV_STUCK': (-19998, 19998),
                  'STALL': (-19998, 19998), 'TRACK': (-19998, 19998), 'THERMAL': (0, 60000), 'TTL': (0, 65535),
                  'CRC': (1, 50)}
PIN_ACK_PINS = (8, 5, 3, 0, 1)

# CFG keys in record order (spec 4.10): (key, default, lo, hi, choices or None). Cross checks (minp <= maxp,
# rstd <= rmov, pvmin < pvmax, cwmin < cwmax, tha <= thw * 1000) need the MCU work copy and stay on the MCU.
CFG_KEYS = (
    ('kp', 245, 1, 4096, None), ('db', 20, 0, 200, None), ('minp', 102, 0, 255, None), ('maxp', 255, 1, 255, None),
    ('dwell', 200, 0, 2000, None), ('rev', 20, 0, 500, None), ('rmov', 960, 1, 5000, None),
    ('rstd', 480, 1, 5000, None), ('ttlmax', 300, 50, 1000, None), ('wd', 500, 100, 5000, None),
    ('ttlmode', 0, 0, 1, None), ('wdmode', 0, 0, 2, None), ('hloss', 2000, 0, 10000, None),
    ('hstop', 2000, 0, 10000, None), ('hdis', 2000, 0, 10000, None), ('stoppol', 0, 0, 2, None),
    ('sthz', 20, 10, 50, (10, 20, 25, 40, 50)), ('med', 5, 1, 9, (1, 3, 5, 7, 9)), ('pvmin', 20, 0, 1023, None),
    ('pvmax', 1003, 0, 1023, None), ('cwmin', 60, 0, 1023, None), ('cwmax', 963, 0, 1023, None),
    ('jmp', 8, 1, 1023, None), ('jmpw', 2, 1, 100, None), ('nsd', 50, 1, 1000, None), ('nsw', 50, 2, 100, None),
    ('polp', 40, 1, 1000, None), ('polt', 100, 1, 1000, None), ('stp', 153, 0, 255, None),
    ('stt', 500, 1, 1000, None), ('stm', 20, 0, 1000, None), ('trk', 120, 1, 2000, None),
    ('trt', 1000, 1, 10000, None), ('dsm', 80, 1, 1000, None), ('dst', 300, 1, 1000, None),
    ('thp', 204, 0, 255, None), ('thw', 30, 1, 60, None), ('tha', 5000, 1, 60000, None), ('crcn', 5, 1, 50, None),
    ('movms', 300, 0, 2000, None), ('svhz', 500, 500, 1000, (500, 1000)),
)
CFG_KEY_NAMES = tuple(row[0] for row in CFG_KEYS)
_CFG = {row[0]: row for row in CFG_KEYS}


class V3LineError(ValueError):
    """A received MCU line is malformed. .code: framing, crc, syntax, range or kind."""

    def __init__(self, code, detail=''):
        self.code = code
        self.detail = detail
        super().__init__(code + (': ' + detail if detail else ''))


# --- CRC-8 (spec 3): poly 0x07, init 0, no reflection, xorout 0 ---------------------------------------------------
def _crc8_table():
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return tuple(table)


CRC8_TABLE = _crc8_table()


def crc8(data):
    """CRC-8 (poly 0x07, init 0x00, no reflection, xorout 0) of bytes. crc8(b'123456789') == 0xF4."""
    crc = 0
    for byte in bytes(data):
        crc = CRC8_TABLE[crc ^ byte]
    return crc


# --- host command builders (spec 2.5, 4) ---------------------------------------------------------------------------
def _int(value, lo, hi, name):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError('%s_must_be_int' % name)
    if not lo <= value <= hi:
        raise ValueError('%s_out_of_range' % name)
    return value


def frame(body, crc=True):
    """One host line: body (+ '*HH' uppercase when crc) + LF. No CR. Checks the line frame of spec 2."""
    if not isinstance(body, str):
        raise TypeError('frame body must be str')
    data = body.encode('ascii', 'strict') if all(ord(ch) < 128 for ch in body) else None
    if data is None or any(not 0x20 <= b <= 0x7E for b in data):
        raise ValueError('frame_body_not_printable_ascii')
    if not data or data.startswith(b' ') or data.endswith(b' ') or b'  ' in data:
        raise ValueError('frame_body_bad_spacing')
    if b'*' in data:
        raise ValueError('frame_body_contains_star')
    if crc:
        data += b'*%02X' % crc8(data)
    if len(data) > MAX_HOST_CONTENT:
        raise ValueError('frame_too_long')
    return data + b'\n'


_BOOT_ID = re.compile(r'[0-9A-F]{8}\Z')


def _boot(boot_id):
    if not isinstance(boot_id, str) or not _BOOT_ID.match(boot_id) or boot_id == '00000000':
        raise ValueError('boot_id_must_be_8_uppercase_hex')
    return boot_id


def info_line():
    return frame('INFO', crc=False)


def ping_line():
    return frame('PING', crc=False)


def stop_line():
    return frame('STOP', crc=False)


def arm_line(boot_id):
    return frame('ARM ' + _boot(boot_id))


def disarm_line():
    return frame('DISARM')


def bench_line(boot_id):
    return frame('BENCH ' + _boot(boot_id))


def c_line(q, ttl, left, right, steer):
    """C <q> <ttl> <L> <R> <S> with CRC (spec 4.8). ttl is checked against 1..1000 (the MCU ttlmax range)."""
    return frame('C %d %d %d %d %d' % (_int(q, 0, 65535, 'q'), _int(ttl, 1, 1000, 'ttl'),
                                        _int(left, -255, 255, 'left'), _int(right, -255, 255, 'right'),
                                        _int(steer, -1000, 1000, 'steer')))


def cal_jog_line(pwm, ms):
    _int(pwm, -255, 255, 'pwm')
    if pwm == 0:
        raise ValueError('pwm_must_not_be_zero')
    return frame('CAL JOG %d %d' % (pwm, _int(ms, 1, 500, 'ms')))


def cal_pol_line():
    return frame('CAL POL')


def cal_mark_line(which):
    if which not in ('L', 'C', 'R'):
        raise ValueError('mark_must_be_L_C_or_R')
    return frame('CAL MARK ' + which)


def cal_soft_line(left, right):
    return frame('CAL SOFT %d %d' % (_int(left, 1, 1000, 'soft_left'), _int(right, -1000, -1, 'soft_right')))


def cal_save_line():
    return frame('CAL SAVE')


def cal_erase_line():
    return frame('CAL ERASE')


def cal_get_line():
    return frame('CAL GET')


def cfg_set_line(key, value):
    row = _CFG.get(key)
    if row is None:
        raise ValueError('cfg_key_unknown')
    _int(value, row[2], row[3], 'cfg_value')
    if row[4] is not None and value not in row[4]:
        raise ValueError('cfg_value_not_a_choice')
    return frame('CFG %s %d' % (key, value))


def cfg_get_line(key):
    if key not in _CFG:
        raise ValueError('cfg_key_unknown')
    return frame('CFG ' + key)


def pin_line(n, value):
    """BENCH pin command D<n> <v>, sent without CRC (spec 2.5). n 0..8 is the grammar; the MCU accepts 8,5,3,0,1."""
    return frame('D%d %d' % (_int(n, 0, 8, 'pin'), _int(value, 0, 255, 'pin_value')), crc=False)


# --- MCU line parser (spec 5) ----------------------------------------------------------------------------------------
_UDEC = re.compile(r'(0|[1-9][0-9]{0,9})\Z')
_SDEC = re.compile(r'(0|-?[1-9][0-9]{0,9})\Z')
_HEX8 = re.compile(r'[0-9A-F]{8}\Z')
_HEX6 = re.compile(r'[0-9A-F]{6}\Z')
_HEX4 = re.compile(r'[0-9A-F]{4}\Z')
_HW = re.compile(r'[a-z0-9_]{1,32}\Z')
_HEXDIGITS = frozenset(b'0123456789ABCDEF')


def _udec(token, lo, hi, name):
    if not _UDEC.match(token):
        raise V3LineError('syntax', '%s: %r is not a canonical unsigned decimal' % (name, token))
    value = int(token)
    if not lo <= value <= hi:
        raise V3LineError('range', '%s: %d outside %d..%d' % (name, value, lo, hi))
    return value


def _sdec(token, lo, hi, name):
    if not _SDEC.match(token):
        raise V3LineError('syntax', '%s: %r is not a canonical signed decimal' % (name, token))
    value = int(token)
    if not lo <= value <= hi:
        raise V3LineError('range', '%s: %d outside %d..%d' % (name, value, lo, hi))
    return value


def _check_len(kind, content):
    if len(content) + 1 > MAX_MCU_LINE[kind]:
        raise V3LineError('syntax', '%s line is %d B with LF, above %d B' % (kind, len(content) + 1, MAX_MCU_LINE[kind]))


def _keyvals(tokens, keys, kind):
    if len(tokens) != len(keys):
        raise V3LineError('syntax', '%s needs %d fields, got %d' % (kind, len(keys), len(tokens)))
    out = {}
    for token, key in zip(tokens, keys):
        name, sep, value = token.partition('=')
        if not sep or name != key:
            raise V3LineError('syntax', '%s field %r where %s= was expected' % (kind, token, key))
        out[key] = value
    return out


_BANNER_KEYS = ('proto', 'build', 'dev', 'boot', 'rst', 'hw', 'out', 'cal', 'sim', 'wd', 'ttl', 'st', 'drop')
_CAL_KEYS = ('id', 'crc', 'left', 'center', 'right', 'soft', 'pol', 'mdir', 'caldirty', 'cfgdirty')
_STATUS_FIELDS = ('ms', 'n', 'st', 'q', 'L', 'R', 'en', 'raw', 'pos', 'tgt', 'cmd', 'sp', 'fl', 'wd', 'tl')


def _parse_banner(fields):
    f = _keyvals(fields, _BANNER_KEYS, 'banner')
    if f['proto'] != '3':
        raise V3LineError('range', 'banner proto=%r (need 3)' % f['proto'])
    for key, pattern in (('build', _HEX8), ('dev', _HEX6), ('boot', _HEX8)):
        if not pattern.match(f[key]):
            raise V3LineError('syntax', 'banner %s=%r is not uppercase hex' % (key, f[key]))
    if f['boot'] == '00000000':
        raise V3LineError('range', 'banner boot id 00000000')
    if f['rst'] not in RESET_NAMES:
        raise V3LineError('range', 'banner rst=%r' % f['rst'])
    if not _HW.match(f['hw']):
        raise V3LineError('syntax', 'banner hw=%r' % f['hw'])
    if f['out'] != 'drive2,steer1':
        raise V3LineError('range', 'banner out=%r' % f['out'])
    cal_id = None if f['cal'] == 'none' else _udec(f['cal'], 1, 65535, 'banner cal')
    if f['sim'] not in ('0', '1'):
        raise V3LineError('syntax', 'banner sim=%r' % f['sim'])
    st = _udec(f['st'], 10, 50, 'banner st')
    if st not in (10, 20, 25, 40, 50):
        raise V3LineError('range', 'banner st=%d is not a status rate choice' % st)
    return dict(kind='banner', proto=3, build=f['build'], dev=f['dev'], boot=f['boot'], rst=f['rst'], hw=f['hw'],
                out=f['out'], cal_id=cal_id, sim=f['sim'] == '1', wd=_udec(f['wd'], 100, 5000, 'banner wd'),
                ttl=_udec(f['ttl'], 50, 1000, 'banner ttl'), st=st, drop=_udec(f['drop'], 0, 65535, 'banner drop'))


def _parse_cal(fields):
    f = _keyvals(fields, _CAL_KEYS, 'cal')
    cal_id = None if f['id'] == 'none' else _udec(f['id'], 1, 65535, 'cal id')
    if not _HEX8.match(f['crc']):
        raise V3LineError('syntax', 'cal crc=%r' % f['crc'])
    if cal_id is None and f['crc'] != '00000000':
        raise V3LineError('range', 'cal id=none needs crc=00000000')
    marks = {}
    for key in ('left', 'center', 'right'):
        marks[key] = None if f[key] == '-' else _udec(f[key], 0, 1023, 'cal ' + key)
    soft_l, sep, soft_r = f['soft'].partition(',')
    if not sep:
        raise V3LineError('syntax', 'cal soft=%r' % f['soft'])
    soft_l = _sdec(soft_l, 0, 1000, 'cal soft left')
    soft_r = _sdec(soft_r, -1000, 0, 'cal soft right')
    if (soft_l == 0) != (soft_r == 0):
        raise V3LineError('range', 'cal soft=%r: both set or both 0' % f['soft'])
    if f['pol'] not in ('ok', 'no'):
        raise V3LineError('syntax', 'cal pol=%r' % f['pol'])
    if f['mdir'] not in ('1', '-1'):
        raise V3LineError('syntax', 'cal mdir=%r' % f['mdir'])
    for key in ('caldirty', 'cfgdirty'):
        if f[key] not in ('0', '1'):
            raise V3LineError('syntax', 'cal %s=%r' % (key, f[key]))
    return dict(kind='cal', cal_id=cal_id, crc32=f['crc'], left=marks['left'], center=marks['center'],
                right=marks['right'], soft_l=soft_l, soft_r=soft_r, soft_set=soft_l != 0, pol_ok=f['pol'] == 'ok',
                mdir=int(f['mdir']), caldirty=f['caldirty'] == '1', cfgdirty=f['cfgdirty'] == '1')


def _parse_status(fields):
    if len(fields) != len(_STATUS_FIELDS):
        raise V3LineError('syntax', 'status needs %d fields, got %d' % (len(_STATUS_FIELDS), len(fields)))
    v = dict(zip(_STATUS_FIELDS, fields))
    if not _HEX4.match(v['fl']):
        raise V3LineError('syntax', 'status fl=%r is not 4 uppercase hex digits' % v['fl'])
    fl = int(v['fl'], 16)
    if fl & 0x8000:
        raise V3LineError('range', 'status fl=%s has the reserved bit 15' % v['fl'])
    st = _udec(v['st'], 1, 3, 'status st')
    return dict(kind='status', ms=_udec(v['ms'], 0, 4294967295, 'status ms'), n=_udec(v['n'], 0, 65535, 'status n'),
                st=st, state=STATE_NAMES[st], q=_udec(v['q'], 0, 65535, 'status q'),
                L=_sdec(v['L'], -255, 255, 'status L'), R=_sdec(v['R'], -255, 255, 'status R'),
                en=_udec(v['en'], 0, 3, 'status en'), raw=_udec(v['raw'], 0, 1023, 'status raw'),
                pos=_sdec(v['pos'], -9999, 9999, 'status pos'), tgt=_sdec(v['tgt'], -9999, 9999, 'status tgt'),
                cmd=_sdec(v['cmd'], -9999, 9999, 'status cmd'), sp=_sdec(v['sp'], -255, 255, 'status sp'),
                fl=fl, wd=_udec(v['wd'], 0, 9999, 'status wd'), tl=_udec(v['tl'], 0, 1000, 'status tl'))


def _parse_reject(fields):
    if len(fields) != 2:
        raise V3LineError('syntax', 'N line needs 2 fields')
    q = None if fields[0] == '-' else _udec(fields[0], 0, 65535, 'N q')
    if fields[1] not in REJECT_REASONS:
        raise V3LineError('range', 'N reason %r' % fields[1])
    return dict(kind='reject', q=q, reason=fields[1])


def _parse_fault(fields):
    if len(fields) != 3:
        raise V3LineError('syntax', 'F line needs 3 fields')
    code = fields[0]
    if code not in FAULT_CODE_ARG:
        raise V3LineError('range', 'F code %r' % code)
    lo, hi = FAULT_CODE_ARG[code]
    return dict(kind='fault', code=code, arg=_sdec(fields[1], lo, hi, 'F ' + code + ' arg'),
                ms=_udec(fields[2], 0, 4294967295, 'F ms'))


def _parse_ack(fields):
    if fields and fields[0] == 'CFG':
        if len(fields) != 3 or fields[1] not in _CFG:
            raise V3LineError('syntax', 'A CFG line')
        row = _CFG[fields[1]]
        return dict(kind='ack', what='CFG', key=fields[1], value=_udec(fields[2], row[2], row[3], 'A CFG value'))
    if fields and fields[0] == 'JOG':
        if len(fields) != 3:
            raise V3LineError('syntax', 'A JOG line')
        pwm = _sdec(fields[1], -255, 255, 'A JOG pwm')
        if pwm == 0:
            raise V3LineError('range', 'A JOG pwm 0')
        return dict(kind='ack', what='JOG', pwm=pwm, ms=_udec(fields[2], 1, 500, 'A JOG ms'))
    if fields and fields[0] == 'POL':
        if len(fields) != 4 or fields[1] not in ('ok', 'fail'):
            raise V3LineError('syntax', 'A POL line')
        return dict(kind='ack', what='POL', ok=fields[1] == 'ok', d1=_sdec(fields[2], -1023, 1023, 'A POL d1'),
                    d2=_sdec(fields[3], -1023, 1023, 'A POL d2'))
    raise V3LineError('syntax', 'A line %r' % (' '.join(fields),))


def _parse_pin(fields):
    if len(fields) != 3 or not re.match(r'D[0-8]\Z', fields[0]) or int(fields[0][1]) not in PIN_ACK_PINS:
        raise V3LineError('syntax', 'P line %r' % (' '.join(fields),))
    return dict(kind='pin', pin=int(fields[0][1]), duty=_udec(fields[1], 0, 255, 'P duty'),
                target=_udec(fields[2], 0, 255, 'P target'))


def parse_mcu_line(content):
    """Parse one MCU line (bytes without the LF). Returns a dict with 'kind'. Raises V3LineError.

    Strict: printable ASCII only, no CR (the MCU never sends one), a '*HH' uppercase CRC-8 suffix that must match,
    single spaces, canonical decimals, field ranges and the maximum length of each kind (spec 5).
    """
    content = bytes(content)
    if any(not 0x20 <= b <= 0x7E for b in content):
        raise V3LineError('framing', 'byte outside 0x20..0x7E (CR included)')
    if len(content) < 4 or content[-3:-2] != b'*' or any(b not in _HEXDIGITS for b in content[-2:]):
        raise V3LineError('crc', 'missing or malformed *HH suffix')
    body = content[:-3]
    if crc8(body) != int(content[-2:].decode('ascii'), 16):
        raise V3LineError('crc', 'CRC-8 mismatch')
    text = body.decode('ascii')
    if '*' in text:
        raise V3LineError('syntax', 'extra * in body')
    tokens = text.split(' ')
    if any(token == '' for token in tokens):
        raise V3LineError('syntax', 'leading, trailing or double space')
    head = tokens[0]
    if head == FIRMWARE and len(tokens) >= 2 and tokens[1] == 'hello':
        _check_len('banner', content)
        return _parse_banner(tokens[2:])
    if head == FIRMWARE and len(tokens) >= 2 and tokens[1] == 'cal':
        _check_len('cal', content)
        return _parse_cal(tokens[2:])
    parsers = {'S': ('status', _parse_status), 'N': ('reject', _parse_reject), 'F': ('fault', _parse_fault),
               'W': ('watchdog', None), 'A': ('ack', _parse_ack), 'P': ('pin', _parse_pin)}
    if head not in parsers:
        raise V3LineError('kind', 'unknown first token %r' % head[:24])
    kind, parser = parsers[head]
    _check_len(kind, content)
    if kind == 'watchdog':
        if len(tokens) != 2:
            raise V3LineError('syntax', 'W line needs 1 field')
        return dict(kind='watchdog', ms=_udec(tokens[1], 0, 4294967295, 'W ms'))
    return parser(tokens[1:])


def mcu_line(body):
    """Build an MCU line (body + '*HH' + LF) like the firmware does. For fake devices in tests and tools."""
    data = body.encode('ascii')
    return data + b'*%02X\n' % crc8(data)


def fault_names(fl):
    """Names of the bits set in a status fl mask (spec 5.3), in bit order."""
    return [name for bit, name in enumerate(FAULT_BIT_NAMES) if fl & (1 << bit)]


class LineSplitter:
    """Splits received bytes into LF-terminated lines. Lines longer than max_line are dropped and counted."""

    def __init__(self, max_line=MAX_RX_LINE):
        self.max_line = max_line
        self.buffer = bytearray()
        self.dropping = False
        self.overlong = 0

    def reset(self):
        self.buffer = bytearray()
        self.dropping = False

    def feed(self, data):
        lines = []
        for byte in bytes(data):
            if byte == 0x0A:
                if self.dropping:
                    self.dropping = False
                else:
                    lines.append(bytes(self.buffer))
                self.buffer = bytearray()
            elif self.dropping:
                continue
            elif len(self.buffer) >= self.max_line:
                self.dropping = True
                self.overlong += 1
                self.buffer = bytearray()
            else:
                self.buffer.append(byte)
        return lines


# --- link: handshake, identity, status bookkeeping -------------------------------------------------------------------
class EspSteerV3:
    """V3 interpretation and state for one device. Clock injected (seconds, monotonic or SIM time).

    simulated=True is the SIM wrapper rule (accept only banners with sim=1); False is the real-hardware rule
    (accept only sim=0). A banner with the other flag refuses the device: the link never becomes ready and only
    STOP may still be written (spec 10.4, design 2.5). The hardware profile is compared and reported
    (hw_mismatch); the bridge refuses arm on a mismatch.

    Port open is not ready: after open() every received byte is discarded until a CRC-valid banner arrives
    (spec 10.7). If none arrives within banner_timeout_s, INFO is sent (again every banner_timeout_s).
    """

    STATUS_HISTORY = 16   # status lines kept for take_status_history() (0.8 s at 20 Hz); older ones are counted

    def __init__(self, transport, clock=time.monotonic, *, simulated, expect_hw=DEFAULT_HW, banner_timeout_s=1.5,
                 tx_log_size=256, event_log_size=64):
        if not isinstance(simulated, bool):
            raise TypeError('simulated must be True (SIM wrapper) or False (real hardware)')
        self.transport = transport
        self.clock = clock
        self.simulated = simulated
        self.expect_hw = expect_hw
        self.banner_timeout_s = float(banner_timeout_s)
        self.splitter = LineSplitter()
        self.tx_log = collections.deque(maxlen=tx_log_size)
        self.recent = collections.deque(maxlen=event_log_size)
        self.counters = collections.Counter()
        self.status_history = collections.deque(maxlen=self.STATUS_HISTORY)
        self.sessions = 0
        self._reset_session()
        self.state = 'closed'
        self.last_error = None

    def _reset_session(self):
        self.state = 'awaiting_banner'
        self.opened_at = None
        self.info_sent_at = None
        self.banner = None
        self.banner_t = None
        self.boot_id = None
        self.identity_error = None
        self.hw_mismatch = False
        self.cal = None
        self.cal_t = None
        self.status = None
        self.status_t = None
        self.status_lines = 0
        self.next_q = 1
        self._q_synced = False

    def take_status_history(self):
        """[(status line dict, receive time)] of every status line since the last call, oldest first. More than
        STATUS_HISTORY lines between two calls: the oldest are dropped and counted (status_history_dropped)."""
        out = list(self.status_history)
        self.status_history.clear()
        return out

    # -- lifecycle
    @property
    def is_open(self):
        return self.state != 'closed' and bool(getattr(self.transport, 'is_open', False))

    @property
    def handshake_done(self):
        return self.state in ('ready', 'refused')

    @property
    def ready(self):
        """Open, banner received for this session and the identity accepted (sim flag)."""
        return self.state == 'ready' and self.is_open

    def open(self):
        self.splitter.reset()
        self._reset_session()
        try:
            self.transport.open()
        except BaseException:
            self.state = 'closed'
            raise
        self.sessions += 1
        self.opened_at = self.clock()
        self.state = 'awaiting_banner'
        self.last_error = None

    def close(self, reason=None):
        try:
            self.transport.close()
        finally:
            self.state = 'closed'
            if reason:
                self.last_error = reason

    # -- sending
    def send(self, data, label=None):
        if self.state == 'closed':
            raise OSError('link closed')
        if self.state == 'refused' and data != stop_line():
            raise OSError('device refused: ' + str(self.identity_error))
        self.transport.write(data)
        self.counters['tx_lines'] += 1
        self.counters['tx_bytes'] += len(data)
        self.tx_log.append((self.clock(), label or data[:8].decode('ascii', 'replace').split(' ')[0].strip(), data))

    def send_stop(self):
        self.send(stop_line(), 'STOP')

    def send_ping(self):
        self.send(ping_line(), 'PING')

    def send_info(self):
        self.send(info_line(), 'INFO')
        self.info_sent_at = self.clock()

    def send_cal_get(self):
        self.send(cal_get_line(), 'CAL GET')

    def send_arm(self):
        if not self.boot_id:
            raise OSError('no boot id yet')
        self.send(arm_line(self.boot_id), 'ARM')

    def send_disarm(self):
        self.send(disarm_line(), 'DISARM')

    def resync_q(self):
        """Take the next q from the next status line (its q is the MCU last_q), e.g. after an 'old' rejection."""
        self._q_synced = False

    def send_c(self, ttl, left, right, steer):
        """Send one C line with a new q (q never repeats within a boot, resends included). Returns q."""
        q = self.next_q
        data = c_line(q, ttl, left, right, steer)
        self.send(data, 'C')
        self.next_q = (q + 1) % 65536
        return q

    # -- receiving
    def poll(self, limit=4096):
        """Read available bytes and return the events they produce, in arrival order.

        Event dicts: {'type': 'banner'|'cal'|'status'|'reject'|'fault'|'watchdog'|'ack'|'pin', 't', 'line', ...},
        {'type': 'bad_line', 'code', 'detail'}, {'type': 'identity', 'error'}, {'type': 'link_error', 'detail'}.
        Before the banner of a session every line is discarded (counted as discarded_before_banner).
        """
        events = []
        if self.state == 'closed':
            return events
        try:
            data = self.transport.read(limit)
        except OSError as exc:
            self.close('read failed: %s' % exc)
            events.append(dict(type='link_error', t=self.clock(), detail=str(exc)))
            return events
        if data:
            self.counters['rx_bytes'] += len(data)
            for line in self.splitter.feed(data):
                event = self._on_line(line)
                if event is not None:
                    events.append(event)
                    self.recent.append(event)
        if self.splitter.overlong:
            self.counters['rx_overlong'] += self.splitter.overlong
            self.splitter.overlong = 0
        if self.state == 'awaiting_banner':
            now = self.clock()
            last = self.info_sent_at if self.info_sent_at is not None else self.opened_at
            if now - last >= self.banner_timeout_s:
                try:
                    self.send_info()
                except OSError as exc:
                    self.close('write failed: %s' % exc)
                    events.append(dict(type='link_error', t=now, detail=str(exc)))
        return events

    def _on_line(self, raw):
        now = self.clock()
        try:
            line = parse_mcu_line(raw)
        except V3LineError as exc:
            if self.state == 'awaiting_banner':
                self.counters['discarded_before_banner'] += 1
                return None
            self.counters['rx_bad_' + exc.code] += 1
            return dict(type='bad_line', t=now, code=exc.code, detail=exc.detail)
        self.counters['rx_lines'] += 1
        kind = line['kind']
        if self.state == 'awaiting_banner':
            if kind != 'banner':
                self.counters['discarded_before_banner'] += 1
                return None
            return self._first_banner(now, line)
        if kind == 'banner':
            changed = line['boot'] != self.boot_id
            self.banner, self.banner_t = line, now
            if changed:
                self.boot_id = line['boot']
                self.cal = self.cal_t = None
                self.status = self.status_t = None
                self.status_lines = 0
                self.next_q = 1
                self._q_synced = True   # a new boot starts at last_q 0 (spec 4.8)
                self.counters['boot_changes'] += 1
            identity = self._check_identity(line)
            return dict(type='banner', t=now, line=line, new_session=False, boot_changed=changed, identity=identity)
        if kind == 'cal':
            self.cal, self.cal_t = line, now
            return dict(type='cal', t=now, line=line)
        if kind == 'status':
            return self._on_status(now, line)
        return dict(type=kind, t=now, line=line)

    def _first_banner(self, now, line):
        self.banner, self.banner_t = line, now
        self.boot_id = line['boot']
        identity = self._check_identity(line)
        self.state = 'refused' if identity else 'ready'
        self.next_q = 1
        self._q_synced = False   # same boot may already have a last_q: take it from the first status line
        return dict(type='banner', t=now, line=line, new_session=True, boot_changed=False, identity=identity)

    def _check_identity(self, line):
        self.hw_mismatch = line['hw'] != self.expect_hw
        if line['sim'] != self.simulated:
            self.identity_error = 'sim_identity_mismatch'
            self.state = 'refused'
            return 'sim_identity_mismatch'
        return None

    def _on_status(self, now, line):
        prev = self.status
        ms_regressed = False
        n_missing = 0
        n_bad = False
        if prev is not None:
            dms = (line['ms'] - prev['ms']) % 4294967296
            ms_regressed = dms >= 2147483648
            dn = (line['n'] - prev['n']) % 65536
            if 1 <= dn <= 32767:
                n_missing = dn - 1
            else:
                n_bad = True
        if not self._q_synced:
            self.next_q = (line['q'] + 1) % 65536 if line['q'] else 1
            self._q_synced = True
        self.status, self.status_t = line, now
        self.status_lines += 1
        if len(self.status_history) == self.status_history.maxlen:
            self.counters['status_history_dropped'] += 1
        self.status_history.append((line, now))
        return dict(type='status', t=now, line=line, ms_regressed=ms_regressed, n_missing=n_missing, n_bad=n_bad)

    def describe(self):
        banner = self.banner or {}
        return dict(state=self.state, transport=getattr(self.transport, 'kind', None),
                    path=getattr(self.transport, 'path', None), boot_id=self.boot_id, build=banner.get('build'),
                    identity_error=self.identity_error, hw_mismatch=self.hw_mismatch, sessions=self.sessions,
                    counters=dict(sorted(self.counters.items())), last_error=self.last_error)


# --- transports -----------------------------------------------------------------------------------------------------
class MemoryTransport:
    """In-process byte pipes with an injected clock (tests and the T1 path).

    Host side: open(), close(), read(), write() like the other transports. Device side: .device (MemoryDevice):
    take()/take_timed() return host bytes (with their write time), send(data, at=None) queues device bytes that
    the host can read once clock() >= at (default clock() + latency_s). Bytes sent while the host side is closed
    are lost, like a real port. on_open callbacks run after each open (a device can model a port-open reset).
    """

    kind = 'memory'

    def __init__(self, clock=time.monotonic, latency_s=0.0, path='memory'):
        self.clock = clock
        self.latency_s = float(latency_s)
        self.path = path
        self.is_open = False
        self.open_count = 0
        self.unplugged = False
        self.fail_writes = False
        self.on_open = []
        self._to_host = collections.deque()
        self._to_device = collections.deque()
        self.lost_while_closed = 0
        self.device = MemoryDevice(self)

    def open(self):
        if self.unplugged:
            raise OSError('memory device unplugged')
        self._to_host.clear()
        self.is_open = True
        self.open_count += 1
        for callback in list(self.on_open):
            callback()

    def close(self):
        self.is_open = False

    def write(self, data):
        if not self.is_open:
            raise OSError('memory transport closed')
        if self.unplugged or self.fail_writes:
            raise OSError('memory device write failed')
        self._to_device.append((self.clock(), bytes(data)))

    def read(self, limit=4096):
        if not self.is_open:
            raise OSError('memory transport closed')
        if self.unplugged:
            raise OSError('memory device unplugged')
        now = self.clock()
        out = bytearray()
        while self._to_host and self._to_host[0][0] <= now and len(out) < limit:
            at, chunk = self._to_host.popleft()
            room = limit - len(out)
            if len(chunk) > room:
                self._to_host.appendleft((at, chunk[room:]))
                chunk = chunk[:room]
            out += chunk
        return bytes(out)


class MemoryDevice:
    """Device end of a MemoryTransport."""

    def __init__(self, transport):
        self.transport = transport

    def take(self):
        return b''.join(chunk for _, chunk in self.take_timed())

    def take_timed(self):
        out = list(self.transport._to_device)
        self.transport._to_device.clear()
        return out

    def send(self, data, at=None):
        t = self.transport
        if not t.is_open:
            t.lost_while_closed += len(data)
            return
        t._to_host.append((t.clock() + t.latency_s if at is None else float(at), bytes(data)))

    def unplug(self):
        self.transport.unplugged = True

    def replug(self):
        self.transport.unplugged = False


def _raw_tty(fd, baud):
    import termios
    speeds = {115200: termios.B115200, 230400: termios.B230400}
    if baud not in speeds:
        raise OSError('unsupported baud %r' % (baud,))
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[4] = attrs[5] = speeds[baud]
    attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def _fd_write(fd, data, timeout_s):
    import select
    deadline = time.monotonic() + timeout_s   # I/O deadline; wall-time bound of one write, not protocol time
    data = bytes(data)
    while data:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
            raise OSError('device write deadline')
        try:
            size = os.write(fd, data)
        except BlockingIOError:
            continue
        if size <= 0:
            raise OSError('device write failed')
        data = data[size:]


def _fd_read(fd, limit):
    # A raw tty (VMIN = VTIME = 0) returns b'' both for "no data" and after a hangup. Like the V2 EspMotor, look
    # with select first: readable + empty read means the device went away (a pty slave gives EIO instead).
    import select
    if not select.select([fd], [], [], 0)[0]:
        return b''
    try:
        data = os.read(fd, limit)
    except BlockingIOError:
        return b''
    except OSError as exc:
        raise OSError('device read failed: %s' % exc)
    if not data:
        raise OSError('device disconnected')
    return data


def path_under(path, root):
    """True if the POSIX device path (absolute, normalised) is root or inside it. Lexical, no file access."""
    if not isinstance(path, str) or not path.startswith('/'):
        return False
    path = posixpath.normpath(path)
    root = posixpath.normpath(root)
    return path == root or path.startswith(root.rstrip('/') + '/')


class PtyTransport:
    """T2: the slave end of a pseudo terminal created by the ESP emulator, under allowed_root only.

    A path outside allowed_root is refused before opening. The path (or the target of a link there, e.g. the
    emulator's link to /dev/pts/N) must be a Unix98 pty slave (character device, major 136..143) before it is
    opened: opening a USB serial port could reset the NodeMCU (DTR/RTS, spec 10.7). After opening, the fd must be
    the same device. There is no udev identity for a pty, so the real-hardware transport refuses it by design.
    """

    kind = 'pty'

    def __init__(self, path=SIM_DEVICE_PATH, allowed_root=SIM_DEVICE_ROOT, baud=DEFAULT_BAUD, write_timeout_s=0.1):
        self.path = path
        self.allowed_root = allowed_root
        self.baud = baud
        self.write_timeout_s = write_timeout_s
        self.fd = None

    @property
    def is_open(self):
        return self.fd is not None

    def open(self):
        import stat
        if not isinstance(self.path, str) or not path_under(self.path, self.allowed_root):
            raise OSError('SIM device path must be under %s' % self.allowed_root)
        info = os.stat(self.path)        # follows a link; stat never opens the device
        if not stat.S_ISCHR(info.st_mode):
            raise OSError('SIM device is not a character device')
        if not 136 <= os.major(info.st_rdev) <= 143:
            raise OSError('SIM device is not a pty slave (major %d)' % os.major(info.st_rdev))
        fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            if not os.isatty(fd):
                raise OSError('SIM device is not a tty')
            if os.fstat(fd).st_rdev != info.st_rdev:
                raise OSError('SIM device changed between the check and the open')
            _raw_tty(fd, self.baud)
        except BaseException:
            os.close(fd)
            raise
        self.fd = fd

    def close(self):
        if self.fd is not None:
            fd, self.fd = self.fd, None
            os.close(fd)

    def write(self, data):
        if self.fd is None:
            raise OSError('pty closed')
        _fd_write(self.fd, data, self.write_timeout_s)

    def read(self, limit=4096):
        if self.fd is None:
            raise OSError('pty closed')
        return _fd_read(self.fd, limit)


class SerialTransport:
    """T3: the real ESP8266 by its udev by-id path (same open rules as the V2 EspMotor).

    Requires an explicit device path and ID_SERIAL identity, checks the live udev identity, takes the per-device
    lock file and TIOCEXCL. Paths under the SIM device root are refused. Never opened by the tests.
    """

    kind = 'serial'

    def __init__(self, path, identity, baud=DEFAULT_BAUD, write_timeout_s=0.1, lock_dir='/run/lock',
                 udev_dir='/run/udev/data', forbidden_roots=(SIM_DEVICE_ROOT,)):
        self.path = path
        self.identity = identity
        self.baud = baud
        self.write_timeout_s = write_timeout_s
        self.lock_dir = lock_dir
        self.udev_dir = udev_dir
        self.forbidden_roots = tuple(forbidden_roots)
        self.fd = None
        self.lock = None

    @property
    def is_open(self):
        return self.fd is not None

    def check_config(self):
        """Argument checks done before any device access (also used by tests)."""
        if not self.identity or not self.path or self.path == 'auto':
            raise RuntimeError('explicit ESP identity and device path required')
        if any(path_under(self.path, root) for root in self.forbidden_roots):
            raise RuntimeError('SIM device path refused by the real-hardware transport')
        if self.baud not in (115200, 230400):
            raise RuntimeError('unsupported baud %r' % (self.baud,))

    def open(self):
        import fcntl
        import termios
        self.check_config()
        info = os.stat(self.path)
        rdev = '%d:%d' % (os.major(info.st_rdev), os.minor(info.st_rdev))
        with open(os.path.join(self.udev_dir, 'c' + rdev), encoding='utf-8', errors='replace') as handle:
            properties = handle.read().splitlines()
        if 'E:ID_SERIAL=' + self.identity not in properties:
            raise RuntimeError('live udev USB identity mismatch')
        self.lock = open(os.path.join(self.lock_dir, 'ubts-esp-%d-%d.lock' % (os.major(info.st_rdev),
                                                                              os.minor(info.st_rdev))), 'a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            _raw_tty(self.fd, self.baud)
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.fd is not None:
            try:
                import fcntl
                import termios
                fcntl.ioctl(self.fd, termios.TIOCNXCL)
            except OSError:
                pass
            fd, self.fd = self.fd, None
            os.close(fd)
        if self.lock is not None:
            self.lock.close()
            self.lock = None

    def write(self, data):
        if self.fd is None:
            raise OSError('ESP disconnected')
        _fd_write(self.fd, data, self.write_timeout_s)

    def read(self, limit=4096):
        if self.fd is None:
            raise OSError('ESP disconnected')
        return _fd_read(self.fd, limit)
