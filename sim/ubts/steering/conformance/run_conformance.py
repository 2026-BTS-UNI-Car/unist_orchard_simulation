#!/usr/bin/env python3
"""Conformance runner for the ESP protocol V3 steering firmware (design 8.3 P3, acceptance A19 MCU part, A20).

Script format: ubts.steer_conformance.v1 (docs/steering/ESP_PROTOCOL_V3_DRAFT_KO.md section 13). Each script
is checked strictly (unknown keys, types, ranges, tick order, tolerance rules) before anything runs.

Execution (T1, native): the runner starts the native lockstep program in firmware mode with the startup line
of lockstep v2 (docs/steering/LOCKSTEP_PROTOCOL_KO.md section 11), feeds the script's host bytes and events
tick by tick in FW_STEP_REQUEST frames (default 10 ticks per step, split where a plant table changes) and
collects the MCU bytes (with their queued and done ticks) and the pin channel changes from FW_STEP_RESPONSE.
The frame codec here is written from the protocol document alone, so it also cross-checks the native side.
Any object with the transport interface (FirmwareTransport below) can be used instead, for example an
adapter around NativeSteeringBackend once it has a firmware mode.

Checks per script:
  - every expectation of the script (line, status, no_line, count, pins, pins_window, change, first_change)
    and the end checks;
  - every MCU line: printable ASCII, ends in *HH with the right CRC-8 (upper case), known first token,
    the section 5 field format and the section 5.1 maximum length;
  - no unexpected line (end.unexpected / end.ignore_kinds) and no line left half sent after the drain ticks;
  - determinism and step independence (section 13.9): the same process is RESET and the script runs again
    with another step size; lines, ticks, channel changes and the base fields at common step ends must be
    identical. On a difference a fresh process runs the second partition to tell RESET from partitioning.

Command line (from sim/ubts):
  python -B -m steering.conformance.run_conformance --list
  python -B -m steering.conformance.run_conformance --validate-only
  python -B -m steering.conformance.run_conformance [--executable PATH | --compiler CXX --build-dir DIR]
         [--out result.json] [--step-ticks 10] [--check-step-ticks 7] [--expect-build HEX] [NAME_OR_GLOB ...]
Exit code: 0 all selected scripts pass, 1 a script failed, 2 a script is invalid or the arguments are wrong,
3 the native firmware mode is not available (no compiler, build failure, startup refused).

Standard library only on the format side; executing needs the steering package (vehicle profile, startup
builder, native build). Results built on the placeholder profile are placeholder-based SIM results.
"""
from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import threading
import time

if __package__ in (None, ''):          # run as a plain script: make `steering` importable
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    __package__ = 'steering.conformance'

SCHEMA = 'ubts.steer_conformance.v1'
RESULT_SCHEMA = 'ubts.steer_conformance_result.v1'
LOCKSTEP_PROTOCOL = 'ubts.steer_lockstep.v2'
SCRIPTS_DIR = Path(__file__).resolve().parent / 'scripts'
DEFAULT_PROFILE = 'front_steer_v0_1_placeholder'
DEFAULT_STEP_TICKS = 10
DEFAULT_CHECK_STEP_TICKS = 7
MCU_TICK_HZ = 1000
MAX_TICKS = 2000000                 # longest script (end.tick + drain) the runner accepts
NAME_RE = re.compile(r'^[a-z0-9_]{1,64}$')
HEX_RE = re.compile(r'^(?:[0-9A-Fa-f]{2})*$')
EXIT_PASS, EXIT_FAIL, EXIT_INVALID, EXIT_UNAVAILABLE = 0, 1, 2, 3

# --- CRC-8 (section 3): poly 0x07, init 0, no reflection, xorout 0 --------------------------------------------------
def _crc8_table():
    table = []
    for value in range(256):
        crc = value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return tuple(table)


CRC8_TABLE = _crc8_table()


def crc8(data):
    """CRC-8 of bytes (a str is taken as ASCII)."""
    if isinstance(data, str):
        data = data.encode('ascii')
    crc = 0
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc


def crc8_hex(data):
    return '%02X' % crc8(data)


# Section 3 test vectors (covered bytes, CRC-8). self_check() and the tests compare crc8() with them.
CRC8_VECTORS = (
    ('', '00'), ('123456789', 'F4'), ('A', 'C0'), ('STOP', '66'), ('PING', '1F'), ('INFO', '58'),
    ('DISARM', '88'), ('ARM 5F3A19C2', 'CC'), ('ARM 5f3a19c2', 'A7'), ('BENCH 5F3A19C2', '8A'),
    ('C 1842 250 102 102 -310', '8E'), ('C 1 300 0 0 0', 'A7'), ('C 2 300 128 128 -250', 'E2'),
    ('CAL MARK L', 'DD'), ('CAL SAVE', '0F'), ('CFG kp 245', 'C9'),
    ('S 123456 2210 2 1842 102 102 3 611 -310 -323 -318 -64 0000 40 260', 'CB'), ('N 1842 crc', '24'),
    ('N - long', '3C'), ('F POT_OPEN 3 123456', 'C7'), ('W 123456', 'CE'), ('P D8 0 0', 'C6'),
    ('N - crc', 'D8'), ('F TTL 1 320', '6B'), ('W 521', '78'), ('P D1 255 255', 'EB'), ('A CFG kp 245', '58'))
CRC8_TABLE_FIRST16 = (0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D)


def crc32_iso_hdlc(data):
    """CRC-32/ISO-HDLC (section 9.3), bit by bit from the specification."""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return crc ^ 0xFFFFFFFF


def self_check():
    """Compare the CRC-8, CRC-32, record and boot id code with the specification's vectors; raises on a mismatch."""
    if CRC8_TABLE[:16] != CRC8_TABLE_FIRST16:
        raise AssertionError('crc8 table differs from section 3')
    for covered, expected in CRC8_VECTORS:
        if crc8_hex(covered) != expected:
            raise AssertionError('crc8(%r) = %s, specification %s' % (covered, crc8_hex(covered), expected))
    if crc32_iso_hdlc(b'123456789') != 0xCBF43926:
        raise AssertionError('crc32 check value differs from section 9.3')
    image = eeprom_image({'record': dict(EXAMPLE_RECORD)})
    if image[:106].hex().upper() != EXAMPLE_RECORD_HEX or image[106:] != b'\xff' * 150:
        raise AssertionError('record builder differs from the section 9.6 example')
    tomb = eeprom_image({'record': dict(EXAMPLE_RECORD, magic='USCX')})
    if struct.unpack_from('<I', tomb, 102)[0] != 0x4F1B5580:
        raise AssertionError('tombstone CRC-32 differs from section 9.6')
    for seed, ids in BOOT_ID_VECTORS:
        if boot_ids(seed, len(ids)) != list(ids):
            raise AssertionError('boot ids for seed %d differ from section 10.1' % seed)
    return True


# --- EEPROM record (section 9) --------------------------------------------------------------------------------------
CFG_KEYS = ('kp', 'db', 'minp', 'maxp', 'dwell', 'rev', 'rmov', 'rstd', 'ttlmax', 'wd', 'ttlmode', 'wdmode', 'hloss',
            'hstop', 'hdis', 'stoppol', 'sthz', 'med', 'pvmin', 'pvmax', 'cwmin', 'cwmax', 'jmp', 'jmpw', 'nsd', 'nsw',
            'polp', 'polt', 'stp', 'stt', 'stm', 'trk', 'trt', 'dsm', 'dst', 'thp', 'thw', 'tha', 'crcn', 'movms',
            'svhz')
CFG_DEFAULTS = dict(kp=245, db=20, minp=102, maxp=255, dwell=200, rev=20, rmov=960, rstd=480, ttlmax=300, wd=500,
                    ttlmode=0, wdmode=0, hloss=2000, hstop=2000, hdis=2000, stoppol=0, sthz=20, med=5, pvmin=20,
                    pvmax=1003, cwmin=60, cwmax=963, jmp=8, jmpw=2, nsd=50, nsw=50, polp=40, polt=100, stp=153,
                    stt=500, stm=20, trk=120, trt=1000, dsm=80, dst=300, thp=204, thw=30, tha=5000, crcn=5,
                    movms=300, svhz=500)
RECORD_KEYS = ('cal_id', 'raw_l', 'raw_c', 'raw_r', 'soft_l', 'soft_r', 'pol_ok', 'mdir')
RECORD_MAGICS = {'USC3': b'USC3', 'USCX': b'USCX'}
EXAMPLE_RECORD = dict(cal_id=7, raw_l=842, raw_c=512, raw_r=181, soft_l=935, soft_r=-935, pol_ok=1, mdir=1, params={})
EXAMPLE_RECORD_HEX = ('55534333016A07004A030002B500A70359FC0101F50014006600FF00C8001400C003E0012C01F40100000000D007D007'
                      'D0070000140005001400EB033C00C3030800020032003200280064009900F40114007800E80350002C01CC001E0088'
                      '1305002C01F401A4B1AE61')
BOOT_ID_VECTORS = ((0, ('7B1DCDAF', 'A1B965F4', '8009454F')), (1, ('89025CC1', '658EEC67', 'FB32555E')),
                   (42, ('2FEB6E95', 'B266F103', '130F9F52')))


def record_bytes(record):
    """The 106-byte record of section 9.2 (fields as given; validity is the firmware's business)."""
    params = dict(CFG_DEFAULTS)
    params.update(record.get('params') or {})
    magic = RECORD_MAGICS[record.get('magic', 'USC3')]
    body = magic + bytes([1, 106]) + struct.pack(
        '<HHHHhhBB', record['cal_id'], record['raw_l'], record['raw_c'], record['raw_r'], record['soft_l'],
        record['soft_r'], record['pol_ok'], 0x01 if record['mdir'] == 1 else 0xFF)
    body += b''.join(struct.pack('<H', params[key]) for key in CFG_KEYS)
    return body + struct.pack('<I', crc32_iso_hdlc(body))


def eeprom_image(spec):
    """256-byte EEPROM image for setup.eeprom ('blank', {'record': R} or {'hex': 512 hex digits})."""
    if spec == 'blank':
        return b'\xff' * 256
    if 'hex' in spec:
        return bytes.fromhex(spec['hex'])
    return record_bytes(spec['record']) + b'\xff' * 150


# --- SIM boot ids (section 10.1) -------------------------------------------------------------------------------------
def boot_ids(seed, count):
    """The first `count` SIM boot ids (upper-case 8 hex digits) of the splitmix64 stream seeded with `seed`."""
    state, out = seed & 0xFFFFFFFFFFFFFFFF, []
    while len(out) < count:
        state = (state + 0x9E3779B97F4A7C15) & 0xFFFFFFFFFFFFFFFF
        z = state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & 0xFFFFFFFFFFFFFFFF
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & 0xFFFFFFFFFFFFFFFF
        value = (z ^ (z >> 31)) & 0xFFFFFFFF
        if value:
            out.append('%08X' % value)
    return out


# --- script format ----------------------------------------------------------------------------------------------------
class ScriptError(ValueError):
    """A script breaks the ubts.steer_conformance.v1 format. `where` names the place ('expect[3].tol')."""

    def __init__(self, where, message):
        super().__init__('%s: %s' % (where, message))
        self.where = where


TOP_REQUIRED = ('schema', 'name', 'spec', 'setup', 'inputs', 'expect', 'end')
TOP_OPTIONAL = ('v2_map', 'description', 'plant', 'events')
SETUP_DEFAULTS = dict(mcu_tick_hz=MCU_TICK_HZ, boot_delay_ticks=0, millis_start=0, eeprom='blank',
                      adc={'mode': 'script', 'raw': 512}, report_adc=False, initial_delta_eq_deg=0.0,
                      profile=DEFAULT_PROFILE, seed=0, startup_overrides={}, injections=[], motor_power=True,
                      boot_garbage_hex='')
SETUP_REQUIRED = ('boot_seed', 'chip_id')
EVENT_KINDS = {'mcu_reset': 1, 'power_cycle': 2, 'port_open_reset': 3, 'motor_power': 4, 'mcu_stall': 5,
               'adc_set': 6, 'adc_model': 7}
RESET_EVENT_KINDS = ('mcu_reset', 'port_open_reset', 'power_cycle')
LINE_KINDS = ('BANNER', 'CAL', 'S', 'N', 'F', 'W', 'A', 'P', 'OTHER')
STATUS_FIELDS = ('ms', 'n', 'st', 'q', 'L', 'R', 'en', 'raw', 'pos', 'tgt', 'cmd', 'sp', 'fl', 'wd', 'tl')
PIN_CHANNELS = ('drive_l', 'drive_r', 'drive_en', 'steer_pwm', 'steer_en', 'mcu_state', 'fault_bits', 'gpio_writes',
                'eeprom_commits')
CHANNELS = {'drive_l': 1, 'drive_r': 2, 'drive_en': 3, 'steer_pwm': 4, 'steer_en': 5, 'mcu_state': 6,
            'fault_bits': 7, 'gpio_writes': 8, 'eeprom_commits': 9, 'adc_read': 10}
CHANNEL_NAMES = {code: name for name, code in CHANNELS.items()}
CHANNEL_INITIAL = {name: (255 if name == 'mcu_state' else 0) for name in CHANNELS}
UNSIGNED_CHANNELS = ('gpio_writes', 'eeprom_commits')
SIGN_WORDS = ('pos', 'neg', 'zero', 'nonzero')
EXPECT_KEYS = {
    'line': (('text',), ('tick', 'window', 'at')),
    'status': (('tick', 'fields'), ()),
    'no_line': (('window',), ('kinds', 'text')),
    'count': (('window', 'min', 'max'), ('kinds', 'text')),
    'pins': (('tick',), PIN_CHANNELS),
    'pins_window': (('window',), PIN_CHANNELS),
    'change': (('channel', 'tick', 'to'), ()),
    'first_change': (('channel', 'after', 'tick', 'to'), ()),
}
EXPECT_COMMON = ('type', 'id', 'spec', 'tol')
MACRO_RE = re.compile(r'\{(crc|boot|any)\}')
# Section 5.1: longest line per kind, LF included.
MAX_LINE_BYTES = {'BANNER': 174, 'CAL': 129, 'S': 85, 'N': 18, 'F': 35, 'W': 16, 'A': 26, 'P': 16}
N_REASONS = ('long', 'ctrl', 'crc', 'cmd', 'nocrc', 'state', 'syntax', 'range', 'boot', 'nocal', 'cal', 'pol', 'fault',
             'old', 'busy', 'wear', 'key', 'pin')
F_CODES = ('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'STALL', 'TRACK',
           'THERMAL', 'TTL', 'CRC')


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _need_int(value, where, lo=None, hi=None):
    if not _is_int(value):
        raise ScriptError(where, 'must be an integer, got %r' % (value,))
    if (lo is not None and value < lo) or (hi is not None and value > hi):
        raise ScriptError(where, 'must be in [%s, %s], got %d' % (lo, hi, value))
    return value


def _need_number(value, where):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ScriptError(where, 'must be a finite number, got %r' % (value,))
    return float(value)


def _need_bool(value, where):
    if not isinstance(value, bool):
        raise ScriptError(where, 'must be true or false, got %r' % (value,))
    return value


def _need_str(value, where):
    if not isinstance(value, str):
        raise ScriptError(where, 'must be a string, got %r' % (value,))
    return value


def _need_obj(value, where, required=(), optional=()):
    if not isinstance(value, dict):
        raise ScriptError(where, 'must be an object, got %s' % type(value).__name__)
    unknown = sorted(set(value) - set(required) - set(optional))
    if unknown:
        raise ScriptError(where, 'unknown key(s): %s' % ', '.join(unknown))
    missing = [key for key in required if key not in value]
    if missing:
        raise ScriptError(where, 'missing key(s): %s' % ', '.join(missing))
    return value


def _need_list(value, where):
    if not isinstance(value, list):
        raise ScriptError(where, 'must be an array, got %s' % type(value).__name__)
    return value


def _need_window(value, where, top):
    _need_list(value, where)
    if len(value) != 2:
        raise ScriptError(where, 'must be [t0, t1]')
    t0 = _need_int(value[0], where + '[0]', 0, top)
    t1 = _need_int(value[1], where + '[1]', 0, top)
    if t1 < t0:
        raise ScriptError(where, 'window end %d before start %d' % (t1, t0))
    return (t0, t1)


def _need_hex(value, where, max_chars=None):
    _need_str(value, where)
    if not HEX_RE.match(value):
        raise ScriptError(where, 'must be an even number of hex digits')
    if max_chars is not None and len(value) > max_chars:
        raise ScriptError(where, 'longer than %d hex digits' % max_chars)
    return value


class Comparator:
    """One value comparator of section 13.7. kind: int, range, any, sign, hex, mask."""

    def __init__(self, kind, a=None, b=None, text=None):
        self.kind, self.a, self.b, self.text = kind, a, b, text

    def matches(self, value):
        if self.kind == 'any':
            return True
        if self.kind in ('int', 'hex'):
            return value == self.a
        if self.kind == 'range':
            return (self.a is None or value >= self.a) and (self.b is None or value <= self.b)
        if self.kind == 'mask':
            return (value & self.a) == self.b
        if self.kind == 'sign':
            return {'pos': value > 0, 'neg': value < 0, 'zero': value == 0, 'nonzero': value != 0}[self.a]
        raise AssertionError(self.kind)

    def describe(self):
        return self.text


def _hex4(value, where):
    if not isinstance(value, str) or not re.match(r'^[0-9A-F]{4}$', value):
        raise ScriptError(where, 'must be 4 upper-case hex digits, got %r' % (value,))
    return int(value, 16)


def parse_comparator(value, where, mask_style=False, signs=False):
    """Comparator for a status field, a pin value or a change target. mask_style: fl / fault_bits forms."""
    if value == 'any':
        return Comparator('any', text='any')
    if mask_style:
        if isinstance(value, str):
            return Comparator('hex', _hex4(value, where), text=value)
        if isinstance(value, dict) and set(value) == {'mask', 'value'}:
            mask = _hex4(value['mask'], where + '.mask')
            want = _hex4(value['value'], where + '.value')
            if want & ~mask:
                raise ScriptError(where, 'value has bits outside the mask')
            return Comparator('mask', mask, want, text='mask %s value %s' % (value['mask'], value['value']))
        raise ScriptError(where, 'must be "any", 4 upper-case hex digits or {"mask", "value"}, got %r' % (value,))
    if signs and isinstance(value, str) and value in SIGN_WORDS:
        return Comparator('sign', value, text=value)
    if _is_int(value):
        return Comparator('int', value, text=str(value))
    if isinstance(value, dict) and value and set(value) <= {'min', 'max'}:
        lo = _need_int(value['min'], where + '.min') if 'min' in value else None
        hi = _need_int(value['max'], where + '.max') if 'max' in value else None
        if lo is not None and hi is not None and hi < lo:
            raise ScriptError(where, 'max below min')
        return Comparator('range', lo, hi, text='[%s, %s]' % ('' if lo is None else lo, '' if hi is None else hi))
    allowed = 'an integer, {"min", "max"}, "any"' + (' or ' + '/'.join(SIGN_WORDS) if signs else '')
    raise ScriptError(where, 'must be %s, got %r' % (allowed, value))


def _check_pattern(text, where, allow_crc=True):
    """Expectation text patterns: literal text with {crc} (after '*'), {boot} and {any}; no other braces."""
    _need_str(text, where)
    stripped = MACRO_RE.sub('', text)
    if '{' in stripped or '}' in stripped:
        raise ScriptError(where, 'unknown macro or stray brace in %r' % (text,))
    for match in MACRO_RE.finditer(text):
        if match.group(1) == 'crc':
            if not allow_crc:
                raise ScriptError(where, '{crc} is not allowed here')
            if match.start() == 0 or text[match.start() - 1] != '*':
                raise ScriptError(where, '{crc} must follow "*"')
    if '\n' in text or '\r' in text:
        raise ScriptError(where, 'a line pattern has no CR or LF')
    return text


def compile_pattern(text):
    """Regular expression for a line pattern; returns (regex, list of ('crc'|'boot', group name))."""
    regex, groups = [], []
    for index, part in enumerate(MACRO_RE.split(text)):
        if index % 2 == 0:
            regex.append(re.escape(part))
        elif part == 'any':
            regex.append(r'[^ *]+')
        else:
            name = '%s%d' % (part, len(groups))
            groups.append((part, name))
            regex.append('(?P<%s>[0-9A-F]{%d})' % (name, 2 if part == 'crc' else 8))
    return re.compile(''.join(regex)), groups


class BootPlan:
    """Resets, power-offs and setup() ticks of a script (section 10.1 boot ids, lockstep 11.5 events)."""

    def __init__(self, seed, boot_delay, events, last_tick):
        timeline = [(0, 0, 'reset')]
        offs = []
        for ev in events:
            if ev['kind'] in ('mcu_reset', 'port_open_reset'):
                timeline.append((ev['tick'], 0, 'reset'))
            elif ev['kind'] == 'power_cycle':
                timeline.append((ev['tick'], 0, 'off'))
                offs.append((ev['tick'], ev['tick'] + ev['off_ticks'] - 1))
                timeline.append((ev['tick'] + ev['off_ticks'], 0, 'reset'))
        for ev in events:
            for start, stop in offs:
                if ev['kind'] in RESET_EVENT_KINDS and start <= ev['tick'] <= stop and ev['kind'] != 'power_cycle':
                    raise ScriptError('events', 'a reset at tick %d falls inside a power-off (%d..%d)'
                                      % (ev['tick'], start, stop))
        resets = sorted((t for t, _, kind in timeline if kind in ('reset', 'off')))
        setups = []
        for t, _, kind in sorted(timeline):
            if kind != 'reset':
                continue
            setup = t + boot_delay
            later = [r for r in resets if t < r <= setup]
            if not later and setup <= last_tick:
                setups.append(setup)
        for setup in setups:
            timeline.append((setup, 1, 'setup'))
        self.timeline = sorted(timeline)
        self.setups = setups
        self.ids = boot_ids(seed, max(1, len(setups)))
        self.cuts = sorted(set(t for t, _, kind in timeline if kind in ('reset', 'off') and t > 0))

    def alive(self, tick):
        """Boot id of the boot running at `tick` (None during a boot delay or a power-off)."""
        last = None
        for t, order, kind in self.timeline:
            if t > tick:
                break
            last = kind
        if last != 'setup':
            return None
        count = sum(1 for s in self.setups if s <= tick)
        return self.ids[count - 1]


def expand_input_text(text, tick, plan, where):
    """Apply {boot} then {crc} (CRC-8 from the line start up to the '*') to a text input; returns bytes."""
    if '{boot}' in text:
        boot = plan.alive(tick)
        if boot is None:
            raise ScriptError(where, '{boot} at tick %d, where no boot is running' % tick)
        text = text.replace('{boot}', boot)
    out = []
    line_start = 0
    index = 0
    while index < len(text):
        if text.startswith('{crc}', index):
            body = ''.join(out)
            out.append(crc8_hex(body[line_start:-1]))
            index += 5
            continue
        char = text[index]
        out.append(char)
        if char == '\n':
            line_start = len(''.join(out))
        index += 1
    return ''.join(out).encode('ascii')


class Script:
    """A validated script. Attributes mirror the file with defaults filled in."""

    def __init__(self, data, name, path=None, sha256=None):
        self.raw = data
        self.name = name
        self.path = path
        self.sha256 = sha256
        _need_obj(data, 'script', TOP_REQUIRED, TOP_OPTIONAL)
        if data['schema'] != SCHEMA:
            raise ScriptError('schema', 'must be %r, got %r' % (SCHEMA, data['schema']))
        if not isinstance(data['name'], str) or not NAME_RE.match(data['name']):
            raise ScriptError('name', 'must match [a-z0-9_]{1,64}')
        if data['name'] != name:
            raise ScriptError('name', '%r differs from the file name %r' % (data['name'], name))
        spec = _need_list(data['spec'], 'spec')
        if not spec or not all(isinstance(item, str) and item for item in spec):
            raise ScriptError('spec', 'must be a non-empty array of section numbers')
        self.spec = list(spec)
        self.v2_map = _need_str(data['v2_map'], 'v2_map') if 'v2_map' in data else None
        self.description = _need_str(data['description'], 'description') if 'description' in data else ''
        self.end = self._end(data['end'])
        self.setup = self._setup(data['setup'])
        self.events = self._events(data.get('events', []))
        self.plan = BootPlan(self.setup['boot_seed'], self.setup['boot_delay_ticks'], self.events, self.total_ticks - 1)
        self.plant = self._plant(data.get('plant', {}))
        self.inputs = self._inputs(data['inputs'])
        self.expect = [self._expect(item, 'expect[%d]' % i, False) for i, item in enumerate(
            _need_list(data['expect'], 'expect'))]
        self.checks = [self._expect(item, 'end.checks[%d]' % i, True) for i, item in enumerate(self.end['checks'])]

    # -- sections
    def _end(self, end):
        _need_obj(end, 'end', ('tick',), ('drain_ticks', 'unexpected', 'ignore_kinds', 'checks'))
        out = dict(tick=_need_int(end['tick'], 'end.tick', 0, MAX_TICKS),
                   drain_ticks=_need_int(end.get('drain_ticks', 300), 'end.drain_ticks', 0, MAX_TICKS),
                   unexpected=end.get('unexpected', 'fail'), ignore_kinds=end.get('ignore_kinds', ['S']),
                   checks=_need_list(end.get('checks', []), 'end.checks'))
        if out['unexpected'] not in ('fail', 'ignore'):
            raise ScriptError('end.unexpected', 'must be "fail" or "ignore"')
        kinds = _need_list(out['ignore_kinds'], 'end.ignore_kinds')
        for kind in kinds:
            if kind not in LINE_KINDS:
                raise ScriptError('end.ignore_kinds', 'unknown line kind %r' % (kind,))
        out['ignore_kinds'] = tuple(kinds)
        self.total_ticks = out['tick'] + 1 + out['drain_ticks']
        if self.total_ticks > MAX_TICKS:
            raise ScriptError('end', 'end.tick + drain_ticks above %d' % MAX_TICKS)
        return out

    def _setup(self, setup):
        _need_obj(setup, 'setup', SETUP_REQUIRED, tuple(SETUP_DEFAULTS))
        out = dict(SETUP_DEFAULTS)
        out.update(setup)
        if out['mcu_tick_hz'] != MCU_TICK_HZ or not _is_int(out['mcu_tick_hz']):
            raise ScriptError('setup.mcu_tick_hz', 'only 1000 is allowed')
        _need_int(out['boot_seed'], 'setup.boot_seed', 0, 0xFFFFFFFFFFFFFFFF)
        if not isinstance(out['chip_id'], str) or not re.match(r'^[0-9A-F]{6}$', out['chip_id']):
            raise ScriptError('setup.chip_id', 'must be 6 upper-case hex digits')
        _need_int(out['boot_delay_ticks'], 'setup.boot_delay_ticks', 0, 1000)
        _need_int(out['millis_start'], 'setup.millis_start', 0, 0xFFFFFFFF)
        out['eeprom'] = self._eeprom(out['eeprom'])
        adc = out['adc']
        if isinstance(adc, dict) and adc.get('mode') == 'model':
            _need_obj(adc, 'setup.adc', ('mode',))
        else:
            _need_obj(adc, 'setup.adc', ('mode', 'raw'))
            if adc['mode'] != 'script':
                raise ScriptError('setup.adc.mode', 'must be "script" or "model"')
            _need_int(adc['raw'], 'setup.adc.raw', 0, 1023)
        _need_bool(out['report_adc'], 'setup.report_adc')
        _need_number(out['initial_delta_eq_deg'], 'setup.initial_delta_eq_deg')
        if not isinstance(out['profile'], str) or not NAME_RE.match(out['profile']):
            raise ScriptError('setup.profile', 'must be a profile file name without .yaml')
        _need_int(out['seed'], 'setup.seed', 0, 0xFFFFFFFFFFFFFFFF)
        if not isinstance(out['startup_overrides'], dict):
            raise ScriptError('setup.startup_overrides', 'must be an object (JSON merge patch)')
        for i, item in enumerate(_need_list(out['injections'], 'setup.injections')):
            _need_obj(item, 'setup.injections[%d]' % i, ('kind', 'at_tick', 'params'))
            _need_str(item['kind'], 'setup.injections[%d].kind' % i)
            _need_int(item['at_tick'], 'setup.injections[%d].at_tick' % i, 0, MAX_TICKS)
            if not isinstance(item['params'], dict):
                raise ScriptError('setup.injections[%d].params' % i, 'must be an object')
        _need_bool(out['motor_power'], 'setup.motor_power')
        _need_hex(out['boot_garbage_hex'], 'setup.boot_garbage_hex', 512)
        return out

    def _eeprom(self, spec):
        where = 'setup.eeprom'
        if spec == 'blank':
            return spec
        _need_obj(spec, where, (), ('record', 'hex'))
        if len(spec) != 1:
            raise ScriptError(where, 'needs exactly one of "record" or "hex"')
        if 'hex' in spec:
            _need_hex(spec['hex'], where + '.hex')
            if len(spec['hex']) != 512:
                raise ScriptError(where + '.hex', 'must have exactly 512 hex digits')
            return {'hex': spec['hex']}
        rec = _need_obj(spec['record'], where + '.record', RECORD_KEYS, ('params', 'magic'))
        limits = dict(cal_id=(0, 65535), raw_l=(0, 65535), raw_c=(0, 65535), raw_r=(0, 65535),
                      soft_l=(-32768, 32767), soft_r=(-32768, 32767), pol_ok=(0, 255))
        for key, (lo, hi) in limits.items():
            _need_int(rec[key], '%s.record.%s' % (where, key), lo, hi)
        if rec['mdir'] not in (1, -1) or not _is_int(rec['mdir']):
            raise ScriptError(where + '.record.mdir', 'must be 1 or -1')
        params = _need_obj(rec.get('params', {}), where + '.record.params', (), CFG_KEYS)
        for key, value in params.items():
            _need_int(value, '%s.record.params.%s' % (where, key), 0, 65535)
        magic = rec.get('magic', 'USC3')
        if magic not in RECORD_MAGICS:
            raise ScriptError(where + '.record.magic', 'must be "USC3" or "USCX"')
        return {'record': dict(rec, params=dict(params), magic=magic)}

    def _plant(self, plant):
        _need_obj(plant, 'plant', (), ('speed_m_s', 'stopped', 'supply_scale'))
        top = self.end['tick']

        def table(key, default, check):
            rows = _need_list(plant.get(key, default), 'plant.' + key)
            out, last = [], -1
            for i, row in enumerate(rows):
                where = 'plant.%s[%d]' % (key, i)
                if not isinstance(row, list) or len(row) != 2:
                    raise ScriptError(where, 'must be [tick, value]')
                tick = _need_int(row[0], where + '[0]', 0, top)
                if tick <= last:
                    raise ScriptError(where, 'ticks must increase')
                if i == 0 and tick != 0:
                    raise ScriptError(where, 'the first row must be at tick 0')
                out.append((tick, check(row[1], where + '[1]')))
                last = tick
            if not out:
                raise ScriptError('plant.' + key, 'needs at least one row')
            return out

        def supply(value, where):
            value = _need_number(value, where)
            if not 0. < value <= 2.:
                raise ScriptError(where, 'supply scale must be in (0, 2]')
            return value

        speed = table('speed_m_s', [[0, 0.0]], _need_number)
        if 'stopped' in plant:
            stopped = table('stopped', None, _need_bool)
        else:
            stopped = [(tick, value == 0.) for tick, value in speed]
        return dict(speed=speed, stopped=stopped, supply=table('supply_scale', [[0, 1.0]], supply))

    def _events(self, events):
        out, last = [], -1
        top = self.end['tick']
        for i, ev in enumerate(_need_list(events, 'events')):
            where = 'events[%d]' % i
            if not isinstance(ev, dict) or ev.get('kind') not in EVENT_KINDS:
                raise ScriptError(where, 'needs "kind", one of %s' % ', '.join(EVENT_KINDS))
            kind = ev['kind']
            extra = {'mcu_reset': ('reason',), 'power_cycle': ('off_ticks',), 'port_open_reset': (),
                     'motor_power': ('on',), 'mcu_stall': ('ticks',), 'adc_set': ('raw',), 'adc_model': ()}[kind]
            _need_obj(ev, where, ('tick', 'kind') + extra)
            tick = _need_int(ev['tick'], where + '.tick', 0, top)
            if tick < last:
                raise ScriptError(where, 'events must be in tick order')
            last = tick
            item = dict(tick=tick, kind=kind)
            if kind == 'mcu_reset':
                item['arg'] = item['reason'] = _need_int(ev['reason'], where + '.reason', 1, 6)
            elif kind == 'power_cycle':
                item['arg'] = item['off_ticks'] = _need_int(ev['off_ticks'], where + '.off_ticks', 1, 60000)
            elif kind == 'motor_power':
                item['arg'] = int(_need_bool(ev['on'], where + '.on'))
            elif kind == 'mcu_stall':
                item['arg'] = _need_int(ev['ticks'], where + '.ticks', 1, 60000)
            elif kind == 'adc_set':
                item['arg'] = _need_int(ev['raw'], where + '.raw', 0, 1023)
            else:
                item['arg'] = 0
            out.append(item)
        per_tick = {}
        for item in out:
            per_tick[item['tick']] = per_tick.get(item['tick'], 0) + 1
        if per_tick and max(per_tick.values()) > 64:
            raise ScriptError('events', 'more than 64 events in one tick')
        return out

    def _inputs(self, inputs):
        out, last = [], -1
        for i, item in enumerate(_need_list(inputs, 'inputs')):
            where = 'inputs[%d]' % i
            if not isinstance(item, dict):
                raise ScriptError(where, 'must be an object')
            if ('text' in item) == ('hex' in item):
                raise ScriptError(where, 'needs exactly one of "text" or "hex"')
            _need_obj(item, where, ('tick',), ('text', 'hex', 'pace'))
            tick = _need_int(item['tick'], where + '.tick', 0, self.end['tick'])
            if tick < last:
                raise ScriptError(where, 'inputs must be in tick order')
            last = tick
            pace = item.get('pace', 'instant')
            if pace not in ('instant', 'uart'):
                raise ScriptError(where + '.pace', 'must be "instant" or "uart"')
            if 'hex' in item:
                data = bytes.fromhex(_need_hex(item['hex'], where + '.hex'))
            else:
                text = _need_str(item['text'], where + '.text')
                if any(ord(ch) > 0x7F for ch in text):
                    raise ScriptError(where + '.text', 'only ASCII; write bytes >= 0x80 with "hex"')
                stripped = re.sub(r'\{(crc|boot)\}', '', text)
                if '{' in stripped or '}' in stripped:
                    raise ScriptError(where + '.text', 'unknown macro or stray brace')
                for match in re.finditer(r'\{crc\}', text):
                    if match.start() == 0 or text[match.start() - 1] != '*':
                        raise ScriptError(where + '.text', '{crc} must follow "*"')
                data = expand_input_text(text, tick, self.plan, where + '.text')
            if not data:
                raise ScriptError(where, 'no bytes')
            out.append(dict(tick=tick, data=data, pace=pace))
        return out

    def _expect(self, item, where, in_end):
        if not isinstance(item, dict) or item.get('type') not in EXPECT_KEYS:
            raise ScriptError(where, 'needs "type", one of %s' % ', '.join(EXPECT_KEYS))
        kind = item['type']
        required, optional = EXPECT_KEYS[kind]
        if in_end and kind in ('line', 'no_line', 'count', 'pins_window'):
            optional = tuple(optional) + tuple(k for k in ('window',) if k in required)
            required = tuple(k for k in required if k != 'window')
        _need_obj(item, where, ('type',) + tuple(required), EXPECT_COMMON + tuple(optional))
        top = self.end['tick']
        out = dict(type=kind, id=item.get('id') or where, spec=item.get('spec'), tol=item.get('tol', 0), where=where)
        if 'id' in item:
            _need_str(item['id'], where + '.id')
        if 'spec' in item and not (isinstance(item['spec'], str) or (
                isinstance(item['spec'], list) and all(isinstance(x, str) for x in item['spec']))):
            raise ScriptError(where + '.spec', 'must be a section string or an array of them')
        tol = _need_int(out['tol'], where + '.tol', 0, 1)
        if kind == 'line':
            out['text'] = _check_pattern(item['text'], where + '.text')
            out['regex'], out['groups'] = compile_pattern(out['text'])
            out['at'] = item.get('at', 'queued')
            if out['at'] not in ('queued', 'done'):
                raise ScriptError(where + '.at', 'must be "queued" or "done"')
            limit = top + (self.end['drain_ticks'] if out['at'] == 'done' else 0)
            if 'tick' in item and 'window' in item:
                raise ScriptError(where, 'give "tick" or "window", not both')
            if 'tick' in item:
                out['tick'] = _need_int(item['tick'], where + '.tick', 0, limit)
            elif 'window' in item:
                out['window'] = _need_window(item['window'], where + '.window', limit)
            elif in_end:
                out['window'] = (0, limit)
            else:
                raise ScriptError(where, 'needs "tick" or "window"')
            if tol and not (out['text'].startswith('F ') or out['text'].startswith('W ')):
                raise ScriptError(where + '.tol', 'tol 1 only for F and W lines (fault decision times)')
            if tol and 'window' in out:
                raise ScriptError(where + '.tol', 'tol needs "tick", not "window"')
        elif kind == 'status':
            out['tick'] = _need_int(item['tick'], where + '.tick', 0, top)
            fields = _need_obj(item['fields'], where + '.fields', (), STATUS_FIELDS)
            if not fields:
                raise ScriptError(where + '.fields', 'needs at least one field')
            out['fields'] = {key: parse_comparator(value, '%s.fields.%s' % (where, key), mask_style=key == 'fl')
                             for key, value in fields.items()}
        elif kind in ('no_line', 'count'):
            out['window'] = (_need_window(item['window'], where + '.window', top) if 'window' in item
                             else (0, top))
            if ('kinds' in item) == ('text' in item):
                raise ScriptError(where, 'needs exactly one of "kinds" or "text"')
            if 'kinds' in item:
                kinds = _need_list(item['kinds'], where + '.kinds')
                if not kinds or any(k not in LINE_KINDS for k in kinds):
                    raise ScriptError(where + '.kinds', 'must be a non-empty array of %s' % ', '.join(LINE_KINDS))
                out['kinds'] = tuple(kinds)
            else:
                out['text'] = _check_pattern(item['text'], where + '.text')
                out['regex'], out['groups'] = compile_pattern(out['text'])
            if kind == 'count':
                out['min'] = _need_int(item['min'], where + '.min', 0)
                out['max'] = _need_int(item['max'], where + '.max', 0)
                if out['max'] < out['min']:
                    raise ScriptError(where, 'max below min')
        elif kind in ('pins', 'pins_window'):
            if kind == 'pins':
                out['tick'] = _need_int(item['tick'], where + '.tick', 0, top)
            else:
                out['window'] = (_need_window(item['window'], where + '.window', top) if 'window' in item
                                 else (0, top))
            pins = {key: item[key] for key in PIN_CHANNELS if key in item}
            if not pins:
                raise ScriptError(where, 'needs at least one pin field (%s)' % ', '.join(PIN_CHANNELS))
            out['pins'] = {key: self._channel_comparator(key, value, '%s.%s' % (where, key))
                           for key, value in pins.items()}
        else:
            channel = item['channel']
            if channel not in CHANNELS:
                raise ScriptError(where + '.channel', 'unknown channel %r' % (channel,))
            if channel == 'adc_read' and not self.setup['report_adc']:
                raise ScriptError(where + '.channel', 'adc_read needs setup.report_adc true')
            out['channel'] = channel
            out['tick'] = _need_int(item['tick'], where + '.tick', 0, top)
            out['to'] = self._channel_comparator(channel, item['to'], where + '.to')
            if kind == 'first_change':
                out['after'] = _need_int(item['after'], where + '.after', 0, top)
                if out['after'] > out['tick']:
                    raise ScriptError(where, '"after" is later than "tick"')
            if tol and channel != 'fault_bits':
                raise ScriptError(where + '.tol', 'tol 1 only for fault_bits changes (fault decision times)')
        if tol and kind not in ('line', 'change', 'first_change'):
            raise ScriptError(where + '.tol', 'tol is only for F/W line and fault_bits change expectations')
        out['tol'] = tol
        return out

    @staticmethod
    def _channel_comparator(channel, value, where):
        if channel == 'fault_bits':
            return parse_comparator(value, where, mask_style=True)
        return parse_comparator(value, where, signs=True)


def load_script(path):
    """Read and validate one script file; raises ScriptError (with the file name) or OSError."""
    path = Path(path)
    data = path.read_bytes()
    where = path.name
    if data.startswith(b'\xef\xbb\xbf'):
        raise ScriptError(where, 'UTF-8 byte order mark is not allowed')
    if b'\r' in data:
        raise ScriptError(where, 'line ends must be LF only')
    try:
        obj = json.loads(data.decode('utf-8'), object_pairs_hook=_no_duplicates)
    except (UnicodeDecodeError, ValueError) as exc:
        raise ScriptError(where, 'not valid UTF-8 JSON: %s' % exc) from None
    try:
        return Script(obj, path.stem, path=str(path), sha256=hashlib.sha256(data).hexdigest())
    except ScriptError as exc:
        raise ScriptError('%s: %s' % (where, exc.where), str(exc).split(': ', 1)[1]) from None


def _no_duplicates(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise ValueError('duplicate key %r' % key)
        out[key] = value
    return out


def script_paths(scripts_dir=SCRIPTS_DIR):
    return sorted(Path(scripts_dir).glob('*.json'))


def load_scripts(names=(), scripts_dir=SCRIPTS_DIR):
    """Validated scripts selected by exact names or glob patterns (all when `names` is empty)."""
    paths = script_paths(scripts_dir)
    if names:
        chosen = []
        for name in names:
            hits = [p for p in paths if fnmatch.fnmatchcase(p.stem, name)]
            if not hits:
                raise ScriptError(name, 'no script matches')
            chosen.extend(p for p in hits if p not in chosen)
        paths = chosen
    return [load_script(path) for path in paths]


# --- lockstep v2 firmware transport -----------------------------------------------------------------------------------
class TransportError(RuntimeError):
    """The native process broke the lockstep v2 contract or failed (exit code and stderr line when known)."""

    def __init__(self, message, exit_code=None, stderr_line='', unavailable=False):
        super().__init__(message)
        self.exit_code = exit_code
        self.stderr_line = stderr_line
        self.unavailable = unavailable


WIRE_MAGIC = b'USTP'
WIRE_VERSION = 2
FRAME_RESET_REQUEST, FRAME_FW_STEP_REQUEST = 0x0002, 0x0003
FRAME_READY, FRAME_RESET_RESPONSE, FRAME_FW_STEP_RESPONSE = 0x8000, 0x8002, 0x8003
HEADER = struct.Struct('<4sHHII')
FW_STEP_REQUEST = struct.Struct('<dddBBHI')        # 32 B
EVENT_ITEM = struct.Struct('<HHI')                 # 8 B
RX_ITEM = struct.Struct('<HBB')                    # 4 B
BASE_BLOCK = struct.Struct('<ddddddddIHHHBBI')     # 80 B (v1 response block)
FW_TAIL = struct.Struct('<HHBBHII')                # 16 B
CHANGE_ITEM = struct.Struct('<HBBi')               # 8 B
TX_ITEM = struct.Struct('<IIBBH')                  # 12 B
READY_BLOCK = struct.Struct('<II')
BASE_FIELDS = ('sim_time_s', 'delta_left_rad', 'delta_right_rad', 'delta_eq_rad', 'motor_side_angle_rad',
               'applied_steer_pwm', 'drive_duty_left', 'drive_duty_right', 'ticks', 'pot_raw', 'fault_bits',
               'model_flags', 'steer_en', 'drive_en', 'serial_out_len')
COMPARED_BASE_FIELDS = tuple(key for key in BASE_FIELDS if key not in ('ticks', 'serial_out_len'))
MAX_ITEMS = 4096
MAX_EVENTS = 64
MAX_FRAME_PAYLOAD = FW_STEP_REQUEST.size + BASE_BLOCK.size + FW_TAIL.size + MAX_ITEMS * (CHANGE_ITEM.size
                                                                                       + TX_ITEM.size)
EXIT_NAMES = {0: 'ok', 2: 'bad_startup', 3: 'bad_header', 4: 'sequence_mismatch', 5: 'bad_length', 6: 'bad_value',
              7: 'truncated_frame', 8: 'io_error', 9: 'internal_error'}


class FwStep:
    """One FW_STEP_RESPONSE: base block (dict), channel changes, MCU bytes and the run fields."""

    __slots__ = ('first_tick', 'ticks', 'base', 'changes', 'tx', 'mcu_state', 'run_flags', 'boot_count',
                 'mcu_millis', 'rx_lost_total')

    def __init__(self, first_tick, ticks, base, changes, tx, mcu_state, run_flags, boot_count, mcu_millis,
                 rx_lost_total):
        self.first_tick, self.ticks, self.base, self.changes, self.tx = first_tick, ticks, base, changes, tx
        self.mcu_state, self.run_flags, self.boot_count = mcu_state, run_flags, boot_count
        self.mcu_millis, self.rx_lost_total = mcu_millis, rx_lost_total


class FirmwareTransport:
    """Interface the runner drives (lockstep v2 firmware mode semantics).

    ready        dict(build_id, fw_build, executable, executable_sha256)
    reset(delta_eq_rad)                                   RESET_REQUEST; MCU state as after READY
    step(first_tick, ticks, speed, supply, stopped, events, rx) -> FwStep
                 events: [(tick_offset, kind_code, arg)], rx: [(tick_offset, byte)]
    close()
    """

    ready = None

    def reset(self, delta_eq_rad):
        raise NotImplementedError

    def step(self, first_tick, ticks, speed, supply, stopped, events, rx):
        raise NotImplementedError

    def close(self):
        pass


class NativeFirmwareTransport(FirmwareTransport):
    """The native lockstep program in firmware mode, codec written from LOCKSTEP_PROTOCOL_KO.md section 11."""

    def __init__(self, executable, startup, timeout_s=30.):
        self.executable = str(executable)
        self.timeout_s = float(timeout_s)
        self._seq = 0
        self._plant_time = 0.
        self._ticks_total = 0
        self._stderr = []
        self._deadline = None
        self._killed = False
        self._lock = threading.Lock()
        try:
            self.proc = subprocess.Popen([self.executable], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=subprocess.PIPE)
        except OSError as exc:
            raise TransportError('cannot start %s: %s' % (self.executable, exc), unavailable=True) from None
        self._stderr_thread = threading.Thread(target=self._drain_stderr, daemon=True)
        self._stderr_thread.start()
        self._watch = threading.Thread(target=self._watchdog, daemon=True)
        self._stop = threading.Event()
        self._watch.start()
        line = json.dumps(startup, ensure_ascii=True, allow_nan=False, separators=(',', ':')).encode('ascii') + b'\n'
        if len(line) > 65536:
            raise TransportError('startup line longer than 64 KiB')
        self._write(line)
        frame_type, seq, payload = self._read_frame()
        if frame_type != FRAME_READY or seq != 0:
            self._fail('expected READY seq 0, got type 0x%04x seq %d' % (frame_type, seq))
        if len(payload) < READY_BLOCK.size:
            self._fail('READY payload too short')
        tick_hz, length = READY_BLOCK.unpack_from(payload)
        if tick_hz != MCU_TICK_HZ or len(payload) != READY_BLOCK.size + length or length > 256:
            self._fail('READY tick %d build id length %d payload %d' % (tick_hz, length, len(payload)))
        build_id = payload[READY_BLOCK.size:].decode('ascii', 'replace')
        match = re.search(r'\bfw=([0-9A-F]{8})\b', build_id)
        self.ready = dict(build_id=build_id, fw_build=match.group(1) if match else None, executable=self.executable,
                          executable_sha256=_file_sha256(self.executable))

    # -- process plumbing
    def _drain_stderr(self):
        try:
            for raw in iter(self.proc.stderr.readline, b''):
                if len(self._stderr) < 200:
                    self._stderr.append(raw.decode('utf-8', 'replace').rstrip())
        except (OSError, ValueError):
            pass

    def _watchdog(self):
        while not self._stop.wait(0.2):
            deadline = self._deadline
            if deadline is not None and time.monotonic() > deadline:
                self._killed = True
                try:
                    self.proc.kill()
                except OSError:
                    pass
                return

    def _arm(self):
        self._deadline = time.monotonic() + self.timeout_s

    def _disarm(self):
        self._deadline = None

    def _write(self, data):
        self._arm()
        try:
            self.proc.stdin.write(data)
            self.proc.stdin.flush()
        except (OSError, ValueError) as exc:
            self._fail('write failed: %s' % exc)
        finally:
            self._disarm()

    def _read_exact(self, size):
        chunks, have = [], 0
        while have < size:
            chunk = self.proc.stdout.read(size - have)
            if not chunk:
                return None if have == 0 and size == HEADER.size else b''.join(chunks)
            chunks.append(chunk)
            have += len(chunk)
        return b''.join(chunks)

    def _read_frame(self):
        self._arm()
        try:
            header = self._read_exact(HEADER.size)
            if header is None or len(header) < HEADER.size:
                self._fail('no frame (EOF)')
            magic, version, frame_type, seq, length = HEADER.unpack(header)
            if magic != WIRE_MAGIC or version != WIRE_VERSION:
                self._fail('bad header magic %r version %d' % (magic, version))
            if length > MAX_FRAME_PAYLOAD:
                self._fail('payload length %d too large' % length)
            payload = self._read_exact(length) if length else b''
            if payload is None or len(payload) != length:
                self._fail('truncated payload')
            return frame_type, seq, payload
        finally:
            self._disarm()

    def _exit_info(self):
        code = None
        try:
            code = self.proc.wait(timeout=2.)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            code = self.proc.wait()
        self._stderr_thread.join(timeout=2.)
        line = self._stderr[-1] if self._stderr else ''
        return code, line

    def _fail(self, message):
        self._stop.set()
        try:
            self.proc.stdin.close()
        except (OSError, ValueError):
            pass
        code, line = self._exit_info()
        for stream in (self.proc.stdout, self.proc.stderr):
            try:
                stream.close()
            except (OSError, ValueError):
                pass
        killed = ' (killed: no response within %.0f s)' % self.timeout_s if self._killed else ''
        unavailable = self.ready is None and code == 2
        raise TransportError('native firmware process: %s; exit %s (%s)%s; stderr: %s' % (
            message, code, EXIT_NAMES.get(code, '?'), killed, line or '(none)'), exit_code=code, stderr_line=line,
            unavailable=unavailable)

    # -- frames
    def reset(self, delta_eq_rad):
        self._seq += 1
        payload = struct.pack('<d', float(delta_eq_rad))
        self._write(HEADER.pack(WIRE_MAGIC, WIRE_VERSION, FRAME_RESET_REQUEST, self._seq, len(payload)) + payload)
        frame_type, seq, payload = self._read_frame()
        if frame_type != FRAME_RESET_RESPONSE or seq != self._seq or len(payload) != BASE_BLOCK.size:
            self._fail('bad RESET_RESPONSE type 0x%04x seq %d length %d' % (frame_type, seq, len(payload)))
        base = dict(zip(BASE_FIELDS, BASE_BLOCK.unpack(payload)))
        if base['ticks'] or base['fault_bits'] or base['serial_out_len'] or base['steer_en'] or base['drive_en']:
            self._fail('RESET_RESPONSE must have ticks 0, outputs 0, fault_bits 0, serial_out_len 0: %r' % base)
        self._plant_time = 0.
        self._ticks_total = 0
        return base

    def step(self, first_tick, ticks, speed, supply, stopped, events, rx):
        if first_tick != self._ticks_total:
            raise TransportError('step starts at tick %d, native is at %d' % (first_tick, self._ticks_total))
        dt = ticks / 1000.
        self._plant_time += dt
        target = int(math.floor(self._plant_time * MCU_TICK_HZ + .5))
        if target - self._ticks_total != ticks:
            raise TransportError('dt %r gives %d ticks, wanted %d' % (dt, target - self._ticks_total, ticks))
        self._seq += 1
        body = [FW_STEP_REQUEST.pack(dt, float(speed), float(supply), int(bool(stopped)), 0, len(events), len(rx))]
        body.extend(EVENT_ITEM.pack(offset, kind, arg) for offset, kind, arg in events)
        body.extend(RX_ITEM.pack(offset, byte, 0) for offset, byte in rx)
        payload = b''.join(body)
        self._write(HEADER.pack(WIRE_MAGIC, WIRE_VERSION, FRAME_FW_STEP_REQUEST, self._seq, len(payload)) + payload)
        frame_type, seq, payload = self._read_frame()
        if frame_type != FRAME_FW_STEP_RESPONSE or seq != self._seq:
            self._fail('expected FW_STEP_RESPONSE seq %d, got type 0x%04x seq %d' % (self._seq, frame_type, seq))
        step = decode_fw_step(payload, first_tick, ticks)
        self._ticks_total = target
        return step

    def close(self):
        self._stop.set()
        if self.proc.poll() is None:
            try:
                self.proc.stdin.close()
            except (OSError, ValueError):
                pass
            try:
                self.proc.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        for stream in (self.proc.stdout, self.proc.stderr):
            try:
                stream.close()
            except (OSError, ValueError):
                pass


def decode_fw_step(payload, first_tick, ticks):
    """FwStep from a FW_STEP_RESPONSE payload; raises TransportError when it breaks section 11.6."""
    if len(payload) < BASE_BLOCK.size + FW_TAIL.size:
        raise TransportError('FW_STEP_RESPONSE payload %d bytes, needs at least 96' % len(payload))
    base = dict(zip(BASE_FIELDS, BASE_BLOCK.unpack_from(payload, 0)))
    count_c, count_t, mcu_state, run_flags, boot_count, mcu_millis, lost = FW_TAIL.unpack_from(payload, 80)
    if len(payload) != 96 + CHANGE_ITEM.size * count_c + TX_ITEM.size * count_t:
        raise TransportError('FW_STEP_RESPONSE length %d does not match %d changes and %d tx bytes'
                             % (len(payload), count_c, count_t))
    if base['ticks'] != ticks:
        raise TransportError('response ran %d ticks, the request needs %d' % (base['ticks'], ticks))
    if base['serial_out_len'] != 0:
        raise TransportError('serial_out_len must be 0 in firmware mode')
    if mcu_state not in (0, 1, 2, 3, 255) or run_flags & ~0x0F:
        raise TransportError('mcu_state %d run_flags 0x%02x' % (mcu_state, run_flags))
    changes, offset, last = [], 96, (-1, 0)
    for _ in range(count_c):
        tick_offset, channel, reserved, value = CHANGE_ITEM.unpack_from(payload, offset)
        offset += CHANGE_ITEM.size
        if reserved or channel not in CHANNEL_NAMES or tick_offset >= ticks or (tick_offset, channel) <= last:
            raise TransportError('bad change entry offset %d channel %d reserved %d' % (tick_offset, channel,
                                                                                       reserved))
        last = (tick_offset, channel)
        name = CHANNEL_NAMES[channel]
        if name in UNSIGNED_CHANNELS:
            value &= 0xFFFFFFFF
        changes.append((first_tick + tick_offset, name, value))
    tx, last_done = [], -1
    for _ in range(count_t):
        queued, done, byte, flags, reserved = TX_ITEM.unpack_from(payload, offset)
        offset += TX_ITEM.size
        if reserved or flags & ~1 or not first_tick <= done < first_tick + ticks or queued > done or done < last_done:
            raise TransportError('bad tx entry queued %d done %d flags %d reserved %d' % (queued, done, flags,
                                                                                          reserved))
        last_done = done
        tx.append((queued, done, byte, flags))
    return FwStep(first_tick, ticks, base, changes, tx, mcu_state, run_flags, boot_count, mcu_millis, lost)


def _file_sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def startup_config(script, expect_build=None):
    """The lockstep v2 firmware-mode startup object for a script (profile values from the steering package)."""
    from .. import backend_api as ba
    from .. import vehicle_profile as vp
    setup = script.setup
    path = Path(vp.__file__).resolve().parent / 'profiles' / (setup['profile'] + '.yaml')
    if not path.is_file():
        raise ScriptError('setup.profile', 'no profile file %s' % path.name)
    profile = vp.load_profile(path)
    injections = [dict(kind=item['kind'], at_s=item['at_tick'] / 1000., params=item['params'])
                  for item in setup['injections']]
    try:
        injections = ba.validate_injections(injections)
    except ValueError as exc:
        raise ScriptError('setup.injections', str(exc)) from None
    config = ba.native_startup_config(profile, setup['seed'], mode='firmware', mcu_tick_hz=MCU_TICK_HZ,
                                      injections=injections)
    adc = setup['adc']
    config['protocol'] = LOCKSTEP_PROTOCOL
    config['firmware'] = dict(
        expect_build=expect_build, boot_seed=setup['boot_seed'], chip_id=int(setup['chip_id'], 16),
        eeprom_hex=None if setup['eeprom'] == 'blank' else eeprom_image(setup['eeprom']).hex().upper(),
        initial_delta_eq_rad=math.radians(setup['initial_delta_eq_deg']),
        boot_delay_ticks=setup['boot_delay_ticks'], millis_start=setup['millis_start'],
        boot_garbage_hex=setup['boot_garbage_hex'].upper(), adc_mode=adc['mode'],
        adc_script_raw=adc.get('raw', 512), motor_power=setup['motor_power'], report_adc=setup['report_adc'],
        report_servo=False)   # servo evaluation channels (LOCKSTEP 11.6) are not part of a bench script
    return merge_patch(config, setup['startup_overrides'])


def merge_patch(target, patch):
    """RFC 7386 JSON merge patch."""
    if not isinstance(patch, dict):
        return json.loads(json.dumps(patch))
    result = dict(target) if isinstance(target, dict) else {}
    for key, value in patch.items():
        if value is None:
            result.pop(key, None)
        else:
            result[key] = merge_patch(result.get(key), value)
    return result


def resolve_executable(executable=None, compiler=None, build_dir=None):
    """Path of the native program: the given file, else steering.native.build.ensure_built()."""
    if executable:
        if not Path(executable).is_file():
            raise TransportError('executable not found: %s' % executable, unavailable=True)
        return str(executable)
    from ..native import build as native_build
    try:
        return native_build.ensure_built(compiler=compiler, build_dir=build_dir).path
    except native_build.NativeBuildError as exc:
        raise TransportError('native build failed: %s' % exc, unavailable=True) from None


def probe_firmware_mode(executable, timeout_s=30.):
    """(True, build_id) if the native program accepts a firmware-mode startup line, else (False, reason)."""
    data = dict(schema=SCHEMA, name='probe', spec=['11.2'], setup=dict(boot_seed=1, chip_id='000001'), inputs=[],
                expect=[], end=dict(tick=0, drain_ticks=0))
    try:
        transport = NativeFirmwareTransport(executable, startup_config(Script(data, 'probe')), timeout_s)
    except TransportError as exc:
        return False, str(exc)
    try:
        return True, transport.ready['build_id']
    finally:
        transport.close()


# --- execution ------------------------------------------------------------------------------------------------------
class Trace:
    """Everything one run produced: MCU bytes, channel changes, base blocks at step ends."""

    def __init__(self):
        self.tx = []
        self.changes = []
        self.step_ends = {}
        self.steps = 0
        self.final = None


def _uart_byte_units(baud):
    return 5760000 // baud


def schedule_inputs(script, baud=115200):
    """{tick: [bytes in arrival order]} from the script inputs (instant at the tick, uart per lockstep 11.7)."""
    per_byte = _uart_byte_units(baud)
    arrivals = []
    chain = 0
    for index, item in enumerate(script.inputs):
        if item['pace'] == 'instant':
            arrivals.extend((item['tick'], index, k, byte) for k, byte in enumerate(item['data']))
            continue
        for k, byte in enumerate(item['data']):
            chain = max(item['tick'] * 576, chain) + per_byte
            arrivals.append(((chain + 575) // 576, index, k, byte))
    arrivals.sort()
    per_tick = {}
    for tick, _, _, byte in arrivals:
        per_tick.setdefault(tick, []).append(byte)
    if per_tick and max(len(v) for v in per_tick.values()) > MAX_ITEMS:
        raise ScriptError('inputs', 'more than %d bytes arrive in one tick' % MAX_ITEMS)
    return per_tick


def _value_at(table, tick):
    value = table[0][1]
    for start, item in table:
        if start > tick:
            break
        value = item
    return value


def execute(script, transport, step_ticks, rx_schedule, reset_first=False):
    """Run the script's ticks through the transport; returns a Trace."""
    trace = Trace()
    if reset_first:
        transport.reset(math.radians(script.setup['initial_delta_eq_deg']))
    plant = script.plant
    boundaries = sorted(set(t for key in ('speed', 'stopped', 'supply') for t, _ in plant[key]) | {0})
    events = {}
    for ev in script.events:
        events.setdefault(ev['tick'], []).append((EVENT_KINDS[ev['kind']], ev['arg']))
    total = script.total_ticks
    tick = 0
    while tick < total:
        next_boundary = min([b for b in boundaries if b > tick] + [total])
        n = min(step_ticks, next_boundary - tick)
        while n > 1:
            count_e = sum(len(events.get(t, ())) for t in range(tick, tick + n))
            count_r = sum(len(rx_schedule.get(t, ())) for t in range(tick, tick + n))
            if count_e <= MAX_EVENTS and count_r <= MAX_ITEMS:
                break
            n = max(1, n // 2)
        step_events = [(t - tick, kind, arg) for t in range(tick, tick + n) for kind, arg in events.get(t, ())]
        step_rx = [(t - tick, byte) for t in range(tick, tick + n) for byte in rx_schedule.get(t, ())]
        speed = _value_at(plant['speed'], tick)
        supply = _value_at(plant['supply'], tick)
        stopped = _value_at(plant['stopped'], tick)
        result = transport.step(tick, n, speed, supply, stopped, step_events, step_rx)
        trace.tx.extend(result.tx)
        trace.changes.extend(result.changes)
        trace.step_ends[tick + n - 1] = tuple(result.base[key] for key in COMPARED_BASE_FIELDS) + (
            result.mcu_state, result.run_flags, result.boot_count, result.mcu_millis, result.rx_lost_total)
        trace.final = result
        trace.steps += 1
        tick += n
    return trace


class Line:
    __slots__ = ('text', 'queued', 'done', 'kind', 'index')

    def __init__(self, text, queued, done, index):
        self.text, self.queued, self.done, self.index = text, queued, done, index
        self.kind = line_kind(text)

    def summary(self):
        return dict(queued=self.queued, done=self.done, kind=self.kind, text=self.text)


def line_kind(text):
    tokens = text.split(' ', 2)
    if tokens[0] == 'UBTS_STEER_V3' and len(tokens) > 1:
        second = tokens[1].split('*', 1)[0]
        return {'hello': 'BANNER', 'cal': 'CAL'}.get(second, 'OTHER')
    first = tokens[0].split('*', 1)[0]
    return first if first in ('S', 'N', 'F', 'W', 'A', 'P') else 'OTHER'


def assemble_lines(tx, cuts):
    """(lines, fragments, garbage byte count, incomplete tail) from the MCU byte list.

    Bytes with flag bit0 (boot ROM garbage) are not firmware output and are counted apart. A partial line that a
    reset or power-off cut (the rest of its bytes were discarded) becomes a fragment.
    """
    lines, fragments, garbage = [], [], 0
    current, first_queued = [], None
    cut_list = list(cuts)
    for queued, done, byte, flags in tx:
        if flags & 1:
            garbage += 1
            continue
        if current and any(first_queued < cut <= queued for cut in cut_list):
            fragments.append(dict(queued=first_queued, text=bytes(current).decode('latin-1')))
            current, first_queued = [], None
        if not current:
            first_queued = queued
        if byte == 0x0A:
            lines.append(Line(bytes(current).decode('latin-1'), first_queued, done, len(lines)))
            current, first_queued = [], None
        else:
            current.append(byte)
    tail = dict(queued=first_queued, text=bytes(current).decode('latin-1')) if current else None
    return lines, fragments, garbage, tail


INT_RE = re.compile(r'^(?:0|-?[1-9][0-9]*)$')
UINT_RE = re.compile(r'^(?:0|[1-9][0-9]*)$')


def check_line_format(line):
    """Problems of one MCU line against sections 3 and 5 ([] when it is well formed)."""
    text, problems = line.text, []
    if any(not 0x20 <= ord(ch) <= 0x7E for ch in text):
        return ['byte outside 0x20..0x7E (CR or control) in the line']
    match = re.match(r'^(.*)\*([0-9A-F]{2})$', text)
    if not match:
        return ['line does not end in *HH (upper-case hex)']
    body, given = match.group(1), match.group(2)
    if crc8_hex(body) != given:
        problems.append('CRC-8 %s, line says %s' % (crc8_hex(body), given))
    if '*' in body:
        problems.append('"*" inside the line body')
    kind = line.kind
    if kind == 'OTHER':
        problems.append('unknown first token (the firmware emits only the section 5.1 kinds)')
        return problems
    if len(text) + 1 > MAX_LINE_BYTES[kind]:
        problems.append('%d bytes with LF, section 5.1 maximum for %s is %d' % (len(text) + 1, kind,
                                                                                 MAX_LINE_BYTES[kind]))
    tokens = body.split(' ')
    if kind == 'S':
        if parse_status(body) is None:
            problems.append('status line fields do not follow section 5.3')
    elif kind == 'N':
        if len(tokens) != 3 or not (tokens[1] == '-' or UINT_RE.match(tokens[1])) or tokens[2] not in N_REASONS:
            problems.append('N line is not "N <q|-> <reason>"')
    elif kind == 'F':
        if len(tokens) != 4 or tokens[1] not in F_CODES or not INT_RE.match(tokens[2]) or not UINT_RE.match(tokens[3]):
            problems.append('F line is not "F <CODE> <arg> <ms>"')
    elif kind == 'W':
        if len(tokens) != 2 or not UINT_RE.match(tokens[1]):
            problems.append('W line is not "W <ms>"')
    elif kind == 'A':
        ok = (len(tokens) == 4 and tokens[1] == 'CFG' and tokens[2] in CFG_KEYS and UINT_RE.match(tokens[3])) or (
            len(tokens) == 4 and tokens[1] == 'JOG' and INT_RE.match(tokens[2]) and UINT_RE.match(tokens[3])) or (
            len(tokens) == 5 and tokens[1] == 'POL' and tokens[2] in ('ok', 'fail') and INT_RE.match(tokens[3])
            and INT_RE.match(tokens[4]))
        if not ok:
            problems.append('A line is not one of the section 5.8 forms')
    elif kind == 'P':
        if (len(tokens) != 4 or tokens[1] not in ('D8', 'D5', 'D3', 'D0', 'D1') or not UINT_RE.match(tokens[2])
                or not UINT_RE.match(tokens[3]) or int(tokens[2]) > 255 or int(tokens[3]) > 255):
            problems.append('P line is not "P D<n> <duty> <target>"')
    return problems


STATUS_LIMITS = dict(ms=(0, 0xFFFFFFFF), n=(0, 65535), st=(1, 3), q=(0, 65535), L=(-255, 255), R=(-255, 255),
                     en=(0, 3), raw=(0, 1023), pos=(-9999, 9999), tgt=(-9999, 9999), cmd=(-9999, 9999),
                     sp=(-255, 255), wd=(0, 9999), tl=(0, 1000))


def parse_status(body):
    """{field: value} of a status line body (without *HH), or None if it breaks section 5.3."""
    tokens = body.split(' ')
    if len(tokens) != 16 or tokens[0] != 'S':
        return None
    out = {}
    for name, token in zip(STATUS_FIELDS, tokens[1:]):
        if name == 'fl':
            if not re.match(r'^[0-7][0-9A-F]{3}$', token):
                return None
            out[name] = int(token, 16)
            continue
        if not INT_RE.match(token):
            return None
        value = int(token)
        lo, hi = STATUS_LIMITS[name]
        if not lo <= value <= hi:
            return None
        out[name] = value
    return out


class Channels:
    """Per-tick channel values rebuilt from the change lists (values after each tick's MCU processing)."""

    def __init__(self, changes):
        self.series = {name: [] for name in CHANNELS}
        for tick, name, value in changes:
            self.series[name].append((tick, value))

    def value(self, name, tick):
        series = self.series[name]
        lo, hi = 0, len(series)
        while lo < hi:
            mid = (lo + hi) // 2
            if series[mid][0] <= tick:
                lo = mid + 1
            else:
                hi = mid
        return series[lo - 1][1] if lo else CHANNEL_INITIAL[name]

    def changes(self, name):
        return self.series[name]


def _fmt_value(name, value):
    if name in ('fault_bits', 'fl'):
        return '%04X' % value
    return value


class Evaluator:
    """Checks the expectations of a script against one run."""

    def __init__(self, script, lines, channels):
        self.script, self.lines, self.channels = script, lines, channels
        self.claimed = set()

    def _pattern_hits(self, item, line):
        match = item['regex'].fullmatch(line.text)
        if not match:
            return False
        for kind, group in item['groups']:
            value = match.group(group)
            if kind == 'crc':
                star = match.start(group) - 1
                if crc8_hex(line.text[:star]) != value:
                    return False
            elif self.script.plan.alive(line.queued) != value:
                return False
        return True

    def _tick_of(self, line, at):
        return line.done if at == 'done' else line.queued

    def _in(self, tick, window):
        return window[0] <= tick <= window[1]

    def run(self, item):
        handler = getattr(self, '_' + item['type'])
        ok, detail, actual = handler(item)
        out = dict(id=item['id'], type=item['type'], spec=item['spec'], pass_=ok, detail=detail, actual=actual)
        out['pass'] = out.pop('pass_')
        return out

    def _line(self, item):
        at = item.get('at', 'queued')
        if 'tick' in item:
            window = (item['tick'] - item['tol'], item['tick'] + item['tol'])
        else:
            window = item['window']
        hits = [line for line in self.lines if self._pattern_hits(item, line)]
        inside = [line for line in hits if self._in(self._tick_of(line, at), window)]
        for line in inside:
            self.claimed.add(line.index)
        if len(inside) == 1:
            return True, 'line at %s tick %d' % (at, self._tick_of(inside[0], at)), inside[0].summary()
        near = sorted(hits, key=lambda ln: abs(self._tick_of(ln, at) - window[0]))[:3]
        if not inside:
            detail = 'no line matches %r at %s tick %s' % (item['text'], at, _window_text(window))
            if near:
                detail += '; same pattern at %s' % ', '.join(str(self._tick_of(ln, at)) for ln in near)
            else:
                same_kind = [ln for ln in self.lines if ln.kind == line_kind(item['text'])
                             and abs(self._tick_of(ln, at) - window[0]) <= 20][:3]
                if same_kind:
                    detail += '; nearby %s lines: %s' % (line_kind(item['text']), ' | '.join(
                        '%d:%s' % (self._tick_of(ln, at), ln.text) for ln in same_kind))
            return False, detail, [ln.summary() for ln in near]
        return False, '%d lines match %r in %s (exactly one expected)' % (len(inside), item['text'],
                                                                         _window_text(window)), [
            ln.summary() for ln in inside]

    def _status(self, item):
        found = [line for line in self.lines if line.kind == 'S' and line.queued == item['tick']]
        if len(found) != 1:
            nearby = [ln.queued for ln in self.lines if ln.kind == 'S' and abs(ln.queued - item['tick']) <= 100]
            return False, '%d status lines made at tick %d (nearby: %s)' % (len(found), item['tick'], nearby), None
        line = found[0]
        self.claimed.add(line.index)
        fields = parse_status(line.text.rsplit('*', 1)[0])
        if fields is None:
            return False, 'status line does not parse: %s' % line.text, line.summary()
        bad = ['%s=%s (expected %s)' % (key, _fmt_value(key, fields[key]), comp.describe())
               for key, comp in item['fields'].items() if not comp.matches(fields[key])]
        if bad:
            return False, 'status tick %d: %s; line %s' % (item['tick'], ', '.join(bad), line.text), line.summary()
        return True, 'status tick %d matches' % item['tick'], line.summary()

    def _select(self, item):
        window = item['window']
        out = []
        for line in self.lines:
            if not self._in(line.queued, window):
                continue
            if 'kinds' in item:
                if line.kind in item['kinds']:
                    out.append(line)
            elif self._pattern_hits(item, line):
                out.append(line)
        return out

    def _no_line(self, item):
        found = self._select(item)
        if found:
            return False, '%d unwanted line(s) in %s: %s' % (len(found), _window_text(item['window']), ' | '.join(
                '%d:%s' % (ln.queued, ln.text) for ln in found[:5])), [ln.summary() for ln in found[:20]]
        return True, 'no such line in %s' % _window_text(item['window']), None

    def _count(self, item):
        found = self._select(item)
        for line in found:
            self.claimed.add(line.index)
        ok = item['min'] <= len(found) <= item['max']
        return ok, '%d line(s) in %s, expected %d..%d' % (len(found), _window_text(item['window']), item['min'],
                                                          item['max']), len(found)

    def _pins_at(self, tick, pins):
        bad = []
        for name, comp in pins.items():
            value = self.channels.value(name, tick)
            if not comp.matches(value):
                bad.append('%s=%s (expected %s)' % (name, _fmt_value(name, value), comp.describe()))
        return bad

    def _pins(self, item):
        bad = self._pins_at(item['tick'], item['pins'])
        actual = {name: self.channels.value(name, item['tick']) for name in item['pins']}
        if bad:
            return False, 'tick %d: %s' % (item['tick'], ', '.join(bad)), actual
        return True, 'tick %d matches' % item['tick'], actual

    def _pins_window(self, item):
        t0, t1 = item['window']
        for name, comp in item['pins'].items():
            if not comp.matches(self.channels.value(name, t0)):
                return False, 'tick %d: %s=%s (expected %s)' % (t0, name, _fmt_value(
                    name, self.channels.value(name, t0)), comp.describe()), None
            for tick, value in self.channels.changes(name):
                if t0 < tick <= t1 and not comp.matches(value):
                    return False, 'tick %d: %s=%s (expected %s)' % (tick, name, _fmt_value(name, value),
                                                                    comp.describe()), None
        return True, 'ticks %d..%d match' % (t0, t1), None

    def _transition(self, name, tick, comp):
        series = self.channels.changes(name)
        for t, value in series:
            if t == tick:
                before = self.channels.value(name, tick - 1) if tick > 0 else CHANNEL_INITIAL[name]
                if comp.matches(value) and (comp.kind == 'any' or not comp.matches(before)):
                    return True
        return False

    def _change(self, item):
        name, comp = item['channel'], item['to']
        ticks = [t for t in range(item['tick'] - item['tol'], item['tick'] + item['tol'] + 1)
                 if t >= 0 and self._transition(name, t, comp)]
        if len(ticks) == 1:
            return True, '%s changed to %s at tick %d' % (name, comp.describe(), ticks[0]), ticks[0]
        near = [(t, _fmt_value(name, v)) for t, v in self.channels.changes(name)
                if abs(t - item['tick']) <= 50][:8]
        if not ticks:
            return False, '%s did not change to %s at tick %s; changes near: %s' % (
                name, comp.describe(), _window_text((item['tick'] - item['tol'], item['tick'] + item['tol'])),
                near or 'none'), near
        return False, '%s changed to %s at several ticks %s' % (name, comp.describe(), ticks), near

    def _first_change(self, item):
        name = item['channel']
        later = [(t, v) for t, v in self.channels.changes(name) if t >= item['after']]
        if not later:
            return False, '%s did not change at or after tick %d' % (name, item['after']), None
        tick, value = later[0]
        ok_tick = abs(tick - item['tick']) <= item['tol']
        ok_value = item['to'].matches(value)
        actual = dict(tick=tick, value=_fmt_value(name, value))
        if ok_tick and ok_value:
            return True, 'first %s change after %d: tick %d to %s' % (name, item['after'], tick,
                                                                      _fmt_value(name, value)), actual
        return False, 'first %s change at or after tick %d: tick %d to %s (expected tick %d%s to %s)' % (
            name, item['after'], tick, _fmt_value(name, value), item['tick'],
            ' +-%d' % item['tol'] if item['tol'] else '', item['to'].describe()), actual


def _window_text(window):
    return str(window[0]) if window[0] == window[1] else '%d..%d' % window


def compare_traces(a, b):
    """First difference between two runs (None when lines, ticks, changes and common step ends agree)."""
    if a.tx != b.tx:
        for index, (x, y) in enumerate(zip(a.tx, b.tx)):
            if x != y:
                return 'MCU byte %d: %r vs %r' % (index, x, y)
        return 'MCU byte count %d vs %d' % (len(a.tx), len(b.tx))
    if a.changes != b.changes:
        for index, (x, y) in enumerate(zip(a.changes, b.changes)):
            if x != y:
                return 'channel change %d: %r vs %r' % (index, x, y)
        return 'channel change count %d vs %d' % (len(a.changes), len(b.changes))
    for tick in sorted(set(a.step_ends) & set(b.step_ends)):
        if a.step_ends[tick] != b.step_ends[tick]:
            return 'step end tick %d: %r vs %r' % (tick, a.step_ends[tick], b.step_ends[tick])
    return None


def evaluate(script, trace):
    """Result dict (without run identity) of one run."""
    lines, fragments, garbage, tail = assemble_lines(trace.tx, script.plan.cuts)
    channels = Channels(trace.changes)
    evaluator = Evaluator(script, lines, channels)
    results = [evaluator.run(item) for item in script.expect]
    checks = [evaluator.run(item) for item in script.checks]
    end = script.end['tick']
    format_failures = []
    for line in lines:
        problems = check_line_format(line)
        if problems:
            format_failures.append(dict(line.summary(), problems=problems))
    unexpected = []
    if script.end['unexpected'] == 'fail':
        unexpected = [line.summary() for line in lines if line.queued <= end and line.index not in evaluator.claimed
                      and line.kind not in script.end['ignore_kinds']]
    incomplete = []
    if tail is not None and tail['queued'] is not None and tail['queued'] <= end:
        incomplete.append(tail)
    ok = (all(r['pass'] for r in results) and all(r['pass'] for r in checks) and not format_failures
          and not unexpected and not incomplete)
    shown = [line.summary() for line in lines if line.kind != 'S']
    return dict(pass_=ok, expectations=results, checks=checks, format_failures=format_failures,
                unexpected_lines=unexpected, incomplete_lines=incomplete, fragments=fragments,
                boot_garbage_bytes=garbage, line_count=len(lines),
                status_line_count=sum(1 for line in lines if line.kind == 'S'), lines=shown)


def run_script(script, transport_factory, step_ticks=DEFAULT_STEP_TICKS, check_step_ticks=DEFAULT_CHECK_STEP_TICKS,
               baud=115200):
    """Run and check one script. transport_factory() -> FirmwareTransport (fresh process each call)."""
    started = time.monotonic()
    result = dict(schema=RESULT_SCHEMA, name=script.name, path=script.path, sha256=script.sha256, spec=script.spec,
                  v2_map=script.v2_map, status='error', error=None)
    rx_schedule = schedule_inputs(script, baud)
    transport = None
    try:
        transport = transport_factory()
        result.update(transport.ready or {})
        trace = execute(script, transport, step_ticks, rx_schedule)
        partition = dict(step_ticks=step_ticks, check_step_ticks=check_step_ticks or None, same=None, difference=None)
        if check_step_ticks:
            again = execute(script, transport, check_step_ticks, rx_schedule, reset_first=True)
            difference = compare_traces(trace, again)
            partition['same'] = difference is None
            if difference is not None:
                transport.close()
                transport = transport_factory()
                fresh = execute(script, transport, check_step_ticks, rx_schedule)
                if compare_traces(trace, fresh) is None:
                    cause = 'RESET_REQUEST does not restore the startup state'
                elif compare_traces(again, fresh) is None:
                    cause = 'results depend on the step partition'
                else:
                    cause = 'runs are not deterministic'
                partition['difference'] = '%s: %s' % (cause, difference)
        evaluated = evaluate(script, trace)
        ok = evaluated.pop('pass_') and partition['same'] is not False
        result.update(evaluated)
        result.update(partition=partition, ticks_run=script.total_ticks, steps=trace.steps,
                      rx_lost_total=trace.final.rx_lost_total if trace.final else None,
                      boot_count=trace.final.boot_count if trace.final else None,
                      status='pass' if ok else 'fail')
    except TransportError as exc:
        result.update(status='error', error=str(exc), unavailable=exc.unavailable)
    finally:
        if transport is not None:
            transport.close()
    result['pass'] = result['status'] == 'pass'
    result['wall_s'] = round(time.monotonic() - started, 3)
    result['label'] = 'placeholder-based SIM result (not field validation)'
    return result


def native_transport_factory(script, executable, expect_build=None, timeout_s=30.):
    config = startup_config(script, expect_build)
    return lambda: NativeFirmwareTransport(executable, config, timeout_s)


class BackendFirmwareTransport(FirmwareTransport):
    """The same native program through steering.actuator_link.FirmwareSteeringBackend (the package codec,
    native/lockstep_v2.py) instead of this runner's own codec. Running the scripts through both transports checks
    the two codecs against each other (CONFORMANCE_KO.md 9). The backend builds its startup object from the
    script setup; it must equal startup_config(script) or the transport refuses to run."""

    def __init__(self, script, executable, expect_build=None, timeout_s=30.):
        from .. import vehicle_profile as vp
        from ..actuator_link import FirmwareSteeringBackend, NativeBackendError
        self._error = NativeBackendError
        setup = script.setup
        profile = vp.load_profile(Path(vp.__file__).resolve().parent / 'profiles' / (setup['profile'] + '.yaml'))
        expected = startup_config(script, expect_build)
        injections = [dict(kind=item['kind'], at_s=item['at_tick'] / 1000., params=item['params'])
                      for item in setup['injections']]
        try:
            self.backend = FirmwareSteeringBackend(profile, setup['seed'], firmware=dict(expected['firmware']),
                                                   injections=injections,
                                                   startup_overrides=setup['startup_overrides'] or None,
                                                   executable=executable, response_timeout_s=timeout_s)
        except (NativeBackendError, ValueError) as exc:
            raise TransportError('FirmwareSteeringBackend: %s' % exc, unavailable=True) from None
        if self.backend.config != expected:
            self.backend.close()
            raise TransportError('FirmwareSteeringBackend startup differs from the runner startup_config()')
        self.ready = dict(build_id=self.backend.build_id, fw_build=self.backend.firmware_build,
                          executable=str(executable), executable_sha256=_file_sha256(executable))

    @staticmethod
    def _base(out):
        base = {name: getattr(out, name) for name in BASE_FIELDS if name != 'serial_out_len'}
        base['serial_out_len'] = len(out.serial_out)
        for name in ('steer_en', 'drive_en'):
            base[name] = int(base[name])
        return base

    def reset(self, delta_eq_rad):
        try:
            out = self.backend.reset(delta_eq_rad)
        except (self._error, ValueError) as exc:
            raise TransportError('reset: %s' % exc) from None
        base = self._base(out)
        if base['ticks'] or base['fault_bits'] or base['serial_out_len'] or base['steer_en'] or base['drive_en']:
            raise TransportError('RESET_RESPONSE must have ticks 0, outputs 0, fault_bits 0, serial_out_len 0: %r'
                                 % base)
        return base

    def step(self, first_tick, ticks, speed, supply, stopped, events, rx):
        if first_tick != self.backend.ticks:
            raise TransportError('step starts at tick %d, the backend is at %d' % (first_tick, self.backend.ticks))
        try:
            out = self.backend.fw_step(ticks / 1000., events=events, rx=rx, vehicle_speed_m_s=float(speed),
                                       vehicle_stopped=bool(stopped), supply_scale=float(supply))
        except (self._error, ValueError) as exc:
            raise TransportError('step: %s' % exc) from None
        changes = [(c.tick, c.channel, c.value & 0xFFFFFFFF if c.channel in UNSIGNED_CHANNELS else c.value)
                   for c in out.changes]
        tx = [(b.queued_tick, b.done_tick, b.byte, 1 if b.boot_garbage else 0) for b in out.tx]
        millis = 0xFFFFFFFF if out.mcu_millis is None else out.mcu_millis
        return FwStep(first_tick, ticks, self._base(out.base), changes, tx, out.mcu_state, out.run_flags,
                      out.boot_count, millis, out.rx_lost_total)

    def close(self):
        self.backend.close()


def backend_transport_factory(script, executable, expect_build=None, timeout_s=30.):
    startup_config(script, expect_build)   # script errors surface here, as for the native transport
    return lambda: BackendFirmwareTransport(script, executable, expect_build, timeout_s)


TRANSPORT_FACTORIES = {'native': native_transport_factory, 'backend': backend_transport_factory}


# --- report -----------------------------------------------------------------------------------------------------------
def format_result(result, verbose=False):
    head = '%-44s %s' % (result['name'], result['status'].upper())
    out = [head]
    if result.get('error'):
        out.append('    error: %s' % result['error'])
        return '\n'.join(out)
    for group in ('expectations', 'checks'):
        for item in result.get(group, []):
            if not item['pass'] or verbose:
                out.append('    %s %s [%s]: %s' % ('ok  ' if item['pass'] else 'FAIL', item['id'], item['type'],
                                                   item['detail']))
    for item in result.get('format_failures', []):
        out.append('    FAIL line format at %d: %s: %s' % (item['queued'], item['text'], '; '.join(item['problems'])))
    for item in result.get('unexpected_lines', []):
        out.append('    FAIL unexpected line at %d: %s' % (item['queued'], item['text']))
    for item in result.get('incomplete_lines', []):
        out.append('    FAIL line still incomplete after the drain: queued %s: %r' % (item['queued'], item['text']))
    part = result.get('partition') or {}
    if part.get('same') is False:
        out.append('    FAIL step partition %s vs %s: %s' % (part['step_ticks'], part['check_step_ticks'],
                                                             part['difference']))
    return '\n'.join(out)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Run the ESP protocol V3 conformance scripts against the native '
                                                 'firmware build (lockstep v2 firmware mode).')
    parser.add_argument('names', nargs='*', help='script names or glob patterns (default: all)')
    parser.add_argument('--list', action='store_true', help='list the scripts and exit')
    parser.add_argument('--validate-only', action='store_true', help='only validate the scripts')
    parser.add_argument('--scripts-dir', default=str(SCRIPTS_DIR), help='script folder')
    parser.add_argument('--executable', help='native lockstep program (default: build it)')
    parser.add_argument('--compiler', help='C++ compiler for the build (default: $UBTS_STEER_CXX or g++)')
    parser.add_argument('--build-dir', help='build folder (default: $UBTS_STEER_BUILD_DIR)')
    parser.add_argument('--out', help='write the JSON result here')
    parser.add_argument('--step-ticks', type=int, default=DEFAULT_STEP_TICKS, help='ticks per lockstep step')
    parser.add_argument('--check-step-ticks', type=int, default=DEFAULT_CHECK_STEP_TICKS,
                        help='step size of the partition check run (0: skip it)')
    parser.add_argument('--expect-build', help='firmware build id (8 hex) the native program must report')
    parser.add_argument('--timeout', type=float, default=30., help='seconds per lockstep response')
    parser.add_argument('--transport', choices=sorted(TRANSPORT_FACTORIES), default='native',
                        help="native: this runner's own lockstep v2 codec (default); backend: "
                             "actuator_link.FirmwareSteeringBackend (the package codec)")
    parser.add_argument('--verbose', action='store_true', help='print passing expectations too')
    args = parser.parse_args(argv)
    if not 1 <= args.step_ticks <= 100 or not 0 <= args.check_step_ticks <= 100:
        parser.error('step sizes must be 1..100 ticks (dt <= 0.1 s)')
    if args.expect_build and not re.match(r'^[0-9A-F]{8}$', args.expect_build):
        parser.error('--expect-build needs 8 upper-case hex digits')
    try:
        self_check()
        scripts = load_scripts(args.names, args.scripts_dir)
    except (ScriptError, OSError, AssertionError) as exc:
        print('invalid: %s' % exc, file=sys.stderr)
        return EXIT_INVALID
    if args.list:
        for script in scripts:
            print('%-44s spec %-28s %s' % (script.name, ','.join(script.spec), script.v2_map or ''))
        return EXIT_PASS
    if args.validate_only:
        print('%d scripts valid' % len(scripts))
        return EXIT_PASS
    try:
        executable = resolve_executable(args.executable, args.compiler, args.build_dir)
        ok, reason = probe_firmware_mode(executable, args.timeout)
    except TransportError as exc:
        ok, reason = False, str(exc)
    if not ok:
        print('firmware mode unavailable: %s' % reason, file=sys.stderr)
        return EXIT_UNAVAILABLE
    results = []
    for script in scripts:
        try:
            factory = TRANSPORT_FACTORIES[args.transport](script, executable, args.expect_build, args.timeout)
        except ScriptError as exc:
            print('invalid: %s: %s' % (script.name, exc), file=sys.stderr)
            return EXIT_INVALID
        result = run_script(script, factory, args.step_ticks, args.check_step_ticks)
        result['transport'] = args.transport
        results.append(result)
        print(format_result(result, args.verbose))
    passed = sum(1 for r in results if r['pass'])
    summary = dict(schema=RESULT_SCHEMA, scripts=results, total=len(results), passed=passed,
                   failed=len(results) - passed, executable=executable,
                   executable_sha256=_file_sha256(executable), build_id=reason, transport=args.transport,
                   label='placeholder-based SIM result (not field validation)')
    print('%d/%d scripts pass' % (passed, len(results)))
    if args.out:
        Path(args.out).write_bytes((json.dumps(summary, indent=1, sort_keys=True) + '\n').encode('utf-8'))
    return EXIT_PASS if passed == len(results) else EXIT_FAIL


if __name__ == '__main__':
    sys.exit(main())
