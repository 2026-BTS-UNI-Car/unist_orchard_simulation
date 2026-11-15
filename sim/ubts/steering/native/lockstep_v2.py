"""Lockstep protocol v2 (firmware mode) for the native steering process (LOCKSTEP_PROTOCOL_KO.md section 11).

The v1 frames and payloads stay as in backend_api (only the header version becomes 2). Firmware mode adds
FW_STEP_REQUEST (0x0003) and FW_STEP_RESPONSE (0x8003); RESET keeps the v1 layout. This module holds the
Python side of those frames, the v2 startup line, the UART timing rule and the cumulative tick rule.
Standard library only (no numpy); runs on Python 3.10, 3.12 and 3.13.

Change channels and the run fields of FW_STEP_RESPONSE are evaluation and conformance data (truth-like): they
never go to an executor or estimator; a host sees only the serial bytes.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import math
import struct
from typing import Optional, Tuple

from .. import abs_hold
from .. import backend_api as ba

PROTOCOL_V2 = 'ubts.steer_lockstep.v2'
WIRE_VERSION_V2 = 2
FRAME_FW_STEP_REQUEST = 0x0003
FRAME_FW_STEP_RESPONSE = 0x8003
FW_STEP_REQUEST = struct.Struct('<dddBBHI')     # 32 B: dt_s, speed, supply, stopped, reserved, events, rx
FW_EVENT = struct.Struct('<HHI')                # 8 B: tick_offset, kind, arg
FW_RX = struct.Struct('<HBB')                   # 4 B: tick_offset, byte, reserved
FW_STEP_TAIL = struct.Struct('<HHBBHII')        # 16 B after the 80 B block
FW_CHANGE = struct.Struct('<HBBi')              # 8 B: tick_offset, channel, reserved, value
FW_TX = struct.Struct('<IIBBH')                 # 12 B: queued_tick, done_tick, byte, flags, reserved
FW_STEP_RESPONSE_BYTES = ba.STEP_RESPONSE.size + FW_STEP_TAIL.size   # 96
MAX_FW_DT_S = 0.1
MAX_EVENTS = 64
MAX_RX = 4096
MAX_CHANGES = 4096
MAX_TX = 4096
MAX_FW_REQUEST_BYTES = FW_STEP_REQUEST.size + FW_EVENT.size * MAX_EVENTS + FW_RX.size * MAX_RX
MAX_FW_RESPONSE_BYTES = FW_STEP_RESPONSE_BYTES + FW_CHANGE.size * MAX_CHANGES + FW_TX.size * MAX_TX
MCU_TICK_HZ = 1000
TICK_UNITS = 576                    # time unit u = 1/576000 s; one tick (1 ms) = 576 u
UNITS_PER_BIT_SECOND = 5760000      # one byte (10 bits) = 5,760,000 / baud u
MCU_NOT_RUNNING = 255
MILLIS_NOT_RUNNING = 0xFFFFFFFF

EVENT_KINDS = {'mcu_reset': 1, 'power_cycle': 2, 'port_open_reset': 3, 'motor_power': 4, 'mcu_stall': 5,
               'adc_set': 6, 'adc_model': 7}
EVENT_NAMES = {code: name for name, code in EVENT_KINDS.items()}
EVENT_ARG_RANGES = {'mcu_reset': (1, 6), 'power_cycle': (1, 60000), 'port_open_reset': (0, 0), 'motor_power': (0, 1),
                    'mcu_stall': (1, 60000), 'adc_set': (0, 1023), 'adc_model': (0, 0)}
CHANNELS = ('drive_l', 'drive_r', 'drive_en', 'steer_pwm', 'steer_en', 'mcu_state', 'fault_bits', 'gpio_writes',
            'eeprom_commits', 'adc_read', 'servo_pos', 'servo_tgt', 'servo_cmd', 'steer_mode')
# Channels 11..14 (report_servo, SIM evaluation only): the firmware's own P[n], tgt and cmd (permille) and steering mode
# (0 OFF, 1 ACTIVE, 2 HOLD, 3 JOG, 4 POL) after each tick; LOCKSTEP_PROTOCOL_KO.md 11.6.
SERVO_CHANNELS = ('servo_pos', 'servo_tgt', 'servo_cmd', 'steer_mode')
STEER_MODE_NAMES = {0: 'OFF', 1: 'ACTIVE', 2: 'HOLD', 3: 'JOG', 4: 'POL'}
CHANNEL_CODES = {name: index + 1 for index, name in enumerate(CHANNELS)}
CHANNEL_NAMES = {code: name for name, code in CHANNEL_CODES.items()}
RUN_POWERED = 1
RUN_SETUP_DONE = 2
RUN_STALLED = 4
RUN_MOTOR_POWER = 8
RESET_REASONS = {0: 'power_on', 1: 'hw_wdt', 2: 'exception', 3: 'soft_wdt', 4: 'soft_restart', 5: 'deep_sleep',
                 6: 'ext_sys'}

FIRMWARE_KEYS = ('expect_build', 'boot_seed', 'chip_id', 'eeprom_hex', 'initial_delta_eq_rad', 'boot_delay_ticks',
                 'millis_start', 'boot_garbage_hex', 'adc_mode', 'adc_script_raw', 'motor_power', 'report_adc',
                 'report_servo')
FIRMWARE_DEFAULTS = dict(expect_build=None, boot_seed=0, chip_id=0, eeprom_hex=None, initial_delta_eq_rad=0.,
                         boot_delay_ticks=0, millis_start=0, boot_garbage_hex='', adc_mode='model', adc_script_raw=512,
                         motor_power=True, report_adc=False, report_servo=False)
_HEX = frozenset('0123456789abcdefABCDEF')


# --- startup line ----------------------------------------------------------------------------------------------------
def _int_in(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError('firmware_%s_out_of_range:%r' % (name, value))
    return value


def firmware_block(**values):
    """The startup 'firmware' object (section 11.2): FIRMWARE_DEFAULTS updated by `values`, checked."""
    unknown = set(values) - set(FIRMWARE_KEYS)
    if unknown:
        raise ValueError('firmware_keys_unknown:%s' % ','.join(sorted(unknown)))
    block = dict(FIRMWARE_DEFAULTS)
    block.update(values)
    build = block['expect_build']
    if build is not None and (not isinstance(build, str) or len(build) != 8
                              or any(c not in '0123456789ABCDEF' for c in build)):
        raise ValueError('firmware_expect_build_needs_8_uppercase_hex:%r' % (build,))
    _int_in(block['boot_seed'], 0, 0xFFFFFFFFFFFFFFFF, 'boot_seed')
    _int_in(block['chip_id'], 0, 0xFFFFFF, 'chip_id')
    image = block['eeprom_hex']
    if image is not None and (not isinstance(image, str) or len(image) != 512 or any(c not in _HEX for c in image)):
        raise ValueError('firmware_eeprom_hex_needs_512_hex_digits')
    delta = block['initial_delta_eq_rad']
    if isinstance(delta, bool) or not isinstance(delta, (int, float)) or not math.isfinite(delta):
        raise ValueError('firmware_initial_delta_not_finite')
    block['initial_delta_eq_rad'] = float(delta)
    _int_in(block['boot_delay_ticks'], 0, 1000, 'boot_delay_ticks')
    _int_in(block['millis_start'], 0, 0xFFFFFFFF, 'millis_start')
    garbage = block['boot_garbage_hex']
    if not isinstance(garbage, str) or len(garbage) % 2 or len(garbage) > 512 or any(c not in _HEX for c in garbage):
        raise ValueError('firmware_boot_garbage_hex_invalid')
    if block['adc_mode'] not in ('model', 'script'):
        raise ValueError('firmware_adc_mode_unknown:%r' % (block['adc_mode'],))
    _int_in(block['adc_script_raw'], 0, 1023, 'adc_script_raw')
    for name in ('motor_power', 'report_adc', 'report_servo'):
        if not isinstance(block[name], bool):
            raise ValueError('firmware_%s_not_bool' % name)
    return block


def merge_patch(target, patch):
    """JSON merge patch (RFC 7386) of `patch` onto a copy of `target`."""
    if not isinstance(patch, dict):
        return copy.deepcopy(patch)
    out = copy.deepcopy(target) if isinstance(target, dict) else {}
    for key, value in patch.items():
        if value is None:
            out.pop(key, None)
        else:
            out[key] = merge_patch(out.get(key), value)
    return out


def firmware_startup_config(profile, seed, firmware=None, injections=(), mcu_tick_hz=None, startup_overrides=None):
    """Startup object of protocol v2 in firmware mode: the v1 object of backend_api.native_startup_config with
    protocol v2 and the 'firmware' block (firmware_block(**firmware)); startup_overrides is a merge patch applied
    last (conformance scripts). The native reads mcu.baud only; the other mcu values reach the firmware through
    the EEPROM image (fw_eeprom). A schema v4 profile in steer_fw_mode abs_hold_8 adds the block 'steer_law' (the
    ABS-HOLD-8 law replaces the V3 servo law; abs_hold.py); every other profile gives the same object as before."""
    config = ba.native_startup_config(profile, seed, mode='firmware',
                                      mcu_tick_hz=MCU_TICK_HZ if mcu_tick_hz is None else mcu_tick_hz,
                                      injections=injections)
    config['protocol'] = PROTOCOL_V2
    config['firmware'] = firmware_block(**(firmware or {}))
    law = abs_hold.startup_block(profile)
    if law is not None:
        config['steer_law'] = law
    if startup_overrides:
        config = merge_patch(config, startup_overrides)
    return config


def v2_startup_config(profile, seed, mode='direct_pwm', mcu_tick_hz=None, injections=()):
    """Startup object of protocol v2 for the v1 modes (direct_pwm, test_servo): firmware null."""
    if mode not in ('direct_pwm', 'test_servo'):
        raise ValueError('v2_startup_mode_needs_direct_pwm_or_test_servo:%r' % (mode,))
    config = ba.native_startup_config(profile, seed, mode=mode, mcu_tick_hz=mcu_tick_hz, injections=injections)
    config['protocol'] = PROTOCOL_V2
    config['firmware'] = None
    return config


# --- tick and UART rules ------------------------------------------------------------------------------------------------
def _round_nonnegative(x):
    whole = math.floor(x)
    return int(whole) + (1 if x - whole >= .5 else 0)


class TickClock:
    """The cumulative tick rule of protocol section 5 (same double arithmetic as the native)."""

    def __init__(self, tick_hz=MCU_TICK_HZ):
        self.tick_hz = tick_hz
        self.plant_time = 0.
        self.ticks_total = 0

    def reset(self):
        self.plant_time = 0.
        self.ticks_total = 0

    def ticks_for(self, dt_s):
        return _round_nonnegative((self.plant_time + dt_s) * float(self.tick_hz)) - self.ticks_total

    def advance(self, dt_s):
        self.plant_time += dt_s
        target = _round_nonnegative(self.plant_time * float(self.tick_hz))
        if target > self.ticks_total:
            self.ticks_total = target


def byte_units(baud):
    if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
        raise ValueError('baud_invalid:%r' % (baud,))
    return UNITS_PER_BIT_SECOND // baud


def host_arrival_ticks(count, write_tick, line_end_u=0, delay_ticks=0, baud=115200):
    """Host -> MCU arrival ticks of `count` bytes written at the start of `write_tick` (section 11.7):
    E_k = max(h + d, E_{k-1}) + bytes_u, arrival = ceil(E_k / 576). Returns (ticks, new line end in u)."""
    unit = byte_units(baud)
    start = write_tick * TICK_UNITS + delay_ticks * TICK_UNITS
    ticks = []
    end = line_end_u
    for _ in range(count):
        end = max(start, end) + unit
        ticks.append(-(-end // TICK_UNITS))
    return ticks, end


def mcu_done_tick(end_u):
    """done_tick of an MCU byte whose stop bit ends at end_u: ceil(E / 576) - 1."""
    return -(-end_u // TICK_UNITS) - 1


# --- frames ---------------------------------------------------------------------------------------------------------
def encode_header_v2(frame_type, seq, payload_length):
    return ba.HEADER.pack(ba.WIRE_MAGIC, WIRE_VERSION_V2, frame_type, seq, payload_length)


_KNOWN_TYPES = (ba.FRAME_STEP_REQUEST, ba.FRAME_RESET_REQUEST, FRAME_FW_STEP_REQUEST, ba.FRAME_READY,
                ba.FRAME_STEP_RESPONSE, ba.FRAME_RESET_RESPONSE, FRAME_FW_STEP_RESPONSE)


def decode_header_v2(data, expected_type=None, expected_seq=None):
    """(frame_type, seq, payload_length) of a version-2 header; raises backend_api.ProtocolError."""
    if len(data) < ba.HEADER.size:
        raise ba.ProtocolError(ba.EXIT_TRUNCATED, 'header needs %d bytes, got %d' % (ba.HEADER.size, len(data)))
    magic, version, frame_type, seq, length = ba.HEADER.unpack_from(data)
    if magic != ba.WIRE_MAGIC or version != WIRE_VERSION_V2:
        raise ba.ProtocolError(ba.EXIT_BAD_HEADER, 'magic %r version %r' % (magic, version))
    if frame_type not in _KNOWN_TYPES:
        raise ba.ProtocolError(ba.EXIT_BAD_HEADER, 'frame type 0x%04x' % frame_type)
    if expected_type is not None and frame_type != expected_type:
        raise ba.ProtocolError(ba.EXIT_BAD_HEADER, 'frame type 0x%04x, expected 0x%04x' % (frame_type, expected_type))
    if expected_seq is not None and seq != expected_seq:
        raise ba.ProtocolError(ba.EXIT_SEQUENCE, 'seq %d, expected %d' % (seq, expected_seq))
    return frame_type, seq, length


def read_frame_v2(stream):
    """One whole version-2 frame from a binary stream; b'' on a clean EOF before the first byte."""
    header = ba._read_exact(stream, ba.HEADER.size, allow_eof=True)
    if not header:
        return b''
    _, _, length = decode_header_v2(header)
    if length > MAX_FW_RESPONSE_BYTES:
        raise ba.ProtocolError(ba.EXIT_BAD_LENGTH, 'payload length %d' % length)
    return header + ba._read_exact(stream, length, allow_eof=False)


def _body(frame, frame_type, expected_seq, low, high):
    _, seq, length = decode_header_v2(frame, frame_type, expected_seq)
    if not low <= length <= high:
        raise ba.ProtocolError(ba.EXIT_BAD_LENGTH, 'payload length %d' % length)
    if len(frame) != ba.HEADER.size + length:
        raise ba.ProtocolError(ba.EXIT_TRUNCATED if len(frame) < ba.HEADER.size + length else ba.EXIT_BAD_LENGTH,
                               'frame has %d bytes, header says %d' % (len(frame), ba.HEADER.size + length))
    return seq, memoryview(frame)[ba.HEADER.size:]


def decode_ready_v2(frame, expected_tick_hz=None):
    """(mcu_tick_hz, build_id) of the version-2 READY frame."""
    _, body = _body(frame, ba.FRAME_READY, 0, ba.READY.size, ba.READY.size + ba.MAX_BUILD_ID_BYTES)
    tick, length = ba.READY.unpack_from(body)
    if len(body) != ba.READY.size + length:
        raise ba.ProtocolError(ba.EXIT_BAD_LENGTH, 'build id length %d does not match the payload' % length)
    if expected_tick_hz is not None and tick != expected_tick_hz:
        raise ba.ProtocolError(ba.EXIT_BAD_VALUE, 'ready tick %d, startup tick %d' % (tick, expected_tick_hz))
    try:
        build_id = bytes(body[ba.READY.size:]).decode('ascii')
    except UnicodeDecodeError:
        raise ba.ProtocolError(ba.EXIT_BAD_VALUE, 'build id is not ASCII') from None
    return tick, build_id


def encode_reset_request_v2(seq, delta_eq_rad):
    delta = float(delta_eq_rad)
    if not math.isfinite(delta) or abs(delta) >= math.pi / 2:
        raise ValueError('reset_delta_out_of_range')
    payload = ba.RESET_REQUEST.pack(delta)
    return encode_header_v2(ba.FRAME_RESET_REQUEST, seq, len(payload)) + payload


def _decode_block(body):
    (sim_time, left, right, eq, motor, pwm, drive_left, drive_right, ticks, pot_raw, faults, flags, steer_en,
     drive_en, serial_length) = ba.STEP_RESPONSE.unpack_from(body)
    if serial_length != 0:
        raise ba.ProtocolError(ba.EXIT_BAD_LENGTH, 'serial_out_len %d in a v2 response' % serial_length)
    if steer_en > 1 or drive_en > 1:
        raise ba.ProtocolError(ba.EXIT_BAD_VALUE, 'steer_en %d drive_en %d' % (steer_en, drive_en))
    out = ba.SteerStepOutput(sim_time_s=sim_time, ticks=ticks, delta_left_rad=left, delta_right_rad=right,
                             delta_eq_rad=eq, motor_side_angle_rad=motor, pot_raw=pot_raw, applied_steer_pwm=pwm,
                             steer_en=bool(steer_en), fault_bits=faults, model_flags=flags,
                             drive_duty_left=drive_left, drive_duty_right=drive_right, drive_en=bool(drive_en))
    try:
        out.validate()
    except ValueError as exc:
        raise ba.ProtocolError(ba.EXIT_BAD_VALUE, str(exc)) from None
    return out


def decode_reset_response_v2(frame, expected_seq):
    _, body = _body(frame, ba.FRAME_RESET_RESPONSE, expected_seq, ba.STEP_RESPONSE.size, ba.STEP_RESPONSE.size)
    return _decode_block(body)


def decode_step_response_v2(frame, expected_seq):
    """v1-mode STEP_RESPONSE under a version-2 header (same payload as v1)."""
    _, body = _body(frame, ba.FRAME_STEP_RESPONSE, expected_seq, ba.STEP_RESPONSE.size, ba.STEP_RESPONSE.size)
    return _decode_block(body)


def encode_step_request_v2(seq, inp):
    """v1-mode STEP_REQUEST under a version-2 header."""
    frame = ba.encode_step_request(seq, inp)
    return encode_header_v2(ba.FRAME_STEP_REQUEST, seq, len(frame) - ba.HEADER.size) + frame[ba.HEADER.size:]


def event_code(kind):
    if isinstance(kind, str):
        if kind not in EVENT_KINDS:
            raise ValueError('fw_event_kind_unknown:%r' % (kind,))
        return EVENT_KINDS[kind]
    if isinstance(kind, bool) or not isinstance(kind, int):
        raise ValueError('fw_event_kind_invalid:%r' % (kind,))
    return kind


def encode_fw_step_request(seq, dt_s, vehicle_speed_m_s=0., supply_scale=1., vehicle_stopped=True, events=(), rx=(),
                           check=True):
    """FW_STEP_REQUEST frame. events: (tick_offset, kind, arg) with kind a name or code; rx: (tick_offset, byte).
    check=False skips the value checks (protocol tests that send bad frames on purpose)."""
    events = [(int(o), event_code(k), int(a)) for o, k, a in events]
    rx = [(int(o), int(b)) for o, b in rx]
    if check:
        if not (isinstance(dt_s, (int, float)) and math.isfinite(dt_s) and 0. < dt_s <= MAX_FW_DT_S):
            raise ValueError('fw_step_dt_out_of_range:%r' % (dt_s,))
        if not math.isfinite(vehicle_speed_m_s) or not (math.isfinite(supply_scale) and 0. < supply_scale <= 2.):
            raise ValueError('fw_step_speed_or_supply_invalid')
        if len(events) > MAX_EVENTS or len(rx) > MAX_RX:
            raise ValueError('fw_step_too_many_entries')
        for offsets in ([o for o, _, _ in events], [o for o, _ in rx]):
            if any(b < a for a, b in zip(offsets, offsets[1:])) or any(not 0 <= o <= 0xFFFF for o in offsets):
                raise ValueError('fw_step_offsets_not_ordered')
        for _, code, arg in events:
            name = EVENT_NAMES.get(code)
            if name is None:
                raise ValueError('fw_event_kind_unknown:%r' % (code,))
            low, high = EVENT_ARG_RANGES[name]
            if not low <= arg <= high:
                raise ValueError('fw_event_arg_out_of_range:%s:%r' % (name, arg))
        if any(not 0 <= b <= 255 for _, b in rx):
            raise ValueError('fw_rx_byte_out_of_range')
    payload = FW_STEP_REQUEST.pack(float(dt_s), float(vehicle_speed_m_s), float(supply_scale), int(bool(vehicle_stopped)),
                                   0, len(events), len(rx))
    payload += b''.join(FW_EVENT.pack(o, k, a) for o, k, a in events)
    payload += b''.join(FW_RX.pack(o, b, 0) for o, b in rx)
    return encode_header_v2(FRAME_FW_STEP_REQUEST, seq, len(payload)) + payload


@dataclass(frozen=True)
class FwTxByte:
    """One MCU -> host byte: queued_tick (written by the firmware), done_tick (stop bit ends), boot garbage flag."""

    queued_tick: int
    done_tick: int
    byte: int
    boot_garbage: bool


@dataclass(frozen=True)
class FwChange:
    """A change channel took `value` at the end of absolute tick `tick` (evaluation data, not for a host)."""

    tick: int
    channel: str
    value: int


@dataclass(frozen=True)
class FwStepOutput:
    """FW_STEP_RESPONSE with absolute ticks. base: the v1 block (truth fields, drive duties, pot_raw, fault_bits)."""

    base: ba.SteerStepOutput
    first_tick: int
    ticks: int
    changes: Tuple[FwChange, ...]
    tx: Tuple[FwTxByte, ...]
    mcu_state: int
    run_flags: int
    boot_count: int
    mcu_millis: Optional[int]
    rx_lost_total: int

    @property
    def running(self):
        return self.mcu_state != MCU_NOT_RUNNING


def decode_fw_step_response(frame, expected_seq, first_tick):
    _, body = _body(frame, FRAME_FW_STEP_RESPONSE, expected_seq, FW_STEP_RESPONSE_BYTES, MAX_FW_RESPONSE_BYTES)
    base = _decode_block(body)
    change_count, tx_count, mcu_state, run_flags, boot_count, millis, lost = FW_STEP_TAIL.unpack_from(
        body, ba.STEP_RESPONSE.size)
    if change_count > MAX_CHANGES or tx_count > MAX_TX:
        raise ba.ProtocolError(ba.EXIT_BAD_LENGTH, 'change_count %d tx_count %d' % (change_count, tx_count))
    if len(body) != FW_STEP_RESPONSE_BYTES + FW_CHANGE.size * change_count + FW_TX.size * tx_count:
        raise ba.ProtocolError(ba.EXIT_BAD_LENGTH, 'payload does not match change_count %d tx_count %d'
                               % (change_count, tx_count))
    if not (mcu_state <= 3 or mcu_state == MCU_NOT_RUNNING) or run_flags & ~0x0F:
        raise ba.ProtocolError(ba.EXIT_BAD_VALUE, 'mcu_state %d run_flags %d' % (mcu_state, run_flags))
    changes, offset = [], FW_STEP_RESPONSE_BYTES
    for _ in range(change_count):
        tick_offset, channel, reserved, value = FW_CHANGE.unpack_from(body, offset)
        offset += FW_CHANGE.size
        if channel not in CHANNEL_NAMES or reserved or tick_offset >= base.ticks:
            raise ba.ProtocolError(ba.EXIT_BAD_VALUE, 'change entry %d %d %d' % (tick_offset, channel, reserved))
        changes.append(FwChange(first_tick + tick_offset, CHANNEL_NAMES[channel], value))
    tx = []
    for _ in range(tx_count):
        queued, done, byte, flags, reserved = FW_TX.unpack_from(body, offset)
        offset += FW_TX.size
        if reserved or flags & ~1 or queued > done:
            raise ba.ProtocolError(ba.EXIT_BAD_VALUE, 'tx entry %d %d %d %d' % (queued, done, flags, reserved))
        tx.append(FwTxByte(queued, done, byte, bool(flags & 1)))
    return FwStepOutput(base=base, first_tick=first_tick, ticks=base.ticks, changes=tuple(changes), tx=tuple(tx),
                        mcu_state=mcu_state, run_flags=run_flags, boot_count=boot_count,
                        mcu_millis=None if millis == MILLIS_NOT_RUNNING else millis, rx_lost_total=lost)


# --- splitmix64 boot ids (ESP protocol 10.1) ---------------------------------------------------------------------------
def sim_boot_ids(boot_seed, count):
    """The first `count` SIM boot_ids of a session with this boot_seed (zero outputs skipped)."""
    state, out = boot_seed & 0xFFFFFFFFFFFFFFFF, []
    mask = 0xFFFFFFFFFFFFFFFF
    while len(out) < count:
        state = (state + 0x9E3779B97F4A7C15) & mask
        z = state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & mask
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & mask
        z ^= z >> 31
        if z & 0xFFFFFFFF:
            out.append(z & 0xFFFFFFFF)
    return out
