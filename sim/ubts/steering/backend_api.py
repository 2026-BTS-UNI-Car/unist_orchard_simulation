"""Contract between the steered plant and a steering actuator backend (design 2.3.1, 3.3, 3.4).

One plant step is one SteerStepInput -> one SteerStepOutput. The backend owns the
steering actuator and potentiometer model (C++, decision R1) and, from phase P3, the MCU
logic in the same lockstep subprocess. The plant side never sees anything else.

Modes:
  direct_pwm  the plant/harness gives a signed steering driver PWM in [-1, 1] and steer EN
  test_servo  the harness gives a target equivalent angle; a simple test-only P servo inside
              the backend harness (NOT firmware) turns it into PWM each servo period
  firmware    reserved for P3: serial bytes in/out per step, the MCU makes PWM and drive duty

Wire format v1 of the native process: docs/steering/LOCKSTEP_PROTOCOL_KO.md. This module
holds the Python side of that format (struct layouts, encode/decode, startup line builder).
Plain dataclasses and struct only; no numpy. Truth outputs (TRUTH_OUTPUT_FIELDS: wheel angles,
motor-side angle, the driver's actual output applied_steer_pwm, model flags) are for the plant,
sensor synthesis and evaluation, never for the executor or estimator.
"""
from __future__ import annotations

import abc
from dataclasses import dataclass
import json
import math
import struct

from . import geometry

PROTOCOL = 'ubts.steer_lockstep.v1'
MODES = ('direct_pwm', 'test_servo', 'firmware')
MODE_CODES = {'direct_pwm': 0, 'test_servo': 1, 'firmware': 2}
MODE_NAMES = {code: name for name, code in MODE_CODES.items()}

# --- fault bits (MCU fault codes, design 4.1 fault table; the status line field `fl`) ---------------------------
FAULT_BIT_NAMES = ('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'SOFTLIMIT',
                   'STALL', 'TRACK', 'THERMAL', 'TTL', 'WATCHDOG', 'CRC', 'BOOT')


class FaultBit:
    """Bit i of the 16-bit MCU fault mask is FAULT_BIT_NAMES[i]; bit 15 is reserved (0)."""

    NOCAL = 1 << 0        # no calibration or calibration CRC wrong; ARM refused
    POT_OPEN = 1 << 1     # filtered raw below pot_valid_raw.min_raw
    POT_SHORT = 1 << 2    # filtered raw above pot_valid_raw.max_raw
    POT_JUMP = 1 << 3     # pot_jump_counts
    POT_NOISE = 1 << 4    # pot_noise_counts
    POLARITY = 1 << 5     # polarity_fault
    DRV_STUCK = 1 << 6    # driver_stuck_fault
    SOFTLIMIT = 1 << 7    # measured position outside the soft limit (warning, not latched)
    STALL = 1 << 8        # stall_detect
    TRACK = 1 << 9        # track_fault
    THERMAL = 1 << 10     # thermal_proxy
    TTL = 1 << 11         # C frame TTL expired (counted, not latched)
    WATCHDOG = 1 << 12    # link watchdog
    CRC = 1 << 13         # crc_burst_n bad lines in a row
    BOOT = 1 << 14        # just booted (information)


FAULT_BITS = {name: 1 << index for index, name in enumerate(FAULT_BIT_NAMES)}
FAULT_RESERVED_MASK = 0xFFFF & ~sum(FAULT_BITS.values())
# Latching per the design 4.1 fault table (SOFTLIMIT warning, TTL counter, BOOT information).
LATCHED_FAULTS = ('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'STALL', 'TRACK',
                  'THERMAL', 'WATCHDOG', 'CRC')
# Host-side faults (bridge, P3); not in the MCU mask.
HOST_FAULT_NAMES = ('host_status_gap', 'host_seq_gap', 'mcu_reset_detected', 'cal_id_mismatch',
                    'hardware_profile_mismatch', 'host_track_mismatch', 'stop_unconfirmed', 'steering_ineffective')

# --- model flags (truth side; plant and evaluation only) --------------------------------------------------------------
MODEL_FLAG_NAMES = ('END_STOP_LEFT', 'END_STOP_RIGHT', 'IN_BACKLASH', 'FREE_WHEELING', 'INJECTION_ACTIVE',
                    'POT_CLIPPED')


class ModelFlag:
    """Bit i of the 16-bit model flag mask is MODEL_FLAG_NAMES[i]; bits 6..15 are reserved (0)."""

    END_STOP_LEFT = 1 << 0     # wheel-side equivalent angle at the left mechanical limit
    END_STOP_RIGHT = 1 << 1    # ... at the right mechanical limit
    IN_BACKLASH = 1 << 2       # motor side inside the play, not pushing the link
    FREE_WHEELING = 1 << 3     # driver gives no torque and the steering is not self-locking
    INJECTION_ACTIVE = 1 << 4  # at least one fault injection is active
    POT_CLIPPED = 1 << 5       # last ADC sample clipped at 0 or 2^bits - 1


MODEL_FLAGS = {name: 1 << index for index, name in enumerate(MODEL_FLAG_NAMES)}
MODEL_FLAG_RESERVED_MASK = 0xFFFF & ~sum(MODEL_FLAGS.values())

# SteerStepOutput fields that are SIM truth (protocol section 4.4): plant, sensor synthesis and evaluation only,
# never an executor, estimator or MCU-status input (design 1.4 rule 1, 2.6, A13). applied_steer_pwm is the
# driver's actual output with driver faults (driver_dead, driver_stuck_on), which a real MCU cannot read.
TRUTH_OUTPUT_FIELDS = ('delta_left_rad', 'delta_right_rad', 'delta_eq_rad', 'motor_side_angle_rad', 'applied_steer_pwm',
                       'model_flags')

# --- fault injections (startup line "injections"; SI units) -------------------------------------------------------------
INJECTION_PARAMS = {
    'jam': (),                          # motor-side angle frozen from at_s
    'slow': ('rate_scale',),            # steering rate limits x rate_scale (0 < scale <= 1)
    'driver_dead': (),                  # driver output stays 0 whatever the command
    'driver_stuck_on': ('pwm',),        # driver output stuck at pwm (-1..1) while steer EN is on
    'polarity_reversed': (),            # motor wiring reversed: +PWM moves toward the right
    'pot_open': (),                     # raw = 0
    'pot_short': (),                    # raw = 2^bits - 1
    'pot_stuck': (),                    # raw frozen at the value at at_s
    'pot_noise_x10': ('scale',),        # noise sd x scale
    'pot_jump': ('counts', 'duration_s'),  # raw + counts for duration_s, then back
    'pot_slip': ('offset_rad',),        # measured angle + offset (shaft slip, inside the range)
    'pot_drift': ('rate_rad_per_s',),   # measured angle + rate * (t - at_s)
    'pot_reversed': (),                 # measured angle sign reversed
}
INJECTION_KINDS = tuple(INJECTION_PARAMS)

# --- profile names read for the startup line --------------------------------------------------------------------------
GEOMETRY_PROFILE_NAMES = ('steer_type', 'wheelbase_m', 'kingpin_width_m', 'steer_ackermann_ratio',
                          'steer_mech_limit_deg', 'steer_soft_margin_deg', 'steer_link_table')
ACTUATOR_PROFILE_NAMES = ('steer_actuator_level', 'steer_rate_max_deg_s', 'steer_rate_standstill_deg_s',
                          'steer_motor_tau_s', 'steer_min_pwm_pct', 'steer_min_pwm_standstill_pct', 'steer_backlash_deg',
                          'steer_backlash_wander', 'steer_self_locking', 'steer_release_restoring', 'steer_lr_asymmetry',
                          'steer_motor_stall_torque_nm', 'steer_load_moving_nm', 'steer_load_standstill_nm',
                          'steer_zero_offset_deg')
POT_PROFILE_NAMES = ('pot_location', 'pot_range_deg', 'pot_to_steer_ratio', 'pot_center_raw', 'adc_bits',
                     'adc_ratiometric', 'pot_supply_sag_per_pwm', 'pot_noise_sd_counts', 'pot_linearity_pct',
                     'pot_gain_error', 'pot_drift_deg_per_h', 'pot_cal_residual_deg', 'pot_fault_injections')
TEST_SERVO_PROFILE_NAMES = ('servo_hz', 'servo_kp_pct_per_deg', 'servo_deadband_deg', 'servo_min_pwm_pct',
                            'servo_max_pwm_pct')
MCU_PROFILE_NAMES = ('sim_mcu_tick_hz', 'servo_dwell_brake_s', 'servo_reversal_pause_ms', 'c_ttl_max_ms',
                     'ttl_expiry_drive_mode', 'watchdog_ms', 'steer_watchdog_mode', 'steer_hold_after_loss_s',
                     'steer_hold_after_stop_s', 'steer_hold_after_disarm_s', 'status_hz', 'status_latency_s',
                     'serial_delay_ms', 'baud', 'mcu_ramp', 'pot_filter', 'pot_valid_raw', 'cal_end_raw_window',
                     'pot_jump_counts', 'pot_noise_counts', 'polarity_fault', 'stall_detect', 'track_fault',
                     'driver_stuck_fault', 'thermal_proxy', 'crc_burst_n')
PER_RUN_DRAW_NAMES = ('steer_zero_offset_deg', 'pot_cal_residual_deg')

# --- wire format v1 -----------------------------------------------------------------------------------------------------
WIRE_MAGIC = b'USTP'
WIRE_VERSION = 1
FRAME_STEP_REQUEST = 0x0001
FRAME_RESET_REQUEST = 0x0002
FRAME_READY = 0x8000
FRAME_STEP_RESPONSE = 0x8001
FRAME_RESET_RESPONSE = 0x8002
HEADER = struct.Struct('<4sHHII')                   # 16 B: magic, version, frame type, seq, payload length
STEP_REQUEST = struct.Struct('<dddddBBBBI')         # 48 B + serial_in
STEP_RESPONSE = struct.Struct('<ddddddddIHHHBBI')   # 80 B + serial_out (also the reset response)
RESET_REQUEST = struct.Struct('<d')                 # 8 B
READY = struct.Struct('<II')                        # 8 B + build id: mcu_tick_hz, build_id length
MAX_SERIAL_BYTES = 4096
MAX_BUILD_ID_BYTES = 256
MAX_STARTUP_LINE_BYTES = 65536
POT_RAW_MAX = 1023
EXIT_OK = 0
EXIT_BAD_STARTUP = 2
EXIT_BAD_HEADER = 3
EXIT_SEQUENCE = 4
EXIT_BAD_LENGTH = 5
EXIT_BAD_VALUE = 6
EXIT_TRUNCATED = 7
EXIT_IO = 8
EXIT_INTERNAL = 9
EXIT_CODES = {EXIT_OK: 'ok', EXIT_BAD_STARTUP: 'bad_startup', EXIT_BAD_HEADER: 'bad_header',
              EXIT_SEQUENCE: 'sequence_mismatch', EXIT_BAD_LENGTH: 'bad_length', EXIT_BAD_VALUE: 'bad_value',
              EXIT_TRUNCATED: 'truncated_frame', EXIT_IO: 'io_error', EXIT_INTERNAL: 'internal_error'}


class ProtocolError(ValueError):
    """A frame or startup line breaks the wire format. .exit_code is the native exit code for it."""

    def __init__(self, exit_code, reason):
        self.exit_code = exit_code
        self.reason = reason
        super().__init__('%s: %s' % (EXIT_CODES.get(exit_code, exit_code), reason))


def fault_names(mask):
    """Names of the set MCU fault bits (raises on reserved bits)."""
    if isinstance(mask, bool) or not isinstance(mask, int) or not 0 <= mask <= 0xFFFF or mask & FAULT_RESERVED_MASK:
        raise ValueError('fault_mask_invalid:%r' % (mask,))
    return [name for name, bit in FAULT_BITS.items() if mask & bit]


def fault_mask(names):
    mask = 0
    for name in names:
        if name not in FAULT_BITS:
            raise ValueError('fault_name_unknown:%r' % (name,))
        mask |= FAULT_BITS[name]
    return mask


def model_flag_names(mask):
    if (isinstance(mask, bool) or not isinstance(mask, int) or not 0 <= mask <= 0xFFFF
            or mask & MODEL_FLAG_RESERVED_MASK):
        raise ValueError('model_flag_mask_invalid:%r' % (mask,))
    return [name for name, bit in MODEL_FLAGS.items() if mask & bit]


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('steer_step_value_not_finite:' + name)
    return float(value)


def _flag(value, name):
    if not isinstance(value, bool):
        raise ValueError('steer_step_flag_not_bool:' + name)
    return value


# --- step dataclasses ---------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class SteerStepInput:
    """Plant -> backend for one plant step (fields map 1:1 to the STEP_REQUEST payload).

    dt_s                plant step (s), 0 < dt_s <= 1
    mode                'direct_pwm' | 'test_servo' | 'firmware' (must equal the startup mode)
    steer_pwm           direct_pwm: signed steering driver PWM in [-1, 1] (+ moves the motor side to the
                        left with correct wiring); must be 0 in the other modes
    steer_en            direct_pwm / test_servo: steering driver enable; ignored (must be True) in firmware
    target_delta_eq_rad test_servo: target equivalent angle (rad, + left); must be 0 in the other modes
    vehicle_speed_m_s   plant truth longitudinal speed (signed); plant side may read truth
    vehicle_stopped     plant truth: the vehicle stands still (standstill steering rates and loads)
    supply_scale        pot supply rail scale from the plant side (drive PWM sag, rail faults), 0 < s <= 2
    serial_in           firmware mode only (P3): bytes host -> MCU during this step; empty otherwise
    """

    dt_s: float
    mode: str = 'direct_pwm'
    steer_pwm: float = 0.
    steer_en: bool = True
    target_delta_eq_rad: float = 0.
    vehicle_speed_m_s: float = 0.
    vehicle_stopped: bool = True
    supply_scale: float = 1.
    serial_in: bytes = b''

    def validate(self):
        if self.mode not in MODES:
            raise ValueError('steer_step_mode_unknown:%r' % (self.mode,))
        dt = _finite(self.dt_s, 'dt_s')
        if not 0. < dt <= 1.:
            raise ValueError('steer_step_dt_out_of_range')
        if abs(_finite(self.steer_pwm, 'steer_pwm')) > 1.:
            raise ValueError('steer_step_pwm_out_of_range')
        if abs(_finite(self.target_delta_eq_rad, 'target_delta_eq_rad')) >= math.pi / 2:
            raise ValueError('steer_step_target_out_of_range')
        _finite(self.vehicle_speed_m_s, 'vehicle_speed_m_s')
        if not 0. < _finite(self.supply_scale, 'supply_scale') <= 2.:
            raise ValueError('steer_step_supply_scale_out_of_range')
        _flag(self.steer_en, 'steer_en')
        _flag(self.vehicle_stopped, 'vehicle_stopped')
        if not isinstance(self.serial_in, (bytes, bytearray)) or len(self.serial_in) > MAX_SERIAL_BYTES:
            raise ValueError('steer_step_serial_in_invalid')
        if self.mode != 'firmware' and self.serial_in:
            raise ValueError('steer_step_serial_in_needs_firmware_mode')
        if self.mode != 'direct_pwm' and self.steer_pwm != 0.:
            raise ValueError('steer_step_pwm_only_in_direct_pwm')
        if self.mode != 'test_servo' and self.target_delta_eq_rad != 0.:
            raise ValueError('steer_step_target_only_in_test_servo')
        if self.mode == 'firmware' and not self.steer_en:
            raise ValueError('steer_step_steer_en_belongs_to_firmware')
        return self


@dataclass(frozen=True)
class SteerStepOutput:
    """Backend -> plant after one step (fields map 1:1 to the STEP_RESPONSE payload).

    sim_time_s            backend time after the step = ticks_total / mcu_tick_hz (s)
    ticks                 MCU ticks run in this step (tick rule in the protocol document)
    delta_left_rad / delta_right_rad / delta_eq_rad   truth wheel angles and wheel-side equivalent angle
    motor_side_angle_rad  truth actuator angle before the backlash, on the equivalent-angle scale
    pot_raw               last ADC sample of the step, 0..1023 (unfiltered; logging and MCU input)
    applied_steer_pwm     truth: the driver's actual output after EN and driver faults, before the wiring
                          polarity (L0: the normalised effort, step / (rate * dt)); not an MCU status value
    steer_en              actual steering driver enable
    fault_bits            MCU fault mask (FaultBit); 0 unless mode is firmware
    model_flags           truth-side flags (ModelFlag)
    drive_duty_left / drive_duty_right / drive_en / serial_out   reserved for firmware mode (0 / False / b'')
    """

    sim_time_s: float
    ticks: int
    delta_left_rad: float
    delta_right_rad: float
    delta_eq_rad: float
    motor_side_angle_rad: float
    pot_raw: int
    applied_steer_pwm: float
    steer_en: bool
    fault_bits: int = 0
    model_flags: int = 0
    drive_duty_left: float = 0.
    drive_duty_right: float = 0.
    drive_en: bool = False
    serial_out: bytes = b''

    def validate(self):
        for name in ('sim_time_s', 'delta_left_rad', 'delta_right_rad', 'delta_eq_rad', 'motor_side_angle_rad',
                     'applied_steer_pwm', 'drive_duty_left', 'drive_duty_right'):
            _finite(getattr(self, name), name)
        for name in ('applied_steer_pwm', 'drive_duty_left', 'drive_duty_right'):
            if abs(getattr(self, name)) > 1.:
                raise ValueError('steer_step_output_duty_out_of_range:' + name)
        for name, top in (('ticks', 0xFFFFFFFF), ('pot_raw', POT_RAW_MAX)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= top:
                raise ValueError('steer_step_output_int_out_of_range:' + name)
        fault_names(self.fault_bits)
        model_flag_names(self.model_flags)
        _flag(self.steer_en, 'steer_en')
        _flag(self.drive_en, 'drive_en')
        if not isinstance(self.serial_out, (bytes, bytearray)) or len(self.serial_out) > MAX_SERIAL_BYTES:
            raise ValueError('steer_step_serial_out_invalid')
        return self

    def faults(self):
        return fault_names(self.fault_bits)

    def flags(self):
        return model_flag_names(self.model_flags)


# --- backend interface -------------------------------------------------------------------------------------------------
class SteeringBackend(abc.ABC):
    """A steering actuator backend (the native lockstep process behind a Python link).

    configure(profile, seed)    build the startup configuration from the vehicle profile and start
    reset(delta_eq_rad=0.0)     new episode: time 0, states and noise streams from the seed, wheels at
                                delta_eq_rad (wheel side, backlash centred); returns the state
    step(inp)                   advance one plant step; returns the state at its end
    close()                     stop the backend; safe to call twice
    """

    @abc.abstractmethod
    def configure(self, profile, seed):
        """Prepare the backend for this profile and seed."""

    @abc.abstractmethod
    def reset(self, delta_eq_rad=0.):
        """Start an episode at the equivalent angle; returns a SteerStepOutput with ticks 0."""

    @abc.abstractmethod
    def step(self, inp):
        """Run one SteerStepInput; returns a SteerStepOutput."""

    @abc.abstractmethod
    def close(self):
        """Release the backend."""

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


# --- codec -------------------------------------------------------------------------------------------------------------
def encode_header(frame_type, seq, payload_length):
    return HEADER.pack(WIRE_MAGIC, WIRE_VERSION, frame_type, seq, payload_length)


def decode_header(data, expected_type=None, expected_seq=None):
    """(frame_type, seq, payload_length) of a 16-byte header; raises ProtocolError."""
    if len(data) < HEADER.size:
        raise ProtocolError(EXIT_TRUNCATED, 'header needs %d bytes, got %d' % (HEADER.size, len(data)))
    magic, version, frame_type, seq, length = HEADER.unpack_from(data)
    if magic != WIRE_MAGIC or version != WIRE_VERSION:
        raise ProtocolError(EXIT_BAD_HEADER, 'magic %r version %r' % (magic, version))
    if frame_type not in (FRAME_STEP_REQUEST, FRAME_RESET_REQUEST, FRAME_READY, FRAME_STEP_RESPONSE,
                          FRAME_RESET_RESPONSE):
        raise ProtocolError(EXIT_BAD_HEADER, 'frame type 0x%04x' % frame_type)
    if expected_type is not None and frame_type != expected_type:
        raise ProtocolError(EXIT_BAD_HEADER, 'frame type 0x%04x, expected 0x%04x' % (frame_type, expected_type))
    if expected_seq is not None and seq != expected_seq:
        raise ProtocolError(EXIT_SEQUENCE, 'seq %d, expected %d' % (seq, expected_seq))
    return frame_type, seq, length


def _payload(frame, frame_type, expected_seq, fixed, variable_max):
    kind, seq, length = decode_header(frame, frame_type, expected_seq)
    if not fixed <= length <= fixed + variable_max:
        raise ProtocolError(EXIT_BAD_LENGTH, 'payload length %d' % length)
    if len(frame) != HEADER.size + length:
        raise ProtocolError(EXIT_TRUNCATED if len(frame) < HEADER.size + length else EXIT_BAD_LENGTH,
                            'frame has %d bytes, header says %d' % (len(frame), HEADER.size + length))
    return seq, memoryview(frame)[HEADER.size:]


def encode_step_request(seq, inp):
    inp.validate()
    payload = STEP_REQUEST.pack(inp.dt_s, inp.steer_pwm, inp.target_delta_eq_rad, inp.vehicle_speed_m_s,
                                inp.supply_scale, MODE_CODES[inp.mode], int(inp.steer_en), int(inp.vehicle_stopped), 0,
                                len(inp.serial_in)) + bytes(inp.serial_in)
    return encode_header(FRAME_STEP_REQUEST, seq, len(payload)) + payload


def decode_step_request(frame, expected_seq=None, expected_mode=None):
    """(seq, SteerStepInput) from a STEP_REQUEST frame (reference decoder for tests and fakes)."""
    seq, body = _payload(frame, FRAME_STEP_REQUEST, expected_seq, STEP_REQUEST.size, MAX_SERIAL_BYTES)
    (dt, pwm, target, speed, supply, mode, steer_en, stopped, reserved,
     serial_length) = STEP_REQUEST.unpack_from(body)
    if len(body) != STEP_REQUEST.size + serial_length:
        raise ProtocolError(EXIT_BAD_LENGTH, 'serial_in length %d does not match the payload' % serial_length)
    if mode not in MODE_NAMES or steer_en > 1 or stopped > 1 or reserved:
        raise ProtocolError(EXIT_BAD_VALUE, 'mode %d steer_en %d stopped %d reserved %d'
                            % (mode, steer_en, stopped, reserved))
    if expected_mode is not None and MODE_NAMES[mode] != expected_mode:
        raise ProtocolError(EXIT_BAD_VALUE, 'mode %s, startup mode %s' % (MODE_NAMES[mode], expected_mode))
    inp = SteerStepInput(dt_s=dt, mode=MODE_NAMES[mode], steer_pwm=pwm, steer_en=bool(steer_en),
                         target_delta_eq_rad=target, vehicle_speed_m_s=speed, vehicle_stopped=bool(stopped),
                         supply_scale=supply, serial_in=bytes(body[STEP_REQUEST.size:]))
    try:
        inp.validate()
    except ValueError as exc:
        raise ProtocolError(EXIT_BAD_VALUE, str(exc)) from None
    return seq, inp


def _encode_output(frame_type, seq, out):
    out.validate()
    payload = STEP_RESPONSE.pack(out.sim_time_s, out.delta_left_rad, out.delta_right_rad, out.delta_eq_rad,
                                 out.motor_side_angle_rad, out.applied_steer_pwm, out.drive_duty_left,
                                 out.drive_duty_right, out.ticks, out.pot_raw, out.fault_bits, out.model_flags,
                                 int(out.steer_en), int(out.drive_en), len(out.serial_out)) + bytes(out.serial_out)
    return encode_header(frame_type, seq, len(payload)) + payload


def _decode_output(frame, frame_type, expected_seq):
    _, body = _payload(frame, frame_type, expected_seq, STEP_RESPONSE.size, MAX_SERIAL_BYTES)
    (sim_time, left, right, eq, motor, pwm, drive_left, drive_right, ticks, pot_raw, faults, flags, steer_en,
     drive_en, serial_length) = STEP_RESPONSE.unpack_from(body)
    if len(body) != STEP_RESPONSE.size + serial_length:
        raise ProtocolError(EXIT_BAD_LENGTH, 'serial_out length %d does not match the payload' % serial_length)
    if steer_en > 1 or drive_en > 1:
        raise ProtocolError(EXIT_BAD_VALUE, 'steer_en %d drive_en %d' % (steer_en, drive_en))
    out = SteerStepOutput(sim_time_s=sim_time, ticks=ticks, delta_left_rad=left, delta_right_rad=right,
                          delta_eq_rad=eq, motor_side_angle_rad=motor, pot_raw=pot_raw, applied_steer_pwm=pwm,
                          steer_en=bool(steer_en), fault_bits=faults, model_flags=flags,
                          drive_duty_left=drive_left, drive_duty_right=drive_right, drive_en=bool(drive_en),
                          serial_out=bytes(body[STEP_RESPONSE.size:]))
    try:
        out.validate()
    except ValueError as exc:
        raise ProtocolError(EXIT_BAD_VALUE, str(exc)) from None
    return out


def encode_step_response(seq, out):
    return _encode_output(FRAME_STEP_RESPONSE, seq, out)


def decode_step_response(frame, expected_seq):
    return _decode_output(frame, FRAME_STEP_RESPONSE, expected_seq)


def encode_reset_request(seq, delta_eq_rad):
    delta = float(delta_eq_rad)
    if not math.isfinite(delta) or abs(delta) >= math.pi / 2:
        raise ValueError('reset_delta_out_of_range')
    payload = RESET_REQUEST.pack(delta)
    return encode_header(FRAME_RESET_REQUEST, seq, len(payload)) + payload


def decode_reset_request(frame, expected_seq=None):
    seq, body = _payload(frame, FRAME_RESET_REQUEST, expected_seq, RESET_REQUEST.size, 0)
    (delta,) = RESET_REQUEST.unpack_from(body)
    if not math.isfinite(delta) or abs(delta) >= math.pi / 2:
        raise ProtocolError(EXIT_BAD_VALUE, 'reset delta %r' % (delta,))
    return seq, delta


def encode_reset_response(seq, out):
    return _encode_output(FRAME_RESET_RESPONSE, seq, out)


def decode_reset_response(frame, expected_seq):
    return _decode_output(frame, FRAME_RESET_RESPONSE, expected_seq)


def encode_ready(mcu_tick_hz, build_id):
    data = build_id.encode('ascii')
    if len(data) > MAX_BUILD_ID_BYTES:
        raise ValueError('build_id_too_long')
    payload = READY.pack(int(mcu_tick_hz), len(data)) + data
    return encode_header(FRAME_READY, 0, len(payload)) + payload


def decode_ready(frame, expected_tick_hz=None):
    """(mcu_tick_hz, build_id) from the READY frame (seq 0)."""
    _, body = _payload(frame, FRAME_READY, 0, READY.size, MAX_BUILD_ID_BYTES)
    tick, length = READY.unpack_from(body)
    if len(body) != READY.size + length:
        raise ProtocolError(EXIT_BAD_LENGTH, 'build id length %d does not match the payload' % length)
    if expected_tick_hz is not None and tick != expected_tick_hz:
        raise ProtocolError(EXIT_BAD_VALUE, 'ready tick %d, startup tick %d' % (tick, expected_tick_hz))
    try:
        build_id = bytes(body[READY.size:]).decode('ascii')
    except UnicodeDecodeError:
        raise ProtocolError(EXIT_BAD_VALUE, 'build id is not ASCII') from None
    return tick, build_id


def read_frame(stream):
    """Read one whole frame from a binary stream; b'' on a clean EOF before the first byte."""
    header = _read_exact(stream, HEADER.size, allow_eof=True)
    if not header:
        return b''
    _, _, length = decode_header(header)
    if length > STEP_RESPONSE.size + MAX_SERIAL_BYTES:
        raise ProtocolError(EXIT_BAD_LENGTH, 'payload length %d' % length)
    return header + _read_exact(stream, length, allow_eof=False)


def _read_exact(stream, size, allow_eof):
    chunks, have = [], 0
    while have < size:
        chunk = stream.read(size - have)
        if not chunk:
            if allow_eof and have == 0:
                return b''
            raise ProtocolError(EXIT_TRUNCATED, 'EOF after %d of %d bytes' % (have, size))
        chunks.append(chunk)
        have += len(chunk)
    return b''.join(chunks)


# --- startup line ---------------------------------------------------------------------------------------------------------
def validate_injections(injections):
    """Normalized list of {'kind', 'at_s', 'params'} (raises ValueError)."""
    out = []
    for item in injections:
        if not isinstance(item, dict) or set(item) != {'kind', 'at_s', 'params'}:
            raise ValueError('injection_needs_kind_at_s_params')
        kind = item['kind']
        if kind not in INJECTION_PARAMS:
            raise ValueError('injection_kind_unknown:%r' % (kind,))
        at = _finite(item['at_s'], 'at_s')
        if at < 0.:
            raise ValueError('injection_at_s_negative')
        params = item['params']
        if not isinstance(params, dict) or set(params) != set(INJECTION_PARAMS[kind]):
            raise ValueError('injection_params_for_%s_need:%s' % (kind, ','.join(INJECTION_PARAMS[kind])))
        clean = {}
        for key, value in params.items():
            if key == 'counts':
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ValueError('injection_counts_not_int')
                clean[key] = value
            else:
                clean[key] = _finite(value, key)
        if kind == 'slow' and not 0. < clean['rate_scale'] <= 1.:
            raise ValueError('injection_rate_scale_out_of_range')
        if kind == 'driver_stuck_on' and abs(clean['pwm']) > 1.:
            raise ValueError('injection_pwm_out_of_range')
        if kind == 'pot_noise_x10' and clean['scale'] <= 0.:
            raise ValueError('injection_scale_not_positive')
        if kind == 'pot_jump' and clean['duration_s'] <= 0.:
            raise ValueError('injection_duration_not_positive')
        out.append(dict(kind=kind, at_s=at, params=clean))
    return out


def native_startup_config(profile, seed, mode='direct_pwm', mcu_tick_hz=None, injections=()):
    """The startup JSON object of protocol v1, from a VehicleProfile (SI units: m, rad, s, fractions).

    Per-run values (PER_RUN_DRAW_NAMES) come from profile.per_run_value(seed, name): the seeded draw of a
    distribution or a fixed number (scenario conditions); the native process seeds its own noise streams
    from `seed`.
    """
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError('startup_seed_must_be_u64')
    if mode not in MODES:
        raise ValueError('startup_mode_unknown:%r' % (mode,))
    v = profile.value
    tick = v('sim_mcu_tick_hz') if mcu_tick_hz is None else mcu_tick_hz
    if isinstance(tick, bool) or not isinstance(tick, int) or not 100 <= tick <= 100000:
        raise ValueError('startup_tick_hz_invalid')
    if tick % v('servo_hz'):
        raise ValueError('startup_tick_hz_not_a_multiple_of_servo_hz')
    rad = math.radians
    geom = geometry.SteeringGeometry.from_profile(profile)
    mech_left, mech_right = geom.mech_limits_eq()
    soft_left, soft_right = geom.soft_limits_eq()
    draws = {name: profile.per_run_value(seed, name) for name in PER_RUN_DRAW_NAMES}
    wander, release = v('steer_backlash_wander'), v('steer_release_restoring')
    known = set(INJECTION_PARAMS)
    for kind in v('pot_fault_injections'):
        if 'pot_' + kind not in known:
            raise ValueError('profile_pot_fault_injection_unknown:' + kind)
    servo = dict(servo_hz=v('servo_hz'), kp_per_rad=v('servo_kp_pct_per_deg') / 100. / rad(1.),
                 deadband_rad=rad(v('servo_deadband_deg')), min_pwm=v('servo_min_pwm_pct') / 100.,
                 max_pwm=v('servo_max_pwm_pct') / 100.)
    limits = v('steer_mech_limit_deg')
    faults = dict(
        pot_valid_raw=[v('pot_valid_raw')['min_raw'], v('pot_valid_raw')['max_raw']],
        cal_end_raw_window=[v('cal_end_raw_window')['min_raw'], v('cal_end_raw_window')['max_raw']],
        pot_jump_counts=v('pot_jump_counts')['counts'], pot_jump_window_s=v('pot_jump_counts')['window_ms'] / 1000.,
        pot_noise_sd_counts=v('pot_noise_counts')['sd_counts'],
        pot_noise_window_s=v('pot_noise_counts')['window_ms'] / 1000.,
        polarity_away_rad=rad(v('polarity_fault')['away_deg']), polarity_duration_s=v('polarity_fault')['duration_s'],
        stall_pwm=v('stall_detect')['pwm_pct'] / 100., stall_duration_s=v('stall_detect')['duration_s'],
        stall_min_motion_rad=rad(v('stall_detect')['min_motion_deg']),
        track_error_rad=rad(v('track_fault')['error_deg']), track_duration_s=v('track_fault')['duration_s'],
        driver_stuck_motion_rad=rad(v('driver_stuck_fault')['motion_deg']),
        driver_stuck_duration_s=v('driver_stuck_fault')['duration_s'],
        thermal_pwm=v('thermal_proxy')['pwm_pct'] / 100., thermal_window_s=v('thermal_proxy')['window_s'],
        thermal_accumulated_s=v('thermal_proxy')['accumulated_s'], crc_burst_n=v('crc_burst_n'))
    ramp = v('mcu_ramp')
    return dict(
        protocol=PROTOCOL, mode=mode, seed=seed, mcu_tick_hz=tick, profile_id=profile.profile_id,
        profile_sha256=profile.sha256, profile_overrides=profile.result_label()['overrides'],
        geometry=dict(
            steer_type=v('steer_type'), wheelbase_m=v('wheelbase_m'), kingpin_width_m=v('kingpin_width_m'),
            ackermann_ratio=v('steer_ackermann_ratio'),
            wheel_mech_limit_rad={key: rad(limits[key]) for key in geometry.WHEEL_LIMIT_KEYS},
            eq_mech_limit_left_rad=mech_left, eq_mech_limit_right_rad=mech_right,
            eq_soft_limit_left_rad=soft_left, eq_soft_limit_right_rad=soft_right,
            link_table_rad=geom.link_table.rows_rad()),
        actuator=dict(
            level=v('steer_actuator_level'), rate_max_rad_s=rad(v('steer_rate_max_deg_s')),
            rate_standstill_rad_s=rad(v('steer_rate_standstill_deg_s')), tau_s=v('steer_motor_tau_s'),
            min_pwm=v('steer_min_pwm_pct') / 100., min_pwm_standstill=v('steer_min_pwm_standstill_pct') / 100.,
            backlash_rad=rad(v('steer_backlash_deg')), backlash_wander_sd_rad=rad(wander['sd_deg']),
            backlash_wander_corr_length_m=wander['corr_length_m'], self_locking=v('steer_self_locking'),
            release_scale=release['scale'], release_centering_per_s_per_m_s=release['centering_per_s_per_m_s'],
            release_noise_sd_rad=rad(release['noise_sd_deg']),
            release_noise_corr_length_m=release['noise_corr_length_m'], lr_asymmetry=v('steer_lr_asymmetry'),
            stall_torque_nm=v('steer_motor_stall_torque_nm'), load_moving_nm=v('steer_load_moving_nm'),
            load_standstill_nm=v('steer_load_standstill_nm'), zero_offset_rad=rad(draws['steer_zero_offset_deg'])),
        pot=dict(
            location=v('pot_location'), range_rad=rad(v('pot_range_deg')), to_steer_ratio=v('pot_to_steer_ratio'),
            center_raw=v('pot_center_raw'), adc_bits=v('adc_bits'), ratiometric=v('adc_ratiometric'),
            supply_sag_per_pwm=v('pot_supply_sag_per_pwm'), noise_sd_counts=v('pot_noise_sd_counts'),
            linearity_frac=v('pot_linearity_pct') / 100., gain_error=v('pot_gain_error'),
            drift_rad_per_s=rad(v('pot_drift_deg_per_h')) / 3600.,
            cal_residual_rad=rad(draws['pot_cal_residual_deg'])),
        test_servo=servo,
        mcu=dict(
            servo=servo, dwell_brake_s=v('servo_dwell_brake_s'), reversal_pause_s=v('servo_reversal_pause_ms') / 1000.,
            c_ttl_max_s=v('c_ttl_max_ms') / 1000., ttl_expiry_drive_mode=v('ttl_expiry_drive_mode'),
            watchdog_s=v('watchdog_ms') / 1000., steer_watchdog_mode=v('steer_watchdog_mode'),
            hold_after_loss_s=v('steer_hold_after_loss_s'), hold_after_stop_s=v('steer_hold_after_stop_s'),
            hold_after_disarm_s=v('steer_hold_after_disarm_s'), status_hz=v('status_hz'),
            status_latency_s=v('status_latency_s'), serial_delay_s=v('serial_delay_ms') / 1000., baud=v('baud'),
            drive_ramp_pwm_per_s=ramp['pwm_per_s'], drive_reversal_dwell_s=ramp['reversal_dwell_ms'] / 1000.,
            pot_filter_sample_hz=v('pot_filter')['sample_hz'], pot_filter_median_of=v('pot_filter')['median_of'],
            faults=faults),
        injections=validate_injections(injections))


def startup_line(config):
    """One ASCII JSON line (compact, no NaN) ending in '\\n'."""
    text = json.dumps(config, ensure_ascii=True, allow_nan=False, separators=(',', ':'))
    data = text.encode('ascii') + b'\n'
    if len(data) > MAX_STARTUP_LINE_BYTES:
        raise ValueError('startup_line_too_long')
    return data
