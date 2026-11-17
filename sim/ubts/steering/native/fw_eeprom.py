"""EEPROM calibration image of the steering firmware V3 (ESP_PROTOCOL_V3_DRAFT_KO.md sections 4.10, 9) and the
parent-side image builder of LOCKSTEP_PROTOCOL_KO.md section 11.11.

A second, independent writing of the record format (the firmware C++ is the other); tests compare the two.
Standard library only. All parameter values are placeholders: results built on them are placeholder-based
SIM results, not field validation.
"""
from __future__ import annotations

import math
import struct
import zlib

from .. import geometry

RECORD_BYTES = 106
CRC_OFFSET = 102
IMAGE_BYTES = 256
MAGIC_VALID = b'USC3'
MAGIC_TOMBSTONE = b'USCX'
MARK_WINDOW_SPAN_MIN = 50

# (key, default, low, high) in record order (spec 4.10); defaults = placeholder profile with s = 40 permille/deg.
CFG_PARAMS = (
    ('kp', 245, 1, 4096), ('db', 20, 0, 200), ('minp', 102, 0, 255), ('maxp', 255, 1, 255),
    ('dwell', 200, 0, 2000), ('rev', 20, 0, 500), ('rmov', 960, 1, 5000), ('rstd', 480, 1, 5000),
    ('ttlmax', 300, 50, 1000), ('wd', 500, 100, 5000), ('ttlmode', 0, 0, 1), ('wdmode', 0, 0, 2),
    ('hloss', 2000, 0, 10000), ('hstop', 2000, 0, 10000), ('hdis', 2000, 0, 10000), ('stoppol', 0, 0, 2),
    ('sthz', 20, 10, 50), ('med', 5, 1, 9), ('pvmin', 20, 0, 1023), ('pvmax', 1003, 0, 1023),
    ('cwmin', 60, 0, 1023), ('cwmax', 963, 0, 1023), ('jmp', 8, 1, 1023), ('jmpw', 2, 1, 100),
    ('nsd', 50, 1, 1000), ('nsw', 50, 2, 100), ('polp', 40, 1, 1000), ('polt', 100, 1, 1000),
    ('stp', 153, 0, 255), ('stt', 500, 1, 1000), ('stm', 20, 0, 1000), ('trk', 120, 1, 2000),
    ('trt', 1000, 1, 10000), ('dsm', 80, 1, 1000), ('dst', 300, 1, 1000), ('thp', 204, 0, 255),
    ('thw', 30, 1, 60), ('tha', 5000, 1, 60000), ('crcn', 5, 1, 50), ('movms', 300, 0, 2000),
    ('svhz', 500, 500, 1000),
)
CFG_KEYS = tuple(key for key, _, _, _ in CFG_PARAMS)
CFG_VALUE_SETS = {'sthz': (10, 20, 25, 40, 50), 'med': (1, 3, 5, 7, 9), 'svhz': (500, 1000)}
PLACEHOLDER_SCALE_PERMILLE_PER_DEG = 40.
_PARAM = struct.Struct('<41H')


def crc8(data):
    """CRC-8 of the serial lines: poly 0x07, init 0, no reflection (spec 3)."""
    crc = 0
    for byte in bytes(data):
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crc32(data):
    """CRC-32/ISO-HDLC (zlib), spec 9.3."""
    return zlib.crc32(bytes(data)) & 0xFFFFFFFF


def cfg_defaults():
    return {key: default for key, default, _, _ in CFG_PARAMS}


def cfg_value_ok(key, value):
    if isinstance(value, bool) or not isinstance(value, int):
        return False
    if key in CFG_VALUE_SETS:
        return value in CFG_VALUE_SETS[key]
    for name, _, low, high in CFG_PARAMS:
        if name == key:
            return low <= value <= high
    return False


def cfg_cross_ok(params):
    p = params
    return (p['minp'] <= p['maxp'] and p['rstd'] <= p['rmov'] and p['pvmin'] < p['pvmax'] and p['cwmin'] < p['cwmax']
            and p['tha'] <= p['thw'] * 1000)


def check_params(params):
    """All 41 keys present, each value allowed and the bound pairs hold (raises ValueError)."""
    if set(params) != set(CFG_KEYS):
        raise ValueError('cfg_params_keys:%s' % ','.join(sorted(set(params) ^ set(CFG_KEYS))))
    for key in CFG_KEYS:
        if not cfg_value_ok(key, params[key]):
            raise ValueError('cfg_param_out_of_range:%s=%r' % (key, params[key]))
    if not cfg_cross_ok(params):
        raise ValueError('cfg_params_bound_pairs')
    return params


def marks_valid(raw_l, raw_c, raw_r):
    values = (raw_l, raw_c, raw_r)
    if any(isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 1023 for v in values):
        return False
    if not (raw_l > raw_c > raw_r or raw_l < raw_c < raw_r):
        return False
    return abs(raw_l - raw_c) >= MARK_WINDOW_SPAN_MIN and abs(raw_c - raw_r) >= MARK_WINDOW_SPAN_MIN


def encode_record(cal_id, raw_l, raw_c, raw_r, soft_l, soft_r, pol_ok=1, mdir=1, params=None, magic=MAGIC_VALID):
    """The 106-byte record (spec 9.2) with its CRC-32. params: overrides of cfg_defaults()."""
    full = cfg_defaults()
    full.update(params or {})
    check_params(full)
    if magic not in (MAGIC_VALID, MAGIC_TOMBSTONE, 'USC3', 'USCX'):
        raise ValueError('record_magic_unknown:%r' % (magic,))
    magic = magic.encode('ascii') if isinstance(magic, str) else magic
    if not 0 <= cal_id <= 0xFFFF or pol_ok not in (0, 1) or mdir not in (1, -1):
        raise ValueError('record_fields_out_of_range')
    body = magic + struct.pack('<BBHHHHhhBB', 1, RECORD_BYTES, cal_id, raw_l, raw_c, raw_r, soft_l, soft_r, pol_ok,
                               0x01 if mdir == 1 else 0xFF)
    body += _PARAM.pack(*[full[key] for key in CFG_KEYS])
    assert len(body) == CRC_OFFSET
    return body + struct.pack('<I', crc32(body))


def image_bytes(record=None):
    """256-byte EEPROM image: the record at offset 0, then 0xFF (all 0xFF without a record)."""
    data = bytes(record or b'')
    if len(data) > IMAGE_BYTES:
        raise ValueError('eeprom_image_too_long')
    return data + b'\xff' * (IMAGE_BYTES - len(data))


def image_hex(record=None):
    """512 uppercase hex digits of image_bytes(record) (startup firmware.eeprom_hex)."""
    return image_bytes(record).hex().upper()


def decode_record(data):
    """Fields of a record that passes the load check of spec 9.4, else None."""
    data = bytes(data[:RECORD_BYTES])
    if len(data) < RECORD_BYTES or data[:4] != MAGIC_VALID or data[4] != 1 or data[5] != RECORD_BYTES:
        return None
    if crc32(data[:CRC_OFFSET]) != struct.unpack_from('<I', data, CRC_OFFSET)[0]:
        return None
    cal_id, raw_l, raw_c, raw_r, soft_l, soft_r, pol_ok, mdir = struct.unpack_from('<HHHHhhBB', data, 6)
    params = dict(zip(CFG_KEYS, _PARAM.unpack_from(data, 20)))
    if cal_id < 1 or pol_ok > 1 or mdir not in (0x01, 0xFF) or not marks_valid(raw_l, raw_c, raw_r):
        return None
    if not 1 <= soft_l <= 1000 or not -1000 <= soft_r <= -1:
        return None
    try:
        check_params(params)
    except ValueError:
        return None
    if not all(params['cwmin'] <= v <= params['cwmax'] for v in (raw_l, raw_c, raw_r)):
        return None
    return dict(cal_id=cal_id, raw_l=raw_l, raw_c=raw_c, raw_r=raw_r, soft_l=soft_l, soft_r=soft_r, pol_ok=pol_ok,
                mdir=1 if mdir == 0x01 else -1, params=params, crc32=crc32(data[:CRC_OFFSET]))


def raw_to_permille(raw_l, raw_c, raw_r, f):
    """Spec 7.3 (round half away from zero; clamp +-9999); 0 without valid marks."""
    if not marks_valid(raw_l, raw_c, raw_r):
        return 0
    direction = 1 if raw_l > raw_c else -1
    d = (f - raw_c) * direction
    if d >= 0:
        span = abs(raw_l - raw_c)
        p = (2000 * d + span) // (2 * span)
    else:
        span = abs(raw_c - raw_r)
        p = -((2000 * (-d) + span) // (2 * span))
    return max(-9999, min(9999, p))


# --- spec 4.10 conversion and the section 11.11 builder ------------------------------------------------------------------
def _round_half_up(x):
    return int(math.floor(x + .5))


def _round_half_away(x):
    return int(math.floor(x + .5)) if x >= 0 else -int(math.floor(-x + .5))


_POLICY = {'hold_measured': 0, 'hold_target': 1, 'center': 2}
_WATCHDOG_MODE = {'hold': 0, 'center': 1, 'free': 2}
_TTL_MODE = {'brake': 0, 'release': 1}


# The CFG values that switch off the V3 fault monitors the vehicle's steering board (firmware r5) does not have, for a
# profile with steer_monitor_stack esp32_r5 (schema v6, docs/steering/SMOOTH_R5_DISTURBANCE_KO.md). The V3 firmware
# source is not changed (build id 878B5F56): each monitor gets the end of its CFG range where its condition cannot
# become true. POT_JUMP |f[n] - f[n - jmpw]| > 1023 (raw is 0..1023); POT_NOISE sd > 100 raw within nsw (the whole
# steering travel is about 140 raw); POLARITY 1000 permille against the output within 1 tick; DRV_STUCK 1000 permille
# within 1 tick; STALL needs PWM 255 and |motion| < 0; TRACK 2000 permille for 10 s (the host track rule of the stack,
# 2 s, stops first); THERMAL PWM 255 for the whole 60 s window. POT_OPEN / POT_SHORT (pvmin, pvmax) stay: the board too
# stops on an invalid POT (hold cancel, no valid angle for the host).
BOARD_R5_MONITORS_OFF = dict(jmp=1023, nsd=1000, polp=1000, polt=1, dsm=1000, dst=1, stp=255, stm=0, trk=2000,
                             trt=10000, thp=255, thw=60, tha=60000)


def monitor_stack(profile):
    """'v3' (every profile before schema v6) or the profile's steer_monitor_stack."""
    return profile.value('steer_monitor_stack') if profile.has('steer_monitor_stack') else 'v3'


def params_from_profile(profile, scale_permille_per_deg=PLACEHOLDER_SCALE_PERMILLE_PER_DEG):
    """The 41 CFG values from the profile's MCU parameters (spec 4.10 conversion with scale s). A profile with
    steer_monitor_stack esp32_r5 gets BOARD_R5_MONITORS_OFF for the monitors its board does not have."""
    v = profile.value
    s = float(scale_permille_per_deg)

    def pwm(pct):
        return _round_half_up(pct * 255. / 100.)

    def ms(seconds):
        return _round_half_up(seconds * 1000.)

    def permille(deg):
        return _round_half_up(deg * s)
    params = dict(
        kp=_round_half_up(v('servo_kp_pct_per_deg') / 100. * 255. / s * 256.), db=permille(v('servo_deadband_deg')),
        minp=pwm(v('servo_min_pwm_pct')), maxp=pwm(v('servo_max_pwm_pct')), dwell=ms(v('servo_dwell_brake_s')),
        rev=_round_half_up(v('servo_reversal_pause_ms')), rmov=_round_half_up(0.8 * v('steer_rate_max_deg_s') * s),
        rstd=_round_half_up(0.8 * v('steer_rate_standstill_deg_s') * s), ttlmax=_round_half_up(v('c_ttl_max_ms')),
        wd=_round_half_up(v('watchdog_ms')), ttlmode=_TTL_MODE[v('ttl_expiry_drive_mode')],
        wdmode=_WATCHDOG_MODE[v('steer_watchdog_mode')], hloss=ms(v('steer_hold_after_loss_s')),
        hstop=ms(v('steer_hold_after_stop_s')), hdis=ms(v('steer_hold_after_disarm_s')),
        stoppol=_POLICY[v('stop_policy_steer')], sthz=int(v('status_hz')), med=int(v('pot_filter')['median_of']),
        pvmin=int(v('pot_valid_raw')['min_raw']), pvmax=int(v('pot_valid_raw')['max_raw']),
        cwmin=int(v('cal_end_raw_window')['min_raw']), cwmax=int(v('cal_end_raw_window')['max_raw']),
        jmp=int(v('pot_jump_counts')['counts']), jmpw=_round_half_up(v('pot_jump_counts')['window_ms']),
        nsd=_round_half_up(v('pot_noise_counts')['sd_counts'] * 10.),
        nsw=_round_half_up(v('pot_noise_counts')['window_ms']), polp=permille(v('polarity_fault')['away_deg']),
        polt=ms(v('polarity_fault')['duration_s']), stp=pwm(v('stall_detect')['pwm_pct']),
        stt=ms(v('stall_detect')['duration_s']), stm=permille(v('stall_detect')['min_motion_deg']),
        trk=permille(v('track_fault')['error_deg']), trt=ms(v('track_fault')['duration_s']),
        dsm=permille(v('driver_stuck_fault')['motion_deg']), dst=ms(v('driver_stuck_fault')['duration_s']),
        thp=pwm(v('thermal_proxy')['pwm_pct']), thw=_round_half_up(v('thermal_proxy')['window_s']),
        tha=ms(v('thermal_proxy')['accumulated_s']), crcn=int(v('crc_burst_n')), movms=300, svhz=int(v('servo_hz')))
    if monitor_stack(profile) == 'esp32_r5':
        params.update(BOARD_R5_MONITORS_OFF)
    return check_params(params)


def _link_from_delta_eq(startup, delta):
    """Link-side angle of a wheel-side equivalent angle (protocol 7: delta = T(link)(1 + lr s) + zero_offset)."""
    act = startup['actuator']
    u = delta - act['zero_offset_rad']
    lr = act['lr_asymmetry']
    wheel = u / (1. + lr) if u >= 0. else u / (1. - lr)
    rows = startup['geometry']['link_table_rad']
    table = geometry.LinkTable([[math.degrees(x), math.degrees(y)] for x, y in rows])
    return table.pot_from_wheel(wheel)


def pot_raw_noise_free(startup, delta_eq_rad):
    """Protocol 7 raw at a wheel-side equivalent angle: no noise, supply 1, backlash centred (theta_m = theta_l),
    no drift (t = 0), round half away from zero, clipped to 0 .. 2^bits - 1."""
    pot = startup['pot']
    theta = _link_from_delta_eq(startup, delta_eq_rad)
    n = 1 << pot['adc_bits']
    gain = n * pot['to_steer_ratio'] * (1. + pot['gain_error']) / pot['range_rad']
    r0 = pot['center_raw'] + gain * theta
    r1 = r0 + pot['linearity_frac'] * n * math.sin(2. * math.pi * (r0 - pot['center_raw']) / n)
    return min(n - 1, max(0, _round_half_away(r1)))


def calibrated_eeprom(startup, profile, cal_id=1, mdir=1, pol_ok=1):
    """Section 11.11: the EEPROM image of a car calibrated before the run. Returns (image_hex, details)."""
    geo = startup['geometry']
    one_deg = math.radians(1.)
    left = geo['eq_mech_limit_left_rad'] - one_deg
    right = -(geo['eq_mech_limit_right_rad'] - one_deg)
    center = startup['pot']['cal_residual_rad']
    raw_l, raw_c, raw_r = (pot_raw_noise_free(startup, angle) for angle in (left, center, right))
    scale = 1000. / ((abs(math.degrees(left)) + abs(math.degrees(right))) / 2.)
    soft_l = _round_half_away(1000. * geo['eq_soft_limit_left_rad'] / left)
    soft_r = -_round_half_away(1000. * geo['eq_soft_limit_right_rad'] / abs(right))
    params = params_from_profile(profile, scale)
    record = encode_record(cal_id, raw_l, raw_c, raw_r, soft_l, soft_r, pol_ok=pol_ok, mdir=mdir, params=params)
    details = dict(raw_l=raw_l, raw_c=raw_c, raw_r=raw_r, soft_l=soft_l, soft_r=soft_r, scale_permille_per_deg=scale,
                   mark_angles_rad=(left, center, right), params=params, cal_id=cal_id, mdir=mdir, pol_ok=pol_ok,
                   crc32=struct.unpack_from('<I', record, CRC_OFFSET)[0])
    return image_hex(record), details
