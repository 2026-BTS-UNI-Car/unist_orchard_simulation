"""The ABS-HOLD-8 steering firmware mode of the SIM ESP emulator (profile schema v4; docs/steering/ABS_HOLD8_SIM_KO.md).

The real board (ESP32-S3, three BTS7960 drivers, firmware ABS-HOLD-8-REBOUND-LIMITS, received 2026-11-16) holds an
absolute steering angle with short PWM pulses decided on quiet potentiometer samples. A profile with
steer_fw_mode abs_hold_8 runs that law in the native firmware session (native/abs_hold8.cc, startup block
steer_law) in place of the firmware V3 servo law; the V3 link, arming, TTL, watchdog, STOP and drive stay.

Command scale (the mapping the SIM uses, and the one a host bridge for the real board should use):

  executor steer_mrad (equivalent wheel angle, + left)
    -> host calibration table (host_profile_export: the SIM calibration marks at the equivalent mechanical limit
       - 1 deg are +-1000 permille)                                                    -> S permille (+ left)
    -> firmware ANGLE a = -S x command_max_deg / 1000 (command degrees, + right; +-20 = the board's operating
       references RIGHT 1681 / LEFT 2180 counts, centre 1900)
    -> the SIM host policy (abs_hold_host: step, deadband, minimum interval, poll) decides when an ANGLE is sent
    -> firmware target counts = CENTER + round(span x |a| / 20), span LEFT - CENTER (left) or RIGHT - CENTER (right)

So ANGLE +-20 is the calibration mark angle of the SIM plant (mech limit - 1 deg), not 20 physical degrees: the
physical degrees of the real board's command scale are not measured (the firmware prints "physical degrees
unverified"). The counts per command degree differ per side (right 219 / 20 = 10.95, left 280 / 20 = 14.0), so the
firmware tolerances are 0.91 / 0.71 command degrees (target 10 counts) and 1.46 / 1.14 (restart 16 counts).

Pure Python. Every number is a placeholder of the profile or a firmware constant; results are SIM results.
"""
from __future__ import annotations

import math

from . import vehicle_profile as vp

LAW_KIND = 'abs_hold_8'
# startup block key -> abs_hold_fw key (the native parser requires every key)
_FW_KEYS = ('cal_right', 'cal_center', 'cal_left', 'command_max_deg', 'pulse_pwm', 'pulse_long_ms', 'pulse_short_ms',
            'pulse_long_above_counts', 'settle_ms', 'rebound_window_ms', 'target_tol_counts', 'restart_tol_counts',
            'rebound_max_counts', 'progress_min_counts', 'no_progress_max', 'drive_budget_ms', 'guard_counts',
            'sample_ms', 'median_of', 'pair_tol_counts', 'stale_ms', 'limit_margin_counts')


def active(profile):
    """True when the profile runs the ABS-HOLD-8 law (schema v4, steer_fw_mode abs_hold_8)."""
    return vp.abs_hold_mode(profile)


def startup_block(profile):
    """The native startup block 'steer_law' for a profile in abs_hold_8 mode, else None (the V3 law)."""
    if not active(profile):
        return None
    fw = profile.value('abs_hold_fw')
    host = profile.value('abs_hold_host')
    block = dict(kind=LAW_KIND)
    for key in _FW_KEYS:
        block[key] = float(fw[key]) if key == 'command_max_deg' else int(fw[key])
    block.update(host_step_deg=float(host['step_deg']), host_deadband_deg=float(host['deadband_deg']),
                 host_min_interval_ms=int(host['min_interval_ms']), host_poll_ms=int(host['poll_ms']))
    return block


def counts_per_command_deg(profile):
    """(right, left) ADC counts per command degree of the board's calibration."""
    fw = profile.value('abs_hold_fw')
    return ((fw['cal_center'] - fw['cal_right']) / fw['command_max_deg'],
            (fw['cal_left'] - fw['cal_center']) / fw['command_max_deg'])


def angle_from_permille(profile, permille):
    """Firmware ANGLE (command degrees, + right) of a V3 position / target in permille (+ left)."""
    return -float(permille) * profile.value('abs_hold_fw')['command_max_deg'] / 1000.


def target_counts(profile, angle_deg):
    """The firmware's absolute target (counts) for ANGLE angle_deg: CENTER + round(span x |a| / max)."""
    fw = profile.value('abs_hold_fw')
    span = (fw['cal_left'] - fw['cal_center']) if angle_deg < 0 else (fw['cal_right'] - fw['cal_center'])
    x = span * abs(angle_deg) / fw['command_max_deg']
    return fw['cal_center'] + int(math.floor(x + .5) if x >= 0 else -math.floor(-x + .5))


def mark_angle_deg(profile):
    """Mean equivalent wheel angle (deg) of the SIM calibration marks = ANGLE +-command_max_deg in the SIM."""
    from . import geometry
    from . import host_profile_export as hpe
    left, right = geometry.SteeringGeometry.from_profile(profile).mech_limits_eq()
    inset = math.radians(hpe.MARK_INSET_DEG)
    return math.degrees(((left - inset) + (right - inset)) / 2.)


def effective_rates_deg_s(profile):
    """The measured effective steering rates as equivalent wheel degrees per second in the SIM scale:
    (outward from the centre to ANGLE 20, back from 20 to the centre) = mark angle / measured step time."""
    timing = profile.value('steer_step_time_measured_s')
    mark = mark_angle_deg(profile)
    return mark / float(timing['outward_20']), mark / float(timing['return_20'])


def executor_values(profile):
    """The 'measured' steering values a SIM executor configuration takes in abs_hold_8 mode (the B5 step response of
    the real board, not the motor's rate at full PWM): the outward effective rate for moving and standstill (the
    measurement was taken standing; the slower direction), and the firmware target tolerance as the servo deadband
    (the band where the board stops correcting; V3 db), on the side with fewer counts per degree, in equivalent
    degrees. The restart tolerance (the V3 re-push) is reported too. None for a V3 profile."""
    if not active(profile):
        return None
    outward, back = effective_rates_deg_s(profile)
    right, left = counts_per_command_deg(profile)
    fw = profile.value('abs_hold_fw')
    scale = mark_angle_deg(profile) / fw['command_max_deg']   # equivalent deg per command deg
    tol_deg = fw['target_tol_counts'] / min(right, left) * scale
    restart_deg = fw['restart_tol_counts'] / min(right, left) * scale
    return dict(rate_deg_s=round(outward, 3), rate_standstill_deg_s=round(outward, 3),
                return_rate_deg_s=round(back, 3), deadband_deg=round(tol_deg, 3),
                restart_deg=round(restart_deg, 3))
