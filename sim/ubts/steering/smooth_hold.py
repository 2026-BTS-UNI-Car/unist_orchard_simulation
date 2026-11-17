"""The SMOOTH r5 steering firmware mode of the SIM ESP emulator (profile schema v5; docs/steering/SMOOTH_R5_SIM_KO.md).

The vehicle's board (ESP32-S3, three BTS7960 drivers) runs since 2026-11-16 23:20 the firmware r5 (CAL_RUNTIME=1,
SMOOTH=1, app sha256 f448b2db...) with SM MODE=SMOOTH saved in its NVS: an ANGLE target is held by one continuous run
of the steering motor (PWM floor..PWM_MAX with a ramp and a slow zone near the target, a short brake at the end, a
settle, no-progress floor boost and STALL cancel, takeover of a same-direction target while running, a reversal pause,
and r5's quiet reads: the driver off for QUIET_US every QUIET_MS and the run uses that reading). A profile with
steer_fw_mode smooth_r5 runs that law in the native firmware session (native/smooth_r5.cc, startup block steer_law
with kind smooth_r5) in place of the firmware V3 servo law; the V3 link, arming, TTL, watchdog, STOP and drive stay.

Command scale: as abs_hold.py (ANGLE +-command_max_deg = the SIM calibration marks = the board's CAL RIGHT / LEFT,
centre CAL CENTER; the board's targets are CENTER + lroundf(span x |a| / 20) in float arithmetic). The SIM host policy
(smooth_host) is the SMOOTH rule of the vehicle's bridge (dashboard da39e8a: 0.25 command degree quantum, a new ANGLE at
most every 0.2 s, no force retry of a cancelled target) and of the executor's ESP32 adapter (dad6a93
ESP32_SMOOTH_POLICY).

Schema v6 adds steer_fw_mode smooth_r6p, the SIM proposal r6p (kind smooth_r6p: the r5 block plus the four
smooth_fw_proposal keys; native/smooth_r5.h; docs/steering/SMOOTH_R5_DISTURBANCE_KO.md). It is not the vehicle's
firmware; nothing is flashed.

Pure Python. The firmware values come from the field record field_steer_20261116_2321 and the r5 source; the plant
values stay placeholders; results are SIM results (자리값 기반 SIM 결과).
"""
from __future__ import annotations

import math
import struct

from . import abs_hold
from . import vehicle_profile as vp

LAW_KIND = 'smooth_r5'
PROPOSAL_LAW_KIND = 'smooth_r6p'      # SIM proposal r6p (schema v6)
# startup block key -> smooth_fw / smooth_adc_sim / smooth_host key (the native parser requires every key)
_ADC_KEYS = (('adc_powered_offset_counts', 'powered_offset_counts'), ('adc_burst_counts', 'burst_counts'),
             ('adc_burst_ms', 'burst_ms'), ('adc_burst_period_ms', 'burst_period_ms'))
_HOST_KEYS = (('host_step_deg', 'step_deg'), ('host_deadband_deg', 'deadband_deg'),
              ('host_min_interval_ms', 'min_interval_ms'), ('host_poll_ms', 'poll_ms'))
CANCEL_REASONS = {1: 'ADC STALE/INVALID', 2: 'STALL (no force retry)', 3: 'RUN TIME LIMIT',
                  4: 'REBOUND (no force retry)', 5: 'NO CONVERGENCE (no force retry)', 6: 'QUIET OPERATING LIMIT'}


def active(profile):
    """True when the profile runs the SMOOTH hold (schema v5 smooth_r5; schema v6 also the SIM proposal smooth_r6p)."""
    return vp.smooth_hold_mode(profile)


def proposal(profile):
    """True when the profile runs the SIM proposal r6p of the SMOOTH hold (schema v6, steer_fw_mode smooth_r6p)."""
    return active(profile) and profile.value('steer_fw_mode') == 'smooth_r6p'


def startup_block(profile):
    """The native startup block 'steer_law' (kind smooth_r5, or smooth_r6p with the proposal keys) for a profile in a
    SMOOTH mode, else None."""
    if not active(profile):
        return None
    fw = profile.value('smooth_fw')
    adc = profile.value('smooth_adc_sim')
    host = profile.value('smooth_host')
    block = dict(kind=PROPOSAL_LAW_KIND if proposal(profile) else LAW_KIND)
    for key in vp.SMOOTH_FW_KEYS:
        block[key] = float(fw[key]) if key in vp.SMOOTH_FW_FLOAT_KEYS else int(fw[key])
    for key, name in _ADC_KEYS:
        block[key] = int(adc[name])
    for key, name in _HOST_KEYS:
        block[key] = float(host[name]) if key.endswith('_deg') else int(host[name])
    if proposal(profile):
        prop = profile.value('smooth_fw_proposal')
        for key in vp.SMOOTH_PROPOSAL_KEYS:
            block[key] = int(prop[key])
    return block


def counts_per_command_deg(profile):
    """(right, left) ADC counts per command degree of the board's calibration."""
    fw = profile.value('smooth_fw')
    return ((fw['cal_center'] - fw['cal_right']) / fw['command_max_deg'],
            (fw['cal_left'] - fw['cal_center']) / fw['command_max_deg'])


def _f32(x):
    return struct.unpack('<f', struct.pack('<f', x))[0]


def target_counts(profile, angle_deg):
    """The firmware's absolute target (counts) for ANGLE angle_deg: CENTER + lroundf(span x |a| / 20) with the float
    arithmetic of moveSteeringToAngle (span int, degrees float)."""
    fw = profile.value('smooth_fw')
    span = (fw['cal_left'] - fw['cal_center']) if angle_deg < 0 else (fw['cal_right'] - fw['cal_center'])
    x = _f32(_f32(_f32(float(span)) * _f32(abs(angle_deg))) / _f32(fw['command_max_deg']))
    return fw['cal_center'] + int(math.floor(x + .5) if x >= 0 else -math.floor(-x + .5))


def quantize_command(angle_deg, step_deg, limit_deg=20.):
    """The SIM host's (and the bridge's) rounding: half away from zero to step_deg, clamped to +-limit_deg."""
    value = max(-limit_deg, min(limit_deg, float(angle_deg)))
    if step_deg > 0:
        value = math.floor(abs(value) / step_deg + .5) * step_deg * (1 if value >= 0 else -1)
    value = max(-limit_deg, min(limit_deg, value))
    return 0. if value == 0 else value


def step_rates_deg_s(profile):
    """The measured SMOOTH step rates as equivalent wheel degrees per second in the SIM scale: (outward from the centre
    to ANGLE 20, back from 20 to the centre) = mark angle / measured step time (abs_hold.mark_angle_deg)."""
    timing = profile.value('smooth_step_time_measured_s')
    mark = abs_hold.mark_angle_deg(profile)
    return mark / float(timing['outward_20']), mark / float(timing['return_20'])


def executor_values(profile):
    """The 'measured' steering values a SIM executor configuration takes in smooth_r5 mode: the board's measured SMOOTH
    step response (field record field_steer_20261116_2321, lifted wheels: the slowest full step 0 -> 20, 1.439 s) for
    moving and standstill, and the firmware's stop tolerance as the servo deadband on the side with fewer counts per
    degree, in equivalent degrees; the restart tolerance is reported too. None for any other profile.

    The rates are rounded down to 0.01 deg/s: the SIM executor configuration sets its command slew limit to
    round(0.8 x rate, 3), and the executor refuses a limit above 0.8 x the measured rate (design 6.2); with two decimals
    0.8 x rate has three and the rounding cannot go above it."""
    if not active(profile):
        return None
    outward, back = step_rates_deg_s(profile)
    right, left = counts_per_command_deg(profile)
    fw = profile.value('smooth_fw')
    scale = abs_hold.mark_angle_deg(profile) / fw['command_max_deg']   # equivalent deg per command deg
    tol_deg = fw['stop_tol_counts'] / min(right, left) * scale
    restart_deg = fw['restart_tol_counts'] / min(right, left) * scale
    rate = math.floor(outward * 100.) / 100.
    return dict(rate_deg_s=rate, rate_standstill_deg_s=rate, return_rate_deg_s=math.floor(back * 100.) / 100.,
                deadband_deg=round(tol_deg, 3), restart_deg=round(restart_deg, 3))
