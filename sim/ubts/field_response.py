"""Field-observed drive response for SIM only (2026-10-10 manual orchard drive).

This sits NEXT TO actuator_response.ActuatorResponse (the older qualitative
stress profile); it does not replace it. Every coefficient has provenance:

  MEASURED         read directly from the 2026-10-10 manual orchard records
                   (6 recordings, 57 min; LiDAR scan matching, raw IMU and the
                   ~2 Hz MCU status). Re-derived read-only from the evidence
                   files (the LiDAR and drive extract JSON files).
  ESTIMATED        derived from MEASURED statistics through this model's
                   structure (e.g. a coast deceleration fitted so the plant
                   reproduces the measured stop distance/time).
  SIM_ASSUMED      not measured. A placeholder that the planned 30-60% duty
                   manual test must replace (low-duty speed/yaw, deadband,
                   breakaway ...).
  FIRMWARE_SOURCE  a constant read from rover_control_test/firmware/wheel_pwm/
                   wheel_pwm.ino; the flashed build is identified only by name
                   (UBTS_WHEEL_PWM_V2). The ~2 Hz status cannot resolve it.
  DASHBOARD_SOURCE a constant read from the dashboard static/app.js mixer.
  CATALOGUE        product listing value; the real cut-down footprint is unmeasured.

Duty/PWM is a COMMAND. Nothing here is a wheel speed, encoder or odometry.
The plant owns the truth state; closed-loop estimators and controllers must
consume only the sensor samples it emits (truth is for evaluation metrics).
There is no real-vehicle validation of this model.
"""
from dataclasses import dataclass, fields, replace
import hashlib
import json
import math

import numpy as np


MEASURED = 'MEASURED_20261010'
ESTIMATED = 'ESTIMATED_FROM_20261010'
SIM_ASSUMED = 'SIM_ASSUMED'
FIRMWARE = 'FIRMWARE_SOURCE'
DASHBOARD = 'DASHBOARD_SOURCE'
CATALOGUE = 'CATALOGUE_UNMEASURED'
SOURCES = (MEASURED, ESTIMATED, SIM_ASSUMED, FIRMWARE, DASHBOARD, CATALOGUE)
PROFILE_NAME = 'field_20261010'

# LiDAR-rate (deg/s, absolute) of the 16 later-recording +-100 opposite-duty
# events held >= 1 s (lidar/pivot_events_2026*.json, recordings 040927..051127).
PIVOT_EVENT_RATES_DEG_S = (5.7, 9.3, 11.6, 13.9, 19.6, 20.3, 21.1, 21.3, 22.8, 24.7, 26.1, 26.3, 26.5,
                           27.4, 27.7, 28.1)
# Recording 035952 (stall-prone ground): the >= 1 s opposite-duty events that
# rotated, excluding the 217.4 s event that coincides with a person handling
# the vehicle (camera; ENCODERLESS_DRIVE_RESPONSE_20261013.md section 3).
STALL_PRONE_ROTATING_RATES_DEG_S = (6.5, 7.2, 7.3)

# name: (default, provenance, basis). Two-element tuples are (low, high) ranges.
PARAMETERS = {
    # MCU firmware and manual mixer
    'mcu_ramp_pct_per_s': (100., MEASURED, 'MCU-reported duty mid-ramp slope p50 249 units/s (p10-p90 200-278, '
                           'n=230 consecutive 2 Hz status pairs) = firmware 255/s on 0..255'),
    'mcu_reversal_wait_s': (.12, FIRMWARE, 'wheel_pwm.ino BREAK_MS 120 after any duty reaches 0 (not resolvable at '
                            '2 Hz); the total +100 -> -100 is consistent with the records: status-to-status p50 '
                            '2.06 s (p10-p90 1.79-2.32, n=36), model 2.12 s'),
    'mcu_watchdog_s': (2., FIRMWARE, 'wheel_pwm.ino WATCHDOG_MS 2000: checkWatchdog() -> stopDirections() writes duty 0 '
                       'IMMEDIATELY (no fade); no watchdog trip occurs in the records'),
    'mcu_status_hz': (2., MEASURED, 'status interval p50 0.515 s (p10-p90 0.504-0.524), periodic, not on change'),
    'mixer_outside_ratio': (1.5, DASHBOARD, 'static/app.js roverMotionVector(): outside=min(100,1.5*speed), '
                            'inside=speed; the records only contain 100/100, 90/100 and 100/90'),
    # forward response
    'speed_at_100_m_s': (.90, MEASURED, 'straight 100/100 LiDAR forward speed p50 0.899 m/s (p10-p90 0.727-0.999, '
                         'n=11211 settled 0.4 s samples, 6 recordings); GNSS antenna speed ~0.08 m/s lower, so the '
                         'LiDAR scale is not independently verified'),
    'speed_at_90_m_s': (.72, MEASURED, 'straight 90/90 p50 0.715 m/s (n=582) from recording 035952 ONLY; the same '
                        'recording\'s 100/100 p50 was 0.672, so the 90->100 slope is not resolved by the data'),
    'forward_deadband_pct': (50., SIM_ASSUMED, 'placeholder: the line through 0.72@90% (035952 only) and 0.90@100% '
                             '(all recordings) reaches 0 at 50%; the points come from different recordings and the '
                             'within-035952 slope is not positive; no duty below 90% was driven'),
    'speed_run_scale_mode': (1.03, ESTIMATED, 'per-run speed scale, two-piece normal (mode)'),
    'speed_run_scale_sd_low': (.16, ESTIMATED, 'lower-side spread; with the high side and fluctuation fitted so 1 s '
                               'plant samples give p10/p50/p90 ~0.74/0.90/1.00 vs LiDAR 0.727/0.899/0.999 (the slow '
                               'tail is mostly recording 035952, p50 0.672)'),
    'speed_run_scale_sd_high': (.04, ESTIMATED, 'upper-side spread of the per-run speed scale'),
    'speed_fluctuation_sd': (.05, ESTIMATED, 'within-run relative speed fluctuation; field within-run relative sd of '
                             '0.4 s LiDAR samples p50 0.037 (includes estimate noise); the split is assumed'),
    'speed_fluctuation_tau_s': (2., SIM_ASSUMED, 'correlation time of speed fluctuation'),
    'lateral_slip_sd_m_s': (.032, ESTIMATED, 'straight 100/100 lateral p10/p90 -0.041/+0.041 m/s of 0.4 s LiDAR '
                            'samples -> sigma 0.041/1.2816 = 0.032 (includes LiDAR estimate noise; standstill '
                            'p10/p90 -0.008/+0.007)'),
    'arc_speed_loss_per_pct_diff': (0., SIM_ASSUMED, 'no speed loss is resolved: arc 90/100 0.752 m/s was FASTER than '
                                    'straight 90/90 0.715 in the same recording; only a 10% difference was ever '
                                    'commanded; sensitivity scenario uses 0.0074'),
    'accel_tau_s': (.25, SIM_ASSUMED, 'start-up lag; acceleration not measured'),
    'coast_onset_s': (.25, ESTIMATED, 'speed held after the duty starts to fall, then the coast deceleration; fitted '
                      'with coast_decel_* to the clean fade-to-zero stops (from release, fade start located from the '
                      'MCU-reported duty): 0.559/0.566/0.595 m in 0.89-0.90 s from 0.90-0.95 m/s, 0.383 m in 0.84 s '
                      'from 0.757 m/s'),
    'coast_decel_median_m_s2': (1.3, ESTIMATED, 'per-stop coast deceleration (log-normal median); with coast_onset_s '
                                'the plant gives 0.560 m / 0.88 s (p50) from release at 0.88-0.97 m/s and 0.42 m / '
                                '0.76 s at 0.70-0.80 m/s; NOT a brake model'),
    'coast_decel_log_sd': (.15, ESTIMATED, 'per-stop spread; clean stops from 0.90-0.95 m/s span 0.56-0.60 m (n=3)'),
    'decel_tau_s': (.08, SIM_ASSUMED, 'short lag while slowing, under the coast deceleration limit'),
    # yaw response while moving
    'arc_yaw_deg_s_per_pct_diff': (.12, ESTIMATED, 'arc 90/100 (diff 10%) LiDAR yaw p50 0.96 deg/s (IMU 0.98) net of the '
                                   'mean straight drift (~-0.2 deg/s); 100/90 -1.48 (recording 035952 only)'),
    'arc_yaw_low_duty_scale': (1., SIM_ASSUMED, 'differential gain below 90% relative to 90-100%; unmeasured'),
    'yaw_tau_s': (.12, ESTIMATED, 'rotation settles p50/p90 0.03/0.31-0.54 s after both sides report 0'),
    'wander_mean_deg_per_m': (-.15, ESTIMATED, 'mean of the straight-line curvature (OU) process = LiDAR run mean; '
                              'straight runs (92, >= 3 s): LiDAR deg/m mean -0.153, median -0.23, p10/p90 -1.33/+0.90, '
                              'right share 0.60; IMU mean -0.10, median -0.11, right share 0.565; no constant trim'),
    'wander_sd_deg_per_m': (.95, ESTIMATED, 'stationary sd; 3-8 s plant segments (600 seeds): deg/m p10/p50/p90 '
                            '-1.23/-0.19/+0.90, right share 0.60 (120-seed blocks 0.56-0.64), chord offset p50/p90 '
                            '0.042/0.109 m vs LiDAR 0.041/0.103'),
    'wander_corr_length_m': (8., SIM_ASSUMED, 'distance correlation of straight-line curvature bias'),
    # pivot (only +-100 % held >= 1 s was measured)
    'pivot_rate_events_deg_s': (PIVOT_EVENT_RATES_DEG_S, MEASURED, '16/16 later +-100 events >= 1 s rotated; LiDAR '
                                'event rates min/p10/p50/p90/max 5.7/10.5/22.1/27.6/28.1 deg/s (IMU 4.8-27.4); all 37 '
                                'later full-duty events (incl. 0.5 s) reach 34.6; right pivots slower in 042301/045553'),
    'pivot_rate_jitter_log_sd': (.10, SIM_ASSUMED, 'per-event multiplicative jitter around a resampled measured event '
                                 'rate (no clip to an inter-quartile range)'),
    'pivot_rate_stall_prone_deg_s': (STALL_PRONE_ROTATING_RATES_DEG_S, ESTIMATED, 'recording 035952: the 3 rotating '
                                     '>= 1 s opposite-duty events without human contact turned at 6.5/7.2/7.3 deg/s'),
    'pivot_min_duty_pct': (80., SIM_ASSUMED, 'opposite duty needed to rotate at all; only +-100 measured'),
    'pivot_breakaway_delay_s': ((.1, .5), SIM_ASSUMED, 'delay after duty passes pivot_min_duty'),
    'pivot_icr_behind_lidar_m': (.18, ESTIMATED, 'pooled per-sample median 0.181 m behind the LiDAR (right class 0.158, '
                                 'left 0.264, per-event median 0.158); includes slip/translation during the pivot'),
    'pivot_icr_sd_m': (.25, ESTIMATED, 'sd of the per-event median ICR 0.249 m (28 events with >= 3 samples)'),
    'pivot_vibration_rms_m_s2': ((3., 6.), MEASURED, 'IMU vibration RMS of the later +-100 events >= 1 s: IQR 3.0-6.4 '
                                 '(p10 1.8, max 8.0)'),
    'still_vibration_rms_m_s2': ((.02, .3), MEASURED, 'IMU vibration RMS while still/stalled: settled 0.5 s stop '
                                 'samples p25/p50/p75 0.02/0.03/0.26, stop segments 0.29; drawn log-uniform per run'),
    'drive_vibration_at_100_m_s2': (3.0, MEASURED, 'straight 100/100 vibration RMS p50 3.01 (settled 0.5 s samples, '
                                    'n=3958; segment p50 3.18)'),
    'drive_vibration_at_90_m_s2': (1.7, MEASURED, 'straight 90/90 p50 1.71 (n=186, recording 035952 only); arc 90/100 1.57'),
    'drive_vibration_rms_m_s2': (1.7, SIM_ASSUMED, 'vibration RMS used below 90% duty (no lower duty was driven); '
                                 'held at the lowest measured driving level'),
    'pivot_stall_probability_normal': (.05, SIM_ASSUMED, '0/16 later events stalled; kept non-zero (0/16 cannot exclude)'),
    'pivot_stall_probability_stall_prone': (.75, ESTIMATED, 'recording 035952 only: 8 of 11 >= 1 s opposite-duty events '
                                            'without human contact did not rotate; clustered retries at 3 spots '
                                            '(207-217 s road edge, 248-268 s grass, 305 s), so not independent '
                                            'trials; cause (torque vs driver protection) unconfirmed, no surface labels'),
    'stall_late_breakaway_probability': (0., SIM_ASSUMED, 'the only late breakaway in the records (035952 ~220 s) '
                                         'coincides with a person handling the vehicle; default 0, sensitivity only'),
    'stall_late_breakaway_s': ((2., 7.), SIM_ASSUMED, 'used only when the late-breakaway probability is overridden; the '
                               'longest full-duty span without rotation observed is ~2.6 s'),
    'pivot_low_duty_forward_gain': (.3, SIM_ASSUMED, 'forward drift from unequal opposite duty'),
    # stop targets (request 0 incl. MCU fade) -- reproduced, not a brake
    'stop_from_release': ((.56, .60), MEASURED, 'clean fade-to-zero stops from 0.90-0.95 m/s, distance from the release '
                          '(fade start located from MCU-reported duty): 0.559/0.566/0.595 m, 0.89-0.90 s (n=3); '
                          'from 0.757 m/s 0.383 m / 0.84 s'),
    'stop_evidence_procedure': ((.342, .548), MEASURED, 'LIDAR_RESPONSE stops_after_forward: distance p50/p90 from the '
                                'first 2 Hz status with request 0 (lags the release by U(0, 0.515) s); 10 events with '
                                'mixed pre-speeds 0.08-0.945 m/s incl. a hard stop, a re-press and a pivot transition; '
                                'time p50/p90 0.71/0.96 s'),
    # IMU (raw BNO085 ~20 Hz)
    'imu_hz': (20., MEASURED, 'raw IMU dt median 0.050 s'),
    'gyro_scale_mean': (1.013, ESTIMATED, 'LiDAR/IMU yaw-rate ratio pooled 0.987 (per recording 0.938-0.996); '
                        'attributing the ratio to the gyro (not to the un-deskewed LiDAR) is an assumption'),
    'gyro_scale_sd': (.02, ESTIMATED, 'sd of the per-recording inverse ratios (1.004-1.066)'),
    'gyro_bias_sd_deg_s': (.03, ESTIMATED, 'stop-segment median gyro bias per recording -0.009..-0.041 deg/s'),
    'gyro_still_noise_sd_deg_s': (.09, MEASURED, 'gravity-projected gyro 0.5 s std while stopped, p50 0.09 deg/s'),
    'gyro_vibration_deg_s_per_m_s2': (1.5, ESTIMATED, '0.5 s gyro std / vibration RMS: straight100 4.11/3.01=1.37, '
                                      'straight90 3.06/1.71=1.79, arc 2.75/1.57=1.75, pivot right 6.24/6.14=1.02'),
    # geometry
    'lidar_x_in_base_m': (.18, SIM_ASSUMED, 'rover.yaml lidar.x_m; mount lever arm unmeasured'),
    'lidar_hz': (10., MEASURED, 'LiDAR 10 Hz, 3240 beams, 0.60 m high'),
    'footprint_length_m': (1.22, CATALOGUE, 'catalogue 1.22 m; real swept footprint unmeasured'),
    'footprint_width_m': (.70, CATALOGUE, 'catalogue 0.70 m; real swept footprint unmeasured'),
}
# Reported targets/evidence, not plant inputs.
REPRODUCED_TARGETS = ('mcu_status_hz', 'stop_from_release', 'stop_evidence_procedure', 'lidar_hz')
_EVENT_LISTS = ('pivot_rate_events_deg_s', 'pivot_rate_stall_prone_deg_s')

UNKNOWN_TO_REPLACE = (
    'speed and yaw response below 90% duty (forward_deadband_pct, arc_yaw_low_duty_scale)',
    'breakaway/deadband per side and per surface',
    'arc response and speed loss at lower base duty and larger differentials',
    'per-surface pivot stall probability and pivot response below +-100%',
    'MCU hard-stop physical braking (only fade-to-zero stops were measured)',
    'real swept footprint, LiDAR/IMU lever arm and pivot centre per surface',
    'IMU vibration below 90% duty',
)


@dataclass(frozen=True)
class FieldResponseParams:
    mcu_ramp_pct_per_s: float = PARAMETERS['mcu_ramp_pct_per_s'][0]
    mcu_reversal_wait_s: float = PARAMETERS['mcu_reversal_wait_s'][0]
    mcu_watchdog_s: float = PARAMETERS['mcu_watchdog_s'][0]
    mixer_outside_ratio: float = PARAMETERS['mixer_outside_ratio'][0]
    speed_at_100_m_s: float = PARAMETERS['speed_at_100_m_s'][0]
    speed_at_90_m_s: float = PARAMETERS['speed_at_90_m_s'][0]
    forward_deadband_pct: float = PARAMETERS['forward_deadband_pct'][0]
    speed_run_scale_mode: float = PARAMETERS['speed_run_scale_mode'][0]
    speed_run_scale_sd_low: float = PARAMETERS['speed_run_scale_sd_low'][0]
    speed_run_scale_sd_high: float = PARAMETERS['speed_run_scale_sd_high'][0]
    speed_fluctuation_sd: float = PARAMETERS['speed_fluctuation_sd'][0]
    speed_fluctuation_tau_s: float = PARAMETERS['speed_fluctuation_tau_s'][0]
    lateral_slip_sd_m_s: float = PARAMETERS['lateral_slip_sd_m_s'][0]
    arc_speed_loss_per_pct_diff: float = PARAMETERS['arc_speed_loss_per_pct_diff'][0]
    accel_tau_s: float = PARAMETERS['accel_tau_s'][0]
    coast_onset_s: float = PARAMETERS['coast_onset_s'][0]
    coast_decel_median_m_s2: float = PARAMETERS['coast_decel_median_m_s2'][0]
    coast_decel_log_sd: float = PARAMETERS['coast_decel_log_sd'][0]
    decel_tau_s: float = PARAMETERS['decel_tau_s'][0]
    arc_yaw_deg_s_per_pct_diff: float = PARAMETERS['arc_yaw_deg_s_per_pct_diff'][0]
    arc_yaw_low_duty_scale: float = PARAMETERS['arc_yaw_low_duty_scale'][0]
    yaw_tau_s: float = PARAMETERS['yaw_tau_s'][0]
    wander_mean_deg_per_m: float = PARAMETERS['wander_mean_deg_per_m'][0]
    wander_sd_deg_per_m: float = PARAMETERS['wander_sd_deg_per_m'][0]
    wander_corr_length_m: float = PARAMETERS['wander_corr_length_m'][0]
    pivot_rate_events_deg_s: tuple = PARAMETERS['pivot_rate_events_deg_s'][0]
    pivot_rate_jitter_log_sd: float = PARAMETERS['pivot_rate_jitter_log_sd'][0]
    pivot_rate_stall_prone_deg_s: tuple = PARAMETERS['pivot_rate_stall_prone_deg_s'][0]
    pivot_min_duty_pct: float = PARAMETERS['pivot_min_duty_pct'][0]
    pivot_breakaway_delay_s: tuple = PARAMETERS['pivot_breakaway_delay_s'][0]
    pivot_icr_behind_lidar_m: float = PARAMETERS['pivot_icr_behind_lidar_m'][0]
    pivot_icr_sd_m: float = PARAMETERS['pivot_icr_sd_m'][0]
    pivot_vibration_rms_m_s2: tuple = PARAMETERS['pivot_vibration_rms_m_s2'][0]
    still_vibration_rms_m_s2: tuple = PARAMETERS['still_vibration_rms_m_s2'][0]
    drive_vibration_at_100_m_s2: float = PARAMETERS['drive_vibration_at_100_m_s2'][0]
    drive_vibration_at_90_m_s2: float = PARAMETERS['drive_vibration_at_90_m_s2'][0]
    drive_vibration_rms_m_s2: float = PARAMETERS['drive_vibration_rms_m_s2'][0]
    pivot_stall_probability_normal: float = PARAMETERS['pivot_stall_probability_normal'][0]
    pivot_stall_probability_stall_prone: float = PARAMETERS['pivot_stall_probability_stall_prone'][0]
    stall_late_breakaway_probability: float = PARAMETERS['stall_late_breakaway_probability'][0]
    stall_late_breakaway_s: tuple = PARAMETERS['stall_late_breakaway_s'][0]
    pivot_low_duty_forward_gain: float = PARAMETERS['pivot_low_duty_forward_gain'][0]
    imu_hz: float = PARAMETERS['imu_hz'][0]
    gyro_scale_mean: float = PARAMETERS['gyro_scale_mean'][0]
    gyro_scale_sd: float = PARAMETERS['gyro_scale_sd'][0]
    gyro_bias_sd_deg_s: float = PARAMETERS['gyro_bias_sd_deg_s'][0]
    gyro_still_noise_sd_deg_s: float = PARAMETERS['gyro_still_noise_sd_deg_s'][0]
    gyro_vibration_deg_s_per_m_s2: float = PARAMETERS['gyro_vibration_deg_s_per_m_s2'][0]
    lidar_x_in_base_m: float = PARAMETERS['lidar_x_in_base_m'][0]
    footprint_length_m: float = PARAMETERS['footprint_length_m'][0]
    footprint_width_m: float = PARAMETERS['footprint_width_m'][0]

    def __post_init__(self):
        for item in fields(self):
            value = getattr(self, item.name)
            values = value if isinstance(value, tuple) else (value,)
            if (not values or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                  or not math.isfinite(v) for v in values)):
                raise ValueError('invalid_field_response_parameter:' + item.name)
            if item.name in _EVENT_LISTS:
                if any(v <= 0 for v in values):
                    raise ValueError('invalid_field_response_events:' + item.name)
            elif isinstance(value, tuple) and (len(value) != 2 or value[0] > value[1]):
                raise ValueError('invalid_field_response_range:' + item.name)
        if not 0 < self.mcu_ramp_pct_per_s <= 1000 or not 0 <= self.mcu_reversal_wait_s <= 2:
            raise ValueError('invalid_mcu_ramp')
        if not 0 < self.forward_deadband_pct < 90 or not 0 < self.speed_at_90_m_s < self.speed_at_100_m_s <= 3:
            raise ValueError('invalid_speed_curve')
        if (not 0 < self.pivot_min_duty_pct <= 100 or not 0 < self.yaw_tau_s <= 2
                or self.accel_tau_s <= 0 or self.decel_tau_s <= 0 or self.coast_decel_median_m_s2 <= 0
                or not 0 <= self.coast_onset_s <= 1):
            raise ValueError('invalid_pivot_or_lag')
        for name in ('pivot_stall_probability_normal', 'pivot_stall_probability_stall_prone',
                     'stall_late_breakaway_probability'):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError('invalid_probability:' + name)

    def pivot_stall_probability(self, surface):
        if surface == 'normal':
            return self.pivot_stall_probability_normal
        if surface == 'stall_prone':
            return self.pivot_stall_probability_stall_prone
        raise ValueError('unknown_surface')

    def pivot_rates(self, surface):
        self.pivot_stall_probability(surface)
        return self.pivot_rate_stall_prone_deg_s if surface == 'stall_prone' else self.pivot_rate_events_deg_s

    def drive_vibration(self, duty_pct):
        """Vibration RMS while driving: MEASURED at 90/100 %, SIM_ASSUMED below 90 %."""
        duty = abs(duty_pct)
        if duty < 90.:
            return self.drive_vibration_rms_m_s2
        a, b = self.drive_vibration_at_90_m_s2, self.drive_vibration_at_100_m_s2
        return a + (min(duty, 100.) - 90.) * (b - a) / 10.


def provenance(params=None):
    """Value/provenance/basis for every coefficient; overrides are flagged."""
    params = params or FieldResponseParams()
    table = {}
    defaults = FieldResponseParams()
    for item in fields(params):
        default, source, basis = PARAMETERS[item.name]
        value = getattr(params, item.name)
        changed = value != getattr(defaults, item.name)
        table[item.name] = dict(value=list(value) if isinstance(value, tuple) else value,
                                provenance='SCENARIO_OVERRIDE_OF_' + source if changed else source,
                                basis=basis)
    for key in REPRODUCED_TARGETS:
        default, source, basis = PARAMETERS[key]
        table[key] = dict(value=list(default) if isinstance(default, tuple) else default,
                          provenance=source, basis=basis, reproduced_target=True)
    counts = {}
    for entry in table.values():
        counts[entry['provenance']] = counts.get(entry['provenance'], 0) + 1
    return dict(profile=PROFILE_NAME, parameters=table, counts=dict(sorted(counts.items())),
                unknown_to_replace=list(UNKNOWN_TO_REPLACE),
                pwm_is_wheel_speed=False, encoder=False, real_vehicle_validated=False)


def provenance_digest(params=None):
    """Stable SHA-256 of the provenance table (for compact status messages)."""
    payload = json.dumps(provenance(params), sort_keys=True, separators=(',', ':'))
    return hashlib.sha256(payload.encode('utf-8')).hexdigest()


def js_round(value):
    """JavaScript Math.round (half toward +inf), used by the dashboard mixer."""
    return int(math.floor(value + .5))


def manual_mix(speed_pct, forward, turn, outside_ratio=PARAMETERS['mixer_outside_ratio'][0]):
    """Port of dashboard roverMotionVector(): forward W=+1/S=-1, turn D=+1/A=-1.

    Diagonals keep the inside wheel at the selected speed and multiply the
    outside wheel by 1.5 (clipped at 100); at speed 100 W+A equals W.
    A/D alone is an opposite-duty pivot. Returns signed integer percents.
    """
    if forward not in (-1, 0, 1) or turn not in (-1, 0, 1):
        raise ValueError('invalid_manual_keys')
    if not 0 <= speed_pct <= 100:
        raise ValueError('invalid_manual_speed')
    if forward and turn:
        outside = min(100., speed_pct * outside_ratio)
        return (js_round(forward * (outside if turn > 0 else speed_pct)),
                js_round(forward * (outside if turn < 0 else speed_pct)))
    return js_round((forward + turn) * speed_pct), js_round((forward - turn) * speed_pct)


class McuRamp:
    """Per-side MCU duty ramp in percent with the firmware break and watchdog.

    MEASURED: 100 %/s per side (status slope). FIRMWARE_SOURCE (wheel_pwm.ino):
    a sign reversal fades to 0; any restart from 0 waits BREAK_MS 120 ms; no
    command for WATCHDOG_MS 2 s zeroes the duty IMMEDIATELY (stopDirections()),
    as hard_stop() does. The PHYSICAL braking of an immediate zero is unmeasured.
    """

    def __init__(self, params=None):
        self.params = params or FieldResponseParams()
        self.applied = [0., 0.]
        self.target = [0., 0.]
        self.hold_until = [-math.inf, -math.inf]
        self.last_sign = [0, 0]
        self.last_command_s = None
        self.watchdog_tripped = False

    def command(self, left, right, now_s):
        values = []
        for value in (left, right):
            if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
                raise ValueError('mcu_duty_must_be_integer_percent')
            values.append(float(max(-100, min(100, int(value)))))
        self.target = values
        self.last_command_s = float(now_s)
        self.watchdog_tripped = False

    def hard_stop(self, now_s=None):
        self.target = [0., 0.]
        if now_s is not None:
            for side in range(2):
                if self.applied[side] != 0.:
                    self.hold_until[side] = now_s + self.params.mcu_reversal_wait_s
        self.applied = [0., 0.]

    def step(self, now_s, dt):
        if dt <= 0:
            return tuple(self.applied)
        if (self.last_command_s is not None and not self.watchdog_tripped
                and now_s - self.last_command_s > self.params.mcu_watchdog_s):
            self.watchdog_tripped = True
            self.hard_stop(now_s)            # firmware stopDirections(): duty 0 at once
            return tuple(self.applied)
        change = self.params.mcu_ramp_pct_per_s * dt
        for side in range(2):
            applied, target = self.applied[side], self.target[side]
            if applied != 0. and target * applied < 0:
                # Reversal: fade through zero first, never jump across it.
                new = math.copysign(max(0., abs(applied) - change), applied)
            elif applied == 0. and target != 0. and now_s < self.hold_until[side]:
                new = 0.  # firmware BREAK_MS after any duty reached zero
            else:
                new = applied + max(-change, min(change, target - applied))
            if applied != 0. and new == 0.:
                self.hold_until[side] = now_s + self.params.mcu_reversal_wait_s
            if new != 0.:
                self.last_sign[side] = 1 if new > 0 else -1
            self.applied[side] = new
        return tuple(self.applied)

    def reported_units(self):
        """Duty as the MCU reports it (0..255 magnitude with sign)."""
        return tuple(int(round(v * 255 / 100)) for v in self.applied)


def forward_curve(duty_pct, params):
    """Steady straight speed for |duty|; MEASURED >= 90%, SIM_ASSUMED below."""
    duty = abs(duty_pct)
    p = params
    if duty <= p.forward_deadband_pct:
        return 0.
    if duty < 90.:
        return p.speed_at_90_m_s * (duty - p.forward_deadband_pct) / (90. - p.forward_deadband_pct)
    return p.speed_at_90_m_s + (duty - 90.) * (p.speed_at_100_m_s - p.speed_at_90_m_s) / 10.


@dataclass
class PivotEvent:
    direction: int
    rate_rad_s: float
    icr_behind_lidar_m: float
    stalled: bool
    late_breakaway_s: float | None
    breakaway_delay_s: float
    vibration_rms: float
    loaded_s: float = 0.
    rotated: bool = False


def _seed_sequence(seed):
    """int or tuple/list of non-negative ints -> numpy SeedSequence."""
    if isinstance(seed, np.random.SeedSequence):
        return seed
    entropy = [seed] if isinstance(seed, (int, np.integer)) else list(seed)
    if not entropy or any(isinstance(v, bool) or not isinstance(v, (int, np.integer)) or v < 0 for v in entropy):
        raise ValueError('invalid_plant_seed')
    return np.random.SeedSequence([int(v) for v in entropy])


class _NormalBlock:
    """Standard normals drawn in blocks from ONE dedicated generator."""

    def __init__(self, rng):
        self.rng = rng
        self.block = np.empty(0)
        self.index = 0

    def __call__(self):
        if self.index >= len(self.block):
            self.block = self.rng.standard_normal(4096)
            self.index = 0
        self.index += 1
        return float(self.block[self.index - 1])


class FieldPlant:
    """Stateful SE(2) skid-steer plant driven by integer duty commands.

    State is the footprint-centre ("base") pose. Response: MCU ramp ->
    forward_curve/arc/pivot steady targets -> first-order lags -> kinematic
    integration with the pivot-centre lever arm. Sensor outputs (IMU) carry
    noise/bias/vibration; truth() is for evaluation only.

    Randomness: one SeedSequence per run, spawned into independent streams
    for run constants, process noise, pivot-event outcomes (drawn per event,
    so the k-th pivot's stall/rate does not depend on timing) and sensor noise.
    """

    def __init__(self, params=None, *, surface='normal', seed=0, pose=(0., 0., 0.)):
        self.params = params or FieldResponseParams()
        self.params.pivot_stall_probability(surface)
        self.surface = surface
        run_ss, process_ss, pivot_ss, sensor_ss = _seed_sequence(seed).spawn(4)
        self.rng = np.random.default_rng(run_ss)
        self.normal = _NormalBlock(np.random.default_rng(process_ss))
        self._sensor_normal = _NormalBlock(np.random.default_rng(sensor_ss))
        self._pivot_rng = np.random.default_rng(pivot_ss)
        p = self.params
        self.mcu = McuRamp(p)
        self.time_s = 0.
        self.x, self.y, self.yaw = map(float, pose)
        self.v = 0.
        self.w = 0.
        self.lateral = 0.
        self.distance_m = 0.
        z = float(self.rng.normal())
        spread = p.speed_run_scale_sd_low if z < 0 else p.speed_run_scale_sd_high
        self.speed_run_scale = float(np.clip(p.speed_run_scale_mode + spread * z, .5, 1.25))
        self.speed_fluctuation = 0.
        self.wander_deg_per_m = float(self.rng.normal(p.wander_mean_deg_per_m, p.wander_sd_deg_per_m))
        self.coast_decel = p.coast_decel_median_m_s2
        self.coast_since = None
        self.decelerating = False
        self.gyro_scale = float(self.rng.normal(p.gyro_scale_mean, p.gyro_scale_sd))
        self.gyro_bias = math.radians(float(self.rng.normal(0., p.gyro_bias_sd_deg_s)))
        self.still_vibration = float(math.exp(self.rng.uniform(*np.log(p.still_vibration_rms_m_s2))))
        self.pivot = None
        self.pivot_count = 0
        self.pivot_stalls = 0
        self.pivot_log = []                # (direction, rate_deg_s, stalled) per event, evaluation only
        self.regime = 'still'
        self.vibration_rms = self.still_vibration
        self.accel = 0.
        self.mean_duty = 0.
        # SIM disturbance (e.g. a skid on a root): extra body yaw rate that the
        # gyro does sense; set by a scenario, zero otherwise.
        self.external_yaw_rate = 0.
        # SIM disturbance: forward motion blocked (root/step below the scan plane).
        self.forward_blocked = False

    # --- command side -------------------------------------------------
    def command(self, left, right):
        self.mcu.command(left, right, self.time_s)

    def hard_stop(self):
        self.mcu.hard_stop(self.time_s)

    # --- dynamics -----------------------------------------------------
    def _new_pivot(self, direction):
        """One pivot event from a fixed block of uniforms on the dedicated stream."""
        p = self.params
        u = self._pivot_rng.random(8)
        rates = p.pivot_rates(self.surface)
        base = rates[min(len(rates) - 1, int(u[0] * len(rates)))]
        jitter = math.exp(p.pivot_rate_jitter_log_sd * _normal_from_uniform(u[1]))
        rate = base * jitter
        stalled = bool(u[2] < p.pivot_stall_probability(self.surface))
        late = (p.stall_late_breakaway_s[0] + u[4] * (p.stall_late_breakaway_s[1] - p.stall_late_breakaway_s[0])
                if stalled and u[3] < p.stall_late_breakaway_probability else None)
        icr = float(np.clip(p.pivot_icr_behind_lidar_m + p.pivot_icr_sd_m * _normal_from_uniform(u[5]), -.6, .9))
        delay = p.pivot_breakaway_delay_s[0] + u[6] * (p.pivot_breakaway_delay_s[1] - p.pivot_breakaway_delay_s[0])
        vibration = p.pivot_vibration_rms_m_s2[0] + u[7] * (p.pivot_vibration_rms_m_s2[1] - p.pivot_vibration_rms_m_s2[0])
        self.pivot_count += 1
        self.pivot_stalls += stalled
        self.pivot_log.append((direction, round(rate, 2), stalled))
        return PivotEvent(direction, math.radians(rate), icr, stalled, late, delay, vibration)

    def _targets(self, left, right, dt):
        """Steady (v, w, icr_x_in_base, regime) for the applied duties."""
        p = self.params
        icr_x = p.lidar_x_in_base_m - p.pivot_icr_behind_lidar_m
        if left * right < 0:
            direction = 1 if right > 0 else -1
            if self.pivot is None or self.pivot.direction != direction:
                self.pivot = self._new_pivot(direction)
            event = self.pivot
            icr_x = p.lidar_x_in_base_m - event.icr_behind_lidar_m
            loaded = min(abs(left), abs(right))
            if loaded >= p.pivot_min_duty_pct:
                event.loaded_s += dt
            else:
                event.loaded_s = 0.
            free = event.loaded_s >= event.breakaway_delay_s and (
                not event.stalled or (event.late_breakaway_s is not None
                                      and event.loaded_s >= event.late_breakaway_s))
            if not free:
                return 0., 0., icr_x, 'pivot_stalled' if loaded >= p.pivot_min_duty_pct else 'pivot_below_breakaway'
            event.rotated = True
            fraction = .5 + .5 * (loaded - p.pivot_min_duty_pct) / max(1e-9, 100. - p.pivot_min_duty_pct)
            v = p.pivot_low_duty_forward_gain * p.speed_at_100_m_s * (left + right) / 200.
            return v, direction * event.rate_rad_s * fraction, icr_x, 'pivot'
        if self.pivot is not None and left == 0 and right == 0:
            self.pivot = None
        elif self.pivot is not None and left * right > 0:
            self.pivot = None
        mean = (left + right) / 2.
        if max(abs(left), abs(right)) <= p.forward_deadband_pct and abs(mean) <= p.forward_deadband_pct:
            return 0., 0., icr_x, 'still'
        diff = right - left
        scale = self.speed_run_scale * (1. + self.speed_fluctuation)
        v = math.copysign(forward_curve(mean, p), mean) * scale
        v *= max(0., 1. - p.arc_speed_loss_per_pct_diff * abs(diff))
        m = abs(mean)
        gain = 1. if m >= 90. else 1. + (p.arc_yaw_low_duty_scale - 1.) * (
            (90. - m) / max(1e-9, 90. - p.forward_deadband_pct))
        w = math.radians(p.arc_yaw_deg_s_per_pct_diff * diff * max(0., gain))
        return v, w, icr_x, 'drive' if abs(v) > 1e-6 or abs(w) > 1e-6 else 'still'

    def advance_wander(self, distance_m):
        """Straight-line curvature bias: OU process in travelled distance."""
        if distance_m <= 0:
            return
        p = self.params
        b = math.exp(-distance_m / p.wander_corr_length_m)
        self.wander_deg_per_m = (p.wander_mean_deg_per_m
                                 + b * (self.wander_deg_per_m - p.wander_mean_deg_per_m)
                                 + math.sqrt(1 - b * b) * p.wander_sd_deg_per_m * self.normal())

    def step(self, dt):
        if not 0 < dt <= .05:
            raise ValueError('plant_step_out_of_range')
        p = self.params
        self.time_s += dt
        left, right = self.mcu.step(self.time_s, dt)
        self.mean_duty = (left + right) / 2.
        # Within-run speed fluctuation (AR(1)); wander curvature OU in distance.
        a = math.exp(-dt / p.speed_fluctuation_tau_s)
        self.speed_fluctuation = a * self.speed_fluctuation + math.sqrt(1 - a * a) * p.speed_fluctuation_sd * self.normal()
        v_target, w_target, icr_x, regime = self._targets(left, right, dt)
        if self.forward_blocked and regime == 'drive':
            v_target, w_target = 0., 0.      # stuck (e.g. root below the scan plane): no response
        self.advance_wander(abs(self.v) * dt)
        # Speeding up / small fluctuation: first-order lag. A real slow-down
        # (stop, fade) holds speed for coast_onset_s, then is limited by a
        # per-event coast deceleration; both are fitted to the MEASURED clean
        # fade-to-zero stops. This is not a brake model.
        slowing = abs(v_target) < abs(self.v) - .03 or v_target * self.v < 0
        if slowing and not self.decelerating:
            self.coast_decel = float(p.coast_decel_median_m_s2 * math.exp(p.coast_decel_log_sd * self.normal()))
            self.coast_since = self.time_s
        self.decelerating = slowing or (self.decelerating and abs(v_target) < abs(self.v) - 1e-6)
        previous_v = self.v
        if self.forward_blocked and regime == 'drive':
            self.v = 0.                      # SIM disturbance: abrupt stop against the obstacle
        elif self.decelerating:
            if self.time_s - self.coast_since >= p.coast_onset_s:
                change = min(abs(self.v - v_target) * (1 - math.exp(-dt / p.decel_tau_s)), self.coast_decel * dt)
                self.v -= math.copysign(change, self.v - v_target)
        else:
            self.v += (v_target - self.v) * (1 - math.exp(-dt / p.accel_tau_s))
        if v_target == 0. and abs(self.v) < .005:
            self.v = 0.
        self.accel = (self.v - previous_v) / dt
        wander = math.radians(self.wander_deg_per_m) * self.v if regime == 'drive' else 0.
        self.w += (w_target + wander - self.w) * (1 - math.exp(-dt / p.yaw_tau_s))
        if w_target == 0. and wander == 0. and abs(self.w) < math.radians(.05):
            self.w = 0.
        c = math.exp(-dt / .5)
        slip_sd = p.lateral_slip_sd_m_s * min(1., abs(self.v) / p.speed_at_100_m_s)
        self.lateral = c * self.lateral + math.sqrt(1 - c * c) * slip_sd * self.normal()
        w = self.w + self.external_yaw_rate
        lateral = self.lateral - w * icr_x
        mid = self.yaw + .5 * w * dt
        self.x += (self.v * math.cos(mid) - lateral * math.sin(mid)) * dt
        self.y += (self.v * math.sin(mid) + lateral * math.cos(mid)) * dt
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
        self.distance_m += math.hypot(self.v, lateral) * dt
        self.regime = regime
        if regime == 'pivot':
            self.vibration_rms = self.pivot.vibration_rms
        elif regime == 'drive' and abs(self.v) > .02:
            self.vibration_rms = p.drive_vibration(self.mean_duty)
        else:
            self.vibration_rms = self.still_vibration

    # --- sensors / evaluation ----------------------------------------
    def imu_sample(self):
        """Raw-IMU-like sample: gyro z with scale/bias/noise/vibration, accel."""
        p = self.params
        vib = self.vibration_rms
        noise = math.radians(math.hypot(p.gyro_still_noise_sd_deg_s, p.gyro_vibration_deg_s_per_m_s2 * vib))
        gyro = self.gyro_scale * (self.w + self.external_yaw_rate) + self.gyro_bias + noise * self._sensor_normal()
        axis = vib / math.sqrt(3.)
        acc = (self.accel + axis * self._sensor_normal(), self.v * self.w + axis * self._sensor_normal(),
               9.81 + axis * self._sensor_normal())
        return dict(stamp_s=self.time_s, gyro_z_rad_s=gyro, accel_m_s2=acc)

    def lidar_pose(self):
        dx = self.params.lidar_x_in_base_m
        return (self.x + dx * math.cos(self.yaw), self.y + dx * math.sin(self.yaw), self.yaw)

    def truth(self):
        return dict(time_s=self.time_s, x=self.x, y=self.y, yaw=self.yaw, v=self.v, w=self.w,
                    distance_m=self.distance_m, regime=self.regime,
                    applied_duty_pct=tuple(self.mcu.applied))

    def status(self):
        """MCU-like status (reported duty only; no speed, no encoder)."""
        return dict(requested=tuple(int(v) for v in self.mcu.target),
                    reported_duty_units=self.mcu.reported_units(),
                    watchdog_tripped=self.mcu.watchdog_tripped)


def _normal_from_uniform(u):
    """Inverse standard-normal CDF (Acklam) so one uniform gives one normal."""
    u = min(max(float(u), 1e-12), 1 - 1e-12)
    a = (-39.69683028665376, 220.9460984245205, -275.9285104469687, 138.3577518672690, -30.66479806614716,
         2.506628277459239)
    b = (-54.47609879822406, 161.5858368580409, -155.6989798598866, 66.80131188771972, -13.28068155288572)
    c = (-7.784894002430293e-03, -0.3223964580411365, -2.400758277161838, -2.549732539343734, 4.374664141464968,
         2.938163982698783)
    d = (7.784695709041462e-03, 0.3224671290700398, 2.445134137142996, 3.754408661907416)
    low = .02425
    if u < low:
        q = math.sqrt(-2 * math.log(u))
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / \
               ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1)
    if u > 1 - low:
        q = math.sqrt(-2 * math.log(1 - u))
        return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / \
            ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1)
    q = u - .5
    r = q * q
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q / \
           (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1)


class FieldActuatorProfile:
    """ActuatorResponse-compatible view of the field response for SimMotor.

    Gazebo keeps its own inertia/friction, so this returns the NON-lagged
    steady target (incl. stochastic wander and pivot breakaway/stall) as
    equivalent wheel speeds, plus the MEASURED MCU ramp. Not run in Gazebo
    in this change; unit-tested only through the SimMotor AST harness.

    Inert range: with the SIM_ASSUMED 50 % forward deadband and 80 % pivot
    minimum, commands below those duties produce NO motion (e.g. the dashboard
    default manual speed 25 % and the Gazebo AutonomousControl defaults
    0.4/0.3 m/s = 40/30 % of max_wheel). See inert_below_pct in status().
    """

    def __init__(self, config=None):
        config = dict(config or {})
        seed = config.pop('seed', 0)
        surface = config.pop('surface', 'normal')
        self.track_m = float(config.pop('track_m', .56))
        overrides = {k: (tuple(v) if isinstance(v, list) else v) for k, v in config.items()}
        self.params = replace(FieldResponseParams(), **overrides)
        self.plant = FieldPlant(self.params, surface=surface, seed=seed)
        self.mcu = McuRamp(self.params)
        self.last_stamp = None
        self.ramp_clock_s = 0.

    def ramp_step(self, applied, target, dt, maximum):
        """Advance the MEASURED MCU ramp; inputs/outputs in SimMotor m/s units."""
        if maximum <= 0 or dt <= 0:
            return list(applied)
        self.mcu.applied = [max(-100., min(100., v / maximum * 100)) for v in applied]
        self.mcu.target = [float(max(-100, min(100, round(v / maximum * 100)))) for v in target]
        self.mcu.last_command_s = None  # SimMotor/bridge own the watchdog here
        self.ramp_clock_s += dt
        self.mcu.step(self.ramp_clock_s, dt)
        return [v / 100 * maximum for v in self.mcu.applied]

    def wheel_targets(self, left, right, maximum, stamp):
        if not all(math.isfinite(v) for v in (left, right, maximum, stamp)) or maximum <= 0:
            raise ValueError('invalid_actuator_input')
        dl = max(-100., min(100., left / maximum * 100))
        dr = max(-100., min(100., right / maximum * 100))
        dt = 0. if self.last_stamp is None else min(.05, max(0., stamp - self.last_stamp))
        self.last_stamp = stamp
        if dl == 0. and dr == 0.:
            self.plant.pivot = None
            return 0., 0.
        v, w, _, _ = self.plant._targets(dl, dr, dt)
        self.plant.advance_wander(abs(v) * dt)  # steady speed; no lag here
        if abs(v) > 1e-6:
            w += math.radians(self.plant.wander_deg_per_m) * v
        self.plant.regime = 'drive' if abs(v) > 1e-6 else 'still'
        return v - w * self.track_m / 2, v + w * self.track_m / 2

    def status(self):
        """Compact status: profile, provenance digest/counts and pivot draws (full table: provenance())."""
        table = provenance(self.params)
        return dict(enabled=True, profile=PROFILE_NAME, provenance_sha256=provenance_digest(self.params),
                    provenance_counts=table['counts'], surface=self.plant.surface,
                    pivot_events=self.plant.pivot_count, pivot_stalls=self.plant.pivot_stalls,
                    inert_below_pct=dict(forward=self.params.forward_deadband_pct,
                                         pivot_opposite=self.params.pivot_min_duty_pct),
                    calibrated_speed_curve=False, braking_calibrated=False,
                    measured_points_only_at_duty_pct=[90, 100])
