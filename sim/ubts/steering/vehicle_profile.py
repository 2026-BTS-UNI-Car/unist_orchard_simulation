"""Vehicle profile loader for the steering SIM (schema ubts.vehicle_profile.v2, design 3.8).

One profile is one YAML file under sim/ubts/steering/profiles/. Files are versioned and
not edited after use; a new measurement gives a new file. Every result carries
profile_id and profile_sha256 (SHA-256 of the file bytes).

Validation (design 3.8.1, task rules):
  * closed provenance tags: PLAIN_TAGS, MEASURED_<YYYYMMDD> and
    MEASURED_<YYYYMMDD>_OLD_DRIVETRAIN (the date must be a real calendar date)
  * each parameter record has value, range, unit, provenance, identify, stale_after_mod;
    a DERIVED record has unit, provenance, identify and derived_from, and may also write
    range and stale_after_mod (checked against the computation). A DERIVED record must NOT
    write a value: it is computed on load (design 3.8.1 "DERIVED 값은 파일에 적지 않는다");
    see the computed values with provenance_table() or value(name)
  * every name of DERIVATIONS must be tagged DERIVED (a computed quantity cannot be given
    another tag and a free value)
  * source is required for ESTIMATED, MEASURED_* and DEFINITION
  * DERIVED values are computed on load (DERIVATIONS); derived_from must name exactly the
    inputs of the computation; a written stale_after_mod must equal OR(inputs)
  * values must have the shape of the parameter (SHAPES: number, integer, bool, string,
    per-run number-or-distribution, keyed mapping, list form) and lie in range
  * semantic checks (_semantic_checks) refuse inconsistent sets, e.g. a standstill steering
    rate above the moving rate or a standstill minimum PWM below the moving one (design 3.3 L1)
  * unknown or duplicate keys, unknown parameters and missing parameters are rejected

Consistency warnings (warnings(), also in result_label()): combinations that load but give
a known artefact, e.g. a servo minimum PWM that cannot move the steering motor or pot end
stops outside the calibration window. They are reported, not refused, because such
combinations are also legitimate stress scenarios.

Range forms:
  [lo, hi]                 numeric interval; for a list/table value it applies to every leaf;
                           for a distribution value it is the support of the per-run draw
  {choices: [a, b, ...]}   allowed values (categorical, booleans or discrete numbers); for a
                           list value every element must be a choice
  {key: <range>, ...}      per key of a mapping value (same keys as the value)
  [<range>, <range>, ...]  per element of a list value (same length)
  null                     only with value null (SPEC_PENDING, "not decided, not used yet")

Distribution values (drawn once per run by sample()):
  {dist: uniform, low: a, high: b}  lo <= a <= b <= hi
  {dist: normal, mean: m, sd: s}    lo <= m <= hi, s >= 0; draws are truncated to [lo, hi]
A per-run parameter (shape PER_RUN) may also hold a plain number: a fixed per-run value
(scenario conditions such as a fixed zero offset). per_run_value(seed, name) gives the
value of a run in both cases; sample() draws only the distributions.

Schema v3 (ubts.vehicle_profile.v3, profile v0.2 and later): the v2 parameters plus the groups I (belt
potentiometer, end switches, steering supply) and J (external steering torque) of V3_PARAMETER_GROUPS. A v2 file
keeps loading with the v2 parameter set (its results stay reproducible); has(name) tells whether a view has a
parameter. v3 also refuses at load what v2 only warns about: pot end stops outside the calibration window, a
calibration half width below MARK_HALF_WIDTH_MIN counts, and end switches (not modelled, no MCU pin).

Schema v4 (ubts.vehicle_profile.v4, profile front_steer_v1_spec_20261116 and later): the v3 parameters plus group K
(K_GROUP: the steering firmware law of the SIM ESP emulator). steer_fw_mode v3_servo keeps the firmware V3 servo
(every earlier result); abs_hold_8 runs the pulse hold of the real board's firmware ABS-HOLD-8-REBOUND-LIMITS with its
constants (abs_hold_fw), the host ANGLE policy of the SIM (abs_hold_host) and the measured step times
(steer_step_time_measured_s; docs/steering/ABS_HOLD8_SIM_KO.md). v2 and v3 files keep loading unchanged.

Pure Python + PyYAML (safe loader only). No numpy.
"""
from __future__ import annotations

import copy
import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import statistics

import yaml

from . import geometry

SCHEMA = 'ubts.vehicle_profile.v2'
SCHEMA_V3 = 'ubts.vehicle_profile.v3'
SCHEMA_V4 = 'ubts.vehicle_profile.v4'
SCHEMAS = (SCHEMA, SCHEMA_V3, SCHEMA_V4)
STEER_FW_MODES = ('v3_servo', 'abs_hold_8')
MARK_HALF_WIDTH_MIN = 50      # counts between the centre mark and each end mark (fw_eeprom.MARK_WINDOW_SPAN_MIN)
MARK_INSET_DEG = 1.0          # calibration marks at the equivalent mechanical limit - 1 deg (LOCKSTEP 11.11)
PROFILES_DIR = Path(__file__).resolve().parent / 'profiles'
DEFAULT_PROFILE_ID = 'front_steer_v0_1_placeholder'
DEFAULT_PROFILE_PATH = PROFILES_DIR / (DEFAULT_PROFILE_ID + '.yaml')
RESULT_LABEL_PLACEHOLDER = '자리값 기반 SIM 결과'
RESULT_LABEL_PARTIAL = '일부 실측 기반 SIM 결과'
RESULT_LABEL_MEASURED = '실측 프로필 기반 SIM 결과'

PLAIN_TAGS = ('SPEC_PENDING', 'SIM_ASSUMED', 'SIM_NUMERIC', 'PRODUCT_LABEL', 'USER_APPROX', 'GENERIC_SPEC',
              'DEFINITION', 'CONFIG', 'FIRMWARE', 'DERIVED', 'ESTIMATED')
MEASURED_PATTERN = re.compile(r'MEASURED_(\d{4})(\d{2})(\d{2})(_OLD_DRIVETRAIN)?\Z')
SOURCE_REQUIRED_TAGS = ('DEFINITION', 'ESTIMATED')  # and every MEASURED_* tag
TOP_LEVEL_KEYS = ('schema', 'profile_id', 'description', 'parameters')
REQUIRED_TOP_LEVEL_KEYS = ('schema', 'profile_id', 'parameters')
RECORD_KEYS = ('value', 'range', 'unit', 'provenance', 'identify', 'stale_after_mod', 'source', 'derived_from',
               'where', 'note')
REQUIRED_FIELDS = ('value', 'range', 'unit', 'provenance', 'identify', 'stale_after_mod')
REQUIRED_FIELDS_DERIVED = ('unit', 'provenance', 'identify', 'derived_from')
WHERE_VALUES = ('mcu', 'host', 'executor')
IDENTIFY_WORDS = ('none', 'part', 'design', 'photo', 'wiring', 'standstill_record', 'jetson_config',
                  'sim_compare', 'record_analysis')
IDENTIFY_TEST_PATTERN = re.compile(r'(B(10|[0-9])|F[1-6]|Q(2[0-9]|1[0-9]|[1-9]))\Z')
DISTRIBUTIONS = {'uniform': ('low', 'high'), 'normal': ('mean', 'sd')}
CURVE_NAMES = ('P_fit1003', 'field1010')
_PROFILE_ID = re.compile(r'[a-z0-9_]+\Z')
# Safe loader: libyaml CSafeLoader when present (same resolver and constructors as yaml.safe_load).
SAFE_LOADER = getattr(yaml, 'CSafeLoader', yaml.SafeLoader)

# Parameter names of schema v2, in file order, grouped like the design tables (3.8.2 A-H).
PARAMETER_GROUPS = (
    ('A_body_geometry', (
        'wheelbase_m', 'track_front_m', 'track_rear_m', 'kingpin_width_m', 'wheel_radius_front_m',
        'wheel_radius_rear_m', 'wheel_width_m', 'rear_axle_x_in_base_m', 'footprint_length_m', 'footprint_width_m',
        'front_bumper_x_in_base_m', 'ground_clearance_m', 'steer_type', 'caster_deg', 'kingpin_inclination_deg',
        'scrub_radius_m', 'steer_hardware_margin_m', 'steer_sweep_m')),
    ('B_mass_and_gsim', (
        'mass_kg', 'cog_x_in_base_m', 'cog_z_m', 'wheel_mass_kg', 'wheel_inertia_kgm2', 'steer_link_mass_kg',
        'gz_body_box_m', 'gz_physics_hz', 'gz_drive_mode', 'gz_steer_joint_cmd_max_rad_s')),
    ('C_steering_actuator', (
        'steer_ackermann_ratio', 'steer_mech_limit_deg', 'steer_soft_margin_deg', 'steer_soft_limit_eq_left_deg',
        'steer_soft_limit_eq_right_deg', 'steer_rate_max_deg_s', 'steer_rate_standstill_deg_s', 'steer_motor_tau_s',
        'steer_min_pwm_pct', 'steer_min_pwm_standstill_pct', 'steer_backlash_deg', 'steer_zero_offset_deg',
        'steer_lr_asymmetry', 'steer_backlash_wander', 'steer_self_locking', 'steer_release_restoring',
        'steer_link_table', 'steer_gear_ratio', 'steer_motor_stall_torque_nm', 'steer_load_moving_nm',
        'steer_load_standstill_nm', 'steer_motor_stall_current_a', 'steer_actuator_level',
        'imu_vibration_steer_m_s2')),
    ('D_servo_and_link', (
        'steer_loop_location', 'sim_mcu_tick_hz', 'servo_hz', 'servo_kp_pct_per_deg', 'servo_deadband_deg',
        'servo_min_pwm_pct', 'servo_max_pwm_pct', 'servo_dwell_brake_s', 'servo_reversal_pause_ms', 'c_ttl_max_ms',
        'c_resend_hz', 'ttl_expiry_drive_mode', 'watchdog_ms', 'steer_watchdog_mode', 'steer_hold_after_loss_s',
        'steer_hold_after_stop_s', 'steer_hold_after_disarm_s', 'stop_policy_steer', 'stop_confirm_periods',
        'stop_resend_interval_ms', 'stop_resend_max', 'status_hz', 'status_latency_s', 'serial_delay_ms', 'baud')),
    ('D2_fault_thresholds', (
        'pot_valid_raw', 'cal_end_raw_window', 'pot_jump_counts', 'pot_noise_counts', 'polarity_fault',
        'stall_detect', 'track_fault', 'driver_stuck_fault', 'thermal_proxy', 'crc_burst_n', 'host_status_gap_ms',
        'host_seq_gap', 'host_track_mismatch', 'steering_ineffective', 'steering_authority_insufficient')),
    ('E_potentiometer', (
        'pot_location', 'pot_range_deg', 'pot_to_steer_ratio', 'pot_center_raw', 'adc_bits', 'adc_ratiometric',
        'pot_supply_sag_per_pwm', 'pot_lsb_deg', 'pot_noise_sd_counts', 'pot_linearity_pct', 'pot_gain_error',
        'pot_cal_residual_deg', 'pot_drift_deg_per_h', 'pot_filter', 'pot_fault_injections')),
    ('F_drive', (
        'drive_wheels', 'drive_lr_mode', 'forward_curve', 'drive_curve_P_fit1003', 'drive_curve_field1010_measured',
        'drive_curve_field1010_deadband_pct', 'reverse_curve', 'mcu_ramp', 'stop_release', 'stop_brake',
        'stop_coast', 'steering_efficiency', 'scrub_speed_loss_per_deg', 'understeer_gradient', 'curvature_wander',
        'lateral_slip_sd_m_s', 'speed_run_scale', 'speed_fluctuation_sd', 'speed_fluctuation_tau_s',
        'drive_accel_tau_s', 'drive_decel_tau_s', 'drive_omega0_rad_s', 'drive_deadband_frac', 'drive_tau_stall_nm',
        'battery_voltage_scale', 'v_target_m_s')),
    ('G_tyre_and_ground', ('wheel_mu_lat', 'wheel_mu_long', 'ground_mu', 'terrain')),
    ('H_sensors', (
        'lidar_x_in_base_m', 'lidar_y_in_base_m', 'lidar_z_m', 'lidar_yaw_offset_deg', 'lidar_roll_pitch_deg',
        'lidar_noise_sd_m', 'lidar_latency_s', 'lidar_sweep_s', 'lidar_beams', 'lidar_hz', 'imu_pose_in_base',
        'imu_mount_rpy_deg', 'imu_rate_hz', 'imu_gyro_scale', 'imu_gyro_bias_sd_deg_s',
        'imu_gyro_still_noise_sd_deg_s', 'imu_gyro_vibration_deg_s_per_m_s2', 'imu_vibration_drive_at_90_m_s2',
        'imu_vibration_drive_at_100_m_s2', 'imu_vibration_drive_below_90_m_s2', 'imu_vibration_still_m_s2',
        'camera_pose_in_base', 'gnss_antenna_pose_in_base', 'self_reflection_mask')),
)
PARAMETER_NAMES = tuple(name for _, names in PARAMETER_GROUPS for name in names)
# Schema v3 groups added after the v2 groups (ST-10 belt potentiometer, ST-11 external torque, ST-17 spec fields).
V3_PARAMETER_GROUPS = (
    ('I_belt_and_supply', (
        'belt_type', 'belt_pulley_teeth_pot', 'belt_tooth_step_deg', 'belt_pot_play_deg', 'belt_creep_sd_deg_per_100deg',
        'belt_skip_per_10000deg', 'pot_travel_deg', 'pot_cal_half_width_counts', 'steer_limit_switch',
        'steer_supply_shared_with_drive')),
    ('J_steer_disturbance', (
        'steer_backdrive_coast_gain', 'steer_self_lock_breakaway_nm', 'steer_disturbance_design_nm',
        'steer_terrain_impacts', 'steer_terrain_push')),
)
PARAMETER_GROUPS_V3 = PARAMETER_GROUPS + V3_PARAMETER_GROUPS
PARAMETER_NAMES_V3 = tuple(name for _, names in PARAMETER_GROUPS_V3 for name in names)
# Schema v4 group K (2026-11-16): the steering firmware law of the SIM ESP emulator.
V4_PARAMETER_GROUPS = (
    ('K_steer_firmware', ('steer_fw_mode', 'abs_hold_fw', 'abs_hold_host', 'steer_step_time_measured_s')),
)
PARAMETER_GROUPS_V4 = PARAMETER_GROUPS_V3 + V4_PARAMETER_GROUPS
PARAMETER_NAMES_V4 = tuple(name for _, names in PARAMETER_GROUPS_V4 for name in names)
NAMES_OF_SCHEMA = {SCHEMA: PARAMETER_NAMES, SCHEMA_V3: PARAMETER_NAMES_V3, SCHEMA_V4: PARAMETER_NAMES_V4}
GROUP_OF = {name: group for group, names in PARAMETER_GROUPS_V4 for name in names}
# Keys of the abs_hold_fw mapping: the constants of BTS7960_Serial_Control.ino (ABS-HOLD-8-REBOUND-LIMITS), counts of
# the board's 12-bit ADC and milliseconds. The SIM emulator (native/abs_hold8.cc) uses every key.
ABS_HOLD_FW_KEYS = (
    'cal_right', 'cal_center', 'cal_left', 'command_max_deg', 'pulse_pwm', 'pulse_long_ms', 'pulse_short_ms',
    'pulse_long_above_counts', 'settle_ms', 'rebound_window_ms', 'target_tol_counts', 'restart_tol_counts',
    'rebound_max_counts', 'progress_min_counts', 'no_progress_max', 'drive_budget_ms', 'guard_counts', 'sample_ms',
    'median_of', 'pair_tol_counts', 'stale_ms', 'limit_margin_counts')
ABS_HOLD_FW_FLOAT_KEYS = ('command_max_deg',)
ABS_HOLD_HOST_KEYS = ('step_deg', 'deadband_deg', 'min_interval_ms', 'poll_ms')

# Names the steered plant (F-SIM SteeredPlant, P1/P2) reads; geometry.PROFILE_NAMES come on top.
PLANT_PROFILE_NAMES = (
    'lidar_x_in_base_m', 'lidar_y_in_base_m', 'lidar_z_m', 'lidar_yaw_offset_deg', 'lidar_roll_pitch_deg',
    'lidar_noise_sd_m', 'lidar_latency_s', 'lidar_beams', 'lidar_hz', 'self_reflection_mask', 'imu_pose_in_base',
    'imu_mount_rpy_deg', 'imu_rate_hz', 'imu_gyro_scale', 'imu_gyro_bias_sd_deg_s', 'imu_gyro_still_noise_sd_deg_s',
    'imu_gyro_vibration_deg_s_per_m_s2', 'imu_vibration_drive_at_90_m_s2', 'imu_vibration_drive_at_100_m_s2',
    'imu_vibration_drive_below_90_m_s2', 'imu_vibration_still_m_s2', 'imu_vibration_steer_m_s2', 'drive_wheels',
    'drive_lr_mode', 'forward_curve', 'drive_curve_P_fit1003', 'drive_curve_field1010_measured',
    'drive_curve_field1010_deadband_pct', 'reverse_curve', 'mcu_ramp', 'stop_release', 'stop_brake', 'stop_coast',
    'ttl_expiry_drive_mode', 'steering_efficiency', 'scrub_speed_loss_per_deg', 'understeer_gradient',
    'curvature_wander', 'lateral_slip_sd_m_s', 'speed_run_scale', 'speed_fluctuation_sd', 'speed_fluctuation_tau_s',
    'drive_accel_tau_s', 'drive_decel_tau_s', 'battery_voltage_scale', 'pot_supply_sag_per_pwm', 'v_target_m_s')

# Ideal values for geometry checks only (A1, A12). Default runs use sample().
IDEAL_DRAWS = {'steering_efficiency': 1.0, 'scrub_speed_loss_per_deg': 0.0, 'understeer_gradient': 0.0,
               'steer_zero_offset_deg': 0.0, 'pot_cal_residual_deg': 0.0}

# --- value shapes (every non-DERIVED value and every override must have its parameter's shape) ------------------
NUMBER = 'number'                  # int or float (not bool)
INTEGER = 'integer'                # int (not bool)
BOOL = 'bool'
STRING = 'string'
PER_RUN = 'per_run'                # a distribution drawn once per run, or a number (a fixed per-run value)
NUMBER_OR_NULL = 'number_or_null'  # SPEC_PENDING "not decided" (null) or a number
STRINGS = 'strings'                # non-empty list of strings
ROWS2 = 'rows2'                    # non-empty list of [number, number] rows


def _keys(shape, *names):
    return {name: shape for name in names}


_STOP_SHAPE = _keys(NUMBER, 'onset_s', 'decel_median_m_s2', 'decel_log_sd')
_POSE_SHAPE = _keys(NUMBER, 'x', 'y', 'z')
# Shapes of the parameters that are not a plain NUMBER (all other non-DERIVED parameters are NUMBER).
SHAPES = {
    'steer_type': STRING, 'gz_body_box_m': ('numbers', 3), 'gz_physics_hz': INTEGER, 'gz_drive_mode': STRING,
    'steer_mech_limit_deg': _keys(NUMBER, *geometry.WHEEL_LIMIT_KEYS), 'steer_zero_offset_deg': PER_RUN,
    'steer_backlash_wander': _keys(NUMBER, 'sd_deg', 'corr_length_m'), 'steer_self_locking': BOOL,
    'steer_release_restoring': _keys(NUMBER, 'scale', 'centering_per_s_per_m_s', 'noise_sd_deg',
                                     'noise_corr_length_m'),
    'steer_link_table': ROWS2, 'steer_gear_ratio': NUMBER_OR_NULL, 'steer_motor_stall_current_a': NUMBER_OR_NULL,
    'steer_actuator_level': STRING, 'steer_loop_location': STRING, 'sim_mcu_tick_hz': INTEGER, 'servo_hz': INTEGER,
    'ttl_expiry_drive_mode': STRING, 'steer_watchdog_mode': STRING, 'stop_policy_steer': STRING,
    'stop_confirm_periods': INTEGER, 'stop_resend_max': INTEGER, 'status_hz': INTEGER, 'baud': INTEGER,
    'pot_valid_raw': _keys(INTEGER, 'min_raw', 'max_raw'), 'cal_end_raw_window': _keys(INTEGER, 'min_raw', 'max_raw'),
    'pot_jump_counts': {'counts': INTEGER, 'window_ms': NUMBER}, 'pot_noise_counts': _keys(NUMBER, 'sd_counts',
                                                                                             'window_ms'),
    'polarity_fault': _keys(NUMBER, 'away_deg', 'duration_s'),
    'stall_detect': _keys(NUMBER, 'pwm_pct', 'duration_s', 'min_motion_deg'),
    'track_fault': _keys(NUMBER, 'error_deg', 'duration_s'), 'driver_stuck_fault': _keys(NUMBER, 'motion_deg',
                                                                                         'duration_s'),
    'thermal_proxy': _keys(NUMBER, 'pwm_pct', 'window_s', 'accumulated_s'), 'crc_burst_n': INTEGER,
    'host_seq_gap': {'missing': INTEGER, 'window_s': NUMBER}, 'host_track_mismatch': _keys(NUMBER, 'error_deg',
                                                                                           'duration_s'),
    'steering_ineffective': _keys(NUMBER, 'min_speed_m_s', 'min_steer_deg', 'rate_error_deg_s', 'rate_error_frac',
                                  'duration_s'),
    'steering_authority_insufficient': _keys(NUMBER, 'limit_fraction', 'duration_s'),
    'pot_location': STRING, 'pot_center_raw': INTEGER, 'adc_bits': INTEGER, 'adc_ratiometric': BOOL,
    'pot_cal_residual_deg': PER_RUN, 'pot_filter': _keys(INTEGER, 'sample_hz', 'median_of'),
    'pot_fault_injections': STRINGS, 'drive_wheels': STRING, 'drive_lr_mode': STRING,
    'forward_curve': _keys(STRING, 'main', 'bracket'),
    'drive_curve_P_fit1003': {'speed_at_90_m_s': NUMBER, 'speed_at_100_m_s': NUMBER, 'deadband_pct': NUMBER,
                              'fit_points_pct_m_s': ROWS2},
    'drive_curve_field1010_measured': _keys(NUMBER, 'speed_at_90_m_s', 'speed_at_100_m_s'),
    'mcu_ramp': {'pwm_per_s': INTEGER, 'reversal_dwell_ms': NUMBER},
    'stop_release': _STOP_SHAPE, 'stop_brake': _STOP_SHAPE, 'stop_coast': _STOP_SHAPE,
    'steering_efficiency': PER_RUN, 'scrub_speed_loss_per_deg': PER_RUN, 'understeer_gradient': PER_RUN,
    'curvature_wander': _keys(NUMBER, 'mean_deg_per_m', 'sd_deg_per_m', 'corr_length_m'),
    'speed_run_scale': _keys(NUMBER, 'mode', 'sd_low', 'sd_high'), 'terrain': STRING,
    'lidar_roll_pitch_deg': _keys(NUMBER, 'roll', 'pitch'), 'lidar_beams': INTEGER, 'lidar_hz': INTEGER,
    'imu_pose_in_base': _POSE_SHAPE, 'imu_mount_rpy_deg': {'roll': NUMBER, 'pitch': NUMBER, 'yaw': NUMBER,
                                                           'yaw_sign': INTEGER},
    'imu_rate_hz': INTEGER, 'imu_gyro_scale': _keys(NUMBER, 'mean', 'sd'),
    'imu_vibration_still_m_s2': _keys(NUMBER, 'low', 'high'), 'camera_pose_in_base': _POSE_SHAPE,
    'gnss_antenna_pose_in_base': _POSE_SHAPE, 'self_reflection_mask': INTEGER,
    # schema v3
    'belt_type': STRING, 'belt_pulley_teeth_pot': INTEGER, 'steer_limit_switch': STRING,
    'steer_supply_shared_with_drive': BOOL,
    'steer_terrain_impacts': _keys(NUMBER, 'rate_per_m', 'torque_sd_nm', 'duration_s'),
    'steer_terrain_push': _keys(NUMBER, 'sd_nm', 'corr_length_m'),
    # schema v4
    'steer_fw_mode': STRING,
    'abs_hold_fw': {key: (NUMBER if key in ABS_HOLD_FW_FLOAT_KEYS else INTEGER) for key in ABS_HOLD_FW_KEYS},
    'abs_hold_host': {'step_deg': NUMBER, 'deadband_deg': NUMBER, 'min_interval_ms': INTEGER, 'poll_ms': INTEGER},
    'steer_step_time_measured_s': _keys(NUMBER, 'outward_20', 'return_20'),
}


class ProfileError(ValueError):
    """A profile file breaks a rule. .code is a short machine-readable reason."""

    def __init__(self, code, name='', detail=''):
        self.code = code
        self.name = name
        self.detail = detail
        text = code if not name else '%s: %s' % (code, name)
        super().__init__(text + (': ' + detail if detail else ''))


# --- helpers -------------------------------------------------------------------------------------------------
def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _finite_number(value):
    return _is_number(value) and math.isfinite(value)


def _same(a, b):
    if isinstance(a, bool) or isinstance(b, bool):
        return isinstance(a, bool) and isinstance(b, bool) and a == b
    if _is_number(a) and _is_number(b):
        return a == b
    if isinstance(a, str) and isinstance(b, str):
        return a == b
    return False


_EMPTY = object()  # stands for an empty list or mapping; never a valid leaf


def _leaves(value):
    if isinstance(value, (dict, list)) and not value:
        return [_EMPTY]
    if isinstance(value, dict):
        out = []
        for item in value.values():
            out.extend(_leaves(item))
        return out
    if isinstance(value, list):
        out = []
        for item in value:
            out.extend(_leaves(item))
        return out
    return [value]


def _is_distribution(value):
    return isinstance(value, dict) and 'dist' in value


def _shape_text(shape):
    if isinstance(shape, dict):
        return 'a mapping with keys ' + ', '.join(sorted(shape))
    if isinstance(shape, tuple):
        return 'a list of %d numbers' % shape[1]
    return {NUMBER: 'a number', INTEGER: 'an integer', BOOL: 'true or false', STRING: 'a string',
            PER_RUN: 'a number or a distribution', NUMBER_OR_NULL: 'a number or null',
            STRINGS: 'a non-empty list of strings', ROWS2: 'a non-empty list of [number, number] rows'}[shape]


def _check_shape(name, value, shape):
    """Raise ProfileError unless value has the parameter shape (SHAPES; default NUMBER).

    A float where an integer is expected gives code not_integer; any other mismatch bad_shape.
    """
    if isinstance(shape, dict):
        if not isinstance(value, dict) or _is_distribution(value) or set(value) != set(shape):
            raise ProfileError('bad_shape', name, 'expected %s, got %r' % (_shape_text(shape), value))
        for key, item in value.items():
            _check_shape('%s.%s' % (name, key), item, shape[key])
        return
    if shape == INTEGER and _is_number(value) and not isinstance(value, int):
        raise ProfileError('not_integer', name, repr(value))
    if shape == NUMBER:
        ok = _is_number(value)
    elif shape == INTEGER:
        ok = isinstance(value, int) and not isinstance(value, bool)
    elif shape == BOOL:
        ok = isinstance(value, bool)
    elif shape == STRING:
        ok = isinstance(value, str)
    elif shape == PER_RUN:
        ok = _is_number(value) or _is_distribution(value)
    elif shape == NUMBER_OR_NULL:
        ok = value is None or _is_number(value)
    elif shape == STRINGS:
        ok = isinstance(value, list) and bool(value) and all(isinstance(item, str) for item in value)
    elif shape == ROWS2:
        ok = isinstance(value, list) and bool(value) and all(
            isinstance(row, list) and len(row) == 2 and all(_is_number(x) for x in row) for row in value)
    elif isinstance(shape, tuple) and shape[0] == 'numbers':
        ok = isinstance(value, list) and len(value) == shape[1] and all(_is_number(x) for x in value)
    else:  # pragma: no cover - a broken SHAPES entry
        raise AssertionError('unknown shape %r for %s' % (shape, name))
    if not ok:
        raise ProfileError('bad_shape', name, 'expected %s, got %r' % (_shape_text(shape), value))


def shape_of(name):
    """The value shape of a non-DERIVED parameter (SHAPES entry or NUMBER)."""
    return SHAPES.get(name, NUMBER)


def parse_provenance(tag):
    """(kind, date) for a valid tag; kind is the tag itself or 'MEASURED' / 'MEASURED_OLD_DRIVETRAIN'."""
    if not isinstance(tag, str):
        raise ProfileError('provenance_unknown', '', repr(tag))
    if tag in PLAIN_TAGS:
        return tag, None
    match = MEASURED_PATTERN.match(tag)
    if not match:
        raise ProfileError('provenance_unknown', '', tag)
    year, month, day, old = match.groups()
    try:
        date = datetime.date(int(year), int(month), int(day))
    except ValueError:
        raise ProfileError('provenance_bad_date', '', tag) from None
    if not 2000 <= date.year <= 2099:
        raise ProfileError('provenance_bad_date', '', tag)
    return ('MEASURED_OLD_DRIVETRAIN' if old else 'MEASURED'), date


def _source_required(tag):
    return tag in SOURCE_REQUIRED_TAGS or tag.startswith('MEASURED_')


def _check_identify(name, identify):
    tokens = [identify] if isinstance(identify, str) else identify
    if not isinstance(tokens, list) or not tokens or not all(isinstance(t, str) and t for t in tokens):
        raise ProfileError('bad_field', name, 'identify must be a string or a non-empty list of strings')
    for token in tokens:
        if token not in IDENTIFY_WORDS and not IDENTIFY_TEST_PATTERN.match(token):
            raise ProfileError('identify_unknown', name, token)
    if 'none' in tokens and len(tokens) > 1:
        raise ProfileError('identify_unknown', name, "'none' must stand alone")
    if len(set(tokens)) != len(tokens):
        raise ProfileError('identify_unknown', name, 'duplicate token')
    return list(tokens)


def _check_range(name, value, rng, check_bounds=True):
    """Raise ProfileError unless value fits range (see the module docstring)."""
    if _is_distribution(value):
        _check_distribution(name, value, rng, check_bounds)
        return
    if isinstance(value, (list, dict)) and not value:
        raise ProfileError('bad_value', name, 'empty list or mapping')
    if isinstance(rng, dict):
        if set(rng) == {'choices'}:
            choices = rng['choices']
            if (not isinstance(choices, list) or not choices
                    or any(isinstance(c, (list, dict)) or c is None for c in choices)):
                raise ProfileError('bad_range', name, 'choices must be a non-empty list of scalars')
            for leaf in _leaves(value):
                if not any(_same(leaf, choice) for choice in choices):
                    raise ProfileError('not_a_choice', name, '%r not in %r' % (leaf, choices))
            return
        if not isinstance(value, dict):
            raise ProfileError('bad_value', name, 'range is a mapping but the value is not')
        if set(rng) != set(value):
            raise ProfileError('range_keys_mismatch', name, 'value keys %s, range keys %s'
                               % (sorted(value), sorted(rng)))
        for key in value:
            _check_range('%s.%s' % (name, key), value[key], rng[key], check_bounds)
        return
    if isinstance(rng, list) and len(rng) == 2 and all(_is_number(x) for x in rng):
        lo, hi = rng
        if not (math.isfinite(lo) and math.isfinite(hi)) or lo > hi:
            raise ProfileError('bad_range', name, repr(rng))
        for leaf in _leaves(value):
            if not _finite_number(leaf):
                raise ProfileError('bad_value', name, 'numeric range needs finite numbers, got %r' % (leaf,))
            if check_bounds and not lo <= leaf <= hi:
                raise ProfileError('out_of_range', name, '%r not in [%r, %r]' % (leaf, lo, hi))
        return
    if isinstance(rng, list) and rng and all(isinstance(r, (list, dict)) for r in rng):
        if not isinstance(value, list) or len(value) != len(rng):
            raise ProfileError('range_shape_mismatch', name, 'need a list of %d elements' % len(rng))
        for index, (item, item_range) in enumerate(zip(value, rng)):
            _check_range('%s[%d]' % (name, index), item, item_range, check_bounds)
        return
    raise ProfileError('bad_range', name, repr(rng))


def _check_distribution(name, value, rng, check_bounds):
    kind = value.get('dist')
    if kind not in DISTRIBUTIONS or set(value) != {'dist'} | set(DISTRIBUTIONS[kind]):
        raise ProfileError('bad_distribution', name, repr(value))
    if not (isinstance(rng, list) and len(rng) == 2 and all(_finite_number(x) for x in rng)) or rng[0] > rng[1]:
        raise ProfileError('bad_range', name, 'a distribution needs a numeric [lo, hi] range')
    numbers = [value[key] for key in DISTRIBUTIONS[kind]]
    if not all(_finite_number(x) for x in numbers):
        raise ProfileError('bad_distribution', name, 'parameters must be finite numbers')
    lo, hi = rng
    if kind == 'uniform':
        low, high = numbers
        if low > high:
            raise ProfileError('bad_distribution', name, 'low > high')
        if check_bounds and not lo <= low <= high <= hi:
            raise ProfileError('out_of_range', name, 'U(%r, %r) not inside [%r, %r]' % (low, high, lo, hi))
    else:
        mean, sd = numbers
        if sd < 0:
            raise ProfileError('bad_distribution', name, 'sd < 0')
        if check_bounds and not lo <= mean <= hi:
            raise ProfileError('out_of_range', name, 'mean %r not in [%r, %r]' % (mean, lo, hi))


def _check_duplicate_keys(data):
    node = yaml.compose(data, Loader=SAFE_LOADER)
    seen_nodes, stack = set(), [node]
    while stack:
        current = stack.pop()
        if current is None or id(current) in seen_nodes:
            continue
        seen_nodes.add(id(current))
        if isinstance(current, yaml.MappingNode):
            keys = set()
            for key_node, value_node in current.value:
                key = (key_node.tag, key_node.value) if isinstance(key_node, yaml.ScalarNode) else id(key_node)
                if key in keys:
                    raise ProfileError('duplicate_key', str(getattr(key_node, 'value', '?')))
                keys.add(key)
                stack.append(value_node)
        elif isinstance(current, yaml.SequenceNode):
            stack.extend(current.value)


# --- drive-curve helpers (pure; also used by drive.py) ---------------------------------------------------------
def fit_duty_speed_line(points):
    """Least-squares line v = slope * duty_pct + intercept through (duty_pct, speed_m_s) points.

    Returns dict(slope_m_s_per_pct, intercept_m_s, deadband_pct, r_omega0_m_s); the G-SIM
    DC line is v = r_omega0 * (d - d0) with d0 = deadband_pct / 100 (design 3.6).
    """
    pts = [(float(d), float(v)) for d, v in points]
    if len(pts) < 2:
        raise ValueError('duty_speed_line_needs_2_points')
    n = float(len(pts))
    mean_d = sum(d for d, _ in pts) / n
    mean_v = sum(v for _, v in pts) / n
    sxx = sum((d - mean_d) ** 2 for d, _ in pts)
    if sxx == 0.:
        raise ValueError('duty_speed_line_needs_distinct_duties')
    slope = sum((d - mean_d) * (v - mean_v) for d, v in pts) / sxx
    if slope <= 0.:
        raise ValueError('duty_speed_line_slope_not_positive')
    intercept = mean_v - slope * mean_d
    return dict(slope_m_s_per_pct=slope, intercept_m_s=intercept, deadband_pct=-intercept / slope,
                r_omega0_m_s=100. * slope)


def curve_fit_points(get, curve):
    """(duty_pct, speed_m_s) points behind a curve; get is a name -> value callable (e.g. profile.value)."""
    if curve == 'P_fit1003':
        return [(float(d), float(v)) for d, v in get('drive_curve_P_fit1003')['fit_points_pct_m_s']]
    if curve == 'field1010':
        measured = get('drive_curve_field1010_measured')
        return [(90., float(measured['speed_at_90_m_s'])), (100., float(measured['speed_at_100_m_s']))]
    raise ValueError('unknown_drive_curve:%r' % (curve,))


def drive_wheel_radius(get):
    """Radius of the driven wheels: front for front2, rear otherwise (4wd pairs share the rear value)."""
    return get('wheel_radius_front_m') if get('drive_wheels') == 'front2' else get('wheel_radius_rear_m')


# --- derivations ---------------------------------------------------------------------------------------------
def _wheel_limits_rad(get):
    limits = get('steer_mech_limit_deg')
    return {key: math.radians(limits[key]) for key in geometry.WHEEL_LIMIT_KEYS}


def _soft_limit_eq_deg(get, index):
    limits = geometry.equivalent_limits(_wheel_limits_rad(get), math.radians(get('steer_soft_margin_deg')),
                                        get('steer_type'), get('wheelbase_m'), get('kingpin_width_m'),
                                        get('steer_ackermann_ratio'))
    return math.degrees(limits[index])


def _sweep(get):
    return geometry.sweep_at_limits(_wheel_limits_rad(get), body_half_width_m=get('footprint_width_m') / 2.,
                                    steer_type=get('steer_type'), track_front_m=get('track_front_m'),
                                    kingpin_width_m=get('kingpin_width_m'), wheel_radius_m=get('wheel_radius_front_m'),
                                    wheel_width_m=get('wheel_width_m'))


def _main_line(get):
    return fit_duty_speed_line(curve_fit_points(get, get('forward_curve')['main']))


_EQ_LIMIT_INPUTS = ('steer_mech_limit_deg', 'steer_type', 'wheelbase_m', 'kingpin_width_m', 'steer_ackermann_ratio')
_LINK_INPUTS = _EQ_LIMIT_INPUTS + ('steer_zero_offset_deg', 'steer_lr_asymmetry', 'steer_link_table')
_POT_TRAVEL_INPUTS = _LINK_INPUTS + ('pot_to_steer_ratio', 'steer_backlash_deg', 'pot_location')
_CAL_MARK_INPUTS = _LINK_INPUTS + ('pot_to_steer_ratio', 'pot_range_deg', 'adc_bits', 'pot_gain_error', 'pot_center_raw',
                                   'pot_linearity_pct')


def _link_of_delta(get):
    """link(delta_eq) on the nominal link: zero offset at its centre value, left / right gain, link table inverse."""
    zero = math.radians(_nominal_number(get('steer_zero_offset_deg')))
    lr = float(get('steer_lr_asymmetry'))
    table = geometry.LinkTable(get('steer_link_table'))

    def link(delta):
        u = delta - zero
        return table.pot_from_wheel(u / (1. + lr) if u >= 0. else u / (1. - lr))
    return link


def pot_travel_deg(get):
    """Pot shaft rotation (deg) from the right to the left end stop: the link travel (plus the backlash when the pot
    sits on the motor side) times pot_to_steer_ratio. Compare with pot_range_deg (the electrical range)."""
    left, right = geometry.equivalent_limits(_wheel_limits_rad(get), 0., get('steer_type'), get('wheelbase_m'),
                                             get('kingpin_width_m'), get('steer_ackermann_ratio'))
    link = _link_of_delta(get)
    play = math.radians(get('steer_backlash_deg')) if get('pot_location') == 'motor' else 0.
    return math.degrees(link(left) - link(-right) + play) * get('pot_to_steer_ratio')


def _nominal_raw(get):
    n = 2. ** get('adc_bits')
    gain = n * get('pot_to_steer_ratio') * (1. + get('pot_gain_error')) / math.radians(get('pot_range_deg'))
    centre = float(get('pot_center_raw'))
    linearity = get('pot_linearity_pct') / 100.

    def raw(theta):
        r0 = centre + gain * theta
        return r0 + linearity * n * math.sin(2. * math.pi * (r0 - centre) / n)
    return raw


def cal_mark_half_widths(get):
    """(left, right) count distances of the end calibration marks from the centre mark (LOCKSTEP 11.11: marks at the
    equivalent mechanical limit - MARK_INSET_DEG and at 0, nominal noise-free pot, backlash centred). The firmware
    refuses marks closer than MARK_HALF_WIDTH_MIN counts (fw_eeprom.marks_valid)."""
    left, right = geometry.equivalent_limits(_wheel_limits_rad(get), 0., get('steer_type'), get('wheelbase_m'),
                                             get('kingpin_width_m'), get('steer_ackermann_ratio'))
    inset = math.radians(MARK_INSET_DEG)
    link, raw = _link_of_delta(get), _nominal_raw(get)
    centre = raw(link(0.))
    return abs(raw(link(left - inset)) - centre), abs(centre - raw(link(-(right - inset))))


_SOFT_LIMIT_INPUTS = ('steer_mech_limit_deg', 'steer_soft_margin_deg', 'steer_ackermann_ratio', 'wheelbase_m',
                      'kingpin_width_m', 'steer_type')
_CURVE_INPUTS = ('forward_curve', 'drive_curve_P_fit1003', 'drive_curve_field1010_measured')
DERIVATIONS = {
    'scrub_radius_m': (
        ('track_front_m', 'kingpin_width_m', 'kingpin_inclination_deg', 'wheel_radius_front_m'),
        lambda get: geometry.scrub_radius(get('track_front_m'), get('kingpin_width_m'),
                                          math.radians(get('kingpin_inclination_deg')), get('wheel_radius_front_m'))),
    'steer_sweep_m': (
        ('wheel_radius_front_m', 'wheel_width_m', 'kingpin_width_m', 'track_front_m', 'steer_mech_limit_deg',
         'footprint_width_m', 'steer_type'), _sweep),
    'wheel_inertia_kgm2': (     # driven wheel (the one of the G-SIM DC line), solid disc
        ('wheel_mass_kg', 'wheel_radius_front_m', 'wheel_radius_rear_m', 'drive_wheels'),
        lambda get: .5 * get('wheel_mass_kg') * drive_wheel_radius(get) ** 2),
    'steer_soft_limit_eq_left_deg': (_SOFT_LIMIT_INPUTS, lambda get: _soft_limit_eq_deg(get, 0)),
    'steer_soft_limit_eq_right_deg': (_SOFT_LIMIT_INPUTS, lambda get: _soft_limit_eq_deg(get, 1)),
    'pot_lsb_deg': (
        ('pot_range_deg', 'adc_bits', 'pot_to_steer_ratio'),
        lambda get: get('pot_range_deg') / 2. ** get('adc_bits') / get('pot_to_steer_ratio')),
    'drive_omega0_rad_s': (
        _CURVE_INPUTS + ('wheel_radius_rear_m', 'wheel_radius_front_m', 'drive_wheels'),
        lambda get: _main_line(get)['r_omega0_m_s'] / drive_wheel_radius(get)),
    'drive_deadband_frac': (_CURVE_INPUTS, lambda get: _main_line(get)['deadband_pct'] / 100.),
    # schema v3
    'belt_tooth_step_deg': (
        ('belt_pulley_teeth_pot', 'pot_to_steer_ratio'),
        lambda get: 360. / get('belt_pulley_teeth_pot') / get('pot_to_steer_ratio')),
    'pot_travel_deg': (_POT_TRAVEL_INPUTS, lambda get: pot_travel_deg(get)),
    'pot_cal_half_width_counts': (_CAL_MARK_INPUTS, lambda get: min(cal_mark_half_widths(get))),
}


# --- semantic checks -------------------------------------------------------------------------------------------
def _semantic_checks(values):
    # design 3.3 L1: a standing car steers slower and needs more PWM to start the steering motor
    if values['steer_rate_standstill_deg_s'] > values['steer_rate_max_deg_s']:
        raise ProfileError('standstill_order', 'steer_rate_standstill_deg_s',
                           '%r deg/s is above steer_rate_max_deg_s %r deg/s; the standstill rate must not exceed '
                           'the moving rate (sweep both together)' % (values['steer_rate_standstill_deg_s'],
                                                                      values['steer_rate_max_deg_s']))
    if values['steer_min_pwm_standstill_pct'] < values['steer_min_pwm_pct']:
        raise ProfileError('standstill_order', 'steer_min_pwm_standstill_pct',
                           '%r %% is below steer_min_pwm_pct %r %%; the standstill minimum PWM must not be below '
                           'the moving one (sweep both together)' % (values['steer_min_pwm_standstill_pct'],
                                                                     values['steer_min_pwm_pct']))
    try:
        geometry.LinkTable(values['steer_link_table'])
    except (TypeError, ValueError) as exc:
        raise ProfileError('bad_value', 'steer_link_table', str(exc)) from None
    for name in ('pot_valid_raw', 'cal_end_raw_window'):
        if values[name]['min_raw'] >= values[name]['max_raw']:
            raise ProfileError('bad_value', name, 'min_raw must be below max_raw')
    valid, window = values['pot_valid_raw'], values['cal_end_raw_window']
    if not valid['min_raw'] <= window['min_raw'] < window['max_raw'] <= valid['max_raw']:
        raise ProfileError('bad_value', 'cal_end_raw_window', 'must lie inside pot_valid_raw')
    if not window['min_raw'] < values['pot_center_raw'] < window['max_raw']:
        raise ProfileError('bad_value', 'pot_center_raw', 'must lie inside cal_end_raw_window')
    roles = values['forward_curve']
    if roles['main'] == roles['bracket']:
        raise ProfileError('bad_value', 'forward_curve', 'main and bracket must differ')
    fit = values['drive_curve_P_fit1003']
    if not 0. < fit['deadband_pct'] < 90. or not 0. < fit['speed_at_90_m_s'] < fit['speed_at_100_m_s'] <= 3.:
        raise ProfileError('bad_value', 'drive_curve_P_fit1003', 'need 0 < deadband < 90 and 0 < v90 < v100 <= 3')
    duties = [row[0] for row in fit['fit_points_pct_m_s']]
    if len(duties) < 2 or any(b <= a for a, b in zip(duties, duties[1:])) or any(
            len(row) != 2 for row in fit['fit_points_pct_m_s']):
        raise ProfileError('bad_value', 'drive_curve_P_fit1003', 'fit points need >= 2 rows with rising duty')
    measured = values['drive_curve_field1010_measured']
    if not 0. < measured['speed_at_90_m_s'] < measured['speed_at_100_m_s'] <= 3.:
        raise ProfileError('bad_value', 'drive_curve_field1010_measured', 'need 0 < v90 < v100 <= 3')
    if not 0. < values['drive_curve_field1010_deadband_pct'] < 90.:
        raise ProfileError('bad_value', 'drive_curve_field1010_deadband_pct', 'need 0 < deadband < 90')
    for name in ('stop_release', 'stop_brake', 'stop_coast'):
        mode = values[name]
        if mode['decel_median_m_s2'] <= 0. or mode['decel_log_sd'] < 0. or mode['onset_s'] < 0.:
            raise ProfileError('bad_value', name, 'need decel_median > 0, decel_log_sd >= 0, onset_s >= 0')
    if values['sim_mcu_tick_hz'] % values['servo_hz']:
        raise ProfileError('bad_value', 'servo_hz', 'sim_mcu_tick_hz must be a multiple of servo_hz')


def _semantic_checks_v3(values, strict_pot=True):
    """Schema v3 checks (design v2 V2-6.5, decisions R36). strict_pot False (scenario overrides with
    allow_out_of_range) leaves the pot range checks to warnings()."""
    if values['steer_limit_switch'] != 'none':
        raise ProfileError('not_modelled', 'steer_limit_switch',
                           'end switches are not modelled: the ESP8266 has one free pin (D4) and the firmware V3 reads '
                           'no switch; give the wiring with the specification first (SPEC_UPDATE_KO.md, Q25)')
    if values['belt_skip_per_10000deg'] > 0. and values['belt_type'] != 'timing':
        raise ProfileError('bad_value', 'belt_skip_per_10000deg', 'tooth skips need belt_type timing')
    if not strict_pot:
        return
    get = values.__getitem__
    low, high = sorted(pot_end_stop_raws(get))
    window = values['cal_end_raw_window']
    if low < window['min_raw'] or high > window['max_raw']:
        raise ProfileError('pot_range_exceeded', 'pot_to_steer_ratio',
                           'the steering travel x pot_to_steer_ratio puts the end stops at raw %.1f / %.1f, outside '
                           'cal_end_raw_window %d..%d (pot_range_deg %g, pot_travel_deg %.1f): a smaller belt ratio or '
                           'a pot with a wider range' % (low, high, window['min_raw'], window['max_raw'],
                                                          values['pot_range_deg'], pot_travel_deg(get)))
    half = min(cal_mark_half_widths(get))
    if half < MARK_HALF_WIDTH_MIN:
        raise ProfileError('pot_cal_half_width_small', 'pot_to_steer_ratio',
                           'the calibration marks are %.1f counts from the centre (need >= %d, fw marks_valid): a larger '
                           'belt ratio' % (half, MARK_HALF_WIDTH_MIN))


def _semantic_checks_v4(values):
    """Schema v4 checks: the firmware law constants keep the order the firmware relies on."""
    if values['steer_fw_mode'] not in STEER_FW_MODES:
        raise ProfileError('out_of_range', 'steer_fw_mode', 'one of %s' % ', '.join(STEER_FW_MODES))
    fw = values['abs_hold_fw']
    if not 0 < fw['cal_right'] < fw['cal_center'] < fw['cal_left'] < 4096:
        raise ProfileError('bad_value', 'abs_hold_fw', 'need 0 < cal_right < cal_center < cal_left < 4096 (12-bit ADC, '
                                                       'the pot value falls toward the right)')
    if not 0 < fw['target_tol_counts'] <= fw['restart_tol_counts']:
        raise ProfileError('bad_value', 'abs_hold_fw', 'need 0 < target_tol_counts <= restart_tol_counts')
    if not 0 < fw['pulse_short_ms'] <= fw['pulse_long_ms'] < fw['drive_budget_ms']:
        raise ProfileError('bad_value', 'abs_hold_fw', 'need 0 < pulse_short_ms <= pulse_long_ms < drive_budget_ms')
    if not 0 < fw['pulse_pwm'] <= 255 or fw['median_of'] % 2 != 1 or fw['median_of'] < 1:
        raise ProfileError('bad_value', 'abs_hold_fw', 'need 0 < pulse_pwm <= 255 and an odd median_of')
    if fw['rebound_window_ms'] > fw['settle_ms'] or fw['sample_ms'] <= 0 or fw['stale_ms'] <= fw['sample_ms']:
        raise ProfileError('bad_value', 'abs_hold_fw', 'need rebound_window_ms <= settle_ms and 0 < sample_ms < stale_ms')
    host = values['abs_hold_host']
    if not 0. < host['step_deg'] <= host['deadband_deg'] or host['min_interval_ms'] < host['poll_ms'] or             host['poll_ms'] <= 0:
        raise ProfileError('bad_value', 'abs_hold_host', 'need 0 < step_deg <= deadband_deg and 0 < poll_ms <= '
                                                         'min_interval_ms')
    timing = values['steer_step_time_measured_s']
    if not (timing['return_20'] > 0. and timing['outward_20'] > 0.):
        raise ProfileError('bad_value', 'steer_step_time_measured_s', 'step times must be > 0')


def abs_hold_mode(profile):
    """True when the profile runs the ABS-HOLD-8 pulse law in the SIM ESP emulator (schema v4, steer_fw_mode)."""
    return profile.has('steer_fw_mode') and profile.value('steer_fw_mode') == 'abs_hold_8'


# --- consistency warnings ----------------------------------------------------------------------------------------
def _nominal_number(value):
    """A distribution's centre (uniform mid, normal mean) or the number itself."""
    if _is_distribution(value):
        if value['dist'] == 'uniform':
            return .5 * (float(value['low']) + float(value['high']))
        return float(value['mean'])
    return float(value)


def pot_end_stop_raws(get):
    """Nominal raw ADC values (left stop, right stop) at the steering end stops, before rounding and clipping.

    get is a name -> value callable (e.g. VehicleProfile.value). The measured angle is the motor side
    (pot_location motor: the link end stop +- half the backlash) or the link side (wheel), with the nominal
    zero offset, no noise and full supply; the raw follows the protocol section 7 formula (gain error and
    linearity term included).
    """
    left, right = geometry.equivalent_limits(_wheel_limits_rad(get), 0., get('steer_type'), get('wheelbase_m'),
                                             get('kingpin_width_m'), get('steer_ackermann_ratio'))
    zero = math.radians(_nominal_number(get('steer_zero_offset_deg')))
    lr = float(get('steer_lr_asymmetry'))
    table = geometry.LinkTable(get('steer_link_table'))

    def link(delta):
        u = delta - zero
        return table.pot_from_wheel(u / (1. + lr) if u >= 0. else u / (1. - lr))
    half = math.radians(get('steer_backlash_deg')) / 2. if get('pot_location') == 'motor' else 0.
    n = 2. ** get('adc_bits')
    gain = n * get('pot_to_steer_ratio') * (1. + get('pot_gain_error')) / math.radians(get('pot_range_deg'))
    centre = float(get('pot_center_raw'))
    linearity = get('pot_linearity_pct') / 100.

    def raw(theta):
        r0 = centre + gain * theta
        return r0 + linearity * n * math.sin(2. * math.pi * (r0 - centre) / n)
    return raw(link(left) + half), raw(link(-right) - half)


def _consistency_warnings(get):
    """Combinations that load but give a known artefact (reported in warnings() and result_label())."""
    out = []
    servo_min, servo_max = get('servo_min_pwm_pct'), get('servo_max_pwm_pct')
    kp, deadband = get('servo_kp_pct_per_deg'), get('servo_deadband_deg')
    lowest = min(servo_max, max(servo_min, kp * deadband))   # smallest servo output just outside the deadband
    for name, when in (('steer_min_pwm_pct', 'moving'), ('steer_min_pwm_standstill_pct', 'standing')):
        start = get(name)
        if lowest <= start:
            out.append(dict(
                code='servo_min_pwm_cannot_start_the_motor', names=['servo_min_pwm_pct', name],
                detail='servo output just outside the deadband %g %% <= %s %g %%: while %s the test servo (and a '
                       'firmware servo with this minimum) cannot start the steering motor near the target; the '
                       'steering stops about %.2f deg from the target, outside the %g deg deadband'
                       % (lowest, name, start, when, start / kp if servo_max > start else float('inf'), deadband)))
    window, valid = get('cal_end_raw_window'), get('pot_valid_raw')
    low, high = sorted(pot_end_stop_raws(get))
    names = ['pot_to_steer_ratio', 'pot_range_deg', 'pot_center_raw', 'pot_gain_error', 'pot_linearity_pct',
             'steer_mech_limit_deg', 'steer_backlash_deg', 'pot_location']
    if low < window['min_raw'] or high > window['max_raw']:
        out.append(dict(code='pot_end_stops_outside_cal_window', names=names + ['cal_end_raw_window'],
                        detail='nominal raw %.1f / %.1f at the end stops is outside cal_end_raw_window %d..%d: the '
                               'firmware end-point calibration (B2) would refuse them (NOCAL, P3)'
                               % (low, high, window['min_raw'], window['max_raw'])))
    if low < valid['min_raw'] or high > valid['max_raw']:
        out.append(dict(code='pot_end_stops_outside_valid_raw', names=names + ['pot_valid_raw'],
                        detail='nominal raw %.1f / %.1f at the end stops is outside pot_valid_raw %d..%d: the MCU would '
                               'read POT_OPEN / POT_SHORT at the end stops (P3)'
                               % (low, high, valid['min_raw'], valid['max_raw'])))
    return out


# --- sampling --------------------------------------------------------------------------------------------------
def _unit_uniform(seed, name):
    """Deterministic uniform in (0, 1) from (seed, name); independent per name."""
    digest = hashlib.sha256(('ubts.vehicle_profile.sample.v1|%d|%s' % (seed, name)).encode('utf-8')).digest()
    k = int.from_bytes(digest[:8], 'big') >> 12  # 52 bits, so (k + 0.5) / 2**52 is exact and < 1
    return (k + .5) / 2. ** 52


def _draw(value, rng, u):
    lo, hi = float(rng[0]), float(rng[1])
    if value['dist'] == 'uniform':
        return float(value['low']) + u * (float(value['high']) - float(value['low']))
    mean, sd = float(value['mean']), float(value['sd'])
    if sd == 0.:
        return mean
    dist = statistics.NormalDist(mean, sd)
    a, b = dist.cdf(lo), dist.cdf(hi)
    p = a + u * (b - a)
    if not 0. < p < 1.:
        return min(hi, max(lo, mean))
    return min(hi, max(lo, dist.inv_cdf(p)))


# --- the profile object ------------------------------------------------------------------------------------------
class VehicleProfile:
    """A validated vehicle profile. Use load_profile() or parse_profile() to build one."""

    def __init__(self, *, schema, profile_id, description, sha256, origin, records, overrides=None,
                 override_label=None, out_of_range_overrides=()):
        self.schema = schema
        self.profile_id = profile_id
        self.description = description
        self.sha256 = sha256
        self.origin = origin
        self._records = records
        self._overrides = dict(overrides or {})
        self._override_label = override_label
        self._out_of_range = tuple(out_of_range_overrides)
        self._warnings = None

    def __repr__(self):
        extra = '' if not self._overrides else ' overrides=%s' % self._override_label
        return '<VehicleProfile %s sha256=%s%s>' % (self.profile_id, self.sha256[:12], extra)

    # -- access
    def names(self):
        """Parameter names in file order."""
        return list(self._records)

    def has(self, name):
        """True when this view has the parameter (schema v3 names are absent from a v2 file)."""
        return name in self._records

    @property
    def schema_version(self):
        return {SCHEMA_V3: 3, SCHEMA_V4: 4}.get(self.schema, 2)

    def _record(self, name):
        try:
            return self._records[name]
        except KeyError:
            raise KeyError('unknown profile parameter %r' % (name,)) from None

    def value(self, name):
        """Value of a parameter (computed for DERIVED; None for undecided SPEC_PENDING). A copy."""
        return copy.deepcopy(self._record(name)['value'])

    def param(self, name):
        """Full record (copy): value, range, unit, provenance, identify, stale_after_mod, source, ..."""
        return copy.deepcopy(self._record(name))

    def provenance(self, name):
        return self._record(name)['provenance']

    def is_derived(self, name):
        return self._record(name)['provenance'] == 'DERIVED'

    def spec_pending(self):
        """Names whose provenance is SPEC_PENDING (values waiting for the user's specification)."""
        return [name for name, rec in self._records.items() if rec['provenance'] == 'SPEC_PENDING']

    def stale_after_mod(self):
        """Names marked stale_after_mod (obtained before the steering modification; DERIVED: from inputs)."""
        return [name for name, rec in self._records.items() if rec['stale_after_mod']]

    def provenance_counts(self):
        counts = {}
        for rec in self._records.values():
            counts[rec['provenance']] = counts.get(rec['provenance'], 0) + 1
        return dict(sorted(counts.items()))

    def provenance_table(self):
        """One JSON-friendly row per parameter (file order) for reports."""
        rows = []
        for name, rec in self._records.items():
            row = dict(name=name, group=GROUP_OF[name], value=copy.deepcopy(rec['value']), unit=rec['unit'],
                       range=copy.deepcopy(rec['range']), provenance=rec['provenance'],
                       stale_after_mod=rec['stale_after_mod'], identify=list(rec['identify']),
                       source=rec.get('source'), derived_from=list(rec.get('derived_from') or []) or None,
                       where=rec.get('where'), note=rec.get('note'))
            if name in self._overrides:
                row['override'] = self._override_label
                row['override_out_of_range'] = name in self._out_of_range
            rows.append(row)
        return rows

    def provenance_grade(self):
        """'placeholder' if any value is SPEC_PENDING; 'partially_measured' if any value is not MEASURED_*,
        FIRMWARE, CONFIG, DEFINITION, GENERIC_SPEC, PRODUCT_LABEL, SIM_NUMERIC or DERIVED; else 'measured'."""
        tags = {rec['provenance'] for rec in self._records.values()}
        if 'SPEC_PENDING' in tags:
            return 'placeholder'
        settled = {'FIRMWARE', 'CONFIG', 'DEFINITION', 'GENERIC_SPEC', 'PRODUCT_LABEL', 'SIM_NUMERIC', 'DERIVED'}
        if any(not (tag in settled or tag.startswith('MEASURED_')) for tag in tags):
            return 'partially_measured'
        return 'measured'

    def result_label(self):
        """Identity block to attach to every SIM result (design 1.4 rule 6)."""
        grade = self.provenance_grade()
        text = {'placeholder': RESULT_LABEL_PLACEHOLDER, 'partially_measured': RESULT_LABEL_PARTIAL,
                'measured': RESULT_LABEL_MEASURED}[grade]
        label = dict(schema=self.schema, profile_id=self.profile_id, profile_sha256=self.sha256,
                     provenance_grade=grade, result_kind=text, field_validated=False,
                     spec_pending_count=len(self.spec_pending()), overrides=None,
                     warnings=[dict(code=w['code'], names=list(w['names'])) for w in self.warnings()])
        if self._overrides:
            payload = json.dumps(dict(label=self._override_label, values=self._overrides), sort_keys=True,
                                 separators=(',', ':'), ensure_ascii=False)
            label['overrides'] = dict(label=self._override_label, names=sorted(self._overrides),
                                      out_of_range=sorted(self._out_of_range),
                                      sha256=hashlib.sha256(payload.encode('utf-8')).hexdigest())
        return label

    @property
    def overrides(self):
        return copy.deepcopy(self._overrides)

    def warnings(self):
        """Consistency warnings [{code, names, detail}] of this view (empty for the shipped profile).

        servo_min_pwm_cannot_start_the_motor: the smallest servo output outside the deadband does not start the
          steering motor (moving or standing), so the servo stops outside its deadband (design 3.3 L1).
        pot_end_stops_outside_cal_window / pot_end_stops_outside_valid_raw: the nominal raw ADC values at the
          end stops fall outside the calibration window / the valid raw range (design 3.4; NOCAL / POT_* in P3).
        """
        if self._warnings is None:
            self._warnings = _consistency_warnings(lambda name: self._record(name)['value'])
        return copy.deepcopy(self._warnings)

    # -- per-run draws
    def sampled_names(self):
        """Names whose value is a per-run distribution (design 3.5 / 3.8 F, C, E)."""
        return [name for name, rec in self._records.items() if _is_distribution(rec['value'])]

    def sample(self, seed, names=None):
        """Seeded per-run draws {name: float} inside each parameter's range.

        Deterministic: the draw of a name depends only on (seed, name) and its distribution,
        so adding names does not change other draws. names=None draws all sampled_names().
        """
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ValueError('sample_seed_must_be_non_negative_int')
        names = self.sampled_names() if names is None else list(names)
        out = {}
        for name in names:
            rec = self._record(name)
            if not _is_distribution(rec['value']):
                raise ValueError('not_a_per_run_distribution:' + name)
            out[name] = _draw(rec['value'], rec['range'], _unit_uniform(seed, name))
        return out

    def per_run_value(self, seed, name):
        """The value of a per-run parameter in the run with this seed (float).

        A distribution gives its seeded draw (the same number as sample(seed, [name])); a plain number (a fixed
        per-run value, e.g. a scenario condition) is returned as it is. Both backends and the plant use this rule.
        """
        value = self._record(name)['value']
        if _is_distribution(value):
            return self.sample(seed, (name,))[name]
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ValueError('sample_seed_must_be_non_negative_int')
        return self.nominal(name)

    def ideal_draws(self):
        """Ideal values (eta 1, scrub 0, understeer 0, offsets 0) for geometry checks only (A1, A12)."""
        return {name: IDEAL_DRAWS[name] for name in self.sampled_names() if name in IDEAL_DRAWS}

    def nominal(self, name):
        """A number for a parameter: the value, or the centre of a distribution (uniform mid, normal mean)."""
        value = self._record(name)['value']
        if not (_is_distribution(value) or _is_number(value)):
            raise ValueError('nominal_needs_a_number_or_distribution:' + name)
        return _nominal_number(value)

    # -- scenario overrides
    def with_overrides(self, overrides, *, label, allow_out_of_range=False):
        """New profile view with some non-DERIVED values replaced (scenario conditions, sweeps).

        DERIVED values are recomputed. Values must have the parameter's shape (SHAPES; a per-run
        parameter takes a number or a distribution) and keep its choices; numeric bounds may be
        exceeded only with allow_out_of_range=True (they are then listed). Semantic checks apply.
        The file identity (profile_id, sha256) is kept; result_label() reports the overrides.
        """
        if not isinstance(label, str) or not label:
            raise ValueError('override_label_required')
        if not isinstance(overrides, dict) or not overrides:
            raise ValueError('overrides_must_be_a_non_empty_dict')
        merged = dict(self._overrides)
        out_of_range = set(self._out_of_range)
        raw = {name: copy.deepcopy(rec) for name, rec in self._records.items()}
        for name, value in overrides.items():
            rec = raw.get(name)
            if rec is None:
                raise ProfileError('parameter_unknown', name)
            if rec['provenance'] == 'DERIVED':
                raise ProfileError('override_derived', name)
            value = copy.deepcopy(value)
            if value is None or rec['range'] is None:
                raise ProfileError('null_not_allowed', name, 'cannot override an undecided value')
            try:
                _check_range(name, value, rec['range'])
                out_of_range.discard(name)
            except ProfileError as exc:
                if exc.code != 'out_of_range' or not allow_out_of_range:
                    raise
                _check_range(name, value, rec['range'], check_bounds=False)
                out_of_range.add(name)
            _check_shape(name, value, shape_of(name))
            rec['value'] = value
            merged[name] = copy.deepcopy(value)
        records, derived_out = _finish_records(raw, derived_bounds=not allow_out_of_range)
        out_of_range.update(derived_out)
        return VehicleProfile(schema=self.schema, profile_id=self.profile_id, description=self.description,
                              sha256=self.sha256, origin=self.origin, records=records, overrides=merged,
                              override_label=label if not self._override_label else self._override_label + '+' + label,
                              out_of_range_overrides=sorted(out_of_range))


# --- loading ----------------------------------------------------------------------------------------------------
def _validate_record(name, rec, all_names):
    if not isinstance(rec, dict):
        raise ProfileError('bad_record', name, 'a parameter record must be a mapping')
    unknown = sorted(set(rec) - set(RECORD_KEYS))
    if unknown:
        raise ProfileError('field_unknown', name, ', '.join(map(str, unknown)))
    if 'provenance' not in rec:
        raise ProfileError('field_missing', name, 'provenance')
    tag = rec['provenance']
    try:
        parse_provenance(tag)
    except ProfileError as exc:
        raise ProfileError(exc.code, name, exc.detail) from None
    derived = tag == 'DERIVED'
    if derived and 'derived_from' not in rec:
        raise ProfileError('derived_from_missing', name)
    for field_name in (REQUIRED_FIELDS_DERIVED if derived else REQUIRED_FIELDS):
        if field_name not in rec:
            raise ProfileError('field_missing', name, field_name)
    if 'derived_from' in rec and not derived:
        raise ProfileError('derived_from_not_allowed', name)
    if derived:
        inputs = rec['derived_from']
        if (not isinstance(inputs, list) or not inputs or not all(isinstance(x, str) for x in inputs)
                or len(set(inputs)) != len(inputs)):
            raise ProfileError('bad_derived_from', name, 'need a non-empty list of distinct names')
        missing = [x for x in inputs if x not in all_names]
        if missing:
            raise ProfileError('bad_derived_from', name, 'unknown inputs: ' + ', '.join(missing))
    source = rec.get('source')
    if _source_required(tag) and (not isinstance(source, str) or not source.strip()):
        raise ProfileError('source_missing', name, tag)
    if 'source' in rec and (not isinstance(rec['source'], str) or not rec['source'].strip()):
        raise ProfileError('bad_field', name, 'source must be a non-empty string')
    if not isinstance(rec['unit'], str) or not rec['unit'].strip():
        raise ProfileError('bad_field', name, 'unit must be a non-empty string')
    identify = _check_identify(name, rec['identify'])
    if 'stale_after_mod' in rec and not isinstance(rec['stale_after_mod'], bool):
        raise ProfileError('bad_field', name, 'stale_after_mod must be true or false')
    if 'where' in rec and rec['where'] not in WHERE_VALUES:
        raise ProfileError('bad_field', name, 'where must be one of ' + ', '.join(WHERE_VALUES))
    if 'note' in rec and not isinstance(rec['note'], str):
        raise ProfileError('bad_field', name, 'note must be a string')
    out = dict(rec)
    out['identify'] = identify
    if derived:
        if 'value' in rec:
            raise ProfileError('derived_value_written', name, 'a DERIVED value is computed on load and must not be '
                                                              'written in the file (design 3.8.1)')
        return out
    value, rng = rec['value'], rec['range']
    if value is None or rng is None:
        if not (value is None and rng is None and tag == 'SPEC_PENDING'):
            raise ProfileError('null_not_allowed', name, 'null value needs SPEC_PENDING and a null range')
        if shape_of(name) != NUMBER_OR_NULL:
            raise ProfileError('bad_shape', name, 'expected %s, got None' % _shape_text(shape_of(name)))
        return out
    _check_range(name, value, rng)
    _check_shape(name, value, shape_of(name))
    return out


def _finish_records(records, derived_bounds=True):
    """Compute DERIVED values and stale flags.

    Returns (records in file order, names of DERIVED values outside their written range).
    The second list is empty unless derived_bounds is False (scenario overrides).
    """
    resolved = {name: rec['value'] for name, rec in records.items() if rec['provenance'] != 'DERIVED'}
    stale = {name: rec['stale_after_mod'] for name, rec in records.items() if rec['provenance'] != 'DERIVED'}
    _semantic_checks(resolved)
    if 'belt_type' in records:
        _semantic_checks_v3(resolved, strict_pot=derived_bounds)
    if 'steer_fw_mode' in records:
        _semantic_checks_v4(resolved)
    computing = set()

    def get(name):
        if name in resolved:
            return resolved[name]
        rec = records[name]
        if name in computing:
            raise ProfileError('derivation_cycle', name)
        computing.add(name)
        if name not in DERIVATIONS:
            raise ProfileError('derivation_unknown', name)
        inputs, function = DERIVATIONS[name]
        if set(rec['derived_from']) != set(inputs):
            raise ProfileError('derived_from_mismatch', name, 'computation uses ' + ', '.join(sorted(inputs)))
        for item in inputs:
            get(item)
        try:
            value = function(get)
        except ProfileError:
            raise
        except (ArithmeticError, TypeError, ValueError, KeyError) as exc:
            raise ProfileError('derivation_failed', name, str(exc)) from None
        resolved[name] = value
        stale[name] = any(stale[item] for item in inputs)
        computing.discard(name)
        return value

    out, derived_out = {}, []
    for name, rec in records.items():
        new = dict(rec)
        if rec['provenance'] == 'DERIVED':
            value = get(name)
            if rec.get('range') is not None:
                try:
                    _check_range(name, value, rec['range'])
                except ProfileError as exc:
                    if exc.code != 'out_of_range' or derived_bounds:
                        raise
                    derived_out.append(name)
            if 'stale_after_mod' in rec and rec['stale_after_mod'] != stale[name]:
                raise ProfileError('derived_stale_mismatch', name, 'inputs give %r' % (stale[name],))
            new['value'] = value
            new['stale_after_mod'] = stale[name]
            new.setdefault('range', None)
        out[name] = new
    return out, derived_out


def parse_profile(data, origin='<memory>'):
    """Validate profile bytes (UTF-8 YAML) and return a VehicleProfile. Raises ProfileError."""
    if isinstance(data, str):
        data = data.encode('utf-8')
    if not isinstance(data, (bytes, bytearray)):
        raise TypeError('profile data must be bytes')
    data = bytes(data)
    digest = hashlib.sha256(data).hexdigest()
    try:
        _check_duplicate_keys(data)
        doc = yaml.load(data, Loader=SAFE_LOADER)
    except yaml.YAMLError as exc:
        raise ProfileError('yaml_error', '', str(exc).splitlines()[0] if str(exc) else 'parse error') from None
    if not isinstance(doc, dict):
        raise ProfileError('bad_document', '', 'top level must be a mapping')
    unknown = sorted(str(k) for k in set(doc) - set(TOP_LEVEL_KEYS))
    if unknown:
        raise ProfileError('top_level_key_unknown', '', ', '.join(unknown))
    for key in REQUIRED_TOP_LEVEL_KEYS:
        if key not in doc:
            raise ProfileError('top_level_key_missing', '', key)
    if doc['schema'] not in NAMES_OF_SCHEMA:
        raise ProfileError('schema_mismatch', '', '%r (need one of %r)' % (doc['schema'], SCHEMAS))
    schema = doc['schema']
    names_of_schema = NAMES_OF_SCHEMA[schema]
    profile_id = doc['profile_id']
    if not isinstance(profile_id, str) or not _PROFILE_ID.match(profile_id):
        raise ProfileError('bad_profile_id', '', repr(profile_id))
    description = doc.get('description')
    if description is not None and not isinstance(description, str):
        raise ProfileError('bad_document', '', 'description must be a string')
    params = doc['parameters']
    if not isinstance(params, dict):
        raise ProfileError('bad_document', '', 'parameters must be a mapping')
    unknown = sorted(str(k) for k in set(params) - set(names_of_schema))
    if unknown:
        raise ProfileError('parameter_unknown', unknown[0], ', '.join(unknown))
    missing = [name for name in names_of_schema if name not in params]
    if missing:
        raise ProfileError('parameter_missing', missing[0], ', '.join(missing))
    all_names = set(names_of_schema)
    records = {name: _validate_record(name, params[name], all_names) for name in names_of_schema}
    for name, rec in records.items():
        if rec['provenance'] == 'DERIVED' and name not in DERIVATIONS:
            raise ProfileError('derivation_unknown', name)
        if name in DERIVATIONS and rec['provenance'] != 'DERIVED':
            raise ProfileError('derived_tag_required', name, 'computed from %s; tag it DERIVED (got %s)'
                               % (', '.join(DERIVATIONS[name][0]), rec['provenance']))
    records, _ = _finish_records(records)
    return VehicleProfile(schema=schema, profile_id=profile_id, description=description, sha256=digest,
                          origin=origin, records=records)


def load_profile(path=None):
    """Load and validate a profile file (default: the shipped placeholder profile)."""
    path = DEFAULT_PROFILE_PATH if path is None else Path(path)
    return parse_profile(path.read_bytes(), origin=str(path))
