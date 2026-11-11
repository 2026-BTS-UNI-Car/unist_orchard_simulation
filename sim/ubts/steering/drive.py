"""Drive model values for the steering SIM (design 3.5 and 3.6).

Values come from a vehicle_profile.VehicleProfile. field_response.py is only imported
(forward_curve, FieldResponseParams, PARAMETERS); it is not changed.

Duty/PWM is a COMMAND. Nothing here is a wheel speed, an encoder or odometry.

Curves (PREREG roles, profile forward_curve):
  main     P_fit1003  10/3 fit, 40/55/70/88 % -> 0.33/0.47/0.61/0.78 m/s, deadband 5 %
  bracket  field1010  10/10 measured 90/100 % (0.72/0.90 m/s), deadband 50 % SIM_ASSUMED
Both are old-drivetrain values (stale_after_mod). The reverse curve is forward x reverse_curve.

Battery scale s_b multiplies the effective duty: v = curve(s_b * |duty|). This matches the
G-SIM DC line with a fixed Coulomb friction when omega0 and tau_stall are both scaled by s_b
(v = r*omega0*(s_b*d - d0)).

Stop modes (design 3.5, 4.1): release (EN on, MCU ramp), brake (EN on, duty 0 at once),
coast (EN off). A stop after onset_s holds speed, then decelerates at a per-stop draw
median * exp(log_sd * z). Closed forms: distance = v*onset + v^2/(2a), time = onset + v/a.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Dict, Tuple

from field_response import PARAMETERS as FIELD_PARAMETERS, FieldResponseParams, forward_curve

from .vehicle_profile import CURVE_NAMES, curve_fit_points, drive_wheel_radius, fit_duty_speed_line

ROLES = ('main', 'bracket')
STOP_MODE_NAMES = ('release', 'brake', 'coast')
# single-row sim_in_loop.P_FIT1003 curve overrides (PREREG plants P_fit1003), copied for cross-checks.
P_FIT1003_CURVE_OVERRIDES = {'speed_at_90_m_s': .80, 'forward_deadband_pct': 5.}
# Drive stop mode per event (design 4.1 stop-mode table). 'ttl_expiry' follows the profile.
STOP_EVENT_DRIVE_MODE = {
    'operator_stop': 'brake', 'disarm': 'release', 'throttle_zero': 'release', 'ttl_expiry': None,
    'link_watchdog': 'brake', 'pot_fault': 'brake', 'polarity_fault': 'brake', 'driver_stuck': 'brake',
    'stall_or_track': 'brake', 'thermal': 'release', 'crc_burst': 'brake', 'mcu_reset': 'coast',
    'power_cut': 'coast',
}
# Profile entries copied from field_response.PARAMETERS: (profile name, mapping key or None, field_response name).
FIELD_RESPONSE_COPIES = (
    ('drive_curve_field1010_measured', 'speed_at_90_m_s', 'speed_at_90_m_s'),
    ('drive_curve_field1010_measured', 'speed_at_100_m_s', 'speed_at_100_m_s'),
    ('drive_curve_field1010_deadband_pct', None, 'forward_deadband_pct'),
    ('drive_curve_P_fit1003', 'speed_at_100_m_s', 'speed_at_100_m_s'),
    ('stop_release', 'onset_s', 'coast_onset_s'),
    ('stop_release', 'decel_median_m_s2', 'coast_decel_median_m_s2'),
    ('stop_release', 'decel_log_sd', 'coast_decel_log_sd'),
    ('curvature_wander', 'mean_deg_per_m', 'wander_mean_deg_per_m'),
    ('curvature_wander', 'sd_deg_per_m', 'wander_sd_deg_per_m'),
    ('curvature_wander', 'corr_length_m', 'wander_corr_length_m'),
    ('lateral_slip_sd_m_s', None, 'lateral_slip_sd_m_s'),
    ('speed_run_scale', 'mode', 'speed_run_scale_mode'),
    ('speed_run_scale', 'sd_low', 'speed_run_scale_sd_low'),
    ('speed_run_scale', 'sd_high', 'speed_run_scale_sd_high'),
    ('speed_fluctuation_sd', None, 'speed_fluctuation_sd'),
    ('speed_fluctuation_tau_s', None, 'speed_fluctuation_tau_s'),
    ('drive_accel_tau_s', None, 'accel_tau_s'),
    ('drive_decel_tau_s', None, 'decel_tau_s'),
    ('imu_gyro_scale', 'mean', 'gyro_scale_mean'),
    ('imu_gyro_scale', 'sd', 'gyro_scale_sd'),
    ('imu_gyro_bias_sd_deg_s', None, 'gyro_bias_sd_deg_s'),
    ('imu_gyro_still_noise_sd_deg_s', None, 'gyro_still_noise_sd_deg_s'),
    ('imu_gyro_vibration_deg_s_per_m_s2', None, 'gyro_vibration_deg_s_per_m_s2'),
    ('imu_vibration_drive_at_90_m_s2', None, 'drive_vibration_at_90_m_s2'),
    ('imu_vibration_drive_at_100_m_s2', None, 'drive_vibration_at_100_m_s2'),
    ('imu_vibration_drive_below_90_m_s2', None, 'drive_vibration_rms_m_s2'),
    ('imu_vibration_still_m_s2', 'low', ('still_vibration_rms_m_s2', 0)),
    ('imu_vibration_still_m_s2', 'high', ('still_vibration_rms_m_s2', 1)),
    ('lidar_hz', None, 'lidar_hz'),
    ('lidar_x_in_base_m', None, 'lidar_x_in_base_m'),
    ('footprint_length_m', None, 'footprint_length_m'),
    ('footprint_width_m', None, 'footprint_width_m'),
)


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('drive_value_not_finite:' + name)
    return float(value)


# --- curves -------------------------------------------------------------------------------------------------
def curve_roles(profile) -> Dict[str, str]:
    """{'main': ..., 'bracket': ...} from the profile (PREREG roles)."""
    return dict(profile.value('forward_curve'))


def resolve_curve(profile, curve='main'):
    """'main' / 'bracket' / a curve name -> the curve name."""
    if curve in ROLES:
        return curve_roles(profile)[curve]
    if curve in CURVE_NAMES:
        return curve
    raise ValueError('unknown_drive_curve:%r' % (curve,))


def curve_params(profile, curve='main') -> FieldResponseParams:
    """FieldResponseParams carrying this curve's speed_at_90/100 and deadband (for forward_curve only)."""
    name = resolve_curve(profile, curve)
    if name == 'P_fit1003':
        c = profile.value('drive_curve_P_fit1003')
        return replace(FieldResponseParams(), speed_at_90_m_s=c['speed_at_90_m_s'],
                       speed_at_100_m_s=c['speed_at_100_m_s'], forward_deadband_pct=c['deadband_pct'])
    m = profile.value('drive_curve_field1010_measured')
    return replace(FieldResponseParams(), speed_at_90_m_s=m['speed_at_90_m_s'], speed_at_100_m_s=m['speed_at_100_m_s'],
                   forward_deadband_pct=profile.value('drive_curve_field1010_deadband_pct'))


def steady_speed_m_s(profile, duty_pct, curve='main', battery_scale=None):
    """Steady straight speed (m/s, signed) for a signed duty in percent (-100..100).

    Forward: field_response.forward_curve(s_b * |duty|). Reverse: -reverse_curve x forward speed.
    battery_scale None uses the profile battery_voltage_scale.
    """
    duty = _finite(duty_pct, 'duty_pct')
    if abs(duty) > 100.:
        raise ValueError('drive_duty_out_of_range')
    scale = profile.value('battery_voltage_scale') if battery_scale is None else _finite(battery_scale,
                                                                                          'battery_scale')
    if scale <= 0.:
        raise ValueError('drive_battery_scale_not_positive')
    speed = forward_curve(abs(duty) * scale, curve_params(profile, curve))
    return -profile.value('reverse_curve') * speed if duty < 0. else speed


def drive_vibration_m_s2(profile, duty_pct):
    """IMU vibration RMS while driving (field_response rule): below 90 % one value, 90..100 % linear."""
    duty = abs(_finite(duty_pct, 'duty_pct'))
    if duty < 90.:
        return profile.value('imu_vibration_drive_below_90_m_s2')
    a, b = profile.value('imu_vibration_drive_at_90_m_s2'), profile.value('imu_vibration_drive_at_100_m_s2')
    return a + (min(duty, 100.) - 90.) * (b - a) / 10.


# --- MCU ramp -------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class McuRampSpec:
    """MCU drive ramp (firmware): duty slope and the wait after any duty reaches 0 (reversal dwell)."""

    pwm_per_s: int
    pct_per_s: float
    reversal_dwell_s: float


def mcu_ramp(profile) -> McuRampSpec:
    ramp = profile.value('mcu_ramp')
    return McuRampSpec(pwm_per_s=int(ramp['pwm_per_s']), pct_per_s=ramp['pwm_per_s'] * 100. / 255.,
                       reversal_dwell_s=ramp['reversal_dwell_ms'] / 1000.)


# --- stop modes -------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class StopMode:
    """One drive stop mode: hold speed for onset_s, then decelerate at a per-stop draw."""

    name: str
    onset_s: float
    decel_median_m_s2: float
    decel_log_sd: float
    decel_median_range_m_s2: Tuple[float, float]
    provenance: str

    def decel_m_s2(self, z=0.):
        """Per-stop deceleration median * exp(log_sd * z); z is a standard normal from the caller's stream."""
        return self.decel_median_m_s2 * math.exp(self.decel_log_sd * _finite(z, 'z'))

    def _decel(self, decel_m_s2):
        a = self.decel_median_m_s2 if decel_m_s2 is None else _finite(decel_m_s2, 'decel_m_s2')
        if a <= 0.:
            raise ValueError('drive_decel_not_positive')
        return a

    def distance_m(self, speed_m_s, decel_m_s2=None):
        """Closed-form stop distance from speed (no MCU ramp fade before the onset)."""
        v = abs(_finite(speed_m_s, 'speed_m_s'))
        return v * self.onset_s + v * v / (2. * self._decel(decel_m_s2))

    def time_s(self, speed_m_s, decel_m_s2=None):
        v = abs(_finite(speed_m_s, 'speed_m_s'))
        return self.onset_s + v / self._decel(decel_m_s2)


def stop_modes(profile) -> Dict[str, StopMode]:
    out = {}
    for name in STOP_MODE_NAMES:
        param = profile.param('stop_' + name)
        value, rng = param['value'], param['range']
        out[name] = StopMode(name=name, onset_s=float(value['onset_s']),
                             decel_median_m_s2=float(value['decel_median_m_s2']),
                             decel_log_sd=float(value['decel_log_sd']),
                             decel_median_range_m_s2=(float(rng['decel_median_m_s2'][0]),
                                                      float(rng['decel_median_m_s2'][1])),
                             provenance=param['provenance'])
    return out


def drive_stop_mode(profile, event):
    """Drive stop mode name for a design 4.1 event (STOP_EVENT_DRIVE_MODE keys)."""
    if event not in STOP_EVENT_DRIVE_MODE:
        raise ValueError('unknown_stop_event:%r' % (event,))
    mode = STOP_EVENT_DRIVE_MODE[event]
    return profile.value('ttl_expiry_drive_mode') if mode is None else mode


# --- G-SIM DC motor line ----------------------------------------------------------------------------------------
@dataclass(frozen=True)
class DcMotorLine:
    """Per-wheel DC motor torque-speed line for Gazebo JointController force mode (design 3.6).

    tau = tau_stall * (d - omega / omega0); joint velocity target = d * omega0;
    p_gain = tau_stall / omega0; cmd_max = tau_stall; cmd_min = -tau_stall; i and d gains 0.
    The deadband comes from a joint Coulomb friction d0 * tau_stall_nominal (fixed).
    omega0 and tau_stall are already multiplied by battery_scale.
    """

    curve: str
    fit_points: Tuple[Tuple[float, float], ...]
    slope_m_s_per_pct: float
    deadband_frac: float
    r_omega0_m_s: float
    wheel_radius_m: float
    omega0_nominal_rad_s: float
    tau_stall_nominal_nm: float
    battery_scale: float
    omega0_rad_s: float
    tau_stall_nm: float
    coulomb_friction_nm: float

    @property
    def p_gain(self):
        """N*m per rad/s (= tau_stall / omega0, unchanged by the battery scale)."""
        return self.tau_stall_nm / self.omega0_rad_s

    @property
    def cmd_max_nm(self):
        return self.tau_stall_nm

    @property
    def cmd_min_nm(self):
        return -self.tau_stall_nm

    def joint_velocity_target_rad_s(self, duty_frac):
        d = _finite(duty_frac, 'duty_frac')
        if abs(d) > 1.:
            raise ValueError('drive_duty_frac_out_of_range')
        return d * self.omega0_rad_s

    def torque_nm(self, duty_frac, omega_rad_s):
        """Motor torque on the line, clipped to +-tau_stall (joint friction not included)."""
        tau = self.tau_stall_nm * (_finite(duty_frac, 'duty_frac') - _finite(omega_rad_s, 'omega_rad_s')
                                   / self.omega0_rad_s)
        return max(-self.tau_stall_nm, min(self.tau_stall_nm, tau))

    def steady_speed_m_s(self, duty_frac):
        """Steady ground speed with the friction: r * omega0 * (d - friction / tau_stall), 0 inside the deadband."""
        d = abs(_finite(duty_frac, 'duty_frac'))
        omega = self.omega0_rad_s * (d - self.coulomb_friction_nm / self.tau_stall_nm)
        return math.copysign(max(0., omega) * self.wheel_radius_m, _finite(duty_frac, 'duty_frac'))

    def stability_number(self, inertia_kgm2, dt_s):
        """p_gain * dt / J; the force-mode loop needs it below 1 (design 3.6)."""
        return self.p_gain * _finite(dt_s, 'dt_s') / _finite(inertia_kgm2, 'inertia_kgm2')


def dc_motor_line(profile, curve='main', battery_scale=None, tau_stall_nm=None) -> DcMotorLine:
    """Derive the DC line from the curve's least-squares duty-speed line (design 3.6)."""
    name = resolve_curve(profile, curve)
    points = tuple(curve_fit_points(profile.value, name))
    fit = fit_duty_speed_line(points)
    radius = drive_wheel_radius(profile.value)
    scale = profile.value('battery_voltage_scale') if battery_scale is None else _finite(battery_scale,
                                                                                          'battery_scale')
    tau = profile.value('drive_tau_stall_nm') if tau_stall_nm is None else _finite(tau_stall_nm, 'tau_stall_nm')
    if scale <= 0. or tau <= 0.:
        raise ValueError('drive_dc_line_needs_positive_scale_and_torque')
    omega0 = fit['r_omega0_m_s'] / radius
    d0 = fit['deadband_pct'] / 100.
    return DcMotorLine(curve=name, fit_points=points, slope_m_s_per_pct=fit['slope_m_s_per_pct'], deadband_frac=d0,
                       r_omega0_m_s=fit['r_omega0_m_s'], wheel_radius_m=radius, omega0_nominal_rad_s=omega0,
                       tau_stall_nominal_nm=tau, battery_scale=scale, omega0_rad_s=scale * omega0,
                       tau_stall_nm=scale * tau, coulomb_friction_nm=d0 * tau)


def vehicle_mass_kg(profile):
    """Body mass plus 4 wheels (the 30 kg of the design's acceleration assumption)."""
    return profile.value('mass_kg') + 4. * profile.value('wheel_mass_kg')


def tau_stall_from_acceleration(vehicle_mass_kg_value, accel_m_s2, wheel_radius_m, driven_wheels=4):
    """Per-wheel stall torque that gives the acceleration from rest: m * a * r / n (placeholder rule)."""
    if isinstance(driven_wheels, bool) or not isinstance(driven_wheels, int) or driven_wheels <= 0:
        raise ValueError('drive_driven_wheels_must_be_positive_int')
    return (_finite(vehicle_mass_kg_value, 'vehicle_mass_kg') * _finite(accel_m_s2, 'accel_m_s2')
            * _finite(wheel_radius_m, 'wheel_radius_m') / driven_wheels)


def gsim_stability_number(profile, curve='main', tau_stall_nm=None):
    """p_gain * dt / J with J = wheel_inertia_kgm2 and dt = 1 / gz_physics_hz (about 0.13 by default)."""
    line = dc_motor_line(profile, curve, tau_stall_nm=tau_stall_nm)
    return line.stability_number(profile.value('wheel_inertia_kgm2'), 1. / profile.value('gz_physics_hz'))


# --- cross-check against field_response -----------------------------------------------------------------------
def _field_value(name):
    if isinstance(name, tuple):
        return FIELD_PARAMETERS[name[0]][0][name[1]]
    return FIELD_PARAMETERS[name][0]


def check_profile_against_field_response(profile):
    """Mismatch messages between profile copies and field_response.PARAMETERS (empty list = consistent)."""
    problems = []
    for name, key, field_name in FIELD_RESPONSE_COPIES:
        value = profile.value(name)
        if key is not None:
            value = value[key]
        expected = _field_value(field_name)
        if not math.isclose(float(value), float(expected), rel_tol=0., abs_tol=1e-12):
            problems.append('%s%s = %r, field_response %r = %r' % (name, '' if key is None else '.' + key, value,
                                                                   field_name, expected))
    ramp = mcu_ramp(profile)
    if not math.isclose(ramp.pct_per_s, FIELD_PARAMETERS['mcu_ramp_pct_per_s'][0], abs_tol=1e-9):
        problems.append('mcu_ramp.pwm_per_s gives %r %%/s, field_response mcu_ramp_pct_per_s %r'
                        % (ramp.pct_per_s, FIELD_PARAMETERS['mcu_ramp_pct_per_s'][0]))
    if not math.isclose(ramp.reversal_dwell_s, FIELD_PARAMETERS['mcu_reversal_wait_s'][0], abs_tol=1e-12):
        problems.append('mcu_ramp.reversal_dwell_ms gives %r s, field_response mcu_reversal_wait_s %r'
                        % (ramp.reversal_dwell_s, FIELD_PARAMETERS['mcu_reversal_wait_s'][0]))
    fit = profile.value('drive_curve_P_fit1003')
    if not math.isclose(fit['speed_at_90_m_s'], P_FIT1003_CURVE_OVERRIDES['speed_at_90_m_s'], abs_tol=1e-12) or \
            not math.isclose(fit['deadband_pct'], P_FIT1003_CURVE_OVERRIDES['forward_deadband_pct'], abs_tol=1e-12):
        problems.append('drive_curve_P_fit1003 differs from the PREREG P_fit1003 overrides %r'
                        % (P_FIT1003_CURVE_OVERRIDES,))
    return problems
