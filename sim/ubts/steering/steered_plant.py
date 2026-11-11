"""Steered fast-SIM plant (F-SIM SteeredPlant; design 2.1, 3.1-3.7 and 8.3 P1).

A front-wheel steered vehicle for the closed-loop harness (closed_loop/harness.py). It
replaces the differential FieldPlant through the harness hooks plant_factory and
actuation; it never takes left/right duty commands (command() raises).

Model (all values from one vehicle profile, steering/vehicle_profile.py; placeholders):
  drive      throttle (signed duty %, left = right) -> MCU ramp 100 %/s with the 120 ms wait
             after the duty reaches 0 (field_response.McuRamp rules, no V2 watchdog; the V3
             TTL and watchdog belong to the MCU logic, P3) -> steady speed from the drive
             curve (drive.py: main P_fit1003 by default, bracket field1010 selectable;
             reverse = forward x reverse_curve; battery scale on the duty) x per-run speed
             scale x (1 + speed fluctuation) x (1 - scrub loss per deg x |delta|).
             Speeding up: first-order lag drive_accel_tau_s. Slowing down: one event per
             slow-down that holds the speed for onset_s and then decelerates at a per-event
             draw median x exp(log_sd x z). The event takes the stop mode: release for a
             throttle fade (key release, DISARM; FieldPlant structure, the deceleration limit
             acts under the drive_decel_tau_s lag), brake for stop('brake') (duty 0 at once,
             EN on) and coast for stop('coast') (EN off); brake and coast decelerate at a
             constant rate after the onset, so their distance is v * onset + v^2 / (2 a)
             (design 3.5 table, drive.StopMode) up to the step size. The onset counts from
             the step that detects the slow-down or applies the stop.
             Root snag: forward_blocked stops forward motion at once.
  steering   a SteeringBackend (backend_api.py) stepped once per plant step. By default the
             backend is the one the profile's steer_actuator_level asks for (default_backend():
             L0 -> l0_servo.py, L1 -> the native lockstep backend, design 3.3 default L1;
             geometry_check sets L0, design 3.3 "L0 for A1, A12"). A backend of another level
             than the profile gives a SteeringBackendLevelWarning and result_label() records both
             levels. test_servo mode: the plant gives the target equivalent angle (command
             clamped to the equivalent soft limit, A21); direct_pwm mode: the plant gives the
             steering PWM. The backend returns the true wheel angles; the plant uses the
             wheel-side equivalent angle delta_eq.
  body       kinematic bicycle at the rear axle; the base (footprint centre) pose is the
             state, the rear axle sits at rear_axle_x_in_base_m:
               kappa = eta * tan(delta_eq) / (L + K_us * v^2) + kappa_wander
               omega = v * kappa;   base lateral velocity = slip - (omega + omega_ext) * x_rear
             eta (steering efficiency), K_us (understeer gradient, s^2/m, so K_us * v^2 is a
             length) and the scrub loss are per-run draws; kappa_wander is the 10/10 OU
             curvature process in distance (curvature_wander; unlike FieldPlant it also acts
             while slowing down, since omega = v * kappa); slip is the lateral slip OU
             (correlation 0.5 s, sd scaled by |v| / v100) as in FieldPlant. Integration is
             FieldPlant's (mid-step heading, icr_x = rear axle).
  IMU        field_response error model (per-run gyro scale and bias, still noise, vibration
             coupling, drive vibration by duty, per-run still vibration) plus the steering
             motor vibration term imu_vibration_steer_m_s2 x |steer PWM| added in quadrature
             (placeholder 0), then the mount rotation imu_mount_rpy_deg (identity by default).
             The accelerometer model is FieldPlant's (dv/dt, v * w, g); lever arms are ignored.

geometry_check=True (A1 and A12 only): the profile view gets the ideal values (Ackermann
ratio 1, steering efficiency 1, scrub loss 0, understeer 0, zero offset 0, calibration
residual 0, left/right asymmetry 0, backlash 0, actuator level L0) and the plant has no
process noise (speed scale 1, no speed fluctuation, no curvature wander, no lateral slip,
median stop deceleration). The IMU keeps its sensor errors (they do not change the trajectory).

Randomness: one SeedSequence per run (seed = int or tuple of non-negative ints, as the harness
passes it) spawned into independent streams for run constants, process noise, sensor noise
and stop events (the k-th slow-down event takes the k-th stop draw), plus one u64 draw seed.
The draw seed gives the per-run profile draws (profile.sample: eta, scrub loss, understeer,
zero offset, calibration residual) and configures the backend, so the plant and the backend
use the same per-run draw set.

Truth isolation: x, y, yaw, v, w, distance_m and truth() are evaluation values (the harness
uses them for metrics and sensor synthesis). An executor or estimator may use only
imu_sample(), the LiDAR synthesised from lidar_pose() by the harness, and steer_status().
steer_status() carries what an MCU knows (raw ADC, commands, EN, MCU faults); the backend's
truth outputs (backend_api.TRUTH_OUTPUT_FIELDS: wheel angles, motor-side angle, the driver's
actual output applied_steer_pwm, model flags) are in truth() only. The hold_measured stop target is
the MCU-side filtered potentiometer measurement (measured_hold_rad(): median of the last
pot_filter.median_of plant-step ADC samples, nominal decode), not the truth-side servo view
(decision R13-2); the start hold before the first command is still the servo view.

Not modelled here: MCU logic (V3 protocol, TTL, watchdog, faults, latches; P3), tyre forces,
rolling-scan distortion, terrain. Duty is a command, not a wheel speed. Every result from
the placeholder profile is a placeholder-based SIM result, not field validation.
"""
from __future__ import annotations

import collections
from dataclasses import dataclass, field, replace
import math
from typing import Tuple
import warnings

import numpy as np

from field_response import FieldResponseParams, forward_curve

from . import backend_api as ba
from . import drive
from . import geometry as geo
from .l0_servo import L0ServoBackend, per_run_value
from .vehicle_profile import load_profile

PLANT_MODEL = 'ubts.steered_plant.v1'
SURFACES = ('normal', 'stall_prone')          # FieldPlant surfaces; no effect on this plant (no pivots)
STEER_MODES = ('test_servo', 'direct_pwm')    # firmware mode is P3
STOP_MODES = drive.STOP_MODE_NAMES            # ('release', 'brake', 'coast')
STEER_STOP_POLICIES = ('hold_measured', 'hold_target', 'center', 'off')
PER_RUN_NAMES = ('steering_efficiency', 'scrub_speed_loss_per_deg', 'understeer_gradient', 'steer_zero_offset_deg',
                 'pot_cal_residual_deg')
STANDSTILL_SPEED_M_S = .01      # SIM_ASSUMED: |v| below this -> the backend uses the standstill steering rate
LATERAL_SLIP_TAU_S = .5         # FieldPlant.step lateral slip OU correlation time
SLOWING_THRESHOLD_M_S = .03     # FieldPlant.step: a slow-down event starts when the target is 0.03 below |v|
SPEED_SNAP_M_S = .005           # FieldPlant.step: v -> 0 when the target is 0 and |v| < 0.005
VIBRATION_SPEED_M_S = .02       # FieldPlant.step: drive vibration above this speed
SPEED_RUN_SCALE_CLIP = (.5, 1.25)   # FieldPlant per-run speed scale clip
GRAVITY_M_S2 = 9.81
# Ideal profile values for geometry checks (A1, A12; design 3.3 uses the L0 ideal servo there).
# Distributions with zero spread keep the per-run draw form.
GEOMETRY_CHECK_OVERRIDES = {
    'steer_actuator_level': 'L0',
    'steer_ackermann_ratio': 1.0,
    'steering_efficiency': {'dist': 'uniform', 'low': 1.0, 'high': 1.0},
    'scrub_speed_loss_per_deg': {'dist': 'uniform', 'low': 0.0, 'high': 0.0},
    'understeer_gradient': {'dist': 'uniform', 'low': 0.0, 'high': 0.0},
    'steer_zero_offset_deg': {'dist': 'normal', 'mean': 0.0, 'sd': 0.0},
    'pot_cal_residual_deg': {'dist': 'normal', 'mean': 0.0, 'sd': 0.0},
    'steer_lr_asymmetry': 0.0,
    'steer_backlash_deg': 0.0,
    'steer_backlash_wander': {'sd_deg': 0.0, 'corr_length_m': 5.0},
}
GEOMETRY_CHECK_LABEL = 'geometry_check'


class SteeringBackendLevelWarning(UserWarning):
    """The steering backend models another actuator level than the profile's steer_actuator_level."""


def default_backend(params):
    """A new unconfigured backend of the level the profile asks for (steer_actuator_level, design 3.3).

    L0 -> l0_servo.L0ServoBackend (test_servo only); L1 -> steering.actuator_link.NativeSteeringBackend
    (built on demand: compiler UBTS_STEER_CXX or g++ on PATH, output UBTS_STEER_BUILD_DIR or the git-ignored
    native/_build/); L1b -> ValueError (not implemented, design 3.3 and Q11).
    """
    level = params.profile.value('steer_actuator_level')
    if level == 'L0':
        return L0ServoBackend(mode=params.steer_mode)
    if level == 'L1':
        from .actuator_link import NativeSteeringBackend   # subprocess backend; imported only when needed
        return NativeSteeringBackend(mode=params.steer_mode)
    raise ValueError('steered_plant_actuator_level_not_available:%r (L1b is not implemented; design 3.3, Q11)'
                     % (level,))


def backend_level(backend):
    """Actuator level a backend models: its LEVEL attribute, else the level of its startup config, else None."""
    level = getattr(backend, 'LEVEL', None)
    if level is None:
        config = getattr(backend, 'config', None)
        if isinstance(config, dict) and isinstance(config.get('actuator'), dict):
            level = config['actuator'].get('level')
    return level


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)):
        raise ValueError('steered_plant_value_not_a_number:' + name)
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('steered_plant_value_not_finite:' + name)
    return value


def _seed_sequence(seed):
    """int, tuple/list of non-negative ints or a SeedSequence -> numpy SeedSequence (FieldPlant rule)."""
    if isinstance(seed, np.random.SeedSequence):
        return seed
    entropy = [seed] if isinstance(seed, (int, np.integer)) else list(seed)
    if not entropy or any(isinstance(v, bool) or not isinstance(v, (int, np.integer)) or v < 0 for v in entropy):
        raise ValueError('invalid_plant_seed')
    return np.random.SeedSequence([int(v) for v in entropy])


class _NormalBlock:
    """Standard normals drawn in blocks from ONE dedicated generator (same pattern as FieldPlant)."""

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


def _mount_matrix(roll_deg, pitch_deg, yaw_deg, yaw_sign):
    """Body -> sensor rotation (R^T) for sensor axes R = Rz(yaw) Ry(pitch) Rx(roll) [Rx(pi) if yaw_sign = -1].

    yaw_sign -1 means the sensor is mounted upside down (its z axis points down), so it reads
    -omega about z and -g on z.
    """
    r, p, y = (math.radians(a) for a in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rz = ((cy, -sy, 0.), (sy, cy, 0.), (0., 0., 1.))
    ry = ((cp, 0., sp), (0., 1., 0.), (-sp, 0., cp))
    rx = ((1., 0., 0.), (0., cr, -sr), (0., sr, cr))

    def mul(a, b):
        return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))
    rot = mul(mul(rz, ry), rx)
    if yaw_sign == -1:
        rot = mul(rot, ((1., 0., 0.), (0., -1., 0.), (0., 0., -1.)))
    elif yaw_sign != 1:
        raise ValueError('imu_yaw_sign_must_be_plus_or_minus_one')
    return tuple(tuple(rot[j][i] for j in range(3)) for i in range(3))   # transpose: body -> sensor


@dataclass(frozen=True)
class SteeredPlantParams:
    """Plant constants read once from a vehicle profile (use from_profile())."""

    profile: object                 # vehicle_profile.VehicleProfile (the ideal view when geometry_check)
    geometry: geo.SteeringGeometry
    curve: str                      # drive curve name ('P_fit1003' or 'field1010')
    curve_role: str                 # what the caller asked for ('main', 'bracket' or a curve name)
    curve_params: FieldResponseParams   # carries this curve's speed_at_90/100 and deadband for forward_curve
    reverse_curve: float
    battery_voltage_scale: float
    ramp_pct_per_s: float
    reversal_dwell_s: float
    stop_release: drive.StopMode
    stop_brake: drive.StopMode
    stop_coast: drive.StopMode
    accel_tau_s: float
    decel_tau_s: float
    speed_run_scale_mode: float
    speed_run_scale_sd_low: float
    speed_run_scale_sd_high: float
    speed_fluctuation_sd: float
    speed_fluctuation_tau_s: float
    lateral_slip_sd_m_s: float
    wander_mean_deg_per_m: float
    wander_sd_deg_per_m: float
    wander_corr_length_m: float
    gyro_scale_mean: float
    gyro_scale_sd: float
    gyro_bias_sd_deg_s: float
    gyro_still_noise_sd_deg_s: float
    gyro_vibration_deg_s_per_m_s2: float
    drive_vibration_at_90_m_s2: float
    drive_vibration_at_100_m_s2: float
    drive_vibration_below_90_m_s2: float
    still_vibration_rms_m_s2: Tuple[float, float]
    steer_vibration_m_s2: float
    imu_mount_rpy_deg: Tuple[float, float, float]
    imu_yaw_sign: int
    lidar_x_in_base_m: float
    lidar_y_in_base_m: float
    footprint_length_m: float
    footprint_width_m: float
    pot_supply_sag_per_pwm: float
    pot_center_raw: float
    pot_counts_per_rad: float
    pot_location: str
    steer_lr_asymmetry: float
    drive_wheels: str
    drive_lr_mode: str
    stop_policy_steer: str
    geometry_check: bool = False
    steer_mode: str = 'test_servo'
    gyro_noise_scale: float = 1.
    pot_median_of: int = 5          # pot_filter.median_of: the MCU median filter length (hold_measured target)
    imu_rotation: Tuple[Tuple[float, float, float], ...] = field(init=False, compare=False, repr=False,
                                                                 default=((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)))
    soft_limits_eq_rad: Tuple[float, float] = field(init=False, compare=False, repr=False, default=(0., 0.))

    def __post_init__(self):
        if self.steer_mode not in STEER_MODES:
            raise ValueError('steered_plant_steer_mode_unknown:%r (P1 supports %s)' % (self.steer_mode, STEER_MODES))
        for name in ('accel_tau_s', 'decel_tau_s', 'speed_fluctuation_tau_s', 'wander_corr_length_m', 'ramp_pct_per_s',
                     'battery_voltage_scale', 'reverse_curve', 'pot_counts_per_rad'):
            if not getattr(self, name) > 0.:
                raise ValueError('steered_plant_value_not_positive:' + name)
        if self.reversal_dwell_s < 0. or self.gyro_noise_scale < 0.:
            raise ValueError('steered_plant_value_negative')
        if self.stop_policy_steer not in STEER_STOP_POLICIES:
            raise ValueError('steered_plant_stop_policy_unknown:%r' % (self.stop_policy_steer,))
        if isinstance(self.pot_median_of, bool) or not isinstance(self.pot_median_of, int) or not (
                1 <= self.pot_median_of <= 9):
            raise ValueError('steered_plant_pot_median_of_must_be_1_to_9')
        roll, pitch, yaw = self.imu_mount_rpy_deg
        object.__setattr__(self, 'imu_rotation', _mount_matrix(roll, pitch, yaw, self.imu_yaw_sign))
        # (left, right) equivalent soft limits, computed once (geometry.equivalent_limits bisects)
        object.__setattr__(self, 'soft_limits_eq_rad', tuple(self.geometry.soft_limits_eq()))

    def clamp_steer_target(self, delta_rad):
        """Clamp an equivalent-angle target to the equivalent soft limits (A21; same as geometry.clamp_to_soft)."""
        left, right = self.soft_limits_eq_rad
        return min(left, max(-right, delta_rad))

    @classmethod
    def from_profile(cls, profile=None, *, curve='main', geometry_check=False, steer_mode='test_servo',
                     gyro_noise_scale=1.):
        """Plant constants from a VehicleProfile (None: the shipped placeholder profile).

        curve: 'main' (P_fit1003, default), 'bracket' (field1010) or a curve name.
        geometry_check: ideal geometry values and no process noise (A1, A12 only).
        steer_mode: 'test_servo' (target angle, default) or 'direct_pwm' (steering PWM).
        gyro_noise_scale: multiplies the gyro still noise and vibration coupling (harness knob).
        """
        profile = load_profile() if profile is None else profile
        if geometry_check:
            profile = profile.with_overrides(GEOMETRY_CHECK_OVERRIDES, label=GEOMETRY_CHECK_LABEL)
        v = profile.value
        name = drive.resolve_curve(profile, curve)
        ramp = drive.mcu_ramp(profile)
        modes = drive.stop_modes(profile)
        run_scale, wander, gyro = v('speed_run_scale'), v('curvature_wander'), v('imu_gyro_scale')
        still, mount = v('imu_vibration_still_m_s2'), v('imu_mount_rpy_deg')
        scale = _number(gyro_noise_scale, 'gyro_noise_scale')
        return cls(
            profile=profile, geometry=geo.SteeringGeometry.from_profile(profile), curve=name, curve_role=curve,
            curve_params=drive.curve_params(profile, name), reverse_curve=float(v('reverse_curve')),
            battery_voltage_scale=float(v('battery_voltage_scale')), ramp_pct_per_s=ramp.pct_per_s,
            reversal_dwell_s=ramp.reversal_dwell_s, stop_release=modes['release'], stop_brake=modes['brake'],
            stop_coast=modes['coast'], accel_tau_s=float(v('drive_accel_tau_s')),
            decel_tau_s=float(v('drive_decel_tau_s')), speed_run_scale_mode=float(run_scale['mode']),
            speed_run_scale_sd_low=float(run_scale['sd_low']), speed_run_scale_sd_high=float(run_scale['sd_high']),
            speed_fluctuation_sd=float(v('speed_fluctuation_sd')),
            speed_fluctuation_tau_s=float(v('speed_fluctuation_tau_s')),
            lateral_slip_sd_m_s=float(v('lateral_slip_sd_m_s')),
            wander_mean_deg_per_m=float(wander['mean_deg_per_m']), wander_sd_deg_per_m=float(wander['sd_deg_per_m']),
            wander_corr_length_m=float(wander['corr_length_m']), gyro_scale_mean=float(gyro['mean']),
            gyro_scale_sd=float(gyro['sd']), gyro_bias_sd_deg_s=float(v('imu_gyro_bias_sd_deg_s')),
            gyro_still_noise_sd_deg_s=float(v('imu_gyro_still_noise_sd_deg_s')) * scale,
            gyro_vibration_deg_s_per_m_s2=float(v('imu_gyro_vibration_deg_s_per_m_s2')) * scale,
            drive_vibration_at_90_m_s2=float(v('imu_vibration_drive_at_90_m_s2')),
            drive_vibration_at_100_m_s2=float(v('imu_vibration_drive_at_100_m_s2')),
            drive_vibration_below_90_m_s2=float(v('imu_vibration_drive_below_90_m_s2')),
            still_vibration_rms_m_s2=(float(still['low']), float(still['high'])),
            steer_vibration_m_s2=float(v('imu_vibration_steer_m_s2')),
            imu_mount_rpy_deg=(float(mount['roll']), float(mount['pitch']), float(mount['yaw'])),
            imu_yaw_sign=int(mount['yaw_sign']), lidar_x_in_base_m=float(v('lidar_x_in_base_m')),
            lidar_y_in_base_m=float(v('lidar_y_in_base_m')), footprint_length_m=float(v('footprint_length_m')),
            footprint_width_m=float(v('footprint_width_m')), pot_supply_sag_per_pwm=float(v('pot_supply_sag_per_pwm')),
            pot_center_raw=float(v('pot_center_raw')),
            pot_counts_per_rad=2. ** v('adc_bits') * v('pot_to_steer_ratio') / math.radians(v('pot_range_deg')),
            pot_location=v('pot_location'), steer_lr_asymmetry=float(v('steer_lr_asymmetry')),
            drive_wheels=v('drive_wheels'), drive_lr_mode=v('drive_lr_mode'),
            stop_policy_steer=v('stop_policy_steer'), geometry_check=bool(geometry_check), steer_mode=steer_mode,
            gyro_noise_scale=scale, pot_median_of=int(v('pot_filter')['median_of']))

    def with_gyro_noise_scale(self, scale):
        """Copy with the gyro still noise and vibration coupling multiplied by scale (harness gyro_noise_scale)."""
        scale = _number(scale, 'gyro_noise_scale')
        return replace(self, gyro_still_noise_sd_deg_s=self.gyro_still_noise_sd_deg_s * scale,
                       gyro_vibration_deg_s_per_m_s2=self.gyro_vibration_deg_s_per_m_s2 * scale,
                       gyro_noise_scale=self.gyro_noise_scale * scale)

    @property
    def wheelbase_m(self):
        return self.geometry.wheelbase_m

    @property
    def rear_axle_x_in_base_m(self):
        return self.geometry.rear_axle_x_in_base_m

    @property
    def speed_at_100_m_s(self):
        return self.curve_params.speed_at_100_m_s

    def stop_mode(self, name):
        if name not in STOP_MODES:
            raise ValueError('unknown_stop_mode:%r' % (name,))
        return getattr(self, 'stop_' + name)

    def drive_vibration(self, duty_pct):
        """Vibration RMS while driving: below 90 % one value, 90..100 % linear (field_response rule)."""
        duty = abs(duty_pct)
        if duty < 90.:
            return self.drive_vibration_below_90_m_s2
        a, b = self.drive_vibration_at_90_m_s2, self.drive_vibration_at_100_m_s2
        return a + (min(duty, 100.) - 90.) * (b - a) / 10.


class _DriveRamp:
    """One MCU drive channel (equal duty left = right): field_response.McuRamp.step rules, no V2 watchdog."""

    def __init__(self, pct_per_s, dwell_s):
        self.pct_per_s = pct_per_s
        self.dwell_s = dwell_s
        self.applied = 0.
        self.target = 0.
        self.hold_until = -math.inf

    def command(self, duty_pct):
        self.target = float(max(-100., min(100., duty_pct)))

    def hard_stop(self, now_s):
        self.target = 0.
        if self.applied != 0.:
            self.hold_until = now_s + self.dwell_s
        self.applied = 0.

    def step(self, now_s, dt):
        if dt <= 0:
            return self.applied
        change = self.pct_per_s * dt
        applied, target = self.applied, self.target
        if applied != 0. and target * applied < 0:
            new = math.copysign(max(0., abs(applied) - change), applied)   # fade through zero first
        elif applied == 0. and target != 0. and now_s < self.hold_until:
            new = 0.                                                         # wait after any duty reached 0
        else:
            new = applied + max(-change, min(change, target - applied))
        if applied != 0. and new == 0.:
            self.hold_until = now_s + self.dwell_s
        self.applied = new
        return new


class SteeredPlant:
    """Front-wheel steered vehicle for the F-SIM harness (see the module docstring).

    SteeredPlant(params=None, *, surface='normal', seed=0, pose=(0, 0, 0), backend=None, initial_steer_rad=0.)
      params            SteeredPlantParams (None: from the placeholder profile, main curve)
      surface           'normal' | 'stall_prone' (kept for the harness; no effect, there are no pivots)
      seed              int or tuple of non-negative ints (the harness passes (seed, crc32(key), purpose))
      pose              (x, y, yaw) of the base (footprint centre), m and rad
      backend           an unconfigured SteeringBackend whose mode equals params.steer_mode
                        (None: default_backend(params), the level of the profile's steer_actuator_level);
                        the plant configures and resets it and owns it (close() releases a native one)
      initial_steer_rad true wheel-side equivalent angle at the start (e.g. wheels turned 10 deg)
    A backend whose level differs from the profile gives a SteeringBackendLevelWarning; result_label()
    records the backend level next to the profile level.
    """

    def __init__(self, params=None, *, surface='normal', seed=0, pose=(0., 0., 0.), backend=None,
                 initial_steer_rad=0.):
        if params is None:
            params = SteeredPlantParams.from_profile()
        if not isinstance(params, SteeredPlantParams):
            raise TypeError('SteeredPlant needs SteeredPlantParams (SteeredPlantParams.from_profile); %s is for '
                            'another plant' % type(params).__name__)
        if surface not in SURFACES:
            raise ValueError('unknown_surface')
        p = params
        self.params = params
        self.surface = surface
        self.seed = seed
        run_ss, process_ss, sensor_ss, stop_ss, draw_ss = _seed_sequence(seed).spawn(5)
        self.draw_seed = int(draw_ss.generate_state(1, dtype=np.uint64)[0])
        self.rng = np.random.default_rng(run_ss)
        self.normal = _NormalBlock(np.random.default_rng(process_ss))
        self._sensor_normal = _NormalBlock(np.random.default_rng(sensor_ss))
        self._stop_normal = _NormalBlock(np.random.default_rng(stop_ss))
        # per-run profile draws (the backend uses the same draw seed, so the zero offset is the same value)
        self.draws = {name: float(per_run_value(p.profile, self.draw_seed, name)) for name in PER_RUN_NAMES}
        self.zero_offset_rad = math.radians(self.draws['steer_zero_offset_deg'])
        self.steering_efficiency = self.draws['steering_efficiency']
        self.scrub_speed_loss_per_deg = self.draws['scrub_speed_loss_per_deg']
        self.understeer_gradient = self.draws['understeer_gradient']
        # run constants (FieldPlant draw order on the run stream)
        z = float(self.rng.normal())
        spread = p.speed_run_scale_sd_low if z < 0 else p.speed_run_scale_sd_high
        self.speed_run_scale = float(np.clip(p.speed_run_scale_mode + spread * z, *SPEED_RUN_SCALE_CLIP))
        self.wander_deg_per_m = float(self.rng.normal(p.wander_mean_deg_per_m, p.wander_sd_deg_per_m))
        self.gyro_scale = float(self.rng.normal(p.gyro_scale_mean, p.gyro_scale_sd))
        self.gyro_bias = math.radians(float(self.rng.normal(0., p.gyro_bias_sd_deg_s)))
        self.still_vibration = float(math.exp(self.rng.uniform(*np.log(p.still_vibration_rms_m_s2))))
        if p.geometry_check:
            self.speed_run_scale = 1.
            self.wander_deg_per_m = 0.
        # state (the harness reads these names)
        self.time_s = 0.
        self.x, self.y, self.yaw = (float(value) for value in pose)
        self.v = 0.
        self.w = 0.
        self.lateral = 0.
        self.distance_m = 0.
        self.accel = 0.
        self.speed_fluctuation = 0.
        self.curvature = 0.
        self.steer_curvature = 0.
        self.external_yaw_rate = 0.     # SIM disturbance (skid on a root): body yaw rate that the gyro senses
        self.forward_blocked = False    # SIM disturbance: forward motion blocked (root below the scan plane)
        self.pivot_count = 0
        self.pivot_stalls = 0
        self.pivot_log = []
        self.regime = 'still'
        self.vibration_rms = self.still_vibration
        self.decelerating = False
        self._decel_event = None
        self._decel_events = 0
        self._stop_request = None
        self.stop_mode = None
        # drive channel
        self._ramp = _DriveRamp(p.ramp_pct_per_s, p.reversal_dwell_s)
        self.drive_en = True
        self.throttle_request_pct = 0.
        # steering backend
        self.backend = default_backend(p) if backend is None else backend
        missing = [name for name in ('configure', 'reset', 'step', 'close') if not callable(getattr(self.backend, name,
                                                                                                    None))]
        if missing:
            raise TypeError('steered_plant_backend_lacks:%s (backend_api.SteeringBackend contract)' % ','.join(missing))
        self.backend.configure(p.profile, self.draw_seed)
        self.profile_actuator_level = p.profile.value('steer_actuator_level')
        self.backend_level = backend_level(self.backend)
        if self.backend_level is not None and self.backend_level != self.profile_actuator_level:
            warnings.warn(SteeringBackendLevelWarning(
                'steering backend %s models level %s but the profile steer_actuator_level is %s; results are %s-based '
                '(result_label() records both)' % (type(self.backend).__name__, self.backend_level,
                                                   self.profile_actuator_level, self.backend_level)), stacklevel=2)
        self._steer = self.backend.reset(_number(initial_steer_rad, 'initial_steer_rad'))
        # the potentiometer samples the plant has (one per step) for the MCU-side filtered measurement
        self._pot_samples = collections.deque([self._steer.pot_raw], maxlen=p.pot_median_of)
        # hold the start angle until the first steering command (the servo target is the angle it sees now)
        self.steer_target_rad = self.servo_view_rad()
        self.steer_request_mrad = None
        self.steer_target_clamped = False
        self.steer_en = True
        self.steer_pwm_command = 0.
        self.command_time_s = None

    @classmethod
    def from_profile(cls, profile=None, *, curve='main', geometry_check=False, steer_mode='test_servo',
                     gyro_noise_scale=1., surface='normal', seed=0, pose=(0., 0., 0.), backend=None,
                     initial_steer_rad=0.):
        """SteeredPlant(SteeredPlantParams.from_profile(profile, ...), surface=..., seed=..., ...)."""
        params = SteeredPlantParams.from_profile(profile, curve=curve, geometry_check=geometry_check,
                                                 steer_mode=steer_mode, gyro_noise_scale=gyro_noise_scale)
        return cls(params, surface=surface, seed=seed, pose=pose, backend=backend,
                   initial_steer_rad=initial_steer_rad)

    # --- harness interface -------------------------------------------------------------------------------------
    def command(self, *args):
        raise RuntimeError('SteeredPlant takes no differential command %r: use the steering actuation hook '
                           '(SteeringActuation -> command_actuation(throttle_pct, steer_target_mrad))' % (args,))

    def command_actuation(self, throttle_pct, steer_target_mrad):
        """Throttle (signed duty %, -100..100, clipped like the MCU) and steering target (mrad, + left).

        The steering target is clamped to the equivalent soft limit (A21). Re-enables the
        drive after stop('coast'). Needs steer_mode 'test_servo'.
        """
        throttle = _number(throttle_pct, 'throttle_pct')
        steer_mrad = _number(steer_target_mrad, 'steer_target_mrad')
        if self.params.steer_mode != 'test_servo':
            raise RuntimeError('command_actuation needs steer_mode test_servo; in direct_pwm mode use command_drive() '
                               'and command_steer_pwm()')
        self._command_drive(throttle)
        target = steer_mrad / 1000.
        clamped = self.params.clamp_steer_target(target)
        self.steer_request_mrad = steer_mrad
        self.steer_target_rad = clamped
        self.steer_target_clamped = clamped != target
        self.steer_en = True

    def command_drive(self, throttle_pct):
        """Throttle only (signed duty %); keeps the steering command."""
        self._command_drive(_number(throttle_pct, 'throttle_pct'))

    def command_steer_pwm(self, pwm, steer_en=True):
        """direct_pwm mode: signed steering driver PWM in [-1, 1] and the steering enable."""
        if self.params.steer_mode != 'direct_pwm':
            raise RuntimeError('command_steer_pwm needs steer_mode direct_pwm')
        value = _number(pwm, 'pwm')
        if abs(value) > 1. or not isinstance(steer_en, bool):
            raise ValueError('steer_pwm_out_of_range_or_enable_not_bool')
        self.steer_pwm_command = value
        self.steer_en = steer_en

    def stop(self, mode='brake', steer=None):
        """Drive stop mode ('release' fade, 'brake' duty 0 at once with EN on, 'coast' EN off).

        steer: 'hold_measured' (servo target = the MCU-side filtered potentiometer measurement now,
        measured_hold_rad(); decision R13-2, not the truth-side servo view), 'hold_target', 'center',
        'off' (steering EN off), or None for the profile stop_policy_steer. Not a latch: a
        later command re-applies (the MCU latch is P3).
        """
        if mode not in STOP_MODES:
            raise ValueError('unknown_stop_mode:%r' % (mode,))
        policy = self.params.stop_policy_steer if steer is None else steer
        if policy not in STEER_STOP_POLICIES:
            raise ValueError('unknown_steer_stop_policy:%r' % (policy,))
        self.throttle_request_pct = 0.
        if mode == 'release':
            self._ramp.command(0.)
        else:
            self._ramp.hard_stop(self.time_s)
            self._stop_request = mode
            self.drive_en = mode != 'coast'
        self.stop_mode = mode
        self.steer_pwm_command = 0.     # direct_pwm: no servo, the steering output stops in every policy
        if policy == 'off':
            self.steer_en = False
        elif policy == 'center':
            self.steer_target_rad = 0.
        elif policy == 'hold_measured':
            self.steer_target_rad = self.params.clamp_steer_target(self.measured_hold_rad())

    def hard_stop(self):
        """FieldPlant name: duty 0 at once (brake)."""
        self.stop('brake')

    def step(self, dt):
        if isinstance(dt, bool) or not isinstance(dt, (int, float)) or not 0 < dt <= .05:
            raise ValueError('plant_step_out_of_range')
        dt = float(dt)
        p = self.params
        self.time_s += dt
        duty = self._ramp.step(self.time_s, dt) if self.drive_en else 0.
        if not p.geometry_check:
            a = math.exp(-dt / p.speed_fluctuation_tau_s)
            self.speed_fluctuation = (a * self.speed_fluctuation
                                      + math.sqrt(1 - a * a) * p.speed_fluctuation_sd * self.normal())
        # steering: one backend step with the speed at the start of the step
        supply = 1. if p.geometry_check else 1. - p.pot_supply_sag_per_pwm * abs(duty) / 100.
        test_servo = p.steer_mode == 'test_servo'
        self._steer = self.backend.step(ba.SteerStepInput(
            dt_s=dt, mode=p.steer_mode, steer_pwm=0. if test_servo else self.steer_pwm_command,
            steer_en=self.steer_en, target_delta_eq_rad=self.steer_target_rad if test_servo else 0.,
            vehicle_speed_m_s=self.v, vehicle_stopped=abs(self.v) < STANDSTILL_SPEED_M_S,
            supply_scale=max(1e-6, supply)))
        self._pot_samples.append(self._steer.pot_raw)
        delta = self._steer.delta_eq_rad
        # drive speed
        v_target = self._speed_target(duty, delta)
        blocked = self.forward_blocked and (v_target > 0. or self.v > 0.)
        if blocked:
            v_target = 0.
        if not p.geometry_check:
            self._advance_wander(abs(self.v) * dt)
        slowing = abs(v_target) < abs(self.v) - SLOWING_THRESHOLD_M_S or v_target * self.v < 0
        if self._stop_request is not None:
            mode, self._stop_request = self._stop_request, None
            if self.v != 0.:
                self._start_decel_event(mode)       # brake / coast replace any running slow-down event
                self.decelerating = True
        elif slowing and not self.decelerating:
            self._start_decel_event('release')
        self.decelerating = slowing or (self.decelerating and abs(v_target) < abs(self.v) - 1e-6)
        previous_v = self.v
        if blocked:
            self.v = min(self.v, 0.)          # SIM disturbance: abrupt stop against the obstacle
        elif self.decelerating:
            event = self._decel_event
            if self.time_s - event['since'] >= event['onset_s'] - 1e-9:
                gap = abs(self.v - v_target)
                if event['lag']:              # release: FieldPlant fit, deceleration limit under a short lag
                    change = min(gap * (1 - math.exp(-dt / p.decel_tau_s)), event['decel_m_s2'] * dt)
                else:                         # brake / coast: constant deceleration (design 3.5 table)
                    change = min(gap, event['decel_m_s2'] * dt)
                self.v -= math.copysign(change, self.v - v_target)
        else:
            self.v += (v_target - self.v) * (1 - math.exp(-dt / p.accel_tau_s))
        if v_target == 0. and abs(self.v) < SPEED_SNAP_M_S:
            self.v = 0.
        self.accel = (self.v - previous_v) / dt
        # kinematic bicycle at the rear axle
        length = p.geometry.wheelbase_m + self.understeer_gradient * self.v * self.v
        self.steer_curvature = self.steering_efficiency * math.tan(delta) / length
        self.curvature = self.steer_curvature + math.radians(self.wander_deg_per_m)
        self.w = self.v * self.curvature
        if not p.geometry_check:
            c = math.exp(-dt / LATERAL_SLIP_TAU_S)
            slip_sd = p.lateral_slip_sd_m_s * min(1., abs(self.v) / p.speed_at_100_m_s)
            self.lateral = c * self.lateral + math.sqrt(1 - c * c) * slip_sd * self.normal()
        w = self.w + self.external_yaw_rate
        lateral = self.lateral - w * p.geometry.rear_axle_x_in_base_m
        mid = self.yaw + .5 * w * dt
        self.x += (self.v * math.cos(mid) - lateral * math.sin(mid)) * dt
        self.y += (self.v * math.sin(mid) + lateral * math.cos(mid)) * dt
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
        self.distance_m += math.hypot(self.v, lateral) * dt
        # regime and vibration
        if blocked:
            self.regime = 'blocked'
        elif abs(v_target) > 1e-6:
            self.regime = 'drive'
        elif self.v != 0.:
            self.regime = 'stopping'
        else:
            self.regime = 'still'
        if self.regime == 'drive' and abs(self.v) > VIBRATION_SPEED_M_S:
            self.vibration_rms = p.drive_vibration(duty)
        else:
            self.vibration_rms = self.still_vibration

    def imu_vibration_rms(self):
        """IMU vibration RMS: drive or still vibration and the steering-motor term in quadrature (m/s^2)."""
        return math.hypot(self.vibration_rms, self.params.steer_vibration_m_s2 * abs(self._steer.applied_steer_pwm))

    def imu_sample(self):
        """Raw-IMU-like sample (field_response error model + steering vibration + mount rotation)."""
        p = self.params
        vib = self.imu_vibration_rms()
        noise = math.radians(math.hypot(p.gyro_still_noise_sd_deg_s, p.gyro_vibration_deg_s_per_m_s2 * vib))
        omega = self.w + self.external_yaw_rate
        rot = p.imu_rotation
        force = (self.accel, self.v * self.w, GRAVITY_M_S2)
        gyro = self.gyro_scale * rot[2][2] * omega + self.gyro_bias + noise * self._sensor_normal()
        axis = vib / math.sqrt(3.)
        acc = tuple(sum(rot[i][k] * force[k] for k in range(3)) + axis * self._sensor_normal() for i in range(3))
        return dict(stamp_s=self.time_s, gyro_z_rad_s=gyro, accel_m_s2=acc)

    def lidar_pose(self):
        p = self.params
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = p.lidar_x_in_base_m, p.lidar_y_in_base_m
        return (self.x + dx * c - dy * s, self.y + dx * s + dy * c, self.yaw)

    # --- status, truth, records -------------------------------------------------------------------------------
    def steer_status(self):
        """MCU-like steering status (no truth): what an MCU knows.

        raw ADC and its nominal decode, the target, the commanded steering PWM (direct_pwm: the command the
        plant gave; test_servo: None, because the test servo computes its output from the noise-free truth
        inside the backend), the commanded EN and the MCU fault mask. The driver's actual output
        (applied_steer_pwm, after EN and driver faults) is truth: it is in truth() only.
        """
        p = self.params
        out = self._steer
        measured = p.geometry.link_table.wheel_from_pot((out.pot_raw - p.pot_center_raw) / p.pot_counts_per_rad)
        return dict(stamp_s=self.time_s, backend_time_s=out.sim_time_s, mode=p.steer_mode,
                    target_rad=self.steer_target_rad if p.steer_mode == 'test_servo' else None,
                    target_request_mrad=self.steer_request_mrad, target_clamped=self.steer_target_clamped,
                    pot_raw=out.pot_raw, measured_rad=measured,
                    command_steer_pwm=self.steer_pwm_command if p.steer_mode == 'direct_pwm' else None,
                    steer_en=self.steer_en, fault_bits=out.fault_bits, faults=out.faults())

    def status(self):
        """MCU-like drive status (requested and applied duty only; no speed, no encoder)."""
        return dict(requested_pct=self.throttle_request_pct, target_pct=self._ramp.target,
                    applied_pct=self._ramp.applied, reported_duty_units=int(round(self._ramp.applied * 255 / 100)),
                    drive_en=self.drive_en)

    def servo_view_rad(self):
        """The angle the test servo sees now: T(pot-side true angle), zero offset unknown (protocol section 6).

        pot_location motor reads the motor side; wheel reads the link side, recovered from the true
        wheel-side angle without the zero offset and the left/right asymmetry. Truth-side helper: the start
        hold target (before the first command) and evaluation; the hold_measured stop uses
        measured_hold_rad() (decision R13-2). Not an executor input.
        """
        p = self.params
        if p.pot_location == 'motor':
            return p.geometry.link_table.wheel_from_pot(self._steer.motor_side_angle_rad)
        u = self._steer.delta_eq_rad - self.zero_offset_rad
        return u / (1. + p.steer_lr_asymmetry) if u >= 0. else u / (1. - p.steer_lr_asymmetry)

    def measured_hold_rad(self):
        """What a hold_measured stop holds (decision R13-2): the MCU-side potentiometer measurement, filtered like the
        MCU filters it and decoded with the nominal calibration (the decode of steer_status()['measured_rad']).

        The MCU takes one ADC sample per 1 ms tick and holds the median of the last pot_filter.median_of of them
        (ESP protocol 7.2). The plant sees one sample per plant step (the last ADC sample of the backend step), so
        here the median is over the last median_of plant-step samples; before median_of steps the first sample
        fills the buffer, as the MCU refills it at boot. In firmware mode the MCU computes the real one (T1 rig).
        Not truth: it carries the potentiometer errors (noise, supply sag, gain, linearity, drift), so with the
        test servo (whose feedback is the noise-free servo view) the wheel can move by that error after the stop.
        """
        p = self.params
        samples = list(self._pot_samples)
        samples = [samples[0]] * (p.pot_median_of - len(samples)) + samples
        raw = sorted(samples)[p.pot_median_of // 2]
        return p.geometry.link_table.wheel_from_pot((raw - p.pot_center_raw) / p.pot_counts_per_rad)

    def rear_axle_pose(self):
        """Rear-axle centre (x, y, yaw); evaluation and geometry checks only."""
        x_rear = self.params.geometry.rear_axle_x_in_base_m
        return (self.x + x_rear * math.cos(self.yaw), self.y + x_rear * math.sin(self.yaw), self.yaw)

    def truth(self):
        """Evaluation-only state (never an executor or estimator input)."""
        out = self._steer
        event = self._decel_event
        return dict(time_s=self.time_s, x=self.x, y=self.y, yaw=self.yaw, v=self.v, w=self.w,
                    distance_m=self.distance_m, regime=self.regime, applied_duty_pct=self._ramp.applied,
                    drive_target_pct=self._ramp.target, drive_en=self.drive_en, stop_mode=self.stop_mode,
                    decelerating=self.decelerating, decel_event=None if event is None else dict(event),
                    steer_target_rad=self.steer_target_rad, delta_eq_rad=out.delta_eq_rad,
                    delta_left_rad=out.delta_left_rad, delta_right_rad=out.delta_right_rad,
                    motor_side_angle_rad=out.motor_side_angle_rad, applied_steer_pwm=out.applied_steer_pwm,
                    model_flags=out.flags(), curvature_1_m=self.curvature, steer_curvature_1_m=self.steer_curvature,
                    wander_deg_per_m=self.wander_deg_per_m, lateral_slip_m_s=self.lateral,
                    speed_fluctuation=self.speed_fluctuation, rear_axle=self.rear_axle_pose()[:2])

    def result_label(self):
        """Identity block for results: the profile label plus the actuator backend (class, level, profile level).

        actuator_backend.matches_profile is False when the backend models another level than the profile's
        steer_actuator_level (None when the backend does not report its level, e.g. a test double).
        """
        label = self.params.profile.result_label()
        level = self.backend_level
        label['actuator_backend'] = dict(cls=type(self.backend).__name__, level=level,
                                         profile_level=self.profile_actuator_level,
                                         matches_profile=None if level is None else level == self.profile_actuator_level)
        return label

    def run_info(self):
        """Per-run constants and identities for result records (evaluation only)."""
        backend_info = getattr(self.backend, 'info', None)
        return dict(plant_model=PLANT_MODEL, profile=self.result_label(),
                    seed=list(self.seed) if isinstance(self.seed, (tuple, list)) else self.seed,
                    draw_seed=self.draw_seed, surface=self.surface, curve=self.params.curve,
                    curve_role=self.params.curve_role, geometry_check=self.params.geometry_check,
                    steer_mode=self.params.steer_mode, gyro_noise_scale=self.params.gyro_noise_scale,
                    draws=dict(self.draws), speed_run_scale=self.speed_run_scale, gyro_scale=self.gyro_scale,
                    gyro_bias_rad_s=self.gyro_bias, still_vibration_rms_m_s2=self.still_vibration,
                    backend=dict(cls=type(self.backend).__name__, level=self.backend_level,
                                 info=backend_info() if callable(backend_info) else None))

    def close(self):
        """Release the steering backend (a native backend owns a process)."""
        self.backend.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # --- internals -----------------------------------------------------------------------------------------------
    def _command_drive(self, throttle):
        # A pending brake / coast request stays: the duty is already 0, the next step starts its slow-down event.
        self.throttle_request_pct = throttle
        self.drive_en = True
        self.stop_mode = None
        self._ramp.command(throttle)
        self.command_time_s = self.time_s

    def _speed_target(self, duty, delta):
        if duty == 0.:
            return 0.
        p = self.params
        speed = forward_curve(abs(duty) * p.battery_voltage_scale, p.curve_params)
        if speed == 0.:
            return 0.
        if duty < 0.:
            speed *= p.reverse_curve
        speed *= self.speed_run_scale * (1. + self.speed_fluctuation)
        speed *= max(0., 1. - self.scrub_speed_loss_per_deg * abs(math.degrees(delta)))
        return math.copysign(speed, duty)

    def _advance_wander(self, distance_m):
        """Curvature wander: OU process in travelled distance (FieldPlant.advance_wander)."""
        if distance_m <= 0:
            return
        p = self.params
        b = math.exp(-distance_m / p.wander_corr_length_m)
        self.wander_deg_per_m = (p.wander_mean_deg_per_m + b * (self.wander_deg_per_m - p.wander_mean_deg_per_m)
                                 + math.sqrt(1 - b * b) * p.wander_sd_deg_per_m * self.normal())

    def _start_decel_event(self, mode):
        """One slow-down event. The speed is held in the event step and the following steps until
        (step end time - since) reaches onset_s, i.e. for onset_s in total (FieldPlant convention)."""
        stop = self.params.stop_mode(mode)
        z = self._stop_normal()                       # the k-th event takes the k-th draw
        decel = stop.decel_median_m_s2 if self.params.geometry_check else stop.decel_m_s2(z)
        self._decel_events += 1
        self._decel_event = dict(mode=mode, index=self._decel_events, since=self.time_s, onset_s=stop.onset_s,
                                 decel_m_s2=decel, lag=mode == 'release', speed_m_s=self.v)


class SteeringActuation:
    """Harness actuation hook for SteeredPlant (closed_loop.harness run_episode(actuation=...)).

    values(command) reads command.throttle_pct (signed duty %) and command.steer_target_mrad
    (mrad, + left); apply() calls plant.command_actuation(throttle_pct, steer_target_mrad);
    a reversal is a sign flip of the throttle. No turn_sign: the steering chatter metrics are
    defined by the steering evaluator (P2), so the harness reports them as None.
    """

    def values(self, command):
        return (command.throttle_pct, command.steer_target_mrad)

    def apply(self, plant, values):
        plant.command_actuation(*values)

    def is_reversal(self, values, last_values):
        return values[0] * last_values[0] < 0

    def trace_fields(self, values):
        return dict(throttle_pct=values[0], steer_target_mrad=values[1])

    def initial_values(self):
        return (0, 0)


def steered_plant_factory(profile=None, *, curve='main', geometry_check=False, steer_mode='test_servo',
                          backend_factory=None, initial_steer_rad=0.):
    """plant_factory for closed_loop.harness.run_episode: (params, *, surface, seed, pose, config) -> SteeredPlant.

    The harness passes FieldResponseParams as params; they describe the differential plant and
    are not used. From the EpisodeConfig only gyro_noise_scale is applied; plant_overrides
    (FieldPlant parameter names) are refused, steered-plant conditions come from
    profile.with_overrides(). backend_factory() returns a new unconfigured SteeringBackend per
    episode (None: default_backend(), the level of the profile's steer_actuator_level, so a native
    process per episode for L1). The harness does not close plants; close a native backend yourself
    (plant.close()); otherwise the backend's finalizer stops the process when the plant is collected.
    """
    base = SteeredPlantParams.from_profile(profile, curve=curve, geometry_check=geometry_check, steer_mode=steer_mode)

    def factory(params=None, *, surface='normal', seed=0, pose=(0., 0., 0.), config=None):
        plant_params = base
        if config is not None:
            overrides = getattr(config, 'plant_overrides', ())
            if overrides:
                raise ValueError('steered_plant_factory: EpisodeConfig.plant_overrides %r name FieldPlant parameters; '
                                 'give steered-plant conditions as profile.with_overrides()' % (dict(overrides),))
            scale = getattr(config, 'gyro_noise_scale', 1.)
            if scale != 1.:
                plant_params = base.with_gyro_noise_scale(scale)
        backend = backend_factory() if backend_factory is not None else None
        return SteeredPlant(plant_params, surface=surface, seed=seed, pose=pose, backend=backend,
                            initial_steer_rad=initial_steer_rad)

    factory.params = base
    return factory
