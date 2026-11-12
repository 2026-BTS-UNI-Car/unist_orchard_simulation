"""IMU error model of the steering G-SIM (design 3.7, 8.3 P6a): Gazebo's ideal IMU -> /imu/data_raw.

The same error model as the F-SIM plant (steered_plant.SteeredPlant.imu_sample, field_response rules), applied to the
Gazebo IMU sample (sensor frame, mount rotation already in the SDF sensor pose by gz_model):

  per run (seeded)  gyro scale ~ N(imu_gyro_scale.mean, .sd), z gyro bias ~ N(0, imu_gyro_bias_sd_deg_s),
                    still vibration RMS ~ log-uniform(imu_vibration_still_m_s2.low, .high)
  per sample        vibration RMS v = hypot(drive or still vibration, imu_vibration_steer_m_s2 x |steer PWM|);
                    drive vibration (by duty, field_response: below 90 % one value, 90..100 % linear) while the
                    drive duty is not 0 and |speed| > 0.02 m/s, else the still vibration
                    gyro z = scale x gyro z + bias + n, gyro x, y + n, n ~ N(0, hypot(still noise, coupling x v))
                    accel each axis + N(0, v / sqrt 3)
  orientation       not simulated (the F-SIM IMU model has none): identity quaternion with orientation_covariance[0]
                    = -1 (REP 145: orientation not provided); Gazebo's ideal orientation would be a truth heading
  covariances       angular velocity and acceleration covariance all 0 = unknown (the old SIM convention)

The drive duty, steering PWM and speed come from the plant side (/sim/truth/steering, /ground_truth/odom): the IMU
model is part of the sensor simulation, like the Embree LiDAR. The draws differ from the F-SIM plant's draw order
(G-SIM is not deterministic anyway, A9); the distributions are the same. Pure Python (random, math): no numpy.
compare_with_fsim(profile) (WF-4 g_gaps, audit G15) checks that claim against the F-SIM plant itself: the parameters,
the vibration rule and the gyro noise sd for a grid of states, the per-run draws (gyro scale, bias, still vibration)
and the per-sample gyro noise over seeds (it imports steered_plant, so numpy, only when called).
Every value is a placeholder of the profile: placeholder-based SIM results, not field validation.
"""
from __future__ import annotations

import math
import random
import warnings

VIBRATION_SPEED_M_S = .02      # steered_plant / FieldPlant: drive vibration only above this speed
SCHEMA = 'ubts.steer_imu_model.v1'


class ImuErrorModel:
    """Error model of one run. apply(gyro, accel, drive_duty_pct, steer_pwm_frac, speed_m_s) -> (gyro, accel)."""

    def __init__(self, profile, seed=0, gyro_noise_scale=1.):
        v = profile.value
        self.seed = int(seed)
        self.rng = random.Random(self.seed)
        gyro = v('imu_gyro_scale')
        still = v('imu_vibration_still_m_s2')
        scale = float(gyro_noise_scale)
        if not (math.isfinite(scale) and scale >= 0.):
            raise ValueError('gyro_noise_scale must be a finite number >= 0')
        self.gyro_noise_scale = scale
        self.gyro_scale_mean, self.gyro_scale_sd = float(gyro['mean']), float(gyro['sd'])
        self.gyro_bias_sd_deg_s = float(v('imu_gyro_bias_sd_deg_s'))
        self.still_noise_sd_deg_s = float(v('imu_gyro_still_noise_sd_deg_s')) * scale
        self.vibration_coupling = float(v('imu_gyro_vibration_deg_s_per_m_s2')) * scale
        self.drive_at_90 = float(v('imu_vibration_drive_at_90_m_s2'))
        self.drive_at_100 = float(v('imu_vibration_drive_at_100_m_s2'))
        self.drive_below_90 = float(v('imu_vibration_drive_below_90_m_s2'))
        self.steer_vibration = float(v('imu_vibration_steer_m_s2'))
        self.rate_hz = float(v('imu_rate_hz'))
        low, high = float(still['low']), float(still['high'])
        # per-run draws (fixed order)
        self.gyro_scale = self.rng.gauss(self.gyro_scale_mean, self.gyro_scale_sd)
        self.gyro_bias = math.radians(self.rng.gauss(0., self.gyro_bias_sd_deg_s))
        self.still_vibration = math.exp(self.rng.uniform(math.log(low), math.log(high)))
        self.samples = 0

    def drive_vibration(self, duty_pct):
        duty = abs(float(duty_pct))
        if duty < 90.:
            return self.drive_below_90
        return self.drive_at_90 + (min(duty, 100.) - 90.) * (self.drive_at_100 - self.drive_at_90) / 10.

    def vibration_rms(self, drive_duty_pct, steer_pwm_frac, speed_m_s):
        driving = abs(drive_duty_pct) > 1e-9 and abs(speed_m_s) > VIBRATION_SPEED_M_S
        base = self.drive_vibration(drive_duty_pct) if driving else self.still_vibration
        return math.hypot(base, self.steer_vibration * abs(float(steer_pwm_frac)))

    def apply(self, gyro, accel, drive_duty_pct=0., steer_pwm_frac=0., speed_m_s=0.):
        vib = self.vibration_rms(drive_duty_pct, steer_pwm_frac, speed_m_s)
        noise = math.radians(math.hypot(self.still_noise_sd_deg_s, self.vibration_coupling * vib))
        gauss = self.rng.gauss
        gx, gy, gz = (float(value) for value in gyro)
        out_gyro = (gx + noise * gauss(0., 1.), gy + noise * gauss(0., 1.),
                    self.gyro_scale * gz + self.gyro_bias + noise * gauss(0., 1.))
        axis = vib / math.sqrt(3.)
        out_accel = tuple(float(a) + axis * gauss(0., 1.) for a in accel)
        self.samples += 1
        return out_gyro, out_accel, vib

    def describe(self):
        return dict(schema=SCHEMA, seed=self.seed, rate_hz=self.rate_hz, gyro_scale=self.gyro_scale,
                    gyro_bias_deg_s=math.degrees(self.gyro_bias), still_vibration_rms_m_s2=self.still_vibration,
                    still_noise_sd_deg_s=self.still_noise_sd_deg_s,
                    vibration_coupling_deg_s_per_m_s2=self.vibration_coupling,
                    drive_vibration_m_s2=dict(below_90=self.drive_below_90, at_90=self.drive_at_90,
                                              at_100=self.drive_at_100),
                    steer_vibration_m_s2_per_full_pwm=self.steer_vibration, gyro_noise_scale=self.gyro_noise_scale,
                    orientation='not simulated: identity with orientation_covariance[0] = -1',
                    samples=self.samples, rule='steered_plant.imu_sample (field_response) on the Gazebo IMU sample')


def _stats(values):
    n = len(values)
    mean = sum(values) / n
    sd = math.sqrt(sum((x - mean) ** 2 for x in values) / max(n - 1, 1))
    return mean, sd


def compare_with_fsim(profile, seeds=range(40), samples=400, sigma=4.):
    """G15: this model against the F-SIM plant (steered_plant.SteeredPlant) for the same profile.

    parameters      every error-model constant equals the F-SIM plant parameter
    vibration rule  vibration_rms(duty, steer PWM, speed) equals the plant rule (drive vibration while the duty is not
                    0 and |speed| > 0.02 m/s, else the still vibration; the steering term in quadrature) on a grid
    noise sd        the gyro noise sd formula radians(hypot(still noise, coupling x vibration)) equals the plant's
    per-run draws   the means of gyro scale, gyro bias (deg/s) and log still vibration over the seeds agree within
                    sigma standard errors, and their sds within 35 % (draw order and generators differ by design)
    per-sample      the at-rest gyro z noise sd of both (samples per seed, bias and scale removed) within 15 %
    Returns dict(comparable, checks={name: dict(passed, ...)})."""
    from .l0_servo import L0ServoBackend
    from .steered_plant import SteeredPlant, SteeredPlantParams
    params = SteeredPlantParams.from_profile(profile)
    g = ImuErrorModel(profile, seed=0)
    checks = {}
    pairs = dict(gyro_scale_mean=(g.gyro_scale_mean, params.gyro_scale_mean),
                 gyro_scale_sd=(g.gyro_scale_sd, params.gyro_scale_sd),
                 gyro_bias_sd_deg_s=(g.gyro_bias_sd_deg_s, params.gyro_bias_sd_deg_s),
                 gyro_still_noise_sd_deg_s=(g.still_noise_sd_deg_s, params.gyro_still_noise_sd_deg_s),
                 gyro_vibration_coupling=(g.vibration_coupling, params.gyro_vibration_deg_s_per_m_s2),
                 drive_at_90=(g.drive_at_90, params.drive_vibration_at_90_m_s2),
                 drive_at_100=(g.drive_at_100, params.drive_vibration_at_100_m_s2),
                 drive_below_90=(g.drive_below_90, params.drive_vibration_below_90_m_s2),
                 steer_vibration=(g.steer_vibration, params.steer_vibration_m_s2))
    differ = {k: v for k, v in pairs.items() if abs(v[0] - v[1]) > 1e-12}
    checks['parameters'] = dict(passed=not differ, differ=differ, compared=sorted(pairs))
    worst_vib = worst_noise = 0.
    for duty in (0., 20., 89.9, 90., 95., 100., 120.):
        for pwm in (0., .3, 1.):
            for speed in (0., .01, .02, .03, .5):
                mine = g.vibration_rms(duty, pwm, speed)
                driving = abs(duty) > 1e-9 and abs(speed) > VIBRATION_SPEED_M_S
                base = params.drive_vibration(duty) if driving else g.still_vibration
                theirs = math.hypot(base, params.steer_vibration_m_s2 * abs(pwm))
                worst_vib = max(worst_vib, abs(mine - theirs))
                noise_mine = math.radians(math.hypot(g.still_noise_sd_deg_s, g.vibration_coupling * mine))
                noise_theirs = math.radians(math.hypot(params.gyro_still_noise_sd_deg_s,
                                                       params.gyro_vibration_deg_s_per_m_s2 * theirs))
                worst_noise = max(worst_noise, abs(noise_mine - noise_theirs))
    checks['vibration_rule'] = dict(passed=worst_vib <= 1e-12, max_abs_difference_m_s2=worst_vib)
    checks['gyro_noise_sd_formula'] = dict(passed=worst_noise <= 1e-15, max_abs_difference_rad_s=worst_noise)
    seeds = list(seeds)
    g_draws = dict(scale=[], bias=[], log_still=[])
    f_draws = dict(scale=[], bias=[], log_still=[])
    g_noise, f_noise = [], []
    for seed in seeds:
        model = ImuErrorModel(profile, seed=seed)
        g_draws['scale'].append(model.gyro_scale)
        g_draws['bias'].append(math.degrees(model.gyro_bias))
        g_draws['log_still'].append(math.log(model.still_vibration))
        for _ in range(samples):
            gyro, _, _ = model.apply((0., 0., 0.), (0., 0., 9.81), 0., 0., 0.)
            g_noise.append(gyro[2] - model.gyro_bias)
        backend = L0ServoBackend()
        with warnings.catch_warnings():          # an L0 backend for the IMU draws only (no steering is stepped)
            warnings.simplefilter('ignore')
            plant = SteeredPlant(params, seed=seed, backend=backend)
        try:
            f_draws['scale'].append(plant.gyro_scale)
            f_draws['bias'].append(math.degrees(plant.gyro_bias))
            f_draws['log_still'].append(math.log(plant.still_vibration))
            for _ in range(samples):
                f_noise.append(plant.imu_sample()['gyro_z_rad_s'] - plant.gyro_bias)
        finally:
            close = getattr(plant, 'close', None)
            if callable(close):
                close()
    for key in g_draws:
        gm, gs = _stats(g_draws[key])
        fm, fs = _stats(f_draws[key])
        se = math.sqrt((gs * gs + fs * fs) / len(seeds)) or 1e-12
        checks['per_run_%s' % key] = dict(
            passed=abs(gm - fm) <= sigma * se and (max(gs, fs) == 0. or abs(gs - fs) <= .35 * max(gs, fs)),
            g_mean=gm, f_mean=fm, g_sd=gs, f_sd=fs, standard_error=se)
    _, gs = _stats(g_noise)
    _, fs = _stats(f_noise)
    checks['per_sample_gyro_noise_at_rest'] = dict(passed=abs(gs - fs) <= .15 * max(gs, fs), g_sd_rad_s=gs,
                                                   f_sd_rad_s=fs, samples=len(g_noise))
    return dict(comparable=True, seeds=len(seeds), samples_per_seed=samples, checks=checks,
                note='the still vibration differs per seed in each model (different generators); the per-sample noise '
                     'is compared over all seeds together')
