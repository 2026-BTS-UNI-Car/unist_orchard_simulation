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
Every value is a placeholder of the profile: placeholder-based SIM results, not field validation.
"""
from __future__ import annotations

import math
import random

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
