"""Offline vehicle linear/yaw feedback experiment; no actuator/runtime authority.

Outputs are hypothetical left/right PWM percentages, not measured wheel RPM.
All feedforward, gains and limits must be explicitly supplied as a calibration
fixture. No motor calibration is invented for the real vehicle. This module
does not establish safe braking, traction, obstacles, or estimator accuracy.
"""

from dataclasses import dataclass, field
import math

try:
    from .body_motion import MAX_INTERVAL_NS, _epoch, _number, motion_error
except ImportError:
    from body_motion import MAX_INTERVAL_NS, _epoch, _number, motion_error


@dataclass(frozen=True)
class VelocityCalibration:
    linear_ff_pct_per_m_s: float
    yaw_ff_pct_per_rad_s: float
    linear_kp: float
    linear_ki: float
    yaw_kp: float
    yaw_ki: float
    max_desired_linear_m_s: float
    max_desired_yaw_rad_s: float
    max_feedback_linear_m_s: float
    max_feedback_yaw_rad_s: float
    max_lateral_m_s: float
    max_percent: float
    slew_percent_per_s: float
    integral_limit_percent: float

    def __post_init__(self):
        if any(not _number(v) or v < 0. for v in vars(self).values()):
            raise ValueError('invalid_velocity_calibration')
        required_positive = (self.linear_ff_pct_per_m_s, self.yaw_ff_pct_per_rad_s,
                             self.max_desired_linear_m_s, self.max_desired_yaw_rad_s,
                             self.max_feedback_linear_m_s, self.max_feedback_yaw_rad_s,
                             self.max_lateral_m_s, self.max_percent,
                             self.slew_percent_per_s, self.integral_limit_percent)
        if (min(required_positive) <= 0. or self.max_percent > 100.
                or self.integral_limit_percent > self.max_percent
                or self.max_feedback_linear_m_s < self.max_desired_linear_m_s
                or self.max_feedback_yaw_rad_s < self.max_desired_yaw_rad_s):
            raise ValueError('invalid_velocity_limits')


@dataclass(frozen=True)
class VelocityCommand:
    valid: bool
    reason: str
    left_percent: float = 0.
    right_percent: float = 0.
    saturated: bool = False
    slew_limited: bool = False
    epoch: str | None = None
    measurement_stamp_ns: int | None = None
    runtime_authority: bool = field(default=False, init=False)


def _clamp(value, limit):
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class _PIResult:
    error: str | None = None
    linear_integral: float = 0.
    yaw_integral: float = 0.
    output: tuple[float, float] = (0., 0.)
    saturated: bool = False
    slew_limited: bool = False


def _limit_output(calibration, last_output, common, differential, dt):
    raw = (common - differential, common + differential)
    clipped = tuple(_clamp(v, calibration.max_percent) for v in raw)
    change = calibration.slew_percent_per_s * dt
    output = tuple(old + _clamp(new - old, change)
                   for old, new in zip(last_output, clipped))
    return output, any(abs(a - b) > 1e-12 for a, b in zip(raw, clipped)), any(
        abs(a - b) > 1e-12 for a, b in zip(clipped, output))


def _numeric_pi_update(c, linear_integral, yaw_integral, last_output,
                       desired_linear_m_s, desired_yaw_rad_s,
                       measured_linear_m_s, measured_yaw_rad_s, dt):
    """Shared pure arithmetic ONLY; each caller owns admission and clock gates."""
    linear_error = desired_linear_m_s - measured_linear_m_s
    yaw_error = desired_yaw_rad_s - measured_yaw_rad_s
    if not all(math.isfinite(v) for v in (linear_error, yaw_error)):
        return _PIResult('nonfinite_feedback_error')
    proposed_linear = linear_integral + c.linear_ki * linear_error * dt
    proposed_yaw = yaw_integral + c.yaw_ki * yaw_error * dt
    if not all(math.isfinite(v) for v in (proposed_linear, proposed_yaw)):
        return _PIResult('nonfinite_integral')
    pi = _clamp(proposed_linear, c.integral_limit_percent)
    qi = _clamp(proposed_yaw, c.integral_limit_percent)
    linear_base = c.linear_ff_pct_per_m_s * desired_linear_m_s + c.linear_kp * linear_error
    yaw_base = c.yaw_ff_pct_per_rad_s * desired_yaw_rad_s + c.yaw_kp * yaw_error
    common, differential = linear_base + pi, yaw_base + qi
    if not all(math.isfinite(v) for v in (pi, qi, common, differential,
                                         common - differential, common + differential)):
        return _PIResult('nonfinite_feedback_output')
    output, _, _ = _limit_output(c, last_output, common, differential, dt)
    achieved_common = (output[0] + output[1]) / 2.
    achieved_differential = (output[1] - output[0]) / 2.
    # Preserve conditional integration against both saturation and slew.
    if (common - achieved_common) * linear_error <= 1e-12:
        linear_integral = pi
    if (differential - achieved_differential) * yaw_error <= 1e-12:
        yaw_integral = qi
    output, saturated, slew = _limit_output(
        c, last_output, linear_base + linear_integral, yaw_base + yaw_integral, dt)
    return _PIResult(None, linear_integral, yaw_integral, output, saturated, slew)


class BodyVelocityTracker:
    """One update per consecutive validated motion interval, with latched STOP.

    The common output regulates body longitudinal velocity, the differential
    output regulates IMU yaw rate. Lateral velocity is retained and bounded as
    feedback validity, never projected into invented wheel-speed measurements.
    Every STOP/fault clears integrators and bypasses slew with a zero request;
    reset requires a new localization epoch. Zero desired linear AND yaw values
    invoke that same explicit stop. The first post-reset sample holds zero.
    """
    def __init__(self, calibration):
        if not isinstance(calibration, VelocityCalibration):
            raise ValueError('explicit_velocity_calibration_required')
        self.calibration = calibration
        self.epoch = None
        self.fault = 'not_initialized'
        self.linear_integral = self.yaw_integral = 0.
        self.last = None
        self.last_source_now = self.last_receipt_now = None
        self.last_output = (0., 0.)

    def reset(self, epoch):
        if not _epoch(epoch) or epoch == self.epoch:
            raise ValueError('new_epoch_required')
        self.epoch, self.fault, self.last = epoch, None, None
        self.linear_integral = self.yaw_integral = 0.
        self.last_source_now = self.last_receipt_now = None
        self.last_output = (0., 0.)

    def stop(self, reason='operator_stop'):
        if not isinstance(reason, str) or not reason.strip() or len(reason) > 128:
            reason = 'operator_stop'
        self.fault = self.fault or reason
        self.linear_integral = self.yaw_integral = 0.
        self.last_output = (0., 0.)
        return VelocityCommand(False, self.fault, epoch=self.epoch)

    def _limited(self, common, differential, dt):
        return _limit_output(self.calibration, self.last_output, common, differential, dt)

    def step(self, desired_linear_m_s, desired_yaw_rad_s, motion, *,
             source_now_ns, receipt_now_ns):
        if self.fault:
            return self.stop(self.fault)
        error = motion_error(motion, epoch=self.epoch, source_now_ns=source_now_ns,
                             receipt_now_ns=receipt_now_ns)
        if error:
            return self.stop(error)
        c = self.calibration
        if (not _number(desired_linear_m_s) or not _number(desired_yaw_rad_s)
                or abs(desired_linear_m_s) > c.max_desired_linear_m_s
                or abs(desired_yaw_rad_s) > c.max_desired_yaw_rad_s):
            return self.stop('invalid_desired_velocity')
        if (abs(motion.linear_x_m_s) > c.max_feedback_linear_m_s
                or abs(motion.linear_y_m_s) > c.max_lateral_m_s
                or abs(motion.yaw_rate_rad_s) > c.max_feedback_yaw_rad_s):
            return self.stop('feedback_outside_calibration')
        if self.last is not None:
            if motion.sequence != self.last.sequence + 1:
                return self.stop('nonconsecutive_sequence')
            if motion.previous_scan_stamp_ns != self.last.scan_stamp_ns:
                return self.stop('discontinuous_motion')
            if receipt_now_ns <= self.last_receipt_now:
                return self.stop('nonincreasing_control_clock')
            if (source_now_ns < self.last_source_now
                    or source_now_ns - self.last_source_now > MAX_INTERVAL_NS
                    or receipt_now_ns - self.last_receipt_now > MAX_INTERVAL_NS):
                return self.stop('control_clock_gap_or_regression')
            if (motion.scan_receipt_ns < self.last.scan_receipt_ns
                    or motion.imu_receipt_ns < self.last.imu_receipt_ns
                    or motion.imu_stamp_ns <= self.last.imu_stamp_ns):
                return self.stop('reordered_measurement')
        if desired_linear_m_s == 0. and desired_yaw_rad_s == 0.:
            return self.stop('zero_command')
        first_sample = self.last is None
        # Measurement intervals describe the observed motion, not how much
        # actuator time passed. Delayed-but-fresh scans can arrive in a burst;
        # only the caller's monotonic control clock permits slew/integration.
        dt = (0. if first_sample else
              (receipt_now_ns - self.last_receipt_now) * 1e-9)
        if not 0. <= dt <= MAX_INTERVAL_NS * 1e-9:
            return self.stop('invalid_control_interval')
        numeric = _numeric_pi_update(c, self.linear_integral, self.yaw_integral, self.last_output,
            desired_linear_m_s, desired_yaw_rad_s, motion.linear_x_m_s, motion.yaw_rate_rad_s, dt)
        if numeric.error:
            return self.stop(numeric.error)
        self.linear_integral, self.yaw_integral = numeric.linear_integral, numeric.yaw_integral
        output, saturated, slew = numeric.output, numeric.saturated, numeric.slew_limited
        self.last, self.last_output = motion, output
        self.last_source_now, self.last_receipt_now = source_now_ns, receipt_now_ns
        return VelocityCommand(True, 'initial_sample_zero' if first_sample else 'offline_feedback',
                               output[0], output[1], saturated, slew, self.epoch,
                               motion.scan_stamp_ns)
