"""Offline body-motion contract; no ROS, actuator or runtime authority.

An ``accepted`` increment means the caller has separately validated its LiDAR
registration and IMU pairing. This flag is not an accuracy certificate: the
current registration candidate has failed continuity and tilt characterization.
``planar_supported`` must be established upstream, never inferred from a small
relative yaw. This module neither decides a safe terrain envelope nor filters
ground returns. Wheel speeds, PWM and requested commands are not inputs.
"""

from dataclasses import dataclass, field
import math


SOURCE_MAX_AGE_NS = 100_000_000
RECEIPT_MAX_AGE_NS = 200_000_000
FUTURE_TOLERANCE_NS = 33_333_333
MAX_INTERVAL_NS = 250_000_000


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _stamp(value):
    return type(value) is int and 0 <= value <= 2**63 - 1


def _epoch(value):
    return isinstance(value, str) and bool(value.strip()) and len(value) <= 128


def _pair(value):
    return (isinstance(value, tuple) and len(value) == 2
            and all(_number(v) for v in value))


@dataclass(frozen=True)
class LaserMount:
    """Explicit static laser pose in base; no assumed 0.18 m default."""
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self):
        if (not all(_number(v) for v in (self.x_m, self.y_m, self.yaw_rad))
                or max(abs(self.x_m), abs(self.y_m)) > 2.
                or abs(self.yaw_rad) > math.pi):
            raise ValueError('invalid_laser_mount')


@dataclass(frozen=True)
class ScanIncrement:
    epoch: str
    sequence: int
    previous_scan_stamp_ns: int
    scan_stamp_ns: int
    scan_receipt_ns: int
    imu_stamp_ns: int
    imu_receipt_ns: int
    delta_xy_laser_m: tuple[float, float] | None
    delta_yaw_rad: float | None
    imu_yaw_rate_rad_s: float | None
    accepted: bool
    planar_supported: bool


@dataclass(frozen=True)
class BodyMotion:
    valid: bool
    reason: str
    epoch: str | None = None
    sequence: int | None = None
    previous_scan_stamp_ns: int | None = None
    scan_stamp_ns: int | None = None
    scan_receipt_ns: int | None = None
    imu_stamp_ns: int | None = None
    imu_receipt_ns: int | None = None
    delta_xy_base_m: tuple[float, float] | None = None
    linear_x_m_s: float | None = None
    linear_y_m_s: float | None = None
    yaw_rate_rad_s: float | None = None
    source: str = field(default='lidar_scanmatch_imu_heading', init=False)
    velocity_frame: str = field(default='base_interval_body_twist', init=False)
    runtime_authority: bool = field(default=False, init=False)


def _freshness(sample, source_now_ns, receipt_now_ns):
    if not _stamp(source_now_ns) or not _stamp(receipt_now_ns):
        return 'invalid_clock'
    for name in ('previous_scan_stamp_ns', 'scan_stamp_ns', 'scan_receipt_ns',
                 'imu_stamp_ns', 'imu_receipt_ns'):
        if not _stamp(getattr(sample, name, None)):
            return 'invalid_timestamps'
    interval = sample.scan_stamp_ns - sample.previous_scan_stamp_ns
    if not 0 < interval <= MAX_INTERVAL_NS:
        return 'invalid_scan_interval'
    for label, stamp in (('scan', sample.scan_stamp_ns), ('imu', sample.imu_stamp_ns)):
        age = source_now_ns - stamp
        if age < -FUTURE_TOLERANCE_NS:
            return 'future_' + label
        if age > SOURCE_MAX_AGE_NS:
            return 'stale_' + label
    for label, stamp in (('scan', sample.scan_receipt_ns), ('imu', sample.imu_receipt_ns)):
        age = receipt_now_ns - stamp
        if age < 0:
            return 'future_' + label + '_receipt'
        if age > RECEIPT_MAX_AGE_NS:
            return 'stale_' + label + '_receipt'
    return None


def motion_error(sample, *, epoch, source_now_ns, receipt_now_ns):
    """Recheck an offline motion sample at the consumer's original clock boundary."""
    if not isinstance(sample, BodyMotion) or sample.valid is not True:
        return 'invalid_motion'
    if not _epoch(epoch) or not _epoch(sample.epoch) or sample.epoch != epoch:
        return 'epoch_mismatch'
    if not _stamp(sample.sequence):
        return 'invalid_sequence'
    error = _freshness(sample, source_now_ns, receipt_now_ns)
    if error:
        return error
    if (not _pair(sample.delta_xy_base_m)
            or not all(_number(v) for v in
                       (sample.linear_x_m_s, sample.linear_y_m_s, sample.yaw_rate_rad_s))
            or sample.source != 'lidar_scanmatch_imu_heading'
            or sample.velocity_frame != 'base_interval_body_twist'
            or sample.runtime_authority is not False):
        return 'invalid_motion_values'
    return None


def _laser_to_base_twist(increment, mount):
    """Pure SE(2) math on a separately validated increment and explicit mount.

    No clocks, acceptance decisions or fabricated measurement times are used.
    Both localization evidence and the legacy fresh-motion estimator share this
    exact calculation; callers retain their own admission/freshness contracts.
    """
    yaw = increment.delta_yaw_rad
    c, s = math.cos(yaw), math.sin(yaw)
    cm, sm = math.cos(mount.yaw_rad), math.sin(mount.yaw_rad)
    x, y = increment.delta_xy_laser_m
    mx, my = mount.x_m, mount.y_m
    # T_base_laser @ delta_laser @ inverse(T_base_laser).
    bx = cm * x - sm * y + mx - (c * mx - s * my)
    by = sm * x + cm * y + my - (s * mx + c * my)
    if abs(yaw) < 1e-6:
        a = 1. - yaw * yaw / 6.
        b = yaw / 2. - yaw ** 3 / 24.
    else:
        a, b = math.sin(yaw) / yaw, (1. - math.cos(yaw)) / yaw
    determinant = a * a + b * b
    dt = (increment.scan_stamp_ns - increment.previous_scan_stamp_ns) * 1e-9
    vx = (a * bx + b * by) / determinant / dt
    vy = (-b * bx + a * by) / determinant / dt
    return (bx, by), vx, vy


class BodyMotionEstimator:
    """Validate consecutive increments and convert laser motion into base twist.

    Linear velocity is the SE(2) logarithm divided by the scan interval: an
    interval body-twist estimate under a constant-twist approximation, not an
    instantaneous speed measurement. End/start timestamps are retained. The
    yaw-rate field remains the independently supplied IMU measurement.
    """
    def __init__(self, mount):
        if not isinstance(mount, LaserMount):
            raise ValueError('explicit_laser_mount_required')
        self.mount = mount
        self.epoch = None
        self.fault = 'not_initialized'
        self.last = None
        self.last_source_now = self.last_receipt_now = None

    def reset(self, epoch):
        if not _epoch(epoch) or epoch == self.epoch:
            raise ValueError('new_epoch_required')
        self.epoch, self.fault, self.last = epoch, None, None
        self.last_source_now = self.last_receipt_now = None

    def _fail(self, reason):
        self.fault = self.fault or reason
        return BodyMotion(False, self.fault, epoch=self.epoch)

    def observe(self, increment, *, source_now_ns, receipt_now_ns):
        if self.fault:
            return self._fail(self.fault)
        if not isinstance(increment, ScanIncrement):
            return self._fail('invalid_increment')
        if not _epoch(increment.epoch) or increment.epoch != self.epoch:
            return self._fail('epoch_mismatch')
        if not _stamp(increment.sequence):
            return self._fail('invalid_sequence')
        error = _freshness(increment, source_now_ns, receipt_now_ns)
        if error:
            return self._fail(error)
        if self.last_source_now is not None and (source_now_ns < self.last_source_now
                                                or receipt_now_ns < self.last_receipt_now):
            return self._fail('clock_regression')
        if increment.accepted is not True:
            return self._fail('rejected_registration')
        if increment.planar_supported is not True:
            return self._fail('unsupported_tilt_or_surface')
        if (not _pair(increment.delta_xy_laser_m)
                or not _number(increment.delta_yaw_rad)
                or not _number(increment.imu_yaw_rate_rad_s)
                or math.hypot(*increment.delta_xy_laser_m) > .6 + 1e-10
                or abs(increment.delta_yaw_rad) >= math.pi):
            return self._fail('invalid_increment_values')
        if self.last is not None:
            if increment.sequence != self.last.sequence + 1:
                return self._fail('nonconsecutive_sequence')
            if increment.scan_stamp_ns <= self.last.scan_stamp_ns:
                return self._fail('nonincreasing_scan_stamp')
            if increment.previous_scan_stamp_ns != self.last.scan_stamp_ns:
                return self._fail('discontinuous_scan_pair')
            if (increment.scan_receipt_ns < self.last.scan_receipt_ns
                    or increment.imu_receipt_ns < self.last.imu_receipt_ns
                    or increment.imu_stamp_ns <= self.last.imu_stamp_ns):
                return self._fail('reordered_measurement')
        (bx, by), vx, vy = _laser_to_base_twist(increment, self.mount)
        if not all(math.isfinite(v) for v in (bx, by, vx, vy)):
            return self._fail('nonfinite_motion')
        result = BodyMotion(True, 'validated_increment', increment.epoch, increment.sequence,
                            increment.previous_scan_stamp_ns, increment.scan_stamp_ns,
                            increment.scan_receipt_ns, increment.imu_stamp_ns,
                            increment.imu_receipt_ns, (bx, by), vx, vy,
                            increment.imu_yaw_rate_rad_s)
        self.last = result
        self.last_source_now, self.last_receipt_now = source_now_ns, receipt_now_ns
        return result
