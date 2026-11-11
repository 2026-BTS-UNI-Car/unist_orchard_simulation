"""Conditional SIM localization evidence, separate from fresh control motion.

Upstream must establish ScanIncrement.accepted and planar_supported from actual
registration and bounded IMU pairing under the explicit modeled profile. These
caller assertions are not accuracy certificates. No fit is recomputed here and
no physical mount, static scene or height assumption is proved by this module.

Consumer delay does not invalidate already observed geometry. The original
increment, profile and real evaluation clock remain attached to each immutable
record. Conversion to legacy BodyMotion rechecks its UNCHANGED freshness bounds
at the actual consumer clock. This module has no ROS or actuator IO.
"""
from dataclasses import dataclass, field
import math

try:
    from .body_motion import (BodyMotion, ScanIncrement, _epoch, _number, _pair,
                              _stamp, _freshness, _laser_to_base_twist,
                              FUTURE_TOLERANCE_NS)
    from .compact_registration import MAX_PAIR_INTERVAL_NS
    from .shadow_pipeline import (LiveClockReading, LiveSimOperatingProfile,
                                  LIVE_CLOCK_MAX_AGE_NS, MAX_IMU_BRACKET_NS)
except ImportError:
    from body_motion import (BodyMotion, ScanIncrement, _epoch, _number, _pair,
                             _stamp, _freshness, _laser_to_base_twist,
                             FUTURE_TOLERANCE_NS)
    from compact_registration import MAX_PAIR_INTERVAL_NS
    from shadow_pipeline import (LiveClockReading, LiveSimOperatingProfile,
                                 LIVE_CLOCK_MAX_AGE_NS, MAX_IMU_BRACKET_NS)


@dataclass(frozen=True)
class LocalizationProvenance:
    run_id: str
    scan_frame: str
    imu_frame: str
    profile_ref: str

    @classmethod
    def from_profile(cls, profile):
        if not isinstance(profile, LiveSimOperatingProfile):
            raise ValueError('explicit_live_sim_profile_required')
        return cls(profile.run_id, profile.scan_frame, profile.imu_frame, profile.evidence_ref)


@dataclass(frozen=True)
class LocalizedMotion:
    localization_usable: bool
    reason: str
    profile: LiveSimOperatingProfile
    provenance: LocalizationProvenance
    increment: ScanIncrement | None = None
    evaluated_clock: LiveClockReading | None = None
    delta_xy_base_m: tuple[float, float] | None = None
    linear_x_m_s: float | None = None
    linear_y_m_s: float | None = None
    source: str = field(default='lidar_scanmatch_imu_heading', init=False)
    velocity_frame: str = field(default='base_interval_body_twist', init=False)
    runtime_authority: bool = field(default=False, init=False)
    execution_eligible: bool = field(default=False, init=False)

    # Preserve, rather than copy/relabel, the original source/receipt metadata.
    @property
    def epoch(self): return self.increment.epoch if self.increment is not None else None
    @property
    def sequence(self): return self.increment.sequence if self.increment is not None else None
    @property
    def previous_scan_stamp_ns(self): return self.increment.previous_scan_stamp_ns if self.increment is not None else None
    @property
    def scan_stamp_ns(self): return self.increment.scan_stamp_ns if self.increment is not None else None
    @property
    def scan_receipt_ns(self): return self.increment.scan_receipt_ns if self.increment is not None else None
    @property
    def imu_stamp_ns(self): return self.increment.imu_stamp_ns if self.increment is not None else None
    @property
    def imu_receipt_ns(self): return self.increment.imu_receipt_ns if self.increment is not None else None
    @property
    def yaw_rate_rad_s(self): return self.increment.imu_yaw_rate_rad_s if self.increment is not None else None


def _clock_structure_error(clock):
    if not isinstance(clock, LiveClockReading):
        return 'clock_provenance_mismatch'
    try:
        LiveClockReading.__post_init__(clock)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return 'invalid_live_clock'
    return None


def _clock_error(clock, profile, previous=None):
    error = _clock_structure_error(clock)
    if error: return error
    if clock.run_id != profile.run_id: return 'clock_provenance_mismatch'
    if previous is not None:
        error = _clock_structure_error(previous)
        if error: return error
        if (clock.source_ns < previous.source_ns or clock.receipt_ns < previous.receipt_ns
                or clock.clock_sequence < previous.clock_sequence
                or clock.clock_receipt_ns < previous.clock_receipt_ns
                or clock.progress_receipt_ns < previous.progress_receipt_ns):
            return 'clock_regression'
        if clock.clock_sequence == previous.clock_sequence and (
                clock.source_ns, clock.clock_receipt_ns, clock.progress_receipt_ns) != (
                previous.source_ns, previous.clock_receipt_ns, previous.progress_receipt_ns):
            return 'clock_reused_sequence_changed'
    if clock.receipt_ns-clock.clock_receipt_ns > LIVE_CLOCK_MAX_AGE_NS:
        return 'clock_receipt_stale'
    if clock.receipt_ns-clock.progress_receipt_ns > LIVE_CLOCK_MAX_AGE_NS:
        return 'clock_source_stalled'
    return None


def _increment_error(increment, profile, provenance, clock):
    if (not isinstance(provenance, LocalizationProvenance)
            or not all(isinstance(value, str) and value.strip() and len(value) <= 256
                       for value in (provenance.run_id, provenance.scan_frame, provenance.imu_frame, provenance.profile_ref))
            or provenance != LocalizationProvenance.from_profile(profile)):
        return 'localization_provenance_mismatch'
    if not isinstance(increment, ScanIncrement): return 'invalid_increment'
    if not _epoch(increment.epoch): return 'epoch_mismatch'
    if not _stamp(increment.sequence): return 'invalid_sequence'
    fields = ('previous_scan_stamp_ns', 'scan_stamp_ns', 'scan_receipt_ns', 'imu_stamp_ns', 'imu_receipt_ns')
    if not all(_stamp(getattr(increment, name)) for name in fields): return 'invalid_timestamps'
    if not 0 < increment.scan_stamp_ns-increment.previous_scan_stamp_ns <= MAX_PAIR_INTERVAL_NS:
        return 'invalid_scan_interval'
    if increment.previous_scan_stamp_ns < profile.active_from_ns: return 'before_profile_active'
    if increment.accepted is not True: return 'rejected_registration'
    if increment.planar_supported is not True: return 'unsupported_tilt_or_surface'
    if (not _pair(increment.delta_xy_laser_m) or not _number(increment.delta_yaw_rad)
            or not _number(increment.imu_yaw_rate_rad_s)
            or math.hypot(*increment.delta_xy_laser_m) > .6 + 1e-10
            or abs(increment.delta_yaw_rad) >= math.pi):
        return 'invalid_increment_values'
    if increment.imu_stamp_ns > increment.scan_stamp_ns: return 'imu_after_scan_endpoint'
    if increment.scan_stamp_ns-increment.imu_stamp_ns > MAX_IMU_BRACKET_NS:
        return 'imu_outside_endpoint_bracket'
    for name, stamp in (('scan', increment.scan_stamp_ns), ('imu', increment.imu_stamp_ns)):
        if stamp-clock.source_ns > FUTURE_TOLERANCE_NS: return 'future_'+name
    for name, receipt in (('scan', increment.scan_receipt_ns), ('imu', increment.imu_receipt_ns)):
        if receipt > clock.receipt_ns: return 'future_'+name+'_receipt'
    return None


def localized_motion_error(record, *, profile, clock):
    """Revalidate explicit record structure/provenance, including copied geometry.

    A frozen dataclass is not an attestation: manually constructed/replaced
    records receive all checks again, including exact shared SE(2) derivation.
    This check deliberately does not discard geometry because of consumer age.
    """
    if not isinstance(profile, LiveSimOperatingProfile): return 'explicit_live_sim_profile_required'
    if not isinstance(record, LocalizedMotion) or record.localization_usable is not True:
        return 'invalid_localized_motion'
    if not isinstance(record.profile, LiveSimOperatingProfile):
        return 'localization_profile_mismatch'
    # Re-run the declared profile/mount validation; no implicit default mount.
    try:
        for declared in (profile, record.profile):
            LiveSimOperatingProfile.__post_init__(declared)
            declared.mount.__post_init__()
    except (AttributeError, TypeError, ValueError, OverflowError):
        return 'invalid_localization_profile'
    if record.profile != profile: return 'localization_profile_mismatch'
    error = _clock_error(record.evaluated_clock, profile)
    if error: return error
    error = _clock_error(clock, profile, record.evaluated_clock)
    if error: return error
    error = _increment_error(record.increment, profile, record.provenance, record.evaluated_clock)
    if error: return error
    error = _increment_error(record.increment, profile, record.provenance, clock)
    if error: return error
    if (not _pair(record.delta_xy_base_m)
            or not all(_number(v) for v in (record.linear_x_m_s, record.linear_y_m_s))
            or record.source != 'lidar_scanmatch_imu_heading'
            or record.velocity_frame != 'base_interval_body_twist'
            or record.runtime_authority is not False or record.execution_eligible is not False):
        return 'invalid_localized_motion_values'
    expected = _laser_to_base_twist(record.increment, profile.mount)
    if (record.delta_xy_base_m, record.linear_x_m_s, record.linear_y_m_s) != expected:
        return 'localized_geometry_mismatch'
    return None


def to_body_motion(record, *, profile, clock):
    """Create valid legacy motion ONLY after the actual current freshness gate."""
    error = localized_motion_error(record, profile=profile, clock=clock)
    if not error:
        error = _freshness(record.increment, clock.source_ns, clock.receipt_ns)
    if error:
        epoch = record.epoch if isinstance(record, LocalizedMotion) and isinstance(record.increment, ScanIncrement) else None
        return BodyMotion(False, error, epoch=epoch)
    increment = record.increment
    return BodyMotion(True, 'validated_increment', increment.epoch, increment.sequence,
                      increment.previous_scan_stamp_ns, increment.scan_stamp_ns,
                      increment.scan_receipt_ns, increment.imu_stamp_ns, increment.imu_receipt_ns,
                      record.delta_xy_base_m, record.linear_x_m_s, record.linear_y_m_s,
                      increment.imu_yaw_rate_rad_s)


class LocalizedMotionEstimator:
    """Age-independent geometry continuity under one explicit modeled profile.

    Clock staleness pauses admission without inventing a segment. Geometry,
    provenance, gaps and clock regressions latch until an explicit new segment.
    Clock boundaries remain monotonic across such localization-only resets.
    """
    def __init__(self, profile):
        if not isinstance(profile, LiveSimOperatingProfile):
            raise ValueError('explicit_live_sim_profile_required')
        profile.__post_init__()
        profile.mount.__post_init__()
        self.profile = profile
        self.provenance = LocalizationProvenance.from_profile(profile)
        self.epoch, self.fault = None, 'not_initialized'
        self.last = self.last_clock = None
        self.used_segments = set()

    def reset(self, epoch):
        if not _epoch(epoch) or epoch in self.used_segments or len(self.used_segments) >= 10000:
            raise ValueError('new_bounded_segment_required')
        self.epoch, self.fault, self.last = epoch, None, None
        self.used_segments.add(epoch)

    def _fail(self, reason, *, latch=True):
        if latch: self.fault = self.fault or reason
        return LocalizedMotion(False, self.fault or reason, self.profile, self.provenance)

    def observe(self, increment, *, provenance, clock):
        if self.fault: return self._fail(self.fault)
        error = _clock_error(clock, self.profile, self.last_clock)
        if error:
            if error in ('clock_receipt_stale', 'clock_source_stalled'):
                self.last_clock = clock
                return self._fail(error, latch=False)
            return self._fail(error)
        self.last_clock = clock
        error = _increment_error(increment, self.profile, provenance, clock)
        if error: return self._fail(error)
        if increment.epoch != self.epoch: return self._fail('epoch_mismatch')
        if self.last is not None:
            if increment.sequence != self.last.sequence+1: return self._fail('nonconsecutive_sequence')
            if increment.previous_scan_stamp_ns != self.last.scan_stamp_ns: return self._fail('discontinuous_scan_pair')
            if (increment.scan_receipt_ns < self.last.scan_receipt_ns
                    or increment.imu_receipt_ns < self.last.imu_receipt_ns
                    or increment.imu_stamp_ns <= self.last.imu_stamp_ns):
                return self._fail('reordered_measurement')
        xy, vx, vy = _laser_to_base_twist(increment, self.profile.mount)
        if not _pair(xy) or not all(_number(v) for v in (vx, vy)):
            return self._fail('nonfinite_motion')
        result = LocalizedMotion(True, 'validated_localization_increment', self.profile,
                                 provenance, increment, clock, xy, vx, vy)
        self.last = result
        return result
