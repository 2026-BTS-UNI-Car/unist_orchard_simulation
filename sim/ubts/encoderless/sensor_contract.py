"""Bounded, offline reader for native LaserScan/Imu extraction packets.

No ROS, receipt clock, estimator, motion authority, deduplication or deskew is
provided. Storage log time is NOT receipt time. A driver profile is an explicit
caller assertion about array order, not proof of the actual producer's behavior.
The current matcher still has its separate 3200-point limit. Even an input marked
matching-compatible has no geometry, freshness, planar-motion or accuracy proof.

Packets use the private extractor's native field names; portable tests contain
synthetic data only. ``None`` may represent a nonfinite value lost by JSON export;
the original NaN/+inf/-inf distinction cannot be recovered or invented.
"""

from collections.abc import Mapping
from dataclasses import dataclass, field
import math

import numpy as np


MAX_RAW_BEAMS = 8192
MATCHER_POINT_LIMIT = 3200  # Compatibility only; registration.py is unchanged.
MAX_STAMP_NS = 2**63 - 1


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _stamp(value):
    return type(value) is int and 0 <= value <= MAX_STAMP_NS


def _label(value):
    return isinstance(value, str) and bool(value.strip()) and len(value) <= 256


def _numeric_array(value, minimum, maximum):
    if not isinstance(value, (list, tuple)) or not minimum <= len(value) <= maximum:
        raise ValueError('invalid_array_length')
    # Keep nonfinite/null numerical payloads for diagnostics, never coerce bool,
    # text, nested containers or arbitrary objects into measurements.
    if not all(v is None or type(v) in (int, float) for v in value):
        raise ValueError('invalid_array_value')
    if any(type(v) is int and abs(v) > MAX_STAMP_NS for v in value):
        raise ValueError('unbounded_array_integer')
    return tuple(value)


@dataclass(frozen=True)
class SensorIdentity:
    archive_id: str
    window_id: str
    topic: str
    stamp_ns: int
    log_ns: int
    cdr_sha256: str

    @property
    def message_key(self):
        """Same archived message across overlapping extraction windows."""
        return (self.archive_id, self.topic, self.stamp_ns,
                self.log_ns, self.cdr_sha256)

    @property
    def occurrence_key(self):
        return (self.window_id,) + self.message_key


def _identity(packet, archive_id, window_id):
    if not isinstance(packet, Mapping):
        raise ValueError('invalid_packet')
    if not all(_label(v) for v in (archive_id, window_id, packet.get('topic'))):
        raise ValueError('invalid_identity_labels')
    if not _stamp(packet.get('stamp_ns')) or not _stamp(packet.get('log_ns')):
        raise ValueError('invalid_integer_timestamps')
    digest = packet.get('cdr_sha256')
    if (not isinstance(digest, str) or len(digest) != 64
            or any(c not in '0123456789abcdef' for c in digest)):
        raise ValueError('invalid_cdr_digest')
    return SensorIdentity(archive_id, window_id, packet['topic'],
                          packet['stamp_ns'], packet['log_ns'], digest)


@dataclass(frozen=True)
class LaserDriverProfile:
    name: str
    frame_id: str
    max_raw_beams: int
    array_time_direction: int
    range_max_semantics: str
    header_reference: str

    def __post_init__(self):
        if (not _label(self.name) or not _label(self.frame_id)
                or type(self.max_raw_beams) is not int
                or not 2 <= self.max_raw_beams <= MAX_RAW_BEAMS
                or type(self.array_time_direction) is not int
                or self.array_time_direction not in (-1, 1)
                or self.range_max_semantics not in ('unknown', 'exclusive_no_return')
                or self.header_reference != 'first_acquired_ray'):
            raise ValueError('invalid_driver_profile')


@dataclass(frozen=True)
class RangeMasks:
    finite_return: tuple[bool, ...]
    at_range_max: tuple[bool, ...]
    below_min: tuple[bool, ...]
    above_max: tuple[bool, ...]
    missing_or_nonfinite: tuple[bool, ...]
    no_return: tuple[bool, ...]


@dataclass(frozen=True)
class RawLaserScan:
    identity: SensorIdentity
    frame_id: str
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float
    ranges: tuple
    intensities: tuple


@dataclass(frozen=True)
class LaserContract:
    valid: bool
    reason: str
    raw: RawLaserScan | None = None
    masks: RangeMasks | None = None
    ray_stamp_ns: tuple[int, ...] | None = None
    matching_compatible: bool = False
    matching_reasons: tuple[str, ...] = ()
    profile_name: str | None = None
    runtime_authority: bool = field(default=False, init=False)


def read_laser_scan(packet, archive_id, window_id, profile=None):
    """Validate raw metadata separately from the instantaneous matcher contract.

    ``no_return`` means no finite measurement was supplied, plus an explicitly
    declared maximum-range sentinel, if any. It never means a ray is free space.
    Exactly range_max stays distinct even when its producer semantics are unknown.
    No range is clamped, no array truncated, and no XY points are manufactured.
    """
    raw = None
    try:
        identity = _identity(packet, archive_id, window_id)
        frame = packet.get('frame_id')
        if not _label(frame):
            raise ValueError('invalid_frame')
        ranges = _numeric_array(packet.get('ranges'), 2, MAX_RAW_BEAMS)
        intensities = _numeric_array(packet.get('intensities'), 0, MAX_RAW_BEAMS)
        if len(intensities) not in (0, len(ranges)):
            raise ValueError('intensity_length_mismatch')
        names = ('angle_min', 'angle_max', 'angle_increment', 'time_increment',
                 'scan_time', 'range_min', 'range_max')
        fields = tuple(packet.get(name) for name in names)
        if not all(_finite(v) for v in fields):
            raise ValueError('invalid_geometry_number')
        raw = RawLaserScan(identity, frame, *fields, ranges, intensities)
        if not (0 <= raw.range_min < raw.range_max
                and raw.time_increment >= 0 and raw.scan_time >= 0
                and raw.angle_increment != 0):
            raise ValueError('invalid_geometry')
        angle_span = (len(ranges) - 1) * raw.angle_increment
        if (abs(angle_span) > 2 * math.pi + abs(raw.angle_increment)
                or not math.isclose(raw.angle_min + angle_span, raw.angle_max,
                                    rel_tol=0, abs_tol=max(1e-6, abs(raw.angle_increment) * .001))):
            raise ValueError('inconsistent_angles')
        duration = (len(ranges) - 1) * raw.time_increment
        if not math.isfinite(duration) or duration > raw.scan_time + 1e-6:
            raise ValueError('inconsistent_scan_timing')
    except ValueError as error:
        return LaserContract(False, str(error), raw=raw)

    reasons = []
    profile_ok = isinstance(profile, LaserDriverProfile)
    if not profile_ok:
        reasons.append('missing_driver_profile' if profile is None else 'invalid_driver_profile')
    elif raw.frame_id != profile.frame_id:
        reasons.append('profile_frame_mismatch')
        profile_ok = False
    elif len(ranges) > profile.max_raw_beams:
        reasons.append('profile_beam_limit_exceeded')
        profile_ok = False

    finite = tuple(_finite(v) for v in ranges)
    inside = tuple(ok and raw.range_min <= v < raw.range_max for v, ok in zip(ranges, finite))
    maximum = tuple(ok and v == raw.range_max for v, ok in zip(ranges, finite))
    below = tuple(ok and v < raw.range_min for v, ok in zip(ranges, finite))
    above = tuple(ok and v > raw.range_max for v, ok in zip(ranges, finite))
    missing = tuple(not ok for ok in finite)
    max_is_no_return = profile_ok and profile.range_max_semantics == 'exclusive_no_return'
    no_return = tuple(miss or (at_max and max_is_no_return)
                      for miss, at_max in zip(missing, maximum))
    masks = RangeMasks(inside, maximum, below, above, missing, no_return)
    ray_stamps = None
    if profile_ok and duration > (MAX_STAMP_NS - identity.stamp_ns) / 1_000_000_000:
        reasons.append('ray_timestamp_overflow')
    elif profile_ok:
        # Round each absolute offset ONCE; never accumulate rounded increments.
        offsets = tuple(round(i * raw.time_increment * 1_000_000_000)
                        for i in range(len(ranges)))
        if identity.stamp_ns + offsets[-1] > MAX_STAMP_NS:
            reasons.append('ray_timestamp_overflow')
        elif raw.time_increment > 0 and any(a >= b for a, b in zip(offsets, offsets[1:])):
            reasons.append('ray_timing_not_resolvable_ns')
        else:
            acquisition_offsets = offsets if profile.array_time_direction == 1 else offsets[::-1]
            ray_stamps = tuple(identity.stamp_ns + offset for offset in acquisition_offsets)
    if raw.time_increment > 0:
        reasons.append('rolling_deskew_unavailable')
    if sum(inside) > MATCHER_POINT_LIMIT:
        reasons.append('matcher_point_limit_exceeded')
    if sum(inside) < 24:
        reasons.append('insufficient_finite_returns')
    return LaserContract(True, 'valid_raw_scan', raw, masks, ray_stamps,
                         not reasons, tuple(reasons), profile.name if profile_ok else None)


@dataclass(frozen=True)
class ImuGroup:
    valid: bool
    available: bool
    reason: str
    values: tuple | None
    covariance: tuple | None
    uncertainty_unknown: bool


@dataclass(frozen=True)
class ImuContract:
    valid: bool
    reason: str
    identity: SensorIdentity | None = None
    frame_id: str | None = None
    orientation: ImuGroup | None = None
    angular_velocity: ImuGroup | None = None
    linear_acceleration: ImuGroup | None = None
    runtime_authority: bool = field(default=False, init=False)


def _covariance_reason(covariance):
    if not all(_finite(v) for v in covariance):
        return 'nonfinite_covariance'
    if any(covariance[i] < 0 for i in (0, 4, 8)):
        return 'negative_variance'
    if any(not math.isclose(covariance[i], covariance[j], rel_tol=1e-9, abs_tol=1e-12)
           for i, j in ((1, 3), (2, 6), (5, 7))):
        return 'asymmetric_covariance'
    scale = max(abs(v) for v in covariance)
    if scale == 0:
        return None
    normalized = np.asarray(covariance, dtype=float).reshape(3, 3) / scale
    # Check eigenvalues, not tiny principal minors: e.g. a zero-variance block
    # with 1e-7 off-diagonals has a material -1e-7 eigenvalue but only -1e-14
    # minors. Symmetrization is for this numerical check only; raw is unchanged.
    try:
        eigenvalues = np.linalg.eigvalsh((normalized + normalized.T) * .5)
    except np.linalg.LinAlgError:
        return 'covariance_eigendecomposition_failed'
    if eigenvalues[0] < -1e-12:
        return 'non_psd_covariance'
    return None


def _imu_group(packet, value_name, covariance_name, size):
    values = covariance = None
    try:
        values = _numeric_array(packet.get(value_name), size, size)
        covariance = _numeric_array(packet.get(covariance_name), 9, 9)
    except ValueError as error:
        return ImuGroup(False, False, str(error), values, covariance, True)
    if covariance[0] == -1:
        # The unavailable contract wins over even a plausible numerical payload.
        return ImuGroup(True, False, 'unavailable_minus_one', values, covariance, True)
    if not all(_finite(v) for v in values):
        return ImuGroup(False, False, 'nonfinite_values', values, covariance, True)
    if size == 4 and not math.isclose(math.hypot(*values), 1., rel_tol=0, abs_tol=1e-6):
        return ImuGroup(False, False, 'nonunit_quaternion', values, covariance, True)
    reason = _covariance_reason(covariance)
    if reason:
        return ImuGroup(False, False, reason, values, covariance, True)
    unknown = all(v == 0 for v in covariance)
    return ImuGroup(True, True, 'unknown_uncertainty' if unknown else 'reported_covariance',
                    values, covariance, unknown)


def read_imu(packet, archive_id, window_id):
    """Respect availability and uncertainty independently for each IMU group."""
    try:
        identity = _identity(packet, archive_id, window_id)
        frame = packet.get('frame_id')
        if not _label(frame):
            raise ValueError('invalid_frame')
    except ValueError as error:
        return ImuContract(False, str(error))
    orientation = _imu_group(packet, 'quaternion', 'orientation_covariance', 4)
    angular = _imu_group(packet, 'angular_velocity', 'angular_velocity_covariance', 3)
    linear = _imu_group(packet, 'linear_acceleration', 'linear_acceleration_covariance', 3)
    valid = all(group.valid for group in (orientation, angular, linear))
    return ImuContract(valid, 'valid_raw_imu' if valid else 'invalid_imu_group',
                       identity, frame, orientation, angular, linear)
