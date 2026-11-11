"""Memory-only native navigation admission, not a SLAM or localization engine.

A separately qualified native adapter must bind this depth-one inbox to its
actual mapper lifetime. Nothing here reads SDK/ROS/network/tiles or accepts
PWM/encoder/truth as a measured pose. Fake providers belong to tests only.
"""
from dataclasses import dataclass
import math
import re
import threading

HEX64 = re.compile(r'^[0-9a-f]{64}$')
TRACKING = ('TRACKING', 'WAIT', 'LOST', 'CLOSED')
HAZARD = ('CLEAR', 'OCCUPIED', 'UNKNOWN')


def finite(value):
    if type(value) not in (int, float):return False
    try:return math.isfinite(value)
    except (OverflowError, ValueError):return False


@dataclass(frozen=True)
class NativeSourceBinding:
    """Adapter-computed canonical digests, never a viewer or motor capability."""
    session_SHA256: str
    publisher_epoch_SHA256: str
    transform_revision_SHA256: str
    map_frame: str

    def validate(self):
        for value in (self.session_SHA256, self.publisher_epoch_SHA256, self.transform_revision_SHA256):
            if type(value) is not str or not HEX64.fullmatch(value):
                raise ValueError('native_navigation_source_binding')
        if type(self.map_frame) is not str or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]{0,63}', self.map_frame):
            raise ValueError('native_navigation_frame')
        return self


@dataclass(frozen=True)
class NativePose2D:
    """Local native LiDAR/IMU pose in the bound frame, private RAM only."""
    x_m: float
    y_m: float
    yaw_rad: float
    age_s: object = None
    quality: str = 'UNKNOWN'

    def validate(self):
        if not all(finite(v) for v in (self.x_m, self.y_m, self.yaw_rad)):
            raise ValueError('finite_native_pose_only')
        if self.age_s is not None and (not finite(self.age_s) or not 0 <= self.age_s <= 30):
            raise ValueError('measured_same_domain_pose_age')
        if type(self.quality) is not str or self.quality not in ('QUALIFIED_LOCAL', 'UNCERTAIN', 'UNKNOWN'):
            raise ValueError('typed_native_pose_quality')
        return self


@dataclass(frozen=True)
class NativeNavigationObservation:
    binding: NativeSourceBinding
    sequence: int
    # Age measured by the adapter at receipt, within its qualified clock domain.
    # Host evaluate adds only elapsed host time; it never subtracts OS clocks.
    source_age_s: float
    tracking: str
    pose: object
    occupancy_complete: bool
    known_cells: int
    forward_hazard: str
    # A complete, same-snapshot, same-pose forward footprint/corridor query.
    # UNKNOWN includes absent or unqualified cost queries. It is never free.
    cost_pose_sequence: int

    def validate(self):
        if type(self.binding) is not NativeSourceBinding:
            raise ValueError('typed_native_source_binding')
        self.binding.validate()
        if type(self.sequence) is not int or not 1 <= self.sequence <= 9223372036854775807:
            raise ValueError('native_navigation_sequence')
        if not finite(self.source_age_s) or not 0 <= self.source_age_s <= 30:
            raise ValueError('measured_native_source_age_required')
        if type(self.tracking) is not str or self.tracking not in TRACKING or type(self.occupancy_complete) is not bool:
            raise ValueError('typed_native_tracking_and_completeness')
        if self.pose is not None and type(self.pose) is not NativePose2D:
            raise ValueError('native_pose_not_PWM_truth_or_encoder')
        if self.pose is not None:self.pose.validate()
        if type(self.known_cells) is not int or not 0 <= self.known_cells <= 4194304:
            raise ValueError('bounded_native_forward_query_known_cells')
        if type(self.forward_hazard) is not str or self.forward_hazard not in HAZARD or type(self.cost_pose_sequence) is not int:
            raise ValueError('typed_same_snapshot_native_cost')
        return self


class NativeNavigationGate:
    """One immutable observation; duplicates never refresh its age.

    Context change, changed same-sequence content and invalid objects fence this
    instance. A new source needs a new externally admitted gate, never adoption
    from an untrusted first packet. Evaluation and offer perform no external I/O.
    """
    def __init__(self, expected, max_age_s, *, test_only=False):
        if type(expected) is not NativeSourceBinding:raise ValueError('explicit_expected_navigation_source')
        self._expected = expected.validate()
        if not finite(max_age_s) or not 0 < max_age_s <= .5:
            raise ValueError('native_navigation_age_not_looser_than_sensor_guard')
        self._max_age_s = max_age_s
        if type(test_only) is not bool:raise ValueError('typed_generated_navigation_scope')
        # No qualified native adapter exists in this source component yet.
        # Only explicit in-process simulation injection may exercise READY.
        self._test_only = test_only
        self._lock = threading.Lock()
        self._latest = self._receipt = self._last_evaluate = None
        self._fault = None
        self._accepted = self._duplicates = self._older = self._rejected = 0

    @property
    def expected(self):return self._expected

    @property
    def max_age_s(self):return self._max_age_s

    @property
    def test_only(self):return self._test_only

    def offer(self, observation, receipt_monotonic):
        try:
            if type(observation) is not NativeNavigationObservation or not finite(receipt_monotonic):
                raise ValueError('typed_native_navigation_offer')
            observation.validate()
        except (TypeError, ValueError):
            with self._lock:
                self._fault = self._fault or 'INVALID_OBSERVATION'
                self._rejected = min(9223372036854775807, self._rejected + 1)
            return False
        with self._lock:
            if self._fault:return False
            if observation.binding != self.expected:
                self._fault = 'SOURCE_CHANGED';return False
            previous = self._latest
            if previous is not None:
                if observation.sequence < previous.sequence:
                    self._older = min(9223372036854775807, self._older + 1);return False
                if observation.sequence == previous.sequence:
                    if observation != previous:self._fault = 'SAME_SEQUENCE_CHANGED'
                    else:self._duplicates = min(9223372036854775807, self._duplicates + 1)
                    return False
                if receipt_monotonic < self._receipt:
                    self._fault = 'RECEIPT_REGRESSION';return False
            self._latest, self._receipt = observation, receipt_monotonic
            self._accepted = min(9223372036854775807, self._accepted + 1)
            return True

    def evaluate(self, now):
        with self._lock:
            if not finite(now) or (self._last_evaluate is not None and now < self._last_evaluate):
                self._fault = self._fault or 'CLOCK_REGRESSION'
            self._last_evaluate = now if finite(now) else self._last_evaluate
            o = self._latest
            reason = self._fault
            if reason is None and not self.test_only:reason = 'ADAPTER_UNQUALIFIED'
            age = None
            if reason is None:
                if o is None:reason = 'NO_OBSERVATION'
                elif now < self._receipt:reason = 'FUTURE_RECEIPT'
                else:
                    age = o.source_age_s + now - self._receipt
                    if age > self.max_age_s:reason = 'STALE'
                    elif o.tracking != 'TRACKING':reason = 'POSE_' + o.tracking
                    elif o.pose is None:reason = 'POSE_UNKNOWN'
                    elif o.pose.age_s is None or o.pose.quality != 'QUALIFIED_LOCAL':reason = 'POSE_UNCERTAIN'
                    elif o.pose.age_s + now - self._receipt > self.max_age_s:reason = 'POSE_STALE'
                    elif not o.occupancy_complete or o.known_cells == 0:reason = 'MAP_INCOMPLETE'
                    elif o.cost_pose_sequence != o.sequence:reason = 'COST_POSE_MISMATCH'
                    elif o.forward_hazard != 'CLEAR':reason = 'COST_' + o.forward_hazard
            return dict(state='READY' if reason is None else 'WAIT', allow_motion=reason is None,
                reason=reason or 'READY', source_binding_matches_expected=reason not in
                ('SOURCE_CHANGED', 'NO_OBSERVATION', 'INVALID_OBSERVATION'),
                provider_present=True, sequence=None if o is None else o.sequence,
                tracking='UNKNOWN' if o is None else o.tracking,
                pose_present=o is not None and o.pose is not None,
                pose_quality='UNKNOWN' if o is None or o.pose is None else o.pose.quality,
                pose_age_known=o is not None and o.pose is not None and o.pose.age_s is not None,
                occupancy_complete=False if o is None else o.occupancy_complete,
                forward_hazard='UNKNOWN' if o is None else o.forward_hazard,
                source_age_known=age is not None, age_s=age,
                accepted=self._accepted, duplicates=self._duplicates, older=self._older,
                rejected=self._rejected, generated_test_provider=self.test_only, SDK_adapter_qualified=False)
