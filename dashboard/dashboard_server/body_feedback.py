"""Pure LiDAR interval diagnostics and optional body feedback; no I/O/authority.

The SE(2) conjugation/logarithm is ported from
sim/ubts/encoderless/body_motion.py::_laser_to_base_twist, without importing SIM.
Body translation is an interval constant-twist estimate, not instantaneous
speed. Yaw feedback is the separately supplied causal native gravity-projected
IMU rate, not the LiDAR yaw difference. Caller measurement/validation flags
are declarations, not hardware accuracy certificates. No actual mount, clock
validation, planar envelope or localization qualification is supplied here.
Callers must serialize observe/invalidate/reset. PWM, wheels, commands, truth,
camera and GNSS priors are not inputs. Snapshot metadata never gains freshness
from being read; a consumer must recheck the original clocks.
"""
from dataclasses import dataclass, asdict
import copy
import math


def _number(v):
    try:
        return type(v) in (int, float) and math.isfinite(v)
    except OverflowError:
        return False


def _stamp(v):
    return type(v) is int and 0 <= v <= 2**63 - 1


def _name(v):
    return type(v) is str and 0 < len(v) <= 128 and v.strip() == v


def _pose(v):
    return type(v) is tuple and len(v) == 3 and all(_number(x) for x in v)


@dataclass(frozen=True)
class FeedbackContext:
    source_session: str
    epoch: str
    generation: int
    source_clock: str
    receipt_clock: str
    pose_frame: str
    laser_frame: str

    def valid(self):
        return _stamp(self.generation) and all(_name(v) for v in
            (self.source_session, self.epoch, self.source_clock,
             self.receipt_clock, self.pose_frame, self.laser_frame))


@dataclass(frozen=True)
class LaserPosePair:
    """Poses share context.pose_frame and use continuous, unwrapped yaw.

    Wrapped/branch-changed localization poses need an independently established
    continuous representation upstream; this observer does not guess a branch.
    """
    context: FeedbackContext
    previous_sequence: int
    sequence: int
    previous_source_stamp_ns: int
    source_stamp_ns: int
    previous_receipt_ns: int
    receipt_ns: int
    previous_pose: tuple
    pose: tuple
    accepted: bool = False
    map_eligible: bool = False
    independent_localization_validated: bool = False
    planar_supported: bool = False
    source_clock_verified: bool = False
    path_break: bool = False
    motion_prior: bool = False


@dataclass(frozen=True)
class NativeImuYaw:
    context: FeedbackContext
    source_stamp_ns: int
    receipt_ns: int
    yaw_rad_s: float
    native: bool = False
    gravity_projected: bool = False
    mount_verified: bool = False
    imu_frame: str = ""


@dataclass(frozen=True)
class MeasuredLaserMount:
    x_m: float
    y_m: float
    yaw_rad: float
    calibration_id: str
    measured: bool = False

    def valid(self):
        return (self.measured is True and _name(self.calibration_id)
            and all(_number(v) for v in (self.x_m, self.y_m, self.yaw_rad))
            and abs(self.yaw_rad) <= math.pi)


@dataclass(frozen=True)
class FeedbackTiming:
    """Caller-validated bounds in the explicitly named clocks, no default TTL."""
    max_interval_ns: int
    source_age_ns: int
    receipt_age_ns: int
    imu_lag_ns: int

    def valid(self):
        return all(_stamp(v) and v > 0 for v in
            (self.max_interval_ns, self.source_age_ns, self.receipt_age_ns, self.imu_lag_ns))


def _laser_to_base_twist(xy, yaw, interval_ns, mount):
    """Exact original SE(2) conjugation and logarithm arithmetic."""
    c, s = math.cos(yaw), math.sin(yaw)
    cm, sm = math.cos(mount.yaw_rad), math.sin(mount.yaw_rad)
    x, y = xy
    mx, my = mount.x_m, mount.y_m
    bx = cm * x - sm * y + mx - (c * mx - s * my)
    by = sm * x + cm * y + my - (s * mx + c * my)
    if abs(yaw) < 1e-6:
        a = 1. - yaw * yaw / 6.
        b = yaw / 2. - yaw ** 3 / 24.
    else:
        a, b = math.sin(yaw) / yaw, (1. - math.cos(yaw)) / yaw
    determinant = a * a + b * b
    dt = interval_ns * 1e-9
    vx = (a * bx + b * by) / determinant / dt
    vy = (-b * bx + a * by) / determinant / dt
    return (bx, by), vx, vy


class BodyFeedbackObserver:
    def __init__(self, mount=None, timing=None):
        self.mount, self.timing = mount, timing
        self._context = self._last = self._imu_last = self._clocks = None
        self._profile = None
        self._latest = self._view("not_observed")

    @staticmethod
    def _view(reason):
        return dict(schema="ubts.body_feedback.observation.v1", valid=False,
            body_eligible_at_observation=False, state="HOLD", reason=reason,
            context=None, sequence=None, previous_sequence=None,
            source_stamp_ns=None, receipt_ns=None, imu_source_stamp_ns=None,
            imu_receipt_ns=None, laser=None, body=None,
            freshness_recheck_required=True, calibration_certified=False,
            runtime_authority=False, motor_command_sent=False)

    def invalidate(self, reason="invalidated"):
        """Discard lineage/body on Stop, reset, reconnection or rejected fit."""
        self._context = self._last = self._imu_last = self._clocks = None
        self._profile = None
        self._latest.update(valid=False, body_eligible_at_observation=False,
            state="HOLD", reason=reason if _name(reason) else "invalidated", body=None,
            freshness_recheck_required=True)
        return self.snapshot()

    def reset(self, context=None):
        self.invalidate("reset_requires_fresh_pair")
        if context is not None:
            if type(context) is not FeedbackContext or not context.valid():
                return self.invalidate("invalid_context")
            self._context = context
        return self.snapshot()

    def _hold(self, reason):
        # Retain this observation's diagnostic, but never chain across a gap.
        self._last = self._imu_last = self._clocks = self._profile = None
        self._latest.update(valid=False, body_eligible_at_observation=False,
            state="HOLD", reason=reason, body=None)
        return self.snapshot()

    def _freshness(self, pair, imu, source_now_ns, receipt_now_ns):
        if not _stamp(source_now_ns) or not _stamp(receipt_now_ns):
            return "invalid_clock"
        for name, stamp in (("scan", pair.source_stamp_ns), ("imu", imu.source_stamp_ns)):
            if not 0 <= source_now_ns - stamp <= self.timing.source_age_ns:
                return "stale_or_future_" + name
        for name, stamp in (("scan", pair.receipt_ns), ("imu", imu.receipt_ns)):
            if not 0 <= receipt_now_ns - stamp <= self.timing.receipt_age_ns:
                return "stale_or_future_" + name + "_receipt"
        return None

    def observe(self, pair, imu=None, *, source_now_ns, receipt_now_ns):
        self._latest = self._view("invalid_pair")
        if type(pair) is not LaserPosePair:
            return self._hold("invalid_pair")
        if type(pair.context) is not FeedbackContext or not pair.context.valid():
            return self._hold("invalid_context")
        self._latest.update(context=asdict(pair.context))
        if self._context is not None and pair.context != self._context:
            self._context = pair.context
            return self._hold("context_changed")
        self._context = pair.context
        stamps = (pair.previous_sequence, pair.sequence, pair.previous_source_stamp_ns,
            pair.source_stamp_ns, pair.previous_receipt_ns, pair.receipt_ns)
        if not all(_stamp(v) for v in stamps):
            return self._hold("invalid_pair_metadata")
        self._latest.update(sequence=pair.sequence, previous_sequence=pair.previous_sequence,
            source_stamp_ns=pair.source_stamp_ns, receipt_ns=pair.receipt_ns)
        interval = pair.source_stamp_ns - pair.previous_source_stamp_ns
        if (interval <= 0 or pair.previous_receipt_ns >= pair.receipt_ns
                or pair.sequence != pair.previous_sequence + 1):
            return self._hold("nonconsecutive_pair")
        if not _pose(pair.previous_pose) or not _pose(pair.pose):
            return self._hold("invalid_pose_values")
        dx, dy = pair.pose[0]-pair.previous_pose[0], pair.pose[1]-pair.previous_pose[1]
        yaw = pair.pose[2]-pair.previous_pose[2]
        if not all(_number(v) for v in (dx, dy, yaw)):
            return self._hold("nonfinite_pose_delta")
        if abs(yaw) >= math.pi:
            return self._hold("ambiguous_or_wrapped_yaw_increment")
        c, s = math.cos(pair.previous_pose[2]), math.sin(pair.previous_pose[2])
        xy = (c*dx+s*dy, -s*dx+c*dy)
        try:
            _, lx, ly = _laser_to_base_twist(xy, yaw, interval,
                MeasuredLaserMount(0., 0., 0., "diagnostic-only"))
        except (ValueError, OverflowError, ZeroDivisionError):
            return self._hold("nonfinite_laser_diagnostic")
        if not all(_number(v) for v in (*xy, lx, ly, yaw/(interval*1e-9))):
            return self._hold("nonfinite_laser_diagnostic")
        self._latest["laser"] = dict(frame=pair.context.laser_frame,
            pose_frame=pair.context.pose_frame, previous_source_stamp_ns=pair.previous_source_stamp_ns,
            previous_receipt_ns=pair.previous_receipt_ns, interval_ns=interval,
            delta_xy_m=list(xy), delta_yaw_rad=yaw, forward_m_s=lx, lateral_m_s=ly,
            registration_yaw_rad_s=yaw/(interval*1e-9), diagnostic_only=True,
            registration_accepted=pair.accepted is True,
            map_eligible=pair.map_eligible is True)
        if pair.path_break is not False or pair.motion_prior is not False:
            return self._hold("path_break_or_motion_prior")
        if pair.accepted is not True or pair.map_eligible is not True:
            return self._hold("rejected_or_ineligible_registration")
        if pair.source_clock_verified is not True:
            return self._hold("source_clock_unverified")
        if pair.independent_localization_validated is not True:
            return self._hold("localization_unvalidated")
        if pair.planar_supported is not True:
            return self._hold("planar_envelope_unvalidated")
        if type(self.mount) is not MeasuredLaserMount or not self.mount.valid():
            return self._hold("measured_mount_required")
        if type(self.timing) is not FeedbackTiming or not self.timing.valid():
            return self._hold("timing_envelope_required")
        if interval > self.timing.max_interval_ns:
            return self._hold("scan_interval_outside_envelope")
        if type(imu) is not NativeImuYaw or type(imu.context) is not FeedbackContext or not imu.context.valid() or imu.context != pair.context:
            return self._hold("native_imu_context_missing_or_changed")
        if not all(_stamp(v) for v in (imu.source_stamp_ns, imu.receipt_ns)):
            return self._hold("invalid_imu_metadata")
        if (imu.native is not True or imu.gravity_projected is not True
                or imu.mount_verified is not True or not _name(imu.imu_frame)
                or not _number(imu.yaw_rad_s)):
            return self._hold("native_gravity_yaw_unverified")
        self._latest.update(imu_source_stamp_ns=imu.source_stamp_ns, imu_receipt_ns=imu.receipt_ns)
        if not pair.previous_source_stamp_ns < imu.source_stamp_ns <= pair.source_stamp_ns:
            return self._hold("imu_not_causal_in_scan_interval")
        if pair.source_stamp_ns - imu.source_stamp_ns > self.timing.imu_lag_ns:
            return self._hold("imu_pair_too_old")
        reason = self._freshness(pair, imu, source_now_ns, receipt_now_ns)
        if reason:
            return self._hold(reason)
        if self._last is not None:
            previous = self._last
            if (pair.previous_sequence != previous.sequence
                    or pair.previous_source_stamp_ns != previous.source_stamp_ns
                    or pair.previous_receipt_ns != previous.receipt_ns
                    or pair.previous_pose != previous.pose):
                return self._hold("discontinuous_original_pair")
            if (imu.source_stamp_ns <= self._imu_last.source_stamp_ns
                    or imu.receipt_ns <= self._imu_last.receipt_ns
                    or imu.imu_frame != self._imu_last.imu_frame
                    or source_now_ns < self._clocks[0] or receipt_now_ns <= self._clocks[1]):
                return self._hold("measurement_or_clock_reordered")
            if (self.mount, self.timing) != self._profile:
                return self._hold("mount_or_timing_changed")
        priming = self._last is None
        try:
            delta, vx, vy = _laser_to_base_twist(xy, yaw, interval, self.mount)
        except (ValueError, OverflowError, ZeroDivisionError):
            return self._hold("nonfinite_body_estimate")
        if not all(_number(v) for v in (*delta, vx, vy)):
            return self._hold("nonfinite_body_estimate")
        self._last, self._imu_last = pair, imu
        self._clocks = (source_now_ns, receipt_now_ns)
        self._profile = (self.mount, self.timing)
        if priming:
            return self._hold_priming()
        self._latest.update(valid=True, body_eligible_at_observation=True,
            state="OBSERVATION_AVAILABLE", reason="validated_interval_declarations",
            body=dict(frame="base_interval_body_twist", forward_m_s=vx, lateral_m_s=vy,
                yaw_rad_s=imu.yaw_rad_s, delta_xy_m=list(delta),
                calibration_id=self.mount.calibration_id, instantaneous_speed=False))
        return self.snapshot()

    def _hold_priming(self):
        # Keep only this qualified endpoint as a baseline; no velocity release.
        self._latest.update(reason="first_pair_priming", body=None)
        return self.snapshot()

    def snapshot(self, *, source_now_ns=None, receipt_now_ns=None):
        result = copy.deepcopy(self._latest)
        if source_now_ns is not None or receipt_now_ns is not None:
            reason = None
            if result["valid"]:
                if (self.mount, self.timing) != self._profile:
                    reason = "mount_or_timing_changed"
                else:
                    reason = self._freshness(self._last, self._imu_last, source_now_ns, receipt_now_ns)
                    if not reason and (source_now_ns < self._clocks[0] or receipt_now_ns < self._clocks[1]):
                        reason = "clock_regression"
            if reason:
                result.update(valid=False, state="HOLD", reason=reason, body=None)
                # A read may invalidate, never revive, the existing observation.
                # Historical body_eligible_at_observation remains explicit.
                self._latest = copy.deepcopy(result)
                self._last = self._imu_last = self._clocks = self._profile = None
            result["freshness_recheck_required"] = not result["valid"]
        return result
