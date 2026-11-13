"""Pure forward-only PWM proposal; never sends a command or grants authority.

All calibration, geometry and clock declarations are caller-supplied evidence,
not evidence verified by this module. No actual rover calibration is provided.
Feedback is native body velocity and yaw rate, never PWM-derived wheel speed.
The caller must serialize reset/step and independently validate the feedback
and swept-space decision. This is source preparation, not motor qualification.
"""
from dataclasses import dataclass, field
import math


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _ns(value):
    return type(value) is int and 0 <= value <= (1 << 63) - 1


def _name(value):
    return type(value) is str and 0 < len(value) <= 128 and value.strip() == value


@dataclass(frozen=True)
class NativeContext:
    source_session: str
    epoch: str
    generation: int
    source_clock: str
    receipt_clock: str
    calibration_id: str

    def valid(self):
        return all(_name(v) for v in (self.source_session, self.epoch,
            self.source_clock, self.receipt_clock, self.calibration_id)) and _ns(self.generation)


@dataclass(frozen=True)
class MeasuredProfile:
    """No usable default. Units are m/s, rad/s and PWM percentage points.

    FF/Kp units: percentage points per m/s or per rad/s. Slew is percentage
    points/s. Measurement flags/IDs are declarations, not a certificate.
    Braking measurements must cover the current body and requested speed.
    """
    calibration_id: str = ""
    measured: bool = False
    braking_measured: bool = False
    linear_ff: float = None
    yaw_ff: float = None
    linear_kp: float = None
    yaw_kp: float = None
    max_forward_m_s: float = None
    max_feedback_m_s: float = None
    max_yaw_rad_s: float = None
    max_lateral_m_s: float = None
    max_percent: float = None
    slew_percent_per_s: float = None
    braking_test_speed_m_s: float = None
    stop_distance_m: float = None
    stop_time_s: float = None

    def valid(self):
        positive = (self.linear_ff, self.max_forward_m_s, self.max_feedback_m_s,
            self.max_yaw_rad_s, self.max_lateral_m_s, self.max_percent,
            self.slew_percent_per_s, self.braking_test_speed_m_s,
            self.stop_distance_m, self.stop_time_s)
        gains = (self.yaw_ff, self.linear_kp, self.yaw_kp)
        return (self.measured is True and self.braking_measured is True
            and _name(self.calibration_id)
            and all(_finite(v) and v > 0 for v in positive)
            and all(_finite(v) and v >= 0 for v in gains)
            and self.max_percent <= 100
            and self.max_feedback_m_s >= self.max_forward_m_s
            and self.braking_test_speed_m_s >= self.max_forward_m_s)


@dataclass(frozen=True)
class TimingLimits:
    """Explicit validated clock/latency envelope; no guessed field TTL."""
    feedback_age_ns: int
    swept_age_ns: int
    max_step_ns: int

    def valid(self):
        return all(_ns(v) and v > 0 for v in
            (self.feedback_age_ns, self.swept_age_ns, self.max_step_ns))


@dataclass(frozen=True)
class NativeFeedback:
    context: NativeContext
    source_stamp_ns: int
    receipt_ns: int
    forward_m_s: float
    lateral_m_s: float
    yaw_rad_s: float
    independently_valid: bool


@dataclass(frozen=True)
class SweptSpace:
    """Current source-bound decision for this exact forward command.

    CLEAR must cover the measured body/extrinsics, unknown-space policy and
    curved swept envelope, not just a front ray. No planner is implemented here.
    """
    context: NativeContext
    source_stamp_ns: int
    scan_receipt_ns: int
    decision_receipt_ns: int
    decision: str
    body_geometry_verified: bool
    forward_m_s: float
    yaw_rad_s: float
    clear_distance_m: float


@dataclass(frozen=True)
class Preview:
    left_percent: float
    right_percent: float
    reason: str
    latched: bool
    capped: bool = False
    slew_limited: bool = False
    motor_command_sent: bool = field(default=False, init=False)
    runtime_authority: bool = field(default=False, init=False)


class EncoderlessDrivePreview:
    def __init__(self, profile=None, timing=None):
        self.profile = profile
        self.timing = timing
        self._context = None
        self._last = None
        self._output = (0., 0.)
        self._fault = "reset_required"
        self._reset_profile = None
        self._reset_timing = None

    @property
    def context(self):
        return self._context

    def _stop(self, reason):
        self._fault = reason
        self._output = (0., 0.)
        self._last = None
        return Preview(0., 0., reason, True)

    def stop(self, reason="explicit_stop"):
        return self._stop(reason if _name(reason) else "explicit_stop")

    def reset(self, context):
        """Explicitly clears a stop latch; does not propose motion."""
        if type(context) is not NativeContext or not context.valid():
            return self._stop("invalid_context")
        if type(self.profile) is not MeasuredProfile or not self.profile.valid():
            return self._stop("measured_profile_required")
        if type(self.timing) is not TimingLimits or not self.timing.valid():
            return self._stop("timing_envelope_required")
        if context.calibration_id != self.profile.calibration_id:
            return self._stop("calibration_context_mismatch")
        self._context = context
        self._reset_profile = self.profile
        self._reset_timing = self.timing
        self._fault = None
        self._last = None
        self._output = (0., 0.)
        return Preview(0., 0., "reset_zero", False)

    def step(self, feedback, swept, *, desired_forward_m_s, desired_yaw_rad_s,
             source_now_ns, receipt_now_ns, manual_takeover=False,
             hold=False, mode="forward"):
        if self._fault is not None:
            return Preview(0., 0., self._fault, True)
        if manual_takeover is not False:
            return self._stop("manual_takeover")
        if hold is not False:
            return self._stop("hold")
        if type(mode) is not str or mode != "forward":
            return self._stop("unsupported_motion_mode")
        if not all(_finite(v) for v in (desired_forward_m_s, desired_yaw_rad_s)):
            return self._stop("invalid_target")
        if desired_forward_m_s == 0:
            return self._stop("zero_speed")
        p = self.profile
        if type(p) is not MeasuredProfile or not p.valid():
            return self._stop("measured_profile_required")
        t = self.timing
        if type(t) is not TimingLimits or not t.valid():
            return self._stop("timing_envelope_required")
        if p != self._reset_profile or t != self._reset_timing:
            return self._stop("profile_or_timing_changed")
        if not 0 < desired_forward_m_s <= p.max_forward_m_s or abs(desired_yaw_rad_s) > p.max_yaw_rad_s:
            return self._stop("target_outside_forward_envelope")
        if type(feedback) is not NativeFeedback or type(swept) is not SweptSpace:
            return self._stop("native_feedback_or_space_missing")
        if (type(feedback.context) is not NativeContext or not feedback.context.valid()
                or type(swept.context) is not NativeContext or not swept.context.valid()):
            return self._stop("invalid_context")
        if feedback.context != self.context or swept.context != self.context:
            return self._stop("context_changed")
        if self.context.calibration_id != p.calibration_id:
            return self._stop("calibration_context_mismatch")
        stamps = (source_now_ns, receipt_now_ns, feedback.source_stamp_ns,
            feedback.receipt_ns, swept.source_stamp_ns, swept.scan_receipt_ns,
            swept.decision_receipt_ns)
        if not all(_ns(v) for v in stamps):
            return self._stop("invalid_clock_metadata")
        if swept.source_stamp_ns != feedback.source_stamp_ns or swept.scan_receipt_ns != feedback.receipt_ns:
            return self._stop("swept_feedback_endpoint_mismatch")
        if not feedback.receipt_ns <= swept.decision_receipt_ns <= receipt_now_ns:
            return self._stop("invalid_decision_order")
        if not 0 <= source_now_ns - feedback.source_stamp_ns <= t.feedback_age_ns or not 0 <= receipt_now_ns - feedback.receipt_ns <= t.feedback_age_ns:
            return self._stop("feedback_stale_or_future")
        if (not 0 <= source_now_ns - swept.source_stamp_ns <= t.swept_age_ns
                or not 0 <= receipt_now_ns - swept.scan_receipt_ns <= t.swept_age_ns
                or not 0 <= receipt_now_ns - swept.decision_receipt_ns <= t.swept_age_ns):
            return self._stop("swept_space_stale_or_future")
        values = (feedback.forward_m_s, feedback.lateral_m_s, feedback.yaw_rad_s,
            swept.forward_m_s, swept.yaw_rad_s, swept.clear_distance_m)
        if not all(_finite(v) for v in values):
            return self._stop("nonfinite_feedback_or_space")
        if feedback.independently_valid is not True:
            return self._stop("native_feedback_unconfirmed")
        if type(swept.decision) is not str or swept.decision != "CLEAR" or swept.body_geometry_verified is not True:
            return self._stop("swept_space_not_clear")
        if (swept.forward_m_s, swept.yaw_rad_s) != (desired_forward_m_s, desired_yaw_rad_s):
            return self._stop("swept_target_mismatch")
        if not 0 <= feedback.forward_m_s <= p.max_feedback_m_s or abs(feedback.lateral_m_s) > p.max_lateral_m_s or abs(feedback.yaw_rad_s) > p.max_yaw_rad_s:
            return self._stop("feedback_outside_measured_envelope")
        if feedback.forward_m_s > p.braking_test_speed_m_s:
            return self._stop("speed_above_braking_test")
        # Measured distance plus motion during the older source/receipt age.
        age_s = max(source_now_ns - feedback.source_stamp_ns,
            receipt_now_ns - feedback.receipt_ns) / 1e9
        clearance = p.stop_distance_m + max(desired_forward_m_s, feedback.forward_m_s) * age_s
        if not _finite(clearance) or swept.clear_distance_m < clearance:
            return self._stop("insufficient_braking_space")
        previous = self._last
        current = (feedback.source_stamp_ns, feedback.receipt_ns,
            source_now_ns, receipt_now_ns, swept.decision_receipt_ns)
        if previous is not None:
            if (current[0] <= previous[0] or current[1] <= previous[1]
                    or current[2] < previous[2] or current[3] <= previous[3]
                    or current[4] <= previous[4]):
                return self._stop("out_of_order")
            if current[0] - previous[0] > t.max_step_ns or current[3] - previous[3] > t.max_step_ns:
                return self._stop("control_or_source_gap")
        common = p.linear_ff * desired_forward_m_s + p.linear_kp * (desired_forward_m_s - feedback.forward_m_s)
        differential = p.yaw_ff * desired_yaw_rad_s + p.yaw_kp * (desired_yaw_rad_s - feedback.yaw_rad_s)
        raw = (common - differential, common + differential)
        if not all(_finite(v) for v in raw):
            return self._stop("nonfinite_proposal")
        if min(raw) <= 0:
            return self._stop("proposal_not_both_wheels_forward")
        capped = tuple(min(v, p.max_percent) for v in raw)
        # First sample is always zero; dt comes from control receipt time, not
        # sensor backlog. Stop bypasses this limiter and clears all history.
        dt = 0. if previous is None else (receipt_now_ns - previous[3]) / 1e9
        change = p.slew_percent_per_s * dt
        if not _finite(change):
            return self._stop("nonfinite_slew")
        output = tuple(old + max(-change, min(change, new - old))
            for old, new in zip(self._output, capped))
        self._last = current
        self._output = output
        return Preview(*output, "initial_sample_zero" if previous is None else "forward_preview",
            False, capped != raw, output != capped)
