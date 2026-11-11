"""Single-row centreline following with a row-end stop (SIM evaluation candidate; forward only, no pivot).

Goal (single-row goal note of 2026-11-02, kept outside the repository): drive ONE row between two adjacent tree lines along
the centreline, correct a straight-line drift that changes along the row in real time, slow down
before the row end and stop there. Headland turns, the next row, several rows and the last-row
decision are out of scope: next-row / turn-space / topology evidence is never a start condition here.

Input: SensorEstimator.snapshot() (2D LiDAR + raw IMU only; no truth, no PWM, no wheel speed, no GNSS,
no camera). Output: the same Command(left, right, state, reason) of signed integer duty percents as
RowController; outputs are always forward (0..100, never a pivot, never a reversal).

PWM is a command, not a speed: speed and yaw-rate feedback come from the sensors (LiDAR scan-matching
speed, gyro yaw rate); the arc gain and the speeds below are NOMINAL SIM values, not calibration.

FOLLOW (always-on continuous steering; no equal-duty deadband gaps, no stop -> pivot -> go):
  refs    the steering lateral offset is plausibility limited: it follows the estimate at most
          lateral_ref_slack_m_s faster than the dead-reckoned motion (measured speed x sin(heading
          error)) explains, so a band refit (end wall, re-lock after missing trunks) steers gradually;
          the heading error is used as estimated.
  outer   pure-pursuit desired yaw rate r_des = v x 2 sin(e) / hypot(lookahead, lateral), e = the lookahead
          error (heading + lateral at lookahead_m 1.5 m), v = the MEASURED LiDAR speed floored at
          speed_floor_m_s, clamped to +-max_yaw_rate_deg_s (<= 0.6 x the nominal authority 2 x arc gain x
          max differential, the executor feasibility rule). r_ref = r_des through a first-order lag
          (expected plant yaw + gyro filter response).
  inner   differential d = r_ref / (2 arc gain) + kp (r_ref - r_gyro) + b, b = a PERSISTENT bounded
          drift-bias estimate [% differential] integrated (ki) from the same yaw-rate error: two-sided (it
          may oppose the desired yaw; a drift is corrected whatever its sign), never reset on an error or
          output sign change, only by a new controller; integrated only while the vehicle is seen moving
          and frozen while the output is held below the demand in the error direction (clamp, slew,
          zero-crossing hold, side-swing rule: conditional integration anti-windup). kp and ki are scaled
          by min(1, gyro_trust_vibration_m_s2 / vibration RMS) (gyro noise grows with vibration).
  output  |d| <= min(max differential, 100 - base, base) (headroom: base <= 100 - max differential), slew
          limited (diff_slew_pct_s); against chatter only: a demand of the opposite sign smaller than
          sign_hysteresis_pct does not cross zero, and integer duties change with a small hysteresis
          (output_deadband_pct). Rolling start: equal duty in each drive segment until the sensors see
          the vehicle roll (a differential at standstill would be a partial pivot).
  speed   base duty adapted from the observed LiDAR speed toward the target within [base_min_pct,
          base_max_pct] (asymmetric slew: down faster than up; no duty-speed model). Target = cruise,
          limited so the pure-pursuit curvature fits the yaw-rate clamp (curvature_speed_margin), reduced
          toward min_speed as |lateral| nears the safety band, authority_slow_speed while the authority
          monitor reports SLOW and approach_speed in the row-end slow zone (more steering per metre).
  authority  saturated (demand >= limit) for authority_sat_s while the lookahead error does not shrink
          by authority_progress_deg -> SLOW until unsaturated for authority_release_s. |lateral| (the
          plausibility-limited reference) above the safety band (lateral_stop_fraction x the free half
          width of the corridor) or |heading error| above heading_limit_deg for authority_confirm_s ->
          stop 'steering_authority_insufficient' (CONTROL_AUTHORITY): software does not claim to fix a
          physical authority shortfall.
Row end: row_end_ahead_m is the estimator's two-sided end (both lines' support ended with observed
  free space beyond, temporally confirmed, fresh); time, distance, GNSS or one missing tree never decide
  it. remaining = row_end_ahead_m - end_stop_offset_m (the base stops end_stop_offset_m before the
  observed end of the row support: with the default half body length the FRONT stops at the observed
  end). remaining <= approach_start_m + stopping_distance(v) -> phase APPROACH_END (latched; released
  only after approach_release_s without any end evidence); remaining <= stopping_distance(v) -> stop
  'row_end'. The controller state stays FOLLOW during the approach (the executor accepts output only
  in FOLLOW/REDUCE); the phase is reported by telemetry().
Front blocked (front clearance below the stop model) -> 'end_structure_ahead' when end evidence lies
  at the blocking return (the two-sided end, or each side's support either ending within end_structure_before_m
  before it or ended with observed free space, and no side continuing beyond end_structure_beyond_m past it),
  else 'obstacle_ahead'. A structure the sensors cannot distinguish from a mid-row wall (both lines visibly
  end at it) is END_STRUCTURE, never ROW_END.
Kept from RowController unchanged: freshness (stale -> recoverable HOLD), sensor_missing, corridor_lost
  (stop, wait, reacquire or HOLD), side-clearance rule (side_blocked / swing inhibit, ON by default
  here), drive_no_response, stop_not_observed, stop repeat cap, the stop model. Every stop except the
  corridor wait and the stale pause is terminal: no automatic turn and no restart after it.
Start: like REACQUIRE 'start' (corridor valid, fresh, sensor-still, front and side clear), with the
  standstill held start_still_s (1.5 s; the estimator's gyro bias settles while still) and refused
  ('start_misaligned', terminal) when the lookahead error at start_lookahead_m (3 m) or the heading error
  exceeds start_max_deg (25 deg, the executor's geometry guard) or |lateral| exceeds the safety band; the
  same check after a corridor wait / stale recovery gives 'reacquire_misaligned'.
All thresholds are SIM tuning unless their comment cites evidence; nothing here is a field
calibration or a real-vehicle validation.
"""
from dataclasses import dataclass, fields
import math

from .controllers import Command, ControlConfig, RowController


CATEGORIES = ('ROW_END', 'OBSTACLE', 'END_STRUCTURE', 'PERCEPTION_OR_LOCALISATION', 'CONTROL_AUTHORITY', 'DRIVE',
              'OPERATOR_OR_BACKSTOP', 'SENSOR', 'SYSTEM_FAULT')
# Terminal HOLD reasons this controller can produce (RowController's reasons that need a pivot, a turn or
# REDUCE cannot occur: no TURN/REDUCE/SETTLE state is ever entered).
TERMINAL_HOLDS = ('row_end', 'obstacle_ahead', 'end_structure_ahead', 'steering_authority_insufficient',
                  'start_misaligned', 'reacquire_misaligned', 'side_blocked', 'corridor_not_acquired',
                  'corridor_not_reacquired', 'drive_no_response', 'sensor_missing', 'stop_not_observed',
                  'stop_repeat_limit', 'stop_without_decision')
PHASES = ('START', 'FOLLOW', 'APPROACH_END', 'STOPPING', 'WAIT', 'STOPPED')

# Every terminal reason -> one stop category. Sections: this controller, RowController (so one function
# classifies both in comparisons), and the executor Runner/Backstop/geometry-guard end reasons
# (tools/row_executor core.py / guard.py, read-only reference for a later integration).
STOP_CATEGORIES = {
    # --- SingleRowController -------------------------------------------------------------------
    'row_end': 'ROW_END',
    'obstacle_ahead': 'OBSTACLE',
    'side_blocked': 'OBSTACLE',                     # hard side margin on distinct scans (in-row obstacle)
    'end_structure_ahead': 'END_STRUCTURE',
    'corridor_not_acquired': 'PERCEPTION_OR_LOCALISATION',
    'corridor_not_reacquired': 'PERCEPTION_OR_LOCALISATION',
    'corridor_lost': 'PERCEPTION_OR_LOCALISATION',
    'stop_repeat_limit': 'PERCEPTION_OR_LOCALISATION',   # only corridor-lost waits are non-terminal stops here
    'steering_authority_insufficient': 'CONTROL_AUTHORITY',
    'start_misaligned': 'CONTROL_AUTHORITY',        # beyond what forward-only steering may correct
    'reacquire_misaligned': 'CONTROL_AUTHORITY',
    'drive_no_response': 'DRIVE',
    'stop_not_observed': 'DRIVE',                   # zero commanded, the sensors did not see the stop
    'stop_without_decision': 'OPERATOR_OR_BACKSTOP',   # fail-closed internal fallback (unreachable): a person decides
    'sensor_missing': 'SENSOR',
    'stale_lidar': 'SENSOR',
    'stale_imu': 'SENSOR',
    # --- RowController (comparison runs) ----------------------------------------------------------
    'row_end_candidate': 'ROW_END',
    'front_blocked': 'OBSTACLE',
    'pivot_clearance_insufficient': 'OBSTACLE',
    'pivot_stall_abort': 'DRIVE',
    'turn_timeout': 'DRIVE',
    'settle_timeout': 'DRIVE',
    'stop_turn_repeat_limit': 'CONTROL_AUTHORITY',
    'turn_not_permitted': 'CONTROL_AUTHORITY',
    # --- executor Runner / Backstop / geometry guard (end reasons) --------------------------------
    'row_distance_backstop': 'OPERATOR_OR_BACKSTOP',
    'row_time_backstop': 'OPERATOR_OR_BACKSTOP',
    'operator_end': 'OPERATOR_OR_BACKSTOP',
    'hold_released': 'OPERATOR_OR_BACKSTOP',
    'backstop_odometry_invalid': 'PERCEPTION_OR_LOCALISATION',
    'corridor_lost_repeat_limit': 'PERCEPTION_OR_LOCALISATION',
    'guard_heading': 'CONTROL_AUTHORITY',
    'guard_lateral': 'CONTROL_AUTHORITY',
    'drive_no_response_repeat': 'DRIVE',
    'stale_repeat_limit': 'SENSOR',
    'stall_guard_repeat_limit': 'SENSOR',
    'clock_regression': 'SENSOR',
    # --- executor Runner / ExecutorLoop pause and end reasons that were missing above (added 2026-11-02 before any
    # FREEZE, PREREG stop_categories rule: new entries only, no registered reason re-mapped; the same categories as
    # the executor's SINGLE_ROW_EXECUTOR_CATEGORIES display table) ---------------------------------------------
    'reacquire_needs_press': 'PERCEPTION_OR_LOCALISATION',   # corridor lost: re-acquisition waits for a new press
    'stall_guard': 'SENSOR',                                 # control-loop stall pause (its repeat limit is SENSOR)
    'operator_stop': 'OPERATOR_OR_BACKSTOP',                 # page / dashboard STOP
    'native_cross_check_cost_occupied': 'OBSTACLE',          # optional native cross-check veto (executor_main)
    'bridge_error': 'SYSTEM_FAULT',
    'bridge_latched': 'SYSTEM_FAULT',
    'bridge_arm_failed': 'SYSTEM_FAULT',
    'shutdown': 'SYSTEM_FAULT',
    'loop_exception': 'SYSTEM_FAULT',
}
# Prefix rules for parameterised executor reasons ('guard_violation:<rule>' = an output the OutputGuard
# refused: the command could not be executed inside the allowed duty envelope).
STOP_CATEGORY_PREFIXES = (('guard_violation:', 'CONTROL_AUTHORITY'),)


def stop_category(reason):
    """Stop category of a terminal reason, or None for a reason that is not a known stop (e.g. 'FOLLOW')."""
    if reason in STOP_CATEGORIES:
        return STOP_CATEGORIES[reason]
    for prefix, category in STOP_CATEGORY_PREFIXES:
        if isinstance(reason, str) and reason.startswith(prefix):
            return category
    return None


@dataclass(frozen=True)
class SingleRowConfig(ControlConfig):
    # --- inherited fields with single-row defaults ------------------------------------------------
    mode: str = 'continuous'           # forward-only; no pivot, no REDUCE
    side_clearance_check: bool = True  # the opt-in side rule is ON in single-row mode
    base_duty_pct: int = 72            # START base; field1010 plant: (72 - 50) / 40 x 0.72 = 0.40 m/s (SIM_ASSUMED
                                       # deadband 50 %) at run scale 1; adapted by the speed loop afterwards
    max_yaw_rate_deg_s: float = 3.6    # 0.6 x nominal authority 2 x 0.12 x 25 = 6.0 deg/s (executor feasibility rule)
    yaw_kp_pct_per_deg_s: float = .5   # small: the 20 Hz gyro carries ~0.8 deg/s noise after the 0.25 s estimator
                                       # low-pass at the SIM_ASSUMED 1.7 m/s^2 drive vibration; ff + bias do the work
    yaw_ki_pct_per_deg: float = 2.     # bias time constant ~ (1 + 2 g kp) / (2 g ki) = 2.3 s at the nominal 0.12 gain
                                       # (0.9 m at 0.4 m/s), well below the ~8 m drift correlation (ESTIMATED wander);
                                       # 4 doubled the gyro-noise driven sign flips on the dev seeds
    lookahead_m: float = 1.5           # pure pursuit: lateral natural frequency sqrt(2) / L per metre; dev seeds 0-3:
                                       # lateral RMSE (per metre) 3 m 0.118, 2 m 0.087, 1.5 m 0.073 (plain 0.091)
    # --- speed loop (PWM is not speed: base duty adapted from the observed LiDAR speed) --------------
    cruise_speed_m_s: float = .40
    min_speed_m_s: float = .20         # floor of the error-based reduction
    approach_speed_m_s: float = .20    # row-end slow zone
    base_min_pct: int = 60             # field1010: 0.18 m/s at run scale 1 (above the SIM_ASSUMED 50 % deadband)
    base_max_pct: int = 75             # + max differential 25 = 100: no differential headroom lost to the 100 % cap
    speed_gain_pct_per_m_s: float = 40.    # base duty rate [%/s] per m/s of speed error (~1.4 s loop at the
                                           # field1010 0.018 m/s per % slope; ~2.7 s at P_fit1003 0.0094)
    base_slew_up_pct_s: float = 5.
    base_slew_down_pct_s: float = 15.  # slowing down (approach, large error) is the safe direction
    speed_deadband_m_s: float = .02    # LiDAR speed noise (median of 5 scan-to-scan speeds)
    curvature_speed_margin: float = .8 # target <= margin x max yaw rate / |pure-pursuit curvature|: slow down only
                                       # when the curvature demand would hit the yaw-rate clamp at the current speed
    slow_lateral_start: float = .4     # target reduced from cruise to min_speed between these fractions
    slow_lateral_full: float = .8      # of the lateral safety band
    # --- steering ------------------------------------------------------------------------------------
    speed_floor_m_s: float = .15       # pure pursuit speed floor (steers from the first metre)
    rolling_min_speed_m_s: float = .05     # rolling start: equal duty until the sensors see the vehicle roll (not
                                           # still, LiDAR speed above this) in each drive segment. A differential
                                           # before that is a partial pivot (no pivots in this mode), and the
                                           # estimator still learns its gyro bias while it reports stillness
                                           # (~0.2-0.4 s into the drive): a yaw commanded then would enter that bias
    yaw_reference_tau_s: float = .35   # expected response lag: plant yaw 0.12 s (ESTIMATED) + gyro low-pass 0.25 s;
                                       # the feed-forward also uses this lagged reference (less estimate noise)
    lateral_ref_slack_m_s: float = .10     # plausibility: the steering lateral follows the estimate at most this
                                           # much faster than the measured motion explains (dead reckoning). Dev
                                           # seeds: normal 1.5 s estimate-vs-dead-reckoning disagreement 0.05-0.16 m,
                                           # end-wall band refit 0.62 m (narrow_headland seed 3: CONTROL_AUTHORITY
                                           # stop without it). A heading-axis limit in the gyro-integral frame was
                                           # tried and removed (gyro random walk at high vibration: imu_noise_vibration
                                           # lateral RMSE 0.058 -> 0.075 m)
    diff_slew_pct_s: float = 30.       # 1.5 %/tick at 20 Hz; the MCU ramps 100 %/s per side (MEASURED)
    output_deadband_pct: float = .3    # integer-duty hysteresis: a side changes only when > 0.5 + this away
    sign_hysteresis_pct: float = 1.5   # zero-crossing hold: a demand of the opposite sign to the last output stays
                                       # 0 (equal duty) until it reaches this (R - L 3 %, the harness' own threshold of
                                       # a meaningful differential); dev seeds 0-3: sign flips -27 %, tracking equal
    bias_max_pct: float = 15.          # 0.6 x max differential: the drift estimate never takes all the authority
    gyro_trust_vibration_m_s2: float = 1.7     # kp and ki x min(1, this / vibration RMS): the 20 Hz gyro noise grows
                                               # with vibration (ESTIMATED 1.5 deg/s per m/s^2, 10/10 records); 1.7 =
                                               # the SIM drive level the gains were tuned at (0 = off)
    vibration_tau_s: float = 1.        # low-pass of the snapshot's 0.5 s vibration RMS for that scaling
    bias_min_speed_m_s: float = .08    # integrate only while the sensors see the vehicle move
    # --- authority monitor -------------------------------------------------------------------------
    authority_sat_s: float = 1.5       # >= MCU ramp 1.0 s (MEASURED) + yaw response
    authority_progress_deg: float = 1.
    authority_release_s: float = 2.
    authority_slow_speed_m_s: float = .20
    lateral_stop_fraction: float = .5  # of the free half width (corridor width / 2 - footprint width / 2)
    lateral_stop_min_m: float = .20
    heading_limit_deg: float = 30.     # above start_max_deg (a 25 deg start is accepted)
    authority_confirm_s: float = .3
    start_max_deg: float = 25.         # = executor geometry guard_heading_deg (C1 gate P3)
    start_still_s: float = 1.5         # initial start: sensor-observed stillness this long (re-acquisition keeps
                                       # still_confirm_s 0.4). The estimator learns its gyro bias only while it reports
                                       # stillness (from 0.5 s) and keeps learning ~0.2-0.4 s into the drive (lag):
                                       # the extra samples dilute the first drive-vibration samples (dev case above:
                                       # 6 still + 4 vibrating samples -> 1.7 deg/s bias error)
    start_lookahead_m: float = 3.      # lookahead of the start check (RowController/executor default), independent
                                       # of the steering lookahead: the same pose gives the same start decision
    # --- row end / front classification ----------------------------------------------------------
    end_stop_offset_m: float = .61     # base stops this far before the observed end: the front (half the
                                       # CATALOGUE 1.22 m footprint) stops at the observed end of the support
    approach_start_m: float = 1.5      # slow zone begins this + stopping distance before the stop point
    approach_release_s: float = 3.     # end evidence absent this long -> approach released (a false end)
    end_structure_before_m: float = 3.     # support of both lines ends at most this far before the blocking return
    end_structure_beyond_m: float = .5     # ... and does not continue more than this beyond it
    end_evidence_max_age_s: float = 2.5    # = EstimatorConfig.row_end_valid_s
    footprint_length_m: float = 1.22   # NOMINAL (CATALOGUE; = EstimatorConfig): front classification only
    footprint_width_m: float = .70     # NOMINAL (CATALOGUE): lateral safety band only

    def __post_init__(self):
        super().__post_init__()
        if self.mode != 'continuous' or self.pivot_failure_fallback:
            raise ValueError('single_row_requires_continuous_mode')
        if self.row_end_stop_m != 0.:
            raise ValueError('single_row_uses_end_stop_offset_m_not_row_end_stop_m')
        if not (0 <= self.base_min_pct <= self.base_duty_pct <= self.base_max_pct
                and self.base_max_pct + self.max_differential_pct <= 100
                and self.base_min_pct - self.max_differential_pct >= 0):
            raise ValueError('single_row_base_range_invalid')
        authority = 2 * self.arc_yaw_deg_s_per_pct_diff * self.max_differential_pct
        if not 0 < self.max_yaw_rate_deg_s <= .6 * authority + 1e-9:
            raise ValueError('single_row_max_yaw_rate_above_0p6_authority')
        speeds = (self.cruise_speed_m_s, self.min_speed_m_s, self.approach_speed_m_s, self.speed_floor_m_s,
                  self.authority_slow_speed_m_s, self.bias_min_speed_m_s, self.rolling_min_speed_m_s)
        if not (all(math.isfinite(v) and v > 0 for v in speeds) and self.min_speed_m_s <= self.cruise_speed_m_s
                and self.approach_speed_m_s <= self.cruise_speed_m_s):
            raise ValueError('single_row_speeds_invalid')
        positive = (self.speed_gain_pct_per_m_s, self.base_slew_up_pct_s, self.base_slew_down_pct_s,
                    self.diff_slew_pct_s, self.yaw_reference_tau_s, self.authority_sat_s, self.authority_release_s,
                    self.lateral_stop_fraction, self.heading_limit_deg, self.start_max_deg, self.approach_start_m,
                    self.approach_release_s, self.footprint_length_m, self.footprint_width_m,
                    self.start_lookahead_m, self.lateral_ref_slack_m_s, self.vibration_tau_s, self.start_still_s)
        if not all(math.isfinite(v) and v > 0 for v in positive):
            raise ValueError('single_row_parameter_not_positive')
        if not (0 <= self.bias_max_pct <= self.max_differential_pct and self.yaw_kp_pct_per_deg_s >= 0
                and self.yaw_ki_pct_per_deg >= 0 and 0 <= self.output_deadband_pct < .5
                and 0 <= self.sign_hysteresis_pct <= self.max_differential_pct / 2
                and self.gyro_trust_vibration_m_s2 >= 0):
            raise ValueError('single_row_inner_loop_invalid')
        if not (self.start_max_deg < self.heading_limit_deg and 0 < self.curvature_speed_margin <= 1
                and 0 <= self.slow_lateral_start < self.slow_lateral_full <= 1):
            raise ValueError('single_row_slowdown_or_limits_invalid')
        if not (self.end_stop_offset_m >= 0 and self.end_structure_before_m >= 0 and self.end_structure_beyond_m >= 0
                and self.authority_confirm_s >= 0 and self.authority_progress_deg >= 0
                and self.lateral_stop_min_m >= 0 and self.end_evidence_max_age_s > 0):
            raise ValueError('single_row_row_end_or_authority_invalid')


_CONTROL_DEFAULTS = ControlConfig()
_CONTROL_NAMES = tuple(f.name for f in fields(ControlConfig))


def make_single_row_config(**overrides):
    """SingleRowConfig with the single-row defaults and the given overrides (validated)."""
    return SingleRowConfig(**overrides)


def single_row_config_from(control, **overrides):
    """A SingleRowConfig from a continuous-mode ControlConfig (e.g. harness config_for('continuous'[_side])).

    Only the fields the caller CHANGED from the ControlConfig defaults are carried over (the remaining
    ControlConfig defaults belong to RowController's tuning, e.g. base 75 / max yaw 4.5); the others take
    the single-row defaults. side_clearance_check stays ON unless explicitly overridden here."""
    if isinstance(control, SingleRowConfig):
        return SingleRowConfig(**{**{f.name: getattr(control, f.name) for f in fields(SingleRowConfig)}, **overrides})
    if not isinstance(control, ControlConfig):
        raise TypeError('single_row_needs_a_control_config')
    if control.mode != 'continuous':
        raise ValueError('single_row_requires_continuous_mode')
    carried = {name: getattr(control, name) for name in _CONTROL_NAMES
               if name != 'mode' and getattr(control, name) != getattr(_CONTROL_DEFAULTS, name)}
    return SingleRowConfig(**{**carried, **overrides})


def _ramp(value, start, full):
    """0 at or below start, 1 at or above full, linear between."""
    if value <= start:
        return 0.
    if value >= full:
        return 1.
    return (value - start) / (full - start)


def _clamp(value, low, high):
    return max(low, min(high, value))


class SingleRowController(RowController):
    """Forward-only single-row follower (see the module docstring). Same step()/Command interface."""

    def __init__(self, config=None):
        config = SingleRowConfig() if config is None else single_row_config_from(config)
        super().__init__(config)
        c = config
        self.phase = 'FOLLOW'          # FOLLOW substate: FOLLOW | APPROACH_END (state stays 'FOLLOW')
        self.phase_events = []         # (t, phase/authority event, detail): diagnostics only
        self.base_duty = float(c.base_duty_pct)
        self.bias_pct = 0.             # persistent drift estimate [% differential]; reset only by a new controller
        self.r_ref = 0.                # reference-model yaw rate [deg/s]
        self.authority_state = 'OK'
        self.stop_category = None
        self.target_speed = c.cruise_speed_m_s
        self.end_seen_at = None
        self.counts.update(approach_entries=0, approach_releases=0, authority_slow_entries=0, follow_ticks=0,
                           saturated_ticks=0, slew_limited_ticks=0, bias_frozen_ticks=0)
        self._reset_drive()
        self._telemetry = {}

    # --- state helpers ----------------------------------------------------------------------------------
    def _reset_drive(self):
        """Per-drive-segment state (a stop ends the segment); the bias and the base duty persist."""
        self.differential = 0.
        self.r_ref = 0.
        self._last_drive_t = None
        self._out = [None, None]
        self._out_sign = 0
        self._sat_since = self._sat_error0 = self._unsat_since = None
        self._over_since = None
        self._lateral_ref = None       # plausibility-limited steering lateral offset
        self._rolling = False          # rolling-start latch of this drive segment
        self._vibration = None
        self._ref_t = None
        self._ref_heading_error = self._ref_speed = 0.

    def _reset_follow(self):
        super()._reset_follow()
        if hasattr(self, 'bias_pct'):
            self._reset_drive()

    def _hold(self, reason):
        command = super()._hold(reason)
        if self.hold_terminal:
            self.stop_category = stop_category(reason) or 'OPERATOR_OR_BACKSTOP'   # fail closed: a person decides
        return command

    def _stop(self, pending, reason):
        command = super()._stop(pending, reason)
        if self.pending and self.pending[0] == 'hold':
            self.stop_category = stop_category(self.pending[1])
        return command

    def _phase_event(self, name, detail=''):
        self.phase_events.append((round(self.now, 3), name, detail))

    def current_phase(self):
        if self.state == 'HOLD':
            return 'STOPPED' if self.hold_terminal else 'WAIT'
        if self.state in ('STOP_REQUESTED', 'STOP_OBSERVED'):
            return 'WAIT' if self.pending and self.pending[0] == 'wait_corridor' else 'STOPPING'
        if self.state == 'REACQUIRE':
            return 'START' if self.reason == 'start' else 'WAIT'
        return self.phase

    # --- geometry helpers ---------------------------------------------------------------------------
    def lateral_limit(self, s):
        """Lateral safety band [m] from the tracked corridor width (inf without a width)."""
        c = self.config
        width = s.get('corridor_width')
        if width is None or not math.isfinite(width):
            return math.inf
        free_half = max(0., width / 2 - c.footprint_width_m / 2)
        return max(c.lateral_stop_min_m, c.lateral_stop_fraction * free_half)

    def _end_remaining(self, s):
        """Distance from the base to the stop point (row end - end_stop_offset_m), None without an end."""
        end = s.get('row_end_ahead_m')
        if end is None or not math.isfinite(end):
            return None
        return end - self.config.end_stop_offset_m

    def front_reason(self, s):
        """Classify a blocked front: 'end_structure_ahead' with end evidence at the return, else 'obstacle_ahead'."""
        c = self.config
        obstacle = s['front_clearance_m'] + c.footprint_length_m / 2      # along distance from the base
        end = s.get('row_end_ahead_m')
        if end is not None and math.isfinite(end) and end <= obstacle + c.end_structure_beyond_m:
            return 'end_structure_ahead'
        ends = s.get('side_ends_ahead_m') or (None, None)
        free = s.get('side_end_free') or (False, False)
        age = s.get('end_age_s')
        # Each line must have ended: its support stops within end_structure_before_m before the return, or it ended
        # with observed free space (per side: staggered row ends, one row ended earlier in the open and the other
        # runs up to the structure - dev round 1, 035952 rev wall_1p5 classified as an in-row obstacle). No line
        # continues beyond the return (max(ends) <= obstacle + end_structure_beyond_m).
        if (len(ends) == 2 and all(e is not None and math.isfinite(e) for e in ends)
                and age is not None and age <= c.end_evidence_max_age_s
                and max(ends) <= obstacle + c.end_structure_beyond_m
                and all(f or e >= obstacle - c.end_structure_before_m for e, f in zip(ends, free))):
            return 'end_structure_ahead'
        return 'obstacle_ahead'

    def _misaligned(self, s):
        """Start / re-acquisition refusal: lookahead error at start_lookahead_m or heading above start_max_deg,
        or the lateral offset outside the safety band (raw estimates, no filter)."""
        c = self.config
        error = s['heading_error'] - math.atan2(s['lateral_offset'], c.start_lookahead_m)
        return (abs(math.degrees(math.atan2(math.sin(error), math.cos(error)))) > c.start_max_deg
                or abs(math.degrees(s['heading_error'])) > c.start_max_deg
                or abs(s['lateral_offset']) > self.lateral_limit(s))

    def _update_references(self, s):
        """Plausibility-limited steering lateral offset; the heading error is used as estimated.

        The tree lines are fixed in the world and the vehicle moves continuously, so the estimated lateral
        offset may change only by the dead-reckoned motion (measured speed x sin(heading error)) plus
        lateral_ref_slack_m_s. A faster change of the estimate (a band refit at an end wall, a re-lock after
        missing trunks) is followed only gradually. Initialised to the raw estimate at every start of a
        drive segment (after a stop the vehicle starts again from the raw estimate)."""
        c = self.config
        lateral, heading = s['lateral_offset'], s['heading_error']
        if self._lateral_ref is None:
            self._lateral_ref = lateral
        else:
            dt = _clamp(self.now - self._ref_t, 0., .2)
            predicted = self._lateral_ref - self._ref_speed * dt * math.sin(self._ref_heading_error)
            slack = c.lateral_ref_slack_m_s * dt
            self._lateral_ref = _clamp(lateral, predicted - slack, predicted + slack)
        self._ref_heading_error = heading
        speed = s.get('speed')
        self._ref_speed = max(0., speed) if speed is not None and math.isfinite(speed) else 0.
        self._ref_t = self.now
        return self._lateral_ref, self._ref_heading_error

    def steering_error(self, lateral, heading_error):
        """Lookahead error [rad] from the (filtered) lateral offset and heading error; side bias (opt-in) included."""
        c = self.config
        bias = self.side_bias_m if c.side_bias_max_m > 0 else 0.
        error = heading_error - math.atan2(lateral - bias, c.lookahead_m)
        return math.atan2(math.sin(error), math.cos(error))

    # --- states --------------------------------------------------------------------------------------
    def _reacquire(self, s):
        c = self.config
        initial = self.reason == 'start'
        if self.now - self.since > c.reacquire_timeout_s:
            return self._hold('corridor_not_acquired' if initial else 'corridor_not_reacquired')
        if not s['corridor_valid'] or s['corridor_age_s'] > .6 or not self._still(s):
            return self._zero()
        if initial and s.get('still_s', 0.) < c.start_still_s:
            return self._zero()
        if not initial and s.get('corridor_updates', 0) - self.reacquire_updates0 < c.reacquire_updates:
            return self._zero()
        if self._misaligned(s):
            return self._hold('start_misaligned' if initial else 'reacquire_misaligned')
        if s['front_clearance_m'] < self.required_front(0.):
            return self._hold(self.front_reason(s))
        remaining = self._end_remaining(s)
        if remaining is not None and remaining <= self.stopping_distance(0.):
            return self._hold('row_end')
        if self._side_blocked():
            return self._side_stop(immediate=True)
        self._go('FOLLOW', 'corridor_acquired')
        return self._follow(s)

    def _follow(self, s):
        c = self.config
        if not s['corridor_valid'] or s['corridor_age_s'] > c.corridor_lost_s:
            return self._stop('wait_corridor', 'corridor_lost')
        lateral, heading_error = self._update_references(s)
        speed = max(0., s['speed'])
        if s['front_clearance_m'] < self.required_front(speed):
            return self._stop('hold', self.front_reason(s))
        remaining = self._end_remaining(s)
        if remaining is not None and remaining <= self.stopping_distance(speed):
            return self._stop('hold', 'row_end')
        if self._side_blocked():
            return self._side_stop(immediate=False)
        if self._drive_no_response(s):
            return self._stop('hold', 'drive_no_response')
        if self._authority_exceeded(s, lateral, heading_error):
            return self._stop('hold', 'steering_authority_insufficient')
        self._update_approach(remaining, speed)
        return self._drive(s, lateral, heading_error)

    # --- FOLLOW parts -------------------------------------------------------------------------------
    def _authority_exceeded(self, s, lateral, heading_error):
        """|lateral| beyond the safety band or |heading| beyond the limit (plausibility-limited references),
        confirmed for authority_confirm_s."""
        c = self.config
        over = (abs(lateral) > self.lateral_limit(s) or abs(math.degrees(heading_error)) > c.heading_limit_deg)
        if not over:
            self._over_since = None
            return False
        if self._over_since is None:
            self._over_since = self.now
        return self.now - self._over_since >= c.authority_confirm_s

    def _update_approach(self, remaining, speed):
        c = self.config
        if remaining is not None:
            self.end_seen_at = self.now
            if self.phase != 'APPROACH_END' and remaining <= c.approach_start_m + self.stopping_distance(speed):
                self.phase = 'APPROACH_END'
                self.counts['approach_entries'] += 1
                self._phase_event('APPROACH_END', 'remaining_%.2f_m' % remaining)
        elif (self.phase == 'APPROACH_END' and self.end_seen_at is not None
              and self.now - self.end_seen_at > c.approach_release_s):
            self.phase = 'FOLLOW'
            self.counts['approach_releases'] += 1
            self._phase_event('APPROACH_RELEASED', 'no_end_evidence')

    def _target_speed(self, s, curvature, lateral):
        """Cruise, limited by the yaw-rate clamp for the curvature demand and by the lateral band, then by
        the authority SLOW state and the row-end slow zone (more steering per metre at a lower speed)."""
        c = self.config
        target = c.cruise_speed_m_s
        if abs(curvature) > 1e-9:
            target = min(target, c.curvature_speed_margin * math.radians(c.max_yaw_rate_deg_s) / abs(curvature))
        limit = self.lateral_limit(s)
        fraction = abs(lateral) / limit if math.isfinite(limit) and limit > 0 else 0.
        reduction = _ramp(fraction, c.slow_lateral_start, c.slow_lateral_full)
        lateral_target = c.cruise_speed_m_s - (c.cruise_speed_m_s - c.min_speed_m_s) * reduction
        target = max(c.min_speed_m_s, min(target, lateral_target))
        if self.authority_state == 'SLOW':
            target = min(target, c.authority_slow_speed_m_s)
        if self.phase == 'APPROACH_END':
            target = min(target, c.approach_speed_m_s)
        return target

    def _speed_loop(self, s, dt, target):
        """Integrate the base duty toward the target LiDAR speed (no speed model; PWM is not speed)."""
        c = self.config
        speed = s.get('speed')
        valid = s.get('odom_valid', True) is not False and speed is not None and math.isfinite(speed)
        ramped = self.drive_since is not None and self.now - self.drive_since >= c.mcu_ramp_s
        if valid and ramped and abs(target - speed) > c.speed_deadband_m_s:
            change = _clamp(c.speed_gain_pct_per_m_s * (target - speed) * dt,
                            -c.base_slew_down_pct_s * dt, c.base_slew_up_pct_s * dt)
            self.base_duty = _clamp(self.base_duty + change, c.base_min_pct, c.base_max_pct)
        return self.base_duty

    def _drive(self, s, lateral, heading_error):
        c = self.config
        dt = 1. / c.control_hz
        if self._last_drive_t is not None:
            dt = _clamp(self.now - self._last_drive_t, 0., 2. / c.control_hz)
        self._last_drive_t = self.now
        self.counts['follow_ticks'] += 1
        error = self.steering_error(lateral, heading_error)
        error_deg = math.degrees(error)
        speed = s.get('speed')
        speed = speed if speed is not None and math.isfinite(speed) else 0.
        # outer loop: pure-pursuit curvature x measured (floored) speed
        v = _clamp(speed, c.speed_floor_m_s, 1.)
        curvature = 2 * math.sin(error) / math.hypot(c.lookahead_m, lateral)     # [1/m]
        desired = _clamp(math.degrees(v * curvature), -c.max_yaw_rate_deg_s, c.max_yaw_rate_deg_s)
        self.r_ref += (desired - self.r_ref) * (1 - math.exp(-dt / c.yaw_reference_tau_s))
        measured = math.degrees(s['yaw_rate'])
        rate_error = self.r_ref - measured
        if not self._rolling and s.get('still') is not True and speed >= c.rolling_min_speed_m_s:
            self._rolling = True
        # speed loop
        self.target_speed = self._target_speed(s, curvature, lateral)
        base = self._speed_loop(s, dt, self.target_speed)
        limit = min(c.max_differential_pct, 100 - base, base)
        # inner loop: feed-forward + P + persistent bias; gyro gains scaled by the vibration-driven noise
        trust = self._gyro_trust(s, dt)
        kp, ki = c.yaw_kp_pct_per_deg_s * trust, c.yaw_ki_pct_per_deg * trust
        feedforward = self.r_ref / (2 * c.arc_yaw_deg_s_per_pct_diff)
        raw = feedforward + kp * rate_error + self.bias_pct
        saturated = self._rolling and abs(raw) >= limit
        target = _clamp(raw, -limit, limit) if self._rolling else 0.     # rolling start: equal duty first
        if c.sign_hysteresis_pct > 0 and target * self._out_sign < 0 and abs(target) < c.sign_hysteresis_pct:
            target = 0.                    # a small opposite demand does not cross zero (anti-chatter); on the
                                           # TARGET, so the slew can always carry a larger one across
        step = c.diff_slew_pct_s * dt
        applied = _clamp(target, self.differential - step, self.differential + step)
        if applied != target:
            self.counts['slew_limited_ticks'] += 1
        if applied:
            self._out_sign = 1 if applied > 0 else -1
        inhibited = False
        if c.side_clearance_check and applied and self._swing_toward_obstacle(s, 1 if applied > 0 else -1):
            # Equal duty instead of a swing toward a soft-side obstacle (never the opposite differential).
            applied, inhibited = 0., True
            self.counts['side_inhibit'] += 1
        # conditional integration: only while seen moving after the MCU ramp, and frozen while the
        # output is held below the demand in the direction the error would push it further
        moving = (self._rolling and s.get('still') is not True and s.get('odom_valid', True) is not False
                  and speed >= c.bias_min_speed_m_s and self.drive_since is not None
                  and self.now - self.drive_since >= c.mcu_ramp_s)
        held = (raw - applied) * rate_error > 0 and abs(raw - applied) > 1e-9
        if moving and not held:
            self.bias_pct = _clamp(self.bias_pct + ki * rate_error * dt, -c.bias_max_pct, c.bias_max_pct)
        else:
            self.counts['bias_frozen_ticks'] += 1
        self.differential = applied
        self._authority_update(saturated, abs(error_deg))
        left = self._quantize(0, base - applied)
        right = self._quantize(1, base + applied)
        if inhibited:
            reason = 'side_swing_inhibit'
        elif not self._rolling:
            reason = 'rolling_start'
        elif self.phase == 'APPROACH_END':
            reason = 'approach_end'
        elif self.authority_state == 'SLOW':
            reason = 'authority_slow'
        else:
            reason = 'single_row_follow'
        self._telemetry = dict(lookahead_error_deg=error_deg, target_yaw_rate_deg_s=desired,
                               lateral_reference_m=lateral, heading_reference_deg=math.degrees(heading_error),
                               reference_yaw_rate_deg_s=self.r_ref, measured_yaw_rate_deg_s=measured,
                               differential_pct=applied, raw_differential_pct=raw, differential_limit_pct=limit,
                               saturated=saturated, gyro_trust=trust)
        return Command(left, right, self.state, reason)

    def _gyro_trust(self, s, dt):
        """min(1, gyro_trust_vibration_m_s2 / low-passed vibration RMS); 1 when off or unknown."""
        c = self.config
        vibration = s.get('vibration_rms')
        if vibration is not None and math.isfinite(vibration) and vibration >= 0:
            if self._vibration is None:
                self._vibration = vibration
            else:
                self._vibration += (vibration - self._vibration) * (1 - math.exp(-dt / c.vibration_tau_s))
        reference = c.gyro_trust_vibration_m_s2
        if reference <= 0 or self._vibration is None or self._vibration <= reference:
            return 1.
        return c.gyro_trust_vibration_m_s2 / self._vibration

    def _quantize(self, side, value):
        """Integer duty in [0, 100] with a hysteresis of output_deadband_pct beyond the rounding step."""
        value = _clamp(value, 0., 100.)
        last = self._out[side]
        if last is None or abs(value - last) > .5 + self.config.output_deadband_pct:
            last = int(round(value))
        self._out[side] = last
        return last

    def _authority_update(self, saturated, error_deg):
        c = self.config
        if saturated:
            self.counts['saturated_ticks'] += 1
            self._unsat_since = None
            if self._sat_since is None:
                self._sat_since, self._sat_error0 = self.now, error_deg
            elif self.now - self._sat_since >= c.authority_sat_s:
                if error_deg <= self._sat_error0 - c.authority_progress_deg:
                    self._sat_since, self._sat_error0 = self.now, error_deg      # converging: re-arm
                elif self.authority_state != 'SLOW':
                    self.authority_state = 'SLOW'
                    self.counts['authority_slow_entries'] += 1
                    self._phase_event('AUTHORITY_SLOW', 'error_%.1f_deg' % error_deg)
            return
        self._sat_since = None
        if self.authority_state == 'SLOW':
            if self._unsat_since is None:
                self._unsat_since = self.now
            elif self.now - self._unsat_since >= c.authority_release_s:
                self.authority_state = 'OK'
                self._phase_event('AUTHORITY_OK', '')

    # --- unreachable RowController states: fail closed ---------------------------------------------
    def _reduce(self, s):
        return self._hold('stop_without_decision')

    def _turn(self, s):
        return self._hold('stop_without_decision')

    def _settle(self, s):
        return self._hold('stop_without_decision')

    # --- reporting -------------------------------------------------------------------------------------
    def telemetry(self):
        """Latest control quantities (SIM diagnostics; no truth)."""
        follow = self.counts['follow_ticks']
        t = self._telemetry
        s = getattr(self, '_last_snapshot', None) or {}
        return dict(
            phase=self.current_phase(), state=self.state, reason=self.reason,
            lateral_error_m=s.get('lateral_offset'),
            heading_error_deg=None if s.get('heading_error') is None else math.degrees(s['heading_error']),
            lateral_reference_m=t.get('lateral_reference_m'),       # plausibility-limited value steered on
            lookahead_error_deg=t.get('lookahead_error_deg'),
            target_yaw_rate_deg_s=t.get('target_yaw_rate_deg_s'),
            reference_yaw_rate_deg_s=t.get('reference_yaw_rate_deg_s'),
            measured_yaw_rate_deg_s=t.get('measured_yaw_rate_deg_s'),
            differential_pct=t.get('differential_pct'), raw_differential_pct=t.get('raw_differential_pct'),
            differential_limit_pct=t.get('differential_limit_pct'),
            bias_pct=self.bias_pct, base_duty_pct=self.base_duty,
            target_speed_m_s=self.target_speed, measured_speed_m_s=s.get('speed'),
            saturated=bool(t.get('saturated', False)),
            saturation_fraction=self.counts['saturated_ticks'] / follow if follow else 0.,
            gyro_trust=t.get('gyro_trust'), rolling=self._rolling,
            authority_state=self.authority_state, row_end_ahead_m=s.get('row_end_ahead_m'),
            stop_category=self.stop_category)

    def step(self, now, s):
        self._last_snapshot = s
        return super().step(now, s)
