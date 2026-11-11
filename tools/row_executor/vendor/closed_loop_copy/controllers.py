"""Encoderless row-following controllers (SIM evaluation candidates only).

Input: SensorEstimator.snapshot() (LiDAR/IMU-derived; no truth, no wheel
speed). Output: signed integer duty percents (left, right) for the MCU, i.e.
commands only. Three modes share one state machine:

  A 'continuous'  low-speed continuous correction only (never stops to turn)
  B 'stop_turn'   equal-duty straight driving; any heading/centre error is
                  corrected by stop -> pivot -> settle -> restart
  C 'hybrid'      default: continuous correction; stop -> turn -> restart only
                  for a large lookahead (heading + centre) error or an
                  ineffective steering response with a material error

States: FOLLOW, REDUCE, STOP_REQUESTED, STOP_OBSERVED, TURN, SETTLE,
REACQUIRE, HOLD. Stale/missing sensors -> HOLD (recoverable after fresh,
sensor-observed stillness) unless a terminal decision is already pending, in
which case that terminal HOLD is entered. Pivot stall (at the start or mid
turn) -> abort -> HOLD (never pushed). A commanded forward drive that the
sensors do not see move -> HOLD (never pushed). Every HOLD reason except the
explicitly recoverable stale-input ones is terminal: a human must decide.
All thresholds are SIM tuning unless their comment cites the evidence.

Opt-in variants '<mode>_side' (default OFF) add a side-clearance rule on the
estimator's per-side beside band: a hard margin confirmed on distinct scans is a
terminal 'side_blocked' (start refused / stop while driving, after the front and
row-end checks); a soft margin refuses only the differential sign that would
swing the footprint toward that side about the NOMINAL rotation centre (equal
duty instead); 'hybrid_side_bias' adds a bounded centreline bias for steering.
"""
from collections import deque
from dataclasses import dataclass, replace
import math


MODES = ('continuous', 'stop_turn', 'hybrid')
# Optional side-clearance rule on top of a base mode (opt-in variants, SIM tuning; see ControlConfig).
SIDE_VARIANTS = dict(continuous_side='continuous', stop_turn_side='stop_turn', hybrid_side='hybrid',
                     hybrid_side_bias='hybrid')
# hybrid_fallback: hybrid + continuous fallback after a failed pivot (evaluated, opt-in)
VARIANTS = ('hybrid_fallback',) + tuple(SIDE_VARIANTS)
TERMINAL_HOLDS = ('front_blocked', 'row_end_candidate', 'pivot_stall_abort', 'turn_timeout',
                  'stop_turn_repeat_limit', 'stop_repeat_limit', 'pivot_clearance_insufficient',
                  'corridor_lost', 'stop_not_observed', 'settle_timeout', 'corridor_not_reacquired',
                  'corridor_not_acquired', 'turn_not_permitted', 'drive_no_response', 'sensor_missing',
                  'side_blocked')
RECOVERABLE_HOLDS = ('stale_lidar', 'stale_imu')
DRIVING_STATES = ('FOLLOW', 'REDUCE')


@dataclass(frozen=True)
class ControlConfig:
    mode: str = 'hybrid'
    control_hz: float = 20.
    # --- duty (command) levels; low-duty response is SIM_ASSUMED -----------
    base_duty_pct: int = 75            # leaves +-25 differential below 100% (headroom)
    max_differential_pct: int = 25     # outputs stay within [50, 100] in FOLLOW
    reduce_duty_pct: int = 62
    pivot_duty_pct: int = 100          # only +-100 pivots were MEASURED
    # --- expected response ----------------------------------------------------
    arc_yaw_deg_s_per_pct_diff: float = .12   # ESTIMATED: 90/100 arc -> ~1 deg/s
    pivot_rate_slow_deg_s: float = 5.         # ESTIMATED: slowest measured rotating +-100 events 4.8 (IMU)/5.7 (LiDAR)
    mcu_ramp_s: float = 1.                    # MEASURED 0->100% in ~1.0 s (249 units/s)
    # --- outer loop (corridor) ----------------------------------------------
    lookahead_m: float = 3.0
    heading_gain: float = .8           # desired yaw rate per rad of heading error [1/s]
    max_yaw_rate_deg_s: float = 4.5    # ~75% of the +-25 authority (6 deg/s)
    # deadband/hysteresis from MEASURED straight wander: chord offset p90
    # 0.10 m and heading change ~1 deg/m; corrections below this are noise.
    enter_error_deg: float = 2.5
    exit_error_deg: float = 1.0
    urgent_error_deg: float = 6.
    min_hold_s: float = 1.0            # >= MCU ramp time (MEASURED 1.0 s)
    recorrect_interval_s: float = 1.5
    # --- inner loop (IMU yaw rate) --------------------------------------
    yaw_kp_pct_per_deg_s: float = 2.
    yaw_ki_pct_per_deg: float = 1.5
    # --- stop-turn trigger (lookahead error, the same quantity the pivot aims at)
    turn_enter_deg: float = 15.
    turn_exit_deg: float = 10.         # REDUCE returns to FOLLOW below this
    reduce_confirm_s: float = .6
    weak_response_ratio: float = .3
    weak_response_hold_s: float = 2.0
    weak_min_error_deg: float = 6.     # weak steering stops only for a material error ...
    weak_progress_deg: float = 1.      # ... that did not shrink by this much in the window
    min_turn_deg: float = 3.
    # --- stop / turn / settle ------------------------------------------------
    still_confirm_s: float = .4        # sensor stillness must have lasted this long
    stop_fade_margin_s: float = .1     # + |last duty|/100 * ramp: MCU fade must be over
    stop_timeout_s: float = 3.
    turn_coast_s: float = .4           # initial guess, adapted from observed overshoot
    stall_grace_s: float = 1.0         # after the MEASURED 1.0 s ramp
    stall_min_turn_deg: float = 3.     # gyro-integral fallback when no scan-matching heading is reported
    stall_rate_floor_deg_s: float = 2.  # below every measured rotating event (>= 4.8 deg/s)
    stall_min_turn_lidar_deg: float = 1.5   # scan-matching rotation (not vibration-noisy like the gyro)
    turn_progress_window_s: float = 1.
    turn_progress_min_deg: float = 1.5  # mid-turn stick: less than this per window
    settle_s: float = .5               # continuous dwell of stillness + low vibration
    settle_vibration_m_s2: float = .8
    settle_timeout_s: float = 4.
    reacquire_updates: int = 2
    reacquire_timeout_s: float = 6.
    sensor_missing_s: float = 3.
    # --- repeat caps -----------------------------------------------------
    max_turns_per_window: int = 2
    max_stops_per_window: int = 3      # every non-terminal stop, turned or not
    turn_window_m: float = 5.
    # --- optional: continuous fallback when a pivot is impossible or stalls (hybrid) --
    pivot_failure_fallback: bool = False
    fallback_distance_m: float = 6.    # continuous correction only (no pivot) for this distance
    max_fallbacks: int = 1
    # --- forward drive response (no encoder: LiDAR/IMU stillness) ----------
    drive_grace_s: float = 1.          # after the MEASURED 1.0 s ramp
    drive_no_response_s: float = 1.
    drive_min_speed_m_s: float = .03
    # --- freshness ------------------------------------------------------
    lidar_max_age_s: float = .45
    imu_max_age_s: float = .3
    fresh_recover_s: float = 1.
    corridor_lost_s: float = 2.
    corridor_wait_s: float = 5.
    # --- front / row end -------------------------------------------------
    front_margin_m: float = .35
    # ESTIMATED from the refitted plant (clean fade-to-zero stops): speed held
    # ~0.25 s after the release, then a coast deceleration whose p10 is ~1.07 m/s^2.
    stop_onset_s: float = .25
    stop_decel_m_s2: float = 1.07
    reaction_s: float = .15
    row_end_stop_m: float = .0           # stop when the base reaches the observed end
    # --- optional side-clearance rule (variants *_side; SIM tuning) ---------------
    # Reads the estimator's per-side split of its beside band (newest raw scan). OFF
    # by default: with side_clearance_check False none of the fields below is read.
    side_clearance_check: bool = False
    side_hard_margin_m: float = .03    # 2x SIM LiDAR noise .015 (SIM_ASSUMED); estimate reads 1.6-2 cm low
    side_soft_margin_m: float = .15    # hard + straight-wander chord p90 ~0.10 m (ESTIMATED)
    side_soft_exit_m: float = .20      # +0.05 m hysteresis
    side_confirm_scans: int = 2        # DISTINCT scans (20 Hz ticks, 10 Hz scans)
    side_bias_max_m: float = 0.        # 0 = off; hybrid_side_bias: .15 (atan(.15/3) = 2.9 deg < 10 deg exit)
    side_bias_gain: float = 1.

    def __post_init__(self):
        if self.mode not in MODES:
            raise ValueError('unknown_controller_mode')
        if (self.base_duty_pct + self.max_differential_pct > 100
                or self.base_duty_pct - self.max_differential_pct < 0):
            raise ValueError('no_differential_headroom')
        if self.min_hold_s < self.mcu_ramp_s or self.exit_error_deg >= self.enter_error_deg:
            raise ValueError('hysteresis_or_hold_invalid')
        if self.turn_exit_deg >= self.turn_enter_deg or self.stop_decel_m_s2 <= 0:
            raise ValueError('turn_hysteresis_or_stop_model_invalid')
        if (not 0 < self.side_hard_margin_m < self.side_soft_margin_m < self.side_soft_exit_m
                or self.side_confirm_scans < 1 or not 0 <= self.side_bias_max_m <= .3):
            raise ValueError('side_rule_invalid')


def config_for(mode, **overrides):
    if mode in SIDE_VARIANTS:
        # The base mode's settings (e.g. the lenient stop_turn caps) are kept; the rule is added.
        extra = dict(side_clearance_check=True, side_bias_max_m=.15 if mode == 'hybrid_side_bias' else 0.)
        return config_for(SIDE_VARIANTS[mode], **{**extra, **overrides})
    if mode == 'hybrid_fallback':
        return replace(ControlConfig(mode='hybrid', pivot_failure_fallback=True), **overrides)
    base = ControlConfig(mode=mode)
    if mode == 'stop_turn':
        # Lenient repeat caps so the baseline can show its stop count over a row.
        base = replace(base, turn_enter_deg=4., turn_exit_deg=2., min_turn_deg=2.,
                       max_turns_per_window=8, max_stops_per_window=8, reduce_confirm_s=0.)
    return replace(base, **overrides)


@dataclass
class Command:
    left: int
    right: int
    state: str
    reason: str


class RowController:
    def __init__(self, config=ControlConfig()):
        self.config = config
        self.state = 'REACQUIRE'
        self.reason = 'start'
        self.since = 0.
        self.now = 0.
        self._distance = 0.
        self.started = None
        self.correcting = False
        self.correction_sign = 0
        self.correction_since = -math.inf
        self.correction_end = -math.inf
        self.sign_change = -math.inf
        self.integral = 0.
        self.differential = 0.
        self.weak_since = None
        self.weak_error0 = None
        self.reduce_since = None
        self.pending = None
        self.turn = None
        self.settle_turn = None
        self.settle_ok_since = None
        self.turn_coast_s = config.turn_coast_s
        self.turn_starts = []          # odometry distance at each turn start
        self.stop_starts = []          # odometry distance at each non-terminal stop
        self.fresh_since = None
        self.reacquire_updates0 = 0
        self.events = []
        self.hold_terminal = False
        self.last_command = (0, 0)
        self.stop_fade_s = 0.
        self.drive_since = None
        self.no_motion_since = None
        self.pivots_disabled_until = -math.inf   # odometry distance (continuous fallback)
        self.side_streak = 0           # consecutive DISTINCT scans with a hard side violation (opt-in rule)
        self.side_scan = None          # scan_count of the last side update
        self.side_soft = dict(left=False, right=False)
        self.side_x = dict(left=None, right=None)       # base x of the nearest return per side
        self.side_comp = dict(left=math.inf, right=math.inf)
        self.side_bias_m = 0.          # desired lateral offset (band left normal +), opt-in bias only
        self.counts = dict(stops=0, turns=0, stalls=0, holds=0, stops_without_turn=0, fallbacks=0,
                           side_stops=0, side_inhibit=0)

    # --- helpers -------------------------------------------------------
    def _go(self, state, reason=''):
        if state != self.state:
            self.events.append((round(self.now, 3), state, reason))
        if state == 'STOP_REQUESTED' and self.state != 'STOP_REQUESTED':
            self.counts['stops'] += 1
            # The MCU fades the last commanded duty at ~100 %/s; stillness can
            # only certify a stop once that fade is over.
            self.stop_fade_s = max(map(abs, self.last_command)) / 100. * self.config.mcu_ramp_s
        self.state, self.reason, self.since = state, reason, self.now

    def _hold(self, reason):
        # Fail closed: only the explicitly recoverable reasons auto-resume.
        self.hold_terminal = reason not in RECOVERABLE_HOLDS
        self.counts['holds'] += 1
        self._reset_follow()
        self.turn = None
        self._go('HOLD', reason)
        return self._zero()

    def _zero(self):
        return Command(0, 0, self.state, self.reason)

    def _reset_follow(self):
        self.correcting = False
        self.correction_sign = 0
        self.integral = 0.
        self.differential = 0.
        self.weak_since = None
        self.weak_error0 = None

    def _still(self, s):
        """Sensor stillness confirmed for still_confirm_s (LiDAR keyframe poses + gyro)."""
        return bool(s['still']) and s.get('still_s', math.inf) >= self.config.still_confirm_s

    def _stop(self, pending, reason):
        """Begin a stop; 'pending' is 'turn', 'front', 'wait_corridor' or 'hold'."""
        c = self.config
        if pending in ('turn', 'wait_corridor', 'front'):
            distance = self._distance
            recent = [d for d in self.stop_starts if distance - d <= c.turn_window_m]
            if len(recent) >= c.max_stops_per_window:
                pending, reason = 'hold', 'stop_repeat_limit'
            else:
                self.stop_starts.append(distance)
        self.pending = (pending, reason)
        self._reset_follow()
        self._go('STOP_REQUESTED', reason)
        return self._zero()

    def _pivots_disabled(self):
        return self._distance < self.pivots_disabled_until

    def _fallback(self, s, reason):
        """Continuous correction instead of a pivot that is impossible or stalled (opt-in, capped)."""
        c = self.config
        if (not c.pivot_failure_fallback or self.counts['fallbacks'] >= c.max_fallbacks
                or not s['corridor_valid'] or s['front_clearance_m'] < self.required_front(0.)):
            return None
        self.counts['fallbacks'] += 1
        self.pivots_disabled_until = s['distance_m'] + c.fallback_distance_m
        self.turn = None
        self.reacquire_updates0 = s.get('corridor_updates', 0)
        self._go('REACQUIRE', 'continuous_fallback_after_' + reason)
        return self._zero()

    def stopping_distance(self, speed):
        """Distance to rest after a zero command (ESTIMATED stop model, conservative p10 decel)."""
        c = self.config
        v = max(0., speed)
        return v * (c.reaction_s + c.stop_onset_s) + v * v / (2 * c.stop_decel_m_s2)

    def required_front(self, speed):
        return self.stopping_distance(speed) + self.config.front_margin_m

    def target_error(self, s, desired_lateral=0.):
        """Heading error towards a lookahead point on the centreline [rad].

        desired_lateral (opt-in side bias only) shifts the aim point; 0 gives the
        identical value (x - 0.0 == x bit for bit, including -0.0).
        """
        e = s['heading_error'] - math.atan2(s['lateral_offset'] - desired_lateral, self.config.lookahead_m)
        return math.atan2(math.sin(e), math.cos(e))

    # --- main tick -------------------------------------------------------
    def step(self, now, s):
        c = self.config
        self.now = now
        self._distance = s.get('distance_m', 0.)
        if self.started is None:
            self.started = now
        if self.state == 'HOLD' and self.hold_terminal:
            return self._record(self._zero())
        stale = ('stale_lidar' if s['lidar_age_s'] > c.lidar_max_age_s else
                 'stale_imu' if s['imu_age_s'] > c.imu_max_age_s else None)
        if stale and self.state == 'REACQUIRE' and self.reason == 'start' and (
                math.isinf(s['lidar_age_s']) or math.isinf(s['imu_age_s'])):
            if now - self.started > c.sensor_missing_s:
                return self._record(self._hold('sensor_missing'))
            return self._record(self._zero())   # no data yet: wait, outputs zero (not a fault yet)
        if stale:
            self.fresh_since = None
            if self.state == 'STOP_REQUESTED' and self.pending and self.pending[0] == 'hold':
                # A terminal decision is already pending: never downgrade it.
                reason = self.pending[1]
                self.pending = None
                return self._record(self._hold(reason))
            if self.state == 'TURN' and self.turn is not None:
                t = self.turn
                if (now - t['start'] >= c.mcu_ramp_s + c.stall_grace_s
                        and t.get('progress', 0.) < t.get('stall_threshold', c.stall_min_turn_deg)):
                    self.counts['stalls'] += 1
                    return self._record(self._hold('pivot_stall_abort'))   # stall-suspect: terminal
                self.turn = None
            if not (self.state == 'HOLD' and self.reason in RECOVERABLE_HOLDS):
                return self._record(self._hold(stale))
            return self._record(self._zero())
        if self.state == 'HOLD':   # recoverable: needs fresh input and stillness
            if self.fresh_since is None:
                self.fresh_since = now
            if now - self.fresh_since >= c.fresh_recover_s and self._still(s):
                self.reacquire_updates0 = s.get('corridor_updates', 0)
                self._go('REACQUIRE', 'fresh_input_and_still')
            return self._record(self._zero())
        if c.side_clearance_check:
            self._side_update(s)
        handler = getattr(self, '_' + self.state.lower())
        return self._record(handler(s))

    def _record(self, command):
        driving = self.state in DRIVING_STATES and (command.left > 0 or command.right > 0)
        if driving:
            if self.drive_since is None:
                self.drive_since = self.now
        else:
            self.drive_since = None
            self.no_motion_since = None
        if command.left or command.right:
            self.last_command = (command.left, command.right)
        return command

    # --- optional side-clearance rule (opt-in; ControlConfig.side_*) -----------
    def _side_update(self, s):
        """Once per DISTINCT scan: yaw-compensated per-side clearance, hard streak, soft hysteresis.

        comp = clearance - |yaw rate| * (|x - xc| + sd) * LiDAR age, i.e. the
        sideways sweep since the scan of the footprint part at the nearest return's
        x about the NOMINAL rotation centre xc (uncertainty sd). A snapshot without
        scan_count (no distinct-scan evidence) updates on every tick (fail-safe).
        """
        c = self.config
        scan = s.get('scan_count')
        if scan is not None and scan == self.side_scan:
            return
        self.side_scan = scan
        xc, sd = s.get('rotation_centre_x_m', 0.), s.get('rotation_centre_sd_m', 0.)
        sweep = abs(s.get('yaw_rate', 0.)) * max(0., s.get('lidar_age_s', 0.))
        for side in ('left', 'right'):
            x = s.get('side_nearest_x_%s_m' % side)
            clearance = s.get('side_clearance_%s_m' % side, math.inf)
            comp = math.inf if x is None else clearance - sweep * (abs(x - xc) + sd)
            self.side_x[side], self.side_comp[side] = x, comp
            if comp <= c.side_soft_margin_m:
                self.side_soft[side] = True
            elif comp >= c.side_soft_exit_m:
                self.side_soft[side] = False
        self.side_streak = self.side_streak + 1 if min(self.side_comp.values()) <= c.side_hard_margin_m else 0
        self.side_bias_m = 0.
        soft = [side for side in ('left', 'right') if self.side_soft[side]]
        if c.side_bias_max_m > 0 and len(soft) == 1:
            near = soft[0]
            far = 'right' if near == 'left' else 'left'
            x = self.side_x[near]
            bias = min(c.side_bias_max_m, c.side_bias_gain * (c.side_soft_exit_m - self.side_comp[near]))
            # Only for a return AHEAD of the uncertain rotation centre (yawing away then swings
            # that part away) and only when the other side has room for the shift.
            if x is not None and x > xc + sd and bias > 0 and self.side_comp[far] > c.side_soft_margin_m + bias:
                self.side_bias_m = -bias if near == 'left' else bias

    def _side_blocked(self):
        c = self.config
        return c.side_clearance_check and self.side_streak >= c.side_confirm_scans

    def _side_stop(self, immediate):
        """Terminal side_blocked: refuse to start (immediate HOLD) or stop while driving."""
        self.counts['side_stops'] += 1
        return self._hold('side_blocked') if immediate else self._stop('hold', 'side_blocked')

    def _swing_toward_obstacle(self, s, w_sign):
        """Would a yaw of sign w_sign (+1 = left/CCW) swing the footprint toward a soft side?

        A base point at x moves sideways by w * (x - xc): ahead of the rotation
        centre it follows the yaw, behind it moves the other way, and inside the
        uncertain zone (|x - xc| <= sd) either can happen, so both signs are refused.
        """
        xc, sd = s.get('rotation_centre_x_m', 0.), s.get('rotation_centre_sd_m', 0.)
        for side, side_sign in (('left', 1), ('right', -1)):
            x = self.side_x[side]
            if not self.side_soft[side] or x is None:
                continue
            if x > xc + sd:
                refused = (side_sign,)
            elif x < xc - sd:
                refused = (-side_sign,)
            else:
                refused = (1, -1)
            if w_sign in refused:
                return True
        return False

    def _steer_error(self, s, error):
        """Optional bounded centreline bias for steering only (never triggers or the pivot aim)."""
        if not self.side_bias_m or not s.get('corridor_valid'):
            return error
        return self.target_error(s, self.side_bias_m)

    # --- states ------------------------------------------------------------
    def _reacquire(self, s):
        c = self.config
        if self.now - self.since > c.reacquire_timeout_s:
            return self._hold('corridor_not_acquired' if self.reason == 'start' else 'corridor_not_reacquired')
        if not s['corridor_valid'] or s['corridor_age_s'] > .6 or not self._still(s):
            return self._zero()
        if s.get('corridor_updates', 0) - self.reacquire_updates0 < c.reacquire_updates and self.reason != 'start':
            return self._zero()
        error = math.degrees(self.target_error(s))
        if c.mode != 'continuous' and abs(error) > c.turn_enter_deg and not self._pivots_disabled():
            return self._prepare_turn(s)
        if s['front_clearance_m'] < self.required_front(0.):
            return self._hold('front_blocked')
        if self._side_blocked():
            return self._side_stop(immediate=True)
        self._go('FOLLOW', 'corridor_acquired')
        return self._follow(s)

    def _drive_no_response(self, s):
        """Forward drive commanded past the ramp + grace, but the sensors see no motion."""
        c = self.config
        if self.drive_since is None or self.now - self.drive_since < c.mcu_ramp_s + c.drive_grace_s:
            self.no_motion_since = None
            return False
        no_motion = self._still(s) or (s.get('odom_valid', True) and s['speed'] < c.drive_min_speed_m_s)
        if not no_motion:
            self.no_motion_since = None
            return False
        if self.no_motion_since is None:
            self.no_motion_since = self.now
        return self.now - self.no_motion_since >= c.drive_no_response_s

    def _follow(self, s):
        c = self.config
        if not s['corridor_valid'] or s['corridor_age_s'] > c.corridor_lost_s:
            return self._stop('wait_corridor', 'corridor_lost')
        speed = max(0., s['speed'])
        if s['front_clearance_m'] < self.required_front(speed):
            return self._stop('front', 'front_blocked')
        end = s.get('row_end_ahead_m')
        if end is not None and end <= c.row_end_stop_m + self.stopping_distance(speed):
            return self._stop('hold', 'row_end_candidate')
        if self._side_blocked():
            return self._side_stop(immediate=False)
        if self._drive_no_response(s):
            return self._stop('hold', 'drive_no_response')
        error = self.target_error(s)
        if c.mode == 'stop_turn':
            # The lookahead error contains the lateral offset (0.25 m at 3 m is
            # ~4.8 deg), the same quantity the pivot aims at.
            if abs(math.degrees(error)) > c.turn_enter_deg:
                return self._stop('turn', 'stop_turn_correction')
            return Command(c.base_duty_pct, c.base_duty_pct, self.state, 'straight_equal_duty')
        if c.mode == 'hybrid' and abs(math.degrees(error)) > c.turn_enter_deg and not self._pivots_disabled():
            self.reduce_since = self.now
            self._go('REDUCE', 'large_heading_error')
            return self._steer(s, self._steer_error(s, error), c.reduce_duty_pct)
        if c.mode == 'hybrid' and not self._pivots_disabled() and self._weak_response(s, error):
            self.reduce_since = self.now
            self._go('REDUCE', 'steering_ineffective')
            return self._steer(s, self._steer_error(s, error), c.reduce_duty_pct)
        return self._steer(s, self._steer_error(s, error), c.base_duty_pct)

    def _weak_response(self, s, error):
        """Saturated differential, little yaw, and a material error that is not shrinking."""
        c = self.config
        e = abs(math.degrees(error))
        if not self.correcting or abs(self.differential) < .6 * c.max_differential_pct:
            self.weak_since = self.weak_error0 = None
            return False
        expected = c.arc_yaw_deg_s_per_pct_diff * 2 * self.differential
        measured = math.degrees(s['yaw_rate'])
        if measured * math.copysign(1, expected) >= c.weak_response_ratio * abs(expected):
            self.weak_since = self.weak_error0 = None
            return False
        if self.weak_since is None:
            self.weak_since, self.weak_error0 = self.now, e
        if self.now - self.weak_since < c.weak_response_hold_s + c.mcu_ramp_s:
            return False
        if e < c.weak_min_error_deg:
            return False                       # small residual error: accept it, no stop
        if e < self.weak_error0 - c.weak_progress_deg:
            self.weak_since, self.weak_error0 = self.now, e   # still converging: re-arm
            return False
        return True

    def _steer(self, s, error, base):
        """Continuous correction with deadband, hysteresis, hold and anti-windup."""
        c = self.config
        dt = 1 / c.control_hz
        e = math.degrees(error)
        if not self.correcting:
            allowed = (self.now - self.correction_end >= c.recorrect_interval_s
                       or abs(e) >= c.urgent_error_deg)
            if abs(e) >= c.enter_error_deg and allowed:
                self.correcting = True
                self.correction_since = self.now
                self.integral = 0.
        elif abs(e) <= c.exit_error_deg and self.now - self.correction_since >= c.min_hold_s:
            self.correcting = False
            self.correction_end = self.now
            self.integral = 0.
        if not self.correcting:
            self.differential = 0.
            self.correction_sign = 0
            return Command(base, base, self.state, 'deadband_equal_duty')
        desired = max(-c.max_yaw_rate_deg_s, min(c.max_yaw_rate_deg_s, c.heading_gain * e))
        sign = 1 if desired > 0 else -1
        if self.correction_sign and sign != self.correction_sign and self.now - self.sign_change < c.min_hold_s:
            desired, sign = 0., self.correction_sign      # min hold: no quick reversal
        elif sign != self.correction_sign:
            self.sign_change = self.now
            self.correction_sign = sign
            self.integral = 0.
        measured = math.degrees(s['yaw_rate'])
        rate_error = desired - measured
        feedforward = desired / (2 * c.arc_yaw_deg_s_per_pct_diff)
        limit = min(c.max_differential_pct, 100 - base, base)
        proposed = self.integral + c.yaw_ki_pct_per_deg * rate_error * dt
        raw = feedforward + c.yaw_kp_pct_per_deg_s * rate_error + proposed
        if not (abs(raw) > limit and raw * rate_error > 0):   # conditional integration
            self.integral = max(-limit, min(limit, proposed))
        raw = feedforward + c.yaw_kp_pct_per_deg_s * rate_error + self.integral
        # One-sided: never command the opposite differential to brake a yaw
        # overshoot (that is what produced sign chatter); outputs ease to equal.
        if desired > 0:
            raw = max(0., raw)
        elif desired < 0:
            raw = min(0., raw)
        else:
            raw = 0.
        self.differential = max(-limit, min(limit, raw))
        if c.side_clearance_check and self.differential and \
                self._swing_toward_obstacle(s, 1 if self.differential > 0 else -1):
            # Equal duty instead of a swing toward a soft-side obstacle (never the opposite
            # differential); correcting/correction_sign are kept, the integral is reset.
            self.integral = 0.
            self.differential = 0.
            self.counts['side_inhibit'] += 1
            return Command(base, base, self.state, 'side_swing_inhibit')
        left = int(round(base - self.differential))
        right = int(round(base + self.differential))
        return Command(left, right, self.state, 'continuous_correction')

    def _reduce(self, s):
        c = self.config
        if not s['corridor_valid']:
            return self._stop('wait_corridor', 'corridor_lost')
        if s['front_clearance_m'] < self.required_front(max(0., s['speed'])):
            return self._stop('front', 'front_blocked')
        if self._side_blocked():
            return self._side_stop(immediate=False)
        if self._drive_no_response(s):
            return self._stop('hold', 'drive_no_response')
        error = self.target_error(s)
        if self.reason == 'large_heading_error' and abs(math.degrees(error)) < c.turn_exit_deg:
            self._go('FOLLOW', 'heading_recovered')
            return self._steer(s, self._steer_error(s, error), c.base_duty_pct)
        if self.now - self.reduce_since >= c.reduce_confirm_s:
            return self._stop('turn', self.reason)
        return self._steer(s, self._steer_error(s, error), c.reduce_duty_pct)

    def _stop_requested(self, s):
        c = self.config
        if self._still(s) and self.now - self.since >= self.stop_fade_s + c.stop_fade_margin_s:
            self._go('STOP_OBSERVED', 'sensor_stillness')
            return self._stop_observed(s)
        if self.now - self.since > c.stop_timeout_s + self.stop_fade_s:
            self.pending = None
            return self._hold('stop_not_observed')
        return self._zero()

    def _stop_observed(self, s):
        c = self.config
        kind, reason = self.pending or ('hold', 'stop_without_decision')
        self.pending = None
        if kind == 'hold':
            if reason == 'pivot_stall_abort':
                fallback = self._fallback(s, reason)
                if fallback is not None:
                    return fallback
            return self._hold(reason)
        if kind == 'wait_corridor':
            self.reacquire_updates0 = s.get('corridor_updates', 0)
            self._go('REACQUIRE', 'corridor_lost_wait')
            self.since = self.now - (c.reacquire_timeout_s - c.corridor_wait_s)
            return self._zero()
        if kind == 'front':
            if (c.mode != 'continuous' and s['corridor_valid']
                    and abs(math.degrees(self.target_error(s))) > c.turn_enter_deg):
                return self._prepare_turn(s)
            return self._hold('front_blocked')
        return self._prepare_turn(s)

    def _prepare_turn(self, s):
        c = self.config
        if c.mode == 'continuous':
            return self._hold('turn_not_permitted')
        if not s['corridor_valid']:
            return self._hold('corridor_lost')
        # Aim at a lookahead point on the centreline (heading + bounded lateral).
        target = math.degrees(self.target_error(s))
        if abs(target) < c.min_turn_deg:
            if s['front_clearance_m'] < self.required_front(0.):
                return self._hold('front_blocked')
            if self._side_blocked():
                return self._side_stop(immediate=True)
            self.counts['stops_without_turn'] += 1
            self._go('FOLLOW', 'turn_not_needed')
            return Command(c.base_duty_pct, c.base_duty_pct, self.state, self.reason)
        distance = s['distance_m']
        recent = [d for d in self.turn_starts if distance - d <= c.turn_window_m]
        if len(recent) >= c.max_turns_per_window:
            return self._hold('stop_turn_repeat_limit')
        if s['pivot_clearance_m'] < s['pivot_radius_required_m']:
            return self._fallback(s, 'pivot_clearance_insufficient') or self._hold('pivot_clearance_insufficient')
        self.turn_starts.append(distance)
        self.counts['turns'] += 1
        self.turn = dict(target=target, heading0=s['gyro_heading'], start=self.now, released=None,
                         rate_at_release=0., turned=0., history=deque(), lidar0=s.get('lidar_heading'))
        self._go('TURN', 'pivot_%+.1f_deg' % target)
        return self._turn(s)

    def _abort_turn(self, reason):
        self.turn = None
        self.pending = ('hold', reason)
        self._go('STOP_REQUESTED', reason)
        return self._zero()

    def _turn(self, s):
        c = self.config
        t = self.turn
        rate = math.degrees(s['yaw_rate'])
        t['turned'] = math.degrees(s['gyro_heading'] - t['heading0'])
        sign = 1 if t['target'] > 0 else -1
        elapsed = self.now - t['start']
        if t['released'] is None:
            remaining = abs(t['target']) - sign * t['turned']
            coast = max(0., sign * rate) * self.turn_coast_s
            if remaining <= coast:
                t['released'] = self.now
                t['turned_at_release'] = t['turned']
                t['rate_at_release'] = max(1., sign * rate)
                self._go('SETTLE', 'turn_released')
                self.settle_turn = t
                self.settle_ok_since = None
                return self._zero()
            # Progress/stall evidence: scan-matching rotation when the estimator reports it
            # (pivot vibration makes the gyro noisy), else the bias-corrected gyro integral.
            lidar = s.get('lidar_heading')
            if t['lidar0'] is not None and lidar is not None:
                progress = sign * math.degrees(math.atan2(math.sin(lidar - t['lidar0']), math.cos(lidar - t['lidar0'])))
                stalled = progress < c.stall_min_turn_lidar_deg
            else:
                progress = sign * t['turned']
                stalled = progress < c.stall_min_turn_deg and sign * rate < c.stall_rate_floor_deg_s
            t['progress'] = progress
            t['stall_threshold'] = c.stall_min_turn_lidar_deg if lidar is not None and t['lidar0'] is not None                 else c.stall_min_turn_deg
            history = t['history']
            history.append((self.now, progress))
            while history and self.now - history[0][0] > c.turn_progress_window_s:
                history.popleft()
            if elapsed >= c.mcu_ramp_s + c.stall_grace_s:
                if stalled:
                    self.counts['stalls'] += 1
                    return self._abort_turn('pivot_stall_abort')
                window = history[-1][1] - history[0][1]
                if (history[-1][0] - history[0][0] >= .95 * c.turn_progress_window_s
                        and window < c.turn_progress_min_deg):
                    self.counts['stalls'] += 1      # stuck after partial rotation
                    return self._abort_turn('pivot_stall_abort')
            limit = c.mcu_ramp_s + 1.5 * abs(t['target']) / c.pivot_rate_slow_deg_s + 1.5
            if elapsed > limit:
                return self._abort_turn('turn_timeout')
            duty = c.pivot_duty_pct
            return Command(-sign * duty, sign * duty, self.state, self.reason)
        return self._zero()

    def _settle(self, s):
        c = self.config
        t = self.settle_turn
        if t is not None:
            t['turned'] = math.degrees(s['gyro_heading'] - t['heading0'])
        vibration = s['vibration_rms']
        ok = self._still(s) and (vibration is None or vibration < c.settle_vibration_m_s2)
        if not ok:
            self.settle_ok_since = None
        elif self.settle_ok_since is None:
            self.settle_ok_since = self.now
        if ok and self.now - self.settle_ok_since >= c.settle_s:
            if t is not None:
                sign = 1 if t['target'] > 0 else -1
                coast_angle = sign * (t['turned'] - t['turned_at_release'])
                observed = max(0., coast_angle) / t['rate_at_release']
                self.turn_coast_s = .5 * self.turn_coast_s + .5 * min(1.5, observed)
                self.settle_turn = None
                self.turn = None
            self.reacquire_updates0 = s.get('corridor_updates', 0)
            self._go('REACQUIRE', 'settled')
            return self._zero()
        if self.now - self.since > c.settle_timeout_s:
            return self._hold('settle_timeout')
        return self._zero()
