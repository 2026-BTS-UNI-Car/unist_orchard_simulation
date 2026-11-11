"""Last checks before a duty leaves the executor (OutputGuard) and the MVP-1 row backstop.

OutputGuard is deliberately independent of the controller: it re-checks every proposed command
and turns any violation into (0, 0) plus a violation string, which the Runner treats as a
terminal 'guard_violation' (a bug, never a recoverable condition):
  - both duties are Python ints (bool refused)
  - no negative duty (reverse or pivot): the weekend executor is forward-only
  - 0 <= duty <= duty_cap_pct
  - |left - right| <= diff_cap_pct
  - optional inner-wheel rule: when driving, min(left, right) >= inner_min_pct (measured
    breakaway + margin), so the slower wheel never sits in the stall band
  - (review F2) when a rolling threshold is measured, the minimum is the static one
    (inner_min_pct) unless the caller states rolling=True (the Runner does so only while the
    estimator reports sustained motion); then it is rolling_inner_min_pct (<= the static one)
(0, 0) always passes.

SteerOutputGuard (steering branch P4, design 6.2 / 8.3 P4) is the same last check for the front-steer vehicle
(config vehicle_profile 'front_steer'): one drive duty for both sides and an equivalent steering target, no inner-wheel
rule (equal duty both sides). See its docstring.

Backstop implements the stop rule of FEASIBILITY_CRITIQUE H2 / MVP-1 for the parts that do not
come from the controller (row_end_candidate is the controller's own terminal hold):
  distance  odometry distance since Start >= row_len - margin (default 2 m)
  time      motion time >= factor * (row_len - margin) / v_measured (default factor 1.15)
"Motion time" accumulates only while the last command was non-zero or the estimator does not
report sensor stillness, so a paused, standing vehicle does not run the clock down, while fade,
coast and an unknown state count (conservative). The time limit does not depend on odometry, so
it still bounds the run when scan matching under-reports distance (072530 lost 10.8 m).
An invalid odometry distance (non-finite or going backwards) aborts with its own reason.

Distance counting (review F4): the estimator's distance_m is a path length that also grows with
scan-matching jitter while the vehicle stands (10/10 replay: 2.6 cm/s, 1.99-4.57 cm/s per record).
The Backstop therefore counts an increment of distance_m only when the snapshot of that tick
reports motion (sensor_reports_motion: not 'still', or |speed| > MOTION_SPEED_M_S); increments
seen while the estimator reports stillness are only recorded as 'still_excluded_m'. Real motion
clears 'still' as soon as the pose moved > 2.5 cm / 0.6 deg within the 0.45 s window, so at most
about 2.5 cm per start from rest is not counted (late stop, small against the +-2 m odometry
error); a still-flagged creep below MOTION_SPEED_M_S is bounded by the time backstop.

Single-row runs (mode 'single_row', start_single_row): the row length is optional. The distance limit is the safety
cap single_row_max_row_length_m, or min(cap, row_len - margin) with an entered length; the motion-time limit is
factor x that limit / the controller's lowest target speed (the run is speed-regulated, so v_measured at the start base
does not describe it). Same counting, same reasons: a backstop end is OPERATOR_OR_BACKSTOP, never a row end.

Effective stop position of the distance rule: the run ENDS when the counted distance reaches
row_len - margin; the executor then commands 0/0, so the vehicle comes to rest about one stopping
distance later: rest ~= Start + (row_len - margin) + stopping distance (stop model at 0.45 m/s:
0.45 * (0.15 + 0.25) + 0.45^2 / (2 * 1.07) = 0.28 m, i.e. about row_len - 1.7 m; SIM-in-the-loop
median -1.80 m from the true end), +- the odometry error (straight speed bias +-0.02 m/s, about
+-2 m over 50 m). The Runner reports the model value as backstop.expected_rest_m.

TurnGuard (end of this file) is the separate last check of the opt-in assisted headland turn
(turn_assist.py, default disabled); row runs never use it and OutputGuard is unchanged.
"""
import math

MOTION_SPEED_M_S = .05      # |speed| above this counts as motion even if the snapshot says 'still'


def sensor_reports_motion(snapshot, speed_threshold_m_s=MOTION_SPEED_M_S):
    """True unless the estimator snapshot reports stillness AND a speed at or below the threshold.

    Fail-safe for counting distance: a missing / non-boolean 'still' or a non-finite speed is motion.
    """
    if snapshot.get('still') is not True:
        return True
    speed = snapshot.get('speed', 0.)
    try:
        speed = float(speed)
    except (TypeError, ValueError):
        return True
    return not (math.isfinite(speed) and abs(speed) <= speed_threshold_m_s)


class MotionOdometer:
    """Estimator path length counted only while the estimator reports motion (review F4).

    update(distance_m, motion) -> True, or False for an invalid distance (non-finite or more than
    1 cm below the largest value seen); counted_m / excluded_m are the counted / still-time parts.
    """

    TOLERANCE_M = .01

    def __init__(self, distance0):
        distance0 = float(distance0)
        if not math.isfinite(distance0):
            raise ValueError('distance0 must be finite')
        self.distance0 = distance0
        self.last_m = distance0
        self.excluded_m = 0.

    @property
    def counted_m(self):
        # (largest distance - start) - excluded: identical to the pre-F4 rule when nothing was excluded
        return (self.last_m - self.distance0) - self.excluded_m

    def update(self, distance_m, motion):
        if distance_m is None:
            return False
        try:
            distance_m = float(distance_m)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(distance_m) or distance_m < self.last_m - self.TOLERANCE_M:
            return False
        if distance_m > self.last_m:
            if not motion:
                self.excluded_m += distance_m - self.last_m
            self.last_m = distance_m
        return True


class OutputGuard:
    def __init__(self, duty_cap_pct, diff_cap_pct, inner_min_pct=None, rolling_inner_min_pct=None):
        for name, value in (('duty_cap_pct', duty_cap_pct), ('diff_cap_pct', diff_cap_pct)):
            if type(value) is not int or not 0 <= value <= 100:
                raise ValueError('%s must be an int in [0, 100]' % name)
        for name, value in (('inner_min_pct', inner_min_pct), ('rolling_inner_min_pct', rolling_inner_min_pct)):
            if value is not None and (type(value) is not int or not 0 <= value <= 100):
                raise ValueError('%s must be None or an int in [0, 100]' % name)
        if (rolling_inner_min_pct is not None and inner_min_pct is not None
                and rolling_inner_min_pct > inner_min_pct):
            raise ValueError('rolling_inner_min_pct must not exceed inner_min_pct')
        self.duty_cap_pct = duty_cap_pct
        self.diff_cap_pct = diff_cap_pct
        self.inner_min_pct = inner_min_pct
        self.rolling_inner_min_pct = rolling_inner_min_pct
        self.violations = 0

    @classmethod
    def from_config(cls, cfg):
        return cls(int(cfg.duty_cap_pct), int(cfg.diff_cap_pct), cfg.guard_inner_min_pct(),
                   cfg.guard_rolling_inner_min_pct())

    def inner_min(self, rolling=False):
        """The inner-wheel minimum that applies: rolling only when stated AND a rolling minimum exists."""
        if rolling is True and self.rolling_inner_min_pct is not None:
            return self.rolling_inner_min_pct
        return self.inner_min_pct

    def check(self, left, right, rolling=False):
        """None when (left, right) may be sent, else a short violation name."""
        if type(left) is not int or type(right) is not int:
            return 'non_integer_duty'
        if left < 0 or right < 0:
            if left < 0 and right < 0:
                return 'reverse_command'
            if left > 0 or right > 0:
                return 'pivot_command'
            return 'negative_duty'
        if left == 0 and right == 0:
            return None
        if max(left, right) > self.duty_cap_pct:
            return 'duty_above_cap'
        if abs(left - right) > self.diff_cap_pct:
            return 'differential_above_cap'
        inner_min = self.inner_min(rolling)
        if inner_min is not None and min(left, right) < inner_min:
            return 'inner_below_breakaway'
        return None

    def apply(self, left, right, rolling=False):
        """(left, right, None) when allowed, else (0, 0, violation)."""
        violation = self.check(left, right, rolling)
        if violation is None:
            return left, right, None
        self.violations += 1
        return 0, 0, violation


class SteerOutputGuard:
    """Last check of a front-steer command (throttle %, steer_mrad), independent of the controller like OutputGuard.

      throttle    a Python int (bool refused), 0 <= throttle <= duty_cap_pct: forward only, both sides get this duty
                  (host drive_lr_mode equal_duty); there is no inner-wheel rule
      steer_mrad  a Python int (bool refused; a float, NaN or inf is a violation: the executor sends integer mrad), inside
                  the equivalent soft limit -limit_right_mrad <= steer_mrad <= limit_left_mrad (the host refuses anything
                  else with invalid_drive)
      rate        |steer_mrad - previous| <= rate x dt + RATE_SLACK_MRAD, rate = rate_deg_s while the drive command
                  (this one or the previous one) is not 0, else rate_standstill_deg_s (the MCU's own rule: moving means a
                  drive duty other than 0, MCU CFG rmov / rstd). previous = the last command that passed apply() or was
                  recorded by hold() (the Runner's held target while PAUSED / ENDED); dt = the time since then. No
                  previous command: no rate check (SHADOW without steering feedback).
    A violation returns (0, previous steer) and its name; the Runner ends the run ('guard_violation:<name>', an
    immediate STOP). check() is stateless; apply() and hold() record the output.
    """

    RATE_SLACK_MRAD = 1

    def __init__(self, duty_cap_pct, limit_left_mrad, limit_right_mrad, rate_deg_s, rate_standstill_deg_s):
        for name, value, low, high in (('duty_cap_pct', duty_cap_pct, 0, 100),
                                       ('limit_left_mrad', limit_left_mrad, 1, 1570),
                                       ('limit_right_mrad', limit_right_mrad, 1, 1570)):
            if type(value) is not int or not low <= value <= high:
                raise ValueError('%s must be an int in [%d, %d]' % (name, low, high))
        for name, value in (('rate_deg_s', rate_deg_s), ('rate_standstill_deg_s', rate_standstill_deg_s)):
            if not (isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                    and value > 0):
                raise ValueError('%s must be a positive finite number' % name)
        if rate_standstill_deg_s > rate_deg_s:
            raise ValueError('rate_standstill_deg_s must not exceed rate_deg_s')
        self.duty_cap_pct = duty_cap_pct
        self.limit_left_mrad, self.limit_right_mrad = limit_left_mrad, limit_right_mrad
        self.rate_mrad_s = math.radians(float(rate_deg_s)) * 1000.
        self.rate_standstill_mrad_s = math.radians(float(rate_standstill_deg_s)) * 1000.
        self.violations = 0
        self.last_t = None
        self.last_steer = None
        self.last_throttle = 0

    @classmethod
    def from_config(cls, cfg):
        return cls(int(cfg.duty_cap_pct), int(cfg.steer_limit_left_mrad), int(cfg.steer_limit_right_mrad),
                   float(cfg.steer_rate_limit_deg_s), float(cfg.steer_rate_limit_standstill_deg_s))

    def rate_limit_mrad_s(self, throttle):
        """The slew limit that applies to a command with this throttle after the recorded one (mrad/s)."""
        moving = (type(throttle) is int and throttle != 0) or self.last_throttle != 0
        return self.rate_mrad_s if moving else self.rate_standstill_mrad_s

    def check(self, now, throttle, steer_mrad):
        """None when (throttle, steer_mrad) may be sent at time now, else a short violation name."""
        if type(throttle) is not int:
            return 'non_integer_throttle'
        if throttle < 0:
            return 'reverse_command'
        if throttle > self.duty_cap_pct:
            return 'duty_above_cap'
        if type(steer_mrad) is not int:
            return 'non_integer_steer'
        if not -self.limit_right_mrad <= steer_mrad <= self.limit_left_mrad:
            return 'steer_outside_soft_limit'
        if self.last_steer is not None and self.last_t is not None:
            dt = float(now) - self.last_t
            if dt < 0:
                return 'clock_regression'
            if abs(steer_mrad - self.last_steer) > self.rate_limit_mrad_s(throttle) * dt + self.RATE_SLACK_MRAD:
                return 'steer_rate_above_limit'
        return None

    def apply(self, now, throttle, steer_mrad):
        """(throttle, steer_mrad, None) when allowed (recorded), else (0, previous steer, violation) (not recorded)."""
        violation = self.check(now, throttle, steer_mrad)
        if violation is not None:
            self.violations += 1
            return 0, self.last_steer, violation
        self.last_t, self.last_steer, self.last_throttle = float(now), steer_mrad, throttle
        return throttle, steer_mrad, None

    def hold(self, now, steer_mrad):
        """Record a zero-throttle output that keeps steer_mrad (or nothing, None) as the previous command."""
        self.last_t, self.last_throttle = float(now), 0
        if steer_mrad is not None:
            self.last_steer = int(steer_mrad)


class Backstop:
    DISTANCE = 'row_distance_backstop'
    TIME = 'row_time_backstop'
    ODOMETRY = 'backstop_odometry_invalid'

    def __init__(self, margin_m=2., time_factor=1.15, min_run_m=1., max_row_len_m=300.):
        self.margin_m = float(margin_m)
        self.time_factor = float(time_factor)
        self.min_run_m = float(min_run_m)
        self.max_row_len_m = float(max_row_len_m)
        self.active = False
        self.limit_m = None
        self.time_limit_s = None
        self.distance0 = 0.
        self.travelled_m = 0.
        self.odometer = None
        self.motion_time_s = 0.
        self.last_t = None
        self.last_moving = True
        self.reason = None
        self.single_row = None              # start_single_row() bounds (mode 'single_row' runs only)

    @classmethod
    def from_config(cls, cfg):
        return cls(cfg.backstop_margin_m, cfg.backstop_time_factor, cfg.min_run_distance_m, cfg.max_row_len_m)

    def start(self, t, distance_m, row_len_m, v_measured_m_s=None):
        """Arm for one run. row_len_m: operator-measured length from the start pose to the row end."""
        if not (isinstance(row_len_m, (int, float)) and not isinstance(row_len_m, bool) and math.isfinite(row_len_m)):
            raise ValueError('row_len_m must be a finite number')
        if row_len_m > self.max_row_len_m:
            raise ValueError('row_len_m above max_row_len_m')
        limit = row_len_m - self.margin_m
        if limit < self.min_run_m:
            raise ValueError('row_len_m - margin is shorter than min_run_distance_m')
        if v_measured_m_s is not None and not (math.isfinite(v_measured_m_s) and v_measured_m_s > 0):
            raise ValueError('v_measured_m_s must be a positive finite number or None')
        if not math.isfinite(distance_m):
            raise ValueError('distance_m must be finite at start')
        self.active = True
        self.single_row = None
        self.limit_m = float(limit)
        self.time_limit_s = None if v_measured_m_s is None else self.time_factor * limit / float(v_measured_m_s)
        self.distance0 = float(distance_m)
        self.travelled_m = 0.
        self.odometer = MotionOdometer(distance_m)
        self.motion_time_s = 0.
        self.last_t = float(t)
        self.last_moving = True
        self.reason = None

    def start_single_row(self, t, distance_m, cap_m, ref_speed_m_s, row_len_m=None):
        """Arm for one single-row run (mode 'single_row'; the row length is optional, PWM is not speed).

        distance limit = min(cap_m, row_len_m - margin) (cap_m alone without a row length): the safety cap of a run;
        an entered length keeps the L - margin bound of start() as an additional bound.
        time limit = time_factor x distance limit / ref_speed_m_s (the controller's lowest target speed; v_measured is
        the speed at the START base and does not describe a speed-regulated run).
        check() and its reasons are those of start(): a backstop end is never a row end."""
        for name, value in (('cap_m', cap_m), ('ref_speed_m_s', ref_speed_m_s), ('distance_m', distance_m)):
            if not (isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)):
                raise ValueError('%s must be a finite number' % name)
        if ref_speed_m_s <= 0:
            raise ValueError('ref_speed_m_s must be positive')
        if cap_m > self.max_row_len_m:
            raise ValueError('cap_m above max_row_len_m')
        limit = float(cap_m)
        if row_len_m is not None:
            if not (isinstance(row_len_m, (int, float)) and not isinstance(row_len_m, bool)
                    and math.isfinite(row_len_m)):
                raise ValueError('row_len_m must be a finite number or None')
            if row_len_m > self.max_row_len_m:
                raise ValueError('row_len_m above max_row_len_m')
            if row_len_m - self.margin_m < self.min_run_m:
                raise ValueError('row_len_m - margin is shorter than min_run_distance_m')
            limit = min(limit, float(row_len_m) - self.margin_m)
        if limit < self.min_run_m:
            raise ValueError('single-row distance cap is shorter than min_run_distance_m')
        self.active = True
        self.limit_m = limit
        self.time_limit_s = self.time_factor * limit / float(ref_speed_m_s)
        self.distance0 = float(distance_m)
        self.travelled_m = 0.
        self.odometer = MotionOdometer(distance_m)
        self.motion_time_s = 0.
        self.last_t = float(t)
        self.last_moving = True
        self.reason = None
        self.single_row = dict(cap_m=round(float(cap_m), 3), ref_speed_m_s=round(float(ref_speed_m_s), 3),
                               row_len_m=None if row_len_m is None else round(float(row_len_m), 3),
                               bound='row_length' if row_len_m is not None and limit < float(cap_m) else 'cap')

    def abort(self, reason):
        """Latch a backstop reason (e.g. invalid odometry); check() keeps returning it."""
        if self.reason is None:
            self.reason = str(reason)
        return self.reason

    def check(self, t, distance_m, moving, sensor_motion=True):
        """Update with the current odometry distance and motion flags; reason or None.

        moving         motion-time flag (non-zero command or no sensor stillness), see the module doc
        sensor_motion  the estimator reports motion this tick (sensor_reports_motion): only then does a
                       distance_m increment count towards the distance limit (review F4). The default True
                       counts every increment (the behaviour before F4).
        """
        if not self.active:
            return None
        if self.reason is not None:
            return self.reason
        dt = 0. if self.last_t is None else max(0., float(t) - self.last_t)
        if moving or self.last_moving:
            self.motion_time_s += dt
        self.last_t, self.last_moving = float(t), bool(moving)
        if not self.odometer.update(distance_m, sensor_motion is not False):
            return self.abort(self.ODOMETRY)
        self.travelled_m = self.odometer.counted_m
        if self.travelled_m >= self.limit_m:
            return self.abort(self.DISTANCE)
        if self.time_limit_s is not None and self.motion_time_s >= self.time_limit_s:
            return self.abort(self.TIME)
        return None

    def summary(self):
        out = dict(limit_m=self.limit_m, travelled_m=round(self.travelled_m, 3),
                   still_excluded_m=None if self.odometer is None else round(self.odometer.excluded_m, 3),
                   time_limit_s=None if self.time_limit_s is None else round(self.time_limit_s, 2),
                   motion_time_s=round(self.motion_time_s, 2), reason=self.reason)
        if self.single_row is not None:
            out['single_row'] = dict(self.single_row)      # start_single_row() only (row runs: unchanged summary)
        return out


# --- assisted headland turn (TurnAssist only; OutputGuard and the row runs are unchanged) ----------
PIVOT, FORWARD = 'pivot', 'forward'


class TurnGuard:
    """Independent last check for TurnAssist outputs (turn runs only, opt-in; never used by a row run).

    Shapes (direction s is fixed at construction: +1 = left / CCW, -1 = right / CW):
      (0, 0)   always allowed (also accounts the zero time used by the shape-change rule)
      pivot    exactly (-s*d, +s*d) with ints pivot_min_pct <= d <= pivot_cap_pct <= 100
               (left turn = left wheels backwards, right wheels forwards; 10/10 [-100, 100] = CCW)
      forward  both ints in [drive_min_pct, drive_cap_pct], |left - right| <= drive_diff_cap_pct
    Phase binding: phase PIVOT allows only pivot or zero, FORWARD only forward or zero, anything else
    only zero. A pivot in the other direction, reverse, one wheel only, mixed signs or unequal pivot
    magnitudes are violations.
    Transitions (monotonic time of the caller):
      - a non-zero shape different from the last non-zero one (forward <-> pivot) needs (0, 0) for
        >= |d_last| / 100 + shape_change_extra_s (MCU fade 100 %/s + BREAK 120 ms + margin) AND the
        caller-reported sensor stillness >= shape_change_still_s, so the MCU reversal path
        (fade -> 120 ms -> ramp) is never used;
      - inside one continuous pivot (no zero in between) d may stay or increase by <= step_max_pct,
        at least step_interval_s after the previous change; a decrease is a violation;
      - budgets: pivot command time per turn <= pivot_total_s; stall-loaded time (pivot commanded,
        |yaw rate| < stall_rate_deg_s, i.e. not turning at all) per pivot step <= stall_loaded_total_s
        (reset_step()).
    A violation returns (0, 0) and its name; TurnAssist ends the run ('turn_guard_violation:<name>').
    shape_problem() is the stateless part (used again by the signed bridge client, defence in depth).
    """

    def __init__(self, direction, pivot_min_pct, pivot_cap_pct, drive_min_pct, drive_cap_pct, drive_diff_cap_pct,
                 step_max_pct=10, step_interval_s=1., shape_change_extra_s=.32, shape_change_still_s=.4,
                 pivot_total_s=30., stall_loaded_total_s=4.5, stall_rate_deg_s=1.5):
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError('direction must be -1 or +1')
        for name, value in (('pivot_min_pct', pivot_min_pct), ('pivot_cap_pct', pivot_cap_pct),
                            ('drive_min_pct', drive_min_pct), ('drive_cap_pct', drive_cap_pct),
                            ('drive_diff_cap_pct', drive_diff_cap_pct), ('step_max_pct', step_max_pct)):
            if type(value) is not int or not 0 <= value <= 100:
                raise ValueError('%s must be an int in [0, 100]' % name)
        if not 1 <= pivot_min_pct <= pivot_cap_pct:
            raise ValueError('need 1 <= pivot_min_pct <= pivot_cap_pct')
        if not 1 <= drive_min_pct <= drive_cap_pct:
            raise ValueError('need 1 <= drive_min_pct <= drive_cap_pct')
        for name, value in (('step_interval_s', step_interval_s), ('shape_change_extra_s', shape_change_extra_s),
                            ('shape_change_still_s', shape_change_still_s), ('pivot_total_s', pivot_total_s),
                            ('stall_loaded_total_s', stall_loaded_total_s), ('stall_rate_deg_s', stall_rate_deg_s)):
            if not (isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                    and value >= 0):
                raise ValueError('%s must be a finite number >= 0' % name)
        self.direction = direction
        self.pivot_min_pct, self.pivot_cap_pct = pivot_min_pct, pivot_cap_pct
        self.drive_min_pct, self.drive_cap_pct = drive_min_pct, drive_cap_pct
        self.drive_diff_cap_pct = drive_diff_cap_pct
        self.step_max_pct, self.step_interval_s = step_max_pct, float(step_interval_s)
        self.shape_change_extra_s = float(shape_change_extra_s)
        self.shape_change_still_s = float(shape_change_still_s)
        self.pivot_total_s, self.stall_loaded_total_s = float(pivot_total_s), float(stall_loaded_total_s)
        self.stall_rate_deg_s = float(stall_rate_deg_s)
        self.violations = 0
        self.last_violation = None
        self.last_t = None
        self.last_out = (0, 0)
        self.last_shape = None          # shape of the last NON-ZERO output
        self.last_duty = 0              # max |duty| of the last non-zero output
        self.zero_since = None          # time of the first zero after the last non-zero output
        self.pivot_d = None             # duty of the current continuous pivot (None after any zero)
        self.pivot_d_since = None
        self.pivot_time_s = 0.
        self.stall_loaded_s = 0.

    def shape(self, left, right):
        """'zero' / PIVOT / FORWARD, or None for a shape that is never allowed."""
        if type(left) is not int or type(right) is not int:
            return None
        if left == 0 and right == 0:
            return 'zero'
        if abs(left) == abs(right) and left == -self.direction * abs(left) and right == self.direction * abs(right):
            return PIVOT
        if left > 0 and right > 0:
            return FORWARD
        return None

    def shape_problem(self, left, right):
        """Stateless check of one output: None when its shape and magnitude are allowed in some phase."""
        if type(left) is not int or type(right) is not int:
            return 'non_integer_duty'
        if left == 0 and right == 0:
            return None
        if left < 0 < right or right < 0 < left:
            if abs(left) != abs(right):
                return 'pivot_unequal'
            if self.shape(left, right) != PIVOT:
                return 'pivot_wrong_direction'
            if not self.pivot_min_pct <= abs(left) <= self.pivot_cap_pct:
                return 'pivot_duty_out_of_range'
            return None
        if left < 0 or right < 0:
            return 'reverse_command'
        if left == 0 or right == 0:
            return 'one_side_only'
        if max(left, right) > self.drive_cap_pct:
            return 'drive_above_cap'
        if min(left, right) < self.drive_min_pct:
            return 'drive_below_breakaway'
        if abs(left - right) > self.drive_diff_cap_pct:
            return 'drive_differential_above_cap'
        return None

    def reset_step(self):
        """New pivot step (PIVOT_1 / PIVOT_2): restart the per-step stall-loaded budget."""
        self.stall_loaded_s = 0.

    def check(self, now, left, right, phase, still_s=0., yaw_rate_deg_s=0.):
        """None when (left, right) may be sent at time now in this phase (no state change)."""
        problem = self.shape_problem(left, right)
        if problem is not None:
            return problem
        shape = self.shape(left, right)
        if shape == 'zero':
            return None
        if phase not in (PIVOT, FORWARD):
            return 'output_in_stop_phase'
        if shape != phase:
            return 'shape_not_allowed_in_phase'
        if self.last_t is not None and now < self.last_t:
            return 'clock_regression'
        dt = 0. if self.last_t is None else now - self.last_t
        if self.last_out != (0, 0):
            if self.shape(*self.last_out) != shape:
                return 'shape_change_without_stop'
        elif self.last_shape is not None and self.last_shape != shape:
            need = self.last_duty / 100. + self.shape_change_extra_s
            if self.zero_since is None or now - self.zero_since < need - 1e-9:
                return 'shape_change_zero_too_short'
            if not still_s >= self.shape_change_still_s:
                return 'shape_change_not_still'
        if shape == PIVOT:
            d = abs(left)
            prev_pivot = self.last_out != (0, 0)
            if prev_pivot and self.pivot_d is not None:
                if d < self.pivot_d:
                    return 'pivot_duty_decrease'
                if d > self.pivot_d:
                    if d - self.pivot_d > self.step_max_pct:
                        return 'pivot_duty_step_too_large'
                    if now - self.pivot_d_since < self.step_interval_s - 1e-9:
                        return 'pivot_duty_step_too_soon'
            if prev_pivot and self.pivot_time_s + dt > self.pivot_total_s:
                return 'pivot_time_budget'
            if prev_pivot and abs(yaw_rate_deg_s) < self.stall_rate_deg_s and \
                    self.stall_loaded_s + dt > self.stall_loaded_total_s:
                return 'stall_loaded_budget'
            # review fix: an exhausted budget also refuses the FIRST tick of a new pivot (after a zero)
            if not prev_pivot and self.pivot_time_s >= self.pivot_total_s:
                return 'pivot_time_budget'
            if not prev_pivot and self.stall_loaded_s >= self.stall_loaded_total_s:
                return 'stall_loaded_budget'
        return None

    def apply(self, now, left, right, phase, still_s=0., yaw_rate_deg_s=0.):
        """(left, right, None) when allowed (state updated), else (0, 0, violation) (the zero is recorded)."""
        now = float(now)
        violation = self.check(now, left, right, phase, still_s, yaw_rate_deg_s)
        if violation is not None:
            self.violations += 1
            self.last_violation = violation
            left, right = 0, 0
        self._account(now, left, right, yaw_rate_deg_s)
        return left, right, violation

    def _account(self, now, left, right, yaw_rate_deg_s):
        dt = 0. if self.last_t is None else max(0., now - self.last_t)
        if self.last_out != (0, 0) and self.shape(*self.last_out) == PIVOT:
            self.pivot_time_s += dt
            if abs(yaw_rate_deg_s) < self.stall_rate_deg_s:
                self.stall_loaded_s += dt
        if (left, right) == (0, 0):
            if self.last_out != (0, 0) or self.zero_since is None:
                self.zero_since = now
            self.pivot_d = None
        else:
            shape = self.shape(left, right)
            if shape == PIVOT:
                d = abs(left)
                if self.pivot_d is None or d != self.pivot_d:
                    self.pivot_d, self.pivot_d_since = d, now
            else:
                self.pivot_d = None
            self.last_shape = shape
            self.last_duty = max(abs(left), abs(right))
            self.zero_since = None
        self.last_out = (left, right)
        self.last_t = now

    def summary(self):
        return dict(violations=self.violations, last_violation=self.last_violation,
                    pivot_time_s=round(self.pivot_time_s, 2), stall_loaded_s=round(self.stall_loaded_s, 2),
                    last_shape=self.last_shape)

    @classmethod
    def from_settings(cls, settings, direction):
        """From turn_assist.TurnSettings (validated config values)."""
        return cls(direction, settings.pivot_start_pct, settings.pivot_cap_pct, settings.drive_min_pct,
                   settings.drive_cap_pct, settings.drive_diff_cap_pct, settings.pivot_step_pct,
                   settings.pivot_step_interval_s, settings.shape_change_extra_s, settings.shape_change_still_s,
                   settings.pivot_total_s, settings.stall_loaded_total_s, settings.stall_rate_deg_s)
