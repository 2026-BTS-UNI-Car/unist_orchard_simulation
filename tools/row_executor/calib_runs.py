"""Calibration run kinds 'arc' (C5) and 'rolldown' (C3b): bounded duty patterns, field profile, MOTOR only.

Why (review_correctness_day1 F2): the dashboard manual mixer only makes (s, 1.5 s) pairs, so the yaw authority at
the differential the executor really uses (C5) can not be measured with it, and the bench pattern must never run
on the ground. These two run kinds go through the SAME hold-to-run, freshness, stop and bridge paths as a row run,
but command a fixed, bounded duty table instead of the row controller. No numpy, no ROS, no I/O.

  arc       forward only. Steps from a bounded table of (left, right) duty pairs around the base duty, e.g.
            base 55, set 'd10_d20' = (55,45) (45,55) (55,35) (35,55) (R > L = CCW = +yaw). Each step: settle, then
            measure for step_s seconds or until 0.9 x distance_cap / steps metres were counted. Result per step: gyro
            yaw, LiDAR yaw, speed, gain = yaw / (R - L) [deg/s per %].
  rolldown  forward only, both wheels equal. Start at the base duty from standstill; once rolling, step the duty
            down by step_pct every hold_s seconds down to the floor. The lowest duty after which the vehicle kept
            moving is the rolling threshold candidate (C3b, config rolling_breakaway_duty_pct). A stall ends the run.
  stop      (C7, review_safety_day2 N8) forward only, both wheels at the base duty: once moving, settle STOP_SETTLE_S,
            measure steady_s seconds, then the run ends by itself ('calib_stop_test') so the executor commands its own
            0/0 (MCU fade, the same zero command a row end or a hold release sends). The stop that follows is observed
            (observe_stop: counted distance from the 0/0 tick to rest) and becomes measured_stop_distance_m.

Minimum duties (replace the inner-wheel rule for these runs; review_correctness_day2 R1, day-2 fix):
  start floor  both wheels from standstill: the measured static breakaway_duty_pct (no margin, nothing subtracted),
               UNKNOWN_BREAKAWAY_FLOOR_PCT (40) when it is not measured. The base duty must reach it (else refused).
  arc floor    the inner wheel of a pair: the arc starts at (base, base) and switches to its first pair only once the
               vehicle rolls (speed >= ARC_SWITCH_SPEED_M_S, review_safety_day2 N4: no pivot-like start on one wheel),
               so a pair needs the ROLLING threshold when it is measured (rolling_breakaway_duty_pct, with the static
               one), else the start floor.
  rolldown     equal wheels after >= 2 s of rolling at the base: steps go down to floor_pct (operator, ROLLDOWN_MIN_
               FLOOR_PCT 20 .. base - step, default ROLLDOWN_DEFAULT_FLOOR_PCT 25), BELOW the static breakaway, so a
               rolling threshold under it is measurable (the previous floor = breakaway made C3b unmeasurable whenever
               C2 was in the config). No pivot risk (equal duty); a stall ends the run, the yaw cap 6 deg/s and the
               heading cap 20 deg stay.
  No calibration duty is ever below CALIB_MIN_DUTY_PCT (20). An own OutputGuard(duty_cap, diff_cap, inner_min = the
  plan's lowest duty) checks every output. Pairs or steps below their floor are refused at Start (never clipped).

Terminal (the run ENDS, never pauses; a new Start + Arm + press is needed; reasons start with 'calib_' or
'rolldown_'):
  any pause (hold released, stale LiDAR/IMU, loop stall) -> 'calib_<why>'       (Runner)
  front clearance below the vendored stop model (required_front(speed))        'calib_front_blocked'
  side clearance <= side_hard_margin_m on side_confirm_scans distinct scans     'calib_side_blocked' (same rule and
      yaw-sweep compensation as the vendored continuous_side controller)
  speed > max_speed_m_s + OVERSPEED_MARGIN_M_S                                  'calib_overspeed'
  |gyro yaw rate| > the kind's cap (arc 12, rolldown 6 deg/s: an inner wheel stalling turns the arc into a pivot)
                                                                                'calib_yaw_rate'
  |heading change| (bias-corrected gyro integral) above the caps                'calib_heading_change'
  no motion within NO_RESPONSE_S of the first press                            'calib_no_response'
  LiDAR odometry invalid for ODOM_LOST_S while commanding                       'calib_odometry_lost' (the distance
      cap needs it; run calibrations where the LiDAR sees structures)
  odometry going backwards / non-finite                                         'calib_odometry_invalid'
  counted distance >= distance cap / elapsed >= time cap                        'calib_distance_cap' / 'calib_time_cap'
  arc: the vehicle stops while commanded (still or speed < STALL_SPEED_M_S for STALL_S)   'calib_arc_stalled'
  arc / stop: the vehicle moved at (base, base) and stopped again before its pair / measurement  'calib_start_stalled'
  output guard violation (a bug)                                                'guard_violation:<what>' (Runner)
Normal ends: 'calib_complete' (arc), 'rolldown_stalled' (stall at a step below the base: the measurement end),
'rolldown_floor_reached', 'rolldown_stalled_at_base' (the base duty does not keep the vehicle rolling),
'calib_stop_test' (stop kind: the executor's own 0/0 starts the C7 measurement).
Distance is counted only while the estimator reports motion (review F4, guard.MotionOdometer).
Phase tags (review_correctness_day2 R5): the step index changes only after the tick record, so the tick that closes a
step is still tagged (i, 'measure') and the next step starts with 'settle': a step's 'measure' ticks are exactly the
ticks of its measurement window (calib_analysis recomputes from them; before, the closing tick was tagged with the
NEXT step's index and 'measure', so the recomputation of every later step began one step early).
"""
from dataclasses import dataclass, field
import math

from .guard import MotionOdometer, OutputGuard, sensor_reports_motion

KINDS = ('arc', 'rolldown', 'stop')
UNKNOWN_BREAKAWAY_FLOOR_PCT = 40          # floor when breakaway_duty_pct (C2) is not measured
CALIB_MIN_DUTY_PCT = 20                   # no calibration duty below this (the C2 ladder starts at 20 %)
ROLLDOWN_MIN_FLOOR_PCT = 20               # rolldown floor_pct range: 20 .. base - step (review_correctness_day2 R1)
ROLLDOWN_DEFAULT_FLOOR_PCT = 25
ARC_SWITCH_SPEED_M_S = .10                # arc: (base, base) until the vehicle rolls this fast, then the pairs (N4)
STOP_SETTLE_S = 1.5                       # stop kind: after the first motion, before the steady window
STOP_STEADY_S = (2., 6.)                  # stop kind: steady window (operator), then the executor's own 0/0
STOP_STEADY_S_DEFAULT = 3.
STOP_REST_TIMEOUT_S = 6.                  # observe_stop: no rest within this after the 0/0 -> 'not_observed'
STOP_TEST_WAIT_S = 4.                     # executor: 0/0 (fade, lease kept) this long before the op stop after a
                                          # 'calib_stop_test' end (instead of terminal_stop_wait_s 1.5 s), so the
                                          # observed stop is the pure MCU fade the C7 value stands for
# (left, right) offsets from the base duty. R > L turns left (+yaw, CCW; 10/03 measurement).
ARC_SETS = {
    'd10': ((0, -10), (-10, 0)),
    'd20': ((0, -20), (-20, 0)),
    'd10_d20': ((0, -10), (-10, 0), (0, -20), (-20, 0)),          # WEEKEND task example (55,45) (45,55) (55,35) (35,55)
    'sym10': ((5, -5), (-5, 5)),                                  # mean = base (what the controller does: base +- d)
    'sym20': ((10, -10), (-10, 10)),
    'sym10_sym20': ((5, -5), (-5, 5), (10, -10), (-10, 10)),
}
ARC_DEFAULT_SET = 'd10_d20'
ARC_STEP_S = (3., 8.)                     # measured seconds per arc step (after the settle)
ARC_STEP_S_DEFAULT = 4.
ARC_STEP_DISTANCE_SHARE = .9              # per-step distance = 0.9 x distance cap / steps (the cap stays a backstop)
ARC_MIN_STEP_M = 2.                       # less leaves no measurement after the settle at ~0.45 m/s
ARC_SETTLE_S = 1.5                        # after a step change (MCU ramp 100 %/s + estimator median 0.5 s)
ARC_MAX_YAW_DPS = 12.
ARC_STEP_HEADING_DEG = 60.
ARC_RUN_HEADING_DEG = 90.
ROLL_STEP_PCT = (2, 3, 4, 5)
ROLL_STEP_PCT_DEFAULT = 3
ROLL_HOLD_S = (2., 5.)                    # seconds per rolldown step below the base
ROLL_HOLD_S_DEFAULT = 3.
ROLL_FIRST_HOLD_S = 2.                    # rolling at the base this long before the first step down
ROLL_SETTLE_S = 1.                        # first part of a step not used for the speed statistics
ROLL_MAX_YAW_DPS = 6.
ROLL_RUN_HEADING_DEG = 20.
MAX_STEPS = 16
DISTANCE_MIN_M = 2.
TIME_MIN_S = 5.
NO_RESPONSE_S = 3.                        # MCU ramp 1 s + grace 1 s + 1 s (as drive_no_response)
MOVED_SPEED_M_S = .05                     # motion seen: estimator reports motion and speed >= this
STALL_SPEED_M_S = .05                     # stall: still, or odometry valid and speed below this ...
STALL_S = .6                              # ... for this long
ODOM_LOST_S = 1.
OVERSPEED_MARGIN_M_S = .15
MIN_MEASURE_S = 1.5                       # a step result is 'valid' with at least this much measurement
LIDAR_OK_FRACTION = .8                    # LiDAR yaw of a step only when this fraction of its scans matched
LIDAR_TICK_WINDOW_S = .5                  # per-tick LiDAR yaw window (logging)
PARAM_KEYS = dict(arc=('arc_set', 'step_s', 'distance_m', 'time_s'),
                  rolldown=('step_pct', 'hold_s', 'floor_pct', 'distance_m', 'time_s'),
                  stop=('steady_s', 'distance_m', 'time_s'))


class CalibPlanError(ValueError):
    def __init__(self, problems):
        self.problems = list(problems)
        super().__init__('; '.join(self.problems))


def start_floor_pct(cfg):
    """(minimum duty of both wheels from standstill, source): the measured static breakaway itself, else 40."""
    if cfg.breakaway_duty_pct is not None:
        return max(CALIB_MIN_DUTY_PCT, int(cfg.breakaway_duty_pct)), 'breakaway_duty_pct'
    return UNKNOWN_BREAKAWAY_FLOOR_PCT, 'unknown_breakaway_default'


def floor_pct(cfg):
    """(minimum duty of the inner wheel of an arc pair, source): the pairs start only once the vehicle rolls, so the
    measured rolling threshold (with the static one measured too), else the start floor."""
    if cfg.rolling_breakaway_duty_pct is not None and cfg.breakaway_duty_pct is not None:
        return max(CALIB_MIN_DUTY_PCT, int(cfg.rolling_breakaway_duty_pct)), 'rolling_breakaway_duty_pct'
    return start_floor_pct(cfg)


def _pair_problem(cfg, left, right, floor):
    if left <= 0 or right <= 0:
        return 'duty <= 0 (forward only, both wheels driving)'
    if min(left, right) < floor:
        return 'inner %d below the minimum duty %d' % (min(left, right), floor)
    if max(left, right) > cfg.duty_cap_pct:
        return 'outer %d above duty_cap_pct %d' % (max(left, right), cfg.duty_cap_pct)
    if abs(left - right) > cfg.diff_cap_pct:
        return '|L-R| %d above diff_cap_pct %d' % (abs(left - right), cfg.diff_cap_pct)
    return None


def arc_pairs(cfg, set_name):
    base = int(cfg.base_duty_pct)
    return tuple((base + dl, base + dr) for dl, dr in ARC_SETS[set_name])


def rolldown_duties(base, step, floor):
    duties = []
    d = int(base)
    while d >= floor and len(duties) < MAX_STEPS:
        duties.append(d)
        d -= step
    if duties and duties[-1] != floor and len(duties) < MAX_STEPS and base > floor:
        duties.append(int(floor))
    return tuple(duties)


@dataclass(frozen=True)
class CalibPlan:
    kind: str
    base_pct: int
    floor_pct: int
    floor_source: str
    steps: tuple                    # ((left, right), ...)
    step_s: float                   # arc: measured seconds per step; rolldown: seconds per step below the base
    step_m: float                   # arc: metres per step at most (0.9 x distance cap / steps); rolldown: inf
    distance_cap_m: float
    time_cap_s: float
    yaw_cap_dps: float
    run_heading_cap_deg: float
    step_heading_cap_deg: float
    overspeed_m_s: float
    duty_cap_pct: int
    diff_cap_pct: int
    max_differential_pct: int
    params: dict = field(default_factory=dict)
    start_floor_pct: int = UNKNOWN_BREAKAWAY_FLOOR_PCT     # both wheels from standstill (day-2 R1)
    start_floor_source: str = 'unknown_breakaway_default'

    def to_dict(self):
        return dict(kind=self.kind, base_pct=self.base_pct, floor_pct=self.floor_pct, floor_source=self.floor_source,
                    start_floor_pct=self.start_floor_pct, start_floor_source=self.start_floor_source,
                    steps=[list(p) for p in self.steps], step_s=self.step_s,
                    step_m=None if math.isinf(self.step_m) else round(self.step_m, 3),
                    distance_cap_m=self.distance_cap_m, time_cap_s=self.time_cap_s, yaw_cap_dps=self.yaw_cap_dps,
                    run_heading_cap_deg=self.run_heading_cap_deg, step_heading_cap_deg=self.step_heading_cap_deg,
                    overspeed_m_s=round(self.overspeed_m_s, 3), duty_cap_pct=self.duty_cap_pct,
                    diff_cap_pct=self.diff_cap_pct, max_differential_pct=self.max_differential_pct,
                    params=dict(self.params))


def _number(params, key, lo, hi, default, problems, integer=False):
    value = params.get(key, default)
    ok = (type(value) is int) if integer else (type(value) in (int, float) and math.isfinite(value))
    if not ok or not lo <= value <= hi:
        problems.append('%s: %r outside [%g, %g]%s' % (key, value, lo, hi, ' (integer)' if integer else ''))
        return default
    return int(value) if integer else float(value)


def make_plan(cfg, kind, params=None):
    """Validated CalibPlan from the config and the operator's bounded inputs; CalibPlanError lists every problem.

    The config must allow calibration output (field profile, calib_motor_output_enabled) - checked by the caller
    (ExecutorLoop) together with the --motor-output flag; here only the pattern itself is checked.
    """
    params = {} if params is None else params
    if kind not in KINDS:
        raise CalibPlanError(['kind: must be one of %s' % (KINDS,)])
    if not isinstance(params, dict):
        raise CalibPlanError(['params: must be an object'])
    problems = ['%s: not a %s parameter' % (k, kind) for k in sorted(set(params) - set(PARAM_KEYS[kind]))]
    if cfg.profile != 'field':
        problems.append('calibration runs need the field profile')
    floor, source = floor_pct(cfg)
    start, start_source = start_floor_pct(cfg)
    distance = _number(params, 'distance_m', DISTANCE_MIN_M, cfg.calib_distance_cap_m, cfg.calib_distance_cap_m,
                       problems)
    time_cap = _number(params, 'time_s', TIME_MIN_S, cfg.calib_time_cap_s, cfg.calib_time_cap_s, problems)
    base = int(cfg.base_duty_pct)
    if base < start:
        problems.append('base %d below the start minimum %d (%s): the vehicle would not start from standstill'
                        % (base, start, start_source))
    if kind == 'stop':
        step_s = _number(params, 'steady_s', STOP_STEADY_S[0], STOP_STEADY_S[1], STOP_STEADY_S_DEFAULT, problems)
        steps = ((base, base),)
        why = _pair_problem(cfg, base, base, start)
        if why:
            problems.append('stop (%d, %d): %s' % (base, base, why))
        step_m = math.inf
        caps = (ROLL_MAX_YAW_DPS, ROLL_RUN_HEADING_DEG, ROLL_RUN_HEADING_DEG)
        echo = dict(steady_s=step_s)
        floor, source = start, start_source
    elif kind == 'arc':
        set_name = params.get('arc_set', ARC_DEFAULT_SET)
        if set_name not in ARC_SETS:
            problems.append('arc_set: %r not one of %s' % (set_name, sorted(ARC_SETS)))
            set_name = ARC_DEFAULT_SET
        step_s = _number(params, 'step_s', ARC_STEP_S[0], ARC_STEP_S[1], ARC_STEP_S_DEFAULT, problems)
        steps = arc_pairs(cfg, set_name)
        for k, (left, right) in enumerate(steps):
            why = _pair_problem(cfg, left, right, floor)
            if why:
                problems.append('arc step %d (%d, %d): %s' % (k + 1, left, right, why))
        step_m = ARC_STEP_DISTANCE_SHARE * distance / max(1, len(steps))
        if step_m < ARC_MIN_STEP_M:
            problems.append('distance_m %g leaves %.2f m per arc step (< %g m: no measurement after the settle); '
                            'raise distance_m or choose a set with fewer steps' % (distance, step_m, ARC_MIN_STEP_M))
        caps = (ARC_MAX_YAW_DPS, ARC_RUN_HEADING_DEG, ARC_STEP_HEADING_DEG)
        echo = dict(arc_set=set_name, step_s=step_s)
    else:
        step = params.get('step_pct', ROLL_STEP_PCT_DEFAULT)
        if type(step) is not int or step not in ROLL_STEP_PCT:
            problems.append('step_pct: %r not one of %s' % (step, ROLL_STEP_PCT))
            step = ROLL_STEP_PCT_DEFAULT
        step_s = _number(params, 'hold_s', ROLL_HOLD_S[0], ROLL_HOLD_S[1], ROLL_HOLD_S_DEFAULT, problems)
        # review_correctness_day2 R1: the floor is independent of the static breakaway (equal wheels, already rolling)
        low = _number(params, 'floor_pct', ROLLDOWN_MIN_FLOOR_PCT, max(ROLLDOWN_MIN_FLOOR_PCT, base - 1),
                      ROLLDOWN_DEFAULT_FLOOR_PCT, problems, integer=True)
        if base - step < low:
            problems.append('rolldown: base %d - step %d is below the floor %d (nothing to step down to)'
                            % (base, step, low))
        duties = rolldown_duties(base, step, low)
        steps = tuple((d, d) for d in duties)
        for k, (left, right) in enumerate(steps):
            why = _pair_problem(cfg, left, right, low)
            if why:
                problems.append('rolldown step %d (%d): %s' % (k + 1, left, why))
        step_m = math.inf
        caps = (ROLL_MAX_YAW_DPS, ROLL_RUN_HEADING_DEG, ROLL_RUN_HEADING_DEG)
        echo = dict(step_pct=step, hold_s=step_s, floor_pct=low)
        floor, source = low, 'rolldown_floor'           # the guard minimum is the lowest step
    if len(steps) < (1 if kind == 'stop' else 2) or len(steps) > MAX_STEPS:
        problems.append('%d steps (need 2..%d)' % (len(steps), MAX_STEPS))
    if problems:
        raise CalibPlanError(problems)
    echo.update(distance_m=distance, time_s=time_cap)
    return CalibPlan(kind=kind, base_pct=base, floor_pct=int(floor), floor_source=source, steps=steps, step_s=step_s,
                     step_m=step_m, distance_cap_m=distance, time_cap_s=time_cap, yaw_cap_dps=caps[0],
                     run_heading_cap_deg=caps[1], step_heading_cap_deg=caps[2],
                     overspeed_m_s=float(cfg.max_speed_m_s) + OVERSPEED_MARGIN_M_S, duty_cap_pct=int(cfg.duty_cap_pct),
                     diff_cap_pct=int(cfg.diff_cap_pct), max_differential_pct=int(cfg.max_differential_pct),
                     params=echo, start_floor_pct=int(start), start_floor_source=start_source)


def plan_options(cfg):
    """What the operator page offers (every option with the duties it would command, or why it is refused)."""
    floor, source = floor_pct(cfg)
    arcs = []
    for name in ARC_SETS:
        try:
            plan = make_plan(cfg, 'arc', dict(arc_set=name))
            arcs.append(dict(id=name, steps=[list(p) for p in plan.steps], ok=True, why=None))
        except CalibPlanError as exc:
            arcs.append(dict(id=name, steps=[list(p) for p in arc_pairs(cfg, name)], ok=False, why=exc.problems[0]))
    rolls = []
    for step in ROLL_STEP_PCT:
        try:
            plan = make_plan(cfg, 'rolldown', dict(step_pct=step))
            rolls.append(dict(step_pct=step, duties=[p[0] for p in plan.steps], ok=True, why=None))
        except CalibPlanError as exc:
            rolls.append(dict(step_pct=step, duties=[], ok=False, why=exc.problems[0]))
    try:
        make_plan(cfg, 'stop')
        stop = dict(duty=int(cfg.base_duty_pct), ok=True, why=None)
    except CalibPlanError as exc:
        stop = dict(duty=int(cfg.base_duty_pct), ok=False, why=exc.problems[0])
    start, start_source = start_floor_pct(cfg)
    return dict(floor_pct=floor, floor_source=source, start_floor_pct=start, start_floor_source=start_source,
                base_pct=int(cfg.base_duty_pct), arc_sets=arcs,
                arc_default=ARC_DEFAULT_SET, arc_step_s=list(ARC_STEP_S), rolldown=rolls,
                rolldown_hold_s=list(ROLL_HOLD_S), rolldown_floor_pct=[ROLLDOWN_MIN_FLOOR_PCT, ROLLDOWN_DEFAULT_FLOOR_PCT],
                stop=stop, stop_steady_s=list(STOP_STEADY_S), distance_m=[DISTANCE_MIN_M, cfg.calib_distance_cap_m],
                time_s=[TIME_MIN_S, cfg.calib_time_cap_s])


# --- per-step statistics (shared with calib_analysis) ---------------------------------------------------------
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _r(v, nd=3):
    return None if v is None or not math.isfinite(v) else round(float(v), nd)


def lidar_yaw(scans, min_span_s=1., ok_fraction=LIDAR_OK_FRACTION):
    """LiDAR yaw rate [deg/s] from consecutive (stamp, heading_rad, odom_valid) scans: the sum of the wrapped
    heading steps between consecutive VALID scans over their time span; None when fewer than ok_fraction of the
    scans matched or the span is shorter than min_span_s. Returns (yaw_dps or None, ok fraction)."""
    if len(scans) < 2:
        return None, 0.
    ok = sum(1 for s in scans if s[2]) / len(scans)
    total, span = 0., 0.
    for a, b in zip(scans, scans[1:]):
        if a[2] and b[2] and b[0] > a[0]:
            total += wrap(b[1] - a[1])
            span += b[0] - a[0]
    if ok < ok_fraction or span < min_span_s:
        return None, ok
    return math.degrees(total / span), ok


def ls_gain(rows, key):
    """Least-squares gain through the origin of rows[key] on rows['diff'], weighted by seconds (the single-row branch's
    field_calib _ls_gain definition; that analyser is not in the steering branch): sum w y d / sum w d^2 [deg/s per % of
    R-L]."""
    num = sum(r['seconds'] * r[key] * r['diff'] for r in rows if r.get(key) is not None)
    den = sum(r['seconds'] * r['diff'] ** 2 for r in rows if r.get(key) is not None)
    return num / den if den > 0 else None


def arc_outcome(results, max_differential_pct):
    """Per-side gains and the yaw at the configured full differential (R-L = 2 max_differential, extrapolated by the
    smaller side gain: the controller needs the authority in both directions)."""
    valid = [r for r in results if r.get('valid')]
    out = dict(sides={}, steps_valid=len(valid))
    for side, sign in (('left', 1), ('right', -1)):
        rows = [r for r in valid if (r['diff'] > 0) == (sign > 0)]
        g_l = ls_gain([dict(r, y=r['lidar_yaw_dps']) for r in rows if r.get('lidar_yaw_dps') is not None], 'y')
        g_g = ls_gain([dict(r, y=r['gyro_yaw_dps']) for r in rows if r.get('gyro_yaw_dps') is not None], 'y')
        out['sides'][side] = dict(steps=len(rows), gain_lidar=_r(g_l, 4), gain_gyro=_r(g_g, 4),
                                  gain=_r(g_l if g_l is not None else g_g, 4),
                                  max_abs_diff=max((abs(r['diff']) for r in rows), default=None))
    gains = [out['sides'][s]['gain'] for s in ('left', 'right')]
    if all(g is not None for g in gains):
        g = min(gains)
        full = 2 * int(max_differential_pct)
        out.update(gain_min=_r(g, 4), full_diff_pct=full, yaw_at_full_diff_dps=_r(g * full, 2),
                   extrapolated=max(out['sides'][s]['max_abs_diff'] or 0 for s in ('left', 'right')) < full,
                   note='measured_yaw_at_max_diff_deg_s = smaller side gain x 2 x max_differential_pct')
    else:
        out.update(gain_min=None, yaw_at_full_diff_dps=None,
                   note='needs at least one valid step on EACH side (left: R > L, right: L > R)')
    return out


def rolldown_outcome(results, floor):
    """Lowest duty after which the vehicle kept rolling (rolling threshold candidate, an upper bound)."""
    sustained = [r['duty'] for r in results if r.get('sustained')]
    stalled = [r['duty'] for r in results if r.get('stalled')]
    lowest = min(sustained) if sustained else None
    out = dict(lowest_sustained_pct=lowest, stalled_at_pct=stalled[0] if stalled else None,
               floor_pct=int(floor), floor_reached=lowest is not None and lowest <= floor)
    if lowest is None:
        out['note'] = 'no step below the base completed: no rolling threshold'
    elif out['floor_reached']:
        out['note'] = ('kept rolling down to the floor %d: rolling threshold <= %d (upper bound; rolling_breakaway_'
                       'duty_pct = %d is conservative)' % (floor, lowest, lowest))
    else:
        out['note'] = ('stalled at %s after rolling at %d: rolling threshold in (%s, %d]' %
                       (out['stalled_at_pct'], lowest, out['stalled_at_pct'], lowest))
    out['rolling_breakaway_duty_pct'] = lowest
    return out


def stop_outcome(results):
    """Stop kind: the steady speed at the base duty before the executor's 0/0 (the stop itself: CalibRun.stop)."""
    steady = [r for r in results if r.get('steady')]
    r = steady[-1] if steady else None
    return dict(steady=r is not None, speed_m_s=None if r is None else r.get('speed_m_s'),
                min_speed_m_s=None if r is None else r.get('min_speed_m_s'),
                note=('steady window measured; the stop after the 0/0 is in calib.stop (calib_analysis: '
                      'measured_stop_distance_m)') if r is not None else 'no steady window: no C7 stop measurement')


class _Measure:
    """Accumulates one step's measurement window."""

    def __init__(self):
        self.t0 = self.t1 = None
        self.g0 = self.g1 = None
        self.d0 = self.d1 = None
        self.speeds = []
        self.scans = []
        self.last_scan = None

    def add(self, now, s, counted):
        if self.t0 is None:
            self.t0, self.g0, self.d0 = now, _float(s.get('gyro_heading'), 0.), counted
        self.t1, self.g1, self.d1 = now, _float(s.get('gyro_heading'), self.g1 or 0.), counted
        speed = s.get('speed')
        if s.get('odom_valid') is True and _finite(speed):
            self.speeds.append(float(speed))
        scan = s.get('scan_count')
        if scan is not None and scan != self.last_scan and _finite(s.get('lidar_age_s')):
            self.last_scan = scan
            self.scans.append((now - float(s['lidar_age_s']), _float(s.get('lidar_heading'), 0.),
                               s.get('odom_valid') is True))

    @property
    def seconds(self):
        return 0. if self.t0 is None else self.t1 - self.t0

    def result(self):
        sec = self.seconds
        gyro = math.degrees(self.g1 - self.g0) / sec if sec > 0 else None
        lidar, ok = lidar_yaw(self.scans)
        speed = sum(self.speeds) / len(self.speeds) if self.speeds else None
        return dict(seconds=_r(sec, 2), distance_m=_r(0. if self.d0 is None else self.d1 - self.d0, 3),
                    speed_m_s=_r(speed, 3), min_speed_m_s=_r(min(self.speeds) if self.speeds else None, 3),
                    gyro_yaw_dps=_r(gyro, 3), lidar_yaw_dps=_r(lidar, 3), lidar_ok_fraction=_r(ok, 2),
                    scans=len(self.scans))


def _finite(v):
    return type(v) in (int, float) and math.isfinite(v)


def _float(v, default):
    return float(v) if _finite(v) else default


class CalibRun:
    """One calibration run (state machine). step(now, snapshot) -> (left, right, end_reason or None)."""

    def __init__(self, plan, stop_model, controller_config):
        self.plan = plan
        self.stop_model = stop_model
        self.side_hard = float(controller_config.side_hard_margin_m)
        self.side_confirm = int(controller_config.side_confirm_scans)
        # explicit minimum duty instead of the inner-wheel rule (breakaway + margin): see the module docstring
        self.guard = OutputGuard(plan.duty_cap_pct, plan.diff_cap_pct, inner_min_pct=plan.floor_pct)
        self.started = False
        self.t0 = None
        self.i = 0
        self.phase = 'await_hold'
        self.step_t0 = None
        self.step_d0 = 0.
        self.step_h0 = 0.
        self.run_h0 = None
        self.odometer = None
        self.counted = 0.
        self.moved_t = None
        self.stall_since = None
        self.odom_bad_since = None
        self.side_scan = object()
        self.side_streak = 0
        self.measure = None
        self.results = []
        self.end_reason = None
        self.outcome = None
        self.last_tick = {}
        self._lidar_recent = []
        self.go_t = None                # arc: the first pair started (the vehicle rolled), review_safety_day2 N4
        self._next_pending = False      # step change after the tick record (review_correctness_day2 R5)
        self.stop = None                # the stop after the run's 0/0 (observe_stop; C7, review_safety_day2 N8)
        self._stop_ref = None

    # --- life cycle -------------------------------------------------------------------------------------
    def begin(self, now):
        """The first (and only) press: the pattern starts. Any later pause ends the run (Runner)."""
        if self.started:
            raise RuntimeError('calibration run already started')
        self.started = True
        self.t0 = self.step_t0 = float(now)
        self.phase = 'start'

    def end(self, reason):
        if self.end_reason is None:
            self.end_reason = str(reason)
            if self.measure is not None and self.started:
                self._close_step(interrupted=True)
            if self.plan.kind == 'arc':
                self.outcome = arc_outcome(self.results, self.plan.max_differential_pct)
            elif self.plan.kind == 'stop':
                self.outcome = stop_outcome(self.results)
            else:
                self.outcome = rolldown_outcome(self.results, self.plan.floor_pct)
            self.phase = 'ended'

    def observe_stop(self, now, s):
        """Runner, on the tick the run ended (the tick whose decision is the first 0/0) and every tick after it: the
        counted distance (review F4: only while the estimator reports motion) from that tick to rest = the stop
        distance of the executor's 0/0 (C7). Rest = the first tick that reports stillness; its time minus still_s
        estimates when the vehicle stopped. Result in self.stop (status, JSONL event 'calib_stop')."""
        if not self.started or self.end_reason is None or (self.stop is not None and self.stop.get('done')):
            return
        now = float(now)
        distance, speed = s.get('distance_m'), s.get('speed')
        if self._stop_ref is None:
            if not _finite(distance):
                self.stop = dict(done=True, ok=False, why='odometry_invalid', end_reason=self.end_reason)
                return
            self._stop_ref = (now, MotionOdometer(distance))
            self.stop = dict(done=False, ok=None, end_reason=self.end_reason,
                             v_at_zero_m_s=_r(float(speed) if _finite(speed) else None, 3),
                             odom_valid_at_zero=s.get('odom_valid') is True)
            return
        t0, odo = self._stop_ref
        if not odo.update(distance, sensor_reports_motion(s)):
            self.stop.update(done=True, ok=False, why='odometry_invalid')
            return
        still_s = s.get('still_s')
        if s.get('still') is True and _finite(still_s):
            self.stop.update(done=True, ok=True, stop_distance_m=_r(odo.counted_m, 3),
                             time_to_rest_s=_r(max(0., now - float(still_s) - t0), 2), excluded_m=_r(odo.excluded_m, 3))
        elif now - t0 > STOP_REST_TIMEOUT_S:
            self.stop.update(done=True, ok=False, why='not_observed', stop_distance_m=_r(odo.counted_m, 3))

    # --- the tick ------------------------------------------------------------------------------------------
    def step(self, now, s):
        now = float(now)
        if not self.started or self.end_reason is not None:
            return 0, 0, self.end_reason or 'calib_not_started'
        why = self._checks(now, s)
        if why is None:
            why = self._advance(now, s)
        self._tick_record(now, s)
        if self._next_pending:          # R5: the step changes after the record (closing tick stays (i, 'measure'))
            self._next_pending = False
            self._next_step(now, s)
        if why is not None:
            self.end(why)
            return 0, 0, why
        left, right = self._command()
        return int(left), int(right), None

    def _command(self):
        """The duty pair commanded now: an arc drives (base, base) until the vehicle rolls (review_safety_day2 N4:
        never a pair from standstill, whose inner wheel at the floor could stay put and pivot the vehicle)."""
        if self.plan.kind == 'arc' and self.go_t is None:
            return self.plan.base_pct, self.plan.base_pct
        return self.plan.steps[self.i]

    def _checks(self, now, s):
        p = self.plan
        distance = s.get('distance_m')
        if self.odometer is None:
            if not _finite(distance):
                return 'calib_odometry_invalid'
            self.odometer = MotionOdometer(distance)
            self.run_h0 = _float(s.get('gyro_heading'), 0.)
            self.step_h0 = self.run_h0
        if not self.odometer.update(distance, sensor_reports_motion(s)):
            return 'calib_odometry_invalid'
        self.counted = self.odometer.counted_m
        speed = s.get('speed')
        speed = float(speed) if _finite(speed) else 0.
        odom_ok = s.get('odom_valid') is True
        front = s.get('front_clearance_m')
        if front is not None and not front >= self.stop_model.required_front(max(0., speed)):
            return 'calib_front_blocked'
        if self._side_blocked(s):
            return 'calib_side_blocked'
        if odom_ok and speed > p.overspeed_m_s:
            return 'calib_overspeed'
        yaw = s.get('yaw_rate')
        if _finite(yaw) and abs(math.degrees(yaw)) > p.yaw_cap_dps:
            return 'calib_yaw_rate'
        heading = _float(s.get('gyro_heading'), self.run_h0)
        if (abs(math.degrees(heading - self.run_h0)) > p.run_heading_cap_deg
                or abs(math.degrees(heading - self.step_h0)) > p.step_heading_cap_deg):
            return 'calib_heading_change'
        if self.moved_t is None:
            if sensor_reports_motion(s) and odom_ok and speed >= MOVED_SPEED_M_S:
                self.moved_t = now
            elif now - self.t0 >= NO_RESPONSE_S:
                return 'calib_no_response'
        if odom_ok:
            self.odom_bad_since = None
        else:
            if self.odom_bad_since is None:
                self.odom_bad_since = now
            if now - self.odom_bad_since >= ODOM_LOST_S:
                return 'calib_odometry_lost'
        if self.counted >= p.distance_cap_m:
            return 'calib_distance_cap'
        if now - self.t0 >= p.time_cap_s:
            return 'calib_time_cap'
        return None

    def _side_blocked(self, s):
        """The vendored continuous_side hard rule (RowController._side_update / _side_blocked), per distinct scan."""
        scan = s.get('scan_count')
        if scan is not None and scan == self.side_scan:
            return self.side_streak >= self.side_confirm
        self.side_scan = scan
        xc, sd = _float(s.get('rotation_centre_x_m'), 0.), _float(s.get('rotation_centre_sd_m'), 0.)
        yaw = s.get('yaw_rate')
        sweep = abs(_float(yaw, 0.)) * max(0., _float(s.get('lidar_age_s'), 0.))
        comps = []
        for side in ('left', 'right'):
            x = s.get('side_nearest_x_%s_m' % side)
            clearance = s.get('side_clearance_%s_m' % side, math.inf)
            clearance = math.inf if clearance is None else clearance
            comps.append(math.inf if x is None else clearance - sweep * (abs(x - xc) + sd))
        self.side_streak = self.side_streak + 1 if min(comps) <= self.side_hard else 0
        return self.side_streak >= self.side_confirm

    def _stalled(self, now, s):
        speed = s.get('speed')
        stop = s.get('still') is True or (s.get('odom_valid') is True and _finite(speed) and speed < STALL_SPEED_M_S)
        if not stop:
            self.stall_since = None
            return False
        if self.stall_since is None:
            self.stall_since = now
        return now - self.stall_since >= STALL_S

    def _next_step(self, now, s):
        self.i += 1
        self.step_t0 = now
        self.step_d0 = self.counted
        self.step_h0 = _float(s.get('gyro_heading'), self.step_h0)
        self.stall_since = None
        self.measure = None
        self.phase = 'settle'           # R5: the next tick is the new step's settle (re-evaluated by _advance)

    def _close_step(self, interrupted=False, stalled=False):
        left, right = self.plan.steps[self.i]
        res = dict(i=self.i, L=int(left), R=int(right))
        m = self.measure.result() if self.measure is not None else _Measure().result()
        res.update(m)
        if self.plan.kind == 'arc':
            res['diff'] = int(right) - int(left)
            yaw = res['lidar_yaw_dps'] if res['lidar_yaw_dps'] is not None else res['gyro_yaw_dps']
            res['gain_dps_per_pct'] = _r(yaw / res['diff'], 4) if yaw is not None and res['diff'] else None
            res['valid'] = (not interrupted and not stalled and (m['seconds'] or 0.) >= MIN_MEASURE_S
                            and res['gyro_yaw_dps'] is not None)
        elif self.plan.kind == 'stop':
            res['duty'] = int(left)
            res['stalled'] = bool(stalled)
            res['steady'] = (not stalled and not interrupted and (m['seconds'] or 0.) >= MIN_MEASURE_S
                             and m['min_speed_m_s'] is not None and m['min_speed_m_s'] >= STALL_SPEED_M_S)
        else:
            res['duty'] = int(left)
            res['stalled'] = bool(stalled)
            res['sustained'] = (not stalled and not interrupted and (m['seconds'] or 0.) >= MIN_MEASURE_S * .5
                                and m['min_speed_m_s'] is not None and m['min_speed_m_s'] >= STALL_SPEED_M_S)
        res['interrupted'] = bool(interrupted)
        self.results.append(res)
        self.measure = None
        return res

    def _advance(self, now, s):
        p = self.plan
        elapsed = now - self.step_t0
        if p.kind in ('arc', 'stop') and self.go_t is None:
            # (base, base) from standstill until the vehicle rolls; then the arc's first pair / the stop's settle
            self.phase = 'start'
            if self.moved_t is None:
                return None
            speed = s.get('speed')
            if (sensor_reports_motion(s) and s.get('odom_valid') is True and _finite(speed)
                    and speed >= ARC_SWITCH_SPEED_M_S):
                self.go_t = self.step_t0 = now
                self.step_d0 = self.counted
                self.step_h0 = _float(s.get('gyro_heading'), self.step_h0)
                self.stall_since = None
                self.phase = 'settle'
                return None
            if self._stalled(now, s):
                return 'calib_start_stalled'
            return None
        if p.kind == 'stop':
            in_window = now - self.go_t >= STOP_SETTLE_S
            self.phase = 'measure' if in_window else 'settle'
            if in_window:
                if self.measure is None:
                    self.measure = _Measure()
                self.measure.add(now, s, self.counted)
            if self._stalled(now, s):
                self.measure = self.measure or _Measure()
                self._close_step(stalled=True)
                return 'calib_stop_stalled'
            if in_window and self.measure.seconds >= p.step_s:
                self._close_step()
                return 'calib_stop_test'
            return None
        if p.kind == 'arc':
            ready = now - max(self.go_t, self.step_t0) >= ARC_SETTLE_S
            if not ready:
                self.phase = 'settle'
                if self._stalled(now, s):
                    self.measure = self.measure or _Measure()
                    self._close_step(stalled=True)
                    return 'calib_arc_stalled'
                return None
            self.phase = 'measure'
            if self.measure is None:
                self.measure = _Measure()
            self.measure.add(now, s, self.counted)
            if self._stalled(now, s):
                self._close_step(stalled=True)
                return 'calib_arc_stalled'
            if self.measure.seconds >= p.step_s or self.counted - self.step_d0 >= p.step_m:
                self._close_step()
                if self.i + 1 >= len(p.steps):
                    return 'calib_complete'
                self._next_pending = True
            return None
        # rolldown
        if self.i == 0:
            if self.moved_t is None:
                self.phase = 'start'
                return None
            self.phase = 'rolling'
            if self.measure is None:
                self.measure = _Measure()
            self.measure.add(now, s, self.counted)
            if self._stalled(now, s):
                self._close_step(stalled=True)
                return 'rolldown_stalled_at_base'
            if now - self.moved_t >= ROLL_FIRST_HOLD_S:
                self._close_step()
                self._next_pending = True
            return None
        stalled = self._stalled(now, s)
        if elapsed >= ROLL_SETTLE_S:
            self.phase = 'measure'
            if self.measure is None:
                self.measure = _Measure()
            self.measure.add(now, s, self.counted)
        else:
            self.phase = 'settle'
        if stalled:
            self.measure = self.measure or _Measure()
            self._close_step(stalled=True)
            return 'rolldown_stalled'
        if elapsed >= p.step_s:
            self._close_step()
            if self.i + 1 >= len(p.steps):
                return 'rolldown_floor_reached'
            self._next_pending = True
        return None

    # --- reporting ----------------------------------------------------------------------------------------
    def _tick_record(self, now, s):
        scan = s.get('scan_count')
        if scan is not None and _finite(s.get('lidar_age_s')) and (not self._lidar_recent
                                                                   or self._lidar_recent[-1][3] != scan):
            self._lidar_recent.append((now - float(s['lidar_age_s']), _float(s.get('lidar_heading'), 0.),
                                       s.get('odom_valid') is True, scan))
        while self._lidar_recent and now - self._lidar_recent[0][0] > LIDAR_TICK_WINDOW_S + .15:
            self._lidar_recent.pop(0)
        lidar, ok = lidar_yaw([x[:3] for x in self._lidar_recent], min_span_s=.25, ok_fraction=1.)
        yaw = s.get('yaw_rate')
        left, right = self._command() if self.end_reason is None else (0, 0)
        self.last_tick = dict(k=self.plan.kind, i=self.i, n=len(self.plan.steps), ph=self.phase, sL=int(left),
                              sR=int(right), gyro=_r(math.degrees(yaw) if _finite(yaw) else None, 3),
                              lidar=_r(lidar, 3), lok=bool(ok >= 1. and lidar is not None),
                              odom=s.get('odom_valid') is True,
                              hd=_r(math.degrees(_float(s.get('gyro_heading'), self.run_h0 or 0.) - (self.run_h0 or 0.)), 2),
                              cd=_r(self.counted, 3), et=_r(now - self.t0, 2))

    def status(self):
        p = self.plan
        cur = None if self.end_reason is not None or not self.started else list(self._command())
        return dict(kind=p.kind, phase=self.phase, step_index=self.i, steps_total=len(p.steps), step=cur,
                    steps=[list(x) for x in p.steps], floor_pct=p.floor_pct, floor_source=p.floor_source,
                    start_floor_pct=p.start_floor_pct, start_floor_source=p.start_floor_source,
                    counted_m=_r(self.counted, 2), distance_cap_m=p.distance_cap_m, time_cap_s=p.time_cap_s,
                    elapsed_s=None if self.t0 is None or not self.last_tick else self.last_tick.get('et'),
                    results=[dict(r) for r in self.results], end_reason=self.end_reason, outcome=self.outcome,
                    params=dict(p.params), stop=None if self.stop is None else dict(self.stop))
