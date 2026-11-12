"""Row-following controller of the front-wheel-steering vehicle (P8; design v1 6.2-6.4, design v2 V2-4 / V2-5).

One row: align at standstill, follow the inter-row centre line, stop at the row end. No turn, no next row.

Input: ONLY the estimator snapshot the executor's Runner hands the controller slot and the controller's own clock t
(seconds since the resume). Read keys: lidar_age_s, imu_age_s, scan_count, speed, still / still_s, corridor_valid,
corridor_reason, corridor_age_s, heading_error, lateral_offset, row_end_ahead_m, front_clearance_m, odom_valid, the
estimator's actuator prior block (actuator_prior: only its 'predicting' flag), the map age hook (map_age_s, absent until
PC-07) and the steering fields of the front_steer Runner (steer_command_mrad, steer_measured_mrad). No plant, world or
truth value, no GNSS, no duty-derived distance (A13 role 'decision').

Output: SteerCommand(state, reason, throttle, steer_mrad) as the Runner expects (left / right repeat the throttle):
  'FOLLOW'          driving state. Phase ALIGN: throttle 0, the steering moves to the first target at standstill
                    (design 6.5 start alignment). Phase FOLLOW / APPROACH_END: throttle > 0.
  'STOP_REQUESTED'  throttle 0, steering held (steer_mrad None), pending ('hold', reason); then
  'HOLD'            reason, once the snapshot shows stillness for still_confirm_s. Stale input is a HOLD at once
                    (stale_lidar / stale_imu: the Runner's recoverable holds).
Hold reasons: row_end, obstacle_ahead, corridor_not_acquired, corridor_lost, speed_cap_below_min, overspeed, stale_lidar,
stale_imu. RECOVERABLE (SteerCommand.recoverable True): speed_cap_below_min, overspeed, stale_lidar, stale_imu - the
Runner pauses the run (a new go / press resumes it, design v2 V2-4 items 5-6); every other hold ends the run.

Steering (design 6.2 with the two gains of V2-5): lookahead L_d = clamp(base + per_speed x v, base, max); lookahead
point y_L = -k_d x lateral_offset + k_psi x L_d x sin(heading_error); curvature 2 y_L / L_d^2; steering angle
atan(wheelbase x curvature) + trim. The trim is a slow, bounded integral of y_L (a steering zero offset, a side pull).
The target is limited to steer_follow_limit_deg, low-pass filtered and slew limited to rate_margin x the executor
guard's slew limit, inside limit_margin x the equivalent soft limit. The yaw-rate inner loop gain k_r is a hook (0).

Speed (V2-4, PC-10): v_ref = min of the caps below; duty = table^-1(v_ref / speed_scale) + PI(v_ref - LiDAR speed),
slewed. The binding cap's name is the limit reason (telemetry 'cap'; every cap's value in 'caps').
  cruise     cruise_speed_m_s inside [min_speed_m_s, max_speed_m_s] (the field tuning 'target speed')
  scan       d_scan / T_scan       T_scan: p90 of the scan stamp spacing over timing_window_s
  blind      d_blind / tau_loop    tau_loop: p90 of the scan age at its first tick + loop_fixed_s + the p90 of the
                                   controller step spacing beyond one 20 Hz tick (late or missed ticks: computation)
  map        d_map / A_map         hook (off): the local grid age of PC-07, only with a grid fusion gain > 0
  stop       the largest speed whose stopping distance + front_margin_m fits the front clearance
  curvature  sqrt(lateral_accel_max / |curvature of the steering target|)
  lateral    slower with a large lateral offset
  steer      slower with a large steering demand: from steer_slow_start of steer_follow_limit_deg to the limit
  quality    min speed while the corridor is one-sided, its last update is older than corridor_weak_age_s or the
             estimator predicts on the actuator prior (snapshot actuator_prior 'predicting')
  fresh      min speed while the newest scan is older than fresh_slow_age_s (before the stale stop)
  end        approach_speed_m_s inside approach_window_m (+ the stopping distance) of the stop point
A reference below min_speed_m_s stops the run (speed_cap_below_min): no creeping inside the drive deadband.
Overspeed (V2-4 item 6): a LiDAR speed above max_speed_m_s + overspeed_margin_m_s sets the duty to the table duty of the
minimum speed; for longer than overspeed_s it stops the run (overspeed, a pause).
Row end (v1 6.4): the stop point is end_stop_offset_m before the estimator's row end (the front bumper at the last
trunk line); zero throttle when the remaining distance is within the stopping distance of the current speed.

Hooks that are OFF in this version (clearly separate): the local grid age cap (map_age_s None -> no map cap), a GNSS
geofence. Every number is a placeholder (자리값) until the steered
vehicle is measured; results with it are placeholder-based SIM results, not field performance. Standard library only.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, fields
import math

from .actuator_prior import DEFAULT_SPEED_TABLE, SpeedTable

DRIVING_STATES = ('FOLLOW',)
HOLD_REASONS = ('row_end', 'obstacle_ahead', 'corridor_not_acquired', 'corridor_lost', 'speed_cap_below_min',
                'overspeed', 'stale_lidar', 'stale_imu')
RECOVERABLE_HOLDS = ('speed_cap_below_min', 'overspeed', 'stale_lidar', 'stale_imu')
PHASES = ('ALIGN', 'FOLLOW', 'APPROACH_END', 'STOP', 'STOPPED')
CAPS = ('cruise', 'scan', 'blind', 'map', 'stop', 'curvature', 'lateral', 'steer', 'quality', 'fresh', 'end')
NOMINAL_TICK_S = .05                # the executor's 20 Hz tick
KIND = 'row_follow'


@dataclass(frozen=True)
class SteerCommand:
    state: str
    reason: str
    throttle: int
    steer_mrad: object        # int mrad, or None: keep the held steering
    recoverable: bool = False  # HOLD only: True = the Runner pauses the run (RECOVERABLE_HOLDS), False = the run ends

    @property
    def left(self):
        return self.throttle

    @property
    def right(self):
        return self.throttle


@dataclass(frozen=True)
class SteerFollowConfig:
    # --- from the executor configuration (make_factory fills them; never above the executor guard) -----------------
    rate_deg_s: float = 24.
    rate_standstill_deg_s: float = 12.
    limit_left_mrad: float = 409.
    limit_right_mrad: float = 409.
    lidar_max_age_s: float = .45
    imu_max_age_s: float = .40
    duty_cap_pct: int = 75
    # --- steering (placeholders) -----------------------------------------------------------------------------------
    rate_margin: float = .75
    limit_margin: float = .90
    wheelbase_m: float = .76
    lookahead_base_m: float = 1.5
    lookahead_per_speed_s: float = 1.25
    lookahead_max_m: float = 3.
    k_d: float = 1.
    k_psi: float = 1.
    k_r: float = 0.                  # yaw-rate inner loop (hook, off)
    trim_gain_rad_per_m_s: float = .05
    trim_limit_deg: float = 3.
    steer_follow_limit_deg: float = 12.
    steer_filter_s: float = .25
    align_tol_deg: float = 1.
    align_timeout_s: float = 4.
    # --- speed (placeholders) ----------------------------------------------------------------------------------------
    cruise_speed_m_s: float = .40
    min_speed_m_s: float = .20
    max_speed_m_s: float = .50
    approach_speed_m_s: float = .20
    approach_window_m: float = 1.5
    end_stop_offset_m: float = .61
    d_scan_m: float = .05
    d_blind_m: float = .15
    loop_fixed_s: float = .24        # tick wait 0.05 + step 0.06 + send wait 0.10 + link 0.03 (TIMING_BASELINE 8.1)
    timing_window_s: float = 2.
    front_margin_m: float = .35
    stop_onset_s: float = .25
    stop_decel_m_s2: float = 1.07
    lateral_slow_start_m: float = .15
    lateral_slow_full_m: float = .40
    corridor_weak_age_s: float = .6
    lateral_accel_max_m_s2: float = .3   # curvature cap (design v2 V2-4 v_curv)
    steer_slow_start: float = .8         # steering demand cap: from this fraction of steer_follow_limit_deg ...
    fresh_slow_age_s: float = .30        # newest scan older than this: min speed (stale stop at lidar_max_age_s)
    d_map_m: float = .20                 # map cap hook (only with a map age in the snapshot: PC-07, off)
    overspeed_margin_m_s: float = .05    # locked: LiDAR speed above max_speed + this ...
    overspeed_s: float = 1.              # ... for longer than this: stop (overspeed)
    speed_table: tuple = DEFAULT_SPEED_TABLE
    speed_scale: float = 1.
    speed_kp_pct_s_per_m: float = 30.
    speed_ki_pct_per_m: float = 15.
    speed_integral_limit_pct: float = 15.
    duty_rise_pct_s: float = 5.
    duty_fall_pct_s: float = 15.
    duty_max_pct: int = 70
    # --- holds ---------------------------------------------------------------------------------------------------------
    acquire_timeout_s: float = 6.
    corridor_lost_s: float = 1.
    still_confirm_s: float = .4

    def __post_init__(self):
        SpeedTable(self.speed_table)
        numbers = [getattr(self, f.name) for f in fields(self) if f.name != 'speed_table']
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in numbers):
            raise ValueError('steer_follow_invalid: every value must be a finite number')
        if not 0. < self.min_speed_m_s <= self.cruise_speed_m_s <= self.max_speed_m_s:
            raise ValueError('steer_follow_invalid: need 0 < min_speed <= cruise_speed <= max_speed')
        if not self.min_speed_m_s <= self.approach_speed_m_s <= self.max_speed_m_s:
            raise ValueError('steer_follow_invalid: approach_speed must be inside [min_speed, max_speed]')
        if not 0. < self.lookahead_base_m <= self.lookahead_max_m:
            raise ValueError('steer_follow_invalid: need 0 < lookahead_base <= lookahead_max')
        if not (0. < self.rate_margin <= 1. and 0. < self.limit_margin <= 1.):
            raise ValueError('steer_follow_invalid: rate_margin and limit_margin must be in (0, 1]')
        limit = math.degrees(self.limit_margin * min(self.limit_left_mrad, self.limit_right_mrad) / 1000.)
        if not 0. < self.steer_follow_limit_deg <= limit + 1e-9:
            raise ValueError('steer_follow_invalid: steer_follow_limit_deg must be inside the soft limit x limit_margin')
        if not (self.rate_deg_s > 0. and 0. < self.rate_standstill_deg_s <= self.rate_deg_s and self.wheelbase_m > 0.
                and self.trim_limit_deg >= 0. and self.trim_gain_rad_per_m_s >= 0. and self.steer_filter_s >= 0.
                and self.k_d >= 0. and self.k_psi >= 0. and self.k_r >= 0.):
            raise ValueError('steer_follow_invalid: steering rates, wheelbase, gains')
        if not (self.lateral_accel_max_m_s2 > 0. and 0. < self.steer_slow_start < 1. and self.d_map_m > 0.
                and 0. < self.fresh_slow_age_s < self.lidar_max_age_s and self.overspeed_margin_m_s >= 0.
                and self.overspeed_s > 0.):
            raise ValueError('steer_follow_invalid: curvature, steering demand, freshness or overspeed settings')
        if not (self.d_scan_m > 0. and self.d_blind_m > 0. and self.loop_fixed_s >= 0. and self.timing_window_s > 0.
                and self.stop_decel_m_s2 > 0. and self.stop_onset_s >= 0. and self.front_margin_m >= 0.
                and 0. <= self.lateral_slow_start_m < self.lateral_slow_full_m and self.speed_scale > 0.
                and self.duty_rise_pct_s > 0. and self.duty_fall_pct_s > 0.
                and 0 < self.duty_max_pct <= self.duty_cap_pct <= 100):
            raise ValueError('steer_follow_invalid: speed caps, stop model or duty limits')


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _p90(samples):
    values = sorted(v for _, v in samples)
    return values[min(len(values) - 1, int(math.ceil(.9 * len(values))) - 1)]


class RowFollowController:
    """One instance per resume (the Runner builds a new controller at every press). step(t, snapshot) -> SteerCommand."""

    def __init__(self, config=None):
        self.config = cfg = config or SteerFollowConfig()
        self.table = SpeedTable(cfg.speed_table)
        self.state, self.reason = 'FOLLOW', 'start'
        self.pending = None
        self.counts = dict(stops=0, holds=0)
        self.phase_events = []
        self._phase = None
        self._telemetry = {}
        self.steer = None                # shaped steering command [mrad, float]
        self.filtered = None             # low-pass state of the steering target [mrad]
        self.trim = 0.                   # [rad]
        self.last_t = None
        self.last_throttle = 0
        self.duty = 0.                   # slewed duty [%]
        self.integral = 0.               # speed PI integral [%]
        self.drive_t0 = None
        self.align_t0 = None
        self.invalid_since = None
        self.scan_seen = None            # (scan_count, scan stamp on the controller clock)
        self.scan_spacing = deque()      # (t, T_scan sample)
        self.scan_age = deque()          # (t, scan age at its first tick)
        self.step_spacing = deque()      # (t, controller step spacing)
        self.caps = {}
        self.bound = 'cruise'
        self.over_since = None           # overspeed start (controller clock)
        self.demand = 0.                 # |raw steering target| / steer_follow_limit of the last step
        self.curvature = 0.              # curvature of the last steering target [1/m]

    # --- the attributes the Runner and the SIM facades read ----------------------------------------------------------
    def current_phase(self):
        return self._phase or 'ALIGN'

    def telemetry(self):
        return dict(self._telemetry)

    def _set_phase(self, t, name):
        if name != self._phase:
            self._phase = name
            self.phase_events.append([round(float(t), 3), name])

    def _command(self, state, reason, throttle, steer):
        self.state, self.reason = state, reason
        self.last_throttle = int(throttle)
        return SteerCommand(state, reason, int(throttle), steer, state == 'HOLD' and reason in RECOVERABLE_HOLDS)

    # --- stops ---------------------------------------------------------------------------------------------------------
    def _stop(self, t, s, reason):
        if self.pending is None:
            self.pending = ('hold', reason)
            self.counts['stops'] += 1
            self._set_phase(t, 'STOP')
            self.duty = self.integral = 0.
        still_s = s.get('still_s')
        if s.get('still') is True and _finite(still_s) and still_s >= self.config.still_confirm_s:
            self.counts['holds'] += 1
            self._set_phase(t, 'STOPPED')
            self._telemetry = dict(self._telemetry, phase='STOPPED', stop_reason=reason)
            return self._command('HOLD', reason, 0, None)
        self._telemetry = dict(self._telemetry, phase='STOP', stop_reason=reason)
        return self._command('STOP_REQUESTED', 'stopping_' + reason, 0, None)

    def _stale(self, s):
        cfg = self.config
        lidar, imu = s.get('lidar_age_s'), s.get('imu_age_s')
        if not _finite(lidar) or lidar > cfg.lidar_max_age_s:
            return 'stale_lidar'
        if not _finite(imu) or imu > cfg.imu_max_age_s:
            return 'stale_imu'
        return None

    # --- timing of the loop (V2-4 item 3) ------------------------------------------------------------------------------
    def _timing(self, t, s, dt=0.):
        cfg = self.config
        if dt > 0.:
            self.step_spacing.append((t, dt))
        count, age = s.get('scan_count'), s.get('lidar_age_s')
        if isinstance(count, int) and _finite(age):
            stamp = t - age
            if self.scan_seen is None:
                self.scan_seen = (count, stamp)
            elif count > self.scan_seen[0]:
                spacing = (stamp - self.scan_seen[1]) / (count - self.scan_seen[0])
                if spacing > 0.:
                    self.scan_spacing.append((t, spacing))
                self.scan_age.append((t, max(0., age)))
                self.scan_seen = (count, stamp)
        for samples in (self.scan_spacing, self.scan_age, self.step_spacing):
            while samples and t - samples[0][0] > cfg.timing_window_s:
                samples.popleft()
        t_scan = _p90(self.scan_spacing) if self.scan_spacing else .10
        late = max(0., round(_p90(self.step_spacing) - NOMINAL_TICK_S, 6)) if self.step_spacing else 0.
        tau = (_p90(self.scan_age) if self.scan_age else .10) + cfg.loop_fixed_s + late
        return t_scan, tau

    def stopping_distance(self, speed, tau=0.):
        cfg = self.config
        v = max(0., speed)
        return v * (tau + cfg.stop_onset_s) + v * v / (2. * cfg.stop_decel_m_s2)

    def _speed_reference(self, s, t_scan, tau, remaining):
        """(v_ref, name of the binding cap, every cap)."""
        cfg = self.config
        caps = dict(cruise=min(cfg.max_speed_m_s, max(cfg.min_speed_m_s, cfg.cruise_speed_m_s)),
                    scan=cfg.d_scan_m / t_scan, blind=cfg.d_blind_m / tau)
        front = s.get('front_clearance_m')
        if _finite(front):
            room = front - cfg.front_margin_m
            if room <= 0.:
                caps['stop'] = 0.
            else:                        # v (tau + onset) + v^2 / 2a = room
                lag = tau + cfg.stop_onset_s
                a = cfg.stop_decel_m_s2
                caps['stop'] = -a * lag + math.sqrt(a * a * lag * lag + 2. * a * room)
        lateral = s.get('lateral_offset')
        if s.get('corridor_valid') is True and _finite(lateral):
            excess = (abs(lateral) - cfg.lateral_slow_start_m) / (cfg.lateral_slow_full_m - cfg.lateral_slow_start_m)
            caps['lateral'] = caps['cruise'] + (cfg.min_speed_m_s - caps['cruise']) * max(0., min(1., excess))
        age = s.get('corridor_age_s')
        prior = s.get('actuator_prior')
        predicting = isinstance(prior, dict) and prior.get('predicting') is True
        weak = (s.get('corridor_valid') is not True or 'one_side' in str(s.get('corridor_reason'))
                or (_finite(age) and age > cfg.corridor_weak_age_s) or predicting)
        if weak:
            caps['quality'] = cfg.min_speed_m_s
        map_age = s.get('map_age_s')             # hook: the local grid of PC-07 (absent: no map cap)
        if _finite(map_age) and map_age > 0.:
            caps['map'] = cfg.d_map_m / map_age
        if self.curvature > 1e-9:
            caps['curvature'] = math.sqrt(cfg.lateral_accel_max_m_s2 / self.curvature)
        if self.demand > cfg.steer_slow_start:
            excess = (self.demand - cfg.steer_slow_start) / (1. - cfg.steer_slow_start)
            caps['steer'] = caps['cruise'] + (cfg.min_speed_m_s - caps['cruise']) * max(0., min(1., excess))
        lidar_age = s.get('lidar_age_s')
        if _finite(lidar_age) and lidar_age > cfg.fresh_slow_age_s:
            caps['fresh'] = cfg.min_speed_m_s
        if remaining is not None and remaining <= cfg.approach_window_m + self.stopping_distance(caps['cruise'], tau):
            caps['end'] = cfg.approach_speed_m_s
        bound = min(caps, key=lambda name: (caps[name], CAPS.index(name)))
        return caps[bound], bound, caps

    # --- steering --------------------------------------------------------------------------------------------------------
    def _steer_target(self, s, speed, dt, driving):
        """Steering target [mrad] from the corridor (None without a usable corridor)."""
        cfg = self.config
        lateral, heading = s.get('lateral_offset'), s.get('heading_error')
        if s.get('corridor_valid') is not True or not _finite(lateral) or not _finite(heading):
            return None, {}
        v = max(cfg.min_speed_m_s, speed if _finite(speed) else 0.)
        lookahead = max(cfg.lookahead_base_m, min(cfg.lookahead_max_m,
                                                 cfg.lookahead_base_m + cfg.lookahead_per_speed_s * v))
        y_l = -cfg.k_d * lateral + cfg.k_psi * lookahead * math.sin(heading)
        curvature = 2. * y_l / (lookahead * lookahead)
        limit = math.radians(cfg.steer_follow_limit_deg)
        raw = math.atan(cfg.wheelbase_m * curvature)
        yaw_rate = s.get('yaw_rate')
        if cfg.k_r > 0. and _finite(yaw_rate):          # hook (off by default): yaw-rate inner loop
            raw += cfg.k_r * (v * curvature - yaw_rate) * cfg.wheelbase_m / v
        if driving and abs(raw + self.trim) < limit:    # slow bounded trim; frozen while the target is at its limit
            bound = math.radians(cfg.trim_limit_deg)
            self.trim = max(-bound, min(bound, self.trim + cfg.trim_gain_rad_per_m_s * y_l * dt))
        self.demand = abs(raw + self.trim) / limit       # steering demand (> 1: beyond the follow limit)
        self.curvature = abs(math.tan(min(limit, abs(raw + self.trim)))) / cfg.wheelbase_m
        target = max(-limit, min(limit, raw + self.trim)) * 1000.
        return target, dict(lookahead_m=round(lookahead, 3), lookahead_y_m=round(y_l, 4),
                            curvature_1_m=round(curvature, 5), trim_mrad=round(self.trim * 1000., 2),
                            lookahead_error_deg=round(math.degrees(math.atan2(y_l, lookahead)), 3))

    def _shape(self, t, target_mrad, throttle, dt):
        """Low-pass, then slew limit (moving = this or the previous throttle is not 0: the guard / MCU rule)."""
        cfg = self.config
        if self.filtered is None or cfg.steer_filter_s <= 0.:
            self.filtered = float(target_mrad)
        else:
            self.filtered += (float(target_mrad) - self.filtered) * (1. - math.exp(-dt / cfg.steer_filter_s))
        lo, hi = -cfg.limit_margin * cfg.limit_right_mrad, cfg.limit_margin * cfg.limit_left_mrad
        wanted = max(lo, min(hi, self.filtered))
        moving = throttle != 0 or self.last_throttle != 0
        rate = math.radians(cfg.rate_deg_s if moving else cfg.rate_standstill_deg_s) * 1000. * cfg.rate_margin
        step = rate * dt
        self.steer = wanted if self.steer is None else self.steer + max(-step, min(step, wanted - self.steer))
        return int(round(self.steer))

    # --- throttle --------------------------------------------------------------------------------------------------------
    def _throttle(self, v_ref, speed, dt, over=False):
        cfg = self.config
        if over:
            # overspeed (V2-4 item 6): the table duty of the minimum speed at once (no slew, the integral cleared)
            duty = self.table.duty(min(cfg.min_speed_m_s / cfg.speed_scale, self.table.points[-1][1])) or 1.
            self.duty, self.integral = max(1., min(float(cfg.duty_max_pct), duty)), 0.
            return int(round(self.duty)), duty
        feedforward = self.table.duty(min(v_ref / cfg.speed_scale, self.table.points[-1][1]))
        feedforward = cfg.duty_max_pct if feedforward is None else feedforward
        error = v_ref - (speed if _finite(speed) else 0.)
        wanted = feedforward + cfg.speed_kp_pct_s_per_m * error + self.integral
        if self.duty <= 0.:                              # start: the duty of the minimum speed, then the slew
            start = self.table.duty(min(cfg.min_speed_m_s / cfg.speed_scale, self.table.points[-1][1]))
            self.duty = min(float(cfg.duty_max_pct), start or 0.)
        else:
            step = wanted - self.duty
            limited = max(-cfg.duty_fall_pct_s * dt, min(cfg.duty_rise_pct_s * dt, step))
            if limited == step and 0. < self.duty + limited < cfg.duty_max_pct:      # no wind-up while limited
                bound = cfg.speed_integral_limit_pct
                self.integral = max(-bound, min(bound, self.integral + cfg.speed_ki_pct_per_m * error * dt))
            self.duty = max(1., min(float(cfg.duty_max_pct), self.duty + limited))
        return int(round(self.duty)), feedforward

    # --- step ----------------------------------------------------------------------------------------------------------
    def step(self, t, s):
        t = float(t)
        cfg = self.config
        dt = 0. if self.last_t is None else max(0., t - self.last_t)
        self.last_t = t
        if self.steer is None:
            held = s.get('steer_command_mrad')
            held = s.get('steer_measured_mrad') if held is None else held
            self.steer = float(held) if _finite(held) else 0.
            self.filtered = self.steer
        if self.pending is not None:
            return self._stop(t, s, self.pending[1])
        stale = self._stale(s)
        if stale is not None:
            self.counts['holds'] += 1
            self._telemetry = dict(self._telemetry, phase='STOPPED', stop_reason=stale)
            return self._command('HOLD', stale, 0, None)
        t_scan, tau = self._timing(t, s, dt if self.drive_t0 is not None else 0.)
        speed = s.get('speed')
        speed = float(speed) if _finite(speed) else 0.
        valid = s.get('corridor_valid') is True
        if self.drive_t0 is None:
            return self._align(t, s, dt, valid)
        if valid:
            self.invalid_since = None
        else:
            self.invalid_since = t if self.invalid_since is None else self.invalid_since
            if t - self.invalid_since > cfg.corridor_lost_s:
                return self._stop(t, s, 'corridor_lost')
        end = s.get('row_end_ahead_m') if valid else None
        remaining = end - cfg.end_stop_offset_m if _finite(end) else None
        if remaining is not None and remaining <= self.stopping_distance(speed):
            return self._stop(t, s, 'row_end')
        v_ref, bound, caps = self._speed_reference(s, t_scan, tau, remaining)
        self.caps, self.bound = caps, bound
        if bound == 'stop' and v_ref < cfg.min_speed_m_s:
            return self._stop(t, s, 'obstacle_ahead')
        if v_ref < cfg.min_speed_m_s - 1e-9:
            return self._stop(t, s, 'speed_cap_below_min')
        over = s.get('odom_valid', True) is not False and speed > cfg.max_speed_m_s + cfg.overspeed_margin_m_s
        if not over:
            self.over_since = None
        else:
            self.over_since = t if self.over_since is None else self.over_since
            if t - self.over_since > cfg.overspeed_s:
                return self._stop(t, s, 'overspeed')
        self._set_phase(t, 'APPROACH_END' if bound == 'end' else 'FOLLOW')
        throttle, feedforward = self._throttle(v_ref, speed, dt, over)
        target, detail = self._steer_target(s, speed, dt, True)
        steer = self._shape(t, self.steer if target is None else target, throttle, dt)
        self._telemetry = dict(detail, phase=self._phase, target_speed_m_s=round(v_ref, 3), cap=bound,
                               caps={k: round(v, 3) for k, v in caps.items()}, tau_loop_s=round(tau, 3),
                               t_scan_s=round(t_scan, 3), steer_demand=round(self.demand, 3),
                               overspeed=self.over_since is not None, base_duty_pct=round(self.duty, 2),
                               duty_feedforward_pct=round(feedforward, 2), steer_mrad=steer,
                               end_remaining_m=None if remaining is None else round(remaining, 3))
        return self._command('FOLLOW', 'follow_' + bound, throttle, steer)

    def _align(self, t, s, dt, valid):
        """Standstill: wait for the corridor, move the steering to the first target, then start driving."""
        cfg = self.config
        self._set_phase(t, 'ALIGN')
        if not valid:
            if t > cfg.acquire_timeout_s:
                return self._stop(t, s, 'corridor_not_acquired')
            self._telemetry = dict(phase='ALIGN', waiting='corridor')
            return self._command('FOLLOW', 'align_wait_corridor', 0, int(round(self.steer)))
        self.align_t0 = t if self.align_t0 is None else self.align_t0
        target, detail = self._steer_target(s, cfg.min_speed_m_s, dt, False)
        if target is None:
            self._telemetry = dict(phase='ALIGN', waiting='corridor')
            return self._command('FOLLOW', 'align_wait_corridor', 0, int(round(self.steer)))
        steer = self._shape(t, target, 0, dt)
        measured = s.get('steer_measured_mrad')
        tol = math.radians(cfg.align_tol_deg) * 1000.
        aligned = abs(self.steer - target) <= tol and (not _finite(measured) or abs(measured - self.steer) <= 2. * tol)
        if aligned or t - self.align_t0 >= cfg.align_timeout_s:
            self.drive_t0 = t
        self._telemetry = dict(detail, phase='ALIGN', steer_mrad=steer)
        return self._command('FOLLOW', 'align', 0, steer)


def controller_config(spec, executor_cfg):
    """SteerFollowConfig from a JSON-able spec {'kind': 'row_follow', <SteerFollowConfig fields>...} and the executor
    configuration (slew and soft limits, freshness limits, duty cap: never above the executor's own)."""
    known = {f.name for f in fields(SteerFollowConfig)}
    values = {k: v for k, v in dict(spec).items() if k != 'kind'}
    unknown = sorted(set(values) - known)
    if unknown:
        raise ValueError('unknown row_follow settings: %s' % ', '.join(unknown))
    if 'speed_table' in values:
        values['speed_table'] = tuple((float(d), float(v)) for d, v in values['speed_table'])
    values.update(rate_deg_s=float(executor_cfg.steer_rate_limit_deg_s),
                  rate_standstill_deg_s=float(executor_cfg.steer_rate_limit_standstill_deg_s),
                  limit_left_mrad=float(executor_cfg.steer_limit_left_mrad),
                  limit_right_mrad=float(executor_cfg.steer_limit_right_mrad),
                  lidar_max_age_s=float(executor_cfg.lidar_max_age_s),
                  imu_max_age_s=float(executor_cfg.imu_max_age_s), duty_cap_pct=int(executor_cfg.duty_cap_pct))
    return SteerFollowConfig(**values)


def make_factory(spec, executor_cfg):
    """A zero-argument controller factory (the Runner's controller_factory) for the spec; raises on a bad spec."""
    config = controller_config(spec, executor_cfg)
    return lambda: RowFollowController(config)
