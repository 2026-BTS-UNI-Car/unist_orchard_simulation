"""Scripted controllers for the front-steer executor's controller slot (design 8.3 P4 'scripted controllers').

These are SIM development drivers, NOT the row-following controller (that is P8: sim/ubts/closed_loop/steer_control.py).
Every controller reads ONLY what the executor's Runner hands it: its own clock t (s since the resume) and the estimator
snapshot (2D LiDAR + raw IMU: heading_error / lateral_offset of a valid corridor, gyro_heading, speed, still, distance_m,
front_clearance_m, row_end_ahead_m) with the steering fields the front_steer Runner adds (steer_measured_mrad from the
bridge's steering status, steer_command_mrad = the held command). No plant, world or truth value (A13 role 'decision').

Command: SteerCommand(state, reason, throttle, steer_mrad); left / right repeat the throttle (equal duty both sides).
  'FOLLOW'          driving state: throttle >= 0 and the steering may change, also at standstill (alignment before
                    the start, design 6.5, and the standstill steps of a maneuver script)
  'STOP_REQUESTED'  throttle 0, steering held (steer_mrad None), pending ('hold', reason): terminal; the controller goes
                    to 'HOLD' (reason) once the snapshot shows stillness for still_confirm_s (the Runner ends the run then)
  'HOLD'            terminal: the run ends with reason
Steering commands go through a slew shaper (SteerShaper): margin x the executor's moving / standstill slew limit, with the
same moving rule as the guard and the MCU (a drive command other than 0 = moving). A 'raw' segment skips the shaper (guard
tests). Steering targets are clamped to margin_limit x the equivalent soft limit unless the segment is raw.

Controllers
  StraightHold     heading / lateral hold on a straight line: corridor-relative (valid corridor: delta = k_psi x
                   heading_error - k_d x lateral_offset) or, without a corridor (W0), the gyro heading at the drive start
                   and the dead-reckoned lateral offset from it (speed x sin(heading error) integrated); align at
                   standstill first; stops at a distance, at the estimator's row end (row_end_ahead_m - end_offset), or
                   when the front clearance falls below min_front_m
  SegmentScript    a list of segments (duration, throttle, steering target or function of the segment time, shaped or
                   raw), then a stop (STOP_REQUESTED) or an end hold; maneuvers: steps(), circle(), zigzag(),
                   stop_test(), standstill_sweeps()
  HoldStill        throttle 0 and the held steering for a duration, then HOLD (sensor statistics at standstill)
make_factory(spec, cfg) builds a zero-argument factory (the Runner's controller_factory) from a JSON-able spec
{'kind': ..., ...}; spec_digest(spec) is its canonical SHA-256. The kind 'row_follow' is not a script: it hands the
slot to the P8 row-following controller (closed_loop/steer_control.py make_factory) with the same spec.

Every number is a placeholder of the SIM scripts. Results are placeholder-based SIM results (자리값 기반 SIM 결과).
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math

KINDS = ('straight_hold', 'segments', 'hold_still', 'row_follow')
ROW_FOLLOW = 'row_follow'     # the P8 row-following controller (closed_loop/steer_control.py), not a script
STILL_CONFIRM_S = .4          # vendored ControlConfig still_confirm_s (the Runner's stillness rule)
DEFAULT_RATE_MARGIN = .75     # shaped slew = this x the executor guard's slew limit
DEFAULT_LIMIT_MARGIN = .90    # shaped targets stay inside this x the equivalent soft limit


@dataclass(frozen=True)
class SteerCommand:
    state: str
    reason: str
    throttle: int
    steer_mrad: object        # int mrad, or None: keep the held steering

    @property
    def left(self):
        return self.throttle

    @property
    def right(self):
        return self.throttle


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class SteerShaper:
    """Slew-limited steering command (int mrad). rate_mrad_s / rate_standstill_mrad_s: the executor guard's slew limits;
    margin: the fraction used. moving = this or the previous throttle is not 0 (the guard / MCU rule)."""

    def __init__(self, rate_mrad_s, rate_standstill_mrad_s, limit_left_mrad, limit_right_mrad,
                 margin=DEFAULT_RATE_MARGIN, limit_margin=DEFAULT_LIMIT_MARGIN):
        self.rate = float(rate_mrad_s) * margin
        self.rate_standstill = float(rate_standstill_mrad_s) * margin
        self.limit_left = float(limit_left_mrad)
        self.limit_right = float(limit_right_mrad)
        self.limit_margin = float(limit_margin)
        self.value = None
        self.last_t = None
        self.last_throttle = 0

    def start(self, t, value_mrad):
        self.value = None if value_mrad is None else float(value_mrad)
        self.last_t = float(t)

    def clamp(self, target_mrad, margin=None):
        margin = self.limit_margin if margin is None else margin
        return max(-margin * self.limit_right, min(margin * self.limit_left, float(target_mrad)))

    def step(self, t, target_mrad, throttle, raw=False):
        t = float(t)
        if raw:
            out = float(target_mrad)
        else:
            target = self.clamp(target_mrad)
            if self.value is None:
                out = target
            else:
                dt = 0. if self.last_t is None else max(0., t - self.last_t)
                moving = throttle != 0 or self.last_throttle != 0
                limit = (self.rate if moving else self.rate_standstill) * dt
                out = self.value + max(-limit, min(limit, target - self.value))
        self.value, self.last_t, self.last_throttle = out, t, int(throttle)
        return int(round(out))


class _Base:
    """Common state of the scripted controllers (the attributes the Runner and the SIM facades read)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.state, self.reason = 'FOLLOW', 'start'
        self.pending = None
        self.counts = dict(stops=0, holds=0)
        self.phase_events = []
        self._phase = None
        self._stop_since = None
        self.shaper = SteerShaper(math.radians(cfg['rate_deg_s']) * 1000., math.radians(cfg['rate_standstill_deg_s'])
                                  * 1000., cfg['limit_left_mrad'], cfg['limit_right_mrad'],
                                  cfg.get('rate_margin', DEFAULT_RATE_MARGIN),
                                  cfg.get('limit_margin', DEFAULT_LIMIT_MARGIN))
        self._started = False
        self._telemetry = {}

    def current_phase(self):
        return self._phase or 'START'

    def telemetry(self):
        return dict(self._telemetry)

    def _set_phase(self, t, name):
        if name != self._phase:
            self._phase = name
            self.phase_events.append([round(float(t), 3), name])

    def _first(self, t, s):
        if not self._started:
            self._started = True
            measured = s.get('steer_command_mrad')
            if measured is None:
                measured = s.get('steer_measured_mrad')
            self.shaper.start(t, measured if _finite(measured) else 0.)

    def _command(self, state, reason, throttle, steer):
        self.state, self.reason = state, reason
        return SteerCommand(state, reason, int(throttle), steer)

    def _stop(self, t, s, reason):
        """STOP_REQUESTED until the snapshot shows stillness for still_confirm_s, then HOLD(reason)."""
        if self.pending is None:
            self.pending = ('hold', reason)
            self.counts['stops'] += 1
            self._set_phase(t, 'STOP')
        still_s = s.get('still_s')
        if s.get('still') is True and _finite(still_s) and still_s >= self.cfg.get('still_confirm_s', STILL_CONFIRM_S):
            self.counts['holds'] += 1
            self._set_phase(t, 'STOPPED')
            return self._command('HOLD', reason, 0, None)
        return self._command('STOP_REQUESTED', 'stopping_' + reason, 0, None)


class StraightHold(_Base):
    """Heading / lateral hold on a straight line (module docstring). Spec keys (all optional except the limits that
    make_factory fills from the executor config): drive_pct (40), distance_m (8.0, None = no distance stop),
    k_psi (0.9 rad/rad), k_d (0.4 rad/m), max_steer_deg (8.0), align_tol_deg (1.0), align_timeout_s (6.0),
    end_offset_m (0.61: the base stops this far before the estimator's row end), row_end (True), min_front_m (0.6)."""

    def __init__(self, cfg):
        super().__init__(cfg)
        self.drive_pct = int(cfg.get('drive_pct', 40))
        self.distance_m = cfg.get('distance_m', 8.)
        self.k_psi = float(cfg.get('k_psi', .9))
        self.k_d = float(cfg.get('k_d', .4))
        self.max_steer = math.radians(float(cfg.get('max_steer_deg', 8.))) * 1000.
        self.align_tol = math.radians(float(cfg.get('align_tol_deg', 1.))) * 1000.
        self.align_timeout_s = float(cfg.get('align_timeout_s', 6.))
        self.end_offset_m = float(cfg.get('end_offset_m', .61))
        self.use_row_end = bool(cfg.get('row_end', True))
        self.min_front_m = float(cfg.get('min_front_m', .6))
        self.mode = None            # 'corridor' or 'dead_reckoning' (fixed at the drive start)
        self.psi_ref = None
        self.lateral = 0.
        self.last_t = None
        self.d0 = None
        self.drive_t0 = None

    def step(self, t, s):
        t = float(t)
        self._first(t, s)
        if self.pending is not None:
            return self._stop(t, s, self.pending[1])
        if self.drive_t0 is None:
            self._set_phase(t, 'ALIGN')
            steer = self.shaper.step(t, 0., 0)
            measured = s.get('steer_measured_mrad')
            aligned = abs(self.shaper.value) <= .5 and (not _finite(measured) or abs(measured) <= self.align_tol)
            if aligned or t >= self.align_timeout_s:
                self.drive_t0 = t
                self.d0 = s.get('distance_m') or 0.
                corridor = s.get('corridor_valid') is True
                self.mode = 'corridor' if corridor else 'dead_reckoning'
                self.psi_ref = s.get('gyro_heading') if _finite(s.get('gyro_heading')) else 0.
                self.last_t = t
            self._telemetry = dict(phase='ALIGN', steer_mrad=steer)
            return self._command('FOLLOW', 'align', 0, steer)
        self._set_phase(t, 'DRIVE')
        travelled = (s.get('distance_m') or 0.) - self.d0
        if self.distance_m is not None and travelled >= float(self.distance_m):
            return self._stop(t, s, 'script_distance')
        end = s.get('row_end_ahead_m') if s.get('corridor_valid') else None
        if self.use_row_end and _finite(end) and end - self.end_offset_m <= 0.:
            return self._stop(t, s, 'row_end')
        front = s.get('front_clearance_m')
        if _finite(front) and front < self.min_front_m:
            return self._stop(t, s, 'obstacle_ahead')
        dt = 0. if self.last_t is None else max(0., t - self.last_t)
        self.last_t = t
        if self.mode == 'corridor' and s.get('corridor_valid') is True and _finite(s.get('heading_error')) \
                and _finite(s.get('lateral_offset')):
            psi_rel, d = -float(s['heading_error']), float(s['lateral_offset'])
            source = 'corridor'
        else:
            heading = s.get('gyro_heading')
            psi_rel = wrap(float(heading) - self.psi_ref) if _finite(heading) else 0.
            speed = s.get('speed')
            if _finite(speed):
                self.lateral += float(speed) * math.sin(psi_rel) * dt
            d = self.lateral
            source = 'dead_reckoning'
        delta = -self.k_d * d - self.k_psi * psi_rel
        target = max(-self.max_steer, min(self.max_steer, delta * 1000.))
        steer = self.shaper.step(t, target, self.drive_pct)
        self._telemetry = dict(phase='DRIVE', source=source, lateral_m=round(d, 4),
                               heading_rel_deg=round(math.degrees(psi_rel), 3), target_mrad=round(target, 2),
                               steer_mrad=steer, travelled_m=round(travelled, 3))
        return self._command('FOLLOW', 'hold_' + source, self.drive_pct, steer)


class SegmentScript(_Base):
    """Segments: dicts {duration_s, throttle (int %), steer_deg (number) or steer_fn ('sine', amplitude_deg, period_s,
    offset_deg) or 'hold' (keep), raw (bool, default False), label}. After the last segment: end 'stop' (STOP_REQUESTED
    -> HOLD script_complete) or 'hold_end' (HOLD at once, the steering held). The first segment starts at the resume."""

    def __init__(self, cfg):
        super().__init__(cfg)
        self.segments = [dict(seg) for seg in cfg['segments']]
        self.end = cfg.get('end', 'stop')
        self.bounds = []
        t0 = 0.
        for seg in self.segments:
            self.bounds.append((t0, t0 + float(seg['duration_s'])))
            t0 += float(seg['duration_s'])
        self.total_s = t0

    def _target(self, seg, t_seg):
        steer = seg.get('steer_deg')
        if steer == 'hold':
            return None
        if isinstance(steer, dict):
            amp, period = float(steer['amplitude_deg']), float(steer['period_s'])
            offset = float(steer.get('offset_deg', 0.))
            if steer.get('fn') == 'triangle':
                phase = (t_seg / period) % 1.
                value = offset + amp * (4. * phase - 1. if phase < .5 else 3. - 4. * phase)
            else:
                value = offset + amp * math.sin(2. * math.pi * t_seg / period)
            return math.radians(value) * 1000.
        return math.radians(float(steer)) * 1000.

    def step(self, t, s):
        t = float(t)
        self._first(t, s)
        if self.pending is not None:
            return self._stop(t, s, self.pending[1])
        for k, (a, b) in enumerate(self.bounds):
            if t < b:
                seg = self.segments[k]
                self._set_phase(t, seg.get('label', 'segment_%d' % k))
                throttle = int(seg.get('throttle', 0))
                target = self._target(seg, t - a)
                if target is None:
                    steer = self.shaper.step(t, self.shaper.value if self.shaper.value is not None else 0., throttle)
                else:
                    steer = self.shaper.step(t, target, throttle, raw=bool(seg.get('raw', False)))
                self._telemetry = dict(phase=seg.get('label', 'segment_%d' % k), segment=k, steer_mrad=steer,
                                       target_mrad=None if target is None else round(target, 2))
                return self._command('FOLLOW', 'script_%s' % seg.get('label', k), throttle, steer)
        if self.end == 'hold_end':
            self._set_phase(t, 'END')
            return self._command('HOLD', 'script_complete', 0, None)
        return self._stop(t, s, 'script_complete')


class HoldStill(_Base):
    """Throttle 0 and the held steering for duration_s, then HOLD('script_complete')."""

    def __init__(self, cfg):
        super().__init__(cfg)
        self.duration_s = float(cfg.get('duration_s', 10.))

    def step(self, t, s):
        self._first(t, s)
        if float(t) >= self.duration_s:
            return self._command('HOLD', 'script_complete', 0, None)
        self._set_phase(t, 'HOLD_STILL')
        return self._command('FOLLOW', 'hold_still', 0, int(round(self.shaper.value)))


# --- maneuver builders (segment lists) ---------------------------------------------------------------------------------
def steps(throttle=0, levels_deg=(5., -5., 10., -10., 0.), dwell_s=3., settle_s=1., raw=False):
    """Steering steps (standstill with throttle 0, or while driving); a settle segment at 0 first."""
    out = [dict(duration_s=settle_s, throttle=throttle, steer_deg=0., label='settle')]
    for k, level in enumerate(levels_deg):
        out.append(dict(duration_s=dwell_s, throttle=throttle, steer_deg=float(level), raw=raw, label='step_%d' % k))
    return out


def circle(throttle=40, steer_deg=10., duration_s=12., lead_s=2.):
    """Constant steering at a constant throttle (a circle); a straight lead first."""
    return [dict(duration_s=lead_s, throttle=throttle, steer_deg=0., label='lead'),
            dict(duration_s=duration_s, throttle=throttle, steer_deg=float(steer_deg), label='circle')]


def zigzag(throttle=40, amplitude_deg=6., period_s=4., cycles=3, lead_s=2., fn='sine'):
    return [dict(duration_s=lead_s, throttle=throttle, steer_deg=0., label='lead'),
            dict(duration_s=period_s * cycles, throttle=throttle,
                 steer_deg=dict(fn=fn, amplitude_deg=float(amplitude_deg), period_s=float(period_s)), label='zigzag')]


def stop_test(throttle=40, steer_deg=0., drive_s=6.):
    """Drive with a fixed steering, then the script's own stop (STOP_REQUESTED: zero throttle, steering held)."""
    return [dict(duration_s=drive_s, throttle=throttle, steer_deg=float(steer_deg), label='drive')]


def standstill_sweeps(throttle=0, amplitude_deg=22., period_s=2.5, sweeps=3):
    """Full-range steering sweeps (triangle) at a fixed drive duty (RF-03: duty 0 or inside the drive deadband)."""
    return [dict(duration_s=period_s * sweeps, throttle=throttle,
                 steer_deg=dict(fn='triangle', amplitude_deg=float(amplitude_deg), period_s=float(period_s)),
                 label='sweeps')]


# --- factory -------------------------------------------------------------------------------------------------------------
def controller_cfg(spec, executor_cfg):
    """Controller settings: the spec plus the executor's slew and soft limits (never above them)."""
    cfg = dict(spec)
    cfg.update(rate_deg_s=float(executor_cfg.steer_rate_limit_deg_s),
               rate_standstill_deg_s=float(executor_cfg.steer_rate_limit_standstill_deg_s),
               limit_left_mrad=int(executor_cfg.steer_limit_left_mrad),
               limit_right_mrad=int(executor_cfg.steer_limit_right_mrad))
    for key in ('rate_margin', 'limit_margin'):
        if spec.get(key) is not None and not 0. < float(spec[key]) <= 1.:
            raise ValueError('%s must be in (0, 1]' % key)
    return cfg


def make_factory(spec, executor_cfg):
    """A zero-argument controller factory (core.Runner controller_factory) from a JSON-able spec {'kind', ...}."""
    kind = spec.get('kind')
    if kind not in KINDS:
        raise ValueError('unknown scripted controller kind %r (known: %s)' % (kind, ', '.join(KINDS)))
    if kind == ROW_FOLLOW:
        # the row-following controller fills the same slot: {'kind': 'row_follow', <SteerFollowConfig fields>...}
        from closed_loop import steer_control
        return steer_control.make_factory(spec, executor_cfg)
    cfg = controller_cfg(spec, executor_cfg)
    cls = {'straight_hold': StraightHold, 'segments': SegmentScript, 'hold_still': HoldStill}[kind]
    if kind == 'segments' and not spec.get('segments'):
        raise ValueError('a segments script needs segments')
    return lambda: cls(cfg)


def spec_digest(spec):
    text = json.dumps(spec, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(text.encode('utf-8')).hexdigest()
