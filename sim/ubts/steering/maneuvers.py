"""Open-loop scripted maneuvers for the steering fast SIM (design 8.3 P2: step, ramp, fixed circle, sine sweep,
zigzag, stop test, stop by stop mode).

A maneuver is a SIM test input, not an executor. Every controller period (20 Hz, like closed_loop.harness) the
runner calls

    command(t_s, state) -> Command(throttle_pct, steer_target_mrad, stop_mode=None, steer_policy=None, phase=...)
    done(t_s, state)    -> True when the maneuver has finished (the runner then ends the episode)

with state = ManeuverState(t_s, v_m_s, yaw_change_rad, distance_m) taken from the plant TRUTH after the step.
Only two maneuvers read the state, and only to time their phases: zigzag with switch 'heading' (classic zig-zag:
the steering reverses when the heading change reaches +-heading_deg) and the stop maneuvers (the rest after a
stop starts when the speed is 0). Nothing here feeds an executor or an estimator.

Command fields (the first two are what closed_loop.harness / SteeringActuation read):
  throttle_pct       signed drive duty in percent (-100..100); a command, not a speed
  steer_target_mrad  steering target, wheel-side equivalent angle in mrad, + left (the plant clamps it to the
                     equivalent soft limit, A21)
  stop_mode          None for a drive command; 'release' / 'brake' / 'coast' asks for SteeredPlant.stop(mode)
                     (the runner calls it once on the change into that stop mode and leaves the plant alone
                     while the same stop command repeats; the steering target is then ignored)
  steer_policy       steering policy of the stop ('hold_measured', 'hold_target', 'center', 'off'; None = the
                     profile stop_policy_steer)
  phase              phase name for the trace

Parameters (normalise_params fills defaults and refuses unknown names). Times in s, angles in deg, duty in %:
  step        at_s, throttle_pct [before, after] or one value, steer_deg [before, after] or one value
  ramp        start_s, end_s, throttle_pct [from, to] or one value, steer_deg [from, to] or one value
  circle      throttle_pct or speed_m_s (exactly one), steer_deg
  sine_sweep  throttle_pct or speed_m_s, amplitude_deg, f0_hz, f1_hz, sweep_s, start_s, offset_deg
              (linear chirp: phase 2 pi (f0 tau + (f1 - f0) tau^2 / (2 sweep_s)), tau = t - start_s)
  zigzag      throttle_pct or speed_m_s, amplitude_deg, switch ('time' | 'heading'), half_period_s,
              heading_deg, start_s, switches (after that many reversals the steering goes to 0)
  stop_test   throttle_pct or speed_m_s, steer_deg, drive_s, hold_s, repeat
              key-release stop: drive for drive_s, then throttle 0 (MCU fade, EN on); after the vehicle
              rests hold_s the next cycle starts (repeat cycles), then done
  stop_mode   mode ('release' | 'brake' | 'coast'), steer_policy, throttle_pct or speed_m_s, steer_deg, drive_s,
              hold_s, repeat: drive, then SteeredPlant.stop(mode, steer_policy); rest hold_s; repeat; done
  schedule    knots [[t_s, throttle_pct, steer_deg], ...] (>= 2, times strictly increasing, the first >= 0),
              interpolation 'linear' | 'hold' (zero-order: the value of the last knot at or before t), end_s (done at
              t >= end_s; null = the last knot time). Before the first knot the first knot's values, after the last
              the last knot's values. A time script that both SIMs can follow exactly (A12; schedule_value() gives
              the same values to a G-SIM probe controller)
speed_m_s is turned into a duty once, at build time, with the throttle_for_speed callable of the runner (the
nominal steady speed of the profile's drive curve; the per-run speed scale of the plant still applies).

Pure Python (math). Placeholder-based results are placeholder-based SIM results, not field validation.
"""
from __future__ import annotations

import math
from typing import Callable, NamedTuple, Optional

STOP_MODES = ('release', 'brake', 'coast')
STEER_POLICIES = ('hold_measured', 'hold_target', 'center', 'off')
SWITCH_KINDS = ('time', 'heading')
MAX_STEER_DEG = 80.
MAX_TIME_S = 3600.


class ManeuverError(ValueError):
    """Invalid maneuver kind or parameters."""


class Command(NamedTuple):
    throttle_pct: float
    steer_target_mrad: float
    stop_mode: Optional[str] = None
    steer_policy: Optional[str] = None
    phase: str = 'run'


class ManeuverState(NamedTuple):
    """Plant truth after the step (SIM test timing only): time, speed, heading change since the start, distance."""

    t_s: float
    v_m_s: float
    yaw_change_rad: float
    distance_m: float


def _mrad(deg):
    return 1000. * math.radians(deg)


# --- parameter schema --------------------------------------------------------------------------------------------------
# (type, default). Types: 'time' >= 0, 'positive' > 0, 'number', 'steer' |deg| <= MAX_STEER_DEG, 'throttle'
# -100..100, 'pair_throttle' / 'pair_steer' (one value or [a, b], stored as [a, b]), 'optional_throttle',
# 'optional_speed' (> 0 or None), 'count' (int >= 1), 'choice:<name>' (one of the tuples below), 'optional_choice:..',
# 'knots' (schedule knots), 'optional_time' (>= 0 or None)
INTERPOLATIONS = ('linear', 'hold')
MAX_KNOTS = 512
_CHOICES = {'stop_mode': STOP_MODES, 'steer_policy': STEER_POLICIES, 'switch': SWITCH_KINDS,
            'interpolation': INTERPOLATIONS}
_DRIVE = {'throttle_pct': ('optional_throttle', None), 'speed_m_s': ('optional_speed', None)}
SCHEMAS = {
    'step': {'at_s': ('time', 1.), 'throttle_pct': ('pair_throttle', 0.), 'steer_deg': ('pair_steer', 0.)},
    'ramp': {'start_s': ('time', 1.), 'end_s': ('positive', 3.), 'throttle_pct': ('pair_throttle', 0.),
             'steer_deg': ('pair_steer', 0.)},
    'circle': dict(_DRIVE, steer_deg=('steer', 10.)),
    'sine_sweep': dict(_DRIVE, amplitude_deg=('steer', 5.), f0_hz=('positive', .05), f1_hz=('positive', 1.),
                       sweep_s=('positive', 20.), start_s=('time', 2.), offset_deg=('steer', 0.)),
    'zigzag': dict(_DRIVE, amplitude_deg=('steer', 10.), switch=('choice:switch', 'time'),
                   half_period_s=('positive', 2.), heading_deg=('positive', 10.), start_s=('time', 2.),
                   switches=('count', 6)),
    'stop_test': dict(_DRIVE, steer_deg=('steer', 0.), drive_s=('positive', 4.), hold_s=('time', 1.),
                      repeat=('count', 1)),
    'stop_mode': dict(_DRIVE, mode=('choice:stop_mode', 'brake'), steer_policy=('optional_choice:steer_policy', None),
                      steer_deg=('steer', 0.), drive_s=('positive', 4.), hold_s=('time', 1.), repeat=('count', 1)),
    'schedule': {'knots': ('knots', None), 'interpolation': ('choice:interpolation', 'linear'),
                 'end_s': ('optional_time', None)},
}
DRIVING = ('circle', 'sine_sweep', 'zigzag', 'stop_test', 'stop_mode')   # need exactly one of throttle / speed


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ManeuverError('maneuver_param_not_a_number: ' + name)
    value = float(value)
    if not math.isfinite(value):
        raise ManeuverError('maneuver_param_not_finite: ' + name)
    return value


def _check(kind, value, name):
    if kind.startswith('optional_') and value is None:
        return None
    if kind == 'knots':
        if not isinstance(value, (list, tuple)) or not 2 <= len(value) <= MAX_KNOTS:
            raise ManeuverError('maneuver_param_knots_need_2_to_%d_entries: %s' % (MAX_KNOTS, name))
        out = []
        for item in value:
            if not isinstance(item, (list, tuple)) or len(item) != 3:
                raise ManeuverError('maneuver_param_knot_needs_t_throttle_steer: ' + name)
            t, throttle, steer = (_number(v, name) for v in item)
            if not 0. <= t <= MAX_TIME_S or abs(throttle) > 100. or abs(steer) > MAX_STEER_DEG:
                raise ManeuverError('maneuver_param_out_of_range: ' + name)
            if out and not t > out[-1][0]:
                raise ManeuverError('maneuver_param_knot_times_not_increasing: ' + name)
            out.append([t, throttle, steer])
        return out
    if kind == 'optional_time':
        v = _number(value, name)
        if not 0. <= v <= MAX_TIME_S:
            raise ManeuverError('maneuver_param_out_of_range: ' + name)
        return v
    if kind in ('pair_throttle', 'pair_steer'):
        if isinstance(value, (list, tuple)):
            if len(value) != 2:
                raise ManeuverError('maneuver_param_pair_needs_two_values: ' + name)
            pair = [_number(v, name) for v in value]
        else:
            v = _number(value, name)
            pair = [v, v]
        limit = 100. if kind == 'pair_throttle' else MAX_STEER_DEG
        if any(abs(v) > limit for v in pair):
            raise ManeuverError('maneuver_param_out_of_range: ' + name)
        return pair
    if kind.startswith('choice:') or kind.startswith('optional_choice:'):
        choices = _CHOICES[kind.split(':', 1)[1]]
        if value not in choices:
            raise ManeuverError('maneuver_param_choice: %s must be one of %s' % (name, choices))
        return value
    if kind == 'count':
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 10000:
            raise ManeuverError('maneuver_param_count: ' + name)
        return value
    v = _number(value, name)
    if kind == 'time' and not 0. <= v <= MAX_TIME_S:
        raise ManeuverError('maneuver_param_out_of_range: ' + name)
    if kind in ('positive', 'optional_speed') and not v > 0.:
        raise ManeuverError('maneuver_param_not_positive: ' + name)
    if kind == 'steer' and abs(v) > MAX_STEER_DEG:
        raise ManeuverError('maneuver_param_out_of_range: ' + name)
    if kind in ('throttle', 'optional_throttle') and abs(v) > 100.:
        raise ManeuverError('maneuver_param_out_of_range: ' + name)
    return v


def normalise_params(kind, params):
    """Validated parameters with every default filled in (the canonical form of the scene spec)."""
    if kind not in SCHEMAS:
        raise ManeuverError('maneuver_kind_unknown: %r (known: %s)' % (kind, ', '.join(SCHEMAS)))
    params = {} if params is None else params
    if not isinstance(params, dict):
        raise ManeuverError('maneuver_params_not_a_mapping')
    schema = SCHEMAS[kind]
    unknown = sorted(str(k) for k in set(params) - set(schema))
    if unknown:
        raise ManeuverError('maneuver_param_unknown: %s (%s)' % (', '.join(unknown), kind))
    out = {name: _check(type_, params.get(name, default), name) for name, (type_, default) in schema.items()}
    if kind in DRIVING and (out['throttle_pct'] is None) == (out['speed_m_s'] is None):
        raise ManeuverError('maneuver_needs_exactly_one_of_throttle_pct_and_speed_m_s: ' + kind)
    if kind == 'ramp' and not out['end_s'] > out['start_s']:
        raise ManeuverError('maneuver_ramp_end_before_start')
    return out


# --- maneuvers ------------------------------------------------------------------------------------------------------------
class Maneuver:
    """Base class: kind, normalised params, resolved throttle; command() / done() / describe()."""

    KIND = ''
    PHASES = ()

    def __init__(self, params, throttle_for_speed: Optional[Callable[[float], float]] = None):
        self.params = normalise_params(self.KIND, params)
        self.throttle = None
        if 'speed_m_s' in self.params:
            if self.params['speed_m_s'] is not None:
                if throttle_for_speed is None:
                    raise ManeuverError('maneuver_speed_needs_throttle_for_speed: ' + self.KIND)
                self.throttle = _number(throttle_for_speed(self.params['speed_m_s']), 'resolved throttle')
                if not 0. < self.throttle <= 100.:
                    raise ManeuverError('maneuver_resolved_throttle_out_of_range: %r' % (self.throttle,))
            else:
                self.throttle = self.params['throttle_pct']

    def command(self, t_s, state):
        raise NotImplementedError

    def done(self, t_s, state):
        return False

    def describe(self):
        return dict(kind=self.KIND, params=dict(self.params), resolved_throttle_pct=self.throttle,
                    phases=list(self.PHASES), reads_truth_state=False)


class Step(Maneuver):
    KIND = 'step'
    PHASES = ('before', 'after')

    def command(self, t_s, state):
        i = 0 if t_s < self.params['at_s'] else 1
        return Command(self.params['throttle_pct'][i], _mrad(self.params['steer_deg'][i]), phase=self.PHASES[i])


class Ramp(Maneuver):
    KIND = 'ramp'
    PHASES = ('hold_start', 'ramp', 'hold_end')

    def command(self, t_s, state):
        p = self.params
        if t_s < p['start_s']:
            u, phase = 0., 'hold_start'
        elif t_s < p['end_s']:
            u, phase = (t_s - p['start_s']) / (p['end_s'] - p['start_s']), 'ramp'
        else:
            u, phase = 1., 'hold_end'
        (t0, t1), (s0, s1) = p['throttle_pct'], p['steer_deg']
        return Command(t0 + (t1 - t0) * u, _mrad(s0 + (s1 - s0) * u), phase=phase)


class Circle(Maneuver):
    KIND = 'circle'
    PHASES = ('circle',)

    def command(self, t_s, state):
        return Command(self.throttle, _mrad(self.params['steer_deg']), phase='circle')


class SineSweep(Maneuver):
    KIND = 'sine_sweep'
    PHASES = ('lead_in', 'sweep', 'hold')

    def command(self, t_s, state):
        p = self.params
        tau = t_s - p['start_s']
        if tau < 0.:
            return Command(self.throttle, _mrad(p['offset_deg']), phase='lead_in')
        if tau >= p['sweep_s']:
            return Command(self.throttle, _mrad(p['offset_deg']), phase='hold')
        f0, f1, span = p['f0_hz'], p['f1_hz'], p['sweep_s']
        angle = 2. * math.pi * (f0 * tau + (f1 - f0) * tau * tau / (2. * span))
        return Command(self.throttle, _mrad(p['offset_deg'] + p['amplitude_deg'] * math.sin(angle)), phase='sweep')


class Zigzag(Maneuver):
    KIND = 'zigzag'
    PHASES = ('lead_in', 'left', 'right', 'hold')

    def __init__(self, params, throttle_for_speed=None):
        super().__init__(params, throttle_for_speed)
        self._reference = None
        self._sign = 1
        self._switches = 0

    def command(self, t_s, state):
        p = self.params
        if t_s < p['start_s']:
            return Command(self.throttle, 0., phase='lead_in')
        if p['switch'] == 'time':
            k = int(math.floor((t_s - p['start_s']) / p['half_period_s']))
            self._switches = k
            self._sign = 1 if k % 2 == 0 else -1
        else:
            if self._reference is None:
                self._reference = state.yaw_change_rad
            heading = state.yaw_change_rad - self._reference
            limit = math.radians(p['heading_deg'])
            if self._sign > 0 and heading >= limit:
                self._sign, self._switches = -1, self._switches + 1
            elif self._sign < 0 and heading <= -limit:
                self._sign, self._switches = 1, self._switches + 1
        if self._switches >= p['switches']:
            return Command(self.throttle, 0., phase='hold')
        return Command(self.throttle, _mrad(self._sign * p['amplitude_deg']), phase='left' if self._sign > 0 else 'right')

    def describe(self):
        out = super().describe()
        out['reads_truth_state'] = self.params['switch'] == 'heading'
        return out


class _StopCycles(Maneuver):
    """drive (drive_s) -> stop -> rest (speed 0 for hold_s) -> ... repeat cycles -> done."""

    PHASES = ('drive', 'stop', 'rest')

    def __init__(self, params, throttle_for_speed=None):
        super().__init__(params, throttle_for_speed)
        self._cycle = 0
        self._phase = 'drive'
        self._since = 0.
        self._finished = False

    def _stop_command(self):
        raise NotImplementedError

    def command(self, t_s, state):
        p = self.params
        if self._phase == 'drive' and t_s - self._since >= p['drive_s'] - 1e-9:
            self._phase, self._since = 'stop', t_s
        if self._phase == 'stop' and state.v_m_s == 0.:
            self._phase, self._since = 'rest', t_s
        if self._phase == 'rest' and t_s - self._since >= p['hold_s'] - 1e-9:
            self._cycle += 1
            if self._cycle >= p['repeat']:
                self._finished = True
            else:
                self._phase, self._since = 'drive', t_s
        if self._phase == 'drive':
            return Command(self.throttle, _mrad(p['steer_deg']), phase='drive')
        stop = self._stop_command()
        return stop._replace(phase=self._phase)

    def done(self, t_s, state):
        return self._finished

    def describe(self):
        out = super().describe()
        out['reads_truth_state'] = True     # the rest starts when the true speed is 0
        return out


class StopTest(_StopCycles):
    KIND = 'stop_test'

    def _stop_command(self):
        return Command(0., _mrad(self.params['steer_deg']))


class StopMode(_StopCycles):
    KIND = 'stop_mode'

    def _stop_command(self):
        return Command(0., _mrad(self.params['steer_deg']), stop_mode=self.params['mode'],
                       steer_policy=self.params['steer_policy'])


def schedule_value(params, t_s):
    """(throttle_pct, steer_deg) of a normalised schedule at time t_s (linear or zero-order hold between knots)."""
    knots = params['knots']
    if t_s <= knots[0][0]:
        return knots[0][1], knots[0][2]
    if t_s >= knots[-1][0]:
        return knots[-1][1], knots[-1][2]
    for (t0, a0, s0), (t1, a1, s1) in zip(knots, knots[1:]):
        if t0 <= t_s < t1:
            if params['interpolation'] == 'hold':
                return a0, s0
            u = (t_s - t0) / (t1 - t0)
            return a0 + (a1 - a0) * u, s0 + (s1 - s0) * u
    return knots[-1][1], knots[-1][2]


class Schedule(Maneuver):
    KIND = 'schedule'
    PHASES = ('schedule', 'hold')

    def command(self, t_s, state):
        throttle, steer = schedule_value(self.params, t_s)
        end = self.params['end_s'] if self.params['end_s'] is not None else self.params['knots'][-1][0]
        return Command(throttle, _mrad(steer), phase='schedule' if t_s < end else 'hold')

    def done(self, t_s, state):
        end = self.params['end_s'] if self.params['end_s'] is not None else self.params['knots'][-1][0]
        return t_s >= end - 1e-9


MANEUVERS = {cls.KIND: cls for cls in (Step, Ramp, Circle, SineSweep, Zigzag, StopTest, StopMode, Schedule)}
assert set(MANEUVERS) == set(SCHEMAS)


def make_maneuver(kind, params, throttle_for_speed=None):
    """A new maneuver object (fresh state) of this kind."""
    if kind not in MANEUVERS:
        raise ManeuverError('maneuver_kind_unknown: %r' % (kind,))
    return MANEUVERS[kind](params, throttle_for_speed)


def duty_for_speed(profile, speed_m_s, curve='main'):
    """Duty % whose NOMINAL steady straight speed on the profile's drive curve is speed_m_s (bisection;
    drive.steady_speed_m_s, battery scale of the profile, no per-run speed scale). Raises above the 100 % speed."""
    from . import drive          # numpy-free, imported here so that this module stays pure Python
    speed = _number(speed_m_s, 'speed_m_s')
    if not 0. < speed <= drive.steady_speed_m_s(profile, 100., curve):
        raise ManeuverError('speed_outside_the_drive_curve: %r m/s' % (speed,))
    lo, hi = 0., 100.
    for _ in range(80):
        mid = .5 * (lo + hi)
        if drive.steady_speed_m_s(profile, mid, curve) < speed:
            lo = mid
        else:
            hi = mid
    return .5 * (lo + hi)
