"""Field tuning file of the front-steer row executor (design v2 V2-5, decision R35; PW-07, PW-08, PC-13).

One JSON file holds the values an operator may change in the field, read by the vehicle executor, the fast SIM
(tools/row_executor/sim_in_loop_steer.py) and the Gazebo executor container (sim/ubts/steering/exec_container_main.py):
all three build the executor loop, which loads the file named by the executor config key tuning_file (None = the
package default field_tuning.default.json next to this module). Standard library only (Python 3.10 - 3.13).

Groups (R35):
  basic     the screen may change them (POST /api/params) - only at rest: no open run, no lease, vehicle still
  advanced  the file only (the screen shows them read-only for now)
  locked    the file only, and only with a restart; the executor-config ones must equal the executor configuration
            (a mismatch refuses the file: the safety limits have one source, the executor config)

File (schema ubts.field_tuning.v1; strict JSON: duplicate keys, NaN / Infinity, unknown keys refused):
  {"schema": "ubts.field_tuning.v1", "preset": "<active preset name>", "note": "...",
   "values": {"basic": {...}, "advanced": {...}, "locked": {...}},       every group optional, keys optional:
                                                                         a missing key takes the default of KEYS
   "presets": {"<name>": {"note": "...", "surface": "...", "battery": "...",
                          "values": {"basic": {...}, "advanced": {...}}}}}   a preset = the defaults + its values
The built-in preset 'default' is KEYS' defaults (back to it: the digest equals the defaults' digest).

digest()     SHA-256 of the canonical JSON of every resolved value (the tuning identity in the run records);
file_sha256  SHA-256 of the file bytes (the file identity in the run records: executor log, fast SIM RESULT, G-SIM).
controller_fields()  the values of the steering controller (closed_loop/steer_control.SteerFollowConfig field names).
config_fields()      the executor config keys the tuning sets (prior_speed_table, prior_speed_scale).

Every default is a placeholder (자리값) until the steered vehicle is measured; nothing here was validated in the field.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
from dataclasses import dataclass

SCHEMA = 'ubts.field_tuning.v1'
GROUPS = ('basic', 'advanced', 'locked')
DEFAULT_PRESET = 'default'
PACKAGE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'field_tuning.default.json')
MAX_FILE_BYTES = 65536
MAX_TABLE_POINTS = 32
MAX_PRESETS = 32
NAME_CHARS = set('abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-.')
# the vendored placeholder table (closed_loop/actuator_prior.py DEFAULT_SPEED_TABLE; a test compares them)
DEFAULT_SPEED_TABLE = ((25., .239), (80., .709), (90., .715), (100., .899))


@dataclass(frozen=True)
class Key:
    name: str
    group: str
    kind: str                 # 'real', 'int', 'table'
    low: object
    high: object
    default: object
    unit: str
    target: tuple             # ('controller', field) / ('config', field) / ('scale',) / ('lock_controller', field)
    note: str


def _k(name, group, kind, low, high, default, unit, target, note):
    return Key(name, group, kind, low, high, default, unit, target, note)


KEYS = (
    # --- basic (R35: the screen may change them at rest) -------------------------------------------------------------
    _k('speed_scale', 'basic', 'real', .5, 1.3, 1., '', ('scale',),
       'k_field: table speed x k_field x surface x battery = expected speed'),
    _k('target_speed_m_s', 'basic', 'real', .2, .5, .4, 'm/s', ('controller', 'cruise_speed_m_s'), 'target (cruise) speed'),
    _k('lookahead_base_m', 'basic', 'real', .8, 3., 1.5, 'm', ('controller', 'lookahead_base_m'), 'lookahead floor'),
    _k('lookahead_per_speed_s', 'basic', 'real', 0., 3., 1.25, 's', ('controller', 'lookahead_per_speed_s'),
       'lookahead per speed'),
    _k('lookahead_max_m', 'basic', 'real', 1., 5., 3., 'm', ('controller', 'lookahead_max_m'), 'lookahead ceiling'),
    _k('k_d', 'basic', 'real', 0., 3., 1., '', ('controller', 'k_d'), 'lateral gain of the lookahead point'),
    _k('k_psi', 'basic', 'real', 0., 3., 1., '', ('controller', 'k_psi'), 'heading gain of the lookahead point'),
    _k('steer_rate_fraction', 'basic', 'real', .2, 1., .75, '', ('controller', 'rate_margin'),
       'steering command slew: this fraction of the executor guard slew limit (itself <= 0.8 x the measured rate)'),
    # --- advanced (the file) ---------------------------------------------------------------------------------------------
    _k('speed_table', 'advanced', 'table', None, None, DEFAULT_SPEED_TABLE, '[duty %, m/s]', ('table',),
       'duty -> steady speed table (rising duty, non-falling speed)'),
    _k('surface_scale', 'advanced', 'real', .7, 1.3, 1., '', ('scale',), 'ground preset factor of the speed scale'),
    _k('align_timeout_s', 'advanced', 'real', 2., 12., 4., 's', ('controller', 'align_timeout_s'),
       'standstill alignment: drive after this even when the steering has not reached its first target'),
    _k('battery_scale', 'advanced', 'real', .8, 1.1, 1., '', ('scale',), 'battery factor of the speed scale'),
    _k('min_speed_m_s', 'advanced', 'real', .15, .3, .2, 'm/s', ('controller', 'min_speed_m_s'),
       'minimum speed (a cap below it pauses)'),
    _k('approach_speed_m_s', 'advanced', 'real', .15, .4, .2, 'm/s', ('controller', 'approach_speed_m_s'),
       'row-end approach speed'),
    _k('approach_window_m', 'advanced', 'real', .5, 4., 1.5, 'm', ('controller', 'approach_window_m'),
       'row-end approach distance (+ the stopping distance)'),
    _k('end_stop_offset_m', 'advanced', 'real', 0., 2., .61, 'm', ('controller', 'end_stop_offset_m'),
       'stop point before the estimated row end (front bumper at the last trunk line)'),
    _k('duty_max_pct', 'advanced', 'int', 20, 100, 70, '%', ('controller', 'duty_max_pct'), 'drive duty ceiling'),
    _k('speed_kp_pct_s_per_m', 'advanced', 'real', 0., 100., 30., '%/(m/s)', ('controller', 'speed_kp_pct_s_per_m'),
       'speed PI proportional gain'),
    _k('speed_ki_pct_per_m', 'advanced', 'real', 0., 60., 15., '%/m', ('controller', 'speed_ki_pct_per_m'),
       'speed PI integral gain'),
    _k('speed_integral_limit_pct', 'advanced', 'real', 0., 30., 15., '%', ('controller', 'speed_integral_limit_pct'),
       'speed PI integral limit'),
    _k('duty_rise_pct_s', 'advanced', 'real', 1., 20., 5., '%/s', ('controller', 'duty_rise_pct_s'), 'duty rise rate'),
    _k('duty_fall_pct_s', 'advanced', 'real', 5., 50., 15., '%/s', ('controller', 'duty_fall_pct_s'), 'duty fall rate'),
    _k('speed_window_s', 'advanced', 'real', 0., 3., 0., 's', ('controller', 'speed_window_s'),
       'speed input max(LiDAR speed, path rate over this window); 0 = the LiDAR speed alone (2026-11-19 corridor)'),
    _k('speed_prior_floor', 'advanced', 'real', 0., 1., 0., '', ('controller', 'speed_prior_floor'),
       'speed input also >= this x the duty-table speed at the applied duty (0 = off; opt-in, design rule A13)'),
    _k('k_r', 'advanced', 'real', 0., 2., 0., '', ('controller', 'k_r'), 'yaw-rate inner loop gain (0 = off)'),
    _k('trim_gain_rad_per_m_s', 'advanced', 'real', 0., .2, .05, 'rad/(m s)', ('controller', 'trim_gain_rad_per_m_s'),
       'steering trim integral gain'),
    _k('trim_limit_deg', 'advanced', 'real', 0., 5., 3., 'deg', ('controller', 'trim_limit_deg'), 'steering trim limit'),
    _k('steer_follow_limit_deg', 'advanced', 'real', 3., 20., 12., 'deg', ('controller', 'steer_follow_limit_deg'),
       'steering limit while following (also inside 0.9 x the soft limit)'),
    _k('steer_filter_s', 'advanced', 'real', 0., 1., .25, 's', ('controller', 'steer_filter_s'),
       'steering target low-pass'),
    _k('d_scan_m', 'advanced', 'real', .02, .2, .05, 'm', ('controller', 'd_scan_m'), 'scan cap: travel per scan'),
    _k('d_blind_m', 'advanced', 'real', .05, .5, .15, 'm', ('controller', 'd_blind_m'), 'blind cap: travel per tau_loop'),
    _k('loop_fixed_s', 'advanced', 'real', 0., .6, .24, 's', ('controller', 'loop_fixed_s'),
       'fixed part of tau_loop (tick, step, send, link)'),
    _k('corridor_weak_age_s', 'advanced', 'real', .2, 2., .6, 's', ('controller', 'corridor_weak_age_s'),
       'quality cap: corridor update older than this'),
    _k('lateral_slow_start_m', 'advanced', 'real', 0., .5, .15, 'm', ('controller', 'lateral_slow_start_m'),
       'lateral cap: slow down from this offset ...'),
    _k('lateral_slow_full_m', 'advanced', 'real', .1, 1., .4, 'm', ('controller', 'lateral_slow_full_m'),
       '... to the minimum speed at this offset'),
    _k('lateral_accel_max_m_s2', 'advanced', 'real', .1, 1., .3, 'm/s^2', ('controller', 'lateral_accel_max_m_s2'),
       'curvature cap'),
    _k('steer_slow_start', 'advanced', 'real', .5, .95, .8, '', ('controller', 'steer_slow_start'),
       'steering demand cap starts at this fraction of the follow limit'),
    _k('fresh_slow_age_s', 'advanced', 'real', .15, .44, .3, 's', ('controller', 'fresh_slow_age_s'),
       'freshness cap: newest scan older than this'),
    # --- locked (the file and a restart; the executor-config ones must equal the executor config) ------------------
    _k('max_speed_m_s', 'locked', 'real', .2, .5, .5, 'm/s', ('lock_controller', 'max_speed_m_s'), 'maximum speed'),
    _k('front_margin_m', 'locked', 'real', .2, 1., .35, 'm', ('lock_controller', 'front_margin_m'),
       'front margin of the stop cap'),
    _k('overspeed_margin_m_s', 'locked', 'real', 0., .2, .05, 'm/s', ('lock_controller', 'overspeed_margin_m_s'),
       'overspeed: above max_speed + this ...'),
    _k('overspeed_s', 'locked', 'real', .2, 3., 1., 's', ('lock_controller', 'overspeed_s'), '... for longer: pause'),
    _k('duty_cap_pct', 'locked', 'int', 1, 100, 75, '%', ('config', 'duty_cap_pct'), 'executor output guard duty cap'),
    _k('lidar_max_age_s', 'locked', 'real', .1, 1., .45, 's', ('config', 'lidar_max_age_s'), 'scan freshness'),
    _k('imu_max_age_s', 'locked', 'real', .05, 1., .4, 's', ('config', 'imu_max_age_s'), 'IMU freshness'),
    _k('steer_status_max_age_s', 'locked', 'real', .05, 1., .3, 's', ('config', 'steer_status_max_age_s'),
       'steering status freshness'),
    _k('stall_guard_s', 'locked', 'real', .1, 2., .35, 's', ('config', 'stall_guard_s'), 'loop gap'),
    _k('operator_timeout_s', 'locked', 'real', .5, 3., 1., 's', ('config', 'operator_timeout_s'),
       'operator link timeout (one_press)'),
    _k('guard_heading_deg', 'locked', 'real', 5., 90., 25., 'deg', ('config', 'guard_heading_deg'),
       'geometry guard heading'),
    _k('prior_predict_max_s', 'locked', 'real', .1, 3., 1., 's', ('config', 'prior_predict_max_s'),
       'actuator prior prediction bound (time)'),
    _k('prior_predict_max_m', 'locked', 'real', .05, 2., .5, 'm', ('config', 'prior_predict_max_m'),
       'actuator prior prediction bound (distance)'),
)
BY_NAME = {k.name: k for k in KEYS}
BASIC = tuple(k.name for k in KEYS if k.group == 'basic')
SCALE_RANGE = (.5, 1.3)          # the product k_field x surface x battery (= executor prior_speed_scale range)


class TuningError(ValueError):
    """problems: the list of reasons (each names the key)."""

    def __init__(self, problems):
        self.problems = list(problems)
        super().__init__('; '.join(self.problems))


def defaults():
    """Every key's default value (a fresh dict)."""
    return {k.name: copy.deepcopy(k.default) for k in KEYS}


def _canonical(value):
    if isinstance(value, tuple):
        return [_canonical(v) for v in value]
    return value


def digest(values):
    text = json.dumps({k: _canonical(values[k]) for k in sorted(values)}, sort_keys=True, separators=(',', ':'),
                      allow_nan=False)
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def _real(value):
    return type(value) in (int, float) and math.isfinite(value)


def _table(value, problems, name='speed_table'):
    if not isinstance(value, (list, tuple)) or not 2 <= len(value) <= MAX_TABLE_POINTS:
        problems.append('%s: 2 .. %d points [duty %%, speed m/s]' % (name, MAX_TABLE_POINTS))
        return None
    points, previous = [], None
    for k, point in enumerate(value):
        if not isinstance(point, (list, tuple)) or len(point) != 2 or not all(_real(v) for v in point):
            problems.append('%s[%d]: must be [duty %%, speed m/s], finite numbers' % (name, k))
            return None
        duty, speed = float(point[0]), float(point[1])
        if not (0. < duty <= 100. and 0. <= speed <= 3.):
            problems.append('%s[%d]: duty outside (0, 100] or speed outside [0, 3]' % (name, k))
        if previous is not None and not (duty > previous[0] and speed >= previous[1]):
            problems.append('%s[%d]: the duty must rise and the speed must not fall with the duty' % (name, k))
        previous = (duty, speed)
        points.append(previous)
    if points and not points[-1][1] > 0.:
        problems.append('%s: the last speed must be > 0' % name)
    return tuple(points)


def check_value(name, value):
    """(value, problems) of one key: the type and range of KEYS (tables as tuples of (duty, speed))."""
    key = BY_NAME.get(name)
    if key is None:
        return None, ['%s: not a tuning key' % name]
    problems = []
    if key.kind == 'table':
        return _table(value, problems, name), problems
    if key.kind == 'int':
        if type(value) is not int:
            return None, ['%s: must be an integer' % name]
    elif not _real(value):
        return None, ['%s: must be a finite number' % name]
    if not key.low <= value <= key.high:
        return None, ['%s: %r outside [%g, %g] %s' % (name, value, key.low, key.high, key.unit)]
    return (int(value) if key.kind == 'int' else float(value)), []


def cross_problems(values):
    """The rules between values (R35 / design v2 V2-5 item 5)."""
    p = []
    v = values
    if not v['min_speed_m_s'] < v['target_speed_m_s'] <= v['max_speed_m_s']:
        p.append('target_speed_m_s: need min_speed_m_s < target_speed_m_s <= max_speed_m_s (locked)')
    if not v['min_speed_m_s'] <= v['approach_speed_m_s'] <= v['max_speed_m_s']:
        p.append('approach_speed_m_s: need min_speed_m_s <= approach_speed_m_s <= max_speed_m_s')
    if not v['lookahead_base_m'] <= v['lookahead_max_m']:
        p.append('lookahead_base_m: must not exceed lookahead_max_m')
    if not v['lateral_slow_start_m'] < v['lateral_slow_full_m']:
        p.append('lateral_slow_start_m: must be below lateral_slow_full_m')
    if not v['duty_max_pct'] <= v['duty_cap_pct']:
        p.append('duty_max_pct: must not exceed duty_cap_pct (locked)')
    if not v['fresh_slow_age_s'] < v['lidar_max_age_s']:
        p.append('fresh_slow_age_s: must be below lidar_max_age_s (locked)')
    scale = v['speed_scale'] * v['surface_scale'] * v['battery_scale']
    if not SCALE_RANGE[0] - 1e-9 <= scale <= SCALE_RANGE[1] + 1e-9:
        p.append('speed_scale x surface_scale x battery_scale = %.3f outside [%g, %g]' % (scale, *SCALE_RANGE))
    return p


def _reject_constant(token):
    raise ValueError('non-finite number %s' % token)


def _reject_duplicates(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise ValueError('duplicate key %r' % key)
        out[key] = value
    return out


def loads(text):
    """Strict JSON: duplicate keys, NaN / Infinity refused."""
    try:
        return json.loads(text, parse_constant=_reject_constant, object_pairs_hook=_reject_duplicates)
    except ValueError as exc:
        raise TuningError(['file: %s' % exc]) from None


def _grouped(section, groups, where):
    """Flatten {"basic": {...}, ...} into {key: value}, refusing unknown groups / keys and keys of another group."""
    problems, flat = [], {}
    if not isinstance(section, dict):
        return {}, ['%s: must be an object of the groups %s' % (where, groups)]
    for group, entries in section.items():
        if group not in groups:
            problems.append('%s.%s: not a group (%s)' % (where, group, ', '.join(groups)))
            continue
        if not isinstance(entries, dict):
            problems.append('%s.%s: must be an object' % (where, group))
            continue
        for name, value in entries.items():
            key = BY_NAME.get(name)
            if key is None:
                problems.append('%s.%s.%s: not a tuning key' % (where, group, name))
            elif key.group != group:
                problems.append('%s.%s.%s: belongs to the group %s' % (where, group, name, key.group))
            else:
                flat[name] = value
    return flat, problems


def resolve(overrides):
    """defaults() + checked overrides; raises TuningError with every problem (ranges, types, cross rules)."""
    values, problems = defaults(), []
    for name, value in overrides.items():
        checked, why = check_value(name, value)
        problems += why
        if not why:
            values[name] = checked
    if not problems:
        problems += cross_problems(values)
    if problems:
        raise TuningError(problems)
    return values


def _name_ok(name):
    return isinstance(name, str) and 1 <= len(name) <= 40 and set(name) <= NAME_CHARS


@dataclass
class TuningFile:
    path: str
    sha256: str
    preset: str
    note: str
    values: dict
    presets: dict             # name -> dict(note, surface, battery, values (resolved))


def parse(raw, path='<memory>'):
    """A TuningFile from the file bytes (every check of the module docstring)."""
    if len(raw) > MAX_FILE_BYTES:
        raise TuningError(['file: larger than %d bytes' % MAX_FILE_BYTES])
    try:
        text = raw.decode('utf-8')
    except UnicodeDecodeError:
        raise TuningError(['file: not UTF-8']) from None
    data = loads(text)
    if not isinstance(data, dict):
        raise TuningError(['file: must be a JSON object'])
    problems = ['file: unknown key %r' % k for k in sorted(set(data) - {'schema', 'preset', 'note', 'values', 'presets'})]
    if data.get('schema') != SCHEMA:
        problems.append('schema: expected %s' % SCHEMA)
    preset = data.get('preset', DEFAULT_PRESET)
    if not _name_ok(preset):
        problems.append('preset: a name of 1-40 letters, digits, _ - .')
    note = data.get('note', '')
    if not isinstance(note, str) or len(note) > 400:
        problems.append('note: a string of at most 400 characters')
    flat, why = _grouped(data.get('values', {}), GROUPS, 'values')
    problems += why
    presets = {}
    raw_presets = data.get('presets', {})
    if not isinstance(raw_presets, dict) or len(raw_presets) > MAX_PRESETS:
        problems.append('presets: an object of at most %d presets' % MAX_PRESETS)
        raw_presets = {}
    for name, entry in raw_presets.items():
        if not _name_ok(name) or name == DEFAULT_PRESET:
            problems.append('presets.%s: a name of 1-40 letters, digits, _ - . other than %r' % (name, DEFAULT_PRESET))
            continue
        if not isinstance(entry, dict) or set(entry) - {'note', 'surface', 'battery', 'values'}:
            problems.append('presets.%s: keys note, surface, battery, values' % name)
            continue
        extra = [m for m in ('note', 'surface', 'battery') if not isinstance(entry.get(m, ''), str)
                 or len(entry.get(m, '')) > 200]
        problems += ['presets.%s.%s: a string of at most 200 characters' % (name, m) for m in extra]
        pflat, why = _grouped(entry.get('values', {}), ('basic', 'advanced'), 'presets.%s.values' % name)
        problems += why
        try:
            presets[name] = dict(note=entry.get('note', ''), surface=entry.get('surface', ''),
                                 battery=entry.get('battery', ''), values=resolve(pflat))
        except TuningError as exc:
            problems += ['presets.%s: %s' % (name, p) for p in exc.problems]
    values = None
    try:
        values = resolve(flat)
    except TuningError as exc:
        problems += exc.problems
    if problems:
        raise TuningError(problems)
    return TuningFile(path=str(path), sha256=hashlib.sha256(raw).hexdigest(), preset=preset, note=note, values=values,
                      presets=presets)


def load(path=None):
    """Read and parse a tuning file (None = the package default)."""
    path = PACKAGE_FILE if path is None else str(path)
    try:
        with open(path, 'rb') as fh:
            raw = fh.read(MAX_FILE_BYTES + 1)
    except OSError as exc:
        raise TuningError(['file %s: %s' % (path, exc)]) from None
    return parse(raw, path)


def lock_problems(values, cfg):
    """Locked keys that live in the executor config must equal it (one source of the safety limits)."""
    p = []
    for key in KEYS:
        if key.group == 'locked' and key.target[0] == 'config':
            have = getattr(cfg, key.target[1])
            if have is None or abs(float(have) - float(values[key.name])) > 1e-9:
                p.append('%s: the tuning file says %r, the executor config %r (locked: one source, edit the executor '
                         'config or the file)' % (key.name, values[key.name], have))
    return p


def speed_scale(values):
    return values['speed_scale'] * values['surface_scale'] * values['battery_scale']


def controller_fields(values):
    """{SteerFollowConfig field: value} of the resolved values (the speed scale is the product of the three factors)."""
    out = {}
    for key in KEYS:
        if key.target[0] in ('controller', 'lock_controller'):
            out[key.target[1]] = values[key.name]
    out['speed_scale'] = round(speed_scale(values), 9)
    out['speed_table'] = [list(point) for point in values['speed_table']]
    return out


def controller_field_names():
    """The keys of controller_fields() (the executor config validates tuning_values against them)."""
    return {k.target[1] for k in KEYS if k.target[0] in ('controller', 'lock_controller')} | {'speed_scale', 'speed_table'}


def config_fields(values):
    """The executor config keys the tuning sets: the actuator prior's table and scale."""
    return dict(prior_speed_table=tuple(tuple(p) for p in values['speed_table']),
                prior_speed_scale=round(speed_scale(values), 9))


def grouped(values):
    """{'basic': {...}, 'advanced': {...}, 'locked': {...}} (JSON-able)."""
    out = {g: {} for g in GROUPS}
    for key in KEYS:
        out[key.group][key.name] = _canonical(values[key.name])
    return out


def describe():
    """The key table for screens: name, group, unit, range, default, note."""
    return [dict(name=k.name, group=k.group, unit=k.unit, low=k.low, high=k.high, default=_canonical(k.default),
                 note=k.note) for k in KEYS]


def dumps(values, preset=DEFAULT_PRESET, note='', presets=None):
    """The file text of resolved values (every group, sorted; LF), with presets (name -> TuningFile.presets entry)."""
    data = dict(schema=SCHEMA, preset=preset, note=note, values=grouped(values))
    if presets:
        base = grouped(defaults())
        data['presets'] = {
            name: dict(note=e.get('note', ''), surface=e.get('surface', ''), battery=e.get('battery', ''),
                       values={g: {k: v for k, v in entries.items() if v != base[g][k]}         # only what differs
                               for g, entries in grouped(e['values']).items() if g != 'locked'})
            for name, e in sorted(presets.items())}
    return json.dumps(data, indent=1, sort_keys=True, ensure_ascii=False, allow_nan=False) + '\n'


class State:
    """The tuning in force in one executor loop: the file it came from, the values, the preset, the revision (+1 per
    accepted change). change() checks a request without changing anything; commit() applies the checked result."""

    API_KEYS = ('preset', 'save', 'save_preset', 'note')

    def __init__(self, tfile):
        self.file = tfile
        self.values = dict(tfile.values)
        self.preset = tfile.preset
        self.presets = dict(tfile.presets)
        self.revision = 0
        self.saved_revision = 0

    @property
    def digest(self):
        return digest(self.values)

    @property
    def writable(self):
        """Saving goes to the configured file only, never to the package default (it is part of SHA256SUMS)."""
        return os.path.abspath(self.file.path) != os.path.abspath(PACKAGE_FILE)

    def preset_values(self, name):
        if name == DEFAULT_PRESET:
            return defaults()
        if name not in self.presets:
            raise TuningError(['preset %r: unknown (%s)' % (name, ', '.join([DEFAULT_PRESET] + sorted(self.presets)))])
        return dict(self.presets[name]['values'])

    def change(self, params):
        """(values, preset, changes, action) of an API request (basic keys only, or a preset, or a save); raises
        TuningError (the HTTP 400 of the contract) with every problem."""
        if not isinstance(params, dict) or not params:
            raise TuningError(['params: a non-empty object'])
        problems = []
        keys = set(params)
        unknown = sorted(k for k in keys if k not in BY_NAME and k not in self.API_KEYS)
        problems += ['%s: not a tuning key' % k for k in unknown]
        locked = sorted(k for k in keys if k in BY_NAME and BY_NAME[k].group != 'basic')
        problems += ['%s: group %s is not editable from the screen (file only%s)'
                     % (k, BY_NAME[k].group, ', restart' if BY_NAME[k].group == 'locked' else '') for k in locked]
        if problems:
            raise TuningError(problems)
        values, preset = dict(self.values), self.preset
        if 'preset' in params:
            if keys - {'preset'}:
                raise TuningError(['preset: alone (load a preset, then change values)'])
            values, preset = self.preset_values(params['preset']), params['preset']
            action = 'load_preset'
        elif 'save' in params or 'save_preset' in params:
            if params.get('save', True) is not True or keys - {'save', 'save_preset', 'note'}:
                raise TuningError(['save: {"save": true} or {"save_preset": name, "note": text}'])
            if 'save_preset' in params and (not _name_ok(params['save_preset'])
                                            or params['save_preset'] == DEFAULT_PRESET):
                raise TuningError(['save_preset: a name of 1-40 letters, digits, _ - . other than %r' % DEFAULT_PRESET])
            note = params.get('note', '')
            if not isinstance(note, str) or len(note) > 200:
                raise TuningError(['note: a string of at most 200 characters'])
            if not self.writable:
                raise TuningError(['save: the package default file is read only (configure tuning_file)'])
            action = 'save_preset' if 'save_preset' in params else 'save'
        else:
            override = {k: v for k, v in self.values.items()}
            override.update({k: params[k] for k in keys})
            values = resolve({k: v for k, v in override.items()})
            action = 'set'
            if any(values[k] != self.values[k] for k in values):
                preset = self.preset if self.preset.endswith('*') else self.preset + '*'   # modified from the preset
        changes = [dict(key=k, old=_canonical(self.values[k]), new=_canonical(values[k]))
                   for k in sorted(values) if values[k] != self.values[k]]
        return values, preset, changes, action

    def commit(self, values, preset):
        self.values, self.preset = dict(values), preset
        self.revision += 1

    def save(self, preset_name=None, note=''):
        """Write the file (atomic: a temporary file, then rename). preset_name: also store the values as that preset."""
        if preset_name is not None:
            self.presets[preset_name] = dict(note=note, surface='', battery='', values=dict(self.values))
        text = dumps(self.values, preset=preset_name or self.preset.rstrip('*'), note=self.file.note,
                     presets=self.presets)
        tmp = self.file.path + '.tmp%d' % os.getpid()
        with open(tmp, 'wb') as fh:
            fh.write(text.encode('utf-8'))
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, self.file.path)
        self.file = parse(text.encode('utf-8'), self.file.path)
        self.preset = self.file.preset
        self.saved_revision = self.revision
        return self.file.sha256

    def brief(self):
        return dict(preset=self.preset, revision=self.revision, digest=self.digest, file=self.file.path,
                    file_sha256=self.file.sha256)

    def status(self, editable, why_not=None):
        basic = [dict(name=k.name, value=_canonical(self.values[k.name]), low=k.low, high=k.high, unit=k.unit)
                 for k in KEYS if k.group == 'basic']
        return dict(self.brief(), editable=bool(editable), not_editable_because=why_not,
                    modified=self.preset.endswith('*'), presets=[DEFAULT_PRESET] + sorted(self.presets),
                    writable=self.writable, unsaved=self.revision != self.saved_revision, basic=basic,
                    values=grouped(self.values))
