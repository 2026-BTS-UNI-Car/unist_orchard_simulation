"""Scene specification of the steering fast SIM (design 2.3 "장면 명세", 4.5, 5.1, 5.3, 7.3; 8.3 P2).

A scene names everything an episode needs and nothing else:

  world        W0 (flat calibration floor, optional LiDAR posts; flat_world.py), W2 (a 10/10 raster corridor
               of the single-row catalogue: record, catalogue id, direction, modifiers; the catalogue and the
               maps are found through closed_loop/external_paths.py, never through a built-in path) or W3 (the
               statistics-matched procedural orchard of procedural_orchard.py: 'procedural' parameters with a seed,
               modifiers; the corridor of an episode is its corridor_index aisle, travel +x)
  start        base pose (x, y in m, yaw in deg; W0: the world frame, W2: the corridor frame s, d, heading
               error), the true wheel-side equivalent steering angle at the start (deg) and a condition label
  profile      vehicle profile id (file in steering/profiles/), optional expected SHA-256, optional overrides
               (VehicleProfile.with_overrides), drive curve role, geometry_check, steering mode, optional
               plant_tables (A12 twin form, SIM_ACCEPTANCE amendment 3: per-delta steering efficiency eta(|delta|)
               and turning speed loss(|delta|, v) tables that replace the profile's constant eta / understeer /
               scrub draws in the F-SIM plant; written only when given, so other scenes keep their SHA-256)
  seeds        one episode per seed
  backend      steering backend level: 'profile' (the profile's steer_actuator_level), 'L0' (Python ideal servo,
               l0_servo.py) or 'L1' (native C++ actuator model, actuator_link.py). L0 / L1 set the profile's
               steer_actuator_level in the profile view, so the plant label always matches the backend.
  driver       exactly one of maneuver (open loop, maneuvers.py) or controller (closed loop; see CONTROLLERS)
  runner       'direct' (the fast_sim plant loop) or 'harness' (closed_loop.harness.run_episode with the
               plant_factory and actuation hooks)
  timing       duration_s (SIM time cap), command_latency_s (harness default 0.05 s), trace_every
  stop_rules   [{kind, value}] (direct runner only): travel_m, field_x_m, turned_deg, rest_s, abs_lateral_m

Canonical JSON: every field written (defaults filled in, maneuver parameters normalised by maneuvers.py), sorted
keys, compact separators, ASCII, no NaN. scene_sha256 is the SHA-256 of the canonical JSON of the scene;
episode_sha256(seed) is the SHA-256 of the same scene with seeds = [seed], so adding a seed does not change
the identity of the other episodes. Every result stores both (design 7.3). Note: numbers inside the free-form
profile overrides are kept as written (30 and 30.0 give different SHAs there); every other number is normalised.

Randomness: like closed_loop.harness, every random stream of an episode comes from (seed, CRC-32 of the stream
key, purpose); the stream key defaults to the scene name. Scenes with the same seed and different names do
not share draws; the direct and the harness runner of one scene and seed give the plant the same seed.

Sealed corridors (single-row amendment A1, design 7.3): the three report-only directions and the reverse direction
of each sealed row pair (design 7.3: the same row pair reversed is not development evidence) are refused unless
world.allow_sealed is true. sealed_status() is the one rule; entry_feasibility (A24) uses it too.

Repository location (design 4.5: runs and builds live outside the repository): repository_roots() lists the root of
this work tree and of every work tree of the same git repository (read from the git files, no git command);
inside_repository(path) is True for a path inside any of them. fast_sim, evaluate_steer and entry_feasibility use it.

Pure Python (json, hashlib, os, re, pathlib). Placeholder-based scenes give placeholder-based SIM results, not field
validation.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
import hashlib
import json
import math
import os
from pathlib import Path
import re
from typing import Optional, Tuple

from . import maneuvers

SCHEMA = 'ubts.steer_sim.scene_spec.v1'
NAME_PATTERN = re.compile(r'[a-z0-9][a-z0-9_.-]{0,95}\Z')
LABEL_PATTERN = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.+-]{0,95}\Z')
PROFILE_ID_PATTERN = re.compile(r'[a-z0-9_]+\Z')
SHA256_PATTERN = re.compile(r'[0-9a-f]{64}\Z')
WORLD_KINDS = ('W0', 'W2', 'W3')
PLANT_TABLE_KEYS = ('label', 'eta_by_delta_deg', 'loss_by_delta_deg', 'loss_speeds_m_s')
DIRECTIONS = ('fwd', 'rev')
BACKEND_LEVELS = ('profile', 'L0', 'L1')
RUNNERS = ('direct', 'harness')
ESTIMATORS = ('sensor', 'null')
STEER_MODES = ('test_servo',)                      # direct_pwm and firmware drivers are not part of P2
CURVES = ('main', 'bracket', 'P_fit1003', 'field1010')
STOP_RULE_KINDS = ('travel_m', 'field_x_m', 'turned_deg', 'rest_s', 'abs_lateral_m')
DEFAULT_PROFILE_ID = 'front_steer_v0_1_placeholder'
DEFAULT_COMMAND_LATENCY_S = .05                    # closed_loop.harness.EpisodeConfig.command_latency_s
MAX_DURATION_S = 3600.
MAX_SEEDS = 100000
MAX_MARKERS = 64
# Single-row amendment A1 (SEALED_CORRIDORS.json; design 7.3): (record, catalogue id, direction).
SEALED_DIRECTIONS = (('042301', 'c01', 'fwd'), ('042301', 'c04', 'fwd'), ('042301', 'c07', 'fwd'))
_OTHER_DIRECTION = dict(fwd='rev', rev='fwd')
# Design 7.3: the reverse direction of a sealed row pair is not development evidence either.
SEALED_PAIR_REVERSE_DIRECTIONS = tuple((r, c, _OTHER_DIRECTION[d]) for r, c, d in SEALED_DIRECTIONS)
SEALED_STATUS_REASONS = {
    'sealed': 'sealed report-only direction (single-row amendment A1)',
    'sealed_pair_reverse': 'reverse direction of a sealed row pair (design 7.3: not development evidence)',
}

# Closed-loop drivers. probe_pure_pursuit is the survey probe re-run (design 6.2, 8.3 P2): pure pursuit on a
# noisy observation made from the TRUE rear-axle pose relative to the line y = 0 (truth-fed stand-in for the row
# estimator, exactly like steering-sim-20261108/survey/original_ackermann_probe). It is NOT an executor; results
# carry truth_fed_driver = true. Direct runner on W0 only.
CONTROLLERS = {
    'probe_pure_pursuit': {
        'lookahead_m': ('positive', 2.0),
        'throttle_pct': ('optional_throttle', None),
        'speed_m_s': ('optional_positive', None),
        'row_length_m': ('positive', 30.0),
        'obs_hz': ('positive', 10.0),
        'obs_latency_s': ('nonneg', 0.1),
        'obs_offset_sd_m': ('nonneg', 0.03),
        'obs_heading_sd_deg': ('nonneg', 1.0),
        'max_yaw_rate_rad_s': ('positive', 0.8),
        'wheelbase_m': ('optional_positive', None),     # None: the profile nominal wheelbase (a config value)
    },
}


class SceneError(ValueError):
    """A scene specification is not valid."""


def _err(code, detail=''):
    return SceneError(code + (': ' + detail if detail else ''))


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _err('scene_value_not_a_number', name)
    value = float(value)
    if not math.isfinite(value):
        raise _err('scene_value_not_finite', name)
    return value


def _check_keys(data, allowed, where):
    if not isinstance(data, dict):
        raise _err('scene_not_a_mapping', where)
    unknown = sorted(str(k) for k in set(data) - set(allowed))
    if unknown:
        raise _err('scene_key_unknown', '%s: %s' % (where, ', '.join(unknown)))


def _json_value(value, where):
    """A deep copy that must survive a strict JSON round trip (no NaN, string keys)."""
    try:
        text = json.dumps(value, sort_keys=True, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise _err('scene_value_not_json', '%s (%s)' % (where, exc))
    return json.loads(text)


def canonical_json(value):
    """Canonical bytes: sorted keys, compact separators, ASCII, no NaN."""
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True, allow_nan=False).encode('ascii')


def sha256_of(value):
    return hashlib.sha256(canonical_json(value)).hexdigest()


def sealed_status(record, corridor_id, direction):
    """'sealed' (report-only direction), 'sealed_pair_reverse' (the reverse of a sealed row pair) or None. The one
    rule for scenes (WorldSpec, refused without allow_sealed) and for the A24 list (entry_feasibility)."""
    key = (record, corridor_id, direction)
    if key in SEALED_DIRECTIONS:
        return 'sealed'
    if key in SEALED_PAIR_REVERSE_DIRECTIONS:
        return 'sealed_pair_reverse'
    return None


# --- repository location (runs and builds live outside the repository, design 4.5) ---------------------------------
WORK_TREE_ROOT = Path(__file__).resolve().parents[3]
_WINDOWS_DRIVE_PATH = re.compile(r'^([A-Za-z]):[\\/](.*)$')
_WSL_DRIVE_PATH = re.compile(r'^/mnt/([A-Za-z])/(.*)$')


def _git_file_path(text, base):
    """A path written in a git file (.git, commondir, worktrees/*/gitdir): absolute, or relative to `base` (the folder
    of the file that names it; git worktree.useRelativePaths writes relative paths). A work tree made by Windows git
    stores drive paths (D:/...); read on POSIX they name the WSL mount /mnt/<drive>/... (the WSL default mount; a path
    that does not exist here, for example inside a container, is left out by the caller). Read on Windows,
    /mnt/<drive>/... names that drive."""
    text = text.strip()
    if os.name == 'nt':
        match = _WSL_DRIVE_PATH.match(text)
        if match:
            return Path('%s:/' % match.group(1).upper()) / match.group(2)
    else:
        match = _WINDOWS_DRIVE_PATH.match(text)
        if match:
            return Path('/mnt') / match.group(1).lower() / match.group(2).replace('\\', '/')
    path = Path(text)
    return path if path.is_absolute() else (Path(base) / path)


def _git_dir_of(root):
    """The git dir of a work tree root (.git folder, or the folder a .git file points to), or None."""
    marker = Path(root) / '.git'
    if marker.is_dir():
        return marker
    if marker.is_file():
        try:
            text = marker.read_bytes().decode('utf-8', 'replace').strip()
        except OSError:
            return None
        if text.startswith('gitdir:'):
            return _git_file_path(text[len('gitdir:'):], root)
    return None


def repository_roots(root=None):
    """Resolved roots of every work tree of the git repository of `root` (default: this work tree), read from the git
    files (.git, commondir, worktrees/*/gitdir; _git_file_path); no git command. Always includes `root` itself. Paths
    that do not exist here (for example a host path inside a container) are left out."""
    root = Path(WORK_TREE_ROOT if root is None else root).resolve()
    out = {root}
    git_dir = _git_dir_of(root)
    if git_dir is None or not git_dir.is_dir():
        return sorted(out)
    common = git_dir
    commondir = git_dir / 'commondir'
    if commondir.is_file():
        try:
            common = _git_file_path(commondir.read_bytes().decode('utf-8', 'replace'), git_dir)
        except OSError:
            pass
    common = common.resolve()
    if common.name == '.git' and common.parent.is_dir():
        out.add(common.parent.resolve())                  # the main work tree
    worktrees = common / 'worktrees'
    if worktrees.is_dir():
        for entry in worktrees.iterdir():
            gitdir = entry / 'gitdir'
            if not gitdir.is_file():
                continue
            try:
                text = gitdir.read_bytes().decode('utf-8', 'replace').strip()
            except OSError:
                continue
            tree = _git_file_path(text, entry).parent
            if tree.is_dir():
                out.add(tree.resolve())
    return sorted(out)


def inside_repository(path, roots=None):
    """True if `path` lies inside this work tree or any other work tree of the same git repository."""
    target = Path(path).resolve()
    for root in (repository_roots() if roots is None else roots):
        try:
            target.relative_to(Path(root))
            return True
        except ValueError:
            continue
    return False


def normalise_controller_params(kind, params):
    """Validated, filled controller parameters (dict). Raises SceneError."""
    if kind not in CONTROLLERS:
        raise _err('controller_kind_unknown', repr(kind))
    schema = CONTROLLERS[kind]
    params = {} if params is None else params
    _check_keys(params, schema, 'controller.params')
    out = {}
    for name, (kind_, default) in schema.items():
        value = params.get(name, default)
        if value is None:
            if not kind_.startswith('optional'):
                raise _err('controller_param_missing', name)
            out[name] = None
            continue
        value = _number(value, 'controller.params.' + name)
        if kind_ in ('positive', 'optional_positive') and not value > 0.:
            raise _err('controller_param_not_positive', name)
        if kind_ == 'nonneg' and value < 0.:
            raise _err('controller_param_negative', name)
        if kind_ == 'optional_throttle' and not 0. < value <= 100.:
            raise _err('controller_param_out_of_range', name)
        out[name] = value
    if kind == 'probe_pure_pursuit' and (out['throttle_pct'] is None) == (out['speed_m_s'] is None):
        raise _err('controller_needs_exactly_one_of_throttle_pct_and_speed_m_s')
    return out


# --- parts --------------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class WorldSpec:
    """W0: flat floor with optional hard posts ((x, y, radius) in m). W2: one catalogued corridor direction."""

    kind: str = 'W0'
    markers: Tuple[Tuple[float, float, float], ...] = ()
    record: Optional[str] = None
    corridor_id: Optional[str] = None
    direction: str = 'fwd'
    modifiers: Tuple[dict, ...] = ()
    allow_sealed: bool = False
    procedural: Optional[dict] = None
    allow_holdout: bool = False

    @classmethod
    def from_dict(cls, data):
        data = {} if data is None else data
        _check_keys(data, ('kind', 'markers', 'record', 'corridor_id', 'direction', 'modifiers', 'allow_sealed',
                           'procedural', 'allow_holdout'), 'world')
        kind = data.get('kind', 'W0')
        if kind not in WORLD_KINDS:
            raise _err('world_kind_unknown', repr(kind))
        markers = data.get('markers') or ()
        if not isinstance(markers, (list, tuple)) or len(markers) > MAX_MARKERS:
            raise _err('world_markers_invalid')
        posts = []
        for item in markers:
            if not isinstance(item, (list, tuple)) or len(item) != 3:
                raise _err('world_marker_needs_x_y_radius')
            x, y, r = (_number(v, 'world.markers') for v in item)
            if not r > 0.:
                raise _err('world_marker_radius_not_positive')
            posts.append((x, y, r))
        allow_sealed = data.get('allow_sealed', False)
        if not isinstance(allow_sealed, bool):
            raise _err('world_allow_sealed_not_bool')
        direction = data.get('direction', 'fwd')
        if direction not in DIRECTIONS:
            raise _err('world_direction_unknown', repr(direction))
        modifiers = data.get('modifiers') or ()
        if not isinstance(modifiers, (list, tuple)):
            raise _err('world_modifiers_not_a_list')
        mods = []
        for item in modifiers:
            if not isinstance(item, dict) or not isinstance(item.get('kind'), str):
                raise _err('world_modifier_needs_kind')
            mods.append(_json_value(item, 'world.modifiers'))
        record, corridor = data.get('record'), data.get('corridor_id')
        procedural = data.get('procedural')
        allow_holdout = data.get('allow_holdout', False)
        if not isinstance(allow_holdout, bool):
            raise _err('world_allow_holdout_not_bool')
        if kind != 'W3' and procedural is not None:
            raise _err('world_procedural_params_only_for_w3')
        if kind != 'W3' and allow_holdout:
            raise _err('world_allow_holdout_only_for_w3')
        if kind == 'W0':
            if record is not None or corridor is not None or mods or direction != 'fwd' or allow_sealed:
                raise _err('world_w0_takes_markers_only')
        elif kind == 'W3':
            if record is not None or corridor is not None or posts or direction != 'fwd' or allow_sealed:
                raise _err('world_w3_takes_procedural_params_and_modifiers_only')
            from . import procedural_orchard
            try:
                procedural = procedural_orchard.normalise_params(procedural)
            except ValueError as exc:
                raise _err('world_w3_procedural_params_invalid', str(exc)) from None
            if procedural_orchard.is_holdout_seed(procedural['seed']) and not allow_holdout:
                raise _err('world_w3_holdout_seed', 'seed %d is a holdout seed (%d-%d); set allow_holdout only for the '
                           'pre-registered evaluation' % ((procedural['seed'],) + procedural_orchard.HOLDOUT_SEEDS))
        else:
            if posts:
                raise _err('world_w2_takes_no_markers')
            if not (isinstance(record, str) and re.match(r'[0-9]{6}\Z', record)):
                raise _err('world_w2_record_invalid', repr(record))
            if not (isinstance(corridor, str) and re.match(r'c[0-9]{2}\Z', corridor)):
                raise _err('world_w2_corridor_id_invalid', repr(corridor))
            status = sealed_status(record, corridor, direction)
            if status is not None and not allow_sealed:
                raise _err('world_w2_sealed_corridor', '%s %s %s: %s; set allow_sealed only for the pre-registered '
                           'report runs' % (record, corridor, direction, SEALED_STATUS_REASONS[status]))
        return cls(kind=kind, markers=tuple(posts), record=record, corridor_id=corridor, direction=direction,
                   modifiers=tuple(mods), allow_sealed=allow_sealed, procedural=procedural, allow_holdout=allow_holdout)

    def to_dict(self):
        out = dict(kind=self.kind, markers=[list(m) for m in self.markers], record=self.record,
                   corridor_id=self.corridor_id, direction=self.direction,
                   modifiers=[copy.deepcopy(m) for m in self.modifiers], allow_sealed=self.allow_sealed)
        if self.kind == 'W3':                   # written only for W3: W0 / W2 scenes keep their canonical JSON
            out['procedural'] = copy.deepcopy(self.procedural)
            if self.allow_holdout:              # written only when set: development W3 scenes keep theirs
                out['allow_holdout'] = True
        return out


@dataclass(frozen=True)
class StartSpec:
    """Start pose of the base (footprint centre) and start condition; steer_deg = true wheel-side angle."""

    x_m: float = 0.
    y_m: float = 0.
    yaw_deg: float = 0.
    steer_deg: float = 0.
    condition: str = 'nominal'

    @classmethod
    def from_dict(cls, data):
        data = {} if data is None else data
        _check_keys(data, ('x_m', 'y_m', 'yaw_deg', 'steer_deg', 'condition'), 'start')
        condition = data.get('condition', 'nominal')
        if not (isinstance(condition, str) and LABEL_PATTERN.match(condition)):
            raise _err('start_condition_invalid', repr(condition))
        steer = _number(data.get('steer_deg', 0.), 'start.steer_deg')
        if abs(steer) >= 60.:
            raise _err('start_steer_out_of_range')
        yaw = _number(data.get('yaw_deg', 0.), 'start.yaw_deg')
        if abs(yaw) > 180.:
            raise _err('start_yaw_out_of_range')
        return cls(x_m=_number(data.get('x_m', 0.), 'start.x_m'), y_m=_number(data.get('y_m', 0.), 'start.y_m'),
                   yaw_deg=yaw, steer_deg=steer, condition=condition)

    def to_dict(self):
        return dict(x_m=self.x_m, y_m=self.y_m, yaw_deg=self.yaw_deg, steer_deg=self.steer_deg,
                    condition=self.condition)


@dataclass(frozen=True)
class ProfileSpec:
    """Vehicle profile view: file id, optional SHA-256 pin, overrides (VehicleProfile.with_overrides), curve role,
    geometry_check (ideal values, A1 / A12 only) and steering mode."""

    profile_id: str = DEFAULT_PROFILE_ID
    sha256: Optional[str] = None
    overrides: dict = field(default_factory=dict)
    override_label: Optional[str] = None
    allow_out_of_range: bool = False
    curve: str = 'main'
    geometry_check: bool = False
    steer_mode: str = 'test_servo'
    plant_tables: Optional[dict] = None

    @classmethod
    def from_dict(cls, data, scene_name):
        data = {} if data is None else data
        _check_keys(data, ('profile_id', 'sha256', 'overrides', 'override_label', 'allow_out_of_range', 'curve',
                           'geometry_check', 'steer_mode', 'plant_tables'), 'profile')
        profile_id = data.get('profile_id', DEFAULT_PROFILE_ID)
        if not (isinstance(profile_id, str) and PROFILE_ID_PATTERN.match(profile_id)):
            raise _err('profile_id_invalid', repr(profile_id))
        sha = data.get('sha256')
        if sha is not None and not (isinstance(sha, str) and SHA256_PATTERN.match(sha)):
            raise _err('profile_sha256_invalid')
        overrides = data.get('overrides') or {}
        if not isinstance(overrides, dict):
            raise _err('profile_overrides_not_a_mapping')
        overrides = _json_value(overrides, 'profile.overrides')
        label = data.get('override_label')
        if overrides:
            label = label or 'scene_' + scene_name
            if not (isinstance(label, str) and LABEL_PATTERN.match(label)):
                raise _err('profile_override_label_invalid', repr(label))
        elif label is not None:
            raise _err('profile_override_label_without_overrides')
        flags = {}
        for name in ('allow_out_of_range', 'geometry_check'):
            flags[name] = data.get(name, False)
            if not isinstance(flags[name], bool):
                raise _err('profile_flag_not_bool', name)
        curve = data.get('curve', 'main')
        if curve not in CURVES:
            raise _err('profile_curve_unknown', repr(curve))
        mode = data.get('steer_mode', 'test_servo')
        if mode not in STEER_MODES:
            raise _err('profile_steer_mode_not_supported', '%r (P2 drivers give target angles: %s)'
                       % (mode, STEER_MODES))
        tables = normalise_plant_tables(data.get('plant_tables'))
        if tables is not None and flags['geometry_check']:
            raise _err('profile_plant_tables_exclude_geometry_check', 'the tables replace the ideal values')
        return cls(profile_id=profile_id, sha256=sha, overrides=overrides, override_label=label,
                   allow_out_of_range=flags['allow_out_of_range'], curve=curve,
                   geometry_check=flags['geometry_check'], steer_mode=mode, plant_tables=tables)

    def to_dict(self):
        out = dict(profile_id=self.profile_id, sha256=self.sha256, overrides=copy.deepcopy(self.overrides),
                   override_label=self.override_label, allow_out_of_range=self.allow_out_of_range,
                   curve=self.curve, geometry_check=self.geometry_check, steer_mode=self.steer_mode)
        if self.plant_tables is not None:      # written only when given: other scenes keep their canonical JSON
            out['plant_tables'] = copy.deepcopy(self.plant_tables)
        return out


def normalise_plant_tables(tables):
    """Validated plant tables (or None): label, eta_by_delta_deg [[|delta| deg, eta], ...] (ascending, eta(0) = 1 is
    implied when 0 is not given), optional loss_by_delta_deg [[|delta| deg, loss at v1, loss at v2, ...], ...] with
    loss_speeds_m_s [v1, v2, ...] (ascending; loss(0, v) = 0 implied)."""
    if tables is None:
        return None
    _check_keys(tables, PLANT_TABLE_KEYS, 'profile.plant_tables')
    label = tables.get('label')
    if not (isinstance(label, str) and LABEL_PATTERN.match(label)):
        raise _err('plant_tables_label_invalid', repr(label))
    eta = tables.get('eta_by_delta_deg')
    if not isinstance(eta, (list, tuple)) or not eta:
        raise _err('plant_tables_eta_needs_rows')
    eta_rows = []
    for row in eta:
        if not isinstance(row, (list, tuple)) or len(row) != 2:
            raise _err('plant_tables_eta_row_needs_delta_eta')
        d, e = (_number(v, 'plant_tables.eta_by_delta_deg') for v in row)
        if not 0. <= d <= 60. or not 0. < e <= 1.5 or (eta_rows and not d > eta_rows[-1][0]):
            raise _err('plant_tables_eta_row_out_of_range_or_order')
        eta_rows.append([d, e])
    out = dict(label=label, eta_by_delta_deg=eta_rows, loss_by_delta_deg=None, loss_speeds_m_s=None)
    loss, speeds = tables.get('loss_by_delta_deg'), tables.get('loss_speeds_m_s')
    if (loss is None) != (speeds is None):
        raise _err('plant_tables_loss_needs_rows_and_speeds')
    if loss is not None:
        if not isinstance(speeds, (list, tuple)) or not speeds:
            raise _err('plant_tables_loss_speeds_invalid')
        v = [_number(x, 'plant_tables.loss_speeds_m_s') for x in speeds]
        if any(not x > 0. for x in v) or any(not b > a for a, b in zip(v, v[1:])):
            raise _err('plant_tables_loss_speeds_not_positive_ascending')
        rows = []
        for row in loss:
            if not isinstance(row, (list, tuple)) or len(row) != 1 + len(v):
                raise _err('plant_tables_loss_row_needs_delta_and_one_loss_per_speed')
            values = [_number(x, 'plant_tables.loss_by_delta_deg') for x in row]
            if not 0. <= values[0] <= 60. or any(not 0. <= x < 1. for x in values[1:]) \
                    or (rows and not values[0] > rows[-1][0]):
                raise _err('plant_tables_loss_row_out_of_range_or_order')
            rows.append(values)
        out.update(loss_by_delta_deg=rows, loss_speeds_m_s=v)
    return out


@dataclass(frozen=True)
class DriverSpec:
    """An open-loop maneuver (maneuvers.MANEUVERS) or a closed-loop controller (CONTROLLERS); params normalised."""

    kind: str
    params: dict = field(default_factory=dict)

    def to_dict(self):
        return dict(kind=self.kind, params=copy.deepcopy(self.params))


@dataclass(frozen=True)
class StopRule:
    kind: str
    value: float

    def to_dict(self):
        return dict(kind=self.kind, value=self.value)


# --- scene ----------------------------------------------------------------------------------------------------------
SCENE_KEYS = ('schema', 'name', 'stream_key', 'world', 'start', 'profile', 'seeds', 'backend', 'runner', 'estimator',
              'maneuver', 'controller', 'duration_s', 'command_latency_s', 'gyro_noise_scale', 'stop_rules',
              'trace_every')


@dataclass(frozen=True)
class SceneSpec:
    """One validated scene (use SceneSpec.from_dict / scene()). Immutable value; to_dict() gives the canonical dict."""

    name: str
    stream_key: str
    world: WorldSpec
    start: StartSpec
    profile: ProfileSpec
    seeds: Tuple[int, ...]
    backend: str
    runner: str
    estimator: Optional[str]
    maneuver: Optional[DriverSpec]
    controller: Optional[DriverSpec]
    duration_s: float
    command_latency_s: float
    gyro_noise_scale: float
    stop_rules: Tuple[StopRule, ...]
    trace_every: int

    @classmethod
    def from_dict(cls, data):
        """Validate and normalise a scene dict (defaults filled in). Raises SceneError."""
        _check_keys(data, SCENE_KEYS, 'scene')
        schema = data.get('schema', SCHEMA)
        if schema != SCHEMA:
            raise _err('scene_schema_mismatch', '%r (need %r)' % (schema, SCHEMA))
        name = data.get('name')
        if not (isinstance(name, str) and NAME_PATTERN.match(name)):
            raise _err('scene_name_invalid', '%r (lower-case letters, digits, _ . -; at most 96)' % (name,))
        stream_key = data.get('stream_key') or name
        if not (isinstance(stream_key, str) and LABEL_PATTERN.match(stream_key)):
            raise _err('scene_stream_key_invalid', repr(stream_key))
        world = WorldSpec.from_dict(data.get('world'))
        start = StartSpec.from_dict(data.get('start'))
        profile = ProfileSpec.from_dict(data.get('profile'), name)
        seeds = data.get('seeds', [0])
        if not isinstance(seeds, (list, tuple)) or not seeds or len(seeds) > MAX_SEEDS:
            raise _err('scene_seeds_invalid')
        for seed in seeds:
            if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0 or seed >= 2 ** 32:
                raise _err('scene_seed_invalid', repr(seed))
        if len(set(seeds)) != len(seeds):
            raise _err('scene_seeds_repeated')
        backend = data.get('backend', 'profile')
        if backend not in BACKEND_LEVELS:
            raise _err('scene_backend_unknown', repr(backend))
        if profile.geometry_check and backend == 'L1':
            raise _err('scene_geometry_check_runs_on_l0', 'geometry_check sets the L0 ideal servo (design 3.3)')
        runner = data.get('runner', 'direct')
        if runner not in RUNNERS:
            raise _err('scene_runner_unknown', repr(runner))
        estimator = data.get('estimator')
        if runner == 'harness':
            estimator = estimator or 'sensor'
            if estimator not in ESTIMATORS:
                raise _err('scene_estimator_unknown', repr(estimator))
        elif estimator is not None:
            raise _err('scene_estimator_needs_the_harness_runner', 'the direct runner has no estimator')
        maneuver_data, controller_data = data.get('maneuver'), data.get('controller')
        if (maneuver_data is None) == (controller_data is None):
            raise _err('scene_needs_exactly_one_of_maneuver_and_controller')
        maneuver = controller = None
        if maneuver_data is not None:
            _check_keys(maneuver_data, ('kind', 'params'), 'maneuver')
            kind = maneuver_data.get('kind')
            try:
                params = maneuvers.normalise_params(kind, maneuver_data.get('params'))
            except maneuvers.ManeuverError as exc:
                raise _err('scene_maneuver_invalid', str(exc)) from None
            maneuver = DriverSpec(kind=kind, params=params)
        else:
            _check_keys(controller_data, ('kind', 'params'), 'controller')
            kind = controller_data.get('kind')
            controller = DriverSpec(kind=kind, params=normalise_controller_params(kind, controller_data.get('params')))
            if kind == 'probe_pure_pursuit' and (runner != 'direct' or world.kind != 'W0'):
                raise _err('scene_probe_needs_the_direct_runner_on_w0', 'the probe controller reads the true pose')
        duration = _number(data.get('duration_s', 30.), 'duration_s')
        if not 0. < duration <= MAX_DURATION_S:
            raise _err('scene_duration_out_of_range')
        latency = _number(data.get('command_latency_s', DEFAULT_COMMAND_LATENCY_S), 'command_latency_s')
        if not 0. <= latency <= 2.:
            raise _err('scene_command_latency_out_of_range')
        gyro = _number(data.get('gyro_noise_scale', 1.), 'gyro_noise_scale')
        if gyro < 0.:
            raise _err('scene_gyro_noise_scale_negative')
        rules = []
        for item in data.get('stop_rules') or ():
            _check_keys(item, ('kind', 'value'), 'stop_rules')
            if item.get('kind') not in STOP_RULE_KINDS:
                raise _err('scene_stop_rule_unknown', repr(item.get('kind')))
            value = _number(item.get('value'), 'stop_rules.value')
            if not value > 0. and item['kind'] != 'field_x_m':
                raise _err('scene_stop_rule_value_not_positive', item['kind'])
            rules.append(StopRule(kind=item['kind'], value=value))
        if rules and runner != 'direct':
            raise _err('scene_stop_rules_need_the_direct_runner', 'the harness runner ends at duration_s or '
                       'after the maneuver is done (terminal hold)')
        if len({r.kind for r in rules}) != len(rules):
            raise _err('scene_stop_rule_repeated')
        every = data.get('trace_every', 1)
        if isinstance(every, bool) or not isinstance(every, int) or not 1 <= every <= 100:
            raise _err('scene_trace_every_invalid')
        return cls(name=name, stream_key=stream_key, world=world, start=start, profile=profile, seeds=tuple(seeds),
                   backend=backend, runner=runner, estimator=estimator, maneuver=maneuver, controller=controller,
                   duration_s=duration, command_latency_s=latency, gyro_noise_scale=gyro, stop_rules=tuple(rules),
                   trace_every=every)

    # -- canonical form and identity
    def to_dict(self):
        return dict(schema=SCHEMA, name=self.name, stream_key=self.stream_key, world=self.world.to_dict(),
                    start=self.start.to_dict(), profile=self.profile.to_dict(), seeds=list(self.seeds),
                    backend=self.backend, runner=self.runner, estimator=self.estimator,
                    maneuver=None if self.maneuver is None else self.maneuver.to_dict(),
                    controller=None if self.controller is None else self.controller.to_dict(),
                    duration_s=self.duration_s, command_latency_s=self.command_latency_s,
                    gyro_noise_scale=self.gyro_noise_scale, stop_rules=[r.to_dict() for r in self.stop_rules],
                    trace_every=self.trace_every)

    def canonical_json(self):
        return canonical_json(self.to_dict())

    def sha256(self):
        return hashlib.sha256(self.canonical_json()).hexdigest()

    def episode_dict(self, seed):
        """The single-seed form of this scene (the scene of one episode)."""
        if seed not in self.seeds:
            raise _err('scene_seed_not_in_scene', repr(seed))
        data = self.to_dict()
        data['seeds'] = [seed]
        return data

    def episode_sha256(self, seed):
        return sha256_of(self.episode_dict(seed))

    def episode_id(self, seed):
        return '%s__seed%d' % (self.name, seed)

    def episodes(self):
        """[(episode_id, seed), ...] in seed order as given."""
        return [(self.episode_id(seed), seed) for seed in self.seeds]

    @property
    def driver(self):
        return self.maneuver if self.maneuver is not None else self.controller

    @property
    def driver_role(self):
        return 'maneuver' if self.maneuver is not None else 'controller'

    def with_changes(self, **changes):
        """A new validated scene from this one's canonical dict with top-level keys replaced."""
        data = self.to_dict()
        unknown = sorted(set(changes) - set(SCENE_KEYS))
        if unknown:
            raise _err('scene_key_unknown', ', '.join(unknown))
        data.update(changes)
        if 'name' in changes and 'stream_key' not in changes and data['stream_key'] == self.name:
            data['stream_key'] = changes['name']
        return SceneSpec.from_dict(data)


def scene(name, **fields):
    """SceneSpec.from_dict(dict(name=name, **fields)) - a short form for code that builds scenes."""
    data = dict(fields)
    data['name'] = name
    return SceneSpec.from_dict(data)


def load_scenes(raw):
    """Scenes from JSON bytes / text: a list of scene dicts or {"scenes": [...]}. Names must be unique."""
    if isinstance(raw, bytes):
        raw = raw.decode('utf-8')
    doc = json.loads(raw)
    items = doc.get('scenes') if isinstance(doc, dict) else doc
    if not isinstance(items, list) or not items:
        raise _err('scene_file_needs_a_list')
    scenes = [SceneSpec.from_dict(item) for item in items]
    check_unique_names(scenes)
    return scenes


def check_unique_names(scenes):
    names = [s.name for s in scenes]
    if len(set(names)) != len(names):
        raise _err('scene_names_repeated', ', '.join(sorted({n for n in names if names.count(n) > 1})))
    return scenes


def dump_scenes(scenes):
    """Canonical JSON bytes of a scene list (load_scenes reads it back to equal scenes)."""
    return canonical_json(dict(schema=SCHEMA + '.list', scenes=[s.to_dict() for s in scenes]))
