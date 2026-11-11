"""Field world specification (design 5.2, 8.3 P7): one spec -> a raster copy and an SDF (build_field_world).

Fields (canonical JSON: every field written with defaults filled in, sorted keys, compact, ASCII, no NaN):

  schema        'ubts.steer_field_world.v1'
  name          lower-case name (also the SDF model prefix)
  source        'catalogue' (a 10/10 corridor direction of the single-row catalogue) or 'procedural' (W3, the
                statistics-matched orchard of steering/procedural_orchard.py)
  record, corridor_id, direction    the catalogue corridor direction (source catalogue; null otherwise)
  procedural    the procedural orchard parameters (source procedural; normalised by procedural_orchard)
  modifiers     raster_corridor modifier dicts (gap, remove_trunks, branch_rect, branch_blob, obstacle_box, wall), in
                the corridor travel frame, applied to the raster copy exactly as the F-SIM does
  mode          'R' (raster-equivalent: every reflector cell a box) or 'T' (tree shapes)
  crop          {'kind': 'fsim'}: the F-SIM raster crop (corridor polyline box + 35 m, edge cells unobserved);
                {'kind': 'band', 'half_width_m', 'before_m', 'after_m'}: cells outside the corridor band become
                UNOBSERVED in the raster copy (rays end there in both worlds) and get no geometry
  box_z_m       [bottom, top] of the scan-plane cell boxes (mode R; mode T canopy and the scan-plane copy of trunks)
  collision     'none' or 'hard': with 'hard' the mode T trunk cylinders, structures and the added hard cells get
                collision geometry (mode R adds collision to added hard cells only); default R 'none', T 'hard'
  trunk_cells   mode T: also keep the cell boxes of trunk components in the scan plane (the 0.6 m plane then shows
                the same cells as mode R); default true
  upper_canopy  mode T: SIM_ASSUMED canopy band z 1.0-2.8 m along the corridor's two rows (visual only)
  ground        {'mu': null (the profile ground_mu), 'margin_m': 10.0, 'thickness_m': 0.2}
  start         {'s_m', 'd_m', 'heading_deg'}: rover spawn of the Gazebo scene in the corridor frame (field1010)
  allow_sealed  false: the sealed directions and the reverse directions of sealed row pairs are refused
                (steering.scene_spec.sealed_status, single-row amendment A1 / design 7.3)
  allow_holdout false: a procedural spec with a holdout seed (procedural_orchard.HOLDOUT_SEEDS) is refused; written
                in the canonical form only when true (development specs keep their SHA-256; review RW-06)

Coordinates: the SDF uses the world frame W (origin = corridor start s 0, d 0; +x = start tangent); the manifest
holds only the raster-local -> W transform (raster-local metres). No GNSS coordinates, no map YAML origin.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import re

SCHEMA = 'ubts.steer_field_world.v1'
NAME_PATTERN = re.compile(r'[a-z0-9][a-z0-9_.-]{0,63}\Z')
SOURCES = ('catalogue', 'procedural')
MODES = ('R', 'T')
CROPS = ('fsim', 'band')
COLLISIONS = ('none', 'hard')
DIRECTIONS = ('fwd', 'rev')
MODIFIER_KINDS = ('gap', 'remove_trunks', 'branch_rect', 'branch_blob', 'obstacle_box', 'wall')
DEFAULT_BOX_Z_M = (.3, .9)
DEFAULT_BAND = dict(half_width_m=12., before_m=8., after_m=15.)
DEFAULT_GROUND = dict(mu=None, margin_m=10., thickness_m=.2)
DEFAULT_START = dict(s_m=1., d_m=0., heading_deg=0.)


class FieldSpecError(ValueError):
    """A field world specification is not valid."""


def _err(code, detail=''):
    return FieldSpecError(code + (': ' + detail if detail else ''))


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _err('field_value_not_a_number', name)
    value = float(value)
    if not math.isfinite(value):
        raise _err('field_value_not_finite', name)
    return value


def _keys(data, allowed, where):
    if not isinstance(data, dict):
        raise _err('field_not_a_mapping', where)
    unknown = sorted(str(k) for k in set(data) - set(allowed))
    if unknown:
        raise _err('field_key_unknown', '%s: %s' % (where, ', '.join(unknown)))


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True, allow_nan=False).encode('ascii')


def sha256_of(value):
    return hashlib.sha256(canonical_json(value)).hexdigest()


def sealed_status(record, corridor_id, direction):
    """The steering scene rule (one place): 'sealed', 'sealed_pair_reverse' or None."""
    from steering import scene_spec as steer_scene
    return steer_scene.sealed_status(record, corridor_id, direction)


def _modifiers(items):
    if items is None:
        return []
    if not isinstance(items, (list, tuple)):
        raise _err('field_modifiers_not_a_list')
    out = []
    for item in items:
        if not isinstance(item, dict) or item.get('kind') not in MODIFIER_KINDS:
            raise _err('field_modifier_kind_unknown', repr(item))
        try:
            text = json.dumps(item, sort_keys=True, allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise _err('field_modifier_not_json', str(exc))
        out.append(json.loads(text))
    return out


def normalise(data):
    """Validated canonical dict of a field world spec (defaults filled in). Raises FieldSpecError."""
    _keys(data, ('schema', 'name', 'source', 'record', 'corridor_id', 'direction', 'procedural', 'modifiers', 'mode',
                 'crop', 'box_z_m', 'collision', 'trunk_cells', 'upper_canopy', 'ground', 'start', 'allow_sealed',
                 'allow_holdout'), 'field_world')
    schema = data.get('schema', SCHEMA)
    if schema != SCHEMA:
        raise _err('field_schema_mismatch', '%r (need %r)' % (schema, SCHEMA))
    name = data.get('name')
    if not (isinstance(name, str) and NAME_PATTERN.match(name)):
        raise _err('field_name_invalid', repr(name))
    source = data.get('source', 'catalogue')
    if source not in SOURCES:
        raise _err('field_source_unknown', repr(source))
    allow_sealed = data.get('allow_sealed', False)
    if not isinstance(allow_sealed, bool):
        raise _err('field_allow_sealed_not_bool')
    allow_holdout = data.get('allow_holdout', False)
    if not isinstance(allow_holdout, bool):
        raise _err('field_allow_holdout_not_bool')
    if allow_holdout and source != 'procedural':
        raise _err('field_allow_holdout_only_for_procedural')
    record = corridor = None
    direction = data.get('direction', 'fwd' if source == 'catalogue' else None)
    procedural = None
    if source == 'catalogue':
        record, corridor = data.get('record'), data.get('corridor_id')
        if not (isinstance(record, str) and re.match(r'[0-9]{6}\Z', record)):
            raise _err('field_record_invalid', repr(record))
        if not (isinstance(corridor, str) and re.match(r'c[0-9]{2}\Z', corridor)):
            raise _err('field_corridor_id_invalid', repr(corridor))
        if direction not in DIRECTIONS:
            raise _err('field_direction_unknown', repr(direction))
        status = sealed_status(record, corridor, direction)
        if status is not None and not allow_sealed:
            raise _err('field_sealed_corridor', '%s %s %s is %s; refused before the steering pre-registration'
                       % (record, corridor, direction, status))
        if data.get('procedural') is not None:
            raise _err('field_catalogue_takes_no_procedural_params')
    else:
        if any(data.get(k) is not None for k in ('record', 'corridor_id')) or direction not in (None, 'fwd'):
            raise _err('field_procedural_takes_no_catalogue_keys')
        direction = None
        from steering import procedural_orchard
        try:
            procedural = procedural_orchard.normalise_params(data.get('procedural'))
        except ValueError as exc:
            raise _err('field_procedural_params_invalid', str(exc))
        if allow_sealed:
            raise _err('field_procedural_has_no_sealed_corridor')
        if procedural_orchard.is_holdout_seed(procedural['seed']) and not allow_holdout:
            raise _err('field_procedural_holdout_seed', 'seed %d is a holdout seed (%d-%d); set allow_holdout only for '
                       'the pre-registered evaluation' % ((procedural['seed'],) + procedural_orchard.HOLDOUT_SEEDS))
    mode = data.get('mode', 'R')
    if mode not in MODES:
        raise _err('field_mode_unknown', repr(mode))
    crop = data.get('crop') or {'kind': 'fsim'}
    _keys(crop, ('kind', 'half_width_m', 'before_m', 'after_m'), 'crop')
    if crop.get('kind') not in CROPS:
        raise _err('field_crop_unknown', repr(crop.get('kind')))
    if crop['kind'] == 'fsim':
        if set(crop) != {'kind'}:
            raise _err('field_crop_fsim_takes_no_band_values')
        crop = {'kind': 'fsim'}
    else:
        band = {k: _number(crop.get(k, v), 'crop.' + k) for k, v in DEFAULT_BAND.items()}
        if not band['half_width_m'] > 0 or band['before_m'] < 0 or band['after_m'] < 0:
            raise _err('field_crop_band_values')
        crop = dict(kind='band', **band)
    box = data.get('box_z_m', list(DEFAULT_BOX_Z_M))
    if not (isinstance(box, (list, tuple)) and len(box) == 2):
        raise _err('field_box_z_needs_two_values')
    box = [_number(box[0], 'box_z_m'), _number(box[1], 'box_z_m')]
    if not 0. <= box[0] < box[1] <= 5.:
        raise _err('field_box_z_range')
    collision = data.get('collision', 'none' if mode == 'R' else 'hard')
    if collision not in COLLISIONS:
        raise _err('field_collision_unknown', repr(collision))
    flags = {}
    for key, default in (('trunk_cells', True), ('upper_canopy', False)):
        value = data.get(key)
        flags[key] = default if value is None else value          # None: the default (the canonical form of mode R)
        if not isinstance(flags[key], bool):
            raise _err('field_flag_not_bool', key)
    if mode == 'R' and (data.get('upper_canopy') is not None or data.get('trunk_cells') is not None):
        raise _err('field_mode_r_takes_no_tree_options')
    ground = dict(DEFAULT_GROUND)
    given = data.get('ground') or {}
    _keys(given, DEFAULT_GROUND, 'ground')
    ground.update(given)
    if ground['mu'] is not None:
        ground['mu'] = _number(ground['mu'], 'ground.mu')
        if not 0. < ground['mu'] <= 2.:
            raise _err('field_ground_mu_range')
    ground['margin_m'] = _number(ground['margin_m'], 'ground.margin_m')
    ground['thickness_m'] = _number(ground['thickness_m'], 'ground.thickness_m')
    if ground['margin_m'] < 0 or not ground['thickness_m'] > 0:
        raise _err('field_ground_values')
    start = dict(DEFAULT_START)
    given = data.get('start') or {}
    _keys(given, DEFAULT_START, 'start')
    start.update({k: _number(v, 'start.' + k) for k, v in given.items()})
    if abs(start['heading_deg']) > 180.:
        raise _err('field_start_heading_range')
    out = dict(schema=SCHEMA, name=name, source=source, record=record, corridor_id=corridor, direction=direction,
               procedural=procedural, modifiers=_modifiers(data.get('modifiers')), mode=mode, crop=crop,
               box_z_m=box, collision=collision, trunk_cells=flags['trunk_cells'] if mode == 'T' else None,
               upper_canopy=flags['upper_canopy'] if mode == 'T' else None, ground=ground, start=start,
               allow_sealed=allow_sealed)
    if allow_holdout:
        out['allow_holdout'] = True
    return out


class FieldWorldSpec:
    """A validated field world spec (immutable view of its canonical dict)."""

    def __init__(self, data):
        self._data = normalise(data)

    @classmethod
    def from_dict(cls, data):
        return cls(data)

    @classmethod
    def from_json(cls, raw):
        if isinstance(raw, bytes):
            raw = raw.decode('utf-8')
        return cls(json.loads(raw))

    def to_dict(self):
        return copy.deepcopy(self._data)

    def canonical_json(self):
        return canonical_json(self._data)

    def sha256(self):
        return hashlib.sha256(self.canonical_json()).hexdigest()

    def __getattr__(self, name):
        data = self.__dict__.get('_data')
        if data is not None and name in data:
            value = data[name]
            return copy.deepcopy(value) if isinstance(value, (dict, list)) else value
        raise AttributeError(name)

    def with_changes(self, **changes):
        data = self.to_dict()
        data.update(changes)
        if 'mode' in changes and changes['mode'] == 'R':
            data.pop('trunk_cells', None)
            data.pop('upper_canopy', None)
        if 'mode' in changes and 'collision' not in changes:
            data.pop('collision', None)
        for key in ('trunk_cells', 'upper_canopy'):
            if data.get(key) is None:
                data.pop(key, None)
        return FieldWorldSpec(data)


def catalogue_spec(record, corridor_id, direction='fwd', *, mode='R', name=None, **fields):
    """Short form for a catalogue field world."""
    data = dict(fields, source='catalogue', record=record, corridor_id=corridor_id, direction=direction, mode=mode)
    data['name'] = name or ('f%s_%s_%s_%s' % (record, corridor_id, direction, mode.lower()))
    return FieldWorldSpec(data)


def development_directions(catalogue):
    """[(record, corridor_id, direction, role)] of the development set (frozen 8 + other included directions, no
    sealed direction and no reverse of a sealed pair): steering.entry_feasibility.split_directions (one rule)."""
    from steering import entry_feasibility
    development, _ = entry_feasibility.split_directions(catalogue)
    return [(d['record'], d['corridor_id'], d['direction'], d['role']) for d in development]
