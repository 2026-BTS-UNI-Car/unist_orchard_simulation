"""Build a field world: raster copy + SDF + manifest + eval_truth from one spec (design 5.2, 8.3 P7).

build(spec) -> FieldWorld (in memory); write(fw, out_dir) writes:

  raster.npz        the raster copy: RASTER_V1 classes of the crop (uint8, row 0 = lowest y), resolution, crop
                    offsets, the raster-local -> W transform; the sensing and contact model of the F-SIM and the
                    raster of the G-SIM clip node (steering/scan_unobserved_clip.py). SIM truth, plant side.
  field_world.sdf   a standalone Gazebo world (ground + field models, no rover) in the world frame W
  manifest.json     spec and its SHA-256, input SHA-256 (catalogue, map PGM / YAML), generator file SHA-256, the
                    transform (raster-local metres only), counts (cells, boxes, triangles, SDF bytes), the size limits
                    (limits_report), output SHA-256
  eval_truth.json   evaluation only (never a controller / estimator input): centreline, row lines, row end, line
                    ends, support units, cross structures, walls - catalogue (SLAM) labels or the procedural truth

World frame W: origin = the corridor start (s 0, d 0), +x = the start tangent, +y left, z up. Raster-local metres L
(the record raster's lower-left corner) appear only in the manifest transform and in raster.npz.

Mode R (raster-equivalent, the LiDAR equivalence reference of A7): every reflector cell (OCCUPIED, ADDED_HARD,
ADDED_BRANCH) of the raster copy becomes part of a box z box_z_m (default 0.3-0.9 m); cells are merged into disjoint
rectangles per class (raster_ops.merged_rectangles), all boxes share the yaw of L in W. Collision: none (default; the
raster evaluator is the contact reference) or hard (ADDED_HARD only).
Mode T (tree shapes): OCCUPIED components (8-connected after a 1-cell dilation) by size: diagonal <= 0.30 m trunk
candidate (cylinder at the centroid, r = clamp((extent - 0.05) / 2, 0.03, 0.12), z 0-0.9 m, collision with 'hard';
plus its cells as scan-plane boxes when trunk_cells), 0.30-1.2 m canopy section (cell boxes z box_z_m, no collision),
> 1.2 m structure (cell boxes z 0-1.5 m, collision with 'hard'); ADDED_HARD cells z 0-1.5 m (collision with 'hard'),
ADDED_BRANCH cells z 0.4-0.9 m (visual); optional upper canopy band (SIM_ASSUMED, z 1.0-2.8 m, visual) along the
corridor's two rows. Heights and the trunk radius rule are SIM_ASSUMED (the map has no height).

Limits (design 8.3 P7: box count, SDF size, load time, time to the first scan; review RW-03, 2026-11-09): LIMITS
per mode for the sizes (checked by write(): a world over a limit is refused unless allow_over_limits, and the
manifest carries the report) and TIME_LIMITS for the Embree load and first scan (a7_equivalence compares each world)
and the Gazebo start (gz_probe rtf, P6a). The values were set after the first P7 measurement (32 development
directions, SIZE_TABLE.json and a7_result_R/T.json of runs/wf3b/p7), which is their basis and not a judgement:
mode R boxes and triangles = the design 5.2 whole-map figures (10,359 boxes, about 124k triangles), the SDF bytes
from that box count (about 365 B a box measured); mode T triangles = the original detailed orchard lite that the
G-SIM W1 already runs (768k); times with a wide margin over the measured maxima (load 12.1 s, first scan 0.58 s,
Gazebo first statistics 4.4 s) and below the supervisor's readiness limits.

The 10/10 maps are SLAM reconstructions: a world made from them is 'the orchard as SLAM saw it'. Placeholder-based
results are placeholder-based SIM results, not field validation.
CLI: python -B -m field_world.build_field_world --spec <spec.json> --out <folder>   (sim/ubts on sys.path)
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as E
import zipfile

import numpy as np

from . import raster_ops as ro
from .scene_spec import FieldWorldSpec

MANIFEST_SCHEMA = 'ubts.steer_field_world.manifest.v1'
TRUTH_SCHEMA = 'ubts.steer_field_world.eval_truth.v1'
RASTER_SCHEMA = 'ubts.steer_field_world.raster.v1'
RESULT_KIND = '자리값 기반 SIM 결과'
UBTS_DIR = Path(__file__).resolve().parents[1]                  # sim/ubts
GENERATOR_FILES = ('field_world/__init__.py', 'field_world/scene_spec.py', 'field_world/raster_ops.py',
                   'field_world/build_field_world.py', 'closed_loop/raster_corridor.py', 'steering/scene_spec.py',
                   'steering/procedural_orchard.py')
# Mode T shape rules (SIM_ASSUMED: the 2D map has no height; design 5.2 and the world survey 5.4).
TRUNK_DIAG_MAX_M = .30
CANOPY_DIAG_MAX_M = 1.2
TRUNK_HEIGHT_M = .9
TRUNK_RADIUS_RANGE_M = (.03, .12)
STRUCTURE_Z_M = (0., 1.5)
ADDED_HARD_Z_M = (0., 1.5)
ADDED_BRANCH_Z_M = (.4, .9)
UPPER_CANOPY_Z_M = (1.0, 2.8)
UPPER_CANOPY_WIDTH_M = 1.2
CYLINDER_TRIANGLES = 384                 # mesh_raycast: trimesh cylinder with 96 sections
BOX_TRIANGLES = 12
COLOURS = {'occupied': '.20 .32 .14 1', 'added_hard': '.62 .62 .62 1', 'added_branch': '.25 .55 .20 1',
           'trunk': '.36 .22 .10 1', 'canopy': '.18 .42 .12 1', 'structure': '.55 .52 .48 1',
           'upper_canopy': '.15 .38 .10 .6', 'ground': '.33 .36 .22 1'}
SDF_DECIMALS = 5                         # positions in W: 10 um (coord_lint: lat/lon-like number pairs need >= 6)
MAX_CROP_CELLS = 4096 * 4096             # the raster copy (16 MiB of classes; procedural_orchard.MAX_CROP_CELLS)
LIMITS = {
    'R': dict(boxes=10359, cylinders=0, triangles_estimate=124320, sdf_bytes=4000000, collision_shapes=10359,
              crop_cells=MAX_CROP_CELLS),
    'T': dict(boxes=10359, cylinders=2000, triangles_estimate=768000, sdf_bytes=8000000, collision_shapes=12359,
              crop_cells=MAX_CROP_CELLS),
}
TIME_LIMITS = dict(embree_load_s=30., embree_first_scan_s=1., time_to_first_scan_s=30., gz_first_stats_s=60.)
LIMITS_BASIS = ('set 2026-11-09 after the first P7 measurement (basis, not a judgement): design 5.2 whole-map boxes '
                'and triangles (mode R), the G-SIM W1 original lite triangles (mode T), times with a wide margin over '
                'the measured maxima')


class FieldWorldError(ValueError):
    """The field world cannot be built from this spec."""


def _sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def generator_identity():
    files = {}
    for rel in GENERATOR_FILES:
        path = UBTS_DIR / rel
        files[rel] = _sha256_file(path) if path.is_file() else None
    bundle = hashlib.sha256(''.join('%s %s\n' % (v, k) for k, v in sorted(files.items())).encode('utf-8')).hexdigest()
    return dict(files=files, bundle_sha256=bundle)


# --- the in-memory world --------------------------------------------------------------------------------------------
class FieldWorld:
    """Built field world (see the module docstring). Attributes: spec, classes, resolution, row0, col0, transform,
    frame (CurvilinearFrame in raster-local metres), config (closed_loop RasterCorridorConfig), sim_world (the F-SIM
    raster world of the same spec; its classes equal self.classes for crop fsim), models (SDF model elements),
    ground (SDF model element), parts (geometry rows), eval_truth, source_info."""

    def __init__(self, spec, classes, resolution, row0, col0, transform, frame, config, sim_world, source_info,
                 truth, profile_ground_mu):
        self.spec = spec
        self.classes = classes
        self.resolution = float(resolution)
        self.row0, self.col0 = int(row0), int(col0)
        self.transform = transform
        self.frame = frame
        self.config = config
        self.sim_world = sim_world
        self.source_info = source_info
        self.eval_truth = truth
        self.ground_mu = profile_ground_mu if spec.ground['mu'] is None else spec.ground['mu']
        self.parts = []                  # dict(kind, shape, centre_w, size or radius, z, collision, cells)
        self.models = []
        self.ground = None

    @property
    def shape(self):
        return self.classes.shape

    def cell_centres_w(self, rows, cols):
        x = (np.asarray(cols) + self.col0 + .5) * self.resolution
        y = (np.asarray(rows) + self.row0 + .5) * self.resolution
        return self.transform.to_w(x, y)

    def class_counts(self):
        from closed_loop import raster_corridor as RC
        counts = np.bincount(self.classes.ravel(), minlength=len(RC.CLASS_NAMES))
        return {name: int(counts[i]) for i, name in enumerate(RC.CLASS_NAMES)}


def _catalogue_source(spec, catalogue):
    from closed_loop import raster_corridor as RC
    catalogue = catalogue if isinstance(catalogue, dict) else RC.load_catalogue(catalogue)
    modifiers = tuple(RC.modifier_from_dict(m) for m in spec.modifiers)
    config = RC.corridor_world_config(spec.record, spec.corridor_id, spec.direction, modifiers=modifiers,
                                      catalogue=catalogue)
    sim_world = RC.build_raster_world(config)
    rmap = sim_world.map
    info = dict(kind='catalogue', record=spec.record, corridor_id=spec.corridor_id, direction=spec.direction,
                catalogue_sha256=catalogue.get('_sha256'), pgm_sha256=rmap.info['pgm_sha256'],
                yaml_sha256=rmap.info['yaml_sha256'], occupied_thresh=rmap.info['occupied_thresh'],
                free_thresh=rmap.info['free_thresh'], sensing_model=RC.SENSING_MODEL, map_source=RC.SOURCE,
                modifier_log=sim_world.modifier_log)
    return config, sim_world, info


def _procedural_source(spec):
    from steering import procedural_orchard as po
    config, sim_world, info = po.build_raster_world(spec.procedural, modifiers=spec.modifiers,
                                                    allow_holdout=bool(spec.to_dict().get('allow_holdout')))
    return config, sim_world, info


def _band_mask(fw, crop):
    """Cells of the crop whose centre lies in the corridor band (|d| <= half width, -before <= s <= s_E + after)."""
    frame, s_end = fw.frame, float(fw.config.end_s_m)
    s = np.arange(-crop['before_m'], s_end + crop['after_m'] + .25, .25)
    left = np.column_stack(frame.point_many(s, np.full(len(s), crop['half_width_m'])))
    right = np.column_stack(frame.point_many(s, np.full(len(s), -crop['half_width_m'])))
    polygon = np.concatenate((right, left[::-1]))
    return ro.polygon_mask(fw.classes.shape, fw.resolution, fw.row0, fw.col0, polygon)


def eval_truth_catalogue(config, frame, transform, s_end, walls_s):
    """Evaluation-only corridor labels in W (catalogue = SLAM labels)."""
    prof_s = np.asarray(config.profile_s_m, dtype=float)

    def to_w_points(xs, ys):
        wx, wy = transform.to_w(xs, ys)
        return np.column_stack((wx, wy))
    centre = to_w_points(frame.points[:, 0], frame.points[:, 1])
    rows = {}
    for side, values in (('left', config.left_row_d_m), ('right', config.right_row_d_m)):
        x, y = frame.point_many(prof_s, np.asarray(values, dtype=float))
        rows[side] = dict(s_m=prof_s, line_w=to_w_points(x, y))
    end_x, end_y = frame.point(s_end, 0.)
    units = {}
    for side, items in (('left', config.left_units), ('right', config.right_units)):
        units[side] = [[float(s), float(d)] for s, d in items]
    return ro.rounded(dict(
        schema=TRUTH_SCHEMA, use='evaluation only; never an executor, estimator or controller input',
        source='single-row corridor catalogue labels of a SLAM reconstruction (not physical truth)',
        frame='W: origin = corridor start (s 0, d 0), +x = start tangent, +y left; corridor frame (s, d) along the '
              'labelled centreline', centreline=dict(vertex_s_m=frame.s_vertex, points_w=centre),
        s_start_m=0., s_end_m=s_end, end_point_w=list(to_w_points([end_x], [end_y])[0]),
        line_end_s_m=list(config.line_end_s_m), row_lines=rows, support_units_s_d=units,
        row_spacing_median_m=config.row_width_m, row_band_m=config.band_m, walls_near_face_s_m=list(walls_s),
        cross_structures_s_m=None), 4)


def build(spec, *, catalogue=None, profile=None):
    """FieldWorld of a spec (FieldWorldSpec or its dict). catalogue: dict or path (default: the configured one)."""
    from closed_loop import raster_corridor as RC
    spec = spec if isinstance(spec, FieldWorldSpec) else FieldWorldSpec(spec)
    if spec.source == 'catalogue':
        config, sim_world, info = _catalogue_source(spec, catalogue)
    else:
        config, sim_world, info = _procedural_source(spec)
    frame = sim_world.frame
    rmap = sim_world.map
    x0, y0, theta = frame.pose(0., 0., 0.)
    transform = ro.Transform2D(x0, y0, theta)
    if profile is None:
        from steering.vehicle_profile import load_profile
        profile = load_profile()
    truth = getattr(sim_world, 'eval_truth', None)
    if truth is None:
        truth = eval_truth_catalogue(config, frame, transform, float(config.end_s_m), sim_world.walls_s)
    fw = FieldWorld(spec, rmap.classes, rmap.resolution, rmap.row0, rmap.col0, transform, frame, config, sim_world,
                    info, truth, float(profile.value('ground_mu')))
    crop = spec.crop
    if crop['kind'] == 'band':
        classes = fw.classes.copy()
        classes[~_band_mask(fw, crop)] = RC.UNOBSERVED
        classes.flags.writeable = False
        fw.classes = classes
        fw.source_info['fsim_equivalent'] = False
    else:
        fw.source_info['fsim_equivalent'] = True
    if spec.mode == 'R':
        _mode_r(fw)
    else:
        _mode_t(fw)
    fw.ground = _ground_model(fw)
    fw.models = _models(fw)
    return fw


# --- geometry ----------------------------------------------------------------------------------------------------------
def _rect_parts(fw, mask, kind, z, collision):
    rects = ro.merged_rectangles(mask)
    res = fw.resolution
    yaw = ro.wrap(-fw.transform.theta)
    out = []
    for r0, r1, c0, c1 in rects:
        cx = ((c0 + c1 + 1) / 2. + fw.col0) * res
        cy = ((r0 + r1 + 1) / 2. + fw.row0) * res
        wx, wy = fw.transform.to_w(cx, cy)
        out.append(dict(kind=kind, shape='box', centre_w=(float(wx), float(wy)), yaw=float(yaw),
                        size=((c1 - c0 + 1) * res, (r1 - r0 + 1) * res), z=tuple(z), collision=bool(collision),
                        cells=int((r1 - r0 + 1) * (c1 - c0 + 1)), rect=(int(r0), int(r1), int(c0), int(c1))))
    return out


def _mode_r(fw):
    from closed_loop import raster_corridor as RC
    spec = fw.spec
    z = spec.box_z_m
    for cls, kind in ((RC.OCCUPIED, 'occupied'), (RC.ADDED_HARD, 'added_hard'), (RC.ADDED_BRANCH, 'added_branch')):
        mask = fw.classes == cls
        if mask.any():
            fw.parts += _rect_parts(fw, mask, kind, z, spec.collision == 'hard' and kind == 'added_hard')


def _mode_t(fw):
    from closed_loop import raster_corridor as RC
    spec = fw.spec
    hard = spec.collision == 'hard'
    rows, cols = np.nonzero(fw.classes == RC.OCCUPIED)
    labels = ro.label_components(rows, cols)
    table = ro.component_table(rows, cols, labels, fw.resolution, fw.row0, fw.col0)
    masks = {k: np.zeros(fw.classes.shape, dtype=bool) for k in ('trunk', 'canopy', 'structure')}
    lo, hi = TRUNK_RADIUS_RANGE_M
    trunks = []
    for comp in table:
        if comp['diag_m'] <= TRUNK_DIAG_MAX_M + 1e-9:
            kind = 'trunk'
        elif comp['diag_m'] <= CANOPY_DIAG_MAX_M + 1e-9:
            kind = 'canopy'
        else:
            kind = 'structure'
        masks[kind][comp['rows'], comp['cols']] = True
        if kind == 'trunk':
            radius = min(hi, max(lo, (comp['extent_m'] - .05) / 2.))
            wx, wy = fw.transform.to_w(comp['cx'], comp['cy'])
            trunks.append(dict(kind='trunk', shape='cylinder', centre_w=(float(wx), float(wy)), yaw=0.,
                               radius=float(radius), z=(0., TRUNK_HEIGHT_M), collision=hard, cells=comp['cells']))
    fw.parts += trunks
    if spec.trunk_cells and masks['trunk'].any():
        fw.parts += _rect_parts(fw, masks['trunk'], 'trunk_cells', spec.box_z_m, False)
    if masks['canopy'].any():
        fw.parts += _rect_parts(fw, masks['canopy'], 'canopy', spec.box_z_m, False)
    if masks['structure'].any():
        fw.parts += _rect_parts(fw, masks['structure'], 'structure', STRUCTURE_Z_M, hard)
    for cls, kind, z, coll in ((RC.ADDED_HARD, 'added_hard', ADDED_HARD_Z_M, hard),
                               (RC.ADDED_BRANCH, 'added_branch', ADDED_BRANCH_Z_M, False)):
        mask = fw.classes == cls
        if mask.any():
            fw.parts += _rect_parts(fw, mask, kind, z, coll)
    if spec.upper_canopy:
        fw.parts += _upper_canopy(fw)
    fw.component_summary = dict(components=len(table), trunk=sum(1 for p in trunks),
                                canopy_cells=int(masks['canopy'].sum()), structure_cells=int(masks['structure'].sum()),
                                trunk_cells=int(masks['trunk'].sum()))


def _upper_canopy(fw):
    """SIM_ASSUMED canopy band boxes along the corridor's two rows, one per 1 m profile window."""
    config, frame = fw.config, fw.frame
    out = []
    prof_s = np.asarray(config.profile_s_m, dtype=float)
    for side, values, units in (('left', config.left_row_d_m, config.left_units),
                                ('right', config.right_row_d_m, config.right_units)):
        if not units:
            continue
        s_lo, s_hi = min(u[0] for u in units), max(u[0] for u in units)
        for s in np.arange(math.floor(s_lo), s_hi + 1e-9, 1.):
            d = float(np.interp(s + .5, prof_s, np.asarray(values, dtype=float)))
            x, y, heading = frame.pose(s + .5, d, 0.)
            wx, wy = fw.transform.to_w(x, y)
            out.append(dict(kind='upper_canopy', shape='box', centre_w=(float(wx), float(wy)),
                            yaw=float(fw.transform.yaw_to_w(heading)), size=(1.0, UPPER_CANOPY_WIDTH_M),
                            z=UPPER_CANOPY_Z_M, collision=False, cells=0))
    return out


# --- SDF -----------------------------------------------------------------------------------------------------------------
def _num(value):
    return ro.fmt(value, SDF_DECIMALS)


def _pose(x, y, z, yaw=0.):
    return '%s %s %s 0 0 %s' % (_num(x), _num(y), _num(z), ro.fmt(yaw, 9))


def _geometry(parent, part):
    geometry = E.SubElement(parent, 'geometry')
    if part['shape'] == 'box':
        box = E.SubElement(geometry, 'box')
        E.SubElement(box, 'size').text = '%s %s %s' % (_num(part['size'][0]), _num(part['size'][1]),
                                                        _num(part['z'][1] - part['z'][0]))
    else:
        cylinder = E.SubElement(geometry, 'cylinder')
        E.SubElement(cylinder, 'radius').text = _num(part['radius'])
        E.SubElement(cylinder, 'length').text = _num(part['z'][1] - part['z'][0])


def _models(fw):
    groups = {}
    for part in fw.parts:
        groups.setdefault(part['kind'], []).append(part)
    model = E.Element('model', name='field_%s' % fw.spec.name.replace('.', '_'))
    E.SubElement(model, 'static').text = 'true'
    for kind in sorted(groups):
        link = E.SubElement(model, 'link', name=kind)
        colour = COLOURS['occupied' if kind == 'trunk_cells' else kind]
        for index, part in enumerate(groups[kind]):
            zc = .5 * (part['z'][0] + part['z'][1])
            pose = _pose(part['centre_w'][0], part['centre_w'][1], zc, part['yaw'])
            visual = E.SubElement(link, 'visual', name='v%d' % index)
            E.SubElement(visual, 'pose').text = pose
            _geometry(visual, part)
            material = E.SubElement(visual, 'material')
            E.SubElement(material, 'ambient').text = colour
            E.SubElement(material, 'diffuse').text = colour
            if part['collision']:
                collision = E.SubElement(link, 'collision', name='c%d' % index)
                E.SubElement(collision, 'pose').text = pose
                _geometry(collision, part)
    return [model]


def ground_extent_w(fw):
    """(xmin, ymin, xmax, ymax) in W of the crop corners plus the ground margin."""
    res = fw.resolution
    h, w = fw.classes.shape
    xs = np.array([fw.col0, fw.col0 + w, fw.col0, fw.col0 + w], dtype=float) * res
    ys = np.array([fw.row0, fw.row0, fw.row0 + h, fw.row0 + h], dtype=float) * res
    wx, wy = fw.transform.to_w(xs, ys)
    m = fw.spec.ground['margin_m']
    return float(wx.min() - m), float(wy.min() - m), float(wx.max() + m), float(wy.max() + m)


def _ground_model(fw):
    x0, y0, x1, y1 = ground_extent_w(fw)
    t = fw.spec.ground['thickness_m']
    model = E.Element('model', name='field_ground')
    E.SubElement(model, 'static').text = 'true'
    link = E.SubElement(model, 'link', name='ground')
    E.SubElement(link, 'pose').text = _pose(.5 * (x0 + x1), .5 * (y0 + y1), -t / 2.)
    part = dict(shape='box', size=(x1 - x0, y1 - y0), z=(0., t))
    visual = E.SubElement(link, 'visual', name='ground')
    _geometry(visual, part)
    material = E.SubElement(visual, 'material')
    E.SubElement(material, 'ambient').text = COLOURS['ground']
    E.SubElement(material, 'diffuse').text = COLOURS['ground']
    collision = E.SubElement(link, 'collision', name='ground_collision')
    _geometry(collision, part)
    ode = E.SubElement(E.SubElement(E.SubElement(collision, 'surface'), 'friction'), 'ode')
    E.SubElement(ode, 'mu').text = _num(fw.ground_mu)
    E.SubElement(ode, 'mu2').text = _num(fw.ground_mu)       # DART takes the smaller of the two surfaces
    fw.ground_extent_w = (x0, y0, x1, y1)
    return model


def world_sdf_text(fw, *, world_name='ubts_field', physics_hz=120.):
    """Standalone SDF world: physics, the basic systems, a sun, the ground and the field models (no rover)."""
    sdf = E.Element('sdf', version='1.9')
    world = E.SubElement(sdf, 'world', name=world_name)
    physics = E.SubElement(world, 'physics', name='field', type='ignored')
    E.SubElement(physics, 'max_step_size').text = repr(1. / physics_hz)
    E.SubElement(physics, 'real_time_factor').text = '1.0'
    E.SubElement(physics, 'real_time_update_rate').text = repr(float(physics_hz))
    E.SubElement(world, 'gravity').text = '0 0 -9.80665'
    for short in ('Physics', 'UserCommands', 'SceneBroadcaster'):
        file_name = ''.join(('-' + c.lower() if c.isupper() else c) for c in short).lstrip('-')
        E.SubElement(world, 'plugin', filename='gz-sim-%s-system' % file_name, name='gz::sim::systems::%s' % short)
    scene = E.SubElement(world, 'scene')
    E.SubElement(scene, 'ambient').text = '.55 .55 .55 1'
    E.SubElement(scene, 'background').text = '.55 .73 .90 1'
    E.SubElement(scene, 'shadows').text = 'false'
    sun = E.SubElement(world, 'light', name='sun', type='directional')
    E.SubElement(sun, 'pose').text = '0 0 10 0 0 0'
    E.SubElement(sun, 'direction').text = '-.5 .1 -1'
    E.SubElement(sun, 'diffuse').text = '.9 .9 .9 1'
    E.SubElement(sun, 'cast_shadows').text = 'false'
    world.append(fw.ground)
    for model in fw.models:
        world.append(model)
    E.indent(sdf)
    return E.tostring(sdf, encoding='unicode')


# --- counts, limits, files ---------------------------------------------------------------------------------------------
def limits_report(counts_value, mode):
    """dict(within, items {name: [value, limit, ok]}, basis) of a world's counts against LIMITS[mode]."""
    items = {}
    for name, limit in LIMITS[mode].items():
        value = counts_value.get(name)
        items[name] = [value, limit, value is None or value <= limit]
    return dict(within=all(ok for _, _, ok in items.values()), items=items, basis=LIMITS_BASIS, mode=mode)


def time_limits_report(values):
    """dict(within, items) of measured times (keys of TIME_LIMITS; missing keys are skipped)."""
    items = {name: [values[name], limit, values[name] <= limit] for name, limit in TIME_LIMITS.items()
             if values.get(name) is not None}
    return dict(within=all(ok for _, _, ok in items.values()), items=items, basis=LIMITS_BASIS)


def counts(fw, sdf_text=None):
    by_kind = {}
    triangles = 0
    for part in fw.parts:
        row = by_kind.setdefault(part['kind'], dict(parts=0, cells=0, collisions=0))
        row['parts'] += 1
        row['cells'] += part['cells']
        row['collisions'] += int(part['collision'])
        triangles += BOX_TRIANGLES if part['shape'] == 'box' else CYLINDER_TRIANGLES
    boxes = sum(1 for p in fw.parts if p['shape'] == 'box')
    out = dict(class_counts=fw.class_counts(), parts_by_kind=by_kind, boxes=boxes,
               cylinders=sum(1 for p in fw.parts if p['shape'] == 'cylinder'),
               collision_shapes=sum(1 for p in fw.parts if p['collision']), triangles_estimate=triangles + BOX_TRIANGLES,
               triangles_rule='box 12, cylinder 384 (96 sections, mesh_raycast), + the ground box',
               crop_shape=list(fw.classes.shape), crop_cells=int(fw.classes.size))
    if sdf_text is not None:
        out['sdf_bytes'] = len(sdf_text.encode('utf-8'))
    if getattr(fw, 'component_summary', None):
        out['mode_t_components'] = fw.component_summary
    return out


def write_npz(path, arrays):
    """npz readable by numpy.load whose bytes depend only on the arrays (fixed member time, sorted members)."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for name in sorted(arrays):
            data = io.BytesIO()
            np.lib.format.write_array(data, np.asarray(arrays[name], order='C'), allow_pickle=False)
            info = zipfile.ZipInfo(name + '.npy', date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.create_system = 3
            info.external_attr = 0o644 << 16
            archive.writestr(info, data.getvalue())
    Path(path).write_bytes(buffer.getvalue())


def raster_arrays(fw):
    from closed_loop import raster_corridor as RC
    t = fw.transform
    return dict(schema=np.array(RASTER_SCHEMA), classes=np.ascontiguousarray(fw.classes, dtype=np.uint8),
                resolution_m=np.array(fw.resolution), row0=np.array(fw.row0, dtype=np.int64),
                col0=np.array(fw.col0, dtype=np.int64),
                transform_l_from_w=np.array([t.x0, t.y0, t.theta], dtype=np.float64),
                class_names=np.array(RC.CLASS_NAMES), spec_sha256=np.array(fw.spec.sha256()),
                unobserved_class=np.array(RC.UNOBSERVED, dtype=np.int64),
                reflector_classes=np.array(RC.REFLECTORS, dtype=np.int64))


def _json_bytes(value):
    return (json.dumps(value, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False) + '\n').encode('utf-8')


def manifest_dict(fw, sdf_text):
    spec = fw.spec
    info = ro.rounded(fw.source_info, 6)
    return dict(schema=MANIFEST_SCHEMA, name=spec.name, spec=spec.to_dict(), spec_sha256=spec.sha256(),
                mode=spec.mode, source=info, transform=fw.transform.to_dict(),
                crop=dict(spec.crop, row0=fw.row0, col0=fw.col0, shape=list(fw.classes.shape),
                          resolution_m=fw.resolution, edge_cells='unobserved (the F-SIM crop rule)'),
                ground=dict(extent_w_m=[round(v, 4) for v in fw.ground_extent_w], mu=fw.ground_mu,
                            thickness_m=spec.ground['thickness_m']),
                counts=counts(fw, sdf_text), generator=generator_identity(),
                world_frame='W: origin corridor start (s 0, d 0), +x start tangent, +y left, z up',
                coordinates_note='no GNSS coordinates, no map YAML origin; raster-local metres only in the transform',
                result_kind=RESULT_KIND, physical_truth=False, field_validated=False)


def write(fw, out_dir, *, world_name='ubts_field', allow_over_limits=False):
    """Write raster.npz, field_world.sdf, manifest.json and eval_truth.json; returns the manifest (with file SHAs).
    A world over LIMITS is refused (FieldWorldError) unless allow_over_limits (the manifest then keeps the report)."""
    out = Path(out_dir)
    sdf_text = world_sdf_text(fw, world_name=world_name)
    report = limits_report(counts(fw, sdf_text), fw.spec.mode)
    if not report['within'] and not allow_over_limits:
        over = {k: v[:2] for k, v in report['items'].items() if not v[2]}
        raise FieldWorldError('field_world_over_limits: %s (LIMITS[%r]; allow_over_limits to write it anyway)'
                              % (over, fw.spec.mode))
    out.mkdir(parents=True, exist_ok=True)
    (out / 'field_world.sdf').write_bytes(sdf_text.encode('utf-8'))
    write_npz(out / 'raster.npz', raster_arrays(fw))
    (out / 'eval_truth.json').write_bytes(_json_bytes(fw.eval_truth))
    manifest = manifest_dict(fw, sdf_text)
    manifest['limits'] = report
    manifest['files'] = {name: _sha256_file(out / name) for name in ('field_world.sdf', 'raster.npz', 'eval_truth.json')}
    from . import coord_lint
    manifest['coord_lint'] = coord_lint.lint_files([out / n for n in ('field_world.sdf', 'eval_truth.json')]
                                                   + [_ManifestProbe(manifest)])
    (out / 'manifest.json').write_bytes(_json_bytes(manifest))
    return manifest


class _ManifestProbe:
    """The manifest text itself for the lint (before it is written)."""

    def __init__(self, manifest):
        self.name = 'manifest.json'
        self.text = json.dumps(manifest, sort_keys=True, indent=1, ensure_ascii=False)


# --- raster copy loader ---------------------------------------------------------------------------------------------------
class RasterCopy:
    """raster.npz loaded: classes, resolution, row0, col0, transform (Transform2D), spec_sha256."""

    def __init__(self, path):
        def scalar(value):
            return np.asarray(value).reshape(-1)[0].item()
        with np.load(str(path), allow_pickle=False) as data:
            if str(scalar(data['schema'])) != RASTER_SCHEMA:
                raise FieldWorldError('raster_schema_mismatch: %s' % scalar(data['schema']))
            self.classes = data['classes'].copy()
            self.resolution = float(scalar(data['resolution_m']))
            self.row0, self.col0 = int(scalar(data['row0'])), int(scalar(data['col0']))
            x0, y0, theta = (float(v) for v in data['transform_l_from_w'])
            self.spec_sha256 = str(scalar(data['spec_sha256']))
            self.unobserved_class = int(scalar(data['unobserved_class']))
            self.reflector_classes = tuple(int(v) for v in np.asarray(data['reflector_classes']).reshape(-1))
        self.classes.flags.writeable = False
        self.transform = ro.Transform2D(x0, y0, theta)
        self.path = str(path)
        self.sha256 = _sha256_file(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Build a field world (raster copy + SDF) from a spec (design 5.2).')
    parser.add_argument('--spec', type=Path, help='field world spec JSON file')
    parser.add_argument('--record')
    parser.add_argument('--corridor')
    parser.add_argument('--direction', choices=('fwd', 'rev'), default='fwd')
    parser.add_argument('--mode', choices=('R', 'T'), default='R')
    parser.add_argument('--out', type=Path, required=True, help='output folder (outside the repository)')
    args = parser.parse_args(argv)
    from steering import scene_spec as steer_scene
    if steer_scene.inside_repository(args.out):
        parser.error('the output folder is inside the repository (design 4.5)')
    if args.spec:
        spec = FieldWorldSpec.from_json(args.spec.read_bytes())
    else:
        if not (args.record and args.corridor):
            parser.error('--spec or --record and --corridor')
        from .scene_spec import catalogue_spec
        spec = catalogue_spec(args.record, args.corridor, args.direction, mode=args.mode)
    fw = build(spec)
    manifest = write(fw, args.out)
    print(json.dumps(dict(name=manifest['name'], spec_sha256=manifest['spec_sha256'],
                          boxes=manifest['counts']['boxes'], sdf_bytes=manifest['counts']['sdf_bytes'],
                          coord_lint_ok=manifest['coord_lint']['ok'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
