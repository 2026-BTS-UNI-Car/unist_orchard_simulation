"""field_world (design 5.2, 8.3 P7): spec, raster helpers, builder (modes R / T), lint, clip, closure.

Synthetic data and the procedural orchard only (no map needed); tests that need the 10/10 maps and the catalogue
skip with a reason when they are not configured. Plain asserts, no pytest fixtures: the same functions run under
pytest (3.12, 3.13) and under a unittest-style runner on Python 3.10 (decision R10). Temporary files go to the
system temporary folder (never into the work tree).
Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_field_world.py
"""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import tempfile

import numpy as np

from closed_loop import raster_corridor as RC
from field_world import build_field_world as fwb
from field_world import closure as fclosure
from field_world import coord_lint
from field_world import raster_ops as ro
from field_world.scene_spec import FieldSpecError, FieldWorldSpec, catalogue_spec
from steering import scan_unobserved_clip as clip

HERE = Path(__file__).resolve().parent
REF_PATH = HERE / 'closed_loop' / 'testdata' / 'raster_world_ref_e71e27d4.py'
REF_SHA256 = 'e71e27d40522073908d2b5212a4a05f92cb1187c783f69256fc5080bd9d5f507'


def reference_module():
    raw = REF_PATH.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == REF_SHA256
    spec = importlib.util.spec_from_file_location('raster_world_ref', REF_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def real_data_missing():
    try:
        RC.maps_root()
        RC.catalogue_path()
        return None
    except FileNotFoundError as error:
        return str(error)


def skip(reason):
    try:
        import pytest
    except ImportError:
        raise SkipTest(reason)
    pytest.skip(reason)


class SkipTest(Exception):
    """Skip outside pytest (the 3.10 runner counts these)."""


def raises(exc_type, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc_type as error:
        return error
    raise AssertionError('%s not raised' % exc_type.__name__)


def procedural_spec(mode='R', **extra):
    data = dict(name='t_w3_%s' % mode.lower(), source='procedural', procedural=dict(seed=7, rows=3, row_length_m=25.),
                mode=mode)
    data.update(extra)
    return FieldWorldSpec(data)


# --- raster_ops ------------------------------------------------------------------------------------------------------
def test_merged_rectangles_cover_the_mask_exactly_and_disjointly():
    rng = np.random.default_rng(3)
    for _ in range(30):
        mask = rng.uniform(size=(int(rng.integers(1, 40)), int(rng.integers(1, 40)))) < rng.uniform(.05, .6)
        rects = ro.merged_rectangles(mask)
        cover = np.zeros(mask.shape, dtype=np.int64)
        for r0, r1, c0, c1 in rects:
            cover[r0:r1 + 1, c0:c1 + 1] += 1
        assert np.array_equal(cover > 0, mask) and cover.max(initial=0) <= 1
        assert np.array_equal(ro.rectangles_mask(rects, mask.shape), mask)


def test_label_components_equals_the_chebyshev_3_closure():
    rng = np.random.default_rng(11)
    for _ in range(20):
        n = int(rng.integers(1, 60))
        cells = {(int(r), int(c)) for r, c in rng.integers(0, 30, size=(n, 2))}
        rows = np.array([c[0] for c in cells])
        cols = np.array([c[1] for c in cells])
        labels = ro.label_components(rows, cols)
        # brute force: BFS over Chebyshev distance <= 3
        index = {cell: i for i, cell in enumerate(zip(rows.tolist(), cols.tolist()))}
        seen, comp = {}, 0
        for start in index:
            if start in seen:
                continue
            stack = [start]
            seen[start] = comp
            while stack:
                r, c = stack.pop()
                for dr in range(-3, 4):
                    for dc in range(-3, 4):
                        other = (r + dr, c + dc)
                        if other in index and other not in seen:
                            seen[other] = comp
                            stack.append(other)
            comp += 1
        brute = np.array([seen[cell] for cell in zip(rows.tolist(), cols.tolist())])
        # same partition (labels may be numbered differently)
        pairs = set(zip(labels.tolist(), brute.tolist()))
        assert len(pairs) == len(set(labels.tolist())) == len(set(brute.tolist()))


def test_first_stop_equals_the_reference_supercover_dda():
    REF = reference_module()
    rng = np.random.default_rng(5)
    lut = ro.kind_lut(reflect=RC.REFLECTORS, terminate=(RC.UNOBSERVED,))
    for _ in range(12):
        h, w = (int(v) for v in rng.integers(20, 90, 2))
        cls = rng.choice([RC.FREE, RC.OCCUPIED, RC.UNOBSERVED, RC.UNCERTAIN, RC.ADDED_BRANCH], size=(h, w),
                         p=[.84, .06, .04, .04, .02]).astype(np.uint8)
        cells = np.where(np.isin(cls, RC.REFLECTORS), REF.OCCUPIED,
                         np.where(cls == RC.UNOBSERVED, REF.UNKNOWN, REF.FREE)).astype(np.uint8)
        grid = REF.RasterGrid(cells, .05)
        x, y = rng.uniform(0, w * .05), rng.uniform(0, h * .05)
        angles = -math.pi + np.arange(3200) * (2 * math.pi / 3200) + rng.uniform(-1, 1)
        rr, kk = grid.raycast(x, y, angles, 30.)
        dist, kind = ro.first_stop(lut[cls], .05, x, y, np.cos(angles), np.sin(angles), 30.)
        assert np.array_equal(kk, kind)
        finite = np.isfinite(rr)
        assert np.array_equal(finite, np.isfinite(dist))
        assert np.abs(rr[finite] - dist[finite]).max(initial=0.) < 1e-12


def test_first_stop_per_ray_limits():
    grid = np.zeros((10, 40), dtype=np.uint8)
    grid[:, 30] = ro.REFLECT
    dist, kind = ro.first_stop(grid, .05, .26, .26, np.array([1., 1.]), np.array([0., 0.]), np.array([1.2, 1.3]))
    assert kind[0] == ro.PASS and not np.isfinite(dist[0])               # the reflector face is 1.24 m away
    assert kind[1] == ro.REFLECT and abs(dist[1] - (1.5 - .26)) < 1e-12


def test_transform_round_trip_and_polygon_mask():
    t = ro.Transform2D(12.3, -4.5, .7)
    x, y = t.to_w(*t.to_l(1.25, -2.5))
    assert abs(x - 1.25) < 1e-12 and abs(y + 2.5) < 1e-12
    assert abs(ro.wrap(t.yaw_to_w(t.yaw_to_l(.3)) - .3)) < 1e-12
    assert ro.Transform2D.from_dict(t.to_dict()).theta == round(.7, 9)
    mask = ro.polygon_mask((20, 20), .05, 0, 0, [(.1, .1), (.6, .1), (.6, .6), (.1, .6)])
    assert mask.sum() == 100 and mask[2, 2] and not mask[1, 1]


def test_fmt_and_rounded_keep_few_decimals():
    assert ro.fmt(1.23456789) == '1.23457' and ro.fmt(-0.0000001) == '0' and ro.fmt(2.) == '2'
    assert ro.rounded(dict(a=(1.234567, np.float64(2.5)), b=float('inf')), 3) == dict(a=[1.235, 2.5], b=None)


# --- spec --------------------------------------------------------------------------------------------------------
def test_spec_defaults_canonical_form_and_sha():
    spec = catalogue_spec('040927', 'c02', 'fwd', mode='R')
    data = spec.to_dict()
    assert data['crop'] == {'kind': 'fsim'} and data['collision'] == 'none' and data['box_z_m'] == [.3, .9]
    assert data['trunk_cells'] is None and data['upper_canopy'] is None and data['ground']['mu'] is None
    again = FieldWorldSpec(data)
    assert again.sha256() == spec.sha256() and again.canonical_json() == spec.canonical_json()
    t = spec.with_changes(mode='T')
    assert t.collision == 'hard' and t.trunk_cells is True and t.sha256() != spec.sha256()
    band = FieldWorldSpec(dict(data, crop={'kind': 'band'}))
    assert band.crop == dict(kind='band', half_width_m=12., before_m=8., after_m=15.)


def test_spec_refuses_sealed_directions_and_bad_values():
    for direction in ('fwd', 'rev'):
        error = raises(FieldSpecError, catalogue_spec, '042301', 'c01', direction)
        assert 'sealed' in str(error)
    assert 'field_mode_unknown' in str(raises(FieldSpecError, catalogue_spec, '040927', 'c02', mode='X'))
    base = catalogue_spec('040927', 'c02').to_dict()
    for change, code in ((dict(name='Bad Name'), 'name'), (dict(box_z_m=[.9, .3]), 'box_z'),
                         (dict(crop={'kind': 'fsim', 'half_width_m': 3.}), 'crop'), (dict(extra=1), 'key_unknown'),
                         (dict(modifiers=[{'kind': 'spin'}]), 'modifier'), (dict(trunk_cells=True), 'mode_r')):
        assert code in str(raises(FieldSpecError, FieldWorldSpec, dict(base, **change)))
    proc = procedural_spec().to_dict()
    assert proc['record'] is None and proc['direction'] is None and proc['procedural']['seed'] == 7
    assert 'catalogue_keys' in str(raises(FieldSpecError, FieldWorldSpec, dict(proc, record='040927')))


# --- builder on the procedural orchard -------------------------------------------------------------------------------
def test_mode_r_boxes_are_exactly_the_reflector_cells_and_outputs_are_deterministic():
    spec = procedural_spec('R')
    with tempfile.TemporaryDirectory() as tmp:
        a, b = Path(tmp) / 'a', Path(tmp) / 'b'
        manifest = fwb.write(fwb.build(spec), a)
        fwb.write(fwb.build(spec), b)
        for name in ('field_world.sdf', 'raster.npz', 'eval_truth.json', 'manifest.json'):
            assert (a / name).read_bytes() == (b / name).read_bytes(), name
            assert b'\r\n' not in (a / name).read_bytes()
        assert manifest['coord_lint']['ok'] and manifest['coord_lint']['files_checked'] == 3
        assert manifest['counts']['boxes'] > 0 and manifest['counts']['collision_shapes'] == 0
        row = fclosure.closure(a)
        assert row['cells']['missing'] == 0 and row['cells']['extra'] == 0
        raster = fwb.RasterCopy(a / 'raster.npz')
        assert raster.spec_sha256 == spec.sha256() and raster.classes.dtype == np.uint8
        text = (a / 'field_world.sdf').read_text(encoding='utf-8')
        assert coord_lint.lint_text(text, coord_lint.origin_tokens_from_values([]))['gnss_word'] == 0
        assert 'ubts_rover' not in text
        truth = json.loads((a / 'eval_truth.json').read_text(encoding='utf-8'))
        assert truth['use'].startswith('evaluation only') and truth['s_start_m'] == 0.
        # the corridor start is the W origin with the start tangent along +x
        centre = np.asarray(truth['centreline']['points_w'])
        s = np.asarray(truth['centreline']['vertex_s_m'])
        x0, y0 = np.interp(0., s, centre[:, 0]), np.interp(0., s, centre[:, 1])
        assert abs(x0) < 1e-3 and abs(y0) < 1e-3


def test_mode_t_shapes_and_collisions():
    fw = fwb.build(procedural_spec('T', upper_canopy=True))
    kinds = {p['kind'] for p in fw.parts}
    assert {'trunk', 'trunk_cells'} <= kinds and 'upper_canopy' in kinds
    for part in fw.parts:
        assert part['collision'] == (part['kind'] in ('trunk', 'structure', 'added_hard'))
        if part['kind'] == 'trunk':
            assert fwb.TRUNK_RADIUS_RANGE_M[0] <= part['radius'] <= fwb.TRUNK_RADIUS_RANGE_M[1]
            assert part['z'] == (0., fwb.TRUNK_HEIGHT_M)
    plain = fwb.build(procedural_spec('T', collision='none', trunk_cells=False))
    assert not any(p['collision'] for p in plain.parts) and 'trunk_cells' not in {p['kind'] for p in plain.parts}


def test_band_crop_marks_cells_outside_the_band_unobserved():
    fw = fwb.build(procedural_spec('R', crop={'kind': 'band', 'half_width_m': 3., 'before_m': 2., 'after_m': 2.}))
    full = fwb.build(procedural_spec('R'))
    assert fw.source_info['fsim_equivalent'] is False and full.source_info['fsim_equivalent'] is True
    assert (fw.classes == RC.UNOBSERVED).sum() > (full.classes == RC.UNOBSERVED).sum()
    assert np.isin(fw.classes, RC.REFLECTORS).sum() < np.isin(full.classes, RC.REFLECTORS).sum()


# --- lint ------------------------------------------------------------------------------------------------------------
def test_coord_lint_rules():
    # fixture texts are assembled at run time so that this file itself stays clean for the lint
    tokens = coord_lint.origin_tokens_from_values([(-123.456789, -45.678912)])
    clean = 'pose ' + '128.' + '12345 ' + '35.' + '12345 0.6 ; origin [-12.3, 4.5]'
    counts = coord_lint.lint_text(clean, tokens)
    assert all(not counts[k] for k in coord_lint.JUDGED + coord_lint.EXTRA)
    assert counts['latlon_pair_strict'] == 1          # 5 decimals: the strict rule (run outputs) sees it
    pair = 'fix ' + '35.' + '1234567, ' + '129.' + '1234567 ok'
    assert coord_lint.lint_text(pair, tokens)['latlon_pair'] == 1
    assert coord_lint.lint_text('{"' + 'lati' + 'tude": 1}', tokens)['gnss_word'] == 1
    origin = 'origin: [' + '-123.' + '456789, ' + '-45.' + '678912, 0.0]'
    assert coord_lint.lint_text(origin, tokens)['yaml_origin'] >= 2
    assert coord_lint.lint_text('a -123.457 b -45.679 c', tokens)['yaml_origin'] == 1
    assert coord_lint.lint_text('a -123.457 then far ' + 'x' * 120 + ' -45.679', tokens)['yaml_origin'] == 0
    assert coord_lint.lint_text('/' + 'gps/fix', tokens)['gnss_word'] == 1
    assert coord_lint.lint_text('folder ' + 'gnss' + '_check/x', tokens)['gnss_path'] == 1
    report = coord_lint.lint_files([_Probe('x.json', pair)], tokens)
    assert not report['ok'] and report['totals']['latlon_pair'] == 1
    words = coord_lint.lint_files([_Probe('y.json', '/' + 'gps/data')], tokens)
    assert words['ok'] and not words['extra_ok']


def test_coord_lint_strict_rule_for_run_outputs():
    # 2026-11-09 review RW-07: 4-5 decimal pairs, pairs with other numbers between them and pairs further apart than
    # 60 characters were not found; the strict rule (judged for run outputs) sees them
    lat, lon = '35.' + '1234567', '129.' + '1234567'
    tokens = coord_lint.origin_tokens_from_values([])
    cases = [('a %s, %s b' % ('35.' + '12345', '129.' + '12345'), 0, 1),            # 5 decimals
             ('a %s, %s, %s b' % (lat, '12.' + '3456789', lon), 0, 1),             # another number between
             ('a %s %s %s b' % (lat, 'x' * 70, lon), 0, 1),                        # 70 characters apart
             ('a %s %s %s b' % (lat, 'x' * 260, lon), 0, 0),                       # beyond the strict window
             ('a %s, %s b' % ('35.' + '123', '129.' + '123'), 0, 0)]               # 3 decimals: not a position
    for text, plain, strict in cases:
        counts = coord_lint.lint_text(text, tokens)
        assert (counts['latlon_pair'], counts['latlon_pair_strict']) == (plain, strict), (text[:30], counts)
    probe = _Probe('run.json', cases[0][0])
    assert coord_lint.lint_files([probe], tokens)['ok']
    report = coord_lint.lint_files([probe], tokens, strict=True)
    assert not report['ok'] and 'latlon_pair_strict' in report['judged_rules']


class _Probe:
    def __init__(self, name, text):
        self.name, self.text = name, text


# --- clip node ----------------------------------------------------------------------------------------------------------
def test_clip_restores_the_raster_rule_on_synthetic_grids():
    REF = reference_module()
    rng = np.random.default_rng(8)
    for trial in range(8):
        h, w = (int(v) for v in rng.integers(30, 100, 2))
        cls = rng.choice([RC.FREE, RC.OCCUPIED, RC.UNOBSERVED, RC.UNCERTAIN], size=(h, w),
                         p=[.87, .05, .05, .03]).astype(np.uint8)
        cls[[0, -1], :] = RC.UNOBSERVED
        cls[:, [0, -1]] = RC.UNOBSERVED
        transform = ro.Transform2D(rng.uniform(0, w * .05), rng.uniform(0, h * .05), rng.uniform(-3, 3))
        raster = _Raster(cls, transform)
        full = REF.RasterGrid(np.where(np.isin(cls, RC.REFLECTORS), REF.OCCUPIED,
                                       np.where(cls == RC.UNOBSERVED, REF.UNKNOWN, REF.FREE)).astype(np.uint8), .05)
        through = REF.RasterGrid(np.where(np.isin(cls, RC.REFLECTORS), REF.OCCUPIED, REF.FREE).astype(np.uint8), .05)
        xl, yl = rng.uniform(.5, w * .05 - .5), rng.uniform(.5, h * .05 - .5)
        xw, yw = transform.to_w(xl, yl)
        yaw_w = rng.uniform(-math.pi, math.pi)
        ang_w = yaw_w - math.pi + np.arange(1600) * (2 * math.pi / 1600)
        ang_l = ang_w + transform.theta
        r_through, k_through = through.raycast(xl, yl, ang_l, 30.)
        embree_like = np.where(k_through == REF.OCCUPIED, r_through, np.inf)
        directions = np.column_stack((np.cos(ang_w), np.sin(ang_w), np.zeros(len(ang_w))))
        out, info = clip.clip_ranges(raster, (float(xw), float(yw), .6), directions, embree_like)
        r_full, k_full = full.raycast(xl, yl, ang_l, 30.)
        expected = np.where(k_full == REF.OCCUPIED, r_full, np.inf)
        assert np.array_equal(np.isfinite(out), np.isfinite(expected)), trial
        both = np.isfinite(out)
        assert np.abs(out[both] - expected[both]).max(initial=0.) < 1e-9
        assert info['rays'] == 1600 and info['returns_out'] == int(both.sum())


class _Raster:
    def __init__(self, classes, transform):
        self.classes, self.transform = classes, transform
        self.resolution, self.row0, self.col0, self.unobserved_class = .05, 0, 0, RC.UNOBSERVED


def test_clip_sensor_in_unobserved_space_and_tilted_rays():
    cls = np.full((40, 40), RC.FREE, dtype=np.uint8)
    cls[:, 30:] = RC.UNOBSERVED
    raster = _Raster(cls, ro.Transform2D(0., 0., 0.))
    out, _ = clip.clip_ranges(raster, (1.0, 1.0, .6), np.array([[1., 0., 0.], [-1., 0., 0.]]), np.array([5., .8]))
    assert not np.isfinite(out[0]) and out[1] == .8            # behind the unobserved edge at x 1.5: clipped
    out, _ = clip.clip_ranges(raster, (.5, 1.0, .6), np.array([[1., 0., 0.]]), np.array([.8]))
    assert out[0] == .8
    tilted = np.array([[math.cos(.3), 0., -math.sin(.3)]])         # horizontal reach 1.2 x cos(0.3) = 1.146 > 1.0
    out, _ = clip.clip_ranges(raster, (.5, 1.0, .6), tilted, np.array([1.2]))
    assert not np.isfinite(out[0])
    out, _ = clip.clip_ranges(raster, (.5, 1.0, .6), tilted, np.array([1.03]))       # reach 0.984 < 1.0: kept
    assert out[0] == 1.03
    inside = clip.clip_ranges(raster, (1.8, 1.0, .6), np.array([[1., 0., 0.]]), np.array([.1]))[0]
    assert not np.isfinite(inside[0])


def test_clip_raster_from_a_written_field_world():
    spec = procedural_spec('R')
    with tempfile.TemporaryDirectory() as tmp:
        fwb.write(fwb.build(spec), Path(tmp) / 'w')
        cr = clip.ClipRaster(Path(tmp) / 'w' / 'raster.npz')
        assert cr.sha256 and cr.kinds.shape == cr.shape and cr.boundary_x.size > 0
        angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
        directions = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(360)))
        out, info = clip.clip_ranges(cr, (2., 0., .6), directions, np.full(360, 29.))
        assert info['clipped'] > 0 and np.all(np.isfinite(out) | (out == np.inf))


def test_clip_node_cli_processor_and_noise_after_the_clip():
    # 2026-11-09 reviews RW-01 (the supervisor's arguments, the stamp pairing, a scan without a pose is not
    # published) and RW-04 (the range noise is added after the clip, with the ScanSynthesizer contract)
    args = clip.build_parser().parse_args(['--input', '/sim/scan', '--output', '/sim/scan_clipped', '--raster',
                                           '/x/raster.npz', '--noise-sd', '0.01', '--noise-seed', '4'])
    assert (args.input, args.output, args.raster, args.noise_sd, args.noise_seed) == (
        '/sim/scan', '/sim/scan_clipped', '/x/raster.npz', .01, 4)
    cls = np.full((40, 40), RC.FREE, dtype=np.uint8)
    cls[:, 30:] = RC.UNOBSERVED                                     # unobserved beyond x = 1.5 m
    raster = _Raster(cls, ro.Transform2D(0., 0., 0.))
    processor = clip.ClipProcessor(raster, (0., 0., .6), noise_sd=0.)
    count = 360
    angle_min, increment = -math.pi, 2. * math.pi / count
    ranges = np.full(count, .4)                                     # inside the observed square around the pose
    ranges[count // 2] = 5.                                         # straight ahead (+x): behind the edge
    # a scan before the pose of its stamp waits for it (the executor can take the scan first) ...
    out, info = processor.scan(7, angle_min, increment, ranges, .05, 30., token='scan7')
    assert out is None and not info['matched'] and info['waiting'] and processor.counts['waiting'] == 1
    released = processor.pose(7, (.5, 1., 0.), (0., 0., 0., 1.))
    assert [r[0] for r in released] == ['scan7'] and processor.counts['late_matched'] == 1
    assert released[0][2]['matched'] and processor.counts['unmatched'] == 0 and processor.counts['waiting'] == 0
    out, info = processor.scan(7, angle_min, increment, ranges, .05, 30.)
    assert info['matched'] and not np.isfinite(out[count // 2]) and out[0] == .4
    assert np.array_equal(out, released[0][1], equal_nan=True)
    assert processor.counts['published'] == 2 and processor.counts['clipped_rays'] >= 1
    # ... a lost pose (a newer pose came first) is interpolated between neighbours at most 50 ms apart ...
    ms = 1_000_000
    processor.pose(100 * ms, (.5, 1., 0.), (0., 0., 0., 1.))
    processor.scan(108 * ms, angle_min, increment, ranges, .05, 30., token='scan108')
    released = processor.pose(116 * ms, (.5, 1., 0.), (0., 0., 0., 1.))
    assert [r[0] for r in released] == ['scan108'] and released[0][2]['interpolated']
    assert processor.counts['interpolated'] == 1 and processor.counts['unmatched'] == 0
    assert np.array_equal(released[0][1], out, equal_nan=True)            # same pose: same clipped ranges
    out2, info2 = processor.scan(112 * ms, angle_min, increment, ranges, .05, 30.)   # its pose never came
    assert info2['interpolated'] and processor.counts['interpolated'] == 2
    yawed = clip.ClipProcessor(raster, (0., 0., .6))
    yawed.pose(0, (0., 0., 0.), (0., 0., 0., 1.))
    yawed.pose(20 * ms, (.2, 0., 0.), (0., 0., math.sin(.25), math.cos(.25)))     # 0.5 rad turn
    position, quaternion = yawed._interpolated(10 * ms)
    assert abs(position[0] - .1) < 1e-12 and abs(2. * math.atan2(quaternion[2], quaternion[3]) - .25) < 1e-3
    # ... and dropped when the neighbours are too far apart or too many wait
    processor.pose(400 * ms, (.5, 1., 0.), (0., 0., 0., 1.))
    processor.scan(300 * ms, angle_min, increment, ranges, .05, 30.)      # neighbours 116 and 400 ms
    assert processor.counts['unmatched'] == 1
    for stamp in range(500, 500 + clip.PENDING_SCANS + 2):
        processor.scan(stamp * ms, angle_min, increment, ranges, .05, 30.)
    assert processor.counts['unmatched'] == 3 and processor.counts['waiting'] == clip.PENDING_SCANS
    noisy = clip.ClipProcessor(raster, (0., 0., .6), noise_sd=.01, noise_seed=4)
    noisy.pose(7, (.5, 1., 0.), (0., 0., 0., 1.))
    a, _ = noisy.scan(7, angle_min, increment, ranges, .05, 30.)
    assert not np.isfinite(a[count // 2])                           # the clip decision is made on exact ranges
    finite = np.isfinite(a)
    assert 0. < float(np.std(a[finite] - .4)) < .03
    again = clip.ClipProcessor(raster, (0., 0., .6), noise_sd=.01, noise_seed=4)
    again.pose(7, (.5, 1., 0.), (0., 0., 0., 1.))
    assert np.array_equal(again.scan(7, angle_min, increment, ranges, .05, 30.)[0], a)       # seeded
    contract = clip.add_range_noise(np.array([.051, 29.999, np.inf]), .5, np.random.default_rng(1), .05, 30.)
    assert not np.isfinite(contract[2]) and (not np.isfinite(contract[1]) or contract[1] <= 30.)
    assert not np.isfinite(contract[0]) or contract[0] >= .05
    assert np.array_equal(clip.add_range_noise(ranges, 0., None, .05, 30.), ranges)
    for bad in (-1., float('nan')):
        assert raises(ValueError, clip.ClipProcessor, raster, (0., 0., .6), noise_sd=bad)


def test_world_limits_reports_and_refusal():
    # 2026-11-09 review RW-03: the P7 size and time limits (set after the first measurement, which is their basis)
    ok = fwb.limits_report(dict(boxes=100, cylinders=0, triangles_estimate=1212, sdf_bytes=10000, collision_shapes=0,
                                crop_cells=1000), 'R')
    assert ok['within'] and ok['items']['boxes'] == [100, 10359, True]
    over = fwb.limits_report(dict(boxes=20000, cylinders=3), 'R')
    assert not over['within'] and not over['items']['boxes'][2] and not over['items']['cylinders'][2]
    assert fwb.limits_report(dict(triangles_estimate=700000, cylinders=1500), 'T')['within']
    times = fwb.time_limits_report(dict(embree_load_s=12.1, embree_first_scan_s=.58, time_to_first_scan_s=12.7))
    assert times['within'] and set(times['items']) == {'embree_load_s', 'embree_first_scan_s', 'time_to_first_scan_s'}
    assert not fwb.time_limits_report(dict(embree_first_scan_s=2.))['within']
    spec = procedural_spec('R')
    fw = fwb.build(spec)
    saved = dict(fwb.LIMITS['R'])
    try:
        fwb.LIMITS['R']['boxes'] = 1
        with tempfile.TemporaryDirectory() as tmp:
            assert 'over_limits' in str(raises(fwb.FieldWorldError, fwb.write, fw, Path(tmp) / 'w'))
            assert not (Path(tmp) / 'w').exists()
            manifest = fwb.write(fw, Path(tmp) / 'w2', allow_over_limits=True)
            assert not manifest['limits']['within']
    finally:
        fwb.LIMITS['R'].update(saved)
    with tempfile.TemporaryDirectory() as tmp:
        manifest = fwb.write(fw, Path(tmp) / 'w')
        assert manifest['limits']['within'] and manifest['limits']['mode'] == 'R'


def test_procedural_spec_holdout_seed():
    # 2026-11-09 review RW-06
    base = dict(name='t_hold', source='procedural', procedural=dict(seed=100002, rows=2, row_length_m=12.))
    assert 'holdout_seed' in str(raises(FieldSpecError, FieldWorldSpec, base))
    held = FieldWorldSpec(dict(base, allow_holdout=True))
    assert held.to_dict()['allow_holdout'] is True
    plain = FieldWorldSpec(dict(base, procedural=dict(seed=2, rows=2, row_length_m=12.)))
    assert 'allow_holdout' not in plain.to_dict()                   # development specs keep their canonical form
    assert 'only_for_procedural' in str(raises(FieldSpecError, catalogue_spec, '040927', 'c02', allow_holdout=True))


# --- real data (skip without the maps) -----------------------------------------------------------------------------------
def test_catalogue_field_world_mode_r_closure_and_lint():
    missing = real_data_missing()
    if missing:
        skip('10/10 maps or the catalogue not configured: %s' % missing)
    spec = catalogue_spec('040927', 'c06', 'rev', mode='R')
    fw = fwb.build(spec)
    assert fw.source_info['pgm_sha256'] and fw.source_info['fsim_equivalent']
    sim = fw.sim_world.map.classes
    assert np.array_equal(sim, fw.classes)
    with tempfile.TemporaryDirectory() as tmp:
        manifest = fwb.write(fw, Path(tmp) / 'w')
        assert manifest['coord_lint']['ok'] and manifest['coord_lint']['yaml_origin_checked']
        row = fclosure.closure(Path(tmp) / 'w')
        assert row['passed'] is True, row
