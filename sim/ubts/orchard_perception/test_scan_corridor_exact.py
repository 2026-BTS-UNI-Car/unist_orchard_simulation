"""Bit-exact identity of the optimised observe_scan_corridor against the frozen 999a863 body.

``testdata/scan_corridor_ref_999a863.py`` is the unmodified main-branch file
(``git show 999a863:sim/ubts/orchard_perception/scan_corridor.py``); its git
blob id is checked before it is executed under a private module name. Every
CorridorObservation field (diagnostics included) must match bit for bit:
floats are compared by their IEEE-754 bytes (so -0.0 != 0.0, and np.float64 is
not accepted for float), arrays by dtype, shape and bytes.

Inputs: ray-cast scans of the closed-loop scene generator (closed_loop.world,
50 seeds x 5 poses, harness LiDAR noise and float32 ranges) and constructed
scans whose laser-frame y values are reproduced bit for bit, so that at the
0 deg heading (cross == y exactly) points sit exactly on +-tolerance window
edges, one ulp outside them, and inside the bisection superset of
_band_window; plus +-0.0 returns, duplicated cross values, more than
max_band_peaks_per_side bands on one side, empty/one-point scans and returns
exactly at the support range. With the default 0.22 (an odd multiple of
2**-55) equality |cross - centre| == tolerance cannot occur for seed-estimate
centres (|centre| >= .5: the difference of the two values is exact and a
multiple of 2**-54), so the edge scenes also run with the dyadic
band_tolerance_m=0.25, where it does occur.

Known helper-level difference (strict xfail, unreachable from
observe_scan_corridor): _finite_span_sorted of an along list that contains
-0.0 can return a zero of the other sign than the reference _finite_span,
because np.sort (reference, point order) and sorted() (candidate, cross order)
place equal +-0.0 differently. along/cross come from matmul, which never
returns -0.0; test_premise_matmul_never_returns_negative_zero pins that.
"""
from collections import deque
import bisect
import copy
import dataclasses
import hashlib
import importlib.util
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from closed_loop.world import WorldConfig, build_world, make_lidar, yaw_quaternion

from . import scan_corridor as new

HERE = Path(__file__).resolve().parent
REF_PATH = HERE / 'testdata' / 'scan_corridor_ref_999a863.py'
REF_BLOB = '1a593e1763d6e1a8fe979625a01afed620deb218'   # git rev-parse 999a863:sim/ubts/orchard_perception/scan_corridor.py
REF_MODULE = '_frozen_999a863_scan_corridor'


# --- frozen reference loading and exact comparison (shared with sim/ubts/test_closed_loop_estimator_exact.py) ---

def git_blob_id(path):
    data = Path(path).read_bytes()
    return hashlib.sha1(b'blob %d\0' % len(data) + data).hexdigest()


def load_frozen(name, path, blob, *, modules=None):
    """Execute a frozen copy under a unique module name, after checking its git blob id.

    ``modules`` pre-binds absolute imports of the frozen file (e.g. the frozen
    estimator's ``from orchard_perception.scan_corridor import ...``) only
    while it executes; sys.modules is restored afterwards.
    """
    if name in sys.modules:
        return sys.modules[name]
    actual = git_blob_id(path)
    if actual != blob:
        raise AssertionError(f'{path} is not the frozen blob {blob} (got {actual})')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    saved = {key: sys.modules.get(key) for key in (modules or {})}
    sys.modules[name] = module
    sys.modules.update(modules or {})
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    finally:
        for key, value in saved.items():
            if value is None:
                sys.modules.pop(key, None)
            else:
                sys.modules[key] = value
    return module


def canon(value):
    """Exact, type-strict canonical form (floats as IEEE-754 bytes, arrays as dtype/shape/bytes)."""
    if value is None or type(value) in (bool, str):
        return (type(value).__name__, value)
    if isinstance(value, np.ndarray):
        return ('ndarray', value.dtype.str, value.shape, np.ascontiguousarray(value).tobytes())
    if isinstance(value, np.generic):
        return (type(value).__name__, value.tobytes())
    if type(value) is float:
        return ('float', struct.pack('<d', value))
    if type(value) is int:
        return ('int', value)
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return ('dataclass', type(value).__name__,
                tuple((f.name, canon(getattr(value, f.name))) for f in dataclasses.fields(value)))
    if type(value) is dict:
        return ('dict', tuple((canon(k), canon(v)) for k, v in value.items()))
    if type(value) is deque:
        return ('deque', value.maxlen, tuple(canon(v) for v in value))
    if type(value) in (list, tuple):
        return (type(value).__name__, tuple(canon(v) for v in value))
    raise TypeError(f'no exact canonical form for {type(value)!r}')


def first_difference(a, b, path=''):
    if a == b:
        return None
    if type(a) is tuple and type(b) is tuple and len(a) == len(b):
        for index, (x, y) in enumerate(zip(a, b)):
            if x != y:
                label = x[1] if (type(x) is tuple and len(x) == 2 and x[0] == 'str') else index
                return first_difference(x, y, f'{path}/{label}')
    return path or '/', a, b


def assert_identical(expected, actual, context):
    ce, ca = canon(expected), canon(actual)
    if ce != ca:
        pytest.fail(f'{context}: not bit-identical (reference vs candidate) at {first_difference(ce, ca)}')


ref = load_frozen(REF_MODULE, REF_PATH, REF_BLOB)


def ref_config(config):
    return None if config is None else ref.CorridorConfig(**dataclasses.asdict(config))


def compare_observation(scan, config=None, context=''):
    before = canon(copy.deepcopy(getattr(scan, 'ranges', None)))
    expected = ref.observe_scan_corridor(scan, ref_config(config))
    actual = new.observe_scan_corridor(scan, config)
    assert canon(getattr(scan, 'ranges', None)) == before, 'scan was modified'
    assert type(expected).__name__ == type(actual).__name__ == 'CorridorObservation'
    assert_identical(expected, actual, context)
    assert_identical(expected.width_m, actual.width_m, context + ' width_m')
    return actual


def test_reference_copy_is_the_main_blob_and_configs_match():
    assert git_blob_id(REF_PATH) == REF_BLOB
    assert ref.__file__ != new.__file__ and ref.observe_scan_corridor is not new.observe_scan_corridor
    assert [(f.name, f.default) for f in dataclasses.fields(ref.CorridorConfig)] == \
        [(f.name, f.default) for f in dataclasses.fields(new.CorridorConfig)]


def test_canon_distinguishes_signed_zero_and_float_types():
    assert canon(0.) != canon(-0.) and canon(1.) != canon(np.float64(1.)) and canon(1) != canon(True)
    assert canon(np.array([0.])) != canon(np.array([-0.])) and canon(np.zeros(2)) != canon(np.zeros((1, 2)))


# --- (a) ray-cast scans of the closed-loop scene generator -----------------------------------------

def _variant(seed):
    s = dict(seed=seed)
    variants = (
        WorldConfig(**s),
        WorldConfig(scenario='wall_beyond_rows', **s),
        WorldConfig(scenario='wall_across_before_end', **s),
        WorldConfig(scenario='narrow_headland', **s),
        WorldConfig(scenario='missing_trunk_gap', missing_probability=.15, **s),
        WorldConfig(scenario='occluded_next_row', **s),
        WorldConfig(row_count=6, stagger=True, **s),
        WorldConfig(row_width_m=2.6, tree_spacing_m=2., field_yaw_deg=30., branch_reach_m=.3, **s),
        WorldConfig(row_count=6, row_width_m=3., tree_spacing_m=3., **s),
        WorldConfig(branches_per_tree=(4, 7), branch_reach_m=.55, field_yaw_deg=-20., **s),
    )
    return variants[seed % len(variants)]


# (field x as a fraction of the row length or absolute headland offset, lateral, yaw deg, occluded sector deg)
POSES = ((lambda length: 2.5, 0., 5., None),
         (lambda length: .5 * length, .4, -12., None),
         (lambda length: length - 1., -.3, 25., (30., 150.)),
         (lambda length: -1.5, .1, 10., None),
         (lambda length: .5 * length, 0., 88., None))


def world_scans(seed):
    config = _variant(seed)
    world = build_world(config)
    caster, lidar = make_lidar(world, seed=1_000 + seed)
    centre = (2 if config.row_count >= 6 else 0) * config.row_width_m
    scans = []
    for index, (fx, fy, yaw_deg, sector) in enumerate(POSES):
        x, y, yaw = world.pose_in_field(fx(config.row_length_m), centre + fy, math.radians(yaw_deg))
        caster.occluded_sectors = () if sector is None else ((math.radians(sector[0]), math.radians(sector[1])),)
        frame = lidar.sample((index + 1) * 100_000_000, (x, y, .6), yaw_quaternion(yaw))
        assert frame is not None
        scans.append(SimpleNamespace(ranges=frame.ranges, angle_min=lidar.angle_min,
                                     angle_increment=lidar.angle_increment, range_min=.05, range_max=30.))
    return scans


WORLD_REASONS = {}


@pytest.mark.parametrize('seed', range(50))
def test_world_scans_match_reference(seed):
    for index, scan in enumerate(world_scans(seed)):
        observation = compare_observation(scan, context=f'seed {seed} pose {index}')
        WORLD_REASONS[(seed, index)] = observation.reason


def test_world_scans_cover_valid_and_rejected_observations():
    if len(WORLD_REASONS) < 250:
        pytest.skip('runs after test_world_scans_match_reference in the same session')
    reasons = set(WORLD_REASONS.values())
    valid = sum(reason == 'observed_parallel_surface_bands' for reason in WORLD_REASONS.values())
    assert valid >= 50 and 'ambiguous_parallel_surface_bands' in reasons, reasons


CONFIGS = (new.CorridorConfig(max_band_peaks_per_side=1),
           new.CorridorConfig(max_band_peaks_per_side=2, heading_step_deg=3.),
           new.CorridorConfig(heading_step_deg=1., spatial_cell_m=.05, band_tolerance_m=.3),
           new.CorridorConfig(band_tolerance_m=.25, longitudinal_bin_m=.5, min_occupied_bins=4, min_overlap_m=2.),
           new.CorridorConfig(support_range_m=6., min_width_m=2., max_width_m=4., ambiguity_score_ratio=.5))


@pytest.mark.parametrize('config_index', range(len(CONFIGS)))
def test_world_scans_match_reference_with_other_configs(config_index):
    config = CONFIGS[config_index]
    for seed in range(0, 50, 7):
        for index, scan in enumerate(world_scans(seed)):
            compare_observation(scan, config, context=f'config {config_index} seed {seed} pose {index}')


# --- (b) constructed scans: exact laser-frame y values -------------------------------------------

EXACT_N = 4096
EXACT_INC = (math.pi / 2) / 1024        # exact: 4096 * inc == 2 * pi in floating point
FULL_SIN = np.sin(0. + np.arange(EXACT_N) * EXACT_INC)


def _exact_range(y, s):
    """A range r with r * s == y bit for bit (r > 0), or None."""
    r = y / s
    for direction in (math.inf, -math.inf):
        candidate = r
        for _ in range(64):
            if candidate * s == y:
                return candidate
            candidate = math.nextafter(candidate, direction)
    return None


def support_points(scan, support_range=10.):
    """The support points exactly as observe_scan_corridor computes them (before cell thinning)."""
    ranges = np.asarray(scan.ranges, dtype=float)
    n = len(ranges)
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges < scan.range_max)
    support = valid & (ranges <= support_range)
    a = scan.angle_min + np.arange(n)[support] * scan.angle_increment
    r = ranges[support]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def exact_scan(exact=(), approximate=(), pinned=(), range_min=.05):
    """Scan whose support points reproduce every y in ``exact`` bit for bit (x approximately).

    exact/approximate: (x, y) targets, one beam each; pinned: (beam, range).
    """
    ranges = np.full(EXACT_N, np.inf)
    taken = set()
    for beam, distance in pinned:
        ranges[beam] = distance
        taken.add(beam)
    for x, y, want_exact in [(x, y, True) for x, y in exact] + [(x, y, False) for x, y in approximate]:
        centre = int(round((math.atan2(y, x) % (2 * math.pi)) / EXACT_INC)) % EXACT_N
        for step in range(600):
            done = False
            for beam in ((centre + step) % EXACT_N, (centre - step) % EXACT_N):
                if beam in taken:
                    continue
                if want_exact:
                    s = float(FULL_SIN[beam])
                    if abs(s) < .05:
                        continue
                    distance = _exact_range(y, s)
                else:
                    distance = math.hypot(x, y)
                if distance is None or not range_min < distance <= 10.:
                    continue
                ranges[beam] = distance
                taken.add(beam)
                done = True
                break
            if done:
                break
        else:
            raise AssertionError(f'no beam for target {(x, y)}')
    scan = SimpleNamespace(ranges=ranges, angle_min=0., angle_increment=EXACT_INC, range_min=range_min,
                           range_max=30.)
    ys = {struct.pack('<d', v) for v in support_points(scan)[:, 1]}
    missing = [y for _, y in exact if struct.pack('<d', y) not in ys]
    assert not missing, f'construction did not reproduce y exactly: {missing[:5]}'
    return scan


def kept_points(scan, cell=.1):
    raw = support_points(scan)
    _, index = np.unique(np.floor(raw / cell).astype(np.int32), axis=0, return_index=True)
    return raw[np.sort(index)]


def boundary_values(centre, tolerance):
    """(on-edge-or-last-inside, first-outside, inside-bisection-superset-but-outside) above and below centre."""
    result = []
    for sign in (1., -1.):
        # bisection, moving away from centre, for the last double inside the window (exactly
        # on the edge when the difference is representable) and the first double outside it
        inside, outside = centre, centre + sign * 2 * tolerance
        while math.nextafter(inside, outside) != outside:
            middle = inside + (outside - inside) / 2
            if middle in (inside, outside):
                middle = math.nextafter(inside, outside)
            if abs(middle - centre) <= tolerance:
                inside = middle
            else:
                outside = middle
        superset = centre + sign * (tolerance + 5e-10)
        assert abs(outside - centre) > tolerance and abs(superset - centre) > tolerance
        result.append((inside, outside, superset))
    return result


GAP_BINS = dict(in_hi=2, in_lo=-3, out_hi=5, out_lo=-6, sup_hi=8, sup_lo=-9)   # 0.75 m along bins


def boundary_scene(tolerance, k=6, inside=True):
    """Two 18 m bands at +-k*tolerance (exact seed estimates; 0 deg is the best heading).

    The band points leave six 0.75 m along bins empty; each holds one point on a
    window edge (in), one ulp outside it (out) or inside the +-1e-9 bisection
    superset but outside the window (sup), so including or excluding it changes
    the band's occupied-bin count and therefore the score.
    """
    exact = []
    for sign in (1., -1.):
        centre = sign * (k * tolerance)
        exact += [(x, centre) for x in np.arange(-9., 9.01, .25)          # duplicated cross values
                  if math.floor(x / .75) not in GAP_BINS.values()]
        (in_hi, out_hi, sup_hi), (in_lo, out_lo, sup_lo) = boundary_values(centre, tolerance)
        values = dict(in_hi=in_hi, in_lo=in_lo, out_hi=out_hi, out_lo=out_lo, sup_hi=sup_hi, sup_lo=sup_lo)
        exact += [((along_bin + .5) * .75, values[name]) for name, along_bin in GAP_BINS.items()
                  if inside or not name.startswith('in_')]
    return exact


@pytest.fixture
def window_log(monkeypatch):
    """Record every _band_window call of the candidate (pure pass-through)."""
    calls = []
    original = new._band_window

    def recorded(sorted_cross, centre, tolerance):
        first, last = original(sorted_cross, centre, tolerance)
        lo = bisect.bisect_left(sorted_cross, centre - tolerance - 1e-9)
        hi = bisect.bisect_right(sorted_cross, centre + tolerance + 1e-9)
        calls.append(dict(on_edge=sum(abs(v - centre) == tolerance for v in sorted_cross[first:last]),
                          trimmed=(first - lo) + (hi - last)))
        return first, last
    monkeypatch.setattr(new, '_band_window', recorded)
    return calls


@pytest.mark.parametrize('tolerance', (.25, .22))
def test_points_on_window_edges_match_reference(tolerance, window_log):
    config = new.CorridorConfig(band_tolerance_m=tolerance)
    scan = exact_scan(boundary_scene(tolerance))
    kept = {struct.pack('<d', v) for v in kept_points(scan)[:, 1]}
    for centre in (6 * tolerance, -6 * tolerance):
        for inside, outside, superset in boundary_values(centre, tolerance):
            assert {struct.pack('<d', v) for v in (inside, outside, superset)} <= kept
    observation = compare_observation(scan, config, context=f'window edges tol={tolerance}')
    assert observation.valid and observation.heading_axis_laser_rad == 0.
    assert sum(call['trimmed'] > 0 for call in window_log) >= 2       # superset values removed by the predicate
    if tolerance == .25:                                                 # dyadic: |cross - centre| == tol occurs
        assert sum(call['on_edge'] >= 2 for call in window_log) >= 2
    else:                                                                # 0.22: no exact equality here
        assert all(call['on_edge'] == 0 for call in window_log)
    # The edge points are observable: without them the reference result differs.
    without = ref.observe_scan_corridor(exact_scan(boundary_scene(tolerance, inside=False)), ref_config(config))
    assert canon(without) != canon(observation)
    # The same scene with the default configuration and with the other tolerance.
    compare_observation(scan, context=f'window edges tol={tolerance}, default config')
    compare_observation(scan, new.CorridorConfig(band_tolerance_m=.47 - tolerance), context='other tolerance')


def _parallel_lines(left_count, right_count=2, spacing=.6, half_length=4.):
    targets = []
    for count, sign in ((left_count, 1.), (right_count, -1.)):
        for j in range(1, count + 1):
            targets += [(x, sign * j * spacing) for x in np.arange(-half_length, half_length + .01, .25)]
    return targets


@pytest.mark.parametrize('left_count', (7, 8, 9, 10))
def test_band_capacity_per_side_matches_reference(left_count):
    scan = exact_scan(approximate=_parallel_lines(left_count))
    for config in (None, new.CorridorConfig(max_band_peaks_per_side=4), new.CorridorConfig(spatial_cell_m=.05)):
        observation = compare_observation(scan, config, context=f'{left_count} left lines, config {config}')
        if left_count >= 9 and config is None:
            assert observation.diagnostics['band_capacity_rejected_headings'] > 0


def test_signed_zero_and_degenerate_returns_match_reference():
    # the first origin return in beam order is (-0.0, -0.0): it is the one kept by cell thinning
    pinned = [(beam, -0.) for beam in (2, 1500, 3000)] + [(beam, 0.) for beam in (5, 900, 2047)]
    targets = boundary_scene(.25)
    # rays along the x/y axes (sin/cos of 0, pi/2, pi give 0, +-1 and tiny values)
    pinned += [(0, 3.), (1024, 2.5), (2048, 4.), (3072, 1.5)]
    for range_min in (0., .05):
        scan = exact_scan(targets, pinned=pinned, range_min=range_min)
        if range_min == 0.:
            raw = support_points(scan)
            zeros = raw[(raw[:, 0] == 0) & (raw[:, 1] == 0)]
            assert np.signbit(zeros).any() and (~np.signbit(zeros)).any()
        for config in (None, new.CorridorConfig(band_tolerance_m=.25)):
            compare_observation(scan, config, context=f'signed zero range_min={range_min}')


def test_returns_exactly_at_the_support_range_match_reference():
    assert FULL_SIN[1024] == 1. and FULL_SIN[3072] == -1.
    pinned = [(0, 10.), (1024, 10.), (3072, 10.), (2048, 10.), (1, math.nextafter(10., 11.)),
              (1025, math.nextafter(10., 0.))]
    scan = exact_scan(boundary_scene(.25), pinned=pinned)
    raw = support_points(scan)
    assert np.any(raw[:, 1] == 10.) and np.any(raw[:, 1] == -10.)        # |cross| == support range at 0 deg
    for config in (None, new.CorridorConfig(band_tolerance_m=.25)):
        compare_observation(scan, config, context='support range edge')
    ring = SimpleNamespace(ranges=np.full(EXACT_N, 10.), angle_min=0., angle_increment=EXACT_INC,
                           range_min=.05, range_max=30.)
    compare_observation(ring, context='ring at the support range')
    ring.ranges = np.where(np.arange(EXACT_N) % 3 == 0, 10., 9.)
    compare_observation(ring, context='ring at and inside the support range')


@pytest.mark.parametrize('ranges', (
    [], [1.], [1., 2.], [np.inf] * 3, [1.] + [np.inf] * 2, [np.nan] * 5,
    [2.] * 11 + [np.inf] * 5, [2.] * 12 + [np.inf] * 5, [2.] * 13,     # min_occupied_bins * 2 support threshold
), ids=lambda r: f'n{len(r)}')
def test_tiny_and_empty_scans_match_reference(ranges):
    n = max(len(ranges), 1)
    scan = SimpleNamespace(ranges=np.asarray(ranges, dtype=float), angle_min=0., angle_increment=1e-4,
                           range_min=.05, range_max=30.)
    compare_observation(scan, context=f'tiny scan {ranges}')
    scan.angle_increment = 2 * math.pi / n
    compare_observation(scan, context=f'tiny scan full circle {ranges}')


def test_single_kept_point_after_cell_thinning_matches_reference():
    # 13 support returns that all fall into one 0.1 m cell -> one point after thinning
    scan = SimpleNamespace(ranges=np.full(13, 3.05), angle_min=0., angle_increment=1e-5, range_min=.05,
                           range_max=30.)
    observation = compare_observation(scan, context='one cell')
    assert observation.diagnostics['spatial_support_cells'] == 1


def test_invalid_geometry_and_capacity_match_reference():
    base = dict(ranges=np.full(100, 2.), angle_min=0., angle_increment=.01, range_min=.05, range_max=30.)
    for change in (dict(), dict(ranges=np.full(4097, 2.)), dict(angle_increment=0.), dict(angle_min=7.),
                   dict(range_min=-1.), dict(range_max=31.), dict(range_max=.05), dict(angle_increment=.07),
                   dict(ranges=np.full((10, 2), 2.)), dict(angle_min=float('nan')), dict(ranges='abc')):
        compare_observation(SimpleNamespace(**{**base, **change}), context=f'geometry {change}')
    compare_observation(SimpleNamespace(ranges=[2.] * 50), context='missing attributes')


@pytest.mark.parametrize('tolerance', (.25, .5, .22, 1e-3))
def test_band_window_equals_the_reference_mask(tolerance):
    """_band_window == the reference predicate np.abs(cross - centre) <= tolerance on sorted values."""
    rng = np.random.default_rng(int(tolerance * 1000))
    for trial in range(400):
        centre = float(rng.choice((rng.uniform(-10., 10.), int(rng.integers(-40, 41)) * tolerance, 0., -0.,
                                   rng.uniform(-10., 10.) // tolerance * tolerance)))
        values = [float(v) for v in rng.uniform(centre - 3 * tolerance, centre + 3 * tolerance,
                                                int(rng.integers(0, 30)))]
        for inside, outside, superset in boundary_values(centre, tolerance):
            values += [inside] * int(rng.integers(0, 3)) + [outside] * int(rng.integers(0, 2))
            values += [superset, math.nextafter(superset, centre)] * int(rng.integers(0, 2))
        values += [centre] * int(rng.integers(0, 3)) + [0., -0.] * int(rng.integers(0, 2))
        ordered = sorted(float(v) for v in rng.permutation(np.asarray(values, dtype=float)))
        first, last = new._band_window(ordered, centre, tolerance)
        mask = np.abs(np.asarray(ordered, dtype=float) - centre) <= tolerance
        assert np.flatnonzero(mask).tolist() == list(range(first, last)), (trial, centre, tolerance)


def test_sorted_span_equals_the_reference_span():
    """_finite_span_sorted(sorted(L)) == reference _finite_span(any order of L) for along values as
    matmul produces them: duplicates, +0.0, subnormals, but never -0.0 (see the next two tests)."""
    rng = np.random.default_rng(5)
    for trial in range(2000):
        size = int(rng.integers(1, 60))
        if trial % 2:
            values = rng.choice((0., 5e-324, -5e-324, 1., -1., .75, 2.5, -3.25), size)
        else:
            values = rng.normal(0., 4., size)
        reference_order, candidate_order = rng.permutation(values), rng.permutation(values)
        assert_identical(ref._finite_span(reference_order),
                         new._finite_span_sorted(sorted(candidate_order.tolist())), f'span trial {trial}')


def test_premise_matmul_never_returns_negative_zero():
    """cross/along come from points @ vector, whose accumulation starts at +0.0: a -0.0 coordinate
    (zero/-0.0 ranges with range_min=0, underflowing subnormal ranges) never yields a -0.0 value.
    This keeps the next test's signed-zero divergence unreachable from observe_scan_corridor."""
    points = np.array([[-0., -0.], [-5e-324, 0.], [0., -5e-324], [-0., 0.], [-0., 1.], [1., -0.], [-5e-324, -0.]])
    for deg in np.arange(-90., 90., 1.):
        theta = math.radians(float(deg))
        direction = np.array((math.cos(theta), math.sin(theta)))
        normal = np.array((-direction[1], direction[0]))
        for values in (points @ direction, points @ normal, np.repeat(points, 37, axis=0) @ normal):
            assert not np.any((values == 0) & np.signbit(values)), deg


@pytest.mark.xfail(strict=True, reason='helper-level only: with -0.0 inputs the 5th percentile takes the sign of '
                   'the first zero after sorting, which differs between np.sort (reference, point order) and '
                   'sorted() (candidate, cross order); matmul never produces -0.0 (premise test above)')
def test_sorted_span_with_negative_zero_inputs_differs_from_reference():
    values = [-5e-324, -0., 0.] + [float(i) for i in range(1, 9)]        # n=11: index .5, upper branch
    reference = ref._finite_span(np.asarray(values))                      # np.sort keeps -0.0 first: [-0.0, 7.5]
    candidate = new._finite_span_sorted(sorted([0., -0., -5e-324] + values[3:]))   # +0.0 first: [0.0, 7.5]
    assert_identical(reference, candidate, 'signed-zero span')


def _random_scene_scan(rng):
    targets = []
    for _ in range(int(rng.integers(1, 7))):
        heading, offset = rng.uniform(-math.pi, math.pi), rng.uniform(-6., 6.)
        start, length = rng.uniform(-8., 2.), rng.uniform(1., 12.)
        step = rng.uniform(.08, .5)
        for along in np.arange(start, start + length, step):
            if rng.random() < .15:
                continue                                    # gaps
            cross = offset + rng.normal(0., rng.choice((0., .02, .1)))
            targets.append((along * math.cos(heading) - cross * math.sin(heading),
                            along * math.sin(heading) + cross * math.cos(heading)))
    targets += [tuple(rng.uniform(-9., 9., 2)) for _ in range(int(rng.integers(0, 80)))]
    targets = [t for t in targets if .1 < math.hypot(*t) <= 9.9]
    if rng.random() < .5:                                   # quantised coordinates: many exact ties
        targets = [(round(x / .05) * .05, round(y / .05) * .05) for x, y in targets]
    if rng.random() < .5:
        exact = [t for t in targets if abs(t[1]) >= .3][:400]
        rest = [t for t in targets if abs(t[1]) < .3] + [t for t in targets if abs(t[1]) >= .3][400:]
        return exact_scan(exact=exact, approximate=rest[:1100])
    return exact_scan(approximate=targets[:1500])


@pytest.mark.parametrize('block', range(4))
def test_random_line_scenes_match_reference(block):
    rng = np.random.default_rng(20261017 + block)
    configs = (None, new.CorridorConfig(band_tolerance_m=.25), new.CorridorConfig(max_band_peaks_per_side=3))
    for index in range(25):
        scan = _random_scene_scan(rng)
        compare_observation(scan, configs[index % 3], context=f'random block {block} scene {index}')
