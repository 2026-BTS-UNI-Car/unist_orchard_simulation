"""Bit-exact identity of the optimised SensorEstimator against the frozen 999a863 body.

``closed_loop/testdata/estimator_ref_999a863.py`` is the unmodified main-branch
estimator (``git show 999a863:sim/ubts/closed_loop/estimator.py``, git blob id
checked). It is executed under a private module name with its own
``from orchard_perception.scan_corridor import observe_scan_corridor`` bound to
the frozen 999a863 scan_corridor, so the end-to-end comparison is main
(estimator + scan_corridor) against the candidate (estimator + scan_corridor).

(a) _first_cell_indices (both copies) == np.sort(np.unique(c, axis=0, return_index=True)[1]);
(b) _axis_pair / _side_extents outputs on closed-loop ray-cast scans, constructed
    edge cases (exact bin/slab/spread/veto/width boundaries, ties, +-0.0) and
    random clouds;
(c) closed-loop episodes in which both estimators receive every scan and IMU
    sample: the full estimator state (every attribute, arrays by bytes), every
    _axis_pair/_side_extents call (inputs and outputs) and every controller
    snapshot must be bit-identical. The candidate's snapshot drives the loop.
"""
from collections import Counter
from dataclasses import asdict, fields, replace
import math
from pathlib import Path

import numpy as np
import pytest

import closed_loop.estimator as new_estimator
from closed_loop.harness import EpisodeConfig, run_episode
from closed_loop.world import WorldConfig
from orchard_perception import scan_corridor as new_scan_corridor
from orchard_perception.test_scan_corridor_exact import (REF_BLOB as SCAN_CORRIDOR_BLOB, REF_MODULE as
                                                         SCAN_CORRIDOR_MODULE, REF_PATH as SCAN_CORRIDOR_PATH,
                                                         assert_identical, canon, first_difference, load_frozen,
                                                         world_scans)

HERE = Path(__file__).resolve().parent
REF_PATH = HERE / 'closed_loop' / 'testdata' / 'estimator_ref_999a863.py'
REF_BLOB = 'b5de30b3d80d2474e7eaa85326d6064452eae433'   # git rev-parse 999a863:sim/ubts/closed_loop/estimator.py

ref_scan_corridor = load_frozen(SCAN_CORRIDOR_MODULE, SCAN_CORRIDOR_PATH, SCAN_CORRIDOR_BLOB)
ref_estimator = load_frozen('_frozen_999a863_closed_loop_estimator', REF_PATH, REF_BLOB,
                            modules={'orchard_perception.scan_corridor': ref_scan_corridor})


# Added by the opt-in row-end position bridge: (name, OFF default). Never read while OFF.
ROW_END_POSITION_FIELDS = (('row_end_position_confirm_m', None), ('row_end_position_max_gap_s', 6.))
# Added by the opt-in line-continuation veto: (name, OFF default). Never read while OFF.
CONTINUATION_FIELDS = (('row_end_continuation_m', None), ('row_end_continuation_band_m', .35),
                       ('row_end_continuation_band_slope', .035), ('row_end_continuation_band_max_m', .8),
                       ('row_end_continuation_bin_m', .5), ('row_end_continuation_min_bins', 2),
                       # v2 pair rule (the default rule of row_end_continuation_m; read only while it is set)
                       ('row_end_continuation_rule', 'pair'),
                       ('row_end_continuation_link_m', .35), ('row_end_continuation_object_along_m', 1.2),
                       ('row_end_continuation_object_cross_m', .8), ('row_end_continuation_object_min_points', 3),
                       ('row_end_continuation_object_sep_m', 1.5), ('row_end_continuation_min_objects', 2))
DISTANCE_FIELDS = (('distance_mode', 'scan_norm_legacy'),)
# Added by the single-row options (2026-11-02): support segmentation at the row end and the first-acquisition window.
# Never read while OFF (span / behind None).
SINGLE_ROW_FIELDS = (('row_end_segment_span_m', None), ('row_end_segment_min_gap_m', 6.),
                     ('row_end_segment_bin_m', .5), ('row_end_segment_min_bins', 4),
                     ('band_rotate_about_vehicle', False), ('row_end_stagger_gap_m', None),
                     ('row_end_stagger_window_m', 2.), ('row_end_last_trunk', False), ('acquire_behind_m', None))
# Added by the design v2 inputs / outputs (2026-11-12): actuator priors (L1), the corridor quality block and the external
# corridor observation slot. Never read while OFF (None / False / gain 0); their state stays None then.
V2_FIELDS = (('actuator_prior', None), ('corridor_quality', False), ('external_corridor_gain', 0.),
             ('external_corridor_sources', ()), ('external_corridor_max_age_s', .5),
             ('external_corridor_min_quality', .5))
# Added by the curved row ends (S1c, 2026-11-13). Never read while OFF (row_end_curve_fit False); no state then.
CURVE_FIELDS = (('row_end_curve_fit', False), ('row_end_curve_reach_m', (4., 6., 8., 10.)),
                ('row_end_curve_behind_m', 2.), ('row_end_curve_max_1_m', 1. / 15.), ('row_end_curve_min_points', 6),
                ('row_end_curve_edge_m', 1.5), ('row_end_curve_max_gap_m', 3.))
OPT_IN_FIELDS = (DISTANCE_FIELDS + ROW_END_POSITION_FIELDS + CONTINUATION_FIELDS + SINGLE_ROW_FIELDS + V2_FIELDS
                 + CURVE_FIELDS)


def test_frozen_estimator_uses_the_frozen_scan_corridor():
    assert ref_estimator.observe_scan_corridor is ref_scan_corridor.observe_scan_corridor
    assert new_estimator.observe_scan_corridor is new_scan_corridor.observe_scan_corridor
    assert ref_estimator.SensorEstimator is not new_estimator.SensorEstimator
    added = {name for name, _ in OPT_IN_FIELDS}
    candidate = [(f.name, f.default) for f in fields(new_estimator.EstimatorConfig)]
    assert [(f.name, f.default) for f in fields(ref_estimator.EstimatorConfig)] == \
        [field for field in candidate if field[0] not in added]
    assert [field for field in candidate if field[0] in added] == list(OPT_IN_FIELDS)


def main_config(config, variant=False):
    """The frozen reference's EstimatorConfig: every candidate field except the opt-in fields (row-end
    position bridge, line-continuation veto), which must be OFF unless the caller compares a variant on
    purpose."""
    values = asdict(config)
    added = {name: values.pop(name) for name, _ in OPT_IN_FIELDS}
    assert variant or (added['row_end_position_confirm_m'] is None and added['row_end_continuation_m'] is None
                       and added['distance_mode'] == 'scan_norm_legacy' and added['row_end_segment_span_m'] is None
                       and added['acquire_behind_m'] is None and added['band_rotate_about_vehicle'] is False
                       and added['row_end_stagger_gap_m'] is None and added['row_end_last_trunk'] is False
                       and added['actuator_prior'] is None and added['corridor_quality'] is False
                       and added['external_corridor_gain'] == 0. and added['row_end_curve_fit'] is False), \
        'reference comparison needs the variants OFF'
    return ref_estimator.EstimatorConfig(**values)


def pair(config=None):
    """(reference, candidate) estimators with the same configuration and initial state."""
    config = config or new_estimator.EstimatorConfig()
    return ref_estimator.SensorEstimator(main_config(config)), new_estimator.SensorEstimator(config)


# --- (a) _first_cell_indices ---------------------------------------------------------------------

def reference_first_cells(cells):
    return np.sort(np.unique(cells, axis=0, return_index=True)[1])


INT32 = np.iinfo(np.int32)


def cell_arrays():
    rng = np.random.default_rng(17)
    yield np.zeros((0, 2), np.int32)
    yield np.array([[3, -4]], np.int32)
    yield np.array([[INT32.min, INT32.max]], np.int32)
    extremes = np.array([[INT32.min, INT32.min], [INT32.max, INT32.max], [INT32.min, INT32.max],
                         [INT32.max, INT32.min], [0, 0], [-1, 0], [0, -1], [INT32.min, 0], [0, INT32.max]], np.int32)
    yield extremes
    yield extremes[rng.integers(0, len(extremes), 200)]                 # duplicated extremes, random order
    yield np.repeat(np.array([[5, 5]], np.int32), 7, axis=0)            # one cell
    yield np.array([[0, 1], [1, 0], [0, 1], [1, 0], [1, 1], [0, 0]], np.int32)
    yield np.array([[INT32.max, 0], [INT32.min, 1], [INT32.max, 1], [INT32.min, 0]], np.int32)   # full x span
    yield np.array([[0, INT32.max], [1, INT32.min], [1, INT32.max], [0, INT32.min]], np.int32)   # full y span
    for size in (1, 2, 3, 10, 100, 1_000, 5_000):
        for low, high in ((-3, 3), (-200, 200), (-50_000, 50_000), (INT32.min, INT32.max)):
            yield rng.integers(low, high, (size, 2), dtype=np.int32, endpoint=True)
    for _ in range(50):                                                 # scan-like cells, many duplicates
        points = rng.normal(0., rng.uniform(.5, 12.), (int(rng.integers(1, 3_000)), 2))
        yield np.floor(points / rng.choice((.1, .12))).astype(np.int32)
    lines = np.column_stack((rng.integers(-5, 5, 400), np.full(400, -7))).astype(np.int32)
    yield lines                                                         # constant y column
    yield lines[:, ::-1].copy()                                         # constant x column


@pytest.mark.parametrize('function', (new_estimator._first_cell_indices, new_scan_corridor._first_cell_indices),
                         ids=('estimator', 'scan_corridor'))
def test_first_cell_indices_match_numpy_unique_rows(function):
    count = 0
    for cells in cell_arrays():
        before = cells.copy()
        expected, actual = reference_first_cells(cells), function(cells)
        assert np.array_equal(cells, before)
        assert expected.dtype == actual.dtype and expected.shape == actual.shape, (expected.dtype, actual.dtype)
        assert expected.tobytes() == actual.tobytes(), f'cells {cells[:4].tolist()}... (n={len(cells)})'
        count += 1
    assert count > 70


# --- (b) _axis_pair and _side_extents ------------------------------------------------------------

def acquisition_points(points, config=new_estimator.EstimatorConfig()):
    """The reference _acquire preprocessing (support range + first point per 0.1 m cell)."""
    pts = points[np.hypot(points[:, 0], points[:, 1]) < config.support_range_m]
    if len(pts):
        _, keep = np.unique(np.floor(pts / .1).astype(np.int32), axis=0, return_index=True)
        pts = pts[np.sort(keep)]
    return pts


COVERAGE = Counter()


def compare_axis_pair(estimators, pts, axis, context):
    reference, candidate = estimators
    before = pts.copy()
    expected, actual = reference._axis_pair(pts, axis), candidate._axis_pair(pts, axis)
    assert np.array_equal(pts, before)
    assert_identical(expected, actual, f'_axis_pair {context}')
    COVERAGE['axis_pair_found' if expected[0] is not None else 'axis_pair_none'] += 1
    return expected


def compare_side_extents(estimators, points, axis, left, right, context):
    reference, candidate = estimators
    before = points.copy()
    expected = reference._side_extents(points, axis, left, right)
    actual = candidate._side_extents(points, axis, left, right)
    assert np.array_equal(points, before)
    assert_identical(expected, actual, f'_side_extents {context}')
    COVERAGE['front' if expected[2] is not None else 'no_front'] += 1
    return expected


@pytest.mark.parametrize('seed', range(0, 50, 3))
def test_axis_pair_and_side_extents_on_world_scans(seed):
    estimators = pair()
    for index, scan in enumerate(world_scans(seed)):
        points = new_estimator.SensorEstimator._points(scan.ranges, scan.angle_min, scan.angle_increment, 30.)
        pts = acquisition_points(points)
        found = []
        for offset in np.arange(-50., 50. + 1e-9, 2.):
            axis = math.radians(float(offset))
            result, _ = compare_axis_pair(estimators, pts, axis, f'seed {seed} pose {index} axis {offset}')
            if result is not None:
                found.append(result)
        for result in found[:6] + [dict(axis=.07, left=1.7, right=-1.8)]:
            axis, left, right = result['axis'], result['left'], result['right']
            for args in ((left, right), (left, None), (None, right), (None, None), (left + .2, right - .15),
                         (left - .4, right + .4)):
                compare_side_extents(estimators, points, axis, *args, f'seed {seed} pose {index} {args}')
            compare_side_extents(estimators, points, axis + .05, left, right, f'seed {seed} pose {index} skew')


def at_axis_zero(rows):
    """Points whose along/cross at axis 0.0 are exactly (x, y): pts @ (1, 0) and pts @ (-0., 1)."""
    pts = np.asarray(rows, dtype=float).reshape(-1, 2)
    assert np.array_equal(pts @ np.array((1., 0.)), pts[:, 0] + 0.)
    assert np.array_equal(pts @ np.array((-0., 1.)), pts[:, 1] + 0.)
    return pts


def line(cross, alongs):
    return [(a, cross) for a in alongs]


EIGHT = 8.     # acquire_along_range_m


def axis_pair_cases():
    span = np.arange(-7.5, 7.6, .75)                                   # one point per 0.75 m along bin
    yield 'plain pair', line(1.75, span) + line(-1.75, span)
    yield 'cross on 0.2 m bin edges', line(.6000000000000001, span) + line(.8, span[::2]) + line(-1.2, span) + \
        line(math.nextafter(-1.2, 0), span[1::2])
    yield 'along exactly +-8 and one ulp beyond', line(1.75, [-EIGHT, EIGHT, math.nextafter(EIGHT, 9.),
                                                               math.nextafter(-EIGHT, -9.)] + list(span)) + \
        line(-1.75, span)
    yield 'centre +-0.25 window edges', line(1.4, span) + line(1.15, span[::3]) + line(1.65, span[1::3]) + \
        line(math.nextafter(1.65, 2.), span[2::3]) + line(-1.4, span)
    yield 'equal coverage in the snap window', line(1.0, span) + line(1.4, span) + line(-1.0, span) + \
        line(-1.6, span)
    yield 'nearest qualifying bin is the closest of several', line(.7, span[:5]) + line(1.3, span) + \
        line(2.5, span) + line(-.9, span) + line(-2.9, span)
    yield 'even and odd band sizes', line(1.75, span) + [(0., 1.78), (.1, 1.72)] + line(-1.75, span[1:])
    yield 'duplicated points and values', line(1.75, list(span) * 3) + line(-1.75, list(span) * 2)
    yield 'inner veto: returns in one along bin', line(2., span) + line(-2., span) + [(1., 0.), (1.2, .3)]
    yield 'inner veto: returns in two along bins', line(2., span) + line(-2., span) + [(1., 0.), (-1., .3)]
    yield 'inner veto: negative along bins only', line(2., span) + line(-2., span) + [(-7.9, 0.), (-6.1, .2)]
    yield 'inner veto: zone edges', line(2., span) + line(-2., span) + [(1., .8), (-1., -.8)]
    yield 'width exactly band_min_width_m', line(.7, span) + line(-.5, span)
    below = -.5
    while .7 - below >= 1.2:                                           # first right offset whose width < 1.2
        below = math.nextafter(below, 0.)
    yield 'width one ulp below band_min_width_m', line(.7, span) + line(below, span)
    yield 'width exactly band_max_width_m', line(4., span) + line(-4., span)
    yield 'left side only', line(1.75, span) + line(.3, span)
    yield 'right side only', line(-1.75, span) + line(-.3, span)
    yield 'coverage one bin short', line(1.75, span[:3]) + line(-1.75, span)
    yield 'too few points', [(0., 1.), (1., 1.), (2., -1.), (3., -1.), (4., 1.), (5., -1.), (6., 1.)]
    yield 'all in one cross bin', line(.61, span) + line(.62, span)
    yield 'signed zeros', line(1.75, span) + line(-1.75, span) + [(-0., -0.), (0., -0.), (-0., 1.75), (-0., -1.75),
                                                                  (1.5, -0.)]


@pytest.mark.parametrize('name,rows', list(axis_pair_cases()), ids=[name for name, _ in axis_pair_cases()])
def test_axis_pair_constructed_cases(name, rows):
    estimators = pair()
    pts = at_axis_zero(rows)
    for axis in (0., math.radians(2.), -math.radians(2.), math.radians(50.)):
        compare_axis_pair(estimators, pts, axis, f'{name} axis {axis}')
    for config in (new_estimator.EstimatorConfig(acquire_min_coverage=2, acquire_veto_coverage=1),
                   new_estimator.EstimatorConfig(acquire_cross_bin_m=.25, acquire_along_bin_m=.5)):
        compare_axis_pair(pair(config), pts, 0., f'{name} {config}')
    result, _ = ref_estimator.SensorEstimator()._axis_pair(pts, 0.)
    expected_found = dict(zip(['inner veto: returns in one along bin', 'inner veto: returns in two along bins',
                               'width exactly band_min_width_m', 'width one ulp below band_min_width_m',
                               'left side only', 'too few points', 'plain pair'],
                              [True, False, True, False, False, False, True]))
    if name in expected_found:
        assert (result is not None) == expected_found[name], result


def side_extents_cases():
    left, right = 1.75, -1.75
    core_half = max(.3, (left - right) / 2 - .6)
    threshold = .5 * 2 * core_half                                     # cross_structure_min_span * 2 * core_half
    start = 2.
    edge = start + .6
    rows = line(left, np.arange(-6., 6.1, .5)) + line(right, np.arange(-6., 6.1, .5))
    slab = [(start, -threshold / 2), (start, threshold / 2), (2.2, .1), (2.3, -.1), (2.4, 0.)]
    yield 'slab end exactly start + 0.6 (6 points)', rows + slab + [(edge, 0.)], left, right
    yield 'slab end one ulp beyond (5 points)', rows + slab + [(math.nextafter(edge, 3.), 0.)], left, right
    high = threshold / 2
    while high - (-threshold / 2) >= threshold:                         # first value whose spread < threshold
        high = math.nextafter(high, 0.)
    narrow = [(start, -threshold / 2), (start, high), (2.2, .1), (2.3, -.1), (2.4, 0.), (edge, 0.)]
    yield 'spread one ulp below the threshold', rows + narrow, left, right
    yield 'duplicated starts and ties', rows + slab * 3 + [(edge, 0.)] * 2 + [(5., -.5), (5., .5)] * 3, left, right
    yield 'core points at along 0.3 (excluded) and beyond 20 m', rows + [(.3, x) for x in (-.5, 0., .5)] * 3 + \
        [(21., x) for x in np.linspace(-1., 1., 8)], left, right
    yield 'two separate structures', rows + slab + [(edge, 0.)] + [(6. + d, x) for d, x in
                                                                    ((0., -.6), (.1, .6), (.2, 0.), (.3, .2),
                                                                     (.4, -.2), (.5, 0.))], left, right
    yield 'five core points only', rows + slab, left, right
    yield 'narrow aisle (core_half floor 0.3)', line(.6, np.arange(-6., 6.1, .5)) + line(-.6, np.arange(-6., 6.1, .5)) + \
        [(3., -.3), (3., .29), (3.1, 0.), (3.2, .1), (3.3, -.1), (3.6, 0.)], .6, -.6
    yield 'one side only', rows + slab + [(edge, 0.)], left, None
    yield 'row end with free space', line(left, np.arange(-6., 3.1, .5)) + line(right, np.arange(-6., 3.6, .5)), left, right
    yield 'signed zeros', rows + slab + [(edge, 0.), (-0., -0.), (0.4, -0.), (-0., 1.75)], left, right


@pytest.mark.parametrize('name,rows,left,right', list(side_extents_cases()),
                         ids=[case[0] for case in side_extents_cases()])
def test_side_extents_constructed_cases(name, rows, left, right):
    estimators = pair()
    points = at_axis_zero(rows)
    result = compare_side_extents(estimators, points, 0., left, right, name)
    for axis in (math.radians(1.), -math.radians(3.)):
        compare_side_extents(estimators, points, axis, left, right, f'{name} axis {axis}')
    compare_side_extents(pair(new_estimator.EstimatorConfig(cross_structure_min_points=3)), points, 0., left, right,
                         f'{name} min points 3')
    expected_front = {'slab end exactly start + 0.6 (6 points)': 2., 'slab end one ulp beyond (5 points)': None,
                      'spread one ulp below the threshold': None, 'five core points only': None}
    if name in expected_front:
        assert result[2] == expected_front[name], result


@pytest.mark.parametrize('block', range(4))
def test_axis_pair_and_side_extents_on_random_clouds(block):
    rng = np.random.default_rng(900 + block)
    estimators = pair()
    for trial in range(60):
        rows = []
        width = rng.uniform(1., 6.)
        heading = rng.normal(0., .3)
        for offset in (width / 2, -width / 2) + tuple(rng.uniform(-9., 9., int(rng.integers(0, 3)))):
            alongs = np.sort(rng.uniform(-9., 9., int(rng.integers(3, 60))))
            crosses = offset + rng.normal(0., rng.choice((0., .03, .15)), len(alongs))
            rows += list(zip(alongs * math.cos(heading) - crosses * math.sin(heading),
                             alongs * math.sin(heading) + crosses * math.cos(heading)))
        rows += [tuple(v) for v in rng.uniform(-12., 12., (int(rng.integers(0, 60)), 2))]
        points = np.asarray(rows)
        if trial % 3 == 0:
            points = np.round(points / .05) * .05                       # exact ties
        pts = acquisition_points(points)
        found = None
        for axis in (heading, 0., heading + rng.normal(0., .05), rng.uniform(-1., 1.)):
            result, _ = compare_axis_pair(estimators, pts, axis, f'random {block}/{trial} axis {axis}')
            found = found or result
        guess = found or dict(axis=heading, left=width / 2, right=-width / 2)
        for left, right in ((guess['left'], guess['right']), (guess['left'] + rng.normal(0., .3), None),
                            (rng.uniform(.3, 4.), -rng.uniform(.3, 4.))):
            compare_side_extents(estimators, points, guess['axis'], left, right, f'random {block}/{trial}')


# --- (c) closed-loop episodes: both estimators fed the same packets ---------------------------------

def _recording(base):
    class Recording(base):
        """Pure pass-through that records the inputs and outputs of the rewritten methods."""

        def _axis_pair(self, pts, axis):
            result = super()._axis_pair(pts, axis)
            self._exact_log.append(('_axis_pair', canon((pts, axis)), canon(result)))
            self._exact_counts['axis_pair_found' if result[0] is not None else 'axis_pair_none'] += 1
            return result

        def _side_extents(self, points, axis, left, right):
            result = super()._side_extents(points, axis, left, right)
            self._exact_log.append(('_side_extents', canon((points, axis, left, right)), canon(result)))
            self._exact_counts['front' if result[2] is not None else 'no_front'] += 1
            return result
    return Recording


RefRecording = _recording(ref_estimator.SensorEstimator)
NewRecording = _recording(new_estimator.SensorEstimator)
PRIVATE = ('_exact_log', '_exact_counts')
# Added by the opt-in side-clearance change (additive only; never read by main behaviour).
SIDE_STATE = ('side_left', 'side_right', 'side_left_x', 'side_right_x')
SIDE_SNAPSHOT = ('side_clearance_left_m', 'side_clearance_right_m', 'side_nearest_x_left_m',
                 'side_nearest_x_right_m', 'rotation_centre_x_m', 'rotation_centre_sd_m', 'scan_count')
# Added by the opt-in row-end position bridge (written only while it is ON; asserted untouched while OFF).
ROW_END_POSITION_STATE = ('end_position_memory', 'end_position_counts', 'end_position_events')
# Added by the opt-in line-continuation veto (written only while it is ON; asserted untouched while OFF).
CONTINUATION_STATE = ('continuation_last', 'continuation_counts', 'continuation_events')
# Added by the design v2 inputs / outputs (None while OFF; asserted untouched in every comparison).
V2_STATE = ('prior', 'quality', 'external')
V2_SNAPSHOT = ('actuator_prior', 'corridor_quality', 'external_corridor')


def state(estimator):
    """Every attribute except the private recording and the additive opt-in ones; the candidate's
    configuration is compared as the reference's view of it (the opt-in fields are checked apart)."""
    values = {key: value for key, value in vars(estimator).items()
              if key not in PRIVATE + SIDE_STATE + ROW_END_POSITION_STATE + CONTINUATION_STATE + V2_STATE}
    if isinstance(values['config'], new_estimator.EstimatorConfig):
        values['config'] = main_config(values['config'], variant=True)
    return canon(values)


def assert_row_end_position_untouched(estimator):
    assert estimator.end_position_memory is None and estimator.end_position_events == []
    assert estimator.end_position_counts == dict.fromkeys(new_estimator.END_POSITION_REASONS, 0)


def assert_v2_untouched(estimator):
    assert estimator.prior is None and estimator.quality is None and estimator.external is None


def assert_continuation_untouched(estimator):
    assert estimator.continuation_last is None and estimator.continuation_events == []
    assert estimator.continuation_counts == dict.fromkeys(new_estimator.CONTINUATION_KEYS, 0)


def main_view(snapshot):
    """The candidate snapshot without the additive side keys (asserted present)."""
    assert all(key in snapshot for key in SIDE_SNAPSHOT), 'side snapshot keys present'
    assert not any(key in snapshot for key in V2_SNAPSHOT), 'design v2 blocks absent while OFF'
    return {key: value for key, value in snapshot.items() if key not in SIDE_SNAPSHOT}


class DualEstimator:
    """Feeds every packet to the reference and the candidate and returns the candidate's outputs
    after checking that the two are bit-identical (state, rewritten-method calls, snapshots)."""

    def __init__(self, config, variant=False):
        self.ref = RefRecording(main_config(config, variant))
        self.new = NewRecording(config)
        for estimator in (self.ref, self.new):
            estimator._exact_log, estimator._exact_counts = [], Counter()
        self.scans = self.imu_samples = self.snapshots = 0
        self.band_events = Counter()
        self.reasons = Counter()
        self.row_end_snapshots = 0
        self._check('initial state')

    def _check(self, context):
        ref_log, new_log = self.ref._exact_log, self.new._exact_log
        if ref_log != new_log:
            index = next((i for i, (a, b) in enumerate(zip(ref_log, new_log)) if a != b), min(len(ref_log), len(new_log)))
            a = ref_log[index] if index < len(ref_log) else None
            b = new_log[index] if index < len(new_log) else None
            pytest.fail(f'{context}: call {index} differs: {None if a is None else a[0]} '
                        f'{first_difference(a, b) if a and b else (len(ref_log), len(new_log))}')
        ref_log.clear()
        new_log.clear()
        expected, actual = state(self.ref), state(self.new)
        if expected != actual:
            pytest.fail(f'{context}: estimator state differs at {first_difference(expected, actual)}')
        if self.new.config.row_end_position_confirm_m is None:
            assert_row_end_position_untouched(self.new)
        if self.new.config.row_end_continuation_m is None:
            assert_continuation_untouched(self.new)
        assert_v2_untouched(self.new)                 # the comparisons run with the design v2 options OFF

    def on_scan(self, stamp, ranges, angle_min, increment, range_max):
        original = np.array(ranges, copy=True)
        mine, theirs = np.array(ranges, copy=True), np.array(ranges, copy=True)
        had_band = self.new.band is not None
        assert self.ref.on_scan(stamp, theirs, angle_min, increment, range_max) is None
        assert self.new.on_scan(stamp, mine, angle_min, increment, range_max) is None
        assert theirs.tobytes() == original.tobytes() == mine.tobytes()
        self.scans += 1
        self._check(f'scan {self.scans} (stamp {stamp})')
        if had_band != (self.new.band is not None):
            self.band_events['acquired' if not had_band else 'lost'] += 1

    def on_imu(self, stamp, gyro, accel):
        assert self.ref.on_imu(stamp, gyro, accel) is None
        assert self.new.on_imu(stamp, gyro, accel) is None
        self.imu_samples += 1
        self._check(f'imu {self.imu_samples} (stamp {stamp})')

    def snapshot(self, now):
        expected, actual = self.ref.snapshot(now), self.new.snapshot(now)
        assert_identical(expected, main_view(actual), f'snapshot {self.snapshots} at {now}')
        self._check(f'after snapshot at {now}')
        self.snapshots += 1
        self.reasons[actual['corridor_reason']] += 1
        self.row_end_snapshots += actual.get('row_end_ahead_m') is not None
        return actual

    def next_corridor(self):
        expected, actual = self.ref.next_corridor(), self.new.next_corridor()
        assert_identical(expected, actual, 'next_corridor')
        return actual

    @property
    def odom_failures(self):
        assert self.ref.odom_failures == self.new.odom_failures
        return self.new.odom_failures

    @property
    def distance(self):
        assert_identical(self.ref.distance, self.new.distance, 'distance')
        return self.new.distance


SHORT = WorldConfig(row_length_m=12.)
EPISODES = {
    # row end + next-corridor evidence in an edge corridor
    'nominal_row_end': EpisodeConfig(name='exact_nominal', mode='hybrid', seed=0, world=SHORT, start_yaw_deg=5.,
                                     max_time_s=40.),
    # cross-aisle wall: _side_extents' cross-structure (front) path; LiDAR latency jitter
    'wall_across': EpisodeConfig(name='exact_wall', mode='continuous', seed=3,
                                 world=replace(SHORT, scenario='wall_across_before_end', seed=3), start_yaw_deg=3.,
                                 lidar_latency_s=.05, lidar_latency_jitter_s=.1, max_time_s=40.),
    # interior staggered rows, 30 deg entry, LiDAR blackout longer than the band timeout (corridor lost and
    # re-acquired around the learnt axis), dropped scans and a blocked sector
    'interior_blackout': EpisodeConfig(name='exact_interior', mode='continuous', seed=5,
                                       world=replace(SHORT, row_count=6, stagger=True, seed=5), corridor_index=2,
                                       start_yaw_deg=30., start_y_m=.2, blackouts=((8., 11., 'lidar'),),
                                       scan_drop_probability=.05, occlusions=((14., 16., 30., 150.),),
                                       max_time_s=40.),
}
EPISODE_TOTALS = Counter()


@pytest.mark.parametrize('name', list(EPISODES))
def test_closed_loop_episode_is_bit_identical(name):
    duals = []

    def factory(config):
        duals.append(DualEstimator(config))
        return duals[-1]
    result = run_episode(EPISODES[name], estimator_factory=factory)
    dual, = duals
    counts = dual.new._exact_counts
    assert counts == dual.ref._exact_counts
    EPISODE_TOTALS['scans'] += dual.scans
    assert dual.scans >= 100 and dual.snapshots >= 200 and dual.imu_samples >= 200, (dual.scans, result['end'])
    assert counts['axis_pair_found'] > 0 and dual.band_events['acquired'] >= 1
    assert dual.reasons['tracked_full_candidate'] > 0, dual.reasons
    if name == 'nominal_row_end':
        assert dual.row_end_snapshots > 0 and result['final_reason'] == 'row_end_candidate'
        assert counts['no_front'] > 0
    if name == 'wall_across':
        assert counts['front'] > 0 and result['final_reason'] == 'front_blocked'
    if name == 'interior_blackout':                                     # lost after the blackout, re-acquired
        assert dual.band_events['lost'] >= 1 and dual.band_events['acquired'] >= 2, dual.band_events


def test_episodes_cover_at_least_300_scans():
    if len(EPISODE_TOTALS) == 0:
        pytest.skip('runs after test_closed_loop_episode_is_bit_identical in the same session')
    assert EPISODE_TOTALS['scans'] >= 300, EPISODE_TOTALS
