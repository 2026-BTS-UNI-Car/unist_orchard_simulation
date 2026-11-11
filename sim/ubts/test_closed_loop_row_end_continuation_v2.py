"""Opt-in line-continuation veto, v2 pair rule (row_end_continuation_rule='pair', the default rule; SIM only).

Review of v1 (taskE_crossalley/review_code/REVIEW_CODE.txt) and the v2 answers tested here:
  F1 one-sided evidence cancelled true ends -> a two-sided end of the original rule is cancelled only when
     BOTH line extensions continue; a one-sided view or a side the rule did not end is never examined.
  F3 cross structures counted as continuation -> only trunk-sized clusters count (extent over a context as
     large as the accepted object); fences, oblique walls, stubs and bin faces are excluded.
  F4 0.5 m bins (one object = 1-2 bins by phase) -> separated objects are counted (>= object_sep_m apart).
  F2 an aligned next block within ~L is geometrically a cross alley: characterised, not "fixed".
The review's static probes (review_code/static_probe.py) are tests below. Scans are ray cast from the origin
(LiDAR frame = row frame), trunks r 0.13 m at y = +-1.75 m up to x = 5 m (true end), offsets +-1.62 m.
"""
from dataclasses import replace
import json
import math

import numpy as np
import pytest

from closed_loop.estimator import (CONTINUATION_KEYS, CONTINUATION_PAIR_KEYS, Band, EstimatorConfig, SensorEstimator,
                                   _components, continuation_objects)
from closed_loop.evaluate import main as evaluate_main
from closed_loop.harness import PURPOSES, estimator_config, run_episode, stream_seed
from closed_loop.matrix import (EXTRA_GROUPS, EXTRA_SCENARIOS, HEADLAND_SCENARIOS, SCENARIOS, episodes,
                                extra_groups, scenario_pool, scenario_spec)
from closed_loop.world import build_world, footprint_clearance
from orchard_perception.scenarios import Circle, Segment
from test_closed_loop_estimator import row, scan_of

L = 12.                                                           # the recommended (dev-tuned) length
V2 = EstimatorConfig(row_end_continuation_m=L)                    # pair rule, default object settings
V1 = EstimatorConfig(row_end_continuation_m=12., row_end_continuation_rule='per_side')
OFF = EstimatorConfig()
BEFORE = np.arange(-7.5, 5.1, 2.5)                                # trunks up to 5 m on both lines
END = row(1.75, BEFORE) + row(-1.75, BEFORE)                      # true row end, open headland beyond


def alley(gap):
    """Both lines resume gap m after the 5 m trunk (cross alley), trunks every 2.5 m to 25 m."""
    xs = np.concatenate((BEFORE, np.arange(5. + gap, 25.1, 2.5)))
    return row(1.75, xs) + row(-1.75, xs)


CROSS_ALLEY = alley(7.5)
LEFT_OCCLUDER = [Segment((4.5, .7), (7.5, .7))]       # inside the aisle: hides the left extension 10.4-17.4 m


def extents(config, circles, segments=()):
    _, ranges, angles, points = scan_of(circles, segments)
    estimator = SensorEstimator(config)
    estimator._scan_ranges, estimator._scan_angles = ranges, angles
    return estimator, estimator._side_extents(points, 0., 1.62, -1.62)


def two_sided_lost(config, circles, segments=()):
    """True when OFF ends both sides and the variant cancels that two-sided end."""
    off = extents(OFF, circles, segments)[1][1]
    return all(off) and not all(extents(config, circles, segments)[1][1])


def tracked(config, circles, segments=()):
    _, ranges, angles, points = scan_of(circles, segments)
    estimator = SensorEstimator(config)
    estimator._scan_ranges, estimator._scan_angles = ranges, angles
    estimator.band = Band(0., 0., 3.24, 0., 5)
    estimator.last_scan_stamp = 0.
    return estimator, points


def full_updates(estimator, points, stamps=(.2, .4)):
    for stamp in stamps:
        assert estimator._apply(stamp, 0., 0., 1.62, -1.62, points, 'candidate')
        estimator.last_scan_stamp = stamp
    return estimator.snapshot(stamps[-1] + .05)


def segment_square(x0, cy, side=1.2):
    h = side / 2
    a, b, c, d = (x0, cy - h), (x0 + side, cy - h), (x0 + side, cy + h), (x0, cy + h)
    return [Segment(a, b), Segment(b, c), Segment(c, d), Segment(d, a)]


# --- defaults / OFF ------------------------------------------------------------------------------------

def test_the_pair_rule_is_the_default_rule_and_off_stays_the_default():
    assert EstimatorConfig().row_end_continuation_m is None
    assert EstimatorConfig().row_end_continuation_rule == 'pair'
    # OFF: no continuation setting is validated or read, the counters keep the original keys.
    assert EstimatorConfig(row_end_continuation_link_m=-1., row_end_continuation_min_objects=0) is not None
    assert tuple(SensorEstimator().continuation_counts) == CONTINUATION_KEYS
    assert tuple(SensorEstimator(V2).continuation_counts) == CONTINUATION_PAIR_KEYS
    assert tuple(SensorEstimator(V1).continuation_counts) == CONTINUATION_KEYS


def test_off_takes_the_cross_alley_for_an_end_and_the_pair_rule_cancels_it():
    estimator, (ends, free, front) = extents(V2, CROSS_ALLEY)
    _, (off_ends, off_free, off_front) = extents(OFF, CROSS_ALLEY)
    assert off_free == [True, True] and free == [False, False]
    assert ends == off_ends and front == off_front     # only the ended flags change
    assert all(n >= V2.row_end_continuation_min_objects for n in estimator.continuation_last)
    estimator, points = tracked(V2, CROSS_ALLEY)
    snapshot = full_updates(estimator, points)
    assert snapshot['row_end_ahead_m'] is None and estimator.end_streak == 0
    assert snapshot['side_end_free'] == [False, False]
    assert estimator.continuation_counts == dict(checked=4, vetoed=4, cancelled=2, one_sided=0)
    first = estimator.continuation_events[0]
    assert set(first) == {'stamp', 'ends', 'objects'} and min(first['objects']) >= 2


def test_true_end_with_an_observed_free_headland_stays_an_end():
    estimator, result = extents(V2, END)
    assert result == extents(OFF, END)[1] and result[1] == [True, True]
    assert estimator.continuation_last == [0, 0]
    estimator, points = tracked(V2, END)
    assert full_updates(estimator, points)['row_end_ahead_m'] is not None and estimator.end_streak == 2
    assert estimator.continuation_counts == dict(checked=4, vetoed=0, cancelled=0, one_sided=0)


# --- F1: pair semantics -----------------------------------------------------------------------------

def test_a_hidden_continuation_keeps_the_original_end_on_both_sides():
    """Cross alley with the left continuation occluded: v1 cancelled the end with the right side alone; the pair
    rule keeps the original (false) end - the documented cost of not trusting one-sided evidence."""
    estimator, (_, free, _) = extents(V2, CROSS_ALLEY, LEFT_OCCLUDER)
    assert extents(OFF, CROSS_ALLEY, LEFT_OCCLUDER)[1][1] == [True, True] and free == [True, True]
    left, right = estimator.continuation_last
    assert left < V2.row_end_continuation_min_objects <= right
    assert not all(extents(V1, CROSS_ALLEY, LEFT_OCCLUDER)[1][1])       # v1: right side vetoed
    estimator, points = tracked(V2, CROSS_ALLEY, LEFT_OCCLUDER)
    assert full_updates(estimator, points)['row_end_ahead_m'] is not None
    assert estimator.continuation_counts == dict(checked=4, vetoed=0, cancelled=0, one_sided=2)


def test_a_side_the_original_rule_did_not_end_is_never_examined():
    circles = row(1.75, BEFORE) + row(-1.75, np.arange(-7.5, 25.1, 2.5))    # right line never ends
    estimator, (_, free, _) = extents(V2, circles)
    assert free == extents(OFF, circles)[1][1] == [True, False]
    assert estimator.continuation_last == [None, None]


def test_a_one_sided_view_is_never_examined():
    _, ranges, angles, points = scan_of(CROSS_ALLEY)
    for offsets in ((1.62, None), (None, -1.62)):
        estimator = SensorEstimator(V2)
        result = estimator._side_extents(points, 0., *offsets)
        reference = SensorEstimator(OFF)._side_extents(points, 0., *offsets)
        assert result[1] == reference[1] and estimator.continuation_last == [None, None]


def test_v1_vs_v2_on_one_sided_clutter_review_probes_1_2_4_5():
    """Review static probes 1, 2, 4, 5: clutter near ONE line extension. v1 cancels the true end, v2 never."""
    v1_lost = v2_lost = 0
    for radius in (.05, .15, .4):                                        # probe 1: one post, x 8-12 m
        for x in np.arange(8., 12.001, .05):
            circles = END + [Circle(float(x), 1.75, radius)]
            v1_lost += two_sided_lost(V1, circles)
            v2_lost += two_sided_lost(V2, circles)
    assert v1_lost > 20 and v2_lost == 0
    for x in (11., 14.):                                                 # probe 2: stake pair 0.8 m apart
        stakes = END + [Circle(x, 1.75, .05), Circle(x + .8, 1.75, .05)]
        assert two_sided_lost(V1, stakes) and not two_sided_lost(V2, stakes)
    for x in (9., 10., 11.):                                             # probe 4: perpendicular stub
        assert not two_sided_lost(V2, END, [Segment((x, 1.2), (x, 4.))])
    for d in (.3, .5, .6, .7, .9):                                       # probe 5: parallel fence, left only
        fence = [Segment((0., 1.75 + d), (20., 1.75 + d))]
        assert two_sided_lost(V1, END, fence) == (d <= .6)
        assert not two_sided_lost(V2, END, fence)


# --- F3: structures are not trunks -------------------------------------------------------------------

@pytest.mark.parametrize('angle', [20., 30., 45., 60., 70., 80., 90.])
def test_oblique_walls_never_cancel_a_true_end_review_probe_3(angle):
    t = math.radians(angle)
    for centre in (11., 12.):                                            # both extensions inside the zone at 11
        dx, dy = 6. * math.cos(t), 6. * math.sin(t)
        wall = [Segment((centre - dx, -dy), (centre + dx, dy))]
        estimator, (_, free, _) = extents(V2, END, wall)
        assert free == [True, True]
        assert estimator.continuation_last == [0, 0]
    if angle == 45.:                                                     # v1 cancelled it with both sides
        assert two_sided_lost(V1, END, [Segment((12. - 6 * math.cos(t), -6 * math.sin(t)),
                                                (12. + 6 * math.cos(t), 6 * math.sin(t)))])


@pytest.mark.parametrize('d', [.3, .5, .6, .7, .9])
def test_parallel_fences_beside_both_lines_never_cancel_a_true_end(d):
    walls = [Segment((0., 1.75 + d), (20., 1.75 + d)), Segment((0., -1.75 - d), (20., -1.75 - d))]
    estimator, (_, free, _) = extents(V2, END, walls)
    assert free == [True, True] and estimator.continuation_last == [0, 0]
    posts = [Circle(float(x), s * (1.75 + d), .04) for s in (1., -1.) for x in np.arange(6., 20., .15)]
    estimator, (_, free, _) = extents(V2, END + posts)                  # a fence of posts (0.15 m step)
    assert free == [True, True]


def test_bins_beside_both_lines_never_cancel_a_true_end_review_probe_7():
    for cy in (2.6,):
        walls = segment_square(9., cy) + segment_square(9., -cy)
        assert two_sided_lost(V1, END, walls) and not two_sided_lost(V2, END, walls)
        two = walls + segment_square(11.5, cy) + segment_square(11.5, -cy)
        assert not two_sided_lost(V2, END, two)


def test_perpendicular_stubs_over_both_extensions_do_not_count():
    for x in (9., 10., 11.):
        walls = [Segment((x, 1.2), (x, 4.)), Segment((x, -1.2), (x, -4.))]
        estimator, (_, free, _) = extents(V2, END, walls)
        assert free == [True, True] and estimator.continuation_last == [0, 0]


# --- F4: separated objects, not bins -------------------------------------------------------------------

def test_one_object_per_side_never_cancels_whatever_its_size_or_position():
    """v1 counted 0.5 m bins: one object filled 2 bins depending on the bin phase. v2 counts objects."""
    for radius in (.05, .15, .4):
        for x in np.arange(8., 12.001, .1):
            circles = END + [Circle(float(x), 1.75, radius), Circle(float(x), -1.75, radius)]
            estimator, (_, free, _) = extents(V2, circles)
            assert free == extents(OFF, circles)[1][1]      # (near the end the post is row support for OFF)
            assert estimator.continuation_last == [None, None] or max(estimator.continuation_last) <= 1


def test_two_separated_trunk_like_objects_on_both_lines_cancel_ambiguity():
    """Two posts 2.5 m apart on both line extensions look like resuming trunks: cancelled (documented)."""
    for x in (10., 12.):
        circles = END + [Circle(x + d, y, .08) for y in (1.75, -1.75) for d in (0., 2.5)]
        assert two_sided_lost(V2, circles)


def test_the_cross_alley_tree_gap_reach_of_l():
    """Single scan, end 4.91 m (trunk face, no canopy): with L = 12 the zone ends 16.91 m, so two resuming trunks
    (2.5 m apart) fit up to a ~9.5 m tree gap; L = 13 reaches ~10.5 m. (Closed loop the end is the p97 of trunk +
    canopy, 0.5-0.9 m further: L = 12 cancelled every development 10 m-gap near miss, DESIGN_V2.txt.)"""
    for gap, cancelled in ((7.5, True), (9.5, True), (10., False), (12.5, False)):
        assert two_sided_lost(V2, alley(gap)) == cancelled, gap
    longer = replace(V2, row_end_continuation_m=13.)
    assert two_sided_lost(longer, alley(10.)) and not two_sided_lost(longer, alley(11.))


# --- F2: an aligned next block within ~L is a cross alley (characterisation) -------------------------------

@pytest.mark.parametrize('headland, cancelled', [(6., True), (8., True), (9., True), (9.5, True), (10., False),
                                                 (11., False), (12., False), (13., False)])
def test_an_aligned_next_block_is_indistinguishable_from_a_cross_alley(headland, cancelled):
    """Next block on the same lines behind an H m (tree-to-tree) headland: cancelled while two of its trunks fit
    in [end + needed, end + L) - exactly the cross-alley geometry (tree gap H). Here the end is 4.91 m (p97 of the
    last trunk face) and the second trunk's near face is at 5 + H + 2.5 - 0.13 m: cancelled for H < 9.54 m with
    L = 12 m. No geometric rule separates them; L is the operating trade-off (DESIGN_V2.txt, F2)."""
    x0 = 5. + headland
    circles = END + [c for y in (-1.75, 1.75, 5.25) for c in row(y, np.arange(x0, x0 + 10., 2.5))]
    assert all(extents(OFF, circles)[1][1])
    assert two_sided_lost(V2, circles) == cancelled
    assert two_sided_lost(V2, alley(headland)) == cancelled             # the cross alley of the same tree gap


# --- continuation_objects guards (direct, exact geometry) --------------------------------------------------

def objects(config, along, cross, end=5., needed=2., front=None, rng=None, offset=1.6):
    along = np.asarray(along, dtype=float)
    cross = np.asarray(cross, dtype=float) if np.ndim(cross) else np.full(len(along), float(cross))
    rng = np.full(len(along), 10.) if rng is None else np.asarray(rng, dtype=float)
    return continuation_objects(config, along, cross, rng, offset, end, needed, front)


def blob(x, y=1.6, n=5, spread=.2):
    """n returns of one trunk face: across the line at along x (as seen from far ahead)."""
    return [x] * n, list(y + np.linspace(-spread / 2, spread / 2, n))


def joined(*blobs):
    return sum((b[0] for b in blobs), []), sum((b[1] for b in blobs), [])


def test_objects_count_separated_trunks_and_merge_pieces_closer_than_sep():
    assert objects(V2, *joined(blob(10.), blob(12.5))) == 2
    assert objects(V2, *joined(blob(10.), blob(12.5), blob(15.))) == 3
    sep = V2.row_end_continuation_object_sep_m
    assert objects(V2, *joined(blob(10.), blob(10. + sep - .05))) == 1      # canopy pieces of one tree
    assert objects(V2, *joined(blob(10.), blob(10. + sep + .05))) == 2
    assert objects(V2, [], ()) == 0


def test_objects_need_min_points_and_a_zone_return():
    n = V2.row_end_continuation_object_min_points
    assert objects(V2, *joined(blob(10., n=n), blob(12.5, n=n))) == 2
    assert objects(V2, *joined(blob(10., n=n - 1), blob(12.5, n=n))) == 1  # a stray return / branch tip
    # A cluster whose returns are all outside the band (context only) is not an object.
    assert objects(V2, *joined(blob(10., y=1.6 + 1.), blob(12.5))) == 1


def test_the_zone_starts_at_end_plus_needed_and_stops_at_end_plus_l():
    assert objects(V2, *joined(blob(6.9), blob(9.4))) == 1                   # 6.9 < end + needed = 7
    assert objects(V2, *joined(blob(7.0), blob(9.5))) == 2
    stop = 5. + L
    assert objects(V2, *joined(blob(stop - 3.), blob(stop - .01))) == 2
    assert objects(V2, *joined(blob(stop - 3.), blob(stop))) == 1            # end + L excluded


def test_the_band_widens_with_the_along_distance_up_to_its_maximum():
    # Trunk centre 0.6 m off the line at 10/12.5 m: inside 0.35 + 0.035 x along, outside a fixed 0.35 m band.
    assert objects(V2, *joined(blob(10., y=2.2, spread=.1), blob(12.5, y=2.2, spread=.1))) == 2
    flat = replace(V2, row_end_continuation_band_slope=0.)
    assert objects(flat, *joined(blob(10., y=2.2, spread=.1), blob(12.5, y=2.2, spread=.1))) == 0
    # 0.85 m off at 15/17.5 m: above the 0.8 m maximum.
    assert objects(V2, *joined(blob(15., y=2.45, spread=.05), blob(17.5, y=2.45, spread=.05))) == 0
    assert objects(V2, *joined(blob(10., y=1., spread=.1), blob(12.5, y=1., spread=.1))) == 2   # symmetric


def test_returns_beyond_the_free_check_range_or_a_cross_aisle_structure_do_not_count():
    along, cross = joined(blob(10.), blob(12.5))
    assert objects(V2, along, cross, rng=[25.] * len(along)) == 0
    assert objects(V2, along, cross, front=12.) == 1                         # 12.5 >= front - 0.3
    assert objects(V2, along, cross, front=13.) == 2


def test_long_or_wide_clusters_are_structures_not_trunks():
    a_max = V2.row_end_continuation_object_along_m
    c_max = V2.row_end_continuation_object_cross_m
    # A row of returns along the line (fence fragment) longer than the along limit: one structure, no object.
    xs = list(np.arange(9., 9. + a_max + .2, .1))
    assert objects(V2, *joined((xs, [1.6] * len(xs)), blob(13.))) == 1
    xs = list(np.arange(9., 9. + a_max - .2, .1))
    assert objects(V2, *joined((xs, [1.6] * len(xs)), blob(13.))) == 2
    # A cluster crossing the band wider than the cross limit (wall / stub): no object, also when most of it is
    # outside the band (the context is as large as the limit, so the edge does not cut it to trunk size).
    ys = list(1.6 + np.arange(-.3, -.3 + c_max + .2, .05))
    assert objects(V2, *joined(([10.] * len(ys), ys), blob(13.))) == 1
    ys = list(1.6 + np.arange(.6, .6 + c_max + .2, .05))
    assert objects(V2, *joined(([10.] * len(ys), ys), blob(13.))) == 1


def test_a_structure_crossing_a_zone_edge_is_not_cut_to_trunk_size():
    """The context extends object_along_m beyond both zone edges: a fence fragment that enters the zone by only
    0.4 m at its start (end + needed = 7 m) or end (end + L) keeps its full along extent and is no object."""
    a_max = V2.row_end_continuation_object_along_m
    start, stop = 7., 5. + L
    xs = list(np.arange(start - a_max + .1, start + .4, .1))                 # 1.5 m long, 0.4 m inside the zone
    assert np.ptp(xs) > a_max and objects(V2, *joined((xs, [1.6] * len(xs)), blob(10.))) == 1
    xs = list(np.arange(stop - .4, stop + a_max - .1, .1))
    assert np.ptp(xs) > a_max and objects(V2, *joined((xs, [1.6] * len(xs)), blob(10.))) == 1
    short = list(np.arange(start, start + .4, .1))                           # the same piece alone: a trunk face
    assert objects(V2, *joined((short, [1.6] * len(short)), blob(10.))) == 2


def test_the_link_distance_joins_returns_into_one_cluster():
    assert len(set(_components(np.array([[0., 0.], [.3, 0.], [1., 0.]]), .35))) == 2
    assert len(set(_components(np.array([[0., 0.], [.3, 0.], [.6, 0.]]), .35))) == 1
    assert _components(np.zeros((0, 2)), .35).shape == (0,)


# --- configuration ---------------------------------------------------------------------------------------

@pytest.mark.parametrize('values', [dict(row_end_continuation_link_m=0.), dict(row_end_continuation_link_m=math.nan),
                                    dict(row_end_continuation_object_along_m=0.),
                                    dict(row_end_continuation_object_cross_m=-.1),
                                    dict(row_end_continuation_object_sep_m=-.1),
                                    dict(row_end_continuation_object_sep_m=math.inf),
                                    dict(row_end_continuation_object_min_points=0),
                                    dict(row_end_continuation_object_min_points=3.),
                                    dict(row_end_continuation_min_objects=0),
                                    dict(row_end_continuation_min_objects=True),
                                    dict(row_end_continuation_rule='both'),
                                    dict(row_end_continuation_m=1.4), dict(row_end_continuation_m=20.5)])
def test_invalid_pair_settings_are_refused(values):
    with pytest.raises(ValueError, match='row_end_continuation_invalid'):
        EstimatorConfig(**{**dict(row_end_continuation_m=12.), **values})


def test_valid_pair_settings():
    for values in (dict(), dict(row_end_continuation_object_sep_m=0.), dict(row_end_continuation_min_objects=1),
                   dict(row_end_continuation_m=20.), dict(row_end_position_confirm_m=.5)):
        config = EstimatorConfig(**{**dict(row_end_continuation_m=12.), **values})
        assert config.row_end_continuation_rule == 'pair'


# --- headland clutter scenes (review F5): opt-in, truth unchanged -----------------------------------------

def test_headland_scenes_are_an_opt_in_group():
    assert set(EXTRA_GROUPS) == {'side_clearance', 'headland_clutter'}
    assert EXTRA_GROUPS['side_clearance'] is EXTRA_SCENARIOS and len(HEADLAND_SCENARIOS) == 20
    assert not set(HEADLAND_SCENARIOS) & (set(SCENARIOS) | set(EXTRA_SCENARIOS))
    assert extra_groups(False) == () and extra_groups(True) == ('side_clearance',)
    assert extra_groups(('headland_clutter',)) == ('headland_clutter',)
    with pytest.raises(ValueError):
        extra_groups(('bogus',))
    assert set(scenario_pool(False)) == set(SCENARIOS)
    assert set(scenario_pool(True)) == set(SCENARIOS) | set(EXTRA_SCENARIOS)
    assert {c.name for c in episodes(seeds=(0,), modes=('hybrid',))} == set(SCENARIOS)
    assert {c.name for c in episodes(seeds=(0,), modes=('hybrid',), extra=True)} == set(SCENARIOS) | set(EXTRA_SCENARIOS)
    names = {c.name for c in episodes(seeds=(0,), modes=('hybrid',), extra=('headland_clutter',))}
    assert names == set(SCENARIOS) | set(HEADLAND_SCENARIOS)
    for name in HEADLAND_SCENARIOS:
        assert scenario_spec(name) is HEADLAND_SCENARIOS[name] and scenario_spec(name).category == 'headland_clutter'
    with pytest.raises(KeyError):
        scenario_spec('bogus')


def test_headland_scenes_share_the_nominal_draws_and_keep_the_truth():
    nominal, = episodes(seeds=(3,), modes=('hybrid',), names=['nominal_open_end'])
    base = build_world(nominal.world)
    for config in episodes(seeds=(3,), modes=('hybrid',), names=list(HEADLAND_SCENARIOS),
                           extra=('headland_clutter',)):
        assert config.stream_key == 'nominal_open_end' and config.start_yaw_deg == nominal.start_yaw_deg
        assert all(stream_seed(config, p) == stream_seed(nominal, p) for p in PURPOSES)
        world = build_world(config.world)
        assert world.trunks == base.trunks and world.walls == base.walls
        assert world.branches[:len(base.branches)] == base.branches        # extra canopy only appended
        assert world.corridor_truth(0) == base.corridor_truth(0)           # clutter is not a row / scene wall
        assert world.extras or world.extra_walls


def test_extra_walls_and_trees_are_hard_ray_cast_obstacles():
    nominal, = episodes(seeds=(0,), modes=('hybrid',), names=['nominal_open_end'])
    fence, = episodes(seeds=(0,), modes=('hybrid',), names=['headland_fence_both_0p5'], extra=('headland_clutter',))
    world = build_world(fence.world)
    assert len(world.extra_walls) == 2 and world.chunks[-1].segments == world.extra_walls
    x, y, yaw = world.pose_in_field(15., 1.75 + .5 - .35 - .1, 0.)       # footprint edge 0.1 m from the fence
    assert footprint_clearance(world, x, y, yaw, 1.22, .70)[0] == pytest.approx(.1, abs=1e-6)
    block, = episodes(seeds=(0,), modes=('hybrid',), names=['headland_next_block_12m'], extra=('headland_clutter',))
    world = build_world(block.world)
    assert len(world.extras) == 18 and len(world.branches) > len(build_world(nominal.world).branches)
    first = min(world.to_field(c.x, c.y)[0] for c in world.extras)
    assert 31.9 <= first <= 32.1 and world.corridor_truth(0)['end_x'] <= 20.1


def test_headland_starts_are_passable():
    from closed_loop.harness import plant_params
    for config in episodes(seeds=tuple(range(8)), modes=('hybrid',), names=list(HEADLAND_SCENARIOS),
                           extra=('headland_clutter',)):
        world = build_world(config.world)
        params = plant_params(config)
        pose = world.pose_in_field(config.start_x_m, config.start_y_m, math.radians(config.start_yaw_deg))
        hard, branch = footprint_clearance(world, *pose, params.footprint_length_m, params.footprint_width_m)
        assert hard > .15 and branch > .15, (config.name, config.seed, hard, branch)


@pytest.mark.parametrize('argv', [['--extra-scenarios', 'bogus'],
                                  ['--scenarios', 'headland_fence_both_0p5'],
                                  ['--extra-scenarios', '--scenarios', 'headland_fence_both_0p5']])
def test_evaluate_rejects_unknown_groups_and_ungrouped_headland_scenes(tmp_path, argv):
    with pytest.raises(SystemExit):
        evaluate_main(['--out', str(tmp_path / 'x.json'), '--seeds', '0', '--modes', 'hybrid'] + argv)
    assert not (tmp_path / 'x.json').exists()


# --- evaluate / harness plumbing --------------------------------------------------------------------------

VARIANT = (('row_end_continuation_m', L),)


def test_episodes_attach_the_variant_without_touching_the_draws():
    base = episodes(seeds=(0, 9), modes=('hybrid',), extra=('headland_clutter',))
    variant = episodes(seeds=(0, 9), modes=('hybrid',), extra=('headland_clutter',), estimator_overrides=VARIANT)
    for b, v in zip(base, variant):
        assert replace(v, estimator_overrides=()) == b and v.estimator_overrides == VARIANT
        assert all(stream_seed(b, p) == stream_seed(v, p) for p in PURPOSES)
        assert estimator_config(b) == OFF and estimator_config(v) == V2


def test_evaluate_records_the_pair_rule_and_the_headland_group(tmp_path):
    argv = ['--seeds', '0', '--modes', 'hybrid', '--workers', '1', '--extra-scenarios', 'headland_clutter',
            '--scenarios', 'headland_posts_both_9m']
    evaluate_main(['--out', str(tmp_path / 'variant.json'), '--row-end-continuation-m', str(L)] + argv)
    variant = json.loads((tmp_path / 'variant.json').read_text(encoding='utf-8'))
    assert variant['estimator_overrides'] == {'row_end_continuation_m': L}
    assert variant['estimator']['row_end_continuation_rule'] == 'pair'
    assert variant['extra_scenarios'] == sorted(HEADLAND_SCENARIOS)
    assert variant['extra_scenario_groups'] == ['headland_clutter']
    (v,) = variant['runs']
    assert set(v['row_end_continuation']['counts']) == set(CONTINUATION_PAIR_KEYS)
    assert 'row_end_continuation_counts' in variant['summary_by_mode']['C_hybrid']


def test_evaluate_bare_extra_flag_keeps_the_side_clearance_payload(tmp_path):
    argv = ['--seeds', '0', '--modes', 'hybrid', '--workers', '1', '--scenarios', 'midrow_side_post']
    evaluate_main(['--out', str(tmp_path / 'side.json'), '--extra-scenarios'] + argv)
    side = json.loads((tmp_path / 'side.json').read_text(encoding='utf-8'))
    assert side['extra_scenarios'] == sorted(EXTRA_SCENARIOS) and 'extra_scenario_groups' not in side


# --- closed loop: v2 ON equals the frozen OFF reference except cancelled two-sided ends -------------------

from test_closed_loop_estimator_exact import assert_identical, canon, main_config, main_view, ref_estimator  # noqa: E402
from test_closed_loop_estimator_exact import new_estimator, state  # noqa: E402

DIFFERS = ('end_free', 'end_streak')


def _recording(base):
    class Recording(base):
        def _side_extents(self, points, axis, left, right):
            result = super()._side_extents(points, axis, left, right)
            objs = getattr(self, 'continuation_last', None)
            self._log.append((canon((points, axis, left, right)), result, None if objs is None else list(objs)))
            return result
    return Recording


RefRecording = _recording(ref_estimator.SensorEstimator)
NewRecording = _recording(new_estimator.SensorEstimator)


class PairDual:
    """v2 candidate against the frozen 999a863 (OFF) estimator on the same packets; the candidate's outputs drive
    the loop. Every _side_extents call must have identical inputs, ends and front; the ended flags may differ only
    as a whole two-sided end: reference [True, True], candidate [False, False], both sides >= min_objects."""

    def __init__(self, config):
        self.ref = RefRecording(main_config(config, variant=True))
        self.new = NewRecording(config)
        self.ref._log, self.new._log = [], []
        self.calls = self.cancelled = self.compared = 0

    def _check(self, context):
        assert len(self.ref._log) == len(self.new._log), context
        need = self.new.config.row_end_continuation_min_objects
        for (ref_in, ref_out, _), (new_in, new_out, objs) in zip(self.ref._log, self.new._log):
            assert ref_in == new_in, context
            assert canon(ref_out[0]) == canon(new_out[0]) and canon(ref_out[2]) == canon(new_out[2]), context
            if list(ref_out[1]) != list(new_out[1]):
                assert list(ref_out[1]) == [True, True] and list(new_out[1]) == [False, False], context
                assert min(objs) >= need, context
                self.cancelled += 1
            elif objs is not None and objs != [None, None]:
                assert list(ref_out[1]) == [True, True] and min(objs) < need, context
            self.calls += 1
        self.ref._log.clear()
        self.new._log.clear()
        skip = {('str', key) for key in DIFFERS}
        expected = (state(self.ref)[0], tuple(i for i in state(self.ref)[1] if i[0] not in skip))
        actual = (state(self.new)[0], tuple(i for i in state(self.new)[1] if i[0] not in skip))
        assert expected == actual, f'{context}: estimator state differs'
        self.compared += 1

    def on_scan(self, stamp, ranges, angle_min, increment, range_max):
        self.ref.on_scan(stamp, np.array(ranges, copy=True), angle_min, increment, range_max)
        self.new.on_scan(stamp, np.array(ranges, copy=True), angle_min, increment, range_max)
        self._check(f'scan {stamp}')

    def on_imu(self, stamp, gyro, accel):
        self.ref.on_imu(stamp, gyro, accel)
        self.new.on_imu(stamp, gyro, accel)
        self._check(f'imu {stamp}')

    def snapshot(self, now):
        expected, actual = self.ref.snapshot(now), self.new.snapshot(now)
        drop = ('row_end_ahead_m', 'side_end_free')
        assert_identical({k: v for k, v in expected.items() if k not in drop},
                         {k: v for k, v in main_view(actual).items() if k not in drop}, f'snapshot {now}')
        self._check(f'after snapshot {now}')
        return actual

    def next_corridor(self):
        assert self.ref.next_corridor() == self.new.next_corridor()
        return self.new.next_corridor()

    @property
    def odom_failures(self):
        return self.new.odom_failures

    @property
    def distance(self):
        return self.new.distance

    @property
    def continuation_counts(self):
        return self.new.continuation_counts

    @property
    def continuation_events(self):
        return self.new.continuation_events


def test_v2_differs_from_the_frozen_off_estimator_only_in_cancelled_two_sided_ends():
    """Development seed 5 (gap_cross_alley, hybrid): OFF publishes a false end ~14.4 m early (0.50 m from its stop
    trigger); the pair rule cancels those two-sided ends. (Its first published end is a short-lived window-edge end
    ~2.7 m early with only the last trunk beyond it - one object - which v1's bins cancelled; harmless class,
    DESIGN_V2.txt.)"""
    config, = episodes(seeds=(5,), modes=('hybrid',), names=['gap_cross_alley'])
    duals = []

    def factory(estimator_config_):
        duals.append(PairDual(estimator_config_))
        return duals[-1]
    result = run_episode(replace(config, estimator_overrides=VARIANT), estimator_factory=factory)
    dual, = duals
    assert dual.calls >= 100 and dual.compared >= 300 and dual.cancelled > 0
    assert result['row_end_continuation']['counts']['cancelled'] == dual.cancelled
    assert result['final_reason'] == 'row_end_candidate' and -.5 <= result['final_x_rel_row_end_m'] <= 1.5
    first = result['row_end_first_estimate']
    assert first is not None and first[1] > -5.          # the cross-alley end (-14.4 m) is never published
