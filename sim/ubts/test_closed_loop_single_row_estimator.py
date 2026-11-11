"""Single-row estimator options (opt-in, default OFF = original rules; SIM development, 2026-11-02 DEV round 1).

  acquire_behind_m        the first corridor acquisition of a run uses returns from acquire_behind_m behind the LiDAR
                          to acquire_along_range_m ahead: a headland structure behind a row-entry start no longer
                          vetoes the pair (040927 L6-R8 rev, 042301 L3-R11 fwd: corridor_not_acquired).
  row_end_segment_span_m  a side's support is cut at its first free gap of >= row_end_segment_min_gap_m (and the
                          original needed gap) that reaches ahead of the LiDAR, unless the support beyond is a continuing
                          row on BOTH sides (cross gap): a post / fence / bush on the line extension 7-10 m beyond the row
                          end no longer moves that side's end onto it (035952 L0-R2 rev, 042301 L4-R1 fwd).
  band_rotate_about_vehicle  a one-sided band update turns the band about the vehicle, not the run start (dev round 2).
  row_end_stagger_gap_m   both sides ended, one end > 2 m beyond the other: an isolated structure 4-6 m beyond one row's
                          last trunk is not that row's end (dev round 2, 045553 L3-R2 fwd).
  row_end_last_trunk      a small last trunk that the 97th percentile trims stays the side end (dev round 2).
Scans are ray cast from the origin (LiDAR frame = row frame) with the helpers of test_closed_loop_estimator; trunks r 0.13
m at y = +-1.75 m, offsets +-1.62 m.
"""
from dataclasses import replace

import numpy as np
import pytest

import math

from closed_loop.estimator import Band, EstimatorConfig, SensorEstimator
from orchard_perception.scenarios import Segment
from test_closed_loop_estimator import row, scan_of

OFF = EstimatorConfig()
SEG = EstimatorConfig(row_end_segment_span_m=2.)
BEHIND = EstimatorConfig(acquire_behind_m=2.5)
BEFORE = np.arange(-7.5, 5.1, 2.5)                       # trunks up to the true end at x = 5 m on both lines


def extents(config, circles, segments=()):
    _, ranges, angles, points = scan_of(circles, segments)
    estimator = SensorEstimator(config)
    estimator._scan_ranges, estimator._scan_angles = ranges, angles
    return estimator._side_extents(points, 0., 1.62, -1.62)


def post(x, y, size=.3):
    """A short square structure (post, bin face) centred at (x, y)."""
    h = size / 2
    a, b, c, d = (x - h, y - h), (x + h, y - h), (x + h, y + h), (x - h, y + h)
    return [Segment(a, b), Segment(b, c), Segment(c, d), Segment(d, a)]


def test_both_options_are_off_by_default():
    assert OFF.acquire_behind_m is None and OFF.row_end_segment_span_m is None and OFF.band_rotate_about_vehicle is False
    assert OFF.row_end_stagger_gap_m is None and OFF.row_end_last_trunk is False
    assert OFF.row_end_segment_min_gap_m == 6. and OFF.row_end_segment_min_bins == 4


def test_a_post_on_the_line_extension_beyond_the_end_no_longer_moves_the_end():
    xs = np.arange(-8., 2.1, 2.)                          # the true end 2 m ahead of the LiDAR
    circles = row(1.75, xs) + row(-1.75, xs)
    segments = post(9.5, 1.62)                            # 7.5 m beyond the end, inside the 10 m support range
    ends_off, free_off, _ = extents(OFF, circles, segments)
    ends_seg, free_seg, _ = extents(SEG, circles, segments)
    assert ends_off[0] > 9.                               # original rule: the post is taken for left row support
    assert ends_seg[0] == pytest.approx(ends_off[1], abs=.3) and ends_seg[0] < 2.5
    assert all(free_seg)                                  # both sides ended at the trunks, the post bounds the gap
    assert ends_seg[1] == ends_off[1] and free_seg[1] == free_off[1]   # the right side is unchanged


def test_a_cross_gap_with_both_rows_continuing_is_not_cut():
    xs = np.concatenate((BEFORE, np.arange(12.5, 25.1, 2.5)))   # a 7.5 m gap on both lines, rows resume
    circles = row(1.75, xs) + row(-1.75, xs)
    assert extents(SEG, circles) == extents(OFF, circles)


def test_a_cross_gap_whose_far_trunks_are_beyond_the_range_on_one_line_is_not_cut():
    # dev round 2 (gap2_both): the right line's trunks beyond a 7 m gap lie just beyond the 10 m support range (that
    # side ends there by the original rule), the left line's inside it; both continue as rows up to 20 m: no cut
    left = np.concatenate((np.arange(-8., 2.1, 1.), np.arange(9., 20.1, 1.)))
    right = np.concatenate((np.arange(-8., 2.1, 1.), np.arange(10.5, 20.1, 1.)))
    circles = row(1.75, left) + row(-1.75, right)
    ends_off, free_off, _ = extents(OFF, circles)
    assert not all(free_off)                              # the original rule: the left side continues inside 10 m
    assert extents(SEG, circles) == extents(OFF, circles)


def test_a_one_sided_gap_ends_one_side_only():
    left = np.concatenate((BEFORE, np.arange(12.5, 25.1, 2.5)))  # a gap on the left line only
    right = np.arange(-7.5, 25.1, 2.5)
    _, free, _ = extents(SEG, row(1.75, left) + row(-1.75, right))
    assert free[1] is False or free[1] == False           # noqa: E712  (the right row continues: never a corridor end)
    assert not all(free)


def test_sparse_last_trunks_closer_than_the_minimum_gap_keep_the_original_end():
    xs = np.concatenate((BEFORE, (9.5,)))                # 4.5 m to the last trunk: a sparse row, not a cut
    circles = row(1.75, xs) + row(-1.75, xs)
    assert extents(SEG, circles) == extents(OFF, circles)


CLUTTER = post(-3.2, .2, .5) + post(-5.1, -.3, .5)        # bins / bushes in the headland, 3-5 m behind the LiDAR


def correct(found):
    return (found is not None and abs(found[0]) < .05 and found[1] == pytest.approx(1.62, abs=.15)
            and found[2] == pytest.approx(-1.62, abs=.15))


def test_the_first_acquisition_ignores_headland_clutter_behind_the_start():
    xs = np.arange(-1., 8.1, 1.5)                         # a row entry: trunks from 1 m behind the LiDAR
    circles = row(1.75, xs) + row(-1.75, xs)
    _, _, _, points = scan_of(circles, CLUTTER)
    assert not correct(SensorEstimator(OFF)._acquire(points))   # the clutter vetoes the pair: a skewed pair instead
    assert correct(SensorEstimator(BEHIND)._acquire(points))


def test_reacquisition_after_a_loss_keeps_the_symmetric_window():
    xs = np.arange(-1., 8.1, 1.5)
    circles = row(1.75, xs) + row(-1.75, xs)
    _, _, _, points = scan_of(circles, CLUTTER)
    found = []
    for config in (OFF, BEHIND):
        estimator = SensorEstimator(config)
        estimator.last_band_heading = 0.                  # a band was learnt before: re-acquisition
        found.append(estimator._acquire(points))
    assert found[0] == found[1]


def one_sided_jump(config, distance_from_start=36., axis_deg=2.):
    """Lateral-offset change at the vehicle from ONE one-sided update that sees the left line exactly where predicted
    but at an axis axis_deg off the band (vehicle distance_from_start m from the odometry origin, on the band axis)."""
    estimator = SensorEstimator(config)
    estimator.band = Band(0., 0., 3.4, 0., 5)
    estimator.pose = np.array((distance_from_start, 0., 0.))
    estimator.last_scan_stamp = 0.
    before = estimator.snapshot(0.)['lateral_offset']
    heading = math.radians(axis_deg)
    assert estimator._apply(.2, heading, heading, 1.7, None, np.zeros((0, 2)), 'candidate')
    assert estimator.corridor_reason == 'tracked_one_side_candidate'
    after = estimator.snapshot(.2)['lateral_offset']
    return after - before, estimator.band.heading


def test_a_one_sided_update_turns_the_band_about_the_vehicle_not_the_run_start():
    jump_off, heading_off = one_sided_jump(OFF)
    jump_on, heading_on = one_sided_jump(replace(OFF, band_rotate_about_vehicle=True))
    assert heading_on == heading_off == pytest.approx(math.radians(.2 * 2.))       # the same heading step (g / 2)
    assert abs(jump_off) == pytest.approx(36. * math.sin(math.radians(.4)) * .75, abs=.01)   # ~0.19 m from 0.4 deg
    assert abs(jump_on) < .005
    near_off, _ = one_sided_jump(OFF, distance_from_start=.5)                      # near the start both agree
    assert abs(near_off) < .005


STAGGER = EstimatorConfig(row_end_segment_span_m=2., row_end_stagger_gap_m=4.)


def test_an_isolated_object_beyond_one_row_end_is_not_that_row_s_end():
    # dev round 2 (045553 L3-R2 fwd): both rows end 2 m ahead; a 0.6 m object (bush, bin) on the LEFT line extension
    # 4.9 m beyond the last trunk (inside the 6 m segmentation gap) was the left row's end, the row end moved onto it
    xs = np.arange(-8., 2.1, 1.)
    circles = row(1.75, xs) + row(-1.75, xs) + row(1.6, [7.2], .3)
    ends_seg, free_seg, _ = extents(SEG, circles)
    ends_st, free_st, _ = extents(STAGGER, circles)
    assert ends_seg[0] > 6.5 and all(free_seg)              # segmentation alone: the left end is the object
    assert ends_st[0] == pytest.approx(ends_seg[1], abs=.3) and ends_st[0] < 2.5
    assert ends_st[1] == ends_seg[1] and free_st == free_seg


def test_a_far_side_that_continues_as_a_row_keeps_its_end():
    # a real stagger: the left row goes on for 3.2 m (5 trunks) after a 4.5 m gap at the right row's end (last-trunk
    # ends: both sides ended in this scan, the left far beyond the right)
    left = np.concatenate((np.arange(-8., 2.1, 1.), np.arange(6.5, 9.8, .8)))
    right = np.arange(-8., 2.1, 1.)
    circles = row(1.75, left) + row(-1.75, right)
    ends_seg, free_seg, _ = extents(replace(SEG, row_end_last_trunk=True), circles)
    assert all(free_seg) and ends_seg[0] > 9.
    assert extents(replace(STAGGER, row_end_last_trunk=True), circles) == (ends_seg, free_seg, None)


def test_ends_within_the_window_are_unchanged():
    xs = np.concatenate((BEFORE, np.arange(12.5, 25.1, 2.5)))   # a 7.5 m gap on both lines: both ends at the gap
    circles = row(1.75, xs) + row(-1.75, xs)
    assert extents(STAGGER, circles) == extents(SEG, circles)


def test_a_small_last_trunk_trimmed_by_the_percentile_stays_the_end():
    from closed_loop.estimator import _last_trunk_end
    dense = np.linspace(-8., 2., 200)                      # dense canopy support up to 2 m
    small = np.concatenate((dense, (3.0, 3.02)))           # + a 2-return last trunk 1 m beyond
    p97 = float(np.percentile(small, 97))
    assert p97 < 2.1 and _last_trunk_end(small, p97) == pytest.approx(3.01)
    assert _last_trunk_end(dense, float(np.percentile(dense, 97))) == pytest.approx(np.percentile(dense, 97))
    lone = np.concatenate((dense, (3.0,)))                 # a single stray return is not a trunk
    assert _last_trunk_end(lone, float(np.percentile(lone, 97))) == pytest.approx(np.percentile(lone, 97))
    far = np.concatenate((dense, (6.0, 6.02)))             # more than 2 m beyond the percentile: not this support
    assert _last_trunk_end(far, float(np.percentile(far, 97))) == pytest.approx(np.percentile(far, 97))


@pytest.mark.parametrize('values', [dict(row_end_segment_span_m=0.), dict(row_end_segment_span_m=float('nan')),
                                    dict(row_end_segment_span_m=2., row_end_segment_min_gap_m=1.),
                                    dict(row_end_segment_span_m=2., row_end_segment_min_bins=0),
                                    dict(acquire_behind_m=-.1), dict(acquire_behind_m=8.),
                                    dict(row_end_stagger_gap_m=4.),
                                    dict(row_end_segment_span_m=2., row_end_stagger_gap_m=1.),
                                    dict(row_end_segment_span_m=2., row_end_stagger_gap_m=4., row_end_stagger_window_m=0.)])
def test_invalid_settings_are_refused(values):
    with pytest.raises(ValueError):
        replace(OFF, **values)
