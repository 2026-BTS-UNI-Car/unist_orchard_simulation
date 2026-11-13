"""Curved row ends of the row estimator (EstimatorConfig.row_end_curve_fit, S1c 2026-11-13): unit tests on synthetic
laser-frame returns (two lines of posts), no closed loop. The option is OFF by default (bit identity with the frozen
body: test_closed_loop_estimator_exact.py). Every number is a test value.

Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_closed_loop_estimator_curve.py
"""
import math

import numpy as np
import pytest

from closed_loop import estimator as E

HALF = 1.75


def row(curvature=0., first=-6., last=12., spacing=1.5, offset=0.):
    """Laser-frame returns of two post lines at +-HALF (+ offset) bending with the curvature (+ = left): three returns
    per post (front face of a 0.12 m trunk)."""
    pts = []
    s = first
    while s <= last + 1e-9:
        for d in (HALF + offset, -HALF + offset):
            for dy in (-.04, 0., .04):
                pts.append((s, d + .5 * curvature * s * s + dy))
        s += spacing
    return np.asarray(pts, dtype=float)


def extents(points, curve_fit, left=HALF, right=-HALF):
    est = E.SensorEstimator(E.EstimatorConfig(row_end_curve_fit=curve_fit))
    ends, free, front = est._side_extents(points, 0., left, right)
    return est, ends, free


@pytest.mark.parametrize('curvature', (1. / 30., -1. / 30., 1. / 60.))
def test_a_curved_row_that_goes_on_is_not_a_row_end(curvature):
    points = row(curvature)
    _, ends, free = extents(points, False)
    if abs(curvature) > 1. / 45.:
        assert all(free)                                  # the straight rule: both lines 'end' a few metres ahead
    est, ends, free = extents(points, True)
    assert not any(free)
    assert all(e > 8. for e in ends)
    assert est.row_end_curve == pytest.approx(curvature / 2., abs=.004)


@pytest.mark.parametrize('curvature', (0., 1. / 30.))
def test_a_real_row_end_stays_a_row_end(curvature):
    points = row(curvature, last=6.)
    _, ends_straight, free_straight = extents(points, False)
    est, ends, free = extents(points, True)
    assert all(free)
    assert all(abs(e - 6.) < .2 for e in ends)
    if curvature == 0.:
        assert ends == ends_straight and free == free_straight
        assert abs(est.row_end_curve) < .002


def test_near_a_row_end_the_curve_is_not_grown_into_structures_beyond_it():
    # the rows end 3 m ahead; two posts stand 9 m ahead, off the straight line extension: the lines do not reach the
    # first reach (4 m - 1.5 m edge), so c = 0 and the straight rule ends both sides at the last trunks
    points = np.vstack([row(0., last=3.), [(9., HALF + .9), (9., -HALF + .9), (9.05, HALF + .9), (9.05, -HALF + .9),
                                           (9.1, HALF + .9), (9.1, -HALF + .9)]])
    est, ends, free = extents(points, True)
    assert abs(est.row_end_curve) < 1e-9                  # only the straight lines up to 3 m were fitted
    assert all(free) and all(abs(e - 3.) < .2 for e in ends)
    _, ends_straight, free_straight = extents(points, False)
    assert ends == ends_straight and free == free_straight


def test_a_gap_stops_the_growth():
    # straight lines to 5 m, then a 4 m gap, then posts bending away (as a curve would): the growth stops at the gap,
    # the far posts never enter the fit
    near = row(0., last=5.)
    far = row(1. / 20., first=9., last=12.)
    est, _, _ = extents(np.vstack([near, far]), True)
    assert abs(est.row_end_curve) < 1e-9


def test_one_line_missing_keeps_the_straight_rule():
    points = row(1. / 30.)
    points = points[points[:, 1] > 0.]
    est = E.SensorEstimator(E.EstimatorConfig(row_end_curve_fit=True))
    est._side_extents(points, 0., HALF, None)
    assert est.row_end_curve == 0.


def test_the_curve_is_bounded_and_the_option_is_checked():
    est, _, _ = extents(row(.2), True)
    assert abs(est.row_end_curve) <= E.EstimatorConfig().row_end_curve_max_1_m / 2. + 1e-12
    for bad in (dict(row_end_curve_reach_m=()), dict(row_end_curve_reach_m=(6., 4.)),
                dict(row_end_curve_behind_m=-1.), dict(row_end_curve_max_1_m=0.),
                dict(row_end_curve_min_points=0), dict(row_end_curve_edge_m=-1.), dict(row_end_curve_max_gap_m=0.)):
        with pytest.raises(ValueError, match='row_end_curve_invalid'):
            E.EstimatorConfig(row_end_curve_fit=True, **bad)
    E.EstimatorConfig(row_end_curve_reach_m=())            # read only while the option is on
    assert not hasattr(E.SensorEstimator(), 'row_end_curve')
    assert math.isclose(E.EstimatorConfig().row_end_curve_max_1_m, 1. / 15.)
