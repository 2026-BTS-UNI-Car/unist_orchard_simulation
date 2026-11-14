"""Nearer-line hold of the row estimator (EstimatorConfig.band_widen_side_gate_m, fail-investigation 2026-11-14,
decision R90): unit tests of the full / one-sided gate of _apply with a stand-in band, no closed loop. The row-end and
next-corridor bookkeeping of a full update is stubbed out (it is not what these tests look at). The option is OFF by
default (bit identity with the frozen body: test_closed_loop_estimator_exact.py). Every number is a test value.

Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_closed_loop_estimator_widen.py
"""
import math

import numpy as np
import pytest

from closed_loop import estimator as E

WIDTH = 3.1          # learnt band width [m]; the LiDAR at the band centre, heading along the band (laser frame = band)
HALF = WIDTH / 2


def line(offset, along=(0.5, 7.5), step=.25):
    """Laser-frame returns of a row line at cross offset `offset` over an along span."""
    a = np.arange(along[0], along[1] + 1e-9, step)
    return np.column_stack((a, np.full(len(a), offset)))


# both near lines continue ahead (returns 0.5 .. 7.5 m ahead), and a far line 0.45 m behind the right one
NEAR_AHEAD = np.vstack((line(HALF), line(-HALF), line(-HALF - .45)))
# the right near line has ended: its returns only behind the LiDAR, the far line ahead
NEAR_ENDED = np.vstack((line(HALF), line(-HALF, along=(-6., -.5)), line(-HALF - .45)))


def estimator(gate=None):
    est = E.SensorEstimator(E.EstimatorConfig(band_widen_side_gate_m=gate))
    est.band = E.Band(heading=0., center=0., width=WIDTH, stamp=0., full_updates=5)
    est.pose = np.array((0., 0., 0.))
    est._update_ends = lambda *args, **kwargs: None          # row-end / next-corridor bookkeeping: not tested here
    est._next_corridor_votes = lambda *args, **kwargs: None
    est._side_extents = lambda *args, **kwargs: None
    return est


def apply(est, left, right, points=NEAR_AHEAD, stamp=1., heading=0.):
    ok = est._apply(stamp, heading, heading, left, right, points, 'candidate')
    band = est.band
    return ok, est.corridor_reason, round(band.center, 9), round(band.width, 9), round(band.heading, 12)


def test_off_accepts_a_one_line_outward_jump_as_a_full_update():
    est = estimator()
    ok, reason, center, width, _ = apply(est, HALF, -HALF - .45)            # right line 0.45 m outward
    assert ok and reason == 'tracked_full_candidate'
    assert width == pytest.approx(WIDTH + E.EstimatorConfig().band_gain * .45)
    assert center < 0.                                                       # the centre moved toward the jump
    assert not hasattr(est, 'widen_refusals')


def test_on_the_same_jump_is_the_one_sided_update_of_the_line_that_stayed():
    est = estimator(.3)
    ok, reason, center, width, _ = apply(est, HALF + .02, -HALF - .45)
    assert ok and reason == 'tracked_one_side_candidate'
    assert width == WIDTH                                                    # the learnt width is kept
    # one-sided left: centre_obs = left - width / 2 = +0.02, blended by band_center_gain
    assert center == pytest.approx(E.EstimatorConfig().band_center_gain * .02)
    assert est.widen_refusals == 1


def test_on_a_left_jump_keeps_the_right_line():
    est = estimator(.3)
    points = np.vstack((line(HALF), line(-HALF), line(HALF + .45)))
    ok, reason, center, width, _ = apply(est, HALF + .45, -HALF, points)
    assert ok and reason == 'tracked_one_side_candidate'
    assert width == WIDTH and center == pytest.approx(0.)
    assert est.widen_refusals == 1


def test_on_the_jump_is_a_full_update_again_once_the_nearer_line_has_nothing_ahead():
    off, on = estimator(), estimator(.3)
    assert apply(on, HALF, -HALF - .45, NEAR_ENDED) == apply(off, HALF, -HALF - .45, NEAR_ENDED)
    assert on.corridor_reason == 'tracked_full_candidate' and on.widen_refusals == 0
    # a few returns ahead (fewer than line_min_points) do not hold the line
    few = np.vstack((NEAR_ENDED, line(-HALF, along=(1., 2.5), step=.5)))
    assert len(line(-HALF, along=(1., 2.5), step=.5)) < E.EstimatorConfig().line_min_points
    on2, off2 = estimator(.3), estimator()
    assert apply(on2, HALF, -HALF - .45, few) == apply(off2, HALF, -HALF - .45, few)


@pytest.mark.parametrize('left, right', [
    (HALF, -HALF + .45),             # inward move of one line (a nearer structure)
    (HALF + .32, -HALF + .32),       # common shift of both lines (a curve, a lateral error)
    (HALF + .2, -HALF - .2),         # symmetric widening inside the gate
    (HALF, -HALF - .25),             # a one-line move below the gate
    (HALF + .4, -HALF - .4),         # both lines out: the original width gate decides
    (HALF + .35, -HALF + .33),       # the other line is off its prediction too: not a one-line jump
])
def test_on_other_updates_are_unchanged(left, right):
    off, on = estimator(), estimator(.3)
    assert apply(on, left, right) == apply(off, left, right)
    assert on.widen_refusals == 0


def test_on_a_jump_beyond_the_width_gate_is_still_a_one_sided_update():
    off, on = estimator(), estimator(.3)
    # 0.7 m outward: the original width gate refuses the full update, the left line updates alone (both)
    assert apply(on, HALF, -HALF - .7) == apply(off, HALF, -HALF - .7)
    assert on.corridor_reason == 'tracked_one_side_candidate'
    assert on.widen_refusals == 0                                            # counted only when it was a full update


def test_on_a_sequence_of_far_line_observations_does_not_widen_the_band():
    """The S1c round 2 W2 045553 c03 pattern: the right near trees continue ahead and the detector alternates them
    and a structure 0.47 m behind them; OFF the band walks outward, ON it stays at the near line."""
    widths = {}
    for gate in (None, .3):
        est = estimator(gate)
        for k in range(12):
            far = k % 3 != 2
            band = est.band
            right = band.center - band.width / 2 - (.47 if far else 0.)      # relative to the current prediction
            left = band.center + band.width / 2
            points = np.vstack((line(left), line(band.center - band.width / 2), line(right)))
            apply(est, left, right, points, stamp=1. + .3 * k)
        widths[gate] = est.band.width
    assert widths[None] > WIDTH + .8
    assert widths[.3] == pytest.approx(WIDTH)


@pytest.mark.parametrize('bad', (0., -.3, float('inf'), float('nan'), True, '0.3'))
def test_the_gate_is_checked(bad):
    with pytest.raises(ValueError, match='band_widen_side_gate_invalid'):
        E.EstimatorConfig(band_widen_side_gate_m=bad)


def test_the_executor_option_maps_to_the_estimator_for_front_steer_only():
    import dataclasses
    import sys
    from pathlib import Path
    tools = str(Path(__file__).resolve().parents[2] / 'tools')
    if tools not in sys.path:
        sys.path.insert(0, tools)
    from row_executor import config as XC
    base = XC.ExecutorConfig()
    assert base.steer_band_widen_side_gate_m is None                            # OFF by default (R90 candidate)
    assert 'steer_band_widen_side_gate_m' not in base.to_dict()                  # untouched steering block: same digest
    front = dataclasses.replace(base, vehicle_profile=XC.FRONT_STEER)
    assert 'band_widen_side_gate_m' not in front.estimator_overrides()
    on = dataclasses.replace(front, steer_band_widen_side_gate_m=.3).validate()
    assert on.estimator_overrides()['band_widen_side_gate_m'] == .3
    assert math.isclose(.3, E.EstimatorConfig().band_side_gate_m)
    differential = dataclasses.replace(base, steer_band_widen_side_gate_m=.3)
    assert 'band_widen_side_gate_m' not in differential.estimator_overrides()   # the differential history: never
