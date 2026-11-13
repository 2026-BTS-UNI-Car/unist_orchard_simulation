"""Row-end agreement of the row estimator (EstimatorConfig.row_end_agree_m, S1c round 2, decision R53): unit tests of
the streak rule on _update_ends with a stand-in band, no closed loop. The option is OFF by default (bit identity with
the frozen body: test_closed_loop_estimator_exact.py). Every number is a test value.

Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_closed_loop_estimator_agree.py
"""
import types

import pytest

from closed_loop import estimator as E


def band():
    return types.SimpleNamespace(end_left=None, end_right=None, end_stamp=None)


def feed(est, b, updates):
    """updates: (stamp, along0, (end_left, end_right) ahead of the LiDAR or None = not ended); returns the streaks."""
    streaks = []
    for stamp, along0, ends in updates:
        if ends is None:
            extents = ([None, None], [False, False], None)
        else:
            extents = (list(ends), [True, True], None)
        est._update_ends(b, extents, along0, stamp)
        streaks.append(est.end_streak)
    return streaks


# the vehicle drives 0.12 m per 0.3 s update toward an end 3 m ahead (world along 3.0 / 2.6)
STEADY = [(10. + .3 * i, .12 * i, (3. - .12 * i, 2.6 - .12 * i)) for i in range(4)]


def test_off_keeps_the_original_streak_even_when_the_end_jumps():
    est = E.SensorEstimator(E.EstimatorConfig())
    jump = STEADY[:2] + [(10.6, .24, (-.34, -.89))]
    assert feed(est, band(), jump) == [1, 2, 3]
    assert not hasattr(est, 'end_disagreements')


def test_on_counts_agreeing_updates_as_before():
    est = E.SensorEstimator(E.EstimatorConfig(row_end_agree_m=1.))
    assert feed(est, band(), STEADY) == [1, 2, 3, 4]
    assert est.end_disagreements == 0


def test_on_a_jumped_end_restarts_the_streak_and_needs_a_second_agreeing_update():
    est = E.SensorEstimator(E.EstimatorConfig(row_end_agree_m=1.))
    b = band()
    # two steady updates, then one update with the later end at the LiDAR (2.86 m nearer), then the steady end again
    updates = STEADY[:2] + [(10.6, .24, (-.34, -.89)), (10.9, .36, (2.64, 2.24)), (11.2, .48, (2.52, 2.12))]
    assert feed(est, b, updates) == [1, 2, 1, 1, 2]
    assert est.end_disagreements == 2
    assert est.end_streak >= E.EstimatorConfig().row_end_confirm_updates


def test_on_a_move_within_the_distance_continues_and_a_one_sided_update_still_resets():
    est = E.SensorEstimator(E.EstimatorConfig(row_end_agree_m=1.))
    b = band()
    updates = STEADY[:2] + [(10.6, .24, (3.66, 2.0))]          # later end +0.9 m: within 1.0 m
    assert feed(est, b, updates) == [1, 2, 3]
    assert feed(est, b, [(10.9, .36, None), (11.2, .48, (3.4, 2.0))]) == [0, 1]
    assert est.end_disagreements == 0


def test_an_old_previous_end_is_not_compared():
    est = E.SensorEstimator(E.EstimatorConfig(row_end_agree_m=1.))
    b = band()
    late = 10.3 + E.EstimatorConfig().row_end_valid_s + .5    # beyond row_end_valid_s: a new streak anyway
    assert feed(est, b, STEADY[:2] + [(late, .24, (-.34, -.89))]) == [1, 2, 1]
    assert est.end_disagreements == 0


@pytest.mark.parametrize('bad', (0., -1., float('inf'), float('nan'), True, '1'))
def test_the_distance_is_checked(bad):
    with pytest.raises(ValueError, match='row_end_agree_invalid'):
        E.EstimatorConfig(row_end_agree_m=bad)
