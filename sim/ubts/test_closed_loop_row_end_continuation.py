"""Opt-in line-continuation veto, v1 rule (row_end_continuation_rule='per_side'; SIM only).

The frozen v1 semantics (cf2ed7f) stay selectable to reproduce that evaluation; the default
rule of row_end_continuation_m is now the v2 pair rule (test_closed_loop_row_end_continuation_v2.py).
Review F1: this per-side rule lets one-sided clutter cancel true row ends.

OFF (None, the default) keeps the original row-end rule bit-identical (also pinned end
to end by test_closed_loop_estimator_exact.py). ON (per_side): a side that the original rule calls
ENDED is not ended when its row line visibly continues beyond the end (returns on the line
extension in >= min_bins distinct along bins between end + the required free gap and end +
L, inside free_check_range_m, before any cross-aisle structure, in a band that widens with
the LiDAR along distance). Only positive evidence counts: an occluded / unreturned (UNKNOWN)
or observed free (ABSENT) extension keeps the original decision, so the variant can only
remove an end and never requires evidence of emptiness.

Scans are ray cast from the origin (LiDAR frame = row frame), trunks r 0.13 m at y = +-1.75 m.
"""
from dataclasses import replace
import json
import math

import numpy as np
import pytest

from closed_loop.estimator import CONTINUATION_KEYS, Band, EstimatorConfig, SensorEstimator
from closed_loop.evaluate import main as evaluate_main
from closed_loop.harness import PURPOSES, estimator_config, run_episode, stream_seed
from closed_loop.matrix import SCENARIOS, episodes
from orchard_perception.scenarios import Segment
from test_closed_loop_estimator import row, scan_of

ON = EstimatorConfig(row_end_continuation_m=12., row_end_continuation_rule='per_side')   # the v1 rule
OFF = EstimatorConfig()
ZERO = dict.fromkeys(CONTINUATION_KEYS, 0)
BEFORE = np.arange(-7.5, 5.1, 2.5)                    # trunks up to 5 m on both lines
ALLEY = np.concatenate((BEFORE, np.arange(12.5, 20.1, 2.5)))   # 7.5 m cross alley, the line resumes at 12.5 m
END = row(1.75, BEFORE) + row(-1.75, BEFORE)          # true row end, open headland beyond
CROSS_ALLEY = row(1.75, ALLEY) + row(-1.75, ALLEY)
LEFT_OCCLUDER = [Segment((4.5, .7), (7.5, .7))]       # inside the aisle: hides the left extension 10.4-17.4 m


def extents(config, circles, segments=()):
    _, ranges, angles, points = scan_of(circles, segments)
    estimator = SensorEstimator(config)
    estimator._scan_ranges, estimator._scan_angles = ranges, angles
    return estimator, estimator._side_extents(points, 0., 1.62, -1.62)


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


# --- OFF: original behaviour ------------------------------------------------------------------------

def test_off_is_the_default_and_reads_no_continuation_setting():
    assert EstimatorConfig().row_end_continuation_m is None
    # OFF: the other continuation settings are not validated (and never read).
    assert EstimatorConfig(row_end_continuation_band_m=-1., row_end_continuation_min_bins=0,
                           row_end_continuation_rule='bogus') is not None


def test_off_takes_a_cross_alley_for_a_row_end_and_leaves_the_variant_state_untouched():
    """The original rule: the resuming line only shortens the free gap (7.3 m >> needed), so both
    sides are 'ended' and two updates publish a false end ~7.5 m before the line resumes."""
    estimator, (ends, free, front) = extents(OFF, CROSS_ALLEY)
    assert front is None and free == [True, True] and all(4.8 < e < 5.2 for e in ends)
    estimator, points = tracked(OFF, CROSS_ALLEY)
    snapshot = full_updates(estimator, points)
    assert snapshot['row_end_ahead_m'] is not None and snapshot['row_end_ahead_m'] < 5.5
    assert estimator.continuation_last is None and estimator.continuation_counts == ZERO
    assert estimator.continuation_events == []


# --- ON: the four cases -----------------------------------------------------------------------------

def test_cross_alley_with_visible_continuation_is_not_an_end():
    estimator, (ends, free, front) = extents(ON, CROSS_ALLEY)
    _, (off_ends, _, off_front) = extents(OFF, CROSS_ALLEY)
    assert free == [False, False]
    assert ends == off_ends and front == off_front     # the variant changes only the ended flags
    assert all(bins >= ON.row_end_continuation_min_bins for bins in estimator.continuation_last)
    estimator, points = tracked(ON, CROSS_ALLEY)
    snapshot = full_updates(estimator, points)
    assert snapshot['row_end_ahead_m'] is None and estimator.end_streak == 0
    assert snapshot['side_end_free'] == [False, False]
    assert estimator.continuation_counts == dict(checked=4, vetoed=4, cancelled=2)
    first = estimator.continuation_events[0]
    assert first['stamp'] == .2 and all(b >= 2 for b in first['bins']) and all(4.8 < e < 5.2 for e in first['ends'])


def test_true_end_with_an_observed_free_headland_stays_an_end():
    estimator, (ends, free, front) = extents(ON, END)
    assert extents(OFF, END)[1] == (ends, free, front)
    assert free == [True, True] and estimator.continuation_last == [0, 0]
    estimator, points = tracked(ON, END)
    snapshot = full_updates(estimator, points)
    assert snapshot['row_end_ahead_m'] is not None and estimator.end_streak == 2
    assert estimator.continuation_counts == dict(checked=4, vetoed=0, cancelled=0)
    assert estimator.continuation_events == []


def test_an_occluded_extension_keeps_the_original_decision():
    """UNKNOWN (occluded) never cancels an end and is never required to be ABSENT: a true end with
    the left extension hidden stays an end (as OFF)."""
    estimator, result = extents(ON, END, LEFT_OCCLUDER)
    assert result == extents(OFF, END, LEFT_OCCLUDER)[1]
    ends, free, front = result
    assert front is None and free == [True, True] and estimator.continuation_last == [0, 0]
    estimator, points = tracked(ON, END, LEFT_OCCLUDER)
    assert full_updates(estimator, points)['row_end_ahead_m'] is not None


def test_a_hidden_continuation_is_not_seen_but_the_visible_side_still_vetoes():
    """Cross alley with the left continuation occluded: the left side stays ended (no positive
    evidence; limitation: a fully hidden continuation falls back to the original rule), the right
    side's visible continuation cancels it, so there is no two-sided end."""
    estimator, (ends, free, front) = extents(ON, CROSS_ALLEY, LEFT_OCCLUDER)
    _, (_, off_free, _) = extents(OFF, CROSS_ALLEY, LEFT_OCCLUDER)
    assert off_free == [True, True] and free == [True, False] and front is None
    assert estimator.continuation_last[0] < 2 <= estimator.continuation_last[1]
    estimator, points = tracked(ON, CROSS_ALLEY, LEFT_OCCLUDER)
    assert full_updates(estimator, points)['row_end_ahead_m'] is None
    assert estimator.continuation_counts == dict(checked=4, vetoed=2, cancelled=2)


def test_one_side_continuing_is_not_an_end_in_both_settings():
    circles = row(1.75, BEFORE) + row(-1.75, np.arange(-7.5, 15.1, 2.5))
    for config in (OFF, ON):
        estimator, (_, free, _) = extents(config, circles)
        assert free == [True, False]                   # right: the original rule already says not ended
    assert estimator.continuation_last == [0, None]    # only ended sides are examined


# --- the guards of _line_continues (direct, exact geometry) -----------------------------------------

def continues(config, along, cross, end=5., needed=2., front=None, far=None, offset=1.6):
    estimator = SensorEstimator(config)
    estimator.continuation_last = [None, None]
    along = np.asarray(along, dtype=float)
    cross = np.asarray(cross, dtype=float) if np.ndim(cross) else np.full(len(along), float(cross))
    far = np.ones(len(along), bool) if far is None else np.asarray(far)
    result = estimator._line_continues(0, along, cross, far, offset, end, needed, front)
    return result, estimator.continuation_last[0]


def test_two_bins_on_the_extension_cancel_and_one_does_not():
    assert continues(ON, [10.1, 10.6], 1.6) == (True, 2)       # zone [7, 17): bins 6 and 7
    assert continues(ON, [10.1, 10.2, 10.3], 1.6) == (False, 1)  # one bin: one object face / stray return
    assert continues(replace(ON, row_end_continuation_min_bins=1), [10.1], 1.6) == (True, 1)
    assert continues(ON, [], ()) == (False, 0)


def test_the_zone_starts_at_end_plus_needed_and_stops_at_end_plus_l():
    assert continues(ON, [6.2, 6.9], 1.6) == (False, 0)        # canopy of the end tree (< end + needed)
    assert continues(ON, [6.9, 7.0, 7.6], 1.6) == (True, 2)    # 7.0 is inside (closed start)
    assert continues(ON, [17.0, 17.4], 1.6) == (False, 0)      # end + L excluded
    assert continues(replace(ON, row_end_continuation_m=5.), [10.1, 10.6], 1.6) == (False, 0)   # stop 10 m


def test_the_band_widens_with_the_along_distance_up_to_its_maximum():
    # 0.6 m off the line at 10 m: inside 0.35 + 0.035 x 10.1 = 0.70 m, outside a fixed 0.35 m band.
    assert continues(ON, [10.1, 10.6], 2.2) == (True, 2)
    assert continues(replace(ON, row_end_continuation_band_slope=0.), [10.1, 10.6], 2.2) == (False, 0)
    assert continues(ON, [7.1, 7.6], 2.2) == (False, 1)        # 7.1 m: band 0.5985 < 0.6 (only 7.6 in)
    assert continues(ON, [15.1, 15.6], 2.45) == (False, 0)     # 0.85 m: above the 0.8 m maximum
    assert continues(ON, [15.1, 15.6], 2.39) == (True, 2)
    assert continues(ON, [10.1, 10.6], 1.0) == (True, 2)       # the band is symmetric about the line


def test_returns_beyond_the_free_check_range_or_a_cross_aisle_structure_do_not_count():
    assert continues(ON, [10.1, 10.6], 1.6, far=[False, False]) == (False, 0)
    assert continues(ON, [10.1, 10.6], 1.6, front=9.) == (False, 0)      # behind a wall at 9 m
    assert continues(ON, [10.1, 10.6], 1.6, front=11.) == (True, 2)      # before it (10.6 < 11 - 0.3)


def test_a_wall_beyond_the_end_keeps_the_original_end():
    wall = [Segment((9., -4.), (9., 4.))]
    estimator, result = extents(ON, END, wall)
    assert result == extents(OFF, END, wall)[1]
    assert result[2] is not None and result[1] == [True, True]


V1 = dict(row_end_continuation_rule='per_side')


@pytest.mark.parametrize('values', [dict(row_end_continuation_m=1.5), dict(row_end_continuation_m=1.),
                                    dict(row_end_continuation_m=20.01), dict(row_end_continuation_m=math.inf),
                                    dict(row_end_continuation_m=math.nan),
                                    dict(row_end_continuation_m=12., row_end_continuation_band_m=0.),
                                    dict(row_end_continuation_m=12., row_end_continuation_band_m=.9),
                                    dict(row_end_continuation_m=12., row_end_continuation_band_max_m=1.2),
                                    dict(row_end_continuation_m=12., row_end_continuation_band_slope=-.01),
                                    dict(row_end_continuation_m=12., row_end_continuation_band_slope=math.nan),
                                    dict(row_end_continuation_m=12., row_end_continuation_bin_m=0., **V1),
                                    dict(row_end_continuation_m=12., row_end_continuation_min_bins=0, **V1),
                                    dict(row_end_continuation_m=12., row_end_continuation_min_bins=2., **V1),
                                    dict(row_end_continuation_m=12., row_end_continuation_min_bins=True, **V1),
                                    dict(row_end_continuation_m=12., row_end_continuation_rule='both')])
def test_invalid_continuation_settings_are_refused(values):
    with pytest.raises(ValueError, match='row_end_continuation_invalid'):
        EstimatorConfig(**values)


def test_valid_continuation_settings():
    for values in (dict(row_end_continuation_m=12., **V1), dict(row_end_continuation_m=20., **V1),
                   dict(row_end_continuation_m=1.51, row_end_continuation_band_slope=0., **V1),
                   dict(row_end_continuation_m=12., row_end_position_confirm_m=.5, **V1)):
        assert EstimatorConfig(**values).row_end_continuation_m == values['row_end_continuation_m']
    # The v1 bin settings are read (validated) only by the per_side rule; the default rule is pair.
    assert EstimatorConfig(row_end_continuation_m=12., row_end_continuation_min_bins=0).row_end_continuation_rule \
        == 'pair'


# --- matrix / evaluate / harness plumbing ----------------------------------------------------------

VARIANT = (('row_end_continuation_m', 12.), ('row_end_continuation_rule', 'per_side'))


def test_episodes_attach_the_variant_without_touching_the_draws():
    base = episodes(seeds=(0, 9), modes=('hybrid', 'continuous'))
    variant = episodes(seeds=(0, 9), modes=('hybrid', 'continuous'), estimator_overrides=VARIANT)
    assert len(base) == len(variant) == 4 * len(SCENARIOS)
    for b, v in zip(base, variant):
        assert replace(v, estimator_overrides=()) == b and v.estimator_overrides == VARIANT
        assert all(stream_seed(b, p) == stream_seed(v, p) for p in PURPOSES)
        assert estimator_config(b) == OFF and estimator_config(v) == ON


@pytest.mark.parametrize('argv', [['--row-end-continuation-m', '1'], ['--row-end-continuation-m', '25'],
                                  ['--row-end-continuation-m', 'inf'], ['--row-end-continuation-m', 'nan'],
                                  ['--row-end-continuation-rule', 'per_side'],
                                  ['--row-end-continuation-m', '12', '--row-end-continuation-rule', 'both']])
def test_evaluate_rejects_invalid_continuation_flags(tmp_path, argv):
    with pytest.raises(SystemExit):
        evaluate_main(['--out', str(tmp_path / 'x.json'), '--seeds', '0', '--scenarios', 'nominal_open_end',
                       '--modes', 'hybrid'] + argv)
    assert not (tmp_path / 'x.json').exists()


def test_evaluate_records_the_variant_and_the_default_run_does_not_change(tmp_path):
    argv = ['--seeds', '0', '--scenarios', 'nominal_open_end', '--modes', 'hybrid', '--workers', '1']
    evaluate_main(['--out', str(tmp_path / 'base.json')] + argv)
    evaluate_main(['--out', str(tmp_path / 'variant.json'), '--row-end-continuation-m', '12',
                   '--row-end-continuation-rule', 'per_side'] + argv)
    base = json.loads((tmp_path / 'base.json').read_text(encoding='utf-8'))
    variant = json.loads((tmp_path / 'variant.json').read_text(encoding='utf-8'))
    assert 'estimator_overrides' not in base and base['estimator']['row_end_continuation_m'] is None
    assert variant['estimator_overrides'] == {'row_end_continuation_m': 12., 'row_end_continuation_rule': 'per_side'}
    assert variant['estimator']['row_end_continuation_m'] == 12.
    assert variant['estimator']['row_end_continuation_band_max_m'] == .8
    assert variant['scenarios']['nominal_open_end']['config']['estimator_overrides'] == \
        [['row_end_continuation_m', 12.], ['row_end_continuation_rule', 'per_side']]
    (b,), (v,) = base['runs'], variant['runs']
    assert 'row_end_continuation' not in b and set(v['row_end_continuation']['counts']) == set(CONTINUATION_KEYS)
    assert 'row_end_continuation_counts' not in base['summary_by_mode']['C_hybrid']
    assert 'row_end_continuation_counts' in variant['summary_by_mode']['C_hybrid']
    if not v['row_end_continuation']['counts']['vetoed']:   # nothing vetoed: the variant run is the base run
        assert {k: x for k, x in v.items() if k not in ('wall_s', 'row_end_continuation')} == \
            {k: x for k, x in b.items() if k != 'wall_s'}


def test_harness_reports_the_variant_only_when_it_is_on():
    config, = episodes(seeds=(0,), modes=('hybrid',), names=['nominal_open_end'])
    short = replace(config, max_time_s=6.)
    assert 'row_end_continuation' not in run_episode(short)
    result = run_episode(replace(short, estimator_overrides=VARIANT))
    assert set(result['row_end_continuation']) == {'counts', 'cancellations'}


# --- closed loop: ON equals the frozen OFF reference except the vetoed ended flags --------------------

from test_closed_loop_estimator_exact import assert_identical, canon, main_config, main_view, ref_estimator  # noqa: E402
from test_closed_loop_estimator_exact import new_estimator, state  # noqa: E402

DIFFERS = ('end_free', 'end_streak')


def _recording(base):
    class Recording(base):
        def _side_extents(self, points, axis, left, right):
            result = super()._side_extents(points, axis, left, right)
            bins = getattr(self, 'continuation_last', None)
            self._log.append((canon((points, axis, left, right)), result, None if bins is None else list(bins)))
            return result
    return Recording


RefRecording = _recording(ref_estimator.SensorEstimator)
NewRecording = _recording(new_estimator.SensorEstimator)


class ContinuationDual:
    """ON candidate against the frozen 999a863 (OFF) estimator on the same packets; the candidate's
    outputs drive the loop. Every _side_extents call must have identical inputs, ends and front; an
    ended flag may differ only where the reference says ended, the candidate says not ended and the
    candidate counted >= min_bins continuation bins for that side. Estimator state may differ only in
    end_free / end_streak, the snapshot only in row_end_ahead_m / side_end_free."""

    def __init__(self, config):
        self.ref = RefRecording(main_config(config, variant=True))
        self.new = NewRecording(config)
        self.ref._log, self.new._log = [], []
        self.calls = self.vetoed_sides = self.compared = 0

    def _check(self, context):
        assert len(self.ref._log) == len(self.new._log), context
        for (ref_in, ref_out, _), (new_in, new_out, bins) in zip(self.ref._log, self.new._log):
            assert ref_in == new_in, context
            assert canon(ref_out[0]) == canon(new_out[0]) and canon(ref_out[2]) == canon(new_out[2]), context
            for side, (a, b) in enumerate(zip(ref_out[1], new_out[1])):
                if a != b:
                    assert a is True and b is False and bins[side] >= self.new.config.row_end_continuation_min_bins
                    self.vetoed_sides += 1
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


def test_on_differs_from_the_frozen_off_estimator_only_in_vetoed_ended_flags():
    """Development seed 5 (gap_cross_alley, hybrid; v1 per_side rule): OFF publishes a false end 14.4 m early
    from 3.2 s on (0.50 m from its stop trigger); ON vetoes it and the first published end is the true one."""
    config, = episodes(seeds=(5,), modes=('hybrid',), names=['gap_cross_alley'])
    duals = []

    def factory(estimator_config_):
        duals.append(ContinuationDual(estimator_config_))
        return duals[-1]
    result = run_episode(replace(config, estimator_overrides=VARIANT), estimator_factory=factory)
    dual, = duals
    assert dual.calls >= 100 and dual.compared >= 300 and dual.vetoed_sides > 0
    assert result['row_end_continuation']['counts']['cancelled'] > 0
    assert result['final_reason'] == 'row_end_candidate' and -.5 <= result['final_x_rel_row_end_m'] <= 1.5
    first = result['row_end_first_estimate']
    assert first is not None and abs(first[1]) <= 1.5          # the first published end is the true end
