"""Opt-in row-end position bridge (EstimatorConfig.row_end_position_confirm_m; SIM only).

OFF (None, the default) keeps the original streak rule bit-identical (also pinned
end to end by test_closed_loop_estimator_exact.py). ON: a two-sided ended update
whose previous two-sided ended update is older than row_end_valid_s continues the
streak only when both side end points agree within the tolerance along and across
the band, the gap is at most row_end_position_max_gap_s, the band object is the
same, no scan match failed in between, the NET LiDAR viewpoint displacement along the
band exceeds the tolerance (not the accumulated odometry path) and no one-sided view in
between contradicted the remembered end of its side. One-sided views never create or extend an end, a
two-sided update without an end resets, and the final freshness rule is unchanged.

Scans are ray cast from a vehicle pose that advances along a fixed world scene,
so a consistent end keeps its world position while its laser range shrinks.
"""
from dataclasses import replace
import json
import math

import numpy as np
import pytest

from closed_loop.estimator import (END_POSITION_REASONS, ROW_END_POSITION_MAX_GAP_FACTOR, Band, EstimatorConfig,
                                   SensorEstimator)
from closed_loop.evaluate import main as evaluate_main
from closed_loop.harness import estimator_config, run_episode, stream_seed, PURPOSES
from closed_loop.matrix import SCENARIOS, episodes
from orchard_perception.scenarios import Circle
from test_closed_loop_estimator import row, scan_of

ROW = np.arange(-7.5, 3.6, 2.5)                       # trunks to 2.5 m: both lines end, open beyond
TRUNKS = row(1.75, ROW) + row(-1.75, ROW)
ON = EstimatorConfig(row_end_position_confirm_m=.5)
OFF = EstimatorConfig()
ZERO = dict.fromkeys(END_POSITION_REASONS, 0)


def tracked(config):
    estimator = SensorEstimator(config)
    estimator.band = Band(0., 0., 3.24, 0., 5)
    estimator.last_scan_stamp = 0.
    return estimator


def view(estimator, trunks, x=0., y=0.):
    """Scan points of the world scene seen from vehicle pose (x, y, 0); the pose is set."""
    _, ranges, angles, points = scan_of([Circle(c.x - x, c.y - y, c.radius) for c in trunks])
    estimator._scan_ranges, estimator._scan_angles = ranges, angles
    estimator.pose = np.array((x, y, 0.))
    return points


def observe(estimator, stamp, trunks=TRUNKS, x=0., y=0., left=1.62, right=-1.62, path=None):
    """One corridor update (two-sided, or one-sided when a side is None) from pose (x, y).

    path = odometry path accumulated since the previous update (estimator.distance). By default the
    straight pose change, as the scan matcher would add it; a larger value models path that does not
    move the viewpoint (scan-match jitter while stopped, the LiDAR arc of a pivot, a detour)."""
    straight = float(np.hypot(x - estimator.pose[0], y - estimator.pose[1]))
    assert path is None or path >= straight, 'a path is never shorter than the net displacement'
    estimator.distance += straight if path is None else path
    points = view(estimator, trunks, x, y)
    assert estimator._apply(stamp, 0., 0., None if left is None else left - y,
                            None if right is None else right - y, points, 'candidate')
    estimator.last_scan_stamp = stamp


def confirmed(estimator, now):
    return estimator.snapshot(now)['row_end_ahead_m'] is not None


def moved_last_trunk(shift):
    """The same rows with the last trunk of both lines moved shift metres along the row."""
    return [Circle(c.x + shift, c.y, c.radius) if c.x == ROW[-1] else c for c in TRUNKS]


# --- OFF: original behaviour ------------------------------------------------------------------------

def test_off_is_the_default_and_keeps_the_original_streak():
    assert EstimatorConfig().row_end_position_confirm_m is None
    estimator = tracked(OFF)
    observe(estimator, .2)
    observe(estimator, 3.2, x=1.)                    # consistent end, but 3.0 s > row_end_valid_s
    assert estimator.end_streak == 1 and not confirmed(estimator, 3.25)
    observe(estimator, 3.4, x=1.1)                   # the original rule still confirms within 2.5 s
    assert estimator.end_streak == 2 and confirmed(estimator, 3.45)
    assert estimator.end_position_memory is None and estimator.end_position_counts == ZERO
    assert estimator.end_position_events == []


def test_off_one_sided_views_leave_the_variant_state_untouched():
    estimator = tracked(OFF)
    observe(estimator, .2)
    observe(estimator, 1., x=.3, right=None)
    observe(estimator, 2., x=.6, trunks=row(1.75, np.arange(-7.5, 12.6, 2.5)) + row(-1.75, ROW), right=None)
    assert estimator.end_streak == 1 and estimator.end_position_memory is None
    assert estimator.end_position_counts == ZERO


# --- ON: bridge and guards ---------------------------------------------------------------------------

def test_consistent_end_positions_across_a_longer_gap_confirm():
    estimator = tracked(ON)
    observe(estimator, .2)
    assert estimator.end_streak == 1 and not confirmed(estimator, .25)
    observe(estimator, 3.2, x=1.)                    # 3.0 s later, 1 m closer: same world end
    assert estimator.end_streak == 2 and confirmed(estimator, 3.25)
    assert estimator.end_position_counts == dict(ZERO, bridged=1)
    event, = estimator.end_position_events
    assert event['gap_s'] == pytest.approx(3.) and event['along_m'] < .1 and event['across_m'] < .1
    estimator.last_scan_stamp = 5.8
    assert not confirmed(estimator, 5.8)             # freshness rule unchanged: 2.6 s after the update


@pytest.mark.parametrize('shift, bridged', [(.3, True), (.7, False), (2.5, False)])
def test_end_positions_must_agree_within_the_tolerance(shift, bridged):
    estimator = tracked(ON)
    observe(estimator, .2)
    observe(estimator, 3.2, trunks=moved_last_trunk(shift), x=1.)
    assert estimator.end_free == [True, True]        # the second view is two-sided ended too
    assert estimator.end_streak == (2 if bridged else 1) and confirmed(estimator, 3.25) == bridged
    assert estimator.end_position_counts == dict(ZERO, **{'bridged' if bridged else 'along': 1})


@pytest.mark.parametrize('gap, bridged', [(5.9, True), (6.2, False)])
def test_gap_above_the_maximum_does_not_confirm(gap, bridged):
    estimator = tracked(ON)
    observe(estimator, .2)
    observe(estimator, .2 + gap, x=1.)
    assert estimator.end_streak == (2 if bridged else 1)
    assert estimator.end_position_counts == dict(ZERO, **{'bridged' if bridged else 'gap': 1})
    shorter = tracked(replace(ON, row_end_position_max_gap_s=4.))
    observe(shorter, .2)
    observe(shorter, 4.4, x=1.)
    assert shorter.end_streak == 1 and shorter.end_position_counts == dict(ZERO, gap=1)


def test_a_two_sided_update_without_an_end_resets_the_streak():
    estimator = tracked(ON)
    observe(estimator, .2)
    continuing = row(1.75, ROW) + row(-1.75, np.arange(-7.5, 15.1, 2.5))
    observe(estimator, 1.5, trunks=continuing, x=.5)  # right line continues: not ended
    assert estimator.end_streak == 0 and estimator.end_position_memory is None
    observe(estimator, 4.6, x=1.)                    # ended again, 3.1 s after the reset
    assert estimator.end_streak == 1 and not confirmed(estimator, 4.65)
    assert estimator.end_position_counts == ZERO     # no previous ended observation: no attempt


def test_a_new_band_starts_without_a_previous_observation():
    results = {}
    for name, config in (('off', OFF), ('on', ON)):
        estimator = tracked(config)
        observe(estimator, .2)
        estimator.band, estimator.candidates = None, []   # band lost
        points = view(estimator, TRUNKS, x=1.)
        for stamp in (3.0, 3.1, 3.2):                     # re-initialised from the same (ended) view
            estimator._initialise(stamp, 0., 0., 1.62, -1.62, points)
        estimator.last_scan_stamp = 3.2
        assert estimator.band is not None and estimator.band.full_updates == 1
        results[name] = (estimator.end_streak, confirmed(estimator, 3.25), estimator.end_position_counts)
    assert results['off'] == (1, False, ZERO)
    assert results['on'] == (1, False, dict(ZERO, no_memory=1))


@pytest.mark.parametrize('travel, bridged', [(0., False), (.45, False), (.6, True)])
def test_the_viewpoint_must_move_more_than_the_tolerance(travel, bridged):
    estimator = tracked(ON)
    observe(estimator, .2)
    observe(estimator, 3.2, x=travel)                # same world end, seen from travel metres further
    assert estimator.end_streak == (2 if bridged else 1) and confirmed(estimator, 3.25) == bridged
    assert estimator.end_position_counts == dict(ZERO, **{'bridged' if bridged else 'travel': 1})


def test_odometry_path_without_net_viewpoint_travel_does_not_count():
    """Review case (19eafed read the accumulated path): an end tied to the viewpoint moves 0.3 m with a
    0.3 m net viewpoint change and agrees trivially; 0.25 m of extra path (scan-match jitter while
    stopped / a pivot arc; the matrix showed 0.09-0.28 m within <= 6 s) must not make it evidence."""
    for extra in ('helper', 'direct'):
        estimator = tracked(ON)
        observe(estimator, .2)
        if extra == 'direct':
            estimator.distance += .25                     # exactly the review's demonstration
            observe(estimator, 3.2, trunks=moved_last_trunk(.3), x=.3)
        else:
            observe(estimator, 3.2, trunks=moved_last_trunk(.3), x=.3, path=.55)
        assert estimator.distance > ON.row_end_position_confirm_m           # path 0.55 m > 0.5 m
        assert estimator.end_free == [True, True]
        assert estimator.end_position_counts == dict(ZERO, travel=1)
        assert estimator.end_streak == 1 and not confirmed(estimator, 3.25)


def test_scan_match_jitter_while_stopped_is_no_travel():
    estimator = tracked(ON)
    observe(estimator, .2)
    observe(estimator, 3.2, x=0., path=.8)           # still: 0.8 m of jitter path, same viewpoint
    assert estimator.end_position_counts == dict(ZERO, travel=1) and not confirmed(estimator, 3.25)


@pytest.mark.parametrize('x, path', [(1., None), (1., 1.4), (.6, .9)])
def test_a_real_forward_move_still_bridges_with_or_without_extra_path(x, path):
    estimator = tracked(ON)
    observe(estimator, .2)
    observe(estimator, 3.2, x=x, path=path)          # same world end, seen from x metres further
    assert estimator.end_position_counts == dict(ZERO, bridged=1)
    assert estimator.end_streak == 2 and confirmed(estimator, 3.25)


def test_a_sideways_viewpoint_change_is_not_along_band_travel():
    """Net euclidean displacement 0.54 m > 0.5 m, but only 0.3 m along the band: refused (an end tied
    to the viewpoint moves with the along-band component only). 0.6 m along with the same sideways
    shift bridges."""
    for x, bridged in ((.3, False), (.6, True)):
        estimator = tracked(ON)
        observe(estimator, .2)
        observe(estimator, 3.2, x=x, y=.45)
        assert math.hypot(x, .45) > ON.row_end_position_confirm_m
        assert estimator.end_position_counts == dict(ZERO, **{'bridged' if bridged else 'travel': 1})
        assert confirmed(estimator, 3.25) == bridged


def test_viewpoint_travel_is_projected_on_the_current_band_axis():
    estimator = SensorEstimator(ON)
    estimator.pose = np.array((1.6, 2.1, .3))
    band = Band(0., 0., 3.24, 0.)
    assert estimator._viewpoint_travel(band, (1., 2.)) == pytest.approx(.6)
    band.heading = math.pi / 2
    assert estimator._viewpoint_travel(band, (1., 2.)) == pytest.approx(.1)
    band.heading = math.pi                           # backwards along the axis counts by magnitude
    assert estimator._viewpoint_travel(band, (1., 2.)) == pytest.approx(.6)


def test_a_failed_scan_match_in_the_gap_blocks_the_bridge():
    estimator = tracked(ON)
    observe(estimator, .2)
    estimator.odom_failures += 1                     # e.g. a dropped/blacked-out scan or a failed ICP
    observe(estimator, 3.2, x=1.)
    assert estimator.end_streak == 1 and not confirmed(estimator, 3.25)
    assert estimator.end_position_counts == dict(ZERO, odometry=1)


def test_end_points_must_also_agree_across_the_band():
    config = replace(ON, row_end_position_confirm_m=.2)
    estimator = tracked(config)
    observe(estimator, .2)
    # The scene appears 0.3 m to the left (lateral odometry drift or another line): the band
    # update is still accepted (offset gate 0.35 m) and the end along the row is unchanged.
    shifted = [Circle(c.x, c.y + .3, c.radius) for c in TRUNKS]
    observe(estimator, 3.2, trunks=shifted, x=1., left=1.92, right=-1.32)
    assert estimator.end_streak == 1 and estimator.end_position_counts == dict(ZERO, lateral=1)
    same = tracked(config)
    observe(same, .2)
    observe(same, 3.2, x=1.)
    assert same.end_position_counts == dict(ZERO, bridged=1)


def test_end_points_are_the_laser_end_points_in_the_odometry_frame():
    estimator = SensorEstimator(ON)
    estimator.pose = np.array((1., 2., .1))
    axis = .2
    (lx, ly), (rx, ry) = estimator._end_points((.1 + axis, 1.62, -1.4), (3., 4.))
    for (x, y), along, cross in (((lx, ly), 3., 1.62), ((rx, ry), 4., -1.4)):
        # laser point on the line (axis frame -> laser frame), then laser -> odometry frame
        px = along * np.cos(axis) - cross * np.sin(axis)
        py = along * np.sin(axis) + cross * np.cos(axis)
        assert x == pytest.approx(1. + px * np.cos(.1) - py * np.sin(.1), abs=1e-12)
        assert y == pytest.approx(2. + px * np.sin(.1) + py * np.cos(.1), abs=1e-12)
    # Differences are taken on the CURRENT band axis: the same displacement reads along or across.
    band = Band(0., 0., 3.24, 0.)
    assert estimator._end_disagreement(band, (1.3, .4), (1., 0.)) == pytest.approx((.3, .4))
    band.heading = np.pi / 2
    assert estimator._end_disagreement(band, (1.3, .4), (1., 0.)) == pytest.approx((.4, .3))


def test_one_sided_views_never_confirm_but_may_keep_the_end():
    estimator = tracked(ON)
    observe(estimator, .2)
    for k, stamp in enumerate((1., 2., 3.)):         # only the left line is matched
        observe(estimator, stamp, x=.3 * (k + 1), right=None)
        assert estimator.end_streak == 1 and not confirmed(estimator, stamp + .05)
    assert estimator.end_position_memory is not None and estimator.end_position_counts == ZERO
    observe(estimator, 3.4, x=1.2)                   # 3.2 s after the last two-sided ended update
    assert estimator.end_streak == 2 and confirmed(estimator, 3.45)
    assert estimator.end_position_counts == dict(ZERO, bridged=1)


def test_a_contradicting_one_sided_view_clears_the_remembered_end():
    estimator = tracked(ON)
    observe(estimator, .2)
    continuing = row(1.75, np.arange(-7.5, 12.6, 2.5)) + row(-1.75, ROW)
    observe(estimator, 2., trunks=continuing, x=.6, right=None)   # the left support goes on
    assert estimator.end_position_memory is None and estimator.end_position_counts == dict(ZERO, contradicted=1)
    observe(estimator, 3.4, x=1.)
    assert estimator.end_streak == 1 and not confirmed(estimator, 3.45)
    assert estimator.end_position_counts == dict(ZERO, contradicted=1, no_memory=1)


@pytest.mark.parametrize('side', (0, 1))
@pytest.mark.parametrize('free, kept', [(True, True), (False, False)])
def test_a_one_sided_view_that_sees_its_side_not_ended_contradicts(monkeypatch, side, free, kept):
    """Same end position, but the per-view rule says the side has NOT ended (support resumes within
    the required free gap): that alone contradicts the remembered end of that side."""
    estimator = tracked(ON)
    observe(estimator, .2)
    estimator.pose = np.array((.6, 0., 0.))
    remembered = estimator.end_position_memory['points'][side]
    ends = [None, None]
    ends[side] = remembered[0] - .6                    # laser along of the remembered end (heading 0)
    flags = [False, False]
    flags[side] = free
    monkeypatch.setattr(estimator, '_side_extents', lambda points, axis, left, right: (list(ends), list(flags), None))
    estimator._one_sided_end_check(estimator.band, 0., 0., remembered[1], side, np.zeros((0, 2)))
    assert (estimator.end_position_memory is not None) == kept
    assert estimator.end_position_counts == (ZERO if kept else dict(ZERO, contradicted=1))


@pytest.mark.parametrize('across, kept', [(.3, True), (.7, False)])
def test_a_one_sided_view_off_its_remembered_line_contradicts(monkeypatch, across, kept):
    estimator = tracked(ON)
    observe(estimator, .2)
    remembered = estimator.end_position_memory['points'][0]
    monkeypatch.setattr(estimator, '_side_extents',
                        lambda points, axis, left, right: ([remembered[0], None], [True, False], None))
    estimator._one_sided_end_check(estimator.band, 0., 0., remembered[1] + across, 0, np.zeros((0, 2)))
    assert (estimator.end_position_memory is not None) == kept


def test_a_one_sided_view_without_support_in_range_says_nothing():
    estimator = tracked(ON)
    observe(estimator, .2)
    far = row(1.75, ROW) + row(-1.75, ROW)
    # Vehicle 13 m further: no left support within the 10 m support range (ends None).
    points = view(estimator, far, x=13.)
    estimator._one_sided_end_check(estimator.band, 0., 0., 1.62, 0, points)
    assert estimator.end_position_memory is not None and estimator.end_position_counts == ZERO


@pytest.mark.parametrize('values', [dict(row_end_position_confirm_m=0.), dict(row_end_position_confirm_m=-.1),
                                    dict(row_end_position_confirm_m=1.5),
                                    dict(row_end_position_confirm_m=math.nan),
                                    dict(row_end_position_confirm_m=math.inf),
                                    dict(row_end_position_confirm_m=.5, row_end_position_max_gap_s=2.5),
                                    dict(row_end_position_confirm_m=.5, row_end_position_max_gap_s=math.inf),
                                    dict(row_end_position_confirm_m=.5, row_end_position_max_gap_s=math.nan),
                                    dict(row_end_position_confirm_m=.5, row_end_position_max_gap_s=10.01),
                                    dict(row_end_position_confirm_m=.5, row_end_position_max_gap_s=12.,
                                         row_end_valid_s=2.9)])
def test_invalid_variant_settings_are_refused(values):
    with pytest.raises(ValueError, match='row_end_position_invalid'):
        EstimatorConfig(**values)


@pytest.mark.parametrize('values', [dict(row_end_position_confirm_m=.5),
                                    dict(row_end_position_confirm_m=.5, row_end_position_max_gap_s=10.),
                                    dict(row_end_position_confirm_m=1., row_end_position_max_gap_s=6.),
                                    dict(row_end_position_confirm_m=.5, row_end_position_max_gap_s=12.,
                                         row_end_valid_s=3.)])
def test_the_maximum_gap_bound_is_four_times_row_end_valid_s(values):
    """Upper bound ROW_END_POSITION_MAX_GAP_FACTOR x row_end_valid_s (10 s by default; the 19eafed
    sensitivity settings 0.5 m/10 s and 1.0 m/6 s stay valid)."""
    config = EstimatorConfig(**values)
    assert ROW_END_POSITION_MAX_GAP_FACTOR == 4.
    assert config.row_end_valid_s < config.row_end_position_max_gap_s <= 4. * config.row_end_valid_s


def test_off_ignores_the_maximum_gap_setting():
    assert EstimatorConfig(row_end_position_max_gap_s=1.).row_end_position_confirm_m is None


# --- matrix / evaluate plumbing -------------------------------------------------------------------------

VARIANT = (('row_end_position_confirm_m', .5),)


def test_episodes_attach_the_variant_without_touching_the_draws():
    base = episodes(seeds=(0, 9), modes=('hybrid', 'continuous'))
    variant = episodes(seeds=(0, 9), modes=('hybrid', 'continuous'), estimator_overrides=VARIANT)
    assert len(base) == len(variant) == 4 * len(SCENARIOS)
    for b, v in zip(base, variant):
        assert b.estimator_overrides == () and v.estimator_overrides == VARIANT
        assert replace(v, estimator_overrides=()) == b
        assert all(stream_seed(b, p) == stream_seed(v, p) for p in PURPOSES)
        assert estimator_config(b) == EstimatorConfig() and estimator_config(v) == ON
        assert estimator_config(v).lidar_x_in_base_m == estimator_config(b).lidar_x_in_base_m


@pytest.mark.parametrize('argv', [['--row-end-position-max-gap-s', '5'],
                                  ['--row-end-position-confirm-m', '0'],
                                  ['--row-end-position-confirm-m', '2'],
                                  ['--row-end-position-confirm-m', '.5', '--row-end-position-max-gap-s', '2'],
                                  ['--row-end-position-confirm-m', '.5', '--row-end-position-max-gap-s', 'inf'],
                                  ['--row-end-position-confirm-m', '.5', '--row-end-position-max-gap-s', 'nan'],
                                  ['--row-end-position-confirm-m', '.5', '--row-end-position-max-gap-s', '10.5'],
                                  ['--row-end-position-confirm-m', 'nan'],
                                  ['--row-end-position-confirm-m', 'inf']])
def test_evaluate_rejects_invalid_variant_flags(tmp_path, argv):
    with pytest.raises(SystemExit):
        evaluate_main(['--out', str(tmp_path / 'x.json'), '--seeds', '0', '--scenarios', 'nominal_open_end',
                       '--modes', 'hybrid'] + argv)
    assert not (tmp_path / 'x.json').exists()


def test_evaluate_records_the_variant_and_the_default_run_does_not_change(tmp_path):
    argv = ['--seeds', '0', '--scenarios', 'nominal_open_end', '--modes', 'hybrid', '--workers', '1']
    evaluate_main(['--out', str(tmp_path / 'base.json')] + argv)
    evaluate_main(['--out', str(tmp_path / 'variant.json'), '--row-end-position-confirm-m', '.5'] + argv)
    base = json.loads((tmp_path / 'base.json').read_text(encoding='utf-8'))
    variant = json.loads((tmp_path / 'variant.json').read_text(encoding='utf-8'))
    assert 'estimator_overrides' not in base and base['estimator']['row_end_position_confirm_m'] is None
    assert variant['estimator_overrides'] == {'row_end_position_confirm_m': .5}
    assert variant['estimator']['row_end_position_confirm_m'] == .5
    assert variant['estimator']['row_end_position_max_gap_s'] == 6.
    assert base['scenarios']['nominal_open_end']['config']['estimator_overrides'] == []
    assert variant['scenarios']['nominal_open_end']['config']['estimator_overrides'] == [['row_end_position_confirm_m', .5]]
    (b,), (v,) = base['runs'], variant['runs']
    assert 'row_end_position' not in b and set(v['row_end_position']['counts']) == set(END_POSITION_REASONS)
    summary = base['summary_by_mode']['C_hybrid']
    assert 'row_end_position_attempts' not in summary
    assert 'row_end_position_attempts' in variant['summary_by_mode']['C_hybrid']
    if not v['row_end_position']['counts']['bridged']:   # no bridge: the variant run is the base run
        assert {k: x for k, x in v.items() if k not in ('wall_s', 'row_end_position')} == \
            {k: x for k, x in b.items() if k != 'wall_s'}


def test_harness_reports_the_variant_only_when_it_is_on():
    config = [c for c in episodes(seeds=(0,), modes=('hybrid',), names=['nominal_open_end'])][0]
    short = replace(config, max_time_s=6.)
    assert 'row_end_position' not in run_episode(short)
    result = run_episode(replace(short, estimator_overrides=VARIANT))
    assert set(result['row_end_position']) == {'counts', 'bridges'}


# --- closed loop: ON equals the frozen OFF reference except the streak --------------------------------

from test_closed_loop_estimator_exact import (EPISODES, DualEstimator, assert_identical, main_view,  # noqa: E402
                                              state)


def _contradiction_check(entry):
    """The opt-in one-sided check is the only _side_extents call with a missing side (every original
    call is a full update or a band initialisation, both sides present)."""
    name, arguments, _ = entry
    return name == '_side_extents' and ('NoneType', None) in arguments[1][2:4]


def _without(canonical, keys):
    drop = {('str', key) for key in keys}
    return canonical[0], tuple(item for item in canonical[1] if item[0] not in drop)


class VariantDual(DualEstimator):
    """ON candidate against the frozen 999a863 (OFF) estimator on the same packets. Before the first
    bridge everything must be bit-identical; after it only end_streak (state) and row_end_ahead_m
    (snapshot) may differ. The candidate's outputs drive the loop."""

    def __init__(self, config):
        self.bridged_at = None
        self.extra_checks = self.compared = self.after_bridge = 0
        super().__init__(config, variant=True)

    def _check(self, context):
        log = self.new._exact_log
        self.extra_checks += sum(map(_contradiction_check, log))
        log[:] = [entry for entry in log if not _contradiction_check(entry)]
        assert self.ref._exact_log == log, context
        self.ref._exact_log.clear()
        log.clear()
        if self.bridged_at is None and self.new.end_position_counts['bridged']:
            self.bridged_at = context
        skip = ('end_streak',) if self.bridged_at is not None else ()
        expected, actual = _without(state(self.ref), skip), _without(state(self.new), skip)
        assert expected == actual, f'{context}: estimator state differs'
        self.compared += 1
        self.after_bridge += self.bridged_at is not None

    def snapshot(self, now):
        expected, actual = self.ref.snapshot(now), self.new.snapshot(now)
        candidate = main_view(actual)
        if self.bridged_at is not None:
            expected = {k: v for k, v in expected.items() if k != 'row_end_ahead_m'}
            candidate = {k: v for k, v in candidate.items() if k != 'row_end_ahead_m'}
        assert_identical(expected, candidate, f'snapshot {self.snapshots} at {now}')
        self._check(f'after snapshot at {now}')
        self.snapshots += 1
        self.row_end_snapshots += actual.get('row_end_ahead_m') is not None
        return actual


VARIANT_EPISODES = dict(EPISODES)
# A development-seed episode in which the bridge fires (found by a mechanism scan of seeds 0-7).
VARIANT_EPISODES['interior_staggered_4'] = [c for c in episodes(seeds=(4,), modes=('hybrid',),
                                                                names=['interior_staggered'])][0]


@pytest.mark.parametrize('name', list(VARIANT_EPISODES))
def test_on_differs_from_the_frozen_off_estimator_only_in_the_streak(name):
    duals = []

    def factory(config):
        duals.append(VariantDual(config))
        return duals[-1]
    result = run_episode(replace(VARIANT_EPISODES[name], estimator_overrides=VARIANT), estimator_factory=factory)
    dual, = duals
    assert dual.scans >= 100 and dual.compared >= 300
    if name == 'interior_staggered_4':
        assert dual.bridged_at is not None and dual.new.end_position_counts['bridged'] >= 1
        assert dual.after_bridge >= 20 and dual.extra_checks > 0    # compared on; one-sided checks ran
        assert result['final_reason'] == 'row_end_candidate'
