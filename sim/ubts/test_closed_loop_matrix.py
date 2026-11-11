"""Scenario-matrix and harness contracts (no episodes are run here).

Judge: completions are position-checked against scene truth; seeds: every
(scenario, seed, purpose) has its own random stream and A/B/C share it; the
result JSON records every scenario's configuration and plant overrides; the
estimator keeps its nominal calibration even when the plant is overridden.
"""
from dataclasses import replace

import pytest

from closed_loop.harness import EpisodeConfig, describe_config, estimator_config, plant_params, stream_seed
from closed_loop.matrix import HOLDOUT_SEEDS, SCENARIOS, SEEDS, episodes, judge
from closed_loop.world import WorldConfig, build_world


def result(**overrides):
    base = dict(final_reason='row_end_candidate', collisions=0, left_corridor=False, final_x_rel_row_end_m=.6,
                wall_gap_m=None, progress_m=17., end='terminal_hold', row_end_first_estimate=(40., .6),
                row_end_last_estimate=(41., .6), last_row_claim='NOT_LAST', last_row_expected='NOT_LAST')
    base.update(overrides)
    return base


def test_row_end_completion_is_position_checked():
    assert judge(result(), 'nominal_open_end')['completed']
    assert not judge(result(final_x_rel_row_end_m=-.8), 'nominal_open_end')['completed']    # early end
    assert not judge(result(final_x_rel_row_end_m=1.7), 'nominal_open_end')['completed']    # overrun
    assert not judge(result(left_corridor=True), 'nominal_open_end')['completed']           # crossed a gap


def test_front_blocked_counts_only_at_the_wall():
    at_wall = result(final_reason='front_blocked', wall_gap_m=.45, final_x_rel_row_end_m=-5.)
    assert judge(at_wall, 'wall_across_before_end')['completed']
    early = result(final_reason='front_blocked', wall_gap_m=12., progress_m=1.2, final_x_rel_row_end_m=-16.)
    assert not judge(early, 'wall_across_before_end')['completed']
    assert not judge(early, 'narrow_headland')['completed']
    # Beyond-rows fence: completion needs the recognised row end; a stop at the fence is only a safe hold.
    fence = result(final_reason='front_blocked', wall_gap_m=.45, final_x_rel_row_end_m=1.9)
    verdict = judge(fence, 'wall_beyond_rows')
    assert not verdict['completed'] and verdict['expected_hold'] and verdict['acceptable']


def test_only_listed_safe_holds_are_acceptable():
    stall = result(final_reason='pivot_stall_abort', final_x_rel_row_end_m=-17.5)
    assert judge(stall, 'pivot_stall_surface')['acceptable']
    assert not judge(stall, 'nominal_open_end')['acceptable'] and judge(stall, 'nominal_open_end')['safe_hold']
    stuck = result(final_reason='drive_no_response', final_x_rel_row_end_m=-13.)
    assert judge(stuck, 'root_stuck_midrow')['acceptable'] and not judge(stuck, 'root_stuck_midrow')['completed']


def test_last_row_claims_are_scored_against_truth():
    assert judge(result(last_row_claim='LAST', last_row_expected='LAST'), 'nominal_open_end')['last_row'] == 'correct'
    assert judge(result(last_row_claim='LAST', last_row_expected='NOT_LAST'), 'nominal_open_end')['last_row'] == 'wrong'
    assert judge(result(last_row_claim='UNKNOWN', last_row_expected='LAST'), 'nominal_open_end')['last_row'] == 'undecided'


def test_corridor_truth_for_interior_last_and_occluded_corridors():
    interior = build_world(WorldConfig(row_count=6)).corridor_truth(2)
    assert interior['next_left'] == 'NOT_LAST' and interior['next_right'] == 'NOT_LAST'
    last = build_world(WorldConfig(row_count=4)).corridor_truth(2)
    assert last['next_left'] == 'LAST'
    hidden = build_world(WorldConfig(scenario='occluded_next_row', occluder_length_m=40.)).corridor_truth(0)
    assert hidden['next_left'] == 'UNKNOWN'
    partial = build_world(WorldConfig(scenario='occluded_next_row')).corridor_truth(0)
    assert partial['next_left'] == 'NOT_LAST'                       # observable before the 6 m barrier
    fence = build_world(WorldConfig(scenario='wall_beyond_rows', headland_depth_m=6.)).corridor_truth(0)
    assert fence['cross_walls_x'] == (26.,) and fence['end_x'] == 20.


def test_random_streams_are_per_scenario_and_shared_across_modes():
    a = EpisodeConfig(name='nominal_open_end', seed=3, mode='hybrid')
    b = replace(a, mode='continuous')
    c = replace(a, name='heading_25deg')
    for purpose in ('plant', 'lidar_noise', 'scan_drop', 'world'):
        assert stream_seed(a, purpose) == stream_seed(b, purpose)       # paired A/B/C comparison
        assert stream_seed(a, purpose) != stream_seed(c, purpose)       # no shared coin flips across scenarios
    assert stream_seed(a, 'plant') != stream_seed(replace(a, seed=4), 'plant')
    worlds = {cfg.world.seed for cfg in episodes(seeds=(3,), names=['nominal_open_end', 'heading_25deg'])}
    assert len(worlds) == 2


def test_overrides_are_recorded_and_the_estimator_keeps_its_nominal_calibration():
    config = [c for c in episodes(seeds=(0,), names=['mount_footprint_mismatch']) if c.mode == 'hybrid'][0]
    record = describe_config(config)
    overrides = record['plant_provenance_overrides']
    assert overrides['lidar_x_in_base_m']['provenance'] == 'SCENARIO_OVERRIDE_OF_SIM_ASSUMED'
    assert overrides['footprint_length_m']['provenance'] == 'SCENARIO_OVERRIDE_OF_CATALOGUE_UNMEASURED'
    assert plant_params(config).lidar_x_in_base_m == pytest.approx(.30)
    assert estimator_config(config).lidar_x_in_base_m == pytest.approx(.18)   # never copied from the plant
    assert estimator_config(config).footprint_length_m == pytest.approx(1.22)


def test_seed_sets_are_disjoint_and_every_scenario_is_runnable():
    assert len(SEEDS) >= 8 and not set(SEEDS) & set(HOLDOUT_SEEDS)
    assert len(episodes(seeds=(0,), modes=('hybrid',))) == len(SCENARIOS)
    interior = [name for name, spec in SCENARIOS.items() if spec.overrides.get('corridor_index', 0) > 0]
    assert len(interior) >= 5
