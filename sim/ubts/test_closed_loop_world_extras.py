"""Opt-in scene extras and EXTRA_SCENARIOS plumbing (no episodes are run here).

Empty extras leave the scene, its random draws and the default matrix unchanged
(the default-scene digest is pinned in test_closed_loop_side_clearance.py); the
extras only add geometry used for ray casting and truth metrics.
"""
import math

import pytest

from closed_loop.evaluate import main as evaluate_main
from closed_loop.matrix import EXTRA_SCENARIOS, SCENARIOS, episodes, judge, scenario_spec
from closed_loop.world import WorldConfig, build_world, footprint_clearance


def test_branch_reach_scale_keeps_the_draws_and_scales_only_that_row():
    base = build_world(WorldConfig(seed=11))
    deep = build_world(WorldConfig(seed=11, branch_reach_scale_by_row=((1, 2.5),)))
    assert deep.trunks == base.trunks and len(deep.branches) == len(base.branches)
    assert [b.radius for b in deep.branches] == [b.radius for b in base.branches]   # same draw order
    rows = base.oracle.row_y_m
    reach = {0: [], 1: []}
    for a, b in zip(base.branches, deep.branches):
        fa, fb = base.to_field(a.x, a.y), deep.to_field(b.x, b.y)
        row = min(range(len(rows)), key=lambda r: abs(fa[1] - rows[r]))
        if fb == fa:
            assert row != 1
        else:
            assert row == 1 and fb[0] == pytest.approx(fa[0], abs=1e-9)       # along unchanged
        reach.setdefault(row, []).append(abs(fb[1] - rows[row]))
    assert max(reach[0]) <= .45 + .2 and max(reach[1]) > .45 + .2      # trunk jitter is below 0.2 m


def test_extra_posts_are_hard_ray_cast_and_not_row_trunks():
    plain = build_world(WorldConfig(field_yaw_deg=30.))
    world = build_world(WorldConfig(field_yaw_deg=30., extra_posts=((10., .6, .08),)))
    assert len(world.extras) == 1 and world.trunks == plain.trunks and world.branches == plain.branches
    post = world.extras[0]
    fx, fy, _ = world.to_field(post.x, post.y)
    assert (fx, fy, post.radius) == (pytest.approx(10.), pytest.approx(.6), .08)
    circles = [c for chunk in world.chunks for c in chunk.circles]
    assert circles[-1] == post and len(circles) == sum(len(chunk.circles) for chunk in plain.chunks) + 1
    assert world.corridor_truth(0) == plain.corridor_truth(0)            # truth row end ignores posts
    x, y, yaw = world.pose_in_field(10., 0., 0.)
    hard, branch = footprint_clearance(world, x, y, yaw, 1.22, .70)
    assert hard == pytest.approx(.6 - .08 - .35, abs=1e-9)               # 0.17 m beside the footprint
    assert branch == footprint_clearance(plain, x, y, yaw, 1.22, .70)[1]


def test_extra_scenarios_are_opt_in():
    assert not set(EXTRA_SCENARIOS) & set(SCENARIOS)
    assert len(EXTRA_SCENARIOS) == 3
    default = {c.name for c in episodes(seeds=(0,), modes=('hybrid',))}
    assert default == set(SCENARIOS)
    extra = episodes(seeds=(0, 1), modes=('hybrid', 'hybrid_side'), names=list(EXTRA_SCENARIOS), extra=True)
    assert {c.name for c in extra} == set(EXTRA_SCENARIOS) and len(extra) == 12
    for name in EXTRA_SCENARIOS:
        spec = scenario_spec(name)
        assert spec.expected == ('row_end_candidate',) and spec.acceptable_holds == ()
        verdict = judge(dict(final_reason='side_blocked', collisions=0, left_corridor=False, end='terminal_hold',
                             final_x_rel_row_end_m=-10., wall_gap_m=None, progress_m=7.,
                             row_end_first_estimate=None, last_row_claim='NOT_LAST', last_row_expected='NOT_LAST'),
                        name)
        assert not verdict['acceptable'] and verdict['safe_hold']       # a side stop there is a false stop


@pytest.mark.parametrize('argv', [['--scenarios', 'aisle_2p2m'], ['--modes', 'hybrid', 'bogus'],
                                  ['--pairs', 'hybrid:hybrid_side'], ['--pairs', 'hybrid']])
def test_evaluate_rejects_unknown_scenarios_modes_and_pairs(tmp_path, argv):
    with pytest.raises(SystemExit):
        evaluate_main(['--out', str(tmp_path / 'x.json'), '--seeds', '0'] + argv)
    assert not (tmp_path / 'x.json').exists()


def test_extra_scenario_starts_are_passable():
    """Truth at the start pose (evaluation only): no extra scene starts in contact."""
    from closed_loop.harness import plant_params
    for config in episodes(seeds=tuple(range(8)), modes=('hybrid',), names=list(EXTRA_SCENARIOS), extra=True):
        world = build_world(config.world)
        params = plant_params(config)
        pose = world.pose_in_field(config.start_x_m, config.start_y_m, math.radians(config.start_yaw_deg))
        hard, branch = footprint_clearance(world, *pose, params.footprint_length_m, params.footprint_width_m)
        assert hard > .15 and branch > .15, (config.name, config.seed, hard, branch)
