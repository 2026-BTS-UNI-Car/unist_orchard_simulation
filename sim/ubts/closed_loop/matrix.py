"""Seeded scenario matrix for the A/B/C controller comparison (SIM only).

Every case starts the vehicle inside a corridor (default: the outermost
corridor of a 3-row block at field x=2.5 m; interior cases use corridor 2 of a
6-row block, with rows on both sides beyond the immediate corridor) and drives
along +x. Outcomes are judged with scene truth AFTER the run; nothing here is
given to the estimator or controller.

Seeds: SEEDS (0-7) are the development set; HOLDOUT_SEEDS (8-11) are a
confirmation set that is not used for tuning.
"""
from dataclasses import dataclass, replace
import math

from .harness import EpisodeConfig, stream_seed
from .world import WorldConfig


MODES = ('continuous', 'stop_turn', 'hybrid')
# The variants are run only on request (--modes); *_side add the opt-in side-clearance rule.
ALL_MODES = MODES + ('hybrid_fallback', 'continuous_side', 'stop_turn_side', 'hybrid_side', 'hybrid_side_bias')
MODE_LABEL = dict(continuous='A_continuous', stop_turn='B_stop_turn', hybrid='C_hybrid',
                  hybrid_fallback='C2_hybrid_fallback', continuous_side='A_side', stop_turn_side='B_side',
                  hybrid_side='C_side', hybrid_side_bias='C_side_bias')
SEEDS = tuple(range(8))
HOLDOUT_SEEDS = (8, 9, 10, 11)

ROW_END = ('row_end_candidate',)
SIX = dict(row_count=6)          # interior block: start in corridor 2 (rows 2|3), two rows beyond each side


@dataclass(frozen=True)
class Scenario:
    overrides: dict
    expected: tuple = ROW_END              # completion reasons (position-checked)
    acceptable_holds: tuple = ()           # safe, expected non-completion outcomes
    description: str = ''
    category: str = 'nominal'
    wall_stop: bool = False                # front_blocked must be at the oracle wall


def _w(**kw):
    return WorldConfig(**kw)


SCENARIOS = {
    # --- nominal / new orchards ------------------------------------------------
    'nominal_open_end': Scenario(dict(start_yaw_deg=5.), description='edge corridor of 3 rows, 3.5 m rows, 2.5 m '
                                 'spacing, branches, open end', category='nominal'),
    'new_rows_narrow_rotated': Scenario(dict(world=_w(row_width_m=2.6, tree_spacing_m=2., field_yaw_deg=30.,
                                                      branch_reach_m=.3), start_yaw_deg=8., start_y_m=.1),
                                        description='2.6 m rows, 2.0 m spacing, field rotated 30 deg (rotation is '
                                        'an invariance check, not new geometry)', category='new_orchard'),
    'new_rows_wide_rotated': Scenario(dict(world=_w(row_width_m=4.5, tree_spacing_m=3.5, field_yaw_deg=-20.,
                                                    branch_reach_m=.6), start_yaw_deg=-6.),
                                      acceptable_holds=('corridor_not_acquired',),
                                      description='4.5 m rows, 3.5 m spacing, wide canopy', category='new_orchard'),
    'interior_nominal': Scenario(dict(world=_w(**SIX), corridor_index=2, start_yaw_deg=5.),
                                 description='INTERIOR corridor of a 6-row block (2 rows beyond each side)',
                                 category='new_orchard'),
    'interior_staggered': Scenario(dict(world=_w(stagger=True, **SIX), corridor_index=2, start_yaw_deg=5.),
                                   description='interior corridor, alternate rows shifted half a spacing '
                                   '(diagonal trunk alignments)', category='new_orchard'),
    'interior_wide_rows': Scenario(dict(world=_w(row_width_m=4.5, tree_spacing_m=3.5, branch_reach_m=.6, **SIX),
                                        corridor_index=2, start_yaw_deg=-6.),
                                   acceptable_holds=('corridor_not_acquired',),
                                   description='interior corridor, 4.5 m rows, 3.5 m spacing', category='new_orchard'),
    'interior_square_grid_aliasing': Scenario(dict(world=_w(row_width_m=3., tree_spacing_m=3., **SIX),
                                                   corridor_index=2, start_yaw_deg=4.),
                                              description='interior corridor, 3.0 x 3.0 m grid: cross-row trunk '
                                              'columns as strong as the rows', category='new_orchard'),
    'interior_heading_30deg': Scenario(dict(world=_w(**SIX), corridor_index=2, start_yaw_deg=30., start_y_m=.2),
                                       description='interior corridor, 30 deg entry error', category='entry_error'),
    'headland_entry': Scenario(dict(start_x_m=-1.5, start_yaw_deg=10.), description='start 1.5 m before the row in '
                               'the headland, 10 deg', category='entry_error'),
    'long_row_50m': Scenario(dict(world=_w(row_length_m=50.), start_yaw_deg=5., max_time_s=260.),
                             description='50 m rows (max time scaled)', category='new_orchard'),
    'thick_trunks': Scenario(dict(world=_w(trunk_radius_m=.25), start_yaw_deg=5.),
                             description='trunk radius 0.25 m', category='new_orchard'),
    'canopy_narrow_aisle': Scenario(dict(world=_w(row_width_m=2.6, tree_spacing_m=2., branch_reach_m=.9),
                                         start_yaw_deg=3.), expected=(),
                                    acceptable_holds=('front_blocked', 'corridor_not_acquired', 'side_blocked'),
                                    description='canopy narrows the aisle below footprint + margin: a safe stop is '
                                    'the only acceptable outcome', category='canopy_gap'),
    # --- entry errors / disturbances ---------------------------------------------
    'heading_25deg': Scenario(dict(start_yaw_deg=25.), description='entry heading error 25 deg', category='entry_error'),
    'heading_40deg_offset': Scenario(dict(start_yaw_deg=40., start_y_m=.3),
                                     description='entry heading error 40 deg, 0.3 m off centre', category='entry_error'),
    'midrow_yaw_disturbance': Scenario(dict(yaw_disturbances=((15., 16., 25.),), start_yaw_deg=3.),
                                       description='25 deg skid (gyro-sensed) at t=15 s in mid-row',
                                       category='entry_error'),
    # --- canopy / gaps -----------------------------------------------------------
    'dense_branches': Scenario(dict(world=_w(branches_per_tree=(4, 7), branch_reach_m=.55), start_yaw_deg=6.),
                               description='dense canopy cross-sections reaching 0.55 m into the aisle',
                               category='canopy_gap'),
    'missing_trunk_gap': Scenario(dict(world=_w(scenario='missing_trunk_gap', missing_probability=.15),
                                       start_yaw_deg=4.), description='3 consecutive trunks missing on one side + '
                                  '15% random gaps', category='canopy_gap'),
    'gap_left_line_at_start': Scenario(dict(world=_w(gaps=((3, .5, 8.5),), **SIX), corridor_index=2, start_yaw_deg=4.),
                                       acceptable_holds=('corridor_not_acquired',),
                                       description='interior corridor; its left line has no trunks from 0.5 to 8.5 m '
                                       '(the next row beyond is visible through the gap)', category='canopy_gap'),
    'gap_cross_alley': Scenario(dict(world=_w(gaps=((0, 7.5, 12.6), (1, 7.5, 12.6))), start_yaw_deg=4.),
                                description='5 m cross alley in both lines mid-row (not a row end)',
                                category='canopy_gap'),
    # --- walls / row end / last row ------------------------------------------------
    'wall_beyond_rows': Scenario(dict(world=_w(scenario='wall_beyond_rows'), start_yaw_deg=4.),
                                 acceptable_holds=('front_blocked',), wall_stop=True,
                                 description='fence 3 m beyond the row end: completion needs the row end '
                                 'recognised; a stop at the fence is safe but not completion', category='wall_row_end'),
    'wall_beyond_6m': Scenario(dict(world=_w(scenario='wall_beyond_rows', headland_depth_m=6.), start_yaw_deg=4.),
                               acceptable_holds=('front_blocked',), wall_stop=True,
                               description='fence 6 m beyond the row end', category='wall_row_end'),
    'wall_beyond_9m': Scenario(dict(world=_w(scenario='wall_beyond_rows', headland_depth_m=9.), start_yaw_deg=4.),
                               acceptable_holds=('front_blocked',), wall_stop=True,
                               description='fence 9 m beyond the row end', category='wall_row_end'),
    'wall_across_before_end': Scenario(dict(world=_w(scenario='wall_across_before_end'), start_yaw_deg=4.),
                                       expected=('front_blocked',), wall_stop=True,
                                       description='wall across the aisle 4 m before the row end (must not be a row '
                                       'end)', category='wall_row_end'),
    'narrow_headland': Scenario(dict(world=_w(scenario='narrow_headland'), start_yaw_deg=4.),
                                expected=('front_blocked', 'row_end_candidate'), wall_stop=True,
                                description='wall 0.6 m beyond the row end', category='wall_row_end'),
    'occluded_next_row': Scenario(dict(world=_w(scenario='occluded_next_row'), start_yaw_deg=4.),
                                  description='opaque 6 m side barrier near the end hides part of the next corridor',
                                  category='last_row'),
    'occluded_next_row_full': Scenario(dict(world=_w(scenario='occluded_next_row', occluder_length_m=40.),
                                            start_yaw_deg=4.),
                                       description='full-length side barrier: the next corridor is never observable '
                                       '(correct last-row answer UNKNOWN)', category='last_row'),
    'last_corridor_open_side': Scenario(dict(world=_w(row_count=4), corridor_index=2, start_yaw_deg=4.),
                                        description='last corridor of a 4-row block: open field on the next-row side '
                                        '(correct last-row answer LAST)', category='last_row'),
    # --- sensors -----------------------------------------------------------------------
    'sensor_sector_occlusion': Scenario(dict(occlusions=((14., 17., 30., 150.),), start_yaw_deg=4.),
                                        description='left 120 deg LiDAR sector blocked for 3 s', category='sensor'),
    'lidar_latency_100ms': Scenario(dict(lidar_latency_s=.1, start_yaw_deg=12.),
                                    description='LiDAR delivery latency 100 ms', category='sensor'),
    'lidar_latency_250ms': Scenario(dict(lidar_latency_s=.25, imu_latency_s=.05, start_yaw_deg=12.),
                                    description='LiDAR 250 ms, IMU 50 ms latency', category='sensor'),
    'latency_jitter': Scenario(dict(lidar_latency_s=.05, lidar_latency_jitter_s=.25, imu_latency_s=.05,
                                    command_latency_s=.12, start_yaw_deg=10.),
                               description='LiDAR latency 50-300 ms jitter (in-order delivery), IMU 50 ms, '
                               'command 120 ms', category='sensor'),
    'imu_noise_vibration': Scenario(dict(gyro_noise_scale=2., gyro_bias_deg_s=.6, start_yaw_deg=10.,
                                         plant_overrides=(('drive_vibration_rms_m_s2', 3.),)),
                                    description='gyro noise x2 of the measured level, 0.6 deg/s bias, 3.0 m/s^2 '
                                    'drive vibration at low duty', category='sensor'),
    'lidar_blackout_dropouts': Scenario(dict(blackouts=((12., 13.5, 'lidar'),), scan_drop_probability=.05,
                                             start_yaw_deg=6.), description='1.5 s LiDAR blackout + 5% dropped scans',
                                        category='sensor'),
    'lidar_blackout_long': Scenario(dict(blackouts=((12., 16., 'lidar'),), start_yaw_deg=6.),
                                    acceptable_holds=('corridor_not_reacquired',),
                                    description='4 s LiDAR blackout (longer than the 2 s band timeout)',
                                    category='sensor'),
    'imu_blackout': Scenario(dict(blackouts=((12., 13.5, 'imu'),), start_yaw_deg=6.),
                             description='1.5 s IMU blackout', category='sensor'),
    'lidar_imu_blackout': Scenario(dict(blackouts=((12., 13.5, 'both'),), start_yaw_deg=6.),
                                   description='1.5 s LiDAR + IMU blackout', category='sensor'),
    # --- plant sensitivity / drive response ---------------------------------------------
    'pivot_stall_surface': Scenario(dict(surface='stall_prone', start_yaw_deg=30.),
                                    acceptable_holds=('pivot_stall_abort',),
                                    description='stall-prone surface (035952: 8/11 pivots did not rotate, rotating '
                                    'ones 6.5-7.3 deg/s), 30 deg entry error', category='plant_sensitivity'),
    'slow_pivot_surface': Scenario(dict(start_yaw_deg=30., plant_overrides=(
        ('pivot_rate_events_deg_s', (4.8, 5.7, 6.5, 7.3)),)),
        acceptable_holds=('pivot_stall_abort', 'turn_timeout'),
        description='SENSITIVITY: every pivot rotates only 4.8-7.3 deg/s (slow tail of the measured events)',
        category='plant_sensitivity'),
    'steering_ineffective': Scenario(dict(start_yaw_deg=8., plant_overrides=(('arc_yaw_deg_s_per_pct_diff', .02),)),
                                     description='SENSITIVITY: differential yields ~17% of the expected yaw',
                                     category='plant_sensitivity'),
    'low_duty_weak_unknown': Scenario(dict(start_yaw_deg=15., plant_overrides=(
        ('forward_deadband_pct', 60.), ('arc_yaw_low_duty_scale', .5))),
        description='SENSITIVITY: 60% deadband and half differential gain at low duty (unmeasured)',
        category='plant_sensitivity'),
    'low_duty_strong_unknown': Scenario(dict(start_yaw_deg=15., plant_overrides=(
        ('forward_deadband_pct', 35.), ('arc_yaw_low_duty_scale', 1.5))),
        description='SENSITIVITY: 35% deadband and 1.5x differential gain at low duty (unmeasured)',
        category='plant_sensitivity'),
    'arc_speed_loss': Scenario(dict(start_yaw_deg=12., plant_overrides=(('arc_speed_loss_per_pct_diff', .0074),)),
                               description='SENSITIVITY: 0.74% speed loss per % differential (unresolved in data)',
                               category='plant_sensitivity'),
    'forward_deadband_above_base': Scenario(dict(start_yaw_deg=4., plant_overrides=(('forward_deadband_pct', 78.),)),
                                            expected=(), acceptable_holds=('drive_no_response',),
                                            description='SENSITIVITY: 78% deadband above the 75% base duty: the '
                                            'vehicle cannot move; a no-response HOLD is the only acceptable outcome',
                                            category='plant_sensitivity'),
    'root_stuck_midrow': Scenario(dict(start_yaw_deg=4., forward_blocks=((12., 45.),), max_time_s=60.),
                                  expected=(), acceptable_holds=('drive_no_response',),
                                  description='forward motion blocked from t=12 s (root/step below the scan plane): '
                                  'a no-response HOLD is the only acceptable outcome', category='plant_sensitivity'),
    'mount_footprint_mismatch': Scenario(dict(start_yaw_deg=20., plant_overrides=(
        ('lidar_x_in_base_m', .30), ('footprint_length_m', 1.30), ('footprint_width_m', .76))),
        acceptable_holds=('pivot_clearance_insufficient',),
        description='plant mount 0.30 m and footprint 1.30x0.76 m while the estimator keeps its nominal 0.18 m and '
        '1.22x0.70 m', category='plant_sensitivity'),
}


# Opt-in side-clearance scenes (evaluate --extra-scenarios); NOT part of the default matrix.
# All three are passable: a side_blocked stop there is a false stop. Clearances are nominal
# (centred vehicle); the run records the truth start clearance (start_*_clearance_m).
EXTRA_SCENARIOS = {
    'aisle_2p2m': Scenario(dict(world=_w(row_width_m=2.2, tree_spacing_m=2., branch_reach_m=.3), start_yaw_deg=3.),
                           description='2.2 m rows, 2.0 m spacing, 0.3 m canopy: centred straight-pass truth clearance '
                           '0.29-0.44 m (seeds 0-7; soft-exit region; false soft/hard and oscillation check)',
                           category='side_clearance'),
    'one_side_deep_canopy': Scenario(dict(world=_w(branch_reach_scale_by_row=((1, 2.5),)), start_yaw_deg=3.),
                                     description='left row canopy reach x2.5 (to ~1.1 m): centred straight-pass truth '
                                     'clearance 0.15-0.38 m on the left (seeds 0-7), right ~0.85 m (bias and '
                                     'swing-inhibit check)', category='side_clearance'),
    'midrow_side_post': Scenario(dict(world=_w(extra_posts=((10., .60, .08),)), start_yaw_deg=3.),
                                 description='hard post (r 0.08 m) at field (10, 0.60): edge 0.52 m from the centreline, '
                                 'outside the 0.45 m front band; ~0.17 m side clearance (swing-inhibit check)',
                                 category='side_clearance'),
}


# Opt-in headland clutter scenes (evaluate --extra-scenarios headland_clutter); NOT part of the default matrix.
# Review F5 of the row-end line-continuation veto (taskE_crossalley): objects BEYOND the true row end near the
# row-line extensions. Base geometry = nominal_open_end (3 rows at field y -1.75 / +1.75 / +5.25, corridor 0
# centred on y = 0, 2.5 m spacing, nominal last trunk x = 20 m, open headland). The clutter never enters the
# scene truth (row trunks, row end, scene walls): the correct outcome stays a row-end stop at the last trunk.
# Placement note (OFF behaviour): returns inside the original rule's 0.35 m line band and its 10 m support
# range are row support, so an object ON a line extension closer than about 10-11 m beyond the end moves the
# OFF end before the stop trigger (an existing OFF limitation; those scenes show it, they are not tuned away).
# stream_key: every headland scene shares the orchard, plant and sensor draws of nominal_open_end for the same seed
# (only the clutter differs), so a headland run can be compared with that nominal run.
_HL = dict(start_yaw_deg=5., stream_key='nominal_open_end')    # = nominal_open_end's start
_HL_ROWS = (-1.75, 1.75, 5.25)
_HL_END = 20.


def _fence(y, x0=10., x1=32.):
    return (x0, y, x1, y)


def _bin(x0, y_centre, side=1.2):
    """Square bin (side x side m) as 4 hard wall segments, near face at field x0."""
    h = side / 2
    a, b, c, d = (x0, y_centre - h), (x0 + side, y_centre - h), (x0 + side, y_centre + h), (x0, y_centre + h)
    return tuple((*p, *q) for p, q in ((a, b), (b, c), (c, d), (d, a)))


def _oblique(angle_deg, x_right, y_lo=-3.5, y_hi=7.):
    """Boundary wall at angle_deg to the row axis through (x_right, -1.75) on the RIGHT line extension;
    it crosses the left line extension 3.5 / tan(angle) m further."""
    t = 1. / math.tan(math.radians(angle_deg))
    return ((x_right + (y_lo + 1.75) * t, y_lo, x_right + (y_hi + 1.75) * t, y_hi),)


def _next_block(headland_m, trees=6, spacing=2.5):
    """Aligned next block: trees on the same 3 lines from x = 20 + headland_m (tree-to-tree gap)."""
    return tuple((_HL_END + headland_m + k * spacing, y) for y in _HL_ROWS for k in range(trees))


HEADLAND_SCENARIOS = {
    'headland_fence_right_0p5': Scenario(
        dict(world=_w(extra_walls=(_fence(-2.25),)), **_HL), acceptable_holds=('front_blocked',),
        description='fence parallel to the RIGHT (block edge) line 0.5 m outside it, x 10-32 m (12 m beyond the end)',
        category='headland_clutter'),
    **{'headland_fence_both_%s' % tag: Scenario(
        dict(world=_w(extra_walls=(_fence(-1.75 - d), _fence(1.75 + d))), **_HL),
        acceptable_holds=('front_blocked',),
        description='fences parallel to BOTH lines %.1f m outside them, x 10-32 m' % d, category='headland_clutter')
       for tag, d in (('0p5', .5), ('0p7', .7), ('0p9', .9))},
    **{'headland_next_block_%dm' % h: Scenario(
        dict(world=_w(extra_trees=_next_block(h)), **_HL),
        description='next block aligned with the rows, its first trees %d m beyond the last trunk (tree-to-tree '
        'headland; trunks + canopy, 6 trees per line)' % h, category='headland_clutter')
       for h in (6, 9, 10, 12, 14)},
    **{'headland_oblique_wall_%d' % a: Scenario(
        dict(world=_w(extra_walls=_oblique(a, _HL_END + 11.5)), **_HL), acceptable_holds=('front_blocked',),
        description='boundary wall at %d deg to the rows crossing the right line extension 11.5 m and the left one '
        '%.1f m beyond the end' % (a, 11.5 + 3.5 / math.tan(math.radians(a))), category='headland_clutter')
       for a in (20, 30, 45, 60, 70)},
    'headland_oblique_wall_45_near': Scenario(
        dict(world=_w(extra_walls=_oblique(45, _HL_END + 7.25)), **_HL), acceptable_holds=('front_blocked',),
        description='boundary wall at 45 deg crossing the right line extension 7.25 m and the left one 10.75 m '
        'beyond the end (both inside a 12 m continuation zone; inside the OFF support range near the end)',
        category='headland_clutter'),
    **{'headland_posts_both_%s' % tag: Scenario(
        dict(world=_w(extra_posts=((_HL_END + x, -1.75, .08), (_HL_END + x, 1.75, .08))), **_HL),
        description='one post (r 0.08 m) on each line extension %.1f m beyond the end%s' % (x, note),
        category='headland_clutter')
       for tag, x, note in (('9m', 9., ' (inside the OFF support range before the stop: row support for OFF)'),
                            ('11p5m', 11.5, ''))},
    'headland_end_posts_1p5m': Scenario(
        dict(world=_w(extra_posts=((_HL_END + 1.5, -1.75, .1), (_HL_END + 1.5, 1.75, .1))), **_HL),
        description='trellis end post (r 0.10 m) on each line 1.5 m beyond the last trunk (row support for the '
        'original rule)', category='headland_clutter'),
    # Bins 1.0 m (centre) outside the line extensions: the inner face (0.4 m outside) is outside the original
    # rule's 0.35 m line band (not OFF row support) but inside the widened continuation band (0.7-0.8 m).
    'headland_bins_both': Scenario(
        dict(world=_w(extra_walls=_bin(_HL_END + 10., -2.75) + _bin(_HL_END + 10., 2.75)), **_HL),
        acceptable_holds=('front_blocked',),
        description='1.2 m harvest bin beside each line extension (centre 1.0 m outside), 10-11.2 m beyond the end',
        category='headland_clutter'),
    'headland_bins_two_both': Scenario(
        dict(world=_w(extra_walls=(_bin(_HL_END + 9., -2.75) + _bin(_HL_END + 11.5, -2.75)
                                   + _bin(_HL_END + 9., 2.75) + _bin(_HL_END + 11.5, 2.75))), **_HL),
        acceptable_holds=('front_blocked',),
        description='two 1.2 m bins beside each line extension (centre 1.0 m outside), 9 and 11.5 m beyond the end',
        category='headland_clutter'),
}

# Opt-in scene groups for evaluate --extra-scenarios [GROUP ...] (a bare flag = side_clearance, as before).
EXTRA_GROUPS = dict(side_clearance=EXTRA_SCENARIOS, headland_clutter=HEADLAND_SCENARIOS)


def extra_groups(extra):
    """Group names of an episodes()/evaluate 'extra' value: False/None/() -> none, True -> side_clearance
    (the meaning of the original boolean flag), an iterable of names -> those groups (validated)."""
    if extra is True:
        return ('side_clearance',)
    if not extra:
        return ()
    groups = tuple(extra)
    unknown = sorted(set(groups) - set(EXTRA_GROUPS))
    if unknown:
        raise ValueError('unknown extra scenario groups %s (known: %s)' % (unknown, sorted(EXTRA_GROUPS)))
    return groups


def scenario_pool(extra=False):
    """The default SCENARIOS plus the requested opt-in groups (group names never collide)."""
    pool = dict(SCENARIOS)
    for group in extra_groups(extra):
        pool.update(EXTRA_GROUPS[group])
    return pool


def scenario_spec(name):
    if name in SCENARIOS:
        return SCENARIOS[name]
    for group in EXTRA_GROUPS.values():
        if name in group:
            return group[name]
    raise KeyError(name)


def episodes(seeds=SEEDS, modes=MODES, names=None, extra=False, estimator_overrides=()):
    """Episode configurations; estimator_overrides ((name, value), ...) are appended to every episode's
    (opt-in estimator variants such as row_end_position_confirm_m). They do not enter the random stream
    seeds, so a base and a variant run of the same (scenario, mode, seed) share every draw.
    extra: False (default matrix), True (+ side_clearance, the original flag) or EXTRA_GROUPS names."""
    result = []
    pool = scenario_pool(extra)
    for name, spec in pool.items():
        if names and name not in names:
            continue
        for mode in modes:
            for seed in seeds:
                world = spec.overrides.get('world', WorldConfig())
                config = EpisodeConfig(name=name, mode=mode, seed=seed, **{
                    k: v for k, v in spec.overrides.items() if k != 'world'})
                world = replace(world, seed=stream_seed(config, 'world'))
                if estimator_overrides:
                    config = replace(config, estimator_overrides=tuple(config.estimator_overrides)
                                     + tuple(estimator_overrides))
                result.append(replace(config, world=world))
    return result


def judge(result, scenario_name):
    """Truth-based outcome labels for one run (evaluation only)."""
    spec = scenario_spec(scenario_name)
    reason = result['final_reason']
    collision = result['collisions'] > 0
    left = result['left_corridor']
    ok = reason in spec.expected
    if reason == 'row_end_candidate':
        ok = ok and -.5 <= result['final_x_rel_row_end_m'] <= 1.5
    if reason == 'front_blocked' and spec.wall_stop:
        gap = result['wall_gap_m']
        ok = ok and gap is not None and .1 <= gap <= 1.2 and result['progress_m'] >= 1.
    completed = not collision and not left and ok
    terminal = result['end'] == 'terminal_hold'
    hold_ok = reason in spec.acceptable_holds
    if hold_ok and reason == 'front_blocked' and spec.wall_stop:
        gap = result['wall_gap_m']
        hold_ok = gap is not None and .1 <= gap <= 1.2
    expected_hold = not completed and not collision and not left and terminal and hold_ok
    safe_hold = not completed and not collision and terminal
    estimate = result.get('row_end_first_estimate')
    claim, truth = result['last_row_claim'], result['last_row_expected']
    return dict(completed=completed, expected_hold=expected_hold, safe_hold=safe_hold, collision=collision,
                acceptable=completed or expected_hold,
                row_end_recognised=bool(estimate is not None and abs(estimate[1]) <= 1.5),
                last_row=('correct' if claim == truth else 'undecided' if claim == 'UNKNOWN' else 'wrong'))
