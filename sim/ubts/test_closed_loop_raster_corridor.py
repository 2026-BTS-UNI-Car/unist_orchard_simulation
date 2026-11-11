"""closed_loop.raster_corridor: map I/O, corridor frame, raster LiDAR, clearance, modifiers, evaluator, hooks.

Synthetic rasters for the contracts; the real 10/10 maps and the catalogue only where present (skipped
otherwise). ``closed_loop/testdata/raster_world_ref_e71e27d4.py`` is the unmodified reference raster
world of mapping-rotation-20261022/native_recorded_042301120_runtime_v1/RASTER/raster_world.py
(sha256 e71e27d40522073908d2b5212a4a05f92cb1187c783f69256fc5080bd9d5f507): its supercover DDA
RasterGrid.raycast is the oracle for the fast ray/cell-square raycaster.
"""
import hashlib
import importlib.util
import json
import math
from dataclasses import replace
from pathlib import Path
import types

import numpy as np
import pytest

from closed_loop import raster_corridor as RC
from closed_loop.harness import EpisodeConfig, run_episode
from closed_loop.world import WorldConfig, build_world, footprint_clearance, make_lidar

HERE = Path(__file__).resolve().parent
REF_PATH = HERE / 'closed_loop' / 'testdata' / 'raster_world_ref_e71e27d4.py'
REF_SHA256 = 'e71e27d40522073908d2b5212a4a05f92cb1187c783f69256fc5080bd9d5f507'


def reference_module():
    raw = REF_PATH.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == REF_SHA256
    spec = importlib.util.spec_from_file_location('raster_world_ref', REF_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def real_data_missing():
    """None when the 10/10 maps and the corridor catalogue are configured and present, else the reason.
    Their locations come from UBTS_MAPS_1010 / UBTS_SINGLE_ROW_CATALOGUE or the UBTS_STEER_PATHS file."""
    try:
        RC.maps_root()
        RC.catalogue_path()
        return None
    except FileNotFoundError as error:
        return str(error)


REAL_DATA_MISSING = real_data_missing()
real = pytest.mark.skipif(REAL_DATA_MISSING is not None,
                          reason='10/10 maps or the corridor catalogue not available: %s' % REAL_DATA_MISSING)


# --- map files ---------------------------------------------------------------------------------------------

def pgm(width, height, payload, comment=b'# Cartographer map; 0.050000 m/pixel\n', maxval=b'255', sep=b'\n'):
    return b'P5\n' + comment + b'%d %d\n' % (width, height) + maxval + sep + payload


def test_pgm_parser_accepts_binary_p5_with_comment_and_rejects_malformed_files():
    pixels, header = RC.parse_pgm(pgm(3, 2, bytes(range(6))))
    assert pixels.shape == (2, 3) and pixels[1, 2] == 5 and header['comments'][0].startswith('Cartographer map')
    with pytest.raises(ValueError, match='pgm_not_binary_P5'):
        RC.parse_pgm(pgm(3, 2, bytes(6)).replace(b'P5', b'P2'))
    with pytest.raises(ValueError, match='pgm_size_or_maxval'):
        RC.parse_pgm(pgm(3, 2, bytes(6), maxval=b'65535'))
    with pytest.raises(ValueError, match='pgm_payload_size'):
        RC.parse_pgm(pgm(3, 2, bytes(5)))                       # truncated
    with pytest.raises(ValueError, match='pgm_payload_size'):
        RC.parse_pgm(pgm(3, 2, bytes(7)))                       # trailing byte
    with pytest.raises(ValueError, match='pgm_no_separator'):
        RC.parse_pgm(b'P5 3 2 255')


def test_map_yaml_and_the_trinary_class_rule():
    doc = RC.parse_map_yaml('image: a_offline.pgm\nresolution: 0.050000\norigin: [-1.5, -2.0, 0.0]\nnegate: 0\n'
                            'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
    assert doc['origin'] == [-1.5, -2., 0.] and doc['negate'] == 0 and doc['resolution'] == .05
    with pytest.raises(ValueError, match='missing_free_thresh'):
        RC.parse_map_yaml('image: a.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\n')
    lut = RC.class_lut(.65, .196)
    # p = (255 - v) / 255: occupied p > 0.65 (v <= 89), free p < 0.196 (v >= 206), 128 = no submap data
    assert lut[0] == lut[89] == RC.OCCUPIED
    assert lut[90] == lut[127] == RC.UNCERTAIN_OCC                 # observed, 0.5 <= p <= 0.65
    assert lut[128] == RC.UNOBSERVED
    assert lut[129] == lut[205] == RC.UNCERTAIN                    # observed, 0.196 <= p < 0.5
    assert lut[206] == lut[255] == RC.FREE


# --- corridor frame ------------------------------------------------------------------------------------------

def arc(radius=40., step=.25, length=30.):
    a = np.arange(0., length + 1e-9, step) / radius
    return np.column_stack((radius * np.sin(a), radius * (1 - np.cos(a))))


def test_frame_round_trip_left_positive_and_segment_tangent():
    line = RC.CurvilinearFrame([(0., 0.), (10., 0.)], s_origin=2.)
    assert line.to_field(5., .7, .1) == pytest.approx((3., .7, .1))
    assert line.to_field(5., -.7, 0.)[1] == pytest.approx(-.7)          # right of travel is negative
    assert line.to_field(-3., .2)[0] == pytest.approx(-5.)              # first segment extrapolated
    assert line.to_field(14., .2)[0] == pytest.approx(12.)              # last segment extrapolated
    frame = RC.CurvilinearFrame(arc(), s_origin=1.)
    rng = np.random.default_rng(3)
    for _ in range(200):
        k = int(rng.integers(1, len(frame.L) - 1))
        s = frame.s_vertex[k] + frame.L[k] / 2                          # segment middle: unambiguous segment
        d, yaw = rng.uniform(-1., 1.), rng.uniform(-.5, .5)
        x, y, a = frame.pose(s, d, yaw)
        back = frame.to_field(x, y, a)
        assert back == pytest.approx((s, d, yaw), abs=1e-9)
        assert frame.tangent(s) == pytest.approx(frame.angle[k])       # direction of the segment
    s, d = frame.project_many(*frame.point_many([3.1, 12.4], [.5, -.4]))
    assert s == pytest.approx([3.1, 12.4]) and d == pytest.approx([.5, -.4])


# --- LiDAR ---------------------------------------------------------------------------------------------------

def synthetic_map(classes, res=.05):
    classes = np.asarray(classes, dtype=np.uint8)
    return RC.RasterMap(classes, res, dict(record='synthetic', pgm_sha256='', yaml_sha256='', occupied_thresh=.65,
                                           free_thresh=.196))


def fake_world(rmap):
    return types.SimpleNamespace(map=rmap, lidar_stats=None)


def test_fast_raycaster_equals_the_reference_supercover_dda():
    ref = reference_module()
    rng = np.random.default_rng(20261102)
    checked = 0
    for case in range(30):
        h, w = int(rng.integers(30, 70)), int(rng.integers(30, 70))
        classes = rng.choice(np.array([RC.FREE, RC.UNCERTAIN, RC.UNCERTAIN_OCC, RC.UNOBSERVED, RC.OCCUPIED],
                                      dtype=np.uint8), size=(h, w), p=[.62, .08, .05, .1, .15])
        res = (.05, .1, .25)[case % 3]
        rmap = synthetic_map(classes, res)
        caster = RC.RasterLidarCaster(fake_world(rmap), rays=720, maximum=30.)
        # reference semantics: occupied -> return, unobserved -> ends without return, the rest transparent
        cells = np.where(classes == RC.OCCUPIED, ref.OCCUPIED, np.where(classes == RC.UNOBSERVED, ref.UNKNOWN,
                                                                         ref.FREE)).astype(np.uint8)
        grid = ref.RasterGrid(cells, res)
        for _ in range(4):
            x, y = rng.uniform(.5, w * res - .5), rng.uniform(.5, h * res - .5)
            yaw = rng.uniform(-math.pi, math.pi)
            if classes[int(y / res), int(x / res)] != RC.FREE:
                continue
            ours = caster.cast(x, y, yaw)
            ranges, kinds = grid.raycast(x, y, caster.angles + yaw, 30.)
            theirs = np.where(kinds == ref.OCCUPIED, ranges, np.inf)
            theirs[theirs > 30.] = np.inf
            assert np.array_equal(np.isfinite(ours), np.isfinite(theirs))
            both = np.isfinite(ours)
            assert np.allclose(ours[both], theirs[both], atol=1e-9, rtol=0)
            checked += 1
    assert checked > 60


def test_raycaster_inside_cells_occlusion_and_scan_contract():
    classes = np.full((40, 40), RC.FREE, dtype=np.uint8)
    classes[:, 30] = RC.OCCUPIED
    classes[5, 5] = RC.UNOBSERVED
    classes[20, 5] = RC.OCCUPIED
    rmap = synthetic_map(classes)
    caster = RC.RasterLidarCaster(fake_world(rmap))
    assert np.all(np.isinf(caster.cast(5.5 * .05, 5.5 * .05, 0.)))      # sensor in an unobserved cell
    assert np.all(caster.cast(5.5 * .05, 20.5 * .05, 0.) == 0.)       # sensor inside an obstacle
    ranges, origin, directions = caster.scan(np.array((.5, 1., .6)), np.eye(3))
    assert ranges.shape == (RC.LIDAR_RAYS,) and directions.shape == (RC.LIDAR_RAYS, 3)
    forward = int(np.argmin(np.abs(caster.angles)))
    assert ranges[forward] == pytest.approx(30 * .05 - .5)              # wall face at x = 1.5 m
    caster.occluded_sectors = ((-.1, .1),)
    assert np.isinf(caster.scan(np.array((.5, 1., .6)), np.eye(3))[0][forward])


# --- clearance, modifiers, world (synthetic corridor) -------------------------------------------------------------

def synthetic_corridor(length=30., spacing=3.7, tree_pitch=2., res=.05):
    """Straight corridor along +x at y = 4 (raster-local); trunks = 3x3-cell occupied blocks at y = 4 +- 1.85."""
    h, w = int(8. / res), int((length + 10.) / res)
    classes = np.full((h, w), RC.FREE, dtype=np.uint8)
    units = {'left': [], 'right': []}
    for side, sign in (('left', 1.), ('right', -1.)):
        y = 4. + sign * spacing / 2
        for x in np.arange(2., 2. + length + 1e-9, tree_pitch):
            r, c = int(y / res), int(x / res)
            classes[r - 1:r + 2, c - 1:c + 2] = RC.OCCUPIED
            units[side].append(((c + .5) * res - 2., (r + .5) * res - 4.))
    classes[:, :2] = RC.UNOBSERVED
    rmap = synthetic_map(classes, res)
    end = length
    config = RC.RasterCorridorConfig(
        record='synthetic', corridor_id='s01', polyline=((0., 4.), (length + 10., 4.)), s_origin_m=2.,
        end_s_m=end, line_end_s_m=(end, end), profile_s_m=tuple(np.arange(-2., length + 8.)),
        left_row_d_m=tuple([spacing / 2] * len(np.arange(-2., length + 8.))),
        right_row_d_m=tuple([-spacing / 2] * len(np.arange(-2., length + 8.))),
        left_units=tuple(units['left']), right_units=tuple(units['right']), row_width_m=spacing, band_m=.75)
    return rmap, config


def build_synthetic(modifiers=()):
    rmap, config = synthetic_corridor()
    config = replace(config, modifiers=tuple(modifiers))
    frame = RC.CurvilinearFrame(config.polyline, config.s_origin_m)
    copy = RC.RasterMap(rmap.classes.copy(), rmap.resolution, rmap.info)
    log, walls = RC.apply_modifiers(copy, frame, config)
    return rmap, RC.RasterCorridorWorld(config, copy, frame, log, walls)


def test_footprint_clearance_groups_are_conservative_full_cell_distances():
    classes = np.full((60, 60), RC.FREE, dtype=np.uint8)
    classes[30, 45] = RC.OCCUPIED                                       # cell centre x = 2.275 m
    classes[10, 30] = RC.UNCERTAIN
    rmap = synthetic_map(classes)
    c = RC.footprint_clearances(rmap, 1.525, 1.525, 0., 1.22, .70)      # front face at x = 2.135
    assert c['hard'] == pytest.approx(2.275 - 2.135 - .05 / math.sqrt(2), abs=1e-9)
    assert c['uncertain'] < math.inf and c['branch'] == math.inf and c['added_hard'] == math.inf
    far = RC.footprint_clearances(rmap, -10., -10., 0., 1.22, .70)      # outside the raster
    assert far['unobserved'] <= 0


def test_modifiers_change_a_copy_only_and_follow_the_prereg_placements():
    rmap, world = build_synthetic(RC.prereg_gap(('left',), 30., 5., .35)
                                  + RC.prereg_branches(30.) + RC.prereg_obstacle(30.) + RC.prereg_wall(1.5))
    before = hashlib.sha256(rmap.classes.tobytes()).hexdigest()
    gap = world.modifier_log[0]
    lo, hi = gap['s_from_m'], gap['s_to_m']
    assert hi - lo == pytest.approx(5.) and (lo + hi) / 2 == pytest.approx(.35 * 30.)
    assert gap['unmatched_catalogue_units'] == 0 and gap['removed_cells'] == 9 * len(gap['removed_unit_s_m'])
    assert all(lo <= s <= hi for s in gap['removed_unit_s_m']) and len(gap['removed_unit_s_m']) == 3
    occupied_left = [s for s, d in world.config.left_units
                     if world.map.class_at(*world.frame.point(s, d))[0] == RC.OCCUPIED]
    assert not any(lo <= s <= hi for s in occupied_left) and len(occupied_left) == len(world.config.left_units) - 3
    branches = [m for m in world.modifier_log if m['kind'] == 'branch_rect']
    assert [m['side'] for m in branches] == ['left', 'right', 'left'] and [m['s_m'] for m in branches] == [6., 14., 22.]
    s_b, d_b = world.frame.project_many(*world.map.centres(*np.nonzero(world.map.classes == RC.ADDED_BRANCH)))
    assert np.abs(d_b).min() == pytest.approx(.95, abs=.05) and np.abs(d_b).max() <= 1.85 + 1e-9
    box = world.modifier_log[-2]
    assert box['kind'] == 'obstacle_box' and box['s_m'] == 15. and box['cells'] > 100
    wall = world.modifier_log[-1]
    assert wall['near_face_s_m'] == pytest.approx(31.5) and world.corridor_truth(0)['cross_walls_x'] == (wall['near_face_s_m'],)
    s_w, d_w = world.frame.project_many(*world.map.centres(*np.nonzero(world.map.classes == RC.ADDED_HARD)))
    wall_cells = s_w > 25.
    assert s_w[wall_cells].min() == pytest.approx(31.5, abs=.05) and s_w[wall_cells].max() <= 31.6 + 1e-9
    assert np.abs(d_w[wall_cells]).max() <= 3.7 / 2 + 1.5 + 1e-9
    assert hashlib.sha256(rmap.classes.tobytes()).hexdigest() == before        # the base grid is untouched
    _, removed = build_synthetic((RC.RemoveTrunks('right', 10., 2),))
    assert removed.modifier_log[0]['removed_unit_s_m'] == pytest.approx([10., 12.], abs=.05)   # cell centres
    with pytest.raises(ValueError):
        build_synthetic((RC.RemoveTrunks('right', 29., 3),))


def test_world_truth_and_clearance_counters():
    _, world = build_synthetic()
    truth = world.corridor_truth(0)
    assert truth['end_x'] == 30. and truth['next_left'] == truth['next_right'] == 'UNKNOWN'
    with pytest.raises(ValueError):
        world.corridor_truth(1)
    x, y, yaw = world.pose_in_field(5., 0., 0.)
    assert world.clearance(x, y, yaw, 1.22, .70)[0] > 1.
    x, y, yaw = world.pose_in_field(4., 1.6, 0.)                     # on the left row: hard contact
    assert world.clearance(x, y, yaw, 1.22, .70)[0] <= 0 and world.contacts['hard'] == 1


# --- evaluator --------------------------------------------------------------------------------------------------

def synthetic_result(world, s_stop=29.6, d=.05, reason='row_end', negative=False):
    trace, t, s = [], 0., 1.
    while True:
        t += .05
        moving = s < s_stop
        if moving:
            s = min(s + .02, s_stop)
        trace.append(dict(t=round(t, 2), fx=round(s, 3), fy=d, fyaw_deg=0.5, v=.4 if moving else 0.,
                          left=-5 if negative and t < 1 else (60 if moving else 0), right=60 if moving else 0,
                          state='FOLLOW' if moving else 'HOLD', reason=reason, est_heading_deg=0., est_lateral=0.,
                          corridor_reason='ok'))
        if not moving and len([r for r in trace if r['v'] == 0.]) * .05 >= 1.2:
            break
    return dict(trace=trace, end='terminal_hold', final_reason=reason, row_end_first_estimate=None)


def test_evaluator_prereg_metrics_on_a_synthetic_trace():
    _, world = build_synthetic()
    ev = RC.evaluate_episode(synthetic_result(world), world, s_start=1., duty_caps=(25, 100))
    assert ev['prereg_end'] == 'TERMINAL_STOP' and ev['stop_category'] == 'ROW_END'
    assert ev['stop_position_e_m'] == pytest.approx(29.6 + .61 - 30., abs=2e-3)
    assert ev['row_end_outcome'] == 'OK' and not ev['false_row_end_detection'] and not ev['missed_row_end']
    assert ev['lateral_m']['rmse'] == pytest.approx(.05) and ev['heading_deg']['max'] == pytest.approx(.5)
    assert ev['contacts']['collision_samples'] == 0 and not ev['safety_band']['exceeded']
    assert ev['forbidden'] == dict(pivot_or_reverse=False, auto_restart=False)
    assert ev['pwm_saturation']['saturated_fraction'] == 0.
    early = RC.evaluate_episode(synthetic_result(world, s_stop=25.), world, s_start=1.)
    assert early['row_end_outcome'] == 'FALSE' and early['false_row_end_detection']
    missed = RC.evaluate_episode(synthetic_result(world, s_stop=33.5, reason='obstacle_ahead'), world, s_start=1.)
    assert missed['prereg_end'] == 'MISSED_ROW_END' and missed['missed_row_end'] and missed['stop_category'] is None
    reverse = RC.evaluate_episode(synthetic_result(world, negative=True), world, s_start=1.)
    assert reverse['forbidden']['pivot_or_reverse']
    off = synthetic_result(world, d=1.6)
    assert RC.evaluate_episode(off, world, s_start=1.)['prereg_end'] in ('CORRIDOR_DEPARTURE', 'COLLISION')
    assert RC.stop_category('drive_no_response')[0] == 'CONTROL_AUTHORITY'
    assert RC.stop_category('stale_lidar')[0] == 'PERCEPTION_OR_LOCALISATION'
    assert RC.stop_category('no_such_reason')[0] == 'UNCLASSIFIED'


def test_wall_and_obstacle_clearance_at_rest_reach_beyond_the_contact_reach():
    # dev round 1 (2026-11-02): a correct row-end stop (front at the end) 3 m before a PREREG wall_3p0 wall was
    # reported as 'no wall within the 2 m contact reach' (None); the at-rest clearance now uses AT_REST_REACH_M
    for distance in (1.5, 3.):
        _, world = build_synthetic(RC.prereg_wall(distance))
        ev = RC.evaluate_episode(synthetic_result(world, s_stop=30. - .61), world, s_start=1.)
        assert ev['prereg_end'] == 'TERMINAL_STOP'
        assert ev['obstacle_wall_clearance_at_rest_m'] == pytest.approx(distance - .05 / math.sqrt(2), abs=.06)
        assert ev['contacts']['min_hard_clearance_m'] > 0
    _, world = build_synthetic()
    ev = RC.evaluate_episode(synthetic_result(world), world, s_start=1.)
    assert ev['obstacle_wall_clearance_at_rest_m'] is None                  # nothing added within the reach
    assert RC.AT_REST_REACH_M == 6.


# --- harness hooks ----------------------------------------------------------------------------------------------

def test_harness_default_hooks_are_the_procedural_world():
    config = EpisodeConfig(name='hook_identity', mode='continuous', world=WorldConfig(row_length_m=12.),
                           start_yaw_deg=4., max_time_s=10.)
    implicit = run_episode(config, record_trace=True)
    explicit = run_episode(config, record_trace=True, world_factory=build_world, lidar_factory=make_lidar,
                           clearance_evaluator=footprint_clearance)
    assert json.dumps(implicit, sort_keys=True, default=repr) == json.dumps(explicit, sort_keys=True, default=repr)


# --- real 10/10 data ----------------------------------------------------------------------------------------------

def included_entry():
    catalogue = RC.load_catalogue()
    entry = next(e for e in catalogue['corridors']
                 if all(e['directions'][k]['status'] == 'included' for k in ('fwd', 'rev')))
    return catalogue, entry


@real
def test_real_corridor_world_both_directions_and_unit_matching():
    catalogue, entry = included_entry()
    fwd = RC.build_raster_world(RC.corridor_world_config(entry['record'], entry['corridor_id'], 'fwd',
                                                         catalogue=catalogue))
    rev = RC.build_raster_world(RC.corridor_world_config(entry['record'], entry['corridor_id'], 'rev',
                                                         catalogue=catalogue))
    assert fwd.corridor_truth(0)['end_x'] == pytest.approx(entry['directions']['fwd']['length_m'], abs=1e-3)
    assert rev.corridor_truth(0)['end_x'] == pytest.approx(entry['directions']['rev']['length_m'], abs=1e-3)
    start_rev = entry['directions']['rev']['start_s_fwd_m']
    x, y = fwd.frame.point(start_rev, .3)
    s, d, _ = rev.to_field(x, y, 0.)
    assert s == pytest.approx(0., abs=1e-6) and d == pytest.approx(-.3, abs=1e-6)
    for world in (fwd, rev):
        for side in ('left', 'right'):
            matched, unmatched = RC.support_units(world.map, world.frame, world.config, side)
            assert not unmatched and len(matched) == len(getattr(world.config, side + '_units'))
    x, y, yaw = fwd.pose_in_field(1., 0., 0.)
    assert fwd.map.info['pgm_sha256'] == entry['map']['pgm_sha256']
    assert fwd.clearances(x, y, yaw, 1.22, .70)['hard'] > 0


@real
def test_real_raster_episode_runs_and_is_evaluated():
    catalogue, entry = included_entry()
    world_config = RC.corridor_world_config(entry['record'], entry['corridor_id'], 'fwd', catalogue=catalogue)
    config = EpisodeConfig(name='raster_smoke_test', mode='continuous', world=world_config, start_x_m=1.,
                           max_time_s=6.)
    result, world = RC.run_raster_episode(config)
    assert result['raster_world']['lidar']['scans'] >= 50
    assert result['raster_world']['lidar']['scans_sensor_outside_exact_region'] == 0
    assert result['raster_evaluation']['prereg_end'] == 'OPEN' and result['collisions'] == 0
    assert result['raster_world']['controller_receives_map_or_pose'] is False
