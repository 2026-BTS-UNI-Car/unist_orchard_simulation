"""Sensor-only estimator invariants on synthetic ray-cast scans (no plant, no truth input).

Each test names one estimator property claimed in the results document and
fails when that property is removed (latency wrap, corridor acquisition gate
and gap veto, row-end support rules, one-sided updates, ICP failure handling,
next-corridor/last-row evidence).
"""
import math

import numpy as np
import pytest

from closed_loop.estimator import Band, SensorEstimator, advance, wrap
from orchard_perception.scenarios import Circle, OrchardScene, Pose2D, Segment, raycast_scan


def scan_of(circles=(), segments=(), yaw_deg=0.):
    scene = OrchardScene(tuple(circles), tuple(segments))
    scan = raycast_scan(scene, Pose2D(0., 0., math.radians(yaw_deg)))
    ranges = np.asarray(scan.ranges)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    ok = np.isfinite(ranges)
    points = np.column_stack((ranges[ok] * np.cos(angles[ok]), ranges[ok] * np.sin(angles[ok])))
    return scan, ranges, angles, points


def row(y, xs, radius=.13):
    return [Circle(float(x), float(y), radius) for x in xs]


def estimator_with(ranges, angles):
    estimator = SensorEstimator()
    estimator._scan_ranges, estimator._scan_angles = ranges, angles
    return estimator


def test_latency_advance_uses_a_wrapped_mid_heading():
    pose = np.array((0., 0., math.radians(179.9)))
    theta = wrap(math.radians(179.9 + .3))                           # crossed to about -179.8 deg
    x, y = advance(pose, theta, .4, .35)
    assert x == pytest.approx(-.14, abs=1e-3) and abs(y) < 1e-3     # naive mean would give +0.14


def test_acquisition_ignores_cross_row_columns_of_a_square_grid():
    circles = [c for y in (-7.5, -4.5, -1.5, 1.5, 4.5, 7.5) for c in row(y, np.arange(-7.5, 7.6, 3.))]
    _, _, _, points = scan_of(circles, yaw_deg=4.)
    found = SensorEstimator()._acquire(points)
    assert found is not None
    axis, left, right = found
    assert abs(math.degrees(axis) + 4.) <= 2.5                       # the row axis, not 86 deg columns
    assert 1.2 < left < 1.6 and -1.6 < right < -1.2


def test_acquisition_refuses_rows_outside_the_travel_window():
    circles = [c for y in (-1.75, 1.75) for c in row(y, np.arange(-10, 10.1, 2.5))]
    _, _, _, points = scan_of(circles, yaw_deg=70.)                  # rows 70 deg off the travel direction
    assert SensorEstimator()._acquire(points) is None                # +-50 deg window: refuse, do not pivot 70 deg


def test_acquisition_uses_the_dominant_axis_not_the_best_scoring_diagonal():
    # Canopy-rich 3 x 3 m interior grid (matrix scenario, development seed 3) where a diagonal
    # pair out-scores the row pair while the rows are the sharpest lines.
    from closed_loop.matrix import episodes
    from closed_loop.world import build_world, make_lidar, yaw_quaternion
    config = episodes(seeds=(3,), modes=('hybrid',), names=['interior_square_grid_aliasing'])[0]
    world = build_world(config.world)
    caster, _ = make_lidar(world, seed=1)
    x, y, yaw = world.pose_in_field(config.start_x_m, 2 * config.world.row_width_m, math.radians(4.))
    ranges, _, _ = caster.scan((x + .18 * math.cos(yaw), y + .18 * math.sin(yaw), .6), (
        (math.cos(yaw), -math.sin(yaw), 0.), (math.sin(yaw), math.cos(yaw), 0.), (0., 0., 1.)))
    angles = caster.angles
    ok = np.isfinite(ranges)
    points = np.column_stack((ranges[ok] * np.cos(angles[ok]), ranges[ok] * np.sin(angles[ok])))
    found = SensorEstimator()._acquire(points)
    assert found is not None and abs(math.degrees(found[0]) + 4.) <= 4.


def test_acquisition_never_pairs_across_a_gapped_row():
    circles = (row(-1.75, np.arange(-10, 10.1, 2.5)) + row(1.75, (-2.5, 7.5))
               + row(5.25, np.arange(-10, 10.1, 2.5)))
    _, _, _, points = scan_of(circles)
    found = SensorEstimator()._acquire(points)
    assert found is None or found[1] < 2.5                           # never the 7 m pair to the far row


def test_acquisition_accepts_a_large_entry_heading_inside_the_window():
    circles = [c for y in (-1.75, 1.75, 5.25) for c in row(y, np.arange(-10, 10.1, 2.5))]
    _, _, _, points = scan_of(circles, yaw_deg=40.)
    found = SensorEstimator()._acquire(points)
    assert found is not None and abs(math.degrees(found[0]) + 40.) <= 2.5


def _ends(circles, segments=()):
    _, ranges, angles, points = scan_of(circles, segments)
    estimator = estimator_with(ranges, angles)
    return estimator._side_extents(points, 0., 1.62, -1.62)


def test_open_row_end_on_both_sides_is_an_end():
    ends, free, front = _ends(row(1.75, np.arange(-7.5, 6.1, 2.5)) + row(-1.75, np.arange(-7.5, 6.1, 2.5)))
    assert front is None and all(free)
    assert all(4.5 < e < 5.5 for e in ends)


def test_one_side_continuing_is_not_an_end():
    ends, free, _ = _ends(row(1.75, np.arange(-7.5, 6.1, 2.5)) + row(-1.75, np.arange(-7.5, 15.1, 2.5)))
    assert free[0] and not free[1]


def test_a_fence_beyond_the_row_end_is_not_row_support():
    fence = [Segment((8., -4.), (8., 4.))]
    ends, free, front = _ends(row(1.75, np.arange(-7.5, 3.6, 2.5)) + row(-1.75, np.arange(-7.5, 3.6, 2.5)), fence)
    assert front is not None and 7.5 < front < 8.1
    assert all(e < 3.5 for e in ends) and all(free)                  # the end is the last trunk, not the fence


def test_a_wall_right_after_the_trunks_or_across_the_aisle_is_not_a_row_end():
    trunks = row(1.75, np.arange(-7.5, 7.6, 2.5)) + row(-1.75, np.arange(-7.5, 7.6, 2.5))
    ends, free, front = _ends([c for c in trunks if c.x < 6.], [Segment((6.4, -4.), (6.4, 4.))])
    assert front is not None and not any(free)


def _tracked(points_scan):
    _, ranges, angles, points = points_scan
    estimator = estimator_with(ranges, angles)
    estimator.band = Band(0., 0., 3.24, 0., 5)
    estimator.last_scan_stamp = 0.
    return estimator, points


def test_row_end_needs_two_consecutive_two_sided_updates_and_expires():
    trunks = row(1.75, np.arange(-7.5, 3.6, 2.5)) + row(-1.75, np.arange(-7.5, 3.6, 2.5))
    estimator, points = _tracked(scan_of(trunks))
    assert estimator._apply(.2, 0., 0., 1.62, -1.62, points, 'candidate')
    estimator.last_scan_stamp = .2
    assert estimator.snapshot(.25)['row_end_ahead_m'] is None        # one two-sided update: not yet
    assert estimator._apply(.4, 0., 0., 1.62, -1.62, points, 'candidate')
    estimator.last_scan_stamp = .4
    assert estimator.snapshot(.45)['row_end_ahead_m'] is not None    # confirmed
    estimator.last_scan_stamp = 3.
    assert estimator.snapshot(3.05)['row_end_ahead_m'] is None       # 2.65 s old: expired


def test_one_sided_updates_never_create_or_extend_a_row_end():
    trunks = row(1.75, np.arange(-7.5, 3.6, 2.5)) + row(-1.75, np.arange(-7.5, 3.6, 2.5))
    estimator, points = _tracked(scan_of(trunks))
    for k in range(4):
        assert estimator._apply(.2 * (k + 1), 0., 0., 1.62, None, points, 'candidate')
    estimator.last_scan_stamp = .8
    assert estimator.snapshot(.85)['row_end_ahead_m'] is None and estimator.end_streak == 0
    assert estimator._apply(1., 0., 0., 1.62, -1.62, points, 'candidate')
    for k in range(3):                                                # one-sided: no confirmation
        assert estimator._apply(1.1 + .1 * k, 0., 0., 1.62, None, points, 'candidate')
    estimator.last_scan_stamp = 1.3
    assert estimator.end_streak == 1 and estimator.snapshot(1.35)['row_end_ahead_m'] is None


def test_failed_scan_match_keeps_translation_and_uses_gyro_rotation():
    estimator = SensorEstimator()
    _, ranges, angles, _ = scan_of(row(1.75, np.arange(-7.5, 7.6, 2.5)) + row(-1.75, np.arange(-7.5, 7.6, 2.5)))
    estimator.on_imu(0., 0., (0., 0., 9.81))
    estimator.on_scan(.05, ranges, angles[0], angles[1] - angles[0], 30.)
    estimator.speed = .5                                              # a moving prior must NOT be integrated
    for k in range(1, 3):
        estimator.on_imu(.05 + k * .05, math.radians(10.), (0., 0., 9.81))
    empty = np.full(len(ranges), np.inf)
    empty[:5] = 3.                                                    # far too few returns to match
    estimator.on_scan(.15, empty, angles[0], angles[1] - angles[0], 30.)
    assert not estimator.odom_valid
    assert estimator.pose[0] == 0. and estimator.pose[1] == 0.
    assert math.degrees(estimator.pose[2]) == pytest.approx(1., abs=.2)


def _votes(circles, segments=(), scans=6):
    _, ranges, angles, points = scan_of(circles, segments)
    estimator = estimator_with(ranges, angles)
    for _ in range(scans):
        estimator._next_corridor_votes(points, 0., 1.62, -1.62, 3.5)
    return estimator.next_corridor()


def test_next_corridor_present_absent_and_unknown():
    own = row(1.75, np.arange(-7.5, 7.6, 2.5)) + row(-1.75, np.arange(-7.5, 7.6, 2.5))
    beyond = row(5.25, np.arange(-7.5, 7.6, 2.5))
    claims = _votes(own + beyond)
    assert claims == dict(left='PRESENT', right='ABSENT')
    barrier = [Segment((-9., 2.6), (9., 2.6))]                        # opaque wall hides the left side
    claims = _votes(own + beyond, barrier)
    assert claims['left'] == 'UNKNOWN' and claims['right'] == 'ABSENT'


def test_absence_beyond_the_row_start_is_not_last_row_evidence():
    # The next row exists only alongside this row (x >= 0); behind the row start nothing is there.
    own = row(1.75, np.arange(0., 7.6, 2.5)) + row(-1.75, np.arange(0., 7.6, 2.5))
    barrier = [Segment((0., 2.6), (9., 2.6))]                          # hides the next row alongside
    claims = _votes(own + row(5.25, np.arange(0., 7.6, 2.5)), barrier)
    assert claims['left'] == 'UNKNOWN'


def test_snapshot_reports_stillness_duration_and_last_row_claims():
    estimator = SensorEstimator()
    snap = estimator.snapshot(0.)
    assert snap['still'] is False and snap['still_s'] == 0.
    assert set(snap['next_corridor']) == {'left', 'right'}


def test_tracked_width_cannot_drift_below_the_minimum():
    estimator = SensorEstimator()
    estimator.band = Band(0., 0., 1.4, 0., 5)
    _, _, _, points = scan_of(row(.5, np.arange(-7.5, 7.6, 2.5)) + row(-.5, np.arange(-7.5, 7.6, 2.5)))
    for k in range(10):                                               # 1.0 m "corridor": within 0.5 m of 1.4
        estimator._apply(.2 * (k + 1), 0., 0., .5, -.5, points, 'candidate')
    assert estimator.band.width >= 1.2 - 1e-9


def test_learnt_width_expires_after_moving_away_from_the_lost_band():
    estimator = SensorEstimator()
    estimator.known_width, estimator.band_lost_at_m, estimator.distance = 3.5, 0., 4.
    _, _, _, points = scan_of(row(2.4, np.arange(-7.5, 7.6, 2.5)) + row(-2.4, np.arange(-7.5, 7.6, 2.5)))
    estimator._initialise(0., 0., 0., 2.3, -2.3, points)              # 4.6 m: rejected within 10 m
    assert estimator.corridor_reason == 'implausible_initial_width'
    estimator.distance = 14.5
    estimator._initialise(.1, 0., 0., 2.3, -2.3, points)              # far away: another area
    assert estimator.corridor_reason == 'initialising' and estimator.known_width is None


def _matrix_start_scans(name, seed, yaw_deg, corridor=0, count=1):
    """The first scans of a matrix episode (vehicle at rest), as the harness synthesises them."""
    from closed_loop.harness import stream_seed
    from closed_loop.matrix import episodes
    from closed_loop.world import build_world, make_lidar, yaw_quaternion
    config = episodes(seeds=(seed,), modes=('hybrid',), names=[name])[0]
    world = build_world(config.world)
    caster, lidar = make_lidar(world, seed=stream_seed(config, 'lidar_noise'))
    x, y, yaw = world.pose_in_field(config.start_x_m, corridor * config.world.row_width_m, math.radians(yaw_deg))
    scans = []
    for k in range(1, count + 1):
        frame = lidar.sample(int(round(.1 * k * 1e9)), (x + .18 * math.cos(yaw), y + .18 * math.sin(yaw), .6),
                             yaw_quaternion(yaw))
        ranges = np.asarray(frame.ranges, dtype=float)
        ok = np.isfinite(ranges)
        angles = lidar.angle_min + np.arange(len(ranges)) * lidar.angle_increment
        scans.append(np.column_stack((ranges[ok] * np.cos(angles[ok]), ranges[ok] * np.sin(angles[ok]))))
    return scans


def test_acquisition_keeps_the_trunk_line_next_to_a_parallel_fence():
    # Full-length side barrier 0.875 m outside the left trunk line (matrix scenario, development seed 3):
    # snapping 0.8 m outward from the canopy fringe paired the right line with the barrier (4.2 m band)
    # on several of the first scans.
    for points in _matrix_start_scans('occluded_next_row_full', 3, 4., count=10):
        found = SensorEstimator()._acquire(points)
        assert found is None or found[1] - found[2] < 3.9            # the 3.5 m corridor, never 4.2 m


def test_sparse_two_sided_updates_still_confirm_a_row_end():
    trunks = row(1.75, np.arange(-7.5, 3.6, 2.5)) + row(-1.75, np.arange(-7.5, 3.6, 2.5))
    estimator, points = _tracked(scan_of(trunks))
    assert estimator._apply(.2, 0., 0., 1.62, -1.62, points, 'candidate')
    assert estimator._apply(1.8, 0., 0., 1.62, -1.62, points, 'candidate')   # 1.6 s later (sparse)
    estimator.last_scan_stamp = 1.8
    assert estimator.snapshot(1.85)['row_end_ahead_m'] is not None
