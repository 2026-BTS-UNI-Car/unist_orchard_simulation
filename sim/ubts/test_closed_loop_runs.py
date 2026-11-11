"""Short headless closed-loop episodes (field plant + ray-cast LiDAR + IMU).

12 m rows keep the runtime small. These check behaviour (no collision,
bounded lateral error, no chatter, safe holds, sensor-only inputs), not
real-vehicle accuracy.
"""
import ast
import copy
from dataclasses import replace
from pathlib import Path

import cpu_lidar
import field_response
from closed_loop.controllers import RowController
from closed_loop.estimator import SensorEstimator
from closed_loop.harness import EpisodeConfig, run_episode
from closed_loop.world import WorldConfig

SHORT = WorldConfig(row_length_m=12.)
HERE = Path(__file__).resolve().parent


def short(**overrides):
    overrides.setdefault('world', SHORT)
    overrides.setdefault('max_time_s', 60.)
    return EpisodeConfig(**overrides)


def test_hybrid_follows_a_new_row_without_chatter_or_collision():
    result = run_episode(short(name='short_nominal', mode='hybrid', start_yaw_deg=5.))
    assert result['final_reason'] == 'row_end_candidate' and result['collisions'] == 0
    assert -.5 <= result['final_x_rel_row_end_m'] <= 1.5
    assert result['lateral_max_m'] < .25 and result['min_clearance_m'] > .5 and not result['left_corridor']
    assert result['rapid_flips'] <= 2 and result['chatter_per_min'] < 6
    assert result['turns'] == 0 and result['intermediate_stops'] == 0
    # Edge corridor of a 3-row block: the next corridor (left) exists and is observable.
    assert result['last_row_expected'] == 'NOT_LAST' and result['last_row_claim'] == 'NOT_LAST'
    # LiDAR odometry distance is a sensor estimate, checked against truth here only.
    assert abs(result['estimated_distance_m'] - result['truth_distance_m']) < .1 * result['truth_distance_m']


def test_interior_corridor_is_acquired_and_followed():
    world = replace(SHORT, row_count=6, row_width_m=3., tree_spacing_m=3.)   # columns as strong as rows
    result = run_episode(short(name='short_interior_grid', mode='hybrid', world=world, corridor_index=2,
                               start_yaw_deg=4.))
    assert result['final_reason'] == 'row_end_candidate' and result['collisions'] == 0
    assert result['lateral_max_m'] < .3 and not result['left_corridor'] and result['turns'] == 0


def test_large_entry_error_uses_one_bounded_pivot_in_hybrid_mode():
    hybrid = run_episode(short(name='short_heading40', mode='hybrid', start_yaw_deg=40., start_y_m=.3))
    assert hybrid['final_reason'] == 'row_end_candidate' and hybrid['collisions'] == 0
    assert hybrid['turns'] == 1 and hybrid['stall_aborts'] == 0
    continuous = run_episode(short(name='short_heading40', mode='continuous', start_yaw_deg=40., start_y_m=.3,
                                   max_time_s=20.))
    # Continuous-only cannot turn hard enough; it must stop before the trees.
    assert continuous['collisions'] == 0 and continuous['final_reason'] == 'front_blocked'


def test_wall_across_the_aisle_stops_before_it_and_is_not_a_row_end():
    world = replace(SHORT, scenario='wall_across_before_end')
    result = run_episode(short(name='short_wall', mode='hybrid', world=world, start_yaw_deg=3.))
    assert result['final_reason'] == 'front_blocked' and result['collisions'] == 0
    assert result['min_clearance_m'] > .2 and not result['row_end_claimed']
    assert result['final_x_rel_row_end_m'] < -world.wall_before_end_m
    assert .1 <= result['wall_gap_m'] <= 1.2


def test_stuck_forward_drive_ends_in_a_no_response_hold():
    result = run_episode(short(name='short_root', mode='hybrid', start_yaw_deg=3., forward_blocks=((6., 40.),),
                               max_time_s=30.))
    assert result['final_reason'] == 'drive_no_response' and result['end'] == 'terminal_hold'
    assert result['time_s'] < 12.                                 # held within seconds, not pushed to the limit


def test_lidar_blackout_holds_then_recovers():
    result = run_episode(short(name='short_blackout', mode='hybrid', start_yaw_deg=3.,
                               blackouts=((8., 9.5, 'lidar'),)))
    assert 'stale_lidar' in result['holds'] and result['collisions'] == 0
    assert result['final_reason'] == 'row_end_candidate'


def test_estimator_and_controller_never_touch_truth():
    forbidden = {'oracle', 'truth', 'plant', 'world', 'scene', 'harness', 'field_response',
                 'raycast_scan', 'footprint_clearance'}
    for name in ('estimator.py', 'controllers.py'):
        tree = ast.parse((HERE / 'closed_loop' / name).read_text(encoding='utf-8'))
        identifiers = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
        identifiers |= {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        modules = {alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
        modules |= {node.module or '' for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
        assert not (identifiers & forbidden), (name, identifiers & forbidden)
        assert not any(part in module for module in modules for part in ('world', 'harness', 'field_response',
                                                                          'scenarios', 'matrix')), modules
    # Sensor-only inputs: scans with their stamp and raw IMU samples.
    assert list(SensorEstimator.on_scan.__code__.co_varnames[1:6]) == [
        'stamp', 'ranges', 'angle_min', 'increment', 'range_max']
    assert list(SensorEstimator.on_imu.__code__.co_varnames[1:4]) == ['stamp', 'gyro', 'accel']


def test_harness_feeds_only_sensor_outputs_to_the_estimator_and_controller(monkeypatch):
    """Dynamic guard at the harness boundary (where truth lives)."""
    produced_imu, produced_scans = [], []
    original_imu = field_response.FieldPlant.imu_sample
    original_sample = cpu_lidar.ScanSynthesizer.sample

    def recording_imu(self):
        sample = original_imu(self)
        produced_imu.append(sample)
        return sample

    def recording_sample(self, *args, **kwargs):
        frame = original_sample(self, *args, **kwargs)
        if frame is not None:
            produced_scans.append(frame.ranges)
        return frame
    monkeypatch.setattr(field_response.FieldPlant, 'imu_sample', recording_imu)
    monkeypatch.setattr(cpu_lidar.ScanSynthesizer, 'sample', recording_sample)
    received_imu, received_scans, produced_snapshots, given = [], [], [], []

    class RecordingEstimator(SensorEstimator):
        def on_imu(self, stamp, gyro, accel):
            received_imu.append((stamp, gyro, tuple(accel)))
            return super().on_imu(stamp, gyro, accel)

        def on_scan(self, stamp, ranges, angle_min, increment, range_max):
            received_scans.append(ranges)
            return super().on_scan(stamp, ranges, angle_min, increment, range_max)

        def snapshot(self, now):
            result = super().snapshot(now)
            produced_snapshots.append(copy.deepcopy(result))
            return result

    class RecordingController(RowController):
        def step(self, now, s):
            given.append(copy.deepcopy(s))
            return super().step(now, s)

    run_episode(short(name='short_guard', mode='hybrid', start_yaw_deg=20., max_time_s=12.),
                estimator_factory=RecordingEstimator, controller_factory=RecordingController)
    produced = {(s['gyro_z_rad_s'], tuple(s['accel_m_s2'])): s['stamp_s'] for s in produced_imu}
    assert received_imu
    for stamp, gyro, accel in received_imu:                   # exactly a plant IMU sample, same stamp
        assert (gyro, accel) in produced and abs(produced[(gyro, accel)] - stamp) < 1e-9
    scan_ids = {id(ranges) for ranges in produced_scans}
    assert received_scans and all(id(ranges) in scan_ids for ranges in received_scans)
    assert given == produced_snapshots


def test_latency_jitter_delays_but_never_reorders_scans():
    stamps = []

    class RecordingEstimator(SensorEstimator):
        def on_scan(self, stamp, ranges, angle_min, increment, range_max):
            stamps.append(stamp)
            return super().on_scan(stamp, ranges, angle_min, increment, range_max)
    result = run_episode(short(name='short_jitter', mode='hybrid', start_yaw_deg=3., lidar_latency_s=.05,
                               lidar_latency_jitter_s=.25, max_time_s=8.), estimator_factory=RecordingEstimator)
    assert len(stamps) > 60 and all(b > a for a, b in zip(stamps, stamps[1:]))
    assert result['collisions'] == 0
