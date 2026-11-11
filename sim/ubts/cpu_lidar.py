"""ROS wrapper for the simulator's physical 2D ray sensor, without GPU readback.

Ground-truth model pose is used only to place this simulated sensor in the
rendered world, exactly as Gazebo does. Mapping and control still consume the
wheel-odometry frame; this module never publishes navigation odometry.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import time

# Keep the small 3D matrix operations from starting a BLAS worker pool for
# every scan. The supervisor also enforces these only for this child process.
for _thread_variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_thread_variable, '1')
import numpy as np


ROOT = Path(__file__).resolve().parent


def quaternion_rotation(quaternion):
    """ROS xyzw quaternion to a full, proper 3D rotation matrix."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('invalid_pose_quaternion')
    norm = float(np.linalg.norm(q))
    if norm < 1e-8 or abs(norm-1.) > .05:
        raise ValueError('non_unit_pose_quaternion')
    x, y, z, w = q/norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


@dataclass(frozen=True)
class ScanFrame:
    stamp_ns: int
    ranges: np.ndarray
    origin: np.ndarray
    directions: np.ndarray
    raycast_ms: float


class ScanSynthesizer:
    """ROS-free timestamp gating and range/noise contract around PlanarScanner.

    The scanner must implement ``scan(xyz, rotation=3x3)`` and return
    ``ranges, world_origin, world_directions``. Duplicate truth samples never
    generate another scan. A backward simulator clock starts a fresh phase.
    """

    def __init__(self, scanner, count=3200, hz=10., minimum=.05, maximum=30.,
                 noise_stddev=.01, seed=7):
        if count < 2 or hz <= 0 or not 0 < minimum < maximum or noise_stddev < 0:
            raise ValueError('invalid_sensor_configuration')
        if not all(math.isfinite(v) for v in (hz, minimum, maximum, noise_stddev)):
            raise ValueError('invalid_sensor_configuration')
        self.scanner, self.count = scanner, int(count)
        self.hz, self.minimum, self.maximum = float(hz), float(minimum), float(maximum)
        self.noise_stddev = float(noise_stddev)
        self.period_ns = max(1, round(1e9/self.hz))
        self.rng = np.random.default_rng(seed)
        self.last_seen_ns = None
        self.next_due_ns = None
        self.published = self.duplicates = self.rate_skips = self.clock_resets = 0
        self.last_frame = None

    @property
    def angle_increment(self):
        return 2*math.pi/self.count

    @property
    def angle_min(self):
        return -math.pi

    @property
    def angle_max(self):
        return self.angle_min+(self.count-1)*self.angle_increment

    def sample(self, stamp_ns, position, quaternion):
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, (int, np.integer)) or stamp_ns < 0:
            raise ValueError('invalid_truth_timestamp')
        stamp_ns = int(stamp_ns)
        if self.last_seen_ns is not None:
            if stamp_ns < self.last_seen_ns:
                self.clock_resets += 1
                self.next_due_ns = None
            elif stamp_ns == self.last_seen_ns:
                self.duplicates += 1
                return None
        self.last_seen_ns = stamp_ns
        if self.next_due_ns is not None and stamp_ns < self.next_due_ns:
            self.rate_skips += 1
            return None
        xyz = np.asarray(position, dtype=float)
        if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
            raise ValueError('invalid_truth_position')
        rotation = quaternion_rotation(quaternion)
        started = time.perf_counter()
        raw, origin, directions = self.scanner.scan(xyz, rotation=rotation)
        elapsed = (time.perf_counter()-started)*1000
        ranges = np.asarray(raw, dtype=float).copy()
        if ranges.shape != (self.count,) or np.any(np.isnan(ranges)) or np.any(ranges < 0.):
            raise ValueError('invalid_raycast_output')
        origin = np.asarray(origin, dtype=float)
        directions = np.asarray(directions, dtype=float)
        if origin.shape != (3,) or directions.shape != (self.count, 3) or not np.all(np.isfinite(origin)) or not np.all(np.isfinite(directions)):
            raise ValueError('invalid_raycast_geometry')
        valid = np.isfinite(ranges) & (ranges <= self.maximum)
        ranges[~valid] = math.inf
        # A real surface closer than the minimum range still blocks the beam.
        # Keep it conservatively at the nearest measurable range, never free.
        ranges[valid] = np.maximum(ranges[valid], self.minimum)
        if self.noise_stddev:
            ranges[valid] += self.rng.normal(0., self.noise_stddev, int(valid.sum()))
            ranges[valid] = np.maximum(ranges[valid], self.minimum)
            ranges[ranges > self.maximum] = math.inf
        frame = ScanFrame(stamp_ns, ranges.astype(np.float32), origin, directions, elapsed)
        self.last_frame = frame
        self.published += 1
        # Anchor the schedule to simulator time instead of accumulating odom
        # update jitter. A delayed pose produces one scan, never a catch-up burst.
        self.next_due_ns = (stamp_ns//self.period_ns+1)*self.period_ns
        return frame

    def status(self):
        frame = self.last_frame
        finite = np.isfinite(frame.ranges) if frame is not None else None
        return {'published_scans': self.published, 'duplicate_samples_skipped': self.duplicates,
                'rate_samples_skipped': self.rate_skips, 'clock_resets': self.clock_resets,
                'target_sim_hz': self.hz, 'points_per_scan': self.count,
                'last_stamp_ns': frame.stamp_ns if frame is not None else None,
                'last_raycast_ms': frame.raycast_ms if frame is not None else None,
                'finite_returns': int(finite.sum()) if finite is not None else 0}


def write_status(path, payload):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix('.tmp')
    temporary.write_text(json.dumps(payload, allow_nan=False), encoding='utf-8')
    temporary.replace(target)


def main():
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from mesh_raycast import StaticVisualWorld, PlanarScanner

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='/runtime/orchard.sdf')
    parser.add_argument('--config', default=str(ROOT/'rover.yaml'))
    parser.add_argument('--status-file', default='/runtime/cpu_lidar_status.json')
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text())
    lidar = config['lidar']
    base_status = {'simulated': True, 'backend': 'embree', 'pose_source': '/ground_truth/odom',
                   'output_topic': '/sim/scan', 'scene_source': 'static_visual_triangles',
                   'navigation_pose_source': 'wheel_distance_imu_heading'}
    write_status(args.status_file, {**base_status, 'state': 'loading', 'updated_at': time.time()})
    world = StaticVisualWorld(args.world)
    scanner = PlanarScanner(world.rays, count=int(lidar['samples']), minimum=float(lidar['min_m']),
                            maximum=float(lidar['max_m']), mount=(float(lidar['x_m']), float(lidar['y_m']),
                                                                  float(lidar['height_above_ground_m'])))
    # Embree builds its BVH lazily on the first intersection. Warm it before
    # subscribing so an old truth sample is never published after that delay.
    warming = time.perf_counter()
    scanner.scan((0., 0., 0.), rotation=np.eye(3))
    bvh_warmup_seconds = time.perf_counter()-warming
    synth = ScanSynthesizer(scanner, count=int(lidar['samples']), hz=float(lidar['hz']),
                            minimum=float(lidar['min_m']), maximum=float(lidar['max_m']),
                            noise_stddev=float(lidar.get('noise_stddev_m', .01)),
                            seed=int(config.get('world', {}).get('seed', 7)))
    description = world.describe()
    # File locations are implementation details; the runtime health report only
    # needs scene size, skipped dynamic models and timings.
    scene = {key: description[key] for key in ('triangles', 'vertices', 'bounds',
             'static_visual_count', 'skipped_dynamic_models', 'mesh_load_seconds',
             'rss_after_load_mb') if key in description}
    scene['bvh_warmup_seconds'] = bvh_warmup_seconds
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

    class CpuLidarNode(Node):
        def __init__(self):
            # Synthesizer cadence and output stamps use the incoming pose's
            # exact simulation stamp, including rewind. No node clock/timer is
            # consumed, so avoid a separate 120Hz /clock subscription.
            super().__init__('ubts_cpu_lidar', parameter_overrides=[Parameter('use_sim_time', value=False)])
            self.publisher = self.create_publisher(LaserScan, '/sim/scan', qos)
            self.subscription = self.create_subscription(Odometry, '/ground_truth/odom', self.pose_received, qos)
            self.last_status = self.last_warning = 0.
            self.last_error = None
            self.errors = 0
            self.report('waiting_for_pose')

        def report(self, state='running'):
            write_status(args.status_file, {**base_status, **synth.status(), 'scene': scene,
                          'state': state, 'updated_at': time.time(), 'errors': self.errors,
                          'last_error': self.last_error})
            self.last_status = time.monotonic()

        def pose_received(self, message):
            stamp = message.header.stamp
            pose = message.pose.pose
            p, q = pose.position, pose.orientation
            try:
                if message.header.frame_id != 'world' or message.child_frame_id != 'base_footprint':
                    raise ValueError('truth_pose_frame_mismatch')
                if not 0 <= int(stamp.nanosec) < 1_000_000_000:
                    raise ValueError('invalid_truth_timestamp')
                frame = synth.sample(int(stamp.sec)*1_000_000_000+int(stamp.nanosec),
                                     (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
                if frame is None:
                    return
                scan = LaserScan()
                scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(frame.stamp_ns, 1_000_000_000)
                scan.header.frame_id = 'laser'
                scan.angle_min, scan.angle_max, scan.angle_increment = synth.angle_min, synth.angle_max, synth.angle_increment
                scan.time_increment, scan.scan_time = 0., 1./synth.hz
                scan.range_min, scan.range_max = synth.minimum, synth.maximum
                scan.ranges = frame.ranges.tolist()
                scan.intensities = []
                self.publisher.publish(scan)
                self.last_error = None
                if time.monotonic()-self.last_status >= 1.:
                    self.report()
            except (ValueError, RuntimeError) as error:
                self.errors += 1
                self.last_error = str(error)
                if time.monotonic()-self.last_warning >= 5.:
                    self.get_logger().error(f'CPU LiDAR sample rejected: {error}')
                    self.last_warning = time.monotonic()
                if time.monotonic()-self.last_status >= 1.:
                    self.report('sample_error')

    rclpy.init(args=[])
    node = CpuLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report('stopped')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
