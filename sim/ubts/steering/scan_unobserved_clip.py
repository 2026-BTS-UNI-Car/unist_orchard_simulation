"""UNOBSERVED clip of the Gazebo LiDAR (design 5.2, 8.3 P6a / P7): /sim/scan -> /sim/scan_clipped.

The raster world (RASTER_V1) ends a ray WITHOUT a return where it enters an UNOBSERVED cell (or leaves the raster);
the Gazebo world has no geometry there, so the Embree LiDAR (pinned cpu_lidar.py, unchanged) would see through
unobserved space to whatever lies behind it. This node restores the raster rule: per ray it finds the distance to
the first UNOBSERVED cell of the field world's raster copy (field_world raster.npz; sensor-synthesis truth, plant
side, like cpu_lidar's use of the true pose) along the ray, and a ray whose Embree return lies at or beyond that
distance becomes 'no return' (+inf). Returns in front of the first unobserved cell are kept unchanged by the clip.

Range noise (2026-11-09 review RW-04): the F-SIM ends a ray at UNOBSERVED on the exact range and adds the range noise
after (RasterLidarCaster, then the ScanSynthesizer contract). If cpu_lidar adds its noise first, a return just in
front of the first UNOBSERVED cell can be clipped and one just behind it kept (about 0.03-0.05 % of the rays of the
A7 sample). So in the steering stack cpu_lidar runs with noise 0 (a runtime copy of rover.yaml, supervisor_steer) and
this node adds N(0, --noise-sd) after the clip with the ScanSynthesizer contract (finite returns only, then the
minimum range clamp and +inf above the maximum range). --noise-sd 0 (the default) adds nothing.

  clip_ranges(raster, origin_w, directions_w, ranges)   the pure clip (numpy, no ROS)
  add_range_noise(ranges, sd, rng, range_min, range_max)  the noise contract after the clip
  ClipProcessor                                         poses by stamp, scan -> clipped (+ noise) ranges (no ROS)
  main()                                                the ROS 2 node (rclpy; SIM container)

Ray geometry is the cpu_lidar rule: the scan origin = base pose + R(base) x the rover.yaml LiDAR mount, ray
directions = R(base) x (cos a, sin a, 0) with a = angle_min + i x angle_increment. The raster is 2D: a ray is
followed along its horizontal projection and its range r is compared as the horizontal distance r x |d_h|
(exact for the flat scan plane; for a tilted scan the 2D raster rule is applied to the projection).
A terminator at the same distance as the return (an exact corner) also clips (the raster tie rule: a terminator
wins a tie), with a tolerance of CLIP_TOLERANCE_M for the float32 Embree range.

ROS interface (node ubts_steer_scan_clip; p6a_graph.CLIP_NODE):
  subscribes  --input (default /sim/scan; sensor_msgs/LaserScan, cpu_lidar output) and --pose-topic (default
              /ground_truth/odom; nav_msgs/Odometry, truth, plant side; cpu_lidar stamps each scan with the stamp of
              the pose it used, so the pose of a scan is the odometry message with the same stamp)
  publishes   --output (default /sim/scan_clipped; sensor_msgs/LaserScan: the header, angles and limits of the input,
              ranges with the clipped rays at +inf and the noise added after the clip) for the S2 format node of the
              next phase (which publishes /scan)
  status      --status-file (default /runtime/scan_clip_status.json): state (waiting, running, stopped), counts
              (scans, published, clipped rays, scans without a pose), last and mean processing ms; a scan without a
              matching pose is NOT published (an unclipped scan would see through unobserved space)
The clip node and the raster are plant side: the executor never subscribes to /sim/* or /ground_truth/* (A13).
Placeholder-based SIM results, not field validation.
CLI (container; the supervisor's command for field1010 scenes, supervisor_steer.clip_node_command):
  python3 -B -m steering.scan_unobserved_clip --input /sim/scan --output /sim/scan_clipped
      --raster <scene>/field_world/raster.npz [--status-file F] [--noise-sd SD] [--noise-seed N] [--config rover.yaml]
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

import numpy as np

CLIP_TOLERANCE_M = 1e-4
INPUT_TOPIC = '/sim/scan'
POSE_TOPIC = '/ground_truth/odom'
OUTPUT_TOPIC = '/sim/scan_clipped'
NODE_NAME = 'ubts_steer_scan_clip'
POSE_BUFFER = 400
PENDING_SCANS = 8          # scans waiting for the pose of their stamp (the executor can take a scan before its pose)
POSE_INTERP_MAX_NS = 50_000_000   # a lost pose (best effort): interpolate between neighbours at most 50 ms apart
ROOT = Path(__file__).resolve().parents[1]


class ClipRaster:
    """Terminator grid of a raster copy: UNOBSERVED -> TERMINATE, everything else PASS (reflectors do not stop the
    clip search; the Embree range already is the first reflector). boundary_*: terminator cells next to a passing
    cell (a ray from observed space must cross one of them first), for the per-scan nearest-terminator bound."""

    def __init__(self, raster):
        from field_world import raster_ops as ro
        if isinstance(raster, (str, Path)):
            from field_world.build_field_world import RasterCopy
            raster = RasterCopy(raster)
        self.source = getattr(raster, 'path', None)
        self.sha256 = getattr(raster, 'sha256', None)
        self.resolution = float(raster.resolution)
        self.row0, self.col0 = int(raster.row0), int(raster.col0)
        self.transform = raster.transform
        unobserved = int(getattr(raster, 'unobserved_class', 2))
        classes = np.asarray(raster.classes)
        self.kinds = np.where(classes == unobserved, ro.TERMINATE, ro.PASS).astype(np.uint8)
        term = self.kinds == ro.TERMINATE
        passing = ~term
        near = np.zeros_like(passing)
        h, w = passing.shape
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                shifted = np.zeros_like(passing)
                shifted[max(dr, 0):h + min(dr, 0), max(dc, 0):w + min(dc, 0)] = \
                    passing[max(-dr, 0):h + min(-dr, 0), max(-dc, 0):w + min(-dc, 0)]
                near |= shifted
        rows, cols = np.nonzero(term & near)
        self.boundary_x = (cols + .5) * self.resolution          # grid frame (from the crop corner)
        self.boundary_y = (rows + .5) * self.resolution
        self.shape = classes.shape

    def to_grid(self, x_w, y_w):
        """W -> the grid frame of the crop (metres from its lower-left corner)."""
        x_l, y_l = self.transform.to_l(x_w, y_w)
        return (np.asarray(x_l) - self.col0 * self.resolution, np.asarray(y_l) - self.row0 * self.resolution)

    def nearest_terminator_m(self, gx, gy):
        if not self.boundary_x.size:
            return math.inf
        d = np.hypot(self.boundary_x - gx, self.boundary_y - gy).min()
        return float(d) - self.resolution / math.sqrt(2.)


def clip_ranges(raster, origin_w, directions_w, ranges, tolerance=CLIP_TOLERANCE_M):
    """Clipped copy of ranges (+inf = no return) and a dict of counts.

    raster        ClipRaster (or anything ClipRaster accepts)
    origin_w      scan origin (x, y[, z]) in the field world frame W
    directions_w  (N, 3) or (N, 2) ray directions in W (need not be unit)
    ranges        (N,) Embree ranges (inf / nan = no return)"""
    from field_world import raster_ops as ro
    raster = raster if isinstance(raster, ClipRaster) else ClipRaster(raster)
    r = np.array(ranges, dtype=float, copy=True)
    r[~np.isfinite(r)] = math.inf
    d = np.asarray(directions_w, dtype=float)
    dx, dy = d[:, 0], d[:, 1]
    norm3 = np.linalg.norm(d, axis=1) if d.shape[1] == 3 else np.hypot(dx, dy)
    horizontal = np.hypot(dx, dy) / np.maximum(norm3, 1e-15)
    ux_w = np.divide(dx, np.hypot(dx, dy), out=np.zeros_like(dx), where=np.hypot(dx, dy) > 0)
    uy_w = np.divide(dy, np.hypot(dx, dy), out=np.zeros_like(dy), where=np.hypot(dx, dy) > 0)
    ux, uy = raster.transform.dir_to_l(ux_w, uy_w)
    gx, gy = raster.to_grid(float(origin_w[0]), float(origin_w[1]))
    gx, gy = float(gx), float(gy)
    hit = np.isfinite(r)
    reach = r * horizontal
    info = dict(rays=int(len(r)), returns_in=int(hit.sum()), dda_rays=0, clipped=0)
    if not hit.any():
        info['returns_out'] = 0
        return r, info
    col, row = int(math.floor(gx / raster.resolution)), int(math.floor(gy / raster.resolution))
    h, w = raster.kinds.shape
    if not (0 <= row < h and 0 <= col < w) or raster.kinds[row, col] != ro.PASS:
        info['clipped'] = int(hit.sum())                   # the sensor itself is in unobserved space: no return
        info['returns_out'] = 0
        return np.full(len(r), math.inf), info
    bound = raster.nearest_terminator_m(gx, gy)
    need = hit & (reach + tolerance >= bound)
    if need.any():
        ids = np.flatnonzero(need)
        dist, kind = ro.first_stop(raster.kinds, raster.resolution, gx, gy, ux[ids], uy[ids], reach[ids] + tolerance)
        clip = (kind != ro.PASS) & (dist <= reach[ids] + tolerance)
        r[ids[clip]] = math.inf
        info['dda_rays'] = int(len(ids))
        info['clipped'] = int(clip.sum())
    info['returns_out'] = int(np.isfinite(r).sum())
    return r, info


def scan_directions(rotation, angle_min, angle_increment, count):
    """World-frame directions of the cpu_lidar rays: R x (cos a, sin a, 0)."""
    angles = angle_min + np.arange(count) * angle_increment
    local = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(count)))
    return local @ np.asarray(rotation, dtype=float).T


def lidar_mount(config_path=None):
    """(x, y, z) LiDAR mount of rover.yaml (the mount cpu_lidar uses)."""
    import yaml
    config = yaml.safe_load(Path(config_path or (ROOT / 'rover.yaml')).read_text(encoding='utf-8'))
    lidar = config['lidar']
    return float(lidar['x_m']), float(lidar['y_m']), float(lidar['height_above_ground_m'])


def add_range_noise(ranges, sd, rng, range_min, range_max):
    """The ScanSynthesizer range contract after the clip: N(0, sd) on the finite returns, then the minimum range clamp
    and +inf above the maximum range. sd 0 changes nothing."""
    r = np.array(ranges, dtype=float, copy=True)
    if not sd:
        return r
    valid = np.isfinite(r)
    r[valid] += rng.normal(0., float(sd), int(valid.sum()))
    r[valid] = np.maximum(r[valid], float(range_min))
    r[r > float(range_max)] = math.inf
    return r


class ClipProcessor:
    """Poses by stamp and scans -> published ranges, without ROS (the node's logic; tested directly).

    pose(stamp_ns, position, quaternion) keeps the last POSE_BUFFER poses and returns [(token, ranges, info)] for a
    waiting scan of that stamp; scan(stamp_ns, angle_min, angle_increment, ranges, range_min, range_max, token)
    returns (ranges or None, info): the clipped ranges with the noise added after the clip, or None while no pose
    has the scan's stamp. Such a scan waits (at most PENDING_SCANS) for its pose: the single-threaded executor takes
    one message per subscription per wait, so a busy node can see a scan before the pose of its stamp (first W2
    stack run: 3.4 % of the scans were dropped this way). When a newer pose comes first, the scan's own pose was lost
    (best-effort pose topic, second W2 run under load: 13 of 383 scans): the pose is interpolated between the two
    neighbouring poses (position linear, quaternion normalised linear) when they are at most POSE_INTERP_MAX_NS
    apart (counts 'interpolated'), else the scan is dropped (unmatched). More than PENDING_SCANS waiting: the oldest
    is dropped."""

    def __init__(self, raster, mount, noise_sd=0., noise_seed=7, buffer=POSE_BUFFER):
        self.raster = raster if isinstance(raster, ClipRaster) else ClipRaster(raster)
        self.mount = np.asarray(mount, dtype=float)
        self.noise_sd = float(noise_sd)
        if not (math.isfinite(self.noise_sd) and self.noise_sd >= 0.):
            raise ValueError('noise sd must be a finite number >= 0')
        self.rng = np.random.default_rng(int(noise_seed))
        self.buffer = int(buffer)
        self.poses = {}
        self.order = []
        self.pending = {}                               # stamp -> (scan arguments, token), insertion order
        self.counts = dict(scans=0, published=0, unmatched=0, late_matched=0, interpolated=0, waiting=0,
                           clipped_rays=0, dda_rays=0, last_ms=None, mean_ms=None)
        self._ms_total = 0.

    def pose(self, stamp_ns, position, quaternion):
        stamp_ns = int(stamp_ns)
        if stamp_ns not in self.poses:
            self.order.append(stamp_ns)
        self.poses[stamp_ns] = (tuple(float(v) for v in position), tuple(float(v) for v in quaternion))
        while len(self.order) > self.buffer:
            self.poses.pop(self.order.pop(0), None)
        released = []
        for stamp in [k for k in self.pending if k <= stamp_ns]:
            args, token = self.pending.pop(stamp)
            if stamp == stamp_ns:
                out, info = self._clip(self.poses[stamp_ns], *args)
                self.counts['late_matched'] += 1
                released.append((token, out, info))
                continue
            pose = self._interpolated(stamp)            # a newer pose came first: the scan's own pose was lost
            if pose is None:
                self.counts['unmatched'] += 1
                continue
            out, info = self._clip(pose, *args)
            self.counts['interpolated'] += 1
            released.append((token, out, dict(info, interpolated=True)))
        self.counts['waiting'] = len(self.pending)
        return released

    def _interpolated(self, stamp_ns):
        """The pose at stamp_ns between the nearest buffered poses before and after it (None without both or when
        they are more than POSE_INTERP_MAX_NS apart)."""
        before = max((k for k in self.order if k < stamp_ns), default=None)
        after = min((k for k in self.order if k > stamp_ns), default=None)
        if before is None or after is None or after - before > POSE_INTERP_MAX_NS:
            return None
        w = (stamp_ns - before) / float(after - before)
        (p0, q0), (p1, q1) = self.poses[before], self.poses[after]
        if sum(a * b for a, b in zip(q0, q1)) < 0.:     # the shorter way round
            q1 = tuple(-v for v in q1)
        q = [a + w * (b - a) for a, b in zip(q0, q1)]
        norm = math.sqrt(sum(v * v for v in q))
        return tuple(a + w * (b - a) for a, b in zip(p0, p1)), tuple(v / norm for v in q)

    def scan(self, stamp_ns, angle_min, angle_increment, ranges, range_min, range_max, token=None):
        self.counts['scans'] += 1
        stamp_ns = int(stamp_ns)
        pose = self.poses.get(stamp_ns)
        if pose is None and self.order and max(self.order) > stamp_ns:
            # newer poses are here already: the pose of this stamp was lost (poses arrive in order)
            pose = self._interpolated(stamp_ns)
            if pose is None:
                self.counts['unmatched'] += 1
                return None, dict(matched=False, waiting=False)
            out, info = self._clip(pose, angle_min, angle_increment, ranges, range_min, range_max)
            self.counts['interpolated'] += 1
            return out, dict(info, interpolated=True)
        if pose is None:
            self.pending[stamp_ns] = ((angle_min, angle_increment, ranges, range_min, range_max), token)
            while len(self.pending) > PENDING_SCANS:
                self.pending.pop(next(iter(self.pending)))
                self.counts['unmatched'] += 1
            self.counts['waiting'] = len(self.pending)
            return None, dict(matched=False, waiting=stamp_ns in self.pending)
        return self._clip(pose, angle_min, angle_increment, ranges, range_min, range_max)

    def _clip(self, pose, angle_min, angle_increment, ranges, range_min, range_max):
        from cpu_lidar import quaternion_rotation       # pinned module, imported only
        started = time.perf_counter()
        rotation = quaternion_rotation(pose[1])
        origin = np.asarray(pose[0], dtype=float) + rotation @ self.mount
        count = len(ranges)
        directions = scan_directions(rotation, angle_min, angle_increment, count)
        clipped, info = clip_ranges(self.raster, origin, directions, np.asarray(ranges, dtype=float))
        out = add_range_noise(clipped, self.noise_sd, self.rng, range_min, range_max)
        elapsed = (time.perf_counter() - started) * 1000.
        self._ms_total += elapsed
        self.counts['published'] += 1
        self.counts['clipped_rays'] += info['clipped']
        self.counts['dda_rays'] += info['dda_rays']
        self.counts['last_ms'] = round(elapsed, 3)
        self.counts['mean_ms'] = round(self._ms_total / self.counts['published'], 3)
        return out, dict(info, matched=True, ms=elapsed)


def build_parser():
    parser = argparse.ArgumentParser(prog='scan_unobserved_clip', description=__doc__.split('\n')[0])
    parser.add_argument('--input', default=INPUT_TOPIC, help='LaserScan in (default %s)' % INPUT_TOPIC)
    parser.add_argument('--output', default=OUTPUT_TOPIC, help='clipped LaserScan out (default %s)' % OUTPUT_TOPIC)
    parser.add_argument('--pose-topic', default=POSE_TOPIC, help='truth pose (default %s)' % POSE_TOPIC)
    parser.add_argument('--raster', required=True, help='field world raster.npz of the scene')
    parser.add_argument('--config', default=str(ROOT / 'rover.yaml'), help='rover.yaml (LiDAR mount)')
    parser.add_argument('--status-file', default='/runtime/scan_clip_status.json')
    parser.add_argument('--noise-sd', type=float, default=0., help='range noise sd added after the clip (m)')
    parser.add_argument('--noise-seed', type=int, default=7)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from cpu_lidar import write_status

    processor = ClipProcessor(ClipRaster(args.raster), lidar_mount(args.config), noise_sd=args.noise_sd,
                              noise_seed=args.noise_seed)
    raster = processor.raster
    base = dict(simulated=True, node=NODE_NAME, input_topic=args.input, pose_topic=args.pose_topic,
                output_topic=args.output, raster_sha256=raster.sha256, raster_shape=list(raster.shape),
                noise_sd_m=args.noise_sd, noise_after_clip=True,
                rule='ray -> no return at or beyond the first UNOBSERVED cell (RASTER_V1), then the range noise')
    qos_out = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
    qos_in = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
    qos_pose = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE)

    class ClipNode(Node):
        def __init__(self):
            super().__init__(NODE_NAME, parameter_overrides=[Parameter('use_sim_time', value=False)])
            self.pub = self.create_publisher(LaserScan, args.output, qos_out)
            self.create_subscription(Odometry, args.pose_topic, self.pose, qos_pose)
            self.create_subscription(LaserScan, args.input, self.scan, qos_in)
            self.last_status = 0.
            self.report('waiting')

        def report(self, state='running'):
            write_status(args.status_file, dict(base, **processor.counts, state=state, updated_at=time.time()))
            self.last_status = time.monotonic()

        def pose(self, message):
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            p, q = message.pose.pose.position, message.pose.pose.orientation
            for scan_message, ranges, _ in processor.pose(stamp, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)):
                self.publish(scan_message, ranges)          # a scan that waited for this pose

        def scan(self, message):
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            ranges, _ = processor.scan(stamp, message.angle_min, message.angle_increment, message.ranges,
                                       message.range_min, message.range_max, token=message)
            if ranges is not None:
                self.publish(message, ranges)
            if time.monotonic() - self.last_status >= 1.:
                self.report()

        def publish(self, message, ranges):
            out = LaserScan()
            out.header = message.header
            out.angle_min, out.angle_max, out.angle_increment = (message.angle_min, message.angle_max,
                                                                 message.angle_increment)
            out.time_increment, out.scan_time = message.time_increment, message.scan_time
            out.range_min, out.range_max = message.range_min, message.range_max
            out.ranges = ranges.astype(np.float32).tolist()
            out.intensities = list(message.intensities)
            self.pub.publish(out)

    rclpy.init(args=[])
    node = ClipNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report('stopped')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
