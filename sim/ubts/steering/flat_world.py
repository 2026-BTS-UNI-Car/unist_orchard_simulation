"""W0 flat world of the steering fast SIM (design 5.1 W0: flat calibration floor, optional LiDAR posts).

The world has the interface closed_loop.harness reads from a world (the same names as closed_loop.world.World
and closed_loop.raster_corridor.RasterCorridorWorld):

  config            FlatWorldConfig (scenario, row_width_m, headland_depth_m are read by the harness)
  to_field(x, y, yaw) / pose_in_field(fx, fy, fyaw)   identity: the field frame IS the world frame
  corridor_truth(k) end_x far away (end_x_m), no cross walls, next corridors UNKNOWN (W0 has no rows)
  clearance(...)    (hard, branch) footprint clearance to the posts (branch: none, inf)

plus the two harness hooks make_flat_lidar(world, seed) -> (caster, cpu_lidar.ScanSynthesizer) and
flat_clearance(world, x, y, yaw, length, width), and FlatHooks.kwargs() for run_episode.

Posts are hard vertical cylinders (x, y, radius) in the world frame; registration_posts() gives a few along both
sides of the x axis for LiDAR registration (design 5.1 "LiDAR 정합용 기둥 몇 개"). Without posts every ray has no
return. The caster is exact ray/circle intersection with numpy (no scipy), so it also runs on Python 3.10.
The LiDAR beam count, range and noise are the SIM values of closed_loop.world (3200 SIM beams; the S2 format
conversion is P4). Truth (pose, posts) is used here only to make sensor data and to score runs.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

LIDAR_RAYS = 3200            # closed_loop.world.LIDAR_RAYS (the real S2 emits 3240; S2 format is P4)
LIDAR_NOISE_SD_M = .015      # closed_loop.world.LIDAR_NOISE_SD_M (SIM_ASSUMED)
LIDAR_MAX_RANGE_M = 30.
LIDAR_HZ = 10.
SCENARIO = 'W0_flat'
SENSING_MODEL = 'W0: posts (vertical cylinders) reflect; everything else gives no return'


@dataclass(frozen=True)
class FlatWorldConfig:
    """Harness world config of W0. row_width_m and headland_depth_m only satisfy the names the harness reads
    (W0 has no rows: the harness 'left_corridor' metric uses row_width_m / 2 and is meaningless here)."""

    scenario: str = SCENARIO
    markers: tuple = ()              # ((x, y, radius), ...) hard posts, world frame (m)
    row_width_m: float = 100.
    headland_depth_m: float = 0.
    end_x_m: float = 1000.           # harness row end (far away: W0 has no row end)
    seed: int = 0                    # no random scene draws (kept for config plumbing)

    def __post_init__(self):
        posts = []
        for item in self.markers:
            x, y, r = (float(v) for v in item)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(r) and r > 0.):
                raise ValueError('flat_world_marker_invalid')
            posts.append((x, y, r))
        object.__setattr__(self, 'markers', tuple(posts))
        if not (self.row_width_m > 0. and self.end_x_m > 0. and self.headland_depth_m >= 0.):
            raise ValueError('flat_world_config_invalid')


def registration_posts(count=8, spacing_m=6., offset_m=4., first_x_m=6., radius_m=.1):
    """Posts alternating left / right of the x axis: (first_x + k * spacing, +-offset, radius), k = 0..count-1."""
    return tuple((first_x_m + k * spacing_m, offset_m if k % 2 == 0 else -offset_m, radius_m) for k in range(count))


class FlatWorld:
    """W0 world for closed_loop.harness and the fast_sim runner (field frame = world frame)."""

    def __init__(self, config=None):
        self.config = FlatWorldConfig() if config is None else config
        self.posts = np.asarray(self.config.markers, dtype=float).reshape(-1, 3)
        self.contacts = dict(samples=0, hard=0)
        self.min_hard_clearance = math.inf
        self.lidar_stats = None

    def to_field(self, x, y, yaw=0.):
        return x, y, yaw

    def pose_in_field(self, fx, fy, fyaw=0.):
        return fx, fy, fyaw

    def corridor_truth(self, k):
        if k != 0:
            raise ValueError('the flat world has one (row-less) corridor, index 0')
        end = float(self.config.end_x_m)
        return dict(end_x=end, line_end_x=(end, end), cross_walls_x=(), centre_y=0., start_x=0.,
                    next_left='UNKNOWN', next_right='UNKNOWN')

    def clearance(self, x, y, yaw, length, width):
        """(hard, branch): signed distance from the footprint rectangle to the nearest post (inf: no post);
        no branches in W0. Contact (hard <= 0) is counted on the world."""
        hard = math.inf
        if len(self.posts):
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = self.posts[:, 0] - x, self.posts[:, 1] - y
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            ox, oy = np.abs(lx) - length / 2., np.abs(ly) - width / 2.
            outside = np.hypot(np.maximum(ox, 0.), np.maximum(oy, 0.))
            inside = np.maximum(ox, oy)
            distance = np.where(outside > 0., outside, inside) - self.posts[:, 2]
            hard = float(distance.min())
        self.contacts['samples'] += 1
        if hard <= 0.:
            self.contacts['hard'] += 1
        self.min_hard_clearance = min(self.min_hard_clearance, hard)
        return hard, math.inf

    def describe(self):
        return dict(scenario=self.config.scenario, sensing_model=SENSING_MODEL,
                    posts=[list(p) for p in self.config.markers], contact_samples=dict(self.contacts),
                    min_hard_clearance_m=round(self.min_hard_clearance, 3) if math.isfinite(self.min_hard_clearance)
                    else None, physical_truth=False)


class FlatLidarCaster:
    """cpu_lidar.ScanSynthesizer scanner over the posts: scan(xyz, rotation) -> (ranges, origin, directions).

    Exact ray / circle intersection in the scan plane; the nearest post wins; a sensor inside a post sees 0 m.
    occluded_sectors ((lo, hi) rad, laser frame) give no return (the harness sets them)."""

    def __init__(self, world, rays=LIDAR_RAYS, maximum=LIDAR_MAX_RANGE_M):
        self.world = world
        self.rays = int(rays)
        self.maximum = float(maximum)
        self.angles = -math.pi + np.arange(self.rays) * (2. * math.pi / self.rays)
        self.occluded_sectors = ()

    def cast(self, sx, sy, yaw):
        out = np.full(self.rays, math.inf)
        posts = self.world.posts
        if not len(posts):
            return out
        cx, cy, r = posts[:, 0] - sx, posts[:, 1] - sy, posts[:, 2]
        inside = cx * cx + cy * cy <= r * r
        if inside.any():
            return np.zeros(self.rays)
        phi = self.angles + yaw
        dx, dy = np.cos(phi)[:, None], np.sin(phi)[:, None]
        b = dx * cx[None, :] + dy * cy[None, :]                      # projection of the centre on the ray
        disc = b * b - (cx * cx + cy * cy - r * r)[None, :]
        hit = (disc >= 0.) & (b > 0.)
        t = np.where(hit, b - np.sqrt(np.where(hit, disc, 0.)), math.inf)
        out = t.min(axis=1)
        out[out > self.maximum] = math.inf
        return out

    def scan(self, xyz, rotation):
        yaw = math.atan2(rotation[1][0], rotation[0][0])
        ranges = self.cast(float(xyz[0]), float(xyz[1]), yaw)
        for lo, hi in self.occluded_sectors:
            ranges[(self.angles >= lo) & (self.angles <= hi)] = math.inf
        directions = np.column_stack((np.cos(self.angles + yaw), np.sin(self.angles + yaw), np.zeros(self.rays)))
        return ranges, np.array((xyz[0], xyz[1], xyz[2]), dtype=float), directions


def make_flat_lidar(world, seed, noise_sd=LIDAR_NOISE_SD_M):
    """harness lidar_factory: (FlatLidarCaster, ScanSynthesizer) with the closed_loop.world SIM LiDAR values."""
    from cpu_lidar import ScanSynthesizer
    caster = FlatLidarCaster(world)
    synth = ScanSynthesizer(caster, count=LIDAR_RAYS, hz=LIDAR_HZ, minimum=.05, maximum=LIDAR_MAX_RANGE_M,
                            noise_stddev=noise_sd, seed=seed)
    return caster, synth


def flat_clearance(world, x, y, yaw, length, width):
    """harness clearance_evaluator: (hard, branch)."""
    return world.clearance(x, y, yaw, length, width)


def build_flat_world(config):
    """harness world_factory."""
    if not isinstance(config, FlatWorldConfig):
        raise TypeError('build_flat_world needs a FlatWorldConfig')
    return FlatWorld(config)


class FlatHooks:
    """The three harness hooks for W0; keeps the episode's world for the caller."""

    def __init__(self):
        self.world = None
        self.caster = None

    def world_factory(self, config):
        self.world = build_flat_world(config)
        return self.world

    def lidar_factory(self, world, seed):
        self.caster, lidar = make_flat_lidar(world, seed)
        return self.caster, lidar

    def kwargs(self):
        return dict(world_factory=self.world_factory, lidar_factory=self.lidar_factory,
                    clearance_evaluator=flat_clearance)
