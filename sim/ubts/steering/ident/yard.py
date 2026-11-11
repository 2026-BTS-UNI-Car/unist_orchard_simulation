"""Calibration yard for the identification runs (plant side): a flat W0 floor with walls, panels and a few posts.

The field tests F1-F5 run on flat open ground with buildings and trees around; the LiDAR odometry of the analysis
needs line structure in two directions. The yard gives that: four perimeter walls, short panels and posts, all
vertical. Rays hit the nearest wall segment or post (exact intersection in the scan plane, numpy, no scipy); a
sensor inside a post sees 0 m. Ranges above the LiDAR maximum give no return (inf, SIM convention).

The world has what T1Rig reads from a W0 world (config.markers for the identity) and what flat_world.FlatWorld
offers for evaluation (clearance). Truth: this module is plant side (sensor synthesis); analysis code never imports it.
All dimensions are SIM choices (no field site is modelled).
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .. import flat_world as fw

YARD_KIND = 'W0'
SCENARIO = 'ident_yard'
LIDAR_RAYS = fw.LIDAR_RAYS
LIDAR_MAX_RANGE_M = fw.LIDAR_MAX_RANGE_M
LIDAR_HZ = fw.LIDAR_HZ


@dataclass(frozen=True)
class YardConfig:
    """Yard geometry (m, world frame). walls and panels: ((x1, y1, x2, y2), ...); markers: posts ((x, y, r), ...)."""

    walls: tuple = ((-30., -22., 30., -22.), (30., -22., 30., 22.), (30., 22., -30., 22.), (-30., 22., -30., -22.))
    panels: tuple = ((-27., -4., -27., -1.), (-27., 1., -27., 4.), (-8., 17., -5., 17.), (5., 17., 8., 17.),
                     (-8., -17., -5., -17.), (5., -17., 8., -17.), (27., -3., 27., 0.), (27., 2., 27., 5.))
    markers: tuple = ((-25., 15., .3), (-25., -15., .3), (25., 15., .3), (25., -15., .3), (0., 19., .3),
                      (0., -19., .3), (-18., 19., .3), (-18., -19., .3))
    scenario: str = SCENARIO
    row_width_m: float = 100.
    headland_depth_m: float = 0.
    end_x_m: float = 1000.
    seed: int = 0

    def segments(self):
        return tuple(self.walls) + tuple(self.panels)

    def to_dict(self):
        return dict(kind=YARD_KIND, scenario=self.scenario, walls=[list(w) for w in self.walls],
                    panels=[list(p) for p in self.panels], markers=[list(m) for m in self.markers])


@dataclass(frozen=True)
class YardSpec:
    """World spec handed to IdentRig (kind 'W0' like a scene_spec W0 world; T1Rig reads kind and markers)."""

    config: YardConfig = YardConfig()
    kind: str = YARD_KIND

    @property
    def markers(self):
        return self.config.markers


class Yard:
    """Yard world (field frame = world frame)."""

    def __init__(self, config=None):
        self.config = YardConfig() if config is None else config
        self.posts = np.asarray(self.config.markers, dtype=float).reshape(-1, 3)
        self.segs = np.asarray(self.config.segments(), dtype=float).reshape(-1, 4)

    def to_field(self, x, y, yaw=0.):
        return x, y, yaw

    def pose_in_field(self, fx, fy, fyaw=0.):
        return fx, fy, fyaw

    def inside(self, x, y, margin_m):
        """True when (x, y) keeps margin_m to every wall and panel (the operator's 'stay clear' rule)."""
        return self.min_distance(x, y) >= margin_m

    def min_distance(self, x, y):
        best = math.inf
        for x1, y1, x2, y2 in self.segs:
            ex, ey = x2 - x1, y2 - y1
            t = max(0., min(1., ((x - x1) * ex + (y - y1) * ey) / max(ex * ex + ey * ey, 1e-12)))
            best = min(best, math.hypot(x - (x1 + t * ex), y - (y1 + t * ey)))
        for px, py, r in self.posts:
            best = min(best, math.hypot(x - px, y - py) - r)
        return best

    def describe(self):
        return dict(self.config.to_dict(), physical_truth=False)


class YardCaster:
    """cpu_lidar.ScanSynthesizer scanner over the yard: scan(xyz, rotation) -> (ranges, origin, directions)."""

    def __init__(self, world, rays=LIDAR_RAYS, maximum=LIDAR_MAX_RANGE_M):
        self.world = world
        self.rays = int(rays)
        self.maximum = float(maximum)
        self.angles = -math.pi + np.arange(self.rays) * (2. * math.pi / self.rays)
        self.occluded_sectors = ()

    def cast(self, sx, sy, yaw):
        out = np.full(self.rays, math.inf)
        phi = self.angles + yaw
        dx, dy = np.cos(phi), np.sin(phi)
        segs = self.world.segs
        if len(segs):
            x1, y1 = segs[:, 0] - sx, segs[:, 1] - sy
            ex, ey = segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1]
            den = dx[:, None] * ey[None, :] - dy[:, None] * ex[None, :]
            with np.errstate(divide='ignore', invalid='ignore'):
                t = (x1[None, :] * ey[None, :] - y1[None, :] * ex[None, :]) / den
                u = (x1[None, :] * dy[:, None] - y1[None, :] * dx[:, None]) / den
            ok = (np.abs(den) > 1e-12) & (t > 0.) & (u >= 0.) & (u <= 1.)
            out = np.minimum(out, np.where(ok, t, math.inf).min(axis=1))
        posts = self.world.posts
        if len(posts):
            cx, cy, r = posts[:, 0] - sx, posts[:, 1] - sy, posts[:, 2]
            if np.any(cx * cx + cy * cy <= r * r):
                return np.zeros(self.rays)
            b = dx[:, None] * cx[None, :] + dy[:, None] * cy[None, :]
            disc = b * b - (cx * cx + cy * cy - r * r)[None, :]
            hit = (disc >= 0.) & (b > 0.)
            t = np.where(hit, b - np.sqrt(np.where(hit, disc, 0.)), math.inf)
            out = np.minimum(out, t.min(axis=1))
        out[out > self.maximum] = math.inf
        return out

    def scan(self, xyz, rotation):
        yaw = math.atan2(rotation[1][0], rotation[0][0])
        ranges = self.cast(float(xyz[0]), float(xyz[1]), yaw)
        for lo, hi in self.occluded_sectors:
            ranges[(self.angles >= lo) & (self.angles <= hi)] = math.inf
        directions = np.column_stack((np.cos(self.angles + yaw), np.sin(self.angles + yaw), np.zeros(self.rays)))
        return ranges, np.array((xyz[0], xyz[1], xyz[2]), dtype=float), directions


def make_yard_lidar(world, seed, noise_sd=fw.LIDAR_NOISE_SD_M):
    """(YardCaster, cpu_lidar.ScanSynthesizer) with the W0 SIM LiDAR values (3200 SIM beams, 10 Hz, 30 m)."""
    from cpu_lidar import ScanSynthesizer
    caster = YardCaster(world)
    synth = ScanSynthesizer(caster, count=LIDAR_RAYS, hz=LIDAR_HZ, minimum=.05, maximum=LIDAR_MAX_RANGE_M,
                            noise_stddev=noise_sd, seed=seed)
    return caster, synth
