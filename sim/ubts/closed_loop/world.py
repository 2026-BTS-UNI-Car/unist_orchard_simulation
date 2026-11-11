"""Scene, sensor synthesis and truth metrics for the headless closed loop.

Scenes come from orchard_perception.scenarios.generate_scene (trunks, walls,
occluder) plus SIM-ASSUMED canopy cross-sections ("branches") at the 0.60 m
scan height. raycast_scan accepts at most 200 circles per scene, so circles
are ray-cast in chunks and the nearest hit is kept. ScanSynthesizer adds the
10 Hz gating, minimum-range clamp and Gaussian range noise. Truth (pose,
geometry, oracle) is used here only to make sensor packets and score runs.
"""
from dataclasses import dataclass, field
import math

import numpy as np

from cpu_lidar import ScanSynthesizer
from orchard_perception.scenarios import (Circle, OrchardScene, Pose2D, SceneConfig, ScanConfig, Segment,
                                          field_pose, generate_scene, raycast_scan)


LIDAR_RAYS = 3200          # raycast_scan cap; the real S2 driver emits 3240 beams
LIDAR_NOISE_SD_M = .015    # SIM_ASSUMED; the +-30 mm spec is not a 1-sigma value


@dataclass(frozen=True)
class WorldConfig:
    scenario: str = 'open_end'
    seed: int = 0
    row_count: int = 3
    row_width_m: float = 3.5
    tree_spacing_m: float = 2.5
    row_length_m: float = 20.
    field_yaw_deg: float = 0.
    missing_probability: float = .05
    gap_tree_count: int = 3
    headland_depth_m: float = 3.
    narrow_headland_depth_m: float = .6
    wall_before_end_m: float = 4.
    occluder_length_m: float = 6.
    branches: bool = True
    branches_per_tree: tuple = (2, 5)
    branch_reach_m: float = .45        # SIM_ASSUMED canopy reach at 0.60 m
    branch_along_m: float = .9
    branch_radius_m: tuple = (.04, .10)
    trunk_radius_m: float = .13
    stagger: bool = False              # odd rows shifted by half a tree spacing
    gaps: tuple = ()                   # ((row_index, x_from_m, x_to_m), ...) trunks removed
    # Opt-in scene extras (EXTRA_SCENARIOS only; empty = the identical scene and draws):
    extra_posts: tuple = ()            # ((field_x, field_y, radius), ...) hard posts; field y 0 = corridor 0 centre
    branch_reach_scale_by_row: tuple = ()   # ((row_index, scale), ...) applied AFTER the canopy draws
    # Opt-in headland clutter (matrix.HEADLAND_SCENARIOS only; empty = the identical scene and draws). Neither
    # enters the scene truth (row trunks, scene walls, row end, cross walls): both are obstacles beyond it.
    extra_walls: tuple = ()            # ((fx0, fy0, fx1, fy1), ...) hard wall segments (fence, boundary wall, bin side)
    extra_trees: tuple = ()            # ((field_x, field_y), ...) trees of ANOTHER block: trunk (hard) + canopy,
                                       # own random stream (position jitter, canopy) so the scene draws are unchanged

    def scene_config(self):
        return SceneConfig(seed=self.seed, row_count=self.row_count, row_width_m=self.row_width_m,
                           tree_spacing_m=self.tree_spacing_m, row_length_m=self.row_length_m,
                           field_yaw_rad=math.radians(self.field_yaw_deg),
                           missing_probability=self.missing_probability,
                           gap_tree_count=self.gap_tree_count, headland_depth_m=self.headland_depth_m,
                           narrow_headland_depth_m=self.narrow_headland_depth_m,
                           wall_before_end_m=self.wall_before_end_m,
                           occluder_length_m=self.occluder_length_m, trunk_radius_m=self.trunk_radius_m)


@dataclass
class World:
    config: WorldConfig
    scene: OrchardScene
    trunks: tuple
    branches: tuple
    walls: tuple
    chunks: tuple
    oracle: object
    _circle_array: np.ndarray = field(default=None, repr=False)
    extras: tuple = ()                 # hard extra posts (ray-cast after the branches; not row trunks)
    extra_walls: tuple = ()            # hard extra wall segments (ray-cast; not scene walls, not in corridor_truth)

    def to_field(self, x, y, yaw=0.):
        """World -> field frame (row axis +x, first corridor centred on y=0)."""
        a = -self.oracle.field_yaw_rad
        c, s = math.cos(a), math.sin(a)
        return c * x - s * y, s * x + c * y, yaw + a

    def pose_in_field(self, fx, fy, fyaw=0.):
        p = field_pose(self.scene, fx, fy, fyaw)
        return p.x, p.y, p.yaw

    def line_end_x(self, row):
        """Field x of the last trunk on one row line (truth, evaluation only)."""
        y = self.oracle.row_y_m[row]
        xs = [fx for fx, fy, _ in (self.to_field(t.x, t.y) for t in self.trunks) if abs(fy - y) < .4]
        return float(max(xs)) if xs else None

    def corridor_truth(self, k):
        """Truth for corridor k (between rows k and k+1): end x, walls across it, next corridors."""
        rows, width, length = self.oracle.row_y_m, self.config.row_width_m, self.config.row_length_m
        ends = [self.line_end_x(r) for r in (k, k + 1)]
        segments = []
        for wall in self.walls:
            a, b = self.to_field(*wall.start)[:2], self.to_field(*wall.end)[:2]
            segments.append((a, b))
        centre = k * width
        cross_walls = sorted(float(a[0]) for a, b in segments
                             if abs(a[0] - b[0]) < .2 and min(a[1], b[1]) <= centre <= max(a[1], b[1]))

        def hidden(y_lo, y_hi):
            for a, b in segments:
                if abs(a[1] - b[1]) < .2 and y_lo < a[1] < y_hi:
                    covered = min(max(a[0], b[0]), length) - max(min(a[0], b[0]), 0.)
                    if covered >= .8 * length:
                        return True
            return False

        def expected(row_next, line_y, side):
            next_y = line_y + side * width
            if hidden(min(line_y, next_y), max(line_y, next_y)):
                return 'UNKNOWN'
            return 'NOT_LAST' if 0 <= row_next < len(rows) else 'LAST'
        return dict(end_x=max(e for e in ends if e is not None), line_end_x=ends,
                    cross_walls_x=tuple(cross_walls), centre_y=centre,
                    next_left=expected(k + 2, rows[k + 1], 1), next_right=expected(k - 1, rows[k], -1))


def _modify_trunks(scene, config):
    """Stagger odd rows and remove explicit gaps (field frame), keeping walls/oracle."""
    if not config.stagger and not config.gaps:
        return scene
    oracle = scene.oracle
    yaw = oracle.field_yaw_rad
    c, s = math.cos(yaw), math.sin(yaw)
    spacing = oracle.actual_tree_spacing_m
    circles = []
    for circle in scene.circles:
        fx, fy = c * circle.x + s * circle.y, -s * circle.x + c * circle.y
        row = int(np.argmin([abs(fy - y) for y in oracle.row_y_m]))
        if config.stagger and row % 2 == 1:
            fx += spacing / 2
            if fx > config.row_length_m + 1e-6:
                continue
        if any(row == r and x0 <= fx <= x1 for r, x0, x1 in config.gaps):
            continue
        circles.append(Circle(c * fx - s * fy, s * fx + c * fy, circle.radius))
    return OrchardScene(tuple(circles), scene.segments, oracle)


def build_world(config):
    scene = _modify_trunks(generate_scene(config.scenario, config.scene_config()), config)
    rng = np.random.default_rng(10_000 + config.seed)
    yaw = math.radians(config.field_yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    branches = []
    reach_scale = dict(config.branch_reach_scale_by_row)
    if config.branches:
        lo, hi = config.branches_per_tree
        for trunk in scene.circles:
            scale = 1.
            if reach_scale:     # row index as in _modify_trunks (field y nearest row line)
                fy = -s * trunk.x + c * trunk.y
                scale = reach_scale.get(int(np.argmin([abs(fy - y) for y in scene.oracle.row_y_m])), 1.)
            for _ in range(int(rng.integers(lo, hi + 1))):
                along = rng.uniform(-config.branch_along_m, config.branch_along_m)
                across = rng.uniform(-config.branch_reach_m, config.branch_reach_m)
                radius = rng.uniform(*config.branch_radius_m)
                if scale != 1.:
                    across *= scale         # after every draw: the random stream is unchanged
                branches.append(Circle(trunk.x + c * along - s * across,
                                       trunk.y + s * along + c * across, float(radius)))
    extras = []
    for fx, fy, radius in config.extra_posts:
        pose = field_pose(scene, float(fx), float(fy))
        extras.append(Circle(pose.x, pose.y, float(radius)))
    if config.extra_trees:
        # Another block's trees: own stream (the scene and canopy draws above are untouched), the scene's
        # trunk position jitter (+-0.08 m) and canopy model. Trunks are hard extras, NOT row trunks (truth).
        tree_rng = np.random.default_rng((config.seed, 1))
        lo, hi = config.branches_per_tree
        for fx, fy in config.extra_trees:
            fx, fy = float(fx) + tree_rng.uniform(-.08, .08), float(fy) + tree_rng.uniform(-.08, .08)
            trunk = field_pose(scene, fx, fy)
            extras.append(Circle(trunk.x, trunk.y, float(config.trunk_radius_m)))
            if not config.branches:
                continue
            for _ in range(int(tree_rng.integers(lo, hi + 1))):
                along = tree_rng.uniform(-config.branch_along_m, config.branch_along_m)
                across = tree_rng.uniform(-config.branch_reach_m, config.branch_reach_m)
                radius = tree_rng.uniform(*config.branch_radius_m)
                branches.append(Circle(trunk.x + c * along - s * across,
                                       trunk.y + s * along + c * across, float(radius)))
    extra_walls = []
    for fx0, fy0, fx1, fy1 in config.extra_walls:
        a, b = field_pose(scene, float(fx0), float(fy0)), field_pose(scene, float(fx1), float(fy1))
        extra_walls.append(Segment((a.x, a.y), (b.x, b.y)))
    circles = list(scene.circles) + branches + extras
    chunks = []
    for start in range(0, len(circles), 200):
        part = tuple(circles[start:start + 200])
        chunks.append(OrchardScene(part, scene.segments if start == 0 else ()))
    if not chunks:
        chunks.append(OrchardScene((), scene.segments))
    for start in range(0, len(extra_walls), 20):     # OrchardScene holds at most 20 segments
        chunks.append(OrchardScene((), tuple(extra_walls[start:start + 20])))
    world = World(config, scene, tuple(scene.circles), tuple(branches), tuple(scene.segments),
                  tuple(chunks), scene.oracle, extras=tuple(extras), extra_walls=tuple(extra_walls))
    return world


class ChunkedRaycaster:
    """``scan(xyz, rotation)`` contract of cpu_lidar.ScanSynthesizer."""

    def __init__(self, world, rays=LIDAR_RAYS):
        self.world = world
        self.config = ScanConfig(rays=rays)
        self.occluded_sectors = ()   # (lo_rad, hi_rad) in the laser frame -> no return
        self.angles = -math.pi + np.arange(rays) * (2 * math.pi / rays)

    def scan(self, xyz, rotation):
        yaw = math.atan2(rotation[1][0], rotation[0][0])
        pose = Pose2D(float(xyz[0]), float(xyz[1]), yaw)
        ranges = None
        for chunk in self.world.chunks:
            part = np.asarray(raycast_scan(chunk, pose, config=self.config).ranges)
            ranges = part if ranges is None else np.minimum(ranges, part)
        for lo, hi in self.occluded_sectors:
            ranges[(self.angles >= lo) & (self.angles <= hi)] = np.inf
        directions = np.column_stack((np.cos(self.angles + yaw), np.sin(self.angles + yaw),
                                      np.zeros_like(self.angles)))
        return ranges, np.array((xyz[0], xyz[1], xyz[2])), directions


def make_lidar(world, seed, noise_sd=LIDAR_NOISE_SD_M):
    caster = ChunkedRaycaster(world)
    synth = ScanSynthesizer(caster, count=LIDAR_RAYS, hz=10., minimum=.05, maximum=30.,
                            noise_stddev=noise_sd, seed=seed)
    return caster, synth


def yaw_quaternion(yaw):
    return (0., 0., math.sin(yaw / 2), math.cos(yaw / 2))


# --- truth metrics (evaluation only) ---------------------------------------

def footprint_clearance(world, x, y, yaw, length, width):
    """Signed clearance from the footprint rectangle to trunks/walls/branches."""
    c, s = math.cos(yaw), math.sin(yaw)

    def rect_distance(px, py):
        dx, dy = px - x, py - y
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        ox, oy = abs(lx) - length / 2, abs(ly) - width / 2
        outside = math.hypot(max(ox, 0.), max(oy, 0.))
        return outside if outside > 0 else max(ox, oy)

    def circles(items):
        best = math.inf
        for item in items:
            if abs(item.x - x) > 3 or abs(item.y - y) > 3:
                continue
            best = min(best, rect_distance(item.x, item.y) - item.radius)
        return best

    hard = circles(world.trunks)
    for wall in world.walls + world.extra_walls:
        hard = min(hard, _segment_rectangle_distance(wall, x, y, c, s, length / 2, width / 2))
    if world.extras:
        hard = min(hard, circles(world.extras))
    return hard, circles(world.branches)


def _segment_rectangle_distance(wall, x, y, c, s, hx, hy):
    """Exact distance between a wall segment and the footprint (<0: overlap)."""
    def local(point):
        dx, dy = point[0] - x, point[1] - y
        return c * dx + s * dy, -s * dx + c * dy
    (ax, ay), (bx, by) = local(wall.start), local(wall.end)
    # Liang-Barsky clip: any part of the segment inside the rectangle?
    t0, t1, dx, dy = 0., 1., bx - ax, by - ay
    inside = True
    for p, q in ((-dx, ax + hx), (dx, hx - ax), (-dy, ay + hy), (dy, hy - ay)):
        if p == 0:
            if q < 0:
                inside = False
                break
        else:
            r = q / p
            if p < 0:
                t0 = max(t0, r)
            else:
                t1 = min(t1, r)
            if t0 > t1:
                inside = False
                break
    if inside:
        return -1e-3
    best = math.inf
    for px, py in ((ax, ay), (bx, by)):
        best = min(best, math.hypot(max(abs(px) - hx, 0.), max(abs(py) - hy, 0.)))
    norm2 = dx * dx + dy * dy
    for cx in (-hx, hx):
        for cy in (-hy, hy):
            t = max(0., min(1., ((cx - ax) * dx + (cy - ay) * dy) / norm2))
            best = min(best, math.hypot(cx - ax - t * dx, cy - ay - t * dy))
    return best
