"""Bounded offline orchard fixtures, never a source of navigation knowledge.

Scene geometry/oracle metadata belong to the test harness. Only SensorScan is
passed to a perception candidate. These ideal instantaneous 2D rays are not
Gazebo integration or a calibrated model of the physical S2's failure modes.
"""

from dataclasses import dataclass
import math

import numpy as np


SCENARIOS = (
    "open_end", "wall_beyond_rows", "wall_across_before_end",
    "narrow_headland", "missing_trunk_gap", "occluded_next_row",
)
MAX_TREES = 200
MAX_WALLS = 20
MAX_RAYS = 3200
MAX_RANGE_M = 30.0


def _finite(*values):
    return all(isinstance(value, (int, float, np.integer, np.floating))
               and not isinstance(value, (bool, np.bool_)) and math.isfinite(value)
               for value in values)


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def __post_init__(self):
        if not _finite(self.x, self.y, self.yaw):
            raise ValueError("pose must contain finite numbers")


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    radius: float

    def __post_init__(self):
        if not _finite(self.x, self.y, self.radius) or self.radius <= 0:
            raise ValueError("circle must have finite center and positive radius")


@dataclass(frozen=True)
class Segment:
    start: tuple[float, float]
    end: tuple[float, float]

    def __post_init__(self):
        if (len(self.start) != 2 or len(self.end) != 2
                or not _finite(*self.start, *self.end) or tuple(self.start) == tuple(self.end)):
            raise ValueError("segment needs distinct finite 2D endpoints")
        object.__setattr__(self, "start", tuple(self.start))
        object.__setattr__(self, "end", tuple(self.end))


@dataclass(frozen=True)
class SceneConfig:
    seed: int = 17
    row_count: int = 5
    row_width_m: float = 3.5
    tree_spacing_m: float = 2.5
    row_length_m: float = 20.0
    field_yaw_rad: float = 0.0
    trunk_radius_m: float = 0.13
    radius_jitter_m: float = 0.015
    position_jitter_m: float = 0.08
    missing_probability: float = 0.05
    headland_depth_m: float = 3.0
    narrow_headland_depth_m: float = 0.6
    wall_before_end_m: float = 4.0
    gap_tree_count: int = 3
    occluder_length_m: float = 6.0
    wall_margin_m: float = 2.0

    def __post_init__(self):
        for name in ("seed", "row_count", "gap_tree_count"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool):
                raise ValueError(f"{name} must be an integer")
        if self.seed < 0 or not 2 <= self.row_count <= MAX_TREES or self.gap_tree_count < 1:
            raise ValueError("invalid seed, row count or gap size")
        for name, value in vars(self).items():
            if not _finite(value):
                raise ValueError(f"{name} must be finite")
        positive = ("row_width_m", "tree_spacing_m", "row_length_m", "trunk_radius_m",
                    "headland_depth_m", "narrow_headland_depth_m", "wall_before_end_m",
                    "occluder_length_m", "wall_margin_m")
        if any(getattr(self, name) <= 0 for name in positive):
            raise ValueError("geometric lengths must be positive")
        if self.row_length_m < 2 * self.tree_spacing_m:
            raise ValueError("each row requires at least three nominal tree positions")
        if self.tree_count_per_row * self.row_count > MAX_TREES:
            raise ValueError("fixture exceeds 200 nominal trees")
        if (self.radius_jitter_m < 0 or self.radius_jitter_m >= self.trunk_radius_m
                or self.position_jitter_m < 0 or not 0 <= self.missing_probability < 1):
            raise ValueError("invalid radius/position jitter or missing probability")
        clearance = 2 * (self.trunk_radius_m + self.radius_jitter_m + self.position_jitter_m)
        if min(self.row_width_m, self.actual_tree_spacing_m) <= clearance:
            raise ValueError("jitter bounds must not overlap adjacent nominal trunks")
        if self.wall_before_end_m >= self.row_length_m:
            raise ValueError("before-end wall must lie inside the row length")

    @property
    def tree_count_per_row(self):
        return max(3, int(round(self.row_length_m / self.tree_spacing_m)) + 1)

    @property
    def actual_tree_spacing_m(self):
        # Keep both row endpoints exact while approximating requested spacing.
        return self.row_length_m / (self.tree_count_per_row - 1)


@dataclass(frozen=True)
class SceneOracle:
    """Test-only field coordinates and labels; never attached to SensorScan."""

    scenario: str
    seed: int
    field_yaw_rad: float
    row_y_m: tuple[float, ...]
    row_start_x_m: float
    row_end_x_m: float
    actual_tree_spacing_m: float
    missing_slots: tuple[tuple[int, int], ...]
    wall_roles: tuple[str, ...]


@dataclass(frozen=True)
class OrchardScene:
    circles: tuple[Circle, ...] = ()
    segments: tuple[Segment, ...] = ()
    oracle: SceneOracle | None = None

    def __post_init__(self):
        if len(self.circles) > MAX_TREES or len(self.segments) > MAX_WALLS:
            raise ValueError("fixture geometry exceeds 200 circles or 20 segments")
        if (any(not isinstance(item, Circle) for item in self.circles)
                or any(not isinstance(item, Segment) for item in self.segments)):
            raise ValueError("invalid fixture geometry type")
        object.__setattr__(self, "circles", tuple(self.circles))
        object.__setattr__(self, "segments", tuple(self.segments))


@dataclass(frozen=True)
class ScanConfig:
    rays: int = 3200
    range_min: float = 0.05
    range_max: float = 30.0

    def __post_init__(self):
        if not isinstance(self.rays, int) or isinstance(self.rays, bool) or not 4 <= self.rays <= MAX_RAYS:
            raise ValueError("ray count must be an integer from 4 to 3200")
        if not _finite(self.range_min, self.range_max) or not 0 < self.range_min < self.range_max <= MAX_RANGE_M:
            raise ValueError("require 0 < range_min < range_max <= 30 m")


@dataclass(frozen=True)
class SensorScan:
    """Sensor-only packet: no pose, rows, count, seed, scenario or endpoint labels."""

    ranges: tuple[float, ...]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


def _rotate(point, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * point[0] - s * point[1], s * point[0] + c * point[1])


def field_pose(scene, x, y, yaw=0.0):
    """Test harness helper: convert a field-coordinate pose into world space."""
    if scene.oracle is None:
        raise ValueError("field_pose requires generator oracle metadata")
    if not _finite(x, y, yaw):
        raise ValueError("field pose must contain finite numbers")
    wx, wy = _rotate((x, y), scene.oracle.field_yaw_rad)
    return Pose2D(wx, wy, yaw + scene.oracle.field_yaw_rad)


def generate_scene(scenario="open_end", config=SceneConfig()):
    """Generate one seeded test world; oracle metadata stays in the harness.

    The first corridor is centered on field y=0, between rows at +/-width/2.
    Other rows extend in +y. Longitudinal row endpoints are x=0 and x=length;
    spacing is adjusted slightly to fit both endpoints (reported in oracle).
    Endpoint trunks are retained, and only interior trunks get random missing
    slots. The targeted gap scenario also retains the row's two endpoints.
    """
    if scenario not in SCENARIOS:
        raise ValueError(f"unknown scenario: {scenario}")
    if scenario == "occluded_next_row" and config.row_count < 3:
        raise ValueError("occluded_next_row requires at least three rows")
    count = config.tree_count_per_row
    if scenario == "missing_trunk_gap" and config.gap_tree_count > count - 2:
        raise ValueError("gap must fit inside a row while retaining both endpoints")
    rng = np.random.default_rng(config.seed)
    row_y = tuple((row - 0.5) * config.row_width_m for row in range(config.row_count))
    positions = np.linspace(0.0, config.row_length_m, count)
    gap_start = max(1, (count - config.gap_tree_count) // 2)
    forced_gap = set(range(gap_start, gap_start + config.gap_tree_count))
    circles, missing = [], []
    for row, y in enumerate(row_y):
        for index, x in enumerate(positions):
            # Draw every slot before deciding omission; missing-slot policy does
            # not shift randomness for all the remaining trees in the field.
            omit = rng.random() < config.missing_probability
            jitter = rng.uniform(-config.position_jitter_m, config.position_jitter_m, 2)
            radius = config.trunk_radius_m + rng.uniform(-config.radius_jitter_m, config.radius_jitter_m)
            interior = 0 < index < count - 1
            if interior and (omit or (scenario == "missing_trunk_gap" and row == 1 and index in forced_gap)):
                missing.append((row, index))
                continue
            # Keep longitudinal endpoint coordinates exact for oracle assertions.
            local_x = float(x + jitter[0]) if interior else float(x)
            wx, wy = _rotate((local_x, y + jitter[1]), config.field_yaw_rad)
            circles.append(Circle(wx, wy, float(radius)))
    local_walls, roles = [], []
    wall_x = None
    if scenario == "wall_beyond_rows":
        wall_x = config.row_length_m + config.headland_depth_m
    elif scenario == "wall_across_before_end":
        wall_x = config.row_length_m - config.wall_before_end_m
    elif scenario == "narrow_headland":
        wall_x = config.row_length_m + config.narrow_headland_depth_m
    if wall_x is not None:
        local_walls.append(((wall_x, row_y[0] - config.wall_margin_m),
                            (wall_x, row_y[-1] + config.wall_margin_m)))
        roles.append(scenario)
    if scenario == "occluded_next_row":
        # An opaque side barrier hides the next corridor from the current one.
        # It does not imply that the hidden corridor is safe, present or drivable.
        y = config.row_width_m * 0.75
        local_walls.append(((max(0.0, config.row_length_m - config.occluder_length_m), y),
                            (config.row_length_m + config.headland_depth_m, y)))
        roles.append("next_row_occluder")
    segments = tuple(Segment(_rotate(a, config.field_yaw_rad), _rotate(b, config.field_yaw_rad))
                     for a, b in local_walls)
    oracle = SceneOracle(scenario, config.seed, config.field_yaw_rad, row_y, 0.0,
                         config.row_length_m, config.actual_tree_spacing_m, tuple(missing), tuple(roles))
    return OrchardScene(tuple(circles), segments, oracle)


def raycast_scan(scene, pose=Pose2D(), *, config=ScanConfig()):
    """Nearest ideal 2D ray hit; inf means outside the observable range.

    Find the nearest nonnegative hit first, then apply range limits: an object
    inside the blind near range must not reveal a farther object behind it.
    No return noise, deskew, rolling timing or transparent surfaces are modeled.
    """
    angles = -math.pi + np.arange(config.rays) * (2 * math.pi / config.rays)
    directions = np.column_stack((np.cos(angles + pose.yaw), np.sin(angles + pose.yaw)))
    origin = np.array((pose.x, pose.y))
    ranges = np.full(config.rays, np.inf)
    for circle in scene.circles:
        offset = np.array((circle.x, circle.y)) - origin
        projection = directions @ offset
        discriminant = projection ** 2 - (offset @ offset - circle.radius ** 2)
        visible = discriminant >= 0
        root = np.sqrt(np.maximum(discriminant, 0))
        near, far = projection - root, projection + root
        distance = np.where(near >= 0, near, far)
        visible &= distance >= 0
        ranges[visible] = np.minimum(ranges[visible], distance[visible])
    for segment in scene.segments:
        edge = np.subtract(segment.end, segment.start)
        offset = np.subtract(segment.start, origin)
        denominator = directions[:, 0] * edge[1] - directions[:, 1] * edge[0]
        visible = np.abs(denominator) > 1e-12
        distance, parameter = np.full(config.rays, np.inf), np.full(config.rays, np.inf)
        distance[visible] = (offset[0] * edge[1] - offset[1] * edge[0]) / denominator[visible]
        parameter[visible] = (offset[0] * directions[visible, 1] - offset[1] * directions[visible, 0]) / denominator[visible]
        visible &= (distance >= 0) & (parameter >= 0) & (parameter <= 1)
        ranges[visible] = np.minimum(ranges[visible], distance[visible])
    ranges[(ranges < config.range_min) | (ranges > config.range_max)] = np.inf
    return SensorScan(tuple(float(value) for value in ranges), -math.pi, 2 * math.pi / config.rays,
                      config.range_min, config.range_max)
