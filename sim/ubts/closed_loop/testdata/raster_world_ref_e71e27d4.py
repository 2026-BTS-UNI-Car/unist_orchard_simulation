"""SLAM-raster SIM world, never a navigation pose or physical ground truth.

Only Root may load the real map. Pure tests use small generated rasters.
The default preserves v2 opaque unknown visibility. The explicitly selected
v3 development model may cross unknown cells to mapped occupied surfaces;
unknown remains uncertainty, never a reflecting wall or certified free space.
A separate evaluator still aborts footprint traversal of unknown space.
"""
from dataclasses import dataclass, field
import hashlib
import math
from pathlib import Path
import re
import stat

import numpy as np

FREE, OCCUPIED, UNKNOWN, OUTSIDE = 0, 1, 2, 3
MAX_PIXELS = 8_388_608
MAX_PGM_BYTES = MAX_PIXELS + 4096
MAX_RAYS = 3240

def require(ok, reason):
    if not ok:
        raise ValueError(reason)

@dataclass(frozen=True)
class MapSpec:
    resolution: float
    occupied_thresh: float = .65
    free_thresh: float = .196
    negate: int = 0
    # Retained only in local memory. SIM coordinates use raster-local metres.
    origin: tuple = field(default=(0., 0., 0.), repr=False)

    def validate(self):
        require(type(self.negate) is int and self.negate in (0, 1), 'trinary_negate')
        require(type(self.origin) is tuple and len(self.origin) == 3, 'private_origin_shape')
        require(all(type(v) in (int, float) and math.isfinite(v) for v in
                    (self.resolution, self.occupied_thresh, self.free_thresh, *self.origin)), 'finite_map_spec')
        require(.02 <= self.resolution <= 1 and 0 <= self.free_thresh < self.occupied_thresh <= 1
                , 'bounded_map_spec')
        return self

def classify_pixels(top_down_pixels, spec):
    """ROS trinary strict thresholds, with PGM top row -> metric highest y."""
    spec.validate()
    pixels = np.asarray(top_down_pixels)
    require(pixels.ndim == 2 and pixels.dtype == np.uint8 and 0 < pixels.size <= MAX_PIXELS, 'uint8_bounded_PGM')
    probability = pixels.astype(np.float64) / 255.
    if not spec.negate:
        probability = 1. - probability
    values = np.full(pixels.shape, UNKNOWN, dtype=np.uint8)
    values[probability > spec.occupied_thresh] = OCCUPIED
    values[probability < spec.free_thresh] = FREE
    # A private local copy prevents caller mutation; no probabilistic/unknown
    # cell is flattened to free, no smoothing or resampling is applied.
    values = values[::-1].copy()
    values.flags.writeable = False
    return values

def _pgm_header(raw):
    require(type(raw) is bytes and 0 < len(raw) <= MAX_PGM_BYTES, 'bounded_PGM_bytes')
    position = 0
    tokens = []
    while len(tokens) < 4:
        require(position < min(len(raw), 4096), 'bounded_PGM_header')
        while position < len(raw) and raw[position] in b' \t\r\n':
            position += 1
        if position < len(raw) and raw[position] == 35:
            end = raw.find(b'\n', position, 4096)
            require(end >= 0, 'terminated_PGM_comment')
            position = end + 1
            continue
        end = position
        while end < len(raw) and raw[end] not in b' \t\r\n#':
            end += 1
        require(end > position and end <= 4096, 'bounded_PGM_token')
        tokens.append(raw[position:end])
        position = end
    require(tokens[0] == b'P5' and all(re.fullmatch(b'[0-9]{1,8}', t) for t in tokens[1:]), 'PGM_P5_uint8_only')
    width, height, maximum = map(int, tokens[1:])
    require(width > 0 and height > 0 and width * height <= MAX_PIXELS and maximum == 255, 'PGM_size_maxval')
    require(position < len(raw) and raw[position] in b' \t\r\n', 'PGM_binary_separator')
    # One separator only: arbitrary whitespace may be a legitimate pixel.
    position += 2 if raw[position:position+2] == b'\r\n' else 1
    return width, height, position

def parse_pgm(raw):
    width, height, offset = _pgm_header(raw)
    require(len(raw) - offset == width * height, 'exact_PGM_payload_no_trailing_or_truncation')
    return np.frombuffer(raw, dtype=np.uint8, offset=offset).reshape(height, width)

def load_pgm_root_only(path, expected_sha256, spec):
    """Explicit Root loader; full SHA and descriptor/named lifetime before use."""
    require(re.fullmatch('[0-9a-f]{64}', expected_sha256 or ''), 'Root_pinned_full_PGM_SHA_required')
    path = Path(path)
    named = path.lstat()
    require(stat.S_ISREG(named.st_mode) and not path.is_symlink() and 0 < named.st_size <= MAX_PGM_BYTES,
            'bounded_regular_PGM')
    with path.open('rb') as stream:
        import os
        before = os.fstat(stream.fileno())
        raw = stream.read(MAX_PGM_BYTES + 1)
        after = os.fstat(stream.fileno())
    current = path.lstat()
    common = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns)
    require(common(named) == common(before) == common(after) == common(current)
            and named.st_ctime_ns == current.st_ctime_ns and before.st_ctime_ns == after.st_ctime_ns,
            'same_PGM_named_FD_lifetime')
    require(len(raw) == named.st_size and hashlib.sha256(raw).hexdigest() == expected_sha256, 'exact_full_PGM_SHA')
    return RasterGrid(classify_pixels(parse_pgm(raw), spec), spec.resolution,
                      source_sha256=expected_sha256)

def spec_from_yaml_root_only(path, expected_sha256):
    """Small local YAML only; origin is never emitted into a public receipt."""
    import yaml
    path = Path(path)
    require(re.fullmatch('[0-9a-f]{64}', expected_sha256 or ''), 'Root_pinned_YAML_SHA_required')
    info = path.lstat()
    require(stat.S_ISREG(info.st_mode) and not path.is_symlink() and 0 < info.st_size <= 4096, 'small_regular_map_yaml')
    raw = path.read_bytes()
    require(hashlib.sha256(raw).hexdigest() == expected_sha256 and path.lstat().st_mtime_ns == info.st_mtime_ns,
            'same_local_YAML_SHA_lifetime')
    doc = yaml.safe_load(raw)
    require(type(doc) is dict and doc.get('mode', 'trinary') == 'trinary', 'trinary_map_yaml')
    image = doc.get('image')
    require(type(image) is str and Path(image).name == image and '/' not in image and '\\' not in image
            and image.endswith('.pgm'), 'same_directory_PGM_only')
    origin = doc.get('origin')
    require(type(origin) is list and len(origin) == 3, 'private_yaml_origin')
    spec = MapSpec(doc['resolution'], doc['occupied_thresh'], doc['free_thresh'], doc['negate'], tuple(origin)).validate()
    return spec, path.parent / image

class RasterGrid:
    def __init__(self, cells, resolution, *, source_sha256=None):
        require(type(resolution) in (int, float) and math.isfinite(resolution) and .02 <= resolution <= 1, 'bounded_resolution')
        values = np.asarray(cells)
        require(values.ndim == 2 and values.dtype == np.uint8 and 0 < values.size <= MAX_PIXELS
                and np.all(values <= UNKNOWN), 'bounded_trinary_cells')
        self.cells = values.copy()
        self.cells.flags.writeable = False
        self.resolution = float(resolution)
        self.height, self.width = values.shape
        self.source_sha256 = source_sha256

    def _states(self, xs, ys):
        xs, ys = np.asarray(xs), np.asarray(ys)
        result = np.full(xs.shape, OUTSIDE, dtype=np.uint8)
        inside = (xs >= 0) & (xs < self.width) & (ys >= 0) & (ys < self.height)
        result[inside] = self.cells[ys[inside], xs[inside]]
        return result

    def raycast(self, x, y, angles, maximum=30.):
        """Vectorized supercover DDA; work <= (2*ceil(R/res)+4)*3240.

        Ranges terminate at occupied OR unknown/outside boundary. Hit kinds
        remain evaluator metadata and are not passed to the controller.
        """
        angles = np.asarray(angles, dtype=float)
        require(angles.ndim == 1 and 1 <= angles.size <= MAX_RAYS and np.all(np.isfinite(angles))
                and all(math.isfinite(v) for v in (x, y, maximum)) and 0 < maximum <= 30., 'bounded_raycast')
        count, res = angles.size, self.resolution
        dx, dy = np.cos(angles), np.sin(angles)
        dx[np.abs(dx) < 1e-14] = 0.
        dy[np.abs(dy) < 1e-14] = 0.
        cx = np.full(count, math.floor(x/res), dtype=np.int64)
        cy = np.full(count, math.floor(y/res), dtype=np.int64)
        sx, sy = np.sign(dx).astype(np.int64), np.sign(dy).astype(np.int64)
        invx = np.divide(res, np.abs(dx), out=np.full(count, np.inf), where=dx != 0)
        invy = np.divide(res, np.abs(dy), out=np.full(count, np.inf), where=dy != 0)
        edge_x = np.where(sx > 0, (cx+1)*res, cx*res)
        edge_y = np.where(sy > 0, (cy+1)*res, cy*res)
        tx = np.divide(edge_x-x, dx, out=np.full(count, np.inf), where=dx != 0)
        ty = np.divide(edge_y-y, dy, out=np.full(count, np.inf), where=dy != 0)
        ranges = np.full(count, np.inf)
        kinds = self._states(cx, cy)
        active = kinds == FREE
        ranges[~active] = 0.
        for _ in range(2 * math.ceil(maximum/res) + 4):
            ids = np.flatnonzero(active)
            if not ids.size:
                break
            near = np.minimum(tx[ids], ty[ids])
            finished = near > maximum
            active[ids[finished]] = False
            ids, near = ids[~finished], near[~finished]
            if not ids.size:
                continue
            cross_x = tx[ids] <= ty[ids] + 1e-12
            cross_y = ty[ids] <= tx[ids] + 1e-12
            nx, ny = cx[ids] + cross_x*sx[ids], cy[ids] + cross_y*sy[ids]
            state = self._states(nx, ny)
            # At exact corners include both adjacent cells: no diagonal leak
            # through touching walls or unknown cells.
            tie = cross_x & cross_y
            if np.any(tie):
                ii = ids[tie]
                side_x = self._states(cx[ii]+sx[ii], cy[ii])
                side_y = self._states(cx[ii], cy[ii]+sy[ii])
                state[tie] = np.maximum(state[tie], np.maximum(side_x, side_y))
            hit = state != FREE
            ranges[ids[hit]] = np.maximum(0., near[hit])
            kinds[ids[hit]] = state[hit]
            active[ids[hit]] = False
            cx[ids], cy[ids] = nx, ny
            tx[ids] += np.where(cross_x, invx[ids], 0.)
            ty[ids] += np.where(cross_y, invy[ids], 0.)
        require(not np.any(active), 'bounded_DDA_exhaustion_not_silent_free')
        return ranges, kinds

    def footprint_clearance(self, x, y, yaw, length, width, reach=2.):
        require(all(math.isfinite(v) for v in (x, y, yaw, length, width, reach))
                and 0 < length <= 3 and 0 < width <= 3 and 0 < reach <= 3, 'bounded_footprint')
        res = self.resolution
        radius = math.hypot(length/2, width/2) + reach + res
        lo_x, hi_x = math.floor((x-radius)/res), math.floor((x+radius)/res)
        lo_y, hi_y = math.floor((y-radius)/res), math.floor((y+radius)/res)
        require((hi_x-lo_x+1)*(hi_y-lo_y+1) <= 262144, 'footprint_ROI_cell_cap')
        xx, yy = np.meshgrid(np.arange(lo_x, hi_x+1), np.arange(lo_y, hi_y+1))
        state = self._states(xx.ravel(), yy.ravel())
        px, py = (xx.ravel()+.5)*res-x, (yy.ravel()+.5)*res-y
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy = np.abs(c*px+s*py)-length/2, np.abs(-s*px+c*py)-width/2
        dist = np.hypot(np.maximum(ox, 0.), np.maximum(oy, 0.)) + np.minimum(np.maximum(ox, oy), 0.)
        dist -= res/math.sqrt(2)  # Conservative full-cell enclosure, not point obstacles.
        def minimum(mask):
            return float(dist[mask].min()) if np.any(mask) else math.inf
        occupied = minimum(state == OCCUPIED)
        uncertain = minimum((state == UNKNOWN) | (state == OUTSIDE))
        return occupied, uncertain

@dataclass(frozen=True)
class EvaluationRoute:
    """Private raster-local evaluation annotation; never passed to estimator."""
    length_m: float
    anchor_x_m: float = field(repr=False)
    anchor_y_m: float = field(repr=False)
    heading_rad: float = field(repr=False)
    next_left: str = 'UNKNOWN'
    next_right: str = 'UNKNOWN'
    cross_walls_x: tuple = ()
    end_annotation_qualified: bool = False

    def validate(self):
        require(all(math.isfinite(v) for v in (self.length_m, self.anchor_x_m, self.anchor_y_m, self.heading_rad))
                and 0 < self.length_m <= 500, 'explicit_bounded_private_route_required')
        require(self.next_left in ('LAST', 'NOT_LAST', 'UNKNOWN') and self.next_right in ('LAST', 'NOT_LAST', 'UNKNOWN')
                and len(self.cross_walls_x) <= 32 and all(math.isfinite(v) for v in self.cross_walls_x), 'bounded_route_annotations')
        require(type(self.end_annotation_qualified) is bool, 'explicit_end_annotation_qualification')
        return self

class RasterWorld:
    def __init__(self, grid, route, config, *, visibility_model='UNKNOWN_OPAQUE_V2'):
        from visibility_model import MODEL
        require(visibility_model in ('UNKNOWN_OPAQUE_V2', MODEL), 'explicit_known_visibility_model')
        self.grid, self.route, self.config = grid, route.validate(), config
        self.visibility_model = visibility_model
        self.visibility_sample_calls = 0
        self.visibility_counts = {}
        self.unknown_contact_samples = 0
        self.occupied_contact_samples = 0

    def pose_in_field(self, x, y, yaw=0.):
        c, s = math.cos(self.route.heading_rad), math.sin(self.route.heading_rad)
        return (self.route.anchor_x_m+c*x-s*y, self.route.anchor_y_m+s*x+c*y, yaw+self.route.heading_rad)

    def to_field(self, x, y, yaw=0.):
        x, y = x-self.route.anchor_x_m, y-self.route.anchor_y_m
        c, s = math.cos(self.route.heading_rad), math.sin(self.route.heading_rad)
        return c*x+s*y, -s*x+c*y, yaw-self.route.heading_rad

    def corridor_truth(self, index):
        require(index == 0, 'one_explicit_evaluation_corridor_only')
        return dict(end_x=self.route.length_m, line_end_x=(None, None), centre_y=0.,
                    cross_walls_x=self.route.cross_walls_x, next_left=self.route.next_left, next_right=self.route.next_right)

    def clearance(self, x, y, yaw, length, width):
        occupied, uncertain = self.grid.footprint_clearance(x, y, yaw, length, width)
        if uncertain <= 0:
            self.unknown_contact_samples += 1
        if occupied <= 0:
            self.occupied_contact_samples += 1
        # No branch/trunk semantic label exists in a 2D occupancy raster.
        # The min is an evaluator abort boundary only. The v2 recorded judge
        # separates unknown-abort samples from occupied raster intersections.
        return min(occupied, uncertain), math.inf

class RasterRaycaster:
    def __init__(self, world, count=3200):
        require(type(count) is int and 2 <= count <= MAX_RAYS, 'bounded_sensor_ray_count')
        self.world = world
        self.angles = -math.pi + np.arange(count)*(2*math.pi/count)
        self.occluded_sectors = ()
        self.last_hit_counts = {}

    def scan(self, xyz, rotation):
        xyz, rotation = np.asarray(xyz, dtype=float), np.asarray(rotation, dtype=float)
        require(xyz.shape == (3,) and rotation.shape == (3, 3) and np.all(np.isfinite(xyz))
                and np.all(np.isfinite(rotation)), 'finite_sensor_pose')
        # This 2D map has no height semantics; only planar yaw is qualified.
        require(abs(rotation[2, 0]) < 1e-6 and abs(rotation[2, 1]) < 1e-6, 'planar_raster_sensor_only')
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        from visibility_model import MODEL
        from visibility_acceleration import cast
        if self.world.visibility_model == MODEL:
            visibility = cast(self.world.grid, float(xyz[0]), float(xyz[1]), self.angles+yaw)
            ranges, kinds = visibility.ranges, visibility.kinds
            for key, value in visibility.counts().items():
                self.world.visibility_counts[key] = self.world.visibility_counts.get(key, 0)+value
            self.world.visibility_sample_calls += 1
        else:
            ranges, kinds = self.world.grid.raycast(float(xyz[0]), float(xyz[1]), self.angles+yaw)
        # Only mapped occupied cells can emit returns. The explicit v3
        # hypothesis counts any preceding unknown traversal separately;
        # neither unknown nor outside nor a range limit invents a wall.
        ranges[kinds != OCCUPIED] = math.inf
        for lo, hi in self.occluded_sectors:
            ranges[(self.angles >= lo) & (self.angles <= hi)] = math.inf
        self.last_hit_counts = {name: int(np.count_nonzero(kinds == code)) for name, code in
                                (('occupied', OCCUPIED), ('unknown', UNKNOWN), ('outside', OUTSIDE))}
        directions = np.column_stack((np.cos(self.angles+yaw), np.sin(self.angles+yaw), np.zeros_like(self.angles)))
        return ranges, xyz.copy(), directions

def make_lidar(world, seed, noise_sd=.015):
    # Imported only during explicit Root-run integration, never in map loading.
    from cpu_lidar import ScanSynthesizer
    caster = RasterRaycaster(world)
    synth = ScanSynthesizer(caster, count=3200, hz=10., minimum=.05, maximum=30., noise_stddev=noise_sd, seed=seed)
    return caster, synth
