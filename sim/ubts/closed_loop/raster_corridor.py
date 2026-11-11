"""10/10 reconstructed full-row corridors: a raster world for closed_loop.harness (SIM only).

Source maps: the Cartographer OFFLINE whole-record maps of the 10/10 manual drives
(maps-1010-20261022/offline/maps/<id>/<id>_offline.{pgm,yaml}, 0.05 m cells). They are SLAM
reconstructions, not physical truth. This module uses them ONLY to synthesise LiDAR scans and to score
runs: the estimator and the controller never receive the map, the corridor labels or the pose (the
harness hands them sensor packets only). No GNSS position is read or written here.

Cell classes (the map YAML trinary thresholds, applied as ROS map_server does for negate 0:
p = (255 - v) / 255):
  OCCUPIED    p >  occupied_thresh (0.65 in every 10/10 YAML): an obstacle. Reflects a LiDAR ray; a
              footprint overlap is a collision (harness 'hard' clearance <= 0 ends the episode).
  FREE        p <  free_thresh (0.196): transparent, verified free.
  UNOBSERVED  v == 128 (Cartographer writes 128 where no submap has data) = outside the mapped area: a
              ray entering such a cell ends WITHOUT a return; so does a ray leaving the raster.
  UNCERTAIN   every other value (observed, between the thresholds): transparent for rays (no return is
              invented for it), NOT verified free. Split for the contact accounting into UNCERTAIN
              (free-leaning, p < 0.5) and UNCERTAIN_OCC (occupied-leaning, 0.5 <= p <= occupied_thresh).
  ADDED_HARD / ADDED_BRANCH: written by the scenario modifiers into a COPY of the grid (obstacle box,
              wall / branch protrusions). Both reflect rays. Gap / RemoveTrunks set the OCCUPIED cells of
              whole catalogue support units of one row to FREE.
Contact accounting: OCCUPIED/ADDED_HARD -> harness 'hard' clearance (collision); ADDED_BRANCH -> harness
'branch' clearance (soft contact, counted); UNCERTAIN, UNCERTAIN_OCC and UNOBSERVED (and outside the raster)
are never verified free: they are counted separately as unknown contact (world counters, evaluate_episode)
and do not end the episode. Clearance = footprint-rectangle distance to the cell centre - res/sqrt(2)
(conservative full-cell enclosure).

Frames: raster-local metres (x right, y up, origin at the raster's lower-left corner; the YAML origin
only places the row-analysis lines and the SLAM trajectory in the catalogue and is never written out).
The curvilinear corridor frame comes from the labelled centreline polyline (catalogue, 0.25 m step):
to_field(x, y, yaw) = (s, d, wrap(yaw - tangent)), s = arclength in the travel direction from the corridor
start (both rows begin), d = signed distance to the polyline (left of travel positive), tangent = direction
of the closest polyline segment (the end segments are extrapolated); pose_in_field is the inverse.
corridor_truth(0)['end_x'] = s of the last trunk (row-unit centre) on BOTH rows. These are the frame and
label definitions of single-row-20261102/PREREG.json (corridor_frame_and_labels, world.modifications,
placement_rules). The geometry of a catalogued corridor (raster_catalogue.py) is embedded in its
RasterCorridorConfig; the map is pinned by SHA-256.

Harness hooks: run_episode(config, world_factory=build_raster_world, lidar_factory=make_raster_lidar,
clearance_evaluator=raster_clearance), or RasterHooks / run_raster_episode below.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np

from . import external_paths

FREE, UNCERTAIN, UNOBSERVED, OCCUPIED, ADDED_HARD, ADDED_BRANCH, UNCERTAIN_OCC = range(7)
CLASS_NAMES = ('free', 'uncertain', 'unobserved', 'occupied', 'added_hard', 'added_branch', 'uncertain_occ')
UNKNOWN_GROUPS = ('uncertain', 'uncertain_occ', 'unobserved')
CLEARANCE_GROUPS = ('hard', 'added_hard', 'branch') + UNKNOWN_GROUPS
REFLECTORS = (OCCUPIED, ADDED_HARD, ADDED_BRANCH)
PGM_NO_DATA = 128
SENSING_MODEL = 'RASTER_V1: occupied reflects; unobserved/outside ends the ray without return; uncertain transparent'
LIDAR_RAYS = 3200                    # harness/raycast_scan beam count (the real S2 emits 3240)
LIDAR_NOISE_SD_M = .015              # same SIM_ASSUMED value as closed_loop.world
LIDAR_MAX_RANGE_M = 30.
CROP_MARGIN_M = 35.                  # sensing is exact while the sensor stays within 5 m of the corridor box
POLYLINE_STEP_M = .25
RECORDS = ('035952', '040927', '042301', '045553', '051127')
# External inputs: no built-in local path. Each comes from its environment variable or from its key in the JSON
# file named by UBTS_STEER_PATHS (closed_loop/external_paths.py, docs/steering/PATHS_KO.md).
MAP_ROOT_ENV = external_paths.KEYS['maps_1010_root']            # UBTS_MAPS_1010
CATALOGUE_ENV = external_paths.KEYS['corridor_catalogue']       # UBTS_SINGLE_ROW_CATALOGUE
SOURCE = 'SLAM_RECONSTRUCTED_20261010_OFFLINE_MAP_NOT_PHYSICAL_TRUTH'


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def sha256_bytes(raw):
    return hashlib.sha256(raw).hexdigest()


# --- map files -----------------------------------------------------------------------------------

def parse_pgm(raw):
    """Binary P5 PGM, maxval 255, '#' comments between header tokens, exactly one whitespace byte after
    maxval and exactly width*height payload bytes. Returns (top-down uint8 pixels, header dict)."""
    if not isinstance(raw, (bytes, bytearray)) or len(raw) < 8:
        raise ValueError('pgm_too_short')
    raw = bytes(raw)
    pos, tokens, comments = 0, [], []
    while len(tokens) < 4:
        if pos >= min(len(raw), 4096):
            raise ValueError('pgm_header_unbounded')
        ch = raw[pos]
        if ch in b' \t\r\n':
            pos += 1
            continue
        if ch == 0x23:                                    # '#': comment to the end of the line
            end = raw.find(b'\n', pos, 4096)
            if end < 0:
                raise ValueError('pgm_comment_unterminated')
            comments.append(raw[pos + 1:end].decode('ascii', 'replace').strip())
            pos = end + 1
            continue
        end = pos
        while end < len(raw) and raw[end] not in b' \t\r\n#':
            end += 1
        tokens.append(raw[pos:end])
        pos = end
    if tokens[0] != b'P5':
        raise ValueError('pgm_not_binary_P5')
    if not all(t.isdigit() and len(t) <= 8 for t in tokens[1:]):
        raise ValueError('pgm_bad_number')
    width, height, maxval = (int(t) for t in tokens[1:])
    if width <= 0 or height <= 0 or maxval != 255:
        raise ValueError('pgm_size_or_maxval')
    if pos >= len(raw) or raw[pos] not in b' \t\r\n':
        raise ValueError('pgm_no_separator')
    pos += 1
    if len(raw) - pos != width * height:
        raise ValueError('pgm_payload_size')
    pixels = np.frombuffer(raw, dtype=np.uint8, count=width * height, offset=pos).reshape(height, width)
    return pixels, dict(width=width, height=height, maxval=maxval, header_bytes=pos, comments=comments)


def parse_map_yaml(text):
    """The flat map_server YAML ('key: value' lines, one [a, b, c] list). No other YAML is accepted."""
    doc = {}
    for line in text.splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        key, sep, value = line.partition(':')
        key, value = key.strip(), value.strip()
        if not sep or not key or key in doc:
            raise ValueError('map_yaml_line')
        if value.startswith('['):
            if not value.endswith(']'):
                raise ValueError('map_yaml_list')
            doc[key] = [float(v) for v in value[1:-1].split(',')]
            continue
        for kind in (int, float):
            try:
                doc[key] = kind(value)
                break
            except ValueError:
                continue
        else:
            doc[key] = value.strip('\'"')
    for key in ('image', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'):
        if key not in doc:
            raise ValueError('map_yaml_missing_' + key)
    if doc.get('mode', 'trinary') != 'trinary':
        raise ValueError('map_yaml_not_trinary')
    if not isinstance(doc['origin'], list) or len(doc['origin']) != 3:
        raise ValueError('map_yaml_origin')
    return doc


def class_lut(occupied_thresh, free_thresh):
    """Pixel value -> class (negate 0). 128 is Cartographer's no-data value (UNOBSERVED)."""
    if not (0. <= free_thresh < occupied_thresh <= 1.):
        raise ValueError('thresholds')
    lut = np.empty(256, dtype=np.uint8)
    for value in range(256):
        p = (255 - value) / 255.
        lut[value] = (OCCUPIED if p > occupied_thresh else FREE if p < free_thresh else
                      UNCERTAIN_OCC if p >= .5 else UNCERTAIN)
    lut[PGM_NO_DATA] = UNOBSERVED
    return lut


def maps_root():
    """10/10 maps folder: UBTS_MAPS_1010, else 'maps_1010_root' of the UBTS_STEER_PATHS file. Raises
    FileNotFoundError (external_paths.PathNotConfigured when neither is set) instead of guessing a path."""
    root = external_paths.configured_path('maps_1010_root', 'dir')
    if not (root / 'offline' / 'maps').is_dir():
        raise FileNotFoundError('the 10/10 maps folder %s has no offline/maps folder' % root)
    return str(root)


def record_paths(record, root=None):
    if record not in RECORDS:
        raise ValueError('unknown_record %r' % (record,))
    base = Path(root or maps_root()) / 'offline' / 'maps' / record
    return base / ('%s_offline.pgm' % record), base / ('%s_offline.yaml' % record)


class RasterMap:
    """Class grid (uint8, row 0 = lowest y) of a whole raster or of a crop of it (offsets row0/col0)."""

    def __init__(self, classes, resolution, info, origin_xy=(0., 0.), row0=0, col0=0):
        self.classes = classes
        self.resolution = float(resolution)
        self.info = info
        self._origin = (float(origin_xy[0]), float(origin_xy[1]))    # map frame of the raster corner (private)
        self.row0, self.col0 = int(row0), int(col0)

    @property
    def shape(self):
        return self.classes.shape

    def map_to_local(self, x, y):
        """Map-frame metres (row analysis, SLAM trajectory) -> raster-local metres."""
        return np.asarray(x, dtype=float) - self._origin[0], np.asarray(y, dtype=float) - self._origin[1]

    def cell_of(self, x, y):
        """(row, col) indices into self.classes (may be out of bounds)."""
        res = self.resolution
        return (np.floor(np.asarray(y, dtype=float) / res).astype(np.int64) - self.row0,
                np.floor(np.asarray(x, dtype=float) / res).astype(np.int64) - self.col0)

    def class_at(self, x, y):
        rows, cols = self.cell_of(x, y)
        rows, cols = np.atleast_1d(rows), np.atleast_1d(cols)
        out = np.full(rows.shape, UNOBSERVED, dtype=np.uint8)
        inside = (rows >= 0) & (rows < self.shape[0]) & (cols >= 0) & (cols < self.shape[1])
        out[inside] = self.classes[rows[inside], cols[inside]]
        return out

    def centres(self, rows, cols):
        res = self.resolution
        return ((np.asarray(cols) + self.col0 + .5) * res, (np.asarray(rows) + self.row0 + .5) * res)

    def crop(self, x0, y0, x1, y1):
        """Writable COPY of the cells covering [x0, x1] x [y0, y1] (clipped to the raster)."""
        res = self.resolution
        r0 = max(0, int(math.floor(y0 / res)) - self.row0)
        r1 = min(self.shape[0], int(math.floor(y1 / res)) - self.row0 + 1)
        c0 = max(0, int(math.floor(x0 / res)) - self.col0)
        c1 = min(self.shape[1], int(math.floor(x1 / res)) - self.col0 + 1)
        if r1 <= r0 or c1 <= c0:
            raise ValueError('empty_crop')
        return RasterMap(self.classes[r0:r1, c0:c1].copy(), res, dict(self.info), self._origin,
                         self.row0 + r0, self.col0 + c0)

    def class_counts(self):
        counts = np.bincount(self.classes.ravel(), minlength=len(CLASS_NAMES))
        return {name: int(counts[i]) for i, name in enumerate(CLASS_NAMES)}


_MAP_CACHE = {}


def load_record_map(record, *, root=None, occupied_thresh=None, expected_pgm_sha256=None,
                    expected_yaml_sha256=None):
    """Whole-raster RasterMap of one record (read-only classes, cached per process).

    Verifies: P5 header, payload size, maxval 255, YAML image name, resolution 0.05 m (YAML and the PGM
    header comment when present), origin yaw 0, negate 0, thresholds; records both SHA-256 values. The
    occupied threshold is the YAML value unless occupied_thresh is given (a stated sensitivity option)."""
    pgm_path, yaml_path = record_paths(record, root)
    key = (str(pgm_path), os.stat(pgm_path).st_size, os.stat(pgm_path).st_mtime_ns,
           os.stat(yaml_path).st_mtime_ns, occupied_thresh)
    rmap = _MAP_CACHE.get(key)
    if rmap is None:
        yaml_raw = yaml_path.read_bytes()
        doc = parse_map_yaml(yaml_raw.decode('utf-8'))
        if doc['image'] != pgm_path.name:
            raise ValueError('map_yaml_image_mismatch')
        if abs(float(doc['resolution']) - .05) > 1e-9:
            raise ValueError('map_resolution_not_0.05')
        if doc['origin'][2] != 0.:
            raise ValueError('map_origin_yaw_not_zero')
        if doc['negate'] != 0:
            raise ValueError('map_negate_not_0')
        raw = pgm_path.read_bytes()
        pixels, header = parse_pgm(raw)
        for comment in header['comments']:
            if 'm/pixel' in comment:
                stated = float(comment.split(';')[-1].split('m/pixel')[0])
                if abs(stated - float(doc['resolution'])) > 1e-6:
                    raise ValueError('pgm_comment_resolution_mismatch')
        used = float(doc['occupied_thresh']) if occupied_thresh is None else float(occupied_thresh)
        classes = class_lut(used, float(doc['free_thresh']))[pixels[::-1]]
        classes.flags.writeable = False
        info = dict(record=record, source=SOURCE, pgm_file=pgm_path.name, yaml_file=yaml_path.name,
                    pgm_sha256=sha256_bytes(raw), yaml_sha256=sha256_bytes(yaml_raw), pgm_bytes=len(raw),
                    width=header['width'], height=header['height'], header_bytes=header['header_bytes'],
                    resolution_m=float(doc['resolution']), yaml_occupied_thresh=float(doc['occupied_thresh']),
                    occupied_thresh=used, free_thresh=float(doc['free_thresh']), negate=int(doc['negate']),
                    unobserved_pixel_value=PGM_NO_DATA, sensing_model=SENSING_MODEL)
        rmap = RasterMap(classes, doc['resolution'], info, origin_xy=doc['origin'][:2])
        rmap.info['class_counts'] = rmap.class_counts()
        _MAP_CACHE[key] = rmap
    if expected_pgm_sha256 and rmap.info['pgm_sha256'] != expected_pgm_sha256:
        raise ValueError('pgm_sha256_mismatch %s' % record)
    if expected_yaml_sha256 and rmap.info['yaml_sha256'] != expected_yaml_sha256:
        raise ValueError('yaml_sha256_mismatch %s' % record)
    return rmap


# --- curvilinear corridor frame -------------------------------------------------------------------------

class CurvilinearFrame:
    """Piecewise-linear centreline: s = arclength - s_origin, d = signed distance (left positive),
    tangent = direction of the closest polyline segment; the first and last segments extend beyond the
    polyline ends. pose() uses the segment that contains s, to_field()/project() the closest segment, so a
    round trip is exact wherever the two agree (everywhere but within mm of a vertex at large |d|)."""

    def __init__(self, points, s_origin=0.):
        pts = np.asarray(points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2 or not np.all(np.isfinite(pts)):
            raise ValueError('polyline')
        seg = np.diff(pts, axis=0)
        length = np.hypot(seg[:, 0], seg[:, 1])
        if np.any(length < 1e-6):
            raise ValueError('polyline_repeated_point')
        self.points = pts
        self.A = pts[:-1]
        self.T = seg / length[:, None]
        self.L = length
        self.angle = np.arctan2(self.T[:, 1], self.T[:, 0])
        self.s_vertex = np.concatenate(([0.], np.cumsum(length))) - float(s_origin)
        self.s_origin = float(s_origin)
        self.length = float(length.sum())
        self._lo, self._hi = np.zeros(len(length)), length.copy()
        self._lo[0], self._hi[-1] = -np.inf, np.inf

    def segment(self, s):
        k = int(np.searchsorted(self.s_vertex, s, side='right')) - 1
        return min(max(k, 0), len(self.L) - 1)

    def tangent(self, s):
        return float(self.angle[self.segment(s)])

    def point(self, s, d=0.):
        k = self.segment(s)
        t = s - self.s_vertex[k]
        T = self.T[k]
        return (float(self.A[k, 0] + t * T[0] - d * T[1]), float(self.A[k, 1] + t * T[1] + d * T[0]))

    def pose(self, s, d=0., yaw=0.):
        x, y = self.point(s, d)
        return x, y, wrap(yaw + self.tangent(s))

    def project(self, x, y):
        """(s, d, closest segment index)."""
        dx, dy = x - self.A[:, 0], y - self.A[:, 1]
        t = dx * self.T[:, 0] + dy * self.T[:, 1]
        tc = np.minimum(np.maximum(t, self._lo), self._hi)
        ex, ey = dx - tc * self.T[:, 0], dy - tc * self.T[:, 1]
        d2 = ex * ex + ey * ey
        k = int(np.argmin(d2))
        cross = self.T[k, 0] * dy[k] - self.T[k, 1] * dx[k]
        dist = math.sqrt(float(d2[k]))
        return float(self.s_vertex[k] + tc[k]), (math.copysign(dist, cross) if dist > 0 else 0.), k

    def to_field(self, x, y, yaw=0.):
        s, d, k = self.project(x, y)
        return s, d, wrap(yaw - float(self.angle[k]))

    def project_many(self, xs, ys, chunk=2048):
        """Vectorised project(): (s, d) arrays."""
        xs, ys = np.asarray(xs, dtype=float).ravel(), np.asarray(ys, dtype=float).ravel()
        s_out, d_out = np.empty(len(xs)), np.empty(len(xs))
        for start in range(0, len(xs), chunk):
            px, py = xs[start:start + chunk, None], ys[start:start + chunk, None]
            dx, dy = px - self.A[None, :, 0], py - self.A[None, :, 1]
            t = dx * self.T[None, :, 0] + dy * self.T[None, :, 1]
            tc = np.minimum(np.maximum(t, self._lo[None, :]), self._hi[None, :])
            ex, ey = dx - tc * self.T[None, :, 0], dy - tc * self.T[None, :, 1]
            d2 = ex * ex + ey * ey
            k = np.argmin(d2, axis=1)
            rows = np.arange(len(k))
            cross = self.T[k, 0] * dy[rows, k] - self.T[k, 1] * dx[rows, k]
            s_out[start:start + chunk] = self.s_vertex[k] + tc[rows, k]
            d_out[start:start + chunk] = np.sqrt(d2[rows, k]) * np.where(cross < 0, -1., 1.)
        return s_out, d_out

    def point_many(self, s, d):
        s, d = np.asarray(s, dtype=float).ravel(), np.asarray(d, dtype=float).ravel()
        k = np.clip(np.searchsorted(self.s_vertex, s, side='right') - 1, 0, len(self.L) - 1)
        t = s - self.s_vertex[k]
        T = self.T[k]
        return self.A[k, 0] + t * T[:, 0] - d * T[:, 1], self.A[k, 1] + t * T[:, 1] + d * T[:, 0]

    def tangent_many(self, s):
        s = np.asarray(s, dtype=float).ravel()
        k = np.clip(np.searchsorted(self.s_vertex, s, side='right') - 1, 0, len(self.L) - 1)
        return self.angle[k]


# --- scenario modifiers (applied to a COPY of the grid) -------------------------------------------------

@dataclass(frozen=True)
class Gap:
    """PREREG gap removal: the OCCUPIED cells of the catalogue support units of one row whose unit centre
    lies in [s_from_m, s_to_m] (travel frame) are set FREE, whole units."""
    side: str
    s_from_m: float
    s_to_m: float
    kind: str = 'gap'


@dataclass(frozen=True)
class RemoveTrunks:
    """`count` consecutive catalogue support units of one row, from the first whose centre s >= s_from_m:
    OCCUPIED cells set FREE, whole units (as Gap)."""
    side: str
    s_from_m: float
    count: int = 1
    kind: str = 'remove_trunks'


@dataclass(frozen=True)
class BranchRect:
    """PREREG branch protrusion: ADDED_BRANCH cells with s in [s_m - along_m/2, s_m + along_m/2] and d from
    the row position (catalogue profile) of `side` to |d| = tip_abs_d_m. Hard cells stay hard."""
    side: str
    s_m: float
    tip_abs_d_m: float = .95
    along_m: float = .20
    kind: str = 'branch_rect'


@dataclass(frozen=True)
class BranchBlob:
    """Soft capsule (ADDED_BRANCH) from the row position toward the centre; its tip reaches reach_m from the
    row position. Hard cells stay hard."""
    side: str
    s_m: float
    reach_m: float
    radius_m: float = .12
    kind: str = 'branch_blob'


@dataclass(frozen=True)
class ObstacleBox:
    """PREREG obstacle: ADDED_HARD box centred at (s_m, d_m), sides parallel/normal to the local tangent."""
    s_m: float
    d_m: float = 0.
    length_m: float = .6
    width_m: float = .6
    kind: str = 'obstacle_box'


@dataclass(frozen=True)
class Wall:
    """PREREG wall: ADDED_HARD, near face at end + beyond_end_m along the END tangent (the segment at the row
    end), normal to it, thickness_m thick, |d| <= half_length_m (None: spacing_median / 2 + 1.5 m)."""
    beyond_end_m: float
    thickness_m: float = .10
    half_length_m: float | None = None
    kind: str = 'wall'


MODIFIER_TYPES = dict(gap=Gap, remove_trunks=RemoveTrunks, branch_rect=BranchRect, branch_blob=BranchBlob,
                      obstacle_box=ObstacleBox, wall=Wall)


def modifier_from_dict(data):
    data = dict(data)
    return MODIFIER_TYPES[data.pop('kind')](**data)


# PREREG placement_rules (single-row-20261102/PREREG.json) as modifier builders; s_end = row end (end_s_m).
def prereg_gap(sides, s_end, length_m, centre_fraction):
    """centre = clip(centre_fraction x s_E, 4.0 + G/2, s_E - 4.0 - G/2); interval [centre - G/2, centre + G/2]."""
    centre = min(max(centre_fraction * s_end, 4. + length_m / 2), s_end - 4. - length_m / 2)
    return tuple(Gap(side, centre - length_m / 2, centre + length_m / 2) for side in sides)


def prereg_branches(s_end, first_s_m=6., pitch_m=8., last_before_end_m=4., along_m=.2, tip_abs_d_m=.95,
                    first_side='left'):
    """Rectangles at s = first + pitch k (k >= 0, s <= s_E - last_before_end), sides alternating."""
    out, k = [], 0
    other = 'right' if first_side == 'left' else 'left'
    while first_s_m + pitch_m * k <= s_end - last_before_end_m:
        out.append(BranchRect(first_side if k % 2 == 0 else other, first_s_m + pitch_m * k, tip_abs_d_m, along_m))
        k += 1
    return tuple(out)


def prereg_obstacle(s_end, size_m=(.6, .6), d_m=0.):
    """Box centred at (s = max(8.0, 0.5 s_E), d), sides parallel/normal to the local tangent."""
    return (ObstacleBox(max(8., .5 * s_end), d_m, float(size_m[0]), float(size_m[1])),)


def prereg_wall(distance_m):
    """Near face at s_E + D, normal to the end tangent, 0.10 m thick, |d| <= spacing_median / 2 + 1.5."""
    return (Wall(distance_m, .10, None),)


# --- harness world config --------------------------------------------------------------------------------

@dataclass(frozen=True)
class RasterCorridorConfig:
    """Harness world config of one catalogued corridor in one travel direction (geometry embedded, the map
    pinned by SHA-256). Build it with corridor_world_config(); row_width_m / headland_depth_m / scenario
    are the attributes closed_loop.harness reads from a world config."""
    record: str = ''
    corridor_id: str = ''
    direction: str = 'fwd'
    scenario: str = 'raster_corridor'
    polyline: tuple = ()             # ((x, y), ...) raster-local metres, oriented in the travel direction
    s_origin_m: float = 0.           # polyline arclength of field s = 0 (both rows begin)
    end_s_m: float = 0.              # field s of the last trunk on BOTH rows (corridor_truth end_x)
    line_end_s_m: tuple = ()         # (left, right): field s of each row's last trunk
    profile_s_m: tuple = ()          # 1 m windows (field s)
    left_row_d_m: tuple = ()         # row centre d (median of the row's occupied cells; fallback line fit)
    right_row_d_m: tuple = ()
    left_units: tuple = ()           # ((s, d), ...) catalogue support-unit centres of each row (travel frame)
    right_units: tuple = ()
    row_width_m: float = 3.7         # median row-centre spacing (harness left_corridor uses width/2 - footprint/2)
    band_m: float = .75              # row band half-width used by the catalogue (side assignment)
    headland_depth_m: float = 3.     # harness 'left_field' check at end_x + headland_depth_m + 6
    pgm_sha256: str = ''
    yaml_sha256: str = ''
    occupied_thresh: float | None = None   # None: the map YAML value
    catalogue_sha256: str = ''
    modifiers: tuple = ()
    seed: int = 0                    # no random scene draws in a raster world (kept for config plumbing)


def catalogue_path(path=None):
    """The given path, else UBTS_SINGLE_ROW_CATALOGUE, else 'corridor_catalogue' of the UBTS_STEER_PATHS file.
    Raises FileNotFoundError (external_paths.PathNotConfigured when neither is set) instead of guessing."""
    if path:
        return Path(path)
    return external_paths.configured_path('corridor_catalogue', 'file')


def load_catalogue(path=None):
    path = catalogue_path(path)
    raw = path.read_bytes()
    data = json.loads(raw.decode('utf-8'))
    data['_sha256'] = sha256_bytes(raw)
    data['_path'] = str(path)
    return data


def find_entry(catalogue, record, corridor_id):
    for entry in catalogue['corridors']:
        if entry['record'] == record and entry['corridor_id'] == corridor_id:
            return entry
    raise KeyError('%s/%s not in the catalogue' % (record, corridor_id))


def oriented_geometry(entry, direction):
    """Travel-direction geometry from a catalogue entry (stored in the forward field frame)."""
    if direction not in ('fwd', 'rev'):
        raise ValueError('direction')
    geo, prof = entry['geometry'], entry['profile']
    points = [tuple(p) for p in geo['polyline_fwd']]
    s_w = list(prof['s_m'])
    left, right = list(prof['left_row_d_m']), list(prof['right_row_d_m'])
    d = entry['directions'][direction]
    start, end = d['start_s_fwd_m'], d['end_s_fwd_m']
    first, last = entry['sides']['left']['first_s_m'], entry['sides']['left']['last_s_m']
    rfirst, rlast = entry['sides']['right']['first_s_m'], entry['sides']['right']['last_s_m']
    units = {side: list(zip(entry['sides'][side]['unit_s_m'], entry['sides'][side]['unit_d_m']))
             for side in ('left', 'right')}
    if direction == 'fwd':
        return dict(polyline=tuple(points), s_origin_m=geo['s_origin_fwd_m'] + start, end_s_m=end - start,
                    line_end_s_m=(last - start, rlast - start), profile_s_m=tuple(v - start for v in s_w),
                    left_row_d_m=tuple(left), right_row_d_m=tuple(right),
                    left_units=tuple((s - start, d) for s, d in units['left']),
                    right_units=tuple((s - start, d) for s, d in units['right']))
    total = CurvilinearFrame(points).length
    # rev field s' = start - s_fwd, d' = -d, left/right swap
    return dict(polyline=tuple(points[::-1]), s_origin_m=total - start - geo['s_origin_fwd_m'],
                end_s_m=start - end, line_end_s_m=(start - rfirst, start - first),
                profile_s_m=tuple(start - v for v in s_w[::-1]),
                left_row_d_m=tuple(-v for v in right[::-1]), right_row_d_m=tuple(-v for v in left[::-1]),
                left_units=tuple(sorted((start - s, -d) for s, d in units['right'])),
                right_units=tuple(sorted((start - s, -d) for s, d in units['left'])))


def corridor_world_config(record, corridor_id, direction='fwd', *, modifiers=(), catalogue=None,
                          occupied_thresh=None, scenario=None, headland_depth_m=3., allow_excluded=False):
    """RasterCorridorConfig from the catalogue (default: the configured one, see catalogue_path)."""
    catalogue = catalogue if isinstance(catalogue, dict) else load_catalogue(catalogue)
    entry = find_entry(catalogue, record, corridor_id)
    status = entry['directions'][direction]['status']
    if status != 'included' and not allow_excluded:
        raise ValueError('%s/%s %s is %s (%s)' % (record, corridor_id, direction, status,
                                                  entry['directions'][direction]['exclusions']))
    geo = oriented_geometry(entry, direction)
    for modifier in modifiers:
        if not isinstance(modifier, tuple(MODIFIER_TYPES.values())):
            raise TypeError('modifier %r' % (modifier,))
    return RasterCorridorConfig(
        record=record, corridor_id=corridor_id, direction=direction,
        scenario=scenario or 'raster_%s_%s_%s' % (record, corridor_id, direction),
        row_width_m=float(entry['spacing_median_m']), band_m=float(entry['band_m']),
        headland_depth_m=float(headland_depth_m), pgm_sha256=entry['map']['pgm_sha256'],
        yaml_sha256=entry['map']['yaml_sha256'], occupied_thresh=occupied_thresh,
        catalogue_sha256=catalogue.get('_sha256', ''), modifiers=tuple(modifiers), **geo)


# --- world -------------------------------------------------------------------------------------------------

def footprint_clearances(rmap, x, y, yaw, length, width, reach=2.):
    """Conservative footprint-rectangle clearance per class group: cell-centre distance - res/sqrt(2) (a cell
    within that of the rectangle may touch it); <= 0 is contact. Cells outside the raster count as
    unobserved; inf = nothing of that group within `reach` of the footprint."""
    res = rmap.resolution
    radius = math.hypot(length / 2, width / 2) + reach + res
    r_lo = int(math.floor((y - radius) / res)) - rmap.row0
    r_hi = int(math.floor((y + radius) / res)) - rmap.row0 + 1
    c_lo = int(math.floor((x - radius) / res)) - rmap.col0
    c_hi = int(math.floor((x + radius) / res)) - rmap.col0 + 1
    sub = np.full((r_hi - r_lo, c_hi - c_lo), UNOBSERVED, dtype=np.uint8)
    a0, a1 = max(r_lo, 0), min(r_hi, rmap.shape[0])
    b0, b1 = max(c_lo, 0), min(c_hi, rmap.shape[1])
    if a1 > a0 and b1 > b0:
        sub[a0 - r_lo:a1 - r_lo, b0 - c_lo:b1 - c_lo] = rmap.classes[a0:a1, b0:b1]
    rows, cols = np.nonzero(sub != FREE)
    out = {name: math.inf for name in CLEARANCE_GROUPS}
    if not len(rows):
        return out
    cls = sub[rows, cols]
    px = (cols + c_lo + rmap.col0 + .5) * res - x
    py = (rows + r_lo + rmap.row0 + .5) * res - y
    c, s = math.cos(yaw), math.sin(yaw)
    ox = np.abs(c * px + s * py) - length / 2
    oy = np.abs(-s * px + c * py) - width / 2
    dist = np.hypot(np.maximum(ox, 0.), np.maximum(oy, 0.)) + np.minimum(np.maximum(ox, oy), 0.)
    dist -= res / math.sqrt(2.)
    for name, mask in (('hard', (cls == OCCUPIED) | (cls == ADDED_HARD)), ('added_hard', cls == ADDED_HARD),
                       ('branch', cls == ADDED_BRANCH), ('uncertain', cls == UNCERTAIN),
                       ('uncertain_occ', cls == UNCERTAIN_OCC), ('unobserved', cls == UNOBSERVED)):
        if mask.any():
            out[name] = float(dist[mask].min())
    return out


def _rect_cells(rmap, cx, cy, heading, half_along, half_across):
    """(rows, cols) of the cells whose centres lie inside an oriented rectangle."""
    radius = math.hypot(half_along, half_across) + rmap.resolution
    rows, cols = _box_cells(rmap, cx - radius, cy - radius, cx + radius, cy + radius)
    x, y = rmap.centres(rows, cols)
    c, s = math.cos(heading), math.sin(heading)
    along, across = c * (x - cx) + s * (y - cy), -s * (x - cx) + c * (y - cy)
    keep = (np.abs(along) <= half_along) & (np.abs(across) <= half_across)
    return rows[keep], cols[keep]


def _box_cells(rmap, x0, y0, x1, y1, where=None):
    """(rows, cols) of the cells overlapping an axis-aligned box (optionally only the classes in `where`)."""
    res = rmap.resolution
    r0 = max(0, int(math.floor(y0 / res)) - rmap.row0)
    r1 = min(rmap.shape[0], int(math.floor(y1 / res)) - rmap.row0 + 1)
    c0 = max(0, int(math.floor(x0 / res)) - rmap.col0)
    c1 = min(rmap.shape[1], int(math.floor(x1 / res)) - rmap.col0 + 1)
    if r1 <= r0 or c1 <= c0:
        return np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.int64)
    if where is not None:
        rr, cc = np.nonzero(np.isin(rmap.classes[r0:r1, c0:c1], where))
        return rr + r0, cc + c0
    rr, cc = np.mgrid[r0:r1, c0:c1]
    return rr.ravel(), cc.ravel()


def _side_row_d(config, side, s):
    """Row centre d of one side (travel frame) at field s (catalogue 1 m profile, linear interpolation)."""
    values = config.left_row_d_m if side == 'left' else config.right_row_d_m
    return np.interp(s, np.asarray(config.profile_s_m), np.asarray(values))


def _band_cells(rmap, frame, config, side, s_lo, s_hi, extra=0., where=None):
    """Cells of one row band (|d - row d(s)| <= band + extra) with lo <= s <= hi: rows, cols, s, d.
    where: only these classes (filtered before the projection, so long diagonal bands stay cheap)."""
    band = config.band_m + extra
    ss = np.arange(s_lo, s_hi + .5, .5)
    dd = _side_row_d(config, side, ss)
    xs, ys = [], []
    for offset in (-band - .3, band + .3):
        x, y = frame.point_many(ss, dd + offset)
        xs.append(x)
        ys.append(y)
    xs, ys = np.concatenate(xs), np.concatenate(ys)
    rows, cols = _box_cells(rmap, xs.min(), ys.min(), xs.max(), ys.max(), where)
    x, y = rmap.centres(rows, cols)
    s, d = frame.project_many(x, y)
    keep = (s >= s_lo) & (s <= s_hi) & (np.abs(d - _side_row_d(config, side, s)) <= band)
    return rows[keep], cols[keep], s[keep], d[keep]


def _components(rows, cols):
    """Component label per cell (8-connected after a 1-cell dilation: the catalogue / row-analysis rule)."""
    from scipy import ndimage
    if not len(rows):
        return np.zeros(0, dtype=np.int64)
    r0, c0 = rows.min(), cols.min()
    mask = np.zeros((rows.max() - r0 + 1, cols.max() - c0 + 1), dtype=bool)
    mask[rows - r0, cols - c0] = True
    labels, _ = ndimage.label(ndimage.binary_dilation(mask, structure=np.ones((3, 3), bool)),
                              structure=np.ones((3, 3), bool))
    return labels[rows - r0, cols - c0]


UNIT_MATCH_M = .06                   # catalogue unit centres are rounded to 0.01 m (s) / 0.001 m (d)


def support_units(rmap, frame, config, side):
    """The catalogue support units of one row matched to raster components: OCCUPIED cells of the row band
    + 2 m, components as in the catalogue, centroid (mean cell centre, projected) within UNIT_MATCH_M of the
    catalogue unit centre. Returns (matched [(s, d, rows, cols)], unmatched [(s, d)])."""
    units = config.left_units if side == 'left' else config.right_units
    if not units:
        return [], []
    s_vals = [u[0] for u in units]
    rows, cols, _, _ = _band_cells(rmap, frame, config, side, min(s_vals) - 3., max(s_vals) + 3., extra=2.,
                                   where=(OCCUPIED,))
    labels = _components(rows, cols)
    comps = []
    for value in np.unique(labels):
        sel = labels == value
        x, y = rmap.centres(rows[sel], cols[sel])
        cs, cd, _ = frame.project(float(x.mean()), float(y.mean()))
        comps.append((cs, cd, rows[sel], cols[sel]))
    matched, unmatched = [], []
    for us, ud in units:
        best = min(comps, key=lambda c: (c[0] - us) ** 2 + (c[1] - ud) ** 2) if comps else None
        if best is not None and abs(best[0] - us) <= UNIT_MATCH_M and abs(best[1] - ud) <= UNIT_MATCH_M:
            matched.append((us, ud, best[2], best[3]))
        else:
            unmatched.append((us, ud))
    return matched, unmatched


def _frame_box_cells(rmap, frame, s_lo, s_hi, d_lo, d_hi):
    """Cells whose centre projects into [s_lo, s_hi] x [d_lo, d_hi] (small regions only)."""
    corners = [frame.point(s, d) for s in (s_lo, s_hi) for d in (d_lo, d_hi)]
    xs, ys = [c[0] for c in corners], [c[1] for c in corners]
    pad = rmap.resolution
    rows, cols = _box_cells(rmap, min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)
    x, y = rmap.centres(rows, cols)
    s, d = frame.project_many(x, y)
    keep = (s >= s_lo) & (s <= s_hi) & (d >= d_lo) & (d <= d_hi)
    return rows[keep], cols[keep]


def apply_modifiers(rmap, frame, config):
    """Apply config.modifiers to rmap (a writable crop copy). Returns (log, wall near-face s list)."""
    log, walls = [], []
    for modifier in config.modifiers:
        entry = asdict(modifier)
        if modifier.kind in ('gap', 'remove_trunks'):
            if modifier.side not in ('left', 'right'):
                raise ValueError(modifier.kind + ': side')
            matched, unmatched = support_units(rmap, frame, config, modifier.side)
            matched.sort(key=lambda u: u[0])
            if modifier.kind == 'gap':
                if not modifier.s_from_m < modifier.s_to_m:
                    raise ValueError('gap: s_from_m < s_to_m')
                chosen = [u for u in matched if modifier.s_from_m <= u[0] <= modifier.s_to_m]
            else:
                if modifier.count < 1:
                    raise ValueError('remove_trunks: count')
                first = next((i for i, u in enumerate(matched) if u[0] >= modifier.s_from_m), None)
                chosen = [] if first is None else matched[first:first + modifier.count]
                if len(chosen) < modifier.count:
                    raise ValueError('remove_trunks: only %d units after s=%.2f' % (len(chosen), modifier.s_from_m))
            removed = 0
            for _, _, rows, cols in chosen:
                occupied = rmap.classes[rows, cols] == OCCUPIED
                rmap.classes[rows[occupied], cols[occupied]] = FREE
                removed += int(occupied.sum())
            entry.update(removed_unit_s_m=[round(u[0], 3) for u in chosen], removed_cells=removed,
                         unmatched_catalogue_units=len(unmatched), catalogue_units=len(matched) + len(unmatched))
        elif modifier.kind == 'branch_rect':
            if modifier.side not in ('left', 'right') or modifier.along_m <= 0:
                raise ValueError('branch_rect')
            row_d = float(_side_row_d(config, modifier.side, modifier.s_m))
            tip = modifier.tip_abs_d_m if modifier.side == 'left' else -modifier.tip_abs_d_m
            rows, cols = _frame_box_cells(rmap, frame, modifier.s_m - modifier.along_m / 2,
                                          modifier.s_m + modifier.along_m / 2, min(row_d, tip), max(row_d, tip))
            soft = ~np.isin(rmap.classes[rows, cols], (OCCUPIED, ADDED_HARD))
            rmap.classes[rows[soft], cols[soft]] = ADDED_BRANCH
            entry.update(row_d_m=round(row_d, 3), tip_d_m=round(tip, 3), cells=int(soft.sum()))
        elif modifier.kind == 'branch_blob':
            if modifier.side not in ('left', 'right') or modifier.reach_m <= 0 or modifier.radius_m <= 0:
                raise ValueError('branch_blob')
            row_d = float(_side_row_d(config, modifier.side, modifier.s_m))
            inward = -1. if modifier.side == 'left' else 1.
            tip_axis = row_d + inward * max(modifier.reach_m - modifier.radius_m, 0.)
            ax, ay = frame.point(modifier.s_m, row_d)
            bx, by = frame.point(modifier.s_m, tip_axis)
            r = modifier.radius_m
            rows, cols = _box_cells(rmap, min(ax, bx) - r, min(ay, by) - r, max(ax, bx) + r, max(ay, by) + r)
            x, y = rmap.centres(rows, cols)
            vx, vy = bx - ax, by - ay
            norm2 = max(vx * vx + vy * vy, 1e-12)
            u = np.clip(((x - ax) * vx + (y - ay) * vy) / norm2, 0., 1.)
            inside = np.hypot(x - ax - u * vx, y - ay - u * vy) <= r
            rows, cols = rows[inside], cols[inside]
            soft = ~np.isin(rmap.classes[rows, cols], (OCCUPIED, ADDED_HARD))
            rmap.classes[rows[soft], cols[soft]] = ADDED_BRANCH
            entry.update(row_d_m=round(row_d, 3), tip_d_m=round(row_d + inward * modifier.reach_m, 3),
                         cells=int(soft.sum()))
        elif modifier.kind == 'obstacle_box':
            cx, cy, heading = frame.pose(modifier.s_m, modifier.d_m, 0.)
            rows, cols = _rect_cells(rmap, cx, cy, heading, modifier.length_m / 2, modifier.width_m / 2)
            rmap.classes[rows, cols] = ADDED_HARD
            entry.update(cells=int(len(rows)))
        elif modifier.kind == 'wall':
            if modifier.thickness_m <= 0:
                raise ValueError('wall: thickness')
            half = (config.row_width_m / 2 + 1.5) if modifier.half_length_m is None else modifier.half_length_m
            heading = frame.tangent(config.end_s_m)                        # the END tangent
            ex, ey = frame.point(config.end_s_m, 0.)
            fx, fy = ex + modifier.beyond_end_m * math.cos(heading), ey + modifier.beyond_end_m * math.sin(heading)
            cx = fx + modifier.thickness_m / 2 * math.cos(heading)
            cy = fy + modifier.thickness_m / 2 * math.sin(heading)
            rows, cols = _rect_cells(rmap, cx, cy, heading, modifier.thickness_m / 2, half)
            rmap.classes[rows, cols] = ADDED_HARD
            near_s = frame.project(fx, fy)[0]
            walls.append(near_s)
            entry.update(near_face_s_m=round(near_s, 3), half_length_m=round(half, 3), cells=int(len(rows)))
        else:
            raise ValueError('unknown modifier %r' % (modifier,))
        log.append(entry)
    return log, walls


class RasterCorridorWorld:
    """Harness world (to_field / pose_in_field / corridor_truth + truth clearance) of one corridor."""

    def __init__(self, config, rmap, frame, modifier_log=(), walls_s=()):
        self.config = config
        self.map = rmap
        self.frame = frame
        self.s_end = float(config.end_s_m)
        self.walls_s = tuple(float(w) for w in walls_s)
        self.modifier_log = list(modifier_log)
        self.contacts = dict(samples=0, **{name: 0 for name in CLEARANCE_GROUPS})
        self.min_clearance = {name: math.inf for name in CLEARANCE_GROUPS}
        self.lidar_stats = None

    def to_field(self, x, y, yaw=0.):
        return self.frame.to_field(x, y, yaw)

    def pose_in_field(self, fx, fy, fyaw=0.):
        return self.frame.pose(fx, fy, fyaw)

    def corridor_truth(self, k):
        if k != 0:
            raise ValueError('a raster corridor world has one corridor (index 0)')
        return dict(end_x=self.s_end, line_end_x=tuple(self.config.line_end_s_m), cross_walls_x=self.walls_s,
                    centre_y=0., start_x=0., next_left='UNKNOWN', next_right='UNKNOWN')

    def clearances(self, x, y, yaw, length, width, reach=2.):
        return footprint_clearances(self.map, x, y, yaw, length, width, reach)

    def clearance(self, x, y, yaw, length, width):
        """Harness clearance_evaluator contract: (hard, branch); unknown contact is counted here."""
        c = self.clearances(x, y, yaw, length, width)
        self.contacts['samples'] += 1
        for name, value in c.items():
            self.min_clearance[name] = min(self.min_clearance[name], value)
            if value <= 0:
                self.contacts[name] += 1
        return c['hard'], c['branch']

    def describe(self):
        info = self.map.info
        return dict(source=SOURCE, record=self.config.record, corridor_id=self.config.corridor_id,
                    direction=self.config.direction, pgm_sha256=info['pgm_sha256'], yaml_sha256=info['yaml_sha256'],
                    catalogue_sha256=self.config.catalogue_sha256, occupied_thresh=info['occupied_thresh'],
                    free_thresh=info['free_thresh'], sensing_model=SENSING_MODEL, end_s_m=round(self.s_end, 3),
                    walls_s_m=[round(w, 3) for w in self.walls_s], modifiers=self.modifier_log,
                    contact_samples=dict(self.contacts),
                    min_clearance_m={k: (round(v, 3) if math.isfinite(v) else None)
                                     for k, v in self.min_clearance.items()},
                    lidar=None if self.lidar_stats is None else self.lidar_stats.summary(),
                    unknown_is_verified_free=False, controller_receives_map_or_pose=False,
                    physical_truth=False)


def build_raster_world(config):
    """harness world_factory: pinned map -> crop around the corridor -> modifiers (on the copy) -> world."""
    if not isinstance(config, RasterCorridorConfig):
        raise TypeError('build_raster_world needs a RasterCorridorConfig')
    base = load_record_map(config.record, occupied_thresh=config.occupied_thresh,
                           expected_pgm_sha256=config.pgm_sha256 or None,
                           expected_yaml_sha256=config.yaml_sha256 or None)
    frame = CurvilinearFrame(config.polyline, config.s_origin_m)
    pts = frame.points
    box = (pts[:, 0].min() - CROP_MARGIN_M, pts[:, 1].min() - CROP_MARGIN_M,
           pts[:, 0].max() + CROP_MARGIN_M, pts[:, 1].max() + CROP_MARGIN_M)
    rmap = base.crop(*box)
    # The crop edge ends rays without a return (it lies >= 35 m from the corridor box; see make_raster_lidar).
    rmap.classes[[0, -1], :] = UNOBSERVED
    rmap.classes[:, [0, -1]] = UNOBSERVED
    log, walls = apply_modifiers(rmap, frame, config)
    rmap.classes.flags.writeable = False
    world = RasterCorridorWorld(config, rmap, frame, log, walls)
    # Sensing equals the whole-raster sensing while the 30 m disc stays inside the REQUESTED crop box (where the
    # crop was clipped at the raster edge, outside the raster is unobserved = no return in both).
    margin = LIDAR_MAX_RANGE_M + rmap.resolution
    world.exact_box = (box[0] + margin, box[1] + margin, box[2] - margin, box[3] - margin)
    return world


# --- LiDAR ----------------------------------------------------------------------------------------------

def _circular_range_max(values, lo, count):
    """max(values[(lo + i) % n] for i in range(count)) per query (sparse table on the doubled array)."""
    n = len(values)
    ext = np.concatenate((values, values))
    levels = [ext]
    size = 1
    while size * 2 <= int(count.max()):
        prev = levels[-1]
        nxt = np.full(len(ext), -math.inf)
        nxt[:len(ext) - size] = np.maximum(prev[:len(ext) - size], prev[size:])
        levels.append(nxt)
        size *= 2
    table = np.stack(levels)
    level = np.floor(np.log2(np.maximum(count, 1))).astype(np.int64)
    width = np.left_shift(1, level)
    return np.maximum(table[level, lo], table[level, lo + count - width])


class ScanTimer:
    def __init__(self):
        self.ms = []
        self.outside_exact = 0

    def summary(self):
        ms = np.asarray(self.ms) if self.ms else np.zeros(1)
        return dict(scans=len(self.ms), ms_p50=round(float(np.percentile(ms, 50)), 3),
                    ms_p99=round(float(np.percentile(ms, 99)), 3), ms_max=round(float(ms.max()), 3),
                    ms_mean=round(float(ms.mean()), 3), scans_sensor_outside_exact_region=self.outside_exact)


class RasterLidarCaster:
    """ScanSynthesizer scanner (scan(xyz, rotation) -> ranges, origin, directions) over a raster world.

    Exact ray/cell-square intersection (slab test) against the reflector cells (occupied + added) and the
    UNOBSERVED cells bordering observed space (a ray leaving observed space must cross one of them first);
    the nearest hit wins, a terminator (or a tie) gives no return. Candidate cells come from a KD-tree query
    of the 30 m disc; only rays inside a cell's circumscribed angular interval are tested. Equivalent to a
    supercover DDA in which reflectors return and unobserved cells end the ray (see the tests: the reference
    RasterGrid.raycast of mapping-rotation-20261022 RASTER/raster_world.py with unobserved -> UNKNOWN)."""

    def __init__(self, world, rays=LIDAR_RAYS, maximum=LIDAR_MAX_RANGE_M):
        from scipy import ndimage
        from scipy.spatial import cKDTree
        rmap = world.map
        self.world, self.rays, self.maximum = world, int(rays), float(maximum)
        self.res = rmap.resolution
        self.half = rmap.resolution / 2
        self.angles = -math.pi + np.arange(self.rays) * (2 * math.pi / self.rays)
        self.step = 2 * math.pi / self.rays
        self.occluded_sectors = ()
        cls = rmap.classes
        unobserved = cls == UNOBSERVED
        boundary = unobserved & ndimage.binary_dilation(~unobserved, structure=np.ones((3, 3), bool))
        rr, rc = np.nonzero(np.isin(cls, REFLECTORS))
        tr, tc = np.nonzero(boundary)
        x, y = rmap.centres(np.concatenate((rr, tr)), np.concatenate((rc, tc)))
        self.cx, self.cy = x, y
        self.reflector = np.concatenate((np.ones(len(rr), bool), np.zeros(len(tr), bool)))
        self.tree = cKDTree(np.column_stack((x, y))) if len(x) else None
        exact = getattr(world, 'exact_box', None)
        if exact is None:
            x0, y0 = rmap.centres(0, 0)
            x1, y1 = rmap.centres(rmap.shape[0] - 1, rmap.shape[1] - 1)
            margin = self.maximum + self.res
            exact = (x0 + margin, y0 + margin, x1 - margin, y1 - margin)
        self.exact_box = exact
        self.timer = ScanTimer()
        world.lidar_stats = self.timer

    def cast(self, sx, sy, yaw):
        """Ranges (inf = no return) of the rays angles + yaw from (sx, sy).

        Pass 1: nearest reflector per ray. Pass 2: only terminator cells that can lie before a ray's reflector
        (r - h sqrt 2 <= the circular range maximum of the finite reflector ranges over the cell's ray interval;
        a ray without a reflector returns nothing whatever the terminators) - an exact pruning."""
        n, h = self.rays, self.half
        out = np.full(n, math.inf)
        own = int(self.world.map.class_at(sx, sy)[0])
        if own == UNOBSERVED:
            return out
        if own in REFLECTORS:
            return np.zeros(n)
        if self.tree is None:
            return out
        idx = np.asarray(self.tree.query_ball_point((sx, sy), self.maximum + h * math.sqrt(2.) + 1e-6),
                         dtype=np.int64)
        if not len(idx):
            return out
        rx, ry = self.cx[idx] - sx, self.cy[idx] - sy
        r = np.hypot(rx, ry)
        theta = np.arctan2(ry, rx)
        near = r <= h * math.sqrt(2.) + 1e-9
        alpha = np.arcsin(np.minimum(1., (h * math.sqrt(2.) + 1e-9) / np.maximum(r, 1e-12)))
        a0 = yaw - math.pi
        lo = np.ceil((theta - alpha - a0) / self.step - 1e-9).astype(np.int64)
        hi = np.floor((theta + alpha - a0) / self.step + 1e-9).astype(np.int64)
        count = np.clip(hi - lo + 1, 0, n)
        lo[near], count[near] = 0, n
        lo %= n
        reflect = self.reflector[idx]
        best_reflector = self._first_hits(rx, ry, lo, count, reflect & (count > 0), yaw)
        if not np.isfinite(best_reflector).any():
            return out
        candidates = ~reflect & (count > 0)
        if candidates.any():
            finite = np.where(np.isfinite(best_reflector), best_reflector, -math.inf)
            reach = _circular_range_max(finite, lo[candidates], count[candidates])
            keep = np.zeros(len(idx), dtype=bool)
            keep[np.flatnonzero(candidates)[r[candidates] - h * math.sqrt(2.) <= reach]] = True
            best_terminator = self._first_hits(rx, ry, lo, count, keep, yaw)
        else:
            best_terminator = np.full(n, math.inf)
        out = np.where(best_reflector < best_terminator, best_reflector, math.inf)
        out[out > self.maximum] = math.inf
        return out

    def _first_hits(self, rx, ry, lo, count, select, yaw):
        """Per ray: the smallest entry distance into the selected cell squares (inf: none)."""
        n, h = self.rays, self.half
        best = np.full(n, math.inf)
        sel = np.flatnonzero(select)
        if not len(sel):
            return best
        cnt = count[sel]
        total = int(cnt.sum())
        cell = np.repeat(sel, cnt)
        offset = np.repeat(np.cumsum(cnt) - cnt, cnt)
        k = (lo[cell] + (np.arange(total) - offset)) % n
        phi = self.angles[k] + yaw
        dx, dy = np.cos(phi), np.sin(phi)
        ex, ey = rx[cell], ry[cell]
        with np.errstate(divide='ignore', invalid='ignore'):
            tx0, tx1 = (ex - h) / dx, (ex + h) / dx
            ty0, ty1 = (ey - h) / dy, (ey + h) / dy
        tminx, tmaxx = np.minimum(tx0, tx1), np.maximum(tx0, tx1)
        tminy, tmaxy = np.minimum(ty0, ty1), np.maximum(ty0, ty1)
        zero_x, zero_y = dx == 0, dy == 0
        if zero_x.any():
            inside = (ex - h <= 0) & (ex + h >= 0)
            tminx = np.where(zero_x, np.where(inside, -np.inf, np.inf), tminx)
            tmaxx = np.where(zero_x, np.where(inside, np.inf, -np.inf), tmaxx)
        if zero_y.any():
            inside = (ey - h <= 0) & (ey + h >= 0)
            tminy = np.where(zero_y, np.where(inside, -np.inf, np.inf), tminy)
            tmaxy = np.where(zero_y, np.where(inside, np.inf, -np.inf), tmaxy)
        enter, leave = np.maximum(tminx, tminy), np.minimum(tmaxx, tmaxy)
        t = np.where((enter <= leave) & (leave >= 0), np.maximum(enter, 0.), np.inf)
        np.minimum.at(best, k, t)
        return best

    def scan(self, xyz, rotation):
        started = time.perf_counter()
        yaw = math.atan2(rotation[1][0], rotation[0][0])
        sx, sy = float(xyz[0]), float(xyz[1])
        x0, y0, x1, y1 = self.exact_box
        if not (x0 <= sx <= x1 and y0 <= sy <= y1):
            self.timer.outside_exact += 1
        ranges = self.cast(sx, sy, yaw)
        for lo, hi in self.occluded_sectors:
            ranges[(self.angles >= lo) & (self.angles <= hi)] = math.inf
        directions = np.column_stack((np.cos(self.angles + yaw), np.sin(self.angles + yaw),
                                      np.zeros_like(self.angles)))
        self.timer.ms.append((time.perf_counter() - started) * 1000.)
        return ranges, np.array((xyz[0], xyz[1], xyz[2])), directions


def make_raster_lidar(world, seed, noise_sd=LIDAR_NOISE_SD_M):
    """harness lidar_factory: the same ScanSynthesizer contract as closed_loop.world.make_lidar."""
    from cpu_lidar import ScanSynthesizer
    caster = RasterLidarCaster(world)
    synth = ScanSynthesizer(caster, count=LIDAR_RAYS, hz=10., minimum=.05, maximum=LIDAR_MAX_RANGE_M,
                            noise_stddev=noise_sd, seed=seed)
    return caster, synth


def raster_clearance(world, x, y, yaw, length, width):
    """harness clearance_evaluator: (hard, branch); unknown contact is counted on the world."""
    return world.clearance(x, y, yaw, length, width)


class RasterHooks:
    """The three harness hooks for raster worlds; keeps the episode's world for evaluate_episode."""

    def __init__(self):
        self.world = None
        self.caster = None

    def world_factory(self, config):
        self.world = build_raster_world(config)
        return self.world

    def lidar_factory(self, world, seed):
        self.caster, lidar = make_raster_lidar(world, seed)
        return self.caster, lidar

    def kwargs(self):
        return dict(world_factory=self.world_factory, lidar_factory=self.lidar_factory,
                    clearance_evaluator=raster_clearance)


# --- evaluation (PREREG metrics in the curvilinear frame) ---------------------------------------------------

STOP_ENUM = ('ROW_END', 'END_STRUCTURE', 'OBSTACLE', 'PERCEPTION_OR_LOCALISATION', 'CONTROL_AUTHORITY',
             'OPERATOR_OR_BACKSTOP', 'SYSTEM_FAULT', 'UNCLASSIFIED')
SUB_LABELS = dict(DRIVE='CONTROL_AUTHORITY', SENSOR='PERCEPTION_OR_LOCALISATION')
# Copy of PREREG.json stop_categories.category_table_at_registration (+ prefixes): used only when
# closed_loop.single_row (the table the PREREG names) cannot be imported.
REGISTERED_STOP_TABLE = {
    'row_end': 'ROW_END', 'obstacle_ahead': 'OBSTACLE', 'side_blocked': 'OBSTACLE',
    'end_structure_ahead': 'END_STRUCTURE', 'corridor_not_acquired': 'PERCEPTION_OR_LOCALISATION',
    'corridor_not_reacquired': 'PERCEPTION_OR_LOCALISATION', 'corridor_lost': 'PERCEPTION_OR_LOCALISATION',
    'stop_repeat_limit': 'PERCEPTION_OR_LOCALISATION', 'steering_authority_insufficient': 'CONTROL_AUTHORITY',
    'start_misaligned': 'CONTROL_AUTHORITY', 'reacquire_misaligned': 'CONTROL_AUTHORITY',
    'drive_no_response': 'DRIVE', 'stop_not_observed': 'DRIVE', 'stop_without_decision': 'OPERATOR_OR_BACKSTOP',
    'sensor_missing': 'SENSOR', 'stale_lidar': 'SENSOR', 'stale_imu': 'SENSOR', 'row_end_candidate': 'ROW_END',
    'front_blocked': 'OBSTACLE', 'pivot_clearance_insufficient': 'OBSTACLE', 'pivot_stall_abort': 'DRIVE',
    'turn_timeout': 'DRIVE', 'settle_timeout': 'DRIVE', 'stop_turn_repeat_limit': 'CONTROL_AUTHORITY',
    'turn_not_permitted': 'CONTROL_AUTHORITY', 'row_distance_backstop': 'OPERATOR_OR_BACKSTOP',
    'row_time_backstop': 'OPERATOR_OR_BACKSTOP', 'operator_end': 'OPERATOR_OR_BACKSTOP',
    'hold_released': 'OPERATOR_OR_BACKSTOP', 'backstop_odometry_invalid': 'PERCEPTION_OR_LOCALISATION',
    'corridor_lost_repeat_limit': 'PERCEPTION_OR_LOCALISATION', 'guard_heading': 'CONTROL_AUTHORITY',
    'guard_lateral': 'CONTROL_AUTHORITY', 'drive_no_response_repeat': 'DRIVE', 'stale_repeat_limit': 'SENSOR',
    'stall_guard_repeat_limit': 'SENSOR', 'clock_regression': 'SENSOR'}
REGISTERED_STOP_PREFIXES = (('guard_violation:', 'CONTROL_AUTHORITY'),)
REST_V_M_S, REST_W_RAD_S, REST_HOLD_S = .01, .01, 1.   # PREREG rest / TERMINAL_STOP
MISSED_MARGIN_M = 2.5                                  # MISSED_ROW_END: s_front > s_E + 2.5
SAFETY_BAND_M = .6
DEPARTURE_MARGIN_M = .35                               # CORRIDOR_DEPARTURE: |d| > local spacing / 2 - 0.35
LEFT_FIELD_BEFORE_M, LEFT_FIELD_BEYOND_M = 2., 6.
# Obstacle / wall clearance at rest (PREREG END_STRUCTURE >= 0.30 m, OBSTACLE in [0.30, 3.00] m) is measured with this
# reach: the per-sample contact clearance uses 2 m (nothing farther matters for contact) and reported a wall 3 m ahead of
# a correct row-end stop as "none within reach" (None), which the verdict took for a failed clearance (dev round 1,
# 2026-11-02 evaluator fix; recorded in single-row-20261102/DEV_REPORT_KO.txt).
AT_REST_REACH_M = 6.
NO_PROGRESS_WINDOW_S, NO_PROGRESS_MIN_M = 60., .5
ROW_END_WINDOWS = (('FALSE', -math.inf, -2.), ('EARLY', -2., -1.), ('OK', -1., 1.), ('LATE', 1., 2.5))


def _registered_lookup(reason):
    if reason in REGISTERED_STOP_TABLE:
        return REGISTERED_STOP_TABLE[reason]
    for prefix, category in REGISTERED_STOP_PREFIXES:
        if isinstance(reason, str) and reason.startswith(prefix):
            return category
    return None


def stop_category(reason):
    """(PREREG enum category, raw table label, table source) of a terminal pause/end reason. The table is
    closed_loop.single_row.stop_category (named by the PREREG) when importable, else the registered copy;
    sub-labels DRIVE / SENSOR map to CONTROL_AUTHORITY / PERCEPTION_OR_LOCALISATION, unknown -> UNCLASSIFIED."""
    try:
        from .single_row import stop_category as lookup
        source = 'closed_loop.single_row.stop_category'
    except ImportError:
        lookup, source = _registered_lookup, 'raster_corridor.REGISTERED_STOP_TABLE (PREREG copy)'
    raw = lookup(reason)
    category = SUB_LABELS.get(raw, raw) if raw is not None else 'UNCLASSIFIED'
    return (category if category in STOP_ENUM else 'UNCLASSIFIED'), raw, source


def time_cap_s(s_end, s_start=1.):
    """PREREG TIME_CAP: 120 s + (s_E - s_start + 2.5 m) / 0.075 m/s."""
    return 120. + (s_end - s_start + MISSED_MARGIN_M) / .075


def _stats(values, digits=4):
    a = np.abs(np.asarray(values, dtype=float))
    if not a.size:
        return dict(n=0, rmse=None, p95=None, max=None)
    return dict(n=int(a.size), rmse=round(float(np.sqrt(np.mean(a ** 2))), digits),
                p95=round(float(np.percentile(a, 95)), digits), max=round(float(a.max()), digits))


def _runs(mask):
    """(start, stop) index pairs of the True runs of a boolean array."""
    padded = np.concatenate(([False], np.asarray(mask, bool), [False]))
    edges = np.flatnonzero(np.diff(padded.astype(np.int8)))
    return list(zip(edges[::2], edges[1::2]))


def evaluate_episode(result, world, *, footprint_length_m=1.22, footprint_width_m=.70, s_start=None,
                     duty_caps=None, prereg_termination=True):
    """PREREG metrics of one raster episode from its harness trace (record_trace=True), curvilinear frame.

    Truth only (evaluation): base pose from the trace (s, d, heading error) through world.pose_in_field,
    front bumper = base + L/2 along the heading projected to s, yaw rate from the true yaw. With
    prereg_termination the PREREG episode_termination rules are applied post hoc in sample order (COLLISION,
    MISSED_ROW_END, CORRIDOR_DEPARTURE, LEFT_FIELD, NO_PROGRESS, TIME_CAP; else TERMINAL_STOP at the final
    rest of >= 1 s, else OPEN) and the metrics use the samples up to that end. duty_caps = (D_max, U_max) of
    the frozen config for the PWM saturation rule (None: not computed)."""
    trace = result.get('trace')
    if not trace:
        raise ValueError('evaluate_episode needs record_trace=True')
    s_end = float(world.s_end)
    half = footprint_length_m / 2
    t = np.array([r['t'] for r in trace])
    s = np.array([r['fx'] for r in trace])
    d = np.array([r['fy'] for r in trace])
    e = np.radians(np.array([r['fyaw_deg'] for r in trace]))
    e = np.arctan2(np.sin(e), np.cos(e))
    v = np.array([r['v'] for r in trace])
    left = np.array([r['left'] for r in trace], dtype=float)
    right = np.array([r['right'] for r in trace], dtype=float)
    poses = np.array([world.pose_in_field(si, di, ei) for si, di, ei in zip(s, d, e)])
    yaw = np.unwrap(poses[:, 2])
    w = np.gradient(yaw, t) if len(t) > 1 else np.zeros(len(t))
    front = np.array([world.to_field(x + half * math.cos(a), y + half * math.sin(a), a)[0] for x, y, a in poses])
    clear = {name: np.empty(len(t)) for name in CLEARANCE_GROUPS}
    for i, (x, y, a) in enumerate(poses):
        for name, value in world.clearances(x, y, a, footprint_length_m, footprint_width_m).items():
            clear[name][i] = value
    profile_s = np.asarray(world.config.profile_s_m)
    spacing = (np.interp(s, profile_s, np.asarray(world.config.left_row_d_m))
               - np.interp(s, profile_s, np.asarray(world.config.right_row_d_m)))
    at_rest = (np.abs(v) < REST_V_M_S) & (np.abs(w) < REST_W_RAD_S)
    moving_mask = np.abs(v) > REST_V_M_S
    first_motion = int(np.argmax(moving_mask)) if moving_mask.any() else None
    s0 = float(s[0]) if s_start is None else float(s_start)
    # Terminal rest: the final run of at-rest samples (>= 1 s) after first motion, reaching the trace end.
    rest = None
    runs = _runs(at_rest)
    if runs and first_motion is not None and runs[-1][1] == len(t) and runs[-1][0] > first_motion:
        a, b = runs[-1]
        if t[b - 1] - t[a] >= REST_HOLD_S - 1e-6:
            rest = int(a)
    events = []
    if prereg_termination:
        cap = time_cap_s(s_end, s0)
        for i in range(len(t)):
            if rest is not None and i > rest:
                break
            if clear['hard'][i] <= 0:
                events.append(('COLLISION', i))
            elif front[i] > s_end + MISSED_MARGIN_M:
                events.append(('MISSED_ROW_END', i))
            elif abs(d[i]) > spacing[i] / 2 - DEPARTURE_MARGIN_M:
                events.append(('CORRIDOR_DEPARTURE', i))
            elif s[i] < -LEFT_FIELD_BEFORE_M:
                events.append(('LEFT_FIELD', i))
            elif s[i] > s_end + LEFT_FIELD_BEYOND_M:
                events.append(('MISSED_ROW_END', i))
            elif t[i] > cap:
                events.append(('TIME_CAP', i))
            elif t[i] - t[0] >= NO_PROGRESS_WINDOW_S:
                j = int(np.searchsorted(t, t[i] - NO_PROGRESS_WINDOW_S))
                if s[i] - s[j] < NO_PROGRESS_MIN_M:
                    events.append(('NO_PROGRESS', i))
            if events:
                break
    if events:
        prereg_end, end_index = events[0]
    elif rest is not None:
        prereg_end, end_index = 'TERMINAL_STOP', len(t) - 1
    else:
        prereg_end, end_index = 'OPEN', len(t) - 1
    upto = np.arange(len(t)) <= end_index
    stop_index = rest if prereg_end == 'TERMINAL_STOP' else None
    window = upto.copy()
    if first_motion is not None:
        window &= np.arange(len(t)) >= first_motion
    if stop_index is not None:
        window &= np.arange(len(t)) <= stop_index
    in_row = window & (s >= 0.) & (s <= s_end)
    reason = result.get('final_reason')
    category, raw_label, table_source = stop_category(reason)
    if prereg_end != 'TERMINAL_STOP':
        category = None
    e_stop = float(front[stop_index] - s_end) if stop_index is not None else None
    outcome = None
    if prereg_end == 'MISSED_ROW_END':
        outcome = 'MISSED'
    elif category == 'ROW_END' and e_stop is not None:
        outcome = next((name for name, lo, hi in ROW_END_WINDOWS
                        if (lo <= e_stop < hi if name != 'OK' else lo <= e_stop <= hi)), None)
        outcome = outcome or ('MISSED' if e_stop > 2.5 else None)
    hard_contact = clear['hard'] <= 0
    any_contact = hard_contact | (clear['branch'] <= 0)
    moving_any = (np.abs(v) > REST_V_M_S) | (np.abs(w) > REST_W_RAD_S)
    unknown = {name: int(np.sum((clear[name] <= 0) & upto)) for name in UNKNOWN_GROUPS}
    rest_slice = slice(stop_index, end_index + 1) if stop_index is not None else None
    duty_neg = bool(np.any((left < 0) | (right < 0)))
    auto_restart = False
    if first_motion is not None:
        idle = at_rest & (left == 0) & (right == 0)
        for a, b in _runs(idle):
            if a < first_motion or t[b - 1] - t[a] < .5 - 1e-6:
                continue
            if np.any((left[b:] != 0) | (right[b:] != 0)):
                auto_restart = True
                break
    pwm = None
    if duty_caps is not None:
        d_max, u_max = duty_caps
        driving = upto & (left > 0) & (right > 0)
        sat = driving & ((np.abs(right - left) >= .95 * d_max) | (np.maximum(left, right) >= u_max - 1))
        dt = float(np.median(np.diff(t))) if len(t) > 1 else .05
        longest = max((b - a for a, b in _runs(sat)), default=0) * dt
        fraction = float(sat.sum() / max(driving.sum(), 1))
        pwm = dict(d_max=d_max, u_max=u_max, driving_ticks=int(driving.sum()), saturated_fraction=round(fraction, 4),
                   longest_saturated_run_s=round(longest, 2),
                   authority_margin_low=bool(prereg_end == 'TERMINAL_STOP' and (fraction > .2 or longest > 3.)))
    pre_stop_ratio = None
    if stop_index is not None:
        zero = (left == 0) & (right == 0)
        k = stop_index
        while k > 0 and zero[k - 1]:
            k -= 1
        cruise = window & moving_mask & (s >= s0 + 5.) & (s <= s[stop_index] - 5.)
        if cruise.any():
            pre_stop_ratio = round(float(abs(v[k]) / max(np.median(np.abs(v[cruise])), 1e-9)), 3)
    span = window if first_motion is not None else upto
    return dict(
        schema='ubts.single_row.raster_evaluation.v2', definitions='single-row-20261102/PREREG.json metrics',
        record=world.config.record, corridor_id=world.config.corridor_id, direction=world.config.direction,
        s_E_m=round(s_end, 3), s_start_m=round(s0, 3), samples=int(len(t)),
        prereg_end=prereg_end, end_t_s=round(float(t[end_index]), 2), harness_end=result.get('end'),
        final_reason=reason, stop_category=category, stop_table_label=raw_label, stop_table_source=table_source,
        lateral_m=_stats(d[in_row]), heading_deg=dict((k, v_) for k, v_ in _stats(np.degrees(e[in_row]), 3).items()
                                                      if k != 'p95'),
        safety_band=dict(max_abs_d_m=round(float(np.abs(d[span]).max()), 4) if span.any() else None,
                         band_m=SAFETY_BAND_M,
                         exceeded=bool(span.any() and np.abs(d[span]).max() > SAFETY_BAND_M)),
        progress=dict(s_rest_m=round(float(s[stop_index]), 3) if stop_index is not None else None,
                      s_final_m=round(float(s[end_index]), 3),
                      progress_m=round(float((s[stop_index] if stop_index is not None else s[end_index]) - s0), 3),
                      fraction_of_row=round(float((s[stop_index] if stop_index is not None else s[end_index]) / s_end), 4)),
        stop_position_e_m=None if e_stop is None else round(e_stop, 3),
        overshoot_m=round(max(0., float(front[upto].max() - s_end)), 3),
        row_end_outcome=outcome,
        false_row_end_detection=bool(category == 'ROW_END' and e_stop is not None and e_stop < -2.),
        missed_row_end=prereg_end == 'MISSED_ROW_END',
        contacts=dict(collision_samples=int(np.sum(hard_contact & upto)),
                      moving_contact_samples=int(np.sum(any_contact & moving_any & upto)),
                      rest_contact_samples=int(np.sum(any_contact & ~moving_any & upto)),
                      branch_contact_samples=int(np.sum((clear['branch'] <= 0) & upto)),
                      unknown_overlap_samples=unknown, unknown_overlap_total=int(sum(unknown.values())),
                      min_hard_clearance_m=round(float(clear['hard'][upto].min()), 3)
                      if np.isfinite(clear['hard'][upto].min()) else None,
                      min_branch_clearance_m=round(float(clear['branch'][upto].min()), 3)
                      if np.isfinite(clear['branch'][upto].min()) else None),
        obstacle_wall_clearance_at_rest_m=_added_hard_at_rest(world, poses, rest_slice, footprint_length_m,
                                                              footprint_width_m),
        pwm_saturation=pwm,
        forbidden=dict(pivot_or_reverse=duty_neg, auto_restart=auto_restart),
        report_only=dict(mean_speed_m_s=round(float(np.mean(v[span])), 3) if span.any() else None,
                         pre_stop_speed_ratio=pre_stop_ratio,
                         first_row_end_estimate=result.get('row_end_first_estimate'),
                         first_motion_t_s=None if first_motion is None else round(float(t[first_motion]), 2),
                         max_front_s_minus_end_m=round(float(front.max() - s_end), 3)),
        world=dict(modifiers=world.modifier_log, walls_near_face_s_m=[round(x, 3) for x in world.walls_s]),
        physical_truth=False)


def _added_hard_at_rest(world, poses, rest_slice, length, width):
    """Smallest footprint clearance to the ADDED_HARD cells (scenario obstacle box / wall) over the rest samples,
    with the AT_REST_REACH_M reach; None without a terminal rest or with nothing added within that reach."""
    if rest_slice is None:
        return None
    values = [world.clearances(x, y, a, length, width, reach=AT_REST_REACH_M)['added_hard']
              for x, y, a in poses[rest_slice]]
    value = min(values) if values else math.inf
    return round(float(value), 3) if math.isfinite(value) else None


def run_raster_episode(config, *, record_trace=True, duty_caps=None, **kwargs):
    """harness.run_episode on a raster world (config.world: RasterCorridorConfig); returns (result, world) with
    result['raster_world'] (world description, LiDAR timing, counters) and, when traced,
    result['raster_evaluation'] (evaluate_episode)."""
    from .harness import plant_params, run_episode
    hooks = RasterHooks()
    started = time.perf_counter()
    result = run_episode(config, record_trace=record_trace, **hooks.kwargs(), **kwargs)
    result['wall_s'] = round(time.perf_counter() - started, 3)
    result['raster_world'] = hooks.world.describe()
    if record_trace:
        params = plant_params(config)
        result['raster_evaluation'] = evaluate_episode(
            result, hooks.world, footprint_length_m=params.footprint_length_m,
            footprint_width_m=params.footprint_width_m, s_start=config.start_x_m, duty_caps=duty_caps)
    return result, hooks.world
