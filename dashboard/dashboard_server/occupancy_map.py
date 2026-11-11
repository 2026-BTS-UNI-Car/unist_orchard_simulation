"""Bounded display raster copied from the SIM; no planning/control authority."""
from __future__ import annotations

import base64
import gzip
import json
import math
import os
from pathlib import Path
import re
import struct
import threading
import time
import zlib
import uuid

import numpy as np


ALGORITHM = "lidar-imu-display-logodds-v1"
LEGACY_LOCALIZATION = "unqualified_lidar_imu_display_without_loop_closure"
LEGACY_POSE_SOURCE = "unqualified_lidar_imu_display"


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, separators=(",", ":"), allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def read_json(path):
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def line_cells(x0, y0, x1, y1):
    """Integer Bresenham, including both endpoints."""
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


def raster_cells(endpoints):
    """Exact Bresenham cells for rays from (0,0), in deterministic x/y order.

    Array arithmetic avoids building/sorting tens of thousands of Python
    tuples per scan. Inclusive half-step rounding matches line_cells in every
    octant. An occupied endpoint wins over all crossing free rays.
    """
    if not endpoints:
        return np.empty((0,2),dtype=np.int64), np.empty((0,2),dtype=np.int64)
    rays = np.asarray(endpoints, dtype=np.int64)
    ends_x, ends_y, hits = rays.T
    dx, dy = np.abs(ends_x), np.abs(ends_y)
    major = np.maximum(dx, dy)
    n = np.arange(int(major.max()) + 1, dtype=np.int64)[None, :]
    x_major = (dx >= dy)[:, None]
    xx = np.sign(ends_x)[:, None] * np.where(
        x_major, n, (2*n*dx[:, None] + dy[:, None]) // (2*np.maximum(dy[:, None], 1)))
    yy = np.sign(ends_y)[:, None] * np.where(
        x_major, (2*n*dy[:, None] + dx[:, None]) // (2*np.maximum(dx[:, None], 1)), n)
    allowed = n <= (major - hits)[:, None]
    xmin, ymin = min(0, int(ends_x.min())), min(0, int(ends_y.min()))
    height = max(0, int(ends_y.max())) - ymin + 1
    free = np.unique((xx[allowed]-xmin)*height + (yy[allowed]-ymin))
    occupied = np.unique((ends_x[hits != 0]-xmin)*height + ends_y[hits != 0]-ymin)
    free = np.setdiff1d(free, occupied, assume_unique=True)
    def coordinates(codes):
        return np.column_stack((codes//height+xmin, codes % height+ymin))
    return coordinates(free), coordinates(occupied)


def grayscale_png(width, height, data):
    def chunk(kind, value):
        return struct.pack(">I", len(value)) + kind + value + struct.pack(">I", zlib.crc32(kind + value) & 0xffffffff)
    rows = b"".join(b"\0" + data[y * width:(y + 1) * width] for y in range(height))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(rows, 3)) + chunk(b"IEND", b""))


class OccupancyMap:
    def __init__(self, resolution=0.10, max_cells=1000000, max_span=1024,
                 max_beams=360, laser_x=0.18, laser_y=0.0):
        self.resolution = resolution
        self.max_cells, self.max_span, self.max_beams = max_cells, max_span, max_beams
        self.laser_x, self.laser_y = laser_x, laser_y
        self._dense = None
        self._cell_count = 0
        self._cells_cache = None
        self.bounds = None
        self.path = []
        self.pose = None
        self.stamp = 0.0
        self.scan_count = 0
        # Missing provenance identifies historical encoder-only journals, not
        # the current Adapter. Never infer IMU fusion from an attached IMU row.
        self.localization = LEGACY_LOCALIZATION
        self.pose_source = LEGACY_POSE_SOURCE
        self.limited = False
        self._grid_cache = None
        self._raster_cache = None

    def config(self):
        return {name: getattr(self, name) for name in
                ("resolution", "max_cells", "max_span", "max_beams", "laser_x", "laser_y")}

    @property
    def cells(self):
        """Materialized diagnostic view; live mapping uses the bounded array."""
        if self._cells_cache is None:
            if self.bounds is None:
                self._cells_cache = {}
            else:
                xmin,ymin,xmax,ymax = self.bounds
                grid = self._dense[:ymax-ymin+1,:xmax-xmin+1]
                ys,xs = np.nonzero(grid != 127)
                self._cells_cache = {(int(x)+xmin,int(y)+ymin):int(grid[y,x]) for x,y in zip(xs,ys)}
        return self._cells_cache

    def _ensure_bounds(self, bounds):
        old = self.bounds
        if self._dense is None:
            self._dense = np.full((self.max_span,self.max_span),127,dtype=np.int8)
        elif bounds[0] != old[0] or bounds[1] != old[1]:
            shifted = np.full_like(self._dense,127)
            ox,oy = old[0]-bounds[0],old[1]-bounds[1]
            width,height = old[2]-old[0]+1,old[3]-old[1]+1
            shifted[oy:oy+height,ox:ox+width] = self._dense[:height,:width]
            self._dense = shifted
        self.bounds = bounds

    def _update(self, key, delta):
        x,y = key
        bounds = self.bounds
        if bounds and bounds[0] <= x <= bounds[2] and bounds[1] <= y <= bounds[3]:
            previous = int(self._dense[y-bounds[1],x-bounds[0]])
        else:
            previous = 127
        if previous == 127:
            if self._cell_count >= self.max_cells:
                self.limited = True
                return
            if bounds:
                xmin, ymin, xmax, ymax = bounds
                bounds = (min(xmin, x), min(ymin, y), max(xmax, x), max(ymax, y))
                if bounds[2] - bounds[0] + 1 > self.max_span or bounds[3] - bounds[1] + 1 > self.max_span:
                    self.limited = True
                    return
            else:
                bounds = (x,y,x,y)
            self._ensure_bounds(bounds)
            previous = 0
            self._cell_count += 1
        self._dense[y-self.bounds[1],x-self.bounds[0]] = max(-40,min(60,previous+delta))

    def _update_raster(self, coordinates, x0, y0, delta):
        if len(coordinates) == 0:
            return
        xs,ys = coordinates[:,0]+x0,coordinates[:,1]+y0
        bounds = (int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max()))
        if self.bounds:
            old = self.bounds
            bounds = (min(old[0],bounds[0]),min(old[1],bounds[1]),max(old[2],bounds[2]),max(old[3],bounds[3]))
        if bounds[2]-bounds[0]+1 > self.max_span or bounds[3]-bounds[1]+1 > self.max_span:
            # At the spatial boundary retain the original deterministic,
            # lexicographic admission policy rather than dropping a full ray.
            for x,y in zip(xs.tolist(),ys.tolist()):
                self._update((x,y),delta)
            return
        # At the cell cap, admit new cells in the same lexicographic order as
        # the sparse implementation and continue updating all existing cells.
        if self._cell_count+len(coordinates) > self.max_cells:
            existing = np.zeros(len(xs),dtype=bool)
            if self.bounds is not None:
                old = self.bounds
                inside = (xs>=old[0]) & (xs<=old[2]) & (ys>=old[1]) & (ys<=old[3])
                existing[inside] = self._dense[ys[inside]-old[1],xs[inside]-old[0]] != 127
            accepted = existing | (np.cumsum(~existing) <= self.max_cells-self._cell_count)
            if not accepted.all():
                self.limited = True
                xs,ys = xs[accepted],ys[accepted]
                if not len(xs):
                    return
                bounds = (int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max()))
                if self.bounds:
                    old = self.bounds
                    bounds = (min(old[0],bounds[0]),min(old[1],bounds[1]),max(old[2],bounds[2]),max(old[3],bounds[3]))
        self._ensure_bounds(bounds)
        ix,iy = xs-bounds[0],ys-bounds[1]
        values = self._dense[iy,ix].astype(np.int16)
        new = values == 127
        self._cell_count += int(new.sum())
        values[new] = 0
        self._dense[iy,ix] = np.clip(values+delta,-40,60)

    def observe_pose(self, sample):
        pose = sample.get("pose") or {}
        try:
            values = [float(pose[k]) for k in ("x", "y", "yaw")]
            stamp = float(sample["stamp"])
        except (TypeError, ValueError, KeyError):
            return False
        if not all(math.isfinite(v) for v in values + [stamp]):
            return False
        self.pose = dict(zip(("x", "y", "yaw"), values), stamp=stamp)
        if isinstance(sample.get("recording_t_s"), (int, float)) and math.isfinite(sample["recording_t_s"]):
            self.pose["t_s"] = sample["recording_t_s"]
        return True

    def update(self, sample):
        if not self.observe_pose(sample):
            return False
        scan = sample.get("scan") or {}
        ranges = scan.get("ranges")
        if not isinstance(ranges, list) or not ranges or len(ranges) > 20000:
            return False
        try:
            amin, step, rmin, rmax = [float(scan[k]) for k in
                                     ("angle_min", "angle_increment", "range_min", "range_max")]
        except (KeyError, TypeError, ValueError):
            return False
        if not all(math.isfinite(v) for v in (amin, step, rmin, rmax)) or not 0 <= rmin < rmax <= 100:
            return False
        yaw = self.pose["yaw"]
        sx = self.pose["x"] + self.laser_x * math.cos(yaw) - self.laser_y * math.sin(yaw)
        sy = self.pose["y"] + self.laser_x * math.sin(yaw) + self.laser_y * math.cos(yaw)
        x0, y0 = math.floor(sx / self.resolution), math.floor(sy / self.resolution)
        stride = max(1, math.ceil(len(ranges) / self.max_beams))
        endpoints = []
        for i in range(0, len(ranges), stride):
            value = ranges[i]
            # Null is the publisher's explicit no-return (+infinity), not a hit.
            if value is None:
                distance, hit = rmax, False
            elif isinstance(value, (int, float)) and math.isfinite(value) and value >= rmin:
                distance, hit = min(value, rmax), value < rmax
            else:
                continue
            angles = scan.get('angles')
            angle = yaw + (float(angles[i]) if isinstance(angles, list) and len(angles)==len(ranges) else amin+i*step)
            if not math.isfinite(angle):
                continue
            x1 = math.floor((sx + distance * math.cos(angle)) / self.resolution)
            y1 = math.floor((sy + distance * math.sin(angle)) / self.resolution)
            endpoints.append((x1-x0, y1-y0, int(hit)))
        # A hit in this scan wins over a crossing free ray. Update once per scan,
        # avoiding near-sensor confidence depending on the number of laser rays.
        if self._raster_cache is not None and endpoints == self._raster_cache[0]:
            free, occupied = self._raster_cache[1:]
        else:
            free, occupied = raster_cells(endpoints)
            self._raster_cache = (endpoints, free, occupied)
        self._update_raster(free, x0, y0, -4)
        self._update_raster(occupied, x0, y0, 9)
        self._cells_cache = None
        self.stamp = self.pose["stamp"]
        self.scan_count += 1
        localization, pose_source = sample.get("localization"), sample.get("pose_source")
        self.localization = localization if isinstance(localization, str) and localization else LEGACY_LOCALIZATION
        self.pose_source = pose_source if isinstance(pose_source, str) and pose_source else LEGACY_POSE_SOURCE
        if not self.path or math.hypot(self.pose["x"] - self.path[-1]["x"], self.pose["y"] - self.path[-1]["y"]) >= .04 or abs(self.pose["yaw"] - self.path[-1]["yaw"]) >= .04:
            self.path.append(dict(self.pose))
            # Retain the entire time span and the newest point, while bounding RAM.
            if len(self.path) > 12000:
                self.path = self.path[::2] + self.path[-1:]
        self._grid_cache = None
        return True

    def grid(self):
        if self._grid_cache is not None:
            return self._grid_cache
        xmin, ymin, xmax, ymax = self.bounds or (0, 0, 0, 0)
        width, height = xmax - xmin + 1, ymax - ymin + 1
        odds = self._dense[:height,:width] if self._dense is not None else np.full((1,1),127,dtype=np.int8)
        pixels = np.full((height,width),127,dtype=np.uint8)
        occupied = (odds != 127) & (odds >= 9)
        pixels[occupied] = 25
        pixels[odds <= -4] = 245
        # Publish object bounds with the PNG, once per cached generation. Free
        # rays and unknown padding can enlarge the raster without enlarging the
        # useful display area. Bounds describe outer cell edges in world metres
        # and deliberately use the very same occupied mask as the dark pixels.
        columns = np.flatnonzero(occupied.any(axis=0))
        rows = np.flatnonzero(occupied.any(axis=1))
        occupied_bounds = None
        if len(columns):
            occupied_bounds = {
                "minX": (xmin + int(columns[0])) * self.resolution,
                "minY": (ymin + int(rows[0])) * self.resolution,
                "maxX": (xmin + int(columns[-1]) + 1) * self.resolution,
                "maxY": (ymin + int(rows[-1]) + 1) * self.resolution,
            }
        pixels = pixels.tobytes()
        self._grid_cache = dict(width=width, height=height, resolution=self.resolution,
                               origin={"x": xmin * self.resolution, "y": ymin * self.resolution},
                               occupied_bounds=occupied_bounds,
                               encoding="png", row_order="min_y_first", image_base64=base64.b64encode(grayscale_png(width, height, pixels)).decode())
        return self._grid_cache

    def snapshot(self):
        return {"algorithm": ALGORITHM, "coordinate_frame": "odom", "stamp": self.stamp,
                "scan_count": self.scan_count, "limited": self.limited, "cell_count": self._cell_count,
                "grid": self.grid(), "pose": dict(self.pose) if self.pose else None,
                "path": list(self.path), "config": self.config(),
                "localization": self.localization, "pose_source": self.pose_source}

    def planning(self):
        if self.bounds is None:
            occupied,free_count = [],0
        else:
            xmin,ymin,xmax,ymax = self.bounds
            odds = self._dense[:ymax-ymin+1,:xmax-xmin+1]
            ys,xs = np.nonzero((odds != 127) & (odds >= 9))
            occupied = np.column_stack((xs+xmin,ys+ymin)).tolist()
            free_count = int((odds <= -4).sum())
        return {"resolution": self.resolution, "bounds": self.bounds,
                "occupied": occupied, "free_count": free_count,
                "pose": self.pose, "stamp": self.stamp, "coordinate_frame": "odom"}

