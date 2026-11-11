"""Bounded 2-D inverse-sensor map for the isolated simulator.

This is pose-based dead-reckoning mapping, not SLAM: it deliberately does not use Gazebo
ground truth, camera AI, or real-device APIs. Raster row zero is minimum world Y.
"""
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
from scripts.sim_clock import metadata as clock_metadata, freshness as clock_freshness


ALGORITHM = "sim-source-pose-tiled-logodds-v2"
LEGACY_LOCALIZATION = "wheel_odometry_without_loop_closure"
LEGACY_POSE_SOURCE = "wheel_odometry"


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


from dashboard_server.tile_transport import StreamingOccupancyMap


class OccupancyMap(StreamingOccupancyMap):
    """Retained tiled raster using the original SIM sample pose provenance."""
    def __init__(self, **config):
        super().__init__(**config)
        self.localization=LEGACY_LOCALIZATION;self.pose_source=LEGACY_POSE_SOURCE

    def update(self,sample):
        return super().update(dict(sample,
            localization=sample.get('localization') or LEGACY_LOCALIZATION,
            pose_source=sample.get('pose_source') or LEGACY_POSE_SOURCE))

    def snapshot(self):
        return dict(super().snapshot(),algorithm=ALGORITHM)


class SimulationMapping:
    """One worker for the simulator's latest-sample mailbox and archive sidecars."""
    def __init__(self, sample_path="/runtime/mapping_sample.json", state_dir="/run/ubts-sim/mapping",
                 archive_dir="/runtime/recordings", start_worker=True):
        self.sample_path, self.state_dir, self.archive_dir = map(Path, (sample_path, state_dir, archive_dir))
        self.lock = threading.RLock()
        self._planning_write_lock = threading.Lock()
        self.map = OccupancyMap()
        self.map_epoch = uuid.uuid4().hex
        self.localization_epoch = None
        self.active = False
        self.state = "idle"
        self.version = 0
        self.last_seq = None
        self.last_stamp = None
        self.last_seen = 0.0
        self.planning_source_clock = {}
        self.last_publish = 0.0
        self.error = None
        self.archive = None
        self._stop = threading.Event()
        self.thread = None
        self._regenerating = set()
        self._snapshot = {}
        self.persistence = {}
        self._publish_snapshot()
        self._restore_saved_map()
        if start_worker:
            self.thread = threading.Thread(target=self._run, daemon=True, name="sim-occupancy-map")
            self.thread.start()

    def close(self):
        self._stop.set()
        if self.thread:
            self.thread.join(timeout=3)
        with self.lock:
            if self.archive:
                self.finish_archive()
            if self.map.scan_count:
                self.active=False
                if self.state!='reset_detected':self.state='paused'
                self._save_native_map()

    def _run(self):
        while not self._stop.wait(.2):
            try:
                sample = read_json(self.sample_path)
                if sample and clock_freshness(sample, 2.)['fresh']:
                    self.ingest(sample)
                if time.monotonic() - self.last_publish >= .5:
                    self._publish_planning()
            except (OSError, ValueError, TypeError) as exc:
                with self.lock:
                    self.error = str(exc)

    def _age(self):
        return max(0, time.monotonic() - self.last_seen) if self.last_seen else None

    def _publish_snapshot(self):
        # Called by the single mapper writer while holding self.lock. Nested
        # snapshot values are replaced, never mutated after publication. HTTP
        # readers therefore get one completed generation without waiting for
        # ray integration, archive compression, PNG rendering, or disk writes.
        self._snapshot = {"ok": True, "simulated": True, "active": self.active,
                          "state": self.state, "version": self.version,
                          "map_epoch": self.map_epoch,
                          "localization_epoch": self.localization_epoch,
                          **self.map.snapshot()}
        if self.persistence:self._snapshot['persistence']=dict(self.persistence)
        # Immutable numeric view of the same accepted generation. HTTP readers
        # select changed blocks without acquiring the integration writer lock.
        packet=self.map.tile_packet()
        packet['tiles']=[dict(t,revision=self.map.tile_versions.get((t['x'],t['y']),0)) for t in packet['tiles']]
        self._cell_snapshot={**self._snapshot,**packet,'tile_epoch':self.map_epoch}

    def _save_native_map(self):
        try:
            proof=self.map.save_checkpoint(self.state_dir/'last_native_map.json')
            self.persistence={'saved':True,'native_cells':True,**proof}
            self._publish_snapshot()
            atomic_json(self.state_dir/'last_map.json',self._snapshot)
        except (OSError,ValueError,TypeError) as exc:
            # Persistence must not prevent an already requested stop/finalize.
            self.persistence={'saved':False,'error':str(exc)[:160]}
            self._publish_snapshot()

    def _restore_saved_map(self):
        saved=read_json(self.state_dir/'last_map.json')
        if not saved:return
        try:
            from dashboard_server.tiled_checkpoint import digest
            epoch=saved.get('map_epoch','')
            if not re.fullmatch(r'[0-9a-f]{32}',epoch):raise ValueError('invalid_saved_map_epoch')
            path=self.state_dir/'last_native_map.json'
            if digest(path)!=(saved.get('persistence') or {}).get('manifest_sha256'):
                raise ValueError('saved_map_generation_mismatch')
            mapper=OccupancyMap.load_checkpoint(path)
            if mapper.scan_count!=saved.get('scan_count'):raise ValueError('saved_map_generation_mismatch')
            if not mapper.scan_count:return
            version=saved.get('version')
            if type(version) is not int or version<0:raise ValueError('invalid_saved_map_version')
            self.map=mapper;self.map_epoch=epoch;self.version=version
            self.active=False;self.state='reset_detected'
            self.persistence={**saved['persistence'],'restored':True,'requires_new_localization':True}
            self._publish_snapshot()
        except (OSError,ValueError,TypeError,KeyError,EOFError,zlib.error) as exc:
            self.persistence={'restored':False,'error':str(exc)[:160]}
            self._publish_snapshot()

    def _publish_planning(self):
        # Serialize concurrent action/worker writes without holding the mapper
        # lock during filesystem I/O. A late worker cannot overwrite a newer
        # start/stop/reset generation with an old active flag.
        with self._planning_write_lock:
            with self.lock:
                payload = {**self.map.planning(), **clock_metadata(), "active": self.active,
                           "version": self.version, "updated_at": time.time(),
                           "scan_age_s": self._age(), "state": self.state,
                           "map_epoch": self.map_epoch, "localization_epoch": self.localization_epoch,
                           "source_scan_clock": dict(self.planning_source_clock)}
            atomic_json(self.state_dir / "planning_map.json", payload)
            self.last_publish = time.monotonic()

    def ingest(self, sample):
        with self.lock:
            if self.state == "reset_detected":
                return False
            epoch = (sample.get("session_id"), sample.get("localization_epoch"),
                     sample.get("pose_source", LEGACY_POSE_SOURCE),
                     sample.get("localization", LEGACY_LOCALIZATION))
            if (sample.get("localization_valid") is False or
                    (self.localization_epoch is not None and epoch != self.localization_epoch)):
                self._freeze_discontinuity()
                return False
            seq = (sample.get("session_id"), sample.get("seq"), sample.get("stamp"))
            if seq == self.last_seq:
                return False
            try:
                stamp = float(sample["stamp"])
            except (KeyError, TypeError, ValueError):
                return False
            if not math.isfinite(stamp):
                return False
            if self.last_stamp is not None and stamp < self.last_stamp - .05:
                # A new simulation coordinate epoch must not corrupt the old map.
                self._freeze_discontinuity()
                return False
            self.localization_epoch = epoch
            self.last_seq, self.last_stamp, self.last_seen = seq, stamp, time.monotonic()
            self.map.observe_pose(sample)
            if self.active and self.map.update(sample):
                # Preserve the sensor producer's receipt clock. A periodic
                # planning write or UI action must not renew an old scan.
                self.planning_source_clock = {key: sample.get(key) for key in
                    ('clock_schema', 'clock_source', 'clock_domain', 'updated_monotonic_raw')}
                self.version += 1
                self.state = "mapping"
            if self.archive:
                item = self.archive
                recorded = {**sample, "recording_t_s": max(0.0, time.time() - item["started_at"])}
                scan = recorded.get("scan") or {}
                ranges = scan.get("ranges") or []
                stride = max(1, math.ceil(len(ranges) / item["map"].max_beams))
                if stride > 1:
                    # The recorder's rosbag retains the full sensor data. This
                    # journal needs only the exact rays integrated by the map.
                    recorded["scan"] = {**scan, "ranges": ranges[::stride],
                                        "angle_increment": scan["angle_increment"] * stride}
                line = json.dumps(recorded, separators=(",", ":"), allow_nan=False)
                if item["bytes"] + len(line) > 512 * 1024 * 1024:
                    item["limit_reached"] = True
                    self.finish_archive()
                elif item["map"].update(recorded):
                    # Preserve the exact samples used by this mapper for replay.
                    item["bytes"] += len(line)
                    item["file"].write(line + "\n")
                    item["file"].flush()
            self._publish_snapshot()
            return True

    def _freeze_discontinuity(self):
        # Keep pixels AND the last pose/path in their original frame. A fresh
        # timestamp cannot establish a transform between different epochs.
        self.active, self.state = False, "reset_detected"
        if self.archive:
            self.archive["discontinuity"] = True
            self.finish_archive()
        self.version += 1
        self._publish_snapshot()

    def snapshot(self, since=None, transport=None, tile_since=None, tile_epoch=None):
        if transport=='tiles':
            data=self._cell_snapshot
            try:revision=int(tile_since) if tile_epoch==data['tile_epoch'] else -1
            except (TypeError,ValueError):revision=-1
            full=not 0<=revision<=data['tile_version']
            return {**data,'tiles':data['tiles'] if full else [t for t in data['tiles'] if t['revision']>revision],
                    'tiles_reset':full,'scan_age_s':self._age(),'error':self.error}
        data = {**self._snapshot, "scan_age_s": self._age(), "error": self.error}
        # The UI can skip repeating a PNG when only the current pose changed.
        if str(since) == str(data["version"]):
            data["grid"] = {k: v for k, v in data["grid"].items() if k != "image_base64"}
            data["unchanged"] = True
        return data

    def action(self, action, transport=None):
        with self.lock:
            if action == "start":
                if self.state == "reset_detected":
                    return {"ok": False, "error": "reset_map_after_simulator_reset", "state": self.state}
                self.active, self.state = True, "mapping"
            elif action == "stop":
                self.active = False
                if self.state != "reset_detected":
                    self.state = "paused"
            elif action == "reset":
                self.map = OccupancyMap()
                self.map_epoch = uuid.uuid4().hex
                self.localization_epoch = None
                self.last_seq = self.last_stamp = None
                self.last_seen = 0.0
                self.planning_source_clock = {}
                self.active, self.state = False, "idle"
                self.persistence={}
            else:
                return {"ok": False, "error": "unknown_mapping_action"}
            self.version += 1
            self._publish_snapshot()
            if action in ('stop','reset'):self._save_native_map()
        # Publish the action before responding; autonomy need not wait for the
        # next 5Hz scan or 2Hz planning tick. scan_age_s remains truthful.
        self._publish_planning()
        return self.snapshot(transport=transport)

    def _archive_path(self, recording_id):
        if not isinstance(recording_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", recording_id):
            raise ValueError("invalid_recording_id")
        path = (self.archive_dir / recording_id).resolve()
        if path.parent != self.archive_dir.resolve() or not path.is_dir():
            raise ValueError("recording_not_found")
        return path

    def begin_archive(self, recording_id):
        with self.lock:
            if self.archive:
                self.finish_archive()
            path = self._archive_path(recording_id)
            if (path / "mapping_samples.jsonl.gz").exists():
                return {"ok": False, "error": "mapping_archive_already_exists"}
            atomic_json(path / "mapping_config.json", {"algorithm": ALGORITHM, **OccupancyMap().config()})
            self.archive = {"id": recording_id, "path": path, "map": OccupancyMap(), "bytes": 0,
                            "started_at": time.time(), "start_stamp": self.last_stamp,
                            "file": gzip.open(path / "mapping_samples.jsonl.gz", "wt", encoding="utf-8", compresslevel=1)}
            return {"ok": True, "recording_id": recording_id}

    def finish_archive(self):
        with self.lock:
            if not self.archive:
                return {"ok": True, "state": "idle"}
            item, self.archive = self.archive, None
            item["file"].close()
            result = {"ok": True, "simulated": True, "recording_id": item["id"],
                      "active": False, "state": "archived", **item["map"].snapshot(),
                      "recording_start_stamp": item["start_stamp"], "recording_time_base": "wall_elapsed_seconds",
                      "reproducible": True, "sample_limit_reached": item.get("limit_reached", False),
                      "simulator_reset": item.get("discontinuity", False)}
            result['native_tiles']=self._archive_tiles(item['map'],item['path'])
            atomic_json(item["path"] / "occupancy_grid.json", result)
            return result

    def _archive_tiles(self,mapper,path):
        try:
            return {'saved':True,'native_cells':True,**mapper.save_checkpoint(path/'occupancy_tiles.json')}
        except (OSError,ValueError,TypeError) as exc:
            return {'saved':False,'error':str(exc)[:160]}

    def archive_snapshot(self, recording_id):
        try:
            path = self._archive_path(recording_id)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        with self.lock:
            if self.archive and self.archive["id"] == recording_id:
                return {"ok": True, "active": True, "state": "recording", **self.archive["map"].snapshot()}
            if recording_id in self._regenerating:
                return {**read_json(path / "occupancy_grid.json"), "ok": True, "state": "regenerating"}
        result = read_json(path / "occupancy_grid.json") or {"ok": False, "error": "grid_not_archived"}
        error = read_json(path / "mapping_error.json")
        if error:
            result["regeneration_error"] = error.get("error")
        return result

    def regenerate(self, recording_id, background=True):
        try:
            path = self._archive_path(recording_id)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        with self.lock:
            if self.archive and self.archive["id"] == recording_id:
                return {"ok": False, "error": "recording_still_active"}
            if recording_id in self._regenerating:
                return {"ok": True, "state": "regenerating"}
            if not (path / "mapping_samples.jsonl.gz").is_file():
                return {"ok": False, "error": "mapping_samples_not_archived"}
            self._regenerating.add(recording_id)
        if background:
            threading.Thread(target=self._regenerate, args=(recording_id, path), daemon=True,
                             name="sim-map-regenerate").start()
            return {"ok": True, "state": "regenerating", "recording_id": recording_id}
        return self._regenerate(recording_id, path)

    def _regenerate(self, recording_id, path):
        try:
            config = read_json(path / "mapping_config.json")
            config.pop("algorithm", None)
            mapper = OccupancyMap(**config)
            previous = read_json(path / "occupancy_grid.json")
            preserved = {key: previous[key] for key in ("recording_start_stamp", "recording_time_base",
                         "sample_limit_reached", "simulator_reset") if key in previous}
            consumed = 0
            with gzip.open(path / "mapping_samples.jsonl.gz", "rt", encoding="utf-8") as source:
                for line in source:
                    consumed += len(line)
                    if consumed > 512 * 1024 * 1024:
                        raise ValueError("mapping_archive_size_limit")
                    mapper.update(json.loads(line))
            result = {**preserved, "ok": True, "simulated": True, "recording_id": recording_id,
                      "active": False, "state": "archived", "reproducible": True, **mapper.snapshot()}
            result['native_tiles']=self._archive_tiles(mapper,path)
            atomic_json(path / "occupancy_grid.json", result)
            (path / "mapping_error.json").unlink(missing_ok=True)
            return result
        except (OSError, ValueError, TypeError, EOFError) as exc:
            result = {"ok": False, "state": "regeneration_failed", "error": str(exc)}
            atomic_json(path / "mapping_error.json", result)
            return result
        finally:
            with self.lock:
                self._regenerating.discard(recording_id)
