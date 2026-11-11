"""Read-only historical displays, separate from live mapping and recordings.

The original snapshot bytes are preserved. A PNG is not a log-odds backup and
is never fed into a mapper, planner, or recording regenerator.
"""
from __future__ import annotations

import base64
import hashlib
import json
import math
from pathlib import Path
import re
import struct


MAX_BYTES = 8 * 1024 * 1024
ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,95}")


def decode_snapshot(raw):
    if len(raw) > MAX_BYTES:
        raise ValueError("history_too_large")
    def invalid(_):
        raise ValueError("history_nonfinite_number")
    value = json.loads(raw, parse_constant=invalid)
    if not isinstance(value, dict) or value.get("ok") is not True:
        raise ValueError("history_invalid_snapshot")
    grid = value.get("grid")
    if not isinstance(grid, dict) or grid.get("encoding") != "png" or grid.get("row_order") != "min_y_first":
        raise ValueError("history_full_png_required")
    width, height = grid.get("width"), grid.get("height")
    if type(width) is not int or type(height) is not int or not (0 < width <= 1024 and 0 < height <= 1024):
        raise ValueError("history_invalid_dimensions")
    resolution = grid.get("resolution")
    if not isinstance(resolution, (int, float)) or not math.isfinite(resolution) or not 0 < resolution <= 10:
        raise ValueError("history_invalid_resolution")
    png = base64.b64decode(grid.get("image_base64", ""), validate=True)
    if len(png) < 33 or png[:16] != b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR" or struct.unpack(">II", png[16:24]) != (width, height):
        raise ValueError("history_png_dimensions_mismatch")
    path = value.get("path", [])
    if not isinstance(path, list) or len(path) > 50000:
        raise ValueError("history_invalid_path")
    for point in [grid.get("origin"), value.get("pose"), *path]:
        if not isinstance(point, dict) or any(type(point.get(k)) not in (int, float) or not math.isfinite(point[k]) for k in ("x", "y")):
            raise ValueError("history_invalid_point")
    return value


class MapHistory:
    def __init__(self, directory):
        self.directory = Path(directory)

    def _path(self, history_id):
        if not isinstance(history_id, str) or not ID.fullmatch(history_id):
            raise ValueError("invalid_history_id")
        path = self.directory / history_id
        if path.is_symlink() or path.resolve().parent != self.directory.resolve():
            raise ValueError("invalid_history_path")
        return path

    @staticmethod
    def _read(path):
        if path.is_symlink():
            raise ValueError("invalid_history_path")
        with path.open("rb") as stream:
            raw = stream.read(MAX_BYTES + 1)
        if len(raw) > MAX_BYTES:
            raise ValueError("history_too_large")
        return raw

    def detail(self, history_id):
        try:
            path = self._path(history_id)
            manifest = json.loads(self._read(path / "manifest.json"))
            raw = self._read(path / "snapshot.json")
            if manifest.get("sha256") != hashlib.sha256(raw).hexdigest():
                raise ValueError("history_hash_mismatch")
            snapshot = decode_snapshot(raw)
            # Original fields remain unchanged on disk. This API is explicitly
            # a frozen display, regardless of the original active flag.
            return {"ok": True, "id": history_id, "historical": True,
                    "continuable": False, "reproducible": False,
                    "captured_at": manifest.get("captured_at"),
                    "sha256": manifest["sha256"],
                    "snapshot": {**snapshot, "active": False, "state": "historical"}}
        except (OSError, ValueError, TypeError, KeyError) as exc:
            return {"ok": False, "error": str(exc)}

    def listing(self):
        items = []
        if self.directory.is_dir():
            for path in sorted(self.directory.iterdir(), reverse=True):
                if not ID.fullmatch(path.name) or len(items) >= 100:
                    continue
                item = self.detail(path.name)
                if item.get("ok"):
                    snapshot = item.pop("snapshot")
                    items.append({**item, "localization": snapshot.get("localization"),
                                  "pose_source": snapshot.get("pose_source"),
                                  "scan_count": snapshot.get("scan_count")})
        return {"ok": True, "items": items}

    def preserve(self, history_id, raw, *, captured_at, source):
        """Offline deployment operation only; no HTTP write route exists."""
        decode_snapshot(raw)
        path = self._path(history_id)
        path.mkdir(parents=True, exist_ok=False)
        manifest = {"sha256": hashlib.sha256(raw).hexdigest(), "captured_at": captured_at,
                    "source": source, "historical": True, "continuable": False,
                    "reproducible": False}
        (path / "snapshot.json").write_bytes(raw)
        (path / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        return self.detail(history_id)
