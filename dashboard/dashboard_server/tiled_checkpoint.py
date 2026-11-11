"""Lossless, bounded tile checkpoints. No pickle or localization restoration.

The immutable gzip blob is installed before its small atomic manifest. Readers
verify the compressed SHA and every tile before returning a map. A saved map
does not establish the vehicle's position after a process/device restart.
"""
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import uuid

import numpy as np
from dashboard_server.occupancy_map import atomic_json

SCHEMA = 'orchard.tiled_checkpoint.v1'
MAX_TILES = 65536
MAX_COMPRESSED_BYTES = 272 * 1024 * 1024


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError('invalid_tile_checkpoint_integer')
    return value


def finite(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('invalid_tile_checkpoint_number')
    return value


def validate_pose(value):
    if not isinstance(value, dict):
        raise ValueError('invalid_tile_checkpoint_pose')
    for key in ('x', 'y', 'yaw', 'stamp'):
        finite(value.get(key))
    if 't_s' in value:
        finite(value['t_s'])
    return value


def save(grid, path):
    path = Path(path)
    if path.is_symlink():
        raise ValueError('symlink_tile_checkpoint')
    count = len(grid._tiles)
    integer(count, 0, MAX_TILES)
    path.parent.mkdir(parents=True, exist_ok=True)
    name = path.stem + '-' + uuid.uuid4().hex + '.tiles.gz'
    blob = path.with_name(name)
    temporary = blob.with_suffix(blob.suffix + '.partial')
    try:
        with temporary.open('xb') as raw:
            with gzip.GzipFile(filename='',fileobj=raw, mode='wb', compresslevel=3, mtime=0) as stream:
                for (tx, ty), tile in sorted(grid._tiles.items()):
                    if tile.dtype != np.int8 or tile.shape != (64, 64):
                        raise ValueError('invalid_tile_checkpoint_array')
                    stream.write(struct.pack('<qq', tx, ty))
                    stream.write(tile.tobytes(order='C'))
            raw.flush()
            os.fsync(raw.fileno())
        integer(temporary.stat().st_size, 0, MAX_COMPRESSED_BYTES)
        sha=digest(temporary)
        previous={}
        if path.is_file() and path.stat().st_size<=4*1024*1024:
            try:previous=json.loads(path.read_text(encoding='utf-8'))
            except (OSError,ValueError):pass
        old_name=previous.get('blob','') if isinstance(previous,dict) else ''
        if isinstance(old_name,str) and re.fullmatch(re.escape(path.stem)+r'-[0-9a-f]{32}\.tiles\.gz',old_name):
            old_blob=path.with_name(old_name)
            if (previous.get('schema')==SCHEMA and previous.get('sha256')==sha and
                old_blob.is_file() and not old_blob.is_symlink() and
                old_blob.stat().st_size==temporary.stat().st_size and digest(old_blob)==sha):
                blob=old_blob;name=old_name
        if blob.name!=old_name or not blob.exists():temporary.replace(blob)
        value = dict(schema=SCHEMA, blob=name, sha256=sha,
                     bytes=blob.stat().st_size, tile_count=count, tile_size=64,
                     cell_count=grid._cell_count, bounds=grid.bounds,
                     config=grid.config(), path=grid.path, pose=grid.pose,
                     stamp=grid.stamp, scan_count=grid.scan_count,
                     localization=grid.localization, pose_source=grid.pose_source,
                     limited=grid.limited, runtime_authority=False,
                     position_after_restart_known=False)
        atomic_json(path, value)
        return dict(manifest=path.name, manifest_sha256=digest(path),sha256=value['sha256'], bytes=value['bytes'],
                    tile_count=count, cell_count=grid._cell_count)
    finally:
        if temporary.exists():
            temporary.unlink()


def load(cls, path):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size > 4 * 1024 * 1024:
        raise ValueError('invalid_tile_checkpoint_manifest')
    value = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict) or value.get('schema') != SCHEMA or value.get('tile_size') != 64:
        raise ValueError('invalid_tile_checkpoint_schema')
    name = value.get('blob', '')
    if not re.fullmatch(re.escape(path.stem) + r'-[0-9a-f]{32}\.tiles\.gz', name):
        raise ValueError('invalid_tile_checkpoint_blob_path')
    blob = path.with_name(name)
    size = integer(value.get('bytes'), 0, MAX_COMPRESSED_BYTES)
    sha = value.get('sha256', '')
    if not re.fullmatch(r'[0-9a-f]{64}', sha) or blob.is_symlink() or not blob.is_file():
        raise ValueError('invalid_tile_checkpoint_blob')
    if blob.stat().st_size != size or digest(blob) != sha:
        raise ValueError('tile_checkpoint_hash_mismatch')
    count = integer(value.get('tile_count'), 0, MAX_TILES)
    config = value.get('config', {})
    if set(config) != {'resolution', 'max_cells', 'max_span', 'max_beams', 'laser_x', 'laser_y'}:
        raise ValueError('invalid_tile_checkpoint_configuration')
    if not .0001 <= finite(config['resolution']) <= 10:
        raise ValueError('invalid_tile_checkpoint_resolution')
    integer(config['max_cells'], 1, 32000000)
    integer(config['max_span'], 16, 4096)
    integer(config['max_beams'], 1, 20000)
    for key in ('laser_x', 'laser_y'):
        if abs(finite(config[key])) > 100:
            raise ValueError('invalid_tile_checkpoint_offset')
    expected_cells = integer(value.get('cell_count'), 0, config['max_cells'])
    grid = cls(**config)
    bounds = None
    with gzip.open(blob, 'rb') as stream:
        for _ in range(count):
            packed = stream.read(16 + 4096)
            if len(packed) != 16 + 4096:
                raise ValueError('truncated_tile_checkpoint')
            key = struct.unpack('<qq', packed[:16])
            if key in grid._tiles or max(abs(n) for n in key) > 1000000000:
                raise ValueError('invalid_tile_checkpoint_key')
            tile = np.frombuffer(packed[16:], dtype=np.int8).reshape(64, 64).copy()
            known = tile != 127
            if not known.any() or ((tile[known] < -40) | (tile[known] > 60)).any():
                raise ValueError('invalid_tile_checkpoint_logodds')
            cells = int(known.sum())
            grid._cell_count += cells
            if grid._cell_count > expected_cells:
                raise ValueError('invalid_tile_checkpoint_cell_count')
            grid._tiles[key] = tile
            ys, xs = np.nonzero(known)
            b = (int(xs.min()) + key[0] * 64, int(ys.min()) + key[1] * 64,
                 int(xs.max()) + key[0] * 64, int(ys.max()) + key[1] * 64)
            bounds = b if bounds is None else (min(bounds[0], b[0]), min(bounds[1], b[1]),
                                              max(bounds[2], b[2]), max(bounds[3], b[3]))
        if stream.read(1):
            raise ValueError('unexpected_tile_checkpoint_payload')
    if grid._cell_count != expected_cells or value.get('bounds') != (None if bounds is None else list(bounds)):
        raise ValueError('invalid_tile_checkpoint_bounds_or_count')
    grid.bounds = bounds
    rows = value.get('path')
    if not isinstance(rows, list) or len(rows) > 12000:
        raise ValueError('invalid_tile_checkpoint_path')
    grid.path = [validate_pose(row) for row in rows]
    grid.pose = None if value.get('pose') is None else validate_pose(value['pose'])
    grid.stamp = finite(value.get('stamp'))
    grid.scan_count = integer(value.get('scan_count'), 0, 1000000000)
    if type(value.get('limited')) is not bool:
        raise ValueError('invalid_tile_checkpoint_limit_flag')
    grid.limited = value['limited']
    for key in ('localization', 'pose_source'):
        text = value.get(key)
        if not isinstance(text, str) or not 1 <= len(text) <= 200:
            raise ValueError('invalid_tile_checkpoint_provenance')
        setattr(grid, key, text)
    return grid
