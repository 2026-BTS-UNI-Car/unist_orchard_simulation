"""Coordinate/cell transport; the browser owns rasterization, never the server.

Each changed tile is an idempotent full replacement. Skipped HTTP polls need no
patch chain: ``since`` selects every tile modified after the client's revision.
Native log odds remain lossless in TiledOccupancyMap and its checkpoints.
"""
import base64
from collections import OrderedDict
import json
import numpy as np
from dashboard_server.sensor_mapping.tiled_occupancy import TiledOccupancyMap

# Unsigned byte index: 0..8 uncertain, 9..126 occupied, 127 unknown,
# 128..252 represent signed -128..-4 free, 253..255 uncertain.
_CELL_STATES_LUT = np.array([3]*9 + [2]*118 + [0] + [1]*125 + [3]*3, dtype=np.uint8)
_CELL_STATES_LUT.setflags(write=False)


def encode_cells(states):
    values = np.asarray(states, dtype=np.uint8).ravel()
    if len(values) != 4096 or np.any(values > 3):
        raise ValueError('invalid_tile_states')
    edges = np.r_[0, np.flatnonzero(values[1:] != values[:-1])+1, len(values)]
    counts = np.diff(edges)
    # JSON of [state,count,...] has 2 brackets, 2*n-1 commas, n one-digit
    # states and the decimal digits of n counts. Counts are in [1,4096].
    # This is exactly the old compact JSON length, with no list/string build.
    run_count = len(counts)
    run_bytes = 4*run_count+1
    if run_bytes <= 1370:
        run_bytes += int(np.count_nonzero(counts >= 10))
        run_bytes += int(np.count_nonzero(counts >= 100))
        run_bytes += int(np.count_nonzero(counts >= 1000))
    # 4096 two-bit states always pack into 1024 bytes, 1368 base64 characters.
    # The existing tie rule includes two JSON quotes for that base64 string.
    if run_bytes <= 1370:
        runs = np.column_stack((values[edges[:-1]], counts)).ravel().tolist()
        return dict(encoding='rle2-v1', cells=runs)
    groups = values.reshape(-1, 4)
    packed = (groups[:, 0] | (groups[:, 1] << 2) | (groups[:, 2] << 4) | (groups[:, 3] << 6))
    data = base64.b64encode(packed.tobytes()).decode('ascii')
    return dict(encoding='packed2-v1', cells=data)


class StreamingOccupancyMap(TiledOccupancyMap):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.tile_versions = {}
        self.tile_payloads = {}
        self.tile_classes = {}
        self.object_bounds = {}
        self.dirty_bounds = set()
        self.scan_stamps = OrderedDict()
        self.path_compacted_at = 0

    def _update_raster(self, coordinates, x0, y0, delta):
        super()._update_raster(coordinates, x0, y0, delta)
        if len(coordinates):
            for key in self._last_updated_tiles:
                key = tuple(int(v) for v in key)
                if key in self._tiles:
                    self.dirty_bounds.add(key)

    @staticmethod
    def cell_states(tile):
        if type(tile) is np.ndarray and tile.dtype == np.dtype(np.int8):
            states = _CELL_STATES_LUT[tile.view(np.uint8)]
            # Scalar indexing returns a NumPy scalar; other unusual layouts can
            # preserve Fortran order. Keep the original base ndarray/C output.
            if tile.ndim == 0 or not states.flags.c_contiguous:
                return np.array(states, dtype=np.uint8, order='C', copy=True)
            return states
        states = np.full(tile.shape, 3, dtype=np.uint8)
        states[tile == 127] = 0
        states[tile <= -4] = 1
        states[(tile != 127) & (tile >= 9)] = 2
        return states

    def update(self, sample):
        previous_path = len(self.path)
        result = super().update(sample)
        if len(self.path) < previous_path:
            self.path_compacted_at = self.scan_count
        self.scan_stamps[self.scan_count] = self.stamp
        if len(self.scan_stamps) > 512:
            self.scan_stamps.popitem(last=False)
        for key in self.dirty_bounds:
            tile = self._tiles[key]
            states = self.cell_states(tile)
            if key in self.tile_classes and np.array_equal(self.tile_classes[key], states):
                continue
            self.tile_classes[key] = states
            self.tile_versions[key] = self.scan_count
            self.tile_payloads.pop(key, None)
            ys, xs = np.nonzero(states == 2)
            if len(xs):
                self.object_bounds[key] = (key[0]*64+int(xs.min()), key[1]*64+int(ys.min()),
                                          key[0]*64+int(xs.max())+1, key[1]*64+int(ys.max())+1)
            else:
                self.object_bounds.pop(key, None)
        self.dirty_bounds.clear()
        return result

    def tile_packet(self, since=None):
        if since is not None and (since < 0 or since > self.scan_count):
            since = None
        bounds = self.bounds or (0, 0, 0, 0)
        objects = list(self.object_bounds.values())
        occupied = None if not objects else dict(zip(('minX','minY','maxX','maxY'),
            (min(b[0] for b in objects)*self.resolution, min(b[1] for b in objects)*self.resolution,
             max(b[2] for b in objects)*self.resolution, max(b[3] for b in objects)*self.resolution)))
        tiles = []
        for key, tile in self._tiles.items():
            if since is not None and self.tile_versions.get(key, 0) <= since:
                continue
            if key not in self.tile_payloads:
                states = self.tile_classes.get(key)
                if states is None:
                    states = self.cell_states(tile)
                self.tile_payloads[key] = encode_cells(states)
            tiles.append(dict(x=key[0], y=key[1], **self.tile_payloads[key]))
        append_path = since is not None and since in self.scan_stamps and since >= self.path_compacted_at
        path = [p for p in self.path if p['stamp'] > self.scan_stamps[since]] if append_path else list(self.path)
        return dict(scan_count=self.scan_count, cell_count=self._cell_count, limited=self.limited,
                    storage='tiled_logodds_v1', retained_tiles=len(self._tiles),
                    stamp=self.stamp, pose=self.pose, path=path, path_append=append_path,
                    tiles=tiles, tile_size=64, tiles_reset=since is None,
                    tile_version=self.scan_count,
                    grid=dict(width=bounds[2]-bounds[0]+1,height=bounds[3]-bounds[1]+1,
                              resolution=self.resolution,source_resolution=self.resolution,
                              origin=dict(x=bounds[0]*self.resolution,y=bounds[1]*self.resolution),
                              occupied_bounds=occupied,encoding='tiles-cells-v1',row_order='min_y_first'))
