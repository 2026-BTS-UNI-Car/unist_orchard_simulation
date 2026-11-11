"""Pure-numpy raster helpers of the field worlds (no scipy: runs on Python 3.10 / 3.12 / 3.13).

  Transform2D        raster-local metres L (x right, y up, origin at the record raster's lower-left corner)
                     <-> the world frame W of a field world (origin at the corridor start s = 0, d = 0; +x = the
                     start tangent; +y left). p_L = p0 + R(theta0) p_W.
  label_components   components of cells, 8-connected after a 1-cell dilation (the catalogue / row-analysis rule):
                     two cells belong together when a chain of cells with Chebyshev distance <= 3 joins them
  merged_rectangles  exact disjoint rectangle cover of a boolean mask: runs along a grid row, merged with the run of
                     the same columns in the next rows (the 'simple merged boxes' of design 5.2)
  first_stop         vectorised supercover DDA from one sensor point: per ray the entry distance into the first cell
                     whose kind is a stop kind (reflector, terminator, outside), within a per-ray limit. At an exact
                     corner both side cells count and the larger kind wins (terminator over reflector over pass),
                     the rule of the reference raster world (closed_loop/testdata/raster_world_ref_e71e27d4.py)
  fmt                fixed-decimal number text for SDF / JSON outputs (at most 6 decimals, see coord_lint)
"""
from __future__ import annotations

import math

import numpy as np

# Kind codes of first_stop (ordered: a tie at a corner takes the larger code).
PASS, REFLECT, TERMINATE, OUTSIDE = 0, 1, 2, 3
KIND_NAMES = ('pass', 'reflect', 'terminate', 'outside')


def fmt(value, decimals=5):
    """Number text with a fixed number of decimals, trailing zeros removed ('-0' -> '0')."""
    text = ('%.' + str(int(decimals)) + 'f') % float(value)
    if '.' in text:
        text = text.rstrip('0').rstrip('.')
    return '0' if text in ('-0', '') else text


def rounded(value, decimals=4):
    """JSON-safe copy with every float rounded (numpy scalars to Python, tuples to lists)."""
    if isinstance(value, dict):
        return {str(k): rounded(v, decimals) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [rounded(v, decimals) for v in value]
    if isinstance(value, np.ndarray):
        return rounded(value.tolist(), decimals)
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        value = float(value)
        if not math.isfinite(value):
            return None
        out = round(value, decimals)
        return 0. if out == 0 else out
    return value


class Transform2D:
    """Planar rigid transform between raster-local metres L and a field world frame W.

    x0, y0: raster-local position of the W origin; theta: raster-local direction of W +x (rad)."""

    def __init__(self, x0, y0, theta):
        self.x0, self.y0, self.theta = float(x0), float(y0), float(theta)
        self.c, self.s = math.cos(self.theta), math.sin(self.theta)

    def to_w(self, x, y):
        dx, dy = np.asarray(x, dtype=float) - self.x0, np.asarray(y, dtype=float) - self.y0
        return self.c * dx + self.s * dy, -self.s * dx + self.c * dy

    def to_l(self, x, y):
        x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
        return self.x0 + self.c * x - self.s * y, self.y0 + self.s * x + self.c * y

    def yaw_to_w(self, yaw):
        return wrap(np.asarray(yaw, dtype=float) - self.theta)

    def yaw_to_l(self, yaw):
        return wrap(np.asarray(yaw, dtype=float) + self.theta)

    def dir_to_l(self, dx, dy):
        dx, dy = np.asarray(dx, dtype=float), np.asarray(dy, dtype=float)
        return self.c * dx - self.s * dy, self.s * dx + self.c * dy

    def to_dict(self):
        return dict(raster_local_origin_of_w_m=[round(self.x0, 6), round(self.y0, 6)],
                    raster_local_heading_of_w_x_rad=round(self.theta, 9),
                    rule='p_L = p0 + R(theta) p_W; p_W = R(-theta) (p_L - p0); raster-local metres only')

    @classmethod
    def from_dict(cls, data):
        x0, y0 = data['raster_local_origin_of_w_m']
        return cls(x0, y0, data['raster_local_heading_of_w_x_rad'])


def wrap(angle):
    angle = np.asarray(angle, dtype=float)
    out = np.arctan2(np.sin(angle), np.cos(angle))
    return float(out) if out.ndim == 0 else out


# --- components ---------------------------------------------------------------------------------------------------
def label_components(rows, cols, reach=3):
    """Component index (0..k-1, numbered in the order of the first cell) of each cell (rows[i], cols[i]).

    reach 3 = 8-connected after a 1-cell (3x3) dilation: dilated squares touch or overlap exactly when the cells are
    within Chebyshev distance 3. Hook-and-compress union over the neighbour pairs (numpy only)."""
    rows, cols = np.asarray(rows, dtype=np.int64), np.asarray(cols, dtype=np.int64)
    n = len(rows)
    if n == 0:
        return np.zeros(0, dtype=np.int64)
    width = int(cols.max() - cols.min()) + 2 * reach + 3
    base_c = int(cols.min()) - reach - 1
    keys = rows * width + (cols - base_c)
    order = np.argsort(keys, kind='stable')
    sorted_keys = keys[order]
    if np.any(np.diff(sorted_keys) == 0):
        raise ValueError('label_components: repeated cell')
    pairs_i, pairs_j = [], []
    index = np.arange(n)
    for dr in range(0, reach + 1):
        for dc in range(-reach, reach + 1):
            if dr == 0 and dc <= 0:
                continue
            query = keys + dr * width + dc
            pos = np.searchsorted(sorted_keys, query)
            pos_ok = pos < n
            found = np.zeros(n, dtype=bool)
            found[pos_ok] = sorted_keys[pos[pos_ok]] == query[pos_ok]
            if found.any():
                pairs_i.append(index[found])
                pairs_j.append(order[pos[found]])
    label = np.arange(n)
    if pairs_i:
        a, b = np.concatenate(pairs_i), np.concatenate(pairs_j)
        while True:
            la, lb = label[a], label[b]
            low = np.minimum(la, lb)
            new = label.copy()
            np.minimum.at(new, la, low)
            np.minimum.at(new, lb, low)
            while True:
                nxt = new[new]
                if np.array_equal(nxt, new):
                    break
                new = nxt
            if np.array_equal(new, label):
                break
            label = new
    _, first = np.unique(label, return_index=True)
    roots = label[np.sort(first)]
    remap = np.empty(n, dtype=np.int64)
    remap[roots] = np.arange(len(roots))
    return remap[label]


def component_table(rows, cols, labels, resolution, row0=0, col0=0):
    """Per component: dict(cells, rows, cols, cx, cy (mean cell centre, raster-local m), width_m, height_m,
    diag_m (diagonal of the cell-square bounding box), extent_m (max of width and height))."""
    out = []
    if not len(labels):
        return out
    order = np.argsort(labels, kind='stable')
    bounds = np.flatnonzero(np.diff(labels[order])) + 1
    for group in np.split(order, bounds):
        r, c = rows[group], cols[group]
        width = (int(c.max()) - int(c.min()) + 1) * resolution
        height = (int(r.max()) - int(r.min()) + 1) * resolution
        out.append(dict(index=int(labels[group[0]]), cells=int(len(group)), rows=r, cols=c,
                        cx=float(((c + col0 + .5) * resolution).mean()), cy=float(((r + row0 + .5) * resolution).mean()),
                        width_m=width, height_m=height, diag_m=math.hypot(width, height), extent_m=max(width, height)))
    return out


# --- rectangles -----------------------------------------------------------------------------------------------------
def merged_rectangles(mask):
    """Disjoint rectangles (r0, r1, c0, c1), inclusive, whose union is exactly mask (2D bool). Rows ascending, then
    columns: runs of one grid row merged with an identical run (same c0, c1) of the rows that follow."""
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2:
        raise ValueError('merged_rectangles: 2D mask')
    rects = []
    open_runs = {}                                   # (c0, c1) -> r0 of the open rectangle (last row = r - 1)
    for r in range(mask.shape[0]):
        row = mask[r]
        runs = set()
        if row.any():
            padded = np.concatenate(([False], row, [False]))
            edges = np.flatnonzero(np.diff(padded.astype(np.int8)))
            runs = {(int(a), int(b) - 1) for a, b in zip(edges[::2], edges[1::2])}
        for key in [k for k in open_runs if k not in runs]:
            rects.append((open_runs.pop(key), r - 1, key[0], key[1]))
        for key in runs:
            if key not in open_runs:
                open_runs[key] = r
    last = mask.shape[0] - 1
    for key, r0 in open_runs.items():
        rects.append((r0, last, key[0], key[1]))
    rects.sort()
    return np.asarray(rects, dtype=np.int64).reshape(-1, 4)


def rectangles_mask(rects, shape):
    """Boolean mask of the union of inclusive rectangles (the inverse of merged_rectangles)."""
    out = np.zeros(shape, dtype=bool)
    for r0, r1, c0, c1 in np.asarray(rects, dtype=np.int64).reshape(-1, 4):
        out[r0:r1 + 1, c0:c1 + 1] = True
    return out


# --- polygon fill -----------------------------------------------------------------------------------------------------
def polygon_mask(shape, resolution, row0, col0, polygon):
    """Cells of a grid crop (shape, resolution, offsets) whose centres lie inside a simple polygon (even-odd rule,
    raster-local metres, [(x, y), ...])."""
    poly = np.asarray(polygon, dtype=float).reshape(-1, 2)
    rows, cols = shape
    out = np.zeros(shape, dtype=bool)
    if len(poly) < 3:
        return out
    x0, y0 = poly[:, 0], poly[:, 1]
    x1, y1 = np.roll(x0, -1), np.roll(y0, -1)
    lo_r = max(0, int(math.floor(poly[:, 1].min() / resolution)) - row0)
    hi_r = min(rows, int(math.floor(poly[:, 1].max() / resolution)) - row0 + 1)
    centres_x = (np.arange(cols) + col0 + .5) * resolution
    for r in range(lo_r, hi_r):
        y = (r + row0 + .5) * resolution
        crosses = (y0 <= y) != (y1 <= y)
        if not crosses.any():
            continue
        xa, ya, xb, yb = x0[crosses], y0[crosses], x1[crosses], y1[crosses]
        xs = np.sort(xa + (y - ya) * (xb - xa) / (yb - ya))
        for a, b in zip(xs[::2], xs[1::2]):
            out[r] |= (centres_x >= a) & (centres_x < b)
    return out


# --- supercover DDA -------------------------------------------------------------------------------------------------
def first_stop(kind_grid, resolution, x, y, dx, dy, limit):
    """Supercover DDA of many rays from one point.

    kind_grid  2D uint8 kind codes (PASS / REFLECT / TERMINATE) of a grid crop, row 0 = lowest y; the grid's own
               frame starts at its corner (x, y in metres from that corner); cells outside count as OUTSIDE
    dx, dy     unit ray directions (horizontal), arrays of the same length
    limit      per-ray (or scalar) maximum entry distance in metres
    Returns (distance, kind): the entry distance into the first stop cell (0 when the start cell stops) and its kind
    code, or (inf, PASS) when no stop cell is entered within the limit. Entering = crossing the cell boundary, the
    same as the reference raster world: a ray that reaches a cell at its corner enters both side cells."""
    grid = np.asarray(kind_grid)
    height, width = grid.shape
    dx = np.array(dx, dtype=float, copy=True)
    dy = np.array(dy, dtype=float, copy=True)
    count = dx.size
    limit = np.broadcast_to(np.asarray(limit, dtype=float), (count,)).copy()
    res = float(resolution)
    dx[np.abs(dx) < 1e-14] = 0.
    dy[np.abs(dy) < 1e-14] = 0.

    def states(cx, cy):
        out = np.full(cx.shape, OUTSIDE, dtype=np.uint8)
        inside = (cx >= 0) & (cx < width) & (cy >= 0) & (cy < height)
        out[inside] = grid[cy[inside], cx[inside]]
        return out

    cx = np.full(count, int(math.floor(x / res)), dtype=np.int64)
    cy = np.full(count, int(math.floor(y / res)), dtype=np.int64)
    sx, sy = np.sign(dx).astype(np.int64), np.sign(dy).astype(np.int64)
    invx = np.divide(res, np.abs(dx), out=np.full(count, np.inf), where=dx != 0)
    invy = np.divide(res, np.abs(dy), out=np.full(count, np.inf), where=dy != 0)
    edge_x = np.where(sx > 0, (cx + 1) * res, cx * res)
    edge_y = np.where(sy > 0, (cy + 1) * res, cy * res)
    tx = np.divide(edge_x - x, dx, out=np.full(count, np.inf), where=dx != 0)
    ty = np.divide(edge_y - y, dy, out=np.full(count, np.inf), where=dy != 0)
    distance = np.full(count, np.inf)
    kind = states(cx, cy)
    active = kind == PASS
    distance[~active] = 0.
    kind[active] = PASS
    finite = limit[np.isfinite(limit)]
    longest = float(finite.max()) if finite.size else 0.
    for _ in range(2 * int(math.ceil(max(longest, 0.) / res)) + 4):
        ids = np.flatnonzero(active)
        if not ids.size:
            break
        near = np.minimum(tx[ids], ty[ids])
        finished = near > limit[ids]
        active[ids[finished]] = False
        ids, near = ids[~finished], near[~finished]
        if not ids.size:
            continue
        cross_x = tx[ids] <= ty[ids] + 1e-12
        cross_y = ty[ids] <= tx[ids] + 1e-12
        nx, ny = cx[ids] + cross_x * sx[ids], cy[ids] + cross_y * sy[ids]
        state = states(nx, ny)
        tie = cross_x & cross_y
        if np.any(tie):
            ii = ids[tie]
            side_x = states(cx[ii] + sx[ii], cy[ii])
            side_y = states(cx[ii], cy[ii] + sy[ii])
            state[tie] = np.maximum(state[tie], np.maximum(side_x, side_y))
        hit = state != PASS
        distance[ids[hit]] = np.maximum(0., near[hit])
        kind[ids[hit]] = state[hit]
        active[ids[hit]] = False
        cx[ids], cy[ids] = nx, ny
        tx[ids] += np.where(cross_x, invx[ids], 0.)
        ty[ids] += np.where(cross_y, invy[ids], 0.)
    if np.any(active):
        raise RuntimeError('first_stop: DDA exhausted with active rays (limit too large for the loop bound)')
    return distance, kind


def kind_lut(reflect=(), terminate=()):
    """uint8 lookup class -> kind for first_stop (256 entries; unlisted classes PASS)."""
    lut = np.full(256, PASS, dtype=np.uint8)
    for value in reflect:
        lut[int(value)] = REFLECT
    for value in terminate:
        lut[int(value)] = TERMINATE
    return lut
