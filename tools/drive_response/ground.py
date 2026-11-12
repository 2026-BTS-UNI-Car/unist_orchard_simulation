"""In-row / out-of-row for the 10/10 orchard drives (PW-01).

Corridors: single-row-20261102/corridors/CATALOGUE.json (raster-local metres = map frame - YAML origin). A position
is 'in a row aisle' when, for some corridor, its signed distance d from the corridor centreline satisfies
|d| < spacing_median / 2 - 0.35 m and its field coordinate s lies in the corridor's two-sided interval (both rows
present) - the catalogue builder's own trajectory rule (raster_catalogue._trajectory). Corridors excluded for
X_width (pair skips a row line, so the band would cover a neighbouring aisle and the row between) are not used;
every other catalogued pair with a centreline counts (the single-row exclusions concern the test design, not
whether the place is an aisle). The SLAM map is a reconstruction, not truth; positions come from the offline
Cartographer nodes interpolated in time (nodes more than MAX_NODE_GAP_S apart = position unknown).
"""
import json
import math

import numpy as np

MAX_NODE_GAP_S = 2.
EDGE_MARGIN_M = .35
SKIP_RULES = ('X_width', 'X_nosupport')


class Polyline:
    """Closest-segment projection (s along, d signed left) like raster_corridor.CurvilinearFrame."""

    def __init__(self, points, s_origin):
        pts = np.asarray(points, dtype=float)
        seg = np.diff(pts, axis=0)
        length = np.hypot(seg[:, 0], seg[:, 1])
        keep = length > 1e-6
        self.A = pts[:-1][keep]
        self.T = seg[keep] / length[keep][:, None]
        self.L = length[keep]
        self.s_vertex = np.concatenate(([0.], np.cumsum(self.L)))[:-1] - s_origin
        self.lo = np.zeros(len(self.L))
        self.hi = self.L.copy()
        self.lo[0], self.hi[-1] = -np.inf, np.inf

    def project(self, xs, ys, chunk=4096):
        xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
        s_out = np.empty(len(xs))
        d_out = np.empty(len(xs))
        for c0 in range(0, len(xs), chunk):
            x, y = xs[c0:c0 + chunk, None], ys[c0:c0 + chunk, None]
            dx, dy = x - self.A[None, :, 0], y - self.A[None, :, 1]
            t = dx * self.T[None, :, 0] + dy * self.T[None, :, 1]
            tc = np.minimum(np.maximum(t, self.lo[None, :]), self.hi[None, :])
            ex, ey = dx - tc * self.T[None, :, 0], dy - tc * self.T[None, :, 1]
            d2 = ex * ex + ey * ey
            k = np.argmin(d2, axis=1)
            r = np.arange(len(k))
            cross = self.T[k, 0] * dy[r, k] - self.T[k, 1] * dx[r, k]
            dist = np.sqrt(d2[r, k])
            s_out[c0:c0 + chunk] = self.s_vertex[k] + tc[r, k]
            d_out[c0:c0 + chunk] = np.where(dist > 0, np.copysign(dist, cross), 0.)
        return s_out, d_out


class Corridors:
    def __init__(self, catalogue_path):
        with open(catalogue_path, 'r', encoding='utf-8') as fh:
            cat = json.load(fh)
        self.by_record = {}
        self.skipped = []
        for c in cat['corridors']:
            geo = c.get('geometry')
            rules = [e.get('rule') for e in c.get('exclusions') or []]
            if not geo or any(r in SKIP_RULES for r in rules):
                self.skipped.append(dict(record=c['record'], corridor=c['corridor_id'], rules=rules))
                continue
            entry = dict(id=c['corridor_id'], status=c.get('status'),
                         curve=Polyline(geo['polyline_fwd'], geo['s_origin_fwd_m']),
                         s_lo=float(c['two_sided_s_m'][0]), s_hi=float(c['two_sided_s_m'][1]),
                         spacing=float(c['spacing_median_m']), catalogue_traj=c.get('trajectory'))
            self.by_record.setdefault(c['record'], []).append(entry)

    def inside(self, record, x_local, y_local):
        """Boolean array: inside any usable corridor of the record."""
        out = np.zeros(len(x_local), dtype=bool)
        for c in self.by_record.get(record, []):
            s, d = c['curve'].project(x_local, y_local)
            out |= (np.abs(d) < c['spacing'] / 2 - EDGE_MARGIN_M) & (s >= c['s_lo']) & (s <= c['s_hi'])
        return out

    def check_against_catalogue(self, record, traj):
        """Re-count nodes_inside per corridor and compare with the catalogue's own numbers."""
        x = traj['x_map'] - traj['origin'][0]
        y = traj['y_map'] - traj['origin'][1]
        rows = []
        for c in self.by_record.get(record, []):
            s, d = c['curve'].project(x, y)
            n = int(((np.abs(d) < c['spacing'] / 2 - EDGE_MARGIN_M) & (s >= c['s_lo']) & (s <= c['s_hi'])).sum())
            ref = (c['catalogue_traj'] or {}).get('nodes_inside')
            rows.append(dict(corridor=c['id'], nodes_inside=n, catalogue=ref, same=(ref == n)))
        return rows


class Locator:
    """Time -> in-row flag for one recording (None = position unknown)."""

    def __init__(self, rec, corridors):
        tr = rec.traj
        order = np.argsort(tr['t_host'], kind='stable')
        self.t = tr['t_host'][order]
        self.x = tr['x_map'][order] - tr['origin'][0]
        self.y = tr['y_map'][order] - tr['origin'][1]
        self.node_inside = corridors.inside(rec.short, self.x, self.y)
        self.corridors = corridors
        self.record = rec.short

    def at(self, times):
        """(known, inside) boolean arrays for the given host times."""
        times = np.asarray(times, dtype=float)
        j = np.searchsorted(self.t, times)
        known = (j > 0) & (j < len(self.t))
        jj = np.clip(j, 1, len(self.t) - 1)
        gap = self.t[jj] - self.t[jj - 1]
        known &= gap <= MAX_NODE_GAP_S
        f = np.clip((times - self.t[jj - 1]) / np.maximum(gap, 1e-9), 0., 1.)
        x = self.x[jj - 1] + f * (self.x[jj] - self.x[jj - 1])
        y = self.y[jj - 1] + f * (self.y[jj] - self.y[jj - 1])
        inside = np.zeros(len(times), dtype=bool)
        if known.any():
            inside[known] = self.corridors.inside(self.record, x[known], y[known])
        return known, inside

    def path_fraction(self):
        """Share of the node-to-node path length with both nodes inside."""
        seg = np.hypot(np.diff(self.x), np.diff(self.y))
        both = self.node_inside[:-1] & self.node_inside[1:]
        total = float(seg.sum())
        return (float(seg[both].sum()) / total if total > 0 else math.nan), total
