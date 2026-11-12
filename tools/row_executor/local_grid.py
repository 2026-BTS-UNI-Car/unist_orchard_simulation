"""Local occupancy grid of the row executor: layer L2 of design v2 (V2-3, PC-07). No truth, no SIM import, numpy only.

What it is: a rolling occupancy grid in the estimator's odometry frame (the frame of SensorEstimator.base_pose: it starts
at the run's first scan and is not a global map). The window (length_m along the grid axis x width_m across, default
20 x 10 m, behind_m of it behind the vehicle) follows the vehicle; cells that leave the window are dropped. The grid
axis is the vehicle heading when the grid is created (a single row is nearly straight; the window is re-centred, never
rotated).

Update (every every_n_scans processed scans, only while the estimator's scan matching is valid - a scan with an invalid
match has no trustworthy pose and is not inserted):
  * every return of the converted scan (the ranges the estimator got, inf = no return) inside max_range_m is a hit; a
    cell counts at most one hit per update (hits saturate at 255)
  * optional free-space carving (free_beam_step N > 0): every N-th beam with a return marks the cells before the
    return as free (free counts saturate at 255); a beam without a return carves nothing (no return is no evidence)
  * a cell is OCCUPIED when it was hit in at least min_hits updates (persistence over several scans: a speckle, a single
    branch tip or a passing object seen once is not a row) and, with carving, hits >= free_ratio x free.

Row lines (fit, after every update): the occupied cells within [-fit_behind_m, +fit_ahead_m] along and
[fit_cross_min_m, fit_cross_max_m] across the vehicle are searched for the axis that concentrates them into the
sharpest pair of parallel bands (the coverage of a band = distinct along bins with a cell, as the estimator's
acquisition does).
  * with the estimator's band (the caller passes its heading and line offsets): the axis is searched +-fit_track_deg
    around the band heading and each side takes the strongest grid band within 0.6 m of the band's line; a side without
    one gives no fit ('unconfirmed_*'). The grid confirms and refines the lines the fast layer tracks; it never offers
    another pair while a band exists.
  * without a band: +-fit_track_deg around the last fit while it is recent (fit_track_s), else +-fit_window_deg around
    the vehicle heading; each side takes its nearest band with enough coverage, snapped outward to the strongest band
    within 0.6 m (trunk line, not the canopy fringe).
Both lines are then refined by one shared-slope least-squares fit with trimming. A fit needs both sides and a width
inside [width_min_m, width_max_m]; its axis keeps the band's orientation (else the travel direction).

Output (GridFit): the row axis and the two line offsets in the odometry frame, the width, the coverage per side and a
quality in [0, 1]; observation() turns it into the estimator's corridor-observation convention (lateral offset of the
base from the corridor centre along the corridor's left normal, heading error = corridor axis - vehicle heading).
export() gives a small, bounded serialisation for an operator map (bitmap or cell list, size cap).

Every value is a placeholder (SIM development values, not field calibrations). Design v2 V2-3: the fusion with the fast
layer L0 (estimator.on_corridor_observation, source 'local_grid') uses a low gain and is OFF by default.
"""
import base64
from dataclasses import dataclass
import math
import time

import numpy as np

from .perception_settings import GridSettings

SCHEMA_EXPORT = 'ubts.row_executor.local_grid.v1'
SOURCE_NAME = 'local_grid'
SNAP_OUTWARD_M = .6          # a side's nearest band is snapped outward to the strongest band within this distance
EXPORT_ENCODINGS = ('bitmap', 'cells')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class GridFit:
    stamp: float
    axis: float                 # row axis in the odometry frame [rad], oriented with travel
    left_n: float               # left line: n . p = left_n, n = (-sin axis, cos axis) [m]
    right_n: float              # right line
    width: float
    coverage_left: int
    coverage_right: int
    cells_left: int
    cells_right: int
    rms_m: float
    quality: float
    search: str                 # 'band' (around the estimator band), 'track' (last fit) or 'window' (vehicle)

    def observation(self, base_pose):
        """(lateral_offset_m, heading_error_rad) of a base pose (x, y, yaw) in the odometry frame, in the estimator's
        snapshot convention: lateral = base offset from the corridor centre along the corridor's left normal,
        heading error = corridor axis - vehicle heading."""
        x, y, yaw = base_pose
        n = -math.sin(self.axis) * x + math.cos(self.axis) * y
        return n - .5 * (self.left_n + self.right_n), wrap(self.axis - yaw)

    def summary(self, now=None):
        out = dict(axis_deg=round(math.degrees(self.axis), 3), width_m=round(self.width, 3),
                   coverage=[self.coverage_left, self.coverage_right], cells=[self.cells_left, self.cells_right],
                   rms_m=round(self.rms_m, 4), quality=round(self.quality, 3), search=self.search)
        if now is not None:
            out['age_s'] = round(float(now) - self.stamp, 3)
        return out


class LocalGrid:
    """The rolling grid of one run (module docstring). Not thread-safe (the Runner serialises the calls)."""

    def __init__(self, settings=GridSettings()):
        problems = settings.problems()
        if problems:
            raise ValueError('local_grid settings: ' + '; '.join(problems))
        self.s = settings
        self.res = float(settings.cell_m)
        self.ni = int(round(settings.length_m / self.res))
        self.nj = int(round(settings.width_m / self.res))
        self.axis = None                  # grid axis in the odometry frame (set by the first insertion)
        self._c = self._s = 0.
        self.i0 = self.j0 = 0             # global cell index of array cell [0, 0]
        self.hits = np.zeros((self.ni, self.nj), dtype=np.uint8)
        self.free = np.zeros((self.ni, self.nj), dtype=np.uint8) if settings.free_beam_step > 0 else None
        self.updates = 0
        self.scans_seen = 0
        self.skipped_invalid = 0
        self.recentres = 0
        self.last_update_stamp = None
        self.fit = None                   # last valid GridFit
        self.last_fit_reason = 'no_update'
        self.fits = 0
        self.fit_failures = {}
        self.last_cost_s = None           # wall time of the last update + fit (perf_counter)

    # --- frames ------------------------------------------------------------------------------------------------
    def _to_grid(self, x, y):
        return self._c * x + self._s * y, -self._s * x + self._c * y

    def _to_odom(self, u, v):
        return self._c * u - self._s * v, self._s * u + self._c * v

    def _start(self, base_pose):
        self.axis = float(base_pose[2])
        self._c, self._s = math.cos(self.axis), math.sin(self.axis)
        self.i0, self.j0 = self._origin_for(base_pose)

    def _origin_for(self, base_pose):
        u, v = self._to_grid(base_pose[0], base_pose[1])
        return (int(math.floor(u / self.res)) - int(round(self.s.behind_m / self.res)),
                int(math.floor(v / self.res)) - self.nj // 2)

    def _recentre(self, base_pose):
        i0, j0 = self._origin_for(base_pose)
        limit = max(1, int(round(self.s.recentre_m / self.res)))
        if abs(i0 - self.i0) < limit and abs(j0 - self.j0) < limit:
            return
        di, dj = i0 - self.i0, j0 - self.j0
        for name in ('hits', 'free'):
            old = getattr(self, name)
            if old is None:
                continue
            new = np.zeros_like(old)
            si0, si1 = max(0, di), min(self.ni, self.ni + di)
            sj0, sj1 = max(0, dj), min(self.nj, self.nj + dj)
            if si1 > si0 and sj1 > sj0:
                new[si0 - di:si1 - di, sj0 - dj:sj1 - dj] = old[si0:si1, sj0:sj1]
            setattr(self, name, new)
        self.i0, self.j0 = i0, j0
        self.recentres += 1

    def _flat_cells(self, x, y):
        """Flat array indices of odometry points inside the window (unique)."""
        u, v = self._to_grid(x, y)
        gi = np.floor(u / self.res).astype(np.int64) - self.i0
        gj = np.floor(v / self.res).astype(np.int64) - self.j0
        keep = (gi >= 0) & (gi < self.ni) & (gj >= 0) & (gj < self.nj)
        return np.unique(gi[keep] * self.nj + gj[keep])

    @staticmethod
    def _bump(array, flat):
        view = array.reshape(-1)
        values = view[flat]
        view[flat] = np.where(values < 255, values + 1, 255).astype(np.uint8)

    # --- update -------------------------------------------------------------------------------------------------
    def on_scan(self, stamp, ranges, angle_min, increment, laser_pose, odom_valid, base_pose, prior_axis=None,
                prior_lines=None):
        """One processed scan. ranges: the converted scan the estimator got (inf / NaN = no return), laser_pose /
        base_pose: (x, y, yaw) in the odometry frame at the scan stamp. Returns the new GridFit, None (no update this
        scan, or no fit: last_fit_reason), never raises on data. prior_axis: the estimator's band heading (odometry)
        or None, prior_lines: its (left_n, right_n) line offsets or None (fit_lines)."""
        self.scans_seen += 1
        if self.scans_seen % self.s.every_n_scans:
            return None
        if not odom_valid:
            self.skipped_invalid += 1
            self.last_fit_reason = 'odometry_invalid'
            return None
        t0 = time.perf_counter()
        self.insert(ranges, angle_min, increment, laser_pose, base_pose)
        self.last_update_stamp = float(stamp)
        fit = self.fit_lines(stamp, base_pose, prior_axis, prior_lines)
        self.last_cost_s = time.perf_counter() - t0
        return fit

    def insert(self, ranges, angle_min, increment, laser_pose, base_pose):
        if self.axis is None:
            self._start(base_pose)
        else:
            self._recentre(base_pose)
        s = self.s
        r = np.asarray(ranges, dtype=np.float64)
        angles = float(angle_min) + np.arange(len(r)) * float(increment)
        lx, ly, lyaw = (float(v) for v in laser_pose)
        valid = np.isfinite(r) & (r > 0.) & (r <= s.max_range_m)
        bearing = angles + lyaw
        x = lx + r[valid] * np.cos(bearing[valid])
        y = ly + r[valid] * np.sin(bearing[valid])
        self._bump(self.hits, self._flat_cells(x, y))
        if self.free is not None:
            beams = np.flatnonzero(np.isfinite(r) & (r > self.res))[::s.free_beam_step]
            if len(beams):
                reach = np.minimum(r[beams], s.max_range_m)
                k = int(math.ceil(s.max_range_m / self.res))
                steps = (np.arange(k) + .5) * self.res
                inside = steps[None, :] < (reach[:, None] - self.res)
                dist = np.broadcast_to(steps[None, :], inside.shape)[inside]
                angle = np.broadcast_to(bearing[beams][:, None], inside.shape)[inside]
                self._bump(self.free, self._flat_cells(lx + dist * np.cos(angle), ly + dist * np.sin(angle)))
        self.updates += 1

    def occupied(self):
        occ = self.hits >= self.s.min_hits
        if self.free is not None:
            occ &= self.hits.astype(np.float32) >= self.s.free_ratio * self.free.astype(np.float32)
        return occ

    def occupied_points(self):
        """Odometry-frame centres of the occupied cells (N x 2)."""
        i, j = np.nonzero(self.occupied())
        u = (i + self.i0 + .5) * self.res
        v = (j + self.j0 + .5) * self.res
        x, y = self._to_odom(u, v)
        return np.column_stack((x, y))

    # --- row lines -----------------------------------------------------------------------------------------------
    def _fail(self, reason):
        self.last_fit_reason = reason
        self.fit_failures[reason] = self.fit_failures.get(reason, 0) + 1
        return None

    def fit_lines(self, stamp, base_pose, prior_axis=None, prior_lines=None):
        """Row lines from the occupied cells around the base. prior_axis / prior_lines: the estimator's band (heading,
        (left_n, right_n) normal offsets in the odometry frame) or None. With a band the axis is searched
        +-fit_track_deg around its heading and each side takes the grid band nearest to the band's line (within
        SNAP_OUTWARD_M; none there = no line on that side, so no fit): the grid confirms and refines the lines the fast
        layer tracks with the evidence of many scans; it does not offer another pair while the band exists. Without a band: +-fit_track_deg around the last recent fit, else +-fit_window_deg around
        the vehicle heading, and each side takes its nearest band with enough coverage (snapped outward)."""
        s = self.s
        bx, by, yaw = (float(v) for v in base_pose)
        pts = self.occupied_points()
        if len(pts) < 2 * s.fit_min_cells:
            return self._fail('too_few_cells')
        dx, dy = pts[:, 0] - bx, pts[:, 1] - by
        reach = math.hypot(max(s.fit_ahead_m, s.fit_behind_m), s.fit_cross_max_m) + .5
        near = dx * dx + dy * dy <= reach * reach
        dx, dy = dx[near], dy[near]
        if len(dx) < 2 * s.fit_min_cells:
            return self._fail('too_few_cells')
        last = self.fit
        expect = (None, None)
        if prior_axis is not None and prior_lines is not None:
            centre, half, search = float(prior_axis), s.fit_track_deg, 'band'
            nb = -math.sin(centre) * bx + math.cos(centre) * by
            expect = (float(prior_lines[0]) - nb, float(prior_lines[1]) - nb)
        elif last is not None and float(stamp) - last.stamp <= s.fit_track_s:
            centre, half, search = last.axis, s.fit_track_deg, 'track'
        else:
            centre = yaw if prior_axis is None else float(prior_axis)
            if abs(wrap(centre - yaw)) > math.pi / 2:          # orient with travel
                centre = wrap(centre + math.pi)
            half, search = s.fit_window_deg, 'window'
        offsets = np.radians(np.arange(-half, half + 1e-9, s.fit_step_deg))
        thetas = centre + offsets
        best = self._axis_scores(dx, dy, thetas)
        if best is None:
            return self._fail('no_band_pair')
        k, cov2, centres, along, cross = best
        theta = float(thetas[k])
        left = self._side(cov2, centres, cross, 1, expect[0])
        right = self._side(cov2, centres, cross, -1, expect[1])
        if left is None or right is None:
            reason = 'one_sided' if (left is None) != (right is None) else 'no_side'
            return self._fail(('unconfirmed_' if search == 'band' else '') + reason)
        refined = self._refine(along, cross, left[0], right[0])
        if refined is None:
            return self._fail('refine_failed')
        angle, off_l, off_r, n_l, n_r, rms = refined
        width = off_l - off_r
        if not s.width_min_m <= width <= s.width_max_m:
            return self._fail('implausible_width')
        axis = wrap(theta + angle)
        reference = yaw if prior_axis is None else float(prior_axis)
        if abs(wrap(axis - reference)) > math.pi / 2:
            # keep the orientation of the band (or of travel): the lines swap sides and the offsets change sign
            axis = wrap(axis + math.pi)
            off_l, off_r, n_l, n_r = -off_r, -off_l, n_r, n_l
            left, right = right, left
        nb = -math.sin(axis) * bx + math.cos(axis) * by
        quality = min(1., min(left[1], right[1]) / float(s.fit_full_coverage))
        fit = GridFit(stamp=float(stamp), axis=axis, left_n=nb + off_l, right_n=nb + off_r, width=width,
                      coverage_left=int(left[1]), coverage_right=int(right[1]), cells_left=int(n_l),
                      cells_right=int(n_r), rms_m=float(rms), quality=float(quality), search=search)
        self.fit = fit
        self.fits += 1
        self.last_fit_reason = 'fit'
        return fit

    def _axis_scores(self, dx, dy, thetas):
        """Best candidate axis: (index, cov2 of that axis, band centres, along, cross of the region cells)."""
        s = self.s
        c, sn = np.cos(thetas)[:, None], np.sin(thetas)[:, None]
        along = c * dx[None, :] + sn * dy[None, :]
        cross = -sn * dx[None, :] + c * dy[None, :]
        region = (along >= -s.fit_behind_m) & (along <= s.fit_ahead_m) & (np.abs(cross) <= s.fit_cross_max_m)
        kk, nn = np.nonzero(region)
        if len(kk) < 2 * s.fit_min_cells:
            return None
        bin_c, bin_a = s.fit_cross_bin_m, s.fit_along_bin_m
        off_c = int(math.ceil(s.fit_cross_max_m / bin_c)) + 1
        nc = 2 * off_c + 2
        off_a = int(math.ceil(s.fit_behind_m / bin_a)) + 1
        na = off_a + int(math.ceil(s.fit_ahead_m / bin_a)) + 2
        cb = np.floor(cross[kk, nn] / bin_c).astype(np.int64) + off_c
        ab = np.floor(along[kk, nn] / bin_a).astype(np.int64) + off_a
        # distinct along bins of the 2-bin band j = bins j-1 and j (a band centred on the lower edge of bin j)
        key = np.concatenate(((kk * nc + cb) * na + ab, (kk * nc + cb + 1) * na + ab))
        unique = np.unique(key)
        cov2 = np.bincount(unique // na, minlength=len(thetas) * nc).reshape(len(thetas), nc)
        centres = (np.arange(nc) - off_c) * bin_c
        left_mask, right_mask = centres >= s.fit_cross_min_m, centres <= -s.fit_cross_min_m
        score = cov2[:, left_mask].max(axis=1) + cov2[:, right_mask].max(axis=1)
        top = score.max()
        if top <= 0:
            return None
        ties = np.flatnonzero(score == top)
        k = int(ties[np.argmin(np.abs(ties - (len(thetas) - 1) / 2.))])     # the most central of equal scores
        sel = region[k]
        return k, cov2[k], centres, along[k, sel], cross[k, sel]

    def _side(self, cov2, centres, cross, sign, expect=None):
        """(offset, coverage) of a side's line. With `expect` (the band's line offset from the base): the strongest band
        with enough coverage within SNAP_OUTWARD_M of it, None when there is none (the grid does not confirm that
        line). Without: the nearest band with enough coverage, snapped outward to the strongest band within
        SNAP_OUTWARD_M. The offset is the median of the cells near that band. None without a band."""
        s = self.s
        qualifying = np.flatnonzero((sign * centres >= s.fit_cross_min_m) & (sign * centres <= s.fit_cross_max_m)
                                    & (cov2 >= s.fit_min_coverage))
        if not len(qualifying):
            return None
        if expect is not None:
            # confirm-or-nothing: a band line without multi-scan support near it gives no line on this side
            close = qualifying[np.abs(centres[qualifying] - expect) <= SNAP_OUTWARD_M]
            if not len(close):
                return None
            j = int(close[np.argmax(cov2[close])])
        else:
            i = int(qualifying[0] if sign > 0 else qualifying[-1])
            x = centres[i]
            near = (sign * centres >= sign * x) & (sign * centres <= sign * x + SNAP_OUTWARD_M)
            j = int(np.flatnonzero(near)[np.argmax(cov2[near])])
        values = cross[np.abs(cross - centres[j]) <= s.fit_gate_m]
        if len(values) < s.fit_min_cells:
            return None
        return float(np.median(values)), int(cov2[j])

    def _refine(self, along, cross, left, right):
        """Shared-slope least squares of the cells near the two lines (two trims); (angle, left, right, n_left,
        n_right, rms) or None."""
        s = self.s
        masks = [np.abs(cross - offset) <= s.fit_gate_m for offset in (left, right)]
        if any(m.sum() < s.fit_min_cells for m in masks):
            return None
        a = np.concatenate([along[m] for m in masks])
        y = np.concatenate([cross[m] for m in masks])
        side = np.concatenate([np.zeros(int(masks[0].sum())), np.ones(int(masks[1].sum()))])
        design = np.column_stack((a, side == 0, side == 1)).astype(np.float64)
        solution = None
        for _ in range(2):
            solution, *_ = np.linalg.lstsq(design, y, rcond=None)
            keep = np.abs(design @ solution - y) < s.fit_trim_m
            if keep.sum() < 2 * s.fit_min_cells or design[keep, 1].sum() < s.fit_min_cells \
                    or design[keep, 2].sum() < s.fit_min_cells:
                return None
            design, y = design[keep], y[keep]
        solution, *_ = np.linalg.lstsq(design, y, rcond=None)
        slope = float(solution[0])
        if abs(slope) > math.tan(math.radians(4.)):
            return None
        angle = math.atan(slope)
        rms = float(np.sqrt(np.mean((design @ solution - y) ** 2)))
        scale = math.cos(angle)
        return (angle, float(solution[1]) * scale, float(solution[2]) * scale, int(design[:, 1].sum()),
                int(design[:, 2].sum()), rms)

    # --- reporting ------------------------------------------------------------------------------------------------
    def summary(self, now=None):
        """Small status block (every tick; no cell data). proc_ms is a wall-clock value."""
        out = dict(updates=self.updates, fits=self.fits, reason=self.last_fit_reason, recentres=self.recentres,
                   skipped_invalid=self.skipped_invalid, occupied=int(np.count_nonzero(self.occupied())),
                   cell_m=self.res)
        if self.fit is not None:
            out['fit'] = self.fit.summary(now)
        if self.last_cost_s is not None:
            out['proc_ms'] = round(self.last_cost_s * 1e3, 3)
        return out

    def export(self, now=None, base_pose=None, encoding='bitmap', max_bytes=16384, max_cells=2000):
        """Bounded serialisation of the occupied cells for an operator map (run-local odometry frame, no GNSS).

        encoding 'bitmap': the occupied mask (rows = cells along the grid axis from the window origin, columns =
        across), np.packbits big-endian, base64; OR-pooled by 2, 4, 8 until the text fits max_bytes (cell_m is the
        pooled size). encoding 'cells': [[i, j], ...] window indices of at most max_cells occupied cells (the nearest to
        the base when base_pose is given). Grid frame: u = x cos(axis) + y sin(axis), v = -x sin(axis) + y cos(axis);
        cell [i, j] spans u in origin_uv[0] + [i, i+1] * cell_m, v likewise. Lines: the last fit in the odometry frame."""
        if encoding not in EXPORT_ENCODINGS:
            raise ValueError('encoding must be one of %s' % (EXPORT_ENCODINGS,))
        occ = self.occupied()
        out = dict(schema=SCHEMA_EXPORT, frame='run_odometry', updates=self.updates,
                   axis_rad=None if self.axis is None else round(self.axis, 6),
                   origin_uv=[round(self.i0 * self.res, 4), round(self.j0 * self.res, 4)],
                   occupied=int(np.count_nonzero(occ)), encoding=encoding)
        if now is not None and self.last_update_stamp is not None:
            out['age_s'] = round(float(now) - self.last_update_stamp, 3)
        if encoding == 'bitmap':
            pool = 1
            while True:
                grid = occ if pool == 1 else _pool_or(occ, pool)
                text = base64.b64encode(np.packbits(grid.reshape(-1)).tobytes()).decode('ascii')
                if len(text) <= max_bytes or pool >= 8:
                    break
                pool *= 2
            out.update(cell_m=round(self.res * pool, 4), shape=[int(grid.shape[0]), int(grid.shape[1])], bits=text,
                       truncated=len(text) > max_bytes)
            if out['truncated']:
                out['bits'] = ''
        else:
            i, j = np.nonzero(occ)
            if base_pose is not None and len(i):
                u, v = self._to_grid(float(base_pose[0]), float(base_pose[1]))
                d = ((i + self.i0 + .5) * self.res - u) ** 2 + ((j + self.j0 + .5) * self.res - v) ** 2
                order = np.argsort(d, kind='stable')
                i, j = i[order], j[order]
            out.update(cell_m=self.res, shape=[self.ni, self.nj], cells=np.column_stack((i, j))[:max_cells].tolist(),
                       truncated=len(i) > max_cells)
        if self.fit is not None:
            out['lines'] = self.lines(base_pose)
        if base_pose is not None:
            out['base_pose'] = [round(float(v), 4) for v in base_pose]
        return out

    def lines(self, base_pose=None, behind_m=None, ahead_m=None):
        """The last fit's two row lines and centre line as odometry-frame segments [[x0, y0], [x1, y1]] around the
        base (or around the fit's foot point)."""
        fit = self.fit
        if fit is None:
            return None
        behind = self.s.fit_behind_m if behind_m is None else behind_m
        ahead = self.s.fit_ahead_m if ahead_m is None else ahead_m
        c, s = math.cos(fit.axis), math.sin(fit.axis)
        n = (-s, c)
        if base_pose is None:
            along0 = 0.
        else:
            along0 = c * float(base_pose[0]) + s * float(base_pose[1])
        out = {}
        for name, offset in (('left', fit.left_n), ('right', fit.right_n),
                             ('centre', .5 * (fit.left_n + fit.right_n))):
            seg = []
            for a in (along0 - behind, along0 + ahead):
                seg.append([round(a * c + offset * n[0], 3), round(a * s + offset * n[1], 3)])
            out[name] = seg
        out.update(quality=round(fit.quality, 3), stamp=round(fit.stamp, 4))
        return out


def _pool_or(mask, k):
    ni, nj = mask.shape
    pi, pj = -(-ni // k), -(-nj // k)
    padded = np.zeros((pi * k, pj * k), dtype=bool)
    padded[:ni, :nj] = mask
    return padded.reshape(pi, k, pj, k).any(axis=(1, 3))

