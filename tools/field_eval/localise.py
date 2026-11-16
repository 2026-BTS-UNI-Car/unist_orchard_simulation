"""Map-based LiDAR localisation against a landmark map (poles, trunks); independent of the executor's corridor band.

Per scan (all in the base frame = footprint centre, x forward, y left; the map frame is the reference map's):
  1. points     beam j looks at the body angle angle_min + j * angle_increment + lidar_yaw_offset (the executor's
                ScanAdapter convention, sim/ubts/steering/s2_format.py); point = lidar mount + r (cos, sin). Ranges
                outside [min_range_m, max_range_m] and points on the vehicle (footprint + body_margin_m) are dropped.
  2. clusters   consecutive points (beam order, the scan wraps) closer than cluster_gap_m + 2 r dtheta form a
                cluster; clusters with >= cluster_min_points points and a chord <= cluster_max_extent_m are landmark
                candidates (long objects: walls, hedges, the ground are not).
  3. centres    a cluster's centre is the circle of the landmark radius fitted to its points (Gauss-Newton, fixed
                radius, started at centroid + pi/4 R away from the sensor: the mean of a half circle sampled evenly
                across its width); radius 0 = the centroid.
  4. associate  the centres, moved by the predicted pose, go to the nearest map landmark within gate_init_m, then
                (after a first solve) within gate_m; one cluster per landmark (the closer one).
  5. solve      weighted least squares SE(2) of the matched centre pairs (closed form, weights sqrt(points)); pairs
                with a residual above max(outlier_m, 3 x median) are dropped and the pose solved again. One match:
                translation only with the predicted heading.
  6. predict    the next pose from the last pose, the smoothed speed along the heading and the IMU yaw rate (gyro
                projected on the gravity 'up' from the accelerometer, bias from the standstill before the drive),
                or the last yaw rate when there is no IMU.
Start: the given start pose (map frame); on the first scan with landmarks a grid search around it (along +-init_along_m,
lateral +-init_lateral_m, yaw +-init_yaw_deg) picks the candidate with the best soft match count. After a loss longer
than relock_after_s the same search runs around the prediction.

Quality per scan: OK (>= ok_match pairs on both lane rows, residual RMS <= ok_rms_m, jump from the prediction <=
jump_m), WEAK (>= 1 pair, otherwise), LOST (no pair: the pose is the prediction), REJECTED (a solution farther than
reject_jump_m from the prediction: the prediction is kept). Only OK and WEAK poses are measurements.

The start search and along_check() score a pose by the clusters near map landmarks minus the map landmarks that
should be seen (lidar distance <= check_range_m) but have no cluster; that term separates poses one landmark spacing
apart near the row ends. along_check() scores the whole trajectory moved by -1, 0, +1 spacing and reports 'suspect'
when the solved one is not clearly the best.

Limits: a pole map is periodic along the row; when no row end (or other irregular structure) is in view near the start,
a start along-row error above about half the spacing can lock the solution one spacing off (along_check() reports it;
the lateral and heading values are not affected, the row end error e is). Scan motion during one revolution (0.1 s) is
not corrected. Landmarks that are not in the map are ignored only when they are farther than the gate from every map
landmark. Occlusion is not modelled (hence the short check range).
"""
from __future__ import annotations

import math

import numpy as np

OK, WEAK, LOST, REJECTED, NONE = 1, 2, 3, 4, 0
QUALITY_NAMES = {OK: 'ok', WEAK: 'weak', LOST: 'lost', REJECTED: 'rejected', NONE: 'none'}
DEFAULTS = dict(min_range_m=.30, max_range_m=15., body_margin_m=.10, cluster_gap_m=.10, cluster_min_points=3,
                cluster_max_extent_m=.60, gate_init_m=.50, gate_m=.25, outlier_m=.10, ok_match=4, ok_rms_m=.05,
                jump_m=.30, reject_jump_m=.75, speed_alpha=.5, init_along_m=1.0, init_lateral_m=.5, init_yaw_deg=10.,
                init_step_m=.1, init_step_deg=2., init_score_m=.20, relock_after_s=1.0, max_dt_s=1.0,
                check_range_m=8., along_check_every=2)


def wrap(a):
    return (a + np.pi) % (2. * np.pi) - np.pi


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def solve_se2(body, world, weight):
    """Weighted least-squares (x, y, yaw) with world ~ R(yaw) body + (x, y) (closed form, >= 2 pairs)."""
    w = weight / weight.sum()
    cb, cw = w @ body, w @ world
    b, m = body - cb, world - cw
    s = float(np.sum(w * (b[:, 0] * m[:, 1] - b[:, 1] * m[:, 0])))
    c = float(np.sum(w * (b[:, 0] * m[:, 0] + b[:, 1] * m[:, 1])))
    th = math.atan2(s, c)
    t = cw - rot(th) @ cb
    return float(t[0]), float(t[1]), th


def transform(points, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.column_stack((x + c * points[:, 0] - s * points[:, 1], y + s * points[:, 0] + c * points[:, 1]))


class Vehicle:
    """Lidar mount and footprint in the base frame (footprint centre)."""

    def __init__(self, lidar_x_in_base_m, lidar_yaw_offset_deg, lidar_y_in_base_m=0., footprint_length_m=1.22,
                 footprint_width_m=.70, imu_yaw_sign=1.):
        self.lidar_x = float(lidar_x_in_base_m)
        self.lidar_y = float(lidar_y_in_base_m)
        self.yaw_offset = math.radians(float(lidar_yaw_offset_deg))
        self.length = float(footprint_length_m)
        self.width = float(footprint_width_m)
        self.imu_yaw_sign = 1. if float(imu_yaw_sign) >= 0 else -1.

    @property
    def front_bumper_m(self):
        return self.length / 2.

    def describe(self):
        return dict(lidar_x_in_base_m=self.lidar_x, lidar_y_in_base_m=self.lidar_y,
                    lidar_yaw_offset_deg=round(math.degrees(self.yaw_offset), 6), footprint_length_m=self.length,
                    footprint_width_m=self.width, imu_yaw_sign=self.imu_yaw_sign)


class ScanGeometry:
    def __init__(self, beams, angle_min, angle_increment, vehicle):
        a = angle_min + np.arange(beams) * angle_increment + vehicle.yaw_offset
        self.cos, self.sin = np.cos(a), np.sin(a)
        self.inc = abs(float(angle_increment))
        self.vehicle = vehicle

    def points(self, ranges, cfg):
        v = self.vehicle
        r = np.asarray(ranges, dtype=float)
        ok = np.isfinite(r) & (r >= cfg['min_range_m']) & (r <= cfg['max_range_m'])
        idx = np.flatnonzero(ok)
        rr = r[idx]
        px, py = v.lidar_x + rr * self.cos[idx], v.lidar_y + rr * self.sin[idx]
        body = (np.abs(px) <= v.length / 2. + cfg['body_margin_m']) & (np.abs(py) <= v.width / 2. + cfg['body_margin_m'])
        keep = ~body
        return np.column_stack((px[keep], py[keep])), rr[keep]


def clusters(points, ranges, inc, cfg):
    """List of point arrays (beam order; the last and the first cluster join across the wrap)."""
    n = len(points)
    if n == 0:
        return []
    step = np.hypot(np.diff(points[:, 0]), np.diff(points[:, 1]))
    limit = cfg['cluster_gap_m'] + 2. * ranges[1:] * inc
    cuts = np.flatnonzero(step > limit) + 1
    parts = np.split(np.arange(n), cuts)
    if len(parts) > 1:
        wrap_gap = math.hypot(points[0, 0] - points[-1, 0], points[0, 1] - points[-1, 1])
        if wrap_gap <= cfg['cluster_gap_m'] + 2. * ranges[0] * inc:
            parts[0] = np.concatenate((parts[-1], parts[0]))
            parts = parts[:-1]
    out = []
    for p in parts:
        if len(p) < cfg['cluster_min_points']:
            continue
        pts = points[p]
        chord = math.hypot(pts[-1, 0] - pts[0, 0], pts[-1, 1] - pts[0, 1])
        spread = float(np.max(np.hypot(pts[:, 0] - pts[:, 0].mean(), pts[:, 1] - pts[:, 1].mean())))
        if max(chord, 2. * spread) > cfg['cluster_max_extent_m']:
            continue
        out.append(pts)
    return out


def circle_centre(pts, radius, sensor):
    """Centre of a circle of the given radius through the cluster points (fixed-radius Gauss-Newton)."""
    c0 = pts.mean(axis=0)
    if radius <= 0.:
        return c0
    u = c0 - sensor
    norm = math.hypot(u[0], u[1])
    if norm > 0:
        c0 = c0 + (math.pi / 4.) * radius * u / norm
    c = c0.copy()
    for _ in range(6):
        d = pts - c
        dist = np.hypot(d[:, 0], d[:, 1])
        dist[dist < 1e-9] = 1e-9
        res = dist - radius
        jac = -d / dist[:, None]
        jtj = jac.T @ jac + 1e-6 * np.eye(2)
        step = -np.linalg.solve(jtj, jac.T @ res)
        c = c + step
        if math.hypot(step[0], step[1]) < 1e-5:
            break
    if not np.all(np.isfinite(c)) or math.hypot(c[0] - c0[0], c[1] - c0[1]) > radius:
        return c0
    return c


class Imu:
    """Yaw rate from the raw IMU: gyro projected on the gravity 'up', sign, bias; cumulative integral for prediction."""

    def __init__(self, imu, sign=1., bias_window=None):
        t = np.asarray(imu.get('t', ()), dtype=float)
        self.ok = len(t) >= 2
        self.bias = 0.
        self.up = None
        if not self.ok:
            return
        order = np.argsort(t, kind='stable')
        t = t[order]
        gyro, accel = np.asarray(imu['gyro'], float)[order], np.asarray(imu['accel'], float)[order]
        head = accel[: max(10, min(len(accel), 200))]
        up = np.median(head, axis=0)
        norm = np.linalg.norm(up)
        up = np.array([0., 0., 1.]) if not np.isfinite(norm) or norm < 1. else up / norm
        self.up = up
        rate = sign * (gyro @ up)
        if bias_window is not None:
            a, b = bias_window
            sel = (t >= a) & (t <= b)
            if np.count_nonzero(sel) >= 10:
                self.bias = float(np.mean(rate[sel]))
        self.t, self.rate = t, rate - self.bias
        self.cum = np.concatenate(([0.], np.cumsum(.5 * (self.rate[1:] + self.rate[:-1]) * np.diff(t))))

    def delta(self, t0, t1):
        """Integrated yaw [rad] from t0 to t1, or None outside the IMU record."""
        if not self.ok or t0 < self.t[0] or t1 > self.t[-1]:
            return None
        return float(np.interp(t1, self.t, self.cum) - np.interp(t0, self.t, self.cum))

    def describe(self):
        return dict(available=self.ok, bias_rad_s=self.bias, up=None if self.up is None else [round(float(v), 4)
                                                                                         for v in self.up])


def landmark_spacing(refmap):
    """Median along-row gap between neighbouring landmarks of the lane rows, or None (fewer than 4 gaps)."""
    gaps = []
    for row in refmap.lane_rows:
        along = sorted(refmap.to_row(lm[2], lm[3], 0.)[0] for lm in refmap.landmarks if lm[1] == row)
        gaps += [b - a for a, b in zip(along, along[1:]) if b - a > .05]
    return float(np.median(gaps)) if len(gaps) >= 4 else None


class Localiser:
    def __init__(self, refmap, vehicle, config=None):
        self.cfg = dict(DEFAULTS, **(config or {}))
        self.map = refmap
        self.vehicle = vehicle
        self.lm, self.side = refmap.landmark_array()
        if len(self.lm) < 2:
            raise ValueError('localisation needs a landmark map (>= 2 landmarks); this map has %d' % len(self.lm))
        self.search_radius = float(np.median(self.lm[:, 2]))
        self.spacing = landmark_spacing(refmap)

    # --- one scan ------------------------------------------------------------------------------------------------
    def _centres(self, cl, radius):
        sensor = np.array([self.vehicle.lidar_x, self.vehicle.lidar_y])
        return np.array([circle_centre(p, r, sensor) for p, r in zip(cl, radius)]).reshape(-1, 2)

    def _associate(self, world, gate):
        if not len(world):
            return np.zeros(0, int), np.zeros(0, int), np.zeros(0)
        d = np.hypot(world[:, None, 0] - self.lm[None, :, 0], world[:, None, 1] - self.lm[None, :, 1])
        best = np.argmin(d, axis=1)
        dist = d[np.arange(len(world)), best]
        ok = np.flatnonzero(dist <= gate)
        chosen = {}
        for i in ok:
            j = best[i]
            if j not in chosen or dist[i] < dist[chosen[j]]:
                chosen[j] = i
        ci = np.array(sorted(chosen.values()), dtype=int)
        return ci, best[ci], dist[ci]

    def _expected(self, pose):
        """Mask of the map landmarks that a scan from this pose should see (lidar distance within
        [min_range_m + 0.2, check_range_m]); occlusion is not modelled, hence the short check range."""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        lx = pose[0] + self.vehicle.lidar_x * c - self.vehicle.lidar_y * s
        ly = pose[1] + self.vehicle.lidar_x * s + self.vehicle.lidar_y * c
        d = np.hypot(self.lm[:, 0] - lx, self.lm[:, 1] - ly)
        return (d >= self.cfg['min_range_m'] + .2) & (d <= self.cfg['check_range_m'])

    def _score(self, centres, pose):
        """Soft count of clusters near a map landmark minus the expected landmarks without a cluster (the second
        term separates poses one landmark spacing apart near the row ends)."""
        if not len(centres):
            return 0.
        world = transform(centres, pose)
        dm = np.hypot(world[:, None, 0] - self.lm[None, :, 0], world[:, None, 1] - self.lm[None, :, 1])
        matched = float(np.sum(np.clip(1. - dm.min(axis=1) / self.cfg['init_score_m'], 0., None)))
        expected = self._expected(pose)
        missing = int(np.count_nonzero(dm[:, expected].min(axis=0) > self.cfg['gate_m'])) if np.any(expected) else 0
        return matched - missing

    def search(self, cl, centre_pose):
        """Grid search around centre_pose (map frame) for the pose with the best soft match count."""
        cfg = self.cfg
        centres = self._centres(cl, [self.search_radius] * len(cl))
        x0, y0, th0 = centre_pose
        axis = self.map.geometry['axis_rad']
        ca, sa = math.cos(axis), math.sin(axis)
        alongs = np.arange(-cfg['init_along_m'], cfg['init_along_m'] + 1e-9, cfg['init_step_m'])
        lats = np.arange(-cfg['init_lateral_m'], cfg['init_lateral_m'] + 1e-9, cfg['init_step_m'])
        yaws = np.radians(np.arange(-cfg['init_yaw_deg'], cfg['init_yaw_deg'] + 1e-9, cfg['init_step_deg']))
        best, best_key = None, None
        for dy in yaws:
            for da in alongs:
                for dl in lats:
                    pose = (x0 + da * ca - dl * sa, y0 + da * sa + dl * ca, th0 + dy)
                    score = self._score(centres, pose)
                    key = (round(score, 6), -(da * da + dl * dl + (dy * 2.) ** 2))
                    if best_key is None or key > best_key:
                        best, best_key = pose, key
        return best, best_key[0]

    def solve(self, cl, pred):
        """(pose, info) for one scan's clusters from the predicted pose."""
        cfg = self.cfg
        info = dict(n_clusters=len(cl), n_match=0, sides=0, rms_m=math.nan)
        if not cl:
            return None, info
        npts = np.array([len(p) for p in cl], dtype=float)
        radius = np.full(len(cl), self.map.default_radius_m)
        centres = self._centres(cl, radius)
        pose = pred
        for gate in (cfg['gate_init_m'], cfg['gate_m']):
            ci, lj, _ = self._associate(transform(centres, pose), gate)
            if not len(ci):
                return None, info
            changed = radius[ci] != self.lm[lj, 2]
            if np.any(changed):
                radius[ci] = self.lm[lj, 2]
                centres[ci] = self._centres([cl[i] for i in ci], radius[ci])
            pose = self._fit(centres[ci], self.lm[lj, :2], np.sqrt(npts[ci]), pred)
        for _ in range(2):
            res = np.hypot(*(transform(centres[ci], pose) - self.lm[lj, :2]).T)
            limit = max(cfg['outlier_m'], 3. * float(np.median(res)))
            keep = res <= limit
            if keep.all() or np.count_nonzero(keep) < 1:
                break
            ci, lj = ci[keep], lj[keep]
            pose = self._fit(centres[ci], self.lm[lj, :2], np.sqrt(npts[ci]), pred)
        res = np.hypot(*(transform(centres[ci], pose) - self.lm[lj, :2]).T)
        info.update(n_match=int(len(ci)), sides=int(len({int(s) for s in self.side[lj] if s < 2})),
                    rms_m=float(np.sqrt(np.mean(res ** 2))))
        return pose, info

    @staticmethod
    def _fit(body, world, weight, pred):
        if len(body) >= 2:
            return solve_se2(body, world, weight)
        th = pred[2]
        t = world[0] - rot(th) @ body[0]
        return float(t[0]), float(t[1]), th

    # --- a whole record ------------------------------------------------------------------------------------------
    def run(self, scans, imu=None, start_pose=None, drive_start=None):
        """Trajectory dict (numpy columns, one row per scan) from the scans dict of sources.py."""
        cfg = self.cfg
        if start_pose is None:
            start_pose = self.map.start_pose
        if start_pose is None:
            raise ValueError('a start pose (map frame x, y, yaw_deg) is needed')
        x0, y0, yaw0_deg = start_pose
        geo = ScanGeometry(scans['beams'], scans['angle_min'], scans['angle_increment'], self.vehicle)
        t_all = np.asarray(scans['t'], dtype=float)
        bias_window = None
        if drive_start is not None and len(t_all):
            bias_window = (float(t_all[0]) - 5., float(drive_start) - .2)
        gyro = Imu(imu or {}, self.vehicle.imu_yaw_sign, bias_window)
        n = len(t_all)
        out = {k: np.full(n, np.nan) for k in ('x', 'y', 'yaw', 'rms_m', 'jump_m', 'pred_x', 'pred_y', 'pred_yaw')}
        out.update({k: np.zeros(n, dtype=int) for k in ('quality', 'n_clusters', 'n_match', 'sides')})
        centres_store = [np.zeros((0, 2))] * n
        pose, t_prev, speed, yaw_rate = None, None, 0., 0.
        last_fix_t, init = None, None
        for i in range(n):
            t = float(t_all[i])
            pts, rr = geo.points(scans['ranges'][i], cfg)
            cl = clusters(pts, rr, geo.inc, cfg)
            out['n_clusters'][i] = len(cl)
            centres_store[i] = self._centres(cl, [self.search_radius] * len(cl))
            if pose is None:
                if len(cl) < 3:
                    continue
                guess, score = self.search(cl, (x0, y0, math.radians(yaw0_deg)))
                init = dict(t=t, scan=i, score=score, pose=[guess[0], guess[1], math.degrees(guess[2])],
                            moved_m=math.hypot(guess[0] - x0, guess[1] - y0),
                            turned_deg=math.degrees(guess[2]) - yaw0_deg)
                pred = guess
            else:
                dt = t - t_prev
                if dt <= 0 or dt > cfg['max_dt_s']:
                    dt = max(dt, 0.)
                dyaw = gyro.delta(t_prev, t)
                if dyaw is None:
                    dyaw = yaw_rate * dt
                mid = pose[2] + dyaw / 2.
                pred = (pose[0] + speed * dt * math.cos(mid), pose[1] + speed * dt * math.sin(mid), pose[2] + dyaw)
                if last_fix_t is not None and t - last_fix_t > cfg['relock_after_s'] and len(cl) >= 3:
                    pred, _ = self.search(cl, pred)
            out['pred_x'][i], out['pred_y'][i], out['pred_yaw'][i] = pred
            sol, info = self.solve(cl, pred)
            for k in ('n_clusters', 'n_match', 'sides'):
                out[k][i] = info[k]
            out['rms_m'][i] = info['rms_m']
            quality = LOST
            new = pred
            if sol is not None:
                jump = math.hypot(sol[0] - pred[0], sol[1] - pred[1])
                out['jump_m'][i] = jump
                if jump > cfg['reject_jump_m'] and pose is not None:
                    quality = REJECTED
                else:
                    new = sol
                    good = (info['n_match'] >= cfg['ok_match'] and info['sides'] >= 2
                            and info['rms_m'] <= cfg['ok_rms_m'] and jump <= cfg['jump_m'])
                    quality = OK if good else WEAK
            if quality in (OK, WEAK):
                if pose is not None and t_prev is not None and t > t_prev:
                    dt = t - t_prev
                    heading = (new[2] + pose[2]) / 2.
                    v_new = ((new[0] - pose[0]) * math.cos(heading) + (new[1] - pose[1]) * math.sin(heading)) / dt
                    w_new = float(wrap(new[2] - pose[2])) / dt
                    a = cfg['speed_alpha']
                    speed = a * v_new + (1. - a) * speed
                    yaw_rate = a * w_new + (1. - a) * yaw_rate
                last_fix_t = t
            pose = (new[0], new[1], float(wrap(new[2])))
            t_prev = t
            out['x'][i], out['y'][i], out['yaw'][i] = pose
            out['quality'][i] = quality
        out['t'] = t_all
        out['init'] = init
        out['imu'] = gyro.describe()
        out['_centres'] = centres_store
        out['along_check'] = self.along_check(out)
        return out

    def along_check(self, traj):
        """Scores of the whole trajectory moved by -1, 0, +1 landmark spacing along the row: a pole map is periodic
        along the row, so a solution one spacing off fits the middle of the row as well as the right one; the row
        ends (and an irregular map) tell them apart. suspect = the trajectory as solved is not clearly the best."""
        if self.spacing is None:
            return dict(spacing_m=None, suspect=None, note='fewer than 4 landmark gaps in the lane rows')
        axis = self.map.geometry['axis_rad']
        ca, sa = math.cos(axis), math.sin(axis)
        meas = np.flatnonzero((traj['quality'] == OK) | (traj['quality'] == WEAK))[::max(1, int(self.cfg['along_check_every']))]
        scores = {}
        for k in (-1, 0, 1):
            total = 0.
            for i in meas:
                pose = (traj['x'][i] + k * self.spacing * ca, traj['y'][i] + k * self.spacing * sa, traj['yaw'][i])
                total += self._score(traj['_centres'][i], pose)
            scores[k] = round(total, 3)
        best = max(scores, key=lambda k: (scores[k], -abs(k)))
        margin = scores[0] - max(scores[-1], scores[1])
        return dict(spacing_m=round(self.spacing, 4), scans=int(len(meas)), scores={str(k): v for k, v in scores.items()},
                    best_shift=best, margin=round(margin, 3),
                    suspect=bool(best != 0 or margin < max(5., .01 * abs(scores[0]))))


def summary(traj, interval=None):
    """Counts by quality, residual and match statistics (inside interval (t_a, t_b) when given)."""
    q = traj['quality']
    sel = np.ones(len(q), dtype=bool)
    if interval is not None:
        sel = (traj['t'] >= interval[0]) & (traj['t'] <= interval[1])
    qs = q[sel]
    meas = sel & ((q == OK) | (q == WEAK))
    rms = traj['rms_m'][meas]
    nm = traj['n_match'][meas]

    def pct(v, p):
        return None if not len(v) else round(float(np.percentile(v, p)), 4)
    return dict(scans=int(np.count_nonzero(sel)),
                counts={QUALITY_NAMES[k]: int(np.count_nonzero(qs == k)) for k in QUALITY_NAMES},
                ok_fraction=None if not len(qs) else round(float(np.mean(qs == OK)), 4),
                measured_fraction=None if not len(qs) else round(float(np.mean((qs == OK) | (qs == WEAK))), 4),
                rms_m=dict(p50=pct(rms, 50), p95=pct(rms, 95), max=pct(rms, 100)),
                n_match=dict(p5=pct(nm, 5), p50=pct(nm, 50)),
                clusters_p50=pct(traj['n_clusters'][sel], 50))
