"""C1-C4 and C7 drive measurements from LiDAR odometry + raw IMU + motor status (no encoders, no GNSS).

Every result carries its sample counts: n_* = events / segments, samples = LiDAR velocity samples
(10 Hz) or IMU samples, seconds = covered time. Definitions follow the analyses behind
drive-evidence.txt so that the 10/3 and 10/10 numbers can be reproduced:
  per-scan velocity, travel axis, LiDAR/IMU yaw sign  lidar_response.py
  start-from-rest onset                               drive-evidence-scripts/start_onset_1010.py
  stop from the fade start                            resume-audit-20261013/sim-review/fix/from_release.py
Deliberate differences are listed where they occur (e.g. straight runs are split on a request change).
Front-steer branch: the differential C5 arc gain and C6 pivot ladder of 546fa8c are not imported (left / right
duty differences; the steered vehicle commands throttle and a steering angle, both sides get the same duty).
"""
from collections import defaultdict
import math

import numpy as np

from . import odometry as odo
from . import timeline as tl

STILL_LIDAR_M_S = .01
START_SPEED_M_S = .05            # onset: translation speed > 0.05 m/s for 3 scans (start_onset_1010.py)
STEADY_DROP_S = 1.               # first 1 s of a settled straight segment dropped (10/3 GNSS/SLAM tables)
STRAIGHT_RUN_MIN_SAMPLES = 30    # >= 3 s straight runs (lidar_response.py)
STOP_SPEED_M_S = .03
WANDER_WINDOW_M = 5.             # G5 window: the 10/10 basis (>= 3 s runs at ~0.9 m/s) averaged 4.8 m
WANDER_STEP_M = 1.


def q(values, ps=(10, 50, 90), nd=3):
    v = np.asarray([x for x in values if x is not None and np.isfinite(x)], dtype=float)
    return [round(float(np.percentile(v, p)), nd) for p in ps] if len(v) else None


def r(x, nd=3):
    return None if x is None or not np.isfinite(x) else round(float(x), nd)


# ---------------------------------------------------------------------------------------------------
class Series:
    """Per-recording aligned arrays (scan grid) + IMU yaw (IMU grid)."""

    def __init__(self, rec, pose, quality, chain):
        self.rec = rec
        self.rid = rec.rid
        self.label = rec.label or 'all'
        self.motor = rec.motor
        self.t = rec.scan_t
        self.pose = pose
        self.quality = quality
        self.chain = chain
        v = odo.velocities(self.t, pose, quality, chain)
        self.vx, self.vy, self.w, self.valid, self.theta, self.good = v['vx'], v['vy'], v['w'], v['valid'], \
            v['theta'], v['good']
        self.spd = np.hypot(self.vx, self.vy)
        # IMU: gyro projected on the 2 s low-passed specific force (lidar_response.py imu_yaw)
        imu = rec.imu
        self.ti = imu[:, 0] if len(imu) else np.zeros(0)
        if len(imu) >= 5:
            acc = imu[:, 5:8]
            # bursty receipt (many equal stamps) would give a median step of 0: fall back to the mean rate
            dt = float(np.median(np.diff(self.ti)))
            dt_mean = max(float(self.ti[-1] - self.ti[0]) / (len(self.ti) - 1), 1e-3)
            win = max(1, int(round(2. / (dt if dt >= 1e-3 else dt_mean))))
            k = np.ones(win) / win
            lp = np.column_stack([np.convolve(acc[:, i], k, mode='same') for i in range(3)])
            norm = np.linalg.norm(lp, axis=1, keepdims=True)
            self.up = lp / np.maximum(norm, 1e-9)
            self.wi = np.einsum('ij,ij->i', imu[:, 2:5], self.up)        # rad/s
            self.vib = np.linalg.norm(acc - lp, axis=1)
            self.imu_dt = dt if dt >= 1e-3 else dt_mean
        else:
            self.up = np.zeros((0, 3))
            self.wi = np.zeros(0)
            self.vib = np.zeros(0)
            self.imu_dt = math.nan
        # IMU yaw / vibration averaged over each velocity window (+-2 scans), >= 3 samples
        n = len(self.t)
        self.w_imu = np.full(n, np.nan)
        self.vib_scan = np.full(n, np.nan)
        if len(self.ti) and n > 2 * odo.HALF:
            idx = np.nonzero(self.valid)[0]
            lo = np.searchsorted(self.ti, self.t[idx - odo.HALF], side='left')
            hi = np.searchsorted(self.ti, self.t[idx + odo.HALF], side='right')
            csum = np.concatenate([[0.], np.cumsum(self.wi)])
            csq = np.concatenate([[0.], np.cumsum(self.vib ** 2)])
            cnt = hi - lo
            ok = cnt >= 3
            self.w_imu[idx[ok]] = (csum[hi[ok]] - csum[lo[ok]]) / cnt[ok]
            self.vib_scan[idx[ok]] = np.sqrt((csq[hi[ok]] - csq[lo[ok]]) / cnt[ok])
        # motor status per scan (latest at or before)
        self.k = tl.status_index(self.motor, self.t)
        kk = np.clip(self.k, 0, max(len(self.motor) - 1, 0))
        has = (self.k >= 0) & (len(self.motor) > 0)
        self.req_l = np.where(has, self.motor.req_l[kk], np.nan) if len(self.motor) else np.full(n, np.nan)
        self.req_r = np.where(has, self.motor.req_r[kk], np.nan) if len(self.motor) else np.full(n, np.nan)
        self.settled = np.array([tl.is_settled(self.motor, int(k)) for k in self.k], dtype=bool) if n else \
            np.zeros(0, dtype=bool)
        self.cls = np.array([tl.classify(a, b) for a, b in zip(self.req_l, self.req_r)])
        self.axis = None
        self.lidar_sign = 1.

    def set_axis(self, axis_rad, lidar_sign):
        self.axis = axis_rad
        self.lidar_sign = lidar_sign
        fx, fy = math.cos(axis_rad), math.sin(axis_rad)
        self.u = self.vx * fx + self.vy * fy
        self.lat = -self.vx * fy + self.vy * fx
        self.yaw_dps = np.degrees(self.w) * lidar_sign
        self.imu_dps = np.degrees(self.w_imu)

    def axis_at(self, i):
        """Travel direction (unit vector) in the chain frame at scan i."""
        th = self.theta[i]
        fx, fy = math.cos(self.axis), math.sin(self.axis)
        return np.array([math.cos(th) * fx - math.sin(th) * fy, math.sin(th) * fx + math.cos(th) * fy])


def _contiguous(series, mask, max_gap=.35, split_on_request=True):
    """Index runs of mask (scan gap < max_gap, same chain, optionally same request)."""
    idx = np.nonzero(mask)[0]
    runs = []
    cur = []
    for i in idx:
        if cur:
            j = cur[-1]
            same = (series.t[i] - series.t[j] < max_gap and series.chain[i] == series.chain[j]
                    and (not split_on_request or (series.req_l[i] == series.req_l[j]
                                                  and series.req_r[i] == series.req_r[j])))
            if not same:
                runs.append(np.array(cur))
                cur = []
        cur.append(i)
    if cur:
        runs.append(np.array(cur))
    return runs


# --- axis / sign ---------------------------------------------------------------------------------
def travel_axis(series_list, default_axis_deg):
    """Circular mean direction of LiDAR travel under settled straight forward commands (|v| > 0.2)."""
    angles = []
    for s in series_list:
        m = s.valid & s.settled & (s.cls == 'straight_fwd') & (s.spd > .2)
        angles.append(np.arctan2(s.vy[m], s.vx[m]))
    a = np.concatenate(angles) if angles else np.zeros(0)
    out = dict(samples=int(len(a)), default_axis_deg=default_axis_deg)
    if len(a) >= 50:
        mean = math.atan2(np.sin(a).mean(), np.cos(a).mean())
        dev = np.degrees(np.angle(np.exp(1j * (a - mean))))
        out.update(axis_deg=round(math.degrees(mean), 2), deviation_deg_p10_p50_p90=q(dev, nd=2), used='measured')
    else:
        out.update(axis_deg=default_axis_deg, used='default (too few straight forward samples)')
    return out


def lidar_imu_sign(series_list):
    pairs = []
    for s in series_list:
        m = s.valid & np.isfinite(s.w_imu) & (np.abs(s.w_imu) > math.radians(5))
        pairs.append(np.column_stack([s.w[m], s.w_imu[m]]))
    p = np.concatenate(pairs) if pairs else np.zeros((0, 2))
    out = dict(samples=int(len(p)))
    if len(p) >= 30:
        c = float(np.corrcoef(p[:, 0], p[:, 1])[0, 1])
        sign = 1. if c >= 0 else -1.
        out.update(sign=sign, corr=round(abs(c), 4), gain_median=round(float(np.median(p[:, 0] * sign / p[:, 1])), 4),
                   residual_dps_p10_p50_p90=q(np.degrees(p[:, 0] * sign - p[:, 1]), nd=2), used='measured')
    else:
        out.update(sign=1., used='default +1 (fewer than 30 samples with |IMU yaw| > 5 deg/s)')
    return out


# --- health --------------------------------------------------------------------------------------
def health(rec, series):
    m = rec.motor
    out = dict(rid=rec.rid, layout=rec.layout, label=rec.label or 'all', duration_s=r(rec.duration_s, 1),
               metadata_state=rec.metadata.get('state'), warnings=list(rec.warnings),
               sidecar_manifest=rec.sidecar_manifest)
    out['motor'] = dict(rows=len(m), rate_hz=r((len(m) - 1) / max(m.t[-1] - m.t[0], 1e-9), 2) if len(m) > 1 else None,
                        max_gap_s=r(float(np.max(np.diff(m.t))), 2) if len(m) > 1 else None,
                        stale_rows=int(m.stale.sum()), states=sorted({x for x in m.state if x}))
    ti = rec.imu[:, 0] if len(rec.imu) else np.zeros(0)
    di = np.diff(ti)
    out['imu'] = dict(samples=int(len(ti)), rate_hz=r((len(ti) - 1) / max(ti[-1] - ti[0], 1e-9), 1) if len(ti) > 1 else None,
                      max_gap_s=r(float(di.max()), 3) if len(di) else None,
                      gaps_over_0p3s=int((di > .3).sum()) if len(di) else 0,
                      header_minus_receipt_s_p50=r(float(np.median(rec.imu[:, 1] - ti)), 4) if len(ti) else None)
    ts = rec.scan_t
    ds = np.diff(ts)
    in_window = ds[ds < 2.] if len(ds) else ds
    out['scan'] = dict(total_in_file=int(rec.scan_total), parsed=int(len(ts)), beams=rec.geometry.get('beams'),
                       geometry_rejected=int(rec.scan_geometry_rejected),
                       rate_hz_parsed=r(1. / float(np.median(in_window)), 2) if len(in_window) else None,
                       gaps_over_0p45s=int(((in_window > .45)).sum()) if len(in_window) else 0,
                       header_minus_receipt_s_p50=r(float(np.median(rec.scan_header_t - ts)), 4) if len(ts) else None,
                       windows=len(rec.scan_windows_s) if rec.scan_windows_s is not None else 'all',
                       odometry_from_cache=rec.cached_odometry is not None)
    if series is not None and len(series.t):
        mt = series.chain >= 0
        out['odometry'] = dict(matched=int(mt.sum()), good=int(series.good.sum()),
                               velocity_samples=int(series.valid.sum()),
                               inlier_p50=r(float(np.nanmedian(series.quality[:, 0])), 3) if mt.any() else None,
                               rms_p50=r(float(np.nanmedian(series.quality[:, 1])), 4) if mt.any() else None)
    # counted in FILE order by the reader (the arrays here are sorted and can not show a step back)
    out['clock_regressions'] = int(getattr(rec, 'clock_back', 0) or 0)
    return out


# --- C1 ------------------------------------------------------------------------------------------
def c1_standstill(series_list, min_s=10.):
    windows, ws_all, raw_all = [], [], []
    for s in series_list:
        for a, b in tl.still_windows(s.motor, min_s=min_s, head_s=2.):
            mi = (s.ti >= a) & (s.ti <= b)
            if mi.sum() < 20:
                continue
            wdeg = np.degrees(s.wi[mi])
            raw = np.degrees(s.rec.imu[mi, 2:5])
            up = s.up[mi].mean(axis=0)
            tilt = math.degrees(math.acos(max(-1., min(1., up[2] / max(np.linalg.norm(up), 1e-9)))))
            ms = (s.t >= a) & (s.t <= b) & s.valid
            lidar_speed = float(np.median(s.spd[ms])) if ms.sum() >= 5 else None
            lidar_yaw = float(np.median(np.abs(np.degrees(s.w[ms])))) if ms.sum() >= 5 else None
            windows.append(dict(rid=s.rid, label=s.label, start_s=r(a - s.motor.t[0], 1), seconds=r(b - a, 1),
                                imu_samples=int(mi.sum()), yaw_bias_dps=r(float(wdeg.mean()), 4),
                                yaw_std_dps=r(float(wdeg.std()), 4), raw_gyro_mean_dps=[r(x, 4) for x in raw.mean(axis=0)],
                                accel_norm_m_s2=r(float(np.linalg.norm(s.rec.imu[mi, 5:8], axis=1).mean()), 3),
                                imu_z_tilt_deg=r(tilt, 2), lidar_speed_p50_m_s=r(lidar_speed, 4),
                                lidar_abs_yaw_p50_dps=r(lidar_yaw, 3), lidar_samples=int(ms.sum())))
            ws_all.append(wdeg)
            raw_all.append(raw)
    out = dict(windows=windows, n_windows=len(windows))
    if ws_all:
        w = np.concatenate(ws_all)
        out.update(imu_samples=int(len(w)), seconds=r(sum(x['seconds'] for x in windows), 1),
                   yaw_bias_dps=r(float(w.mean()), 4), yaw_std_dps=r(float(w.std()), 4),
                   bias_window_range_dps=[min(x['yaw_bias_dps'] for x in windows), max(x['yaw_bias_dps'] for x in windows)],
                   lidar_speed_max_p50_m_s=r(max((x['lidar_speed_p50_m_s'] for x in windows
                                                  if x['lidar_speed_p50_m_s'] is not None), default=None), 4))
        out['pass'] = dict(bias_lt_0p1=abs(out['yaw_bias_dps']) < .1, std_le_0p15=out['yaw_std_dps'] <= .15,
                           lidar_still=(out['lidar_speed_max_p50_m_s'] is None or out['lidar_speed_max_p50_m_s'] < STILL_LIDAR_M_S),
                           seconds_ge_25=out['seconds'] >= 25.)
    return out


# --- C2 ------------------------------------------------------------------------------------------
def _onset(s, t_from, t_to, signal, threshold):
    idx = np.nonzero((s.t >= t_from) & (s.t <= t_to))[0]
    for a in range(len(idx) - 2):
        three = signal[idx[a:a + 3]]
        if np.all(np.isfinite(three)) and np.all(three > threshold):
            return int(idx[a])
    return None


def start_events(series_list):
    """Starts from rest to a forward request (both sides > 0). Pivots from rest are in c6."""
    events = []
    for s in series_list:
        m = s.motor
        for run in tl.request_runs(m):
            k = run['k0']
            l, rr = run['l'], run['r']
            if not (math.isfinite(l) and math.isfinite(rr) and l > 0 and rr > 0) or k < 2:
                continue
            prev = range(k - 2, k)
            if any(not (tl.request_zero(m, p) and tl.reported_zero(m, p)) for p in prev):
                continue
            pre = (s.t > m.t[k - 2]) & (s.t < m.t[k - 1]) & s.valid
            if pre.any() and np.nanmax(s.spd[pre]) > START_SPEED_M_S:
                continue
            ramp = tl.ramp_start_from_rest(m, run)
            if ramp is None:
                t0, precise, basis = m.t[k], False, 'no non-zero report (ramp start = first status)'
            else:
                t0, precise, basis = ramp
            req = max(l, rr)
            t_end = run['t_last'] + tl.STATUS_PERIOD_S
            win = (s.t >= t0 - .3) & (s.t <= min(t_end, t0 + 6.))
            coverage = float(s.valid[win].mean()) if win.any() else 0.
            i_on = _onset(s, t0 - .3, min(t_end, t0 + 6.), s.spd, START_SPEED_M_S)
            ev = dict(rid=s.rid, label=s.label, req=[int(l), int(rr)], straight=bool(l == rr),
                      t_rel=r(t0 - m.t[0], 1), precise_t0=bool(precise), basis=basis,
                      hold_s=r(t_end - m.t[k], 2), lidar_coverage=r(coverage, 2))
            full_at = t0 + req / tl.RAMP_PCT_PER_S
            if i_on is not None:
                lag = float(s.t[i_on] - t0)
                ev.update(started=True, onset_after_ramp_start_s=r(lag, 2),
                          duty_at_onset_pct=r(min(req, max(0., lag * tl.RAMP_PCT_PER_S)), 0),
                          onset_after_full_duty_s=r(float(s.t[i_on] - full_at), 2))
                early = (s.t >= s.t[i_on]) & (s.t <= s.t[i_on] + .5) & s.valid
                ev['onset_yaw_dps'] = r(float(np.median(s.yaw_dps[early])), 2) if early.sum() >= 2 else None
                late = (s.t >= t0 + 2.) & (s.t <= min(t_end, t0 + 6.)) & s.valid
                if late.sum() >= 5:
                    vss = float(np.median(s.spd[late]))
                    ev['steady_m_s'] = r(vss, 3)
                    for i in np.nonzero(win & s.valid)[0]:
                        if s.spd[i] >= .9 * vss and s.t[i] >= s.t[i_on]:
                            ev['t90_after_ramp_start_s'] = r(float(s.t[i] - t0), 2)
                            break
            else:
                held = t_end - full_at
                disp, pose_cov = _displacement(s, t0, min(t_end, t0 + 6.))
                ev.update(pose_displacement_m=r(disp, 3), pose_coverage=r(pose_cov, 2), held_at_duty_s=r(held, 2))
                if disp is not None and disp > MOVED_M:
                    # moved, but the quality-gated velocity samples were too sparse to time the onset
                    ev.update(started=True, onset_after_ramp_start_s=None, basis=basis + '; started by pose displacement')
                else:
                    still = (disp is not None and disp < STILL_M and pose_cov >= .7) or coverage >= .5
                    ev.update(started=False, conclusive=bool(held >= 2. and still))
            events.append(ev)
    return events


MOVED_M = .15                    # pose displacement that counts as a start when no onset could be timed
STILL_M = .05                    # pose displacement below which a hold is a conclusive non-start
LADDER_MIN_ATTEMPTS = 3          # B_fwd level: >= 3 attempts, all started (CALIB_SHEET C2 "3/3")


def _displacement(s, a, b):
    """Largest LiDAR displacement from the first matched pose in [a, b] (same chain), pose coverage."""
    idx = np.nonzero((s.t >= a) & (s.t <= b))[0]
    if not len(idx):
        return None, 0.
    m = idx[s.chain[idx] >= 0]
    if len(m) < 3:
        return None, len(m) / len(idx)
    m = m[s.chain[m] == s.chain[m[0]]]
    d = np.hypot(s.pose[m, 0] - s.pose[m[0], 0], s.pose[m, 1] - s.pose[m[0], 1])
    return float(d.max()), len(m) / len(idx)


def c2_breakaway(events):
    """Ladder table per label and straight duty, B_fwd per label, ramp-onset duty distribution."""
    out = dict(events=len(events), labels={})
    by_label = defaultdict(list)
    for e in events:
        by_label[e['label']].append(e)
    for label, es in by_label.items():
        levels = defaultdict(lambda: dict(attempts=0, started=0, conclusive_fail=0, inconclusive=0, onset_duty=[]))
        for e in es:
            if not e['straight']:
                continue
            lev = levels[e['req'][0]]
            lev['attempts'] += 1
            if e['started']:
                lev['started'] += 1
                if e['precise_t0'] and e.get('duty_at_onset_pct') is not None:
                    lev['onset_duty'].append(e['duty_at_onset_pct'])
            elif e['conclusive']:
                lev['conclusive_fail'] += 1
            else:
                lev['inconclusive'] += 1
        table = {str(d): dict(attempts=v['attempts'], started=v['started'], conclusive_fail=v['conclusive_fail'],
                              inconclusive=v['inconclusive'],
                              onset_duty_pct_p50=q(v['onset_duty'], (50,), 0)) for d, v in sorted(levels.items())}
        fails = [d for d, v in levels.items() if v['conclusive_fail']]
        highest_fail = max(fails) if fails else None
        # every attempt at the level started (an inconclusive or failed attempt disqualifies it) and at least
        # LADDER_MIN_ATTEMPTS attempts (the sheet's "3/3"); a level with fewer attempts can not be B_fwd
        above = sorted(d for d, v in levels.items()
                       if v['attempts'] >= LADDER_MIN_ATTEMPTS and v['started'] == v['attempts']
                       and (highest_fail is None or d > highest_fail))
        b = above[0] if above else None
        ladder = dict(table=table, highest_conclusive_fail_pct=highest_fail, b_fwd_pct=b,
                      b_fwd_attempts=levels[b]['attempts'] if b is not None else 0,
                      b_fwd_started=levels[b]['started'] if b is not None else 0,
                      bracketed=highest_fail is not None and b is not None,
                      definition='lowest straight request level above the highest level with a conclusive non-start '
                                 '(held >= 2 s at full duty, LiDAR coverage >= 50 %%) with >= %d attempts that all '
                                 'started' % LADDER_MIN_ATTEMPTS)
        timed = [e for e in es if e['started'] and e['precise_t0'] and e.get('duty_at_onset_pct') is not None]
        onset = [e['duty_at_onset_pct'] for e in timed]
        lag = [e['onset_after_ramp_start_s'] for e in timed]
        yaws = [e.get('onset_yaw_dps') for e in es if e['started'] and e.get('onset_yaw_dps') is not None]
        out['labels'][label] = dict(
            ladder=ladder,
            ramp_onset=dict(n_precise=len(onset), duty_at_onset_pct_p10_p50_p90=q(onset, nd=0),
                            onset_after_ramp_start_s_p10_p50_p90=q(lag, nd=2),
                            note='centred +-2 scan speed window -> onset up to ~0.2 s EARLY (duty up to ~20 %p low)'),
            onset_yaw=dict(n=len(yaws), p10_p50_p90_dps=q(yaws, nd=2),
                           ccw_fraction=r(float(np.mean([y > 0 for y in yaws])), 2) if yaws else None,
                           meaning='yaw right after motion onset: + (CCW) = right side broke away first; weak per-side hint'),
            starts=dict(total=len(es), started=sum(1 for e in es if e['started']),
                        straight=sum(1 for e in es if e['straight'])))
    return out


# --- command segments (C3, C4) ----------------------------------------------------------------------
def command_segments(s, classes, drop_s, min_s):
    """Request runs of the given classes as time intervals: from the first settled status + drop_s to the
    last status of the run (the request may change any time before the next status).

    Per segment: idx (scans inside), valid (scans with a velocity sample), poses (the longest stretch of
    matched scans of one chain: these keep a continuous pose even where the quality gate rejects the
    per-scan velocity, e.g. in dense foliage), IMU mean yaw. Segments are defined by the command, not by
    the continuity of the ICP quality, so one bad scan does not split a 20 m run."""
    m = s.motor
    out = []
    for run in tl.request_runs(m):
        if run['cls'] not in classes:
            continue
        ks = [k for k in range(run['k0'], run['k1'] + 1) if tl.is_settled(m, k)]
        if not ks:
            continue
        a, b = float(m.t[ks[0]]) + drop_s, float(run['t_last'])
        if b - a < min_s:
            continue
        idx = np.nonzero((s.t >= a) & (s.t <= b))[0]
        poses = np.zeros(0, dtype=int)
        matched = idx[s.chain[idx] >= 0]
        if len(matched):
            cuts = np.nonzero(np.diff(s.chain[matched]) != 0)[0] + 1
            for part in np.split(matched, cuts):
                if len(part) > len(poses):
                    poses = part
        mi = (s.ti >= a) & (s.ti <= b)
        out.append(dict(run=run, t_from=a, t_to=b, seconds=b - a, idx=idx, valid=idx[s.valid[idx]], poses=poses,
                        imu_mean_dps=float(np.degrees(s.wi[mi].mean())) if mi.sum() >= 5 else None,
                        imu_heading_deg=float(np.degrees(np.sum(s.wi[mi] * np.minimum(np.diff(s.ti[mi], append=s.ti[mi][-1]),
                                                                                      .2)))) if mi.sum() >= 5 else None,
                        imu_samples=int(mi.sum())))
    return out


def _pose_motion(s, seg, span=4):
    """Pose-based motion over the segment's pose stretch (None when too short or < 70 % covered).

    speed / yaw_dps = medians of the rates over every span of `span` scans inside the stretch, so one
    bad match (a pose jump in foliage) does not set the result; heading_deg / chord = end minus start."""
    p = seg['poses']
    if len(p) <= span or s.t[p[-1]] - s.t[p[0]] < .5 or len(p) < .7 * max(len(seg['idx']), 1):
        return None
    dt = float(s.t[p[-1]] - s.t[p[0]])
    heading = math.degrees(float(s.theta[p[-1]] - s.theta[p[0]])) * s.lidar_sign
    a, b = p[:-span], p[span:]
    dts = s.t[b] - s.t[a]
    ok = (dts > .2) & (dts < .8)
    if ok.sum() < 2:
        return None
    a, b, dts = a[ok], b[ok], dts[ok]
    th = s.theta[a]
    fx, fy = math.cos(s.axis), math.sin(s.axis)
    ux, uy = np.cos(th) * fx - np.sin(th) * fy, np.sin(th) * fx + np.cos(th) * fy
    rates = ((s.pose[b, 0] - s.pose[a, 0]) * ux + (s.pose[b, 1] - s.pose[a, 1]) * uy) / dts
    yaws = np.degrees(s.theta[b] - s.theta[a]) * s.lidar_sign / dts
    return dict(dt=dt, speed=float(np.median(rates)), yaw_dps=float(np.median(yaws)), heading_deg=heading,
                coverage=len(p) / max(len(seg['idx']), 1), spans=int(len(rates)))


def _segment_speed(s, seg):
    val = seg['valid']
    if len(val) >= 5 and len(val) >= .3 * max(len(seg['idx']), 1):
        return float(np.median(s.u[val])), 'samples', s.u[val]
    pm = _pose_motion(s, seg)
    if pm is not None:
        return pm['speed'], 'pose', np.zeros(0)
    return None, None, np.zeros(0)


def straight_bias(series_list):
    ys = [s.yaw_dps[s.valid & s.settled & (s.cls == 'straight_fwd')] for s in series_list]
    y = np.concatenate(ys) if ys else np.zeros(0)
    return dict(samples=int(len(y)), mean_dps=r(float(y.mean()), 3) if len(y) else None)


def c3_speed(series_list):
    by = defaultdict(lambda: dict(segments=[], pooled_all=[], not_moving=0, no_lidar=0))
    for s in series_list:
        base = s.valid & s.settled & (s.cls == 'straight_fwd')
        for d in np.unique(s.req_l[base]):
            by[int(d)]['pooled_all'].append(s.u[base & (s.req_l == d)])
        for seg in command_segments(s, ('straight_fwd',), STEADY_DROP_S, 1.):
            d = int(seg['run']['l'])
            v, method, u = _segment_speed(s, seg)
            if v is None:
                by[d]['no_lidar'] += 1
                continue
            if v < START_SPEED_M_S:
                by[d]['not_moving'] += 1          # a C2 non-start, not a speed sample
                continue
            g = seg['poses'][s.good[seg['poses']]] if len(seg['poses']) else seg['poses']
            path = float(np.sum(np.hypot(np.diff(s.pose[g, 0]), np.diff(s.pose[g, 1])))) if len(g) > 1 else 0.
            by[d]['segments'].append(dict(
                rid=s.rid, label=s.label, t_rel=r(seg['t_from'] - s.motor.t[0], 1), seconds=r(seg['seconds'], 1),
                samples=int(len(u)), speed_m_s=r(v, 3), method=method,
                cv=r(float(u.std() / max(abs(u.mean()), 1e-6)), 3) if len(u) >= 5 else None, path_m=r(path, 2), u=u))
    table = {}
    for d in sorted(by):
        segs = by[d]['segments']
        pooled = np.concatenate([x['u'] for x in segs]) if segs else np.zeros(0)
        allp = np.concatenate(by[d]['pooled_all']) if by[d]['pooled_all'] else np.zeros(0)
        table[str(d)] = dict(
            n_segments=len(segs), seconds=r(sum(x['seconds'] for x in segs), 1), samples=int(len(pooled)),
            segment_speed_p10_p50_p90=q([x['speed_m_s'] for x in segs]), v_p10_p50_p90=q(pooled),
            pose_based_segments=sum(1 for x in segs if x['method'] == 'pose'),
            cv_p50=q([x['cv'] for x in segs], (50,)),
            settled_all_samples=int(len(allp)), settled_all_p10_p50_p90=q(allp),
            not_moving_segments=by[d]['not_moving'], no_lidar_segments=by[d]['no_lidar'],
            labels=sorted({x['label'] for x in segs}),
            segments=[{k: v for k, v in x.items() if k != 'u'} for x in segs])
    return dict(table=table, definition='segment = constant straight request, from the first settled status '
                                        '(MCU duty within 2 %% of the request) + %.1f s to the last status, >= 1 s; '
                                        'speed = median LiDAR velocity sample along the travel axis (>= 5 samples '
                                        'and >= 30 %% coverage) else the pose displacement over the segment; '
                                        'segments slower than %.2f m/s are C2 non-starts. v(duty) = median of the '
                                        'segment speeds. settled_all = every settled sample (LIDAR_RESPONSE rule)'
                                        % (STEADY_DROP_S, START_SPEED_M_S))


def c4_straight(series_list):
    runs = []
    for s in series_list:
        for seg in command_segments(s, ('straight_fwd',), STEADY_DROP_S, 3.):
            pm = _pose_motion(s, seg)
            if pm is None:
                continue
            p = seg['poses']
            g = p[s.good[p]]
            pts = s.pose[g if len(g) >= 10 else p, :2]
            chord = s.pose[p[-1], :2] - s.pose[p[0], :2]
            length = float(np.linalg.norm(chord))
            if length < 1.:
                continue
            nrm = np.array([-chord[1], chord[0]]) / length
            off = float(np.max(np.abs((pts - s.pose[p[0], :2]) @ nrm)))
            windows = window_offsets(pts)
            runs.append(dict(rid=s.rid, label=s.label, duty=int(seg['run']['l']), t_rel=r(seg['t_from'] - s.motor.t[0], 1),
                             seconds=r(pm['dt'], 1), length_m=r(length, 2), pose_coverage=r(pm['coverage'], 2),
                             heading_change_lidar_deg=r(pm['heading_deg'], 2),
                             heading_change_imu_deg=r(seg['imu_heading_deg'], 2),
                             deg_per_m=r(pm['heading_deg'] / length, 3), max_offset_from_chord_m=r(off, 3),
                             window_offsets_m=[r(x, 3) for x in windows],
                             speed_m_s=r(pm['speed'], 3)))
    strict = _c4_strict(series_list)
    return dict(runs=len(runs), summary=_straight_summary(runs), by_duty={
        str(d): _straight_summary([x for x in runs if x['duty'] == d]) for d in sorted({x['duty'] for x in runs})},
        detail=runs, strict=dict(runs=len(strict), summary=_straight_summary(strict), detail=strict,
                                 definition='LIDAR_RESPONSE.json rule: contiguous valid velocity samples (gap < 0.35 s) '
                                            'of >= %d samples, heading = mean sample yaw x duration'
                                            % STRAIGHT_RUN_MIN_SAMPLES),
        definition='run = constant straight request, first settled status + %.1f s to the last status, >= 3 s and '
                   '>= 1 m; heading change from the LiDAR pose (IMU: integrated gravity-projected gyro); offset = '
                   'max distance of the LiDAR track from the chord of the whole run; window offsets = the same over '
                   '%.0f m windows every %.0f m (G5: the 0.15 m criterion came from ~5 m long 10/10 runs, so a 15-20 m '
                   'pass is judged per 5 m window, not over its whole length)' % (STEADY_DROP_S, WANDER_WINDOW_M,
                                                                                 WANDER_STEP_M))


def window_offsets(pts, length_m=WANDER_WINDOW_M, step_m=WANDER_STEP_M):
    """Max distance from the chord over every window of length_m (by track distance) starting each step_m."""
    if len(pts) < 3:
        return []
    d = np.concatenate([[0.], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    out = []
    start = 0.
    while start + length_m <= d[-1] + 1e-9:
        i = int(np.searchsorted(d, start))
        j = int(np.searchsorted(d, start + length_m))
        j = min(j, len(pts) - 1)
        if j - i >= 2:
            chord = pts[j] - pts[i]
            n = float(np.linalg.norm(chord))
            if n > .5 * length_m:
                nrm = np.array([-chord[1], chord[0]]) / n
                out.append(float(np.max(np.abs((pts[i:j + 1] - pts[i]) @ nrm))))
        start += step_m
    return out


def _c4_strict(series_list):
    runs = []
    for s in series_list:
        base = s.valid & s.settled & (s.cls == 'straight_fwd')
        for run in _contiguous(s, base):
            if len(run) < STRAIGHT_RUN_MIN_SAMPLES:
                continue
            dur = float(s.t[run[-1]] - s.t[run[0]])
            dl = float(np.mean(s.yaw_dps[run]) * dur)
            imu = s.imu_dps[run]
            di = float(np.nanmean(imu) * dur) if np.isfinite(imu).any() else None
            pts = s.pose[run, :2]
            chord = pts[-1] - pts[0]
            length = float(np.linalg.norm(chord))
            nrm = np.array([-chord[1], chord[0]]) / max(length, 1e-6)
            off = float(np.max(np.abs((pts - pts[0]) @ nrm)))
            runs.append(dict(rid=s.rid, label=s.label, duty=int(s.req_l[run[0]]), t_rel=r(s.t[run[0]] - s.motor.t[0], 1),
                             seconds=r(dur, 1), samples=int(len(run)), length_m=r(length, 2),
                             heading_change_lidar_deg=r(dl, 2), heading_change_imu_deg=r(di, 2),
                             deg_per_m=r(dl / max(length, .5), 3), max_offset_from_chord_m=r(off, 3),
                             speed_m_s=r(float(np.median(s.u[run])), 3)))
    return runs


def _straight_summary(runs):
    if not runs:
        return dict(runs=0)
    dpm = [x['deg_per_m'] for x in runs]
    return dict(runs=len(runs), seconds=r(sum(x['seconds'] for x in runs), 1),
                metres=r(sum(x['length_m'] for x in runs), 1),
                deg_per_m_p10_p50_p90=q(dpm, nd=2), deg_per_m_mean=r(float(np.mean(dpm)), 3),
                right_fraction=r(float(np.mean([x < 0 for x in dpm])), 2),
                heading_change_lidar_deg_p10_p50_p90=q([x['heading_change_lidar_deg'] for x in runs], nd=2),
                lidar_minus_imu_deg_p10_p50_p90=q([x['heading_change_lidar_deg'] - x['heading_change_imu_deg']
                                                   for x in runs if x['heading_change_imu_deg'] is not None], nd=2),
                max_offset_from_chord_m_p10_p50_p90=q([x['max_offset_from_chord_m'] for x in runs]),
                window_offset_m_p50_p90=q([w for x in runs for w in x.get('window_offsets_m', [])], (50, 90)),
                windows=sum(len(x.get('window_offsets_m', [])) for x in runs))


# --- C7 ------------------------------------------------------------------------------------------
def stop_events(series_list):
    events = []
    for s in series_list:
        m = s.motor
        p = s.pose[:, :2]
        for k in range(1, len(m)):
            if not (tl.request_zero(m, k) and not tl.request_zero(m, k - 1)):
                continue
            l, rr = m.req_l[k - 1], m.req_r[k - 1]
            if not (math.isfinite(l) and math.isfinite(rr) and l > 0 and rr > 0):
                continue
            hard = not (m.state[k] == 'armed_manual')
            t_status = float(m.t[k])
            tags = []
            prev_duty = .5 * (abs(m.rep_l[k - 1]) + abs(m.rep_r[k - 1]))
            if not math.isfinite(prev_duty) or not tl.is_settled(m, k - 1):
                prev_duty = .5 * (l + rr)
                tags.append('previous duty not settled')
            if k >= 2 and not (m.req_l[k - 2] == l and m.req_r[k - 2] == rr):
                tags.append('forward request held < ~0.5-1 s before')
            if k + 1 < len(m) and not tl.request_zero(m, k + 1):
                tags.append('next request not 0 (re-press)')
            if hard:
                lo = float(m.t[k - 1])
                t_dec = _decel_onset(s, lo, t_status)
                if t_dec is not None:
                    t_cmd = min(t_status, max(lo, t_dec - STOP_LATENCY_S))
                    unc = max(t_cmd - lo, .1)
                    basis = 'hard stop: LiDAR deceleration onset - %.1f s (clamped to the status interval)' % STOP_LATENCY_S
                else:
                    t_cmd = .5 * (lo + t_status)
                    unc = .5 * (t_status - lo)
                    basis = 'hard stop: between the last forward status and the first zero status (midpoint)'
            else:
                t_cmd, basis = tl.fade_start(m, k, prev_duty)
                unc = .1
                if t_cmd is None:
                    # fade already complete at the first zero status: the request changed after the previous
                    # status and at least prev_duty / 100 s before this one (low duties fade in < 0.5 s)
                    lo, hi = float(m.t[k - 1]), t_status - prev_duty / tl.RAMP_PCT_PER_S
                    if hi > lo and tl.reported_zero(m, k):
                        t_cmd, unc = .5 * (lo + hi), .5 * (hi - lo)
                        basis = 'fade complete at the first zero status: midpoint of (previous status, t - duty/100)'
                    else:
                        tags.append('fade start not recoverable')
            i0 = int(np.searchsorted(s.t, t_status))
            if i0 < 5 or i0 + 40 >= len(s.t) or not s.good[i0 - 5:i0 + 40].mean() > .8:
                tags.append('LiDAR quality/coverage')
                if i0 < 5 or i0 + 40 >= len(s.t):
                    events.append(dict(rid=s.rid, label=s.label, kind='hard_stop' if hard else 'release',
                                       prev_req=[int(l), int(rr)], t_rel=r(t_status - m.t[0], 1), tags=tags, clean=False))
                    continue
            axis = s.axis_at(i0) if np.isfinite(s.theta[i0]) else None
            ev = dict(rid=s.rid, label=s.label, kind='hard_stop' if hard else 'release', prev_req=[int(l), int(rr)],
                      straight=bool(l == rr), prev_duty_pct=r(prev_duty, 1), t_rel=r(t_status - m.t[0], 1),
                      basis=basis, t_cmd_uncertainty_s=r(unc, 2), tags=tags)
            if axis is None:
                ev['tags'].append('no pose at the stop')
                ev['clean'] = False
                events.append(ev)
                continue
            # LIDAR_RESPONSE.json rule: from the first zero status (only with >= 80 % good scans around it)
            stop_i = _stop_index(s, i0, i0 + 40) if 'LiDAR quality/coverage' not in tags else None
            if stop_i is not None:
                ev['from_first_zero_status'] = dict(distance_m=r(float((p[stop_i] - p[i0]) @ axis), 3),
                                                   time_s=r(float(s.t[stop_i] - t_status), 2))
                pre = (p[i0] - p[i0 - 5]) @ axis / (s.t[i0] - s.t[i0 - 5])
                ev['from_first_zero_status']['pre_speed_m_s'] = r(float(pre), 3)
            if t_cmd is not None:
                j = int(np.searchsorted(s.t, t_cmd))
                stop_j = _stop_index(s, j, i0 + 40)
                if j >= 5 and stop_j is not None and np.all(np.isfinite(p[[j, j - 5], 0])):
                    pr = np.array([np.interp(t_cmd, s.t, p[:, 0]), np.interp(t_cmd, s.t, p[:, 1])])
                    v_cmd = float((p[j] - p[j - 5]) @ axis / (s.t[j] - s.t[j - 5]))
                    ev['speed_at_command_m_s'] = r(v_cmd, 3)
                    pre = (s.t >= t_cmd - 1.2) & (s.t <= t_cmd - .2) & s.valid
                    v_pre = float(np.median(s.u[pre])) if pre.sum() >= 3 else None
                    ev['speed_before_m_s'] = r(v_pre, 3)
                    if v_pre is None or abs(v_cmd - v_pre) > max(.15, .25 * abs(v_pre)):
                        ev['tags'].append('speed not steady before the command')
                    ev['distance_m'] = r(float((p[stop_j] - pr) @ axis), 3)
                    ev['time_s'] = r(float(s.t[stop_j] - t_cmd), 2)
                    if unc > .15:            # command time from a status interval: report the early bound too
                        early = np.array([np.interp(t_cmd - unc, s.t, p[:, 0]), np.interp(t_cmd - unc, s.t, p[:, 1])])
                        ev['distance_upper_bound_m'] = r(float((p[stop_j] - early) @ axis), 3)
            ev['clean'] = not ev['tags'] and 'distance_m' in ev
            events.append(ev)
    return events


STOP_LATENCY_S = .1             # [assumed] STOP -> bridge -> serial -> MCU duty 0 -> motor torque change


def _decel_onset(s, lo, hi, lag_m=.03):
    """First scan after lo whose LiDAR along-track position lags the constant-speed extrapolation of the
    second before lo by more than lag_m (not a centred window, so it is not early). None if not found."""
    ref = np.nonzero((s.t >= lo - 1.) & (s.t <= lo) & (s.chain >= 0))[0]
    if len(ref) < 5:
        return None
    ref = ref[s.chain[ref] == s.chain[ref[-1]]]
    if len(ref) < 5 or not np.isfinite(s.theta[ref[-1]]):
        return None
    axis = s.axis_at(ref[-1])
    origin = s.pose[ref[-1], :2]
    along = (s.pose[ref, :2] - origin) @ axis
    b, a = np.polyfit(s.t[ref] - lo, along, 1)
    if b < .1:
        return None
    nxt = np.nonzero((s.t > lo) & (s.t <= hi + .5) & (s.chain == s.chain[ref[-1]]))[0]
    for i in nxt:
        lag = (a + b * (s.t[i] - lo)) - float((s.pose[i, :2] - origin) @ axis)
        if lag > lag_m:
            return float(s.t[i])
    return None


def _stop_index(s, start, stop):
    p = s.pose[:, :2]
    for j in range(max(start, 0), min(stop, len(s.t) - 6)):
        if np.all(np.isfinite(p[j:j + 6, 0])) and s.chain[j] == s.chain[j + 5] and s.chain[j] >= 0:
            if np.linalg.norm(p[j + 5] - p[j]) / (s.t[j + 5] - s.t[j]) < STOP_SPEED_M_S:
                return j
    return None


def model_distance(v, reaction_s, onset_s, decel):
    v = max(0., v)
    return v * (reaction_s + onset_s) + v * v / (2. * decel)


def c7_stop(events, model):
    out = dict(events=len(events), kinds={})
    for kind in ('release', 'hard_stop'):
        es = [e for e in events if e.get('kind') == kind]
        clean = [e for e in es if e.get('clean')]
        d = [e['distance_m'] for e in clean]
        t = [e['time_s'] for e in clean]
        v = [e['speed_at_command_m_s'] for e in clean]
        ratio = [e['distance_m'] / max(model_distance(e['speed_at_command_m_s'], **model), 1e-6) for e in clean
                 if e['speed_at_command_m_s'] > .05]
        first = [e['from_first_zero_status'] for e in es if 'from_first_zero_status' in e and e.get('straight')]
        out['kinds'][kind] = dict(
            events=len(es), clean=len(clean), distance_m_p10_p50_p90=q(d), distance_m_max=r(max(d), 3) if d else None,
            time_s_p10_p50_p90=q(t, nd=2), speed_at_command_m_s_p10_p50_p90=q(v),
            distance_over_model_p50_p90=q(ratio, (50, 90)),
            upper_bound_distance_m_p90=q([e.get('distance_upper_bound_m') for e in clean], (90,)),
            from_first_zero_status=dict(events=len(first), distance_m_p10_p50_p90=q([x['distance_m'] for x in first]),
                                        time_s_p10_p50_p90=q([x['time_s'] for x in first], nd=2),
                                        pre_speed_m_s_p10_p50_p90=q([x['pre_speed_m_s'] for x in first])))
    out['model'] = model
    out['detail'] = events
    return out
