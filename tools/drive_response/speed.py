"""PW-03: duty -> steady straight speed, per segment, with three references and a grade per table cell.

Segment rule (fixed before looking at the numbers, from the 10/3 GNSS/SLAM tables and todo PW-03):
  - consecutive ~2 Hz motor statuses with the same request, left == right != 0 (straight), every status settled
    (MCU-reported duty within 2 % of the request on both sides), state armed_manual, no status gap > 1.5 s;
  - settled span (first to last settled status) >= MIN_SETTLED_S = 2.0 s; the first DROP_S = 1.0 s is dropped,
    so the measuring window is [first status + 1 s, last status] (conservative: the request may have changed up
    to 0.5 s after the last status, and settled up to 0.5 s before the first one);
  - straight check: |median LiDAR yaw rate| < 2 deg/s in the window (IMU yaw when LiDAR has no samples).
References per segment (each a median over the window):
  LiDAR   per-scan centred-difference speed along the travel axis (lidar_response.py rule); needs >= MIN_LIDAR_N
          valid samples and >= MIN_LIDAR_COVER of the scans in the window (main reference).
  GNSS    receiver Doppler ground speed of the antenna, one value per receiver epoch (i_tow_ms), >= 2 epochs.
  Carto   offline Cartographer node path length / time between the first and last node in the window, >= 3 nodes
          (10/10 five recordings and 10/3 072530 only).
Statistics per table cell: median of the segment medians, p10 / p90 of the segment medians, number of segments,
summed window seconds. Grade (LiDAR column): 측정 >= 5 segments, 참고 1-4 segments, 보간 no segment but measured
duties on both sides (linear between the neighbouring rows), 외삽 outside the measured duty range (straight
line through the measured rows, one point per duty).
"""
import math

import numpy as np

import common
import motion as mo
from field_calib import timeline as tl

MIN_SETTLED_S = 2.
DROP_S = 1.
MAX_STATUS_GAP_S = 1.5
SETTLED_TOL = 2.
STRAIGHT_YAW_DPS = 2.
MIN_LIDAR_N = 5
MIN_LIDAR_COVER = .25
MIN_GNSS_EPOCHS = 2
MIN_CARTO_NODES = 3
MEASURED_MIN_SEGMENTS = 5
GRID = list(range(25, 101, 5))

GROUP_ROAD = '도로·포장(10/3)'
GROUP_ORCH3 = '과수원(10/3, 열 구분 없음)'
GROUP_IN = '과수원 열 안(10/10)'
GROUP_OUT = '과수원 그 밖(10/10)'
GROUP_UNK = '과수원 위치 모름(10/10)'
GROUPS = [GROUP_ROAD, GROUP_ORCH3, GROUP_IN, GROUP_OUT, GROUP_UNK]


def _ok_status(m, k):
    if not tl.is_settled(m, k, SETTLED_TOL):
        return False
    l, r = m.req_l[k], m.req_r[k]
    if not (l == r and l != 0):
        return False
    if m.state[k] != 'armed_manual' or m.stale[k]:
        return False
    return True


def settled_runs(m):
    """[(k0, k1)] maximal runs of settled straight statuses with one request (any length)."""
    runs = []
    n = len(m)
    k = 0
    while k < n:
        if not _ok_status(m, k):
            k += 1
            continue
        j = k
        while (j + 1 < n and _ok_status(m, j + 1) and m.req_l[j + 1] == m.req_l[k]
               and m.t[j + 1] - m.t[j] <= MAX_STATUS_GAP_S):
            j += 1
        runs.append((k, j))
        k = j + 1
    return runs


def settled_seconds(m):
    """Inventory-style time per settled straight duty: each settled status counts until the next status (<= 1.5 s)."""
    out = {}
    for k in range(len(m) - 1):
        if not _ok_status(m, k):
            continue
        d = int(round(m.req_l[k]))
        out[d] = out.get(d, 0.) + min(max(m.t[k + 1] - m.t[k], 0.), MAX_STATUS_GAP_S)
    return out


def build_segments(rec, mot, locator, gnss_epochs, imu_t, imu_w):
    m = rec.motor
    segs = []
    for k0, k1 in settled_runs(m):
        ta, tb = float(m.t[k0]), float(m.t[k1])
        span = tb - ta
        duty = int(round(m.req_l[k0]))
        seg = dict(rid=rec.rid, short=rec.short, day=rec.day, record_ground=rec.ground, duty=duty,
                   direction='fwd' if duty > 0 else 'rev', t_first=ta, t_last=tb, settled_s=span,
                   n_status=k1 - k0 + 1, used=span >= MIN_SETTLED_S)
        if not seg['used']:
            segs.append(seg)
            continue
        w0, w1 = ta + DROP_S, tb
        seg['w0'], seg['w1'], seg['window_s'] = w0, w1, w1 - w0
        kst = rec.to_kst(.5 * (w0 + w1))
        seg['kst'] = kst.strftime('%H:%M:%S') if kst else None
        seg['kst_hours'] = (kst.hour + kst.minute / 60. + kst.second / 3600.) if kst else None
        # LiDAR
        if mot is not None:
            inwin = (mot.t >= w0) & (mot.t <= w1)
            sel = inwin & mot.valid
            n_scan = int(inwin.sum())
            u = mot.u[sel] * (1. if duty > 0 else 1.)
            seg['lidar_n'] = int(sel.sum())
            seg['lidar_cover'] = float(sel.sum()) / n_scan if n_scan else 0.
            if seg['lidar_n'] >= MIN_LIDAR_N and seg['lidar_cover'] >= MIN_LIDAR_COVER:
                seg['lidar_v'] = float(np.median(u))
                seg['lidar_p10'] = float(np.percentile(u, 10))
                seg['lidar_p90'] = float(np.percentile(u, 90))
                seg['lidar_yaw_dps'] = float(np.median(mot.yaw_dps[sel]))
                seg['lidar_lat'] = float(np.median(mot.lat[sel]))
        # IMU yaw
        if len(imu_t):
            si = (imu_t >= w0) & (imu_t <= w1)
            if si.sum() >= 5:
                seg['imu_yaw_dps'] = float(np.median(imu_w[si]))
        yaw = seg.get('lidar_yaw_dps', seg.get('imu_yaw_dps'))
        seg['straight_ok'] = True if yaw is None else abs(yaw) < STRAIGHT_YAW_DPS
        # GNSS
        vals = [v for (t, v) in gnss_epochs if w0 <= t <= w1]
        seg['gnss_n'] = len(vals)
        if len(vals) >= MIN_GNSS_EPOCHS:
            seg['gnss_v'] = float(np.median(vals))
        # Cartographer
        if rec.traj is not None:
            tt = rec.traj['t_host']
            order = np.argsort(tt, kind='stable')
            tt = tt[order]
            sel = (tt >= w0) & (tt <= w1)
            if sel.sum() >= MIN_CARTO_NODES:
                x = rec.traj['x_map'][order][sel]
                y = rec.traj['y_map'][order][sel]
                ts = tt[sel]
                if ts[-1] - ts[0] >= .5:
                    seg['carto_v'] = float(np.hypot(np.diff(x), np.diff(y)).sum() / (ts[-1] - ts[0]))
                    seg['carto_n'] = int(sel.sum())
        # surface group
        if rec.day == '1003':
            seg['group'] = GROUP_ROAD if rec.ground == 'road' else (GROUP_ORCH3 if rec.ground == 'orchard' else None)
        else:
            if locator is None:
                seg['group'] = GROUP_UNK
                seg['inrow_known'] = 0.
            else:
                times = np.arange(w0, w1 + 1e-9, .1)
                known, inside = locator.at(times)
                kf = float(known.mean()) if len(times) else 0.
                seg['inrow_known'] = kf
                seg['inrow_frac'] = float(inside[known].mean()) if known.any() else None
                if kf >= .5:
                    seg['group'] = GROUP_IN if seg['inrow_frac'] >= .5 else GROUP_OUT
                else:
                    seg['group'] = GROUP_UNK
        segs.append(seg)
    return segs


def gnss_epochs(rec):
    """[(receipt time, speed)] one per receiver epoch, valid fixes only (positions are not read)."""
    seen = set()
    out = []
    for g in rec.gnss:
        if not g.get('valid') or g.get('speed_mps') is None or g.get('received_at') is None:
            continue
        key = g.get('i_tow_ms')
        if key is not None:
            if key in seen:
                continue
            seen.add(key)
        out.append((float(g['received_at']), float(g['speed_mps'])))
    return out


# --- tables ------------------------------------------------------------------------------------------
def main_segments(segs):
    """Segments that enter the speed tables: used, forward, straight."""
    return [s for s in segs if s.get('used') and s['direction'] == 'fwd' and s.get('straight_ok', True)]


def cell(segs, key):
    vals = [s[key] for s in segs if s.get(key) is not None]
    secs = sum(s['window_s'] for s in segs if s.get(key) is not None)
    if not vals:
        return dict(n=0, seconds=0., v=None, p10=None, p90=None, records=[])
    return dict(n=len(vals), seconds=secs, v=float(np.median(vals)), p10=float(np.percentile(vals, 10)),
                p90=float(np.percentile(vals, 90)),
                records=sorted({s['short'] for s in segs if s.get(key) is not None}))


def grade_of(n):
    if n >= MEASURED_MIN_SEGMENTS:
        return '측정'
    if n >= 1:
        return '참고'
    return None


def line_fit(points):
    """Least squares v = a + b * duty through (duty, v) points; (a, b, duty at v = 0)."""
    if len(points) < 2:
        return None
    x = np.array([p[0] for p in points], dtype=float)
    y = np.array([p[1] for p in points], dtype=float)
    b, a = np.polyfit(x, y, 1)
    return dict(a=float(a), b=float(b), duty_at_zero=float(-a / b) if b else math.nan, points=len(points))


def ref_cell(segs):
    """Cell value: LiDAR when any segment has it, else GNSS, else Cartographer (the reference is named)."""
    out = dict(lidar=cell(segs, 'lidar_v'), gnss=cell(segs, 'gnss_v'), carto=cell(segs, 'carto_v'))
    for key, name in (('lidar', 'LiDAR'), ('gnss', 'GNSS'), ('carto', 'Cartographer')):
        c = out[key]
        if c['n']:
            out.update(ref=name, v=c['v'], p10=c['p10'], p90=c['p90'], n=c['n'], seconds=c['seconds'],
                       grade=grade_of(c['n']), records=c['records'])
            return out
    out.update(ref=None, v=None, n=0, seconds=0., grade=None, records=[])
    return out


def speed_table(segs, grid=GRID):
    """Rows for every duty with segments plus the grid duties (보간 / 외삽). Returns (rows, fit, measured duties).
    Measured rows use the LiDAR median; a duty whose segments have no LiDAR value uses GNSS (named in 'ref')."""
    duties = sorted({s['duty'] for s in segs})
    rows = {}
    for d in duties:
        c = ref_cell([s for s in segs if s['duty'] == d])
        if c['n'] == 0:
            continue
        rows[d] = dict(duty=d, **c)
    measured = sorted(rows)
    fit = line_fit([(d, rows[d]['v']) for d in measured])
    for d in grid:
        if d in rows:
            continue
        if measured and measured[0] < d < measured[-1]:
            lo = max(x for x in measured if x < d)
            hi = min(x for x in measured if x > d)
            f = (d - lo) / (hi - lo)
            v = rows[lo]['v'] + f * (rows[hi]['v'] - rows[lo]['v'])
            rows[d] = dict(duty=d, grade='보간', v=v, n=0, seconds=0., ref=None,
                           basis='%d %%와 %d %% 사이 직선' % (lo, hi))
        elif fit is not None:
            v = fit['a'] + fit['b'] * d
            rows[d] = dict(duty=d, grade='외삽', v=max(v, 0.), n=0, seconds=0., ref=None,
                           basis='측정 duty 직선 맞춤(v = %.4f·duty %+.3f)' % (fit['b'], fit['a']))
    return [rows[d] for d in sorted(rows)], fit, measured


def missing_ranges(segs, lo=0, hi=100):
    """Integer duty ranges [first, last] (inclusive) without any used forward segment (grey bands, '외삽' rows)."""
    have = sorted({s['duty'] for s in segs if lo <= s['duty'] <= hi})
    out = []
    prev = lo - 1
    for d in have + [hi + 1]:
        if d - prev > 1:
            out.append((prev + 1, d - 1))
        prev = d
    return out


def sample_level_reference(motions):
    """lidar_response.py settled_response: settled straight samples of the whole day, per request (no segments)."""
    out = {}
    for duty in (100, 95, 90, 88, 70, 58, 55, 45, 40):
        vals = [m.u[m.valid & m.settled & (m.req_l == duty) & (m.req_r == duty)] for m in motions]
        v = np.concatenate(vals) if vals else np.zeros(0)
        if len(v):
            out[duty] = dict(samples=int(len(v)), p10_p50_p90=[round(float(np.percentile(v, p)), 3) for p in (10, 50, 90)])
    return out
