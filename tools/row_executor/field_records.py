"""Read-only loaders for the real 2026-10-10 manual orchard records and a human-driving reference.

Sources (never written):
  orchard-realdata-audit/20261010-field-manual/lidar/<rid>.scans.npz          ranges (uint16 mm, 0 = none),
      bag_time_ns (host receipt), header_ns, beam geometry
  <drive extract folder>/<rid>.drive.json.gz (sim_paths 'field_manual_1010_drive')  imu rows
      [receipt_ns, header_ns, gx, gy, gz, ax, ay, az, qx, qy, qz, qw], motor status rows (~2 Hz)
  orchard-realdata-audit/20261010-field-manual/lidar/<rid>.lidar_motion.npz   offline point-to-line ICP
      (independent of IMU/PWM; a REFERENCE for the human motion, not truth and never fed to the estimator)

The reference gives per-tick forward speed along the travel axis, yaw rate, heading and travelled distance,
which define what the human did (still / forward / turning). Only the scans and IMU go to the estimator.
"""
import gzip
import json
import math
import os
from pathlib import Path

import numpy as np

try:
    from . import sim_paths as _paths          # imported as row_executor.field_records
except ImportError:
    import sim_paths as _paths                 # body_mask_fit.py puts this folder on sys.path

# The folder holding lidar/<rid>.scans.npz and lidar/<rid>.lidar_motion.npz (e.g. a copy on the Jetson for the V5
# timing replay): sim_paths 'field_manual_1010' = UBTS_RD_1010 or the path config file. The drive extract files
# <rid>.drive.json.gz (IMU and motor rows) come from their own location: sim_paths 'field_manual_1010_drive' =
# UBTS_RD_1010_DRIVE or the path config file (the repository does not name that data folder). No built-in path
# (steering branch, docs/steering/EXECUTOR_IMPORT_KO.md). None when not configured: load_record then raises a clear
# FileNotFoundError.
BASE_1010 = _paths.optional_path('field_manual_1010')
DRIVE_1010 = _paths.optional_path('field_manual_1010_drive')
RECORDS_1010 = ('20261010T035952Z_both', '20261010T040927Z_both', '20261010T042200Z_both',
                '20261010T042301Z_both', '20261010T045553Z_both', '20261010T051127Z_both')
TRAVEL_AXIS_DEG = -179.08
GOOD = dict(inlier=.5, rms=.08, cond=60.)     # LIDAR_RESPONSE.json quality_thresholds
# Reference-only override (never reaches the estimator): the 10/03 ICP reference has inlier p50 ~0.5.
if os.environ.get('ROW_REPLAY_REF_INLIER'):
    GOOD['inlier'] = float(os.environ['ROW_REPLAY_REF_INLIER'])
HALF = 2                                       # +-2 scans velocity window (lidar_response.py)

FORWARD_M_S = .10          # human "driving forward"
STILL_M_S = .03            # human still (reference)
STILL_YAW_DEG_S = 1.5
SEGMENT_BRIDGE_S = 1.      # non-forward gaps up to this long stay inside one forward segment
STANDING_STILL_S = 1.      # a run starts at forward onset after >= this much stillness ...
STANDING_GAP_S = 1.        # ... that ended at most this long before the onset (ramp-up)
INTERP_GAP_S = 1.          # reference samples farther apart than this are not interpolated


def load_record(rid, base=BASE_1010, drive_dir=DRIVE_1010):
    base = Path(base) if base is not None else _paths.configured_path('field_manual_1010', 'dir')
    drive_dir = (Path(drive_dir) if drive_dir is not None
                 else _paths.configured_path('field_manual_1010_drive', 'dir'))
    z = np.load(base / 'lidar' / f'{rid}.scans.npz')
    drive = json.loads(gzip.open(drive_dir / f'{rid}.drive.json.gz', 'rt', encoding='utf-8').read())
    motion = np.load(base / 'lidar' / f'{rid}.lidar_motion.npz')
    imu = np.array(drive['imu'], dtype=np.float64)
    rec = dict(rid=rid, ranges_mm=z['ranges_mm'], t_scan=z['bag_time_ns'] / 1e9, t_header=z['header_ns'] / 1e9,
               angle_min=float(z['angle_min']), increment=float(z['angle_increment']),
               range_min=float(z['range_min']), range_max=float(z['range_max']),
               scan_source_sha256=str(z['source_sha256']), imu=imu, motor=drive['motor'],
               drive_source_sha256=drive.get('source_sha256'))
    rec['ref'] = reference_motion(motion)
    return rec


def reference_motion(motion):
    t, pose, qual = motion['t'], motion['pose'], motion['quality']
    good = (np.isfinite(pose[:, 0]) & (qual[:, 0] >= GOOD['inlier']) & (qual[:, 1] <= GOOD['rms'])
            & (qual[:, 3] <= GOOD['cond']))
    finite = np.isfinite(pose[:, 2])
    theta = np.unwrap(np.where(finite, pose[:, 2], 0.))
    axis = math.radians(TRAVEL_AXIS_DEG)
    vf = np.full(len(t), np.nan)
    w = np.full(len(t), np.nan)
    for i in range(HALF, len(t) - HALF):
        a, b = i - HALF, i + HALF
        if not (good[a] and good[b] and good[i]):
            continue
        dt = t[b] - t[a]
        if not .25 <= dt <= .6:
            continue
        c, s = math.cos(theta[i]), math.sin(theta[i])
        d = pose[b, :2] - pose[a, :2]
        vx, vy = (c * d[0] + s * d[1]) / dt, (-s * d[0] + c * d[1]) / dt
        vf[i] = vx * math.cos(axis) + vy * math.sin(axis)
        w[i] = (theta[b] - theta[a]) / dt
    return dict(t=t, vf=vf, w=w, theta=np.where(finite, theta, np.nan), good=good)


def _interp_gapped(t_src, v_src, t_dst, max_gap=INTERP_GAP_S):
    ok = np.isfinite(v_src)
    ts, vs = t_src[ok], v_src[ok]
    out = np.full(len(t_dst), np.nan)
    if len(ts) < 2:
        return out
    j = np.searchsorted(ts, t_dst)
    inside = (j > 0) & (j < len(ts))
    jj = np.clip(j, 1, len(ts) - 1)
    gap = ts[jj] - ts[jj - 1]
    use = inside & (gap <= max_gap)
    out[use] = np.interp(t_dst[use], ts, vs)
    exact = (j < len(ts)) & (np.abs(ts[np.clip(j, 0, len(ts) - 1)] - t_dst) < 1e-9)
    out[exact] = vs[np.clip(j, 0, len(ts) - 1)][exact]
    return out


def motor_rows_at(motor, t):
    """Index of the latest motor status row received at or before each t (-1 = none)."""
    mt = np.array([row['received_at'] for row in motor], dtype=np.float64)
    return np.searchsorted(mt, t, side='right') - 1


def human_ticks(rec, t_grid):
    """Per-tick human reference: forward speed, yaw rate, heading, distance, flags."""
    ref = rec['ref']
    vf = _interp_gapped(ref['t'], ref['vf'], t_grid)
    w = _interp_gapped(ref['t'], ref['w'], t_grid)
    theta = _interp_gapped(ref['t'], ref['theta'], t_grid)
    dt = np.diff(t_grid, prepend=t_grid[0])
    dist = np.cumsum(np.where(np.isfinite(vf), np.clip(vf, 0., None), 0.) * dt)
    k = motor_rows_at(rec['motor'], t_grid)
    motor = rec['motor']
    req_l = np.array([np.nan if i < 0 or motor[i]['req_l'] is None else motor[i]['req_l'] for i in k], dtype=float)
    req_r = np.array([np.nan if i < 0 or motor[i]['req_r'] is None else motor[i]['req_r'] for i in k], dtype=float)
    pins_zero = np.array([i >= 0 and all((motor[i]['rep'].get(p) or 0) == 0 for p in ('D8', 'D5', 'D3', 'D0'))
                          for i in k])
    moving_flag = np.array([i >= 0 and bool(motor[i].get('moving')) for i in k])
    forward = np.isfinite(vf) & (vf > FORWARD_M_S)
    still_ref = np.isfinite(vf) & (np.abs(vf) < STILL_M_S) & np.isfinite(w) & (np.abs(np.degrees(w)) < STILL_YAW_DEG_S)
    still_motor = ~np.isfinite(vf) & (req_l == 0) & (req_r == 0) & pins_zero & ~moving_flag
    return dict(t=t_grid, vf=vf, w=w, theta=theta, dist=dist, forward=forward, still=still_ref | still_motor,
                req_l=req_l, req_r=req_r)


def forward_segments(h):
    """Maximal forward-driving intervals (non-forward gaps <= SEGMENT_BRIDGE_S bridged) with start type."""
    t, fwd, still = h['t'], h['forward'], h['still']
    idx = np.nonzero(fwd)[0]
    segments = []
    if not len(idx):
        return segments
    start = last = idx[0]
    for i in idx[1:]:
        if t[i] - t[last] > SEGMENT_BRIDGE_S:
            segments.append((start, last))
            start = i
        last = i
    segments.append((start, last))
    out = []
    for s, e in segments:
        # standing start: a still interval >= STANDING_STILL_S ending <= STANDING_GAP_S before the onset
        k = s - 1
        while k >= 0 and not still[k] and t[s] - t[k] <= STANDING_GAP_S:
            k -= 1
        still_dur, still_end = 0., None
        if k >= 0 and still[k]:
            still_end = k
            while k >= 0 and still[k]:
                k -= 1
            still_dur = t[still_end] - t[k + 1] + (t[1] - t[0])
        fwd_ticks = int(fwd[s:e + 1].sum())
        out.append(dict(i0=int(s), i1=int(e), t_on=float(t[s]), t_off=float(t[e]) + (t[1] - t[0]),
                        standing=bool(still_dur >= STANDING_STILL_S), still_s=float(still_dur),
                        forward_s=fwd_ticks * float(t[1] - t[0]),
                        dist_m=float(h['dist'][e] - h['dist'][s])))
    return out


def straight_ahead_m(h, i, limit_deg=20., horizon_m=30.):
    """Reference distance the human kept driving from tick i before turning > limit_deg, stopping or reversing."""
    theta0 = h['theta'][i]
    if not math.isfinite(theta0):
        return None
    d0 = h['dist'][i]
    limit = math.radians(limit_deg)
    n = len(h['t'])
    j = i
    while j + 1 < n:
        j += 1
        if h['dist'][j] - d0 >= horizon_m:
            return horizon_m
        th = h['theta'][j]
        if math.isfinite(th) and abs(th - theta0) > limit:
            break
        v = h['vf'][j]
        if h['still'][j] or (math.isfinite(v) and v < -.05):
            break
    return float(h['dist'][j] - d0)
