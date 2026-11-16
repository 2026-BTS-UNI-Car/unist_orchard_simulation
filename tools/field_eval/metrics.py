"""Metrics of one run against the reference centre line (README 7, docs/SYSTEM_REFERENCE_KO.md 8).

measure() is follow_matrix.measure / gsim_follow.measure in the row frame of the reference map, on any sampled
trajectory (the LiDAR reference of a field run, or the SIM truth for the verification):
  drive interval   [t_a, t_b] = first and last time with a drive command > 0 (executor tick T, or the trace
                   ctrl_throttle_pct column); the samples inside it
  lateral          d of the footprint centre (left positive); heading psi = yaw - row axis
  RMSE / p95       sqrt(mean d^2), 95 % quantile of |d| (numpy linear), heading RMSE, over the samples after settle_m
                   of travelled path (0 = all; 6.0 for offset starts); max |d| and max |psi| over the whole interval
  row end e        along(rest) + front bumper * cos(psi rest) - row end s_E; rest = the last sample (or the last
                   sample at or before t_b + rest_window_s when that is given)
  speed            max and mean of the speed samples inside the interval
estimate_error() compares the executor's online lateral / heading (tick lat, -head) with a reference at the tick
times (linear interpolation between reference samples no more than max_gap_s apart).
"""
from __future__ import annotations

import math

import numpy as np

LIMITS = dict(lateral_rmse_m=.20, lateral_p95_m=.35, lateral_max_m=.50, heading_rmse_deg=5., heading_max_deg=15.,
              row_end_error_m=1.0, speed_max_m_s=.50)      # steer_follow_v1 NOMINAL (follow_matrix.CRITERIA)


def path_speed(t, x, y, yaw, half_window_s=.5):
    """Speed [m/s] along the heading from a local linear fit of x(t), y(t) over +-half_window_s."""
    t, x, y, yaw = (np.asarray(v, dtype=float) for v in (t, x, y, yaw))
    out = np.full(len(t), np.nan)
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(t)
    idx = np.flatnonzero(ok)
    if len(idx) < 3:
        return out
    tt = t[idx]
    lo = np.searchsorted(tt, tt - half_window_s, side='left')
    hi = np.searchsorted(tt, tt + half_window_s, side='right')
    for k, i in enumerate(idx):
        sel = idx[lo[k]:hi[k]]
        if len(sel) < 3:
            continue
        ts = t[sel] - t[i]
        den = float(np.sum((ts - ts.mean()) ** 2))
        if den <= 0:
            continue
        vx = float(np.sum((ts - ts.mean()) * (x[sel] - x[sel].mean()))) / den
        vy = float(np.sum((ts - ts.mean()) * (y[sel] - y[sel].mean()))) / den
        out[i] = vx * math.cos(yaw[i]) + vy * math.sin(yaw[i])
    return out


def measure(refmap, t, x, y, yaw, v, drive, settle_m=0., front_bumper_m=.61, valid=None, rest_window_s=None):
    t, x, y, yaw, v = (np.asarray(a, dtype=float) for a in (t, x, y, yaw, v))
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(yaw)
    if valid is not None:
        ok &= np.asarray(valid, dtype=bool)
    out = dict(samples=int(np.count_nonzero(ok)), settle_m=float(settle_m), drove=False)
    if not np.any(ok):
        return out
    s, d, psi = refmap.to_row_arrays(x, y, yaw)
    g = refmap.geometry
    rest_sel = ok.copy()
    if drive is not None and rest_window_s is not None:
        rest_sel &= t <= drive[1] + rest_window_s
    r = int(np.flatnonzero(rest_sel)[-1]) if np.any(rest_sel) else int(np.flatnonzero(ok)[-1])
    out.update(rest_t_s=round(float(t[r]), 3), rest_along_m=round(float(s[r]), 3), rest_lateral_m=round(float(d[r]), 4),
               rest_heading_deg=round(math.degrees(float(psi[r])), 3),
               row_end_error_m=round(float(s[r] + front_bumper_m * math.cos(psi[r]) - g['row_end_s']), 3))
    first = int(np.flatnonzero(ok)[0])
    out.update(start_along_m=round(float(s[first]), 3), start_lateral_m=round(float(d[first]), 4),
               start_heading_deg=round(math.degrees(float(psi[first])), 3))
    if drive is None:
        return out
    t_a, t_b = drive
    idx = np.flatnonzero(ok & (t >= t_a) & (t <= t_b))
    if len(idx) < 2:
        return out
    a, b = idx[0], idx[-1]
    path = np.concatenate(([0.], np.cumsum(np.hypot(np.diff(x[idx]), np.diff(y[idx])))))
    k = int(np.searchsorted(path, float(settle_m)))
    lat, head = d[idx], psi[idx]
    speed = v[idx]
    speed_ok = speed[np.isfinite(speed)]
    out.update(drove=True, drive_start_s=round(float(t_a), 3), drive_stop_s=round(float(t_b), 3),
               samples_in_drive=int(len(idx)), first_sample_s=round(float(t[a]), 3), last_sample_s=round(float(t[b]), 3),
               distance_m=round(float(path[-1]), 2), lateral_max_m=round(float(np.max(np.abs(lat))), 4),
               heading_max_deg=round(math.degrees(float(np.max(np.abs(head)))), 3),
               speed_mean_m_s=None if not len(speed_ok) else round(float(np.mean(speed_ok)), 3),
               speed_max_m_s=None if not len(speed_ok) else round(float(np.max(speed_ok)), 3))
    if k < len(idx) - 1:
        tail_d, tail_h = lat[k:], head[k:]
        out.update(lateral_rmse_m=round(float(np.sqrt(np.mean(tail_d ** 2))), 4),
                   lateral_p95_m=round(float(np.percentile(np.abs(tail_d), 95)), 4),
                   heading_rmse_deg=round(math.degrees(float(np.sqrt(np.mean(tail_h ** 2)))), 3))
    return out


def against_limits(m):
    """The steer_follow_v1 limits next to the values (information, not a registered judgement)."""
    out = {}
    for key, limit in LIMITS.items():
        value = m.get(key)
        if value is None:
            out[key] = dict(value=None, limit=limit, within=None)
        elif key == 'row_end_error_m':
            out[key] = dict(value=value, limit=[-limit, limit], within=abs(value) <= limit)
        else:
            out[key] = dict(value=value, limit=limit, within=value <= limit)
    return out


def interp_valid(t_ref, values, valid, t_query, max_gap_s=.25):
    """values at t_query by linear interpolation between valid reference samples <= max_gap_s apart; NaN else."""
    t_ref = np.asarray(t_ref, float)[valid]
    values = np.asarray(values, float)[valid]
    out = np.full(len(t_query), np.nan)
    if len(t_ref) < 2:
        return out
    j = np.searchsorted(t_ref, t_query)
    inside = (j > 0) & (j < len(t_ref))
    jj = np.clip(j, 1, len(t_ref) - 1)
    gap = t_ref[jj] - t_ref[jj - 1]
    inside &= gap <= max_gap_s
    w = np.where(gap > 0, (t_query - t_ref[jj - 1]) / np.where(gap > 0, gap, 1.), 0.)
    out[inside] = (values[jj - 1] + w * (values[jj] - values[jj - 1]))[inside]
    return out


def _stats(err):
    err = err[np.isfinite(err)]
    if not len(err):
        return dict(n=0)
    return dict(n=int(len(err)), mean=round(float(np.mean(err)), 4), rmse=round(float(np.sqrt(np.mean(err ** 2))), 4),
                p95_abs=round(float(np.percentile(np.abs(err), 95)), 4), max_abs=round(float(np.max(np.abs(err))), 4))


def estimate_error(refmap, ref_t, ref_x, ref_y, ref_yaw, ref_valid, ticks, drive, max_gap_s=.25):
    """Executor online lateral / heading minus the reference at the tick times inside the drive interval.
    ticks: dict with t (same time base as ref_t), lat [m], head [deg]."""
    s, d, psi = refmap.to_row_arrays(ref_x, ref_y, ref_yaw)
    t = np.asarray(ticks['t'], float)
    sel = np.ones(len(t), dtype=bool) if drive is None else (t >= drive[0]) & (t <= drive[1])
    tq = t[sel]
    d_ref = interp_valid(ref_t, d, ref_valid, tq, max_gap_s)
    psi_ref = interp_valid(ref_t, np.unwrap(psi), ref_valid, tq, max_gap_s)
    s_ref = interp_valid(ref_t, s, ref_valid, tq, max_gap_s)
    lat = np.asarray(ticks['lat'], float)[sel]
    head = -np.asarray(ticks['head'], float)[sel]
    lat_err = lat - d_ref
    head_err = head - np.degrees(psi_ref)
    return dict(lateral_m=_stats(lat_err), heading_deg=_stats(head_err),
                ticks_in_drive=int(np.count_nonzero(sel)), ticks_with_estimate=int(np.count_nonzero(np.isfinite(lat))),
                ticks_with_reference=int(np.count_nonzero(np.isfinite(d_ref)))), \
        dict(t=tq, along=s_ref, lat_ref=d_ref, lat_exec=lat, head_ref_deg=np.degrees(psi_ref), head_exec_deg=head)
