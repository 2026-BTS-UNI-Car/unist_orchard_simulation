"""PC-only check of the tool on an F-SIM run with truth (a run folder of sim_capture.py).

The tool runs as on a field record (the map, the executor JSONL, the captured scans and IMU; no truth), then its
output is compared with the trace truth (TRACE.npz truth_x_m / truth_y_m / truth_yaw_rad / truth_v_m_s, the footprint
centre; W0: the map frame is the world frame):
  localisation   position error |p_loc - p_true| at the scan times, its along / lateral parts and the heading error
                 (all measured scans, and the scans inside the drive interval)
  metrics        the tool's reference metrics minus (a) the registered metrics of the run (follow_matrix.measure on
                 the 10 ms truth, FOLLOW_METRICS.json) and (b) the same definitions on the truth sampled at the scan
                 times (separates the localisation error from the sampling)
  estimate       the executor's online estimate error as the tool measures it (against the LiDAR reference) and as
                 it is (against the truth)
Options model field conditions: a start pose error, Gaussian noise on the map positions (tape-measure error), and
landmarks left out of the map. The truth metrics always use the true map.
Every number is a placeholder-based SIM result (자리값 기반 SIM 결과).
"""
from __future__ import annotations

import json
import math
import os

import numpy as np

from . import evaluate as EV
from . import exec_log as XL
from . import localise as LOC
from . import metrics as MET
from . import refmap as RM

METRIC_KEYS = ('lateral_rmse_m', 'lateral_p95_m', 'lateral_max_m', 'heading_rmse_deg', 'heading_max_deg',
               'row_end_error_m', 'speed_max_m_s')


def load_truth(run_dir):
    with np.load(os.path.join(run_dir, 'TRACE.npz')) as z:
        return dict(t=np.asarray(z['t_s'], float), x=np.asarray(z['truth_x_m'], float),
                    y=np.asarray(z['truth_y_m'], float), yaw=np.asarray(z['truth_yaw_rad'], float),
                    v=np.asarray(z['truth_v_m_s'], float), throttle=np.asarray(z['ctrl_throttle_pct'], float))


def perturbed_map(src, out_path, noise_m=0., drop=0., seed=0, start_error=(0., 0., 0.)):
    with open(src, 'rb') as fh:
        doc = json.loads(fh.read().decode('utf-8'))
    rng = np.random.default_rng(seed)
    marks = []
    for item in doc['landmarks']:
        if drop > 0. and rng.random() < drop:
            continue
        item = dict(item)
        if noise_m > 0.:
            item['x'] = round(float(item['x']) + float(rng.normal(0., noise_m)), 4)
            item['y'] = round(float(item['y']) + float(rng.normal(0., noise_m)), 4)
        marks.append(item)
    doc['landmarks'] = marks
    x, y, yaw = doc['start_pose']
    da, dl, dyaw = start_error
    doc['start_pose'] = [x + da, y + dl, yaw + dyaw]
    doc['note'] = '%s; perturbed: noise %.3f m, drop %.2f, seed %d, start error %s' % (doc.get('note', ''), noise_m,
                                                                                       drop, seed, list(start_error))
    with open(out_path, 'wb') as fh:
        fh.write((json.dumps(doc, indent=1, sort_keys=True) + '\n').encode('utf-8'))
    return out_path


def _err_stats(v):
    v = np.abs(v[np.isfinite(v)])
    if not len(v):
        return dict(n=0)
    return dict(n=int(len(v)), p50=round(float(np.percentile(v, 50)), 4), p95=round(float(np.percentile(v, 95)), 4),
                max=round(float(np.max(v)), 4))


def verify(run_dir, out_dir, start_error=(0., 0., 0.), map_noise_m=0., map_drop=0., seed=0, figures=True):
    os.makedirs(out_dir, exist_ok=True)
    true_map_path = os.path.join(run_dir, 'MAP.json')
    true_map = RM.load_map(true_map_path)
    with open(os.path.join(run_dir, 'FOLLOW_METRICS.json'), 'rb') as fh:
        registered = json.loads(fh.read().decode('utf-8'))
    map_path = true_map_path
    if map_noise_m or map_drop or any(start_error):
        map_path = perturbed_map(true_map_path, os.path.join(out_dir, 'MAP_USED.json'), map_noise_m, map_drop, seed,
                                 start_error)
    truth = load_truth(run_dir)
    report = EV.evaluate(map_path, os.path.join(run_dir, 'EXECUTOR.jsonl'), run_dir, out_dir,
                         settle_m=registered.get('settle_m', 0.) or 0., figures=figures,
                         truth=dict(truth, label='SIM 참값'))
    traj = report['_trajectory']
    meas = (traj['quality'] == LOC.OK) | (traj['quality'] == LOC.WEAK)
    t = traj['t']
    tx, ty = np.interp(t, truth['t'], truth['x']), np.interp(t, truth['t'], truth['y'])
    tyaw = np.interp(t, truth['t'], np.unwrap(truth['yaw']))
    dx, dy = traj['x'] - tx, traj['y'] - ty
    along = dx * np.cos(tyaw) + dy * np.sin(tyaw)
    lateral = -dx * np.sin(tyaw) + dy * np.cos(tyaw)
    dyaw = np.degrees(LOC.wrap(traj['yaw'] - tyaw))
    drive = report['time']['drive_interval_s']
    inside = meas & (t >= drive[0]) & (t <= drive[1]) if drive else meas

    def block(sel):
        return dict(scans=int(np.count_nonzero(sel)), position_m=_err_stats(np.hypot(dx, dy)[sel]),
                    along_m=_err_stats(along[sel]), lateral_m=_err_stats(lateral[sel]),
                    heading_deg=_err_stats(dyaw[sel]),
                    lateral_mean_m=None if not np.any(sel) else round(float(np.mean(lateral[sel])), 4))
    loc = dict(all_measured=block(meas), in_drive=block(inside))
    # metrics: tool vs registered (10 ms truth) vs truth at the scan times
    executor = XL.ExecutorLog(os.path.join(run_dir, 'EXECUTOR.jsonl'))
    settle = registered.get('settle_m', 0.) or 0.
    truth_at_scans = MET.measure(true_map, t, tx, ty, tyaw, np.interp(t, truth['t'], truth['v']), drive, settle,
                                 .61, rest_window_s=EV.REST_WINDOW_S)
    drive_rows = np.flatnonzero(truth['throttle'] > 0)
    truth_10ms = MET.measure(true_map, truth['t'], truth['x'], truth['y'], truth['yaw'], truth['v'],
                             (float(truth['t'][drive_rows[0]]), float(truth['t'][drive_rows[-1]])), settle, .61)
    tool = report['reference_metrics']
    reg = registered['metrics']
    rows = {}
    for key in METRIC_KEYS:
        a, b, c, d = tool.get(key), reg.get(key), truth_at_scans.get(key), truth_10ms.get(key)
        rows[key] = dict(tool=a, registered=b, truth_at_scans=c, truth_10ms_this_code=d,
                         tool_minus_registered=None if a is None or b is None else round(a - b, 4),
                         tool_minus_truth_at_scans=None if a is None or c is None else round(a - c, 4))
    # executor estimate error: against the reference (tool) and against the truth
    ticks = executor.run_ticks()
    true_est, _ = MET.estimate_error(true_map, truth['t'], truth['x'], truth['y'], truth['yaw'],
                                     np.ones(len(truth['t']), dtype=bool), dict(t=ticks['t'], lat=ticks['lat'],
                                                                                head=ticks['head']), drive, .05)
    row = dict(run=os.path.basename(os.path.normpath(run_dir)), result_kind=EV.SIM_KIND, map_used=os.path.basename(map_path),
               map_noise_m=map_noise_m, map_drop=map_drop, seed=seed, start_error=list(start_error),
               localisation_quality=report['localisation']['in_drive'], localisation_init=report['localisation']['init'],
               localisation_error=loc, metrics=rows,
               executor_estimate_error=dict(by_tool=report['executor_estimate_error'], by_truth=true_est),
               registered_check=dict(this_code_equals_registered={k: rows[k]['truth_10ms_this_code'] == rows[k]['registered']
                                                                  for k in METRIC_KEYS}),
               report_digest=EV.digest_report(report))
    with open(os.path.join(out_dir, 'VERIFY.json'), 'wb') as fh:
        fh.write((json.dumps(EV.clean(row), indent=1, sort_keys=True, ensure_ascii=False) + '\n').encode('utf-8'))
    return row
