"""Open-loop replay of the local grid layer L2 on the real 2026-10-10 manual records (PC-07; part of PC-17).

No motor, no ROS, read-only on the records. The vendored estimator (fast layer L0) runs as in the PC-04 replay
(row_shadow_replay, setting c_proposed: core.ScanAdapter with the leave-one-out body mask, near-return confirmation
2.0 m / 0.35 m / 2 scans, receipt stamps - 0.08 s, gravity-projected IMU yaw rate); the grid variants ride on the same
estimator through the executor's own PerceptionLayers (fusion gain 0: the estimator is the same for every variant, so
one pass compares them all). Runs: every forward segment of the human drive of at least --min-segment-m (the 'flying'
runs of PC-04: a new estimator and new grids 2 s before the onset).

Scored ticks (20 Hz) = 'valid ticks': the human drives forward (reference ICP, labels only) and L0 reports a valid band.
'steady' ticks are the subset where the human drives straight (|reference yaw rate| <= 3 deg/s) and L0's band width is
aisle-like (2.5-5.5 m): in turns L0's band (heading gain 0.4) lags the new axis and the two layers differ by design.
At each one the newest grid fit (if not older than --max-fit-age-s) is evaluated at the estimator's base pose and
compared with the L0 snapshot: lateral offset, heading error and width differences. Agreement = |d lateral| <= 0.10 m and
|d heading| <= 1.5 deg (PC-07). 'strict' counts a valid tick without a fresh fit as a miss; 'with_fit' only scores ticks
that have one.

Cost: wall time of every grid update (insert + fit, perf_counter) and of every estimator on_scan, per variant. The
values depend on the machine load (write it next to the results); ARM values are not measured here (run this file on
the Jetson for those: it needs only this package, numpy, scipy and the records).

Usage (from tools/row_executor; outputs outside the repository):
  python -I -B local_grid_replay.py --out-dir <dir> [--records ...] [--workers 3] [--variants c010_n2 ...]
         [--min-segment-m 3] [--max-fit-age-s 1.0] [--mask-dir <PC-04 masks>]
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import platform
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import row_shadow_replay as R            # noqa: E402  (adapters, stamps, IMU, vendored estimator E)
import field_records as F                # noqa: E402
from row_executor import config as XC    # noqa: E402
from row_executor.perception_layers import PerceptionLayers  # noqa: E402

SCHEMA = 'ubts.row_executor.local_grid_replay.v1'
TICK_S = .05
LEAD_S = 2.
AGREE_LATERAL_M = .10
AGREE_HEADING_DEG = 1.5
COLUMNS = 10                  # run, t, d lateral, d heading [deg], d width, fit age, quality, |human yaw rate| [deg/s],
                              # L0 width, L0 row axis in the reference (ICP) frame [deg] = ref heading + L0 heading error
STEADY_TURN_DEG_S = 3.        # 'steady' ticks: the human drives straight (|yaw rate| <= this) ...
AISLE_WIDTH_M = (2.5, 5.5)    # ... and L0's band has an aisle-like width (row spacing 3.6-4.1 m minus the trunks)
C_PROPOSED = dict(transient=dict(near_m=2., tol_m=.35, confirm_scans=2))
VARIANTS = {
    'c010_n2': dict(local_grid_cell_m=.10, local_grid_every_n_scans=2),
    'c005_n2': dict(local_grid_cell_m=.05, local_grid_every_n_scans=2),
    'c020_n2': dict(local_grid_cell_m=.20, local_grid_every_n_scans=2),
    'c010_n1': dict(local_grid_cell_m=.10, local_grid_every_n_scans=1),
    'c010_n4': dict(local_grid_cell_m=.10, local_grid_every_n_scans=4),
    'c010_n2_free8': dict(local_grid_cell_m=.10, local_grid_every_n_scans=2, local_grid_free_beam_step=8),
    'c005_n2_free16': dict(local_grid_cell_m=.05, local_grid_every_n_scans=2, local_grid_free_beam_step=16),
    # fit region of L0's line fit (|along| < 8 m around the LiDAR): window 20 m with 8 m behind
    'c010_n2_l0region': dict(local_grid_cell_m=.10, local_grid_every_n_scans=2,
                             grid=dict(behind_m=8., fit_behind_m=8., fit_ahead_m=8.)),
    'c010_n2_hits5': dict(local_grid_cell_m=.10, local_grid_every_n_scans=2, local_grid_min_hits=5),
    'c010_n2_cov3': dict(local_grid_cell_m=.10, local_grid_every_n_scans=2, grid=dict(fit_min_coverage=3)),
    'c010_n2_hits2': dict(local_grid_cell_m=.10, local_grid_every_n_scans=2, local_grid_min_hits=2),
}
DEFAULT_VARIANTS = ('c010_n2', 'c005_n2', 'c020_n2', 'c010_n1', 'c010_n4', 'c010_n2_free8')


def layer_config(values):
    return replace(XC.ExecutorConfig(), local_grid_enabled=True,
                   **{k: v for k, v in values.items() if k != 'grid'}).validate()


def make_layer(name, config):
    """PerceptionLayers of a variant; a variant's 'grid' dict overrides GridSettings fields the config has no key for."""
    layer = PerceptionLayers(config)
    extra = VARIANTS[name].get('grid')
    if extra:
        layer.grid_settings = replace(layer.grid_settings, **extra)
    return layer


def pct(values, qs=(50, 90, 99, 100), scale=1., digits=4):
    if not len(values):
        return None
    arr = np.asarray(values, dtype=float) * scale
    return [round(float(np.percentile(arr, q)), digits) for q in qs]


def run_segment(rec, h, seg, adapter, imu_order, w_imu, stamps, names, configs, max_fit_age):
    est = R.E.SensorEstimator(R.E.EstimatorConfig())
    adapter.transient = R.fresh_transient(C_PROPOSED)
    layers = {}
    for name in names:
        layer = make_layer(name, configs[name])
        layer.new_run(seg['t_on'] - LEAD_S, None)
        layers[name] = layer
    t_on, t_off = seg['t_on'], seg['t_off']
    t_est = max(t_on - LEAD_S, rec['t_scan'][0] - 1e-6)
    imu = rec['imu']
    t_imu = imu[imu_order, 0] / 1e9
    w_sorted, acc_sorted = w_imu[imu_order], imu[imu_order, 5:8]
    ts = rec['t_scan']
    ks, k_end = int(np.searchsorted(ts, t_est, side='left')), int(np.searchsorted(ts, t_off, side='right'))
    ji = int(np.searchsorted(t_imu, t_est, side='left'))
    k_grid0 = int(round(h['t'][0] / TICK_S))
    amin, inc, rmax = rec['angle_min'], rec['increment'], rec['range_max']
    events = []
    j = ji
    while j < len(t_imu) and t_imu[j] <= t_off:
        events.append((float(t_imu[j]), 0, j))
        j += 1
    for k in range(ks, k_end):
        events.append((float(ts[k]), 1, k))
    for kt in range(math.ceil(t_on / TICK_S - 1e-6), math.floor(t_off / TICK_S + 1e-6) + 1):
        events.append((kt * TICK_S, 2, kt))
    events.sort()
    out = {n: dict(cost=[], rows=[]) for n in names}
    est_cost = []
    ticks = valid = 0
    for when, typ, idx in events:
        if typ == 0:
            if math.isfinite(w_sorted[idx]):
                est.on_imu(float(t_imu[idx]), float(w_sorted[idx]), tuple(acc_sorted[idx]))
        elif typ == 1:
            try:
                r, a0, inc_, rmax_, _ = adapter.convert(rec['ranges_mm'][idx].astype(np.float64) / 1000., amin, inc,
                                                        rmax)
            except R.X.ScanRejected:
                continue
            stamp = float(stamps[idx])
            c0 = time.perf_counter()
            est.on_scan(stamp, r, a0, inc_, rmax_)
            est_cost.append(time.perf_counter() - c0)
            for name in names:
                layer = layers[name]
                before = layer.grid.updates
                layer.on_scan(est, stamp, r, a0, inc_)
                if layer.grid.updates != before and layer.grid.last_cost_s is not None:
                    out[name]['cost'].append(layer.grid.last_cost_s)
        else:
            i = idx - k_grid0
            if not 0 <= i < len(h['t']) or not h['forward'][i]:
                continue
            ticks += 1
            snap = est.snapshot(when)
            if not snap['corridor_valid']:
                continue
            valid += 1
            base = est.base_pose(when)
            turn = abs(math.degrees(h['w'][i])) if math.isfinite(h['w'][i]) else np.nan
            ref_axis_l0 = math.degrees(R.X.wrap(h['theta'][i] + snap['heading_error']))                 if math.isfinite(h['theta'][i]) else np.nan
            for name in names:
                fit = layers[name].grid.fit
                if fit is None or when - fit.stamp > max_fit_age:
                    out[name]['rows'].append((when, np.nan, np.nan, np.nan, np.nan, np.nan, turn,
                                              snap['corridor_width'], ref_axis_l0))
                    continue
                lateral, heading = fit.observation(base)
                out[name]['rows'].append((when, lateral - snap['lateral_offset'],
                                          math.degrees(R.X.wrap(heading - snap['heading_error'])),
                                          fit.width - snap['corridor_width'], when - fit.stamp, fit.quality, turn,
                                          snap['corridor_width'], ref_axis_l0))
    info = {n: dict(updates=layers[n].grid.updates, fits=layers[n].grid.fits,
                    failures=dict(layers[n].grid.fit_failures), skipped_invalid=layers[n].grid.skipped_invalid)
            for n in names}
    return out, est_cost, ticks, valid, info


def replay_record(rid, names, out_dir, mask_dir, min_segment_m, max_fit_age, limit_segments=None):
    started = time.perf_counter()
    rec = F.load_record(rid, os.environ.get('ROW_REPLAY_BASE') or F.BASE_1010,
                        os.environ.get('ROW_REPLAY_DRIVE') or F.DRIVE_1010)
    k0, k1 = math.ceil(rec['t_scan'][0] / TICK_S), math.floor(rec['t_scan'][-1] / TICK_S)
    t_grid = np.arange(k0, k1 + 1, dtype=np.int64) * TICK_S
    h = F.human_ticks(rec, t_grid)
    segments = [s for s in F.forward_segments(h) if s['dist_m'] >= min_segment_m][:limit_segments]
    adapter, _ = R.make_adapter(C_PROPOSED, rid, mask_dir)
    imu_order, w_imu, _ = R.imu_yaw_rates(rec['imu'])
    stamps = R.scan_stamps(rec, C_PROPOSED, R.A.SCAN_LATENCY_S)
    configs = {n: layer_config(VARIANTS[n]) for n in names}
    rows = {n: [] for n in names}
    costs = {n: [] for n in names}
    est_costs = []
    info_total = {n: dict(updates=0, fits=0, failures={}, skipped_invalid=0) for n in names}
    ticks = valid = 0
    for run, seg in enumerate(segments):
        out, est_cost, t_count, v_count, info = run_segment(rec, h, seg, adapter, imu_order, w_imu, stamps, names,
                                                            configs, max_fit_age)
        ticks += t_count
        valid += v_count
        est_costs += est_cost
        for n in names:
            rows[n] += [(run,) + row for row in out[n]['rows']]
            costs[n] += out[n]['cost']
            total = info_total[n]
            for key in ('updates', 'fits', 'skipped_invalid'):
                total[key] += info[n][key]
            for key, count in info[n]['failures'].items():
                total['failures'][key] = total['failures'].get(key, 0) + count
    tag = rid[9:15]
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    arrays = {}
    for n in names:
        arrays[n] = np.asarray(rows[n], dtype=np.float64).reshape(-1, COLUMNS)
        arrays[n + '_cost_s'] = np.asarray(costs[n], dtype=np.float64)
    arrays['estimator_on_scan_s'] = np.asarray(est_costs, dtype=np.float64)
    np.savez_compressed(out_dir / ('grid_%s.npz' % tag), **arrays)
    summary = dict(schema=SCHEMA, rid=rid, segments=len(segments), forward_ticks=ticks, valid_ticks=valid,
                   variants={n: dict(stats(arrays[n], arrays[n + '_cost_s']), **info_total[n]) for n in names},
                   estimator_on_scan_ms_p50_p90_p99_max=pct(est_costs, scale=1e3, digits=3),
                   wall_s=round(time.perf_counter() - started, 1))
    (out_dir / ('summary_%s.json' % tag)).write_bytes((json.dumps(summary, indent=1, sort_keys=True) + '\n')
                                                        .encode('utf-8'))
    return summary


def stats(rows, cost):
    """Agreement and cost of one variant from its rows (COLUMNS); 'steady' = the subset of straight human driving with
    an aisle-like L0 width."""
    out = _agreement(rows, cost)
    if len(rows):
        steady = (rows[:, 7] <= STEADY_TURN_DEG_S) & (rows[:, 8] >= AISLE_WIDTH_M[0]) & (rows[:, 8] <= AISLE_WIDTH_M[1])
        part = _agreement(rows[steady], [])
        out['steady'] = {k: part[k] for k in ('valid_ticks', 'with_fit', 'agree', 'agree_strict', 'agree_with_fit',
                                              'lateral_abs_m_p50_p90_p99_max', 'heading_abs_deg_p50_p90_p99_max')}
        out['axis_consistency'] = axis_consistency(rows[steady])
    return out


def _wrap_deg(values):
    return (np.asarray(values) + 180.) % 360. - 180.


def axis_consistency(rows, window_s=2.5):
    """Row-axis scatter without a ground truth: along a straight row the corridor axis expressed in the reference (ICP)
    frame (reference heading + heading error) is constant. For every steady tick with a grid fit: |deviation from the
    median over the same run's steady ticks within +-window_s| (a segment may hold several rows: a run-wide median
    would mix them), for L0 and for the grid on the same ticks. Smaller = steadier; the reference's own heading noise
    is in both."""
    dev_l0, dev_grid = [], []
    has = np.isfinite(rows[:, 3]) & np.isfinite(rows[:, 9])
    for run in np.unique(rows[has, 0]):
        sel = has & (rows[:, 0] == run)
        t = rows[sel, 1]
        l0 = rows[sel, 9]
        grid = l0 + rows[sel, 3]              # grid heading error = L0 heading error + d heading
        for values, out in ((l0, dev_l0), (grid, dev_grid)):
            for i in range(len(t)):
                near = np.abs(t - t[i]) <= window_s
                if near.sum() < 10:
                    continue
                d = _wrap_deg(values[near] - values[i])
                out.append(abs(float(np.median(d))))
    return dict(ticks=len(dev_l0), window_s=window_s, l0_abs_deg_p50_p90=pct(dev_l0, (50, 90), digits=3),
                grid_abs_deg_p50_p90=pct(dev_grid, (50, 90), digits=3))


def _agreement(rows, cost):
    n = len(rows)
    has = np.isfinite(rows[:, 2]) if n else np.zeros(0, bool)
    ok = has & (np.abs(np.nan_to_num(rows[:, 2], nan=9.)) <= AGREE_LATERAL_M) \
        & (np.abs(np.nan_to_num(rows[:, 3], nan=99.)) <= AGREE_HEADING_DEG) if n else has
    dl, dh, dw = (np.abs(rows[has, c]) for c in (2, 3, 4)) if n else ([], [], [])
    return dict(valid_ticks=int(n), with_fit=int(has.sum()), agree=int(ok.sum()),
                agree_strict=None if not n else round(float(ok.sum()) / n, 4),
                agree_with_fit=None if not has.sum() else round(float(ok.sum()) / float(has.sum()), 4),
                lateral_abs_m_p50_p90_p99_max=pct(dl), heading_abs_deg_p50_p90_p99_max=pct(dh, digits=3),
                width_abs_m_p50_p90_p99_max=pct(dw),
                lateral_signed_mean_m=None if not len(dl) else round(float(np.mean(rows[has, 2])), 4),
                heading_signed_mean_deg=None if not len(dh) else round(float(np.mean(rows[has, 3])), 3),
                width_signed_mean_m=None if not len(dw) else round(float(np.mean(rows[has, 4])), 4),
                update_ms_p50_p90_p99_max=pct(cost, scale=1e3, digits=3), updates_timed=int(len(cost)))


def combine(out_dir, names, records):
    """Totals over the records from the saved arrays (the summary of the whole replay)."""
    out_dir = Path(out_dir)
    rows = {n: [] for n in names}
    costs = {n: [] for n in names}
    est = []
    per_record = {}
    for rid in records:
        tag = rid[9:15]
        data = np.load(out_dir / ('grid_%s.npz' % tag))
        for n in names:
            rows[n].append(data[n])
            costs[n].append(data[n + '_cost_s'])
        est.append(data['estimator_on_scan_s'])
        per_record[rid] = json.loads((out_dir / ('summary_%s.json' % tag)).read_bytes().decode('utf-8'))
    total = {}
    for n in names:
        r = np.concatenate(rows[n]) if rows[n] else np.zeros((0, COLUMNS))
        c = np.concatenate(costs[n]) if costs[n] else np.zeros(0)
        total[n] = dict(stats(r, c), settings=VARIANTS[n],
                        failures={k: sum(per_record[rid]['variants'][n]['failures'].get(k, 0) for rid in records)
                                  for k in sorted({k for rid in records
                                                   for k in per_record[rid]['variants'][n]['failures']})})
    e = np.concatenate(est) if est else np.zeros(0)
    return dict(schema=SCHEMA, result_kind='10/10 수동 주행 기록의 개방 루프 재생 (자율 주행 아님)', records=records,
                agree_rule=dict(lateral_m=AGREE_LATERAL_M, heading_deg=AGREE_HEADING_DEG),
                steady_rule=dict(human_yaw_rate_deg_s_max=STEADY_TURN_DEG_S, l0_width_m=list(AISLE_WIDTH_M)),
                variants=total, estimator_on_scan_ms_p50_p90_p99_max=pct(e, scale=1e3, digits=3),
                per_record={rid: dict(segments=per_record[rid]['segments'], valid_ticks=per_record[rid]['valid_ticks'],
                                      agree_strict={n: per_record[rid]['variants'][n]['agree_strict'] for n in names},
                                      steady_agree_strict={n: (per_record[rid]['variants'][n].get('steady') or {})
                                                           .get('agree_strict') for n in names})
                            for rid in records},
                machine=dict(platform=platform.platform(), python=platform.python_version(), numpy=np.__version__,
                             cpu_count=os.cpu_count()))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--records', nargs='*', default=list(F.RECORDS_1010))
    ap.add_argument('--variants', nargs='*', default=list(DEFAULT_VARIANTS))
    ap.add_argument('--workers', type=int, default=1)
    ap.add_argument('--mask-dir')
    ap.add_argument('--min-segment-m', type=float, default=3.)
    ap.add_argument('--max-fit-age-s', type=float, default=1.)
    ap.add_argument('--limit-segments', type=int, help='development: only the first N segments of each record')
    args = ap.parse_args(argv)
    unknown = [n for n in args.variants if n not in VARIANTS]
    if unknown:
        raise SystemExit('unknown variants: %s (known: %s)' % (', '.join(unknown), ', '.join(VARIANTS)))
    mask_dir = args.mask_dir or os.path.join(args.out_dir, 'masks')
    R.wait_for_measurement_hold()
    jobs = [(rid, args.variants, args.out_dir, mask_dir, args.min_segment_m, args.max_fit_age_s, args.limit_segments)
            for rid in args.records]
    started = time.perf_counter()
    if args.workers > 1:
        with ProcessPoolExecutor(args.workers) as pool:
            for result in pool.map(_job, jobs):
                print(json.dumps(dict(rid=result['rid'], valid=result['valid_ticks'], wall_s=result['wall_s'])),
                      flush=True)
    else:
        for job in jobs:
            result = _job(job)
            print(json.dumps(dict(rid=result['rid'], valid=result['valid_ticks'], wall_s=result['wall_s'])), flush=True)
    total = combine(args.out_dir, args.variants, args.records)
    total.update(workers=args.workers, wall_s=round(time.perf_counter() - started, 1),
                 min_segment_m=args.min_segment_m, max_fit_age_s=args.max_fit_age_s, limit_segments=args.limit_segments)
    Path(args.out_dir, 'SUMMARY.json').write_bytes((json.dumps(total, indent=1, sort_keys=True, ensure_ascii=False)
                                                    + '\n').encode('utf-8'))
    for n in args.variants:
        v = total['variants'][n]
        print('%-16s strict %s with_fit %s steady %s/%s lat %s head %s ms %s' % (
            n, v['agree_strict'], v['agree_with_fit'], (v.get('steady') or {}).get('agree_strict'),
            (v.get('steady') or {}).get('valid_ticks'), v['lateral_abs_m_p50_p90_p99_max'],
            v['heading_abs_deg_p50_p90_p99_max'], v['update_ms_p50_p90_p99_max']))
    return 0


def _job(job):
    return replay_record(*job)


if __name__ == '__main__':
    sys.exit(main())
