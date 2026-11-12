#!/usr/bin/env python3
"""Drive analysis of calibration recordings (C1-C4, C7): the analysis part of 546fa8c field_quick_calib.analyse().

Pipeline (unchanged from 546fa8c): motor.jsonl -> time windows around every command (+ standstills for C1) -> parse
only those scans -> point-to-line ICP odometry (worker processes) -> travel axis, LiDAR/IMU yaw sign -> C1 standstill
gyro, C2 breakaway ladder, C3 v(duty), C4 straight drift, C7 stop. Not here: the differential C5 arc and C6 pivot and
the config recommendation of 546fa8c (recommend.py; the front-steer executor config is not the differential one).

Usage (from the repository root or anywhere; nothing is written next to the recordings):
  python -B tools/field_calib/drive_calib.py --root <recordings folder> --out <result.json> <recording id> ...
  python -B tools/field_calib/drive_calib.py --root <folder> --latest 3 --out <result.json>
The front-steer bridge writes requested.left = requested.right = the throttle duty (design 4.2 / 4.5), so every
forward command is a 'straight_fwd' command for these steps; select straight recordings (F3) for C3 and C4, or give
--steer-sidecar so that segments with a steering target are dropped (sim/ubts/steering/ident writes that sidecar).

Dependencies: numpy, scipy (ICP, odometry.py). Results are measurements of the recordings only (no truth).
"""
import os

for _var in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_var, '1')     # worker processes do the parallelism; keep BLAS single-threaded

import argparse                          # noqa: E402
from concurrent.futures import ProcessPoolExecutor   # noqa: E402
import json                              # noqa: E402
import math                              # noqa: E402
import sys                               # noqa: E402
import time                              # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import numpy as np                       # noqa: E402

from field_calib import odometry as odo  # noqa: E402
from field_calib import records, steps   # noqa: E402
from field_calib import timeline as tl   # noqa: E402

SCHEMA = 'ubts.field_calib.drive_result.v1'
DEFAULT_LIDAR_YAW_OFFSET_DEG = 179.08     # executor config lidar_yaw_offset_deg (design 3.7; travel axis -179.08 deg)
# Stop model for the C7 distance-over-model ratio (546fa8c row_executor STOP_MODEL_DEFAULTS; report only here)
DEFAULT_STOP_MODEL = dict(reaction_s=.15, onset_s=.25, decel=1.07)
STEER_STRAIGHT_MRAD = 35                  # --steer-sidecar: |target| above this (about 2 deg) is not a straight run


def steering_mask_windows(sidecar_path, straight_mrad=STEER_STRAIGHT_MRAD):
    """[(t0, t1)] receipt-time windows (host wall clock, s) in which the steering target was not straight, from a
    steering sidecar (orchard.steering.timeline.v1 rows with received_at and target_mrad). Empty when no file."""
    if not sidecar_path or not os.path.isfile(sidecar_path):
        return []
    out, start = [], None
    with open(sidecar_path, 'r', encoding='utf-8') as fh:
        rows = [json.loads(line) for line in fh if line.strip()]
    rows.sort(key=lambda r: r.get('received_at') or 0.)
    for row in rows:
        t, target = row.get('received_at'), row.get('target_mrad')
        turned = target is not None and abs(target) > straight_mrad
        if turned and start is None:
            start = t
        elif not turned and start is not None:
            out.append((start, t))
            start = None
    if start is not None and rows:
        out.append((start, rows[-1].get('received_at')))
    return out


def steering_stop_lines(sidecar_path):
    """MCU status rows of a steering sidecar (orchard.steering.timeline.v1, one per status line, 20 Hz) as
    [dict(t (host time of the MCU tick, s), L, R, state)], sorted: the host time of an MCU ms is the smallest receipt -
    ms over the file (the line read soonest after it was made). [] without a file. Used for the C7 hard stop command time
    (audit AUD3A-03: the 2 Hz motor log cannot place a brake)."""
    if not sidecar_path or not os.path.isfile(sidecar_path):
        return []
    with open(sidecar_path, 'r', encoding='utf-8') as fh:
        rows = [json.loads(line) for line in fh if line.strip()]
    rows = [r for r in rows if r.get('mcu_ms') is not None and r.get('received_at') is not None
            and r.get('drive_l') is not None and r.get('drive_r') is not None]
    if not rows:
        return []
    offset = min(float(r['received_at']) - float(r['mcu_ms']) / 1000. for r in rows)
    out = [dict(t=offset + float(r['mcu_ms']) / 1000., L=int(r['drive_l']), R=int(r['drive_r']), state=r.get('state'))
           for r in rows]
    out.sort(key=lambda r: r['t'])
    return out


def drop_steered(rec, windows):
    """Mark motor statuses inside steered windows as unknown requests (the steps then skip them)."""
    if not windows:
        return 0
    t = rec.motor.t
    hit = np.zeros(len(t), dtype=bool)
    for a, b in windows:
        hit |= (t >= a - .5) & (t <= b + .5)
    rec.motor.req_l = np.where(hit, np.nan, rec.motor.req_l)
    rec.motor.req_r = np.where(hit, np.nan, rec.motor.req_r)
    return int(hit.sum())


def analyse(recs, workers=1, chunk_scans=600, lidar_yaw_offset_deg=DEFAULT_LIDAR_YAW_OFFSET_DEG,
            stop_model=None, pool=None, log=print, skipped=(), cache_dir=None):
    """C1-C4 and C7 of opened recordings (records.Recording). Returns (result dict, steps.Series list).
    cache_dir: save the ICP result next to the cache key of records.open_archive (a re-run skips the scans)."""
    timing = dict(odometry=0., analysis=0.)
    series = []
    for rec in recs:
        t0 = time.perf_counter()
        n = len(rec.scan_t)
        if rec.cached_odometry is not None:
            pose, quality, chain = rec.cached_odometry
        elif n:
            gaps = np.nonzero(np.diff(rec.scan_t) > 1.)[0]
            bounds = [0] + list(gaps + 1) + [n]
            windows = [(a, b) for a, b in zip(bounds[:-1], bounds[1:]) if b - a >= 2]
            pose, quality, chain = odo.odometry(rec.ranges_mm, rec.geometry['angle_min'],
                                                rec.geometry['angle_increment'], windows, workers=workers,
                                                chunk_scans=chunk_scans, pool=pool)
            if cache_dir and rec.cache_key:
                records.save_cache(cache_dir, rec, pose, quality, chain)
        else:
            pose, quality, chain = np.zeros((0, 3)), np.zeros((0, 4)), np.zeros(0, dtype=np.int64)
        timing['odometry'] += time.perf_counter() - t0
        log('  odometry %s: %d scans in %.1f s' % (rec.rid, n, time.perf_counter() - t0))
        series.append(steps.Series(rec, pose, quality, chain))
    t1 = time.perf_counter()
    axis = steps.travel_axis(series, -float(lidar_yaw_offset_deg))
    sign = steps.lidar_imu_sign(series)
    for s in series:
        s.set_axis(math.radians(axis['axis_deg']), sign['sign'])
    model = dict(DEFAULT_STOP_MODEL if stop_model is None else stop_model)
    c2_events = steps.start_events(series)
    results = dict(
        schema=SCHEMA, axis=axis, lidar_imu=sign, straight_bias=steps.straight_bias(series),
        health=[steps.health(rec, s) for rec, s in zip(recs, series)],
        c1=steps.c1_standstill(series), c2=steps.c2_breakaway(c2_events), c2_events=c2_events,
        c3=steps.c3_speed(series), c4=steps.c4_straight(series), c7=steps.c7_stop(steps.stop_events(series), model),
        skipped_recordings=list(skipped))
    timing['analysis'] = time.perf_counter() - t1
    results['timing_s'] = {k: round(v, 2) for k, v in timing.items()}
    return results, series


def open_recordings(root, rids, labels=None, workers=1, pool=None, hashes=True, cache_dir=None, log=print,
                    steer_sidecars=None):
    """Open each recording (archive layout); unreadable ones are skipped and listed (546fa8c rule)."""
    labels = labels or {}
    recs, skipped = [], []
    for rid in rids:
        try:
            rec = records.open_archive(root, rid, labels.get(rid, ''), lambda m: tl.interest_windows(m), hashes=hashes,
                                       pool=pool, workers=workers, cache_dir=cache_dir)
        except (records.RecordingError, OSError, ValueError, KeyError, EOFError) as exc:
            log('  SKIP %s: %s' % (rid, exc))
            skipped.append(dict(rid=rid, error=str(exc)[:200]))
            continue
        if steer_sidecars:
            dropped = drop_steered(rec, steering_mask_windows(steer_sidecars.get(rid)))
            if dropped:
                rec.warnings.append('steered_statuses_dropped:%d' % dropped)
            rec.stop_lines = steering_stop_lines(steer_sidecars.get(rid))
        recs.append(rec)
    return recs, skipped


def _json_safe(value):
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return None if not math.isfinite(float(value)) else float(value)
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, (np.bool_,)):
        return bool(value)
    return value


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('rids', nargs='*', help='recording ids (directory names)')
    ap.add_argument('--root', required=True, help='recordings folder')
    ap.add_argument('--latest', type=int, default=None, help='the N most recent recordings under --root')
    ap.add_argument('--label', action='append', default=[], help='RID=surface label, repeatable')
    ap.add_argument('--out', required=True, help='result JSON (outside the recording folders)')
    ap.add_argument('--workers', type=int, default=1)
    ap.add_argument('--lidar-yaw-offset-deg', type=float, default=DEFAULT_LIDAR_YAW_OFFSET_DEG)
    ap.add_argument('--steer-sidecar', action='append', default=[],
                    help='RID=steering.jsonl: drop the statuses with a steering target (repeatable)')
    ap.add_argument('--no-hash', action='store_true')
    ap.add_argument('--cache', default=None, help='folder for ICP results keyed by the exact scan selection')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args(argv)
    log = (lambda *a: None) if args.quiet else (lambda *a: print(*a, file=sys.stderr, flush=True))
    rids = list(args.rids)
    if not rids:
        names = records.list_recordings(args.root)
        rids = names[-args.latest:] if args.latest else names
    if not rids:
        raise SystemExit('no recordings selected')
    labels = dict(item.split('=', 1) for item in args.label if '=' in item)
    sidecars = dict(item.split('=', 1) for item in args.steer_sidecar if '=' in item)
    workers = max(1, min(odo.MAX_WORKERS, args.workers))
    pool = ProcessPoolExecutor(max_workers=workers) if workers > 1 else None
    try:
        recs, skipped = open_recordings(args.root, rids, labels, workers, pool, not args.no_hash, cache_dir=args.cache,
                                        log=log, steer_sidecars=sidecars)
        results, _ = analyse(recs, workers=workers, lidar_yaw_offset_deg=args.lidar_yaw_offset_deg, pool=pool,
                             log=log, skipped=skipped, cache_dir=args.cache)
    finally:
        if pool is not None:
            pool.shutdown()
    with open(args.out, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(json.dumps(_json_safe(results), indent=1, sort_keys=True, allow_nan=False) + '\n')
    log('wrote %s' % args.out)
    return 0 if not skipped else 3


if __name__ == '__main__':
    sys.exit(main())
