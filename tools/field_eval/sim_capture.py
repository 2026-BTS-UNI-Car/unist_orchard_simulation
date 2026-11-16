"""PC-only (not for the Jetson): F-SIM runs of the existing follow matrices with the executor's sensor inputs saved,
as test data with truth for this tool. Nothing in the simulation changes: the episode spec of
steering.safety_scripts.follow_matrix (steer_follow_v1 conditions, profile v0.1) or follow_realdims (matrix part,
measured-dimension profile) is run by tools/row_executor/sim_in_loop_steer.run_episode with capture_inputs (the
facade keeps the converted S2 LaserScan and IMU messages it feeds the executor; the digests of the run do not
depend on it).

Writes per run folder: TRACE.npz, SERIAL.jsonl, EXECUTOR.jsonl, RESULT.json (facade.write, as the matrices do),
SCANS.npz (stamp_s, receipt_s, ranges_mm uint16 (0 = no return), angle_min, angle_increment, range_min, range_max),
IMU.npz (stamp_s, receipt_s, gyro (n, 3), accel (n, 3)), MAP.json (the W0 posts as a field_eval reference map, both
post lines, with the episode start pose) and FOLLOW_METRICS.json (follow_matrix.measure on the trace truth: the
registered metric values of this run).

CLI (from the work tree root): python -B tools/field_eval/sim_capture.py --out <folder outside the repository>
       [--matrix follow_matrix|follow_realdims] [--only nominal lateral_p040 ...] [--seeds 0 1] [--workers N]
Every number is a placeholder-based SIM result (자리값 기반 SIM 결과), not field performance.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for _p in (os.path.join(REPO, 'sim', 'ubts'), TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

CAPTURE_ALL = 10 ** 7


def _condition(name):
    from steering.safety_scripts import follow_matrix as FM
    for cond in FM.CONDITIONS:
        if cond['name'] == name:
            return cond
    raise KeyError(name)


def episode_for(matrix, cond, seed):
    from steering.safety_scripts import follow_matrix as FM
    if matrix == 'follow_matrix':
        episode = FM.spec(cond, seed)
        shown = FM.printable(episode, cond)
    elif matrix == 'follow_realdims':
        from steering.safety_scripts import follow_realdims as FR
        episode = FR.matrix_spec(cond, seed, 'matrix')
        shown = FR.printable(episode, cond, 'matrix')
    else:
        raise ValueError(matrix)
    episode = dict(episode, capture_inputs=CAPTURE_ALL)
    shown = dict(shown, capture_inputs=CAPTURE_ALL)
    return episode, shown


def _write_bytes(path, raw):
    with open(path, 'wb') as fh:
        fh.write(raw)


def save_inputs(folder, inputs):
    scans = inputs['scans']
    stamp = np.array([float(s['header']) for s in scans])
    receipt = np.array([float(s['receipt']) for s in scans])
    ranges = np.stack([np.asarray(s['msg'].ranges, dtype=np.float64) for s in scans])
    mm = np.where(np.isfinite(ranges) & (ranges > 0.), np.round(ranges * 1000.), 0.).astype(np.uint16)
    first = scans[0]['msg']
    np.savez_compressed(os.path.join(folder, 'SCANS.npz'), stamp_s=stamp, receipt_s=receipt, ranges_mm=mm,
                        angle_min=np.float64(first.angle_min), angle_increment=np.float64(first.angle_increment),
                        range_min=np.float64(first.range_min), range_max=np.float64(first.range_max))
    imus = inputs['imus']
    np.savez_compressed(os.path.join(folder, 'IMU.npz'), stamp_s=np.array([float(m['header']) for m in imus]),
                        receipt_s=np.array([float(m['receipt']) for m in imus]),
                        gyro=np.array([[0., 0., float(m['gyro_z'])] for m in imus]).reshape(-1, 3),
                        accel=np.array([list(m['accel']) for m in imus], dtype=float).reshape(-1, 3))
    return len(scans), len(imus)


def map_from_markers(markers, start_pose, note):
    """field_eval reference map JSON of W0 posts: y > 0 row 'left', y < 0 row 'right' (others 'other')."""
    marks = []
    for k, (x, y, r) in enumerate(markers):
        row = 'left' if y > .5 else 'right' if y < -.5 else 'other'
        marks.append(dict(id='P%02d' % k, row=row, x=float(x), y=float(y), radius_m=float(r), kind='post'))
    return dict(schema='ubts.field_eval.map.v1', frame='row_local', units='m', lane_rows=['right', 'left'],
                landmarks=marks, start_pose=[float(v) for v in start_pose], note=note)


def run_one(item):
    matrix, name, seed, out_root = item
    from row_executor import sim_in_loop_steer as S
    from steering.safety_scripts import follow_matrix as FM
    cond = _condition(name)
    episode, shown = episode_for(matrix, cond, seed)
    folder = os.path.join(out_root, episode['name'])
    os.makedirs(folder, exist_ok=True)
    result, facade = S.run_episode(episode, None, keep=True)
    try:
        result = facade.write(folder, shown)
        n_scans, n_imu = save_inputs(folder, facade.inputs)
        arrays = facade.trace_arrays()
        ticks = [r for r in facade.log.records if r.get('k') == 'tick']
        metrics = FM.measure(arrays, ticks, facade.summary(), episode['world']['markers'], cond.get('settle_m', 0.))
    finally:
        facade.rig.close()
    doc = map_from_markers(episode['world']['markers'], episode['start_pose'],
                           'SIM W0 posts of %s (truth map; SIM only)' % episode['name'])
    _write_bytes(os.path.join(folder, 'MAP.json'),
                 (json.dumps(doc, indent=1, sort_keys=True) + '\n').encode('utf-8'))
    row = dict(matrix=matrix, condition=name, seed=seed, name=episode['name'], settle_m=cond.get('settle_m', 0.),
               metrics=metrics, digests=result['digests'], config_digest=result['config_digest'], scans=n_scans,
               imu=n_imu, end_reason=result['summary']['end_reason'], result_kind='자리값 기반 SIM 결과')
    _write_bytes(os.path.join(folder, 'FOLLOW_METRICS.json'),
                 (json.dumps(row, indent=1, sort_keys=True, default=str) + '\n').encode('utf-8'))
    return row


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--out', required=True)
    p.add_argument('--matrix', default='follow_matrix', choices=('follow_matrix', 'follow_realdims'))
    p.add_argument('--only', nargs='*', default=['nominal'])
    p.add_argument('--seeds', nargs='*', type=int, default=[0])
    p.add_argument('--workers', type=int, default=1)
    args = p.parse_args(argv)
    from steering.scene_spec import inside_repository
    out = os.path.abspath(args.out)
    if inside_repository(out):
        raise SystemExit('refused: %s lies inside the repository (runs live outside)' % out)
    items = [(args.matrix, name, seed, out) for name in args.only for seed in args.seeds]
    if args.workers > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=min(4, args.workers)) as pool:
            rows = list(pool.map(run_one, items))
    else:
        rows = [run_one(item) for item in items]
    for r in rows:
        m = r['metrics']
        print('%-40s end %s rmse %s p95 %s max %s e %s scans %d' % (r['name'], r['end_reason'], m.get('lateral_rmse_m'),
                                                                 m.get('lateral_p95_m'), m.get('lateral_max_m'),
                                                                 m.get('row_end_error_m'), r['scans']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
