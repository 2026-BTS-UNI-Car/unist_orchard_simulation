"""Development comparison of the perception layers L2 (local grid) and L3 (GNSS) on the fast SIM T1 rig (PC-07, PC-12).

NOT a pre-registered acceptance measurement: the layers are compared ON and OFF on the conditions of the row-following
matrix (follow_matrix.CONDITIONS, registered for the controller, steer_follow_v1) and on GNSS conditions, with the
follow_matrix metrics and checks applied as they are (the check results here are development readings). The wide
pre-registered matrix is another work item (PC-14 / PC-15).

Variants (ExecutorConfig changes; the rig gets a virtual GNSS only where a variant or condition says so):
  off         the registered configuration (both layers off)
  grid_log    local grid on, fusion gain 0 (display / log only: the commands must equal 'off')
  grid_fuse   local grid on, fusion gain 0.2 (design v2 V2-3 placeholder)
  gnss_log    virtual GNSS, gnss_mode log_only (the commands must equal 'off')
  all         grid fusion 0.2 + gnss_mode geofence (+ virtual GNSS)
GNSS conditions (nominal row, seed 0..; variants gnss_log and all): gnss_nominal, gnss_dropout_60s (no fix 10-70 s),
gnss_jump_5m / gnss_jump_15m (three one-fix spikes at 30, 45, 60 s), gnss_hacc_x3 (reported accuracy and noise x3 for
10-90 s), gnss_bias_20m (a 20 m constant offset: the fence must not trip, the offset is common to the whole run).

Metrics per run: follow_matrix.measure / judge (truth columns: evaluation only), the executor log's grid and GNSS blocks
(fusion outcomes, GNSS outcome counts, fence), the virtual receiver's injection counters, and for the *_log variants
whether the executor's command sequence (throttle, steering) equals the 'off' run of the same condition and seed.
Wall-clock values (executor step, grid update) depend on the load; they are reported, never judged.

CLI (from sim/ubts): python -B -m steering.safety_scripts.perception_compare --out <folder outside the repository>
                     [--workers N (<= 4)] [--variants ...] [--only CONDITION ...] [--gnss-seeds 0 1 ...]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time

import numpy as np

from . import acceptance as A
from . import follow_matrix as FM
from . import measure as M

SCHEMA = 'ubts.steer_sim.perception_compare.v1'
GRID_FUSE = dict(local_grid_enabled=True, local_grid_fusion_gain=.2)
VARIANTS = {
    'off': dict(config={}, gnss=False),
    'grid_log': dict(config=dict(local_grid_enabled=True), gnss=False),
    'grid_fuse': dict(config=dict(GRID_FUSE), gnss=False),
    'gnss_log': dict(config=dict(gnss_mode='log_only'), gnss=True),
    'all': dict(config=dict(GRID_FUSE, gnss_mode='geofence'), gnss=True),
}
LOG_VARIANTS = ('grid_log', 'gnss_log')          # must not change any command
SPIKES = (30., 45., 60.)
GNSS_CONDITIONS = (
    dict(name='gnss_nominal', gnss={}),
    dict(name='gnss_dropout_60s', gnss=dict(dropouts=[[10., 70.]])),
    dict(name='gnss_jump_5m', gnss=dict(jumps=[dict(at_s=t, east_m=5. * math.cos(k), north_m=5. * math.sin(k),
                                                    duration_s=.5) for k, t in enumerate(SPIKES)])),
    dict(name='gnss_jump_15m', gnss=dict(jumps=[dict(at_s=t, east_m=15. * math.cos(k), north_m=15. * math.sin(k),
                                                     duration_s=.5) for k, t in enumerate(SPIKES)])),
    dict(name='gnss_hacc_x3', gnss=dict(hacc_windows=[[10., 90., 3.]])),
    dict(name='gnss_bias_20m', gnss=dict(frame_offset_m=[20., -10.], bias_sigma_m=0.)),
)


def episode(condition, seed, variant, gnss_spec=None, stream_condition=None):
    """The follow_matrix episode of (condition, seed) with the variant's configuration. The random streams keep the
    follow_matrix key of (stream_condition or condition, seed), so every variant (and a GNSS condition's 'off' run) sees
    the same plant and sensor noise as the registered follow_matrix run."""
    ep = FM.spec(condition, seed)
    v = VARIANTS[variant]
    ep['stream_key'] = 'steer_exec_follow_%s_s%d' % (stream_condition or condition['name'], seed)
    ep['name'] = '%s__%s' % (ep['name'], variant)
    ep['config'] = dict(v['config'])
    if v['gnss']:
        ep['rig'] = dict(ep['rig'], gnss=dict(gnss_spec or {}))
    return ep


def commands(ticks):
    """(t, throttle, steering) of every executor tick: the command sequence."""
    return [(r['t'], r.get('T'), r.get('S')) for r in ticks]


def layer_blocks(ticks):
    last_grid = next((r['grid'] for r in reversed(ticks) if 'grid' in r), None)
    last_gnss = next((r['gnss'] for r in reversed(ticks) if 'gnss' in r), None)
    grid_ms = [r['grid']['proc_ms'] for r in ticks if 'grid' in r and r['grid'].get('proc_ms') is not None]
    fits = [r['grid'].get('fit') for r in ticks if 'grid' in r and r['grid'].get('fit')]
    out = {}
    if last_grid is not None:
        out['grid'] = dict(updates=last_grid.get('updates'), fits=last_grid.get('fits'),
                           fusion=(last_grid.get('fusion') or {}).get('outcomes'),
                           # proc_ms of the tick records repeats the last update; distinct values approximate the updates
                           update_ms_p50_p99=None if not grid_ms else [round(float(np.percentile(grid_ms, q)), 3)
                                                                       for q in (50, 99)],
                           lateral_vs_l0_abs_p50_p90=_vs_l0(ticks), quality_p50=None if not fits else
                           round(float(np.median([f['quality'] for f in fits])), 3))
    if last_gnss is not None:
        out['gnss'] = dict(counts=last_gnss.get('counts'), flags=last_gnss.get('flags'),
                           fence_tripped=last_gnss.get('fence_tripped'), aligned=last_gnss.get('aligned'),
                           course_drift_deg=last_gnss.get('course_drift_deg'),
                           disagreement_m=last_gnss.get('disagreement_m'))
    return out


def truth_errors(ticks, arrays):
    """Accuracy against the SIM truth (evaluation only) of L0 (the tick record's lat / head) and of the grid fit
    (its lateral / heading at the same tick, fit not older than 1 s) on the ticks where both exist: |error| p50 / p90.
    W0: the centre line is y = 0 along +x, so the true lateral offset is truth_y and the true heading error -truth_yaw."""
    t = np.asarray(arrays['t_s'], dtype=float)
    y = np.asarray(arrays['truth_y_m'], dtype=float)
    yaw = np.asarray(arrays['truth_yaw_rad'], dtype=float)
    rows = []
    for r in ticks:
        fit = (r.get('grid') or {}).get('fit')
        if not fit or fit.get('lateral_m') is None or fit.get('age_s', 9.) > 1. or r.get('lat') is None                 or r.get('head') is None or not r.get('T'):
            continue
        ty = float(np.interp(r['t'], t, y))
        th = -math.degrees(float(np.interp(r['t'], t, yaw)))
        rows.append((r['lat'] - ty, r['head'] - th, fit['lateral_m'] - ty, fit['heading_deg'] - th))
    if not rows:
        return None
    a = np.abs(np.asarray(rows))

    def q(col):
        return [round(float(np.percentile(a[:, col], p)), 4) for p in (50, 90)]
    return dict(ticks=len(rows), l0_lateral_abs_m=q(0), l0_heading_abs_deg=q(1), grid_lateral_abs_m=q(2),
                grid_heading_abs_deg=q(3))


def _vs_l0(ticks):
    d = [abs(r['grid']['fit']['lateral_m'] - r['lat']) for r in ticks
         if 'grid' in r and r['grid'].get('fit') and r['grid']['fit'].get('lateral_m') is not None
         and r.get('lat') is not None and r['grid']['fit'].get('age_s', 9.) <= 1.]
    return None if not d else [round(float(np.percentile(d, q)), 4) for q in (50, 90)]


def run_one(item):
    condition, seed, variant, gnss_spec, out_root, stream_condition = item
    S = A._sim_in_loop()
    started = time.perf_counter()
    ep = episode(condition, seed, variant, gnss_spec, stream_condition)
    result, facade = S.run_episode(ep, None, keep=True)
    try:
        if out_root is not None:
            result = facade.write(os.path.join(out_root, ep['name']), FM.printable(ep, condition))
        arrays = facade.trace_arrays()
        ticks = [r for r in facade.log.records if r.get('k') == 'tick']
        metrics = FM.measure(arrays, ticks, facade.summary(), ep['world']['markers'], condition.get('settle_m', 0.))
        metrics['after_terminal'] = FM.terminal_window(M.artefacts(facade, FM.printable(ep, condition)), metrics)
        receiver = facade.rig.gnss_report()
        accuracy = truth_errors(ticks, arrays)
        timing = facade.summary()['timing']
    finally:
        facade.rig.close()
    checks = FM.judge(condition['expect'], metrics)
    cmd = commands(ticks)
    return dict(condition=condition['name'], seed=seed, variant=variant, name=ep['name'], expect=condition['expect'],
                ok=all(c['ok'] for c in checks), failed=[c['check'] for c in checks if not c['ok']], metrics=metrics,
                layers=layer_blocks(ticks), accuracy=accuracy, receiver=None if receiver is None else dict(
                    counts=receiver['counts'], jump_stamps=receiver['jump_stamps']),
                commands_sha256=hashlib.sha256(json.dumps(cmd, separators=(',', ':')).encode('ascii')).hexdigest(),
                digests=result['digests'], step_ms=timing['step_ms'], wall_s=round(time.perf_counter() - started, 1))


def items(variants, only, gnss_seeds, out_dir):
    out = []
    for condition in FM.CONDITIONS:
        if only and condition['name'] not in only:
            continue
        for seed in condition['seeds']:
            for variant in variants:
                out.append((condition, seed, variant, {} if variant in ('gnss_log', 'all') else None, out_dir, None))
    nominal = FM.CONDITIONS[0]
    for gc in GNSS_CONDITIONS:
        if only and gc['name'] not in only:
            continue
        cond = dict(nominal, name=gc['name'])
        for seed in gnss_seeds:
            for variant in ('off', 'gnss_log', 'all'):
                if variant in variants:
                    out.append((cond, seed, variant, None if variant == 'off' else gc['gnss'], out_dir, nominal['name']))
    return out


def run_all(out_dir, workers=1, variants=tuple(VARIANTS), only=None, gnss_seeds=(0,)):
    started = time.perf_counter()
    todo = items(variants, only, gnss_seeds, os.path.join(out_dir, 'runs'))
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=min(4, workers)) as pool:
            rows = list(pool.map(run_one, todo))
    else:
        rows = [run_one(item) for item in todo]
    base = {(r['condition'], r['seed']): r for r in rows if r['variant'] == 'off'}
    for r in rows:
        ref = base.get((r['condition'], r['seed']))
        r['commands_equal_off'] = None if ref is None or r is ref else r['commands_sha256'] == ref['commands_sha256']
        r['trace_equal_off'] = (None if ref is None or r is ref else
                                r['digests']['trace_content_sha256'] == ref['digests']['trace_content_sha256'])
    report = dict(schema=SCHEMA, result_kind=A.RESULT_KIND, field_validated=False, kind='development comparison',
                  prereg=None, follow_criteria=FM.CRITERIA, variants={k: VARIANTS[k] for k in variants},
                  gnss_conditions=[g for g in GNSS_CONDITIONS if not only or g['name'] in only],
                  runs=len(rows), rows=rows, summary=summarise(rows), workers=workers,
                  wall_s=round(time.perf_counter() - started, 1))
    os.makedirs(out_dir, exist_ok=True)
    raw = json.dumps(report, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False, default=str) + '\n'
    with open(os.path.join(out_dir, 'PERCEPTION_COMPARE.json'), 'wb') as fh:
        fh.write(raw.encode('utf-8'))
    return report


def summarise(rows):
    out = {}
    for variant in sorted({r['variant'] for r in rows}):
        part = [r for r in rows if r['variant'] == variant]
        follow = [r for r in part if not r['condition'].startswith('gnss_')]
        rm = [r['metrics'].get('lateral_rmse_m') for r in follow if r['metrics'].get('lateral_rmse_m') is not None]
        out[variant] = dict(runs=len(part), passed=sum(1 for r in part if r['ok']),
                            failed=sorted('%s s%d: %s' % (r['condition'], r['seed'], '; '.join(r['failed']))
                                          for r in part if not r['ok']),
                            commands_equal_off=[r['commands_equal_off'] for r in part].count(True),
                            commands_differ_off=[r['commands_equal_off'] for r in part].count(False),
                            lateral_rmse_m_mean=None if not rm else round(float(np.mean(rm)), 4),
                            lateral_rmse_m_max=None if not rm else round(float(np.max(rm)), 4),
                            step_ms_p99_max=max((r['step_ms'].get('p99') or 0.) for r in part),
                            accuracy=_accuracy_summary(part))
    return out


def _accuracy_summary(part):
    """Median over the runs of the per-run p50 / p90 truth errors (runs with grid fits only)."""
    rows = [r['accuracy'] for r in part if r.get('accuracy')]
    if not rows:
        return None
    keys = ('l0_lateral_abs_m', 'l0_heading_abs_deg', 'grid_lateral_abs_m', 'grid_heading_abs_deg')
    return dict(runs=len(rows), **{k: [round(float(np.median([x[k][i] for x in rows])), 4) for i in (0, 1)]
                                   for k in keys})


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--out', required=True)
    p.add_argument('--workers', type=int, default=1)
    p.add_argument('--variants', nargs='*', default=list(VARIANTS))
    p.add_argument('--only', nargs='*')
    p.add_argument('--gnss-seeds', nargs='*', type=int, default=[0])
    args = p.parse_args(argv)
    from steering.scene_spec import inside_repository
    if inside_repository(args.out):
        raise SystemExit('refused: %s lies inside the repository (runs live outside)' % args.out)
    if 'off' not in args.variants:
        args.variants = ['off'] + list(args.variants)
    report = run_all(args.out, args.workers, tuple(args.variants), args.only, tuple(args.gnss_seeds))
    for r in report['rows']:
        m = r['metrics']
        g = r['layers'].get('gnss') or {}
        print('%-4s %-18s s%d %-9s end %-14s rmse %-7s max %-7s e %-6s cmd=off %-5s jump %s/%s fence %s %s' % (
            'ok' if r['ok'] else 'FAIL', r['condition'], r['seed'], r['variant'],
            m.get('end_reason') or m.get('first_pause'), m.get('lateral_rmse_m'), m.get('lateral_max_m'),
            m.get('row_end_error_m'), r['commands_equal_off'], (g.get('counts') or {}).get('jump'),
            (r['receiver'] or {}).get('counts', {}).get('jumped'), g.get('fence_tripped'),
            '' if r['ok'] else 'failed: ' + '; '.join(r['failed'])))
    print(json.dumps(report['summary'], indent=1))
    print('%d runs, %.0f s' % (report['runs'], report['wall_s']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
