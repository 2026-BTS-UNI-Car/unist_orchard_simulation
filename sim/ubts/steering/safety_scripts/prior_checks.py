"""Actuator prior checks on the T1 rig (PC-06; design v2 V2-1 item 2; SIM_ACCEPTANCE amendment 33;
docs/steering/ACTUATOR_PRIOR_KO.md).

The executor runs in the loop (tools/row_executor/sim_in_loop_steer.py) with the straight-hold script at 40 % on the W0
floor (two lines of posts). Per seed, with one plant stream for every run of the seed:
  off            the default executor (prior_enabled False)
  monitor        prior_enabled, weight 0, the profile's nominal table: the same decisions as 'off' (T1 identity) and
                 the calibration run - its online scale (LiDAR / table, the executor's own value) becomes speed_scale
                 of the runs below, as an operator would store it after a drive
  table_ok       weight 0, calibrated scale, table x 1.0: table_mismatch must stay off
  table_plus30   ... table x 1.3: table_mismatch must be on
  table_minus30  ... table x 0.7: table_mismatch must be on
  blackout       weight 1, calibrated scale, LiDAR blackout of 1.0 s while driving (rig fault sensor_blackout):
                 the predicted position at the end of the blackout against the truth
  blackout_w0    the same blackout with weight 0 (the original estimate, reported for comparison)
Judged (LIMITS, registered before the first run):
  identity       off vs monitor: serial log, trace (common columns) and executor ticks (without the 'prior' block) equal
  prediction     |predicted displacement - true displacement| over the blackout, in the body frame of the blackout
                 start, <= prediction_error_m
  prior_only     executor ticks that drive or move the steering target while the estimate has run on the prior for
                 more than predict_max_s: 0 (every run)
  mismatch       the three table runs as above
The truth columns of the rig trace are read here only (evaluation code, A13 role 'evaluator'); the calibration value
comes from the executor's own log. Every number is a placeholder of the SIM (자리값 기반 SIM 결과), not a field result.

CLI (from sim/ubts): python -B -m steering.safety_scripts.prior_checks --out <folder outside the repository>
                     [--seeds 0 1 2 3 4] [--workers N]
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
from . import catalogue as CAT

SCHEMA = 'ubts.steer_sim.prior_checks.v1'
SEEDS = (0, 1, 2, 3, 4)
DRIVE_PCT = 40
DISTANCE_M = 10.
DURATION_S = 60.
BLACKOUT_AT_S = 16.
BLACKOUT_S = 1.
# Registered limits (SIM_ACCEPTANCE amendment 33). Do not change after a run.
LIMITS = dict(prediction_error_m=.15, prior_only_drive_ticks=0, table_error=.30, predict_max_s=1.)
SCALE_RANGE = (.5, 1.3)                  # executor config range of prior_speed_scale (field tuning k_field)
RUNS = ('off', 'monitor', 'table_ok', 'table_plus30', 'table_minus30', 'blackout', 'blackout_w0')
EST_COLUMNS = ('ctrl_est_x_m', 'ctrl_est_y_m', 'ctrl_est_yaw_rad', 'ctrl_est_predict_s', 'ctrl_est_odom_valid')


def spec(run, seed, scale=1., table=None):
    """The episode spec of one run (module docstring). table: explicit prior table (None = the profile's nominal one)."""
    config = {}
    if run != 'off':
        weight = 1. if run == 'blackout' else 0.
        config = dict(prior_enabled=True, prior_weight=weight, prior_speed_scale=float(scale))
        if table is not None:
            config['prior_speed_table'] = [list(point) for point in table]
    faults = []
    if run.startswith('blackout'):
        faults = [CAT.fault('sensor_blackout', at_s=BLACKOUT_AT_S, duration_s=BLACKOUT_S, which='lidar')]
    return CAT.episode('prior_%s_s%d' % (run, seed), controller=dict(CAT.HOLD_40, drive_pct=DRIVE_PCT,
                                                                     distance_m=DISTANCE_M),
                       faults=faults, config=config, duration_s=DURATION_S, post_end_s=None, seed=int(seed),
                       stream_key='prior_checks_s%d' % seed)


def _ticks(facade):
    return [r for r in facade.log.records if r.get('k') == 'tick']


def _row(t_s, t):
    return int(np.searchsorted(t_s, t + 1e-9, side='right') - 1)


def _body(dx, dy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def displacement_error(arrays, t0, t1):
    """Predicted against true displacement of the base over [t0, t1], each in its own body frame at t0:
    dict(error_m, along_error_m, lateral_error_m, true_m, predicted_m). Evaluation only."""
    t_s = np.asarray(arrays['t_s'], dtype=float)
    a, b = _row(t_s, t0), _row(t_s, t1)
    est = [np.asarray(arrays[name], dtype=float) for name in EST_COLUMNS[:3]]
    ref = [np.asarray(arrays[name], dtype=float) for name in ('truth_x_m', 'truth_y_m', 'truth_yaw_rad')]
    e = _body(est[0][b] - est[0][a], est[1][b] - est[1][a], est[2][a])
    r = _body(ref[0][b] - ref[0][a], ref[1][b] - ref[1][a], ref[2][a])
    return dict(error_m=round(math.hypot(e[0] - r[0], e[1] - r[1]), 4), along_error_m=round(e[0] - r[0], 4),
                lateral_error_m=round(e[1] - r[1], 4), true_m=round(math.hypot(*r), 4),
                predicted_m=round(math.hypot(*e), 4))


def prior_only_drive_ticks(ticks, predict_max_s=LIMITS['predict_max_s']):
    """Ticks that drive (throttle > 0) or change the steering target while the estimate has run on the prior for more
    than predict_max_s, and the longest prediction time of a driving tick."""
    count, longest, previous = 0, 0., None
    for tick in ticks:
        prior = tick.get('prior') or {}
        steer = tick.get('S')
        moved = previous is not None and steer is not None and steer != previous
        previous = steer if steer is not None else previous
        if tick.get('state') != 'ACTIVE' or not ((tick.get('T') or 0) > 0 or moved):
            continue
        ps = prior.get('ps') or 0.
        longest = max(longest, ps)
        if ps > predict_max_s + 1e-9:
            count += 1
    return count, round(longest, 4)


def last_monitor(ticks):
    """The 'prior' block of the last driving tick (None without one)."""
    for tick in reversed(ticks):
        if tick.get('state') == 'ACTIVE' and (tick.get('T') or 0) > 0 and tick.get('prior'):
            return tick['prior']
    return None


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), default=str).encode()).hexdigest()


def run_one(run, seed, scale=1., table=None, out_dir=None, native_executable=None, build_dir=None):
    """Run one episode and reduce it (a picklable dict; the trace stays in out_dir when given)."""
    S = A._sim_in_loop()
    started = time.perf_counter()
    episode = spec(run, seed, scale, table)
    result, facade = S.run_episode(episode, out_dir, native_executable, build_dir, keep=True)
    try:
        arrays = facade.trace_arrays()
        ticks = _ticks(facade)
        summary = facade.summary()
        common = {k: v for k, v in arrays.items() if k != 'meta_json' and k not in EST_COLUMNS}
        row = dict(run=run, seed=seed, name=episode['name'], scale=scale, wall_s=None,
                   end_reason=summary.get('end_reason'), ended_at_s=summary.get('ended_at_s'),
                   config_digest=result['config_digest'], digests=result['digests'],
                   serial_sha256=result['digests']['serial_log_sha256'],
                   trace_common_sha256=S.TR.content_sha256(common),
                   ticks_without_prior_sha256=_digest([{k: v for k, v in facade.log._strip(t).items() if k != 'prior'}
                                                       for t in ticks]),
                   ticks=len(ticks), pauses=sorted({t['reason'] for t in ticks if t.get('state') == 'PAUSED'}),
                   true_distance_m=round(float(np.asarray(arrays['truth_distance_m'])[-1]), 3))
        if run != 'off':
            count, longest = prior_only_drive_ticks(ticks)
            monitor = last_monitor(ticks)
            row.update(prior_only_drive_ticks=count, longest_prediction_of_a_driving_tick_s=longest,
                       monitor=monitor, prior_fed=facade.executor.runner.stats.get('prior_fed', 0))
        if run.startswith('blackout'):
            t0, t1 = BLACKOUT_AT_S, BLACKOUT_AT_S + BLACKOUT_S
            row['blackout'] = displacement_error(arrays, t0, t1)
            row['blackout']['after_0p5_s'] = displacement_error(arrays, t0, t1 + .5)
            t_s = np.asarray(arrays['t_s'], dtype=float)
            valid = np.asarray(arrays['ctrl_est_odom_valid'], dtype=float)
            after = np.flatnonzero((t_s > t1) & (valid > .5))
            lost = np.flatnonzero((t_s > t0) & (t_s <= t1 + 2.) & (valid < .5))
            row['blackout'].update(
                first_pause=next(([t['t'], t['reason']] for t in ticks if t['t'] >= t0 and t.get('state') == 'PAUSED'),
                                 None),
                scan_match_lost_s=round(float(len(lost)) * float(np.median(np.diff(t_s))), 3),
                first_match_after_s=None if not len(after) else round(float(t_s[after[0]] - t1), 3),
                predict_s_at_end=round(float(np.asarray(arrays['ctrl_est_predict_s'])[_row(t_s, t1)]), 3))
        row['wall_s'] = round(time.perf_counter() - started, 2)
        return row
    finally:
        facade.rig.close()


def _seed_job(item):
    seed, out_root, native_executable, build_dir = item
    S = A._sim_in_loop()
    from steering.vehicle_profile import load_profile
    profile = load_profile()

    def out(run):
        return None if out_root is None else os.path.join(out_root, 'prior_%s_s%d' % (run, seed))
    rows = {'off': run_one('off', seed, out_dir=out('off'), native_executable=native_executable, build_dir=build_dir),
            'monitor': run_one('monitor', seed, out_dir=out('monitor'), native_executable=native_executable,
                               build_dir=build_dir)}
    online = (rows['monitor'].get('monitor') or {}).get('k')
    scale = None if online is None else min(SCALE_RANGE[1], max(SCALE_RANGE[0], float(online)))
    rows['calibration'] = dict(online_scale=online, speed_scale=scale,
                               clamped=bool(online is not None and scale != float(online)))
    if scale is None:
        return seed, rows
    for run, factor in (('table_ok', 1.), ('table_plus30', 1. + LIMITS['table_error']),
                        ('table_minus30', 1. - LIMITS['table_error'])):
        rows[run] = run_one(run, seed, scale, S.profile_speed_table(profile, factor=factor), out(run), native_executable,
                            build_dir)
    for run in ('blackout', 'blackout_w0'):
        rows[run] = run_one(run, seed, scale, None, out(run), native_executable, build_dir)
    return seed, rows


def judge(by_seed):
    """The registered checks over every seed (module docstring): dict(checks=[...], ok)."""
    checks = []

    def add(name, ok, measured, limit):
        checks.append(dict(check=name, ok=bool(ok), measured=measured, limit=limit))
    for seed, rows in sorted(by_seed.items()):
        off, mon = rows['off'], rows['monitor']
        same = {k: off[k] == mon[k] for k in ('serial_sha256', 'trace_common_sha256', 'ticks_without_prior_sha256')}
        add('s%d identity (weight 0 = the default decisions)' % seed, all(same.values()), same, 'all equal')
        if rows['calibration']['speed_scale'] is None:
            add('s%d calibration (online scale of the monitor run)' % seed, False, rows['calibration'], 'a value')
            continue
        for run, want in (('table_ok', False), ('table_plus30', True), ('table_minus30', True)):
            block = rows[run].get('monitor') or {}
            add('s%d %s: table_mismatch %s' % (seed, run, 'on' if want else 'off'), block.get('mismatch') is want,
                dict(mismatch=block.get('mismatch'), online_scale=block.get('k'), monitor_s=block.get('mon')), want)
        error = rows['blackout']['blackout']['error_m']
        add('s%d blackout 1.0 s: predicted position error at the end' % seed, error <= LIMITS['prediction_error_m'],
            rows['blackout']['blackout'], '<= %g m' % LIMITS['prediction_error_m'])
        for run in RUNS[1:]:
            count = rows[run]['prior_only_drive_ticks']
            add('s%d %s: driving ticks on the prior alone beyond %g s' % (seed, run, LIMITS['predict_max_s']),
                count == LIMITS['prior_only_drive_ticks'],
                dict(ticks=count, longest_s=rows[run]['longest_prediction_of_a_driving_tick_s']), 0)
    return dict(checks=checks, ok=bool(checks) and all(c['ok'] for c in checks))


def run_all(out_dir=None, seeds=SEEDS, workers=1, native_executable=None, build_dir=None, traces=False):
    started = time.perf_counter()
    items = [(int(seed), out_dir if traces else None, native_executable, build_dir) for seed in seeds]
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=workers) as pool:
            done = list(pool.map(_seed_job, items))
    else:
        done = [_seed_job(item) for item in items]
    by_seed = dict(done)
    verdict = judge(by_seed)
    report = dict(schema=SCHEMA, result_kind=A.RESULT_KIND, field_validated=False, limits=LIMITS,
                  setup=dict(drive_pct=DRIVE_PCT, distance_m=DISTANCE_M, duration_s=DURATION_S,
                             blackout_at_s=BLACKOUT_AT_S, blackout_s=BLACKOUT_S, seeds=list(seeds), workers=workers),
                  ok=verdict['ok'], checks=verdict['checks'], seeds={str(k): v for k, v in sorted(by_seed.items())},
                  wall_s=round(time.perf_counter() - started, 1))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        raw = json.dumps(report, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False, default=str) + '\n'
        with open(os.path.join(out_dir, 'PRIOR_CHECKS.json'), 'wb') as fh:
            fh.write(raw.encode('utf-8'))
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--out', required=True)
    p.add_argument('--seeds', nargs='*', type=int, default=list(SEEDS))
    p.add_argument('--workers', type=int, default=1)
    p.add_argument('--traces', action='store_true', help='also write TRACE / SERIAL / EXECUTOR of every run')
    args = p.parse_args(argv)
    from steering.scene_spec import inside_repository
    if inside_repository(args.out):
        raise SystemExit('refused: %s lies inside the repository (runs live outside)' % args.out)
    report = run_all(args.out, args.seeds, args.workers, traces=args.traces)
    for check in report['checks']:
        print('%-4s %s :: %s' % ('ok' if check['ok'] else 'FAIL', check['check'],
                                 json.dumps(check['measured'], default=str)))
    print('%s: %d checks, %d failed, %.1f s' % ('PASS' if report['ok'] else 'FAIL', len(report['checks']),
                                                sum(1 for c in report['checks'] if not c['ok']), report['wall_s']))
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
