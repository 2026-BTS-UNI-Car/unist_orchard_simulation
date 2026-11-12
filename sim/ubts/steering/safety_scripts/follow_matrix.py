"""Single-row following matrix of the row-following controller on the T1 rig (fast SIM, executor in the loop).

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_KO.md and PREREG_STEER_FOLLOW.json (the JSON is prereg() of this
module: world, conditions, criteria). Do not change CRITERIA or CONDITIONS after a run; an amendment is written in the
pre-registration document first.

Chain (tools/row_executor/sim_in_loop_steer.py): the real executor loop and Runner (front_steer, MOTOR, single-row
mode) with the controller slot {'kind': 'row_follow'} (closed_loop/steer_control.py) <-> bridge client <-> host bridge
core <-> MCU firmware and actuator model <-> steered plant <-> W0 floor with two lines of posts <-> S2 scan and IMU.
World: posts every POST_SPACING_M on y = +-ROW_HALF_WIDTH_M from x = ROW_START_X_M to ROW_END_X_M (the last post line:
the row end of design v1 1.1). The vehicle starts at x = 0 inside the row, operator emulation of the rig (Start, Arm,
one press, no re-press).

Metrics come from the truth columns of the rig trace (evaluation only, A13 role 'evaluator'): lateral and heading
error of the footprint centre against the centre line y = 0 from the first to the last tick with a drive command
(offset starts: RMSE and p95 after SETTLE_M of travel; the maxima over the whole drive), the rest position
e = front bumper x - row end x, the smallest footprint clearance to a post, the end reason, pauses after the drive
started, and the A22 rule after the terminal moment (measure.command_after). Every number is a placeholder of the SIM
(자리값 기반 SIM 결과), not field performance.

CLI (from sim/ubts): python -B -m steering.safety_scripts.follow_matrix --out <folder outside the repository>
                     [--workers N (<= 4)] [--only NAME ...] [--seeds N ...]   |   --prereg   (print the JSON)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

from . import acceptance as A
from . import catalogue as CAT
from . import measure as M

SCHEMA = 'ubts.steer_sim.follow_matrix.v1'
PREREG_ID = 'steer_follow_v1'
ROW_HALF_WIDTH_M = 1.75
POST_SPACING_M = 1.5
POST_RADIUS_M = .12
ROW_START_X_M = -4.5
ROW_END_X_M = 24.0
FRONT_BUMPER_M = .61                 # footprint length 1.22 m / 2 (profile nominal)
HALF_WIDTH_M = .35                   # footprint width 0.70 m / 2
DURATION_S = 150.
FAULT_AT_S = 20.
CONTROLLER = {'kind': 'row_follow'}
# Registered criteria (sources in the pre-registration document). Do not change after a run.
CRITERIA = dict(lateral_rmse_m=.20, lateral_p95_m=.35, lateral_max_m=.50, heading_rmse_deg=5., heading_max_deg=15.,
                row_end_error_m=1.0, speed_max_m_s=.50, min_clearance_m=0., after_terminal_window_s=3.,
                stale_drive_zero_s=.1, rest_speed_m_s=.02)
SETTLE_M = 6.0                       # offset starts: RMSE / p95 are taken after this travelled distance (3 look-aheads)
# name, seeds, expectation, changes
CONDITIONS = (
    dict(name='nominal', seeds=(0, 1, 2, 3, 4), expect='row_end'),
    dict(name='lateral_p040', seeds=(0,), expect='row_end', start_pose=(0., .40, 0.), settle_m=SETTLE_M),
    dict(name='lateral_n040', seeds=(0,), expect='row_end', start_pose=(0., -.40, 0.), settle_m=SETTLE_M),
    dict(name='heading_p8', seeds=(0,), expect='row_end', start_pose=(0., 0., 8.), settle_m=SETTLE_M),
    dict(name='heading_n8', seeds=(0,), expect='row_end', start_pose=(0., 0., -8.), settle_m=SETTLE_M),
    dict(name='steer_zero_p2', seeds=(0,), expect='row_end', profile={'steer_zero_offset_deg': 2.0}),
    dict(name='steer_zero_n2', seeds=(0,), expect='row_end', profile={'steer_zero_offset_deg': -2.0}),
    dict(name='side_pull', seeds=(0,), expect='row_end', wander_mean_deg_per_m=.6),
    dict(name='lidar_dropout_0p2', seeds=(0,), expect='row_end',
         faults=[dict(kind='sensor_blackout', at_s=FAULT_AT_S, params=dict(duration_s=.2, which='lidar'))]),
    dict(name='lidar_stale_1p0', seeds=(0,), expect='stale_lidar',
         faults=[dict(kind='sensor_blackout', at_s=FAULT_AT_S, params=dict(duration_s=1., which='lidar'))]),
    dict(name='obstacle', seeds=(0,), expect='obstacle_ahead', extra_posts=[[12.0, 0.0, POST_RADIUS_M]]),
)


def posts(extra=()):
    count = int(round((ROW_END_X_M - ROW_START_X_M) / POST_SPACING_M)) + 1
    out = [[round(ROW_START_X_M + POST_SPACING_M * k, 3), side * ROW_HALF_WIDTH_M, POST_RADIUS_M]
           for k in range(count) for side in (1, -1)]
    return out + [list(p) for p in extra]


def prereg():
    """The machine-readable pre-registration (docs/steering/PREREG_STEER_FOLLOW.json)."""
    return dict(schema='ubts.steer_sim.prereg_follow.v1', id=PREREG_ID, result_kind='placeholder-based SIM result',
                simulator='F-SIM T1 rig, executor in the loop (tools/row_executor/sim_in_loop_steer.py)',
                world=dict(kind='W0', row_half_width_m=ROW_HALF_WIDTH_M, post_spacing_m=POST_SPACING_M,
                           post_radius_m=POST_RADIUS_M, row_start_x_m=ROW_START_X_M, row_end_x_m=ROW_END_X_M,
                           posts=len(posts())),
                controller=CONTROLLER, duration_s=DURATION_S, fault_at_s=FAULT_AT_S,
                footprint=dict(front_bumper_m=FRONT_BUMPER_M, half_width_m=HALF_WIDTH_M),
                criteria=CRITERIA, settle_m=SETTLE_M,
                conditions=[dict(c, seeds=list(c['seeds'])) for c in CONDITIONS],
                runs=sum(len(c['seeds']) for c in CONDITIONS))


def spec(condition, seed):
    """Episode spec of one run (the profile override is an object: the spec is not JSON-able then)."""
    x, y, yaw = condition.get('start_pose', (0., 0., 0.))
    rig = {}
    overrides = dict(condition.get('profile') or {})
    if 'wander_mean_deg_per_m' in condition:
        from steering.vehicle_profile import load_profile
        wander = dict(load_profile().value('curvature_wander'))
        wander['mean_deg_per_m'] = float(condition['wander_mean_deg_per_m'])
        overrides['curvature_wander'] = wander
    if overrides:
        from steering.vehicle_profile import load_profile
        rig['profile'] = load_profile().with_overrides(overrides, label=condition['name'])
    world = {'kind': 'W0', 'markers': posts(condition.get('extra_posts') or ())}
    return CAT.episode('follow_%s_s%d' % (condition['name'], seed), controller=dict(CONTROLLER), world=world,
                       faults=[dict(f) for f in condition.get('faults') or ()], rig=rig, duration_s=DURATION_S,
                       seed=int(seed), start_pose=(x, y, yaw), auto_repress=False)


def clearance(x, y, yaw, markers):
    """Smallest distance [m] from the footprint rectangle to a post surface over the given poses."""
    best = math.inf
    c, s = np.cos(yaw), np.sin(yaw)
    for px, py, radius in markers:
        dx, dy = px - x, py - y
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        gap = np.hypot(np.maximum(np.abs(lx) - FRONT_BUMPER_M, 0.), np.maximum(np.abs(ly) - HALF_WIDTH_M, 0.)) - radius
        best = min(best, float(gap.min()))
    return best


def measure(arrays, ticks, summary, markers, settle_m=0.):
    """Metrics of one run from the trace (truth: evaluation only), the executor ticks and the state changes."""
    t = np.asarray(arrays['t_s'], dtype=float)
    x, y = np.asarray(arrays['truth_x_m'], dtype=float), np.asarray(arrays['truth_y_m'], dtype=float)
    yaw = np.asarray(arrays['truth_yaw_rad'], dtype=float)
    v = np.asarray(arrays['truth_v_m_s'], dtype=float)
    throttle = np.asarray(arrays['ctrl_throttle_pct'], dtype=float)
    driving = np.flatnonzero(throttle > 0)
    out = dict(end_reason=summary.get('end_reason'), ended_at_s=summary.get('ended_at_s'),
               final_state=ticks[-1].get('state') if ticks else None,
               final_reason=ticks[-1].get('reason') if ticks else None, drove=bool(len(driving)),
               min_clearance_m=round(clearance(x[::5], y[::5], yaw[::5], markers), 4),
               rest_speed_m_s=round(float(abs(v[-1])), 4), rest_x_m=round(float(x[-1]), 3),
               rest_y_m=round(float(y[-1]), 3), rest_yaw_deg=round(float(np.degrees(yaw[-1])), 3),
               row_end_error_m=round(float(x[-1] + FRONT_BUMPER_M * math.cos(yaw[-1]) - ROW_END_X_M), 3),
               pauses_after_start=[], first_pause=None)
    if not len(driving):
        return out
    a, b = int(driving[0]), int(driving[-1])
    t_a = float(t[a])
    pauses = [(float(e[0]), e[2]) for e in summary.get('events', ()) if e[1] == 'PAUSED' and float(e[0]) > t_a]
    out['pauses_after_start'] = sorted({reason for _, reason in pauses})
    out['first_pause'] = None if not pauses else [round(pauses[0][0], 3), pauses[0][1]]
    path = np.concatenate(([0.], np.cumsum(np.hypot(np.diff(x[a:b + 1]), np.diff(y[a:b + 1])))))
    k = a + int(np.searchsorted(path, float(settle_m)))
    out.update(drive_start_s=round(t_a, 2), drive_stop_s=round(float(t[b]), 2), distance_m=round(float(path[-1]), 2),
               lateral_max_m=round(float(np.max(np.abs(y[a:b + 1]))), 4),
               heading_max_deg=round(float(np.degrees(np.max(np.abs(yaw[a:b + 1])))), 3),
               speed_mean_m_s=round(float(np.mean(v[a:b + 1])), 3), speed_max_m_s=round(float(np.max(v[a:b + 1])), 3),
               settle_m=float(settle_m))
    if k < b:
        out.update(lateral_rmse_m=round(float(np.sqrt(np.mean(y[k:b + 1] ** 2))), 4),
                   lateral_p95_m=round(float(np.percentile(np.abs(y[k:b + 1]), 95)), 4),
                   heading_rmse_deg=round(float(np.degrees(np.sqrt(np.mean(yaw[k:b + 1] ** 2)))), 3))
    stale = next((r for r in ticks if (r.get('lidar_age') or 0.) > .45 and r['t'] >= t_a), None)
    if stale is not None:
        zero = next((r for r in ticks if r['t'] >= stale['t'] and not (r.get('T') or 0)), None)
        out['stale_first_tick_s'] = stale['t']
        out['stale_drive_zero_s'] = None if zero is None else round(zero['t'] - stale['t'], 3)
        out['restart_after_stale'] = bool(zero is None or any((r.get('T') or 0) > 0 for r in ticks
                                                              if r['t'] > zero['t']))
    return out


def terminal_window(art, metrics):
    """The A22 rule (measure.command_after) from the terminal moment, or from the first pause of a run that pauses."""
    t0 = metrics.get('ended_at_s')
    if t0 is None and metrics.get('first_pause'):
        t0 = metrics['first_pause'][0]
    window = None if t0 is None else M.command_after(art, float(t0), CRITERIA['after_terminal_window_s'])
    if window is None:
        return dict(passed=False, complete=False, from_s=t0)
    return dict(passed=window['passed'], complete=window['complete'], from_s=round(float(t0), 3),
                max_target_move_deg=window['max_target_move_deg'], tolerance_deg=window['tolerance_deg'],
                executor_command_move_deg=window['executor_command_move_deg'], auto_centre=window['auto_centre'])


def judge(expect, m):
    """Registered checks of one run: list of dict(check, ok, measured, limit)."""
    c = CRITERIA
    checks = []

    def add(name, ok, measured, limit):
        checks.append(dict(check=name, ok=bool(ok), measured=measured, limit=limit))

    def below(name, key, unit):
        value = m.get(key)
        add(name, value is not None and value <= c[key], value, '<= %g %s' % (c[key], unit))
    add('no contact', m['min_clearance_m'] > c['min_clearance_m'], m['min_clearance_m'],
        '> %g m' % c['min_clearance_m'])
    add('at rest at the end', m['rest_speed_m_s'] <= c['rest_speed_m_s'], m['rest_speed_m_s'],
        '<= %g m/s' % c['rest_speed_m_s'])
    add('no guard violation', not str(m['end_reason']).startswith('guard_violation'), m['end_reason'], 'none')
    after = m.get('after_terminal') or {}
    add('no commanded steering move after the terminal moment (A22 rule)', after.get('passed') is True,
        dict((k, after.get(k)) for k in ('passed', 'complete', 'max_target_move_deg', 'executor_command_move_deg')),
        'passed')
    if expect == 'row_end':
        add('ends at the row end', m['end_reason'] == 'row_end', m['end_reason'], 'row_end')
        add('no pause after the drive started', not m['pauses_after_start'], m['pauses_after_start'], 'none')
        below('lateral RMSE', 'lateral_rmse_m', 'm')
        below('lateral p95', 'lateral_p95_m', 'm')
        below('lateral max', 'lateral_max_m', 'm')
        below('heading RMSE', 'heading_rmse_deg', 'deg')
        below('heading max', 'heading_max_deg', 'deg')
        below('true speed max', 'speed_max_m_s', 'm/s')
        add('row end stop position e', abs(m['row_end_error_m']) <= c['row_end_error_m'], m['row_end_error_m'],
            '[-%g, +%g] m' % (c['row_end_error_m'], c['row_end_error_m']))
    elif expect == 'stale_lidar':
        first = m.get('first_pause')
        add('first pause is stale_lidar', bool(first) and first[1] == 'stale_lidar', first, 'stale_lidar')
        add('drive command 0 after the scan is stale', m.get('stale_drive_zero_s') is not None
            and m['stale_drive_zero_s'] <= c['stale_drive_zero_s'], m.get('stale_drive_zero_s'),
            '<= %g s' % c['stale_drive_zero_s'])
        add('no restart', m.get('restart_after_stale') is False, m.get('restart_after_stale'), False)
    else:
        add('ends with %s' % expect, m['end_reason'] == expect, m['end_reason'], expect)
    return checks


def printable(episode, condition):
    """The episode spec for RESULT.json: the profile object replaced by its overrides."""
    out = dict(episode, rig=dict(episode['rig']))
    if 'profile' in out['rig']:
        out['rig']['profile'] = dict(overrides=condition.get('profile') or {},
                                     wander_mean_deg_per_m=condition.get('wander_mean_deg_per_m'))
    return out


def run_one(item):
    condition, seed, out_root = item
    S = A._sim_in_loop()
    started = time.perf_counter()
    episode = spec(condition, seed)
    result, facade = S.run_episode(episode, None, keep=True)
    try:
        if out_root is not None:
            result = facade.write(os.path.join(out_root, episode['name']), printable(episode, condition))
        arrays = facade.trace_arrays()
        ticks = [r for r in facade.log.records if r.get('k') == 'tick']
        metrics = measure(arrays, ticks, facade.summary(), episode['world']['markers'], condition.get('settle_m', 0.))
        metrics['after_terminal'] = terminal_window(M.artefacts(facade, printable(episode, condition)), metrics)
    finally:
        facade.rig.close()
    checks = judge(condition['expect'], metrics)
    return dict(condition=condition['name'], seed=seed, name=episode['name'], expect=condition['expect'],
                ok=all(c['ok'] for c in checks), checks=checks, metrics=metrics, digests=result['digests'],
                config_digest=result['config_digest'], wall_s=round(time.perf_counter() - started, 1))


def run_all(out_dir, workers=1, only=None, seeds=None):
    started = time.perf_counter()
    items = [(c, seed, out_dir) for c in CONDITIONS for seed in c['seeds']
             if (not only or c['name'] in only) and (not seeds or seed in seeds)]
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=min(4, workers)) as pool:
            rows = list(pool.map(run_one, items))
    else:
        rows = [run_one(item) for item in items]
    report = dict(schema=SCHEMA, prereg=PREREG_ID, result_kind=A.RESULT_KIND, field_validated=False,
                  complete=not only and not seeds, criteria=CRITERIA, runs=len(rows), passed=sum(1 for r in rows if r['ok']),
                  ok=bool(rows) and all(r['ok'] for r in rows), rows=rows, workers=workers,
                  wall_s=round(time.perf_counter() - started, 1))
    os.makedirs(out_dir, exist_ok=True)
    raw = json.dumps(report, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False, default=str) + '\n'
    with open(os.path.join(out_dir, 'FOLLOW_MATRIX.json'), 'wb') as fh:
        fh.write(raw.encode('utf-8'))
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--out')
    p.add_argument('--workers', type=int, default=1)
    p.add_argument('--only', nargs='*')
    p.add_argument('--seeds', nargs='*', type=int)
    p.add_argument('--prereg', action='store_true')
    args = p.parse_args(argv)
    if args.prereg:
        sys.stdout.buffer.write((json.dumps(prereg(), sort_keys=True, indent=1) + '\n').encode('utf-8'))   # LF
        return 0
    if not args.out:
        raise SystemExit('--out is required')
    from steering.scene_spec import inside_repository
    if inside_repository(args.out):
        raise SystemExit('refused: %s lies inside the repository (runs live outside)' % args.out)
    report = run_all(args.out, args.workers, args.only, args.seeds)
    for row in report['rows']:
        failed = [c['check'] for c in row['checks'] if not c['ok']]
        m = row['metrics']
        print('%-4s %-18s s%d end %s e %s rmse %s p95 %s max %s head %s/%s v %s clear %s %s' % (
            'ok' if row['ok'] else 'FAIL', row['condition'], row['seed'], m.get('end_reason') or m.get('first_pause'),
            m.get('row_end_error_m'), m.get('lateral_rmse_m'), m.get('lateral_p95_m'), m.get('lateral_max_m'),
            m.get('heading_rmse_deg'), m.get('heading_max_deg'), m.get('speed_max_m_s'), m.get('min_clearance_m'),
            '' if not failed else 'failed: ' + '; '.join(failed)))
    print('%s: %d of %d runs pass%s, %.0f s' % ('PASS' if report['ok'] else 'FAIL', report['passed'], report['runs'],
                                                '' if report['complete'] else ' (partial: not the whole matrix)',
                                                report['wall_s']))
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
