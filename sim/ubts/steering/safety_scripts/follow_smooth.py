"""Row following with the vehicle's SMOOTH steering firmware r5 (sim-smooth-model, 2026-11-17).

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_SMOOTH_KO.md and PREREG_STEER_FOLLOW_SMOOTH.json (= prereg() of this
module; sim/ubts/test_steering_follow_smooth.py compares them). Do not change PARTS, conditions, seeds, criteria or the
executor and controller settings after a run; an amendment is written in the pre-registration document first.

Profile: front_steer_v2_field_20261116 (schema v5; the measured dimensions of v1, the board's NVS calibration 1475 /
2017 / 2513, steer_fw_mode smooth_r5: the SIM ESP emulator runs the SMOOTH hold of the firmware r5 with the SM values
saved on the board, docs/steering/SMOOTH_R5_SIM_KO.md; the steering motor rate fitted to the r5 lifted-wheel steps).
The executor takes the board's measured SMOOTH step response as its 'measured' steering rate
(steering.smooth_hold.executor_values, about 12.4 deg/s); that passes the steering GO rule (8 deg/s), so unlike
follow_realdims no threshold is changed. The controller is row_follow with the profile wheelbase.

Parts (simulator and measuring rules of follow_matrix.py and follow_wide.py; the same criteria as follow_realdims):
  matrix   the 15 runs of follow_matrix (steer_follow_v1 conditions), executor configuration as the rig makes it
  wide     follow_realdims.WIDE_SUBSET of follow_wide (steer_follow_wide_v2 conditions, round 2 measuring rules) x
           follow_realdims.WIDE_SEEDS; the disturbance conditions run on this profile (it has the external torque model)

Every number is a placeholder-based SIM result (자리값 기반 SIM 결과), not field performance.

CLI (from sim/ubts): python -B -m steering.safety_scripts.follow_smooth --out <folder outside the repository>
                     [--parts NAME ...] [--workers N (<= 4)] [--only NAME ...]  |  --prereg
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

from . import acceptance as A
from . import follow_matrix as FM
from . import follow_realdims as RD
from . import follow_wide as FW
from . import measure as M

SCHEMA = 'ubts.steer_sim.follow_smooth.v1'
PREREG_SCHEMA = 'ubts.steer_sim.prereg_follow_smooth.v1'
PREREG_ID = 'steer_follow_smooth_r5_v1'
PROFILE_ID = 'front_steer_v2_field_20261116'
WIDE_SUBSET = RD.WIDE_SUBSET
WIDE_SEEDS = RD.WIDE_SEEDS
PARTS = ('matrix', 'wide')


def base_profile():
    from steering.vehicle_profile import PROFILES_DIR, load_profile
    return load_profile(PROFILES_DIR / (PROFILE_ID + '.yaml'))


def matrix_spec(cond, seed):
    """follow_matrix.spec() on this profile, the executor configuration as the rig makes it."""
    from steering.safety_scripts import catalogue as CAT
    base = base_profile()
    profile = RD._profile_with(base, cond.get('profile'), cond.get('wander_mean_deg_per_m'), cond['name'])
    world = {'kind': 'W0', 'markers': FM.posts(cond.get('extra_posts') or ())}
    x, y, yaw = cond.get('start_pose', (0., 0., 0.))
    return CAT.episode('smooth_matrix_%s_s%d' % (cond['name'], seed), controller=RD.controller(base), world=world,
                       faults=[dict(f) for f in cond.get('faults') or ()], rig=dict(profile=profile), config={},
                       duration_s=FM.DURATION_S, seed=int(seed), start_pose=(x, y, yaw), auto_repress=False)


def wide_spec(c, seed):
    """follow_wide.spec() on this profile (every condition, the disturbance ones too)."""
    from steering.safety_scripts import catalogue as CAT
    base = base_profile()
    rig = dict(profile=RD._profile_with(base, c.get('profile'), c.get('wander_mean_deg_per_m'), c['name']))
    if c.get('gnss') is not None:
        rig['gnss'] = json.loads(json.dumps(c['gnss']))
    if c.get('drive_table'):
        rig['drive_table'] = FW.drive_table(c['drive_table'])
    if c.get('w2'):
        world = dict(kind='W2', record=c['w2']['record'], corridor_id=c['w2']['corridor_id'],
                     direction=c['w2'].get('direction', 'fwd'))
        start = (1., 0., 0.)
    else:
        geo = FW.geometry(c.get('world'))
        world = dict(kind='W0', markers=geo.posts())
        start = geo.start_pose(*c.get('start', (0., 0., 0.)))
    return CAT.episode('smooth_wide_%s_s%d' % (c['name'], seed), controller=RD.controller(base, c.get('controller')),
                       world=world, faults=[dict(f) for f in c.get('faults') or ()],
                       actions=[dict(a) for a in c.get('actions') or ()], rig=rig, config=dict(c.get('config') or {}),
                       duration_s=FW.duration(c), seed=int(seed), start_pose=start, auto_repress=False)


def printable(episode, cond):
    out = dict(episode, rig=dict(episode['rig']))
    out['rig']['profile'] = dict(profile_id=PROFILE_ID, overrides=cond.get('profile') or {},
                                 wander_mean_deg_per_m=cond.get('wander_mean_deg_per_m'))
    return out


def items(parts, only=None):
    out = []
    for part in parts:
        if part == 'matrix':
            for cond in FM.CONDITIONS:
                if only and cond['name'] not in only:
                    continue
                out.extend((part, cond['name'], seed) for seed in cond['seeds'])
        elif part == 'wide':
            for name in WIDE_SUBSET:
                if only and name not in only:
                    continue
                out.extend((part, name, seed) for seed in WIDE_SEEDS)
        else:
            raise ValueError('unknown part %r' % (part,))
    return out


def run_one(item):
    part, name, seed, out_root = item
    S = A._sim_in_loop()
    started = time.perf_counter()
    if part == 'matrix':
        cond = RD._matrix_condition(name)
        episode = matrix_spec(cond, seed)
    else:
        cond = FW.condition(name)
        episode = wide_spec(cond, seed)
    result, facade = S.run_episode(episode, None, keep=True)
    try:
        if out_root is not None:
            result = facade.write(os.path.join(out_root, episode['name']), printable(episode, cond))
        arrays = facade.trace_arrays()
        ticks = [r for r in facade.log.records if r.get('k') == 'tick']
        summary = facade.summary()
        art = M.artefacts(facade, printable(episode, cond))
        if part == 'matrix':
            metrics = FM.measure(arrays, ticks, summary, episode['world']['markers'], cond.get('settle_m', 0.))
            metrics['steer_fault_lines'] = FW.steer_fault_lines(art)
        else:
            metrics = FW.measure(cond, arrays, ticks, summary, facade.rig.world, episode['world'].get('markers') or (),
                                 art, facade.rig.gnss_report())
        metrics['after_terminal'] = FM.terminal_window(art, metrics)
        feasibility = summary.get('feasibility') or (summary.get('config') or {}).get('feasibility')
    finally:
        facade.rig.close()
    checks = FM.judge(cond['expect'], metrics) if part == 'matrix' else FW.judge(cond, metrics)
    return dict(part=part, condition=name, seed=seed, name=episode['name'], expect=cond['expect'],
                ok=all(ch['ok'] for ch in checks), failed=[ch['check'] for ch in checks if not ch['ok']],
                checks=checks, metrics=metrics, digests=result['digests'], config_digest=result['config_digest'],
                feasibility=feasibility, wall_s=round(time.perf_counter() - started, 1))


def run_all(out_dir, parts=PARTS, workers=1, only=None):
    started = time.perf_counter()
    todo = [item + (os.path.join(out_dir, 'runs'),) for item in items(parts, only)]
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=min(4, workers)) as pool:
            rows = list(pool.map(run_one, todo))
    else:
        rows = [run_one(item) for item in todo]
    by_part = {}
    for r in rows:
        entry = by_part.setdefault(r['part'], dict(runs=0, passed=0, failed_checks={}))
        entry['runs'] += 1
        entry['passed'] += int(r['ok'])
        for name in r['failed']:
            entry['failed_checks'][name] = entry['failed_checks'].get(name, 0) + 1
    report = dict(schema=SCHEMA, prereg=PREREG_ID, result_kind=A.RESULT_KIND, field_validated=False,
                  profile_id=PROFILE_ID, parts=list(parts), only=only,
                  complete=not only and set(parts) == set(PARTS), runs=len(rows), by_part=by_part, rows=rows,
                  workers=workers, wall_s=round(time.perf_counter() - started, 1))
    os.makedirs(out_dir, exist_ok=True)
    raw = json.dumps(report, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False, default=str) + '\n'
    with open(os.path.join(out_dir, 'FOLLOW_SMOOTH.json'), 'wb') as fh:
        fh.write(raw.encode('utf-8'))
    return report


def prereg():
    """The machine-readable pre-registration (docs/steering/PREREG_STEER_FOLLOW_SMOOTH.json)."""
    return dict(schema=PREREG_SCHEMA, id=PREREG_ID, result_kind='placeholder-based SIM result',
                simulator='F-SIM T1 rig, executor in the loop (tools/row_executor/sim_in_loop_steer.py)',
                profile_id=PROFILE_ID, steer_fw_mode='smooth_r5',
                controller=dict(FM.CONTROLLER, wheelbase_m='profile wheelbase_m (0.657)'),
                executor='sim_in_loop_steer.steer_executor_config with the smooth_r5 measured values '
                         '(steering.smooth_hold.executor_values through steering.abs_hold.executor_values); no '
                         'threshold change (the measured SMOOTH rate passes the steering GO rule)',
                parts=dict(
                    matrix=dict(conditions=[c['name'] for c in FM.CONDITIONS],
                                seeds={c['name']: list(c['seeds']) for c in FM.CONDITIONS},
                                runs=sum(len(c['seeds']) for c in FM.CONDITIONS), criteria=FM.CRITERIA,
                                settle_m=FM.SETTLE_M, rules='follow_matrix.measure / judge (steer_follow_v1)',
                                pass_rule='every run passes its registered checks'),
                    wide=dict(conditions=list(WIDE_SUBSET), seeds=list(WIDE_SEEDS),
                              runs=len(WIDE_SUBSET) * len(WIDE_SEEDS), criteria=FW.CRITERIA,
                              rules='follow_wide.measure / judge (steer_follow_wide_v2, round 2 rules)',
                              disturbance_profile=PROFILE_ID,
                              pass_rule='reported per condition; the part passes when every run passes')),
                order='matrix first; wide when the time allows (reported as not run otherwise)',
                footprint=dict(front_bumper_m=FM.FRONT_BUMPER_M, half_width_m=FM.HALF_WIDTH_M,
                               note='footprint 1.22 x 0.70 m is still the placeholder of the profile'),
                iterations=dict(max=0, allowed='none in this registration: the results are reported as they are'))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--out')
    p.add_argument('--parts', nargs='*', default=list(PARTS))
    p.add_argument('--workers', type=int, default=1)
    p.add_argument('--only', nargs='*')
    p.add_argument('--prereg', action='store_true')
    args = p.parse_args(argv)
    if args.prereg:
        sys.stdout.buffer.write((json.dumps(prereg(), sort_keys=True, indent=1) + '\n').encode('utf-8'))
        return 0
    if not args.out:
        raise SystemExit('--out is required')
    from steering.scene_spec import inside_repository
    if inside_repository(args.out):
        raise SystemExit('refused: %s lies inside the repository (runs live outside)' % args.out)
    report = run_all(args.out, tuple(args.parts), args.workers, args.only)
    for row in report['rows']:
        m = row['metrics']
        print('%-4s %-7s %-18s s%-4d end %s e %s rmse %s p95 %s max %s head %s/%s v %s clear %s %s' % (
            'ok' if row['ok'] else 'FAIL', row['part'], row['condition'], row['seed'],
            m.get('end_reason') or m.get('first_pause'), m.get('row_end_error_m'), m.get('lateral_rmse_m'),
            m.get('lateral_p95_m'), m.get('lateral_max_m'), m.get('heading_rmse_deg'), m.get('heading_max_deg'),
            m.get('speed_max_m_s'), m.get('min_clearance_m'), '' if row['ok'] else 'failed: ' + '; '.join(row['failed'])))
    for part, entry in report['by_part'].items():
        print('%s: %d of %d runs pass' % (part, entry['passed'], entry['runs']))
    print('%.0f s' % report['wall_s'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
