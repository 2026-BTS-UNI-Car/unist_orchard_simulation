"""Row-following re-check with the real dimensions and the real steering firmware (sim-realdims, 2026-11-16).

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_REALDIMS_KO.md and PREREG_STEER_FOLLOW_REALDIMS.json (= prereg() of
this module; sim/ubts/test_steering_follow_realdims.py compares them). Do not change PARTS, conditions, seeds, criteria
or the executor and controller settings after a run; an amendment is written in the pre-registration document first.

Profile: front_steer_v1_spec_20261116 (schema v4; measured wheelbase, track, wheel size, steering limit 20 deg, LiDAR
mount; steer_fw_mode abs_hold_8: the SIM ESP emulator runs the pulse hold of the real board firmware
ABS-HOLD-8-REBOUND-LIMITS, docs/steering/ABS_HOLD8_SIM_KO.md). The executor takes the board's measured step response as
its 'measured' steering rate (steering.abs_hold.executor_values). The controller is row_follow with the profile
wheelbase (wheelbase_m, a controller setting of the spec; the default 0.76 m is the old placeholder).

Parts (simulator and measuring rules of follow_matrix.py and follow_wide.py; same criteria):
  gate_as_is     follow_matrix nominal seed 0 with the executor configuration as it is. Registered expectation: the
                 executor's steering GO rule (measured steering rate >= steer_rate_min_deg_s 8 deg/s, a placeholder;
                 config.validate) refuses the configuration with the board's rate (about 5.1 deg/s). Recorded, not a
                 pass / fail of following.
  matrix         the 15 runs of follow_matrix (steer_follow_v1 conditions) with the GO threshold
                 steer_rate_min_deg_s = GATE_RATE_MIN_DEG_S (documented change, EXEC_GATE)
  wide           WIDE_SUBSET of follow_wide (steer_follow_wide_v2 conditions, round 2 measuring rules) x WIDE_SEEDS with
                 the same change; the disturbance conditions run on this profile (it has the external torque model)
  geometry_only  comparison only (not judged): the matrix runs with steer_fw_mode v3_servo (new dimensions, the
                 firmware V3 servo at the fitted motor speed)

Every number is a placeholder-based SIM result (자리값 기반 SIM 결과), not field performance.

CLI (from sim/ubts): python -B -m steering.safety_scripts.follow_realdims --out <folder outside the repository>
                     [--parts NAME ...] [--workers N (<= 4)] [--only NAME ...] [--tuning NAME]  |  --prereg
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

from . import acceptance as A
from . import follow_matrix as FM
from . import follow_wide as FW
from . import measure as M

SCHEMA = 'ubts.steer_sim.follow_realdims.v1'
PREREG_SCHEMA = 'ubts.steer_sim.prereg_follow_realdims.v1'
PREREG_ID = 'steer_follow_realdims_v1'
PROFILE_ID = 'front_steer_v1_spec_20261116'
GATE_RATE_MIN_DEG_S = 4.0
EXEC_GATE = dict(steer_rate_min_deg_s=GATE_RATE_MIN_DEG_S)
WIDE_SUBSET = ('curve_l30', 'curve_r30', 'gap3_both', 'endpost_2p0', 'width_3p0', 'w2_042301_c03', 'w2_040927_c06',
               'heading_p10', 'heading_n10', 'pull_left', 'pull_right', 'speed_0p2', 'speed_0p5', 'table_x070',
               'table_x130', 'dist_step_p2p5', 'dist_terrain', 'imu_stale_1p0', 'gnss_nominal', 'operator_link_lost')
WIDE_SEEDS = (4000, 4001, 4002)
PARTS = ('gate_as_is', 'matrix', 'wide', 'geometry_only')
JUDGED_PARTS = ('matrix', 'wide')
# Field-tuning sets of later documented iterations (pre-registration amendments); None = the defaults.
TUNINGS = {}


def base_profile(fw_mode=None):
    from steering.vehicle_profile import PROFILES_DIR, load_profile
    profile = load_profile(PROFILES_DIR / (PROFILE_ID + '.yaml'))
    if fw_mode is not None:
        profile = profile.with_overrides({'steer_fw_mode': fw_mode}, label='fw_' + fw_mode)
    return profile


def controller(profile, extra=None):
    return dict(FM.CONTROLLER, wheelbase_m=float(profile.value('wheelbase_m')), **(extra or {}))


def _profile_with(base, overrides, wander, label):
    overrides = dict(overrides or {})
    if wander is not None:
        value = dict(base.value('curvature_wander'))
        value['mean_deg_per_m'] = float(wander)
        overrides['curvature_wander'] = value
    return base.with_overrides(overrides, label=label) if overrides else base


def _config(part, extra=None, tuning=None):
    config = dict(extra or {})
    if part != 'gate_as_is':
        config.update(EXEC_GATE)
    if tuning:
        config.update(tuning.get('config') or {})
    return config


def matrix_spec(cond, seed, part, tuning=None):
    """follow_matrix.spec() on this profile (part geometry_only: steer_fw_mode v3_servo)."""
    from steering.safety_scripts import catalogue as CAT
    base = base_profile('v3_servo' if part == 'geometry_only' else None)
    profile = _profile_with(base, cond.get('profile'), cond.get('wander_mean_deg_per_m'), cond['name'])
    world = {'kind': 'W0', 'markers': FM.posts(cond.get('extra_posts') or ())}
    x, y, yaw = cond.get('start_pose', (0., 0., 0.))
    ctrl = controller(base, (tuning or {}).get('controller'))
    return CAT.episode('realdims_%s_%s_s%d' % (part, cond['name'], seed), controller=ctrl, world=world,
                       faults=[dict(f) for f in cond.get('faults') or ()], rig=dict(profile=profile),
                       config=_config(part, tuning=tuning), duration_s=FM.DURATION_S, seed=int(seed),
                       start_pose=(x, y, yaw), auto_repress=False)


def wide_spec(c, seed, part, tuning=None):
    """follow_wide.spec() on this profile (every condition, the disturbance ones too)."""
    from steering.safety_scripts import catalogue as CAT
    base = base_profile()
    rig = dict(profile=_profile_with(base, c.get('profile'), c.get('wander_mean_deg_per_m'), c['name']))
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
    extra = dict(c.get('controller') or {})
    extra.update((tuning or {}).get('controller') or {})
    return CAT.episode('realdims_%s_%s_s%d' % (part, c['name'], seed), controller=controller(base, extra), world=world,
                       faults=[dict(f) for f in c.get('faults') or ()], actions=[dict(a) for a in c.get('actions') or ()],
                       rig=rig, config=_config(part, c.get('config'), tuning), duration_s=FW.duration(c),
                       seed=int(seed), start_pose=start, auto_repress=False)


def printable(episode, cond, part):
    out = dict(episode, rig=dict(episode['rig']))
    out['rig']['profile'] = dict(profile_id=PROFILE_ID, fw_mode='v3_servo' if part == 'geometry_only' else None,
                                 overrides=cond.get('profile') or {},
                                 wander_mean_deg_per_m=cond.get('wander_mean_deg_per_m'))
    return out


def items(parts, only=None, tuning_name=None):
    out = []
    for part in parts:
        if part in ('gate_as_is', 'matrix', 'geometry_only'):
            for cond in FM.CONDITIONS:
                seeds = (0,) if part == 'gate_as_is' else cond['seeds']
                if part == 'gate_as_is' and cond['name'] != 'nominal':
                    continue
                if only and cond['name'] not in only:
                    continue
                out.extend((part, 'matrix', cond['name'], seed, tuning_name) for seed in seeds)
        elif part == 'wide':
            for name in WIDE_SUBSET:
                if only and name not in only:
                    continue
                out.extend((part, 'wide', name, seed, tuning_name) for seed in WIDE_SEEDS)
        else:
            raise ValueError('unknown part %r' % (part,))
    return out


def _matrix_condition(name):
    for cond in FM.CONDITIONS:
        if cond['name'] == name:
            return cond
    raise KeyError(name)


def run_one(item):
    part, family, name, seed, tuning_name, out_root = item
    tuning = TUNINGS.get(tuning_name) if tuning_name else None
    S = A._sim_in_loop()
    started = time.perf_counter()
    if family == 'matrix':
        cond = _matrix_condition(name)
        episode = matrix_spec(cond, seed, part, tuning)
    else:
        cond = FW.condition(name)
        episode = wide_spec(cond, seed, part, tuning)
    try:
        result, facade = S.run_episode(episode, None, keep=True)
    except Exception as exc:      # the executor configuration itself is refused (gate_as_is: the steering GO rule)
        if type(exc).__name__ != 'ConfigError':
            raise
        return dict(part=part, family=family, condition=name, seed=seed, name=episode['name'], expect=cond['expect'],
                    ok=False, refused=str(exc), failed=['executor configuration refused'], checks=[], metrics={},
                    digests=None, config_digest=None, feasibility=None, tuning=tuning_name,
                    wall_s=round(time.perf_counter() - started, 1))
    try:
        if out_root is not None:
            result = facade.write(os.path.join(out_root, episode['name']), printable(episode, cond, part))
        arrays = facade.trace_arrays()
        ticks = [r for r in facade.log.records if r.get('k') == 'tick']
        summary = facade.summary()
        art = M.artefacts(facade, printable(episode, cond, part))
        if family == 'matrix':
            metrics = FM.measure(arrays, ticks, summary, episode['world']['markers'], cond.get('settle_m', 0.))
            metrics['steer_fault_lines'] = FW.steer_fault_lines(art)
        else:
            metrics = FW.measure(cond, arrays, ticks, summary, facade.rig.world, episode['world'].get('markers') or (),
                                 art, facade.rig.gnss_report())
        metrics['after_terminal'] = FM.terminal_window(art, metrics)
        feasibility = summary.get('feasibility') or (summary.get('config') or {}).get('feasibility')
    finally:
        facade.rig.close()
    checks = FM.judge(cond['expect'], metrics) if family == 'matrix' else FW.judge(cond, metrics)
    return dict(part=part, family=family, condition=name, seed=seed, name=episode['name'], expect=cond['expect'],
                ok=all(ch['ok'] for ch in checks), failed=[ch['check'] for ch in checks if not ch['ok']],
                checks=checks, metrics=metrics, digests=result['digests'], config_digest=result['config_digest'],
                feasibility=feasibility, tuning=tuning_name, wall_s=round(time.perf_counter() - started, 1))


def run_all(out_dir, parts=PARTS, workers=1, only=None, tuning_name=None):
    started = time.perf_counter()
    todo = [item + (os.path.join(out_dir, 'runs'),) for item in items(parts, only, tuning_name)]
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
                  profile_id=PROFILE_ID, parts=list(parts), only=only, tuning=tuning_name,
                  complete=not only and set(parts) == set(PARTS), runs=len(rows), by_part=by_part, rows=rows,
                  workers=workers, wall_s=round(time.perf_counter() - started, 1))
    os.makedirs(out_dir, exist_ok=True)
    raw = json.dumps(report, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False, default=str) + '\n'
    with open(os.path.join(out_dir, 'FOLLOW_REALDIMS.json'), 'wb') as fh:
        fh.write(raw.encode('utf-8'))
    return report


def prereg():
    """The machine-readable pre-registration (docs/steering/PREREG_STEER_FOLLOW_REALDIMS.json)."""
    return dict(schema=PREREG_SCHEMA, id=PREREG_ID, result_kind='placeholder-based SIM result',
                simulator='F-SIM T1 rig, executor in the loop (tools/row_executor/sim_in_loop_steer.py)',
                profile_id=PROFILE_ID, steer_fw_mode='abs_hold_8',
                controller=dict(FM.CONTROLLER, wheelbase_m='profile wheelbase_m (0.657)'),
                executor='sim_in_loop_steer.steer_executor_config with the abs_hold_8 measured values '
                         '(steering.abs_hold.executor_values)',
                gate_change=dict(EXEC_GATE, reason='steering GO rule threshold 8 deg/s is a placeholder (design 6.6, '
                                                   'P8); the board measures about 5.1 deg/s'),
                parts=dict(
                    gate_as_is=dict(runs='follow_matrix nominal seed 0', config='as is',
                                    expectation='the executor configuration is refused by the steering GO rule '
                                                '(config.validate: measured rate below 8 deg/s; recorded, not '
                                                'judged)'),
                    matrix=dict(conditions=[c['name'] for c in FM.CONDITIONS],
                                seeds={c['name']: list(c['seeds']) for c in FM.CONDITIONS},
                                runs=sum(len(c['seeds']) for c in FM.CONDITIONS), criteria=FM.CRITERIA,
                                settle_m=FM.SETTLE_M, rules='follow_matrix.measure / judge (steer_follow_v1)',
                                pass_rule='every run passes its registered checks'),
                    wide=dict(conditions=list(WIDE_SUBSET), seeds=list(WIDE_SEEDS),
                              runs=len(WIDE_SUBSET) * len(WIDE_SEEDS), criteria=FW.CRITERIA,
                              rules='follow_wide.measure / judge (steer_follow_wide_v2, round 2 rules)',
                              disturbance_profile=PROFILE_ID,
                              pass_rule='reported per condition; the part passes when every run passes'),
                    geometry_only=dict(runs='the matrix runs with steer_fw_mode v3_servo', judged=False)),
                footprint=dict(front_bumper_m=FM.FRONT_BUMPER_M, half_width_m=FM.HALF_WIDTH_M,
                               note='footprint 1.22 x 0.70 m is still the placeholder of the profile'),
                iterations=dict(max=2, allowed='field tuning values or a documented controller change with tests; '
                                               'every safety limit kept; before / after recorded'))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--out')
    p.add_argument('--parts', nargs='*', default=list(PARTS))
    p.add_argument('--workers', type=int, default=1)
    p.add_argument('--only', nargs='*')
    p.add_argument('--tuning')
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
    if args.tuning and args.tuning not in TUNINGS:
        raise SystemExit('unknown tuning %r (known: %s)' % (args.tuning, ', '.join(sorted(TUNINGS)) or 'none'))
    report = run_all(args.out, tuple(args.parts), args.workers, args.only, args.tuning)
    for row in report['rows']:
        m = row['metrics']
        print('%-4s %-13s %-18s s%-4d end %s e %s rmse %s p95 %s max %s head %s/%s v %s clear %s %s' % (
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
