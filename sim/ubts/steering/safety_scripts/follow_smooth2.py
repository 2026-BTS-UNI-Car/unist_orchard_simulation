"""Row following with the SMOOTH r5 hold on the vehicle's monitor stack, and the SIM proposal r6p (sim-smooth-wide,
2026-11-17): pre-registration steer_follow_smooth_r5_v2.

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_SMOOTH_V2_KO.md and PREREG_STEER_FOLLOW_SMOOTH_V2.json (= prereg()
of this module; sim/ubts/test_steering_follow_smooth2.py compares them). Do not change PARTS, profiles, conditions,
seeds, criteria or the executor and controller settings after a run; an amendment is written in the pre-registration
document first. The v1 registration (follow_smooth.py, steer_follow_smooth_r5_v1) and its result stay as they are.

Why a v2 (docs/steering/SMOOTH_R5_DISTURBANCE_KO.md): in v1 the two steering disturbance conditions stopped on V3 MCU
fault monitors (POT_NOISE, TRACK) that the vehicle's board (firmware r5) does not have, and the host's
steering_not_following rule of the SIM was the V3 bridge's (ramp gate, 3 s), not the executor's ESP32 adapter rule
(3 deg / 2 s without a gate); operator_link_lost was judged against 0.55 s, older than decision R60 (operator timeout
1.0 s, SIM_ACCEPTANCE amendment 60: drive 0 <= operator_timeout_s + 1 tick).

Parts (simulator, conditions, seeds and measuring rules of follow_smooth.py; the same 15 + 60 runs twice):
  matrix            follow_matrix's 15 runs on front_steer_v2_1_field_20261117 (the vehicle: SMOOTH r5, steer_monitor_stack
                    esp32_r5)
  wide              follow_realdims.WIDE_SUBSET x WIDE_SEEDS (60 runs) on the same profile
  proposal_matrix   the 15 matrix runs on front_steer_v2_2_proposal_20261117 (SIM proposal r6p, not on the vehicle)
  proposal_wide     the 60 wide runs on the proposal profile
Criteria: follow_matrix.CRITERIA and follow_wide.CRITERIA unchanged, except the wide operator_link_lost check 'drive
command 0 after the last heartbeat': <= LINK_DRIVE_ZERO_S = operator_timeout_s 1.0 + one executor tick 0.05 s (R60,
amendment 60) instead of 0.55 s; the 0.55 s outcome stays in the row as a report (v1_link_check).

Every number is a placeholder-based SIM result (자리값 기반 SIM 결과), not field performance; the proposal parts show the
effect of a SIM proposal, not of the vehicle's firmware.

CLI (from sim/ubts): python -B -m steering.safety_scripts.follow_smooth2 --out <folder outside the repository>
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

SCHEMA = 'ubts.steer_sim.follow_smooth2.v1'
PREREG_SCHEMA = 'ubts.steer_sim.prereg_follow_smooth2.v1'
PREREG_ID = 'steer_follow_smooth_r5_v2'
V1_PREREG_ID = 'steer_follow_smooth_r5_v1'
PROFILE_ID = 'front_steer_v2_1_field_20261117'
PROPOSAL_PROFILE_ID = 'front_steer_v2_2_proposal_20261117'
PROFILE_OF_PART = dict(matrix=PROFILE_ID, wide=PROFILE_ID, proposal_matrix=PROPOSAL_PROFILE_ID,
                       proposal_wide=PROPOSAL_PROFILE_ID)
PARTS = ('matrix', 'wide', 'proposal_matrix', 'proposal_wide')
WIDE_SUBSET = RD.WIDE_SUBSET
WIDE_SEEDS = RD.WIDE_SEEDS
OPERATOR_TIMEOUT_S = 1.0        # executor operator_timeout_s default (decision R60, OPERATOR_API_KO.md 3.1)
EXECUTOR_TICK_S = .05           # executor control_hz 20
LINK_DRIVE_ZERO_S = OPERATOR_TIMEOUT_S + EXECUTOR_TICK_S   # SIM_ACCEPTANCE amendment 60
LINK_CHECK = 'drive command 0 after the last heartbeat'
WIDE_CRITERIA = dict(FW.CRITERIA, link_drive_zero_s=LINK_DRIVE_ZERO_S)


def base_profile(profile_id):
    from steering.vehicle_profile import PROFILES_DIR, load_profile
    return load_profile(PROFILES_DIR / (profile_id + '.yaml'))


def matrix_spec(cond, seed, profile_id, prefix):
    """follow_matrix.spec() on the profile, the executor configuration as the rig makes it (as follow_smooth)."""
    from steering.safety_scripts import catalogue as CAT
    base = base_profile(profile_id)
    profile = RD._profile_with(base, cond.get('profile'), cond.get('wander_mean_deg_per_m'), cond['name'])
    world = {'kind': 'W0', 'markers': FM.posts(cond.get('extra_posts') or ())}
    x, y, yaw = cond.get('start_pose', (0., 0., 0.))
    return CAT.episode('%s_matrix_%s_s%d' % (prefix, cond['name'], seed), controller=RD.controller(base), world=world,
                       faults=[dict(f) for f in cond.get('faults') or ()], rig=dict(profile=profile), config={},
                       duration_s=FM.DURATION_S, seed=int(seed), start_pose=(x, y, yaw), auto_repress=False)


def wide_spec(c, seed, profile_id, prefix):
    """follow_wide.spec() on the profile (every condition, the disturbance ones too; as follow_smooth)."""
    from steering.safety_scripts import catalogue as CAT
    base = base_profile(profile_id)
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
    return CAT.episode('%s_wide_%s_s%d' % (prefix, c['name'], seed), controller=RD.controller(base, c.get('controller')),
                       world=world, faults=[dict(f) for f in c.get('faults') or ()],
                       actions=[dict(a) for a in c.get('actions') or ()], rig=rig, config=dict(c.get('config') or {}),
                       duration_s=FW.duration(c), seed=int(seed), start_pose=start, auto_repress=False)


def printable(episode, cond, profile_id):
    out = dict(episode, rig=dict(episode['rig']))
    out['rig']['profile'] = dict(profile_id=profile_id, overrides=cond.get('profile') or {},
                                 wander_mean_deg_per_m=cond.get('wander_mean_deg_per_m'))
    return out


def judge_wide(cond, metrics):
    """follow_wide.judge with the v2 operator link limit (LINK_DRIVE_ZERO_S). Returns (checks, v1 link check or None)."""
    checks = FW.judge(cond, metrics)
    v1_link = None
    for i, ch in enumerate(checks):
        if ch['check'] == LINK_CHECK:
            v1_link = dict(ch)
            value = metrics.get('link_drive_zero_s')
            checks[i] = dict(check=LINK_CHECK, ok=value is not None and value <= LINK_DRIVE_ZERO_S + 1e-9,
                             measured=value, limit='<= %g s (operator_timeout_s %g + one executor tick %g s, R60)' % (
                                 LINK_DRIVE_ZERO_S, OPERATOR_TIMEOUT_S, EXECUTOR_TICK_S))
    return checks, v1_link


def items(parts, only=None):
    out = []
    for part in parts:
        if part in ('matrix', 'proposal_matrix'):
            for cond in FM.CONDITIONS:
                if only and cond['name'] not in only:
                    continue
                out.extend((part, cond['name'], seed) for seed in cond['seeds'])
        elif part in ('wide', 'proposal_wide'):
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
    profile_id = PROFILE_OF_PART[part]
    prefix = 'smooth2p' if part.startswith('proposal_') else 'smooth2'
    matrix = part.endswith('matrix')
    if matrix:
        cond = RD._matrix_condition(name)
        episode = matrix_spec(cond, seed, profile_id, prefix)
    else:
        cond = FW.condition(name)
        episode = wide_spec(cond, seed, profile_id, prefix)
    result, facade = S.run_episode(episode, None, keep=True)
    try:
        if out_root is not None:
            result = facade.write(os.path.join(out_root, episode['name']), printable(episode, cond, profile_id))
        arrays = facade.trace_arrays()
        ticks = [r for r in facade.log.records if r.get('k') == 'tick']
        summary = facade.summary()
        art = M.artefacts(facade, printable(episode, cond, profile_id))
        if matrix:
            metrics = FM.measure(arrays, ticks, summary, episode['world']['markers'], cond.get('settle_m', 0.))
            metrics['steer_fault_lines'] = FW.steer_fault_lines(art)
        else:
            metrics = FW.measure(cond, arrays, ticks, summary, facade.rig.world, episode['world'].get('markers') or (),
                                 art, facade.rig.gnss_report())
        metrics['after_terminal'] = FM.terminal_window(art, metrics)
        feasibility = summary.get('feasibility') or (summary.get('config') or {}).get('feasibility')
    finally:
        facade.rig.close()
    v1_link = None
    if matrix:
        checks = FM.judge(cond['expect'], metrics)
    else:
        checks, v1_link = judge_wide(cond, metrics)
    return dict(part=part, profile_id=profile_id, condition=name, seed=seed, name=episode['name'],
                expect=cond['expect'], ok=all(ch['ok'] for ch in checks),
                failed=[ch['check'] for ch in checks if not ch['ok']], checks=checks, v1_link_check=v1_link,
                metrics=metrics, digests=result['digests'], config_digest=result['config_digest'],
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
        entry = by_part.setdefault(r['part'], dict(runs=0, passed=0, failed_checks={}, profile_id=r['profile_id']))
        entry['runs'] += 1
        entry['passed'] += int(r['ok'])
        for name in r['failed']:
            entry['failed_checks'][name] = entry['failed_checks'].get(name, 0) + 1
    report = dict(schema=SCHEMA, prereg=PREREG_ID, result_kind=A.RESULT_KIND, field_validated=False,
                  profiles=dict(PROFILE_OF_PART), parts=list(parts), only=only,
                  complete=not only and set(parts) == set(PARTS), runs=len(rows), by_part=by_part, rows=rows,
                  workers=workers, wall_s=round(time.perf_counter() - started, 1))
    os.makedirs(out_dir, exist_ok=True)
    raw = json.dumps(report, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False, default=str) + '\n'
    with open(os.path.join(out_dir, 'FOLLOW_SMOOTH2.json'), 'wb') as fh:
        fh.write(raw.encode('utf-8'))
    return report


def prereg():
    """The machine-readable pre-registration (docs/steering/PREREG_STEER_FOLLOW_SMOOTH_V2.json)."""
    matrix = dict(conditions=[c['name'] for c in FM.CONDITIONS], seeds={c['name']: list(c['seeds']) for c in FM.CONDITIONS},
                  runs=sum(len(c['seeds']) for c in FM.CONDITIONS), criteria=FM.CRITERIA, settle_m=FM.SETTLE_M,
                  rules='follow_matrix.measure / judge (steer_follow_v1)', pass_rule='every run passes its registered checks')
    wide = dict(conditions=list(WIDE_SUBSET), seeds=list(WIDE_SEEDS), runs=len(WIDE_SUBSET) * len(WIDE_SEEDS),
                criteria=WIDE_CRITERIA,
                rules='follow_wide.measure / judge (steer_follow_wide_v2, round 2 rules) with the operator link limit '
                      'replaced (judge_wide)',
                pass_rule='reported per condition; the part passes when every run passes')
    parts = {}
    for part in PARTS:
        base = matrix if part.endswith('matrix') else wide
        parts[part] = dict(base, profile_id=PROFILE_OF_PART[part])
    return dict(schema=PREREG_SCHEMA, id=PREREG_ID, follows=V1_PREREG_ID, result_kind='placeholder-based SIM result',
                simulator='F-SIM T1 rig, executor in the loop (tools/row_executor/sim_in_loop_steer.py)',
                profiles=dict(vehicle=dict(profile_id=PROFILE_ID, steer_fw_mode='smooth_r5',
                                           steer_monitor_stack='esp32_r5'),
                              proposal=dict(profile_id=PROPOSAL_PROFILE_ID, steer_fw_mode='smooth_r6p',
                                            steer_monitor_stack='esp32_r5',
                                            note='SIM proposal r6p, not the vehicle firmware; nothing flashed')),
                controller=dict(FM.CONTROLLER, wheelbase_m='profile wheelbase_m (0.657)'),
                executor='sim_in_loop_steer.steer_executor_config with the SMOOTH measured values '
                         '(steering.smooth_hold.executor_values); no threshold change',
                parts=parts,
                changed_from_v1=dict(
                    profiles='front_steer_v2_field_20261116 -> front_steer_v2_1_field_20261117 (vehicle monitor stack) '
                             'and front_steer_v2_2_proposal_20261117 (SIM proposal parts)',
                    link_drive_zero_s=dict(v1=FW.CRITERIA['link_drive_zero_s'], v2=LINK_DRIVE_ZERO_S,
                                           reason='decision R60 (2026-11-13) and SIM_ACCEPTANCE amendment 60: drive 0 '
                                                  '<= operator_timeout_s (1.0) + 1 tick (0.05 s); the 0.55 s of '
                                                  'steer_follow_wide_v2 predates it')),
                order='matrix, wide, proposal_matrix, proposal_wide in one locked session',
                footprint=dict(front_bumper_m=FM.FRONT_BUMPER_M, half_width_m=FM.HALF_WIDTH_M,
                               note='footprint 1.22 x 0.70 m is still the placeholder of the profile'),
                iterations=dict(max=0, allowed='none in this registration: the results are reported as they are'),
                v1='steer_follow_smooth_r5_v1 and its result (matrix 15/15, wide 52/60) stay as they are')


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
        print('%-4s %-15s %-18s s%-4d end %s e %s rmse %s p95 %s max %s head %s/%s v %s clear %s %s' % (
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
