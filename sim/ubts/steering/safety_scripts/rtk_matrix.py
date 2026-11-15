"""RTK weak constraint (gnss_mode weak_constraint) on the fast SIM T1 rig: harm and benefit against the same run with the
GNSS layer off (rtk-design, 2026-11-16).

Pre-registration: docs/steering/PREREG_GNSS_RTK_KO.md and PREREG_GNSS_RTK.json (= prereg() of this module;
sim/ubts/test_steering_rtk_matrix.py compares them). Do not change WORLDS, CONDITIONS, seeds, criteria or the executor
settings after a judged run; an amendment is written in the pre-registration document first.

Simulator and measuring rules of follow_wide.py (round 2 rules: the row frame of the RowGeometry, footprint, row end e
against the last structure, the same row-end criteria), executor in the loop (tools/row_executor/sim_in_loop_steer.py).
Every rtk run has an 'off' twin: the same world, profile, seed and random stream key (so the same plant, LiDAR and IMU
noise; the virtual GNSS has its own stream), executor gnss_mode off and no GNSS packets.

Profiles: v01 = front_steer_v0_1_placeholder (the reference v0.1 profile, firmware V3 servo, controller row_follow as
registered in steer_follow_v1); v1 = front_steer_v1_spec_20261116 (measured dimensions, firmware ABS-HOLD-8 model) with
the executor and controller settings of steer_follow_realdims_v1 (steering GO threshold 4.0 deg/s, the profile
wheelbase).

Every number is a placeholder-based SIM result (자리값 기반 SIM 결과): the virtual RTK receiver is a placeholder model,
the real receiver under canopy was never measured.

CLI (from sim/ubts): python -B -m steering.safety_scripts.rtk_matrix --out <folder outside the repository>
                     --seeds dev|judge [--profiles v01 v1] [--only NAME ...] [--workers N (<= 4)]   |   --prereg
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time

import numpy as np

from . import acceptance as A
from . import follow_matrix as FM
from . import follow_realdims as FR
from . import follow_wide as FW
from . import measure as M

SCHEMA = 'ubts.steer_sim.rtk_matrix.v1'
PREREG_SCHEMA = 'ubts.steer_sim.prereg_rtk_matrix.v1'
PREREG_ID = 'gnss_rtk_weak_v1'
DEV_SEEDS = (100, 101)
JUDGE_SEEDS = (6000, 6001, 6002, 6003, 6004)
SEED_SETS = dict(dev=DEV_SEEDS, judge=JUDGE_SEEDS)
PROFILES = dict(
    v01=dict(profile_id='front_steer_v0_1_placeholder', config={}, profile_wheelbase=False),
    v1=dict(profile_id=FR.PROFILE_ID, config=dict(FR.EXEC_GATE), profile_wheelbase=True))
DURATION_S = FW.ROW_DURATION_S
HALF = FM.ROW_HALF_WIDTH_M
ROW_END_M = 40.0                             # the last tree line (follow_wide default 24.0: a longer row, so that a
                                             # line can be learnt before the jog)
JOG_S = (21.0, 22.5, 24.0, 25.5)             # right-row trees moved outward (a tree every 1.5 m from -4.5 m)
JOG_OUT_M = .35
JOG_WINDOW_S = (21.0, 28.5)                  # the stretch of the jog metric (row s; the last moved tree + 3 m)
# Worlds (follow_wide RowGeometry changes, rows to ROW_END_M). jog_r: four right-row trees 0.35 m further out (planting
# irregularity or a wide canopy on one side): the LiDAR corridor centre is pulled toward them, the planting line (the
# judged centre line) stays straight.
WORLDS = dict(
    nominal=dict(end_s_m=ROW_END_M),
    jog_r=dict(end_s_m=ROW_END_M, gaps=[['R', JOG_S[0], len(JOG_S)]],
               extra=[[s, -(HALF + JOG_OUT_M), FM.POST_RADIUS_M] for s in JOG_S]),
    curve_l60=dict(end_s_m=ROW_END_M, curvature_1_m=1. / 60.),
)
# The virtual RTK receiver (steering/gnss_sim.py RTK model; placeholder values): 5 Hz, 0.1 s latency, FIXED unless a
# condition says otherwise. Windows are SIM seconds (dev runs: the drive starts about 3.5 s after the start at about
# 0.4 m/s, so the jog stretch s 21-28.5 m is passed at about t 56-77 s).
RTK = dict(rate_hz=5., phase_ms=30, latency_s=.1)
EXECUTOR_RTK = dict(gnss_mode='weak_constraint')      # every other gnss_rtk_* key at its default (perception_settings)


def _rtk(**changes):
    rtk = dict(default_type=changes.pop('default_type', 'fixed'))
    if 'windows' in changes:
        rtk['windows'] = changes.pop('windows')
    out = dict(RTK, rtk=rtk)
    out.update(changes)
    return out


# name, world, gnss spec of the rig, role. Roles: benefit (the constraint must help), harmless (no harm), identical
# (the commands must equal the off twin: the constraint never engages).
CONDITIONS = (
    dict(name='nominal_fixed', world='nominal', gnss=_rtk(), role='harmless'),
    dict(name='jog_fixed', world='jog_r', gnss=_rtk(), role='benefit'),
    dict(name='jog_float_only', world='jog_r', gnss=_rtk(default_type='float'), role='identical'),
    dict(name='jog_fixed_transitions', world='jog_r', role='harmless',
         gnss=_rtk(windows=[[60., 66., 'float'], [66., 70., 'none'], [76., 79., 'single']])),
    dict(name='jog_fixed_jump', world='jog_r', role='harmless',
         gnss=_rtk(jumps=[dict(at_s=59., east_m=.35, north_m=-.35, duration_s=6.),
                          dict(at_s=70., east_m=-.3, north_m=.2, duration_s=.2),
                          dict(at_s=75., east_m=.25, north_m=.3, duration_s=.2)])),
    dict(name='nominal_mirror', world='nominal', gnss=_rtk(frame_mirror=True), role='identical'),
    dict(name='jog_lever', world='jog_r', gnss=_rtk(antenna_xy_m=[.5, .2]), role='benefit'),
    dict(name='curve_fixed', world='curve_l60', gnss=_rtk(), role='harmless'),
)
ROLES = ('benefit', 'harmless', 'identical')
# Registered criteria (pre-registration document 4). Do not change after a judged run.
CRITERIA = dict(
    follow='follow_wide.judge, expect row_end (round 2 criteria: lateral RMSE 0.20 m, p95 0.35 m, max 0.50 m, heading '
           'RMSE 5 deg, max 15 deg, speed 0.50 m/s, e +-1.0 m, no contact, at rest, A22), for every off and rtk run',
    same_end=True,                       # the rtk run ends with the same reason as its off twin
    cap_m=.10, cap_tolerance_m=1e-4,     # |applied correction| <= gnss_rtk_cap_m at every tick (log rounding 1e-4)
    rate_m_s=.05, rate_tolerance_m=2e-4,  # |correction step| <= gnss_rtk_rate_m_s x tick spacing (rounding)
    # per (profile, condition) over its seeds (single runs are reported: a steering firmware model with pulses turns a
    # 1 cm input change into a different trajectory, dev runs: one v1 run +0.053 m lateral max at a 0.013 m correction)
    harm_rmse_m=.02,                     # harmless / benefit: mean lateral RMSE <= mean off + this
    harm_max_m=.05,                      # ... and mean lateral max <= mean off + this
    benefit_jog_reduction=.10,           # benefit: jog-stretch mean |d|, mean over the seeds, reduced by >= 10 % ...
    benefit_seed_majority=True,          # ... and not larger than off in more than half of the seeds
)


def condition(name):
    for c in CONDITIONS:
        if c['name'] == name:
            return c
    raise KeyError(name)


def _wide_condition(world_name):
    """The follow_wide condition dict the follow_wide measure / judge need (expect row_end, the world changes)."""
    return dict(name=world_name, family='rtk', expect='row_end', world=dict(WORLDS[world_name]))


def _profile(profile_key):
    from steering.vehicle_profile import PROFILES_DIR, load_profile
    return load_profile(PROFILES_DIR / (PROFILES[profile_key]['profile_id'] + '.yaml'))


def spec(profile_key, world_name, seed, gnss=None, label='off'):
    """Episode of one run: label 'off' (no GNSS, gnss_mode off) or a condition name (rig GNSS + weak_constraint)."""
    from steering.safety_scripts import catalogue as CAT
    p = PROFILES[profile_key]
    profile = _profile(profile_key)
    geo = FW.geometry(WORLDS[world_name])
    controller = (FR.controller(profile) if p['profile_wheelbase'] else dict(FM.CONTROLLER))
    rig = dict(profile=profile)
    config = dict(p['config'])
    if gnss is not None:
        rig['gnss'] = json.loads(json.dumps(gnss))
        config.update(EXECUTOR_RTK)
    name = label if gnss is not None else 'off_' + world_name
    return CAT.episode('rtk_%s_%s_s%d' % (profile_key, name, seed), controller=controller,
                       world=dict(kind='W0', markers=geo.posts()), rig=rig, config=config, duration_s=DURATION_S,
                       seed=int(seed), start_pose=geo.start_pose(0., 0., 0.), auto_repress=False,
                       stream_key='steer_exec_rtk_%s_%s_s%d' % (profile_key, world_name, seed))


def printable(episode, profile_key):
    out = dict(episode, rig=dict(episode['rig']))
    out['rig']['profile'] = dict(profile_id=PROFILES[profile_key]['profile_id'])
    return out


def jog_metrics(arrays, world_name, window=JOG_WINDOW_S):
    """|d| of the footprint centre over the jog stretch (row s inside window, while driving): mean and max (truth:
    evaluation only)."""
    geo = FW.geometry(WORLDS[world_name])
    s, d, _ = geo.frame(arrays['truth_x_m'], arrays['truth_y_m'], arrays['truth_yaw_rad'])
    throttle = np.asarray(arrays['ctrl_throttle_pct'], dtype=float)
    driving = np.flatnonzero(throttle > 0)
    if not len(driving):
        return None
    span = np.arange(int(driving[0]), int(driving[-1]) + 1)
    inside = span[(s[span] >= window[0]) & (s[span] < window[1])]
    if not len(inside):
        return None
    return dict(mean_abs_m=round(float(np.mean(np.abs(d[inside]))), 4), max_abs_m=round(float(np.max(np.abs(d[inside]))), 4),
                mean_m=round(float(np.mean(d[inside])), 4), samples=int(len(inside)))


def rtk_metrics(ticks):
    """The RTK block of the executor ticks: applied correction bounds and steps, states, last counts."""
    rows = [(r['t'], r['gnss']['rtk']) for r in ticks if isinstance((r.get('gnss') or {}).get('rtk'), dict)]
    if not rows:
        return None
    u = [(t, b.get('correction_m') or 0.) for t, b in rows]
    steps = [(abs(b[1] - a[1]), b[0] - a[0]) for a, b in zip(u, u[1:])]
    states = {}
    for _, b in rows:
        states[b.get('state')] = states.get(b.get('state'), 0) + 1
    last = rows[-1][1]
    return dict(ticks=len(rows), max_abs_correction_m=round(max(abs(v) for _, v in u), 4),
                max_step_m=round(max((s for s, _ in steps), default=0.), 5),
                max_step_over_rate_limit_m=round(max((s - CRITERIA['rate_m_s'] * dt for s, dt in steps), default=0.), 5),
                state_ticks=states, frame=last.get('frame'), counts=last.get('counts'), line=last.get('line'),
                slope=last.get('slope'), layer_max_step_m=last.get('max_step_m'),
                layer_max_abs_correction_m=last.get('max_abs_correction_m'))


def commands(ticks):
    return [(r['t'], r.get('T'), r.get('S')) for r in ticks]


def run_one(item):
    profile_key, world_name, seed, cond_name, out_root = item
    S = A._sim_in_loop()
    started = time.perf_counter()
    cond = None if cond_name == 'off' else condition(cond_name)
    episode = spec(profile_key, world_name, seed, None if cond is None else cond['gnss'], cond_name)
    wide = _wide_condition(world_name)
    result, facade = S.run_episode(episode, None, keep=True)
    try:
        if out_root is not None:
            result = facade.write(os.path.join(out_root, episode['name']), printable(episode, profile_key))
        arrays = facade.trace_arrays()
        ticks = [r for r in facade.log.records if r.get('k') == 'tick']
        art = M.artefacts(facade, printable(episode, profile_key))
        metrics = FW.measure(wide, arrays, ticks, facade.summary(), facade.rig.world, episode['world']['markers'], art,
                             facade.rig.gnss_report())
        metrics['after_terminal'] = FM.terminal_window(art, metrics)
        jog = jog_metrics(arrays, world_name)
        rtk = rtk_metrics(ticks)
        receiver = facade.rig.gnss_report()
    finally:
        facade.rig.close()
    checks = FW.judge(wide, metrics)
    cmd = commands(ticks)
    return dict(profile=profile_key, world=world_name, seed=seed, condition=cond_name,
                role=None if cond is None else cond['role'], name=episode['name'],
                follow_ok=all(c['ok'] for c in checks), follow_failed=[c['check'] for c in checks if not c['ok']],
                metrics=metrics, jog=jog, rtk=rtk,
                receiver=None if receiver is None else dict(counts=receiver['counts'],
                                                            jump_stamps=receiver['jump_stamps']),
                commands_sha256=hashlib.sha256(json.dumps(cmd, separators=(',', ':')).encode('ascii')).hexdigest(),
                digests=result['digests'], config_digest=result['config_digest'],
                wall_s=round(time.perf_counter() - started, 1))


def items(profiles, seeds, only=None):
    out = []
    for profile_key in profiles:
        for seed in seeds:
            conds = [c for c in CONDITIONS if not only or c['name'] in only]
            for world_name in sorted({c['world'] for c in conds}, key=list(WORLDS).index):
                out.append((profile_key, world_name, seed, 'off'))
            out.extend((profile_key, c['world'], seed, c['name']) for c in conds)
    return out


def judge(rows):
    """Registered checks of each rtk run against its off twin, and the benefit rule per (profile, condition)."""
    crit = CRITERIA
    off = {(r['profile'], r['world'], r['seed']): r for r in rows if r['condition'] == 'off'}
    judged = []
    for r in rows:
        if r['condition'] == 'off':
            r['checks'] = [dict(check='follow_wide row_end checks (off twin)', ok=r['follow_ok'],
                                measured=r['follow_failed'], limit='all pass')]
            r['ok'] = r['follow_ok']
            continue
        twin = off.get((r['profile'], r['world'], r['seed']))
        m, mo, rtk = r['metrics'], (twin or {}).get('metrics') or {}, r.get('rtk') or {}
        checks = []

        def add(name, ok, measured, limit):
            checks.append(dict(check=name, ok=bool(ok), measured=measured, limit=limit))
        add('follow_wide row_end checks', r['follow_ok'], r['follow_failed'], 'all pass')
        add('off twin present', twin is not None, None if twin is None else twin['name'], 'present')
        add('same end reason as off', twin is not None and m.get('end_reason') == mo.get('end_reason'),
            [m.get('end_reason'), mo.get('end_reason')], 'equal')
        add('|correction| <= cap', rtk.get('max_abs_correction_m') is not None
            and rtk['max_abs_correction_m'] <= crit['cap_m'] + crit['cap_tolerance_m'],
            rtk.get('max_abs_correction_m'), '<= %g m' % crit['cap_m'])
        add('no step: correction step <= rate x dt', rtk.get('max_step_over_rate_limit_m') is not None
            and rtk['max_step_over_rate_limit_m'] <= crit['rate_tolerance_m'],
            rtk.get('max_step_over_rate_limit_m'), '<= %g m over the rate limit' % crit['rate_tolerance_m'])
        if r['role'] in ('harmless', 'benefit'):
            r['delta_vs_off'] = {key: (None if m.get(key) is None or mo.get(key) is None
                                       else round(m[key] - mo[key], 4)) for key in ('lateral_rmse_m', 'lateral_max_m')}
        if r['role'] == 'identical':
            add('commands equal the off twin', twin is not None and r['commands_sha256'] == twin['commands_sha256'],
                None if twin is None else r['commands_sha256'] == twin['commands_sha256'], 'equal')
            add('correction never applied', rtk.get('max_abs_correction_m') == 0., rtk.get('max_abs_correction_m'), 0.)
        r['checks'] = checks
        r['ok'] = all(c['ok'] for c in checks)
        judged.append(r)
    groups = []
    for profile_key in sorted({r['profile'] for r in judged}):
        for c in CONDITIONS:
            if c['role'] not in ('harmless', 'benefit'):
                continue
            part = [(r, off[(r['profile'], r['world'], r['seed'])]) for r in judged
                    if r['profile'] == profile_key and r['condition'] == c['name']
                    and (r['profile'], r['world'], r['seed']) in off]
            if not part:
                continue
            group = dict(profile=profile_key, condition=c['name'], role=c['role'], seeds=len(part), checks=[])

            def add(name, ok, measured, limit):
                group['checks'].append(dict(check=name, ok=bool(ok), measured=measured, limit=limit))
            for key, margin in (('lateral_rmse_m', crit['harm_rmse_m']), ('lateral_max_m', crit['harm_max_m'])):
                values = [(r['metrics'].get(key), o['metrics'].get(key)) for r, o in part]
                complete = all(a is not None and b is not None for a, b in values)
                mean_rtk = float(np.mean([a for a, _ in values])) if complete else None
                mean_off = float(np.mean([b for _, b in values])) if complete else None
                add('mean %s <= mean off + %g' % (key, margin), complete and mean_rtk <= mean_off + margin,
                    None if not complete else [round(mean_rtk, 4), round(mean_off, 4)], 'off + %g m' % margin)
            if c['role'] == 'benefit':
                pairs = [(r.get('jog'), o.get('jog')) for r, o in part]
                complete = all(a and b for a, b in pairs)
                mean_rtk = float(np.mean([a['mean_abs_m'] for a, _ in pairs])) if complete else None
                mean_off = float(np.mean([b['mean_abs_m'] for _, b in pairs])) if complete else None
                reduction = None if not complete or mean_off <= 0. else 1. - mean_rtk / mean_off
                better = sum(1 for a, b in pairs if a and b and a['mean_abs_m'] <= b['mean_abs_m'])
                group.update(jog_mean_abs_rtk_m=None if mean_rtk is None else round(mean_rtk, 4),
                             jog_mean_abs_off_m=None if mean_off is None else round(mean_off, 4),
                             reduction=None if reduction is None else round(reduction, 3), seeds_not_worse=better)
                add('jog mean |d| reduced by >= %g' % crit['benefit_jog_reduction'],
                    reduction is not None and reduction >= crit['benefit_jog_reduction'],
                    None if reduction is None else round(reduction, 3), '>= %g' % crit['benefit_jog_reduction'])
                add('jog mean |d| not above off in more than half of the seeds', 2 * better > len(pairs),
                    '%d of %d' % (better, len(pairs)), '> half')
            group['ok'] = all(ch['ok'] for ch in group['checks'])
            groups.append(group)
    return groups


def run_all(out_dir, seed_set, profiles=tuple(PROFILES), workers=1, only=None):
    started = time.perf_counter()
    seeds = SEED_SETS[seed_set]
    todo = [item + (os.path.join(out_dir, 'runs'),) for item in items(profiles, seeds, only)]
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=min(4, workers)) as pool:
            rows = list(pool.map(run_one, todo))
    else:
        rows = [run_one(item) for item in todo]
    groups = judge(rows)
    judged = [r for r in rows if r['condition'] != 'off']
    report = dict(schema=SCHEMA, prereg=PREREG_ID, result_kind=A.RESULT_KIND, field_validated=False, seed_set=seed_set,
                  seeds=list(seeds), profiles=list(profiles), only=only, criteria=CRITERIA,
                  complete=not only and set(profiles) == set(PROFILES), runs=len(rows),
                  rtk_runs=len(judged), rtk_passed=sum(1 for r in judged if r['ok']),
                  off_runs=len(rows) - len(judged), off_passed=sum(1 for r in rows if r['condition'] == 'off' and r['ok']),
                  groups=groups, groups_passed=sum(1 for g in groups if g['ok']),
                  ok=bool(judged) and all(r['ok'] for r in rows) and all(g['ok'] for g in groups),
                  rows=rows, workers=workers, wall_s=round(time.perf_counter() - started, 1))
    os.makedirs(out_dir, exist_ok=True)
    raw = json.dumps(report, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False, default=str) + '\n'
    with open(os.path.join(out_dir, 'RTK_MATRIX.json'), 'wb') as fh:
        fh.write(raw.encode('utf-8'))
    return report


def _rtk_settings():
    """The executor's RtkSettings defaults (row_executor.perception_settings; the gnss_rtk_* keys are not set)."""
    import dataclasses
    A._sim_in_loop()                                  # puts tools/ on sys.path (row_executor)
    from row_executor.perception_settings import RtkSettings
    return json.loads(json.dumps(dataclasses.asdict(RtkSettings())))


def prereg():
    """The machine-readable pre-registration (docs/steering/PREREG_GNSS_RTK.json)."""
    return dict(schema=PREREG_SCHEMA, id=PREREG_ID, result_kind='placeholder-based SIM result',
                simulator='F-SIM T1 rig, executor in the loop (tools/row_executor/sim_in_loop_steer.py)',
                profiles={k: dict(profile_id=v['profile_id'], executor_config=dict(v['config']),
                                  controller='row_follow' + (' with the profile wheelbase' if v['profile_wheelbase']
                                                             else ''))
                          for k, v in PROFILES.items()},
                seeds=dict(dev=list(DEV_SEEDS), judge=list(JUDGE_SEEDS)), duration_s=DURATION_S,
                worlds={k: FW.geometry(v).to_dict() for k, v in WORLDS.items()}, jog_window_s=list(JOG_WINDOW_S),
                receiver=dict(RTK, models='steering/gnss_sim.py RTK_MODELS (placeholders)'),
                executor=dict(EXECUTOR_RTK, rtk_settings=_rtk_settings()),
                conditions=[json.loads(json.dumps(c)) for c in CONDITIONS], roles=list(ROLES), criteria=CRITERIA,
                twin='off: the same profile, world, seed and stream key, gnss_mode off, no GNSS packets',
                runs_per_seed_set=len(PROFILES) * (len(CONDITIONS) + len(WORLDS)))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--out')
    p.add_argument('--seeds', choices=sorted(SEED_SETS), default='dev')
    p.add_argument('--profiles', nargs='*', default=list(PROFILES))
    p.add_argument('--only', nargs='*')
    p.add_argument('--workers', type=int, default=1)
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
    report = run_all(args.out, args.seeds, tuple(args.profiles), args.workers, args.only)
    for r in report['rows']:
        m, rtk, jog = r['metrics'], r.get('rtk') or {}, r.get('jog') or {}
        print('%-4s %-3s %-22s s%-4d end %-10s rmse %-6s max %-6s jog %-6s u %-6s step %-7s %-8s %s' % (
            'ok' if r['ok'] else 'FAIL', r['profile'], r['condition'], r['seed'], m.get('end_reason'),
            m.get('lateral_rmse_m'), m.get('lateral_max_m'), jog.get('mean_abs_m'), rtk.get('max_abs_correction_m'),
            rtk.get('max_step_m'), rtk.get('frame'),
            '' if r['ok'] else 'failed: ' + '; '.join(c['check'] for c in r['checks'] if not c['ok'])))
    for g in report['groups']:
        print('group %-4s %-3s %-22s %s %s' % ('ok' if g['ok'] else 'FAIL', g['profile'], g['condition'],
                                              '; '.join('%s %s' % (c['check'], c['measured']) for c in g['checks']),
                                              '' if g['role'] != 'benefit' else 'jog %s -> %s' % (
                                                  g.get('jog_mean_abs_off_m'), g.get('jog_mean_abs_rtk_m'))))
    print('%s: rtk runs %d/%d, off %d/%d, groups %d/%d, %.0f s' % (
        'PASS' if report['ok'] else 'FAIL', report['rtk_passed'], report['rtk_runs'], report['off_passed'],
        report['off_runs'], report['groups_passed'], len(report['groups']), report['wall_s']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
