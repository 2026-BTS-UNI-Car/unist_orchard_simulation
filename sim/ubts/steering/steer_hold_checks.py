"""Profile v0.2 checks and the straight hold under steering disturbances on the T1 rig (ST-10, ST-11).

Pre-registered in docs/steering/SIM_ACCEPTANCE_KO.md amendments 40 (sets 'normal' and 'faults') and 41 (set
'disturbance'). The rig runs the real firmware V3, the host bridge, the L1 actuator / potentiometer model (with the
steer_ext block of a schema v3 profile) and the plant (T1_RIG_KO.md); the cases drive it through the control API
directly (no executor): throttle and steering every 0.1 s.

  python -B -m steering.steer_hold_checks run --set disturbance --out <dir outside the repo> [--profile <yaml>]
          [--seeds judgement|dev|1,2,3] [--only D1,D5] [--workers N] [--build-dir <dir>]
  python -B -m steering.steer_hold_checks check --run <dir>          judgement of RESULTS.json (exit 0 pass, 1 fail)

Per case and seed (RESULTS.json 'episodes'): MCU F lines (code, queued tick), host latches, the steering deviation
from the pre-onset base angle, the recovery time, the lateral deviation from the pre-onset path line, the thermal
budget use, and for the fault cases the recomputed judgement tick (safety_scripts.measure.expected_fault_tick).
Evaluation code (A13 role 'evaluator'): it reads the rig truth and MCU channels. Placeholder-based SIM results
(자리값 기반 SIM 결과), not field validation.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

from . import backend_api as ba
from . import t1_rig as tr
from . import vehicle_profile as vp
from .safety_scripts import measure as M

SCHEMA = 'ubts.steer_hold_checks.v1'
RESULT_KIND = '자리값 기반 SIM 결과'
PROFILE_V02 = vp.PROFILES_DIR / 'front_steer_v0_2_placeholder.yaml'
PROFILE_V03 = vp.PROFILES_DIR / 'front_steer_v0_3_placeholder.yaml'
STEP_S = 0.1                   # control period of the cases (drive / steering command)
THROTTLE = 60                  # amendment 41: throttle 60 %
T0 = 6.0                       # disturbance onset (amendment 41)
SEEDS = dict(        # 'judgement': amendments 40-42 (profile v0.2); 'judgement43': amendment 43 (profile v0.3)
    disturbance=dict(judgement=(1000, 1001, 1002, 1003, 1004), judgement43=(1500, 1501, 1502, 1503, 1504),
                     dev=(0, 1, 2)),
    normal=dict(judgement=tuple(range(1100, 1110)), judgement43=tuple(range(1300, 1310)), dev=(0, 1)),
    faults=dict(judgement=(1200, 1201, 1202), judgement43=(1400, 1401, 1402), dev=(0,)),
)
JUDGEMENT_KINDS = {'judgement': None, 'judgement43': 43}
STEER_FAULTS = ('POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'STALL', 'TRACK', 'THERMAL')
NORMAL_FORBIDDEN = ('POLARITY', 'DRV_STUCK', 'THERMAL', 'TRACK')
ABOVE_DEFINED = ('STALL', 'TRACK', 'DRV_STUCK')       # amendment 42 (amendment 41: STALL, TRACK)
ABOVE_DEFINED_41 = ('STALL', 'TRACK')
ABOVE_DEFINED_43 = ('STALL', 'TRACK', 'DRV_STUCK', 'POLARITY')   # amendment 43: an overpowering push looks like POLARITY
RECOVERY_HOLD_S = 0.5
RECOVERY_LIMIT_S = 2.0


# --- cases ------------------------------------------------------------------------------------------------------------------
def _torque(at_s, torque_nm, duration_s):
    return dict(kind='steer_torque', at_s=at_s, params=dict(torque_nm=float(torque_nm), duration_s=float(duration_s)))


def _straight(t):
    return THROTTLE, 0.


def _segments(segs, throttle):
    """segs: [(duration_s, steer_deg)] after a 2 s straight lead; then 0 deg."""
    def cmd(t):
        start = 2.
        for duration, steer in segs:
            if start <= t < start + duration:
                return throttle, steer
            start += duration
        return throttle, 0.
    return cmd


def _normal_cases():
    out = {}
    out['N1'] = dict(group='normal', duration_s=22., cmd=_straight, purpose='straight 60 % 20 s')
    out['N2'] = dict(group='normal', duration_s=16., cmd=_segments([(2., 6.), (2., -6.)] * 3, THROTTLE),
                     purpose='zigzag +-6 deg, 4 s period x 3')
    out['N3'] = dict(group='normal', duration_s=14., cmd=_segments([(3., 10.), (2., 0.), (3., -10.)], THROTTLE),
                     purpose='S curve 10 deg left 3 s, straight 2 s, right 3 s')
    out['N4'] = dict(group='normal', duration_s=18., cmd=_segments([(2.5, s) for s in (5., 0., -5., 0., 5., 0.)], 0),
                     purpose='standstill steps +-5 deg every 2.5 s')
    out['N5'] = dict(group='normal', duration_s=22., cmd=_segments([(3., s) for s in (20., -20.) * 3], 0),
                     purpose='standstill full sweeps +-20 deg, 3 s per end, 3 round trips')
    out['N6'] = dict(group='normal', duration_s=16., cmd=_segments([(6., 15.), (6., 0.)], THROTTLE),
                     purpose='moving 15 deg 6 s then 0 deg 6 s (non-self-locking centring)')
    return out


def _fault_cases():
    def step_at_3(t):
        return 0, (5. if t >= 3. else 0.)
    return {
        'G1': dict(group='faults', duration_s=7., cmd=step_at_3, model=[dict(kind='polarity_reversed', at_s=2.,
                                                                             params={})],
                   onset_s=3., expect=('POLARITY',), purpose='polarity reversed, standstill 5 deg step'),
        'G2': dict(group='faults', duration_s=8., cmd=_straight, model=[dict(kind='driver_stuck_on', at_s=4.,
                                                                             params=dict(pwm=.6))],
                   onset_s=4., expect=('DRV_STUCK',), purpose='driver stuck on (pwm 0.6) while driving straight'),
        'G3': dict(group='faults', duration_s=7., cmd=step_at_3, model=[dict(kind='jam', at_s=2., params={})],
                   onset_s=3., expect=('STALL', 'TRACK'), purpose='jammed actuator, standstill 5 deg step'),
    }


def _disturbance_cases(profile):
    belt = profile.has('belt_tooth_step_deg')           # a schema v2 profile (v0, v0.1) has no belt model: no R3
    skip = ba.belt_skip_injection(profile, T0 + 1., teeth=-1) if belt else None
    within = dict(group='within', expect=())
    above = dict(group='above', expect=ABOVE_DEFINED)
    report = dict(group='report', expect=())
    cases = {
        'D1': dict(within, torque=(2.5, 3.), purpose='+2.5 N m step 3 s'),
        'D2': dict(within, torque=(-2.5, 3.), purpose='-2.5 N m step 3 s'),
        'D3': dict(within, torque=(2.5, .1), purpose='+2.5 N m impact 0.1 s'),
        'D4': dict(within, torque=(2., 10.), purpose='+2.0 N m sustained 10 s'),
        'D5': dict(above, torque=(5., 5.), purpose='+5.0 N m step 5 s'),
        'D6': dict(above, torque=(-5., 5.), purpose='-5.0 N m step 5 s'),
        'R1': dict(report, torque=(3.5, 3.), purpose='+3.5 N m step 3 s (boundary)'),
        'R2': dict(report, torque=None, end_s=T0 + 15., purpose='ground impacts 1/m sd 1.0 N m 0.05 s + push sd 0.5 '
                   'N m 2 m, 15 s',
                   overrides={'steer_terrain_impacts': {'rate_per_m': 1.0, 'torque_sd_nm': 1.0, 'duration_s': .05},
                              'steer_terrain_push': {'sd_nm': .5, 'corr_length_m': 2.}}),
        'R3': dict(report, torque=(2.5, 3.), model_extra=[skip], purpose='+2.5 N m step 3 s + belt skip -1 tooth at '
                   't0 + 1 s'),
        'R4': dict(report, torque=(2.5, 3.), runtime=[('motor_power_off', T0)], purpose='steering motor power off at '
                   't0 + +2.5 N m 3 s (back-driven)'),
    }
    if not belt:
        del cases['R3']
    for name, case in cases.items():
        torque = case.get('torque')
        case['model'] = ([_torque(T0, *torque)] if torque else []) + list(case.get('model_extra', ()))
        case['end_s'] = case.get('end_s') or (T0 + torque[1])
        case['onset_s'] = T0
        case['duration_s'] = case['end_s'] + 4.   # recovery (<= 2 s) plus the 2 s lateral window
        case['cmd'] = _straight
    return cases


def cases_of(set_name, profile):
    if set_name == 'normal':
        return _normal_cases()
    if set_name == 'faults':
        return _fault_cases()
    if set_name == 'disturbance':
        return _disturbance_cases(profile)
    raise ValueError('unknown set %r' % (set_name,))


# --- one episode -------------------------------------------------------------------------------------------------------------
def _profile_view(profile, case):
    overrides = case.get('overrides')
    return profile.with_overrides(overrides, label='steer_hold_' + 'terrain') if overrides else profile


def _hot_window_ms(channel, thp, end_tick, window_ms=30000):
    """Largest number of ticks with |steer_pwm| >= thp inside any window of window_ms ticks."""
    if not channel:
        return 0
    ticks = np.zeros(end_tick + 1, dtype=np.int8)
    changes = list(channel) + [(end_tick + 1, 0)]
    for (t0, v), (t1, _) in zip(changes, changes[1:]):
        if abs(v) >= thp and t1 > t0:
            ticks[max(0, t0):min(end_tick + 1, t1)] = 1
    csum = np.concatenate([[0], np.cumsum(ticks)])
    if len(ticks) <= window_ms:
        return int(csum[-1])
    return int(np.max(csum[window_ms:] - csum[:-window_ms]))


def _art(rig):
    channels = {}
    for name in M.CHANNEL_NAMES:
        try:
            channels[name] = [(int(t), int(v)) for t, v in rig.mcu_channel(name)]
        except KeyError:
            channels[name] = []
    lines = [(line.text, int(line.queued_tick), int(line.done_tick)) for line in rig.mcu_lines()]
    return dict(cfg=dict((rig.cal or {}).get('params') or {}), channels=channels, lines=lines)


def _f_lines(art):
    out = []
    for text, queued, _ in M.mcu_lines(art, 'F'):
        parts = text.split('*')[0].split(' ')
        if len(parts) > 1:
            out.append((parts[1], queued))
    return out


def run_episode(set_name, case_name, seed, profile_path=None, build_dir=None, with_trace=False):
    profile = vp.load_profile(profile_path) if profile_path else vp.load_profile(PROFILE_V02)
    case = cases_of(set_name, profile)[case_name]
    view = _profile_view(profile, case)
    faults = [tr.RigFault(m['kind'], float(m['at_s']), dict(m['params'])) for m in case.get('model', ())]
    faults += [tr.RigFault(kind, float(at)) for kind, at in case.get('runtime', ())]
    config = tr.RigConfig(seed=int(seed), stream_key='steer_hold_' + case_name, profile=view, faults=tuple(faults))
    started = time.perf_counter()
    errors = []
    with tr.T1Rig(config, build_dir=build_dir) as rig:
        client = rig.client('steer_hold')
        rig.run_until_true(lambda: rig.core.steering_host()['stop_state'] == 'confirmed', 3.)
        problem = client.arm_precheck()
        reply = client.arm()
        armed = bool(reply.get('ok')) and client.wait_armed()
        if not armed:
            errors.append('arm:%s:%s' % (problem, reply.get('error')))
        steps = int(round(case['duration_s'] / STEP_S))
        for k in range(steps):
            t = rig.now
            throttle, steer_deg = case['cmd'](t)
            try:
                reply = client.drive(int(throttle), int(round(math.radians(steer_deg) * 1000.)))
                if not reply.get('ok') and len(errors) < 5:
                    errors.append('drive@%.1f:%s' % (t, reply.get('error')))
            except Exception as exc:   # a stopped bridge or a revoked lease: keep the rig running
                if len(errors) < 5:
                    errors.append('drive@%.1f:%s' % (t, type(exc).__name__))
            rig.step(STEP_S)
        trace = rig.trace_arrays()
        art = _art(rig)
        latches = sorted(rig.core.latches)
        identity = rig.identity(include_files=False)
        end_tick = int(rig.ms)
    record = dict(set=set_name, case=case_name, seed=int(seed), group=case['group'], purpose=case.get('purpose'),
                  identity=identity, errors=errors, armed=armed, latches=latches, wall_s=None)
    record.update(_measure(case, trace, art, view, end_tick))
    if with_trace:   # figures (tools/steer_sim/steer_model_figures.py): every 5th plant step
        record['trace'] = {k: [round(float(x), 6) for x in list(trace[k])[::5]]
                           for k in ('t_s', 'truth_delta_eq_rad', 'truth_x_m', 'truth_y_m', 'truth_v_m_s')}
    record['wall_s'] = round(time.perf_counter() - started, 2)
    return record


def _measure(case, trace, art, profile, end_tick):
    t = np.asarray(trace['t_s'], dtype=float)
    delta = np.degrees(np.asarray(trace['truth_delta_eq_rad'], dtype=float))
    x = np.asarray(trace['truth_x_m'], dtype=float)
    y = np.asarray(trace['truth_y_m'], dtype=float)
    v = np.asarray(trace['truth_v_m_s'], dtype=float)
    f_lines = _f_lines(art)
    cfg = art['cfg']
    thp = int(cfg.get('thp', 204))
    tha = int(cfg.get('tha', 5000))
    hot = _hot_window_ms(art['channels']['steer_pwm'], thp, end_tick)
    out = dict(f_lines=f_lines, thermal_hot_ms_max_30s=hot, thermal_budget_use=round(hot / float(tha), 4),
               speed_mean_m_s=round(float(np.mean(v[t > 3.])) if np.any(t > 3.) else 0., 4))
    onset = case.get('onset_s')
    if onset is None:
        return out
    tol = (profile.value('steer_backlash_deg') / 2. + profile.value('servo_deadband_deg')
           + (profile.value('belt_pot_play_deg') / 2. if profile.has('belt_pot_play_deg') else 0.) + .5)
    base_mask = (t >= onset - 1.) & (t < onset)
    base = float(np.mean(delta[base_mask])) if np.any(base_mask) else 0.
    end = case.get('end_s', onset)
    after = t >= onset
    dev = np.abs(delta - base)
    out.update(delta_base_deg=round(base, 4), tolerance_deg=round(tol, 4),
               max_steer_dev_deg=round(float(np.max(dev[after])) if np.any(after) else 0., 4))
    # recovery: from the end of the disturbance, first time |delta - base| <= tol holds RECOVERY_HOLD_S
    recovery = None
    idx = np.where(t >= end)[0]
    hold = int(round(RECOVERY_HOLD_S / (t[1] - t[0]))) if len(t) > 1 else 1
    ok = dev <= tol
    for i in idx:
        if i + hold <= len(t) and np.all(ok[i:i + hold]):
            recovery = round(float(t[i] - end), 3)
            break
    out['recovery_s'] = recovery
    # lateral deviation from the straight line of the 2 s before the onset
    ref = (t >= onset - 2.) & (t < onset)
    lateral = None
    if np.sum(ref) >= 5 and np.ptp(x[ref]) > 1e-3:
        b, a = np.polyfit(x[ref], y[ref], 1)
        stop = end + (recovery if recovery is not None else (t[-1] - end)) + 2.
        win = (t >= onset) & (t <= stop)
        lateral = round(float(np.max(np.abs(y[win] - (a + b * x[win])) / math.sqrt(1. + b * b))), 4)
    out['max_lateral_dev_m'] = lateral
    onset_tick = int(round(onset * 1000.))
    steer_faults = [(code, tick) for code, tick in f_lines if code in STEER_FAULTS and tick >= onset_tick]
    out['first_steer_fault'] = list(steer_faults[0]) if steer_faults else None
    expect = case.get('expect') or ()
    if steer_faults and expect:
        code, tick = steer_faults[0]
        try:
            expected, method = M.expected_fault_tick(code, art, onset_tick)
        except Exception as exc:   # report, judged as a failure
            expected, method = None, 'error:%s' % exc
        out['expected_tick'] = expected
        out['expected_method'] = method
        out['tick_diff'] = None if expected is None else int(tick - expected)
    return out


# --- run / check ---------------------------------------------------------------------------------------------------------------
def _seeds(set_name, text):
    if text in SEEDS[set_name]:
        return list(SEEDS[set_name][text])
    return [int(s) for s in text.split(',') if s.strip()]


def _job(item):
    return run_episode(*item)


def run(set_name, out_dir, profile_path=None, seeds='judgement', only=None, workers=1, build_dir=None):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if profile_path is None and seeds == 'judgement43':
        profile_path = PROFILE_V03                  # amendment 43 judges profile v0.3
    profile = vp.load_profile(profile_path) if profile_path else vp.load_profile(PROFILE_V02)
    names = [n for n in cases_of(set_name, profile) if only is None or n in only]
    seed_list = _seeds(set_name, seeds)
    from .native import build as native_build
    built = native_build.ensure_built(build_dir=build_dir or os.environ.get(native_build.ENV_BUILD_DIR))
    jobs = [(set_name, n, s, str(profile_path) if profile_path else None, str(Path(built.path).parent))
            for n in names for s in seed_list]
    started = time.perf_counter()
    if workers <= 1:
        episodes = [_job(j) for j in jobs]
    else:
        import multiprocessing
        ctx = multiprocessing.get_context('spawn')
        with concurrent.futures.ProcessPoolExecutor(max_workers=workers, mp_context=ctx) as pool:
            episodes = list(pool.map(_job, jobs))
    result = dict(schema=SCHEMA, result_kind=RESULT_KIND, field_validated=False, set=set_name, seeds=seed_list,
                  seed_kind=seeds if seeds in SEEDS[set_name] else 'list', cases=names,
                  profile=profile.result_label(), profile_path=str(profile_path or PROFILE_V02),
                  native_binary_sha256=built.binary_sha256, firmware_build_id=built.firmware_build_id,
                  amendment=43 if seeds == 'judgement43' else dict(normal=40, faults=40, disturbance=41)[set_name],
                  workers=workers,
                  wall_s=round(time.perf_counter() - started, 1), episodes=episodes)
    data = json.dumps(result, ensure_ascii=False, indent=1, sort_keys=True).encode('utf-8')
    (out_dir / 'RESULTS.json').write_bytes(data)
    return result


def check(result):
    """Judgement of a RESULTS.json dict (amendments 40, 41). Returns (passed, items)."""
    items = []
    judgement = result.get('seed_kind') in JUDGEMENT_KINDS
    above_defined = ABOVE_DEFINED_43 if result.get('amendment') == 43 else ABOVE_DEFINED
    for ep in result['episodes']:
        group, name, seed = ep['group'], ep['case'], ep['seed']
        codes = [c for c, _ in ep['f_lines']]
        if group == 'normal':
            bad = [c for c in codes if c in NORMAL_FORBIDDEN]
            ok = not bad and 'steering_not_following' not in ep['latches'] and ep['armed']
            why = 'forbidden F %s, latches %s' % (bad, ep['latches'])
        elif group in ('faults', 'above'):
            first = ep.get('first_steer_fault')
            exp_codes = above_defined if group == 'above' else _fault_cases()[name]['expect']
            tick_ok = ep.get('tick_diff') is not None and abs(ep['tick_diff']) <= 1
            ok = first is not None and first[0] in exp_codes and tick_ok and ep['armed']
            if group == 'faults' and first is not None:
                ok = ok and first[1] - int(_fault_cases()[name]['onset_s'] * 1000) <= 3000
            why = 'first %s, tick diff %s' % (first, ep.get('tick_diff'))
            if group == 'above' and result.get('amendment') != 43:
                items.append(dict(case=name, seed=seed, group='above_original_41', report_only=True,
                                  passed=bool(first is not None and first[0] in ABOVE_DEFINED_41 and tick_ok),
                                  detail=why))
        elif group == 'within':
            ok = (not codes and not ep['latches'] and ep['recovery_s'] is not None
                  and ep['recovery_s'] <= RECOVERY_LIMIT_S and ep['armed'])
            why = 'F %s, latches %s, recovery %s' % (codes, ep['latches'], ep['recovery_s'])
        else:
            continue   # report
        items.append(dict(case=name, seed=seed, group=group, passed=bool(ok), detail=why))
    judged = [i for i in items if not i.get('report_only')]
    passed = judgement and all(i['passed'] for i in judged) and bool(judged)
    return passed, items


def summary_rows(result):
    rows = []
    for ep in result['episodes']:
        rows.append(dict(case=ep['case'], seed=ep['seed'], group=ep['group'], f=[c for c, _ in ep['f_lines']],
                         latches=ep['latches'], max_dev=ep.get('max_steer_dev_deg'), recovery=ep.get('recovery_s'),
                         lateral=ep.get('max_lateral_dev_m'), thermal=ep.get('thermal_budget_use'),
                         first=ep.get('first_steer_fault'), tick_diff=ep.get('tick_diff'),
                         speed=ep.get('speed_mean_m_s'), errors=ep.get('errors')))
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--set', required=True, choices=('normal', 'faults', 'disturbance'))
    r.add_argument('--out', required=True)
    r.add_argument('--profile', default=None)
    r.add_argument('--seeds', default='judgement')
    r.add_argument('--only', default=None)
    r.add_argument('--workers', type=int, default=1)
    r.add_argument('--build-dir', default=None)
    c = sub.add_parser('check')
    c.add_argument('--run', required=True)
    args = ap.parse_args(argv)
    if args.cmd == 'run':
        from .scene_spec import inside_repository
        if inside_repository(Path(args.out)):
            ap.error('--out must be outside the repository')
        only = set(args.only.split(',')) if args.only else None
        result = run(args.set, args.out, args.profile, args.seeds, only, args.workers, args.build_dir)
        for row in summary_rows(result):
            print(json.dumps(row, ensure_ascii=False))
        passed, items = check(result)
        print('judgement:', 'PASS' if passed else ('FAIL' if result['seed_kind'] == 'judgement' else 'not a '
                                                                                                    'judgement run'))
        return 0
    result = json.loads(Path(args.run, 'RESULTS.json').read_bytes())
    passed, items = check(result)
    for item in items:
        if not item['passed']:
            print(json.dumps(item, ensure_ascii=False))
    original = [i for i in items if i['group'] == 'above_original_41']
    if original:
        print('amendment 41 original criterion (report):', 'PASS' if all(i['passed'] for i in original) else 'FAIL')
    print('judgement:', 'PASS' if passed else 'FAIL', '(%d items)' % len(items))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
