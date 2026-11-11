"""A18(F): does the identification recover the known placeholder values? (design 3.9, SIM_ACCEPTANCE A18; evaluator)

For every profile view (the v0.1 placeholder and the variants below, inside the sweep ranges) and seed:
  1. procedures.run_session(): the bench tests B0-B7 and the drive tests F1-F5 in an IdentRig write field-format
     records (operator stand-in scripts; nothing is read from the SIM state by the analysis),
  2. analysis.analyse_session(): every estimate from the records only,
  3. compare(): each estimate against the value the SIM knows (eval/truth.json and eval/trace.npz; only here).

Bounds (SIM_ACCEPTANCE A18, proposal): steering rate +-10 %, backlash +-0.3 deg, zero offset +-0.2 deg, kappa(delta)
table +-5 % (every row), minimum PWM +-3 %p, tau +-20 %, duty -> speed +-5 % (every duty). Known values:
  steering rate      steer_rate_max_deg_s (moving, B5 speed line at u = 1) and steer_rate_standstill_deg_s
                     (standstill, B4 speed line at u = 1), equivalent-angle degrees per second
  minimum PWM        steer_min_pwm_pct (moving, B5 line) and steer_min_pwm_standstill_pct (standstill, B4 line)
  tau                steer_motor_tau_s (B4)
  backlash           steer_backlash_deg (B6)
  zero offset        F1 measures the straight-line curvature at steering command 0 and states it as the equivalent
                     angle atan(L kappa0). Known value: atan(L (kappa_steer + kappa_pull)) with kappa_steer the plant
                     steering curvature over the analysed windows (path weighted) and kappa_pull the constant part of
                     the curvature wander (profile curvature_wander mean). The random part of the wander (OU, sd 0.95
                     deg/m, 8 m) is not known to the record and stays in the error: about 2.9 / sqrt(path m) deg.
                     Reported as well: the realised path curvature (total, with the realised wander) and the mean
                     equivalent wheel angle (steering only).
  kappa(delta)       per F2 row (commanded angle): kappa_steer over the row window + kappa_pull; the random wander
                     adds about 3.8 / sqrt(path m) deg/m. Reported as well: the realised path curvature.
  duty -> speed      per F3 duty: forward_curve(applied duty x battery) x speed_run_scale (procedures.session_truth)
Reported without a bound: B3 marks against the mechanical limits, B3 Ackermann ratio, B0 polarity, B7 noise, B5
full-PWM speeds, F3 stop distances, F4 / F5 slew and faults.

CLI (from sim/ubts):
  python -B -m steering.ident.a18 --out <runs folder> [--variants default,A,B] [--seeds 11,12,13] [--jobs 3] [--quick]
writes <out>/<variant>_s<seed>/ (records, eval, session.json, analysis.json, compare.json) and <out>/A18_RESULTS.json.
The output folder must lie outside the repository. Placeholder-based SIM results (자리값 기반 SIM 결과), not field
validation.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback

import numpy as np

SCHEMA = 'ubts.steer_ident.a18.v1'
RESULT_KIND = '자리값 기반 SIM 결과'
BOUNDS = dict(rate_rel=.10, backlash_deg=.3, zero_offset_deg=.2, kappa_rel=.05, min_pwm_pct_points=3., tau_rel=.20,
              speed_rel=.05)
# Profile variants inside the sweep ranges and away from the v0.1 placeholder (VehicleProfile.with_overrides).
# A is a fast motor with a short lag, B a slow motor with a long lag: the default servo (kp 245) stays damped with
# both (lag / servo loop time about the default's). Two first choices are kept as findings (IDENT_KO.md): tau 0.14 s
# at 42 deg/s overshoots and trips POLARITY; a standstill start PWM of 38 % (2 %p under the servo minimum 40 %) makes
# the servo creep and trips POLARITY on the potentiometer noise.
VARIANTS = {
    'default': {},
    'A': dict(steer_rate_max_deg_s=40., steer_rate_standstill_deg_s=20., steer_motor_tau_s=.06, steer_min_pwm_pct=18.,
              steer_min_pwm_standstill_pct=28., steer_backlash_deg=3.5, battery_voltage_scale=.92),
    'B': dict(steer_rate_max_deg_s=20., steer_rate_standstill_deg_s=10., steer_motor_tau_s=.12, steer_min_pwm_pct=30.,
              steer_min_pwm_standstill_pct=33., steer_backlash_deg=.8, battery_voltage_scale=1.04),
}
DEFAULT_SEEDS = (11, 12, 13)


def variant_profile(name):
    from ..vehicle_profile import load_profile
    base = load_profile()
    overrides = VARIANTS[name]
    return base if not overrides else base.with_overrides(dict(overrides), label='a18_variant_' + name)


def quick_plan():
    """A short plan for smoke runs and tests (the A18 numbers come from the default plan)."""
    from .procedures import Plan
    return Plan(cold_s=5., warm_s=5., load_s=4., b4_levels_pct=(40, 60, 80, 100), b4_reps=1,
                b5_moving_levels_pct=(100, 60, 40), b5_standstill_levels_pct=(100,), f4_leg_s=8., f1_leg_m=6.,
                f1_round_trips=1, f2_circles=((13., 1.), (-13., 1.)), f3_duties_pct=(40, 60),
                f3_runs=('release', 'brake'))


# --- truth from the evaluation trace -----------------------------------------------------------------------------------
class Trace:
    def __init__(self, path):
        z = np.load(path)
        self.t = z['t_s']
        self.v = z['truth_v_m_s']
        self.k_steer = z['truth_steer_curvature_1_m']
        self.k_total = z['truth_curvature_1_m']
        self.delta_eq = z['truth_delta_eq_rad']
        self.motor = z['truth_motor_side_rad']
        self.measured = z['steer_measured_rad']
        names = [str(n) for n in z['phase_names']]
        self.drive = np.isin(z['phase'], [k for k, n in enumerate(names) if n.startswith('F')])

    def mean(self, windows, values):
        """Path-weighted mean (weight |v|) of values over the windows [(t0, t1)]."""
        num = den = 0.
        for a, b in windows:
            m = (self.t >= a) & (self.t <= b)
            w = np.abs(self.v[m])
            num += float(np.sum(w * values[m]))
            den += float(np.sum(w))
        return num / den if den > 0 else None


def _rel(est, truth):
    return None if est is None or truth in (None, 0) else (est - truth) / truth


def _row(quantity, test, estimate, truth, error, bound, unit, kind, note=''):
    ok = None if error is None or bound is None else bool(abs(error) <= bound)
    return dict(quantity=quantity, test=test, estimate=estimate, truth=truth, error=error, bound=bound, unit=unit,
                error_kind=kind, passed=ok, note=note)


def _f(x, nd=5):
    return None if x is None else round(float(x), nd)


def compare(case_dir, result):
    """Rows (one per estimate) of one case: estimate, known value, error, A18 bound, pass (module docstring)."""
    case_dir = Path(case_dir)
    with open(case_dir / 'eval' / 'truth.json', encoding='utf-8') as fh:
        truth = json.load(fh)
    trace = Trace(case_dir / 'eval' / 'trace.npz')
    rows = []
    b = BOUNDS
    b4, b5 = result.get('B4') or {}, (result.get('B5') or {}).get('levels') or {}
    mov, sta = b5.get('moving') or {}, b5.get('standstill') or {}
    # steering rate, minimum PWM, tau
    est = mov.get('rate_deg_s_line')
    rows.append(_row('steering_rate_moving', 'B5', est, truth['steer_rate_max_deg_s'],
                     _f(_rel(est, truth['steer_rate_max_deg_s'])), b['rate_rel'], 'deg/s', 'relative',
                     'speed line a u + c at u = 1 (first-order step fits)'))
    est = b4.get('rate_standstill_deg_s_line')
    rows.append(_row('steering_rate_standstill', 'B4', est, truth['steer_rate_standstill_deg_s'],
                     _f(_rel(est, truth['steer_rate_standstill_deg_s'])), b['rate_rel'], 'deg/s', 'relative',
                     'JOG displacement / T, line at u = 1'))
    est = sta.get('rate_deg_s_full_pwm')
    rows.append(_row('steering_rate_standstill_full_pwm', 'B5', est, truth['steer_rate_standstill_deg_s'],
                     _f(_rel(est, truth['steer_rate_standstill_deg_s'])), None, 'deg/s', 'relative',
                     'report: standstill step at the 100 % cap'))
    est = mov.get('min_pwm_pct')
    rows.append(_row('min_pwm_moving', 'B5', est, truth['steer_min_pwm_pct'],
                     _f(None if est is None else est - truth['steer_min_pwm_pct'], 3), b['min_pwm_pct_points'], '%',
                     'percentage points', 'speed line intercept -c / a'))
    est = b4.get('min_pwm_standstill_pct')
    rows.append(_row('min_pwm_standstill', 'B4', est, truth['steer_min_pwm_standstill_pct'],
                     _f(None if est is None else est - truth['steer_min_pwm_standstill_pct'], 3),
                     b['min_pwm_pct_points'], '%', 'percentage points', 'JOG speed line intercept'))
    est = b4.get('tau_s')
    rows.append(_row('tau', 'B4', est, truth['steer_motor_tau_s'], _f(_rel(est, truth['steer_motor_tau_s'])),
                     b['tau_rel'], 's', 'relative', 'first-order fit of the strong JOGs'))
    # backlash
    b6 = result.get('B6') or {}
    est = b6.get('backlash_deg')
    rows.append(_row('backlash', 'B6', est, truth['steer_backlash_deg'],
                     _f(None if est is None else est - truth['steer_backlash_deg'], 4), b['backlash_deg'], 'deg',
                     'absolute', 'gap between the up and the down sweep lines (gauge against potentiometer)'))
    # zero offset (F1)
    L = float(truth['wheelbase_m'])
    pull = math.radians(float(truth['curvature_wander']['mean_deg_per_m']))
    sd_wander = float(truth['curvature_wander']['sd_deg_per_m'])
    corr = float(truth['curvature_wander']['corr_length_m'])
    f1 = result.get('F1')
    if f1:
        windows = [tuple(w) for w in f1['windows']]
        k_steer = trace.mean(windows, trace.k_steer)
        k_total = trace.mean(windows, trace.k_total)
        d_eq = trace.mean(windows, trace.delta_eq)
        known = math.degrees(math.atan(L * (k_steer + pull)))
        realised = math.degrees(math.atan(L * k_total))
        path = float(f1['path_m'])
        expect_sd = L * sd_wander * math.sqrt(2. * corr / max(path, 1e-6))
        est = f1['zero_offset_deg']
        rows.append(_row('zero_offset', 'F1', est, _f(known, 4), _f(est - known, 4), b['zero_offset_deg'], 'deg',
                         'absolute', 'atan(L kappa0) over %.0f m; random wander sd about %.2f deg' % (path,
                                                                                                    expect_sd)))
        rows.append(_row('zero_offset_vs_realised_path', 'F1', est, _f(realised, 4), _f(est - realised, 4), None,
                         'deg', 'absolute', 'report: odometry against the realised curvature (with the wander)'))
        rows.append(_row('zero_offset_wheel_angle', 'F1', est, _f(math.degrees(d_eq), 4),
                         _f(est - math.degrees(d_eq), 4), None, 'deg', 'absolute',
                         'report: mean equivalent wheel angle (steering only; curvature = eta tan(delta) / L)'))
    # kappa(delta) table (F2)
    f2 = result.get('F2')
    for r in (f2 or {}).get('table') or []:
        window = [tuple(w) for w in r['windows']]
        k_steer = trace.mean(window, trace.k_steer)
        k_total = trace.mean(window, trace.k_total)
        known = k_steer + pull
        est = r['kappa_1_m']
        expect = math.radians(sd_wander * math.sqrt(2. * corr / max(abs(r['path_m']), 1e-6))) / max(abs(known), 1e-9)
        rows.append(_row('kappa_table %+.1f deg' % r['target_deg'], 'F2', est, _f(known, 5), _f(_rel(est, known)),
                         b['kappa_rel'], '1/m', 'relative',
                         'path %.0f m; random wander about %.1f %%' % (abs(r['path_m']), 100. * expect)))
        rows.append(_row('kappa_table %+.1f deg vs realised path' % r['target_deg'], 'F2', est, _f(k_total, 5),
                         _f(_rel(est, k_total)), None, '1/m', 'relative', 'report: odometry accuracy'))
    # duty -> speed (F3)
    f3 = result.get('F3') or {}
    for pct, row in sorted(((f3.get('duty_speed') or {}).items()), key=lambda kv: float(kv[0])):
        known = (truth.get('duty_speed') or {}).get(str(pct), {}).get('speed_m_s')
        est = row.get('v_p50_m_s')
        rows.append(_row('duty_speed %s %%' % pct, 'F3', est, _f(known, 4), _f(_rel(est, known)), b['speed_rel'],
                         'm/s', 'relative', 'C3 steady speed p50 (field_calib drive tools)'))
    # reported without a bound: the B2/B3 host calibration seen in the drive phase (measured against the true
    # motor-side angle: gain per side should be 1; the offset is the calibration's zero against the motor side)
    b3 = result.get('B3') or {}
    for side, sel in (('left', trace.measured > math.radians(2.)), ('right', trace.measured < -math.radians(2.))):
        m = trace.drive & sel
        if int(m.sum()) >= 20:
            gain, offset = np.polyfit(trace.motor[m], trace.measured[m], 1)
            rows.append(_row('b3_calibration_gain_%s' % side, 'B3', _f(gain, 5), 1.0, _f(gain - 1., 5), None, '-',
                             'relative', 'report: host-measured angle against the true motor-side angle (drive '
                                         'phase), offset %.3f deg' % math.degrees(offset)))
    rows.append(_row('b3_ackermann_ratio', 'B3', b3.get('ackermann_ratio'), truth['ackermann_ratio'],
                     _f(None if b3.get('ackermann_ratio') is None else b3['ackermann_ratio'] - truth['ackermann_ratio'],
                        4), None, '-', 'absolute', 'report'))
    rows.append(_row('b5_rate_moving_full_pwm', 'B5', mov.get('rate_deg_s_full_pwm'), truth['steer_rate_max_deg_s'],
                     _f(_rel(mov.get('rate_deg_s_full_pwm'), truth['steer_rate_max_deg_s'])), None, 'deg/s',
                     'relative', 'report: step at the 100 % cap'))
    return rows


# --- one case --------------------------------------------------------------------------------------------------------
def run_case(spec):
    """spec: dict(variant, seed, out, quick, native, workers). Runs, analyses and compares one case in this process."""
    sys.dont_write_bytecode = True
    from . import analysis, procedures
    variant, seed = spec['variant'], int(spec['seed'])
    case_dir = Path(spec['out']) / ('%s_s%d' % (variant, seed))
    log_lines = []

    def log(msg):
        log_lines.append(msg)
    t0 = time.time()
    try:
        profile = variant_profile(variant)
        plan = quick_plan() if spec.get('quick') else None
        session = procedures.run_session(case_dir, profile, seed, plan=plan, label='a18_%s_s%d' % (variant, seed),
                                         native_executable=spec.get('native'), log=log)
        t1 = time.time()
        result = analysis.analyse_session(case_dir, workers=int(spec.get('workers', 2)), log=log)
        t2 = time.time()
        rows = compare(case_dir, result)
        _write(case_dir / 'analysis.json', result)
        out = dict(schema=SCHEMA, variant=variant, seed=seed, overrides=VARIANTS[variant], rows=rows,
                   timing=dict(session_wall_s=round(t1 - t0, 1), analysis_wall_s=round(t2 - t1, 1),
                               sim_s=session['timing'].get('sim_s')),
                   log=log_lines, result_kind=RESULT_KIND, field_validated=False, error=None)
    except Exception as exc:                                   # one failed case is reported, the others go on
        out = dict(schema=SCHEMA, variant=variant, seed=seed, overrides=VARIANTS[variant], rows=[],
                   timing=dict(wall_s=round(time.time() - t0, 1)), log=log_lines, result_kind=RESULT_KIND,
                   field_validated=False, error='%s: %s' % (type(exc).__name__, exc),
                   traceback=traceback.format_exc(limit=8))
    case_dir.mkdir(parents=True, exist_ok=True)
    _write(case_dir / 'compare.json', out)
    return out


def _write(path, obj):
    data = (json.dumps(obj, ensure_ascii=False, indent=1, default=_default) + '\n').encode('utf-8')
    with open(path, 'wb') as fh:
        fh.write(data)


def _default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (tuple, set)):
        return list(o)
    raise TypeError(type(o).__name__)


def summarize(cases):
    """Per A18 quantity: cases, passed, worst error; failed rows listed."""
    groups = {}
    for case in cases:
        for r in case['rows']:
            if r['bound'] is None:
                continue
            key = r['quantity'].split(' ')[0]
            g = groups.setdefault(key, dict(rows=0, passed=0, worst_error=None, bound=r['bound'],
                                            error_kind=r['error_kind'], failed=[]))
            g['rows'] += 1
            if r['passed']:
                g['passed'] += 1
            elif r['passed'] is False or r['estimate'] is None:
                g['failed'].append(dict(case='%s_s%d' % (case['variant'], case['seed']), quantity=r['quantity'],
                                        estimate=r['estimate'], truth=r['truth'], error=r['error']))
            if r['error'] is not None and (g['worst_error'] is None or abs(r['error']) > abs(g['worst_error'])):
                g['worst_error'] = r['error']
    errors = [dict(case='%s_s%d' % (c['variant'], c['seed']), error=c['error']) for c in cases if c['error']]
    return dict(quantities=groups, case_errors=errors,
                all_passed=not errors and all(g['passed'] == g['rows'] for g in groups.values()))


def main(argv=None):
    ap = argparse.ArgumentParser(description='A18(F) identification recovery (placeholder-based SIM results)')
    ap.add_argument('--out', required=True, help='output folder outside the repository')
    ap.add_argument('--variants', default=','.join(VARIANTS))
    ap.add_argument('--seeds', default=','.join(str(s) for s in DEFAULT_SEEDS))
    ap.add_argument('--jobs', type=int, default=3, help='cases in parallel (one process each)')
    ap.add_argument('--workers', type=int, default=2, help='odometry workers inside a case')
    ap.add_argument('--quick', action='store_true', help='short plan (smoke run; not the A18 numbers)')
    args = ap.parse_args(argv)
    from .. import t1_rig as tr
    from ..scene_spec import inside_repository
    out = Path(args.out)
    if inside_repository(str(out)):
        raise SystemExit('the output folder %s lies inside the repository; use a runs folder' % out)
    out.mkdir(parents=True, exist_ok=True)
    native = tr.resolve_native()                     # build once here (the cases run in their own processes)
    specs = [dict(variant=v, seed=int(s), out=str(out), quick=args.quick, native=native, workers=args.workers)
             for v in args.variants.split(',') for s in args.seeds.split(',')]
    t0 = time.time()
    if args.jobs <= 1:
        cases = [run_case(s) for s in specs]
    else:
        with ProcessPoolExecutor(max_workers=args.jobs) as pool:
            cases = list(pool.map(run_case, specs))
    summary = summarize(cases)
    report = dict(schema=SCHEMA, bounds=BOUNDS, variants={k: VARIANTS[k] for k in args.variants.split(',')},
                  seeds=[int(s) for s in args.seeds.split(',')], quick=args.quick, wall_s=round(time.time() - t0, 1),
                  summary=summary, cases=[{k: c[k] for k in ('variant', 'seed', 'rows', 'timing', 'error')}
                                          for c in cases],
                  result_kind=RESULT_KIND, field_validated=False)
    _write(out / 'A18_RESULTS.json', report)
    for key, g in summary['quantities'].items():
        print('%-28s %d/%d  worst %s (bound %s %s)' % (key, g['passed'], g['rows'], g['worst_error'], g['bound'],
                                                     g['error_kind']))
    for e in summary['case_errors']:
        print('case error', e['case'], e['error'])
    print('all passed' if summary['all_passed'] else 'not all passed', '- placeholder-based SIM results')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
