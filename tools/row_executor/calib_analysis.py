"""Calibration runs (calib_runs.py) from the executor JSONL logs: reader, summary, config fragment, analyser adapter.

Standard library only, so it runs with the Jetson host python3 on the root-owned log directory (files 0644):
    python3 -B ~/row-executor/src/row_executor/calib_analysis.py \
        --log-dir $HOME/row-executor/logs --config $HOME/row-executor/config/config.json \
        [--out-fragment calib_fragment.json] [--json summary.json]
or on a copy of the logs on the PC. Without --runs every calibration run of the field day is summarised (newest last):
runs older than DAY_WINDOW_H (12 h) before the newest calibration run are left out unless --all-days
(review_safety_day2 N7: a run on another surface must not slip into the config).

Unified field procedure (2026-10-22, README_FIELD.txt 12): this module is the executor-log READER and summary. It no
longer writes a config (--out-config is refused): on the single-row branch the ONE config generator was
tools/field_calib/make_field_config.py (calibration and final stage), which read the runs through load_runs / field_day /
summarize / config_fragment / c5_segments / usable_stop here and applied the decision table and the steering GO rule of
config.py (field decision rules block) together with the recording measurements (C1, C2, C3 with the stopwatch, C4, C6,
C7 cross-check). Steering branch: make_field_config.py, recommend.py, run_field_calib.sh and steps.c5_arc of the single-row branch (546fa8c) were not imported to the steering branch (differential history; docs/steering/EXECUTOR_IMPORT_KO.md). The front_steer
calibration runs are refused (config.steer_rule_problems); the front_steer GO keys come from the steering
identification through tools/field_calib/steer_go_config.py (IDENT_KO.md).

What it reads (format: OPERATIONS.txt 10):
  event 'config'        one per executor process (separates run ids of different processes)
  event 'calib_start'   run_id, plan (calib_runs.CalibPlan.to_dict)
  tick  (k='tick') with 'calib': per 20 Hz tick of a calibration run; the tick's L/R (decision), sent, v (estimator
                        speed m/s), yaw (gyro yaw rate deg/s), dist, still + calib{k, i, n, ph, sL, sR, gyro, lidar,
                        lok, odom, hd, cd, et}
  event 'calib_step'    one finished step (the runtime's own statistics)
  event 'calib_result'  end_reason, results, outcome, plan
  event 'calib_stop'    (C7) the stop after the run's own 0/0: stop{done, ok, v_at_zero_m_s, stop_distance_m,
                        time_to_rest_s, why}, emergency (op stop instead of the fade), terminal_stop_wait_s
Malformed or empty lines are skipped (the writer can leave a half line after a disk error; review S8).

Outputs
  summary per run      the runtime step results, an independent recomputation from the ticks (cross-check), the
                       outcome (arc: per-side gain, yaw at 2 x max_differential; rolldown: rolling threshold candidate)
  config fragment      measured_yaw_at_max_diff_deg_s (arc, smaller side gain x 2 x max_differential of --config, or of
                       the run's plan; the operator-page value - the final authority is make_field_config's steering
                       rule over every usable arc run, never above this newest run), rolling_breakaway_duty_pct (rolldown),
                       measured_stop_distance_m (C7: the largest stop distance of >= STOP_MIN_RUNS 'stop' runs ended by
                       'calib_stop_test', rounded up to the cm). Each value comes from the NEWEST run of its kind; when that
                       run is not usable and an older one is taken, the note says so ('fallback'; make_field_config then
                       refuses to write, exit 4, unless --allow-older: review_safety_day2 N7).
  analyser adapter     c5_segments(run): the arc steps as tools/field_calib (field calibration branch of 2026-10-22;
                       steps.c5_arc is not in the steering branch's tools/field_calib) steps.c5_arc 'segments' rows (rid, label, req, diff, t_rel, seconds, samples, method,
                       yaw_lidar_dps, yaw_imu_dps, yaw_net_dps, yaw_imu_net_dps, speed_m_s, gain_dps_per_pct,
                       radius_m, side), so its least-squares gain (_ls_gain) can pool executor arcs with recording arcs.
"""
import argparse
import glob
import json
import math
import os
import sys

if __package__ in (None, ''):                      # run as a script: python3 calib_analysis.py ...
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from row_executor import calib_runs as CR          # noqa: E402  (stdlib only)
from row_executor import config as C               # noqa: E402

SCHEMA = 'ubts.row_executor.calib_summary.v1'
MEASURE_PHASES = dict(arc=('measure',), rolldown=('rolling', 'measure'), stop=('measure',))
DAY_WINDOW_H = 12.                                 # N7: calibration runs of the same field day
STOP_MIN_RUNS = 3                                  # C7: WEEKEND_PLAN 'base x3'
STOP_SPEED_TOLERANCE = .25                         # C7 run speed vs v_measured_m_s of --config (note only)


# --- reading ------------------------------------------------------------------------------------------------
def find_logs(log_dir):
    return sorted(glob.glob(os.path.join(log_dir, 'row_executor_*.jsonl')))


def iter_records(paths, counts=None):
    """(path, record) for every parseable JSON object line; malformed/empty lines are counted and skipped."""
    counts = {} if counts is None else counts
    counts.setdefault('lines', 0)
    counts.setdefault('malformed', 0)
    for path in paths:
        with open(path, 'r', encoding='utf-8', errors='replace', newline='') as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                counts['lines'] += 1
                try:
                    record = json.loads(line)
                except ValueError:
                    counts['malformed'] += 1
                    continue
                if isinstance(record, dict):
                    yield path, record
                else:
                    counts['malformed'] += 1


def extract_runs(records):
    """Calibration runs in log order: dicts with session, run_id, kind, plan, ticks, steps, result, config, file."""
    session, config, runs, order = 0, None, {}, []
    for path, rec in records:
        kind = rec.get('k')
        event = rec.get('event') if kind == 'event' else None
        if event == 'config':
            session += 1
            config = rec.get('config')
            continue
        if event == 'calib_start':
            plan = rec.get('plan') or {}
            key = (session, rec.get('run_id'))
            runs[key] = dict(session=session, run_id=rec.get('run_id'), kind=plan.get('kind'), plan=plan, ticks=[],
                             steps=[], result=None, config=config, file=os.path.basename(path), start_t=rec.get('t'),
                             start_wall=rec.get('wall'), stop=None)
            order.append(key)
        elif event in ('calib_step', 'calib_result', 'calib_stop'):
            run = runs.get((session, rec.get('run_id')))
            if run is not None:
                if event == 'calib_step':
                    run['steps'].append(rec.get('result') or {})
                elif event == 'calib_stop':
                    run['stop'] = dict(rec.get('stop') or {}, emergency=rec.get('emergency'),
                                       terminal_stop_wait_s=rec.get('terminal_stop_wait_s'))
                else:
                    run['result'] = rec
        elif kind == 'tick' and isinstance(rec.get('calib'), dict):
            run = runs.get((session, rec.get('run')))
            if run is not None:
                run['ticks'].append(dict(t=rec.get('t'), state=rec.get('state'), L=rec.get('L'), R=rec.get('R'),
                                         sent=rec.get('sent'), v=rec.get('v'), yaw=rec.get('yaw'),
                                         dist=rec.get('dist'), still=rec.get('still'), calib=rec['calib']))
    return [runs[key] for key in order]


def load_runs(paths, counts=None):
    return extract_runs(iter_records(paths, counts))


def field_day(runs, hours=DAY_WINDOW_H):
    """(runs of the newest field day, number left out): runs started more than `hours` before the newest calibration
    run (wall clock of its calib_start event) are left out (N7). Runs without a wall time are kept."""
    walls = [r['start_wall'] for r in runs if _finite(r.get('start_wall'))]
    if not walls:
        return list(runs), 0
    newest = max(walls)
    kept = [r for r in runs if not _finite(r.get('start_wall')) or r['start_wall'] >= newest - hours * 3600.]
    return kept, len(runs) - len(kept)


# --- recomputation from the ticks (independent of the runtime's step statistics) -------------------------------
def _finite(v):
    return type(v) in (int, float) and not isinstance(v, bool) and math.isfinite(v)


def _r(v, nd=3):
    return None if v is None or not _finite(v) else round(float(v), nd)


def recompute_steps(run):
    """Per step from the measurement-phase ticks: seconds, gyro yaw (heading change / time), LiDAR yaw (mean of
    the per-tick LiDAR rates, only when >= 80 % of the ticks had one), mean speed (odometry-valid ticks)."""
    phases = MEASURE_PHASES.get(run.get('kind'), ())
    groups = {}
    for tick in run['ticks']:
        c = tick['calib']
        if c.get('ph') in phases and tick.get('state') == 'ACTIVE':
            groups.setdefault(c.get('i'), []).append(tick)
    out = []
    for i in sorted(k for k in groups if isinstance(k, int)):
        ticks = groups[i]
        c0, c1 = ticks[0]['calib'], ticks[-1]['calib']
        seconds = (ticks[-1]['t'] - ticks[0]['t']) if _finite(ticks[-1]['t']) and _finite(ticks[0]['t']) else None
        gyro = None
        if seconds and _finite(c0.get('hd')) and _finite(c1.get('hd')):
            gyro = (c1['hd'] - c0['hd']) / seconds
        lid = [t['calib']['lidar'] for t in ticks if t['calib'].get('lok') and _finite(t['calib'].get('lidar'))]
        lidar = sum(lid) / len(lid) if len(lid) >= CR.LIDAR_OK_FRACTION * len(ticks) and lid else None
        speeds = [t['v'] for t in ticks if t['calib'].get('odom') and _finite(t.get('v'))]
        row = dict(i=i, L=c0.get('sL'), R=c0.get('sR'), ticks=len(ticks), seconds=_r(seconds, 2), gyro_yaw_dps=_r(gyro),
                   lidar_yaw_dps=_r(lidar), speed_m_s=_r(sum(speeds) / len(speeds) if speeds else None),
                   min_speed_m_s=_r(min(speeds) if speeds else None))
        if run.get('kind') == 'arc' and _finite(row['R']) and _finite(row['L']):
            row['diff'] = int(row['R']) - int(row['L'])
        out.append(row)
    return out


def _cross_check(steps, recomputed, tol_dps=.5, tol_m_s=.05):
    """Runtime step results vs the recomputation: the largest differences (diagnostic only)."""
    by_i = {r['i']: r for r in recomputed}
    worst = dict(gyro_dps=0., speed_m_s=0., compared=0)
    for s in steps:
        r = by_i.get(s.get('i'))
        if r is None:
            continue
        worst['compared'] += 1
        for key, name in (('gyro_yaw_dps', 'gyro_dps'), ('speed_m_s', 'speed_m_s')):
            if _finite(s.get(key)) and _finite(r.get(key)):
                worst[name] = max(worst[name], abs(s[key] - r[key]))
    worst['ok'] = worst['gyro_dps'] <= tol_dps and worst['speed_m_s'] <= tol_m_s
    return worst


def summarize(run, max_differential_pct=None):
    plan = run.get('plan') or {}
    result = run.get('result') or {}
    steps = result.get('results') or run['steps']
    recomputed = recompute_steps(run)
    out = dict(file=run.get('file'), session=run.get('session'), run_id=run.get('run_id'), kind=run.get('kind'),
               end_reason=result.get('end_reason'), complete=run.get('result') is not None, plan=plan, steps=steps,
               recomputed=recomputed, cross_check=_cross_check(steps, recomputed), ticks=len(run['ticks']))
    if run.get('kind') == 'arc':
        md = int(max_differential_pct if max_differential_pct is not None else plan.get('max_differential_pct', 0))
        out['outcome'] = CR.arc_outcome(steps, md)
        out['outcome']['max_differential_pct'] = md
    elif run.get('kind') == 'rolldown':
        out['outcome'] = CR.rolldown_outcome(steps, int(plan.get('floor_pct', CR.UNKNOWN_BREAKAWAY_FLOOR_PCT)))
    elif run.get('kind') == 'stop':
        out['outcome'] = CR.stop_outcome(steps)
    else:
        out['outcome'] = {}
    out['stop'] = run.get('stop')
    out['start_wall'] = run.get('start_wall')
    return out


def usable_stop(s):
    """None when the run's stop observation is a C7 measurement, else why not."""
    stop = s.get('stop') or {}
    if s.get('kind') != 'stop':
        return 'not a stop run'
    if s.get('end_reason') != 'calib_stop_test':
        return 'ended %s (no steady 0/0 test)' % s.get('end_reason')
    if not stop.get('done') or not stop.get('ok'):
        return 'stop not observed (%s)' % stop.get('why')
    if stop.get('emergency'):
        return 'op stop instead of the 0/0 fade'
    wait = stop.get('terminal_stop_wait_s')
    if _finite(wait) and _finite(stop.get('time_to_rest_s')) and stop['time_to_rest_s'] > wait:
        return 'rest after %.1f s: the op stop at %.1f s cut the fade short' % (stop['time_to_rest_s'], wait)
    return None


# --- config fragment -------------------------------------------------------------------------------------------
def _newest_usable(summaries, kind, usable):
    """(chosen summary or None, fallback: an older run was chosen because the newest of the kind is not usable)."""
    of_kind = [x for x in summaries if x['kind'] == kind]
    good = [x for x in of_kind if usable(x)]
    if not good:
        return None, False
    return good[-1], good[-1] is not of_kind[-1]


def _where(s):
    return 'run %s/%s session %s %s' % (s['file'], s['run_id'], s['session'], s['end_reason'])


def config_fragment(summaries):
    """Config keys measured by the newest usable arc / rolldown / stop runs, with the reasons
    (dict(fragment, notes, fallback)). fallback lists the keys taken from an older run (N7)."""
    fragment, notes, fallback = {}, [], []
    s, older = _newest_usable(summaries, 'arc', lambda x: x['outcome'].get('yaw_at_full_diff_dps') is not None)
    if s is not None:
        o = s['outcome']
        fragment['measured_yaw_at_max_diff_deg_s'] = round(float(o['yaw_at_full_diff_dps']), 2)
        notes.append('measured_yaw_at_max_diff_deg_s %.2f = smaller side gain %.4f x |L-R| %d (max_differential %d)%s, '
                     '%s' % (o['yaw_at_full_diff_dps'], o['gain_min'], o['full_diff_pct'], o['max_differential_pct'],
                             ' EXTRAPOLATED' if o.get('extrapolated') else '', _where(s)))
        if older:
            fallback.append('measured_yaw_at_max_diff_deg_s')
            notes.append('WARNING: the newest arc run is not usable; measured_yaw_at_max_diff_deg_s comes from the OLDER '
                         '%s (another surface?) -- make_field_config refuses it without --allow-older' % _where(s))
    elif any(x['kind'] == 'arc' for x in summaries):
        notes.append('arc: no run with a valid step on BOTH sides: measured_yaw_at_max_diff_deg_s not set')
    s, older = _newest_usable(summaries, 'rolldown', lambda x: x['outcome'].get('rolling_breakaway_duty_pct')
                              is not None)
    if s is not None:
        o = s['outcome']
        fragment['rolling_breakaway_duty_pct'] = int(o['rolling_breakaway_duty_pct'])
        notes.append('rolling_breakaway_duty_pct %d: %s (%s)' % (o['rolling_breakaway_duty_pct'], o['note'], _where(s)))
        if older:
            fallback.append('rolling_breakaway_duty_pct')
            notes.append('WARNING: the newest rolldown run is not usable; rolling_breakaway_duty_pct comes from the OLDER '
                         '%s -- make_field_config refuses it without --allow-older' % _where(s))
    elif any(x['kind'] == 'rolldown' for x in summaries):
        notes.append('rolldown: no step below the base completed: rolling_breakaway_duty_pct not set')
    stops = [x for x in summaries if x['kind'] == 'stop']
    good = [x for x in stops if usable_stop(x) is None]
    for x in stops:
        why = usable_stop(x)
        if why is not None:
            notes.append('stop %s not used: %s' % (_where(x), why))
    if len(good) >= STOP_MIN_RUNS:
        distances = [x['stop']['stop_distance_m'] for x in good]
        value = max(.05, math.ceil(max(distances) * 100. - 1e-9) / 100.)
        fragment['measured_stop_distance_m'] = value
        notes.append('measured_stop_distance_m %.2f = largest of %d executor 0/0 stops %s m at %s m/s' % (
            value, len(good), [round(d, 3) for d in distances], [x['stop'].get('v_at_zero_m_s') for x in good]))
        stop_speeds = [x['stop'].get('v_at_zero_m_s') for x in good]
        if good[-1] is not stops[-1]:
            fallback.append('measured_stop_distance_m')
            notes.append('WARNING: the newest stop run is not usable; measured_stop_distance_m uses older runs -- '
                         'make_field_config refuses it without --allow-older')
    elif stops:
        notes.append('stop: %d usable C7 runs (need %d): measured_stop_distance_m not set' % (len(good), STOP_MIN_RUNS))
    out = dict(fragment=fragment, notes=notes, fallback=fallback)
    if 'measured_stop_distance_m' in fragment:
        out['stop_speeds'] = stop_speeds
    return out


def fit_fragment(frag, base_cfg):
    """The config rules that a measured yaw authority implies for the base config: max_yaw_rate_deg_s <=
    yaw_authority_fraction x measured (config.feasibility). Lowers it (0.1 deg/s steps, never raises) and notes it."""
    fragment, notes = dict(frag['fragment']), list(frag['notes'])
    stop = fragment.get('measured_stop_distance_m')
    if stop is not None and base_cfg is not None and base_cfg.v_measured_m_s is not None:
        o = base_cfg.controller_overrides
        d = C.STOP_MODEL_DEFAULTS
        model = C.stopping_distance_model(base_cfg.v_measured_m_s, o.get('stop_decel_m_s2', d['stop_decel_m_s2']),
                                          o.get('reaction_s', d['reaction_s']), o.get('stop_onset_s', d['stop_onset_s']))
        if model + 1e-9 < stop:
            notes.append('measured stop %.2f m exceeds the stop model %.3f m at v_measured %.2f m/s: the config refuses '
                         'motor output (re-measure C3/C7, or a slower base)' % (stop, model, base_cfg.v_measured_m_s))
        speeds = [v for v in frag.get('stop_speeds', ()) if _finite(v)]
        for v in speeds:
            if abs(v - base_cfg.v_measured_m_s) > STOP_SPEED_TOLERANCE * base_cfg.v_measured_m_s:
                notes.append('stop run speed %.2f m/s differs from v_measured %.2f m/s by more than %d %% (another base '
                             'duty or surface?)' % (v, base_cfg.v_measured_m_s, round(STOP_SPEED_TOLERANCE * 100)))
                break
    yaw = fragment.get('measured_yaw_at_max_diff_deg_s')
    if yaw is not None and base_cfg is not None:
        limit = C.max_yaw_for_authority(yaw, base_cfg.yaw_authority_fraction)    # the shared decision-table rule
        if base_cfg.max_yaw_rate_deg_s > limit + 1e-9:
            fragment['max_yaw_rate_deg_s'] = limit
            notes.append('max_yaw_rate_deg_s %.2f -> %.1f (<= %.1f x measured %.2f, config rule)' % (
                base_cfg.max_yaw_rate_deg_s, limit, base_cfg.yaw_authority_fraction, yaw))
        need = C.motor_min_yaw_authority(base_cfg)
        if yaw < need - 1e-9:
            notes.append('measured yaw %.2f deg/s is below the row motor minimum %.2f (steering GO rule; '
                         'min_yaw_authority_deg_s %.2f): row output stays refused (differential authority too small at '
                         'max_differential_pct %d)' % (yaw, need, base_cfg.min_yaw_authority_deg_s,
                                                      base_cfg.max_differential_pct))
    rolling = fragment.get('rolling_breakaway_duty_pct')
    if rolling is not None and base_cfg is not None:
        if base_cfg.breakaway_duty_pct is None:
            notes.append('rolling_breakaway_duty_pct needs breakaway_duty_pct (C2) in the config to take effect')
        elif rolling > base_cfg.breakaway_duty_pct:
            notes.append('rolling %d > static breakaway %d: the config refuses it (re-measure C2 / C3b)' % (
                rolling, base_cfg.breakaway_duty_pct))
    return dict(fragment=fragment, notes=notes, fallback=list(frag.get('fallback', ())))


def merge_config(base_cfg, fragment):
    """(validated config, None) or (None, problems): the base config with the fragment's keys."""
    data = base_cfg.to_dict()
    data.update(fragment)
    try:
        return C.from_dict(data), None
    except C.ConfigError as exc:
        return None, exc.problems


# --- adapter for tools/field_calib (other branch) --------------------------------------------------------------
def c5_segments(run, label='executor'):
    """Valid arc steps as field_calib steps.c5_arc segment rows (same keys and units)."""
    if run.get('kind') != 'arc':
        return []
    result = run.get('result') or {}
    steps = result.get('results') or run['steps']
    rows = []
    for s in steps:
        if not s.get('valid'):
            continue
        lidar, gyro, speed = s.get('lidar_yaw_dps'), s.get('gyro_yaw_dps'), s.get('speed_m_s')
        y = lidar if lidar is not None else gyro
        diff = int(s['R']) - int(s['L'])
        rows.append(dict(rid='executor:%s:%s' % (run.get('file'), run.get('run_id')), label=label,
                         req=[int(s['L']), int(s['R'])], diff=diff, t_rel=None,
                         seconds=s.get('seconds'), samples=s.get('scans'),
                         method='executor_lidar' if lidar is not None else 'imu', yaw_lidar_dps=lidar, yaw_imu_dps=gyro,
                         yaw_net_dps=y, yaw_imu_net_dps=gyro, speed_m_s=speed,
                         gain_dps_per_pct=_r(y / diff, 4) if y is not None and diff else None,
                         radius_m=_r(speed / math.radians(abs(y)), 1) if speed and y and abs(y) > .05 else None,
                         side='left' if diff > 0 else 'right'))
    return rows


# --- text report -----------------------------------------------------------------------------------------------
def _f(v, nd=2):
    return '-' if v is None else ('%.*f' % (nd, v) if isinstance(v, float) else str(v))


def report_lines(summaries, frag):
    lines = []
    for s in summaries:
        p = s['plan']
        lines.append('== %s run %s (%s) %s · 끝: %s · 최소 duty %s (%s) · tick %d' % (
            s['file'], s['run_id'], 'session %s' % s['session'], s['kind'], s['end_reason'], p.get('floor_pct'),
            p.get('floor_source'), s['ticks']))
        if s['kind'] == 'arc':
            lines.append('   #  좌/우    R-L   s     gyro°/s  LiDAR°/s  m/s    gain(°/s/%)  유효')
            for r in s['steps']:
                lines.append('   %-2d %3s/%-3s %+4d %5s  %7s  %8s  %5s  %10s  %s' % (
                    r['i'] + 1, r['L'], r['R'], r.get('diff', 0), _f(r.get('seconds'), 1), _f(r.get('gyro_yaw_dps')),
                    _f(r.get('lidar_yaw_dps')), _f(r.get('speed_m_s')), _f(r.get('gain_dps_per_pct'), 4),
                    'O' if r.get('valid') else 'X'))
            o = s['outcome']
            sides = o.get('sides', {})
            lines.append('   gain 좌 %s / 우 %s (LiDAR 우선, 없으면 gyro) · |L-R| %s에서 yaw %s °/s%s' % (
                _f(sides.get('left', {}).get('gain'), 4), _f(sides.get('right', {}).get('gain'), 4),
                o.get('full_diff_pct'), _f(o.get('yaw_at_full_diff_dps')), ' (외삽)' if o.get('extrapolated') else ''))
        else:
            good = 'steady' if s['kind'] == 'stop' else 'sustained'
            lines.append('   #  duty  s     m/s    최저 m/s  결과')
            for r in s['steps']:
                lines.append('   %-2d %4s  %5s  %5s  %8s  %s' % (
                    r['i'] + 1, r.get('duty'), _f(r.get('seconds'), 1), _f(r.get('speed_m_s')),
                    _f(r.get('min_speed_m_s')), '정지' if r.get('stalled') else ('유지' if r.get(good) else '미완')))
            lines.append('   ' + s['outcome'].get('note', ''))
        stop = s.get('stop')
        if stop:
            if stop.get('ok'):
                lines.append('   0/0 뒤 정지: %s m (0/0 때 %s m/s, 정지까지 %s s)%s' % (
                    _f(stop.get('stop_distance_m')), _f(stop.get('v_at_zero_m_s')), _f(stop.get('time_to_rest_s'), 1),
                    ' · op stop(즉시 정지)' if stop.get('emergency') else ''))
            else:
                lines.append('   0/0 뒤 정지: 측정 안 됨 (%s)' % stop.get('why'))
        cc = s['cross_check']
        lines.append('   tick 재계산 대조: %d단계, 최대 차 gyro %.2f °/s, 속도 %.3f m/s %s' % (
            cc['compared'], cc['gyro_dps'], cc['speed_m_s'], 'OK' if cc['ok'] else '(차이 큼: 기록 확인)'))
    lines.append('config 조각: ' + (json.dumps(frag['fragment'], sort_keys=True) if frag['fragment'] else '(없음)'))
    lines += ['  - ' + n for n in frag['notes']]
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description='Calibration runs (arc / rolldown) from the row executor JSONL logs')
    ap.add_argument('files', nargs='*', help='JSONL files (default: every row_executor_*.jsonl in --log-dir)')
    ap.add_argument('--log-dir', default=os.path.expanduser('~/row-executor/logs'))
    ap.add_argument('--config', help='executor config.json: its max_differential_pct is used for the arc result and '
                                     'the fragment is validated against it')
    ap.add_argument('--runs', type=int, nargs='*', help='only these run ids (newest session)')
    ap.add_argument('--all-days', action='store_true', help='also runs older than %g h before the newest one (N7)'
                    % DAY_WINDOW_H)
    ap.add_argument('--out-fragment')
    ap.add_argument('--out-config', help='refused: the config generator was tools/field_calib/make_field_config.py of '
                                         'the single-row branch (not imported here)')
    ap.add_argument('--json', help='write the full summary JSON here')
    args = ap.parse_args(argv)
    if args.out_config:
        print('--out-config is refused: the ONE config generator of the unified field procedure was '
              'tools/field_calib/make_field_config.py of the single-row branch (decision table, steering GO rule, C7; '
              'README_FIELD.txt 12); it and its wrapper run_field_calib.sh were not imported to the steering branch. '
              'front_steer GO keys: tools/field_calib/steer_go_config.py', file=sys.stderr)
        return 2
    paths = args.files or find_logs(args.log_dir)
    if not paths:
        print('no logs found (%s)' % args.log_dir, file=sys.stderr)
        return 2
    base_cfg = None
    if args.config:
        try:
            with open(args.config, 'r', encoding='utf-8') as fh:
                base_cfg = C.loads(fh.read())
        except (OSError, C.ConfigError) as exc:
            print('config: %s' % exc, file=sys.stderr)
            return 2
    counts = {}
    runs = load_runs(paths, counts)
    dropped = 0
    if not args.all_days:
        runs, dropped = field_day(runs)
    if args.runs:
        newest = max((r['session'] for r in runs), default=0)
        runs = [r for r in runs if r['session'] == newest and r['run_id'] in args.runs]
    md = None if base_cfg is None else base_cfg.max_differential_pct
    summaries = [summarize(r, md) for r in runs]
    frag = fit_fragment(config_fragment(summaries), base_cfg)
    for line in report_lines(summaries, frag):
        print(line)
    print('logs %d, lines %d, malformed %d, calibration runs %d' % (len(paths), counts.get('lines', 0),
                                                                  counts.get('malformed', 0), len(runs)))
    if dropped:
        print('left out %d calibration runs older than %g h before the newest one (another day; --all-days to include)'
              % (dropped, DAY_WINDOW_H))
    rc = 0
    if args.out_fragment:
        _write_json(args.out_fragment, frag['fragment'])
    if frag['fallback']:
        print('WARNING: %s would come from an OLDER run (the newest run of that kind is not usable): the single-row '
              'make_field_config refused it without --allow-older (N7)' % ', '.join(frag['fallback']))
    if args.json:
        _write_json(args.json, dict(schema=SCHEMA, summaries=summaries, fragment=frag, counts=counts,
                                    c5_segments=[row for r in runs for row in c5_segments(r)]))
    return rc


def _write_json(path, data):
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(json.dumps(data, indent=1, sort_keys=True, allow_nan=False) + '\n')


if __name__ == '__main__':
    sys.exit(main())
