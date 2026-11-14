"""Online scale checks of PW-10 on the T1 rig (F-SIM, executor in the loop; docs/steering/ACTUATOR_PRIOR_KO.md 4,
pre-registration docs/steering/PREREG_PW10_ONLINE_SCALE.json = prereg() of this module).

The executor runs with its default configuration (front_steer: the weight-0 actuator prior monitors the online scale,
PW-10), the row-following controller and one-press start of the wide evaluation (follow_wide.spec of the condition
'nominal'), with the plant's drive table = the field tuning file's table x FACTOR (follow_wide's drive_table mechanism,
the PW-08 / PC-11 'table_x085' condition). The screen value is the executor status block 'online_scale' (what the
dashboard relays unchanged, executor_proxy.STATUS_FIELDS), read every STATUS_EVERY_S of SIM time; the time series is
the 'prior' block of every executor tick (k, mon, fz, hold).

Runs per seed:
  exact   the plant drives at exactly FACTOR x the table speed in steady state: profile v0.1.2 with the speed fluctuation
          and the steering scrub loss set to 0 (EXACT_OVERRIDES; the profile's per-run fluctuation of 5 % / 2 s moves the
          true ratio of a 20 s window by several percent, see 'wide'). Until the row end.
  cut     exact + a LiDAR blackout of CUT_S at CUT_AT_S (rig fault sensor_blackout): the executor pauses on stale_lidar
          (no new go: it never restarts) and the scale must not change from the cut on.
  wide    the follow_wide condition table_x085 as registered (the default plant with its speed fluctuation). Until the
          row end. The scale is compared with the SIM truth ratio of the same window (evaluation only).
Judged (CRITERIA, registered before the first run):
  exact.converged   every screen value read with a full window (window_s >= target_s - FULL_WINDOW_MARGIN_S) until the
                    run end, and the value of the offer after the run, are within FACTOR +- tolerance; at least
                    MIN_JUDGED_SAMPLES such values
  exact.series      the run log has the time series: a 'prior' block with k on every executor tick after the first
                    value until the end (k None only before monitor_min_s of driving)
  cut.frozen        every executor tick value k from CUT_AT_S + SETTLE_S to the end equals the value at CUT_AT_S +
                    SETTLE_S (not None); every screen read from CUT_AT_S + FROZEN_FLAG_S on reports frozen; a tick inside
                    the blackout reports hold 'lidar_stale'
  wide.tracking     |screen value - SIM truth ratio of the same trailing window| <= track_tolerance for every read with
                    a full window (truth ratio = sum(truth speed dt) / sum(table speed of the MCU duty x prior scale dt)
                    over the trace samples with table speed >= monitor_min_speed_m_s, the last window_s of them)
Reported, not judged: wide's last value and offer against FACTOR (the plant ratio itself wanders), the offer's proposal,
the end reasons.

Every number is a placeholder-based SIM result (자리값 기반 SIM 결과), not field performance.

CLI (from sim/ubts):
  python -B -m steering.safety_scripts.online_scale_checks --out <folder outside the repository> [--seeds 100 101 ...]
         [--runs exact cut wide] [--workers N (<= 2)]
  python -B -m steering.safety_scripts.online_scale_checks --prereg          (print the pre-registration JSON)
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
from . import follow_wide as FW

SCHEMA = 'ubts.steer_sim.online_scale_checks.v1'
PREREG_SCHEMA = 'ubts.steer_sim.prereg_online_scale.v1'
PREREG_ID = 'pw10_online_scale_v1'
SEEDS = (100, 101, 102, 103, 104)
RUNS = ('exact', 'cut', 'wide')
FACTOR = .85
EXACT_OVERRIDES = {'speed_fluctuation_sd': 0., 'scrub_speed_loss_per_deg': 0.}
CUT_AT_S = 30.                       # SIM time of the LiDAR blackout (the 20 s window is full from about 22 s)
CUT_S = 1.
CUT_DURATION_S = 60.                 # the paused run is watched to here
STATUS_EVERY_S = .5                  # the screen reads the status at 2 Hz (dashboard auto panel)
FULL_WINDOW_MARGIN_S = .2
SETTLE_S = .3                        # the last scan before the cut is processed after it (latency + one tick)
FROZEN_FLAG_S = .6                   # actuator_prior.FROZEN_AFTER_S (0.5 s from the last entry) + scan period
MIN_JUDGED_SAMPLES = 10
MIN_SPEED_M_S = .1                   # actuator_prior monitor_min_speed_m_s (the truth window uses the same rule)
# Registered criteria (pre-registration document 3). Do not change after a run.
CRITERIA = dict(target=FACTOR, tolerance=.03, track_tolerance=.03, min_judged_samples=MIN_JUDGED_SAMPLES,
                full_window_margin_s=FULL_WINDOW_MARGIN_S, settle_s=SETTLE_S, frozen_flag_s=FROZEN_FLAG_S,
                series_min_fraction=1.)


def condition(run):
    name = 'table_x085' if run == 'wide' else 'nominal'
    c = dict(next(x for x in FW.CONDITIONS if x['name'] == name))
    c['drive_table'] = dict(factor=FACTOR)
    if run == 'cut':
        c['faults'] = [CAT.fault('sensor_blackout', at_s=CUT_AT_S, duration_s=CUT_S, which='lidar')]
    return c


def episode(run, seed, duration_s=None):
    """The episode spec of one run (module docstring); duration_s: a shorter watch (the pytest)."""
    if run not in RUNS:
        raise ValueError('run must be one of %s' % (RUNS,))
    c = condition(run)
    spec = FW.spec(c, seed, name='pw10_%s_s%d' % (run, seed))
    if run in ('exact', 'cut'):
        spec['rig']['profile'] = spec['rig']['profile'].with_overrides(EXACT_OVERRIDES, label='pw10_exact_drive_table',
                                                                       allow_out_of_range=True)
    if run == 'cut':
        spec['duration_s'] = CUT_DURATION_S
    if duration_s is not None:
        spec['duration_s'] = float(duration_s)
    return spec


def answer(facade, params):
    """POST /api/params of the operator page through the loop (one SIM tick later); the reply dict."""
    future = facade.executor.submit('params', params=params)
    facade.run(facade.rig.now + .1)
    return future.result(timeout=0)


def run_one(run, seed, duration_s=None, out_dir=None, keep=False):
    """One run: (result row, facade or None). The facade (keep=True) must be closed by the caller (facade.rig.close())."""
    A._sim_in_loop()                                  # puts tools/ on sys.path (row_executor)
    from row_executor import sim_in_loop_steer as S
    spec = S.normalise_spec(episode(run, seed, duration_s))
    started = time.perf_counter()
    rig = S.build_rig(spec)
    facade = None
    try:
        cfg = S.steer_executor_config(rig, spec['config'])
        facade = S.SteerExecutorInTheLoop(rig, cfg, spec['controller'], spec['actions'], spec['auto_repress'],
                                          spec['capture_inputs'], operator=spec['operator'])
        reads = []
        t = 0.
        while True:
            t = round(t + STATUS_EVERY_S, 6)
            before = rig.now
            facade.run(min(t, spec['duration_s']), spec['post_end_s'])
            if rig.now <= before + 1e-9 and t > STATUS_EVERY_S:
                break
            status = facade.executor.status()
            reads.append(dict(t=round(rig.now, 3), run_state=status.get('run_state'),
                              scale=json.loads(json.dumps(status.get('online_scale')))))
            if t >= spec['duration_s'] - 1e-9:
                break
        row = evaluate(run, seed, facade, reads)
        row['wall_s'] = round(time.perf_counter() - started, 1)
        if out_dir is not None:
            os.makedirs(out_dir, exist_ok=True)
            with open(os.path.join(out_dir, 'SERIES_%s_s%d.json' % (run, seed)), 'wb') as fh:
                fh.write(json.dumps(series(facade, reads), allow_nan=False).encode('utf-8'))
    finally:
        if not keep:
            rig.close()
    return (row, facade) if keep else (row, None)


def ticks(facade):
    """[(t, state, k, window s, frozen, hold, LiDAR speed)] of the executor ticks with a 'prior' block."""
    out = []
    for r in facade.log.records:
        if r.get('k') == 'tick' and isinstance(r.get('prior'), dict):
            p = r['prior']
            out.append((r['t'], r.get('state'), p.get('k'), p.get('mon'), p.get('fz'), p.get('hold'), r.get('v')))
    return out


def series(facade, reads):
    """The time series of a run for the figure (screen reads and executor ticks)."""
    return dict(schema=SCHEMA + '.series', result_kind='placeholder-based SIM result', reads=reads,
                ticks=[list(t) for t in ticks(facade)], events=[list(e) for e in facade.events])


def truth_ratio_fn(facade):
    """t_end, window -> SIM truth ratio (truth speed / table speed of the MCU duty x the prior scale) over the last
    `window` seconds of the trace samples whose table speed is >= MIN_SPEED_M_S (evaluation only)."""
    from steering import steered_plant as SP
    a = facade.trace_arrays()
    t = np.asarray(a['t_s'], float)
    v = np.asarray(a['truth_v_m_s'], float)
    duty = .5 * (np.asarray(a['truth_mcu_drive_l'], float) + np.asarray(a['truth_mcu_drive_r'], float)) * 100. / 255.
    cfg = facade.executor.config
    spec = SP.drive_table_spec(cfg.prior_speed_table)
    table = np.array([SP.drive_table_speed(spec, abs(d)) for d in duty]) * float(cfg.prior_speed_scale)
    dt = np.diff(t, prepend=t[0])
    drive = table >= MIN_SPEED_M_S

    def ratio(t_end, window):
        idx = np.nonzero(drive & (t <= t_end + 1e-9))[0][::-1]
        if not len(idx):
            return None
        sel = idx[np.cumsum(dt[idx]) <= window + 1e-9]
        den = float(np.sum(table[sel] * dt[sel]))
        return None if den <= 0. else float(np.sum(v[sel] * dt[sel]) / den)
    return ratio


def _full(scale):
    return (scale and scale.get('value') is not None and scale.get('window_s') is not None
            and scale.get('target_s') is not None and scale['window_s'] >= scale['target_s'] - FULL_WINDOW_MARGIN_S)


def evaluate(run, seed, facade, reads):
    c = CRITERIA
    summary = facade.summary()
    tk = ticks(facade)
    ended = facade.ended_at
    offer = None
    for r in reads:
        if r['scale'] and r['scale'].get('offer'):
            offer = r['scale']['offer']
    row = dict(run=run, seed=seed, end_reason=summary['end_reason'], ended_at_s=summary['ended_at_s'],
               reads=len(reads), ticks_with_prior=len(tk), offer=offer, judged={})
    full = [r for r in reads if _full(r['scale']) and (ended is None or r['t'] <= ended + 1e-9)]
    values = [r['scale']['value'] for r in full]
    row['full_window_reads'] = len(values)
    row['values'] = dict(first_full_t=full[0]['t'] if full else None, min=min(values) if values else None,
                         max=max(values) if values else None, last=values[-1] if values else None)
    lo, hi = c['target'] - c['tolerance'], c['target'] + c['tolerance']
    if run == 'exact':
        offer_ok = offer is not None and offer.get('value') is not None and lo <= offer['value'] <= hi
        row['judged']['converged'] = bool(len(values) >= c['min_judged_samples'] and all(lo <= k <= hi for k in values)
                                          and offer_ok)
        first = next((t for t, _, k, *_ in tk if k is not None), None)
        after = [k for t, _, k, *_ in tk if first is not None and t >= first and (ended is None or t <= ended + 1e-9)]
        row['series'] = dict(first_value_t=first, ticks=len(after), with_k=sum(1 for k in after if k is not None))
        row['judged']['series'] = bool(after and sum(1 for k in after if k is not None) >= c['series_min_fraction'] * len(after))
    if run == 'cut':
        settle = CUT_AT_S + c['settle_s']
        k0 = next((k for t, _, k, *_ in tk if t >= settle - 1e-9), None)
        later = [(t, k, fz, hold) for t, _, k, _, fz, hold, _ in tk if t >= settle - 1e-9]
        flag_reads = [r for r in reads if r['t'] >= CUT_AT_S + c['frozen_flag_s'] - 1e-9]
        stale_tick = any(fz is True and hold == 'lidar_stale' for t, _, _, _, fz, hold, _ in tk
                         if CUT_AT_S <= t <= CUT_AT_S + CUT_S + .1)
        before_cut = [k for t, _, k, *_ in tk if t < CUT_AT_S and k is not None]
        row['cut'] = dict(k_settled=k0, k_before_cut=before_cut[-1] if before_cut else None,
                          changed_ticks=sum(1 for _, k, _, _ in later if k != k0),
                          unfrozen_reads=sum(1 for r in flag_reads if not (r['scale'] or {}).get('frozen')),
                          value_changed_reads=sum(1 for r in flag_reads if (r['scale'] or {}).get('value') != k0),
                          holds=sorted({h for _, _, _, h in later if h}), lidar_stale_tick=stale_tick)
        row['judged']['frozen'] = bool(k0 is not None and later and row['cut']['changed_ticks'] == 0 and flag_reads
                                       and row['cut']['unfrozen_reads'] == 0 and row['cut']['value_changed_reads'] == 0
                                       and stale_tick)
    if run == 'wide':
        ratio = truth_ratio_fn(facade)
        errors = []
        for r in full:
            truth = ratio(r['t'], r['scale']['window_s'])
            errors.append(None if truth is None else abs(r['scale']['value'] - truth))
        good = [e for e in errors if e is not None]
        row['tracking'] = dict(reads=len(errors), max_abs_error=max(good) if good else None,
                               p50_abs_error=float(np.median(good)) if good else None, missing_truth=errors.count(None))
        row['judged']['tracking'] = bool(len(good) >= c['min_judged_samples'] and errors.count(None) == 0
                                         and max(good) <= c['track_tolerance'])
        row['report'] = dict(last_minus_target=None if not values else values[-1] - c['target'],
                             offer_minus_target=None if not offer or offer.get('value') is None
                             else offer['value'] - c['target'])
    row['passed'] = all(row['judged'].values())
    return row


def _job(item):
    run, seed, out = item
    return run_one(run, seed, out_dir=out)[0]


def prereg():
    return dict(schema=PREREG_SCHEMA, id=PREREG_ID, result_kind='placeholder-based SIM result', task='PW-10',
                simulator='F-SIM T1 rig, executor in the loop (tools/row_executor/sim_in_loop_steer.py)',
                executor='default front_steer configuration (prior_enabled None = weight-0 monitor), field tuning '
                         'package default, follow_wide controller and one-press start',
                plant=dict(drive_table_factor=FACTOR, exact_overrides=EXACT_OVERRIDES,
                           profile=FW.ROUND2_PROFILE), seeds=list(SEEDS), runs=list(RUNS),
                cut=dict(at_s=CUT_AT_S, duration_s=CUT_S, watch_to_s=CUT_DURATION_S),
                screen=dict(source='executor status online_scale (relayed unchanged by the dashboard)',
                            read_every_s=STATUS_EVERY_S),
                criteria=dict(CRITERIA), judged=['exact.converged', 'exact.series', 'cut.frozen', 'wide.tracking'],
                reported=['wide last value and offer minus target', 'offer proposals', 'end reasons'],
                pass_rule='every judged item passes on every seed')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out')
    ap.add_argument('--seeds', type=int, nargs='*', default=list(SEEDS))
    ap.add_argument('--runs', nargs='*', default=list(RUNS))
    ap.add_argument('--workers', type=int, default=1)
    ap.add_argument('--prereg', action='store_true')
    args = ap.parse_args(argv)
    if args.prereg:
        print(json.dumps(prereg(), indent=1, sort_keys=True))
        return 0
    if not args.out:
        ap.error('--out is required')
    out = os.path.abspath(args.out)
    from steering.scene_spec import inside_repository
    if inside_repository(out):
        raise SystemExit('refused: %s lies inside the repository (runs live outside)' % out)
    if not 1 <= args.workers <= 2:
        ap.error('--workers must be 1 or 2')
    os.makedirs(out, exist_ok=True)
    items = [(run, seed, out) for seed in args.seeds for run in args.runs]
    started = time.time()
    if args.workers == 1:
        rows = [_job(item) for item in items]
    else:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            rows = list(pool.map(_job, items))
    result = dict(schema=SCHEMA, result_kind='placeholder-based SIM result', field_validated=False, prereg=prereg(),
                  rows=rows, passed=all(r['passed'] for r in rows), wall_s=round(time.time() - started, 1))
    with open(os.path.join(out, 'ONLINE_SCALE_CHECKS.json'), 'wb') as fh:
        fh.write(json.dumps(result, indent=1, allow_nan=False).encode('utf-8'))
    for r in rows:
        print('%-5s s%-4d passed %s judged %s end %s values %s' % (r['run'], r['seed'], r['passed'], r['judged'],
                                                                    r['end_reason'], r['values']), flush=True)
    print('passed', result['passed'])
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
