"""Aggregate row_shadow_replay.py summaries into the C1 gate metrics (text table + JSON).

Distance between terminal HOLDs: each run contributes the reference (offline ICP) distance driven while the
controller was in FOLLOW/REDUCE until its first stop decision (event) or until the human stopped driving
(censored). Kaplan-Meier gives the median / p10 of that distance; exposure / events gives a rate-based mean.
'mid_straight' stops are those with >= 3 m of straight human driving left (a premature stop proxy: the human
kept going straight, so it was not a row end or a turn); 'at_turn_or_end' stops are counted as plausible.

Strict premature stops (premature(); the 2026-10-22 gate report's 'strict early stop', its script was not kept and
is rebuilt here from the report's definition): a mid_straight stop that came at least START_REFUSAL_M (1 m) after
FOLLOW (nearer stops are start refusals) and, when its reason is front_blocked, whose front clearance is also below
the stop model's required distance at STOP_V (0.45 m/s). Rate figures: exposure / count, its median distance
(x ln 2, exponential model), exp(-L / rate) for L = 30 / 45 / 60 m and the Kaplan-Meier median / p10 with the
strict premature stops as events. --premature-out writes them per variant, start kind and controller.

Usage: python -I -B row_gate_report.py --runs-dir <runs folder> --variants base n3 ... --out <metrics.json>
"""
import argparse
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np

KINDS = ('standing', 'flying')
STOP_V = .45          # weekend target speed for the front-stop re-check (open loop drove ~0.9 m/s)
START_REFUSAL_M = 1.  # a stop nearer than this after FOLLOW is a start refusal, not a premature stop


def kaplan_meier(samples):
    """samples: list of (distance, event bool). Returns (median, p10) distance or None when not reached."""
    samples = [(d, e) for d, e in samples if d is not None and math.isfinite(d)]
    if not samples:
        return None, None
    data = sorted(samples, key=lambda x: (x[0], not x[1]))
    at_risk = len(data)
    surv = 1.
    median = p10 = None
    i = 0
    while i < len(data):
        d = data[i][0]
        events = censored = 0
        while i < len(data) and data[i][0] == d:
            events += int(data[i][1])
            censored += int(not data[i][1])
            i += 1
        if events:
            surv *= 1 - events / at_risk
            if p10 is None and surv <= .9:
                p10 = d
            if median is None and surv <= .5:
                median = d
        at_risk -= events + censored
    return median, p10


def required_front(v, margin=.35, reaction=.15, onset=.25, decel=1.07):
    return v * (reaction + onset) + v * v / (2 * decel) + margin


def aggregate(summaries, kind, controller):
    runs = []
    rec_fwd = 0
    for s in summaries:
        rec_fwd += s['record_forward_ticks']
        for r in s['runs']:
            if r['kind'] != kind:
                continue
            c = next(c for c in r['controllers'] if c['controller'] == controller)
            runs.append((s['rid'], r, c))
    out = dict(runs=len(runs))
    if not runs:
        return out
    fails = [c['start_fail'] for _, _, c in runs if c['start_fail']]
    followed = [(rid, r, c) for rid, r, c in runs if c['followed']]
    out['start_fail'] = len(fails)
    out['start_fail_reasons'] = Counter('%s/%s' % (f['reason'], f['corridor_reason']) for f in fails).most_common(6)
    fwd = sum(r['forward_ticks'] for _, r, _ in runs)
    fwd_follow = sum(c['follow_forward_ticks'] for _, _, c in runs)
    out['human_forward_s'] = round(fwd * .05, 1)
    out['follow_fraction_of_run_forward'] = round(fwd_follow / fwd, 3) if fwd else None
    out['follow_fraction_of_record_forward'] = round(fwd_follow / rec_fwd, 3) if rec_fwd else None
    exposure = sum(c['exposure_m'] or 0. for _, _, c in followed)
    out['exposure_m'] = round(exposure, 1)
    stops = [c['stop'] for _, _, c in followed if c['stop']]
    out['stops'] = len(stops)
    out['stop_reasons'] = Counter(st['reason'] for st in stops).most_common()
    mid = [st for st in stops if not st['at_turn_or_end']]
    out['mid_straight_stops'] = len(mid)
    out['mid_straight_reasons'] = Counter(st['reason'] for st in mid).most_common()
    out['at_turn_or_end_reasons'] = Counter(st['reason'] for st in stops if st['at_turn_or_end']).most_common()
    out['m_per_stop'] = round(exposure / len(stops), 1) if stops else None
    out['m_per_mid_straight_stop'] = round(exposure / len(mid), 1) if mid else None
    any_samples = [((c['stop']['dist_from_follow_m'] if c['stop'] else c['exposure_m']) or 0., bool(c['stop']))
                   for _, _, c in followed]
    mid_samples = [((c['stop']['dist_from_follow_m'] if c['stop'] else c['exposure_m']) or 0.,
                    bool(c['stop'] and not c['stop']['at_turn_or_end'])) for _, _, c in followed]
    km_any = kaplan_meier(any_samples)
    km_mid = kaplan_meier(mid_samples)
    out['km_any_stop_median_p10_m'] = [None if v is None else round(v, 1) for v in km_any]
    out['km_mid_straight_median_p10_m'] = [None if v is None else round(v, 1) for v in km_mid]
    dists = [st['dist_from_follow_m'] for st in stops if st['dist_from_follow_m'] is not None]
    out['stop_distance_p10_p50_p90_m'] = [round(float(np.percentile(dists, p)), 1) for p in (10, 50, 90)] if dists else None
    out['longest_follow_m'] = round(max(s[0] for s in any_samples), 1)
    out['runs_followed_ge_30m'] = sum(1 for d, _ in any_samples if d >= 30)
    out['runs_followed_ge_10m'] = sum(1 for d, _ in any_samples if d >= 10)
    front = [st for st in stops if st['reason'] == 'front_blocked']
    out['front_blocked_also_at_0.45'] = sum(1 for st in front if st['front_clearance_m'] < required_front(STOP_V))
    rowend = [st for st in stops if st['reason'] == 'row_end_candidate']
    out['row_end_stops'] = len(rowend)
    out['row_end_mid_straight'] = sum(1 for st in rowend if not st['at_turn_or_end'])
    out['row_end_straight_ahead_m'] = sorted(round(st['straight_ahead_m'], 1) for st in rowend
                                             if st['straight_ahead_m'] is not None)
    posted = sum(c['row_end_posted_ticks'] for _, _, c in followed)
    early = sum(c['row_end_early_ticks'] for _, _, c in followed)
    ticks = sum(c['follow_ticks'] for _, _, c in followed)
    out['row_end_posted_fraction_of_follow'] = round(posted / ticks, 3) if ticks else None
    out['row_end_posted_early_ge3m_fraction'] = round(early / posted, 3) if posted else None
    corr = sum(c['correcting_ticks'] for _, _, c in followed)
    agree = sum(c['agree_ticks'] for _, _, c in followed)
    out['correcting_fraction'] = round(corr / ticks, 3) if ticks else None
    out['sign_agreement'] = round(agree / corr, 4) if corr else None
    sat = sum(c['saturated_ticks'] for _, _, c in followed)
    out['saturated_fraction'] = round(sat / ticks, 3) if ticks else None
    out['max_saturated_run_s'] = max((c['max_saturated_run_s'] for _, _, c in followed), default=0)
    lat = [c['lateral_abs_p50_p90_max'] for _, _, c in followed if c['lateral_abs_p50_p90_max']]
    out['lateral_abs_median_of_run_p50_m'] = round(float(np.median([v[0] for v in lat])), 3) if lat else None
    out['lateral_abs_max_m'] = round(max(v[2] for v in lat), 3) if lat else None
    out['side_inhibit'] = sum(c['side_inhibit'] for _, _, c in followed)
    stale = [st for st in stops if st['reason'] in ('stale_lidar', 'stale_imu')]
    out['stale_stops'] = len(stale)
    return out


def is_strict_premature(stop):
    """The gate report's strict early stop (module docstring)."""
    distance = stop.get('dist_from_follow_m')
    if stop['at_turn_or_end'] or distance is None or distance < START_REFUSAL_M:
        return False
    if stop['reason'] == 'front_blocked':
        return stop['front_clearance_m'] < required_front(STOP_V)
    return True


def premature(summaries, kind, controller):
    """Strict premature stop figures of one start kind and controller (module docstring)."""
    followed = []
    for s in summaries:
        for r in s['runs']:
            if r['kind'] != kind:
                continue
            c = next(c for c in r['controllers'] if c['controller'] == controller)
            if c['followed']:
                followed.append(c)
    exposure = sum(c['exposure_m'] or 0. for c in followed)
    stops = [c['stop'] for c in followed if c['stop']]
    refusals = [st for st in stops if st.get('dist_from_follow_m') is not None
                and st['dist_from_follow_m'] < START_REFUSAL_M]
    strict = [st for st in stops if is_strict_premature(st)]
    rate = exposure / len(strict) if strict else None
    samples = [((c['stop']['dist_from_follow_m'] if c['stop'] else c['exposure_m']) or 0.,
                bool(c['stop'] and is_strict_premature(c['stop']))) for c in followed]
    km = kaplan_meier(samples)
    return dict(exposure_m=round(exposure, 1), stops=len(stops), start_refusals_lt1m=len(refusals),
                start_refusal_reasons=[list(x) for x in Counter(st['reason'] for st in refusals).most_common()],
                premature_after_1m_at045=len(strict),
                premature_reasons=[list(x) for x in Counter(st['reason'] for st in strict).most_common()],
                m_per_premature=None if rate is None else round(rate, 1),
                rate_median_m=None if rate is None else round(rate * math.log(2.), 1),
                p_no_premature_30_45_60=None if rate is None else [round(math.exp(-d / rate), 2)
                                                                    for d in (30., 45., 60.)],
                km_median_p10=[None if v is None else round(v, 3) for v in km])


def timing(summaries, runs_dir, variant):
    scan = []
    for s in summaries:
        f = Path(runs_dir) / variant / ('scan_ms_%s.npy' % s['rid'][9:15])
        if f.exists():
            scan.append(np.load(f))
    out = {}
    if scan:
        v = np.concatenate(scan)
        out['on_scan_ms_p50_p99_max'] = [round(float(np.percentile(v, p)), 1) for p in (50, 99, 100)]
        out['on_scan_over_100ms'] = int((v > 100).sum())
        out['scans_timed'] = int(len(v))
    st = Counter()
    for s in summaries:
        for r in s['runs']:
            for k, val in r['stats'].items():
                st[k] += val
    out['scans_processed'] = st['scans_processed']
    out['scans_dropped'] = st['scans_dropped']
    out['stale_lidar_ticks'] = st['stale_lidar_ticks']
    out['stale_imu_ticks'] = st['stale_imu_ticks']
    out['ticks'] = st['ticks']
    out['tick_delay_ms_p50_p90_p99_max_per_record'] = {s['rid'][9:15]: s['tick_delay_ms_p50_p90_p99_max'] for s in summaries}
    out['workers'] = summaries[0].get('workers')
    return out


def load(runs_dir, variant):
    return [json.loads(p.read_text(encoding='utf-8')) for p in sorted((Path(runs_dir) / variant).glob('summary_*.json'))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs-dir', required=True)
    ap.add_argument('--variants', nargs='+', required=True)
    ap.add_argument('--controllers', nargs='*', default=['continuous/sim', 'continuous/weekend',
                                                           'continuous_side/sim', 'continuous_side/weekend'])
    ap.add_argument('--out')
    ap.add_argument('--premature-out', help='JSON file for the strict premature stop figures')
    ap.add_argument('--premature-controller', default='continuous_side/weekend')
    args = ap.parse_args()
    result = {}
    strict = {}
    for variant in args.variants:
        summaries = load(args.runs_dir, variant)
        if not summaries:
            continue
        v = dict(records=[s['rid'] for s in summaries], spec=summaries[0]['spec'],
                 segments=sum(s['segments'] for s in summaries),
                 eligible_segments=sum(s.get('eligible_segments', 0) for s in summaries),
                 eligible_standing=sum(s.get('eligible_standing', 0) for s in summaries),
                 eligible_m=round(sum(s.get('eligible_m', 0) for s in summaries), 1),
                 record_forward_s=round(sum(s['record_forward_ticks'] for s in summaries) * .05, 1),
                 masked_beams={s['rid'][9:15]: s['masked_beams'] for s in summaries},
                 timing=timing(summaries, args.runs_dir, variant), results={})
        for kind in KINDS:
            for ctl in args.controllers:
                v['results']['%s|%s' % (kind, ctl)] = aggregate(summaries, kind, ctl)
            strict['%s|%s' % (variant, kind)] = premature(summaries, kind, args.premature_controller)
        result[variant] = v
        print('=== %s  spec=%s' % (variant, json.dumps(v['spec'])))
        print('    segments %d eligible %d (standing %d) eligible_m %.0f record_forward_s %.0f timing %s' % (
            v['segments'], v['eligible_segments'], v['eligible_standing'], v['eligible_m'], v['record_forward_s'],
            json.dumps({k: v['timing'][k] for k in ('on_scan_ms_p50_p99_max', 'scans_dropped', 'stale_lidar_ticks',
                                                    'stale_imu_ticks') if k in v['timing']})))
        for key, a in v['results'].items():
            if not a.get('runs'):
                continue
            print('  %-34s runs %3d fail %3d follow%%run %s rec %s exp %6.1f m stops %3d mid %3d m/stop %s m/mid %s '
                  'KMany %s KMmid %s agree %s sat %s' % (
                      key, a['runs'], a['start_fail'], a['follow_fraction_of_run_forward'],
                      a['follow_fraction_of_record_forward'], a['exposure_m'], a['stops'], a['mid_straight_stops'],
                      a['m_per_stop'], a['m_per_mid_straight_stop'], a['km_any_stop_median_p10_m'],
                      a['km_mid_straight_median_p10_m'], a['sign_agreement'], a['saturated_fraction']))
            print('      reasons %s | mid %s | fail %s' % (a['stop_reasons'], a['mid_straight_reasons'],
                                                         a['start_fail_reasons'][:3]))
    if args.out:
        Path(args.out).write_bytes(json.dumps(result, indent=1).encode('utf-8'))
    if args.premature_out:
        Path(args.premature_out).write_bytes((json.dumps(strict, indent=1) + chr(10)).encode('utf-8'))


if __name__ == '__main__':
    main()
