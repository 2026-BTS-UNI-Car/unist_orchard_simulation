#!/usr/bin/env python3
"""PC-only validation: compare field_quick_calib results on the 10/10 and 10/3 records with drive-evidence.txt.

Inputs are calib_result.json files produced by field_quick_calib.py on the real records:
  --r1010ref  10/10 compact extracts with --reference-odometry (analysis layer alone, same ICP output as
              LIDAR_RESPONSE.json)
  --r1010     10/10 with the analyser's own ICP (odometry + analysis)
  --r1003     10/3 full extracts (reader + ICP + analysis); labels road/orchard
Targets and tolerances are fixed here, before looking at the result; each row says where the target comes
from. Output: a text table on stdout and --out JSON. Nothing is written into the record folders.
Front-steer branch: the rows of the differential C5 arc gain and C6 pivots of 546fa8c are not imported (those
steps are not part of this package); the drive rows (axis, C1, C2, C3, C4, C7) are unchanged. The inputs are
calib_result.json files of the drive analysis (drive_calib.py, same keys).
"""
import argparse
import json

import numpy as np


def load(path):
    if not path:
        return None
    with open(path, 'r', encoding='utf-8') as fh:
        return json.load(fh)


def g(d, *keys):
    for k in keys:
        if d is None:
            return None
        if isinstance(d, list):
            d = d[k] if -len(d) <= k < len(d) else None
        else:
            d = d.get(k)
    return d


class Table:
    def __init__(self):
        self.rows = []

    def add(self, group, item, target, tol, measured, source, note=''):
        ok = None
        if measured is not None and target is not None:
            if isinstance(target, (list, tuple)) and len(target) == 2 and isinstance(tol, str) and tol == 'range':
                ok = target[0] <= measured <= target[1]
            else:
                ok = abs(measured - target) <= tol
        self.rows.append(dict(group=group, item=item, target=target, tol=tol, measured=measured,
                              status='PASS' if ok else ('FAIL' if ok is False else 'N/A'), source=source, note=note))

    def text(self):
        out = ['%-6s %-58s %-16s %-8s %-12s %-5s %s' % ('group', 'item', 'target', 'tol', 'measured', 'stat', 'source')]
        for r in self.rows:
            out.append('%-6s %-58s %-16s %-8s %-12s %-5s %s%s' % (
                r['group'], r['item'][:58], _fmt(r['target']), _fmt(r['tol']), _fmt(r['measured']), r['status'],
                r['source'], '' if not r['note'] else '  [' + r['note'] + ']'))
        n = {s: sum(1 for r in self.rows if r['status'] == s) for s in ('PASS', 'FAIL', 'N/A')}
        out.append('PASS %d  FAIL %d  N/A %d' % (n['PASS'], n['FAIL'], n['N/A']))
        return '\n'.join(out)


def _fmt(x):
    if x is None:
        return 'n/a'
    if isinstance(x, float):
        return '%.3f' % x
    if isinstance(x, (list, tuple)):
        return '[' + ','.join(_fmt(v) for v in x) + ']'
    return str(x)


def check_1010(t, r, tag):
    grp = '1010' + tag
    if r is None:
        return
    t.add(grp, 'travel axis deg', -179.08, .5, g(r, 'axis', 'axis_deg'), 'LIDAR_RESPONSE travel_axis_lidar_frame')
    t.add(grp, 'LiDAR/IMU yaw corr', .994, .02, g(r, 'lidar_imu', 'corr'), 'LIDAR_RESPONSE lidar_vs_imu_yaw')
    t.add(grp, 'LiDAR/IMU yaw gain', .987, .03, g(r, 'lidar_imu', 'gain_median'), 'LIDAR_RESPONSE lidar_vs_imu_yaw')
    ro = g(r, 'c2', 'labels', 'all', 'ramp_onset')
    t.add(grp, 'C2 start: duty at onset p50 %', 34., 10., g(ro, 'duty_at_onset_pct_p10_p50_p90', 1),
          'start_onset_1010.json (drive-evidence 2.1)')
    t.add(grp, 'C2 start: precise onset events n', 11, 1, g(ro, 'n_precise'), 'start_onset_1010.json')
    t.add(grp, 'C2 start: ramp start -> motion p50 s', .34, .15, g(ro, 'onset_after_ramp_start_s_p10_p50_p90', 1),
          'start_onset_1010.json')
    for d, p50, p10, p90 in (('100', .899, .727, .999), ('90', .715, None, None)):
        row = g(r, 'c3', 'table', d)
        t.add(grp, 'C3 %s%% settled speed p50 m/s' % d, p50, .03, g(row, 'settled_all_p10_p50_p90', 1),
              'LIDAR_RESPONSE settled_response straight_fwd')
        if p10 is not None:
            t.add(grp, 'C3 %s%% settled speed p10 m/s' % d, p10, .05, g(row, 'settled_all_p10_p50_p90', 0), 'LIDAR_RESPONSE')
            t.add(grp, 'C3 %s%% settled speed p90 m/s' % d, p90, .05, g(row, 'settled_all_p10_p50_p90', 2), 'LIDAR_RESPONSE')
        t.add(grp, 'C3 %s%% steady (1 s dropped) p50 m/s' % d, p50, .05, g(row, 'v_p10_p50_p90', 1), 'LIDAR_RESPONSE',
              'drop rule differs')
    s = g(r, 'c4', 'strict', 'summary')
    t.add(grp, 'C4 straight runs >= 3 s (strict rule)', 92, 10, g(s, 'runs'), 'LIDAR_RESPONSE straight_runs')
    for i, (name, target) in enumerate((('p10', -1.33), ('p50', -.23), ('p90', .90))):
        t.add(grp, 'C4 heading drift deg/m %s' % name, target, .3, g(s, 'deg_per_m_p10_p50_p90', i), 'LIDAR_RESPONSE')
    t.add(grp, 'C4 offset from chord p50 m', .041, .02, g(s, 'max_offset_from_chord_m_p10_p50_p90', 1), 'LIDAR_RESPONSE')
    t.add(grp, 'C4 offset from chord p90 m', .103, .03, g(s, 'max_offset_from_chord_m_p10_p50_p90', 2), 'LIDAR_RESPONSE')
    c1 = g(r, 'c1')
    t.add(grp, 'C1 standstill yaw bias deg/s (BNO085 range)', [-.05, .01], 'range', g(c1, 'yaw_bias_dps'),
          'drive-evidence 2.7 (-0.009..-0.041)')
    t.add(grp, 'C1 standstill yaw std deg/s', .09, .05, g(c1, 'yaw_std_dps'), 'drive-evidence 2.7 (0.09)')
    # C7: clean fade stops from the fade start (from_release.py output, 4 events) and LIDAR_RESPONSE first-status rule
    clean = sorted([e for e in g(r, 'c7', 'detail') or [] if e.get('clean') and e.get('kind') == 'release'
                    and e.get('straight')], key=lambda e: e['distance_m'])
    ref_d = [.383, .559, .566, .595]
    ref_t = [.84, .89, .89, .90]
    t.add(grp, 'C7 clean release stops n', 4, 0, len(clean), 'from_release.out.txt (clean fade events)')
    for i in range(4):
        e = clean[i] if i < len(clean) else {}
        t.add(grp, 'C7 release stop #%d distance m' % (i + 1), ref_d[i], .05, e.get('distance_m'), 'from_release.out.txt')
        t.add(grp, 'C7 release stop #%d time s' % (i + 1), ref_t[i], .1, e.get('time_s'), 'from_release.out.txt')
    ff = [e['from_first_zero_status']['distance_m'] for e in g(r, 'c7', 'detail') or []
          if e.get('straight') and 'from_first_zero_status' in e]
    t.add(grp, 'C7 first-zero-status rule events (release+STOP)', 10, 2, len(ff), 'LIDAR_RESPONSE stops_after_forward')
    t.add(grp, 'C7 first-zero-status distance p50 m (pooled)', .342, .1,
          float(np.percentile(ff, 50)) if ff else None, 'LIDAR_RESPONSE (mixed 10 events)')
    t.add(grp, 'C7 first-zero-status distance p90 m (pooled)', .548, .1,
          float(np.percentile(ff, 90)) if ff else None, 'LIDAR_RESPONSE')


def check_1003(t, r):
    if r is None:
        return
    grp = '1003'
    # C3 vs the dense local SLAM speed of 072530 (segment medians, first 1 s dropped; keyed by reported duty)
    slam = {40: .35, 45: .445, 58: .552, 88: .822, 90: .857, 95: .89, 100: .882}
    for d, target in slam.items():
        row = g(r, 'c3', 'table', str(d)) or {}
        meds = [x['speed_m_s'] for x in row.get('segments', []) if x['rid'] == '20261003T072530Z_both']
        t.add(grp, 'C3 072530 %d%% v (segment p50) vs SLAM' % d, target, .05, float(np.median(meds)) if meds else None,
              'slam_by_duty_072530.json', 'n=%d' % len(meds))
    gnss = {45: .359, 55: .467, 70: .652}
    for d, target in gnss.items():
        row = g(r, 'c3', 'table', str(d)) or {}
        meds = [x['speed_m_s'] for x in row.get('segments', []) if x['rid'] in ('20261003T080909Z_both', '20261003T072530Z_both')]
        val = float(np.median(meds)) if meds else None
        t.add(grp, 'C3 %d%% LiDAR minus GNSS antenna speed m/s' % d, [-.05, .15], 'range',
              None if val is None else val - target, 'gnss_by_duty_1003.json',
              'GNSS ran 0.08 below LiDAR on 10/10; n=%d' % len(meds))
    # C4: the longest straight of 072530 (98 s, 85 m, SLAM -11.9 deg)
    runs = [x for x in g(r, 'c4', 'detail') or [] if x['rid'] == '20261003T072530Z_both']
    longest = max(runs, key=lambda x: x['length_m']) if runs else {}
    t.add(grp, 'C4 072530 longest straight heading change deg', -11.9, 3., longest.get('heading_change_lidar_deg'),
          'record accuracy check (SLAM -11.9 over 98 s / 85 m)', 'length %s m' % longest.get('length_m'))
    t.add(grp, 'C1 standstill yaw bias deg/s (BNO085 range)', [-.05, .01], 'range', g(r, 'c1', 'yaw_bias_dps'),
          'drive-evidence 2.7')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--r1010ref')
    ap.add_argument('--r1010')
    ap.add_argument('--r1003')
    ap.add_argument('--out')
    a = ap.parse_args()
    t = Table()
    check_1010(t, load(a.r1010ref), 'r')
    check_1010(t, load(a.r1010), 'i')
    check_1003(t, load(a.r1003))
    text = t.text()
    print(text)
    if a.out:
        with open(a.out, 'w', encoding='utf-8', newline='\n') as fh:
            json.dump(dict(schema='ubts.field_calib.validation.v1', rows=t.rows), fh, indent=1)
            fh.write('\n')


if __name__ == '__main__':
    main()
