"""Runs PW-01 .. PW-05 on the recordings and returns one result dict (no file output here)."""
import datetime as dt
import math
import os
import re

import numpy as np

import common
import ground
import load
import motion as mo
import session
import speed as sp
import startstop as ss

DAYS = ('1003', '1010')
KST = dt.timezone(dt.timedelta(hours=9))
REFERENCE = dict(
    speed_1010_100=0.899, speed_1010_90=0.715, speed_tol=0.01,
    start_events_1010=11, start_duty_p50=34., start_onset_p50=0.34,
    stop_clean_1010=4, stop_clean_dist=[0.383, 0.559, 0.566, 0.595], stop_clean_time=[0.84, 0.89, 0.89, 0.90],
    axis_1010=-179.08)


def _inventory_expectations(cfg):
    """Motor row count and span per recording from the 2026-11-11 inventory probe output (if present)."""
    path = cfg.path_of('inventory_probe_output')
    out = {}
    if not os.path.isfile(path):
        return out
    pat = re.compile(r'^(\d{8}T\d{6}Z_both) \| motor rows (\d+) \| span ([0-9.]+) s')
    with open(path, 'r', encoding='utf-8') as fh:
        for line in fh:
            m = pat.match(line.strip())
            if m:
                out[m.group(1)] = dict(rows=int(m.group(2)), span=float(m.group(3)))
    return out


def run(cfg, log=print):
    R = dict(config=cfg.raw, reference=REFERENCE)
    recs, listing = load.open_all(cfg, log=log)
    R['grounds'] = listing['grounds']
    by_rid = {r.rid: r for r in recs}
    # --- motion per day, travel axis ------------------------------------------------------------
    mots = {}
    R['axis'] = {}
    for day in DAYS:
        ms = [mo.Motion(r) for r in recs if r.day == day and r.lidar is not None]
        ax, n, dev = mo.travel_axis(ms)
        for m in ms:
            m.set_axis(ax)
            mots[m.rec.rid] = m
        R['axis'][day] = dict(axis_deg=math.degrees(ax), samples=n, deviation_p10_p50_p90=dev)
    # --- PW-02 reading check ---------------------------------------------------------------------
    expect = _inventory_expectations(cfg)
    R['read_check'] = []
    for r in recs:
        span = float(r.motor.t[-1] - r.motor.t[0]) if len(r.motor) > 1 else 0.
        e = expect.get(r.rid)
        R['read_check'].append(dict(rid=r.rid, motor_layout=r.motor_layout, rows=len(r.motor), span_s=span,
                                    expected_rows=e and e['rows'], expected_span_s=e and e['span'],
                                    same=(e is not None and e['rows'] == len(r.motor) and abs(e['span'] - span) < .05),
                                    lidar=r.lidar is not None, gnss_rows=len(r.gnss), trajectory=r.traj is not None,
                                    checks=r.checks, warnings=r.warnings))
    R['inputs'] = {k: v for r in recs for k, v in r.sources.items()}
    # --- PW-01 in-row --------------------------------------------------------------------------
    cor = ground.Corridors(cfg.path_of('corridor_catalogue'))
    R['inputs'][cfg.relative(cfg.path_of('corridor_catalogue'))] = common.sha256_file(cfg.path_of('corridor_catalogue'))
    locators = {}
    R['catalogue_check'] = {}
    R['corridors_skipped'] = cor.skipped
    for r in recs:
        if r.day == '1010' and r.traj is not None:
            locators[r.rid] = ground.Locator(r, cor)
            R['catalogue_check'][r.short] = cor.check_against_catalogue(r.short, r.traj)
    # --- PW-03 segments ------------------------------------------------------------------------
    segs = []
    for r in recs:
        if not r.analysed:
            continue
        it, iw = mo.imu_yaw(r)
        segs += sp.build_segments(r, mots.get(r.rid), locators.get(r.rid), sp.gnss_epochs(r), it, iw)
    R['segments'] = segs
    main = sp.main_segments(segs)
    R['main_segments'] = main
    rows, fit, measured = sp.speed_table(main)
    R['speed_main'] = dict(rows=rows, fit=fit, measured=measured)
    R['missing_ranges'] = sp.missing_ranges(main)
    R['speed_by_group'] = {}
    for g in sp.GROUPS:
        gs = [s for s in main if s.get('group') == g]
        R['speed_by_group'][g] = {d: sp.ref_cell([s for s in gs if s['duty'] == d]) for d in sorted({s['duty'] for s in gs})}
    R['speed_by_day'] = {}
    for day in DAYS:
        ds = [s for s in main if s['day'] == day]
        R['speed_by_day'][day] = {d: sp.ref_cell([s for s in ds if s['duty'] == d]) for d in sorted({s['duty'] for s in ds})}
    R['speed_by_record'] = {}
    for r in recs:
        rs = [s for s in main if s['rid'] == r.rid]
        if rs:
            R['speed_by_record'][r.short] = {d: sp.ref_cell([s for s in rs if s['duty'] == d])
                                             for d in sorted({s['duty'] for s in rs})}
    R['excluded_not_straight'] = [s for s in segs if s.get('used') and s['direction'] == 'fwd' and not s.get('straight_ok', True)]
    R['reverse_segments'] = [s for s in segs if s.get('used') and s['direction'] == 'rev']
    R['sample_level'] = {day: sp.sample_level_reference([m for m in mots.values() if m.rec.day == day]) for day in DAYS}
    # --- PW-01 in-row fractions per 10/10 recording -------------------------------------------------
    R['inrow'] = []
    for r in recs:
        if r.day != '1010' or not r.analysed:
            continue
        row = dict(short=r.short, rid=r.rid)
        loc = locators.get(r.rid)
        if loc is None:
            row['note'] = 'Cartographer 궤적 없음'
            R['inrow'].append(row)
            continue
        m = mots[r.rid]
        moving = m.valid & (m.spd > .1)
        known, inside = loc.at(m.t[moving])
        row['nodes'] = int(len(loc.t))
        row['corridors_used'] = len(cor.by_record.get(r.short, []))
        row['node_frac'] = float(loc.node_inside.mean())
        row['path_frac'], row['path_m'] = loc.path_fraction()
        row['moving_s'] = float(moving.sum()) * .1
        row['moving_known_frac'] = float(known.mean()) if len(known) else None
        row['moving_inrow_frac'] = float(inside[known].mean()) if known.any() else None
        rseg = [s for s in main if s['rid'] == r.rid]
        t_in = sum(s['window_s'] for s in rseg if s.get('group') == sp.GROUP_IN)
        t_out = sum(s['window_s'] for s in rseg if s.get('group') == sp.GROUP_OUT)
        t_unk = sum(s['window_s'] for s in rseg if s.get('group') == sp.GROUP_UNK)
        row['segment_s'] = dict(inrow=t_in, out=t_out, unknown=t_unk)
        row['segment_inrow_frac'] = t_in / (t_in + t_out) if (t_in + t_out) > 0 else None
        row['catalogue_check_ok'] = all(x['same'] for x in R['catalogue_check'].get(r.short, []))
        R['inrow'].append(row)
    # --- PW-01 recording table -----------------------------------------------------------------------
    R['recordings'] = []
    for r in recs:
        first = last = None
        gt = [g.get('gps_time') for g in r.gnss if g.get('gps_time')]
        if gt:
            first, last = load._gps_seconds(gt[0]), load._gps_seconds(gt[-1])
        kst = lambda x: dt.datetime.fromtimestamp(x, tz=KST)  # noqa: E731
        secs = sp.settled_seconds(r.motor)
        rseg = [s for s in segs if s['rid'] == r.rid and s.get('used')]
        R['recordings'].append(dict(
            rid=r.rid, short=r.short, day=r.day, ground=r.ground, evidence=r.evidence, analysed=r.analysed,
            kst_start=kst(first).strftime('%Y-%m-%d %H:%M:%S') if first else None,
            kst_end=kst(last).strftime('%H:%M:%S') if last else None,
            duration_s=r.metadata.get('duration_s'), host_minus_receiver_s=(None if r.receiver_minus_host_s is None
                                                                          else -r.receiver_minus_host_s),
            motor_rows=len(r.motor), settled_straight_s=secs,
            segments_used=len([s for s in rseg if s['direction'] == 'fwd' and s.get('straight_ok', True)]),
            refs=[x for x, ok in (('LiDAR', r.lidar is not None), ('GNSS 속도', bool(r.gnss)),
                                  ('Cartographer', r.traj is not None)) if ok],
            files=sorted(r.sources.keys())))
    # --- PW-04 starts / stops / distance --------------------------------------------------------------
    starts, stops = [], []
    for r in recs:
        if not r.analysed:
            continue
        starts += ss.start_events(r, mots.get(r.rid))
        stops += ss.stop_events(r, mots.get(r.rid))
    for e in starts + stops:
        e['ground_label'] = R['grounds'].get(e['ground'], e['ground'])
    R['starts'] = starts
    R['stops'] = stops
    R['distance'] = distance_table(starts, stops, rows)
    # --- PW-05 ---------------------------------------------------------------------------------
    R['session'] = session.analyse(segs, recs)
    R['recs'] = recs
    R['by_rid'] = by_rid
    return R


DIST_DUTIES = (30, 35, 40, 45, 50, 55, 60, 70, 80, 90, 100)


def distance_table(starts, stops, speed_rows):
    fwd = [e for e in starts if e['cls'] == 'straight_fwd' and e['precise']]
    onset = [e['duty_at_onset'] for e in starts if e['cls'] == 'straight_fwd' and e['precise']
             and e.get('onset_s') is not None and e['day'] == '1010']
    breakaway = float(np.median(onset)) if onset else 34.
    tab = sorted((r['duty'], r['v']) for r in speed_rows if r.get('v') is not None)
    xs = np.array([d for d, _ in tab], dtype=float)
    ys = np.array([v for _, v in tab], dtype=float)

    def v_of(d):
        if d <= xs[0]:
            # below the first table row: straight line to zero at 0 % (only used inside the ramp, < breakaway it is 0)
            return float(ys[0] * d / xs[0])
        return float(np.interp(d, xs, ys))
    rows = []
    clean = [e for e in stops if e.get('clean') and e.get('speed_at_release') is not None
             and e.get('speed_consistent') is True]
    for duty in DIST_DUTIES:
        srow = next((r for r in speed_rows if r['duty'] == duty), None)
        row = dict(duty=duty, v_ss=v_of(duty), v_grade=srow['grade'] if srow else '보간',
                   v_ref=(srow or {}).get('ref'))
        evs = [e for e in fwd if e['duty'] == duty and e.get('good_frac', 0.) >= ss.MIN_GOOD_FRAC]
        for T in ss.HOLD_TIMES:
            key = 'd_%g' % T
            meas = [e[key] for e in evs if e.get(key) is not None]
            model = ss.model_distance(duty, T, v_of, breakaway)
            row[key] = dict(measured=float(np.median(meas)) if meas else None, n=len(meas),
                            measured_range=[float(min(meas)), float(max(meas))] if meas else None,
                            model=model)
        # key-release stop distance: measured = clean release stops from this duty (pre-stop speed consistent)
        vss = v_of(duty)
        near = [e['dist_m'] for e in clean if e['prev_duty'] == duty]
        row['stop'] = dict(measured=float(np.median(near)) if near else None, n=len(near),
                           measured_range=[float(min(near)), float(max(near))] if near else None,
                           model=ss.release_model(vss))
        rows.append(row)
    return dict(rows=rows, breakaway_duty=breakaway, breakaway_n=len(onset), model=ss.MODEL)
