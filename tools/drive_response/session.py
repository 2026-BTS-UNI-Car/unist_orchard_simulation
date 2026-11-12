"""PW-05: speed at 100 % duty against the time of day (battery proxy, NOT a battery measurement).

Inputs are the straight 100 % segments of the speed analysis (same rule as PW-03). Real time = host receipt time
+ the receiver-minus-host offset of the recording (median over its GNSS rows), shown in KST. Slopes are fitted
only inside one surface group of one day; a slope over mixed surfaces would mostly measure the surface.
Two slope estimates: over the segment medians (segments of one recording are not independent, so its standard
error is too small) and over the recording medians (few points).
"""
import numpy as np

import speed as sp

KEYS = (('lidar_v', 'LiDAR'), ('gnss_v', 'GNSS'))


def _ols(x, y):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(x) < 3 or np.ptp(x) <= 0:
        return None
    b, a = np.polyfit(x, y, 1)
    res = y - (a + b * x)
    se = float(np.sqrt(res @ res / (len(x) - 2) / ((x - x.mean()) @ (x - x.mean()))))
    return dict(slope_per_h=float(b), se_per_h=se, n=int(len(x)), mean=float(y.mean()),
                rel_slope_per_h=float(b / y.mean()) if y.mean() else None)


def analyse(segs, recs):
    seg100 = [s for s in sp.main_segments(segs) if s['duty'] == 100 and s.get('kst_hours') is not None]
    by_rec = {r.short: r for r in recs}
    out = dict(groups=[], records=[], first_last=[])
    # per recording medians
    for day in ('1003', '1010'):
        shorts = sorted({s['short'] for s in seg100 if s['day'] == day},
                        key=lambda k: min(s['kst_hours'] for s in seg100 if s['short'] == k))
        for short in shorts:
            ss = [s for s in seg100 if s['short'] == short]
            row = dict(day=day, short=short, ground=by_rec[short].ground,
                       kst_first=min(s['kst'] for s in ss), kst_last=max(s['kst'] for s in ss),
                       hours_mid=float(np.median([s['kst_hours'] for s in ss])))
            for key, name in KEYS:
                vals = [s[key] for s in ss if s.get(key) is not None]
                row[name] = dict(n=len(vals), v=float(np.median(vals)) if vals else None)
            for g in sp.GROUPS:
                gv = [s['lidar_v'] for s in ss if s.get('group') == g and s.get('lidar_v') is not None]
                if gv:
                    row.setdefault('by_group', {})[g] = dict(n=len(gv), v=float(np.median(gv)))
            out['records'].append(row)
    # slopes inside one group of one day
    for day in ('1003', '1010'):
        for g in sp.GROUPS:
            ss = [s for s in seg100 if s['day'] == day and s.get('group') == g]
            if not ss:
                continue
            entry = dict(day=day, group=g)
            if len({s['short'] for s in ss}) < 2:
                entry['note'] = 'one recording only: no session-time slope'
                for key, name in KEYS:
                    entry[name + '_segments'] = entry[name + '_records'] = None
                    entry[name + '_record_points'] = []
                out['groups'].append(entry)
                continue
            for key, name in KEYS:
                pts = [(s['kst_hours'], s[key]) for s in ss if s.get(key) is not None]
                entry[name + '_segments'] = _ols([p[0] for p in pts], [p[1] for p in pts]) if pts else None
                recmed = []
                for short in sorted({s['short'] for s in ss}):
                    v = [s[key] for s in ss if s['short'] == short and s.get(key) is not None]
                    h = [s['kst_hours'] for s in ss if s['short'] == short and s.get(key) is not None]
                    if v:
                        recmed.append((float(np.median(h)), float(np.median(v)), short))
                entry[name + '_records'] = _ols([p[0] for p in recmed], [p[1] for p in recmed])
                entry[name + '_record_points'] = recmed
            out['groups'].append(entry)
    # first vs last recording of the day (all surfaces, and inside one group)
    for day in ('1003', '1010'):
        rows = [r for r in out['records'] if r['day'] == day]
        for name in ('LiDAR', 'GNSS'):
            have = [r for r in rows if r[name]['v'] is not None]
            if len(have) >= 2:
                f, l = have[0], have[-1]
                out['first_last'].append(dict(day=day, ref=name, scope='하루 전체(지면 섞임)', first=f['short'],
                                              first_kst=f['kst_first'], last=l['short'], last_kst=l['kst_first'],
                                              v_first=f[name]['v'], v_last=l[name]['v'],
                                              ratio=l[name]['v'] / f[name]['v']))
        for g in sp.GROUPS:
            have = [r for r in rows if (r.get('by_group') or {}).get(g)]
            if len(have) >= 2:
                f, l = have[0], have[-1]
                out['first_last'].append(dict(day=day, ref='LiDAR', scope=g, first=f['short'], first_kst=f['kst_first'],
                                              last=l['short'], last_kst=l['kst_first'], v_first=f['by_group'][g]['v'],
                                              v_last=l['by_group'][g]['v'],
                                              ratio=l['by_group'][g]['v'] / f['by_group'][g]['v']))
    out['segments'] = [dict(day=s['day'], short=s['short'], group=s['group'], kst=s['kst'], hours=s['kst_hours'],
                            lidar_v=s.get('lidar_v'), gnss_v=s.get('gnss_v'), window_s=s['window_s']) for s in seg100]
    return out
