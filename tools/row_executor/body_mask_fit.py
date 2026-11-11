"""Fit a per-beam body self-return mask from real records (leave-one-out capable). Read-only on the records.

Rule (NOTES/WEEKEND_PLAN.txt 4.4): on scans where the reference LiDAR forward speed |v| > 0.3 m/s, for each beam
the fraction of scans with a return nearer than near_m and the spread (sd) of those near returns. A beam with
fraction > min_fraction and sd < max_sd sees a fixed part of the vehicle: mask_r[i] = p95 of its near returns
+ margin. Other beams: mask_r = 0 (no mask). A moving vehicle sweeps the scene past every beam, so only the
vehicle (fixed in the LiDAR frame) can give a frequent, constant near return.

Usage: python -I -B body_mask_fit.py --out mask.json [--exclude RID] [--records RID ...]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import field_records as F       # noqa: E402

RULE = dict(near_m=1.2, moving_m_s=.3, min_fraction=.5, max_sd=.05, margin_m=.05)


def beam_stats(rec, near_m, moving_m_s):
    vf = rec['ref']['vf']
    if len(vf) != len(rec['t_scan']) or not np.allclose(rec['ref']['t'], rec['t_scan']):
        raise ValueError('reference/scan time mismatch: ' + rec['rid'])
    moving = np.isfinite(vf) & (np.abs(vf) > moving_m_s)
    mm = rec['ranges_mm'][moving]
    near = (mm > 0) & (mm < near_m * 1000.)
    rn = np.where(near, mm, 0).astype(np.float64) / 1000.
    return dict(scans=int(moving.sum()), count=near.sum(axis=0).astype(np.int64), sum=rn.sum(axis=0),
                sumsq=(rn * rn).sum(axis=0), near=near, mm=mm)


def fit_mask(rids, rule=RULE, base=F.BASE_1010):
    stats = [beam_stats(F.load_record(rid, base), rule['near_m'], rule['moving_m_s']) for rid in rids]
    beams = len(stats[0]['count'])
    scans = sum(s['scans'] for s in stats)
    count = sum(s['count'] for s in stats)
    total = sum(s['sum'] for s in stats)
    totsq = sum(s['sumsq'] for s in stats)
    fraction = count / max(1, scans)
    with np.errstate(invalid='ignore', divide='ignore'):
        mean = np.where(count > 0, total / np.maximum(count, 1), np.nan)
        sd = np.sqrt(np.maximum(0., np.where(count > 1, totsq / np.maximum(count, 1) - mean * mean, np.nan)))
    chosen = np.nonzero((fraction > rule['min_fraction']) & (sd < rule['max_sd']))[0]
    mask_r = np.zeros(beams)
    for i in chosen:
        values = np.concatenate([s['mm'][s['near'][:, i], i] for s in stats]) / 1000.
        mask_r[i] = float(np.percentile(values, 95)) + rule['margin_m']
    top = np.argsort(-fraction)[:10]
    return dict(schema='ubts.row_executor.body_mask.v1', rule=rule, trained_on=list(rids), moving_scans=int(scans),
                beams=beams, masked_beams=int(len(chosen)), mask_r_m=[round(float(v), 3) for v in mask_r],
                max_near_fraction=round(float(fraction.max()), 4),
                beams_fraction_gt_0_05=int((fraction > .05).sum()), beams_fraction_gt_0_2=int((fraction > .2).sum()),
                top_beams=[dict(beam=int(i), fraction=round(float(fraction[i]), 4),
                                mean_m=None if not np.isfinite(mean[i]) else round(float(mean[i]), 3),
                                sd_m=None if not np.isfinite(sd[i]) else round(float(sd[i]), 3)) for i in top])


def load_mask(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    mask = np.asarray(data['mask_r_m'], dtype=np.float64)
    return (mask if (mask > 0).any() else None), data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--records', nargs='*', default=list(F.RECORDS_1010))
    ap.add_argument('--exclude', nargs='*', default=[])
    args = ap.parse_args()
    rids = [r for r in args.records if r not in args.exclude]
    result = fit_mask(rids)
    result['held_out'] = list(args.exclude)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(json.dumps(result, indent=1) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'mask_r_m'}))


if __name__ == '__main__':
    main()
