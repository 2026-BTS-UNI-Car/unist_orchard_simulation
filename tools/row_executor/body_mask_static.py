"""C0: fit the per-beam body self-return mask from a STANDSTILL scan set (review S5 limits enforced). Read-only.

Why: body_mask_fit.py fits on MOVING scans (a moving vehicle sweeps the scene past every beam, so only the vehicle
gives a constant near return); on site (C0) the vehicle stands still, so a standstill fit must tell the vehicle from
things that merely stand next to it. Standard library only (runs with the Jetson host python3; the config module is
the executor's own, numpy only for the --npz test input).

Input (one of)
  --recording DIR   a dashboard recording (DIR/mapping_source/sensors.jsonl.gz, or DIR/sensors.jsonl.gz): /scan rows
                    {"topic", "message", "bag_time_ns"}; --from / --to seconds relative to the first /scan row
  --sidecar FILE    the same sensors.jsonl.gz directly
  --npz FILE --index A:B   a 10/10 compact extract (<rid>.scans.npz, ranges in mm, 0 = no return), scans A..B-1
Procedure on site: the vehicle stands still on open ground, nobody within 1.5 m, nothing touching it, dashboard
recording 'both' for >= 30 s, then this tool on that recording (README_FIELD.txt C0).

Rule (constants below; every limit comes from the config: lidar_yaw_offset_deg, lidar_x_in_base_m, footprint,
beam grid, body_mask_limit_m)
  1 stationarity: per-beam median of the 1-20 m returns is the reference; a scan whose median |r - ref| exceeds
    3 cm is 'moving' and dropped; more than 10 % moving scans, or fewer than 20 scans left: REFUSED (the vehicle or
    the scene moved; capture again).
  2 per beam, limit = min(footprint boundary + 0.05 m, forward-sector front edge + 0.05 m, body_mask_limit_m)
    (config.body_mask_beam_limits: the S5 rule config.validate() enforces):
      masked        returns <= limit in >= 90 % of the scans, spread (p95 - p5) <= 5 cm, and returns just beyond
                    the limit (< limit + 0.3 m) in < 5 % of the scans: mask = min(max return + 2 cm, limit)
      inconsistent  returns <= limit in 5-90 % of the scans, or spread > 5 cm, or the returns straddle the limit:
                    REFUSED for that beam (not masked; a person, a strap, noise, or a part that partly sticks out)
      outside       returns <= limit in < 5 % but just beyond the limit (limit .. limit + 0.3 m) in >= 50 %: NOT
                    maskable (something stands next to the vehicle, or the footprint/LiDAR position in the config
                    is too small): reported as a warning, never masked
      one object    beams with any of these classes, at most 2 beams apart, are one object; if the object has an
                    'outside' or a straddling beam, it crosses the limit (a person at the corner, a protruding part):
                    none of its beams is masked ('object_crosses_limit')
  3 consecutive masked beams form segments (split where the mask ranges differ by > 5 cm or would exceed a beam's
    own limit); the fragment {"body_mask": [[first, last, max_range_m], ...]} is validated with config.from_dict.
The mask REPLACES the configured one (a fit that finds nothing gives an empty mask).

Output: a printed table (what was masked, what was refused, warnings), --out-fragment (default
body_mask_fragment.json), optional --out-config (base config + fragment, validated, nothing else changed) and
--report (full JSON). Exit 0 = fragment written, 3 = refused (nothing written), 2 = usage/input error,
4 = forward-sector segments not confirmed (nothing written).

Forward sector (review_safety_day2 N5): a person or object touching the bumper during the capture looks exactly like a
body part and would be masked (probe: a leg 3 cm in front of the bumper -> masked -> the next Start drove into it). A
segment within FORWARD_CONFIRM_DEG of the travel axis is therefore written only with --confirm-forward-sector, given
after each such segment (printed with its bearing and range) was matched to a real part of the vehicle.
"""
import argparse
import gzip
import json
import math
import os
import sys

if __package__ in (None, ''):                      # run as a script: python3 body_mask_static.py ...
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from row_executor import config as C               # noqa: E402  (stdlib only)

SCHEMA = 'ubts.row_executor.body_mask_static.v1'
RULE = dict(min_scans=20, max_scans=1200, station_tol_m=.03, max_moving_fraction=.10, ref_min_m=1., ref_max_m=20.,
            mask_fraction=.90, ignore_fraction=.05, max_spread_m=.05, straddle_fraction=.05, outside_reach_m=.30,
            outside_fraction=.50, margin_m=.02, segment_spread_m=.05, object_gap_beams=2)
SCAN_PREFIX = b'{"topic":"/scan"'
FORWARD_CONFIRM_DEG = C.BODY_MASK_FORWARD_SECTOR_DEG     # 35 deg either side of the travel axis


class InputError(ValueError):
    pass


# --- input ------------------------------------------------------------------------------------------------------
def sidecar_path(recording):
    for rel in (('mapping_source', 'sensors.jsonl.gz'), ('sensors.jsonl.gz',)):
        path = os.path.join(recording, *rel)
        if os.path.isfile(path):
            return path
    raise InputError('%s: no mapping_source/sensors.jsonl.gz or sensors.jsonl.gz' % recording)


def read_sidecar(path, t_from=None, t_to=None, max_scans=RULE['max_scans']):
    """/scan rows of a recorder sidecar (gzip JSONL) as scan dicts; times relative to the first /scan row."""
    scans, t0 = [], None
    with gzip.open(path, 'rb') as fh:
        for line in fh:
            if not line.startswith(SCAN_PREFIX):
                if b'"/scan"' not in line[:80]:
                    continue
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if row.get('topic') != '/scan':
                continue
            t = row.get('bag_time_ns')
            t = t / 1e9 if isinstance(t, (int, float)) else None
            if t is None:
                continue
            t0 = t if t0 is None else t0
            rel = t - t0
            if t_from is not None and rel < t_from:
                continue
            if t_to is not None and rel > t_to:
                break
            m = row.get('message') or {}
            scans.append(dict(ranges=m.get('ranges') or [], angle_min=m.get('angle_min'),
                              increment=m.get('angle_increment'), range_min=m.get('range_min', 0.),
                              range_max=m.get('range_max'), t=rel))
            if len(scans) > max_scans:
                raise InputError('more than %d scans selected: give --from/--to (a 30-60 s standstill)' % max_scans)
    return scans


def read_npz(path, first, last):
    """10/10 compact extract (numpy needed): ranges uint16 mm, 0 = no return."""
    import numpy as np
    z = np.load(path)
    mm = z['ranges_mm'][first:last]
    geo = dict(angle_min=float(z['angle_min']), increment=float(z['angle_increment']),
               range_min=float(z['range_min']), range_max=float(z['range_max']))
    return [dict(ranges=[(v / 1000. if v else None) for v in row.tolist()], t=float(k) * .1, **geo)
            for k, row in enumerate(mm)]


# --- fit ----------------------------------------------------------------------------------------------------------
def _quantile(sorted_values, q):
    if not sorted_values:
        return None
    pos = q * (len(sorted_values) - 1)
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(sorted_values) - 1)
    return sorted_values[lo] + (sorted_values[hi] - sorted_values[lo]) * (pos - lo)


def _median(values):
    return _quantile(sorted(values), .5)


def _valid(r, rmin, rmax):
    return isinstance(r, (int, float)) and not isinstance(r, bool) and math.isfinite(r) and rmin <= r < rmax


def _geometry(scans):
    if not scans:
        raise InputError('no scans')
    first = scans[0]
    try:
        n, amin, inc = len(first['ranges']), float(first['angle_min']), float(first['increment'])
        rmin, rmax = float(first.get('range_min') or 0.), float(first['range_max'])
    except (KeyError, TypeError, ValueError):
        raise InputError('scan geometry missing') from None
    if n < 3 or not inc > 0:
        raise InputError('scan geometry invalid')
    for s in scans:
        if (len(s['ranges']) != n or abs(float(s['angle_min']) - amin) > 1e-9
                or abs(float(s['increment']) - inc) > 1e-12):
            raise InputError('scan geometry changes inside the set (beam count / angles): capture again')
    return n, amin, inc, rmin, rmax


def _clusters(beams, info):
    """Consecutive data beams -> [(first, last)]."""
    out = []
    for k in beams:
        if out and k == out[-1][1] + 1:
            out[-1][1] = k
        else:
            out.append([k, k])
    return [tuple(c) for c in out]


def fit_static_mask(cfg, scans, rule=RULE):
    """Fit (see the module docstring). Returns the result dict; 'refused' is a reason string or None."""
    n, amin, inc, rmin_scan, rmax_scan = _geometry(scans)
    rmin = max(rmin_scan, float(cfg.range_min_floor_m))
    rmax = min(rmax_scan, 30.)
    out = dict(schema=SCHEMA, rule=dict(rule), scans_in=len(scans), beams=n, refused=None, segments=[],
               masked=[], inconsistent=[], outside=[], warnings=[])
    if len(scans) < rule['min_scans']:
        out['refused'] = 'too_few_scans (%d < %d)' % (len(scans), rule['min_scans'])
        return out
    # 1 stationarity
    columns = [[] for _ in range(n)]
    for s in scans:
        for k, r in enumerate(s['ranges']):
            if _valid(r, rule['ref_min_m'], rule['ref_max_m']):
                columns[k].append(r)
    reference = [(_median(col) if len(col) >= .5 * len(scans) else None) for col in columns]
    ref_beams = [k for k in range(n) if reference[k] is not None]
    if len(ref_beams) < 30:
        out['refused'] = 'too_few_reference_returns (%d beams with 1-20 m returns): open ground without structures' \
                         % len(ref_beams)
        return out
    keep, deviations = [], []
    for s in scans:
        dev = [abs(s['ranges'][k] - reference[k]) for k in ref_beams if _valid(s['ranges'][k], rule['ref_min_m'],
                                                                                 rule['ref_max_m'])]
        d = _median(dev) if dev else math.inf
        deviations.append(d)
        if d <= rule['station_tol_m']:
            keep.append(s)
    moving = len(scans) - len(keep)
    out.update(scans_used=len(keep), scans_moving=moving,
               deviation_m_p50_p90=[round(_quantile(sorted(deviations), q), 4) for q in (.5, .9)])
    if moving > rule['max_moving_fraction'] * len(scans):
        out['refused'] = 'not_stationary (%d of %d scans moved by > %g m)' % (moving, len(scans), rule['station_tol_m'])
        return out
    if len(keep) < rule['min_scans']:
        out['refused'] = 'too_few_still_scans (%d)' % len(keep)
        return out
    # 2 per beam
    boundary, forward, bearings = C.body_mask_beam_limits(cfg)
    offset = math.radians(cfg.lidar_yaw_offset_deg)
    count, gmin, ginc = cfg.beam_grid_count, cfg.beam_grid_angle_min_rad, cfg.beam_grid_increment_rad
    total = len(keep)
    grid_mask = {}
    classes = {}
    for k in range(n):
        g = int(round((amin + k * inc - gmin) / ginc)) % count
        limit = min(boundary[g], forward[g], float(cfg.body_mask_limit_m))
        inside, beyond = [], 0
        for s in keep:
            r = s['ranges'][k]
            if not _valid(r, rmin, rmax):
                continue
            if r <= limit:
                inside.append(r)
            elif r <= limit + rule['outside_reach_m']:
                beyond += 1
        f_in, f_out = len(inside) / total, beyond / total
        if f_in < rule['ignore_fraction']:
            if f_out >= rule['outside_fraction']:
                classes[k] = ('outside', g, limit, f_in, f_out, None)
            continue
        inside.sort()
        p5, p95 = _quantile(inside, .05), _quantile(inside, .95)
        spread = p95 - p5
        if f_in < rule['mask_fraction']:
            why = 'intermittent'
        elif spread > rule['max_spread_m']:
            why = 'spread'
        elif f_out >= rule['straddle_fraction']:
            why = 'straddles_limit'
        else:
            mask = min(inside[-1] + rule['margin_m'], limit)
            grid_mask[g] = max(grid_mask.get(g, 0.), mask)
            classes[k] = ('masked', g, limit, f_in, f_out, (p5, p95, inside[-1], mask))
            continue
        classes[k] = ('inconsistent', g, limit, f_in, f_out, (p5, p95, inside[-1], why))
    # an object that crosses the limit is one object, not the vehicle: none of its beams is masked
    present = sorted(classes)
    groups, run = [], []
    for k in present:
        if run and k - run[-1] > rule['object_gap_beams'] + 1:
            groups.append(run)
            run = []
        run.append(k)
    if run:
        groups.append(run)
    for members in groups:
        crosses = any(classes[k][0] == 'outside' or (classes[k][0] == 'inconsistent'
                                                     and classes[k][5][3] == 'straddles_limit') for k in members)
        if not crosses:
            continue
        for k in members:
            c = classes[k]
            if c[0] == 'masked':
                classes[k] = ('inconsistent', c[1], c[2], c[3], c[4], c[5][:3] + ('object_crosses_limit',))
                grid_mask.pop(c[1], None)
    # 3 segments on the grid
    segments, current = [], None
    for g in sorted(grid_mask):
        value, limit = grid_mask[g], _grid_limit(g, boundary, forward, cfg)
        if current is not None and g == current['last'] + 1:
            values = current['values'] + [value]
            lim = min(current['limit'], limit)
            if max(values) - min(values) <= rule['segment_spread_m'] and max(values) <= lim + 1e-9:
                current.update(last=g, values=values, limit=lim)
                continue
        current = dict(first=g, last=g, values=[value], limit=limit)
        segments.append(current)
    body_mask = [[s['first'], s['last'], math.floor(max(s['values']) * 1000. + 1e-6) / 1000.] for s in segments]
    if len(body_mask) > C.MAX_MASK_SEGMENTS:
        out['refused'] = 'too_many_segments (%d > %d): the scene is too cluttered' % (len(body_mask),
                                                                                      C.MAX_MASK_SEGMENTS)
        return out
    data = cfg.to_dict()
    data['body_mask'] = body_mask
    try:
        merged = C.from_dict(data)
    except C.ConfigError as exc:
        out['refused'] = 'fragment_rejected_by_config: ' + '; '.join(exc.problems[:5])
        return out
    out['segments'] = [dict(first=s[0], last=s[1], max_range_m=s[2], beams=s[1] - s[0] + 1,
                            bearing_deg=[round(bearings[s[0]], 1), round(bearings[s[1]], 1)]) for s in body_mask]
    out['fragment'] = dict(body_mask=body_mask)
    out['config_digest_with_mask'] = merged.digest()
    for name in ('masked', 'inconsistent', 'outside'):
        beams = sorted(k for k, c in classes.items() if c[0] == name)
        rows = []
        for first, last in _clusters(beams, classes):
            members = [classes[k] for k in range(first, last + 1)]
            row = dict(beams=[first, last], n=last - first + 1,
                       bearing_deg=[round(bearings[members[0][1]], 1), round(bearings[members[-1][1]], 1)],
                       limit_m=[round(min(m[2] for m in members), 3), round(max(m[2] for m in members), 3)],
                       fraction_in_max=round(max(m[3] for m in members), 2),
                       fraction_beyond_max=round(max(m[4] for m in members), 2))
            if name != 'outside':
                row['range_m_p5_p95'] = [round(min(m[5][0] for m in members), 3),
                                         round(max(m[5][1] for m in members), 3)]
            if name == 'inconsistent':
                row['why'] = sorted({m[5][3] for m in members})
            rows.append(row)
        out[name] = rows
    out['effect'] = _effect(cfg, keep, n, amin, inc, rmin, rmax, grid_mask)
    if out['outside']:
        out['warnings'].append('persistent returns just OUTSIDE the footprint limit (not masked): a person/object next '
                               'to the vehicle, or footprint_length/width_m / lidar_x_in_base_m too small in the config')
    if out['inconsistent']:
        out['warnings'].append('inconsistent near returns (not masked): something moved, flapped or stuck out; '
                               'check the vehicle and capture again if it should have been masked')
    return out


def _grid_limit(g, boundary, forward, cfg):
    return min(boundary[g], forward[g], float(cfg.body_mask_limit_m))


def _effect(cfg, scans, n, amin, inc, rmin, rmax, grid_mask):
    """Returns per scan inside footprint + 0.05 m (the zone a mask may cover): p50 before / after the mask."""
    boundary, _, _ = C.body_mask_beam_limits(cfg)
    count, gmin, ginc = cfg.beam_grid_count, cfg.beam_grid_angle_min_rad, cfg.beam_grid_increment_rad
    grid = [int(round((amin + k * inc - gmin) / ginc)) % count for k in range(n)]
    before, after = [], []
    for s in scans:
        b = a = 0
        for k, r in enumerate(s['ranges']):
            if _valid(r, rmin, rmax) and r <= boundary[grid[k]]:
                b += 1
                if r > grid_mask.get(grid[k], 0.):
                    a += 1
        before.append(b)
        after.append(a)
    return dict(inside_footprint_returns_p50_before=_median(before), inside_footprint_returns_p50_after=_median(after),
                inside_footprint_returns_max_after=max(after) if after else None)


# --- report -------------------------------------------------------------------------------------------------------
def forward_segments(result, limit_deg=FORWARD_CONFIRM_DEG):
    """Segments with a beam within limit_deg of the travel axis (bearing_deg is [first, last], increasing)."""
    out = []
    for seg in result.get('segments') or ():
        b0, b1 = seg['bearing_deg']
        nearest = 0. if b0 <= 0. <= b1 else min(abs(b0), abs(b1))
        if nearest <= limit_deg:
            out.append(seg)
    return out


def report_lines(result):
    lines = ['정지 scan %s개 중 사용 %s개 (움직임으로 버림 %s, 편차 p50/p90 %s m)' % (
        result['scans_in'], result.get('scans_used'), result.get('scans_moving'), result.get('deviation_m_p50_p90'))]
    if result['refused']:
        lines.append('거부: %s (config 조각을 쓰지 않음)' % result['refused'])
        return lines
    lines.append('마스크 구간 %d개 (beam 번호는 원래 scan 순서, 방위 0° = 진행 방향, + = 왼쪽):' % len(result['segments']))
    forward = forward_segments(result)
    for s in result['segments']:
        lines.append('  beam %4d-%4d (%3d개) 방위 %+6.1f..%+6.1f°  ≤ %.3f m%s' % (
            s['first'], s['last'], s['beams'], s['bearing_deg'][0], s['bearing_deg'][1], s['max_range_m'],
            '  ← 진행 방향 ±%g°: 실제 부품 확인 필요' % FORWARD_CONFIRM_DEG if s in forward else ''))
    for name, title in (('inconsistent', '일관되지 않아 거부(마스크 안 함)'), ('outside', '차체 윤곽 밖의 지속 반환(마스크 불가, 경고)')):
        if result[name]:
            lines.append(title + ':')
            for r in result[name]:
                lines.append('  beam %4d-%4d 방위 %+6.1f..%+6.1f°  한도 %.3f-%.3f m  빈도 안 %.2f / 밖 %.2f%s%s' % (
                    r['beams'][0], r['beams'][1], r['bearing_deg'][0], r['bearing_deg'][1], r['limit_m'][0],
                    r['limit_m'][1], r['fraction_in_max'], r['fraction_beyond_max'],
                    ('  거리 %.3f-%.3f m' % tuple(r['range_m_p5_p95'])) if 'range_m_p5_p95' in r else '',
                    ('  (%s)' % ','.join(r['why'])) if 'why' in r else ''))
    e = result['effect']
    lines.append('윤곽(+5 cm) 안 반환/scan p50: 마스크 전 %s → 후 %s (후 최대 %s)' % (
        e['inside_footprint_returns_p50_before'], e['inside_footprint_returns_p50_after'],
        e['inside_footprint_returns_max_after']))
    lines += ['경고: ' + w for w in result['warnings']]
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description='Fit the body self-return mask from a standstill scan set (C0)')
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--recording')
    src.add_argument('--sidecar')
    src.add_argument('--npz')
    ap.add_argument('--index', help='A:B scans of --npz')
    ap.add_argument('--from', dest='t_from', type=float, help='seconds after the first /scan row')
    ap.add_argument('--to', dest='t_to', type=float)
    ap.add_argument('--config', help='executor config.json (footprint, LiDAR position, beam grid; default: built-in '
                                     'defaults)')
    ap.add_argument('--out-fragment', default='body_mask_fragment.json')
    ap.add_argument('--out-config', help='write --config with the fitted body_mask (validated)')
    ap.add_argument('--report')
    ap.add_argument('--confirm-forward-sector', action='store_true',
                    help='write segments within %g deg of the travel axis (only after matching each one to a real part '
                         'of the vehicle; review_safety_day2 N5)' % FORWARD_CONFIRM_DEG)
    args = ap.parse_args(argv)
    try:
        if args.config:
            with open(args.config, 'r', encoding='utf-8') as fh:
                cfg = C.loads(fh.read())
        else:
            cfg = C.ExecutorConfig()
            print('note: no --config: built-in defaults (footprint %.2f x %.2f m, LiDAR x %.2f m)' % (
                cfg.footprint_length_m, cfg.footprint_width_m, cfg.lidar_x_in_base_m))
        if args.npz:
            if not args.index or ':' not in args.index:
                raise InputError('--npz needs --index A:B')
            a, b = (int(x) for x in args.index.split(':'))
            scans = read_npz(args.npz, a, b)
        else:
            path = args.sidecar or sidecar_path(args.recording)
            scans = read_sidecar(path, args.t_from, args.t_to)
        result = fit_static_mask(cfg, scans)
    except (OSError, InputError, C.ConfigError) as exc:
        print('input error: %s' % exc, file=sys.stderr)
        return 2
    for line in report_lines(result):
        print(line)
    if args.report:
        _write(args.report, json.dumps(result, indent=1, sort_keys=True, allow_nan=False) + '\n')
    if result['refused']:
        return 3
    forward = forward_segments(result)
    if forward and not args.confirm_forward_sector:
        print('진행 방향 ±%g° 안의 마스크 %d구간: 사람·물건이 범퍼에 닿아 있어도 이렇게 보인다. 각 구간(방위·거리)을 차의 실제 부품과 '
              '맞춰 본 뒤 --confirm-forward-sector를 붙여 다시 실행한다. 아무것도 쓰지 않았다(종료 코드 4).'
              % (FORWARD_CONFIRM_DEG, len(forward)))
        return 4
    _write(args.out_fragment, json.dumps(result['fragment'], indent=1, sort_keys=True) + '\n')
    print('config 조각: %s (body_mask %d구간; 기존 body_mask를 대체)' % (args.out_fragment, len(result['segments'])))
    if args.out_config:
        data = cfg.to_dict()
        data.update(result['fragment'])
        merged = C.from_dict(data)
        _write(args.out_config, C.dumps(merged))
        print('config: %s (digest %s)' % (args.out_config, merged.digest()[:12]))
    return 0


def _write(path, text):
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(text)


if __name__ == '__main__':
    sys.exit(main())
