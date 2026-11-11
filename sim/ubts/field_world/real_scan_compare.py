"""A6 real scan comparison (design 7.2 A6; world survey 5.9 item 2): synthetic scans vs the recorded 10/10 scans.

At SLAM trajectory nodes of the offline maps (maps_1010_root/offline/maps/<record>/trajectory.csv; the node pose is
the laser pose: tracking frame imu_link = laser in the offline run) inside a development corridor (0 <= s <= s_E,
|d| <= NODE_D_MAX_M), a synthetic scan of the corridor's raster world (RASTER_V1: reflectors return, unobserved ends
the ray) with the recorded beam geometry (angle_min + i x angle_increment, 3240 beams, laser frame) is compared with
the recorded scan of that node (<rid>.scans.npz, ranges in mm, 0 = no return). The node -> scan pairing: the offline
run's input scan i is scans.npz index i + k, k = the index of the run's first /scan header in scans.npz (CUSTODY.json
of the run, an exact header match); every pair is checked against the run's scan end ticks (end - header in
[0.05, 0.25] s).

Engines: 'raster' (the F-SIM raster DDA, field_world.raster_ops.first_stop; host) and 'embree' (the G-SIM mode R
field world: Embree on field_world.sdf + the unobserved clip, the A7 worlds; SIM image).

The map is the SLAM reconstruction built from these same scans: this is a self-consistency check, not a physical
truth comparison (design 7.2 A6: report, keep the baseline, compare later runs with it). Output: aggregates only (no
node pose, no trajectory value, no GNSS).

Development set only (scene_spec.development_directions); each development row pair once.
Inputs: the 10/10 maps folder and the catalogue (closed_loop/external_paths.py), the recorded scans folder
(--scans-dir or UBTS_SCANS_1010; no built-in path).
CLI: python -B -m field_world.real_scan_compare --out <A6.json> [--engine raster|embree --worlds <a7 worlds>]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

from . import raster_ops as ro

SCHEMA = 'ubts.steer_field_world.a6.v1'
SCANS_ENV = 'UBTS_SCANS_1010'
NODE_D_MAX_M = 1.0
NODES_PER_PAIR = 60
MAX_RANGE_M = 30.
UNIVERSAL_EPOCH_TICKS = 719162 * 86400 * 10 ** 7           # Cartographer universal time (100 ns ticks) of 1970-01-01
TICK_HEADER_WINDOW_S = (.05, .25)
RESIDUAL_BANDS_M = (.05, .10, .30, 1.0)


def scans_dir(value=None):
    value = value or os.environ.get(SCANS_ENV, '').strip()
    if not value:
        raise FileNotFoundError('the 10/10 recorded scans folder is not configured: pass --scans-dir or set %s'
                                % SCANS_ENV)
    path = Path(value)
    if not path.is_dir():
        raise FileNotFoundError('the recorded scans folder %s does not exist' % path)
    return path


def load_nodes(record):
    """(node_index, scan_index, x, y, yaw_rad) arrays of the record's SLAM trajectory (map frame; in memory only)."""
    from closed_loop import external_paths
    root = external_paths.configured_path('maps_1010_root', 'dir')
    rows = []
    with open(root / 'offline' / 'maps' / record / 'trajectory.csv', newline='', encoding='utf-8') as handle:
        for item in csv.DictReader(handle):
            if item['scan_index'] in ('', 'None'):
                continue
            rows.append((int(item['node_index']), int(item['scan_index']), float(item['x_m']), float(item['y_m']),
                         math.radians(float(item['yaw_deg']))))
    a = np.array(rows, dtype=float)
    return a[:, 0].astype(int), a[:, 1].astype(int), a[:, 2], a[:, 3], a[:, 4]


def scan_pairing(record, scans):
    """Offset k (run scan i = scans.npz index i + k) and the pairing check."""
    from closed_loop import external_paths
    root = external_paths.configured_path('maps_1010_root', 'dir')
    run = root / 'offline' / 'runs' / ('20261010T%sZ_both' % record) / 'a1'
    custody = json.loads((run / 'CUSTODY.json').read_text(encoding='utf-8'))
    first_header = int(custody['first_last_header_ns']['/scan'][0])
    header = scans['header_ns']
    k = int(np.searchsorted(header, first_header))
    if k >= len(header) or int(header[k]) != first_header:
        raise ValueError('a6_pairing_no_exact_first_header %s' % record)
    ticks = np.array(json.loads((run / 'SCAN_END_TICKS.json').read_text(encoding='utf-8'))['ticks'], dtype=np.int64)
    n = min(len(ticks), len(header) - k)
    gap = ((ticks[:n] - UNIVERSAL_EPOCH_TICKS) * 100 - header[k:k + n]) / 1e9
    ok = (gap >= TICK_HEADER_WINDOW_S[0]) & (gap <= TICK_HEADER_WINDOW_S[1])
    return k, n, dict(offset=k, paired=int(n), within_window=int(ok.sum()),
                      end_minus_header_s=dict(min=float(gap.min()), median=float(np.median(gap)), max=float(gap.max())))


def development_pairs(catalogue):
    from .scene_spec import development_directions
    seen, pairs = set(), []
    for record, corridor, direction, role in development_directions(catalogue):
        if (record, corridor) not in seen:
            seen.add((record, corridor))
            pairs.append((record, corridor, direction, role))
    return pairs


def _stats(values):
    a = np.asarray(values, dtype=float)
    if not a.size:
        return dict(n=0)
    out = dict(n=int(a.size), median=float(np.median(a)), p10=float(np.quantile(a, .1)), p90=float(np.quantile(a, .9)),
               mean=float(a.mean()))
    for band in RESIDUAL_BANDS_M:
        out['abs_le_%g_m' % band] = float(np.mean(np.abs(a) <= band))
    return out


def run(out_path=None, *, engine='raster', scans_folder=None, worlds_dir=None, catalogue=None,
        nodes_per_pair=NODES_PER_PAIR):
    from closed_loop import raster_corridor as RC
    catalogue = catalogue if isinstance(catalogue, dict) else RC.load_catalogue(catalogue)
    folder = scans_dir(scans_folder)
    lut = ro.kind_lut(reflect=RC.REFLECTORS, terminate=(RC.UNOBSERVED,))
    rows = []
    pooled = dict(agree=0, beams=0, real_hit=0, syn_hit=0, both=0, res=[], band_res=[], band_beams=0, band_agree=0)
    scans_cache = {}
    started = time.perf_counter()
    for record, corridor, direction, role in development_pairs(catalogue):
        config = RC.corridor_world_config(record, corridor, direction, catalogue=catalogue)
        world = RC.build_raster_world(config)
        rmap, frame = world.map, world.frame
        if record not in scans_cache:
            path = folder / ('20261010T%sZ_both.scans.npz' % record)
            with np.load(str(path), allow_pickle=False) as data:
                scans = {k: data[k] for k in ('ranges_mm', 'header_ns', 'angle_min', 'angle_increment', 'range_min',
                                              'range_max')}
            scans_cache[record] = (scans, scan_pairing(record, scans), load_nodes(record))
        scans, (offset, paired, pairing), nodes = scans_cache[record]
        node_index, scan_index, mx, my, myaw = nodes
        lx, ly = rmap.map_to_local(mx, my)
        s, d = frame.project_many(lx, ly)
        inside = (s >= 0.) & (s <= config.end_s_m) & (np.abs(d) <= NODE_D_MAX_M) & (scan_index < paired)
        idx = np.flatnonzero(inside)
        if len(idx) > nodes_per_pair:
            idx = idx[np.linspace(0, len(idx) - 1, nodes_per_pair).round().astype(int)]
        beams = scans['ranges_mm'].shape[1]
        angle_min, inc = float(scans['angle_min']), float(scans['angle_increment'])
        r_lo, r_hi = float(scans['range_min']), min(float(scans['range_max']), MAX_RANGE_M)
        local = angle_min + np.arange(beams) * inc
        grid = lut[rmap.classes]
        stats = dict(agree=0, beams=0, real_hit=0, syn_hit=0, both=0, res=[], band_res=[], band_beams=0, band_agree=0)
        if engine == 'embree':
            embree = _embree_world(worlds_dir, record, corridor, direction)
        for i in idx:
            real = scans['ranges_mm'][scan_index[i] + offset].astype(float) / 1000.
            real_hit = (real > 0.) & (real >= r_lo) & (real <= r_hi)
            angles = myaw[i] + local                                   # map frame = raster-local axes (yaw 0)
            if engine == 'raster':
                gx, gy = float(lx[i]) - rmap.col0 * rmap.resolution, float(ly[i]) - rmap.row0 * rmap.resolution
                dist, kind = ro.first_stop(grid, rmap.resolution, gx, gy, np.cos(angles), np.sin(angles), MAX_RANGE_M)
                syn = np.where(kind == ro.REFLECT, dist, np.inf)
            else:
                syn = _embree_scan(embree, float(lx[i]), float(ly[i]), angles)
            syn_hit = np.isfinite(syn) & (syn <= r_hi)
            both = real_hit & syn_hit
            stats['beams'] += beams
            stats['agree'] += int(np.sum(real_hit == syn_hit))
            stats['real_only'] = stats.get('real_only', 0) + int(np.sum(real_hit & ~syn_hit))
            stats['syn_only'] = stats.get('syn_only', 0) + int(np.sum(~real_hit & syn_hit))
            stats['real_hit'] += int(real_hit.sum())
            stats['syn_hit'] += int(syn_hit.sum())
            stats['both'] += int(both.sum())
            residual = real[both] - syn[both]
            stats['res'].extend(residual.tolist())
            # row-band beams: the synthetic hit lies in a row band of the corridor (|d - row d| <= band, inside s)
            hx = float(lx[i]) + syn[syn_hit] * np.cos(angles[syn_hit])
            hy = float(ly[i]) + syn[syn_hit] * np.sin(angles[syn_hit])
            hs, hd = frame.project_many(hx, hy)
            prof = np.asarray(config.profile_s_m, dtype=float)
            near = np.zeros(len(hs), dtype=bool)
            for values in (config.left_row_d_m, config.right_row_d_m):
                near |= np.abs(hd - np.interp(hs, prof, np.asarray(values, dtype=float))) <= config.band_m
            near &= (hs >= 0.) & (hs <= config.end_s_m)
            band_rays = np.flatnonzero(syn_hit)[near]
            stats['band_beams'] += int(len(band_rays))
            stats['band_agree'] += int(np.sum(real_hit[band_rays]))
            band_both = band_rays[real_hit[band_rays]]
            stats['band_res'].extend((real[band_both] - syn[band_both]).tolist())
        row = dict(record=record, corridor_id=corridor, direction=direction, role=role, nodes=int(len(idx)),
                   nodes_inside=int(inside.sum()), pairing=pairing, beams=stats['beams'],
                   agreement=stats['agree'] / max(stats['beams'], 1),
                   real_hit_rate=stats['real_hit'] / max(stats['beams'], 1),
                   synthetic_hit_rate=stats['syn_hit'] / max(stats['beams'], 1),
                   both_hit_rate=stats['both'] / max(stats['beams'], 1),
                   real_only_rate=stats.get('real_only', 0) / max(stats['beams'], 1),
                   synthetic_only_rate=stats.get('syn_only', 0) / max(stats['beams'], 1),
                   residual_m=_stats(stats['res']),
                   row_band=dict(beams=stats['band_beams'], real_return_rate=stats['band_agree'] /
                                 max(stats['band_beams'], 1), residual_m=_stats(stats['band_res'])))
        rows.append(row)
        for key in ('agree', 'beams', 'real_hit', 'syn_hit', 'both', 'band_beams', 'band_agree'):
            pooled[key] += stats[key]
        for key in ('real_only', 'syn_only'):
            pooled[key] = pooled.get(key, 0) + stats.get(key, 0)
        pooled['res'].extend(stats['res'])
        pooled['band_res'].extend(stats['band_res'])
        print('[a6] %s %s nodes %d agreement %.4f residual median %.4f' % (record, corridor, len(idx), row['agreement'],
                                                                        row['residual_m'].get('median', float('nan'))),
              flush=True)
    result = dict(
        schema=SCHEMA, check='A6', engine=engine, judgement='report (self-consistency; baseline for regression)',
        nodes_per_pair=nodes_per_pair, node_rule='0 <= s <= s_E, |d| <= %.1f m, evenly spaced by node order' %
        NODE_D_MAX_M, pairs=len(rows), beams=pooled['beams'], agreement=pooled['agree'] / max(pooled['beams'], 1),
        real_hit_rate=pooled['real_hit'] / max(pooled['beams'], 1),
        synthetic_hit_rate=pooled['syn_hit'] / max(pooled['beams'], 1),
        both_hit_rate=pooled['both'] / max(pooled['beams'], 1),
        real_only_rate=pooled.get('real_only', 0) / max(pooled['beams'], 1),
        synthetic_only_rate=pooled.get('syn_only', 0) / max(pooled['beams'], 1), residual_m=_stats(pooled['res']),
        row_band=dict(beams=pooled['band_beams'], real_return_rate=pooled['band_agree'] / max(pooled['band_beams'], 1),
                      residual_m=_stats(pooled['band_res'])),
        per_pair=rows, wall_s=round(time.perf_counter() - started, 2),
        note='the map was built from these scans (SLAM); not a physical-truth comparison; no node pose is written',
        result_kind='자리값 기반 SIM 결과', field_validated=False)
    result = ro.rounded(result, 6)
    if out_path is not None:
        Path(out_path).write_bytes((json.dumps(result, indent=1, sort_keys=True) + '\n').encode('utf-8'))
    return result


def compare_to_baseline(result, baseline, *, agreement_tol=.005, median_tol=.005):
    """Regression check against a stored baseline: pooled agreement and residual median within the tolerances."""
    da = abs(result['agreement'] - baseline['agreement'])
    dm = abs(result['residual_m']['median'] - baseline['residual_m']['median'])
    return dict(agreement_change=da, residual_median_change_m=dm,
                passed=bool(da <= agreement_tol and dm <= median_tol), tolerances=dict(agreement=agreement_tol,
                                                                                         residual_median_m=median_tol))


# --- embree engine (SIM image) -------------------------------------------------------------------------------------
def _embree_world(worlds_dir, record, corridor, direction):
    from mesh_raycast import StaticVisualWorld
    from steering import scan_unobserved_clip as clip
    from .build_field_world import RasterCopy
    folder = Path(worlds_dir) / ('f%s_%s_%s_r' % (record, corridor, direction))
    raster = RasterCopy(folder / 'raster.npz')
    return dict(world=StaticVisualWorld(folder / 'field_world.sdf', excluded_dynamic_models=()),
                clip=clip.ClipRaster(raster), raster=raster)


def _embree_scan(embree, x_l, y_l, angles_l):
    from steering import scan_unobserved_clip as clip
    t = embree['raster'].transform
    wx, wy = t.to_w(x_l, y_l)
    angles_w = np.asarray(angles_l) - t.theta
    directions = np.column_stack((np.cos(angles_w), np.sin(angles_w), np.zeros(len(angles_w))))
    origins = np.tile((float(wx), float(wy), .6), (len(angles_w), 1))
    locations, index_ray, _ = embree['world'].rays.intersects_location(origins, directions, multiple_hits=False)
    ranges = np.full(len(angles_w), np.inf)
    if len(index_ray):
        ranges[index_ray] = np.linalg.norm(locations - origins[0], axis=1)
    ranges[ranges > MAX_RANGE_M] = np.inf
    out, _ = clip.clip_ranges(embree['clip'], (float(wx), float(wy), .6), directions, ranges)
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description='A6: synthetic scans vs the recorded 10/10 scans (report).')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--engine', choices=('raster', 'embree'), default='raster')
    parser.add_argument('--worlds', type=Path, help='embree: the A7 mode R worlds folder')
    parser.add_argument('--scans-dir', type=Path)
    parser.add_argument('--nodes-per-pair', type=int, default=NODES_PER_PAIR)
    parser.add_argument('--baseline', type=Path, help='compare with a stored A6 result')
    args = parser.parse_args(argv)
    result = run(args.out, engine=args.engine, scans_folder=args.scans_dir, worlds_dir=args.worlds,
                 nodes_per_pair=args.nodes_per_pair)
    summary = {k: result[k] for k in ('engine', 'pairs', 'beams', 'agreement', 'real_hit_rate', 'synthetic_hit_rate',
                                      'residual_m', 'wall_s')}
    if args.baseline:
        summary['baseline'] = compare_to_baseline(result, json.loads(args.baseline.read_text(encoding='utf-8')))
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
