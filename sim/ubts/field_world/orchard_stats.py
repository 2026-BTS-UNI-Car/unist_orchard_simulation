"""10/10 orchard statistics of the development corridors (design 5.1 W3, 8.3 P7; survey worlds_data 6 F2).

Only the development set is read (field_world.scene_spec.development_directions: the frozen 8 and the other
included directions; no sealed direction, no reverse of a sealed row pair). Each development row pair (record,
corridor) is measured once, in its first development direction. Per row side of the pair:

  unit spacing        consecutive catalogue support-unit centres along s (m)
  unit offset         unit centre d - the row centre d of the catalogue profile at that s (m; + toward the aisle)
  unit size           the matched raster component (OCCUPIED cells of the row band, 8-connected after a 1-cell
                      dilation, centroid within 0.06 m of the catalogue unit): cells, extent along s, extent across
                      (cell-square bounding box in the corridor frame), bounding-box diagonal
  row wobble          residual sd of the row centre d profile around a straight line (m) and the largest deviation
                      from the chord (bend, m)
  section width       the 10-90 % width of the band's OCCUPIED cell offsets around the row centre (m)
Per pair: row spacing median (catalogue), free width median (catalogue), centreline curvature (2 m chords).

Output: quantiles and fractions only (no positions, no coordinates). These are statistics of a SLAM
reconstruction ('the orchard as SLAM saw it'); units are occupied components, not trees.

Raster level (2026-11-09 review RW-02): raster_summary(worlds) measures what the scan plane shows, with the same rules
on the data and on generated (W3) worlds: the matched fraction of the units, the band components whose centroid lies
in the units' s range (components per 100 m, the spacing of their centroids along s, their box diagonal), the
one-cell fraction of the matched units, the unit offsets and the offset difference of close unit pairs (along-row
spacing below 0.3 m). DATA_RASTER_REFERENCE holds these numbers of the 22 development pairs;
procedural_comparison(seeds) puts a W3 sample next to them.
CLI: python -B -m field_world.orchard_stats --out <stats.json> [--raster-compare N]   (sim/ubts on sys.path; scipy
     not needed; --raster-compare adds the raster level of the data and of W3 seeds 0..N-1)
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

from . import raster_ops as ro

SCHEMA = 'ubts.steer_field_world.orchard_stats.v1'
QUANTILES = (.01, .05, .1, .25, .5, .75, .9, .95, .99)
UNIT_MATCH_M = .06


def quantile_dict(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not a.size:
        return dict(n=0)
    out = dict(n=int(a.size), mean=float(a.mean()), sd=float(a.std()), min=float(a.min()), max=float(a.max()))
    for q in QUANTILES:
        out['q%02d' % int(round(q * 100))] = float(np.quantile(a, q))
    return out


def _curvature(points, chord=2.):
    pts = np.asarray(points, dtype=float)
    seg = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate(([0.], np.cumsum(seg)))
    grid = np.arange(0., s[-1], .5)
    x, y = np.interp(grid, s, pts[:, 0]), np.interp(grid, s, pts[:, 1])
    k = int(round(chord / .5))
    if len(grid) <= 2 * k:
        return np.zeros(0)
    a = np.column_stack((x[:-2 * k], y[:-2 * k]))
    b = np.column_stack((x[k:-k], y[k:-k]))
    c = np.column_stack((x[2 * k:], y[2 * k:]))
    ab, bc, ca = (np.hypot(*(p - q).T) for p, q in ((a, b), (b, c), (c, a)))
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    return 2. * np.abs(cross) / np.maximum(ab * bc * ca, 1e-12)


def side_measurements(world, side):
    """Unit spacing / offset / size, row wobble and section width of one row side of a raster corridor world."""
    from closed_loop import raster_corridor as RC
    config, frame, rmap = world.config, world.frame, world.map
    units = list(config.left_units if side == 'left' else config.right_units)
    out = dict(spacing=[], offset=[], cells=[], along=[], across=[], diag=[], matched=0, units=len(units))
    if len(units) < 2:
        return out
    s_units = np.array([u[0] for u in units])
    d_units = np.array([u[1] for u in units])
    prof_s = np.asarray(config.profile_s_m, dtype=float)
    row_d = np.asarray(config.left_row_d_m if side == 'left' else config.right_row_d_m, dtype=float)
    inward = -1. if side == 'left' else 1.
    out['spacing'] = np.diff(np.sort(s_units)).tolist()
    out['offset'] = (inward * (d_units - np.interp(s_units, prof_s, row_d))).tolist()
    rows, cols, s, d = RC._band_cells(rmap, frame, config, side, s_units.min() - 3., s_units.max() + 3., extra=.3,
                                      where=(RC.OCCUPIED,))
    if len(rows):
        rel = d - np.interp(s, prof_s, row_d)
        out['section_width'] = float(np.quantile(rel, .9) - np.quantile(rel, .1))
    labels = ro.label_components(rows, cols)
    comps = {}
    for i, lab in enumerate(labels):
        comps.setdefault(int(lab), []).append(i)
    centroids = []
    for lab, idx in comps.items():
        idx = np.asarray(idx)
        x, y = rmap.centres(rows[idx], cols[idx])
        cs, cd, _ = frame.project(float(x.mean()), float(y.mean()))
        centroids.append((cs, cd, idx))
    for us, ud in units:
        best = min(centroids, key=lambda c: (c[0] - us) ** 2 + (c[1] - ud) ** 2) if centroids else None
        if best is None or abs(best[0] - us) > UNIT_MATCH_M or abs(best[1] - ud) > UNIT_MATCH_M:
            continue
        idx = best[2]
        cs, cd = s[idx], d[idx]
        res = rmap.resolution
        along = float(cs.max() - cs.min()) + res
        across = float(cd.max() - cd.min()) + res
        out['matched'] += 1
        out['cells'].append(int(len(idx)))
        out['along'].append(along)
        out['across'].append(across)
        out['diag'].append(math.hypot(along, across))
    windows = np.asarray(config.profile_s_m, dtype=float)
    lo, hi = s_units.min(), s_units.max()
    inside = (windows >= lo) & (windows <= hi)
    if inside.sum() >= 5:
        ws, wd = windows[inside], row_d[inside]
        fit = np.polyfit(ws, wd, 1)
        out['wobble_sd'] = float(np.std(wd - np.polyval(fit, ws)))
        chord = wd[0] + (wd[-1] - wd[0]) * (ws - ws[0]) / max(ws[-1] - ws[0], 1e-9)
        out['bend'] = float(np.max(np.abs(wd - chord)))
        out['row_length'] = float(hi - lo)
    return out


def component_measurements(world, side):
    """Band components of one row side whose centroid lies in the units' s range: (centroid s sorted, box diagonal per
    component, cells per component, s range length)."""
    from closed_loop import raster_corridor as RC
    config, frame, rmap = world.config, world.frame, world.map
    units = list(config.left_units if side == 'left' else config.right_units)
    if len(units) < 2:
        return np.zeros(0), np.zeros(0), np.zeros(0, dtype=int), 0.
    s_units = np.array([u[0] for u in units])
    lo, hi = float(s_units.min()), float(s_units.max())
    rows, cols, s, d = RC._band_cells(rmap, frame, config, side, lo - 3., hi + 3., extra=.3, where=(RC.OCCUPIED,))
    if not len(rows):
        return np.zeros(0), np.zeros(0), np.zeros(0, dtype=int), hi - lo
    labels = ro.label_components(rows, cols)
    centres, diags, sizes = [], [], []
    res = rmap.resolution
    for lab in np.unique(labels):
        idx = labels == lab
        x, y = rmap.centres(rows[idx], cols[idx])
        c_s = frame.project(float(x.mean()), float(y.mean()))[0]
        if lo <= c_s <= hi:
            centres.append(c_s)
            along = float(s[idx].max() - s[idx].min()) + res
            across = float(d[idx].max() - d[idx].min()) + res
            diags.append(math.hypot(along, across))
            sizes.append(int(idx.sum()))
    order = np.argsort(centres)
    return (np.asarray(centres)[order], np.asarray(diags)[order], np.asarray(sizes, dtype=int)[order], hi - lo)


def raster_summary(worlds):
    """Raster-level statistics pooled over raster corridor worlds (module docstring); quantiles and rates only."""
    pooled = dict(comp_spacing=[], comp_diag=[], cells=[], offset=[], close_pair_doffset=[])
    units = matched = components = 0
    length = 0.
    for world in worlds:
        for side in ('left', 'right'):
            m = side_measurements(world, side)
            units += m['units']
            matched += m['matched']
            pooled['cells'] += list(m['cells'])
            pooled['offset'] += list(m['offset'])
            centres, diags, sizes, span = component_measurements(world, side)
            components += max(len(centres) - 1, 0)
            pooled['comp_spacing'] += np.diff(centres).tolist()
            pooled['comp_diag'] += diags.tolist()
            length += span
            config = world.config
            side_units = sorted(config.left_units if side == 'left' else config.right_units)
            if len(side_units) > 2:
                prof_s = np.asarray(config.profile_s_m, dtype=float)
                row_d = np.asarray(config.left_row_d_m if side == 'left' else config.right_row_d_m, dtype=float)
                su = np.array([u[0] for u in side_units])
                du = np.array([u[1] for u in side_units]) - np.interp(su, prof_s, row_d)
                gaps = np.diff(su)
                pooled['close_pair_doffset'] += np.abs(np.diff(du))[gaps < .3].tolist()
    cells = np.asarray(pooled['cells'])
    return dict(units=units, matched=matched, matched_fraction=matched / max(units, 1),
                components_per_100m=100. * components / max(length, 1e-9),
                one_cell_fraction=float((cells == 1).mean()) if cells.size else None,
                comp_spacing_m=quantile_dict(pooled['comp_spacing']), comp_diag_m=quantile_dict(pooled['comp_diag']),
                unit_offset_inward_m=quantile_dict(pooled['offset']),
                close_pair_abs_doffset_m=quantile_dict(pooled['close_pair_doffset']), row_length_m=length,
                rule='orchard_stats.side_measurements + band components (1-cell dilation, 8-connected) with the '
                     'centroid in the s range of the units')


# The raster level of the 22 development pairs (raster_summary over the catalogue worlds, 2026-11-09; quantiles and
# rates only; the numbers of runs/wf3b/fix_wf3b/logs/ORCHARD_STATS_RASTER.json rounded). Used by the W3 test and by
# procedural_comparison.
DATA_RASTER_REFERENCE = dict(
    matched_fraction=.999, components_per_100m=131.5, one_cell_fraction=.164,
    comp_spacing_m=dict(q10=.168, q25=.334, q50=.571, q75=.918, q90=1.394),
    comp_diag_m=dict(q10=.071, q25=.116, q50=.214, q75=.335, q90=.551),
    unit_offset_inward_m=dict(q05=-.427, q10=-.256, q25=-.051, q50=0., q75=.045, q90=.226, q95=.391),
    close_pair_abs_doffset_m=dict(q10=.181, q25=.287, q50=.415, q75=.565, q90=.712))


def procedural_comparison(seeds, params=None):
    """raster_summary of W3 worlds (seeds, the given parameters) next to DATA_RASTER_REFERENCE."""
    from steering import procedural_orchard as po
    worlds = [po.build_raster_world(dict(params or {}, seed=int(seed)))[1] for seed in seeds]
    return dict(procedural=raster_summary(worlds), data=DATA_RASTER_REFERENCE, seeds=list(seeds),
                params=dict(params or {}))


def compute(catalogue=None):
    """The statistics dict (see the module docstring)."""
    from closed_loop import raster_corridor as RC
    from .scene_spec import development_directions
    catalogue = catalogue if isinstance(catalogue, dict) else RC.load_catalogue(catalogue)
    seen, pairs = set(), []
    for record, corridor, direction, role in development_directions(catalogue):
        if (record, corridor) in seen:
            continue
        seen.add((record, corridor))
        pairs.append((record, corridor, direction, role))
    pooled = {k: [] for k in ('spacing', 'offset', 'cells', 'along', 'across', 'diag')}
    wobble, bend, section, row_length, spacing_pair, free_width, curvature = [], [], [], [], [], [], []
    gap_rate, gap_long_rate, matched, units = [], [], 0, 0
    for record, corridor, direction, role in pairs:
        config = RC.corridor_world_config(record, corridor, direction, catalogue=catalogue)
        world = RC.build_raster_world(config)
        entry = RC.find_entry(catalogue, record, corridor)
        spacing_pair.append(float(entry['spacing_median_m']))
        free_width.append(float(entry['free_width_median_m']))
        curvature.extend(_curvature(config.polyline).tolist())
        for side in ('left', 'right'):
            m = side_measurements(world, side)
            for key in pooled:
                pooled[key].extend(m[key])
            matched += m['matched']
            units += m['units']
            for key, sink in (('wobble_sd', wobble), ('bend', bend), ('section_width', section),
                              ('row_length', row_length)):
                if key in m:
                    sink.append(m[key])
            if m['spacing'] and m.get('row_length'):
                sp = np.asarray(m['spacing'])
                gap_rate.append(float((sp > 2.5).sum() / m['row_length'] * 100.))
                gap_long_rate.append(float((sp > 6.).sum() / m['row_length'] * 100.))
    return dict(
        schema=SCHEMA, source='single-row catalogue labels + 10/10 offline raster (SLAM reconstruction)',
        development_pairs=[dict(record=r, corridor_id=c, direction=d, role=role) for r, c, d, role in pairs],
        unit_matching=dict(units=units, matched=matched, rule='component centroid within %.2f m' % UNIT_MATCH_M),
        unit_spacing_m=quantile_dict(pooled['spacing']), unit_offset_inward_m=quantile_dict(pooled['offset']),
        unit_cells=quantile_dict(pooled['cells']), unit_along_m=quantile_dict(pooled['along']),
        unit_across_m=quantile_dict(pooled['across']), unit_diag_m=quantile_dict(pooled['diag']),
        unit_along_across_correlation=float(np.corrcoef(pooled['along'], pooled['across'])[0, 1])
        if len(pooled['along']) > 2 else None,
        row_wobble_sd_m=quantile_dict(wobble), row_bend_m=quantile_dict(bend),
        row_section_width_10_90_m=quantile_dict(section), row_length_m=quantile_dict(row_length),
        row_spacing_median_m=quantile_dict(spacing_pair), free_width_median_m=quantile_dict(free_width),
        centreline_curvature_1_m=quantile_dict(curvature),
        gaps_gt_2p5_per_100m=quantile_dict(gap_rate), gaps_gt_6_per_100m=quantile_dict(gap_long_rate),
        note='statistics of occupied components of a SLAM map (units are not trees); quantiles only, no positions')


def main(argv=None):
    parser = argparse.ArgumentParser(description='10/10 development-corridor statistics for the procedural orchard.')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--raster-compare', type=int, default=0,
                        help='add the raster level of the data and of W3 seeds 0..N-1')
    args = parser.parse_args(argv)
    from steering import scene_spec as steer_scene
    if steer_scene.inside_repository(args.out):
        parser.error('the output file is inside the repository (design 4.5)')
    stats = compute()
    if args.raster_compare:
        from closed_loop import raster_corridor as RC
        from .scene_spec import development_directions
        catalogue = RC.load_catalogue(None)
        seen, worlds = set(), []
        for record, corridor, direction, role in development_directions(catalogue):
            if (record, corridor) not in seen:
                seen.add((record, corridor))
                worlds.append(RC.build_raster_world(RC.corridor_world_config(record, corridor, direction,
                                                                            catalogue=catalogue)))
        stats['raster_level'] = dict(data=raster_summary(worlds),
                                     procedural=procedural_comparison(range(args.raster_compare))['procedural'])
    args.out.write_bytes((json.dumps(ro.rounded(stats, 5), indent=1, sort_keys=True) + '\n').encode('utf-8'))
    print(json.dumps({k: stats[k] for k in ('unit_matching', 'unit_spacing_m', 'unit_diag_m')}, indent=1)[:2000])
    return 0


if __name__ == '__main__':
    sys.exit(main())
