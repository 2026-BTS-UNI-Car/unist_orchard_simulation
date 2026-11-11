"""Re-rasterisation closure of a field world (design 8.3 P7 '다시 래스터화해 행 선·중심선·끝이 원래 카탈로그와 맞는지').

From the written outputs only (field_world.sdf, raster.npz, eval_truth.json, manifest.json):

  cells        the scan-plane geometry of the SDF (z = PLANE_Z_M) is rasterised back onto the raster copy's grid
               (box: the cells its rectangle covers, exact for mode R boxes that are cell unions; cylinder: the cells
               whose centre lies inside the circle) and compared per cell with the raster copy's reflector cells
               (OCCUPIED, ADDED_HARD, ADDED_BRANCH): missing = reflector cells not covered, extra = covered cells
               that are not reflectors. Mode R must give 0 / 0.
  units        the catalogue support units are found again in the re-rasterised OCCUPIED cells (8-connected after a
               1-cell dilation, centroid within 0.06 m of the unit centre: the catalogue / raster_corridor rule):
               matched per side, compared with the count matched in the raster copy itself
  row end      s_E recomputed as the larger of the two rows' last matched unit s (fwd rule; the eval_truth
               end for this direction is compared) and the line ends
  centreline   eval_truth's centreline (W) taken back to raster-local metres and compared with the corridor polyline
               of the F-SIM world config (the catalogue polyline): max deviation
  row lines    eval_truth's row lines (W) back in the corridor frame: d at the profile windows vs the catalogue profile

CLI: python -B -m field_world.closure <field world folder> ... [--out <json>]   (sim/ubts on sys.path)
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as E

import numpy as np

from . import raster_ops as ro

SCHEMA = 'ubts.steer_field_world.closure.v1'
PLANE_Z_M = .6
UNIT_MATCH_M = .06


def sdf_parts(path):
    """Visual geometry rows of the field models: (link, shape, x, y, z, yaw, dims) in W."""
    root = E.parse(str(path)).getroot()
    out = []
    for model in root.iter('model'):
        name = model.get('name', '')
        if not name.startswith('field_') or name == 'field_ground':
            continue
        for link in model.findall('link'):
            for visual in link.findall('visual'):
                pose = [float(v) for v in visual.findtext('pose').split()]
                geometry = visual.find('geometry')
                box, cylinder = geometry.find('box'), geometry.find('cylinder')
                if box is not None:
                    dims = [float(v) for v in box.findtext('size').split()]
                    out.append((link.get('name'), 'box', pose[0], pose[1], pose[2], pose[5], dims))
                elif cylinder is not None:
                    out.append((link.get('name'), 'cylinder', pose[0], pose[1], pose[2], pose[5],
                                [float(cylinder.findtext('radius')), float(cylinder.findtext('length'))]))
    return out


OCCUPIED_LINKS = ('occupied', 'trunk', 'trunk_cells', 'canopy', 'structure')


def rasterise_plane(parts, raster, z=PLANE_Z_M, links=None):
    """Boolean mask (raster copy grid) of the cells the parts (of the given links; None = all) cover in the plane z."""
    res, row0, col0 = raster.resolution, raster.row0, raster.col0
    h, w = raster.classes.shape
    mask = np.zeros((h, w), dtype=bool)
    theta = raster.transform.theta
    for link, shape, x, y, zc, yaw, dims in parts:
        if links is not None and link not in links:
            continue
        length = dims[2] if shape == 'box' else dims[1]
        if not (zc - length / 2. - 1e-9 <= z <= zc + length / 2. + 1e-9):
            continue
        cx, cy = raster.transform.to_l(x, y)
        cx, cy = float(cx), float(cy)
        if shape == 'box' and abs(math.atan2(math.sin(yaw + theta), math.cos(yaw + theta))) < 1e-6:
            c0 = int(round((cx - dims[0] / 2.) / res)) - col0
            c1 = int(round((cx + dims[0] / 2.) / res)) - col0
            r0 = int(round((cy - dims[1] / 2.) / res)) - row0
            r1 = int(round((cy + dims[1] / 2.) / res)) - row0
            mask[max(r0, 0):min(r1, h), max(c0, 0):min(c1, w)] = True
            continue
        if shape == 'box':                         # a rotated box (upper canopy is out of the plane): centre test
            reach = math.hypot(dims[0], dims[1]) / 2. + res
            heading = yaw + theta
            ax, ay = math.cos(heading), math.sin(heading)
        else:
            reach = dims[0] + res
        c0, c1 = int(math.floor((cx - reach) / res)) - col0, int(math.floor((cx + reach) / res)) - col0
        r0, r1 = int(math.floor((cy - reach) / res)) - row0, int(math.floor((cy + reach) / res)) - row0
        cc, rr = np.meshgrid(np.arange(max(c0, 0), min(c1 + 1, w)), np.arange(max(r0, 0), min(r1 + 1, h)))
        ex = (cc + col0 + .5) * res - cx
        ey = (rr + row0 + .5) * res - cy
        if shape == 'box':
            inside = (np.abs(ex * ax + ey * ay) <= dims[0] / 2.) & (np.abs(-ex * ay + ey * ax) <= dims[1] / 2.)
        else:
            inside = ex * ex + ey * ey <= dims[0] * dims[0]
        mask[rr[inside], cc[inside]] = True
    return mask


def match_units(rows, cols, raster, frame, units):
    """How many catalogue units (s, d) have a component of these cells with its centroid within UNIT_MATCH_M."""
    if not units:
        return 0, []
    labels = ro.label_components(rows, cols)
    table = ro.component_table(rows, cols, labels, raster.resolution, raster.row0, raster.col0)
    if not table:
        return 0, []
    cx = np.array([c['cx'] for c in table])
    cy = np.array([c['cy'] for c in table])
    s, d = frame.project_many(cx, cy)
    matched = []
    for us, ud in units:
        k = int(np.argmin((s - us) ** 2 + (d - ud) ** 2))
        if abs(s[k] - us) <= UNIT_MATCH_M and abs(d[k] - ud) <= UNIT_MATCH_M:
            matched.append(float(us))
    return len(matched), matched


def closure(folder, *, catalogue=None):
    """Closure report of one written field world folder (catalogue source)."""
    from closed_loop import raster_corridor as RC
    from .build_field_world import RasterCopy
    folder = Path(folder)
    manifest = json.loads((folder / 'manifest.json').read_text(encoding='utf-8'))
    truth = json.loads((folder / 'eval_truth.json').read_text(encoding='utf-8'))
    raster = RasterCopy(folder / 'raster.npz')
    spec = manifest['spec']
    parts = sdf_parts(folder / 'field_world.sdf')
    plane = rasterise_plane(parts, raster)
    reflect = np.isin(raster.classes, RC.REFLECTORS)
    out = dict(schema=SCHEMA, name=manifest['name'], mode=manifest['mode'], parts=len(parts),
               cells=dict(reflector=int(reflect.sum()), covered=int(plane.sum()),
                          missing=int(np.sum(reflect & ~plane)), extra=int(np.sum(plane & ~reflect))))
    if spec['source'] != 'catalogue':
        return out
    catalogue = catalogue if isinstance(catalogue, dict) else RC.load_catalogue(catalogue)
    config = RC.corridor_world_config(spec['record'], spec['corridor_id'], spec['direction'], catalogue=catalogue)
    frame = RC.CurvilinearFrame(config.polyline, config.s_origin_m)
    units = {}
    occupied_plane = rasterise_plane(parts, raster, links=OCCUPIED_LINKS)
    for side, catalogue_units in (('left', config.left_units), ('right', config.right_units)):
        rr, cc = np.nonzero(occupied_plane)
        rr0, cc0 = np.nonzero(raster.classes == RC.OCCUPIED)
        n_re, s_re = match_units(rr, cc, raster, frame, catalogue_units)
        n_or, s_or = match_units(rr0, cc0, raster, frame, catalogue_units)
        units[side] = dict(catalogue=len(catalogue_units), matched_reraster=n_re, matched_raster_copy=n_or,
                           last_s_reraster=max(s_re) if s_re else None, last_s_raster_copy=max(s_or) if s_or else None,
                           first_s_reraster=min(s_re) if s_re else None)
    last = [units[s]['last_s_reraster'] for s in ('left', 'right')]
    end_re = max(last) if None not in last else None
    end_truth = truth['s_end_m']
    poly = np.asarray(config.polyline, dtype=float)
    cw = np.asarray(truth['centreline']['points_w'], dtype=float)
    lx, ly = raster.transform.to_l(cw[:, 0], cw[:, 1])
    centre_dev = float(np.max(np.hypot(lx - poly[:, 0], ly - poly[:, 1]))) if len(poly) == len(cw) else None
    row_dev = {}
    prof = np.asarray(config.profile_s_m, dtype=float)
    for side, values in (('left', config.left_row_d_m), ('right', config.right_row_d_m)):
        line = np.asarray(truth['row_lines'][side]['line_w'], dtype=float)
        x, y = raster.transform.to_l(line[:, 0], line[:, 1])
        s, d = frame.project_many(x, y)
        row_dev[side] = float(np.max(np.abs(d - np.interp(s, prof, np.asarray(values, dtype=float)))))
    out.update(units=units, row_end=dict(s_end_reraster_m=end_re, s_end_eval_truth_m=end_truth,
                                         s_end_catalogue_config_m=float(config.end_s_m),
                                         rule='fwd: max(last matched unit s of each row); rev: the reverse field frame '
                                              'of the same rule'),
               centreline_max_deviation_m=centre_dev, row_line_max_d_deviation_m=row_dev,
               row_line_note='report: eval_truth row lines (W, 4 decimals) projected back on the piecewise-linear '
                             'corridor frame; near a polyline vertex the closest-segment projection of a point at '
                             '|d| ~ 1.8 m moves by up to |d| x the vertex turn (frame round trip, not a world change)',
               passed=bool(out['cells']['missing'] == 0 and out['cells']['extra'] == 0
                           and all(units[s]['matched_reraster'] == units[s]['matched_raster_copy'] for s in units)
                           and end_re is not None and abs(end_re - end_truth) <= .011 and centre_dev is not None
                           and centre_dev <= 1e-3)
               if manifest['mode'] == 'R' else None,
               rule='mode R: cells exact, every unit matched as in the raster copy, row end within 0.011 m (units are '
                    'rounded to 0.01 m in the catalogue), centreline within 1 mm; mode T: report')
    return ro.rounded(out, 6)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Re-rasterisation closure of field worlds.')
    parser.add_argument('folders', nargs='+', type=Path)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args(argv)
    rows = []
    for folder in args.folders:
        row = closure(folder)
        rows.append(row)
        print('%s cells missing %d extra %d units %s end %s passed %s' % (
            row['name'], row['cells']['missing'], row['cells']['extra'],
            {k: (v['matched_reraster'], v['matched_raster_copy'], v['catalogue']) for k, v in row.get('units',
                                                                                                    {}).items()},
            row.get('row_end', {}).get('s_end_reraster_m'), row.get('passed')))
    if args.out:
        args.out.write_bytes((json.dumps(dict(schema=SCHEMA, rows=rows), indent=1) + '\n').encode('utf-8'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
