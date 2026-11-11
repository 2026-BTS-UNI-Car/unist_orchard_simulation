"""Corridor catalogue of the 10/10 offline maps for the single-row study (SIM only, no truth to the controller).

Selection rule (fixed; the same text is pre-registered in single-row-20261102/PREREG.json):
  'every corridor between two adjacent row lines of the record's major family with both lines >= 25 m and
  overlap >= 20 m'
Implementation (rows_offline_<id>.json of maps-1010-20261022/quality/rows, map frame -> raster-local metres):
  major family   = the family with the largest length share (families[k].share). Other families flagged
                   major=True are reported per record, not used.
  row lines      = groups with row=True of that family; line length = group length_m.
  pair           = two lines whose s-ranges, projected on the pair's mean axis, overlap by >= 20 m and whose
                   offsets at the middle of the overlap differ by >= 1.5 m (0.3-1.5 m apart = images of the same
                   row, the row analysis' near-parallel class).
  adjacent       = no other row line of the family overlapping that interval by >= 5 m lies between the two at
                   >= 1.5 m from both.
Labels (raster OCCUPIED cells, closed_loop.raster_corridor class rule):
  row units      = occupied components (8-connected after a 1-cell dilation, as rows.py) whose centroid lies
                   within band = min(0.75 m, 0.3 x separation) of a line (extrapolated beyond its ends), whose
                   cells span <= 1.2 m across the row, and that are not within 1 m (along s) of a cross structure
                   (a 1 m slice in which occupied cells cover >= 40 % of the aisle interior, relative offset
                   0.2-0.8 between the lines, 12 bins: fence, hedge or wall across the aisle).
  side support   = chains of row units with gaps (unit extent to unit extent) <= 6 m (the row analysis split
                   gap) and no cross structure between consecutive units; the chains of >= 3 units meeting the
                   lines' overlap form that row's support.
  start s0 (fwd) = both rows begin = max(first unit centre of each row); end (fwd) = last trunk on BOTH rows =
                   max(last unit centre of each row). Reverse direction: start = min(last), end = min(first).
  centreline     = mean of the two rows' local midlines: per 1 m window the median offset of the row's support
                   cells (>= 3 cells) or the line fit where a side is missing, Gaussian-smoothed (sigma 2 m),
                   resampled every 0.25 m, extended 8 m before the first and 15 m beyond the last row unit.
  gaps           = consecutive row-unit centres > 2.5 m apart (the row analysis gap threshold).
Exclusion rules (finalised while reviewing the PNGs of the candidate pairs, before any closed-loop run; applied
uniformly to every pair; every exclusion is reported with its rule and value). Corridor level:
  X_nosupport    a row has no raster support chain on the overlap (labels cannot be formed)
  X_width        median row-centre spacing (two-sided part) outside [0.6, 1.6] x the record median neighbour
                 spacing (not unambiguously one aisle: a missed row, a road or another block may lie between)
  X_short        the raster support of both rows overlaps by < 20 m (min(last) - max(first)): shorter than the
                 selection overlap, the line pair is not borne out by the raster rows
  X_dup          the same aisle as another selected pair (one line shared, the other lines < 1.5 m apart); the
                 pair with the longer overlap is kept
Direction level (row interval [start, end] of that travel direction):
  X_between      another row line of the family (overlap >= 5 m with the interval) lies between the two rows at
                 >= 1.5 m from both (adjacency re-checked over the whole interval, not only the line overlap)
  X_midrow       occupied cells within |d| <= 0.5 m in >= 25 % of its 1 m windows (a row or structure along the
                 middle)
  X_unobserved   > 5 % UNOBSERVED cells in the drivable band |d| <= 0.6 m (interior not mapped)
  X_blocked      the free width (inner faces of the occupied cells between the rows) is < 1.0 m (vehicle 0.70 m +
                 2 x 0.15 m) in a 1 m window whose centre lies in [start + 0.61, end - 0.61] (footprint centre
                 positions with the whole vehicle inside the row interval): not traversable
  X_crossbreak   both rows have a > 6 m gap at the same place inside the interval (cross alley or block break:
                 one row or two is ambiguous)
  X_end_resumes  on both rows a row chain that is not part of the row's support (>= 3 units spanning >= 3 m
                 along s; separated from the support by > 6 m or a cross structure) starts within 20 m beyond the
                 end (cross alley or block boundary: the row end is ambiguous)
  X_end_stagger  the two rows' last units at the end are > 6 m apart along s (one row continues alone for longer
                 than a row gap: aisle end vs 'both rows ended' ambiguous)
Not exclusions (flags): directions.<dir>.centreline_clear / centreline_hard_contact_intervals_m (an occupied
cell within the nominal footprint centred on the exact centreline, e.g. a small blob 0.37 m off-centre: the aisle
is still wide enough), interior uncertain fractions (corridors seen only from neighbouring aisles), trajectory
coverage (offline SLAM nodes; attribute only).

Usage: python -B -m closed_loop.raster_catalogue --out <corridors output folder>
Inputs (no built-in local path; closed_loop/external_paths.py, docs/steering/PATHS_KO.md): the 10/10 maps folder
(UBTS_MAPS_1010, or key 'maps_1010_root' of the JSON file named by UBTS_STEER_PATHS) and the row analysis folder
(UBTS_ROWS_1010, or key 'rows_root' of that file).
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

from . import external_paths
from . import raster_corridor as RC

SELECTION_RULE = ("every corridor between two adjacent row lines of the record's major family with both lines "
                  ">= 25 m and overlap >= 20 m")
MIN_LINE_M, MIN_OVERLAP_M = 25., 20.
SAME_ROW_M = 1.5                      # lines closer than this are images of one row
BETWEEN_OVERLAP_M = 5.
BAND_MAX_M, BAND_FRACTION = .75, .3
UNIT_MAX_ACROSS_M = 1.2               # a row unit (trunk + canopy section) spans <= 1.2 m across the row
CROSS_SLICE_M, CROSS_BINS, CROSS_COVERAGE, CROSS_NEAR_M = 1., 12, .4, 1.
MIN_CHAIN_UNITS = 3                   # a row is >= 3 units; shorter chains are clutter in the band
RESUME_MIN_EXTENT_M = 3.              # a resumed row runs along s (a wall crossing the band does not)
MIN_FREE_WIDTH_M = 1.                 # vehicle width 0.70 + 2 x 0.15 m
END_STAGGER_M = 6.                    # = CHAIN_GAP_M: one row continues alone for longer than a row gap
CHAIN_GAP_M = 6.                      # rows.py RANSAC segment split gap
GAP_REPORT_M = 2.5                    # rows.py 'gaps_gt_2p5m'
EXTEND_BEFORE_M, EXTEND_AFTER_M = 8., 15.
SMOOTH_SIGMA_M = 2.
FOOTPRINT = (1.22, .70)               # field_response catalogue footprint (length, width)
DRIVABLE_HALF_M = .6
RULES = dict(width_factor=(.6, 1.6), midrow_band_m=.5, midrow_window_fraction=.25, unobserved_max_fraction=.05,
             resume_window_m=(0., 20.), resume_min_chains=1, crossbreak_gap_m=CHAIN_GAP_M)
EXCLUSION_TEXT = dict(
    X_nosupport='corridor: a row has no raster support chain on the overlap (labels cannot be formed)',
    X_width='corridor: median row-centre spacing outside [0.6, 1.6] x the record median neighbour spacing',
    X_midrow='direction: occupied cells within |d| <= 0.5 m in >= 25 % of the 1 m windows of its interval',
    X_unobserved='direction: > 5 % UNOBSERVED cells in |d| <= 0.6 m over its interval',
    X_blocked='direction: free width < 1.0 m in a 1 m window centred in [start + 0.61, end - 0.61]',
    X_crossbreak='direction: both rows have a > 6 m gap at the same place inside its interval',
    X_dup='same aisle as another selected pair (shared line, other lines < 1.5 m apart); longer overlap kept',
    X_short='raster support of both rows overlaps by < 20 m',
    X_between='direction: another family row line lies between the two rows over its interval',
    X_end_resumes='direction: on both rows a separate row chain (>= 3 units, >= 3 m along s) starts within 20 m beyond the end',
    X_end_stagger='direction only: the rows\' last units at that end are > 6 m apart along s')


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows_path(record):
    """rows_offline_<record>.json in the row analysis folder (UBTS_ROWS_1010, else 'rows_root' of the
    UBTS_STEER_PATHS file). Raises FileNotFoundError instead of guessing a path."""
    path = external_paths.configured_path('rows_root', 'dir') / ('rows_offline_%s.json' % record)
    if not path.is_file():
        raise FileNotFoundError('rows_offline_%s.json not in the row analysis folder: %s' % (record, path))
    return path


def trajectory_path(record):
    return Path(RC.maps_root()) / 'offline' / 'maps' / record / 'trajectory.csv'


# --- lines and pairs ---------------------------------------------------------------------------------------

def major_family(rows):
    shares = [f['share'] for f in rows['families']]
    return int(np.argmax(shares))


def row_lines(rows, rmap, family):
    """Major-family row lines in raster-local metres: endpoints a, b (a -> b along the fitted direction)."""
    lines = []
    for g in rows['groups']:
        if not g['row'] or g['family'] != family:
            continue
        c, d = np.asarray(g['line_c'], float), np.asarray(g['line_d'], float)
        d = d / np.hypot(*d)
        a = np.array(rmap.map_to_local(*(c + d * g['s_range'][0])))
        b = np.array(rmap.map_to_local(*(c + d * g['s_range'][1])))
        lines.append(dict(id=int(g['id']), length=float(g['length_m']), angle=float(g['angle_deg']), a=a, b=b,
                          d=(b - a) / np.hypot(*(b - a)), trunks=(g.get('trunk_spacing') or {}).get('trunks')))
    return lines


class PairFrame:
    """Straight frame of a line pair: u = mean direction (angle in [0, 180) deg), n = left normal."""

    def __init__(self, line_i, line_j):
        di, dj = line_i['d'], line_j['d']
        if np.dot(di, dj) < 0:
            dj = -dj
        u = di + dj
        u /= np.hypot(*u)
        if u[1] < 0 or (u[1] == 0 and u[0] < 0):
            u = -u
        self.u, self.n = u, np.array((-u[1], u[0]))
        self.ref = (line_i['a'] + line_i['b']) / 2

    def sd(self, xy):
        xy = np.asarray(xy, float) - self.ref
        return xy @ self.u, xy @ self.n

    def xy(self, s, o):
        s, o = np.asarray(s, float), np.asarray(o, float)
        return self.ref[0] + s * self.u[0] + o * self.n[0], self.ref[1] + s * self.u[1] + o * self.n[1]

    def interval(self, line):
        sa, _ = self.sd(line['a'])
        sb, _ = self.sd(line['b'])
        return min(sa, sb), max(sa, sb)

    def offset(self, line, s):
        """Offset of the (extrapolated) line at s."""
        sa, oa = self.sd(line['a'])
        sb, ob = self.sd(line['b'])
        return oa + (ob - oa) * (np.asarray(s, float) - sa) / (sb - sa)


def select_pairs(lines):
    """Pairs satisfying the selection rule (see the module doc), with the frame and overlap."""
    pairs = []
    long_lines = [l for l in lines if l['length'] >= MIN_LINE_M]
    for a in range(len(long_lines)):
        for b in range(a + 1, len(long_lines)):
            li, lj = long_lines[a], long_lines[b]
            frame = PairFrame(li, lj)
            (lo_i, hi_i), (lo_j, hi_j) = frame.interval(li), frame.interval(lj)
            lo, hi = max(lo_i, lo_j), min(hi_i, hi_j)
            if hi - lo < MIN_OVERLAP_M:
                continue
            mid = (lo + hi) / 2
            oi, oj = float(frame.offset(li, mid)), float(frame.offset(lj, mid))
            if abs(oi - oj) < SAME_ROW_M:
                continue
            between = []
            for lk in lines:
                if lk['id'] in (li['id'], lj['id']):
                    continue
                lo_k, hi_k = frame.interval(lk)
                k_lo, k_hi = max(lo_k, lo), min(hi_k, hi)
                if k_hi - k_lo < BETWEEN_OVERLAP_M:
                    continue
                sk = (k_lo + k_hi) / 2
                ok, oik, ojk = float(frame.offset(lk, sk)), float(frame.offset(li, sk)), float(frame.offset(lj, sk))
                if min(oik, ojk) < ok < max(oik, ojk) and min(abs(ok - oik), abs(ok - ojk)) >= SAME_ROW_M:
                    between.append(lk['id'])
            if between:
                continue
            left, right = (li, lj) if oi > oj else (lj, li)
            pairs.append(dict(frame=frame, left=left, right=right, overlap=(lo, hi), separation=abs(oi - oj),
                              angle_between_deg=round(math.degrees(math.acos(min(1., abs(float(np.dot(li['d'], lj['d'])))))), 3)))
    return pairs


def dedupe(pairs):
    """X_dup: one line shared and the other lines < 1.5 m apart -> keep the longer overlap."""
    excluded = {}
    for a in range(len(pairs)):
        for b in range(a + 1, len(pairs)):
            pa, pb = pairs[a], pairs[b]
            ids_a, ids_b = {pa['left']['id'], pa['right']['id']}, {pb['left']['id'], pb['right']['id']}
            shared = ids_a & ids_b
            if len(shared) != 1:
                continue
            other_a = (pa['left'] if pa['left']['id'] not in shared else pa['right'])
            other_b = (pb['left'] if pb['left']['id'] not in shared else pb['right'])
            lo, hi = max(pa['overlap'][0], pb['overlap'][0]), min(pa['overlap'][1], pb['overlap'][1])
            mid = (lo + hi) / 2
            frame = pa['frame']
            if abs(float(frame.offset(other_a, mid)) - float(frame.offset(other_b, mid))) >= SAME_ROW_M:
                continue
            len_a, len_b = pa['overlap'][1] - pa['overlap'][0], pb['overlap'][1] - pb['overlap'][0]
            loser, winner = (b, a) if len_a >= len_b else (a, b)
            excluded[loser] = winner
    return excluded


# --- raster analysis of one pair ------------------------------------------------------------------------------

def _region_cells(rmap, frame, s_lo, s_hi, o_lo, o_hi, where):
    corners = [frame.xy(s, o) for s in (s_lo, s_hi) for o in (o_lo, o_hi)]
    xs, ys = [c[0] for c in corners], [c[1] for c in corners]
    rows, cols = RC._box_cells(rmap, min(xs), min(ys), max(xs), max(ys), where)
    x, y = rmap.centres(rows, cols)
    s, o = frame.sd(np.column_stack((x, y)))
    keep = (s >= s_lo) & (s <= s_hi) & (o >= o_lo) & (o <= o_hi)
    return rows[keep], cols[keep], x[keep], y[keep], s[keep], o[keep]


def components(rmap, rows, cols):
    """Component label per cell (8-connected after a 1-cell dilation, as rows.py)."""
    from scipy import ndimage
    if not len(rows):
        return np.zeros(0, dtype=np.int64)
    r0, c0 = rows.min(), cols.min()
    mask = np.zeros((rows.max() - r0 + 1, cols.max() - c0 + 1), dtype=bool)
    mask[rows - r0, cols - c0] = True
    labels, _ = ndimage.label(ndimage.binary_dilation(mask, structure=np.ones((3, 3), bool)),
                              structure=np.ones((3, 3), bool))
    return labels[rows - r0, cols - c0]


def unit_table(labels, s, o, x, y):
    units = []
    for value in np.unique(labels):
        sel = labels == value
        units.append(dict(label=int(value), s=float(s[sel].mean()), o=float(o[sel].mean()), s_min=float(s[sel].min()),
                          s_max=float(s[sel].max()), o_extent=float(o[sel].max() - o[sel].min()), cells=int(sel.sum()),
                          x=float(x[sel].mean()), y=float(y[sel].mean()), sel=np.flatnonzero(sel)))
    return units


def cross_structures(s, o, frame, left, right):
    """1 m slices (along the pair axis) in which occupied cells cover >= 40 % of the aisle interior (relative
    offset 0.2-0.8 between the two row lines, 12 bins): a fence, hedge or wall across the aisle."""
    ol, orr = frame.offset(left, s), frame.offset(right, s)
    rel = (o - orr) / (ol - orr)
    inside = (rel >= .2) & (rel <= .8)
    if not inside.any():
        return []
    slices = np.floor(s[inside] / CROSS_SLICE_M).astype(np.int64)
    bins = np.clip(np.floor((rel[inside] - .2) / .6 * CROSS_BINS).astype(np.int64), 0, CROSS_BINS - 1)
    keys = np.unique(slices * CROSS_BINS + bins)
    slice_of = np.floor_divide(keys, CROSS_BINS)
    values, counts = np.unique(slice_of, return_counts=True)
    hits = values[counts >= CROSS_COVERAGE * CROSS_BINS]
    return [(float(k * CROSS_SLICE_M), float((k + 1) * CROSS_SLICE_M)) for k in hits]


def chains(units, gap=CHAIN_GAP_M, barriers=()):
    """Units (sorted by s) split where the extent-to-extent gap exceeds `gap` or a cross structure (barrier
    interval) lies between two consecutive units."""
    out = []
    for unit in sorted(units, key=lambda u: u['s']):
        if out:
            end = max(u['s_max'] for u in out[-1])
            blocked = any(a < unit['s_min'] and b > end for a, b in barriers)
            if unit['s_min'] - end <= gap and not blocked:
                out[-1].append(unit)
                continue
        out.append([unit])
    return out


def analyse_pair(rmap, pair, record_spacing, trajectory_xy, lines):
    frame = pair['frame']
    left, right = pair['left'], pair['right']
    sep = pair['separation']
    band = min(BAND_MAX_M, BAND_FRACTION * sep)
    (lo_l, hi_l), (lo_r, hi_r) = frame.interval(left), frame.interval(right)
    s_lo, s_hi = min(lo_l, lo_r) - 40., max(hi_l, hi_r) + 40.
    ends = (s_lo, s_hi)
    o_lo = min(float(frame.offset(right, v)) for v in ends) - 2.5
    o_hi = max(float(frame.offset(left, v)) for v in ends) + 2.5
    rows, cols, x, y, s, o = _region_cells(rmap, frame, s_lo, s_hi, o_lo, o_hi, (RC.OCCUPIED,))
    labels = components(rmap, rows, cols)
    units = unit_table(labels, s, o, x, y)
    cross_slices = cross_structures(s, o, frame, left, right)
    side_units = dict(left=[], right=[])
    for unit in units:
        if unit['o_extent'] > UNIT_MAX_ACROSS_M:      # a cross wall / fence / clutter spanning the band: not a row unit
            continue
        if any(unit['s_min'] - CROSS_NEAR_M < b and unit['s_max'] + CROSS_NEAR_M > a for a, b in cross_slices):
            continue                                  # next to a structure across the aisle: not a row unit
        for side, line in (('left', left), ('right', right)):
            if abs(unit['o'] - float(frame.offset(line, unit['s']))) <= band:
                side_units[side].append(unit)
    ov_lo, ov_hi = pair['overlap']
    support, issues = {}, []
    for side in ('left', 'right'):
        meeting = [c for c in chains(side_units[side], barriers=cross_slices) if len(c) >= MIN_CHAIN_UNITS
                   and max(u['s_max'] for u in c) >= ov_lo
                   and min(u['s_min'] for u in c) <= ov_hi]
        if not meeting:
            issues.append(('X_nosupport', side))
            support[side] = []
            continue
        support[side] = [u for c in meeting for u in c]
    if issues:
        return dict(excluded=[dict(rule='X_nosupport', value=[side for _, side in issues])], band=band)
    # Straight-frame centreline (1 m windows) -> polyline.
    s_first = min(min(u['s'] for u in support[k]) for k in support)
    s_last = max(max(u['s'] for u in support[k]) for k in support)
    w_lo, w_hi = math.floor(s_first - EXTEND_BEFORE_M), math.ceil(s_last + EXTEND_AFTER_M)
    centres = np.arange(w_lo, w_hi) + .5
    mids = []
    side_cells = {}
    for side in ('left', 'right'):
        sel = np.concatenate([u['sel'] for u in support[side]])
        side_cells[side] = (s[sel], o[sel], x[sel], y[sel])
    for wc in centres:
        values = []
        for side, line in (('left', left), ('right', right)):
            ss, oo = side_cells[side][0], side_cells[side][1]
            inwin = (ss >= wc - .5) & (ss < wc + .5)
            values.append(float(np.median(oo[inwin])) if inwin.sum() >= 3 else float(frame.offset(line, wc)))
        mids.append((values[0] + values[1]) / 2)
    from scipy.ndimage import gaussian_filter1d
    smooth = gaussian_filter1d(np.asarray(mids), SMOOTH_SIGMA_M, mode='nearest')
    sp = np.arange(centres[0], centres[-1] + 1e-9, RC.POLYLINE_STEP_M)
    px, py = frame.xy(sp, np.interp(sp, centres, smooth))
    points = np.column_stack((px, py))
    curve = RC.CurvilinearFrame(points, 0.)
    # Field frame: s_origin = arclength where both rows begin (first pass on arclength).
    unit_xy = {side: np.array([(u['x'], u['y']) for u in support[side]]) for side in support}
    arc = {side: curve.project_many(unit_xy[side][:, 0], unit_xy[side][:, 1])[0] for side in support}
    s_origin = max(arc['left'].min(), arc['right'].min())
    curve = RC.CurvilinearFrame(points, s_origin)
    field_units = {}
    for side in ('left', 'right'):
        su, du = curve.project_many(unit_xy[side][:, 0], unit_xy[side][:, 1])
        order = np.argsort(su)
        field_units[side] = (su[order], du[order], np.array([support[side][i]['cells'] for i in order]))
    first = {k: float(v[0][0]) for k, v in field_units.items()}
    last = {k: float(v[0][-1]) for k, v in field_units.items()}
    end_fwd = max(last.values())
    start_rev, end_rev = min(last.values()), min(first.values())
    # Row chains (>= 3 units, barriers) of each side beyond the support, in the field frame: end-region resume test.
    all_side, all_units = {}, {}
    for side in ('left', 'right'):
        spans = []
        for chain in chains(side_units[side], barriers=cross_slices):
            xy = np.array([(u['x'], u['y']) for u in chain])
            cs = curve.project_many(xy[:, 0], xy[:, 1])[0]
            if len(chain) >= MIN_CHAIN_UNITS and cs.max() - cs.min() >= RESUME_MIN_EXTENT_M:
                spans.append((float(cs.min()), float(cs.max())))
        all_side[side] = spans
        xy = np.array([(u['x'], u['y']) for u in side_units[side]]).reshape(-1, 2)
        all_units[side] = curve.project_many(xy[:, 0], xy[:, 1]) if len(xy) else (np.zeros(0), np.zeros(0))
    # All occupied cells of the region in the field frame (profile, interior, end region).
    fs, fd = curve.project_many(x, y)
    cross_field = []
    for a, b in cross_slices:
        mid_o = (float(frame.offset(left, (a + b) / 2)) + float(frame.offset(right, (a + b) / 2))) / 2
        px, py = frame.xy((a + b) / 2, mid_o)
        centre = curve.project(float(px), float(py))[0]
        cross_field.append([round(centre - CROSS_SLICE_M / 2, 2), round(centre + CROSS_SLICE_M / 2, 2)])
    side_field = {k: curve.project_many(side_cells[k][2], side_cells[k][3]) for k in side_cells}
    profile = _profile(curve, frame, left, right, side_field, fs, fd, first, last)
    s_w = np.asarray(profile['s_m'])
    spacing = np.asarray(profile['spacing_m'], float)
    free = np.array([np.nan if v is None else v for v in profile['free_width_m']], float)
    both_lo, both_hi = max(first.values()), min(last.values())       # both rows present
    two = (s_w >= both_lo) & (s_w <= both_hi)
    if not two.any():
        two = (s_w >= 0.) & (s_w <= end_fwd)
    free_two = free[two & np.isfinite(free)]
    gaps = {k: _gaps(field_units[k][0], min(first.values()), max(last.values())) for k in field_units}
    long_gaps = {k: _gaps(field_units[k][0], -1e9, 1e9, CHAIN_GAP_M) for k in field_units}
    union_lo, union_hi = min(first.values()), max(last.values())
    sweep = _sweep(rmap, curve, union_lo + FOOTPRINT[0] / 2, union_hi - FOOTPRINT[0] / 2)
    entry = dict(
        band_m=round(band, 3), separation_m=round(sep, 3), overlap_m=round(ov_hi - ov_lo, 3),
        angle_between_deg=pair['angle_between_deg'],
        geometry=dict(polyline_fwd=[[round(float(a), 4), round(float(b), 4)] for a, b in points],
                      s_origin_fwd_m=round(float(s_origin), 4), polyline_step_m=RC.POLYLINE_STEP_M,
                      frame='raster-local metres (x right, y up, raster lower-left corner)'),
        length_m=round(end_fwd, 3), two_sided_s_m=[round(both_lo, 3), round(both_hi, 3)],
        two_sided_length_m=round(both_hi - both_lo, 3),
        spacing_median_m=round(float(np.median(spacing[two])), 3),
        spacing_p10_p90_m=[round(float(np.percentile(spacing[two], q)), 3) for q in (10, 90)],
        free_width_median_m=round(float(np.median(free_two)), 3) if free_two.size else None,
        free_width_p10_m=round(float(np.percentile(free_two, 10)), 3) if free_two.size else None,
        free_width_min_m=round(float(free_two.min()), 3) if free_two.size else None,
        profile=profile,
        sides={k: dict(first_s_m=round(first[k], 3), last_s_m=round(last[k], 3), units=int(len(field_units[k][0])),
                       unit_s_m=[round(float(v), 2) for v in field_units[k][0]],
                       unit_d_m=[round(float(v), 3) for v in field_units[k][1]],
                       gaps_gt_2p5_m=gaps[k], gaps_gt_6_m=long_gaps[k]) for k in field_units},
        cross_structures_s_m=cross_field, centreline_sweep=sweep,
        trajectory=_trajectory(curve, trajectory_xy, both_lo, both_hi,
                               float(np.median(spacing[two]))),
        _trajectory_sd=None if trajectory_xy is None else curve.project_many(trajectory_xy[:, 0], trajectory_xy[:, 1]),
        _curve=curve, _units=field_units, _all_side=all_side, _all_units=all_units)
    def between(lo, hi):
        """Family row lines (other than the pair) between the two rows over field s [lo, hi] (>= 5 m overlap,
        >= 1.5 m from both lines at the middle of the overlap)."""
        a = float(frame.sd(np.array(curve.point(lo, 0.)))[0])
        b = float(frame.sd(np.array(curve.point(hi, 0.)))[0])
        found = []
        for lk in lines:
            if lk['id'] in (left['id'], right['id']):
                continue
            lo_k, hi_k = frame.interval(lk)
            k_lo, k_hi = max(lo_k, min(a, b)), min(hi_k, max(a, b))
            if k_hi - k_lo < BETWEEN_OVERLAP_M:
                continue
            sk = (k_lo + k_hi) / 2
            ok, ol, orr = float(frame.offset(lk, sk)), float(frame.offset(left, sk)), float(frame.offset(right, sk))
            if orr < ok < ol and min(ol - ok, ok - orr) >= SAME_ROW_M:
                found.append(lk['id'])
        return found
    entry['directions'] = dict(
        fwd=_direction(rmap, curve, 0., end_fwd, end_fwd, 1., fs, fd, s_w, free, long_gaps, sweep, all_side,
                       (last['left'], last['right']), between(0., end_fwd)),
        rev=_direction(rmap, curve, end_rev, start_rev, end_rev, -1., fs, fd, s_w, free, long_gaps, sweep, all_side,
                       (first['left'], first['right']), between(end_rev, start_rev)))
    entry['directions']['fwd'].update(start_s_fwd_m=0., end_s_fwd_m=round(end_fwd, 3))
    entry['directions']['rev'].update(start_s_fwd_m=round(start_rev, 3), end_s_fwd_m=round(end_rev, 3))
    excluded = []
    lo_w, hi_w = RULES['width_factor']
    if not lo_w * record_spacing <= entry['spacing_median_m'] <= hi_w * record_spacing:
        excluded.append(dict(rule='X_width', value=entry['spacing_median_m'],
                             limits=[round(lo_w * record_spacing, 3), round(hi_w * record_spacing, 3)]))
    if both_hi - both_lo < MIN_OVERLAP_M:
        excluded.append(dict(rule='X_short', value=round(both_hi - both_lo, 3)))
    entry['excluded'] = excluded
    return entry


def _direction(rmap, curve, lo, hi, end, sign, fs, fd, s_w, free, long_gaps, sweep, all_side, ends, between):
    """Rules and attributes of one travel direction over its row interval [lo, hi] (forward field s)."""
    half = FOOTPRINT[0] / 2
    interior = _interior(rmap, curve, lo, hi, fs, fd)
    drivable = (s_w >= lo + half) & (s_w <= hi - half) & np.isfinite(free)
    contacts = [iv for iv in sweep['hard_contact_intervals_m'] if iv[1] >= lo + half and iv[0] <= hi - half]
    out = dict(length_m=round(hi - lo, 3), interior=interior,
               free_width_min_drivable_m=round(float(free[drivable].min()), 3) if drivable.any() else None,
               centreline_clear=not contacts, centreline_hard_contact_intervals_m=contacts,
               cross_breaks=_cross_breaks(long_gaps, lo, hi),
               end_region=_end_region(rmap, curve, end, sign, all_side, fs, fd),
               end_stagger_m=round(abs(ends[0] - ends[1]), 3))
    excl = []
    if between:
        excl.append(dict(rule='X_between', value=between))
    if interior['centre_band_occupied_window_fraction'] >= RULES['midrow_window_fraction']:
        excl.append(dict(rule='X_midrow', value=interior['centre_band_occupied_window_fraction']))
    if interior['drivable_band_fractions']['unobserved'] > RULES['unobserved_max_fraction']:
        excl.append(dict(rule='X_unobserved', value=interior['drivable_band_fractions']['unobserved']))
    if drivable.any() and free[drivable].min() < MIN_FREE_WIDTH_M:
        where = float(s_w[drivable][np.argmin(free[drivable])])
        excl.append(dict(rule='X_blocked', value=dict(min_free_width_m=round(float(free[drivable].min()), 3),
                                                      at_s_fwd_m=where)))
    if out['cross_breaks']:
        excl.append(dict(rule='X_crossbreak', value=out['cross_breaks']))
    resume = out['end_region']['resumed_units']
    need = RULES['resume_min_chains']
    if resume['left'] >= need and resume['right'] >= need:
        excl.append(dict(rule='X_end_resumes', value=resume))
    if out['end_stagger_m'] > END_STAGGER_M:
        excl.append(dict(rule='X_end_stagger', value=out['end_stagger_m']))
    out['exclusions'] = excl
    return out


def _profile(curve, frame, left, right, side_field, fs, fd, first, last):
    lo = math.floor(min(first.values()) - EXTEND_BEFORE_M)
    hi = math.ceil(max(last.values()) + EXTEND_AFTER_M)
    centres = np.arange(lo, hi) + .5
    # Line-fit fallback offsets relative to the centreline.
    cx, cy = curve.point_many(centres, np.zeros(len(centres)))
    s_pair, _ = frame.sd(np.column_stack((cx, cy)))
    fallback = {}
    for side, line in (('left', left), ('right', right)):
        lx, ly = frame.xy(s_pair, frame.offset(line, s_pair))
        fallback[side] = curve.project_many(lx, ly)[1]
    out = dict(s_m=[], left_row_d_m=[], right_row_d_m=[], left_measured=[], right_measured=[], spacing_m=[],
               left_face_d_m=[], right_face_d_m=[], free_width_m=[])
    for i, wc in enumerate(centres):
        row = {}
        for side in ('left', 'right'):
            ss, dd = side_field[side]
            win = (ss >= wc - .5) & (ss < wc + .5)
            measured = bool(win.sum() >= 3)
            row[side] = (float(np.median(dd[win])) if measured else float(fallback[side][i]), measured)
        win = (fs >= wc - .5) & (fs < wc + .5)
        dl, dr = row['left'][0], row['right'][0]
        lf = fd[win & (fd > 0) & (fd <= dl + .3)]
        rf = fd[win & (fd < 0) & (fd >= dr - .3)]
        lface = float(lf.min()) if lf.size else None
        rface = float(rf.max()) if rf.size else None
        out['s_m'].append(round(float(wc), 2))
        out['left_row_d_m'].append(round(dl, 3))
        out['right_row_d_m'].append(round(dr, 3))
        out['left_measured'].append(row['left'][1])
        out['right_measured'].append(row['right'][1])
        out['spacing_m'].append(round(dl - dr, 3))
        out['left_face_d_m'].append(None if lface is None else round(lface, 3))
        out['right_face_d_m'].append(None if rface is None else round(rface, 3))
        out['free_width_m'].append(None if lface is None or rface is None else round(lface - rface, 3))
    return out


def _gaps(unit_s, lo, hi, threshold=GAP_REPORT_M):
    s = np.sort(np.asarray(unit_s))
    out = []
    for a, b in zip(s[:-1], s[1:]):
        if b - a > threshold and b >= lo and a <= hi:
            out.append([round(float(a), 2), round(float(b), 2), round(float(b - a), 2)])
    return out


def _cross_breaks(long_gaps, lo, hi):
    out = []
    for a0, a1, _ in long_gaps['left']:
        for b0, b1, _ in long_gaps['right']:
            c0, c1 = max(a0, b0, lo), min(a1, b1, hi)
            if c1 > c0:
                out.append([round(c0, 2), round(c1, 2)])
    return out


def _lattice_classes(rmap, curve, s_lo, s_hi, d_half, step=.05):
    ss = np.arange(s_lo, s_hi + 1e-9, step)
    dd = np.arange(-d_half, d_half + 1e-9, step)
    S, D = np.meshgrid(ss, dd)
    x, y = curve.point_many(S.ravel(), D.ravel())
    return rmap.class_at(x, y).reshape(S.shape), ss, dd


def _fractions(classes):
    n = max(classes.size, 1)
    return {RC.CLASS_NAMES[k]: round(float(np.sum(classes == k)) / n, 4) for k in (RC.FREE, RC.UNCERTAIN,
                                                                                    RC.UNCERTAIN_OCC, RC.UNOBSERVED,
                                                                                    RC.OCCUPIED)}


def _interior(rmap, curve, lo, hi, fs, fd):
    band, _, _ = _lattice_classes(rmap, curve, lo, hi, DRIVABLE_HALF_M)
    wide, _, _ = _lattice_classes(rmap, curve, lo, hi, 1.)
    windows = np.arange(lo, max(hi, lo + 1.), 1.)
    hit = 0
    for w in windows:
        if np.any((fs >= w) & (fs < w + 1.) & (np.abs(fd) <= RULES['midrow_band_m'])):
            hit += 1
    return dict(drivable_band_d_m=DRIVABLE_HALF_M, drivable_band_fractions=_fractions(band),
                band_1m_fractions=_fractions(wide),
                centre_band_occupied_window_fraction=round(hit / max(len(windows), 1), 4))


def _sweep(rmap, curve, lo, hi):
    """Footprint centred on the centreline (heading = tangent) every 0.25 m for centre s in [lo, hi]:
    conservative clearances per class and the s intervals of hard contact."""
    length, width = FOOTPRINT
    first_hard = None
    minimum = {name: math.inf for name in RC.CLEARANCE_GROUPS}
    contacts = {name: 0 for name in RC.CLEARANCE_GROUPS}
    hard_s = []
    for s in np.arange(lo, hi + 1e-9, .25):
        x, y, yaw = curve.pose(float(s), 0., 0.)
        c = RC.footprint_clearances(rmap, x, y, yaw, length, width)
        for k, v in c.items():
            minimum[k] = min(minimum[k], v)
            if v <= 0:
                contacts[k] += 1
                if k == 'hard':
                    hard_s.append(round(float(s), 2))
                    if first_hard is None:
                        first_hard = round(float(s), 2)
    intervals = []
    for v in hard_s:
        if intervals and v - intervals[-1][1] <= .26:
            intervals[-1][1] = v
        else:
            intervals.append([v, v])
    return dict(footprint_m=list(FOOTPRINT), step_m=.25, centre_s_range_m=[round(lo, 3), round(hi, 3)],
                min_clearance_m={k: (round(v, 3) if math.isfinite(v) else None) for k, v in minimum.items()},
                contact_samples=contacts, first_hard_contact_s_m=first_hard, hard_contact_intervals_m=intervals,
                centreline_clear=first_hard is None)


def _end_region(rmap, curve, end, sign, all_side, fs, fd):
    """Beyond `end` in the travel direction `sign` (+1 fwd, -1 rev)."""
    ahead = sign * (fs - end)
    sel = (ahead > 0) & (ahead <= 15.) & (np.abs(fd) <= 1.)
    structure = round(float(ahead[sel].min()), 2) if sel.any() else None
    lo, hi = (end, end + 10.) if sign > 0 else (end - 10., end)
    classes, _, _ = _lattice_classes(rmap, curve, lo, hi, 1.5)
    r0, r1 = RULES['resume_window_m']
    resumed = {}
    for side, spans in all_side.items():
        # a row chain (>= 3 units) whose near end lies 6-20 m beyond the end in the travel direction
        near = [(lo if sign > 0 else hi) for lo, hi in spans]
        resumed[side] = int(sum(r0 < sign * (v - end) <= r1 for v in near))
    return dict(structure_ahead_within_1m_of_centreline_m=structure, beyond_10m_fractions=_fractions(classes),
                resumed_units=resumed)


def _trajectory(curve, xy, lo, hi, spacing):
    if xy is None or not len(xy):
        return None
    s, d = curve.project_many(xy[:, 0], xy[:, 1])
    inside = (np.abs(d) < spacing / 2 - .35) & (s >= lo) & (s <= hi)
    bins = np.unique(np.floor(s[inside]).astype(int)) if inside.any() else np.zeros(0)
    ds = np.diff(s)
    both = inside[:-1] & inside[1:]
    return dict(nodes_inside=int(inside.sum()), covered_fraction=round(len(bins) / max(math.ceil(hi - lo), 1), 4),
                travel_fwd_m=round(float(ds[both & (ds > 0)].sum()), 2),
                travel_rev_m=round(float(-ds[both & (ds < 0)].sum()), 2),
                note='offline SLAM node positions (map estimate, not truth)')


# --- catalogue ------------------------------------------------------------------------------------------------

def load_trajectory(rmap, record):
    path = trajectory_path(record)
    if not path.is_file():
        return None, None
    with open(path, newline='') as fh:
        rows = list(csv.DictReader(fh))
    xs, ys = rmap.map_to_local([float(r['x_m']) for r in rows], [float(r['y_m']) for r in rows])
    return np.column_stack((xs, ys)), sha256_file(path)


def _pair_position(ref, pair):
    mid = (pair['overlap'][0] + pair['overlap'][1]) / 2
    frame = pair['frame']
    offset = (float(frame.offset(pair['left'], mid)) + float(frame.offset(pair['right'], mid))) / 2
    x, y = frame.xy(mid, offset)
    return float(ref.sd(np.array((float(x), float(y))))[1])


def build_record(record):
    rmap = RC.load_record_map(record)
    path = rows_path(record)
    rows = json.loads(path.read_text(encoding='utf-8'))
    family = major_family(rows)
    lines = row_lines(rows, rmap, family)
    pairs = select_pairs(lines)
    if pairs:      # corridor ids follow the position across the record (first pair's normal)
        ref = pairs[0]['frame']
        pairs.sort(key=lambda pair: _pair_position(ref, pair))
    duplicates = dedupe(pairs)
    spacing_record = float(np.median(rows['spacing_pairs']))
    trajectory, trajectory_sha = load_trajectory(rmap, record)
    entries = []
    for index, pair in enumerate(pairs):
        corridor_id = 'c%02d' % (index + 1)
        analysis = analyse_pair(rmap, pair, spacing_record, trajectory, lines)
        excluded = list(analysis.get('excluded', []))
        if index in duplicates:
            excluded.append(dict(rule='X_dup', value='c%02d' % (duplicates[index] + 1)))
        entry = dict(record=record, corridor_id=corridor_id,
                     lines=dict(left=dict(group_id=pair['left']['id'], length_m=pair['left']['length'],
                                          angle_deg=pair['left']['angle'], rows_json_trunks=pair['left']['trunks']),
                                right=dict(group_id=pair['right']['id'], length_m=pair['right']['length'],
                                           angle_deg=pair['right']['angle'], rows_json_trunks=pair['right']['trunks'])),
                     map=dict(pgm_sha256=rmap.info['pgm_sha256'], yaml_sha256=rmap.info['yaml_sha256']))
        if 'geometry' in analysis:
            entry.update({k: v for k, v in analysis.items() if k not in ('excluded',)})
        else:
            entry.update(band_m=analysis.get('band'), directions=dict(fwd=dict(exclusions=[]), rev=dict(exclusions=[])),
                         _line_xy=[pair['left']['a'], pair['left']['b'], pair['right']['a'], pair['right']['b']])
        entry['exclusions'] = excluded
        for direction in entry['directions'].values():
            direction.setdefault('exclusions', [])
            direction['status'] = 'excluded' if excluded or direction['exclusions'] else 'included'
        entry['status'] = ('excluded' if excluded or all(d['status'] == 'excluded' for d in entry['directions'].values())
                           else 'included')
        entries.append(entry)
    info = dict(pgm_sha256=rmap.info['pgm_sha256'], yaml_sha256=rmap.info['yaml_sha256'],
                rows_json=path.name, rows_json_sha256=sha256_file(path), trajectory_sha256=trajectory_sha,
                major_family=family, major_family_share=round(rows['families'][family]['share'], 4),
                major_family_mean_angle_deg=round(rows['families'][family]['mean_angle'], 2),
                other_major_families=[dict(index=i, share=round(f['share'], 4), mean_angle_deg=round(f['mean_angle'], 2))
                                      for i, f in enumerate(rows['families']) if f['major'] and i != family],
                major_family_row_lines=len(lines), lines_ge_25m=sum(l['length'] >= MIN_LINE_M for l in lines),
                pairs_selected=len(pairs), record_median_neighbour_spacing_m=round(spacing_record, 3),
                class_counts=rmap.info['class_counts'], occupied_thresh=rmap.info['occupied_thresh'],
                free_thresh=rmap.info['free_thresh'], raster_cells=[rmap.shape[1], rmap.shape[0]])
    return rmap, entries, info


def strip_private(entry):
    return {k: v for k, v in entry.items() if not k.startswith('_')}


# --- PNG -----------------------------------------------------------------------------------------------------

CLASS_RGB = {RC.FREE: (.86, .86, .86), RC.UNCERTAIN: (.74, .74, .76), RC.UNOBSERVED: (1., 1., 1.),
             RC.OCCUPIED: (0., 0., 0.), RC.ADDED_HARD: (.65, 0., 0.), RC.ADDED_BRANCH: (0., .45, 0.),
             RC.UNCERTAIN_OCC: (.55, .55, .6)}
# max-pool priority per class code: unobserved < uncertain < free < uncertain_occ < occupied < added
PRIORITY = np.array([2, 1, 0, 4, 5, 6, 3], dtype=np.uint8)


def class_image(classes, factor=1):
    """RGB image (top row = highest y) of a class grid, max-pooled by priority (occupied survives)."""
    if factor > 1:
        h, w = classes.shape[0] // factor * factor, classes.shape[1] // factor * factor
        pri = PRIORITY[classes[:h, :w]].reshape(h // factor, factor, w // factor, factor).max(axis=(1, 3))
        inverse = np.argsort(PRIORITY)
        classes = inverse[pri].astype(np.uint8)
    lut = np.array([CLASS_RGB[k] for k in range(len(RC.CLASS_NAMES))], dtype=float)
    return lut[classes[::-1]]


def render_corridor_png(rmap, entry, record_entries, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    fig = plt.figure(figsize=(19, 10.5), dpi=90)
    gs = fig.add_gridspec(3, 3, width_ratios=(1.05, .75, .75), height_ratios=(1.15, 1.05, .7), hspace=.34, wspace=.12)
    title = '%s %s  status=%s  %s' % (entry['record'], entry['corridor_id'], entry['status'],
                                     ' '.join('%s(%s)' % (e['rule'], e['value'] if not isinstance(e['value'], list)
                                                          or len(e['value']) < 4 else '...') for e in entry['exclusions']))
    fig.suptitle(title, fontsize=13, x=.01, ha='left')
    ax = fig.add_subplot(gs[:, 0])
    factor = max(1, math.ceil(max(rmap.shape) / 1500))
    res = rmap.resolution * factor
    image = class_image(rmap.classes, factor)
    ax.imshow(image, extent=(0, image.shape[1] * res, 0, image.shape[0] * res), interpolation='nearest')
    for other in record_entries:
        if 'geometry' not in other:
            continue
        curve = other['_curve']
        mine = other is entry
        ax.add_patch(Polygon(_corridor_polygon(curve, other), closed=True,
                             fc=(1., .5, 0., .45) if mine else (0., .4, 1., .12),
                             ec=(.85, .3, 0.) if mine else (0., .3, .9, .5), lw=1.5 if mine else .6))
        x, y = curve.point(other['length_m'] / 2, 0.)
        ax.text(x, y, other['corridor_id'], fontsize=11 if mine else 8, color='k' if mine else (0, .2, .7),
                ha='center', va='center', weight='bold' if mine else None)
    if 'geometry' in entry:
        curve = entry['_curve']
        x0, y0 = curve.point(0., 0.)
        x1, y1 = curve.point(entry['length_m'], 0.)
        ax.plot([x0], [y0], 'o', color='lime', ms=8, mec='k')
        ax.plot([x1], [y1], 's', color='red', ms=8, mec='k')
    ax.set_title('whole map (raster-local m, x%d max-pooled); orange = this corridor (green o fwd start, red [] fwd '
                 'end), blue = other pairs' % factor, fontsize=8.5)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    if 'geometry' not in entry:
        if '_line_xy' in entry:
            la, lb, ra, rb = entry['_line_xy']
            ax.plot([la[0], lb[0]], [la[1], lb[1]], '-', color=(.1, .3, 1.), lw=2.5)
            ax.plot([ra[0], rb[0]], [ra[1], rb[1]], '-', color=(.9, .1, .6), lw=2.5)
            ax.set_title('line pair (blue left, magenta right row line of the row analysis): no raster support',
                         fontsize=9)
        fig.savefig(path)
        plt.close(fig)
        return
    curve = entry['_curve']
    length = entry['length_m']
    rev = entry['directions']['rev']
    s_lo = min(-EXTEND_BEFORE_M, rev['end_s_fwd_m'] - 21.)
    s_hi = max(length + 21., rev['start_s_fwd_m'] + 4.)
    half = max(5., entry['spacing_median_m'] + 1.5)
    classes, ss, dd = _lattice_classes(rmap, curve, s_lo, s_hi, half)
    lut = np.array([CLASS_RGB[k] for k in range(len(RC.CLASS_NAMES))], dtype=float)
    picture = lut[classes[::-1]]
    units, prof = entry['_units'], entry['profile']
    sweep = entry['centreline_sweep']
    traj = entry.get('_trajectory_sd')
    marks = ((0., 'lime', '-', 'fwd start'), (length, 'red', '-', 'fwd end'),
             (rev['start_s_fwd_m'], 'lime', ':', 'rev start'), (rev['end_s_fwd_m'], 'red', ':', 'rev end'))

    def draw(axis, lo, hi, aspect, labels):
        axis.imshow(picture, extent=(ss[0], ss[-1], dd[0], dd[-1]), aspect=aspect, interpolation='nearest')
        for side, color, marker in (('left', (.1, .3, 1.), '^'), ('right', (.9, .1, .6), 'v')):
            au, ad = entry['_all_units'][side]
            axis.plot(au, ad, marker, mec=color, mfc='none', ms=4, ls='none', alpha=.6)
            us, ud, _ = units[side]
            axis.plot(us, ud, marker, color=color, ms=3.5, ls='none')
            key = 'left_row_d_m' if side == 'left' else 'right_row_d_m'
            axis.plot(prof['s_m'], prof[key], '-', color=color, lw=.8, alpha=.7)
            for a, b, _ in entry['sides'][side]['gaps_gt_2p5_m']:
                d0 = np.interp((a + b) / 2, prof['s_m'], prof[key])
                axis.plot([a, b], [d0, d0], '-', color='orange', lw=3, alpha=.8)
        axis.axhline(0., color=(0., .6, 1.), lw=1.2, ls='--')
        for a, b in entry['cross_structures_s_m']:
            axis.axvspan(a, b, color=(1., .6, 0.), alpha=.25, lw=0)
        if traj is not None:
            keep = (traj[0] >= lo) & (traj[0] <= hi) & (np.abs(traj[1]) <= half)
            axis.plot(traj[0][keep], traj[1][keep], '.', color=(0., .55, 0.), ms=1.5, alpha=.6)
        for s_mark, color, ls, label in marks:
            axis.axvline(s_mark, color=color, ls=ls, lw=1.5)
            if labels and lo <= s_mark <= hi:
                axis.text(s_mark, half * .9, label, color=color, fontsize=7.5, ha='center', va='top',
                          bbox=dict(fc='w', ec='none', alpha=.7, pad=1))
        if sweep['first_hard_contact_s_m'] is not None:
            axis.plot([sweep['first_hard_contact_s_m']], [0.], 'x', color='red', ms=12, mew=2.5)
        axis.set_xlim(lo, hi)
        axis.set_ylim(-half, half)
        axis.set_ylabel('d [m]')
    ax2 = fig.add_subplot(gs[0, 1:])
    draw(ax2, s_lo, s_hi, 'auto', True)
    ax2.set_title('corridor unrolled (s along the centreline, d left +; vertical exaggerated). blue ^ / magenta v = '
                  'left / right row support units (hollow: other band units), orange = gaps > 2.5 m, shaded = cross '
                  'structures, green = SLAM trajectory, red x = first centreline footprint contact', fontsize=7.5)
    span = min(30., s_hi - s_lo)
    ax3 = fig.add_subplot(gs[1, 1])
    draw(ax3, s_lo, s_lo + span, 'equal', False)
    ax3.set_title('true scale: start region', fontsize=8)
    ax4 = fig.add_subplot(gs[1, 2])
    draw(ax4, s_hi - span, s_hi, 'equal', False)
    ax4.set_title('true scale: end region', fontsize=8)
    ax5 = fig.add_subplot(gs[2, 1:])
    s_w = np.asarray(prof['s_m'])
    ax5.plot(s_w, prof['spacing_m'], '-', color='k', lw=1.2, label='row-centre spacing')
    free = np.array([np.nan if v is None else v for v in prof['free_width_m']])
    ax5.plot(s_w, free, '-', color=(.9, .45, 0.), lw=1.2, label='free width (inner faces)')
    ax5.axhline(FOOTPRINT[1], color='r', ls=':', lw=1, label='vehicle width 0.70 m')
    for s_mark, color, ls, _ in marks:
        ax5.axvline(s_mark, color=color, ls=ls)
    ax5.set_xlim(s_lo, s_hi)
    ax5.set_ylim(0, max(6., np.nanmax(np.asarray(prof['spacing_m'], float)) + .5))
    ax5.set_xlabel('s [m] (forward field frame: 0 = both rows begin)')
    ax5.set_ylabel('[m]')
    ax5.legend(fontsize=7, loc='lower left', ncol=3)
    lines = []
    for name in ('fwd', 'rev'):
        d = entry['directions'][name]
        fr = d['interior']['drivable_band_fractions']
        lines.append('%s: %s len %.1f m; |d|<=0.6 unobserved %.3f uncertain %.3f uncertain_occ %.3f; free min %s; '
                     'centreline clear %s; excl %s; end: %s' % (
                         name, d['status'], d['length_m'], fr['unobserved'], fr['uncertain'], fr['uncertain_occ'],
                         d['free_width_min_drivable_m'], d['centreline_clear'], [e['rule'] for e in d['exclusions']],
                         _end_text(d['end_region'])))
    text = ('spacing %.2f m (p10-p90 %.2f-%.2f), free width median %s min %s (two-sided part); gaps>2.5m L %d R %d; '
            'trajectory covered %s\n%s' % (
                entry['spacing_median_m'], *entry['spacing_p10_p90_m'], entry['free_width_median_m'],
                entry['free_width_min_m'], len(entry['sides']['left']['gaps_gt_2p5_m']),
                len(entry['sides']['right']['gaps_gt_2p5_m']),
                None if entry['trajectory'] is None else entry['trajectory']['covered_fraction'], '\n'.join(lines)))
    fig.text(.33, .004, text, fontsize=7.5, family='monospace')
    fig.savefig(path)
    plt.close(fig)


def _end_text(region):
    return 'structure %s m, resumed chains L%d R%d' % (region['structure_ahead_within_1m_of_centreline_m'],
                                                region['resumed_units']['left'], region['resumed_units']['right'])


def _corridor_polygon(curve, entry):
    half = entry['spacing_median_m'] / 2
    ss = np.linspace(0., entry['length_m'], 40)
    lx, ly = curve.point_many(ss, np.full(len(ss), half))
    rx, ry = curve.point_many(ss[::-1], np.full(len(ss), -half))
    return np.column_stack((np.concatenate((lx, rx)), np.concatenate((ly, ry))))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    p.add_argument('--out', required=True, help='output directory (CATALOGUE.json + one PNG per corridor)')
    p.add_argument('--records', nargs='*', default=list(RC.RECORDS))
    p.add_argument('--no-png', action='store_true')
    args = p.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    corridors, records = [], {}
    for record in args.records:
        rmap, entries, info = build_record(record)
        records[record] = info
        for entry in entries:
            if not args.no_png:
                name = 'corridor_%s_%s.png' % (record, entry['corridor_id'])
                render_corridor_png(rmap, entry, entries, out / name)
                entry['png'] = name
            corridors.append(strip_private(entry))
        print('%s: %d pairs, %d included' % (record, len(entries), sum(e['status'] == 'included' for e in entries)),
              flush=True)
    summary = dict(pairs=len(corridors), included=sum(c['status'] == 'included' for c in corridors),
                   directions_included=sum(d['status'] == 'included' for c in corridors for d in c['directions'].values()),
                   exclusions={})
    for c in corridors:
        for e in c['exclusions']:
            summary['exclusions'][e['rule']] = summary['exclusions'].get(e['rule'], 0) + 1
        for name, d in c['directions'].items():
            for e in d.get('exclusions', []):
                key = e['rule'] + ':' + name
                summary['exclusions'][key] = summary['exclusions'].get(key, 0) + 1
    catalogue = dict(
        schema='ubts.single_row.corridor_catalogue.v1',
        created_utc=datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        selection_rule=SELECTION_RULE,
        selection_implementation=dict(major_family='largest length share', min_line_m=MIN_LINE_M,
                                      min_overlap_m=MIN_OVERLAP_M, same_row_m=SAME_ROW_M,
                                      between_overlap_m=BETWEEN_OVERLAP_M),
        labels=dict(band='min(0.75, 0.3 x separation) around each line', unit_max_across_m=UNIT_MAX_ACROSS_M,
                    chain_gap_m=CHAIN_GAP_M,
                    start='fwd: max(first row-unit centre of each row) = s 0; rev: min(last)',
                    end='fwd: max(last row-unit centre of each row) (last trunk on BOTH rows); rev: min(first)',
                    centreline='per 1 m window median offset of each row support (>= 3 cells) else line fit; mean; '
                               'Gaussian sigma 2 m; 0.25 m polyline; +8 m before / +15 m after the row units',
                    gaps='consecutive row-unit centres > 2.5 m apart', footprint_m=list(FOOTPRINT)),
        exclusion_rules=EXCLUSION_TEXT, rule_parameters=RULES,
        cell_classes=dict(occupied='p > occupied_thresh (YAML 0.65)', free='p < free_thresh (YAML 0.196)',
                          unobserved='pixel 128 (no submap data): rays end without return',
                          uncertain='observed, free_thresh <= p < 0.5: transparent, not verified free',
                          uncertain_occ='observed, 0.5 <= p <= occupied_thresh: transparent, not verified free'),
        sensing_model=RC.SENSING_MODEL, source=RC.SOURCE,
        builder=dict(raster_catalogue_py_sha256=sha256_file(__file__),
                     raster_corridor_py_sha256=sha256_file(RC.__file__)),
        records=records, summary=summary, corridors=corridors,
        notes=['raster-local metres only; no GNSS coordinates', 'SLAM reconstruction, not physical truth',
               'trajectory = offline SLAM nodes (attribute only)'])
    raw = json.dumps(catalogue, indent=1, default=float).encode('utf-8')
    (out / 'CATALOGUE.json').write_bytes(raw + b'\n')
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
