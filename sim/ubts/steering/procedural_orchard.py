"""Statistics-matched procedural orchard W3 (design 5.1 W3, 8.3 P7; world survey 6 F2).

One parameter set (normalise_params) with a seed gives one orchard. The same layout gives the raster copy (RASTER_V1
classes, 0.05 m) used by the F-SIM and the G-SIM clip node, and, through field_world.build_field_world, the SDF.
The truth (row lines, centreline, row end, units) is exact: it is the generator's own geometry (eval_truth, for
evaluation only).

What is matched (statistics of the 10/10 development corridors, field_world.orchard_stats, 2026-11-09; the 22
development row pairs, 3,849 matched units; quantile knots below, inverse-CDF sampling, linear between knots):
  row spacing          per aisle, the development pairs' median spacing (3.19-3.95 m, median 3.75)
  unit spacing         consecutive occupied components along a row (median 0.60 m, q90 1.45, q99 4.05; the long
                       tail gives the gaps: > 2.5 m about 3-6 per 100 m in the data)
  unit size            extent along / across the row (cell-square box; medians 0.15 / 0.14 m), Gaussian copula with
                       the measured correlation 0.67
  unit offset          lateral offset of a unit from its row line (median 0, q10 -0.26, q90 +0.23 m)
  row line shape       two low-frequency sinusoids per row (wavelength 30-90 m, amplitude 0-0.25 m: SIM_ASSUMED,
                       chosen so that the centreline curvature stays in the range of the data, p95 0.017 1/m)
Units are occupied components of the scan plane, as in the data (not trees): small units (box diagonal <= 0.30 m,
about 70 %) are the trunk-like ones that mode T turns into trunk cylinders; larger ones are canopy sections. An
optional regular trunk layer (tree_spacing_m, trunk_diameter_m; default off, SIM_ASSUMED) adds explicit trunks.

Raster units (2026-11-09 review RW-02): the statistics are measured on the raster (field_world.orchard_stats: the
OCCUPIED components of a row band, 8-connected after a 1-cell dilation; 99.9 % of the data units are their own
component), so the generator keeps the units apart and sizes their cells by the measuring rule:
  separation   a unit's cells keep a Chebyshev distance of at least SEPARATION_CELLS + 1 = 4 cells from the cells of
               every unit placed before it (so it is its own component under the measuring rule). A unit that would
               touch another draws a new lateral offset (PLACE_TRIES times, its own seeded stream), then is pushed
               sideways; the along-row position (the spacing statistic) is never changed. The redrawn offsets widen
               the offset distribution of close units (reported by the stats comparison, procedural_stats()).
  cell size    the unit's cells are the cell centres inside a rectangle along the row; its half sizes are chosen per
               unit (5 x 5 candidates around (size - res) / 2) so that the measured cell-square box (max - min of the
               centre projections + res, the orchard_stats rule) is closest to the drawn along / across size. The
               earlier ellipse rule made the measured boxes smaller (median along 0.12 instead of 0.15 m) and gave
               27 % one-cell units.
The optional trunk layer is placed without the separation (explicit trunks may touch units).

Frames: the generator frame G (rows along +x, row 0 at y = 0) is placed in raster-local metres L by the field yaw
(seeded, U(0, 180) deg by default) and an offset that keeps every cell positive; the field world frame W is the
corridor start frame (field_world.raster_ops.Transform2D from the corridor frame). Cells outside the observed field
(rows' box + observed_margin_m) are UNOBSERVED, inside FREE, unit cells OCCUPIED; the crop edge is UNOBSERVED (the
F-SIM crop rule: corridor polyline box + 35 m).

Crop and length (review RW-05): the crop is the F-SIM rule (axis-aligned corridor polyline box + 35 m), so a long
row at a diagonal yaw makes the largest raster. row_length_m is 10-120 m (the data rows are 23.8-106.9 m) and the crop
may have up to MAX_CROP_CELLS = 4096^2 cells (16 MiB of classes), which every yaw of a 120 m row fits.

Seeds: development seeds 0-999, holdout seeds 100000-100999 (HOLDOUT_SEEDS; proposed for the steering
pre-registration, not used for development). build_raster_world refuses a holdout seed unless allow_holdout is true
(the scene specs carry the flag; review RW-06). The corridor of an episode is the aisle corridor_index (between rows
k and k+1, travel +x).

Pure Python + numpy (no scipy). Placeholder-based SIM results, not field validation.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, replace
import hashlib
import json
import math

import numpy as np

SCHEMA = 'ubts.steer_procedural_orchard.v1'
RESOLUTION_M = .05
CROP_MARGIN_M = 35.                 # closed_loop.raster_corridor.CROP_MARGIN_M (the F-SIM crop rule)
LIDAR_MAX_RANGE_M = 30.
POLYLINE_STEP_M = .25
BEFORE_M, AFTER_M = 8., 15.          # catalogue centreline extension before the start / after the end
DEVELOPMENT_SEEDS = (0, 999)
HOLDOUT_SEEDS = (100000, 100999)
ROW_LENGTH_RANGE_M = (10., 120.)
MAX_CROP_CELLS = 4096 * 4096
SEPARATION_CELLS = 3                 # orchard_stats components: 8-connected after a 1-cell dilation (Chebyshev 3)
PLACE_TRIES = 40                     # lateral offset redraws of a unit that touches another
PUSH_LIMIT_M = 1.5                   # the sideways push after the redraws (then the unit is kept, counted merged)
# Inverse-CDF knots (probability, value) from field_world.orchard_stats over the development corridors (2026-11-09).
SPACING_KNOTS_M = ((0., 0.), (.01, .02), (.05, .09), (.10, .19), (.25, .36), (.50, .60), (.75, .95), (.90, 1.452),
                   (.95, 2.096), (.99, 4.048), (.997, 6.0), (1., 12.0))
OFFSET_KNOTS_M = ((0., -1.168), (.01, -.667), (.05, -.427), (.10, -.256), (.25, -.051), (.50, 0.), (.75, .045),
                  (.90, .226), (.95, .391), (.99, .658), (1., 1.080))
# Offsets with the separation rule (review RW-02, measured 2026-11-09 on the same development units): the first draw
# of every unit comes from the offsets of the units whose along-row neighbours are both at least 0.5 m away (1,585
# units: q10 -0.064, q90 +0.077). A unit that touches the previous unit of its row and is closer than
# CLOSE_PAIR_SPACING_M along the row is put beside it: the neighbour's offset + a side x CLOSE_PAIR_SCALE x |delta d|
# of the data's close pairs (along-row spacing below 0.3 m, 745 pairs: median 0.415 m); other touching units move
# sideways by the smallest amount that frees them. CLOSE_PAIR_SPACING_M and CLOSE_PAIR_SCALE (SIM_ASSUMED) were
# chosen so that the generated offsets and close-pair |delta d| come near the data (seeds 0-11: offsets q05 / q10 /
# q25 / q75 / q90 / q95 -0.43 / -0.29 / -0.04 / 0.04 / 0.27 / 0.46 m against -0.43 / -0.26 / -0.05 / 0.05 / 0.23 /
# 0.39; close-pair |delta d| median 0.42 against 0.415).
BASE_OFFSET_KNOTS_M = ((0., -1.168), (.01, -.436), (.05, -.130), (.10, -.064), (.25, -.023), (.50, 0.), (.75, .025),
                       (.90, .077), (.95, .147), (.99, .415), (1., 1.080))
CLOSE_PAIR_DOFFSET_KNOTS_M = ((0., 0.), (.01, .022), (.05, .098), (.10, .181), (.25, .287), (.50, .415), (.75, .565),
                              (.90, .712), (.95, .863), (.99, 1.197), (1., 1.5))
CLOSE_PAIR_SPACING_M = .25
CLOSE_PAIR_SCALE = .8
ALONG_KNOTS_M = ((0., .05), (.10, .05), (.25, .0999), (.50, .1524), (.75, .2531), (.90, .4311), (.95, .6003),
                 (.99, 1.018), (1., 1.6))
ACROSS_KNOTS_M = ((0., .05), (.10, .05), (.25, .0620), (.50, .1419), (.75, .2033), (.90, .3072), (.95, .4107),
                  (.99, .6485), (1., 1.0))
ROW_SPACING_KNOTS_M = ((0., 3.188), (.05, 3.239), (.10, 3.596), (.25, 3.694), (.50, 3.753), (.75, 3.808),
                       (.90, 3.927), (1., 3.947))
ALONG_ACROSS_CORRELATION = .6714
STATS_SOURCE = 'field_world.orchard_stats over the 22 development row pairs of the 10/10 catalogue (2026-11-09)'

DEFAULTS = dict(
    seed=0, rows=4, corridor_index=None, row_length_m=60., row_spacing_m=None, field_yaw_deg=None,
    bend_wavelength_m=[30., 90.], bend_amplitude_m=[0., .25], end_stagger_m=1.5, observed_margin_m=6.,
    headland_m=15., end_wall_m=None, tree_spacing_m=None, trunk_diameter_m=.12, unit_scale=1.)


class OrchardError(ValueError):
    """Invalid procedural orchard parameters."""


def _number(value, name, lo=None, hi=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise OrchardError('procedural_value_not_a_finite_number: ' + name)
    value = float(value)
    if (lo is not None and value < lo) or (hi is not None and value > hi):
        raise OrchardError('procedural_value_out_of_range: %s %r not in [%s, %s]' % (name, value, lo, hi))
    return value


def normalise_params(params):
    """Validated parameters with every default filled in (canonical form)."""
    params = {} if params is None else params
    if not isinstance(params, dict):
        raise OrchardError('procedural_params_not_a_mapping')
    unknown = sorted(set(params) - set(DEFAULTS))
    if unknown:
        raise OrchardError('procedural_param_unknown: ' + ', '.join(unknown))
    out = copy.deepcopy(DEFAULTS)
    out.update(copy.deepcopy(params))
    seed = out['seed']
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2 ** 32:
        raise OrchardError('procedural_seed_invalid')
    rows = out['rows']
    if isinstance(rows, bool) or not isinstance(rows, int) or not 2 <= rows <= 12:
        raise OrchardError('procedural_rows_invalid (2..12)')
    k = out['corridor_index']
    if k is None:
        k = (rows - 2) // 2
    if isinstance(k, bool) or not isinstance(k, int) or not 0 <= k <= rows - 2:
        raise OrchardError('procedural_corridor_index_invalid')
    out['corridor_index'] = k
    out['row_length_m'] = _number(out['row_length_m'], 'row_length_m', *ROW_LENGTH_RANGE_M)
    if out['row_spacing_m'] is not None:
        out['row_spacing_m'] = _number(out['row_spacing_m'], 'row_spacing_m', 2., 6.)
    if out['field_yaw_deg'] is not None:
        out['field_yaw_deg'] = _number(out['field_yaw_deg'], 'field_yaw_deg', -360., 360.)
    for name, lo, hi in (('bend_wavelength_m', 5., 500.), ('bend_amplitude_m', 0., 2.)):
        pair = out[name]
        if not (isinstance(pair, (list, tuple)) and len(pair) == 2):
            raise OrchardError('procedural_%s_needs_two_values' % name)
        pair = [_number(v, name, lo, hi) for v in pair]
        if pair[0] > pair[1]:
            raise OrchardError('procedural_%s_order' % name)
        out[name] = pair
    out['end_stagger_m'] = _number(out['end_stagger_m'], 'end_stagger_m', 0., 6.)
    out['observed_margin_m'] = _number(out['observed_margin_m'], 'observed_margin_m', 0., 40.)
    out['headland_m'] = _number(out['headland_m'], 'headland_m', 0., 40.)
    if out['end_wall_m'] is not None:
        out['end_wall_m'] = _number(out['end_wall_m'], 'end_wall_m', .2, 30.)
    if out['tree_spacing_m'] is not None:
        out['tree_spacing_m'] = _number(out['tree_spacing_m'], 'tree_spacing_m', .5, 10.)
    out['trunk_diameter_m'] = _number(out['trunk_diameter_m'], 'trunk_diameter_m', .05, .4)
    out['unit_scale'] = _number(out['unit_scale'], 'unit_scale', .25, 4.)
    return out


def canonical_json(params):
    return json.dumps(normalise_params(params), sort_keys=True, separators=(',', ':'), ensure_ascii=True,
                      allow_nan=False).encode('ascii')


def params_sha256(params):
    return hashlib.sha256(canonical_json(params)).hexdigest()


def is_holdout_seed(seed):
    return HOLDOUT_SEEDS[0] <= int(seed) <= HOLDOUT_SEEDS[1]


def check_seed(params, allow_holdout=False):
    """Refuse a holdout seed unless allow_holdout (the steering pre-registration step sets it)."""
    seed = normalise_params(params)['seed']
    if is_holdout_seed(seed) and not allow_holdout:
        raise OrchardError('procedural_holdout_seed_refused: seed %d is in the holdout range %d-%d (allow_holdout '
                           'only for the pre-registered evaluation)' % (seed, HOLDOUT_SEEDS[0], HOLDOUT_SEEDS[1]))
    return seed


def _inverse_cdf(knots, u):
    p = np.array([k[0] for k in knots])
    v = np.array([k[1] for k in knots])
    return np.interp(u, p, v)


def _normal_cdf(z):
    return .5 * (1. + np.vectorize(math.erf)(np.asarray(z, dtype=float) / math.sqrt(2.)))


# --- layout ---------------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Unit:
    row: int
    x_g: float              # along-row position in G (m)
    y_g: float              # centre y in G (row line + offset)
    along_m: float
    across_m: float
    kind: str               # 'unit' (statistics) or 'trunk' (the optional regular trunk layer)


class Layout:
    """Rows, units and the corridor of one orchard in the generator frame G (see the module docstring)."""

    def __init__(self, params):
        self.params = normalise_params(params)
        p = self.params
        rng = np.random.default_rng(np.random.SeedSequence([p['seed'], 0x57A7_0C4D]))
        self.yaw = math.radians(p['field_yaw_deg'] if p['field_yaw_deg'] is not None else float(rng.uniform(0., 180.)))
        n = p['rows']
        spacing = ([p['row_spacing_m']] * (n - 1) if p['row_spacing_m'] is not None
                   else list(_inverse_cdf(ROW_SPACING_KNOTS_M, rng.uniform(size=n - 1))))
        self.row_y0 = np.concatenate(([0.], np.cumsum(spacing)))
        self.spacing = np.asarray(spacing, dtype=float)
        self.bends = []
        lo_w, hi_w = p['bend_wavelength_m']
        lo_a, hi_a = p['bend_amplitude_m']
        for _ in range(n):
            self.bends.append([(float(rng.uniform(lo_a, hi_a)), float(rng.uniform(lo_w, hi_w)),
                                float(rng.uniform(0., 2. * math.pi))) for _ in range(2)])
        length, stagger = p['row_length_m'], p['end_stagger_m']
        self.row_x = [(float(rng.uniform(-stagger, stagger)), length + float(rng.uniform(-stagger, stagger)))
                      for _ in range(n)]
        units = []
        for j in range(n):
            x, (x_start, x_end) = self.row_x[j][0], self.row_x[j]
            while x <= x_end:
                z = rng.normal(size=2)
                z2 = ALONG_ACROSS_CORRELATION * z[0] + math.sqrt(1. - ALONG_ACROSS_CORRELATION ** 2) * z[1]
                along = float(_inverse_cdf(ALONG_KNOTS_M, float(_normal_cdf(z[0])))) * p['unit_scale']
                across = float(_inverse_cdf(ACROSS_KNOTS_M, float(_normal_cdf(z2)))) * p['unit_scale']
                offset = float(_inverse_cdf(BASE_OFFSET_KNOTS_M, float(rng.uniform())))
                units.append(Unit(j, x, float(self.row_y(j, x) + offset), along, across, 'unit'))
                x += float(_inverse_cdf(SPACING_KNOTS_M, float(rng.uniform())))
            if p['tree_spacing_m'] is not None:
                t = x_start
                while t <= x_end:
                    units.append(Unit(j, t, float(self.row_y(j, t)), p['trunk_diameter_m'], p['trunk_diameter_m'],
                                      'trunk'))
                    t += p['tree_spacing_m']
        self.units = units
        k = p['corridor_index']
        self.right_row, self.left_row = k, k + 1
        first = [min(u.x_g for u in units if u.row == r) for r in (k, k + 1)]
        last = [max(u.x_g for u in units if u.row == r) for r in (k, k + 1)]
        self.x_start = max(first)
        self.x_end = max(last)
        self.line_end_x = (last[1], last[0])          # (left, right)

    def row_y(self, j, x):
        x = np.asarray(x, dtype=float)
        y = self.row_y0[j] + sum(a * np.sin(2. * math.pi * x / w + phi) for a, w, phi in self.bends[j])
        return y

    def mid_y(self, x):
        return .5 * (self.row_y(self.right_row, x) + self.row_y(self.left_row, x))


# --- raster -----------------------------------------------------------------------------------------------------------
class Placement:
    """G -> L: rotate by the field yaw, then translate so that the crop starts at (0, 0) of L."""

    def __init__(self, yaw, offset):
        self.yaw, self.c, self.s = yaw, math.cos(yaw), math.sin(yaw)
        self.ox, self.oy = offset

    def to_l(self, x, y):
        x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
        return self.ox + self.c * x - self.s * y, self.oy + self.s * x + self.c * y


def _corridor_polyline(layout):
    xs = np.arange(layout.x_start - BEFORE_M, layout.x_end + AFTER_M + 1e-9, POLYLINE_STEP_M)
    if not np.any(np.isclose(xs, layout.x_start)):
        xs = np.sort(np.concatenate((xs, [layout.x_start])))
    return xs, layout.mid_y(xs)


def raster_unit_cells(cx, cy, heading, along_m, across_m, res=RESOLUTION_M):
    """(rows, cols, measured along, measured across) of one unit: the cell centres inside a rectangle along heading
    whose half sizes are chosen (5 x 5 candidates from (size - res) / 2 to (size + res) / 2) so that the measured
    cell-square box (max - min of the centre projections + res: the orchard_stats rule) is closest to (along_m,
    across_m); an empty set gives the cell of the centre."""
    ax, ay = math.cos(heading), math.sin(heading)
    reach = .5 * max(along_m, across_m) + 2. * res
    c0, c1 = int(math.floor((cx - reach) / res)), int(math.floor((cx + reach) / res))
    r0, r1 = int(math.floor((cy - reach) / res)), int(math.floor((cy + reach) / res))
    cc, rr = np.meshgrid(np.arange(c0, c1 + 1), np.arange(r0, r1 + 1))
    ex, ey = (cc.ravel() + .5) * res - cx, (rr.ravel() + .5) * res - cy
    a, b = ex * ax + ey * ay, -ex * ay + ey * ax
    steps = np.arange(5) * (res / 4.)
    half_a = np.maximum((along_m - res) / 2. + steps, 0.)
    half_b = np.maximum((across_m - res) / 2. + steps, 0.)
    best = None
    for ha in half_a:
        inside_a = np.abs(a) <= ha + 1e-9
        for hb in half_b:
            inside = inside_a & (np.abs(b) <= hb + 1e-9)
            if not inside.any():
                continue
            ma = float(a[inside].max() - a[inside].min()) + res
            mb = float(b[inside].max() - b[inside].min()) + res
            error = abs(ma - along_m) / along_m + abs(mb - across_m) / across_m
            if best is None or error < best[0] - 1e-12:
                best = (error, inside, ma, mb)
    if best is None:
        row, col = int(math.floor(cy / res)), int(math.floor(cx / res))
        return np.array([row]), np.array([col]), res, res
    _, inside, ma, mb = best
    return rr.ravel()[inside], cc.ravel()[inside], ma, mb


def _in_crop(rows, cols, shape):
    ok = (rows >= 0) & (rows < shape[0]) & (cols >= 0) & (cols < shape[1])
    return rows[ok], cols[ok]


def _mark_keepout(keepout, rows, cols, k=SEPARATION_CELLS):
    h, w = keepout.shape
    for dr in range(-k, k + 1):
        rr = rows + dr
        for dc in range(-k, k + 1):
            cc = cols + dc
            ok = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
            keepout[rr[ok], cc[ok]] = True


def _unit_stream(seed, index):
    return np.random.default_rng(np.random.SeedSequence([int(seed), 0x5E9A, int(index)]))


def crop_shape(layout):
    """(width, height, placement offset) of the crop of a layout (the F-SIM rule: corridor polyline box + 35 m, the
    lower-left corner on a whole cell)."""
    res = RESOLUTION_M
    xs, ys = _corridor_polyline(layout)
    rot = Placement(layout.yaw, (0., 0.))
    px, py = rot.to_l(xs, ys)
    lo_x, lo_y = px.min() - CROP_MARGIN_M, py.min() - CROP_MARGIN_M
    hi_x, hi_y = px.max() + CROP_MARGIN_M, py.max() + CROP_MARGIN_M
    offset = (-math.floor(lo_x / res) * res + res, -math.floor(lo_y / res) * res + res)
    width = int(math.ceil((hi_x - lo_x) / res)) + 3
    height = int(math.ceil((hi_y - lo_y) / res)) + 3
    return width, height, offset


def build_raster(layout):
    """(classes uint8 (row 0 = lowest y), placement, polyline_l, unit_cells) of a layout. Places the units with the
    separation rule and the measured-size cells (module docstring); layout.units is updated with the final offsets
    and layout.placement_log counts the redraws."""
    from closed_loop import raster_corridor as RC
    from field_world import raster_ops as ro
    p = layout.params
    res = RESOLUTION_M
    xs, ys = _corridor_polyline(layout)
    # field box in G: rows' x range with the headland, y range of all rows
    gx0 = min(a for a, _ in layout.row_x) - BEFORE_M
    gx1 = max(b for _, b in layout.row_x) + max(p['headland_m'], AFTER_M)
    gy0, gy1 = float(layout.row_y0[0]) - 1.5, float(layout.row_y0[-1]) + 1.5
    m = p['observed_margin_m']
    field_g = np.array([(gx0 - m, gy0 - m), (gx1 + m, gy0 - m), (gx1 + m, gy1 + m), (gx0 - m, gy1 + m)])
    width, height, offset = crop_shape(layout)
    if width * height > MAX_CROP_CELLS:
        raise OrchardError('procedural_raster_too_large: %d x %d cells (limit %d)' % (width, height, MAX_CROP_CELLS))
    place = Placement(layout.yaw, offset)
    classes = np.full((height, width), RC.UNOBSERVED, dtype=np.uint8)
    fxl, fyl = place.to_l(field_g[:, 0], field_g[:, 1])
    observed = ro.polygon_mask(classes.shape, res, 0, 0, np.column_stack((fxl, fyl)))
    classes[observed] = RC.FREE
    del observed
    keepout = np.zeros(classes.shape, dtype=bool)
    unit_cells = []
    log = dict(units=0, redrawn=0, redraws=0, pushed=0, merged=0, trunk_layer=0)
    placed = []
    placed_row = {}
    for index, u in enumerate(layout.units):
        # the unit's along axis follows the local row tangent
        dy = float(layout.row_y(u.row, u.x_g + .05) - layout.row_y(u.row, u.x_g - .05)) / .1
        heading = layout.yaw + math.atan(dy)
        nx, ny = -math.sin(math.atan(dy)), math.cos(math.atan(dy))         # row normal in G (unit)

        def cells_at(y_g, unit=u):
            cx, cy = place.to_l(unit.x_g, y_g)
            rows, cols, ma, mb = raster_unit_cells(float(cx), float(cy), heading, unit.along_m, unit.across_m, res)
            return _in_crop(rows, cols, classes.shape)
        rows, cols = cells_at(u.y_g)
        if u.kind == 'unit':
            log['units'] += 1
            if len(rows) and keepout[rows, cols].any():
                # a close pair: beside the previous unit of the row (|delta d| of the data's close pairs, side
                # seeded); otherwise (or when that fails) the smallest sideways move that frees it
                rng = _unit_stream(p['seed'], index)
                found = False
                previous = placed_row.get(u.row)
                close = previous is not None and abs(u.x_g - previous.x_g) < CLOSE_PAIR_SPACING_M
                for _ in range(PLACE_TRIES if close else 0):
                    log['redraws'] += 1
                    side = 1. if rng.uniform() < .5 else -1.
                    y_new = previous.y_g + side * CLOSE_PAIR_SCALE * float(
                        _inverse_cdf(CLOSE_PAIR_DOFFSET_KNOTS_M, float(rng.uniform())))
                    r2, c2 = cells_at(y_new)
                    if not keepout[r2, c2].any():
                        u, rows, cols, found = replace(u, y_g=y_new), r2, c2, True
                        log['redrawn'] += 1
                        break
                first_side = 1. if rng.uniform() < .5 else -1.
                for k in range(1, 0 if found else int(round(PUSH_LIMIT_M / res)) + 1):
                    for side in (first_side, -first_side):
                        y_new = u.y_g + side * k * res * ny
                        r2, c2 = cells_at(y_new)
                        if not keepout[r2, c2].any():
                            u, rows, cols, found = replace(u, y_g=y_new), r2, c2, True
                            log['pushed'] += 1
                            break
                    if found:
                        break
                if not found:
                    log['merged'] += 1
            placed_row[u.row] = u
            _mark_keepout(keepout, rows, cols)
        else:
            log['trunk_layer'] += 1
        classes[rows, cols] = RC.OCCUPIED
        unit_cells.append((rows, cols))
        placed.append(u)
    layout.units = placed
    layout.placement_log = log
    del keepout
    if p['end_wall_m'] is not None:
        # a fence across all rows beyond the corridor end (0.10 m thick), SIM_ASSUMED
        wx = layout.x_end + p['end_wall_m']
        gy, gxw = np.meshgrid(np.arange(gy0 - 1.5, gy1 + 1.5, res / 2.), np.arange(wx, wx + .10, res / 2.))
        lx, ly = place.to_l(gxw.ravel(), gy.ravel())
        c, r = np.floor(lx / res).astype(np.int64), np.floor(ly / res).astype(np.int64)
        ok = (r >= 0) & (r < height) & (c >= 0) & (c < width)
        classes[r[ok], c[ok]] = RC.OCCUPIED
    classes[[0, -1], :] = RC.UNOBSERVED
    classes[:, [0, -1]] = RC.UNOBSERVED
    lx, ly = place.to_l(xs, ys)
    return classes, place, np.column_stack((lx, ly)), unit_cells


# --- corridor config and world ------------------------------------------------------------------------------------
def _world_class():
    from closed_loop import raster_corridor as RC

    class _Procedural(RC.RasterCorridorWorld):
        """RasterCorridorWorld over a generated raster; describe() names the generator, not a SLAM map."""

        def describe(self):
            out = super().describe()
            out.update(source=SOURCE, procedural_params_sha256=self.map.info.get('params_sha256'),
                       pgm_sha256=None, yaml_sha256=None, generated_truth=True)
            return out
    return _Procedural


SOURCE = 'PROCEDURAL_ORCHARD_W3_STATISTICS_MATCHED_SIM_WORLD'


def ProceduralCorridorWorld(*args, **kwargs):
    """A RasterCorridorWorld subclass instance for a procedural raster (see _world_class)."""
    return _world_class()(*args, **kwargs)


def _project_points(frame, x, y):
    s, d = frame.project_many(np.atleast_1d(x), np.atleast_1d(y))
    return s, d


def build_raster_world(params, modifiers=(), allow_holdout=False):
    """(RasterCorridorConfig, RasterCorridorWorld, source info) of a procedural orchard. The world is the F-SIM
    harness world (closed_loop.raster_corridor) over the procedural raster; world.eval_truth holds the exact truth in
    the field world frame W; world.layout the generator layout. Gap / remove_trunks modifiers remove whole generated
    units (exact cells); the other modifiers go through raster_corridor.apply_modifiers. A holdout seed is refused
    unless allow_holdout (check_seed)."""
    from closed_loop import raster_corridor as RC
    from field_world import raster_ops as ro
    params = normalise_params(params)
    check_seed(params, allow_holdout)
    layout = Layout(params)
    classes, place, polyline, unit_cells = build_raster(layout)
    frame = RC.CurvilinearFrame(polyline)
    xs, _ = _corridor_polyline(layout)
    start_index = int(np.argmin(np.abs(xs - layout.x_start)))
    s_origin = float(frame.s_vertex[start_index] + frame.s_origin)
    frame = RC.CurvilinearFrame(polyline, s_origin)
    k = params['corridor_index']
    unit_sd = []
    for u, (rows, cols) in zip(layout.units, unit_cells):
        lx, ly = place.to_l(u.x_g, u.y_g)
        s, d = frame.project(float(lx), float(ly))[:2]
        unit_sd.append((s, d))
    side_units = {'left': [], 'right': []}
    side_index = {'left': [], 'right': []}
    for i, u in enumerate(layout.units):
        if u.row == layout.left_row:
            side_units['left'].append(unit_sd[i])
            side_index['left'].append(i)
        elif u.row == layout.right_row:
            side_units['right'].append(unit_sd[i])
            side_index['right'].append(i)
    end_s = float(frame.project(*[float(v) for v in place.to_l(layout.x_end, layout.mid_y(layout.x_end))])[0])
    line_end = []
    for row, x_last in ((layout.left_row, layout.line_end_x[0]), (layout.right_row, layout.line_end_x[1])):
        line_end.append(float(frame.project(*[float(v) for v in place.to_l(x_last, layout.mid_y(x_last))])[0]))
    prof_s = np.arange(math.floor(-BEFORE_M), math.ceil(end_s + AFTER_M) + 1e-9, 1.)
    gx = np.interp(prof_s, frame.s_vertex, xs)               # G x of each profile window (polyline vertices)
    row_d = {}
    for side, row in (('left', layout.left_row), ('right', layout.right_row)):
        lx, ly = place.to_l(gx, layout.row_y(row, gx))
        _, d = _project_points(frame, lx, ly)
        row_d[side] = tuple(float(v) for v in d)
    separation = float(np.median(np.asarray(row_d['left']) - np.asarray(row_d['right'])))
    spec_sha = params_sha256(params)
    config = RC.RasterCorridorConfig(
        record='procedural', corridor_id='k%d' % k, direction='fwd', scenario='procedural_s%d_k%d' % (params['seed'], k),
        polyline=tuple(map(tuple, polyline.tolist())), s_origin_m=s_origin, end_s_m=end_s,
        line_end_s_m=tuple(line_end), profile_s_m=tuple(float(v) for v in prof_s), left_row_d_m=row_d['left'],
        right_row_d_m=row_d['right'], left_units=tuple(sorted(side_units['left'])),
        right_units=tuple(sorted(side_units['right'])), row_width_m=separation, band_m=min(.75, .3 * separation),
        headland_depth_m=3., pgm_sha256='', yaml_sha256='', occupied_thresh=None, catalogue_sha256=spec_sha,
        modifiers=tuple(RC.modifier_from_dict(m) for m in modifiers), seed=params['seed'])
    info = dict(record='procedural', resolution_m=RESOLUTION_M, source='procedural_orchard (W3)',
                params_sha256=spec_sha, sensing_model=RC.SENSING_MODEL, pgm_sha256='', yaml_sha256='',
                occupied_thresh=None, free_thresh=None, class_rule='cells are generated as classes (no PGM)')
    rmap = RC.RasterMap(classes, RESOLUTION_M, info, origin_xy=(0., 0.), row0=0, col0=0)
    log, walls = [], []
    removed_units = set()
    for modifier in config.modifiers:
        if modifier.kind in ('gap', 'remove_trunks'):
            indices = side_index[modifier.side]
            order = sorted(indices, key=lambda i: unit_sd[i][0])
            if modifier.kind == 'gap':
                chosen = [i for i in order if modifier.s_from_m <= unit_sd[i][0] <= modifier.s_to_m]
            else:
                first = next((n for n, i in enumerate(order) if unit_sd[i][0] >= modifier.s_from_m), None)
                chosen = [] if first is None else order[first:first + modifier.count]
                if len(chosen) < modifier.count:
                    raise OrchardError('remove_trunks: only %d units after s=%.2f' % (len(chosen), modifier.s_from_m))
            cells = 0
            for i in chosen:
                rows, cols = unit_cells[i]
                hit = rmap.classes[rows, cols] == RC.OCCUPIED
                rmap.classes[rows[hit], cols[hit]] = RC.FREE
                cells += int(hit.sum())
                removed_units.add(i)
            entry = dict(kind=modifier.kind, side=modifier.side, removed_unit_s_m=[round(unit_sd[i][0], 3)
                                                                                   for i in chosen],
                         removed_cells=cells, rule='whole generated units (exact cells)')
            log.append(entry)
        else:
            sub_log, sub_walls = RC.apply_modifiers(rmap, frame, replace(config, modifiers=(modifier,)))
            log += sub_log
            walls += sub_walls
    rmap.classes.flags.writeable = False
    world = ProceduralCorridorWorld(config, rmap, frame, log, walls)
    margin = LIDAR_MAX_RANGE_M + RESOLUTION_M
    h, w = classes.shape
    world.exact_box = (margin, margin, w * RESOLUTION_M - margin, h * RESOLUTION_M - margin)
    x0, y0, theta = frame.pose(0., 0., 0.)
    transform = ro.Transform2D(x0, y0, theta)
    world.layout = layout
    world.eval_truth = eval_truth(layout, place, frame, transform, config, unit_sd, removed_units, walls)
    info.update(modifier_log=log, units=len(layout.units), rows=params['rows'], corridor_index=k,
                holdout_seed=is_holdout_seed(params['seed']), field_yaw_deg=round(math.degrees(layout.yaw), 6),
                stats_source=STATS_SOURCE, placement=dict(layout.placement_log, separation_cells=SEPARATION_CELLS,
                                                          tries=PLACE_TRIES),
                crop_cells=int(classes.size))
    return config, world, info


def eval_truth(layout, place, frame, transform, config, unit_sd, removed_units, walls):
    """Exact truth in W (evaluation only)."""
    from field_world import raster_ops as ro
    from field_world.build_field_world import TRUTH_SCHEMA

    def to_w(lx, ly):
        wx, wy = transform.to_w(lx, ly)
        return np.column_stack((np.atleast_1d(wx), np.atleast_1d(wy)))
    centre = to_w(frame.points[:, 0], frame.points[:, 1])
    rows = {}
    for side, row in (('left', layout.left_row), ('right', layout.right_row)):
        x0, x1 = layout.row_x[row]
        gx = np.arange(x0, x1 + 1e-9, .5)
        lx, ly = place.to_l(gx, layout.row_y(row, gx))
        rows[side] = dict(line_w=to_w(lx, ly))
    all_rows = []
    for j in range(layout.params['rows']):
        x0, x1 = layout.row_x[j]
        gx = np.arange(x0, x1 + 1e-9, 1.)
        lx, ly = place.to_l(gx, layout.row_y(j, gx))
        all_rows.append(to_w(lx, ly))
    units = {'left': [], 'right': []}
    for i, u in enumerate(layout.units):
        side = 'left' if u.row == layout.left_row else 'right' if u.row == layout.right_row else None
        if side is None or i in removed_units:
            continue
        s, d = unit_sd[i]
        units[side].append([s, d, u.along_m, u.across_m, u.kind,
                            'trunk_like' if math.hypot(u.along_m, u.across_m) <= .30 else 'canopy_like'])
    end_x, end_y = frame.point(config.end_s_m, 0.)
    return ro.rounded(dict(
        schema=TRUTH_SCHEMA, use='evaluation only; never an executor, estimator or controller input',
        source='procedural orchard generator (exact geometry of the generated world)',
        frame='W: origin = corridor start (s 0, d 0), +x = start tangent, +y left',
        params=layout.params, params_sha256=params_sha256(layout.params),
        centreline=dict(vertex_s_m=frame.s_vertex, points_w=centre), s_start_m=0., s_end_m=config.end_s_m,
        end_point_w=list(to_w([end_x], [end_y])[0]), line_end_s_m=list(config.line_end_s_m), row_lines=rows,
        all_row_lines_w=all_rows, row_spacing_m=list(layout.spacing), support_units_s_d=units,
        units_columns=['s_m', 'd_m', 'along_m', 'across_m', 'layer', 'size_class'],
        walls_near_face_s_m=list(walls), corridor_index=layout.params['corridor_index'],
        holdout_seed=is_holdout_seed(layout.params['seed'])), 4)
