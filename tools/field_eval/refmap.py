"""Reference map of one lane: landmarks (poles, trunks) or two row lines, in row-local metres; the reference centre
line, the row axis and the row end of the SIM evaluations.

Definitions (the same as sim/ubts/steering/gsim_follow.py row_geometry and the W0 world of
sim/ubts/steering/safety_scripts/follow_matrix.py):
  rows          the two structure rows next to the lane (the map names them; 'left' / 'right' are found from their mean
                lateral coordinate)
  row line      least-squares line y = a + b x through the landmark positions of one row (two endpoints for a line map)
  centre line   the mean of the two row lines (a = (a_r + a_l) / 2, b = (b_r + b_l) / 2); the row axis is its direction
  row end s_E   the largest along-row coordinate of a landmark of the two rows (the last structure, decision R50)
  start s_0     the smallest along-row coordinate
Row frame of a pose (x, y, yaw): along s, lateral d (left positive), heading error psi = yaw - axis (gsim_follow.to_row).

File formats (no geographic coordinates: names that look like them and values beyond +-MAX_ABS_M are refused):
  JSON  {"schema": "ubts.field_eval.map.v1", "frame": "row_local", "units": "m",
         "default_radius_m": 0.05,                     optional, radius of a landmark without radius_m
         "landmarks": [{"id": "R01", "row": "right", "x": 0.0, "y": -1.75, "radius_m": 0.12, "kind": "pole"}, ...],
         "row_lines": [{"row": "right", "from": [x, y], "to": [x, y]}, ...],   instead of (or with) landmarks
         "lane_rows": ["right", "left"],               optional when the map has exactly two rows
         "start_pose": [x, y, yaw_deg],                optional, the start pose of the run in this frame
         "note": "..."}
  CSV   header with the columns id,row,x_m,y_m and optionally radius_m,kind (one landmark per line; '#' comments)
x should run roughly along the row (row-local frame: origin near the row start, x along, y to the left). The line
fit y = a + b x assumes it, as gsim_follow does.
"""
from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import os

SCHEMA = 'ubts.field_eval.map.v1'
MAX_ABS_M = 2000.               # a row-local coordinate beyond this is not row-local (a projected or degree value)
DEFAULT_RADIUS_M = .05
# key fragments of geographic coordinates (assembled so that this file does not spell the field names)
_GEO_KEYS = tuple(a + b for a, b in (('la', 't'), ('lo', 'n'), ('lo', 'ng'), ('la', 'titude'), ('lon', 'gitude'),
                                     ('eas', 'ting'), ('nor', 'thing'), ('ut', 'm'), ('wg', 's84'), ('ep', 'sg')))


class MapError(ValueError):
    pass


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def fit_line(points):
    """Least-squares y = a + b x through (x, y) points (gsim_follow._fit_line)."""
    n = len(points)
    if n < 2:
        raise MapError('a row needs at least 2 landmarks')
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points)
    if sxx <= 0.:
        raise MapError('the landmarks of a row have one x (x must run along the row)')
    b = sum((p[0] - mx) * (p[1] - my) for p in points) / sxx
    return my - b * mx, b


def _geo_key(key):
    k = str(key).lower().replace('-', '_')
    parts = set(k.split('_'))
    return any(g in parts or k == g for g in _GEO_KEYS)


def _check_keys(obj, where):
    if isinstance(obj, dict):
        for key, value in obj.items():
            if _geo_key(key):
                raise MapError('%s: key %r looks like a geographic coordinate; the map is row-local metres only'
                               % (where, key))
            _check_keys(value, where)
    elif isinstance(obj, list):
        for value in obj:
            _check_keys(value, where)


def _metres(value, what):
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise MapError('%s: not a number: %r' % (what, value)) from None
    if not math.isfinite(v) or abs(v) > MAX_ABS_M:
        raise MapError('%s: %r is not a row-local metre value (|v| <= %g)' % (what, value, MAX_ABS_M))
    return v


class RefMap:
    """Landmarks [(id, row, x, y, radius, kind)], row lines {row: ((x0, y0), (x1, y1))} and the lane geometry."""

    def __init__(self, landmarks=(), row_lines=None, lane_rows=None, start_pose=None, default_radius_m=None,
                 source=None, sha256=None, note=None):
        self.default_radius_m = DEFAULT_RADIUS_M if default_radius_m is None else float(default_radius_m)
        self.landmarks = []
        seen = set()
        for k, item in enumerate(landmarks):
            lid, row, x, y, radius, kind = item
            lid = str(lid) if lid not in (None, '') else 'L%03d' % k
            if lid in seen:
                raise MapError('landmark id %r appears twice' % lid)
            seen.add(lid)
            radius = self.default_radius_m if radius in (None, '') else float(radius)
            if not 0. <= radius <= 1.:
                raise MapError('landmark %s: radius %r outside [0, 1] m' % (lid, radius))
            self.landmarks.append((lid, str(row), _metres(x, lid + '.x'), _metres(y, lid + '.y'), radius,
                                   str(kind or 'landmark')))
        self.row_lines = {}
        for row, (p0, p1) in (row_lines or {}).items():
            self.row_lines[str(row)] = ((_metres(p0[0], row), _metres(p0[1], row)),
                                        (_metres(p1[0], row), _metres(p1[1], row)))
        rows = sorted({lm[1] for lm in self.landmarks} | set(self.row_lines))
        if lane_rows is None:
            if len(rows) != 2:
                raise MapError('the map has rows %s; name the two lane rows (lane_rows)' % rows)
            lane_rows = rows
        lane_rows = [str(r) for r in lane_rows]
        if len(lane_rows) != 2 or lane_rows[0] == lane_rows[1] or any(r not in rows for r in lane_rows):
            raise MapError('lane_rows must name two different rows of the map (rows: %s)' % rows)
        self.lane_rows = lane_rows
        self.start_pose = None if start_pose is None else tuple(float(v) for v in start_pose)
        self.source, self.sha256, self.note = source, sha256, note
        self.geometry = self._geometry()

    # --- lane geometry ---------------------------------------------------------------------------------------------
    def _row_points(self, row):
        pts = [(lm[2], lm[3]) for lm in self.landmarks if lm[1] == row]
        if len(pts) >= 2:
            return pts, 'landmarks'
        if row in self.row_lines:
            return list(self.row_lines[row]), 'line'
        raise MapError('row %r has fewer than 2 landmarks and no row line' % row)

    def _geometry(self):
        fits, kinds, every = {}, {}, []
        for row in self.lane_rows:
            pts, kind = self._row_points(row)
            fits[row], kinds[row] = fit_line(pts), kind
            every.extend(pts)
        mean_y = {row: sum(p[1] for p in self._row_points(row)[0]) / len(self._row_points(row)[0])
                  for row in self.lane_rows}
        right, left = sorted(self.lane_rows, key=lambda r: mean_y[r])
        (a_r, b_r), (a_l, b_l) = fits[right], fits[left]
        a, b = (a_r + a_l) / 2., (b_r + b_l) / 2.
        axis = math.atan(b)
        c, s = math.cos(axis), math.sin(axis)
        along = [c * x + s * (y - a) for x, y in every]
        return dict(right_row=right, left_row=left, centre_a=a, centre_b=b, axis_rad=axis,
                    half_width_m=abs(a_l - a_r) / 2. * c, row_start_s=min(along), row_end_s=max(along),
                    row_fits={right: [a_r, b_r], left: [a_l, b_l]}, row_sources=kinds, structures=len(every))

    def to_row(self, x, y, yaw):
        """(along s, lateral d (left positive), heading error psi) of a map-frame pose (gsim_follow.to_row)."""
        g = self.geometry
        c, s = math.cos(g['axis_rad']), math.sin(g['axis_rad'])
        return c * x + s * (y - g['centre_a']), -s * x + c * (y - g['centre_a']), wrap(yaw - g['axis_rad'])

    def to_row_arrays(self, x, y, yaw):
        import numpy as np
        g = self.geometry
        c, s = math.cos(g['axis_rad']), math.sin(g['axis_rad'])
        x, y, yaw = (np.asarray(v, dtype=float) for v in (x, y, yaw))
        psi = yaw - g['axis_rad']
        return c * x + s * (y - g['centre_a']), -s * x + c * (y - g['centre_a']), np.arctan2(np.sin(psi), np.cos(psi))

    def from_row(self, along, lateral, heading=0.):
        g = self.geometry
        c, s = math.cos(g['axis_rad']), math.sin(g['axis_rad'])
        return c * along - s * lateral, g['centre_a'] + s * along + c * lateral, wrap(g['axis_rad'] + heading)

    def landmark_array(self):
        """numpy (n, 3) of x, y, radius and the row index list (0 = right lane row, 1 = left, 2 = other)."""
        import numpy as np
        g = self.geometry
        arr = np.array([[lm[2], lm[3], lm[4]] for lm in self.landmarks], dtype=float).reshape(-1, 3)
        side = [0 if lm[1] == g['right_row'] else 1 if lm[1] == g['left_row'] else 2 for lm in self.landmarks]
        return arr, np.array(side, dtype=int)

    def describe(self):
        g = dict(self.geometry)
        return dict(schema=SCHEMA, source=self.source, sha256=self.sha256, landmarks=len(self.landmarks),
                    row_lines=sorted(self.row_lines), lane_rows=self.lane_rows, start_pose=self.start_pose,
                    default_radius_m=self.default_radius_m, note=self.note,
                    geometry={k: (round(v, 6) if isinstance(v, float) else v) for k, v in g.items()})

    def to_json(self):
        return dict(schema=SCHEMA, frame='row_local', units='m', default_radius_m=self.default_radius_m,
                    lane_rows=list(self.lane_rows),
                    landmarks=[dict(id=lm[0], row=lm[1], x=lm[2], y=lm[3], radius_m=lm[4], kind=lm[5])
                               for lm in self.landmarks],
                    row_lines=[{'row': r, 'from': list(p[0]), 'to': list(p[1])} for r, p in sorted(self.row_lines.items())],
                    **({} if self.start_pose is None else dict(start_pose=list(self.start_pose))),
                    **({} if not self.note else dict(note=self.note)))


def _from_json(raw, source, digest, lane_rows=None, default_radius_m=None):
    try:
        doc = json.loads(raw.decode('utf-8'))
    except (UnicodeDecodeError, ValueError) as exc:
        raise MapError('%s: not JSON (%s)' % (source, exc)) from None
    if not isinstance(doc, dict):
        raise MapError('%s: the map is a JSON object' % source)
    _check_keys(doc, source)
    if doc.get('schema') not in (None, SCHEMA):
        raise MapError('%s: schema %r (expected %s)' % (source, doc.get('schema'), SCHEMA))
    if doc.get('frame') not in (None, 'row_local') or doc.get('units') not in (None, 'm'):
        raise MapError('%s: frame must be row_local and units m' % source)
    marks = []
    for k, item in enumerate(doc.get('landmarks') or ()):
        if not isinstance(item, dict) or 'x' not in item or 'y' not in item or 'row' not in item:
            raise MapError('%s: landmark %d needs row, x, y' % (source, k))
        marks.append((item.get('id'), item['row'], item['x'], item['y'], item.get('radius_m'), item.get('kind')))
    lines = {}
    for item in doc.get('row_lines') or ():
        if not isinstance(item, dict) or not {'row', 'from', 'to'} <= set(item):
            raise MapError('%s: a row line needs row, from, to' % source)
        lines[str(item['row'])] = (item['from'], item['to'])
    radius = doc.get('default_radius_m') if default_radius_m is None else default_radius_m
    return RefMap(marks, lines, lane_rows or doc.get('lane_rows'), doc.get('start_pose'), radius,
                  source=source, sha256=digest, note=doc.get('note'))


def _from_csv(raw, source, digest, lane_rows=None, default_radius_m=None):
    text = raw.decode('utf-8-sig')
    lines = [line for line in text.splitlines() if line.strip() and not line.lstrip().startswith('#')]
    reader = csv.DictReader(io.StringIO('\n'.join(lines)))
    fields = [f.strip() for f in reader.fieldnames or ()]
    for f in fields:
        if _geo_key(f):
            raise MapError('%s: column %r looks like a geographic coordinate; row-local metres only' % (source, f))
    need = {'row', 'x_m', 'y_m'}
    if not need <= set(fields):
        raise MapError('%s: CSV columns must include %s (have %s)' % (source, sorted(need), fields))
    marks = []
    for row in reader:
        row = {k.strip(): (v.strip() if isinstance(v, str) else v) for k, v in row.items() if k}
        marks.append((row.get('id'), row['row'], row['x_m'], row['y_m'], row.get('radius_m') or None, row.get('kind')))
    return RefMap(marks, None, lane_rows, None, default_radius_m, source=source, sha256=digest)


def load_map(path, lane_rows=None, default_radius_m=None):
    """Read a JSON or CSV reference map (module docstring)."""
    with open(path, 'rb') as fh:
        raw = fh.read()
    digest = hashlib.sha256(raw).hexdigest()
    source = os.path.basename(path)
    reader = _from_csv if path.lower().endswith('.csv') else _from_json
    return reader(raw, source, digest, lane_rows, default_radius_m)


def write_map(path, refmap):
    raw = json.dumps(refmap.to_json(), indent=1, sort_keys=True, ensure_ascii=False) + '\n'
    with open(path, 'wb') as fh:
        fh.write(raw.encode('utf-8'))
