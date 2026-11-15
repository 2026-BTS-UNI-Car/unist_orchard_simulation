"""GNSS weak layer L3 of the row executor (design v2 V2-1 3, PC-12). No truth, no SIM import, standard library only.

A general (non-RTK) GNSS fix is 4-10 m off (10/10 records: horizontal accuracy p50 5.3-10.0 m) while the row spacing is
3.6-4.1 m and the target speed 0.40 m/s: it can not say where the vehicle is inside the aisle, which row it is in, or the
heading at walking speed (speed accuracy p50 0.43-0.56 m/s). This layer therefore NEVER feeds the corridor estimate, the
row end or a steering / drive command. It only checks, with wide margins, things a gross error would break (the one
exception is mode 'weak_constraint': gnss_rtk.py takes the fixes this layer accepted and, with an RTK FIXED solution
only, nudges the controller's lateral input by a bounded amount; it never writes into the estimator):

  quality gate   a fix is used only when the receiver reports a fix, the horizontal accuracy is known and <= max_hacc_m
                 and it is newer than the previous one; its age at use must stay <= max_age_s (a dropout = 'GNSS absent',
                 a normal state: nothing stops for it)
  jump gate      innovation against the dead-reckoned last accepted fix: |dGNSS| vs the odometry path between the two
                 fixes (the rotation between the frames is unknown before the course alignment, so the bound
                 | |dGNSS| - |dodo| | is used; after the alignment the full vector difference). Above jump_gate_m the fix is
                 rejected ('jump'). reanchor_after consecutive rejected fixes that agree with each other restart the
                 track (a real step of the receiver solution); a gap longer than jump_max_gap_s restarts it too.
  course         once the odometry path of the run passes course_baseline_m, the rotation between the run's odometry
                 frame and the GNSS frame is fitted (2D Kabsch on all accepted (odometry, GNSS) pairs of the run: the
                 travel direction seen by both), refitted at every accepted fix until the path reaches twice the
                 baseline, then frozen (the anchor). The window of the last course_baseline_m of path, fitted alone,
                 against the anchor gives the drift of the odometry heading against GNSS (flag 'course_disagree' beyond
                 course_tolerance_deg). A 15 m window of a 7 m-class fix is still +-10 deg off in the SIM noise model:
                 this is a gross check, never a heading.
  position       with the anchor alignment every accepted fix is mapped into the odometry frame; its distance from the
                 estimator's base pose is the disagreement (flag 'position_disagree' beyond
                 max(fence_margin_m, disagree_factor x hacc)).
  row range      the fence: before the alignment a circle of radius row length + margin around the run's first fix;
                 after it a rectangle along the row axis (the estimator band heading when the alignment was made, else
                 the start heading): along [-margin, row length + margin], across +-margin, margin = max(fence_margin_m,
                 fence_hacc_factor x hacc). fence_confirm consecutive accepted fixes outside trip it. Modes 'geofence'
                 and 'weak_constraint' end the run then ('gnss_geofence'); 'log_only' only reports it.

Positions are local metres (east, north) of a frame the caller chooses: LocalFrame converts latitude / longitude to
metres about the first fix of the process (the origin stays in memory; it is never logged or exported). The summary has
run-local metres only (relative to the run's first accepted fix).
"""
from collections import deque
import math

from .perception_settings import GnssSettings

OUTCOMES = ('accepted', 'restarted', 'reanchored', 'invalid', 'order', 'no_fix', 'quality', 'jump')
FLAGS = ('course_disagree', 'position_disagree', 'fence_outside')
EARTH_RADIUS_M = 6371008.8
TRACK_PAIRS = 600                 # accepted (odometry, GNSS) pairs kept per run (10 min at 1 Hz)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalFrame:
    """Latitude / longitude [deg] -> local east / north metres about the first fix (equirectangular; within a few km
    the error is far below the GNSS error). The origin is private and never exported."""

    def __init__(self):
        self._origin = None

    def to_local(self, latitude_deg, longitude_deg):
        lat, lon = float(latitude_deg), float(longitude_deg)
        if not (math.isfinite(lat) and math.isfinite(lon) and -90. <= lat <= 90. and -180. <= lon <= 180.):
            return None
        if self._origin is None:
            self._origin = (lat, lon)
        lat0, lon0 = self._origin
        east = wrap(math.radians(lon - lon0)) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
        north = math.radians(lat - lat0) * EARTH_RADIUS_M
        return east, north


def fix_from_message(msg, frame):
    """(east_m, north_m, hacc_m or None, fix_ok) of a GNSS message, or None when it can not be read.

    SIM messages carry local metres (east_m, north_m, hacc_m, fix_ok). A NavSatFix-like message (latitude, longitude,
    status.status >= 0 = fix, position_covariance with a known type) is converted by `frame` (LocalFrame); hacc = sqrt of
    the larger horizontal variance, None when the covariance type is unknown (0)."""
    try:
        if hasattr(msg, 'east_m'):
            hacc = getattr(msg, 'hacc_m', None)
            return float(msg.east_m), float(msg.north_m), None if hacc is None else float(hacc), bool(msg.fix_ok)
        status = int(msg.status.status)
        local = frame.to_local(msg.latitude, msg.longitude) if status >= 0 else None
        if local is None:
            return (math.nan, math.nan, None, False) if status < 0 else None
        hacc = None
        if int(getattr(msg, 'position_covariance_type', 0)) > 0:
            cov = msg.position_covariance
            variance = max(float(cov[0]), float(cov[4]))
            hacc = math.sqrt(variance) if math.isfinite(variance) and variance >= 0. else None
        return local[0], local[1], hacc, True
    except (AttributeError, TypeError, ValueError, IndexError):
        return None


def _kabsch(odo, gnss):
    """Rotation angle [rad] that best maps the odometry displacements onto the GNSS ones (2D, centred), with the
    centroids; None when the pairs do not span a direction."""
    n = len(odo)
    if n < 2:
        return None
    ox = sum(p[0] for p in odo) / n
    oy = sum(p[1] for p in odo) / n
    gx = sum(p[0] for p in gnss) / n
    gy = sum(p[1] for p in gnss) / n
    sxx = sxy = 0.
    spread = 0.
    for (px, py), (qx, qy) in zip(odo, gnss):
        ax, ay, bx, by = px - ox, py - oy, qx - gx, qy - gy
        sxx += ax * bx + ay * by
        sxy += ax * by - ay * bx
        spread += ax * ax + ay * ay
    if spread < 1.:
        return None
    return math.atan2(sxy, sxx), (ox, oy), (gx, gy)


class GnssLayer:
    """One executor's GNSS layer; new_run() at every Start. Not thread-safe (the Runner serialises the calls)."""

    def __init__(self, settings=GnssSettings()):
        problems = settings.problems()
        if problems:
            raise ValueError('gnss settings: ' + '; '.join(problems))
        if settings.mode == 'off':
            raise ValueError('gnss mode off: no layer')
        self.s = settings
        self.counts = dict.fromkeys(OUTCOMES, 0)
        self.new_run(None)

    def new_run(self, row_length_m):
        self.row_length_m = None if row_length_m is None else float(row_length_m)
        self.run_counts = dict.fromkeys(OUTCOMES, 0)
        self.flags = dict.fromkeys(FLAGS, 0)          # number of accepted fixes with the flag up
        self.last = None              # last fix of any outcome: (t, east, north, hacc, ok, outcome)
        self.anchor = None            # last accepted: (t, east, north, odo_x, odo_y)
        self.first = None             # first accepted fix of the run (east, north)
        self.rejected_run = []        # consecutive jump-rejected fixes (t, east, north)
        self.track = deque(maxlen=TRACK_PAIRS)       # accepted (path_m, odo_x, odo_y, east, north)
        self.first_pose = None        # first base pose seen (odometry)
        self.align = None             # (angle, odo centroid, gnss centroid) of the anchor alignment
        self.align_axis = None        # row axis (odometry) used by the fence after the alignment
        self.align_frozen = False
        self.course_drift = None
        self.disagreement = None
        self.fence_outside_run = 0
        self.fence_tripped = False
        self.fence_inside = None
        self.path_m = 0.
        self._last_odo = None

    # --- inputs --------------------------------------------------------------------------------------------------
    def note_pose(self, base_pose):
        """The estimator base pose of this tick (odometry): the run's start pose and the odometry path length."""
        if base_pose is None:
            return
        x, y = float(base_pose[0]), float(base_pose[1])
        if self.first_pose is None:
            self.first_pose = (x, y, float(base_pose[2]))
        if self._last_odo is not None:
            self.path_m += math.hypot(x - self._last_odo[0], y - self._last_odo[1])
        self._last_odo = (x, y)

    def _count(self, outcome):
        self.counts[outcome] += 1
        self.run_counts[outcome] += 1
        return outcome

    def on_fix(self, t, east, north, hacc, fix_ok, base_pose, band_heading=None):
        """One GNSS fix received at executor time t. base_pose: the estimator base pose at t (odometry) or None.
        Returns the outcome (OUTCOMES)."""
        t = float(t)
        if self.last is not None and t <= self.last[0]:
            return self._count('order')
        values = (t, east, north)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values) and fix_ok:
            return self._count('invalid')
        outcome = self._judge(t, east, north, hacc, fix_ok, base_pose, band_heading)
        self.last = (t, east, north, hacc, bool(fix_ok), outcome)
        return self._count(outcome)

    def _judge(self, t, east, north, hacc, fix_ok, base_pose, band_heading):
        s = self.s
        if not fix_ok:
            return 'no_fix'
        if hacc is None or not math.isfinite(hacc) or hacc > s.max_hacc_m:
            return 'quality'
        odo = None if base_pose is None else (float(base_pose[0]), float(base_pose[1]))
        anchor = self.anchor
        if anchor is None or t - anchor[0] > s.jump_max_gap_s or odo is None or anchor[3] is None:
            self.rejected_run = []
            self._accept(t, east, north, odo, hacc, band_heading)
            return 'accepted' if anchor is None else 'restarted'
        innovation = self._innovation(anchor, east, north, odo)
        if innovation > s.jump_gate_m:
            run = self.rejected_run
            if run and math.hypot(east - run[-1][1], north - run[-1][2]) > s.jump_gate_m:
                run.clear()
            run.append((t, east, north))
            if len(run) >= s.reanchor_after:
                self.rejected_run = []
                self._accept(t, east, north, odo, hacc, band_heading, reanchor=True)
                return 'reanchored'
            return 'jump'
        self.rejected_run = []
        self._accept(t, east, north, odo, hacc, band_heading)
        return 'accepted'

    def _innovation(self, anchor, east, north, odo):
        de, dn = east - anchor[1], north - anchor[2]
        dx, dy = odo[0] - anchor[3], odo[1] - anchor[4]
        if self.align is None:
            return abs(math.hypot(de, dn) - math.hypot(dx, dy))
        c, s = math.cos(self.align[0]), math.sin(self.align[0])
        return math.hypot(de - (c * dx - s * dy), dn - (s * dx + c * dy))

    def _accept(self, t, east, north, odo, hacc, band_heading, reanchor=False):
        s = self.s
        self.anchor = (t, east, north, None if odo is None else odo[0], None if odo is None else odo[1])
        if self.first is None:
            self.first = (east, north)
        if odo is None:
            return
        if reanchor:
            # a step of the receiver solution: the pairs before the step are dropped (the alignment stays until the
            # path is long enough again)
            self.track.clear()
        self.track.append((self.path_m, odo[0], odo[1], east, north))
        if not self.align_frozen and self.path_m >= s.course_baseline_m and len(self.track) >= 3:
            # the anchor alignment: all pairs of the run so far, refitted until the path reaches twice the baseline,
            # then frozen (the reference of the course windows); the fence's row axis is taken at the first alignment
            fitted = _kabsch([p[1:3] for p in self.track], [p[3:5] for p in self.track])
            if fitted is not None:
                if self.align is None:
                    start = 0. if self.first_pose is None else self.first_pose[2]
                    self.align_axis = start if band_heading is None else float(band_heading)
                self.align = fitted
                self.align_frozen = self.path_m >= 2. * s.course_baseline_m
        if self.align is not None:
            window = [p for p in self.track if self.path_m - p[0] <= s.course_baseline_m]
            if window and self.path_m - window[0][0] >= .75 * s.course_baseline_m:
                fitted = _kabsch([p[1:3] for p in window], [p[3:5] for p in window])
                if fitted is not None:
                    self.course_drift = wrap(fitted[0] - self.align[0])
            self._position_checks(east, north, odo, hacc)
        else:
            self._radial_fence(east, north, hacc)

    def _margin(self, hacc):
        return max(self.s.fence_margin_m, self.s.fence_hacc_factor * hacc)

    def _fence_step(self, inside):
        self.fence_inside = inside
        if inside:
            self.fence_outside_run = 0
            return
        self.flags['fence_outside'] += 1
        self.fence_outside_run += 1
        if self.fence_outside_run >= self.s.fence_confirm:
            self.fence_tripped = True

    def _radial_fence(self, east, north, hacc):
        if self.row_length_m is None or self.first is None:
            return
        reach = math.hypot(east - self.first[0], north - self.first[1])
        self._fence_step(reach <= self.row_length_m + self._margin(hacc))

    def _position_checks(self, east, north, odo, hacc):
        s = self.s
        angle, (ox, oy), (gx, gy) = self.align
        c, sn = math.cos(angle), math.sin(angle)
        de, dn = east - gx, north - gy
        px, py = ox + c * de + sn * dn, oy - sn * de + c * dn      # the fix in the odometry frame
        self.disagreement = math.hypot(px - odo[0], py - odo[1])
        if self.disagreement > max(s.fence_margin_m, s.disagree_factor * hacc):
            self.flags['position_disagree'] += 1
        if self.course_drift is not None and abs(math.degrees(self.course_drift)) > s.course_tolerance_deg:
            self.flags['course_disagree'] += 1
        if self.row_length_m is None or self.first_pose is None:
            return
        axis = self.align_axis
        ca, sa = math.cos(axis), math.sin(axis)
        rx, ry = px - self.first_pose[0], py - self.first_pose[1]
        along, across = ca * rx + sa * ry, -sa * rx + ca * ry
        margin = self._margin(hacc)
        self._fence_step(-margin <= along <= self.row_length_m + margin and abs(across) <= margin)

    # --- outputs -------------------------------------------------------------------------------------------------
    def present(self, now):
        """True when the last fix was accepted and is not older than max_age_s."""
        last = self.last
        return (last is not None and last[5] in ('accepted', 'restarted', 'reanchored')
                and float(now) - last[0] <= self.s.max_age_s)

    def end_reason(self):
        """'gnss_geofence' in modes geofence / weak_constraint once the fence tripped, else None (log_only never ends
        a run)."""
        return ('gnss_geofence' if self.s.mode in ('geofence', 'weak_constraint') and self.fence_tripped
                else None)

    def summary(self, now):
        last = self.last
        out = dict(mode=self.s.mode, present=self.present(now), counts=dict(self.run_counts),
                   flags=dict(self.flags), fence_tripped=self.fence_tripped, aligned=self.align is not None)
        if last is not None:
            out.update(age_s=round(float(now) - last[0], 3), outcome=last[5],
                       hacc_m=None if last[3] is None or not math.isfinite(last[3]) else round(last[3], 2))
            if self.first is not None and last[4] and math.isfinite(last[1]):
                out['rel_m'] = [round(last[1] - self.first[0], 1), round(last[2] - self.first[1], 1)]
        if self.course_drift is not None:
            out['course_drift_deg'] = round(math.degrees(self.course_drift), 2)
        if self.disagreement is not None:
            out['disagreement_m'] = round(self.disagreement, 2)
        if self.fence_inside is not None:
            out['fence_inside'] = self.fence_inside
        return out
