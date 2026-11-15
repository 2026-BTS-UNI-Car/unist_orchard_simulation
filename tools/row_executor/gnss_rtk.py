"""RTK weak constraint of the GNSS layer L3 (gnss_mode 'weak_constraint'; docs/steering/GNSS_RTK_KO.md). No truth, no SIM
import, standard library only (A13 role 'decision').

An RTK FIXED solution (u-blox ZED-F9P with a base station or NTRIP corrections) is a few centimetres off, against 4-10 m
for the general fix the layer was built for (gnss_layer.py). With it the layer may nudge ONE controller input: the
lateral offset from the row centre line. It never touches the estimator (corridor band, heading, width, row end), the
heading input or the drive command, and it never keeps a lost corridor alive.

Row line (run-local, anchored at the run's first usable fix; no absolute coordinate is kept beyond the run origin in
memory, none is logged):
  learning   every usable fix while the LiDAR corridor is valid and TWO-SIDED gives a centre-line point: the antenna
             position minus the lever arm (heading = odometry yaw + the odometry->GNSS rotation alpha, a 2D Kabsch fit of
             the run's odometry and RTK tracks) minus the LiDAR lateral offset along the line normal. A straight line is
             fitted (total least squares) to the points of the last window_m of path. It is used only when it has
             min_points over min_span_m and its residual RMS is <= max_rms_m (a curved row or bad data: no line).
  frame      the RTK cross-track position of the base against the LiDAR lateral offset must have a slope inside
             slope_range (1 = same frame; a mirrored or badly scaled GNSS frame gives -1 / far from 1) once the LiDAR
             lateral spread reaches min_spread_m. Checked at most once per metre of path; frame_reject_after
             consecutive failed checks refuse the constraint for the run. Until a check passes nothing is applied.
  bias       a sample farther than learn_gate_m from the active line is not learnt (a LiDAR bias, e.g. misplaced trees
             or a skipped line, is not learnt into the line). Disagreement for disagree_reset_m of path drops the line
             (the row really moved, or the line was wrong): it is relearnt from the LiDAR.

Gate (every condition, else the fix is not used; reasons are counted): solution type in fix_types ('fixed'; 'float'
optional; a NavSatFix GBAS status - the UBTS driver's RTK FIXED or FLOAT - counts as float), reported horizontal accuracy
<= max_hacc_m, the general layer accepted the fix (quality, order, 3 m jump gate) and raised no flag (course, position,
fence), the odometry-predicted RTK innovation <= jump_gate_m (reanchor_after consistent rejections restart the track),
an odometry pose exists, the LiDAR corridor is valid.

Correction: the disagreement delta = RTK lateral - LiDAR lateral of each used fix is low-passed (innovation_beta); the
target is clamp(gain x delta, +-cap_m) while a line and a verified frame exist and the newest used fix is not older
than hold_s, else 0. The applied correction u moves to the target by at most rate_m_s x dt per tick: a FIXED -> FLOAT ->
none transition, a dropout, a reset or a refused frame fades u out without a step. The controller gets the snapshot
with lateral_offset + u (apply(); the same object while u is 0, so a constraint that never engages leaves every command
bit-identical). The geometry guard and every other rule keep the raw LiDAR values.
"""
from collections import deque
import math

from .gnss_layer import _kabsch
from .perception_settings import RtkSettings

FIX_TYPES = ('none', 'single', 'dgps', 'float', 'fixed', 'rtk', 'unknown')
REASONS = ('fixes', 'used', 'fix_type', 'hacc', 'layer', 'layer_flag', 'no_pose', 'rtk_jump', 'no_corridor',
           'track_restart', 'line_reset')
NMEA_QUALITY = {0: 'none', 1: 'single', 2: 'dgps', 4: 'fixed', 5: 'float', 6: 'none'}    # GGA field 6 (6: dead reckoning)
UBX_CARR_SOLN = {1: 'float', 2: 'fixed'}                                                  # NAV-PVT flags bits 6-7
NAVSAT_STATUS = {-1: 'none', 0: 'single', 1: 'dgps', 2: 'rtk'}    # NavSatStatus; UBTS driver: 2 = GGA quality 4 or 5
SAMPLE_CAP = 2000                 # samples kept (window_m of path at 10 Hz and 0.1 m/s)


def fix_type_from_message(msg):
    """The receiver solution of a GNSS message: one of FIX_TYPES. Order: an explicit fix_type string (SIM), a UBX NAV-PVT
    carr_soln (1 float, 2 fixed), an NMEA GGA quality, a SIM local-metre fix without a type ('single' / 'none'), the
    NavSatFix status (2 GBAS = 'rtk': the UBTS driver maps GGA quality 4 AND 5 to it, fixed or float)."""
    try:
        text = getattr(msg, 'fix_type', None)
        if isinstance(text, str):
            return text if text in FIX_TYPES else 'unknown'
        carr = getattr(msg, 'carr_soln', None)
        if carr is not None and not isinstance(carr, bool) and int(carr) in UBX_CARR_SOLN:
            return UBX_CARR_SOLN[int(carr)]
        quality = getattr(msg, 'quality', None)
        if quality is not None and not isinstance(quality, bool):
            return NMEA_QUALITY.get(int(quality), 'unknown')
        if hasattr(msg, 'east_m'):
            return 'single' if bool(getattr(msg, 'fix_ok', False)) else 'none'
        return NAVSAT_STATUS.get(int(msg.status.status), 'unknown')
    except (AttributeError, TypeError, ValueError):
        return 'unknown'


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _direction(points):
    """Principal direction [rad] of 2D points (total least squares), oriented from the first point to the last."""
    n = len(points)
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points)
    syy = sum((p[1] - my) ** 2 for p in points)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in points)
    phi = .5 * math.atan2(2. * sxy, sxx - syy)
    dx, dy = points[-1][0] - points[0][0], points[-1][1] - points[0][1]
    if dx * math.cos(phi) + dy * math.sin(phi) < 0.:
        phi += math.pi
    return math.atan2(math.sin(phi), math.cos(phi)), (mx, my)


class RtkConstraint:
    """One executor's RTK constraint; new_run() at every Start. Not thread-safe (the Runner serialises the calls)."""

    def __init__(self, settings=RtkSettings()):
        problems = settings.problems()
        if problems:
            raise ValueError('gnss rtk settings: ' + '; '.join(problems))
        self.s = settings
        self.new_run()

    def new_run(self):
        self.pending = []
        self.counts = dict.fromkeys(REASONS, 0)
        self.origin = None                # first usable fix of the run (memory only; positions are relative to it)
        self.path_m = 0.
        self._pose = None                 # odometry base pose of the latest tick
        self.samples = deque(maxlen=SAMPLE_CAP)   # [path, gx, gy, ox, oy, yaw, lateral, heading, two_sided, learn]
        self.prev = None                  # last fix that passed the jump gate: (gx, gy, ox, oy)
        self.rejected = []                # consecutive jump-rejected fixes: (gx, gy, ox, oy)
        self.alpha = None                 # odometry -> RTK frame rotation [rad]
        self.line = None                  # (px, py, phi, rms, span, points)
        self.slope = self.spread = None
        self.frame = 'unverified'         # 'unverified' | 'verified' | 'rejected'
        self.frame_failures = 0
        self._frame_path = None
        self.disagree_from = None
        self.delta_f = None
        self.last_used_t = None
        self.last_fix = None              # (t, fix_type, hacc, reason)
        self.u = 0.
        self.target = 0.
        self.last_tick_t = None
        self.max_abs_u = 0.
        self.max_step = 0.
        self.state = 'learning'

    # --- inputs ----------------------------------------------------------------------------------------------------
    def on_fix(self, t, east, north, hacc, fix_type, layer_outcome, flagged):
        """A fix the general layer judged (layer_outcome, flagged = it raised a flag); handled at the next tick()."""
        self.pending.append((float(t), east, north, hacc, fix_type, layer_outcome, bool(flagged)))

    def tick(self, now, s, base_pose):
        """Once per Runner tick with the estimator snapshot s and the odometry base pose: the pending fixes, then the
        correction target and the rate-limited applied correction."""
        now = float(now)
        if base_pose is not None:
            x, y, yaw = float(base_pose[0]), float(base_pose[1]), float(base_pose[2])
            if self._pose is not None:
                self.path_m += math.hypot(x - self._pose[0], y - self._pose[1])
            self._pose = (x, y, yaw)
        pending, self.pending = self.pending, []
        for fix in pending:
            reason = self._process(fix, s)
            self.counts[reason] += 1
            self.counts['fixes'] += 1
            self.last_fix = (fix[0], fix[4], fix[3], reason)
        cfg = self.s
        usable = (self.frame == 'verified' and self.line is not None and self.delta_f is not None
                  and self.last_used_t is not None and now - self.last_used_t <= cfg.hold_s)
        self.target = max(-cfg.cap_m, min(cfg.cap_m, cfg.gain * self.delta_f)) if usable else 0.
        dt = 0. if self.last_tick_t is None else max(0., now - self.last_tick_t)
        self.last_tick_t = now
        limit = cfg.rate_m_s * dt
        step = max(-limit, min(limit, self.target - self.u))
        self.u += step
        self.max_step = max(self.max_step, abs(step))
        self.max_abs_u = max(self.max_abs_u, abs(self.u))
        if self.frame == 'rejected':
            self.state = 'refused' if self.u == 0. else 'fading'
        elif usable:
            self.state = 'active'
        else:
            self.state = 'fading' if self.u != 0. else 'learning'

    # --- output ----------------------------------------------------------------------------------------------------
    def apply(self, s):
        """The controller's snapshot: s itself while u is 0 or the corridor is not valid, else a copy with
        lateral_offset + u."""
        if self.u == 0.:
            return s
        lateral = s.get('lateral_offset')
        if s.get('corridor_valid') is not True or not _finite(lateral):
            return s
        out = dict(s)
        out['lateral_offset'] = lateral + self.u
        return out

    def summary(self, now):
        out = dict(state=self.state, frame=self.frame, correction_m=round(self.u, 4), target_m=round(self.target, 4),
                   max_abs_correction_m=round(self.max_abs_u, 4), max_step_m=round(self.max_step, 5),
                   counts=dict(self.counts))
        if self.delta_f is not None:
            out['innovation_m'] = round(self.delta_f, 4)
        if self.last_fix is not None:
            t, fix_type, hacc, reason = self.last_fix
            out.update(fix_type=fix_type, fix_reason=reason, fix_age_s=round(float(now) - t, 3),
                       fix_hacc_m=round(hacc, 3) if _finite(hacc) else None)
        if self.last_used_t is not None:
            out['used_age_s'] = round(float(now) - self.last_used_t, 3)
        if self.line is not None:
            out['line'] = dict(points=self.line[5], span_m=round(self.line[4], 2), rms_m=round(self.line[3], 4))
        if self.slope is not None:
            out.update(slope=round(self.slope, 3), spread_m=round(self.spread, 4))
        return out

    # --- internals -------------------------------------------------------------------------------------------------
    def _usable_type(self, fix_type):
        return fix_type == 'fixed' or (fix_type in ('float', 'rtk') and 'float' in self.s.fix_types)

    def _restart_track(self):
        """A step of the receiver solution: the run's RTK track and line start again (the frame verdict stays)."""
        self.samples.clear()
        self.prev = None
        self.rejected = []
        self.alpha = None
        self.line = None
        self.delta_f = None
        self.disagree_from = None
        self.counts['track_restart'] += 1

    def _innovation(self, ref, gx, gy, ox, oy):
        rgx, rgy, rox, roy = ref
        dox, doy = ox - rox, oy - roy
        if self.alpha is None:
            return abs(math.hypot(gx - rgx, gy - rgy) - math.hypot(dox, doy))
        c, sn = math.cos(self.alpha), math.sin(self.alpha)
        return math.hypot(gx - rgx - (c * dox - sn * doy), gy - rgy - (sn * dox + c * doy))

    def _process(self, fix, s):
        cfg = self.s
        t, east, north, hacc, fix_type, outcome, flagged = fix
        if not self._usable_type(fix_type):
            return 'fix_type'
        if not _finite(hacc) or hacc > cfg.max_hacc_m:
            return 'hacc'
        if outcome in ('restarted', 'reanchored'):
            self._restart_track()          # the general layer saw a gap or a lasting step: a new RTK track
        elif outcome != 'accepted':
            return 'layer'
        if flagged:
            return 'layer_flag'
        if self._pose is None or not (_finite(east) and _finite(north)):
            return 'no_pose'
        if self.origin is None:
            self.origin = (float(east), float(north))
        gx, gy = float(east) - self.origin[0], float(north) - self.origin[1]
        ox, oy, yaw = self._pose
        if self.prev is not None and self._innovation(self.prev, gx, gy, ox, oy) > cfg.jump_gate_m:
            run = self.rejected
            if run and self._innovation(run[-1], gx, gy, ox, oy) > cfg.jump_gate_m:
                run.clear()
            run.append((gx, gy, ox, oy))
            if len(run) < cfg.reanchor_after:
                return 'rtk_jump'
            self._restart_track()          # a lasting step (another solution): start again from this fix
        self.rejected = []
        self.prev = (gx, gy, ox, oy)
        lateral, heading = s.get('lateral_offset'), s.get('heading_error')
        if s.get('corridor_valid') is not True or not _finite(lateral) or not _finite(heading):
            return 'no_corridor'
        two_sided = 'one_side' not in str(s.get('corridor_reason'))
        sample = [self.path_m, gx, gy, ox, oy, yaw, float(lateral), float(heading), two_sided, two_sided]
        if self.line is not None and self.alpha is not None:
            delta = self._delta(sample)
            if abs(delta) > cfg.learn_gate_m:
                sample[9] = False
                if self.disagree_from is None:
                    self.disagree_from = self.path_m
                elif self.path_m - self.disagree_from >= cfg.disagree_reset_m:
                    for old in self.samples:
                        old[9] = False
                    sample[9] = two_sided
                    self.line = None
                    self.delta_f = None
                    self.disagree_from = None
                    self.counts['line_reset'] += 1
            else:
                self.disagree_from = None
            if self.line is not None:
                self.delta_f = delta if self.delta_f is None else (self.delta_f
                                                                   + cfg.innovation_beta * (delta - self.delta_f))
                self.last_used_t = t
        self.samples.append(sample)
        while self.samples and self.path_m - self.samples[0][0] > cfg.window_m:
            self.samples.popleft()
        self._fit()
        return 'used'

    def _base(self, sample):
        """Base position of a sample in the run-local RTK frame (antenna minus the lever arm at the aligned yaw)."""
        psi = sample[5] + self.alpha
        c, sn = math.cos(psi), math.sin(psi)
        ax, ay = self.s.antenna_x_m, self.s.antenna_y_m
        return sample[1] - (c * ax - sn * ay), sample[2] - (sn * ax + c * ay)

    def _delta(self, sample):
        px, py, phi = self.line[0], self.line[1], self.line[2]
        bx, by = self._base(sample)
        return -math.sin(phi) * (bx - px) + math.cos(phi) * (by - py) - sample[6]

    def _fit(self):
        cfg = self.s
        pts = self.samples
        if len(pts) < 3 or pts[-1][0] - pts[0][0] < cfg.min_span_m:
            return
        fitted = _kabsch([(p[3], p[4]) for p in pts], [(p[1], p[2]) for p in pts])
        if fitted is None:
            return
        self.alpha = fitted[0]
        learn = [p for p in pts if p[9]]
        if len(learn) < max(3, cfg.min_points) or learn[-1][0] - learn[0][0] < cfg.min_span_m:
            self.line = None
            return
        bases = [self._base(p) for p in learn]
        lats = [p[6] for p in learn]
        phi0, _ = _direction(bases)
        n0x, n0y = -math.sin(phi0), math.cos(phi0)
        centres = [(bx - lat * n0x, by - lat * n0y) for (bx, by), lat in zip(bases, lats)]
        phi, (px, py) = _direction(centres)
        nx, ny = -math.sin(phi), math.cos(phi)
        residuals = [nx * (cx - px) + ny * (cy - py) for cx, cy in centres]
        rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))
        span = learn[-1][0] - learn[0][0]
        self.line = (px, py, phi, rms, span, len(learn)) if rms <= cfg.max_rms_m else None
        cross = [nx * (bx - px) + ny * (by - py) for bx, by in bases]
        mean_l = sum(lats) / len(lats)
        mean_q = sum(cross) / len(cross)
        var_l = sum((lat - mean_l) ** 2 for lat in lats) / len(lats)
        self.spread = math.sqrt(var_l)
        self.slope = (None if var_l <= 0. else
                      sum((q - mean_q) * (lat - mean_l) for q, lat in zip(cross, lats)) / len(lats) / var_l)
        if (self.frame != 'unverified' or self.spread < cfg.min_spread_m or self.slope is None
                or (self._frame_path is not None and self.path_m - self._frame_path < 1.)):
            return
        self._frame_path = self.path_m
        lo, hi = cfg.slope_range
        if lo <= self.slope <= hi:
            self.frame = 'verified'
            self.frame_failures = 0
        else:
            self.frame_failures += 1
            if self.frame_failures >= cfg.frame_reject_after:
                self.frame = 'rejected'
