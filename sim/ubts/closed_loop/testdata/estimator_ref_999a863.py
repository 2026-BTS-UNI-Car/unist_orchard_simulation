"""Sensor-only state for encoderless row following (no truth, no PWM, no wheels).

Inputs are delayed sensor packets only: LaserScan-like ranges with their
sensor stamp, and raw-IMU-like gyro/accel samples with their stamp. The
configuration holds NOMINAL calibration values (mount lever arm, footprint,
pivot centre); the harness never copies the plant's true values into it.

* odometry: bounded keyframe point-to-point ICP (scipy cKDTree), seeded by
  the integrated gyro. This stands in for the native LiDAR scan matcher; it is
  a harness simplification, not the deployed matcher. Stillness is decided
  from keyframe-relative poses (non-accumulating noise) plus the gyro; the
  snapshot reports how long it has lasted. A failed match keeps the gyro
  rotation and never invents translation.
* corridor acquisition: an axis search restricted to +-acquire_axis_window_deg
  around the current travel direction (or around the last tracked band after a
  loss) that pairs the NEAREST parallel support band on each side and vetoes a
  pair with any intervening band (gapped rows), so cross-row trunk columns and
  rows across a gap are not chosen. Ambiguous best axes are refused.
* corridor tracking: orchard_perception.scan_corridor.observe_scan_corridor on
  every second scan (5 Hz), refined by a shared-slope least-squares line fit
  and tracked as ONE world-fixed band in the odometry frame. When the detector
  is ambiguous or disagrees, returns near the predicted lines are fitted
  instead (gated tracking). One-sided updates keep the learnt width; the
  learnt width also survives a band loss.
* row end: per-side forward support extents excluding any structure that spans
  the aisle core (a wall/fence across both line extensions is not row
  support). A side has ENDED only if observed free space of at least
  end_free_m (and more than the largest in-row gap when a structure closes the
  view) follows its support. Both sides must end, in 2 consecutive two-sided
  updates (temporal confirmation), and the evidence expires after 2.5 s; the
  corridor ends at the later of the two ends. A one-sided view never creates
  or extends an end, and a one-sided gap is never an end.
* next corridor (last-row evidence): per side, a parallel line one learnt width
  beyond the side line is PRESENT, ABSENT (rays observed through that zone and
  no line) or UNKNOWN (zone not observed, e.g. an opaque barrier).
* front and pivot clearances come from the newest raw scan, including branch
  returns. Gyro bias is learnt only while sensor-observed still.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial import cKDTree

from orchard_perception.scan_corridor import observe_scan_corridor


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class EstimatorConfig:
    # --- nominal calibration (NOT the plant's truth) -------------------------
    lidar_x_in_base_m: float = .18       # SIM_ASSUMED mount (rover.yaml); unmeasured
    footprint_length_m: float = 1.22     # CATALOGUE
    footprint_width_m: float = .70       # CATALOGUE
    pivot_icr_behind_lidar_m: float = .18   # ESTIMATED pooled median 0.18 m (incl. slip)
    pivot_icr_uncertainty_m: float = .25    # ESTIMATED per-event sd 0.25 m
    # --- odometry -------------------------------------------------------------
    icp_max_range_m: float = 12.
    icp_cell_m: float = .12
    icp_iterations: int = 12
    icp_min_pairs: int = 25
    # --- corridor ---------------------------------------------------------------
    corridor_every_n_scans: int = 2
    band_heading_gate_deg: float = 8.
    band_offset_gate_m: float = .35
    band_width_gate_m: float = .5
    band_side_gate_m: float = .3
    band_gain: float = .4
    band_center_gain: float = .25
    band_init_count: int = 3
    band_timeout_s: float = 2.
    band_min_width_m: float = 1.2
    band_max_width_m: float = 8.
    known_width_forget_m: float = 10.        # learnt width expires after this odometry distance without a band
    line_gate_m: float = .3
    line_min_points: int = 8
    acquire_axis_window_deg: float = 50.     # first acquisition: +- around the travel direction
    reacquire_axis_window_deg: float = 20.   # after a loss: +- around the last band axis
    acquire_axis_step_deg: float = 2.
    acquire_cross_bin_m: float = .2
    acquire_along_bin_m: float = .75
    acquire_along_range_m: float = 8.
    acquire_min_coverage: int = 4            # along bins (0.75 m) with returns on a band
    acquire_veto_coverage: int = 2           # returns in >= this many along bins between the lines veto a pair
    acquire_branch_margin_m: float = .7      # canopy zone next to a line, ignored by the veto ...
    acquire_inner_fraction: float = .3       # ... or this fraction of the pair width if larger
    acquire_ambiguity_deg: float = 8.
    acquire_ambiguity_ratio: float = .95
    # --- row end / next corridor -------------------------------------------------
    side_extent_tolerance_m: float = .35
    support_range_m: float = 10.         # matches CorridorConfig.support_range_m
    free_check_range_m: float = 20.      # support resuming within this range cancels an end
    row_end_confirm_updates: int = 2     # consecutive full updates with both sides ended ...
    row_end_valid_s: float = 2.5         # ... each within this age; ends expire after it
    core_margin_m: float = .6            # aisle core = between the lines minus this canopy margin
    cross_structure_min_points: int = 6
    cross_structure_min_span: float = .5  # fraction of the aisle core width
    end_free_m: float = 1.5
    end_gap_margin_m: float = .5
    next_line_gate_m: float = .35
    next_min_coverage: int = 4
    next_min_observed_rays: int = 40
    next_present_votes: int = 3
    next_absent_votes: int = 5
    # --- stillness / IMU ---------------------------------------------------------
    gyro_lowpass_s: float = .25
    vibration_window_s: float = .5
    still_yaw_deg_s: float = 1.5
    still_translation_m: float = .025
    still_rotation_deg: float = .6
    keyframe_translation_m: float = .4
    keyframe_rotation_deg: float = 8.


@dataclass
class Band:
    heading: float          # world (odometry) axis, oriented with travel
    center: float           # centreline offset along the left normal
    width: float
    stamp: float
    full_updates: int = 0
    end_left: float | None = None    # along-axis world coordinate of observed support end
    end_right: float | None = None
    end_stamp: float | None = None


class SensorEstimator:
    def __init__(self, config=EstimatorConfig()):
        self.config = config
        self.imu = deque()
        self.last_imu_stamp = None
        self.gyro_lp = 0.
        self.gyro_bias = 0.
        self.bias_samples = 0
        self.gyro_heading = 0.           # bias-corrected raw gyro integral (not a map yaw)
        self.last_scan_stamp = None
        self.scan_count = 0
        self.keyframe = None             # (points, pose)
        self.history = deque(maxlen=12)  # (stamp, pose, odom_valid) of recent scans
        self.pose = np.zeros(3)          # laser pose in odometry frame at last scan
        self.speed = 0.
        self.speed_samples = deque(maxlen=5)
        self.odom_valid = False
        self.odom_failures = 0
        self.distance = 0.
        self.still_since = None
        self.band = None
        self.candidates = []
        self.known_width = None          # learnt row width survives a band loss ...
        self.band_lost_at_m = None       # ... until the vehicle has moved known_width_forget_m
        self.last_band_heading = None    # learnt axis survives a band loss (re-acquisition prior)
        self.accepted_updates = 0        # monotonic count of accepted corridor updates
        self.front_clearance = math.inf
        self.pivot_clearance = math.inf
        self.side_clearance = math.inf
        self.corridor_reason = 'waiting'
        self.reference_heading = None
        self.end_free = [False, False]
        self.end_streak = 0              # consecutive full updates with both sides ended
        self.next_votes = dict(left=[0, 0], right=[0, 0])   # [present, absent]

    # --- IMU -------------------------------------------------------------
    def on_imu(self, stamp, gyro, accel):
        if self.last_imu_stamp is not None and stamp <= self.last_imu_stamp:
            return
        dt = .05 if self.last_imu_stamp is None else min(.2, stamp - self.last_imu_stamp)
        self.last_imu_stamp = stamp
        self.imu.append((stamp, gyro, accel))
        self.gyro_heading += (gyro - self.gyro_bias) * dt
        while self.imu and stamp - self.imu[0][0] > 3.:
            self.imu.popleft()
        a = 1 - math.exp(-dt / self.config.gyro_lowpass_s)
        self.gyro_lp += (gyro - self.gyro_lp) * a
        if self.is_still(stamp, minimum_s=.5):
            self.bias_samples = min(self.bias_samples + 1, 200)
            self.gyro_bias += (gyro - self.gyro_bias) / self.bias_samples

    def gyro_integral(self, t0, t1):
        total, previous = 0., None
        for stamp, gyro, _ in self.imu:
            if t0 < stamp <= t1:
                step = .05 if previous is None else min(.1, stamp - previous)
                total += (gyro - self.gyro_bias) * step
            previous = stamp
        return total

    def vibration_rms(self, now):
        window = [a for s, _, a in self.imu if now - s <= self.config.vibration_window_s]
        if len(window) < 3:
            return None
        values = np.asarray(window)
        return float(np.sqrt(np.mean(np.sum((values - values.mean(axis=0)) ** 2, axis=1))))

    def yaw_rate(self):
        return self.gyro_lp - self.gyro_bias

    # --- scans -----------------------------------------------------------
    @staticmethod
    def _points(ranges, angle_min, increment, range_max):
        r = np.asarray(ranges, dtype=float)
        angles = angle_min + np.arange(len(r)) * increment
        valid = np.isfinite(r) & (r > 0) & (r < range_max)
        return np.column_stack((r[valid] * np.cos(angles[valid]), r[valid] * np.sin(angles[valid])))

    def _icp(self, reference, current, guess):
        cfg = self.config
        tree = cKDTree(reference)
        x, y, t = guess
        limit = .5
        for _ in range(cfg.icp_iterations):
            c, s = math.cos(t), math.sin(t)
            moved = current @ np.array(((c, s), (-s, c))) + (x, y)
            distance, index = tree.query(moved, distance_upper_bound=limit)
            mask = np.isfinite(distance)
            if mask.sum() < cfg.icp_min_pairs:
                return None
            a, b = current[mask], reference[index[mask]]
            ma, mb = a.mean(axis=0), b.mean(axis=0)
            h = (a - ma).T @ (b - mb)
            t = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
            c, s = math.cos(t), math.sin(t)
            x, y = mb - np.array(((c, -s), (s, c))) @ ma
            limit = max(.15, limit * .8)
        c, s = math.cos(t), math.sin(t)
        distance, _ = tree.query(current @ np.array(((c, s), (-s, c))) + (x, y), distance_upper_bound=.15)
        return (float(x), float(y), float(t)), float(np.isfinite(distance).mean())

    def on_scan(self, stamp, ranges, angle_min, increment, range_max):
        cfg = self.config
        if self.last_scan_stamp is not None and stamp <= self.last_scan_stamp:
            return
        points = self._points(ranges, angle_min, increment, range_max)
        near = points[np.hypot(points[:, 0], points[:, 1]) < cfg.icp_max_range_m]
        _, keep = np.unique(np.floor(near / cfg.icp_cell_m).astype(np.int32), axis=0, return_index=True)
        sparse = near[np.sort(keep)]
        if self.last_scan_stamp is not None and self.keyframe is not None:
            dt = stamp - self.last_scan_stamp
            rotation = self.gyro_integral(self.last_scan_stamp, stamp)
            c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
            predicted = np.array((self.pose[0] + c * self.speed * dt, self.pose[1] + s * self.speed * dt,
                                  wrap(self.pose[2] + rotation)))
            key_points, key_pose = self.keyframe
            result = (self._icp(key_points, sparse, _relative(key_pose, predicted))
                      if len(sparse) >= cfg.icp_min_pairs and dt < .5 else None)
            previous = self.pose
            if result is not None and result[1] >= .5:
                self.pose = _compose(key_pose, result[0])
                self.odom_valid = True
            else:
                # Degraded: gyro rotation only; translation is NOT invented.
                self.pose = np.array((previous[0], previous[1], wrap(previous[2] + rotation)))
                self.odom_valid = False
                self.odom_failures += 1
            step = _relative(previous, self.pose)
            if dt > 0 and self.odom_valid:
                self.speed_samples.append(step[0] / dt)
                self.speed = float(np.median(self.speed_samples))
                self.distance += math.hypot(step[0], step[1])
            relative = _relative(key_pose, self.pose)
            if (not self.odom_valid or math.hypot(relative[0], relative[1]) > cfg.keyframe_translation_m
                    or abs(relative[2]) > math.radians(cfg.keyframe_rotation_deg)):
                self.keyframe = (sparse, self.pose.copy())
        else:
            self.keyframe = (sparse, self.pose.copy())
        self.history.append((stamp, self.pose.copy(), self.odom_valid))
        self._update_stillness(stamp)
        self.last_scan_stamp = stamp
        self.scan_count += 1
        if self.reference_heading is None:
            self.reference_heading = float(self.pose[2])
        self._clearances(points)
        if self.band is None or self.scan_count % cfg.corridor_every_n_scans == 0:
            self._corridor(stamp, _Scan(ranges, angle_min, increment, range_max), points)
        if self.band is not None and stamp - self.band.stamp > cfg.band_timeout_s:
            self.last_band_heading = self.band.heading
            self.band_lost_at_m = self.distance
            self.band = None
            self.candidates = []
            self.corridor_reason = 'corridor_lost'

    def _update_stillness(self, stamp):
        """Still when every recent pose stays within a few cm / tenths of a degree."""
        cfg = self.config
        window = [(t, pose, ok) for t, pose, ok in self.history if stamp - t <= .45]
        current = self.history[-1][1]
        steady = (len(window) >= 4 and all(ok for _, _, ok in window) and all(
            math.hypot(pose[0] - current[0], pose[1] - current[1]) <= cfg.still_translation_m
            and abs(wrap(pose[2] - current[2])) <= math.radians(cfg.still_rotation_deg)
            for _, pose, _ in window))
        if not steady:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = window[0][0]

    def _clearances(self, points):
        cfg = self.config
        bx = points[:, 0] + cfg.lidar_x_in_base_m
        by = points[:, 1]
        half_l, half_w = cfg.footprint_length_m / 2, cfg.footprint_width_m / 2
        ahead = (bx > half_l - .05) & (np.abs(by) <= half_w + .10)
        self.front_clearance = float(bx[ahead].min() - half_l) if ahead.any() else math.inf
        beside = (np.abs(bx) <= half_l) & (np.abs(by) > half_w - .05)
        self.side_clearance = float(np.abs(by[beside]).min() - half_w) if beside.any() else math.inf
        px = points[:, 0] + cfg.pivot_icr_behind_lidar_m
        self.pivot_clearance = float(np.hypot(px, points[:, 1]).min()) if len(points) else math.inf

    def pivot_radius_required(self, margin=.10):
        """Swept radius of the footprint about the uncertain pivot centre (nominal calibration)."""
        cfg = self.config
        icr_x = cfg.lidar_x_in_base_m - cfg.pivot_icr_behind_lidar_m
        reach = max(abs(cfg.footprint_length_m / 2 - icr_x), abs(-cfg.footprint_length_m / 2 - icr_x))
        return math.hypot(reach + cfg.pivot_icr_uncertainty_m, cfg.footprint_width_m / 2) + margin

    # --- corridor acquisition ------------------------------------------------
    def _acquire(self, points):
        """Constrained axis search: nearest band on each side, no intervening band.

        Returns (axis_laser, left, right) or None; sets corridor_reason.
        """
        cfg = self.config
        if self.last_band_heading is not None:
            prior, window = wrap(self.last_band_heading - self.pose[2]), cfg.reacquire_axis_window_deg
        else:
            prior, window = 0., cfg.acquire_axis_window_deg     # laser +x = current travel direction
        pts = points[np.hypot(points[:, 0], points[:, 1]) < cfg.support_range_m]
        if len(pts):
            _, keep = np.unique(np.floor(pts / .1).astype(np.int32), axis=0, return_index=True)
            pts = pts[np.sort(keep)]
        if len(pts) < 2 * cfg.acquire_min_coverage:
            self.corridor_reason = 'insufficient_returns'
            return None
        candidates, strengths = [], []
        for offset in np.arange(-window, window + 1e-9, cfg.acquire_axis_step_deg):
            axis = prior + math.radians(float(offset))
            found, strength = self._axis_pair(pts, axis)
            strengths.append((strength, axis))
            if found is not None:
                candidates.append(found)
        if not candidates:
            self.corridor_reason = 'no_nearest_band_pair_near_travel_axis'
            return None
        # The dominant (sharpest) line axis comes first; another clearly different axis
        # that is nearly as sharp is ambiguous. The pair must lie on the dominant axis:
        # a pair found only at a skewed axis (e.g. while the sharpest lines are vetoed
        # by a gapped row, or a diagonal of a square grid) is refused.
        gate = math.radians(cfg.acquire_ambiguity_deg)
        top, dominant = max(strengths)
        if any(strength >= cfg.acquire_ambiguity_ratio * top and abs(wrap(2 * (axis - dominant))) / 2 > gate
               for strength, axis in strengths):     # line axes compare modulo pi
            self.corridor_reason = 'ambiguous_corridor_axis'
            return None
        near = [c for c in candidates if abs(wrap(c['axis'] - dominant)) <= gate]
        if not near:
            self.corridor_reason = 'pair_not_on_dominant_axis'
            return None
        best = min(near, key=lambda c: (-c['score'], c['residual']))
        return best['axis'], best['left'], best['right']

    def _axis_pair(self, pts, axis):
        cfg = self.config
        c, s = math.cos(axis), math.sin(axis)
        along = pts @ np.array((c, s))
        cross = pts @ np.array((-s, c))
        sel = np.abs(along) <= cfg.acquire_along_range_m
        along, cross = along[sel], cross[sel]
        if len(cross) < 2 * cfg.acquire_min_coverage:
            return None, 0
        cb = np.floor(cross / cfg.acquire_cross_bin_m).astype(int)
        ab = np.floor(along / cfg.acquire_along_bin_m).astype(int)
        lo = cb.min()
        keys = np.unique((cb - lo) * 10_000 + (ab + 5_000))
        cov = np.bincount(keys // 10_000, minlength=cb.max() - lo + 1).astype(int)
        # coverage of a 2-bin (0.4 m) wide band = distinct along bins in bin j-1 or j,
        # i.e. a band centred on the lower edge of bin j
        both = np.unique(np.concatenate([keys, keys + 10_000]))
        cov2 = np.bincount(both // 10_000, minlength=len(cov) + 1)[:len(cov)].astype(int)
        centres = (np.arange(len(cov)) + lo) * cfg.acquire_cross_bin_m

        def nearest(sign):
            order = np.argsort(sign * centres)
            for i in order:
                x = centres[i]
                if sign * x < .5 or cov2[i] < cfg.acquire_min_coverage:
                    continue
                # Snap outwards to the strongest band within 0.6 m (trunk line, not the
                # canopy fringe on the aisle side of it; not a fence just outside the row).
                near = (sign * centres >= sign * x) & (sign * centres <= sign * x + .6)
                j = int(np.flatnonzero(near)[np.argmax(cov2[near])])
                band = np.abs(cross - centres[j]) <= .25
                return float(np.median(cross[band])), int(cov2[j]), float(np.std(cross[band]))
            return None
        side_strength = [int(cov2[mask].max()) if mask.any() else 0
                         for mask in (centres >= .5, centres <= -.5)]
        strength = side_strength[0] + side_strength[1]
        left, right = nearest(1), nearest(-1)
        if left is None or right is None:
            return None, strength
        width = left[0] - right[0]
        if not cfg.band_min_width_m <= width <= cfg.band_max_width_m:
            return None, strength
        # Veto: an intervening band (e.g. a gapped row) between the chosen pair.
        margin = max(cfg.acquire_branch_margin_m, cfg.acquire_inner_fraction * width)
        inner = (cross > right[0] + margin) & (cross < left[0] - margin)
        if len(np.unique(ab[inner])) >= cfg.acquire_veto_coverage:
            return None, strength   # returns between the lines (e.g. a gapped row): not this pair
        return dict(axis=axis, left=left[0], right=right[0], score=left[1] + right[1],
                    residual=left[2] + right[2]), strength

    # --- corridor tracking ---------------------------------------------------
    def _line_fit(self, points, axis, left, right):
        """Shared-slope LS fit of the returns near two laser-frame lines.

        Returns (axis, left, right); a side without enough support is None.
        Branch/trunk returns are only *selected* here, never deleted.
        """
        cfg = self.config
        c, s = math.cos(axis), math.sin(axis)
        along = points @ np.array((c, s))
        cross = points @ np.array((-s, c))
        near = np.abs(along) < 8.
        masks = [(np.abs(cross - offset) < cfg.line_gate_m) & near if offset is not None
                 else np.zeros(len(points), bool) for offset in (left, right)]
        sides = [k for k, m in enumerate(masks) if m.sum() >= cfg.line_min_points]
        if not sides:
            return axis, None, None
        a = np.concatenate([along[masks[k]] for k in sides])
        y = np.concatenate([cross[masks[k]] for k in sides])
        columns = [a] + [np.concatenate([np.full(int(masks[j].sum()), float(j == k)) for j in sides])
                         for k in sides]
        design = np.column_stack(columns)
        solution = None
        for _ in range(2):
            solution, *_ = np.linalg.lstsq(design, y, rcond=None)
            keep = np.abs(design @ solution - y) < .18
            if keep.sum() < cfg.line_min_points or any(design[keep, 1 + i].sum() < 4 for i in range(len(sides))):
                return axis, None, None
            design, y = design[keep], y[keep]
        slope = float(solution[0])
        if abs(slope) > math.tan(math.radians(4)):
            return axis, None, None
        angle = math.atan(slope)
        offsets = [None, None]
        for index, k in enumerate(sides):
            offsets[k] = float(solution[1 + index] * math.cos(angle))
        return axis + angle, offsets[0], offsets[1]

    def _side_extents(self, points, axis, left, right):
        """Per-side support end (laser along) and whether observed free space follows it.

        The end is the 97th percentile of line support inside support_range_m,
        excluding any structure that spans the aisle core. The side has ENDED
        ("free") only if the space after it, up to the next support on the same
        line, a cross-aisle structure, or the free_check_range_m visibility limit,
        is longer than end_free_m and than the robust (p75) in-row gap + margin.
        """
        cfg = self.config
        c, s = math.cos(axis), math.sin(axis)
        along = points @ np.array((c, s))
        cross = points @ np.array((-s, c))
        rng = np.hypot(points[:, 0], points[:, 1])
        inside = rng < cfg.support_range_m
        far = rng < cfg.free_check_range_m
        front = None
        if left is not None and right is not None:
            centre, half = (left + right) / 2, (left - right) / 2
            core_half = max(.3, half - cfg.core_margin_m)
            core = far & (np.abs(cross - centre) < core_half) & (along > .3)
            if core.sum() >= cfg.cross_structure_min_points:
                a_core, x_core = along[core], cross[core]
                for start in np.sort(a_core):
                    slab = (a_core >= start) & (a_core <= start + .6)
                    if (slab.sum() >= cfg.cross_structure_min_points
                            and np.ptp(x_core[slab]) >= cfg.cross_structure_min_span * 2 * core_half):
                        front = float(start)
                        break
        ends, free = [], []
        for offset in (left, right):
            if offset is None:
                ends.append(None)
                free.append(False)
                continue
            line = np.abs(cross - offset) < cfg.side_extent_tolerance_m
            if front is not None:
                line &= along < front - .3        # a cross-aisle structure is not row support
            mask = line & inside
            if mask.sum() < 4:
                ends.append(None)
                free.append(False)
                continue
            a = np.sort(along[mask])
            end = float(np.percentile(a, 97))
            resumed = line & far & (along > end + .3)
            limit = math.sqrt(max(0., cfg.free_check_range_m ** 2 - offset ** 2))
            stop_at = min(limit, float(along[resumed].min()) if resumed.any() else math.inf,
                          front if front is not None else math.inf)
            gap = stop_at - end
            behind = a[(a >= -cfg.acquire_along_range_m) & (a <= end)]
            gaps = np.diff(behind)
            gaps = gaps[gaps > .3]
            # Open view: a robust (p75) in-row gap, so one missing trunk does not veto
            # every end. View closed by a cross-aisle structure (fence/wall): ambiguous
            # with a mid-row wall after missing trunks, so the gap must beat the
            # LARGEST in-row gap (a false row end is worse than a missed one).
            if not len(gaps):
                in_row = 0.
            elif front is not None and stop_at == front:
                in_row = float(gaps.max())
            else:
                in_row = float(np.percentile(gaps, 75))
            needed = max(cfg.end_free_m, in_row + cfg.end_gap_margin_m)
            ends.append(end)
            free.append(gap >= needed)
        return ends, free, front

    def _next_corridor_votes(self, points, axis, left, right, width):
        """Per side: is a parallel line one width beyond the side line observed?"""
        cfg = self.config
        c, s = math.cos(axis), math.sin(axis)
        along = points @ np.array((c, s))
        cross = points @ np.array((-s, c))
        rng = np.hypot(points[:, 0], points[:, 1])
        for name, offset, sign in (('left', left, 1.), ('right', right, -1.)):
            z0, z1 = sign * offset + .5 * width, sign * offset + 1.5 * width   # distance beyond, signed side
            sc = sign * cross
            # Only alongside the observed part of THIS corridor's line: beyond the
            # row start/end the absence of a next row says nothing.
            own = (np.abs(cross - offset) < cfg.side_extent_tolerance_m) & (rng < cfg.support_range_m)
            if own.sum() < 4:
                continue
            lo = max(-cfg.acquire_along_range_m, float(along[own].min()))
            hi = min(cfg.acquire_along_range_m, float(along[own].max()))
            if hi - lo < 2.:
                continue
            zone = (sc > z0) & (sc < z1) & (along >= lo) & (along <= hi) & (rng < cfg.support_range_m)
            present = False
            if zone.sum() >= cfg.next_min_coverage:
                expected = sign * offset + width
                near = zone & (np.abs(sc - expected) < .5 * width - .1)
                if near.sum():
                    peak = float(np.median(sc[near]))
                    band = zone & (np.abs(sc - peak) < cfg.next_line_gate_m)
                    present = len(np.unique(np.floor(along[band] / .75))) >= cfg.next_min_coverage
            if present:
                self.next_votes[name][0] += 1
                continue
            # Observed through the zone: rays (incl. no-return) that reach its far edge.
            angles = self._scan_angles
            ranges = self._scan_ranges
            direction = sign * np.sin(angles - axis)
            usable = direction > .2
            reach = z1 / np.maximum(direction, 1e-6)
            along_at = reach * np.cos(angles - axis)
            through = usable & ((~np.isfinite(ranges)) | (ranges <= 0) | (ranges >= reach)) \
                & (along_at >= lo) & (along_at <= hi) & (reach < cfg.support_range_m + 2)
            zone_hits = int(zone.sum())
            if int(through.sum()) >= cfg.next_min_observed_rays and zone_hits < cfg.next_min_coverage:
                self.next_votes[name][1] += 1

    def _world(self, axis, left, right):
        """Laser-frame lines -> world heading oriented with travel; offsets follow."""
        heading = wrap(self.pose[2] + axis)
        reference = self.band.heading if self.band is not None else self.reference_heading
        if abs(wrap(heading - reference)) > math.pi / 2:
            heading, axis = wrap(heading + math.pi), wrap(axis + math.pi)
            left, right = (None if right is None else -right), (None if left is None else -left)
        return heading, axis, left, right

    def _detected_lines(self, observation, points):
        axis, left, right = self._line_fit(points, observation.heading_axis_laser_rad,
                                           observation.left_offset_m, observation.right_offset_m)
        if left is None or right is None:
            axis, left, right = (observation.heading_axis_laser_rad, observation.left_offset_m,
                                 observation.right_offset_m)
        return self._world(axis, left, right)

    def _corridor(self, stamp, scan, points):
        r = np.asarray(scan.ranges, dtype=float)
        self._scan_ranges = r
        self._scan_angles = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        if self.band is None:
            found = self._acquire(points)
            if found is None:
                self.candidates = []
                return
            axis, left, right = self._line_fit(points, *found)
            if left is None or right is None:
                axis, left, right = found
            self._initialise(stamp, *self._world(axis, left, right), points)
            return
        observation = observe_scan_corridor(scan)
        if observation.valid and self._apply(stamp, *self._detected_lines(observation, points),
                                             points, 'candidate'):
            return
        # Detector ambiguous/inconsistent: fit raw returns near the PREDICTED lines.
        band = self.band
        x, y, theta = self.pose
        normal = np.array((-math.sin(band.heading), math.cos(band.heading)))
        lateral = float(np.array((x, y)) @ normal)
        axis, left, right = self._line_fit(points, wrap(band.heading - theta),
                                           band.center + band.width / 2 - lateral,
                                           band.center - band.width / 2 - lateral)
        if left is None and right is None:
            self.corridor_reason = 'no_support_near_tracked_lines'
            return
        if not self._apply(stamp, wrap(theta + axis), axis, left, right, points, 'guided'):
            self.corridor_reason = 'inconsistent_corridor_rejected'

    def _apply(self, stamp, heading, axis, left, right, points, source):
        """Gated full (both sides) or one-sided update of the tracked band."""
        cfg, band = self.config, self.band
        x, y, _ = self.pose
        if abs(wrap(heading - band.heading)) >= math.radians(cfg.band_heading_gate_deg):
            return False
        normal = np.array((-math.sin(band.heading), math.cos(band.heading)))
        lateral = float(np.array((x, y)) @ normal)
        pred_left = band.center + band.width / 2 - lateral
        pred_right = band.center - band.width / 2 - lateral
        along0 = float(np.array((x, y)) @ np.array((math.cos(heading), math.sin(heading))))
        g, gc = cfg.band_gain, cfg.band_center_gain
        if left is not None and right is not None:
            width = left - right
            offset_pred = -(pred_left + pred_right) / 2
            offset_obs = -(left + right) / 2
            if (abs(offset_obs - offset_pred) < cfg.band_offset_gate_m
                    and abs(width - band.width) < cfg.band_width_gate_m
                    and cfg.band_min_width_m <= width <= cfg.band_max_width_m):   # no slow drift to 0
                band.heading = wrap(band.heading + g * wrap(heading - band.heading))
                band.width += g * (width - band.width)
                normal = np.array((-math.sin(band.heading), math.cos(band.heading)))
                lateral = float(np.array((x, y)) @ normal)
                band.center = lateral - ((1 - gc) * offset_pred + gc * offset_obs)
                band.stamp = stamp
                band.full_updates += 1
                self.accepted_updates += 1
                self.known_width = band.width
                self.last_band_heading = band.heading
                self._update_ends(band, self._side_extents(points, axis, left, right), along0, stamp)
                self._next_corridor_votes(points, axis, left, right, band.width)
                self.corridor_reason = 'tracked_full_' + source
                return True
        # One-sided: the matched line keeps the learnt width.
        for observed, predicted, sign in ((left, pred_left, -1), (right, pred_right, 1)):
            if observed is not None and abs(observed - predicted) < cfg.band_side_gate_m:
                band.heading = wrap(band.heading + g / 2 * wrap(heading - band.heading))
                normal = np.array((-math.sin(band.heading), math.cos(band.heading)))
                lateral = float(np.array((x, y)) @ normal)
                center_obs = lateral + observed + sign * band.width / 2
                band.center += gc * (center_obs - band.center)
                band.stamp = stamp
                self.accepted_updates += 1
                # A one-sided view never yields or extends a row end (the other side is
                # unknown); the last two-sided evidence simply ages out (row_end_valid_s).
                self.corridor_reason = 'tracked_one_side_' + source
                return True
        return False

    def _update_ends(self, band, extents, along0, stamp):
        """Row-end evidence comes ONLY from full (two-sided) updates."""
        ends, free, _ = extents
        recent = band.end_stamp is not None and stamp - band.end_stamp <= self.config.row_end_valid_s
        both = all(free) and None not in ends
        self.end_streak = (self.end_streak + 1 if recent else 1) if both else 0
        band.end_left = None if ends[0] is None else along0 + ends[0]
        band.end_right = None if ends[1] is None else along0 + ends[1]
        band.end_stamp = stamp
        self.end_free = list(free)

    def _initialise(self, stamp, heading, axis, left, right, points):
        cfg = self.config
        width = left - right
        if (self.known_width is not None and self.band_lost_at_m is not None
                and self.distance - self.band_lost_at_m > cfg.known_width_forget_m):
            self.known_width = None      # another area: rows may have a different width
            self.last_band_heading = None
        if not cfg.band_min_width_m <= width <= cfg.band_max_width_m or (
                self.known_width is not None and abs(width - self.known_width) > cfg.band_width_gate_m):
            self.candidates = []
            self.corridor_reason = 'implausible_initial_width'
            return
        x, y, _ = self.pose
        normal = np.array((-math.sin(heading), math.cos(heading)))
        center = float(np.array((x, y)) @ normal) + (left + right) / 2
        if self.candidates:
            h, c, w, _ = self.candidates[-1]
            if (abs(wrap(heading - h)) > math.radians(cfg.band_heading_gate_deg)
                    or abs(center - c) > .4 or abs(width - w) > cfg.band_width_gate_m):
                self.candidates = []
        self.candidates.append((heading, center, width, stamp))
        self.corridor_reason = 'initialising'
        if len(self.candidates) >= cfg.band_init_count:
            headings = [h for h, *_ in self.candidates]
            mean_heading = math.atan2(sum(map(math.sin, headings)), sum(map(math.cos, headings)))
            self.band = Band(mean_heading, float(np.mean([c for _, c, _, _ in self.candidates])),
                             float(np.mean([w for _, _, w, _ in self.candidates])), stamp, 1)
            self.known_width = self.band.width
            self.last_band_heading = self.band.heading
            self.accepted_updates += 1
            along0 = float(np.array((x, y)) @ np.array((math.cos(heading), math.sin(heading))))
            self._update_ends(self.band, self._side_extents(points, axis, left, right), along0, stamp)
            self.candidates = []
            self.corridor_reason = 'tracked_full_init'

    # --- outputs ------------------------------------------------------------
    def still_duration(self, now):
        """Seconds of sensor-observed stillness (0 when moving, stale or not steady)."""
        if self.still_since is None or self.last_scan_stamp is None or now - self.last_scan_stamp >= .5:
            return 0.
        if abs(self.gyro_lp - self.gyro_bias) > math.radians(self.config.still_yaw_deg_s):
            return 0.
        return max(0., self.last_scan_stamp - self.still_since)

    def is_still(self, now, minimum_s=.4):
        if self.still_since is None or self.last_scan_stamp is None:
            return False
        return self.still_duration(now) >= minimum_s and now - self.last_scan_stamp < .5

    def next_corridor(self):
        cfg = self.config
        result = {}
        for side, (present, absent) in self.next_votes.items():
            if present >= cfg.next_present_votes:
                result[side] = 'PRESENT'
            elif absent >= cfg.next_absent_votes and present == 0:
                result[side] = 'ABSENT'
            else:
                result[side] = 'UNKNOWN'
        return result

    def snapshot(self, now):
        cfg = self.config
        lidar_age = math.inf if self.last_scan_stamp is None else now - self.last_scan_stamp
        imu_age = math.inf if self.last_imu_stamp is None else now - self.last_imu_stamp
        theta = self.pose[2]
        if self.last_scan_stamp is not None:
            theta = wrap(theta + self.gyro_integral(self.last_scan_stamp, now))
        # Latency compensation: advance the scan pose with gyro and LiDAR speed
        # along the wrapped mid-heading (no naive mean across +-pi).
        horizon = 0. if self.last_scan_stamp is None else max(0., now - self.last_scan_stamp)
        lx, ly = advance(self.pose, theta, self.speed, horizon)
        bx = lx - cfg.lidar_x_in_base_m * math.cos(theta)
        by = ly - cfg.lidar_x_in_base_m * math.sin(theta)
        still_s = self.still_duration(now)
        result = dict(lidar_age_s=lidar_age, imu_age_s=imu_age, yaw_rate=self.yaw_rate(),
                      gyro_heading=self.gyro_heading, gyro_bias=self.gyro_bias,
                      speed=self.speed, odom_valid=self.odom_valid, lidar_heading=float(self.pose[2]),
                      distance_m=self.distance, still=still_s > 0., still_s=still_s,
                      vibration_rms=self.vibration_rms(now),
                      front_clearance_m=self.front_clearance - max(0., self.speed) * horizon,
                      side_clearance_m=self.side_clearance,
                      pivot_clearance_m=self.pivot_clearance,
                      pivot_radius_required_m=self.pivot_radius_required(),
                      corridor_valid=False, corridor_reason=self.corridor_reason,
                      corridor_updates=self.accepted_updates, next_corridor=self.next_corridor())
        band = self.band
        if band is not None:
            normal = (-math.sin(band.heading), math.cos(band.heading))
            direction = (math.cos(band.heading), math.sin(band.heading))
            lateral = bx * normal[0] + by * normal[1] - band.center
            along = bx * direction[0] + by * direction[1]
            ends = [None if e is None else e - along for e in (band.end_left, band.end_right)]
            # A side is "ended" only when its support stops inside the sensed
            # range and observed free space follows it (not a cross-aisle wall),
            # on BOTH sides; the corridor ends where the LATER of the two ends
            # (staggered rows legitimately end a part-spacing apart).
            limit = cfg.support_range_m - 1.5 - cfg.lidar_x_in_base_m
            fresh = band.end_stamp is not None and now - band.end_stamp <= cfg.row_end_valid_s
            both_end = (fresh and self.end_streak >= cfg.row_end_confirm_updates
                        and all(e is not None and e < limit and f for e, f in zip(ends, self.end_free)))
            result.update(corridor_valid=True, heading_error=wrap(band.heading - theta),
                          lateral_offset=lateral, corridor_width=band.width,
                          corridor_age_s=now - band.stamp, corridor_full_updates=band.full_updates,
                          side_ends_ahead_m=ends, side_end_free=list(self.end_free),
                          row_end_ahead_m=max(ends) if both_end else None,
                          end_age_s=None if band.end_stamp is None else now - band.end_stamp)
        return result


def advance(pose, theta, speed, horizon):
    """Advance the scan pose by speed*horizon along the WRAPPED mid-heading (pose yaw -> theta)."""
    mid = wrap(pose[2] + .5 * wrap(theta - pose[2]))
    return pose[0] + math.cos(mid) * speed * horizon, pose[1] + math.sin(mid) * speed * horizon


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array((a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])))


def _relative(a, b):
    """Pose b expressed in the frame of pose a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2]))


class _Scan:
    __slots__ = ('ranges', 'angle_min', 'angle_increment', 'range_min', 'range_max')

    def __init__(self, ranges, angle_min, increment, range_max):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = increment
        self.range_min = .05
        self.range_max = range_max
