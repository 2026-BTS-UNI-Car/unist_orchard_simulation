"""ROS-free sensor adapters shared by the row executor and its shadow replay.

ScanAdapter   real RPLIDAR S2 LaserScan -> the SIM estimator's laser frame (+x = travel).
ImuYawRate    raw BNO085 gyro/accel -> gravity-projected yaw rate (causal).
NearSpeckleFilter (opt-in, default OFF) drops isolated near returns before the estimator.

Conventions (same as resume-audit-20261013/sim-review/sim2real/replay.py, NOTES/WEEKEND_PLAN.txt 1.3):
  * The travel axis is -179.08 deg in the real scan frame (orchard-realdata-audit/20261010-field-manual/
    lidar/LIDAR_RESPONSE.json, ESTIMATED from straight forward driving, p10/p90 -2.7/+2.75 deg). The range
    array is kept as is; only angle_min is rotated by +179.08 deg.
  * 0 / NaN / < range_min / >= range_max = no return -> inf (the estimator ignores non-finite ranges).
  * An optional per-beam body mask: a return nearer than mask_r[i] on beam i is removed (-> inf).
  * observe_scan_corridor rejects a scan with n*inc > 2*pi + 1.01*inc; the end beams are trimmed and counted.
  * yaw rate = gyro . up, up = EMA (tau 1 s) of the accelerometer specific force, sign +1.
Nothing here commands a motor.
"""
import math

import numpy as np

LIDAR_YAW_OFFSET_DEG = 179.08     # ESTIMATED: scan-frame travel axis -179.08 deg
SCAN_LATENCY_S = .08              # scan stamp = receive - this (replay shift range 0.005-0.15 s)
RANGE_MIN_M = .05                 # RPLIDAR S2 range_min in the real LaserScan


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ScanAdapter:
    def __init__(self, yaw_offset_deg=LIDAR_YAW_OFFSET_DEG, mask_r=None, range_min=RANGE_MIN_M,
                 speckle=None, transient=None):
        self.yaw_offset = math.radians(yaw_offset_deg)
        self.mask_r = None if mask_r is None else np.asarray(mask_r, dtype=np.float64)
        self.range_min = float(range_min)
        self.speckle = speckle
        self.transient = transient

    def convert(self, ranges, angle_min, increment, range_max):
        """Return (ranges [m, inf = none], angle_min', increment, range_max, stats)."""
        r = np.array(ranges, dtype=np.float64)          # always a copy: the caller's scan is never edited
        invalid = ~np.isfinite(r) | (r < self.range_min) | (r >= range_max)
        r[invalid] = np.inf
        stats = dict(beams=int(len(r)), invalid=int(invalid.sum()), masked=0, trimmed=0, speckle=0, transient=0,
                     mask_mismatch=False)
        if self.mask_r is not None:
            if len(self.mask_r) != len(r):
                stats['mask_mismatch'] = True               # beam layout changed: the mask is meaningless
            else:
                hit = r < self.mask_r
                stats['masked'] = int(hit.sum())
                r[hit] = np.inf
        if self.speckle is not None:
            stats['speckle'] = self.speckle.apply(r)
        if self.transient is not None:
            stats['transient'] = self.transient.apply(r, angle_min, increment)
        limit = 2 * math.pi + 1.01 * increment
        if len(r) * increment > limit:
            keep = int(limit // increment)
            while keep * increment > limit:
                keep -= 1
            stats['trimmed'] = int(len(r) - keep)
            r = r[:keep]
        return r, wrap(angle_min + self.yaw_offset), float(increment), float(range_max), stats


class NearSpeckleFilter:
    """Opt-in: remove isolated near returns (no other near return within +-window beams).

    A real near obstacle (trunk, person, post) at < near_m subtends many 0.11 deg beams; a single-beam
    return inside it is a speckle (dust, grass blade, mixed pixel). Default OFF in every config.
    """
    def __init__(self, near_m=1.2, window=2, min_neighbours=1, tolerance_m=.15):
        self.near_m, self.window, self.min_neighbours, self.tolerance_m = near_m, window, min_neighbours, tolerance_m

    def apply(self, r):
        near = np.nonzero(np.isfinite(r) & (r < self.near_m))[0]
        removed = []
        for i in near:
            lo, hi = max(0, i - self.window), min(len(r), i + self.window + 1)
            neighbours = np.abs(r[lo:hi] - r[i]) <= self.tolerance_m
            if int(neighbours.sum()) - 1 < self.min_neighbours:
                removed.append(i)
        if removed:
            r[np.asarray(removed)] = np.inf
        return len(removed)


class NearTransientFilter:
    """Opt-in (default OFF): a return nearer than near_m survives only if the PREVIOUS scan had a return within
    tol_m of it (same rotated laser frame). Real obstacles persist from scan to scan (0.1 s, <= 0.1 m of travel at
    1 m/s); the observed false front stops on the 10/10 records were one-scan clusters of 5-14 identical
    returns (one physical return repeated over adjacent beams) that are absent in the scans before and after.
    Cost: a new near obstacle is accepted one scan (0.1 s) later. The comparison uses the previous scan's
    UNFILTERED near returns, so a persistent object is kept from its second scan on.
    """
    def __init__(self, near_m=2., tol_m=.35, confirm_scans=1, tol_step_m=.1):
        from scipy.spatial import cKDTree       # scipy is already an estimator dependency
        self._tree = cKDTree
        self.near_m, self.tol_m = float(near_m), float(tol_m)
        self.confirm_scans, self.tol_step_m = int(confirm_scans), float(tol_step_m)
        self.history = []                       # unfiltered near returns of the previous scans, newest first

    def apply(self, r, angle_min, increment):
        """confirm_scans previous scans must each hold a return within tol_m (+tol_step_m per older scan)."""
        idx = np.nonzero(np.isfinite(r) & (r < self.near_m))[0]
        a = angle_min + idx * increment
        pts = np.column_stack((r[idx] * np.cos(a), r[idx] * np.sin(a)))
        removed = 0
        if len(self.history) >= self.confirm_scans and len(idx):
            keep = np.ones(len(idx), dtype=bool)
            for age, prev in enumerate(self.history[:self.confirm_scans]):
                if not len(prev):
                    keep[:] = False
                    break
                d, _ = self._tree(prev).query(pts, distance_upper_bound=self.tol_m + age * self.tol_step_m)
                keep &= np.isfinite(d)
            removed = int((~keep).sum())
            r[idx[~keep]] = np.inf
        self.history.insert(0, pts)
        del self.history[self.confirm_scans:]
        return removed


class ImuYawRate:
    """Causal gravity projection: up = EMA(accel, tau), yaw_rate = sign * gyro . up / |up|."""
    def __init__(self, tau_s=1., sign=1.):
        self.tau_s, self.sign = float(tau_s), float(sign)
        self.state = None
        self.last_t = None

    def update(self, t, gyro_xyz, acc_xyz):
        acc = np.asarray(acc_xyz, dtype=np.float64)
        if self.state is None:
            self.state = acc.copy()
            dt = .05
        else:
            dt = min(.2, max(1e-3, t - self.last_t))
        self.last_t = t
        a = 1 - math.exp(-dt / self.tau_s)
        self.state += (acc - self.state) * a
        norm = float(np.linalg.norm(self.state))
        if not norm > 0:
            return 0.
        return self.sign * float(np.dot(np.asarray(gyro_xyz, dtype=np.float64), self.state)) / norm

    def tilt_deg(self):
        if self.state is None:
            return None
        return math.degrees(math.acos(max(-1., min(1., self.state[2] / np.linalg.norm(self.state)))))
