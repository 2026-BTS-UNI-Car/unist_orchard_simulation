"""LiDAR motion per scan, exactly as orchard-realdata-audit/20261010-field-manual/scripts/lidar_response.py:
centred +-2 scan difference, scans i-2, i, i+2 all 'good' (inlier >= 0.5, rms <= 0.08, cond <= 60), 0.25 <= dt <= 0.6 s,
velocity in the scan-i LiDAR frame; theta unwrapped with non-finite poses set to 0 (the reference rule).
The travel axis is the circular mean direction of settled straight-forward samples faster than 0.2 m/s; it is
estimated per day (10/3, 10/10) because the two days are separate ICP runs.
"""
import math

import numpy as np

from field_calib import timeline as tl

HALF = 2
GOOD = dict(inlier=.5, rms=.08, cond=60.)


def good_mask(lidar):
    pose, qual = lidar['pose'], lidar['quality']
    return (np.isfinite(pose[:, 0]) & (qual[:, 0] >= GOOD['inlier']) & (qual[:, 1] <= GOOD['rms'])
            & (qual[:, 3] <= GOOD['cond']))


class Motion:
    """Per-scan arrays for one recording."""

    def __init__(self, rec):
        lid = rec.lidar
        self.rec = rec
        self.t = lid['t']
        self.pose = lid['pose']
        self.good = good_mask(lid)
        self.theta = np.unwrap(np.where(np.isfinite(self.pose[:, 2]), self.pose[:, 2], 0.))
        n = len(self.t)
        self.vx = np.full(n, np.nan)
        self.vy = np.full(n, np.nan)
        self.w = np.full(n, np.nan)
        if n > 2 * HALF:
            i = np.arange(HALF, n - HALF)
            a, b = i - HALF, i + HALF
            dtt = self.t[b] - self.t[a]
            ok = self.good[a] & self.good[b] & self.good[i] & (dtt >= .25) & (dtt <= .6)
            ii, aa, bb, dd = i[ok], a[ok], b[ok], dtt[ok]
            c, s = np.cos(self.theta[ii]), np.sin(self.theta[ii])
            dx = self.pose[bb, 0] - self.pose[aa, 0]
            dy = self.pose[bb, 1] - self.pose[aa, 1]
            self.vx[ii] = (c * dx + s * dy) / dd
            self.vy[ii] = (-s * dx + c * dy) / dd
            self.w[ii] = (self.theta[bb] - self.theta[aa]) / dd
        self.valid = np.isfinite(self.vx)
        self.spd = np.hypot(self.vx, self.vy)
        m = rec.motor
        # latest status strictly before the scan (lidar_response: searchsorted(mt, t) - 1)
        self.k = np.searchsorted(m.t, self.t) - 1
        has = self.k >= 0
        kk = np.clip(self.k, 0, len(m) - 1)
        self.req_l = np.where(has, m.req_l[kk], np.nan)
        self.req_r = np.where(has, m.req_r[kk], np.nan)
        self.settled = np.array([tl.is_settled(m, int(k)) for k in self.k], dtype=bool)
        self.status_age = np.where(has, self.t - m.t[kk], np.nan)
        self.axis = None
        self.u = None

    def straight_fwd(self):
        return (self.req_l == self.req_r) & (self.req_l > 0)

    def set_axis(self, axis_rad):
        self.axis = axis_rad
        fx, fy = math.cos(axis_rad), math.sin(axis_rad)
        self.u = self.vx * fx + self.vy * fy
        self.lat = -self.vx * fy + self.vy * fx
        self.yaw_dps = np.degrees(self.w)

    def axis_vector(self, i):
        """Travel direction in the chain frame at scan i (from_release.py)."""
        fx, fy = math.cos(self.axis), math.sin(self.axis)
        th = self.theta[i]
        return np.array([math.cos(th) * fx - math.sin(th) * fy, math.sin(th) * fx + math.cos(th) * fy])

    def position_at(self, t, max_gap=.35):
        """Chain-frame (x, y) interpolated at time t when the neighbouring scans are finite and close; else None."""
        j = int(np.searchsorted(self.t, t))
        if j <= 0 or j >= len(self.t):
            return None
        a, b = j - 1, j
        if not (np.isfinite(self.pose[a, 0]) and np.isfinite(self.pose[b, 0])):
            return None
        if self.t[b] - self.t[a] > max_gap:
            return None
        f = (t - self.t[a]) / max(self.t[b] - self.t[a], 1e-9)
        return self.pose[a, :2] + f * (self.pose[b, :2] - self.pose[a, :2])


def travel_axis(motions):
    """lidar_response.py section 2 over the given recordings: (axis_rad, samples, deviation p10/p50/p90 deg)."""
    angs = []
    for m in motions:
        sel = m.valid & m.settled & m.straight_fwd() & (m.spd > .2)
        angs.append(np.arctan2(m.vy[sel], m.vx[sel]))
    ang = np.concatenate(angs) if angs else np.zeros(0)
    if not len(ang):
        return math.pi, 0, None
    mean = math.atan2(np.sin(ang).mean(), np.cos(ang).mean())
    dev = np.degrees(np.angle(np.exp(1j * (ang - mean))))
    return mean, int(len(ang)), [round(float(np.percentile(dev, p)), 2) for p in (10, 50, 90)]


def imu_yaw(rec):
    """Gravity-projected gyro (lidar_response.py imu_yaw): (receipt times s, yaw rate deg/s); empty when no IMU."""
    imu = rec.imu
    if len(imu) < 5:
        return np.zeros(0), np.zeros(0)
    t = imu[:, 0]
    acc = imu[:, 5:8]
    dt = float(np.median(np.diff(t)))
    if dt < 1e-3:
        dt = max(float(t[-1] - t[0]) / (len(t) - 1), 1e-3)
    win = max(1, int(round(2. / dt)))
    k = np.ones(win) / win
    lp = np.column_stack([np.convolve(acc[:, i], k, mode='same') for i in range(3)])
    up = lp / np.maximum(np.linalg.norm(lp, axis=1, keepdims=True), 1e-9)
    w = np.einsum('ij,ij->i', imu[:, 2:5], up)
    return t, np.degrees(w)
