"""Offline 2-D LiDAR odometry for the calibration analyser (no IMU, PWM or GNSS input).

Method = orchard-realdata-audit/20261010-field-manual/scripts/lidar_motion.py (the reference behind
LIDAR_RESPONSE.json and drive-evidence.txt): voxelised points 0.35-25 m, point-to-line ICP against a
keyframe (new keyframe after 0.5 m or 15 deg or a failed match), constant-velocity initial guess, Huber
weights, 25 iterations with the match threshold shrinking 0.6 -> 0.12 m. Not deskewed, LiDAR frame
(the travel axis is estimated separately), so these are LiDAR-frame reference motions, not body truth.

Differences to the reference script (bounded run time on the Jetson):
  * only the requested windows of scans are matched (each window is its own chain);
  * a long window is cut into chunks of `chunk_scans` scans that overlap by one scan and are matched in
    parallel worker processes; chunk poses are composed back into one chain. Each chunk starts a new
    keyframe, so results differ from one continuous chain only by the keyframe placement.
"""
from concurrent.futures import ProcessPoolExecutor
import math
import os

import numpy as np

VOXEL = .05
MIN_RANGE, MAX_RANGE = .35, 25.
KF_DIST, KF_ANGLE = .5, math.radians(15)
MIN_POINTS = 60
GOOD = dict(inlier=.5, rms=.08, cond=60.)    # LIDAR_RESPONSE.json quality_thresholds
HALF = 2                                      # +-2 scans velocity window (lidar_response.py)
MAX_WORKERS = 8


def _cKDTree():
    from scipy.spatial import cKDTree     # scipy >= 1.8 (the executor's vendored estimator needs it too)
    return cKDTree


def points_from(ranges_mm, cos_a, sin_a):
    r = ranges_mm.astype(np.float64) / 1000.
    ok = (ranges_mm > 0) & (r >= MIN_RANGE) & (r <= MAX_RANGE)
    p = np.column_stack([r[ok] * cos_a[ok], r[ok] * sin_a[ok]])
    if len(p) == 0:
        return p
    keys = np.floor(p / VOXEL).astype(np.int64)
    _, idx = np.unique(keys[:, 0] * 1_000_003 + keys[:, 1], return_index=True)
    return p[np.sort(idx)]


def normals_for(points, tree, k=8):
    _, nn = tree.query(points, k=min(k, len(points)))
    nbr = points[nn]
    centred = nbr - nbr.mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', centred, centred) / nbr.shape[1]
    w, v = np.linalg.eigh(cov)
    return v[:, :, 0], 1. - w[:, 0] / np.maximum(w[:, 1], 1e-12)


def transform(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]]) + pose[:2]


def icp(src, ref, init, iters=25):
    """Pose (x, y, theta) mapping src into ref's frame, or None."""
    pts, tree, normal, linear = ref
    pose = np.array(init, dtype=np.float64)
    info = None
    for thr in np.linspace(.6, .12, iters):
        moved = transform(src, pose)
        d, j = tree.query(moved, distance_upper_bound=thr)
        ok = np.isfinite(d) & (linear[np.minimum(j, len(pts) - 1)] > .3)
        if ok.sum() < 30:
            return None
        p, q, n = moved[ok], pts[j[ok]], normal[j[ok]]
        r = np.einsum('ij,ij->i', n, p - q)
        rel = p - pose[:2]
        jac = np.column_stack([n[:, 0], n[:, 1], n[:, 0] * (-rel[:, 1]) + n[:, 1] * rel[:, 0]])
        w = np.where(np.abs(r) <= .05, 1., .05 / np.maximum(np.abs(r), 1e-12))
        h = jac.T @ (jac * w[:, None])
        g = jac.T @ (w * r)
        try:
            delta = -np.linalg.solve(h, g)
        except np.linalg.LinAlgError:
            return None
        pose += delta
        info = (ok, r, h)
        if np.abs(delta[:2]).max() < 1e-4 and abs(delta[2]) < 1e-5:
            break
    ok, r, h = info
    eig = np.linalg.eigvalsh(h[:2, :2])
    return dict(pose=pose, inlier=float(ok.mean()), rms=float(np.sqrt(np.mean(r ** 2))),
                pairs=int(ok.sum()), cond=float(eig[1] / max(eig[0], 1e-9)))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]])


def match_chain(ranges_mm, angle_min, increment):
    """One keyframe chain over consecutive scans: poses (n, 3) in the first scan's frame, quality (n, 4)."""
    cKDTree = _cKDTree()
    n = len(ranges_mm)
    angles = angle_min + np.arange(ranges_mm.shape[1]) * increment
    cos_a, sin_a = np.cos(angles), np.sin(angles)
    poses = np.full((n, 3), np.nan)
    quality = np.full((n, 4), np.nan)        # inlier, rms, pairs, translation conditioning
    if n == 0:
        return poses, quality

    def ref_of(points):
        tree = cKDTree(points)
        normal, linear = normals_for(points, tree)
        return points, tree, normal, linear

    kf_pts = points_from(ranges_mm[0], cos_a, sin_a)
    poses[0] = 0.
    if len(kf_pts) < MIN_POINTS:
        return poses, quality
    kf = ref_of(kf_pts)
    kf_pose = np.zeros(3)
    rel_prev = np.zeros(3)
    rel_vel = np.zeros(3)
    for i in range(1, n):
        src = points_from(ranges_mm[i], cos_a, sin_a)
        if len(src) < MIN_POINTS:
            continue
        result = icp(src, kf, rel_prev + rel_vel)
        if result is None or result['inlier'] < .35:
            if np.isfinite(poses[i - 1, 0]):
                prev_pts = points_from(ranges_mm[i - 1], cos_a, sin_a)
                if len(prev_pts) >= MIN_POINTS:
                    kf = ref_of(prev_pts)
                    kf_pose = poses[i - 1].copy()
                    result = icp(src, kf, rel_vel)
                    rel_prev = np.zeros(3)
            if result is None:
                rel_vel = np.zeros(3)
                continue
        rel = result['pose']
        poses[i] = compose(kf_pose, rel)
        quality[i] = (result['inlier'], result['rms'], result['pairs'], result['cond'])
        rel_vel = rel - rel_prev
        rel_prev = rel
        if math.hypot(rel[0], rel[1]) > KF_DIST or abs(rel[2]) > KF_ANGLE:
            kf = ref_of(src)
            kf_pose = poses[i].copy()
            rel_prev = np.zeros(3)
    return poses, quality


def _chain_job(args):
    ranges_mm, angle_min, increment = args
    return match_chain(ranges_mm, angle_min, increment)


def plan_chunks(windows, chunk_scans):
    """[(window_index, first_scan, last_scan_exclusive)]: chunks overlap by one scan."""
    chunks = []
    for w, (a, b) in enumerate(windows):
        start = a
        while start < b - 1:
            stop = min(b, start + chunk_scans)
            chunks.append((w, start, stop))
            if stop >= b:
                break
            start = stop - 1
        if b - a == 1:
            chunks.append((w, a, b))
    return chunks


def default_workers():
    return max(1, min(4, os.cpu_count() or 1))


def odometry(ranges_mm, angle_min, increment, windows, workers=None, chunk_scans=600, pool=None):
    """Match every scan inside `windows` ([(first, last_exclusive)] scan indices).

    Returns pose (n, 3) in a per-chain frame (NaN outside windows / failed), quality (n, 4) and chain
    (n,) int: scans of one chain share a frame; -1 = not matched. A chunk whose overlap scan failed
    starts a new chain (frames are never guessed across a gap).
    """
    n = len(ranges_mm)
    workers = default_workers() if workers is None else max(1, min(MAX_WORKERS, int(workers)))
    chunks = plan_chunks(windows, max(20, int(chunk_scans)))
    jobs = [(ranges_mm[a:b], float(angle_min), float(increment)) for _, a, b in chunks]
    if pool is not None and len(jobs) > 1:
        results = list(pool.map(_chain_job, jobs))
    elif workers == 1 or len(jobs) <= 1:
        results = [_chain_job(job) for job in jobs]
    else:
        with ProcessPoolExecutor(max_workers=min(workers, len(jobs))) as pool:
            results = list(pool.map(_chain_job, jobs))
    pose = np.full((n, 3), np.nan)
    quality = np.full((n, 4), np.nan)
    chain = np.full(n, -1, dtype=np.int64)
    chain_id = -1
    previous_window = None
    for (w, a, b), (p, q) in zip(chunks, results):
        if w != previous_window or not np.isfinite(pose[a, 0]):
            chain_id += 1
            base = None
        else:
            base = pose[a].copy()
        previous_window = w
        for k in range(b - a):
            i = a + k
            if base is not None and k == 0:
                continue                      # overlap scan: keep the previous chunk's pose and quality
            if not np.isfinite(p[k, 0]):
                pose[i] = np.nan
                quality[i] = np.nan
                chain[i] = -1
                continue
            pose[i] = p[k] if base is None else compose(base, p[k])
            quality[i] = q[k]
            chain[i] = chain_id
    return pose, quality, chain


def good_mask(quality, pose):
    return (np.isfinite(pose[:, 0]) & (quality[:, 0] >= GOOD['inlier']) & (quality[:, 1] <= GOOD['rms'])
            & (quality[:, 3] <= GOOD['cond']))


def velocities(t, pose, quality, chain, half=HALF):
    """Per-scan LiDAR-frame velocity from the +-half scan centred difference (lidar_response.py rule).

    Returns dict(vx, vy, w [rad/s], valid, theta (unwrapped per chain)). A sample is valid only when
    the scans i-half, i and i+half are all good, in the same chain and 0.25 <= dt <= 0.6 s.
    """
    n = len(t)
    good = good_mask(quality, pose)
    theta = np.full(n, np.nan)
    for c in np.unique(chain[chain >= 0]):
        idx = np.nonzero(chain == c)[0]
        theta[idx] = np.unwrap(pose[idx, 2])
    vx = np.full(n, np.nan)
    vy = np.full(n, np.nan)
    w = np.full(n, np.nan)
    if n > 2 * half:
        i = np.arange(half, n - half)
        a, b = i - half, i + half
        dt = t[b] - t[a]
        ok = (good[a] & good[b] & good[i] & (chain[a] == chain[i]) & (chain[b] == chain[i]) & (chain[i] >= 0)
              & (dt >= .25) & (dt <= .6))
        ii, aa, bb, dd = i[ok], a[ok], b[ok], dt[ok]
        c, s = np.cos(theta[ii]), np.sin(theta[ii])
        dx = pose[bb, 0] - pose[aa, 0]
        dy = pose[bb, 1] - pose[aa, 1]
        vx[ii] = (c * dx + s * dy) / dd
        vy[ii] = (-s * dx + c * dy) / dd
        w[ii] = (theta[bb] - theta[aa]) / dd
    valid = np.isfinite(vx)
    return dict(vx=vx, vy=vy, w=w, valid=valid, theta=theta, good=good)
