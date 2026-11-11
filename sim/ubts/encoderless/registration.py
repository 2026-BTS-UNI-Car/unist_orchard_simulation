"""Experimental scan translation with externally measured, fixed IMU yaw.

The convention is ``previous = R(delta_yaw_rad) @ current + delta_xy``.
Only the translation is fitted. No wheel, command, ground-truth or ROS inputs
exist here. This is an offline candidate, not a calibrated odometry source.
Tree queries use exact Euclidean distance (eps=0, p=2) with one worker; potential
ties retain the dense NumPy selection convention through a local fallback.
"""

from dataclasses import dataclass
from math import isfinite
from time import perf_counter

import numpy as np
from scipy.spatial import cKDTree


MAX_INPUT_POINTS = 3200
NORMAL_QUERY_CHUNK = 64


@dataclass(frozen=True)
class RegistrationConfig:
    """Experimental thresholds in metres; none are hardware-calibrated."""

    max_points: int = 384
    min_points: int = 24
    max_iterations: int = 15
    max_translation_m: float = 0.6
    correspondence_distance_m: float = 0.25
    normal_neighbors: int = 7
    normal_max_radius_m: float = 0.4
    normal_max_eigen_ratio: float = 0.15
    huber_delta_m: float = 0.03
    convergence_translation_m: float = 1e-4
    min_inliers: int = 24
    min_overlap: float = 0.45
    max_rmse_m: float = 0.05
    max_condition: float = 20.0
    min_normal_eigenvalue: float = 0.05
    ambiguity_distance_m: float = 0.08
    ambiguity_overlap_margin: float = 0.05
    seed_step_m: float = 0.3

    def __post_init__(self):
        for name in ("max_points", "min_points", "max_iterations",
                     "normal_neighbors", "min_inliers"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if not 3 <= self.normal_neighbors <= self.min_points <= self.max_points <= 384:
            raise ValueError("require 3 <= neighbors <= min_points <= max_points <= 384")
        if self.max_iterations > 15 or self.min_inliers > self.max_points:
            raise ValueError("iterations must be <= 15 and min_inliers <= max_points")
        for name, value in vars(self).items():
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.max_translation_m > 0.6:
            raise ValueError("offline candidate translation bound is at most 0.6 m")
        if self.seed_step_m < self.max_translation_m / 4:
            raise ValueError("seed_step_m must be at least translation_bound / 4")
        if not 0 < self.min_overlap <= 1 or self.ambiguity_overlap_margin >= 1:
            raise ValueError("overlap thresholds must be fractions")
        if self.normal_max_eigen_ratio >= 1 or self.min_normal_eigenvalue >= 0.5:
            raise ValueError("invalid normal geometry thresholds")


@dataclass(frozen=True)
class CandidateResult:
    """One seed's diagnostic. Even valid candidates are not final acceptance."""

    seed_xy: tuple[float, float]
    valid: bool
    reason: str
    delta_xy: tuple[float, float] | None
    inliers: int
    overlap: float
    rmse_m: float
    condition: float
    eigenvalues: tuple[float, float]
    iterations: int


@dataclass(frozen=True)
class RegistrationResult:
    valid: bool
    reason: str
    delta_xy: tuple[float, float] | None
    delta_yaw_rad: float | None
    inliers: int
    overlap: float
    rmse_m: float
    condition: float
    eigenvalues: tuple[float, float]
    ambiguity: bool
    competing_delta: tuple[float, float] | None
    iterations: int
    elapsed_s: float
    candidates: tuple[CandidateResult, ...]


def _points(value, config):
    try:
        points = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None, "invalid_points"
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        return None, "invalid_points"
    if len(points) > MAX_INPUT_POINTS:
        return None, "too_many_points"
    if len(points) < config.min_points:
        return None, "insufficient_points"
    # Do not silently turn missing returns or duplicate-only scans into motion.
    if len(np.unique(points, axis=0)) < config.min_points:
        return None, "insufficient_unique_points"
    # Also keeps squared-distance arithmetic in a sensible numerical range.
    if np.max(np.abs(points)) > 1e3:
        return None, "invalid_point_range"
    return points.copy(), None


def _sample_points(points, config):
    indices = np.linspace(0, len(points) - 1, min(len(points), config.max_points), dtype=int)
    return points[indices].copy()


def _distances(a, b):
    delta = a[:, None, :] - b[None, :, :]
    return np.einsum("ijk,ijk->ij", delta, delta)


def _reference_normals(points, config, *, support_points=None):
    support = points if support_points is None else support_points
    count = config.normal_neighbors
    support_tree = cKDTree(support)
    kept_points, kept_normals = [], []
    for start in range(0, len(points), NORMAL_QUERY_CHUNK):
        anchors = points[start:start + NORMAL_QUERY_CHUNK]
        distances, indices = support_tree.query(
            anchors, k=count + 1, eps=0.0, p=2, workers=1)
        neighbors = indices[:, :count].copy()
        # np.argpartition does not define tie order. For a boundary tie, use
        # exactly the baseline's partition, preserving its selected subset.
        tied = np.isclose(distances[:, count - 1], distances[:, count],
                          rtol=1e-12, atol=1e-14)
        for i in np.flatnonzero(tied):
            distance2 = _distances(anchors[i:i + 1], support)
            neighbors[i] = np.argpartition(distance2, count - 1, axis=1)[0, :count]
        neighborhoods = support[neighbors]
        centered = neighborhoods - neighborhoods.mean(axis=1, keepdims=True)
        covariance = np.einsum("nki,nkj->nij", centered, centered) / count
        values, vectors = np.linalg.eigh(covariance)
        delta = anchors[:, None, :] - neighborhoods
        radii2 = np.einsum("nki,nki->nk", delta, delta).max(axis=1)
        good = ((values[:, 1] > 1e-8)
                & (values[:, 0] <= config.normal_max_eigen_ratio * values[:, 1])
                & (radii2 <= config.normal_max_radius_m ** 2))
        kept_points.append(anchors[good])
        kept_normals.append(vectors[good, :, 0])
    return np.concatenate(kept_points), np.concatenate(kept_normals)


def _correspondences(reference, normals, moved, config, *, final=False, tree=None):
    if tree is None:
        tree = cKDTree(reference)
    distances, candidates = tree.query(moved, k=2, eps=0.0, p=2, workers=1)
    indices = candidates[:, 0].copy()
    # np.argmin chooses the lowest reference index for equal squared distances.
    # Recompute baseline distances only for potential ties, including rounding.
    tied = np.isclose(distances[:, 0], distances[:, 1], rtol=1e-12, atol=1e-14)
    for i in np.flatnonzero(tied):
        indices[i] = np.argmin(_distances(moved[i:i + 1], reference)[0])
    delta = moved - reference[indices]
    nearest2 = np.einsum("ij,ij->i", delta, delta)
    selected_normals = normals[indices]
    residual = np.einsum("ij,ij->i", selected_normals, delta)
    keep = nearest2 <= config.correspondence_distance_m ** 2
    if final:
        keep &= np.abs(residual) <= 3 * config.huber_delta_m
    return selected_normals[keep], residual[keep]


def _information(normals, residual, config):
    weights = np.minimum(1.0, config.huber_delta_m / np.maximum(np.abs(residual), 1e-12))
    hessian = normals.T @ (weights[:, None] * normals)
    eigenvalues = np.linalg.eigvalsh(hessian / weights.sum())
    condition = float(eigenvalues[1] / eigenvalues[0]) if eigenvalues[0] > 1e-12 else float("inf")
    observable = eigenvalues[0] >= config.min_normal_eigenvalue and condition <= config.max_condition
    return weights, hessian, tuple(float(v) for v in eigenvalues), condition, observable


def _solve_seed(reference, normals, rotated, seed, config, tree, source_anchor_count):
    translation = np.array(seed, dtype=float)
    reason, eigenvalues, condition = "iteration_limit", (0.0, 0.0), float("inf")
    inliers, overlap, rmse, iteration = 0, 0.0, float("inf"), 0
    for iteration in range(1, config.max_iterations + 1):
        directions, residual = _correspondences(reference, normals, rotated + translation, config, tree=tree)
        inliers, overlap = len(residual), len(residual) / source_anchor_count
        if inliers < config.min_inliers:
            reason = "insufficient_inliers"
            break
        weights, hessian, eigenvalues, condition, observable = _information(directions, residual, config)
        if not observable:
            reason = "degenerate_geometry"
            break
        step = np.linalg.solve(hessian, -(directions.T @ (weights * residual)))
        proposed = translation + step
        if np.linalg.norm(proposed) > config.max_translation_m + 1e-10:
            reason = "translation_limit"
            break
        translation = proposed
        if np.linalg.norm(step) <= config.convergence_translation_m:
            reason = "converged"
            break
    if reason == "converged":
        directions, residual = _correspondences(reference, normals, rotated + translation, config, final=True, tree=tree)
        inliers, overlap = len(residual), len(residual) / source_anchor_count
        if inliers < config.min_inliers:
            reason = "insufficient_inliers"
        elif overlap < config.min_overlap:
            reason = "insufficient_overlap"
        else:
            _, _, eigenvalues, condition, observable = _information(directions, residual, config)
            rmse = float(np.sqrt(np.mean(residual ** 2)))
            if not observable:
                reason = "degenerate_geometry"
            elif rmse > config.max_rmse_m:
                reason = "residual_too_large"
            else:
                reason = "accepted"
    valid = reason == "accepted"
    return CandidateResult(tuple(float(v) for v in seed), valid, reason,
                           tuple(float(v) for v in translation) if valid else None,
                           inliers, overlap, rmse, condition, eigenvalues, iteration)


def _seeds(config):
    step = min(config.seed_step_m, config.max_translation_m)
    steps = int(np.floor(config.max_translation_m / step + 1e-10))
    seeds = [(x * step, y * step)
             for x in range(-steps, steps + 1) for y in range(-steps, steps + 1)
             if np.hypot(x * step, y * step)
             <= config.max_translation_m + 1e-10]
    return sorted(seeds, key=lambda seed: (seed[0] ** 2 + seed[1] ** 2, seed))


def register_translation(previous_points, current_points, delta_yaw_rad, *,
                         config=RegistrationConfig()) -> RegistrationResult:
    """Return accepted XY in the previous scan frame, or ``delta_xy=None``.

    Yaw is an independently provided IMU delta in radians and is held fixed;
    this function cannot assess IMU age, calibration or whether it is truthful.
    Invalid/missing data, poor geometry, failed convergence and competing local
    fits are rejected. Numerical quality is not a guarantee of correct motion.
    """
    started = perf_counter()
    yaw = None

    def result(reason, best=None, *, candidates=(), ambiguity=False, competing=None):
        accepted = reason == "accepted"
        return RegistrationResult(accepted, reason, best.delta_xy if accepted else None, yaw,
                                  best.inliers if best else 0, best.overlap if best else 0.0,
                                  best.rmse_m if best else float("inf"),
                                  best.condition if best else float("inf"),
                                  best.eigenvalues if best else (0.0, 0.0), ambiguity, competing,
                                  best.iterations if best else 0, perf_counter() - started, candidates)

    try:
        if (delta_yaw_rad is None or isinstance(delta_yaw_rad, (bool, str, np.bool_))
                or np.ndim(delta_yaw_rad) != 0):
            return result("invalid_yaw")
        yaw = float(delta_yaw_rad)
    except (TypeError, ValueError, OverflowError):
        return result("invalid_yaw")
    if not isfinite(yaw):
        yaw = None
        return result("invalid_yaw")
    previous, error = _points(previous_points, config)
    if error:
        return result(error)
    current, error = _points(current_points, config)
    if error:
        return result(error)
    previous_anchors = _sample_points(previous, config)
    current_anchors = _sample_points(current, config)
    reference, normals = _reference_normals(previous_anchors, config, support_points=previous)
    # Apply the same surface-support test on both sides. Otherwise source
    # anchors rejected from an identical reference scan can pull a stationary
    # fit toward a different nearby surface and introduce artificial motion.
    source, _ = _reference_normals(current_anchors, config, support_points=current)
    if min(len(reference), len(source)) < config.min_inliers:
        return result("insufficient_geometry")
    c, s = np.cos(yaw), np.sin(yaw)
    rotated = source @ np.array([[c, s], [-s, c]])
    tree = cKDTree(reference)
    # Keep unsupported source anchors in the overlap denominator: removing
    # them from fitting must not inflate coverage or weaken its existing gate.
    candidates = tuple(_solve_seed(reference, normals, rotated, seed, config, tree,
                                   len(current_anchors)) for seed in _seeds(config))
    accepted = [candidate for candidate in candidates if candidate.valid]
    if not accepted:
        best = max(candidates, key=lambda candidate: (candidate.inliers, -candidate.rmse_m))
        return result(best.reason, best, candidates=candidates)
    # First prefer coverage; do not choose a tiny, very clean accidental match.
    best = max(accepted, key=lambda candidate: (candidate.overlap, -candidate.rmse_m))
    for competitor in accepted:
        # Each competitor already passed the absolute residual/geometry gates.
        # Similar coverage at a separated translation is ambiguous even if
        # anchor decimation makes one residual appear much cleaner (including
        # a zero-residual identical scan in a periodic orchard).
        if (np.linalg.norm(np.subtract(best.delta_xy, competitor.delta_xy)) >= config.ambiguity_distance_m
                and competitor.overlap >= best.overlap - config.ambiguity_overlap_margin):
            return result("ambiguous_translation", best, candidates=candidates,
                          ambiguity=True, competing=competitor.delta_xy)
    return result("accepted", best, candidates=candidates)
