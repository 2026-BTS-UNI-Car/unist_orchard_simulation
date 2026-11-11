from __future__ import annotations

import math
from typing import Any


ODOMETRY_ALGORITHM = "scan_context_distance_field_coarse_to_fine_v1"


def estimate_relative_transform(
    previous_points: list[dict[str, Any]],
    current_points: list[dict[str, Any]],
    prior: dict[str, Any] | None = None,
    dt_s: float = 0.6,
) -> dict[str, Any]:
    """Estimate the current sensor pose in the previous LiDAR frame.

    A circular scan-context supplies rotation hypotheses over the full 360
    degrees. Each hypothesis is scored against a Euclidean distance field and
    refined in translation and rotation. The return value includes enough
    quality evidence for the caller to reject ambiguous motion.
    """
    import cv2  # type: ignore
    import numpy as np  # type: ignore

    previous = scan_array(previous_points, np)
    current = scan_array(current_points, np)
    if len(previous) < 120 or len(current) < 120:
        return zero_transform("insufficient_points", point_count=min(len(previous), len(current)))

    descriptor_candidates, descriptor = rotation_hypotheses(previous, current, np, cv2, prior=prior)
    distance_field = make_distance_field(previous, np, cv2)
    max_translation = max(0.7, min(1.45, 0.35 + max(0.1, float(dt_s)) * 1.55))
    prior_dx = float((prior or {}).get("dx_m") or 0.0)
    prior_dy = float((prior or {}).get("dy_m") or 0.0)
    coarse_results = []
    for theta in descriptor_candidates:
        rotated = rotate_points(current, theta, np)
        result = best_translation(
            distance_field,
            rotated,
            np,
            center_dx=prior_dx * 0.35,
            center_dy=prior_dy * 0.35,
            span_m=max_translation,
            step_m=0.2,
        )
        result["dtheta_rad"] = theta
        result["descriptor_score"] = descriptor_score_for_angle(descriptor, theta)
        result["objective"] = transform_objective(result, theta, prior)
        coarse_results.append(result)

    coarse_results.sort(key=lambda item: float(item["objective"]), reverse=True)
    refined_results = []
    for coarse in coarse_results[:4]:
        center_theta = float(coarse["dtheta_rad"])
        for theta_offset_deg in (-2.0, -1.0, 0.0, 1.0, 2.0):
            theta = wrap_angle(center_theta + math.radians(theta_offset_deg))
            rotated = rotate_points(current, theta, np)
            result = best_translation(
                distance_field,
                rotated,
                np,
                center_dx=float(coarse["dx_m"]),
                center_dy=float(coarse["dy_m"]),
                span_m=0.24,
                step_m=0.06,
            )
            result["dtheta_rad"] = theta
            result["descriptor_score"] = descriptor_score_for_angle(descriptor, theta)
            result["objective"] = transform_objective(result, theta, prior)
            refined_results.append(result)

    refined_results.sort(key=lambda item: float(item["objective"]), reverse=True)
    if not refined_results:
        return zero_transform("no_transform_candidates")
    best = refined_results[0]
    second = next(
        (
            row
            for row in refined_results[1:]
            if transform_separation(best, row) >= 1.0
        ),
        refined_results[1] if len(refined_results) > 1 else None,
    )
    objective_margin = float(best["objective"]) - float((second or {}).get("objective") or 0.0)
    descriptor_margin = float(descriptor.get("margin") or 0.0)
    dtheta = float(best["dtheta_rad"])
    score = float(best["score"])
    inlier_ratio = float(best["inlier_ratio"])
    rmse_m = float(best["rmse_m"])
    large_turn = abs(math.degrees(dtheta)) >= 12.0
    accepted = score >= 0.43 and inlier_ratio >= 0.3 and rmse_m <= 0.38
    if large_turn and descriptor_margin < 0.006 and objective_margin < 0.008:
        accepted = False
    pose_confidence = max(
        0.0,
        min(
            0.99,
            0.46 * score
            + 0.28 * inlier_ratio
            + 0.16 * min(1.0, max(0.0, objective_margin) / 0.035)
            + 0.1 * min(1.0, max(0.0, descriptor_margin) / 0.04),
        ),
    )
    payload = {
        "dx_m": round(float(best["dx_m"]), 3) if accepted else 0.0,
        "dy_m": round(float(best["dy_m"]), 3) if accepted else 0.0,
        "dtheta_rad": round(dtheta, 6) if accepted else 0.0,
        "score": round(score, 4),
        "inlier_ratio": round(inlier_ratio, 4),
        "rmse_m": round(rmse_m, 4),
        "objective_margin": round(objective_margin, 5),
        "descriptor_score": round(float(best.get("descriptor_score") or 0.0), 4),
        "descriptor_margin": round(descriptor_margin, 5),
        "pose_confidence": round(pose_confidence if accepted else pose_confidence * 0.45, 3),
        "accepted": accepted,
        "turn_event": bool(accepted and large_turn),
        "method": ODOMETRY_ALGORITHM if accepted else "scan_match_quality_rejected",
        "rotation_hypotheses_deg": [round(math.degrees(value), 2) for value in descriptor_candidates[:8]],
        "point_count": min(len(previous), len(current)),
    }
    if not accepted:
        payload["rejection_reasons"] = quality_rejection_reasons(score, inlier_ratio, rmse_m, large_turn, descriptor_margin, objective_margin)
    return payload


def scan_array(points: list[dict[str, Any]], np: Any) -> Any:
    rows = []
    for point in points:
        try:
            x = float(point["x_m"])
            y = float(point["y_m"])
        except (KeyError, TypeError, ValueError):
            continue
        distance = math.hypot(x, y)
        if not math.isfinite(distance) or distance < 0.72 or distance > 14.5:
            continue
        # The operator stood immediately behind the rover in both research
        # archives. Remove only that narrow near-field sector; preserve the
        # rest of the rear scan because it is essential during a U-turn.
        if y < -0.35 and abs(x) < 1.15 and distance < 4.2:
            continue
        rows.append((x, y))
    if not rows:
        return np.empty((0, 2), dtype=np.float32)
    array = np.asarray(rows, dtype=np.float32)
    if len(array) > 850:
        indexes = np.linspace(0, len(array) - 1, 850, dtype=np.int32)
        array = array[indexes]
    return array


def rotation_hypotheses(previous: Any, current: Any, np: Any, cv2: Any, prior: dict[str, Any] | None = None) -> tuple[list[float], dict[str, Any]]:
    sectors = 180
    previous_descriptor = scan_context(previous, np, cv2, sectors=sectors)
    current_descriptor = scan_context(current, np, cv2, sectors=sectors)
    denominator = max(1e-6, float(np.linalg.norm(previous_descriptor) * np.linalg.norm(current_descriptor)))
    scores = []
    for shift in range(sectors):
        score = float(np.sum(previous_descriptor * np.roll(current_descriptor, shift, axis=1)) / denominator)
        scores.append(score)
    order = np.argsort(np.asarray(scores))[::-1].tolist()
    selected_shifts = []
    separation = max(3, int(round(sectors * 8.0 / 360.0)))
    for shift in order:
        if all(circular_index_distance(shift, other, sectors) >= separation for other in selected_shifts):
            selected_shifts.append(int(shift))
        if len(selected_shifts) >= 7:
            break
    candidates = [wrap_angle(math.radians(shift * 360.0 / sectors)) for shift in selected_shifts]
    prior_theta = float((prior or {}).get("dtheta_rad") or 0.0)
    candidates.extend([0.0, prior_theta])
    for offset_deg in (-4.0, -2.0, 2.0, 4.0):
        candidates.append(wrap_angle(prior_theta + math.radians(offset_deg)))
    candidates = unique_angles(candidates, tolerance_deg=0.75)
    best_score = scores[selected_shifts[0]] if selected_shifts else 0.0
    second_score = scores[selected_shifts[1]] if len(selected_shifts) > 1 else 0.0
    return candidates, {
        "scores": scores,
        "sectors": sectors,
        "best": best_score,
        "second": second_score,
        "margin": max(0.0, best_score - second_score),
    }


def scan_context(points: Any, np: Any, cv2: Any, sectors: int = 180, rings: int = 29) -> Any:
    descriptor = np.zeros((rings, sectors), dtype=np.float32)
    radii = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2)
    angles = (np.arctan2(points[:, 1], points[:, 0]) + 2.0 * np.pi) % (2.0 * np.pi)
    ring_indexes = np.clip((radii / 0.5).astype(np.int32), 0, rings - 1)
    sector_indexes = np.floor(angles / (2.0 * np.pi) * sectors).astype(np.int32) % sectors
    descriptor[ring_indexes, sector_indexes] = 1.0
    descriptor = cv2.dilate(descriptor, np.ones((3, 3), dtype=np.uint8), iterations=1)
    return descriptor.astype(np.float32)


def make_distance_field(previous: Any, np: Any, cv2: Any, limit_m: float = 16.0, resolution_m: float = 0.07) -> dict[str, Any]:
    size = int(math.ceil(2.0 * limit_m / resolution_m)) + 1
    image = np.full((size, size), 255, dtype=np.uint8)
    ix = np.rint((previous[:, 0] + limit_m) / resolution_m).astype(np.int32)
    iy = np.rint((limit_m - previous[:, 1]) / resolution_m).astype(np.int32)
    valid = (ix >= 0) & (ix < size) & (iy >= 0) & (iy < size)
    image[iy[valid], ix[valid]] = 0
    field = cv2.distanceTransform(image, cv2.DIST_L2, 3) * resolution_m
    return {"field": field, "limit_m": limit_m, "resolution_m": resolution_m, "size": size}


def rotate_points(points: Any, theta: float, np: Any) -> Any:
    cosine = math.cos(theta)
    sine = math.sin(theta)
    matrix = np.asarray([[cosine, -sine], [sine, cosine]], dtype=np.float32)
    return points @ matrix.T


def best_translation(
    distance_field: dict[str, Any],
    rotated: Any,
    np: Any,
    center_dx: float,
    center_dy: float,
    span_m: float,
    step_m: float,
) -> dict[str, float]:
    steps = max(1, int(round(span_m / step_m)))
    best = {"dx_m": center_dx, "dy_m": center_dy, "score": -1.0, "inlier_ratio": 0.0, "rmse_m": 99.0}
    for x_index in range(-steps, steps + 1):
        dx = center_dx + x_index * step_m
        if abs(dx) > 1.55:
            continue
        for y_index in range(-steps, steps + 1):
            dy = center_dy + y_index * step_m
            if abs(dy) > 1.55:
                continue
            score, inlier_ratio, rmse_m = distance_field_score(distance_field, rotated, dx, dy, np)
            if score > float(best["score"]):
                best = {"dx_m": dx, "dy_m": dy, "score": score, "inlier_ratio": inlier_ratio, "rmse_m": rmse_m}
    return best


def distance_field_score(distance_field: dict[str, Any], rotated: Any, dx: float, dy: float, np: Any) -> tuple[float, float, float]:
    field = distance_field["field"]
    limit_m = float(distance_field["limit_m"])
    resolution_m = float(distance_field["resolution_m"])
    size = int(distance_field["size"])
    x = rotated[:, 0] + dx
    y = rotated[:, 1] + dy
    ix = np.rint((x + limit_m) / resolution_m).astype(np.int32)
    iy = np.rint((limit_m - y) / resolution_m).astype(np.int32)
    valid = (ix >= 0) & (ix < size) & (iy >= 0) & (iy < size)
    if int(np.count_nonzero(valid)) < max(60, int(len(rotated) * 0.45)):
        return 0.0, 0.0, 99.0
    distances = field[iy[valid], ix[valid]]
    inliers = distances <= 0.26
    inlier_ratio = float(np.count_nonzero(inliers) / max(1, len(distances)))
    clipped = np.minimum(distances, 0.8)
    robust_mean = float(np.mean(clipped))
    rmse = float(np.sqrt(np.mean(distances[inliers] ** 2))) if np.any(inliers) else 99.0
    valid_ratio = float(np.count_nonzero(valid) / max(1, len(rotated)))
    score = 0.62 * inlier_ratio + 0.28 * math.exp(-robust_mean / 0.18) + 0.1 * valid_ratio
    return float(score), inlier_ratio, rmse


def transform_objective(result: dict[str, Any], theta: float, prior: dict[str, Any] | None) -> float:
    objective = float(result.get("score") or 0.0) + 0.035 * float(result.get("descriptor_score") or 0.0)
    if prior:
        prior_theta = float(prior.get("dtheta_rad") or 0.0)
        angle_delta = abs(math.degrees(wrap_angle(theta - prior_theta)))
        translation_delta = math.hypot(float(result["dx_m"]) - float(prior.get("dx_m") or 0.0), float(result["dy_m"]) - float(prior.get("dy_m") or 0.0))
        objective += 0.018 * math.exp(-angle_delta / 18.0) + 0.012 * math.exp(-translation_delta / 0.5)
    return objective


def descriptor_score_for_angle(descriptor: dict[str, Any], theta: float) -> float:
    scores = descriptor.get("scores") or []
    sectors = int(descriptor.get("sectors") or 1)
    if not scores:
        return 0.0
    shift = int(round((math.degrees(theta) % 360.0) / 360.0 * sectors)) % sectors
    return float(scores[shift])


def quality_rejection_reasons(score: float, inlier_ratio: float, rmse_m: float, large_turn: bool, descriptor_margin: float, objective_margin: float) -> list[str]:
    reasons = []
    if score < 0.43:
        reasons.append("low_distance_field_score")
    if inlier_ratio < 0.3:
        reasons.append("low_inlier_ratio")
    if rmse_m > 0.38:
        reasons.append("high_inlier_rmse")
    if large_turn and descriptor_margin < 0.006 and objective_margin < 0.008:
        reasons.append("ambiguous_large_rotation")
    return reasons or ["ambiguous_transform"]


def transform_separation(a: dict[str, Any], b: dict[str, Any]) -> float:
    angle = abs(math.degrees(wrap_angle(float(a["dtheta_rad"]) - float(b["dtheta_rad"])))) / 3.0
    translation = math.hypot(float(a["dx_m"]) - float(b["dx_m"]), float(a["dy_m"]) - float(b["dy_m"])) / 0.12
    return angle + translation


def circular_index_distance(a: int, b: int, size: int) -> int:
    delta = abs(a - b) % size
    return min(delta, size - delta)


def unique_angles(values: list[float], tolerance_deg: float) -> list[float]:
    rows = []
    for value in values:
        value = wrap_angle(value)
        if any(abs(math.degrees(wrap_angle(value - other))) <= tolerance_deg for other in rows):
            continue
        rows.append(value)
    return rows


def wrap_angle(value: float) -> float:
    return ((float(value) + math.pi) % math.tau) - math.pi


def zero_transform(method: str, point_count: int = 0) -> dict[str, Any]:
    return {
        "dx_m": 0.0,
        "dy_m": 0.0,
        "dtheta_rad": 0.0,
        "score": 0.0,
        "inlier_ratio": 0.0,
        "rmse_m": None,
        "objective_margin": 0.0,
        "descriptor_margin": 0.0,
        "pose_confidence": 0.0,
        "accepted": False,
        "turn_event": False,
        "method": method,
        "point_count": point_count,
    }
