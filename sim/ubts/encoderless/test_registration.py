"""Independent geometric checks for the offline, fixed-IMU-yaw candidate.

The oracle raycasts world line segments to generate two distinct sensor scans.
Only scan coordinates and IMU delta yaw enter the estimator. World poses below
are synthetic test oracles, never a wheel, truth, or control input to it.
"""

from __future__ import annotations

import math
import unittest

import numpy as np

try:
    from .registration import RegistrationConfig, register_translation
except ImportError:
    from registration import RegistrationConfig, register_translation


def rotation(yaw: float) -> np.ndarray:
    return np.array(((math.cos(yaw), -math.sin(yaw)),
                     (math.sin(yaw), math.cos(yaw))))


def raycast_segments(segments, pose=(0.0, 0.0, 0.0), *, rays=720,
                     max_range=10.0) -> np.ndarray:
    """Nearest positive ray/segment intersection, with no-return rays omitted."""
    angles = np.linspace(-math.pi, math.pi, rays, endpoint=False)
    local_directions = np.column_stack((np.cos(angles), np.sin(angles)))
    directions = local_directions @ rotation(pose[2]).T
    origin = np.asarray(pose[:2], dtype=float)
    ranges = np.full(rays, np.inf)
    for endpoint_a, endpoint_b in segments:
        edge = np.asarray(endpoint_b, dtype=float) - endpoint_a
        offset = np.asarray(endpoint_a, dtype=float) - origin
        denominator = directions[:, 0] * edge[1] - directions[:, 1] * edge[0]
        not_parallel = np.abs(denominator) > 1e-10
        distance = np.full(rays, np.inf)
        parameter = np.full(rays, np.inf)
        distance[not_parallel] = (
            offset[0] * edge[1] - offset[1] * edge[0]
        ) / denominator[not_parallel]
        parameter[not_parallel] = (
            offset[0] * directions[not_parallel, 1]
            - offset[1] * directions[not_parallel, 0]
        ) / denominator[not_parallel]
        visible = ((distance >= 0.05) & (distance <= max_range)
                   & (parameter >= 0.0) & (parameter <= 1.0))
        ranges[visible] = np.minimum(ranges[visible], distance[visible])
    returned = np.isfinite(ranges)
    return local_directions[returned] * ranges[returned, None]


# Unequal room sides plus short, oblique obstacles make both translation axes
# observable without relying on finite-wall endpoints as point landmarks.
ASYMMETRIC_SCENE = (
    ((-4.0, -3.0), (6.0, -3.0)),
    ((6.0, -3.0), (6.0, 4.5)),
    ((6.0, 4.5), (-4.0, 4.5)),
    ((-4.0, 4.5), (-4.0, -3.0)),
    ((2.0, 1.2), (3.1, 2.2)),
    ((-2.5, 1.0), (-1.9, 2.5)),
    ((0.8, -2.1), (1.9, -1.7)),
)


class RaycastOracleTests(unittest.TestCase):
    """Verify the independent fixture before using it to judge registration."""

    def test_ray_geometry_and_no_return(self):
        scan = raycast_segments((((2.0, -1.0), (2.0, 1.0)),), rays=8)
        np.testing.assert_allclose(scan, [[2.0, 0.0]], atol=1e-12)
        self.assertEqual(raycast_segments((), rays=8).shape, (0, 2))

    def test_sensor_coordinates_rotate_and_translate(self):
        scan = raycast_segments((((2.0, -2.0), (2.0, 2.0)),),
                                pose=(1.0, 0.0, math.pi / 2), rays=4)
        np.testing.assert_allclose(scan, [[0.0, -1.0]], atol=1e-12)


class RegistrationGeometryTests(unittest.TestCase):
    def assert_valid_motion(self, delta, yaw=0.0, prior_pose=(0.0, 0.0, 0.0),
                            *, noise=0.0, clutter=0):
        position = np.asarray(prior_pose[:2]) + rotation(prior_pose[2]) @ delta
        current_pose = (*position, prior_pose[2] + yaw)
        previous = raycast_segments(ASYMMETRIC_SCENE, prior_pose)
        current = raycast_segments(ASYMMETRIC_SCENE, current_pose)
        if noise:
            rng = np.random.default_rng(8921)
            previous += rng.normal(0.0, noise, previous.shape)
            current += rng.normal(0.0, noise, current.shape)
        if clutter:
            rng = np.random.default_rng(301)
            # Independent false returns across scans; no shared distractors.
            previous = np.vstack((previous, rng.uniform(-4, 4, (clutter, 2))))
            current = np.vstack((current, rng.uniform(-4, 4, (clutter, 2))))
        result = register_translation(previous, current, yaw)
        self.assertTrue(result.valid, str(result))
        self.assertIsNotNone(result.delta_xy)
        self.assertFalse(result.ambiguity, str(result))
        np.testing.assert_allclose(result.delta_xy, delta, atol=0.03,
                                   err_msg=str(result))
        self.assertEqual(result.delta_yaw_rad, yaw)
        return result

    def assert_rejected(self, previous, current, yaw=0.0, **kwargs):
        result = register_translation(previous, current, yaw, **kwargs)
        self.assertFalse(result.valid, str(result))
        self.assertIsNone(result.delta_xy, str(result))
        self.assertTrue(result.reason, str(result))
        return result

    def test_stationary(self):
        self.assert_valid_motion((0.0, 0.0))

    def test_forward_and_reverse(self):
        for displacement in ((0.18, 0.0), (-0.18, 0.0)):
            with self.subTest(displacement=displacement):
                self.assert_valid_motion(displacement)

    def test_lateral_both_directions(self):
        for displacement in ((0.0, 0.16), (0.0, -0.16)):
            with self.subTest(displacement=displacement):
                self.assert_valid_motion(displacement)

    def test_pure_rotation_both_directions(self):
        for yaw in (-0.08, 0.08):
            with self.subTest(yaw=yaw):
                self.assert_valid_motion((0.0, 0.0), yaw)

    def test_combined_rotation_translation(self):
        self.assert_valid_motion((0.11, -0.06), 0.065)

    def test_larger_motion_inside_default_search_bound(self):
        self.assert_valid_motion((0.38, -0.15), -0.05)

    def test_delta_uses_previous_lidar_frame_not_world_axes(self):
        self.assert_valid_motion((0.16, -0.09), -0.06,
                                 prior_pose=(0.3, -0.1, 0.7))

    def test_noisy_sensor_returns(self):
        self.assert_valid_motion((-0.12, 0.08), 0.04, noise=0.005)

    def test_independent_sparse_outliers(self):
        self.assert_valid_motion((0.12, 0.07), -0.04, clutter=36)

    def test_single_wall_is_unobservable(self):
        scene = (((-100.0, 3.0), (100.0, 3.0)),)
        self.assert_rejected(raycast_segments(scene),
                             raycast_segments(scene, (0.14, 0.08, 0.0)))

    def test_parallel_row_walls_are_unobservable(self):
        scene = (((-100.0, 2.0), (100.0, 2.0)),
                 ((-100.0, -2.0), (100.0, -2.0)))
        self.assert_rejected(raycast_segments(scene),
                             raycast_segments(scene, (-0.18, -0.06, 0.0)))

    def test_repeated_rows_reject_ambiguous_translation(self):
        # Identical rectangular objects every 30 cm make stationary and
        # one-period motion indistinguishable. Both surface-normal directions
        # exist; good local observability alone must not certify this scene.
        scene = []
        for x in np.arange(-10.0, 10.01, 0.3):
            for y in (-2.0, 2.0):
                a, b = (x - 0.07, y - 0.15), (x + 0.07, y - 0.15)
                c, d = (x + 0.07, y + 0.15), (x - 0.07, y + 0.15)
                scene.extend(((a, b), (b, c), (c, d), (d, a)))
        previous = raycast_segments(scene)
        for displacement in (0.0, 0.3):
            with self.subTest(displacement=displacement):
                current = raycast_segments(scene, (displacement, 0.0, 0.0))
                result = self.assert_rejected(previous, current)
                self.assertTrue(result.ambiguity, str(result))
                self.assertIsNotNone(result.competing_delta)

    def test_empty_and_sparse_returns(self):
        full_scan = raycast_segments(ASYMMETRIC_SCENE)
        for scan in (np.empty((0, 2)), full_scan[:3]):
            for previous, current in ((scan, full_scan), (full_scan, scan)):
                with self.subTest(previous=len(previous), current=len(current)):
                    self.assert_rejected(previous, current)

    def test_invalid_coordinates_fail_closed(self):
        full_scan = raycast_segments(ASYMMETRIC_SCENE)
        for invalid in (math.nan, math.inf, -math.inf):
            corrupt = full_scan.copy()
            corrupt[0, 1] = invalid
            for previous, current in ((corrupt, full_scan), (full_scan, corrupt)):
                with self.subTest(invalid=invalid):
                    self.assert_rejected(previous, current)

    def test_missing_or_invalid_imu_delta_is_not_estimated(self):
        scan = raycast_segments(ASYMMETRIC_SCENE)
        for yaw in (None, math.nan, math.inf, -math.inf):
            with self.subTest(yaw=yaw):
                self.assert_rejected(scan, scan, yaw)

    def test_nonoverlapping_scans_reject(self):
        scan = raycast_segments(ASYMMETRIC_SCENE)
        self.assert_rejected(scan, scan + (40.0, 40.0))

    def test_motion_beyond_explicit_bound_rejects(self):
        previous = raycast_segments(ASYMMETRIC_SCENE)
        current = raycast_segments(ASYMMETRIC_SCENE, (0.42, 0.0, 0.0))
        self.assert_rejected(previous, current,
                             config=RegistrationConfig(max_translation_m=0.2))


if __name__ == "__main__":
    unittest.main()
