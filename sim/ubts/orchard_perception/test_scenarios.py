"""Independent known geometry checks for offline scenario sensors only."""

from dataclasses import asdict, replace
import math
import unittest

import numpy as np

try:
    from .scenarios import (SCENARIOS, Circle, Segment, OrchardScene, Pose2D,
                            SceneConfig, ScanConfig, field_pose, generate_scene, raycast_scan)
except ImportError:
    from scenarios import (SCENARIOS, Circle, Segment, OrchardScene, Pose2D,
                           SceneConfig, ScanConfig, field_pose, generate_scene, raycast_scan)


class ExactRayTests(unittest.TestCase):
    def test_circle_front_surface_and_absent_returns(self):
        scene = OrchardScene(circles=(Circle(2.0, 0.0, 0.5),))
        scan = raycast_scan(scene, config=ScanConfig(rays=4))
        self.assertAlmostEqual(scan.ranges[2], 1.5)
        self.assertTrue(all(math.isinf(scan.ranges[i]) for i in (0, 1, 3)))

    def test_wall_occludes_farther_tree(self):
        scene = OrchardScene(circles=(Circle(3.0, 0.0, 0.5),),
                             segments=(Segment((2.0, -1.0), (2.0, 1.0)),))
        self.assertAlmostEqual(raycast_scan(scene, config=ScanConfig(rays=4)).ranges[2], 2.0)

    def test_near_blind_return_does_not_reveal_wall_behind_it(self):
        scene = OrchardScene(segments=(Segment((0.02, -1.0), (0.02, 1.0)),
                                       Segment((2.0, -1.0), (2.0, 1.0))))
        self.assertTrue(math.isinf(raycast_scan(scene, config=ScanConfig(rays=4)).ranges[2]))

    def test_sensor_pose_rotation_and_range_limit(self):
        scene = OrchardScene(circles=(Circle(0.0, 4.0, 0.5),))
        pose = Pose2D(0.0, 1.0, math.pi / 2)
        self.assertAlmostEqual(raycast_scan(scene, pose, config=ScanConfig(rays=4)).ranges[2], 2.5)
        self.assertTrue(math.isinf(raycast_scan(scene, pose, config=ScanConfig(rays=4, range_max=2.0)).ranges[2]))


class GeneratedSceneTests(unittest.TestCase):
    def test_all_requested_scenarios_are_deterministic_and_sensor_only(self):
        for name in SCENARIOS:
            with self.subTest(name=name):
                scene = generate_scene(name)
                self.assertEqual(scene, generate_scene(name))
                self.assertLessEqual(len(scene.circles), 200)
                self.assertLessEqual(len(scene.segments), 20)
                scan = raycast_scan(scene, field_pose(scene, 17.0, 0.0))
                self.assertEqual(len(scan.ranges), 3200)
                self.assertEqual(set(asdict(scan)), {"ranges", "angle_min", "angle_increment", "range_min", "range_max"})
                self.assertTrue(all(math.isinf(value) or 0.05 <= value <= 30 for value in scan.ranges))

    def test_field_rotation_preserves_sensor_view(self):
        config = SceneConfig(seed=8, row_width_m=4.2, tree_spacing_m=2.2, row_length_m=17.0)
        plain = generate_scene("wall_beyond_rows", config)
        rotated = generate_scene("wall_beyond_rows", replace(config, field_yaw_rad=0.73))
        a = raycast_scan(plain, field_pose(plain, 13.0, 0.2, 0.11))
        b = raycast_scan(rotated, field_pose(rotated, 13.0, 0.2, 0.11))
        np.testing.assert_allclose(a.ranges, b.ranges, atol=1e-9)

    def test_end_walls_and_open_end_have_known_forward_ranges(self):
        config = SceneConfig(missing_probability=0.0, position_jitter_m=0.0, radius_jitter_m=0.0)
        expected = {"open_end": math.inf, "wall_beyond_rows": 9.0,
                    "wall_across_before_end": 2.0, "narrow_headland": 6.6}
        for name, distance in expected.items():
            with self.subTest(name=name):
                scene = generate_scene(name, config)
                packet = raycast_scan(scene, field_pose(scene, 14.0, 0.0))
                self.assertAlmostEqual(packet.ranges[1600], distance)

    def test_missing_gap_is_interior_and_bounded(self):
        scene = generate_scene("missing_trunk_gap", SceneConfig(missing_probability=0.0))
        self.assertEqual(scene.oracle.missing_slots, ((1, 3), (1, 4), (1, 5)))
        self.assertEqual(len(scene.circles), 42)
        self.assertEqual(scene.oracle.row_end_x_m, 20.0)

    def test_next_row_is_hidden_by_opaque_side_segment(self):
        config = SceneConfig(position_jitter_m=0.0, missing_probability=0.0)
        scene = generate_scene("occluded_next_row", config)
        packet = raycast_scan(scene, field_pose(scene, 17.0, 0.0))
        # +90 degree ray crosses the side barrier at .75 * width.
        self.assertAlmostEqual(packet.ranges[2400], 0.75 * config.row_width_m)

    def test_seed_and_field_parameters_change_oracle_geometry(self):
        base = generate_scene()
        different = generate_scene(config=SceneConfig(seed=18, row_width_m=4.0,
                                                       tree_spacing_m=3.0, row_length_m=24.0))
        self.assertNotEqual(base.circles, different.circles)
        self.assertEqual(different.oracle.row_y_m[:2], (-2.0, 2.0))
        self.assertEqual(different.oracle.actual_tree_spacing_m, 3.0)
        self.assertEqual(different.oracle.row_end_x_m, 24.0)

    def test_resource_caps_and_invalid_geometry_reject(self):
        for kwargs in ({"rays": 3201}, {"rays": True}, {"range_max": 30.01}, {"range_min": -1.0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                ScanConfig(**kwargs)
        with self.assertRaises(ValueError):
            SceneConfig(row_count=10, row_length_m=100.0)
        with self.assertRaises(ValueError):
            OrchardScene(circles=(Circle(1.0, 0.0, 0.1),) * 201)
        with self.assertRaises(ValueError):
            OrchardScene(segments=(Segment((0.0, 0.0), (1.0, 0.0)),) * 21)
        with self.assertRaises(ValueError):
            generate_scene("occluded_next_row", SceneConfig(row_count=2))
        with self.assertRaises(ValueError):
            generate_scene("unknown")


if __name__ == "__main__":
    unittest.main()
