"""Pure checks of steering.gz_views (no Gazebo): the extra demo camera views and the parked-rover scripts."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from steering import gz_direct_drive as dd  # noqa: E402
from steering import gz_views as gv  # noqa: E402


class GzViewsTest(unittest.TestCase):
    def setUp(self):
        gv.install_views()
        gv.install_scripts()

    def test_views_are_installed_once_and_known_to_the_demo_tool(self):
        before = dict(dd.VIEWS)
        gv.install_views()
        self.assertEqual(dd.VIEWS, before)
        for name in gv.EXTRA_VIEWS:
            self.assertIn(name, dd.VIEWS)
            self.assertIn(name, dd.VIEW_KO)
        for views in gv.VIEW_SETS.values():
            self.assertTrue(set(views) <= set(dd.VIEWS))
        # the original demo views stay as they were
        self.assertEqual(dd.VIEWS['chase']['offset'], (-2.05, 1.0, 1.2))

    def test_cameras_look_at_the_rover(self):
        # rover at (2, 1) heading 30 deg: the front camera stands ahead of it and looks back at it
        x, y, z, yaw = 2., 1., 0., math.radians(30.)
        cx, cy, cz, roll, pitch, cam_yaw = dd.follow_pose(dd.VIEWS['model_front'], x, y, z, yaw)
        self.assertAlmostEqual(math.hypot(cx - x, cy - y), 2.7, places=6)
        self.assertAlmostEqual(math.atan2(math.sin(cam_yaw - yaw - math.pi), math.cos(cam_yaw - yaw - math.pi)), 0.,
                               places=6)
        self.assertGreater(pitch, 0.)                             # looks down
        sx, sy, _, _, _, side_yaw = dd.follow_pose(dd.VIEWS['model_side'], x, y, z, yaw)
        to_rover = math.atan2(y - sy, x - sx)
        self.assertLess(abs(math.atan2(math.sin(side_yaw - to_rover), math.cos(side_yaw - to_rover))), .1)
        tx, ty, tz, _, tp, _ = dd.follow_pose(dd.VIEWS['orchard_high'], x, y, z, yaw)
        self.assertAlmostEqual(tp, math.pi / 2.)                  # straight down
        self.assertAlmostEqual(tz, 36.)

    def test_parked_scripts(self):
        ctx = object()
        for world, name in gv.SCRIPT_NAMES.items():
            script = dd.make_script(name, ctx)
            self.assertEqual(script['world'], world)
            self.assertEqual(script['duty'](5.), 0.)                  # the rover never drives
            self.assertEqual(script['steer'](1.), 0.)
            self.assertEqual(script['steer'](6.), gv.STEER_DEG)
            times = [t for t, _, _ in script['stills']]
            self.assertTrue(all(0. < t < script['duration_s'] for t in times))
            self.assertEqual(set(script['stills'][0][2]) - {dd.ROVER_VIEW}, set(gv.VIEW_SETS[world]))
        with self.assertRaises(dd.DemoError):
            dd.make_script('no_such_script', object())


if __name__ == '__main__':
    unittest.main()
