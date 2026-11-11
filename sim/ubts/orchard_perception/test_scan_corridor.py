"""Independent ray/wall/vegetation scenes for an observation-only candidate."""
from dataclasses import replace
import math
from types import SimpleNamespace
import unittest

import numpy as np

from .scan_corridor import CorridorConfig,axis_difference,observe_scan_corridor


def scene_scan(*,rotation=0.,left=True,right=True,front_wall=None,back_wall=None,branches=False,rays=3240):
    angles=np.linspace(-math.pi,math.pi,rays,endpoint=False)
    # Independent intersection in scene coordinates; candidate receives only
    # scalar ranges in the rotated laser frame, never this scene geometry.
    dx,dy=np.cos(angles-rotation),np.sin(angles-rotation)
    ranges=np.full(rays,np.inf)
    if branches:
        rng=np.random.default_rng(139)
        for y in ([2.] if left else [])+([-2.] if right else []):
            for x in np.arange(-10.,10.,.35):
                cy=y+rng.normal(0,.10);radius=.23
                along=x*dx+cy*dy;disc=along**2-(x*x+cy*cy-radius**2)
                hit=(along>0)&(disc>=0)
                distance=along-np.sqrt(np.maximum(0,disc))
                ranges[hit]=np.minimum(ranges[hit],distance[hit])
    else:
        for y in ([2.] if left else [])+([-2.] if right else []):
            hit=dy*y>0
            distance=np.divide(y,dy,out=np.full(rays,np.inf),where=hit)
            hit&=np.abs(np.where(hit,distance,0)*dx)<=10
            ranges[hit]=np.minimum(ranges[hit],distance[hit])
    for x in (front_wall,back_wall):
        if x is None:continue
        hit=dx*x>0
        distance=np.divide(x,dx,out=np.full(rays,np.inf),where=hit)
        hit&=np.abs(np.where(hit,distance,0)*dy)<=2.1
        ranges[hit]=np.minimum(ranges[hit],distance[hit])
    return SimpleNamespace(ranges=ranges,angle_min=-math.pi,angle_increment=2*math.pi/rays,
                           range_min=.05,range_max=30.)


class ScanCorridorTests(unittest.TestCase):
    def test_parallel_walls_report_geometry_without_tree_or_free_space_claim(self):
        scan=scene_scan();before=scan.ranges.copy();r=observe_scan_corridor(scan)
        self.assertTrue(r.valid,r)
        self.assertAlmostEqual(r.width_m,4.,delta=.2)
        self.assertLess(axis_difference(r.heading_axis_laser_rad,0),math.radians(3))
        self.assertEqual(r.last_row,'UNKNOWN')
        for claim in ('runtime_authority','free_space_verified','semantic_tree_rows_verified','forward_direction_verified'):
            self.assertFalse(getattr(r,claim))
        np.testing.assert_array_equal(scan.ranges,before)

    def test_branch_bands_do_not_require_compact_trunk_clusters(self):
        r=observe_scan_corridor(scene_scan(branches=True))
        self.assertTrue(r.valid,r)
        self.assertAlmostEqual(r.width_m,3.6,delta=.35)
        self.assertLess(axis_difference(r.heading_axis_laser_rad,0),math.radians(4))
        self.assertFalse(r.semantic_tree_rows_verified)

    def test_unknown_mount_near_ninety_degree_seam_preserves_axis(self):
        for yaw in (math.radians(80),math.radians(89),math.radians(-89)):
            r=observe_scan_corridor(scene_scan(rotation=yaw))
            self.assertTrue(r.valid,r)
            self.assertLess(axis_difference(r.heading_axis_laser_rad,yaw),math.radians(3))
            self.assertFalse(r.forward_direction_verified)

    def test_one_wall_does_not_manufacture_an_aisle(self):
        r=observe_scan_corridor(scene_scan(right=False))
        self.assertFalse(r.valid,r)

    def test_visible_wall_in_the_corridor_is_kept_as_an_intrusion(self):
        scan=scene_scan(front_wall=4.)
        before=scan.ranges.copy();r=observe_scan_corridor(scan)
        self.assertTrue(r.valid,r)
        self.assertGreater(r.interior_returns,100)
        self.assertFalse(r.free_space_verified)
        np.testing.assert_array_equal(scan.ranges,before)

    def test_equally_plausible_room_axes_are_rejected(self):
        r=observe_scan_corridor(scene_scan(front_wall=2.,back_wall=-2.))
        self.assertFalse(r.valid,r)
        self.assertEqual(r.reason,'ambiguous_parallel_surface_bands')

    def test_real_packet_size_and_invalid_ranges_have_finite_work_bounds(self):
        self.assertEqual(observe_scan_corridor(scene_scan()).diagnostics['scan_points'],3240)
        self.assertEqual(observe_scan_corridor(scene_scan(rays=4097)).reason,'scan_capacity_exceeded')
        scan=scene_scan();scan.ranges[:]=np.nan
        self.assertFalse(observe_scan_corridor(scan).valid)
        scan.angle_increment=-.001
        self.assertEqual(observe_scan_corridor(scan).reason,'invalid_scan_geometry')
        with self.assertRaises(ValueError):CorridorConfig(heading_step_deg=.01)

    def test_support_roi_never_edits_unobserved_or_far_collision_evidence(self):
        scan=scene_scan();scan.ranges[100:120]=16.
        before=scan.ranges.copy();r=observe_scan_corridor(scan)
        self.assertGreaterEqual(r.diagnostics['outside_support_returns'],20)
        np.testing.assert_array_equal(scan.ranges,before)
        self.assertFalse(r.free_space_verified)


if __name__=='__main__':unittest.main()
