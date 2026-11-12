"""Local grid layer L2 (local_grid.py, perception_settings.GridSettings): synthetic scans only (no truth, no SIM)."""
import base64
import math
import os
import sys
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np  # noqa: E402

from row_executor.local_grid import LocalGrid, SOURCE_NAME  # noqa: E402
from row_executor.perception_settings import GridSettings  # noqa: E402

BEAMS = 1800
AMIN = -math.pi
INC = 2 * math.pi / BEAMS
LEVER = .18


def trunks(half_width=1.8, spacing=1.0, x0=-15., x1=40., radius=.1, axis=0., offset=0.):
    """Two rows of trunk circles parallel to `axis` (odometry frame), centre line through (0, offset) normal-wise."""
    c, s = math.cos(axis), math.sin(axis)
    out = []
    x = x0
    while x <= x1 + 1e-9:
        for side in (1, -1):
            n = offset + side * half_width
            out.append((x * c - n * s, x * s + n * c, radius))
        x += spacing
    return out


def scan(laser, circles, noise=0., rng=None):
    lx, ly, lyaw = laser
    angles = AMIN + np.arange(BEAMS) * INC + lyaw
    dx, dy = np.cos(angles), np.sin(angles)
    r = np.full(BEAMS, np.inf)
    for cx, cy, rho in circles:
        px, py = cx - lx, cy - ly
        b = dx * px + dy * py
        disc = b * b - (px * px + py * py - rho * rho)
        ok = disc >= 0
        t = np.where(ok, b - np.sqrt(np.where(ok, disc, 0.)), np.inf)
        t[t <= 0] = np.inf
        r = np.minimum(r, t)
    r[r > 30.] = np.inf
    if noise and rng is not None:
        r = r + rng.normal(0., noise, BEAMS)
    return r


def drive(grid, circles, poses, every=1, noise=0., seed=0, prior=None):
    """Feed the grid one scan per base pose; returns the fits."""
    rng = np.random.default_rng(seed)
    fits = []
    for k, (bx, by, yaw) in enumerate(poses):
        laser = (bx + LEVER * math.cos(yaw), by + LEVER * math.sin(yaw), yaw)
        fit = grid.on_scan(k * .1, scan(laser, circles, noise, rng), AMIN, INC, laser, True, (bx, by, yaw), prior)
        if fit is not None:
            fits.append(fit)
    return fits


def straight(n, y=0., yaw=0., v=.04):
    return [(k * v * math.cos(yaw), y + k * v * math.sin(yaw), yaw) for k in range(n)]


class FitTest(unittest.TestCase):
    def test_centred_vehicle_fit(self):
        grid = LocalGrid()
        fits = drive(grid, trunks(), straight(30))
        self.assertTrue(fits)
        fit = fits[-1]
        lateral, heading = fit.observation(straight(30)[-1])
        self.assertAlmostEqual(lateral, 0., delta=.05)
        self.assertAlmostEqual(math.degrees(heading), 0., delta=.5)
        self.assertAlmostEqual(fit.width, 3.6, delta=.25)    # cell centres of trunk faces: width ~ 2 x (1.8 - radius)
        self.assertEqual(fit.quality, 1.)

    def test_offset_and_heading_recovered(self):
        for lateral0, yaw0 in ((.4, 0.), (-.35, math.radians(5.)), (.2, math.radians(-7.))):
            with self.subTest(lateral=lateral0, yaw=yaw0):
                grid = LocalGrid()
                poses = [(x, lateral0 + x * math.tan(yaw0), yaw0) for x in np.arange(20) * .04]
                fits = drive(grid, trunks(), poses)
                self.assertTrue(fits)
                lateral, heading = fits[-1].observation(poses[-1])
                self.assertAlmostEqual(lateral, poses[-1][1], delta=.06)
                self.assertAlmostEqual(math.degrees(heading), -math.degrees(yaw0), delta=.6)

    def test_rotated_rows_and_noise(self):
        axis = math.radians(30.)
        grid = LocalGrid()
        poses = [(k * .04 * math.cos(axis) - .3 * math.sin(axis), k * .04 * math.sin(axis) + .3 * math.cos(axis),
                  axis + math.radians(2.)) for k in range(30)]
        fits = drive(grid, trunks(axis=axis), poses, noise=.02, prior=axis)
        lateral, heading = fits[-1].observation(poses[-1])
        self.assertAlmostEqual(lateral, .3, delta=.07)
        self.assertAlmostEqual(math.degrees(heading), -2., delta=.8)

    def test_one_sided_view_gives_no_fit(self):
        grid = LocalGrid()
        left = [c for c in trunks() if c[1] > 0]
        fits = drive(grid, left, straight(20))
        self.assertEqual(fits, [])
        self.assertEqual(grid.last_fit_reason, 'one_sided')

    def test_band_prior_confirms_its_lines_or_gives_no_fit(self):
        # a third line of posts inside the aisle at +0.9 m: without a prior the nearest band (0.9 m) is the left line
        inner = [(x, .9, .1) for x in np.arange(-15., 40., 1.)]
        circles = trunks() + inner
        poses = straight(20)
        free = LocalGrid()
        fit = drive(free, circles, poses)[-1]
        self.assertAlmostEqual(fit.left_n, .8, delta=.12)
        banded = LocalGrid()
        fits = []
        rng = np.random.default_rng(0)
        for k, pose in enumerate(poses):        # the estimator's band: lines at +-1.7 m (trunk faces)
            laser = (pose[0] + LEVER, 0., 0.)
            out = banded.on_scan(k * .1, scan(laser, circles, 0., rng), AMIN, INC, laser, True, pose, 0., (1.7, -1.7))
            if out is not None:
                fits.append(out)
        self.assertAlmostEqual(fits[-1].left_n, 1.7, delta=.12)
        self.assertEqual(fits[-1].search, 'band')
        lonely = LocalGrid()
        for k, pose in enumerate(poses):        # a band 1 m off the real lines: not confirmed, no fit
            laser = (pose[0] + LEVER, 0., 0.)
            self.assertIsNone(lonely.on_scan(k * .1, scan(laser, trunks()), AMIN, INC, laser, True, pose, 0.,
                                             (2.8, -.7)))
        self.assertTrue(lonely.last_fit_reason.startswith('unconfirmed_'))

    def test_orientation_follows_the_band(self):
        grid = LocalGrid()
        poses = straight(20)
        rng = np.random.default_rng(0)
        fit = None
        for k, pose in enumerate(poses):        # a band oriented against travel (heading pi)
            laser = (pose[0] + LEVER, 0., 0.)
            out = grid.on_scan(k * .1, scan(laser, trunks(), 0., rng), AMIN, INC, laser, True, pose, math.pi,
                               (1.7, -1.7))
            fit = out or fit
        self.assertGreater(abs(fit.axis), math.radians(170.))
        lateral, heading = fit.observation(poses[-1])
        self.assertAlmostEqual(abs(math.degrees(heading)), 180., delta=1.)
        self.assertAlmostEqual(lateral, 0., delta=.06)

    def test_track_search_follows_the_last_fit(self):
        grid = LocalGrid()
        fits = drive(grid, trunks(), straight(20))
        self.assertEqual(fits[0].search, 'window')
        self.assertEqual(fits[-1].search, 'track')


class GridTest(unittest.TestCase):
    def test_every_n_scans_and_invalid_odometry(self):
        grid = LocalGrid(GridSettings(every_n_scans=3))
        laser = (LEVER, 0., 0.)
        r = scan(laser, trunks())
        for k in range(6):
            grid.on_scan(k * .1, r, AMIN, INC, laser, k != 2, (0., 0., 0.))
        # scans 3 and 6 are update slots; scan 3 is index 2 (invalid) -> skipped
        self.assertEqual(grid.updates, 1)
        self.assertEqual(grid.skipped_invalid, 1)

    def test_persistence_rejects_a_single_scan_speckle(self):
        grid = LocalGrid(GridSettings(every_n_scans=1))
        poses = straight(10)
        rng = np.random.default_rng(1)
        for k, pose in enumerate(poses):
            laser = (pose[0] + LEVER, 0., 0.)
            circles = trunks() + ([(3., .5, .1)] if k == 4 else [])      # an object seen in one scan only
            grid.on_scan(k * .1, scan(laser, circles, .0, rng), AMIN, INC, laser, True, pose)
        pts = grid.occupied_points()
        near = np.hypot(pts[:, 0] - 3., pts[:, 1] - .5) < .3
        self.assertEqual(int(near.sum()), 0)
        self.assertGreater(len(pts), 50)

    def test_free_carving_clears_a_moved_object(self):
        settings = GridSettings(every_n_scans=1, free_beam_step=1)
        grid = LocalGrid(settings)
        laser = (LEVER, 0., 0.)
        wall = [(8., y, .1) for y in np.arange(-1.5, 1.51, .15)]     # rays need a return behind it to carve
        with_obj = trunks() + wall + [(3., 0., .15)]
        for k in range(5):
            grid.on_scan(k * .1, scan(laser, with_obj), AMIN, INC, laser, True, (0., 0., 0.))
        pts = grid.occupied_points()
        self.assertTrue((np.hypot(pts[:, 0] - 2.85, pts[:, 1]) < .2).any())
        for k in range(5, 20):
            grid.on_scan(k * .1, scan(laser, trunks() + wall), AMIN, INC, laser, True, (0., 0., 0.))
        pts = grid.occupied_points()
        self.assertFalse((np.hypot(pts[:, 0] - 2.85, pts[:, 1]) < .2).any())

    def test_recentre_moves_the_window_and_drops_old_cells(self):
        grid = LocalGrid(GridSettings(every_n_scans=1))
        drive(grid, trunks(), straight(10))
        before = grid.i0
        drive(grid, trunks(), [(x, 0., 0.) for x in np.arange(10) * .5 + 10.])
        self.assertGreater(grid.i0, before)
        self.assertGreater(grid.recentres, 0)
        pts = grid.occupied_points()
        self.assertGreaterEqual(pts[:, 0].min(), 14.5 - 6. - 1.1)    # behind the window (lags <= recentre_m)

    def test_settings_problems(self):
        self.assertEqual(GridSettings().problems(), [])
        self.assertTrue(GridSettings(cell_m=0.).problems())
        self.assertTrue(GridSettings(behind_m=25.).problems())
        self.assertTrue(GridSettings(fit_cross_max_m=6.).problems())
        self.assertTrue(GridSettings(every_n_scans=1.5).problems())
        with self.assertRaises(ValueError):
            LocalGrid(GridSettings(width_min_m=9.))


class ExportTest(unittest.TestCase):
    def setUp(self):
        self.grid = LocalGrid()
        self.poses = straight(30)
        drive(self.grid, trunks(), self.poses)

    def test_bitmap_round_trip(self):
        data = self.grid.export(3., self.poses[-1])
        self.assertEqual(data['encoding'], 'bitmap')
        self.assertFalse(data['truncated'])
        bits = np.unpackbits(np.frombuffer(base64.b64decode(data['bits']), dtype=np.uint8))
        shape = data['shape']
        mask = bits[:shape[0] * shape[1]].reshape(shape).astype(bool)
        self.assertTrue(np.array_equal(mask, self.grid.occupied()))
        self.assertEqual(data['occupied'], int(mask.sum()))
        self.assertIn('lines', data)
        self.assertLessEqual(len(data['bits']), 16384)

    def test_bitmap_is_pooled_to_fit_the_cap(self):
        data = self.grid.export(3., None, max_bytes=800)
        self.assertGreater(data['cell_m'], self.grid.res)
        self.assertLessEqual(len(data['bits']), 800)

    def test_cells_encoding_nearest_first_and_capped(self):
        data = self.grid.export(3., self.poses[-1], encoding='cells', max_cells=50)
        self.assertEqual(len(data['cells']), 50)
        self.assertTrue(data['truncated'])
        with self.assertRaises(ValueError):
            self.grid.export(3., None, encoding='png')

    def test_summary_is_small(self):
        import json
        text = json.dumps(self.grid.summary(3.))
        self.assertLess(len(text), 600)
        self.assertEqual(SOURCE_NAME, 'local_grid')


if __name__ == '__main__':
    unittest.main()
