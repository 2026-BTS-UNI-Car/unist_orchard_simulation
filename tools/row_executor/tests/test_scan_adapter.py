import math
import unittest

import numpy as np

import rowexec_testlib as L
from row_executor.core import ScanAdapter, ScanRejected


def replay_conversion(ranges_mm, amin, offset_deg=-179.08):
    """resume-audit-20261013/sim-review/sim2real/replay.py conversion (for equality checks)."""
    r = ranges_mm.astype(np.float64) / 1000.
    r[ranges_mm == 0] = np.inf
    return r, amin + math.radians(-offset_deg)


class ScanAdapterTest(unittest.TestCase):
    def setUp(self):
        self.adapter = ScanAdapter()

    def test_rotation_only_no_reordering(self):
        ranges = np.linspace(.5, 20., L.BEAMS)
        r, amin, inc, rmax, stats = self.adapter.convert(ranges, L.RAW_AMIN, L.RAW_INC, 30., .05)
        self.assertTrue(np.array_equal(r, ranges))
        self.assertEqual(inc, L.RAW_INC)
        self.assertEqual(rmax, 30.)
        # The replay used -pi + 179.08 deg unwrapped; it is already inside (-pi, pi].
        self.assertAlmostEqual(amin, L.RAW_AMIN + math.radians(179.08), places=12)
        self.assertEqual(stats, dict(beams_in=3240, beams=3240, trimmed=0, invalid=0, masked=0, near_0p5=0))

    def test_angle_wrapped(self):
        _, amin, _, _, _ = ScanAdapter(yaw_offset_deg=179.08).convert(np.ones(100), 3., .01, 30.)
        self.assertAlmostEqual(amin, 3. + math.radians(179.08) - 2 * math.pi, places=12)
        self.assertTrue(-math.pi < amin <= math.pi)

    def test_invalid_returns_become_inf_and_input_untouched(self):
        ranges = np.full(L.BEAMS, 5.)
        ranges[:6] = (0., np.nan, -1., .04, 30., 31.)
        original = ranges.copy()
        r, _, _, _, stats = self.adapter.convert(ranges, L.RAW_AMIN, L.RAW_INC, 30., .05)
        self.assertTrue(np.all(np.isinf(r[:6])))
        self.assertTrue(np.all(r[6:] == 5.))
        self.assertEqual(stats['invalid'], 6)
        self.assertTrue(np.array_equal(original, ranges, equal_nan=True))
        lst = list(ranges)                     # a plain list (rclpy array.array / list) also works
        r2, *_ = self.adapter.convert(lst, L.RAW_AMIN, L.RAW_INC, 30., .05)
        self.assertTrue(np.array_equal(r, r2))

    def test_matches_replay_conversion_on_real_like_data(self):
        rng = np.random.default_rng(3)
        mm = rng.integers(50, 29999, L.BEAMS).astype(np.uint16)
        mm[rng.random(L.BEAMS) < .2] = 0
        expected, amin_expected = replay_conversion(mm, L.RAW_AMIN)
        r, amin, _, _, _ = self.adapter.convert(mm.astype(np.float64) / 1000., L.RAW_AMIN, L.RAW_INC, 30., .05)
        self.assertTrue(np.array_equal(r, expected))
        self.assertAlmostEqual(amin, amin_expected, places=12)

    def test_beam_trim_keeps_scan_corridor_geometry(self):
        for extra, trimmed in ((0, 0), (1, 1), (3, 3)):
            n = L.BEAMS + extra
            with self.subTest(n=n):
                r, amin, inc, _, stats = self.adapter.convert(np.full(n, 3.), L.RAW_AMIN, L.RAW_INC, 30.)
                self.assertEqual(stats['trimmed'], trimmed)
                self.assertEqual(len(r), L.BEAMS)
                self.assertLessEqual(len(r) * inc, 2 * math.pi + inc * 1.01)   # scan_corridor.py rule
        with self.assertRaisesRegex(ScanRejected, 'invalid_scan_geometry'):
            self.adapter.convert(np.full(L.BEAMS + 5, 3.), L.RAW_AMIN, L.RAW_INC, 30.)

    def test_rejections(self):
        cases = ((np.ones(4097) * 3, L.RAW_AMIN, 1e-4, 30., 'scan_capacity_exceeded'),
                 (np.ones(10), L.RAW_AMIN, 0., 30., 'invalid_scan_geometry'),
                 (np.ones(10), float('nan'), .01, 30., 'invalid_scan_geometry'),
                 (np.ones(10), 7., .01, 30., 'invalid_scan_geometry'),
                 (np.ones(10), 0., .01, float('inf'), 'invalid_scan_geometry'),
                 (np.ones((2, 5)), 0., .01, 30., 'invalid_ranges'),
                 (np.ones(2), 0., .01, 30., 'too_few_beams'),
                 (['a', 'b', 'c'], 0., .01, 30., 'invalid_ranges'))
        for ranges, amin, inc, rmax, why in cases:
            with self.subTest(why=why), self.assertRaises(ScanRejected) as ctx:
                self.adapter.convert(ranges, amin, inc, rmax)
            self.assertEqual(ctx.exception.reason, why)

    def test_range_max_clipped_to_30(self):
        ranges = np.full(100, 35.)
        ranges[0] = 29.
        r, _, _, rmax, _ = self.adapter.convert(ranges, 0., .01, 40.)
        self.assertEqual(rmax, 30.)
        self.assertEqual(r[0], 29.)
        self.assertTrue(np.all(np.isinf(r[1:])))

    def test_body_mask_by_beam(self):
        adapter = ScanAdapter(body_mask=((10, 12, .5), (3000, 3000, .3)))
        ranges = np.full(L.BEAMS, 5.)
        ranges[[9, 10, 11, 12, 13, 3000]] = (.4, .4, .5, .6, .3, .29)
        r, _, _, _, stats = adapter.convert(ranges, L.RAW_AMIN, L.RAW_INC, 30., .05)
        self.assertEqual(r[9], .4)                  # outside the segment
        self.assertTrue(np.isinf(r[10]) and np.isinf(r[11]))   # <= mask range
        self.assertEqual(r[12], .6)                 # farther than the mask: kept (never hides real objects)
        self.assertEqual(r[13], .3)
        self.assertTrue(np.isinf(r[3000]))
        self.assertEqual(stats['masked'], 3)

    def test_body_mask_follows_angle_when_grid_shifts(self):
        adapter = ScanAdapter(body_mask=((100, 100, .5),))
        ranges = np.full(L.BEAMS, .4)
        r, *_ = adapter.convert(ranges, L.RAW_AMIN + L.RAW_INC, L.RAW_INC, 30.)
        # beam 99 of a scan starting one increment later points where grid beam 100 points
        self.assertTrue(np.isinf(r[99]))
        self.assertEqual(int(np.count_nonzero(np.isinf(r))), 1)

    def test_bad_mask_segment_refused(self):
        for seg in ((5, 4, .5), (0, 3240, .5), (0, 1, 1.3), (0, 1, 0.)):
            with self.subTest(seg=seg), self.assertRaises(ValueError):
                ScanAdapter(body_mask=(seg,))


if __name__ == '__main__':
    unittest.main()
