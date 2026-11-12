"""Unit tests for the C1 gate replay pieces (stdlib unittest; no ROS, no records, no motor).

Run: python -I -B -m unittest discover -s tools/row_executor/tests -p "test_replay_gate.py" -v
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
try:
    import scipy  # noqa: F401  (the vendored estimator needs it; some test environments run without)
except ImportError:
    raise unittest.SkipTest('scipy not available: the replay loads the vendored estimator')
import replay_adapters as A                # noqa: E402
import row_gate_report as G                # noqa: E402
import row_shadow_replay as R              # noqa: E402  (loads the vendored modules via core.load_vendor)

C = R.C

INC = 0.0019398535
N = 3240


class ScanAdapterTest(unittest.TestCase):
    def test_rotation_invalid_and_copy(self):
        raw = np.full(N, 5.)
        raw[0], raw[1], raw[2], raw[3] = 0., np.nan, 30., .03
        adapter = A.ScanAdapter()
        r, amin, inc, rmax, st = adapter.convert(raw, -math.pi, INC, 30.)
        self.assertTrue(np.isinf(r[:4]).all())
        self.assertEqual(st['invalid'], 4)
        self.assertEqual(raw[0], 0.)                       # caller's array untouched
        self.assertAlmostEqual(amin, A.wrap(-math.pi + math.radians(179.08)), places=12)
        # a return straight behind the LiDAR in the scan frame (scan angle ~ -179.08 deg) is now ahead (+x)
        k = int(round((math.radians(-179.08) + math.pi) / INC))
        self.assertLess(abs(A.wrap(amin + k * INC)), INC)

    def test_mask(self):
        mask = np.zeros(N)
        mask[10] = .8
        raw = np.full(N, 5.)
        raw[10], raw[11] = .5, .5
        r, *_, st = ScanAdapterTest._convert(A.ScanAdapter(mask_r=mask), raw)
        self.assertTrue(np.isinf(r[10]) and r[11] == .5 and st['masked'] == 1)
        r, *_, st = ScanAdapterTest._convert(A.ScanAdapter(mask_r=np.zeros(N + 1)), raw)
        self.assertTrue(st['mask_mismatch'] and r[10] == .5)

    def test_trim_extra_beam(self):
        r, amin, inc, rmax, st = A.ScanAdapter().convert(np.full(N + 1, 5.), -math.pi, INC, 30.)
        self.assertEqual(st['trimmed'], 1)
        self.assertLessEqual(len(r) * inc, 2 * math.pi + 1.01 * inc)
        r, *_, st = A.ScanAdapter().convert(np.full(N, 5.), -math.pi, INC, 30.)
        self.assertEqual(st['trimmed'], 0)

    @staticmethod
    def _convert(adapter, raw):
        return adapter.convert(raw, -math.pi, INC, 30.)


class TransientFilterTest(unittest.TestCase):
    def scan(self, near_at=None, near_r=.5, width=6):
        r = np.full(N, 8.)
        if near_at is not None:
            r[near_at:near_at + width] = near_r
        return r

    def test_one_scan_phantom_removed_persistent_kept(self):
        f = A.NearTransientFilter(near_m=2., tol_m=.35, confirm_scans=1)
        r = self.scan()
        self.assertEqual(f.apply(r, 0., INC), 0)              # no history yet: nothing removed
        r = self.scan(100)
        self.assertEqual(f.apply(r, 0., INC), 6)               # appears in one scan only: removed
        self.assertTrue(np.isinf(r[100:106]).all())
        r = self.scan(100, .45)                                # same object, next scan, 5 cm nearer: kept
        self.assertEqual(f.apply(r, 0., INC), 0)
        self.assertTrue((r[100:106] == .45).all())
        self.assertTrue((r[200:300] == 8.).all())              # far returns are never touched

    def test_confirm_two_scans(self):
        f = A.NearTransientFilter(near_m=2., tol_m=.35, confirm_scans=2)
        f.apply(self.scan(), 0., INC)
        f.apply(self.scan(), 0., INC)
        r = self.scan(50)
        self.assertEqual(f.apply(r, 0., INC), 6)
        r = self.scan(50)
        self.assertEqual(f.apply(r, 0., INC), 6)               # seen in only 1 of the 2 previous scans
        r = self.scan(50)
        self.assertEqual(f.apply(r, 0., INC), 0)               # third consecutive scan: kept


class ImuYawRateTest(unittest.TestCase):
    def test_level_and_tilted(self):
        y = A.ImuYawRate()
        for k in range(100):
            w = y.update(k * .05, (0., 0., .2), (0., 0., 9.81))
        self.assertAlmostEqual(w, .2, places=9)
        y = A.ImuYawRate()
        tilt = math.radians(8)
        up = (math.sin(tilt), 0., math.cos(tilt))
        for k in range(400):
            w = y.update(k * .05, (.2 * up[0], 0., .2 * up[2]), tuple(9.81 * u for u in up))
        self.assertAlmostEqual(w, .2, places=6)
        self.assertAlmostEqual(y.tilt_deg(), 8., places=4)


def snapshot(**kw):
    s = dict(lidar_age_s=.1, imu_age_s=.02, yaw_rate=0., gyro_heading=0., gyro_bias=0., speed=0., odom_valid=True,
             lidar_heading=0., distance_m=0., still=True, still_s=2., vibration_rms=.05, front_clearance_m=20.,
             side_clearance_m=1., side_clearance_left_m=1., side_clearance_right_m=1., side_nearest_x_left_m=0.,
             side_nearest_x_right_m=0., rotation_centre_x_m=0., rotation_centre_sd_m=.25, scan_count=10,
             pivot_clearance_m=2., pivot_radius_required_m=1.03, corridor_valid=True, corridor_reason='tracked',
             corridor_updates=5, next_corridor=dict(left='UNKNOWN', right='UNKNOWN'), heading_error=0.,
             lateral_offset=0., corridor_width=3.5, corridor_age_s=.1, corridor_full_updates=5,
             side_ends_ahead_m=[None, None], side_end_free=[False, False], row_end_ahead_m=None, end_age_s=None)
    s.update(kw)
    return s


class ControllerClockTest(unittest.TestCase):
    """RowController starts its REACQUIRE timer at since = 0.: it needs a run-relative clock."""
    def test_absolute_clock_times_out_on_first_tick(self):
        ctl = C.RowController(C.config_for('continuous'))
        cmd = ctl.step(1.79e9, snapshot())
        self.assertEqual((cmd.state, cmd.reason), ('HOLD', 'corridor_not_acquired'))

    def test_relative_clock_follows(self):
        ctl = C.RowController(C.config_for('continuous', **R.WEEKEND))
        cmd = ctl.step(0., snapshot())
        self.assertEqual(cmd.state, 'FOLLOW')
        self.assertEqual((cmd.left, cmd.right), (55, 55))


class CoreEquivalenceTest(unittest.TestCase):
    """The replay runs on the executor's core.ScanAdapter/ImuYawRate; the replay_adapters copies must agree."""
    def test_scan(self):
        rng = np.random.default_rng(3)
        raw = rng.uniform(.02, 35., N)
        raw[rng.integers(0, N, 200)] = 0.
        raw[rng.integers(0, N, 50)] = np.nan
        a = A.ScanAdapter().convert(raw, -math.pi, INC, 30.)
        b = R.X.ScanAdapter().convert(raw, -math.pi, INC, 30.)
        self.assertTrue(np.array_equal(a[0], b[0]))
        self.assertEqual(a[1:4], b[1:4])

    def test_imu(self):
        rng = np.random.default_rng(4)
        ya, yb = A.ImuYawRate(), R.X.ImuYawRate()
        for k in range(300):
            g = rng.normal(0, .1, 3)
            acc = np.array((.3, -.2, 9.8)) + rng.normal(0, .5, 3)
            wa = ya.update(k * .05, g, acc)
            wb = yb.update(k * .05, g, acc)[0]
            self.assertAlmostEqual(wa, wb, places=12)


class ScanStampTest(unittest.TestCase):
    def test_header_offset_survives_receive_burst(self):
        header = 100. + .1 * np.arange(60)
        receipt = header + .1
        receipt[30:33] = receipt[33]                 # three scans delivered at once after a hiccup
        rec = dict(t_scan=receipt, t_header=header)
        plain = R.scan_stamps(rec, {}, .08)
        self.assertAlmostEqual(float(np.min(np.diff(plain))), 0., places=9)   # receipt stamps collapse
        hdr = R.scan_stamps(rec, dict(stamp='header_offset'), .08)
        self.assertTrue(np.allclose(np.diff(hdr), .1))
        self.assertTrue(np.allclose(hdr, header + .1 - .08))


class GeometryGuardTest(unittest.TestCase):
    def test_guard(self):
        g = dict(heading_deg=25., body_half_width_m=.35, lateral_margin_m=0.)
        cfg = C.config_for('continuous')
        self.assertIsNone(R.geometry_guard(g, snapshot(), cfg))
        self.assertEqual(R.geometry_guard(g, snapshot(heading_error=math.radians(30)), cfg), 'guard_heading')
        self.assertEqual(R.geometry_guard(g, snapshot(lateral_offset=1.45, corridor_width=3.5), cfg), 'guard_lateral')
        self.assertIsNone(R.geometry_guard(g, snapshot(lateral_offset=1.35, corridor_width=3.5), cfg))
        self.assertIsNone(R.geometry_guard(g, snapshot(corridor_valid=False, heading_error=3.), cfg))


def stop(reason='row_end_candidate', dist=12., at_end=False, front=9.):
    return dict(reason=reason, dist_from_follow_m=dist, at_turn_or_end=at_end, front_clearance_m=front)


def summary(*controllers):
    """One record summary with one flying run per controller entry (stop dict or None, exposure)."""
    runs = [dict(kind='flying', controllers=[dict(controller='continuous_side/weekend', followed=True, stop=s,
                                                  exposure_m=e)]) for s, e in controllers]
    return dict(runs=runs)


class StrictPrematureTest(unittest.TestCase):
    """row_gate_report.premature(): the gate report's strict early stop (rebuilt from its definition)."""
    def test_definition(self):
        self.assertTrue(G.is_strict_premature(stop()))
        self.assertFalse(G.is_strict_premature(stop(at_end=True)))            # at a turn / segment end
        self.assertFalse(G.is_strict_premature(stop(dist=.99)))               # start refusal (< 1 m after FOLLOW)
        self.assertTrue(G.is_strict_premature(stop(dist=1.)))
        self.assertFalse(G.is_strict_premature(stop(dist=None)))
        need = G.required_front(G.STOP_V)                                     # 0.45 m/s stop model
        self.assertAlmostEqual(need, .45 * .4 + .45 * .45 / (2 * 1.07) + .35, places=12)
        self.assertTrue(G.is_strict_premature(stop('front_blocked', front=need - .01)))
        self.assertFalse(G.is_strict_premature(stop('front_blocked', front=need + .01)))   # only at the driven speed
        self.assertTrue(G.is_strict_premature(stop('guard_lateral', front=20.)))           # front rule: front only

    def test_figures(self):
        s = summary((stop(dist=20.), 20.), (None, 30.), (stop(dist=.4), .4), (stop(at_end=True, dist=9.6), 9.6),
                    (stop('front_blocked', dist=40., front=5.), 40.))
        out = G.premature([s], 'flying', 'continuous_side/weekend')
        self.assertEqual((out['exposure_m'], out['stops'], out['start_refusals_lt1m']), (100., 4, 1))
        self.assertEqual(out['premature_after_1m_at045'], 1)
        self.assertEqual(out['premature_reasons'], [['row_end_candidate', 1]])
        self.assertEqual(out['start_refusal_reasons'], [['row_end_candidate', 1]])
        self.assertEqual(out['m_per_premature'], 100.)
        self.assertEqual(out['rate_median_m'], round(100. * math.log(2.), 1))
        self.assertEqual(out['p_no_premature_30_45_60'], [round(math.exp(-d / 100.), 2) for d in (30., 45., 60.)])
        # Kaplan-Meier with the one strict stop (20 m) as the event: 4 of 5 runs at risk at 20 m -> survival 0.75
        self.assertEqual(out['km_median_p10'], [None, 20.])

    def test_no_premature_stop(self):
        out = G.premature([summary((None, 10.), (stop(at_end=True, dist=5.), 5.))], 'flying', 'continuous_side/weekend')
        self.assertEqual(out['premature_after_1m_at045'], 0)
        self.assertIsNone(out['m_per_premature'])
        self.assertIsNone(out['p_no_premature_30_45_60'])
        self.assertEqual(G.premature([summary()], 'standing', 'continuous_side/weekend')['stops'], 0)


if __name__ == '__main__':
    unittest.main()
