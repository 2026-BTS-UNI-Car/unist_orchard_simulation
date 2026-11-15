"""RTK weak constraint of the GNSS layer (gnss_rtk.py, gnss_mode 'weak_constraint'; docs/steering/GNSS_RTK_KO.md).

Synthetic drives only (no truth module, no SIM): the vehicle moves along a row centre line in the odometry frame at
0.4 m/s with a small true lateral wander; the LiDAR lateral offset is the true one plus an injected bias; the RTK fix is
the true antenna position in a rotated, shifted local frame (optionally mirrored), with spikes, steps or other solution
types. Every number is a test value, not a field value.
"""
import math
import os
import sys
import types
import unittest
from dataclasses import replace

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from row_executor.gnss_rtk import RtkConstraint, fix_type_from_message  # noqa: E402
from row_executor.perception_settings import RtkSettings  # noqa: E402

ROT = math.radians(30.)
TICK = .05
FIX_EVERY = 4                 # 5 Hz fixes on the 20 Hz tick


def to_rtk(x, y, mirror=False):
    east = math.cos(ROT) * x - math.sin(ROT) * y + 50.
    north = math.sin(ROT) * x + math.cos(ROT) * y - 20.
    return (north, east) if mirror else (east, north)


def drive(rtk, length_m=30., v=.4, wander=lambda s: .05 * math.sin(s / 1.5), bias=lambda s: 0.,
          fix=lambda t, s: ('fixed', .014), offset=lambda t, s: (0., 0.), mirror=False, curvature=0.,
          outcome=lambda t, s: 'accepted'):
    """Ticks of a drive along the row; returns [(t, s_m, u, target, state)]. fix(t, s) -> (type, hacc) or None (no fix
    that tick); offset(t, s) -> an error added to the RTK fix (east, north)."""
    out = []
    k = 0
    t = 100.
    while True:
        s_m = v * k * TICK
        if s_m > length_m:
            break
        y = wander(s_m)
        dy = (wander(s_m + 1e-3) - y) / 1e-3
        if curvature:
            r = 1. / curvature
            phi = s_m * curvature
            cx, cy = r * math.sin(phi), r * (1. - math.cos(phi))
            x, yy = cx - y * math.sin(phi), cy + y * math.cos(phi)
            yaw = phi + math.atan(dy)
        else:
            x, yy, yaw = s_m, y, math.atan(dy)
        snap = dict(corridor_valid=True, lateral_offset=y + bias(s_m), heading_error=-math.atan(dy),
                    corridor_reason='tracked_full_detector')
        if k % FIX_EVERY == 0:
            item = fix(t, s_m)
            if item is not None:
                east, north = to_rtk(x, yy, mirror)
                de, dn = offset(t, s_m)
                rtk.on_fix(t, east + de, north + dn, item[1], item[0], outcome(t, s_m), False)
        rtk.tick(t, snap, (x, yy, yaw))
        out.append((t, s_m, rtk.u, rtk.target, rtk.state))
        k += 1
        t += TICK
    return out


def steps(trace):
    return max(abs(b[2] - a[2]) for a, b in zip(trace, trace[1:]))


class FixTypeTest(unittest.TestCase):
    def test_message_mapping(self):
        ns = types.SimpleNamespace
        self.assertEqual(fix_type_from_message(ns(east_m=1., north_m=2., hacc_m=.01, fix_ok=True, fix_type='fixed')),
                         'fixed')
        self.assertEqual(fix_type_from_message(ns(east_m=1., north_m=2., hacc_m=7., fix_ok=True)), 'single')
        self.assertEqual(fix_type_from_message(ns(east_m=1., north_m=2., hacc_m=None, fix_ok=False)), 'none')
        self.assertEqual(fix_type_from_message(ns(fix_type='bogus')), 'unknown')
        for status, want in ((-1, 'none'), (0, 'single'), (1, 'dgps'), (2, 'rtk'), (7, 'unknown')):
            self.assertEqual(fix_type_from_message(ns(status=ns(status=status))), want)
        for quality, want in ((0, 'none'), (1, 'single'), (2, 'dgps'), (4, 'fixed'), (5, 'float'), (6, 'none')):
            self.assertEqual(fix_type_from_message(ns(quality=quality)), want)
        self.assertEqual(fix_type_from_message(ns(carr_soln=2, quality=1)), 'fixed')
        self.assertEqual(fix_type_from_message(ns(carr_soln=1)), 'float')
        self.assertEqual(fix_type_from_message(ns(carr_soln=0, quality=2)), 'dgps')
        self.assertEqual(fix_type_from_message(object()), 'unknown')

    def test_settings(self):
        self.assertEqual(RtkSettings().problems(), [])
        for bad in (dict(fix_types=('float',)), dict(fix_types=('fixed', 'single')), dict(fix_types=['fixed']),
                    dict(cap_m=.5), dict(gain=1.5), dict(min_span_m=30.), dict(frame_ratio=1.),
                    dict(rate_m_s=0.)):
            with self.subTest(bad=bad):
                self.assertTrue(replace(RtkSettings(), **bad).problems())
        with self.assertRaises(ValueError):
            RtkConstraint(RtkSettings(cap_m=1.))
        # the ExecutorConfig gnss_rtk_* defaults are the RtkSettings defaults
        from row_executor import perception_settings as PS
        from row_executor.config import ExecutorConfig
        self.assertEqual(PS.rtk_settings(ExecutorConfig()), RtkSettings())


class ConstraintTest(unittest.TestCase):
    def test_consistent_rtk_activates_and_stays_small(self):
        rtk = RtkConstraint()
        trace = drive(rtk)
        self.assertEqual(rtk.frame, 'verified')
        self.assertIsNotNone(rtk.line)
        self.assertEqual(trace[-1][4], 'active')
        self.assertLess(max(abs(r[2]) for r in trace), .02)
        self.assertLess(rtk.frame_rms[0], .01)                     # rotation fits ...
        self.assertGreater(rtk.frame_rms[1], 3. * rtk.frame_rms[0])    # ... the reflection does not
        self.assertGreater(rtk.counts['used'], 100)
        self.assertEqual(rtk.counts['rtk_jump'], 0)
        summary = rtk.summary(trace[-1][0])
        self.assertEqual(summary['state'], 'active')
        self.assertNotIn('east', str(summary))            # run-local values only, no position

    def test_lidar_bias_is_pulled_back_bounded_and_rate_limited(self):
        bias = .15
        rtk = RtkConstraint()
        trace = drive(rtk, bias=lambda s: -bias if 10. <= s < 16. else 0.)
        cfg = rtk.s
        inside = [r for r in trace if 14. <= r[1] < 16.]
        self.assertAlmostEqual(inside[-1][2], cfg.gain * bias, delta=.02)       # RTK - LiDAR = +bias
        self.assertLessEqual(max(abs(r[2]) for r in trace), cfg.cap_m + 1e-12)
        self.assertLessEqual(steps(trace), cfg.rate_m_s * TICK + 1e-12)
        self.assertLess(abs(trace[-1][2]), .02)                                 # back after the stretch
        self.assertEqual(rtk.counts['line_reset'], 0)
        # a larger bias saturates at the cap
        rtk = RtkConstraint(RtkSettings(max_disagree_m=.5))
        trace = drive(rtk, bias=lambda s: .3 if 10. <= s < 18. else 0.)
        self.assertAlmostEqual(min(r[2] for r in trace), -cfg.cap_m, delta=1e-9)
        # beyond max_disagree_m the line is dropped instead (a bend or a wrong line): the correction fades
        rtk = RtkConstraint()
        trace = drive(rtk, bias=lambda s: .3 if 10. <= s < 18. else 0.)
        self.assertGreaterEqual(rtk.counts['divergence_reset'], 1)
        self.assertLessEqual(max(abs(r[2]) for r in trace), cfg.cap_m + 1e-12)

    def test_lasting_disagreement_drops_the_line(self):
        rtk = RtkConstraint()
        trace = drive(rtk, length_m=40., bias=lambda s: .12 if s >= 10. else 0.)
        self.assertGreaterEqual(rtk.counts['line_reset'], 1)
        self.assertLess(abs(trace[-1][2]), .02)
        self.assertLessEqual(steps(trace), rtk.s.rate_m_s * TICK + 1e-12)

    def test_float_is_not_used_by_default(self):
        rtk = RtkConstraint()
        trace = drive(rtk, fix=lambda t, s: ('float', .25), bias=lambda s: .2 if s > 10. else 0.)
        self.assertEqual(max(abs(r[2]) for r in trace), 0.)
        self.assertEqual(rtk.counts['used'], 0)
        self.assertGreater(rtk.counts['fix_type'], 100)
        snap = dict(corridor_valid=True, lateral_offset=.1)
        self.assertIs(rtk.apply(snap), snap)
        for kind in ('rtk', 'single', 'dgps', 'none', 'unknown'):
            rtk = RtkConstraint()
            drive(rtk, length_m=6., fix=lambda t, s: (kind, .014))
            self.assertEqual(rtk.counts['used'], 0, kind)
        # with float allowed (and a looser accuracy gate) a float / NavSatFix GBAS fix is used
        rtk = RtkConstraint(RtkSettings(fix_types=('fixed', 'float'), max_hacc_m=.3))
        drive(rtk, length_m=12., fix=lambda t, s: ('rtk', .2))
        self.assertGreater(rtk.counts['used'], 50)

    def test_accuracy_gate(self):
        rtk = RtkConstraint()
        drive(rtk, length_m=8., fix=lambda t, s: ('fixed', .08))
        self.assertEqual(rtk.counts['used'], 0)
        self.assertGreater(rtk.counts['hacc'], 50)

    def test_transitions_fade_without_a_step(self):
        cfg = RtkSettings()

        def fix(t, s):
            if s < 13.:
                return 'fixed', .014
            if s < 15.:
                return 'float', .25
            if s < 17.:
                return None
            return 'fixed', .014
        rtk = RtkConstraint(cfg)
        trace = drive(rtk, length_m=24., fix=fix, bias=lambda s: -.15 if s >= 10. else 0.)
        self.assertLessEqual(steps(trace), cfg.rate_m_s * TICK + 1e-12)
        at_13 = [r for r in trace if r[1] < 13.][-1][2]
        self.assertGreater(at_13, .05)
        gap = [r for r in trace if 13. + .4 * cfg.hold_s + .3 <= r[1] < 17.]
        self.assertTrue(gap)
        self.assertTrue(all(r[2] < at_13 for r in gap))           # fading
        self.assertTrue(any(r[2] == 0. for r in gap))              # down to 0 before the fix returns

    def test_spike_is_rejected_and_a_lasting_step_restarts_the_track(self):
        plain = drive(RtkConstraint())
        rtk = RtkConstraint()
        spiked = drive(rtk, offset=lambda t, s: (.6, -.3) if abs(s - 12.) < .05 else (0., 0.))
        self.assertEqual(rtk.counts['rtk_jump'], 1)
        self.assertLess(max(abs(a[2] - b[2]) for a, b in zip(plain, spiked)), .005)
        rtk = RtkConstraint()
        trace = drive(rtk, length_m=40., offset=lambda t, s: (.5, .5) if 14. <= s < 20. else (0., 0.))
        self.assertGreaterEqual(rtk.counts['track_restart'], 1)
        self.assertGreaterEqual(rtk.counts['rtk_jump'], rtk.s.reanchor_after - 1)
        self.assertLessEqual(max(abs(r[2]) for r in trace), rtk.s.cap_m + 1e-12)
        self.assertLessEqual(steps(trace), rtk.s.rate_m_s * TICK + 1e-12)

    def test_layer_outcomes_and_flags_gate(self):
        rtk = RtkConstraint()
        drive(rtk, length_m=8., outcome=lambda t, s: 'jump')
        self.assertEqual(rtk.counts['used'], 0)
        self.assertGreater(rtk.counts['layer'], 50)
        rtk = RtkConstraint()
        rtk.on_fix(1., 0., 0., .01, 'fixed', 'accepted', True)
        rtk.tick(1., dict(corridor_valid=True, lateral_offset=0., heading_error=0.), (0., 0., 0.))
        self.assertEqual(rtk.counts['layer_flag'], 1)

    def test_mirrored_frame_is_refused(self):
        rtk = RtkConstraint()
        trace = drive(rtk, mirror=True, bias=lambda s: .2 if s > 15. else 0.)
        self.assertEqual(rtk.frame, 'rejected')
        self.assertEqual(max(abs(r[2]) for r in trace), 0.)
        self.assertLess(rtk.frame_rms[1], rtk.frame_rms[0])        # the reflection fits
        self.assertEqual(trace[-1][4], 'refused')
        # a scaled frame fits neither: never verified, nothing applied
        rtk = RtkConstraint()
        trace = drive(rtk, offset=lambda t, s: (.05 * s, 0.), bias=lambda s: .2 if s > 15. else 0.)
        self.assertEqual(rtk.frame, 'unverified')
        self.assertEqual(max(abs(r[2]) for r in trace), 0.)

    def test_lever_arm_error_cancels(self):
        # the executor assumes the antenna at the base; it sits 0.5 m ahead and 0.2 m left (a constant lever error)
        rtk = RtkConstraint()
        trace = drive(rtk, offset=lambda t, s: (0., 0.))
        lever = RtkConstraint(RtkSettings(antenna_x_m=.5, antenna_y_m=.2))
        trace_lever = drive(lever)
        self.assertEqual(lever.frame, 'verified')
        # only the heading wander (about 2 deg) x the 0.5 m lever remains: a few cm of disagreement x the gain
        self.assertLess(max(abs(a[2] - b[2]) for a, b in zip(trace, trace_lever)), .05)

    def test_curved_row_is_bounded(self):
        rtk = RtkConstraint()
        trace = drive(rtk, curvature=1. / 30.)
        self.assertLessEqual(max(abs(r[2]) for r in trace), rtk.s.cap_m + 1e-12)
        self.assertLessEqual(steps(trace), rtk.s.rate_m_s * TICK + 1e-12)
        self.assertIsNone(rtk.line)                                 # the long window is not straight
        self.assertLess(abs(trace[-1][2]), .01)

    def test_no_corridor_and_no_pose(self):
        rtk = RtkConstraint()
        rtk.on_fix(1., 0., 0., .01, 'fixed', 'accepted', False)
        rtk.tick(1., dict(corridor_valid=True, lateral_offset=0., heading_error=0.), None)
        self.assertEqual(rtk.counts['no_pose'], 1)
        rtk.on_fix(1.2, 0., 0., .01, 'fixed', 'accepted', False)
        rtk.tick(1.2, dict(corridor_valid=False, lateral_offset=None, heading_error=None), (0., 0., 0.))
        self.assertEqual(rtk.counts['no_corridor'], 1)
        snap = dict(corridor_valid=False, lateral_offset=None)
        rtk.u = .1
        self.assertIs(rtk.apply(snap), snap)
        out = rtk.apply(dict(corridor_valid=True, lateral_offset=.05, heading_error=.01))
        self.assertAlmostEqual(out['lateral_offset'], .15)
        self.assertEqual(out['heading_error'], .01)

    def test_new_run_clears(self):
        rtk = RtkConstraint()
        drive(rtk, bias=lambda s: .2 if s > 12. else 0.)
        rtk.new_run()
        self.assertEqual((rtk.u, rtk.frame, rtk.line, rtk.counts['used']), (0., 'unverified', None, 0))


class RunnerTest(unittest.TestCase):
    """The front_steer Runner with gnss_mode weak_constraint (the steering executor test's static orchard scene and
    stub controller)."""

    @classmethod
    def setUpClass(cls):
        import test_steer_executor as TS
        if TS.SCIPY_PROBLEM:
            raise unittest.SkipTest(TS.SCIPY_PROBLEM)
        cls.TS = TS

    def test_config_rules(self):
        from row_executor import config as C
        from row_executor.config import ExecutorConfig
        TS = self.TS
        with self.assertRaises(C.ConfigError):
            replace(ExecutorConfig(), gnss_mode='weak_constraint').validate()       # differential profile
        cfg = TS.steer_config(gnss_mode='weak_constraint')
        self.assertEqual(cfg.to_dict()['gnss_rtk_fix_types'], 'fixed')
        TS.steer_config(gnss_mode='weak_constraint', gnss_rtk_fix_types='fixed,float', gnss_rtk_max_hacc_m=.3)
        for bad in (dict(gnss_rtk_fix_types='float'), dict(gnss_rtk_fix_types='fixed,single'), dict(gnss_rtk_cap_m=.5),
                    dict(gnss_rtk_min_span_m=30.)):
            with self.subTest(bad=bad):
                self.assertTrue(TS.config_problems(gnss_mode='weak_constraint', **bad))
        self.assertFalse([k for k in ExecutorConfig().to_dict() if k.startswith('gnss_')])

    def test_without_a_fix_every_decision_and_controller_input_is_unchanged(self):
        TS = self.TS
        a, b = TS.StubSteer(), TS.StubSteer()
        _, _, out_a = TS.run_static(TS.steer_config(), a)
        runner, _, out_b = TS.run_static(TS.steer_config(gnss_mode='weak_constraint'), b)

        def trace(out):
            return [(round(t, 3), d.state, d.reason, d.throttle, d.steer_mrad) for t, d in out]
        self.assertEqual(trace(out_a), trace(out_b))
        self.assertTrue(a.snapshots)
        self.assertEqual(a.snapshots, b.snapshots)
        block = out_b[-1][1].summary['gnss']['rtk']
        self.assertEqual((block['state'], block['correction_m'], block['counts']['fixes']), ('learning', 0., 0))

    def test_a_correction_reaches_the_controller_only(self):
        TS = self.TS
        a, b = TS.StubSteer(), TS.StubSteer()
        _, _, out_a = TS.run_static(TS.steer_config(), a)
        from row_executor import gnss_rtk

        original = gnss_rtk.RtkConstraint.tick

        def fixed_u(self, now, s, base_pose):
            self.u = .1                       # a correction held at 0.1 m (the constraint's own rules are unit-tested)
        gnss_rtk.RtkConstraint.tick = fixed_u
        try:
            _, _, out_b = TS.run_static(TS.steer_config(gnss_mode='weak_constraint'), b)
        finally:
            gnss_rtk.RtkConstraint.tick = original
        self.assertEqual(len(a.snapshots), len(b.snapshots))
        valid = 0
        for sa, sb in zip(a.snapshots, b.snapshots):
            if sa.get('corridor_valid') is True:
                valid += 1
                self.assertAlmostEqual(sb['lateral_offset'], sa['lateral_offset'] + .1, places=9)
                self.assertEqual({k: v for k, v in sa.items() if k != 'lateral_offset'},
                                 {k: v for k, v in sb.items() if k != 'lateral_offset'})
            else:
                self.assertEqual(sa, sb)
        self.assertGreater(valid, 10)
        # the decisions log the raw LiDAR lateral offset (the stub controller ignores it: same commands)
        lat_a = [d.summary.get('lateral_offset_m') for _, d in out_a]
        lat_b = [d.summary.get('lateral_offset_m') for _, d in out_b]
        self.assertEqual(lat_a, lat_b)


if __name__ == '__main__':
    unittest.main()
