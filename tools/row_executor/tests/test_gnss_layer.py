"""GNSS weak layer L3 (gnss_layer.py, perception_settings.GnssSettings): synthetic fixes only (no truth, no SIM)."""
import json
import math
import os
import random
import sys
import types
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from row_executor.gnss_layer import GnssLayer, LocalFrame, fix_from_message  # noqa: E402
from row_executor.perception_settings import GNSS_MODES, GnssSettings  # noqa: E402

ROT = math.radians(30.)


def to_gnss(x, y, bias=(4., -3.)):
    """Odometry (run frame) -> a rotated, shifted local GNSS frame."""
    return (math.cos(ROT) * x - math.sin(ROT) * y + 100. + bias[0],
            math.sin(ROT) * x + math.cos(ROT) * y - 50. + bias[1])


def run_layer(layer, n=80, v=.4, hacc=7., spikes=(), y_of=lambda x: 0., hold_s=0.):
    """1 Hz fixes of a vehicle driving along the odometry x axis; spikes: {k: (de, dn)}. Returns the outcomes."""
    outcomes = []
    for k in range(n):
        t = 10. + k
        x = max(0., k - hold_s) * v
        pose = (x, y_of(x), 0.)
        for j in range(20):
            xj = max(0., k - 1 + (j + 1) / 20. - hold_s) * v
            layer.note_pose((xj, y_of(xj), 0.))
        e, nn = to_gnss(x, pose[1])
        if k in spikes:
            e, nn = e + spikes[k][0], nn + spikes[k][1]
        outcomes.append(layer.on_fix(t, e, nn, hacc, True, pose))
    return outcomes


class GateTest(unittest.TestCase):
    def setUp(self):
        self.layer = GnssLayer(GnssSettings(mode='log_only'))
        self.layer.new_run(120.)

    def test_quality_order_and_no_fix(self):
        L = self.layer
        self.assertEqual(L.on_fix(1., 0., 0., 12., True, (0., 0., 0.)), 'quality')
        self.assertEqual(L.on_fix(2., 0., 0., None, True, (0., 0., 0.)), 'quality')
        self.assertEqual(L.on_fix(3., 0., 0., 5., False, (0., 0., 0.)), 'no_fix')
        self.assertEqual(L.on_fix(4., 0., 0., 5., True, (0., 0., 0.)), 'accepted')
        self.assertEqual(L.on_fix(4., 0., 0., 5., True, (0., 0., 0.)), 'order')
        self.assertTrue(L.present(4.5))
        self.assertFalse(L.present(7.))           # older than max_age_s = 2.5 s: GNSS absent

    def test_spikes_are_rejected_one_for_one(self):
        spikes = {20: (5., 0.), 35: (0., -15.), 50: (-3.6, 3.6)}
        outcomes = run_layer(self.layer, spikes=spikes)
        self.assertEqual(outcomes.count('jump'), len(spikes))
        self.assertEqual([k for k, o in enumerate(outcomes) if o == 'jump'], sorted(spikes))
        self.assertTrue(self.layer.summary(100.)['aligned'])

    def test_a_lasting_step_is_reanchored(self):
        spikes = {k: (8., 0.) for k in range(30, 80)}
        outcomes = run_layer(self.layer, spikes=spikes)
        self.assertEqual(outcomes[30:32], ['jump', 'jump'])
        self.assertEqual(outcomes[32], 'reanchored')
        self.assertEqual(outcomes[33:].count('jump'), 0)

    def test_gap_restarts_the_track(self):
        L = self.layer
        L.on_fix(1., 0., 0., 5., True, (0., 0., 0.))
        self.assertEqual(L.on_fix(20., 30., 0., 5., True, (0., 0., 0.)), 'restarted')

    def test_alignment_and_course(self):
        run_layer(self.layer)
        angle = math.degrees(self.layer.align[0])
        self.assertAlmostEqual(angle, 30., delta=.5)
        self.assertLess(abs(self.layer.summary(100.)['course_drift_deg']), 1.)
        self.assertLess(self.layer.summary(100.)['disagreement_m'], .5)
        self.assertEqual(self.layer.flags['course_disagree'], 0)

    def test_course_disagreement_flag(self):
        # the odometry says straight, GNSS shows the vehicle turned 60 deg after 35 m: odometry heading lost
        layer = self.layer
        for k in range(140):
            t = 10. + k
            x = k * .4
            for j in range(20):
                layer.note_pose(((k - 1 + (j + 1) / 20.) * .4, 0., 0.))
            if x <= 35.:
                gx, gy = x, 0.
            else:
                gx, gy = 35. + (x - 35.) * math.cos(math.radians(60.)), (x - 35.) * math.sin(math.radians(60.))
            e, n = to_gnss(gx, gy)
            layer.on_fix(t, e, n, 7., True, (x, 0., 0.))
        self.assertGreater(layer.flags['course_disagree'], 0)


def transition_run(layer, path_end=45., single=(29.0, 30.6), offset=(4.0, -3.5), seed=7, rate_hz=5., v=.4):
    """A FIXED-like receiver (white 0.01 m, hacc 0.014 m) with one standalone stretch (path `single`: the receiver's
    7 m-class solution, here a constant `offset` + white 0.4 m, hacc 7 m), as in the RTK matrix condition
    jog_fixed_transitions (gnss_rtk_weak_v1). The vehicle drives along odometry x. Returns [(path_m, outcome)]."""
    rng = random.Random(seed)
    dt = 1. / rate_hz
    out = []
    k = 0
    x = 0.
    while x <= path_end:
        t = 10. + k * dt
        for j in range(4):
            layer.note_pose((x - v * dt + v * dt * (j + 1) / 4., 0., 0.) if k else (0., 0., 0.))
        standalone = single[0] <= x < single[1]
        sigma, hacc = (.4, 7.) if standalone else (.01, .014)
        e, n = to_gnss(x, 0., bias=offset if standalone else (0., 0.))
        e, n = e + rng.gauss(0., sigma), n + rng.gauss(0., sigma)
        out.append((x, layer.on_fix(t, e, n, hacc, True, (x, 0., 0.))))
        k += 1
        x = k * v * dt
    return out


class ReanchorAlignmentTest(unittest.TestCase):
    """A reanchor drops the pairs before the receiver step; the alignment is refitted (and frozen) only once the new
    track is as long as the first fit needs (course_baseline_m, frozen at twice that), never from the short new track.
    The defect (gnss_rtk_weak_v1, 2 runs gnss_geofence): a standalone stretch that started before the freeze distance
    refitted the alignment from about 1 m of 0.4 m-noise fixes and froze it; the FIXED fixes after it were mapped with
    the wrong rotation and the standalone translation and the fence tripped."""

    def test_standalone_stretch_does_not_refit_the_alignment(self):
        layer = GnssLayer(GnssSettings(mode='geofence'))
        layer.new_run(120.)
        out = transition_run(layer, seed=10)      # before R101: alignment 4.4 deg, fence tripped (gnss_geofence)
        self.assertEqual([o for _, o in out].count('reanchored'), 2)       # into the standalone stretch and out of it
        self.assertAlmostEqual(math.degrees(layer.align[0]), 30., delta=.5)
        self.assertLess(layer.disagreement, 1.)
        self.assertEqual(layer.flags['position_disagree'], 0)
        self.assertFalse(layer.fence_tripped)
        self.assertIsNone(layer.end_reason())

    def test_refit_after_a_reanchor_waits_for_the_baseline(self):
        s = GnssSettings(mode='log_only')
        layer = GnssLayer(s)
        layer.new_run(120.)
        # first fit at 15 m, then a lasting 8 m step at 20 m (before the freeze at 30 m): the alignment of the first
        # track stays until the new track spans 15 m (35 m), refitted from there, frozen at 50 m of run path
        spikes = {k: (8., 0.) for k in range(50, 200)}
        aligns = []
        for k in range(200):
            x = k * .4
            for j in range(20):
                layer.note_pose(((k - 1 + (j + 1) / 20.) * .4 if k else 0., 0., 0.))
            e, n = to_gnss(x, 0.)
            if k in spikes:
                e, n = e + spikes[k][0], n + spikes[k][1]
            outcome = layer.on_fix(10. + k, e, n, 7., True, (x, 0., 0.))
            aligns.append((x, outcome, layer.align, layer.align_frozen))
        reanchor = [x for x, o, _, _ in aligns if o == 'reanchored']
        self.assertEqual(len(reanchor), 1)
        held = [a for x, _, a, _ in aligns if reanchor[0] <= x < reanchor[0] + s.course_baseline_m - .4]
        self.assertTrue(held and all(a is held[0] for a in held))          # the old alignment object, unchanged
        refit = [x for x, _, a, _ in aligns if x >= reanchor[0] and a is not held[0]]
        self.assertGreaterEqual(refit[0] - reanchor[0], s.course_baseline_m)
        frozen = [x for x, _, _, f in aligns if f]
        self.assertGreaterEqual(frozen[0] - reanchor[0], 2. * s.course_baseline_m)
        self.assertAlmostEqual(math.degrees(layer.align[0]), 30., delta=.5)

    def test_a_locked_alignment_survives_a_reanchor(self):
        layer = GnssLayer(GnssSettings(mode='log_only'))
        layer.new_run(120.)
        out = transition_run(layer, path_end=50., single=(35., 37.))
        self.assertEqual([o for _, o in out].count('reanchored'), 2)
        self.assertTrue(layer.align_frozen)
        self.assertAlmostEqual(math.degrees(layer.align[0]), 30., delta=.5)
        self.assertLess(layer.disagreement, 1.)

    def test_no_reanchor_keeps_the_first_fit_rule(self):
        # without a reanchor the alignment is fitted from the run path (as before the fix): first fit at the first
        # accepted fix with 15 m of run path, frozen at 30 m
        layer = GnssLayer(GnssSettings(mode='log_only'))
        layer.new_run(120.)
        first = frozen = None
        for k in range(100):
            x = k * .4
            for j in range(20):
                layer.note_pose(((k - 1 + (j + 1) / 20.) * .4 if k else 0., 0., 0.))
            e, n = to_gnss(x, 0.)
            layer.on_fix(10. + k, e, n, 7., True, (x, 0., 0.))
            if first is None and layer.align is not None:
                first = layer.path_m
            if frozen is None and layer.align_frozen:
                frozen = layer.path_m
        self.assertAlmostEqual(first, 15.2, delta=1e-6)
        self.assertAlmostEqual(frozen, 30., delta=.41)


class FenceTest(unittest.TestCase):
    def test_geofence_trips_only_in_geofence_mode(self):
        for mode in ('log_only', 'geofence'):
            with self.subTest(mode=mode):
                layer = GnssLayer(GnssSettings(mode=mode))
                layer.new_run(20.)
                # 20 m row: the vehicle goes on to 50 m (beyond 20 + margin 21 m)
                run_layer(layer, n=130)
                self.assertTrue(layer.fence_tripped)
                self.assertEqual(layer.end_reason(), 'gnss_geofence' if mode == 'geofence' else None)

    def test_no_trip_inside_the_row_with_noise_and_spikes(self):
        layer = GnssLayer(GnssSettings(mode='geofence'))
        layer.new_run(40.)
        wobble = lambda x: .3 * math.sin(x)      # noqa: E731
        run_layer(layer, n=95, spikes={20: (15., 0.), 60: (0., 15.)}, y_of=wobble)
        self.assertFalse(layer.fence_tripped)
        self.assertIsNone(layer.end_reason())

    def test_radial_fence_before_alignment(self):
        layer = GnssLayer(GnssSettings(mode='geofence'))
        layer.new_run(10.)
        layer.note_pose((0., 0., 0.))
        layer.on_fix(1., 0., 0., 5., True, (0., 0., 0.))
        for k in range(2, 6):          # a 30 m walk of GNSS while the odometry stands: jump rejected, then reanchored
            layer.on_fix(float(k), 30. + k * .1, 0., 5., True, (0., 0., 0.))
        self.assertGreater(layer.flags['fence_outside'], 0)


class SettingsAndAdapterTest(unittest.TestCase):
    def test_settings(self):
        self.assertEqual(GNSS_MODES, ('off', 'log_only', 'geofence', 'weak_constraint'))
        self.assertEqual(GnssSettings(mode='log_only').problems(), [])
        self.assertTrue(GnssSettings(mode='weak').problems())
        self.assertTrue(GnssSettings(mode='log_only', max_hacc_m=0.).problems())
        with self.assertRaises(ValueError):
            GnssLayer(GnssSettings())               # mode off: no layer

    def test_navsat_message_and_local_frame(self):
        frame = LocalFrame()
        msg = types.SimpleNamespace(latitude=1.0, longitude=2.0, status=types.SimpleNamespace(status=0),
                                    position_covariance=[49., 0., 0., 0., 36., 0., 0., 0., 100.],
                                    position_covariance_type=2)
        e, n, hacc, ok = fix_from_message(msg, frame)
        self.assertEqual((e, n, hacc, ok), (0., 0., 7., True))
        msg2 = types.SimpleNamespace(latitude=1.0001, longitude=2.0001, status=types.SimpleNamespace(status=0),
                                     position_covariance=[0.] * 9, position_covariance_type=0)
        e, n, hacc, ok = fix_from_message(msg2, frame)
        self.assertAlmostEqual(n, 11.12, delta=.05)
        self.assertAlmostEqual(e, 11.12 * math.cos(math.radians(1.)), delta=.05)
        self.assertIsNone(hacc)
        nofix = types.SimpleNamespace(latitude=0., longitude=0., status=types.SimpleNamespace(status=-1),
                                      position_covariance=[0.] * 9, position_covariance_type=0)
        self.assertFalse(fix_from_message(nofix, frame)[3])
        sim = types.SimpleNamespace(east_m=3., north_m=4., hacc_m=6., fix_ok=True)
        self.assertEqual(fix_from_message(sim, frame), (3., 4., 6., True))
        self.assertIsNone(fix_from_message(object(), frame))

    def test_summary_is_run_local(self):
        layer = GnssLayer(GnssSettings(mode='log_only'))
        layer.new_run(120.)
        run_layer(layer, n=40)
        text = json.dumps(layer.summary(60.))
        summary = json.loads(text)
        self.assertLess(max(abs(v) for v in summary['rel_m']), 30.)       # run-local metres, not the frame's
        self.assertNotIn('100', text.split('rel_m')[0][-10:])
        self.assertNotIn('origin', text)


if __name__ == '__main__':
    unittest.main()
