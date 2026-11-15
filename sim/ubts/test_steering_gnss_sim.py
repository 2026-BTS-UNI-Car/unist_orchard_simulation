"""Virtual GNSS of the T1 rig (steering/gnss_sim.py, PC-12): rate, noise model, dropouts, jumps, accuracy windows."""
import math
import sys
import unittest

sys.dont_write_bytecode = True

import numpy as np  # noqa: E402

from steering import gnss_sim as G  # noqa: E402


def run(spec, seconds=60., seed=7, v=.4):
    em = G.GnssEmulator(spec, seed)
    out = []
    for k in range(int(seconds * 100)):
        t = k / 100.
        fix = em.sample(t, v * t, 0., 0.)
        if fix is not None:
            out.append((t, fix))
    return em, out


class GnssSimTest(unittest.TestCase):
    def test_rate_phase_and_determinism(self):
        em, out = run({})
        self.assertEqual(len(out), 60)
        self.assertTrue(all(abs((t * 1000.) % 1000. - 230.) < 1e-6 for t, _ in out))
        _, again = run({})
        self.assertEqual(out, again)
        _, other = run({}, seed=8)
        self.assertNotEqual(out, other)

    def test_error_size_and_frame(self):
        errors = []
        for seed in range(20):
            _, out = run({}, seed=seed)
            for t, fix in out:
                x = .4 * t
                c, s = math.cos(math.radians(30.)), math.sin(math.radians(30.))
                errors.append(math.hypot(fix['east_m'] - c * x, fix['north_m'] - s * x))
                self.assertTrue(fix['fix_ok'])
                self.assertAlmostEqual(fix['hacc_m'], 7., delta=1.5)
        median = float(np.median(errors))
        self.assertGreater(median, 2.)
        self.assertLess(median, 8.)

    def test_dropout_jump_and_hacc_window(self):
        spec = dict(dropouts=[[10., 20.]], jumps=[dict(at_s=30., east_m=5., north_m=0., duration_s=.5),
                                                   dict(at_s=40., east_m=0., north_m=15., duration_s=.5)],
                    hacc_windows=[[45., 50., 3.]])
        em, out = run(spec)
        stamps = [t for t, _ in out]
        self.assertFalse([t for t in stamps if 10. <= t < 20.])
        report = em.report()
        self.assertEqual(report['counts']['suppressed'], 10)
        self.assertEqual(report['counts']['jumped'], 2)
        self.assertEqual(report['jump_stamps'], [30.23, 40.23])
        scaled = [fix['hacc_m'] for t, fix in out if 45. <= t < 50.]
        self.assertEqual(len(scaled), 5)
        self.assertTrue(all(h > 15. for h in scaled))
        _, nofix = run(dict(dropouts=[[10., 20.]], dropout_mode='no_fix'))
        self.assertEqual(sum(1 for t, f in nofix if not f['fix_ok']), 10)

    def test_canonical_spec(self):
        self.assertIsNone(G.canonical(None))
        self.assertEqual(G.canonical({'rate_hz': 1., 'hacc_m': 7.}), G.canonical('{"hacc_m":7.0,"rate_hz":1.0}'))
        with self.assertRaises(ValueError):
            G.canonical({'latitude': 1.})
        with self.assertRaises(ValueError):
            G.GnssEmulator({'rate_hz': 3.}, 0)          # 333 ms is not on the 10 ms grid
        with self.assertRaises(ValueError):
            G.GnssEmulator({'dropout_mode': 'off'}, 0)


class RtkModelTest(unittest.TestCase):
    RTK = dict(rate_hz=5., latency_s=.1, rtk=dict(default_type='fixed', windows=[[10., 20., 'float'],
                                                                                [20., 25., 'none'],
                                                                                [25., 30., 'single']]))

    def truth(self, t):
        x = .4 * t
        c, s = math.cos(math.radians(30.)), math.sin(math.radians(30.))
        return c * x, s * x

    def test_types_accuracy_and_errors(self):
        em, out = run(self.RTK, seconds=40.)
        self.assertEqual(len(out), 200)
        kinds = {}
        for t, fix in out:
            kinds.setdefault(fix['fix_type'], []).append((t, fix))
        self.assertEqual({k: len(v) for k, v in kinds.items()}, dict(fixed=100, float=50, none=25, single=25))
        self.assertTrue(all(10. <= t < 20. for t, _ in kinds['float']))
        self.assertTrue(all(not f['fix_ok'] and f['hacc_m'] is None for _, f in kinds['none']))
        for kind, hacc, err_max in (('fixed', .014, .06), ('float', .25, 1.2)):
            errs = [math.hypot(f['east_m'] - self.truth(t)[0], f['north_m'] - self.truth(t)[1]) for t, f in kinds[kind]]
            self.assertLess(max(errs), err_max, kind)
            self.assertTrue(all(abs(f['hacc_m'] - hacc) < 5. * G.RTK_MODELS[kind]['hacc_jitter_m'] + 1e-9
                                for _, f in kinds[kind]), kind)
        self.assertTrue(all(abs(f['hacc_m'] - 7.) < 2. for _, f in kinds['single']))
        counts = em.report()['counts']
        self.assertEqual((counts['rtk_fixed'], counts['rtk_float'], counts['rtk_none'], counts['rtk_single']),
                         (100, 50, 25, 25))
        _, again = run(self.RTK, seconds=40.)
        self.assertEqual(repr(out), repr(again))                 # deterministic ('none' fixes carry NaN)
        self.assertIn('rtk', em.report()['spec'])

    def test_general_receiver_unchanged(self):
        # no RTK key: no fix_type in the packets and the same values as before the RTK model (pinned sample)
        em, out = run({}, seconds=3.)
        self.assertTrue(all('fix_type' not in f for _, f in out))
        self.assertEqual([(t, f['east_m'], f['north_m'], f['hacc_m']) for t, f in out],
                         [(0.23, -0.42, -0.105, 6.85), (1.23, -0.115, -0.24, 6.72), (2.23, -0.446, -0.052, 6.43)])
        self.assertFalse({'rtk', 'frame_mirror', 'frame_scale'} & set(em.report()['spec']))

    def test_mirror_scale_and_spec_checks(self):
        _, mirror = run(dict(self.RTK, rtk=dict(default_type='fixed'), frame_mirror=True), seconds=5.)
        self.assertLess(max(abs(f['east_m'] - self.truth(t)[1]) for t, f in mirror), .06)      # east <- north
        self.assertLess(max(abs(f['north_m'] - self.truth(t)[0]) for t, f in mirror), .06)
        _, scaled = run(dict(self.RTK, rtk=dict(default_type='fixed'), frame_scale=[2., 1.]), seconds=5.)
        self.assertLess(max(abs(f['east_m'] - 2. * self.truth(t)[0]) for t, f in scaled), .06)
        for bad in (dict(rtk=dict(default_type='rtk_fixed')), dict(rtk=dict(windows=[[0., 1., 'x']])),
                    dict(rtk=dict(fixed=dict(sigma=1.))), dict(rtk=dict(colour='red'))):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    G.canonical(bad)


if __name__ == '__main__':
    unittest.main()
