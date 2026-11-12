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


if __name__ == '__main__':
    unittest.main()
