import math
import unittest

import rowexec_testlib  # noqa: F401  (path setup)
from row_executor.config import ExecutorConfig
from row_executor.guard import Backstop, OutputGuard


class OutputGuardTest(unittest.TestCase):
    def setUp(self):
        self.guard = OutputGuard(75, 40)

    def test_allowed(self):
        for cmd in ((0, 0), (55, 55), (35, 75), (75, 35), (1, 1)):
            with self.subTest(cmd=cmd):
                self.assertEqual(self.guard.apply(*cmd), cmd + (None,))
        self.assertEqual(self.guard.violations, 0)

    def test_violations_become_zero(self):
        cases = {(-55, 55): 'pivot_command', (55, -55): 'pivot_command', (-30, -30): 'reverse_command',
                 (0, -10): 'negative_duty', (-1, 0): 'negative_duty', (76, 70): 'duty_above_cap',
                 (40, 76): 'duty_above_cap', (30, 71): 'differential_above_cap', (55., 55): 'non_integer_duty',
                 (True, 55): 'non_integer_duty', (55, None): 'non_integer_duty',
                 (float('nan'), 0): 'non_integer_duty'}
        for cmd, why in cases.items():
            with self.subTest(cmd=cmd):
                self.assertEqual(self.guard.apply(*cmd), (0, 0, why))
        self.assertEqual(self.guard.violations, len(cases))

    def test_inner_wheel_rule(self):
        guard = OutputGuard(75, 40, inner_min_pct=50)
        self.assertEqual(guard.apply(50, 62), (50, 62, None))
        self.assertEqual(guard.apply(49, 62), (0, 0, 'inner_below_breakaway'))
        self.assertEqual(guard.apply(0, 30), (0, 0, 'inner_below_breakaway'))   # one wheel stopped
        self.assertEqual(guard.apply(0, 0), (0, 0, None))

    def test_from_config(self):
        cfg = ExecutorConfig(breakaway_duty_pct=30)
        guard = OutputGuard.from_config(cfg)
        self.assertEqual((guard.duty_cap_pct, guard.diff_cap_pct, guard.inner_min_pct), (75, 40, 40))
        self.assertIsNone(OutputGuard.from_config(ExecutorConfig(breakaway_duty_pct=30, inner_rule_enabled=False))
                          .inner_min_pct)
        with self.assertRaises(ValueError):
            OutputGuard(101, 40)


class BackstopTest(unittest.TestCase):
    def test_start_validation(self):
        b = Backstop()
        for row_len, v in ((2.5, .4), (float('nan'), .4), (400., .4), (True, .4), (20., 0.), (20., -1.),
                           (20., float('inf'))):
            with self.subTest(row_len=row_len, v=v), self.assertRaises(ValueError):
                b.start(0., 0., row_len, v)
        self.assertFalse(b.active)
        b.start(0., 0., 3., .4)       # 1 m run is the minimum
        self.assertEqual(b.limit_m, 1.)

    def test_distance_rule_l_minus_2(self):
        b = Backstop()
        b.start(10., 5., 20., None)          # estimator distance 5 at start
        self.assertIsNone(b.time_limit_s)
        self.assertIsNone(b.check(11., 5. + 17.99, True))
        self.assertEqual(b.check(12., 5. + 18., True), Backstop.DISTANCE)
        self.assertEqual(b.check(13., 0., False), Backstop.DISTANCE)     # latched

    def test_time_rule_counts_motion_time_only(self):
        b = Backstop()
        b.start(0., 0., 20., .5)             # 1.15 * 18 / 0.5 = 41.4 s
        self.assertAlmostEqual(b.time_limit_s, 41.4)
        t = 0.
        for _ in range(400):                 # 20 s moving
            t += .05
            self.assertIsNone(b.check(t, 1., True))
        for _ in range(2000):                # 100 s paused and still: no accumulation
            t += .05
            self.assertIsNone(b.check(t, 1., False))
        self.assertAlmostEqual(b.motion_time_s, 20.05, places=6)   # + the first still tick after motion
        reason = None
        while reason is None:
            t += .05
            reason = b.check(t, 1., True)
        self.assertEqual(reason, Backstop.TIME)
        self.assertGreaterEqual(b.motion_time_s, 41.4)
        self.assertLess(b.motion_time_s, 41.4 + .051)

    def test_odometry_invalid_aborts(self):
        for bad in (float('nan'), float('inf'), None):
            b = Backstop()
            b.start(0., 0., 20., .5)
            with self.subTest(bad=bad):
                self.assertEqual(b.check(1., bad, True), Backstop.ODOMETRY)
        b = Backstop()
        b.start(0., 0., 20., .5)
        self.assertIsNone(b.check(1., 3., True))
        self.assertIsNone(b.check(2., 2.995, True))       # jitter below 1 cm tolerated
        self.assertEqual(b.check(3., 2.9, True), Backstop.ODOMETRY)

    def test_abort_latches_first_reason(self):
        b = Backstop()
        b.start(0., 0., 20., .5)
        self.assertEqual(b.abort('operator'), 'operator')
        self.assertEqual(b.check(1., 19., True), 'operator')
        self.assertTrue(math.isfinite(b.summary()['limit_m']))


if __name__ == '__main__':
    unittest.main()
