import math
import unittest

import numpy as np

import rowexec_testlib  # noqa: F401  (path setup)
from row_executor.core import ImuYawRate


def replay_gravity_projected_yaw(t, gyro, acc):
    """Copied from resume-audit-20261013/sim-review/sim2real/replay.py (gravity_projected_yaw)."""
    up = np.empty_like(acc)
    state = acc[0].copy()
    for k in range(len(acc)):
        dt = 0.05 if k == 0 else min(.2, max(1e-3, t[k] - t[k - 1]))
        a = 1 - math.exp(-dt / 1.)
        state += (acc[k] - state) * a
        up[k] = state / np.linalg.norm(state)
    return np.einsum('ij,ij->i', gyro, up)


def tilted(deg):
    """Unit 'up' vector of an IMU whose z axis is tilted deg about its x axis."""
    a = math.radians(deg)
    return np.array((0., math.sin(a), math.cos(a)))


class ImuYawRateTest(unittest.TestCase):
    def test_identical_to_replay_projection(self):
        rng = np.random.default_rng(7)
        n = 600
        t = 100. + np.cumsum(rng.uniform(.02, .04, n))
        t[50] = t[49]                                   # equal receipt times: dt clamps to 1 ms
        up = tilted(5.)
        acc = 9.81 * up + rng.normal(0., 1.5, (n, 3))  # vibration
        gyro = np.outer(rng.normal(0., .2, n), up) + rng.normal(0., .02, (n, 3))
        expected = replay_gravity_projected_yaw(t, gyro, acc)
        imu = ImuYawRate(1., 1)
        got = [imu.update(t[k], gyro[k], acc[k]) for k in range(n)]
        self.assertTrue(all(g is not None for g in got))
        self.assertLess(max(abs(g[0] - e) for g, e in zip(got, expected)), 1e-12)
        self.assertEqual(got[3][1], tuple(acc[3]))      # raw acceleration passed unchanged

    def test_recovers_yaw_about_vertical_for_tilted_imu(self):
        imu = ImuYawRate()
        up = tilted(8.)
        out = None
        for k in range(200):
            out = imu.update(k * .03, .3 * up, 9.81 * up)
        self.assertAlmostEqual(out[0], .3, places=9)
        raw_z_only = .3 * up[2]
        self.assertGreater(abs(out[0] - raw_z_only), 1e-3)   # projection differs from raw gyro z

    def test_sign(self):
        a, b = ImuYawRate(sign=1), ImuYawRate(sign=-1)
        ya = a.update(0., (0., 0., .1), (0., 0., 9.8))[0]
        yb = b.update(0., (0., 0., .1), (0., 0., 9.8))[0]
        self.assertAlmostEqual(ya, .1)
        self.assertAlmostEqual(yb, -.1)
        with self.assertRaises(ValueError):
            ImuYawRate(sign=0)

    def test_rejects_unusable_samples(self):
        imu = ImuYawRate()
        self.assertIsNone(imu.update(0., (float('nan'), 0., 0.), (0., 0., 9.8)))
        self.assertIsNone(imu.update(0., (0., 0., 0.), (0., 0., float('inf'))))
        self.assertIsNone(imu.update(0., (0., 0.), (0., 0., 9.8)))
        self.assertIsNone(imu.update(0., None, (0., 0., 9.8)))
        self.assertIsNone(imu.update(0., (0., 0., 0.), (0., 0., 0.)))           # zero gravity estimate
        imu2 = ImuYawRate()
        self.assertIsNotNone(imu2.update(5., (0., 0., 0.), (0., 0., 9.8)))
        self.assertIsNone(imu2.update(4.9, (0., 0., 0.), (0., 0., 9.8)))        # receipt time regression
        self.assertEqual(imu2.rejected, 1)


if __name__ == '__main__':
    unittest.main()
