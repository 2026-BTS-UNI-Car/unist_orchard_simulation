"""steering.p6a_imu_model.GameRotationVector (gsim-final3): the BNO085-like game rotation vector of the G-SIM IMU,
from the modelled gyro and accel only (the dashboard's live map needs an orientation). No ROS."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from steering import p6a_imu_model as M  # noqa: E402

G = M.GRAVITY_M_S2


def tilted(pitch_deg, roll_deg=0.):
    """The accelerometer at rest (gravity reaction, z up) of a sensor pitched / rolled by these angles."""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    return (-G * math.sin(p), G * math.cos(p) * math.sin(r), G * math.cos(p) * math.cos(r))


class GameRotationVectorTest(unittest.TestCase):
    def test_first_sample_takes_the_tilt_from_the_accelerometer(self):
        o = M.GameRotationVector()
        q = o.update((0., 0., 0.), tilted(5., -3.), 10.)
        self.assertAlmostEqual(sum(v * v for v in q), 1., places=12)
        roll, pitch, yaw = (math.degrees(a) for a in o.rpy())
        self.assertAlmostEqual(pitch, 5., places=6)
        self.assertAlmostEqual(roll, -3., places=6)
        self.assertAlmostEqual(yaw, 0., places=9)

    def test_tilt_follows_the_accelerometer_with_the_time_constant(self):
        o = M.GameRotationVector()
        o.update((0., 0., 0.), tilted(0.), 0.)
        t = 0.
        for _ in range(int(20. / .03)):          # 33 Hz for 20 s, the sensor now pitched 4 deg, gyro at rest
            t += .03
            o.update((0., 0., 0.), tilted(4.), t)
        self.assertAlmostEqual(math.degrees(o.rpy()[1]), 4., delta=.05)
        self.assertEqual(o.corrected, int(20. / .03))

    def test_heading_integrates_the_gyro_and_is_not_corrected(self):
        o = M.GameRotationVector()
        o.update((0., 0., 0.), tilted(0.), 0.)
        t = 0.
        for _ in range(100):                      # 0.2 rad/s about z for 3 s
            t += .03
            o.update((0., 0., .2), tilted(0.), t)
        self.assertAlmostEqual(o.rpy()[2], .6, places=3)
        self.assertAlmostEqual(o.rpy()[0], 0., places=6)

    def test_no_tilt_correction_under_large_acceleration_and_gaps_are_not_integrated(self):
        o = M.GameRotationVector()
        o.update((0., 0., 0.), tilted(0.), 0.)
        o.update((0., 0., 0.), (5., 0., G), .03)  # | a | 11 % above g: corrected (gate 20 %)
        o.update((0., 0., 0.), (9., 0., G), .06)  # 35 % above g: not corrected
        self.assertEqual(o.corrected, 1)
        o.update((0., 0., 5.), tilted(0.), 2.)    # a 1.94 s gap: not integrated
        self.assertEqual(o.skipped_gaps, 1)
        self.assertAlmostEqual(o.rpy()[2], 0., places=9)

    def test_covariance_is_the_placeholder_diagonal(self):
        cov = M.GameRotationVector.covariance()
        self.assertEqual(len(cov), 9)
        self.assertGreater(cov[0], 0.)            # orientation provided (REP 145: -1 would mean not provided)
        self.assertEqual([cov[1], cov[2], cov[3], cov[5], cov[6], cov[7]], [0.] * 6)
        self.assertAlmostEqual(cov[8], math.radians(3.) ** 2)


if __name__ == '__main__':
    unittest.main()
