"""Independent homogeneous-transform oracle for offline body-motion primitives."""

from dataclasses import replace
import math
import unittest

import numpy as np

try:
    from .body_motion import BodyMotionEstimator, LaserMount, ScanIncrement, motion_error
except ImportError:
    from body_motion import BodyMotionEstimator, LaserMount, ScanIncrement, motion_error


def transform(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, -s, x), (s, c, y), (0., 0., 1.)))


def twist_transform(vx, vy, rate, dt):
    # Independent matrix exponential via its convergent power series; no use
    # of the implementation's closed-form logarithm or lever-arm equations.
    generator = np.array(((0., -rate, vx), (rate, 0., vy), (0., 0., 0.))) * dt
    result = np.eye(3)
    term = np.eye(3)
    for order in range(1, 25):
        term = term @ generator / order
        result += term
    return result


def sample(sequence=0, *, epoch='test-1', velocity=(.2, -.04), rate=.3,
           mount=LaserMount(.18, .02, .2), dt=.1):
    body_delta = twist_transform(*velocity, rate, dt)
    static = transform(mount.x_m, mount.y_m, mount.yaw_rad)
    laser_delta = np.linalg.inv(static) @ body_delta @ static
    interval = round(dt * 1e9)
    end = 1_000_000_000 + sequence * interval
    return ScanIncrement(epoch, sequence, end - interval, end,
                         end + 4_000_000_000, end, end + 4_000_000_000,
                         tuple(float(v) for v in laser_delta[:2, 2]), rate * dt,
                         rate, True, True)


def observe(estimator, increment, **clocks):
    return estimator.observe(increment,
        source_now_ns=clocks.get('source_now_ns', increment.scan_stamp_ns),
        receipt_now_ns=clocks.get('receipt_now_ns', increment.scan_receipt_ns))


class BodyMotionTests(unittest.TestCase):
    def estimator(self, mount=LaserMount(.18, .02, .2)):
        estimator = BodyMotionEstimator(mount)
        estimator.reset('test-1')
        return estimator

    def assert_invalid(self, result):
        self.assertFalse(result.valid, result)
        self.assertIsNone(result.delta_xy_base_m)
        self.assertIsNone(result.linear_x_m_s)
        self.assertIsNone(result.linear_y_m_s)
        self.assertIsNone(result.yaw_rate_rad_s)
        self.assertFalse(result.runtime_authority)

    def test_explicit_mount_and_initialization_are_required(self):
        with self.assertRaises(TypeError):
            LaserMount()
        with self.assertRaises(ValueError):
            BodyMotionEstimator(None)
        estimator = BodyMotionEstimator(LaserMount(.18, 0., 0.))
        self.assert_invalid(observe(estimator, sample()))

    def test_forward_lateral_reverse_and_curved_twists(self):
        for velocity, rate in (((.2, 0.), 0.), ((0., .13), 0.), ((-.2, -.04), -.7),
                               ((.3, -.1), .8), ((.2, .1), 1e-8)):
            with self.subTest(velocity=velocity, rate=rate):
                increment = sample(velocity=velocity, rate=rate)
                result = observe(self.estimator(), increment)
                self.assertTrue(result.valid, result)
                np.testing.assert_allclose((result.linear_x_m_s, result.linear_y_m_s),
                                           velocity, rtol=0., atol=1e-12)
                self.assertEqual(result.yaw_rate_rad_s, rate)
                self.assertEqual(result.scan_stamp_ns, increment.scan_stamp_ns)
                self.assertEqual(result.scan_receipt_ns, increment.scan_receipt_ns)
                self.assertFalse(result.runtime_authority)

    def test_pure_rotation_removes_nonzero_laser_lever_arm(self):
        increment = sample(velocity=(0., 0.), rate=.8)
        self.assertGreater(math.hypot(*increment.delta_xy_laser_m), .01)
        result = observe(self.estimator(), increment)
        self.assertTrue(result.valid, result)
        np.testing.assert_allclose(result.delta_xy_base_m, (0., 0.), rtol=0., atol=1e-12)
        np.testing.assert_allclose((result.linear_x_m_s, result.linear_y_m_s),
                                   (0., 0.), rtol=0., atol=1e-12)

    def test_yaw_rate_is_independent_imu_feedback(self):
        increment = replace(sample(rate=.3), imu_yaw_rate_rad_s=.41)
        self.assertEqual(observe(self.estimator(), increment).yaw_rate_rad_s, .41)

    def test_rejection_and_unsupported_tilt_latch_without_zero_velocity(self):
        for changes in ({'accepted': False}, {'planar_supported': False}, {'accepted': 1},
                        {'delta_xy_laser_m': (math.nan, 0.)}, {'delta_xy_laser_m': [0., 0.]},
                        {'delta_yaw_rad': math.pi}, {'delta_xy_laser_m': (.61, 0.)},
                        {'imu_yaw_rate_rad_s': math.inf}, {'imu_yaw_rate_rad_s': True}):
            with self.subTest(changes=changes):
                estimator = self.estimator()
                result = observe(estimator, replace(sample(), **changes))
                self.assert_invalid(result)
                self.assertEqual(observe(estimator, sample()).reason, result.reason)

    def test_original_source_and_receipt_boundaries(self):
        increment = sample()
        for source_age, receipt_age, valid in (
                (100_000_000, 200_000_000, True), (100_000_001, 0, False),
                (0, 200_000_001, False), (-33_333_333, 0, True),
                (-33_333_334, 0, False), (0, -1, False)):
            with self.subTest(source_age=source_age, receipt_age=receipt_age):
                result = observe(self.estimator(), increment,
                    source_now_ns=increment.scan_stamp_ns + source_age,
                    receipt_now_ns=increment.scan_receipt_ns + receipt_age)
                self.assertEqual(result.valid, valid, result)
        for change in ({'imu_stamp_ns': increment.imu_stamp_ns - 100_000_001},
                       {'imu_receipt_ns': increment.imu_receipt_ns - 200_000_001}):
            self.assert_invalid(observe(self.estimator(), replace(increment, **change)))

    def test_malformed_timestamps_and_intervals(self):
        for field, value in (('sequence', True), ('scan_stamp_ns', 1.),
                             ('imu_receipt_ns', -1), ('scan_receipt_ns', 2**63),
                             ('previous_scan_stamp_ns', 1_000_000_000),
                             ('previous_scan_stamp_ns', 749_999_999)):
            with self.subTest(field=field, value=value):
                self.assert_invalid(observe(self.estimator(), replace(sample(), **{field: value})))

    def test_continuity_repeated_reordered_and_epoch_changes_reject(self):
        variants = (sample(), replace(sample(1), sequence=3),
                    replace(sample(1), previous_scan_stamp_ns=1_000_000_001),
                    replace(sample(1), epoch='other'),
                    replace(sample(1), imu_stamp_ns=sample().imu_stamp_ns))
        for increment in variants:
            with self.subTest(increment=increment):
                estimator = self.estimator()
                self.assertTrue(observe(estimator, sample()).valid)
                self.assert_invalid(observe(estimator, increment))

    def test_explicit_new_epoch_recovery(self):
        estimator = self.estimator()
        observe(estimator, replace(sample(), accepted=False))
        with self.assertRaises(ValueError):
            estimator.reset('test-1')
        estimator.reset('test-2')
        self.assertTrue(observe(estimator, sample(epoch='test-2')).valid)

    def test_consumer_rechecks_age_without_refreshing_translation(self):
        increment = sample()
        result = observe(self.estimator(), increment)
        self.assertEqual(motion_error(result, epoch='test-1',
            source_now_ns=result.scan_stamp_ns + 100_000_001,
            receipt_now_ns=result.scan_receipt_ns), 'stale_scan')

    def test_clock_regression_is_not_a_new_measurement_epoch(self):
        estimator = self.estimator()
        self.assertTrue(observe(estimator, sample(),
                               receipt_now_ns=5_100_000_000).valid)
        result = observe(estimator, replace(sample(1), scan_receipt_ns=5_050_000_000,
                                           imu_receipt_ns=5_050_000_000),
                         receipt_now_ns=5_050_000_000)
        self.assert_invalid(result)
        self.assertEqual(result.reason, 'clock_regression')

    def test_consumer_malformed_epoch_and_velocity_fail_closed(self):
        result = observe(self.estimator(), sample())
        for invalid in (replace(result, epoch=np.array(['bad', 'epoch'])),
                        replace(result, linear_x_m_s=math.nan),
                        replace(result, scan_receipt_ns=True)):
            self.assertIsNotNone(motion_error(invalid, epoch='test-1',
                source_now_ns=1_000_000_000, receipt_now_ns=5_000_000_000))


if __name__ == '__main__':
    unittest.main()
