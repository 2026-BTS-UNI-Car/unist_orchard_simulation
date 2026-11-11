"""Portable synthetic native-message fixtures; no private archive required."""

from dataclasses import replace
import math
import unittest

try:
    from .sensor_contract import (LaserDriverProfile, MAX_RAW_BEAMS, MAX_STAMP_NS,
                                  read_imu, read_laser_scan)
except ImportError:
    from sensor_contract import (LaserDriverProfile, MAX_RAW_BEAMS, MAX_STAMP_NS,
                                 read_imu, read_laser_scan)


def profile(**changes):
    return replace(LaserDriverProfile('synthetic-explicit-order', 'laser', 3240,
                                     1, 'unknown', 'first_acquired_ray'), **changes)


def scan(count=32, *, rolling=False):
    # Large native timestamps exceed float's exact-integer range deliberately.
    stamp = 1_800_000_000_123_456_789
    increment = 30.5853e-6 if rolling else 0.
    return dict(topic='/scan', stamp_ns=stamp, log_ns=stamp + 101_000_007,
                frame_id='laser', cdr_sha256='a' * 64,
                angle_min=-math.pi, angle_max=math.pi,
                angle_increment=2 * math.pi / (count - 1),
                time_increment=increment, scan_time=.1,
                range_min=.05, range_max=30., ranges=[2.] * count,
                intensities=[47.] * count)


def imu():
    return dict(topic='/imu/data_raw', stamp_ns=1_800_000_000_123_456_789,
                log_ns=1_800_000_000_124_456_790, frame_id='imu_link',
                cdr_sha256='b' * 64, quaternion=[0., 0., 0., 1.],
                orientation_covariance=[0.] * 9,
                angular_velocity=[.1, .2, .3], angular_velocity_covariance=[0.] * 9,
                linear_acceleration=[0., 0., 9.81], linear_acceleration_covariance=[0.] * 9)


def read_scan(packet=None, driver=None, *, window='window-a'):
    return read_laser_scan(scan() if packet is None else packet,
                           'synthetic-archive', window, driver)


def read_inertial(packet=None, *, window='window-a'):
    return read_imu(imu() if packet is None else packet, 'synthetic-archive', window)


class LaserContractTests(unittest.TestCase):
    def test_native_3240_preserved_separate_from_matcher_limit(self):
        packet = scan(3240, rolling=True)
        result = read_scan(packet, profile())
        self.assertTrue(result.valid)
        self.assertEqual(result.raw.ranges, tuple(packet['ranges']))
        self.assertEqual(len(result.raw.ranges), 3240)
        self.assertEqual(len(result.raw.intensities), 3240)
        self.assertEqual(result.matching_reasons,
                         ('rolling_deskew_unavailable', 'matcher_point_limit_exceeded'))
        self.assertFalse(result.matching_compatible)
        self.assertFalse(result.runtime_authority)

    def test_missing_profile_keeps_raw_diagnostics_without_invented_ray_order(self):
        packet = scan(3240, rolling=True)
        result = read_scan(packet)
        self.assertTrue(result.valid)
        self.assertEqual(result.raw.time_increment, packet['time_increment'])
        self.assertIsNone(result.ray_stamp_ns)
        self.assertIn('missing_driver_profile', result.matching_reasons)
        self.assertFalse(result.matching_compatible)

    def test_integer_header_log_and_per_ray_offsets_round_once(self):
        packet = scan(32, rolling=True)
        packet['time_increment'] = 1.5e-9
        result = read_scan(packet, profile())
        start = packet['stamp_ns']
        self.assertEqual(result.raw.identity.stamp_ns, start)
        self.assertEqual(result.raw.identity.log_ns, packet['log_ns'])
        self.assertIs(type(result.raw.identity.stamp_ns), int)
        self.assertEqual(result.ray_stamp_ns[:5], (start, start + 2, start + 3,
                                                start + 4, start + 6))
        self.assertEqual(result.ray_stamp_ns[-1], start + 46)
        self.assertIn('rolling_deskew_unavailable', result.matching_reasons)

    def test_reverse_array_direction_header_is_first_acquired_ray(self):
        packet = scan(32, rolling=True)
        forward = read_scan(packet, profile())
        reverse = read_scan(packet, profile(array_time_direction=-1))
        self.assertEqual(reverse.ray_stamp_ns, forward.ray_stamp_ns[::-1])
        self.assertEqual(reverse.ray_stamp_ns[-1], packet['stamp_ns'])
        self.assertTrue(all(a > b for a, b in zip(reverse.ray_stamp_ns,
                                                reverse.ray_stamp_ns[1:])))
        self.assertEqual(reverse.raw.ranges, forward.raw.ranges)

    def test_range_masks_do_not_clamp_or_claim_free_space(self):
        packet = scan()
        values = [.05, 2., 30., 54.8, .01, None, math.inf, -math.inf, math.nan]
        packet['ranges'][:len(values)] = values
        result = read_scan(packet, profile())
        self.assertTrue(result.valid)
        self.assertEqual(result.raw.ranges[3], 54.8)
        self.assertEqual(result.raw.ranges[4], .01)
        self.assertIsNone(result.raw.ranges[5])
        self.assertTrue(math.isnan(result.raw.ranges[8]))
        masks = result.masks
        self.assertEqual(masks.finite_return[:9], (True, True, False, False,
                                                 False, False, False, False, False))
        self.assertEqual(tuple(i for i, v in enumerate(masks.above_max) if v), (3,))
        self.assertEqual(tuple(i for i, v in enumerate(masks.below_min) if v), (4,))
        self.assertEqual(tuple(i for i, v in enumerate(masks.at_range_max) if v), (2,))
        self.assertEqual(masks.no_return[:9], (False, False, False, False,
                                             False, True, True, True, True))
        declared = read_scan(packet, profile(range_max_semantics='exclusive_no_return'))
        self.assertTrue(declared.masks.no_return[2])
        self.assertEqual(declared.masks.at_range_max, masks.at_range_max)
        self.assertFalse(hasattr(result, 'free_space'))

    def test_instantaneous_contract_counts_usable_points_without_truncation(self):
        packet = scan(3240)
        packet['ranges'][-40:] = [None] * 40
        result = read_scan(packet, profile())
        self.assertTrue(result.matching_compatible)
        self.assertEqual(len(result.raw.ranges), 3240)
        self.assertEqual(sum(result.masks.finite_return), 3200)
        self.assertEqual(set(result.ray_stamp_ns), {packet['stamp_ns']})
        packet['ranges'][-1] = 1.
        self.assertIn('matcher_point_limit_exceeded', read_scan(packet, profile()).matching_reasons)

    def test_global_limit_rejects_without_truncation_and_profile_limit_is_separate(self):
        result = read_scan(scan(MAX_RAW_BEAMS + 1), profile())
        self.assertFalse(result.valid)
        self.assertIsNone(result.raw)
        small_profile = read_scan(scan(3240), profile(max_raw_beams=3200))
        self.assertTrue(small_profile.valid)
        self.assertEqual(len(small_profile.raw.ranges), 3240)
        self.assertIn('profile_beam_limit_exceeded', small_profile.matching_reasons)
        self.assertIsNone(small_profile.ray_stamp_ns)

    def test_profile_frame_and_profile_configuration_are_not_guessed(self):
        mismatch = read_scan(driver=profile(frame_id='another_laser'))
        self.assertTrue(mismatch.valid)
        self.assertIn('profile_frame_mismatch', mismatch.matching_reasons)
        self.assertIsNone(mismatch.ray_stamp_ns)
        for changes in ({'max_raw_beams': 8193}, {'max_raw_beams': True},
                        {'array_time_direction': 0}, {'array_time_direction': True},
                        {'header_reference': 'last_ray'}, {'range_max_semantics': 'free_space'},
                        {'frame_id': ''}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                profile(**changes)

    def test_invalid_geometry_arrays_and_booleans_are_rejected(self):
        for field, value in (('range_min', True), ('range_min', -1), ('angle_increment', 0.),
                             ('angle_max', 1.), ('time_increment', -.1),
                             ('time_increment', .1), ('scan_time', math.nan),
                             ('intensities', [47.]), ('ranges', [False] * 32),
                             ('ranges', ['2'] * 32)):
            packet = scan()
            packet[field] = value
            with self.subTest(field=field, value=str(value)[:30]):
                result = read_scan(packet, profile())
                self.assertFalse(result.valid)
                self.assertFalse(result.matching_compatible)

    def test_timestamp_overflow_and_unresolvable_timing_keep_raw_only(self):
        packet = scan(32, rolling=True)
        packet['stamp_ns'] = MAX_STAMP_NS
        result = read_scan(packet, profile())
        self.assertTrue(result.valid)
        self.assertIsNone(result.ray_stamp_ns)
        self.assertIn('ray_timestamp_overflow', result.matching_reasons)
        packet['stamp_ns'] = 1
        packet['time_increment'] = 1e-12
        result = read_scan(packet, profile())
        self.assertTrue(result.valid)
        self.assertIsNone(result.ray_stamp_ns)
        self.assertIn('ray_timing_not_resolvable_ns', result.matching_reasons)

    def test_extreme_declared_duration_cannot_overflow_float_to_integer_conversion(self):
        packet = scan()
        packet['time_increment'] = 1e300
        packet['scan_time'] = 1e302
        result = read_scan(packet, profile())
        self.assertTrue(result.valid)
        self.assertIsNone(result.ray_stamp_ns)
        self.assertIn('ray_timestamp_overflow', result.matching_reasons)

    def test_angular_coordinate_direction_does_not_choose_acquisition_direction(self):
        packet = scan(32, rolling=True)
        packet['angle_min'], packet['angle_max'] = math.pi, -math.pi
        packet['angle_increment'] *= -1
        result = read_scan(packet, profile())
        self.assertTrue(result.valid)
        self.assertEqual(result.ray_stamp_ns[0], packet['stamp_ns'])
        self.assertLess(result.ray_stamp_ns[0], result.ray_stamp_ns[-1])

    def test_bounded_arrays_are_snapshots_not_mutable_packet_aliases(self):
        packet = scan()
        result = read_scan(packet, profile())
        packet['ranges'][0] = 99.
        packet['intensities'][0] = 99.
        self.assertEqual(result.raw.ranges[0], 2.)
        self.assertEqual(result.raw.intensities[0], 47.)

    def test_empty_intensity_and_nonfinite_intensity_are_diagnostic(self):
        packet = scan()
        packet['intensities'] = []
        self.assertEqual(read_scan(packet).raw.intensities, ())
        packet['intensities'] = [None, math.nan] + [47.] * 30
        result = read_scan(packet)
        self.assertTrue(result.valid)
        self.assertIsNone(result.raw.intensities[0])
        self.assertTrue(math.isnan(result.raw.intensities[1]))


class IdentityTests(unittest.TestCase):
    def test_overlap_occurrence_identity_does_not_deduplicate_or_retime(self):
        for reader, packet in ((read_scan, scan()), (read_inertial, imu())):
            first = reader(packet, window='first')
            second = reader(packet, window='overlap')
            a = first.raw.identity if hasattr(first, 'raw') else first.identity
            b = second.raw.identity if hasattr(second, 'raw') else second.identity
            self.assertEqual(a.message_key, b.message_key)
            self.assertNotEqual(a.occurrence_key, b.occurrence_key)
            self.assertEqual(a.stamp_ns, b.stamp_ns)
            self.assertEqual(a.log_ns, b.log_ns)
            self.assertNotEqual(a.stamp_ns, a.log_ns)

    def test_integer_timestamps_never_accept_bool_float_or_rewritten_clock(self):
        for factory, reader in ((scan, read_scan), (imu, read_inertial)):
            for field in ('stamp_ns', 'log_ns'):
                for value in (True, 1., -1, MAX_STAMP_NS + 1, None):
                    packet = factory()
                    packet[field] = value
                    with self.subTest(field=field, value=value):
                        self.assertFalse(reader(packet).valid)
            packet = factory()
            packet['log_ns'] = packet['stamp_ns'] - 1000
            result = reader(packet)
            self.assertTrue(result.valid)  # No invented common-clock ordering.


class ImuContractTests(unittest.TestCase):
    def test_all_zero_covariance_is_unknown_not_zero_noise(self):
        result = read_inertial()
        self.assertTrue(result.valid)
        self.assertFalse(result.runtime_authority)
        for group in (result.orientation, result.angular_velocity, result.linear_acceleration):
            self.assertTrue(group.valid)
            self.assertTrue(group.available)
            self.assertTrue(group.uncertainty_unknown)
            self.assertEqual(group.reason, 'unknown_uncertainty')

    def test_historical_minus_one_contract_overrides_plausible_and_nonfinite_values(self):
        packet = imu()
        for key in ('orientation_covariance', 'angular_velocity_covariance',
                    'linear_acceleration_covariance'):
            packet[key] = [-1.] + [0.] * 8
        packet['quaternion'] = [None] * 4
        packet['angular_velocity'] = [math.nan] * 3
        result = read_inertial(packet)
        self.assertTrue(result.valid)
        for group in (result.orientation, result.angular_velocity, result.linear_acceleration):
            self.assertTrue(group.valid)
            self.assertFalse(group.available)
            self.assertTrue(group.uncertainty_unknown)
            self.assertEqual(group.reason, 'unavailable_minus_one')
            self.assertEqual(group.covariance[0], -1.)
        self.assertEqual(result.orientation.values, (None,) * 4)
        self.assertEqual(result.linear_acceleration.values, (0., 0., 9.81))

    def test_availability_is_independent_for_each_group(self):
        packet = imu()
        packet['orientation_covariance'][0] = -1
        packet['angular_velocity_covariance'] = [.01, 0., 0., 0., .02, 0., 0., 0., .03]
        result = read_inertial(packet)
        self.assertTrue(result.valid)
        self.assertFalse(result.orientation.available)
        self.assertTrue(result.angular_velocity.available)
        self.assertFalse(result.angular_velocity.uncertainty_unknown)
        self.assertTrue(result.linear_acceleration.uncertainty_unknown)

    def test_available_orientation_must_be_finite_unit_quaternion_without_normalizing(self):
        for values, reason in (([0., 0., 0., 2.], 'nonunit_quaternion'),
                               ([0., 0., 0., 0.], 'nonunit_quaternion'),
                               ([None, 0., 0., 1.], 'nonfinite_values'),
                               ([math.inf, 0., 0., 1.], 'nonfinite_values')):
            packet = imu()
            packet['quaternion'] = values
            result = read_inertial(packet)
            self.assertFalse(result.valid)
            self.assertEqual(result.orientation.reason, reason)
            self.assertEqual(result.orientation.values, tuple(values))
            self.assertTrue(result.angular_velocity.available)
            self.assertTrue(result.linear_acceleration.available)

    def test_reported_covariance_requires_finite_symmetric_psd_nonnegative_variance(self):
        fixtures = (([1., 0., 0., 0., -1., 0., 0., 0., 1.], 'negative_variance'),
                    ([1., .5, 0., 0., 1., 0., 0., 0., 1.], 'asymmetric_covariance'),
                    ([1., 2., 0., 2., 1., 0., 0., 0., 1.], 'non_psd_covariance'),
                    ([1., .9, .9, .9, 1., -.9, .9, -.9, 1.], 'non_psd_covariance'),
                    ([1., None, 0., 0., 1., 0., 0., 0., 1.], 'nonfinite_covariance'))
        for covariance, reason in fixtures:
            packet = imu()
            packet['angular_velocity_covariance'] = covariance
            result = read_inertial(packet)
            with self.subTest(reason=reason):
                self.assertFalse(result.valid)
                self.assertFalse(result.angular_velocity.available)
                self.assertEqual(result.angular_velocity.reason, reason)
                self.assertTrue(result.orientation.available)
        packet = imu()
        # Singular PSD covariance is valid reported uncertainty, not all unknown.
        packet['angular_velocity_covariance'] = [1., 1., 0., 1., 1., 0., 0., 0., 0.]
        self.assertEqual(read_inertial(packet).angular_velocity.reason, 'reported_covariance')

    def test_malformed_group_does_not_erase_other_groups_and_bool_is_not_numeric(self):
        for field, value in (('quaternion', [False, 0., 0., 1.]),
                             ('orientation_covariance', [True] + [0.] * 8),
                             ('quaternion', [0., 1.]), ('orientation_covariance', [-1.])):
            packet = imu()
            packet[field] = value
            result = read_inertial(packet)
            self.assertFalse(result.valid)
            self.assertFalse(result.orientation.valid)
            self.assertTrue(result.angular_velocity.valid)
            self.assertTrue(result.linear_acceleration.valid)

    def test_small_indefinite_covariance_is_not_hidden_by_tiny_principal_minors(self):
        packet = imu()
        covariance = [1., 0., 0., 0., 0., 1e-7, 0., 1e-7, 0.]
        packet['angular_velocity_covariance'] = covariance
        result = read_inertial(packet)
        # The lower 2x2 block independently has eigenvectors (1,1), (1,-1)
        # with eigenvalues +1e-7 and -1e-7 despite its determinant being -1e-14.
        self.assertFalse(result.angular_velocity.available)
        self.assertEqual(result.angular_velocity.reason, 'non_psd_covariance')
        self.assertEqual(result.angular_velocity.covariance, tuple(covariance))


if __name__ == '__main__':
    unittest.main()
