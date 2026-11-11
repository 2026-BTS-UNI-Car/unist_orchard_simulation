"""Generated geometry, not ground-truth inputs to a runtime estimator."""
import math
from types import SimpleNamespace
import unittest

from closed_loop.estimator import ResolvedPathLength, SensorEstimator


class SensorPathGeometry(unittest.TestCase):
    def test_stationary_cloud_is_bounded_instead_of_a_positive_random_walk(self):
        path = ResolvedPathLength(.4, .12)
        old = last = 0.
        previous = (0., 0.)
        for i in range(10000):
            point = (.01 * math.sin(i * .91), .012 * math.cos(i * 1.23))
            old += math.hypot(point[0] - previous[0], point[1] - previous[1]);previous = point
            current = path.update(*point)
            self.assertGreaterEqual(current, last);last = current
        self.assertLess(path.distance, .04)
        self.assertGreater(old, 100.)
        self.assertEqual(path.segments, 0)

    def test_noisy_straight_translation_retains_scale_without_a_fitted_gain(self):
        path = ResolvedPathLength(.4, .12)
        for i in range(401):path.update(i * .005 + .003 * math.sin(i), .006 * math.sin(i * .7))
        self.assertAlmostEqual(path.distance, 2., delta=.03)
        self.assertGreaterEqual(path.segments, 4)

    def test_subsegment_backtracking_commits_extreme_and_counts_both_directions(self):
        path = ResolvedPathLength(.4, .12)
        points = [i * .01 for i in range(31)] + [i * .01 for i in range(29, -1, -1)]
        values = [path.update(x, 0.) for x in points]
        self.assertEqual(sorted(values), values)
        self.assertAlmostEqual(path.distance, .6, places=10)
        self.assertEqual(path.reversals, 1)

    def test_resolved_curve_and_body_rotation_have_separate_meanings(self):
        path = ResolvedPathLength(.4, .12)
        for i in range(201):
            angle = i * .005
            path.update(2 * math.sin(angle), 2 * (1 - math.cos(angle)))
        self.assertAlmostEqual(path.distance, 2., delta=.02)
        pure_pivot = ResolvedPathLength(.4, .12)
        lever = .18
        for i in range(181):
            yaw = math.radians(i)
            # Caller applies its existing nominal laser->base geometry, not
            # a new mount calibration or a wheel/power displacement.
            laser = (lever * math.cos(yaw), lever * math.sin(yaw))
            pure_pivot.update(laser[0] - lever * math.cos(yaw), laser[1] - lever * math.sin(yaw))
        self.assertEqual(pure_pivot.distance, 0.)

    def test_unobserved_gap_never_becomes_a_fabricated_translation(self):
        path = ResolvedPathLength(.4, .12)
        path.update(0., 0.);path.update(.2, 0.)
        path.break_segment();path.update(5., 0.);path.update(5.1, 0.)
        self.assertAlmostEqual(path.distance, .3)
        self.assertEqual(path.gaps, 1)

    def test_invalid_geometry_is_rejected(self):
        for args in ((0., .1), (.4, 0.), (.4, .5), (math.nan, .1)):
            with self.assertRaises(ValueError):ResolvedPathLength(*args)
        path = ResolvedPathLength(.4, .12)
        with self.assertRaises(ValueError):path.update(math.inf, 0.)


class LowSpeedStillness(unittest.TestCase):
    def state(self, speed=0., mode='resolved_polyline', stop_at=None, missing=False):
        history = []
        for i in range(12):
            t = i / 10
            position = speed * t if stop_at is None else speed * min(t, stop_at)
            history.append((t, (position, 0., 0.), not (missing and i == 8)))
        return SimpleNamespace(config=SimpleNamespace(distance_mode=mode,
            still_translation_m=.025, still_rotation_deg=.6), history=history, still_since=None)

    def test_observed_slow_translation_is_not_still_without_relaxing_tolerances(self):
        legacy = self.state(.059, 'scan_norm_legacy')
        resolved = self.state(.059)
        SensorEstimator._update_stillness(legacy, 1.1)
        SensorEstimator._update_stillness(resolved, 1.1)
        self.assertIsNotNone(legacy.still_since)
        self.assertIsNone(resolved.still_since)

    def test_stationary_requires_long_valid_sensor_coverage(self):
        valid = self.state();SensorEstimator._update_stillness(valid, 1.1)
        self.assertIsNotNone(valid.still_since)
        short = self.state();short.history = short.history[-5:]
        SensorEstimator._update_stillness(short, 1.1);self.assertIsNone(short.still_since)
        missing = self.state(missing=True)
        SensorEstimator._update_stillness(missing, 1.1);self.assertIsNone(missing.still_since)

    def test_recently_stopped_translation_does_not_immediately_claim_still(self):
        state = self.state(.09, stop_at=.9)
        SensorEstimator._update_stillness(state, 1.1)
        self.assertIsNone(state.still_since)


if __name__ == '__main__':unittest.main(verbosity=2)
