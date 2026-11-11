"""Sensor timing, full-pose and range contracts independent of ROS installation."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cpu_lidar import ScanSynthesizer, quaternion_rotation


class ConstantScanner:
    def __init__(self, ranges):
        self.ranges = np.asarray(ranges, dtype=float)
        self.calls = []

    def scan(self, position, *, rotation):
        self.calls.append((np.asarray(position), rotation))
        count = len(self.ranges)
        a = -math.pi+np.arange(count)*2*math.pi/count
        local = np.column_stack((np.cos(a), np.sin(a), np.zeros(count)))
        return self.ranges.copy(), np.asarray(position)+rotation@np.array([.18, 0., .6]), local@rotation.T


def sensor(ranges=None, **kwargs):
    ranges = [2.]*3200 if ranges is None else ranges
    caster = ConstantScanner(ranges)
    return ScanSynthesizer(caster, count=len(ranges), noise_stddev=0., **kwargs), caster


def sample(synth, ns, xyz=(0., 0., 0.), quaternion=(0., 0., 0., 1.)):
    return synth.sample(ns, xyz, quaternion)


def test_3200_unique_bearings_full_circle_and_rigid_sensor_height():
    synth, caster = sensor()
    frame = sample(synth, 123_000_000, (2., 3., -.05))
    assert frame.stamp_ns == 123_000_000
    assert len(frame.ranges) == 3200
    assert np.allclose(frame.origin, (2.18, 3., .55))
    assert np.allclose(frame.directions[1600], (1., 0., 0.), atol=1e-12)
    assert synth.angle_max < math.pi
    assert synth.angle_increment*3200 == pytest.approx(2*math.pi)
    assert not np.allclose(frame.directions[0], frame.directions[-1])


def test_full_pitch_rotates_mount_and_scan_plane_instead_of_flattening_pose():
    synth, _ = sensor()
    pitch = math.pi/6
    q = (0., math.sin(pitch/2), 0., math.cos(pitch/2))
    frame = sample(synth, 0, (1., 2., 3.), q)
    rotation = quaternion_rotation(q)
    assert np.allclose(frame.origin, np.array([1., 2., 3.])+rotation@np.array([.18, 0., .6]))
    assert np.allclose(frame.directions[1600], (math.cos(pitch), 0., -math.sin(pitch)))
    assert np.allclose(rotation.T@rotation, np.eye(3))
    assert np.linalg.det(rotation) == pytest.approx(1.)


def test_10hz_gating_uses_original_pose_stamp_without_duplicate_or_catchup():
    synth, caster = sensor()
    timestamps = [round(i*1_000_000_000/30) for i in range(31)]
    published = [frame.stamp_ns for ns in timestamps if (frame := sample(synth, ns)) is not None]
    assert published == [i*100_000_000 for i in range(11)]
    assert sample(synth, 1_000_000_000) is None
    assert synth.duplicates == 1 and len(caster.calls) == 11
    delayed = sample(synth, 3_533_000_000)
    assert delayed.stamp_ns == 3_533_000_000
    assert sample(synth, 3_550_000_000) is None
    assert len(caster.calls) == 12


def test_clock_reset_starts_new_scan_phase_and_never_reuses_old_frame():
    synth, _ = sensor()
    assert sample(synth, 8_000_000_000).stamp_ns == 8_000_000_000
    reset = sample(synth, 0)
    assert reset.stamp_ns == 0 and synth.clock_resets == 1
    assert sample(synth, 0) is None
    assert sample(synth, 33_000_000) is None
    assert sample(synth, 100_000_000).stamp_ns == 100_000_000


def test_near_hits_stay_blocked_and_no_returns_stay_infinite():
    synth, _ = sensor([math.inf, .01, .05, 2., 30., 31.])
    ranges = sample(synth, 0).ranges
    assert np.isposinf(ranges[[0, 5]]).all()
    assert np.allclose(ranges[[1, 2, 3, 4]], [.05, .05, 2., 30.])


def test_noise_underflow_does_not_turn_a_near_obstacle_into_free_space():
    synth = ScanSynthesizer(ConstantScanner([.001, .05, math.inf]), count=3,
                            noise_stddev=.01)

    class NegativeNoise:
        def normal(self, mean, deviation, size):
            return np.full(size, -.1)

    synth.rng = NegativeNoise()
    ranges = sample(synth, 0).ranges
    assert np.allclose(ranges[:2], [.05, .05])
    assert np.isposinf(ranges[2])


def test_negative_raycast_distance_rejects_the_sample():
    synth, _ = sensor([-.001, 2.])
    with pytest.raises(ValueError, match='raycast_output'):
        sample(synth, 0)
    assert synth.published == 0


def test_assumed_noise_applies_to_hits_only_and_is_reproducible():
    values = [math.inf]+[2.]*3199
    a = ScanSynthesizer(ConstantScanner(values), noise_stddev=.01, seed=17)
    b = ScanSynthesizer(ConstantScanner(values), noise_stddev=.01, seed=17)
    first, second = sample(a, 0).ranges, sample(b, 0).ranges
    assert np.array_equal(first, second)
    assert math.isinf(first[0])
    assert .008 < float(np.std(first[1:]-2.)) < .012
    assert np.all(np.isfinite(first[1:]))


@pytest.mark.parametrize('quaternion', [(0, 0, 0, 0), (0, 0, 0, 2), (0, math.nan, 0, 1)])
def test_invalid_orientation_never_reaches_raycast(quaternion):
    synth, caster = sensor()
    with pytest.raises(ValueError, match='quaternion'):
        sample(synth, 0, quaternion=quaternion)
    assert not caster.calls and synth.published == 0


def test_nan_backend_output_is_rejected_instead_of_inventing_a_scan():
    synth, _ = sensor([math.nan, 2.])
    with pytest.raises(ValueError, match='raycast_output'):
        sample(synth, 0)
    assert synth.published == 0
