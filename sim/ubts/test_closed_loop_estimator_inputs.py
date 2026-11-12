"""Design v2 opt-in outputs / inputs of the estimator: the corridor quality block and the external corridor observation
slot (closed_loop/estimator.py; the actuator priors have their own file test_closed_loop_actuator_prior.py).

Synthetic only: ray-cast scans of a two-row corridor from a scripted pose and a zero gyro. No plant and no SIM truth
enters an estimator. OFF (the defaults) must leave the estimate untouched: the same snapshot keys and values.
"""
import math

import numpy as np
import pytest

from closed_loop.estimator import (EXTERNAL_REASONS, QUALITY_GAIN, CorridorQuality, EstimatorConfig, SensorEstimator,
                                   wrap)
from orchard_perception.scenarios import Circle, OrchardScene, Pose2D, raycast_scan

XS = np.arange(-6., 60.1, 1.5)
BOTH = OrchardScene(tuple(Circle(float(x), y, .13) for y in (-1.75, 1.75) for x in XS), ())
LEFT_ONLY = OrchardScene(tuple(Circle(float(x), 1.75, .13) for x in XS), ())
RIGHT_ONLY = OrchardScene(tuple(Circle(float(x), -1.75, .13) for x in XS), ())
EMPTY = OrchardScene((), ())
SPEED = .4


def scene_at(t, plan):
    for until, scene in plan:
        if t < until:
            return scene
    return plan[-1][1]


def drive(estimators, seconds, plan=((math.inf, BOTH),), y=0., yaw=0., hook=None, start=0.):
    """Scripted straight drive at SPEED along +x, offset y, heading yaw: 10 Hz scans of the scene the plan gives for the
    time, 20 Hz gyro 0. hook(t, estimators) runs at 20 Hz after the packets of that time."""
    steps = int(round(seconds / .05))
    for k in range(steps + 1):
        t = round(start + k * .05, 6)
        for est in estimators:
            est.on_imu(t, 0., (0., 0., 9.81))
        if k % 2 == 0:
            s = raycast_scan(scene_at(t, plan), Pose2D(SPEED * t, y, yaw))
            for est in estimators:
                est.on_scan(t, np.asarray(s.ranges), s.angle_min, s.angle_increment, s.range_max)
        if hook is not None:
            hook(t, estimators)


def same(a, b):
    return a == b or (a != a and b != b)


# --- corridor quality -----------------------------------------------------------------------------------------------

def test_quality_block_is_absent_by_default():
    est = SensorEstimator()
    drive([est], 3.)
    assert est.quality is None and 'corridor_quality' not in est.snapshot(3.)


def test_quality_reports_both_sides_support_ages_and_spread():
    est = SensorEstimator(EstimatorConfig(corridor_quality=True))
    assert est.snapshot(0.)['corridor_quality']['sides'] == 'none'
    drive([est], 6.)
    s = est.snapshot(6.05)
    q = s['corridor_quality']
    assert s['corridor_valid'] and q['sides'] == 'both' and q['source'] in ('candidate', 'guided')
    assert q['support_left'] >= 8 and q['support_right'] >= 8                # line_min_points on each side
    assert q['age_s'] == pytest.approx(s['corridor_age_s']) and q['full_age_s'] == q['age_s']
    assert q['left_age_s'] == q['right_age_s'] == q['age_s'] and 0. < q['age_s'] < .4
    assert q['one_sided_run'] == 0 and q['rejected_run'] == 0
    assert 0. <= q['lateral_sd_m'] < .1 and 0. <= q['heading_sd_rad'] < math.radians(2.)
    assert 0. <= q['width_sd_m'] < .2


def test_quality_follows_a_one_sided_view_and_a_lost_band():
    est = SensorEstimator(EstimatorConfig(corridor_quality=True))
    seen = {}

    def note(t, estimators):
        if t in (5.95, 8.95, 12.95, 14.95, 19.95):
            seen[t] = estimators[0].snapshot(t)
    drive([est], 20., plan=((6., BOTH), (9., LEFT_ONLY), (13., RIGHT_ONLY), (15., BOTH), (math.inf, EMPTY)), hook=note)
    assert seen[5.95]['corridor_quality']['sides'] == 'both'
    left = seen[8.95]['corridor_quality']
    assert seen[8.95]['corridor_valid'] and left['sides'] == 'left' and left['one_sided_run'] >= 3
    assert left['support_left'] >= 8 and left['support_right'] == 0
    assert left['left_age_s'] < .4 < left['right_age_s'] and left['full_age_s'] > 2.5
    right = seen[12.95]['corridor_quality']
    assert right['sides'] == 'right' and right['support_right'] >= 8 and right['support_left'] == 0
    assert right['right_age_s'] < .4 < right['left_age_s'] and right['one_sided_run'] > left['one_sided_run']
    again = seen[14.95]['corridor_quality']
    assert again['sides'] == 'both' and again['one_sided_run'] == 0 and again['full_age_s'] < .4
    gone = seen[19.95]
    assert not gone['corridor_valid'] and gone['corridor_quality']['sides'] == 'none'
    assert gone['corridor_quality']['support_left'] == 0


def test_quality_counts_refused_updates():
    est = SensorEstimator(EstimatorConfig(corridor_quality=True))
    drive([est], 5.)
    assert est.quality.rejected_run == 0
    drive([est], 1., plan=((math.inf, EMPTY),), start=5.05)                   # no returns near the tracked lines
    assert est.quality.rejected_run >= 3 and est.corridor_reason == 'no_support_near_tracked_lines'
    drive([est], 1., start=6.1)
    assert est.quality.rejected_run == 0 and est.quality.sides == 'both'


def test_quality_bookkeeping_is_an_ema_of_squared_innovations():
    q = CorridorQuality()
    q.accepted(1., 'both', 'candidate', (10, 12), lateral=.1, heading=.02, width=-.2)
    assert q.variance == dict(lateral=pytest.approx(.01), heading=pytest.approx(.0004), width=pytest.approx(.04))
    q.accepted(1.2, 'left', 'guided', (9, 0), lateral=0., heading=0.)
    assert q.variance['lateral'] == pytest.approx(.01 * (1 - QUALITY_GAIN))
    assert q.variance['width'] == pytest.approx(.04)                          # a one-sided view has no width
    assert (q.sides, q.one_sided_run, q.side_stamp, q.full_stamp) == ('left', 1, [1.2, 1.], 1.)
    out = q.snapshot(1.5, band=object())
    assert out['lateral_sd_m'] == pytest.approx(math.sqrt(.008)) and out['left_age_s'] == pytest.approx(.3)
    assert out['right_age_s'] == pytest.approx(.5) and out['source'] == 'guided'
    assert q.snapshot(1.5, band=None)['sides'] == 'none'


def test_quality_on_leaves_the_estimate_bit_identical():
    off, on = SensorEstimator(), SensorEstimator(EstimatorConfig(corridor_quality=True))

    def compare(t, estimators):
        a, b = (e.snapshot(t) for e in estimators)
        b.pop('corridor_quality')
        assert list(a) == list(b)
        assert all(same(a[k], b[k]) for k in a), t
    drive([off, on], 14., plan=((5., BOTH), (8., LEFT_ONLY), (10., EMPTY), (math.inf, BOTH)), y=.2,
          yaw=math.radians(2.), hook=compare)
    assert off.pose.tobytes() == on.pose.tobytes()
    assert (off.band.heading, off.band.center, off.band.width) == (on.band.heading, on.band.center, on.band.width)


def test_base_pose_is_the_pose_of_the_snapshot_values():
    est = SensorEstimator()
    drive([est], 6., y=.25, yaw=math.radians(3.))
    for now in (6., 6.05, 6.3):
        s = est.snapshot(now)
        x, y, yaw = est.base_pose(now)
        band = est.band
        assert s['heading_error'] == wrap(band.heading - yaw)
        assert s['lateral_offset'] == pytest.approx(-math.sin(band.heading) * x + math.cos(band.heading) * y
                                                    - band.center, abs=1e-12)
    assert SensorEstimator().base_pose(1.) == (pytest.approx(-.18), 0., 0.)      # before any scan: the mount offset


# --- external corridor observation slot ---------------------------------------------------------------------------

def external_config(gain=.2, **changes):
    values = dict(external_corridor_gain=gain, external_corridor_sources=('local_grid', 'camera'))
    values.update(changes)
    return EstimatorConfig(**values)


def tracking(config, seconds=5.):
    est = SensorEstimator(config)
    drive([est], seconds)
    assert est.band is not None
    return est


def band_state(est):
    return (est.band.heading, est.band.center, est.band.width, est.band.stamp, est.band.full_updates,
            est.accepted_updates, est.end_streak, est.band.end_left, est.band.end_right, est.band.end_stamp)


def test_slot_is_off_by_default_and_ignores_observations():
    est = tracking(EstimatorConfig())
    before = band_state(est)
    assert est.external is None
    assert est.on_corridor_observation(5., .3, .05, 3.5, 1., 'local_grid') is None
    assert band_state(est) == before and 'external_corridor' not in est.snapshot(5.)


@pytest.mark.parametrize('changes', (
    dict(external_corridor_gain=-.1), dict(external_corridor_gain=1.5), dict(external_corridor_sources=()),
    dict(external_corridor_sources=['local_grid']), dict(external_corridor_sources=('',)),
    dict(external_corridor_max_age_s=0.), dict(external_corridor_min_quality=2.),
))
def test_slot_config_is_checked(changes):
    with pytest.raises(ValueError, match='external_corridor_invalid'):
        external_config(**changes)


def test_slot_refuses_with_a_reason():
    est = SensorEstimator(external_config())
    est.on_imu(0., 0., (0., 0., 9.81))
    assert est.on_corridor_observation(0., 0., 0., source='local_grid') == 'no_band'
    est = tracking(external_config())
    before = band_state(est)
    y = est.snapshot(5.)['lateral_offset']
    assert est.on_corridor_observation(5., y, 0., source='lidar') == 'source'
    assert est.on_corridor_observation(5., math.nan, 0., source='local_grid') == 'invalid'
    assert est.on_corridor_observation(5., y, 0., quality=True, source='local_grid') == 'invalid'
    assert est.on_corridor_observation(4.9, y, 0., quality=.2, source='local_grid') == 'quality'
    assert est.on_corridor_observation(4.9, y, 0., source='local_grid') == 'order'       # not newer than the last one
    assert est.on_corridor_observation(4.95, y + .5, 0., source='local_grid') == 'offset_gate'
    assert est.on_corridor_observation(4.96, y, math.radians(10.), source='local_grid') == 'heading_gate'
    assert est.on_corridor_observation(4.97, y, 0., est.band.width + .6, source='local_grid') == 'width_gate'
    assert est.on_corridor_observation(5.9, y, 0., source='local_grid') == 'future'
    assert band_state(est) == before
    late = tracking(external_config())
    assert late.on_corridor_observation(4.3, 0., 0., source='camera') == 'stale'
    assert late.on_corridor_observation(4.4, 0., 0., source='camera') == 'stale'
    block = est.snapshot(5.)['external_corridor']
    assert block['fused'] == 0 and block['refused'] == 9 and block['last_reason'] == 'future'
    assert set(est.external.counts) == set(EXTERNAL_REASONS)


def test_slot_pulls_the_band_by_the_gain_and_touches_nothing_else():
    est = tracking(external_config(gain=.25))
    s = est.snapshot(5.)
    y, psi, width = s['lateral_offset'], s['heading_error'], s['corridor_width']
    before = band_state(est)
    # the provider says: 0.2 m further left of the centre, the corridor axis 2 deg further left, 0.3 m wider
    assert est.on_corridor_observation(5., y + .2, psi + math.radians(2.), width + .3, .9, 'local_grid') == 'fused'
    after = est.snapshot(5.)
    assert after['lateral_offset'] - y == pytest.approx(.25 * .2, abs=1e-9)
    assert wrap(after['heading_error'] - psi) == pytest.approx(.25 * math.radians(2.), abs=1e-12)
    assert after['corridor_width'] - width == pytest.approx(.25 * .3, abs=1e-12)
    assert band_state(est)[3:] == before[3:]         # band stamp, update counts and row-end evidence untouched
    block = after['external_corridor']
    assert block['fused'] == 1 and block['source'] == 'local_grid' and block['age_s'] == 0.
    assert after['corridor_age_s'] == s['corridor_age_s'] and after['corridor_updates'] == s['corridor_updates']


def test_slot_uses_the_pose_at_the_observation_stamp():
    est = tracking(external_config(gain=1.), seconds=5.)
    s_then = {}

    def note(t, estimators):
        if t == 4.7:
            s_then.update(estimators[0].snapshot(4.7))
    again = SensorEstimator(external_config(gain=1.))
    drive([again], 5., y=.3, yaw=0., hook=note)
    # an observation stamped 0.3 s ago that agrees with the estimate of that time changes nothing (gain 1)
    before = band_state(again)
    assert again.on_corridor_observation(4.7, s_then['lateral_offset'], s_then['heading_error'],
                                         source='camera') == 'fused'
    assert band_state(again)[0] == pytest.approx(before[0], abs=2e-3)
    assert band_state(again)[1] == pytest.approx(before[1], abs=.02)
    assert est.on_corridor_observation(3.5, 0., 0., source='camera') == 'stale'          # older than max_age_s 0.5


def test_external_observations_never_keep_a_band_alive():
    est = tracking(external_config(gain=.2))
    last_band = est.band.stamp
    outcomes, lost_at = [], []

    def provide(t, estimators):
        e = estimators[0]
        if e.band is not None:
            s = e.snapshot(t)
            outcomes.append(e.on_corridor_observation(t, s['lateral_offset'], s['heading_error'], source='local_grid'))
        elif not lost_at:
            lost_at.append(t)
    drive([est], 4., plan=((math.inf, EMPTY),), hook=provide, start=5.05)      # no LiDAR support any more
    assert outcomes.count('fused') >= 30
    assert est.band is None                                                  # band_timeout_s 2 s all the same
    assert 2. < lost_at[0] - last_band <= 2.25


def test_gain_zero_is_the_original_estimator():
    off, zero = SensorEstimator(), SensorEstimator(EstimatorConfig(external_corridor_gain=0.,
                                                                  external_corridor_sources=('local_grid',)))

    def provide(t, estimators):
        for e in estimators:
            assert e.on_corridor_observation(t, .1, .01, 3.5, 1., 'local_grid') is None
        a, b = (e.snapshot(t) for e in estimators)
        assert list(a) == list(b) and all(same(a[k], b[k]) for k in a), t
    drive([off, zero], 8., hook=provide)
    assert zero.external is None and off.pose.tobytes() == zero.pose.tobytes()
