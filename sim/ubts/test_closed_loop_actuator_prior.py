"""Actuator priors (layer L1, closed_loop/actuator_prior.py) and their use in the estimator (design v2 V2-1 item 2).

Synthetic only: ray-cast scans of a two-row corridor from a scripted pose, a zero or scripted gyro and a scripted
applied duty. No plant and no SIM truth enters an estimator (the scripted pose only makes the scans and is the
reference the tests compare with). Every number is a test value, not a vehicle value.
"""
import math

import numpy as np
import pytest

from closed_loop.actuator_prior import (DEFAULT_SPEED_TABLE, FROZEN_AFTER_S, HOLD_REASONS, KEEP_S, PREDICTING_AFTER_S,
                                        ActuatorPrior, ActuatorPriorConfig, SpeedTable)
from closed_loop.estimator import EstimatorConfig, SensorEstimator
from orchard_perception.scenarios import Circle, OrchardScene, Pose2D, raycast_scan

TABLE = ((20., .2), (40., .4), (60., .6))            # test table: 1 % duty = 0.01 m/s
SCENE = OrchardScene(tuple(Circle(float(x), y, .13) for y in (-1.75, 1.75) for x in np.arange(-6., 60.1, 1.5)), ())
EMPTY = OrchardScene((), ())
LEGACY_KEYS = None                                   # filled by the first OFF snapshot (the original snapshot keys)


def prior_config(**changes):
    values = dict(speed_table=TABLE, wheelbase_m=.8)
    values.update(changes)
    return ActuatorPriorConfig(**values)


# --- speed table ------------------------------------------------------------------------------------------------

def test_speed_table_is_piecewise_linear_through_the_origin_and_held_at_the_top():
    table = SpeedTable(TABLE)
    assert table.speed(0.) == 0.
    assert table.speed(10.) == pytest.approx(.1)                 # below the first point: through (0, 0)
    assert table.speed(30.) == pytest.approx(.3)
    assert table.speed(60.) == pytest.approx(.6)
    assert table.speed(100.) == pytest.approx(.6)                # above the last point: held
    assert table.speed(-30.) == pytest.approx(-.3)               # signed like the duty
    assert table.duty(.3) == pytest.approx(30.)
    assert table.duty(0.) == 0.
    assert table.duty(.61) is None                               # above the table
    plateau = SpeedTable(((20., .2), (40., .2), (60., .6)))
    assert plateau.duty(.2) == pytest.approx(20.)                # the smallest duty that reaches the speed


def test_default_table_is_the_documented_placeholder():
    table = SpeedTable(DEFAULT_SPEED_TABLE)
    assert table.speed(100.) == pytest.approx(.899) and table.speed(90.) == pytest.approx(.715)
    for duty in (25., 40., 55., 80.):                            # the fitted line of docs/drive_response up to 80 %
        assert table.speed(duty) == pytest.approx(.00855 * duty + .025, abs=2e-3)
    assert 42. <= table.duty(.40) <= 45.                         # about 44 % for the 0.40 m/s cruise target


@pytest.mark.parametrize('points', (
    ((40., .4),),                                                # one point
    ((40., .4), (30., .5)),                                      # duty not increasing
    ((30., .4), (40., .3)),                                      # speed falls with the duty
    ((0., 0.), (40., .4)),                                       # duty 0 is implied, not a point
    ((30., .3), (120., .9)),                                     # duty above 100 %
    ((30., -.1), (40., .4)),                                     # negative speed
    ((30., .3), (40., math.nan)),
    ((30., 0.), (40., 0.)),                                      # never moves
    'table',
))
def test_speed_table_refuses_bad_points(points):
    with pytest.raises(ValueError, match='speed_table_invalid'):
        SpeedTable(points)


@pytest.mark.parametrize('changes', (
    dict(speed_scale=0.), dict(speed_scale=math.inf), dict(wheelbase_m=0.), dict(weight=-.1), dict(weight=1.1),
    dict(status_max_age_s=0.), dict(predict_max_s=0.), dict(predict_max_s=KEEP_S), dict(predict_max_m=0.),
    dict(monitor_min_s=30.), dict(monitor_min_speed_m_s=0.), dict(mismatch_fraction=1.), dict(speed_table=((1., 1.),)),
))
def test_config_refuses_bad_values(changes):
    with pytest.raises(ValueError):
        prior_config(**changes)


def test_default_config_is_monitor_only():
    cfg = ActuatorPriorConfig()
    assert cfg.weight == 0. and cfg.predict_max_s == 1. and cfg.predict_max_m == .5 and cfg.status_max_age_s == .3
    assert cfg.wheelbase_m is None and cfg.speed_scale == 1.


# --- input, hold --------------------------------------------------------------------------------------------------

def test_update_refuses_bad_samples_and_keeps_the_order():
    prior = ActuatorPrior(prior_config())
    assert prior.update(1., 40., 40., .1)
    assert not prior.update(1., 40., 40., .1)                    # same stamp
    assert not prior.update(.9, 40., 40., .1)                    # older
    assert not prior.update(1.1, math.nan, 40.)
    assert not prior.update(1.1, 101., 40.)                      # beyond full scale
    assert not prior.update(1.1, 40., 40., 2.)                   # not a steering angle
    assert not prior.update(1.1, True, 40.)
    assert prior.update(1.1, 40., 40.)                           # no steering angle: speed prior only
    assert (prior.accepted, prior.rejected) == (2, 6)
    assert prior.samples[0][2] == pytest.approx(.4 * math.tan(.1) / .8)      # yaw-rate prior v tan(delta) / L
    assert prior.samples[1][2] is None
    assert ActuatorPrior(prior_config(wheelbase_m=None)).update(0., 40., 40., .1)


def test_speed_prior_uses_the_mean_duty_and_the_scale():
    prior = ActuatorPrior(prior_config(speed_scale=1.2))
    prior.update(0., 30., 50.)
    assert prior.samples[-1][1] == pytest.approx(.4 * 1.2) and prior.samples[-1][3] == 40.
    prior.update(.1, -40., -40.)
    assert prior.samples[-1][1] == pytest.approx(-.4 * 1.2)


def test_a_status_sample_holds_for_at_most_the_status_age():
    prior = ActuatorPrior(prior_config())                        # status_max_age_s 0.3
    for k in range(11):
        prior.update(k * .1, 40., 40.)
    assert prior.displacement(0., 1.) == (pytest.approx(.4), True)
    assert prior.displacement(.95, 1.25) == (pytest.approx(.4 * .3), True)   # the last sample holds 0.3 s
    step, complete = prior.displacement(1., 1.5)
    assert not complete and step == pytest.approx(.4 * .3)                   # then no prior
    assert prior.displacement(-1., 0.5)[1] is False                          # nothing before the first sample
    assert prior.displacement(2., 2.) == (0., True)
    assert prior.latest(1.2)[1] == pytest.approx(.4)
    assert prior.latest(1.31) is None and prior.latest(.9) is None           # stale / from the future
    for k in range(60):
        prior.update(2. + k * .1, 40., 40.)
    assert prior.samples[0][0] >= prior.samples[-1][0] - KEEP_S              # old samples are dropped


# --- (a) seed, (b) prediction ------------------------------------------------------------------------------------

def feed(prior, t0, t1, duty, step=.05):
    t = t0
    while t <= t1 + 1e-9:
        prior.update(round(t, 6), duty, duty)
        t += step


def test_seed_is_the_blend_and_none_with_weight_zero_or_without_a_status():
    off = ActuatorPrior(prior_config())
    feed(off, 0., 1., 40.)
    assert off.seed(.5, .6, .3) is None                          # weight 0: the original seed
    half = ActuatorPrior(prior_config(weight=.5))
    feed(half, 0., 1., 40.)
    assert half.seed(.5, .6, .3) == pytest.approx(.5 * .3 * .1 + .5 * .4 * .1)
    assert half.seed(1.2, 1.6, .3) is None                       # the status does not cover the interval
    full = ActuatorPrior(prior_config(weight=1.))
    feed(full, 0., 1., 40.)
    assert full.seed(.5, .6, 9.) == pytest.approx(.04)           # weight 1: the LiDAR speed is not used


def test_prediction_stops_at_the_time_bound():
    prior = ActuatorPrior(prior_config(weight=1., predict_max_m=10.))
    feed(prior, 0., 3., 40.)
    moved = [prior.predict(k * .1, (k + 1) * .1) for k in range(14)]
    assert sum(moved[:10]) == pytest.approx(.4, abs=1e-9) and moved[10:] == [0.] * 4     # 1.0 s, then nothing
    assert prior.limit and prior.predict_s == pytest.approx(1.4) and prior.predicted_scans == 10
    prior.matched()
    assert not prior.limit and prior.predict_s == 0. and prior.predict_m == 0.
    assert prior.predict(1.4, 1.5) == pytest.approx(.04)


def test_prediction_stops_at_the_distance_bound():
    prior = ActuatorPrior(prior_config(weight=1., predict_max_s=3., predict_max_m=.5, speed_table=((50., 1.), (100., 2.))))
    feed(prior, 0., 3., 50.)                                     # 1 m/s
    moved = [prior.predict(k * .1, (k + 1) * .1) for k in range(8)]
    assert sum(moved) == pytest.approx(.5) and moved[5:] == [0.] * 3 and prior.limit      # 0.5 m after 0.5 s
    assert prior.predict_m == pytest.approx(.5)


def test_prediction_needs_a_weight_and_a_fresh_status():
    off = ActuatorPrior(prior_config())
    feed(off, 0., 1., 40.)
    assert off.predict(.2, .3) == 0. and off.predict_s == 0. and not off.limit           # weight 0: untouched
    stale = ActuatorPrior(prior_config(weight=1.))
    feed(stale, 0., .5, 40.)
    assert stale.predict(.9, 1.) == 0.                           # the last status line is older than 0.3 s
    assert stale.predict_s == pytest.approx(.1) and not stale.limit
    part = ActuatorPrior(prior_config(weight=.5))
    feed(part, 0., 1., 40.)
    assert part.predict(.2, .3) == pytest.approx(.02)            # the weight scales the predicted step


def test_extrapolation_is_bounded_and_does_not_change_the_state():
    prior = ActuatorPrior(prior_config(weight=1.))
    feed(prior, 0., 3., 40.)
    assert prior.extrapolate(1., 1.1, .1) == (pytest.approx(.04), False)
    travel, limit = prior.extrapolate(1., 2.5, .1)               # 1.5 s after the newest scan: 1 s counts
    assert travel == pytest.approx(.4) and limit
    fast = ActuatorPrior(prior_config(weight=1., speed_table=((50., 1.), (100., 2.))))
    feed(fast, 0., 3., 50.)
    assert fast.extrapolate(1., 1.8, 0.) == (pytest.approx(.5), True)        # 0.5 m
    assert (prior.predict_s, prior.predict_m, prior.limit) == (0., 0., False)
    assert ActuatorPrior(prior_config()).extrapolate(0., 1., .3) is None     # weight 0: the original rule
    stale = ActuatorPrior(prior_config(weight=1.))
    feed(stale, 0., .5, 40.)
    assert stale.extrapolate(.5, 1.2, .3) is None                # no status over the time: the original rule
    prior.predict(1., 1.1)
    for k in range(8):
        prior.predict(1.1 + k * .1, 1.2 + k * .1)                # 0.9 s of failed matches already predicted
    travel, limit = prior.extrapolate(1.9, 2.3, .1)
    assert travel == pytest.approx(.04) and limit                # only the remaining 0.1 s of the bound


# --- (c) monitor ----------------------------------------------------------------------------------------------------

def monitored(table_factor, seconds=25., lidar_speed=.4):
    """Prior with a table that is table_factor x the 'driven' speed; LiDAR steps of lidar_speed."""
    prior = ActuatorPrior(prior_config(speed_scale=table_factor))
    t = 0.
    while t < seconds:
        prior.update(round(t, 6), 40., 40.)
        prior.update(round(t + .05, 6), 40., 40.)
        prior.observe(round(t, 6), round(t + .1, 6), lidar_speed * .1)
        t += .1
    return prior


def test_online_scale_and_the_table_mismatch_flag():
    good = monitored(1.)
    assert good.online_scale() == pytest.approx(1., abs=1e-6) and not good.table_mismatch()
    assert good.window_s == pytest.approx(20., abs=.11)          # the monitor window
    assert good.distance == pytest.approx(10., abs=.05)          # (e) cumulative prior distance over 25 s
    high = monitored(1.3)                                        # table 30 % too fast
    assert high.online_scale() == pytest.approx(1 / 1.3, abs=1e-6) and high.table_mismatch()
    low = monitored(.7)                                          # table 30 % too slow
    assert low.online_scale() == pytest.approx(1 / .7, abs=1e-6) and low.table_mismatch()
    assert not monitored(1.15).table_mismatch() and not monitored(.85).table_mismatch()    # inside the 20 % band
    young = monitored(1.3, seconds=8.)
    assert young.online_scale() is None and not young.table_mismatch()       # less than monitor_min_s of driving
    stuck = monitored(1., lidar_speed=0.)                        # the duty says driving, the LiDAR sees no motion
    assert stuck.online_scale() == 0. and stuck.table_mismatch()


def test_monitor_ignores_standstill_and_intervals_without_a_status():
    prior = ActuatorPrior(prior_config())
    for k in range(300):                                         # 0.05 m/s prior: below monitor_min_speed_m_s
        prior.update(k * .1, 5., 5.)
        prior.update(k * .1 + .05, 5., 5.)
        prior.observe(k * .1, (k + 1) * .1, 0.)
    assert prior.window_s == 0. and prior.online_scale() is None
    assert prior.distance == pytest.approx(.05 * 30., abs=.02)   # the distance still counts
    prior.observe(40., 40.1, .04)                                # no status line there
    assert prior.window_s == 0.


def test_held_scale_reports_why_it_does_not_change():
    """PW-10: the online scale changes only with a scan-matched driving interval; otherwise it is held and the snapshot
    says why (failed match, no new scan, not driving, no status line)."""
    prior = monitored(1. / .85)                                  # the vehicle drives at 0.85 x the table speed
    scale = prior.online_scale()
    assert scale == pytest.approx(.85, abs=1e-9)
    block = prior.snapshot(25.05, 25., .4)
    assert not block['online_scale_frozen'] and block['online_scale_hold'] is None
    assert block['online_scale_age_s'] == pytest.approx(.05) and block['monitor_window_s'] == 20.
    assert block['monitor_min_s'] == 10. and block['unmatched_scans'] == 0
    t = 25.
    for k in range(10):                                          # 1 s of scans that do not match (driving on)
        prior.update(round(t, 6), 40., 40.)
        prior.update(round(t + .05, 6), 40., 40.)
        prior.predict(round(t, 6), round(t + .1, 6))             # weight 0: nothing predicted, only counted
        t += .1
    assert prior.online_scale() == scale and prior.predicted_scans == 0
    block = prior.snapshot(round(t + .05, 6), round(t, 6), .4)
    assert block['online_scale'] == scale and block['online_scale_frozen'] is True
    assert block['online_scale_hold'] == 'lidar_unmatched' and block['unmatched_scans'] == 10
    # no new scan for more than FROZEN_AFTER_S (a LiDAR blackout): lidar_stale
    block = prior.snapshot(round(t + FROZEN_AFTER_S + .1, 6), round(t, 6), .4)
    assert block['online_scale_hold'] == 'lidar_stale' and block['online_scale'] == scale
    assert prior.snapshot(t, None, 0.)['online_scale_hold'] == 'lidar_stale'
    # matched scans again while the vehicle stands (duty 0): not_driving, the scale still held
    for k in range(10):
        prior.update(round(t, 6), 0., 0.)
        prior.update(round(t + .05, 6), 0., 0.)
        prior.observe(round(t, 6), round(t + .1, 6), 0.)
        t += .1
    block = prior.snapshot(round(t + .05, 6), round(t, 6), 0.)
    assert block['online_scale_hold'] == 'not_driving' and block['online_scale'] == scale
    # a matched interval without a status line: no_status
    prior.observe(round(t + 10., 6), round(t + 10.1, 6), .04)
    assert prior.snapshot(round(t + 10.15, 6), round(t + 10.1, 6), .4)['online_scale_hold'] == 'no_status'
    # a matched driving interval updates it again
    t += 10.2
    prior.update(round(t, 6), 40., 40.)
    prior.update(round(t + .05, 6), 40., 40.)
    prior.observe(round(t, 6), round(t + .1, 6), .06)                # faster than 0.85 x: the scale moves
    block = prior.snapshot(round(t + .15, 6), round(t + .1, 6), .4)
    assert not block['online_scale_frozen'] and block['online_scale_hold'] is None
    assert block['online_scale'] != scale
    assert set(HOLD_REASONS) == {'lidar_stale', 'lidar_unmatched', 'not_driving', 'no_status'}


def test_no_scale_yet_is_not_frozen_but_says_why():
    prior = ActuatorPrior(prior_config())
    block = prior.snapshot(1., None, 0.)
    assert block['online_scale'] is None and block['online_scale_frozen'] is False
    assert block['online_scale_hold'] == 'lidar_stale' and block['online_scale_age_s'] is None


# --- the estimator --------------------------------------------------------------------------------------------------

def scan_at(x, y=0., yaw=0., scene=SCENE):
    s = raycast_scan(scene, Pose2D(x, y, yaw))
    return np.asarray(s.ranges), s.angle_min, s.angle_increment, s.range_max


def drive(estimators, seconds, speed=.4, duty=40., gaps=(), blind=(), status_until=math.inf, snapshots=None,
          start=0.):
    """Scripted straight drive along the corridor at `speed`: 10 Hz scans (none inside `gaps`, featureless inside
    `blind`), 20 Hz gyro 0, 20 Hz status lines of `duty` (none after status_until). snapshots(t, estimators) is called
    at 20 Hz after the packets of that time."""
    steps = int(round(seconds / .05))
    for k in range(steps + 1):
        t = round(start + k * .05, 6)
        for est in estimators:
            est.on_imu(t, 0., (0., 0., 9.81))
            if t <= status_until:
                est.on_actuator(t, duty, duty, 0.)
        if k % 2 == 0 and not any(a <= t < b for a, b in gaps):
            scene = EMPTY if any(a <= t < b for a, b in blind) else SCENE
            packet = scan_at(speed * t, scene=scene)
            for est in estimators:
                est.on_scan(t, packet[0].copy(), *packet[1:])
        if snapshots is not None:
            snapshots(t, estimators)


def estimator(weight=None, **prior):
    if weight is None:
        return SensorEstimator(EstimatorConfig())
    return SensorEstimator(EstimatorConfig(actuator_prior=prior_config(weight=weight, **prior)))


def base_x(est, now):
    """Along-corridor position of the estimator's latency-compensated LiDAR pose (the drive is along +x)."""
    s = est.snapshot(now)
    travel = est.prior.extrapolate(est.last_scan_stamp, now, est.speed) if est.prior is not None else None
    horizon = max(0., now - est.last_scan_stamp)
    return est.pose[0] + (travel[0] if travel is not None else est.speed * horizon), s


def test_off_estimator_ignores_the_actuator_input():
    est = estimator()
    assert est.prior is None and est.on_actuator(0., 40., 40., 0.) is False
    drive([est], 2.)
    assert est.prior is None and 'actuator_prior' not in est.snapshot(2.)


def test_weight_zero_leaves_the_estimate_bit_identical():
    off, monitor = estimator(), estimator(0.)
    seen = []

    def compare(t, estimators):
        a, b = (e.snapshot(t) for e in estimators)
        block = b.pop('actuator_prior')
        assert list(a) == list(b)
        for key in a:
            assert a[key] == b[key] or (a[key] != a[key] and b[key] != b[key]), (t, key)
        seen.append(block)
    drive([off, monitor], 16., gaps=((6., 7.),), blind=((10., 10.6),), snapshots=compare)
    assert off.pose.tobytes() == monitor.pose.tobytes() and off.distance == monitor.distance
    assert off.odom_failures == monitor.odom_failures > 0
    block = seen[-1]
    assert block['fresh'] and block['weight'] == 0. and block['speed_m_s'] == pytest.approx(.4)
    assert not block['predicting'] and not block['limit'] and block['predict_m'] == 0. and block['predicted_scans'] == 0
    assert block['monitor_s'] > 10. and block['online_scale'] is not None
    assert block['distance_m'] > 4.                              # (e) the prior distance is recorded all the same


def test_prior_seed_matches_across_a_scan_gap_where_lidar_alone_loses_the_translation():
    off, on = estimator(), estimator(1.)
    drive([off, on], 9., gaps=((4.05, 5.05),))                   # scans at 4.1 .. 5.0 are missing (gap 1.1 s)
    true_x = .4 * 9.
    assert off.odom_failures >= 1 and on.odom_failures == 0      # dt >= 0.5 s: no match without a seed
    assert abs(off.pose[0] - true_x) > .3                        # the original rule never invents translation
    assert abs(on.pose[0] - true_x) < .05                        # the prior seed let the scan match bridge the gap
    assert on.prior.predict_s == 0. and not on.prior.limit


def test_snapshot_follows_the_prior_through_a_scan_gap_and_stops_at_the_bound():
    on = estimator(1.)
    errors, blocks = {}, {}

    def watch(t, estimators):
        if 4. <= t <= 6.5:
            x, s = base_x(estimators[0], t)
            errors[t], blocks[t] = x - .4 * t, s['actuator_prior']
    drive([on], 8., gaps=((4.05, 6.55),), snapshots=watch)       # 2.5 s without scans
    assert all(abs(errors[t]) < .03 for t in errors if t <= 5.)                # 1 s of prediction: on the prior
    assert not blocks[4.9]['limit'] and blocks[4.9]['predicting']
    assert blocks[4.2]['predicting'] is False                    # 0.2 s after a scan is ordinary latency
    assert blocks[5.5]['limit'] and blocks[6.5]['limit']         # beyond 1 s: the bound
    frozen = [errors[t] + .4 * t for t in errors if t >= 5.1]
    assert max(frozen) - min(frozen) < 1e-9                      # nothing is added after the bound
    assert max(frozen) - .4 * 4. == pytest.approx(.4, abs=.03)   # 1 s x 0.4 m/s = 0.4 m (< predict_max_m)
    assert blocks[6.5]['predict_s'] > 1. and blocks[6.5]['predict_m'] == pytest.approx(.4, abs=.03)


def test_failed_matches_get_a_bounded_prediction_and_a_match_clears_it():
    off, on = estimator(), estimator(1.)
    marks = {}

    def watch(t, estimators):
        if t in (5.9, 6.4, 7.4):
            marks[t] = (estimators[1].pose[0], estimators[1].snapshot(t)['actuator_prior'], estimators[0].pose[0])
    drive([off, on], 9., blind=((5., 7.), ), snapshots=watch)    # featureless scans for 2 s: every match fails
    x_before = marks[5.9][0]
    assert marks[5.9][1]['predicted_scans'] == 10 and not marks[5.9][1]['limit']
    assert x_before == pytest.approx(.4 * 5.9, abs=.05)          # 1 s of failed matches: predicted on the prior
    assert marks[6.4][0] == pytest.approx(.4 * 5.9, abs=.06) and marks[6.4][1]['limit']    # then frozen
    assert marks[6.4][2] == pytest.approx(.4 * 4.9, abs=.05)     # the original rule stayed at the last matched pose
    assert not marks[7.4][1]['limit'] and marks[7.4][1]['predict_s'] < .5    # matched again: the bound restarts
    assert off.odom_failures == on.odom_failures >= 20


def test_stale_status_switches_the_prior_off():
    on = estimator(1.)
    drive([on], 6., gaps=((4.05, 5.55),), status_until=3.)       # the status lines stop at 3 s
    block = on.snapshot(6.)['actuator_prior']
    assert not block['fresh'] and block['speed_m_s'] is None and block['age_s'] == pytest.approx(3.)
    assert on.odom_failures >= 1                                 # no seed: the gap is not matched (original rule)
    assert on.prior.predicted_scans == 0


def test_table_error_shows_in_the_online_scale_of_a_drive():
    exact, high, low = estimator(0.), estimator(0., speed_scale=1.3), estimator(0., speed_scale=.7)
    drive([exact, high, low], 26.)
    blocks = [e.snapshot(26.)['actuator_prior'] for e in (exact, high, low)]
    assert blocks[0]['online_scale'] == pytest.approx(1., abs=.08) and not blocks[0]['table_mismatch']
    assert blocks[1]['online_scale'] == pytest.approx(1 / 1.3, abs=.08) and blocks[1]['table_mismatch']
    assert blocks[2]['online_scale'] == pytest.approx(1 / .7, abs=.1) and blocks[2]['table_mismatch']
    assert all(b['monitor_s'] >= 10. for b in blocks)


def test_estimator_holds_the_scale_through_failed_matches_and_a_scan_blackout():
    """PW-10: the estimator feeds the monitor only after a scan match; featureless scans (fresh, no match) and a scan
    gap leave the online scale unchanged and the snapshot reports the hold."""
    est = estimator(0., speed_scale=1. / .85)                   # driven at 0.85 x the table speed
    seen = []

    def watch(t, estimators):
        block = estimators[0].snapshot(t)['actuator_prior']
        seen.append((t, block['online_scale'], block['online_scale_frozen'], block['online_scale_hold']))
    drive([est], 30., blind=((16., 19.),), gaps=((23., 26.),), snapshots=watch)
    at = {round(t, 2): (k, frozen, hold) for t, k, frozen, hold in seen}
    k_blind = at[16.0][0]
    assert k_blind == pytest.approx(.85, abs=.05)
    assert all(at[round(16. + .05 * i, 2)][0] == k_blind for i in range(61))          # 16.0 .. 19.0 s: held
    assert at[17.0][1] and at[17.0][2] == 'lidar_unmatched'
    assert not at[21.0][1] and at[21.0][0] != k_blind                                  # matched again: updating
    k_gap = at[23.0][0]
    assert all(at[round(23. + .05 * i, 2)][0] == k_gap for i in range(61))            # 23.0 .. 26.0 s: held
    assert at[24.0][1] and at[24.0][2] == 'lidar_stale'
    assert est.prior.predicted_scans == 0                                              # weight 0: monitor only


def test_snapshot_block_reports_the_yaw_rate_prior():
    on = estimator(0.)
    on.on_imu(0., 0., (0., 0., 9.81))
    on.on_actuator(0., 40., 40., .1)
    block = on.snapshot(.1)['actuator_prior']
    assert block['yaw_rate_rad_s'] == pytest.approx(.4 * math.tan(.1) / .8) and block['steer_rad'] == .1
    assert block['duty_pct'] == 40. and block['age_s'] == pytest.approx(.1) and block['samples'] == 1
    assert PREDICTING_AFTER_S == .25
