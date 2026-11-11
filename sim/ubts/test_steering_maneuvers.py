"""Open-loop scripted maneuvers (steering/maneuvers.py; design 8.3 P2).

Command sequences of step, ramp, circle, sine sweep, zigzag (time and heading switch), stop test and stop by stop
mode against hand-computed values; parameter normalisation and errors; speed -> duty against the drive curve;
the Command fields that closed_loop.harness / SteeringActuation read. Stand-in states only (no plant), so this
runs on Python 3.10 / 3.12 / 3.13.

Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_maneuvers.py
"""
import math

import pytest

from steering import drive
from steering import maneuvers as mv
from steering import steered_plant as sp
from steering import vehicle_profile as vp

DT = .05


def state(t, v=.4, yaw=0., dist=0.):
    return mv.ManeuverState(t, v, yaw, dist)


def run(maneuver, until_s, v=.4, yaw_of=None):
    """Commands at 20 Hz from t = 0.05 (harness controller ticks)."""
    out = []
    k = 1
    while k * DT <= until_s + 1e-9:
        t = k * DT
        yaw = 0. if yaw_of is None else yaw_of(t)
        out.append((t, maneuver.command(t, state(t, v=v, yaw=yaw))))
        k += 1
    return out


def test_command_carries_the_harness_fields():
    command = mv.Command(30., 120.)
    assert command.throttle_pct == 30. and command.steer_target_mrad == 120.
    assert command.stop_mode is None and command.steer_policy is None and command.phase == 'run'
    assert sp.SteeringActuation().values(command) == (30., 120.)
    assert command[:2] == (30., 120.)


def test_step():
    m = mv.make_maneuver('step', dict(at_s=1., throttle_pct=[20., 50.], steer_deg=[0., 5.]))
    seq = run(m, 2.)
    before = [c for t, c in seq if t < 1.]
    after = [c for t, c in seq if t >= 1.]
    assert all(c == mv.Command(20., 0., phase='before') for c in before)
    assert all(c.throttle_pct == 50. and c.steer_target_mrad == pytest.approx(1000. * math.radians(5.))
               and c.phase == 'after' for c in after)
    assert not m.done(2., state(2.))


def test_ramp_interpolates_both_channels():
    m = mv.make_maneuver('ramp', dict(start_s=1., end_s=3., throttle_pct=[0., 40.], steer_deg=[-10., 10.]))
    assert m.command(.5, state(.5)) == mv.Command(0., 1000. * math.radians(-10.), phase='hold_start')
    mid = m.command(2., state(2.))
    assert mid.phase == 'ramp' and mid.throttle_pct == pytest.approx(20.) and mid.steer_target_mrad == pytest.approx(0.)
    quarter = m.command(1.5, state(1.5))
    assert quarter.throttle_pct == pytest.approx(10.) and quarter.steer_target_mrad == pytest.approx(
        1000. * math.radians(-5.))
    assert m.command(3., state(3.)) == mv.Command(40., 1000. * math.radians(10.), phase='hold_end')


def test_circle_with_a_throttle_or_a_speed():
    m = mv.make_maneuver('circle', dict(throttle_pct=45., steer_deg=-12.))
    assert m.command(.05, state(.05)) == mv.Command(45., 1000. * math.radians(-12.), phase='circle')
    calls = []
    speed = mv.make_maneuver('circle', dict(speed_m_s=.3, steer_deg=5.), lambda v: calls.append(v) or 33.3)
    assert calls == [.3] and speed.throttle == 33.3
    assert speed.describe()['resolved_throttle_pct'] == 33.3 and speed.describe()['reads_truth_state'] is False
    with pytest.raises(mv.ManeuverError, match='throttle_for_speed'):
        mv.make_maneuver('circle', dict(speed_m_s=.3, steer_deg=5.))
    with pytest.raises(mv.ManeuverError, match='resolved_throttle_out_of_range'):
        mv.make_maneuver('circle', dict(speed_m_s=.3, steer_deg=5.), lambda v: 140.)


def test_sine_sweep_is_a_linear_chirp():
    p = dict(throttle_pct=40., amplitude_deg=6., f0_hz=.1, f1_hz=.5, sweep_s=10., start_s=2., offset_deg=1.)
    m = mv.make_maneuver('sine_sweep', p)
    assert m.command(1., state(1.)) == mv.Command(40., 1000. * math.radians(1.), phase='lead_in')
    for t in (2., 3.3, 7.5, 11.95):
        tau = t - 2.
        expected = 1. + 6. * math.sin(2. * math.pi * (.1 * tau + .4 * tau * tau / 20.))
        command = m.command(t, state(t))
        assert command.phase == 'sweep' and command.steer_target_mrad == pytest.approx(1000. * math.radians(expected))
    assert m.command(12., state(12.)) == mv.Command(40., 1000. * math.radians(1.), phase='hold')


def test_zigzag_time_switch():
    m = mv.make_maneuver('zigzag', dict(throttle_pct=40., amplitude_deg=8., switch='time', half_period_s=1.,
                                        start_s=1., switches=4))
    seq = run(m, 6.)
    amp = 1000. * math.radians(8.)
    phases = set()
    for t, c in seq:
        k = int(math.floor((t - 1.) / 1.))         # the maneuver's half-period index
        if t < 1.:
            assert c.phase == 'lead_in' and c.steer_target_mrad == 0.
        elif k < 4:
            assert c.steer_target_mrad == pytest.approx(amp if k % 2 == 0 else -amp)
            assert c.phase == ('left' if k % 2 == 0 else 'right')
        else:
            assert c.phase == 'hold' and c.steer_target_mrad == 0.
        phases.add(c.phase)
    assert phases == {'lead_in', 'left', 'right', 'hold'}


def test_zigzag_heading_switch_reads_the_heading_change():
    m = mv.make_maneuver('zigzag', dict(speed_m_s=.4, amplitude_deg=10., switch='heading', heading_deg=5.,
                                        start_s=0., switches=3), lambda v: 47.)
    assert m.describe()['reads_truth_state'] is True
    amp = 1000. * math.radians(10.)
    # heading starts at 0.2 rad (the zigzag measures the change from the first command)
    seq = [(.05, .2), (.10, .2 + math.radians(4.)), (.15, .2 + math.radians(5.01)), (.20, .2 + math.radians(2.)),
           (.25, .2 - math.radians(5.01)), (.30, .2 - math.radians(1.)), (.35, .2 + math.radians(6.)), (.40, .2)]
    out = [m.command(t, state(t, yaw=yaw)) for t, yaw in seq]
    assert [c.steer_target_mrad for c in out[:2]] == [pytest.approx(amp)] * 2
    assert out[2].steer_target_mrad == pytest.approx(-amp) and out[2].phase == 'right'   # +5 deg reached: reverse
    assert out[3].steer_target_mrad == pytest.approx(-amp)
    assert out[4].steer_target_mrad == pytest.approx(amp) and out[4].phase == 'left'     # -5 deg reached: reverse
    assert out[5].steer_target_mrad == pytest.approx(amp)
    assert out[6].phase == 'hold' and out[6].steer_target_mrad == 0.                       # third reversal: done
    assert out[7].phase == 'hold'


def test_stop_test_cycles_and_done():
    m = mv.make_maneuver('stop_test', dict(throttle_pct=60., steer_deg=2., drive_s=1., hold_s=.5, repeat=2))
    assert m.describe()['reads_truth_state'] is True
    speeds = {}
    timeline = []
    t, k = 0., 0
    v = .5
    phase_log = []
    while k < 200 and not m.done(t, state(t, v=v)):
        k += 1
        t = k * DT
        command = m.command(t, state(t, v=v))
        timeline.append((t, command))
        phase_log.append(command.phase)
        if command.phase == 'stop':
            v = max(0., v - .1)                     # stand-in deceleration: reaches 0 after five ticks
        elif command.phase == 'drive':
            v = .5
        speeds[t] = v
    assert m.done(t, state(t, v=v))
    stops = [c for _, c in timeline if c.phase == 'stop']
    assert stops and all(c.throttle_pct == 0. and c.stop_mode is None for c in stops)
    assert all(c.steer_target_mrad == pytest.approx(1000. * math.radians(2.)) for _, c in timeline)
    runs = [p for i, p in enumerate(phase_log) if i == 0 or p != phase_log[i - 1]]
    assert runs == ['drive', 'stop', 'rest', 'drive', 'stop', 'rest']
    first_stop = next(t for t, c in timeline if c.phase == 'stop')
    assert first_stop == pytest.approx(1.)
    rest_times = [t for t, c in timeline if c.phase == 'rest']
    assert rest_times[0] > first_stop


def test_stop_mode_commands():
    m = mv.make_maneuver('stop_mode', dict(mode='coast', steer_policy='hold_target', throttle_pct=50., drive_s=.5,
                                           hold_s=0., repeat=1))
    first = m.command(.05, state(.05))
    assert first == mv.Command(50., 0., phase='drive')
    stop = m.command(.5, state(.5, v=.3))
    assert stop.stop_mode == 'coast' and stop.steer_policy == 'hold_target' and stop.phase == 'stop'
    assert stop.throttle_pct == 0.
    rest = m.command(.55, state(.55, v=0.))
    assert rest.phase == 'rest' and rest.stop_mode == 'coast'          # the stop command repeats while resting
    assert m.done(.55, state(.55, v=0.))                                # hold_s 0 and one cycle: done at once
    default = mv.make_maneuver('stop_mode', dict(throttle_pct=50.))
    assert default.params['mode'] == 'brake' and default.params['steer_policy'] is None


def test_stop_without_motion_rests_at_once():
    m = mv.make_maneuver('stop_mode', dict(mode='brake', throttle_pct=3., drive_s=.1, hold_s=.1))
    phases = [m.command(t, state(t, v=0.)).phase for t in (.05, .1, .15, .2)]
    assert phases == ['drive', 'rest', 'rest', 'rest'] and m.done(.2, state(.2, v=0.))


@pytest.mark.parametrize('kind, params, code', [
    ('spin', {}, 'maneuver_kind_unknown'),
    ('circle', dict(steer_deg=5.), 'exactly_one_of_throttle_pct_and_speed_m_s'),
    ('circle', dict(throttle_pct=30., speed_m_s=.3), 'exactly_one_of_throttle_pct_and_speed_m_s'),
    ('circle', dict(throttle_pct=130.), 'out_of_range'),
    ('circle', dict(throttle_pct=30., steer_deg=90.), 'out_of_range'),
    ('circle', dict(throttle_pct=30., colour=1), 'maneuver_param_unknown'),
    ('step', dict(throttle_pct=[1., 2., 3.]), 'pair_needs_two_values'),
    ('step', dict(at_s=-1.), 'out_of_range'),
    ('step', dict(steer_deg=True), 'not_a_number'),
    ('ramp', dict(start_s=3., end_s=2.), 'ramp_end_before_start'),
    ('zigzag', dict(throttle_pct=30., switch='distance'), 'maneuver_param_choice'),
    ('zigzag', dict(throttle_pct=30., switches=0), 'maneuver_param_count'),
    ('stop_mode', dict(throttle_pct=30., mode='drift'), 'maneuver_param_choice'),
    ('stop_test', dict(speed_m_s=-.2), 'not_positive'),
    ('sine_sweep', dict(throttle_pct=30., f0_hz=0.), 'not_positive'),
])
def test_invalid_parameters(kind, params, code):
    with pytest.raises(mv.ManeuverError, match=code):
        mv.normalise_params(kind, params)


def test_normalised_parameters_are_complete():
    out = mv.normalise_params('zigzag', dict(throttle_pct=30))
    assert out == dict(throttle_pct=30., speed_m_s=None, amplitude_deg=10., switch='time', half_period_s=2.,
                       heading_deg=10., start_s=2., switches=6)
    assert mv.normalise_params('step', dict(throttle_pct=-20, steer_deg=3)) == dict(
        at_s=1., throttle_pct=[-20., -20.], steer_deg=[3., 3.])


@pytest.mark.parametrize('curve', ['main', 'bracket'])
def test_duty_for_speed_inverts_the_drive_curve(curve):
    profile = vp.load_profile()
    top = drive.steady_speed_m_s(profile, 100., curve)
    for speed in (.3, .4, .5, .8 * top):
        duty = mv.duty_for_speed(profile, speed, curve)
        assert 0. < duty <= 100.
        assert drive.steady_speed_m_s(profile, duty, curve) == pytest.approx(speed, abs=1e-9)
    with pytest.raises(mv.ManeuverError, match='outside_the_drive_curve'):
        mv.duty_for_speed(profile, top + .01, curve)
