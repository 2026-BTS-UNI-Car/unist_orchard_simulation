"""SteeredPlant (F-SIM) and the L0 ideal steering servo backend (steering/steered_plant.py, steering/l0_servo.py).

Design 7.2 checks: A1(F) circle radius, A2 (the L0 part: rate limit, lag, end stop), A3(F) (the L0 part: error-free
potentiometer, LSB, clipping), A4(F) drive curve, ramp and stop modes, A8 determinism, A21 (geometry: targets clamped,
wheels inside the per-wheel mechanical limits). Also the harness hook contract (command() refuses differential commands,
the attributes the harness reads, SteeringActuation and the plant factory), the _KinematicBicycle contract stand-in and
a comparison with the bicycle model of the survey probe. All values are placeholders; results are placeholder-based SIM
results, not field validation. The default backend follows the profile's steer_actuator_level (shipped profile: L1,
the native backend); the tests that check the L0 servo use a profile view with steer_actuator_level L0 (as_l0). The
native-backend tests skip without a C++ compiler; every native build goes to a pytest temporary folder (never into
the work tree).

Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_plant.py
"""
import importlib
import inspect
import json
import math
import os
import statistics
import warnings

import numpy as np
import pytest

import field_response as FR
from steering import backend_api as ba
from steering import drive
from steering import geometry as geo
from steering import l0_servo as L0
from steering import steered_plant as SP
from steering import vehicle_profile as vp

DT = .01
ZERO = {'dist': 'normal', 'mean': 0.0, 'sd': 0.0}
L0_LEVEL = {'steer_actuator_level': 'L0'}


@pytest.fixture(scope='module', autouse=True)
def native_build_dir_outside_the_work_tree(tmp_path_factory):
    """A plant whose profile asks for L1 builds the native backend on demand; keep that build out of the work tree."""
    if os.environ.get('UBTS_STEER_BUILD_DIR'):
        yield
        return
    os.environ['UBTS_STEER_BUILD_DIR'] = str(tmp_path_factory.mktemp('steer_native_build'))
    try:
        yield
    finally:
        os.environ.pop('UBTS_STEER_BUILD_DIR', None)


@pytest.fixture(scope='module')
def profile():
    return vp.load_profile()


def as_l0(profile, label='l0_backend', **extra):
    """The profile view with steer_actuator_level L0: the plant's default backend is then the Python L0 servo."""
    overrides = dict(L0_LEVEL)
    overrides.update(extra)
    return profile.with_overrides(overrides, label=label)


@pytest.fixture(scope='module')
def default_params(profile):
    return SP.SteeredPlantParams.from_profile(as_l0(profile))


@pytest.fixture(scope='module')
def ideal_params(profile):
    return SP.SteeredPlantParams.from_profile(profile, geometry_check=True)


def no_offset(profile, **extra):
    overrides = dict(L0_LEVEL, steer_zero_offset_deg=ZERO)
    overrides.update(extra)
    return profile.with_overrides(overrides, label='test_no_offset')


def l0_backend(profile, seed=0, **overrides):
    backend = L0.L0ServoBackend()
    backend.configure(profile.with_overrides(overrides, label='test_l0') if overrides else profile, seed)
    return backend


def servo(backend, target_rad, dt=DT, moving=True, en=True):
    return backend.step(ba.SteerStepInput(dt_s=dt, mode='test_servo', target_delta_eq_rad=target_rad, steer_en=en,
                                          vehicle_speed_m_s=.4 if moving else 0., vehicle_stopped=not moving))


def duty_for_speed(params, speed):
    """Throttle % whose steady speed on the plant curve is `speed` (no process noise): bisection."""
    lo, hi = 0., 100.
    for _ in range(200):
        mid = .5 * (lo + hi)
        if FR.forward_curve(mid * params.battery_voltage_scale, params.curve_params) < speed:
            lo = mid
        else:
            hi = mid
    return .5 * (lo + hi)


def fit_circle(points):
    """Least-squares (Kasa) circle through 2-D points: (radius, centre)."""
    pts = np.asarray(points, dtype=float)
    centre = pts.mean(axis=0)
    x, y = (pts - centre).T
    solution = np.linalg.lstsq(np.column_stack([x, y, np.ones_like(x)]), -(x * x + y * y), rcond=None)[0]
    cx, cy = -solution[0] / 2., -solution[1] / 2.
    return math.sqrt(cx * cx + cy * cy - solution[2]), (cx + centre[0], cy + centre[1])


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def run_commands(plant, seconds, throttle, steer_mrad=0.):
    """Command at 20 Hz (like the harness) and step at 100 Hz."""
    for k in range(int(round(seconds / DT))):
        if k % 5 == 0:
            plant.command_actuation(throttle, steer_mrad)
        plant.step(DT)


# --- A1(F): geometry ----------------------------------------------------------------------------------------------
@pytest.mark.parametrize('speed', [.3, .5])
@pytest.mark.parametrize('delta_deg', [5., 10., 20., -10.])
def test_a1_geometry_check_circle_radius_is_l_over_tan_delta(ideal_params, delta_deg, speed):
    delta = math.radians(delta_deg)
    wheelbase = ideal_params.wheelbase_m
    expected = wheelbase / math.tan(abs(delta))
    plant = SP.SteeredPlant(ideal_params, seed=11, initial_steer_rad=delta)
    duty = duty_for_speed(ideal_params, speed)
    points, turned, last_yaw, step = [], 0., plant.yaw, 0
    while turned < math.radians(150.) and step < 40000:
        if step % 5 == 0:
            plant.command_actuation(duty, delta * 1000.)
        plant.step(DT)
        step += 1
        turned += abs(wrap(plant.yaw - last_yaw))
        last_yaw = plant.yaw
        if abs(plant.v - speed) < 1e-6:
            points.append(plant.rear_axle_pose()[:2])
    radius, _ = fit_circle(points)
    assert len(points) > 300
    assert abs(radius - expected) / expected <= .01                       # A1(F): R error <= 1 %
    assert math.copysign(1., plant.w) == math.copysign(1., delta)         # yaw-rate sign
    assert plant.w == pytest.approx(speed * math.tan(delta) / wheelbase, rel=1e-4)
    assert plant.truth()['delta_eq_rad'] == pytest.approx(delta, abs=1e-12)


def test_default_runs_are_not_ideal_steering_and_drift(profile, default_params):
    """Critique R7: default runs draw eta, scrub loss and understeer and carry wander, slip and zero offset."""
    delta = math.radians(10.)
    ratios = []
    for seed in range(20):
        plant = SP.SteeredPlant(default_params, seed=seed, initial_steer_rad=delta)
        run_commands(plant, 3., 50., delta * 1000.)
        ideal = math.tan(plant.truth()['delta_eq_rad']) / default_params.wheelbase_m
        ratios.append(plant.steer_curvature / ideal)
        eta, k_us = plant.steering_efficiency, plant.understeer_gradient
        expected = eta * default_params.wheelbase_m / (default_params.wheelbase_m + k_us * plant.v ** 2)
        assert ratios[-1] == pytest.approx(expected, rel=1e-9)
    assert max(ratios) <= 1. and min(ratios) >= .84 and statistics.mean(ratios) < .97
    straight = SP.SteeredPlant(default_params, seed=3)
    run_commands(straight, 20., 50., 0.)
    assert abs(straight.y) > .01 and abs(straight.yaw) > 1e-3


def test_geometry_check_uses_ideal_values_and_no_process_noise(ideal_params):
    assert ideal_params.geometry.ackermann_ratio == 1. and ideal_params.geometry_check
    label = ideal_params.profile.result_label()
    assert label['overrides']['label'] == SP.GEOMETRY_CHECK_LABEL and label['field_validated'] is False
    assert label['result_kind'] == vp.RESULT_LABEL_PLACEHOLDER
    plant = SP.SteeredPlant(ideal_params, seed=9)
    assert plant.draws == {'steering_efficiency': 1., 'scrub_speed_loss_per_deg': 0., 'understeer_gradient': 0.,
                           'steer_zero_offset_deg': 0., 'pot_cal_residual_deg': 0.}
    assert plant.speed_run_scale == 1. and plant.wander_deg_per_m == 0. and plant.backend.zero_offset == 0.
    run_commands(plant, 20., 60., 0.)
    assert plant.y == 0. and plant.yaw == 0. and plant.lateral == 0. and plant.v > .5


# --- A2: L0 actuator response ---------------------------------------------------------------------------------------
@pytest.mark.parametrize('moving, rate_name', [(True, 'steer_rate_max_deg_s'), (False, 'steer_rate_standstill_deg_s')])
def test_a2_l0_rate_limit(profile, moving, rate_name):
    backend = l0_backend(profile, steer_zero_offset_deg=ZERO)
    rate = math.radians(profile.value(rate_name))
    tau = profile.value('steer_motor_tau_s')
    target = math.radians(20.)
    angles = [backend.reset(0.).delta_eq_rad]
    for _ in range(300):
        angles.append(servo(backend, target, moving=moving).delta_eq_rad)
    rate_steps = int((target - rate * tau) / rate / DT)                  # steps spent at the rate limit
    measured = (angles[rate_steps] - angles[1]) / ((rate_steps - 1) * DT)
    assert measured == pytest.approx(rate, rel=.05)                        # A2: rate +-5 %
    assert max(b - a for a, b in zip(angles, angles[1:])) <= rate * DT * (1. + 1e-9)
    assert all(a <= target + 1e-12 for a in angles) and angles[-1] == pytest.approx(target, abs=1e-6)


@pytest.mark.parametrize('moving', [True, False])
def test_a2_l0_first_order_lag(profile, moving):
    backend = l0_backend(profile, steer_zero_offset_deg=ZERO)
    tau = profile.value('steer_motor_tau_s')
    size = math.radians(1.)                       # below rate * tau for both rates: no rate limit
    backend.reset(0.)
    angles = [servo(backend, size, moving=moving).delta_eq_rad for _ in range(40)]
    estimates = [-(k + 1) * DT / math.log(1. - a / size) for k, a in enumerate(angles) if a < .98 * size]
    assert len(estimates) >= 20
    for estimate in estimates:
        assert estimate == pytest.approx(tau, rel=.05)                     # A2: tau +-5 %
    assert angles[int(round(tau / DT)) - 1] / size == pytest.approx(1. - math.exp(-1.), rel=1e-9)


def test_a2_l0_end_stop_holds_each_wheel_at_its_mechanical_limit(profile):
    backend = l0_backend(profile, steer_zero_offset_deg=ZERO)
    geometry = geo.SteeringGeometry.from_profile(profile)
    mech_left, mech_right = geometry.mech_limits_eq()
    limits = geometry.wheel_limits()
    backend.reset(0.)
    for _ in range(300):
        out = servo(backend, math.radians(60.))
    assert out.delta_eq_rad == pytest.approx(mech_left, abs=1e-12) and 'END_STOP_LEFT' in out.flags()
    assert out.delta_left_rad == pytest.approx(limits['left_wheel_toward_left'], abs=1e-9)
    assert out.delta_right_rad < limits['right_wheel_toward_left']
    for _ in range(600):
        out = servo(backend, math.radians(-60.))
    assert out.delta_eq_rad == pytest.approx(-mech_right, abs=1e-12) and 'END_STOP_RIGHT' in out.flags()
    assert out.applied_steer_pwm == -1.                                    # still pushing against the stop
    assert out.delta_right_rad == pytest.approx(-limits['right_wheel_toward_right'], abs=1e-9)
    assert -out.delta_left_rad < limits['left_wheel_toward_right']


def test_a2_l0_has_no_deadband_and_no_backlash(profile):
    backend = l0_backend(profile, steer_zero_offset_deg=ZERO)
    tiny = math.radians(profile.value('servo_deadband_deg')) / 10.
    backend.reset(0.)
    for _ in range(300):
        out = servo(backend, tiny)
    assert out.delta_eq_rad == pytest.approx(tiny, rel=1e-9)
    assert out.motor_side_angle_rad == pytest.approx(out.delta_eq_rad, abs=1e-15) and 'IN_BACKLASH' not in out.flags()


def test_l0_enable_off_holds_the_angle(profile):
    backend = l0_backend(profile, steer_zero_offset_deg=ZERO)
    backend.reset(math.radians(5.))
    out = servo(backend, 0., en=False)
    assert out.delta_eq_rad == pytest.approx(math.radians(5.), abs=1e-12)
    assert out.applied_steer_pwm == 0. and out.steer_en is False and 'FREE_WHEELING' in out.flags()
    locked = l0_backend(profile, steer_zero_offset_deg=ZERO, steer_self_locking=True)
    locked.reset(0.)
    assert 'FREE_WHEELING' not in servo(locked, 0., en=False).flags()


def test_l0_tick_rule_and_backend_time(profile):
    backend = l0_backend(profile)
    backend.reset(0.)
    assert [servo(backend, 0.).ticks for _ in range(5)] == [10] * 5
    backend.reset(0.)
    outs = [servo(backend, 0., dt=1. / 120.) for _ in range(12)]
    assert sum(o.ticks for o in outs) == 100 and {o.ticks for o in outs} == {8, 9}
    assert outs[-1].sim_time_s == pytest.approx(.1, abs=1e-12)
    assert backend.reset(0.).ticks == 0


def test_l0_reset_zero_offset_and_errors(profile):
    seed = 5
    backend = l0_backend(profile, seed=seed)
    offset = math.radians(profile.sample(seed, ['steer_zero_offset_deg'])['steer_zero_offset_deg'])
    assert backend.zero_offset == offset != 0.
    out = backend.reset(math.radians(4.))
    assert out.ticks == 0 and out.delta_eq_rad == pytest.approx(math.radians(4.), abs=1e-12)
    for _ in range(200):
        out = servo(backend, 0.)
    assert out.delta_eq_rad == pytest.approx(offset, abs=1e-9)      # the test servo does not know the zero offset
    with pytest.raises(ValueError):
        backend.reset(math.radians(40.))
    with pytest.raises(ValueError):
        L0.L0ServoBackend('direct_pwm')
    with pytest.raises(ValueError):
        backend.step(ba.SteerStepInput(dt_s=DT, mode='direct_pwm', steer_pwm=.5))
    with pytest.raises(RuntimeError):
        L0.L0ServoBackend().reset(0.)
    with pytest.raises(ValueError):
        L0.L0ServoBackend().configure(profile, -1)
    backend.close()
    with pytest.raises(RuntimeError):
        servo(backend, 0.)


def test_l0_link_table_and_left_right_asymmetry(profile):
    backend = l0_backend(profile, steer_zero_offset_deg=ZERO, steer_link_table=[[-40., -44.], [0., 0.], [40., 36.]],
                         steer_lr_asymmetry=.05)
    backend.reset(0.)
    for target_deg, eq_deg, pot_deg in ((10., 10.5, 10. * 40. / 36.), (-10., -9.5, -10. * 40. / 44.)):
        for _ in range(300):
            out = servo(backend, math.radians(target_deg))
        assert math.degrees(out.delta_eq_rad) == pytest.approx(eq_deg, abs=1e-9)
        assert math.degrees(out.motor_side_angle_rad) == pytest.approx(pot_deg, abs=1e-9)


# --- A3(F): L0 potentiometer ------------------------------------------------------------------------------------------
def test_a3_l0_potentiometer_is_error_free_and_lsb_exact(profile):
    prof = no_offset(profile)
    backend = L0.L0ServoBackend()
    backend.configure(prof, 0)
    gain = 2. ** prof.value('adc_bits') * prof.value('pot_to_steer_ratio') / math.radians(prof.value('pot_range_deg'))
    lsb = 1. / gain
    center = prof.value('pot_center_raw')
    assert math.degrees(lsb) == pytest.approx(prof.value('pot_lsb_deg'), rel=1e-12)
    backend.reset(math.radians(3.))
    raws = [servo(backend, math.radians(3.), moving=False).pot_raw for _ in range(10000)]   # 10,000 still samples
    assert len(set(raws)) == 1                                   # L0: no noise (the L1 noise check is native)
    assert raws[0] == L0.round_half_away(center + gain * math.radians(3.))
    for theta_deg in np.linspace(-25., 25., 501):
        theta = math.radians(float(theta_deg))
        out = backend.reset(theta)
        assert out.motor_side_angle_rad == pytest.approx(theta, abs=1e-15)
        assert out.pot_raw == L0.round_half_away(center + gain * out.motor_side_angle_rad)
        assert abs((out.pot_raw - center) * lsb - theta) <= .5 * lsb + 1e-12
    base = backend.reset(math.radians(1.)).pot_raw
    assert backend.reset(math.radians(1.) + lsb).pot_raw == base + 1
    assert backend.reset(math.radians(1.) + 2. * lsb).pot_raw == base + 2


def test_a3_l0_pot_location_and_clipping(profile):
    raws = {}
    for location in ('motor', 'wheel'):
        backend = l0_backend(profile, steer_zero_offset_deg=ZERO, pot_location=location)
        raws[location] = [backend.reset(math.radians(a)).pot_raw for a in (-20., -3., 0., 7., 20.)]
    assert raws['motor'] == raws['wheel']               # L0 has no backlash: both points read the same angle
    high = l0_backend(profile, steer_zero_offset_deg=ZERO, pot_to_steer_ratio=4., pot_range_deg=240.,
                      pot_center_raw=624)
    out = high.reset(math.radians(25.))
    assert out.pot_raw == 1023 and 'POT_CLIPPED' in out.flags()
    assert 'POT_CLIPPED' not in high.reset(0.).flags()
    low = l0_backend(profile, steer_zero_offset_deg=ZERO, pot_to_steer_ratio=4., pot_range_deg=240., pot_center_raw=400)
    out = low.reset(math.radians(-25.))
    assert out.pot_raw == 0 and 'POT_CLIPPED' in out.flags()


def test_a3_steer_status_is_the_nominal_decode_and_carries_no_truth(profile):
    prof = no_offset(profile)
    plant = SP.SteeredPlant.from_profile(prof, seed=2)
    lsb = math.radians(prof.value('pot_lsb_deg'))
    for k in range(1500):
        if k % 5 == 0:
            plant.command_actuation(40., 1000. * math.radians(15.) * math.sin(.01 * k))
        plant.step(DT)
        status = plant.steer_status()
        assert abs(status['measured_rad'] - plant.truth()['delta_eq_rad']) <= .5 * lsb + 1e-12
    assert set(status) == {'stamp_s', 'backend_time_s', 'mode', 'target_rad', 'target_request_mrad', 'target_clamped',
                           'pot_raw', 'measured_rad', 'command_steer_pwm', 'steer_en', 'fault_bits', 'faults'}
    assert not set(status) & set(ba.TRUTH_OUTPUT_FIELDS)                 # F2: no truth field, not even the driver output
    assert status['command_steer_pwm'] is None                           # test_servo: the servo output uses truth
    assert status['fault_bits'] == 0 and status['faults'] == [] and status['mode'] == 'test_servo'


# --- A4(F): drive -----------------------------------------------------------------------------------------------------
@pytest.mark.parametrize('curve, duties, design', [('main', (40., 55., 70., 88.), (.33, .47, .61, .78)),
                                                   ('bracket', (90., 95., 100.), (.72, .81, .90))])
def test_a4_steady_speed_reproduces_the_drive_curve(profile, curve, duties, design):
    params = SP.SteeredPlantParams.from_profile(profile, curve=curve, geometry_check=True)
    for duty, speed in zip(duties, design):
        plant = SP.SteeredPlant(params, seed=1)
        run_commands(plant, 8., duty)
        expected = drive.steady_speed_m_s(params.profile, duty, curve)
        assert plant.v == pytest.approx(expected, rel=.01)               # A4(F): forward_curve +-1 %
        assert plant.v == pytest.approx(speed, rel=.01)                  # design 3.5 numbers
    reverse = SP.SteeredPlant(params, seed=1)
    run_commands(reverse, 8., -duties[0])
    expected = drive.steady_speed_m_s(params.profile, -duties[0], curve)
    assert expected < 0. and reverse.v == pytest.approx(expected, rel=.01)


def test_a4_drive_ramp_equals_the_field_response_mcu_ramp():
    ramp = SP._DriveRamp(100., .12)
    mcu = FR.McuRamp()
    rng = np.random.default_rng(3)
    now = 0.
    for k in range(6000):
        if k % 5 == 0:
            duty = int(rng.choice([-100, -60, -20, 0, 0, 30, 70, 100]))
            ramp.command(duty)
            mcu.command(duty, duty, now)
        now += DT
        applied = ramp.step(now, DT)
        assert (applied, applied) == mcu.step(now, DT)


def test_a4_plant_ramp_slope_and_reversal_dwell(profile):
    params = SP.SteeredPlantParams.from_profile(profile, geometry_check=True)
    assert params.ramp_pct_per_s == pytest.approx(100.) and params.reversal_dwell_s == pytest.approx(.12)
    plant = SP.SteeredPlant(params, seed=0)
    duties = []
    for k in range(250):
        if k % 5 == 0:
            plant.command_actuation(50. if k < 100 else -50., 0.)
        plant.step(DT)
        duties.append(plant.truth()['applied_duty_pct'])
    rising = [b - a for a, b in zip(duties[:40], duties[1:41])]
    assert rising == pytest.approx([1.] * 40)                             # 100 %/s = 255 PWM/s
    zero_at = next(k for k in range(100, 250) if duties[k] == 0.)
    negative_at = next(k for k in range(zero_at, 250) if duties[k] < 0.)
    assert (negative_at - zero_at) * DT == pytest.approx(params.reversal_dwell_s, abs=1.01 * DT)


@pytest.mark.parametrize('curve', ['bracket', 'main'])
def test_a4_release_stop_distance_median_reproduces_the_old_drivetrain(profile, curve):
    params = SP.SteeredPlantParams.from_profile(as_l0(profile), curve=curve)
    distances = []
    for seed in range(400):
        plant = SP.SteeredPlant(params, seed=seed)
        plant.command_actuation(100., 0.)
        for _ in range(400):
            plant.step(DT)
        v0, x0, y0 = plant.v, plant.x, plant.y
        plant.command_actuation(0., 0.)                                    # key release: MCU fade, EN on
        for _ in range(1000):
            plant.step(DT)
            if plant.v == 0.:
                break
        assert plant.v == 0.
        if .90 <= v0 <= .95:                                               # the measured stops started at 0.90-0.95
            distances.append(math.hypot(plant.x - x0, plant.y - y0))
    assert len(distances) >= 80
    assert .56 <= statistics.median(distances) <= .60                      # A4(F): median 0.56-0.60 m


@pytest.mark.parametrize('mode', ['brake', 'coast'])
def test_a4_brake_and_coast_follow_their_designed_distribution(profile, mode):
    param = profile.param('stop_' + mode)
    lo, hi = param['range']['decel_median_m_s2']
    for decel in (lo, param['value']['decel_median_m_s2'], hi):
        prof = profile.with_overrides(dict(L0_LEVEL, **{'stop_' + mode: dict(param['value'], decel_median_m_s2=decel)}),
                                      label='stop')
        params = SP.SteeredPlantParams.from_profile(prof)
        stop = params.stop_mode(mode)
        for throttle in (30., 60., 100.):
            plant = SP.SteeredPlant(params, seed=4)
            run_commands(plant, 4., throttle)
            v0, x0, y0 = plant.v, plant.x, plant.y
            plant.stop(mode)
            steps = 0
            while plant.v != 0. and steps < 2000:
                plant.step(DT)
                steps += 1
            assert plant.truth()['decel_event']['mode'] == mode
            assert math.hypot(plant.x - x0, plant.y - y0) == pytest.approx(stop.distance_m(v0), abs=v0 * DT)
            assert steps * DT == pytest.approx(stop.time_s(v0), abs=2. * DT + SP.SPEED_SNAP_M_S / decel)
            assert plant.drive_en is (mode != 'coast')


def test_a4_coast_reenables_on_the_next_command_and_stop_is_not_a_latch(default_params):
    plant = SP.SteeredPlant(default_params, seed=8)
    run_commands(plant, 3., 50.)
    plant.stop('coast')
    for _ in range(300):
        plant.step(DT)
    assert plant.v == 0. and plant.drive_en is False and plant.truth()['applied_duty_pct'] == 0.
    run_commands(plant, 3., 50.)
    assert plant.drive_en is True and plant.v > .3


# --- A8: determinism ---------------------------------------------------------------------------------------------------
def scripted_run(params, seed, steps=1500, backend=None):
    plant = SP.SteeredPlant(params, seed=seed, backend=backend)
    rows = []
    for k in range(steps):
        t = k * DT
        if k % 5 == 0:
            plant.command_actuation(60. if t < 8. else 0., 1000. * math.radians(12.) * math.sin(.7 * t))
        if k == 1200:
            plant.stop('brake')
        plant.step(DT)
        if k % 5 == 4:
            imu = plant.imu_sample()
            rows.append((plant.x, plant.y, plant.yaw, plant.v, plant.w, imu['gyro_z_rad_s'], imu['accel_m_s2'],
                         plant.steer_status()['pot_raw']))
    return rows, plant


def test_a8_same_seed_same_trajectory_different_seed_different(default_params):
    a, plant_a = scripted_run(default_params, (3, 17, 1))
    b, plant_b = scripted_run(default_params, (3, 17, 1))
    c, plant_c = scripted_run(default_params, (4, 17, 1))
    assert a == b and plant_a.run_info() == plant_b.run_info()
    assert a != c and plant_a.draws != plant_c.draws
    digest = json.dumps([list(row[:6]) + list(row[6]) + [row[7]] for row in a]).encode('ascii')
    assert digest == json.dumps([list(row[:6]) + list(row[6]) + [row[7]] for row in b]).encode('ascii')
    assert scripted_run(default_params, 5, steps=300)[0] == scripted_run(default_params, 5, steps=300)[0]


def test_per_run_draws_inside_the_profile_ranges_and_shared_with_the_backend(profile, default_params):
    seen = {name: set() for name in SP.PER_RUN_NAMES}
    for seed in range(40):
        plant = SP.SteeredPlant(default_params, seed=(seed, 1, 1))
        assert isinstance(plant.draw_seed, int) and 0 <= plant.draw_seed < 2 ** 64
        for name, value in plant.draws.items():
            lo, hi = profile.param(name)['range']
            assert lo <= value <= hi
            seen[name].add(value)
        assert plant.draws == {name: profile.sample(plant.draw_seed, [name])[name] for name in SP.PER_RUN_NAMES}
        assert math.radians(plant.draws['steer_zero_offset_deg']) == plant.backend.zero_offset
        assert .85 <= plant.steering_efficiency <= 1. and 0. <= plant.understeer_gradient <= .03
    assert all(len(values) == 40 for values in seen.values())


# --- A21: limits ---------------------------------------------------------------------------------------------------------
A21_VARIANTS = {
    'default': {},
    'parallel': {'steer_ackermann_ratio': 0.0},
    'ideal_ackermann': {'steer_ackermann_ratio': 1.0},
    'axle_pivot': {'steer_type': 'axle_pivot'},
    'asymmetric_limits': {'steer_mech_limit_deg': {'left_wheel_toward_left': 32., 'left_wheel_toward_right': 22.,
                                                   'right_wheel_toward_left': 24., 'right_wheel_toward_right': 30.}},
    'offset_plus': {'steer_zero_offset_deg': {'dist': 'normal', 'mean': 2.0, 'sd': 0.0}, 'steer_lr_asymmetry': .1},
    'offset_minus_table': {'steer_zero_offset_deg': {'dist': 'normal', 'mean': -2.0, 'sd': 0.0},
                           'steer_lr_asymmetry': -.1, 'steer_link_table': [[-40., -44.], [0., 0.], [40., 36.]]},
}


@pytest.mark.parametrize('variant', sorted(A21_VARIANTS))
def test_a21_targets_clamped_and_wheels_never_pass_their_mechanical_limits(profile, variant):
    overrides = A21_VARIANTS[variant]
    params = SP.SteeredPlantParams.from_profile(profile.with_overrides(dict(L0_LEVEL, **overrides), label='a21'))
    geometry = params.geometry
    limits = geometry.wheel_limits()
    soft_left, soft_right = geometry.soft_limits_eq()
    mech_left, mech_right = geometry.mech_limits_eq()
    rng = np.random.default_rng(21)
    plant = SP.SteeredPlant(params, seed=7)
    clamped = 0
    for k in range(3000):
        if k % 5 == 0:
            request = math.radians(float(rng.choice([-60., 60.])) if k % 50 == 0 else float(rng.uniform(-60., 60.)))
            plant.command_actuation(50. if k < 2000 else 0., 1000. * request)
            status = plant.steer_status()
            assert -soft_right <= status['target_rad'] <= soft_left
            beyond = request > soft_left or request < -soft_right
            assert status['target_clamped'] is beyond
            clamped += beyond
            if beyond:
                assert status['target_rad'] == (soft_left if request > 0 else -soft_right)
        plant.step(DT)
        truth = plant.truth()
        assert -mech_right - 1e-12 <= truth['delta_eq_rad'] <= mech_left + 1e-12
        assert -limits['left_wheel_toward_right'] - 1e-9 <= truth['delta_left_rad'] <= limits['left_wheel_toward_left'] + 1e-9
        assert (-limits['right_wheel_toward_right'] - 1e-9 <= truth['delta_right_rad']
                <= limits['right_wheel_toward_left'] + 1e-9)
    assert clamped > 30


@pytest.mark.parametrize('ratio', [0., .5, 1.])
def test_a21_without_offsets_the_wheels_stay_inside_their_soft_limits(profile, ratio):
    params = SP.SteeredPlantParams.from_profile(no_offset(profile, steer_ackermann_ratio=ratio))
    soft = params.geometry.soft_wheel_limits()
    plant = SP.SteeredPlant(params, seed=1)
    for k in range(2000):
        if k % 5 == 0:
            plant.command_actuation(40., 1000. * math.radians(60. if (k // 200) % 2 else -60.))
        plant.step(DT)
        truth = plant.truth()
        assert -soft['left_wheel_toward_right'] - 1e-9 <= truth['delta_left_rad'] <= soft['left_wheel_toward_left'] + 1e-9
        assert -soft['right_wheel_toward_right'] - 1e-9 <= truth['delta_right_rad'] <= soft['right_wheel_toward_left'] + 1e-9


# --- harness interface ----------------------------------------------------------------------------------------------------
def test_command_refuses_differential_commands(default_params):
    plant = SP.SteeredPlant(default_params)
    with pytest.raises(RuntimeError, match='use the steering actuation hook'):
        plant.command(40, 40)
    with pytest.raises(RuntimeError, match='use the steering actuation hook'):
        plant.command()


def test_plant_offers_the_names_the_harness_reads(default_params):
    plant = SP.SteeredPlant(default_params, pose=(1., 2., .3))
    assert (plant.x, plant.y, plant.yaw) == (1., 2., .3)
    for name in ('x', 'y', 'yaw', 'v', 'w', 'distance_m', 'external_yaw_rate', 'gyro_bias'):
        assert math.isfinite(getattr(plant, name))
    assert plant.forward_blocked is False and plant.pivot_count == 0 and plant.pivot_stalls == 0
    assert plant.pivot_log == []
    imu = plant.imu_sample()
    assert set(imu) == {'stamp_s', 'gyro_z_rad_s', 'accel_m_s2'} and len(imu['accel_m_s2']) == 3
    lx, ly, lyaw = plant.lidar_pose()
    assert (lx, ly, lyaw) == pytest.approx((1. + .18 * math.cos(.3), 2. + .18 * math.sin(.3), .3))
    for bad in (0., .06, -.01, True):
        with pytest.raises(ValueError):
            plant.step(bad)
    with pytest.raises(TypeError):
        SP.SteeredPlant(FR.FieldResponseParams())
    with pytest.raises(ValueError):
        SP.SteeredPlant(default_params, surface='ice')
    for bad in (float('nan'), 'x', None):
        with pytest.raises(ValueError):
            plant.command_actuation(bad, 0.)


def test_disturbances_forward_block_external_yaw_and_gyro_bias(ideal_params):
    plant = SP.SteeredPlant(ideal_params, seed=1)
    run_commands(plant, 3., 60.)
    assert plant.v > .4 and plant.regime == 'drive'
    plant.forward_blocked = True
    run_commands(plant, 1., 60.)
    assert plant.v == 0. and plant.regime == 'blocked'
    plant.forward_blocked = False
    run_commands(plant, 3., 60.)
    assert plant.v > .4
    still = SP.SteeredPlant(ideal_params, seed=2)                         # standing: small vibration, clear mean
    still.gyro_bias = math.radians(1.)                                    # what the harness does for gyro_bias_deg_s
    still.external_yaw_rate = .1
    yaw0, gyro = still.yaw, []
    for k in range(400):
        still.step(DT)
        if k % 5 == 4:
            gyro.append(still.imu_sample()['gyro_z_rad_s'])
    assert still.w == 0. and still.v == 0. and wrap(still.yaw - yaw0) == pytest.approx(.4, abs=1e-9)
    assert statistics.mean(gyro) == pytest.approx(still.gyro_scale * .1 + math.radians(1.), abs=.004)


def test_imu_mount_rotation_and_steering_vibration(profile):
    still = SP.SteeredPlant.from_profile(as_l0(
        profile, label='mount', imu_mount_rpy_deg={'roll': 5., 'pitch': 0., 'yaw': 0., 'yaw_sign': 1}), seed=3)
    samples = []
    for _ in range(2000):
        still.step(DT)
        samples.append(still.imu_sample()['accel_m_s2'])
    mean = tuple(float(v) for v in np.mean(samples, axis=0))
    g = SP.GRAVITY_M_S2
    assert mean == pytest.approx((0., g * math.sin(math.radians(5.)), g * math.cos(math.radians(5.))), abs=.03)
    flipped = SP.SteeredPlant.from_profile(no_offset(profile, imu_mount_rpy_deg={'roll': 0., 'pitch': 0., 'yaw': 0.,
                                                                                  'yaw_sign': -1}), seed=3)
    flipped.external_yaw_rate = .2
    flipped.gyro_bias = 0.
    rates, zs = [], []
    for _ in range(2000):
        flipped.step(DT)
        sample = flipped.imu_sample()
        rates.append(sample['gyro_z_rad_s'])
        zs.append(sample['accel_m_s2'][2])
    assert statistics.mean(zs) == pytest.approx(-g, abs=.03)
    assert statistics.mean(rates) == pytest.approx(-.2 * flipped.gyro_scale, abs=.005)
    shaky = SP.SteeredPlant.from_profile(no_offset(profile, imu_vibration_steer_m_s2=.5), seed=3)
    assert shaky.imu_vibration_rms() == shaky.still_vibration
    shaky.command_actuation(0., 1000. * math.radians(20.))
    shaky.step(DT)
    assert abs(shaky.truth()['applied_steer_pwm']) == pytest.approx(1.)
    assert shaky.imu_vibration_rms() == pytest.approx(math.hypot(shaky.still_vibration, .5))
    base = SP.SteeredPlant.from_profile(no_offset(profile), seed=3)
    base.command_actuation(0., 1000. * math.radians(20.))
    base.step(DT)
    assert base.imu_vibration_rms() == base.still_vibration               # placeholder steering vibration 0


def test_stop_steering_policies(profile):
    prof = no_offset(profile)
    delta = math.radians(10.)

    def settled():
        plant = SP.SteeredPlant.from_profile(prof, seed=6, initial_steer_rad=delta)
        run_commands(plant, 3., 40., delta * 1000.)
        return plant
    off = settled()
    off.stop('brake', steer='off')
    for _ in range(100):
        off.step(DT)
    assert off.truth()['delta_eq_rad'] == pytest.approx(delta, abs=1e-9) and off.steer_status()['steer_en'] is False
    centre = settled()
    centre.stop('brake', steer='center')
    for _ in range(300):
        centre.step(DT)
    assert centre.truth()['delta_eq_rad'] == pytest.approx(0., abs=1e-6)
    hold = settled()
    held = hold.measured_hold_rad()
    hold.stop('brake')                                                     # profile stop_policy_steer: hold_measured
    for _ in range(300):
        hold.step(DT)
    assert prof.value('stop_policy_steer') == 'hold_measured'
    # decision R13-2: the target is the MCU-side measurement; the L0 potentiometer is error free but quantised, so the
    # wheel goes to the decoded ADC count (within half an LSB of where it was), not exactly back to the true angle
    lsb = 1. / hold.params.pot_counts_per_rad
    assert hold.steer_target_rad == held and abs(held - delta) <= lsb / 2 + 1e-12
    assert hold.truth()['delta_eq_rad'] == pytest.approx(held, abs=1e-9) and hold.v == 0.
    with pytest.raises(ValueError):
        hold.stop('anchor')
    with pytest.raises(ValueError):
        hold.stop('brake', steer='wiggle')


def _nominal_decode(params, raw):
    """The nominal potentiometer decode written out again (protocol section 7 with every error term zero)."""
    return params.geometry.link_table.wheel_from_pot((raw - params.pot_center_raw) / params.pot_counts_per_rad)


def _record_pot(plant, seconds, throttle, steer_mrad):
    """run_commands() that also records the ADC sample the plant got in every step (steer_status pot_raw)."""
    samples = []
    for k in range(int(round(seconds / DT))):
        if k % 5 == 0:
            plant.command_actuation(throttle, steer_mrad)
        plant.step(DT)
        samples.append(plant.steer_status()['pot_raw'])
    return samples


@pytest.mark.parametrize('level', ['L0', 'L1'])
def test_hold_measured_target_is_the_mcu_filtered_measurement_not_the_servo_view(profile, level):
    """Decision R13-2 (review R9 of WF-2b): the hold_measured stop target is the MCU-side filtered potentiometer
    measurement (median of the last pot_filter.median_of samples, nominal decode), never the truth-side servo view.
    Before the fix the target was servo_view_rad() exactly (made from the true motor-side angle)."""
    if level == 'L1':
        try:
            from steering.native import build as native_build
            native_build.find_compiler()
        except Exception as exc:
            pytest.skip('native steering backend not buildable here: %s' % exc)
        prof = profile
    else:
        prof = as_l0(profile, label='hold_l0')
    plant = SP.SteeredPlant.from_profile(prof, seed=8, initial_steer_rad=math.radians(6.))
    try:
        samples = _record_pot(plant, 3., 100., math.radians(6.) * 1000.)   # full duty: supply sag on the L1 pot
        n = plant.params.pot_median_of
        assert n == profile.value('pot_filter')['median_of']
        expected = plant.params.clamp_steer_target(_nominal_decode(plant.params, sorted(samples[-n:])[n // 2]))
        view = plant.servo_view_rad()
        plant.stop('brake', steer='hold_measured')
        assert plant.steer_target_rad == pytest.approx(expected, abs=1e-15)
        assert plant.steer_status()['target_rad'] == plant.steer_target_rad          # the MCU-side value
        if level == 'L1':
            # the L1 pot carries noise and supply sag: the measurement is not the servo view (before: equal)
            assert abs(plant.steer_target_rad - view) > 1e-4
        else:
            # L0: error free, only the ADC quantisation separates them
            assert abs(plant.steer_target_rad - view) <= .5 / plant.params.pot_counts_per_rad + 1e-12
        for _ in range(100):
            plant.step(DT)
        assert plant.steer_target_rad == pytest.approx(expected, abs=1e-15)            # the stop target stays
    finally:
        plant.close()


def test_measured_hold_median_and_start_refill(profile):
    """measured_hold_rad(): the median of the last median_of plant-step samples; before median_of steps the first
    sample fills the buffer (the MCU refills its median buffer with the first sample at boot)."""
    class Scripted(L0.L0ServoBackend):
        def __init__(self, raws):
            super().__init__()
            self.raws = list(raws)

        def reset(self, delta_eq_rad=0.):
            out = super().reset(delta_eq_rad)
            return _replace_raw(out, self.raws.pop(0))

        def step(self, inp):
            return _replace_raw(super().step(inp), self.raws.pop(0))

    def _replace_raw(out, raw):
        from dataclasses import replace
        return replace(out, pot_raw=raw)

    raws = [512, 600, 520, 530, 700, 540, 510, 512]     # the first one is the reset inside L0 configure()
    plant = SP.SteeredPlant.from_profile(as_l0(profile, label='scripted'), seed=1, backend=Scripted(raws))
    p = plant.params
    assert p.pot_median_of == 5
    assert plant.measured_hold_rad() == _nominal_decode(p, 600)                # [600] * 5
    plant.step(DT)                                                             # [600, 600, 600, 600, 520]
    assert plant.measured_hold_rad() == _nominal_decode(p, 600)
    plant.step(DT)                                                             # [600, 600, 600, 520, 530]
    assert plant.measured_hold_rad() == _nominal_decode(p, 600)
    plant.step(DT)                                                             # [600, 600, 520, 530, 700]
    assert plant.measured_hold_rad() == _nominal_decode(p, 600)
    plant.step(DT)                                                             # [600, 520, 530, 700, 540]
    assert plant.measured_hold_rad() == _nominal_decode(p, 540)
    plant.step(DT)                                                             # [520, 530, 700, 540, 510]
    assert plant.measured_hold_rad() == _nominal_decode(p, 530)
    plant.step(DT)                                                             # [530, 700, 540, 510, 512]
    assert plant.measured_hold_rad() == _nominal_decode(p, 530)
    plant.stop('brake', steer='hold_measured')
    assert plant.steer_target_rad == p.clamp_steer_target(_nominal_decode(p, 530))
    from dataclasses import replace
    for bad in (0, 10, 2.):
        with pytest.raises(ValueError):
            replace(p, pot_median_of=bad)


@pytest.mark.parametrize('location', ['motor', 'wheel'])
def test_start_hold_with_turned_wheels_and_servo_view(profile, location):
    """start_wheels_turned (design 5.3): the wheels keep their start angle until the first steering command."""
    prof = as_l0(profile, label='hold', pot_location=location, steer_lr_asymmetry=.05,
                 steer_zero_offset_deg={'dist': 'normal', 'mean': 1.0, 'sd': 0.0})
    plant = SP.SteeredPlant.from_profile(prof, seed=4, initial_steer_rad=math.radians(10.))
    view = plant.servo_view_rad()
    assert view == pytest.approx(plant.params.geometry.link_table.wheel_from_pot(
        plant.truth()['motor_side_angle_rad']), abs=1e-12)                  # L0: motor side = link side
    assert view == pytest.approx(math.radians(9.) / 1.05, abs=1e-12)
    for _ in range(100):
        plant.step(DT)
    assert plant.truth()['delta_eq_rad'] == pytest.approx(math.radians(10.), abs=1e-12)
    plant.command_actuation(0., 0.)
    for _ in range(300):
        plant.step(DT)
    assert plant.truth()['delta_eq_rad'] == pytest.approx(math.radians(1.), abs=1e-9)   # zero offset remains


# --- any SteeringBackend ---------------------------------------------------------------------------------------------------
class _RecordingBackend(L0.L0ServoBackend):
    """L0 that records what the plant passes (a test double around the L0 backend)."""

    def __init__(self):
        super().__init__()
        self.calls = []

    def configure(self, profile, seed):
        self.calls.append(('configure', profile, seed))
        super().configure(profile, seed)

    def reset(self, delta_eq_rad=0.):
        self.calls.append(('reset', delta_eq_rad))
        return super().reset(delta_eq_rad)

    def step(self, inp):
        self.calls.append(('step', inp))
        return super().step(inp)

    def close(self):
        self.calls.append(('close',))
        super().close()


class _FixedAngleBackend(ba.SteeringBackend):
    """Holds the wheels at one equivalent angle whatever the command (a test double, not a model).
    mode direct_pwm: the angle moves at pwm x 0.5 rad/s instead."""

    def __init__(self, delta, mode='test_servo'):
        self.delta, self.mode, self.inputs, self.ticks_total = delta, mode, [], 0

    def configure(self, profile, seed):
        self.geometry = geo.SteeringGeometry.from_profile(profile)

    def reset(self, delta_eq_rad=0.):
        self.ticks_total = 0
        return self._out(0)

    def step(self, inp):
        inp.validate()
        if inp.mode != self.mode:
            raise ValueError('mode')
        self.inputs.append(inp)
        if self.mode == 'direct_pwm':
            self.delta += inp.steer_pwm * .5 * inp.dt_s
        ticks = int(round(inp.dt_s * 1000.))
        self.ticks_total += ticks
        return self._out(ticks)

    def close(self):
        pass

    def _out(self, ticks):
        left, right = self.geometry.wheel_angles(self.delta)
        return ba.SteerStepOutput(sim_time_s=self.ticks_total / 1000., ticks=ticks, delta_left_rad=left,
                                  delta_right_rad=right, delta_eq_rad=self.delta, motor_side_angle_rad=self.delta,
                                  pot_raw=512, applied_steer_pwm=0., steer_en=True)


def test_plant_passes_the_contract_inputs_to_the_backend(default_params):
    backend = _RecordingBackend()
    plant = SP.SteeredPlant(default_params, seed=(1, 2, 3), backend=backend, initial_steer_rad=math.radians(4.))
    assert backend.calls[0] == ('configure', default_params.profile, plant.draw_seed)
    assert backend.calls[-1] == ('reset', math.radians(4.))
    run_commands(plant, 2., 60., 1000. * math.radians(30.))                # beyond the soft limit
    steps = [call[1] for call in backend.calls if call[0] == 'step']
    assert len(steps) == 200
    soft_left = default_params.geometry.soft_limits_eq()[0]
    assert all(s.mode == 'test_servo' and s.dt_s == DT and s.steer_en and s.target_delta_eq_rad == soft_left
               and s.steer_pwm == 0. for s in steps)
    assert steps[0].vehicle_stopped and steps[0].vehicle_speed_m_s == 0. and not steps[-1].vehicle_stopped
    assert steps[-1].supply_scale == pytest.approx(1. - default_params.pot_supply_sag_per_pwm * .6)
    plant.close()
    assert backend.calls[-1] == ('close',)


def test_plant_integrates_whatever_angle_the_backend_reports(ideal_params):
    delta = math.radians(12.)
    plant = SP.SteeredPlant(ideal_params, seed=0, backend=_FixedAngleBackend(delta))
    points = []
    for k in range(3000):
        if k % 5 == 0:
            plant.command_actuation(50., 0.)                               # the backend ignores the target
        plant.step(DT)
        if k > 600:
            points.append(plant.rear_axle_pose()[:2])
    radius, _ = fit_circle(points)
    assert radius == pytest.approx(ideal_params.wheelbase_m / math.tan(delta), rel=1e-3)


def test_direct_pwm_mode(profile):
    params = SP.SteeredPlantParams.from_profile(profile, geometry_check=True, steer_mode='direct_pwm')
    backend = _FixedAngleBackend(0., mode='direct_pwm')
    plant = SP.SteeredPlant(params, seed=0, backend=backend)
    plant.command_drive(40.)
    plant.command_steer_pwm(.5)
    for _ in range(100):
        plant.step(DT)
    assert all(i.mode == 'direct_pwm' and i.steer_pwm == .5 and i.target_delta_eq_rad == 0. for i in backend.inputs)
    assert plant.truth()['delta_eq_rad'] == pytest.approx(.25)
    with pytest.raises(RuntimeError):
        plant.command_actuation(40., 0.)
    with pytest.raises(ValueError):
        plant.command_steer_pwm(1.5)
    with pytest.raises(ValueError):
        SP.SteeredPlant(params, seed=0).step(DT)                           # the default L0 refuses direct_pwm
    with pytest.raises(RuntimeError, match='direct_pwm'):
        SP.SteeredPlant(SP.SteeredPlantParams.from_profile(as_l0(profile))).command_steer_pwm(.1)
    with pytest.raises(ValueError):
        SP.SteeredPlantParams.from_profile(profile, steer_mode='firmware')


# --- _KinematicBicycle contract stand-in (fast-SIM survey section 10 item 3) ---------------------------------------------
class _KinematicBicycle:
    """Kinematic stand-in for steering contract tests: the steered counterpart of _Kinematic in
    closed_loop/test_single_row.py (not the SIM plant; no noise, no MCU ramp, no drive curve above 90 %).

    Rear-axle reference: x along the row, y lateral (left +), psi heading (CCW +). command = (throttle_pct,
    steer_target_rad). v -> slope x (throttle - deadband) with lag tau_v; delta -> target with a first-order lag
    tau_delta under a rate limit (rate_standstill when v < 0.01 m/s); omega = v tan(delta) / L. Explicit Euler in
    1 ms sub-steps."""

    def __init__(self, wheelbase=.76, slope=.8 / 85., deadband=5., tau_v=.25, tau_delta=.08, rate_moving_deg_s=30.,
                 rate_standstill_deg_s=15., y=0., psi_deg=0.):
        self.wheelbase, self.slope, self.deadband, self.tau_v, self.tau_delta = wheelbase, slope, deadband, tau_v, \
            tau_delta
        self.rate_moving, self.rate_standstill = math.radians(rate_moving_deg_s), math.radians(rate_standstill_deg_s)
        self.x, self.y, self.psi, self.v, self.delta = 0., y, math.radians(psi_deg), 0., 0.
        self.command = (0., 0.)

    def step(self, dt):
        throttle, target = self.command
        substeps = max(1, int(round(dt / .001)))
        h = dt / substeps
        for _ in range(substeps):
            rate = self.rate_standstill if abs(self.v) < .01 else self.rate_moving
            self.delta += max(-rate, min(rate, (target - self.delta) / self.tau_delta)) * h
            v_target = max(0., self.slope * (throttle - self.deadband))
            self.v += (v_target - self.v) * (1. - math.exp(-h / self.tau_v))
            self.psi += self.v * math.tan(self.delta) / self.wheelbase * h
            self.x += self.v * math.cos(self.psi) * h
            self.y += self.v * math.sin(self.psi) * h

    def snapshot(self):
        return dict(lateral_offset=self.y, heading_error=-self.psi, speed=self.v, steer_angle=self.delta)


def pursuit_delta(y, psi, lookahead=2., wheelbase=.76, limit_deg=20.):
    """Pure pursuit toward the centreline y = 0 from the rear axle (truth state; test controller only)."""
    dx, dy = lookahead, -y
    px = dx * math.cos(psi) + dy * math.sin(psi)
    py = -dx * math.sin(psi) + dy * math.cos(psi)
    kappa = 2. * py / (px * px + py * py)
    limit = math.radians(limit_deg)
    return max(-limit, min(limit, math.atan(wheelbase * kappa)))


def test_kinematic_bicycle_contract_stand_in_matches_the_plant(ideal_params):
    """Regression: under the same closed loop the noise-free plant and the contract stand-in agree, so contract
    tests written against _KinematicBicycle transfer to SteeredPlant (geometry_check, L0)."""
    stand_in = _KinematicBicycle(wheelbase=ideal_params.wheelbase_m, y=.3)
    plant = SP.SteeredPlant(ideal_params, seed=0, pose=(0., .3, 0.))
    lateral = []
    for k in range(680):                                                   # 34 s at 20 Hz: 4 s straight, then pursuit
        rear = plant.rear_axle_pose()
        if k < 80:
            commands = (0., 0.)
        else:
            commands = (pursuit_delta(stand_in.y, stand_in.psi), pursuit_delta(rear[1], rear[2]))
        stand_in.command = (40., commands[0])
        plant.command_actuation(40., 1000. * commands[1])
        stand_in.step(.05)
        for _ in range(5):
            plant.step(DT)
        lateral.append((stand_in.y, plant.rear_axle_pose()[1]))
    assert stand_in.v == pytest.approx(plant.v, rel=1e-6)                  # same steady speed (40 %)
    assert max(abs(a - b) for a, b in lateral) < .01
    assert max(abs(a) for a, _ in lateral[-40:]) < .02 and max(abs(b) for _, b in lateral[-40:]) < .02
    assert min(a for a, _ in lateral) == pytest.approx(min(b for _, b in lateral), abs=.01)   # same overshoot


# --- comparison with the survey probe bicycle --------------------------------------------------------------------------
def probe_bicycle_step(state, cmd_delta, speed, dt, wheelbase, rate, tau, deadband=0., max_steer=math.radians(30.)):
    """One 'bicycle' step copied from the survey probe (steering-sim-20261108/survey/original_ackermann_probe/
    probe_pure_pursuit_bicycle.py, run()): Euler first-order lag + rate limit + deadband on the steering angle, then
    semi-implicit Euler on the rear axle. Only the formula is copied; the file is outside the repository."""
    x, y, yaw, delta = state
    err = cmd_delta - delta
    if abs(err) > deadband:
        target = cmd_delta if tau <= 0 else delta + err * min(1.0, dt / tau)
        step = max(-rate * dt, min(rate * dt, target - delta))
        delta = max(-max_steer, min(max_steer, delta + step))
    yaw += speed * math.tan(delta) / wheelbase * dt
    x += speed * math.cos(yaw) * dt
    y += speed * math.sin(yaw) * dt
    return x, y, yaw, delta


def test_plant_agrees_with_the_survey_probe_bicycle(profile, ideal_params):
    rate = math.radians(profile.value('steer_rate_max_deg_s'))
    tau = profile.value('steer_motor_tau_s')
    plant = SP.SteeredPlant(ideal_params, seed=0)
    run_commands(plant, 3., 45.)                                           # get rolling straight first
    reference = plant.rear_axle_pose() + (plant.truth()['delta_eq_rad'],)
    delta_gap = 0.
    for k in range(1600):                                                  # 16 s steering script
        t = k * DT
        if t < 3.:
            command = math.radians(10.)
        elif t < 6.:
            command = math.radians(-12.)
        else:
            command = math.radians(6.) * math.sin(2. * math.pi * .3 * (t - 6.))
        if k % 5 == 0:
            plant.command_actuation(45., 1000. * command)
            held = command
        plant.step(DT)
        reference = probe_bicycle_step(reference, held, plant.v, DT, ideal_params.wheelbase_m, rate, tau)
        delta_gap = max(delta_gap, abs(plant.truth()['delta_eq_rad'] - reference[3]))
    x, y, yaw = plant.rear_axle_pose()
    assert delta_gap < math.radians(.15)
    assert math.hypot(x - reference[0], y - reference[1]) < .02
    assert abs(wrap(yaw - reference[2])) < math.radians(.3)


# --- harness hooks (smoke) -----------------------------------------------------------------------------------------------
class _SteerCommand:
    def __init__(self, throttle_pct, steer_target_mrad):
        self.throttle_pct = throttle_pct
        self.steer_target_mrad = steer_target_mrad


class _ScriptedController:
    """controller_factory stand-in for the smoke test: command k of a schedule at the k-th controller step."""

    def __init__(self, schedule):
        self.schedule = schedule
        self.state, self.reason, self.hold_terminal = 'FOLLOW', None, False
        self.events = []
        self.counts = dict(stops=0, turns=0, stops_without_turn=0, stalls=0)
        self.issued = 0

    def step(self, now, snapshot):
        command = _SteerCommand(*self.schedule(self.issued))
        self.issued += 1
        return command


def _harness_or_skip():
    try:
        from closed_loop import harness
    except ImportError as exc:     # e.g. a broken scipy (the estimator imports scipy.sparse)
        pytest.skip('closed_loop.harness cannot be imported here: %s: %s' % (type(exc).__name__, exc))
    params = inspect.signature(harness.run_episode).parameters
    if 'plant_factory' not in params or 'actuation' not in params:
        pytest.skip('closed_loop.harness.run_episode has no plant_factory / actuation hooks yet')
    return harness


def test_steering_actuation_contract():
    actuation = SP.SteeringActuation()
    assert actuation.initial_values() == (0, 0)
    values = actuation.values(_SteerCommand(35, -120))
    assert values == (35, -120)
    assert actuation.trace_fields(values) == dict(throttle_pct=35, steer_target_mrad=-120)
    assert actuation.is_reversal((20, 0), (-5, 0)) and not actuation.is_reversal((20, 50), (0, -50))
    assert not hasattr(actuation, 'turn_sign')

    class Recorder:
        def __init__(self):
            self.calls = []

        def command_actuation(self, throttle_pct, steer_target_mrad):
            self.calls.append((throttle_pct, steer_target_mrad))
    plant = Recorder()
    actuation.apply(plant, values)
    assert plant.calls == [(35, -120)]


def test_steered_plant_factory(profile):
    harness = _harness_or_skip()
    factory = SP.steered_plant_factory(as_l0(profile))
    config = harness.EpisodeConfig(name='factory', gyro_noise_scale=2.)
    plant = factory(harness.plant_params(config), surface='normal', seed=(0, 1, 1), pose=(1., 2., .1), config=config)
    assert isinstance(plant, SP.SteeredPlant) and (plant.x, plant.y, plant.yaw) == (1., 2., .1)
    assert plant.params.gyro_still_noise_sd_deg_s == pytest.approx(2. * factory.params.gyro_still_noise_sd_deg_s)
    assert plant.params.gyro_noise_scale == 2.
    bad = harness.EpisodeConfig(name='factory', plant_overrides=(('lateral_slip_sd_m_s', .05),))
    with pytest.raises(ValueError, match='plant_overrides'):
        factory(harness.plant_params(bad), surface='normal', seed=0, pose=(0., 0., 0.), config=bad)


def test_harness_smoke_with_the_steering_actuation(profile):
    harness = _harness_or_skip()
    from closed_loop.world import WorldConfig   # imported by the harness already
    base = SP.steered_plant_factory(as_l0(profile))

    def episode():
        holder = {}

        def plant_factory(params, *, surface, seed, pose, config):
            holder['plant'] = base(params, surface=surface, seed=seed, pose=pose, config=config)
            return holder['plant']
        config = harness.EpisodeConfig(name='steered_smoke', mode='continuous', world=WorldConfig(row_length_m=12.),
                                       max_time_s=4.)
        result = harness.run_episode(config, record_trace=True, plant_factory=plant_factory,
                                     controller_factory=lambda _c: _ScriptedController(
                                         lambda k: (45., round(50. * math.sin(.2 * k), 3))),
                                     actuation=SP.SteeringActuation())
        return result, holder['plant']
    result, plant = episode()
    assert result['end'] == 'time_limit' and result['collisions'] == 0
    assert result['plant_pivots'] == 0 and result['plant_pivot_stalls'] == 0 and result['plant_pivot_rates'] == []
    assert result['chatter_flips'] is None and result['reversal_commands'] == 0
    assert result['truth_distance_m'] > .5 and plant.v > .3
    sample = result['trace'][-1]
    assert 'throttle_pct' in sample and 'steer_target_mrad' in sample and 'left' not in sample
    assert plant.steer_request_mrad is not None
    again, _ = episode()                                                   # A8 at the harness level
    assert json.dumps(again, sort_keys=True, default=str) == json.dumps(result, sort_keys=True, default=str)


# --- native backend (optional) -------------------------------------------------------------------------------------------
@pytest.fixture(scope='module')
def native_backend_factory(tmp_path_factory):
    """A callable returning a new native backend (test_servo), or skip while it is not available.

    The native module is written in parallel (steering.actuator_link). Its build goes to a pytest temporary
    folder, never into the work tree."""
    try:
        module = importlib.import_module('steering.actuator_link')
    except Exception as exc:  # any import problem means "not available yet"
        pytest.skip('native steering backend (steering.actuator_link) not available: %s: %s'
                    % (type(exc).__name__, exc))
    classes = [obj for obj in vars(module).values()
               if inspect.isclass(obj) and issubclass(obj, ba.SteeringBackend) and obj is not ba.SteeringBackend
               and not inspect.isabstract(obj) and obj.__module__ == module.__name__]
    if not classes:
        pytest.skip('steering.actuator_link defines no concrete SteeringBackend yet')
    cls = classes[0]
    accepted = inspect.signature(cls).parameters
    if 'mode' not in accepted or 'build_dir' not in accepted:
        pytest.skip('%s takes no mode / build_dir keyword; cannot build it outside the work tree' % cls.__name__)
    build_dir = str(tmp_path_factory.mktemp('steer_native_build'))

    def make():
        return cls(mode='test_servo', build_dir=build_dir)
    try:
        probe = make()
        try:
            probe.configure(vp.load_profile(), 0)         # builds once (a missing compiler raises here)
        finally:
            probe.close()
    except (OSError, RuntimeError, ValueError) as exc:
        pytest.skip('native backend %s cannot build or start here: %s: %s' % (cls.__name__, type(exc).__name__, exc))
    return make


def native_plant(params, factory, seed):
    backend = factory()
    try:
        return SP.SteeredPlant(params, seed=seed, backend=backend)
    except (OSError, RuntimeError) as exc:
        backend.close()
        pytest.skip('native backend not usable here: %s' % exc)


def test_native_backend_drives_the_plant_when_available(profile, native_backend_factory):
    params = SP.SteeredPlantParams.from_profile(profile)
    limits = params.geometry.wheel_limits()
    runs = []
    for _ in range(2):                                                     # A8 with the native backend: same seed twice
        rows = []
        with native_plant(params, native_backend_factory, (1, 2, 3)) as plant:
            for k in range(600):
                if k % 5 == 0:
                    plant.command_actuation(50., 1000. * math.radians(8. if k < 300 else -8.))
                plant.step(DT)
                truth = plant.truth()
                assert truth['delta_left_rad'] <= limits['left_wheel_toward_left'] + 1e-9
                assert truth['delta_right_rad'] >= -limits['right_wheel_toward_right'] - 1e-9
                assert -limits['left_wheel_toward_right'] - 1e-9 <= truth['delta_left_rad']
                assert truth['delta_right_rad'] <= limits['right_wheel_toward_left'] + 1e-9
                rows.append((plant.x, plant.y, plant.yaw, plant.v, truth['delta_eq_rad'], plant.steer_status()['pot_raw']))
            assert plant.x > 1. and plant.truth()['delta_eq_rad'] < 0.
            assert plant.run_info()['backend']['cls'] != 'L0ServoBackend'
        runs.append(rows)
    assert runs[0] == runs[1]


def test_python_l0_matches_the_native_l0_level(profile, native_backend_factory):
    """Same contract semantics in both backends: native level L0 with backlash 0 and an ideal potentiometer
    against l0_servo (the native integrates per 1 ms tick, l0_servo exactly per step)."""
    prof = profile.with_overrides({'steer_actuator_level': 'L0', 'steer_backlash_deg': 0.0,
                                   'steer_backlash_wander': {'sd_deg': 0.0, 'corr_length_m': 5.0}}, label='l0_compare')
    prof = prof.with_overrides({'pot_noise_sd_counts': 0.0, 'pot_supply_sag_per_pwm': 0.0}, label='ideal_pot',
                               allow_out_of_range=True)
    for seed in (0, 7):
        python_l0 = L0.L0ServoBackend()
        python_l0.configure(prof, seed)
        native = native_backend_factory()
        try:
            native.configure(prof, seed)
            pairs = [(python_l0.reset(math.radians(3.)), native.reset(math.radians(3.)))]
            for k in range(1200):
                t = k * DT
                target = math.radians(20. if t < 2. else (-15. if t < 4. else 6. * math.sin(2. * math.pi * .4 * t)))
                inp = ba.SteerStepInput(dt_s=DT, mode='test_servo', target_delta_eq_rad=target, steer_en=True,
                                        vehicle_speed_m_s=.4 if k > 300 else 0., vehicle_stopped=k <= 300)
                pairs.append((python_l0.step(inp), native.step(inp)))
        finally:
            native.close()
        for a, b in pairs:
            assert (a.ticks, a.sim_time_s, a.pot_raw, a.model_flags) == (b.ticks, b.sim_time_s, b.pot_raw, b.model_flags)
            for name in ('delta_eq_rad', 'delta_left_rad', 'delta_right_rad', 'motor_side_angle_rad'):
                assert abs(getattr(a, name) - getattr(b, name)) < 1e-6, name
            assert abs(a.applied_steer_pwm - b.applied_steer_pwm) < .02


# --- default backend follows the profile level (F8) ---------------------------------------------------------------------
def _native_or_skip():
    try:
        from steering.native import build as native_build
        native_build.find_compiler()
    except Exception as exc:  # no compiler, no native backend
        pytest.skip('native steering backend not buildable here: %s' % exc)


def test_default_backend_is_l0_for_an_l0_profile(profile):
    with warnings.catch_warnings():
        warnings.simplefilter('error', SP.SteeringBackendLevelWarning)
        plant = SP.SteeredPlant(SP.SteeredPlantParams.from_profile(as_l0(profile)), seed=1)
    assert isinstance(plant.backend, L0.L0ServoBackend)
    label = plant.result_label()
    assert label['actuator_backend'] == dict(cls='L0ServoBackend', level='L0', profile_level='L0', matches_profile=True)
    assert plant.run_info()['profile'] == label and plant.run_info()['backend']['level'] == 'L0'


def test_geometry_check_runs_on_the_l0_servo(profile):
    """Design 3.3: A1 and A12 use L0; geometry_check sets the level, so the plant label says so."""
    params = SP.SteeredPlantParams.from_profile(profile, geometry_check=True)
    assert params.profile.value('steer_actuator_level') == 'L0'
    assert 'steer_actuator_level' in params.profile.result_label()['overrides']['names']
    plant = SP.SteeredPlant(params, seed=0)
    assert isinstance(plant.backend, L0.L0ServoBackend) and plant.result_label()['actuator_backend']['matches_profile']


def test_default_backend_follows_the_shipped_profile_level_l1(profile):
    """F8: the shipped profile says L1 (design 3.3 default), so the default plant runs the native L1 backend."""
    _native_or_skip()
    assert profile.value('steer_actuator_level') == 'L1'
    with warnings.catch_warnings():
        warnings.simplefilter('error', SP.SteeringBackendLevelWarning)
        plant = SP.SteeredPlant(SP.SteeredPlantParams.from_profile(profile), seed=(2, 3, 4))
    try:
        assert type(plant.backend).__name__ == 'NativeSteeringBackend' and plant.backend.running
        assert plant.result_label()['actuator_backend'] == dict(cls='NativeSteeringBackend', level='L1',
                                                               profile_level='L1', matches_profile=True)
        run_commands(plant, 3., 50., 1000. * math.radians(6.))
        assert plant.v > .3 and plant.truth()['delta_eq_rad'] > math.radians(3.)
    finally:
        plant.close()
    assert not plant.backend.running


def test_factory_default_backend_is_native_for_the_shipped_profile(profile):
    _native_or_skip()
    factory = SP.steered_plant_factory(profile)
    plant = factory(None, surface='normal', seed=(0, 1, 1), pose=(0., 0., 0.))
    try:
        assert type(plant.backend).__name__ == 'NativeSteeringBackend'
    finally:
        plant.close()


def test_a_backend_of_another_level_warns_and_is_labelled(profile):
    params = SP.SteeredPlantParams.from_profile(profile)                     # shipped profile: L1
    with pytest.warns(SP.SteeringBackendLevelWarning, match='models level L0 but the profile steer_actuator_level is L1'):
        plant = SP.SteeredPlant(params, seed=1, backend=L0.L0ServoBackend())
    assert plant.result_label()['actuator_backend'] == dict(cls='L0ServoBackend', level='L0', profile_level='L1',
                                                           matches_profile=False)
    # a test double that does not report its level gives no warning and matches_profile None
    with warnings.catch_warnings():
        warnings.simplefilter('error', SP.SteeringBackendLevelWarning)
        double = SP.SteeredPlant(params, seed=1, backend=_FixedAngleBackend(0.))
    assert double.result_label()['actuator_backend']['matches_profile'] is None


def test_l1b_profile_has_no_backend_yet(profile):
    with pytest.raises(ValueError, match='L1b'):
        SP.SteeredPlant(SP.SteeredPlantParams.from_profile(profile.with_overrides({'steer_actuator_level': 'L1b'},
                                                                                  label='l1b')), seed=0)


# --- steer_status carries no truth (F2) --------------------------------------------------------------------------------
@pytest.mark.parametrize('kind, params, applied', [('driver_dead', {}, 0.), ('driver_stuck_on', {'pwm': -.7}, -.7)])
def test_steer_status_reports_the_command_not_the_faulty_driver_output(profile, kind, params, applied):
    """F2: with a dead or stuck driver the MCU still only knows its command; the actual output is truth only."""
    _native_or_skip()
    from steering.actuator_link import NativeSteeringBackend
    prof = SP.SteeredPlantParams.from_profile(profile, steer_mode='direct_pwm')
    backend = NativeSteeringBackend(mode='direct_pwm', injections=[dict(kind=kind, at_s=1., params=params)])
    with SP.SteeredPlant(prof, seed=3, backend=backend) as plant:
        plant.command_drive(40.)
        plant.command_steer_pwm(.5)
        before = after = None
        for k in range(150):
            plant.step(DT)
            if k == 50:
                before = (plant.steer_status(), plant.truth())
        after = (plant.steer_status(), plant.truth())
    for status, truth in (before, after):
        assert status['command_steer_pwm'] == .5 and status['steer_en'] is True
        assert status['fault_bits'] == 0 and not set(status) & set(ba.TRUTH_OUTPUT_FIELDS)
    assert before[1]['applied_steer_pwm'] == .5 and after[1]['applied_steer_pwm'] == applied
