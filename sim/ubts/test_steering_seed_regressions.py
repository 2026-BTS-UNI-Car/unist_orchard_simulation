"""Seed regression tests of the steering SIM (design 8.3 P1 third seed; decision R12 "upstream pure-function tests").

1. The curvature parts of the upstream pure-function tests (ros2_ws/src/orchard_navigation/test/test_controller.py
   on orchard_navigation/controller.py; both read only, never imported as a package and never changed) as
   regressions against the steering geometry and the steered plant:
     - fast_sim.pure_pursuit_curvature is the upstream pure_pursuit_curvature (same formula, same numbers);
     - upstream test_turns_toward_centerline: a centreline to the left gives kappa > 0, a left equivalent angle
       (geometry.delta_for_curvature) and a left yaw rate on the plant; to the right the opposite;
     - upstream test_on_centerline_goes_straight: kappa 0 -> delta 0 -> the ideal plant drives exactly straight;
     - upstream test_curvature_limited_by_min_turn_radius: with min_turn_radius = L / tan(equivalent soft limit)
       the upstream clamp equals the curvature the plant reaches when the unclamped angle is commanded (the plant
       clamps the target to the equivalent soft limit, A21);
     - w = v kappa of row_follow_command equals the plant yaw rate v tan(delta) / L for delta = atan(L kappa).
   The upstream obstacle / confidence speed tests are not curvature functions and are not ported.
2. The survey probe bicycle (steering-sim-20261108/survey/original_ackermann_probe/probe_pure_pursuit_bicycle.py,
   outside the repository, read only): its run() loop is ported here (probe_reference_run). If the environment
   variable UBTS_STEER_PROBE_RESULT names the probe's result.json, recorded rows must be reproduced exactly;
   without it that test is skipped. Then the same closed loop is run on the steered plant (geometry_check: L0
   servo with the probe's rate and lag, eta 1, no noise) with the same observation noise draws, and the rear-axle
   lateral path must stay close to the probe bicycle driven with the plant's speed.

Upstream code is loaded from its file text into a private module object (no bytecode, no sys.path change). The
upstream file is found relative to this test (ros2_ws in the same work tree); without it the tests skip. Placeholder
profile: placeholder-based SIM results, not field validation.

Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_seed_regressions.py
"""
import json
import math
import os
from pathlib import Path
import sys
import types

import numpy as np
import pytest

from steering import fast_sim as fs
from steering import geometry as geo
from steering import maneuvers as mv
from steering import steered_plant as sp
from steering import vehicle_profile as vp

REPO = Path(__file__).resolve().parents[2]
UPSTREAM = REPO / 'ros2_ws' / 'src' / 'orchard_navigation' / 'orchard_navigation' / 'controller.py'
PROBE_RESULT_ENV = 'UBTS_STEER_PROBE_RESULT'
DT = .01


@pytest.fixture(scope='module')
def upstream():
    """orchard_navigation.controller loaded read-only from its source text (a private module object)."""
    if not UPSTREAM.is_file():
        pytest.skip('upstream controller not found at %s' % UPSTREAM)
    source = UPSTREAM.read_bytes().decode('utf-8')
    name = '_upstream_orchard_navigation_controller_readonly'
    module = types.ModuleType(name)
    module.__file__ = str(UPSTREAM)
    sys.modules[name] = module                 # dataclasses look the module up while the class is created
    try:
        exec(compile(source, str(UPSTREAM), 'exec'), module.__dict__)
    finally:
        sys.modules.pop(name, None)
    return module


@pytest.fixture(scope='module')
def profile():
    return vp.load_profile()


@pytest.fixture(scope='module')
def ideal(profile):
    return sp.SteeredPlantParams.from_profile(profile, geometry_check=True)


def drive_plant(params, steer_rad, seconds=8., speed=.4, seed=3):
    plant = sp.SteeredPlant(params, seed=seed)
    duty = mv.duty_for_speed(params.profile, speed, params.curve)
    for k in range(int(round(seconds / DT))):
        if k % 5 == 0:
            plant.command_actuation(duty, 1000. * steer_rad)
        plant.step(DT)
    return plant


# --- 1. upstream curvature functions --------------------------------------------------------------------------------------
def test_pure_pursuit_port_equals_upstream(upstream):
    grid = [(o, h, l) for o in (-1.8, -.5, -.03, 0., .02, .3, 1.8) for h in (-.4, -.05, 0., .1, .7)
            for l in (.5, 1., 1.5, 2., 2.5, 4.)]
    for offset, heading, lookahead in grid:
        assert fs.pure_pursuit_curvature(offset, heading, lookahead) == upstream.pure_pursuit_curvature(
            offset, heading, lookahead)


def test_turns_toward_centerline_on_the_plant(upstream, ideal):
    """upstream test_turns_toward_centerline: offset +0.5 -> left (w > 0), -0.5 -> right."""
    p = upstream.FollowParams()
    wheelbase = ideal.wheelbase_m
    for offset, sign in ((.5, 1.), (-.5, -1.)):
        v, w = upstream.row_follow_command(offset=offset, heading=0., confidence=1., obstacle_distance=math.inf, p=p)
        assert v > 0 and math.copysign(1., w) == sign
        kappa = upstream.pure_pursuit_curvature(offset, 0., p.lookahead)
        delta = geo.delta_for_curvature(kappa, wheelbase)
        assert math.copysign(1., delta) == sign
        assert geo.curvature(delta, wheelbase) == pytest.approx(kappa, rel=1e-12)
        plant = drive_plant(ideal, delta, seconds=3.)
        assert math.copysign(1., plant.w) == sign and math.copysign(1., plant.yaw) == sign
        assert plant.truth()['delta_eq_rad'] == pytest.approx(delta, abs=1e-12)


def test_on_centerline_goes_straight_on_the_plant(upstream, ideal):
    """upstream test_on_centerline_goes_straight: w = 0, v = cruise_speed; the ideal plant stays on the line."""
    v, w = upstream.row_follow_command(0., 0., 1., math.inf, upstream.FollowParams())
    assert abs(w) < 1e-9 and abs(v - .8) < 1e-9
    kappa = upstream.pure_pursuit_curvature(0., 0., 2.5)
    delta = geo.delta_for_curvature(kappa, ideal.wheelbase_m)
    assert kappa == 0. and delta == 0.
    plant = drive_plant(ideal, delta, seconds=10.)
    assert plant.y == 0. and plant.yaw == 0. and plant.w == 0. and plant.x > 3.


@pytest.mark.parametrize('offset', [1.8, -1.8])
def test_curvature_limited_by_min_turn_radius_on_the_plant(upstream, ideal, offset):
    """upstream test_curvature_limited_by_min_turn_radius with min_turn_radius = L / tan(equivalent soft limit): the
    upstream clamp gives the curvature the plant reaches when the unclamped equivalent angle is commanded."""
    wheelbase = ideal.wheelbase_m
    left, right = ideal.soft_limits_eq_rad
    limit = left if offset > 0 else right
    r_min = wheelbase / math.tan(limit)
    p = upstream.FollowParams(min_turn_radius=r_min, max_yaw_rate=10., lookahead=1.)
    v, w = upstream.row_follow_command(offset, 0., 1., math.inf, p)
    raw = upstream.pure_pursuit_curvature(offset, 0., 1.)
    assert abs(raw) > 1. / r_min                              # before the clamp the curvature is larger
    assert abs(w / v) <= 1. / r_min + 1e-9 and w / v == pytest.approx(math.copysign(1. / r_min, offset), rel=1e-12)
    unclamped = geo.delta_for_curvature(raw, wheelbase)
    assert abs(unclamped) > limit
    plant = drive_plant(ideal, unclamped, seconds=4.)
    assert plant.steer_target_clamped is True
    assert plant.steer_curvature == pytest.approx(w / v, rel=1e-9)             # same limit curvature
    assert plant.w == pytest.approx(plant.v * w / v, rel=1e-9)


def test_yaw_rate_of_the_curvature_command(upstream, ideal):
    """w = v kappa (row_follow_command) is the plant yaw rate v tan(delta) / L for delta = atan(L kappa)."""
    p = upstream.FollowParams(cruise_speed=.4, lookahead=2., min_turn_radius=.9, max_yaw_rate=.8)
    for offset, heading in ((.3, .05), (-.2, 0.), (.05, -.1)):
        v, w = upstream.row_follow_command(offset, heading, 1., math.inf, p)
        delta = math.atan(ideal.wheelbase_m * w / v)
        plant = drive_plant(ideal, delta, seconds=6., speed=.4)
        assert plant.v == pytest.approx(.4, rel=1e-6)
        assert plant.w == pytest.approx(plant.v * w / v, rel=1e-9)


# --- 2. survey probe bicycle -------------------------------------------------------------------------------------------------
PROBE = dict(WHEELBASE=.76, MAX_STEER=math.radians(30.), DT=.01, CTRL_DT=.05, OBS_DT=.1, LATENCY=.1, SIG_OFF=.03,
             SIG_HEAD=math.radians(1.), ROW_LEN=30., SETTLE_BAND=.05)


def probe_reference_run(upstream, model, speed, lookahead, rate_deg, tau, deadband_deg, y0, yaw0_deg, seed,
                        speed_of=None, record=False):
    """Port of run() of probe_pure_pursuit_bicycle.py (survey evidence, read only). speed_of(t) (optional) replaces
    the constant speed from the controller (used to drive the reference with the plant's speed)."""
    c = PROBE
    wheelbase, max_steer, dt = c['WHEELBASE'], c['MAX_STEER'], c['DT']
    r_min = wheelbase / math.tan(max_steer)
    rng = np.random.default_rng(seed)
    p = upstream.FollowParams(cruise_speed=speed, lookahead=lookahead, min_turn_radius=r_min, max_yaw_rate=.8)
    x, y, yaw, delta = 0., y0, math.radians(yaw0_deg), 0.
    t, next_ctrl, next_obs = 0., 0., 0.
    obs_queue, obs = [], None
    cmd_delta, cmd_w, v = 0., 0., 0.
    xs, ys, cmds_after, sat = [], [], [], 0
    rate = math.radians(rate_deg) if rate_deg else math.inf
    dead = math.radians(deadband_deg)
    n_ctrl = 0
    tv = 0.
    path = []
    while x < c['ROW_LEN'] and t < 300.:
        if t >= next_obs - 1e-9:
            off = -y / math.cos(yaw) + rng.normal(0, c['SIG_OFF'])
            head = -yaw + rng.normal(0, c['SIG_HEAD'])
            obs_queue.append((t + c['LATENCY'], off, head))
            next_obs += c['OBS_DT']
        while obs_queue and obs_queue[0][0] <= t + 1e-9:
            obs = obs_queue.pop(0)[1:]
        if t >= next_ctrl - 1e-9:
            if obs is not None:
                v, cmd_w = upstream.row_follow_command(obs[0], obs[1], 1.0, math.inf, p)
                kappa = cmd_w / v if v > 1e-6 else 0.
                new = max(-max_steer, min(max_steer, math.atan(wheelbase * kappa)))
                if n_ctrl:
                    tv += abs(new - cmd_delta)
                cmd_delta = new
                n_ctrl += 1
                if x > 5.:
                    cmds_after.append(cmd_delta)
                if abs(cmd_delta) >= max_steer - 1e-6:
                    sat += 1
            next_ctrl += c['CTRL_DT']
        speed_now = v if speed_of is None else speed_of(t)
        if model == 'unicycle':
            yaw += cmd_w * dt
        else:
            err = cmd_delta - delta
            if abs(err) > dead:
                target = cmd_delta if tau <= 0 else delta + err * min(1., dt / tau)
                step = max(-rate * dt, min(rate * dt, target - delta))
                delta = max(-max_steer, min(max_steer, delta + step))
            yaw += speed_now * math.tan(delta) / wheelbase * dt
        x += speed_now * math.cos(yaw) * dt
        y += speed_now * math.sin(yaw) * dt
        t += dt
        xs.append(x)
        ys.append(y)
        if record:
            path.append((t, x, y, yaw, delta, cmd_delta))
    xs, ys = np.array(xs), np.array(ys)
    after = ys[xs > 5.]
    outside = np.nonzero(np.abs(ys) >= c['SETTLE_BAND'])[0]
    settle_x = float(xs[outside[-1]]) if outside.size else 0.
    overshoot = float(max(0., -np.sign(y0) * ys.min() if y0 > 0 else ys.max()))
    ca = np.array(cmds_after)
    out = dict(model=model, speed_m_s=speed, lookahead_m=lookahead, steer_rate_deg_s=rate_deg, steer_tau_s=tau,
               steer_deadband_deg=deadband_deg, y0_m=y0, yaw0_deg=yaw0_deg, seed=seed,
               lat_rms_after5m_m=round(float(np.sqrt(np.mean(after ** 2))), 4) if after.size else None,
               lat_max_after5m_m=round(float(np.max(np.abs(after))), 4) if after.size else None,
               lat_max_whole_m=round(float(np.max(np.abs(ys))), 4),
               overshoot_opposite_side_m=round(overshoot, 4),
               settled_after_x_m=round(settle_x, 2),
               steer_cmd_std_after5m_deg=round(float(np.degrees(np.std(ca))), 3) if ca.size else None,
               steer_cmd_tv_deg_per_s=round(float(np.degrees(tv)) / max(t, 1e-6), 2),
               steer_saturated_ctrl_ticks=sat, finished_row=bool(x >= c['ROW_LEN']), sim_time_s=round(t, 2))
    if record:
        out['path'] = path
    return out


def test_probe_port_reproduces_the_recorded_probe_rows(upstream):
    path = os.environ.get(PROBE_RESULT_ENV, '').strip()
    if not path:
        pytest.skip('%s is not set (the survey probe result.json lives outside the repository)' % PROBE_RESULT_ENV)
    doc = json.loads(Path(path).read_bytes().decode('utf-8'))
    assert doc['schema'] == 'ubts.survey.original_ackermann_probe.v2'
    wanted = [('unicycle', .5, 2.5, None, 0., 0., .3, 3., 1), ('bicycle', .5, 2.5, 30., .3, 1., .3, 3., 2),
              ('bicycle', .5, 1.5, 8., .6, 0., .6, 10., 3), ('bicycle', .8, 1.0, 15., 0., 1., .3, 3., 1),
              ('bicycle', .8, 1.5, None, 1.0, 0., .6, 10., 2), ('bicycle', .5, 1.0, 4., .3, 1., .6, 10., 1)]
    keys = ('model', 'speed_m_s', 'lookahead_m', 'steer_rate_deg_s', 'steer_tau_s', 'steer_deadband_deg', 'y0_m',
            'yaw0_deg', 'seed')
    rows = {tuple(r[k] for k in keys): r for r in doc['runs']}
    for case in wanted:
        recorded = rows[case]
        again = probe_reference_run(upstream, *case)
        assert again == recorded, case


@pytest.mark.parametrize('speed, lookahead, y0, yaw0, seed', [(.5, 2.5, .3, 3., 1), (.5, 1.5, .6, 10., 2),
                                                             (.4, 2.0, .3, 3., 3), (.4, 2.0, .6, 10., 1)])
def test_steered_plant_closes_the_probe_loop_like_the_probe_bicycle(upstream, profile, speed, lookahead, y0, yaw0,
                                                                   seed):
    """The probe closed loop on the ideal steered plant (L0: rate 30 deg/s moving and standing, lag 0.3 s, no
    deadband, eta 1, no noise) and on the probe bicycle driven with the plant's speed, with the same observation
    noise draws: the rear-axle lateral paths agree within 2 cm and both meet the probe criterion."""
    rate, tau = 30., .3
    view = profile.with_overrides({'steer_rate_max_deg_s': rate, 'steer_rate_standstill_deg_s': rate,
                                   'steer_motor_tau_s': tau}, label='probe_compare')
    params = sp.SteeredPlantParams.from_profile(view, geometry_check=True)
    c = PROBE
    x_rear = params.rear_axle_x_in_base_m
    yaw = math.radians(yaw0)
    plant = sp.SteeredPlant(params, seed=0, pose=(-x_rear * math.cos(yaw), y0 - x_rear * math.sin(yaw), yaw))
    duty = mv.duty_for_speed(params.profile, speed, params.curve)
    rng = np.random.default_rng(seed)
    p = upstream.FollowParams(cruise_speed=speed, lookahead=lookahead,
                              min_turn_radius=params.wheelbase_m / math.tan(c['MAX_STEER']), max_yaw_rate=.8)
    t, next_ctrl, next_obs, obs_queue, obs, cmd = 0., 0., 0., [], None, 0.
    speeds, plant_path = [], []
    while plant.rear_axle_pose()[0] < c['ROW_LEN'] and t < 300.:
        rx, ry, ryaw = plant.rear_axle_pose()
        if t >= next_obs - 1e-9:
            obs_queue.append((t + c['LATENCY'], -ry / math.cos(ryaw) + rng.normal(0, c['SIG_OFF']),
                              -ryaw + rng.normal(0, c['SIG_HEAD'])))
            next_obs += c['OBS_DT']
        while obs_queue and obs_queue[0][0] <= t + 1e-9:
            obs = obs_queue.pop(0)[1:]
        if t >= next_ctrl - 1e-9:
            if obs is not None:
                v, w = upstream.row_follow_command(obs[0], obs[1], 1., math.inf, p)
                cmd = max(-c['MAX_STEER'], min(c['MAX_STEER'], math.atan(params.wheelbase_m * w / v)))
            plant.command_actuation(duty, 1000. * cmd)
            next_ctrl += c['CTRL_DT']
        speeds.append(plant.v)
        plant.step(DT)
        t += DT
        plant_path.append(plant.rear_axle_pose())
    speed_of = (lambda time_s: speeds[min(int(round(time_s / DT)), len(speeds) - 1)])
    reference = probe_reference_run(upstream, 'bicycle', speed, lookahead, rate, tau, 0., y0, yaw0, seed,
                                    speed_of=speed_of, record=True)
    ref = np.array([(x, y) for _, x, y, _, _, _ in reference['path']])
    mine = np.array([(x, y) for x, y, _ in plant_path])
    assert mine[-1, 0] >= c['ROW_LEN'] and reference['finished_row']
    ys_ref = np.interp(mine[:, 0], ref[:, 0], ref[:, 1])            # compare at the same travelled x
    assert float(np.max(np.abs(mine[:, 1] - ys_ref))) < .02
    after = mine[:, 0] > 5.
    lat_max_plant = float(np.max(np.abs(mine[after, 1])))
    assert reference['lat_max_after5m_m'] < .30 and lat_max_plant < .30           # probe criterion, both
    assert abs(lat_max_plant - reference['lat_max_after5m_m']) < .02
