"""Headless Gazebo probe of the steered rover (design 8.3 P5; acceptance A1(G), A4(G), A10(W0), A21(G), A25).

Two engines, no GUI:

  fixture  the gz-sim server runs in this process (gz.sim8.TestFixture) and is stepped in
           lockstep: after each physics step (post-update) the probe reads the truth state
           from the entity component manager and publishes the commands for the next step
           over gz-transport (in-process delivery is synchronous, so a command published
           after step k is in force during step k+1). Logs are kept at the physics rate.
           Physics real-time factor is set to 0 (as fast as possible); the dynamics are the
           scene's (120 Hz, DART). Used for A1, A4, A21 and A25.
  server   `gz sim -s -r` in a subprocess at real-time pacing with the scene's sensors
           (camera 640x360 if present); the probe drives the rover over gz-transport and
           reads /world/<world>/stats. Used for A10 and for scripted demo drives.

Commands use the model's topics (gz_model): steering wheel angle targets (rad) and drive
joint velocity targets (duty x omega0, rad/s; kinematic scenes: per-wheel kinematic speeds).
A25 steering targets come from the shared actuator model (backend_api): the L0 Python servo
or the native L1 backend in test_servo mode (built with g++ in the container). The model is
stepped once per physics step with the physics dt, and its wheel angles at the end of step
k+1 are the targets in force during step k+1, so A25 measures how the Gazebo joints follow
the shared model at the same instant. a25_firmware (report) does the same with the firmware-mode
backend (the firmware sources with the actuator model, a minimal ARM + C line host, the MCU
drive duties on the wheels): FirmwareSteerController.

The force-mode A1 groups also report the per-delta eta(delta) form with K = 0 (SIM_ACCEPTANCE
amendment 3) with its residuals and a linear eta(delta) fit for A12, and the turning speed loss
against the F-SIM drag loss range (turning_speed_loss).

a25_firmware steps at standstill (WF-4 g_gaps, decision R19-9): the phase 3 steps (20 -> -20 deg swings) went beyond
the placeholder THERMAL budget of the firmware (more than tha = 5000 hot ticks |pwm| >= thp in thw = 30 s: F THERMAL
5001 at 23,755 ms, reproduced without Gazebo by the same host rule). a25_firmware is a normal script of A19(G), so its
standstill steps are now at most 5 deg from 0 (swings of 10 deg, 376 hot ticks in the worst 30 s, margin 4624); the
old steps run in the report group thermal_firmware_report, which expects the THERMAL latch and is not a normal script
(result normal_script false). The registered A25 groups a25_l0 and a25_native keep the old steps (no firmware there).

A12 twin groups (a12_w0 on W0, a12_w2 on the field1010 mode R scene; P7): the open-loop scripts of
field_world.a12_twin (maneuvers 'schedule') with the F-SIM direct runner's command timing (commands
every 50 ms, 50 ms latency, the MCU drive ramp of the F-SIM plant, the equivalent soft-limit clamp)
and the shared L0 servo with the A12 profile view (random terms off); the comparison with the F-SIM
and the judgement are field_world.a12_twin.compare. rtf also takes the field1010 scene (A10 W2).

Placeholder-based SIM results ("자리값 기반 SIM 결과"); never field validation.
Needs the gz Python bindings (SIM image). The analysis helpers need numpy only.
CLI: python3 -B -m steering.gz_probe {run,rtf,drive,report} --help  (sim/ubts on sys.path)
"""
from __future__ import annotations

import argparse
import copy
import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as E

import numpy as np

from . import backend_api as ba
from . import drive as drive_model
from . import geometry as geo
from . import gz_assemble
from . import gz_model
from .l0_servo import L0ServoBackend
from .vehicle_profile import load_profile

SCHEMA = 'ubts.steer_gz_probe.v1'
RESULT_KIND = '자리값 기반 SIM 결과'
STANDSTILL_SPEED_M_S = .01           # same rule as steered_plant: below this the backend uses the standstill rates
STOP_SPEED_M_S = .005                # a stop ends when the base speed falls below this (F-SIM snap speed)
SPIKE_FACTOR = 1.25                  # wheel joint speed spike bound = 1.25 x the largest commanded wheel speed
STEER_SPIKE_FACTOR = 1.5             # steering joint speed spike bound = 1.5 x the largest shared-model rate
                                     # (a wheel turns up to about 1.25 x the equivalent rate at a = 1, 20 deg)
PLOT_LABEL = 'placeholder-based SIM result, not field validation'
DRIFT_LIMIT_M = .01                  # A25 standstill drift limits (registered): < 1 cm and < 0.1 deg in 60 s
DRIFT_LIMIT_DEG = .1
A25_P99_LIMIT_DEG = .2               # A25: joint - shared model target p99 <= 0.2 deg
A1_KIN_R_TOL = .03                   # A1(G) kinematic: R error <= 3 %
A1_KIN_W_TOL = .05                   # A1(G) kinematic: omega within +-5 %
A4_SPEED_TOL = .05                   # A4(G): steady speed +-5 % (3 duties)
A4_SPEED_DUTIES = (40., 70., 88.)    # the 3 duties of the A4(G) speed criterion (55 % is also reported)
A4_RELEASE_TARGET_M = (.56 * .85, .60 * 1.15)   # release target 0.56-0.60 m +-15 %
HEADING_SPREAD_TOL = .01             # start-direction independence: radii within +-1 % of their mean
LIMIT_TOL_RAD = 1e-6                 # A21: a joint angle beyond its limit by more than this is an exceedance
GROUPS = ('a1_kinematic', 'a1_force', 'a1_force_ideal', 'a1_ackermann', 'a4_drive', 'a4_tau_sweep', 'a21_limits',
          'a25_l0', 'a25_native', 'a25_firmware', 'thermal_firmware_report', 'fdir_aniso', 'a12_w0', 'a12_w2')
A12_GROUPS = ('a12_w0', 'a12_w2')
DIAGNOSTIC_GROUPS = ('a1_force_ideal', 'a4_tau_sweep', 'fdir_aniso')     # reported, no registered criterion of their own
A25_GROUPS = ('a25_l0', 'a25_native', 'a25_firmware', 'thermal_firmware_report')
# a25_firmware: the A25 measurement with the firmware-mode backend (phase 3 task: a report, not the registered judgement,
# which uses the L0 and the native L1 test_servo models). Its check is A25_firmware_report.
REPORT_GROUPS = ('a25_firmware', 'thermal_firmware_report')
NOT_NORMAL_GROUPS = ('thermal_firmware_report',)       # A19(G): faults are expected (not a normal script)
A25_STEPS_DEG = ((0., 0.), (2., 20.), (12., -20.), (22., 5.), (32., -5.), (42., 23.), (52., 0.))
# firmware standstill steps within the placeholder thermal budget (module docstring; R18-3: steps of at most 5 deg)
A25_FIRMWARE_STEPS_DEG = ((0., 0.), (2., 5.), (12., -5.), (22., 3.), (32., -3.), (42., 5.), (52., 0.))
# fdir1 check (design 3.6 [추정] "fdir1 0 0 1 turns with the steered wheel"): with mu = mu2 and unsaturated contacts the
# friction basis cannot change a result, so the registered start-direction check (isotropic) does not test it. Strongly
# anisotropic wheel friction (lateral 0.005, longitudinal 1.0; model option, outside the profile range on purpose) makes
# the result depend on the basis: start-frame tracks of several start directions must agree, and differ from the
# profile-friction reference (the anisotropy acted). Review F3.
FDIR_MU_LAT, FDIR_MU_LONG = .005, 1.
FDIR_HEADINGS_DEG = (0., 45., 90., 135., 200.)
FDIR_TRACK_TOL_M = 1e-6               # start-frame track difference between start directions (absolute; phase 3)
FDIR_EFFECT_MIN_M = .05               # the anisotropic track must differ this much from the reference
FDIR_TRACK_REL_TOL = 1e-6             # relative form (WF-4 g_gaps, decision R19-14): spread / effect <= this


# =================================================================================================================
# analysis helpers (pure numpy)
# =================================================================================================================
def fit_circle(x, y):
    """Least-squares circle through points: algebraic (Kasa) start refined by Gauss-Newton on the
    geometric distance. Returns dict(cx, cy, radius, rms, max_abs, arc_deg)."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if x.size < 5 or x.size != y.size:
        raise ValueError('fit_circle_needs_5_points')
    a = np.column_stack([x, y, np.ones_like(x)])
    sol = np.linalg.lstsq(a, x * x + y * y, rcond=None)[0]
    cx, cy = sol[0] / 2., sol[1] / 2.
    r = math.sqrt(max(sol[2] + cx * cx + cy * cy, 1e-12))
    for _ in range(50):
        dx, dy = x - cx, y - cy
        d = np.hypot(dx, dy)
        d[d == 0.] = 1e-12
        jac = np.column_stack([-dx / d, -dy / d, -np.ones_like(d)])
        step = np.linalg.lstsq(jac, -(d - r), rcond=None)[0]
        cx, cy, r = cx + step[0], cy + step[1], r + step[2]
        if np.max(np.abs(step)) < 1e-13 * max(1., abs(r)):
            break
    res = np.hypot(x - cx, y - cy) - r
    angles = np.unwrap(np.arctan2(y - cy, x - cx))
    return dict(cx=float(cx), cy=float(cy), radius=float(abs(r)), rms=float(np.sqrt(np.mean(res * res))),
                max_abs=float(np.max(np.abs(res))), arc_deg=float(math.degrees(abs(angles[-1] - angles[0]))))


def yaw_rate_fit(t, yaw):
    """Slope (rad/s) of the unwrapped yaw against time (least squares)."""
    t = np.asarray(t, dtype=float)
    return float(np.polyfit(t - t[0], np.unwrap(np.asarray(yaw, dtype=float)), 1)[0])


def path_speed(t, x, y):
    """Mean speed along the path (path length / elapsed time)."""
    t, x, y = (np.asarray(a, dtype=float) for a in (t, x, y))
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))) / (t[-1] - t[0]))


def point_track(x, y, yaw, offset_x, offset_y=0.):
    """World track of a body point (offset in the model frame)."""
    x, y, yaw = (np.asarray(a, dtype=float) for a in (x, y, yaw))
    c, s = np.cos(yaw), np.sin(yaw)
    return x + offset_x * c - offset_y * s, y + offset_x * s + offset_y * c


def percentile_stats(values_deg):
    v = np.abs(np.asarray(values_deg, dtype=float))
    if v.size == 0:
        return dict(n=0)
    return dict(n=int(v.size), p50=float(np.percentile(v, 50)), p95=float(np.percentile(v, 95)),
                p99=float(np.percentile(v, 99)), max=float(np.max(v)), mean=float(np.mean(v)))


def fit_effective_bicycle(tan_delta, speed, curvature, wheelbase_m):
    """Fit kappa = tan(delta) / (L_eff + K v^2) by least squares on tan(delta)/kappa = L_eff + K v^2.

    Returns dict(L_eff, K, eta = L / L_eff, understeer_gradient = K * eta (the F-SIM K_us of
    kappa = eta tan(delta) / (L + K_us v^2)), residuals of curvature (relative) and radius (m)).
    """
    td, v, k = (np.asarray(a, dtype=float) for a in (tan_delta, speed, curvature))
    y = td / k
    a = np.column_stack([np.ones_like(v), v * v])
    (l_eff, k_v), *_ = np.linalg.lstsq(a, y, rcond=None)
    pred = td / (l_eff + k_v * v * v)
    eta = wheelbase_m / l_eff
    return dict(L_eff_m=float(l_eff), K_v2_s2_m=float(k_v), eta=float(eta), understeer_gradient_s2_m=float(k_v * eta),
                curvature_rel_residual=[float(r) for r in (k - pred) / pred],
                radius_residual_m=[float(r) for r in 1. / k - 1. / pred],
                max_abs_curvature_rel_residual=float(np.max(np.abs((k - pred) / pred))))


def fit_eta_linear(delta_deg, eta):
    """Fit eta(delta) = 1 - c |delta| (deg) through eta(0) = 1 by least squares on the per-delta eta values (K = 0 form
    of SIM_ACCEPTANCE amendment 3). Returns dict(c_per_deg, residual (eta - model) per delta, max_abs_residual)."""
    d = np.abs(np.asarray(delta_deg, dtype=float))
    e = np.asarray(eta, dtype=float)
    c = float(np.sum((1. - e) * d) / np.sum(d * d))
    residual = e - (1. - c * d)
    return dict(c_per_deg=c, delta_deg=[float(x) for x in d], eta=[float(x) for x in e],
                residual=[float(x) for x in residual], max_abs_residual=float(np.max(np.abs(residual))),
                form='eta(delta) = 1 - c |delta_deg|')


def turning_speed_loss(rows, profile):
    """Turning speed loss 1 - v / v_target of the force-mode circles (heading 0, delta > 0) per delta and target speed,
    with the F-SIM drag loss range at the same delta (speed x (1 - scrub_speed_loss_per_deg |delta_deg|), steered_plant):
    the per-run draw U(low, high) of the profile and the profile range maximum."""
    record = profile.param('scrub_speed_loss_per_deg')
    draw = record['value'] if isinstance(record['value'], dict) else dict(low=record['value'], high=record['value'])
    table = {}
    for r in rows:
        if r['tags']['heading_deg'] != 0. or r['tags']['delta_deg'] <= 0.:
            continue
        d, v = r['tags']['delta_deg'], r['tags']['v']
        cell = table.setdefault('%g' % d, dict(delta_deg=d, fsim_drag_loss_draw=[draw['low'] * d, draw['high'] * d],
                                               fsim_drag_loss_profile_range_max=record['range'][1] * d, by_speed={}))
        loss = 1. - r['speed_m_s'] / v
        cell['by_speed']['%g' % v] = dict(target_m_s=v, speed_m_s=r['speed_m_s'], loss=loss,
                                          inside_fsim_draw=bool(draw['low'] * d - 1e-12 <= loss <= draw['high'] * d + 1e-12))
    return dict(table=table, fsim_parameter='scrub_speed_loss_per_deg', fsim_draw=draw, fsim_range=record['range'],
                note='G-SIM loss = 1 - path speed of the rear axle centre / straight target speed (same duty, 4 wheels '
                     'equal duty); F-SIM loss = scrub_speed_loss_per_deg x |delta_deg| per run')


def fit_dc_line(duty_frac, speed_m_s):
    """Least-squares line v = r_omega0 (d - d0) through measured (duty, speed) points with speed > 0."""
    d, v = (np.asarray(a, dtype=float) for a in (duty_frac, speed_m_s))
    keep = v > 1e-3
    slope, intercept = np.polyfit(d[keep], v[keep], 1)
    return dict(r_omega0_m_s=float(slope), d0=float(-intercept / slope), points=int(keep.sum()))


def stop_distance(t, x, y, speed, t_event):
    """Distance and time from the event until the speed first falls below STOP_SPEED_M_S.

    The samples are step ends (t = k dt). The event command is in force from the first step whose END is at or after
    t_event (DriveProgram evaluates the command at the step end time), so the event starts at the state just before
    that step: index searchsorted(t, t_event) - 1. The distance includes that first event step and speed_at_event_m_s
    is the speed before it (review F4: the first version started one step late)."""
    t, x, y, speed = (np.asarray(a, dtype=float) for a in (t, x, y, speed))
    start = max(int(np.searchsorted(t, t_event)) - 1, 0)
    after = np.nonzero(speed[start:] < STOP_SPEED_M_S)[0]
    stop = start + int(after[0]) if after.size else len(t) - 1
    distance = float(np.sum(np.hypot(np.diff(x[start:stop + 1]), np.diff(y[start:stop + 1]))))
    return dict(distance_m=distance, time_s=float(t[stop] - t[start]), stopped=bool(after.size),
                speed_at_event_m_s=float(speed[start]), event_start_t_s=float(t[start]))


def standstill_drift(t, x, y, yaw, t_from, t_to):
    t = np.asarray(t, dtype=float)
    sel = (t >= t_from) & (t <= t_to)
    xs, ys, yaws = np.asarray(x, float)[sel], np.asarray(y, float)[sel], np.unwrap(np.asarray(yaw, float)[sel])
    move = np.hypot(xs - xs[0], ys - ys[0])
    return dict(window_s=[float(t_from), float(t_to)], max_position_change_m=float(np.max(move)),
                end_position_change_m=float(move[-1]), max_yaw_change_deg=float(np.degrees(np.max(np.abs(yaws - yaws[0])))))


def rtf_windows(sim_s, real_s, window_s=5.):
    """Real-time factors over consecutive real-time windows from (sim, real) samples."""
    sim, real = np.asarray(sim_s, dtype=float), np.asarray(real_s, dtype=float)
    out = []
    start = 0
    for i in range(1, len(real)):
        if real[i] - real[start] >= window_s:
            out.append(float((sim[i] - sim[start]) / (real[i] - real[start])))
            start = i
    return out


def quaternion_rotate(q, v):
    """Rotate vector v by unit quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx, ty, tz = 2. * (y * vz - z * vy), 2. * (z * vx - x * vz), 2. * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


def quaternion_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2. * (w * x + y * z), 1. - 2. * (x * x + y * y))
    pitch = math.asin(max(-1., min(1., 2. * (w * y - z * x))))
    yaw = math.atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))
    return roll, pitch, yaw


def _json_default(value):
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    raise TypeError('not JSON serializable: %r' % (type(value),))


def _clean(value):
    """JSON-safe copy: numpy scalars and arrays to Python, NaN and infinities to None."""
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, np.ndarray):
        return _clean(value.tolist())
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    return value


def write_json(path, payload):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_bytes((json.dumps(_clean(payload), indent=1, ensure_ascii=False, allow_nan=False,
                                       default=_json_default) + '\n').encode('utf-8'))


# =================================================================================================================
# scenario definitions
# =================================================================================================================
class Context:
    """Per-group constants: scene profile view, geometry, DC line, physics step."""

    def __init__(self, view, dt_s):
        self.view = view
        self.geometry = geo.SteeringGeometry.from_profile(view)
        self.line = drive_model.dc_motor_line(view)
        self.dt = dt_s
        self.steer_type = view.value('steer_type')
        self.driven = gz_model.DRIVEN_WHEELS[view.value('drive_wheels')]
        ramp = drive_model.mcu_ramp(view)
        self.ramp_frac_per_s = ramp.pct_per_s / 100.

    def steer_targets(self, delta_eq):
        left, right = self.geometry.wheel_angles(delta_eq)
        if self.steer_type == geo.AXLE_PIVOT:
            return {'axle': delta_eq}
        return {'left': left, 'right': right}

    def duty_for_speed(self, speed):
        return self.line.deadband_frac + speed / self.line.r_omega0_m_s

    def drive_targets(self, duty_frac):
        return {wheel: duty_frac * self.line.omega0_rad_s for wheel in self.driven}


def ramp(t, t0, t1, a, b):
    """Linear ramp from a (t <= t0) to b (t >= t1)."""
    if t <= t0:
        return a
    if t >= t1:
        return b
    return a + (b - a) * (t - t0) / (t1 - t0)


def piecewise(t, schedule):
    """Value of the last (t_start, value) entry with t_start <= t."""
    value = schedule[0][1]
    for start, item in schedule:
        if t >= start:
            value = item
    return value


class ScheduleController:
    """Commands from schedule functions: steer_fn(t, obs) -> {key: rad}, drive_fn(t, obs) -> {wheel: rad/s},
    twist_fn(t, obs) -> (v, w) for the AckermannSteering scene."""

    def __init__(self, steer_fn=None, drive_fn=None, twist_fn=None):
        self.steer_fn, self.drive_fn, self.twist_fn = steer_fn, drive_fn, twist_fn

    def command(self, k_next, t_next, obs):
        return dict(steer=self.steer_fn(t_next, obs) if self.steer_fn else None,
                    drive=self.drive_fn(t_next, obs) if self.drive_fn else None,
                    twist=self.twist_fn(t_next, obs) if self.twist_fn else None)

    def close(self):
        pass


def backend_identity(backend):
    """Identity of a shared actuator model backend for result records (design 7.3: the MCU / model build SHA).
    L0: its info(). Native: result_label() (build id, executable SHA-256, mode, seed) and the build it ran
    (build_key, binary_sha256, source_sha256, firmware_build_id; native/build.py BuildResult)."""
    out = dict(cls=type(backend).__name__)
    info = getattr(backend, 'info', None)
    if callable(info):
        try:
            out['info'] = info()
        except Exception as exc:                  # information only
            out['info_error'] = '%s: %s' % (type(exc).__name__, exc)
    if callable(getattr(backend, 'result_label', None)) and getattr(backend, 'config', None) is not None:
        try:
            out['label'] = backend.result_label()
        except Exception as exc:
            out['label_error'] = '%s: %s' % (type(exc).__name__, exc)
    build = getattr(backend, 'build', None)
    if build is not None:
        out['build'] = dict(build_key=getattr(build, 'build_key', None), binary_sha256=getattr(build, 'binary_sha256', None),
                            source_sha256=getattr(build, 'source_sha256', None),
                            firmware_build_id=getattr(build, 'firmware_build_id', None),
                            file_name=Path(build.path).name if getattr(build, 'path', None) else None,
                            built_now=getattr(build, 'built', None))
    elif getattr(backend, 'executable_sha256', None):
        out['executable_sha256'] = backend.executable_sha256
    return out


def file_sha256(path):
    import hashlib
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def provenance(world_sdf=None, backends=None):
    """Provenance block of a G-SIM result: the source identity (gz_assemble.source_identity: commit from git or the host
    environment, the running files' SHA bundle, the image manifest), the world SDF SHA-256 and the actuator model
    builds."""
    block = dict(source=gz_assemble.source_identity())
    if world_sdf is not None and Path(world_sdf).is_file():
        block['world_sdf'] = dict(file=Path(world_sdf).name, sha256=file_sha256(world_sdf))
    if backends:
        block['actuator_backends'] = backends
    return block


class ModelSteerController:
    """Steering targets from the shared actuator model (test_servo mode), stepped once per physics step."""

    def __init__(self, ctx, backend, target_eq_fn, drive_fn=None):
        self.ctx, self.backend, self.target_eq_fn, self.drive_fn = ctx, backend, target_eq_fn, drive_fn
        self.steps = 0
        self.rows = []
        self.last = backend.reset(0.)

    def command(self, k_next, t_next, obs):
        speed = obs['speed']
        while self.steps < k_next:
            self.steps += 1
            t = self.steps * self.ctx.dt
            target = self.target_eq_fn(t)
            out = self.backend.step(ba.SteerStepInput(dt_s=self.ctx.dt, mode='test_servo', target_delta_eq_rad=target,
                                                      vehicle_speed_m_s=speed,
                                                      vehicle_stopped=abs(speed) < STANDSTILL_SPEED_M_S))
            self.last = out
            self.rows.append((self.steps, t, target, out.delta_eq_rad, out.delta_left_rad, out.delta_right_rad,
                              out.motor_side_angle_rad, out.pot_raw, out.applied_steer_pwm))
        out = self.last
        if self.ctx.steer_type == geo.AXLE_PIVOT:
            steer = {'axle': out.delta_eq_rad}
        else:
            steer = {'left': out.delta_left_rad, 'right': out.delta_right_rad}
        return dict(steer=steer, drive=self.drive_fn(t_next, obs) if self.drive_fn else None, twist=None)

    def close(self):
        self.backend.close()


FW_ARM_TICK = 10            # firmware-mode host (a25_firmware): ARM <boot_id> at this tick after RESET
FW_FIRST_C_TICK = 20        # first C line
FW_C_PERIOD_TICKS = 50      # then a C line every 50 ticks (20 Hz resend with a new q keeps the TTL and the watchdog alive)
FW_C_TTL_MS = 300
FW_CHIP_ID = 0x5A1B2C       # SIM chip id (as the firmware tests); boot_seed = 1 + seed


class FirmwareSteerController:
    """Steering and drive from the firmware-mode backend (lockstep v2: the unchanged firmware sources of
    firmware/ubts_steer_v3 with the actuator and potentiometer model, actuator_link.FirmwareSteeringBackend), stepped
    once per physics step like ModelSteerController.

    A minimal host, not the bridge: the EEPROM image of a car calibrated before the run (fw_eeprom.calibrated_eeprom,
    LOCKSTEP 11.11), ARM <boot_id> at FW_ARM_TICK, then 'C q ttl L R S' lines every FW_C_PERIOD_TICKS ticks from
    FW_FIRST_C_TICK with a new q: S = round(target_deg x the calibration scale) clipped to +-1000 (the MCU clips to its
    soft limits), L = R = round(255 x duty_fn(t)). The Gazebo steering joints get the model's truth wheel angles at the
    end of each step; the driven wheels get the MCU drive duty x omega0 (left duty for the left wheels). summary()
    reports the MCU lines (faults, refusals) and the last MCU state; it is a report, not the A17 / A19 judgement."""

    def __init__(self, ctx, seed, target_eq_fn, duty_fn=None, executable=None):
        from .actuator_link import FirmwareSteeringBackend, host_line
        from .native import fw_eeprom
        from .native import lockstep_v2 as v2
        self.ctx, self.seed, self.target_eq_fn = ctx, seed, target_eq_fn
        self.duty_fn = duty_fn or (lambda t: 0.)
        self.host_line = host_line
        startup = v2.firmware_startup_config(ctx.view, seed)
        image, self.calibration = fw_eeprom.calibrated_eeprom(startup, ctx.view)
        self.backend = FirmwareSteeringBackend(ctx.view, seed, firmware=dict(
            boot_seed=1 + seed, chip_id=FW_CHIP_ID, eeprom_hex=image, adc_mode='model'), executable=executable)
        self.backend.reset(0.)
        self.backend.send(host_line('ARM ' + self.backend.boot_id(0)), tick=FW_ARM_TICK)
        self.next_c_tick, self.q, self.c_lines = FW_FIRST_C_TICK, 1, 0
        self.steps = 0
        self.rows = []
        self.last = None

    def permille(self, target_rad):
        return int(max(-1000, min(1000, round(math.degrees(target_rad) * self.calibration['scale_permille_per_deg']))))

    def _schedule(self, ticks):
        """C lines whose tick falls in the next `ticks` ticks (sent before the step that runs them)."""
        end = self.backend.ticks + ticks
        while self.next_c_tick < end:
            t = self.next_c_tick / 1000.
            duty = int(round(255. * max(-1., min(1., self.duty_fn(t)))))
            self.backend.send(self.host_line('C %d %d %d %d %d' % (self.q, FW_C_TTL_MS, duty, duty,
                                                                   self.permille(self.target_eq_fn(t)))),
                              tick=self.next_c_tick)
            self.q += 1
            self.c_lines += 1
            self.next_c_tick += FW_C_PERIOD_TICKS

    def command(self, k_next, t_next, obs):
        speed = obs['speed']
        while self.steps < k_next:
            self.steps += 1
            t = self.steps * self.ctx.dt
            self._schedule(self.backend.clock.ticks_for(self.ctx.dt))
            out = self.backend.step(ba.SteerStepInput(dt_s=self.ctx.dt, mode='firmware', vehicle_speed_m_s=speed,
                                                      vehicle_stopped=abs(speed) < STANDSTILL_SPEED_M_S))
            self.last = out
            self.rows.append((self.steps, t, self.target_eq_fn(t), out.delta_eq_rad, out.delta_left_rad,
                              out.delta_right_rad, out.motor_side_angle_rad, out.pot_raw, out.applied_steer_pwm))
        out = self.last
        if out is None:
            steer = {'axle': 0.} if self.ctx.steer_type == geo.AXLE_PIVOT else {'left': 0., 'right': 0.}
            drive = self.ctx.drive_targets(0.)
        else:
            if self.ctx.steer_type == geo.AXLE_PIVOT:
                steer = {'axle': out.delta_eq_rad}
            else:
                steer = {'left': out.delta_left_rad, 'right': out.delta_right_rad}
            omega0 = self.ctx.line.omega0_rad_s
            drive = {wheel: (out.drive_duty_left if wheel.startswith('left') else out.drive_duty_right) * omega0
                     if out.drive_en else 0. for wheel in self.ctx.driven}
        return dict(steer=steer, drive=drive, twist=None)

    def summary(self):
        """MCU lines of the run (report): counts per kind, CRC, the first non-status lines, fault (F) and refusal (N)
        lines, the last status line and MCU state, the calibration used by the host conversion."""
        lines = self.backend.lines()
        kinds = {}
        for line in lines:
            kinds[line.kind] = kinds.get(line.kind, 0) + 1
        statuses = [line for line in lines if line.kind == 'S']
        last_status = statuses[-1].status() if statuses else None
        return dict(c_lines_sent=self.c_lines, mcu_lines=len(lines), line_kinds=kinds,
                    crc_ok=all(line.crc_ok for line in lines),
                    first_non_status_lines=[[line.queued_tick, line.text] for line in lines if line.kind != 'S'][:20],
                    fault_lines=[[line.queued_tick, line.text] for line in lines if line.kind == 'F'][:20],
                    refusal_lines=[[line.queued_tick, line.text] for line in lines if line.kind == 'N'][:20],
                    last_status=last_status, last_mcu_state=getattr(self.backend.last, 'mcu_state', None),
                    ticks_run=self.backend.ticks,
                    calibration=dict(scale_permille_per_deg=self.calibration['scale_permille_per_deg'],
                                     soft_l=self.calibration['soft_l'], soft_r=self.calibration['soft_r'],
                                     cal_id=self.calibration['cal_id']),
                    firmware_build=self.backend.firmware_build)

    def close(self):
        self.backend.close()


class DutyRamp:
    """The F-SIM plant's MCU drive ramp (steered_plant._DriveRamp rules): pct_per_s towards the target, through zero
    first, wait dwell_s after the duty reached 0 before a new non-zero duty."""

    def __init__(self, pct_per_s, dwell_s):
        self.pct_per_s, self.dwell_s = pct_per_s, dwell_s
        self.applied, self.target, self.hold_until = 0., 0., -math.inf

    def command(self, duty_pct):
        self.target = float(max(-100., min(100., duty_pct)))

    def step(self, now_s, dt):
        change = self.pct_per_s * dt
        applied, target = self.applied, self.target
        if applied != 0. and target * applied < 0:
            new = math.copysign(max(0., abs(applied) - change), applied)
        elif applied == 0. and target != 0. and now_s < self.hold_until:
            new = 0.
        else:
            new = applied + max(-change, min(change, target - applied))
        if applied != 0. and new == 0.:
            self.hold_until = now_s + self.dwell_s
        self.applied = new
        return new


class ScriptSteerController:
    """A12 twin driver: a maneuvers 'schedule' script with the F-SIM direct runner's command timing.

    Commands are evaluated at t_i = i x period (i >= 1) and are in force from t_i + latency (the F-SIM queue: a
    command acts from the first plant step that starts at or after its due time); before the first one the
    throttle is 0 and the steering target 0 (the start hold). The steering target is clamped to the equivalent soft
    limit (SteeredPlantParams.clamp_steer_target, as SteeredPlant.command_actuation); the duty goes through the
    F-SIM MCU ramp (DutyRamp) and drives the wheels as duty x omega0; the shared L0 servo (test_servo) is stepped
    once per physics step with the speed at the start of the step, like ModelSteerController."""

    def __init__(self, ctx, backend, schedule_params, *, period_s=.05, latency_s=.05):
        from . import maneuvers
        from .steered_plant import SteeredPlantParams
        self.ctx, self.backend = ctx, backend
        self.maneuvers = maneuvers
        self.params = maneuvers.normalise_params('schedule', schedule_params)
        self.period, self.latency = float(period_s), float(latency_s)
        self.clamp = SteeredPlantParams.from_profile(ctx.view).clamp_steer_target
        ramp = drive_model.mcu_ramp(ctx.view)
        self.ramp = DutyRamp(ramp.pct_per_s, ramp.reversal_dwell_s)
        self.steps = 0
        self.rows = []
        self.duty = 0.
        self.last = backend.reset(0.)

    def in_force(self, t_start):
        i = int(math.floor((t_start - self.latency) / self.period + 1e-9))
        if i < 1:
            return 0., 0.
        throttle, steer_deg = self.maneuvers.schedule_value(self.params, i * self.period)
        return throttle, self.clamp(math.radians(steer_deg))

    def command(self, k_next, t_next, obs):
        speed = obs['speed']
        dt = self.ctx.dt
        while self.steps < k_next:
            throttle, target = self.in_force(self.steps * dt)
            self.steps += 1
            t = self.steps * dt
            self.ramp.command(throttle)
            self.duty = self.ramp.step(t, dt)
            out = self.backend.step(ba.SteerStepInput(dt_s=dt, mode='test_servo', target_delta_eq_rad=target,
                                                      vehicle_speed_m_s=speed,
                                                      vehicle_stopped=abs(speed) < STANDSTILL_SPEED_M_S))
            self.last = out
            self.rows.append((self.steps, t, target, out.delta_eq_rad, out.delta_left_rad, out.delta_right_rad,
                              out.motor_side_angle_rad, out.pot_raw, out.applied_steer_pwm))
        out = self.last
        if self.ctx.steer_type == geo.AXLE_PIVOT:
            steer = {'axle': out.delta_eq_rad}
        else:
            steer = {'left': out.delta_left_rad, 'right': out.delta_right_rad}
        return dict(steer=steer, drive=self.ctx.drive_targets(self.duty / 100.), twist=None)

    def summary(self):
        return dict(steps=self.steps, final_duty_pct=self.duty, period_s=self.period, latency_s=self.latency,
                    script=self.params)

    def close(self):
        self.backend.close()


class DriveProgram:
    """Drive duty program for A4: duty d from t_on, then an event at t_event: release (MCU ramp to 0 with EN on),
    brake (duty 0 at once, EN on) or coast (zero motor torque: the velocity target follows the measured wheel speed
    each step; exact in lockstep because the controller then sees zero error - an experiment, not a model of EN off)."""

    def __init__(self, ctx, duty, t_on, event=None, t_event=None):
        self.ctx, self.duty, self.t_on, self.event, self.t_event = ctx, duty, t_on, event, t_event
        self.current = 0.

    def __call__(self, t, obs):
        if t < self.t_on:
            self.current = 0.
        elif self.event is None or t < self.t_event:
            self.current = self.duty
        elif self.event == 'release':
            self.current = max(0., self.current - self.ctx.ramp_frac_per_s * self.ctx.dt)
        elif self.event == 'brake':
            self.current = 0.
        elif self.event == 'coast':
            return {wheel: obs['wheel_velocity'][wheel + '_joint'] for wheel in self.ctx.driven}
        return self.ctx.drive_targets(self.current)


def _grid(index, spacing=40., columns=4, x0=-60., y0=-60.):
    return (x0 + (index % columns) * spacing, y0 + (index // columns) * spacing)


def vehicle(name, xy, yaw_deg, controller_factory, tags, model_options=None):
    return dict(name=name, x=float(xy[0]), y=float(xy[1]), yaw=math.radians(yaw_deg), factory=controller_factory,
                tags=tags, model_options=dict(model_options or {}))


def _a1_cases():
    cases = [(d, v, 0.) for d in (5., 10., 20.) for v in (.3, .5)]
    cases += [(-10., .3, 0.), (10., .3, 90.), (10., .3, 113.33)]
    return cases


def group_definition(group, profile, backend='l0'):
    """dict(scene, duration_s, vehicles, analysis) for a probe group."""
    if group == 'a1_kinematic':
        vehicles = []
        for i, (d, v, heading) in enumerate(_a1_cases()):
            def factory(ctx, d=d, v=v):
                delta = math.radians(d)
                ideal = dict(zip(('left', 'right'), geo.ideal_ackermann_angles(delta, ctx.geometry.wheelbase_m,
                                                                                ctx.geometry.kingpin_width_m)))
                steer = lambda t, obs: {k: ramp(t, .2, 1.2, 0., a) for k, a in ideal.items()}

                def drive_fn(t, obs):
                    left, right = (ramp(t, .2, 1.2, 0., ideal[k]) for k in ('left', 'right'))
                    speeds = gz_model.kinematic_wheel_speeds(ctx.geometry, left, right, ramp(t, 1.5, 3.5, 0., v))
                    return {name[:-len('_joint')]: value for name, value in speeds.items()}
                return ScheduleController(steer, drive_fn)
            vehicles.append(vehicle('k%d' % i, _grid(i), heading, factory,
                                    dict(kind='a1', delta_deg=d, v=v, heading_deg=heading, mode='kinematic')))
        return dict(scene='w0_kinematic', duration_s=48., vehicles=vehicles, window=(8., 48.))
    if group in ('a1_force', 'a1_force_ideal'):
        vehicles = []
        for i, (d, v, heading) in enumerate(_a1_cases()):
            def factory(ctx, d=d, v=v):
                targets = ctx.steer_targets(math.radians(d))
                duty = ctx.duty_for_speed(v)
                steer = lambda t, obs: {k: ramp(t, .2, 1.2, 0., a) for k, a in targets.items()}
                drive_fn = lambda t, obs: ctx.drive_targets(ramp(t, 1.5, 2.5, 0., duty))
                return ScheduleController(steer, drive_fn)
            vehicles.append(vehicle('f%d' % i, _grid(i), heading, factory,
                                    dict(kind='a1', delta_deg=d, v=v, heading_deg=heading, mode='force')))
        if group == 'a1_force_ideal':      # diagnostic only: the force mode with ideal Ackermann (a = 1)
            return dict(scene='w0_force', duration_s=48., vehicles=vehicles, window=(8., 48.),
                        profile_overrides={'steer_ackermann_ratio': 1.0}, diagnostic=True)
        return dict(scene='w0_force', duration_s=48., vehicles=vehicles, window=(8., 48.))
    if group == 'a1_ackermann':
        vehicles = []
        for i, (d, v, heading) in enumerate(_a1_cases()[:7]):
            def factory(ctx, d=d, v=v):
                w = v * math.tan(math.radians(d)) / ctx.geometry.wheelbase_m
                return ScheduleController(twist_fn=lambda t, obs: (ramp(t, 1.5, 3.5, 0., v), ramp(t, 1.5, 3.5, 0., w)))
            vehicles.append(vehicle('a%d' % i, _grid(i), heading, factory,
                                    dict(kind='a1', delta_deg=d, v=v, heading_deg=heading, mode='ackermann_plugin')))
        return dict(scene='w0_ackermann_check', duration_s=48., vehicles=vehicles, window=(12., 48.))
    if group == 'a4_drive':
        vehicles = []
        duties = (20., 30., 40., 55., 70., 88., 100.)
        for i, duty in enumerate(duties):
            def factory(ctx, duty=duty):
                return ScheduleController(drive_fn=DriveProgram(ctx, duty / 100., 1.))
            vehicles.append(vehicle('d%d' % i, (-80., -60. + 10. * i), 0., factory,
                                    dict(kind='a4_speed', duty_pct=duty)))
        for j, event in enumerate(('release', 'brake', 'coast')):
            def factory(ctx, event=event):
                return ScheduleController(drive_fn=DriveProgram(ctx, 1., 1., event, 10.))
            vehicles.append(vehicle('s%d' % j, (-80., 20. + 10. * j), 0., factory,
                                    dict(kind='a4_stop', event=event, t_event=10., tau_stall_nm=None)))
        return dict(scene='w0_force', duration_s=16., vehicles=vehicles, window=(10., 15.))
    if group == 'a4_tau_sweep':
        vehicles = []
        taus = (.5, .8, 1.0, 2.5, 4.4, 5.0)
        for i, tau in enumerate(taus):
            for j, event in enumerate(('release', 'brake')):
                def factory(ctx, event=event):
                    return ScheduleController(drive_fn=DriveProgram(ctx, 1., 1., event, 14.))
                vehicles.append(vehicle('t%d%s' % (i, event[0]), (-80., -60. + 10. * (2 * i + j)), 0., factory,
                                        dict(kind='a4_stop', event=event, t_event=14., tau_stall_nm=tau),
                                        model_options=dict(tau_stall_nm=tau)))
        return dict(scene='w0_force', duration_s=22., vehicles=vehicles, window=(10., 14.))
    if group == 'a21_limits':
        vehicles = []
        push = [(0., 0.), (1., 40.), (4., -40.), (7., 60.), (10., -60.), (13., 0.)]

        def direct_factory(moving, opposite):
            def factory(ctx):
                def steer(t, obs):
                    a = math.radians(piecewise(t, push))
                    if ctx.steer_type == geo.AXLE_PIVOT:
                        return {'axle': a}
                    return {'left': a, 'right': -a if opposite else a}
                duty = ctx.duty_for_speed(.3) if moving else 0.
                return ScheduleController(steer, lambda t, obs: ctx.drive_targets(duty if t > .5 else 0.))
            return factory
        vehicles.append(vehicle('l0', _grid(0), 0., direct_factory(False, False), dict(kind='a21', how='direct',
                                                                                          moving=False)))
        vehicles.append(vehicle('l1', _grid(1), 0., direct_factory(False, True), dict(kind='a21', how='direct_opposite',
                                                                                         moving=False)))
        vehicles.append(vehicle('l2', _grid(2), 0., direct_factory(True, False), dict(kind='a21', how='direct',
                                                                                         moving=True)))

        def model_factory(moving):
            def factory(ctx):
                schedule = [(0., 0.), (1., 35.), (4., -35.), (7., 80.), (10., -80.), (13., 0.)]
                drive = (lambda t, obs: ctx.drive_targets(ctx.duty_for_speed(.3) if t > .5 else 0.)) if moving else None
                backend = L0ServoBackend()
                backend.configure(ctx.view, 21)
                return ModelSteerController(ctx, backend, lambda t: math.radians(piecewise(t, schedule)), drive)
            return factory
        vehicles.append(vehicle('l3', _grid(3), 0., model_factory(False), dict(kind='a21', how='model_l0',
                                                                                  moving=False)))
        vehicles.append(vehicle('l4', _grid(4), 0., model_factory(True), dict(kind='a21', how='model_l0', moving=True)))
        return dict(scene='w0_force', duration_s=16., vehicles=vehicles, window=(0., 16.))
    if group == 'fdir_aniso':
        def factory(ctx, d=20., v=.5):
            targets = ctx.steer_targets(math.radians(d))
            duty = ctx.duty_for_speed(v)
            steer = lambda t, obs: {k: ramp(t, .2, 1.2, 0., a) for k, a in targets.items()}
            drive_fn = lambda t, obs: ctx.drive_targets(ramp(t, 1.5, 2.5, 0., duty))
            return ScheduleController(steer, drive_fn)
        vehicles = []
        for i, heading in enumerate(FDIR_HEADINGS_DEG):
            vehicles.append(vehicle('fa%d' % i, _grid(i), heading, factory,
                                    dict(kind='fdir', delta_deg=20., v=.5, heading_deg=heading, friction='anisotropic'),
                                    model_options=dict(wheel_mu_lat=FDIR_MU_LAT, wheel_mu_long=FDIR_MU_LONG)))
        vehicles.append(vehicle('fr0', _grid(len(FDIR_HEADINGS_DEG)), 0., factory,
                                dict(kind='fdir', delta_deg=20., v=.5, heading_deg=0., friction='profile')))
        return dict(scene='w0_force', duration_s=24., vehicles=vehicles, window=(0., 24.), diagnostic=True)
    if group in A12_GROUPS:
        from field_world import a12_twin
        world = 'W0' if group == 'a12_w0' else 'W2'
        scripts = a12_twin.scripts(world)
        vehicles = []
        for i, (name, script) in enumerate(sorted(scripts.items())):
            def factory(ctx, script=script, seed=i):
                backend = L0ServoBackend()
                backend.configure(ctx.view, a12_twin.A12_SEED)
                return ScriptSteerController(ctx, backend, script['schedule'], period_s=a12_twin.COMMAND_PERIOD_S,
                                             latency_s=a12_twin.COMMAND_LATENCY_S)
            if world == 'W0':
                xy, yaw = _grid(i), 0.
            else:
                pose = a12_twin.w2_start_pose_w(script)
                xy, yaw = (pose[0], pose[1]), math.degrees(pose[2])
            vehicles.append(vehicle('t%d' % i, xy, yaw, factory, dict(kind='a12', script=name, world=world)))
        duration = max(s['schedule']['end_s'] or s['schedule']['knots'][-1][0] for s in scripts.values())
        out = dict(scene='w0_force' if world == 'W0' else 'field1010', duration_s=float(duration), vehicles=vehicles,
                   window=(0., float(duration)), profile_overrides=a12_twin.A12_OVERRIDES,
                   profile_overrides_label=a12_twin.A12_LABEL, profile_overrides_allow_out_of_range=True)
        if world == 'W2':
            out['field_spec'] = a12_twin.w2_field_spec().to_dict()
        return out
    if group in A25_GROUPS:
        kind = {'a25_l0': 'L0', 'a25_native': 'native_L1', 'a25_firmware': 'firmware',
                'thermal_firmware_report': 'firmware'}[group]

        def make_backend(ctx, seed):
            if kind == 'L0':
                b = L0ServoBackend()
                b.configure(ctx.view, seed)
                return b
            from .actuator_link import NativeSteeringBackend
            return NativeSteeringBackend(ctx.view, seed, mode='test_servo')
        steps = list(A25_FIRMWARE_STEPS_DEG if group == 'a25_firmware' else A25_STEPS_DEG)
        moving_steps = [(0., 0.), (4., 10.), (12., -10.), (20., 15.), (28., -5.), (36., 0.), (44., 12.), (52., 0.)]
        specs = [
            ('steps_standstill', lambda t: math.radians(piecewise(t, steps)), None),
            ('sine_moving', lambda t: math.radians(10.) * math.sin(2. * math.pi * .2 * max(0., t - 2.)), .4),
            ('steps_moving', lambda t: math.radians(piecewise(t, moving_steps)), .4),
            ('hold_standstill_0', lambda t: 0., None),
            ('hold_standstill_turned', lambda t: math.radians(ramp(t, 1., 3., 0., 20.)), None),
        ]
        if group == 'thermal_firmware_report':       # only the big standstill steps (THERMAL expected)
            specs = specs[:1]
        vehicles = []
        for i, (case, target_fn, speed) in enumerate(specs):
            def factory(ctx, target_fn=target_fn, speed=speed, seed=i):
                duty = ctx.duty_for_speed(speed) if speed is not None else 0.
                if kind == 'firmware':        # the MCU gets the duty in its C lines and drives the wheels itself
                    return FirmwareSteerController(ctx, seed, target_fn,
                                                   lambda t: ramp(t, 1., 2., 0., duty) if speed is not None else 0.)
                drive = None
                if speed is not None:
                    drive = lambda t, obs: ctx.drive_targets(ramp(t, 1., 2., 0., duty))
                return ModelSteerController(ctx, make_backend(ctx, seed), target_fn, drive)
            vehicles.append(vehicle('m%d' % i, _grid(i), 0., factory, dict(kind='a25', case=case, backend=kind)))
        return dict(scene='w0_force', duration_s=64., vehicles=vehicles, window=(0., 64.), steps_standstill_deg=steps,
                    drift_windows={'hold_standstill_0': (3., 63.), 'hold_standstill_turned': (4., 64.)}
                    if group != 'thermal_firmware_report' else {})
    raise ValueError('unknown_group:%r (choices %s)' % (group, ', '.join(GROUPS)))


# =================================================================================================================
# probe world
# =================================================================================================================
def dynamics_only(world):
    """Remove the Sensors system, every sensor and the W0 pillars (probe worlds need dynamics only)."""
    removed = dict(systems=0, sensors=0, pillars=0)   # field models stay (no collision in mode R: no dynamics)
    for plugin in list(world.findall('plugin')):
        if (plugin.get('name') or '').endswith('::Sensors'):
            world.remove(plugin)
            removed['systems'] += 1
    for link in world.iter('link'):
        for sensor in list(link.findall('sensor')):
            link.remove(sensor)
            removed['sensors'] += 1
    for model in list(world.findall('model')):
        if (model.get('name') or '').startswith('w0_pillar_'):
            world.remove(model)
            removed['pillars'] += 1
    return removed


def build_probe_world(group_def, out_dir, profile):
    """Write <out_dir>/probe_world.sdf with one steered vehicle per spec. Returns (path, ctx, vehicle manifests)."""
    scene = group_def['scene']
    if group_def.get('profile_overrides'):
        profile = profile.with_overrides(group_def['profile_overrides'],
                                         label=group_def.get('profile_overrides_label', 'gz_probe_diagnostic'),
                                         allow_out_of_range=bool(group_def.get('profile_overrides_allow_out_of_range')))
    base = gz_assemble.assemble_base(scene, Path(out_dir) / 'scene', profile=profile, camera=False,
                                     field_spec=group_def.get('field_spec'))
    world = base['world']
    removed = dynamics_only(world)
    template = world.find("model[@name='%s']" % gz_model.MODEL_NAME)
    world.remove(template)
    base_pose = [float(v) for v in template.find("link[@name='base_link']").findtext('pose').split()]
    for spec in group_def['vehicles']:
        spec['base_link_xyz'] = tuple(base_pose[:3])
    options = gz_assemble.scene_options(scene)
    manifests = {}
    for spec in group_def['vehicles']:
        model = copy.deepcopy(template)
        model.set('name', spec['name'])
        model.find('pose').text = '%r %r 0.003 0 0 %r' % (spec['x'], spec['y'], spec['yaw'])
        prefix = '/probe/%s' % spec['name']
        opts = dict(options)
        opts.update(group_def.get('model_options_all') or {})
        opts.update(spec['model_options'])
        manifests[spec['name']] = gz_model.steer_model_element(
            model, base['view'], topic_prefix=prefix + '/act', odom_topic=prefix + '/truth/odom',
            joint_state_topic=prefix + '/truth/joint_states', **opts)
        world.append(model)
    physics = world.find('physics')
    step = float(physics.findtext('max_step_size'))
    physics.find('real_time_factor').text = '0'
    physics.find('real_time_update_rate').text = '0'
    root = base['tree'].getroot()
    E.indent(root)
    path = Path(out_dir) / 'probe_world.sdf'
    path.write_bytes(E.tostring(root, encoding='unicode').encode('utf-8'))
    dt = int(step * 1e9) / 1e9          # gz-sim truncates the step to whole nanoseconds
    ctx = Context(base['view'], dt)
    write_json(Path(out_dir) / 'probe_world_manifest.json',
               dict(schema=SCHEMA, scene=scene, removed=removed, dt_s=dt, physics_step_s=step,
                    vehicles={name: dict(topics=m['topics'], drive=m['drive'], steering=m['steering'],
                                         plugins=m['plugins']) for name, m in manifests.items()},
                    profile=base['view'].result_label()))
    return path, ctx, manifests


# =================================================================================================================
# fixture engine (in-process, lockstep)
# =================================================================================================================
def _seconds(value):
    if isinstance(value, datetime.timedelta):
        return value.total_seconds()
    return float(value)


class _Vehicle:
    """One vehicle in the fixture run: entities, publishers, controller and its log."""

    def __init__(self, spec, manifest, ctx, node, msgs):
        self.spec, self.manifest, self.ctx = spec, manifest, ctx
        self.name = spec['name']
        self.steer_keys = [item['key'] for item in manifest['steering']]
        self.steer_joint = {item['key']: item['joint'] for item in manifest['steering']}
        self.limits = {item['key']: (item['lower_rad'], item['upper_rad']) for item in manifest['steering']}
        self.controller = spec['factory'](ctx)
        topics = manifest['topics']
        self.pub_steer = {key: node.advertise(topic, msgs['Double']) for key, topic in topics['steer'].items()}
        self.pub_drive = {wheel: node.advertise(topic, msgs['Double']) for wheel, topic in topics['drive'].items()}
        self.pub_twist = node.advertise(topics['ackermann'], msgs['Twist']) if topics['ackermann'] else None
        self.msgs = msgs
        self.log = {k: [] for k in ('k', 't', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'vx', 'vy', 'wz', 'speed')}
        for key in self.steer_keys:
            for item in ('q', 'qd', 'target'):
                self.log['steer_%s_%s' % (key, item)] = []
        for wheel in gz_model.WHEELS:
            self.log['wheel_%s_qd' % wheel] = []
            self.log['drive_%s_target' % wheel] = []
        self.log['twist_v'] = []
        self.log['twist_w'] = []
        self.steer_in_force = {key: 0. for key in self.steer_keys}
        self.drive_in_force = {wheel: 0. for wheel in gz_model.WHEELS}
        self.twist_in_force = (0., 0.)
        self.base_offset = None

    def resolve(self, ecm, sim):
        world = sim.World(sim.world_entity(ecm))
        model = sim.Model(world.model_by_name(ecm, self.name))
        self.base = sim.Link(model.link_by_name(ecm, 'base_link'))
        self.base.enable_velocity_checks(ecm)
        self.steer = {key: sim.Joint(model.joint_by_name(ecm, joint)) for key, joint in self.steer_joint.items()}
        self.wheels = {wheel: sim.Joint(model.joint_by_name(ecm, wheel + '_joint')) for wheel in gz_model.WHEELS}
        for joint in list(self.steer.values()) + list(self.wheels.values()):
            joint.enable_position_check(ecm)
            joint.enable_velocity_check(ecm)

    def read(self, ecm, k, t):
        pose = self.base.world_pose(ecm)
        p, r = pose.pos(), pose.rot()
        q = (r.w(), r.x(), r.y(), r.z())
        if self.base_offset is None:
            self.base_offset = tuple(self.spec['base_link_xyz'])
        off = quaternion_rotate(q, self.base_offset)
        x, y, z = p.x() - off[0], p.y() - off[1], p.z() - off[2]
        roll, pitch, yaw = quaternion_rpy(q)
        lin = self.base.world_linear_velocity(ecm)
        ang = self.base.world_angular_velocity(ecm)
        vx, vy = (lin.x(), lin.y()) if lin is not None else (0., 0.)
        wz = ang.z() if ang is not None else 0.
        speed = vx * math.cos(yaw) + vy * math.sin(yaw)
        log = self.log
        for name, value in (('k', k), ('t', t), ('x', x), ('y', y), ('z', z), ('roll', roll), ('pitch', pitch),
                            ('yaw', yaw), ('vx', vx), ('vy', vy), ('wz', wz), ('speed', speed)):
            log[name].append(value)
        steer_q = {}
        for key, joint in self.steer.items():
            pos, vel = joint.position(ecm), joint.velocity(ecm)
            steer_q[key] = pos[0] if pos else float('nan')
            log['steer_%s_q' % key].append(steer_q[key])
            log['steer_%s_qd' % key].append(vel[0] if vel else float('nan'))
            log['steer_%s_target' % key].append(self.steer_in_force[key])
        wheel_qd = {}
        for wheel, joint in self.wheels.items():
            vel = joint.velocity(ecm)
            wheel_qd[wheel + '_joint'] = vel[0] if vel else 0.
            log['wheel_%s_qd' % wheel].append(wheel_qd[wheel + '_joint'])
            log['drive_%s_target' % wheel].append(self.drive_in_force[wheel])
        log['twist_v'].append(self.twist_in_force[0])
        log['twist_w'].append(self.twist_in_force[1])
        return dict(t=t, speed=speed, wheel_velocity=wheel_qd, steer_position=steer_q)

    def command(self, k_next, t_next, obs):
        cmd = self.controller.command(k_next, t_next, obs)
        double = self.msgs['Double']
        if cmd.get('steer'):
            for key, value in cmd['steer'].items():
                msg = double()
                msg.data = float(value)
                self.pub_steer[key].publish(msg)
                self.steer_in_force[key] = float(value)
        if cmd.get('drive') and self.pub_drive:
            for wheel, value in cmd['drive'].items():
                msg = double()
                msg.data = float(value)
                self.pub_drive[wheel].publish(msg)
                self.drive_in_force[wheel] = float(value)
        if cmd.get('twist') is not None and self.pub_twist is not None:
            msg = self.msgs['Twist']()
            msg.linear.x, msg.angular.z = float(cmd['twist'][0]), float(cmd['twist'][1])
            self.pub_twist.publish(msg)
            self.twist_in_force = (msg.linear.x, msg.angular.z)

    def arrays(self):
        out = {key: np.asarray(values, dtype=float) for key, values in self.log.items()}
        rows = getattr(self.controller, 'rows', None)
        if rows:
            model = np.asarray(rows, dtype=float)
            for i, name in enumerate(('model_step', 'model_t', 'model_target_eq', 'model_delta_eq', 'model_delta_left',
                                      'model_delta_right', 'model_motor_side', 'model_pot_raw', 'model_pwm')):
                out[name] = model[:, i]
        return out


def run_fixture(world_sdf, group_def, ctx, manifests, *, chunk_steps=1200, progress=None):
    """Run the lockstep fixture for duration_s. Returns ({vehicle: arrays}, run info)."""
    import gz.math7  # noqa: F401  (registers the math types used by the sim8 bindings)
    import gz.sim8 as sim
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.twist_pb2 import Twist
    from gz.transport13 import Node

    node = Node()
    msgs = dict(Double=Double, Twist=Twist)
    vehicles = [_Vehicle(spec, manifests[spec['name']], ctx, node, msgs) for spec in group_def['vehicles']]
    state = dict(ready=False, last_k=0, errors=[])

    def pre(info, ecm):
        if not state['ready']:
            for v in vehicles:
                v.resolve(ecm, sim)
            state['ready'] = True

    def post(info, ecm):
        try:
            k = int(info.iterations)
            t = k * ctx.dt
            state['last_k'] = k
            for v in vehicles:
                obs = v.read(ecm, k, t)
                v.command(k + 1, (k + 1) * ctx.dt, obs)
        except Exception as exc:          # keep the server alive; report after the run
            if len(state['errors']) < 20:
                state['errors'].append('%s: %s' % (type(exc).__name__, exc))

    fixture = sim.TestFixture(str(world_sdf))
    fixture.on_pre_update(pre)
    fixture.on_post_update(post)
    fixture.finalize()
    server = fixture.server()
    total = int(round(group_def['duration_s'] / ctx.dt))
    started = time.perf_counter()
    done = 0
    while done < total:
        n = min(chunk_steps, total - done)
        server.run(True, n, False)
        done += n
        if progress:
            progress('%d/%d steps, wall %.1f s' % (done, total, time.perf_counter() - started))
        if state['errors']:
            break
    wall = time.perf_counter() - started
    summaries = {}
    for v in vehicles:
        if callable(getattr(v.controller, 'summary', None)):
            try:
                summaries[v.name] = v.controller.summary()
            except Exception as exc:              # information only
                summaries[v.name] = dict(error='%s: %s' % (type(exc).__name__, exc))
    for v in vehicles:
        v.controller.close()
    backends = {v.name: backend_identity(v.controller.backend) for v in vehicles
                if getattr(v.controller, 'backend', None) is not None}
    info = dict(steps=done, sim_s=done * ctx.dt, wall_s=wall, sim_per_wall=done * ctx.dt / wall if wall else None,
                errors=state['errors'], dt_s=ctx.dt, actuator_backends=backends)
    if summaries:
        info['controller_summaries'] = summaries
    return {v.name: v.arrays() for v in vehicles}, info


# =================================================================================================================
# per-group analysis
# =================================================================================================================
def _window(a, t0, t1):
    return (a['t'] >= t0) & (a['t'] <= t1)


def _spikes(a, keys, steer_bound, wheel_bound):
    steer_max = max(float(np.nanmax(np.abs(a['steer_%s_qd' % key]))) for key in keys)
    wheel_max = max(float(np.nanmax(np.abs(a['wheel_%s_qd' % wheel]))) for wheel in gz_model.WHEELS)
    finite = all(np.all(np.isfinite(a[name])) for name in ('x', 'y', 'z', 'yaw'))
    return dict(steer_joint_speed_max_rad_s=steer_max, steer_bound_rad_s=steer_bound,
                wheel_joint_speed_max_rad_s=wheel_max, wheel_bound_rad_s=wheel_bound, finite=finite,
                spike_free=bool(finite and steer_max <= steer_bound and (wheel_bound is None or wheel_max <= wheel_bound)))


def _limit_check(a, keys, limits):
    worst = {}
    ok = True
    for key in keys:
        lower, upper = limits[key]
        q = a['steer_%s_q' % key]
        over = float(np.nanmax(q) - upper)
        under = float(lower - np.nanmin(q))
        worst[key] = dict(max_deg=float(np.degrees(np.nanmax(q))), min_deg=float(np.degrees(np.nanmin(q))),
                          upper_deg=math.degrees(upper), lower_deg=math.degrees(lower),
                          overshoot_upper_deg=math.degrees(over), overshoot_lower_deg=math.degrees(under))
        ok = ok and over <= LIMIT_TOL_RAD and under <= LIMIT_TOL_RAD
    return ok, worst


def analyze_group(group, group_def, ctx, logs, manifests):
    """Metrics and checks for one group. Returns a JSON-friendly dict."""
    rate_bound = STEER_SPIKE_FACTOR * math.radians(max(ctx.view.value('steer_rate_max_deg_s'),
                                                 ctx.view.value('steer_rate_standstill_deg_s')))
    l_geom = ctx.geometry.wheelbase_m
    rear_x = ctx.geometry.rear_axle_x_in_base_m
    out = dict(group=group, scene=group_def['scene'], vehicles={}, checks={})
    if group.startswith('a1_'):
        t0, t1 = group_def['window']
        rows = []
        for spec in group_def['vehicles']:
            a, tags = logs[spec['name']], spec['tags']
            sel = _window(a, t0, t1)
            xr, yr = point_track(a['x'][sel], a['y'][sel], a['yaw'][sel], rear_x)
            circle = fit_circle(xr, yr)
            w = yaw_rate_fit(a['t'][sel], a['yaw'][sel])
            v = path_speed(a['t'][sel], xr, yr)
            delta = math.radians(tags['delta_deg'])
            r_geom = abs(l_geom / math.tan(delta))
            w_cmd = tags['v'] * math.tan(delta) / l_geom
            keys = [item['key'] for item in manifests[spec['name']]['steering']]
            if tags['mode'] == 'ackermann_plugin':      # the plugin's own P control: finite check only
                spikes = _spikes(a, keys, float('inf'), None)
            elif tags['mode'] == 'kinematic':
                wheel_cmd = max(float(np.max(np.abs(a['drive_%s_target' % w_]))) for w_ in gz_model.WHEELS)
                spikes = _spikes(a, keys, rate_bound, SPIKE_FACTOR * wheel_cmd)
            else:
                spikes = _spikes(a, keys, rate_bound, SPIKE_FACTOR * ctx.line.omega0_rad_s)
            row = dict(tags=tags, radius_m=circle['radius'], radius_geometry_m=r_geom,
                       radius_error=(circle['radius'] - r_geom) / r_geom, circle_rms_m=circle['rms'],
                       arc_deg=circle['arc_deg'], yaw_rate_rad_s=w, yaw_rate_cmd_rad_s=w_cmd,
                       yaw_rate_error_vs_cmd=(w - w_cmd) / w_cmd, speed_m_s=v,
                       yaw_rate_error_vs_measured_speed=(w - v * math.tan(delta) / l_geom) / (v * math.tan(delta) / l_geom),
                       curvature_1_m=math.copysign(1. / circle['radius'], delta),
                       yaw_sign_ok=bool(np.sign(w) == np.sign(delta)), spikes=spikes,
                       z_mean_m=float(np.mean(a['z'][sel])))
            out['vehicles'][spec['name']] = row
            rows.append(row)
        base = [r for r in rows if r['tags']['heading_deg'] == 0.]
        headings = [r for r in rows if r['tags']['delta_deg'] == 10. and r['tags']['v'] == .3]
        radii = [r['radius_m'] for r in headings]
        friction = sorted({json.dumps(manifests[spec['name']].get('wheel_friction'), sort_keys=True)
                           for spec in group_def['vehicles']})
        if len({r['tags']['heading_deg'] for r in headings}) >= 2:
            mean_r = float(np.mean(radii))
            deviation = float(max(abs(r - mean_r) / mean_r for r in radii))
            out['checks']['start_direction'] = dict(
                headings_deg=[r['tags']['heading_deg'] for r in headings], radii_m=radii,
                max_rel_deviation=deviation, tolerance=HEADING_SPREAD_TOL, passed=bool(deviation <= HEADING_SPREAD_TOL),
                friction=[json.loads(f) for f in friction],
                scope='start-direction independence with isotropic wheel friction (mu = mu2); not a check of the '
                      'fdir1 friction basis (that is the diagnostic group fdir_aniso)')
        else:                 # review F5: one start direction cannot show independence
            out['checks']['start_direction'] = dict(
                headings_deg=sorted({r['tags']['heading_deg'] for r in headings}), radii_m=radii, passed=None,
                note='not applicable: this group has one start direction')
        out['checks']['yaw_sign'] = dict(passed=all(r['yaw_sign_ok'] for r in rows))
        out['checks']['spike_free'] = dict(passed=all(r['spikes']['spike_free'] for r in rows))
        if group == 'a1_kinematic':
            out['checks']['A1G_kinematic'] = dict(
                criterion='R error <= 3 %, omega within +-5 % of v tan(delta) / L (commanded v), yaw-rate sign',
                max_abs_radius_error=float(max(abs(r['radius_error']) for r in base)),
                max_abs_yaw_rate_error=float(max(abs(r['yaw_rate_error_vs_cmd']) for r in base)),
                passed=bool(all(abs(r['radius_error']) <= A1_KIN_R_TOL and abs(r['yaw_rate_error_vs_cmd']) <= A1_KIN_W_TOL
                                and r['yaw_sign_ok'] for r in base)))
        else:
            fit_rows = [r for r in rows if r['tags']['heading_deg'] == 0.]
            fit = fit_effective_bicycle([math.tan(math.radians(r['tags']['delta_deg'])) for r in fit_rows],
                                        [r['speed_m_s'] for r in fit_rows], [r['curvature_1_m'] for r in fit_rows],
                                        l_geom)
            per_delta = {}
            for size in sorted({abs(r['tags']['delta_deg']) for r in fit_rows}):
                same = [r for r in fit_rows if abs(r['tags']['delta_deg']) == size]
                ratios = [r['radius_geometry_m'] / r['radius_m'] for r in same]
                by_speed = {r['tags']['v']: r for r in same if r['tags']['delta_deg'] > 0.}
                eta_k0 = float(np.mean(ratios))
                # residuals of the per-delta form kappa = eta(delta) tan(delta) / L (K = 0, amendment 3)
                pred = [eta_k0 * math.tan(math.radians(size)) / l_geom for _ in same]
                curvature_res = [(abs(r['curvature_1_m']) - p) / p for r, p in zip(same, pred)]
                per_delta['%g' % size] = dict(
                    eta_k0=eta_k0, radius_ratio=[r['radius_m'] / r['radius_geometry_m'] for r in same],
                    speed_dependence=(by_speed[.5]['radius_m'] / by_speed[.3]['radius_m'] - 1.)
                    if .3 in by_speed and .5 in by_speed else None,
                    speed_ratio_vs_target=[r['speed_m_s'] / r['tags']['v'] for r in same],
                    cases_delta_deg_v=[[r['tags']['delta_deg'], r['tags']['v']] for r in same],
                    curvature_rel_residual=curvature_res,
                    radius_residual_m=[r['radius_m'] - 1. / p for r, p in zip(same, pred)],
                    max_abs_curvature_rel_residual=float(max(abs(x) for x in curvature_res)))
            sizes = sorted(per_delta, key=float)
            per_delta_summary = dict(
                form='kappa = eta(delta) tan(delta) / L, K = 0 (SIM_ACCEPTANCE amendment 3); L = %g m' % l_geom,
                eta_by_delta_deg={k: per_delta[k]['eta_k0'] for k in sizes},
                max_abs_curvature_rel_residual=float(max(per_delta[k]['max_abs_curvature_rel_residual']
                                                         for k in sizes)),
                linear_fit=fit_eta_linear([float(k) for k in sizes], [per_delta[k]['eta_k0'] for k in sizes]),
                interpolation='between the measured delta values: linear in |delta| with eta(0) = 1 (a choice for '
                              'A12, not measured below 5 deg)')
            name = {'a1_force': 'A1G_force_fit', 'a1_force_ideal': 'diag_force_ideal_fit',
                    'a1_ackermann': 'A1G_ackermann_fit'}[group]
            out['checks'][name] = dict(
                criterion='report: effective L, eta, understeer and residuals; yaw-rate sign', fit=fit,
                per_delta_eta_with_zero_understeer=per_delta, per_delta_eta_summary=per_delta_summary,
                passed=bool(all(r['yaw_sign_ok'] for r in rows)))
            if group in ('a1_force', 'a1_force_ideal'):
                out['checks']['turning_speed_loss'] = dict(
                    criterion='report: G-SIM turning speed loss against the F-SIM drag loss range',
                    passed=None, **turning_speed_loss(fit_rows, ctx.view))
        return out
    if group == 'a4_drive' or group == 'a4_tau_sweep':
        t0, t1 = group_def['window']
        speeds = []
        for spec in group_def['vehicles']:
            a, tags = logs[spec['name']], spec['tags']
            row = dict(tags=tags)
            if tags['kind'] == 'a4_speed':
                sel = _window(a, t0, t1)
                v = float(np.mean(a['speed'][sel]))
                d = tags['duty_pct']
                target = drive_model.steady_speed_m_s(ctx.view, d)
                line = ctx.line.steady_speed_m_s(d / 100.)
                row.update(steady_speed_m_s=v, curve_speed_m_s=target, dc_line_speed_m_s=line,
                           error_vs_curve=(v - target) / target if target else None,
                           error_vs_dc_line=(v - line) / line if line else None,
                           speed_sd_m_s=float(np.std(a['speed'][sel])))
                speeds.append((d / 100., v))
                # time to 90 % of the steady speed after the step at t = 1 s
                reach = np.nonzero(a['speed'] >= .9 * v)[0] if v > 0 else np.array([])
                row['t90_s'] = float(a['t'][reach[0]] - 1.) if reach.size else None
            else:
                stop = stop_distance(a['t'], a['x'], a['y'], a['speed'], tags['t_event'])
                row.update(stop)
                row['release_target_m'] = list(A4_RELEASE_TARGET_M) if tags['event'] == 'release' else None
                row['within_release_target'] = (A4_RELEASE_TARGET_M[0] <= stop['distance_m'] <= A4_RELEASE_TARGET_M[1]
                                                if tags['event'] == 'release' else None)
            row['spikes'] = _spikes(a, [item['key'] for item in manifests[spec['name']]['steering']], rate_bound,
                                    SPIKE_FACTOR * ctx.line.omega0_rad_s)
            out['vehicles'][spec['name']] = row
        if group == 'a4_drive':
            fit = fit_dc_line([s[0] for s in speeds], [s[1] for s in speeds])
            crit = [r for r in out['vehicles'].values() if r['tags']['kind'] == 'a4_speed'
                    and r['tags']['duty_pct'] in A4_SPEED_DUTIES]
            release = [r for r in out['vehicles'].values() if r['tags'].get('event') == 'release']
            out['checks']['A4G_speed'] = dict(
                criterion='steady speed within +-5 % of the main forward curve at duty 40/70/88 %',
                fitted_dc_line=fit, design_dc_line=dict(r_omega0_m_s=ctx.line.r_omega0_m_s, d0=ctx.line.deadband_frac),
                max_abs_error=float(max(abs(r['error_vs_curve']) for r in crit)),
                passed=bool(all(abs(r['error_vs_curve']) <= A4_SPEED_TOL for r in crit)))
            out['checks']['A4G_release_target'] = dict(
                criterion='release stop distance within 0.56-0.60 m +-15 % (0.476-0.690 m); brake reported',
                release_distance_m=release[0]['distance_m'] if release else None,
                brake_distance_m=next((r['distance_m'] for r in out['vehicles'].values()
                                       if r['tags'].get('event') == 'brake'), None),
                passed=bool(release and release[0]['within_release_target']))
        else:
            table = {}
            for r in out['vehicles'].values():
                table.setdefault('%g' % r['tags']['tau_stall_nm'], {})[r['tags']['event']] = r['distance_m']
            out['checks']['diag_tau_sweep'] = dict(
                criterion='report: release and brake stop distances against drive_tau_stall_nm (range 0.5-5)',
                distances_m=table, release_target_m=list(A4_RELEASE_TARGET_M),
                taus_inside_release_target=[tau for tau, row in table.items()
                                            if A4_RELEASE_TARGET_M[0] <= row.get('release', -1.) <= A4_RELEASE_TARGET_M[1]],
                passed=None)
        out['checks']['spike_free'] = dict(passed=all(r['spikes']['spike_free'] for r in out['vehicles'].values()))
        return out
    if group == 'a21_limits':
        ok_all = True
        for spec in group_def['vehicles']:
            a = logs[spec['name']]
            keys = [item['key'] for item in manifests[spec['name']]['steering']]
            limits = {item['key']: (item['lower_rad'], item['upper_rad']) for item in manifests[spec['name']]['steering']}
            ok, worst = _limit_check(a, keys, limits)
            ok_all = ok_all and ok
            out['vehicles'][spec['name']] = dict(tags=spec['tags'], within_limits=ok, joints=worst,
                                                 z_min_m=float(np.min(a['z'])), finite=bool(np.all(np.isfinite(a['x']))))
        out['checks']['A21G'] = dict(criterion='no wheel joint angle beyond its per-wheel mechanical limit '
                                               '(numerical tolerance %g rad)' % LIMIT_TOL_RAD, passed=bool(ok_all))
        return out
    if group in A25_GROUPS:
        all_err = []
        ok_p99 = True
        spike_ok = True
        for spec in group_def['vehicles']:
            a, tags = logs[spec['name']], spec['tags']
            keys = [item['key'] for item in manifests[spec['name']]['steering']]
            errs = []
            per_joint = {}
            for key in keys:
                e = np.degrees(a['steer_%s_q' % key][2:] - a['steer_%s_target' % key][2:])
                per_joint[key] = percentile_stats(e)
                errs.append(e)
            e = np.concatenate(errs)
            all_err.append(e)
            stats = percentile_stats(e)
            spikes = _spikes(a, keys, rate_bound, SPIKE_FACTOR * ctx.line.omega0_rad_s)
            row = dict(tags=tags, error_deg=stats, per_joint=per_joint, spikes=spikes,
                       model_vs_target_note='targets are the shared model wheel angles at the end of each step')
            if tags['case'] in group_def['drift_windows']:
                w0, w1 = group_def['drift_windows'][tags['case']]
                drift = standstill_drift(a['t'], a['x'], a['y'], a['yaw'], w0, w1)
                sel = _window(a, w0, w1)
                drift['steer_joint_change_deg'] = {
                    key: float(np.degrees(np.max(np.abs(a['steer_%s_q' % key][sel] - a['steer_%s_q' % key][sel][0]))))
                    for key in keys}
                drift['passed'] = bool(drift['max_position_change_m'] < DRIFT_LIMIT_M
                                       and drift['max_yaw_change_deg'] < DRIFT_LIMIT_DEG)
                row['standstill_drift'] = drift
            ok_p99 = ok_p99 and stats['p99'] <= A25_P99_LIMIT_DEG
            spike_ok = spike_ok and spikes['spike_free']
            out['vehicles'][spec['name']] = row
        drift_rows = [r['standstill_drift'] for r in out['vehicles'].values() if 'standstill_drift' in r]
        combined = percentile_stats(np.concatenate(all_err))
        check = dict(
            criterion='joint angle - shared model target p99 <= 0.2 deg; 60 s standstill drift < 1 cm and < 0.1 deg; '
                      'no joint speed spikes (|steering qd| <= 1.5 x max model rate, |wheel qd| <= 1.25 x omega0)',
            combined_error_deg=combined, p99_each_vehicle_ok=bool(ok_p99), drift=drift_rows,
            spike_free=bool(spike_ok),
            passed=bool(ok_p99 and spike_ok and all(d['passed'] for d in drift_rows)))
        if group == 'thermal_firmware_report':
            check.update(judgement='report', note='THERMAL report group (old phase 3 standstill steps, %s deg): the '
                         'firmware is expected to latch THERMAL (placeholder budget tha 5000 hot ticks in 30 s); not '
                         'a normal script of A19(G); the MCU lines are in run.controller_summaries'
                         % '/'.join('%g' % v for _, v in group_def['steps_standstill_deg']))
            out['checks']['THERMAL_expected_report'] = check
        elif group in REPORT_GROUPS:
            check.update(judgement='report', note='firmware-mode backend (minimal host: ARM, C lines every %d ms); '
                         'the registered A25 judgement uses a25_l0 and a25_native; the MCU lines are in '
                         'run.controller_summaries; standstill steps %s deg (within the thermal budget, WF-4)'
                         % (FW_C_PERIOD_TICKS, '/'.join('%g' % v for _, v in group_def['steps_standstill_deg'])))
            out['checks']['A25_firmware_report'] = check
        else:
            out['checks']['A25'] = check
        return out
    if group in A12_GROUPS:
        ok = True
        for spec in group_def['vehicles']:
            a = logs[spec['name']]
            finite = bool(all(np.all(np.isfinite(a[k])) for k in ('x', 'y', 'yaw')))
            ok = ok and finite
            out['vehicles'][spec['name']] = dict(
                tags=spec['tags'], start_pose=[spec['x'], spec['y'], spec['yaw']],
                end_pose=[float(a['x'][-1]), float(a['y'][-1]), float(a['yaw'][-1])],
                path_m=float(np.sum(np.hypot(np.diff(a['x']), np.diff(a['y'])))), finite=finite,
                z_min_m=float(np.min(a['z'])))
        out['checks']['A12_G_run'] = dict(criterion='G-SIM side of A12 ran (finite); the A12 judgement is '
                                                    'field_world.a12_twin.compare', passed=bool(ok))
        return out
    if group == 'fdir_aniso':
        tracks = {}
        for spec in group_def['vehicles']:
            a = logs[spec['name']]
            heading = spec['yaw']
            c, s = math.cos(-heading), math.sin(-heading)
            dx, dy = a['x'] - a['x'][0], a['y'] - a['y'][0]
            u, w = c * dx - s * dy, s * dx + c * dy           # base track in the start frame
            yaw = np.unwrap(a['yaw']) - a['yaw'][0]
            body_lateral = -np.sin(a['yaw']) * a['vx'] + np.cos(a['yaw']) * a['vy']
            tracks[spec['name']] = (u, w, yaw)
            half = a['t'] >= group_def['duration_s'] / 2.
            out['vehicles'][spec['name']] = dict(
                tags=spec['tags'], wheel_friction=manifests[spec['name']].get('wheel_friction'),
                end_start_frame_m=[float(u[-1]), float(w[-1])], turned_deg=float(np.degrees(yaw[-1])),
                mean_abs_lateral_speed_m_s=float(np.mean(np.abs(body_lateral[half]))),
                finite=bool(np.all(np.isfinite(a['x'])) and np.all(np.isfinite(a['y']))))
        aniso = [spec['name'] for spec in group_def['vehicles'] if spec['tags']['friction'] == 'anisotropic']
        reference = [spec['name'] for spec in group_def['vehicles'] if spec['tags']['friction'] == 'profile']
        first = tracks[aniso[0]]
        n = min(len(tracks[name][0]) for name in tracks)
        spread = max(float(np.max(np.hypot(tracks[name][0][:n] - first[0][:n], tracks[name][1][:n] - first[1][:n])))
                     for name in aniso[1:])
        yaw_spread = max(float(np.degrees(np.max(np.abs(tracks[name][2][:n] - first[2][:n])))) for name in aniso[1:])
        ref = tracks[reference[0]]
        effect = float(np.max(np.hypot(ref[0][:n] - first[0][:n], ref[1][:n] - first[1][:n])))
        absolute = bool(spread <= FDIR_TRACK_TOL_M and effect >= FDIR_EFFECT_MIN_M)
        relative = spread / effect if effect > 0. else None
        out['checks']['fdir1_turns_with_the_wheel'] = dict(
            criterion='diagnostic (design 3.6 fdir1 assumption, review F3): with anisotropic wheel friction (mu lateral '
                      '%g, mu2 longitudinal %g) the start-frame tracks of start directions %s agree within %g of the '
                      'anisotropy effect (relative form, decision R19-14), and differ from the profile-friction track '
                      'by at least %g m (the anisotropy acted)'
                      % (FDIR_MU_LAT, FDIR_MU_LONG, '/'.join('%g' % h for h in FDIR_HEADINGS_DEG), FDIR_TRACK_REL_TOL,
                         FDIR_EFFECT_MIN_M),
            max_track_difference_m=spread, max_yaw_difference_deg=yaw_spread, anisotropy_effect_m=effect,
            relative_track_difference=relative, relative_tolerance=FDIR_TRACK_REL_TOL,
            absolute_form=dict(tolerance_m=FDIR_TRACK_TOL_M, passed=absolute,
                               note='the phase 3 absolute form (1e-6 m) stays reported: it was false with 1.37e-6 m '
                                    'against an effect of 11.9 m'),
            passed=bool(relative is not None and relative <= FDIR_TRACK_REL_TOL and effect >= FDIR_EFFECT_MIN_M))
        out['checks']['finite'] = dict(passed=all(row['finite'] for row in out['vehicles'].values()))
        return out
    raise ValueError('unknown_group:' + group)


# =================================================================================================================
# plots
# =================================================================================================================
def plot_group(group, group_def, ctx, logs, result, out_dir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as exc:                       # plots are optional
        return dict(skipped=str(exc))
    files = []
    if group.startswith('a1_') or group == 'fdir_aniso':
        fig, ax = plt.subplots(figsize=(7, 7))
        for spec in group_def['vehicles']:
            a = logs[spec['name']]
            xr, yr = point_track(a['x'], a['y'], a['yaw'], ctx.geometry.rear_axle_x_in_base_m)
            ax.plot(xr - spec['x'], yr - spec['y'], lw=1,
                    label='%s d=%g v=%g h=%g' % (spec['name'], spec['tags']['delta_deg'], spec['tags']['v'],
                                                 spec['tags']['heading_deg']))
        ax.set_aspect('equal')
        ax.set_xlabel('x - x0 (m, SIM frame)')
        ax.set_ylabel('y - y0 (m, SIM frame)')
        ax.set_title('%s rear-axle tracks (%s)' % (group, PLOT_LABEL), fontsize=9)
        ax.legend(fontsize=6)
        path = Path(out_dir) / ('%s_tracks.png' % group)
        fig.savefig(path, dpi=110)
        plt.close(fig)
        files.append(path.name)
    elif group.startswith('a4_'):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for spec in group_def['vehicles']:
            a = logs[spec['name']]
            label = ('duty %g%%' % spec['tags']['duty_pct']) if spec['tags']['kind'] == 'a4_speed' else \
                ('%s tau %s' % (spec['tags']['event'], spec['tags']['tau_stall_nm']))
            ax.plot(a['t'], a['speed'], lw=1, label=label)
        ax.set_xlabel('SIM time (s)')
        ax.set_ylabel('base speed (m/s)')
        ax.set_title('%s (%s)' % (group, PLOT_LABEL), fontsize=9)
        ax.legend(fontsize=6, ncol=2)
        path = Path(out_dir) / ('%s_speed.png' % group)
        fig.savefig(path, dpi=110)
        plt.close(fig)
        files.append(path.name)
    elif group.startswith('a21') or group.startswith('a25'):
        fig, axes = plt.subplots(len(group_def['vehicles']), 1, figsize=(9, 2.0 * len(group_def['vehicles'])),
                                 sharex=True)
        for ax, spec in zip(np.atleast_1d(axes), group_def['vehicles']):
            a = logs[spec['name']]
            for key in [item['key'] for item in result_steer_keys(result, spec)]:
                ax.plot(a['t'], np.degrees(a['steer_%s_q' % key]), lw=1, label='%s joint' % key)
                ax.plot(a['t'], np.degrees(a['steer_%s_target' % key]), lw=.8, ls='--', label='%s target' % key)
            ax.set_ylabel('deg')
            ax.set_title('%s %s' % (spec['name'], spec['tags'].get('case', spec['tags'].get('how'))), fontsize=8)
            ax.legend(fontsize=6, loc='upper right')
        np.atleast_1d(axes)[-1].set_xlabel('SIM time (s)')
        fig.suptitle('%s (%s)' % (group, PLOT_LABEL), fontsize=9)
        fig.tight_layout()
        path = Path(out_dir) / ('%s_steering.png' % group)
        fig.savefig(path, dpi=100)
        plt.close(fig)
        files.append(path.name)
    return dict(files=files)


def result_steer_keys(result, spec):
    return [dict(key=k) for k in (('axle',) if spec.get('steer_type') == geo.AXLE_PIVOT else ('left', 'right'))]


# =================================================================================================================
# commands: run (fixture groups)
# =================================================================================================================
def run_group(group, out_root, profile, *, quick=False, model_options=None):
    """Run one fixture group. model_options (for example {'suspension': True}) go to every vehicle's transform."""
    out_dir = Path(out_root) / group
    out_dir.mkdir(parents=True, exist_ok=True)
    group_def = group_definition(group, profile)
    if model_options:
        group_def['model_options_all'] = dict(model_options)
    if quick:
        group_def['duration_s'] = min(group_def['duration_s'], 12.)
        if 'window' in group_def:
            group_def['window'] = (min(group_def['window'][0], 4.), 12.)
        for key in list(group_def.get('drift_windows', {})):
            group_def['drift_windows'][key] = (4., 12.)
    world, ctx, manifests = build_probe_world(group_def, out_dir, profile)
    for spec in group_def['vehicles']:
        spec['steer_type'] = ctx.steer_type
    started = time.perf_counter()
    logs, info = run_fixture(world, group_def, ctx, manifests,
                             progress=lambda text: print('[%s] %s' % (group, text), flush=True))
    np.savez_compressed(out_dir / 'logs.npz', **{'%s__%s' % (name, key): value for name, arrays in logs.items()
                                                 for key, value in arrays.items()})
    if info['errors']:
        result = dict(group=group, errors=info['errors'], run=info)
    else:
        result = analyze_group(group, group_def, ctx, logs, manifests)
        result['plots'] = plot_group(group, group_def, ctx, logs, result, out_dir)
    flags = sorted({flag for m in manifests.values() for flag in (m.get('result_flags') or [])})
    result.update(schema=SCHEMA, result_kind=RESULT_KIND, field_validated=False, run=info,
                  diagnostic=group in DIAGNOSTIC_GROUPS, profile_overrides=group_def.get('profile_overrides'),
                  model_options=group_def.get('model_options_all'), model_result_flags=flags,
                  wheel_collision=sorted({m.get('wheel_collision') for m in manifests.values()}),
                  mass_check=next(iter(manifests.values())).get('mass_check') if manifests else None,
                  profile=ctx.view.result_label(), engine='fixture', duration_s=group_def['duration_s'],
                  quick=quick, total_wall_s=time.perf_counter() - started,
                  normal_script=group not in NOT_NORMAL_GROUPS,
                  normal_script_note=None if group not in NOT_NORMAL_GROUPS else
                  'faults are expected in this group (A19(G) does not count it as a normal script)',
                  provenance=provenance(world, info.get('actuator_backends')),
                  vehicle_specs={s['name']: dict(pose=[s['x'], s['y'], math.degrees(s['yaw'])], tags=s['tags'],
                                                 model_options=s['model_options']) for s in group_def['vehicles']})
    write_json(out_dir / 'result.json', result)
    return result


# =================================================================================================================
# server engine (real-time gz sim -s -r): A10 and scripted drives
# =================================================================================================================
def _gz_sim_command(sdf, manifest, verbosity=2):
    renderer = manifest['renderer']
    command = ['gz', 'sim', '-s', '-r', '-v', str(verbosity), '--render-engine-server', renderer['engine']]
    if renderer['headless']:
        command.append('--headless-rendering')
    return command + [str(sdf)]


def _camera_rate(frames, t_ref, warmup_s):
    """Frame count, size and rates (wall, SIM) after the warm-up from (wall, stamp, width, height) rows."""
    f = np.asarray(frames, dtype=float) if frames else np.zeros((0, 4))
    if not len(f):
        return None
    fw = f[f[:, 0] - t_ref >= warmup_s]
    return dict(frames=int(len(f)), size=[int(f[0, 2]), int(f[0, 3])],
                wall_fps=float((len(fw) - 1) / (fw[-1, 0] - fw[0, 0])) if len(fw) > 2 else None,
                sim_fps=float((len(fw) - 1) / (fw[-1, 1] - fw[0, 1])) if len(fw) > 2 and fw[-1, 1] > fw[0, 1]
                else None)


def run_server(scene_dir, out_dir, *, duration_s, steer_deg=0., duty=0., warmup_s=20., label='server',
               camera_subscriber=True, profile=None, extra_camera_topics=()):
    """Run the scene at real-time pacing for duration_s wall seconds while driving it. Returns a result dict.

    extra_camera_topics: more camera topics (for example demo cameras) whose frame rates are recorded too."""
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.transport13 import Node

    scene_dir, out_dir = Path(scene_dir), Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((scene_dir / 'scene_manifest.json').read_text(encoding='utf-8'))
    sdf = scene_dir / manifest['sdf']
    profile = load_profile() if profile is None else profile
    view = gz_assemble.scene_profile(manifest['scene'], profile)
    ctx = Context(view, 1. / float(view.value('gz_physics_hz')))
    topics = manifest['model']['topics']
    node = Node()
    stats, frames, odom = [], [], []
    node.subscribe(WorldStatistics, '/world/%s/stats' % manifest['world_name'],
                   lambda m: stats.append((time.monotonic(), m.sim_time.sec + m.sim_time.nsec * 1e-9,
                                           m.real_time.sec + m.real_time.nsec * 1e-9, m.real_time_factor,
                                           m.iterations)))
    if camera_subscriber and manifest['renderer']['camera']:
        node.subscribe(Image, '/sim/camera', lambda m: frames.append((time.monotonic(), m.header.stamp.sec +
                                                                       m.header.stamp.nsec * 1e-9, m.width, m.height)))
    extra_frames = {topic: [] for topic in extra_camera_topics}
    for topic in extra_camera_topics:
        node.subscribe(Image, topic, lambda m, rows=extra_frames[topic]: rows.append(
            (time.monotonic(), m.header.stamp.sec + m.header.stamp.nsec * 1e-9, m.width, m.height)))
    node.subscribe(Odometry, topics['truth_odometry_gz'],
                   lambda m: odom.append((m.header.stamp.sec + m.header.stamp.nsec * 1e-9, m.pose.position.x,
                                          m.pose.position.y)))
    pub_steer = {k: node.advertise(t, Double) for k, t in topics['steer'].items()}
    pub_drive = {w: node.advertise(t, Double) for w, t in topics['drive'].items()}
    log_path = out_dir / ('%s_gz.log' % label)
    command = _gz_sim_command(sdf, manifest)
    env = dict(os.environ)
    with log_path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
        started = time.monotonic()
        error = None
        first_stats_wall_s = None
        try:
            while not stats:
                if process.poll() is not None or time.monotonic() - started > 120.:
                    raise RuntimeError('gz sim did not publish world statistics (exit %r)' % process.poll())
                time.sleep(.1)
            t_first = time.monotonic()
            first_stats_wall_s = t_first - started
            targets = ctx.steer_targets(math.radians(steer_deg))
            while time.monotonic() - t_first < duration_s:
                if process.poll() is not None:
                    raise RuntimeError('gz sim exited %r' % process.returncode)
                elapsed = time.monotonic() - t_first
                for key, pub in pub_steer.items():
                    msg = Double()
                    msg.data = ramp(elapsed, 1., 3., 0., targets[key])
                    pub.publish(msg)
                for wheel, value in ctx.drive_targets(duty if elapsed > 3. else 0.).items():
                    msg = Double()
                    msg.data = value
                    pub_drive[wheel].publish(msg)
                time.sleep(.05)
        except Exception as exc:
            error = str(exc)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    s = np.asarray(stats, dtype=float) if stats else np.zeros((0, 5))
    keep = s[:, 0] - (s[0, 0] if len(s) else 0.) >= warmup_s if len(s) else np.zeros(0, bool)
    windows = rtf_windows(s[keep, 1], s[keep, 2]) if keep.sum() > 2 else []
    f = np.asarray(frames, dtype=float) if frames else np.zeros((0, 4))
    fkeep = f[:, 0] - (s[0, 0] if len(s) else 0.) >= warmup_s if len(f) else np.zeros(0, bool)
    camera = None
    if len(f):
        fw = f[fkeep]
        camera = dict(frames=int(len(f)), size=[int(f[0, 2]), int(f[0, 3])],
                      wall_fps=float((len(fw) - 1) / (fw[-1, 0] - fw[0, 0])) if len(fw) > 2 else None,
                      sim_fps=float((len(fw) - 1) / (fw[-1, 1] - fw[0, 1])) if len(fw) > 2 and fw[-1, 1] > fw[0, 1]
                      else None)
    o = np.asarray(odom, dtype=float) if odom else np.zeros((0, 3))
    result = dict(schema=SCHEMA, engine='server', label=label, scene=manifest['scene'], command=command,
                  duration_wall_s=duration_s, warmup_s=warmup_s, error=error,
                  stats_samples=int(len(s)), rtf_window_s=5., rtf_windows=windows,
                  rtf_median=float(np.median(windows)) if windows else None,
                  rtf_p10=float(np.percentile(windows, 10)) if windows else None,
                  rtf_min=float(np.min(windows)) if windows else None,
                  reported_rtf_median=float(np.median(s[keep, 3])) if keep.sum() else None,
                  sim_time_end_s=float(s[-1, 1]) if len(s) else None, camera=camera,
                  drive=dict(steer_deg=steer_deg, duty=duty), truth_samples=int(len(o)),
                  truth_path_m=float(np.sum(np.hypot(np.diff(o[:, 1]), np.diff(o[:, 2])))) if len(o) > 1 else 0.,
                  result_kind=RESULT_KIND, field_validated=False, profile=view.result_label(),
                  wheel_collision=manifest['model'].get('wheel_collision'),
                  model_result_flags=list(manifest.get('result_flags') or []), provenance=provenance(sdf),
                  first_stats_wall_s=first_stats_wall_s,
                  scene_size=dict(sdf_bytes=sdf.stat().st_size,
                                  field=(manifest.get('world_summary') or {}).get('boxes')))
    if extra_camera_topics:
        result['extra_cameras'] = {topic: _camera_rate(rows, s[0, 0] if len(s) else 0., warmup_s)
                                   for topic, rows in extra_frames.items()}
    np.savez_compressed(out_dir / ('%s_series.npz' % label), stats=s, frames=f, odom=o)
    write_json(out_dir / ('%s_result.json' % label), result)
    return result


def run_rtf(out_root, profile, *, duration_s, camera_modes=('off', 'on'), steer_deg=10., duty=None,
            model_options=None, scene='w0_force', field_spec=None):
    """A10(W0): build W0 force scenes (camera off / on) and run each at real-time pacing. model_options go to the
    rover transform (for example wheel_collision='cylinder'). scene field1010 with a field_spec: A10(W2), the same
    measurement on the field world (first measurement; the registered draft criterion is >= 0.8)."""
    out_root = Path(out_root)
    results = {}
    for mode in camera_modes:
        scene_dir = out_root / ('%s_camera_%s' % (scene, mode))
        gz_assemble.build_scene(scene, scene_dir, profile=profile, camera=(mode == 'on'),
                                model_options=model_options, field_spec=field_spec)
        view = gz_assemble.scene_profile(scene, profile)
        ctx = Context(view, 1. / float(view.value('gz_physics_hz')))
        d = ctx.duty_for_speed(.4) if duty is None else duty
        results[mode] = run_server(scene_dir, out_root, duration_s=duration_s, steer_deg=steer_deg, duty=d,
                                   label='a10_%s_camera_%s' % (scene, mode), profile=profile)
    if scene == 'w0_force':
        check, criterion, limit = 'A10_W0', 'W0 RTF median >= 0.9 (GUI off, camera 640x360)', .9
    else:
        check, limit = 'A10_W2', .8
        criterion = ('W2 mode R RTF median >= 0.8 (registered draft, first measurement then fixed; GUI off, camera '
                     '640x360)')
    summary = dict(schema=SCHEMA, check=check, criterion=criterion, scene=scene,
                   results={mode: dict(rtf_median=r['rtf_median'], rtf_p10=r['rtf_p10'], rtf_min=r['rtf_min'],
                                       reported_rtf_median=r['reported_rtf_median'], camera=r['camera'],
                                       truth_path_m=r.get('truth_path_m'), error=r['error'])
                            for mode, r in results.items()},
                   model_options=model_options, wheel_collision=sorted({r.get('wheel_collision') for r in
                                                                        results.values()}),
                   result_kind=RESULT_KIND, field_validated=False,
                   provenance={mode: r.get('provenance') for mode, r in results.items()})
    for mode, r in results.items():
        summary['results'][mode]['passed'] = bool(r['rtf_median'] is not None and r['rtf_median'] >= limit)
        summary['results'][mode]['first_stats_wall_s'] = r.get('first_stats_wall_s')
        summary['results'][mode]['scene_size'] = r.get('scene_size')
    write_json(out_root / ('a10_summary.json' if scene == 'w0_force' else 'a10_%s_summary.json' % scene), summary)
    return summary


def run_async_track(out_root, profile, *, duration_s=60., seed=0):
    """Report only: the shared L0 model drives the steering joints of the real-time server from another process.

    The probe receives the truth joint states (every physics step in this scene variant), steps the L0 model to
    the end of the next step and publishes the targets over gz-transport, as the P6a emulator will. The error is
    the joint angle minus the model angle at the same SIM time; late commands show up as a one-step lag.
    """
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.model_pb2 import Model
    from gz.transport13 import Node
    import threading

    out_root = Path(out_root)
    scene_dir = out_root / 'w0_force_track'
    gz_assemble.build_scene('w0_force', scene_dir, profile=profile, camera=False)
    sdf = scene_dir / 'world.sdf'
    tree = E.parse(sdf)
    for plugin in tree.getroot().iter('plugin'):
        kind = (plugin.get('name') or '').rsplit('::', 1)[-1]
        if kind == 'JointStatePublisher':
            plugin.find('update_rate').text = '1000'
    tree.write(sdf, encoding='unicode')
    manifest = json.loads((scene_dir / 'scene_manifest.json').read_text(encoding='utf-8'))
    manifest['sdf'] = sdf.name
    view = gz_assemble.scene_profile('w0_force', profile)
    ctx = Context(view, int(1e9 / float(view.value('gz_physics_hz'))) / 1e9)
    backend = L0ServoBackend()
    backend.configure(view, seed)
    backend.reset(0.)
    schedule = [(0., 0.), (2., 20.), (10., -20.), (18., 5.), (26., 0.)]

    def target(t):
        if t < 30.:
            return math.radians(piecewise(t, schedule))
        return math.radians(10.) * math.sin(2. * math.pi * .2 * (t - 30.))

    duty = ctx.duty_for_speed(.4)
    node = Node()
    pubs = {key: node.advertise(topic, Double) for key, topic in manifest['model']['topics']['steer'].items()}
    drive = {wheel: node.advertise(topic, Double) for wheel, topic in manifest['model']['topics']['drive'].items()}
    lock = threading.Lock()
    state = dict(k_model=0, rows=[], model={}, error=None)
    joint_names = {item['joint']: item['key'] for item in manifest['model']['steering']}

    def on_joints(msg):
        try:
            t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            k = int(round(t / ctx.dt))
            q = {joint_names[j.name]: j.axis1.position for j in msg.joint if j.name in joint_names}
            with lock:
                state['rows'].append((time.monotonic(), k, t, q.get('left', float('nan')), q.get('right', float('nan'))))
                while state['k_model'] < k + 1:
                    state['k_model'] += 1
                    tm = state['k_model'] * ctx.dt
                    out = backend.step(ba.SteerStepInput(dt_s=ctx.dt, mode='test_servo', target_delta_eq_rad=target(tm),
                                                         vehicle_speed_m_s=0.4 if tm > 31. else 0.,
                                                         vehicle_stopped=tm <= 31.))
                    state['model'][state['k_model']] = (out.delta_left_rad, out.delta_right_rad)
                left, right = state['model'][k + 1]
            for key, value in (('left', left), ('right', right)):
                m = Double()
                m.data = value
                pubs[key].publish(m)
            for wheel, pub in drive.items():
                m = Double()
                m.data = ctx.drive_targets(duty)[wheel] if t > 31. else 0.
                pub.publish(m)
        except Exception as exc:            # report after the run
            state['error'] = '%s: %s' % (type(exc).__name__, exc)

    node.subscribe(Model, manifest['model']['topics']['truth_joint_states_gz'], on_joints)
    log_path = out_root / 'track_gz.log'
    command = _gz_sim_command(sdf, manifest)
    with log_path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                   env=dict(os.environ))
        started = time.monotonic()
        try:
            while time.monotonic() - started < duration_s + 15.:
                if process.poll() is not None:
                    state['error'] = state['error'] or 'gz sim exited %r' % process.returncode
                    break
                with lock:
                    if state['rows'] and state['rows'][-1][2] >= duration_s:
                        break
                time.sleep(.2)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    backend.close()
    with lock:
        rows = list(state['rows'])
        model = dict(state['model'])
    errors, lags = [], dict(on_time=0, one_step_late=0, other=0)
    for _, k, t, ql, qr in rows[5:]:
        if k not in model or (k - 1) not in model:
            continue
        for value, index in ((ql, 0), (qr, 1)):
            err = math.degrees(value - model[k][index])
            errors.append(err)
            if abs(err) < 1e-6:
                lags['on_time'] += 1
            elif abs(value - model[k - 1][index]) < 1e-9:
                lags['one_step_late'] += 1
            else:
                lags['other'] += 1
    steps = [b[1] - a[1] for a, b in zip(rows, rows[1:])]
    result = dict(schema=SCHEMA, check='A25_async_report', engine='server', command=command, duration_s=duration_s,
                  samples=len(rows), error_deg=percentile_stats(errors), classification=lags,
                  missed_joint_state_steps=int(sum(max(0, s - 1) for s in steps)),
                  note='report only; the registered A25 criterion is judged in lockstep (fixture engine)',
                  error=state['error'], result_kind=RESULT_KIND, field_validated=False, profile=view.result_label())
    write_json(out_root / 'track_result.json', result)
    return result


# =================================================================================================================
# report
# =================================================================================================================
def build_report(out_root):
    """Collect group results into ACCEPTANCE_P5.json (outside the repository)."""
    out_root = Path(out_root)
    rows = {}
    for group in GROUPS:
        path = out_root / group / 'result.json'
        if path.exists():
            rows[group] = json.loads(path.read_text(encoding='utf-8'))
    a10 = out_root / 'a10' / 'a10_summary.json'
    report = dict(schema=SCHEMA, result_kind=RESULT_KIND, field_validated=False, groups={}, a10=None)
    for group, result in rows.items():
        source = (result.get('provenance') or {}).get('source') or {}
        report['groups'][group] = dict(checks=result.get('checks'), errors=result.get('errors'),
                                       run=result.get('run'), quick=result.get('quick'),
                                       diagnostic=result.get('diagnostic'), model_options=result.get('model_options'),
                                       model_result_flags=result.get('model_result_flags'),
                                       source_commit=source.get('source_commit'),
                                       source_files_sha256=(source.get('source_files') or {}).get('files_sha256'),
                                       world_sdf_sha256=((result.get('provenance') or {}).get('world_sdf') or {}).get(
                                           'sha256'))
    if a10.exists():
        report['a10'] = json.loads(a10.read_text(encoding='utf-8'))
    for name, path in (('a25_async_report', out_root / 'track' / 'track_result.json'),
                       ('w1_drive_report', out_root / 'w1_drive' / 'drive_result.json')):
        if path.exists():
            report[name] = json.loads(path.read_text(encoding='utf-8'))
    write_json(out_root / 'ACCEPTANCE_P5.json', report)
    return report


# =================================================================================================================
# CLI
# =================================================================================================================
def parse_model_options(items):
    """NAME=VALUE strings -> steer_model_element options: true / false, a number, else the text (e.g.
    wheel_collision=sphere)."""
    options = {}
    for item in items:
        name, sep, text = item.partition('=')
        if not sep or not name.strip():
            raise ValueError('model option needs NAME=VALUE: %r' % (item,))
        low = text.strip().lower()
        if low in ('true', 'false'):
            value = low == 'true'
        else:
            try:
                value = float(text)
            except ValueError:
                value = text.strip()
        options[name.strip()] = value
    return options


def main(argv=None):
    parser = argparse.ArgumentParser(description='Headless Gazebo probe of the steered rover (design 8.3 P5).')
    sub = parser.add_subparsers(dest='command', required=True)
    p_run = sub.add_parser('run', help='lockstep fixture groups (A1, A4, A21, A25, A12 twin)')
    p_run.add_argument('groups', nargs='+', help='group names or all (%s)' % ', '.join(GROUPS))
    p_run.add_argument('--out', type=Path, required=True)
    p_run.add_argument('--profile', type=Path)
    p_run.add_argument('--quick', action='store_true', help='12 s runs (smoke test, not acceptance)')
    p_run.add_argument('--model-option', action='append', default=[], metavar='NAME=VALUE',
                       help='gz_model.steer_model_element option for every vehicle (true/false/number/text), '
                            'for example suspension=true or wheel_collision=sphere')
    p_rtf = sub.add_parser('rtf', help='A10(W0): real-time factor of W0 at real-time pacing')
    p_rtf.add_argument('--out', type=Path, required=True)
    p_rtf.add_argument('--profile', type=Path)
    p_rtf.add_argument('--duration', type=float, default=240.)
    p_rtf.add_argument('--camera', choices=('off', 'on', 'both'), default='both')
    p_rtf.add_argument('--model-option', action='append', default=[], metavar='NAME=VALUE',
                       help='gz_model.steer_model_element option of the rover, for example wheel_collision=cylinder')
    p_rtf.add_argument('--scene', choices=('w0_force', 'field1010'), default='w0_force')
    p_rtf.add_argument('--field-spec', type=Path, help='field1010: field_world spec JSON file')
    p_rtf.add_argument('--steer-deg', type=float, default=None, help='default 10 (W0) or 0 (field1010)')
    p_drive = sub.add_parser('drive', help='scripted real-time drive of a built scene (demo helper)')
    p_drive.add_argument('--scene-dir', type=Path, required=True)
    p_drive.add_argument('--out', type=Path, required=True)
    p_drive.add_argument('--profile', type=Path)
    p_drive.add_argument('--duration', type=float, default=60.)
    p_drive.add_argument('--steer-deg', type=float, default=10.)
    p_drive.add_argument('--duty', type=float, default=.48)
    p_track = sub.add_parser('track', help='report: shared L0 model driving the real-time server asynchronously')
    p_track.add_argument('--out', type=Path, required=True)
    p_track.add_argument('--profile', type=Path)
    p_track.add_argument('--duration', type=float, default=60.)
    p_report = sub.add_parser('report', help='collect results into ACCEPTANCE_P5.json')
    p_report.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.command == 'track':
        result = run_async_track(args.out, load_profile(args.profile), duration_s=args.duration)
        print(json.dumps({k: result[k] for k in ('error_deg', 'classification', 'missed_joint_state_steps', 'error')},
                         default=_json_default))
        return 0
    if args.command == 'run':
        profile = load_profile(args.profile)
        groups = list(GROUPS) if 'all' in args.groups else args.groups
        for group in groups:
            if group not in GROUPS:
                parser.error('unknown group %r' % group)
        options = parse_model_options(args.model_option)
        summary = {}
        for group in groups:
            result = run_group(group, args.out, profile, quick=args.quick, model_options=options or None)
            summary[group] = dict(checks={k: v.get('passed') for k, v in (result.get('checks') or {}).items()},
                                  errors=result.get('errors'), wall_s=result.get('total_wall_s'))
            print(json.dumps({group: summary[group]}, default=_json_default), flush=True)
        build_report(args.out)
        return 0
    if args.command == 'rtf':
        profile = load_profile(args.profile)
        modes = ('off', 'on') if args.camera == 'both' else (args.camera,)
        if args.scene == 'field1010' and args.field_spec is None:
            parser.error('rtf --scene field1010 needs --field-spec')
        steer = args.steer_deg if args.steer_deg is not None else (10. if args.scene == 'w0_force' else 0.)
        summary = run_rtf(args.out, profile, duration_s=args.duration, camera_modes=modes, steer_deg=steer,
                          model_options=parse_model_options(args.model_option) or None, scene=args.scene,
                          field_spec=args.field_spec)
        print(json.dumps(summary, indent=1, default=_json_default))
        return 0
    if args.command == 'drive':
        profile = load_profile(args.profile)
        result = run_server(args.scene_dir, args.out, duration_s=args.duration, steer_deg=args.steer_deg,
                            duty=args.duty, warmup_s=0., label='drive', profile=profile)
        print(json.dumps({k: result[k] for k in ('rtf_median', 'camera', 'truth_path_m', 'error')},
                         default=_json_default))
        return 0
    report = build_report(args.out)
    print(json.dumps(report, indent=1, default=_json_default)[:4000])
    return 0


if __name__ == '__main__':
    sys.exit(main())
