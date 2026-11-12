"""Steering fast-SIM runner (F-SIM; design 2.1, 4.5, 7.2 A1 A4 A8 A11, 7.3, 8.3 P2).

Runs scenes (scene_spec.SceneSpec) on the steered plant (steered_plant.SteeredPlant) in one of two ways:

  runner 'direct'   the fast_sim plant loop: plant 100 Hz, driver 20 Hz, commands delivered after
                    command_latency_s through a queue, exactly the timing of closed_loop.harness.run_episode, so a
                    maneuver gives the SAME plant trajectory in both runners (sensor synthesis and the estimator do
                    not touch the plant). Optional stop rules end an episode early.
  runner 'harness'  closed_loop.harness.run_episode with the hooks plant_factory (a SteeredPlant subclass that
                    records the trace) and actuation (ScriptActuation, a SteeringActuation that also handles the
                    maneuvers' stop modes); world hooks flat_world (W0) or raster_corridor (W2, and W3: the
                    procedural orchard of procedural_orchard.py over the same RasterCorridorWorld); estimator 'sensor'
                    (SensorEstimator) or 'null'. The driver is wrapped as the harness controller. The harness's own
                    collision end and clearance metrics use the scene profile's footprint rectangle (EpisodeConfig
                    plant_overrides footprint_length_m / footprint_width_m; the only overrides the plant factory
                    accepts), not the differential FieldResponseParams default. Needs the harness imports (scipy), so
                    not on the WSL Python 3.10 (decision R10).

Steering backend per scene (backend 'profile' | 'L0' | 'L1'): L0 = l0_servo.L0ServoBackend, L1 =
actuator_link.NativeSteeringBackend (a C++ lockstep process per episode). run_scenes() builds the native binary
once (native.build.ensure_built) into native_build_dir or UBTS_STEER_BUILD_DIR, which must be set and outside the
repository (no build next to the sources), and passes the same file to every episode and worker, so all episodes of
a run use one binary even if the sources change meanwhile. Every plant is closed after its episode (plant.close()
stops the native process), also when the episode fails.

Run folder (given by the caller, must be outside every work tree of the repository; design 4.5):
  RUN.json                  run identity (scenes with their SHA-256, code identity, native binary, workers)
  EPISODES.jsonl            one record per episode, in scene order then seed order (independent of workers);
                            run_scene_sha256 = the scene SHA-256 of this run (a reused record keeps scene_sha256 of
                            the run that made it)
  SUMMARY.json              counts, per-scene ends, trace SHA set, A11 timing (pooled over the episodes run)
  trace/<episode_id>.npz    the trace (deterministic zip: fixed member times, sorted members)
  timing/<episode_id>.npz   wall-clock per step (plant step, backend step, loop step; float32 us) - not hashed
  episodes/<episode_id>.json  the episode record (used to skip an episode on a later run)
An episode is skipped only when its record exists with status ok, the same episode scene SHA-256, the same
identity SHA-256 (code files, profile, native binary, Python, numpy, platform, and for W2 the catalogue and the map
SHA-256) and the trace file SHA-256 still matches; force=True runs everything. After an episode the worker hashes
the code files again; if they changed since the run was planned the episode is recorded as failed
(code_identity_changed), so a record never names code that did not run.

Trace (npz, one row per recorded plant step; trace_every in the scene; units in the member name):
  t_s                       SIM time at the end of the step
  cmd_throttle_pct          drive command in effect (after the latency queue; 0 before the first command)
  cmd_steer_target_mrad     steering target command in effect (NaN before the first command: the plant then
                            holds its start angle)
  cmd_stop_mode             0 none, 1 release, 2 brake, 3 coast (stop requested by the command in effect)
  phase                     index into phase_names (phase of the command in effect; 'start' before the first)
  steer_target_rad          servo target after the equivalent soft-limit clamp (host / MCU side); after a
                            hold_measured stop it is the plant's hold target: the MCU-side filtered potentiometer
                            measurement at the stop (SteeredPlant.measured_hold_rad(), R13-2; meta steer_target_note)
  drive_request_pct, drive_applied_pct   requested duty and the MCU ramp output (MCU side, not a speed)
  drive_en, steer_en        commanded enables
  pot_raw, steer_measured_rad, fault_bits   potentiometer ADC count, its nominal decode, MCU fault mask
  truth_*                   SIM TRUTH, evaluation only (never an executor / estimator input): base pose
                            truth_x_m, truth_y_m, truth_yaw_rad (world frame: W0 the floor, W2 raster-local m),
                            truth_yaw_unwrapped_rad, truth_field_x_m / truth_field_y_m / truth_field_heading_rad
                            (W0 = world; W2 = corridor s, d, heading error), truth_rear_x_m / truth_rear_y_m (rear
                            axle), truth_v_m_s, truth_w_rad_s (steering yaw rate, without external yaw),
                            truth_external_yaw_rate_rad_s, truth_distance_m, truth_curvature_1_m,
                            truth_steer_curvature_1_m, truth_wander_deg_per_m, truth_lateral_slip_m_s,
                            truth_delta_eq_rad, truth_delta_left_rad, truth_delta_right_rad, truth_motor_side_rad,
                            truth_applied_steer_pwm, truth_model_flags, truth_decelerating, truth_regime
                            (0 still, 1 drive, 2 stopping, 3 blocked), truth_servo_view_rad (the servo-side angle
                            the test servo sees, SteeredPlant.servo_view_rad()), truth_hold_position_rad (the
                            evaluator's hold reference, R13-3: the hold target a hold_measured stop applied in this
                            row's span, else SteeredPlant.measured_hold_rad() after the row's step; audit AUD3A-10:
                            exact also when trace_every > 1)
  ctrl_*                    controller values (probe_pure_pursuit: its observation, made from TRUTH + noise)
  phase_names, meta_json    phase names; trace meta (schema, ids, SHAs, dt, units, the list of truth keys)
content_sha256 = SHA-256 over (name, dtype, shape, bytes) of every member in sorted name order; A8 compares it.

Determinism (A8): each episode is isolated (fresh plant and backend, seeded streams from (seed, stream key,
purpose)); the parent assembles results in scene / seed order; so traces do not depend on the worker count. Same
platform only (decision R1). Workers are spawned (Windows safe) and wait while the HOLD_MEASUREMENT file exists
(closed_loop.external_paths.hold_file: UBTS_HOLD_DIR or 'hold_dir' of the UBTS_STEER_PATHS file; no built-in
path). OPENBLAS / OMP / MKL threads default to 1 (set in the parent before the pool starts).

Timing (A11, report only): per plant step the wall time of SteeredPlant.step (plant_step), of the backend step in
it (backend_step: the actuator model; the MCU logic joins it in P3) and of the whole loop iteration (loop_step:
direct = driver + plant + recording; harness = everything the harness does per plant step; the first sample starts
at the first plant step, so the episode set-up - world, plant, native process start, estimator, LiDAR - is not in it);
per episode the wall clock. This is the cost of the fast SIM, not of the executor (dev_r3 measured
ExecutorLoop.step).

Plant tables (scene profile.plant_tables, the A12 twin form of SIM_ACCEPTANCE amendment 3): the plant is a
TableSteeredPlant: the steering efficiency is eta(|delta_eq|) of the table (linear in |delta|, eta(0) = 1 unless
given), the understeer gradient is 0 and the scrub speed loss is loss(|delta_eq|, v) of the loss table (bilinear,
speed clamped to the table speeds, loss(0, v) = 0) instead of the profile's per-run draws. Used for A12 only.

Every result from the placeholder profile is a placeholder-based SIM result (자리값 기반 SIM 결과), not field
validation. CLI: python -B -m steering.fast_sim --help (run from sim/ubts).
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import heapq
import io
import json
import math
import multiprocessing
import os
from pathlib import Path
import platform
import statistics
import sys
import time
import traceback
import zipfile
import zlib

import numpy as np

from . import backend_api as ba
from . import drive
from . import flat_world as fw
from . import maneuvers as mv
from . import scene_spec as ss
from . import steered_plant as sp
from .l0_servo import L0ServoBackend
from .vehicle_profile import PROFILES_DIR, load_profile

RUN_SCHEMA = 'ubts.steer_sim.fast_sim_run.v1'
EPISODE_SCHEMA = 'ubts.steer_sim.fast_sim_episode.v1'
SUMMARY_SCHEMA = 'ubts.steer_sim.fast_sim_summary.v1'
TRACE_SCHEMA = 'ubts.steer_sim.fast_sim_trace.v1'
DT_S = .01                    # plant step (closed_loop.harness)
CONTROL_EVERY = 5             # driver 20 Hz (closed_loop.harness controller rate)
SENSOR_EVERY = 10             # 10 Hz observations (closed_loop.harness LiDAR rate)
PLANT_PURPOSE = 1             # closed_loop.harness.PURPOSES['plant']
PROBE_PURPOSE = 101           # probe observation noise stream (not a harness purpose)
MAX_WORKERS = 12
UBTS_DIR = Path(__file__).resolve().parents[1]          # sim/ubts
REPO_ROOT = Path(__file__).resolve().parents[3]
STOP_CODES = {None: 0, 'release': 1, 'brake': 2, 'coast': 3}
STOP_NAMES = {code: name for name, code in STOP_CODES.items()}
REGIME_CODES = {'still': 0, 'drive': 1, 'stopping': 2, 'blocked': 3}
INITIAL_COMMAND = (0., float('nan'), None, None, 'start')
RESULT_KIND_FALLBACK = '자리값 기반 SIM 결과'
# Every sim/ubts source an episode can import (direct L0 / L1, harness with the sensor estimator, W2); the closure is
# checked by test_steering_fast_sim (a fresh process runs episodes and compares the loaded sim/ubts modules).
IDENTITY_FILES = (
    'field_response.py', 'cpu_lidar.py', 'steering/__init__.py', 'steering/vehicle_profile.py', 'steering/geometry.py',
    'steering/drive.py', 'steering/backend_api.py', 'steering/l0_servo.py', 'steering/actuator_link.py',
    'steering/steered_plant.py', 'steering/scene_spec.py', 'steering/maneuvers.py', 'steering/flat_world.py',
    'steering/fast_sim.py', 'steering/native/__init__.py', 'steering/native/build.py', 'steering/native/lockstep_v2.py',
    'closed_loop/__init__.py', 'closed_loop/harness.py', 'closed_loop/world.py', 'closed_loop/estimator.py',
    'closed_loop/actuator_prior.py', 'closed_loop/controllers.py', 'closed_loop/raster_corridor.py',
    'closed_loop/external_paths.py',
    'orchard_perception/__init__.py', 'orchard_perception/scan_corridor.py', 'orchard_perception/scenarios.py',
    'steering/procedural_orchard.py', 'field_world/__init__.py', 'field_world/raster_ops.py',
    'field_world/build_field_world.py', 'field_world/scene_spec.py')
FLOAT_FIELDS = (
    't_s', 'cmd_throttle_pct', 'cmd_steer_target_mrad', 'steer_target_rad', 'drive_request_pct', 'drive_applied_pct',
    'truth_x_m', 'truth_y_m', 'truth_yaw_rad', 'truth_yaw_unwrapped_rad', 'truth_v_m_s', 'truth_w_rad_s',
    'truth_external_yaw_rate_rad_s', 'truth_distance_m', 'truth_curvature_1_m', 'truth_steer_curvature_1_m',
    'truth_wander_deg_per_m', 'truth_lateral_slip_m_s', 'truth_delta_eq_rad', 'truth_delta_left_rad',
    'truth_delta_right_rad', 'truth_motor_side_rad', 'truth_applied_steer_pwm', 'truth_servo_view_rad',
    'truth_hold_position_rad')
# The servo target after a hold_measured stop is SteeredPlant.servo_view_rad() (made from the true motor-side angle;
# the MCU will hold its filtered potentiometer measurement instead, P1 / P3). Written into every trace meta.
STEER_TARGET_NOTE = ('steer_target_rad is the host / MCU-side servo target; after a hold_measured stop it is the '
                     "plant's hold target: the MCU-side filtered potentiometer measurement at the stop "
                     '(SteeredPlant.measured_hold_rad(), R13-2), recorded as truth_hold_position_rad (the evaluator '
                     'hold reference, R13-3)')
# Environment variable of the native build folder (steering/native/build.py ENV_BUILD_DIR); fast_sim needs it set to
# a folder outside the repository, or native_build_dir (design 4.5: builds live outside the repository).
NATIVE_BUILD_DIR_ENV = 'UBTS_STEER_BUILD_DIR'
INT_FIELDS = (('cmd_stop_mode', np.int8), ('phase', np.int16), ('pot_raw', np.int16), ('fault_bits', np.int32),
              ('drive_en', np.int8), ('steer_en', np.int8), ('truth_model_flags', np.int32),
              ('truth_decelerating', np.int8), ('truth_regime', np.int8))
DERIVED_FIELDS = ('steer_measured_rad', 'truth_rear_x_m', 'truth_rear_y_m', 'truth_field_x_m', 'truth_field_y_m',
                  'truth_field_heading_rad')
UNITS = {'_s': 's', '_pct': '% duty', '_mrad': 'mrad', '_rad': 'rad', '_m': 'm', '_m_s': 'm/s', '_rad_s': 'rad/s',
         '_1_m': '1/m', '_deg_per_m': 'deg/m', 'pot_raw': 'ADC count', 'fault_bits': 'FaultBit mask',
         'truth_model_flags': 'ModelFlag mask', 'truth_applied_steer_pwm': 'signed fraction of full PWM'}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def canonical_json(value):
    return ss.canonical_json(value)


def _sha256_bytes(raw):
    return hashlib.sha256(raw).hexdigest()


def _file_sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def _write_bytes(path, raw):
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp%d' % os.getpid())
    tmp.write_bytes(raw)
    os.replace(tmp, path)


def _write_json(path, value):
    _write_bytes(path, (json.dumps(value, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False) + '\n')
                 .encode('utf-8'))


def _plain(value):
    """JSON-safe copy: numpy scalars to Python, non-finite floats to None, tuples to lists."""
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        value = float(value)
        return value if math.isfinite(value) else None
    if isinstance(value, Path):
        return str(value)
    return value


def plant_seed(stream_key, seed):
    """The seed closed_loop.harness gives the plant: (seed, CRC-32 of the stream key, purpose 'plant')."""
    return (int(seed), zlib.crc32(stream_key.encode('utf-8')), PLANT_PURPOSE)


def stream_rng(stream_key, seed, purpose):
    return np.random.default_rng(np.random.SeedSequence([int(seed), zlib.crc32(stream_key.encode('utf-8')),
                                                         int(purpose)]))


# --- deterministic npz ------------------------------------------------------------------------------------------------
def write_npz(path, arrays):
    """npz readable by numpy.load whose bytes depend only on the arrays (fixed member time, sorted members)."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for name in sorted(arrays):
            data = io.BytesIO()
            np.lib.format.write_array(data, np.ascontiguousarray(arrays[name]), allow_pickle=False)
            info = zipfile.ZipInfo(name + '.npy', date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.create_system = 3
            info.external_attr = 0o644 << 16
            archive.writestr(info, data.getvalue())
    _write_bytes(path, buffer.getvalue())


def content_sha256(arrays):
    """SHA-256 over (name, dtype, shape, C-order bytes) of every array, sorted by name."""
    digest = hashlib.sha256()
    for name in sorted(arrays):
        array = np.ascontiguousarray(arrays[name])
        digest.update(name.encode('utf-8') + b'\0' + array.dtype.str.encode('ascii') + b'\0'
                      + repr(tuple(array.shape)).encode('ascii') + b'\0')
        digest.update(array.tobytes())
    return digest.hexdigest()


def load_trace(path):
    """(arrays dict, meta dict) of a trace file."""
    with np.load(str(path), allow_pickle=False) as data:
        arrays = {name: data[name] for name in data.files}
    meta = json.loads(bytes(arrays['meta_json']).decode('utf-8'))
    return arrays, meta


# --- identity -----------------------------------------------------------------------------------------------------------
def code_identity():
    """SHA-256 of the source files an episode can execute (None for a missing file)."""
    files = {}
    for rel in IDENTITY_FILES:
        path = UBTS_DIR / rel
        files[rel] = _file_sha256(path) if path.is_file() else None
    return dict(files=files, code_sha256=ss.sha256_of(files))


def environment_identity():
    return dict(python=platform.python_version(), numpy=np.__version__, platform=sys.platform,
                machine=platform.machine())


def world_data_identity(world_spec):
    """External data a W2 world is built from: catalogue SHA-256 and the map PGM / YAML SHA-256 of the corridor
    (closed_loop.raster_corridor.corridor_world_config reads them from the catalogue; build_raster_world checks the
    map files against them). W3: the SHA-256 of the normalised procedural parameters (no external data; the generator
    file is in the code identity). None for W0."""
    if world_spec.kind == 'W3':
        from . import procedural_orchard
        return dict(procedural_params_sha256=procedural_orchard.params_sha256(world_spec.procedural))
    if world_spec.kind != 'W2':
        return None
    from closed_loop import raster_corridor as rc
    modifiers = tuple(rc.modifier_from_dict(m) for m in world_spec.modifiers)
    config = rc.corridor_world_config(world_spec.record, world_spec.corridor_id, world_spec.direction,
                                      modifiers=modifiers)
    return dict(catalogue_sha256=config.catalogue_sha256, pgm_sha256=config.pgm_sha256,
                yaml_sha256=config.yaml_sha256)


def checked_native_build_dir(build_dir=None):
    """The native build folder: build_dir, else UBTS_STEER_BUILD_DIR. Raises when neither is set or the folder is
    inside a work tree of the repository (design 4.5; native/build.py would otherwise build next to the sources)."""
    chosen = build_dir if build_dir is not None else os.environ.get(NATIVE_BUILD_DIR_ENV)
    if not chosen:
        raise ValueError('fast_sim_native_build_dir_not_set: pass native_build_dir or set %s to a folder outside the '
                         'repository' % NATIVE_BUILD_DIR_ENV)
    if ss.inside_repository(chosen):
        raise ValueError('fast_sim_native_build_dir_inside_the_repository: %s (design 4.5: builds live outside)'
                         % Path(chosen).resolve())
    return str(chosen)


def resolve_native(build_dir=None, compiler=None):
    """Build (or reuse) the native steering binary once: dict(path, binary_sha256, ...). Raises when it cannot.
    build_dir None: UBTS_STEER_BUILD_DIR (must be set and outside the repository, checked_native_build_dir())."""
    from .native import build as native_build
    result = native_build.ensure_built(compiler=compiler, build_dir=checked_native_build_dir(build_dir))
    return dict(path=str(result.path), file_name=Path(result.path).name, binary_sha256=result.binary_sha256,
                source_sha256=result.source_sha256, build_key=result.build_key, built_now=bool(result.built),
                compiler_version=result.toolchain.version, target=result.toolchain.target,
                firmware_build_id=getattr(result, 'firmware_build_id', None))


# --- profile, plant parameters, backend ----------------------------------------------------------------------------------
_PROFILE_CACHE = {}
_PARAMS_CACHE = {}


def scene_profile(scene):
    """The profile view of a scene: file (SHA pinned when given) + overrides + the backend level."""
    spec = scene.profile
    key = (canonical_json(spec.to_dict()), scene.backend)
    if key not in _PROFILE_CACHE:
        path = PROFILES_DIR / (spec.profile_id + '.yaml')
        if not path.is_file():
            raise ss.SceneError('scene_profile_not_found: %s' % spec.profile_id)
        profile = load_profile(path)
        if spec.sha256 is not None and profile.sha256 != spec.sha256:
            raise ss.SceneError('scene_profile_sha256_mismatch: %s is %s, the scene pins %s'
                                % (spec.profile_id, profile.sha256, spec.sha256))
        if spec.overrides:
            profile = profile.with_overrides(spec.overrides, label=spec.override_label,
                                             allow_out_of_range=spec.allow_out_of_range)
        if scene.backend in ('L0', 'L1') and not spec.geometry_check \
                and profile.value('steer_actuator_level') != scene.backend:
            profile = profile.with_overrides({'steer_actuator_level': scene.backend}, label='backend_' + scene.backend)
        _PROFILE_CACHE[key] = profile
    return _PROFILE_CACHE[key]


def scene_params(scene):
    """SteeredPlantParams of a scene (profile view, curve, geometry_check, steering mode, gyro noise scale)."""
    key = (canonical_json(scene.profile.to_dict()), scene.backend, scene.gyro_noise_scale)
    if key not in _PARAMS_CACHE:
        _PARAMS_CACHE[key] = sp.SteeredPlantParams.from_profile(
            scene_profile(scene), curve=scene.profile.curve, geometry_check=scene.profile.geometry_check,
            steer_mode=scene.profile.steer_mode, gyro_noise_scale=scene.gyro_noise_scale)
    return _PARAMS_CACHE[key]


def scene_backend_level(scene):
    return scene_params(scene).profile.value('steer_actuator_level')


def make_backend(params, native_executable=None):
    """A new unconfigured backend of the profile view's level (L0 Python servo or the native L1 process). Without a
    native_executable the L1 backend builds into checked_native_build_dir() (UBTS_STEER_BUILD_DIR, outside the
    repository)."""
    level = params.profile.value('steer_actuator_level')
    if level == 'L0':
        return L0ServoBackend(mode=params.steer_mode)
    if level == 'L1':
        from .actuator_link import NativeSteeringBackend
        build_dir = checked_native_build_dir() if native_executable is None else None
        return NativeSteeringBackend(mode=params.steer_mode, executable=native_executable, build_dir=build_dir)
    raise ValueError('fast_sim_actuator_level_not_available:%r' % (level,))


class _BackendTimer:
    """Times backend.step through an instance attribute (the backend class and its labels stay unchanged)."""

    def __init__(self, backend):
        self.last_us = 0.
        step = backend.step

        def timed_step(inp):
            started = time.perf_counter()
            out = step(inp)
            self.last_us = (time.perf_counter() - started) * 1e6
            return out
        backend.step = timed_step


# --- plant with a recorder, actuation -----------------------------------------------------------------------------------
class TracedSteeredPlant(sp.SteeredPlant):
    """SteeredPlant that keeps an unwrapped heading, the command in effect and calls a recorder after every step."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.yaw_start = self.yaw
        self.yaw_unwrapped = self.yaw
        self.applied_command = INITIAL_COMMAND
        self.recorder = None
        self.stop_hold_rad = None      # the hold target of a hold_measured stop since the last trace row (recorder)

    def stop(self, mode='brake', steer=None):
        super().stop(mode, steer)
        policy = self.params.stop_policy_steer if steer is None else steer
        if policy == 'hold_measured':
            self.stop_hold_rad = self.steer_target_rad     # what the stop holds (clamped measured_hold_rad(), R13-2)

    def step(self, dt):
        started = time.perf_counter()
        last_yaw = self.yaw
        super().step(dt)
        finished = time.perf_counter()
        self.yaw_unwrapped += wrap(self.yaw - last_yaw)
        if self.recorder is not None:
            self.recorder.record(self, started, finished)


class TableSteeredPlant(TracedSteeredPlant):
    """TracedSteeredPlant whose steering efficiency, understeer gradient and scrub speed loss come from per-delta
    tables (scene profile.plant_tables; the A12 twin form, SIM_ACCEPTANCE amendment 3) instead of the per-run draws.

    SteeredPlant.step() reads self.steering_efficiency, self.understeer_gradient and self.scrub_speed_loss_per_deg
    after the backend step of the same plant step (self._steer holds that step's wheel angles); here they are
    properties of the current delta_eq (and speed), so the plant integrates kappa = eta(|delta|) tan(delta) / L and
    speed x (1 - loss(|delta|, v)). The draws assigned by SteeredPlant.__init__ are kept as *_draw and not used."""

    def __init__(self, *args, tables=None, **kwargs):
        if tables is None:
            raise ValueError('TableSteeredPlant needs plant tables')
        rows = sorted(tables['eta_by_delta_deg'])
        if rows[0][0] > 0.:
            rows = [[0., 1.]] + rows
        self._eta_deg = np.array([r[0] for r in rows], dtype=float)
        self._eta_val = np.array([r[1] for r in rows], dtype=float)
        self._loss = None
        if tables.get('loss_by_delta_deg'):
            loss = sorted(tables['loss_by_delta_deg'])
            if loss[0][0] > 0.:
                loss = [[0.] + [0.] * len(tables['loss_speeds_m_s'])] + loss
            self._loss = (np.array([r[0] for r in loss], dtype=float),
                          np.array([r[1:] for r in loss], dtype=float),
                          np.array(tables['loss_speeds_m_s'], dtype=float))
        self.plant_tables_label = tables['label']
        super().__init__(*args, **kwargs)

    def _delta_deg(self):
        steer = self.__dict__.get('_steer')
        return 0. if steer is None else abs(math.degrees(steer.delta_eq_rad))

    @property
    def steering_efficiency(self):
        return float(np.interp(self._delta_deg(), self._eta_deg, self._eta_val))

    @steering_efficiency.setter
    def steering_efficiency(self, value):
        self.__dict__['steering_efficiency_draw'] = value

    @property
    def understeer_gradient(self):
        return 0.

    @understeer_gradient.setter
    def understeer_gradient(self, value):
        self.__dict__['understeer_gradient_draw'] = value

    def turning_loss(self, delta_deg, speed):
        """Loss fraction at |delta| deg and speed (bilinear; speed clamped to the table speeds)."""
        if self._loss is None or delta_deg <= 0.:
            return 0.
        degs, table, speeds = self._loss
        per_speed = [float(np.interp(delta_deg, degs, table[:, k])) for k in range(len(speeds))]
        if len(speeds) == 1:
            return per_speed[0]
        return float(np.interp(min(max(abs(speed), speeds[0]), speeds[-1]), speeds, per_speed))

    @property
    def scrub_speed_loss_per_deg(self):
        delta = self._delta_deg()
        if delta <= 1e-9:
            return 0.
        return self.turning_loss(delta, self.__dict__.get('v', 0.)) / delta

    @scrub_speed_loss_per_deg.setter
    def scrub_speed_loss_per_deg(self, value):
        self.__dict__['scrub_speed_loss_per_deg_draw'] = value


class ScriptActuation(sp.SteeringActuation):
    """SteeringActuation for maneuver commands: values (throttle_pct, steer_target_mrad, stop_mode, steer_policy,
    phase). A drive command goes to command_actuation(); a stop command calls plant.stop(mode, steer=policy) once,
    when the stop mode in effect changes, and repeats of it leave the plant alone (a repeated stop would restart
    the stop event, and a drive command would re-enable a coasting drive)."""

    def values(self, command):
        return (float(command.throttle_pct), float(command.steer_target_mrad), getattr(command, 'stop_mode', None),
                getattr(command, 'steer_policy', None), getattr(command, 'phase', 'run'))

    def apply(self, plant, values):
        throttle, steer, stop_mode, policy, _ = values
        previous = getattr(plant, 'applied_command', INITIAL_COMMAND)
        if stop_mode is None:
            plant.command_actuation(throttle, steer)
        elif previous[2] != stop_mode:
            plant.stop(stop_mode, steer=policy)
        plant.applied_command = values

    def is_reversal(self, values, last_values):
        return values[0] * last_values[0] < 0

    def trace_fields(self, values):
        return dict(throttle_pct=values[0], steer_target_mrad=values[1], stop_mode=values[2], phase=values[4])

    def initial_values(self):
        return INITIAL_COMMAND


class TraceRecorder:
    """Per plant step: the trace row (every `every` steps) and the wall-clock timing (every step)."""

    def __init__(self, every=1, extras=None, extra_names=()):
        self.every = int(every)
        self.extras = extras
        self.extra_names = tuple(extra_names)
        self.floats, self.ints = [], []
        self.phase_names = []
        self._phase_index = {}
        self.plant_us, self.backend_us, self.loop_us = [], [], []
        self.backend_timer = None
        self.steps = 0
        self._last = None             # the first loop sample starts at the first plant step (no set-up time in it)

    def restart_clock(self):
        """Start the loop clock again at the next plant step (the first loop sample then has no set-up time)."""
        self._last = None

    def _phase(self, name):
        index = self._phase_index.get(name)
        if index is None:
            index = self._phase_index[name] = len(self.phase_names)
            self.phase_names.append(name)
        return index

    def record(self, plant, started, finished):
        self.steps += 1
        self.plant_us.append((finished - started) * 1e6)
        self.loop_us.append((finished - (started if self._last is None else self._last)) * 1e6)
        self._last = finished
        self.backend_us.append(self.backend_timer.last_us if self.backend_timer is not None else math.nan)
        if (self.steps - 1) % self.every:
            return
        st, cmd, ramp = plant._steer, plant.applied_command, plant._ramp
        hold = getattr(plant, 'stop_hold_rad', None)
        if hold is None:
            hold = plant.measured_hold_rad()
        else:
            plant.stop_hold_rad = None
        row = (plant.time_s, cmd[0], cmd[1], plant.steer_target_rad, plant.throttle_request_pct, ramp.applied,
               plant.x, plant.y, plant.yaw, plant.yaw_unwrapped, plant.v, plant.w, plant.external_yaw_rate,
               plant.distance_m, plant.curvature, plant.steer_curvature, plant.wander_deg_per_m, plant.lateral,
               st.delta_eq_rad, st.delta_left_rad, st.delta_right_rad, st.motor_side_angle_rad, st.applied_steer_pwm,
               plant.servo_view_rad(), float(hold))
        if self.extras is not None:
            row = row + tuple(self.extras())
        self.floats.append(row)
        self.ints.append((STOP_CODES[cmd[2]], self._phase(cmd[4]), st.pot_raw, st.fault_bits, int(plant.drive_en),
                          int(plant.steer_en), st.model_flags, int(plant.decelerating), REGIME_CODES[plant.regime]))

    def arrays(self, params, field_frame):
        """Trace arrays (without meta_json). field_frame(x, y, yaw) -> (fx, fy, fheading) arrays, or None (W0)."""
        names = FLOAT_FIELDS + self.extra_names
        floats = np.array(self.floats, dtype=np.float64).reshape(-1, len(names))
        ints = np.array(self.ints, dtype=np.int64).reshape(-1, len(INT_FIELDS))
        out = {name: floats[:, k].copy() for k, name in enumerate(names)}
        for k, (name, dtype) in enumerate(INT_FIELDS):
            out[name] = ints[:, k].astype(dtype)
        table = params.geometry.link_table
        out['steer_measured_rad'] = np.array(
            [table.wheel_from_pot((float(raw) - params.pot_center_raw) / params.pot_counts_per_rad)
             for raw in out['pot_raw']], dtype=np.float64)
        x_rear = params.geometry.rear_axle_x_in_base_m
        yaw = out['truth_yaw_rad']
        out['truth_rear_x_m'] = out['truth_x_m'] + x_rear * np.cos(yaw)
        out['truth_rear_y_m'] = out['truth_y_m'] + x_rear * np.sin(yaw)
        if field_frame is None:
            out['truth_field_x_m'] = out['truth_x_m'].copy()
            out['truth_field_y_m'] = out['truth_y_m'].copy()
            out['truth_field_heading_rad'] = np.arctan2(np.sin(yaw), np.cos(yaw))
        else:
            fx, fy, fh = field_frame(out['truth_x_m'], out['truth_y_m'], yaw)
            out['truth_field_x_m'], out['truth_field_y_m'], out['truth_field_heading_rad'] = fx, fy, fh
        out['phase_names'] = np.array(self.phase_names if self.phase_names else ['start'])
        return out

    def timing_arrays(self):
        return dict(plant_step_us=np.array(self.plant_us, dtype=np.float32),
                    backend_step_us=np.array(self.backend_us, dtype=np.float32),
                    loop_step_us=np.array(self.loop_us, dtype=np.float32))


# --- drivers --------------------------------------------------------------------------------------------------------------
def pure_pursuit_curvature(offset, heading, lookahead):
    """Port of orchard_navigation.controller.pure_pursuit_curvature (upstream, read-only; Coulter 1992): curvature
    toward the point x = lookahead of the centreline y = offset + tan(heading) x (rear-axle frame, + left).
    test_steering_seed_regressions.py checks it against the upstream function."""
    y_l = offset + math.tan(heading) * lookahead
    d2 = lookahead * lookahead + y_l * y_l
    return 2.0 * y_l / d2


class ManeuverDriver:
    """A maneuver as a fast_sim driver (the runner's 20 Hz command source)."""

    observes = False
    truth_fed = False
    extra_names = ()

    def __init__(self, maneuver):
        self.maneuver = maneuver

    def bind(self, plant):
        pass

    def command(self, t_s, state):
        return self.maneuver.command(t_s, state)

    def done(self, t_s, state):
        return self.maneuver.done(t_s, state)

    def describe(self):
        out = self.maneuver.describe()
        out.update(role='maneuver', truth_fed_driver=False)
        return out


class ProbePursuit:
    """Survey probe re-run (design 6.2, 8.3 P2): upstream pure pursuit + delta = atan(L_hat kappa) on a noisy
    observation of the TRUE rear-axle pose relative to the line y = 0 (offset -y / cos(psi) + N(0, sd_offset),
    heading -psi + N(0, sd_heading)), taken at obs_hz and delivered after obs_latency_s. Like the probe: curvature
    clipped to 1 / R_min with R_min = L_hat / tan(min equivalent soft limit), yaw rate clipped to max_yaw_rate at the
    nominal speed, constant throttle. A truth-fed stand-in for the row estimator: NOT an executor (A13 concerns the
    executor and the estimator; this driver is marked truth_fed_driver)."""

    observes = True
    truth_fed = True
    extra_names = ('ctrl_obs_offset_m', 'ctrl_obs_heading_rad', 'ctrl_obs_valid', 'ctrl_curvature_cmd_1_m')

    def __init__(self, params, plant_params, rng):
        self.p = dict(params)
        self.rng = rng
        self.wheelbase = self.p['wheelbase_m'] or plant_params.wheelbase_m
        left, right = plant_params.soft_limits_eq_rad
        self.delta_max = min(left, right)
        self.r_min = self.wheelbase / math.tan(self.delta_max)
        self.throttle = self.p['throttle_pct']
        if self.throttle is None:
            self.throttle = mv.duty_for_speed(plant_params.profile, self.p['speed_m_s'], plant_params.curve)
        self.nominal_speed = (self.p['speed_m_s'] if self.p['speed_m_s'] is not None
                              else drive.steady_speed_m_s(plant_params.profile, self.throttle, plant_params.curve))
        self.period = 1. / self.p['obs_hz']
        self.next_obs = self.period            # like the harness LiDAR: first observation at one period
        self.queue = []
        self.obs = None
        self.kappa = 0.
        self.plant = None

    def bind(self, plant):
        self.plant = plant

    def observe(self, t_s):
        """Called after every plant step; takes an observation when one is due (obs_hz schedule)."""
        if t_s < self.next_obs - 1e-9:
            return
        while self.next_obs <= t_s + 1e-9:
            self.next_obs += self.period
        _, y, psi = self.plant.rear_axle_pose()
        offset = -y / math.cos(psi) + self.rng.normal(0., self.p['obs_offset_sd_m'])
        heading = -psi + self.rng.normal(0., math.radians(self.p['obs_heading_sd_deg']))
        self.queue.append((t_s + self.p['obs_latency_s'], offset, heading))

    def command(self, t_s, state):
        while self.queue and self.queue[0][0] <= t_s + 1e-9:
            self.obs = self.queue.pop(0)[1:]
        if self.obs is None:
            self.kappa = 0.
            return mv.Command(self.throttle, 0., phase='wait_observation')
        kappa = pure_pursuit_curvature(self.obs[0], self.obs[1], self.p['lookahead_m'])
        k_max = 1. / self.r_min
        kappa = max(-k_max, min(k_max, kappa))
        v = self.nominal_speed
        w = max(-self.p['max_yaw_rate_rad_s'], min(self.p['max_yaw_rate_rad_s'], v * kappa))
        kappa = w / v if v > 1e-6 else 0.
        self.kappa = kappa
        delta = max(-self.delta_max, min(self.delta_max, math.atan(self.wheelbase * kappa)))
        return mv.Command(self.throttle, 1000. * delta, phase='pursuit')

    def done(self, t_s, state):
        return self.plant.rear_axle_pose()[0] >= self.p['row_length_m']

    def trace_values(self):
        if self.obs is None:
            return (math.nan, math.nan, 0., self.kappa)
        return (self.obs[0], self.obs[1], 1., self.kappa)

    def describe(self):
        return dict(kind='probe_pure_pursuit', role='controller', params=dict(self.p), truth_fed_driver=True,
                    resolved_throttle_pct=self.throttle, nominal_speed_m_s=self.nominal_speed,
                    wheelbase_m=self.wheelbase, delta_max_rad=self.delta_max, r_min_m=self.r_min,
                    note='truth-fed survey probe stand-in (noisy true rear-axle pose); not an executor')


def make_driver(scene, seed, params):
    if scene.maneuver is not None:
        throttle_for_speed = (lambda speed: mv.duty_for_speed(params.profile, speed, params.curve))
        return ManeuverDriver(mv.make_maneuver(scene.maneuver.kind, scene.maneuver.params, throttle_for_speed))
    if scene.controller.kind == 'probe_pure_pursuit':
        return ProbePursuit(scene.controller.params, params, stream_rng(scene.stream_key, seed, PROBE_PURPOSE))
    raise ss.SceneError('controller_kind_unknown: %r' % (scene.controller.kind,))


# --- worlds -----------------------------------------------------------------------------------------------------------------
_W2_CACHE = {}


def _w2_world(world_spec):
    """(RasterCorridorConfig, RasterCorridorWorld) of a W2 (or W3) world spec, cached per process."""
    key = canonical_json(world_spec.to_dict())
    if key not in _W2_CACHE and world_spec.kind == 'W3':
        from . import procedural_orchard
        config, world, _ = procedural_orchard.build_raster_world(world_spec.procedural, modifiers=world_spec.modifiers,
                                                                  allow_holdout=world_spec.allow_holdout)
        _W2_CACHE[key] = (config, world)
    if key not in _W2_CACHE:
        from closed_loop import raster_corridor as rc
        modifiers = tuple(rc.modifier_from_dict(m) for m in world_spec.modifiers)
        config = rc.corridor_world_config(world_spec.record, world_spec.corridor_id, world_spec.direction,
                                          modifiers=modifiers)
        _W2_CACHE[key] = (config, rc.build_raster_world(config))
    return _W2_CACHE[key]


def _field_frame_function(world):
    if world is None:
        return None

    def convert(xs, ys, yaws):
        rows = [world.to_field(float(x), float(y), float(a)) for x, y, a in zip(xs, ys, yaws)]
        arr = np.array(rows, dtype=np.float64).reshape(-1, 3)
        return arr[:, 0].copy(), arr[:, 1].copy(), arr[:, 2].copy()
    return convert


# --- episodes ---------------------------------------------------------------------------------------------------------------
def _new_plant(scene, seed, params, native_executable, pose, recorder):
    backend = make_backend(params, native_executable)
    timer = _BackendTimer(backend)
    tables = scene.profile.plant_tables
    try:
        if tables is None:
            plant = TracedSteeredPlant(params, surface='normal', seed=plant_seed(scene.stream_key, seed), pose=pose,
                                       backend=backend, initial_steer_rad=math.radians(scene.start.steer_deg))
        else:
            plant = TableSteeredPlant(params, surface='normal', seed=plant_seed(scene.stream_key, seed), pose=pose,
                                      backend=backend, initial_steer_rad=math.radians(scene.start.steer_deg),
                                      tables=tables)
    except BaseException:
        backend.close()
        raise
    recorder.backend_timer = timer
    plant.recorder = recorder
    return plant


def _rule_checker(scene, plant, world):
    rules = {r.kind: r.value for r in scene.stop_rules}
    if not rules:
        return lambda now: None
    state = dict(moved=False, rest_since=None)

    def field(index):
        if world is None:
            return (plant.x, plant.y)[index]
        return world.to_field(plant.x, plant.y, plant.yaw)[index]

    def check(now):
        if 'travel_m' in rules and plant.distance_m >= rules['travel_m']:
            return 'stop_rule:travel_m'
        if 'field_x_m' in rules and field(0) >= rules['field_x_m']:
            return 'stop_rule:field_x_m'
        if 'abs_lateral_m' in rules and abs(field(1)) >= rules['abs_lateral_m']:
            return 'stop_rule:abs_lateral_m'
        if 'turned_deg' in rules and abs(plant.yaw_unwrapped - plant.yaw_start) >= math.radians(rules['turned_deg']):
            return 'stop_rule:turned_deg'
        if 'rest_s' in rules:
            if abs(plant.v) > .01:
                state['moved'] = True
            if state['moved'] and plant.v == 0. and abs(plant.w) < 1e-9:
                state['rest_since'] = now if state['rest_since'] is None else state['rest_since']
                if now - state['rest_since'] >= rules['rest_s'] - 1e-9:
                    return 'stop_rule:rest_s'
            else:
                state['rest_since'] = None
        return None
    return check


def _maneuver_state(plant, now):
    return mv.ManeuverState(now, plant.v, plant.yaw_unwrapped - plant.yaw_start, plant.distance_m)


def _run_direct(scene, seed, params, native_executable, recorder):
    world = None if scene.world.kind == 'W0' else _w2_world(scene.world)[1]
    start = scene.start
    pose = ((start.x_m, start.y_m, math.radians(start.yaw_deg)) if world is None
            else world.pose_in_field(start.x_m, start.y_m, math.radians(start.yaw_deg)))
    driver = make_driver(scene, seed, params)
    if driver.extra_names:
        recorder.extras, recorder.extra_names = driver.trace_values, driver.extra_names
    plant = _new_plant(scene, seed, params, native_executable, pose, recorder)
    try:
        driver.bind(plant)
        actuation = ScriptActuation()
        check = _rule_checker(scene, plant, world)
        steps = int(round(scene.duration_s / DT_S))
        queue, issued = [], 0
        end, now, done_steps = 'duration', 0., 0
        recorder.restart_clock()
        for step_index in range(steps):
            now = (step_index + 1) * DT_S
            while queue and queue[0][0] <= now - DT_S + 1e-9:
                _, _, due = heapq.heappop(queue)
                actuation.apply(plant, due)
            plant.step(DT_S)
            done_steps = step_index + 1
            if driver.observes:
                driver.observe(now)
            if done_steps % CONTROL_EVERY == 0:
                state = _maneuver_state(plant, now)
                values = actuation.values(driver.command(now, state))
                heapq.heappush(queue, (now + scene.command_latency_s, issued, values))
                issued += 1
                if driver.done(now, state):
                    end = 'maneuver_done'
                    break
            reason = check(now)
            if reason is not None:
                end = reason
                break
        info = dict(end_reason=end, end_t_s=round(now, 6), steps=done_steps, commands=issued)
        return plant, driver, world, info, None
    except BaseException:
        plant.close()
        raise


class NullEstimator:
    """Harness estimator stand-in that sees nothing (estimator 'null'): maneuvers do not need the estimator."""

    def __init__(self, config=None):
        self.odom_failures = 0
        self.distance = 0.

    def on_scan(self, *args, **kwargs):
        pass

    def on_imu(self, *args, **kwargs):
        pass

    def snapshot(self, now):
        return dict(corridor_valid=False, corridor_reason='null_estimator', heading_error=0., lateral_offset=0.)

    def next_corridor(self):
        return dict(left='UNKNOWN', right='UNKNOWN')


class _HarnessController:
    """The driver as closed_loop.harness controller: 20 Hz commands; when the driver is done it enters a terminal
    HOLD, and the harness ends the episode after 1 s at rest ('terminal_hold')."""

    def __init__(self, driver, holder):
        self.driver = driver
        self.holder = holder
        self.state, self.reason, self.hold_terminal = 'FOLLOW', None, False
        self.events = []
        self.counts = dict(stops=0, turns=0, stops_without_turn=0, stalls=0)

    def step(self, now, snapshot):
        plant = self.holder['plant']
        state = _maneuver_state(plant, now)
        command = self.driver.command(now, state)
        if self.state != 'HOLD' and self.driver.done(now, state):
            self.state, self.reason, self.hold_terminal = 'HOLD', 'maneuver_done', True
            self.events.append((round(now, 2), 'HOLD', 'maneuver_done'))
        return command


def harness_footprint_overrides(params):
    """EpisodeConfig.plant_overrides that give closed_loop.harness the scene profile's footprint rectangle (length,
    width of the body rectangle, geometry.SteeringGeometry footprint_*), so the harness collision end matches the
    profile the evaluator uses (the swept footprint of the evaluator contains this rectangle)."""
    g = params.geometry
    return (('footprint_length_m', float(g.footprint_length_m)), ('footprint_width_m', float(g.footprint_width_m)))


def _run_harness(scene, seed, params, native_executable, recorder):
    from closed_loop import harness as H
    if scene.world.kind == 'W0':
        hooks = fw.FlatHooks()
        world_config = fw.FlatWorldConfig(markers=scene.world.markers)
    else:
        from closed_loop import raster_corridor as rc
        hooks = rc.RasterHooks()
        world_config = _w2_world(scene.world)[0]
        if scene.world.kind == 'W3':          # no map file: a fresh procedural world per episode (own counters)
            def procedural_factory(config, hooks=hooks):
                from . import procedural_orchard
                if config is not world_config:
                    raise ValueError('fast_sim W3: unexpected world config')
                hooks.world = procedural_orchard.build_raster_world(scene.world.procedural,
                                                                    modifiers=scene.world.modifiers,
                                                                    allow_holdout=scene.world.allow_holdout)[1]
                return hooks.world
            hooks.world_factory = procedural_factory
    start = scene.start
    # The harness's collision end and clearance metrics use plant_params(config).footprint_*: give it the scene
    # profile's footprint rectangle (the differential FieldResponseParams default is 1.22 x 0.70 m whatever the
    # profile says). These two are the only plant overrides; the steered plant itself is built from the profile.
    footprint = harness_footprint_overrides(params)
    config = H.EpisodeConfig(name=scene.name, stream_key=scene.stream_key, seed=seed, world=world_config,
                             mode='hybrid', corridor_index=0, start_x_m=start.x_m, start_y_m=start.y_m,
                             start_yaw_deg=start.yaw_deg, max_time_s=scene.duration_s,
                             command_latency_s=scene.command_latency_s, gyro_noise_scale=1., plant_overrides=footprint)
    driver = make_driver(scene, seed, params)
    holder = {}

    def plant_factory(_field_params, *, surface, seed, pose, config):
        if tuple(getattr(config, 'plant_overrides', ())) != footprint:
            raise ValueError('fast_sim harness plant: EpisodeConfig.plant_overrides other than the profile footprint '
                             'are not steered-plant values')
        plant = _new_plant(scene, seed[0], params, native_executable, pose, recorder)
        if tuple(seed) != plant_seed(scene.stream_key, seed[0]):
            plant.close()
            raise RuntimeError('fast_sim harness plant seed differs from the direct runner seed')
        holder['plant'] = plant
        driver.bind(plant)
        return plant

    controller = _HarnessController(driver, holder)
    estimator = NullEstimator if scene.estimator == 'null' else H.SensorEstimator
    try:
        recorder.restart_clock()
        result = H.run_episode(config, record_trace=False, estimator_factory=estimator,
                               controller_factory=lambda _control: controller, plant_factory=plant_factory,
                               actuation=ScriptActuation(), **hooks.kwargs())
    except BaseException:
        if 'plant' in holder:
            holder['plant'].close()
        raise
    plant = holder['plant']
    keep = ('end', 'final_state', 'final_reason', 'time_s', 'collisions', 'reversal_commands', 'truth_distance_m',
            'estimated_distance_m', 'odom_failures', 'min_clearance_m', 'corridor_reason', 'plant_pivots',
            'chatter_flips', 'commands')
    harness = {k: _plain(result.get(k)) for k in keep if k in result}
    world = hooks.world
    harness['world'] = _plain(world.describe()) if hasattr(world, 'describe') else None
    end = 'maneuver_done' if result.get('end') == 'terminal_hold' else 'harness:' + str(result.get('end'))
    info = dict(end_reason=end, end_t_s=round(plant.time_s, 6), steps=recorder.steps, commands=None, harness=harness)
    return plant, driver, (None if scene.world.kind == 'W0' else world), info, harness


def _timing_summary(values_us):
    a = np.asarray(values_us, dtype=np.float64)
    a = a[np.isfinite(a)]
    if not a.size:
        return dict(n=0)
    p50, p99, p999 = np.percentile(a, [50., 99., 99.9])
    return dict(n=int(a.size), mean_us=round(float(a.mean()), 2), p50_us=round(float(p50), 2),
                p99_us=round(float(p99), 2), p999_us=round(float(p999), 2), max_us=round(float(a.max()), 2))


def _quick(arrays):
    v = arrays['truth_v_m_s']
    delta = arrays['truth_delta_eq_rad']
    if not len(v):
        return dict(samples=0)
    return dict(samples=int(len(v)), t_end_s=round(float(arrays['t_s'][-1]), 4),
                distance_m=round(float(arrays['truth_distance_m'][-1]), 4), v_max_m_s=round(float(np.abs(v).max()), 4),
                v_end_m_s=round(float(v[-1]), 4), max_abs_delta_eq_deg=round(float(np.degrees(np.abs(delta).max())), 3),
                end_field_pose=[round(float(arrays['truth_field_x_m'][-1]), 4),
                                round(float(arrays['truth_field_y_m'][-1]), 4),
                                round(float(np.degrees(arrays['truth_field_heading_rad'][-1])), 3)],
                turned_deg=round(float(np.degrees(arrays['truth_yaw_unwrapped_rad'][-1]
                                                  - arrays['truth_yaw_unwrapped_rad'][0])), 3),
                faults_seen=int(np.bitwise_or.reduce(arrays['fault_bits'].astype(np.int64))) if len(v) else 0)


def run_episode(scene, seed, *, native_executable=None):
    """Run one episode in this process. Returns (arrays with meta_json, timing arrays, record dict).

    The plant is always closed (native process stopped) before this returns or raises."""
    if seed not in scene.seeds:
        raise ss.SceneError('scene_seed_not_in_scene: %r' % (seed,))
    params = scene_params(scene)
    recorder = TraceRecorder(every=scene.trace_every)
    started = time.perf_counter()
    runner = _run_direct if scene.runner == 'direct' else _run_harness
    plant, driver, world, info, harness = runner(scene, seed, params, native_executable, recorder)
    try:
        backend_label = None
        if hasattr(plant.backend, 'result_label') and getattr(plant.backend, 'config', None) is not None:
            backend_label = _plain(plant.backend.result_label())
        elif callable(getattr(plant.backend, 'info', None)):
            backend_label = _plain(plant.backend.info())
        profile_label = _plain(plant.result_label())
        run_info = _plain(plant.run_info())
    finally:
        plant.close()
    wall = time.perf_counter() - started
    arrays = recorder.arrays(params, _field_frame_function(world))
    truth_keys = sorted(k for k in arrays if k.startswith('truth_'))
    # Only the episode's own identity goes into the trace (content_sha256 covers meta_json): the whole-scene SHA-256
    # changes when another seed is added, and would make the same episode's trace differ between runs (A8).
    meta = dict(schema=TRACE_SCHEMA, episode_id=scene.episode_id(seed), scene_name=scene.name, seed=seed,
                episode_scene_sha256=scene.episode_sha256(seed), dt_s=DT_S,
                trace_every=scene.trace_every, runner=scene.runner, world=scene.world.kind,
                truth_keys=truth_keys, controller_keys=list(recorder.extra_names),
                controller_keys_note=('derived from truth + noise (truth-fed probe)' if driver.truth_fed else None),
                steer_target_note=STEER_TARGET_NOTE,
                stop_codes={str(k): v for k, v in STOP_NAMES.items()},
                regime_codes={v: k for k, v in REGIME_CODES.items()},
                units={name: unit for name in sorted(arrays) for suffix, unit in UNITS.items()
                       if name.endswith(suffix) or name == suffix})
    arrays['meta_json'] = np.frombuffer(canonical_json(_plain(meta)), dtype=np.uint8).copy()
    timing = recorder.timing_arrays()
    record = dict(
        schema=EPISODE_SCHEMA, episode_id=scene.episode_id(seed), scene_name=scene.name, seed=seed,
        scene_sha256=scene.sha256(), episode_scene_sha256=scene.episode_sha256(seed), runner=scene.runner,
        estimator=scene.estimator, world=dict(kind=scene.world.kind, markers=len(scene.world.markers),
                                              record=scene.world.record, corridor_id=scene.world.corridor_id,
                                              direction=scene.world.direction),
        start=scene.start.to_dict(), backend=dict(requested=scene.backend, level=plant.backend_level,
                                                  cls=type(plant.backend).__name__, label=backend_label),
        driver=_plain(driver.describe()), truth_fed_driver=bool(driver.truth_fed),
        profile_label=profile_label, run_info=run_info, end=info, quick=_quick(arrays),
        result_kind=profile_label.get('result_kind', RESULT_KIND_FALLBACK), field_validated=False,
        timing=dict(wall_s=round(wall, 4), plant_step=_timing_summary(timing['plant_step_us']),
                    backend_step=_timing_summary(timing['backend_step_us']),
                    loop_step=_timing_summary(timing['loop_step_us'])))
    if harness is not None:
        record['harness'] = harness
    return arrays, timing, record


# --- run orchestration --------------------------------------------------------------------------------------------------
def hold_file():
    from closed_loop import external_paths
    return external_paths.hold_file()


def wait_while_held(poll_s=60., log=None, max_wait_s=None):
    """Wait while the HOLD_MEASUREMENT file exists (path from UBTS_HOLD_DIR / the UBTS_STEER_PATHS file). Returns the
    seconds waited; raises TimeoutError after max_wait_s, PathNotConfigured when no hold folder is configured."""
    path = hold_file()
    waited, started = 0., time.monotonic()
    while path.exists():
        if log is not None:
            log('waiting: %s exists' % path)
        if max_wait_s is not None and time.monotonic() - started >= max_wait_s:
            raise TimeoutError('HOLD_MEASUREMENT still present after %.1f s' % (time.monotonic() - started))
        time.sleep(poll_s)
        waited = time.monotonic() - started
    return waited


def _inside_repository(path):
    """Inside this work tree or any other work tree of the same repository (scene_spec.inside_repository)."""
    return ss.inside_repository(path)


def _episode_identity(scene, base_identity, native, world_data=None):
    level = scene_backend_level(scene)
    identity = dict(base_identity, profile_sha256=scene_params(scene).profile.sha256, backend_level=level,
                    native_binary_sha256=(native or {}).get('binary_sha256') if level == 'L1' else None,
                    runner=scene.runner)
    if scene.world.kind == 'W2':          # the external data of the world (catalogue, map) is part of the identity
        identity['world_data'] = world_data if world_data is not None else world_data_identity(scene.world)
    return identity, ss.sha256_of(identity)


def _store_episode(out_dir, arrays, timing, record):
    eid = record['episode_id']
    trace_path = out_dir / 'trace' / (eid + '.npz')
    write_npz(trace_path, arrays)
    write_npz(out_dir / 'timing' / (eid + '.npz'), timing)
    record['trace'] = dict(file='trace/%s.npz' % eid, file_sha256=_file_sha256(trace_path),
                           content_sha256=content_sha256(arrays), samples=int(len(arrays['t_s'])),
                           every=record.get('trace_every', 1))
    record['timing_file'] = 'timing/%s.npz' % eid
    _write_json(out_dir / 'episodes' / (eid + '.json'), _plain(record))
    return record


def _job_entry(job):
    """Worker entry (spawn safe, top level): HOLD wait, one episode, its files. Returns the record.

    After the episode (every module it uses is imported by then) the code files are hashed again: when they differ
    from the identity the run was planned with, the episode is recorded as failed (code_identity_changed) instead of
    ok, so a stored record never names code that did not run (another work stream may edit the sources meanwhile)."""
    out_dir = Path(job['out_dir'])
    waited = wait_while_held(job['hold_poll_s']) if job['hold_check'] else 0.
    scene = ss.SceneSpec.from_dict(job['scene'])
    seed = job['seed']
    base = dict(schema=EPISODE_SCHEMA, episode_id=scene.episode_id(seed), scene_name=scene.name, seed=seed,
                scene_sha256=scene.sha256(), episode_scene_sha256=scene.episode_sha256(seed),
                identity_sha256=job['identity_sha256'], run_id=job['run_id'], hold_waited_s=round(waited, 3),
                worker_pid=os.getpid())
    try:
        arrays, timing, record = run_episode(scene, seed, native_executable=job['native_executable'])
    except Exception as exc:
        record = dict(base, status='failed', error=dict(type=type(exc).__name__, message=str(exc)[:2000],
                                                        traceback=traceback.format_exc()[-4000:]))
        _write_json(out_dir / 'episodes' / (record['episode_id'] + '.json'), _plain(record))
        return record
    after = code_identity()
    expected = job.get('code_sha256')
    if expected is not None and after['code_sha256'] != expected:
        changed = sorted(rel for rel, digest in after['files'].items()
                         if digest != (job.get('code_files') or {}).get(rel))
        record = dict(base, status='failed', error=dict(
            type='code_identity_changed', message='source files changed between planning and the end of the episode: '
                                                  '%s' % ', '.join(changed)[:2000]))
        _write_json(out_dir / 'episodes' / (record['episode_id'] + '.json'), _plain(record))
        return record
    record.update(base)
    record['status'] = 'ok'
    record['trace_every'] = scene.trace_every
    return _store_episode(out_dir, arrays, timing, record)


def _reusable(out_dir, eid, episode_sha, identity_sha):
    path = out_dir / 'episodes' / (eid + '.json')
    if not path.is_file():
        return None
    try:
        record = json.loads(path.read_bytes().decode('utf-8'))
    except (OSError, ValueError):
        return None
    if (record.get('status') != 'ok' or record.get('episode_scene_sha256') != episode_sha
            or record.get('identity_sha256') != identity_sha):
        return None
    trace = out_dir / record.get('trace', {}).get('file', 'missing')
    if not trace.is_file() or _file_sha256(trace) != record['trace'].get('file_sha256'):
        return None
    return record


def _pooled_timing(out_dir, records):
    blocks = {}
    for record in records:
        timing_file = out_dir / record.get('timing_file', '')
        if not timing_file.is_file():
            continue
        with np.load(str(timing_file), allow_pickle=False) as data:
            for name in data.files:
                blocks.setdefault(name, []).append(data[name].astype(np.float64))
    out = {name: _timing_summary(np.concatenate(parts)) for name, parts in blocks.items()}
    walls = [r['timing']['wall_s'] for r in records if 'timing' in r]
    if walls:
        out['episode_wall_s'] = dict(n=len(walls), mean=round(statistics.mean(walls), 4),
                                     p50=round(float(np.percentile(walls, 50)), 4),
                                     p99=round(float(np.percentile(walls, 99)), 4), max=round(max(walls), 4),
                                     total=round(sum(walls), 3))
    return out


def run_scenes(scenes, out_dir, *, workers=1, force=False, hold_check=True, hold_poll_s=60.,
               native_executable=None, native_build_dir=None, progress=None):
    """Run every episode of the scenes into out_dir (outside the repository). Returns the SUMMARY dict.

    workers      1 = in this process; N > 1 = N spawned worker processes (results do not depend on N)
    force        run episodes even when a matching record exists
    hold_check   wait while HOLD_MEASUREMENT exists (needs a configured hold folder; False only for tests)
    native_executable  native binary to use for L1 episodes (None: built once here into native_build_dir)
    native_build_dir   native build folder (None: UBTS_STEER_BUILD_DIR); must be set and outside the repository
                 when an L1 scene needs a build
    progress     callable(text) for one line per finished episode (None: silent)
    """
    scenes = ss.check_unique_names(list(scenes))
    if not scenes:
        raise ValueError('run_scenes needs at least one scene')
    out_dir = Path(out_dir).resolve()
    if _inside_repository(out_dir):
        raise ValueError('fast_sim_run_folder_inside_the_repository: %s (design 4.5: runs live outside)' % out_dir)
    if not 1 <= int(workers) <= MAX_WORKERS:
        raise ValueError('fast_sim_workers_out_of_range (1..%d)' % MAX_WORKERS)
    needs_native = any(scene_backend_level(s) == 'L1' for s in scenes)
    build_dir = checked_native_build_dir(native_build_dir) if needs_native and native_executable is None else None
    for sub in ('trace', 'timing', 'episodes'):
        (out_dir / sub).mkdir(parents=True, exist_ok=True)
    if hold_check:
        wait_while_held(hold_poll_s, log=progress)
    for variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
        os.environ.setdefault(variable, '1')
    native = None
    if needs_native:
        if native_executable is None:
            native = resolve_native(build_dir=build_dir)
        else:
            native = dict(path=str(native_executable), file_name=Path(native_executable).name,
                          binary_sha256=_file_sha256(native_executable))
    started_wall = time.perf_counter()
    run_id = '%s_%d' % (time.strftime('%Y%m%dT%H%M%S'), os.getpid())
    base_identity = dict(code_identity(), **environment_identity())
    plan, jobs, reused, run_scene_sha = [], [], {}, {}
    world_data = {}
    for scene in scenes:
        key = canonical_json(scene.world.to_dict())
        if scene.world.kind in ('W2', 'W3') and key not in world_data:
            world_data[key] = world_data_identity(scene.world)
        identity, identity_sha = _episode_identity(scene, base_identity, native, world_data.get(key))
        for eid, seed in scene.episodes():
            plan.append(eid)
            run_scene_sha[eid] = scene.sha256()
            record = None if force else _reusable(out_dir, eid, scene.episode_sha256(seed), identity_sha)
            if record is not None:
                reused[eid] = record
                continue
            jobs.append(dict(scene=scene.to_dict(), seed=seed, out_dir=str(out_dir), identity_sha256=identity_sha,
                             code_sha256=base_identity['code_sha256'], code_files=base_identity['files'],
                             native_executable=native['path'] if native else None, run_id=run_id,
                             hold_check=bool(hold_check), hold_poll_s=float(hold_poll_s)))
    run_doc = dict(schema=RUN_SCHEMA, run_id=run_id, status='running', workers=int(workers), force=bool(force),
                   scenes=[dict(name=s.name, sha256=s.sha256(), spec=s.to_dict()) for s in scenes],
                   identity=base_identity, native=None if native is None else {k: v for k, v in native.items()
                                                                                 if k != 'path'},
                   episodes_planned=len(plan), episodes_to_run=len(jobs), episodes_reused=len(reused),
                   started_local=time.strftime('%Y-%m-%dT%H:%M:%S'))
    _write_json(out_dir / 'RUN.json', _plain(run_doc))
    results = {}

    def note(record):
        results[record['episode_id']] = record
        if progress is not None:
            progress('[%d/%d] %s %s %s' % (len(results), len(jobs), record['episode_id'], record['status'],
                                           record.get('timing', {}).get('wall_s', record.get('error', {}).get('type'))))
    if jobs:
        if int(workers) == 1:
            for job in jobs:
                note(_job_entry(job))
        else:
            context = multiprocessing.get_context('spawn')
            with ProcessPoolExecutor(max_workers=int(workers), mp_context=context) as pool:
                futures = {pool.submit(_job_entry, job): job for job in jobs}
                for future in as_completed(futures):
                    job = futures[future]
                    try:
                        note(future.result())
                    except Exception as exc:          # a worker process died
                        scene = ss.SceneSpec.from_dict(job['scene'])
                        note(dict(episode_id=scene.episode_id(job['seed']), scene_name=scene.name, seed=job['seed'],
                                  status='failed', error=dict(type=type(exc).__name__, message=str(exc)[:2000])))
    records = []
    for eid in plan:
        record = results.get(eid) or reused.get(eid)
        if eid in reused and eid not in results:
            record = dict(record, reused=True)
        # The scene SHA-256 of THIS run; a reused record keeps scene_sha256 of the run that made it (its episode
        # identity episode_scene_sha256 is the same by the reuse rule).
        record = dict(record, run_scene_sha256=run_scene_sha[eid])
        records.append(record)
    lines = ''.join(json.dumps(_plain(r), sort_keys=True, ensure_ascii=False) + '\n' for r in records)
    _write_bytes(out_dir / 'EPISODES.jsonl', lines.encode('utf-8'))
    summary = summarise(out_dir, scenes, records, [results[e] for e in plan if e in results], run_doc,
                        time.perf_counter() - started_wall)
    _write_json(out_dir / 'SUMMARY.json', summary)
    run_doc.update(status='done', wall_s=round(time.perf_counter() - started_wall, 3),
                   finished_local=time.strftime('%Y-%m-%dT%H:%M:%S'),
                   failed=sum(1 for r in records if r.get('status') != 'ok'))
    _write_json(out_dir / 'RUN.json', _plain(run_doc))
    return summary


def summarise(out_dir, scenes, records, ran, run_doc, wall_s):
    ok = [r for r in records if r.get('status') == 'ok']
    traces = {r['episode_id']: r['trace']['content_sha256'] for r in ok}
    per_scene = {}
    for record in records:
        block = per_scene.setdefault(record['scene_name'], dict(episodes=0, ok=0, failed=0, ends={}))
        block['episodes'] += 1
        if record.get('status') == 'ok':
            block['ok'] += 1
            end = record['end']['end_reason']
            block['ends'][end] = block['ends'].get(end, 0) + 1
        else:
            block['failed'] += 1
    kinds = sorted({r.get('result_kind', RESULT_KIND_FALLBACK) for r in ok}) or [RESULT_KIND_FALLBACK]
    by_group = {}
    for record in [r for r in ran if r.get('status') == 'ok']:
        by_group.setdefault('%s/%s' % (record['runner'], record['backend']['level']), []).append(record)
    return _plain(dict(
        schema=SUMMARY_SCHEMA, run_id=run_doc['run_id'], workers=run_doc['workers'], wall_s=round(wall_s, 3),
        counts=dict(planned=len(records), ok=len(ok), failed=len(records) - len(ok), ran=len(ran),
                    reused=sum(1 for r in records if r.get('reused'))),
        scenes={s.name: dict(sha256=s.sha256(), runner=s.runner, backend=s.backend, world=s.world.kind,
                             driver=s.driver.kind, **per_scene.get(s.name, {})) for s in scenes},
        traces=traces, trace_set_sha256=ss.sha256_of(sorted([k, v] for k, v in traces.items())),
        timing=dict(note='episodes run in this invocation only; wall clock of this PC under its other load; report only '
                         '(A11)', pooled=_pooled_timing(out_dir, [r for r in ran if r.get('status') == 'ok']),
                    by_runner_level={k: _pooled_timing(out_dir, v) for k, v in sorted(by_group.items())}),
        failures=[dict(episode_id=r['episode_id'], error=r.get('error')) for r in records if r.get('status') != 'ok'],
        result_kinds=kinds, field_validated=False))


def read_episodes(run_dir):
    """Episode records of a run folder (EPISODES.jsonl order)."""
    text = (Path(run_dir) / 'EPISODES.jsonl').read_bytes().decode('utf-8')
    return [json.loads(line) for line in text.splitlines() if line.strip()]


# --- trace analyses (A1, A4, probe) ----------------------------------------------------------------------------------------
def fit_circle(points):
    """Least-squares (Kasa) circle through 2-D points: (radius, (cx, cy))."""
    pts = np.asarray(points, dtype=float)
    centre = pts.mean(axis=0)
    x, y = (pts - centre).T
    solution = np.linalg.lstsq(np.column_stack([x, y, np.ones_like(x)]), -(x * x + y * y), rcond=None)[0]
    cx, cy = -solution[0] / 2., -solution[1] / 2.
    return math.sqrt(cx * cx + cy * cy - solution[2]), (cx + centre[0], cy + centre[1])


def phase_mask(arrays, name):
    names = [str(n) for n in arrays['phase_names']]
    if name not in names:
        return np.zeros(len(arrays['t_s']), dtype=bool)
    return arrays['phase'] == names.index(name)


def analyse_circle(arrays, wheelbase_m, rel_tol=1e-6):
    """A1: circle radius of the rear axle over the steady samples (speed equal to the final speed within rel_tol,
    circle phase) against L / tan(delta_eq), and the yaw-rate sign."""
    v = arrays['truth_v_m_s']
    steady = phase_mask(arrays, 'circle') & (np.abs(v - v[-1]) <= rel_tol * max(1., abs(v[-1])))
    points = np.column_stack((arrays['truth_rear_x_m'][steady], arrays['truth_rear_y_m'][steady]))
    if len(points) < 50:
        return dict(ok=False, reason='too_few_steady_samples', samples=int(len(points)))
    delta = float(np.mean(arrays['truth_delta_eq_rad'][steady]))
    radius, _ = fit_circle(points)
    expected = wheelbase_m / math.tan(abs(delta))
    turned = float(np.degrees(arrays['truth_yaw_unwrapped_rad'][steady][-1] - arrays['truth_yaw_unwrapped_rad'][steady][0]))
    w = arrays['truth_w_rad_s'][steady]
    return dict(ok=True, samples=int(len(points)), delta_eq_deg=round(math.degrees(delta), 6),
                speed_m_s=round(float(v[-1]), 6), radius_m=round(radius, 6), expected_radius_m=round(expected, 6),
                radius_error_rel=round(abs(radius - expected) / expected, 8), turned_deg=round(turned, 3),
                yaw_rate_sign_matches=bool(np.all(np.sign(w) == math.copysign(1., delta))),
                yaw_rate_rel_error=round(float(np.max(np.abs(w - v[steady] * math.tan(delta) / wheelbase_m))
                                               / max(abs(float(np.mean(w))), 1e-12)), 8))


def analyse_stops(arrays):
    """Stops of a stop maneuver: each entry of the command in effect into the 'stop' phase. v0 and the start
    position are the sample before the stop command reached the plant; the stop ends at the first zero speed."""
    stop = phase_mask(arrays, 'stop')
    starts = np.flatnonzero(stop & ~np.concatenate(([False], stop[:-1])))
    v, t = arrays['truth_v_m_s'], arrays['t_s']
    x, y, dist = arrays['truth_x_m'], arrays['truth_y_m'], arrays['truth_distance_m']
    out = []
    for i in starts:
        j0 = max(int(i) - 1, 0)
        zero = np.flatnonzero(v[i:] == 0.)
        if not len(zero):
            out.append(dict(index=int(i), v0_m_s=float(v[j0]), completed=False))
            continue
        j = int(i) + int(zero[0])
        out.append(dict(index=int(i), t0_s=float(t[j0]), v0_m_s=float(v[j0]), completed=True,
                        mode=STOP_NAMES[int(arrays['cmd_stop_mode'][i])] or 'throttle_zero',
                        distance_m=float(math.hypot(x[j] - x[j0], y[j] - y[j0])), path_m=float(dist[j] - dist[j0]),
                        time_s=float(t[j] - t[j0])))
    return out


def analyse_probe(arrays, row_length_m=30., settle_band_m=.05, after_m=5.):
    """Survey probe metrics on the rear axle (probe definitions): finished row, |y| max / RMS after 5 m, command sd
    after 5 m, overshoot to the other side, last x outside the settle band, mean y over [10 m, row end]."""
    rx, ry = arrays['truth_rear_x_m'], arrays['truth_rear_y_m']
    after = rx > after_m
    cmd = arrays['cmd_steer_target_mrad']
    valid_cmd = after & np.isfinite(cmd) & phase_mask(arrays, 'pursuit')
    y0 = float(ry[0])
    overshoot = float(max(0., -ry.min() if y0 > 0 else ry.max()))
    outside = np.flatnonzero(np.abs(ry) >= settle_band_m)
    late = (rx >= 10.) & (rx <= row_length_m)
    return dict(finished=bool(rx[-1] >= row_length_m), t_end_s=round(float(arrays['t_s'][-1]), 3),
                lat_max_after5m_m=round(float(np.abs(ry[after]).max()), 4) if after.any() else None,
                lat_rms_after5m_m=round(float(np.sqrt(np.mean(ry[after] ** 2))), 4) if after.any() else None,
                lat_max_whole_m=round(float(np.abs(ry).max()), 4), overshoot_opposite_side_m=round(overshoot, 4),
                settled_after_x_m=round(float(rx[outside[-1]]), 2) if len(outside) else 0.,
                steer_cmd_std_after5m_deg=round(float(np.degrees(np.std(cmd[valid_cmd] / 1000.))), 3)
                if valid_cmd.any() else None,
                mean_y_10m_to_end_m=round(float(np.mean(ry[late])), 4) if late.any() else None,
                mean_speed_m_s=round(float(np.mean(arrays['truth_v_m_s'][after])), 4) if after.any() else None,
                steer_saturated_fraction=round(float(np.mean(np.abs(arrays['steer_target_rad'][after])
                                                             >= .999 * np.abs(arrays['steer_target_rad']).max())), 4)
                if after.any() else None)


def probe_pass(result):
    """Probe criterion (survey): finished row and max |y| after 5 m < 0.30 m (not a divergence test)."""
    return bool(result['finished'] and result['lat_max_after5m_m'] is not None and result['lat_max_after5m_m'] < .30)


# --- built-in scene sets ------------------------------------------------------------------------------------------------------
def _tag(value, digits=0):
    text = ('%.*f' % (digits, abs(value))).replace('.', '')
    return ('m' if value < 0 else 'p') + text


def a1_scenes(runner='direct', seeds=(11,)):
    """A1(F): W0 circles delta 5 / 10 / 20 / -10 deg x v 0.3 / 0.5 m/s, geometry_check (ideal values, L0). The stream
    key does not name the runner, so the direct and the harness set draw the same random numbers."""
    wheelbase = load_profile().value('wheelbase_m')
    out = []
    for delta in (5., 10., 20., -10.):
        for speed in (.3, .5):
            radius = wheelbase / math.tan(math.radians(abs(delta)))
            duration = math.radians(150.) * radius / speed + 4.
            rules = [dict(kind='turned_deg', value=150.)] if runner == 'direct' else []
            key = 'a1_circle_d%s_v%s' % (_tag(delta), _tag(speed * 100))
            out.append(ss.scene('a1_%s_%s' % (runner, key[3:]), stream_key=key, runner=runner,
                                estimator='null' if runner == 'harness' else None, seeds=list(seeds),
                                profile=dict(geometry_check=True), start=dict(steer_deg=delta, condition='wheels_turned'),
                                maneuver=dict(kind='circle', params=dict(speed_m_s=speed, steer_deg=delta)),
                                duration_s=round(duration, 2), stop_rules=rules))
    return out


A4_SPEED_DUTIES = (('main', (40., 55., 70., 88.)), ('bracket', (90., 95., 100.)))
A4_RELEASE_DEVELOPMENT_SEEDS = tuple(range(400))
# SIM_ACCEPTANCE amendment 1 (2026-11-09): the A4(F) release judgement sample = per curve the release scene on the
# not yet used seeds 10000-13999, stream key a4_release_<curve> (no runner name), every stop from 0.90-0.95 m/s.
# Tests and development runs must not run these seeds (release_sample_kind() names a sample).
A4_RELEASE_JUDGEMENT_SEEDS = tuple(range(10000, 14000))
A4_RELEASE_CURVES = ('bracket', 'main')


def a4_release_stream_key(curve):
    return 'a4_release_%s' % curve


def a4_scenes(runner='direct', release_seeds=A4_RELEASE_DEVELOPMENT_SEEDS, stop_seed=4):
    """A4(F): steady speed per duty (geometry_check), reverse, ramp, release stop distance (400 development seeds per
    curve, L0 like P1; the judgement sample is a4_judgement_scenes()), brake / coast distance at the low / median /
    high deceleration (L0). Stream keys do not name the runner (the direct and the harness set draw the same random
    numbers)."""
    profile = load_profile()
    out = []
    harness = dict(estimator='null') if runner == 'harness' else {}
    for curve, duties in A4_SPEED_DUTIES:
        for duty in duties + (-duties[0],):
            out.append(ss.scene('a4_%s_speed_%s_%s' % (runner, curve, _tag(duty)), runner=runner, seeds=[1],
                                stream_key='a4_speed_%s_%s' % (curve, _tag(duty)),
                                profile=dict(geometry_check=True, curve=curve), duration_s=8.,
                                maneuver=dict(kind='step', params=dict(at_s=0., throttle_pct=duty)), **harness))
    for curve in A4_RELEASE_CURVES:
        out.append(ss.scene('a4_%s_release_%s' % (runner, curve), runner=runner, seeds=list(release_seeds), backend='L0',
                            stream_key=a4_release_stream_key(curve),
                            profile=dict(curve=curve), duration_s=12.,
                            maneuver=dict(kind='stop_test', params=dict(throttle_pct=100., drive_s=4., hold_s=.5)),
                            **harness))
    for mode in ('brake', 'coast'):
        param = profile.param('stop_' + mode)
        lo, hi = param['range']['decel_median_m_s2']
        for decel in (lo, param['value']['decel_median_m_s2'], hi):
            for throttle in (30., 60., 100.):
                value = dict(param['value'], decel_median_m_s2=decel)
                key = 'a4_%s_a%s_t%s' % (mode, _tag(decel * 10), _tag(throttle))
                out.append(ss.scene('a4_%s_%s' % (runner, key[3:]), stream_key=key, runner=runner,
                                    seeds=[stop_seed], backend='L0', duration_s=12.,
                                    profile=dict(overrides={'stop_' + mode: value}, override_label='stop'),
                                    maneuver=dict(kind='stop_mode', params=dict(mode=mode, throttle_pct=throttle,
                                                                                drive_s=4., hold_s=.5)), **harness))
    return out


def a4_judgement_scenes(runner='direct'):
    """The A4(F) judgement set (SIM_ACCEPTANCE amendment 1): a4_scenes() with the release scenes on the judgement
    seeds 10000-13999 (4000 per curve). Run it once for the judgement; check_a4 judges the release median only on
    this sample."""
    return a4_scenes(runner=runner, release_seeds=A4_RELEASE_JUDGEMENT_SEEDS)


def release_sample_kind(scene):
    """'amendment_1' when a release (stop_test) scene is the registered judgement sample (seeds exactly
    10000-13999, stream key a4_release_<curve>, L0, no overrides), else 'development' (a sample that is not a
    judgement, e.g. the 400 development seeds)."""
    curve = scene.profile.curve
    if (tuple(scene.seeds) == A4_RELEASE_JUDGEMENT_SEEDS and curve in A4_RELEASE_CURVES
            and scene.stream_key == a4_release_stream_key(curve) and scene.backend == 'L0'
            and not scene.profile.overrides and not scene.profile.geometry_check
            and scene.maneuver is not None and scene.maneuver.kind == 'stop_test'):
        return 'amendment_1'
    return 'development'


def a8_scenes(seeds=(0, 1)):
    """A8 / A11 set: maneuvers and the probe on the default profile (L1 native) and L0, direct and harness."""
    posts = fw.registration_posts()
    seeds = list(seeds)
    sweep = dict(kind='sine_sweep', params=dict(speed_m_s=.4, amplitude_deg=6., f0_hz=.05, f1_hz=.8, sweep_s=20.,
                                                start_s=2.))
    return [
        ss.scene('a8_circle_l1', seeds=seeds, maneuver=dict(kind='circle', params=dict(speed_m_s=.4, steer_deg=10.)),
                 duration_s=20.),
        ss.scene('a8_sine_l1', seeds=seeds, maneuver=sweep, duration_s=25.),
        ss.scene('a8_zigzag_heading_l1', seeds=seeds, duration_s=30.,
                 maneuver=dict(kind='zigzag', params=dict(speed_m_s=.4, amplitude_deg=12., switch='heading',
                                                          heading_deg=8., switches=6))),
        ss.scene('a8_stop_brake_l1', seeds=seeds, duration_s=20.,
                 maneuver=dict(kind='stop_mode', params=dict(mode='brake', speed_m_s=.6, drive_s=4., repeat=2))),
        ss.scene('a8_step_l0', seeds=seeds, backend='L0', duration_s=10.,
                 maneuver=dict(kind='step', params=dict(at_s=3., throttle_pct=50., steer_deg=[0., 15.]))),
        ss.scene('a8_probe_l1', seeds=seeds, duration_s=60.,
                 start=dict(x_m=.38, y_m=.3, condition='entry_lat_p03'),
                 controller=dict(kind='probe_pure_pursuit', params=dict(speed_m_s=.4, lookahead_m=2., row_length_m=10.))),
        ss.scene('a8_sine_l0_harness', seeds=seeds, backend='L0', runner='harness', estimator='sensor',
                 world=dict(markers=[list(p) for p in posts]), maneuver=sweep, duration_s=25.),
        ss.scene('a8_sine_l1_harness', seeds=seeds, runner='harness', estimator='sensor',
                 world=dict(markers=[list(p) for p in posts]), maneuver=sweep, duration_s=25.),
    ]


PROBE_LOOKAHEADS = (1.0, 1.5, 2.0, 2.5)
PROBE_RATES = (10., 15., 30., 60.)
PROBE_STARTS = ((.30, 3.), (.60, 10.))


def probe_scenes(lookaheads=PROBE_LOOKAHEADS, rates=PROBE_RATES, starts=PROBE_STARTS, seeds=tuple(range(12)),
                 speed=.4, row_length=30., backend='profile', geometry_check=False, prefix='probe'):
    """Probe re-run on the steered plant (default profile: L1 actuator with backlash, deadband, per-run zero offset,
    curvature wander, slip, eta, understeer): one scene per (lookahead, moving steering rate, start), the
    standstill rate is half the moving rate (5..30 deg/s). Starts are rear-axle (y0 m, yaw0 deg) like the probe.
    geometry_check=True gives the ideal comparison plant (L0 servo, no backlash / offsets / wander / slip)."""
    profile = load_profile()
    x_rear = profile.value('rear_axle_x_in_base_m')
    default_rates = (profile.value('steer_rate_max_deg_s'), profile.value('steer_rate_standstill_deg_s'))
    out = []
    for lookahead in lookaheads:
        for rate in rates:
            standstill = min(30., max(5., rate / 2.))
            overrides = {} if (rate, standstill) == default_rates else {'steer_rate_max_deg_s': rate,
                                                                        'steer_rate_standstill_deg_s': standstill}
            for y0, yaw0 in starts:
                yaw = math.radians(yaw0)
                profile = dict(overrides=overrides, override_label='probe_rate') if overrides else {}
                if geometry_check:
                    profile['geometry_check'] = True
                out.append(ss.scene(
                    '%s_ld%s_r%s_y%s_h%s' % (prefix, _tag(lookahead * 10), _tag(rate), _tag(y0 * 100), _tag(yaw0)),
                    seeds=list(seeds), backend=backend, duration_s=300., profile=profile,
                    start=dict(x_m=-x_rear * math.cos(yaw), y_m=y0 - x_rear * math.sin(yaw), yaw_deg=yaw0,
                               condition='probe_start'),
                    controller=dict(kind='probe_pure_pursuit', params=dict(speed_m_s=speed, lookahead_m=lookahead,
                                                                           row_length_m=row_length))))
    return out


def probe_set():
    """The probe re-run: the default-profile plant and the ideal comparison plant (same seeds and grid)."""
    return probe_scenes() + probe_scenes(geometry_check=True, prefix='probe_ideal')


SCENE_SETS = dict(a1=a1_scenes, a4=a4_scenes, a4_judgement=a4_judgement_scenes, a8=a8_scenes, probe=probe_set)
RUNNER_SETS = ('a1', 'a4', 'a4_judgement')          # sets that take --runner


# --- acceptance checks ------------------------------------------------------------------------------------------------------
def run_scene_specs(run_dir):
    """{scene name: SceneSpec} of a run folder (from RUN.json)."""
    doc = json.loads((Path(run_dir) / 'RUN.json').read_bytes().decode('utf-8'))
    return {item['name']: ss.SceneSpec.from_dict(item['spec']) for item in doc['scenes']}


def check_a1(run_dir):
    """A1(F): radius error <= 1 %, yaw-rate sign. Writes ACCEPTANCE_A1.json; returns it."""
    run_dir = Path(run_dir)
    specs = run_scene_specs(run_dir)
    rows = []
    for record in read_episodes(run_dir):
        if not record['scene_name'].startswith('a1_') or record.get('status') != 'ok':
            continue
        arrays, _ = load_trace(run_dir / record['trace']['file'])
        result = analyse_circle(arrays, scene_params(specs[record['scene_name']]).wheelbase_m)
        result.update(episode_id=record['episode_id'], runner=record['runner'],
                      passed=bool(result.get('ok') and result['radius_error_rel'] <= .01
                                  and result['yaw_rate_sign_matches']))
        rows.append(result)
    report = dict(check='A1(F)', criterion='R = L / tan(delta) error <= 1 %, yaw-rate sign (SIM_ACCEPTANCE A1)',
                  episodes=len(rows), passed=bool(rows) and all(r['passed'] for r in rows), rows=rows,
                  result_kind=RESULT_KIND_FALLBACK, field_validated=False)
    _write_json(run_dir / 'ACCEPTANCE_A1.json', _plain(report))
    return report


def check_a4(run_dir):
    """A4(F): steady speed +-1 % of forward_curve, ramp 100 %/s (255 PWM/s), release stop-distance median in
    [0.56, 0.60] m for stops from 0.90-0.95 m/s, brake / coast distance = v onset + v^2 / (2 a) within v dt and stop
    time within 2 dt + 0.005 / a (the speed snaps to 0 below 0.005 m/s). Writes ACCEPTANCE_A4.json.

    Release (SIM_ACCEPTANCE amendment 1): the median is judged only on the registered sample (release_sample_kind
    'amendment_1': seeds 10000-13999, stream key a4_release_<curve>, every episode ok). Any other sample is reported
    with passed None (not a judgement); the report's passed is then None unless something else failed."""
    run_dir = Path(run_dir)
    specs = run_scene_specs(run_dir)
    speed_rows, ramp_rows, stop_rows, release = [], [], [], {}
    release_scenes = {}                    # curve -> {scene name: (sample kind, planned seeds)}
    release_ok = {}                        # curve -> ok release episodes
    for name, scene in specs.items():
        if name.startswith('a4_') and scene.maneuver is not None and scene.maneuver.kind == 'stop_test':
            release_scenes.setdefault(scene_params(scene).curve, {})[name] = (release_sample_kind(scene),
                                                                              len(scene.seeds))
    for record in read_episodes(run_dir):
        name = record['scene_name']
        if not name.startswith('a4_') or record.get('status') != 'ok':
            continue
        scene = specs[name]
        params = scene_params(scene)
        arrays, _ = load_trace(run_dir / record['trace']['file'])
        kind = scene.maneuver.kind
        if kind == 'step':
            duty = float(scene.maneuver.params['throttle_pct'][1])
            expected = drive.steady_speed_m_s(params.profile, duty, scene.profile.curve)
            v_end = float(arrays['truth_v_m_s'][-1])
            speed_rows.append(dict(episode_id=record['episode_id'], curve=params.curve, duty_pct=duty,
                                   v_end_m_s=round(v_end, 6), expected_m_s=round(expected, 6),
                                   rel_error=round(abs(v_end - expected) / abs(expected), 8),
                                   passed=bool(abs(v_end - expected) <= .01 * abs(expected))))
            applied = arrays['drive_applied_pct']
            moving = (applied[:-1] != 0.) & (np.abs(applied[1:]) < abs(duty) - 1e-9)
            rising = np.abs(np.diff(applied))[moving]
            if len(rising):
                slope = float(np.median(rising)) / DT_S
                ramp_rows.append(dict(episode_id=record['episode_id'], slope_pct_per_s=round(slope, 6),
                                      slope_pwm_per_s=round(slope * 2.55, 4), steps=int(len(rising)),
                                      passed=bool(abs(slope - 100.) <= 5.)))
        elif kind == 'stop_test':
            release_ok[params.curve] = release_ok.get(params.curve, 0) + 1
            release.setdefault(params.curve, [])
            for stop in analyse_stops(arrays):
                if stop.get('completed') and .90 <= stop['v0_m_s'] <= .95:
                    release[params.curve].append(stop['distance_m'])
        elif kind == 'stop_mode':
            mode = scene.maneuver.params['mode']
            model = params.stop_mode(mode)
            for stop in analyse_stops(arrays):
                if not stop.get('completed'):
                    continue
                expected = model.distance_m(stop['v0_m_s'])
                expected_time = model.time_s(stop['v0_m_s'])
                stop_rows.append(dict(
                    episode_id=record['episode_id'], mode=mode, decel_m_s2=model.decel_median_m_s2,
                    v0_m_s=round(stop['v0_m_s'], 6), distance_m=round(stop['distance_m'], 6), expected_m=round(expected, 6),
                    time_s=round(stop['time_s'], 4), expected_time_s=round(expected_time, 4),
                    passed=bool(abs(stop['distance_m'] - expected) <= stop['v0_m_s'] * DT_S + 1e-9
                                and abs(stop['time_s'] - expected_time) <= 2. * DT_S + .005 / model.decel_median_m_s2)))
    release_rows = {}
    for curve, d in sorted(release.items()):
        kinds = release_scenes.get(curve, {})
        planned = sum(seeds for _, seeds in kinds.values())
        if kinds and all(kind == 'amendment_1' for kind, _ in kinds.values()) and len(kinds) == 1:
            sample = 'amendment_1' if release_ok.get(curve, 0) == planned else 'amendment_1_incomplete'
        else:
            sample = 'development'
        within = bool(d) and .56 <= statistics.median(d) <= .60
        release_rows[curve] = dict(
            n=len(d), median_m=round(statistics.median(d), 4) if d else None,
            p10_m=round(float(np.percentile(d, 10)), 4) if d else None,
            p90_m=round(float(np.percentile(d, 90)), 4) if d else None,
            sample=sample, scenes=sorted(kinds), episodes_ok=release_ok.get(curve, 0), episodes_planned=planned,
            within_window=within, passed=within if sample == 'amendment_1' else None)
    others_ok = bool(speed_rows and ramp_rows and stop_rows
                     and all(r['passed'] for r in speed_rows + ramp_rows + stop_rows))
    release_judged = bool(release_rows) and all(r['sample'] == 'amendment_1' for r in release_rows.values())
    if not others_ok or any(r['passed'] is False for r in release_rows.values()):
        passed = False
    elif release_judged and all(r['passed'] for r in release_rows.values()):
        passed = True
    else:
        passed = None                     # the release median was not judged (not the amendment 1 sample)
    report = dict(check='A4(F)', criteria=dict(
        speed='forward_curve +-1 %', ramp='255 PWM/s (100 %/s) +-5 %',
        release='median 0.56-0.60 m over every stop from 0.90-0.95 m/s of the amendment 1 sample (seeds '
                '10000-13999 per curve, stream key a4_release_<curve>); other samples are reported, not judged',
        brake_coast='closed form v onset + v^2 / (2 a) within v dt; time within 2 dt + 0.005 / a'),
        speed=speed_rows, ramp=ramp_rows, release=release_rows, brake_coast=stop_rows,
        release_judged=release_judged, passed=passed,
        result_kind=RESULT_KIND_FALLBACK, field_validated=False)
    _write_json(run_dir / 'ACCEPTANCE_A4.json', _plain(report))
    return report


def compare_runs(run_dirs):
    """A8: the same episodes in several run folders have the same trace content SHA-256 (and file SHA-256)."""
    maps = []
    for run_dir in run_dirs:
        maps.append({r['episode_id']: (r['trace']['content_sha256'], r['trace']['file_sha256'])
                     for r in read_episodes(run_dir) if r.get('status') == 'ok'})
    common = sorted(set.intersection(*[set(m) for m in maps])) if maps else []
    differ = [eid for eid in common if len({m[eid][0] for m in maps}) != 1]
    file_differ = [eid for eid in common if len({m[eid][1] for m in maps}) != 1]
    return dict(check='A8', runs=[str(r) for r in run_dirs], common=len(common), content_differs=differ,
                file_differs=file_differ, passed=bool(common) and not differ,
                only_in_some=sorted(set.union(*[set(m) for m in maps]) - set(common)) if maps else [])


def probe_table(run_dir):
    """Updated design 6.2 table from a probe run: pass counts per (lookahead, steering rate) over both starts and all
    seeds, plus the zero-offset steady state: mean rear-axle y over [10 m, row end] against the pure-pursuit
    prediction tan(zero offset) / L x L_d^2 / 2 (design 3.2 table). Writes PROBE_TABLE.json."""
    run_dir = Path(run_dir)
    specs = run_scene_specs(run_dir)
    rows, cells = [], {}
    for record in read_episodes(run_dir):
        if record['scene_name'] not in specs or record.get('status') != 'ok':
            continue
        scene = specs[record['scene_name']]
        if scene.controller is None or scene.controller.kind != 'probe_pure_pursuit':
            continue
        arrays, _ = load_trace(run_dir / record['trace']['file'])
        params = scene.controller.params
        result = analyse_probe(arrays, params['row_length_m'])
        draws = record['run_info']['draws']
        wheelbase = record['driver']['wheelbase_m']
        zero = draws['steer_zero_offset_deg']
        predicted = math.tan(math.radians(zero)) / wheelbase * params['lookahead_m'] ** 2 / 2.
        rate = float(scene_params(scene).profile.value('steer_rate_max_deg_s'))
        start = (round(float(arrays['truth_rear_y_m'][0]), 3), scene.start.yaw_deg)
        result.update(episode_id=record['episode_id'], lookahead_m=params['lookahead_m'], rate_deg_s=rate,
                      start_rear_y_m=start[0], start_yaw_deg=start[1], passed=probe_pass(result),
                      zero_offset_deg=round(zero, 4), eta=round(draws['steering_efficiency'], 4),
                      predicted_mean_y_m=round(predicted, 4))
        rows.append(result)
        variant = 'ideal' if scene.profile.geometry_check else 'profile'
        result['variant'] = variant
        cell = cells.setdefault((variant, params['lookahead_m'], rate), dict(passed=0, runs=0, lat_max=[], cmd_sd=[],
                                                                            by_start={}))
        cell['runs'] += 1
        cell['passed'] += int(result['passed'])
        block = cell['by_start'].setdefault('y%.2f_h%.0f' % start, [0, 0])
        block[0] += int(result['passed'])
        block[1] += 1
        if result['lat_max_after5m_m'] is not None:
            cell['lat_max'].append(result['lat_max_after5m_m'])
        if result['steer_cmd_std_after5m_deg'] is not None:
            cell['cmd_sd'].append(result['steer_cmd_std_after5m_deg'])
    table = []
    for (variant, lookahead, rate), cell in sorted(cells.items()):
        table.append(dict(variant=variant, lookahead_m=lookahead, rate_deg_s=rate, passed=cell['passed'],
                          runs=cell['runs'],
                          by_start={k: '%d/%d' % tuple(v) for k, v in sorted(cell['by_start'].items())},
                          lat_max_after5m_p50_m=round(statistics.median(cell['lat_max']), 4) if cell['lat_max'] else None,
                          lat_max_after5m_max_m=round(max(cell['lat_max']), 4) if cell['lat_max'] else None,
                          steer_cmd_sd_p50_deg=round(statistics.median(cell['cmd_sd']), 3) if cell['cmd_sd'] else None))
    fits = {}
    for lookahead in sorted({r['lookahead_m'] for r in rows}):
        done = [r for r in rows if r['lookahead_m'] == lookahead and r['variant'] == 'profile' and r['finished']
                and r['mean_y_10m_to_end_m'] is not None]
        if len(done) < 3:
            continue
        pred = np.array([r['predicted_mean_y_m'] for r in done])
        meas = np.array([r['mean_y_10m_to_end_m'] for r in done])
        slope = float(np.dot(pred, meas) / max(float(np.dot(pred, pred)), 1e-12))
        fits['%.1f' % lookahead] = dict(n=len(done), slope_measured_over_predicted=round(slope, 4),
                                        corr=round(float(np.corrcoef(pred, meas)[0, 1]), 4),
                                        residual_sd_m=round(float(np.std(meas - slope * pred)), 4),
                                        abs_mean_y_p50_m=round(float(np.median(np.abs(meas))), 4),
                                        abs_mean_y_max_m=round(float(np.max(np.abs(meas))), 4))
    report = dict(table=table, zero_offset_steady_state=fits, rows=rows, result_kind=RESULT_KIND_FALLBACK,
                  field_validated=False,
                  criterion='probe: finished row and max |y_rear| after 5 m < 0.30 m (not a divergence test)')
    _write_json(run_dir / 'PROBE_TABLE.json', _plain(report))
    return report


# --- CLI ----------------------------------------------------------------------------------------------------------------------
def _parse(argv):
    p = argparse.ArgumentParser(prog='python -B -m steering.fast_sim', description=__doc__.split('\n')[0])
    sub = p.add_subparsers(dest='command', required=True)
    run = sub.add_parser('run', help='run scenes into a run folder (outside the repository)')
    run.add_argument('--out', required=True)
    run.add_argument('--set', choices=sorted(SCENE_SETS), help='built-in scene set')
    run.add_argument('--scenes', help='JSON file with a list of scene dicts (scene_spec.load_scenes)')
    run.add_argument('--runner', choices=ss.RUNNERS, default='direct', help="runner for the a1 / a4 / a4_judgement sets")
    run.add_argument('--seeds', type=int, nargs='+', help='override the seeds of every scene')
    run.add_argument('--only', nargs='+', help='scene names to keep')
    run.add_argument('--workers', type=int, default=1)
    run.add_argument('--force', action='store_true')
    run.add_argument('--no-hold-check', action='store_true', help='do not look for HOLD_MEASUREMENT (tests only)')
    run.add_argument('--hold-poll-s', type=float, default=60.)
    check = sub.add_parser('check', help='acceptance analysis of a run folder')
    check.add_argument('--kind', choices=('a1', 'a4', 'probe'), required=True)
    check.add_argument('--run', required=True)
    compare = sub.add_parser('compare', help='A8: compare the trace SHA-256 of run folders')
    compare.add_argument('--runs', nargs='+', required=True)
    return p.parse_args(argv)


def main(argv=None):
    args = _parse(argv)
    if args.command == 'run':
        if (args.set is None) == (args.scenes is None):
            raise SystemExit('give exactly one of --set and --scenes')
        if args.scenes:
            scenes = ss.load_scenes(Path(args.scenes).read_bytes())
        elif args.set in RUNNER_SETS:
            scenes = SCENE_SETS[args.set](runner=args.runner)
        else:
            scenes = SCENE_SETS[args.set]()
        if args.only:
            scenes = [s for s in scenes if s.name in set(args.only)]
        if args.seeds:
            if args.set == 'a4_judgement':
                raise SystemExit('the a4_judgement set has its registered seeds (SIM_ACCEPTANCE amendment 1); '
                                 '--seeds would change the judgement sample')
            scenes = [s.with_changes(seeds=list(args.seeds)) for s in scenes]
        summary = run_scenes(scenes, args.out, workers=args.workers, force=args.force,
                             hold_check=not args.no_hold_check, hold_poll_s=args.hold_poll_s,
                             progress=lambda text: print(text, flush=True))
        print(json.dumps(dict(counts=summary['counts'], wall_s=summary['wall_s'],
                              trace_set_sha256=summary['trace_set_sha256'],
                              timing=summary['timing']['pooled']), indent=1, ensure_ascii=False))
        return 0 if summary['counts']['failed'] == 0 else 1
    if args.command == 'check':
        report = dict(a1=check_a1, a4=check_a4, probe=probe_table)[args.kind](args.run)
        brief = {k: v for k, v in report.items() if k not in ('rows', 'brake_coast')}
        print(json.dumps(_plain(brief), indent=1, ensure_ascii=False))
        passed = report.get('passed', True)
        return 0 if passed is True else (2 if passed is None else 1)     # 2: reported, not a judgement
    report = compare_runs(args.runs)
    print(json.dumps(report, indent=1))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
