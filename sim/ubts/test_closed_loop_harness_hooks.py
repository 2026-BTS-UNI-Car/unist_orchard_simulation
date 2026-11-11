"""closed_loop.harness plant_factory / actuation hooks (SIM only).

(a) Default path: with both hooks left at None, and with the defaults passed explicitly, the fixed reference
    episodes give the same canonical results and traces as commit 546fa8c (the single-row version imported at
    47703e9). The reference is a folder outside the repository (key 'baseline_546fa8c_traces' or
    UBTS_BASELINE_546FA8C_TRACES, closed_loop/external_paths.py); the check is skipped when it is not
    configured or when this Python / numpy / platform differs from the one that recorded it.
(b) A fake steered plant and a fake actuation (stand-ins, not the real SteeredPlant): the plant receives
    (throttle_pct, steer_target_mrad) after the configured command latency; the reversal counter, the trace
    fields and the chatter counters come from the actuation object.
(c) A plant whose command() refuses differential commands fails loudly without an actuation hook.
"""
from dataclasses import asdict
import hashlib
import json
import math
import platform
import struct
import sys
import zlib

import numpy as np
import pytest

from closed_loop import external_paths
from closed_loop import harness as H
from closed_loop.controllers import Command
from closed_loop.harness import EpisodeConfig, run_episode
from closed_loop.world import WorldConfig, build_world

SHORT = WorldConfig(row_length_m=12.)

# --- canonical JSON (same rules as the reference generator baseline_546fa8c/generate_traces.py) --------------

BASELINE_SCHEMA = 'ubts.steer_sim.baseline_traces.v1'
LIDAR_TIMING_KEYS = ('ms_p50', 'ms_p99', 'ms_max', 'ms_mean')


def plain(value):
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise TypeError('non-string key %r' % (key,))
            out[key] = plain(item)
        return out
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        if math.isfinite(value):
            return float(value)
        return 'nan' if math.isnan(value) else ('inf' if value > 0 else '-inf')
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return plain(float(value))
    if isinstance(value, np.ndarray):
        return plain(value.tolist())
    raise TypeError('unsupported type %s' % type(value).__name__)


def canonical(value):
    return json.dumps(plain(value), sort_keys=True, separators=(',', ':'), ensure_ascii=True,
                      allow_nan=False).encode('ascii')


def sha256(raw):
    return hashlib.sha256(raw).hexdigest()


def strip_timing(result):
    """The result without its wall-clock fields (wall_s, raster_world.lidar.ms_*)."""
    out = {k: v for k, v in result.items() if k != 'wall_s'}
    world = out.get('raster_world')
    if isinstance(world, dict) and isinstance(world.get('lidar'), dict):
        world = dict(world)
        world['lidar'] = {k: v for k, v in world['lidar'].items() if k not in LIDAR_TIMING_KEYS}
        out['raster_world'] = world
    return out


def environment():
    return dict(python=platform.python_version(), python_implementation=platform.python_implementation(),
                numpy=np.__version__, sys_platform=sys.platform, machine=platform.machine(),
                pointer_bits=8 * struct.calcsize('P'))


# --- (a) the 546fa8c reference episodes --------------------------------------------------------------------

BASELINE_EPISODES = (
    dict(id='E1_matrix_nominal_open_end_continuous_s0', kind='matrix', scenario='nominal_open_end',
         mode='continuous', seed=0),
    dict(id='E2_matrix_heading_40deg_offset_stop_turn_s1', kind='matrix', scenario='heading_40deg_offset',
         mode='stop_turn', seed=1),
    dict(id='E3_matrix_lidar_blackout_dropouts_hybrid_s2', kind='matrix', scenario='lidar_blackout_dropouts',
         mode='hybrid', seed=2),
    dict(id='E4_single_row_drift_reversal_s0', kind='single_row', scene='drift_reversal', seed=0,
         mode='continuous', plant='field1010', single_overrides={}),
    dict(id='E5_raster_040927_c06_fwd_continuous', kind='raster', record='040927', corridor_id='c06',
         direction='fwd', name='baseline_raster_040927_c06_fwd', mode='continuous', seed=0, start_x_m=1.,
         max_time_s=120.),
)
BY_ID = {spec['id']: spec for spec in BASELINE_EPISODES}


def baseline():
    """(manifest, folder) of the 546fa8c reference, or pytest.skip with the reason it cannot be used here."""
    try:
        folder = external_paths.configured_path('baseline_546fa8c_traces', 'dir')
    except FileNotFoundError as error:
        pytest.skip('546fa8c reference traces not available: %s' % error)
    path = folder / 'MANIFEST.json'
    if not path.is_file():
        pytest.skip('546fa8c reference traces not available: no MANIFEST.json in %s' % folder)
    manifest = json.loads(path.read_bytes().decode('utf-8'))
    assert manifest['schema'] == BASELINE_SCHEMA
    if manifest['environment'] != environment():
        pytest.skip('546fa8c reference recorded with %r, this run is %r: bit identity is only compared in the '
                    'same environment' % (manifest['environment'], environment()))
    return manifest, folder


def build(spec):
    """(EpisodeConfig, run_episode keywords, controller description): identical to the reference generator."""
    if spec['kind'] == 'matrix':
        from closed_loop import matrix as M
        config, = M.episodes(seeds=(spec['seed'],), modes=(spec['mode'],), names=(spec['scenario'],))
        return config, {}, dict(factory='RowController (harness default)')
    if spec['kind'] == 'single_row':
        from closed_loop import single_row_eval as SRE
        from closed_loop.single_row import SingleRowController, make_single_row_config
        overrides = dict(spec['single_overrides'])
        config = SRE.episode(spec['scene'], spec['seed'], mode=spec['mode'], plant=spec['plant'])

        def factory(_control):
            return SingleRowController(make_single_row_config(**overrides))
        return config, dict(controller_factory=factory), dict(factory='SingleRowController',
                                                              config=asdict(make_single_row_config(**overrides)))
    if spec['kind'] == 'raster':
        from closed_loop import raster_corridor as RC
        try:
            RC.maps_root()
            RC.catalogue_path()
        except FileNotFoundError as error:
            pytest.skip('10/10 maps or the corridor catalogue not available: %s' % error)
        world = RC.corridor_world_config(spec['record'], spec['corridor_id'], spec['direction'])
        config = EpisodeConfig(name=spec['name'], mode=spec['mode'], seed=spec['seed'], world=world,
                               start_x_m=spec['start_x_m'], max_time_s=spec['max_time_s'])
        return config, {}, dict(factory='RowController (harness default)')
    raise ValueError(spec['kind'])


def run_reference_episode(spec, **hooks):
    config, kwargs, controller = build(spec)
    kwargs.update(hooks)
    if spec['kind'] == 'raster':
        from closed_loop import raster_corridor as RC
        result, _world = RC.run_raster_episode(config, record_trace=True, **kwargs)
    else:
        result = run_episode(config, record_trace=True, **kwargs)
    return config, controller, strip_timing(result)


def difference(stored_result, result):
    """Short text naming the first differences (for the assertion message)."""
    new = plain(result)
    keys = sorted(k for k in set(stored_result) | set(new) if stored_result.get(k) != new.get(k))
    text = 'differing keys: %s' % keys
    if 'trace' in keys:
        old_trace, new_trace = stored_result.get('trace', []), new.get('trace', [])
        for i, (a, b) in enumerate(zip(old_trace, new_trace)):
            if a != b:
                text += '; first trace difference at %d: %r vs %r' % (i, a, b)
                break
        else:
            text += '; trace lengths %d vs %d' % (len(old_trace), len(new_trace))
    return text[:2000]


def check_against_reference(spec, **hooks):
    manifest, folder = baseline()
    entry = next(e for e in manifest['episodes'] if e['id'] == spec['id'])
    assert entry['spec'] == spec, 'the reference episode definition changed'
    config, controller, result = run_reference_episode(spec, **hooks)
    assert sha256(canonical(dict(config=asdict(config), controller=controller))) == entry['config_sha256']
    result_sha = sha256(canonical(result))
    if result_sha != entry['result_sha256']:
        stored = json.loads((folder / entry['file']).read_bytes().decode('ascii'))['result']
        pytest.fail('%s differs from 546fa8c: %s' % (spec['id'], difference(stored, result)))
    assert sha256(canonical(result['trace'])) == entry['trace_sha256']


@pytest.mark.parametrize('episode_id', list(BY_ID))
def test_default_hooks_reproduce_the_546fa8c_reference(episode_id):
    check_against_reference(BY_ID[episode_id])


@pytest.mark.parametrize('episode_id', ['E2_matrix_heading_40deg_offset_stop_turn_s1',
                                        'E5_raster_040927_c06_fwd_continuous'])
def test_explicit_default_hooks_reproduce_the_546fa8c_reference(episode_id):
    check_against_reference(BY_ID[episode_id], plant_factory=H.field_plant_factory,
                            actuation=H.DifferentialActuation())


def test_explicit_default_hooks_equal_the_implicit_defaults():
    config = EpisodeConfig(name='hooks_identity', mode='hybrid', seed=3, world=SHORT, start_yaw_deg=6.,
                           start_y_m=.15, max_time_s=15.)
    calls = []

    def recording_factory(params, *, surface, seed, pose, config):
        calls.append(dict(surface=surface, seed=seed, pose=pose, config=config))
        return H.field_plant_factory(params, surface=surface, seed=seed, pose=pose, config=config)
    implicit = run_episode(config, record_trace=True)
    explicit = run_episode(config, record_trace=True, plant_factory=H.field_plant_factory,
                           actuation=H.DifferentialActuation())
    wrapped = run_episode(config, record_trace=True, plant_factory=recording_factory)
    assert canonical(implicit) == canonical(explicit) == canonical(wrapped)
    assert len(calls) == 1 and calls[0]['config'] is config and calls[0]['surface'] == config.surface
    assert calls[0]['seed'] == (config.seed, zlib.crc32(config.name.encode('utf-8')), H.PURPOSES['plant'])
    assert list(implicit['trace'][0])[5:7] == ['left', 'right']


def test_default_plant_factory_uses_the_module_level_field_plant(monkeypatch):
    # tools/row_executor sim_in_loop.py and single_row_sim.py (546fa8c, P4) replace harness.FieldPlant with a
    # FieldPlant subclass; with plant_factory=None that replacement must still be the plant of the episode
    created = []

    class RecordingPlant(H.FieldPlant):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            created.append(self)
    monkeypatch.setattr(H, 'FieldPlant', RecordingPlant)
    result = run_episode(EpisodeConfig(name='swapped_plant', world=SHORT, max_time_s=2.))
    assert len(created) == 1 and result['end'] == 'time_limit'


def test_default_actuation_keeps_the_previous_formulas():
    actuation = H.DifferentialActuation()
    rng = np.random.default_rng(7)
    assert actuation.initial_values() == (0, 0)
    for _ in range(2000):
        left, right, last_left, last_right = (int(v) for v in rng.integers(-100, 101, size=4))
        values = actuation.values(Command(left, right, 'FOLLOW', ''))
        assert values == (left, right)
        assert actuation.is_reversal(values, (last_left, last_right)) == (
            (left * last_left < 0) or (right * last_right < 0))
        diff = right - left
        assert actuation.turn_sign(values) == (0 if abs(diff) < 3 else (1 if diff > 0 else -1))
        assert list(actuation.trace_fields(values).items()) == [('left', left), ('right', right)]


# --- (b), (c) fake steered plant and fake actuation --------------------------------------------------------

class FakeSteeredPlant:
    """Stand-in for a steered plant (not the real SteeredPlant): kinematic bicycle on (throttle_pct,
    steer_target_mrad), the attributes run_episode reads, and a command() that refuses differential commands."""

    WHEELBASE_M = .76
    SPEED_PER_PCT = .004
    DT = .01

    def __init__(self, params, *, surface, seed, pose, config):
        self.factory_args = dict(params=params, surface=surface, seed=seed, pose=tuple(pose), config=config)
        self.x, self.y, self.yaw = (float(v) for v in pose)
        self.v = self.w = self.delta = 0.
        self.distance_m = 0.
        self.external_yaw_rate = 0.
        self.forward_blocked = False
        self.gyro_bias = 0.
        self.pivot_count = self.pivot_stalls = 0
        self.pivot_log = []
        self.steps = 0
        self.time_s = 0.
        self.applied = []                      # (plant time, throttle_pct, steer_target_mrad)

    def command(self, *args):
        raise RuntimeError('FakeSteeredPlant takes no differential command %r: use the steering actuation hook'
                           % (args,))

    def command_actuation(self, throttle_pct, steer_target_mrad):
        self.applied.append((self.time_s, throttle_pct, steer_target_mrad))
        self.v = self.SPEED_PER_PCT * throttle_pct
        self.delta = steer_target_mrad / 1000.

    def step(self, dt):
        assert dt == self.DT
        self.w = self.v * math.tan(self.delta) / self.WHEELBASE_M
        self.yaw += (self.w + self.external_yaw_rate) * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.distance_m += abs(self.v) * dt
        self.steps += 1
        self.time_s = self.steps * dt          # no accumulated rounding: plant time = steps x dt

    def imu_sample(self):
        return dict(stamp_s=self.time_s, gyro_z_rad_s=self.w + self.external_yaw_rate + self.gyro_bias,
                    accel_m_s2=(0., self.v * self.w, 9.81))

    def lidar_pose(self):
        return (self.x + .18 * math.cos(self.yaw), self.y + .18 * math.sin(self.yaw), self.yaw)


class SteerCommand:
    """What a steering controller hands over: no left/right, no reason."""

    def __init__(self, throttle_pct, steer_target_mrad):
        self.throttle_pct = throttle_pct
        self.steer_target_mrad = steer_target_mrad


class ScriptedController:
    """controller_factory stand-in: command k of a fixed schedule at the k-th controller step, never HOLD."""

    def __init__(self, schedule):
        self.schedule = schedule
        self.state, self.reason, self.hold_terminal = 'FOLLOW', None, False
        self.events = []
        self.counts = dict(stops=0, turns=0, stops_without_turn=0, stalls=0)
        self.issued = []                       # (now, throttle_pct, steer_target_mrad)

    def step(self, now, snapshot):
        throttle, steer = self.schedule(len(self.issued))
        self.issued.append((now, throttle, steer))
        return SteerCommand(throttle, steer)


class ThrottleSteerActuation:
    """Fake steering actuation of the hook contract: values (throttle_pct, steer_target_mrad)."""

    def __init__(self, initial=(0, 0)):
        self.initial = tuple(initial)
        self.values_calls = 0

    def values(self, command):
        self.values_calls += 1
        return (command.throttle_pct, command.steer_target_mrad)

    def apply(self, plant, values):
        plant.command_actuation(*values)

    def is_reversal(self, values, last_values):
        return values[0] * last_values[0] < 0          # drive direction flip

    def trace_fields(self, values):
        return dict(throttle_pct=values[0], steer_target_mrad=values[1])

    def initial_values(self):
        return self.initial


class SignedThrottleSteerActuation(ThrottleSteerActuation):
    def turn_sign(self, values):
        return 0 if abs(values[1]) < 10 else (1 if values[1] > 0 else -1)


def steered_episode(schedule, actuation, **overrides):
    """run_episode with the fake plant and the scripted controller; returns (result, plant, controller)."""
    holder = {}

    def plant_factory(params, *, surface, seed, pose, config):
        holder['plant'] = FakeSteeredPlant(params, surface=surface, seed=seed, pose=pose, config=config)
        return holder['plant']

    def controller_factory(_control):
        holder['controller'] = ScriptedController(schedule)
        return holder['controller']
    config = EpisodeConfig(**dict(dict(name='fake_steered', mode='continuous', world=SHORT, max_time_s=3.),
                                  **overrides))
    result = run_episode(config, record_trace=True, controller_factory=controller_factory,
                         plant_factory=plant_factory, actuation=actuation)
    return result, holder['plant'], holder['controller'], config


def cruise(k):
    return 25, (k * 7) % 41 - 20               # constant throttle, a unique-ish steer target per step


@pytest.mark.parametrize('latency_s', [.05, .15, .033])
def test_steered_values_reach_the_plant_after_the_command_latency(latency_s):
    actuation = ThrottleSteerActuation()
    result, plant, controller, config = steered_episode(cruise, actuation, command_latency_s=latency_s)
    assert result['end'] == 'time_limit' and result['collisions'] == 0
    assert len(controller.issued) == 60 and actuation.values_calls == 60
    pending = [c for c in controller.issued if c[0] + latency_s > config.max_time_s - FakeSteeredPlant.DT + 1e-9]
    assert len(plant.applied) == len(controller.issued) - len(pending) > 50
    for (t_applied, throttle, steer), (t_issued, throttle0, steer0) in zip(plant.applied, controller.issued):
        assert (throttle, steer) == (throttle0, steer0)                      # same values, same order
        lag = t_applied - (t_issued + latency_s)
        assert -1e-9 <= lag < FakeSteeredPlant.DT, (t_issued, t_applied)    # applied at the next plant step
    first = plant.applied[0][0] - controller.issued[0][0]
    assert first == pytest.approx(math.ceil(latency_s / FakeSteeredPlant.DT - 1e-6) * FakeSteeredPlant.DT,
                                  abs=1e-9)
    assert plant.factory_args['config'] is config
    assert plant.factory_args['seed'] == (config.seed, zlib.crc32(config.name.encode('utf-8')),
                                          H.PURPOSES['plant'])
    assert plant.factory_args['surface'] == config.surface
    centre_y = config.corridor_index * config.world.row_width_m
    expected_pose = build_world(config.world).pose_in_field(config.start_x_m, centre_y + config.start_y_m,
                                                            math.radians(config.start_yaw_deg))
    assert plant.factory_args['pose'] == tuple(expected_pose)
    assert plant.factory_args['params'] == H.plant_params(config)


def reversing(k):
    # drive direction flips at k = 10 (25 -> -20) and k = 40 (25 -> -20); 0 in between is not a reversal
    throttle = 25 if k < 10 else -20 if k < 20 else 0 if k < 25 else 25 if k < 40 else -20
    return throttle, 0


@pytest.mark.parametrize('initial, expected', [((0, 0), 2), ((-5, 0), 3)])
def test_reversal_counter_and_trace_fields_come_from_the_actuation(initial, expected):
    result, plant, controller, config = steered_episode(reversing, ThrottleSteerActuation(initial))
    assert result['reversal_commands'] == expected
    trace = result['trace']
    assert len(trace) == len(controller.issued) == 60
    for sample, (t_issued, throttle, steer) in zip(trace, controller.issued):
        assert sample['t'] == round(t_issued, 2)
        assert (sample['throttle_pct'], sample['steer_target_mrad']) == (throttle, steer)
        assert 'left' not in sample and 'right' not in sample
    assert list(trace[0]) == ['t', 'fx', 'fy', 'fyaw_deg', 'v', 'throttle_pct', 'steer_target_mrad', 'state',
                              'reason', 'est_heading_deg', 'est_lateral', 'corridor_reason']
    assert result['plant_pivots'] == 0 and result['plant_pivot_rates'] == []


def zigzag(k):
    # steer sign: + (k < 10), - (10..14, a rapid flip), 0 (15..64), + (65.., a flip after 2.55 s: not rapid)
    steer = 50 if k < 10 else -50 if k < 15 else 0 if k < 65 else 50
    return 20, steer


def test_chatter_counters_use_turn_sign_and_are_none_without_it():
    with_sign, _, _, _ = steered_episode(zigzag, SignedThrottleSteerActuation(), max_time_s=4.)
    assert (with_sign['chatter_flips'], with_sign['rapid_flips']) == (2, 1)
    assert with_sign['chatter_per_min'] == pytest.approx(30.)
    without, _, _, _ = steered_episode(zigzag, ThrottleSteerActuation(), max_time_s=4.)
    assert without['chatter_flips'] is None and without['chatter_per_min'] is None and without['rapid_flips'] is None
    assert without['reversal_commands'] == with_sign['reversal_commands'] == 0
    assert without['trace'] == with_sign['trace']


def test_incomplete_actuation_object_is_rejected():
    class NoTrace(ThrottleSteerActuation):
        trace_fields = None
    with pytest.raises(TypeError, match='trace_fields'):
        steered_episode(cruise, NoTrace())
    built = []
    with pytest.raises(TypeError, match='values, apply, is_reversal, trace_fields, initial_values'):
        run_episode(EpisodeConfig(world=SHORT, max_time_s=1.), actuation=object(),
                    world_factory=lambda world_config: built.append(world_config) or build_world(world_config))
    assert built == []                                  # rejected before any (possibly costly) world is built


@pytest.mark.parametrize('actuation', [None, H.DifferentialActuation()], ids=['no_hook', 'differential_hook'])
def test_steered_plant_without_the_steering_actuation_fails_loudly(actuation):
    holder = {}

    def plant_factory(params, *, surface, seed, pose, config):
        holder['plant'] = FakeSteeredPlant(params, surface=surface, seed=seed, pose=pose, config=config)
        return holder['plant']
    config = EpisodeConfig(name='steered_without_hook', world=SHORT, max_time_s=3.)
    with pytest.raises(RuntimeError, match='use the steering actuation hook'):
        run_episode(config, plant_factory=plant_factory, actuation=actuation)
    plant = holder['plant']
    assert plant.applied == []
    # the first controller command (t = 0.05 s) reaches the plant after the 0.05 s latency: the error is raised
    # there, at plant step 10, not swallowed and not deferred to the end of the episode
    assert plant.steps == 10
