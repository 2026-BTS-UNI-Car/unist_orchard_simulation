"""Steering fast-SIM runner (steering/fast_sim.py, steering/flat_world.py; design 4.5, 7.2, 8.3 P2).

Design 7.2 checks end to end through the runner: A1(F) circles (geometry_check, R = L / tan(delta) within 1 %),
A4(F) drive curve, ramp, release-stop median and brake / coast closed forms, A8 (same seed twice and 1 vs N
workers give the same trace SHA-256, same platform). Also: the W0 world and its LiDAR, the deterministic trace
file and its content hash, the trace format and its truth keys, stop rules, the maneuver-done end, skip / force /
tamper handling of a run folder, the HOLD_MEASUREMENT wait, plant.close() after every episode (also on failure),
ScriptActuation stop transitions, the probe controller, the direct runner == harness runner trajectory, the native
L1 backend and a W2 corridor episode when their prerequisites exist (otherwise skipped with the reason).

Every run folder is a pytest temporary folder; native builds go to UBTS_STEER_BUILD_DIR or a pytest temporary
folder (never into the work tree). The hold folder is a temporary folder (UBTS_HOLD_DIR), so the tests do not
depend on the real HOLD_MEASUREMENT file. Placeholder profile: placeholder-based SIM results, not field validation.

Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_fast_sim.py
"""
import json
import math
import os
import threading
import time

import numpy as np
import pytest

from steering import fast_sim as fs
from steering import flat_world as fw
from steering import maneuvers as mv
from steering import scene_spec as ss
from steering import steered_plant as sp
from steering.l0_servo import L0ServoBackend

CIRCLE = dict(kind='circle', params=dict(speed_m_s=.4, steer_deg=8.))
SWEEP = dict(kind='sine_sweep', params=dict(speed_m_s=.4, amplitude_deg=6., f0_hz=.1, f1_hz=.6, sweep_s=6.,
                                            start_s=1.))


@pytest.fixture(scope='module', autouse=True)
def native_build_dir_outside_the_work_tree(tmp_path_factory):
    if os.environ.get('UBTS_STEER_BUILD_DIR'):
        yield
        return
    os.environ['UBTS_STEER_BUILD_DIR'] = str(tmp_path_factory.mktemp('steer_native_build'))
    try:
        yield
    finally:
        os.environ.pop('UBTS_STEER_BUILD_DIR', None)


@pytest.fixture
def hold_dir(tmp_path, monkeypatch):
    path = tmp_path / 'hold'
    path.mkdir()
    monkeypatch.setenv('UBTS_HOLD_DIR', str(path))
    return path


def _native_or_skip():
    try:
        from steering.native import build as native_build
        native_build.find_compiler()
    except Exception as exc:
        pytest.skip('native steering backend not buildable here: %s' % exc)


def _harness_or_skip():
    try:
        from closed_loop import harness
    except ImportError as exc:      # e.g. the broken scipy of the WSL Python 3.10 (decision R10)
        pytest.skip('closed_loop.harness cannot be imported here: %s: %s' % (type(exc).__name__, exc))
    return harness


def l0(name, **fields):
    fields.setdefault('backend', 'L0')
    return ss.scene(name, **fields)


# --- W0 world ------------------------------------------------------------------------------------------------------------
def test_flat_world_interface():
    world = fw.FlatWorld(fw.FlatWorldConfig(markers=((5., 0., .2),)))
    assert world.to_field(1., 2., .3) == (1., 2., .3) and world.pose_in_field(1., 2., .3) == (1., 2., .3)
    truth = world.corridor_truth(0)
    assert truth['end_x'] == 1000. and truth['cross_walls_x'] == () and truth['next_left'] == 'UNKNOWN'
    with pytest.raises(ValueError):
        world.corridor_truth(1)
    hard, branch = world.clearance(0., 0., 0., 1.22, .70)
    assert hard == pytest.approx(5. - .61 - .2) and branch == math.inf
    hard, _ = world.clearance(4.6, 0., 0., 1.22, .70)
    assert hard < 0. and world.contacts == dict(samples=2, hard=1)
    assert world.describe()['physical_truth'] is False
    empty = fw.FlatWorld()
    assert empty.clearance(0., 0., 0., 1., 1.) == (math.inf, math.inf)
    with pytest.raises(ValueError):
        fw.FlatWorldConfig(markers=((0., 0., -1.),))


def test_registration_posts():
    posts = fw.registration_posts(count=4, spacing_m=5., offset_m=3., first_x_m=2., radius_m=.2)
    assert posts == ((2., 3., .2), (7., -3., .2), (12., 3., .2), (17., -3., .2))


def test_flat_lidar_is_exact_against_circles():
    world = fw.FlatWorld(fw.FlatWorldConfig(markers=((5., 0., .2), (0., -3., .5))))
    caster = fw.FlatLidarCaster(world)
    ranges = caster.cast(0., 0., 0.)
    i0 = int(np.argmin(np.abs(caster.angles)))
    assert ranges[i0] == pytest.approx(4.8, abs=1e-12)
    i_right = int(np.argmin(np.abs(caster.angles + math.pi / 2)))
    assert ranges[i_right] == pytest.approx(2.5, abs=1e-12)
    i_left = int(np.argmin(np.abs(caster.angles - math.pi / 2)))
    assert ranges[i_left] == math.inf
    rotated = caster.cast(0., 0., math.pi / 2)            # sensor yaw +90 deg: the post at +x is at -90 deg in the scan
    assert rotated[i_right] == pytest.approx(4.8, abs=1e-12)
    assert np.all(caster.cast(5., 0., 0.) == 0.)          # inside a post
    far = fw.FlatLidarCaster(fw.FlatWorld(fw.FlatWorldConfig(markers=((40., 0., .2),))))
    assert np.all(np.isinf(far.cast(0., 0., 0.)))         # beyond 30 m
    caster.occluded_sectors = ((-.1, .1),)
    ranges, origin, directions = caster.scan(np.array([0., 0., .6]), np.eye(3))
    assert ranges[i0] == math.inf and directions.shape == (fw.LIDAR_RAYS, 3) and origin.tolist() == [0., 0., .6]
    caster.occluded_sectors = ()
    _, synth = fw.make_flat_lidar(world, seed=3, noise_sd=0.)
    frame = synth.sample(100000000, (0., 0., .6), (0., 0., 0., 1.))
    assert float(frame.ranges[i0]) == pytest.approx(4.8, abs=1e-6) and len(frame.ranges) == fw.LIDAR_RAYS


# --- trace files ---------------------------------------------------------------------------------------------------------
def test_write_npz_is_deterministic_and_loadable(tmp_path):
    arrays = dict(b=np.arange(5, dtype=np.int16), a=np.linspace(0., 1., 7), names=np.array(['start', 'drive']))
    fs.write_npz(tmp_path / 'one.npz', arrays)
    time.sleep(1.1)                                        # zip member times would differ with a clock
    fs.write_npz(tmp_path / 'two.npz', dict(reversed(list(arrays.items()))))
    assert (tmp_path / 'one.npz').read_bytes() == (tmp_path / 'two.npz').read_bytes()
    with np.load(str(tmp_path / 'one.npz'), allow_pickle=False) as data:
        assert sorted(data.files) == ['a', 'b', 'names'] and data['b'].dtype == np.int16
        assert [str(x) for x in data['names']] == ['start', 'drive']
    digest = fs.content_sha256(arrays)
    assert digest == fs.content_sha256(dict(arrays))
    assert digest != fs.content_sha256(dict(arrays, b=np.arange(5, dtype=np.int32)))
    assert digest != fs.content_sha256(dict(arrays, a=arrays['a'].reshape(7, 1)))
    assert digest != fs.content_sha256(dict(arrays, a=arrays['a'] + 1e-15))
    assert digest != fs.content_sha256(dict(c=arrays['a'], b=arrays['b'], names=arrays['names']))


def test_trace_format_and_truth_keys():
    scene = l0('fmt_one', maneuver=CIRCLE, duration_s=3., start=dict(steer_deg=2.))
    arrays, timing, record = fs.run_episode(scene, 0)
    names = set(arrays)
    expected = set(fs.FLOAT_FIELDS) | {n for n, _ in fs.INT_FIELDS} | set(fs.DERIVED_FIELDS) | {'phase_names',
                                                                                                 'meta_json'}
    assert names == expected
    n = len(arrays['t_s'])
    assert n == 300 and all(arrays[k].shape == (n,) for k in names - {'phase_names', 'meta_json'})
    meta = json.loads(bytes(arrays['meta_json']).decode('utf-8'))
    assert meta['truth_keys'] == sorted(k for k in names if k.startswith('truth_'))
    assert meta['episode_scene_sha256'] == scene.episode_sha256(0) and meta['schema'] == fs.TRACE_SCHEMA
    assert meta['units']['truth_v_m_s'] == 'm/s' and meta['units']['cmd_steer_target_mrad'] == 'mrad'
    phases = [str(p) for p in arrays['phase_names']]
    assert phases == ['start', 'circle']
    first = int(np.argmax(arrays['phase'] == 1))
    assert np.all(np.isnan(arrays['cmd_steer_target_mrad'][:first])) and arrays['cmd_throttle_pct'][0] == 0.
    assert first == 10                                     # command at 0.05 s + latency 0.05 s -> step 11
    assert arrays['cmd_steer_target_mrad'][first] == pytest.approx(1000. * math.radians(8.))
    # MCU-side values carry no truth: the measured angle is the nominal decode of the raw ADC count
    params = fs.scene_params(scene)
    raw = arrays['pot_raw'][-1]
    decoded = params.geometry.link_table.wheel_from_pot((raw - params.pot_center_raw) / params.pot_counts_per_rad)
    assert arrays['steer_measured_rad'][-1] == decoded
    assert arrays['truth_rear_x_m'][0] == pytest.approx(params.rear_axle_x_in_base_m)
    assert np.array_equal(arrays['truth_field_x_m'], arrays['truth_x_m'])          # W0: field = world
    assert set(timing) == {'plant_step_us', 'backend_step_us', 'loop_step_us'} and len(timing['plant_step_us']) == n
    assert record['timing']['plant_step']['n'] == n and record['result_kind'] == '자리값 기반 SIM 결과'
    assert record['field_validated'] is False and record['truth_fed_driver'] is False
    assert record['backend']['cls'] == 'L0ServoBackend' and record['backend']['level'] == 'L0'
    assert record['profile_label']['actuator_backend']['matches_profile'] is True
    assert record['end']['end_reason'] == 'duration' and record['quick']['samples'] == n


def test_trace_every_thins_the_trace_not_the_timing():
    scene = l0('thin_one', maneuver=CIRCLE, duration_s=2., trace_every=5)
    arrays, timing, _ = fs.run_episode(scene, 0)
    full, _, _ = fs.run_episode(scene.with_changes(trace_every=1), 0)
    assert len(arrays['t_s']) == 40 and len(timing['plant_step_us']) == 200
    assert np.array_equal(arrays['truth_x_m'], full['truth_x_m'][::5])


# --- A1(F), A4(F) end to end -----------------------------------------------------------------------------------------------
def test_a1_circles_through_the_runner(tmp_path, hold_dir):
    scenes = fs.a1_scenes()
    assert len(scenes) == 8
    summary = fs.run_scenes(scenes, tmp_path / 'a1', workers=1)
    assert summary['counts']['ok'] == 8
    report = fs.check_a1(tmp_path / 'a1')
    assert report['passed'] and report['episodes'] == 8
    for row in report['rows']:
        assert row['radius_error_rel'] <= .01 and row['yaw_rate_sign_matches']   # A1(F)
        assert abs(row['turned_deg']) > 100.
    assert (tmp_path / 'a1' / 'ACCEPTANCE_A1.json').is_file()


def test_a4_through_the_runner(tmp_path, hold_dir):
    scenes = fs.a4_scenes()
    assert sum(len(s.seeds) for s in scenes if '_release_' in s.name) == 800
    summary = fs.run_scenes(scenes, tmp_path / 'a4', workers=4)
    assert summary['counts']['failed'] == 0
    report = fs.check_a4(tmp_path / 'a4')
    assert len(report['speed']) == 9 and all(r['rel_error'] <= .01 for r in report['speed'])       # +-1 %
    assert all(abs(r['slope_pct_per_s'] - 100.) < 1e-6 for r in report['ramp'])                   # 255 PWM/s
    assert len(report['brake_coast']) == 18 and all(r['passed'] for r in report['brake_coast'])
    # Release: the 400 development seeds are reported, never judged (SIM_ACCEPTANCE amendment 1: a sample of about
    # 100 stops put the field1010 median outside 0.56-0.60 m for about 1 in 4 seed sets). The judgement sample is
    # a4_judgement_scenes() (seeds 10000-13999), which tests never run.
    assert set(report['release']) == {'P_fit1003', 'field1010'}
    for curve, block in report['release'].items():
        assert block['sample'] == 'development' and block['passed'] is None, curve
        assert block['n'] > 50 and block['median_m'] is not None and block['episodes_ok'] == 400
    assert report['release_judged'] is False and report['passed'] is None          # nothing failed, nothing judged


def test_a4_judgement_sample_is_the_registered_one():
    scenes = fs.a4_judgement_scenes()
    release = [s for s in scenes if s.maneuver.kind == 'stop_test']
    assert sorted(s.profile.curve for s in release) == ['bracket', 'main']
    for scene in release:
        assert tuple(scene.seeds) == tuple(range(10000, 14000)) == fs.A4_RELEASE_JUDGEMENT_SEEDS
        assert scene.stream_key == 'a4_release_%s' % scene.profile.curve          # no runner name in the key
        assert fs.release_sample_kind(scene) == 'amendment_1' and scene.backend == 'L0'
    harness = [s for s in fs.a4_judgement_scenes(runner='harness') if s.maneuver.kind == 'stop_test']
    assert [s.stream_key for s in harness] == [s.stream_key for s in release]        # same draws in both runners
    development = [s for s in fs.a4_scenes() if s.maneuver.kind == 'stop_test']
    assert all(fs.release_sample_kind(s) == 'development' for s in development)
    assert not set(fs.A4_RELEASE_DEVELOPMENT_SEEDS) & set(fs.A4_RELEASE_JUDGEMENT_SEEDS)
    other = release[0].with_changes(seeds=list(range(10000, 13999)))                  # one seed short
    assert fs.release_sample_kind(other) == 'development'
    renamed = release[0].with_changes(stream_key='a4_direct_release_bracket')        # the run1 key form
    assert fs.release_sample_kind(renamed) == 'development'
    non_release = [s for s in scenes if s.maneuver.kind != 'stop_test']
    assert [s.to_dict() for s in non_release] == [s.to_dict() for s in fs.a4_scenes()
                                                  if s.maneuver.kind != 'stop_test']
    with pytest.raises(SystemExit, match='registered seeds'):
        fs.main(['run', '--set', 'a4_judgement', '--seeds', '1', '--out', 'unused'])


# --- A8 ---------------------------------------------------------------------------------------------------------------------
def _a8_set(include_native, include_harness):
    scenes = [l0('d_circle', maneuver=CIRCLE, duration_s=6., seeds=[0, 1]),
              l0('d_zigzag', duration_s=8., seeds=[0, 1], maneuver=dict(kind='zigzag', params=dict(
                  speed_m_s=.4, amplitude_deg=10., switch='heading', heading_deg=6., switches=4, start_s=1.))),
              ss.scene('d_probe', backend='L0', seeds=[0, 1], duration_s=30., start=dict(x_m=.38, y_m=.2),
                       controller=dict(kind='probe_pure_pursuit', params=dict(speed_m_s=.4, row_length_m=6.)))]
    if include_native:
        scenes.append(ss.scene('d_sweep_native', maneuver=SWEEP, duration_s=8., seeds=[0, 1]))
    if include_harness:
        scenes.append(l0('d_sweep_harness', runner='harness', estimator='null', maneuver=SWEEP, duration_s=8.,
                         seeds=[0, 1]))
    return scenes


def test_a8_same_seed_twice_and_worker_count(tmp_path, hold_dir):
    try:
        _native_or_skip()
        native = True
    except BaseException:
        native = False
    try:
        _harness_or_skip()
        harness = True
    except BaseException:
        harness = False
    scenes = _a8_set(native, harness)
    one = fs.run_scenes(scenes, tmp_path / 'w1a', workers=1)
    two = fs.run_scenes(scenes, tmp_path / 'w1b', workers=1)
    many = fs.run_scenes(scenes, tmp_path / 'w3', workers=3)
    assert one['counts']['failed'] == 0 and many['counts']['failed'] == 0
    assert one['traces'] == two['traces'] == many['traces']                                        # A8
    assert one['trace_set_sha256'] == many['trace_set_sha256']
    report = fs.compare_runs([tmp_path / 'w1a', tmp_path / 'w1b', tmp_path / 'w3'])
    assert report['passed'] and not report['file_differs'] and report['common'] == sum(len(s.seeds) for s in scenes)
    by_scene = {}
    for eid, digest in one['traces'].items():
        by_scene.setdefault(eid.split('__')[0], set()).add(digest)
    assert all(len(v) == 2 for v in by_scene.values())          # seeds 0 and 1 differ
    lines = (tmp_path / 'w3' / 'EPISODES.jsonl').read_bytes().decode('utf-8').splitlines()
    order = [json.loads(line)['episode_id'] for line in lines]
    assert order == [eid for s in scenes for eid, _ in s.episodes()]       # scene / seed order, not finish order
    assert len({json.loads(line)['worker_pid'] for line in lines}) >= 1
    assert many['timing']['pooled']['plant_step_us']['n'] > 0 and 'p999_us' in many['timing']['pooled']['loop_step_us']


def test_direct_and_harness_runners_give_the_same_trajectory():
    _harness_or_skip()
    for estimator, markers in (('null', []), ('sensor', [list(p) for p in fw.registration_posts()])):
        harness = l0('same_' + estimator, runner='harness', estimator=estimator, maneuver=SWEEP, duration_s=6.,
                     world=dict(markers=markers), seeds=[3])
        direct = harness.with_changes(runner='direct', estimator=None)
        a, _, ra = fs.run_episode(harness, 3)
        b, _, rb = fs.run_episode(direct, 3)
        for key in a:
            if key != 'meta_json':
                assert np.array_equal(a[key], b[key], equal_nan=a[key].dtype.kind == 'f'), key
        assert ra['harness']['end'] == 'time_limit' and ra['harness']['collisions'] == 0
        assert ra['run_info']['seed'] == rb['run_info']['seed']


def test_harness_seed_purpose_matches_the_harness():
    harness = _harness_or_skip()
    assert fs.PLANT_PURPOSE == harness.PURPOSES['plant']
    config = harness.EpisodeConfig(name='abc', stream_key='key_x', seed=5)
    import zlib
    assert fs.plant_seed('key_x', 5) == (config.seed, zlib.crc32(b'key_x'), harness.PURPOSES['plant'])


def test_harness_maneuver_done_ends_in_a_terminal_hold():
    _harness_or_skip()
    scene = l0('hold_end', runner='harness', estimator='null', duration_s=20.,
               maneuver=dict(kind='stop_mode', params=dict(mode='brake', speed_m_s=.5, drive_s=2., hold_s=.5)))
    arrays, _, record = fs.run_episode(scene, 0)
    assert record['harness']['end'] == 'terminal_hold' and record['end']['end_reason'] == 'maneuver_done'
    assert arrays['truth_v_m_s'][-1] == 0. and record['end']['end_t_s'] < 10.


# --- stop rules and ends ------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize('rule, value, check', [
    ('travel_m', 1.5, lambda a: a['truth_distance_m'][-1] >= 1.5 > a['truth_distance_m'][-2]),
    ('field_x_m', 1.0, lambda a: a['truth_field_x_m'][-1] >= 1.0 > a['truth_field_x_m'][-2]),
    ('turned_deg', 20., lambda a: abs(math.degrees(a['truth_yaw_unwrapped_rad'][-1])) >= 20.),
    ('abs_lateral_m', .2, lambda a: abs(a['truth_field_y_m'][-1]) >= .2 > abs(a['truth_field_y_m'][-2])),
])
def test_stop_rules_end_the_direct_episode(rule, value, check):
    scene = l0('rule_' + rule, maneuver=CIRCLE, duration_s=40., stop_rules=[dict(kind=rule, value=value)])
    arrays, _, record = fs.run_episode(scene, 0)
    assert record['end']['end_reason'] == 'stop_rule:' + rule and check(arrays)


def test_rest_rule_and_maneuver_done():
    stop = dict(kind='stop_mode', params=dict(mode='brake', speed_m_s=.5, drive_s=2., hold_s=5.))
    rest = l0('rule_rest', maneuver=stop, duration_s=30., stop_rules=[dict(kind='rest_s', value=1.)])
    arrays, _, record = fs.run_episode(rest, 0)
    assert record['end']['end_reason'] == 'stop_rule:rest_s'
    still = np.flatnonzero(arrays['truth_v_m_s'] != 0.)[-1]
    assert arrays['t_s'][-1] - arrays['t_s'][still] == pytest.approx(1.01, abs=.011)
    done = l0('end_done', maneuver=dict(kind='stop_mode', params=dict(mode='brake', speed_m_s=.5, drive_s=2.,
                                                                      hold_s=.5)), duration_s=30.)
    arrays, _, record = fs.run_episode(done, 0)
    assert record['end']['end_reason'] == 'maneuver_done' and arrays['truth_v_m_s'][-1] == 0.
    assert [str(p) for p in arrays['phase_names']] == ['start', 'drive', 'stop', 'rest']


# --- maneuvers on the plant -----------------------------------------------------------------------------------------------------
def test_stop_modes_reach_the_plant():
    for mode, drive_en in (('release', True), ('brake', True), ('coast', False)):
        scene = l0('mode_' + mode, duration_s=12.,
                   maneuver=dict(kind='stop_mode', params=dict(mode=mode, steer_policy='center', speed_m_s=.6,
                                                               drive_s=3., hold_s=.3, steer_deg=6.)))
        arrays, _, record = fs.run_episode(scene, 0)
        stops = fs.analyse_stops(arrays)
        assert len(stops) == 1 and stops[0]['completed'] and stops[0]['mode'] == mode
        assert int(arrays['drive_en'][-1]) == int(drive_en)
        assert arrays['steer_target_rad'][-1] == 0.                       # 'center' policy
        assert record['end']['end_reason'] == 'maneuver_done'


def test_script_actuation_calls_stop_once_per_change():
    class Recorder:
        def __init__(self):
            self.calls = []
            self.applied_command = fs.INITIAL_COMMAND

        def command_actuation(self, throttle, steer):
            self.calls.append(('drive', throttle, steer))

        def stop(self, mode, steer=None):
            self.calls.append(('stop', mode, steer))
    act = fs.ScriptActuation()
    assert isinstance(act, sp.SteeringActuation)
    plant = Recorder()
    sequence = [mv.Command(40., 10.), mv.Command(0., 10., stop_mode='brake', steer_policy='off'),
                mv.Command(0., 10., stop_mode='brake'), mv.Command(0., 0., stop_mode='coast'), mv.Command(30., 0.),
                mv.Command(0., 0., stop_mode='coast')]
    for command in sequence:
        act.apply(plant, act.values(command))
    assert plant.calls == [('drive', 40., 10.), ('stop', 'brake', 'off'), ('stop', 'coast', None), ('drive', 30., 0.),
                           ('stop', 'coast', None)]
    assert act.initial_values() == fs.INITIAL_COMMAND and act.is_reversal((10., 0., None, None, 'x'),
                                                                          (-5., 0., None, None, 'y'))
    assert act.trace_fields(act.values(sequence[1]))['stop_mode'] == 'brake'


def test_probe_controller_episode():
    scene = ss.scene('probe_short', backend='L0', duration_s=60., seeds=[2], start=dict(x_m=.38, y_m=.3,
                                                                                    condition='entry'),
                     controller=dict(kind='probe_pure_pursuit', params=dict(speed_m_s=.4, lookahead_m=2.,
                                                                            row_length_m=10.)))
    arrays, _, record = fs.run_episode(scene, 2)
    assert record['truth_fed_driver'] is True and record['driver']['truth_fed_driver'] is True
    assert record['end']['end_reason'] == 'maneuver_done'
    result = fs.analyse_probe(arrays, 10.)
    assert result['finished'] and fs.probe_pass(result) and result['lat_max_after5m_m'] < .1
    obs = arrays['ctrl_obs_offset_m']
    valid = arrays['ctrl_obs_valid'] == 1.
    changes = np.flatnonzero(np.diff(obs[valid]) != 0.)
    duration = float(arrays['t_s'][-1])
    assert 8. * duration <= len(changes) <= 10. * duration + 2   # 10 Hz observations
    first_valid = arrays['t_s'][np.argmax(valid)]
    assert first_valid == pytest.approx(.21, abs=1e-6)       # taken at 0.1 s, delivered at 0.2 s, used from step 21
    meta = json.loads(bytes(arrays['meta_json']).decode('utf-8'))
    assert meta['controller_keys'][0] == 'ctrl_obs_offset_m' and 'truth' in meta['controller_keys_note']


def test_pure_pursuit_port_formula():
    assert fs.pure_pursuit_curvature(0., 0., 2.) == 0.
    assert fs.pure_pursuit_curvature(.5, 0., 2.) == pytest.approx(2. * .5 / (4. + .25))
    assert fs.pure_pursuit_curvature(-.5, 0., 2.) < 0. < fs.pure_pursuit_curvature(0., .1, 2.)


# --- run folder: skip, force, tamper, failures, location -----------------------------------------------------------------------
def test_skip_force_and_tamper(tmp_path, hold_dir):
    scene = l0('skip_one', maneuver=CIRCLE, duration_s=2., seeds=[0, 1])
    out = tmp_path / 'run'
    first = fs.run_scenes([scene], out)
    assert first['counts'] == dict(planned=2, ok=2, failed=0, ran=2, reused=0)
    again = fs.run_scenes([scene], out)
    assert again['counts']['ran'] == 0 and again['counts']['reused'] == 2 and again['traces'] == first['traces']
    more = fs.run_scenes([scene.with_changes(seeds=[0, 1, 2])], out)
    assert more['counts']['ran'] == 1 and more['counts']['reused'] == 2      # the episode identity is per seed
    changed = fs.run_scenes([scene.with_changes(duration_s=2.5)], out)
    assert changed['counts']['ran'] == 2                                       # another scene SHA: run again
    trace = out / 'trace' / 'skip_one__seed0.npz'
    trace.write_bytes(trace.read_bytes() + b'x')
    tampered = fs.run_scenes([scene.with_changes(duration_s=2.5)], out)
    assert tampered['counts']['ran'] == 1
    forced = fs.run_scenes([scene.with_changes(duration_s=2.5)], out, force=True)
    assert forced['counts']['ran'] == 2
    run = json.loads((out / 'RUN.json').read_bytes().decode('utf-8'))
    assert run['status'] == 'done' and run['scenes'][0]['sha256'] == scene.with_changes(duration_s=2.5).sha256()
    assert run['identity']['files']['steering/fast_sim.py'] is not None


def test_failed_episode_is_recorded_and_the_plant_closed(tmp_path, hold_dir, monkeypatch):
    closed = []
    real = fs.make_backend

    def recording_backend(params, native_executable=None):
        backend = real(params, native_executable)
        close = backend.close

        def counted_close():
            closed.append(type(backend).__name__)
            close()
        backend.close = counted_close
        return backend
    monkeypatch.setattr(fs, 'make_backend', recording_backend)
    good = l0('fail_good', maneuver=CIRCLE, duration_s=1.)
    fs.run_episode(good, 0)
    assert len(closed) == 1                                  # closed after a normal episode

    def broken(self, t_s, state):
        if t_s > .5:
            raise RuntimeError('maneuver broke on purpose')
        return mv.Command(10., 0., phase='circle')
    monkeypatch.setattr(mv.Circle, 'command', broken)
    with pytest.raises(RuntimeError, match='on purpose'):
        fs.run_episode(good, 0)
    assert len(closed) == 2                                  # closed although the episode raised
    summary = fs.run_scenes([good], tmp_path / 'fail')
    assert summary['counts']['failed'] == 1 and len(closed) == 3
    record = json.loads((tmp_path / 'fail' / 'episodes' / 'fail_good__seed0.json').read_bytes().decode('utf-8'))
    assert record['status'] == 'failed' and 'on purpose' in record['error']['message']
    assert summary['failures'][0]['episode_id'] == 'fail_good__seed0'


def test_run_folder_inside_the_repository_is_refused(tmp_path, hold_dir):
    scene = l0('inside', maneuver=CIRCLE, duration_s=1.)
    with pytest.raises(ValueError, match='inside_the_repository'):
        fs.run_scenes([scene], fs.REPO_ROOT / 'runs_here')
    assert not (fs.REPO_ROOT / 'runs_here').exists()
    with pytest.raises(ValueError, match='workers'):
        fs.run_scenes([scene], tmp_path / 'too_many', workers=99)
    # review R12: every work tree of the repository counts (e.g. the main checkout), not only this one
    others = [r for r in ss.repository_roots() if r != fs.REPO_ROOT.resolve()]
    for root in others[:1]:
        with pytest.raises(ValueError, match='inside_the_repository'):
            fs.run_scenes([scene], root / 'runs_here_steer_test')
        assert not (root / 'runs_here_steer_test').exists()


def test_native_build_folder_must_be_set_and_outside_the_repository(tmp_path, hold_dir, monkeypatch):
    # review R12: without a build folder native/build.py would build next to the sources (git-ignored _build/)
    monkeypatch.delenv('UBTS_STEER_BUILD_DIR', raising=False)
    with pytest.raises(ValueError, match='native_build_dir_not_set'):
        fs.checked_native_build_dir()
    with pytest.raises(ValueError, match='native_build_dir_inside_the_repository'):
        fs.checked_native_build_dir(str(fs.REPO_ROOT / 'sim' / 'ubts' / 'steering' / 'native' / '_build'))
    assert fs.checked_native_build_dir(str(tmp_path / 'b')) == str(tmp_path / 'b')
    native = ss.scene('needs_native', backend='L1', maneuver=CIRCLE, duration_s=1.)
    with pytest.raises(ValueError, match='native_build_dir_not_set'):
        fs.run_scenes([native], tmp_path / 'run_no_build_dir')
    with pytest.raises(ValueError, match='native_build_dir_not_set'):
        fs.make_backend(fs.scene_params(native))
    monkeypatch.setenv('UBTS_STEER_BUILD_DIR', str(fs.REPO_ROOT / 'build_here'))
    with pytest.raises(ValueError, match='inside_the_repository'):
        fs.run_scenes([native], tmp_path / 'run_inside_build_dir')
    assert not (fs.REPO_ROOT / 'build_here').exists()
    fs.run_scenes([l0('l0_needs_no_build', maneuver=CIRCLE, duration_s=.5)], tmp_path / 'run_l0')   # L0: no build


def test_trace_meta_has_only_the_episode_identity(tmp_path, hold_dir):
    # review R6: the trace (content SHA, A8) must not depend on the other seeds of the scene
    scene = l0('meta_one', maneuver=CIRCLE, duration_s=1., seeds=[0, 1])
    a, _, _ = fs.run_episode(scene, 0)
    b, _, _ = fs.run_episode(scene.with_changes(seeds=[0, 1, 2]), 0)
    assert fs.content_sha256(a) == fs.content_sha256(b)
    meta = json.loads(bytes(a['meta_json']).decode('utf-8'))
    assert 'scene_sha256' not in meta and meta['episode_scene_sha256'] == scene.episode_sha256(0)
    assert 'hold_measured' in meta['steer_target_note']                            # review R9 marking
    fs.run_scenes([scene], tmp_path / 'A')
    more = scene.with_changes(seeds=[0, 1, 2])
    fs.run_scenes([more], tmp_path / 'A')
    fs.run_scenes([more], tmp_path / 'B')
    report = fs.compare_runs([tmp_path / 'A', tmp_path / 'B'])
    assert report['passed'] and report['common'] == 3 and not report['content_differs']
    records = {r['episode_id']: r for r in fs.read_episodes(tmp_path / 'A')}
    assert {r['run_scene_sha256'] for r in records.values()} == {more.sha256()}
    assert records['meta_one__seed0']['reused'] and records['meta_one__seed0']['scene_sha256'] == scene.sha256()
    assert records['meta_one__seed2']['scene_sha256'] == more.sha256()


def test_first_loop_sample_has_no_set_up_time():
    # review R7: the harness runner's first loop sample held the world / plant / estimator / LiDAR set-up
    scenes = [l0('loop_direct', maneuver=CIRCLE, duration_s=.5)]
    try:
        _harness_or_skip()
        scenes.append(l0('loop_harness', runner='harness', estimator='sensor', maneuver=CIRCLE, duration_s=.5,
                         world=dict(markers=[list(p) for p in fw.registration_posts()])))
    except BaseException:
        pass
    for scene in scenes:
        _, timing, _ = fs.run_episode(scene, 0)
        assert timing['loop_step_us'][0] == timing['plant_step_us'][0], scene.runner


def test_identity_covers_every_loaded_module(tmp_path):
    # review R5: the skip identity must name every sim/ubts source an episode imports (a fresh process, so pytest's own
    # imports do not count)
    import subprocess
    import sys
    script = '\n'.join([
        'import json, sys',
        'from pathlib import Path',
        'from steering import fast_sim as fs, scene_spec as ss, flat_world as fw',
        "C = dict(kind='circle', params=dict(speed_m_s=.4, steer_deg=8.))",
        "fs.run_episode(ss.scene('c0', backend='L0', maneuver=C, duration_s=.5), 0)",
        'try:',
        "    fs.run_episode(ss.scene('c1', backend='L0', runner='harness', estimator='sensor', maneuver=C,",
        "                            duration_s=.5, world=dict(markers=[list(p) for p in fw.registration_posts()])), 0)",
        'except ImportError:',
        '    pass',
        'if sys.argv[1] == "1":',
        "    fs.run_episode(ss.scene('c2', backend='L1', maneuver=C, duration_s=.5), 0)",
        'root = Path(fs.UBTS_DIR).resolve()',
        'loaded = set()',
        'for module in list(sys.modules.values()):',
        "    path = getattr(module, '__file__', None)",
        '    if path:',
        '        try:',
        '            loaded.add(Path(path).resolve().relative_to(root).as_posix())',
        '        except ValueError:',
        '            pass',
        'print(json.dumps(sorted(loaded)))'])
    try:
        _native_or_skip()
        native = '1'
    except BaseException:
        native = '0'
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    out = subprocess.run([sys.executable, '-B', '-c', script, native], cwd=str(fs.UBTS_DIR), env=env,
                         capture_output=True, timeout=300)
    assert out.returncode == 0, out.stderr.decode('utf-8', 'replace')[-2000:]
    loaded = set(json.loads(out.stdout.decode('utf-8').strip().splitlines()[-1]))
    assert loaded and loaded <= set(fs.IDENTITY_FILES), sorted(loaded - set(fs.IDENTITY_FILES))


def test_w2_identity_holds_the_catalogue_and_map_sha():
    try:
        from closed_loop import raster_corridor as rc
        rc.catalogue_path()
    except Exception as exc:
        pytest.skip('corridor catalogue not configured here: %s: %s' % (type(exc).__name__, exc))
    scene = l0('w2_identity', world=dict(kind='W2', record='042301', corridor_id='c02', direction='fwd'),
               maneuver=CIRCLE, duration_s=1.)
    data = fs.world_data_identity(scene.world)
    assert set(data) == {'catalogue_sha256', 'pgm_sha256', 'yaml_sha256'} and all(len(v) == 64 for v in data.values())
    base = dict(fs.code_identity(), **fs.environment_identity())
    identity, sha = fs._episode_identity(scene, base, None)
    assert identity['world_data'] == data
    changed = dict(data, pgm_sha256='0' * 64)
    assert fs._episode_identity(scene, base, None, changed)[1] != sha          # another map -> run again
    assert fs.world_data_identity(l0('w0', maneuver=CIRCLE).world) is None


def test_changed_code_during_the_episode_is_recorded_as_failed(tmp_path, hold_dir, monkeypatch):
    # review R5: the worker hashes the code again after the episode; a change since planning fails the episode
    real = fs.code_identity
    calls = []

    def changing():
        calls.append(1)
        identity = real()
        if len(calls) > 1:                      # the first call is the plan, the later ones the worker re-check
            files = dict(identity['files'], **{'steering/fast_sim.py': '0' * 64})
            identity = dict(files=files, code_sha256=ss.sha256_of(files))
        return identity
    monkeypatch.setattr(fs, 'code_identity', changing)
    scene = l0('changed_code', maneuver=CIRCLE, duration_s=.5)
    summary = fs.run_scenes([scene], tmp_path / 'run')
    assert summary['counts']['failed'] == 1
    record = fs.read_episodes(tmp_path / 'run')[0]
    assert record['status'] == 'failed' and record['error']['type'] == 'code_identity_changed'
    assert 'steering/fast_sim.py' in record['error']['message']
    assert not (tmp_path / 'run' / 'trace' / 'changed_code__seed0.npz').exists()


def test_harness_collision_uses_the_profile_footprint():
    # review R8: the harness ended episodes with the differential 1.22 x 0.70 m footprint whatever the profile said
    _harness_or_skip()
    from steering import evaluate_steer as es
    params = fs.scene_params(l0('fp', maneuver=CIRCLE))
    assert fs.harness_footprint_overrides(params) == (('footprint_length_m', 1.22), ('footprint_width_m', .70))
    narrow = dict(overrides={'footprint_width_m': .60}, override_label='narrow', geometry_check=True)
    drive = dict(kind='step', params=dict(at_s=0., throttle_pct=40.))
    scene = l0('fp_narrow', runner='harness', estimator='null', profile=narrow, duration_s=12., maneuver=drive,
               world=dict(markers=[[4., .33, .01]]))
    assert fs.harness_footprint_overrides(fs.scene_params(scene))[1] == ('footprint_width_m', .60)
    h, _, rh = fs.run_episode(scene, 0)
    d, _, _ = fs.run_episode(scene.with_changes(runner='direct', estimator=None), 0)
    assert rh['end']['end_reason'] != 'harness:collision'        # the 0.60 m body never touches the post
    config = es.EvalConfig.from_profile(fs.scene_params(scene).profile)
    world = es.world_for_scene(scene)
    ev_h = es.evaluate_trace(h, config, world=world, prereg_termination=False, corridor_metrics=False)
    ev_d = es.evaluate_trace(d, config, world=world, prereg_termination=False, corridor_metrics=False)
    assert ev_h['contacts']['body_only']['collision_samples'] == ev_d['contacts']['body_only']['collision_samples'] == 0
    assert (ev_h['contacts']['collision_samples'] > 0) == (ev_d['contacts']['collision_samples'] > 0)


def test_hold_file_wait(hold_dir):
    hold = hold_dir / 'HOLD_MEASUREMENT'
    assert fs.wait_while_held(poll_s=.01) == 0.
    hold.write_bytes(b'hold')
    with pytest.raises(TimeoutError):
        fs.wait_while_held(poll_s=.02, max_wait_s=.1)
    timer = threading.Timer(.3, hold.unlink)
    timer.start()
    messages = []
    try:
        waited = fs.wait_while_held(poll_s=.05, log=messages.append, max_wait_s=20.)
    finally:
        timer.cancel()
    assert waited >= .25 and messages and 'HOLD_MEASUREMENT' in messages[0]


def test_hold_check_needs_a_configured_folder(tmp_path, monkeypatch):
    monkeypatch.delenv('UBTS_HOLD_DIR', raising=False)
    monkeypatch.delenv('UBTS_STEER_PATHS', raising=False)
    from closed_loop import external_paths
    with pytest.raises(external_paths.PathNotConfigured):
        fs.run_scenes([l0('nohold', maneuver=CIRCLE, duration_s=1.)], tmp_path / 'nohold')
    summary = fs.run_scenes([l0('nohold', maneuver=CIRCLE, duration_s=1.)], tmp_path / 'nohold', hold_check=False)
    assert summary['counts']['ok'] == 1


# --- backends and worlds ----------------------------------------------------------------------------------------------------------
def test_backend_levels_follow_the_scene(monkeypatch):
    made = []
    real = fs.make_backend
    monkeypatch.setattr(fs, 'make_backend', lambda params, exe=None: made.append(real(params, exe)) or made[-1])
    _, _, record = fs.run_episode(l0('lvl_l0', maneuver=CIRCLE, duration_s=.5), 0)
    assert isinstance(made[-1], L0ServoBackend) and record['backend']['requested'] == 'L0'
    labels = record['profile_label']['overrides']
    assert labels['label'] == 'backend_L0' and labels['names'] == ['steer_actuator_level']
    _, _, record = fs.run_episode(ss.scene('lvl_geo', profile=dict(geometry_check=True), maneuver=CIRCLE,
                                           duration_s=.5), 0)
    assert isinstance(made[-1], L0ServoBackend) and record['profile_label']['overrides']['label'] == 'geometry_check'


def test_native_l1_episode_is_deterministic_and_closed(monkeypatch):
    _native_or_skip()
    made = []
    real = fs.make_backend
    monkeypatch.setattr(fs, 'make_backend', lambda params, exe=None: made.append(real(params, exe)) or made[-1])
    scene = ss.scene('native_one', maneuver=SWEEP, duration_s=4., seeds=[5])
    a, timing, record = fs.run_episode(scene, 5)
    b, _, _ = fs.run_episode(scene, 5)
    assert fs.content_sha256(a) == fs.content_sha256(b)
    assert record['backend']['cls'] == 'NativeSteeringBackend' and record['backend']['level'] == 'L1'
    assert record['backend']['label']['executable_sha256'] and record['backend']['label']['backend'] == 'native_lockstep_v1'
    assert all(not backend.running for backend in made)                    # every native process stopped
    assert np.isfinite(timing['backend_step_us']).all() and record['timing']['backend_step']['n'] == 400
    c, _, _ = fs.run_episode(scene.with_changes(seeds=[5, 6]), 6)
    assert fs.content_sha256(c) != fs.content_sha256(a)


def test_native_executable_is_pinned_for_a_run(tmp_path, hold_dir):
    _native_or_skip()
    scene = ss.scene('pinned', maneuver=CIRCLE, duration_s=1., seeds=[0, 1])
    summary = fs.run_scenes([scene], tmp_path / 'pin', workers=2)
    run = json.loads((tmp_path / 'pin' / 'RUN.json').read_bytes().decode('utf-8'))
    assert summary['counts']['ok'] == 2 and run['native']['binary_sha256']
    shas = {fs.read_episodes(tmp_path / 'pin')[k]['backend']['label']['executable_sha256'] for k in (0, 1)}
    assert shas == {run['native']['binary_sha256']}


def test_w2_corridor_direct_episode():
    try:
        from closed_loop import raster_corridor as rc
        rc.catalogue_path()
        rc.maps_root()
    except Exception as exc:                      # external data not configured / not present
        pytest.skip('W2 data not available here: %s: %s' % (type(exc).__name__, exc))
    scene = l0('w2_straight', world=dict(kind='W2', record='042301', corridor_id='c02', direction='fwd'),
               start=dict(x_m=2., y_m=.1), duration_s=12., maneuver=dict(kind='circle', params=dict(
                   speed_m_s=.4, steer_deg=0.)), stop_rules=[dict(kind='field_x_m', value=5.)])
    try:
        arrays, _, record = fs.run_episode(scene, 0)
    except ImportError as exc:
        pytest.skip('W2 world needs an import that fails here: %s' % exc)
    assert record['end']['end_reason'] == 'stop_rule:field_x_m'
    assert arrays['truth_field_x_m'][0] == pytest.approx(2., abs=.01)
    assert arrays['truth_field_x_m'][-1] >= 5. and abs(arrays['truth_field_y_m'][-1]) < .5
    assert not np.array_equal(arrays['truth_field_x_m'], arrays['truth_x_m'])       # raster-local != corridor frame
    assert record['world'] == dict(kind='W2', markers=0, record='042301', corridor_id='c02', direction='fwd')


# --- scene sets and CLI ---------------------------------------------------------------------------------------------------------------
def test_built_in_scene_sets_are_valid():
    for name, builder in fs.SCENE_SETS.items():
        scenes = builder()
        ss.check_unique_names(scenes)
        assert scenes and all(isinstance(s, ss.SceneSpec) for s in scenes), name
    harness_a1 = fs.a1_scenes(runner='harness')
    assert all(s.runner == 'harness' and not s.stop_rules for s in harness_a1)
    probe = fs.probe_scenes()
    assert len(probe) == 32 and all(len(s.seeds) == 12 for s in probe)
    assert {s.controller.params['lookahead_m'] for s in probe} == {1., 1.5, 2., 2.5}
    default = [s for s in probe if '_r' + 'p30_' in s.name]
    assert default and all(not s.profile.overrides for s in default)       # the profile rate itself: no override


def test_cli_run_check_and_compare(tmp_path, hold_dir):
    scenes = fs.a1_scenes()[:2]
    scene_file = tmp_path / 'scenes.json'
    scene_file.write_bytes(ss.dump_scenes(scenes))
    assert fs.main(['run', '--scenes', str(scene_file), '--out', str(tmp_path / 'cli1')]) == 0
    assert fs.main(['run', '--set', 'a1', '--only', scenes[0].name, '--out', str(tmp_path / 'cli2')]) == 0
    assert fs.main(['check', '--kind', 'a1', '--run', str(tmp_path / 'cli1')]) == 0
    assert fs.main(['compare', '--runs', str(tmp_path / 'cli1'), str(tmp_path / 'cli2')]) == 0
    with pytest.raises(SystemExit):
        fs.main(['run', '--out', str(tmp_path / 'cli3')])
