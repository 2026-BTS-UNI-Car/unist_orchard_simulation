"""Scene specification of the steering fast SIM (steering/scene_spec.py; design 7.3, 8.3 P2).

Canonical JSON and SHA-256 (defaults filled in, key order and int / float spelling do not matter outside the
free-form profile overrides), the single-seed episode identity, validation of every part, the sealed-corridor
guard and the list file round trip. Pure Python; runs on Python 3.10 / 3.12 / 3.13.

Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_scene_spec.py
"""
import json
import os
from pathlib import Path

import pytest

from steering import maneuvers as mv
from steering import scene_spec as ss

CIRCLE = dict(kind='circle', params=dict(speed_m_s=.4, steer_deg=10.))


def minimal(**extra):
    data = dict(name='circle_one', maneuver=CIRCLE)
    data.update(extra)
    return data


def test_defaults_are_written_and_the_sha_is_stable():
    a = ss.SceneSpec.from_dict(minimal())
    explicit = dict(schema=ss.SCHEMA, name='circle_one', stream_key='circle_one',
                    world=dict(kind='W0', markers=[], record=None, corridor_id=None, direction='fwd', modifiers=[],
                               allow_sealed=False),
                    start=dict(x_m=0, y_m=0, yaw_deg=0, steer_deg=0, condition='nominal'),
                    profile=dict(profile_id=ss.DEFAULT_PROFILE_ID, sha256=None, overrides={}, override_label=None,
                                 allow_out_of_range=False, curve='main', geometry_check=False, steer_mode='test_servo'),
                    seeds=[0], backend='profile', runner='direct', estimator=None,
                    maneuver=dict(kind='circle', params=dict(speed_m_s=.4, steer_deg=10, throttle_pct=None)),
                    controller=None, duration_s=30, command_latency_s=.05, gyro_noise_scale=1, stop_rules=[],
                    trace_every=1)
    b = ss.SceneSpec.from_dict(explicit)
    assert a == b and a.sha256() == b.sha256()
    assert a.canonical_json() == b.canonical_json()
    assert len(a.sha256()) == 64 and a.sha256() == ss.sha256_of(a.to_dict())
    # canonical form: sorted keys, compact, ASCII; to_dict round trip gives the same scene
    raw = a.canonical_json()
    assert raw == json.dumps(json.loads(raw.decode('ascii')), sort_keys=True, separators=(',', ':')).encode('ascii')
    assert ss.SceneSpec.from_dict(a.to_dict()) == a
    reordered = dict(reversed(list(minimal().items())))
    assert ss.SceneSpec.from_dict(reordered).sha256() == a.sha256()


def test_every_field_changes_the_sha():
    base = ss.SceneSpec.from_dict(minimal())
    changes = [dict(seeds=[0, 1]), dict(duration_s=31.), dict(command_latency_s=0.), dict(backend='L0'),
               dict(start=dict(y_m=.1)), dict(start=dict(steer_deg=10.)), dict(profile=dict(curve='bracket')),
               dict(world=dict(markers=[[6., 4., .1]])), dict(stream_key='other_key'), dict(trace_every=2),
               dict(maneuver=dict(kind='circle', params=dict(speed_m_s=.4, steer_deg=11.))),
               dict(stop_rules=[dict(kind='turned_deg', value=90.)]), dict(gyro_noise_scale=2.)]
    shas = {base.sha256()}
    for change in changes:
        shas.add(ss.SceneSpec.from_dict(minimal(**change)).sha256())
    assert len(shas) == len(changes) + 1


def test_episode_identity_is_the_single_seed_scene():
    scene = ss.SceneSpec.from_dict(minimal(seeds=[3, 1, 7]))
    assert scene.episodes() == [('circle_one__seed3', 3), ('circle_one__seed1', 1), ('circle_one__seed7', 7)]
    single = ss.SceneSpec.from_dict(minimal(seeds=[1]))
    assert scene.episode_sha256(1) == single.sha256()
    more = ss.SceneSpec.from_dict(minimal(seeds=[3, 1, 7, 9]))
    assert more.episode_sha256(1) == scene.episode_sha256(1)          # adding a seed keeps the other episodes
    assert more.sha256() != scene.sha256()
    assert scene.episode_sha256(3) != scene.episode_sha256(1)
    with pytest.raises(ss.SceneError, match='not_in_scene'):
        scene.episode_sha256(2)


def test_maneuver_params_are_normalised():
    scene = ss.SceneSpec.from_dict(dict(name='step_one', maneuver=dict(kind='step', params=dict(throttle_pct=40,
                                                                                               steer_deg=[0, 5]))))
    assert scene.maneuver.params == dict(at_s=1., throttle_pct=[40., 40.], steer_deg=[0., 5.])
    same = ss.SceneSpec.from_dict(dict(name='step_one', maneuver=dict(kind='step', params=dict(
        at_s=1, throttle_pct=[40., 40], steer_deg=[0., 5.]))))
    assert same.sha256() == scene.sha256()
    assert scene.driver is scene.maneuver and scene.driver_role == 'maneuver'


def test_profile_overrides_get_a_label_and_are_kept_as_written():
    scene = ss.SceneSpec.from_dict(minimal(profile=dict(overrides={'steer_backlash_deg': 4.0})))
    assert scene.profile.override_label == 'scene_circle_one'
    assert scene.profile.overrides == {'steer_backlash_deg': 4.0}
    spelled = ss.SceneSpec.from_dict(minimal(profile=dict(overrides={'steer_backlash_deg': 4})))
    assert spelled.sha256() != scene.sha256()            # free-form overrides are not normalised (documented)
    with pytest.raises(ss.SceneError, match='override_label_without_overrides'):
        ss.SceneSpec.from_dict(minimal(profile=dict(override_label='x')))
    with pytest.raises(ss.SceneError, match='not_json'):
        ss.SceneSpec.from_dict(minimal(profile=dict(overrides={'steer_backlash_deg': float('nan')})))


@pytest.mark.parametrize('change, code', [
    (dict(name='Bad Name'), 'scene_name_invalid'),
    (dict(name=''), 'scene_name_invalid'),
    (dict(extra=1), 'scene_key_unknown'),
    (dict(seeds=[]), 'scene_seeds_invalid'),
    (dict(seeds=[1, 1]), 'scene_seeds_repeated'),
    (dict(seeds=[-1]), 'scene_seed_invalid'),
    (dict(seeds=[True]), 'scene_seed_invalid'),
    (dict(backend='L2'), 'scene_backend_unknown'),
    (dict(runner='gazebo'), 'scene_runner_unknown'),
    (dict(estimator='sensor'), 'scene_estimator_needs_the_harness_runner'),
    (dict(runner='harness', estimator='oracle'), 'scene_estimator_unknown'),
    (dict(duration_s=0.), 'scene_duration_out_of_range'),
    (dict(duration_s=float('inf')), 'scene_value_not_finite'),
    (dict(command_latency_s=-.1), 'scene_command_latency_out_of_range'),
    (dict(trace_every=0), 'scene_trace_every_invalid'),
    (dict(stop_rules=[dict(kind='teleport', value=1.)]), 'scene_stop_rule_unknown'),
    (dict(stop_rules=[dict(kind='travel_m', value=-1.)]), 'scene_stop_rule_value_not_positive'),
    (dict(stop_rules=[dict(kind='travel_m', value=1.), dict(kind='travel_m', value=2.)]), 'scene_stop_rule_repeated'),
    (dict(runner='harness', stop_rules=[dict(kind='travel_m', value=1.)]), 'stop_rules_need_the_direct_runner'),
    (dict(profile=dict(steer_mode='direct_pwm')), 'profile_steer_mode_not_supported'),
    (dict(profile=dict(curve='fast')), 'profile_curve_unknown'),
    (dict(profile=dict(geometry_check=True), backend='L1'), 'scene_geometry_check_runs_on_l0'),
    (dict(profile=dict(profile_id='../x')), 'profile_id_invalid'),
    (dict(profile=dict(sha256='abc')), 'profile_sha256_invalid'),
    (dict(start=dict(steer_deg=70.)), 'start_steer_out_of_range'),
    (dict(start=dict(condition='bad label!')), 'start_condition_invalid'),
    (dict(world=dict(kind='W1')), 'world_kind_unknown'),
    (dict(world=dict(markers=[[1., 2.]])), 'world_marker_needs_x_y_radius'),
    (dict(world=dict(markers=[[1., 2., 0.]])), 'world_marker_radius_not_positive'),
    (dict(world=dict(kind='W0', record='042301')), 'world_w0_takes_markers_only'),
    (dict(world=dict(kind='W2', record='42301', corridor_id='c02')), 'world_w2_record_invalid'),
    (dict(world=dict(kind='W2', record='042301', corridor_id='L1-R8')), 'world_w2_corridor_id_invalid'),
    (dict(world=dict(kind='W2', record='042301', corridor_id='c02', markers=[[1., 1., .1]])),
     'world_w2_takes_no_markers'),
    (dict(maneuver=None), 'exactly_one_of_maneuver_and_controller'),
    (dict(maneuver=dict(kind='loop', params={})), 'scene_maneuver_invalid'),
    (dict(maneuver=dict(kind='circle', params=dict(steer_deg=10.))), 'scene_maneuver_invalid'),
])
def test_invalid_scenes_are_refused(change, code):
    data = minimal()
    data.update(change)
    if data.get('maneuver') is None:
        data.pop('maneuver')
    with pytest.raises(ss.SceneError, match=code):
        ss.SceneSpec.from_dict(data)


def test_sealed_corridors_are_refused_unless_allowed():
    for record, corridor, direction in ss.SEALED_DIRECTIONS:
        world = dict(kind='W2', record=record, corridor_id=corridor, direction=direction)
        with pytest.raises(ss.SceneError, match='sealed report-only'):
            ss.SceneSpec.from_dict(minimal(world=world))
        allowed = ss.SceneSpec.from_dict(minimal(world=dict(world, allow_sealed=True)))
        assert allowed.world.allow_sealed is True
    other = ss.SceneSpec.from_dict(minimal(world=dict(kind='W2', record='042301', corridor_id='c02', direction='rev')))
    assert other.world.kind == 'W2' and not other.world.allow_sealed


def test_reverse_of_a_sealed_row_pair_is_refused_unless_allowed():
    # design 7.3: the same row pair reversed is not development evidence (review R10: c01 / c04 rev were accepted)
    assert ss.SEALED_PAIR_REVERSE_DIRECTIONS == (('042301', 'c01', 'rev'), ('042301', 'c04', 'rev'),
                                                 ('042301', 'c07', 'rev'))
    for record, corridor, direction in ss.SEALED_PAIR_REVERSE_DIRECTIONS:
        assert ss.sealed_status(record, corridor, direction) == 'sealed_pair_reverse'
        world = dict(kind='W2', record=record, corridor_id=corridor, direction=direction)
        with pytest.raises(ss.SceneError, match='reverse direction of a sealed row pair'):
            ss.SceneSpec.from_dict(minimal(world=world))
        assert ss.SceneSpec.from_dict(minimal(world=dict(world, allow_sealed=True))).world.allow_sealed
    assert ss.sealed_status('042301', 'c01', 'fwd') == 'sealed'
    assert ss.sealed_status('042301', 'c02', 'rev') is None and ss.sealed_status('040927', 'c01', 'fwd') is None


def test_repository_roots_cover_this_work_tree_and_its_repository(tmp_path):
    roots = ss.repository_roots()
    assert ss.WORK_TREE_ROOT.resolve() in roots
    assert ss.inside_repository(ss.WORK_TREE_ROOT / 'sim' / 'ubts' / 'some_run_folder')
    assert not ss.inside_repository(tmp_path / 'run')
    # A fake repository with a main work tree and a linked one (git files only, no git command)
    main = tmp_path / 'main'
    linked = tmp_path / 'linked'
    (main / '.git' / 'worktrees' / 'linked').mkdir(parents=True)
    linked.mkdir()
    (linked / '.git').write_bytes(('gitdir: %s\n' % (main / '.git' / 'worktrees' / 'linked').as_posix()).encode())
    (main / '.git' / 'worktrees' / 'linked' / 'commondir').write_bytes(b'../..\n')
    (main / '.git' / 'worktrees' / 'linked' / 'gitdir').write_bytes(
        ('%s\n' % (linked / '.git').as_posix()).encode())
    found = ss.repository_roots(linked)
    assert main.resolve() in found and linked.resolve() in found
    assert ss.inside_repository(main / 'runs', roots=found) and ss.inside_repository(linked / 'x', roots=found)
    assert not ss.inside_repository(tmp_path / 'elsewhere', roots=found)
    assert ss.repository_roots(main) == sorted({main.resolve(), linked.resolve()})


def test_git_file_paths_cross_the_windows_wsl_mount_and_may_be_relative(tmp_path):
    # Windows git writes drive paths (D:/...) into the git files; on WSL they name /mnt/d/... (review R12: before
    # this, WSL saw only this work tree and a run folder in the main checkout was accepted there)
    if os.name == 'nt':
        assert ss._git_file_path('/mnt/d/Develop/x/.git\n', tmp_path) == Path('D:/Develop/x/.git')
        assert ss._git_file_path('D:/Develop/x/.git', tmp_path) == Path('D:/Develop/x/.git')
    else:
        assert ss._git_file_path('D:/Develop/x/.git\n', tmp_path) == Path('/mnt/d/Develop/x/.git')
        assert ss._git_file_path('D:\\Develop\\x\\.git', tmp_path) == Path('/mnt/d/Develop/x/.git')
        assert ss._git_file_path('/mnt/d/Develop/x/.git', tmp_path) == Path('/mnt/d/Develop/x/.git')
    assert ss._git_file_path('../..', tmp_path) == tmp_path / '..' / '..'
    # relative paths in every git file (git worktree.useRelativePaths): each is relative to its own folder
    main = tmp_path / 'main'
    linked = tmp_path / 'linked'
    entry = main / '.git' / 'worktrees' / 'linked'
    entry.mkdir(parents=True)
    linked.mkdir()
    (linked / '.git').write_bytes(b'gitdir: ../main/.git/worktrees/linked\n')
    (entry / 'commondir').write_bytes(b'../..\n')
    (entry / 'gitdir').write_bytes(b'../../../../linked/.git\n')
    both = sorted({main.resolve(), linked.resolve()})
    assert ss.repository_roots(linked) == both
    assert ss.repository_roots(main) == both
    assert ss.inside_repository(linked / 'runs', roots=ss.repository_roots(main))


def test_harness_runner_defaults_to_the_sensor_estimator():
    scene = ss.SceneSpec.from_dict(minimal(runner='harness'))
    assert scene.estimator == 'sensor'
    assert ss.SceneSpec.from_dict(minimal(runner='harness', estimator='null')).estimator == 'null'
    direct = ss.SceneSpec.from_dict(minimal(estimator=None))
    assert direct.estimator is None


def test_controller_scenes():
    params = dict(speed_m_s=.4, lookahead_m=2.)
    scene = ss.SceneSpec.from_dict(dict(name='probe_one', controller=dict(kind='probe_pure_pursuit', params=params)))
    assert scene.driver_role == 'controller' and scene.maneuver is None
    assert scene.controller.params['obs_latency_s'] == .1 and scene.controller.params['wheelbase_m'] is None
    with pytest.raises(ss.SceneError, match='probe_needs_the_direct_runner_on_w0'):
        ss.SceneSpec.from_dict(dict(name='probe_two', runner='harness',
                                    controller=dict(kind='probe_pure_pursuit', params=params)))
    with pytest.raises(ss.SceneError, match='exactly_one_of_throttle_pct_and_speed_m_s'):
        ss.SceneSpec.from_dict(dict(name='probe_three', controller=dict(kind='probe_pure_pursuit', params={})))
    with pytest.raises(ss.SceneError, match='controller_kind_unknown'):
        ss.SceneSpec.from_dict(dict(name='probe_four', controller=dict(kind='stanley', params={})))
    with pytest.raises(ss.SceneError, match='exactly_one_of_maneuver_and_controller'):
        ss.SceneSpec.from_dict(dict(name='both', maneuver=CIRCLE,
                                    controller=dict(kind='probe_pure_pursuit', params=params)))


def test_list_file_round_trip_and_unique_names():
    scenes = [ss.scene('one', maneuver=CIRCLE), ss.scene('two', maneuver=CIRCLE, seeds=[4, 5])]
    raw = ss.dump_scenes(scenes)
    assert ss.load_scenes(raw) == scenes
    assert ss.load_scenes(raw.decode('ascii')) == scenes
    assert ss.load_scenes(json.dumps([s.to_dict() for s in scenes])) == scenes
    with pytest.raises(ss.SceneError, match='names_repeated'):
        ss.load_scenes(json.dumps([scenes[0].to_dict(), scenes[0].to_dict()]))
    with pytest.raises(ss.SceneError, match='needs_a_list'):
        ss.load_scenes('{}')


def test_with_changes_revalidates_and_follows_the_name():
    scene = ss.scene('base_one', maneuver=CIRCLE)
    renamed = scene.with_changes(name='base_two')
    assert renamed.name == 'base_two' and renamed.stream_key == 'base_two'
    keyed = ss.scene('base_three', maneuver=CIRCLE, stream_key='shared')
    assert keyed.with_changes(name='base_four').stream_key == 'shared'
    with pytest.raises(ss.SceneError):
        scene.with_changes(duration_s=-1.)
    with pytest.raises(ss.SceneError, match='scene_key_unknown'):
        scene.with_changes(colour='red')


def test_maneuver_schema_and_scene_schema_agree():
    for kind in mv.SCHEMAS:
        params = dict(speed_m_s=.3) if kind in mv.DRIVING else {}
        if kind == 'schedule':                          # P7: the A12 time script needs its knots
            params = dict(knots=[[0., 30., 0.], [1., 30., 2.]])
        scene = ss.SceneSpec.from_dict(dict(name='k_' + kind, maneuver=dict(kind=kind, params=params)))
        assert set(scene.maneuver.params) == set(mv.SCHEMAS[kind])
