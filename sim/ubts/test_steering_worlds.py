"""P7 worlds in the steering SIM: procedural orchard W3, W3 scenes and the A12 plant tables in the fast SIM, the
'schedule' maneuver, the field1010 Gazebo scene preset, the A12 twin probe controller.

No real data needed (the procedural orchard); the field1010 scene of a catalogue corridor skips without the maps.
Plain asserts (pytest on 3.12 / 3.13, a unittest-style runner on 3.10). Temporary files go to the system temp folder.
Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_worlds.py
"""
import hashlib
import json
import math
from pathlib import Path
import tempfile

import numpy as np

from closed_loop import raster_corridor as RC
from field_world import coord_lint
from field_world import raster_ops as ro
from steering import maneuvers as mv
from steering import procedural_orchard as po
from steering import scene_spec as ss


class SkipTest(Exception):
    """Skip outside pytest (the 3.10 runner counts these)."""


def skip(reason):
    try:
        import pytest
    except ImportError:
        raise SkipTest(reason)
    pytest.skip(reason)


def raises(exc_type, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc_type as error:
        return error
    raise AssertionError('%s not raised' % exc_type.__name__)


def has_scipy():
    try:
        import scipy  # noqa: F401
        from scipy.spatial import cKDTree  # noqa: F401
        return True
    except Exception:
        return False


# --- procedural orchard -----------------------------------------------------------------------------------------------
def test_params_defaults_validation_and_holdout_seeds():
    p = po.normalise_params(dict(seed=4))
    assert p['rows'] == 4 and p['corridor_index'] == 1 and p['row_length_m'] == 60.
    assert po.params_sha256(dict(seed=4)) == po.params_sha256(p)
    assert po.params_sha256(dict(seed=5)) != po.params_sha256(p)
    for bad, code in ((dict(seed=-1), 'seed'), (dict(rows=1), 'rows'), (dict(colour=1), 'unknown'),
                      (dict(corridor_index=5), 'corridor_index'), (dict(bend_amplitude_m=[.3, .1]), 'order')):
        assert code in str(raises(po.OrchardError, po.normalise_params, bad))
    assert po.is_holdout_seed(100000) and not po.is_holdout_seed(5)
    # 2026-11-09 review RW-06: a holdout seed is refused unless allow_holdout (the pre-registration step)
    error = raises(po.OrchardError, po.build_raster_world, dict(seed=100000, rows=2, row_length_m=12.))
    assert 'holdout_seed_refused' in str(error)
    assert po.check_seed(dict(seed=100000), allow_holdout=True) == 100000
    # 2026-11-09 review RW-05: the row length range covers the data rows (23.8-106.9 m) and fits every yaw
    assert 'row_length_m' in str(raises(po.OrchardError, po.normalise_params, dict(row_length_m=121.)))


def test_procedural_world_is_deterministic_and_seeded():
    a = po.build_raster_world(dict(seed=1, rows=3, row_length_m=30.))[1]
    b = po.build_raster_world(dict(seed=1, rows=3, row_length_m=30.))[1]
    c = po.build_raster_world(dict(seed=2, rows=3, row_length_m=30.))[1]
    digest = lambda w: hashlib.sha256(w.map.classes.tobytes()).hexdigest()
    assert digest(a) == digest(b) != digest(c)
    assert a.eval_truth == b.eval_truth


def test_procedural_truth_and_corridor_frame():
    config, world, info = po.build_raster_world(dict(seed=3, rows=4, row_length_m=40.))
    assert config.record == 'procedural' and config.end_s_m > 30. and info['units'] > 50
    truth = world.eval_truth
    assert truth['s_start_m'] == 0. and abs(truth['s_end_m'] - config.end_s_m) < 1e-3
    # the corridor end is the later of the two rows' last units (the catalogue fwd rule)
    last = max(max(u[0] for u in config.left_units), max(u[0] for u in config.right_units))
    assert abs(last - config.end_s_m) < .3
    # row centre d profile: left row positive, right row negative, separation ~ the aisle spacing
    left, right = np.asarray(config.left_row_d_m), np.asarray(config.right_row_d_m)
    assert np.all(left > 1.) and np.all(right < -1.)
    assert abs(np.median(left - right) - config.row_width_m) < 1e-6
    assert 3. < config.row_width_m < 4.2
    # the raster: OCCUPIED units inside FREE observed ground, unobserved crop edge
    cls = world.map.classes
    assert (cls == RC.OCCUPIED).sum() > 100 and cls[0, 0] == RC.UNOBSERVED
    x, y, yaw = world.pose_in_field(0., 0., 0.)
    assert int(world.map.class_at(x, y)[0]) == RC.FREE


def test_procedural_units_match_the_development_statistics_roughly():
    config, world, _ = po.build_raster_world(dict(seed=11, rows=4, row_length_m=80.))
    spacing = np.diff(sorted(u[0] for u in config.left_units))
    assert .35 < float(np.median(spacing)) < .9                         # 10/10 development median 0.60 m
    rows, cols = np.nonzero(world.map.classes == RC.OCCUPIED)
    table = ro.component_table(rows, cols, ro.label_components(rows, cols), .05)
    diag = np.median([c['diag_m'] for c in table])
    assert .1 < diag < .45                                               # 10/10 development median 0.21 m


def test_procedural_raster_matches_the_data_raster_statistics():
    # 2026-11-09 review RW-02: the scan plane must show what the data shows (same measuring rules on both): the units
    # are separate components (the old generator merged 18 %), the component density and centroid spacing, the
    # one-cell fraction and the box diagonal (the old ellipse cells were smaller), the offsets with the close pairs
    from field_world import orchard_stats as OS
    out = OS.procedural_comparison(range(6))
    w3, data = out['procedural'], OS.DATA_RASTER_REFERENCE
    assert w3['matched_fraction'] >= .95, w3['matched_fraction']                      # old 0.82
    assert abs(w3['components_per_100m'] / data['components_per_100m'] - 1.) <= .10    # old -19 %
    spacing = w3['comp_spacing_m']
    assert abs(spacing['q10'] - data['comp_spacing_m']['q10']) <= .06                  # old 0.319 vs 0.168
    for q in ('q50', 'q90'):
        assert abs(spacing[q] / data['comp_spacing_m'][q] - 1.) <= .15, (q, spacing[q])  # old q50 +27 %
    assert .10 <= w3['one_cell_fraction'] <= .22, w3['one_cell_fraction']             # old 0.27 (data 0.164)
    for q in ('q25', 'q50', 'q75', 'q90'):
        assert abs(w3['comp_diag_m'][q] / data['comp_diag_m'][q] - 1.) <= .25, (q, w3['comp_diag_m'][q])
    off = w3['unit_offset_inward_m']
    for q, tol in (('q10', .08), ('q25', .03), ('q50', .02), ('q75', .03), ('q90', .08)):
        assert abs(off[q] - data['unit_offset_inward_m'][q]) <= tol, (q, off[q])
    assert abs(w3['close_pair_abs_doffset_m']['q50'] - data['close_pair_abs_doffset_m']['q50']) <= .08


def test_long_rows_fit_the_crop_at_every_yaw():
    # 2026-11-09 review RW-05: a 100 m row at 45 deg was refused (crop over 8.4 M cells); 120 m fits every yaw now
    for yaw in range(0, 181, 15):
        layout = po.Layout(dict(seed=3, rows=2, row_length_m=120., field_yaw_deg=float(yaw)))
        width, height, _ = po.crop_shape(layout)
        assert width * height <= po.MAX_CROP_CELLS, (yaw, width, height)
    config, world, info = po.build_raster_world(dict(seed=5, rows=2, row_length_m=107., field_yaw_deg=45.))
    assert config.end_s_m > 100. and info['crop_cells'] > 8_388_608


def test_procedural_gap_modifier_removes_whole_units():
    plain = po.build_raster_world(dict(seed=5, rows=3, row_length_m=30.))
    gap = po.build_raster_world(dict(seed=5, rows=3, row_length_m=30.),
                                modifiers=[dict(kind='gap', side='left', s_from_m=18., s_to_m=22.)])
    removed = gap[1].modifier_log[0]
    assert removed['removed_cells'] > 0 and all(18. <= s <= 22. for s in removed['removed_unit_s_m'])
    assert (gap[1].map.classes == RC.OCCUPIED).sum() == (plain[1].map.classes == RC.OCCUPIED).sum() - \
        removed['removed_cells']
    wall = po.build_raster_world(dict(seed=5, rows=3, row_length_m=30.), modifiers=[dict(kind='wall', beyond_end_m=3.)])
    assert (wall[1].map.classes == RC.ADDED_HARD).sum() > 0 and wall[1].walls_s


# --- scenes ------------------------------------------------------------------------------------------------------------
def test_w3_scene_spec_and_unchanged_w0_w2_canonical_form():
    scene = ss.scene('w3x', world=dict(kind='W3', procedural=dict(seed=2)), maneuver=dict(
        kind='circle', params=dict(speed_m_s=.3)))
    world = scene.to_dict()['world']
    assert world['procedural']['seed'] == 2 and world['procedural']['rows'] == 4
    assert ss.SceneSpec.from_dict(scene.to_dict()).sha256() == scene.sha256()
    w0 = ss.scene('w0x', maneuver=dict(kind='circle', params=dict(speed_m_s=.3))).to_dict()
    assert 'procedural' not in w0['world'] and 'plant_tables' not in w0['profile']
    for bad, code in ((dict(kind='W3', record='040927'), 'w3_takes'), (dict(kind='W0', procedural={}), 'only_for_w3'),
                      (dict(kind='W3', procedural=dict(rows=99)), 'procedural_params_invalid'),
                      (dict(kind='W3', procedural=dict(seed=100000)), 'holdout_seed'),
                      (dict(kind='W0', allow_holdout=True), 'only_for_w3')):
        assert code in str(raises(ss.SceneError, ss.WorldSpec.from_dict, bad))
    held = ss.WorldSpec.from_dict(dict(kind='W3', procedural=dict(seed=100001), allow_holdout=True))
    assert held.to_dict()['allow_holdout'] is True and 'allow_holdout' not in world


def test_plant_tables_validation():
    good = dict(label='t', eta_by_delta_deg=[[5, .99], [10, .98]], loss_by_delta_deg=[[5, .01, .005]],
                loss_speeds_m_s=[.3, .5])
    out = ss.normalise_plant_tables(good)
    assert out['loss_speeds_m_s'] == [.3, .5] and out['eta_by_delta_deg'][1] == [10., .98]
    for change, code in ((dict(eta_by_delta_deg=[[10, .9], [5, .9]]), 'order'),
                         (dict(loss_speeds_m_s=None), 'rows_and_speeds'),
                         (dict(loss_by_delta_deg=[[5, .01]]), 'one_loss_per_speed'), (dict(label='a b'), 'label')):
        assert code in str(raises(ss.SceneError, ss.normalise_plant_tables, dict(good, **change)))
    assert 'geometry_check' in str(raises(ss.SceneError, ss.ProfileSpec.from_dict,
                                          dict(plant_tables=good, geometry_check=True), 'x'))


def test_schedule_maneuver_values_and_done():
    params = mv.normalise_params('schedule', dict(knots=[[1, 10, 0], [3, 30, 4]]))
    assert params['interpolation'] == 'linear' and params['end_s'] is None
    assert mv.schedule_value(params, 0.) == (10., 0.) and mv.schedule_value(params, 2.) == (20., 2.)
    assert mv.schedule_value(params, 9.) == (30., 4.)
    hold = mv.normalise_params('schedule', dict(knots=[[0, 10, 1], [2, 20, 2]], interpolation='hold', end_s=5))
    assert mv.schedule_value(hold, 1.9) == (10., 1.)
    m = mv.make_maneuver('schedule', hold)
    assert not m.done(4.9, None) and m.done(5., None)
    assert abs(m.command(1., None).steer_target_mrad - 1000. * math.radians(1.)) < 1e-9
    for knots, code in (([[0, 1, 2]], 'knots_need'), ([[0, 1, 2], [0, 1, 2]], 'not_increasing'),
                        ([[0, 1, 2], [1, 120, 0]], 'out_of_range'), ([[0, 1], [1, 1]], 'knot_needs')):
        assert code in str(raises(mv.ManeuverError, mv.normalise_params, 'schedule', dict(knots=knots)))


def test_fast_sim_w3_direct_episode_and_identity():
    from steering import fast_sim as fs
    scene = ss.scene('w3_direct', world=dict(kind='W3', procedural=dict(seed=3, rows=3, row_length_m=30.)),
                     backend='L0', start=dict(x_m=1., y_m=0., yaw_deg=0.), duration_s=6.,
                     maneuver=dict(kind='schedule', params=dict(knots=[[0, 40, 0], [6, 40, 2]])))
    arrays, _, record = fs.run_episode(scene, 0)
    assert record['end']['end_reason'] == 'maneuver_done' and record['world']['kind'] == 'W3'
    assert abs(arrays['truth_field_x_m'][0] - 1.) < .01 and arrays['truth_field_x_m'][-1] > 2.
    identity = fs.world_data_identity(scene.world)
    assert identity == dict(procedural_params_sha256=po.params_sha256(scene.world.procedural))
    assert 'steering/procedural_orchard.py' in fs.IDENTITY_FILES


def test_fast_sim_w3_harness_episode():
    if not has_scipy():
        skip('the harness runner needs scipy (decision R10)')
    from steering import fast_sim as fs
    scene = ss.scene('w3_harness', world=dict(kind='W3', procedural=dict(seed=3, rows=3, row_length_m=30.)),
                     backend='L0', runner='harness', estimator='null', start=dict(x_m=1., y_m=0., yaw_deg=0.),
                     duration_s=3., maneuver=dict(kind='circle', params=dict(speed_m_s=.3, steer_deg=0.)))
    arrays, _, record = fs.run_episode(scene, 0)
    assert record['harness']['world']['source'] == po.SOURCE and record['harness']['world']['generated_truth']
    assert record['harness']['collisions'] == 0


def test_table_plant_curvature_and_speed_loss():
    from steering import fast_sim as fs
    tables = dict(label='t', eta_by_delta_deg=[[10, .9]], loss_by_delta_deg=[[10, .05, .05]], loss_speeds_m_s=[.3, .5])
    overrides = {'steer_zero_offset_deg': {'dist': 'normal', 'mean': 0., 'sd': 0.},
                 'speed_run_scale': {'mode': 1., 'sd_low': 0., 'sd_high': 0.}, 'speed_fluctuation_sd': 0.,
                 'curvature_wander': {'mean_deg_per_m': 0., 'sd_deg_per_m': 0., 'corr_length_m': 8.},
                 'lateral_slip_sd_m_s': 0.}
    scene = ss.scene('table', backend='L0', duration_s=30.,
                     profile=dict(plant_tables=tables, overrides=overrides, allow_out_of_range=True),
                     maneuver=dict(kind='circle', params=dict(throttle_pct=40., steer_deg=10.)))
    arrays, _, _ = fs.run_episode(scene, 0)
    delta = arrays['truth_delta_eq_rad'][-300:]
    kappa = arrays['truth_steer_curvature_1_m'][-300:]
    params = fs.scene_params(scene)
    eta = np.interp(np.degrees(np.abs(delta)), [0., 10.], [1., .9])
    assert np.allclose(kappa, eta * np.tan(delta) / params.wheelbase_m, rtol=1e-9, atol=1e-12)
    from steering import drive
    v = arrays['truth_v_m_s'][-1]
    loss = .05 * min(1., np.degrees(abs(delta[-1])) / 10.)
    assert abs(v - drive.steady_speed_m_s(params.profile, 40.) * (1. - loss)) < 2e-3


# --- Gazebo scene preset (pure Python: assemble_world + gz_model) -------------------------------------------------------
def test_field1010_scene_of_a_procedural_orchard():
    from steering import gz_assemble, gz_model
    spec = dict(name='t_scene_w3', source='procedural', procedural=dict(seed=9, rows=3, row_length_m=20.), mode='R',
                start=dict(s_m=2., d_m=.1, heading_deg=5.))
    with tempfile.TemporaryDirectory() as tmp:
        manifest = gz_assemble.build_scene('field1010', tmp, camera=False, field_spec=spec)
        assert manifest['world'] == 'W3' and manifest['gnss_lint'] == []        # review RW-09 (was 'W2')
        summary = manifest['world_summary']
        assert summary['source'] == 'procedural' and summary['boxes'] > 0 and summary['removed_models'] >= 40
        text = (Path(tmp) / 'world.sdf').read_text(encoding='utf-8')
        assert '<model name="field_ground">' in text and '<model name="ground">' not in text and 'tree_0_0' not in text
        meta = json.loads((Path(tmp) / 'world_metadata.json').read_text(encoding='utf-8'))
        assert meta['schema'] == gz_assemble.FIELD_METADATA_SCHEMA and 'lane_centers_y' not in meta
        assert meta['world'] == 'W3' and gz_assemble.world_label('field1010', dict(source='catalogue')) == 'W2'
        assert (Path(tmp) / 'field_world' / 'raster.npz').is_file()
        report = coord_lint.lint_files([Path(tmp)])
        assert report['ok'], report
        import xml.etree.ElementTree as E
        rover = E.parse(str(Path(tmp) / 'world.sdf')).getroot().find(".//model[@name='%s']" % gz_model.MODEL_NAME)
        pose = [float(v) for v in rover.findtext('pose').split()]
        assert abs(pose[0] - summary['spawn_w']['x_m']) < 1e-3 and abs(pose[5] - summary['spawn_w']['yaw_rad']) < 1e-4
        assert abs(pose[0] - 2.) < .2 and abs(pose[1] - .1) < .2 and abs(math.degrees(pose[5]) - 5.) < 3.
    assert 'field_scene_needs' in str(raises(gz_assemble.SceneError, gz_assemble.assemble_base, 'field1010', tmp))
    assert 'field_spec_only' in str(raises(gz_assemble.SceneError, gz_assemble.assemble_base, 'w0_force', tmp,
                                           field_spec=spec))


# --- A12 twin probe controller (no Gazebo) ------------------------------------------------------------------------------
def test_duty_ramp_equals_the_plant_ramp():
    from steering import gz_probe as gp
    from steering import steered_plant as sp
    rng = np.random.default_rng(2)
    a, b = gp.DutyRamp(100., .12), sp._DriveRamp(100., .12)
    t, dt = 0., 1. / 120.
    for _ in range(3000):
        if rng.uniform() < .02:
            target = float(rng.choice([0., 30., -20., 60.]))
            a.command(target)
            b.command(target)
        t += dt
        assert a.step(t, dt) == b.step(t, dt)


class _FakeBackend:
    def __init__(self):
        self.targets = []

    def reset(self, angle):
        return None

    def step(self, inp):
        self.targets.append(inp.target_delta_eq_rad)
        from steering import backend_api as ba
        return _Out(inp.target_delta_eq_rad)

    def close(self):
        pass


class _Out:
    def __init__(self, delta):
        self.delta_eq_rad = self.delta_left_rad = self.delta_right_rad = self.motor_side_angle_rad = delta
        self.pot_raw, self.applied_steer_pwm = 512, 0.


def test_script_controller_reproduces_the_fast_sim_command_timing():
    from steering import gz_probe as gp
    from steering.vehicle_profile import load_profile
    ctx = gp.Context(load_profile(), 1. / 120.)
    backend = _FakeBackend()
    script = dict(knots=[[0., 40., 0.], [1., 40., 10.]], interpolation='hold', end_s=3.)
    controller = gp.ScriptSteerController(ctx, backend, script)
    for k in range(1, 361):
        controller.command(k, k * ctx.dt, dict(speed=0.))
    targets = np.array(backend.targets)
    start = np.arange(360) * ctx.dt
    # the command of t_i = 0.05 i acts from t_i + 0.05; the 10 deg knot at 1.00 s is first seen by the command at 1.00
    first = np.flatnonzero(targets > 0)[0]
    assert abs(start[first] - 1.05) < ctx.dt + 1e-9 and start[first] >= 1.05 - 1e-9
    assert np.all(targets[start < .1 - 1e-9] == 0.)
    assert abs(targets[-1] - math.radians(10.)) < 1e-12 or targets[-1] < math.radians(10.)
    assert controller.duty > 0.


def test_a12_group_definitions_and_tables():
    from field_world import a12_twin
    from steering import gz_probe as gp
    from steering.vehicle_profile import load_profile
    group = gp.group_definition('a12_w0', load_profile())
    assert group['scene'] == 'w0_force' and len(group['vehicles']) == len(a12_twin.scripts('W0'))
    assert group['profile_overrides'] == a12_twin.A12_OVERRIDES and group['profile_overrides_allow_out_of_range']
    a1 = dict(per_delta_form=dict(eta_by_delta_deg={'5': .99, '10': .98}, max_abs_curvature_rel_residual=1e-5),
              registered_form=dict(eta=.97, understeer_gradient_s2_m=-.01, max_abs_curvature_rel_residual=.02),
              turning_speed_loss=dict(table={'5': dict(by_speed={'0.3': dict(loss=.006), '0.5': dict(loss=.005)}),
                                             '10': dict(by_speed={'0.3': dict(loss=.025), '0.5': dict(loss=.02)})}))
    tables = a12_twin.plant_tables(a1)
    assert tables['eta_by_delta_deg'] == [[5., .99], [10., .98]] and tables['loss_speeds_m_s'] == [.3, .5]
    assert ss.normalise_plant_tables(tables) == tables
    overrides, values = a12_twin.registered_overrides(a1)
    assert values['eta'] == .97 and abs(values['scrub_speed_loss_per_deg'] - (5 * .011 + 10 * .045) / 250.) < 1e-12
    scene = a12_twin.fsim_scene('a12.t', a12_twin.scripts('W0')['s2_steps10_v03'], form='per_delta', a1=a1)
    assert scene.profile.plant_tables['label'] == tables['label'] and scene.maneuver.kind == 'schedule'
    track = (np.linspace(0, 10, 101), np.linspace(0, 5, 101), np.zeros(101), np.zeros(101))
    other = (track[0], track[1] + .05, track[2] + .02, track[3])
    diff = a12_twin.track_difference(track, other)
    assert abs(diff['max_position_m'] - math.hypot(.05, .02)) < 1e-9 and diff['max_heading_deg'] == 0.


def test_field1010_scene_of_a_catalogue_corridor():
    try:
        RC.maps_root()
        RC.catalogue_path()
    except FileNotFoundError as error:
        skip('10/10 maps or the catalogue not configured: %s' % error)
    from steering import gz_assemble
    from field_world.scene_spec import catalogue_spec
    with tempfile.TemporaryDirectory() as tmp:
        manifest = gz_assemble.build_scene('field1010', tmp, camera=False,
                                           field_spec=catalogue_spec('040927', 'c06', 'fwd', mode='T'))
        assert manifest['world_summary']['collision_shapes'] > 0 and manifest['world_summary']['mode'] == 'T'
        assert coord_lint.lint_files([Path(tmp)])['ok']
