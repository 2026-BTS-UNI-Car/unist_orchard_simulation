"""Vehicle profile loader (schema ubts.vehicle_profile.v2, design 3.8): the shipped placeholder
profile, every rejection rule (each with a failing file written to tmp_path), derived values,
per-run draws and scenario overrides. Nothing is written inside the work tree.
"""
import copy
import datetime
import hashlib
import itertools
import json
import math
from pathlib import Path
import re

import pytest
import yaml

from steering import backend_api, geometry
from steering import vehicle_profile as vp
from steering.vehicle_profile import ProfileError

DEFAULT = vp.DEFAULT_PROFILE_PATH


def base_doc():
    return yaml.safe_load(DEFAULT.read_bytes())


def write(tmp_path, doc, name='profile.yaml'):
    path = tmp_path / name
    path.write_bytes(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False).encode('utf-8'))
    return path


def rejected(tmp_path, doc, code):
    path = write(tmp_path, doc)
    with pytest.raises(ProfileError) as info:
        vp.load_profile(path)
    assert info.value.code == code, str(info.value)
    return info.value


@pytest.fixture(scope='module')
def profile():
    return vp.load_profile()


# --- the shipped profile ----------------------------------------------------------------------------------------------
def test_default_profile_loads(profile):
    assert profile.profile_id == vp.DEFAULT_PROFILE_ID == DEFAULT.stem
    assert profile.schema == vp.SCHEMA
    assert profile.sha256 == hashlib.sha256(DEFAULT.read_bytes()).hexdigest()
    assert profile.names() == list(vp.PARAMETER_NAMES)
    assert len(set(vp.PARAMETER_NAMES)) == len(vp.PARAMETER_NAMES)
    for name in profile.names():
        record = profile.param(name)
        for field in ('value', 'range', 'unit', 'provenance', 'identify', 'stale_after_mod'):
            assert field in record, (name, field)
        vp.parse_provenance(record['provenance'])
        if vp._source_required(record['provenance']):
            assert record.get('source'), name
    label = profile.result_label()
    assert label['result_kind'] == vp.RESULT_LABEL_PLACEHOLDER
    assert label['provenance_grade'] == 'placeholder' and label['field_validated'] is False
    assert label['profile_sha256'] == profile.sha256 and label['overrides'] is None


def test_default_file_is_lf_utf8_without_bom():
    data = DEFAULT.read_bytes()
    assert b'\r\n' not in data and not data.startswith(b'\xef\xbb\xbf')
    data.decode('utf-8')


def test_round_trip_through_safe_dump_loads(tmp_path):
    loaded = vp.load_profile(write(tmp_path, base_doc()))
    assert loaded.names() == list(vp.PARAMETER_NAMES)
    assert loaded.value('steer_mech_limit_deg') == vp.load_profile().value('steer_mech_limit_deg')


def test_parse_profile_from_bytes_matches_file(profile):
    again = vp.parse_profile(DEFAULT.read_bytes(), origin='bytes')
    assert again.sha256 == profile.sha256
    assert again.provenance_table() == profile.provenance_table()


def test_derived_values(profile):
    assert profile.value('scrub_radius_m') == pytest.approx(.05, abs=1e-12)
    assert profile.value('steer_soft_limit_eq_left_deg') == pytest.approx(23.457, abs=.001)
    assert profile.value('steer_soft_limit_eq_right_deg') == pytest.approx(23.457, abs=.001)
    assert profile.value('pot_lsb_deg') == pytest.approx(270. / 1024.)
    assert profile.value('wheel_inertia_kgm2') == pytest.approx(.0147)
    assert profile.value('drive_omega0_rad_s') == pytest.approx(6.695, abs=.001)
    assert profile.value('drive_deadband_frac') == pytest.approx(.0483, abs=.0001)
    sweep = profile.value('steer_sweep_m')
    assert set(sweep) == set(geometry.WHEEL_LIMIT_KEYS)
    assert all(value == pytest.approx(.0429, abs=.0001) for value in sweep.values())
    for name in vp.DERIVATIONS:
        if profile.has(name):   # the schema v3 derivations are absent from this v2 file
            assert profile.is_derived(name) and profile.provenance(name) == 'DERIVED'


def test_spec_pending_list(profile):
    pending = set(profile.spec_pending())
    assert {'wheelbase_m', 'track_front_m', 'steer_ackermann_ratio', 'steer_mech_limit_deg', 'steer_rate_max_deg_s',
            'steer_backlash_deg', 'steer_zero_offset_deg', 'pot_location', 'pot_range_deg', 'drive_tau_stall_nm',
            'front_bumper_x_in_base_m', 'steer_type', 'drive_wheels', 'imu_mount_rpy_deg', 'steer_gear_ratio'} <= pending
    assert not {'servo_hz', 'lidar_yaw_offset_deg', 'mass_kg', 'steer_soft_limit_eq_left_deg',
                'steer_soft_margin_deg', 'lidar_z_m'} & pending
    assert all(profile.provenance(name) == 'SPEC_PENDING' for name in pending)
    assert len(pending) == profile.provenance_counts()['SPEC_PENDING']


def test_stale_after_mod_list(profile):
    stale = set(profile.stale_after_mod())
    assert {'front_bumper_x_in_base_m', 'footprint_length_m', 'footprint_width_m', 'lidar_yaw_offset_deg',
            'self_reflection_mask', 'forward_curve', 'drive_curve_P_fit1003', 'curvature_wander', 'stop_release',
            'mcu_ramp', 'imu_vibration_drive_at_100_m_s2', 'drive_omega0_rad_s', 'drive_deadband_frac'} <= stale
    assert not {'wheelbase_m', 'servo_hz', 'scrub_radius_m', 'steer_soft_limit_eq_left_deg', 'pot_lsb_deg',
                'stop_brake', 'steer_mech_limit_deg'} & stale
    # steer_sweep_m is measured beyond the (pre-modification) body width, so it is stale through that input
    assert 'steer_sweep_m' in stale


def test_undecided_values_are_null(profile):
    for name in ('steer_gear_ratio', 'steer_motor_stall_current_a'):
        assert profile.value(name) is None and profile.param(name)['range'] is None
        assert profile.provenance(name) == 'SPEC_PENDING'


def test_value_returns_copies(profile):
    limits = profile.value('steer_mech_limit_deg')
    limits['left_wheel_toward_left'] = 99.
    assert profile.value('steer_mech_limit_deg')['left_wheel_toward_left'] == 28.
    with pytest.raises(KeyError):
        profile.value('wheelbase_mm')


def test_provenance_table_rows(profile):
    rows = profile.provenance_table()
    assert [row['name'] for row in rows] == list(vp.PARAMETER_NAMES)
    json.dumps(rows, ensure_ascii=False)
    by_name = {row['name']: row for row in rows}
    assert by_name['scrub_radius_m']['derived_from'] == ['track_front_m', 'kingpin_width_m', 'kingpin_inclination_deg',
                                                         'wheel_radius_front_m']
    assert by_name['host_status_gap_ms']['where'] == 'host' and by_name['stall_detect']['where'] == 'mcu'
    assert sum(profile.provenance_counts().values()) == len(rows)


def test_profile_name_lists_exist():
    names = set(vp.PARAMETER_NAMES)
    assert set(vp.PLANT_PROFILE_NAMES) <= names and set(geometry.PROFILE_NAMES) <= names
    for group in (backend_api.GEOMETRY_PROFILE_NAMES, backend_api.ACTUATOR_PROFILE_NAMES,
                  backend_api.POT_PROFILE_NAMES, backend_api.TEST_SERVO_PROFILE_NAMES,
                  backend_api.MCU_PROFILE_NAMES, backend_api.PER_RUN_DRAW_NAMES):
        assert set(group) <= names
    assert set(vp.IDEAL_DRAWS) <= names
    assert set(vp.DERIVATIONS) <= set(vp.PARAMETER_NAMES_V3) and set(backend_api.EXT_PROFILE_NAMES) <= set(vp.PARAMETER_NAMES_V3)
    assert set(vp.PARAMETER_NAMES) < set(vp.PARAMETER_NAMES_V3)


# --- rejection rules (each builds a failing file in tmp_path) ------------------------------------------------------------
def _set(name, **fields):
    def mutate(doc):
        doc['parameters'][name].update(fields)
    return mutate


def _drop(name, field):
    def mutate(doc):
        del doc['parameters'][name][field]
    return mutate


def _set_key(name, key, value):
    def mutate(doc):
        doc['parameters'][name]['value'][key] = value
    return mutate


def _top(key, value=None, delete=False):
    def mutate(doc):
        if delete:
            del doc[key]
        else:
            doc[key] = value
    return mutate


def _derive(name):
    def mutate(doc):
        doc['parameters'][name] = dict(unit='kg', provenance='DERIVED', derived_from=['wheel_mass_kg'], identify=['B1'])
    return mutate


def _replace(name, **record):
    def mutate(doc):
        doc['parameters'][name] = dict(record)
    return mutate


def _extra_parameter(doc):
    doc['parameters']['wheelbase_mm'] = copy.deepcopy(doc['parameters']['wheelbase_m'])


def _missing_parameter(doc):
    del doc['parameters']['wheelbase_m']


REJECTIONS = [
    ('unknown_tag', _set('mass_kg', provenance='GUESSED'), 'provenance_unknown'),
    ('lower_case_tag', _set('mass_kg', provenance='sim_assumed'), 'provenance_unknown'),
    ('measured_month_13', _set('mass_kg', provenance='MEASURED_20261310', source='scale'), 'provenance_bad_date'),
    ('measured_feb_30', _set('mass_kg', provenance='MEASURED_20260230', source='scale'), 'provenance_bad_date'),
    ('measured_year_1999', _set('mass_kg', provenance='MEASURED_19991108', source='scale'), 'provenance_bad_date'),
    ('measured_7_digits', _set('mass_kg', provenance='MEASURED_2026110', source='scale'), 'provenance_unknown'),
    ('measured_bad_suffix', _set('mass_kg', provenance='MEASURED_20261108_NEW_DRIVETRAIN', source='scale'),
     'provenance_unknown'),
    ('measured_without_source', _set('mass_kg', provenance='MEASURED_20261108'), 'source_missing'),
    ('old_drivetrain_without_source', _drop('drive_curve_field1010_measured', 'source'), 'source_missing'),
    ('estimated_without_source', _drop('lidar_yaw_offset_deg', 'source'), 'source_missing'),
    ('definition_without_source', _drop('forward_curve', 'source'), 'source_missing'),
    ('empty_source', _set('lidar_yaw_offset_deg', source='  '), 'source_missing'),
    ('missing_value', _drop('wheelbase_m', 'value'), 'field_missing'),
    ('missing_range', _drop('wheelbase_m', 'range'), 'field_missing'),
    ('missing_unit', _drop('wheelbase_m', 'unit'), 'field_missing'),
    ('missing_provenance', _drop('wheelbase_m', 'provenance'), 'field_missing'),
    ('missing_identify', _drop('wheelbase_m', 'identify'), 'field_missing'),
    ('missing_stale', _drop('wheelbase_m', 'stale_after_mod'), 'field_missing'),
    ('unknown_record_key', _set('wheelbase_m', comment='x'), 'field_unknown'),
    ('unknown_top_level_key', _top('owner', 'x'), 'top_level_key_unknown'),
    ('missing_top_level_key', _top('profile_id', delete=True), 'top_level_key_missing'),
    ('wrong_schema', _top('schema', 'ubts.vehicle_profile.v1'), 'schema_mismatch'),
    ('bad_profile_id', _top('profile_id', 'Front Steer'), 'bad_profile_id'),
    ('unknown_parameter', _extra_parameter, 'parameter_unknown'),
    ('missing_parameter', _missing_parameter, 'parameter_missing'),
    ('scalar_out_of_range', _set('wheelbase_m', value=1.2), 'out_of_range'),
    ('table_leaf_out_of_range', _set_key('steer_mech_limit_deg', 'left_wheel_toward_left', 40.), 'out_of_range'),
    ('list_leaf_out_of_range', _set('steer_link_table', value=[[-40., -40.], [40., 100.]]), 'out_of_range'),
    ('per_key_range', _set_key('imu_pose_in_base', 'z', .70), 'out_of_range'),
    ('uniform_outside_range', _set('steering_efficiency', value={'dist': 'uniform', 'low': .85, 'high': 1.2}),
     'out_of_range'),
    ('normal_mean_outside_range', _set('steer_zero_offset_deg', value={'dist': 'normal', 'mean': 3., 'sd': .7}),
     'out_of_range'),
    ('bad_distribution_keys', _set('steering_efficiency', value={'dist': 'uniform', 'low': .9}), 'bad_distribution'),
    ('unknown_distribution', _set('steering_efficiency', value={'dist': 'beta', 'low': .9, 'high': 1.}),
     'bad_distribution'),
    ('negative_sd', _set('steer_zero_offset_deg', value={'dist': 'normal', 'mean': 0., 'sd': -.1}), 'bad_distribution'),
    ('choice_not_allowed', _set('steer_type', value='rack'), 'not_a_choice'),
    ('bool_choice_as_int', _set('steer_self_locking', value=0), 'not_a_choice'),
    ('bool_where_number', _set('wheelbase_m', value=True), 'bad_value'),
    ('nan_value', _set('wheelbase_m', value=float('nan')), 'bad_value'),
    ('string_where_number', _set('wheelbase_m', value='0.76'), 'bad_value'),
    ('float_where_integer', _set('baud', value=115200.0), 'not_integer'),
    ('float_integer_leaf', _set_key('pot_valid_raw', 'min_raw', 20.0), 'not_integer'),
    ('range_lo_above_hi', _set('wheelbase_m', range=[0.90, 0.65]), 'bad_range'),
    ('range_keys_mismatch', _set('imu_pose_in_base', value={'x': .18, 'y': 0.}), 'range_keys_mismatch'),
    ('range_shape_mismatch', _set('gz_body_box_m', range=[[.1, 2.], [.1, 2.]]), 'range_shape_mismatch'),
    ('empty_list_value', _set('gz_body_box_m', value=[]), 'bad_value'),
    ('nested_empty_row', _set('steer_link_table', value=[[-40., -40.], []]), 'bad_value'),
    ('empty_choices', _set('steer_type', range={'choices': []}), 'bad_range'),
    ('stale_not_bool', _set('wheelbase_m', stale_after_mod=1), 'bad_field'),
    ('unit_empty', _set('wheelbase_m', unit=''), 'bad_field'),
    ('where_unknown', _set('stall_detect', where='cloud'), 'bad_field'),
    ('identify_unknown_token', _set('wheelbase_m', identify=['B99']), 'identify_unknown'),
    ('identify_none_with_test', _set('wheelbase_m', identify=['none', 'B1']), 'identify_unknown'),
    ('identify_empty', _set('wheelbase_m', identify=[]), 'bad_field'),
    ('derived_without_derived_from', _drop('scrub_radius_m', 'derived_from'), 'derived_from_missing'),
    ('derived_from_mismatch', _set('scrub_radius_m', derived_from=['track_front_m', 'kingpin_width_m']),
     'derived_from_mismatch'),
    ('derived_from_unknown_name', _set('scrub_radius_m', derived_from=['track_front_mm']), 'bad_derived_from'),
    # design 3.8.1: DERIVED values are not written in the file (RP-03: written values passed a digit-count check)
    ('derived_value_written', _set('steer_soft_limit_eq_left_deg', value=25.0), 'derived_value_written'),
    ('derived_value_written_coarse', _set('pot_lsb_deg', value=0.30), 'derived_value_written'),
    ('derived_value_written_zero', _set('drive_deadband_frac', value=0.0), 'derived_value_written'),
    ('derived_value_written_equal', _set('steer_soft_limit_eq_left_deg', value=23.457067668806005),
     'derived_value_written'),
    ('derived_table_written', _set('steer_sweep_m', value={k: .043 for k in geometry.WHEEL_LIMIT_KEYS}),
     'derived_value_written'),
    # RP-05: a computed quantity cannot be given another tag and a free value
    ('derived_name_as_config', _replace('pot_lsb_deg', value=0.1, range=[0.04, 0.5], unit='deg', provenance='CONFIG',
                                        identify=['B3'], stale_after_mod=False), 'derived_tag_required'),
    ('derived_name_as_measured', _replace('wheel_inertia_kgm2', value=0.5, range=[0.0, 1.0], unit='kg*m^2',
                                          provenance='MEASURED_20261108', source='bench', identify=['part'],
                                          stale_after_mod=False), 'derived_tag_required'),
    ('derived_stale_mismatch', _set('scrub_radius_m', stale_after_mod=True), 'derived_stale_mismatch'),
    ('derived_outside_written_range', _set('scrub_radius_m', range=[.06, .10]), 'out_of_range'),
    ('derived_from_on_non_derived', _set('wheelbase_m', derived_from=['track_front_m']), 'derived_from_not_allowed'),
    ('derived_without_derivation', _derive('mass_kg'), 'derivation_unknown'),
    ('null_value_not_spec_pending', _set('mass_kg', value=None, range=None), 'null_not_allowed'),
    ('null_value_with_range', _set('steer_gear_ratio', range=[1., 100.]), 'null_not_allowed'),
    ('cal_window_outside_valid_raw', _set_key('cal_end_raw_window', 'min_raw', 10), 'bad_value'),
    ('link_table_not_increasing', _set('steer_link_table', value=[[-40., -40.], [40., -50.]]), 'bad_value'),
    ('curve_roles_equal', _set_key('forward_curve', 'bracket', 'P_fit1003'), 'bad_value'),
    ('mech_limit_keys', _set('steer_mech_limit_deg', value={'left': 28., 'right': 28.}), 'bad_shape'),
    ('servo_hz_not_a_choice', _set('servo_hz', value=600), 'not_a_choice'),
    ('servo_hz_not_dividing_tick', _set('servo_hz', value=600, range={'choices': [500, 600, 1000]}), 'bad_value'),
    # RP-04: values must have the parameter's shape (a numeric range applies to every leaf, so it cannot tell)
    ('scalar_as_list', _set('steer_rate_max_deg_s', value=[30.0, 40.0]), 'bad_shape'),
    ('scalar_as_distribution', _set('steer_rate_max_deg_s', value={'dist': 'uniform', 'low': 20., 'high': 40.}),
     'bad_shape'),
    ('scalar_as_mapping', _set('mass_kg', value={'a': 24.}), 'bad_shape'),
    ('choice_as_list', _set('steer_type', value=['linkage']), 'bad_shape'),
    ('bool_as_list', _set('steer_self_locking', value=[False]), 'bad_shape'),
    ('strings_as_string', _set('pot_fault_injections', value='open'), 'bad_shape'),
    ('list_wrong_length', _set('gz_body_box_m', value=[1.10, 0.54]), 'bad_shape'),
    ('mapping_leaf_as_list', _set_key('steer_backlash_wander', 'sd_deg', [0.1, 0.2]), 'bad_shape'),
    ('null_for_a_number', _set('wheelbase_m', value=None, range=None), 'bad_shape'),
    # F5: design 3.3 L1, a standing car steers slower and needs more PWM
    ('standstill_rate_above_moving', _set('steer_rate_max_deg_s', value=12.0), 'standstill_order'),
    ('standstill_min_pwm_below_moving', _set('steer_min_pwm_pct', value=40.0), 'standstill_order'),
]


@pytest.mark.parametrize('label, mutate, code', REJECTIONS, ids=[r[0] for r in REJECTIONS])
def test_rejections(tmp_path, label, mutate, code):
    doc = base_doc()
    mutate(doc)
    rejected(tmp_path, doc, code)


def test_duplicate_parameter_rejected(tmp_path):
    text = DEFAULT.read_bytes().decode('utf-8')
    text += ('  wheelbase_m: {value: 0.70, range: [0.65, 0.90], unit: "m", provenance: SPEC_PENDING, '
             'identify: [B1], stale_after_mod: false}\n')
    path = tmp_path / 'duplicate.yaml'
    path.write_bytes(text.encode('utf-8'))
    with pytest.raises(ProfileError) as info:
        vp.load_profile(path)
    assert info.value.code == 'duplicate_key'


def test_yaml_syntax_error_rejected(tmp_path):
    path = tmp_path / 'broken.yaml'
    path.write_bytes(b'schema: [unclosed\n')
    with pytest.raises(ProfileError) as info:
        vp.load_profile(path)
    assert info.value.code == 'yaml_error'


def test_top_level_must_be_mapping():
    with pytest.raises(ProfileError) as info:
        vp.parse_profile(b'- a\n- b\n')
    assert info.value.code == 'bad_document'


# --- accepted variants -------------------------------------------------------------------------------------------------
def test_measured_value_accepted(tmp_path):
    doc = base_doc()
    doc['parameters']['wheelbase_m'].update(value=.81, provenance='MEASURED_20261108', source='B1 tape measure')
    doc['parameters']['drive_tau_stall_nm'].update(provenance='MEASURED_20261108_OLD_DRIVETRAIN', source='bench')
    loaded = vp.load_profile(write(tmp_path, doc))
    assert loaded.value('wheelbase_m') == .81 and loaded.provenance('wheelbase_m') == 'MEASURED_20261108'
    # nothing derived is written, so a new wheelbase simply gives new soft limits
    assert loaded.value('steer_soft_limit_eq_left_deg') != vp.load_profile().value('steer_soft_limit_eq_left_deg')
    assert 'wheelbase_m' not in loaded.spec_pending()
    assert vp.parse_provenance('MEASURED_20261108') == ('MEASURED', datetime.date(2026, 11, 8))


def test_derived_values_are_not_written_and_follow_inputs(tmp_path):
    doc = base_doc()
    assert all('value' not in doc['parameters'][name] for name in vp.DERIVATIONS if name in doc['parameters'])
    doc['parameters']['steer_ackermann_ratio']['value'] = 1.
    loaded = vp.load_profile(write(tmp_path, doc))
    assert loaded.value('steer_soft_limit_eq_left_deg') == pytest.approx(22.227, abs=.001)
    assert loaded.param('steer_soft_limit_eq_left_deg')['stale_after_mod'] is False


def test_identify_may_be_a_string(tmp_path):
    doc = base_doc()
    doc['parameters']['wheelbase_m']['identify'] = 'B1'
    assert vp.load_profile(write(tmp_path, doc)).param('wheelbase_m')['identify'] == ['B1']


# --- per-run draws -----------------------------------------------------------------------------------------------------
def test_sampled_names(profile):
    assert profile.sampled_names() == ['steer_zero_offset_deg', 'pot_cal_residual_deg', 'steering_efficiency',
                                       'scrub_speed_loss_per_deg', 'understeer_gradient']


def test_sample_is_deterministic_and_in_range(profile):
    first, again, other = profile.sample(0), profile.sample(0), profile.sample(1)
    assert first == again and first != other
    assert profile.sample(0, ['steering_efficiency']) == {'steering_efficiency': first['steering_efficiency']}
    for seed in range(200):
        draws = profile.sample(seed)
        assert .85 <= draws['steering_efficiency'] <= 1.
        assert 0. <= draws['scrub_speed_loss_per_deg'] <= .006
        assert 0. <= draws['understeer_gradient'] <= .03
        assert -2. <= draws['steer_zero_offset_deg'] <= 2.
        assert -1.5 <= draws['pot_cal_residual_deg'] <= 1.5


def test_sample_spread_roughly_matches_distribution(profile):
    values = [profile.sample(seed, ['steer_zero_offset_deg'])['steer_zero_offset_deg'] for seed in range(2000)]
    mean = sum(values) / len(values)
    sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (len(values) - 1))
    assert abs(mean) < .05 and .6 < sd < .75  # N(0, 0.7) cut at +-2 has sd ~0.70
    eta = [profile.sample(seed, ['steering_efficiency'])['steering_efficiency'] for seed in range(2000)]
    assert sum(eta) / len(eta) == pytest.approx(.925, abs=.005)


def test_sample_errors(profile):
    for seed in (-1, 1.5, True, '1'):
        with pytest.raises(ValueError):
            profile.sample(seed)
    with pytest.raises(ValueError):
        profile.sample(0, ['wheelbase_m'])


def test_ideal_draws_and_nominal(profile):
    assert profile.ideal_draws() == {'steer_zero_offset_deg': 0., 'pot_cal_residual_deg': 0., 'steering_efficiency': 1.,
                                     'scrub_speed_loss_per_deg': 0., 'understeer_gradient': 0.}
    assert profile.nominal('steering_efficiency') == pytest.approx(.925)
    assert profile.nominal('steer_zero_offset_deg') == 0. and profile.nominal('wheelbase_m') == .76
    with pytest.raises(ValueError):
        profile.nominal('steer_type')


# --- scenario overrides ---------------------------------------------------------------------------------------------
def test_override_recomputes_derived_values(profile):
    changed = profile.with_overrides({'steer_ackermann_ratio': 1.}, label='ideal_ackermann')
    assert changed.value('steer_soft_limit_eq_left_deg') == pytest.approx(22.227, abs=.001)
    assert profile.value('steer_soft_limit_eq_left_deg') == pytest.approx(23.457, abs=.001)
    assert changed.sha256 == profile.sha256 and changed.profile_id == profile.profile_id
    label = changed.result_label()['overrides']
    assert label['names'] == ['steer_ackermann_ratio'] and label['out_of_range'] == [] and len(label['sha256']) == 64
    row = {r['name']: r for r in changed.provenance_table()}['steer_ackermann_ratio']
    assert row['override'] == 'ideal_ackermann' and row['override_out_of_range'] is False


def test_override_bounds_and_rules(profile):
    with pytest.raises(ProfileError) as info:
        profile.with_overrides({'steer_backlash_deg': 8.}, label='x2')
    assert info.value.code == 'out_of_range'
    stressed = profile.with_overrides({'steer_backlash_deg': 8.}, label='stress', allow_out_of_range=True)
    assert stressed.value('steer_backlash_deg') == 8.
    assert stressed.result_label()['overrides']['out_of_range'] == ['steer_backlash_deg']
    for overrides, code in (({'scrub_radius_m': .04}, 'override_derived'), ({'wheelbase_mm': .8}, 'parameter_unknown'),
                            ({'steer_gear_ratio': 30.}, 'null_not_allowed')):
        with pytest.raises(ProfileError) as info:
            profile.with_overrides(overrides, label='bad')
        assert info.value.code == code
    with pytest.raises(ProfileError) as info:
        profile.with_overrides({'steer_type': 'rack'}, label='bad', allow_out_of_range=True)
    assert info.value.code == 'not_a_choice'
    with pytest.raises(ValueError):
        profile.with_overrides({'steer_backlash_deg': 1.}, label='')


def test_override_pushing_a_derived_value_out_of_range_is_reported(profile):
    wide = profile.with_overrides({'track_front_m': .70, 'kingpin_width_m': .40}, label='wide', allow_out_of_range=True)
    assert wide.value('scrub_radius_m') == pytest.approx(.15)
    assert set(wide.result_label()['overrides']['out_of_range']) == {'track_front_m', 'scrub_radius_m'}
    # inputs inside their ranges never push a DERIVED value out of its range (RP-06: this corner was refused)
    corner = profile.with_overrides({'track_front_m': .65, 'kingpin_width_m': .40}, label='corner')
    assert corner.value('scrub_radius_m') == pytest.approx(.125)
    assert corner.result_label()['overrides']['out_of_range'] == []


# --- documentation consistency ------------------------------------------------------------------------------------------
def test_spec_request_lists_every_spec_pending_name(profile):
    doc = Path(vp.__file__).resolve().parents[3] / 'docs' / 'steering' / 'VEHICLE_SPEC_REQUEST_KO.md'
    if not doc.is_file():
        pytest.skip('VEHICLE_SPEC_REQUEST_KO.md is not next to this checkout')
    text = doc.read_bytes().decode('utf-8')
    missing = [name for name in profile.spec_pending() if '`%s`' % name not in text]
    assert not missing, missing


def test_spec_request_lists_every_new_placeholder(profile):
    """RP-08: every value the profile marks "(새 자리값)" (no number in the design table) is listed in section 3.1."""
    doc = Path(vp.__file__).resolve().parents[3] / 'docs' / 'steering' / 'VEHICLE_SPEC_REQUEST_KO.md'
    if not doc.is_file():
        pytest.skip('VEHICLE_SPEC_REQUEST_KO.md is not next to this checkout')
    text = doc.read_bytes().decode('utf-8')
    match = re.search(r'\n### 3\.1 [^\n]*\n(.*?)(?=\n## |\n### |\Z)', text, re.S)
    assert match, 'section 3.1 (new placeholder values) missing'
    section = match.group(1)
    marked = [row['name'] for row in profile.provenance_table() if '(새 자리값)' in (row['note'] or '')]
    assert len(marked) >= 10
    missing = [name for name in marked if '`%s`' % name not in section]
    assert not missing, missing


# --- value shapes (RP-04) -------------------------------------------------------------------------------------------
def test_shapes_cover_the_parameters(profile):
    assert set(vp.SHAPES) <= set(vp.PARAMETER_NAMES_V5)   # schema v4 (2026-11-16) adds group K, v5 (2026-11-17) L
    assert set(vp.PARAMETER_NAMES_V4) < set(vp.PARAMETER_NAMES_V5)
    assert not set(vp.SHAPES) & set(vp.DERIVATIONS)
    for name in profile.names():
        if not profile.is_derived(name) and profile.value(name) is not None:
            vp._check_shape(name, profile.value(name), vp.shape_of(name))   # the shipped values have their shape
    per_run = {name for name, shape in vp.SHAPES.items() if shape == vp.PER_RUN}
    assert per_run == set(profile.sampled_names()) == set(vp.IDEAL_DRAWS)


@pytest.mark.parametrize('overrides', [{'steer_rate_max_deg_s': [30., 40.]},
                                       {'steer_rate_max_deg_s': {'dist': 'uniform', 'low': 20., 'high': 40.}},
                                       {'mass_kg': {'a': 24.}}, {'steer_type': ['linkage']},
                                       {'steer_backlash_wander': {'sd_deg': [.1, .2], 'corr_length_m': 5.}},
                                       {'steer_self_locking': [True]}])
def test_overrides_keep_the_shape(profile, overrides):
    with pytest.raises(ProfileError) as info:
        profile.with_overrides(overrides, label='bad_shape')
    assert info.value.code == 'bad_shape', str(info.value)


def test_per_run_values_take_numbers_and_distributions(profile):
    """Fixed per-run values (design 5.3 steer_zero_p15, pot_offset_m2) work like zero-width distributions."""
    fixed = profile.with_overrides({'steer_zero_offset_deg': 1.5}, label='steer_zero_p15')
    fixed = fixed.with_overrides({'pot_cal_residual_deg': -2.0}, label='pot_offset_m2', allow_out_of_range=True)
    for seed in (0, 1, 12345):
        assert fixed.per_run_value(seed, 'steer_zero_offset_deg') == 1.5
        assert fixed.per_run_value(seed, 'pot_cal_residual_deg') == -2.0
        assert profile.per_run_value(seed, 'steer_zero_offset_deg') == profile.sample(seed)['steer_zero_offset_deg']
    assert 'steer_zero_offset_deg' not in fixed.sampled_names()
    assert fixed.result_label()['overrides']['out_of_range'] == ['pot_cal_residual_deg']
    with pytest.raises(ValueError):
        fixed.per_run_value(-1, 'steer_zero_offset_deg')
    with pytest.raises(ValueError):
        profile.per_run_value(0, 'steer_type')


# --- semantic ordering (F5) -------------------------------------------------------------------------------------------
@pytest.mark.parametrize('overrides', [{'steer_rate_max_deg_s': 10.}, {'steer_rate_standstill_deg_s': 30.1},
                                       {'steer_min_pwm_pct': 50.}, {'steer_min_pwm_standstill_pct': 24.9}])
def test_standstill_values_cannot_invert(profile, overrides):
    with pytest.raises(ProfileError) as info:
        profile.with_overrides(overrides, label='inverted', allow_out_of_range=True)
    assert info.value.code == 'standstill_order', str(info.value)
    # sweeping both together is fine
    both = profile.with_overrides({'steer_rate_max_deg_s': 10., 'steer_rate_standstill_deg_s': 5.,
                                   'steer_min_pwm_pct': 50., 'steer_min_pwm_standstill_pct': 60.}, label='joint')
    assert both.value('steer_rate_standstill_deg_s') <= both.value('steer_rate_max_deg_s')


# --- range consistency (F4, RP-06) -------------------------------------------------------------------------------------
def _interval_or_choices(record):
    rng = record['range']
    if isinstance(rng, list) and len(rng) == 2 and all(isinstance(x, (int, float)) and not isinstance(x, bool)
                                                       for x in rng):
        return list(dict.fromkeys([rng[0], rng[1]]))
    if isinstance(rng, dict) and set(rng) == {'choices'}:
        return list(rng['choices'])
    return None


def test_derived_ranges_cover_every_corner_of_their_inputs(profile):
    """F4 / RP-06: a DERIVED range must hold for every combination of input range ends (the derivations are
    monotone in each input), so no sweep with inputs inside their ranges is refused by a DERIVED range."""
    checked = 0
    for name in vp.DERIVATIONS:
        if not profile.has(name):
            continue
        rng = profile.param(name)['range']
        if rng is None:
            continue
        inputs = vp.DERIVATIONS[name][0]
        ends = []
        for item in inputs:
            values = _interval_or_choices(profile.param(item))
            assert values is not None, (name, item)
            ends.append(values)
        for combination in itertools.product(*ends):
            changed = {item: value for item, value in zip(inputs, combination) if value != profile.value(item)}
            view = profile.with_overrides(changed, label='corner') if changed else profile
            value = view.value(name)
            assert rng[0] <= value <= rng[1], (name, dict(zip(inputs, combination)), value)
            checked += 1
    assert checked == 16 + 4   # scrub_radius_m (4 inputs) and pot_lsb_deg (adc_bits fixed)


JOINT_ORDER = {('steer_rate_max_deg_s', 10.0), ('steer_min_pwm_pct', 50.0)}   # F5: sweep with the standstill value


def test_each_range_end_loads_alone(profile):
    """Every scalar range end and every choice loads with the other values at their defaults, except the two
    documented joint sweeps (standstill value <= / >= moving value, design 3.3 L1)."""
    refused, loaded = [], 0
    for name in profile.names():
        record = profile.param(name)
        if profile.is_derived(name) or record['range'] is None or isinstance(record['value'], (dict, list)):
            continue
        values = _interval_or_choices(record)
        if values is None:
            continue
        for value in values:
            if value == record['value']:
                continue
            try:
                profile.with_overrides({name: value}, label='end')
                loaded += 1
            except ProfileError as exc:
                refused.append((name, value, exc.code))
    assert set((name, value) for name, value, _ in refused) == JOINT_ORDER, refused
    assert all(code == 'standstill_order' for _, _, code in refused) and loaded > 150


def test_servo_hz_choices_divide_every_tick_choice(profile):
    """RP-06: servo_hz is a choice set; each choice works with each MCU tick choice."""
    assert profile.param('servo_hz')['range'] == {'choices': [500, 1000]}
    for servo_hz in (500, 1000):
        for tick in profile.param('sim_mcu_tick_hz')['range']['choices']:
            view = profile.with_overrides({'servo_hz': servo_hz, 'sim_mcu_tick_hz': tick}, label='servo_tick')
            assert view.value('sim_mcu_tick_hz') % view.value('servo_hz') == 0


# --- consistency warnings (F1, RP-06) ------------------------------------------------------------------------------------
def test_shipped_profile_has_no_warnings(profile):
    assert profile.warnings() == [] and profile.result_label()['warnings'] == []
    # F1: the servo minimum PWM is above both start PWMs of the steering motor
    assert profile.value('servo_min_pwm_pct') > profile.value('steer_min_pwm_standstill_pct') >= \
        profile.value('steer_min_pwm_pct')


def test_servo_minimum_below_the_motor_start_is_reported(profile):
    low = profile.with_overrides({'servo_min_pwm_pct': 25.}, label='servo_min_25')
    codes = [(w['code'], w['names'][1]) for w in low.warnings()]
    assert codes == [('servo_min_pwm_cannot_start_the_motor', 'steer_min_pwm_pct'),
                     ('servo_min_pwm_cannot_start_the_motor', 'steer_min_pwm_standstill_pct')]
    assert '1.67 deg' in low.warnings()[0]['detail'] and '2.33 deg' in low.warnings()[1]['detail']
    assert [w['code'] for w in low.result_label()['warnings']] == ['servo_min_pwm_cannot_start_the_motor'] * 2
    sticky = profile.with_overrides({'steer_min_pwm_standstill_pct': 45.}, label='sticky')
    assert [w['names'][1] for w in sticky.warnings()] == ['steer_min_pwm_standstill_pct']
    # a large gain moves the motor just outside the deadband: no artefact
    stiff = low.with_overrides({'servo_kp_pct_per_deg': 40., 'servo_deadband_deg': 1.0}, label='stiff')
    assert stiff.warnings() == []


@pytest.mark.parametrize('ratio, codes', [(4.0, []), (4.5, ['pot_end_stops_outside_cal_window']),
                                          (5.0, ['pot_end_stops_outside_cal_window',
                                                 'pot_end_stops_outside_valid_raw'])])
def test_pot_end_stops_outside_the_calibration_window_are_reported(profile, ratio, codes):
    """RP-06: with the default pot (270 deg, centre 512, motor side, backlash 2 deg) a ratio above about 4.4 puts
    the end-stop raw outside cal_end_raw_window 60..963 (firmware calibration would refuse it)."""
    view = profile.with_overrides({'pot_to_steer_ratio': ratio}, label='pot_ratio')
    assert [w['code'] for w in view.warnings()] == codes
    left, right = vp.pot_end_stop_raws(view.value)
    assert left > 512. > right
    if ratio == 4.5:
        assert left == pytest.approx(974.5, abs=.1)
