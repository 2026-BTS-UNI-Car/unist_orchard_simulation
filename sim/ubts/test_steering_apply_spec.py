"""tools/steer_sim/apply_spec.py: the specification input file -> a new vehicle profile (ST-17 preparation,
docs/steering/SPEC_UPDATE_KO.md). Short checks are not run here (--no-checks); they are rig / fast SIM runs.

Run from sim/ubts: python -B -m pytest -q -p no:cacheprovider test_steering_apply_spec.py
"""
from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import pytest
import yaml

from steering import vehicle_profile as vp

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / 'tools' / 'steer_sim' / 'apply_spec.py'
TEMPLATE = vp.PROFILES_DIR / 'steer_spec_input.yaml'


def _tool():
    spec = importlib.util.spec_from_file_location('apply_spec_under_test', TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


AS = _tool()


def _spec(tmp_path, **fields):
    doc = yaml.safe_load(TEMPLATE.read_bytes())
    for name, (value, source) in fields.items():
        item = doc['fields'][name]
        item.update(value=value, source=source, note='시험 값')
        if source == '측정':
            item['date'] = '20261120'
    doc['new_profile_id'] = 'front_steer_v1_spec_test'
    path = tmp_path / 'spec.yaml'
    path.write_bytes(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False).encode('utf-8'))
    return path


def _run(tmp_path, spec_path):
    out = tmp_path / 'out'
    code = AS.main(['--spec', str(spec_path), '--out', str(out), '--no-checks'])
    report = json.loads((out / 'REPORT.json').read_bytes()) if (out / 'REPORT.json').exists() else None
    return code, out, report


def test_template_fields_match_the_tool_and_the_profile():
    doc = yaml.safe_load(TEMPLATE.read_bytes())
    assert doc['schema'] == AS.SCHEMA
    assert set(doc['fields']) == set(AS.SPEC_FIELDS)
    base = vp.load_profile(vp.PROFILES_DIR / (doc['base_profile'] + '.yaml'))
    for name, item in doc['fields'].items():
        assert item['q'] == AS.SPEC_FIELDS[name] and item['value'] is None
        assert name in AS.INPUT_ONLY or base.has(name), name
        assert vp.IDENTIFY_TEST_PATTERN.match(item['q'])


def test_empty_input_gives_the_base_values(tmp_path):
    code, out, report = _run(tmp_path, TEMPLATE)
    assert code == 0
    base = vp.load_profile(vp.PROFILES_DIR / 'front_steer_v0_3_placeholder.yaml')
    new = vp.load_profile(report['candidate_path'])
    assert all(new.value(n) == base.value(n) for n in base.names())
    assert report['changed'] == [] and report['firmware']['cfg_diff'] == []


def test_teeth_and_measured_values(tmp_path):
    path = _spec(tmp_path, belt_pulley_teeth_pot=(20, '사용자'), belt_pulley_teeth_steer=(30, '사용자'),
                 wheelbase_m=(.8, '측정'), steer_mech_limit_deg=(25, '측정'), steer_motor_stall_torque_nm=(6., '제품표기'))
    code, out, report = _run(tmp_path, path)
    assert code == 0
    new = vp.load_profile(report['candidate_path'])
    assert new.value('pot_to_steer_ratio') == 1.5 and new.provenance('pot_to_steer_ratio') == 'USER_APPROX'
    assert new.value('wheelbase_m') == .8 and new.provenance('wheelbase_m') == 'MEASURED_20261120'
    assert new.param('wheelbase_m')['source'].startswith('Q2 (측정)')
    assert set(new.value('steer_mech_limit_deg').values()) == {25.0}
    assert new.provenance('steer_motor_stall_torque_nm') == 'PRODUCT_LABEL'
    assert new.value('belt_tooth_step_deg') == pytest.approx(12.0)
    assert new.provenance_grade() == 'placeholder'      # other values still wait for the specification
    keys = {d['key'] for d in report['firmware']['cfg_diff']}
    assert {'marks.raw_l', 'marks.raw_r', 'params.trk'} <= keys
    assert report['gazebo']['new']['ok']
    joints = report['gazebo']['new']['value']['steer_joints']
    assert all(abs(j['upper'] - 0.43633) < 1e-4 for j in joints.values())
    assert {r['area'] for r in report['recheck']} == {'geometry', 'actuator', 'pot'}
    assert (out / 'firmware_cfg.txt').read_bytes().decode('utf-8').count('\nCFG ') == 41
    assert json.loads((out / 'host_profile.json').read_bytes())['vehicle']['wheelbase_m'] == .8


def test_adc_end_raws_give_ratio_and_centre(tmp_path):
    path = _spec(tmp_path, pot_adc_end_raws=([620, 500, 380], '측정'))
    code, out, report = _run(tmp_path, path)
    assert code == 0
    new = vp.load_profile(report['candidate_path'])
    # 240 counts = 240 / 1024 x 270 deg = 63.28 deg of pot over 54.19 deg of link travel (backlash included)
    assert new.value('pot_to_steer_ratio') == pytest.approx(63.28 / 54.195, abs=1e-3)
    assert new.value('pot_center_raw') == 500


def test_disagreeing_ratio_inputs_are_refused(tmp_path):
    path = _spec(tmp_path, belt_pulley_teeth_steer=(40, '사용자'), pot_to_steer_ratio=(1.0, '사용자'))
    code, out, report = _run(tmp_path, path)
    assert code == 2 and report is None


def test_out_of_range_value_widens_the_range(tmp_path):
    path = _spec(tmp_path, wheelbase_m=(.95, '측정'))      # base range [0.65, 0.90]
    code, out, report = _run(tmp_path, path)
    assert code == 0
    assert report['widened_ranges'] == [dict(name='wheelbase_m', old=[.65, .9], new=[.65, .95])]


def test_ratio_outside_the_pot_window_is_refused_and_kept(tmp_path):
    path = _spec(tmp_path, pot_to_steer_ratio=(4.6, '사용자'))
    code, out, report = _run(tmp_path, path)
    assert code == 2 and report is None
    assert (out / 'front_steer_v1_spec_test.refused.yaml').exists()


@pytest.mark.parametrize('field,item,why', [
    ('wheelbase_m', dict(value=.8, source='측정', date=None, note='x'), 'date'),
    ('wheelbase_m', dict(value=.8, source='측정', date='20261320', note='x'), 'calendar'),
    ('wheelbase_m', dict(value=.8, source='측정', date='20261120', note=None), 'note'),
    ('wheelbase_m', dict(value=.8, source='누군가', date=None, note='x'), 'source'),
])
def test_input_rules(tmp_path, field, item, why):
    doc = yaml.safe_load(TEMPLATE.read_bytes())
    doc['fields'][field].update(item)
    path = tmp_path / 'spec.yaml'
    path.write_bytes(yaml.safe_dump(doc, allow_unicode=True).encode('utf-8'))
    with pytest.raises(AS.SpecError) as info:
        AS.load_spec(path)
    assert why in str(info.value)


def test_unknown_field_is_refused(tmp_path):
    doc = yaml.safe_load(TEMPLATE.read_bytes())
    doc['fields']['wheel_color'] = dict(q='Q2', value='red', source='사용자', date=None, note=None)
    path = tmp_path / 'spec.yaml'
    path.write_bytes(yaml.safe_dump(doc, allow_unicode=True).encode('utf-8'))
    with pytest.raises(AS.SpecError):
        AS.load_spec(path)


def test_limit_switch_both_is_refused(tmp_path):
    path = _spec(tmp_path, steer_limit_switch=('both', '사용자'))
    code, out, report = _run(tmp_path, path)
    assert code == 2


# --- 2026-11-16: real dimensions, LiDAR mount, base origin convention, steering firmware (schema v4) ----------------
INPUT_20261116 = vp.PROFILES_DIR / 'steer_spec_input_20261116.yaml'
PROFILE_20261116 = vp.PROFILES_DIR / 'front_steer_v1_spec_20261116.yaml'


def test_the_20261116_input_gives_the_installed_profile(tmp_path):
    code, out, report = _run(tmp_path, INPUT_20261116)
    assert code == 0
    assert (out / 'front_steer_v1_spec_20261116.yaml').read_bytes() == PROFILE_20261116.read_bytes()
    assert report['firmware']['new']['ok'] and report['host']['new_ok'] and report['gazebo']['new']['ok']
    gz = report['gazebo']['new']['value']
    assert gz['laser_link_xyz'] == [.1185, 0., .55]                   # the generated laser link follows the profile
    assert gz['wheels_xyz']['left_front_wheel'] == [.3285, .2635, .13]
    assert gz['steer_joints']['left_front_steering_joint']['upper'] == pytest.approx(math.radians(20.), abs=1e-5)


def test_lidar_inputs_and_base_origin(tmp_path):
    doc = yaml.safe_load(TEMPLATE.read_bytes())
    doc['base_origin'] = 'footprint_centre_symmetric_axles'
    for name, value, source in (('wheelbase_m', .7, '측정'), ('lidar_behind_front_axle_m', .2, '측정'),
                                ('lidar_top_height_m', .6, '측정'), ('lidar_scan_below_top_m', .03, '자리값')):
        doc['fields'][name].update(value=value, source=source, note='시험 값',
                                   date='20261120' if source == '측정' else None)
    doc['new_profile_id'] = 'front_steer_v1_spec_test'
    path = tmp_path / 'spec.yaml'
    path.write_bytes(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False).encode('utf-8'))
    code, out, report = _run(tmp_path, path)
    assert code == 0
    new = vp.load_profile(report['candidate_path'])
    assert new.value('rear_axle_x_in_base_m') == pytest.approx(-.35) and new.provenance('rear_axle_x_in_base_m') == \
        'DEFINITION'
    assert new.value('lidar_x_in_base_m') == pytest.approx(.15) and new.provenance('lidar_x_in_base_m') == \
        'MEASURED_20261120'
    assert new.value('lidar_z_m') == pytest.approx(.57) and new.provenance('lidar_z_m') == 'ESTIMATED'
    assert new.schema == vp.SCHEMA_V3                                  # no steer_firmware block: still v3


def test_lidar_top_needs_the_scan_offset(tmp_path):
    path = _spec(tmp_path, lidar_top_height_m=(.57, '측정'))
    code, out, report = _run(tmp_path, path)
    assert code == 2


def test_steer_firmware_makes_schema_v4(tmp_path):
    doc = yaml.safe_load(TEMPLATE.read_bytes())
    doc['steer_firmware'] = dict(mode='abs_hold_8', step_time_measured_s=dict(
        value=dict(outward_20=3.5, return_20=2.5), source='측정', date='20261120', note='시험 값'))
    doc['new_profile_id'] = 'front_steer_v1_spec_test'
    path = tmp_path / 'spec.yaml'
    path.write_bytes(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False).encode('utf-8'))
    code, out, report = _run(tmp_path, path)
    assert code == 0
    new = vp.load_profile(report['candidate_path'])
    assert new.schema == vp.SCHEMA_V4 and vp.abs_hold_mode(new)
    assert new.value('abs_hold_fw') == AS.ABS_HOLD_FW
    assert new.provenance('steer_step_time_measured_s') == 'MEASURED_20261120'
    doc['steer_firmware']['mode'] = 'servo'
    path.write_bytes(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False).encode('utf-8'))
    with pytest.raises(AS.SpecError):
        AS.load_spec(path)
