"""The indoor corridor field record (steering/field_records/corridor_indoor_20261119.yaml): measured values of the
2026-11-18/19 tile-corridor session, kept apart from the orchard values.

Checks: the schema and that every value names a provenance tag of the file and a source; the record is not a field
steering record (field_steer_record.records() / newest() ignore it, so the vehicle executor configuration and the SIM
vehicle profiles stay as they are); the numbers the executor work of that day uses are the ones written here.
"""
from pathlib import Path

import yaml

from steering import field_steer_record as FSR

PATH = Path(__file__).resolve().parent / 'steering' / 'field_records' / 'corridor_indoor_20261119.yaml'


def load():
    return yaml.safe_load(PATH.read_text(encoding='utf-8'))


def test_schema_and_provenance():
    doc = load()
    assert doc['schema'] == 'ubts.field_corridor_record.v1'
    assert doc['vehicle'] == FSR.VEHICLE
    assert 'orchard' in doc['not_for']
    tags = set(doc['provenance_tags'])
    assert tags == {'MEASURED_LEAD', 'MEASURED_LOG', 'FIRMWARE'}
    for group in ('values', 'observations'):
        for name, entry in doc[group].items():
            assert entry['provenance'] in tags, name
            assert entry.get('source'), name
            assert 'value' in entry, name


def test_not_a_field_steering_record():
    names = [r.path.name for r in FSR.records()]
    assert PATH.name not in names
    assert FSR.newest().path.name.startswith('field_steer_')


def test_key_numbers():
    v = load()['values']
    assert v['breakaway_duty_pct']['value'] == [16, 18]
    assert v['stop_distance_m']['value'] == 0.125 and v['stop_distance_m']['duty_pct'] == 40
    assert v['firmware_sm']['value']['RESTART_TOL'] == 10 and v['firmware_sm']['value']['PWM_MIN'] == 70
    assert v['steer_equivalent_angle_at_command_12_deg']['value'] == {'left': 14.4, 'right': 16.7}
    speeds = [p['speed_m_s'] for p in v['speed_by_applied_duty_autonomy']['value']]
    assert speeds == sorted(speeds)
