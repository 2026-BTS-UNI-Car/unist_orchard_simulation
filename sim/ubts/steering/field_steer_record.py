"""Field steering records of the vehicle's ESP32-S3 board (sim/ubts/steering/field_records/; docs/steering/EXECUTOR_ESP32_KO.md 3).

A record is one YAML file field_records/field_steer_<YYYYMMDD>_<HHMM>.yaml (schema ubts.field_steer_record.v1): what was
measured on the real board in one field session (calibration marks saved in the board's NVS, the ends found, the step
response, the potentiometer noise), each value with a provenance tag and its source, plus the conditions (for example
wheels lifted, standstill, a capacitor on the potentiometer). Records are versioned like the vehicle profiles: a file is
never edited after it is committed; a newer measurement is a new file. newest() picks the record with the latest
measured_at (the file name carries the same local date and time, so the names sort in time order); older records stay
for the history. The SIM never reads these files (the SIM profiles keep their own values).

executor_values(record) turns a record into the row executor fields of the vehicle configuration
(steering.vehicle_executor_config):

  steer_esp32_cal                      esp32_cal [FIRMWARE_NVS]: the CAL line the bridge must report at the Arm
  steer_esp32_mark_deg                 mark_angle_deg (equivalent wheel angle of the command +-max_command)
  steer_measured_rate_deg_s            mark_angle_deg / t, t = the slowest full-range step of step_response
  steer_measured_rate_standstill_deg_s (rate_rule slowest_full_range_step), rounded down to 0.001; equivalent wheel
                                       degrees, the executor's scale (the command degrees per second are
                                       max_command / t; the adapter converts with the same mark angle)
  steer_rate_limit_deg_s               rate_fraction x the measured rate (rounded down), the user rule: the slew limit and
  steer_rate_limit_standstill_deg_s    the steering GO threshold are at most 0.8 x the measured rate (todo 0.00 da-6)
  steer_rate_min_deg_s                 the same value (steering GO threshold)
  steer_servo_deadband_deg             the firmware target tolerance (counts) on the side with fewer counts per command
                                       degree, in equivalent degrees
  steer_polarity_checked               polarity_checked (B0)
  steer_range_measured                 range_measured (B2 / B3 with an angle meter)

Nothing else comes from a record: the drive measurements (C2 / C3 / C7), the zero offset (F1), the steering start PWM
(B4) and the per-wheel soft margin stay with the generator (null until measured). Pure Python + PyYAML (safe loader).
"""
from __future__ import annotations

import datetime
import hashlib
import math
import re
from pathlib import Path

import yaml

SCHEMA = 'ubts.field_steer_record.v1'
RECORDS_DIR = Path(__file__).resolve().parent / 'field_records'
VEHICLE = 'ubts_orchard_rover'
NAME = re.compile(r'field_steer_(\d{8})_(\d{4})\Z')
MEASURED_LIFTED = re.compile(r'MEASURED_(\d{4})(\d{2})(\d{2})_LIFTED\Z')
PLAIN_TAGS = ('FIRMWARE', 'FIRMWARE_NVS', 'USER_VISUAL', 'ESTIMATED', 'USER_DECISION', 'DEFINITION', 'NOT_MEASURED')
RESULT_KINDS = ('field_lifted_standstill',)
RATE_RULES = ('slowest_full_range_step',)
TOP_KEYS = ('schema', 'record_id', 'vehicle', 'measured_at', 'result_kind', 'conditions', 'values', 'derive')
CONDITION_KEYS = ('wheels', 'vehicle_motion', 'steering_only', 'drive_pwm_cap', 'pot_capacitor', 'note', 'logs')
VALUE_KEYS = ('firmware', 'esp32_cal', 'end_left', 'end_right', 'center_straight', 'step_response', 'pot_noise_rest',
              'mark_angle_deg', 'polarity_checked', 'range_measured', 'drive_direction_verified')
DERIVE_KEYS = ('rate_rule', 'rate_fraction')
FLAGS = ('polarity_checked', 'range_measured', 'drive_direction_verified')


class RecordError(ValueError):
    pass


def _floor3(value):
    return math.floor(value * 1000. + 1e-9) / 1000.


def _int(value):
    return type(value) is int


def _real(value):
    return type(value) in (int, float) and math.isfinite(value)


def _tag_ok(tag):
    if tag in PLAIN_TAGS:
        return True
    match = MEASURED_LIFTED.match(tag) if isinstance(tag, str) else None
    if not match:
        return False
    try:
        datetime.date(*(int(part) for part in match.groups()))
    except ValueError:
        return False
    return True


class FieldSteerRecord:
    """One validated record file (load())."""

    def __init__(self, path, doc, data):
        self.path = Path(path)
        self.doc = doc
        self.sha256 = hashlib.sha256(data).hexdigest()
        self.record_id = doc['record_id']
        self.measured_at = datetime.datetime.fromisoformat(doc['measured_at'])

    def value(self, name):
        group = 'derive' if name in DERIVE_KEYS else 'values'
        return self.doc[group][name]['value']

    def provenance(self, name):
        group = 'derive' if name in DERIVE_KEYS else 'values'
        return self.doc[group][name]['provenance']

    @property
    def cal(self):
        cal = self.value('esp32_cal')
        return (cal['right'], cal['center'], cal['left'], cal['max_command'])

    def slowest_full_range_step(self):
        """(seconds, from, to) of the slowest step that spans the full command range (|to - from| = max_command)."""
        full = self.value('esp32_cal')['max_command']
        steps = [(s['reached_s'], s['from'], s['to']) for s in self.value('step_response')['steps']
                 if abs(s['to'] - s['from']) == full]
        return max(steps)

    def label(self):
        return '%s (sha256 %s, %s)' % (self.record_id, self.sha256[:12], self.doc['result_kind'])


def _check_entry(group, name, entry, problems):
    if not isinstance(entry, dict) or sorted(entry) != ['provenance', 'source', 'value']:
        problems.append('%s.%s: needs exactly value, provenance, source' % (group, name))
        return False
    if not _tag_ok(entry['provenance']):
        problems.append('%s.%s: unknown provenance %r' % (group, name, entry['provenance']))
    if not isinstance(entry['source'], str) or not entry['source'].strip():
        problems.append('%s.%s: source must be a non-empty text' % (group, name))
    return True


def _problems(stem, doc):
    p = []
    if not isinstance(doc, dict) or sorted(doc) != sorted(TOP_KEYS):
        return ['top level: needs exactly the keys %s' % (TOP_KEYS,)]
    if doc['schema'] != SCHEMA:
        p.append('schema: %r is not %s' % (doc['schema'], SCHEMA))
    match = NAME.match(stem)
    if not match:
        p.append('file name %s: not field_steer_<YYYYMMDD>_<HHMM>' % stem)
    if doc['record_id'] != stem:
        p.append('record_id %r differs from the file name %s' % (doc['record_id'], stem))
    if not isinstance(doc['vehicle'], str) or not doc['vehicle']:
        p.append('vehicle: must be a name')
    try:
        when = datetime.datetime.fromisoformat(doc['measured_at'])
    except (TypeError, ValueError):
        p.append('measured_at: needs an ISO time text with an offset, e.g. 2026-11-16T22:12:00+09:00')
        when = None
    if when is not None:
        if when.tzinfo is None:
            p.append('measured_at: needs an offset (+09:00)')
        elif match and when.strftime('%Y%m%d_%H%M') != '%s_%s' % match.groups():
            p.append('measured_at %s does not match the file name %s' % (doc['measured_at'], stem))
    if doc['result_kind'] not in RESULT_KINDS:
        p.append('result_kind: one of %s' % (RESULT_KINDS,))
    cond = doc['conditions']
    if not isinstance(cond, dict) or not set(cond) <= set(CONDITION_KEYS) or \
            not {'wheels', 'vehicle_motion', 'steering_only', 'drive_pwm_cap', 'pot_capacitor'} <= set(cond):
        p.append('conditions: wheels, vehicle_motion, steering_only, drive_pwm_cap, pot_capacitor (+ note, logs)')
    else:
        if cond['wheels'] not in ('lifted', 'ground') or not isinstance(cond['vehicle_motion'], str):
            p.append('conditions: wheels lifted / ground, vehicle_motion a text')
        if type(cond['steering_only']) is not bool or type(cond['pot_capacitor']) is not bool or \
                not _int(cond['drive_pwm_cap']) or not 0 <= cond['drive_pwm_cap'] <= 255:
            p.append('conditions: steering_only and pot_capacitor booleans, drive_pwm_cap 0..255')
        if doc['result_kind'] == 'field_lifted_standstill' and (cond['wheels'] != 'lifted' or
                                                                 cond['vehicle_motion'] != 'standstill'):
            p.append('result_kind field_lifted_standstill needs wheels lifted and vehicle_motion standstill')
    for group, keys in (('values', VALUE_KEYS), ('derive', DERIVE_KEYS)):
        block = doc[group]
        if not isinstance(block, dict) or sorted(block) != sorted(keys):
            p.append('%s: needs exactly %s' % (group, keys))
            return p
        bad = [name for name in keys if not _check_entry(group, name, block[name], p)]
        if bad:
            return p
    return p + _value_problems(doc)


def _value_problems(doc):
    p = []
    v = {name: entry['value'] for name, entry in doc['values'].items()}
    d = {name: entry['value'] for name, entry in doc['derive'].items()}
    fw = v['firmware']
    fw_keys = ('name', 'cal_runtime', 'drive_watchdog_ms', 'target_tol_counts', 'restart_tol_counts', 'guard_right',
               'guard_left')
    if not isinstance(fw, dict) or sorted(fw) != sorted(fw_keys):
        return ['firmware: needs exactly %s' % (fw_keys,)]
    if not isinstance(fw['name'], str) or type(fw['cal_runtime']) is not bool or \
            not (fw['drive_watchdog_ms'] is None or (_int(fw['drive_watchdog_ms']) and fw['drive_watchdog_ms'] > 0)) or \
            not all(_int(fw[k]) and fw[k] > 0 for k in ('target_tol_counts', 'restart_tol_counts')) or \
            not all(_int(fw[k]) and 0 <= fw[k] <= 4095 for k in ('guard_right', 'guard_left')):
        p.append('firmware: name text, cal_runtime bool, drive_watchdog_ms null or > 0, tolerances > 0, guards 0..4095')
    cal = v['esp32_cal']
    if not isinstance(cal, dict) or sorted(cal) != ['center', 'left', 'max_command', 'right'] or \
            not all(_int(x) for x in cal.values()):
        return p + ['esp32_cal: needs integer right, center, left, max_command']
    right, center, left, full = cal['right'], cal['center'], cal['left'], cal['max_command']
    if not (0 <= right < center < left <= 4095 and 1 <= full <= 90):
        p.append('esp32_cal: need 0 <= right < center < left <= 4095 and 1 <= max_command <= 90')
    if not p and not (fw['guard_right'] <= right and left <= fw['guard_left']):
        p.append('esp32_cal: marks outside the firmware hard guards')
    for side, sign in (('end_left', 1), ('end_right', -1)):
        end = v[side]
        if not isinstance(end, dict) or not _int(end.get('pot')) or not isinstance(end.get('stop_reason'), str):
            p.append('%s: needs an integer pot and a stop_reason' % side)
        elif sign * (end['pot'] - (left if sign > 0 else right)) < 0:
            p.append('%s: the mark lies outside the end found (%d)' % (side, end['pot']))
    center_straight = v['center_straight']
    if not isinstance(center_straight, dict) or not all(_int(center_straight.get(k)) for k in ('pot_min', 'pot_max')) \
            or center_straight['pot_min'] > center_straight['pot_max']:
        p.append('center_straight: needs integer pot_min <= pot_max')
    step = v['step_response']
    steps = step.get('steps') if isinstance(step, dict) else None
    if not isinstance(steps, list) or not steps or not _real(step.get('tolerance_command_deg')) or \
            step['tolerance_command_deg'] <= 0:
        p.append('step_response: needs tolerance_command_deg > 0 and a list of steps')
    else:
        for s in steps:
            if not isinstance(s, dict) or sorted(s) != ['from', 'reached_s', 'to'] or \
                    not all(_real(s[k]) for k in s) or s['reached_s'] <= 0 or \
                    max(abs(s['from']), abs(s['to'])) > full or s['from'] == s['to']:
                p.append('step_response: each step needs from, to (within +-max_command, different) and reached_s > 0')
                break
        else:
            if not any(abs(s['to'] - s['from']) == full for s in steps):
                p.append('step_response: no full-range step (|to - from| = max_command) for rate_rule')
    noise = v['pot_noise_rest']
    if not isinstance(noise, dict) or not all(_real(noise.get(k)) for k in ('min', 'max', 'p2p', 'stdev')) or \
            noise['max'] - noise['min'] != noise['p2p']:
        p.append('pot_noise_rest: needs min, max, p2p = max - min, stdev')
    if not _real(v['mark_angle_deg']) or not 5. <= v['mark_angle_deg'] <= 40.:
        p.append('mark_angle_deg: a number 5..40 (equivalent wheel degrees of the command max_command)')
    for name in FLAGS:
        if type(v[name]) is not bool:
            p.append('%s: must be true or false' % name)
        elif v[name] is False and doc['values'][name]['provenance'] != 'NOT_MEASURED':
            p.append('%s false: provenance NOT_MEASURED' % name)
        elif v[name] is True and doc['values'][name]['provenance'] == 'NOT_MEASURED':
            p.append('%s true: needs the provenance of the check, not NOT_MEASURED' % name)
    if d['rate_rule'] not in RATE_RULES:
        p.append('rate_rule: one of %s' % (RATE_RULES,))
    if not _real(d['rate_fraction']) or not 0. < d['rate_fraction'] <= 1.:
        p.append('rate_fraction: a number in (0, 1]')
    return p


def load(path):
    """Read and validate one record file; RecordError lists every problem."""
    path = Path(path)
    data = path.read_bytes()
    try:
        doc = yaml.safe_load(data)
    except yaml.YAMLError as exc:
        raise RecordError('%s: not YAML (%s)' % (path.name, exc)) from None
    problems = _problems(path.stem if path.suffix == '.yaml' else path.name, doc)
    if path.suffix != '.yaml':
        problems.insert(0, 'file name %s: needs the .yaml suffix' % path.name)
    if problems:
        raise RecordError('%s: %s' % (path.name, '; '.join(problems)))
    return FieldSteerRecord(path, doc, data)


def records(directory=None, vehicle=VEHICLE):
    """Every record of the vehicle in the folder, oldest first (by measured_at). Two records with the same measured_at
    are refused (the newest must be unique)."""
    found = []
    for path in sorted(Path(directory or RECORDS_DIR).glob('field_steer_*.yaml')):
        record = load(path)
        if record.doc['vehicle'] == vehicle:
            found.append(record)
    found.sort(key=lambda r: r.measured_at)
    for older, newer in zip(found, found[1:]):
        if older.measured_at == newer.measured_at:
            raise RecordError('%s and %s have the same measured_at' % (older.path.name, newer.path.name))
    return found


def newest(directory=None, vehicle=VEHICLE):
    """The record with the latest measured_at (older records are kept and ignored); RecordError when there is none."""
    found = records(directory, vehicle)
    if not found:
        raise RecordError('no field steering record for %s in %s' % (vehicle, directory or RECORDS_DIR))
    return found[-1]


def executor_values(record, firmware_name=None, command_limit_deg=None):
    """(values, sources): the row executor fields a record gives (module docstring). firmware_name / command_limit_deg:
    the executor's ESP32 bridge constants (bridge_client.ESP32_FIRMWARE, config.ESP32_COMMAND_LIMIT_DEG); a record of
    another firmware or command scale is refused."""
    fw = record.value('firmware')
    right, center, left, full = record.cal
    if firmware_name is not None and fw['name'] != firmware_name:
        raise RecordError('%s: firmware %s, the executor bridge expects %s' % (record.record_id, fw['name'],
                                                                              firmware_name))
    if command_limit_deg is not None and float(full) != float(command_limit_deg):
        raise RecordError('%s: max_command %d, the executor bridge scale is +-%g' % (record.record_id, full,
                                                                                   command_limit_deg))
    mark = float(record.value('mark_angle_deg'))
    seconds, start, end = record.slowest_full_range_step()
    rate = _floor3(mark / seconds)
    fraction = float(record.value('rate_fraction'))
    limit = _floor3(fraction * rate)
    counts_per_command_deg = min(center - right, left - center) / float(full)
    deadband = round(fw['target_tol_counts'] / counts_per_command_deg * mark / full, 3)
    values = dict(steer_esp32_cal=(right, center, left, full), steer_esp32_mark_deg=mark,
                  steer_measured_rate_deg_s=rate, steer_measured_rate_standstill_deg_s=rate,
                  steer_rate_limit_deg_s=limit, steer_rate_limit_standstill_deg_s=limit, steer_rate_min_deg_s=limit,
                  steer_servo_deadband_deg=deadband,
                  steer_polarity_checked=record.value('polarity_checked'),
                  steer_range_measured=record.value('range_measured'))
    rid = record.record_id
    cond = record.doc['conditions']
    where = 'wheels %s, %s%s' % (cond['wheels'], cond['vehicle_motion'],
                                 ', capacitor on the pot' if cond['pot_capacitor'] else '')
    sources = dict(
        field_record=dict(id=rid, sha256=record.sha256, measured_at=record.doc['measured_at'],
                          result_kind=record.doc['result_kind'], conditions=where,
                          path=record.path.name),
        steer_esp32_cal='%s esp32_cal [%s]: the CAL line the board reports (NVS)' % (rid, record.provenance('esp32_cal')),
        steer_esp32_mark_deg='%s mark_angle_deg [%s]' % (rid, record.provenance('mark_angle_deg')),
        steer_measured_rate_deg_s='%s step_response [%s] (%s): mark %.3g deg / %.2f s, the slowest full-range step '
                                  '%+g -> %+g command deg (%.3f command deg/s); rounded down. Standstill value used for '
                                  'driving too (not measured on the ground or while driving)'
                                  % (rid, record.provenance('step_response'), where, mark, seconds, start, end,
                                     full / seconds),
        steer_rate_limit_deg_s='%s rate_fraction %g [%s] x the measured rate, rounded down (also steer_rate_min_deg_s, '
                               'the steering GO threshold)' % (rid, fraction, record.provenance('rate_fraction')),
        steer_servo_deadband_deg='%s firmware target tolerance %d counts [%s] / %.2f counts per command deg (CAL, the '
                                 'side with fewer counts) x mark / %d' % (rid, fw['target_tol_counts'],
                                                                         record.provenance('firmware'),
                                                                         counts_per_command_deg, full),
        steer_polarity_checked='%s polarity_checked [%s]' % (rid, record.provenance('polarity_checked')),
        steer_range_measured='%s range_measured [%s]' % (rid, record.provenance('range_measured')))
    return values, sources
