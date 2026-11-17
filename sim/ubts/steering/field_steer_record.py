"""Field steering records of the vehicle's ESP32-S3 board (sim/ubts/steering/field_records/; docs/steering/EXECUTOR_ESP32_KO.md 3).

A record is one YAML file field_records/field_steer_<YYYYMMDD>_<HHMM>.yaml (schema ubts.field_steer_record.v1 or v2): what
was measured on the real board in one field session (calibration marks saved in the board's NVS, the ends found, the step
response, the potentiometer noise), each value with a provenance tag and its source, plus the conditions (for example
wheels lifted, standstill, a capacitor on the potentiometer). Records are versioned like the vehicle profiles: a file is
never edited after it is committed; a newer measurement is a new file. newest() picks the record with the latest
measured_at (the file name carries the same local date and time, so the names sort in time order); older records stay
for the history. The SIM never reads these files (the SIM profiles keep their own values).

Schema v2 (from field_steer_20261116_2321, the SMOOTH firmware) adds to v1: the firmware identity (app sha256, the FW
line words after the name, e.g. SMOOTH=1 and DRIVE_WD=300, checked against drive_watchdog_ms / cal_runtime), the centre
guard, the hold method (pulse / smooth; a smooth hold needs SMOOTH=1 and its SM parameters, and its stop / restart
tolerances are SM STOP_TOL / RESTART_TOL), step_response with its timing (reached_within_tolerance or firmware_drive_ms),
scale (bridge command degrees, + left) and final error, and polarity_evidence (which way the pot increases as the user
saw it; it must agree with the firmware word POT_DECREASES and does not set polarity_checked, the B0 check). v1 files
keep loading unchanged. The firmware name must stay the executor's (bridge_client.ESP32_FIRMWARE); other FW words such
as SMOOTH=1 are accepted.

Schema v3 (from field_steer_20261117_0855) is a supplement record: base_record names an older record of the same vehicle
in the same folder, and the file carries only the values it adds or replaces (any v2 value name, and the drive
measurements breakaway_duty_pct (C2), v_measured_m_s (C3: {duty_pct, speed_m_s}) and measured_stop_distance_m (C7)),
with its own result_kind (field_drive_pre_steering) and conditions (where the values come from, the vehicle then and
now). The merged values are validated as one record; origin(name) tells which record a value comes from. Tag
FIELD_OCT2026_PRE_STEERING: the 2026-10-03 / 10-10 drive records of the same drive motors before the steering unit.

executor_values(record) turns a record into the row executor fields of the vehicle configuration
(steering.vehicle_executor_config):

  steer_esp32_cal                      esp32_cal [FIRMWARE_NVS]: the CAL line the bridge must report at the Arm
  steer_esp32_mark_deg                 mark_angle_deg (equivalent wheel angle of the command +-max_command)
  steer_measured_rate_deg_s            mark_angle_deg / t, rounded down to 0.001; t by rate_rule: the slowest
  steer_measured_rate_standstill_deg_s step with |to - from| = max_command (slowest_full_range_step) or the slowest
                                       step scaled to max_command degrees (slowest_per_full_range); equivalent wheel
                                       degrees, the executor's scale (the command degrees per second are
                                       max_command / t; the adapter converts with the same mark angle)
  steer_rate_limit_deg_s               rate_fraction x the measured rate (rounded down), the user rule: the slew limit and
  steer_rate_limit_standstill_deg_s    the steering GO threshold are at most 0.8 x the measured rate (todo 0.00 da-6)
  steer_rate_min_deg_s                 the same value (steering GO threshold)
  steer_servo_deadband_deg             the firmware stop tolerance (counts; the pulse hold's target tolerance or the
                                       smooth hold's STOP_TOL) on the side with fewer counts per command degree, in
                                       equivalent degrees
  steer_polarity_checked               polarity_checked (B0)
  steer_range_measured                 range_measured (B2 / B3 with an angle meter)
  breakaway_duty_pct                   breakaway_duty_pct (C2), when the record (chain) has it
  v_measured_m_s                       v_measured_m_s.speed_m_s (C3), only when measured at the executor's base duty
  measured_stop_distance_m             measured_stop_distance_m (C7)

Nothing else comes from a record: the zero offset (F1), the steering start PWM (B4), the per-wheel soft margin and the
rolling threshold (C3b) stay with the generator (null until measured). Pure Python + PyYAML (safe loader).
"""
from __future__ import annotations

import datetime
import hashlib
import math
import re
from pathlib import Path

import yaml

SCHEMA = 'ubts.field_steer_record.v1'
SCHEMA_V2 = 'ubts.field_steer_record.v2'     # + the hold method (SMOOTH firmware), firmware identity, polarity evidence
SCHEMA_V3 = 'ubts.field_steer_record.v3'     # a supplement: base_record + the values it adds or replaces (drive)
FULL_SCHEMAS = (SCHEMA, SCHEMA_V2)
SCHEMAS = FULL_SCHEMAS + (SCHEMA_V3,)
MAX_CHAIN = 10                                # base_record links followed at most
RECORDS_DIR = Path(__file__).resolve().parent / 'field_records'
VEHICLE = 'ubts_orchard_rover'
NAME = re.compile(r'field_steer_(\d{8})_(\d{4})\Z')
MEASURED_LIFTED = re.compile(r'MEASURED_(\d{4})(\d{2})(\d{2})_LIFTED\Z')
PLAIN_TAGS = ('FIRMWARE', 'FIRMWARE_NVS', 'USER_VISUAL', 'ESTIMATED', 'USER_DECISION', 'DEFINITION', 'NOT_MEASURED',
              'FIELD_OCT2026_PRE_STEERING')   # the 2026-10-03 / 10-10 drive records, before the steering unit
RESULT_KINDS = ('field_lifted_standstill',)
RATE_RULES = ('slowest_full_range_step', 'slowest_per_full_range')
HOLD_MODES = ('pulse', 'smooth')
TIMINGS = ('reached_within_tolerance', 'firmware_drive_ms')
STEP_SCALE = 'bridge_command_deg_left_plus'
TOP_KEYS = ('schema', 'record_id', 'vehicle', 'measured_at', 'result_kind', 'conditions', 'values', 'derive')
CONDITION_KEYS = ('wheels', 'vehicle_motion', 'steering_only', 'drive_pwm_cap', 'pot_capacitor', 'battery', 'note',
                  'logs')
VALUE_KEYS = ('firmware', 'esp32_cal', 'end_left', 'end_right', 'center_straight', 'step_response', 'pot_noise_rest',
              'mark_angle_deg', 'polarity_checked', 'range_measured', 'drive_direction_verified')
VALUE_KEYS_V2 = VALUE_KEYS + ('polarity_evidence',)
FW_KEYS = ('name', 'cal_runtime', 'drive_watchdog_ms', 'target_tol_counts', 'restart_tol_counts', 'guard_right',
           'guard_left')
FW_KEYS_V2 = FW_KEYS + ('app_sha256', 'fw_tokens', 'center_guard', 'hold_mode', 'sm')
DRIVE_KEYS = ('breakaway_duty_pct', 'v_measured_m_s', 'measured_stop_distance_m')
V3_TOP_KEYS = ('schema', 'record_id', 'vehicle', 'measured_at', 'base_record', 'result_kind', 'conditions', 'values')
V3_RESULT_KINDS = ('field_drive_pre_steering',)
V3_CONDITION_KEYS = ('drive_source', 'vehicle_then', 'vehicle_now', 'battery', 'note', 'logs')
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

    def __init__(self, path, doc, data, base=None):
        self.path = Path(path)
        self.own = doc
        self.base = base
        self.doc = doc if base is None else _merged(base.doc, doc)
        self.sha256 = hashlib.sha256(data).hexdigest()
        self.record_id = doc['record_id']
        self.measured_at = datetime.datetime.fromisoformat(doc['measured_at'])

    def has(self, name):
        return name in self.doc['values']

    def origin(self, name):
        """The record id a value comes from (this record, or a base record of a supplement)."""
        return self.doc.get('origin', {}).get(name, self.record_id)

    def chain(self):
        """Record ids from this record down to its full (v1 / v2) base."""
        return [self.record_id] + (self.base.chain() if self.base is not None else [])

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

    @property
    def schema(self):
        return self.doc['schema']

    def rate_basis(self):
        """(seconds per max_command command degrees, from, to, seconds of that step) by the record's rate_rule:
        slowest_full_range_step = the slowest step with |to - from| = max_command; slowest_per_full_range = every step
        scaled to max_command degrees (seconds x max_command / |to - from|), the slowest one."""
        full = self.value('esp32_cal')['max_command']
        steps = self.value('step_response')['steps']
        if self.value('rate_rule') == 'slowest_full_range_step':
            rows = [(s['reached_s'], s['from'], s['to'], s['reached_s']) for s in steps
                    if abs(s['to'] - s['from']) == full]
        else:
            rows = [(s['reached_s'] * full / abs(s['to'] - s['from']), s['from'], s['to'], s['reached_s'])
                    for s in steps]
        return max(rows)

    def label(self):
        return '%s (sha256 %s, %s)' % (self.record_id, self.sha256[:12], self.doc['result_kind'])


def _merged(base_doc, own):
    """The values of a supplement record (schema v3) over those of its base: steering conditions and derive rules stay
    the base's; the supplement's conditions are kept as drive_conditions; origin names the record of every value."""
    m = dict(base_doc)
    origin = dict(base_doc.get('origin') or {name: base_doc['record_id'] for name in base_doc['values']})
    origin.update({name: own['record_id'] for name in own['values']})
    m.update(schema=own['schema'], record_id=own['record_id'], measured_at=own['measured_at'],
             base_record=own['base_record'], full_schema=base_doc.get('full_schema', base_doc['schema']),
             values=dict(base_doc['values'], **own['values']), origin=origin,
             drive_conditions=own['conditions'], drive_result_kind=own['result_kind'])
    return m


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
    if isinstance(doc, dict) and doc.get('schema') == SCHEMA_V3:
        return _v3_problems(stem, doc)
    if not isinstance(doc, dict) or sorted(doc) != sorted(TOP_KEYS):
        return ['top level: needs exactly the keys %s' % (TOP_KEYS,)]
    if doc['schema'] not in FULL_SCHEMAS:
        return ['schema: %r is not one of %s' % (doc['schema'], SCHEMAS)]
    p = _identity_problems(stem, doc)
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
    value_keys = VALUE_KEYS_V2 if doc['schema'] == SCHEMA_V2 else VALUE_KEYS
    for group, keys in (('values', value_keys), ('derive', DERIVE_KEYS)):
        block = doc[group]
        if not isinstance(block, dict) or sorted(block) != sorted(keys):
            p.append('%s: needs exactly %s' % (group, keys))
            return p
        bad = [name for name in keys if not _check_entry(group, name, block[name], p)]
        if bad:
            return p
    return p + _value_problems(doc)


def _identity_problems(stem, doc):
    """File name = record_id = field_steer_<YYYYMMDD>_<HHMM> = the local date and time of measured_at (with offset)."""
    p = []
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
    return p


def _v3_problems(stem, doc):
    """Schema v3, a supplement record: its own identity, base_record (an older record of the same vehicle in the same
    folder, checked at load), result_kind, the conditions of the values it carries and those values (any of the v2 value
    names and the drive measurements DRIVE_KEYS, each with value / provenance / source)."""
    if sorted(doc) != sorted(V3_TOP_KEYS):
        return ['top level (schema v3): needs exactly the keys %s' % (V3_TOP_KEYS,)]
    p = _identity_problems(stem, doc)
    if not isinstance(doc['base_record'], str) or not NAME.match(doc['base_record']) or \
            doc['base_record'] == doc['record_id']:
        p.append('base_record: another record id field_steer_<YYYYMMDD>_<HHMM>')
    if doc['result_kind'] not in V3_RESULT_KINDS:
        p.append('result_kind (schema v3): one of %s' % (V3_RESULT_KINDS,))
    cond = doc['conditions']
    if not isinstance(cond, dict) or not set(cond) <= set(V3_CONDITION_KEYS) or \
            not all(isinstance(cond.get(k), str) and cond[k] for k in ('drive_source', 'vehicle_then', 'vehicle_now')):
        p.append('conditions (schema v3): drive_source, vehicle_then, vehicle_now texts (+ battery, note, logs)')
    values = doc['values']
    allowed = VALUE_KEYS_V2 + DRIVE_KEYS
    if not isinstance(values, dict) or not values or not set(values) <= set(allowed):
        return p + ['values (schema v3): one or more of %s' % (allowed,)]
    for name, entry in values.items():
        _check_entry('values', name, entry, p)
    return p


def _drive_problems(values):
    """The drive measurements: C2 breakaway duty (int %), C3 speed at a stated duty, C7 stop distance."""
    p = []
    if 'breakaway_duty_pct' in values:
        x = values['breakaway_duty_pct']['value']
        if not (_int(x) and 0 <= x <= 100):
            p.append('breakaway_duty_pct: an integer 0..100 (%)')
    if 'v_measured_m_s' in values:
        x = values['v_measured_m_s']['value']
        if not (isinstance(x, dict) and sorted(x) == ['duty_pct', 'speed_m_s'] and _int(x['duty_pct']) and
                1 <= x['duty_pct'] <= 100 and _real(x['speed_m_s']) and .05 <= x['speed_m_s'] <= 2.):
            p.append('v_measured_m_s: {duty_pct: 1..100, speed_m_s: 0.05..2}')
    if 'measured_stop_distance_m' in values:
        x = values['measured_stop_distance_m']['value']
        if not (_real(x) and .05 <= x <= 5.):
            p.append('measured_stop_distance_m: a number 0.05..5 (m)')
    return p


def _value_problems(doc):
    p = []
    v2 = doc.get('full_schema', doc['schema']) == SCHEMA_V2
    v = {name: entry['value'] for name, entry in doc['values'].items()}
    d = {name: entry['value'] for name, entry in doc['derive'].items()}
    fw = v['firmware']
    fw_keys = FW_KEYS_V2 if v2 else FW_KEYS
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
    if v2 and not p:
        p += _firmware_v2_problems(fw, center)
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
    if v2:
        shape_ok = isinstance(step, dict) and sorted(step) == ['final_error_counts', 'scale', 'steps', 'timing'] and \
            step['timing'] in TIMINGS and step['scale'] == STEP_SCALE and _int(step['final_error_counts']) and \
            step['final_error_counts'] >= 0
        wanted = 'timing (%s), scale %s, final_error_counts >= 0, steps' % ('/'.join(TIMINGS), STEP_SCALE)
    else:
        shape_ok = isinstance(step, dict) and _real(step.get('tolerance_command_deg')) and \
            step['tolerance_command_deg'] > 0
        wanted = 'tolerance_command_deg > 0 and a list of steps'
    if not shape_ok or not isinstance(steps, list) or not steps:
        p.append('step_response: needs %s' % wanted)
    else:
        for s in steps:
            if not isinstance(s, dict) or sorted(s) != ['from', 'reached_s', 'to'] or \
                    not all(_real(s[k]) for k in s) or s['reached_s'] <= 0 or \
                    max(abs(s['from']), abs(s['to'])) > full or s['from'] == s['to']:
                p.append('step_response: each step needs from, to (within +-max_command, different) and reached_s > 0')
                break
        else:
            if d['rate_rule'] == 'slowest_full_range_step' and not any(abs(s['to'] - s['from']) == full for s in steps):
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
    if v2:
        p += _polarity_evidence_problems(v['polarity_evidence'], fw.get('fw_tokens'))
    return p


def _firmware_v2_problems(fw, center):
    """Schema v2 firmware block: identity (app sha256, the FW line words after the name), the centre guard and the hold
    method. A smooth hold needs SMOOTH=1 on the FW line and its SM parameters; its stop / restart tolerances are the SM
    STOP_TOL / RESTART_TOL (the pulse hold has its own constants)."""
    p = []
    sha = fw['app_sha256']
    if sha is not None and not (isinstance(sha, str) and re.fullmatch(r'[0-9a-f]{64}', sha)):
        p.append('firmware.app_sha256: null or 64 lower-case hex digits')
    tokens = fw['fw_tokens']
    if not isinstance(tokens, list) or not all(isinstance(t, str) and t and ' ' not in t for t in tokens):
        return p + ['firmware.fw_tokens: the FW line words after the name, a list of texts']
    keyed = dict(t.split('=', 1) for t in tokens if '=' in t)
    watchdog = fw['drive_watchdog_ms']
    if keyed.get('DRIVE_WD') != (None if watchdog is None else str(watchdog)):
        p.append('firmware: drive_watchdog_ms %r differs from the FW token DRIVE_WD=%s'
                 % (watchdog, keyed.get('DRIVE_WD')))
    if fw['cal_runtime'] != (keyed.get('CAL_RUNTIME') == '1'):
        p.append('firmware: cal_runtime differs from the FW token CAL_RUNTIME')
    guard = fw['center_guard']
    if not (isinstance(guard, list) and len(guard) == 2 and all(_int(x) for x in guard) and guard[0] < guard[1]):
        p.append('firmware.center_guard: [low, high] integers')
    elif not guard[0] <= center <= guard[1]:
        p.append('esp32_cal: CENTER %d outside the firmware centre guard %d..%d' % (center, guard[0], guard[1]))
    mode, sm = fw['hold_mode'], fw['sm']
    if mode not in HOLD_MODES:
        p.append('firmware.hold_mode: one of %s' % (HOLD_MODES,))
    elif mode == 'smooth':
        if keyed.get('SMOOTH') != '1':
            p.append('firmware.hold_mode smooth needs the FW token SMOOTH=1')
        if not isinstance(sm, dict) or not all(isinstance(k, str) and _int(x) for k, x in sm.items()) or \
                not {'STOP_TOL', 'RESTART_TOL'} <= set(sm):
            p.append('firmware.sm: the SM line parameters (integers, with STOP_TOL and RESTART_TOL)')
        elif (sm['STOP_TOL'], sm['RESTART_TOL']) != (fw['target_tol_counts'], fw['restart_tol_counts']):
            p.append('firmware: the smooth hold stops within SM STOP_TOL and restarts at RESTART_TOL '
                     '(target_tol_counts / restart_tol_counts)')
    elif keyed.get('SMOOTH') == '1':
        p.append('firmware.hold_mode pulse but the FW token says SMOOTH=1')
    return p


def _polarity_evidence_problems(evidence, tokens):
    """Schema v2 polarity_evidence: which way the pot increases as the user saw it, with the observations; it must agree
    with the firmware's sign word (POT_DECREASES: the pot decreases toward the right). Evidence only: the B0 check
    (polarity_checked) is a separate flag."""
    if not isinstance(evidence, dict) or evidence.get('pot_increases_toward') not in ('left', 'right') or \
            not isinstance(evidence.get('observations'), list) or not evidence['observations'] or \
            not all(isinstance(o, dict) and _int(o.get('pot')) and isinstance(o.get('seen'), str)
                    for o in evidence['observations']):
        return ['polarity_evidence: pot_increases_toward left / right and observations [{pot, seen}, ...]']
    if isinstance(tokens, list) and 'POT_DECREASES' in tokens and evidence['pot_increases_toward'] != 'left':
        return ['polarity_evidence: the firmware word POT_DECREASES (toward the right) means the pot increases to the '
                'left']
    return []


def load(path, _depth=0):
    """Read and validate one record file; RecordError lists every problem. A supplement (schema v3) loads its
    base_record from the same folder (an older record of the same vehicle) and is validated on the merged values."""
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
    if doc['schema'] != SCHEMA_V3:
        return FieldSteerRecord(path, doc, data)
    if _depth >= MAX_CHAIN:
        raise RecordError('%s: base_record chain longer than %d' % (path.name, MAX_CHAIN))
    base_path = path.parent / (doc['base_record'] + '.yaml')
    if not base_path.is_file():
        raise RecordError('%s: base_record %s not found next to it' % (path.name, doc['base_record']))
    base = load(base_path, _depth + 1)
    when = datetime.datetime.fromisoformat(doc['measured_at'])
    if base.doc['vehicle'] != doc['vehicle'] or not base.measured_at < when:
        raise RecordError('%s: base_record %s must be an older record of the same vehicle'
                          % (path.name, doc['base_record']))
    record = FieldSteerRecord(path, doc, data, base)
    problems = _value_problems(record.doc) + _drive_problems(record.doc['values'])
    if problems:
        raise RecordError('%s (with %s): %s' % (path.name, doc['base_record'], '; '.join(problems)))
    return record


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


def executor_values(record, firmware_name=None, command_limit_deg=None, base_duty_pct=None):
    """(values, sources): the row executor fields a record gives (module docstring). firmware_name / command_limit_deg:
    the executor's ESP32 bridge constants (bridge_client.ESP32_FIRMWARE, config.ESP32_COMMAND_LIMIT_DEG); a record of
    another firmware or command scale is refused. base_duty_pct: the executor's base duty; a C3 speed measured at
    another duty is refused (C3 is the speed at the base duty)."""
    fw = record.value('firmware')
    right, center, left, full = record.cal
    if firmware_name is not None and fw['name'] != firmware_name:
        raise RecordError('%s: firmware %s, the executor bridge expects %s' % (record.record_id, fw['name'],
                                                                              firmware_name))
    if command_limit_deg is not None and float(full) != float(command_limit_deg):
        raise RecordError('%s: max_command %d, the executor bridge scale is +-%g' % (record.record_id, full,
                                                                                   command_limit_deg))
    mark = float(record.value('mark_angle_deg'))
    seconds, start, end, step_seconds = record.rate_basis()
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
    o = record.origin
    cond = record.doc['conditions']
    timing = record.value('step_response').get('timing', 'reached_within_tolerance')
    hold = fw.get('hold_mode', 'pulse')
    where = 'wheels %s, %s%s' % (cond['wheels'], cond['vehicle_motion'],
                                 ', capacitor on the pot' if cond['pot_capacitor'] else '')
    sources = dict(
        field_record=dict(id=rid, sha256=record.sha256, measured_at=record.doc['measured_at'],
                          result_kind=record.own['result_kind'], conditions=where,
                          path=record.path.name, chain=record.chain()),
        steer_esp32_cal='%s esp32_cal [%s]: the CAL line the board reports (NVS)'
                        % (o('esp32_cal'), record.provenance('esp32_cal')),
        steer_esp32_mark_deg='%s mark_angle_deg [%s]' % (o('mark_angle_deg'), record.provenance('mark_angle_deg')),
        steer_measured_rate_deg_s='%s step_response [%s] (%s; %s, %s hold): mark %.3g deg / %.3f s per %d command '
                                  'deg (rate_rule %s: step %+g -> %+g command deg in %.3f s; %.3f command deg/s); '
                                  'rounded down. Standstill value used for driving too (not measured on the ground or '
                                  'while driving)'
                                  % (o('step_response'), record.provenance('step_response'), where, timing, hold, mark,
                                     seconds, full, record.value('rate_rule'), start, end, step_seconds, full / seconds),
        steer_rate_limit_deg_s='%s rate_fraction %g [%s] x the measured rate, rounded down (also steer_rate_min_deg_s, '
                               'the steering GO threshold)' % (o('rate_fraction'), fraction,
                                                               record.provenance('rate_fraction')),
        steer_servo_deadband_deg='%s firmware stop tolerance %d counts (%s hold) [%s] / %.2f counts per command deg '
                                 '(CAL, the side with fewer counts) x mark / %d'
                                 % (o('firmware'), fw['target_tol_counts'], hold, record.provenance('firmware'),
                                    counts_per_command_deg, full),
        steer_polarity_checked='%s polarity_checked [%s]' % (o('polarity_checked'), record.provenance('polarity_checked')),
        steer_range_measured='%s range_measured [%s]' % (o('range_measured'), record.provenance('range_measured')))
    note = ' (%s)' % record.doc['drive_result_kind'] if record.doc.get('drive_result_kind') else ''
    if record.has('breakaway_duty_pct'):
        values['breakaway_duty_pct'] = int(record.value('breakaway_duty_pct'))
        sources['breakaway_duty_pct'] = '%s breakaway_duty_pct [%s]%s' % (
            o('breakaway_duty_pct'), record.provenance('breakaway_duty_pct'), note)
    if record.has('v_measured_m_s'):
        c3 = record.value('v_measured_m_s')
        if base_duty_pct is not None and int(c3['duty_pct']) != int(base_duty_pct):
            raise RecordError('%s: v_measured_m_s measured at %d %%, the executor base duty is %d %% (C3 = the speed at '
                              'the base duty)' % (o('v_measured_m_s'), c3['duty_pct'], base_duty_pct))
        values['v_measured_m_s'] = float(c3['speed_m_s'])
        sources['v_measured_m_s'] = '%s v_measured_m_s at %d %% [%s]%s' % (
            o('v_measured_m_s'), c3['duty_pct'], record.provenance('v_measured_m_s'), note)
    if record.has('measured_stop_distance_m'):
        values['measured_stop_distance_m'] = float(record.value('measured_stop_distance_m'))
        sources['measured_stop_distance_m'] = '%s measured_stop_distance_m [%s]%s' % (
            o('measured_stop_distance_m'), record.provenance('measured_stop_distance_m'), note)
    return values, sources
