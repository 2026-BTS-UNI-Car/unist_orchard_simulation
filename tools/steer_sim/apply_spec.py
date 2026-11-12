#!/usr/bin/env python3
"""Turn the steering specification input file into a new vehicle profile and check it (ST-17 preparation).

One file to edit: sim/ubts/steering/profiles/steer_spec_input.yaml (field names = VEHICLE_SPEC_REQUEST_KO.md question
numbers and profile parameter names). One command (from the repository root):

  python -B tools/steer_sim/apply_spec.py --spec sim/ubts/steering/profiles/steer_spec_input.yaml --out <dir outside the repo>
          [--install] [--force] [--no-checks] [--build-dir <native build dir>]

Steps (docs/steering/SPEC_UPDATE_KO.md):
  1. read and check the input (known fields, value types, source / date rules)
  2. resolve the input-only fields (belt teeth -> belt ratio, end-to-end pot ADC or turn -> ratio and centre raw)
  3. write the candidate profile (base profile + the given values; provenance USER_APPROX / MEASURED_<date> /
     PRODUCT_LABEL; a value outside the base sweep range widens that range and is listed) and load it with the profile
     loader (range, semantic and pot window checks; a refused candidate stops here with the loader's code)
  4. derived values, firmware CFG constants and calibration marks (fw_eeprom), host profile (host_profile_export),
     Gazebo model joints (gz_model): new against base
  5. the summary of changed values and what has to be measured / run again
  6. short checks: native startup and steps, T1 rig straight run and a +2.5 N m disturbance (steer_hold_checks N1, D1;
     development seed 0), fast SIM circle at 10 deg / 0.4 m/s (radius against L / tan delta)
Writes <out>/<profile_id>.yaml, REPORT.json, SUMMARY_KO.txt, host_profile.json and firmware_cfg.txt. --install also
copies the profile to sim/ubts/steering/profiles/ (refused when the file exists unless --force). Results of the
checks are placeholder-based or partially measured SIM results, not field validation.
"""
from __future__ import annotations

import argparse
import copy
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
SIM = ROOT / 'sim' / 'ubts'
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))

import yaml  # noqa: E402

from steering import backend_api as ba  # noqa: E402
from steering import vehicle_profile as vp  # noqa: E402

SCHEMA = 'ubts.steer_spec_input.v1'
REPORT_SCHEMA = 'ubts.steer_spec_report.v1'
SOURCES = {'사용자': 'USER_APPROX', '측정': 'MEASURED', '제품표기': 'PRODUCT_LABEL'}
# input field -> (question, profile parameter or None for an input-only field)
SPEC_FIELDS = {
    'wheelbase_m': 'Q2', 'track_front_m': 'Q2', 'track_rear_m': 'Q2', 'kingpin_width_m': 'Q2',
    'wheel_radius_front_m': 'Q2', 'wheel_radius_rear_m': 'Q2', 'steer_type': 'Q10', 'steer_mech_limit_deg': 'Q2',
    'steer_ackermann_ratio': 'Q2', 'steer_backlash_deg': 'Q10', 'footprint_length_m': 'Q2', 'footprint_width_m': 'Q2',
    'mass_kg': 'Q2', 'steer_self_locking': 'Q11', 'steer_motor_stall_torque_nm': 'Q11',
    'steer_motor_stall_current_a': 'Q11', 'steer_gear_ratio': 'Q11', 'steer_rate_max_deg_s': 'Q11',
    'steer_rate_standstill_deg_s': 'Q11', 'steer_min_pwm_pct': 'Q11', 'steer_min_pwm_standstill_pct': 'Q11',
    'steer_supply_shared_with_drive': 'Q26', 'belt_type': 'Q22', 'belt_pulley_teeth_pot': 'Q22',
    'belt_pulley_teeth_steer': 'Q22', 'belt_pot_play_deg': 'Q23', 'pot_location': 'Q23', 'pot_range_deg': 'Q23',
    'pot_to_steer_ratio': 'Q22', 'pot_adc_end_raws': 'Q24', 'pot_turn_deg_end_to_end': 'Q24',
    'steer_limit_switch': 'Q25',
}
INPUT_ONLY = ('belt_pulley_teeth_steer', 'pot_adc_end_raws', 'pot_turn_deg_end_to_end')
# What to measure or run again when a parameter changes (SPEC_UPDATE_KO.md 4).
RECHECK = {
    'geometry': (('wheelbase_m', 'track_front_m', 'track_rear_m', 'kingpin_width_m', 'wheel_radius_front_m',
                  'wheel_radius_rear_m', 'steer_type', 'steer_mech_limit_deg', 'steer_ackermann_ratio',
                  'footprint_length_m', 'footprint_width_m', 'mass_kg'),
                 'A1(F, G) 기하, A12 두 SIM 일치, A21 한계·스윕, Gazebo 모델 다시 만들기(gz_model), 출발 가능성 A24, '
                 '열 추종 사전등록 행렬(PREREG_STEER_FOLLOW) 다시'),
    'actuator': (('steer_rate_max_deg_s', 'steer_rate_standstill_deg_s', 'steer_min_pwm_pct',
                  'steer_min_pwm_standstill_pct', 'steer_backlash_deg', 'steer_self_locking',
                  'steer_motor_stall_torque_nm', 'steer_supply_shared_with_drive'),
                 'A2(native 시험을 새 프로필로), 서보 최소 출력 간격(servo_min_pwm_pct >= 시작 PWM + 5 %p), '
                 'ST-11 외란 판정(개정 41), 정상 대본 오검출(개정 40 ②), A18 식별 복원, THERMAL·DRV_STUCK 문턱'),
    'pot': (('belt_type', 'belt_pulley_teeth_pot', 'belt_pot_play_deg', 'pot_location', 'pot_range_deg',
             'pot_to_steer_ratio', 'pot_center_raw'),
            'A3(F) 가변저항(native 시험), 보정 표지 B2·B3(펌웨어 marks), POT_JUMP·POT_NOISE 문턱(count 기준), '
            'POLARITY 문턱(polp), 벨트 이 건너뜀 대본(R3)'),
    'switch': (('steer_limit_switch',), '리밋 스위치가 생기면 배선·펌웨어 설계부터(지금은 모형 없음)'),
}
FIXED_NOMINAL = {'steer_zero_offset_deg': 0.0, 'pot_cal_residual_deg': 0.0}


class SpecError(ValueError):
    pass


# --- 1. input ---------------------------------------------------------------------------------------------------------------
def _date(text, name):
    if not isinstance(text, (str, int)) or not str(text).isdigit() or len(str(text)) != 8:
        raise SpecError('%s: date must be YYYYMMDD, got %r' % (name, text))
    text = str(text)
    try:
        datetime.date(int(text[:4]), int(text[4:6]), int(text[6:]))
    except ValueError:
        raise SpecError('%s: %s is not a calendar date' % (name, text)) from None
    return text


def load_spec(path):
    doc = yaml.safe_load(Path(path).read_bytes())
    if not isinstance(doc, dict) or doc.get('schema') != SCHEMA:
        raise SpecError('spec schema must be %s' % SCHEMA)
    unknown = sorted(set(doc) - {'schema', 'base_profile', 'new_profile_id', 'description', 'fields'})
    if unknown:
        raise SpecError('unknown top-level keys: %s' % ', '.join(unknown))
    fields = doc.get('fields') or {}
    given = {}
    for name, item in fields.items():
        if name not in SPEC_FIELDS:
            raise SpecError('unknown field %r (known: VEHICLE_SPEC_REQUEST question fields)' % name)
        if not isinstance(item, dict) or set(item) - {'q', 'value', 'source', 'date', 'note'}:
            raise SpecError('%s: a field is {q, value, source, date, note}' % name)
        if item.get('q') not in (None, SPEC_FIELDS[name]):
            raise SpecError('%s: q is %s (the field belongs to %s)' % (name, item.get('q'), SPEC_FIELDS[name]))
        if item.get('value') is None:
            continue
        source = item.get('source')
        if source not in SOURCES:
            raise SpecError('%s: source must be one of %s' % (name, ', '.join(SOURCES)))
        date = _date(item.get('date'), name) if source == '측정' else (
            _date(item['date'], name) if item.get('date') is not None else None)
        note = item.get('note')
        if source in ('측정', '제품표기') and (not isinstance(note, str) or not note.strip()):
            raise SpecError('%s: a %s value needs a note (it becomes the profile source)' % (name, source))
        given[name] = dict(value=item['value'], source=source, date=date, note=note, q=SPEC_FIELDS[name])
    return doc, given


# --- 2. input-only fields ------------------------------------------------------------------------------------------------------
def _travel_at_ratio_one(get):
    view = dict((n, get(n)) for n in vp._POT_TRAVEL_INPUTS)
    view['pot_to_steer_ratio'] = 1.0
    return vp.pot_travel_deg(view.__getitem__)


def resolve_inputs(given, base_doc):
    """Turn the input-only fields into profile parameters (in place). Returns notes."""
    notes = []

    def current(name):
        if name in given:
            value = given[name]['value']
            if name == 'steer_mech_limit_deg' and not isinstance(value, dict):
                return {k: float(value) for k in base_doc['parameters'][name]['value']}
            return value
        return base_doc['parameters'][name]['value']
    ratios = []
    if 'belt_pulley_teeth_steer' in given:
        item = given.pop('belt_pulley_teeth_steer')
        teeth_pot = current('belt_pulley_teeth_pot')
        ratios.append(('belt_pulley_teeth_steer', float(item['value']) / float(teeth_pot), item))
        notes.append('벨트 비 = 조향 쪽 잇수 %s / 가변저항 쪽 잇수 %s' % (item['value'], teeth_pot))
    if 'pot_turn_deg_end_to_end' in given:
        item = given.pop('pot_turn_deg_end_to_end')
        travel = _travel_at_ratio_one(current)
        ratios.append(('pot_turn_deg_end_to_end', float(item['value']) / travel, item))
        notes.append('벨트 비 = 가변저항 회전 %s deg / 링크 끝-끝 %.2f deg' % (item['value'], travel))
    if 'pot_adc_end_raws' in given:
        item = given.pop('pot_adc_end_raws')
        raws = item['value']
        if not (isinstance(raws, list) and len(raws) == 3 and all(isinstance(r, int) for r in raws)):
            raise SpecError('pot_adc_end_raws: need [left, centre, right] integer ADC values')
        left, centre, right = raws
        span_deg = abs(left - right) / 2. ** current('adc_bits') * float(current('pot_range_deg'))
        travel = _travel_at_ratio_one(current)
        ratios.append(('pot_adc_end_raws', span_deg / travel, item))
        given['pot_center_raw'] = dict(item, value=int(centre))
        notes.append('ADC 끝-끝 %d count = 가변저항 %.2f deg / 링크 %.2f deg; 가운데 raw %d' % (abs(left - right), span_deg,
                                                                                         travel, centre))
    if 'pot_to_steer_ratio' in given:
        ratios.append(('pot_to_steer_ratio', float(given['pot_to_steer_ratio']['value']), given['pot_to_steer_ratio']))
    if ratios:
        values = [r for _, r, _ in ratios]
        if max(values) > min(values) * 1.05:
            raise SpecError('the belt ratio inputs disagree by more than 5 %%: %s'
                            % ', '.join('%s %.3f' % (n, r) for n, r, _ in ratios))
        name, ratio, item = ratios[0]
        given['pot_to_steer_ratio'] = dict(item, value=round(ratio, 4))
    return notes


# --- 3. candidate profile ----------------------------------------------------------------------------------------------------
def _widen(rng, value):
    """A range that holds value (numeric [lo, hi] or a per-key mapping); None when no change was needed."""
    if isinstance(rng, list) and len(rng) == 2 and all(isinstance(x, (int, float)) for x in rng):
        leaves = list(value.values()) if isinstance(value, dict) else [value]
        lo, hi = min([rng[0]] + leaves), max([rng[1]] + leaves)
        return None if (lo, hi) == tuple(rng) else [lo, hi]
    if isinstance(rng, dict) and 'choices' in rng:
        if value not in rng['choices']:
            raise SpecError('value %r is not an allowed choice %r' % (value, rng['choices']))
        return None
    return None


def build_candidate(base_doc, given, spec_doc):
    doc = copy.deepcopy(base_doc)
    params = doc['parameters']
    widened, applied = [], {}
    dates = []
    for name, item in sorted(given.items()):
        if name not in params:
            raise SpecError('field %s has no profile parameter' % name)
        rec = params[name]
        if rec.get('provenance') == 'DERIVED':
            raise SpecError('%s is DERIVED; give its inputs' % name)
        value = item['value']
        if name == 'steer_mech_limit_deg' and not isinstance(value, dict):
            value = {k: float(value) for k in rec['value']}
        if isinstance(value, int) and not isinstance(value, bool) and isinstance(rec['value'], float):
            value = float(value)
        tag = SOURCES[item['source']]
        if tag == 'MEASURED':
            tag = 'MEASURED_' + item['date']
            dates.append(item['date'])
        if rec['range'] is None:          # SPEC_PENDING null value
            rec['range'] = [0.0, max(1.0, float(value) * 2.)] if isinstance(value, (int, float)) else None
        new_range = _widen(rec['range'], value)
        if new_range is not None:
            widened.append(dict(name=name, old=rec['range'], new=new_range))
            rec['range'] = new_range
        rec['value'] = value
        rec['provenance'] = tag
        rec['stale_after_mod'] = False
        if tag.startswith('MEASURED_') or tag == 'PRODUCT_LABEL':
            rec['source'] = '%s (%s) %s' % (item['q'], item['source'], item['note'])
        elif item.get('note'):
            rec['source'] = '%s (%s) %s' % (item['q'], item['source'], item['note'])
        rec['note'] = ((rec.get('note') or '') + ' | 사양 반영: %s %s' % (item['q'], item['source'])).strip(' |')
        applied[name] = dict(value=value, provenance=tag, q=item['q'])
    stamp = max(dates) if dates else datetime.date.today().strftime('%Y%m%d')
    doc['profile_id'] = spec_doc.get('new_profile_id') or 'front_steer_v1_spec_%s' % stamp
    doc['description'] = spec_doc.get('description') or (
        '%s + 사양 입력 %d칸(%s). 남은 자리값은 그대로다.' % (base_doc['profile_id'], len(applied), stamp))
    return doc, widened, applied


def dump_profile(doc, base_id, spec_sha):
    header = ('# 조향 차량 프로필 %s (tools/steer_sim/apply_spec.py가 만든 판)\n'
              '# 바탕: %s. 사양 입력 파일 SHA-256 %s. 바꾼 칸의 provenance와 source가 출처다.\n'
              '# 이 파일은 버전 파일이다. 고치지 말고 사양 입력을 고쳐 새 판을 만든다(docs/steering/SPEC_UPDATE_KO.md).\n'
              % (doc['profile_id'], base_id, spec_sha))
    body = yaml.safe_dump(doc, allow_unicode=True, sort_keys=False, default_flow_style=None, width=120)
    return (header + body).encode('utf-8')


# --- 4. derived quantities ---------------------------------------------------------------------------------------------------
def _nominal(profile):
    return profile.with_overrides(FIXED_NOMINAL, label='nominal_offsets')


def firmware_constants(profile):
    from steering.native import fw_eeprom
    view = _nominal(profile)
    startup = ba.native_startup_config(view, 0)
    image, details = fw_eeprom.calibrated_eeprom(startup, view)
    out = dict(params=details['params'], marks=dict(raw_l=details['raw_l'], raw_c=details['raw_c'],
                                                    raw_r=details['raw_r']),
               soft_permille=[details['soft_l'], details['soft_r']],
               scale_permille_per_deg=round(details['scale_permille_per_deg'], 4))
    if view.has('pot_cal_half_width_counts'):
        out['pot_counts_per_deg'] = round(1. / view.value('pot_lsb_deg'), 4)
    return out


def host_values(profile):
    from steering import host_profile_export as hpe
    return hpe.build(_nominal(profile))


def gazebo_values(profile):
    from steering import gz_model
    text = gz_model._generated_rover_sdf()
    sdf, manifest = gz_model.steer_world_sdf(text, profile)
    import xml.etree.ElementTree as E
    model = E.fromstring(sdf).find("world/model[@name='%s']" % gz_model.MODEL_NAME)
    joints = gz_model.describe_model(model)['joints']
    steer = {name: dict(lower=round(j['lower'], 5) if j['lower'] is not None else None,
                        upper=round(j['upper'], 5) if j['upper'] is not None else None)
             for name, j in joints.items() if 'steer' in name}
    keep = ('steer_type', 'drive_mode', 'wheel_collision', 'sdf_sha256')
    return dict(steer_joints=steer, manifest={k: manifest.get(k) for k in keep})


def _flat(value, prefix=''):
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            out.update(_flat(item, '%s%s.' % (prefix, key)))
        return out
    return {prefix.rstrip('.'): value}


def _diff(old, new):
    a, b = _flat(old), _flat(new)
    return [dict(key=k, old=a.get(k), new=b.get(k)) for k in sorted(set(a) | set(b)) if a.get(k) != b.get(k)]


def _safe(fn, *args):
    try:
        return dict(ok=True, value=fn(*args))
    except Exception as exc:   # report, do not stop the summary
        return dict(ok=False, error='%s: %s' % (type(exc).__name__, exc))


# --- 6. short checks ------------------------------------------------------------------------------------------------------------
def short_checks(profile_path, out_dir, build_dir):
    from steering import steer_hold_checks as shc
    from steering.actuator_link import NativeSteeringBackend
    profile = vp.load_profile(profile_path)
    checks = []

    def record(name, fn):
        started = time.perf_counter()
        try:
            value = fn()
            checks.append(dict(name=name, ok=bool(value.pop('ok', True)), wall_s=round(time.perf_counter() - started, 2),
                               **value))
        except Exception as exc:
            checks.append(dict(name=name, ok=False, error='%s: %s' % (type(exc).__name__, exc),
                               trace=traceback.format_exc(limit=3)))

    def native_steps():
        backend = NativeSteeringBackend(_nominal(profile), 1, executable=None, build_dir=build_dir)
        try:
            backend.reset(0.0)
            for _ in range(50):
                out = backend.step(ba.SteerStepInput(dt_s=.01, steer_pwm=.8, vehicle_speed_m_s=.4,
                                                     vehicle_stopped=False))
            return dict(delta_eq_deg_after_0_5s=round(math.degrees(out.delta_eq_rad), 3), pot_raw=out.pot_raw)
        finally:
            backend.close()

    def rig(set_name, case):
        ep = shc.run_episode(set_name, case, 0, str(profile_path), build_dir)
        codes = [c for c, _ in ep['f_lines']]
        ok = ep['armed'] and not codes and not ep['latches']
        return dict(ok=ok, f_lines=codes, latches=ep['latches'], max_steer_dev_deg=ep.get('max_steer_dev_deg'),
                    recovery_s=ep.get('recovery_s'), max_lateral_dev_m=ep.get('max_lateral_dev_m'),
                    speed_mean_m_s=ep.get('speed_mean_m_s'))

    def circle():
        from steering import fast_sim
        from steering import scene_spec as ss
        import numpy as np
        old_dir = fast_sim.PROFILES_DIR
        fast_sim.PROFILES_DIR = Path(profile_path).parent      # this process only (the candidate may be outside)
        fast_sim._PROFILE_CACHE.clear()
        try:
            scene = ss.SceneSpec.from_dict(dict(
                name='spec_circle_10deg', seeds=[0], duration_s=14.,
                profile=dict(profile_id=profile.profile_id, overrides=FIXED_NOMINAL, override_label='nominal'),
                maneuver=dict(kind='circle', params=dict(speed_m_s=.4, steer_deg=10.))))
            arrays, _, rec = fast_sim.run_episode(scene, 0)
        finally:
            fast_sim.PROFILES_DIR = old_dir
            fast_sim._PROFILE_CACHE.clear()
        t = np.asarray(arrays['t_s'])
        x, y = np.asarray(arrays['truth_x_m']), np.asarray(arrays['truth_y_m'])
        keep = t >= 6.
        xs, ys = x[keep], y[keep]
        a = np.column_stack([xs, ys, np.ones_like(xs)])
        sol = np.linalg.lstsq(a, -(xs ** 2 + ys ** 2), rcond=None)[0]
        radius = float(math.sqrt(sol[0] ** 2 / 4. + sol[1] ** 2 / 4. - sol[2]))
        wheelbase = profile.value('wheelbase_m')
        geom = wheelbase / math.tan(math.radians(10.))
        out = dict(radius_m=round(radius, 3), l_over_tan_cmd_m=round(geom, 3), ratio_cmd=round(radius / geom, 4))
        if 'truth_delta_eq_rad' in arrays.dtype.names if hasattr(arrays, 'dtype') else 'truth_delta_eq_rad' in arrays:
            delta = float(np.mean(np.asarray(arrays['truth_delta_eq_rad'])[keep]))
            out.update(true_delta_deg=round(math.degrees(delta), 3),
                       ratio_true=round(radius / (wheelbase / math.tan(delta)), 4))
        return out

    record('native_startup_and_steps', native_steps)
    record('t1_rig_N1_straight', lambda: rig('normal', 'N1'))
    record('t1_rig_D1_disturbance_2_5Nm', lambda: rig('disturbance', 'D1'))
    record('fast_sim_circle_10deg', circle)
    return checks


# --- main -------------------------------------------------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--spec', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--install', action='store_true')
    ap.add_argument('--force', action='store_true')
    ap.add_argument('--no-checks', action='store_true')
    ap.add_argument('--build-dir', default=None)
    args = ap.parse_args(argv)
    out = Path(args.out)
    from steering.scene_spec import inside_repository
    if inside_repository(out):
        ap.error('--out must be outside the repository')
    out.mkdir(parents=True, exist_ok=True)
    spec_bytes = Path(args.spec).read_bytes()
    spec_sha = hashlib.sha256(spec_bytes).hexdigest()
    try:
        spec_doc, given = load_spec(args.spec)
        base_path = vp.PROFILES_DIR / (spec_doc['base_profile'] + '.yaml')
        base_bytes = base_path.read_bytes()
        base = vp.parse_profile(base_bytes, origin=str(base_path))
        base_doc = yaml.safe_load(base_bytes)
        notes = resolve_inputs(given, base_doc)
        doc, widened, applied = build_candidate(base_doc, given, spec_doc)
    except (SpecError, vp.ProfileError, OSError) as exc:
        print('spec input refused: %s' % exc)
        return 2
    data = dump_profile(doc, base.profile_id, spec_sha)
    candidate_path = out / (doc['profile_id'] + '.yaml')
    try:
        profile = vp.parse_profile(data, origin=str(candidate_path))
    except vp.ProfileError as exc:
        (out / (doc['profile_id'] + '.refused.yaml')).write_bytes(data)
        print('candidate profile refused by the loader: code %s, %s' % (exc.code, exc))
        print('written for inspection: %s' % (out / (doc['profile_id'] + '.refused.yaml')))
        return 2
    candidate_path.write_bytes(data)
    changed = []
    for name in profile.names():
        old = base.value(name) if base.has(name) else None
        new = profile.value(name)
        if old != new or base.provenance(name) != profile.provenance(name):
            changed.append(dict(name=name, old=old, new=new, old_provenance=base.provenance(name),
                                new_provenance=profile.provenance(name), derived=profile.is_derived(name)))
    fw_old, fw_new = _safe(firmware_constants, base), _safe(firmware_constants, profile)
    host_old, host_new = _safe(host_values, base), _safe(host_values, profile)
    gz_old, gz_new = _safe(gazebo_values, base), _safe(gazebo_values, profile)
    recheck = [dict(area=area, why=text, names=sorted(set(names) & set(applied)))
               for area, (names, text) in RECHECK.items() if set(names) & set(applied)]
    report = dict(
        schema=REPORT_SCHEMA, spec_path=str(args.spec), spec_sha256=spec_sha, base_profile=base.result_label(),
        candidate=profile.result_label(), candidate_path=str(candidate_path), applied=applied, input_notes=notes,
        widened_ranges=widened, changed=changed, warnings=profile.warnings(),
        spec_pending_left=profile.spec_pending(),
        firmware=dict(base=fw_old, new=fw_new,
                      cfg_diff=_diff(fw_old.get('value', {}), fw_new.get('value', {})) if fw_old['ok'] and fw_new['ok']
                      else None),
        host=dict(diff=_diff(host_old.get('value', {}), host_new.get('value', {})) if host_old['ok'] and host_new['ok']
                  else None, base_ok=host_old['ok'], new_ok=host_new['ok'], error=host_new.get('error')),
        gazebo=dict(base=gz_old, new=gz_new), recheck=recheck, checks=None)
    if host_new['ok']:
        (out / 'host_profile.json').write_bytes(
            (json.dumps(host_new['value'], ensure_ascii=False, indent=1) + '\n').encode('utf-8'))
    if fw_new['ok']:
        lines = ['# CFG 값(실기 펌웨어 V3, ESP_PROTOCOL_V3 4.10). 보정 표지는 B2 벤치에서 실측으로 다시 잡는다.']
        lines += ['CFG %s %d' % (k, v) for k, v in fw_new['value']['params'].items()]
        lines.append('# 계산 보정 표지(자리값 가변저항 모형): raw L/C/R %s, 소프트 한계 permille %s'
                     % (fw_new['value']['marks'], fw_new['value']['soft_permille']))
        (out / 'firmware_cfg.txt').write_bytes(('\n'.join(lines) + '\n').encode('utf-8'))
    if not args.no_checks:
        build_dir = args.build_dir or os.environ.get('UBTS_STEER_BUILD_DIR')
        report['checks'] = short_checks(candidate_path, out, build_dir)
    if args.install:
        target = vp.PROFILES_DIR / candidate_path.name
        if target.exists() and not args.force:
            print('install refused: %s exists (use --force or a new new_profile_id)' % target)
            return 2
        target.write_bytes(data)
        report['installed'] = str(target)
    (out / 'REPORT.json').write_bytes(
        (json.dumps(report, ensure_ascii=False, indent=1, default=str) + '\n').encode('utf-8'))
    summary = summary_text(report)
    (out / 'SUMMARY_KO.txt').write_bytes(summary.encode('utf-8'))
    sys.stdout.buffer.write(summary.encode('utf-8'))
    failed = [c for c in report['checks'] or () if not c['ok']]
    return 1 if failed else 0


def summary_text(report):
    lines = ['사양 반영 요약 (%s)' % report['candidate']['profile_id'],
             '  결과 종류: %s, 남은 자리값(SPEC_PENDING) %d칸' % (report['candidate']['result_kind'],
                                                       len(report['spec_pending_left'])),
             '  바탕: %s (%s)' % (report['base_profile']['profile_id'], report['base_profile']['profile_sha256'][:12]),
             '  후보: %s' % report['candidate_path']]
    for note in report['input_notes']:
        lines.append('  입력 환산: ' + note)
    lines.append('바뀐 값:')
    for item in report['changed']:
        lines.append('  %-32s %s -> %s  [%s -> %s]%s' % (item['name'], item['old'], item['new'], item['old_provenance'],
                                                         item['new_provenance'], ' (계산값)' if item['derived'] else ''))
    for item in report['widened_ranges']:
        lines.append('  범위 넓힘: %s %s -> %s (스윕 결과는 다시)' % (item['name'], item['old'], item['new']))
    for w in report['warnings']:
        lines.append('  경고: %s %s' % (w['code'], w.get('detail', '')))
    fw = report['firmware']
    if fw['cfg_diff'] is not None:
        lines.append('펌웨어 CFG·보정 표지 바뀜 (firmware_cfg.txt):')
        for d in fw['cfg_diff']:
            lines.append('  %s: %s -> %s' % (d['key'], d['old'], d['new']))
    else:
        lines.append('펌웨어 상수 계산 실패: %s' % (fw['new'].get('error') or fw['base'].get('error')))
    host = report['host']
    if host['diff'] is not None:
        lines.append('호스트 프로필 바뀜 (host_profile.json): %d칸' % len(host['diff']))
        for d in host['diff'][:30]:
            lines.append('  %s: %s -> %s' % (d['key'], d['old'], d['new']))
    else:
        lines.append('호스트 프로필 계산 실패: %s' % host.get('error'))
    gz = report['gazebo']
    if gz['new']['ok']:
        lines.append('Gazebo 조향 관절 한계(rad): %s' % gz['new']['value']['steer_joints'])
    else:
        lines.append('Gazebo 모델 계산 실패: %s' % gz['new'].get('error'))
    lines.append('다시 재거나 돌릴 것:')
    for item in report['recheck']:
        lines.append('  [%s] %s: %s' % (item['area'], ', '.join(item['names']), item['why']))
    if report['checks'] is not None:
        lines.append('짧은 확인(자리값/일부 실측 기반 SIM 결과):')
        for c in report['checks']:
            extra = {k: v for k, v in c.items() if k not in ('name', 'ok', 'trace')}
            lines.append('  %-28s %s %s' % (c['name'], 'OK' if c['ok'] else 'FAIL', json.dumps(extra,
                                                                                            ensure_ascii=False)))
    if report.get('installed'):
        lines.append('설치: %s (커밋한다)' % report['installed'])
    return '\n'.join(lines) + '\n'


if __name__ == '__main__':
    sys.exit(main())
