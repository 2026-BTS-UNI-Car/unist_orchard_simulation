"""The vehicle executor configuration of the Jetson kit (steering.vehicle_executor_config, steering.field_steer_record;
docs/steering/EXECUTOR_ESP32_KO.md 3).

Checks: the committed file tools/row_executor/config.vehicle_esp32_shadow.json is the generator's output for the profile
front_steer_v1_spec_20261116 and the NEWEST field steering record (steering/field_records/); it is a SHADOW front_steer
single-row configuration for the ESP32-S3 bridge on the vehicle socket with the operator page on 127.0.0.1; the vehicle
values are the profile's and the board values are the newest record's (no record value is written here as a literal: a
newer record supersedes the older one and these tests follow it); the fixed rules hold for any record (rate limit and
GO threshold at most 0.8 x the measured rate, also on the board's command scale; polarity and range not confirmed; the
drive values C2 / C3 / C7 are the record's or null, C3 at the base duty, and the stop model is at least as long as C7,
loosened nowhere; with motor output only the open steering checks remain); a newer record wins and older ones are kept,
a supplement record (schema v3) adds values over its base; malformed records are refused; without a record the profile
values come back; it builds a SHADOW executor loop that never creates a bridge transport.
When a record confirms the polarity (B0) or the range (B2 / B3), test_fixed_rules must be changed on purpose.
unittest style (pytest or python -m unittest).
"""
import datetime
import math
import shutil
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import yaml

from steering import abs_hold
from steering import field_steer_record as FSR
from steering import vehicle_executor_config as V

COMMITTED = V.REPO / 'tools' / 'row_executor' / 'config.vehicle_esp32_shadow.json'
FIRMWARE = 'ABS-HOLD-8-REBOUND-LIMITS'
DRIVE = ('breakaway_duty_pct', 'v_measured_m_s', 'measured_stop_distance_m')
STILL_OPEN = ('rolling_breakaway_duty_pct', 'measured_yaw_at_max_diff_deg_s', 'steer_wheel_soft_margin_deg',
              'steer_min_pwm_pct')


def newest_full():
    return [r for r in FSR.records() if r.schema in FSR.FULL_SCHEMAS][-1]


def drive_value(record, name):
    if not record.has(name):
        return None
    value = record.value(name)
    return value['speed_m_s'] if name == 'v_measured_m_s' else value


def write_record(folder, doc, stem=None):
    stem = stem or doc['record_id']
    path = Path(folder) / (stem + '.yaml')
    path.write_bytes(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False).encode('utf-8'))
    return path


def shifted_doc(record, hours, **cal):
    """The record's document moved by `hours` (name, record_id, measured_at) with CAL marks changed."""
    when = record.measured_at + datetime.timedelta(hours=hours)
    doc = yaml.safe_load(record.path.read_bytes())
    doc['record_id'] = 'field_steer_' + when.strftime('%Y%m%d_%H%M')
    doc['measured_at'] = when.isoformat()
    doc['values']['esp32_cal']['value'].update(cal)
    return doc


def rate_seconds(record):
    """Seconds per max_command command degrees by the record's rate_rule (written out here, not the module's code)."""
    full = record.value('esp32_cal')['max_command']
    steps = record.value('step_response')['steps']
    if record.value('rate_rule') == 'slowest_full_range_step':
        return max(s['reached_s'] for s in steps if abs(s['to'] - s['from']) == full)
    return max(s['reached_s'] * full / abs(s['to'] - s['from']) for s in steps)


class VehicleExecutorConfig(unittest.TestCase):
    def setUp(self):
        self.record = FSR.newest()
        self.profile = V.load()
        self.cfg = V.vehicle_config(self.profile)

    def test_committed_file_is_the_generator_output(self):
        self.assertEqual(COMMITTED.read_bytes(), V.render().encode('utf-8'))

    def test_values(self):
        cfg, profile, record = self.cfg, self.profile, self.record
        self.assertEqual((cfg.vehicle_profile, cfg.mode, cfg.motor_output_enabled, cfg.steer_bridge_kind),
                         ('front_steer', 'single_row', False, 'esp32_abs_hold'))
        self.assertEqual((cfg.socket_path, cfg.bind_host, cfg.bind_port), ('/run/ubts-rover/control.sock', '127.0.0.1', 8097))
        self.assertEqual(cfg.steer_wheelbase_m, profile.value('wheelbase_m'))
        self.assertEqual((cfg.steer_limit_left_mrad, cfg.steer_limit_right_mrad), (281, 281))
        # the board values are the newest record's
        board, _ = FSR.executor_values(record)
        for name, value in board.items():
            self.assertEqual(getattr(cfg, name), value, name)
        cal = record.value('esp32_cal')
        mark = record.value('mark_angle_deg')
        self.assertEqual(cfg.steer_esp32_cal, (cal['right'], cal['center'], cal['left'], cal['max_command']))
        self.assertEqual(cfg.steer_esp32_mark_deg, mark)
        self.assertAlmostEqual(cfg.steer_measured_rate_deg_s, mark / rate_seconds(record), delta=1e-3)
        self.assertEqual(cfg.steer_measured_rate_standstill_deg_s, cfg.steer_measured_rate_deg_s)
        self.assertAlmostEqual(cfg.steer_rate_limit_deg_s, record.value('rate_fraction') * cfg.steer_measured_rate_deg_s,
                               delta=1e-3)
        self.assertEqual(cfg.steer_rate_min_deg_s, cfg.steer_rate_limit_deg_s)
        counts = min(cal['center'] - cal['right'], cal['left'] - cal['center']) / cal['max_command']
        self.assertAlmostEqual(cfg.steer_servo_deadband_deg,
                               record.value('firmware')['target_tol_counts'] / counts * mark / cal['max_command'], places=3)
        self.assertEqual(cfg.steer_polarity_checked, record.value('polarity_checked'))
        self.assertEqual(cfg.steer_range_measured, record.value('range_measured'))
        for name in DRIVE:                                       # the drive values: the record's, else null
            self.assertEqual(getattr(cfg, name), drive_value(record, name), name)
        self.assertEqual(cfg.steer_zero_offset_deg,               # F1: the record's (an estimate so far), else null
                         record.value('zero_offset_deg') if record.has('zero_offset_deg') else None)
        if record.has('v_measured_m_s'):
            self.assertEqual(record.value('v_measured_m_s')['duty_pct'], cfg.base_duty_pct)   # C3 = speed at base duty

    def test_fixed_rules(self):
        cfg, record = self.cfg, self.record
        XC = V._row_executor_config()
        from row_executor import bridge_client as B
        fraction = XC.STEER_RATE_FRACTION
        measured = cfg.steer_measured_rate_deg_s
        self.assertIsNotNone(measured)
        for name in ('steer_rate_limit_deg_s', 'steer_rate_limit_standstill_deg_s', 'steer_rate_min_deg_s'):
            self.assertLessEqual(getattr(cfg, name), fraction * measured + 1e-12, name)        # user rule (0.8 x)
        self.assertLessEqual(cfg.steer_rate_limit_standstill_deg_s, cfg.steer_rate_limit_deg_s)
        self.assertGreaterEqual(measured, cfg.steer_rate_min_deg_s)                            # GO rate rule passes
        # the same rule on the board's command scale (the adapter converts with the same mark angle)
        board_rate = record.value('esp32_cal')['max_command'] / rate_seconds(record)
        self.assertLessEqual(cfg.steer_rate_limit_deg_s * XC.esp32_scale(cfg), fraction * board_rate + 1e-9)
        # not confirmed yet: polarity (B0), range with an angle meter (B2 / B3), drive direction on this vehicle;
        # still unmeasured values null
        self.assertIs(cfg.steer_polarity_checked, False)
        self.assertIs(cfg.steer_range_measured, False)
        self.assertIs(record.value('drive_direction_verified'), False)
        for name in STILL_OPEN:
            self.assertIsNone(getattr(cfg, name), name)
        if cfg.steer_zero_offset_deg is not None:                # F1 inside the steering GO rule
            self.assertLessEqual(abs(cfg.steer_zero_offset_deg), XC.STEER_GO_ZERO_OFFSET_MAX_DEG)
        # the stop model: only stop-model keys, none looser than the defaults, at v_measured at least the measured C7
        d = XC.STOP_MODEL_DEFAULTS
        self.assertLessEqual(set(cfg.controller_overrides), set(d))
        model = {k: cfg.controller_overrides.get(k, v) for k, v in d.items()}
        self.assertLessEqual(model['stop_decel_m_s2'], d['stop_decel_m_s2'])
        for name in ('front_margin_m', 'reaction_s', 'stop_onset_s'):
            self.assertGreaterEqual(model[name], d[name])
        if cfg.measured_stop_distance_m is not None and cfg.v_measured_m_s is not None:
            self.assertGreaterEqual(XC.stopping_distance_model(cfg.v_measured_m_s, model['stop_decel_m_s2'],
                                                               model['reaction_s'], model['stop_onset_s']),
                                    cfg.measured_stop_distance_m)
        # with motor output only the open steering checks remain once the record carries C2 / C3 / C7
        motor = XC.feasibility(replace(cfg, motor_output_enabled=True, tuning_file=None))
        self.assertEqual(motor['status'], 'infeasible')
        if all(record.has(name) for name in DRIVE):
            self.assertTrue(motor['problems'])
            for problem in motor['problems']:
                self.assertIn('steer_', problem)
        # the Arm checks of the ESP32 client can pass with this board: firmware name, command scale, firmware drive
        # watchdog (required), soft limits inside the bridge range; the marks lie inside the ends found
        cal = record.value('esp32_cal')
        self.assertEqual(record.value('firmware')['name'], B.ESP32_FIRMWARE)
        self.assertEqual(float(cal['max_command']), XC.ESP32_COMMAND_LIMIT_DEG)
        self.assertTrue(cfg.steer_esp32_require_fw_watchdog)
        self.assertGreater(record.value('firmware')['drive_watchdog_ms'] or 0, 0)
        self.assertEqual(XC.esp32_rule_problems(cfg), [])
        self.assertLessEqual(record.value('end_right')['pot'], cal['right'])
        self.assertGreaterEqual(record.value('end_left')['pot'], cal['left'])
        self.assertEqual(record.doc['result_kind'], 'field_lifted_standstill')
        # SHADOW: every missing field measurement is 'unknown', no problem, motor output refused
        feasibility = cfg.feasibility()
        self.assertEqual(feasibility['status'], 'unknown')
        self.assertEqual(feasibility.get('problems') or [], [])
        self.assertFalse(any('measured steering rate unknown' in u for u in feasibility['unknown']))
        self.assertTrue(any('steer_polarity_checked' in u for u in feasibility['unknown']))
        with self.assertRaises(XC.ConfigError):
            replace(cfg, motor_output_enabled=True).validate()

    def test_profile_only_build(self):
        profile = self.profile
        cfg = V.vehicle_config(profile, record=None)
        fw = profile.value('abs_hold_fw')
        self.assertEqual(cfg.steer_esp32_cal, (fw['cal_right'], fw['cal_center'], fw['cal_left'],
                                               int(fw['command_max_deg'])))
        self.assertAlmostEqual(cfg.steer_esp32_mark_deg, abs_hold.mark_angle_deg(profile), places=3)
        self.assertAlmostEqual(cfg.steer_rate_limit_deg_s, .8 * abs_hold.executor_values(profile)['rate_deg_s'], places=3)
        self.assertIsNone(cfg.steer_measured_rate_deg_s)
        self.assertEqual(cfg.steer_rate_min_deg_s, V._row_executor_config().ExecutorConfig().steer_rate_min_deg_s)
        self.assertIsNone(V.vehicle_values(profile, record=None)[1]['field_record'])
        self.assertNotEqual(V.render(profile, record=None), V.render(profile))

    def test_committed_records_load(self):
        found = FSR.records()
        self.assertTrue(found)
        self.assertEqual(found[-1].record_id, self.record.record_id)
        for record in found:
            self.assertEqual(record.path.stem, record.record_id)
            FSR.executor_values(record, firmware_name=FIRMWARE, command_limit_deg=20., base_duty_pct=55)
            if record.schema == FSR.SCHEMA_V3:                   # a supplement over an older record of its folder
                self.assertLess(record.base.measured_at, record.measured_at)
                self.assertIn(record.base.record_id, [r.record_id for r in found])
        sources = V.vehicle_values(self.profile)[1]
        self.assertEqual(sources['field_record']['id'], self.record.record_id)
        self.assertEqual(sources['field_record']['sha256'], self.record.sha256)
        self.assertEqual(sources['field_record']['chain'], self.record.chain())

    def test_stop_model_overrides(self):
        XC = V._row_executor_config()
        d = XC.STOP_MODEL_DEFAULTS
        self.assertEqual(V.stop_model_overrides(None, .6), {})
        self.assertEqual(V.stop_model_overrides(.46, None), {})
        covered = XC.stopping_distance_model(.46, d['stop_decel_m_s2'], d['reaction_s'], d['stop_onset_s'])
        self.assertEqual(V.stop_model_overrides(.46, covered - .01), {})      # the defaults already cover it
        for v, c7 in ((.46, .6), (.4, .5), (.5, .7)):
            decel = V.stop_model_overrides(v, c7)['stop_decel_m_s2']
            self.assertLess(decel, d['stop_decel_m_s2'])
            self.assertGreaterEqual(XC.stopping_distance_model(v, decel, d['reaction_s'], d['stop_onset_s']), c7)
            self.assertLess(XC.stopping_distance_model(v, decel + .01, d['reaction_s'], d['stop_onset_s']), c7)
        with self.assertRaises(ValueError):                                   # would need a deceleration below 0.2
            V.stop_model_overrides(.46, 1.5)


class NewestRecordWins(unittest.TestCase):
    def setUp(self):
        self.base = newest_full()
        self.dir = tempfile.mkdtemp(prefix='field_records_')
        self.addCleanup(shutil.rmtree, self.dir, True)
        shutil.copy(self.base.path, self.dir)

    def test_a_newer_record_supersedes_and_the_older_stays(self):
        center = self.base.value('esp32_cal')['center']
        doc = shifted_doc(self.base, 3, right=center - 400, left=center + 520)
        write_record(self.dir, doc)
        found = FSR.records(self.dir)
        self.assertEqual([r.record_id for r in found], [self.base.record_id, doc['record_id']])
        newest = FSR.newest(self.dir)
        self.assertEqual(newest.record_id, doc['record_id'])
        cfg = V.vehicle_config(record=newest)
        self.assertEqual(cfg.steer_esp32_cal[:3], (center - 400, center, center + 520))
        self.assertEqual(V.vehicle_config(record=newest.path).steer_esp32_cal, cfg.steer_esp32_cal)
        self.assertTrue((Path(self.dir) / self.base.path.name).exists())               # the older record is kept

    def test_order_comes_from_measured_at_and_ties_are_refused(self):
        write_record(self.dir, shifted_doc(self.base, -24, right=self.base.value('esp32_cal')['right'] + 20))
        self.assertEqual(FSR.newest(self.dir).record_id, self.base.record_id)
        # the same instant written with another offset (a valid name of its own): a tie, refused
        utc = self.base.measured_at.astimezone(datetime.timezone.utc)
        same = yaml.safe_load(self.base.path.read_bytes())
        same.update(record_id='field_steer_' + utc.strftime('%Y%m%d_%H%M'), measured_at=utc.isoformat())
        FSR.load(write_record(self.dir, same))
        with self.assertRaises(FSR.RecordError):
            FSR.records(self.dir)
        # a file name that is not field_steer_<YYYYMMDD>_<HHMM> is refused
        copy = yaml.safe_load(self.base.path.read_bytes())
        with self.assertRaises(FSR.RecordError):
            FSR.load(write_record(tempfile.mkdtemp(dir=self.dir), copy, stem=self.base.record_id + '_copy'))

    def supplement(self, hours, base=None, **values):
        when = self.base.measured_at + datetime.timedelta(hours=hours)
        entry = lambda value: dict(value=value, provenance='FIELD_OCT2026_PRE_STEERING', source='test')  # noqa: E731
        return dict(schema=FSR.SCHEMA_V3, record_id='field_steer_' + when.strftime('%Y%m%d_%H%M'),
                    vehicle=self.base.doc['vehicle'], measured_at=when.isoformat(),
                    base_record=base or self.base.record_id, result_kind='field_drive_pre_steering',
                    conditions=dict(drive_source='test', vehicle_then='test', vehicle_now='test'),
                    values={name: entry(value) for name, value in values.items()})

    def test_a_supplement_adds_values_over_its_base(self):
        doc = self.supplement(2, breakaway_duty_pct=30, v_measured_m_s=dict(duty_pct=55, speed_m_s=.4),
                              measured_stop_distance_m=.5)
        write_record(self.dir, doc)
        newest = FSR.newest(self.dir)
        self.assertEqual((newest.record_id, newest.chain()), (doc['record_id'], [doc['record_id'], self.base.record_id]))
        self.assertEqual((newest.cal, newest.origin('esp32_cal'), newest.origin('breakaway_duty_pct')),
                         (self.base.cal, self.base.record_id, doc['record_id']))
        cfg = V.vehicle_config(record=newest)
        self.assertEqual((cfg.breakaway_duty_pct, cfg.v_measured_m_s, cfg.measured_stop_distance_m), (30, .4, .5))
        self.assertEqual(cfg.steer_esp32_cal, V.vehicle_config(record=self.base).steer_esp32_cal)
        self.assertTrue((Path(self.dir) / self.base.path.name).exists())
        # a full record newer than the supplement wins over both
        later = shifted_doc(self.base, 3, right=self.base.value('esp32_cal')['right'] + 10)
        write_record(self.dir, later)
        self.assertEqual(FSR.newest(self.dir).record_id, later['record_id'])

    def test_supplement_refusals(self):
        cases = [
            self.supplement(1, base='field_steer_20200101_0000', breakaway_duty_pct=30),     # base not in the folder
            self.supplement(-1, breakaway_duty_pct=30),                                       # older than its base
            self.supplement(1, breakaway_duty_pct=130),
            self.supplement(1, v_measured_m_s=dict(duty_pct=55)),
            self.supplement(1, measured_stop_distance_m=0.),
            self.supplement(1, esp32_cal=dict(self.base.value('esp32_cal'),
                                              left=self.base.value('end_left')['pot'] + 1)),   # merged rule
            self.supplement(1, colour='red'),
            self.supplement(1),                                                               # no values
        ]
        for i, doc in enumerate(cases):
            with self.subTest(case=i):
                folder = tempfile.mkdtemp(dir=self.dir)
                shutil.copy(self.base.path, folder)
                with self.assertRaises(FSR.RecordError):
                    FSR.load(write_record(folder, doc))
        # C3 measured at another duty than the executor's base duty
        doc = self.supplement(1, v_measured_m_s=dict(duty_pct=45, speed_m_s=.4))
        record = FSR.load(write_record(self.dir, doc))
        with self.assertRaises(FSR.RecordError):
            V.vehicle_config(record=record)

    def test_no_record(self):
        empty = tempfile.mkdtemp(prefix='field_records_empty_')
        self.addCleanup(shutil.rmtree, empty, True)
        with self.assertRaises(FSR.RecordError):
            FSR.newest(empty)


class RecordValidation(unittest.TestCase):
    """Every committed record (schema v1, v2 and v3 supplements) passes; each change below is refused, for every record
    of its schema."""

    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix='field_records_bad_')
        self.addCleanup(shutil.rmtree, self.dir, True)
        for record in FSR.records():                             # the bases of the supplements, unchanged
            shutil.copy(record.path, self.dir)

    def check(self, doc, stem):
        path = write_record(self.dir, doc, stem=stem)
        return FSR.executor_values(FSR.load(path), firmware_name=FIRMWARE, command_limit_deg=20., base_duty_pct=55)

    def each_record(self):
        for record in FSR.records():
            with self.subTest(record=record.record_id):
                yield record, yaml.safe_load(record.path.read_bytes())

    def test_the_committed_records_pass(self):
        for record, doc in self.each_record():
            self.check(doc, record.record_id)
            with self.assertRaises(FSR.RecordError):                    # another command scale than the executor's
                FSR.executor_values(record, firmware_name=FIRMWARE, command_limit_deg=30.)

    def test_refusals(self):
        def v(name):
            return lambda doc: doc['values'][name]

        def fw(doc):
            return doc['values']['firmware']['value']
        common = [
            lambda doc: v('mark_angle_deg')(doc).update(provenance='MEASURED'),                 # unknown tag
            lambda doc: v('mark_angle_deg')(doc).update(provenance='MEASURED_20261131_LIFTED'),  # no such date
            lambda doc: v('mark_angle_deg')(doc).update(source=''),
            lambda doc: v('mark_angle_deg')(doc).update(value=60.),
            lambda doc: v('esp32_cal')(doc)['value'].update(left=v('end_left')(doc)['value']['pot'] + 1),  # outside end
            lambda doc: v('esp32_cal')(doc)['value'].update(right=v('esp32_cal')(doc)['value']['center'] + 1),  # order
            lambda doc: v('esp32_cal')(doc)['value'].update(left=fw(doc)['guard_left'] + 1),     # beyond the guard
            lambda doc: v('polarity_checked')(doc).update(value=True),                           # true but NOT_MEASURED
            lambda doc: v('range_measured')(doc).update(provenance='USER_VISUAL'),               # false needs NOT_MEASURED
            lambda doc: (v('step_response')(doc)['value'].update(steps=[{'from': 0, 'to': 10, 'reached_s': 1.6}]),
                         doc['derive']['rate_rule'].update(value='slowest_full_range_step')),    # no full-range step
            lambda doc: v('step_response')(doc)['value']['steps'].append({'from': 5, 'to': 5, 'reached_s': 1.}),
            lambda doc: v('pot_noise_rest')(doc)['value'].update(p2p=3),
            lambda doc: doc['derive']['rate_fraction'].update(value=1.2),
            lambda doc: doc['derive']['rate_rule'].update(value='mean_of_all'),
            lambda doc: doc.update(measured_at='2026-11-16T22:13:00+09:00'),                     # file name differs
            lambda doc: doc.update(measured_at=doc['measured_at'][:19]),                          # no offset
            lambda doc: doc['conditions'].update(wheels='ground'),                               # kind lifted
            lambda doc: doc.update(extra=1),
            lambda doc: doc.update(schema='ubts.field_steer_record.v9'),
            lambda doc: fw(doc).update(name='OTHER-FW'),                                         # executor expects
        ]
        v2_only = [
            lambda doc: fw(doc).update(fw_tokens=[t for t in fw(doc)['fw_tokens'] if t != 'SMOOTH=1']),  # no token
            lambda doc: fw(doc).update(target_tol_counts=10),                                    # not the SM STOP_TOL
            lambda doc: fw(doc).update(drive_watchdog_ms=500),                                   # not DRIVE_WD=300
            lambda doc: fw(doc).update(center_guard=[1800, 2000]),                               # CENTER outside
            lambda doc: fw(doc).update(app_sha256='F448'),
            lambda doc: fw(doc).update(hold_mode='servo'),
            lambda doc: fw(doc).pop('sm'),
            lambda doc: v('step_response')(doc)['value'].update(scale='firmware_angle_right_plus'),
            lambda doc: v('step_response')(doc)['value'].update(timing='guess'),
            lambda doc: v('polarity_evidence')(doc)['value'].update(pot_increases_toward='right'),  # against the FW sign
            lambda doc: v('polarity_evidence')(doc)['value'].update(observations=[]),
            lambda doc: doc['values'].pop('polarity_evidence'),
        ]
        v3_common = [
            lambda doc: doc.update(base_record='field_steer_20200101_0000'),
            lambda doc: doc.update(base_record=doc['record_id']),
            lambda doc: doc.update(result_kind='field_lifted_standstill'),
            lambda doc: doc['conditions'].pop(FSR.V3_REQUIRED_CONDITIONS[doc['result_kind']][0]),
            lambda doc: doc['values'].update(colour=dict(value=1, provenance='DEFINITION', source='x')),
            lambda doc: doc.update(values={}),
            lambda doc: doc.update(measured_at='2026-11-17T23:13:00+09:00'),                 # file name differs
            lambda doc: doc.update(extra=1),
            lambda doc: next(iter(doc['values'].values())).update(provenance='OCTOBER'),
            lambda doc: next(iter(doc['values'].values())).update(source=''),
        ]
        v3_by_value = {
            'breakaway_duty_pct': [lambda doc: doc['values']['breakaway_duty_pct'].update(value=34.5)],
            'v_measured_m_s': [
                lambda doc: doc['values']['v_measured_m_s'].update(value=dict(duty_pct=55, speed_m_s=3.)),
                lambda doc: doc['values']['v_measured_m_s'].update(value=dict(duty_pct=45, speed_m_s=.4))],  # base duty
            'measured_stop_distance_m': [lambda doc: doc['values']['measured_stop_distance_m'].update(value=-1.)],
            'zero_offset_deg': [lambda doc: doc['values']['zero_offset_deg'].update(value=9.),
                                lambda doc: doc['values']['zero_offset_deg'].update(value='straight')],
        }
        refused, schemas = 0, set()
        for record, base in self.each_record():
            schemas.add(base['schema'])
            if base['schema'] == FSR.SCHEMA_V3:
                cases = v3_common + [c for name in base['values'] for c in v3_by_value.get(name, [])]
            else:
                cases = common + (v2_only if base['schema'] == FSR.SCHEMA_V2 else [])
            for i, change in enumerate(cases):
                doc = yaml.safe_load(record.path.read_bytes())
                change(doc)
                with self.assertRaises(FSR.RecordError, msg='case %d' % i):
                    self.check(doc, record.record_id)
                refused += 1
        self.assertEqual(schemas, set(FSR.SCHEMAS))                                     # every schema exercised
        self.assertGreaterEqual(refused, len(common) * 2 + len(v2_only) + len(v3_common))

    def test_rates_are_rounded_down(self):
        for record, doc in self.each_record():
            if doc['schema'] == FSR.SCHEMA_V3:
                continue                                         # a supplement carries no steps
            for step in doc['values']['step_response']['value']['steps']:
                if abs(step['to'] - step['from']) == 20:
                    step['reached_s'] = 3.0
            doc['values']['mark_angle_deg']['value'] = 17.999
            values, _ = self.check(doc, record.record_id)
            self.assertEqual(values['steer_measured_rate_deg_s'], math.floor(17.999 / 3.0 * 1000) / 1000)
            self.assertLessEqual(values['steer_rate_limit_deg_s'], .8 * values['steer_measured_rate_deg_s'])
            self.assertEqual(values['steer_rate_min_deg_s'], values['steer_rate_limit_deg_s'])


class SmoothFirmwareRecord(unittest.TestCase):
    """The SMOOTH firmware (FW line word SMOOTH=1, same firmware name) is a board the executor accepts: its records load,
    their values reach the configuration, and the stop tolerance of the smooth hold is the SM STOP_TOL."""

    def test_smooth_records(self):
        smooth = [r for r in FSR.records() if r.schema == FSR.SCHEMA_V2 and r.value('firmware')['hold_mode'] == 'smooth']
        self.assertTrue(smooth)                                          # records are never removed
        XC = V._row_executor_config()
        from row_executor import bridge_client as B
        for record in smooth:
            fw = record.value('firmware')
            self.assertIn('SMOOTH=1', fw['fw_tokens'])
            self.assertEqual(fw['name'], B.ESP32_FIRMWARE)
            self.assertEqual(fw['target_tol_counts'], fw['sm']['STOP_TOL'])
            evidence = record.value('polarity_evidence')
            self.assertEqual(evidence['pot_increases_toward'], 'left')     # POT_DECREASES toward the right
            self.assertIs(record.value('polarity_checked'), False)          # evidence is not the B0 check
            values, sources = FSR.executor_values(record, firmware_name=B.ESP32_FIRMWARE,
                                                  command_limit_deg=XC.ESP32_COMMAND_LIMIT_DEG)
            cfg = V.vehicle_config(record=record)
            self.assertEqual(cfg.steer_esp32_cal, values['steer_esp32_cal'])
            self.assertIn('smooth hold', sources['steer_servo_deadband_deg'])


class ShadowLoop(unittest.TestCase):
    def test_shadow_loop_has_no_bridge_transport(self):
        import sys
        V._row_executor_config()                    # tools/ on the path
        from row_executor import executor_main as M
        cfg = replace(V.vehicle_config(), tuning_file=None)
        loop = M.ExecutorLoop(cfg, motor_output_flag=False)
        self.assertFalse(loop.motor_allowed)
        self.assertIsNone(loop._stop_transport)
        self.assertIsNone(loop.esp32)
        self.assertEqual(loop._req_arm(0.)['error'], 'motor_output_not_allowed')
        self.assertIn('row_executor', sys.modules)


if __name__ == '__main__':
    unittest.main()
