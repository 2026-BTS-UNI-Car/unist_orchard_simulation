"""The vehicle executor configuration of the Jetson kit (steering.vehicle_executor_config, steering.field_steer_record;
docs/steering/EXECUTOR_ESP32_KO.md 3).

Checks: the committed file tools/row_executor/config.vehicle_esp32_shadow.json is the generator's output for the profile
front_steer_v1_spec_20261116 and the NEWEST field steering record (steering/field_records/); it is a SHADOW front_steer
single-row configuration for the ESP32-S3 bridge on the vehicle socket with the operator page on 127.0.0.1; the vehicle
values are the profile's and the board values are the newest record's (no record value is written here as a literal: a
newer record supersedes the older one and these tests follow it); the fixed rules hold for any record (rate limit and
GO threshold at most 0.8 x the measured rate, also on the board's command scale; polarity and range not confirmed; drive
measurements null); a newer record wins and older ones are kept; malformed records are refused; without a record the
profile values come back; it builds a SHADOW executor loop that never creates a bridge transport.
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
DRIVE_AND_OPEN = ('breakaway_duty_pct', 'rolling_breakaway_duty_pct', 'v_measured_m_s', 'measured_stop_distance_m',
                  'measured_yaw_at_max_diff_deg_s', 'steer_wheel_soft_margin_deg', 'steer_min_pwm_pct',
                  'steer_zero_offset_deg')


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


def slowest_full_step_s(record):
    full = record.value('esp32_cal')['max_command']
    return max(s['reached_s'] for s in record.value('step_response')['steps'] if abs(s['to'] - s['from']) == full)


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
        self.assertAlmostEqual(cfg.steer_measured_rate_deg_s, mark / slowest_full_step_s(record), delta=1e-3)
        self.assertEqual(cfg.steer_measured_rate_standstill_deg_s, cfg.steer_measured_rate_deg_s)
        self.assertAlmostEqual(cfg.steer_rate_limit_deg_s, record.value('rate_fraction') * cfg.steer_measured_rate_deg_s,
                               delta=1e-3)
        self.assertEqual(cfg.steer_rate_min_deg_s, cfg.steer_rate_limit_deg_s)
        counts = min(cal['center'] - cal['right'], cal['left'] - cal['center']) / cal['max_command']
        self.assertAlmostEqual(cfg.steer_servo_deadband_deg,
                               record.value('firmware')['target_tol_counts'] / counts * mark / cal['max_command'], places=3)
        self.assertEqual(cfg.steer_polarity_checked, record.value('polarity_checked'))
        self.assertEqual(cfg.steer_range_measured, record.value('range_measured'))

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
        board_rate = record.value('esp32_cal')['max_command'] / slowest_full_step_s(record)
        self.assertLessEqual(cfg.steer_rate_limit_deg_s * XC.esp32_scale(cfg), fraction * board_rate + 1e-9)
        # not confirmed yet: polarity (B0), range with an angle meter (B2 / B3), drive direction; drive values null
        self.assertIs(cfg.steer_polarity_checked, False)
        self.assertIs(cfg.steer_range_measured, False)
        self.assertIs(record.value('drive_direction_verified'), False)
        for name in DRIVE_AND_OPEN:
            self.assertIsNone(getattr(cfg, name), name)
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
            FSR.executor_values(record, firmware_name=FIRMWARE, command_limit_deg=20.)
        sources = V.vehicle_values(self.profile)[1]
        self.assertEqual(sources['field_record']['id'], self.record.record_id)
        self.assertEqual(sources['field_record']['sha256'], self.record.sha256)


class NewestRecordWins(unittest.TestCase):
    def setUp(self):
        self.base = FSR.newest()
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

    def test_no_record(self):
        empty = tempfile.mkdtemp(prefix='field_records_empty_')
        self.addCleanup(shutil.rmtree, empty, True)
        with self.assertRaises(FSR.RecordError):
            FSR.newest(empty)


class RecordValidation(unittest.TestCase):
    def setUp(self):
        self.base = FSR.newest()
        self.dir = tempfile.mkdtemp(prefix='field_records_bad_')
        self.addCleanup(shutil.rmtree, self.dir, True)

    def check(self, doc):
        path = write_record(self.dir, doc, stem=self.base.record_id)
        return FSR.executor_values(FSR.load(path), firmware_name=FIRMWARE, command_limit_deg=20.)

    def test_the_committed_record_passes(self):
        self.check(yaml.safe_load(self.base.path.read_bytes()))
        with self.assertRaises(FSR.RecordError):                    # another command scale than the executor's
            FSR.executor_values(self.base, firmware_name=FIRMWARE, command_limit_deg=30.)

    def test_refusals(self):
        def v(name):
            return lambda doc: doc['values'][name]
        cases = [
            lambda doc: v('mark_angle_deg')(doc).update(provenance='MEASURED'),                 # unknown tag
            lambda doc: v('mark_angle_deg')(doc).update(provenance='MEASURED_20261131_LIFTED'),  # no such date
            lambda doc: v('mark_angle_deg')(doc).update(source=''),
            lambda doc: v('mark_angle_deg')(doc).update(value=60.),
            lambda doc: v('esp32_cal')(doc)['value'].update(left=v('end_left')(doc)['value']['pot'] + 1),  # outside end
            lambda doc: v('esp32_cal')(doc)['value'].update(right=2000),                         # order
            lambda doc: v('esp32_cal')(doc)['value'].update(left=2600),                          # beyond the guard
            lambda doc: v('polarity_checked')(doc).update(value=True),                           # true but NOT_MEASURED
            lambda doc: v('range_measured')(doc).update(provenance='USER_VISUAL'),               # false needs NOT_MEASURED
            lambda doc: v('step_response')(doc)['value'].update(steps=[{'from': 0, 'to': 10, 'reached_s': 1.6}]),
            lambda doc: v('pot_noise_rest')(doc)['value'].update(p2p=3),
            lambda doc: doc['derive']['rate_fraction'].update(value=1.2),
            lambda doc: doc['derive']['rate_rule'].update(value='mean_of_all'),
            lambda doc: doc.update(measured_at='2026-11-16T22:13:00+09:00'),                     # file name differs
            lambda doc: doc.update(measured_at='2026-11-16T22:12:00'),                           # no offset
            lambda doc: doc['conditions'].update(wheels='ground'),                               # kind lifted
            lambda doc: doc.update(extra=1),
            lambda doc: v('firmware')(doc)['value'].update(name='OTHER-FW'),                     # executor expects
        ]
        for i, change in enumerate(cases):
            with self.subTest(case=i):
                doc = yaml.safe_load(self.base.path.read_bytes())
                change(doc)
                with self.assertRaises(FSR.RecordError):
                    self.check(doc)

    def test_rates_are_rounded_down(self):
        doc = yaml.safe_load(self.base.path.read_bytes())
        for step in doc['values']['step_response']['value']['steps']:
            if abs(step['to'] - step['from']) == 20:
                step['reached_s'] = 3.0
        doc['values']['mark_angle_deg']['value'] = 17.999
        values, _ = self.check(doc)
        self.assertEqual(values['steer_measured_rate_deg_s'], math.floor(17.999 / 3.0 * 1000) / 1000)
        self.assertLessEqual(values['steer_rate_limit_deg_s'], .8 * values['steer_measured_rate_deg_s'])
        self.assertEqual(values['steer_rate_min_deg_s'], values['steer_rate_limit_deg_s'])


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
