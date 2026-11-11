"""Host profile export (steering.host_profile_export): values, provenance, file bytes and the host loader.

unittest style (runs under pytest and under plain unittest on Python 3.10). Files only in a temporary directory.
"""
import contextlib
import importlib
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest

from steering import geometry
from steering import host_profile_export as hpe
from steering import vehicle_profile as vp

DASHBOARD = Path(__file__).resolve().parents[2] / 'dashboard'


def host_loader():
    """steer_bridge_core from this work tree's dashboard (the host never imports sim code; this test may).

    An environment can already have another 'scripts' package on its path (the SIM image has an older dashboard
    copy). The host package is imported with this tree's dashboard first, then sys.path and the previous
    'scripts*' modules are restored, so other tests keep what they had.
    """
    if not (DASHBOARD / 'scripts' / 'steer_bridge_core.py').is_file():
        return None
    saved_path = list(sys.path)
    saved = {name: module for name, module in sys.modules.items() if name == 'scripts' or name.startswith('scripts.')}
    for name in saved:
        del sys.modules[name]
    try:
        sys.path.insert(0, str(DASHBOARD))
        return importlib.import_module('scripts.steer_bridge_core')
    finally:
        sys.path[:] = saved_path
        for name in [n for n in sys.modules if n == 'scripts' or n.startswith('scripts.')]:
            del sys.modules[name]
        sys.modules.update(saved)


class ExportValuesTest(unittest.TestCase):
    def setUp(self):
        self.profile = vp.load_profile()
        self.data = hpe.build(self.profile)

    def test_identity_and_result_label(self):
        source = self.data['source_profile']
        self.assertEqual(self.data['schema'], 'ubts.steer_host_profile.v1')
        self.assertEqual((self.data['vehicle_profile'], self.data['hw_profile']), ('front_steer', 'front_steer_v0'))
        self.assertEqual((source['profile_id'], source['profile_sha256']), (self.profile.profile_id, self.profile.sha256))
        self.assertEqual(source['provenance_grade'], 'placeholder')
        self.assertEqual(source['result_kind'], vp.RESULT_LABEL_PLACEHOLDER)
        self.assertIs(source['field_validated'], False)
        self.assertIsNone(source['overrides'])

    def test_limits_and_calibration_table(self):
        geom = geometry.SteeringGeometry.from_profile(self.profile)
        soft_left, soft_right = geom.soft_limits_eq()
        mech_left, mech_right = geom.mech_limits_eq()
        lim, cal = self.data['steer_limit'], self.data['calibration']
        self.assertEqual((lim['left_rad'], lim['right_rad']), (soft_left, soft_right))
        self.assertEqual((lim['left_mrad'], lim['right_mrad']), (409, 409))
        self.assertLessEqual(lim['left_mrad'], soft_left * 1000.0)
        self.assertAlmostEqual(math.degrees(soft_left), self.profile.value('steer_soft_limit_eq_left_deg'), places=9)
        self.assertAlmostEqual(cal['mark_left_mrad'], (mech_left - math.radians(1.0)) * 1000.0, places=9)
        self.assertAlmostEqual(cal['mark_right_mrad'], (mech_right - math.radians(1.0)) * 1000.0, places=9)
        self.assertAlmostEqual(cal['permille_per_deg'], 39.8449, places=3)
        self.assertEqual(cal['expected_soft_permille'], {'left': 935, 'right': -935})   # spec 9.6 example record
        self.assertEqual(cal['cal_id'], 1)

    def test_link_stop_and_fault_values(self):
        d = self.data
        self.assertEqual(d['link'], {'baud': 115200, 'status_hz': 20, 'c_ttl_max_ms': 300, 'c_resend_hz': 20.0,
                                     'ping_hz': 10.0, 'watchdog_ms': 500, 'banner_timeout_s': 1.5})
        self.assertEqual(d['stop'], {'confirm_periods': 2, 'resend_interval_ms': 50.0, 'resend_max': 5,
                                     'steer_policy': 'hold_measured', 'hold_after_stop_ms': 2000,
                                     'hold_after_disarm_ms': 2000, 'hold_after_loss_ms': 2000,
                                     'watchdog_steer_mode': 'hold', 'ttl_expiry_drive_mode': 'brake'})
        self.assertEqual(d['arm'], {'confirm_ms': 150.0})
        self.assertEqual(d['host_faults']['status_gap_ms'], 150.0)
        self.assertEqual((d['host_faults']['seq_gap_missing'], d['host_faults']['seq_gap_window_s']), (5, 1.0))
        self.assertEqual((d['host_faults']['track_error_deg'], d['host_faults']['track_duration_s']), (3.0, 1.5))
        self.assertEqual(d['host_faults']['ineffective'], {'min_speed_m_s': 0.2, 'min_steer_deg': 3.0,
                                                           'rate_error_deg_s': 3.0, 'rate_error_frac': 0.5,
                                                           'duration_s': 1.5})
        self.assertEqual(d['servo'], {'deadband_permille': 20, 'max_pwm': 255})   # spec 4.10 db 20, maxp 255
        self.assertEqual(d['drive'], {'lr_mode': 'equal_duty'})
        self.assertEqual(d['vehicle'], {'wheelbase_m': 0.76})

    def test_provenance_covers_every_value(self):
        prov = self.data['provenance']
        for section in ('steer_limit', 'calibration', 'link', 'stop', 'arm', 'host_faults', 'servo', 'drive',
                        'vehicle'):
            for key in self.data[section]:
                with self.subTest(section=section, key=key):
                    self.assertTrue(section in prov or '%s.%s' % (section, key) in prov)
        for entry in prov.values():
            if 'profile_names' in entry:
                for name in entry['profile_names']:
                    self.assertIn(name, vp.PARAMETER_NAMES)
                for tag in entry['tags']:
                    vp.parse_provenance(tag)
            else:
                self.assertTrue(entry['document'])

    def test_options_and_refusals(self):
        self.assertIsNone(hpe.build(self.profile, cal_id=None)['calibration']['cal_id'])
        self.assertEqual(hpe.build(self.profile, cal_id=7)['calibration']['cal_id'], 7)
        for bad in (0, 65536, True, 1.0, '1'):
            with self.assertRaises(hpe.ExportError):
                hpe.build(self.profile, cal_id=bad)
        narrow = self.profile.with_overrides({'steer_mech_limit_deg': {key: 22.0 for key in geometry.WHEEL_LIMIT_KEYS}},
                                             label='narrow')
        data = hpe.build(narrow)
        self.assertLess(data['steer_limit']['left_mrad'], 409)
        self.assertEqual(data['source_profile']['overrides']['label'], 'narrow')
        with self.assertRaises(hpe.ExportError):
            hpe.build(self.profile.with_overrides({'drive_lr_mode': 'electronic_diff'}, label='diff'))
        with self.assertRaises(hpe.ExportError):
            hpe.build(self.profile.with_overrides({'c_ttl_max_ms': 300.5}, label='odd_ttl'))
        # 10 Hz status lines with the fixed 150 ms host status gap: each value is in its range, together they would
        # latch steering_status_stale on one late line (the host loader refuses the same, 2026-11-09 review).
        with self.assertRaises(hpe.ExportError) as caught:
            hpe.build(self.profile.with_overrides({'status_hz': 10}, label='slow_status'))
        self.assertIn('two status periods', str(caught.exception))


class ExportFileTest(unittest.TestCase):
    def test_bytes_are_canonical(self):
        profile = vp.load_profile()
        raw = hpe.to_bytes(hpe.build(profile))
        self.assertEqual(raw, hpe.to_bytes(hpe.build(vp.load_profile())))
        self.assertNotIn(b'\r', raw)
        self.assertTrue(raw.endswith(b'}\n'))
        self.assertFalse(raw.startswith(b'\xef\xbb\xbf'))
        data = json.loads(raw.decode('utf-8'))
        self.assertEqual(list(data), sorted(data))
        self.assertEqual(hpe.to_bytes(data), raw)
        self.assertEqual(len(hpe.sha256(raw)), 64)

    def test_main_writes_the_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, 'host_profile.json')
            printed = io.StringIO()
            with contextlib.redirect_stdout(printed):
                self.assertEqual(hpe.main(['--out', out, '--cal-id', 'any']), 0)
            report = json.loads(printed.getvalue())
            raw = Path(out).read_bytes()
            self.assertEqual(report['sha256'], hpe.sha256(raw))
            self.assertEqual(report['bytes'], len(raw))
            self.assertIsNone(json.loads(raw)['calibration']['cal_id'])

    def test_the_host_loader_accepts_the_export(self):
        loader = host_loader()
        if loader is None:
            self.skipTest('dashboard/scripts/steer_bridge_core.py not found next to sim/ubts')
        raw = hpe.to_bytes(hpe.build(vp.load_profile()))
        hp = loader.HostProfile.from_bytes(raw, origin='export test')
        self.assertEqual(hp.sha256, hpe.sha256(raw))
        self.assertEqual((hp.limit_left_mrad, hp.limit_right_mrad), (409, 409))
        self.assertEqual((hp.permille_from_mrad(409), hp.permille_from_mrad(-409)), (934, -934))
        self.assertLessEqual(hp.permille_from_mrad(hp.limit_left_mrad), hp.expected_soft[0])
        self.assertGreaterEqual(hp.permille_from_mrad(-hp.limit_right_mrad), hp.expected_soft[1])
        self.assertEqual(hp.identity()['profile_sha256'], vp.load_profile().sha256)


if __name__ == '__main__':
    unittest.main()
