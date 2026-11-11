"""Identification run in the IdentRig (steering/ident, design 3.9): a short bench phase (B7, B0 polarity, B2 marks and
CAL SAVE, B3 static table) through the bench tool, then the drive mode with the B3 calibration and a standstill turn
through the control API (F5 style), checked from the records the analysis reads.

Needs the native steering program with firmware mode (built into UBTS_STEER_BUILD_DIR or a temporary folder, never
into the work tree); skipped with the reason otherwise. Placeholder-based SIM results (자리값 기반 SIM 결과), not field
validation. unittest style: pytest (3.12, 3.13) and `python3 -m unittest test_steering_ident_rig` (3.10).
"""
import atexit
import json
import math
import os
import shutil
import tempfile
import unittest
from pathlib import Path

from steering import t1_rig as tr
from steering.native import build as native_build
from steering.vehicle_profile import load_profile
from steering.ident import analysis as an
from steering.ident import instruments
from steering.ident import procedures as pr
from steering.ident import rig as ir
from steering.ident import yard as yard_mod

_STATE = {}


def _cleanup(path):
    shutil.rmtree(path, ignore_errors=True)


def native_path():
    if 'native' not in _STATE:
        try:
            if not native_build.firmware_present():
                raise native_build.NativeBuildError(native_build.missing_firmware_message())
            native_build.find_compiler()
            folder = os.environ.get(native_build.ENV_BUILD_DIR)
            if not folder:
                folder = tempfile.mkdtemp(prefix='steer_ident_build_')
                atexit.register(_cleanup, folder)
            _STATE['native'] = native_build.ensure_built(build_dir=folder).path
        except native_build.NativeBuildError as exc:
            _STATE['native'] = 'native steering program not available: %s' % exc
    value = _STATE['native']
    if not os.path.isfile(value):
        if native_build.firmware_required():
            raise AssertionError('%s is set: %s' % (native_build.ENV_REQUIRE_FIRMWARE, value))
        raise unittest.SkipTest(value)
    return value


class IdentRigRun(unittest.TestCase):
    def setUp(self):
        try:
            tr.host_modules()
        except tr.RigError as exc:
            raise unittest.SkipTest(str(exc))
        native_path()
        self.tmp = tempfile.mkdtemp(prefix='steer_ident_rig_')
        self.addCleanup(_cleanup, self.tmp)

    def test_bench_phase_then_standstill_turn_from_the_records(self):
        profile = load_profile()
        plan = pr.Plan(b3_points_permille=(-560, -190, 190, 560))
        config = tr.RigConfig(seed=3, stream_key='ident_test', world=yard_mod.YardSpec(), start_pose=pr.START_POSE,
                              profile=profile, eeprom_blank=True, trace_every=5, serial_log=False)
        rig = ir.IdentRig(config, native_executable=native_path())
        self.addCleanup(rig.close)
        root = Path(self.tmp) / 'records'
        nominal, copy = pr.nominal_profile_copy()
        b1 = instruments.Tape(3, 'ident_test').measure(rig.profile)
        bench = pr.BenchSession(rig, root, 'ident_t', copy, b1, instruments.AngleGauge(3, 'ident_test'), nominal,
                                plan)
        bench.b7('B7_cold', 1.)
        polarity = bench.b0()
        cal = bench.b2()
        calibration = bench.b3()
        bench.b7_load(2.)
        self.assertTrue(cal['cal_id'])
        self.assertFalse(cal['caldirty'])
        b7 = an.b7_noise({'cold': str(root / bench.records['B7_cold']), 'load': str(root / bench.records['B7_load'])})
        self.assertGreater(b7['load']['samples'], 10)
        self.assertGreater(b7['servo_load']['samples'], 10)
        for key in ('mark_left_deg', 'mark_right_deg'):              # placeholder end stops about 26 deg
            self.assertGreater(calibration[key], 20.)
            self.assertLess(calibration[key], 35.)
        b0 = an.b0_polarity(str(root / bench.records['B0']), str(root / bench.records['B2']))
        self.assertTrue(b0['opposite'], b0)                       # CAL JOG + and - move the potentiometer apart
        self.assertTrue(b0['en_cut']['output_off'], b0)           # STOP during a JOG ends the output
        self.assertTrue(b0['cal_pol']['ok'], b0)
        self.assertEqual(set(polarity), {1, -1})
        # the bench record: serial log with both directions, status rows, gauge readings, B1 tape lengths
        b3 = an.BenchData(root / bench.records['B3'])
        self.assertEqual(len(b3.events('reading')), 8)
        self.assertTrue(any(r['dir'] == 'h2m' for r in b3.serial) and any(r['dir'] == 'm2h' for r in b3.serial))
        self.assertGreater(len(b3.pos), 50)
        self.assertEqual(set(b3.b1()), set(instruments.B1_NAMES))
        # drive mode with the host calibration from B2 / B3, a standstill turn through the control API
        host_cal = dict(cal_id=int(cal['cal_id']), mark_left_mrad=calibration['mark_left_mrad'],
                        mark_right_mrad=calibration['mark_right_mrad'],
                        permille_per_deg=calibration['permille_per_deg'],
                        expected_soft_permille={'left': int(cal['soft_l']), 'right': int(cal['soft_r'])})
        rig.enter_drive_mode(calibration=host_cal)
        drive = pr.DriveSession(rig, root, 'ident_t', copy, dict(host_cal, b3=calibration), plan)
        drive.ready()
        rec = drive.start('F5', lidar=False)
        drive.arm()
        drive.hold(.5, 0, 0.)
        seconds = drive.turn_standstill(8.)
        self.assertLess(seconds, 30.)
        drive.hold(1., 0, 8.)
        drive.release(1.)
        drive.finish('F5', rec)
        rid = drive.records['F5']
        rows = [json.loads(line) for line in (root / '_steering' / rid / 'steering.jsonl').read_text(
            encoding='utf-8').splitlines()]
        armed = [r for r in rows if r.get('state') == 'ARMED' and r.get('angle_mrad') is not None]
        self.assertGreater(len(armed), 20)
        self.assertLess(abs(armed[-1]['angle_mrad'] / 1000. - math.radians(8.)), math.radians(1.2))
        self.assertFalse([r for r in rows if 'steering_not_following' in (r.get('host_faults') or [])])
        desc = json.loads((root / '_steering' / rid / 'descriptor.json').read_text(encoding='utf-8'))
        self.assertEqual(desc['frame_count'], len(rows))
        self.assertEqual(desc['calibration']['cal_id'], host_cal['cal_id'])
        out = an.f4_f5_steering(root, rid, host_cal)
        self.assertGreater(out['rows'], 20)
        bad = {'POLARITY', 'STALL', 'TRACK', 'THERMAL', 'DRV_STUCK', 'steering_not_following'}
        self.assertFalse(bad & set(out['faults']), out['faults'])


if __name__ == '__main__':
    unittest.main()
