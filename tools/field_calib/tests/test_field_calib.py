"""field_calib tests (drive parts): readers, timeline, ICP odometry, ladder rule, end-to-end drive analysis.

Runs under pytest and plain unittest (python3 -B -m unittest discover -s tools/field_calib/tests).
Synthetic data only (calib_testlib.py); the real-record validation is validate_drive_evidence.py.
Front-steer branch: the 546fa8c tests of the differential C5 arc, the C6 pivot, the config recommendation and the
field_quick_calib CLI are not imported (those modules are not imported); the end-to-end run uses drive_calib.py.
"""
import gzip
import json
import math
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import calib_testlib as T   # noqa: E402  (sets sys.path to tools/)

import numpy as np          # noqa: E402

from field_calib import drive_calib as cli         # noqa: E402
from field_calib import odometry as odo            # noqa: E402
from field_calib import records, steps             # noqa: E402
from field_calib import timeline as tl             # noqa: E402

RID = '20261024T000000Z_both'


def _scipy_reason():
    """None when scipy's cKDTree imports (the LiDAR odometry needs it), else the skip reason."""
    try:
        from scipy.spatial import cKDTree  # noqa: F401
    except Exception as exc:                   # a broken install raises more than ImportError (numpy ABI mismatch)
        return 'scipy not usable here (the LiDAR odometry needs cKDTree): %s: %s' % (type(exc).__name__, exc)
    return None


SCIPY_SKIP = _scipy_reason()


class _Session:
    """One synthetic session for the whole module (generation ~7 s, analysis ~5 s)."""
    root = None
    out = None
    result = None

    @classmethod
    def get(cls):
        if SCIPY_SKIP:
            raise unittest.SkipTest(SCIPY_SKIP)
        if cls.root is None:
            cls.root = tempfile.mkdtemp(prefix='fieldcal_')
            T.write_recording(cls.root, RID, T.SESSION)
            cls.out = os.path.join(cls.root, 'out.json')
            code = cli.main(['--root', cls.root, '--latest', '1', '--workers', '2', '--quiet', '--out', cls.out])
            with open(cls.out, 'r', encoding='utf-8') as fh:
                cls.result = json.load(fh)
            cls.code = code
        return cls


def tearDownModule():
    if _Session.root:
        shutil.rmtree(_Session.root, ignore_errors=True)


def _motor(rows):
    return records.Motor([dict(t=t, req_l=l, req_r=r, rep_l=a, rep_r=b, state=st, armed=st == 'armed_manual',
                               moving=bool(l or r), stale=False) for t, l, r, a, b, st in rows])


class TimelineTests(unittest.TestCase):
    def test_ramp_start_from_partial_report(self):
        m = _motor([(0., 0, 0, 0., 0., 'armed_manual'), (.5, 0, 0, 0., 0., 'armed_manual'),
                    (1., 50, 50, 30., 30., 'armed_manual'), (1.5, 50, 50, 50., 50., 'armed_manual')])
        run = [x for x in tl.request_runs(m) if x['l'] == 50][0]
        t0, precise, _ = tl.ramp_start_from_rest(m, run)
        self.assertTrue(precise)
        self.assertAlmostEqual(t0, .7, places=6)          # 30 % reached 0.3 s after the ramp start

    def test_ramp_start_complete_is_bounded_by_previous_status(self):
        m = _motor([(0., 0, 0, 0., 0., 'armed_manual'), (.5, 0, 0, 0., 0., 'armed_manual'),
                    (1., 20, 20, 20., 20., 'armed_manual')])
        run = [x for x in tl.request_runs(m) if x['l'] == 20][0]
        t0, precise, _ = tl.ramp_start_from_rest(m, run)
        self.assertFalse(precise)
        self.assertAlmostEqual(t0, .8, places=6)          # max(1.0 - 0.2, 0.5)

    def test_fade_start(self):
        m = _motor([(0., 60, 60, 60., 60., 'armed_manual'), (.5, 0, 0, 25., 25., 'armed_manual'),
                    (1., 0, 0, 0., 0., 'armed_manual')])
        t, _ = tl.fade_start(m, 1, 60.)
        self.assertAlmostEqual(t, .15, places=6)          # 35 %p faded at 100 %/s
        m2 = _motor([(0., 60, 60, 60., 60., 'armed_manual'), (.5, 0, 0, 0., 0., 'disarmed')])
        self.assertIsNone(tl.fade_start(m2, 1, 60.)[0])

    def test_windows_cover_commands_and_standstill(self):
        rows = [(i * .5, 0, 0, 0., 0., 'armed_manual') for i in range(60)]                 # 30 s still
        rows += [(30. + i * .5, 50, 50, 50., 50., 'armed_manual') for i in range(10)]
        rows += [(35. + i * .5, 0, 0, 0., 0., 'armed_manual') for i in range(120)]         # 60 s still
        m = _motor(rows)
        w = tl.interest_windows(m)
        self.assertTrue(any(a <= 30. and b >= 39.5 for a, b in w))
        self.assertTrue(all(b - a <= 75. for a, b in w))
        self.assertLess(sum(b - a for a, b in w), 95.)
        self.assertEqual(len(tl.still_windows(m, min_s=10.)), 2)
        self.assertEqual(tl.classify(40, 60), 'arc_left_fwd')
        self.assertEqual(tl.classify(-60, 60), 'pivot_left')


class WanderWindowTests(unittest.TestCase):
    def test_window_offsets_straight_and_arc(self):
        from field_calib import steps
        x = np.linspace(0., 20., 201)
        self.assertLess(max(steps.window_offsets(np.column_stack([x, 0. * x]))), 1e-9)
        k = math.radians(1.)                                   # 1 deg/m heading drift
        th = k * x
        pts = np.column_stack([np.cumsum(np.cos(th)) * .1, np.cumsum(np.sin(th)) * .1])
        w = steps.window_offsets(pts)
        self.assertEqual(len(w), 16)                           # 5 m windows starting 0, 1, ..., 15 m on 20 m
        self.assertAlmostEqual(float(np.median(w)), 25. * k / 8., delta=.005)   # sagitta L^2 k / 8


@unittest.skipIf(SCIPY_SKIP, SCIPY_SKIP or '')
class OdometryTests(unittest.TestCase):
    def test_icp_recovers_known_translation_and_rotation(self):
        posts, segs = T.scene()
        rng = np.random.default_rng(3)
        n = 40
        rows = []
        for i in range(n):
            x, h = .045 * i, math.radians(.3 * i)
            _, r = T.raycast(x, 0., h, posts, segs, rng)
            mm = np.zeros(T.BEAMS, dtype=np.uint16)
            ok = np.isfinite(r)
            mm[ok] = np.round(r[ok] * 1000).astype(np.uint16)
            rows.append(mm)
        pose, quality, chain = odo.odometry(np.stack(rows), -math.pi, 2 * math.pi / T.BEAMS, [(0, n)], workers=1,
                                            chunk_scans=15)
        self.assertTrue((chain == 0).all())                               # chunks stitched into one chain
        dist = float(np.hypot(*(pose[-1, :2] - pose[0, :2])))
        self.assertAlmostEqual(dist, .045 * (n - 1), delta=.03)
        self.assertAlmostEqual(math.degrees(pose[-1, 2] - pose[0, 2]), .3 * (n - 1), delta=.4)


class ReaderTests(unittest.TestCase):
    def test_jetson_layout_and_windows(self):
        s = _Session.get()
        rec = records.open_archive(s.root, RID, scan_windows_fn=lambda m: tl.interest_windows(m))
        self.assertEqual(rec.layout, 'jetson')
        self.assertEqual(rec.geometry['beams'], T.BEAMS)
        self.assertGreater(rec.scan_total, len(rec.scan_t) - 1)
        self.assertTrue(np.all(np.diff(rec.imu[:, 0]) >= 0))
        self.assertAlmostEqual(float(np.nanmax(rec.motor.rep_l)), 60., delta=1.)
        self.assertAlmostEqual(float(np.nanmin(rec.motor.rep_l)), -60., delta=1.)
        self.assertIn('sha256', rec.sources['motor.jsonl'])
        self.assertEqual(rec.warnings, [])

    def test_full_extract_layout_gives_the_same_arrays(self):
        s = _Session.get()
        d = tempfile.mkdtemp(prefix='fieldcal_full_')
        try:
            src = os.path.join(s.root, RID)
            dst = os.path.join(d, RID)
            os.makedirs(dst)
            shutil.copy(os.path.join(src, 'motor.jsonl'), dst)
            shutil.copy(os.path.join(src, 'metadata.json'), dst)
            with gzip.open(os.path.join(src, 'mapping_source', 'sensors.jsonl.gz'), 'rt', encoding='utf-8') as fi, \
                    gzip.open(os.path.join(dst, 'sensors.jsonl.gz'), 'wt', encoding='utf-8') as fo:
                for line in fi:          # 10/3 extracts: {"topic", "bag_time_ns", "message"} order, header-only IMU
                    r = json.loads(line)
                    m = r['message']
                    if r['topic'] == '/imu/data_raw':
                        m = {k: v for k, v in m.items() if k not in ('stamp', 'frame_id')}
                    fo.write(json.dumps(dict(topic=r['topic'], bag_time_ns=r['bag_time_ns'], message=m),
                                        separators=(',', ':')) + '\n')
            a = records.open_archive(s.root, RID, hashes=False)
            b = records.open_archive(d, RID, hashes=False)
            self.assertEqual(b.layout, 'full')
            np.testing.assert_array_equal(a.ranges_mm, b.ranges_mm)
            np.testing.assert_allclose(a.imu, b.imu)
            np.testing.assert_allclose(a.scan_t, b.scan_t)
        finally:
            shutil.rmtree(d, ignore_errors=True)

    def test_truncated_sidecar_is_read_up_to_the_cut(self):
        s = _Session.get()
        d = tempfile.mkdtemp(prefix='fieldcal_cut_')
        try:
            dst = os.path.join(d, RID)
            shutil.copytree(os.path.join(s.root, RID), dst)
            path = os.path.join(dst, 'mapping_source', 'sensors.jsonl.gz')
            with open(path, 'rb') as fh:
                data = fh.read()
            with open(path, 'wb') as fh:
                fh.write(data[:len(data) // 2])
            os.remove(os.path.join(dst, 'mapping_source', 'manifest.json'))
            rec = records.open_archive(d, RID, hashes=False)
            self.assertTrue(any(w.startswith('sidecar_truncated') for w in rec.warnings))
            self.assertTrue(any('manifest_missing' in w for w in rec.warnings))
            self.assertGreater(len(rec.scan_t), 50)
        finally:
            shutil.rmtree(d, ignore_errors=True)

    def test_missing_motor_timeline_is_refused(self):
        d = tempfile.mkdtemp(prefix='fieldcal_nomotor_')
        try:
            os.makedirs(os.path.join(d, 'x', 'mapping_source'))
            with gzip.open(os.path.join(d, 'x', 'mapping_source', 'sensors.jsonl.gz'), 'wt') as fh:
                fh.write('')
            with self.assertRaises(records.RecordingError):
                records.open_archive(d, 'x')
        finally:
            shutil.rmtree(d, ignore_errors=True)


class EndToEndTests(unittest.TestCase):
    """Synthetic plant truth (calib_testlib): B 25 %, v = 0.0098 d - 0.04, gain 0.15, pivot 15 deg/s, bias 0.02."""

    @classmethod
    def setUpClass(cls):
        cls.s = _Session.get()
        cls.r = cls.s.result

    def test_axis_and_sign(self):
        self.assertAlmostEqual(self.r['axis']['axis_deg'], -179.08, delta=.5)
        self.assertEqual(self.r['lidar_imu']['sign'], 1.)

    def test_c1_bias(self):
        c1 = self.r['c1']
        self.assertGreaterEqual(c1['n_windows'], 1)
        self.assertAlmostEqual(c1['yaw_bias_dps'], T.BIAS_DPS, delta=.01)
        self.assertLess(c1['yaw_std_dps'], .1)

    def test_c2_ladder_brackets_breakaway(self):
        lad = self.r['c2']['labels']['all']['ladder']
        self.assertEqual(lad['highest_conclusive_fail_pct'], 20)
        self.assertEqual(lad['b_fwd_pct'], 25)
        self.assertTrue(lad['bracketed'])

    def test_c3_speed_curve(self):
        tab = self.r['c3']['table']
        for d in (40, 45, 50):
            self.assertAlmostEqual(tab[str(d)]['v_p10_p50_p90'][1], T.v_of(d), delta=.03, msg=str(d))
        self.assertEqual(tab['20']['n_segments'], 0)       # the 20 % non-start is C2 data, not a speed sample
        self.assertEqual(tab['20']['not_moving_segments'], 1)

    def test_c7_release_and_stop(self):
        kinds = self.r['c7']['kinds']
        self.assertGreaterEqual(kinds['release']['clean'], 3)
        self.assertEqual(kinds['hard_stop']['clean'], 1)
        self.assertLess(kinds['release']['distance_m_p10_p50_p90'][2], .35)

    def test_cache_rerun_skips_scans_and_gives_the_same_result(self):
        cache = os.path.join(self.s.root, 'cache')
        outs = []
        for k in range(2):
            out = os.path.join(self.s.root, 'out_cache%d.json' % k)
            cli.main(['--root', self.s.root, '--latest', '1', '--workers', '2', '--quiet', '--out', out, '--cache', cache])
            with open(out, 'r', encoding='utf-8') as fh:
                outs.append(json.load(fh))
        self.assertFalse(outs[0]['health'][0]['scan']['odometry_from_cache'])
        self.assertTrue(outs[1]['health'][0]['scan']['odometry_from_cache'])
        for key in ('c1', 'c2', 'c3', 'c4', 'c7', 'axis'):
            self.assertEqual(outs[0][key], outs[1][key], key)
        self.assertEqual(outs[0]['c3'], self.r['c3'])

    def test_result_is_strict_json_without_differential_steps(self):
        json.dumps(self.r, allow_nan=False)
        self.assertEqual(self.r['schema'], cli.SCHEMA)
        for key in ('c5', 'c6', 'recommendation'):
            self.assertNotIn(key, self.r)
        for name in ('c5_arc', 'c6_pivot', 'pivot_ladder', 'icr_along_left'):
            self.assertFalse(hasattr(steps, name), name)
        self.assertEqual(self.s.code, 0)



class LadderRuleTests(unittest.TestCase):
    def test_bfwd_needs_three_attempts_that_all_started(self):
        from field_calib import steps
        ev = lambda d, started, conclusive=False: dict(label='g', straight=True, req=[d, d], started=started,
                                                       conclusive=conclusive, precise_t0=False)
        events = [ev(20, False, True)] * 3 + [ev(25, True), ev(25, True), ev(25, False)] + [ev(30, True)] * 2 + \
            [ev(35, True)] * 3
        lad = steps.c2_breakaway(events)['labels']['g']['ladder']
        self.assertEqual(lad['b_fwd_pct'], 35)            # 25: one inconclusive; 30: only 2 attempts
        self.assertTrue(lad['bracketed'])


class RobustnessTests(unittest.TestCase):
    def test_clock_step_back_is_detected_in_file_order(self):
        rows = [dict(t=t, req_l=0, req_r=0, rep_l=0., rep_r=0., state='armed_manual') for t in (0., .5, 1., .2, .7)]
        m = records.Motor(rows)
        self.assertEqual(m.clock_back, 1)
        self.assertTrue(np.all(np.diff(m.t) >= 0))         # sorted for the analysis, but the step is counted

    def test_unreadable_recording_is_skipped_not_fatal(self):
        s = _Session.get()
        out = os.path.join(s.root, 'out_skip.json')
        code = cli.main(['--root', s.root, RID, '20991231T000000Z_both', '--workers', '1', '--quiet', '--out', out,
                         '--no-hash', '--cache', os.path.join(s.root, 'cache')])
        with open(out, 'r', encoding='utf-8') as fh:
            r = json.load(fh)
        self.assertEqual([x['rid'] for x in r['skipped_recordings']], ['20991231T000000Z_both'])
        self.assertEqual(len(r['health']), 1)
        self.assertEqual(code, 3)

    def test_steered_windows_are_dropped_from_the_motor_requests(self):
        s = _Session.get()
        d = tempfile.mkdtemp(prefix='fieldcal_steer_')
        try:
            rec = records.open_archive(s.root, RID, hashes=False)
            t0 = float(rec.motor.t[0])
            path = os.path.join(d, 'steering.jsonl')
            rows = [dict(received_at=t0 + k * .05, target_mrad=0 if not 400 <= k < 600 else 140) for k in range(1000)]
            with open(path, 'w', encoding='utf-8', newline='\n') as fh:
                fh.write(''.join(json.dumps(r) + '\n' for r in rows))
            windows = cli.steering_mask_windows(path)
            self.assertEqual(len(windows), 1)
            self.assertAlmostEqual(windows[0][0], t0 + 20., places=6)
            self.assertAlmostEqual(windows[0][1], t0 + 30., places=6)
            dropped = cli.drop_steered(rec, windows)
            inside = (rec.motor.t >= t0 + 19.5) & (rec.motor.t <= t0 + 30.5)
            self.assertEqual(dropped, int(inside.sum()))
            self.assertTrue(np.all(np.isnan(rec.motor.req_l[inside])))
            self.assertFalse(np.any(np.isnan(rec.motor.req_l[~inside])))
        finally:
            shutil.rmtree(d, ignore_errors=True)


if __name__ == '__main__':
    unittest.main()
