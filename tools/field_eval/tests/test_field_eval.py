"""field_eval tests: reference map, metrics against the registered SIM definitions, executor log, rosbag2 / CDR
readers, the recorder sidecar source, map-based localisation on ray-cast scans, the whole evaluation.

Runs under pytest and plain unittest (python -B -m unittest discover -s tools/field_eval/tests).
Synthetic data only (fe_testlib.py). The comparisons with follow_matrix / gsim_follow need the SIM tree (sim/ubts) and
are skipped without it (the Jetson).
"""
import json
import math
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fe_testlib as L  # noqa: E402

from field_eval import bag_reader as B  # noqa: E402
from field_eval import evaluate as EV  # noqa: E402
from field_eval import exec_log as XL  # noqa: E402
from field_eval import localise as LOC  # noqa: E402
from field_eval import metrics as MET  # noqa: E402
from field_eval import refmap as RM  # noqa: E402
from field_eval import sources as SRC  # noqa: E402

SIM = os.path.join(L.REPO, 'sim', 'ubts')


def _sim_modules():
    if not os.path.isdir(SIM):
        return None
    if SIM not in sys.path:
        sys.path.insert(0, SIM)
    try:
        from steering import gsim_follow as G
        from steering.safety_scripts import follow_matrix as FM
    except Exception:            # noqa: BLE001  (no SIM tree or a missing dependency: skip)
        return None
    return FM, G


class Temp(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix='field_eval_')

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)


class MapTests(Temp):
    def test_parallel_rows_centre_and_end(self):
        m = RM.load_map(L.write_json(os.path.join(self.dir, 'm.json'), L.map_doc()))
        g = m.geometry
        self.assertAlmostEqual(g['centre_a'], 0., places=12)
        self.assertAlmostEqual(g['centre_b'], 0., places=12)
        self.assertAlmostEqual(g['row_end_s'], 24., places=9)
        self.assertAlmostEqual(g['row_start_s'], -4.5, places=9)
        self.assertAlmostEqual(g['half_width_m'], 1.75, places=9)
        self.assertEqual((g['right_row'], g['left_row']), ('right', 'left'))
        self.assertEqual(m.start_pose, (0., 0., 0.))

    def test_slanted_rows_match_gsim_follow(self):
        mods = _sim_modules()
        if mods is None:
            self.skipTest('SIM tree not available')
        _, G = mods
        rng = np.random.default_rng(3)
        trees = []
        for row, base in ((0, -1.6), (1, 1.9)):
            for k in range(12):
                x = 1.5 * k + rng.normal(0, .05)
                trees.append(dict(row=row, x=x, y=base + .03 * x + rng.normal(0, .04), radius=.1))
        meta = dict(config=dict(tree_spacing=1.5, row_length=16.5, row_spacing=3.5, trellis_every=0),
                    spawn=[0., .1, 0.], lane_centers_y=[.15], trees=trees)
        geo = G.row_geometry(meta, lane=0)
        doc = dict(landmarks=[dict(id='T%d' % k, row=str(t['row']), x=t['x'], y=t['y'], radius_m=t['radius'])
                              for k, t in enumerate(trees)])
        m = RM.load_map(L.write_json(os.path.join(self.dir, 'g.json'), doc))
        for key in ('centre_a', 'centre_b', 'axis_rad', 'row_end_s', 'row_start_s', 'half_width_m'):
            self.assertAlmostEqual(m.geometry[key], geo[key], places=10, msg=key)
        for pose in ((3., .4, .1), (10., -.2, -.05)):
            for a, b in zip(m.to_row(*pose), G.to_row(geo, *pose)):
                self.assertAlmostEqual(a, b, places=12)

    def test_csv_and_lines(self):
        path = os.path.join(self.dir, 'm.csv')
        with open(path, 'wb') as fh:
            fh.write(b'# tape measured\nid,row,x_m,y_m,radius_m\nA,r,0,-1.7,0.1\nB,r,10,-1.7,\nC,l,0,1.8,0.1\n'
                     b'D,l,10,1.8,0.1\n')
        m = RM.load_map(path, lane_rows=['r', 'l'], default_radius_m=.07)
        self.assertAlmostEqual(m.geometry['centre_a'], .05)
        self.assertEqual([lm[4] for lm in m.landmarks], [.1, .07, .1, .1])
        lines = dict(row_lines=[{'row': 'a', 'from': [0, -1.5], 'to': [30, -1.5]},
                                {'row': 'b', 'from': [0, 1.5], 'to': [30, 1.7]}])
        m2 = RM.load_map(L.write_json(os.path.join(self.dir, 'l.json'), lines))
        g2 = m2.geometry
        ends = [math.cos(g2['axis_rad']) * px + math.sin(g2['axis_rad']) * (py - g2['centre_a'])
                for px, py in ((0, -1.5), (30, -1.5), (0, 1.5), (30, 1.7))]
        self.assertAlmostEqual(g2['row_end_s'], max(ends), places=12)
        self.assertAlmostEqual(g2['centre_a'], 0., places=12)
        self.assertAlmostEqual(g2['centre_b'], .2 / 30. / 2., places=12)

    def test_refuses_geographic_values(self):
        bad = L.map_doc()
        bad['landmarks'][0]['l' + 'at'] = 1.
        with self.assertRaises(RM.MapError):
            RM.load_map(L.write_json(os.path.join(self.dir, 'a.json'), bad))
        far = L.map_doc()
        far['landmarks'][0]['x'] = 350000.
        with self.assertRaises(RM.MapError):
            RM.load_map(L.write_json(os.path.join(self.dir, 'b.json'), far))
        path = os.path.join(self.dir, 'c.csv')
        with open(path, 'wb') as fh:
            fh.write(('row,x_m,y_m,' + 'lo' + 'n\nr,0,1,2\n').encode('utf-8'))
        with self.assertRaises(RM.MapError):
            RM.load_map(path)
        three = L.map_doc()
        three['landmarks'].append(dict(row='x', x=1., y=5.))
        three['landmarks'].append(dict(row='x', x=2., y=5.))
        del three['lane_rows']
        with self.assertRaises(RM.MapError):
            RM.load_map(L.write_json(os.path.join(self.dir, 'd.json'), three))


class MetricTests(Temp):
    def _trace(self, settle_offset=0.):
        rng = np.random.default_rng(5)
        t = np.arange(.01, 70.005, .01)
        s = np.clip(t - 2., 0, None) * .38
        x = s
        y = settle_offset * np.exp(-s / 3.) + .04 * np.sin(s / 2.) + rng.normal(0, .001, len(t))
        yaw = np.arctan(np.gradient(y, x + 1e-9 * t)) * (s > 0)
        v = np.where(t > 2., .38 + .02 * np.sin(t), 0.)
        throttle = np.where((t >= 1.2) & (t <= 66.), 46., 0.)
        return dict(t_s=t, truth_x_m=x, truth_y_m=y, truth_yaw_rad=yaw, truth_v_m_s=v, ctrl_throttle_pct=throttle)

    def test_measure_equals_follow_matrix(self):
        mods = _sim_modules()
        if mods is None:
            self.skipTest('SIM tree not available')
        FM, _ = mods
        m = RM.load_map(L.write_json(os.path.join(self.dir, 'm.json'), L.map_doc()))
        for settle, offset in ((0., 0.), (6., .4)):
            arr = self._trace(offset)
            ref = FM.measure(arr, [], dict(end_reason='row_end', events=[]), L.posts(), settle)
            on = np.flatnonzero(arr['ctrl_throttle_pct'] > 0)
            got = MET.measure(m, arr['t_s'], arr['truth_x_m'], arr['truth_y_m'], arr['truth_yaw_rad'],
                              arr['truth_v_m_s'], (arr['t_s'][on[0]], arr['t_s'][on[-1]]), settle, FM.FRONT_BUMPER_M)
            for key in ('lateral_rmse_m', 'lateral_p95_m', 'lateral_max_m', 'heading_rmse_deg', 'heading_max_deg',
                        'row_end_error_m', 'speed_max_m_s', 'speed_mean_m_s', 'distance_m'):
                self.assertEqual(got[key], ref[key], key)

    def test_path_speed_and_limits(self):
        t = np.arange(0, 10, .1)
        v = MET.path_speed(t, .3 * t, np.zeros(len(t)), np.zeros(len(t)))
        self.assertTrue(np.allclose(v[5:-5], .3))
        lim = MET.against_limits(dict(lateral_rmse_m=.1, row_end_error_m=-1.2))
        self.assertTrue(lim['lateral_rmse_m']['within'])
        self.assertFalse(lim['row_end_error_m']['within'])
        self.assertIsNone(lim['speed_max_m_s']['within'])


class ExecutorLogTests(Temp):
    def test_ticks_interval_wall(self):
        t, x, y, yaw, v = L.trajectory(duration=12.)
        path = L.write_jsonl(os.path.join(self.dir, 'EXECUTOR.jsonl'), L.executor_records(t, x, y, yaw, v, 2., 10.))
        log = XL.ExecutorLog(path)
        self.assertEqual(log.run, 1)
        self.assertEqual(log.config_digests, ['c' * 64])
        self.assertAlmostEqual(log.wall_minus_t, 1.7e9)
        self.assertEqual(log.drive_interval(), (2.0, 10.0))
        self.assertEqual(log.vehicle()['lidar_x_in_base_m'], L.LIDAR_X)
        self.assertEqual(log.describe()['end_reason'], 'row_end')


class BagTests(Temp):
    def _messages(self):
        r = np.linspace(.5, 9., L.BEAMS).astype(np.float32)
        r[10] = np.inf
        scans = [('/scan', B.SCAN_TYPE, int(1e9 * (100 + .1 * k)),
                  B.encode_laser_scan(100. + .1 * k, r + k, L.RAW_AMIN, L.RAW_INC)) for k in range(3)]
        imus = [('/imu/data_raw', B.IMU_TYPE, int(1e9 * (100 + .03 * k)),
                 B.encode_imu(100. + .03 * k, (.01, .02, .03 * k), (0., .1, 9.8))) for k in range(5)]
        other = [('/rover/status', 'std_msgs/msg/String', int(1e9 * 100.05), b'\x00\x01\x00\x00\x03\x00\x00\x00ab\x00')]
        return scans + imus + other, r

    def _check(self, scans, imus, r):
        self.assertEqual(len(scans), 3)
        self.assertEqual(len(imus), 5)
        self.assertAlmostEqual(scans[1]['stamp_s'], 100.1, places=6)
        self.assertAlmostEqual(scans[1]['receipt_s'], 100.1, places=6)
        self.assertEqual(scans[0]['frame_id'], 'laser')
        np.testing.assert_array_equal(scans[2]['ranges'], (r + 2).astype(np.float32))
        self.assertAlmostEqual(scans[0]['angle_increment'], np.float32(L.RAW_INC), places=9)
        self.assertEqual(imus[4]['gyro'], (.01, .02, .12))
        self.assertEqual(imus[0]['accel'], (0., .1, 9.8))

    def test_sqlite3(self):
        msgs, r = self._messages()
        B.write_sqlite_bag(os.path.join(self.dir, 'bag'), msgs)
        self._check(*B.read_bag(os.path.join(self.dir, 'bag')), r)

    def test_mcap_chunked_and_plain(self):
        msgs, r = self._messages()
        for chunked in (True, False):
            folder = os.path.join(self.dir, 'mcap%d' % chunked)
            os.makedirs(folder)
            B.write_mcap_bag(os.path.join(folder, 'bag_0.mcap'), msgs, chunked=chunked)
            self._check(*B.read_bag(folder), r)

    def test_compressed_chunk(self):
        msgs, r = self._messages()
        try:
            import zstandard  # noqa: F401
        except ImportError:
            zstandard = None
        if zstandard is not None:
            folder = os.path.join(self.dir, 'zstd')
            os.makedirs(folder)
            B.write_mcap_bag(os.path.join(folder, 'bag_0.mcap'), msgs, compression='zstd')
            self._check(*B.read_bag(folder), r)
        from unittest import mock
        with mock.patch.dict(sys.modules, {'zstandard': None, 'compression': None, 'lz4': None}):
            with self.assertRaises(B.BagFormatError):
                B._decompress('zstd', b'1234', 10)
            with self.assertRaises(B.BagFormatError):
                B._decompress('lz4', b'1234', 10)
        with self.assertRaises(B.BagFormatError):
            B._decompress('brotli', b'1234', 10)

    def test_rosbag_source(self):
        msgs, _ = self._messages()
        B.write_sqlite_bag(os.path.join(self.dir, 'arch', 'rosbag2'), msgs)
        scans, imu = SRC.open_source(os.path.join(self.dir, 'arch'))
        self.assertEqual(scans['source'], 'rosbag2')
        self.assertEqual(scans['ranges'].shape, (3, L.BEAMS))
        self.assertTrue(np.isnan(scans['ranges'][0, 10]))
        self.assertEqual(len(imu['t']), 5)


class LocaliseTests(Temp):
    def setUp(self):
        super().setUp()
        self.map = RM.load_map(L.write_json(os.path.join(self.dir, 'm.json'), L.map_doc()))
        self.vehicle = LOC.Vehicle(L.LIDAR_X, L.OFFSET_DEG)

    def _run(self, start_error=(0., 0., 0.), imu=True, duration=25., noise=.01):
        t, x, y, yaw, v = L.trajectory(duration=duration)
        ranges = L.scans_for(t, x, y, yaw, noise=noise)
        r = ranges.astype(np.float32)
        r[r <= 0] = np.nan
        scans = dict(t=t, ranges=r, angle_min=L.RAW_AMIN, angle_increment=L.RAW_INC, beams=L.BEAMS)
        ti, gyro, accel = L.imu_for(t, yaw)
        imu_d = dict(t=ti, gyro=gyro, accel=accel) if imu else None
        start = (x[0] + start_error[0], y[0] + start_error[1], math.degrees(yaw[0]) + start_error[2])
        traj = LOC.Localiser(self.map, self.vehicle).run(scans, imu_d, start, drive_start=2.)
        return traj, (t, x, y, yaw)

    def test_tracks_truth(self):
        traj, (t, x, y, yaw) = self._run()
        err = np.hypot(traj['x'] - x, traj['y'] - y)
        self.assertTrue(np.all(traj['quality'] == LOC.OK))
        self.assertLess(float(np.max(err)), .01)
        self.assertLess(float(np.max(np.abs(np.degrees(LOC.wrap(traj['yaw'] - yaw))))), .3)
        self.assertTrue(traj['imu']['available'])

    def test_start_error_is_found(self):
        traj, (t, x, y, yaw) = self._run(start_error=(.35, -.2, 5.), imu=False)
        self.assertLess(abs(traj['init']['pose'][0] - x[0]), .06)
        err = np.hypot(traj['x'] - x, traj['y'] - y)
        self.assertLess(float(np.max(err)), .01)

    def test_no_landmarks_is_lost(self):
        far = RM.RefMap([('a', 'r', 100., -1., .1, 'p'), ('b', 'r', 110., -1., .1, 'p'), ('c', 'l', 100., 1., .1, 'p'),
                         ('d', 'l', 110., 1., .1, 'p')])
        t, x, y, yaw, v = L.trajectory(duration=3.)
        r = L.scans_for(t, x, y, yaw).astype(np.float32)
        r[r <= 0] = np.nan
        scans = dict(t=t, ranges=r, angle_min=L.RAW_AMIN, angle_increment=L.RAW_INC, beams=L.BEAMS)
        traj = LOC.Localiser(far, self.vehicle).run(scans, None, (0., 0., 0.))
        self.assertTrue(np.all(np.isin(traj['quality'], (LOC.LOST, LOC.NONE))))

    def test_se2_closed_form(self):
        rng = np.random.default_rng(2)
        body = rng.normal(0, 3, (10, 2))
        pose = (1.2, -.4, .3)
        world = LOC.transform(body, pose)
        got = LOC.solve_se2(body, world, np.ones(10))
        np.testing.assert_allclose(got, pose, atol=1e-12)


class EvaluateTests(Temp):
    def _folder(self, sidecar=False, lat_bias=.03):
        t, x, y, yaw, v = L.trajectory(duration=30.)
        ranges = L.scans_for(t, x, y, yaw)
        imu = L.imu_for(t, yaw)
        run = os.path.join(self.dir, 'run')
        map_path = L.write_json(os.path.join(self.dir, 'map.json'), L.map_doc())
        recs = L.executor_records(t, x, y, yaw, v, 2., 29., lat_bias=lat_bias)
        if sidecar:
            wall0 = 1.7e9
            L.write_sidecar(run, t + wall0, ranges, (imu[0] + wall0, imu[1], imu[2]))
            for rec in recs:                            # the executor's monotonic clock differs from the wall clock
                rec['t'] = round(rec['t'] + 500., 4)
        else:
            L.write_fsim_folder(run, t, ranges, imu)
        exe = L.write_jsonl(os.path.join(self.dir, 'EXECUTOR.jsonl'), recs)
        return map_path, exe, run, (t, x, y, yaw, v)

    def test_fsim_folder(self):
        map_path, exe, run, (t, x, y, yaw, v) = self._folder()
        out = os.path.join(self.dir, 'out')
        rep = EV.evaluate(map_path, exe, run, out)
        m = rep['reference_metrics']
        sel = (t >= 2.) & (t <= 29.)
        self.assertAlmostEqual(m['lateral_max_m'], float(np.max(np.abs(y[sel]))), delta=.006)
        self.assertAlmostEqual(m['lateral_rmse_m'], float(np.sqrt(np.mean(y[sel] ** 2))), delta=.004)
        self.assertAlmostEqual(rep['executor_estimate_error']['lateral_m']['mean'], .03, delta=.004)
        self.assertLess(abs(rep['executor_estimate_error']['heading_deg']['mean']), .1)
        self.assertEqual(rep['config_digest'], ['c' * 64])
        self.assertEqual(rep['result_kind'], EV.SIM_KIND)
        with open(os.path.join(out, 'REPORT.json'), 'rb') as fh:
            saved = json.loads(fh.read().decode('utf-8'))
        self.assertEqual(len(saved['inputs']['map']['sha256']), 64)
        self.assertIn('SCANS.npz', saved['inputs']['scans']['sha256'])
        self.assertEqual(sorted(saved['figures']), ['fig_duty_speed.svg', 'fig_lateral_vs_distance.svg',
                                                    'fig_speed.svg', 'fig_steering.svg', 'fig_trajectory.svg'])
        with open(os.path.join(out, 'fig_trajectory.svg'), 'rb') as fh:
            svg = fh.read().decode('utf-8')
        self.assertIn('상세', svg)
        self.assertIn('전체 관측 범위', svg)

    def test_sidecar_wall_clock(self):
        if not os.path.isdir(os.path.join(L.TOOLS, 'field_calib')):
            self.skipTest('field_calib not available')
        map_path, exe, run, (t, x, y, yaw, v) = self._folder(sidecar=True, lat_bias=0.)
        rep = EV.evaluate(map_path, exe, run, None)
        self.assertEqual(rep['inputs']['scans']['source'], 'sidecar')
        self.assertAlmostEqual(rep['time']['executor_to_source_shift_s'], 1.7e9 - 500., places=3)
        self.assertLess(abs(rep['executor_estimate_error']['lateral_m']['mean']), .004)
        self.assertGreater(rep['localisation']['in_drive']['ok_fraction'], .99)

    def test_external_trajectory(self):
        map_path, exe, run, (t, x, y, yaw, v) = self._folder()
        path = os.path.join(self.dir, 'ref.csv')
        with open(path, 'wb') as fh:
            fh.write(('t_s,x_m,y_m,yaw_deg\n' + ''.join('%.2f,%.4f,%.4f,%.3f\n' % (a, b, c, math.degrees(d))
                                                     for a, b, c, d in zip(t, x, y, yaw))).encode('utf-8'))
        rep = EV.evaluate(map_path, exe, None, None, trajectory_csv=path)
        sel = (t >= 2.) & (t <= 29.)
        self.assertAlmostEqual(rep['reference_metrics']['lateral_max_m'], float(np.max(np.abs(y[sel]))), places=3)


if __name__ == '__main__':
    unittest.main()
