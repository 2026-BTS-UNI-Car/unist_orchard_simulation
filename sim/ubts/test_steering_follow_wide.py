"""The wide following matrix (steering/safety_scripts/follow_wide.py) and its condition injector (PC-11).

Hand-made traces and specs only: no rig, no executor, no closed loop is run here (the plant drive table:
test_steering_drive_table.py). What is checked: the row geometry (default row = the follow_matrix row, arc frame,
gaps, end posts, last structure), the episode specs of every registered condition (keys, worlds, the 'off' spec equals
the follow_matrix spec), the metrics and every registered check failing on its own, and that
docs/steering/PREREG_STEER_FOLLOW_V2.json is the pre-registration this module prints.
Every number is a test value (자리값 기반 SIM 결과).

Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_follow_wide.py
"""
import json
import os
import unittest

import numpy as np

from steering.safety_scripts import follow_matrix as fm
from steering.safety_scripts import follow_wide as fw

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PASSED_A22 = dict(passed=True, complete=True, max_target_move_deg=0., executor_command_move_deg=0.)
NO_ART = dict(lines=[])


def trace(d=0., heading_deg=0., s_end=23.45, v=.4, t_drive=(5., 65.), dt=.05, t_end=70., geo=None):
    """A drive along the row's centre line (or offset d), standstill before and after."""
    geo = geo or fw.RowGeometry()
    t = np.arange(0., t_end, dt)
    drive = (t >= t_drive[0]) & (t < t_drive[1])
    s = np.clip((t - t_drive[0]) / (t_drive[1] - t_drive[0]), 0., 1.) * s_end
    dd = d(s) if callable(d) else d * np.ones_like(t)
    hh = np.radians(heading_deg(s) if callable(heading_deg) else heading_deg * np.ones_like(t))
    pts = [geo.point(a, b) for a, b in zip(s, dd)]
    x = np.array([p[0] for p in pts])
    y = np.array([p[1] for p in pts])
    yaw = np.array([geo.centre(a)[2] for a in s]) + hh
    return dict(t_s=t, truth_x_m=x, truth_y_m=y, truth_yaw_rad=yaw, truth_v_m_s=np.where(drive, v, 0.),
                ctrl_throttle_pct=np.where(drive, 45., 0.), ctrl_steer_mrad=0. * t)


def ticks(arrays, lidar_age=None, imu_age=None):
    out = []
    for k in range(0, len(arrays['t_s']), 1):
        t = float(arrays['t_s'][k])
        out.append(dict(k='tick', t=round(t, 3), T=int(arrays['ctrl_throttle_pct'][k]), S=0, state='ACTIVE',
                        reason='follow', lidar_age=.1 if lidar_age is None else lidar_age(t),
                        imu_age=.01 if imu_age is None else imu_age(t)))
    return out


def summary(end_reason='row_end', ended_at_s=65.5, events=(), latches=()):
    return dict(end_reason=end_reason, ended_at_s=ended_at_s, events=[list(e) for e in events],
                host_latches=[list(item) for item in latches])


def measured(c, arrays, summ=None, tick_rows=None, art=NO_ART, gnss_report=None):
    geo = fw.geometry(c.get('world'))
    m = fw.measure(c, arrays, ticks(arrays) if tick_rows is None else tick_rows, summ or summary(), None,
                   geo.posts(), art, gnss_report)
    m['after_terminal'] = dict(PASSED_A22)
    return m


def failed(c, m):
    return [ch['check'] for ch in fw.judge(c, m) if not ch['ok']]


class GeometryTest(unittest.TestCase):
    def test_default_row_is_the_follow_matrix_row(self):
        geo = fw.RowGeometry()
        self.assertEqual(geo.posts(), fm.posts())
        self.assertEqual((geo.last_tree_s(), geo.last_structure_s()), (fm.ROW_END_X_M, fm.ROW_END_X_M))

    def test_arc_frame_round_trip(self):
        for k in (1. / 60., -1. / 60., 1. / 30., -1. / 30.):
            geo = fw.RowGeometry(curvature_1_m=k)
            for s, d, h in ((0., 0., 0.), (9., .3, .05), (21., -.6, -.1)):
                x, y = geo.point(s, d)
                ss, dd, hh = geo.frame([x], [y], [geo.centre(s)[2] + h])
                self.assertAlmostEqual(ss[0], s, places=9)
                self.assertAlmostEqual(dd[0], d, places=9)
                self.assertAlmostEqual(hh[0], h, places=9)
            # every post lies on a line at +-half width
            posts = geo.posts()
            _, d, _ = geo.frame([p[0] for p in posts], [p[1] for p in posts], [0.] * len(posts))
            self.assertTrue(np.allclose(np.abs(d), geo.half_width_m, atol=2e-3))

    def test_gaps_end_posts_and_last_structure(self):
        base = len(fw.RowGeometry().posts())
        self.assertEqual(len(fw.geometry(dict(gaps=[['L', 12.0, 1]])).posts()), base - 1)
        self.assertEqual(len(fw.geometry(dict(gaps=[['B', 10.5, 3]])).posts()), base - 6)
        three = fw.geometry(dict(gaps=[['L', 10.5, 3]]))
        left = sorted(p[0] for p in three.posts() if p[1] > 0)
        self.assertIn((9.0, 15.0), list(zip(left, left[1:])))
        self.assertEqual(max(b - a for a, b in zip(left, left[1:])), 6.0)
        post = fw.geometry(dict(end_post_m=1.2))
        self.assertEqual(len(post.posts()), base + 2)
        self.assertAlmostEqual(post.last_structure_s(), 25.2)
        self.assertAlmostEqual(post.last_tree_s(), 24.0)
        sparse = fw.geometry(dict(spacing_m=3.0))
        self.assertAlmostEqual(sparse.last_tree_s(), 22.5)
        self.assertLessEqual(len(fw.geometry(dict(spacing_m=1.0)).posts()), 64)    # scene_spec.MAX_MARKERS

    def test_start_pose_on_an_arc(self):
        geo = fw.RowGeometry(curvature_1_m=1. / 30.)
        self.assertEqual(geo.start_pose(0., .4, 10.), (0., .4, 10.))


class SpecTest(unittest.TestCase):
    def test_registered_file_is_this_module(self):
        path = os.path.join(REPO, 'docs', 'steering', 'PREREG_STEER_FOLLOW_V2.json')
        with open(path, 'rb') as fh:
            raw = fh.read()
        self.assertNotIn(b'\r', raw)
        self.assertEqual(json.loads(raw.decode('utf-8')), json.loads(json.dumps(fw.prereg())))

    def test_every_condition_builds_and_seeds_are_registered(self):
        names = [c['name'] for c in fw.CONDITIONS]
        self.assertEqual(len(names), len(set(names)))
        self.assertGreaterEqual(len(fw.DEV_SEEDS), 5)
        self.assertGreaterEqual(len(fw.JUDGE_SEEDS), 5)
        self.assertFalse(set(fw.DEV_SEEDS) & set(fw.JUDGE_SEEDS))
        for c in fw.CONDITIONS:
            self.assertIn(c['expect'], fw.EXPECTS)
            ep = fw.spec(c, fw.DEV_SEEDS[0])
            self.assertEqual(ep['controller']['kind'], 'row_follow')
            self.assertFalse(ep['auto_repress'])
            if ep['world']['kind'] == 'W0':
                self.assertLessEqual(len(ep['world']['markers']), 64)

    def test_off_condition_is_the_follow_matrix_spec(self):
        mine = fw.spec(fw.condition('nominal'), 0, name='follow_nominal_s0')
        theirs = fm.spec(next(c for c in fm.CONDITIONS if c['name'] == 'nominal'), 0)
        mine.pop('duration_s')
        theirs.pop('duration_s')
        self.assertEqual(mine, theirs)

    def test_injections_reach_the_spec(self):
        self.assertEqual(fw.spec(fw.condition('speed_0p5'), 100)['controller']['cruise_speed_m_s'], .5)
        self.assertEqual(fw.spec(fw.condition('gnss_jump_5m'), 100)['config']['gnss_mode'], 'geofence')
        self.assertEqual(len(fw.spec(fw.condition('gnss_jump_5m'), 100)['rig']['gnss']['jumps']), 3)
        self.assertEqual(fw.spec(fw.condition('imu_stale_1p0'), 100)['faults'][0]['params']['which'], 'imu')
        self.assertEqual(fw.spec(fw.condition('operator_link_lost'), 100)['actions'][0]['op'], 'screen_hidden')
        table = fw.spec(fw.condition('table_x070'), 100)['rig']['drive_table']
        self.assertEqual(table['factor'], .7)
        self.assertTrue(table['source'].startswith('field_tuning sha256 '))
        dist = fw.spec(fw.condition('dist_step_p2p5'), 100)
        self.assertEqual(dist['rig']['profile'].profile_id, 'front_steer_v0_3_placeholder')
        self.assertEqual(dist['faults'][0]['kind'], 'steer_torque')
        pull = fw.spec(fw.condition('pull_right'), 100)['rig']['profile']
        self.assertEqual(pull.value('curvature_wander')['mean_deg_per_m'], -.6)


class MetricsAndJudgeTest(unittest.TestCase):
    def test_nominal_passes_and_each_row_end_check_fails_on_its_own(self):
        c = fw.condition('nominal')
        m = measured(c, trace(d=.05))
        self.assertEqual(failed(c, m), [])
        self.assertAlmostEqual(m['row_end_error_m'], 23.45 + fw.FRONT_BUMPER_M - 24.0, places=3)
        self.assertIn('lateral max', failed(c, measured(c, trace(d=lambda s: .6 * (s > 10.) * (s < 11.)))))
        self.assertIn('lateral RMSE', failed(c, measured(c, trace(d=.25))))
        self.assertIn('heading max', failed(c, measured(c, trace(heading_deg=lambda s: 16. * (abs(s - 5.) < .2)))))
        self.assertIn('true speed max', failed(c, measured(c, trace(v=.51))))
        self.assertIn('row end stop position e (last structure)', failed(c, measured(c, trace(s_end=25.))))
        self.assertIn('ends at the row end', failed(c, measured(c, trace(), summary('corridor_lost'))))
        self.assertIn('no contact', failed(c, measured(c, trace(d=1.5))))

    def test_fast_speed_limit(self):
        c = fw.condition('speed_0p5')
        self.assertEqual(failed(c, measured(c, trace(v=.53))), [])
        self.assertIn('true speed max', failed(c, measured(c, trace(v=.56))))

    def test_curved_row_metrics_are_in_the_row_frame(self):
        c = fw.condition('curve_l30')
        geo = fw.geometry(c['world'])
        m = measured(c, trace(d=.1, geo=geo))
        self.assertAlmostEqual(m['lateral_max_m'], .1, places=6)
        self.assertLess(m['heading_max_deg'], 1e-6)
        self.assertEqual(failed(c, m), [])

    def test_end_post_e_is_against_the_post(self):
        c = fw.condition('endpost_1p2')
        m = measured(c, trace(s_end=24.6))
        self.assertAlmostEqual(m['row_end_error_m'], 24.6 + fw.FRONT_BUMPER_M - 25.2, places=3)
        self.assertAlmostEqual(m['row_end_error_last_tree_m'], 24.6 + fw.FRONT_BUMPER_M - 24.0, places=3)
        self.assertEqual(failed(c, m), [])

    def test_offset_start_settles(self):
        c = fw.condition('lateral_p040')
        m = measured(c, trace(d=lambda s: .4 * np.exp(-s / 1.5)))
        self.assertEqual(m['settle_m'], fw.SETTLE_M)
        self.assertEqual(failed(c, m), [])

    def test_stale_imu(self):
        c = fw.condition('imu_stale_1p0')
        arrays = trace()
        arrays['ctrl_throttle_pct'] = np.where(arrays['t_s'] >= 20.4, 0., arrays['ctrl_throttle_pct'])
        rows = ticks(arrays, imu_age=lambda t: .5 if 20.4 <= t < 21. else .01)
        summ = summary(None, None, events=[(20.4, 'PAUSED', 'stale_imu')])
        self.assertEqual(failed(c, measured(c, arrays, summ, rows)), [])
        late = ticks(arrays, imu_age=lambda t: .5 if 20.2 <= t < 21. else .01)
        self.assertIn('drive command 0 after the imu input is stale', failed(c, measured(c, arrays, summ, late)))

    def test_operator_link(self):
        c = fw.condition('operator_link_lost')
        arrays = trace()
        arrays['ctrl_throttle_pct'] = np.where(arrays['t_s'] >= 20.5 - 1e-9, 0., arrays['ctrl_throttle_pct'])
        summ = summary(None, None, events=[(20.5, 'PAUSED', 'operator_link_lost')])
        m = measured(c, arrays, summ)
        self.assertEqual(m['link_last_beat_s'], 19.95)
        self.assertEqual(m['link_drive_zero_s'], .55)
        self.assertEqual(failed(c, m), [])
        arrays['ctrl_throttle_pct'] = np.where((arrays['t_s'] >= 20.5 - 1e-9) & (arrays['t_s'] < 20.6), 0., 45.)
        self.assertIn('no restart', failed(c, measured(c, arrays, summ)))

    def test_steering_faults_and_latches(self):
        art = dict(lines=[('F STALL*12', 8406, 8406)])
        c = fw.condition('dist_step_p2p5')
        m = measured(c, trace(), summary(latches=[('mcu_disarmed', 8.44, 'MCU fault STALL')]), art=art)
        self.assertEqual(m['steer_fault_lines'], [['STALL', 8.406]])
        self.assertEqual(sorted(failed(c, m)), ['no host bridge latch', 'no steering fault line of the MCU'])
        above = fw.condition('dist_above_5')
        stop = trace(s_end=3.)
        self.assertEqual(failed(above, measured(above, stop, summary('steering_driver_fault'),
                                                art=dict(lines=[('F DRV_STUCK*1', 6224, 6224)]))), [])
        self.assertIn('first steering fault is a registered one',
                      failed(above, measured(above, stop, summary('steering_driver_fault'),
                                             art=dict(lines=[('F POT_NOISE*1', 6224, 6224)]))))
        self.assertIn('steering fault within 3 s of the onset',
                      failed(above, measured(above, stop, summary('steering_stalled'),
                                             art=dict(lines=[('F STALL*1', 9500, 9500)]))))
        either = fw.condition('dist_terrain_x2')
        self.assertEqual(failed(either, measured(either, trace())), [])
        self.assertEqual(failed(either, measured(either, trace(s_end=10.), summary('steering_stalled'),
                                                 art=dict(lines=[('F STALL*1', 7000, 7000)]))), [])

    def test_gnss_layer_judgement(self):
        c = fw.condition('gnss_jump_5m')
        arrays = trace()
        rows = ticks(arrays)
        rows[-1]['gnss'] = dict(counts=dict(jump=3), fence_tripped=False)
        report = dict(counts=dict(jumped=3), jump_stamps=[30.23, 45.23, 60.23])
        self.assertEqual(failed(c, measured(c, arrays, tick_rows=rows, gnss_report=report)), [])
        rows[-1]['gnss'] = dict(counts=dict(jump=2), fence_tripped=True)
        self.assertEqual(sorted(failed(c, measured(c, arrays, tick_rows=rows, gnss_report=report))),
                         ['GNSS fence not tripped', 'GNSS jumps rejected = fixes carrying a jump'])


if __name__ == '__main__':
    unittest.main()
