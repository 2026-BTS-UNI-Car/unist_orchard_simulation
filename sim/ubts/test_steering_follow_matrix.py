"""The single-row following matrix (steering/safety_scripts/follow_matrix.py): world, specs, metrics and judge.

Hand-made traces only: no rig, no executor, no closed loop is run here. What is checked is the evaluator itself (the
post row, the episode specs of the registered conditions, the footprint clearance, the metrics windows, each registered
check failing on its own) and that docs/steering/PREREG_STEER_FOLLOW.json is the pre-registration this module prints.
Every number is a test value (자리값 기반 SIM 결과).

Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_follow_matrix.py
"""
import json
import math
import os
import unittest

import numpy as np

from steering.safety_scripts import follow_matrix as fm

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PASSED_A22 = dict(passed=True, complete=True, max_target_move_deg=0., executor_command_move_deg=0.)


def trace(y=0., yaw_deg=0., x_end=23.4, v=.4, t_drive=(5., 65.), dt=.05, t_end=70.):
    """A straight drive along x: standstill, drive at v (ramps ignored), rest at x_end."""
    t = np.arange(0., t_end, dt)
    drive = (t >= t_drive[0]) & (t < t_drive[1])
    x = np.clip((t - t_drive[0]) / (t_drive[1] - t_drive[0]), 0., 1.) * x_end
    ones = np.ones_like(t)
    y = y(x) if callable(y) else y * ones
    yaw = np.radians(yaw_deg(x) if callable(yaw_deg) else yaw_deg * ones)
    return dict(t_s=t, truth_x_m=x, truth_y_m=y, truth_yaw_rad=yaw, truth_v_m_s=np.where(drive, v, 0.),
                ctrl_throttle_pct=np.where(drive, 45., 0.), ctrl_steer_mrad=0. * t)


def summary(end_reason='row_end', ended_at_s=65.5, events=()):
    return dict(end_reason=end_reason, ended_at_s=ended_at_s, events=[list(e) for e in events])


def ticks(arrays, lidar_age=None, state='ACTIVE'):
    out = []
    for k in range(0, len(arrays['t_s']), 2):
        t = float(arrays['t_s'][k])
        age = .1 if lidar_age is None else lidar_age(t)
        out.append(dict(k='tick', t=t, T=int(arrays['ctrl_throttle_pct'][k]), S=0, state=state, reason='follow',
                        lidar_age=age))
    return out


def measured(arrays, summ=None, settle_m=0., markers=None, tick_rows=None, a22=PASSED_A22):
    m = fm.measure(arrays, ticks(arrays) if tick_rows is None else tick_rows, summ or summary(),
                   fm.posts() if markers is None else markers, settle_m)
    m['after_terminal'] = dict(a22)
    return m


def failed(expect, m):
    return [c['check'] for c in fm.judge(expect, m) if not c['ok']]


class WorldAndSpecTest(unittest.TestCase):
    def test_post_row(self):
        posts = fm.posts()
        self.assertEqual(len(posts), 40)
        self.assertLessEqual(len(fm.posts([[12., 0., .12]])), 64)
        xs = sorted({p[0] for p in posts})
        self.assertEqual((xs[0], xs[-1]), (fm.ROW_START_X_M, fm.ROW_END_X_M))
        self.assertEqual({p[1] for p in posts}, {fm.ROW_HALF_WIDTH_M, -fm.ROW_HALF_WIDTH_M})
        self.assertTrue(all(abs(b - a - fm.POST_SPACING_M) < 1e-9 for a, b in zip(xs, xs[1:])))

    def test_registered_file_is_this_module(self):
        path = os.path.join(REPO, 'docs', 'steering', 'PREREG_STEER_FOLLOW.json')
        with open(path, 'rb') as fh:
            raw = fh.read()
        self.assertNotIn(b'\r', raw)
        self.assertEqual(json.loads(raw.decode('utf-8')), json.loads(json.dumps(fm.prereg())))
        self.assertEqual(fm.prereg()['runs'], 15)

    def test_specs(self):
        by_name = {c['name']: c for c in fm.CONDITIONS}
        nominal = fm.spec(by_name['nominal'], 3)
        self.assertEqual((nominal['seed'], nominal['controller'], nominal['auto_repress']),
                         (3, {'kind': 'row_follow'}, False))
        self.assertEqual((nominal['start_pose'], nominal['rig'], nominal['faults']), ([0., 0., 0.], {}, []))
        self.assertEqual(fm.spec(by_name['heading_n8'], 0)['start_pose'], [0., 0., -8.])
        zero = fm.spec(by_name['steer_zero_p2'], 0)['rig']['profile']
        self.assertEqual(zero.per_run_value(0, 'steer_zero_offset_deg'), 2.)
        pull = fm.spec(by_name['side_pull'], 0)['rig']['profile']
        self.assertEqual(pull.value('curvature_wander')['mean_deg_per_m'], .6)
        stale = fm.spec(by_name['lidar_stale_1p0'], 0)['faults']
        self.assertEqual(stale, [dict(kind='sensor_blackout', at_s=20., params=dict(duration_s=1., which='lidar'))])
        self.assertIn([12., 0., fm.POST_RADIUS_M], fm.spec(by_name['obstacle'], 0)['world']['markers'])
        printable = fm.printable(fm.spec(by_name['steer_zero_p2'], 0), by_name['steer_zero_p2'])
        json.dumps(printable, allow_nan=False)

    def test_clearance(self):
        zero = np.zeros(1)
        gap = fm.clearance(zero, zero, zero, [[0., 1.75, .12]])
        self.assertAlmostEqual(gap, 1.75 - fm.HALF_WIDTH_M - .12)
        self.assertAlmostEqual(fm.clearance(zero, zero, zero, [[2., 0., .12]]), 2. - fm.FRONT_BUMPER_M - .12)
        self.assertLess(fm.clearance(zero, zero, zero, [[.5, .2, .12]]), 0.)                   # inside the footprint
        turned = fm.clearance(zero, zero, np.array([math.pi / 2]), [[0., 1.75, .12]])
        self.assertAlmostEqual(turned, 1.75 - fm.FRONT_BUMPER_M - .12)


class JudgeTest(unittest.TestCase):
    def test_a_clean_run_passes(self):
        m = measured(trace(y=.05, yaw_deg=1.))
        self.assertEqual(failed('row_end', m), [])
        self.assertAlmostEqual(m['row_end_error_m'], 23.4 + .61 * math.cos(math.radians(1.)) - 24., places=3)
        self.assertEqual((m['lateral_rmse_m'], m['lateral_max_m'], m['heading_max_deg']), (.05, .05, 1.))
        self.assertAlmostEqual(measured(trace(y=.05))['min_clearance_m'], 1.75 - .05 - .35 - .12, places=3)

    def test_each_check_fails_alone(self):
        self.assertEqual(failed('row_end', measured(trace(y=.25))), ['lateral RMSE'])
        bump = lambda x: np.where((x > 10.) & (x < 11.5), .40, 0.)                             # noqa: E731
        self.assertEqual(failed('row_end', measured(trace(y=bump))), ['lateral p95'])
        spike = lambda x: np.where((x > 10.) & (x < 10.3), .55, 0.)                            # noqa: E731
        self.assertEqual(failed('row_end', measured(trace(y=spike))), ['lateral max'])
        self.assertEqual(failed('row_end', measured(trace(yaw_deg=6.))), ['heading RMSE'])
        swing = lambda x: np.where((x > 10.) & (x < 10.3), 16., 0.)                            # noqa: E731
        self.assertEqual(failed('row_end', measured(trace(yaw_deg=swing))), ['heading max'])
        self.assertEqual(failed('row_end', measured(trace(v=.55))), ['true speed max'])
        self.assertEqual(failed('row_end', measured(trace(x_end=24.6))), ['row end stop position e'])
        self.assertEqual(failed('row_end', measured(trace(x_end=22.2))), ['row end stop position e'])
        self.assertEqual(failed('row_end', measured(trace(), summary('corridor_lost'))), ['ends at the row end'])
        paused = summary(events=[(1., 'PAUSED', 'await_hold'), (30., 'PAUSED', 'stale_lidar')])
        self.assertEqual(failed('row_end', measured(trace(), paused)), ['no pause after the drive started'])
        self.assertEqual(failed('row_end', measured(trace(), a22=dict(passed=False, complete=True))),
                         ['no commanded steering move after the terminal moment (A22 rule)'])
        guard = failed('row_end', measured(trace(), summary('guard_violation:rate')))
        self.assertEqual(guard, ['no guard violation', 'ends at the row end'])
        contact = measured(trace(), markers=fm.posts([[12., .2, .12]]))
        self.assertEqual(failed('row_end', contact), ['no contact'])
        moving = trace()
        moving['truth_v_m_s'][-1] = .1
        self.assertEqual(failed('row_end', measured(moving)), ['at rest at the end'])

    def test_offset_start_uses_the_settled_part(self):
        decay = lambda x: .48 * np.exp(-x / 5.)                                               # noqa: E731
        whole = measured(trace(y=decay))
        settled = measured(trace(y=decay), settle_m=fm.SETTLE_M)
        self.assertGreater(whole['lateral_p95_m'], fm.CRITERIA['lateral_p95_m'])
        self.assertEqual(failed('row_end', settled), [])
        self.assertEqual(settled['lateral_max_m'], whole['lateral_max_m'])                    # the maximum: whole drive
        self.assertAlmostEqual(settled['lateral_max_m'], .48, places=3)
        short = measured(trace(x_end=4.), settle_m=fm.SETTLE_M)
        self.assertNotIn('lateral_rmse_m', short)
        self.assertIn('lateral RMSE', failed('row_end', short))

    def test_stale_pause(self):
        arrays = trace(t_drive=(5., 20.5), x_end=6.)
        age = lambda t: .1 if t < 20.4 else min(1.1, .1 + (t - 20.))                           # noqa: E731
        rows = ticks(arrays, age)
        summ = summary(None, None, [(1., 'PAUSED', 'await_hold'), (20.5, 'PAUSED', 'stale_lidar')])
        m = measured(arrays, summ, tick_rows=rows)
        self.assertEqual(m['first_pause'], [20.5, 'stale_lidar'])
        self.assertLessEqual(m['stale_drive_zero_s'], .1)
        self.assertEqual(failed('stale_lidar', m), [])
        late = [dict(r, T=45) if 20.4 <= r['t'] < 20.8 else r for r in rows]
        self.assertEqual(failed('stale_lidar', measured(arrays, summ, tick_rows=late)),
                         ['drive command 0 after the scan is stale'])
        again = [dict(r, T=30) if r['t'] > 40. else r for r in rows]
        self.assertEqual(failed('stale_lidar', measured(arrays, summ, tick_rows=again)), ['no restart'])
        other = summary(None, None, [(20.5, 'PAUSED', 'stale_imu')])
        self.assertEqual(failed('stale_lidar', measured(arrays, other, tick_rows=rows)), ['first pause is stale_lidar'])

    def test_obstacle_end(self):
        arrays = trace(t_drive=(5., 30.), x_end=10.6)
        m = measured(arrays, summary('obstacle_ahead', 30.5), markers=fm.posts([[12., 0., .12]]))
        self.assertEqual(failed('obstacle_ahead', m), [])
        self.assertAlmostEqual(m['min_clearance_m'], 12. - 10.6 - .61 - .12, places=3)
        other = measured(arrays, summary('row_end', 30.5))
        self.assertEqual(failed('obstacle_ahead', other), ['ends with obstacle_ahead'])


if __name__ == '__main__':
    unittest.main()
