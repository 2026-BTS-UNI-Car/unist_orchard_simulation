"""G-SIM row-following confirmation: the evaluation definitions (steering.gsim_follow) and the host run tool
(steering.exec_run). No Docker, no ROS, no Gazebo."""
import json
import math
import tempfile
import unittest
from pathlib import Path

from steering import exec_run
from steering import gsim_follow as GF

ROOT = Path(__file__).resolve().parents[2]


def metadata(rows=6, spacing=3.8, tree_spacing=1.2, length=30., shift=0., slope=0.):
    """Generator-like metadata: straight tree rows (optionally shifted / sloped), spawn 3 m before lane 0."""
    trees = []
    n = int(length / tree_spacing) + 1
    for j in range(rows):
        for i in range(n):
            x = i * tree_spacing
            trees.append(dict(row=j, x=x, y=(j - .5) * spacing + shift + slope * x, z=0., radius=.04, variant=0))
    return dict(config=dict(rows=rows, row_spacing=spacing, tree_spacing=tree_spacing, row_length=length,
                            trellis_every=8, spawn_yaw=0.),
                lane_centers_y=[k * spacing for k in range(rows - 1)], row_y=[(j - .5) * spacing for j in range(rows)],
                spawn=[-3., 0., .2], trees=trees,
                rocks=[dict(x=5., y=.3, r=.1, lane=0), dict(x=6., y=4., r=.1, lane=1)])


def straight_run(geometry, lateral=0., speed=.4, seconds=80., rate=30., stop_s=None):
    """Truth rows of a run along the centre line at a lateral offset, and the duty rows."""
    odom, duty = [], []
    x0 = geometry['spawn'][0]
    stop_s = seconds - 5. if stop_s is None else stop_s
    for k in range(int(seconds * rate)):
        t = k / rate
        moving = 1. <= t < stop_s
        along = x0 + speed * (min(t, stop_s) - 1.) if t >= 1. else x0
        x, y, yaw = GF.from_row(geometry, along, lateral, 0.)
        odom.append((t, x, y, yaw, speed if moving else 0., 0.))
        duty.append((t, .4 if moving else 0.))
    return odom, duty


class GeometryTest(unittest.TestCase):
    def test_centre_line_axis_and_row_end_come_from_the_trunks_of_the_start_lane(self):
        g = GF.row_geometry(metadata(shift=.2))
        self.assertEqual((g['lane'], g['rows']), (0, [0, 1]))
        self.assertAlmostEqual(g['centre_a'], .2, places=9)
        self.assertAlmostEqual(g['axis_rad'], 0., places=9)
        self.assertAlmostEqual(g['half_width_m'], 1.9, places=9)
        self.assertAlmostEqual(g['row_end_s'], 30., places=9)
        self.assertAlmostEqual(g['row_start_s'], 0., places=9)
        self.assertEqual(g['trunks'], 52)
        self.assertEqual(len(g['rocks']), 1)
        posts = [m for m in g['markers'] if m[3] == 'post']
        self.assertEqual(sorted({round(m[0], 2) for m in posts}), [-.6, 9., 18.6, 28.2, 30.6])

    def test_a_sloped_row_gives_a_sloped_axis_and_the_frames_invert(self):
        g = GF.row_geometry(metadata(slope=.02))
        self.assertAlmostEqual(g['axis_rad'], math.atan(.02), places=9)
        x, y, yaw = GF.from_row(g, 7., .4, .1)
        s, d, psi = GF.to_row(g, x, y, yaw)
        self.assertAlmostEqual(s, 7., places=9)
        self.assertAlmostEqual(d, .4, places=9)
        self.assertAlmostEqual(psi, .1, places=9)


class MeasureTest(unittest.TestCase):
    def setUp(self):
        self.g = GF.row_geometry(metadata())

    def test_a_centred_run_to_the_row_end_passes_every_registered_check(self):
        # 0.4 m/s from x = -3: the front bumper reaches the last trunk line (30.0) at centre 29.39 = 80.97 s of drive
        odom, duty = straight_run(self.g, lateral=.05, seconds=92., stop_s=1. + 32.4 / .4)
        m = GF.measure(odom, duty, self.g)
        self.assertTrue(m['drove'])
        self.assertAlmostEqual(m['lateral_rmse_m'], .05, places=3)
        self.assertAlmostEqual(m['lateral_max_m'], .05, places=3)
        self.assertAlmostEqual(m['row_end_error_m'], 29.4 + .61 - 30., places=2)
        self.assertAlmostEqual(m['min_clearance_m'], 1.9 - .05 - .35 - .0707, places=3)   # the nearest post
        self.assertEqual(m['min_clearance_to'], 'post')
        self.assertAlmostEqual(m['distance_m'], 32.4, delta=.1)
        m.update(end_reason='row_end', pauses_after_start=[],
                 after_terminal=GF.after_terminal([(10., 3), (70., 3), (84., 3)], 83., 90.))
        checks = GF.judge('row_end', m)
        self.assertEqual([c['check'] for c in checks if not c['ok']], [])
        self.assertEqual(GF.verdict(checks), 'pass')
        self.assertEqual(GF.verdict(checks, valid=False), 'invalid')

    def test_failures_are_named(self):
        odom, duty = straight_run(self.g, lateral=.6, seconds=60.)             # ends 12 m into the row, 0.6 m off
        m = GF.measure(odom, duty, self.g)
        m.update(end_reason='corridor_lost', pauses_after_start=['stale_lidar'],
                 after_terminal=GF.after_terminal([(10., 3), (57., 9)], 56., 60.))
        failed = {c['check'] for c in GF.judge('row_end', m) if not c['ok']}
        self.assertEqual(failed, {'ends at the row end', 'no pause after the drive started', 'lateral RMSE',
                                  'lateral p95', 'lateral max', 'row end stop position e',
                                  'no commanded steering move after the terminal moment (A22 rule)'})

    def test_contact_and_no_drive(self):
        odom, duty = straight_run(self.g, lateral=1.9 - .35, seconds=40.)       # the side touches the trunk line
        m = GF.measure(odom, duty, self.g)
        self.assertLess(m['min_clearance_m'], 0.)
        still = [(k / 10., -3., 0., 0., 0., 0.) for k in range(50)]
        m = GF.measure(still, [(k / 10., 0.) for k in range(50)], self.g)
        self.assertFalse(m['drove'])
        self.assertNotIn('lateral_rmse_m', m)
        failed = {c['check'] for c in GF.judge('row_end', dict(m, end_reason='corridor_not_acquired'))
                  if not c['ok']}
        self.assertIn('lateral RMSE', failed)
        self.assertIn('ends at the row end', failed)

    def test_offset_start_takes_rmse_after_the_settle_distance_and_the_maximum_from_the_start(self):
        g = self.g
        odom, duty = [], []
        for k in range(30 * 60):
            t = k / 30.
            along = -3. + .4 * t
            lateral = .4 if .4 * t < 3. else 0.
            x, y, yaw = GF.from_row(g, along, lateral, 0.)
            odom.append((t, x, y, yaw, .4, 0.))
            duty.append((t, .4))
        m = GF.measure(odom, duty, g, settle_m=GF.SETTLE_M)
        self.assertAlmostEqual(m['lateral_max_m'], .4, places=3)
        self.assertAlmostEqual(m['lateral_rmse_m'], 0., places=6)

    def test_after_terminal_needs_the_whole_window_and_one_steering_value(self):
        lines = [(1., 0), (5., 40), (9.9, 40), (10.5, 40), (12., 40)]
        self.assertTrue(GF.after_terminal(lines, 10., 13.2)['passed'])
        self.assertFalse(GF.after_terminal(lines, 10., 12.)['complete'])
        self.assertFalse(GF.after_terminal(lines + [(12.5, 0)], 10., 13.2)['passed'])
        self.assertTrue(GF.after_terminal([(1., 7)], 10., 14.)['passed'])                  # no line in the window
        self.assertFalse(GF.after_terminal(lines, None, 14.)['passed'])

    def test_tick_stats_count_the_corridor_over_active_ticks_only(self):
        ticks = [dict(t=0., state='IDLE', corridor=False)]
        ticks += [dict(t=1. + k * .05, state='ACTIVE', T=40, corridor=k % 4 != 0, corridor_reason='insufficient_returns',
                       ctl=['FOLLOW', 'follow'], lidar_age=.1, loop_ms=1.) for k in range(40)]
        ticks += [dict(t=4., state='PAUSED', reason='stale_lidar', corridor=False),
                  dict(t=4.05, state='PAUSED', reason='stale_lidar', corridor=False)]
        s = GF.tick_stats(ticks)
        self.assertEqual((s['active_ticks'], s['corridor_valid_ticks']), (40, 30))
        self.assertEqual(s['corridor_valid_ratio'], .75)
        self.assertEqual(s['pauses_after_start'], [[4., 'stale_lidar']])
        self.assertEqual(s['corridor_invalid_reasons'], {'insufficient_returns': 10})


class OwnTicksTest(unittest.TestCase):
    """Amendment 1: the ticks of an earlier session (same run id, same log folder) are not the run's ticks."""

    def merged(self):
        earlier = [dict(t=100. + k * .05, state='ACTIVE', T=40, corridor=True) for k in range(200)]
        own = [dict(t=230. + k * .05, state='PAUSED', reason='await_hold', corridor=False) for k in range(10)]
        own += [dict(t=230.5 + k * .05, state='ACTIVE', T=40, corridor=True) for k in range(300)]
        return earlier, own

    def test_only_the_ticks_after_the_last_gap_are_the_run(self):
        earlier, own = self.merged()
        self.assertEqual(GF.own_ticks(earlier + own), own)
        self.assertEqual(GF.own_ticks(own), own)
        self.assertEqual(GF.own_ticks([]), [])
        self.assertEqual(GF.tick_stats(earlier + own)['pauses_after_start'], [[230., 'await_hold']])
        self.assertEqual(GF.tick_stats(GF.own_ticks(earlier + own))['pauses_after_start'], [])

    def test_rejudge_uses_the_recorded_truth_metrics_and_the_run_ticks(self):
        earlier, own = self.merged()
        metrics = dict(min_clearance_m=1.4, rest_speed_m_s=0., end_reason='row_end', lateral_rmse_m=.02,
                       lateral_p95_m=.05, lateral_max_m=.06, heading_rmse_deg=.8, heading_max_deg=2., speed_max_m_s=.44,
                       row_end_error_m=.45, pauses_after_start=['await_hold'],
                       after_terminal=dict(passed=True, complete=True))
        report = dict(run=dict(name='nominal_2', expect='row_end'), metrics=metrics, validity=dict(valid=True),
                      verdict='fail', checks=[dict(check='no pause after the drive started', ok=False)])
        result = GF.rejudge(report, dict(ticks=earlier + own))
        self.assertEqual((result['verdict'], result['original_verdict']), ('pass', 'fail'))
        self.assertEqual(result['original_failed'], ['no pause after the drive started'])
        self.assertEqual((result['ticks_recorded'], result['ticks_of_the_run']), (510, 310))
        own[100] = dict(t=own[100]['t'], state='PAUSED', reason='stale_lidar', corridor=False)
        self.assertEqual(GF.rejudge(report, dict(ticks=earlier + own))['verdict'], 'fail')
        self.assertEqual(GF.rejudge(dict(report, validity=dict(valid=False)), dict(ticks=own))['verdict'], 'invalid')


class RegistrationTest(unittest.TestCase):
    def test_the_limits_are_those_of_the_fast_sim_registration(self):
        from steering.safety_scripts import follow_matrix as F
        for key, value in GF.CRITERIA.items():
            self.assertEqual(value, F.CRITERIA[key], key)
        self.assertEqual((GF.FRONT_BUMPER_M, GF.HALF_WIDTH_M, GF.SETTLE_M),
                         (F.FRONT_BUMPER_M, F.HALF_WIDTH_M, F.SETTLE_M))
        self.assertEqual(GF.CONTROLLER, F.CONTROLLER)

    def test_the_registration_file_is_the_module_registration(self):
        path = ROOT / 'docs' / 'steering' / 'PREREG_STEER_FOLLOW_GSIM.json'
        if not path.exists():
            self.skipTest('the registration file is added by its own commit')
        self.assertEqual(json.loads(path.read_bytes().decode('utf-8')), json.loads(json.dumps(GF.prereg())))


# --- exec_run -----------------------------------------------------------------------------------------------------------
class FakeOp:
    def __init__(self, arm_refusals=0):
        self.posts = []
        self.arm_refusals = arm_refusals

    def post(self, route, body=None, timeout_s=None):
        self.posts.append(route)
        if route == '/api/arm' and self.arm_refusals > 0:
            self.arm_refusals -= 1
            return 409, dict(ok=False, error='arm_precheck:steering_fault:steering_status_stale'), 0., 0.
        return 200, dict(ok=True), 0., 0.


class FakeOnePressOp(FakeOp):
    """An executor page in start_mode one_press: /api/status says so; /api/go is refused go_refusals times
    (inputs_not_fresh), or always with session_conflict."""

    def __init__(self, go_refusals=0, conflict=False):
        super().__init__()
        self.go_refusals, self.conflict = go_refusals, conflict
        self.go_sessions, self.beat_sessions = [], []

    def request(self, method, route, body=None, timeout_s=None):
        return 200, dict(start_mode='one_press'), 0., 0.

    def post(self, route, body=None, timeout_s=None):
        if route == '/api/heartbeat':
            self.beat_sessions.append((body or {}).get('session'))
        if route == '/api/go':
            self.go_sessions.append((body or {}).get('session'))
            self.posts.append(route)
            if self.conflict:
                return 409, dict(ok=False, error='session_conflict'), 0., 0.
            if self.go_refusals > 0:
                self.go_refusals -= 1
                return 409, dict(ok=False, error='inputs_not_fresh', blockers=['inputs_not_fresh']), 0., 0.
            return 200, dict(ok=True), 0., 0.
        return super().post(route, body, timeout_s)


class FakeHold:
    def __init__(self):
        self.events = []

    def press(self):
        self.events.append('press')

    def release(self):
        self.events.append('release')

    def lose(self):
        self.events.append('lose')

    def close(self):
        self.events.append('close')


class FakeHooks:
    def __init__(self, statuses):
        self.statuses = list(statuses)
        self.requests = []

    def _status(self):
        return self.statuses.pop(0) if len(self.statuses) > 1 else self.statuses[0]

    def call(self, op, **fields):
        return dict(ok=True, status=self._status())

    def request(self, payload):
        self.requests.append(payload)
        return dict(ok=True)


def status(state, reason=None, client='ARMED', end_reason=None):
    return dict(runner=dict(state=state, reason=reason, run_id=1, end_reason=end_reason), client=dict(state=client))


class ExecRunTest(unittest.TestCase):
    def run_start(self, statuses, **kw):
        op, hold, hooks = FakeOp(kw.pop('arm_refusals', 0)), FakeHold(), FakeHooks(statuses)
        operator = exec_run.Operator('unused', op=op, hold=hold, hooks=hooks)
        lines = []
        code, summary = exec_run.start_run(operator, {'kind': 'row_follow'}, out=lines.append, period_s=0.,
                                           sleep=lambda s: None, tick_reader=lambda d: None, **kw)
        return code, summary, op, hold, hooks, lines

    def test_start_arms_presses_once_and_ends_with_the_row_end(self):
        seq = [status('IDLE', client='IDLE')] * 3 + [status('ACTIVE', 'follow')] * 3 + [
            status('ENDED', 'row_end', end_reason='row_end')]
        code, summary, op, hold, hooks, lines = self.run_start(seq, arm_refusals=1)
        self.assertEqual(code, exec_run.EXIT_ROW_END)
        self.assertEqual(hooks.requests, [dict(op='controller', spec={'kind': 'row_follow'})])
        self.assertEqual(op.posts, ['/api/start', '/api/arm', '/api/arm'])
        self.assertEqual(hold.events, ['press', 'release', 'close'])
        self.assertTrue(summary['seen_active'])
        self.assertTrue(any('arm attempt 1 refused' in line for line in lines))
        self.assertTrue(lines[-1].startswith('ended: row_end'))

    def test_an_open_run_is_not_started_again(self):
        code, summary, op, hold, hooks, lines = self.run_start([status('ACTIVE', 'follow')])
        self.assertEqual((code, op.posts, hold.events), (exec_run.EXIT_NO_START, [], []))

    def test_a_latched_executor_is_acknowledged_and_a_refused_arm_ends_the_request(self):
        seq = [status('ENDED', 'operator_stop', client='LATCHED', end_reason='operator_stop')]
        code, summary, op, hold, hooks, lines = self.run_start(seq, arm_refusals=99)
        self.assertEqual(code, exec_run.EXIT_NO_START)
        self.assertEqual(op.posts[:2], ['/api/ack', '/api/start'])
        self.assertEqual(op.posts[-1], '/api/end')
        self.assertEqual(hold.events, [])

    def test_a_run_that_ends_otherwise_is_reported(self):
        seq = [status('IDLE', client='IDLE')] * 3 + [status('ACTIVE', 'follow'),
                                                    status('ENDED', 'x', end_reason='obstacle_ahead')]
        code, summary, op, hold, hooks, lines = self.run_start(seq)
        self.assertEqual(code, exec_run.EXIT_OTHER_END)
        self.assertEqual(summary['runner']['end_reason'], 'obstacle_ahead')

    def one_press_start(self, statuses, op):
        hooks = FakeHooks(statuses)
        operator = exec_run.Operator('unused', op=op, hooks=hooks)     # no hold given: the start mode decides
        lines = []
        try:
            code, summary = exec_run.start_run(operator, {'kind': 'row_follow'}, out=lines.append, period_s=0.,
                                               sleep=lambda s: None, tick_reader=lambda d: None)
        finally:
            operator.hold.close()
        return code, summary, operator, lines

    def test_one_press_executor_gets_heartbeats_and_one_go(self):
        op = FakeOnePressOp(go_refusals=1)
        seq = [status('IDLE', client='IDLE')] * 3 + [status('ACTIVE', 'follow')] * 3 + [
            status('ENDED', 'row_end', end_reason='row_end')]
        code, summary, operator, lines = self.one_press_start(seq, op)
        self.assertEqual(code, exec_run.EXIT_ROW_END)
        self.assertEqual(summary['start_mode'], 'one_press')
        self.assertIsInstance(operator.hold, exec_run.GoBeater)
        self.assertEqual([p for p in op.posts if p in ('/api/go', '/api/hold')], ['/api/go', '/api/go'])
        self.assertIn('/api/heartbeat', op.posts)
        self.assertEqual(op.go_sessions[0], op.beat_sessions[0])            # one screen: one session id
        self.assertTrue(any(line.startswith('started (one press)') for line in lines))

    def test_one_press_go_refused_by_another_screen_ends_the_request(self):
        op = FakeOnePressOp(conflict=True)
        code, summary, operator, lines = self.one_press_start([status('IDLE', client='IDLE')], op)
        self.assertEqual((code, summary['error']), (exec_run.EXIT_NO_START, 'go_refused:session_conflict'))
        self.assertEqual(op.posts.count('/api/go'), 1)                      # a session conflict is not retried
        self.assertEqual(op.posts[-1], '/api/end')

    def test_stop_posts_the_operator_stop(self):
        op = FakeOp()
        operator = exec_run.Operator('unused', op=op, hold=FakeHold(), hooks=FakeHooks([status('ACTIVE')]))
        self.assertEqual(exec_run.stop_run(operator, out=lambda line: None), 0)
        self.assertEqual(op.posts, ['/api/stop'])

    def test_status_line_and_last_tick(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / 'log'
            log.mkdir()
            rows = [dict(k='tick', t=1., lat=.01), dict(k='event', event='x'), dict(k='tick', t=2., lat=.031, head=-1.2,
                                                                                  corridor=True, ctl=['FOLLOW', 'follow'])]
            (log / 'row_executor_a.jsonl').write_bytes(b''.join(json.dumps(r).encode('utf-8') + bytes([10])
                                                                for r in rows) + b'{"k": "tick", "t"')
            tick = exec_run.last_tick(tmp)
            self.assertEqual(tick['t'], 2.)
            line = exec_run.status_line(3., status('ACTIVE', 'follow'), tick)
            self.assertIn('lat=+3.1 cm', line)
            self.assertIn('head=-1.2 deg', line)
            self.assertIn('corridor=ok', line)
            self.assertIsNone(exec_run.last_tick(Path(tmp) / 'none'))

    def test_start_pose_of_the_scene_with_an_offset(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / 'world_metadata.json').write_bytes(json.dumps(metadata()).encode('utf-8'))
            (Path(tmp) / 'scene_manifest.json').write_bytes(json.dumps(
                dict(world_summary=dict(spawn_height=dict(z_m=.23)))).encode('utf-8'))
            self.assertEqual(exec_run.start_pose(tmp), (-3., 0., .23, 0.))
            x, y, z, yaw = exec_run.start_pose(tmp, lateral_m=.4)
            self.assertEqual((x, y, yaw), (-3., .4, 0.))
            self.assertAlmostEqual(z, .23 + exec_run.OFFSET_DROP_M, places=9)


if __name__ == '__main__':
    unittest.main()
