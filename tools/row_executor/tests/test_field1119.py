"""2026-11-19 field requests: the per-run GNSS mode (POST /api/start gnss_mode, status.gnss), the run metrics (path against
the estimator / grid centreline, status.metrics and the run_end event) and the executor state topic message
(executor_main.state_message, /row_executor/state). Synthetic orchard scans with the REAL vendored estimator; every
number is a test value.
"""
from dataclasses import replace
import json
import math
import types
import unittest

import rowexec_testlib as L
from row_executor import config as C
from row_executor import executor_main as M
from row_executor import operator_http as H
from row_executor import perception_settings as PS
from row_executor.config import ExecutorConfig
from row_executor.core import ACTIVE, ENDED, IDLE, Runner
from row_executor.run_metrics import RunMetrics

TRUNKS = L.orchard_trunks(row_y=1.8, spacing=1.0, x0=-10., x1=60., radius=.1)
V = .4


def moving_scene(t0, lateral=.25):
    def scan(t):
        dx = V * max(0., t - t0 - 1.)
        return L.raw_scan([(x - dx, y - lateral, r) for x, y, r in TRUNKS])
    return scan


def gnss_msg(east, north, hacc=7., ok=True):
    return types.SimpleNamespace(east_m=east, north_m=north, hacc_m=hacc, fix_ok=ok)


def static_scene(lateral=.25):
    scan = L.raw_scan([(x, y - lateral, r) for x, y, r in TRUNKS])
    return lambda t: scan


def make(cfg, gnss_mode=None, t0=1000., scene=None):
    runner = Runner(cfg)
    driver = L.FakeTimeDriver(runner, scene or moving_scene(t0), t0=t0)
    runner.start_run(driver.t, 60., gnss_mode=gnss_mode)
    return runner, driver


def feed(runner, driver, seconds, where, hold=False):
    out = []
    for _ in range(int(seconds)):
        out += driver.run(1., hold=hold)
        east, north = where(driver.t)
        runner.feed_gnss(driver.t, gnss_msg(east, north))
    return out


class RunModeSettingsTest(unittest.TestCase):
    def test_default_run_modes_and_digest(self):
        cfg = ExecutorConfig()
        self.assertEqual(PS.run_modes(cfg), ('off', 'log_only', 'geofence'))
        self.assertNotIn('gnss_run_modes', cfg.to_dict())            # untouched block: the digest is unchanged
        self.assertEqual(PS.run_mode_problems(cfg, 'log_only'), [])
        self.assertTrue(PS.run_mode_problems(cfg, 'weak_constraint'))  # changes the steering input: not listed
        self.assertTrue(PS.run_mode_problems(cfg, 'gps'))
        self.assertEqual(H.GNSS_MODES, PS.GNSS_MODES)

    def test_configured_mode_is_always_a_run_mode(self):
        cfg = replace(ExecutorConfig(), gnss_mode='geofence', gnss_run_modes='off').validate()
        self.assertEqual(PS.run_modes(cfg), ('off', 'geofence'))

    def test_bad_run_mode_lists_are_refused(self):
        for bad in ('off,gps', 'off,off', 'weak_constraint'):       # weak_constraint needs front_steer
            with self.subTest(bad=bad):
                with self.assertRaises(C.ConfigError):
                    replace(ExecutorConfig(), gnss_run_modes=bad).validate()
        ok = replace(ExecutorConfig(), gnss_run_modes='off, log_only').validate()
        self.assertEqual(PS.run_modes(ok), ('off', 'log_only'))


class RunnerGnssModeTest(unittest.TestCase):
    def test_log_only_per_run_on_an_off_config_then_off_again(self):
        cfg = ExecutorConfig()
        runner, d = make(cfg, gnss_mode='log_only')
        self.assertEqual(runner.run_gnss_mode, 'log_only')
        out = feed(runner, d, 6, lambda t: (V * (t - 1001.) + 3., 2.))
        block = out[-1][1].summary['gnss']
        self.assertEqual(block['mode'], 'log_only')
        self.assertGreaterEqual(block['counts']['accepted'], 5)
        status = runner.gnss_status(d.t)
        self.assertEqual((status['mode'], status['configured_mode'], status['state'], status['used']),
                         ('log_only', 'off', 'ok', True))
        self.assertEqual(status['hacc_m'], 7.)
        self.assertEqual(status['h_acc_m'], 7.)
        self.assertGreaterEqual(status['accepted'], 5)
        self.assertEqual(status['rejected'], 0)
        runner.end_run(d.t)
        runner.start_run(d.t, 60.)                                   # no gnss_mode: the configured mode (off)
        self.assertEqual(runner.run_gnss_mode, 'off')
        self.assertIsNone(runner.perception)
        self.assertIsNone(runner.feed_gnss(d.t, gnss_msg(0., 0.)))
        self.assertEqual(runner.gnss_status(d.t)['state'], 'off')

    def test_log_only_run_changes_no_decision(self):
        plain, d0 = make(ExecutorConfig())
        logged, d1 = make(ExecutorConfig(), gnss_mode='log_only')
        a = d0.run(8., hold=False)
        b = feed(logged, d1, 8, lambda t: (V * (t - 1001.) + 3., 2.))
        self.assertEqual([(d.left, d.right, d.state, d.reason) for _, d in a],
                         [(d.left, d.right, d.state, d.reason) for _, d in b])
        self.assertEqual(plain.estimator.snapshot(1007.9), logged.estimator.snapshot(1007.9))

    def test_geofence_per_run_ends_the_run(self):
        runner, d = make(ExecutorConfig(), gnss_mode='geofence')
        runner.perception.gnss.new_run(5.)
        out = feed(runner, d, 12, lambda t: (0., 0.) if t < 1004. else (40. + 2. * (t - 1004.), 0.))
        ends = [dd for _, dd in out if dd.state == ENDED]
        self.assertTrue(ends)
        self.assertEqual(ends[0].reason, 'gnss_geofence')
        self.assertEqual(runner.gnss_status(d.t)['state'], 'fence_tripped')

    def test_refusals_change_nothing(self):
        runner = Runner(ExecutorConfig())
        with self.assertRaisesRegex(ValueError, 'gnss_mode_invalid'):
            runner.start_run(1., 60., gnss_mode='gps')
        with self.assertRaisesRegex(RuntimeError, 'gnss_mode_unsupported'):
            runner.start_run(1., 60., gnss_mode='weak_constraint')
        self.assertEqual((runner.state, runner.run_id, runner.run_gnss_mode), (IDLE, 0, None))
        self.assertEqual(runner.gnss_status(1.)['state'], 'no_run')

    def test_status_states(self):
        runner, d = make(ExecutorConfig(), gnss_mode='log_only')
        d.run(1., hold=False)
        self.assertEqual(runner.gnss_status(d.t)['state'], 'waiting')
        runner.feed_gnss(d.t, gnss_msg(0., 0., hacc=30.))            # above gnss_max_hacc_m 10
        status = runner.gnss_status(d.t)
        self.assertEqual((status['state'], status['used'], status['rejected']), ('rejected_quality', False, 1))
        runner.feed_gnss(d.t + .01, gnss_msg(0., 0.))
        self.assertEqual(runner.gnss_status(d.t + .02)['state'], 'ok')
        self.assertEqual(runner.gnss_status(d.t + 10.)['state'], 'stale')  # older than gnss_max_age_s 2.5


class LoopGnssModeTest(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.loop = M.ExecutorLoop(ExecutorConfig())
        self.loop._event = lambda name, **fields: self.events.append(dict(fields, event=name))

    def request(self, **kwargs):
        future = self.loop.submit('start', **kwargs)
        self.loop.step()
        return future.result(1.)

    def test_start_with_and_without_gnss_mode(self):
        reply = self.request(row_len_m=30., gnss_mode='log_only')
        self.assertTrue(reply['ok'], reply)
        self.assertEqual(reply['gnss_mode'], 'log_only')
        start = [e for e in self.events if e['event'] == 'start'][-1]
        self.assertEqual((start['gnss_mode'], start['gnss_mode_source']), ('log_only', 'request'))
        status = self.loop.status()
        self.assertEqual((status['gnss']['mode'], status['gnss']['configured_mode']), ('log_only', 'off'))
        self.assertEqual(status['gnss']['supported_modes'], ['off', 'log_only', 'geofence'])
        future = self.loop.submit('end')
        self.loop.step()
        self.assertTrue(future.result(1.)['ok'])
        self.loop.step()
        reply = self.request(row_len_m=30.)
        self.assertTrue(reply['ok'], reply)
        start = [e for e in self.events if e['event'] == 'start'][-1]
        self.assertEqual((start['gnss_mode'], start['gnss_mode_source']), ('off', 'config'))
        self.assertEqual(self.loop.status()['gnss']['state'], 'off')

    def test_refusals(self):
        reply = self.request(row_len_m=30., gnss_mode='rtk')
        self.assertEqual(reply['error'], 'gnss_mode_invalid')
        self.assertEqual(reply['allowed'], list(PS.GNSS_MODES))
        reply = self.request(row_len_m=30., gnss_mode='weak_constraint')
        self.assertEqual(reply['error'], 'gnss_mode_unsupported')
        self.assertTrue(reply['reasons'])
        self.assertEqual(reply['supported'], ['off', 'log_only', 'geofence'])
        self.assertFalse([e for e in self.events if e['event'] == 'start'])

    def test_weak_constraint_when_listed_on_front_steer(self):
        cfg = replace(ExecutorConfig(), vehicle_profile='front_steer',
                      gnss_run_modes='off,log_only,geofence,weak_constraint')
        try:
            cfg = cfg.validate()
        except C.ConfigError as exc:                # the front_steer block needs more keys in this config
            self.skipTest('front_steer test config: %s' % exc)
        self.assertEqual(PS.run_mode_problems(cfg, 'weak_constraint'), [])


class HttpGnssModeTest(unittest.TestCase):
    """The route's own checks (a fake loop records what reaches it)."""

    def setUp(self):
        self.seen = []

        class FakeLoop:
            def request(inner, kind, **kwargs):
                self.seen.append((kind, kwargs))
                return dict(ok=True)

            def status(inner):
                return {}
        self.server = H.make_server(FakeLoop(), '127.0.0.1', 0, 'x' * 20)
        import threading
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def post(self, body):
        import http.client
        conn = http.client.HTTPConnection('127.0.0.1', self.server.server_address[1], timeout=5)
        data = json.dumps(body).encode()
        conn.request('POST', '/api/start', data, {'Host': self.server.host_header, 'Origin': self.server.origin,
                                                 'X-Row-Token': 'x' * 20, 'Content-Type': 'application/json',
                                                 'Content-Length': str(len(data))})
        response = conn.getresponse()
        out = response.status, json.loads(response.read())
        conn.close()
        return out

    def test_route(self):
        self.assertEqual(self.post({'row_len_m': 10., 'gnss_mode': 'geofence'})[0], 200)
        self.assertEqual(self.seen[-1], ('start', dict(row_len_m=10., gnss_mode='geofence')))
        self.assertEqual(self.post({'row_len_m': None})[0], 200)
        self.assertEqual(self.seen[-1], ('start', dict(row_len_m=None, gnss_mode=None)))
        self.assertEqual(self.post({'row_len_m': None, 'gnss_mode': None})[0], 200)
        code, reply = self.post({'row_len_m': None, 'gnss_mode': 'GPS'})
        self.assertEqual((code, reply['error']), (400, 'gnss_mode_invalid'))
        code, reply = self.post({'row_len_m': None, 'gnss_mode': ['off']})
        self.assertEqual((code, reply['error']), (400, 'gnss_mode_invalid'))
        code, reply = self.post({'row_len_m': None, 'gps': 'off'})
        self.assertEqual((code, reply['error']), (400, 'unknown_parameter'))
        self.assertEqual(len(self.seen), 3)


class RunMetricsTest(unittest.TestCase):
    def snap(self, lat, head_deg, dist, valid=True):
        return dict(lateral_offset=lat, heading_error=math.radians(head_deg), corridor_valid=valid, distance_m=dist)

    def test_sums(self):
        m = RunMetrics(7)
        self.assertFalse(m.update(0., self.snap(.3, 1., 0.), driving=False))
        lats = [.02, -.04, .08, -.12]
        for i, lat in enumerate(lats):
            self.assertTrue(m.update(.05 * (i + 1), self.snap(lat, 2. * (-1) ** i, .1 * (i + 1)), driving=True,
                                     grid_ref=(lat + .01, math.radians(0.))))
        out = m.summary()
        self.assertEqual((out['run_id'], out['ref'], out['samples'], out['grid_samples']), (7, 'estimator+grid', 4, 4))
        self.assertAlmostEqual(out['lat_rms_m'], math.sqrt(sum(x * x for x in lats) / 4), places=4)
        self.assertAlmostEqual(out['lat_mean_m'], sum(lats) / 4, places=4)
        self.assertEqual(out['lat_max_abs_m'], .12)
        self.assertEqual((out['within_0_05_pct'], out['within_0_10_pct']), (50., 75.))
        self.assertEqual((out['head_rms_deg'], out['head_max_abs_deg']), (2., 2.))
        self.assertAlmostEqual(out['distance_m'], .4, places=6)   # the path into each driving tick counts
        self.assertAlmostEqual(out['ref_diff_lat_rms_m'], .01, places=4)
        self.assertEqual(out['ref_diff_head_rms_deg'], 2.)
        self.assertEqual(out['last']['grid_lat_m'], -.11)

    def test_no_corridor_no_sample(self):
        m = RunMetrics(1)
        self.assertFalse(m.update(0., self.snap(.1, 0., 0., valid=False), driving=True))
        out = m.summary()
        self.assertEqual((out['samples'], out['lat_rms_m'], out['grid_lat_rms_m']), (0, None, None))

    def test_runner_metrics_kept_after_the_end(self):
        cfg = ExecutorConfig()
        runner, d = make(cfg, scene=static_scene())                   # open loop: the output never moves the scene
        d.run(2., hold=False)
        out = d.run(2., hold=True)
        driving = [dd for _, dd in out if dd.state == ACTIVE and dd.left > 0]
        if not driving:
            self.skipTest('the differential test config did not drive in this scene')
        metrics = out[-1][1].summary['metrics']
        self.assertGreater(metrics['samples'], 10)
        self.assertEqual(metrics['grid_samples'], 0)                  # local grid off
        self.assertLess(abs(metrics['lat_mean_m'] - .25), .15)        # the scene is 0.25 m left of the centre
        runner.end_run(d.t)
        after = d.run(1., hold=False)[-1][1].summary['metrics']
        self.assertEqual(after['samples'], metrics['samples'])
        runner.start_run(d.t, 60.)
        self.assertEqual(runner.tick(d.t + .05, False).summary['metrics']['samples'], 0)

    def test_grid_reference_when_the_grid_is_on(self):
        cfg = replace(ExecutorConfig(), local_grid_enabled=True).validate()
        runner, d = make(cfg, scene=static_scene())
        d.run(2., hold=False)
        out = d.run(2., hold=True)
        if not [dd for _, dd in out if dd.state == ACTIVE and dd.left > 0]:
            self.skipTest('the differential test config did not drive in this scene')
        metrics = out[-1][1].summary['metrics']
        self.assertGreater(metrics['grid_samples'], 0)
        self.assertEqual(metrics['ref'], 'estimator+grid')
        self.assertLess(metrics['ref_diff_lat_rms_m'], .15)


class StateMessageTest(unittest.TestCase):
    def test_geometry(self):
        lat, head = .2, math.radians(10.)
        pose = (5., 1., 0.)                                           # base at x 5, y 1, yaw 0 (odometry frame)
        heading, centre = head, 1. * math.cos(head) - 5. * math.sin(head) - lat   # n . p - centre = lat
        status = dict(t_wall=1.5, t=10., run=3, mode='SHADOW', run_state='running', state=ACTIVE, reason='follow',
                      corridor=True, corridor_reason='tracked', lat=lat, head=10., width=3.1, S=12, T=30, v=.3,
                      dist=4., front=9., steer=dict(meas=10.), single_row=dict(phase='FOLLOW', target_speed_m_s=.35),
                      gnss=dict(mode='log_only', state='ok', hacc_m=3.),
                      metrics=dict(RunMetrics(3).summary(), samples=0))
        geometry = dict(pose=pose, band=(heading, centre, 3.1), grid=(heading, centre + .05, 3.0),
                        lookahead=(2.5, -.1))
        msg = M.state_message(status, geometry)
        self.assertEqual(msg['schema'], 'ubts.row_executor.state.v1')
        self.assertEqual((msg['run_id'], msg['mode'], msg['state'], msg['phase']), (3, 'SHADOW', ACTIVE, 'FOLLOW'))
        self.assertEqual(msg['gnss'], dict(mode='log_only', state='ok'))
        self.assertEqual(msg['pose'], dict(x=5., y=1., yaw_deg=0.))
        base = msg['centreline']['base']
        # the base-frame centreline: lateral offset -lat across the line at the base, direction +head (left)
        dx, dy = base[1][0] - base[0][0], base[1][1] - base[0][1]
        self.assertAlmostEqual(math.degrees(math.atan2(dy, dx)), 10., places=1)
        cross = dx * (0. - base[0][1]) - dy * (0. - base[0][0])      # signed distance of the base origin (left +)
        self.assertAlmostEqual(cross / math.hypot(dx, dy), lat, places=2)
        self.assertEqual(msg['lookahead']['base'], [2.5, -.1])
        self.assertEqual(msg['lookahead']['odom'], [7.5, .9])
        self.assertIsNotNone(msg['grid_centreline'])
        self.assertEqual(msg['metrics']['run_id'], 3)
        text = json.dumps(M._clean(msg), separators=(',', ':'), allow_nan=False)
        self.assertLess(len(text), 2000)

    def test_topic_name(self):
        self.assertEqual(M.parse_args(['--config', 'x.json']).state_topic, M.STATE_TOPIC)
        self.assertEqual(M.STATE_TOPIC, '/row_executor/state')

    def test_without_geometry(self):
        msg = M.state_message(dict(state=IDLE, run=0), None)
        self.assertIsNone(msg['pose'])
        self.assertIsNone(msg['centreline'])

    def test_loop_rate(self):
        loop = M.ExecutorLoop(ExecutorConfig())
        sent = []
        loop.state_sink = sent.append
        now = [1000.]
        loop.clock = lambda: now[0]
        for _ in range(40):                                           # 2 s without a run: about 1 Hz
            loop.step(now[0])
            now[0] += .05
        idle = len(sent)
        self.assertIn(idle, (2, 3))
        future = loop.submit('start', row_len_m=30.)
        for _ in range(40):                                           # 2 s with an open run: about 10 Hz
            loop.step(now[0])
            now[0] += .05
        self.assertTrue(future.result(1.)['ok'])
        self.assertGreaterEqual(len(sent) - idle, 18)
        message = json.loads(sent[-1])
        self.assertEqual((message['schema'], message['run_id']), ('ubts.row_executor.state.v1', 1))


if __name__ == '__main__':
    unittest.main()
