"""One-press autonomous start and the operator link (design v2 V2-2, decision R32, docs/steering/OPERATOR_API_KO.md).

OperatorLink on its own (session binding, conflict, expiry once, no revival, new go = new serial); the ExecutorLoop in
start_mode one_press on the virtual clock of test_executor_loop (SHADOW and MOTOR with the real RoverControl behind the
real BridgeClient): go starts the run, the link timeout pauses it with operator_link_lost (zero output the same tick), a
heartbeat never resumes it, a new go does, another session is refused, STOP wins and ends the go, the hold endpoints are
refused in one_press and the go endpoints in hold_to_run, the link pauses count with the stale pauses; the contract
status fields; the HTTP routes over a loopback socket. The T1 executor-in-the-loop scripts are in the safety catalogue
(operator_link_lost, operator_link_back_no_restart, other_screen_refused, one_press_stop). Test numbers only.
"""
import http.client
import json
import threading
import time
import unittest

import rowexec_testlib  # noqa: F401  (path setup)
from row_executor import config as C
from row_executor import executor_main as M
from row_executor import operator_http as H
from row_executor.bridge_client import ARMED, LATCHED
from row_executor.core import ACTIVE, ENDED, PAUSED
from test_executor_loop import Rig, legacy_sensor_config, motor_config

TOKEN = 'test-token-0123456789abcdef'


def one_press(**over):
    return legacy_sensor_config(start_mode='one_press', **over).validate()


class OperatorLinkTest(unittest.TestCase):
    def test_binding_conflict_and_expiry(self):
        link = M.OperatorLink(.5)
        self.assertEqual(link.heartbeat('aaaa1', 0.), (True, None))
        self.assertEqual(link.heartbeat('bbbb2', .2), (False, 'session_conflict'))     # the first one is fresh
        self.assertEqual(link.heartbeat('aaaa1', .4), (True, None))
        ok, why, serial = link.go('bbbb2', .5)
        self.assertEqual((ok, why), (False, 'session_conflict'))
        ok, why, serial = link.go('aaaa1', .5)
        self.assertTrue(ok)
        self.assertEqual(link.current(.9), (True, serial, False))
        # no heartbeat for more than 0.5 s: the go drops for good and 'lost' is reported once
        self.assertEqual(link.current(1.01), (False, serial, True))
        self.assertEqual(link.current(1.02), (False, serial, False))
        self.assertEqual(link.heartbeat('aaaa1', 1.1), (True, None))               # the link is back ...
        self.assertEqual(link.current(1.15), (False, serial, False))              # ... the go is not
        ok, _, serial2 = link.go('aaaa1', 1.2)
        self.assertTrue(ok)
        self.assertEqual(serial2, serial + 1)

    def test_an_expired_session_may_be_replaced_and_never_inherits_the_go(self):
        link = M.OperatorLink(.5)
        link.go('aaaa1', 0.)
        self.assertEqual(link.heartbeat('bbbb2', .7), (True, None))      # the first screen went away (page reload)
        active, _, lost = link.current(.7)
        self.assertFalse(active)
        self.assertTrue(lost)                                             # the old go was dropped by the expiry
        self.assertEqual(link.snapshot(.7)['bindings'], 2)

    def test_lost_is_kept_until_the_main_loop_reads_it(self):
        link = M.OperatorLink(.5)
        link.go('aaaa1', 0.)
        link.heartbeat('aaaa1', .8)                  # an HTTP heartbeat expires the go first ...
        self.assertTrue(link.current(.8)[2])         # ... the main loop still sees the loss
        link.go('aaaa1', 1.)
        link.cancel('operator_stop')
        self.assertEqual(link.current(1.)[:1], (False,))
        self.assertEqual(link.snapshot(1.)['last_cancel'], 'operator_stop')

    def test_invalid_session_ids(self):
        link = M.OperatorLink(.5)
        for bad in (None, 'abc', 'x' * 65, 7):
            self.assertEqual(link.heartbeat(bad, 0.), (False, 'invalid_session_id'))
        self.assertNotIn('aaaa1', json.dumps(link.snapshot(0.)))


class ConfigTest(unittest.TestCase):
    def test_effective_start_mode(self):
        self.assertEqual(C.effective_start_mode(C.ExecutorConfig()), 'hold_to_run')
        self.assertEqual(C.effective_start_mode(C.ExecutorConfig(start_mode='one_press')), 'one_press')
        steer = C.ExecutorConfig(vehicle_profile='front_steer', mode='single_row', require_native_navigation=False,
                                 max_yaw_rate_deg_s=3.6)
        self.assertEqual(C.effective_start_mode(steer), 'one_press')
        self.assertEqual(C.effective_start_mode(C.ExecutorConfig(vehicle_profile='front_steer', mode='single_row',
                                                                 require_native_navigation=False, max_yaw_rate_deg_s=3.6,
                                                                 start_mode='hold_to_run')), 'hold_to_run')

    def test_rules_and_untouched_digest(self):
        with self.assertRaises(C.ConfigError):
            C.ExecutorConfig(start_mode='press_twice').validate()
        for value in (.4, 3.5):
            with self.assertRaises(C.ConfigError):
                C.ExecutorConfig(operator_timeout_s=value).validate()
        C.ExecutorConfig(operator_timeout_s=3.).validate()
        self.assertNotIn('start_mode', C.ExecutorConfig().to_dict())
        self.assertIn('start_mode', C.ExecutorConfig(start_mode='hold_to_run').to_dict())
        loaded = C.from_dict(C.ExecutorConfig(start_mode='one_press', operator_timeout_s=1.).to_dict())
        self.assertEqual((loaded.start_mode, loaded.operator_timeout_s), ('one_press', 1.))


class Driver:
    """A Rig (test_executor_loop) with the operator screen of one_press: heartbeats every tick while visible."""

    def __init__(self, cfg, motor=False):
        self.rig = Rig(cfg, motor=motor)
        self.loop = self.rig.loop
        self.session = 'screen-one-0001'
        self.visible = True

    def run(self, seconds):
        out = []
        for _ in range(int(round(seconds / .05))):
            if self.visible:
                self.loop.operator_heartbeat(self.session)
            out.extend(self.rig.run(.05))
        return out

    def request(self, kind, **kw):
        future = self.loop.submit(kind, **kw)
        self.run(.05)
        return future.result()

    def go(self, session=None):
        return self.request('go', session=session or self.session)


class LoopTest(unittest.TestCase):
    def started(self, cfg=None, motor=False):
        d = Driver(cfg or one_press(), motor=motor)
        self.assertTrue(d.request('start', row_len_m=30.)['ok'])
        d.run(1.6)                                   # inputs fresh for fresh_recover_s, corridor acquired
        return d

    def test_go_runs_and_a_link_loss_pauses_until_a_new_go(self):
        d = self.started()
        status = d.loop.status()
        self.assertEqual(status['start_mode'], 'one_press')
        self.assertEqual(status['run_state'], 'armed')               # SHADOW: no Arm step
        self.assertTrue(status['go_ready'], status['go_blockers'])
        self.assertTrue(d.go()['ok'])
        out = d.run(1.)
        self.assertEqual(d.loop.runner.state, ACTIVE)
        self.assertTrue(any(x.left or x.right for x in out))
        self.assertEqual(d.loop.status()['run_state'], 'running')
        d.visible = False                            # the screen is hidden: the heartbeats stop
        out = d.run(.75)
        paused = [k for k, x in enumerate(out) if x.state == PAUSED]
        self.assertTrue(paused)
        self.assertEqual(d.loop.runner.reason, 'operator_link_lost')
        k = paused[0]
        self.assertEqual((out[k].left, out[k].right), (0, 0))       # zero output on the pause tick
        self.assertLessEqual((k + 1) * .05, .5 + .05 + 1e-9)          # within the timeout + one tick
        status = d.loop.status()
        self.assertEqual((status['run_state'], status['stop_reason'], status['stop_detail']),
                         ('paused', 'OPERATOR_LINK_LOST', 'operator_link_lost'))
        self.assertFalse(status['operator_link']['fresh'])
        self.assertEqual(d.loop.runner.stale_events, 1)               # counted with the stale pauses (V2-2 item 5)
        d.visible = True                                              # the screen is back: no restart by itself
        out = d.run(2.)
        self.assertTrue(all(x.state == PAUSED and not (x.left or x.right) for x in out))
        self.assertTrue(d.go()['ok'])
        d.run(.5)
        self.assertEqual(d.loop.runner.state, ACTIVE)

    def test_link_pauses_end_the_run_after_max_stale_events(self):
        d = self.started()
        for _ in range(d.loop.config.max_stale_events + 1):
            self.assertTrue(d.go()['ok'])
            d.run(.5)
            d.visible = False
            d.run(.7)
            d.visible = True
            d.run(1.2)
        self.assertEqual(d.loop.runner.state, ENDED)
        self.assertEqual(d.loop.runner.end_reason, 'stale_repeat_limit')

    def test_other_session_refused_and_stop_wins(self):
        d = self.started()
        self.assertTrue(d.go()['ok'])
        d.run(.5)
        self.assertEqual(d.loop.operator_heartbeat('screen-two-0002'), (False, 'session_conflict'))
        reply = d.go('screen-two-0002')
        self.assertEqual(reply['error'], 'session_conflict')
        self.assertEqual(d.loop.runner.state, ACTIVE)                  # the first screen's run goes on
        result = d.loop.emergency_stop('page')                         # STOP from any screen, any session
        self.assertTrue(result['ok'])
        d.run(.1)
        self.assertEqual(d.loop.runner.end_reason, 'operator_stop')
        self.assertFalse(d.loop.link.go_active)
        self.assertEqual(d.go()['error'], 'no_run')

    def test_go_refused_before_start_and_while_inputs_warm_up(self):
        d = Driver(one_press())
        self.assertEqual(d.go()['error'], 'no_run')
        self.assertTrue(d.request('start', row_len_m=30.)['ok'])
        reply = d.go()
        self.assertFalse(reply['ok'])
        self.assertIn('inputs_not_fresh', reply['blockers'])
        self.assertEqual(d.go('bad')['error'], 'invalid_session_id')

    def test_hold_endpoints_refused_in_one_press_and_go_in_hold_to_run(self):
        d = Driver(one_press())
        self.assertEqual(d.loop.hold_beat_detail('press-0001'), (False, 'start_mode_one_press'))
        self.assertFalse(d.loop.hold_release('press-0001'))
        h = Driver(legacy_sensor_config().validate())
        self.assertEqual(h.loop.start_mode, 'hold_to_run')
        self.assertEqual(h.loop.operator_heartbeat('screen-one-0001'), (False, 'start_mode_hold_to_run'))
        self.assertEqual(h.go()['error'], 'start_mode_hold_to_run')
        self.assertEqual(h.loop.status()['go_blockers'], [])
        self.assertFalse(h.loop.status()['go_ready'])

    def test_motor_needs_arm_before_go_and_a_link_loss_sends_zero(self):
        d = Driver(motor_config(start_mode='one_press'), motor=True)
        self.assertTrue(d.request('start', row_len_m=30.)['ok'])
        d.run(1.6)
        self.assertEqual(d.loop.status()['run_state'], 'ready')
        self.assertIn('arm_first', d.go()['blockers'])
        self.assertTrue(d.request('arm')['ok'])
        self.assertEqual(d.loop.bridge.state, ARMED)
        self.assertEqual(d.loop.status()['run_state'], 'armed')
        self.assertTrue(d.go()['ok'])
        d.run(1.)
        self.assertNotEqual(d.loop._last_sent, (0, 0))
        d.visible = False
        d.run(.6)
        self.assertEqual(d.loop.runner.reason, 'operator_link_lost')
        self.assertEqual(d.loop._last_sent, (0, 0))                     # 0/0 sent at once, the lease kept
        self.assertEqual(d.loop.bridge.state, ARMED)
        d.loop.emergency_stop('page')
        d.run(.1)
        self.assertEqual(d.loop.bridge.state, LATCHED)
        self.assertEqual(d.loop.status()['run_state'], 'latched')


class HttpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.loop = M.ExecutorLoop(one_press())
        cls.stop = threading.Event()

        def run():
            while not cls.stop.is_set():
                cls.loop.step()
                time.sleep(.02)
        cls.thread = threading.Thread(target=run, daemon=True)
        cls.thread.start()
        cls.server = H.make_server(cls.loop, '127.0.0.1', 0, TOKEN)
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.stop.set()
        cls.thread.join(2.)

    def call(self, method, path, body=None):
        host = self.server.host_header
        conn = http.client.HTTPConnection('127.0.0.1', self.server.server_address[1], timeout=5)
        data = None if body is None else json.dumps(body).encode()
        headers = {'Host': host, 'X-Row-Token': TOKEN, 'Origin': 'http://' + host}
        if data is not None:
            headers.update({'Content-Type': 'application/json', 'Content-Length': str(len(data))})
        conn.request(method, path, body=data, headers=headers)
        resp = conn.getresponse()
        payload = json.loads(resp.read())
        conn.close()
        return resp.status, payload

    def test_routes(self):
        self.assertEqual(self.call('POST', '/api/heartbeat', {'session': 'http-one-0001'}), (200, {'ok': True}))
        self.assertEqual(self.call('POST', '/api/heartbeat', {'session': 'http-two-0002'})[0], 409)
        self.assertEqual(self.call('POST', '/api/heartbeat', {'session': 'http-one-0001', 'x': 1})[0], 400)
        code, reply = self.call('POST', '/api/go', {'session': 'http-one-0001'})
        self.assertEqual((code, reply['error']), (409, 'no_run'))
        code, reply = self.call('POST', '/api/go', {'session': 'http-two-0002'})
        self.assertEqual((code, reply['error']), (409, 'session_conflict'))
        self.assertEqual(self.call('POST', '/api/hold', {'press': 'press-0001'}), (409, {'ok': False,
                                                                                         'error': 'start_mode_one_press'}))
        self.assertEqual(self.call('POST', '/api/release', {'press': 'press-0001'})[0], 409)
        code, status = self.call('GET', '/api/status')
        self.assertEqual(code, 200)
        for key in ('start_mode', 'run_state', 'operator_link', 'stop_reason', 'speed', 'tuning', 'go_ready',
                    'go_blockers'):
            self.assertIn(key, status)
        self.assertEqual(set(status['operator_link']) >= {'fresh', 'age_s', 'timeout_s'}, True)
        self.assertEqual(set(status['speed']) >= {'target_mps', 'limit_mps', 'limit_reason'}, True)
        self.assertEqual(set(status['tuning']) >= {'preset', 'revision', 'editable'}, True)


if __name__ == '__main__':
    unittest.main()
