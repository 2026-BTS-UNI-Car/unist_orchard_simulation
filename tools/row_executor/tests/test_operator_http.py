"""Operator page server (operator_http) over a real loopback socket, with a live ExecutorLoop thread (SHADOW).

Checks the access rules (token, Host, Origin, body size, strict JSON), the hold heartbeat semantics through
HTTP, the request routing and that SHADOW never builds a bridge client. Day-1 review fixes: two devices (S3),
what STOP does (S9), STOP failures and the token page (S10), the log state in /api/status (S8).
The page's JavaScript behaviour itself is exercised in test_page_js.py (node).
"""
import http.client
import json
import threading
import time
import unittest

import rowexec_testlib  # noqa: F401  (path setup)
from row_executor import executor_main as M
from row_executor import operator_http as H
from row_executor.config import ExecutorConfig

TOKEN = 'test-token-0123456789abcdef'


class ServerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.loop = M.ExecutorLoop(ExecutorConfig())
        cls.stop = threading.Event()

        def run():
            while not cls.stop.is_set():
                cls.loop.step()
                time.sleep(.02)
        cls.loop_thread = threading.Thread(target=run, daemon=True)
        cls.loop_thread.start()
        cls.server = H.make_server(cls.loop, '127.0.0.1', 0, TOKEN)
        cls.host = cls.server.host_header
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.stop.set()
        cls.loop_thread.join(2.)

    def call(self, method, path, body=None, token=TOKEN, origin=True, host=None, raw=None, headers=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.server.server_address[1], timeout=5)
        h = {'Host': host or self.host}
        if token is not None:
            h['X-Row-Token'] = token
        if origin is True:
            h['Origin'] = 'http://' + self.host
        elif origin:
            h['Origin'] = origin
        data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
        if data is not None:
            h['Content-Type'] = 'application/json'
            h['Content-Length'] = str(len(data))
        h.update(headers or {})
        conn.putrequest(method, path, skip_host=True, skip_accept_encoding=True)
        for k, v in h.items():
            conn.putheader(k, v)
        conn.endheaders(data)
        resp = conn.getresponse()
        payload = resp.read()
        conn.close()
        try:
            return resp.status, json.loads(payload), resp
        except ValueError:
            return resp.status, payload.decode('utf-8', 'replace'), resp

    def test_page_needs_the_token_and_the_right_host(self):
        self.assertEqual(self.call('GET', '/')[0], 403)
        self.assertEqual(self.call('GET', '/?t=wrong-token-0123456789')[0], 403)
        status, page, resp = self.call('GET', '/?t=' + TOKEN)
        self.assertEqual(status, 200)
        self.assertIn(TOKEN, page)
        csp = resp.getheader('Content-Security-Policy')
        nonce = csp.split("'nonce-")[1].split("'")[0]
        self.assertIn('<script nonce="%s">' % nonce, page)
        self.assertEqual(resp.getheader('Cache-Control'), 'no-store')
        self.assertEqual(self.call('GET', '/?t=' + TOKEN, host='evil.example:8097')[0], 421)

    def test_token_page_explains_how_to_get_the_new_token(self):            # review S10
        status, page, resp = self.call('GET', '/?t=wrong-token-0123456789')
        self.assertEqual(status, 403)
        self.assertTrue(resp.getheader('Content-Type').startswith('text/html'))
        for needle in ('operator_token.txt', 'operator page:', 'run_executor.sh logs', '대시보드 STOP', '물리 E-stop'):
            self.assertIn(needle, page)
        self.assertNotIn(TOKEN, page)
        self.assertIn("default-src 'none'", resp.getheader('Content-Security-Policy'))

    def test_page_says_what_stop_does_and_shows_failures(self):              # review S9, S10
        page = self.call('GET', '/?t=' + TOKEN)[1]
        for needle in ('shadow run 종료 (차량 정지 아님)', '실행기는 차량을 제어하지 않는다', 'STOP (차량 정지)',
                       'STOP 실패', '토큰 무효', 'location.reload', 'AbortController', '연결 끊김'):
            self.assertIn(needle, page)

    def test_status_reports_log_state_and_stop_meaning(self):               # review S8, S9
        body = self.call('GET', '/api/status')[1]
        self.assertEqual((body['stop_button'], body['vehicle_control']), ('ends_shadow_run_only', 'none'))
        self.assertEqual(body['log']['state'], 'ok')
        self.assertIn('dropped', body['log'])
        self.assertEqual(body['loop_exceptions'], 0)

    def test_second_device_releases_both_presses(self):                     # review S3 through HTTP
        self.assertEqual(self.call('POST', '/api/hold', {'press': 'dev-a-press-1'})[0], 200)
        status, body, _ = self.call('POST', '/api/hold', {'press': 'dev-b-press-1'})
        self.assertEqual((status, body['error']), (409, 'press_conflict'))
        self.assertFalse(self.loop.hold.active(self.loop.clock()))
        status, body, _ = self.call('POST', '/api/hold', {'press': 'dev-a-press-1'})
        self.assertEqual((status, body['error']), (409, 'press_retired'))                    # no revival
        self.assertEqual(self.call('POST', '/api/release', {'press': 'dev-b-press-1'})[0], 200)
        self.assertEqual(self.call('POST', '/api/release', {'press': 'dev-a-late-9'})[0], 200)
        self.assertEqual(self.call('POST', '/api/hold', {'press': 'dev-a-late-9'})[0], 409)    # beat after its release
        self.assertEqual(self.call('POST', '/api/hold', {'press': 'dev-a-press-2'})[0], 200)   # a new press
        self.assertEqual(self.call('POST', '/api/release', {'press': 'dev-a-press-2'})[0], 200)
        self.assertFalse(self.loop.hold.active(self.loop.clock()))

    def test_status_and_post_access_rules(self):
        self.assertEqual(self.call('GET', '/api/status', token=None)[0], 403)
        status, body, _ = self.call('GET', '/api/status')
        self.assertEqual(status, 200)
        self.assertEqual(body['mode'], 'SHADOW')
        self.assertEqual(self.call('POST', '/api/hold', {'press': 'abcdefgh'}, origin=False)[0], 403)
        self.assertEqual(self.call('POST', '/api/hold', {'press': 'abcdefgh'}, origin='http://evil')[0], 403)
        self.assertEqual(self.call('POST', '/api/hold', {'press': 'abcdefgh'}, token='x' * 26)[0], 403)
        self.assertEqual(self.call('POST', '/api/hold', raw=b'x' * 5000)[0], 413)
        self.assertEqual(self.call('POST', '/api/hold', raw=b'{"press": NaN}')[0], 400)
        self.assertEqual(self.call('POST', '/api/hold', raw=b'[1]')[0], 400)
        self.assertEqual(self.call('POST', '/api/nothing', {})[0], 404)

    def test_hold_heartbeat_release_and_no_revival(self):
        self.assertEqual(self.call('POST', '/api/hold', {'press': 'press-http-1'})[0], 200)
        self.assertTrue(self.loop.hold.active(self.loop.clock()))
        self.assertEqual(self.call('POST', '/api/release', {'press': 'press-http-1'})[0], 200)
        self.assertFalse(self.loop.hold.active(self.loop.clock()))
        self.assertEqual(self.call('POST', '/api/hold', {'press': 'press-http-1'})[0], 409)
        self.assertEqual(self.call('POST', '/api/hold', {'press': 'press-http-2'})[0], 200)
        time.sleep(.6)                                         # heartbeats stop: expires for good
        self.assertFalse(self.loop.hold.active(self.loop.clock()))
        self.assertEqual(self.call('POST', '/api/hold', {'press': 'press-http-2'})[0], 409)

    def test_requests_are_routed_to_the_loop(self):
        status, body, _ = self.call('POST', '/api/start', {})
        self.assertEqual((status, body['error']), (409, 'row_len_m_required'))
        status, body, _ = self.call('POST', '/api/arm', {})
        self.assertEqual((status, body['error']), (409, 'motor_output_not_allowed'))
        status, body, _ = self.call('POST', '/api/params', {'duty_cap_pct': 90})
        self.assertEqual((status, body['error']), (400, 'unknown_parameter'))
        status, body, _ = self.call('POST', '/api/start', {'row_len_m': 25.})
        self.assertEqual(status, 200, body)
        status, body, _ = self.call('POST', '/api/params', {'base_duty_pct': 50})
        self.assertEqual((status, body['error']), (409, 'end_the_run_first'))
        status, body, _ = self.call('POST', '/api/stop', {})
        self.assertEqual(status, 200)
        self.assertTrue(body['shadow'])
        self.assertIsNone(body['stop_sent'])
        time.sleep(.1)
        status, body, _ = self.call('GET', '/api/status')
        self.assertEqual(body['end_reason'], 'operator_stop')
        self.assertIsNone(self.loop.bridge)
        status, body, _ = self.call('POST', '/api/params', {'base_duty_pct': 50, 'reduce_duty_pct': 50,
                                                            'max_differential_pct': 15})
        self.assertEqual(status, 200, body)
        self.assertEqual(self.call('GET', '/api/status')[1]['params']['base_duty_pct'], 50)


class StopFailureTest(unittest.TestCase):
    """MOTOR STOP that the bridge did not confirm -> HTTP 502 with the body (the page shows it in red)."""

    def test_unconfirmed_stop_is_502(self):
        class FakeLoop:
            def __init__(self):
                self.stops = []

            def emergency_stop(self, source):
                self.stops.append(source)
                return dict(ok=True, stop_sent=False, error='timeout', shadow=False,
                            note='STOP NOT CONFIRMED: use the dashboard STOP / physical E-stop')

            def status(self):
                return {}
        loop = FakeLoop()
        server = H.make_server(loop, '127.0.0.1', 0, TOKEN)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        conn = http.client.HTTPConnection('127.0.0.1', server.server_address[1], timeout=5)
        conn.request('POST', '/api/stop', body=b'{}', headers={
            'Host': server.host_header, 'Origin': server.origin, 'X-Row-Token': TOKEN,
            'Content-Type': 'application/json', 'Content-Length': '2'})
        resp = conn.getresponse()
        body = json.loads(resp.read())
        conn.close()
        self.assertEqual(resp.status, 502)
        self.assertIs(body['stop_sent'], False)
        self.assertIn('NOT CONFIRMED', body['note'])
        self.assertEqual(loop.stops, ['operator_page'])


if __name__ == '__main__':
    unittest.main()
