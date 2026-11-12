"""Routes of the steering control rules against the pinned dashboard and the real host bridge (design 4.4, 4.2, 8.3 P6b;
A14 old screen). The rules are dashboard/static/steer-control-rules.js, kept from the phase-4 steering page draft
without the page (DECISIONS R22): the page, its display feed and its browser serve mode are not kept.

The pinned dashboard_server/app.py (create_server, DASHBOARD_MODE=simulation, SimulationProvider) runs in this
process. The bridge is the real host code (scripts/steer_bridge_core.py + steer_control_core.py + esp_motor_v3.py) on a
memory transport with the scripted fake V3 MCU of scripts/test_steer_support.py (no firmware, no Gazebo):
  SocketRoutesTest (Linux): the bridge wrapper scripts/steer_rover_bridge.py (BridgeRunner, --transport memory) serves
    its control socket and app.py reaches it through dashboard_server/rover_api.py, as in the G-SIM stack.
  InprocRoutesTest (any OS): app.py's rover_api.request is replaced by an in-process call of the same bridge core (the
    request and the reply go through JSON as on the socket). On Windows the modules fcntl and termios are stubbed only
    while app.py is imported (telemetry.py imports them at the top; the rover routes never use them).
Checks: the old differential screen (its request bodies as in the pinned app.js) cannot arm or move the car; the
request bodies of the steering rules work; the status route carries every field the rules read and the fields a
steering screen shows; the rules' fault catalogue covers the host code's fault bits and latches.
Temporary folders only, nothing in the work tree. Placeholder-based SIM results only, never field validation.
"""
from __future__ import annotations

import http.cookiejar
import importlib
import json
import os
from pathlib import Path
import re
import shutil
import socket
import sys
import tempfile
import threading
import time
import types
import unittest
import urllib.error
import urllib.request

sys.dont_write_bytecode = True
REPO = Path(__file__).resolve().parents[2]
DASHBOARD = REPO / 'dashboard'
STATIC = DASHBOARD / 'static'
UNIX = hasattr(socket, 'AF_UNIX') and sys.platform.startswith('linux')
ORIGIN_HEADER = 'Origin'
_STATE = {}

# Fields of the bridge snapshot (GET /api/rover/status) that a steering screen shows (the dropped page draft read all
# of them; the status route keeps carrying them for the new screen). RULE_FIELDS: what steer-control-rules.js reads.
PAGE_STATUS_FIELDS = ('ok', 'armed', 'state', 'reason', 'last_error', 'updated_at', 'firmware', 'boot_id',
                      'reset_reason', 'hw_profile', 'cal_id', 'transport', 'requested', 'outputs', 'steer_limit_mrad',
                      'steering', 'steering_host')
PAGE_STEERING_FIELDS = ('measured_mrad', 'measured_permille', 'raw', 'target_permille', 'cmd_permille', 'pwm', 'mcu_ms',
                        'seq', 'age_ms', 'ttl_remaining_ms', 'drive_en', 'steer_en', 'hold_remaining_ms', 'stop_mode',
                        'faults', 'servo_state', 'saturated', 'calibrated')
PAGE_HOST_FIELDS = ('arm_state', 'stop_state', 'mcu_state', 'stale', 'latches', 'expected_soft_permille', 'link')
PAGE_LINK_FIELDS = ('state', 'identity_error', 'build')
PAGE_OUTPUT_PINS = ('D8', 'D5', 'D3', 'D0', 'D1', 'D2')
RULE_FIELDS = ('armed', 'reason', 'updated_at', 'outputs', 'steer_limit_mrad', 'steering', 'steering_host',
               'measured_mrad', 'measured_permille', 'age_ms', 'mcu_ms', 'seq', 'stale')
RULES_JS = 'steer-control-rules.js'


def dashboard_modules():
    """The dashboard (app, rover_api) and host (esp_motor_v3, steer_bridge_core, steer_rover_bridge, test_steer_support)
    modules of this work tree, or SkipTest with the reason.

    The dashboard imports more of its 'scripts' package lazily (e.g. sim_mapping -> scripts.sim_clock when the SIM
    provider starts), so this tree's dashboard stays first on sys.path for the rest of the process: run this test file
    in its own process (one pytest process per file). A 'scripts' package of another dashboard copy already loaded in
    this process is a skip reason, never mixed."""
    if 'mods' in _STATE:
        if isinstance(_STATE['mods'], str):
            raise unittest.SkipTest(_STATE['mods'])
        return _STATE['mods']
    if not (DASHBOARD / 'dashboard_server' / 'app.py').is_file():
        _STATE['mods'] = 'dashboard_server/app.py not found next to sim/ubts (%s)' % DASHBOARD
        raise unittest.SkipTest(_STATE['mods'])
    for name in ('scripts', 'dashboard_server'):
        loaded = sys.modules.get(name)
        if loaded is None:
            continue
        where = getattr(loaded, '__file__', None) or ''
        if not where or DASHBOARD.resolve() not in Path(where).resolve().parents:
            _STATE['mods'] = ('another %s package (%s) is already loaded in this process: run %s in its own process'
                              % (name, where or 'namespace', Path(__file__).name))
            raise unittest.SkipTest(_STATE['mods'])
    if str(DASHBOARD) not in sys.path:
        sys.path.insert(0, str(DASHBOARD))
    stubs = []
    try:
        for name in ('fcntl', 'termios'):
            try:
                importlib.import_module(name)
            except ImportError:
                sys.modules[name] = types.ModuleType(name)
                stubs.append(name)
        _STATE['mods'] = dict(
            app=importlib.import_module('dashboard_server.app'),
            rover_api=importlib.import_module('dashboard_server.rover_api'),
            v3=importlib.import_module('scripts.esp_motor_v3'),
            core=importlib.import_module('scripts.steer_bridge_core'),
            wrapper=importlib.import_module('scripts.steer_rover_bridge'),
            support=importlib.import_module('scripts.test_steer_support'),
            stubbed=tuple(stubs))
    except Exception as exc:   # an import failure is a skip reason of this test, not its error
        _STATE['mods'] = 'dashboard or host code does not import: %s: %s' % (type(exc).__name__, exc)
    finally:
        for name in stubs:      # telemetry.py keeps its reference; other modules see no fake fcntl / termios
            sys.modules.pop(name, None)
    if isinstance(_STATE['mods'], str):
        raise unittest.SkipTest(_STATE['mods'])
    return _STATE['mods']


def js_catalogue():
    """(MCU fault names, host latch names) of the catalogues in steer-control-rules.js."""
    text = (STATIC / RULES_JS).read_text(encoding='utf-8')

    def block(name):
        match = re.search(r'var %s = \[(.*?)\n  \];' % name, text, re.S)
        if not match:
            raise AssertionError('catalogue %s not found in %s' % (name, RULES_JS))
        return re.findall(r"\['([A-Za-z_]+)'", match.group(1))
    return block('MCU_FAULTS'), block('HOST_LATCHES')


class UiStack:
    """The pinned dashboard + the real bridge (socket or in-process) + the fake V3 MCU, all in this process."""

    LOOP_S = 0.005
    CORE_PERIOD_S = 0.02

    def __init__(self, mode, port=0):
        self.mods = mods = dashboard_modules()
        if mode == 'socket' and not UNIX:
            raise unittest.SkipTest('the bridge control socket needs a Unix socket server (Linux)')
        self.mode = mode
        self.tmp = Path(tempfile.mkdtemp(prefix='ubts_ui_'))
        self.lock = threading.RLock()        # wait() predicates may call rx() / probe() again
        self.stop_event = threading.Event()
        self._env = {}
        self._patched = None
        self.runner = None
        env = dict(DASHBOARD_MODE='simulation', DASHBOARD_ACCESS_LOG='0', TELEMETRY_FILE=str(self.tmp / 'telemetry.json'),
                   SIM_MAPPING_STATE_DIR=str(self.tmp / 'mapping'), SIM_MAP_HISTORY_DIR=str(self.tmp / 'map-history'),
                   RECORDING_ARCHIVE_DIR=str(self.tmp / 'recordings'),
                   ROVER_BRIDGE_STATUS_FILE=str(self.tmp / 'rover_bridge_status.json'),
                   DATA_RECORDER_STATUS_FILE=str(self.tmp / 'data_recorder_status.json'),
                   CAMERA_SETTINGS_FILE=str(self.tmp / 'camera_settings.json'),
                   SIM_MAPPING_SAMPLE_FILE=str(self.tmp / 'mapping_sample.json'),
                   # the provider looks for its own container through the Docker API at start: never reach Docker
                   DOCKER_SOCKET=str(self.tmp / 'no-docker.sock'))
        hp_path = self.tmp / 'host_profile.json'
        hp_path.write_text(json.dumps(mods['support'].host_profile_data()), encoding='utf-8')
        self.clock = time.monotonic
        transport = mods['v3'].MemoryTransport(clock=self.clock)
        self.mcu = mods['support'].FakeMcu(transport)
        if mode == 'socket':
            sock = self.tmp / 'c.sock'
            env['ROVER_CONTROL_SOCKET'] = str(sock)
            args = mods['wrapper'].build_parser().parse_args(
                ['--transport', 'memory', '--no-ros', '--host-profile', str(hp_path), '--socket', str(sock),
                 '--status-file', str(self.tmp / 'bridge_status.json')])
            config = mods['wrapper'].resolve_config(args, have_transport=True)
            self.runner = mods['wrapper'].BridgeRunner(config, transport=transport, clock=self.clock, ros=None)
            self.core = self.runner.core
        else:
            hp = mods['core'].HostProfile.load(hp_path)
            link = mods['v3'].EspSteerV3(transport, self.clock, simulated=True, expect_hw=hp.hw_profile,
                                         banner_timeout_s=hp.banner_timeout_s)
            self.core = mods['core'].SteerBridgeCore(link, hp, clock=self.clock)
            self._patched = mods['rover_api'].request
            mods['rover_api'].request = self._inproc_request
        for key, value in env.items():
            self._env[key] = os.environ.get(key)
            os.environ[key] = value
        if self.runner is not None:
            self.runner.start()
        self.loop = threading.Thread(target=self._loop, name='ubts-ui-bridge', daemon=True)
        self.loop.start()
        app = mods['app']
        self.server = app.create_server('127.0.0.1', port)
        self.port = self.server.server_address[1]
        self.base = 'http://127.0.0.1:%d' % self.port
        self.http = threading.Thread(target=self.server.serve_forever, name='ubts-ui-http', daemon=True)
        self.http.start()
        if not self.wait(lambda: self.core.steering_host()['stop_state'] == 'confirmed', 3.0):
            self.close()
            raise AssertionError('bridge did not become ready: %r' % (self.core.steering_host(),))

    def _inproc_request(self, payload):
        """rover_api.request for the in-process mode: the same JSON round trip as the control socket."""
        try:
            request = json.loads(json.dumps(payload, allow_nan=False))
            if not isinstance(request, dict):
                raise ValueError('object_required')
            reply = self.core.request(request)
            return json.loads(json.dumps(reply, allow_nan=False))
        except (OSError, ValueError):
            return {'ok': False, 'connected': False, 'state': 'unavailable', 'error': 'motor_bridge_unavailable'}

    def _loop(self):
        next_core = self.clock()
        while not self.stop_event.wait(self.LOOP_S):
            now = self.clock()
            with self.lock:
                self.mcu.tick(now)
                if now >= next_core:
                    if self.runner is not None:
                        self.runner.loop_once()
                    else:
                        self.core.step()
                    next_core += self.CORE_PERIOD_S
                    if next_core < now:
                        next_core = now + self.CORE_PERIOD_S

    def probe(self):
        with self.lock:
            lines = [dict(t=t, line=line.decode('ascii', 'replace')) for t, line in self.mcu.rx_lines]
            return dict(rx=lines, latches=list(self.core.latches), state=self.core.snapshot.get('state'),
                        mcu=dict(st=self.mcu.st, L=self.mcu.L, R=self.mcu.R, tgt=self.mcu.tgt, pos=self.mcu.pos))

    def wait(self, predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self.lock:
                if predicate():
                    return True
            time.sleep(0.01)
        with self.lock:
            return bool(predicate())

    def rx(self):
        with self.lock:
            return [line for _, line in self.mcu.rx_lines]

    def close(self):
        self.stop_event.set()
        try:
            self.server.shutdown()
            self.server.server_close()
        except Exception:
            pass
        self.loop.join(2.0)
        if self.runner is not None:
            try:
                self.runner.shutdown()
            except Exception:
                pass
        if self._patched is not None:
            self.mods['rover_api'].request = self._patched
            self._patched = None
        for key, value in self._env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        shutil.rmtree(self.tmp, ignore_errors=True)


class Client:
    """A browser-like client: one cookie jar (the dashboard session = the bridge owner), same-origin POSTs."""

    def __init__(self, base):
        self.base = base
        self.jar = http.cookiejar.CookieJar()
        self.opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(self.jar))

    def get(self, path):
        try:
            with self.opener.open(self.base + path, timeout=5) as res:
                return res.status, res.read()
        except urllib.error.HTTPError as exc:
            return exc.code, exc.read()

    def get_json(self, path):
        status, body = self.get(path)
        return status, json.loads(body.decode('utf-8'))

    def post(self, path, body, origin=None):
        data = json.dumps(body).encode('utf-8')
        req = urllib.request.Request(self.base + path, data=data, method='POST',
                                     headers={'Content-Type': 'application/json',
                                              ORIGIN_HEADER: self.base if origin is None else origin})
        try:
            with self.opener.open(req, timeout=5) as res:
                return res.status, json.loads(res.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            return exc.code, None


def c_lines(lines):
    """[(q, ttl, L, R, S)] of the C lines among host lines."""
    out = []
    for line in lines:
        if line.startswith(b'C '):
            q, ttl, left, right, steer = line.split(b'*')[0].split(b' ')[1:6]
            out.append((int(q), int(ttl), int(left), int(right), int(steer)))
    return out


class RoutesBase:
    """Shared test cases; subclasses set MODE."""
    MODE = None

    def setUp(self):
        self.stack = UiStack(self.MODE)
        self.addCleanup(self.stack.close)
        self.page = Client(self.stack.base)
        status, body = self.page.get('/static/' + RULES_JS)             # a static load gives the session cookie
        self.assertEqual(status, 200)

    def arm(self, client=None):
        client = client or self.page
        status, reply = client.post('/api/rover/arm', {'vehicle_profile': 'front_steer'})
        self.assertEqual(status, 200)
        self.assertTrue(reply.get('ok'), reply)
        self.assertTrue(self.stack.wait(lambda: self.stack.core.steering_host()['arm_state'] == 'confirmed', 2.0))
        return reply

    # -- A14: the old differential screen ------------------------------------------------------------------------
    def test_old_screen_arm_is_refused(self):
        old = Client(self.stack.base)
        status, body = old.get('/rover-control')                       # the pinned differential screen
        self.assertEqual(status, 200)
        self.assertIn(b'roverArmButton', body)
        old.get('/api/rover/status')          # the old screen's JSON polls give the session cookie (the HTML does not)
        before = len(self.stack.rx())
        status, reply = old.post('/api/rover/arm', {})                 # app.js: postJson("/api/rover/arm")
        self.assertEqual(status, 200)
        self.assertEqual(reply, {'ok': False, 'error': 'steer_profile_required'})
        time.sleep(0.2)
        sent = self.stack.rx()[before:]
        self.assertFalse([line for line in sent if line.startswith(b'ARM')], 'no ARM reached the MCU')
        status, snap = old.get_json('/api/rover/status')
        self.assertFalse(snap['armed'])
        self.assertEqual(self.stack.mcu.st, 1, 'MCU stays DISARMED')

    def test_old_screen_left_right_drive_is_rejected_and_stops(self):
        old = Client(self.stack.base)
        old.get('/rover-control')
        old.get('/api/rover/status')
        status, reply = old.post('/api/rover/command', {'lease': None, 'ticket': None, 'seq': 1, 'left': 30,
                                                        'right': 30, 'deadman': True})
        self.assertEqual(reply, {'ok': False, 'error': 'differential_command_rejected'})
        # the rejection stops: arm waits for the STOP confirmation (an arm within ~60 ms gets stop_unconfirmed)
        self.assertTrue(self.stack.wait(lambda: self.stack.core.steering_host()['stop_state'] == 'confirmed', 1.0))
        # with the steering page's valid lease (same session: the old screen in another tab of the same browser)
        grant = self.arm()
        before = len(self.stack.rx())
        status, reply = self.page.post('/api/rover/command', {'lease': grant['lease'], 'ticket': grant['ticket'],
                                                              'seq': grant['seq'] + 1, 'left': 30, 'right': 30,
                                                              'deadman': True})
        self.assertEqual(reply, {'ok': False, 'error': 'differential_command_rejected'})
        self.assertTrue(self.stack.wait(lambda: self.stack.mcu.st == 1, 1.0), 'the bridge stopped the MCU')
        time.sleep(0.3)
        status, snap = self.page.get_json('/api/rover/status')
        self.assertFalse(snap['armed'], 'lease revoked')
        self.assertEqual(snap['reason'], 'differential_command_rejected')
        sent = self.stack.rx()[before:]
        self.assertTrue(any(line.startswith(b'STOP') for line in sent))
        self.assertFalse([c for c in c_lines(sent) if c[2] or c[3]], 'no drive duty was ever sent')
        self.assertEqual((self.stack.mcu.L, self.stack.mcu.R), (0, 0))

    def test_old_screen_sources_send_no_vehicle_profile_and_use_left_right(self):
        """The pinned old screen's request shapes, read from app.js (not changed): arm without a body, drive with
        left/right. With the two tests above this is A14 (old screen): it cannot arm or move the steering car."""
        app_js = (STATIC / 'app.js').read_text(encoding='utf-8')
        self.assertIn('postJson("/api/rover/arm")', app_js)
        self.assertNotIn('vehicle_profile', app_js)
        self.assertRegex(app_js, r'left: motion\.left, right: motion\.right')
        # the notice '이 차량은 조향 화면만 쓴다' belonged to the dropped page; the new dashboard screen carries it

    # -- the steering page's request bodies ----------------------------------------------------------------------
    def test_steer_page_request_flow(self):
        grant = self.arm()
        self.assertEqual(sorted(grant), ['lease', 'ok', 'seq', 'status', 'ticket', 'ticket_ttl_s'])
        before = len(self.stack.rx())
        body = {'lease': grant['lease'], 'ticket': grant['ticket'], 'seq': grant['seq'] + 1, 'deadman': True,
                'throttle': 40, 'steer_mrad': 87}
        status, reply = self.page.post('/api/rover/command', body)
        self.assertTrue(reply.get('ok'), reply)
        self.assertEqual(reply['seq'], 1)
        self.assertTrue(self.stack.wait(lambda: c_lines(self.stack.rx()[before:]), 1.0))
        q, ttl, left, right, steer = c_lines(self.stack.rx()[before:])[0]
        self.assertEqual((left, right), (102, 102), 'throttle 40 % -> duty 102 both sides')
        self.assertEqual(steer, 199, '87 mrad -> 199 permille (marks 438.03 mrad)')
        self.assertTrue(1 <= ttl <= 300)
        self.assertTrue(self.stack.wait(lambda: self.stack.mcu.L == 102, 1.0))
        time.sleep(0.15)
        status, snap = self.page.get_json('/api/rover/status')
        self.assertEqual(snap['requested']['throttle'], 40)
        self.assertEqual(snap['requested']['steer_mrad'], 87)
        self.assertEqual(snap['outputs']['D8'] - snap['outputs']['D5'], 102, 'MCU drive duty in the status')
        status, reply = self.page.post('/api/rover/keepalive', {'lease': grant['lease'], 'ticket': reply['ticket'],
                                                                 'seq': reply['seq'] + 1})
        self.assertTrue(reply.get('ok'), reply)
        status, reply = self.page.post('/api/rover/release', {'lease': grant['lease'], 'ticket': reply['ticket'],
                                                               'seq': reply['seq'] + 1})
        self.assertTrue(reply.get('ok'), reply)
        status, reply = self.page.post('/api/rover/stop', {'reason': 'operator_stop'})
        self.assertEqual(reply, {'ok': True, 'state': 'stop_requested'})
        self.assertTrue(self.stack.wait(lambda: self.stack.mcu.st == 1, 1.0))
        after_stop = len(self.stack.rx())
        time.sleep(0.5)
        self.assertFalse(c_lines(self.stack.rx()[after_stop:]), 'no C line after STOP')
        status, snap = self.page.get_json('/api/rover/status')
        self.assertFalse(snap['armed'])

    def test_steering_only_inputs_need_the_lease_and_the_deadman(self):
        status, reply = self.page.post('/api/rover/command', {'throttle': 0, 'steer_mrad': 50, 'deadman': True,
                                                              'seq': 1})
        self.assertEqual(reply, {'ok': False, 'error': 'control_not_owned'})
        grant = self.arm()
        status, reply = self.page.post('/api/rover/command', {'lease': grant['lease'], 'ticket': grant['ticket'],
                                                              'seq': 1, 'deadman': False, 'throttle': 0,
                                                              'steer_mrad': 50})
        self.assertEqual(reply, {'ok': False, 'error': 'invalid_drive'})
        self.assertTrue(self.stack.wait(lambda: self.stack.mcu.st == 1, 1.0))

    def test_stop_needs_no_lease_and_session_and_origin_are_checked(self):
        status, reply = self.page.post('/api/rover/stop', {'reason': 'escape'})
        self.assertEqual(reply, {'ok': True, 'state': 'stop_requested'})
        stranger = Client(self.stack.base)                             # no page load: no session cookie
        status, _ = stranger.post('/api/rover/stop', {'reason': 'x'})
        self.assertEqual(status, 403)
        status, _ = self.page.post('/api/rover/arm', {'vehicle_profile': 'front_steer'}, origin='http://evil.example')
        self.assertEqual(status, 403)

    def test_status_route_has_every_field_the_page_reads(self):
        self.arm()
        time.sleep(0.15)
        status, snap = self.page.get_json('/api/rover/status')
        self.assertEqual(status, 200)
        for key in PAGE_STATUS_FIELDS:
            self.assertIn(key, snap)
        for key in PAGE_STEERING_FIELDS:
            self.assertIn(key, snap['steering'])
        for key in PAGE_HOST_FIELDS:
            self.assertIn(key, snap['steering_host'])
        for key in PAGE_LINK_FIELDS:
            self.assertIn(key, snap['steering_host']['link'])
        for pin in PAGE_OUTPUT_PINS:
            self.assertIn(pin, snap['outputs'])
        self.assertTrue(snap['armed'])
        self.assertEqual(snap['transport'], 'simulation')
        self.assertEqual(snap['mode'], 'simulation')                   # the SIM dashboard provider sets these two
        self.assertIs(snap['simulated'], True)
        self.assertEqual(snap['steer_limit_mrad'], {'left': 409, 'right': 409})
        self.assertIsInstance(snap['steering']['age_ms'], int)
        self.assertLessEqual(snap['steering']['age_ms'], 150)
        self.assertEqual(snap['steering_host']['link']['state'], 'ready')
        js = (STATIC / RULES_JS).read_text(encoding='utf-8')
        for key in RULE_FIELDS:
            self.assertIn(key, js, 'the rules read ' + key)
        for key in RULE_FIELDS[7:]:
            self.assertIn(key, PAGE_STEERING_FIELDS + PAGE_HOST_FIELDS, 'the route field list covers what the rules read')

    def test_rules_script_is_served(self):
        status, body = self.page.get('/static/' + RULES_JS)
        self.assertEqual(status, 200)
        self.assertIn(b'root.UbtsSteerRules = api;', body)


class InprocRoutesTest(RoutesBase, unittest.TestCase):
    MODE = 'inproc'


@unittest.skipUnless(UNIX, 'the bridge control socket (ThreadingUnixStreamServer, fcntl) needs Linux')
class SocketRoutesTest(RoutesBase, unittest.TestCase):
    MODE = 'socket'


class CatalogueTest(unittest.TestCase):
    def test_fault_catalogue_matches_the_host_code(self):
        mods = dashboard_modules()
        mcu, host = js_catalogue()
        self.assertEqual(mcu, list(mods['v3'].FAULT_BIT_NAMES), 'every MCU fault bit, in spec order')
        missing = sorted(set(mods['core'].LATCH_NAMES) - set(host))
        self.assertEqual(missing, [], 'host latches without a label on the page')
        self.assertEqual(len(host), len(set(host)))

    def test_rules_files_are_utf8_lf_without_bom(self):
        for name in (RULES_JS, 'test_steer_control_rules.mjs'):
            raw = (STATIC / name).read_bytes()
            self.assertFalse(raw.startswith(b'\xef\xbb\xbf'), name)
            self.assertNotIn(b'\r\n', raw, name)
            raw.decode('utf-8')


if __name__ == '__main__':
    unittest.main()
