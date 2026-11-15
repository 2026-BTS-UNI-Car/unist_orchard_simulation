"""The copied dashboard of the simulation stack against the real host bridge and the real executor operator page
(design 4.4, 4.2, 8.3 P6b; A14 old screen; D-13 / UI-15, docs/steering/SIM_DASHBOARD_KO.md).

dashboard/ (config, dashboard_server, static) is the dashboard repository at the commit of
dashboard/DASHBOARD_SOURCE.json; the simulation additions come from dashboard_sim/ through the extension points. This
test runs that dashboard's create_server in this process with the settings of compose.ubts-steer.yaml
(DASHBOARD_MODE=simulation, DASHBOARD_VEHICLE=front_steer, provider ubts_sim_dashboard.provider:SimulationProvider,
extension ubts_sim_dashboard.extension, extra static dashboard_sim/static, executor relay UBTS_EXECUTOR_URL):
  the bridge is the real host code (scripts/steer_bridge_core.py + steer_control_core.py + esp_motor_v3.py) on a
    memory transport with the scripted fake V3 MCU of scripts/test_steer_support.py (no firmware, no Gazebo):
    SocketRoutesTest (Linux): the bridge wrapper scripts/steer_rover_bridge.py (BridgeRunner, --transport memory) serves
      its control socket and the dashboard reaches it through dashboard_server/rover_api.py, as in the G-SIM stack.
    InprocRoutesTest (any OS): rover_api.request is replaced by an in-process call of the same bridge core (the request
      and the reply go through JSON as on the socket). On Windows the modules fcntl and termios are stubbed only while
      the dashboard is imported (telemetry.py imports them at the top; these routes never use them).
  the executor is the real operator page tools/row_executor/operator_http.py (token, Host, Origin, 4 KiB JSON rules)
    over a scripted loop: the dashboard's relay /api/executor/* must pass those rules (ExecutorRelayTest).
Checks: the old differential requests cannot arm or move the car; the steering requests of the page work; the status
route carries every field the steering rules read; the rules' fault catalogue covers the host code's fault bits and
latches; the simulation extension (provider, /api/sim/status without the GPS origin, page script) is plugged in; the
executor relay reaches the operator page and keeps the token and the GNSS fields away from the page.
Temporary folders for every file the test writes (the dashboard's own runtime/ folder is ignored by git). Placeholder-
based SIM results only, never field validation.
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
DASHBOARD_SIM = REPO / 'dashboard_sim'
TOOLS = REPO / 'tools'
STATIC = DASHBOARD / 'static'
UNIX = hasattr(socket, 'AF_UNIX') and sys.platform.startswith('linux')
ORIGIN_HEADER = 'Origin'
_STATE = {}

# Fields of the bridge snapshot (GET /api/rover/status) that a steering screen shows. RULE_FIELDS: what
# steer-control-rules.js reads.
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
RULES_JS = 'steer/steer-control-rules.js'
OLD_SCREEN_JS = 'app/10-rover-control.js'
# a virtual GPS origin in the stack status file: it must never reach the page (common rule 6)
SECRET_LAT, SECRET_LON = 12.3456789, 45.6789012   # made-up values; the test checks they never leave the server
TOKEN = 'sim-dashboard-test-token-' + 'q' * 16


def dashboard_modules():
    """The dashboard (app, rover_api, extensions), the simulation extension, the host modules (esp_motor_v3,
    steer_bridge_core, steer_rover_bridge, test_steer_support) and the executor operator page of this work tree, or
    SkipTest with the reason. This tree's dashboard stays first on sys.path for the rest of the process: run this test
    file in its own process (one pytest process per file). A 'scripts' or 'dashboard_server' package of another copy
    already loaded in this process is a skip reason, never mixed."""
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
    for folder in (TOOLS, DASHBOARD_SIM, DASHBOARD):        # DASHBOARD ends up first
        if str(folder) not in sys.path:
            sys.path.insert(0, str(folder))
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
            extensions=importlib.import_module('dashboard_server.extensions'),
            provider=importlib.import_module('ubts_sim_dashboard.provider'),
            extension=importlib.import_module('ubts_sim_dashboard.extension'),
            operator_http=importlib.import_module('row_executor.operator_http'),
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


def operator_page_has_one_press(mods):
    """Whether this tree's executor operator page serves the one-press routes (contract v0, branch of S1)."""
    return "'/api/heartbeat'" in Path(mods['operator_http'].__file__).read_text(encoding='utf-8')


class ScriptedLoop:
    """The executor loop behind the real operator page: the state machine of contract v0 (start, arm, go, heartbeat
    timeout 0.5 s -> paused OPERATOR_LINK_LOST, stop -> latched, ack, end), no vehicle. Records every request."""

    TIMEOUT_S = 0.5

    def __init__(self):
        self.lock = threading.Lock()
        self.ops = []
        self.run_state, self.session, self.beat = 'idle', None, None
        self.stop_reason = None

    def _link(self):
        age = None if self.beat is None else time.monotonic() - self.beat
        return age, age is not None and age <= self.TIMEOUT_S

    def _tick(self):
        age, fresh = self._link()
        if self.run_state == 'running' and not fresh:
            self.run_state, self.stop_reason = 'paused', 'OPERATOR_LINK_LOST'

    def status(self):
        with self.lock:
            self._tick()
            age, fresh = self._link()
            blockers = [] if fresh else ['operator_link_not_fresh']
            return {'ok': True, 'mode': 'MOTOR', 'state': self.run_state, 'run_state': self.run_state,
                    'start_mode': 'one_press', 'stop_reason': self.stop_reason,
                    'go_ready': self.run_state in ('armed', 'paused') and fresh, 'go_blockers': blockers,
                    'operator_link': {'fresh': fresh, 'age_s': age, 'timeout_s': self.TIMEOUT_S},
                    'gnss': {'state': 'no_fix', 'fix': 0, 'lat': SECRET_LAT, 'lon': SECRET_LON},
                    'token_dir': '/run/ubts-steer/exec/log'}

    def request(self, op, **kw):
        with self.lock:
            self._tick()
            self.ops.append((op, kw))
            if op == 'start':
                self.run_state, self.stop_reason, self.session = 'ready', None, None
            elif op == 'arm' and self.run_state == 'ready':
                self.run_state = 'armed'
            elif op == 'go':
                if self.session not in (None, kw.get('session')):
                    return {'ok': False, 'error': 'session_conflict'}
                if self.run_state not in ('armed', 'paused') or not self._link()[1]:
                    return {'ok': False, 'error': 'operator_link_not_fresh', 'blockers': ['operator_link_not_fresh']}
                self.run_state, self.session, self.stop_reason = 'running', kw.get('session'), None
            elif op == 'ack' and self.run_state == 'latched':
                self.run_state = 'ready'
            elif op == 'end':
                self.run_state = 'ended'
            else:
                return {'ok': False, 'error': 'refused'}
            return {'ok': True, 'run_state': self.run_state}

    def operator_heartbeat(self, session):
        with self.lock:
            self.ops.append(('heartbeat', {'session': session}))
            if self.session not in (None, session):
                return False, 'session_conflict'
            self.beat = time.monotonic()
            return True, None

    def emergency_stop(self, reason):
        with self.lock:
            self.ops.append(('stop', {'reason': reason}))
            self.run_state, self.stop_reason = 'latched', 'OPERATOR_OR_BACKSTOP'
            return {'ok': True, 'stop_sent': True}

    def hold_beat_detail(self, press):
        return False, 'start_mode_one_press'

    def hold_release(self, press):
        return False


class UiStack:
    """The copied dashboard + the real bridge (socket or in-process) + the fake V3 MCU + the real operator page over
    a scripted loop, all in this process."""

    LOOP_S = 0.005
    CORE_PERIOD_S = 0.02

    def __init__(self, mode, port=0, executor=True):
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
        self.executor = None
        self.server = None
        self.write_sim_status()
        env = dict(DASHBOARD_MODE='simulation', DASHBOARD_VEHICLE='front_steer',
                   DASHBOARD_PROVIDER='ubts_sim_dashboard.provider:SimulationProvider',
                   DASHBOARD_EXTENSION='ubts_sim_dashboard.extension',
                   DASHBOARD_EXTRA_STATIC=str(DASHBOARD_SIM / 'static'),
                   DASHBOARD_ACCESS_LOG='0', DASHBOARD_LIVE_MAP_WORKER='0',
                   TELEMETRY_FILE=str(self.tmp / 'telemetry.json'),
                   SIM_STATUS_FILE=str(self.tmp / 'simulation_status.json'),
                   RECORDING_ARCHIVE_DIR=str(self.tmp / 'recordings'),
                   ROVER_BRIDGE_STATUS_FILE=str(self.tmp / 'rover_bridge_status.json'),
                   DATA_RECORDER_STATUS_FILE=str(self.tmp / 'data_recorder_status.json'),
                   CAMERA_SETTINGS_FILE=str(self.tmp / 'camera_settings.json'),
                   CAMERA_DRIVER_STATUS_FILE=str(self.tmp / 'camera_driver_status.json'),
                   RECORDER_API_URL='http://127.0.0.1:8790',
                   UBTS_EXECUTOR_TUNING_LOG=str(self.tmp / 'executor_tuning_log.jsonl'),
                   # the provider looks for its own container through the Docker API at start: never reach Docker
                   DOCKER_SOCKET=str(self.tmp / 'no-docker.sock'))
        if executor:
            self.loop_model = ScriptedLoop()
            self.token_file = self.tmp / 'operator_token.txt'
            self.token_file.write_text(TOKEN + '\n', encoding='utf-8')
            self.executor = mods['operator_http'].make_server(self.loop_model, '127.0.0.1', 0, TOKEN)
            threading.Thread(target=self.executor.serve_forever, name='ubts-ui-executor', daemon=True).start()
            env.update(UBTS_EXECUTOR_URL='http://127.0.0.1:%d' % self.executor.server_address[1],
                       UBTS_EXECUTOR_TOKEN_FILE=str(self.token_file))
        else:
            env.update(UBTS_EXECUTOR_URL='', UBTS_EXECUTOR_TOKEN_FILE='')
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
        self.server = mods['app'].create_server('127.0.0.1', port)
        self.port = self.server.server_address[1]
        self.base = 'http://127.0.0.1:%d' % self.port
        self.http = threading.Thread(target=self.server.serve_forever, name='ubts-ui-http', daemon=True)
        self.http.start()
        if not self.wait(lambda: self.core.steering_host()['stop_state'] == 'confirmed', 3.0):
            self.close()
            raise AssertionError('bridge did not become ready: %r' % (self.core.steering_host(),))

    def write_sim_status(self, age_s=0.0, scene='w1_force'):
        """The sensor contract's status file as the stack writes it (with the virtual GPS origin the page must not see)."""
        status = dict(schema='ubts.steer_simulation_status.v1', updated_at=time.time() - age_s, sim_time=12.5,
                      clock_alive=True, rtf_5s=0.98, scene=scene, result_kind='자리값 기반 SIM 결과',
                      field_validated=False,
                      sensors_alive=dict(sim_scan=True, scan=True, imu=True, camera=True, camera_info=True, gps=True,
                                         steering=True),
                      gps=dict(virtual=True, origin_deg=[SECRET_LAT, SECRET_LON], display_only=True))
        (self.tmp / 'simulation_status.json').write_text(json.dumps(status), encoding='utf-8')
        (self.tmp / 'camera_driver_status.json').write_text(json.dumps(dict(
            updated_at=time.time(), state='streaming', simulated=True, width=640, height=360, fps=30.0)),
            encoding='utf-8')

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
        for server in (self.server, self.executor):
            if server is None:
                continue
            try:
                server.shutdown()
                server.server_close()
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

    def get(self, path, timeout=5):
        # the first page request versions every static file (hashes about 6 MB): slow on a shared folder (WSL /mnt)
        timeout = max(timeout, 30) if path == '/rover-control' else timeout
        try:
            with self.opener.open(self.base + path, timeout=timeout) as res:
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
            raw = exc.read()
            try:
                return exc.code, json.loads(raw.decode('utf-8'))
            except ValueError:
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
        status, body = self.page.get('/api/rover/status')             # a JSON reply gives the session cookie
        self.assertEqual(status, 200)

    def arm(self, client=None):
        client = client or self.page
        status, reply = client.post('/api/rover/arm', {'vehicle_profile': 'front_steer'})
        self.assertEqual(status, 200)
        self.assertTrue(reply.get('ok'), reply)
        self.assertTrue(self.stack.wait(lambda: self.stack.core.steering_host()['arm_state'] == 'confirmed', 2.0))
        return reply

    # -- A14: the old differential requests ----------------------------------------------------------------------
    def test_old_screen_arm_is_refused(self):
        old = Client(self.stack.base)
        status, body = old.get('/rover-control')
        self.assertEqual(status, 200)
        self.assertIn(b'roverArmButton', body)          # the differential pad is still in the page (hidden)
        old.get('/api/rover/status')          # the JSON polls give the session cookie (the HTML does not)
        before = len(self.stack.rx())
        status, reply = old.post('/api/rover/arm', {})                 # 10-rover-control.js: postJson("/api/rover/arm")
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
        """The differential pad's request shapes in the copied page: arm without a body, drive with left/right. With the
        two tests above this is A14 (old screen). The dashboard copy a92cfec sends the control requests through
        controlRequest (01-format-http.js: a POST with a per-operation timeout) instead of postJson; the request shapes
        are the same (G-SIM stage 6: this test expected postJson and failed since that copy)."""
        js = (STATIC / OLD_SCREEN_JS).read_text(encoding='utf-8')
        self.assertIn('controlRequest("/api/rover/arm")', js)
        self.assertNotIn('vehicle_profile', js)
        self.assertRegex(js, r'left: motion\.left, right: motion\.right')

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
        self.assertEqual(snap['vehicle_profile'], 'front_steer')
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


class SimExtensionTest(unittest.TestCase):
    """The simulation additions plugged into the copied dashboard (provider, extension route, page filter)."""

    def setUp(self):
        self.stack = UiStack('inproc')
        self.addCleanup(self.stack.close)
        self.page = Client(self.stack.base)

    def test_config_names_the_simulation_settings(self):
        status, config = self.page.get_json('/api/dashboard/config')
        self.assertEqual(status, 200)
        self.assertEqual(config['runtime_mode'], 'simulation')
        self.assertEqual(config['vehicle_kind'], 'front_steer')
        self.assertEqual(config['provider'], 'SimulationProvider')
        self.assertEqual(config['extension'], 'ubts_sim_dashboard.extension')
        self.assertIs(config['extra_static'], True)
        self.assertIs(config['executor_configured'], True)
        self.assertNotIn(TOKEN, json.dumps(config))

    def test_page_is_the_front_steer_simulation_page_with_the_sim_script(self):
        status, body = self.page.get('/rover-control')
        self.assertEqual(status, 200)
        text = body.decode('utf-8')
        self.assertIn('data-vehicle-kind="front_steer"', text)
        self.assertIn('data-runtime-mode="simulation"', text)
        self.assertIn('<div class="simulation-banner"', text)
        self.assertEqual(text.count('/ext-static/sim-info.js'), 1)
        status, script = self.page.get('/ext-static/sim-info.js')
        self.assertEqual(status, 200)
        self.assertIn(b'/api/sim/status', script)
        status, _ = self.page.get('/ext-static/../ubts_sim_dashboard/provider.py')
        self.assertNotEqual(status, 200)

    def test_sim_status_is_public_without_the_gps_origin(self):
        status, raw = self.page.get('/api/sim/status')
        self.assertEqual(status, 200)
        sim = json.loads(raw.decode('utf-8'))
        self.assertTrue(sim['ok'] and sim['fresh'] and sim['simulated'])
        self.assertEqual(sim['scene'], 'w1_force')
        self.assertAlmostEqual(sim['rtf_5s'], 0.98)
        self.assertTrue(all(sim['sensors_alive'].values()))
        for value in (repr(SECRET_LAT), repr(SECRET_LON)):
            self.assertNotIn(value.encode('ascii'), raw)
        self.assertNotIn('gps', sim)
        self.stack.write_sim_status(age_s=10.)
        status, sim = self.page.get_json('/api/sim/status')
        self.assertFalse(sim['fresh'])
        self.assertEqual(sim['sensors_alive'], {})

    def test_telemetry_is_the_simulation_view(self):
        status, raw = self.page.get('/api/telemetry')
        self.assertEqual(status, 200)
        data = json.loads(raw.decode('utf-8'))
        services = data['services']
        self.assertNotIn('cloudflared', services)
        self.assertEqual(services['simulation']['state'], 'gazebo')
        self.assertIs(services['simulation']['ok'], True)
        self.assertEqual(services['simulation']['scene'], 'w1_force')
        self.assertIs(data['docker']['available'], False)
        self.assertNotIn(repr(SECRET_LAT).encode('ascii'), raw)

    def test_hardware_actions_answer_simulation_only(self):
        provider = self.stack.mods['app'].DashboardHandler.provider
        self.assertIsInstance(provider, self.stack.mods['provider'].SimulationProvider)
        for action in (provider.set_fan_mode, provider.set_network_mode, provider.set_camera_settings):
            self.assertEqual(action({})['state'], 'simulation_only')
        provider.recorder_api_url = 'http://10.77.0.1:8790'       # a copied vehicle URL is refused
        self.assertEqual(provider.start_recording({'kind': 'both'})['error'], 'simulation_recorder_endpoint_required')

    def test_wifi_manager_is_disabled_in_the_simulation(self):
        # the Network page's Wi-Fi manager (dashboard 2026-11-16) answers 'disabled' and never reaches a host helper
        provider = self.stack.mods['app'].DashboardHandler.provider
        self.assertIs(provider.network_control_enabled, False)
        self.assertEqual(provider._network_mode_command('wifi-scan', {})['state'], 'disabled')
        status, wifi = self.page.get_json('/api/network/wifi')
        self.assertEqual(status, 200)
        self.assertEqual((wifi['enabled'], wifi['state'], wifi['saved']), (False, 'disabled', []))
        self.page.get('/api/rover/status')                             # session cookie
        for action, body in (('scan', {}), ('connect', {'ssid': 'x', 'password': 'p' * 8}), ('revert', {})):
            status, reply = self.page.post('/api/network/wifi/' + action, body)
            self.assertEqual(status, 503, action)
            self.assertEqual(reply['state'], 'disabled')
            self.assertNotIn('p' * 8, json.dumps(reply))
        status, reply = self.page.post('/api/network/mode', {'mode': 'ap'})
        self.assertEqual(reply['state'], 'simulation_only')


class ExecutorRelayTest(unittest.TestCase):
    """The dashboard's relay /api/executor/* against the real executor operator page (token, Host, Origin rules)."""

    def setUp(self):
        self.stack = UiStack('inproc')
        self.addCleanup(self.stack.close)
        self.page = Client(self.stack.base)
        self.page.get('/api/rover/status')                             # session cookie

    def ops(self):
        with self.stack.loop_model.lock:
            return [op for op, _ in self.stack.loop_model.ops]

    def test_status_reaches_the_operator_page_and_hides_the_token_and_the_coordinates(self):
        status, raw = self.page.get('/api/executor/status')
        self.assertEqual(status, 200)
        reply = json.loads(raw.decode('utf-8'))
        self.assertTrue(reply['ok'] and reply['reachable'], reply)
        self.assertEqual(reply['status']['run_state'], 'idle')
        self.assertEqual(reply['status']['start_mode'], 'one_press')
        self.assertEqual(reply['status']['operator_link']['timeout_s'], 0.5)
        self.assertNotIn(TOKEN.encode('ascii'), raw)
        self.assertNotIn(repr(SECRET_LAT).encode('ascii'), raw)
        self.assertNotIn('token_dir', reply['status'])

    def test_start_arm_stop_ack_end_pass_the_operator_page_rules(self):
        for action, state in (('start', 'ready'), ('arm', 'armed')):
            status, reply = self.page.post('/api/executor/' + action, {'row_len_m': None} if action == 'start' else {})
            self.assertEqual(status, 200, reply)
            self.assertEqual(reply['run_state'], state)
        status, reply = self.page.post('/api/executor/stop', {})
        self.assertEqual(status, 200, reply)
        status, reply = self.page.get_json('/api/executor/status')
        self.assertEqual(reply['status']['run_state'], 'latched')
        status, reply = self.page.post('/api/executor/ack', {})
        self.assertEqual((status, reply['run_state']), (200, 'ready'))
        status, reply = self.page.post('/api/executor/end', {})
        self.assertEqual((status, reply['run_state']), (200, 'ended'))
        self.assertEqual(self.ops(), ['start', 'arm', 'stop', 'ack', 'end'])

    def test_actions_need_the_dashboard_session_and_origin(self):
        stranger = Client(self.stack.base)
        status, _ = stranger.post('/api/executor/stop', {})
        self.assertEqual(status, 403)
        status, _ = self.page.post('/api/executor/start', {}, origin='http://evil.example')
        self.assertEqual(status, 403)
        self.assertEqual(self.ops(), [])

    def test_a_new_executor_token_is_read_again(self):
        self.stack.token_file.write_text('another-token-' + 'z' * 20 + '\n', encoding='utf-8')
        status, reply = self.page.get_json('/api/executor/status')
        self.assertEqual(reply.get('error'), 'executor_token_rejected')
        self.stack.token_file.write_text(TOKEN + '\n', encoding='utf-8')
        status, reply = self.page.get_json('/api/executor/status')
        self.assertTrue(reply['ok'], reply)

    def test_one_press_go_heartbeat_and_link_loss(self):
        if not operator_page_has_one_press(self.stack.mods):
            self.skipTest('the executor operator page of this tree has no /api/go and /api/heartbeat yet (S1)')
        session = 'simtest' + 'a' * 17
        self.page.post('/api/executor/start', {'row_len_m': None})
        self.page.post('/api/executor/arm', {})
        status, reply = self.page.post('/api/executor/heartbeat', {'session': session})
        self.assertEqual(status, 200, reply)
        status, reply = self.page.post('/api/executor/go', {'session': session})
        self.assertEqual(status, 200, reply)
        self.assertEqual(reply['run_state'], 'running')
        status, reply = self.page.post('/api/executor/heartbeat', {'session': 'other' + 'b' * 19})
        self.assertEqual((status, reply.get('error')), (409, 'session_conflict'))
        time.sleep(0.8)                                                # no heartbeat for > 0.5 s
        status, reply = self.page.get_json('/api/executor/status')
        self.assertEqual(reply['status']['run_state'], 'paused')
        self.assertEqual(reply['status']['stop_reason'], 'OPERATOR_LINK_LOST')
        status, reply = self.page.post('/api/executor/go', {'session': 'bad session!'})
        self.assertEqual((status, reply.get('error')), (400, 'invalid_session'))

    def test_executor_away(self):
        self.stack.executor.shutdown()
        self.stack.executor.server_close()
        self.stack.executor = None
        status, reply = self.page.get_json('/api/executor/status')
        self.assertEqual(status, 200)
        self.assertFalse(reply['ok'] or reply['reachable'])
        status, reply = self.page.post('/api/executor/start', {})
        self.assertEqual((status, reply.get('error')), (502, 'executor_unreachable'))


class CatalogueTest(unittest.TestCase):
    def test_fault_catalogue_matches_the_host_code(self):
        mods = dashboard_modules()
        mcu, host = js_catalogue()
        self.assertEqual(mcu, list(mods['v3'].FAULT_BIT_NAMES), 'every MCU fault bit, in spec order')
        missing = sorted(set(mods['core'].LATCH_NAMES) - set(host))
        self.assertEqual(missing, [], 'host latches without a label on the page')
        self.assertEqual(len(host), len(set(host)))

    def test_rules_file_is_utf8_lf_without_bom(self):
        raw = (STATIC / RULES_JS).read_bytes()
        self.assertFalse(raw.startswith(b'\xef\xbb\xbf'))
        self.assertNotIn(b'\r\n', raw)
        raw.decode('utf-8')


if __name__ == '__main__':
    unittest.main()
