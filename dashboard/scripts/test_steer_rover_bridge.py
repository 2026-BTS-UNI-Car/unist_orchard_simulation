"""Wrapper: argument handling, the SIM/real identity rule, publishing cadence, status file and the control socket.

No ROS (a recording publisher stands in), no real port, files only in a temporary directory.
"""
import contextlib
import io
import json
import math
import os
import socket
import sys
import tempfile
import threading
import time
import unittest

try:
    import test_steer_support as ts      # unittest discover -s dashboard/scripts
except ImportError:                     # pytest imports these modules as scripts.test_steer_*
    from scripts import test_steer_support as ts
from scripts import esp_motor_v3 as v3
from scripts import steer_rover_bridge as wrapper

UNIX = hasattr(socket, 'AF_UNIX') and sys.platform.startswith('linux')


class FakeRos:
    def __init__(self):
        self.published = []
        self.clock_queue = []          # SIM clock samples that spin_once delivers (clock, time) one at a time
        self.spins = 0

    def publish(self, topic, payload):
        json.dumps(payload, allow_nan=False)
        self.published.append((topic, payload))

    def spin_once(self, stop_when=None):
        self.spins += 1
        while self.clock_queue:
            if stop_when is not None and stop_when():
                break
            clock, value = self.clock_queue.pop(0)
            clock.t = value

    def close(self):
        pass

    def count(self, topic):
        return sum(1 for t, _ in self.published if t == topic)


def parse(*argv):
    return wrapper.build_parser().parse_args(list(argv))


class ConfigTest(unittest.TestCase):
    def test_argument_rules(self):
        cases = [
            ((), 'host profile JSON required'),
            (('--host-profile', 'p.json', '--clock', 'sim', '--no-ros'), 'cannot run with --no-ros'),
            (('--host-profile', 'p.json', '--transport', 'serial'), 'identity'),
            (('--host-profile', 'p.json', '--transport', 'pty', '--device', '/dev/ttyUSB0'), 'must be under'),
            (('--host-profile', 'p.json', '--transport', 'pty', '--device', '/run/ubts-steer/../x'), 'must be under'),
            (('--host-profile', 'p.json', '--transport', 'memory'), 'in-process device'),
            (('--host-profile', 'p.json', '--transport', 'serial', '--identity', 'x', '--max-runtime-s', '0'),
             'positive'),
            # the role binds the clock and the control socket (2026-11-09 review)
            (('--host-profile', 'p.json', '--transport', 'serial', '--identity', 'x', '--clock', 'sim'),
             'SIM clock mode'),
            (('--host-profile', 'p.json', '--transport', 'serial', '--identity', 'x', '--socket',
              '/run/ubts-steer/control.sock'), 'must not serve a control socket under /run/ubts-steer'),
            (('--host-profile', 'p.json', '--transport', 'pty', '--socket', '/run/ubts-rover/control.sock'),
             'must not serve a control socket under /run/ubts-rover'),
        ]
        for argv, text in cases:
            with self.subTest(argv=argv):
                with self.assertRaises(wrapper.ConfigError) as caught:
                    wrapper.resolve_config(parse(*argv))
                self.assertIn(text, str(caught.exception))
        with self.assertRaises(wrapper.ConfigError):
            wrapper.resolve_config(parse('--host-profile', 'p.json', '--transport', 'memory', '--device', '/x'),
                                   have_transport=True)
        for transport, root in (('serial', '/run/ubts-steer/control.sock'), ('pty', '/run/ubts-rover/control.sock')):
            with self.subTest(env_socket=root):
                saved = os.environ.get('ROVER_CONTROL_SOCKET')
                os.environ['ROVER_CONTROL_SOCKET'] = root
                try:
                    with self.assertRaises(wrapper.ConfigError):
                        wrapper.resolve_config(parse('--host-profile', 'p.json', '--transport', transport,
                                                     '--identity', 'x'))
                finally:
                    if saved is None:
                        del os.environ['ROVER_CONTROL_SOCKET']
                    else:
                        os.environ['ROVER_CONTROL_SOCKET'] = saved

    def test_roles_and_defaults(self):
        saved = {k: os.environ.pop(k, None) for k in ('ROVER_CONTROL_SOCKET', 'ROVER_SERIAL_PORT')}
        try:
            real = wrapper.resolve_config(parse('--host-profile', 'p.json', '--transport', 'serial', '--identity',
                                                'Silicon_Labs_CP2102'))
            self.assertEqual((real['simulated'], real['socket'], real['device']),
                             (False, wrapper.REAL_SOCKET, wrapper.REAL_DEVICE))
            sim = wrapper.resolve_config(parse('--host-profile', 'p.json', '--transport', 'pty'))
            self.assertEqual((sim['simulated'], sim['socket'], sim['device'], sim['identity']),
                             (True, wrapper.SIM_SOCKET, v3.SIM_DEVICE_PATH, ''))
            mem = wrapper.resolve_config(parse('--host-profile', 'p.json', '--transport', 'memory', '--no-socket'),
                                         have_transport=True)
            self.assertEqual((mem['simulated'], mem['socket']), (True, None))
            clock = wrapper.resolve_config(parse('--host-profile', 'p.json', '--transport', 'pty', '--clock', 'sim'))
            self.assertEqual((clock['clock'], clock['ros']), ('sim', True))
        finally:
            for key, value in saved.items():
                if value is not None:
                    os.environ[key] = value

    def test_main_reports_config_errors(self):
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            self.assertEqual(wrapper.main(['--transport', 'pty']), 2)
        self.assertIn('host profile JSON required', err.getvalue())
        with tempfile.TemporaryDirectory() as tmp:
            bad = os.path.join(tmp, 'bad.json')
            with open(bad, 'wb') as handle:
                handle.write(b'{"schema": "x"}')
            err = io.StringIO()
            with contextlib.redirect_stderr(err):
                self.assertEqual(wrapper.main(['--transport', 'memory', '--no-ros', '--no-socket', '--host-profile',
                                               bad], transport=v3.MemoryTransport()), 2)
            self.assertIn('host profile keys', err.getvalue())


class RunnerRig:
    """BridgeRunner on a memory transport with a fake MCU, a fake clock and a recording publisher."""

    def __init__(self, tmp, simulated=True, status_file=True, profile=None, clock_mode='wall', event_log=None,
                 **mcu):
        path = os.path.join(tmp, 'host_profile.json')
        with open(path, 'wb') as handle:
            handle.write(json.dumps(profile or ts.host_profile_data()).encode('utf-8'))
        self.clock = ts.Clock()
        self.transport = v3.MemoryTransport(clock=self.clock)
        self.mcu = ts.FakeMcu(self.transport, **mcu)
        self.ros = FakeRos()
        self.config = dict(transport='memory', device=None, identity='', simulated=simulated, clock=clock_mode,
                           host_profile=path, socket=None,
                           status_file=os.path.join(tmp, 'status', 'bridge.json') if status_file else None,
                           ros=False, node_name='test', max_runtime_s=None, event_log=event_log)
        self.runner = wrapper.BridgeRunner(self.config, transport=self.transport, clock=self.clock, ros=self.ros,
                                           wall_clock=lambda: 2.0e9)

    def run(self, seconds, step=0.02):
        for _ in range(int(round(seconds / step))):
            self.clock.t = round(self.clock.t + step, 9)
            self.mcu.tick(self.clock.t)
            self.runner.loop_once()


class RunnerTest(unittest.TestCase):
    def test_publishing_cadence_and_status_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            rig = RunnerRig(tmp)
            rig.run(2.0)
            self.assertTrue(4 <= rig.ros.count('/rover/status') <= 5)          # 2 Hz
            self.assertEqual(rig.ros.count('/rover/status'), rig.ros.count('/rover/control/requested'))
            self.assertEqual(rig.ros.count('/rover/status'), rig.ros.count('/rover/control/applied'))
            self.assertTrue(37 <= rig.ros.count('/rover/steering/state') <= 41)  # one per MCU status line (20 Hz)
            status = [p for t, p in rig.ros.published if t == '/rover/status'][-1]
            self.assertEqual(status['firmware'], 'UBTS_STEER_V3')
            self.assertIn('steering', status)
            state = [p for t, p in rig.ros.published if t == '/rover/steering/state'][-1]
            self.assertEqual(state['schema'], 'ubts.steer_state.v1')
            with open(rig.config['status_file'], 'rb') as handle:
                data = handle.read()
            self.assertNotIn(b'\r\n', data)
            self.assertEqual(json.loads(data.decode('utf-8'))['updated_at'], 2.0e9)
            rig.runner.shutdown()

    def test_one_state_message_per_status_line(self):
        # 2026-11-09 review: a loop tick that read two status lines published only the last one (A23 counts).
        with tempfile.TemporaryDirectory() as tmp:
            rig = RunnerRig(tmp, status_file=False)
            rig.run(1.0)
            rig.mcu.send_status = False
            n, ms = rig.mcu.n, rig.mcu.ms(rig.clock.t)
            before = rig.ros.count('/rover/steering/state')
            for _ in range(10):                          # a late loop: two lines 50 ms apart arrive together
                for _ in range(2):
                    n, ms = n + 1, ms + 50
                    rig.transport.device.send(v3.mcu_line('S %d %d 1 0 0 0 1 512 0 0 0 0 4000 0 0' % (ms, n)))
                rig.clock.t = round(rig.clock.t + 0.1, 9)
                rig.runner.loop_once()
            states = [p for t, p in rig.ros.published if t == '/rover/steering/state'][before:]
            self.assertEqual([p['mcu_seq'] for p in states], list(range(n - 19, n + 1)))
            self.assertEqual(rig.runner.link.counters['status_history_dropped'], 0)
            self.assertFalse(any(p['stale'] for p in states))

    def test_loop_errors_do_not_end_the_bridge(self):
        with tempfile.TemporaryDirectory() as tmp:
            rig = RunnerRig(tmp, status_file=False)
            rig.config['max_runtime_s'] = 0.2
            calls = []

            def broken():
                calls.append(1)
                raise ValueError('test defect')
            rig.runner.loop_once = broken
            err = io.StringIO()
            with contextlib.redirect_stderr(err):
                rig.runner.run(threading.Event())
            self.assertGreater(len(calls), 2)            # the loop went on after the first error
            self.assertIn('internal_error', rig.runner.core.latches)
            self.assertIn('loop error: ValueError', err.getvalue())

    def test_real_role_needs_a_fixed_cal_id(self):
        # 2026-11-09 review: cal_id null ("any") switched the calibration id check off on the vehicle too.
        any_id = ts.host_profile_data(calibration=dict(ts.host_profile_data()['calibration'], cal_id=None))
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(wrapper.ConfigError) as caught:
                RunnerRig(tmp, simulated=False, profile=any_id)
            self.assertIn('cal_id', str(caught.exception))
            rig = RunnerRig(tmp, simulated=True, profile=any_id)    # the SIM role may accept any cal_id
            self.assertIsNone(rig.runner.hp.cal_id)
            fixed = os.path.join(tmp, 'fixed.json')
            with open(fixed, 'wb') as handle:
                handle.write(json.dumps(ts.host_profile_data()).encode('utf-8'))
            config = dict(rig.config, simulated=False, clock='sim', host_profile=fixed)
            with self.assertRaises(wrapper.ConfigError) as caught:
                wrapper.BridgeRunner(config, transport=v3.MemoryTransport(), ros=FakeRos())
            self.assertIn('--clock sim', str(caught.exception))

    def test_stale_state_is_republished(self):
        with tempfile.TemporaryDirectory() as tmp:
            rig = RunnerRig(tmp, status_file=False)
            rig.run(0.5)
            rig.mcu.send_status = False
            before = rig.ros.count('/rover/steering/state')
            rig.run(2.0)
            stale = [p for t, p in rig.ros.published if t == '/rover/steering/state'][before:]
            self.assertTrue(3 <= len(stale) <= 5)
            self.assertTrue(stale[-1]['stale'])

    def test_identity_rule(self):
        arm = {'op': 'arm', 'owner': 'operator-a', 'vehicle_profile': 'front_steer'}
        with tempfile.TemporaryDirectory() as tmp:
            for simulated, sim, expected in ((True, False, 'sim_identity_mismatch'),
                                             (False, True, 'sim_identity_mismatch'), (True, True, None)):
                with self.subTest(simulated=simulated, sim=sim):
                    rig = RunnerRig(tmp, simulated=simulated, sim=sim)
                    rig.run(0.3)
                    future = rig.runner.core.submit(arm)
                    rig.run(0.02)
                    reply = future.result()
                    self.assertEqual(reply.get('error'), expected)
                    self.assertEqual(rig.runner.core.snapshot['transport'], 'simulation' if simulated else 'esp_usb')

    def test_sim_clock_needs_ros_and_main_runs_bounded(self):
        with tempfile.TemporaryDirectory() as tmp:
            rig = RunnerRig(tmp, status_file=False)
            config = dict(rig.config, clock='sim', ros=False)
            with self.assertRaises(wrapper.ConfigError):
                wrapper.BridgeRunner(config, transport=v3.MemoryTransport())
            started = time.monotonic()
            code = wrapper.main(['--transport', 'memory', '--no-ros', '--no-socket', '--host-profile',
                                 rig.config['host_profile'], '--max-runtime-s', '0.1'], transport=v3.MemoryTransport())
            self.assertEqual(code, 0)
            self.assertLess(time.monotonic() - started, 5.0)

    def test_wall_loop_keeps_a_fixed_20ms_grid(self):
        # 2026-11-09 P6a: the wall loop waited 20 ms after each tick, so the period grew above 20 ms (A17 STOP
        # latency up to about 21.7 ms); it now runs on a fixed 20 ms grid of the monotonic clock.
        with tempfile.TemporaryDirectory() as tmp:
            rig = RunnerRig(tmp, status_file=False)
            rig.config['max_runtime_s'] = 0.6
            calls = []
            rig.runner.loop_once = lambda: calls.append(time.monotonic())
            rig.runner.run(threading.Event())
            n = len(calls)
            self.assertEqual(rig.runner.grid['steps'], n)
            self.assertTrue(26 <= n <= 31, n)
            mean = (calls[-1] - calls[0]) / (n - 1)
            self.assertTrue(0.0185 <= mean <= 0.0215, mean)

    def test_sim_clock_grid(self):
        # --clock sim (P6a): one core step per 20 ms of SIM time. /clock comes in 1/120 s steps, so a step runs at
        # the first SIM time at or after its grid point (at most one /clock step late); a jump over several grid
        # points runs one step and counts the skipped points; a STOP waits for the next grid point only.
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, 'log', 'events.jsonl')
            rig = RunnerRig(tmp, clock_mode='sim', event_log=log)
            try:
                rig.clock.t = 0.0
                self.assertFalse(rig.runner.sim_grid_once())               # before the first /clock: no step
                self.assertEqual((rig.runner.grid['steps'], rig.runner.grid['polls']), (0, 1))
                dt = 1.0 / 120.0
                times = []

                def advance(k0, count):
                    for k in range(k0, k0 + count):
                        rig.clock.t = round(3.0 + k * dt, 9)
                        rig.mcu.tick(rig.clock.t)
                        if rig.runner.sim_grid_once():
                            times.append(rig.clock.t)
                advance(0, 240)                                              # 2 s of SIM time, grid 3.00 ... 4.98
                self.assertEqual(len(times), 100)
                self.assertEqual((rig.runner.grid['steps'], rig.runner.grid['skipped']), (100, 0))
                for t in times:
                    lag = t - math.floor(t / 0.02 + 1e-9) * 0.02
                    self.assertTrue(-1e-9 <= lag < dt + 1e-9, (t, lag))
                self.assertEqual(rig.runner.core.steering_host()['stop_state'], 'confirmed')
                # a STOP submitted at 4.9917 s (between grid points) is written at the step of the 5.00 s grid point
                self.assertAlmostEqual(rig.clock.t, 3.0 + 239 * dt, places=6)
                self.assertEqual(rig.runner.core.submit({'op': 'stop'}).result(0), {'ok': True,
                                                                                    'state': 'stop_requested'})
                advance(240, 1)
                # a jump of 85 ms over 4 grid points: one step, 4 skipped, the grid stays on the 20 ms multiples
                due = rig.runner.grid['next_t']
                rig.clock.t = round(due + 0.085, 9)
                self.assertTrue(rig.runner.sim_grid_once())
                self.assertEqual(rig.runner.grid['skipped'], 4)
                self.assertAlmostEqual(rig.runner.grid['next_t'], due + 0.1, places=9)
                advance(253, 80)                                 # 5.108 s on
                with open(rig.config['status_file'], 'rb') as handle:
                    loop = json.loads(handle.read().decode('utf-8'))['bridge_loop']
                self.assertEqual((loop['clock'], loop['skipped']), ('sim', 4))
                self.assertLessEqual(loop['steps'], rig.runner.grid['steps'])
                rig.runner.event_log.flush()
                with open(log, 'rb') as handle:
                    data = handle.read()
                self.assertNotIn(b'\r', data)
                rows = [json.loads(line) for line in data.decode('utf-8').splitlines()]
                stops = [r for r in rows if r['kind'] == 'tx' and r['head'].startswith('STOP') and r['t'] > 4.99]
                self.assertTrue(stops and abs(stops[0]['t'] - 5.0) < 1e-6, stops[:1])     # the injected SIM clock
                self.assertTrue(any(r['kind'] == 'state' and r['stop_state'] == 'confirmed' for r in rows))
            finally:
                rig.runner.shutdown()

    def test_sim_clock_stop_does_not_wait_for_the_grid(self):
        # 2026-11-09 review RG-04: in SIM clock mode a STOP waited for the next 20 ms grid point reached by a 120 Hz
        # /clock sample (up to 25 ms of SIM time, above the A17 20 ms). A pending STOP now runs one extra core step
        # at the next poll, at the SIM time it arrived in; the grid keeps its next point.
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, 'events.jsonl')
            rig = RunnerRig(tmp, clock_mode='sim', event_log=log, status_file=False)
            try:
                dt = 1.0 / 120.0
                for k in range(120):                                    # 1 s of SIM time
                    rig.clock.t = round(2.0 + k * dt, 9)
                    rig.mcu.tick(rig.clock.t)
                    rig.runner.sim_grid_once()
                steps, due = rig.runner.grid['steps'], rig.runner.grid['next_t']
                rig.clock.t = round(2.0 + 120 * dt, 9)                  # 3.0: on the grid (step)
                rig.mcu.tick(rig.clock.t)
                self.assertTrue(rig.runner.sim_grid_once())
                rig.clock.t = round(2.0 + 121 * dt, 9)                  # 3.0083: between grid points, no step
                rig.mcu.tick(rig.clock.t)
                self.assertFalse(rig.runner.sim_grid_once())
                front = wrapper.LoggedCore(rig.runner.core, rig.runner.event_log)
                self.assertEqual(front.request({'op': 'stop', 'owner': 't'}), {'ok': True, 'state': 'stop_requested'})
                # /clock samples queued behind the STOP are not taken before the STOP step
                rig.ros.clock_queue = [(rig.clock, round(2.0 + 122 * dt, 9)), (rig.clock, round(2.0 + 123 * dt, 9))]
                t_request = rig.clock.t
                self.assertTrue(rig.runner.sim_grid_once())             # the extra step at once
                self.assertEqual(rig.clock.t, t_request)
                self.assertEqual(rig.runner.grid['urgent_steps'], 1)
                self.assertEqual(rig.runner.grid['steps'], steps + 1)
                self.assertAlmostEqual(rig.runner.grid['next_t'], due + 0.02, places=9)
                self.assertFalse(rig.runner.stop_pending())
                self.assertEqual(len(rig.ros.clock_queue), 2)          # not taken before the STOP step
                rig.ros.clock_queue = []
                for k in range(122, 200):                              # the grid goes on; resends on the grid
                    rig.clock.t = round(2.0 + k * dt, 9)
                    rig.mcu.tick(rig.clock.t)
                    rig.runner.sim_grid_once()
                rig.runner.event_log.flush()
                with open(log, 'rb') as handle:
                    rows = [json.loads(line) for line in handle.read().decode('utf-8').splitlines()]
                request = [r for r in rows if r['kind'] == 'request' and r['op'] == 'stop'][0]
                stop_tx = [r for r in rows if r['kind'] == 'tx' and r['head'].startswith('STOP') and
                           r['t'] >= request['t']]
                self.assertTrue(stop_tx)
                self.assertAlmostEqual(stop_tx[0]['t'] - request['t'], 0.0, places=9)      # <= one /clock sample
                ticks = [r['t'] for r in rows if r['kind'] == 'tick' and r['t'] >= request['t']]
                self.assertTrue(ticks and abs(ticks[0] - stop_tx[0]['t']) < 1e-9, ticks[:3])
                confirmed = [r for r in rows if r['kind'] == 'state' and r['stop_state'] == 'confirmed' and
                             r['t'] >= request['t']]
                self.assertTrue(confirmed)
                self.assertTrue(all(t <= confirmed[0]['t'] + 1e-9 for t in ticks))     # only while pending
            finally:
                rig.runner.shutdown()

    def test_wall_loop_logs_ticks_while_a_stop_is_pending(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, 'events.jsonl')
            rig = RunnerRig(tmp, status_file=False, event_log=log)
            rig.run(0.5)
            t_submit = rig.clock.t
            rig.runner.core.submit({'op': 'stop'})
            rig.run(0.5)
            rig.runner.shutdown()
            with open(log, 'rb') as handle:
                rows = [json.loads(line) for line in handle.read().decode('utf-8').splitlines()]
            ticks = [r['t'] for r in rows if r['kind'] == 'tick' and r['t'] > t_submit]     # (the startup STOP too)
            stops = [r['t'] for r in rows if r['kind'] == 'tx' and r['head'].startswith('STOP') and r['t'] > t_submit]
            self.assertTrue(ticks and stops)
            self.assertAlmostEqual(ticks[0], stops[0], places=9)        # the STOP tick itself
            gaps = [round(b - a, 6) for a, b in zip(ticks, ticks[1:])]
            self.assertTrue(all(abs(g - 0.02) < 1e-6 for g in gaps), gaps)
            self.assertLessEqual(len(ticks), 10)                        # only until the STOP is confirmed

    def test_event_log_requests_and_rotation(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, 'events.jsonl')
            rig = RunnerRig(tmp, status_file=False, event_log=log)
            self.assertIsInstance(rig.runner.transport, wrapper.LoggedTransport)
            rig.run(0.3)
            front = wrapper.LoggedCore(rig.runner.core, rig.runner.event_log)
            self.assertEqual(front.request({'op': 'stop', 'owner': 'tester'}), {'ok': True,
                                                                                 'state': 'stop_requested'})
            self.assertEqual(front.request({'op': 5})['error'], 'unknown_operation')
            rig.runner.shutdown()
            with open(log, 'rb') as handle:
                rows = [json.loads(line) for line in handle.read().decode('utf-8').splitlines()]
            requests = [r for r in rows if r['kind'] in ('request', 'reply')]
            self.assertEqual([(r['kind'], r['op']) for r in requests],
                             [('request', 'stop'), ('reply', 'stop'), ('request', None), ('reply', None)])
            self.assertEqual((requests[0]['owner'], requests[0]['t']), ('tester', rig.clock.t))
            self.assertTrue(requests[1]['ok'])
            self.assertTrue(any(r['kind'] == 'tx' for r in rows))
            # without a path nothing is written and the transport is not wrapped
            plain = RunnerRig(tmp, status_file=False)
            self.assertFalse(plain.runner.event_log.enabled)
            self.assertNotIsInstance(plain.runner.transport, wrapper.LoggedTransport)
            plain.runner.shutdown()
            # rotation: one previous file is kept as .1, the newest rows are in the current file
            path = os.path.join(tmp, 'sub', 'rot.jsonl')
            small = wrapper.EventLog(path, lambda: 1.5, rotate_bytes=400)
            for k in range(20):
                small.write('tx', size=k, head='STOP')
            small.close()
            small.write('tx', size=99)                                   # after close: ignored
            texts = []
            for name in (path + '.1', path):
                with open(name, 'rb') as handle:
                    texts.append(handle.read())
            self.assertTrue(all(b'\r' not in t and len(t) < 500 for t in texts))
            sizes = [json.loads(line)['size'] for t in texts for line in t.decode('utf-8').splitlines()]
            self.assertEqual(sizes, list(range(20 - len(sizes), 20)))
            self.assertLess(len(sizes), 20)

    @unittest.skipUnless(UNIX, 'Unix control socket needs Linux')
    def test_second_bridge_on_the_same_socket_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            rig = RunnerRig(tmp, status_file=False)
            rig.config['socket'] = os.path.join(tmp, 'run', 'control.sock')
            rig.runner.start()
            try:
                err = io.StringIO()
                with contextlib.redirect_stderr(err):
                    code = wrapper.main(['--transport', 'memory', '--no-ros', '--host-profile',
                                         rig.config['host_profile'], '--socket', rig.config['socket'],
                                         '--max-runtime-s', '0.1'], transport=v3.MemoryTransport())
                self.assertEqual(code, 2)
                self.assertTrue(os.path.exists(rig.config['socket']))     # the live peer's socket is kept
            finally:
                rig.runner.shutdown()

    @unittest.skipUnless(UNIX, 'Unix control socket needs Linux')
    def test_socket_protocol(self):
        with tempfile.TemporaryDirectory() as tmp:
            rig = RunnerRig(tmp, status_file=False)
            rig.config['socket'] = os.path.join(tmp, 'run', 'control.sock')
            rig.runner.start()
            try:
                rig.run(0.3)
                self.assertEqual(os.stat(rig.config['socket']).st_mode & 0o777, 0o660)

                def ask(raw):
                    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                        client.settimeout(2.0)
                        client.connect(rig.config['socket'])
                        client.sendall(raw)
                        data = b''
                        while not data.endswith(b'\n'):
                            part = client.recv(65536)
                            if not part:
                                break
                            data += part
                    return json.loads(data)
                status = ask(b'{"op": "status"}\n')
                self.assertEqual((status['firmware'], status['state']), ('UBTS_STEER_V3', 'disarmed'))
                self.assertEqual(ask(b'{"op": "stop"}\n'), {'ok': True, 'state': 'stop_requested'})
                self.assertEqual(ask(b'[1, 2]\n'), {'ok': False, 'error': 'object_required'})
                self.assertFalse(ask(b'not json\n')['ok'])
                self.assertEqual(ask(b'{"x": "' + b'a' * 5000 + b'"}\n'), {'ok': False, 'error': 'request_too_large'})
                # a queued request is answered by the loop thread (threaded use of the core)
                stop = threading.Event()

                def loop():
                    while not stop.is_set():
                        rig.clock.t = round(rig.clock.t + 0.02, 9)
                        rig.mcu.tick(rig.clock.t)
                        rig.runner.loop_once()
                        time.sleep(0.005)
                thread = threading.Thread(target=loop)
                thread.start()
                try:
                    reply = ask(b'{"op": "arm", "owner": "socket-client", "vehicle_profile": "front_steer"}\n')
                finally:
                    stop.set()
                    thread.join()
                self.assertTrue(reply['ok'])
                self.assertEqual(reply['status']['vehicle_profile'], 'front_steer')
            finally:
                rig.runner.shutdown()
            self.assertFalse(os.path.exists(rig.config['socket']))


if __name__ == '__main__':
    unittest.main()
