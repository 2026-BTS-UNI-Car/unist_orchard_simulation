"""Executor container and its checks (WF-4; pure parts: no ROS, no Gazebo, no files in the work tree).

  * steering.exec_container_main: the G-SIM executor configuration equals the T1 configuration
    (tools/row_executor/sim_in_loop_steer.steer_executor_config) except the SIM socket and the operator page address;
    it validates (front_steer, MOTOR, steer_allow_sim with the SIM socket, 127.0.0.1); the controller slot; the SIM
    clock (never backwards); the SIM clock loop (one step per 50 ms grid point of SIM time, missed points counted and
    never caught up, the MOTOR exception rule); the stall hook (the main loop hangs, nothing else); the SIM hooks
    (controller refused while ACTIVE, stall / freeze / kill / wall offset with their bounds); the files (status
    retirement, log pruning); the whole container object without ROS (real ExecutorLoop, Runner, SteerBridgeClient)
  * compose.ubts-steer.yaml: the executor service (profile executor, sim network and IPC, control volume only, no
    runtime or hooks volume, 0.5 GiB, 1-2 CPU, the clock of the stack, the command line parses), the 6 GiB budget
    (whole stack with the executor = 6.0; checks with the executor run without the dashboard); the Dockerfile copies
    tools/row_executor and imports the new modules; the release grace of the bridge is the vehicle's 1.1 s
  * steering.exec_checks: the A13 R6 judgement with amendment 11, the S2 node rule, the graph listing, the chain
    latency by header stamp, rates, the A5 format items, the yaw-offset convention geometry, the IMU noise model, the
    camera WebSocket frame parser and the JPEG size

unittest style (pytest on 3.12 / 3.13). Run this file in its own pytest process (it imports tools/row_executor).
Placeholder-based SIM results only.
"""
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import types
import unittest

import numpy as np
import yaml

SIM_ROOT = Path(__file__).resolve().parent
REPO = SIM_ROOT.parents[1]
for _path in (REPO / 'tools', REPO / 'dashboard'):
    if str(_path) not in sys.path:
        sys.path.append(str(_path))

from steering import exec_checks as EC  # noqa: E402
from steering import exec_container_main as EM  # noqa: E402
from steering import p6a_graph  # noqa: E402
from steering import s2_format as S2  # noqa: E402
from steering.vehicle_profile import load_profile  # noqa: E402

COMPOSE = REPO / 'compose.ubts-steer.yaml'
DOCKERFILE = REPO / 'docker' / 'Dockerfile.ubts-steer'


def executor_importable():
    try:
        import row_executor.config  # noqa: F401
        import scripts.steer_bridge_core  # noqa: F401
        return True
    except ImportError:
        return False


def runner_importable():
    """The executor's Runner needs the vendored estimator (scipy): decision R10 runs those tests on Python 3.12 and 3.13
    only (the WSL 3.10 scipy does not import); no scipy import here."""
    if sys.version_info < (3, 12):
        return False
    try:
        from row_executor import core, executor_main  # noqa: F401
        return True
    except ImportError:
        return False


@unittest.skipUnless(executor_importable(), 'tools/row_executor or dashboard/scripts not importable')
class ConfigTest(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile()
        self.host = EM.host_profile_of(self.profile, 1)

    def test_config_values(self):
        cfg = EM.gsim_executor_config(self.profile, self.host, 1)
        from row_executor import config as XC
        self.assertEqual(cfg.vehicle_profile, XC.FRONT_STEER)
        self.assertEqual(cfg.mode, XC.SINGLE_ROW_MODE)
        self.assertTrue(cfg.motor_output_enabled)
        self.assertTrue(cfg.steer_allow_sim)
        self.assertEqual(cfg.steer_cal_id, 1)
        self.assertEqual(cfg.socket_path, '/run/ubts-steer/control.sock')
        self.assertEqual((cfg.bind_host, cfg.bind_port), ('127.0.0.1', 8097))
        self.assertEqual((cfg.steer_limit_left_mrad, cfg.steer_limit_right_mrad),
                         (self.host.limit_left_mrad, self.host.limit_right_mrad))
        self.assertEqual(cfg.lidar_yaw_offset_deg, float(self.profile.value('lidar_yaw_offset_deg')))
        self.assertEqual(cfg.feasibility()['status'], 'ok', cfg.feasibility())
        over = EM.gsim_executor_config(self.profile, self.host, 1, dict(steer_cal_id=2, base_duty_pct=45))
        self.assertEqual((over.steer_cal_id, over.base_duty_pct), (2, 45))

    @unittest.skipUnless(runner_importable(), 'the T1 harness needs the executor core and scipy')
    def test_same_values_as_the_t1_configuration(self):
        from row_executor import sim_in_loop_steer as T1
        rig = types.SimpleNamespace(profile=self.profile, hp=self.host, config=types.SimpleNamespace(cal_id=1))
        try:
            t1 = T1.steer_executor_config(rig).to_dict()
        except Exception as exc:          # a T1 rule of another stage (e.g. a SIM socket rule) may refuse the default
            self.skipTest('T1 configuration not buildable here: %s' % exc)
        ours = EM.gsim_executor_config(self.profile, self.host, 1).to_dict()
        skip = {'socket_path', 'bind_host', 'bind_port'}
        self.assertEqual({k: v for k, v in ours.items() if k not in skip},
                         {k: v for k, v in t1.items() if k not in skip})

    def test_json_arguments(self):
        self.assertIsNone(EM.parse_json_arg('', 'x'))
        self.assertEqual(EM.parse_json_arg('{"a": 1}', 'x'), {'a': 1})
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'spec.json'
            path.write_bytes(b'{"kind": "hold_still"}')
            self.assertEqual(EM.parse_json_arg(str(path), 'x'), {'kind': 'hold_still'})
        for bad in ('[1, 2]', '{bad'):
            with self.assertRaises(EM.ContainerError):
                EM.parse_json_arg(bad, 'x')

    def test_controller_slot(self):
        cfg = EM.gsim_executor_config(self.profile, self.host, 1)
        slot = EM.ControllerSlot(EM.DEFAULT_CONTROLLER, cfg)
        first = slot()
        self.assertEqual(type(first).__name__, 'StraightHold')
        digest = slot.set(dict(kind='hold_still', duration_s=2.))
        self.assertEqual(type(slot()).__name__, 'HoldStill')
        self.assertEqual(slot.built, 2)
        self.assertEqual(len(digest), 64)
        with self.assertRaises(ValueError):
            slot.set(dict(kind='pivot'))
        self.assertEqual(slot.spec['kind'], 'hold_still')                 # a refused spec keeps the old one


class ClockAndLoopTest(unittest.TestCase):
    def test_sim_clock(self):
        clock = EM.SimClock()
        self.assertFalse(clock.started())
        self.assertEqual(clock(), 0.)
        clock.update(10, 500_000_000)
        clock.update(10, 400_000_000)                                    # a step back is not followed
        clock.update(11, 0)
        self.assertTrue(clock.started())
        self.assertEqual(clock(), 11.)
        self.assertEqual((clock.samples, clock.backwards), (3, 1))

    def test_wall_clock_offset(self):
        wall = EM.WallClock(base=lambda: 100.)
        self.assertEqual(wall(), 100.)
        wall.offset_s = 3600.
        self.assertEqual(wall(), 3700.)

    def run_grid(self, sim_samples, raise_at=()):
        """Feed SIM time samples from a thread; returns (steps at SIM times, grid, loop)."""
        clock = EM.SimClock()
        steps = []

        class Loop:
            counts = {}
            motor_allowed = True
            events = []
            stops = []

            def step(self_inner):
                steps.append(round(clock(), 4))
                if len(steps) in raise_at:
                    raise RuntimeError('boom')

            def _event(self_inner, name, **fields):
                self_inner.events.append(name)

            def emergency_stop(self_inner, source):
                self_inner.stops.append(source)

            def note_loop_exception(self_inner, text):
                pass
        loop = Loop()
        stop = threading.Event()
        done = []
        grid = {}

        def feed():
            for t in sim_samples:
                sec = int(t)
                seen = grid.get('polls', 0)
                clock.update(sec, int(round((t - sec) * 1e9)))
                # the loop polls every sample at least once before the next one comes: with the sleep alone the
                # loop's first poll could see a later sample first (Windows timer granularity, about 15 ms)
                end = time.monotonic() + 2.
                while grid.get('polls', 0) < seen + 2 and time.monotonic() < end:
                    time.sleep(.001)
                time.sleep(.006)
            time.sleep(.05)
            stop.set()
            done.append(True)
        threading.Thread(target=feed, daemon=True).start()
        EM.run_loop_sim(loop, stop, clock, hz=20., poll_s=.001, grid=grid)
        return steps, grid, loop

    def test_one_step_per_grid_point(self):
        samples = [1. + k / 120. for k in range(0, 61)]                # 0.5 s of SIM time at 120 Hz
        steps, grid, loop = self.run_grid(samples)
        self.assertEqual(grid['steps'], len(steps))
        # every grid point 1.00 .. 1.50 is either stepped once or counted as missed (a late poll), never caught up
        self.assertIn(len(steps) + grid['skipped'], (10, 11))
        points = [int(math.floor(t / .05 + 1e-9)) for t in steps]
        self.assertEqual(points, sorted(set(points)))                    # never two steps on one grid point

    def test_a_jump_is_one_step_and_counted(self):
        samples = [1., 1. + 1. / 120., 1.5, 1.5 + 1. / 120.]           # SIM time jumps 0.49 s
        steps, grid, loop = self.run_grid(samples)
        self.assertEqual(grid['skipped'], 9)
        self.assertEqual(loop.counts.get('missed_ticks'), 9)
        self.assertEqual(len(steps), 2)

    def test_exception_stops_the_vehicle(self):
        samples = [1. + k / 120. for k in range(0, 13)]
        steps, grid, loop = self.run_grid(samples, raise_at=(1,))
        self.assertEqual(loop.stops, ['loop_exception'])
        self.assertIn('loop_exception', loop.events)

    def test_stall_hangs_the_step_only(self):
        state = EM.HooksState()
        calls = []
        inner = types.SimpleNamespace(step=lambda now=None: calls.append(time.monotonic()), counts={'x': 1})
        stepped = EM.SteppedLoop(inner, state)
        state.stall_until = time.monotonic() + .15
        started = time.monotonic()
        stepped.step()
        self.assertGreaterEqual(calls[0] - started, .14)
        self.assertIsNone(state.stall_until)
        self.assertEqual(state.stalls_done, 1)
        self.assertEqual(stepped.counts, {'x': 1})                       # other attributes are the loop's


class HooksTest(unittest.TestCase):
    def make(self, state_name='PAUSED'):
        self.kills = []
        runner = types.SimpleNamespace(state=state_name)
        loop = types.SimpleNamespace(runner=runner)

        class Slot:
            spec = {'kind': 'hold_still'}

            def set(self_inner, spec):
                if spec.get('kind') not in ('hold_still', 'segments', 'straight_hold'):
                    raise ValueError('unknown scripted controller kind')
                self_inner.spec = spec
                return 'd' * 64
        self.wall = EM.WallClock(base=lambda: 10.)
        return EM.ExecHooks('/nowhere.sock', EM.HooksState(), loop=loop, slot=Slot(), wall=self.wall,
                            status_fn=lambda: {'x': 1}, pid=4242, kill=lambda pid, sig: self.kills.append((pid, sig)))

    def test_controller_and_status(self):
        hooks = self.make()
        self.assertEqual(hooks.handle({'op': 'status'})['status'], {'x': 1})
        self.assertTrue(hooks.handle({'op': 'controller', 'spec': {'kind': 'segments'}})['ok'])
        self.assertFalse(hooks.handle({'op': 'controller', 'spec': {'kind': 'pivot'}})['ok'])
        busy = self.make('ACTIVE')
        self.assertEqual(busy.handle({'op': 'controller', 'spec': {'kind': 'segments'}})['error'], 'run_active')
        self.assertEqual(hooks.handle({'op': 'nope'})['error'], 'unknown_op')

    def test_stall_and_bounds(self):
        hooks = self.make()
        reply = hooks.handle({'op': 'stall', 'duration_s': 1.5})
        self.assertTrue(reply['ok'])
        self.assertGreater(hooks.state.stall_until, time.monotonic())
        for bad in (0, -1, 31, 'x', True, float('nan')):
            self.assertFalse(hooks.handle({'op': 'stall', 'duration_s': bad})['ok'], bad)

    @unittest.skipUnless(hasattr(__import__('signal'), 'SIGKILL'), 'POSIX signals')
    def test_freeze_and_kill_run_after_the_reply(self):
        import signal
        hooks = self.make()
        self.assertTrue(hooks.handle({'op': 'kill'})['ok'])
        self.assertEqual(self.kills, [])                                   # nothing before the reply is out
        hooks.after_reply()
        self.assertEqual(self.kills, [(4242, signal.SIGKILL)])
        self.assertTrue(hooks.handle({'op': 'freeze', 'duration_s': .01})['ok'])
        hooks.after_reply()                                                # the helper continues pid 4242 (absent)
        self.assertEqual(self.kills[-1], (4242, signal.SIGSTOP))

    def test_wall_offset(self):
        hooks = self.make()
        reply = hooks.handle({'op': 'wall_offset', 'seconds': 3600.})
        self.assertEqual(reply['offset_s'], 3600.)
        self.assertEqual(self.wall(), 3610.)
        self.assertFalse(hooks.handle({'op': 'wall_offset', 'seconds': 1e9})['ok'])
        self.assertFalse(hooks.handle({'op': 'wall_offset', 'seconds': 'x'})['ok'])


class FilesTest(unittest.TestCase):
    def test_retire_status_and_prune_logs(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            self.assertFalse(EM.retire_status(folder, 't0'))
            (folder / 'status.json').write_text('{}')
            self.assertTrue(EM.retire_status(folder, 't1'))
            self.assertTrue((folder / 'retired' / 't1' / 'status.json').is_file())
            logs = folder / 'log'
            logs.mkdir()
            for k in range(6):
                path = logs / ('row_executor_%d.jsonl' % k)
                path.write_text('{}\n')
                os.utime(path, (1000. + k, 1000. + k))
            (logs / 'operator_token.txt').write_text('x' * 24)
            self.assertEqual(EM.prune_logs(logs, keep=2), 4)
            self.assertEqual(sorted(p.name for p in logs.iterdir()),
                             ['operator_token.txt', 'row_executor_4.jsonl', 'row_executor_5.jsonl'])


@unittest.skipUnless(executor_importable() and runner_importable(), 'the executor package, scipy')
class ContainerTest(unittest.TestCase):
    def test_container_without_ros(self):
        with tempfile.TemporaryDirectory() as tmp:
            for clock in ('wall', 'sim'):
                args = EM.build_parser().parse_args(['--clock', clock, '--exec-dir', str(Path(tmp) / clock),
                                                     '--skip-source-check'])
                container = EM.Container(args)
                try:
                    loop = container.loop
                    self.assertTrue(loop.motor_allowed)
                    self.assertTrue(loop.steering)
                    self.assertIs(loop.clock, container.clock)
                    if clock == 'sim':
                        self.assertIsInstance(loop.clock, EM.SimClock)
                    else:
                        self.assertIs(loop.clock, time.monotonic)
                    client = loop._bridge_factory()
                    self.assertTrue(client.allow_sim)
                    self.assertEqual(client.owner, container.owner)
                    self.assertIs(client.clock, container.clock)
                    status = container.status()
                    self.assertEqual(status['clock'], clock)
                    self.assertEqual(status['runner']['state'], 'IDLE')
                    described = json.loads((Path(tmp) / clock / 'config.json').read_bytes().decode('utf-8'))
                    self.assertEqual(described['config_digest'], container.cfg.digest())
                    self.assertEqual(described['topics']['clock'], '/clock' if clock == 'sim' else None)
                    self.assertTrue(described['source_check'].startswith('skipped'))
                    reply = container.hooks.handle({'op': 'controller', 'spec': {'kind': 'hold_still'}})
                    self.assertTrue(reply['ok'], reply)
                    self.assertEqual(type(loop.runner._new_controller()).__name__, 'HoldStill')
                finally:
                    container.log.close()

    def test_source_check_refusal_is_explicit(self):
        from row_executor import executor_main as XM
        saved = XM.verify_sources
        XM.verify_sources = lambda *a, **k: (_ for _ in ()).throw(RuntimeError('source tree does not match'))
        try:
            with tempfile.TemporaryDirectory() as tmp:
                args = EM.build_parser().parse_args(['--exec-dir', tmp])
                with self.assertRaises(EM.ContainerError):
                    EM.Container(args)
        finally:
            XM.verify_sources = saved


class ComposeTest(unittest.TestCase):
    def setUp(self):
        if not COMPOSE.is_file():
            raise unittest.SkipTest('compose.ubts-steer.yaml not next to the work tree')
        self.data = yaml.safe_load(COMPOSE.read_bytes().decode('utf-8'))
        self.services = self.data['services']

    @staticmethod
    def gib(text):
        text = str(text)
        return float(text[:-1]) if text.endswith('g') else float(text[:-1]) / 1024.

    def test_executor_service(self):
        ex = self.services['executor']
        self.assertEqual(ex['profiles'], ['executor'])
        self.assertEqual((ex['network_mode'], ex['ipc']), ('service:sim', 'service:sim'))
        self.assertEqual(ex['volumes'], ['control:/run/ubts-steer'])
        self.assertEqual(self.gib(ex['mem_limit']), .5)
        self.assertTrue(1. <= float(ex['cpus']) <= 2.)
        self.assertEqual((ex['restart'], ex['stop_grace_period']), ('no', '20s'))
        self.assertTrue(ex['init'])
        env = ex['environment']
        self.assertEqual(env['ROS_DOMAIN_ID'], '43')
        self.assertEqual(env['ROVER_CONTROL_SOCKET'], '/run/ubts-steer/control.sock')
        self.assertIn('/opt/ubts/tools', env['PYTHONPATH'].split(':'))
        self.assertEqual(env['UBTS_STEER_CLOCK'], '${UBTS_STEER_CLOCK:-wall}')
        for forbidden in ('UBTS_STEER_HOOK_SOCKET', 'TELEMETRY_FILE'):
            self.assertNotIn(forbidden, env)
        command = ex['command']
        self.assertEqual(command[:4], ['python3', '-B', '-m', 'steering.exec_container_main'])
        args = EM.build_parser().parse_args(command[4:])
        self.assertIn(args.clock, ('wall', 'sim'))
        self.assertEqual(ex['depends_on'], {'steer': {'condition': 'service_started'}})

    def test_budget(self):
        mem = {name: self.gib(service['mem_limit']) for name, service in self.services.items()}
        self.assertAlmostEqual(sum(mem[n] for n in ('sim', 'dashboard', 'steer', 'recorder', 'executor')), 6.)
        self.assertLessEqual(sum(mem[n] for n in ('sim', 'steer', 'recorder', 'executor', 'check')), 6.)
        self.assertLessEqual(sum(mem[n] for n in ('sim', 'dashboard', 'steer', 'recorder', 'check')), 6.)
        self.assertEqual(self.services['check']['profiles'], ['check'])
        self.assertIn('control:/run/ubts-steer', self.services['check']['volumes'])     # the executor's files
        self.assertNotIn('profiles', self.services['sim'])
        self.assertEqual(self.data['x-environment']['UBTS_STEER_S2_NODE'], '${UBTS_STEER_S2_NODE:-}')

    def test_dockerfile_copies_the_executor(self):
        if not DOCKERFILE.is_file():
            self.skipTest('docker/ not here')
        text = DOCKERFILE.read_bytes().decode('utf-8')
        self.assertIn('COPY tools/row_executor /opt/ubts/tools/row_executor', text)
        self.assertIn("'tools/row_executor', \\", text)                    # in the manifest roots
        self.assertIn("'firmware'); \\", text)
        self.assertIn('from steering import s2_format_node, exec_container_main, exec_checks', text)
        self.assertIn('from steering import gz_esp_emulator, supervisor_steer, sensor_contract_steer', text)
        ignore = (REPO / 'docker' / 'Dockerfile.ubts-steer.dockerignore').read_bytes().decode('utf-8').splitlines()
        for line in ('!tools/', '!tools/row_executor/', '!tools/row_executor/**'):
            self.assertIn(line, ignore)

    @unittest.skipUnless(executor_importable(), 'dashboard/scripts not importable')
    def test_release_grace_is_the_vehicle_value(self):
        from scripts.rover_control_core import RoverControl
        from scripts.steer_control_core import SteerControl
        self.assertEqual(SteerControl.RELEASE_TTL, 1.1)
        self.assertEqual(RoverControl.INPUT_TTL, .75)


class CheckHelpersTest(unittest.TestCase):
    def graph(self, executor_topics, s2_topics=('/sim/scan',)):
        def t(pubs, subs):
            return dict(types=['x'], publishers=list(pubs), subscribers=list(subs))
        topics = {}
        for topic in executor_topics:
            topics.setdefault(topic, t([], []))['subscribers'].append(p6a_graph.EXECUTOR_NODE)
        for topic in s2_topics:
            topics.setdefault(topic, t([], []))['subscribers'].append(p6a_graph.S2_NODE)
        topics.setdefault('/scan', t([], []))['publishers'].append(p6a_graph.S2_NODE)
        return dict(nodes=[p6a_graph.EXECUTOR_NODE, p6a_graph.S2_NODE], topics=topics)

    def test_r6_with_amendment_11(self):
        base = ['/scan', '/imu/data_raw', '/rover/control/requested', '/parameter_events', '/rosout']
        self.assertTrue(EC.judge_executor_subscriptions(self.graph(base), 'wall')['passed'])
        sim = EC.judge_executor_subscriptions(self.graph(base + ['/clock']), 'sim')
        self.assertTrue(sim['passed'])
        self.assertFalse(sim['registered_without_amendment']['passed'])
        wall = EC.judge_executor_subscriptions(self.graph(base + ['/clock']), 'wall')
        self.assertFalse(wall['passed'])
        self.assertTrue(wall['clock_in_wall_mode'])
        for topic in ('/ground_truth/odom', '/odom', '/tf', '/joint_states', '/sim/truth/steering', '/gps/fix',
                      '/sim/scan'):
            result = EC.judge_executor_subscriptions(self.graph(base + [topic]), 'sim')
            self.assertFalse(result['passed'], topic)
            self.assertIn(topic, result['truth_or_forbidden'])
        missing = EC.judge_executor_subscriptions(self.graph(['/scan']), 'wall')
        self.assertTrue(missing['passed'])
        self.assertEqual(missing['expected_not_subscribed'], ['/imu/data_raw', '/rover/control/requested'])
        absent = dict(nodes=[], topics={})
        self.assertIsNone(EC.judge_executor_subscriptions(absent, 'wall')['passed'])

    def test_s2_rule_and_listing(self):
        g = self.graph(['/scan'], s2_topics=('/sim/scan_clipped', '/rosout'))
        self.assertTrue(EC.judge_s2_subscriptions(g, '/sim/scan_clipped')['passed'])
        self.assertFalse(EC.judge_s2_subscriptions(g, '/sim/scan')['passed'])
        g['topics']['/ground_truth/odom'] = dict(types=['x'], publishers=[], subscribers=[p6a_graph.S2_NODE])
        self.assertFalse(EC.judge_s2_subscriptions(g, '/sim/scan_clipped')['passed'])
        text = EC.graph_listing(g)
        self.assertIn('/scan | x | pub: ubts_steer_s2_format', text)
        self.assertTrue(text.endswith('\n'))

    def test_chain_latency(self):
        clock_rows = [(100. + k * .01, 5. + k * .01) for k in range(0, 300)]   # RTF 1: wall = SIM + 95
        stamps = [int(round((5.5 + k * .1) * 1e9)) for k in range(10)]
        receipts = {'/sim/scan': {s: s / 1e9 + 95. + .030 for s in stamps},
                    '/sim/scan_clipped': {s: s / 1e9 + 95. + .100 for s in stamps[:-1]},
                    '/scan': {s: s / 1e9 + 95. + .102 for s in stamps[:-1]}}
        out = EC.chain_latency(receipts, clock_rows, ['/sim/scan', '/sim/scan_clipped', '/scan'])
        stages = out['stages']
        self.assertAlmostEqual(stages[0]['latency_ms']['p50'], 30., places=2)
        self.assertAlmostEqual(stages[1]['latency_ms']['p50'], 70., places=2)
        self.assertAlmostEqual(stages[2]['latency_ms']['p50'], 2., places=2)
        self.assertEqual(stages[1]['missing_from_previous'], 1)
        self.assertEqual(stages[2]['missing_from_previous'], 0)
        self.assertAlmostEqual(out['total_ms']['p50'], 102., places=2)
        self.assertIsNone(EC.interpolate_wall(clock_rows, 1.))
        self.assertAlmostEqual(EC.interpolate_sim(clock_rows, 100.005), 5.005)

    def test_rates(self):
        rate = EC.stamp_rate([k * 100_000_000 for k in range(20)] + [2_200_000_000])
        self.assertEqual(rate['gaps'], 1)
        self.assertAlmostEqual(rate['median_step_s'], .1)
        self.assertAlmostEqual(EC.wall_rate([0., .1, .2, .3]), 10.)
        self.assertIsNone(EC.stamp_rate([1, 2])['sim_hz'])

    def test_scan_format_items(self):
        good = np.zeros(S2.BEAMS, dtype=np.float32)
        good[10] = 5.
        self.assertEqual(EC.scan_format_problems(S2.BEAMS, np.float32(S2.RAW_AMIN), np.float32(S2.RAW_INC), 'laser',
                                                 np.float32(.05), 30., good), [])
        bad = good.copy()
        bad[3] = np.inf
        bad[4] = -1.
        problems = EC.scan_format_problems(3200, 0., .01, 'base_link', .1, 40., bad)
        self.assertEqual(len(problems), 7)

    def test_yaw_convention_geometry(self):
        offset = 179.08
        # a pillar straight ahead of the LiDAR: its raw beam looks at body angle 0
        items = EC.pillar_expectations((0., 0., 0.), (.18, 0.), [(10., 0.)], .08, offset)
        j = items[0]['raw_beam']
        body = EC.wrap(S2.RAW_AMIN + j * S2.RAW_INC + math.radians(offset))
        self.assertLess(abs(body), S2.RAW_INC)
        self.assertAlmostEqual(items[0]['expected_m'], 10. - .18 - .08)
        # turned 90 deg left: the pillar is on the right (-90 deg body)
        items = EC.pillar_expectations((0., 0., math.pi / 2.), (.18, 0.), [(10., 0.)], .08, offset)
        self.assertAlmostEqual(items[0]['bearing_deg'], -90. - math.degrees(math.atan2(.18, 10.)), places=3)
        self.assertEqual(EC.pillar_expectations((0., 0., 0.), (0., 0.), [(50., 0.)], .08, offset), [])
        # the real format conversion puts a SIM return at that raw beam
        sim = [math.inf] * S2.SIM_BEAMS
        sim[S2.SIM_BEAMS // 2] = 9.74
        out = S2.RealFormat(offset).convert(sim)
        j0 = EC.raw_beam_of_body_bearing(0., offset)
        self.assertAlmostEqual(float(out[j0]), 9.74, places=3)

    def test_imu_noise_model(self):
        from steering.p6a_imu_model import ImuErrorModel
        model = ImuErrorModel(load_profile(), seed=3)
        expected = EC.imu_expected_noise(model.describe())
        g, a = [], []
        for _ in range(20000):
            gyro, accel, _ = model.apply((0., 0., 0.), (0., 0., 9.81))
            g.append(gyro[0])
            a.append(accel[0])
        self.assertTrue(EC.within(float(np.std(g)), expected['gyro_sd_rad_s'], .03))
        self.assertTrue(EC.within(float(np.std(a)), expected['accel_sd_m_s2'], .03))
        self.assertFalse(EC.within(None, 1., .1))

    def test_camera_message_parser_and_jpeg_size(self):
        jpeg = (b'\xff\xd8' + b'\xff\xe0' + struct.pack('!H', 4) + b'JF' +
                b'\xff\xc0' + struct.pack('!HBHH', 11, 8, 360, 640) + b'\x03\x01\x11\x00' + b'\xff\xd9')
        self.assertEqual(EC.jpeg_size(jpeg), (640, 360))
        import base64
        meta, data = EC.parse_camera_message(json.dumps(dict(type='frame', width=640,
                                                             data=base64.b64encode(jpeg).decode('ascii'))))
        self.assertEqual((meta['width'], data), (640, jpeg))
        raw = json.dumps(dict(type='frame', width=640)).encode('utf-8')
        binary = EC.CAMERA_BINARY_MAGIC + bytes([1]) + struct.pack('!I', len(raw)) + raw + jpeg
        meta, data = EC.parse_camera_message(binary)
        self.assertEqual((meta['width'], data), (640, jpeg))
        self.assertIsNone(EC.parse_camera_message(json.dumps(dict(type='status'))))
        self.assertIsNone(EC.jpeg_size(b'not a jpeg'))

    def test_ensure_idle_waits_for_the_executor_to_release_the_lease(self):
        # ST-05: an ended run keeps the lease until the executor's own stop (up to terminal_stop_wait_s); ensure_idle
        # returned at once because the bridge state of an armed host is 'armed_manual', not 'armed', and the next
        # script's Start was refused with bridge_still_armed
        clock = [0.]

        class Ctx:
            posts = []
            polls = 0

            class hold:
                lose = staticmethod(lambda: None)

            class op:
                @staticmethod
                def post(route, *args, **kwargs):
                    Ctx.posts.append((route, clock[0]))

            def exec_status(self):
                armed = clock[0] < 1.5                    # the executor stops the bridge 1.5 s after the end
                ended = any(route == '/api/end' for route, _ in Ctx.posts)
                return dict(runner=dict(state=EC.STATE_ENDED if ended else EC.STATE_PAUSED),
                            client=dict(state=EC.CLIENT_ARMED if armed else 'IDLE'))

            def wait_exec(self, predicate, timeout_s=5., period_s=.05):
                end = clock[0] + timeout_s
                while clock[0] < end:
                    Ctx.polls += 1
                    status = self.exec_status()
                    if predicate(status):
                        return status
                    clock[0] += period_s
                return None

            def host(self):
                armed = clock[0] < 1.5
                return dict(state='armed_manual' if armed else 'disarmed', armed=armed, moving=False,
                            steering_host=dict(stop_state='confirmed', stale=False))

            def wait_host(self, predicate, timeout_s):
                return predicate(self.host())

            def wait_still(self, timeout_s):
                return True

        ctx = Ctx()
        self.assertTrue(EC.ensure_idle(ctx))
        self.assertEqual([route for route, _ in Ctx.posts], ['/api/end'])
        self.assertGreaterEqual(clock[0], 1.5 - 1e-9)                        # it waited for the release
        self.assertLess(clock[0], EC.EXEC_RELEASE_WAIT_S)
        self.assertEqual(EC.HOST_PLAIN_STATES, ('disarmed', 'armed_manual'))
        # an armed host is not 'ready' and not a fault state either
        self.assertFalse(EC.bridge_ready_to_arm(dict(armed=True, moving=False, state='armed_manual',
                                                     steering_host=dict(stop_state='confirmed', stale=False))))
        self.assertIn('armed_manual', EC.HOST_PLAIN_STATES)

    def test_last_executor_drive_looks_back_beyond_the_mark_window(self):
        # ST-05 A17(G) 2nd run: input_expired_after_last_request_s was null twice; the mark window of ctx.logs() is
        # 0.05 s and the executor drives at send_hz 10 Hz, so its last drive before the injection was outside it
        mark_wall, injection_wall = 100.0, 100.002
        entries = [dict(kind='request', op='drive', owner='row-executor:gsim-1', t=50.0, wall=99.5),
                   dict(kind='request', op='drive', owner='row-executor:gsim-1', t=50.1, wall=99.9),
                   dict(kind='request', op='drive', owner='p6a-check', t=50.12, wall=99.92),
                   dict(kind='request', op='keepalive', owner='row-executor:gsim-1', t=50.13, wall=99.93),
                   dict(kind='request', op='drive', owner='row-executor:gsim-1', t=50.2, wall=100.01)]
        old_window = [e for e in entries if e['wall'] >= mark_wall - .05]
        self.assertIsNone(EC.last_executor_drive(mark_wall, injection_wall, entries=old_window))
        found = EC.last_executor_drive(mark_wall, injection_wall, entries=entries)
        self.assertEqual((found['t'], found['owner']), (50.1, 'row-executor:gsim-1'))
        self.assertIsNone(EC.last_executor_drive(mark_wall, injection_wall, lookback_s=.05, entries=entries))
        self.assertGreaterEqual(EC.DRIVE_LOOKBACK_S, 2. / 10.)            # two send periods of send_hz 10 Hz at least

    def test_stop_writes_split_request_writes_from_resends(self):
        # an op stop only sets the bridge's emergency event; the loop (20 ms) writes STOP at its next tick. gsim_final
        # w0_a17: requests at -14.0 and +21.7 ms, STOP lines at 0, 37.4, then the resends 117.9 .. 324.2 ms
        t0 = 10.0
        bridge = [dict(kind='request', op='stop', t=t0 - .014), dict(kind='reply', op='stop', t=t0 - .0139),
                  dict(kind='tx', head='STOP', t=t0),
                  dict(kind='request', op='stop', t=t0 + .0217), dict(kind='reply', op='stop', t=t0 + .0218)]
        for ms in (37.4, 117.9, 158.6, 217.7, 257.8, 324.2):
            bridge.append(dict(kind='tx', head='STOP', t=t0 + ms / 1000.))
        bridge.append(dict(kind='tx', head='C 0 0', t=t0 + .31))
        bridge.sort(key=lambda e: e['t'])
        by_request, resends, requests = EC.stop_writes(bridge, t0 - .014)
        self.assertEqual([round((e['t'] - t0) * 1000., 1) for e in by_request], [0.0, 37.4])
        self.assertEqual([round((e['t'] - t0) * 1000., 1) for e in resends], [117.9, 158.6, 217.7, 257.8, 324.2])
        self.assertEqual(len(requests), 2)
        # two requests inside one loop period make one STOP: the second request takes no resend 100 ms later
        coalesced = [dict(kind='request', op='stop', t=1.0), dict(kind='request', op='stop', t=1.005),
                     dict(kind='tx', head='STOP', t=1.015), dict(kind='tx', head='STOP', t=1.115)]
        by_request, resends, _ = EC.stop_writes(coalesced, 1.0)
        self.assertEqual((len(by_request), len(resends)), (1, 1))

    def test_outage_precondition_needs_a_driving_run(self):
        # gsim_final w0_a17: executor_loop_stall / executor_freeze were injected 1.9 s / 3.0 s after a stale_lidar pause
        # (drive already 0): nothing left to measure -> invalid step, not a failed one
        ok = EC.driving_precondition(dict(runner=dict(state=EC.STATE_ACTIVE, reason='script_drive')))
        self.assertTrue(ok['met'])
        paused = EC.driving_precondition(dict(runner=dict(state=EC.STATE_PAUSED, reason='stale_lidar')))
        self.assertFalse(paused['met'])
        self.assertIn('stale_lidar', paused['reason'])
        self.assertFalse(EC.driving_precondition(None)['met'])

    def test_output_guard_sequence_may_ramp_to_the_settle_steering(self):
        # gsim_final w0_a17 output_guard: C lines S 112, 94, 57, 5, 0 (settle ramp from the previous steering), then
        # the guard ended the run; the old check wanted S 0 only
        lines = [dict(c=dict(S=s)) for s in (112, 112, 94, 57, 57, 5, 0, 0)]
        seq = EC.s_sequence(lines)
        self.assertEqual(seq, [112, 94, 57, 5, 0])
        self.assertFalse(EC.moves_away_from(seq, 0))
        self.assertTrue(EC.moves_away_from([0, 41], 0))            # a raw step past the guard
        self.assertTrue(EC.moves_away_from([57, 5, -30], 0))       # through 0 to the other side
        self.assertTrue(EC.moves_away_from([-20, -60], 0))
        self.assertFalse(EC.moves_away_from([], 0))

    def test_scripts_registry(self):
        names = [s['name'] for s in EC.SCRIPTS]
        self.assertEqual(sorted(names), sorted(EC.DEFAULT_ORDER))
        self.assertEqual(EC.DEFAULT_ORDER[-1], 'executor_forced_kill')       # the executor is gone after it
        for item in EC.SCRIPTS:
            self.assertTrue(item['purpose'] and item['registered'])


if __name__ == '__main__':
    unittest.main()
