"""P6a headless integration: pure parts (no ROS, no Gazebo, no native program, no files in the work tree).

  * the ROS graph rules of A13 (p6a_graph) on synthetic graphs, the executor allow list (next phase)
  * bridge_steer.yaml against the gz_model topics and the supervisor's manifest check
  * compose.ubts-steer.yaml: project, domain, partition, port, volumes, memory inside the 6 GiB budget, healthcheck
  * the sensor contract helpers: static TF from the profile, the sensors_alive rule that the pinned health.py reads,
    the /scan note of this phase
  * the IMU error model (p6a_imu_model): per-run draws, noise, drive vibration rule of the profile
  * the emulator's pure helpers: banner rewrite with a new CRC, host line corruption, line assembly, PWM levels,
    the physics step, the A19 / A17 log helpers of p6a_checks
  * the 2026-11-09 review fixes: R6 judged as registered (infrastructure topics not exempt), the clip node in the
    graph rules, the laser TF yaw, the compose stop grace, the supervisor's run token / clip node / LiDAR noise
    contracts, the A19(G) false detections of normal scripts, the A17 resend-tick rule and analytic arc, the A25
    emulator path helpers, the a25 check on a fake stack (order: first /clock before the ARM; a run without
    steering motion fails) and the a19g_normal summary

unittest style (pytest on 3.12 / 3.13, python3 -m unittest on 3.10). Placeholder-based SIM results only.
"""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import statistics
import tempfile
import unittest

import yaml

from steering import geometry as geo
from steering import gz_esp_emulator as emu
from steering import gz_model
from steering import p6a_checks as checks
from steering import p6a_graph as graph
from steering import sensor_contract_steer as contract
from steering import supervisor_steer as sup
from steering.p6a_imu_model import ImuErrorModel
from steering.vehicle_profile import load_profile

SIM_ROOT = Path(__file__).resolve().parent
REPO = SIM_ROOT.parents[1]
COMPOSE = REPO / 'compose.ubts-steer.yaml'
HEALTH = SIM_ROOT / 'health.py'

_PROFILE = {}


def profile():
    if 'p' not in _PROFILE:
        _PROFILE['p'] = load_profile()
    return _PROFILE['p']


def manifest(steer_type=geo.LINKAGE, camera=True):
    return dict(model=dict(topics=dict(steer=gz_model.steer_topics(steer_type), drive=gz_model.wheel_topics(),
                                       truth_odometry_gz=gz_model.TRUTH_ODOM_TOPIC,
                                       truth_joint_states_gz=gz_model.TRUTH_JOINT_STATE_TOPIC),
                           steer_type=steer_type, drive_mode='force_dc'),
                renderer=dict(camera=camera, engine='ogre', headless=False))


def clean_graph():
    def t(pubs, subs):
        return dict(types=['x'], publishers=pubs, subscribers=subs)
    return dict(nodes=['ros_gz_bridge', 'ubts_cpu_lidar', 'ubts_steer_sensor_contract', 'ubts_steer_esp_emulator'],
                topics={'/ground_truth/odom': t(['ros_gz_bridge'], ['ubts_cpu_lidar', 'ubts_steer_esp_emulator',
                                                                    'ubts_steer_sensor_contract']),
                        '/sim/truth/steering': t(['ubts_steer_esp_emulator'], ['ubts_steer_sensor_contract']),
                        '/sim/truth/joint_states': t(['ros_gz_bridge'], []),
                        '/sim/scan': t(['ubts_cpu_lidar'], []),
                        '/tf_static': t(['ubts_steer_sensor_contract'], []),
                        '/sim/act/steer_left_cmd': t(['ubts_steer_esp_emulator'], ['ros_gz_bridge']),
                        '/imu/data_raw': t(['ubts_steer_sensor_contract'], [])})


class GraphRulesTest(unittest.TestCase):
    def test_clean_graph_passes(self):
        result = graph.check_truth_isolation(clean_graph())
        self.assertTrue(result['passed'], result['findings'])

    def test_each_rule_finds_its_violation(self):
        cases = [
            ('R1', '/odom', dict(publishers=['someone'], subscribers=[])),
            ('R1', '/tf', dict(publishers=['ubts_sim_adapter'], subscribers=[])),
            ('R1', '/joint_states', dict(publishers=['ros_gz_bridge'], subscribers=[])),
            ('R2', '/ground_truth/odom', dict(publishers=['ros_gz_bridge'], subscribers=['ubts_row_executor'])),
            ('R2', '/sim/truth/steering', dict(publishers=['ubts_steer_esp_emulator'], subscribers=['estimator'])),
            ('R3', '/sim/scan', dict(publishers=['ubts_cpu_lidar'], subscribers=['ubts_row_executor'])),
            ('R4', '/sim/act/left_front_wheel_cmd', dict(publishers=['gz_direct_drive'], subscribers=[])),
            ('R5', '/sim/cmd_vel', dict(publishers=[], subscribers=['ros_gz_bridge'])),
        ]
        for rule, topic, info in cases:
            with self.subTest(rule=rule, topic=topic):
                g = clean_graph()
                g['topics'][topic] = dict(types=['x'], **info)
                result = graph.check_truth_isolation(g)
                self.assertFalse(result['passed'])
                self.assertIn(rule, [f['rule'] for f in result['findings']])

    def test_executor_allow_list(self):
        g = clean_graph()
        self.assertIsNone(graph.check_executor_subscriptions(g)['passed'])
        g['nodes'].append('ubts_row_executor')
        for topic in graph.EXECUTOR_ALLOWED:
            g['topics'].setdefault(topic, dict(types=['x'], publishers=[], subscribers=[]))
            g['topics'][topic]['subscribers'] = g['topics'][topic]['subscribers'] + ['ubts_row_executor']
        result = graph.check_executor_subscriptions(g)
        self.assertTrue(result['passed'], result)
        # 2026-11-09 review RG-07: R6 is judged as registered; /clock (and the other infrastructure topics) are not
        # exempt without an amendment. They are listed apart with the verdict an amendment would give.
        for topic in ('/clock', '/parameter_events'):
            g['topics'].setdefault(topic, dict(types=['x'], publishers=[], subscribers=[]))
            g['topics'][topic]['subscribers'] = g['topics'][topic]['subscribers'] + ['ubts_row_executor']
        result = graph.check_executor_subscriptions(g)
        self.assertFalse(result['passed'])
        self.assertEqual(result['infrastructure'], ['/clock', '/parameter_events'])
        self.assertTrue(result['passed_if_infrastructure_allowed'])
        g['topics']['/ground_truth/odom']['subscribers'].append('ubts_row_executor')
        result = graph.check_executor_subscriptions(g)
        self.assertFalse(result['passed'])
        self.assertFalse(result['passed_if_infrastructure_allowed'])
        self.assertEqual(result['not_allowed_without_infrastructure'], ['/ground_truth/odom'])

    def test_clip_node_is_plant_side_and_the_clipped_scan_is_guarded(self):
        g = clean_graph()
        g['nodes'].append(graph.CLIP_NODE)
        g['topics']['/ground_truth/odom']['subscribers'].append(graph.CLIP_NODE)
        g['topics']['/sim/scan']['subscribers'].append(graph.CLIP_NODE)
        g['topics']['/sim/scan_clipped'] = dict(types=['x'], publishers=[graph.CLIP_NODE], subscribers=[])
        result = graph.check_truth_isolation(g)
        self.assertTrue(result['passed'], result['findings'])
        from steering import scan_unobserved_clip as clip
        self.assertEqual(clip.NODE_NAME, graph.CLIP_NODE)
        g['topics']['/sim/scan_clipped']['subscribers'].append('ubts_row_executor')
        result = graph.check_truth_isolation(g)
        self.assertIn(('R3', '/sim/scan_clipped'), [(f['rule'], f['topic']) for f in result['findings']])
        self.assertIn(graph.CLIP_NODE, checks.expected_nodes('field1010'))
        self.assertNotIn(graph.CLIP_NODE, checks.expected_nodes('w0_force'))


class BridgeConfigTest(unittest.TestCase):
    def test_template_matches_the_gz_model_topics(self):
        entries = sup.load_bridge_template()
        act = sorted(e['gz_topic_name'] for e in entries if e['gz_topic_name'].startswith('/sim/act/'))
        want = sorted(list(gz_model.steer_topics(geo.LINKAGE).values()) + list(gz_model.wheel_topics().values()))
        self.assertEqual(act, want)
        names = [e['ros_topic_name'] for e in entries]
        for forbidden in ('/sim/cmd_vel', '/sim/wheel_odom', '/joint_states', '/odom', '/tf', '/scan', '/sim/scan'):
            self.assertNotIn(forbidden, names)
        self.assertEqual(len(names), len(set(names)))

    def test_manifest_check(self):
        out = sup.bridge_config(manifest())
        self.assertIn('/camera/image_raw', [e['ros_topic_name'] for e in out])
        out = sup.bridge_config(manifest(camera=False))
        self.assertNotIn('/camera/image_raw', [e['ros_topic_name'] for e in out])
        axle = sup.bridge_config(manifest(geo.AXLE_PIVOT))
        act = sorted(e['gz_topic_name'] for e in axle if e['gz_topic_name'].startswith('/sim/act/'))
        self.assertIn('/sim/act/steer_axle_cmd', act)
        self.assertNotIn('/sim/act/steer_left_cmd', act)
        broken = manifest()
        broken['model']['topics']['drive'] = dict(broken['model']['topics']['drive'], extra='/sim/act/extra_cmd')
        with self.assertRaises(sup.SupervisorError):
            sup.bridge_config(broken)
        wrong = [dict(e) for e in sup.load_bridge_template()]
        for e in wrong:
            if e['gz_topic_name'] == '/sim/act/steer_left_cmd':
                e['direction'] = 'GZ_TO_ROS'
        with self.assertRaises(sup.SupervisorError):
            sup.bridge_config(manifest(), template=wrong)


class ComposeTest(unittest.TestCase):
    def setUp(self):
        if not COMPOSE.is_file():
            raise unittest.SkipTest('compose.ubts-steer.yaml not next to the work tree (an image without it)')
        self.data = yaml.safe_load(COMPOSE.read_text(encoding='utf-8'))

    @staticmethod
    def mem_gib(text):
        text = str(text)
        if text.endswith('g'):
            return float(text[:-1])
        if text.endswith('m'):
            return float(text[:-1]) / 1024.
        raise ValueError(text)

    def test_identity_and_budget(self):
        d = self.data
        self.assertEqual(d['name'], 'ubts-steer')
        env = d['x-environment']
        self.assertEqual(env['ROS_DOMAIN_ID'], '43')
        self.assertEqual(env['GZ_PARTITION'], 'ubts-steer')
        self.assertEqual(env['ROVER_CONTROL_SOCKET'], '/run/ubts-steer/control.sock')
        services = d['services']
        self.assertEqual(services['dashboard']['ports'], ['127.0.0.1:18090:8080'])
        for name in ('steer', 'recorder', 'check'):
            self.assertEqual(services[name]['network_mode'], 'service:sim')
            self.assertEqual(services[name]['ipc'], 'service:sim')
        self.assertEqual(services['check']['profiles'], ['check'])
        stack = sum(self.mem_gib(services[n]['mem_limit']) for n in ('sim', 'dashboard', 'steer', 'recorder'))
        self.assertAlmostEqual(stack, 5.5)
        self.assertLessEqual(stack + self.mem_gib(services['check']['mem_limit']), 6.0)
        self.assertEqual(self.mem_gib(services['sim']['mem_limit']), 3.5)
        self.assertEqual(sorted(d['volumes']), ['cache', 'control', 'hooks', 'runtime'])
        self.assertIn('/opt/ubts/sim/ubts/health.py', services['sim']['healthcheck']['test'])
        sim_env = services['sim']['environment']
        self.assertEqual(sim_env['UBTS_LIDAR_BACKEND'], 'embree')
        self.assertIn('d3d12', sim_env['GALLIUM_DRIVER'])
        self.assertIn('/dev/dxg:/dev/dxg', services['sim']['devices'])
        for service in services.values():
            self.assertEqual(service.get('restart'), 'no')
        self.assertNotIn('hooks:/run/ubts-steer-emu', services['dashboard']['volumes'])
        # 2026-11-09 review RG-10: the sim supervisor needs about 6.5 s to stop its children; the default grace of
        # the compose stop SIGKILLed it (exit 137, the READY stamp left behind)
        for name, service in services.items():
            self.assertEqual(service.get('stop_grace_period'), '20s', name)
        self.assertIn('UBTS_STEER_FIELD_SPEC', d['x-environment'])


class SensorContractTest(unittest.TestCase):
    def rover(self):
        return yaml.safe_load((SIM_ROOT / 'rover.yaml').read_text(encoding='utf-8'))

    def test_static_transforms(self):
        rows = {child: (parent, xyz, q) for parent, child, xyz, q in
                contract.static_transforms(profile(), self.rover(), .7)}
        self.assertEqual(set(rows), set(contract.STATIC_FRAMES))
        z = self.rover()['vehicle']['body_center_height_m']
        v = profile().value
        self.assertEqual(rows['laser'][0], 'base_link')
        self.assertAlmostEqual(rows['laser'][1][2] + z, v('lidar_z_m'))
        # 2026-11-09 review RG-12: the laser frame is the S2 /scan frame, turned by lidar_yaw_offset_deg about z: a
        # return straight ahead of the body is raw beam j with RAW_AMIN + j RAW_INC + offset = 0
        q = rows['laser'][2]
        yaw = math.degrees(2. * math.atan2(q[2], q[3]))
        self.assertAlmostEqual(yaw % 360., float(v('lidar_yaw_offset_deg')) % 360., places=9)
        try:
            from steering import s2_format          # WF-3a module (absent in a HEAD-based snapshot)
        except ImportError:
            return
        fmt = s2_format.RealFormat(float(v('lidar_yaw_offset_deg')))
        scan = [math.inf] * s2_format.SIM_BEAMS
        scan[s2_format.SIM_BEAMS // 2] = 5.                     # SIM beam at body angle 0 (-pi + k 2 pi / 3200)
        raw = fmt.convert(scan)
        j = [i for i, r in enumerate(raw) if r > 0.][0]
        a = s2_format.RAW_AMIN + j * s2_format.RAW_INC          # the scan-frame angle of that return
        x = 5. * math.cos(a + math.radians(yaw))                # the point in base_link through the static TF
        self.assertGreater(x, 4.99)
        self.assertAlmostEqual(rows['imu_link'][1][2] + z, v('imu_pose_in_base')['z'])
        for parent, xyz, q in rows.values():
            self.assertAlmostEqual(sum(c * c for c in q), 1., places=12)
        flipped = contract.quaternion_rpy(math.pi, 0., 0.)
        self.assertAlmostEqual(abs(flipped[0]), 1., places=12)

    def test_alive_rule_and_health(self):
        camera = {k: dict(alive=True, count=3) for k in ('camera', 'camera_info')}
        lidar = dict(state='running', updated_at=100., last_stamp_ns=1)
        last = dict(imu=9.5, gps=9.0, steering=9.9)
        clock_alive, alive = contract.sensors_alive(10., 100.5, last, 9.9, camera, lidar, False)
        self.assertTrue(clock_alive)
        self.assertEqual(sorted(alive), ['camera', 'camera_info', 'gps', 'imu', 'sim_scan', 'steering'])
        self.assertTrue(all(alive.values()))
        # the pinned health.py rule
        status = dict(updated_at=100.5, sensors_alive=alive)
        self.assertTrue(all(status['sensors_alive'].values()))
        self.assertIn("all(state['sensors_alive'].values())", HEALTH.read_text(encoding='utf-8'))
        _, alive = contract.sensors_alive(10., 100.5, dict(last, steering=7.), 9.9, camera, lidar, False)
        self.assertFalse(alive['steering'])
        _, alive = contract.sensors_alive(10., 100.5, last, 9.9, camera, dict(lidar, state='waiting_for_pose'), False)
        self.assertFalse(alive['sim_scan'])
        _, alive = contract.sensors_alive(10., 100.5, last, 9.9, camera, lidar, True, camera_present=False)
        self.assertIn('scan', alive)
        self.assertNotIn('camera', alive)
        clock_alive, alive = contract.sensors_alive(10., 100.5, last, 8.5, camera, lidar, False)
        self.assertFalse(clock_alive)
        self.assertFalse(any(alive.values()))
        self.assertIn('absent', contract.scan_contract(False, False)['/scan'])
        self.assertEqual(contract.rtf_estimate([(0., 0.), (2., 1.)]), .5)


class SupervisorContractTest(unittest.TestCase):
    """2026-11-09 reviews RW-01 (clip node CLI, field1010), RW-04 (LiDAR noise after the clip), RG-10 (run token)."""

    def test_clip_node_command_matches_the_node_parser(self):
        from steering import scan_unobserved_clip as clip
        command = sup.clip_node_command('steering.scan_unobserved_clip', Path('/runtime/steer/scene'),
                                        Path('/runtime'), .01, 3)
        self.assertEqual(command[1:4], ['-B', '-m', 'steering.scan_unobserved_clip'])
        args = clip.build_parser().parse_args(command[4:])
        self.assertEqual((args.input, args.output), ('/sim/scan', '/sim/scan_clipped'))
        self.assertEqual(Path(args.raster).as_posix(), '/runtime/steer/scene/field_world/raster.npz')
        self.assertEqual((args.noise_sd, args.noise_seed), (.01, 3))
        self.assertEqual(Path(args.status_file).as_posix(), '/runtime/scan_clip_status.json')
        # the node refuses the old hook form without the raster
        with self.assertRaises(SystemExit):
            import contextlib
            import io
            with contextlib.redirect_stderr(io.StringIO()):
                clip.build_parser().parse_args(['--input', '/sim/scan', '--output', '/sim/scan_clipped'])

    def test_clip_node_only_for_field_scenes_and_scene_list(self):
        self.assertIn('field1010', sup.SCENES)
        args = sup.build_parser().parse_args(['--role', 'sim', '--scene', 'field1010'])
        self.assertEqual(sup.clip_node_of(args, dict(scene='field1010')), sup.DEFAULT_CLIP_NODE)
        self.assertIsNone(sup.clip_node_of(args, dict(scene='w0_force')))
        args = sup.build_parser().parse_args(['--role', 'sim', '--clip-node', 'x.y'])
        with self.assertRaises(sup.SupervisorError):
            sup.clip_node_of(args, dict(scene='w0_force'))
        self.assertEqual(sup.field_spec_value('{"name": "a"}'), {'name': 'a'})
        self.assertIsNone(sup.field_spec_value(''))

    def test_lidar_config_without_noise(self):
        with tempfile.TemporaryDirectory() as tmp:
            target, noise = sup.lidar_config_without_noise(SIM_ROOT / 'rover.yaml', Path(tmp) / 'r.yaml')
            data = yaml.safe_load(target.read_text(encoding='utf-8'))
            original = yaml.safe_load((SIM_ROOT / 'rover.yaml').read_text(encoding='utf-8'))
            self.assertEqual(data['lidar']['noise_stddev_m'], 0.)
            self.assertEqual(noise, original['lidar']['noise_stddev_m'])
            data['lidar']['noise_stddev_m'] = original['lidar']['noise_stddev_m']
            self.assertEqual(data, original)

    def test_run_token_rule(self):
        stamp = dict(token='abc', sdf_sha256='s')
        status = dict(sim_run='abc', updated_at=100., clock_alive=True)
        self.assertTrue(sup.sim_role_live(stamp, status, 101.))
        self.assertFalse(sup.sim_role_live(stamp, dict(status, sim_run='old'), 101.))       # a stamp of an earlier run
        self.assertFalse(sup.sim_role_live(stamp, status, 110.))                            # stale status
        self.assertFalse(sup.sim_role_live(stamp, dict(status, clock_alive=False), 101.))
        self.assertFalse(sup.sim_role_live(None, status, 101.))
        ready = dict(run_token='abc', all_ready=True)
        self.assertTrue(sup.sim_role_live(stamp, status, 101., supervisor=ready))
        self.assertFalse(sup.sim_role_live(stamp, status, 101., supervisor=dict(ready, all_ready=False)))
        self.assertFalse(sup.sim_role_live(stamp, status, 101., supervisor=dict(ready, run_token='old')))
        self.assertFalse(sup.sim_role_live(stamp, status, 101., supervisor={}))
        with tempfile.TemporaryDirectory() as tmp:
            self.assertIsNone(sup.read_scene_stamp(tmp))
            (Path(tmp) / 'READY').write_bytes(b'ready\n')                                   # the old plain stamp
            self.assertIsNone(sup.read_scene_stamp(tmp))
            sup.write_scene_stamp(tmp, 'tok', dict(scene='w0_force', sdf_sha256='x'))
            read = sup.read_scene_stamp(tmp)
            self.assertEqual((read['token'], read['scene'], read['sdf_sha256']), ('tok', 'w0_force', 'x'))
            self.assertNotIn(b'\r', (Path(tmp) / 'READY').read_bytes())


class ImuModelTest(unittest.TestCase):
    def test_draws_noise_and_vibration(self):
        a, b = ImuErrorModel(profile(), seed=3), ImuErrorModel(profile(), seed=3)
        self.assertEqual(a.describe()['gyro_scale'], b.describe()['gyro_scale'])
        self.assertNotEqual(ImuErrorModel(profile(), seed=4).gyro_scale, a.gyro_scale)
        self.assertAlmostEqual(a.drive_vibration(50.), profile().value('imu_vibration_drive_below_90_m_s2'))
        self.assertAlmostEqual(a.drive_vibration(100.), profile().value('imu_vibration_drive_at_100_m_s2'))
        still = a.still_vibration
        low, high = profile().value('imu_vibration_still_m_s2')['low'], profile().value('imu_vibration_still_m_s2')['high']
        self.assertTrue(low <= still <= high)
        self.assertAlmostEqual(a.vibration_rms(40., 0., .3), a.drive_below_90)
        self.assertAlmostEqual(a.vibration_rms(40., 0., 0.), still)       # standing: still vibration
        samples = [a.apply((0., 0., 0.), (0., 0., 9.81), 0., 0., 0.)[0][2] for _ in range(20000)]
        expected = math.radians(math.hypot(a.still_noise_sd_deg_s, a.vibration_coupling * still))
        self.assertAlmostEqual(statistics.pstdev(samples) / expected, 1., delta=.05)
        self.assertAlmostEqual(statistics.mean(samples), a.gyro_bias, delta=4 * expected / math.sqrt(20000))
        gyro, accel, vib = a.apply((0., 0., .2), (0., 0., 9.81), 0., 0., 0.)
        self.assertEqual(len(gyro), 3)
        self.assertEqual(len(accel), 3)


class EmulatorHelpersTest(unittest.TestCase):
    BANNER = ('UBTS_STEER_V3 hello proto=3 build=04353B49 dev=5A1B2C boot=89025CC1 rst=power_on hw=front_steer_v0 '
              'out=drive2,steer1 cal=7 sim=1 wd=500 ttl=300 st=20 drop=0')

    def test_banner_rewrite(self):
        line = emu.recrc_line(self.BANNER)
        out = emu.rewrite_banner(line, {'sim': '0', 'hw': 'front_steer_x9'})
        text = emu.line_text(out)
        body, crc = text.rsplit('*', 1)
        self.assertIn(' sim=0 ', body + ' ')
        self.assertIn(' hw=front_steer_x9 ', body)
        self.assertEqual(crc, '%02X' % emu.crc8(body.encode('ascii')))
        status = emu.recrc_line('S 1 0 1 0 0 0 1 512 0 0 0 0 4000 0 0')
        self.assertEqual(emu.rewrite_banner(status, {'sim': '0'}), status)

    def test_host_line_corruption(self):
        c = emu.recrc_line('C 5 300 102 102 -150')
        bad, applied = emu.corrupt_host_line(c, 'range')
        self.assertTrue(applied)
        body, crc = emu.line_text(bad).rsplit('*', 1)
        self.assertEqual(body.split(' ')[5], '2000')
        self.assertEqual(crc, '%02X' % emu.crc8(body.encode('ascii')))
        bad, applied = emu.corrupt_host_line(c, 'crc')
        self.assertTrue(applied)
        body, crc = emu.line_text(bad).rsplit('*', 1)
        self.assertNotEqual(crc, '%02X' % emu.crc8(body.encode('ascii')))
        self.assertEqual(emu.corrupt_host_line(b'STOP\n', 'crc'), (b'STOP\n', False))
        self.assertEqual(emu.corrupt_host_line(b'PING\n', 'range'), (b'PING\n', False))
        bad, applied = emu.corrupt_host_line(b'PING\n', 'crc')
        self.assertTrue(applied and b'*' in bad)

    def test_line_assembly_and_levels(self):
        from steering.native.lockstep_v2 import FwTxByte
        data = b'S 1\nS 2\nUB'
        tx = [FwTxByte(10 + i // 4, 10 + i, b, False) for i, b in enumerate(data)]
        assembler = emu.LineAssembler()
        lines = assembler.feed(tx)
        self.assertEqual([emu.line_text(l[0]) for l in lines], ['S 1', 'S 2'])
        self.assertEqual(lines[0][1:3], (10, 13))
        self.assertEqual(assembler.buf, bytearray(b'UB'))
        self.assertEqual([emu.pwm_level(v, (153, 204)) for v in (0, 5, -152, 153, -203, 204, 255)],
                         [0, 1, 1, 2, 2, 3, 3])
        self.assertEqual(emu.physics_step_ns(120), 8333333)
        self.assertEqual(emu.stable_u32(5, 201), emu.stable_u32(5, 201))
        self.assertNotEqual(emu.stable_u32(5, 201), emu.stable_u32(5, 202))
        self.assertEqual(emu.fault_bit_names(0x4001), ['NOCAL', 'BOOT'])

    def test_memory_port(self):
        port = emu.MemoryPort(host_open=False)
        self.assertFalse(port.write(b'x'))
        port.host_open = True
        self.assertTrue(port.write(b'ab'))
        self.assertEqual(port.host_read(), b'ab')
        port.host_write(b'PING\n')
        self.assertEqual(port.read(), b'PING\n')


class CheckHelpersTest(unittest.TestCase):
    def test_line_parsers(self):
        s = checks.parse_status_line('S 2520 48 2 4 0 0 3 513 10 0 0 0 0080 18 282*41')
        self.assertEqual((s['ms'], s['n'], s['st'], s['fl'], s['tl']), (2520, 48, 2, 0x80, 282))
        self.assertIsNone(checks.parse_status_line('N - crc*D8'))
        self.assertEqual(checks.parse_c_line('C 3 300 102 102 -150*AA'), dict(q=3, ttl=300, L=102, R=102, S=-150))

    def test_drive_zero_and_firsts(self):
        log = [dict(kind='pin', channel='drive_l', value=40, tick=100), dict(kind='pin', channel='drive_r', value=40,
                                                                             tick=100),
               dict(kind='pin', channel='drive_l', value=0, tick=205), dict(kind='pin', channel='drive_r', value=0,
                                                                            tick=206),
               dict(kind='m2h', tick=206, text='F STALL 0 852*AA')]
        self.assertEqual(checks.drive_zero_tick(log, 200), 206)
        self.assertEqual(checks.f_line(log, 'STALL', 0)['tick'], 206)
        self.assertIsNone(checks.f_line(log, 'TTL', 0))

    def test_rtf_and_lateral(self):
        rows = [(w * .1, w * .1 * .8) for w in range(400)]
        windows = checks.rtf_windows(rows, warmup_s=5., window_s=5.)
        self.assertTrue(windows and all(abs(w - .8) < 1e-9 for w in windows))
        along, lateral = checks.lateral_in_start_frame([(0., 1., 2., math.pi / 2), (1., 1., 3., 0.)])
        self.assertAlmostEqual(along, 1.)
        self.assertAlmostEqual(lateral, 0.)
        self.assertIsNone(checks.verdict([checks.criterion('a', 1, 1, None)]))
        self.assertFalse(checks.verdict([checks.criterion('a', 1, 1, True), checks.criterion('b', 1, 1, False)]))

    def test_bridge_stop_latency(self):
        log = [dict(kind='tx', t=1.000, wall=10., head='PING\n'), dict(kind='request', op='stop', t=1.003, wall=10.01),
               dict(kind='tx', t=1.018, wall=10.02, head='STOP\n'),
               dict(kind='state', t=1.070, wall=10.07, stop_state='confirmed', latches=[])]
        req, tx, confirm = checks.bridge_stop_latency(log, 10.)
        self.assertAlmostEqual(tx['t'] - req['t'], .015)
        self.assertAlmostEqual(confirm['t'] - req['t'], .067)

    def test_a19_timing_rules(self):
        def pin(tick, channel, value, **extra):
            return dict(kind='pin', tick=tick, channel=channel, value=value, **extra)
        # STALL: the |pwm| >= stp run that is on when the code latches (fault_bits), + stt
        log = [pin(100, 'steer_pwm_level', 1), pin(150, 'steer_pwm_level', 2), pin(160, 'steer_pwm_level', 3),
               pin(650, 'fault_bits', 256, faults=['STALL']), pin(650, 'steer_pwm_level', 0)]
        self.assertEqual(checks.latch_tick(log, 'STALL', 0), 650)
        self.assertEqual(checks.run_start(log, 50, 2, 500, 'rule', 'STALL')[0], 650)
        self.assertIsNone(checks.run_start(log, 50, 2, 500, 'rule', 'TRACK')[0])     # no latch, no expected tick
        # THERMAL: hot ticks summed over the runs at level 3 (in and out); the tick after the (tha + 1)-th
        log = [pin(10, 'steer_pwm_level', 3), pin(30, 'steer_pwm_level', 2), pin(40, 'steer_pwm_level', 3),
               pin(100, 'fault_bits', 1024, faults=['THERMAL']), pin(100, 'steer_pwm_level', 0)]
        self.assertEqual(checks.thermal_expected(log, 0, 50, 1)[0], 71)             # 20 + 31 hot ticks: 40 + 30 + 1
        # TRACK: cmd ramps from cmd0 0 past P 10 + trk 120 at rstd 478 permille / s from the C arrival
        log = [dict(kind='m2h', tick=900, text='S 900 1 2 4 0 0 1 513 10 0 0 0 0000 0 0*00'),
               dict(kind='h2m', tick=995, first=996, last=1000, text='C 5 300 0 0 350*00'),
               dict(kind='m2h', tick=1020, text='S 1020 2 2 5 0 0 1 513 10 350 9 0 0000 0 0*00')]
        tick, _, rule = checks.track_expected(log, 990, dict(trk=120, rstd=478, trt=1000))
        self.assertEqual(tick, 1000 + 275 + 999)                                      # ceil(1000 x 131 / 478) = 275
        self.assertIn('cmd0 0, P 10', rule)

    def test_false_detections_of_normal_scripts(self):
        # 2026-11-09 review RG-01 (R15-3): unexpected F / W lines, new fault bits (BOOT excepted) and host latches
        def status(tick, fl):
            return dict(kind='m2h', tick=tick, text='S %d 1 1 0 0 0 1 512 0 0 0 0 %04X 0 0*00' % (tick, fl))
        emu = [status(100, 0x4000), status(150, 0x4000),
               dict(kind='m2h', tick=160, text='F THERMAL 5001 160*73'), status(200, 0x4400),
               dict(kind='m2h', tick=300, text='F TTL 1 300*00'), status(350, 0x4C00),
               dict(kind='m2h', tick=400, text='W 500 400*00')]
        bridge = [dict(kind='state', wall=1., t=1., latches=[]),
                  dict(kind='state', wall=2., t=2., latches=['steering_status_stale'], reason='startup')]
        out = checks.false_detections(emu, bridge, expected=(('mcu', 'TTL', 250, None),))
        self.assertEqual(out['by_code'], {'THERMAL': 1, 'WATCHDOG': 1, 'host:steering_status_stale': 1})
        self.assertEqual(out['count'], 3)
        self.assertEqual([e.get('code') or e.get('bit') for e in out['expected_seen']], ['TTL', 'TTL'])
        none = checks.false_detections(emu[:2], bridge[:1])
        self.assertEqual(none['count'], 0)
        self.assertEqual(checks.fault_code_of('F POT_NOISE 2557 3067*1A'), 'POT_NOISE')
        self.assertEqual(checks.fault_code_of('W 501 900*00'), 'WATCHDOG')
        self.assertIsNone(checks.fault_code_of('S 1 2 3'))
        # a latch present before the window is not new
        late = checks.false_detections([], bridge, after_wall=1.5)
        self.assertEqual(late['count'], 1)
        late = checks.false_detections([], bridge + [dict(kind='state', wall=3., latches=['steering_status_stale'])],
                                       after_wall=2.5)
        self.assertEqual(late['count'], 0)
        # a stale latch inside an emulator /clock wall stall is the SIM's own stall in the wall clock mode (A10)
        stalled = checks.false_detections([], bridge, stalls=[(2.3, .4)], clock='wall')
        self.assertEqual((stalled['count'], len(stalled['sim_stall_latches'])), (0, 1))
        self.assertEqual(stalled['sim_stall_latches'][0]['stall_ms'], 400.)
        far = checks.false_detections([], bridge, stalls=[(9.0, .4)], clock='wall')
        self.assertEqual(far['count'], 1)
        sim_clock = checks.false_detections([], bridge, stalls=[(2.3, .4)], clock='sim')
        self.assertEqual(sim_clock['count'], 1)                       # SIM clock mode: no wall attribution
        # ... and so is one inside a slow /clock interval (SIM time below 0.9 x real time; bridge t is monotonic)
        slow = checks.false_detections([], bridge, slow=[(1.7, 2.05, .3)], clock='wall')
        self.assertEqual((slow['count'], slow['sim_stall_latches'][0]['slow_min_rate']), (0, .3))
        still_open = checks.false_detections([], bridge, slow=[(1.9, None, .5)], clock='wall')
        self.assertEqual(still_open['count'], 0)
        before = checks.false_detections([], bridge, slow=[(.2, .6, .3)], clock='wall')
        self.assertEqual(before['count'], 1)                          # ended more than 1 s before the latch
        self.assertEqual(checks.false_detections([], bridge, slow=[(1.7, 2.05, .3)], clock='sim')['count'], 1)

    def test_emulator_slow_clock_intervals(self):
        # the emulator keeps the windows where the SIM time it receives runs below 0.9 x real time (monotonic)
        class Core:
            clock_slow = collections.deque(maxlen=emu.SLOW_KEPT)
            clock_slow_open = None

        class Fake:
            recent_clock = collections.deque(maxlen=512)
            core = Core()
        fake = Fake()
        now, sim = 100., 10.
        for _ in range(120):                          # 1 s at real time
            now, sim = now + 1. / 120., sim + 1. / 120.
            emu.RosEmulator.track_rate(fake, now, sim)
        self.assertIsNone(fake.core.clock_slow_open)
        now += .34                                    # a 340 ms stall, then the queued messages in a burst
        emu.RosEmulator.track_rate(fake, now, sim + 1. / 120.)
        self.assertIsNotNone(fake.core.clock_slow_open)            # the first message after the stall: slow
        for k in range(1, 40):                        # the burst catches up: the interval closes
            emu.RosEmulator.track_rate(fake, now + k * 1e-4, sim + (k + 1) / 120.)
        self.assertIsNone(fake.core.clock_slow_open)
        self.assertEqual(len(fake.core.clock_slow), 1)
        start, end, rate, _ = fake.core.clock_slow[0]
        self.assertLess(rate, .1)
        self.assertLess(start, 101.01)
        self.assertGreater(end, 101.34)
        now, sim = now + 40e-4, sim + 40. / 120.
        for _ in range(240):                          # real time again: nothing new
            now, sim = now + 1. / 120., sim + 1. / 120.
            emu.RosEmulator.track_rate(fake, now, sim)
        self.assertEqual((len(fake.core.clock_slow), fake.core.clock_slow_open), (1, None))
        for _ in range(240):                          # Gazebo at half speed: one open slow interval
            now, sim = now + 1. / 120., sim + .5 / 120.
            emu.RosEmulator.track_rate(fake, now, sim)
        self.assertAlmostEqual(fake.core.clock_slow_open[1], .5, places=6)

    def test_merged_false_detections_of_the_stop_windows(self):
        a = dict(count=1, by_code={'host:steering_status_stale': 1}, sim_stall_latches=[])
        b = dict(count=2, by_code={'THERMAL': 1, 'host:steering_status_stale': 1}, sim_stall_latches=[dict(wall=1.)])
        merged = checks.merge_false_detections([a, b, dict(count=0, by_code={})])
        self.assertEqual(merged['by_code'], {'host:steering_status_stale': 2, 'THERMAL': 1})
        self.assertEqual((merged['count'], len(merged['sim_stall_latches'])), (3, 1))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'safety.json'
            path.write_text(json.dumps(dict(check='A17_G', false_detection=dict(count=3, by_stop=[a, b]),
                                            emulator_clock=dict(sim_s=10.))), encoding='utf-8')
            summary = checks.summarize_normal([path])
            self.assertEqual(summary['by_code'], {'host:steering_status_stale': 2, 'THERMAL': 1})
            self.assertEqual((summary['total'], summary['passed']), (3, False))

    def test_resend_at_the_first_loop_tick_after_the_grid(self):
        # 2026-11-09 review RG-05 (amendment 7): exact first-tick rule from the bridge's tick log
        ticks = [round(.004 + .02 * k, 6) for k in range(30)]           # loop ticks 4, 24, 44, ... ms
        ok, before = checks.first_tick_after(ticks, .100, .104, 1e-6)
        self.assertTrue(ok)
        self.assertAlmostEqual(before, .084)
        late, _ = checks.first_tick_after(ticks, .100, .124, 1e-6)     # a tick (104 ms) was skipped
        self.assertFalse(late)
        early, _ = checks.first_tick_after(ticks, .100, .084, 1e-6)    # before its grid time
        self.assertFalse(early)

    def test_analytic_arc_of_the_link_loss(self):
        # 2026-11-09 review RG-06: the arc length from the speed, the rest of the TTL and the stop mode
        from steering import drive
        stop = drive.stop_modes(profile())['brake']
        s, mode = checks.analytic_stop_distance(profile(), .4, .29, 'brake')
        self.assertAlmostEqual(s, .4 * .29 + stop.distance_m(.4))
        self.assertAlmostEqual(s, .4 * .29 + .4 * stop.onset_s + .4 * .4 / (2. * stop.decel_m_s2()))
        kappa = math.tan(math.radians(5.)) / 0.75
        self.assertAlmostEqual(checks.arc_lateral(kappa, .2), (1. - math.cos(kappa * .2)) / kappa)
        self.assertEqual(checks.arc_lateral(0., 1.), 0.)

    def test_a25_helpers_and_counters(self):
        step = 1. / 120.
        truth = [dict(sim_time_s=round(10. + k * step, 9), delta_left_rad=.001 * k, delta_right_rad=-.001 * k)
                 for k in range(50)]
        names = ('left_front_steering_joint', 'right_front_steering_joint')
        joints = [(10. + k * step, {names[0]: (.001 * (k - 1), 0.), names[1]: (-.001 * (k - 1), 0.)})
                  for k in range(1, 50)]
        keys = {'left': (names[0], 'delta_left_rad'), 'right': (names[1], 'delta_right_rad')}
        errors, matched = checks.joint_errors(joints, truth, keys, lag_steps=0)
        self.assertEqual(matched, 49)
        self.assertAlmostEqual(max(errors['left']), math.degrees(.001), places=9)
        errors, _ = checks.joint_errors(joints, truth, keys, lag_steps=1)       # the joint is one step behind
        self.assertAlmostEqual(max(errors['left']), 0., places=9)
        per_lag, best, top = checks.a25_summary(joints, truth, keys)
        self.assertEqual(best, 1)
        self.assertEqual(sorted(per_lag), [0, 1, 2, 3])
        self.assertEqual((per_lag[1]['matched'], per_lag[1]['per_joint']['right']['n']), (49, 49))
        self.assertAlmostEqual(per_lag[0]['p99_deg'], math.degrees(.001), places=9)
        self.assertEqual(top, 0.)
        empty = checks.a25_summary([], truth, keys)                           # no joint samples: nothing judged
        self.assertEqual((empty[1], empty[2], empty[0][0]['p99_deg']), (None, None, None))
        self.assertEqual(checks.percentile([1., 2., 3., 4.], .5), 2.5)
        before = dict({k: 0 for k in checks.EMULATOR_CLOCK_COUNTERS}, clock_messages=100, last_clock_s=10.,
                      clock_wall_gap_max_us=0, mono=0.)
        after = dict(before, clock_gaps_over_20ms=3, clock_messages=200, last_clock_s=10. + 120 * step, mono=1.)
        delta = checks.counter_delta(before, after)
        self.assertEqual((delta['clock_gaps_over_20ms'], delta['clock_messages'], delta['clock_messages_missing']),
                         (3, 100, 20))
        rows = [(w * .1, w * .1 * .5) for w in range(100)]
        self.assertAlmostEqual(checks.rtf_between(rows, 2., 8.), .5)
        self.assertIsNone(checks.rtf_between(rows, 2., 2.2))
        summary = checks.rtf_summary(rows)
        self.assertEqual(summary['below_valid_min'], summary['windows'])

    def test_a25_check_on_a_fake_stack(self):
        events = []
        box = {}
        names = (gz_model.STEER_JOINTS['left'], gz_model.STEER_JOINTS['right'])

        class FakeObs:
            step = 1. / 120.

            def __init__(self):
                self.sim_time, self.joints, self.truth, self.clock_rows, self.delta = None, [], [], [], 0.

            def enable_joints(self):
                events.append('joints')

            def spin_for(self, seconds):
                pass

            def spin_sim_for(self, seconds, timeout_s=None):
                if self.sim_time is None:
                    events.append('first_clock')
                    self.sim_time = 10.
                end = self.sim_time + seconds
                while self.sim_time < end - 1e-9:
                    self.sim_time = round(self.sim_time + self.step, 9)
                    loop = box.get('loop')
                    target = loop.command[1] / 1000. if loop is not None and loop.moving else 0.
                    change = max(-.5 * self.step, min(.5 * self.step, target - self.delta))   # 0.5 rad/s
                    self.delta += change
                    self.truth.append(dict(sim_time_s=self.sim_time, delta_left_rad=self.delta,
                                           delta_right_rad=self.delta, delta_eq_rad=self.delta))
                    self.joints.append((self.sim_time, {names[0]: (self.delta, change / self.step),
                                                        names[1]: (self.delta, change / self.step)}))
                    self.clock_rows.append((self.sim_time, self.sim_time))
                return self.sim_time

        class FakeLoop:
            refuse = False

            def __init__(self, client, throttle=0, steer_mrad=0, period_s=.1):
                self.command = (throttle, steer_mrad)
                self.errors = collections.Counter()
                self.moving = False
                box['loop'] = self

            def start(self):
                events.append('loop_start')
                # the bridge refuses drives when the first input comes too late after the ARM (stale_input)
                self.moving = not FakeLoop.refuse
                if FakeLoop.refuse:
                    self.errors['stale_input_rearm_required'] += 1
                return self

            def set(self, throttle, steer_mrad):
                self.command = (throttle, steer_mrad)

            def stop(self):
                self.moving = False

        class FakeHooks:
            def __init__(self, obs):
                self.obs = obs

            def status(self):
                return {'status': {'counters': {}, 'clock_messages': len(self.obs.clock_rows),
                                   'last_clock_s': self.obs.sim_time}}

        class FakeClient:
            def stop(self, mode):
                events.append('stop')

        class FakeCtx:
            def __init__(self):
                self.obs = FakeObs()
                self.hooks = FakeHooks(self.obs)
                self.client = FakeClient()

            def ensure_armed(self):
                events.append('armed')

            def mark(self, label):
                return dict(label=label, tick=0, sim=self.obs.sim_time, session=1, wall=0., mono=0.)

            def logs(self, mark, wait=True):
                return [], []

            def steering_status(self):
                return {'emulator': {'lead_steps': 1}}

        saved = (checks.DriveLoop, checks.read_jsonl, checks.emulator_stalls)
        checks.DriveLoop, checks.read_jsonl, checks.emulator_stalls = FakeLoop, lambda *a, **k: [], lambda: []
        try:
            args = argparse.Namespace(step_hold_s=2., sine_s=8., clock='sim')
            result = checks.check_a25(FakeCtx(), args)
            # the first /clock is taken before the ARM and the drive loop starts right after it
            self.assertEqual(events[:4], ['joints', 'first_clock', 'armed', 'loop_start'])
            self.assertTrue(result['passed'])
            by_name = {c['name']: c for c in result['criteria']}
            self.assertAlmostEqual(by_name['steering_moved_deg']['value'], math.degrees(.15), places=6)
            # every step target stays within 150 mrad of zero (the host's 3 deg / 1.5 s tracking window)
            self.assertLessEqual(max(abs(v) for v in checks.A25_STANDSTILL_STEPS_MRAD + checks.A25_DRIVING_STEPS_MRAD),
                                 150)
            self.assertEqual(by_name['joint_minus_model_same_sim_time_p99_deg']['value'], 0.)
            self.assertTrue(result['validity']['valid'])
            # a run whose drives are refused (no steering motion) must not pass on zeros
            events.clear()
            FakeLoop.refuse = True
            result = checks.check_a25(FakeCtx(), args)
            by_name = {c['name']: c for c in result['criteria']}
            self.assertFalse(result['passed'])
            self.assertFalse(by_name['steering_moved_deg']['passed'])
            self.assertEqual(by_name['drive_loop_refusals']['value'], {'stale_input_rearm_required': 1})
            self.assertTrue(by_name['joint_minus_model_same_sim_time_p99_deg']['passed'])   # zeros alone pass
        finally:
            checks.DriveLoop, checks.read_jsonl, checks.emulator_stalls = saved
        motion = checks.a25_motion([dict(delta_eq_rad=-.2)], ((0, (250, -100)),))
        self.assertEqual((motion['passed'], round(motion['required_deg'], 4)), (True, round(math.degrees(.125), 4)))

    def test_step_validity_rules(self):
        class Obs:
            clock_rows = [(k * .1, k * .1) for k in range(200)]          # RTF 1

        class Ctx:
            obs = Obs()
        base = dict({k: 0 for k in checks.EMULATOR_CLOCK_COUNTERS}, clock_messages=0, last_clock_s=0., mono=0.,
                    clock_wall_gap_max_us=0)
        stalled = dict(base, clock_wall_gaps_over_100ms=1, mono=5.)
        ok = checks.step_validity(Ctx(), 'wall', 8., 12., base, dict(base, mono=5.))
        self.assertTrue(ok['valid'])
        self.assertFalse(checks.step_validity(Ctx(), 'wall', 8., 12., base, stalled)['valid'])
        made = checks.step_validity(Ctx(), 'wall', 8., 12., base, dict(base, mono=5.),
                                    sim_made=[dict(latch='steering_status_stale', stall_ms=123.)])
        self.assertEqual((made['valid'], len(made['sim_made_latches'])), (False, 1))
        self.assertIn('SIM-made', made['reason'])
        self.assertTrue(checks.step_validity(Ctx(), 'sim', 8., 12., base, stalled)['valid'])

    def test_invalidate_keeps_values(self):
        crit = [checks.criterion('a', 1, 1, True), checks.criterion('b', 2, 2, None)]
        out = checks.invalidate(crit, dict(valid=False, reason='RTF 0.5 below 0.9 (A10)'))
        self.assertEqual([c['passed'] for c in out], [None, None])
        self.assertTrue(out[0]['invalid'] and 'RTF 0.5' in out[0]['note'])
        self.assertEqual(checks.invalidate(crit, dict(valid=True)), crit)

    def test_a19g_normal_summary(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = Path(tmp) / 'a.json'
            a.write_bytes(json.dumps(dict(check='drive', false_detection=dict(count=0, by_code={}),
                                          emulator_clock=dict(sim_s=30.))).encode('utf-8'))
            b = Path(tmp) / 'b.json'
            b.write_bytes(json.dumps(dict(check='A17_G', false_detection=dict(count=1, by_code={'STALL': 1}),
                                          emulator_clock=dict(sim_s=60.))).encode('utf-8'))
            g = Path(tmp) / 'g.json'
            g.write_bytes(json.dumps(dict(group='a25_firmware', duration_s=30., run=dict(controller_summaries=dict(
                m0=dict(line_kinds=dict(S=10, F=1), first_non_status_lines=[[0, 'UBTS_STEER_V3 hello x'],
                                                                             [23755, 'F THERMAL 5001 23755*73']]),
                m1=dict(line_kinds=dict(S=10), first_non_status_lines=[]))))).encode('utf-8'))
            out = checks.summarize_normal([a, b], [g])
            self.assertEqual(out['by_code'], {'STALL': 1, 'THERMAL': 1})
            self.assertFalse(out['passed'])
            self.assertEqual(out['stall_false_detections'], 1)
            self.assertAlmostEqual(out['stall_rate_per_hour'], 3600. / (30. + 60. + 60.))
            clean = checks.summarize_normal([a])
            self.assertTrue(clean['passed'])

    def test_provenance(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            (base / 'sim' / 'ubts' / 'steering').mkdir(parents=True)
            (base / 'sim' / 'ubts' / 'steering' / 'a.py').write_bytes(b'x = 1\n')
            files = {'sim/ubts/steering/a.py': hashlib.sha256(b'x = 1\n').hexdigest()}
            (base / 'm.json').write_bytes(json.dumps(dict(files=files, image_tag_hint='t')).encode('utf-8'))
            ok = checks.provenance(root=tmp, manifest_path=str(base / 'm.json'), dirs=('sim/ubts',))
            self.assertTrue(ok['sources_match_image'])
            self.assertEqual((ok['files_checked'], ok['manifest']['image_tag_hint']), (1, 't'))
            (base / 'sim' / 'ubts' / 'steering' / 'b.py').write_bytes(b'y = 2\n')         # a mounted extra file
            extra = checks.provenance(root=tmp, manifest_path=str(base / 'm.json'), dirs=('sim/ubts',))
            self.assertEqual((extra['sources_match_image'], extra['extra']), (False, ['sim/ubts/steering/b.py']))
            (base / 'sim' / 'ubts' / 'steering' / 'a.py').write_bytes(b'x = 2\n')         # a changed file
            changed = checks.provenance(root=tmp, manifest_path=str(base / 'm.json'), dirs=('sim/ubts',))
            self.assertEqual(changed['differ'], ['sim/ubts/steering/a.py'])
            self.assertFalse(checks.provenance(root=tmp, manifest_path=str(base / 'none.json'))['sources_match_image'])


if __name__ == '__main__':
    unittest.main()
