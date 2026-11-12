"""A13 truth isolation, static part (steering/truth_isolation.py; design 2.6, 7.2 A13 F).

The check runs over the current work tree (closed-loop estimator and controllers, the steering plant API users, the
host code present) and must pass; planted violations in temporary files (a decision module reading truth, a
plant-side loop passing truth to an estimator, an unclassified module reading truth) must be caught, while
docstrings, comments, sensor synthesis and the nominal drive curve must not be flagged.

unittest style (also collected by pytest); planted files live in temporary folders only.
Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_truth_isolation.py
                    python3 -B -m unittest test_steering_truth_isolation     (Python 3.10, no pytest)
"""
import os
import tempfile
import textwrap
import unittest

from steering import backend_api as ba
from steering import truth_isolation as ti

PLANTED_DECISION = '''
"""A planted executor-side module. Docstrings may say /gps/fix and plant.truth() freely."""
import math
from steering.steered_plant import SteeredPlant            # truth_import
from field_response import FieldPlant                     # truth_import (plant name of a mixed module)
from field_response import forward_curve                  # allowed: nominal drive curve

TOPIC = '/ground_truth/odom'                              # truth_topic


class Executor:
    def step(self, sensors, holder):
        # a comment about plant.x is not code
        pose = self.plant.truth()                         # plant_reference + truth_method
        speed = holder['plant'].v                         # plant_reference
        wheels = sensors.out.delta_left_rad               # truth_field
        column = sensors.arrays['truth_x_m']              # truth_key
        flags = sensors.ModelFlag                         # truth_contract
        lateral = self.world.to_field(0., 0.)             # world_reference + truth_method
        gps = '/gps/data'                                 # truth_topic
        return math.hypot(speed, lateral[1]) + forward_curve(50., None)
'''

PLANTED_PLANT_LOOP = '''
import heapq


def run(plant, estimator, controller, lidar, world):
    packets = []
    frame = lidar.sample(0, plant.lidar_pose(), None)      # sensor synthesis from the true pose: allowed
    heapq.heappush(packets, (0., 'scan', frame.ranges))
    sample = plant.imu_sample()                            # sensor synthesis: allowed
    _, kind, payload = heapq.heappop(packets)
    estimator.on_scan(0., payload, lidar.angle_min, lidar.angle_increment, 30.)
    estimator.on_imu(0., sample['gyro_z_rad_s'], sample['accel_m_s2'])
    snapshot = estimator.snapshot(0.)
    controller.step(0., snapshot)
    true_rate = plant.w                                    # truth
    fx, fy, fyaw = world.to_field(plant.x, plant.y, plant.yaw)
    estimator.on_imu(0., true_rate, (0., 0., 9.81))        # truth_to_decision
    controller.step(0., dict(lateral=fy))                  # truth_to_decision (through a tainted name)
'''

PLANTED_CLOSURE = '''
def outer(plant, controller):
    leaked = plant.x

    def inner():
        controller.step(0., leaked)                        # truth_to_decision through the closure

    def unrelated(obs):
        controller.step(0., obs)                           # clean
    return inner, unrelated
'''

CLEAN_PLANT_LOOP = '''
def run(plant, estimator, controller, lidar):
    frame = lidar.sample(0, plant.lidar_pose(), None)
    estimator.on_scan(0., frame.ranges, lidar.angle_min, lidar.angle_increment, 30.)
    sample = plant.imu_sample()
    estimator.on_imu(0., sample['gyro_z_rad_s'], sample['accel_m_s2'])
    return controller.step(0., estimator.snapshot(0.))
'''

# Review R2: flows the first version missed (each planted line ends with '# leak'); the '# clean' lines must pass.
PLANTED_SINK_FORMS = '''
def run(config, plant, world, world_factory, estimator_factory, controller_factory, SensorEstimator, lidar, caster,
        quaternion):
    built = world_factory(config.world)                          # clean: a plant-side factory
    estimator = SensorEstimator(plant.x, plant.y)                # leak
    controller = controller_factory(world.corridor_truth(0))     # leak
    estimator.pose = plant.rear_axle_pose()                      # leak
    controller.row_end = world.corridor_truth(0)['end_x']        # leak
    controller.hints['end'] = plant.x                            # leak
    lx, ly, lyaw = plant.lidar_pose()
    frame = lidar.sample(1, (lx, ly, .6), quaternion(lyaw))
    estimator.on_scan(frame)                                     # leak
    estimator.on_pose(frame.origin)                              # leak
    estimator.on_scan(frame.ranges, lidar.angle_min)             # clean
    ranges, origin, directions = caster.scan((lx, ly, .6), None)
    estimator.on_pose(origin)                                    # leak
    estimator.on_scan(ranges)                                    # clean
    estimator.label = 'row'                                      # clean
    return built
'''

PLANTED_DECISION_IMPORTS = '''
import importlib
from steering import scene_spec


def step(name, scene_dict):
    scene = scene_spec.SceneSpec.from_dict(scene_dict)
    harness = importlib.import_module('closed_loop.harness')
    plant = __import__('steering.steered_plant')
    other = importlib.import_module(name)
    fine = importlib.import_module('json')
    return scene.start.y_m, harness, plant, other, fine
'''


class TreeTest(unittest.TestCase):
    def test_the_current_tree_passes(self):
        report = ti.check_tree()
        self.assertTrue(report['passed'], [e for e in report['errors']][:20])
        roles = {m['path']: m['role'] for m in report['modules']}
        for rel in ti.REQUIRED_DECISION_MODULES:
            self.assertEqual(roles.get(rel), ti.ROLE_DECISION, rel)
        for rel in ('sim/ubts/steering/steered_plant.py', 'sim/ubts/steering/fast_sim.py',
                    'sim/ubts/closed_loop/harness.py'):
            self.assertEqual(roles.get(rel), ti.ROLE_PLANT, rel)
        self.assertEqual(roles.get('sim/ubts/steering/evaluate_steer.py'), ti.ROLE_EVALUATOR)
        self.assertNotIn('unclassified', [m['role'] for m in report['modules'] if m['allowed_truth_reads']])
        reads = {m['path']: m['allowed_truth_reads'] for m in report['modules']}
        self.assertGreater(sum(reads['sim/ubts/steering/fast_sim.py'].values()), 0)   # the checker does see truth
        for rel in ti.REQUIRED_DECISION_MODULES:
            self.assertEqual(reads[rel], {})

    def test_truth_fields_come_from_the_backend_contract(self):
        self.assertEqual(ti.TRUTH_FIELDS, tuple(ba.TRUTH_OUTPUT_FIELDS))
        for name in ba.TRUTH_OUTPUT_FIELDS:
            self.assertIn(('truth_field', name), ti.string_findings(name))


class PlantedTest(unittest.TestCase):
    def write(self, folder, rel, text):
        path = os.path.join(folder, *rel.split('/'))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as handle:
            handle.write(textwrap.dedent(text).encode('utf-8'))
        return rel

    def test_decision_module_violations_are_caught(self):
        with tempfile.TemporaryDirectory() as folder:
            rel = self.write(folder, 'sim/ubts/closed_loop/estimator.py', PLANTED_DECISION)
            report = ti.check_tree(folder, files=[rel])
        self.assertFalse(report['passed'])
        rules = {}
        for item in report['errors']:
            rules.setdefault(item['rule'], []).append(item['detail'])
        for rule in ('truth_import', 'truth_topic', 'plant_reference', 'truth_method', 'truth_field', 'truth_key',
                     'truth_contract', 'world_reference'):
            self.assertIn(rule, rules, rule)
        self.assertIn('steering.steered_plant', rules['truth_import'])
        self.assertIn('field_response.FieldPlant', rules['truth_import'])
        self.assertNotIn('field_response.forward_curve', rules['truth_import'])
        self.assertEqual(sorted(rules['truth_topic']), ['/gps/data', '/ground_truth/odom'])
        lines = {item['line'] for item in report['errors']}
        source = textwrap.dedent(PLANTED_DECISION).splitlines()
        for line in lines:       # nothing from the docstring or the comments
            self.assertFalse(source[line - 1].strip().startswith(('"""', '#')), source[line - 1])

    def test_plant_loop_sink_rule(self):
        with tempfile.TemporaryDirectory() as folder:
            bad = self.write(folder, 'sim/ubts/closed_loop/harness.py', PLANTED_PLANT_LOOP)
            report = ti.check_tree(folder, files=[bad])
            clean = self.write(folder, 'sim/ubts/steering/fast_sim.py', CLEAN_PLANT_LOOP)
            clean_report = ti.check_tree(folder, files=[clean])
        found = [(e['rule'], e['line']) for e in report['errors']]
        self.assertEqual([rule for rule, _ in found], ['truth_to_decision', 'truth_to_decision'])
        source = textwrap.dedent(PLANTED_PLANT_LOOP).splitlines()
        self.assertEqual(sorted(source[line - 1].split('(')[0].strip() for _, line in found),
                         ['controller.step', 'estimator.on_imu'])
        self.assertTrue(clean_report['passed'], clean_report['errors'])

    def test_constructor_factory_store_and_sensor_frame_flows(self):
        errors, _ = ti.check_source(textwrap.dedent(PLANTED_SINK_FORMS), 'sim/ubts/closed_loop/harness.py',
                                    ti.ROLE_PLANT)
        source = textwrap.dedent(PLANTED_SINK_FORMS).splitlines()
        flagged = sorted({e['line'] for e in errors})
        leaks = sorted(i + 1 for i, line in enumerate(source) if line.rstrip().endswith('# leak'))
        self.assertEqual(flagged, leaks, [source[i - 1] for i in flagged])
        self.assertEqual({e['rule'] for e in errors}, {'truth_to_decision'})
        self.assertEqual(len(leaks), 8)

    def test_scene_module_and_dynamic_imports_in_decision_code(self):
        errors, _ = ti.check_source(textwrap.dedent(PLANTED_DECISION_IMPORTS), 'sim/ubts/closed_loop/steer_control.py',
                                    ti.ROLE_DECISION)
        found = sorted((e['rule'], e['detail']) for e in errors)
        self.assertIn(('truth_import', 'steering.scene_spec'), found)
        self.assertIn(('truth_import', "importlib.import_module('closed_loop.harness')"), found)
        self.assertIn(('truth_import', "__import__('steering.steered_plant')"), found)
        self.assertIn(('dynamic_import', 'importlib.import_module(<not a checkable constant>)'), found)
        self.assertFalse(any("'json'" in detail for _, detail in found))
        self.assertIn('steering.scene_spec', ti.TRUTH_MODULES)
        self.assertEqual(ti.classify('sim/ubts/steering/scene_spec.py')[0], ti.ROLE_PLANT)

    def test_closures_carry_the_taint(self):
        with tempfile.TemporaryDirectory() as folder:
            rel = self.write(folder, 'sim/ubts/closed_loop/harness.py', PLANTED_CLOSURE)
            report = ti.check_tree(folder, files=[rel])
        self.assertEqual([e['detail'] for e in report['errors']], ['controller.step(...) in inner'])

    def test_unclassified_truth_reader_is_an_error_and_a_clean_one_is_not(self):
        with tempfile.TemporaryDirectory() as folder:
            bad = self.write(folder, 'sim/ubts/steering/new_helper.py', 'def f(plant):\n    return plant.x\n')
            good = self.write(folder, 'sim/ubts/steering/other_helper.py', 'def g(a):\n    return a + 1\n')
            report = ti.check_tree(folder, files=[bad, good])
        rules = {(e['path'], e['rule']) for e in report['errors']}
        self.assertIn(('sim/ubts/steering/new_helper.py', 'unclassified_truth_read:plant_reference'), rules)
        self.assertFalse(any(path.endswith('other_helper.py') for path, _ in rules))

    def test_constants_relative_imports_and_scope(self):
        source = textwrap.dedent('''
            from .harness import run_episode
            from . import world
            PLANT_PROFILE_NAMES = ('wheelbase_m',)
            TOPIC = '/tf_static'
        ''')
        errors, _ = ti.check_source(source, 'sim/ubts/closed_loop/controllers.py', ti.ROLE_DECISION)
        rules = sorted({(e['rule'], e['detail']) for e in errors})
        self.assertIn(('truth_import', 'closed_loop.harness'), rules)
        self.assertIn(('truth_import', 'closed_loop.world'), rules)
        self.assertFalse(any(rule == 'plant_reference' for rule, _ in rules))    # ALL-CAPS constant name
        self.assertFalse(any(rule == 'truth_topic' for rule, _ in rules))        # static TF is allowed
        self.assertEqual(ti.classify('sim/ubts/test_anything.py')[0], ti.ROLE_TEST)
        self.assertEqual(ti.classify('dashboard/scripts/steer_new.py')[0], ti.ROLE_DECISION)
        self.assertEqual(ti.classify('tools/row_executor/sim_in_loop_steer_glue.py')[0], ti.ROLE_PLANT)
        self.assertEqual(ti.classify('sim/ubts/steering/unknown_new.py')[0], None)
        with tempfile.TemporaryDirectory() as folder:
            self.write(folder, 'sim/ubts/adapter.py', 'X = "/odom"\n')                  # outside the scope
            self.write(folder, 'sim/ubts/steering/__pycache__/x.py', 'X = 1\n')         # never scanned
            self.write(folder, 'sim/ubts/steering/y.py', 'X = 1\n')
            self.assertEqual(ti.scope_files(folder), ['sim/ubts/steering/y.py'])
            report = ti.check_tree(folder)
        self.assertFalse(report['passed'])        # the required decision modules are missing in this folder
        self.assertEqual(sorted(report['missing_required_decision_modules']), sorted(ti.REQUIRED_DECISION_MODULES))



PLANTED_EXEMPTION = '''
class Facade:
    def _facade_state(self, plant):
        v = plant.v
        self.rest_since = v
        self._controllers.append(self.ctl)
        self.estimator.on_imu(v)
'''

PLANTED_SUBSCRIPTIONS = '''
import argparse


class Node:
    def start(self, node, topic='/scan'):
        node.create_subscription(object, topic, None, 10)
        node.create_subscription(object, '/ground_truth/odom', None, 10)
        node.create_subscription(object, self.name, None, 10)


def parse():
    p = argparse.ArgumentParser()
    p.add_argument('--imu-topic', default='/imu/data_raw')
    p.add_argument('--gps-topic', default='/gps/data')
    p.add_argument('--other-topic', default=None)
'''


class FrontSteerExecutorTest(unittest.TestCase):
    """P4 (front-steer executor): the roles of the new files, the executor subscription allowlist and the precise
    plant-side exemptions."""

    def test_roles_of_the_p4_files(self):
        roles = {
            'sim/ubts/steering/scripted_control.py': ti.ROLE_DECISION,
            'tools/row_executor/core.py': ti.ROLE_DECISION, 'tools/row_executor/config.py': ti.ROLE_DECISION,
            'tools/row_executor/guard.py': ti.ROLE_DECISION, 'tools/row_executor/bridge_client.py': ti.ROLE_DECISION,
            'tools/row_executor/executor_main.py': ti.ROLE_DECISION,
            'tools/row_executor/sim_in_loop_steer.py': ti.ROLE_PLANT, 'sim/ubts/steering/t1_rig.py': ti.ROLE_PLANT,
            'sim/ubts/steering/safety_scripts/acceptance.py': ti.ROLE_EVALUATOR,
            'sim/ubts/steering/safety_scripts/catalogue.py': ti.ROLE_EVALUATOR,
            'sim/ubts/steering/safety_scripts/measure.py': ti.ROLE_EVALUATOR,
            'sim/ubts/steering/evaluate_steer.py': ti.ROLE_EVALUATOR,
        }
        for rel, role in roles.items():
            self.assertEqual(ti.classify(rel)[0], role, rel)
        report = ti.check_tree()
        checked = {m['path']: m['role'] for m in report['modules']}
        for rel, role in roles.items():
            self.assertEqual(checked.get(rel), role, rel)
        reads = {m['path']: m['allowed_truth_reads'] for m in report['modules']}
        self.assertEqual(reads['sim/ubts/steering/scripted_control.py'], {})

    def test_executor_subscriptions_stay_inside_the_allowlist(self):
        self.assertEqual(ti.EXECUTOR_TOPIC_ALLOWLIST, ('/scan', '/imu/data_raw', '/rover/control/requested'))
        # A13 revision 30 (PC-12): the GNSS fix of the GNSS layer (subscribed only with gnss_mode not off)
        self.assertEqual(ti.EXECUTOR_GNSS_TOPICS, ('/gps/fix',))
        report = ti.subscription_check()
        self.assertTrue(report['passed'], report['problems'])
        self.assertTrue(set(report['topics']) <= set(ti.EXECUTOR_TOPIC_ALLOWLIST + ti.EXECUTOR_GNSS_TOPICS))
        self.assertIn('/scan', report['topics'])

    def test_planted_subscriptions_are_caught(self):
        with tempfile.TemporaryDirectory() as folder:
            rel = 'tools/row_executor/executor_main.py'
            path = os.path.join(folder, *rel.split('/'))
            os.makedirs(os.path.dirname(path))
            with open(path, 'wb') as handle:
                handle.write(textwrap.dedent(PLANTED_SUBSCRIPTIONS).encode('utf-8'))
            report = ti.subscription_check(folder, files=(rel,))
        self.assertFalse(report['passed'])
        text = ' '.join(report['problems'])
        self.assertIn('topic /ground_truth/odom outside the allowlist', text)
        self.assertIn('topic /gps/data outside the allowlist', text)     # revision 30 allows /gps/fix only
        self.assertIn('unresolved subscription topic', text)
        self.assertIn('without a constant default', text)
        self.assertIn('/scan', report['topics'])           # the parameter default is resolved
        self.assertIn('/imu/data_raw', report['topics'])

    def test_exemptions_match_path_function_and_sink(self):
        errors, info = ti.check_source(textwrap.dedent(PLANTED_EXEMPTION), 'tools/row_executor/single_row_sim.py',
                                       ti.ROLE_PLANT)
        self.assertEqual([e['detail'] for e in errors], ['self.estimator.on_imu(...) in _facade_state'])
        exempted = [i for i in info if i['rule'] == 'exempted:truth_to_decision']
        self.assertEqual([i['detail'] for i in exempted], ['self._controllers.append(...) in _facade_state'])
        self.assertTrue(exempted[0]['exempted'])
        # the same function in another file is not exempted
        errors, _ = ti.check_source(textwrap.dedent(PLANTED_EXEMPTION), 'tools/row_executor/other_sim.py',
                                    ti.ROLE_PLANT)
        self.assertEqual(len(errors), 2)
        for key, reason in ti.SINK_EXEMPTIONS.items():
            self.assertEqual(len(key), 3, key)          # (path, function, detail): no function-wide exemption
            self.assertTrue(isinstance(reason, str) and reason)

    def test_a_new_leak_in_an_exempted_function_is_an_error(self):
        # review P4R-01: run_job / load_matrix / main of sim_in_loop.py had (path, function) exemptions that hid any new
        # flow in those functions. A planted estimator call with world truth must now be an error in each of them.
        rel = 'tools/row_executor/sim_in_loop.py'
        source = (ti.repository_root() / rel).read_bytes().decode('utf-8')
        errors, info = ti.check_source(source, rel, ti.ROLE_PLANT)
        self.assertEqual(errors, [])
        exempted = sorted(i['detail'] for i in info if i['rule'] == 'exempted:truth_to_decision')
        self.assertEqual(len(exempted), 8, exempted)
        self.assertIn('ExecutorInTheLoop(...) in run_job', exempted)      # the exec_field operator row length
        lines = source.split('\n')
        import ast
        tree = ast.parse(source)
        for name in ('run_job', 'load_matrix', 'main'):
            function = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
            first = function.body[1] if isinstance(function.body[0], ast.Expr) else function.body[0]
            planted = list(lines)
            planted.insert(first.lineno - 1, ' ' * first.col_offset + 'estimator.on_scan(world.corridor_truth())')
            errors, info = ti.check_source('\n'.join(planted), rel, ti.ROLE_PLANT)
            self.assertEqual([e['detail'] for e in errors], ['estimator.on_scan(...) in %s' % name], name)
            self.assertEqual(len([i for i in info if i['rule'] == 'exempted:truth_to_decision']), 8, name)
        # one key exempts one finding: a second identical sink in the same function is an error
        function = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'load_matrix')
        call = next(n for n in ast.walk(function) if isinstance(n, ast.Call) and getattr(n.func, 'id', '') ==
                    'executor_config')
        planted = list(lines)
        statement = lines[call.lineno - 1]
        planted.insert(call.lineno, statement)
        errors, _ = ti.check_source('\n'.join(planted), rel, ti.ROLE_PLANT)
        self.assertEqual([e['detail'] for e in errors], ['executor_config(...) in load_matrix'])

    def test_tree_report_lists_the_exemptions(self):
        report = ti.check_tree()
        self.assertEqual(report['exempted_count'], len(report['exempted']))
        self.assertEqual(report['unused_exemptions'], [])
        sink = [e for e in report['exempted'] if e['rule'] == 'truth_to_decision']
        self.assertEqual(len(sink), len(ti.SINK_EXEMPTIONS))
        for item in report['exempted']:
            self.assertTrue(item['reason'], item)


PLANTED_RIG_READS = '''
class SteerBridgeClient:
    def peek(self, t):
        a = self.transport.rig.mcu_channel_at('servo_pos', t)          # rig_reference + rig_channel
        b = self.transport._rig.mcu_lines()                           # rig_reference + rig_channel
        c = self.transport.fw                                         # rig_reference (firmware process)
        d = self.config.start_steer_rad                               # scene_truth
        e = self.config.start_pose                                    # scene_truth
        f = self.transport.trace_arrays()                             # rig_channel
        g = self.link.serial_log()                                    # rig_channel
        return a, b, c, d, e, f, g, self.origin, self.right, self.trigger    # not rig words
'''


class RigChannelTest(unittest.TestCase):
    """Review RS-05: the T1 rig evaluation channels (LOCKSTEP 11.6) reached through the executor's transport."""

    def test_rig_channel_reads_in_decision_code_are_caught(self):
        errors, _ = ti.check_source(textwrap.dedent(PLANTED_RIG_READS), 'tools/row_executor/bridge_client.py',
                                    ti.ROLE_DECISION)
        rules = {}
        for item in errors:
            rules.setdefault(item['rule'], set()).add(item['line'])
        source = textwrap.dedent(PLANTED_RIG_READS).splitlines()
        planted = {i + 1 for i, line in enumerate(source) if '# rig' in line or '# scene' in line}
        self.assertEqual(set().union(*rules.values()), planted)
        self.assertEqual(set(rules), {'rig_reference', 'rig_channel', 'scene_truth'})
        for word in ('origin', 'right', 'trigger', 'original', 'RIG_TABLE'):
            self.assertFalse(ti._is_rig_word(word), word)
        for word in ('rig', '_rig', 'rig_client', 'test_rig', 'a_rig_b'):
            self.assertTrue(ti._is_rig_word(word), word)

    def test_plant_side_glue_keeps_its_rig(self):
        # the rig rules are strict-role rules: the plant-side glue hands the executor the rig clock by design
        source = 'def f(rig, executor):\n    executor.step(rig.clock())\n    return rig.mcu_lines()\n'
        errors, reads = ti.check_source(source, 'tools/row_executor/sim_in_loop_steer.py', ti.ROLE_PLANT)
        self.assertEqual(errors, [])
        self.assertTrue(any(r['rule'] == 'rig_channel' for r in reads))

    def test_rig_client_keeps_the_rig_private(self):
        import types
        from steering import t1_rig as tr
        client = tr.RigClient(types.SimpleNamespace(default_client=None), 'owner')
        self.assertFalse(hasattr(client, 'rig'))
        self.assertFalse([name for name in vars(client) if ti._is_rig_word(name) and not name.startswith('_')])


class FieldCalibScopeTest(unittest.TestCase):
    """Review P4R-13: the identification analysis library is in the A13 scope (strict role shared)."""

    def test_field_calib_files_are_checked_as_shared(self):
        report = ti.check_tree()
        roles = {m['path']: m['role'] for m in report['modules']}
        calib = [p for p in roles if p.startswith('tools/field_calib/')]
        self.assertTrue(calib)
        for path in calib:
            expected = ti.ROLE_TEST if path.startswith('tools/field_calib/tests/') else ti.ROLE_SHARED
            self.assertEqual(roles[path], expected, path)
        errors, _ = ti.check_source('def f(rec):\n    return rec.plant.x\n', 'tools/field_calib/records.py',
                                    ti.classify('tools/field_calib/records.py')[0])
        self.assertEqual({e['rule'] for e in errors}, {'plant_reference'})


if __name__ == '__main__':
    unittest.main()
