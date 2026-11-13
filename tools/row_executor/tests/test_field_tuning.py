"""Field tuning file (field_tuning.py; design v2 V2-5, decision R35; PW-07, PW-08, PC-13).

The schema and its checks (duplicate keys, NaN, unknown keys, ranges, a falling table, the rules between values, the
locked keys against the executor config), the defaults against the design table and the steering controller, the
package default file, the API of the executor loop (basic keys only, at rest only, a preset, back to the default preset
gives the original digest, the change log events, save to a configured file), and the same file identity in the
records of the executor, the fast SIM and the Gazebo executor container. Standard library + the test helpers only.
"""
import json
import os
import shutil
import sys
import tempfile
import unittest
from dataclasses import replace

import rowexec_testlib as L  # noqa: F401  (path setup)
from row_executor import config as C
from row_executor import executor_main as M
from row_executor import field_tuning as T
from row_executor.core import PAUSED
from test_executor_loop import Clock
from test_steer_executor import MOTOR, SCIPY_PROBLEM

SIM_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
                       'sim', 'ubts')
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)


def steer_shadow(**over):
    base = dict(vehicle_profile='front_steer', mode='single_row', require_native_navigation=False,
                max_yaw_rate_deg_s=3.6, steer_limit_left_mrad=409, steer_limit_right_mrad=409)
    base.update(over)
    return C.ExecutorConfig(**base).validate()


def file_bytes(values=None, **top):
    data = dict(schema=T.SCHEMA, preset='default', values=values or {})
    data.update(top)
    return json.dumps(data).encode('utf-8')


class SchemaTest(unittest.TestCase):
    def problems(self, raw):
        with self.assertRaises(T.TuningError) as ctx:
            T.parse(raw)
        return ctx.exception.problems

    def test_defaults_are_the_design_table(self):
        d = T.defaults()
        # design v2 V2-5 item 2 (basic) and V2-7: k_field 1.0, target 0.40, lookahead 1.5 / 1.25 / 3.0, k_d = k_psi = 1
        self.assertEqual((d['speed_scale'], d['target_speed_m_s'], d['lookahead_base_m'], d['lookahead_per_speed_s'],
                          d['lookahead_max_m'], d['k_d'], d['k_psi']), (1., .4, 1.5, 1.25, 3., 1., 1.))
        self.assertEqual((d['max_speed_m_s'], d['front_margin_m'], d['operator_timeout_s'], d['lidar_max_age_s'],
                          d['imu_max_age_s'], d['steer_status_max_age_s'], d['stall_guard_s']),
                         (.5, .35, 1., .45, .4, .3, .35))
        self.assertEqual((d['trim_limit_deg'], d['steer_follow_limit_deg'], d['steer_filter_s']), (3., 12., .25))
        self.assertEqual([k.name for k in T.KEYS if k.group == 'basic'], list(T.BASIC))
        self.assertEqual(len(T.BASIC), 8)

    def test_defaults_equal_the_controller_and_the_vendored_table(self):
        from closed_loop import actuator_prior, steer_control
        self.assertEqual(T.DEFAULT_SPEED_TABLE, actuator_prior.DEFAULT_SPEED_TABLE)
        cfg = steer_control.SteerFollowConfig()
        fields = T.controller_fields(T.defaults())
        for name, value in fields.items():
            want = getattr(cfg, name)
            if name == 'speed_table':
                self.assertEqual(tuple(tuple(p) for p in value), want)
            else:
                self.assertEqual(value, want, name)
        self.assertEqual(steer_control.controller_config({'kind': 'row_follow'},
                                                         C.with_tuning(steer_shadow(), T.defaults())),
                         steer_control.controller_config({'kind': 'row_follow'}, steer_shadow()))

    def test_package_file_is_the_defaults_with_two_presets(self):
        f = T.load()
        self.assertEqual(T.digest(f.values), T.digest(T.defaults()))
        self.assertEqual(sorted(f.presets), ['slow_030', 'soft_ground'])
        with open(T.PACKAGE_FILE, 'rb') as fh:
            raw = fh.read()
        self.assertNotIn(b'\r', raw)
        self.assertEqual(T.parse(T.dumps(f.values, note=f.note, presets=f.presets).encode('utf-8')).values, f.values)

    def test_refusals(self):
        self.assertIn('duplicate key', self.problems(b'{"schema": "ubts.field_tuning.v1", "preset": "a", "preset": "b"}')[0])
        self.assertIn('non-finite', self.problems(
            b'{"schema": "ubts.field_tuning.v1", "values": {"basic": {"k_d": NaN}}}')[0])
        self.assertIn('not a tuning key', self.problems(file_bytes({'basic': {'k_x': 1.}}))[0])
        self.assertIn('belongs to the group locked', self.problems(file_bytes({'basic': {'max_speed_m_s': .4}}))[0])
        self.assertIn('outside', self.problems(file_bytes({'basic': {'target_speed_m_s': .6}}))[0])
        self.assertIn('must be an integer', self.problems(file_bytes({'advanced': {'duty_max_pct': 60.5}}))[0])
        self.assertIn('must not fall', ' '.join(self.problems(file_bytes(
            {'advanced': {'speed_table': [[20, .3], [40, .2]]}}))))
        self.assertIn('the duty must rise', ' '.join(self.problems(file_bytes(
            {'advanced': {'speed_table': [[40, .2], [40, .3]]}}))))
        self.assertIn('min_speed_m_s < target_speed_m_s', ' '.join(self.problems(file_bytes(
            {'basic': {'target_speed_m_s': .2}}))))
        self.assertIn('lookahead_base_m', ' '.join(self.problems(file_bytes(
            {'basic': {'lookahead_base_m': 2.5, 'lookahead_max_m': 2.}}))))
        self.assertIn('speed_scale x surface_scale x battery_scale', ' '.join(self.problems(file_bytes(
            {'basic': {'speed_scale': .55}, 'advanced': {'surface_scale': .8}}))))
        self.assertIn('schema', ' '.join(self.problems(json.dumps(dict(schema='x')).encode())))
        self.assertIn('unknown key', ' '.join(self.problems(file_bytes(extra=1))))
        self.assertIn('presets.default', ' '.join(self.problems(file_bytes(presets={'default': {'values': {}}}))))

    def test_locked_values_must_equal_the_executor_config(self):
        values = T.resolve({})
        self.assertEqual(T.lock_problems(values, steer_shadow()), [])
        problems = T.lock_problems(values, steer_shadow(lidar_max_age_s=.4))
        self.assertTrue(problems and problems[0].startswith('lidar_max_age_s'))
        with self.assertRaises(C.ConfigError):
            C.with_tuning(steer_shadow(operator_timeout_s=.5), values)    # default 1.0 (decision R60)

    def test_digest_and_config(self):
        values = T.resolve({'target_speed_m_s': .3})
        self.assertNotEqual(T.digest(values), T.digest(T.defaults()))
        self.assertEqual(T.digest(T.resolve({'target_speed_m_s': .4})), T.digest(T.defaults()))
        cfg = C.with_tuning(steer_shadow(), values)
        self.assertEqual(cfg.tuning_values['cruise_speed_m_s'], .3)
        self.assertEqual(C.from_dict(cfg.to_dict()).digest(), cfg.digest())
        self.assertNotIn('tuning_values', C.ExecutorConfig().to_dict())             # untouched: digest unchanged
        explicit = C.with_tuning(steer_shadow(prior_speed_table=((20., .1), (60., .5)), prior_speed_scale=.9), values)
        self.assertEqual((explicit.prior_speed_table, explicit.prior_speed_scale), (((20., .1), (60., .5)), .9))
        scaled = C.with_tuning(steer_shadow(), T.resolve({'speed_scale': 1.2, 'battery_scale': .9}))
        self.assertAlmostEqual(scaled.prior_speed_scale, 1.08)
        self.assertAlmostEqual(scaled.tuning_values['speed_scale'], 1.08)


@unittest.skipIf(SCIPY_PROBLEM, SCIPY_PROBLEM)
class LoopApiTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='tuning_test_')
        self.path = os.path.join(self.tmp, 'field_tuning.json')
        shutil.copyfile(T.PACKAGE_FILE, self.path)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def events(self, rig, name):
        return [r for r in rig.log.records if r.get('event') == name]

    def rig(self, **over):
        """A SHADOW front_steer ExecutorLoop on a virtual clock with the row-following controller; request() runs one
        step."""
        from closed_loop import steer_control
        from row_executor.core import Runner
        cfg = steer_shadow(tuning_file=self.path, **over)
        log = M._NullLog()
        clock = Clock()
        loop = M.ExecutorLoop(cfg, clock=clock, wall=lambda: 1.7e9, log=log,
                              runner_factory=lambda c: Runner(c, controller_factory=steer_control.make_factory(
                                  {'kind': 'row_follow'}, c)))

        class Handle:
            pass
        h = Handle()
        h.loop, h.log = loop, log

        def request(kind, **kw):
            future = loop.submit(kind, **kw)
            clock.t += .05
            loop.step()
            return future.result()
        h.request = request
        return h

    def test_basic_change_logs_and_reaches_the_controller(self):
        r = self.rig()
        d0 = r.loop.tuning.digest
        reply = r.request('params', params={'target_speed_m_s': .3, 'k_d': 1.2})
        self.assertTrue(reply['ok'], reply)
        self.assertEqual([c['key'] for c in reply['changes']], ['k_d', 'target_speed_m_s'])
        changes = self.events(r, 'tuning_change')
        self.assertEqual(len(changes), 2)
        self.assertEqual({c['digest_before'] for c in changes}, {d0})
        self.assertEqual(r.loop.config.tuning_values['cruise_speed_m_s'], .3)
        self.assertEqual(r.loop.tuning.revision, 1)
        self.assertEqual(r.loop.tuning.preset, 'default*')
        status = r.loop.status()['tuning']
        self.assertTrue(status['editable'] and status['modified'])
        self.assertEqual((status['preset'], status['revision']), ('default*', 1))
        # back to the default preset: the digest of the defaults again (PC-13)
        reply = r.request('params', params={'preset': 'default'})
        self.assertTrue(reply['ok'])
        self.assertEqual(r.loop.tuning.digest, d0)
        self.assertEqual(r.loop.tuning.digest, T.digest(T.defaults()))
        self.assertEqual(r.loop.config.tuning_values['cruise_speed_m_s'], .4)

    def test_refused_keys_and_values(self):
        r = self.rig()
        for params, word in (({'max_speed_m_s': .45}, 'locked'), ({'duty_max_pct': 60}, 'advanced'),
                             ({'nope': 1}, 'not a tuning key'), ({'target_speed_m_s': .55}, 'outside'),
                             ({'target_speed_m_s': True}, 'finite number'), ({}, 'non-empty'),
                             ({'preset': 'nope'}, 'unknown'), ({'preset': 'slow_030', 'k_d': 1.}, 'alone')):
            reply = r.request('params', params=params)
            self.assertEqual((reply['ok'], reply['error'], reply.get('http')), (False, 'invalid_params', 400), params)
            self.assertIn(word, ' '.join(reply['problems']), params)
        self.assertEqual(r.loop.tuning.revision, 0)
        self.assertEqual(self.events(r, 'tuning_change'), [])

    def test_only_at_rest(self):
        r = self.rig()
        self.assertTrue(r.request('start', row_len_m=None)['ok'])
        reply = r.request('params', params={'k_d': 1.1})
        self.assertEqual((reply['error'], reply['http']), ('tuning_locked_while_running', 409))
        self.assertEqual(r.loop.runner.state, PAUSED)
        self.assertFalse(r.loop.status()['tuning']['editable'])
        self.assertEqual(r.loop.status()['tuning']['not_editable_because'], 'tuning_locked_while_running')

    def test_presets_and_save(self):
        r = self.rig()
        self.assertTrue(r.request('params', params={'preset': 'slow_030'})['ok'])
        self.assertEqual(r.loop.config.tuning_values['cruise_speed_m_s'], .3)
        self.assertTrue(r.request('params', params={'k_psi': 1.3})['ok'])
        reply = r.request('params', params={'save_preset': 'field_a', 'note': 'test'})
        self.assertTrue(reply['ok'], reply)
        f = T.load(self.path)
        self.assertEqual((f.preset, f.values['k_psi'], f.values['target_speed_m_s']), ('field_a', 1.3, .3))
        self.assertIn('field_a', f.presets)
        self.assertEqual(self.events(r, 'tuning_save')[-1]['file_sha256'], f.sha256)
        # a fresh loop on the saved file starts with those values
        r2 = self.rig()
        self.assertEqual(r2.loop.config.tuning_values['k_psi'], 1.3)
        self.assertEqual(r2.loop.tuning.file.sha256, f.sha256)

    def test_package_file_is_read_only_and_a_bad_file_refuses_the_start(self):
        r = self.rig()
        r.loop.tuning.file = replace(r.loop.tuning.file, path=T.PACKAGE_FILE)
        reply = r.request('params', params={'save': True})
        self.assertEqual(reply['error'], 'invalid_params')
        self.assertIn('read only', reply['problems'][0])
        with open(self.path, 'wb') as fh:
            fh.write(file_bytes({'basic': {'target_speed_m_s': .7}}))
        with self.assertRaises(ValueError) as ctx:
            self.rig()
        self.assertIn('field tuning refused', str(ctx.exception))

    def test_records_name_the_file(self):
        r = self.rig()
        sha = T.load(self.path).sha256
        config_events = self.events(r, 'config')
        self.assertEqual(config_events[0]['tuning']['file_sha256'], sha)
        self.assertTrue(r.request('start', row_len_m=None)['ok'])
        self.assertEqual(self.events(r, 'start')[-1]['tuning']['file_sha256'], sha)


@unittest.skipIf(SCIPY_PROBLEM, SCIPY_PROBLEM)
class ThreePlacesTest(unittest.TestCase):
    """PW-08: the executor, the fast SIM (T1) and the Gazebo executor container load the same package file."""

    def test_fast_sim_config_and_container_use_the_package_file(self):
        sha = T.load().sha256
        from steering import exec_container_main as X
        from steering.vehicle_profile import load_profile
        profile = load_profile()
        host = X.host_profile_of(profile, 1)
        cfg = X.gsim_executor_config(profile, host, 1)
        self.assertIsNone(cfg.tuning_file)
        self.assertEqual(T.State(T.load(cfg.tuning_file)).file.sha256, sha)
        from row_executor import sim_in_loop_steer as S
        self.assertIs(S.XM.T, T)


if __name__ == '__main__':
    unittest.main()
