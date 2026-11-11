import json
import os
import tempfile
import unittest

import rowexec_testlib  # noqa: F401  (path setup)
from row_executor import config as C
from row_executor.config import ConfigError, ExecutorConfig


def feasible(**over):
    """A configuration that satisfies every motor-output rule (numbers are examples, not field values)."""
    # steering GO rule (field decision rules, 2026-10-22): row motor output needs max_differential 10-20 and a measured
    # authority >= 4.0 deg/s; base 60 keeps the inner wheel at breakaway 40 + margin 10 with differential 10
    base = dict(motor_output_enabled=True, base_duty_pct=60, reduce_duty_pct=60, max_differential_pct=10,
                duty_cap_pct=70, diff_cap_pct=20, breakaway_duty_pct=40, inner_margin_pct=10,
                v_measured_m_s=.48, measured_yaw_at_max_diff_deg_s=4.2, max_yaw_rate_deg_s=2.5,
                measured_stop_distance_m=.25)       # C7 (day-2 N8: required for row motor output)
    base.update(over)
    return ExecutorConfig(**base)


class DefaultsTest(unittest.TestCase):
    def test_defaults_are_shadow_and_valid(self):
        cfg = ExecutorConfig().validate()
        self.assertFalse(cfg.motor_output_enabled)
        self.assertEqual(cfg.profile, 'field')
        self.assertEqual(cfg.mode, 'continuous_side')
        self.assertEqual(cfg.bench_pattern, ())
        self.assertEqual(cfg.feasibility()['status'], 'unknown')
        self.assertIsNone(cfg.guard_inner_min_pct())

    def test_json_round_trip_and_digest(self):
        cfg = feasible(body_mask=((10, 12, .45),), controller_overrides={'yaw_kp_pct_per_deg_s': 1.5})
        text = C.dumps(cfg)
        again = C.loads(text)
        self.assertEqual(again, cfg)
        self.assertEqual(again.digest(), cfg.digest())
        self.assertNotEqual(cfg.digest(), ExecutorConfig().digest())

    def test_shipped_example_is_the_shadow_default(self):
        path = os.path.join(rowexec_testlib.TOOLS, 'row_executor', 'config.field_shadow.example.json')
        cfg = C.load_config(path, expected_profile='field')
        self.assertEqual(cfg, ExecutorConfig())
        self.assertFalse(cfg.motor_output_enabled)

    def test_load_config_profile_must_match(self):
        bench = ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK, bench_pattern=((2., 40, 40),))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'c.json')
            with open(path, 'w', encoding='utf-8', newline='') as fh:
                fh.write(C.dumps(bench))
            with self.assertRaisesRegex(ConfigError, "profile 'bench' loaded where 'field'"):
                C.load_config(path, expected_profile='field')
            self.assertEqual(C.load_config(path, expected_profile='bench').profile, 'bench')


class StrictParsingTest(unittest.TestCase):
    def test_unknown_duplicate_and_nonfinite_refused(self):
        with self.assertRaisesRegex(ConfigError, 'unknown key: turbo'):
            C.loads(json.dumps({'turbo': True}))
        with self.assertRaisesRegex(ConfigError, 'duplicate key'):
            C.loads('{"base_duty_pct": 55, "base_duty_pct": 60}')
        with self.assertRaisesRegex(ConfigError, 'non-finite'):
            C.loads('{"scan_latency_s": NaN}')
        with self.assertRaisesRegex(ConfigError, 'invalid JSON'):
            C.loads('{"base_duty_pct": ')

    def test_types_are_strict(self):
        for key, value in (('base_duty_pct', True), ('base_duty_pct', 55.0), ('motor_output_enabled', 1),
                           ('scan_latency_s', '0.08'), ('breakaway_duty_pct', 40.5)):
            with self.subTest(key=key, value=value), self.assertRaises(ConfigError):
                C.loads(json.dumps({key: value}))

    def test_ranges(self):
        for key, value in (('scan_latency_s', .5), ('base_duty_pct', 0), ('control_hz', 10.),
                           ('corridor_every_n_scans', 1), ('imu_yaw_sign', 0), ('lidar_yaw_offset_deg', 200.)):
            with self.subTest(key=key), self.assertRaises(ConfigError):
                C.loads(json.dumps({key: value}))


class ProfileRulesTest(unittest.TestCase):
    def test_field_profile_refuses_bench_pattern(self):
        with self.assertRaisesRegex(ConfigError, 'bench patterns bypass the controller'):
            ExecutorConfig(bench_pattern=((1., 40, 40),)).validate()
        with self.assertRaisesRegex(ConfigError, 'bench_pattern/bench_ack are not allowed'):
            ExecutorConfig(bench_ack=C.BENCH_ACK).validate()
        with self.assertRaisesRegex(ConfigError, 'bench patterns bypass'):
            C.loads(json.dumps({'bench_pattern': [[1, 40, 40]]}))

    def test_bench_profile_needs_ack_and_forward_steps(self):
        with self.assertRaisesRegex(ConfigError, 'bench_ack must be'):
            ExecutorConfig(profile='bench', bench_pattern=((1., 40, 40),)).validate()
        ok = ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK, bench_pattern=((1., 40, 40), (1., 30, 50)))
        ok.validate()
        for step, msg in (((1., -40, 40), 'negative duty'), ((1., 90, 90), 'duty above'),
                          ((1., 20, 70), 'differential above'), ((11., 40, 40), 'duration outside')):
            with self.subTest(step=step), self.assertRaisesRegex(ConfigError, msg):
                ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK, bench_pattern=(step,)).validate()
        with self.assertRaisesRegex(ConfigError, 'total duration'):
            ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK, bench_pattern=((10., 40, 40),) * 7).validate()

    def test_only_forward_only_modes(self):
        for mode in ('hybrid', 'stop_turn', 'hybrid_side', 'hybrid_fallback', 'stop_turn_side'):
            with self.subTest(mode=mode), self.assertRaisesRegex(ConfigError, 'forward-only'):
                ExecutorConfig(mode=mode).validate()
        ExecutorConfig(mode='continuous').validate()


class FeasibilityTest(unittest.TestCase):
    def test_critique_h1_example_is_infeasible(self):
        # base 55 / diff 20 -> inner 35; breakaway 44 + margin 10 = 54 (FEASIBILITY_CRITIQUE H1).
        cfg = ExecutorConfig(base_duty_pct=55, reduce_duty_pct=45, max_differential_pct=20, breakaway_duty_pct=44)
        with self.assertRaises(ConfigError) as ctx:
            cfg.validate()
        text = str(ctx.exception)
        self.assertIn('inner wheel base-max_differential = 35 < breakaway+margin = 54', text)
        self.assertIn('inner wheel reduce-max_differential = 25 < breakaway+margin = 54', text)

    def test_reduce_below_breakaway_refused(self):
        with self.assertRaisesRegex(ConfigError, 'reduce-max_differential'):
            feasible(reduce_duty_pct=50).validate()

    def test_margin_is_configurable(self):
        # SHADOW: any margin; with motor output (day-2 review N1) the margin can not go below the evaluated 10
        feasible(base_duty_pct=56, max_differential_pct=10, diff_cap_pct=20, duty_cap_pct=66, motor_output_enabled=False,
                 breakaway_duty_pct=46, inner_margin_pct=0, reduce_duty_pct=56).validate()
        with self.assertRaises(ConfigError):
            feasible(base_duty_pct=56, max_differential_pct=10, diff_cap_pct=20, duty_cap_pct=66,
                     motor_output_enabled=False, breakaway_duty_pct=46, inner_margin_pct=1,
                     reduce_duty_pct=56).validate()
        with self.assertRaisesRegex(ConfigError, 'inner_margin_pct 0 below the evaluated field limit 10'):
            feasible(base_duty_pct=56, max_differential_pct=10, diff_cap_pct=20, duty_cap_pct=66,
                     breakaway_duty_pct=36, inner_margin_pct=0, reduce_duty_pct=56).validate()

    def test_feasible_motor_config(self):
        cfg = feasible().validate()
        self.assertEqual(cfg.feasibility()['status'], 'ok')
        self.assertEqual(cfg.guard_inner_min_pct(), 50)

    def test_motor_output_needs_every_measurement(self):
        with self.assertRaises(ConfigError) as ctx:
            ExecutorConfig(motor_output_enabled=True).validate()
        text = str(ctx.exception)
        for needed in ('breakaway_duty_pct', 'v_measured_m_s', 'measured_yaw_at_max_diff_deg_s'):
            self.assertIn('motor_output_enabled needs ' + needed, text)
        with self.assertRaisesRegex(ConfigError, 'needs inner_rule_enabled'):
            feasible(inner_rule_enabled=False).validate()

    def test_speed_and_yaw_authority(self):
        with self.assertRaisesRegex(ConfigError, 'above max_speed_m_s'):
            feasible(v_measured_m_s=.6).validate()
        with self.assertRaisesRegex(ConfigError, 'yaw authority 1.50 deg/s below 2.00'):
            feasible(measured_yaw_at_max_diff_deg_s=1.5, max_yaw_rate_deg_s=.8).validate()
        with self.assertRaisesRegex(ConfigError, 'max_yaw_rate_deg_s 2.60 above'):
            feasible(max_yaw_rate_deg_s=2.6).validate()

    def test_motor_steering_rule(self):
        # field decision rules (2026-10-22): row motor output needs max_differential 10-20 and >= 4.0 deg/s authority
        with self.assertRaisesRegex(ConfigError, r'yaw authority 3.00 deg/s below 4.00 \(steering GO rule'):
            feasible(measured_yaw_at_max_diff_deg_s=3., max_yaw_rate_deg_s=1.8).validate()
        feasible(measured_yaw_at_max_diff_deg_s=4., max_yaw_rate_deg_s=2.4).validate()      # the SIM edge passes
        with self.assertRaisesRegex(ConfigError, 'max_differential_pct 9 outside'):
            feasible(max_differential_pct=9, base_duty_pct=59, reduce_duty_pct=59).validate()
        with self.assertRaisesRegex(ConfigError, 'max_differential_pct 21 outside'):
            feasible(max_differential_pct=21, base_duty_pct=71, reduce_duty_pct=71, duty_cap_pct=92,
                     diff_cap_pct=42, v_measured_m_s=.48, measured_yaw_at_max_diff_deg_s=8.4,
                     max_yaw_rate_deg_s=4.5).validate()
        # the stricter configured minimum keeps its own (single) message
        with self.assertRaises(ConfigError) as ctx:
            feasible(min_yaw_authority_deg_s=5.).validate()
        self.assertEqual([p for p in ctx.exception.problems if 'yaw authority' in p],
                         ['yaw authority 4.20 deg/s below 5.00'])
        # SHADOW and calibration configs keep the 2.0 default (they only compute proposals / measure C5)
        shadow = feasible(motor_output_enabled=False, measured_yaw_at_max_diff_deg_s=3., max_yaw_rate_deg_s=1.8,
                          max_differential_pct=6, base_duty_pct=56, reduce_duty_pct=56)
        self.assertEqual(shadow.validate().feasibility()['status'], 'ok')
        self.assertEqual(ExecutorConfig().min_yaw_authority_deg_s, 2.)
        self.assertEqual(C.motor_min_yaw_authority(ExecutorConfig()), C.STEER_AUTHORITY_MIN_DPS)

    def test_bench_motor_output_needs_no_field_measurements(self):
        cfg = ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK, bench_pattern=((2., 40, 40),),
                             motor_output_enabled=True).validate()
        self.assertEqual(cfg.feasibility()['status'], 'unknown')
        with self.assertRaisesRegex(ConfigError, 'duty above'):
            ExecutorConfig(profile='bench', bench_ack=C.BENCH_ACK, bench_pattern=((2., 90, 90),),
                           motor_output_enabled=True).validate()

    def test_shadow_with_unknowns_is_allowed(self):
        cfg = ExecutorConfig(breakaway_duty_pct=None).validate()
        self.assertEqual(cfg.feasibility()['status'], 'unknown')


class GuardConsistencyTest(unittest.TestCase):
    def test_caps_must_cover_controller_output(self):
        with self.assertRaisesRegex(ConfigError, 'duty_cap_pct 70 < base \\+ max_differential = 75'):
            ExecutorConfig(duty_cap_pct=70).validate()
        with self.assertRaisesRegex(ConfigError, 'diff_cap_pct 30 < 2 \\* max_differential = 40'):
            ExecutorConfig(diff_cap_pct=30).validate()

    def test_controller_overrides_allow_list(self):
        ExecutorConfig(controller_overrides={'yaw_kp_pct_per_deg_s': 1.}).validate()
        for name, value in (('pivot_duty_pct', 100), ('pivot_failure_fallback', True), ('turn_enter_deg', 5.),
                            ('control_hz', 10.), ('side_bias_max_m', .15), ('yaw_kp_pct_per_deg_s', 50.),
                            ('side_confirm_scans', 2.5)):
            with self.subTest(name=name), self.assertRaises(ConfigError):
                ExecutorConfig(controller_overrides={name: value}).validate()
        with self.assertRaisesRegex(ConfigError, 'hard < soft < exit'):
            ExecutorConfig(controller_overrides={'side_soft_margin_m': .25}).validate()

    def test_body_mask_bounds(self):
        ExecutorConfig(body_mask=((0, 3239, .35),)).validate()      # inside footprint + 0.05 m on every beam (S5)
        with self.assertRaisesRegex(ConfigError, 'beam range'):
            ExecutorConfig(body_mask=((3000, 3240, .5),)).validate()
        with self.assertRaisesRegex(ConfigError, 'max_range'):
            ExecutorConfig(body_mask=((1, 2, 1.5),)).validate()

    def test_bind_and_socket(self):
        for host in ('0.0.0.0', '::', ''):
            with self.subTest(host=host), self.assertRaisesRegex(ConfigError, 'wildcard'):
                ExecutorConfig(bind_host=host).validate()
        with self.assertRaisesRegex(ConfigError, 'socket_path'):
            ExecutorConfig(socket_path='run/x.sock').validate()


if __name__ == '__main__':
    unittest.main()
