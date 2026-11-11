"""Generated, bounded software loop; no hardware, ROS, network, SDK or field replay.

The nominal low-duty calibration is an explicit SIM assumption. The six-second
case exercises the actual executor and bridge with a FakeMotor, not a controller
mock. It is a wiring/safety test, not a full-row completion or fresh holdout.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import unittest
from dataclasses import replace

REPO = Path(__file__).resolve().parents[3]
for directory in (REPO / 'tools', REPO / 'sim/ubts'):
    sys.path.insert(0, str(directory))
sys.dont_write_bytecode = True

from field_response import FieldResponseParams
from row_executor import sim_in_loop as S
from row_executor.bridge_client import LATCHED, BridgeClient, BridgeError
from closed_loop.harness import EpisodeConfig
from closed_loop.world import WorldConfig


class ProfileContract(unittest.TestCase):
    def test_field1010_keeps_all_default_coefficients(self):
        selected = S.plant_spec('field1010')
        self.assertEqual(selected['overrides'], ())
        self.assertIsNone(selected['stiction'])
        ep = EpisodeConfig(plant_overrides=selected['overrides'])
        self.assertEqual(S.SH.plant_params(ep), FieldResponseParams())
        self.assertEqual(S.SH.plant_params(ep).forward_deadband_pct, 50.)
        self.assertEqual(S.SH.plant_params(ep).pivot_min_duty_pct, 80.)

    def test_old_1003_baseline_is_preserved(self):
        p = S.SH.plant_params(EpisodeConfig(plant_overrides=S.plant_spec('p_fit1003')['overrides']))
        self.assertEqual(p.forward_deadband_pct, 5.)
        self.assertEqual(p.pivot_min_duty_pct, 55.)
        self.assertEqual(p.speed_at_90_m_s, .80)

    def test_nominal_calibration_does_not_reuse_1003_speed_or_yaw(self):
        cfg = S.executor_config(plant_name='field1010')
        self.assertAlmostEqual(cfg.v_measured_m_s, .09)
        self.assertAlmostEqual(cfg.measured_yaw_at_max_diff_deg_s, 4.8)
        self.assertAlmostEqual(cfg.max_yaw_rate_deg_s, 2.88)
        self.assertEqual(cfg.arc_yaw_deg_s_per_pct_diff, .12)
        self.assertEqual(S.executor_config().v_measured_m_s, .47)

    def test_low_differential_keeps_original_steering_refusal(self):
        with self.assertRaisesRegex(ValueError, 'yaw authority'):
            S.executor_config({'max_differential_pct': 10}, plant_name='field1010')

    def test_old_C7_value_cannot_be_silently_reused_at_new_speed(self):
        with self.assertRaisesRegex(ValueError, 'C7'):
            S.executor_config({'measured_stop_distance_m': .25}, plant_name='field1010')

    def test_sensitivity_is_explicit_and_does_not_rewrite_nominal(self):
        p = S.plant_spec('field1010', extra=(('forward_deadband_pct', 30.),))
        self.assertEqual(p['overrides'], (('forward_deadband_pct', 30.),))
        self.assertEqual(FieldResponseParams().forward_deadband_pct, 50.)


class ActualSoftwareLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cfg = S.executor_config(plant_name='field1010')
        ep = EpisodeConfig(name='generated_field10_wiring', seed=4001,
            world=WorldConfig(seed=4001, branches=False, missing_probability=0.),
            start_yaw_deg=3., start_y_m=.05, max_time_s=6.,
            plant_overrides=S.plant_spec('field1010')['overrides'],
            controller_overrides=tuple(sorted(cfg.controller_settings()[1].items())))
        # Operator-entered mission length is fixed, not copied into localization.
        cls.facade = S.ExecutorInTheLoop(ep, cfg, 30., S.RealFormat(cfg.lidar_yaw_offset_deg), S.rover_control_class())
        try:
            cls.result = S.SH.run_episode(ep, record_trace=True,
                estimator_factory=lambda _cfg: cls.facade, controller_factory=lambda _cfg: cls.facade)
            cls.summary = cls.facade.summary()
            cls.before_stop = cls.facade.motor.duty
            cls.counts = dict(cls.facade.loop.counts)
            # A second client must not steal or stop the existing simulated lease.
            cls.owner_stop_before = cls.facade.transport.ops['stop']
            intruder = BridgeClient(owner='generated-other-owner', transport=cls.facade.transport)
            try:
                intruder.arm()
                cls.other_owner_refused = False
            except BridgeError:
                cls.other_owner_refused = True
            cls.owner_duty_after = cls.facade.motor.duty
            cls.owner_stop_after = cls.facade.transport.ops['stop']
            # No new sensor input is delivered. The real loop detects the gap.
            cls.facade.now += .45
            cls.stale_decision = cls.facade.loop.step(cls.facade.now)
            cls.after_stale = cls.facade.motor.duty
            cls.stop_reply = cls.facade.loop.emergency_stop('generated_test')
            cls.after_stop = cls.facade.motor.duty
            # The HTTP-thread equivalent sends STOP immediately; the original
            # main loop consumes the Stop event and latches on its next step.
            cls.facade.now += .05
            cls.facade.loop.step(cls.facade.now)
            cls.Stop_mainstep_latched = cls.facade.loop.bridge.state == LATCHED
        finally:
            cls.facade.loop.shutdown('generated_test_shutdown')

    def test_real_executor_receives_scans_and_imu_and_sends_bridge_duty(self):
        self.assertGreaterEqual(self.summary['scans_processed'], 30)
        self.assertGreaterEqual(self.counts['imu_fed'], 60)
        self.assertGreater(self.summary['sends'], 0)
        self.assertIsNotNone(self.summary['first_drive_t'])
        self.assertEqual(self.summary['bridge_errors'], 0)
        self.assertEqual(self.summary['deadman_false'], 0)
        self.assertEqual(self.summary['guard_violations'], 0)

    def test_actuator_motion_produces_sensor_estimated_motion(self):
        self.assertGreater(self.result['truth_distance_m'], .02)
        self.assertGreater(self.result['estimated_distance_m'], .02)
        self.assertGreater(self.result['progress_m'], .02)
        self.assertEqual(self.result['collisions'], 0)
        self.assertFalse(self.result['left_corridor'])

    def test_controller_reacts_in_the_closed_loop_not_fixed_replay(self):
        commands = {(row['left'], row['right']) for row in self.result['trace']}
        self.assertGreater(len(commands), 2)
        self.assertGreater(self.summary['max_abs_diff_sent'], 0)
        self.assertGreater(self.facade.transport.ops.get('drive', 0), 0)

    def test_original_stop_latches_and_clears_fake_actuator(self):
        self.assertEqual(self.after_stop, (0, 0))
        self.assertTrue(self.Stop_mainstep_latched)
        self.assertGreater(self.facade.motor.hard_stops, 0)

    def test_original_stale_or_stall_guard_stops_after_sensor_gap(self):
        self.assertNotEqual(self.before_stop, (0, 0))
        self.assertEqual(self.after_stale, (0, 0))
        self.assertEqual((self.stale_decision.left, self.stale_decision.right), (0, 0))

    def test_other_owner_cannot_steal_or_stop_existing_loop(self):
        self.assertTrue(self.other_owner_refused)
        self.assertEqual(self.owner_duty_after, self.before_stop)
        self.assertEqual(self.owner_stop_before, self.owner_stop_after)


if __name__ == '__main__':
    unittest.main(verbosity=2)
