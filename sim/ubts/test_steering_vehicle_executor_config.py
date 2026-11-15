"""The vehicle executor configuration of the Jetson kit (steering.vehicle_executor_config; docs/steering/EXECUTOR_ESP32_KO.md).

Checks: the committed file tools/row_executor/config.vehicle_esp32_shadow.json is the generator's output for the profile
front_steer_v1_spec_20261116; it is a SHADOW front_steer single-row configuration for the ESP32-S3 bridge on the vehicle
socket with the operator page on 127.0.0.1; the measured values are the profile's; every missing field measurement is
null (motor output refused); it builds a SHADOW executor loop that never creates a bridge transport.
unittest style (pytest or python -m unittest).
"""
import unittest
from dataclasses import replace

from steering import abs_hold
from steering import vehicle_executor_config as V

COMMITTED = V.REPO / 'tools' / 'row_executor' / 'config.vehicle_esp32_shadow.json'


class VehicleExecutorConfig(unittest.TestCase):
    def test_committed_file_is_the_generator_output(self):
        self.assertEqual(COMMITTED.read_bytes(), V.render().encode('utf-8'))

    def test_values(self):
        profile = V.load()
        cfg = V.vehicle_config(profile)
        self.assertEqual((cfg.vehicle_profile, cfg.mode, cfg.motor_output_enabled, cfg.steer_bridge_kind),
                         ('front_steer', 'single_row', False, 'esp32_abs_hold'))
        self.assertEqual((cfg.socket_path, cfg.bind_host, cfg.bind_port), ('/run/ubts-rover/control.sock', '127.0.0.1', 8097))
        self.assertEqual(cfg.steer_wheelbase_m, profile.value('wheelbase_m'))
        self.assertEqual(cfg.steer_esp32_cal, (1681, 1900, 2180, 20))
        self.assertAlmostEqual(cfg.steer_esp32_mark_deg, abs_hold.mark_angle_deg(profile), places=3)
        self.assertEqual((cfg.steer_limit_left_mrad, cfg.steer_limit_right_mrad), (281, 281))
        self.assertAlmostEqual(cfg.steer_rate_limit_deg_s, .8 * abs_hold.executor_values(profile)['rate_deg_s'], places=3)
        for name in ('breakaway_duty_pct', 'v_measured_m_s', 'measured_stop_distance_m', 'steer_measured_rate_deg_s',
                     'steer_wheel_soft_margin_deg', 'steer_min_pwm_pct', 'steer_zero_offset_deg'):
            self.assertIsNone(getattr(cfg, name), name)
        self.assertFalse(cfg.steer_polarity_checked or cfg.steer_range_measured)
        self.assertTrue(cfg.steer_esp32_require_fw_watchdog)
        self.assertEqual(cfg.feasibility()['status'], 'unknown')
        XC = V._row_executor_config()
        with self.assertRaises(XC.ConfigError):
            replace(cfg, motor_output_enabled=True).validate()

    def test_shadow_loop_has_no_bridge_transport(self):
        import sys
        V._row_executor_config()                    # tools/ on the path
        from row_executor import executor_main as M
        cfg = replace(V.vehicle_config(), tuning_file=None)
        loop = M.ExecutorLoop(cfg, motor_output_flag=False)
        self.assertFalse(loop.motor_allowed)
        self.assertIsNone(loop._stop_transport)
        self.assertIsNone(loop.esp32)
        self.assertEqual(loop._req_arm(0.)['error'], 'motor_output_not_allowed')
        self.assertIn('row_executor', sys.modules)


if __name__ == '__main__':
    unittest.main()
