"""Steering firmware mode abs_hold_8 (profile schema v4; docs/steering/ABS_HOLD8_SIM_KO.md).

Checks: the 2026-11-16 profile loads as schema v4 with the measured dimensions, the startup block steer_law exists only
for abs_hold_8 (a V3 profile gives the same startup object as before), the ANGLE -> target counts rule of the real
firmware, and on the T1 rig (native firmware session) the pulse hold itself: PWM 100 pulses of 50 / 80 ms, at least
300 ms released between pulses, the steering EN low between pulses, a standstill step to the soft limit in the time
range of the board test, and no pulses in the v3_servo comparison mode.

Every number is a placeholder-based SIM result (자리값 기반 SIM 결과). unittest style (pytest or python -m unittest). The
native program is built into UBTS_STEER_BUILD_DIR or a temporary folder.
"""
import math
import unittest

from steering import abs_hold
from steering import flat_world as fw
from steering import t1_rig as tr
from steering import vehicle_profile as vp
from steering.native import lockstep_v2 as v2
from steering.scene_spec import WorldSpec

import test_steering_t1_rig as rigtest

NEW_ID = 'front_steer_v1_spec_20261116'
NEW_PATH = vp.PROFILES_DIR / (NEW_ID + '.yaml')


def new_profile():
    return vp.load_profile(NEW_PATH)


class ProfileTest(unittest.TestCase):
    def test_schema_v4_and_measured_values(self):
        p = new_profile()
        self.assertEqual(p.schema, vp.SCHEMA_V4)
        self.assertEqual(p.schema_version, 4)
        self.assertTrue(vp.abs_hold_mode(p))
        for name, value in (('wheelbase_m', .657), ('track_front_m', .527), ('track_rear_m', .527),
                            ('wheel_radius_front_m', .13), ('wheel_radius_rear_m', .13), ('wheel_width_m', .095)):
            self.assertAlmostEqual(p.value(name), value, msg=name)
            self.assertEqual(p.provenance(name), 'MEASURED_20261116', name)
        self.assertEqual(set(p.value('steer_mech_limit_deg').values()), {20.0})
        self.assertAlmostEqual(p.value('rear_axle_x_in_base_m'), -.3285)       # SIM convention: symmetric axles
        self.assertEqual(p.provenance('rear_axle_x_in_base_m'), 'DEFINITION')
        self.assertAlmostEqual(p.value('lidar_x_in_base_m'), .1185)            # 0.21 m behind the front axle
        self.assertAlmostEqual(p.value('rear_axle_x_in_base_m') + p.value('wheelbase_m') - p.value('lidar_x_in_base_m'),
                               .21)
        self.assertAlmostEqual(p.value('lidar_z_m'), .55)                      # top 0.57 - placeholder 0.02
        self.assertEqual(p.provenance('lidar_z_m'), 'ESTIMATED')
        self.assertEqual(p.result_label()['result_kind'], vp.RESULT_LABEL_PLACEHOLDER)

    def test_older_profiles_are_unchanged(self):
        for name in ('front_steer_v0_1_placeholder', 'front_steer_v0_3_placeholder'):
            p = vp.load_profile(vp.PROFILES_DIR / (name + '.yaml'))
            self.assertFalse(vp.abs_hold_mode(p))
            self.assertIsNone(abs_hold.startup_block(p))
            self.assertNotIn('steer_law', v2.firmware_startup_config(p, 0))
            self.assertIsNone(abs_hold.executor_values(p))

    def test_v3_servo_override_has_no_law(self):
        p = new_profile().with_overrides({'steer_fw_mode': 'v3_servo'}, label='v3')
        self.assertIsNone(abs_hold.startup_block(p))
        self.assertNotIn('steer_law', v2.firmware_startup_config(p, 0))

    def test_bad_fw_mode_and_order_are_refused(self):
        with self.assertRaises(vp.ProfileError):
            new_profile().with_overrides({'steer_fw_mode': 'servo'}, label='bad')
        fw = dict(new_profile().value('abs_hold_fw'), restart_tol_counts=5)
        with self.assertRaises(vp.ProfileError):
            new_profile().with_overrides({'abs_hold_fw': fw}, label='bad', allow_out_of_range=True)

    def test_startup_block_and_mapping(self):
        p = new_profile()
        block = abs_hold.startup_block(p)
        self.assertEqual(block['kind'], 'abs_hold_8')
        self.assertEqual((block['cal_right'], block['cal_center'], block['cal_left']), (1681, 1900, 2180))
        self.assertEqual((block['pulse_pwm'], block['pulse_long_ms'], block['pulse_short_ms'], block['settle_ms']),
                         (100, 80, 50, 300))
        self.assertEqual(v2.firmware_startup_config(p, 0)['steer_law'], block)
        # the firmware's absolute targets: CENTER + lroundf(span * |a| / 20)
        for angle, counts in ((20., 1681), (-20., 2180), (0., 1900), (5., 1845), (-5., 1970), (1., 1889), (-1., 1914)):
            self.assertEqual(abs_hold.target_counts(p, angle), counts, angle)
        self.assertAlmostEqual(abs_hold.angle_from_permille(p, 1000), -20.)
        right, left = abs_hold.counts_per_command_deg(p)
        self.assertAlmostEqual(right, 10.95)
        self.assertAlmostEqual(left, 14.0)
        values = abs_hold.executor_values(p)
        mark = abs_hold.mark_angle_deg(p)
        self.assertAlmostEqual(values['rate_deg_s'], round(mark / 3.5, 3))
        self.assertLess(values['rate_deg_s'], 8.)          # below the executor's GO rule placeholder (finding)
        self.assertLessEqual(values['deadband_deg'], 1.)   # target tolerance 10 counts, the right (coarser) side

    def test_native_parser_refuses_a_bad_law_block(self):
        from steering.actuator_link import FirmwareSteeringBackend, NativeBackendError, NativeSteeringBackend
        p = new_profile()
        exe = rigtest.native_path()
        with self.assertRaises(NativeBackendError):          # unknown law kind
            FirmwareSteeringBackend(p, 0, executable=exe, startup_overrides={'steer_law': {'kind': 'servo'}}).close()

        class WithLaw(NativeSteeringBackend):                 # a law block outside firmware mode
            def _startup_config(self, profile, seed):
                config = super()._startup_config(profile, seed)
                config['steer_law'] = abs_hold.startup_block(profile)
                return config
        with self.assertRaises(NativeBackendError):
            WithLaw(p, 0, executable=exe).close()
        ok = FirmwareSteeringBackend(p, 0, executable=exe)   # the profile's own block is accepted
        ok.close()


def step_run(profile, targets, hold_s=5.5):
    """Standstill steps on the T1 rig: arm, then drive 0 with each steering target (mrad) for hold_s."""
    world = WorldSpec.from_dict({'kind': 'W0', 'markers': [list(p) for p in fw.registration_posts()]})
    rig = tr.T1Rig(tr.RigConfig(profile=profile, world=world), native_executable=rigtest.native_path())
    try:
        rig.run_until_true(lambda: rig.core.steering_host()['stop_state'] == 'confirmed', 3.)
        client = rig.client('abs-hold-test')
        assert client.arm_precheck() is None
        assert client.arm().get('ok')
        assert client.wait_armed()
        marks = []
        for target in targets:
            t0 = rig.ms
            end = t0 + int(hold_s * 1000)
            while rig.ms < end:
                reply = client.drive(0, target)
                assert reply.get('ok'), reply
                rig.run_until(min(end, rig.ms + 100) / 1000.)
            marks.append((target, t0, end))
        arrays = rig.trace_arrays()
        pwm = rig.mcu_channel('steer_pwm')
        en = rig.mcu_channel('steer_en')
        limit = int(rig.hp.limit_left_mrad)
    finally:
        rig.close()
    return arrays, pwm, en, marks, limit


def pulses(pwm):
    """[(start tick, end tick, value)] of the non-zero steering PWM runs."""
    out, start, value = [], None, 0
    for tick, v in pwm:
        if v != 0 and start is None:
            start, value = tick, v
        elif v == 0 and start is not None:
            out.append((start, tick, value))
            start = None
    return out


class PulseHoldTest(rigtest.RigCase):
    def test_pulses_release_and_step_time(self):
        from steering import host_profile_export as hpe
        p = new_profile()
        limit = int(hpe.build(p)['steer_limit']['left_mrad'])        # the host soft limit (about 16 deg)
        arrays, pwm, en, marks, _ = step_run(p, [0, limit, 0], hold_s=5.5)
        runs = pulses(pwm)
        self.assertGreaterEqual(len(runs), 8)
        for start, end, value in runs:
            self.assertIn(abs(value), (100,), (start, end, value))           # PWM 100 of 255
            self.assertIn(end - start, range(48, 84), (start, end))           # 50 or 80 ms (2 ms servo grid)
        for (s0, e0, _), (s1, _, _) in zip(runs, runs[1:]):
            self.assertGreaterEqual(s1 - e0, 300, (e0, s1))                   # released >= the 300 ms settle
        # EN low between pulses (released, not the V3 deadband BRAKE)
        en_on = [tick for tick, v in en if v]
        self.assertTrue(all(any(s <= t < e + 3 for s, e, _ in runs) for t in en_on))
        # the step to the left soft limit: within 1 deg of the end value in the board's time range (3.3 .. 4.4 s
        # for 20 command degrees; the soft limit is about 18 command degrees)
        t = arrays['t_s']
        delta = arrays['truth_delta_eq_rad']
        _, t0, t1 = marks[1]
        sel = (t * 1000 >= t0) & (t * 1000 < t1)
        d, ts = delta[sel], t[sel]
        final = d[-1]
        within = next(ts[k] for k in range(len(d)) if all(abs(x - final) <= math.radians(1.) for x in d[k:]))
        self.assertGreater(math.degrees(final), 14.)
        self.assertTrue(2.0 <= within - t0 / 1000. <= 4.5, within - t0 / 1000.)

    def test_v3_servo_mode_has_no_pulses(self):
        p = new_profile().with_overrides({'steer_fw_mode': 'v3_servo'}, label='v3')
        arrays, pwm, en, marks, limit = step_run(p, [0, 150, 0], hold_s=2.)
        runs = pulses(pwm)
        self.assertTrue(runs)
        self.assertTrue(any(abs(v) != 100 for _, _, v in runs))   # the V3 servo law: P output, not fixed pulses


if __name__ == '__main__':
    unittest.main()
