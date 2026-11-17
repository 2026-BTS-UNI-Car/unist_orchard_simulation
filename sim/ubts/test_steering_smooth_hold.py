"""Steering firmware mode smooth_r5 (profile schema v5; docs/steering/SMOOTH_R5_SIM_KO.md).

Checks: the field profile front_steer_v2_field_20261116 loads as schema v5 with the board's CAL and SM values and
differs from front_steer_v1_spec_20261116 only in the listed records; smooth_r5 needs a v5 file (a v4 file still allows only
v3_servo and abs_hold_8); the startup block steer_law (kind smooth_r5) and the firmware's float target rule; the
executor values (the measured SMOOTH step response); the native parser refuses a bad smooth block; and on the T1 rig
(native firmware session) the SMOOTH hold itself: one continuous run per step with PWM 60..80 (ramp, slow zone), the
quiet read (EN low one servo period every 16 ms), the 40 ms brake (EN high, PWM 0) before the driver is released, the
step time to the soft limit, and the comparison modes (abs_hold_8 pulses of PWM 100, v3_servo P output) on the same
profile.

Every number is a placeholder-based SIM result (자리값 기반 SIM 결과). unittest style (pytest or python -m unittest). The
native program is built into UBTS_STEER_BUILD_DIR or a temporary folder.
"""
import math
import unittest

from steering import abs_hold
from steering import smooth_hold
from steering import vehicle_profile as vp
from steering.native import lockstep_v2 as v2

import test_steering_abs_hold as abstest
import test_steering_t1_rig as rigtest

NEW_ID = 'front_steer_v2_field_20261116'
NEW_PATH = vp.PROFILES_DIR / (NEW_ID + '.yaml')
V1_PATH = vp.PROFILES_DIR / 'front_steer_v1_spec_20261116.yaml'
CHANGED = {'servo_hz', 'steer_fw_mode', 'abs_hold_fw', 'steer_step_time_measured_s', 'steer_rate_max_deg_s',
           'steer_rate_standstill_deg_s'}
NEW_RECORDS = {'smooth_fw', 'smooth_host', 'smooth_adc_sim', 'smooth_step_time_measured_s'}


def new_profile():
    return vp.load_profile(NEW_PATH)


class ProfileTest(unittest.TestCase):
    def test_schema_v5_and_field_values(self):
        p = new_profile()
        self.assertEqual(p.schema, vp.SCHEMA_V5)
        self.assertEqual(p.schema_version, 5)
        self.assertTrue(vp.smooth_hold_mode(p))
        self.assertFalse(vp.abs_hold_mode(p))
        self.assertTrue(smooth_hold.active(p))
        fw = p.value('smooth_fw')
        self.assertEqual((fw['cal_right'], fw['cal_center'], fw['cal_left']), (1475, 2017, 2513))   # board NVS CAL
        sm = dict(pwm_min=60, pwm_max=80, slow_counts=200, stop_tol_counts=6, restart_tol_counts=16, ramp_per_tick=12,
                  settle_ms=120, stall_ms=200, boost=15, max_run_ms=5000, brake_ms=40, quiet_ms=15, quiet_us=1500)
        self.assertEqual({k: fw[k] for k in sm}, sm)                                       # SM line saved in NVS
        self.assertEqual(p.value('smooth_host'), dict(step_deg=.25, deadband_deg=.25, min_interval_ms=200, poll_ms=200))
        self.assertEqual(set(p.value('smooth_adc_sim').values()), {0})                      # placeholders off
        self.assertEqual(p.value('smooth_step_time_measured_s'), dict(outward_20=1.439, return_20=1.087))
        self.assertEqual(p.provenance('smooth_step_time_measured_s'), 'MEASURED_20261116')
        self.assertEqual(p.value('servo_hz'), 1000)
        self.assertAlmostEqual(p.value('steer_rate_max_deg_s'), 80.)                        # fitted to the r5 steps
        self.assertEqual(p.provenance('steer_rate_max_deg_s'), 'ESTIMATED')
        self.assertEqual(p.result_label()['result_kind'], vp.RESULT_LABEL_PLACEHOLDER)
        self.assertEqual(p.warnings(), [])

    def test_v2_differs_from_v1_only_in_these_records(self):
        new, old = new_profile(), vp.load_profile(V1_PATH)
        self.assertEqual(set(new.names()) - set(old.names()), NEW_RECORDS)
        self.assertEqual(set(old.names()) - set(new.names()), set())
        differ = {name for name in old.names() if new.param(name) != old.param(name)}
        self.assertEqual(differ, CHANGED)
        pulse_new, pulse_old = new.value('abs_hold_fw'), old.value('abs_hold_fw')
        self.assertEqual({k for k in pulse_old if pulse_new[k] != pulse_old[k]},
                         {'cal_right', 'cal_center', 'cal_left'})
        self.assertEqual(new.value('steer_step_time_measured_s'), old.value('steer_step_time_measured_s'))

    def test_older_profiles_are_unchanged(self):
        for name in ('front_steer_v0_1_placeholder', 'front_steer_v0_3_placeholder', 'front_steer_v1_spec_20261116'):
            p = vp.load_profile(vp.PROFILES_DIR / (name + '.yaml'))
            self.assertFalse(vp.smooth_hold_mode(p))
            self.assertIsNone(smooth_hold.startup_block(p))
            law = v2.firmware_startup_config(p, 0).get('steer_law')
            self.assertEqual(law, abs_hold.startup_block(p))       # the pulse block (v1) or nothing (v0.x)
            self.assertEqual(abs_hold.executor_values(p) is None, not vp.abs_hold_mode(p))

    def test_smooth_mode_needs_schema_v5(self):
        old = vp.load_profile(V1_PATH)
        with self.assertRaises(vp.ProfileError):
            old.with_overrides({'steer_fw_mode': 'smooth_r5'}, label='bad', allow_out_of_range=True)
        with self.assertRaises(vp.ProfileError):
            new_profile().with_overrides({'steer_fw_mode': 'servo'}, label='bad', allow_out_of_range=True)

    def test_firmware_rules_are_refused_when_broken(self):
        base = new_profile()
        fw = base.value('smooth_fw')
        for change in (dict(pwm_min=90), dict(restart_tol_counts=6), dict(stop_tol_counts=2), dict(median_of=4),
                       dict(cal_left=2600), dict(quiet_ms=51)):
            with self.assertRaises(vp.ProfileError, msg=change):
                base.with_overrides({'smooth_fw': dict(fw, **change)}, label='bad', allow_out_of_range=True)
        with self.assertRaises(vp.ProfileError):   # bursts need a duration inside the period
            base.with_overrides({'smooth_adc_sim': dict(base.value('smooth_adc_sim'), burst_counts=110)}, label='bad',
                                allow_out_of_range=True)
        ok = base.with_overrides({'smooth_adc_sim': dict(powered_offset_counts=50, burst_counts=110, burst_ms=20,
                                                         burst_period_ms=170)}, label='bursts')
        self.assertEqual(smooth_hold.startup_block(ok)['adc_burst_period_ms'], 170)

    def test_mode_overrides_on_the_same_profile(self):
        p = new_profile()
        pulse = p.with_overrides({'steer_fw_mode': 'abs_hold_8'}, label='pulse')
        self.assertTrue(vp.abs_hold_mode(pulse))
        block = v2.firmware_startup_config(pulse, 0)['steer_law']
        self.assertEqual((block['kind'], block['cal_right'], block['cal_center'], block['cal_left']),
                         ('abs_hold_8', 1475, 2017, 2513))
        servo = p.with_overrides({'steer_fw_mode': 'v3_servo'}, label='v3')
        self.assertNotIn('steer_law', v2.firmware_startup_config(servo, 0))
        self.assertIsNone(abs_hold.executor_values(servo))

    def test_startup_block_and_mapping(self):
        p = new_profile()
        block = smooth_hold.startup_block(p)
        self.assertEqual(block['kind'], 'smooth_r5')
        self.assertEqual(len(block), 1 + len(vp.SMOOTH_FW_KEYS) + 4 + 4)
        self.assertEqual(v2.firmware_startup_config(p, 0)['steer_law'], block)
        self.assertEqual((block['host_step_deg'], block['host_min_interval_ms']), (.25, 200))
        # moveSteeringToAngle: CENTER + lroundf(span * |a| / 20.0f), span LEFT - CENTER for a < 0
        for angle, counts in ((20., 1475), (-20., 2513), (0., 2017), (.25, 2010), (-.25, 2023), (5., 1881),
                              (-5., 2141), (12.75, 1671), (-10., 2265), (-18.75, 2482)):
            self.assertEqual(smooth_hold.target_counts(p, angle), counts, angle)
        self.assertEqual(smooth_hold.quantize_command(-12.3, .25), -12.25)
        self.assertEqual(smooth_hold.quantize_command(.125, .25), .25)       # half away from zero (the bridge)
        self.assertEqual(smooth_hold.quantize_command(-.125, .25), -.25)
        self.assertEqual(smooth_hold.quantize_command(25., .25), 20.)
        right, left = smooth_hold.counts_per_command_deg(p)
        self.assertAlmostEqual(right, 27.1)
        self.assertAlmostEqual(left, 24.8)

    def test_executor_values(self):
        p = new_profile()
        values = smooth_hold.executor_values(p)
        self.assertEqual(abs_hold.executor_values(p), values)              # the callers' entry point
        mark = abs_hold.mark_angle_deg(p)
        self.assertAlmostEqual(values['rate_deg_s'], math.floor(mark / 1.439 * 100.) / 100.)   # 12.39
        self.assertLessEqual(round(.8 * values['rate_deg_s'], 3), .8 * values['rate_deg_s'] + 1e-9)   # design 6.2
        self.assertGreaterEqual(values['rate_deg_s'], 8.)                  # passes the executor's GO rule placeholder
        self.assertAlmostEqual(values['deadband_deg'], round(6 / 24.8 * mark / 20., 3))   # STOP_TOL, left (coarser)
        self.assertAlmostEqual(values['restart_deg'], round(16 / 24.8 * mark / 20., 3))

    def test_native_parser_refuses_a_bad_smooth_block(self):
        from steering.actuator_link import FirmwareSteeringBackend, NativeBackendError
        p = new_profile()
        exe = rigtest.native_path()
        for patch in ({'pwm_min': 120, 'pwm_max': 100}, {'restart_tol_counts': 6}, {'quiet_ms': 51},
                      {'median_of': 4}, {'adc_burst_counts': 50}, {'surprise': 1}, {'cal_left': 2700}):
            with self.assertRaises(NativeBackendError, msg=patch):
                FirmwareSteeringBackend(p, 0, executable=exe, startup_overrides={'steer_law': patch}).close()
        ok = FirmwareSteeringBackend(p, 0, executable=exe)   # the profile's own block is accepted
        ok.close()


def merged_runs(pwm, gap=1):
    """[(start tick, end tick, sign)] of the non-zero steering PWM runs, quiet gaps of <= gap ticks merged."""
    out = []
    for start, end, value in abstest.pulses(pwm):
        sign = 1 if value > 0 else -1
        if out and start - out[-1][1] <= gap and out[-1][2] == sign:
            out[-1][1] = end
        else:
            out.append([start, end, sign])
    return [tuple(r) for r in out]


class SmoothHoldTest(rigtest.RigCase):
    def test_continuous_run_quiet_reads_and_brake(self):
        from steering import host_profile_export as hpe
        p = new_profile()
        limit = int(hpe.build(p)['steer_limit']['left_mrad'])        # the host soft limit (about 16 deg)
        arrays, pwm, en, marks, _ = abstest.step_run(p, [0, limit, 0], hold_s=3.)
        values = {abs(v) for _, v in pwm if v}
        self.assertTrue(values)
        self.assertLessEqual(max(values), 80)                         # PWM_MAX 80 of 255
        self.assertGreaterEqual(min(values), 60)                      # the floor PWM_MIN 60
        runs = abstest.pulses(pwm)
        # quiet reads: within a run the driver goes off for one servo period (1 ms) every 16 ms
        long_run = [r for r in merged_runs(pwm) if r[1] - r[0] > 300]
        self.assertTrue(long_run, merged_runs(pwm))
        s, e, _ = long_run[0]
        inside = [(a, b) for a, b, _ in runs if s <= a and b <= e]
        gaps = [b2 - a1 for (_, a1), (b2, _) in zip(inside, inside[1:])]
        self.assertEqual(set(gaps), {1})
        periods = [b[0] - a[0] for a, b in zip(inside[1:], inside[2:])]
        self.assertEqual(set(periods), {16})
        # ramp: PWM_MIN 60 at the start, +12 per 10 ms (60, 72, 80)
        first = [v for t, v in pwm if s <= t < s + 25 and v]
        self.assertEqual([abs(v) for v in first[:3]], [60, 72, 72])
        # the brake: after the last run EN stays high with both PWM 0 for BRAKE_MS 40, then EN low. A run that ends on a
        # quiet read ends one servo period after its last PWM tick (EN low for the read, then high for the brake).
        en_after = [(t, v) for t, v in en if t >= e and t < e + 100]
        brake_start = e
        if en_after[0] == (e, 0):          # the last PWM tick was followed by the quiet read's EN low
            self.assertEqual(en_after[1], (e + 1, 1), en_after)
            brake_start = e + 1
            en_after = en_after[2:]
        self.assertEqual(en_after[0], (brake_start + 40, 0), en_after)
        self.assertFalse([t for t, v in pwm if brake_start <= t < brake_start + 40 and v])
        # the step to the left soft limit: one continuous run, within 1 deg of the end value in 0.6 .. 1.6 s
        t = arrays['t_s']
        delta = arrays['truth_delta_eq_rad']
        _, t0, t1 = marks[1]
        sel = (t * 1000 >= t0) & (t * 1000 < t1)
        d, ts = delta[sel], t[sel]
        final = d[-1]
        within = next(ts[k] for k in range(len(d)) if all(abs(x - final) <= math.radians(1.) for x in d[k:]))
        self.assertGreater(math.degrees(final), 14.)
        self.assertTrue(.6 <= within - t0 / 1000. <= 1.6, within - t0 / 1000.)
        step_runs = [r for r in merged_runs(pwm) if t0 <= r[0] < t1]
        self.assertEqual(step_runs[0][2], 1)                          # toward + permille (left)
        self.assertGreater(step_runs[0][1] - step_runs[0][0], 500)    # the move is one run (no pulses)

    def test_pulse_mode_on_the_same_profile(self):
        p = new_profile().with_overrides({'steer_fw_mode': 'abs_hold_8'}, label='pulse')
        arrays, pwm, en, marks, limit = abstest.step_run(p, [0, 150, 0], hold_s=2.5)
        runs = abstest.pulses(pwm)
        self.assertTrue(runs)
        self.assertEqual({abs(v) for _, _, v in runs}, {100})        # the pulse hold: PWM 100 pulses

    def test_v3_servo_mode_on_the_same_profile(self):
        p = new_profile().with_overrides({'steer_fw_mode': 'v3_servo'}, label='v3')
        arrays, pwm, en, marks, limit = abstest.step_run(p, [0, 150, 0], hold_s=2.)
        runs = abstest.pulses(pwm)
        self.assertTrue(runs)
        self.assertTrue(any(abs(v) not in (60, 72, 80, 100) for _, v in pwm if v))   # the V3 P output


if __name__ == '__main__':
    unittest.main()
