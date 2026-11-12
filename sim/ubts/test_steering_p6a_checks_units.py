"""Unit tests of the WF-4 g_gaps additions to the P6a checks, without ROS or a stack.

  run scope (G06)            status files of an earlier run are refused (READY stamp time)
  SIM-made attribution       on the monotonic clock when the stall has a monotonic end (the VM's realtime is stepped)
  host latch timing (G09)    status gap, not following, N line and banner rules on synthetic bridge / emulator logs
  A19(G) summary             gz_probe results marked normal_script false are not counted; provenance; checks missing
  A3(G) noise helpers        raw series from a record, the KS distance
  stall probe                /proc parsing helpers (no /proc needed)
  sensor contract            the gps_link static TF and the virtual GPS antenna from the profile (G20); the S2 laser
                             frame check skips with a reason when s2_format is missing (G14)
  IMU model F/G (G15)        the G-SIM IMU error model draws like the F-SIM plant (gyro noise sd, vibration rule,
                             bias and scale distributions)
unittest style (pytest on 3.12 / 3.13, python3 -m unittest on 3.10). No file in the work tree. Placeholder-based SIM
results only.
"""
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import time
import types
import unittest

from steering import p6a_checks as checks
from steering import p6a_stall_probe as probe
from steering import sensor_contract_steer as contract

SIM_ROOT = Path(__file__).resolve().parent


def profile():
    from steering.vehicle_profile import load_profile
    return load_profile()


class RunScopeTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix='steer_g_scope_'))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_files_of_an_earlier_run_are_refused(self):
        stamp = self.tmp / 'READY'
        stamp.write_text(json.dumps(dict(token='abc', written_at=1000., scene='w1_force')), encoding='utf-8')
        scope = checks.run_scope(str(stamp))
        self.assertEqual(scope['token'], 'abc')
        old = self.tmp / 'scan_clip_status.json'
        old.write_text(json.dumps(dict(updated_at=900., scans=890)), encoding='utf-8')
        data, note = checks.scoped_json(str(old), scope)
        self.assertIsNone(data)
        self.assertIn('stale', note)
        old.write_text(json.dumps(dict(updated_at=1001., scans=3)), encoding='utf-8')
        data, note = checks.scoped_json(str(old), scope)
        self.assertEqual(data['scans'], 3)
        self.assertEqual(note, 'current run')
        data, note = checks.scoped_json(str(self.tmp / 'missing.json'), scope)
        self.assertIsNone(data)
        self.assertEqual(note, 'missing')
        self.assertIsNone(checks.run_scope(str(self.tmp / 'no_stamp')))
        data, note = checks.scoped_json(str(old), dict(token=None, written_at=None))
        self.assertIsNotNone(data)
        self.assertIn('taken as is', note)


def state(wall, t, latches, reason=None):
    return dict(kind='state', wall=wall, t=t, latches=list(latches), reason=reason)


class AttributionTest(unittest.TestCase):
    def test_monotonic_attribution_survives_a_realtime_step(self):
        # the emulator stall ended at monotonic 500.0 (wall 2000.0); the latch is 0.4 s later on the monotonic clock,
        # but the VM's realtime was stepped by +1.7 s in between: the wall rule would not attribute it
        bridge = [state(2000.5, 499.0, []), state(2002.1, 500.4, ['steering_status_stale'])]
        stalls = [(2000.0, .34, 500.0)]
        out = checks.false_detections([], bridge, after_wall=2000.0, stalls=stalls, clock='wall')
        self.assertEqual(out['count'], 0)
        self.assertEqual(out['sim_stall_latches'][0]['time_base'], 'monotonic')
        # without the monotonic end (an older status file) the wall rule applies and counts it
        out = checks.false_detections([], bridge, after_wall=2000.0, stalls=[(2000.0, .34)], clock='wall')
        self.assertEqual(out['count'], 1)
        # a latch far from the stall is a false detection
        bridge = [state(2000.5, 499.0, []), state(2010., 509., ['steering_status_stale'])]
        out = checks.false_detections([], bridge, after_wall=2000.0, stalls=stalls, clock='wall')
        self.assertEqual(out['by_code'], {'host:steering_status_stale': 1})


class FakeObs:
    def __init__(self, states):
        self.steering_states = [(0., s) for s in states]


class HostTimingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix='steer_g_host_'))
        self.saved = (checks.BRIDGE_LOG, checks.EMULATOR_LOG, checks.HOST_PROFILE)
        checks.BRIDGE_LOG = str(self.tmp / 'bridge.jsonl')
        checks.EMULATOR_LOG = str(self.tmp / 'emulator.jsonl')
        checks.HOST_PROFILE = str(self.tmp / 'host_profile.json')
        (self.tmp / 'host_profile.json').write_text(json.dumps(dict(host_faults=dict(
            status_gap_ms=150, track_error_deg=3., track_duration_s=1.5))), encoding='utf-8')

    def tearDown(self):
        checks.BRIDGE_LOG, checks.EMULATOR_LOG, checks.HOST_PROFILE = self.saved
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, name, rows):
        (self.tmp / name).write_text(''.join(json.dumps(r) + '\n' for r in rows), encoding='utf-8')

    def case(self):
        return dict(name='x', criteria=[], mark=dict(wall=100., tick=0, session=1), validity=dict(valid=True))

    def ctx(self, states, clock='wall'):
        return types.SimpleNamespace(obs=FakeObs(states), clock=clock)

    def test_status_gap(self):
        self.write('bridge.jsonl', [state(100.1, 10.0, []), state(100.5, 10.37, ['steering_status_stale'])])
        states = [dict(received_monotonic=10.0 + .05 * k) for k in range(5)]      # last line at 10.20
        case = self.case()
        checks.host_timing(self.ctx(states), case, 'steering_status_stale', 'status_gap')
        self.assertTrue(case['passed'], case['criteria'])
        self.assertAlmostEqual(case['host_timing']['latch_minus_reference_s'], .02, places=6)
        # a latch before the gap is over is wrong
        self.write('bridge.jsonl', [state(100.1, 10.0, []), state(100.5, 10.33, ['steering_status_stale'])])
        case = self.case()
        checks.host_timing(self.ctx(states), case, 'steering_status_stale', 'status_gap')
        self.assertFalse(case['passed'])

    def test_not_following(self):
        rows = []
        for k in range(60):                                  # every 50 ms; the target steps at line 10
            target = 300 if k >= 10 else 0
            # cmd_permille == target_permille: the MCU servo ramp has reached its goal (amendment 43)
            rows.append(dict(received_monotonic=20. + .05 * k, target_mrad=target, angle_mrad=10. + k, state='ARMED',
                             target_permille=target, cmd_permille=target))
        since = 20. + .05 * 10
        expected = min(r['received_monotonic'] for r in rows if r['received_monotonic'] - since >= 1.5 - 1e-9)
        self.write('bridge.jsonl', [state(100.2, 20.1, []), state(102., expected + .004, ['steering_not_following'])])
        case = self.case()
        checks.host_timing(self.ctx(rows), case, 'steering_not_following', 'not_following')
        self.assertTrue(case['passed'], case['criteria'])
        self.assertAlmostEqual(case['host_timing']['reference_t'], expected, places=6)
        # the real wrapper publishes the latching line after the stop of the latch: target_mrad None on that line and
        # after it, state DISARMED later (ST-04: the reference was not found on stack data)
        real = [dict(r) for r in rows]
        for r in real:
            if r['received_monotonic'] >= expected - 1e-9:
                r['target_mrad'] = None
        case = self.case()
        checks.host_timing(self.ctx(real), case, 'steering_not_following', 'not_following')
        self.assertTrue(case['passed'], case['criteria'])
        self.assertAlmostEqual(case['host_timing']['reference_t'], expected, places=6)
        # the error clears for one line before the duration is over: the rule time starts again (a latch at the first
        # time is then early: the criterion fails)
        cleared = [dict(r) for r in real]
        cleared[25]['angle_mrad'] = 299.                      # 0.75 s after the step: inside the error band
        case = self.case()
        checks.host_timing(self.ctx(cleared), case, 'steering_not_following', 'not_following')
        self.assertFalse(case['passed'])
        # the request ends (target None) before the duration: no rule time at all
        ended = [dict(r) for r in rows]
        for r in ended[30:]:
            r['target_mrad'] = None
        case = self.case()
        checks.host_timing(self.ctx(ended), case, 'steering_not_following', 'not_following')
        self.assertFalse(case['passed'])
        self.assertIsNone(case['host_timing'].get('reference_t'))
        # amendment 43: while the MCU ramp slews (cmd != tgt, lines 10..19) the bridge does not count; the rule time
        # starts at line 20, so the reference moves by 0.5 s
        slewing = [dict(r) for r in rows]
        for r in slewing[10:20]:
            r['cmd_permille'] = r['target_permille'] - 100
        later = min(r['received_monotonic'] for r in rows if r['received_monotonic'] - (20. + .05 * 20) >= 1.5 - 1e-9)
        self.write('bridge.jsonl', [state(100.2, 20.1, []), state(102.5, later + .004, ['steering_not_following'])])
        case = self.case()
        checks.host_timing(self.ctx(slewing), case, 'steering_not_following', 'not_following')
        self.assertTrue(case['passed'], case['criteria'])
        self.assertAlmostEqual(case['host_timing']['reference_t'], later, places=6)

    def test_n_line_and_banner_in_both_clocks(self):
        self.write('emulator.jsonl', [dict(kind='m2h', session=1, tick=500, done=501, t=5.5, mono=30.0,
                                           text='N 7 range*00'),
                                      dict(kind='m2h', session=1, tick=900, done=905, t=5.9, mono=30.4,
                                           text='UBTS_STEER_V3 hello fw=x*00')])
        self.write('bridge.jsonl', [state(100.1, 29.9, []), state(100.6, 30.011, ['c_rejected'])])
        case = self.case()
        checks.host_timing(self.ctx([]), case, 'c_rejected', 'n_line')
        self.assertTrue(case['passed'], case['criteria'])
        # SIM clock: the line time is the SIM time of its LF tick
        self.write('bridge.jsonl', [state(100.1, 5.0, []), state(100.6, 5.915, ['mcu_reset_detected'])])
        case = self.case()
        checks.host_timing(self.ctx([], clock='sim'), case, 'mcu_reset_detected', 'banner')
        self.assertTrue(case['passed'], case['criteria'])
        self.assertAlmostEqual(case['host_timing']['reference_t'], 5.905, places=6)
        # no latch: a failed criterion
        self.write('bridge.jsonl', [state(100.1, 5.0, [])])
        case = self.case()
        checks.host_timing(self.ctx([]), case, 'c_rejected', 'n_line')
        self.assertFalse(case['passed'])

    def test_invalid_step_is_not_judged(self):
        self.write('bridge.jsonl', [state(100.1, 10.0, []), state(100.5, 10.33, ['steering_status_stale'])])
        case = self.case()
        case['validity'] = dict(valid=False, reason='RTF 0.5 below 0.9 (A10)')
        checks.host_timing(self.ctx([dict(received_monotonic=10.2)]), case, 'steering_status_stale', 'status_gap')
        self.assertIsNone(case['criteria'][-1]['passed'])
        self.assertTrue(case['criteria'][-1]['invalid'])


class NormalSummaryTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix='steer_g_sum_'))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, name, data):
        path = self.tmp / name
        path.write_text(json.dumps(data), encoding='utf-8')
        return str(path)

    def test_not_normal_groups_are_not_counted(self):
        startup = self.write('startup.json', dict(check='A19_G_startup', false_detection=dict(count=0, by_code={}),
                                                  emulator_clock=dict(sim_s=1.), source_commit='c1'))
        rtf = self.write('rtf.json', dict(check='A10_rtf', false_detection=dict(count=0, by_code={}),
                                          emulator_clock=dict(sim_s=120.), source_commit='c1'))
        thermal_line = [[23755, 'F THERMAL 5001 23755*73']]
        summary = dict(line_kinds=dict(F=1), first_non_status_lines=thermal_line)
        a25 = self.write('a25_firmware.json', dict(group='a25_firmware', duration_s=64., normal_script=True,
                                                   run=dict(controller_summaries=dict(m0=dict(line_kinds={},
                                                                                             first_non_status_lines=[])))))
        thermal = self.write('thermal.json', dict(group='thermal_firmware_report', duration_s=64., normal_script=False,
                                                  normal_script_note='faults are expected',
                                                  run=dict(controller_summaries=dict(m0=summary))))
        out = checks.summarize_normal([startup, rtf], [a25, thermal])
        self.assertEqual(out['total'], 0)
        self.assertTrue(out['passed'])
        self.assertEqual(len(out['gsim_not_normal_scripts']), 1)
        self.assertIn('A10_rtf', out['checks_counted'])
        self.assertIn('A3_G_noise', out['checks_missing'])
        self.assertIn('source_commit', out['provenance'])
        self.assertIn('profile_sha256', out['provenance'])
        self.assertEqual(out['provenance']['input_commits'], ['c1'])
        # the same THERMAL line in a normal group is counted
        thermal_normal = self.write('thermal_n.json', dict(group='a25_firmware', duration_s=64., normal_script=True,
                                                           run=dict(controller_summaries=dict(m0=summary))))
        out = checks.summarize_normal([startup], [thermal_normal])
        self.assertEqual(out['by_code'], {'THERMAL': 1})
        self.assertFalse(out['passed'])


class NoiseHelpersTest(unittest.TestCase):
    def test_raw_series_and_ks(self):
        frames = [dict(out=dict(first_tick=0, ticks=10, changes=[[0, 'mcu_state', 255], [3, 'mcu_state', 1],
                                                                  [3, 'adc_read', 512], [6, 'adc_read', 513]])),
                  dict(out=dict(first_tick=10, ticks=5, changes=[[11, 'adc_read', 511]]))]
        setup, raw = checks.raw_series(dict(frames=frames))
        self.assertEqual(setup, 3)
        self.assertEqual(raw, [512, 512, 512, 513, 513, 513, 513, 513, 511, 511, 511, 511])
        self.assertEqual(checks.ks_distance([1, 1, 2, 2], [1, 1, 2, 2]), 0.)
        self.assertAlmostEqual(checks.ks_distance([1, 1, 1, 1], [2, 2, 2, 2]), 1.)
        self.assertIsNone(checks.ks_distance([], [1]))


class StallProbeHelpersTest(unittest.TestCase):
    def test_parsers(self):
        line = '22 (python3 x (y)) S 7 22 7 0 -1 4194560 100 0 0 0 1234 56 0 0 20 0 9 0 3000 1 2 3'
        state, cpu_u, cpu_s, threads = probe.stat_fields(line)
        self.assertEqual((state, cpu_u, cpu_s, threads), ('S', 1234, 56, 9))
        self.assertEqual(probe.ctxt('voluntary_ctxt_switches:\t10\nnonvoluntary_ctxt_switches:\t3\n'), (10, 3))
        self.assertEqual(probe.pressure('some avg10=0.00 avg60=0.00 avg300=0.00 total=123\n'
                                        'full avg10=0.00 avg60=0.00 avg300=0.00 total=45\n'), dict(some=123, full=45))
        self.assertEqual(probe.kv_file('nr_throttled 4\nthrottled_usec 99\n'), dict(nr_throttled=4,
                                                                                    throttled_usec=99))
        self.assertEqual(probe.delta_dict(dict(cpu=dict(a=1, b=2)), dict(cpu=dict(a=4, b=2))), dict(cpu=dict(a=3,
                                                                                                         b=0)))


class SensorContractGpsTest(unittest.TestCase):
    def test_gps_link_and_antenna_from_the_profile(self):
        import yaml
        rover = yaml.safe_load((SIM_ROOT / 'rover.yaml').read_text(encoding='utf-8'))
        rows = {child: (parent, xyz) for parent, child, xyz, q in contract.static_transforms(profile(), rover)}
        ax, ay, az = contract.gnss_antenna(profile())
        z = rover['vehicle']['body_center_height_m']
        self.assertEqual(rows['gps_link'][0], 'base_link')
        self.assertAlmostEqual(rows['gps_link'][1][2] + z, az)
        self.assertAlmostEqual(az, .9)                           # v0.1 placeholder antenna height

    def test_antenna_view_moves_the_pinned_projection_to_the_profile_antenna(self):
        NS = types.SimpleNamespace
        yaw = math.radians(30.)
        q = NS(x=0., y=0., z=math.sin(yaw / 2.), w=math.cos(yaw / 2.))
        msg = NS(header=NS(stamp=NS(sec=5, nanosec=0)),
                 pose=NS(pose=NS(position=NS(x=1., y=2., z=.1), orientation=q)),
                 twist=NS(twist=NS(linear=NS(x=.5, y=0., z=0.), angular=NS(x=0., y=0., z=.2))))
        antenna = (.3, -.1, .9)
        view = contract.antenna_shifted_odometry(msg, antenna, .7)
        # the pinned projection adds R (0, 0, 0.7): the sum is p + R antenna
        c, s = math.cos(yaw), math.sin(yaw)
        x = view.pose.pose.position.x
        y = view.pose.pose.position.y
        zz = view.pose.pose.position.z + .7
        self.assertAlmostEqual(x, 1. + c * .3 - s * (-.1), places=12)
        self.assertAlmostEqual(y, 2. + s * .3 + c * (-.1), places=12)
        self.assertAlmostEqual(zz, .1 + .9, places=12)
        # body velocity of the antenna: v + w x d (only w_z here; the pinned projection adds the vertical arm terms)
        d = (.3, -.1, .2)
        self.assertAlmostEqual(view.twist.twist.linear.x, .5 - .2 * d[1], places=12)
        self.assertAlmostEqual(view.twist.twist.linear.y, .2 * d[0], places=12)
        self.assertIs(view.pose.pose.orientation, q)
        self.assertIs(view.header, msg.header)


class S2LaserFrameTest(unittest.TestCase):
    def test_laser_frame_is_the_s2_scan_frame(self):
        try:
            from steering import s2_format
        except ImportError as exc:                    # G14: a skip with the reason, not a silent pass
            self.skipTest('steering.s2_format is not importable here: %s' % exc)
        import yaml
        rover = yaml.safe_load((SIM_ROOT / 'rover.yaml').read_text(encoding='utf-8'))
        rows = {child: (parent, xyz, q) for parent, child, xyz, q in contract.static_transforms(profile(), rover)}
        v = profile().value
        q = rows['laser'][2]
        yaw = math.degrees(2. * math.atan2(q[2], q[3]))
        fmt = s2_format.RealFormat(float(v('lidar_yaw_offset_deg')))
        scan = [math.inf] * s2_format.SIM_BEAMS
        scan[s2_format.SIM_BEAMS // 2] = 5.
        raw = fmt.convert(scan)
        j = [i for i, r in enumerate(raw) if r > 0.][0]
        a = s2_format.RAW_AMIN + j * s2_format.RAW_INC
        self.assertGreater(5. * math.cos(a + math.radians(yaw)), 4.99)


class ImuModelFgTest(unittest.TestCase):
    """G15: the G-SIM IMU error model (p6a_imu_model) against the F-SIM plant's IMU model for the same inputs."""

    def test_noise_vibration_bias_and_scale_match_the_fsim_model(self):
        from steering import p6a_imu_model
        try:
            from steering import steered_plant
        except Exception as exc:
            self.skipTest('steered_plant not importable: %s' % exc)
        result = p6a_imu_model.compare_with_fsim(profile(), seeds=range(40), samples=400)
        self.assertTrue(result['comparable'], result)
        for key, row in result['checks'].items():
            self.assertTrue(row['passed'], (key, row))


if __name__ == '__main__':
    unittest.main()
