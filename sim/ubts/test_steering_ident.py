"""Identification tools (steering/ident, design 3.9, SIM_ACCEPTANCE A18): estimators on synthetic records, the record
formats read back by tools/field_calib, the records-only analysis (truth isolation) and the A18 comparison.

The short IdentRig run (bench tests and a standstill turn through the control API) is in test_steering_ident_rig.py
(it needs the native build). Placeholder-based SIM results (자리값 기반 SIM 결과), not field validation.
unittest style: pytest (3.12, 3.13) and `python3 -m unittest test_steering_ident` (3.10). Files go to temporary
folders only. The F1 / F2 / F3 odometry needs scipy (tools/field_calib); those parts are skipped without it.
"""
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from steering import geometry as geo
from steering import truth_isolation as ti
from steering.ident import a18
from steering.ident import analysis as an
from steering.ident import recorder as rc

UBTS_DIR = Path(__file__).resolve().parent
IDENT_DIR = UBTS_DIR / 'steering' / 'ident'
L, WK = .76, .46


def _scipy_or_skip():
    try:
        import scipy.spatial  # noqa: F401
    except Exception as exc:                    # broken installs raise more than ImportError
        raise unittest.SkipTest('scipy not usable here (field_calib odometry): %s' % exc)


class EstimatorMath(unittest.TestCase):
    def test_speed_line_recovers_rate_and_minimum(self):
        u = np.array([.4, .5, .6, .8, 1.])
        line = an._speed_line(u, 30. * (u - .25) / .75)
        self.assertAlmostEqual(line['min_pwm'], .25, places=9)
        self.assertAlmostEqual(line['rate'], 30., places=9)
        self.assertIsNone(an._speed_line([.4, .41], [1., 1.1]))            # too few points / no spread

    def test_tau_fit_recovers_the_first_order_lag(self):
        t = np.arange(-.05, 1.1, .02)
        disp = 12. * an._first_order_response(t, .5, .08) + .3
        tau, speed, offset, rms = an._fit_tau(t, disp, .5)
        self.assertAlmostEqual(tau, .08, places=3)
        self.assertAlmostEqual(speed, 12., places=2)
        self.assertLess(rms, 1e-6)

    def test_step_fit_reads_the_steady_speed_where_a_line_reads_low(self):
        t = np.arange(.012, .32, .02)                    # 0.3 s at the cap: a line over it reads the rising speed
        disp = 28. * an._first_order_response(t - .01, 1e9, .08) + 1.
        v, d, c, rms = an._step_fit(t, disp, .08, np.arange(0., .03, .002))
        self.assertAlmostEqual(v, 28., delta=.05)
        self.assertAlmostEqual(d, .01, delta=.0021)
        keep = t >= t[0] + .15
        slope = np.polyfit(t[keep], disp[keep], 1)[0]
        self.assertLess(slope, .98 * 28.)                # the bias the first-order fit removes

    def test_curvature_combination_weights_by_path_over_round_trips(self):
        kappa = .004
        parts = [dict(heading_deg=math.degrees(kappa * 18.), path_m=18.),
                 dict(heading_deg=math.degrees(kappa * -17.5), path_m=-17.5),   # reverse: same curvature
                 dict(heading_deg=math.degrees(kappa * 18.2), path_m=18.2)]
        self.assertAlmostEqual(an.combine_curvature(parts), kappa, places=12)
        with self.assertRaises(an.AnalysisError):
            an.combine_curvature([])

    def test_calibration_converts_per_side(self):
        cal = an.Calibration(math.radians(26.) * 1000., math.radians(27.) * 1000.)
        self.assertAlmostEqual(float(cal.deg(500)), 13., places=9)
        self.assertAlmostEqual(float(cal.deg(-500)), -13.5, places=9)


def _bench_record(root, rid, b1, notes):
    rec = rc.BenchRecord(root, rid, mono_start_s=0., wall_start_s=1.7e9, profile_copy=dict(profile_id='test'),
                         b1=dict(values=b1))
    for k, (event, params) in enumerate(notes):
        rec.add_note(float(k), 1.7e9 + k, 'B3', event, params)
    rec.finish()
    return rec.dir


class SyntheticBench(unittest.TestCase):
    """B3 / B6 on readings made from a known linkage, calibration and play (the gauge reads both wheels)."""

    SLOPE_LEFT, SLOPE_RIGHT, ZERO, RATIO, PLAY = 26., 27., .5, .5, 2.

    def _wheels(self, eq_deg):
        left, right = geo.wheel_angles(math.radians(eq_deg), 'linkage', L, WK, self.RATIO)
        return math.degrees(left), math.degrees(right)

    def _eq(self, pos, direction):
        slope = self.SLOPE_LEFT if pos >= 0 else self.SLOPE_RIGHT
        return slope * pos / 1000. + self.ZERO + (-.5 if direction == 'up' else .5) * self.PLAY

    def readings(self, points):
        notes = []
        for direction, pts in (('up', points), ('down', list(reversed(points)))):
            for p in pts:
                left, right = self._wheels(self._eq(p, direction))
                notes.append(('reading', dict(target_permille=p, direction=direction, pos_permille=float(p),
                                              left_deg=left, right_deg=right)))
        return notes

    def test_b3_recovers_marks_zero_and_ackermann_ratio(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = _bench_record(tmp, 'b3', dict(wheelbase_m=L, kingpin_width_m=WK),
                                   self.readings([-750, -560, -375, -190, 0, 190, 375, 560, 750]))
            out = an.b3_calibration(folder)
        self.assertAlmostEqual(out['mark_left_deg'], self.SLOPE_LEFT, delta=.01)
        self.assertAlmostEqual(out['mark_right_deg'], self.SLOPE_RIGHT, delta=.01)
        self.assertAlmostEqual(out['zero_deg'], self.ZERO, delta=.01)          # play averaged over the two sweeps
        self.assertAlmostEqual(out['ackermann_ratio'], self.RATIO, delta=.01)
        self.assertAlmostEqual(out['residual_deg_rms'], .5 * self.PLAY, delta=.02)

    def test_b6_recovers_the_backlash(self):
        cal = dict(mark_left_mrad=math.radians(self.SLOPE_LEFT) * 1000., mark_right_mrad=math.radians(
            self.SLOPE_RIGHT) * 1000.)
        notes = []
        for direction, pts in (('up', range(-320, 321, 40)), ('down', range(320, -321, -40))):
            for p in pts:
                left, right = self._wheels(self._eq(p, direction))
                notes.append(('reading', dict(target_permille=p, direction=direction, pos_permille=float(p),
                                              left_deg=left, right_deg=right)))
        with tempfile.TemporaryDirectory() as tmp:
            folder = _bench_record(tmp, 'b6', dict(wheelbase_m=L, kingpin_width_m=WK), notes)
            out = an.b6_backlash(folder, cal)
        self.assertAlmostEqual(out['backlash_deg'], self.PLAY, delta=.03)    # the mean wheel angle is near eq here

    def test_b3_refuses_too_few_readings(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = _bench_record(tmp, 'b3', dict(wheelbase_m=L, kingpin_width_m=WK), self.readings([0, 190])[:3])
            with self.assertRaises(an.AnalysisError):
                an.b3_calibration(folder)


def _archive(tmp, scans=3, imus=5):
    rec = rc.ArchiveRecorder(tmp, 'rec1', mono_start_s=10., wall_start_s=1.7e9, lidar_yaw_offset_deg=179.08,
                             profile_copy=dict(profile_id='test'), host_profile_bytes=b'{}\n',
                             calibration=dict(cal_id=1), test='F1')
    for k in range(scans):
        rec.add_packet(dict(kind='scan', ranges=np.full(3200, 5.), stamp_s=10. + .1 * k), 1.7e9 + .1 * k + .01)
    for k in range(imus):
        rec.add_packet(dict(kind='imu', accel_m_s2=(0., 0., 9.81), gyro_z_rad_s=.01, stamp_s=10. + .05 * k),
                       1.7e9 + .05 * k + .002)
    for k in range(4):
        rec.add_motor(dict(ok=True, connected=True, armed=True, moving=k > 0, state='armed_manual',
                           requested=dict(left=102, right=102, throttle=40, steer_mrad=0)),
                      1.7e9 + .5 * k, 10. + .5 * k, dict(L=102, R=102, en=3))
    rec.add_steering(dict(received_monotonic=10.1, received_at=1.7e9 + .1, mcu_ms=100, pos_permille=0,
                          angle_mrad=0, target_mrad=0, steer_pwm=0, faults=[]))
    rec.finish(12., 1.7e9 + 2.)
    return rec


class RecordFormats(unittest.TestCase):
    def test_archive_layout_and_steering_sidecar(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec = _archive(tmp)
            meta = json.loads((rec.dir / 'metadata.json').read_text(encoding='utf-8'))
            self.assertEqual(meta['state'], 'archived')
            desc = json.loads((rec.steer_dir / 'descriptor.json').read_text(encoding='utf-8'))
            self.assertEqual(desc['schema'], rc.DESCRIPTOR_SCHEMA)
            self.assertEqual(desc['frame_count'], 1)
            self.assertEqual(desc['sample_hz'], 20)
            row = rc.load_jsonl(rec.steer_dir / 'steering.jsonl')[0]
            self.assertEqual(row['schema'], rc.STEERING_SCHEMA)
            self.assertAlmostEqual(row['t_s'], .1, places=6)
            for path in (rec.dir / 'motor.jsonl', rec.steer_dir / 'steering.jsonl'):
                self.assertNotIn(b'\r\n', path.read_bytes())

    def test_field_calib_records_reads_the_archive(self):
        an.tools_path()
        from field_calib import records
        with tempfile.TemporaryDirectory() as tmp:
            _archive(tmp)
            rec = records.open_archive(tmp, 'rec1', hashes=False)
        self.assertEqual(rec.layout, 'jetson')
        self.assertEqual(len(rec.scan_t), 3)
        self.assertEqual(rec.ranges_mm.shape, (3, 3240))
        self.assertEqual(len(rec.imu), 5)
        self.assertEqual(len(rec.motor), 4)
        self.assertTrue(np.all(rec.motor.req_l == 102))
        self.assertEqual(rec.warnings, [])


class RecordsOnly(unittest.TestCase):
    """The analysis reads records only: strict truth isolation on analysis.py and the record writers."""

    def test_analysis_and_writers_pass_the_strict_check(self):
        """analysis.py passes every strict rule. The record writers pass every truth rule; bench.py holds the bench rig
        as its serial port and clock (a plant-side tool, ROLE_PLANT in the tree), which only the rig-handle rule of
        decision code (review RS-05: rig_reference) names. The MCU-channel and scene rules still apply to it."""
        for name in ('analysis.py', 'recorder.py', 'bench.py', 'instruments.py'):
            rel = 'sim/ubts/steering/ident/' + name
            errors, _ = ti.check_source((IDENT_DIR / name).read_text(encoding='utf-8'), rel, ti.ROLE_SHARED)
            if name == 'bench.py':
                self.assertTrue(errors and all(e['rule'] == 'rig_reference' for e in errors), errors)
                errors = [e for e in errors if e['rule'] != 'rig_reference']
            self.assertEqual(errors, [], name)
        self.assertEqual(ti.classify('sim/ubts/steering/ident/bench.py')[0], ti.ROLE_PLANT)

    def test_strict_check_would_catch_a_truth_read(self):
        bad = 'def f(rig):\n    return rig.plant.x\n'
        errors, _ = ti.check_source(bad, 'sim/ubts/steering/ident/analysis.py', ti.ROLE_SHARED)
        self.assertTrue(errors)


def _case(tmp, truth_overrides=None):
    """A case folder with eval/truth.json and eval/trace.npz (constant truth) for compare()."""
    case = Path(tmp) / 'case'
    (case / 'eval').mkdir(parents=True)
    truth = dict(steer_rate_max_deg_s=30., steer_rate_standstill_deg_s=15., steer_motor_tau_s=.08,
                 steer_min_pwm_pct=25., steer_min_pwm_standstill_pct=35., steer_backlash_deg=2.,
                 wheelbase_m=L, curvature_wander=dict(mean_deg_per_m=-.15, sd_deg_per_m=.95, corr_length_m=8.),
                 duty_speed={'40': dict(speed_m_s=.3)}, ackermann_ratio=.5)
    truth.update(truth_overrides or {})
    (case / 'eval' / 'truth.json').write_bytes(json.dumps(truth).encode('utf-8'))
    n = 2000
    t = np.arange(n) * .05
    k = np.where(t < 50., .002, .25)                    # F1 window before 50 s, F2 after
    np.savez(case / 'eval' / 'trace.npz', t_s=t, truth_v_m_s=np.full(n, .3), truth_steer_curvature_1_m=k,
             truth_curvature_1_m=k + .001, truth_delta_eq_rad=np.full(n, .001),
             truth_motor_side_rad=np.linspace(-.3, .3, n), steer_measured_rad=np.linspace(-.3, .3, n) * 1.01,
             phase=np.ones(n, dtype=np.int16), phase_names=np.array(['run', 'F1']))
    return case


class A18Compare(unittest.TestCase):
    def result(self, **changes):
        res = dict(B4=dict(rate_standstill_deg_s_line=15.6, min_pwm_standstill_pct=36., tau_s=.085),
                   B5=dict(levels=dict(moving=dict(rate_deg_s_line=29., min_pwm_pct=24., rate_deg_s_full_pwm=29.),
                                       standstill=dict(rate_deg_s_full_pwm=14.8))),
                   B6=dict(backlash_deg=2.2),
                   F1=dict(windows=[[1., 40.]], path_m=400., zero_offset_deg=0.),
                   F2=dict(table=[dict(windows=[[60., 75.], [75., 90.]], target_deg=13., path_m=40., kappa_1_m=.25 * .99 +
                                       math.radians(-.15))]),
                   F3=dict(duty_speed={'40': dict(v_p50_m_s=.31)}), B3=dict(ackermann_ratio=.497))
        res.update(changes)
        return res

    def rows(self, result):
        with tempfile.TemporaryDirectory() as tmp:
            return {r['quantity']: r for r in a18.compare(_case(tmp), result)}

    def test_known_values_and_bounds(self):
        rows = self.rows(self.result())
        self.assertTrue(rows['steering_rate_moving']['passed'])
        self.assertAlmostEqual(rows['steering_rate_moving']['error'], -1. / 30., places=4)
        self.assertTrue(rows['min_pwm_standstill']['passed'])
        self.assertAlmostEqual(rows['min_pwm_standstill']['error'], 1., places=6)
        self.assertTrue(rows['tau']['passed'])
        self.assertTrue(rows['backlash']['passed'])
        z = rows['zero_offset']                     # known: atan(L (kappa_steer + pull)), pull = wander mean
        known = math.degrees(math.atan(L * (.002 + math.radians(-.15))))
        self.assertAlmostEqual(z['truth'], known, places=3)
        self.assertAlmostEqual(z['error'], -known, places=3)
        self.assertTrue(z['passed'])
        realised = rows['zero_offset_vs_realised_path']
        self.assertAlmostEqual(realised['truth'], math.degrees(math.atan(L * .003)), places=3)
        self.assertIsNone(realised['passed'])       # report only
        k = rows['kappa_table +13.0 deg']
        self.assertAlmostEqual(k['error'], -.0099, delta=.0005)
        self.assertTrue(k['passed'])
        self.assertAlmostEqual(rows['duty_speed 40 %']['error'], 1. / 30., places=4)
        self.assertIsNone(rows['steering_rate_standstill_full_pwm']['bound'])
        self.assertAlmostEqual(rows['b3_calibration_gain_left']['estimate'], 1.01, places=4)

    def test_failures_are_reported_not_hidden(self):
        rows = self.rows(self.result(B6=dict(backlash_deg=2.4), F3=dict(duty_speed={'40': dict(v_p50_m_s=.33)})))
        self.assertFalse(rows['backlash']['passed'])
        self.assertFalse(rows['duty_speed 40 %']['passed'])
        summary = a18.summarize([dict(variant='default', seed=1, error=None, rows=list(rows.values()))])
        self.assertFalse(summary['all_passed'])
        failed = {f['quantity'] for g in summary['quantities'].values() for f in g['failed']}
        self.assertIn('backlash', failed)
        self.assertIn('duty_speed 40 %', failed)

    def test_missing_estimate_counts_as_not_passed(self):
        res = self.result(B4=dict(rate_standstill_deg_s_line=None, min_pwm_standstill_pct=None, tau_s=None))
        rows = self.rows(res)
        self.assertIsNone(rows['tau']['passed'])
        summary = a18.summarize([dict(variant='default', seed=1, error=None, rows=list(rows.values()))])
        self.assertLess(summary['quantities']['tau']['passed'], summary['quantities']['tau']['rows'])

    def test_variants_stay_inside_the_sweep_ranges(self):
        for name in a18.VARIANTS:
            profile = a18.variant_profile(name)
            if name != 'default':
                self.assertTrue(a18.VARIANTS[name])
                for key, value in a18.VARIANTS[name].items():
                    self.assertNotEqual(float(profile.value(key)), float(a18.variant_profile('default').value(key)),
                                        (name, key))
                    self.assertEqual(float(profile.value(key)), float(value))


WALL = 1.7e9


def _brake(since, speed, travel, duty, onset=.1, decel=2.):
    return dict(mode='brake', duty=duty, run='brake', part='stop', since_s=since, speed_m_s=speed, decel_m_s2=decel,
                onset_s=onset, travel_along_m=travel, stop_s=None if travel is None else since + .3)


def _c7(t_sim, distance, speed=.4, clean=True):
    return dict(t_cmd_wall_s=WALL + t_sim, distance_m=distance, speed_at_command_m_s=speed, time_s=.3, clean=clean,
                basis='hard stop: MCU status lines (steering sidecar)')


class A18BrakeRows(unittest.TestCase):
    """SIM_ACCEPTANCE amendment 80: C7 hard stop distance against the plant's travel, +-0.03 m, paired by time."""

    def rows(self, events, estimates):
        truth = dict(f3_stop_events=events, wall_minus_sim_s=WALL)
        f3 = dict(c7=dict(hard_stop=dict(events=estimates)))
        return {r['quantity']: r for r in a18.brake_rows(truth, f3)}

    def test_inside_and_outside_the_bound(self):
        events = [dict(mode='release', duty=30, since_s=90., speed_m_s=.3, decel_m_s2=1.3, onset_s=.25,
                       travel_along_m=.5), _brake(100., .4, .12, 30), _brake(200., .5, .15, 40)]
        rows = self.rows(events, [_c7(100.02, .135), _c7(199.97, .19, speed=.48)])
        ok, bad = rows['brake_stop_distance 30 %'], rows['brake_stop_distance 40 %']
        self.assertEqual((ok['bound'], ok['unit'], ok['error_kind']), (.03, 'm', 'absolute'))
        self.assertAlmostEqual(ok['error'], .015, places=6)
        self.assertTrue(ok['passed'])
        self.assertAlmostEqual(bad['error'], .04, places=6)
        self.assertFalse(bad['passed'])
        self.assertNotIn('brake_stop_distance 30 %% (release)', rows)          # release events get no row
        self.assertEqual(sum(1 for q in rows if q.startswith('brake_stop_distance ')), 2)
        self.assertIsNone(rows['brake_stop_speed 40 %']['passed'])               # report rows
        self.assertAlmostEqual(rows['brake_stop_speed 40 %']['error'], -.02, places=6)
        model = rows['brake_stop_distance_model 30 %']
        self.assertAlmostEqual(model['truth'], .4 * .1 + .4 ** 2 / 4., places=4)
        self.assertIsNone(model['passed'])
        summary = a18.summarize([dict(variant='default', seed=1, error=None, rows=list(rows.values()))])
        g = summary['quantities']['brake_stop_distance']
        self.assertEqual((g['rows'], g['passed']), (2, 1))
        self.assertFalse(summary['all_passed'])

    def test_unpaired_unclean_or_unstopped_is_not_a_pass(self):
        events = [_brake(100., .4, .12, 30), _brake(200., .5, .15, 40), _brake(300., .5, None, 50)]
        rows = self.rows(events, [_c7(100.5, .12), _c7(200., .15, clean=False), _c7(300., .15)])
        far, unclean, moving = (rows['brake_stop_distance %d %%' % d] for d in (30, 40, 50))
        self.assertIsNone(far['estimate'])                       # 0.5 s away: beyond the 0.3 s pairing
        self.assertIn('no C7 hard stop', far['note'])
        self.assertIsNone(unclean['estimate'])
        self.assertIn('not clean', unclean['note'])
        self.assertIsNone(moving['truth'])
        for row in (far, unclean, moving):
            self.assertIsNone(row['passed'])
        summary = a18.summarize([dict(variant='default', seed=1, error=None, rows=list(rows.values()))])
        g = summary['quantities']['brake_stop_distance']
        self.assertEqual((g['rows'], g['passed'], len(g['failed'])), (3, 0, 3))

    def test_an_estimate_pairs_once(self):
        rows = self.rows([_brake(100., .4, .12, 30), _brake(100.2, .4, .12, 40)], [_c7(100.1, .12)])
        self.assertEqual(sum(1 for q, r in rows.items() if q.startswith('brake_stop_distance ')
                             and r['estimate'] is not None), 1)

    def test_compare_includes_the_brake_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = _case(tmp, dict(f3_stop_events=[_brake(10., .4, .12, 40)], wall_minus_sim_s=WALL))
            res = A18Compare().result(F3=dict(duty_speed={'40': dict(v_p50_m_s=.31)},
                                              c7=dict(hard_stop=dict(events=[_c7(10.01, .125)]))))
            rows = {r['quantity']: r for r in a18.compare(case, res)}
        self.assertTrue(rows['brake_stop_distance 40 %']['passed'])

    def test_recommended_plan_path_lengths(self):
        default, rec = a18.PLANS['default'](), a18.recommended_plan()
        self.assertIsNone(default)
        from steering.ident.procedures import Plan
        base = Plan()

        def laps(circles, angle):
            return sum(n for a, n in circles if a == angle)
        for angle in (8., -8., 13., -13., 18., -18.):
            self.assertEqual(laps(rec.f2_circles, angle), 2 * laps(base.f2_circles, angle), angle)
        self.assertEqual((rec.f1_leg_m, rec.f1_round_trips), (base.f1_leg_m, base.f1_round_trips))
        self.assertEqual(rec.f3_duties_pct, base.f3_duties_pct)
        self.assertEqual(rec.f3_runs, base.f3_runs)


class _Plant:
    """Just what StopEventLog reads: pose, speed, the slow-down event counter and the current event."""

    def __init__(self):
        self.x = self.y = self.yaw = self.time_s = 0.
        self.v = .5
        self._decel_events = 0
        self._decel_event = None
        self.brake_at = None

    def step(self, dt):
        self.time_s += dt
        if self.brake_at is not None and self.time_s >= self.brake_at - 1e-9 and self._decel_event is None:
            self._decel_events += 1
            self._decel_event = dict(mode='brake', index=self._decel_events, since=self.time_s, onset_s=.1,
                                     decel_m_s2=2., speed_m_s=self.v)
        if self._decel_event is not None and self.time_s - self._decel_event['since'] >= .1 - 1e-9:
            self.v = max(0., self.v - 2. * dt)
        self.x += self.v * dt
        self.y += .001          # a lateral creep that the projection drops
        return 'out'


class StopEventLogTest(unittest.TestCase):
    def test_logs_the_brake_and_the_travel_along_the_heading(self):
        from steering.ident.procedures import StopEventLog
        plant = _Plant()
        log = StopEventLog(plant)
        log.install()
        log.tag(duty=40, run='brake', part='stop')
        plant.brake_at = .2
        x_before = None
        for _ in range(100):
            if x_before is None and plant.time_s + .01 >= plant.brake_at - 1e-9:
                x_before = plant.x
            self.assertEqual(plant.step(.01), 'out')
        log.remove()
        self.assertNotIn('step', plant.__dict__)
        self.assertEqual(len(log.events), 1)
        e = log.events[0]
        self.assertEqual((e['mode'], e['duty'], e['run'], e['part'], e['index']), ('brake', 40, 'brake', 'stop', 1))
        self.assertAlmostEqual(e['x0'], x_before, places=9)
        self.assertAlmostEqual(e['travel_along_m'], plant.x - x_before, places=9)
        self.assertAlmostEqual(e['travel_along_m'], .5 * .1 + .5 ** 2 / 4., delta=.5 * .01 + 1e-6)
        self.assertIsNotNone(e['stop_s'])


if __name__ == '__main__':
    unittest.main()
