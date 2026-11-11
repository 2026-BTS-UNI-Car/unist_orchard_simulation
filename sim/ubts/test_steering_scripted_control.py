"""Scripted controllers of the front-steer executor (steering/scripted_control.py; design 8.3 P4).

Pure Python on hand-made estimator snapshots (no plant, no world, no native program): the slew shaper (moving /
standstill rule, soft-limit clamp, raw segments), the straight hold (alignment at standstill, corridor law, gyro
dead-reckoning law, distance / row end / front stops, STOP_REQUESTED -> HOLD after stillness), the segment scripts
(steps, functions, 'hold', end stop or end hold), the maneuver builders and the factory checks.

unittest style (also collected by pytest); no file is written. Placeholder values (자리값 기반 SIM 결과).
Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_scripted_control.py
                    python3 -B -m unittest test_steering_scripted_control     (Python 3.10, no pytest)
"""
import math
import types
import unittest

from steering import scripted_control as sc

EXEC = types.SimpleNamespace(steer_rate_limit_deg_s=24., steer_rate_limit_standstill_deg_s=12.,
                             steer_limit_left_mrad=409, steer_limit_right_mrad=400)


def snap(**over):
    s = dict(corridor_valid=False, heading_error=None, lateral_offset=None, gyro_heading=0., speed=0., still=True,
             still_s=1., distance_m=0., front_clearance_m=None, row_end_ahead_m=None, steer_measured_mrad=0.,
             steer_command_mrad=0)
    s.update(over)
    return s


class ShaperTest(unittest.TestCase):
    def test_rates_and_clamp(self):
        sh = sc.SteerShaper(math.radians(24.) * 1000., math.radians(12.) * 1000., 409, 400)
        sh.start(0., 0.)
        moving = math.radians(24.) * 1000. * .75 * .05
        standstill = math.radians(12.) * 1000. * .75 * .05
        self.assertEqual(sh.step(.05, 300., 40), int(round(moving)))
        sh.start(0., 0.)
        sh.last_throttle = 0
        self.assertEqual(sh.step(.05, 300., 0), int(round(standstill)))
        self.assertEqual(sh.clamp(1000.), .9 * 409)
        self.assertEqual(sh.clamp(-1000.), -.9 * 400)
        self.assertEqual(sh.clamp(1000., margin=1.), 409)
        sh.start(1., 0.)
        self.assertEqual(sh.step(1.05, 250., 40, raw=True), 250)          # raw: no slew, no clamp

    def test_previous_throttle_counts_as_moving(self):
        sh = sc.SteerShaper(1000., 100., 409, 409)
        sh.start(0., 0.)
        sh.step(.05, 0., 40)
        self.assertEqual(sh.step(.1, 500., 0), int(round(.75 * 1000. * .05)))   # the previous command moved


class StraightHoldTest(unittest.TestCase):
    def make(self, **spec):
        return sc.make_factory(dict(dict(kind='straight_hold', drive_pct=40, distance_m=5.), **spec), EXEC)()

    def test_align_at_standstill_then_drive(self):
        c = self.make()
        cmd = c.step(0., snap(steer_measured_mrad=60., steer_command_mrad=60))
        self.assertEqual((cmd.state, cmd.reason, cmd.throttle, cmd.steer_mrad), ('FOLLOW', 'align', 0, 60))
        cmd = c.step(.05, snap(steer_measured_mrad=60., steer_command_mrad=60))
        standstill = math.radians(12.) * 1000. * .75 * .05                    # toward 0 at the standstill slew
        self.assertEqual(cmd.steer_mrad, int(round(60 - standstill)))
        t, value = .05, cmd.steer_mrad
        while value != 0 and t < 5.:
            t += .05
            value = c.step(t, snap(steer_measured_mrad=float(value), steer_command_mrad=value)).steer_mrad
        self.assertEqual(value, 0)
        cmd = c.step(t + .05, snap(steer_measured_mrad=0.))
        cmd = c.step(t + .1, snap(steer_measured_mrad=0., speed=.1, still=False))
        self.assertEqual((cmd.state, cmd.throttle, cmd.reason), ('FOLLOW', 40, 'hold_dead_reckoning'))
        self.assertEqual([p for _, p in c.phase_events], ['ALIGN', 'DRIVE'])

    def test_corridor_law_sign(self):
        c = self.make(k_psi=1., k_d=0.)
        c.step(0., snap(corridor_valid=True, heading_error=0., lateral_offset=0.))
        c.step(.05, snap(corridor_valid=True, heading_error=0., lateral_offset=0.))
        # heading_error = +0.05 rad (the corridor is to the left of the heading): steer left (positive)
        cmd = c.step(.1, snap(corridor_valid=True, heading_error=.05, lateral_offset=0., still=False, speed=.3))
        self.assertGreater(cmd.steer_mrad, 0)
        self.assertEqual(cmd.reason, 'hold_corridor')
        c = self.make(k_psi=0., k_d=1.)
        c.step(0., snap(corridor_valid=True, heading_error=0., lateral_offset=0.))
        c.step(.05, snap(corridor_valid=True, heading_error=0., lateral_offset=0.))
        # lateral_offset +0.1 m (left of the centre line): steer right (negative)
        cmd = c.step(.1, snap(corridor_valid=True, heading_error=0., lateral_offset=.1, still=False, speed=.3))
        self.assertLess(cmd.steer_mrad, 0)

    def test_dead_reckoning_law_uses_only_the_gyro_heading(self):
        c = self.make(k_psi=1., k_d=0.)
        c.step(0., snap(gyro_heading=.2))
        c.step(.05, snap(gyro_heading=.2))                  # reference heading fixed at the drive start
        cmd = c.step(.1, snap(gyro_heading=.25, speed=.3, still=False))
        self.assertLess(cmd.steer_mrad, 0)                  # turned left by 0.05 rad: steer right

    def test_stops_and_hold_after_stillness(self):
        for over, reason in ((dict(distance_m=5.2), 'script_distance'),
                             (dict(corridor_valid=True, heading_error=0., lateral_offset=0., row_end_ahead_m=.5),
                              'row_end'),
                             (dict(front_clearance_m=.5), 'obstacle_ahead')):
            c = self.make()
            c.step(0., snap())
            c.step(.05, snap())
            cmd = c.step(.1, snap(still=False, still_s=0., speed=.3, **over))
            self.assertEqual((cmd.state, cmd.throttle, cmd.steer_mrad), ('STOP_REQUESTED', 0, None), reason)
            self.assertEqual(c.pending, ('hold', reason))
            cmd = c.step(.15, snap(still=True, still_s=.2))
            self.assertEqual(cmd.state, 'STOP_REQUESTED')
            cmd = c.step(.6, snap(still=True, still_s=.45))
            self.assertEqual((cmd.state, cmd.reason), ('HOLD', reason))

    def test_no_truth_in_the_inputs(self):
        # the controller reads only snapshot keys (a dict without plant values works end to end)
        c = self.make()
        for k in range(10):
            c.step(.05 * k, snap())
        self.assertIn(c.state, ('FOLLOW', 'STOP_REQUESTED', 'HOLD'))


class SegmentScriptTest(unittest.TestCase):
    def test_steps_hold_and_end_stop(self):
        spec = dict(kind='segments', segments=[dict(duration_s=1., throttle=0, steer_deg=0., label='a'),
                                               dict(duration_s=1., throttle=0, steer_deg='hold', label='b'),
                                               dict(duration_s=1., throttle=30, steer_deg=2., label='c')])
        c = sc.make_factory(spec, EXEC)()
        self.assertEqual(c.step(0., snap()).reason, 'script_a')
        self.assertEqual(c.step(1.5, snap()).reason, 'script_b')
        cmd = c.step(2.5, snap())
        self.assertEqual((cmd.reason, cmd.throttle), ('script_c', 30))
        cmd = c.step(3.2, snap(still=False, still_s=0.))
        self.assertEqual((cmd.state, c.pending), ('STOP_REQUESTED', ('hold', 'script_complete')))
        self.assertEqual(c.step(3.8, snap(still=True, still_s=.5)).state, 'HOLD')

    def test_hold_end_and_functions(self):
        spec = dict(kind='segments', end='hold_end', rate_margin=1., segments=[
            dict(duration_s=4., throttle=0, steer_deg=dict(fn='triangle', amplitude_deg=1., period_s=4.)),
            dict(duration_s=1., throttle=0, steer_deg=dict(fn='sine', amplitude_deg=1., period_s=4.))])
        c = sc.make_factory(spec, EXEC)()
        values = [c.step(.05 * k, snap()).steer_mrad for k in range(80)]
        self.assertAlmostEqual(max(values), math.radians(1.) * 1000., delta=1.)
        self.assertAlmostEqual(min(values), -math.radians(1.) * 1000., delta=1.)
        cmd = c.step(5.1, snap())
        self.assertEqual((cmd.state, cmd.reason, cmd.steer_mrad), ('HOLD', 'script_complete', None))

    def test_builders(self):
        for segs in (sc.steps(), sc.circle(), sc.zigzag(), sc.stop_test(), sc.standstill_sweeps(throttle=4)):
            c = sc.make_factory(dict(kind='segments', segments=segs), EXEC)()
            self.assertEqual(c.step(0., snap()).state, 'FOLLOW')
        self.assertEqual(sc.standstill_sweeps(throttle=4)[0]['throttle'], 4)
        self.assertEqual(sc.circle(steer_deg=10.)[1]['steer_deg'], 10.)

    def test_hold_still(self):
        c = sc.make_factory(dict(kind='hold_still', duration_s=1.), EXEC)()
        cmd = c.step(0., snap(steer_command_mrad=33))
        self.assertEqual((cmd.throttle, cmd.steer_mrad), (0, 33))
        self.assertEqual(c.step(1., snap()).state, 'HOLD')


class FactoryTest(unittest.TestCase):
    def test_checks_and_digest(self):
        with self.assertRaises(ValueError):
            sc.make_factory(dict(kind='pivot'), EXEC)
        with self.assertRaises(ValueError):
            sc.make_factory(dict(kind='segments'), EXEC)
        for key in ('rate_margin', 'limit_margin'):
            with self.assertRaises(ValueError):
                sc.make_factory(dict(kind='hold_still', **{key: 1.5}), EXEC)
        cfg = sc.controller_cfg(dict(kind='hold_still', limit_left_mrad=999), EXEC)
        self.assertEqual(cfg['limit_left_mrad'], 409)                 # the executor's limit, never the spec's
        a = sc.spec_digest(dict(kind='hold_still', duration_s=1.))
        self.assertEqual(a, sc.spec_digest({'duration_s': 1., 'kind': 'hold_still'}))
        self.assertEqual(len(a), 64)

    def test_command_shape(self):
        cmd = sc.SteerCommand('FOLLOW', 'x', 40, 12)
        self.assertEqual((cmd.left, cmd.right), (40, 40))


if __name__ == '__main__':
    unittest.main()
