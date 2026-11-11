"""A24 start-condition feasibility (steering/entry_feasibility.py; design 5.3, 7.2 A24).

Synthetic straight corridors check the ideal bang-bang controller against its closed form (peak angle, end heading,
window length), the swept-footprint departure margin (start pose, narrow corridors, the bracket margin), the slow
steering rate, the direction choice and the chord tangent. The corridor split (development vs deferred sealed
directions and the reverse of sealed row pairs) runs on a synthetic catalogue; one small run on the configured
catalogue checks the output files (skipped with the reason when the catalogue is not configured).

unittest style (also collected by pytest); temporary folders only. Placeholder profile: placeholder-based SIM
results, not field validation.
Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_entry_feasibility.py
                    python3 -B -m unittest test_steering_entry_feasibility     (Python 3.10, no pytest)
"""
import json
import math
import os
import tempfile
import unittest

import numpy as np

from closed_loop import raster_corridor as rc
from steering import entry_feasibility as ef
from steering import evaluate_steer as es

GEOMETRY, PARAMS = ef.ideal_params()
_, SLOW = ef.ideal_params(rate_deg_s=10.)
CONDITIONS = {c.name: c for c in ef.CONDITIONS}


def straight_frame():
    return rc.CurvilinearFrame(((-10., 0.), (60., 0.)), 0.)


def spacing(value):
    return es.corridor_spacing_function(None, spacing_m=value)


def check(name, width=3.7, params=PARAMS, slow=None, frame=None):
    return ef.check_entry(GEOMETRY, frame or straight_frame(), spacing(width), CONDITIONS[name], params,
                          slow_params=slow)


class IdealControllerTest(unittest.TestCase):
    def test_bang_bang_matches_its_closed_form(self):
        row = check('entry_hdg_p08')
        gain = PARAMS.speed_m_s / (GEOMETRY.wheelbase_m * PARAMS.rate_rad_s)
        peak = math.degrees(math.acos(math.exp(-math.radians(8.) / (2. * gain))))
        self.assertEqual(row['sigma'], -1)
        self.assertTrue(row['a_direction_corrects'])
        self.assertLess(peak, math.degrees(PARAMS.limit_right_rad))          # does not reach the soft limit
        self.assertAlmostEqual(row['peak_abs_delta_deg'], peak, delta=math.degrees(PARAMS.rate_rad_s * PARAMS.dt_s))
        self.assertLess(abs(row['end_psi_deg']), .3)
        per_m = math.degrees(PARAMS.rate_rad_s) / PARAMS.speed_m_s            # steering deg per metre
        self.assertAlmostEqual(row['window_length_m'], 2. * row['peak_abs_delta_deg'] / per_m, delta=.01)
        self.assertFalse(row['reached_limit'])
        self.assertFalse(row['other_direction']['corrects'])
        sharp = check('entry_hdg_m12')
        self.assertEqual(sharp['sigma'], 1)
        self.assertTrue(sharp['reached_limit'])                               # 12 deg needs the full lock
        self.assertLess(abs(sharp['end_psi_deg']), .3)

    def test_turned_wheels_and_nominal(self):
        nominal = check('nominal')
        self.assertEqual(nominal['window_length_m'], 0.)
        self.assertAlmostEqual(nominal['min_margin_m'], 3.7 / 2 - GEOMETRY.footprint_width_m / 2, places=9)
        wheels = check('start_wheels_turned_p10')
        self.assertEqual(wheels['sigma'], -1)                 # the wheels point left: the correction is to the right
        self.assertLess(abs(wheels['end_psi_deg']), .3)
        self.assertGreater(wheels['window_length_m'], 0.)
        self.assertAlmostEqual(wheels['delta0_deg'], 10.)

    def test_straightening_prediction(self):
        gain = PARAMS.speed_m_s / (GEOMETRY.wheelbase_m * PARAMS.rate_rad_s)
        delta = math.radians(15.)
        expected = gain * (-math.log(math.cos(delta)))
        self.assertAlmostEqual(ef.predicted_final_heading(0., delta, gain), expected, places=12)
        self.assertAlmostEqual(ef.predicted_final_heading(.1, -delta, gain), .1 - expected, places=12)
        frame = straight_frame()
        pose = frame.pose(1., 0., -expected)                                  # straightening alone ends parallel
        sim = ef.simulate_ideal(GEOMETRY, frame, pose, delta, -1, PARAMS)
        self.assertLess(abs(sim['end']['psi_deg']), .05)
        self.assertAlmostEqual(sim['initial_predicted_heading_deg'], 0., places=9)

    def test_start_margin_is_the_footprint_reach(self):
        row = check('entry_lat_p04')
        self.assertAlmostEqual(row['start_margin_m'], 3.7 / 2 - .40 - GEOMETRY.footprint_width_m / 2, places=4)
        div = check('entry_div_pos')
        psi = math.radians(8.)
        corner = GEOMETRY.footprint_width_m / 2 * math.cos(psi) + GEOMETRY.front_bumper_x_in_base_m * math.sin(psi)
        self.assertAlmostEqual(div['start_margin_m'], 3.7 / 2 - .40 - corner, places=4)
        self.assertLess(div['min_margin_m'], div['start_margin_m'])            # it moves on toward the row first
        self.assertEqual(div['min_margin_side'], 'left')

    def test_infeasible_in_a_narrow_corridor(self):
        self.assertEqual(check('entry_div_pos', width=1.6)['verdict'], 'INFEASIBLE')
        self.assertEqual(check('entry_div_pos', width=3.7)['verdict'], 'FEASIBLE')
        self.assertEqual(check('nominal', width=1.6)['verdict'], 'FEASIBLE')
        # between: feasible at the start, departing during the correction (the path does not depend on the width)
        wide = check('entry_div_pos', width=3.7)
        excursion = wide['start_margin_m'] - wide['min_margin_m']
        self.assertGreater(excursion, .005)
        width = 3.7 - 2. * (wide['start_margin_m'] - excursion / 2.)
        row = check('entry_div_pos', width=width)
        self.assertGreater(row['start_margin_m'], 0.)
        self.assertEqual(row['verdict'], 'INFEASIBLE')
        self.assertGreater(row['min_margin_t_s'], 0.)

    def test_wheel_sweep_and_slow_rate_reduce_the_margin(self):
        frame = straight_frame()
        lock = -PARAMS.limit_right_rad                         # full right lock, heading 0, centred
        arrays = dict(t=np.zeros(1), x=np.ones(1), y=np.zeros(1), yaw=np.zeros(1), delta=np.array([lock]))
        swept = ef.departure_margins(GEOMETRY, frame, spacing(3.7), arrays, es.FootprintCache(GEOMETRY))
        bare = ef.departure_margins(GEOMETRY, frame, spacing(3.7), arrays,
                                    es.FootprintCache(GEOMETRY, include_hardware_margin=False))
        left, right = GEOMETRY.wheel_angles(lock)
        sweep_left = GEOMETRY.wheel_sweep_beyond_body('left', left, False)     # outer wheel, rear corner out
        sweep_right = GEOMETRY.wheel_sweep_beyond_body('right', right, False)  # inner wheel, front corner out
        body = 3.7 / 2 - GEOMETRY.footprint_width_m / 2
        self.assertGreater(min(sweep_left, sweep_right), GEOMETRY.hardware_margin_m)
        # 1e-6 m: the footprint cache rounds wheel angles to 1e-5 rad
        self.assertAlmostEqual(float(bare[0][0]), body - sweep_left, places=6)
        self.assertAlmostEqual(float(bare[1][0]), body - sweep_right, places=6)
        self.assertAlmostEqual(float(swept[0][0]), body - sweep_left - GEOMETRY.hardware_margin_m, places=6)
        self.assertAlmostEqual(float(swept[1][0]), body - sweep_right - GEOMETRY.hardware_margin_m, places=6)
        row = check('entry_div_pos', slow=SLOW)
        self.assertLess(row['slow_rate']['min_margin_m'], row['min_margin_m'])
        self.assertGreater(row['slow_rate']['window_length_m'], row['window_length_m'])
        self.assertEqual(row['slow_rate']['rate_deg_s'], 10.)

    def test_chord_tangent_smooths_a_kink(self):
        frame = rc.CurvilinearFrame(((0., 0.), (10., 0.), (20., 20. * math.tan(math.radians(2.)) / 2.)), 0.)
        self.assertAlmostEqual(math.degrees(ef.chord_tangent(frame, 5.)), 0., places=9)
        self.assertAlmostEqual(math.degrees(ef.chord_tangent(frame, 10.)), 1., delta=.01)
        x, y = frame.point(10.)
        self.assertAlmostEqual(math.degrees(ef.chord_heading_error(frame, x, y, math.radians(1.))), 0., delta=.01)


def fake_catalogue(extra=()):
    keys = list(ef.FROZEN_DIRECTIONS) + list(ef.SEALED_DIRECTIONS) + [('042301', 'c01', 'rev'),
                                                                     ('051127', 'c03', 'fwd')] + list(extra)
    corridors = {}
    for record, cid, direction in keys:
        entry = corridors.setdefault((record, cid), dict(record=record, corridor_id=cid, directions=dict(
            fwd=dict(status='excluded'), rev=dict(status='excluded'))))
        entry['directions'][direction]['status'] = 'included'
    return dict(corridors=list(corridors.values()))


class CorridorSplitTest(unittest.TestCase):
    def test_sealed_and_their_reverse_are_deferred(self):
        development, deferred = ef.split_directions(fake_catalogue())
        keys = {d['key'] for d in development}
        self.assertEqual({d['key'] for d in deferred},
                         {'042301:c01:fwd', '042301:c04:fwd', '042301:c07:fwd', '042301:c01:rev'})
        for record, cid, direction in ef.SEALED_DIRECTIONS:
            self.assertNotIn(ef.corridor_key(record, cid, direction), keys)
        roles = {d['key']: d['role'] for d in development}
        self.assertEqual(sum(1 for r in roles.values() if r == 'frozen'), 8)
        self.assertEqual(roles['051127:c03:fwd'], 'included')
        self.assertTrue(all('pre-registration' in d['reason'] for d in deferred))

    def test_missing_frozen_direction_is_an_error(self):
        catalogue = fake_catalogue()
        catalogue['corridors'][0]['directions'] = dict(fwd=dict(status='excluded'), rev=dict(status='excluded'))
        with self.assertRaises(ValueError):
            ef.split_directions(catalogue)

    def test_run_refuses_a_folder_inside_the_repository(self):
        with self.assertRaises(ValueError):
            ef.run_a24(os.path.join(os.path.dirname(os.path.abspath(ef.__file__)), 'a24_out'))


class CatalogueRunTest(unittest.TestCase):
    def test_one_frozen_corridor_and_the_files(self):
        try:
            path = rc.catalogue_path()
        except FileNotFoundError as exc:
            self.skipTest('corridor catalogue not configured: %s' % exc)
        with tempfile.TemporaryDirectory() as folder:
            report = ef.run_a24(folder, only=['040927:c08:fwd'], conditions=['nominal', 'entry_div_pos'],
                                hard_cells=False, catalogue_path=str(path))
            with open(os.path.join(folder, 'A24_ENTRY_FEASIBILITY.json'), 'rb') as handle:
                saved = json.loads(handle.read().decode('utf-8'))
            with open(os.path.join(folder, 'A24_TABLE.txt'), 'rb') as handle:
                table = handle.read().decode('utf-8')
            with self.assertRaises(ValueError):
                ef.run_a24(folder, only=['042301:c01:fwd'], hard_cells=False, catalogue_path=str(path))
        self.assertEqual(saved['schema'], ef.SCHEMA)
        self.assertEqual(report['development_directions'], 1)
        corridor = report['corridors'][0]
        self.assertEqual(corridor['role'], 'frozen')
        self.assertEqual([r['condition'] for r in corridor['rows']], ['nominal', 'entry_div_pos'])
        self.assertTrue(all(r['verdict'] in ('FEASIBLE', 'INFEASIBLE') for r in corridor['rows']))
        deferred = {d['key'] for d in report['deferred_directions']}
        self.assertTrue({'042301:c01:fwd', '042301:c04:fwd', '042301:c07:fwd'} <= deferred)
        self.assertIn('040927:c08:fwd', table)
        self.assertFalse(report['field_validated'])
        self.assertNotIn('polyline', json.dumps(saved))


if __name__ == '__main__':
    unittest.main()
