"""Steering episode evaluator (steering/evaluate_steer.py; design 7.1, 6.5, 7.2 A21 evaluator part).

Synthetic traces (fast_sim trace columns built by hand) exercise every metric and rule: lateral / heading statistics
in the corridor frame, the two safety-band forms, the swept-footprint departure, footprint contacts with the steered
wheels (raster cells and W0 posts) against the body-only footprint, the PREREG termination and the row-end stop
position, stop categories (registered and proposed steering table), steering saturation / activity / tracking,
wheel limits, REVERSE, the drive-based AUTO_RESTART, STEER_AFTER_TERMINAL and the stop response. Two tests use the
real fast_sim runner (L0 servo, W0, direct runner): the A21 evaluator case and a run-folder evaluation.

unittest style (also collected by pytest); no file is written inside the work tree (temporary folders only).
Placeholder profile: placeholder-based SIM results, not field validation.
Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_evaluate.py
                    python3 -B -m unittest test_steering_evaluate        (Python 3.10, no pytest)
"""
import math
import os
import tempfile
import unittest

import numpy as np

from closed_loop import raster_corridor as rc
from steering import evaluate_steer as es
from steering import flat_world as fw
from steering import geometry as geo
from steering.vehicle_profile import load_profile

CONFIG = es.EvalConfig.from_profile(load_profile())
G = CONFIG.geometry
DT = .01


def trace(t, x, y, yaw, v, deq=None, cmd_deg=None, target_deg=None, measured_deg=None, drive=None, applied=None,
          stop=None, wheels=None):
    """fast_sim-like trace columns from per-row arrays (angles in deg where the name says so)."""
    t = np.asarray(t, dtype=float)
    n = len(t)
    full = (lambda value, default=0.: np.full(n, default, dtype=float) if value is None
            else np.asarray(value, dtype=float) * np.ones(n))
    deq = full(deq)
    if wheels is None:
        pairs = np.array([G.wheel_angles(float(d)) for d in deq]) if n else np.zeros((0, 2))
    else:
        pairs = np.asarray(wheels, dtype=float)
    x, y, yaw, v = full(x), full(y), full(yaw), full(v)
    step = np.hypot(np.diff(x, prepend=x[0]), np.diff(y, prepend=y[0]))
    arrays = dict(t_s=t, truth_x_m=x, truth_y_m=y, truth_yaw_rad=yaw, truth_yaw_unwrapped_rad=np.unwrap(yaw),
                  truth_v_m_s=v, truth_delta_eq_rad=deq, truth_delta_left_rad=pairs[:, 0].copy(),
                  truth_delta_right_rad=pairs[:, 1].copy(), truth_distance_m=np.cumsum(step),
                  cmd_steer_target_mrad=1000. * np.radians(full(cmd_deg, math.nan) if cmd_deg is None else full(cmd_deg)),
                  steer_target_rad=np.radians(full(target_deg)) if target_deg is not None else deq.copy(),
                  steer_measured_rad=np.radians(full(measured_deg)) if measured_deg is not None else deq.copy(),
                  drive_request_pct=full(drive), cmd_throttle_pct=full(drive), drive_applied_pct=full(
                      applied if applied is not None else drive),
                  cmd_stop_mode=np.asarray(full(stop), dtype=np.int8))
    return arrays


def straight_run(s0=1., s_stop=10., d=0., heading_deg=0., speed=.4, rest_s=1.5, y0=0., x0=0., **extra):
    """Straight drive along +x (world = field for W0 / no world): moving to s_stop, then at rest for rest_s."""
    xs, vs = [], []
    s, t = s0, 0.
    times = []
    while True:
        t += DT
        moving = s < s_stop - 1e-12
        if moving:
            s = min(s + speed * DT, s_stop)
        xs.append(s)
        vs.append(speed if moving else 0.)
        times.append(round(t, 6))
        if not moving and sum(1 for value in vs if value == 0.) * DT >= rest_s:
            break
    n = len(times)
    drive = np.where(np.asarray(vs) > 0, 60., 0.)
    yaw = np.full(n, math.radians(heading_deg))
    return trace(times, np.asarray(xs) + x0, np.full(n, d + y0), yaw, vs, drive=drive, **extra)


def synthetic_map(classes, res=.05):
    return rc.RasterMap(np.asarray(classes, dtype=np.uint8), res, dict(record='synthetic', pgm_sha256='',
                                                                          yaml_sha256='', occupied_thresh=.65,
                                                                          free_thresh=.196))


def corridor_world(length=30., spacing=3.7, trunks=True, res=.05, extra_cells=()):
    """Straight corridor along +x at y = 4 (raster-local), field s = x - 2, d = y - 4; trunks = 3 x 3 occupied cells
    at d = +-spacing/2 every 2 m; no modifiers (no scipy)."""
    h, w = int(8. / res), int((length + 10.) / res)
    classes = np.full((h, w), rc.FREE, dtype=np.uint8)
    if trunks:
        for sign in (1., -1.):
            yy = 4. + sign * spacing / 2
            for xx in np.arange(2., 2. + length + 1e-9, 2.):
                r, c = int(yy / res), int(xx / res)
                classes[r - 1:r + 2, c - 1:c + 2] = rc.OCCUPIED
    for xx, yy, cls in extra_cells:
        classes[int(yy / res), int(xx / res)] = cls
    rmap = synthetic_map(classes, res)
    profile_s = tuple(float(v) for v in np.arange(-2., length + 8.))
    config = rc.RasterCorridorConfig(
        record='synthetic', corridor_id='s01', polyline=((0., 4.), (length + 10., 4.)), s_origin_m=2., end_s_m=length,
        line_end_s_m=(length, length), profile_s_m=profile_s, left_row_d_m=tuple([spacing / 2] * len(profile_s)),
        right_row_d_m=tuple([-spacing / 2] * len(profile_s)), row_width_m=spacing, band_m=.75)
    frame = rc.CurvilinearFrame(config.polyline, config.s_origin_m)
    return rc.RasterCorridorWorld(config, rmap, frame)


def on_world(arrays):
    """Shift a field-frame trace (x = s, y = d) into the corridor_world raster frame (x = s + 2, y = d + 4)."""
    out = dict(arrays)
    out['truth_x_m'] = arrays['truth_x_m'] + 2.
    out['truth_y_m'] = arrays['truth_y_m'] + 4.
    return out


class SignedDistanceTest(unittest.TestCase):
    def test_convex_polygon_signed_distance_matches_the_box_formula(self):
        rect = es._ccw(np.asarray(geo.rectangle(-.61, .61, -.35, .35), dtype=float))
        rng = np.random.default_rng(5)
        px, py = rng.uniform(-2, 2, 500), rng.uniform(-2, 2, 500)
        ox, oy = np.abs(px) - .61, np.abs(py) - .35
        box = np.hypot(np.maximum(ox, 0.), np.maximum(oy, 0.)) + np.minimum(np.maximum(ox, oy), 0.)
        np.testing.assert_allclose(es.signed_distance_convex(px, py, rect), box, atol=1e-12)
        clockwise = rect[::-1].copy()
        np.testing.assert_allclose(es.signed_distance_convex(px, py, es._ccw(clockwise)), box, atol=1e-12)

    def test_body_only_raster_clearance_equals_the_prereg_rectangle(self):
        world = corridor_world(extra_cells=((8.3, 4.62, rc.OCCUPIED), (9.0, 3.0, rc.UNCERTAIN),
                                            (7.0, 4.5, rc.ADDED_BRANCH)))
        cache = es.FootprintCache(G)
        for x, y, yaw in ((8., 4.1, 0.), (8.05, 4.0, .3), (7.6, 4.2, -.4), (20., 4.5, 1.)):
            polys = {name: es.to_world(p, x, y, yaw) for name, p in cache.polygons(0., 0.).items()}
            ours = es.polygon_clearances(world, polys, x, y)['body']
            prereg = rc.footprint_clearances(world.map, x, y, yaw, G.footprint_length_m, G.footprint_width_m)
            for name in rc.CLEARANCE_GROUPS:
                if math.isinf(prereg[name]):
                    self.assertTrue(math.isinf(ours[name]), name)
                else:
                    self.assertAlmostEqual(ours[name], prereg[name], places=9, msg=name)


class FootprintSweepTest(unittest.TestCase):
    """A21 evaluator part, synthetic: a cell / post just outside the body but inside the steered wheel."""

    def wheel_point(self, side, delta):
        left, right = G.wheel_angles(delta)
        corners = np.asarray(G.front_wheel_corners(side, left if side == 'left' else right, False))
        sign = 1. if side == 'left' else -1.
        cx, cy = corners[int(np.argmax(sign * corners[:, 1]))]
        return cx, cy - sign * .004, (left, right)

    def test_raster_cell_reached_only_by_the_steered_wheel(self):
        # 0.01 m cells: the full-cell enclosure (res / sqrt 2) of the 0.05 m maps would already touch the body here
        delta = G.soft_limits_eq()[0]
        cx, cy, wheels = self.wheel_point('left', delta)
        self.assertGreater(cy, G.footprint_width_m / 2 + .02)
        base = (8., 4.)
        cell = (base[0] + cx, base[1] + cy)
        world = corridor_world(trunks=False, res=.01, extra_cells=((cell[0], cell[1], rc.OCCUPIED),))
        polys = {name: es.to_world(p, base[0], base[1], 0.) for name, p in
                 es.FootprintCache(G).polygons(*wheels).items()}
        values = es.polygon_clearances(world, polys, *base)
        self.assertGreater(values['body']['hard'], 0.)
        self.assertLessEqual(values['swept']['hard'], 0.)
        straight = {name: es.to_world(p, base[0], base[1], 0.) for name, p in
                    es.FootprintCache(G).polygons(0., 0.).items()}
        self.assertGreater(es.polygon_clearances(world, straight, *base)['swept']['hard'], 0.)

    def test_posts_and_the_trace_evaluator_reflect_the_sweep_per_angle(self):
        for side, sign in (('left', 1.), ('right', -1.)):
            delta = sign * G.soft_limits_eq()[0 if side == 'left' else 1]
            cx, cy, _ = self.wheel_point(side, delta)
            world = fw.FlatWorld(fw.FlatWorldConfig(markers=((cx, cy, .01),)))
            n = 200
            deq = np.concatenate((np.zeros(50), np.linspace(0., delta, 100), np.full(50, delta)))
            arrays = trace(np.arange(1, n + 1) * DT, 0., 0., 0., 0., deq=deq, target_deg=np.degrees(deq))
            ev = es.evaluate_trace(arrays, CONFIG, world=world, prereg_termination=False)
            self.assertGreater(ev['contacts']['body_only']['min_hard_clearance_m'], 0.)
            self.assertLessEqual(ev['contacts']['min_hard_clearance_m'], 0.)
            self.assertGreater(ev['contacts']['wheel_sweep_only_collision_samples'], 0)
            first = ev['contacts']['first_collision_t_s']
            self.assertGreater(first, .5)          # not while the wheels were straight
            self.assertEqual(ev['steering']['wheel_limits']['violation_samples'], 0)
            sweep = ev['steering']['sweep']['max_left_wheel_beyond_body_m' if side == 'left'
                                            else 'max_right_wheel_beyond_body_m']
            left, right = G.wheel_angles(delta)
            expected = G.wheel_sweep_beyond_body(side, left if side == 'left' else right, True)
            self.assertAlmostEqual(sweep, expected, places=3)


class CorridorMetricTest(unittest.TestCase):
    def test_lateral_and_heading_statistics_in_the_row_window(self):
        arrays = on_world(straight_run(s_stop=29.6, d=.05, heading_deg=.5))
        world = corridor_world(trunks=False)
        ev = es.evaluate_trace(arrays, CONFIG, world=world, final_reason='row_end')
        self.assertEqual(ev['prereg_end'], 'TERMINAL_STOP')
        self.assertAlmostEqual(ev['lateral_m']['base']['rmse'], .05, places=6)
        self.assertAlmostEqual(ev['lateral_m']['base']['p95'], .05, places=6)
        self.assertAlmostEqual(ev['heading_deg']['max'], .5, places=3)
        self.assertAlmostEqual(ev['heading_deg']['rmse'], .5, places=3)
        rear = .05 + G.rear_axle_x_in_base_m * math.sin(math.radians(.5))
        front = .05 + G.front_bumper_x_in_base_m * math.sin(math.radians(.5))
        self.assertAlmostEqual(ev['lateral_m']['rear_axle']['max'], abs(rear), places=4)
        self.assertAlmostEqual(ev['lateral_m']['front_bumper']['max'], front, places=4)
        self.assertAlmostEqual(ev['lateral_m']['base_distance_weighted']['rmse'], .05, places=6)
        # in-row window: first motion .. rest, 20 Hz grid samples
        self.assertEqual(ev['grid_dt_s'], .05)
        self.assertGreater(ev['lateral_m']['samples'], 500)

    def test_row_end_stop_position_outcomes_and_categories(self):
        world = corridor_world(trunks=False)
        ok = es.evaluate_trace(on_world(straight_run(s_stop=29.6)), CONFIG, world=world, final_reason='row_end')
        self.assertEqual(ok['stop_category'], 'ROW_END')
        self.assertAlmostEqual(ok['stop_position_e_m'], 29.6 + G.front_bumper_x_in_base_m - 30., places=3)
        self.assertEqual(ok['row_end_outcome'], 'OK')
        self.assertFalse(ok['false_row_end_detection'])
        early = es.evaluate_trace(on_world(straight_run(s_stop=25.)), CONFIG, world=world, final_reason='row_end')
        self.assertEqual(early['row_end_outcome'], 'FALSE')
        self.assertIn('ROW_END_FALSE', early['fail_codes'])
        self.assertTrue(early['false_row_end_detection'])
        late = es.evaluate_trace(on_world(straight_run(s_stop=30.9)), CONFIG, world=world, final_reason='row_end')
        self.assertEqual(late['row_end_outcome'], 'LATE')
        missed = es.evaluate_trace(on_world(straight_run(s_stop=33.5)), CONFIG, world=world,
                                   final_reason='obstacle_ahead')
        self.assertEqual(missed['prereg_end'], 'MISSED_ROW_END')
        self.assertTrue(missed['missed_row_end'])
        self.assertIsNone(missed['stop_category'])
        self.assertGreater(missed['overshoot_m'], 2.5)

    def test_stop_category_tables(self):
        self.assertEqual(es.steer_stop_category('row_end')[0], 'ROW_END')
        self.assertEqual(es.steer_stop_category('drive_no_response')[0], 'CONTROL_AUTHORITY')
        category, label, source = es.steer_stop_category('steering_polarity_fault')
        self.assertEqual(category, 'CONTROL_AUTHORITY')
        self.assertEqual(source, es.PROPOSED_TABLE_SOURCE)
        self.assertEqual(es.steer_stop_category('stop_unconfirmed')[0], 'SYSTEM_FAULT')
        self.assertEqual(es.steer_stop_category('steering_authority_insufficient')[2], rc.stop_category(
            'steering_authority_insufficient')[2])                  # the registered entry wins
        self.assertEqual(es.steer_stop_category('no_such_reason')[0], 'UNCLASSIFIED')

    def test_collision_ends_the_episode_with_the_swept_footprint(self):
        world = corridor_world(trunks=False, extra_cells=((2. + 12., 4. + .38, rc.OCCUPIED),))
        arrays = on_world(straight_run(s_stop=29.6))
        ev = es.evaluate_trace(arrays, CONFIG, world=world, final_reason='row_end')
        self.assertEqual(ev['prereg_end'], 'COLLISION')       # body edge at d 0.35, cell centre 0.38 (enclosure)
        self.assertIn('COLLISION', ev['fail_codes'])
        self.assertIsNone(ev['stop_category'])

    def test_no_progress_and_time_cap(self):
        world = corridor_world(trunks=False)
        slow = on_world(straight_run(s_stop=1.4, speed=.005, rest_s=.2))
        ev = es.evaluate_trace(slow, CONFIG, world=world)
        self.assertEqual(ev['prereg_end'], 'NO_PROGRESS')

    def test_safety_band_forms_and_departure(self):
        world = corridor_world(trunks=False)
        band = CONFIG.band_m
        self.assertAlmostEqual(band, 3.7 / 2 - .60 - G.footprint_width_m / 2 - .30, places=12)
        inside = es.evaluate_trace(on_world(straight_run(d=band - .01, s_stop=8.)), CONFIG, world=world)
        self.assertFalse(inside['safety_band']['exceeded'])
        self.assertFalse(inside['safety_band_swept']['exceeded'])
        outside = es.evaluate_trace(on_world(straight_run(d=band + .02, s_stop=8.)), CONFIG, world=world)
        self.assertTrue(outside['safety_band']['exceeded'])
        self.assertIn('SAFETY_BAND', outside['fail_codes'])
        # heading 6 deg at the band edge: the PREREG form holds, the swept form (corner reach) does not
        heading = es.evaluate_trace(on_world(straight_run(d=band - .01, heading_deg=6., s_stop=8.)), CONFIG,
                                    world=world)
        self.assertFalse(heading['safety_band']['exceeded'])
        self.assertTrue(heading['safety_band_swept']['exceeded'])
        # departure: body edge 0.02 m inside the row line, wheels at the soft limit stick out over it
        d = 3.7 / 2 - G.footprint_width_m / 2 - .02
        delta = G.soft_limits_eq()[0]
        steered = es.evaluate_trace(on_world(straight_run(d=d, s_stop=8., deq=delta)), CONFIG, world=world)
        self.assertTrue(steered['departure']['departed'])
        self.assertFalse(steered['departure']['prereg_body_form_departed'])
        self.assertEqual(steered['prereg_end'], 'CORRIDOR_DEPARTURE')
        self.assertEqual(steered['departure']['min_margin_side'], 'left')
        straight = es.evaluate_trace(on_world(straight_run(d=d, s_stop=8.)), CONFIG, world=world)
        self.assertFalse(straight['departure']['departed'])
        self.assertAlmostEqual(straight['departure']['min_margin_m'], .02, places=6)

    def test_w2_trace_without_its_world_is_refused(self):
        arrays = straight_run(s_stop=3.)
        arrays['meta_json'] = np.frombuffer(b'{"world":"W2"}', dtype=np.uint8)
        with self.assertRaises(es.EvaluationError):
            es.evaluate_trace(arrays, CONFIG)
        with self.assertRaises(es.EvaluationError):
            es.evaluate_trace({k: v for k, v in arrays.items() if k != 'truth_delta_left_rad'}, CONFIG)


class SteeringMetricTest(unittest.TestCase):
    def test_saturation_fraction_longest_run_and_authority_flag(self):
        limit = math.degrees(G.soft_limits_eq()[0])
        arrays = straight_run(s_stop=9., rest_s=1.2)
        n = len(arrays['t_s'])
        cmd = np.full(n, 1.)
        t = arrays['t_s']
        cmd[(t >= 5.) & (t < 9.)] = .96 * limit                        # 4 s saturated while driving
        arrays['cmd_steer_target_mrad'] = 1000. * np.radians(cmd)
        ev = es.evaluate_trace(arrays, CONFIG)
        sat = ev['steering']['saturation']
        self.assertAlmostEqual(sat['longest_saturated_run_s'], 4., delta=.06)
        driving = (t[es.grid_indices(t, .05)] < 20.).sum()
        self.assertGreater(sat['saturated_fraction'], .15)
        self.assertLess(sat['saturated_fraction'], .25)
        self.assertTrue(sat['authority_margin_low'])                  # run > 3 s on a completed (rest) episode
        self.assertGreater(driving, 0)

    def test_activity_sign_changes_total_variation_and_rates(self):
        arrays = straight_run(s_stop=11., rest_s=1.2)                    # 10 m at 0.4 m/s
        s = arrays['truth_x_m'] - 1.
        cmd = np.where((np.floor(s) % 2) == 0, 2., -2.)                 # +-2 deg, switching every metre
        arrays['cmd_steer_target_mrad'] = 1000. * np.radians(cmd)
        ev = es.evaluate_trace(arrays, CONFIG)
        act = ev['steering']['activity']
        self.assertAlmostEqual(act['travelled_m'], 10., delta=.03)     # from the first 20 Hz sample in motion
        self.assertEqual(act['sign_changes'], 9)
        self.assertAlmostEqual(act['sign_changes_per_10m'], 9. * 10. / act['travelled_m'], places=3)
        self.assertAlmostEqual(act['total_variation_cmd_deg_per_m'], 9 * 4. / act['travelled_m'], places=3)
        self.assertAlmostEqual(act['rate_cmd_p95_deg_s'], 0., places=6)  # 9 jumps among ~500 grid steps
        small = straight_run(s_stop=11., rest_s=1.2)
        small['cmd_steer_target_mrad'] = 1000. * np.radians(np.where((np.floor(small['truth_x_m'] - 1.) % 2) == 0,
                                                                     .3, -.3))
        self.assertEqual(es.evaluate_trace(small, CONFIG)['steering']['activity']['sign_changes'], 0)   # inside 0.5

    def test_rate_limited_fraction_and_tracking(self):
        arrays = straight_run(s_stop=9., rest_s=1.2)
        t = arrays['t_s']
        rate = math.degrees(CONFIG.rate_moving_rad_s)
        deq = np.radians(np.clip((t - 2.) * rate, 0., 20.))           # ramp at the moving rate limit
        arrays['truth_delta_eq_rad'] = deq
        pairs = np.array([G.wheel_angles(float(d)) for d in deq])
        arrays['truth_delta_left_rad'], arrays['truth_delta_right_rad'] = pairs[:, 0], pairs[:, 1]
        arrays['steer_target_rad'] = np.where(t >= 2., math.radians(20.), 0.)
        arrays['steer_measured_rad'] = deq + math.radians(.3)
        ev = es.evaluate_trace(arrays, CONFIG)
        sat = ev['steering']['saturation']
        ramp_steps = 20. / rate / .05
        self.assertGreater(sat['rate_limited_fraction'], (ramp_steps - 2) / 400.)
        self.assertGreater(sat['command_rate_over_limit_fraction'], 0.)
        self.assertAlmostEqual(ev['steering']['tracking']['measured_minus_true_p95_deg'], .3, places=4)
        self.assertGreater(ev['steering']['tracking']['target_minus_measured_p95_deg'], 0.)

    def test_wheel_limit_violation(self):
        arrays = straight_run(s_stop=3.)
        n = len(arrays['t_s'])
        limit = G.wheel_limits()['left_wheel_toward_left']
        arrays['truth_delta_left_rad'] = np.full(n, limit + math.radians(.5))
        ev = es.evaluate_trace(arrays, CONFIG)
        self.assertEqual(ev['steering']['wheel_limits']['violation_samples'], n)
        self.assertAlmostEqual(ev['steering']['wheel_limits']['max_excess_deg'], .5, places=6)
        self.assertIn('WHEEL_LIMIT', ev['fail_codes'])


class ForbiddenEventTest(unittest.TestCase):
    def test_reverse_rule(self):
        arrays = straight_run(s_stop=4.)
        arrays['drive_request_pct'][50:60] = -20.
        ev = es.evaluate_trace(arrays, CONFIG)
        self.assertTrue(ev['forbidden']['reverse']['flag'])
        self.assertEqual(ev['forbidden']['reverse']['samples'], 10)
        self.assertAlmostEqual(ev['forbidden']['reverse']['first_t_s'], .51, places=6)
        self.assertIn('REVERSE', ev['fail_codes'])
        clean = es.evaluate_trace(straight_run(s_stop=4.), CONFIG)
        self.assertFalse(clean['forbidden']['reverse']['flag'])

    def restart_trace(self, restart_drive=True, steer_only=False):
        first = straight_run(s_stop=3., rest_s=1.)
        t0 = first['t_s'][-1]
        second = straight_run(s0=3., s_stop=5., rest_s=1.2)
        if not restart_drive:
            second['drive_request_pct'][:] = 0.
            second['cmd_throttle_pct'][:] = 0.
            second['truth_v_m_s'][:] = 0.
            second['truth_x_m'][:] = 3.
        out = {}
        for key in first:
            if key == 't_s':
                out[key] = np.concatenate((first[key], second[key] + t0))
            elif key == 'truth_distance_m':
                out[key] = np.concatenate((first[key], second[key] + first[key][-1]))
            else:
                out[key] = np.concatenate((first[key], second[key]))
        if steer_only:
            k = len(first['t_s'])
            out['steer_target_rad'][k + 10:] = math.radians(5.)
            out['cmd_steer_target_mrad'][k + 10:] = 1000. * math.radians(5.)
        return out

    def test_auto_restart_is_drive_based(self):
        ev = es.evaluate_trace(self.restart_trace(), CONFIG, prereg_termination=False)
        self.assertTrue(ev['forbidden']['auto_restart']['flag'])
        self.assertIn('AUTO_RESTART', ev['fail_codes'])
        steer = es.evaluate_trace(self.restart_trace(restart_drive=False, steer_only=True), CONFIG,
                                  prereg_termination=False)
        self.assertFalse(steer['forbidden']['auto_restart']['flag'])          # steering at rest is not a restart
        self.assertTrue(steer['forbidden']['steer_after_terminal']['flag'])    # ... but it is steering after rest

    def stop_trace(self, after_deg, measured_at_stop_deg=10., policy_target=None, measured_follows=False):
        """Drive with 10 deg steering, STOP (brake, cmd_stop_mode 2) at t = 3 s, rest; the target after the stop
        (from 4 s). measured_follows: the measured angle equals the target (an ideal servo), else it stays."""
        n = 600
        t = np.arange(1, n + 1) * DT
        v = np.where(t < 3., .4, np.maximum(.4 - (t - 3.) * 2., 0.))
        x = np.cumsum(v) * DT
        stop = np.where(t >= 3., 2, 0)
        drive = np.where(t < 3., 50., 0.)
        target = np.where(t < 3., 10., policy_target if policy_target is not None else measured_at_stop_deg)
        target = np.where(t >= 4., after_deg, target)
        deq = np.radians(np.full(n, measured_at_stop_deg))
        measured = target if measured_follows else np.full(n, measured_at_stop_deg)
        arrays = trace(t, x, 0., 0., v, deq=deq, target_deg=target, measured_deg=measured,
                       drive=drive, stop=stop, cmd_deg=np.where(t < 3., 10., 10.))
        arrays['drive_applied_pct'] = np.where(t < 3.02, 50., 0.)
        return arrays

    def test_steer_after_terminal_hold_tolerance(self):
        tol = math.degrees(CONFIG.hold_tolerance_rad)
        self.assertAlmostEqual(tol, max(1., 2. * float(load_profile().value('servo_deadband_deg'))), places=9)
        held = es.evaluate_trace(self.stop_trace(10.), CONFIG, prereg_termination=False)
        rule = held['forbidden']['steer_after_terminal']
        self.assertFalse(rule['flag'])
        self.assertEqual(rule['windows'][0]['source'], 'stop_command')
        self.assertAlmostEqual(rule['windows'][0]['start_t_s'], 3., places=6)
        small = es.evaluate_trace(self.stop_trace(10. + .8 * tol), CONFIG, prereg_termination=False)
        self.assertFalse(small['forbidden']['steer_after_terminal']['flag'])
        moved = es.evaluate_trace(self.stop_trace(10. + 2. * tol), CONFIG, prereg_termination=False)
        self.assertTrue(moved['forbidden']['steer_after_terminal']['flag'])
        self.assertAlmostEqual(moved['forbidden']['steer_after_terminal']['windows'][0]['first_violation_t_s'], 4.,
                               places=6)
        self.assertIn('STEER_AFTER_TERMINAL', moved['fail_codes'])
        centre = es.evaluate_trace(self.stop_trace(0., policy_target=0.), CONFIG, prereg_termination=False)
        self.assertTrue(centre['forbidden']['steer_after_terminal']['flag'])
        self.assertTrue(centre['forbidden']['steer_after_terminal']['auto_centre'])
        explicit = es.evaluate_trace(self.stop_trace(10. + 2. * tol, measured_follows=True), CONFIG,
                                     prereg_termination=False, terminal_t_s=4.5)
        self.assertFalse(explicit['forbidden']['steer_after_terminal']['flag'])     # the move came before 4.5 s
        self.assertEqual(explicit['forbidden']['steer_after_terminal']['windows'][0]['source'], 'terminal_t_s')
        self.assertEqual(rule['windows'][0]['hold_position_source'], 'measured_fallback')   # no servo-view column

    def sag_trace(self, before_deg, stop_target_deg, servo_view_deg, measured_deg, wheel_deg=None, after_deg=None):
        """Brake STOP at 3 s. Servo target before_deg while driving, stop_target_deg from the stop (after_deg from 5 s
        when given). truth_servo_view_rad = servo_view_deg (the position the hold_measured policy holds), the pot
        decode steer_measured_rad = measured_deg (supply sag / noise), the true wheel angle = wheel_deg."""
        n = 700
        t = np.arange(1, n + 1) * DT
        v = np.where(t < 3., .9, np.maximum(.9 - (t - 3.) * 2., 0.))
        x = np.cumsum(v) * DT
        target = np.where(t < 3., before_deg, stop_target_deg)
        if after_deg is not None:
            target = np.where(t >= 5., after_deg, target)
        wheel = servo_view_deg if wheel_deg is None else wheel_deg
        arrays = trace(t, x, 0., 0., v, deq=np.radians(np.full(n, wheel)), target_deg=target,
                       measured_deg=np.full(n, measured_deg), drive=np.where(t < 3., 100., 0.),
                       stop=np.where(t >= 3., 2, 0), cmd_deg=np.full(n, before_deg))
        arrays['truth_servo_view_rad'] = np.radians(np.full(n, servo_view_deg))
        arrays['drive_applied_pct'] = np.where(t < 3.02, 100., 0.)
        return arrays

    def test_unchanged_command_with_a_sagged_pot_decode_is_not_an_event(self):
        # review R1: the pot decode differed from the servo target by the supply sag (> 1 deg) with the command
        # unchanged -> STEER_AFTER_TERMINAL / auto_centre were false positives (L1 brake stops, hold_target)
        for before, measured in ((6., 4.5), (-5., -3.5), (6., 7.6)):
            ev = es.evaluate_trace(self.sag_trace(before, before, before, measured), CONFIG, prereg_termination=False)
            rule = ev['forbidden']['steer_after_terminal']
            self.assertFalse(rule['flag'], rule)
            self.assertFalse(rule['auto_centre'], rule)
            self.assertNotIn('STEER_AFTER_TERMINAL', ev['fail_codes'])
            self.assertEqual(rule['windows'][0]['hold_position_source'], 'truth_servo_view')
            self.assertAlmostEqual(rule['windows'][0]['max_dev_from_command_deg'], 0., places=9)

    def test_hold_measured_lag_is_a_hold_and_the_centre_policy_is_not(self):
        # hold_measured: the stop sets the target to where the steering is (2 deg short of the old target)
        hold = es.evaluate_trace(self.sag_trace(8., 6., 6., 4.5), CONFIG, prereg_termination=False)
        rule = hold['forbidden']['steer_after_terminal']
        self.assertFalse(rule['flag'], rule)
        self.assertFalse(rule['auto_centre'])
        window = rule['windows'][0]
        self.assertAlmostEqual(window['stop_change_deg'], -2., places=6)
        self.assertAlmostEqual(window['stop_hold_gap_deg'], 0., places=6)
        self.assertFalse(window['stop_moved_command'])
        self.assertAlmostEqual(window['true_drift_deg'], 0., places=9)
        self.assertFalse(window['true_drift_over_tolerance'])
        # centre: the stop moves the target to 0 away from the steering position
        centre = es.evaluate_trace(self.sag_trace(8., 0., 8., 6.5, wheel_deg=8.), CONFIG, prereg_termination=False)
        rule = centre['forbidden']['steer_after_terminal']
        self.assertTrue(rule['flag'])
        self.assertTrue(rule['auto_centre'])
        self.assertTrue(rule['windows'][0]['stop_moved_command'])
        self.assertAlmostEqual(rule['windows'][0]['first_violation_t_s'], 3., places=6)
        self.assertIn('STEER_AFTER_TERMINAL', centre['fail_codes'])
        # a new steering command at rest after a hold is still an event
        moved = es.evaluate_trace(self.sag_trace(6., 6., 6., 4.5, after_deg=9.), CONFIG, prereg_termination=False)
        self.assertTrue(moved['forbidden']['steer_after_terminal']['flag'])
        self.assertAlmostEqual(moved['forbidden']['steer_after_terminal']['windows'][0]['first_violation_t_s'], 5.,
                               places=6)
        # the true drift after the stop is reported (A17 input), not a fail code here
        drift = self.sag_trace(6., 6., 6., 6.)
        drift['truth_delta_eq_rad'] = np.where(drift['t_s'] >= 4., math.radians(7.5), math.radians(6.))
        ev = es.evaluate_trace(drift, CONFIG, prereg_termination=False)
        rule = ev['forbidden']['steer_after_terminal']
        self.assertFalse(rule['flag'])
        self.assertTrue(rule['true_drift_over_tolerance'])
        self.assertAlmostEqual(rule['windows'][0]['true_drift_deg'], 1.5, places=6)

    def test_stop_response(self):
        ev = es.evaluate_trace(self.stop_trace(10.), CONFIG, prereg_termination=False)
        triggers = ev['stop_response']['triggers']
        self.assertEqual(len(triggers), 1)
        stop = triggers[0]
        self.assertEqual(stop['mode'], 'brake')
        self.assertAlmostEqual(stop['t0_s'], 2.99, places=6)
        self.assertAlmostEqual(stop['v0_m_s'], .4, places=6)
        self.assertAlmostEqual(stop['time_to_drive_zero_s'], .03, places=6)
        self.assertAlmostEqual(stop['time_to_rest_s'], .2 + .01, delta=.011)
        self.assertAlmostEqual(stop['sideways_shift_m'], 0., places=9)
        self.assertEqual(ev['stop_response']['by_mode']['brake']['n'], 1)


class RestClearanceTest(unittest.TestCase):
    """Review R3: PREREG obstacle_wall_clearance at rest (6 m reach) and pre_stop_speed_ratio."""

    def wall_world(self, s_wall, res=.05):
        cells = [(2. + s_wall, 4. + d, rc.ADDED_HARD) for d in np.arange(-1.6, 1.61, res)]
        return corridor_world(trunks=False, res=res, extra_cells=cells)

    def test_wall_ahead_of_the_rest_pose(self):
        s_stop = 20.
        front = s_stop + G.front_bumper_x_in_base_m
        for gap in (3., 1.5):
            world = self.wall_world(front + gap)
            ev = es.evaluate_trace(on_world(straight_run(s_stop=s_stop)), CONFIG, world=world, final_reason='row_end')
            self.assertEqual(ev['prereg_end'], 'TERMINAL_STOP')
            swept, body = ev['obstacle_wall_clearance_at_rest_m'], ev['obstacle_wall_clearance_at_rest_body_m']
            self.assertIsNotNone(swept, gap)        # a 3 m wall is beyond the 2 m contact reach: the 6 m reach finds it
            # the wall's cell column starts at the cell edge before s_wall; full-cell enclosure res / sqrt 2
            self.assertAlmostEqual(swept, gap, delta=.05 + .05 / math.sqrt(2.) + 1e-3)
            self.assertLessEqual(swept, body + 1e-9)
            self.assertEqual(ev['obstacle_wall_reach_m'], rc.AT_REST_REACH_M)
            self.assertIsNone(ev['contacts']['min_hard_clearance_m'] if gap > 2.5 else None)
        far = es.evaluate_trace(on_world(straight_run(s_stop=s_stop)), CONFIG, world=self.wall_world(front + 7.),
                                final_reason='row_end')
        self.assertIsNone(far['obstacle_wall_clearance_at_rest_m'])        # nothing within 6 m
        moving = es.evaluate_trace(on_world(straight_run(s_stop=s_stop, rest_s=.3)), CONFIG,
                                   world=self.wall_world(front + 1.5))
        self.assertIsNone(moving['obstacle_wall_clearance_at_rest_m'])     # no terminal rest (< 1 s)
        posts = es.evaluate_trace(straight_run(s_stop=4.), CONFIG, world=fw.FlatWorld(fw.FlatWorldConfig()))
        self.assertIsNone(posts['obstacle_wall_clearance_at_rest_m'])      # W0: no ADDED_HARD cells

    def test_pre_stop_speed_ratio(self):
        world = corridor_world(trunks=False)
        ev = es.evaluate_trace(on_world(straight_run(s_stop=20.)), CONFIG, world=world, final_reason='row_end')
        self.assertEqual(ev['report_only']['pre_stop_speed_ratio'], 0.)     # an instant stop from the cruise
        arrays = on_world(straight_run(s_stop=20.))
        t, v = arrays['t_s'], arrays['truth_v_m_s']
        last = int(np.flatnonzero(v > 0)[-1])
        cut = last - 40                                   # the drive command ends 0.4 s before the rest at 0.4 m/s
        arrays['drive_request_pct'][cut:] = 0.
        arrays['cmd_throttle_pct'][cut:] = 0.
        ev = es.evaluate_trace(arrays, CONFIG, world=world, final_reason='row_end')
        self.assertAlmostEqual(ev['report_only']['pre_stop_speed_ratio'], 1., places=6)
        short = es.evaluate_trace(on_world(straight_run(s_stop=8.)), CONFIG, world=world)
        self.assertIsNone(short['report_only']['pre_stop_speed_ratio'])     # no cruise of 5 m on both sides


class GridTest(unittest.TestCase):
    """Review R11: a trace_every that is not 1 or 5 thins the 20 Hz evaluation grid."""

    def thinned(self, every):
        arrays = straight_run(s_stop=4.)
        return {k: (v[::every] if isinstance(v, np.ndarray) and v.shape == arrays['t_s'].shape else v)
                for k, v in arrays.items()}

    def test_coarser_grid_is_refused_unless_asked(self):
        for every in (2, 3, 4, 7, 10):
            with self.assertRaises(es.EvaluationError, msg=every):
                es.evaluate_trace(self.thinned(every), CONFIG)
        coarse = es.evaluate_trace(self.thinned(2), CONFIG, require_grid=False)
        self.assertFalse(coarse['grid_matches'])
        self.assertAlmostEqual(coarse['grid_dt_s'], .1, places=6)
        self.assertEqual(coarse['grid_dt_requested_s'], .05)
        for every in (1, 5):
            ev = es.evaluate_trace(self.thinned(every), CONFIG)
            self.assertTrue(ev['grid_matches'])
            self.assertAlmostEqual(ev['grid_dt_s'], .05, places=6)


class HelperTest(unittest.TestCase):
    def test_weighted_percentile_is_the_single_row_eval_rule(self):
        rng = np.random.default_rng(3)
        values, weights = rng.normal(size=200), rng.uniform(0, 1, size=200)
        try:
            from closed_loop.single_row_eval import _weighted_percentile
        except Exception as exc:      # the harness needs scipy (decision R10)
            self.skipTest('closed_loop.single_row_eval cannot be imported here: %s' % type(exc).__name__)
        for q in (5, 50, 95):
            self.assertEqual(es.weighted_percentile(values, weights, q), _weighted_percentile(values, weights, q))

    def test_grid_indices(self):
        t = np.round(np.arange(1, 101) * .01, 6)
        idx = es.grid_indices(t, .05)
        self.assertEqual(list(t[idx][:3]), [.05, .1, .15])
        self.assertEqual(len(es.grid_indices(t, None)), 100)

    def test_config_from_profile(self):
        profile = load_profile()
        self.assertEqual(CONFIG.profile_sha256, profile.sha256)
        self.assertAlmostEqual(math.degrees(CONFIG.rate_moving_rad_s), profile.value('steer_rate_max_deg_s'))
        self.assertEqual(CONFIG.soft_limits_eq_rad, tuple(G.soft_limits_eq()))
        self.assertEqual(CONFIG.result_kind, '자리값 기반 SIM 결과')


class FastSimIntegrationTest(unittest.TestCase):
    def test_a21_evaluator_case_on_the_fast_sim_runner(self):
        report = es.a21_evaluator_case()
        self.assertTrue(report['passed'], report)
        for case in report['cases']:
            self.assertGreater(case['body_only_min_clearance_m'], 0.)
            self.assertLessEqual(case['swept_min_clearance_m'], 0.)
            self.assertLessEqual(case['swept_without_bracket_margin_min_clearance_m'], 0.)
            self.assertEqual(case['wheel_limit_violations']['violation_samples'], 0)
        self.assertFalse(report['field_validated'])

    def test_evaluate_run_folder(self):
        from steering import fast_sim as fs
        from steering import scene_spec as ss
        scenes = [ss.scene('ev_circle_l0', backend='L0', seeds=[0], duration_s=6.,
                           maneuver=dict(kind='circle', params=dict(speed_m_s=.4, steer_deg=10.))),
                  ss.scene('ev_stop_l0', backend='L0', seeds=[1], duration_s=8.,
                           maneuver=dict(kind='stop_mode', params=dict(mode='brake', speed_m_s=.5, drive_s=2.,
                                                                       hold_s=1.)))]
        with tempfile.TemporaryDirectory() as folder:
            summary = fs.run_scenes(scenes, os.path.join(folder, 'run'), hold_check=False)
            self.assertEqual(summary['counts']['ok'], 2)
            report = es.evaluate_run(os.path.join(folder, 'run'))
            self.assertTrue(os.path.isfile(os.path.join(folder, 'run', 'STEER_EVALUATION.json')))
        self.assertEqual(report['episodes'], 2)
        rows = {r['episode']['scene_name']: r for r in report['rows']}
        self.assertEqual(rows['ev_circle_l0']['world'], 'posts')
        self.assertGreater(rows['ev_circle_l0']['steering']['saturation']['driving_samples'], 0)
        stop = rows['ev_stop_l0']
        self.assertEqual(stop['stop_response']['by_mode']['brake']['n'], 1)
        self.assertFalse(stop['forbidden']['reverse']['flag'])
        self.assertFalse(stop['forbidden']['steer_after_terminal']['flag'])
        self.assertEqual(report['result_kinds'], ['자리값 기반 SIM 결과'])

    def test_l1_brake_stops_without_a_steering_move_are_not_events(self):
        # review R1 regression: L1 (native, pot sag and noise), brake from full duty, the command never moves after
        # the stop; the first rule flagged most of these (verify run: hold_target +6 deg 5 of 8 seeds)
        from steering import fast_sim as fs
        from steering import scene_spec as ss
        try:
            from steering.native import build as native_build
            native_build.find_compiler()
        except Exception as exc:
            self.skipTest('native steering backend not buildable here: %s' % exc)
        build_dir = os.environ.get('UBTS_STEER_BUILD_DIR')
        with tempfile.TemporaryDirectory() as folder:
            os.environ['UBTS_STEER_BUILD_DIR'] = build_dir or os.path.join(folder, 'build')
            try:
                native = fs.resolve_native()
                seeds = list(range(100, 108))
                events = {}
                for policy in ('hold_target', 'hold_measured', 'center'):
                    for steer in (6., -8.):
                        scene = ss.scene('l1_brake_%s_%s' % (policy, fs._tag(steer)), backend='L1', seeds=seeds,
                                         duration_s=10., maneuver=dict(kind='stop_mode', params=dict(
                                             mode='brake', steer_policy=policy, throttle_pct=100., steer_deg=steer,
                                             drive_s=4., hold_s=3.)))
                        config = es.EvalConfig.from_profile(fs.scene_params(scene).profile)
                        for seed in seeds:
                            arrays, _, _ = fs.run_episode(scene, seed, native_executable=native['path'])
                            ev = es.evaluate_trace(arrays, config, prereg_termination=False, corridor_metrics=False)
                            rule = ev['forbidden']['steer_after_terminal']
                            events.setdefault(policy, []).append((rule['flag'], rule['auto_centre'],
                                                                 rule['hold_position_source']))
            finally:
                if build_dir is None:
                    os.environ.pop('UBTS_STEER_BUILD_DIR', None)
                else:
                    os.environ['UBTS_STEER_BUILD_DIR'] = build_dir
        for policy in ('hold_target', 'hold_measured'):
            self.assertEqual([e for e in events[policy] if e[0] or e[1]], [], policy)
        self.assertTrue(all(flag and centre for flag, centre, _ in events['center']), events['center'])
        # review P4R-03: the hold_measured stop holds the MCU-side filtered measurement (R13-2); the fast_sim trace
        # (a row per plant step) gives that reference through the recomputation from pot_raw, not the servo view
        self.assertEqual({source for entries in events.values() for _, _, source in entries},
                         {'measured_hold_recomputed'})

    def test_recomputed_hold_reference_is_the_plant_hold_measured_target(self):
        # review P4R-03: es.measured_hold_rows equals SteeredPlant.measured_hold_rad() after every plant step (L0)
        import warnings
        from steering import steered_plant as sp
        from steering.l0_servo import L0ServoBackend
        params = sp.SteeredPlantParams.from_profile(curve='main', steer_mode='test_servo')
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', sp.SteeringBackendLevelWarning)    # L0 backend on the L1 profile
            plant = sp.SteeredPlant(params, seed=7, backend=L0ServoBackend(mode='test_servo'))
        pot, decoded, want = [], [], []
        table = params.geometry.link_table
        for k in range(300):
            plant.command_actuation(40. if k < 200 else 0., 120 if k < 120 else -60)
            plant.step(.01)
            raw = plant.steer_status()['pot_raw']
            pot.append(raw)
            decoded.append(table.wheel_from_pot((raw - params.pot_center_raw) / params.pot_counts_per_rad))
            want.append(plant.measured_hold_rad())
        plant.close()
        got = es.measured_hold_rows(np.array(pot), np.array(decoded), params.pot_median_of)
        self.assertEqual(got[params.pot_median_of:].tolist(), want[params.pot_median_of:])
        meta = dict(trace_every=1)
        arrays = dict(pot_raw=np.array(pot), steer_measured_rad=np.array(decoded),
                      truth_servo_view_rad=np.zeros(len(pot)))
        rows, source = es.hold_reference(arrays, meta, CONFIG)
        self.assertEqual(source, 'measured_hold_recomputed')
        self.assertEqual(es.hold_reference(arrays, dict(trace_every=5), CONFIG)[1], 'truth_servo_view')
        self.assertEqual(es.hold_reference(arrays, meta, CONFIG, force='truth_servo_view')[1], 'truth_servo_view')
        with self.assertRaises(es.EvaluationError):
            es.hold_reference(arrays, meta, CONFIG, force='truth_hold_position')

    def test_repository_guard_covers_every_work_tree(self):
        from steering import scene_spec as ss
        roots = ss.repository_roots()
        self.assertFalse(es._outside_repository(roots[0] / 'sim' / 'runs_here'))
        others = [r for r in roots if r != ss.WORK_TREE_ROOT.resolve()]
        if not others:
            self.skipTest('no other work tree of this repository is reachable here')
        self.assertFalse(es._outside_repository(others[0] / 'runs_here'))     # e.g. the main checkout (review R12)
        with tempfile.TemporaryDirectory() as folder:
            self.assertTrue(es._outside_repository(folder))



class ExecutorTraceTest(unittest.TestCase):
    """P4: executor-in-the-loop traces (tools/row_executor/sim_in_loop_steer.py): the hold-position column
    truth_hold_position_rad (R13-3) is the first hold reference, the terminal moment comes from the result summary
    (ended_at_s), W0 open maneuvers are evaluated without the PREREG termination, sealed corridors are refused."""

    def executor_trace(self, hold_deg, servo_view_deg, measured_deg):
        arrays = ForbiddenEventTest.sag_trace(None, 6., 6., servo_view_deg, measured_deg)
        arrays['truth_hold_position_rad'] = np.radians(np.full(len(arrays['t_s']), hold_deg))
        return arrays

    def test_hold_position_column_precedence(self):
        self.assertEqual([name for name, _ in es.HOLD_POSITION_COLUMNS][:2],
                         ['truth_hold_position_rad', 'truth_servo_view_rad'])
        arrays = self.executor_trace(6., 3., 3.)
        spec = dict(name='x', seed=0, world={'kind': 'W0'}, controller={'kind': 'segments'})
        ev = es.evaluate_executor_trace(arrays, spec, dict(ended_at_s=3., end_reason='script_complete'), CONFIG)
        rule = ev['forbidden']['steer_after_terminal']
        self.assertEqual(rule['hold_position_source'], 'truth_hold_position')
        self.assertFalse(rule['flag'], rule)
        self.assertEqual(rule['windows'][0]['source'], 'terminal_t_s')
        self.assertEqual(ev['episode']['end_reason'], 'script_complete')
        # without the executor column and without a row per plant step the servo view is used (fast_sim traces)
        del arrays['truth_hold_position_rad']
        ev = es.evaluate_executor_trace(arrays, spec, dict(ended_at_s=3., end_reason='script_complete'), CONFIG)
        self.assertEqual(ev['forbidden']['steer_after_terminal']['hold_position_source'], 'truth_servo_view')

    def test_executor_run_folder(self):
        arrays = self.executor_trace(6., 6., 6.)
        spec = dict(name='ep', seed=1, world={'kind': 'W0'}, controller={'kind': 'segments'})
        with tempfile.TemporaryDirectory() as folder:
            run = os.path.join(folder, 'ep')
            os.makedirs(run)
            np.savez(os.path.join(run, 'TRACE.npz'), **arrays)
            import json
            with open(os.path.join(run, 'RESULT.json'), 'wb') as handle:
                handle.write(json.dumps(dict(spec=spec, summary=dict(ended_at_s=3., end_reason='operator_stop'),
                                             digests=dict(trace_content_sha256='0' * 64))).encode('utf-8'))
            report = es.evaluate_executor_run(folder)
            self.assertTrue(os.path.isfile(os.path.join(folder, 'STEER_EVALUATION_EXECUTOR.json')))
        self.assertEqual(report['schema'], es.EXECUTOR_RUN_SCHEMA)
        self.assertEqual(report['episodes'], 1)
        self.assertFalse(report['field_validated'])
        self.assertEqual(report['rows'][0]['episode']['folder'], 'ep')

    def test_sealed_corridor_is_refused(self):
        with self.assertRaises(Exception) as ctx:
            es.world_for_spec({'kind': 'W2', 'record': '042301', 'corridor_id': 'c04', 'direction': 'fwd'})
        self.assertIn('sealed', str(ctx.exception))

    def test_link_reasons_are_system_faults(self):
        for reason in ('link_ttl_expired', 'link_watchdog', 'mcu_reset', 'link_command_rejected', 'mcu_disarmed'):
            self.assertEqual(es.steer_stop_category(reason)[0], 'SYSTEM_FAULT', reason)
        self.assertEqual(es.steer_stop_category('steering_stalled')[0], 'CONTROL_AUTHORITY')


if __name__ == '__main__':
    unittest.main()
