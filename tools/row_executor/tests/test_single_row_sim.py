"""single_row_sim: the SIM runner of the executor's single_row mode (vendor identity with the SIM tree, plants, per-plant
configs, PREREG episode building, operator policy and episode end, verdict rules) and short REAL raster episodes through
the real ExecutorLoop + dashboard RoverControl (wiring only: never a completion / evaluation claim).

Needs the SIM tree (imports closed_loop from sim/ubts: run this file in its own process, like the other SIM-in-the-loop
tests); the raster cases also need the 10/10 maps, CATALOGUE.json and PREREG.json (skipped otherwise). No hardware.
"""
from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import unittest

TESTS = Path(__file__).resolve().parent
REPO = TESTS.parents[2]
for directory in (TESTS, REPO / 'tools', REPO / 'sim/ubts'):
    sys.path.insert(0, str(directory))
sys.dont_write_bytecode = True

from field_response import FieldPlant, FieldResponseParams, forward_curve  # noqa: E402
from row_executor import core as XCORE  # noqa: E402
from row_executor import single_row_sim as R  # noqa: E402
from closed_loop import controllers as SC  # noqa: E402

CORRIDOR = '042301:c03:fwd'          # not in the PREREG probe selection (wiring only)


def have_world():
    try:
        R.prereg_path()
        R.RC.catalogue_path()
        R.RC.maps_root()
        return True
    except (FileNotFoundError, OSError):
        return False


HAVE_WORLD = have_world()


class VendorIdentityTest(unittest.TestCase):
    def test_vendored_files_are_byte_identical_to_the_sim_tree(self):
        manifest = XCORE.verify_vendor()
        for entry in manifest['files']:
            with self.subTest(path=entry['vendored_path']):
                source = REPO / entry['source_path']
                vendored = Path(XCORE.VENDOR_DIR) / entry['vendored_path']
                self.assertEqual(source.read_bytes(), vendored.read_bytes())
                self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), entry['sha256'])

    def test_sim_tree_single_row_module_is_verified_against_the_manifest(self):
        module = XCORE.single_row_module(SC)
        self.assertTrue(str(Path(module.__file__).resolve()).startswith(str((REPO / 'sim' / 'ubts').resolve())))
        self.assertTrue(issubclass(module.SingleRowController, SC.RowController))


class PlantTest(unittest.TestCase):
    def test_sided_gain_multiplies_only_the_applied_duty(self):
        params = FieldResponseParams()
        gained = R.plant_class(dict(left=.85, right=1.))(params, seed=(1, 2, 3))
        plain = R.plant_class()(params, seed=(1, 2, 3))
        self.assertEqual(gained._targets(80., 80., .01), plain._targets(80. * .85, 80., .01))
        self.assertEqual(R.plant_class(dict(left=1., right=.85))(params, seed=(1, 2, 3))._targets(70., 90., .01),
                         plain._targets(70., 90. * .85, .01))
        with self.assertRaises(ValueError):
            R.plant_class(dict(left=0., right=1.))

    def test_eval_plant_is_the_unchanged_field_plant(self):
        params = FieldResponseParams()
        a, b = FieldPlant(params, seed=(4, 5, 6)), R.plant_class()(params, seed=(4, 5, 6))
        for k in range(400):
            duty = (60, 70) if k < 300 else (0, 0)
            a.command(*duty)
            b.command(*duty)
            a.step(.01)
            b.step(.01)
        self.assertEqual((a.x, a.y, a.yaw, a.v, a.w), (b.x, b.y, b.yaw, b.v, b.w))


class ConfigTest(unittest.TestCase):
    def test_per_plant_configs_are_feasible_with_the_inner_wheel_rule_at_base_min(self):
        for plant in R.PLANTS:
            with self.subTest(plant=plant):
                cfg = R.single_row_executor_config(plant)
                self.assertEqual(cfg.mode, 'single_row')
                self.assertFalse(cfg.require_native_navigation)
                self.assertTrue(cfg.motor_output_enabled)
                self.assertEqual(cfg.feasibility()['status'], 'ok')
                self.assertEqual(cfg.distance_mode, 'resolved_polyline')
                self.assertGreaterEqual(cfg.single_row_base_min_pct - cfg.max_differential_pct,
                                        cfg.breakaway_duty_pct + cfg.inner_margin_pct)
                params = R.nominal_plant_params(plant)
                self.assertAlmostEqual(cfg.v_measured_m_s, round(forward_curve(cfg.base_duty_pct, params), 4))
                self.assertAlmostEqual(cfg.measured_yaw_at_max_diff_deg_s,
                                       2 * cfg.max_differential_pct * params.arc_yaw_deg_s_per_pct_diff)
                self.assertEqual(R.single_row_executor_config(plant).digest(), cfg.digest())   # one per plant

    def test_unknown_plant_is_refused(self):
        with self.assertRaises(ValueError):
            R.single_row_executor_config('p_fit1003')


@unittest.skipUnless(HAVE_WORLD, 'maps / catalogue / PREREG not available')
class EpisodeBuildTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.prereg = R.load_prereg()
        cls.catalogue = R.RC.load_catalogue()

    def build(self, condition, plant='P_fit1003'):
        return R.build_episode(self.prereg, self.catalogue, plant, condition, CORRIDOR, 0)

    def test_every_registered_condition_builds(self):
        for condition, cond in self.prereg['conditions'].items():
            for plant in cond['plants']:
                with self.subTest(condition=condition, plant=plant):
                    ep, meta = self.build(condition, plant)
                    self.assertEqual(ep.start_x_m, 1.)
                    self.assertEqual(meta['cls'], cond['cls'])
                    self.assertEqual(ep.stream_key, meta['corridor'] + '|' + condition)
                    self.assertAlmostEqual(ep.max_time_s, R.RC.time_cap_s(meta['s_E_m'], 1.) + R.TIME_CAP_MARGIN_S, 1)

    def test_condition_parameters_come_from_the_prereg(self):
        ep, meta = self.build('entry_div_neg')
        self.assertEqual((ep.start_y_m, ep.start_yaw_deg), (-.4, -8.))
        ep, _ = self.build('drift_mean_p06')
        overrides = dict(ep.plant_overrides)
        self.assertEqual(overrides['wander_mean_deg_per_m'], .6)                     # the condition wins ...
        self.assertEqual(overrides['arc_yaw_deg_s_per_pct_diff'], .15)               # ... the profile keeps the rest
        ep, _ = self.build('arc006', 'field1010')
        self.assertEqual(dict(ep.plant_overrides), {'arc_yaw_deg_s_per_pct_diff': .06})
        ep, _ = self.build('latency_accel')
        self.assertEqual((ep.command_latency_s, dict(ep.plant_overrides)['accel_tau_s']), (.15, .6))
        ep, _ = self.build('lidar_latency')
        self.assertEqual((ep.lidar_latency_s, ep.lidar_latency_jitter_s, ep.imu_latency_s), (.2, .1, .05))
        ep, meta = self.build('lidar_blackout_1p0')
        self.assertEqual(ep.blackouts, ((25., 26., 'lidar'),))
        self.assertEqual(meta['blackouts'], [[25., 26., 'lidar']])
        ep, meta = self.build('yaw_bursts')
        self.assertEqual(ep.yaw_disturbances[:3], ((15., 15.5, 24.), (35., 35.5, -24.), (55., 55.5, 24.)))
        self.assertLess(ep.yaw_disturbances[-1][0], meta['time_cap_s'])
        ep, meta = self.build('occlusion')
        self.assertEqual(ep.occlusions[0], (0., meta['time_cap_s'], 75., 105.))
        self.assertEqual(ep.occlusions[1:3], ((10., 13., -150., -30.), (25., 28., -150., -30.)))
        _, meta = self.build('gain_left085')
        self.assertEqual(meta['sided_gain'], {'left': .85, 'right': 1.})
        for condition, kind in (('gap2_both', 'gap'), ('branches', 'branch_rect'), ('wall_1p5', 'wall'),
                                ('obstacle_in_row', 'obstacle_box')):
            ep, meta = self.build(condition)
            self.assertTrue(ep.world.modifiers and all(m.kind == kind for m in ep.world.modifiers), condition)
        _, meta = self.build('wall_3p0')
        self.assertEqual(meta['wall_distance_m'], 3.)

    def test_refusals(self):
        with self.assertRaises(KeyError):
            self.build('not_registered')
        with self.assertRaises(ValueError):
            self.build('scan_drop10', 'field1010')                        # registered for P_fit1003 only
        changed = json.loads(json.dumps({k: v for k, v in self.prereg.items() if not k.startswith('_')}))
        changed['conditions']['yaw_bursts']['params']['yaw_disturbances'] = '((15 + 10 k, ...))'
        with self.assertRaisesRegex(ValueError, 'expression changed'):
            R.build_episode(changed, self.catalogue, 'P_fit1003', 'yaw_bursts', CORRIDOR, 0)

    def test_label_digest_matches_the_prereg_selection_rule(self):
        entry, direction, label = R.resolve_corridor(self.catalogue, CORRIDOR)
        self.assertEqual(label, '042301:L5-R4:fwd')
        self.assertEqual(R.resolve_corridor(self.catalogue, '042301:L5-R4:fwd')[0]['corridor_id'], 'c03')
        self.assertRegex(R.label_digest(entry, direction), '^[0-9a-f]{64}$')

    def test_holdout_seeds_need_a_freeze(self):
        args = R.argparse.Namespace(prereg=False, episode=['P_fit1003:nominal:%s:7' % CORRIDOR], corridors=None,
                                    plants=None, conditions=None, seeds=None, freeze=None, selection=None,
                                    seed_set='dev')
        with self.assertRaises(SystemExit):
            R.plan_jobs(args, self.prereg, self.catalogue)
        args.episode = ['P_fit1003:nominal:%s:0' % CORRIDOR]
        self.assertEqual(len(R.plan_jobs(args, self.prereg, self.catalogue)), 1)


def verdict_row(**over):
    row = dict(cls='NOMINAL', plant='P_fit1003', end_kind='TERMINAL_STOP', stop_category='ROW_END', e_stop_m=.2,
               collisions=0, contacts_moving=0, forbidden=dict(pivot_or_reverse=False, auto_restart=False),
               evaluation=dict(safety_band=dict(exceeded=False)), lateral_rmse_m=.05, lateral_p95_m=.1,
               lateral_max_m=.15, heading_rmse_deg=1., heading_max_deg=3., blackouts=[], stop_trigger_time_s=None,
               wall_distance_m=None, wall_clearance_m=None, obstacle_clearance_m=None)
    row.update(over)
    return row


@unittest.skipUnless(HAVE_WORLD, 'PREREG not available')
class VerdictTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.prereg = R.load_prereg()

    def verdict(self, **over):
        return R.prereg_verdict(self.prereg, verdict_row(**over))

    def test_nominal_completion_and_row_end_windows(self):
        self.assertEqual(self.verdict()['verdict'], 'PASS')
        self.assertTrue(self.verdict()['completed'])
        for e, code in ((-2.5, 'ROW_END_FALSE'), (-1.5, 'ROW_END_EARLY'), (1.5, 'ROW_END_LATE')):
            self.assertEqual(self.verdict(e_stop_m=e)['fail_codes'], [code])
        self.assertEqual(self.verdict(lateral_max_m=.51)['fail_codes'], ['LATERAL_MAX'])
        self.assertEqual(self.verdict(heading_rmse_deg=5.1)['fail_codes'], ['HEADING_RMSE'])

    def test_backstop_unclassified_and_zero_tolerance(self):
        self.assertEqual(self.verdict(stop_category='OPERATOR_OR_BACKSTOP', e_stop_m=None)['fail_codes'],
                         ['CATEGORY_NOT_ACCEPTABLE'])
        self.assertEqual(self.verdict(stop_category='UNCLASSIFIED', e_stop_m=None)['fail_codes'], ['UNCLASSIFIED'])
        out = self.verdict(end_kind='MISSED_ROW_END', stop_category=None, contacts_moving=3,
                           forbidden=dict(pivot_or_reverse=True, auto_restart=True))
        self.assertEqual(out['fail_codes'], ['MISSED_ROW_END', 'CONTACT_MOVING', 'PIVOT_OR_REVERSE', 'AUTO_RESTART'])
        self.assertIn('SAFETY_BAND', self.verdict(evaluation=dict(safety_band=dict(exceeded=True)))['fail_codes'])

    def test_class_alternatives_and_the_field1010_bracket(self):
        ca = dict(stop_category='CONTROL_AUTHORITY', e_stop_m=None, lateral_max_m=.59)
        self.assertEqual(self.verdict(cls='WEAK_STEERING', **ca)['verdict'], 'PASS')
        self.assertEqual(self.verdict(cls='ENTRY', **ca)['fail_codes'], ['CATEGORY_NOT_ACCEPTABLE'])
        self.assertEqual(self.verdict(cls='ENTRY', plant='field1010', **ca)['verdict'], 'PASS')     # bracket rule
        self.assertEqual(self.verdict(cls='NOMINAL', plant='field1010', **ca)['fail_codes'], ['CATEGORY_NOT_ACCEPTABLE'])

    def test_sensor_stop_window(self):
        perc = dict(cls='SENSOR', stop_category='PERCEPTION_OR_LOCALISATION', e_stop_m=None,
                    blackouts=[[25., 26., 'lidar']])
        self.assertEqual(self.verdict(stop_trigger_time_s=25.4, **perc)['verdict'], 'PASS')
        self.assertEqual(self.verdict(stop_trigger_time_s=31.5, **perc)['fail_codes'], ['PERCEPTION_STOP_WINDOW'])
        self.assertEqual(self.verdict(stop_trigger_time_s=40., **dict(perc, blackouts=[]))['verdict'], 'PASS')

    def test_end_structure_and_obstacle(self):
        es = dict(cls='END_STRUCTURE', wall_distance_m=1.5, wall_clearance_m=.5)
        self.assertEqual(self.verdict(stop_category='END_STRUCTURE', e_stop_m=1.1, **es)['verdict'], 'PASS')
        self.assertEqual(self.verdict(stop_category='END_STRUCTURE', e_stop_m=1.3, **es)['fail_codes'],
                         ['END_STRUCTURE_POSITION'])
        self.assertEqual(self.verdict(stop_category='OBSTACLE', e_stop_m=1., **es)['fail_codes'],
                         ['CATEGORY_NOT_ACCEPTABLE'])
        self.assertIn('WALL_CLEARANCE', self.verdict(**dict(es, wall_clearance_m=.2))['fail_codes'])
        # a correct row-end stop 3 m before a wall: the at-rest clearance is measured with a 6 m reach; None = no wall
        # cell within it (> 6 m), never a failed clearance (dev round 1 evaluator fix)
        far = self.verdict(**dict(es, wall_distance_m=3., wall_clearance_m=None, e_stop_m=0.))
        self.assertEqual(far['verdict'], 'PASS')
        self.assertIn('WALL_BEYOND_6_M', far['verdict_notes'])
        far_obstacle = dict(cls='OBSTACLE', stop_category='OBSTACLE', e_stop_m=None, obstacle_clearance_m=None)
        self.assertEqual(self.verdict(**far_obstacle)['fail_codes'], ['OBSTACLE_CLEARANCE'])   # > 6 m: not its stop
        ob = dict(cls='OBSTACLE', stop_category='OBSTACLE', e_stop_m=None)
        self.assertEqual(self.verdict(obstacle_clearance_m=.5, **ob)['verdict'], 'PASS')
        self.assertEqual(self.verdict(obstacle_clearance_m=.2, **ob)['fail_codes'], ['OBSTACLE_CLEARANCE'])
        self.assertEqual(self.verdict(cls='OBSTACLE', obstacle_clearance_m=.5)['fail_codes'], ['CATEGORY_NOT_ACCEPTABLE'])


def run_short(condition, policy='prereg_fixed', max_time_s=8., plant='P_fit1003', overrides=None):
    prereg, catalogue = R.load_prereg(), R.RC.load_catalogue()
    _, meta = R.build_episode(prereg, catalogue, plant, condition, CORRIDOR, 0)
    meta['corridor_spec'] = CORRIDOR
    meta.pop('sided_gain')
    options = dict(run_id='test', run_kind='smoke', prereg_path=prereg['_path'],
                   catalogue_path=str(R.RC.catalogue_path()), row_length='none', press_policy=policy,
                   config_overrides={plant: dict(overrides or {})}, trace_dir=None, video=None, video_dir=None,
                   max_time_s=max_time_s)
    return R.run_job(dict(episode_id=R.episode_id(meta), meta=meta, options=options))


@unittest.skipUnless(HAVE_WORLD, 'maps / catalogue / PREREG not available')
class EpisodeEndTest(unittest.TestCase):
    def test_end_waits_for_rest_observation_and_a_quiet_burst_window(self):
        import types
        prereg, catalogue = R.load_prereg(), R.RC.load_catalogue()
        ep, _ = R.build_episode(prereg, catalogue, 'P_fit1003', 'yaw_bursts', CORRIDOR, 0)
        cfg = R.single_row_executor_config('P_fit1003')
        truth = types.SimpleNamespace(v=0., w=0., external_yaw_rate=0.)
        plant_cls = types.SimpleNamespace(last=truth)
        facade = R.SingleRowInTheLoop(ep, cfg, None, R.SIL.RealFormat(cfg.lidar_yaw_offset_deg),
                                      R.SIL.rover_control_class(), plant_cls)
        facade.stop = dict(t=10., state='ENDED', reason='row_end')
        signalled = None
        for k in range(0, 400):
            now = round(10. + .05 * k, 2)
            truth.external_yaw_rate = math.radians(24.) if 15. <= now < 15.5 else 0.    # the registered first burst
            facade._facade_state(now)
            if facade.hold_terminal:
                signalled = now
                break
        # rest since 10.0 would end at 14.0, but the 15.0 s burst falls inside the harness' last second: the burst
        # resets the rest (true body yaw incl. the external rate) and the signal follows 4.0 s after it
        self.assertEqual(signalled, 19.5)
        self.assertEqual((facade.state, facade.reason), ('HOLD', 'row_end'))


@unittest.skipUnless(HAVE_WORLD, 'maps / catalogue / PREREG not available')
class RasterLoopTest(unittest.TestCase):
    def test_wiring_real_executor_on_a_raster_corridor(self):
        row = run_short('nominal', max_time_s=10.)
        self.assertIsNone(row.get('error'), row.get('trace_tail'))
        ex = row['executor']
        self.assertGreater(ex['scans_processed'], 60)
        self.assertGreater(ex['sends'], 0)
        self.assertEqual((ex['bridge_errors'], ex['deadman_false'], ex['guard_violations'], ex['presses']), (0, 0, 0, 1))
        self.assertIsNotNone(ex['first_drive_t'])
        self.assertEqual(row['localisation_path'], 'SENSOR_ONLY_SINGLE_ROW')
        self.assertFalse(row['forbidden']['pivot_or_reverse'])
        self.assertEqual(row['executor']['backstop']['single_row']['row_len_m'], None)       # no row length
        self.assertGreater(row['step_ms']['n'], 100)
        for key in ('e_stop_m', 'row_end_outcome', 'overshoot_m', 'lateral_rmse_m', 'heading_max_deg', 'sat_fraction',
                    'pre_stop_speed_ratio', 'unknown_overlap_samples', 'stop_trigger_time_s', 'verdict', 'fail_codes'):
            self.assertIn(key, row)

    def test_first_pause_is_the_stop_no_repress_and_episode_end(self):
        row = run_short('lidar_blackout_1p0', max_time_s=45.)
        self.assertIsNone(row.get('error'), row.get('trace_tail'))
        self.assertEqual(row['stop_trigger']['reason'], 'stale_lidar')
        self.assertTrue(25. <= row['stop_trigger_time_s'] <= 26.5)
        self.assertEqual((row['executor']['presses'], row['executor']['resumes']), (1, 1))     # no re-press
        self.assertEqual(row['harness_end'], 'terminal_hold')
        self.assertGreaterEqual(row['terminal_signal_t_s'] - row['stop_trigger_time_s'], R.REST_THEN_OBSERVE_S - .05)
        self.assertEqual((row['end_kind'], row['stop_category']), ('TERMINAL_STOP', 'PERCEPTION_OR_LOCALISATION'))
        self.assertFalse(row['forbidden']['auto_restart'])

    def test_press_policy_against_lidar_latency(self):
        # PREREG operator policy (one press at 1.2 s): with 0.2-0.3 s LiDAR latency the inputs are fresh for < 1.0 s
        # then. Without the single-row initial-press grace the Runner ignores the press and the PREREG allows no
        # re-press (dev round 1: 16/16 lidar_latency episodes never started); with the grace (default 1 s) the held
        # press starts the run once the inputs are fresh.
        fixed = run_short('lidar_latency', 'prereg_fixed', 8., overrides=dict(single_row_initial_press_grace_s=0.))
        self.assertEqual(fixed['stop_trigger']['reason'], 'press_ignored_inputs_not_fresh')
        self.assertIsNone(fixed['executor']['first_drive_t'])
        held = run_short('lidar_latency', 'prereg_fixed', 8.)
        self.assertEqual(held['press_t_s'], R.PRESS_AT_S)
        self.assertIsNone(held['stop_trigger'])
        self.assertIsNotNone(held['executor']['first_drive_t'])
        self.assertEqual(held['executor']['presses'], 1)
        ready = run_short('lidar_latency', 'page_ready', 8.)
        self.assertGreater(ready['press_t_s'], R.PRESS_AT_S)
        self.assertIsNotNone(ready['executor']['first_drive_t'])
        self.assertIsNone(ready['stop_trigger'])


if __name__ == '__main__':
    unittest.main()
