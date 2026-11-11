"""T1 executor-in-the-loop F-SIM of the front-steer vehicle (tools/row_executor/sim_in_loop_steer.py, design 8.3 P4):
the SIM executor config (front_steer GO rules with the profile placeholders), one short episode end to end (arm, press,
drive, operator STOP, STOP confirmation, steering hold, no restart), determinism (same seed twice, another seed), the
sensor format the executor receives (RealFormat), the trace columns of the evaluator (truth_hold_position_rad) and the
executor-trace evaluation, the sealed-corridor refusal.

Needs the native steering program in firmware mode (built once into UBTS_STEER_BUILD_DIR or a temporary folder, never
into the work tree) and the dashboard host modules; without them the cases are skipped with the reason. Every number is
a placeholder of the SIM (자리값 기반 SIM 결과).
"""
import atexit
import math
import os
import shutil
import tempfile
import unittest

import rowexec_testlib as L  # noqa: F401  (path setup)

from row_executor import config as C

try:
    from row_executor import sim_in_loop_steer as S       # puts sim/ubts on the path
    from steering import t1_rig as TR
    from steering.native import build as native_build
    from steering.s2_format import BEAMS, RAW_AMIN, RAW_INC
    IMPORT_PROBLEM = None
except Exception as exc:              # scipy of the SIM tree estimator (decision R10: the WSL Python 3.10 scipy)
    IMPORT_PROBLEM = 'sim_in_loop_steer not importable here (decision R10): %s: %s' % (type(exc).__name__, exc)

_STATE = {}
POSTS = [[round(-6. + 1.5 * k, 3), side * 1.75, .12] for k in range(20) for side in (1, -1)]


def _cleanup(path):
    shutil.rmtree(path, ignore_errors=True)


def native_or_skip():
    if IMPORT_PROBLEM:
        raise unittest.SkipTest(IMPORT_PROBLEM)
    if 'native' not in _STATE:
        try:
            TR.host_modules()
            if not native_build.firmware_present():
                raise native_build.NativeBuildError(native_build.missing_firmware_message())
            native_build.find_compiler()
            folder = os.environ.get(native_build.ENV_BUILD_DIR)
            if not folder:
                folder = tempfile.mkdtemp(prefix='steer_exec_loop_build_')
                atexit.register(_cleanup, folder)
            _STATE['native'] = native_build.ensure_built(build_dir=folder).path
        except (native_build.NativeBuildError, TR.RigError) as exc:
            _STATE['native'] = 'native steering program or host modules not available: %s' % exc
    value = _STATE['native']
    if not os.path.isfile(value):
        if native_build.firmware_required():
            raise AssertionError('%s is set: %s' % (native_build.ENV_REQUIRE_FIRMWARE, value))
        raise unittest.SkipTest(value)
    return value


def stop_spec(name='exec_stop', seed=0, **over):
    spec = dict(name=name, seed=seed, world={'kind': 'W0', 'markers': POSTS},
                controller={'kind': 'straight_hold', 'drive_pct': 40, 'distance_m': None},
                actions=[{'op': 'stop', 'at_s': 4.}], duration_s=7., post_end_s=None, auto_repress=False)
    spec.update(over)
    return spec


class ConfigTest(unittest.TestCase):
    def test_sim_config_is_front_steer_motor_and_feasible(self):
        native = native_or_skip()
        spec = S.normalise_spec(stop_spec())
        rig = S.build_rig(spec, native)
        try:
            cfg = S.steer_executor_config(rig)
        finally:
            rig.close()
        self.assertEqual((cfg.vehicle_profile, cfg.mode, cfg.motor_output_enabled), ('front_steer', 'single_row', True))
        self.assertEqual(C.feasibility(cfg)['status'], 'ok')
        self.assertTrue(cfg.steer_allow_sim)
        self.assertEqual(cfg.steer_cal_id, rig.config.cal_id)
        self.assertEqual((cfg.steer_limit_left_mrad, cfg.steer_limit_right_mrad),
                         (rig.hp.limit_left_mrad, rig.hp.limit_right_mrad))
        self.assertLessEqual(cfg.steer_rate_limit_deg_s, C.STEER_RATE_FRACTION * cfg.steer_measured_rate_deg_s + 1e-9)
        self.assertAlmostEqual(cfg.steer_rotation_centre_x_m, float(rig.profile.value('rear_axle_x_in_base_m')))

    def test_sealed_corridor_is_refused_before_anything_opens(self):
        native = native_or_skip()
        spec = S.normalise_spec(stop_spec(world={'kind': 'W2', 'record': '042301', 'corridor_id': 'c01',
                                                 'direction': 'fwd'}))
        from steering.scene_spec import SceneError
        with self.assertRaises((TR.RigError, SceneError)) as ctx:      # the scene spec refuses it, the rig again
            S.build_rig(spec, native)
        self.assertIn('sealed', str(ctx.exception))


class EpisodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = native_or_skip()
        cls.result, cls.facade = S.run_episode(stop_spec(capture_inputs=30), None, cls.native, keep=True)
        cls.arrays = cls.facade.trace_arrays()
        cls.summary = cls.facade.summary()
        cls.inputs = cls.facade.inputs
        cls.facade.rig.close()

    def test_run_ends_by_the_operator_stop_and_the_stop_is_confirmed(self):
        s = self.summary
        self.assertEqual(s['end_reason'], 'operator_stop')
        states = [e[1] for e in s['events'] if e[1] in ('ACTIVE', 'ENDED')]
        self.assertEqual(states[0], 'ACTIVE')
        self.assertEqual(states[-1], 'ENDED')
        names = [e.get('event') for e in s['executor_events']]
        self.assertIn('emergency_stop', names)
        self.assertIn('stop_confirmed', names)

    def test_drive_and_steering_after_the_stop(self):
        t = self.arrays['t_s']
        after = t > self.summary['ended_at_s'] + 1.
        self.assertTrue(after.any())
        self.assertTrue((self.arrays['drive_applied_pct'][after] == 0).all())
        self.assertLess(abs(float(self.arrays['truth_v_m_s'][-1])), .01)
        hold = self.arrays['truth_hold_position_rad'][after]
        true = self.arrays['truth_delta_eq_rad'][after]
        self.assertLess(float(abs(true - hold).max()), math.radians(1.))

    def test_executor_commanded_throttle_and_steering(self):
        throttle = self.arrays['ctrl_throttle_pct']
        self.assertTrue((throttle > 0).any())
        self.assertIn('ctrl_steer_mrad', self.arrays)
        self.assertIn('truth_hold_position_rad', self.arrays)

    def test_sensor_messages_have_the_real_format(self):
        scans, imus = self.inputs['scans'], self.inputs['imus']
        self.assertTrue(scans and imus)
        msg = scans[0]['msg']
        self.assertEqual(len(msg.ranges), BEAMS)
        self.assertEqual((msg.angle_min, msg.angle_increment, msg.header.frame_id), (RAW_AMIN, RAW_INC, 'laser'))

    def test_executor_trace_evaluation(self):
        from steering import evaluate_steer as E
        spec = S.normalise_spec(stop_spec())
        ev = E.evaluate_executor_trace(self.arrays, spec, self.summary)
        self.assertEqual(ev['episode']['end_reason'], 'operator_stop')
        self.assertEqual(ev['contacts']['collision_samples'], 0)
        after = ev['forbidden']['steer_after_terminal']
        self.assertTrue(after['evaluated'])
        self.assertEqual(after['hold_position_source'], 'truth_hold_position')     # R13-3: the executor trace column
        self.assertFalse(after['flag'])


class ReviewFixEpisodeTest(unittest.TestCase):
    """Review RS-01 (a TTL event no status line shows) and P4R-05 / RS-04 (a real bridge process restart)."""

    def drive_changes(self, facade):
        return [(int(t), int(v)) for t, v in facade.rig.mcu_channel('drive_l')]

    def test_ttl_event_without_a_status_bit_ends_the_run_before_a_restart(self):
        native = native_or_skip()
        # the status lines 10 ms earlier (boot delay 60 ms) and the C lines lost 4.97 .. 5.29 s: the C written when the
        # link returns reaches the MCU before the next status line, so no status line carries the TTL bit
        spec = stop_spec('exec_ttl_hidden', actions=[], duration_s=8., rig={'boot_delay_ticks': 60},
                         faults=[{'kind': 'link_loss', 'at_s': 4.97, 'params': {'duration_s': .32,
                                                                                 'direction': 'to_mcu'}}])
        result, facade = S.run_episode(spec, None, native, keep=True)
        try:
            summary = facade.summary()
            changes = self.drive_changes(facade)
            ttl = [line for line in facade.rig.mcu_lines(('F',)) if line.text.startswith('F TTL')]
        finally:
            facade.rig.close()
        self.assertTrue(ttl)
        t_ttl = ttl[0].queued_tick
        self.assertEqual(summary['end_reason'], 'link_ttl_expired')
        self.assertLess(summary['ended_at_s'], t_ttl / 1000. + .12)          # before the 120 ms rest could end
        after = [(t, v) for t, v in changes if t > t_ttl]
        self.assertEqual([v for _, v in after if v], [])                       # the drive never restarts

    def test_bridge_restart_is_a_new_process_without_a_lease(self):
        native = native_or_skip()
        spec = stop_spec('exec_bridge_restart', actions=[], duration_s=8.,
                         faults=[{'kind': 'bridge_restart', 'at_s': 4., 'params': {'down_s': 2.}}])
        result, facade = S.run_episode(spec, None, native, keep=True)
        try:
            rig = facade.rig
            summary = facade.summary()
            core = rig.core
            processes = rig.bridge_processes
            events = list(rig.bridge_events)
            lost = rig.counters['requests_without_bridge'] + rig.counters['requests_lost_with_bridge']
            changes = self.drive_changes(facade)
        finally:
            facade.rig.close()
        self.assertEqual(processes, 2)
        self.assertEqual([kind for kind, _ in events], ['exit', 'start'])
        self.assertGreater(lost, 0)
        self.assertIn(summary['end_reason'], ('bridge_error', 'bridge_latched'))
        self.assertIsNone(core.control.lease)                                  # the new process holds no lease
        self.assertEqual(core.latches, {})
        restart = events[1][1]
        self.assertEqual([v for t, v in changes if t > restart and v], [])     # no drive after the new process


class DeterminismTest(unittest.TestCase):
    def test_same_seed_same_digests_other_seed_differs(self):
        native = native_or_skip()
        spec = stop_spec(duration_s=5.)
        a = S.run_episode(dict(spec), None, native)['digests']
        b = S.run_episode(dict(spec), None, native)['digests']
        c = S.run_episode(dict(spec, seed=3), None, native)['digests']
        self.assertEqual(a, b)
        self.assertNotEqual(a['serial_log_sha256'], c['serial_log_sha256'])
        self.assertNotEqual(a['trace_content_sha256'], c['trace_content_sha256'])


if __name__ == '__main__':
    unittest.main()
