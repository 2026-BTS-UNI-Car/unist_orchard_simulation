"""Generated memory-provider tests. No native SDK/map/ROS/HTTP/hardware claim."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest

TESTS = Path(__file__).resolve().parent
REPO = TESTS.parents[2]
for root in (TESTS, REPO / 'tools', REPO / 'sim/ubts'):
    sys.path.insert(0, str(root))
sys.dont_write_bytecode = True

from row_executor import sim_in_loop as S
from row_executor.navigation_observation import (NativeSourceBinding, NativePose2D,
    NativeNavigationObservation, NativeNavigationGate)
from test_executor_loop import Rig
from closed_loop.harness import EpisodeConfig
from closed_loop.world import WorldConfig

BINDING = NativeSourceBinding('a' * 64, 'b' * 64, 'c' * 64, 'map')


def observation(sequence=1, **kwargs):
    value = NativeNavigationObservation(BINDING, sequence, 0., 'TRACKING',
        NativePose2D(0., 0., 0., age_s=0., quality='QUALIFIED_LOCAL'), True, 100, 'CLEAR', sequence)
    return replace(value, **kwargs)


class FakeMapProvider:
    """Declarative generated fixture, never a native adapter or real map."""
    def __init__(self, gate):
        self.gate = gate;self.sequence = 0;self.enabled = True
        self.fields = {}

    def publish(self, now):
        if self.enabled:
            self.sequence += 1
            return self.gate.offer(observation(self.sequence, **self.fields), now)
        return False


class PublishingRig(Rig):
    def __init__(self, provider=True, test_mode=True, required=True):
        cfg = replace(S.executor_config(plant_name='field1010'), require_native_navigation=required)
        gate = NativeNavigationGate(BINDING, cfg.lidar_max_age_s, test_only=True) if provider else None
        self.provider = FakeMapProvider(gate) if gate else None
        super().__init__(cfg=cfg, motor=True, navigation_gate=gate,
                         navigation_test_mode=test_mode if gate else False,
                         runner_factory=lambda c: S.XCORE.Runner(c, vendor_modules=(S.SE, S.SC)))

    def run(self, seconds, hold=None):
        out = []
        for _ in range(int(round(seconds / .05))):
            if self.provider:self.provider.publish(self.clock.t)
            out.extend(super().run(.05, hold))
        return out

    @property
    def duty(self):
        for call in reversed(self.motor.calls):
            if call == 'STOP':return (0, 0)
            if type(call) is tuple and len(call) == 2:return call
        return (0, 0)

    def begin(self):
        start = self.loop.submit('start', row_len_m=30.)
        self.run(1.5)
        assert start.result()['ok']
        arm = self.loop.submit('arm');self.run(.1)
        assert arm.result()['ok']
        self.press('old-press')
        return self.run(1.2, hold=True)


class MemoryContract(unittest.TestCase):
    def test_admitted_source_age_and_generated_scope_are_read_only(self):
        gate = NativeNavigationGate(BINDING, .45, test_only=True)
        for field, value in (('expected', replace(BINDING, map_frame='other')),
                ('max_age_s', 30.), ('test_only', False)):
            with self.assertRaises(AttributeError):setattr(gate, field, value)

    def test_missing_and_unqualified_provider_never_become_ready(self):
        g = NativeNavigationGate(BINDING, .45)
        self.assertTrue(g.offer(observation(), 1.))
        self.assertFalse(g.evaluate(1.)['allow_motion'])
        self.assertEqual(g.evaluate(1.)['reason'], 'ADAPTER_UNQUALIFIED')

    def test_same_frame_repeat_cannot_refresh_age(self):
        g = NativeNavigationGate(BINDING, .45, test_only=True);o = observation()
        self.assertTrue(g.offer(o, 1.));self.assertFalse(g.offer(o, 1.4))
        self.assertEqual(g.evaluate(1.46)['reason'], 'STALE')

    def test_changed_same_sequence_and_source_are_fenced(self):
        for altered, reason in ((observation(forward_hazard='OCCUPIED'), 'SAME_SEQUENCE_CHANGED'),
                (observation(2, binding=replace(BINDING, publisher_epoch_SHA256='d' * 64)), 'SOURCE_CHANGED')):
            g = NativeNavigationGate(BINDING, .45, test_only=True);g.offer(observation(), 1.)
            self.assertFalse(g.offer(altered, 1.1));self.assertEqual(g.evaluate(1.1)['reason'], reason)
            self.assertFalse(g.offer(observation(3), 1.2))

    def test_source_age_and_pose_age_are_independent_of_receipt_freshness(self):
        for altered, reason in ((observation(source_age_s=.5), 'STALE'),
                (observation(pose=NativePose2D(0., 0., 0., age_s=.5, quality='QUALIFIED_LOCAL')), 'POSE_STALE'),
                (observation(pose=NativePose2D(0., 0., 0.)), 'POSE_UNCERTAIN')):
            g = NativeNavigationGate(BINDING, .45, test_only=True);g.offer(altered, 1.)
            self.assertEqual(g.evaluate(1.)['reason'], reason)

    def test_unknown_incomplete_wrong_pose_and_occupied_costs_are_not_free(self):
        for row, reason in ((observation(occupancy_complete=False), 'MAP_INCOMPLETE'),
                (observation(known_cells=0), 'MAP_INCOMPLETE'),
                (observation(forward_hazard='UNKNOWN'), 'COST_UNKNOWN'),
                (observation(forward_hazard='OCCUPIED'), 'COST_OCCUPIED'),
                (observation(cost_pose_sequence=2), 'COST_POSE_MISMATCH')):
            g = NativeNavigationGate(BINDING, .45, test_only=True);g.offer(row, 1.)
            self.assertEqual(g.evaluate(1.)['reason'], reason)

    def test_wait_lost_missing_pose_do_not_allow_motion(self):
        for row, reason in ((observation(tracking='WAIT'), 'POSE_WAIT'),
                (observation(tracking='LOST'), 'POSE_LOST'),
                (observation(pose=None), 'POSE_UNKNOWN')):
            g = NativeNavigationGate(BINDING, .45, test_only=True);g.offer(row, 1.)
            self.assertEqual(g.evaluate(1.)['reason'], reason)

    def test_counter_and_clock_failures_remain_closed(self):
        g = NativeNavigationGate(BINDING, .45, test_only=True)
        self.assertFalse(g.offer(observation(sequence=True), 1.))
        self.assertEqual(g.evaluate(1.)['reason'], 'INVALID_OBSERVATION')
        g = NativeNavigationGate(BINDING, .45, test_only=True);g.offer(observation(), 2.);g.evaluate(2.)
        self.assertEqual(g.evaluate(1.9)['reason'], 'CLOCK_REGRESSION')

    def test_status_excludes_local_pose_and_source_capabilities(self):
        g = NativeNavigationGate(BINDING, .45, test_only=True);g.offer(observation(), 1.)
        value = g.evaluate(1.)
        self.assertTrue(value['allow_motion'])
        for key in ('pose', 'binding', 'x_m', 'y_m', 'yaw_rad', 'token', 'lease'):
            self.assertNotIn(key, value)
        self.assertFalse(value['SDK_adapter_qualified'])


class ActualExecutorAdmission(unittest.TestCase):
    def test_latest_profile_with_missing_provider_waits_despite_fresh_raw_sensors(self):
        rig = PublishingRig(provider=False)
        try:
            out = rig.begin()
            self.assertTrue(all((d.left, d.right) == (0, 0) for d in out))
            self.assertEqual(rig.duty, (0, 0))
            self.assertEqual(rig.loop.status()['native_navigation']['reason'], 'PROVIDER_MISSING')
        finally:rig.loop.shutdown('generated_test')


class GeneratedPlantMapCostLoop(unittest.TestCase):
    def test_fake_map_hazard_inhibits_real_executor_and_10_10_plant_loop(self):
        cfg = replace(S.executor_config(plant_name='field1010'), require_native_navigation=True)
        ep = EpisodeConfig(name='generated_map_cost_loop', seed=4301,
            world=WorldConfig(seed=4301, branches=False, missing_probability=0.), max_time_s=5.,
            controller_overrides=tuple(sorted(cfg.controller_settings()[1].items())))
        gate = NativeNavigationGate(BINDING, cfg.lidar_max_age_s, test_only=True)
        provider = FakeMapProvider(gate)

        class Facade(S.ExecutorInTheLoop):
            def snapshot(self, now):
                provider.fields = {'forward_hazard': 'OCCUPIED'} if now >= 3. else {}
                provider.publish(now)
                return super().snapshot(now)

        facade = Facade(ep, cfg, 30., S.RealFormat(cfg.lidar_yaw_offset_deg), S.rover_control_class(),
                       navigation_gate=gate, navigation_test_mode=True)
        try:
            result = S.SH.run_episode(ep, record_trace=True,
                estimator_factory=lambda _cfg: facade, controller_factory=lambda _cfg: facade)
            self.assertGreater(result['truth_distance_m'], .02)
            self.assertTrue(any(row['left'] or row['right'] for row in result['trace'] if row['t'] < 3.))
            self.assertTrue(all((row['left'], row['right']) == (0, 0) for row in result['trace'] if row['t'] >= 3.))
            self.assertEqual(facade.motor.duty, (0, 0))
            self.assertEqual(facade.loop.status()['native_navigation']['reason'], 'COST_OCCUPIED')
            self.assertGreater(facade.loop.counts['scans_processed'], 30)
            self.assertEqual(result['collisions'], 0)
        finally:facade.loop.shutdown('generated_test')

    def test_generated_provider_is_rejected_without_explicit_simulation_injection(self):
        rig = PublishingRig(test_mode=False)
        try:
            rig.begin()
            self.assertEqual(rig.duty, (0, 0))
            self.assertEqual(rig.loop.status()['native_navigation']['reason'], 'GENERATED_PROVIDER_FORBIDDEN')
        finally:rig.loop.shutdown('generated_test')

    def test_map_cost_pause_recovery_requires_a_genuinely_new_press(self):
        rig = PublishingRig()
        try:
            self.assertTrue(any(d.left or d.right for d in rig.begin()))
            rig.provider.fields = {'forward_hazard': 'OCCUPIED'};rig.run(.1, hold=True)
            self.assertEqual(rig.duty, (0, 0))
            self.assertEqual(rig.loop.status()['native_navigation']['reason'], 'COST_OCCUPIED')
            rig.provider.fields = {};rig.run(.7, hold=True)
            self.assertEqual(rig.duty, (0, 0))
            rig.release();rig.press('new-clear-map-press');rig.run(.3, hold=True)
            self.assertNotEqual(rig.duty, (0, 0))
            rig.loop.emergency_stop('map_test');self.assertEqual(rig.duty, (0, 0))
            rig.run(.05, hold=True);self.assertEqual(rig.loop.bridge.state, 'LATCHED')
        finally:rig.loop.shutdown('generated_test')

    def test_map_staleness_stops_even_while_raw_sensor_callbacks_continue(self):
        rig = PublishingRig()
        try:
            rig.begin();before = rig.loop.counts['scans_processed']
            rig.provider.enabled = False;rig.run(.6, hold=True)
            self.assertGreater(rig.loop.counts['scans_processed'], before)
            self.assertEqual(rig.duty, (0, 0))
            self.assertEqual(rig.loop.status()['native_navigation']['reason'], 'STALE')
        finally:rig.loop.shutdown('generated_test')

    def test_legacy_sensor_only_path_is_explicit_not_latest_default(self):
        self.assertTrue(S.XC.ExecutorConfig().require_native_navigation)
        rig = PublishingRig(provider=False, required=False)
        try:
            self.assertTrue(any(d.left or d.right for d in rig.begin()))
            self.assertEqual(rig.loop.status()['native_navigation']['state'], 'LEGACY_SENSOR_ONLY_DEVELOPMENT')
        finally:rig.loop.shutdown('generated_test')


if __name__ == '__main__':
    unittest.main(verbosity=2)
