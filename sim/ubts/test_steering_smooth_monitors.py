"""Schema v6: the vehicle's monitor stack (steer_monitor_stack esp32_r5) and the SIM proposal r6p of the SMOOTH hold
(docs/steering/SMOOTH_R5_DISTURBANCE_KO.md).

Checks: front_steer_v2_1_field_20261117 differs from front_steer_v2_field_20261116 only in the listed records, and
front_steer_v2_2_proposal_20261117 from v2.1 only in its listed records; every earlier profile keeps its host profile
bytes, its startup steer_law block and its CFG values (the values of main 54f9c34); esp32_r5 writes the CFG values that
switch the V3 MCU monitors off and the host track rule esp32_adapter; the schema v6 checks; the native parser takes kind
smooth_r6p only with the proposal keys; on the T1 rig a 2.5 N·m steering torque pushes the r5 hold to the end stop and
the host latches steering_not_following (vehicle stack), while the proposal holds the steering.

Every number is a placeholder-based SIM result (자리값 기반 SIM 결과); the proposal is not the vehicle's firmware.
unittest style (pytest or python -m unittest). The native program is built into UBTS_STEER_BUILD_DIR or a temporary
folder.
"""
import hashlib
import json
import math
import unittest

from steering import abs_hold
from steering import host_profile_export as hpe
from steering import smooth_hold
from steering import vehicle_profile as vp
from steering.native import fw_eeprom

import test_steering_host_profile_export as hostexport
import test_steering_t1_rig as rigtest

V2 = 'front_steer_v2_field_20261116'
V21 = 'front_steer_v2_1_field_20261117'
V22 = 'front_steer_v2_2_proposal_20261117'
V21_CHANGED = {'host_track_mismatch', 'steer_fw_mode'}
V21_NEW = {'steer_monitor_stack', 'smooth_fw_proposal'}
V22_CHANGED = {'steer_fw_mode', 'smooth_fw_proposal'}
# sha256 of the host profile bytes, of the sorted JSON of the startup steer_law block and of the sorted JSON of the CFG
# values (scale 50 permille/deg) of every profile at main 54f9c34 (before schema v6)
IDENTITY_54F9C34 = {
    'front_steer_v0_1_placeholder': ('f5f1fc6a9918f3f9e69ddc17955ca3701f3a6fc1d91a4b8fcd0fd13b062a59b0', None,
                                     '761cdf989c61dfe6c469a08a871ea8d518ac99c551d563b1dfbe734254c98527'),
    'front_steer_v0_3_placeholder': ('3cf6e3d47777a27f7e4efa3ee5622b1dde131e6e93995ddba99b8d330cab614b', None,
                                     '10c44348bb2ac23fe5672bed255caebc36b0e61dd1f4ce77fb42767dee2e27d6'),
    'front_steer_v1_spec_20261116': ('5f3295071efe1ef75c65023bab932176c5160264efc1acce8555785f00a684fc',
                                     'b26c814916b7d96f79a85c1b27e6c024448686dc0216a8be317669a5996b50fe',
                                     '2bfc2dbf4e3fbcdc3e7eab559df2a264f4f56124ed8a41131eeb8aab45fdbf4f'),
    V2: ('3115bba8ef6d407f1ef5a73b3be82f7e4ade47cd8dfdc6ed2c9e2a468c709893',
         '3e30c57e8e980611f1158ee22d14c687794b85af90a98cb40228cea8a78b6421',
         'b8fbc8974de5dc83c9544506e3052359075d3b0028b980b64a933296913afc8d'),
}


def load(profile_id):
    return vp.load_profile(vp.PROFILES_DIR / (profile_id + '.yaml'))


def identity(profile):
    host = hashlib.sha256(hpe.to_bytes(hpe.build(profile))).hexdigest()
    block = abs_hold.startup_block(profile) or smooth_hold.startup_block(profile)
    block = None if block is None else hashlib.sha256(json.dumps(block, sort_keys=True).encode()).hexdigest()
    params = fw_eeprom.params_from_profile(profile, 50.)
    return host, block, hashlib.sha256(json.dumps(params, sort_keys=True).encode()).hexdigest()


class ProfileTest(unittest.TestCase):
    def test_v2_1_differs_from_v2_only_in_these_records(self):
        new, old = load(V21), load(V2)
        self.assertEqual((new.schema, new.schema_version), (vp.SCHEMA_V6, 6))
        self.assertEqual(set(new.names()) - set(old.names()), V21_NEW)
        self.assertEqual(set(old.names()) - set(new.names()), set())
        self.assertEqual({n for n in old.names() if new.param(n) != old.param(n)}, V21_CHANGED)
        self.assertEqual(new.value('steer_fw_mode'), 'smooth_r5')                       # the value stays, the choices grow
        self.assertEqual(new.value('host_track_mismatch'), dict(error_deg=3.0, duration_s=2.0))   # ESP32_TRACK
        self.assertEqual(new.value('steer_monitor_stack'), 'esp32_r5')
        self.assertEqual(set(new.value('smooth_fw_proposal').values()), {0})
        self.assertEqual(new.warnings(), [])
        self.assertEqual(new.result_label()['result_kind'], vp.RESULT_LABEL_PLACEHOLDER)

    def test_v2_2_differs_from_v2_1_only_in_these_records(self):
        new, old = load(V22), load(V21)
        self.assertEqual(set(new.names()), set(old.names()))
        self.assertEqual({n for n in old.names() if new.param(n) != old.param(n)}, V22_CHANGED)
        self.assertEqual(new.value('steer_fw_mode'), 'smooth_r6p')
        self.assertEqual(new.value('smooth_fw_proposal'), dict(hold_pwm_max=200, push_counts=24, rest_brake=1,
                                                               hold_no_cancel=1))
        self.assertTrue(smooth_hold.proposal(new))
        self.assertFalse(smooth_hold.proposal(old))
        self.assertEqual(smooth_hold.executor_values(new), smooth_hold.executor_values(old))   # same measured steps

    def test_earlier_profiles_keep_their_identity(self):
        for profile_id, expected in IDENTITY_54F9C34.items():
            with self.subTest(profile=profile_id):
                p = load(profile_id)
                self.assertEqual(fw_eeprom.monitor_stack(p), 'v3')
                self.assertNotIn('track_rule', hpe.build(p)['host_faults'])
                self.assertEqual(identity(p), expected)

    def test_vehicle_stack_values(self):
        new, old = load(V21), load(V2)
        params, before = fw_eeprom.params_from_profile(new, 50.), fw_eeprom.params_from_profile(old, 50.)
        self.assertEqual({k: params[k] for k in fw_eeprom.BOARD_R5_MONITORS_OFF}, fw_eeprom.BOARD_R5_MONITORS_OFF)
        self.assertEqual({k for k in params if params[k] != before[k]}, set(fw_eeprom.BOARD_R5_MONITORS_OFF))
        self.assertEqual((params['pvmin'], params['pvmax']), (before['pvmin'], before['pvmax']))   # POT_OPEN / SHORT
        data = hpe.build(new)
        self.assertEqual(data['host_faults']['track_rule'], 'esp32_adapter')
        self.assertEqual((data['host_faults']['track_error_deg'], data['host_faults']['track_duration_s']), (3.0, 2.0))
        self.assertEqual(data['provenance']['host_faults.track_rule']['profile_names'], ['steer_monitor_stack'])
        core = hostexport.host_loader()
        if core is not None:
            hp = core.HostProfile.from_bytes(hpe.to_bytes(data))
            self.assertEqual(hp.track_rule, 'esp32_adapter')
            self.assertEqual(core.HostProfile.from_bytes(hpe.to_bytes(hpe.build(old))).track_rule, 'v3_ramp_gate')
            bad = json.loads(hpe.to_bytes(data))
            bad['host_faults']['track_rule'] = 'other'
            with self.assertRaises(core.HostProfileError):
                core.HostProfile(bad)

    def test_startup_blocks(self):
        v2, v21, v22 = (smooth_hold.startup_block(load(i)) for i in (V2, V21, V22))
        self.assertEqual(v21, v2)
        self.assertEqual(v21['kind'], 'smooth_r5')
        self.assertEqual(v22['kind'], 'smooth_r6p')
        self.assertEqual({k: v22[k] for k in vp.SMOOTH_PROPOSAL_KEYS},
                         dict(hold_pwm_max=200, push_counts=24, rest_brake=1, hold_no_cancel=1))
        self.assertEqual({k: v for k, v in v22.items() if k not in vp.SMOOTH_PROPOSAL_KEYS and k != 'kind'},
                         {k: v for k, v in v21.items() if k != 'kind'})

    def test_schema_v6_checks(self):
        p21, p22 = load(V21), load(V22)
        refused = [
            (p21, {'smooth_fw_proposal': dict(hold_pwm_max=200, push_counts=0, rest_brake=0, hold_no_cancel=0)}),
            (p21, {'steer_fw_mode': 'v3_servo'}),                          # esp32_r5 needs a board law
            (p22, {'smooth_fw_proposal': dict(hold_pwm_max=60, push_counts=24, rest_brake=1, hold_no_cancel=1)}),
            (p22, {'smooth_fw_proposal': dict(hold_pwm_max=200, push_counts=24, rest_brake=2, hold_no_cancel=1)}),
        ]
        for base, change in refused:
            with self.subTest(change=change):
                with self.assertRaises(vp.ProfileError):
                    base.with_overrides(change, label='bad', allow_out_of_range=True)
        ok = p21.with_overrides({'steer_monitor_stack': 'v3'}, label='v3 stack')
        self.assertEqual(fw_eeprom.monitor_stack(ok), 'v3')
        self.assertNotIn('track_rule', hpe.build(ok)['host_faults'])
        v5 = load(V2)
        with self.assertRaises(vp.ProfileError):                          # a v5 file has no smooth_r6p
            v5.with_overrides({'steer_fw_mode': 'smooth_r6p'}, label='bad', allow_out_of_range=True)

    def test_native_parser(self):
        rigtest.host_or_skip()
        from steering.actuator_link import FirmwareSteeringBackend, NativeBackendError
        exe = rigtest.native_path()
        p21, p22 = load(V21), load(V22)
        for profile, patch in ((p21, {'hold_pwm_max': 200}),                   # proposal keys need kind smooth_r6p
                               (p22, {'kind': 'smooth_r6p', 'hold_pwm_max': 60}),
                               (p22, {'rest_brake': 2})):
            with self.assertRaises(NativeBackendError, msg=patch):
                FirmwareSteeringBackend(profile, 0, executable=exe, startup_overrides={'steer_law': patch}).close()
        with self.assertRaises(NativeBackendError):                      # kind smooth_r6p without the proposal keys
            FirmwareSteeringBackend(p21, 0, executable=exe, startup_overrides={'steer_law': {'kind': 'smooth_r6p'}}).close()
        for profile in (p21, p22):
            FirmwareSteeringBackend(profile, 0, executable=exe).close()       # the profiles' own blocks


def torque_run(profile, torque_nm=2.5, at_s=2.5, duration_s=3., seconds=6.5):
    """Standstill on the T1 rig: arm, hold steering 0, a steering torque step; (truth delta deg, t, host latches)."""
    from steering import t1_rig as tr
    from steering.scene_spec import WorldSpec
    import test_steering_abs_hold as abstest
    world = WorldSpec.from_dict({'kind': 'W0', 'markers': [list(m) for m in abstest.fw.registration_posts()]})
    rig = tr.T1Rig(tr.RigConfig(profile=profile, world=world,
                                faults=(tr.RigFault('steer_torque', at_s, dict(torque_nm=torque_nm,
                                                                               duration_s=duration_s)),)),
                   native_executable=rigtest.native_path())
    try:
        rig.run_until_true(lambda: rig.core.steering_host()['stop_state'] == 'confirmed', 3.)
        client = rig.client('monitor-test')
        assert client.arm_precheck() is None
        assert client.arm().get('ok')
        assert client.wait_armed()
        while rig.ms < seconds * 1000:
            if client.drive(0, 0).get('ok') is not True:
                break
            rig.run_until(min(seconds * 1000, rig.ms + 100) / 1000.)
        latches = sorted(rig.core.steering_host().get('latches') or ())
        arrays = rig.trace_arrays()
    finally:
        rig.close()
    return [math.degrees(x) for x in arrays['truth_delta_eq_rad']], list(arrays['t_s']), latches


class TorqueTest(rigtest.RigCase):
    def test_vehicle_stack_r5_loses_the_steering(self):
        delta, t, latches = torque_run(load(V21))
        pushed = [d for d, tt in zip(delta, t) if 2.5 <= tt <= 5.5]
        self.assertGreater(max(pushed), 15.)                    # pushed toward the end stop
        self.assertIn('steering_not_following', latches)        # the adapter rule (3 deg / 2 s) stops it

    def test_proposal_holds_the_steering(self):
        # standstill: the proposal pulls the steering back again and again (a limit cycle of about 1.2 .. 6.5 deg with
        # a period of about 0.9 s under 2.5 N*m: brake and settle let the torque push it off between the hold runs),
        # never to the end stop, and the error drops below 3 deg often enough that the host does not latch
        delta, t, latches = torque_run(load(V22))
        pushed = [d for d, tt in zip(delta, t) if 2.5 <= tt <= 5.5]
        self.assertLess(max(abs(d) for d in pushed), 8.)
        self.assertLess(min(abs(d) for d, tt in zip(delta, t) if 3.5 <= tt <= 5.5), 3.)
        self.assertLess(max(abs(d) for d, tt in zip(delta, t) if tt >= 6.), 1.5)   # after the torque
        self.assertNotIn('steering_not_following', latches)


if __name__ == '__main__':
    unittest.main()
