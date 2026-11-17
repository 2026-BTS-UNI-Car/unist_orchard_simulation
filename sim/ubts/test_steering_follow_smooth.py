"""Pre-registration steer_follow_smooth_r5_v1 (docs/steering/PREREG_STEER_FOLLOW_SMOOTH_KO.md): the JSON copy equals
follow_smooth.prereg(), the parts have the registered runs, and the episodes use the field profile in steer_fw_mode
smooth_r5 with the executor configuration as the rig makes it (no GO threshold change). No run here.

Run from sim/ubts: python -B -m pytest -q -p no:cacheprovider test_steering_follow_smooth.py
"""
import json
import os
import unittest

from steering import smooth_hold
from steering import vehicle_profile as vp
from steering.safety_scripts import follow_matrix as FM
from steering.safety_scripts import follow_realdims as RD
from steering.safety_scripts import follow_smooth as S
from steering.safety_scripts import follow_wide as FW

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class PreregTest(unittest.TestCase):
    def test_json_copy_is_the_module_prereg(self):
        path = os.path.join(REPO, 'docs', 'steering', 'PREREG_STEER_FOLLOW_SMOOTH.json')
        with open(path, 'rb') as fh:
            raw = fh.read()
        self.assertNotIn(b'\r\n', raw)
        self.assertEqual(json.loads(raw.decode('utf-8')), json.loads(json.dumps(S.prereg())))

    def test_parts_and_runs(self):
        self.assertEqual(len(S.items(('matrix',))), 15)
        self.assertEqual(len(S.items(('wide',))), 60)
        self.assertEqual(S.WIDE_SUBSET, RD.WIDE_SUBSET)      # the same subset and seeds as the v1 re-measure
        self.assertEqual(S.WIDE_SEEDS, RD.WIDE_SEEDS)
        self.assertEqual(S.prereg()['parts']['matrix']['criteria'], FM.CRITERIA)
        self.assertEqual(S.prereg()['parts']['wide']['criteria'], FW.CRITERIA)

    def test_episodes_use_the_smooth_profile(self):
        nominal = next(c for c in FM.CONDITIONS if c['name'] == 'nominal')
        ep = S.matrix_spec(nominal, 0)
        profile = ep['rig']['profile']
        self.assertEqual(profile.profile_id, S.PROFILE_ID)
        self.assertTrue(vp.smooth_hold_mode(profile))
        self.assertEqual(ep['config'], {})                    # no threshold change
        self.assertEqual(ep['controller'], dict(kind='row_follow', wheelbase_m=.657))
        self.assertGreaterEqual(smooth_hold.executor_values(profile)['rate_deg_s'], 8.)
        dist = S.wide_spec(FW.condition('dist_step_p2p5'), 4000)
        self.assertEqual(dist['rig']['profile'].profile_id, S.PROFILE_ID)
        self.assertTrue(any(f['kind'] == 'steer_torque' for f in dist['faults']))
        gnss = S.wide_spec(FW.condition('gnss_nominal'), 4000)
        self.assertEqual(gnss['config'], dict(gnss_mode='geofence'))
        fast = S.wide_spec(FW.condition('speed_0p5'), 4000)
        self.assertEqual(fast['controller']['cruise_speed_m_s'], .5)

    def test_executor_configuration_validates(self):
        import sys
        sys.path.insert(0, os.path.join(REPO, 'tools', 'row_executor'))
        try:
            import sim_in_loop_steer as SIL
        except ImportError as exc:              # pragma: no cover - the executor package is part of the repository
            self.skipTest('row executor not importable: %s' % exc)
        self.assertIsNotNone(SIL.abs_hold_executor_values(S.base_profile()))


if __name__ == '__main__':
    unittest.main()
