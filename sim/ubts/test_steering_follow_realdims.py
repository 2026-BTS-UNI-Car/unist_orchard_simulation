"""Pre-registration steer_follow_realdims_v1 (docs/steering/PREREG_STEER_FOLLOW_REALDIMS_KO.md): the JSON copy equals
follow_realdims.prereg(), the parts have the registered runs, and the episodes use the 2026-11-16 profile in
steer_fw_mode abs_hold_8 with the documented GO threshold (geometry_only: v3_servo). No run here.

Run from sim/ubts: python -B -m pytest -q -p no:cacheprovider test_steering_follow_realdims.py
"""
import json
import os
import unittest

from steering import vehicle_profile as vp
from steering.safety_scripts import follow_matrix as FM
from steering.safety_scripts import follow_realdims as R
from steering.safety_scripts import follow_wide as FW

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class PreregTest(unittest.TestCase):
    def test_json_copy_is_the_module_prereg(self):
        path = os.path.join(REPO, 'docs', 'steering', 'PREREG_STEER_FOLLOW_REALDIMS.json')
        with open(path, 'rb') as fh:
            raw = fh.read()
        self.assertNotIn(b'\r\n', raw)
        self.assertEqual(json.loads(raw.decode('utf-8')), json.loads(json.dumps(R.prereg())))

    def test_parts_and_runs(self):
        self.assertEqual(len(R.items(('gate_as_is',))), 1)
        self.assertEqual(len(R.items(('matrix',))), 15)
        self.assertEqual(len(R.items(('wide',))), 60)
        self.assertEqual(len(R.items(('geometry_only',))), 15)
        for name in R.WIDE_SUBSET:
            FW.condition(name)                      # every subset name is a registered wide condition
        self.assertEqual(R.prereg()['parts']['matrix']['criteria'], FM.CRITERIA)
        self.assertEqual(R.prereg()['parts']['wide']['criteria'], FW.CRITERIA)

    def test_episodes_use_the_new_profile(self):
        nominal = next(c for c in FM.CONDITIONS if c['name'] == 'nominal')
        ep = R.matrix_spec(nominal, 0, 'matrix')
        profile = ep['rig']['profile']
        self.assertEqual(profile.profile_id, R.PROFILE_ID)
        self.assertTrue(vp.abs_hold_mode(profile))
        self.assertEqual(ep['config'], R.EXEC_GATE)
        self.assertEqual(ep['controller'], dict(kind='row_follow', wheelbase_m=.657))
        self.assertEqual(R.matrix_spec(nominal, 0, 'gate_as_is')['config'], {})
        geo = R.matrix_spec(nominal, 0, 'geometry_only')['rig']['profile']
        self.assertEqual(geo.value('steer_fw_mode'), 'v3_servo')
        dist = R.wide_spec(FW.condition('dist_step_p2p5'), 4000, 'wide')
        self.assertEqual(dist['rig']['profile'].profile_id, R.PROFILE_ID)
        self.assertTrue(any(f['kind'] == 'steer_torque' for f in dist['faults']))
        gnss = R.wide_spec(FW.condition('gnss_nominal'), 4000, 'wide')
        self.assertEqual(gnss['config'], dict(R.EXEC_GATE, gnss_mode='geofence'))
        fast = R.wide_spec(FW.condition('speed_0p5'), 4000, 'wide')
        self.assertEqual(fast['controller']['cruise_speed_m_s'], .5)


if __name__ == '__main__':
    unittest.main()
