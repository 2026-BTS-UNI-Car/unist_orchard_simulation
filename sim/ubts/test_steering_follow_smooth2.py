"""Pre-registration steer_follow_smooth_r5_v2 (docs/steering/PREREG_STEER_FOLLOW_SMOOTH_V2_KO.md): the JSON copy equals
follow_smooth2.prereg(), the parts have the registered runs (the conditions and seeds of v1), the criteria are v1's
except the operator link limit (R60: operator_timeout_s 1.0 + one executor tick), the episodes use the vehicle stack
profile v2.1 and the SIM proposal profile v2.2, and judge_wide replaces only that one check. No run here.

Run from sim/ubts: python -B -m pytest -q -p no:cacheprovider test_steering_follow_smooth2.py
"""
import json
import os
import unittest

from steering import smooth_hold
from steering import vehicle_profile as vp
from steering.safety_scripts import follow_matrix as FM
from steering.safety_scripts import follow_smooth as S1
from steering.safety_scripts import follow_smooth2 as S
from steering.safety_scripts import follow_wide as FW

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class PreregTest(unittest.TestCase):
    def test_json_copy_is_the_module_prereg(self):
        path = os.path.join(REPO, 'docs', 'steering', 'PREREG_STEER_FOLLOW_SMOOTH_V2.json')
        with open(path, 'rb') as fh:
            raw = fh.read()
        self.assertNotIn(b'\r\n', raw)
        self.assertEqual(json.loads(raw.decode('utf-8')), json.loads(json.dumps(S.prereg())))

    def test_parts_runs_and_criteria(self):
        for part in ('matrix', 'proposal_matrix'):
            self.assertEqual(S.items((part,)), [(part,) + item[1:] for item in S1.items(('matrix',))])
        for part in ('wide', 'proposal_wide'):
            self.assertEqual(S.items((part,)), [(part,) + item[1:] for item in S1.items(('wide',))])
        self.assertEqual(len(S.items(S.PARTS)), 150)
        pre = S.prereg()
        self.assertEqual(pre['parts']['matrix']['criteria'], FM.CRITERIA)
        changed = {k for k in FW.CRITERIA if pre['parts']['wide']['criteria'][k] != FW.CRITERIA[k]}
        self.assertEqual(changed, {'link_drive_zero_s'})
        self.assertAlmostEqual(S.LINK_DRIVE_ZERO_S, 1.05)
        self.assertEqual(FW.CRITERIA['link_drive_zero_s'], .55)          # v1 / follow_wide unchanged
        self.assertEqual(pre['follows'], S1.PREREG_ID)

    def test_profiles_of_the_parts(self):
        nominal = next(c for c in FM.CONDITIONS if c['name'] == 'nominal')
        for part, pid, mode in (('matrix', S.PROFILE_ID, 'smooth_r5'), ('proposal_matrix', S.PROPOSAL_PROFILE_ID,
                                                                         'smooth_r6p')):
            ep = S.matrix_spec(nominal, 0, S.PROFILE_OF_PART[part], 'x')
            profile = ep['rig']['profile']
            self.assertEqual(profile.profile_id, pid)
            self.assertEqual(profile.value('steer_fw_mode'), mode)
            self.assertEqual(profile.value('steer_monitor_stack'), 'esp32_r5')
            self.assertEqual(ep['config'], {})                            # no threshold change
            self.assertEqual(ep['controller'], dict(kind='row_follow', wheelbase_m=.657))
            self.assertGreaterEqual(smooth_hold.executor_values(profile)['rate_deg_s'], 8.)
        dist = S.wide_spec(FW.condition('dist_step_p2p5'), 4000, S.PROFILE_ID, 'x')
        self.assertTrue(any(f['kind'] == 'steer_torque' for f in dist['faults']))
        self.assertTrue(vp.smooth_hold_mode(dist['rig']['profile']))

    def test_judge_wide_replaces_only_the_link_limit(self):
        cond = FW.condition('operator_link_lost')
        metrics = dict(min_clearance_m=1., rest_speed_m_s=0., end_reason=None, first_pause=[21., 'operator_link_lost'],
                       link_drive_zero_s=1.05, link_restart=False, after_terminal=dict(passed=True))
        checks, v1 = S.judge_wide(cond, metrics)
        self.assertTrue(all(ch['ok'] for ch in checks), checks)
        self.assertFalse(v1['ok'])                                        # 0.55 s of v1
        self.assertEqual([ch['check'] for ch in checks], [ch['check'] for ch in FW.judge(cond, metrics)])
        metrics['link_drive_zero_s'] = 1.1
        checks, _ = S.judge_wide(cond, metrics)
        self.assertEqual([ch['check'] for ch in checks if not ch['ok']], [S.LINK_CHECK])
        nominal = FW.condition('curve_l30')
        row = dict(min_clearance_m=1., rest_speed_m_s=0., end_reason='row_end', pauses_after_start=[],
                   lateral_rmse_m=.05, lateral_p95_m=.1, lateral_max_m=.2, heading_rmse_deg=1., heading_max_deg=3.,
                   speed_max_m_s=.4, row_end_error_m=.05, after_terminal=dict(passed=True))
        checks, v1 = S.judge_wide(nominal, row)
        self.assertIsNone(v1)
        self.assertEqual(checks, FW.judge(nominal, row))


if __name__ == '__main__':
    unittest.main()
