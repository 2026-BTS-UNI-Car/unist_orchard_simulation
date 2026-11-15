"""Pre-registration gnss_rtk_weak_v1 (docs/steering/PREREG_GNSS_RTK_KO.md): the JSON copy equals rtk_matrix.prereg(),
every condition has an off twin with the same random stream, the episodes use the registered profiles and executor
settings, and the judge applies the registered rules (synthetic rows). No SIM run here.

Run from sim/ubts: python -B -m pytest -q -p no:cacheprovider test_steering_rtk_matrix.py
"""
import json
import os
import unittest

from steering import gnss_sim as G
from steering.safety_scripts import rtk_matrix as R

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def row(profile, world, seed, condition, role=None, rmse=.05, lmax=.1, jog=None, u=0., step=0., cmd='a',
        end='row_end', follow_ok=True):
    return dict(profile=profile, world=world, seed=seed, condition=condition, role=role, name=condition,
                follow_ok=follow_ok, follow_failed=[], metrics=dict(end_reason=end, lateral_rmse_m=rmse,
                                                                    lateral_max_m=lmax),
                jog=None if jog is None else dict(mean_abs_m=jog), commands_sha256=cmd,
                rtk=None if condition == 'off' else dict(max_abs_correction_m=u, max_step_over_rate_limit_m=step))


class PreregTest(unittest.TestCase):
    def test_json_copy_is_the_module_prereg(self):
        path = os.path.join(REPO, 'docs', 'steering', 'PREREG_GNSS_RTK.json')
        with open(path, 'rb') as fh:
            raw = fh.read()
        self.assertNotIn(b'\r\n', raw)
        self.assertEqual(json.loads(raw.decode('utf-8')), json.loads(json.dumps(R.prereg())))

    def test_runs_twins_and_specs(self):
        todo = R.items(tuple(R.PROFILES), R.JUDGE_SEEDS)
        self.assertEqual(len(todo), R.prereg()['runs_per_seed_set'] * len(R.JUDGE_SEEDS))
        for c in R.CONDITIONS:
            self.assertIn(c['world'], R.WORLDS)
            self.assertIn(c['role'], R.ROLES)
            G.canonical(c['gnss'])                              # a valid virtual receiver spec
        for profile_key in R.PROFILES:
            off = R.spec(profile_key, 'jog_r', 6000)
            on = R.spec(profile_key, 'jog_r', 6000, R.condition('jog_fixed')['gnss'], 'jog_fixed')
            self.assertEqual(off['stream_key'], on['stream_key'])
            self.assertNotEqual(off['name'], on['name'])
            self.assertNotIn('gnss', off['rig'])
            self.assertEqual(on['config']['gnss_mode'], 'weak_constraint')
            self.assertNotIn('gnss_mode', off['config'])
            self.assertEqual(on['world'], off['world'])
            self.assertEqual(off['rig']['profile'].profile_id, R.PROFILES[profile_key]['profile_id'])
        self.assertNotEqual(R.spec('v01', 'nominal', 6000)['name'], R.spec('v01', 'jog_r', 6000)['name'])
        self.assertEqual(R.spec('v1', 'nominal', 6000)['config'], dict(steer_rate_min_deg_s=4.0))
        self.assertEqual(R.spec('v1', 'nominal', 6000)['controller'], dict(kind='row_follow', wheelbase_m=.657))
        self.assertEqual(R.spec('v01', 'nominal', 6000)['controller'], dict(kind='row_follow'))

    def test_judge_rules(self):
        rows = [row('v01', 'jog_r', 1, 'off', jog=.10, cmd='x'), row('v01', 'nominal', 1, 'off', cmd='y'),
                row('v01', 'jog_r', 2, 'off', jog=.10, cmd='x'), row('v01', 'nominal', 2, 'off', cmd='y'),
                row('v01', 'jog_r', 1, 'jog_fixed', 'benefit', jog=.06, u=.05, step=0.),
                row('v01', 'jog_r', 2, 'jog_fixed', 'benefit', jog=.11, u=.05, step=0., lmax=.17),
                row('v01', 'jog_r', 1, 'jog_float_only', 'identical', jog=.10, cmd='x'),
                row('v01', 'nominal', 1, 'nominal_mirror', 'identical', cmd='z'),
                row('v01', 'nominal', 1, 'nominal_fixed', 'harmless', rmse=.08, u=.2, step=.01),
                row('v01', 'nominal', 2, 'nominal_fixed', 'harmless', rmse=.05)]
        groups = {g['condition']: g for g in R.judge(rows)}
        ok = {(r['condition'], r['seed']): r['ok'] for r in rows if r['condition'] != 'off'}
        self.assertEqual(ok, {('jog_fixed', 1): True, ('jog_fixed', 2): True, ('jog_float_only', 1): True,
                              ('nominal_mirror', 1): False, ('nominal_fixed', 1): False, ('nominal_fixed', 2): True})
        failed = [c['check'] for c in rows[-2]['checks'] if not c['ok']]
        self.assertEqual(len(failed), 2)                       # cap, step (RMSE is a group rule)
        self.assertEqual(rows[-2]['delta_vs_off'], dict(lateral_rmse_m=.03, lateral_max_m=0.))
        nominal = groups['nominal_fixed']
        self.assertEqual([c['ok'] for c in nominal['checks']], [True, True])        # mean RMSE 0.065 <= 0.05 + 0.02
        jog = groups['jog_fixed']
        self.assertEqual((jog['reduction'], jog['seeds_not_worse']), (.15, 1))
        self.assertEqual([c['ok'] for c in jog['checks']], [True, True, True, False])   # 1 of 2 is not a majority
        self.assertFalse(jog['ok'])


if __name__ == '__main__':
    unittest.main()
