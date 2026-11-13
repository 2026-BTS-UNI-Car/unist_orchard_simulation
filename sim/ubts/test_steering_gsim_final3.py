"""steering.gsim_final3 (registration steer_follow_gsim_final3_v1, stage 3 of the final G-SIM check); no ROS."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from steering import gsim_final3 as G3  # noqa: E402
from steering import gsim_follow as GF  # noqa: E402

DOCS = Path(__file__).resolve().parents[2] / 'docs' / 'steering'


def report(rtf_min=.97, ok=True, start_error=None):
    return dict(run=dict(name='nominal_1'), clock='sim', rtf=dict(min=rtf_min), start_error=start_error,
                checks=[dict(check='ends at the row end', ok=True), dict(check='lateral RMSE', ok=ok)],
                metrics=dict(end_reason='row_end', row_end_error_m=.45))


class Final3RegistrationTest(unittest.TestCase):
    def test_json_file_is_prereg(self):
        path = DOCS / 'PREREG_STEER_FOLLOW_GSIM_FINAL3.json'
        if not path.exists():
            self.skipTest('the registration file is added by its own commit')
        self.assertEqual(json.loads(path.read_bytes().decode('utf-8')), json.loads(json.dumps(G3.prereg())))

    def test_registration_values(self):
        reg = G3.prereg()
        self.assertEqual(reg['parts']['A']['criteria'], GF.CRITERIA)
        self.assertEqual((reg['common']['image'], reg['common']['clock']), ('ubts-steer-sim:wf6', 'sim'))
        self.assertEqual((reg['operator_timeout_s'], reg['link_lost_limit_s'], reg['heartbeat_gap_max_s']), (1., 1.05, .5))
        self.assertFalse(reg['parts']['W']['judged'])
        self.assertIn('--reset-pose', reg['parts']['C']['check'])
        self.assertIn('sim_auto.py start', reg['parts']['F']['check'])

    def test_judge_follow(self):
        self.assertEqual(G3.judge_follow(report())['verdict'], 'pass')
        self.assertEqual(G3.judge_follow(report(ok=False))['verdict'], 'fail')
        self.assertEqual(G3.judge_follow(report(rtf_min=.85))['verdict'], 'pass')     # SIM clock: no 0.9 rule
        self.assertEqual(G3.judge_follow(report(rtf_min=.4))['verdict'], 'invalid')
        self.assertEqual(G3.judge_follow(report(start_error='SessionError: go refused'))['verdict'], 'invalid')

    def test_judge_browser_and_sim_auto(self):
        checks = [dict(name=n, ok=True) for names in G3.BROWSER_ITEMS.values() for n in names]
        self.assertTrue(G3.judge_browser(dict(checks=checks))['passed'])
        checks[-1]['ok'] = False
        out = G3.judge_browser(dict(checks=checks))
        self.assertFalse(out['passed'])
        self.assertFalse(out['items']['live_map']['passed'])
        self.assertTrue(out['items']['prepare']['passed'])
        self.assertFalse(G3.judge_browser(dict(checks=checks[:3]))['items']['heartbeat']['passed'])   # missing = fail
        ends = [dict(end_reason='row_end')] * 3
        self.assertTrue(G3.judge_sim_auto(ends)['passed'])
        self.assertFalse(G3.judge_sim_auto(ends[:2])['passed'])
        self.assertFalse(G3.judge_sim_auto(ends[:2] + [dict(end_reason='steering_feedback_stale')])['passed'])


if __name__ == '__main__':
    unittest.main()
