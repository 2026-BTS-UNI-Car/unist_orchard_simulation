"""steering.gsim_final5 (registration steer_follow_gsim_final5_v1, stage 5 of the final G-SIM check); no ROS."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from steering import gsim_final3 as G3  # noqa: E402
from steering import gsim_final5 as G5  # noqa: E402
from steering import gsim_follow as GF  # noqa: E402

DOCS = Path(__file__).resolve().parents[2] / 'docs' / 'steering'


class Final5RegistrationTest(unittest.TestCase):
    def test_json_file_is_prereg(self):
        path = DOCS / 'PREREG_STEER_FOLLOW_GSIM_FINAL5.json'
        if not path.exists():
            self.skipTest('the registration file is added by its own commit')
        self.assertEqual(json.loads(path.read_bytes().decode('utf-8')), json.loads(json.dumps(G5.prereg())))

    def test_registration_values(self):
        reg = G5.prereg()
        self.assertEqual(reg['common']['image'], 'ubts-steer-sim:wf8')
        self.assertEqual(reg['parts']['A']['criteria'], GF.CRITERIA)
        self.assertEqual([r['name'] for r in reg['parts']['A']['runs']],
                         ['nominal_1', 'nominal_2', 'nominal_3', 'lateral_p040'])
        self.assertIn('10 s without a wall gap above 0.1 s', reg['parts']['A']['check'])
        self.assertIn('image wf8', reg['parts']['F']['check'])
        self.assertNotIn('wf7', reg['parts']['F']['check'])
        self.assertEqual([i['name'] for i in reg['parts']['B']['items']], list(G5.B_ITEMS))
        self.assertEqual(len(G5.B_ITEMS), 6)
        self.assertNotIn('C', reg['parts'])

    def test_browser_judge_needs_all_six(self):
        checks = [dict(name=n, ok=True) for k in G5.B_ITEMS for n in G3.BROWSER_ITEMS[k]]
        self.assertTrue(G5.judge_browser(dict(checks=checks))['passed'])
        checks[-1]['ok'] = False
        self.assertFalse(G5.judge_browser(dict(checks=checks))['passed'])

    def test_sim_stall_cause(self):
        emu = [(100.36, .34), (200., .11)]
        self.assertEqual(G5.sim_stall_cause(100.8, [(100.35, .35)], emu), 'sim_delivery')
        self.assertEqual(G5.sim_stall_cause(101.5, [(100.35, .35)], emu), 'executor_or_controller')   # too long before
        self.assertEqual(G5.sim_stall_cause(100.8, [(100.35, .2)], emu), 'executor_or_controller')    # gap too short
        self.assertEqual(G5.sim_stall_cause(100.8, [(100.55, .35)], emu), 'executor_or_controller')   # emulator saw none
        self.assertEqual(G5.sim_stall_cause(None, [(100.35, .35)], emu), 'executor_or_controller')

    def test_foreign_sessions(self):
        events = [dict(action='bind', session='aaaa0001'), dict(action='conflict', session='bbbb0002'),
                  dict(action='takeover', session='cccc0003'), dict(action='bind', session='aaaa0001'),
                  dict(action='release', session='dddd0004')]
        self.assertEqual(G5.foreign_sessions(events, {'cccc0003'}), ['aaaa0001'])


if __name__ == '__main__':
    unittest.main()
