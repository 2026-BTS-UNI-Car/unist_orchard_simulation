"""steering.gsim_final4 (registration steer_follow_gsim_final4_v1, stage 4 of the final G-SIM check); no ROS."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from steering import gsim_final3 as G3  # noqa: E402
from steering import gsim_final4 as G4  # noqa: E402
from steering import gsim_follow as GF  # noqa: E402

DOCS = Path(__file__).resolve().parents[2] / 'docs' / 'steering'


class Final4RegistrationTest(unittest.TestCase):
    def test_json_file_is_prereg(self):
        path = DOCS / 'PREREG_STEER_FOLLOW_GSIM_FINAL4.json'
        if not path.exists():
            self.skipTest('the registration file is added by its own commit')
        self.assertEqual(json.loads(path.read_bytes().decode('utf-8')), json.loads(json.dumps(G4.prereg())))

    def test_registration_values(self):
        reg = G4.prereg()
        self.assertEqual(reg['common']['image'], 'ubts-steer-sim:wf7')
        self.assertEqual(reg['parts']['A']['criteria'], GF.CRITERIA)
        self.assertIn('image wf7', reg['parts']['F']['check'])
        self.assertNotIn('wf6', reg['parts']['F']['check'])
        self.assertEqual([i['name'] for i in reg['parts']['B']['items']], list(G4.B_ITEMS))
        self.assertIn('--only bridge_stall', reg['parts']['C']['check'])

    def test_judges(self):
        checks = [dict(name=n, ok=True) for k in G4.B_ITEMS for n in G3.BROWSER_ITEMS[k]]
        self.assertTrue(G4.judge_browser(dict(checks=checks))['passed'])        # live_map is not an item of stage 4
        checks[0]['ok'] = False
        self.assertFalse(G4.judge_browser(dict(checks=checks))['passed'])
        ticks = [dict(k='tick', run=2, state='PAUSED', yaw=9.)] + [dict(k='tick', run=2, state='ACTIVE', yaw=v)
                                                                    for v in (5., 5.2, 4.9, 5.1, 6.)]
        self.assertEqual(G4.start_yaw_offset(ticks, 2), 5.1)
        self.assertIsNone(G4.start_yaw_offset(ticks, 3))


if __name__ == '__main__':
    unittest.main()
