"""steering.gsim_final8b (registration steer_follow_gsim_final8b_v1, stage 8b of the final G-SIM check); no ROS."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from steering import gsim_final7 as G7  # noqa: E402
from steering import gsim_final8b as G8  # noqa: E402
from steering import gsim_follow as GF  # noqa: E402

DOCS = Path(__file__).resolve().parents[2] / 'docs' / 'steering'


def plain(value):
    return json.loads(json.dumps(value))


class Final8bRegistrationTest(unittest.TestCase):
    def test_json_file_is_prereg(self):
        path = DOCS / 'PREREG_STEER_FOLLOW_GSIM_FINAL8B.json'
        if not path.exists():
            self.skipTest('the registration file is added by its own commit')
        self.assertEqual(json.loads(path.read_bytes().decode('utf-8')), plain(G8.prereg()))

    def test_only_placement_and_image_change(self):
        reg, base = G8.prereg(), G7.prereg()
        self.assertEqual(reg['prereg_id'], 'steer_follow_gsim_final8b_v1')
        self.assertEqual(reg['based_on'], G7.PREREG_ID)
        self.assertEqual(reg['common']['image'], 'ubts-steer-sim:wf11')
        self.assertEqual(reg['parts']['A']['criteria'], GF.CRITERIA)
        self.assertEqual(plain(reg['parts']['A']), plain(base['parts']['A']))
        self.assertEqual(plain(reg['parts']['B']), plain(base['parts']['B']))
        self.assertIn('image wf11', reg['parts']['F']['check'])
        self.assertNotIn('wf10', reg['parts']['F']['check'])
        for key in ('host_quiet', 'host_fault', 'rtf_host_failure', 'operator_timeout_s', 'heartbeat_gap_max_s',
                    'link_check_s', 'e_class_key', 'b_script_reasons', 'sim_stall_gap_s', 'sim_stall_before_s',
                    'measure_dashboard_port', 'repeat'):
            self.assertEqual(plain(reg[key]), plain(base[key]), key)
        for key in set(base['common']) - {'image'}:
            self.assertEqual(plain(reg['common'][key]), plain(base['common'][key]), key)
        self.assertEqual(reg['amendments'], [])

    def test_placement(self):
        self.assertEqual(G8.P_MASK, 0xFFC03)
        self.assertEqual(sorted(G8.P_LOGICAL + G8.E_LOGICAL), list(range(22)))
        self.assertFalse(set(G8.P_LOGICAL) & set(G8.E_LOGICAL))
        hp = G8.prereg()['host_placement']
        self.assertEqual(hp['method_order'], ['affinity', 'schedpolicy'])
        self.assertEqual(hp['affinity']['mask'], '0xFFC03')
        self.assertEqual(hp['schedpolicy']['value'], 2)

    def test_rules_are_stage7(self):
        for name in ('judge_follow', 'judge_browser', 'host_fault', 'is_quiet', 'quiet_means', 'failure_event',
                     'b_failure_event', 'slot_verdict', 'judge_part_a', 'judge_part_f', 'sampler_rows'):
            self.assertIs(getattr(G8, name), getattr(G7, name), name)


if __name__ == '__main__':
    unittest.main()
