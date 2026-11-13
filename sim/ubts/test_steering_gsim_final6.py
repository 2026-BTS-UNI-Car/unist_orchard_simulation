"""steering.gsim_final6 (registration steer_follow_gsim_final6_v1, stage 6 of the final G-SIM check); no ROS."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from steering import gsim_final3 as G3  # noqa: E402
from steering import gsim_final6 as G6  # noqa: E402
from steering import gsim_follow as GF  # noqa: E402

DOCS = Path(__file__).resolve().parents[2] / 'docs' / 'steering'


def samples(values, start=1000., gaps=None):
    """Sampler rows: one per second from start, total CPU values, optional {index: gap_s}."""
    gaps = gaps or {}
    return [dict(kind='s', wall=start + k, gap_s=gaps.get(k, 1.), total_pct=v) for k, v in enumerate(values)]


class Final6RegistrationTest(unittest.TestCase):
    def test_json_file_is_prereg(self):
        path = DOCS / 'PREREG_STEER_FOLLOW_GSIM_FINAL6.json'
        if not path.exists():
            self.skipTest('the registration file is added by its own commit')
        self.assertEqual(json.loads(path.read_bytes().decode('utf-8')), json.loads(json.dumps(G6.prereg())))

    def test_registration_values(self):
        reg = G6.prereg()
        self.assertEqual(reg['common']['image'], 'ubts-steer-sim:wf9')
        self.assertEqual(reg['parts']['A']['criteria'], GF.CRITERIA)
        self.assertEqual([r['name'] for r in reg['parts']['A']['runs']],
                         ['nominal_1', 'nominal_2', 'nominal_3', 'lateral_p040'])
        self.assertIn('image wf9', reg['parts']['F']['check'])
        self.assertNotIn('wf8', reg['parts']['F']['check'])
        self.assertEqual([i['name'] for i in reg['parts']['B']['items']], list(G6.B_ITEMS))
        self.assertEqual(len(G6.B_ITEMS), 6)
        self.assertEqual(reg['host_quiet'], dict(cpu_pct=45., window_s=30., poll_s=5., max_wait_s=900.))
        self.assertEqual(reg['host_fault'], dict(cpu_pct=70., late_s=1., window_s=5., sampler_period_s=1.,
                                                 void_reruns=2))
        self.assertEqual(reg['rtf_host_failure'], .5)
        self.assertEqual(reg['operator_timeout_s'], 1.)
        self.assertEqual(reg['measure_dashboard_port'], 18091)
        self.assertEqual(reg['amendments'], [])


class HostRulesTest(unittest.TestCase):
    def test_quiet_mean(self):
        rows = samples([50.] * 10 + [40.] * 30)
        self.assertAlmostEqual(G6.quiet_mean(rows, 1039.), 40.)
        self.assertAlmostEqual(G6.quiet_mean(rows, 1019.), (10 * 50. + 10 * 40.) / 20)   # samples 1000-1019 only
        self.assertIsNone(G6.quiet_mean(rows, 1005.))                                    # 6 samples < 15

    def test_host_fault_cpu(self):
        rows = samples([40.] * 10 + [70.] + [40.] * 10)
        self.assertTrue(G6.host_fault(rows, 1012.)['void'])          # the 70 % sample at 1010 is 2 s before
        self.assertTrue(G6.host_fault(rows, 1015.)['void'])          # 5 s before: still in the window
        self.assertFalse(G6.host_fault(rows, 1016.)['void'])         # 6 s before: outside
        self.assertFalse(G6.host_fault(rows, 1009.)['void'])         # after the event: not counted
        rows = samples([40.] * 10 + [69.9] + [40.] * 10)
        self.assertFalse(G6.host_fault(rows, 1012.)['void'])

    def test_host_fault_lateness(self):
        rows = samples([30.] * 20, gaps={12: 2.0})
        result = G6.host_fault(rows, 1014.)
        self.assertTrue(result['void'])
        self.assertEqual(result['late_max'], 1.)
        rows = samples([30.] * 20, gaps={12: 1.9})
        self.assertFalse(G6.host_fault(rows, 1014.)['void'])

    def test_host_fault_without_evidence(self):
        self.assertFalse(G6.host_fault([], 1000.)['void'])
        self.assertFalse(G6.host_fault(samples([90.]), None)['void'])

    def test_failure_event(self):
        ticks = [dict(k='tick', run=1, state='PAUSED', t=1.0, diag=dict(ww=10., wm=1.)),
                 dict(k='tick', run=1, state='ACTIVE', t=1.05, diag=dict(ww=10.05, wm=1.05)),
                 dict(k='tick', run=1, state='ACTIVE', t=1.1, diag=dict(ww=10.1, wm=1.1)),
                 dict(k='tick', run=1, state='PAUSED', t=1.15, reason='stale_lidar', diag=dict(ww=10.2, wm=1.2)),
                 dict(k='tick', run=1, state='ENDED', t=2., diag=dict(ww=11., wm=2.))]
        event = G6.failure_event(ticks, 1, 'operator_end')
        self.assertEqual((event['t'], event['wall'], event['state'], event['reason']), (1.15, 10.2, 'PAUSED', 'stale_lidar'))
        self.assertIsNone(G6.failure_event(ticks, 1, 'row_end'))
        self.assertIsNone(G6.failure_event(ticks, 2, 'operator_end'))
        self.assertIsNone(G6.failure_event(ticks[:1], 1, 'operator_end'))           # never drove

    def test_browser_failure_event(self):
        names = [n for item in G6.B_ITEMS for n in G3.BROWSER_ITEMS[item]]
        checks = [dict(name=n, ok=True, wall=100. + k) for k, n in enumerate(names)]
        checks.insert(0, dict(name='no page or console error', ok=False, wall=50.))  # not a registered item
        self.assertIsNone(G6.browser_failure_event(dict(checks=checks)))
        checks[5]['ok'] = False
        checks[8]['ok'] = False
        self.assertEqual(G6.browser_failure_event(dict(checks=checks)), checks[5]['wall'])

    def test_sim_stall_cause(self):
        emu = [(100.36, .34), (200., .11)]
        self.assertEqual(G6.sim_stall_cause(100.8, emu), 'sim_delivery')
        self.assertEqual(G6.sim_stall_cause(101.5, emu), 'executor_or_controller')
        self.assertEqual(G6.sim_stall_cause(200.5, emu), 'executor_or_controller')   # stall too short
        self.assertEqual(G6.sim_stall_cause(None, emu), 'executor_or_controller')


class SlotTest(unittest.TestCase):
    def test_slot_verdict(self):
        self.assertEqual(G6.slot_verdict(['pass']), 'pass')
        self.assertEqual(G6.slot_verdict(['void', 'fail']), 'fail')
        self.assertEqual(G6.slot_verdict(['void', 'invalid', 'pass']), 'pass')
        self.assertEqual(G6.slot_verdict(['void', 'void', 'void']), 'not_judged')
        with self.assertRaises(ValueError):
            G6.slot_verdict(['void', 'void', 'void', 'pass'])
        with self.assertRaises(ValueError):
            G6.slot_verdict(['maybe'])

    def test_part_a(self):
        self.assertTrue(G6.judge_part_a(dict(a=['pass'], b=['pass'], c=['void', 'pass'], d=['void'] * 3))['passed'])
        self.assertFalse(G6.judge_part_a(dict(a=['pass'], b=['pass'], c=['void'] * 3, d=['void'] * 3))['passed'])
        self.assertFalse(G6.judge_part_a(dict(a=['pass'], b=['fail'], c=['pass'], d=['pass']))['passed'])

    def test_part_f(self):
        self.assertTrue(G6.judge_part_f([['pass'], ['void', 'pass'], ['pass']])['passed'])
        self.assertEqual(G6.judge_part_f([['pass'], ['void'] * 3, ['pass']])['verdict'], 'not_judged')
        self.assertEqual(G6.judge_part_f([['pass'], ['fail'], ['pass']])['verdict'], 'fail')
        with self.assertRaises(ValueError):
            G6.judge_part_f([['pass']])


if __name__ == '__main__':
    unittest.main()
