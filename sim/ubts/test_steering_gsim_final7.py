"""steering.gsim_final7 (registration steer_follow_gsim_final7_v1, stage 7 of the final G-SIM check); no ROS."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from steering import gsim_final3 as G3  # noqa: E402
from steering import gsim_final6 as G6  # noqa: E402
from steering import gsim_final7 as G7  # noqa: E402
from steering import gsim_follow as GF  # noqa: E402

DOCS = Path(__file__).resolve().parents[2] / 'docs' / 'steering'


def samples(values, e=None, start=1000., gaps=None):
    """Sampler rows: one per second from start, total CPU values, E-class loads (same length or None), gaps."""
    gaps = gaps or {}
    rows = []
    for k, v in enumerate(values):
        row = dict(kind='s', wall=start + k, gap_s=gaps.get(k, 1.), total_pct=v)
        if e is not None:
            row['cls'] = {'0': e[k], '1': 20.}
        rows.append(row)
    return rows


def tick(t, run, state, reason=None, ww=None):
    return dict(k='tick', t=t, run=run, state=state, reason=reason,
                diag=dict(ww=ww if ww is not None else 5000. + t, wm=100. + t))


class Final7RegistrationTest(unittest.TestCase):
    def test_json_file_is_prereg(self):
        path = DOCS / 'PREREG_STEER_FOLLOW_GSIM_FINAL7.json'
        if not path.exists():
            self.skipTest('the registration file is added by its own commit')
        self.assertEqual(json.loads(path.read_bytes().decode('utf-8')), json.loads(json.dumps(G7.prereg())))

    def test_registration_values(self):
        reg = G7.prereg()
        self.assertEqual(reg['prereg_id'], 'steer_follow_gsim_final7_v1')
        self.assertEqual(reg['based_on'], G6.PREREG_ID)
        self.assertEqual(reg['common']['image'], 'ubts-steer-sim:wf10')
        self.assertEqual(reg['parts']['A']['criteria'], GF.CRITERIA)        # the criteria of stages 5 and 6
        self.assertEqual(json.loads(json.dumps(reg['parts']['A'])), json.loads(json.dumps(G6.prereg()['parts']['A'])))
        self.assertEqual(json.loads(json.dumps(reg['parts']['B'])), json.loads(json.dumps(G6.prereg()['parts']['B'])))
        self.assertIn('image wf10', reg['parts']['F']['check'])
        self.assertNotIn('wf9', reg['parts']['F']['check'])
        self.assertEqual(len(G7.B_ITEMS), 6)
        self.assertEqual(reg['host_quiet'], dict(cpu_pct=45., e_class_pct=50., window_s=30., poll_s=5.,
                                                 max_wait_s=900.))
        self.assertEqual(reg['host_fault'], dict(cpu_pct=70., late_s=1., e_class_pct=75., window_s=5.,
                                                 sampler_period_s=1., void_reruns=2))
        self.assertEqual(reg['rtf_host_failure'], .5)
        self.assertEqual(reg['operator_timeout_s'], 1.)
        self.assertEqual(reg['measure_dashboard_port'], 18091)
        self.assertEqual(reg['amendments'], [])


class HostRulesTest(unittest.TestCase):
    def test_quiet_needs_both_means(self):
        self.assertTrue(G7.is_quiet(samples([30.] * 40, e=[40.] * 40), 1039.))
        self.assertFalse(G7.is_quiet(samples([30.] * 40, e=[55.] * 40), 1039.))   # E class busy, total low
        self.assertFalse(G7.is_quiet(samples([50.] * 40, e=[40.] * 40), 1039.))   # total high
        self.assertFalse(G7.is_quiet(samples([30.] * 40), 1039.))                 # no E-class data: not quiet
        self.assertEqual(G7.quiet_means(samples([30.] * 40, e=[40.] * 40), 1005.), (None, None))

    def test_host_fault_e_class(self):
        rows = samples([40.] * 20, e=[40.] * 10 + [80.] * 10)
        hf = G7.host_fault(rows, 1015.)                                 # samples 1010-1015: E 80
        self.assertTrue(hf['void'])
        self.assertEqual(hf['e_mean'], 80.)
        self.assertTrue(any('E class' in r for r in hf['reasons']))
        hf = G7.host_fault(rows, 1012.)                                 # 1007-1012: 3 x 40, 3 x 80 -> 60
        self.assertFalse(hf['void'])
        self.assertEqual(hf['e_mean'], 60.)
        self.assertTrue(G7.host_fault(samples([40.] * 20, e=[75.] * 20), 1015.)['void'])    # at the threshold
        self.assertFalse(G7.host_fault(samples([40.] * 20, e=[74.9] * 20), 1015.)['void'])

    def test_stage6_rules_stay(self):
        rows = samples([40.] * 10 + [70.] + [40.] * 10, e=[10.] * 21)
        self.assertTrue(G7.host_fault(rows, 1012.)['void'])
        rows = samples([40.] * 20, e=[10.] * 20, gaps={12: 2.})
        self.assertTrue(G7.host_fault(rows, 1014.)['void'])
        self.assertFalse(G7.host_fault(samples([40.] * 20, e=[10.] * 20), 1015.)['void'])
        self.assertFalse(G7.host_fault(rows, None)['void'])


class BFailureEventTest(unittest.TestCase):
    def test_executor_stop_first(self):
        ticks = [tick(1., 1, 'PAUSED', 'await_hold'), tick(2., 1, 'ACTIVE', 'align'), tick(3., 1, 'ACTIVE'),
                 tick(4., 1, 'ENDED', 'steering_feedback_stale', ww=7000.),
                 tick(5., 1, 'ENDED', 'steering_feedback_stale')]
        ev = G7.b_failure_event(ticks)
        self.assertEqual((ev['wall'], ev['reason'], ev['run'], ev['source']),
                         (7000., 'steering_feedback_stale', 1, 'executor'))

    def test_script_stops_are_skipped(self):
        ticks = [tick(1., 1, 'ACTIVE'), tick(2., 1, 'PAUSED', 'operator_link_lost'), tick(3., 1, 'ENDED', 'operator_stop'),
                 tick(4., 2, 'ACTIVE'), tick(5., 2, 'ENDED', 'operator_stop'),
                 tick(6., 3, 'ACTIVE'), tick(7., 3, 'PAUSED', 'hold_released'), tick(8., 3, 'ENDED', 'operator_stop')]
        self.assertIsNone(G7.b_failure_event(ticks))
        ticks += [tick(9., 4, 'ACTIVE'), tick(10., 4, 'PAUSED', 'stall_guard', ww=9999.)]
        self.assertEqual(G7.b_failure_event(ticks)['wall'], 9999.)

    def test_browser_check_fallback(self):
        self.assertIsNone(G7.b_failure_event([], dict(checks=[dict(name='x', ok=True, wall=1.)])))
        name = next(iter(G3.BROWSER_ITEMS[G7.B_ITEMS[0]]))
        result = dict(checks=[dict(name=name, ok=False, wall=123.)])
        self.assertEqual(G7.b_failure_event([tick(1., 1, 'PAUSED', 'await_hold')], result),
                         dict(wall=123., source='browser_check'))


if __name__ == '__main__':
    unittest.main()
