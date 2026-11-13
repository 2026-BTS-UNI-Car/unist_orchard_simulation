"""steering.gsim_final (registration steer_follow_gsim_final_v1) and the exec_checks PW-08 helpers; no ROS."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from steering import exec_checks as EC  # noqa: E402
from steering import gsim_final as GX  # noqa: E402
from steering import gsim_follow as GF  # noqa: E402

DOCS = Path(__file__).resolve().parents[2] / 'docs' / 'steering'


def report(valid=True, rtf_min=.97, ok=True):
    return dict(run=dict(name='nominal_1'), validity=dict(valid=valid, reason='x'), rtf=dict(min=rtf_min),
                checks=[dict(check='ends at the row end', ok=True), dict(check='lateral RMSE', ok=ok)],
                metrics=dict(end_reason='row_end', row_end_error_m=.45))


class FinalRegistrationTest(unittest.TestCase):
    def test_json_file_is_prereg(self):
        path = DOCS / 'PREREG_STEER_FOLLOW_GSIM_FINAL.json'
        if not path.exists():
            self.skipTest('the registration file is added by its own commit')
        self.assertEqual(json.loads(path.read_bytes().decode('utf-8')), json.loads(json.dumps(GX.prereg())))

    def test_part_a_keeps_the_v1_runs_and_limits(self):
        reg = GX.prereg()
        self.assertEqual(reg['parts']['A']['criteria'], GF.CRITERIA)
        self.assertEqual([r['name'] for r in reg['parts']['A']['runs']],
                         ['nominal_1', 'nominal_2', 'nominal_3', 'lateral_p040'])
        self.assertEqual((reg['rtf_min'], reg['settle_after_healthy_s'], reg['reruns_per_slot']), (.9, 120., 1))
        self.assertEqual(reg['image'], 'ubts-steer-sim:wf5')

    def test_judge_follow(self):
        self.assertEqual(GX.judge_follow(report())['verdict'], 'pass')
        self.assertEqual(GX.judge_follow(report(ok=False))['verdict'], 'fail')
        low = GX.judge_follow(report(rtf_min=.85))
        self.assertEqual(low['verdict'], 'invalid')
        self.assertIn('rtf.min', low['invalid_reasons'][0])
        self.assertEqual(GX.judge_follow(report(valid=False))['verdict'], 'invalid')
        self.assertEqual(GX.judge_follow(report(rtf_min=None))['verdict'], 'invalid')

    def test_pw08_table_speed_and_registration(self):
        values = dict(advanced=dict(speed_table=[[25., .239], [80., .709], [90., .715], [100., .899]],
                                    surface_scale=1., battery_scale=1.), basic=dict(speed_scale=1.))
        base = EC.table_speed(values, 50)
        self.assertAlmostEqual(base, .239 + (.709 - .239) * 25. / 55., places=9)
        values['basic']['speed_scale'] = 1.1
        self.assertAlmostEqual(EC.table_speed(values, 50) / base, 1.1, places=9)
        self.assertAlmostEqual(EC.table_speed(values, 10), .239 * 10. / 25. * 1.1, places=9)   # through (0, 0)
        self.assertAlmostEqual(EC.table_speed(values, 100), .899 * 1.1, places=9)
        self.assertIn('pw08', EC.CHECKS)

    def test_judge_pw08_amendment_2(self):
        crit = [dict(name='params_accepted', passed=True), dict(name='plant_steady_speed_ratio', passed=None),
                dict(name='default_restored', value=[200, 'default*', False], passed=False)]
        report = dict(criteria=crit, executor=dict(tuning_before='d0'), drives=[dict(validity=dict(valid=True))] * 2,
                      executor_status_end=dict(tuning=dict(preset='default', digest='d0')))
        out = GX.judge_pw08(report)
        self.assertEqual((out['recorded'], out['amendment_2']), ('fail', 'pass'))
        report['executor_status_end']['tuning']['digest'] = 'd1'
        self.assertEqual(GX.judge_pw08(report)['amendment_2'], 'fail')
        report['drives'] = [dict(validity=dict(valid=False))]
        self.assertEqual(GX.judge_pw08(report)['recorded'], 'invalid')
        self.assertEqual(GX.judge_pw08(dict(criteria=[], error='SessionError: go refused'))['recorded'], 'not_judged')
        self.assertEqual([a['n'] for a in GX.prereg()['amendments']], [1, 2])


if __name__ == '__main__':
    unittest.main()
