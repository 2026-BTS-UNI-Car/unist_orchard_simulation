"""Operator page JavaScript (operator_http.PAGE) in node with a fake DOM, fake timers and a scripted fetch.

Review S9/S10: the banner and the STOP button say what they do (SHADOW: only ends the shadow run; MOTOR:
vehicle stop); a failed STOP (HTTP 403, 502, no answer within 2.5 s) is shown as a red alert naming the
dashboard STOP / physical E-stop; a 403 status poll disables the page and reloads it into the token
instructions; a lost connection greys the table. Review S3 (page side): a new press waits for the previous
release request (at most 0.7 s). Skipped where node is not installed (the Jetson-like container); runs on
the development PC.
"""
import json
import os
import shutil
import subprocess
import tempfile
import unittest

import rowexec_testlib  # noqa: F401  (path setup)
from row_executor import operator_http as H

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = shutil.which('node')


def page_script():
    page = H.PAGE.replace('__NONCE__', 'n').replace('__TOKEN__', 'test-token-0123456789abcdef')
    return page.split('<script nonce="n">', 1)[1].split('</script>', 1)[0]


@unittest.skipUnless(NODE, 'node not installed')
class PageScriptTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        tmp = tempfile.mkdtemp(prefix='rowexec-page-')
        try:
            path = os.path.join(tmp, 'page.js')
            with open(path, 'w', encoding='utf-8', newline='\n') as fh:
                fh.write(page_script())
            check = subprocess.run([NODE, '--check', path], capture_output=True, text=True, timeout=60)
            if check.returncode:
                raise AssertionError('page script syntax: ' + check.stderr)
            result = subprocess.run([NODE, os.path.join(HERE, 'page_js_harness.js'), path], capture_output=True,
                                    text=True, encoding='utf-8', timeout=120)
            if result.returncode:
                raise AssertionError('harness failed: ' + result.stderr)
            cls.out = json.loads(result.stdout)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_calibration_card(self):
        c = self.out['calib']
        self.assertFalse(c['cardHidden'])
        self.assertTrue(c['shadowCardHidden'])                 # no calib_options (SHADOW / not enabled): no card
        self.assertEqual(c['filled']['set'], 'd10_d20')
        self.assertEqual(c['filled']['pct'], '3')
        self.assertEqual(c['filled']['dist'], 15)
        self.assertIn('sym20 (불가)', c['filled']['setOptions'])
        self.assertEqual(c['bodies'][0], {'kind': 'arc', 'params': {'arc_set': 'd10_d20', 'step_s': 4, 'distance_m': 8,
                                                                    'time_s': 45}})
        self.assertEqual(c['bodies'][1], {'kind': 'rolldown', 'params': {'step_pct': 3, 'hold_s': 3, 'distance_m': 8,
                                                                         'time_s': 45}})
        self.assertIn('duty(양쪽 같게, B 아래까지): 55 → 52 → 49 → 46 → 43 → 40', c['plan'])
        self.assertIn('출발 최소 34 (측정 breakaway)', c['plan'])
        # day-2: the arc starts at base/base (N4), the stop kind (C7) and its result, R2 yaw authority on the page
        self.assertIn('base/base로 출발, 굴러가면 쌍(안쪽 최소 34, 측정 breakaway)', c['planArc'])
        self.assertIn('직진 55/55', c['planStop'])
        self.assertIn('실행기가 스스로 0/0', c['planStop'])
        self.assertEqual(c['bodies'][2], {'kind': 'stop', 'params': {'distance_m': 8, 'time_s': 45}})
        self.assertIn('0/0 뒤 정지 0.31 m (0/0 때 0.45 m/s, 1.2 s)', c['stopOut'])
        self.assertIn('정속 0.45 m/s', c['stopOut'])
        self.assertIn('유지', c['stopTab'])
        self.assertIn('지금 yaw 권한 3.00 °/s, max_yaw 1.8 °/s', c['limits'])
        self.assertIn('55/45', c['tab'])
        self.assertIn('무효', c['tab'])
        self.assertIn('yaw ≈ 2.96', c['out'])
        self.assertIn('끝: calib_complete', c['out'])
        self.assertIn('보정 run만 출력한다', c['banner'])

    def test_shadow_banner_and_stop_text(self):
        b = self.out['shadow_stop']['before']
        self.assertEqual(b['bannerClass'], 'SHADOW')
        self.assertIn('실행기는 차량을 제어하지 않는다', b['banner'])
        self.assertIn('대시보드 STOP', b['banner'])
        self.assertEqual(b['stop'], 'shadow run 종료 (차량 정지 아님)')
        self.assertEqual(b['stopClass'], 'shadow')
        self.assertIn('차량 안 움직임', b['hold'])
        a = self.out['shadow_stop']['after']
        self.assertIn('차량은 이 버튼으로 멈추지 않는다', a['alert'])
        self.assertEqual(a['alertClass'], 'warn')

    def test_motor_banner_and_confirmed_stop(self):
        b = self.out['motor_ok']['before']
        self.assertEqual((b['stop'], b['stopClass'], b['bannerClass']), ('STOP (차량 정지)', '', 'MOTOR'))
        self.assertIn('Arm 전', b['banner'])
        a = self.out['motor_ok']['after']
        self.assertEqual(a['alertClass'], 'ok')
        self.assertIn('브리지가 확인', a['alert'])
        armed = self.out['armed']['before']
        self.assertEqual(armed['bannerClass'], 'armed')
        self.assertIn('ARMED', armed['banner'])

    def test_failed_stops_are_red_and_name_the_other_stops(self):
        for case, needle in (('motor_502', 'HTTP 502'), ('motor_timeout', '응답 없음/시간 초과')):
            with self.subTest(case=case):
                a = self.out[case]['after']
                self.assertFalse(a['alertHidden'])
                self.assertEqual(a['alertClass'], 'bad')
                self.assertIn('STOP 실패', a['alert'])
                self.assertIn(needle, a['alert'])
                self.assertIn('물리 E-stop', a['alert'])

    def test_stop_403_disables_and_reloads_into_the_token_page(self):
        a = self.out['motor_403']['after']
        self.assertEqual(a['alertClass'], 'bad')
        self.assertIn('토큰 무효', a['alert'])
        self.assertIn('operator_token.txt', a['alert'])
        self.assertTrue(a['holdDisabled'])
        self.assertEqual(a['reloads'], 0)
        self.assertEqual(a['reloadsLater'], 1)

    def test_status_403_and_lost_connection(self):
        b = self.out['status_403']['before']
        self.assertIn('토큰 무효', b['alert'])
        self.assertTrue(b['holdDisabled'])
        self.assertEqual(b['reloadsLater'], 1)
        polls = [p for p in self.out['status_403']['requests'] if p == '/api/status']
        self.assertEqual(len(polls), 1)                    # polling stops after a 403
        lost = self.out['status_lost']['before']
        self.assertEqual(lost['tableClass'], 'stale')
        self.assertIn('연결 끊김', lost['alertLater'])

    def test_log_failure_is_shown(self):
        b = self.out['log_failing']['before']
        self.assertFalse(b['alertHidden'])
        self.assertIn('기록 실패', b['alert'])
        self.assertIn('12', b['alert'])

    def test_conflicting_press_is_released_and_explained(self):
        c = self.out['conflict']
        self.assertFalse(c['holdOn'])                     # the button turns off at the first refusal
        self.assertEqual((c['beats'], c['releases']), (1, 1))
        self.assertEqual(c['alertClass'], 'warn')
        self.assertIn('겹쳐', c['alert'])

    def test_single_row_panel_numbers_and_category_text(self):
        s = self.out['single_row']
        shown = s['shown']
        self.assertFalse(shown['cardHidden'])
        self.assertEqual(shown['phase'], '열 끝 감속')
        for needle in ('0.123', '-2.5', '낮음', '52 / 58', '포화', 'SLOW', '-1.5', '2.50 / 1.89', '41.2', '0.21'):
            self.assertIn(needle, shown['tab'])
        self.assertIn('SENSOR_ONLY_SINGLE_ROW', shown['loc'])
        self.assertIn('GPS', shown['loc'])                       # the page says what the localisation is not
        self.assertEqual(shown['stop'], '')
        stopped = s['stopped']
        self.assertEqual(stopped['phase'], '정지 (종료: 자동 회전·재출발 없음)')
        self.assertEqual(stopped['stop'], '정지 사유: row_end → 정상 열 끝')
        self.assertEqual(stopped['stopClass'], 'ok')
        self.assertIn('첫 정지', stopped['tab'])
        self.assertIn('추정 없음', ' '.join(stopped['texts']))     # no geometry: a note, no exception
        self.assertEqual(s['backstop']['stop'], '정지 사유: row_distance_backstop → 운용자·백스톱 정지 (열 끝 아님)')
        self.assertEqual(s['backstop']['stopClass'], 'bad')

    def test_single_row_start_without_length_and_panel_stop(self):
        s = self.out['single_row']
        self.assertEqual(s['starts'], [{'row_len_m': None}, {'row_len_m': 64.5}, {'row_len_m': None}])
        self.assertNotIn('열 길이를 입력해야', s['startMsg'])
        self.assertEqual(s['stops'], 1)
        # the other modes: card hidden, the main Start still needs a length (no request sent)
        self.assertTrue(s['plainHidden'])
        self.assertEqual(s['plainStarts'], 0)
        self.assertIn('열 길이를 입력해야', s['plainMsg'])

    def test_single_row_canvas_draws_the_corridor_estimate_with_the_axis_up(self):
        s = self.out['single_row']
        self.assertGreater(s['ops'], 20)
        self.assertTrue(s['finite'])
        for colour in ('#43a047', '#64b5f6', '#e53935', '#ff9800', '#ffffff', '#ffeb3b'):
            self.assertIn(colour, s['strokes'])                  # centreline, row lines, support end, path, body, front
        self.assertEqual(len(s['front']), 2)
        for x, y in s['front']:                                  # vehicle centre at (150, 315): the front is above it
            self.assertLess(y, 315 - 10)
            self.assertLess(abs(x - 150), 10)

    def test_new_press_waits_for_the_previous_release(self):
        r = self.out['release_order']
        self.assertEqual(r['releasePath'], '/api/release')
        self.assertEqual(r['beatsWhilePending'], 0)        # nothing overtakes the release
        self.assertGreater(r['beatsAfterRelease'], 0)
        self.assertTrue(r['newPressDiffers'])              # a fresh id for the new press
        self.assertEqual(r['beforeCap'], 0)                # a hung release delays the next press ...
        self.assertGreater(r['afterCap'], 0)               # ... by at most 0.7 s


if __name__ == '__main__':
    unittest.main()
