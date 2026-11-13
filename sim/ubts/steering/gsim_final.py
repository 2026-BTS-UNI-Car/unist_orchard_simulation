"""Final G-SIM confirmation with the final image (registration steer_follow_gsim_final_v1; no ROS here).

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_GSIM_FINAL_KO.md and PREREG_STEER_FOLLOW_GSIM_FINAL.json (the
JSON is prereg() of this module; a test compares them). Do not change it after the first run; an amendment is
written in the document first.

Parts (all on the Docker stack, compose.ubts-steer.yaml, wall clock mode, the final image):
  A  row following with the one-press start (steering.exec_checks follow: Start, Arm, one go with the screen's
     heartbeats): the four runs and the limits of steer_follow_gsim_v1 (steering.gsim_follow), stricter validity
  B  the SIM dashboard screen (http://127.0.0.1:18090): prepare (start + Arm), one press, STOP from the screen, the tab
     hidden -> OPERATOR_LINK_LOST, the dashboard's own live map from the SIM LiDAR
  C  A17(G) executor scripts with the one-press executor (steering.exec_checks safety)
  D  A25 on the emulator path with the host bridge rule of amendment 43 (steering.p6a_checks a25)
  E  PW-08 on the Gazebo stack (steering.exec_checks pw08)
judge_follow() judges a part A record with the registered validity rule. Every number is a placeholder of the SIM
(자리값 기반 SIM 결과), not field performance.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

from . import gsim_follow as GF

SCHEMA = 'ubts.steer_sim.gsim_final.v1'
PREREG_ID = 'steer_follow_gsim_final_v1'
RESULT_KIND = GF.RESULT_KIND
IMAGE = 'ubts-steer-sim:wf5'
SETTLE_AFTER_HEALTHY_S = 120.        # wait after the sim container is healthy before the first step of a session
RTF_MIN = .9                         # every 5 s /clock window of the check (the run inside it) must be at or above
RERUNS_PER_SLOT = 1                  # an invalid run is run once more on a fresh stack; a second invalid one is reported
LINK_LOST_LIMIT_S = .5 + .05         # A17 amendment 31: operator link lost -> drive 0 within 0.5 s + 1 executor tick

PART_A = dict(
    check='steering.exec_checks follow --run <name> --out /runs/follow (start mode one_press)',
    scene=GF.SCENE, controller=dict(GF.CONTROLLER), runs=[dict(r) for r in GF.RUNS], criteria=dict(GF.CRITERIA),
    settle_m=GF.SETTLE_M, fresh_stack_per_run=True,
    validity=('the run is valid when the A10 rule of steer_follow_gsim_v1 holds (RTF of the run, emulator /clock wall '
              'gap above 100 ms, host latch made by a SIM stall) AND the smallest 5 s /clock window RTF of the check '
              '(rtf.min of the record) is >= %.1f; an invalid run is recorded, not judged, and run once more on a fresh '
              'stack; a slot invalid twice is reported as not judged' % RTF_MIN),
    passed='every judged run passes every criterion of steer_follow_gsim_v1 (row_end, no pause after the drive '
           'started, lateral / heading / speed / e / clearance limits, rest speed, no guard ending, no commanded '
           'steering motion in the 3 s after the end) and at least three of the four slots are judged')
PART_B = dict(
    check='the SIM dashboard in Chrome (playwright, the page as an operator uses it); services sim, steer, recorder, '
          'dashboard, executor; scene w1_force',
    items=[
        dict(name='prepare', rule='the auto panel offers 준비; pressing it (and Arm if the page offers it) reaches '
                                  '자율 주행 시작 within 16 s; a second press is recorded'),
        dict(name='one_press_running', rule='go_ready, one press -> 주행 중 within 10 s and the vehicle moves '
                                            '(emulator speed > 0.05 m/s)'),
        dict(name='heartbeat', rule='heartbeats while the tab is visible: largest gap of the last 20 < 0.35 s'),
        dict(name='tab_hidden_link_lost', rule='tab hidden -> run_state paused, stop_reason OPERATOR_LINK_LOST at '
                                               '1.5 s, no heartbeat after hiding, the vehicle stopped (< 0.05 m/s) '
                                               '4 s after hiding, visible again -> 다시 출발 and still paused'),
        dict(name='screen_stop_while_running', rule='a new run, one press, 6 s of driving, the status-bar STOP -> '
                                                    'stop_detail operator_stop (stop_reason OPERATOR_OR_BACKSTOP), '
                                                    'the vehicle stopped within 3 s, no restart'),
        dict(name='live_map', rule='the dashboard\'s own live map started from the page builds from the SIM scans: '
                                   '/api/live/map scan_count > 0 after 20 s'),
    ],
    validity='the session is valid when the RTF of the check node stays >= %.1f in the steps (else run once more)'
             % RTF_MIN,
    passed='every item passes')
PART_C = dict(
    check='steering.exec_checks safety --out /runs/safety (scene w0_force, the executor in one_press)',
    criteria='SIM_ACCEPTANCE_KO.md A17 per script as coded in exec_checks (with amendment 31: operator link lost -> '
             'drive 0 <= 0.5 s + 1 tick); deadman_release is not applicable to one_press',
    validity='A10 rule per step (exec_checks); invalid steps are run once more with --only',
    passed='every valid step passes; steps invalid twice are reported')
PART_D = dict(check='steering.p6a_checks a25 --out /runs/a25 (scene w0_force)',
              criteria='A25 as coded in p6a_checks (joint - shared model p99 <= 0.2 deg at the best lag, no joint speed '
                       'spikes, the steering moved, no drive refusal), with the host bridge of the final image '
                       '(amendment 43: steering_not_following counts settled ramp lines only)',
              validity='A10 rule', passed='every criterion passes')
PART_E = dict(check='steering.exec_checks pw08 --out /runs/pw08 (scene w0_force)',
              criteria='executor: POST /api/params speed_scale 1.10 accepted at rest, the status shows it with revision '
                       '+1 and a new digest, the next run starts with that digest, the executor table speed at 50 % '
                       'duty moves by 1.10 +- 1 %, the default preset restored; plant: the Gazebo plant steady speed '
                       'ratio at 50 % duty is reported (the Gazebo drive model does not read the tuning file)',
              validity='A10 rule for both drives', passed='every executor criterion passes')


def prereg():
    """The machine-readable registration (PREREG_STEER_FOLLOW_GSIM_FINAL.json)."""
    return dict(schema=SCHEMA + '.prereg', prereg_id=PREREG_ID, result_kind=RESULT_KIND, image=IMAGE,
                based_on=GF.PREREG_ID, clock='wall', settle_after_healthy_s=SETTLE_AFTER_HEALTHY_S, rtf_min=RTF_MIN,
                reruns_per_slot=RERUNS_PER_SLOT, link_lost_limit_s=LINK_LOST_LIMIT_S,
                parts=dict(A=PART_A, B=PART_B, C=PART_C, D=PART_D, E=PART_E),
                host='before each session the host-load check of STACK_OPERATIONS_KO.md section 2 is recorded; the '
                     'stall probe is not run in these sessions')


def judge_follow(report):
    """Part A verdict of one GSIM_FOLLOW_RUN.json: 'pass' / 'fail' / 'invalid' (A10 rule or a 5 s window below
    RTF_MIN) with the checks of steer_follow_gsim_v1 as recorded."""
    validity = report.get('validity') or {}
    rtf = report.get('rtf') or {}
    rtf_min = rtf.get('min')
    reasons = []
    if not validity.get('valid'):
        reasons.append('A10: %s' % validity.get('reason'))
    if rtf_min is None or rtf_min < RTF_MIN:
        reasons.append('rtf.min %s < %.1f' % (rtf_min, RTF_MIN))
    checks = report.get('checks') or []
    verdict = 'invalid' if reasons else ('pass' if checks and all(c['ok'] for c in checks) else 'fail')
    return dict(schema=SCHEMA + '.follow', prereg_id=PREREG_ID, run=(report.get('run') or {}).get('name'),
                verdict=verdict, invalid_reasons=reasons, rtf_min=rtf_min, failed=[c['check'] for c in checks
                                                                                   if not c['ok']],
                metrics={k: (report.get('metrics') or {}).get(k) for k in (
                    'end_reason', 'lateral_rmse_m', 'lateral_p95_m', 'lateral_max_m', 'heading_rmse_deg',
                    'heading_max_deg', 'speed_max_m_s', 'row_end_error_m', 'min_clearance_m', 'rest_speed_m_s',
                    'distance_m', 'duration_s')},
                result_kind=RESULT_KIND)


def main(argv=None):
    """python -B -m steering.gsim_final --prereg | <follow folder> ...: the registration JSON, or the part A verdict of
    each run folder (writes GSIM_FINAL_FOLLOW.json next to GSIM_FOLLOW_RUN.json)."""
    args = sys.argv[1:] if argv is None else argv
    if args[:1] == ['--prereg']:
        sys.stdout.buffer.write((json.dumps(prereg(), indent=1, ensure_ascii=False) + '\n').encode('utf-8'))
        return 0
    for folder in args:
        folder = Path(folder)
        report = json.loads((folder / 'GSIM_FOLLOW_RUN.json').read_bytes().decode('utf-8'))
        result = judge_follow(report)
        (folder / 'GSIM_FINAL_FOLLOW.json').write_bytes(
            (json.dumps(result, indent=1, ensure_ascii=False, allow_nan=False) + '\n').encode('utf-8'))
        print(json.dumps(dict(run=result['run'], verdict=result['verdict'], reasons=result['invalid_reasons'],
                              failed=result['failed'])))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
