"""Final G-SIM confirmation, stage 3 (registration steer_follow_gsim_final3_v1; no ROS here).

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_GSIM_FINAL3_KO.md and PREREG_STEER_FOLLOW_GSIM_FINAL3.json (the JSON
is prereg() of this module; a test compares them). Do not change it after the first registered run; an amendment is
written in the document first. Stage 2 (steer_follow_gsim_final_v1, image wf5) stays as recorded.

What is new against stage 2 (decisions R60-R63, acceptance amendments 60-61): image wf6 (operator_timeout_s 1.0 s, the
start check speed 0.08 m/s, the IMU game rotation vector, the dashboard copy a92cfec); the judged runs are in the SIM
clock mode (UBTS_STEER_CLOCK=sim for the bridge wrapper and the executor), wall clock runs are a load record only;
A17(G) one_press runs on W1 with the vehicle put back at the start before each script (exec_checks safety
--reset-pose); the user's local commands (tools/local_env sim_stack.py and sim_auto.py) must take three runs to the
row end.

Parts:
  A  row following, four runs (steering.exec_checks follow), limits of steer_follow_gsim_v1, SIM clock
  W  load record: nominal_1 and lateral_p040 in the wall clock mode on one stack (not judged)
  B  the SIM dashboard in Chrome: prepare, one press, heartbeats, tab hidden, screen STOP, start again after the STOP,
     the dashboard's own live map on /map
  C  A17(G) executor scripts with the one-press executor on W1 (steering.exec_checks safety --reset-pose)
  E  PW-08 on W1 (steering.exec_checks pw08)
  F  the user's local commands: sim_stack.py start, then three times sim_auto.py start (row end) and sim_auto.py reset
Every number is a placeholder of the SIM (자리값 기반 SIM 결과), not field performance.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

from . import gsim_follow as GF

SCHEMA = 'ubts.steer_sim.gsim_final3.v1'
PREREG_ID = 'steer_follow_gsim_final3_v1'
RESULT_KIND = GF.RESULT_KIND
IMAGE = 'ubts-steer-sim:wf6'
CLOCK = 'sim'
SETTLE_AFTER_HEALTHY_S = 120.
RTF_HOST_FAILURE = .5                # a judged step with a 5 s window RTF below this is a host failure: run once more
RERUNS_PER_SLOT = 1
OPERATOR_TIMEOUT_S = 1.              # decision R60
LINK_LOST_LIMIT_S = OPERATOR_TIMEOUT_S + .05      # A17 amendment 60: operator_timeout_s + 1 executor tick
HEARTBEAT_GAP_MAX_S = .5             # half of operator_timeout_s
LINK_CHECK_S = 2.                    # the tab-hidden check looks at the run state this long after hiding
SIM_AUTO_RUNS = 3

COMMON = dict(
    image=IMAGE, clock=CLOCK, settle_after_healthy_s=SETTLE_AFTER_HEALTHY_S,
    validity=('SIM clock mode: the times of the executor, the bridge wrapper and the checks are SIM times, so an RTF '
              'below 1 does not make a run invalid (acceptance amendment 61). The RTF of every judged step is reported. '
              'A judged step with a 5 s window RTF below %.1f is a host failure: recorded, not judged, and run once more '
              'on a fresh stack; twice is reported as not judged' % RTF_HOST_FAILURE),
    host='the host-load check of STACK_OPERATIONS_KO.md section 2 is recorded before each session (waiting at most 60 s); '
         'the stall probe is not run')

PART_A = dict(
    check='steering.exec_checks follow --run <name> --out /runs/follow (one_press: Start, Arm, heartbeats, one go)',
    scene=GF.SCENE, controller=dict(GF.CONTROLLER), runs=[dict(r) for r in GF.RUNS], criteria=dict(GF.CRITERIA),
    settle_m=GF.SETTLE_M, fresh_stack_per_run=True, clock=CLOCK,
    passed='every judged run passes every criterion of steer_follow_gsim_v1 and at least three of four slots are judged')
PART_W = dict(
    check='the same follow check in the wall clock mode, nominal_1 then lateral_p040 on one stack',
    judged=False, note='load record of the vehicle code path (acceptance amendment 61); reported with the RTF and the '
                       'executor loop times')
PART_B = dict(
    check='the SIM dashboard in Chrome (playwright) as an operator uses it; services sim, steer, recorder, dashboard, '
          'executor; scene w1_force; SIM clock',
    items=[
        dict(name='prepare', rule='the auto panel offers 준비; pressing it (and Arm if offered) reaches 자율 주행 시작 '
                                  'within 16 s'),
        dict(name='one_press_running', rule='one press -> 주행 중 within 10 s and the vehicle moves (emulator speed > '
                                            '0.05 m/s)'),
        dict(name='heartbeat', rule='largest gap of the last 20 heartbeats while visible < %.2f s' % HEARTBEAT_GAP_MAX_S),
        dict(name='tab_hidden_link_lost', rule='tab hidden -> run_state paused, stop_reason OPERATOR_LINK_LOST %.1f s '
                                               'after hiding, no heartbeat after hiding, the vehicle below 0.05 m/s 4 s '
                                               'after hiding, visible again -> 다시 출발 and still paused' % LINK_CHECK_S),
        dict(name='screen_stop_while_running', rule='a new run, one press, about 6 s of driving (> 0.05 m/s), the '
                                                    'status-bar STOP -> stop_detail operator_stop (OPERATOR_OR_BACKSTOP), '
                                                    'below 0.05 m/s 3 s later and 6 s later'),
        dict(name='restart_after_stop', rule='after that STOP: latch acknowledged, a new run prepared, one press -> the '
                                             'vehicle drives again (> 0.05 m/s within 6 s), the STOP stops it again '
                                             '(operator_stop, below 0.05 m/s 3 s later)'),
        dict(name='live_map', rule='on /map, 그리기 시작, 20 s later /api/live/map scan_count > 0 and cell_count > 0'),
    ],
    passed='every item passes')
PART_C = dict(
    check='steering.exec_checks safety --out /runs/safety --reset-pose --still-speed 0.05 (scene w1_force, one_press, '
          'SIM clock)',
    criteria='SIM_ACCEPTANCE_KO.md A17 per script as coded in exec_checks, with amendment 60 (operator link lost -> '
             'drive 0 <= operator_timeout_s + 1 tick) and amendment 62 (on W1 the vehicle is still below 0.05 m/s truth: '
             'the start area slopes about 1.6 deg and the plant creeps 0.016-0.03 m/s with drive 0); deadman_release is '
             'not applicable to one_press',
    passed='every script passes; a script that stops with an error (for example a refused go) fails')
PART_E = dict(
    check='steering.exec_checks pw08 --out /runs/pw08 (scene w1_force, SIM clock)',
    criteria='the executor criteria of steer_follow_gsim_final_v1 part E (the restore is read after the status shows it, '
             'amendment 2 of stage 2 now in the check); the plant speed ratio is reported',
    passed='every executor criterion passes')
PART_F = dict(
    check=('the user\'s local commands as they are (the local tool folder tools/local_env outside the repository): '
           'python -B sim_stack.py start --no-browser (image wf6, SIM clock by default), 150 s, then %d times: '
           'python -B sim_auto.py start; python -B sim_auto.py reset; 20 s' % SIM_AUTO_RUNS),
    criteria='each run ends at the row end: the executor\'s run_end of that run has end_reason row_end (the screen '
             'line of sim_auto shows the end); distance and duration are reported',
    passed='%d of %d runs end at the row end' % (SIM_AUTO_RUNS, SIM_AUTO_RUNS))


AMENDMENTS = [dict(
    n=1, date='2026-11-13', written='after parts A, W, B, C, E (09:29-10:31 UTC), before B is run again and before F',
    B=('evaluation defects of the browser script: (i) a speed check written abs(v or 9) < 0.05 counted an emulator speed '
       'of exactly 0.0 as moving (attempt 1: "screen STOP: no restart (3 s later)" and "restart: the STOP stops it '
       'again" with v 0.0); (ii) the heartbeat times were taken when the own event handler of the script ran, and the '
       'script blocked that handler with synchronous docker exec calls (attempt 1: gap 1.926 s while the executor saw '
       'no link loss). (i) is judged again on the same record from the recorded values; (ii) cannot be judged from the '
       'record. B is run once more on a fresh stack with the script fixed (speeds: None only counts as missing; '
       'heartbeat times: the request start times of the browser). The verdict of B is attempt 2; attempt 1 is kept'),
    C=('evaluation defect of second_screen_and_cli: exec_checks still expected the end reason bridge_error or '
       'bridge_latched, against acceptance amendment 34 (registered before this registration: a STOP of another '
       'client that reaches the bridge first ends the run as operator_stop). The same record is judged again with '
       'operator_stop expected; exec_checks is fixed for later runs. Every other script as recorded'))]


def prereg():
    """The machine-readable registration (PREREG_STEER_FOLLOW_GSIM_FINAL3.json)."""
    return dict(schema=SCHEMA + '.prereg', prereg_id=PREREG_ID, result_kind=RESULT_KIND, based_on='steer_follow_gsim_final_v1',
                common=COMMON, rtf_host_failure=RTF_HOST_FAILURE, reruns_per_slot=RERUNS_PER_SLOT,
                operator_timeout_s=OPERATOR_TIMEOUT_S, link_lost_limit_s=LINK_LOST_LIMIT_S,
                heartbeat_gap_max_s=HEARTBEAT_GAP_MAX_S, link_check_s=LINK_CHECK_S,
                parts=dict(A=PART_A, W=PART_W, B=PART_B, C=PART_C, E=PART_E, F=PART_F), amendments=AMENDMENTS)


def judge_follow(report):
    """Part A verdict of one GSIM_FOLLOW_RUN.json: 'pass' / 'fail' / 'invalid' (an error before the drive, or a 5 s
    window RTF below RTF_HOST_FAILURE)."""
    rtf_min = (report.get('rtf') or {}).get('min')
    reasons = []
    if report.get('start_error'):
        reasons.append('start error: %s' % report['start_error'])
    if rtf_min is None or rtf_min < RTF_HOST_FAILURE:
        reasons.append('rtf.min %s < %.1f (host failure)' % (rtf_min, RTF_HOST_FAILURE))
    checks = report.get('checks') or []
    verdict = 'invalid' if reasons else ('pass' if checks and all(c['ok'] for c in checks) else 'fail')
    return dict(schema=SCHEMA + '.follow', prereg_id=PREREG_ID, run=(report.get('run') or {}).get('name'),
                clock=report.get('clock'), verdict=verdict, invalid_reasons=reasons, rtf_min=rtf_min,
                failed=[c['check'] for c in checks if not c['ok']],
                metrics={k: (report.get('metrics') or {}).get(k) for k in (
                    'end_reason', 'lateral_rmse_m', 'lateral_p95_m', 'lateral_max_m', 'heading_rmse_deg',
                    'heading_max_deg', 'speed_max_m_s', 'row_end_error_m', 'min_clearance_m', 'rest_speed_m_s',
                    'distance_m', 'duration_s')},
                result_kind=RESULT_KIND)


BROWSER_ITEMS = {    # registered item -> the check names of the browser script that make it
    'prepare': ("auto: '준비' shown", "prepare: start + arm -> armed ('자율 주행 시작')"),
    'one_press_running': ("one press: running ('주행 중')",
                          'one press: the car drives while the screen is visible (emulator speed)'),
    'heartbeat': ('heartbeat about every 100 ms while visible',),
    'tab_hidden_link_lost': ('tab hidden: heartbeats stop, executor paused (OPERATOR_LINK_LOST)', 'no heartbeat after hiding',
                             'link lost: the car stops (emulator speed 4 s after hiding)',
                             "visible again: '다시 출발', still paused (no automatic restart)"),
    'screen_stop_while_running': ('stop: the car drives before the STOP',
                                  'screen STOP while running: stop_detail operator_stop, OPERATOR_OR_BACKSTOP',
                                  'screen STOP: the car stopped within 3 s', 'screen STOP: no restart (3 s later)'),
    'restart_after_stop': ('restart after the STOP: one press drives again (emulator speed > 0.05 m/s)',
                           'restart: the STOP stops it again (operator_stop, vehicle < 0.05 m/s 3 s later)'),
    'live_map': ("live map (dashboard's own map) builds from the SIM scans: scan_count > 0",
                 'live map: grid cells (cell_count > 0)'),
}


def judge_browser(result):
    """Part B items from the browser script's result.json: each item passes when all its checks are there and ok
    (the first prepare / one-press checks of phase auto; the stop phase repeats them, those count for that item)."""
    checks = result.get('checks') or []
    items = {}
    for item, names in BROWSER_ITEMS.items():
        found = []
        for name in names:
            hits = [c for c in checks if c.get('name') == name]
            found.append(hits[0]['ok'] if hits else None)
        items[item] = dict(passed=all(v is True for v in found), checks=dict(zip(names, found)))
    return dict(items=items, passed=all(v['passed'] for v in items.values()))


def judge_sim_auto(run_ends):
    """Part F from the executor's run_end events of the sim_auto runs (in order)."""
    reasons = [e.get('end_reason') for e in run_ends]
    return dict(runs=len(run_ends), end_reasons=reasons, row_end=sum(r == 'row_end' for r in reasons),
                passed=len(run_ends) == SIM_AUTO_RUNS and all(r == 'row_end' for r in reasons))


def main(argv=None):
    """python -B -m steering.gsim_final3 --prereg | <follow folder> ...: the registration JSON, or the part A verdict of
    each run folder (writes GSIM_FINAL3_FOLLOW.json next to GSIM_FOLLOW_RUN.json)."""
    args = sys.argv[1:] if argv is None else argv
    if args[:1] == ['--prereg']:
        sys.stdout.buffer.write((json.dumps(prereg(), indent=1, ensure_ascii=False) + '\n').encode('utf-8'))
        return 0
    for folder in args:
        folder = Path(folder)
        report = json.loads((folder / 'GSIM_FOLLOW_RUN.json').read_bytes().decode('utf-8'))
        result = judge_follow(report)
        (folder / 'GSIM_FINAL3_FOLLOW.json').write_bytes(
            (json.dumps(result, indent=1, ensure_ascii=False, allow_nan=False) + '\n').encode('utf-8'))
        print(json.dumps(dict(run=result['run'], verdict=result['verdict'], reasons=result['invalid_reasons'],
                              failed=result['failed'])))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
