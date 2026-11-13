"""Final G-SIM confirmation, stage 4 (registration steer_follow_gsim_final4_v1; no ROS here).

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_GSIM_FINAL4_KO.md and PREREG_STEER_FOLLOW_GSIM_FINAL4.json (the JSON
is prereg() of this module; a test compares them). Stages 2 and 3 stay as recorded.

What is new against stage 3: image wf7 (main b00b666: the executor reads a fresh steering status before a tick that
its own long scan processing delayed, gc.freeze() before the loop; compose gives the SIM steer and executor containers
3 CPUs); the dashboard copy stays a92cfec. Parts (SIM clock, scene w1_force, a fresh stack per session):
  A  row following, four runs, the limits of steer_follow_gsim_v1 (as stage 3)
  F  the user's local commands as they are: sim_stack.py start, then three times sim_auto.py start and reset; every
     run must end at the row end; camera frames of run 3 (the README animation when F passes)
  B  the SIM dashboard, short: prepare, one press, heartbeats, tab hidden, screen STOP, start again after the STOP
  C  A17(G) bridge_stall only, to separate the stage 3 failure: C0 at the scene spawn without a pose reset (the first
     script of a fresh stack), C1-C3 with the pose reset of stage 3; the executor's yaw rate at the start of each run
     is recorded
Every number is a placeholder of the SIM (자리값 기반 SIM 결과), not field performance.
"""
from __future__ import annotations

from . import gsim_final3 as G3
from . import gsim_follow as GF

SCHEMA = 'ubts.steer_sim.gsim_final4.v1'
PREREG_ID = 'steer_follow_gsim_final4_v1'
RESULT_KIND = GF.RESULT_KIND
IMAGE = 'ubts-steer-sim:wf7'
CLOCK = 'sim'
B_ITEMS = ('prepare', 'one_press_running', 'heartbeat', 'tab_hidden_link_lost', 'screen_stop_while_running',
           'restart_after_stop')
START_YAW_OFFSET_DEG_S = 2.         # C: a run whose executor yaw rate in its first second (vehicle at rest) is above this
                                    # is counted as a start offset (stage 3: 4.9, 5.5, -2.1 deg/s in three of 25 runs)

COMMON = dict(G3.COMMON, image=IMAGE)
PART_A = dict(G3.PART_A)
PART_F = dict(G3.PART_F, camera='camera frames of run 3 (tools/steer_sim/capture_media.py camera on the user stack)',
              check=G3.PART_F['check'].replace('image wf6', 'image wf7'))
PART_B = dict(
    check='the SIM dashboard in Chrome, phases auto, stop, restart of the stage 3 script (with amendment 1: browser '
          'request times for the heartbeats, None-only missing speeds)',
    items=[i for i in G3.PART_B['items'] if i['name'] in B_ITEMS], passed='every item passes')
PART_C = dict(
    check=('one fresh stack, w1_force, SIM clock: C0 steering.exec_checks safety --only bridge_stall --still-speed 0.05 '
           '(no pose reset: the vehicle at the scene spawn), then C1, C2, C3 the same with --reset-pose'),
    criteria='bridge_stall of A17 as coded in exec_checks (with acceptance amendments 60 and 62)',
    passed='C0-C3 all pass',
    cause_rule=('recorded with each run: the executor yaw rate (estimator gyro, tick field yaw) in the first second of '
                'ACTIVE while the vehicle starts from rest. If every failing run has a start offset above %.1f deg/s '
                'and every run without such an offset passes, the stage 3 failure is put on the start offset (sensor '
                'side, reported with the pose reset or not); a failing run without an offset is put on the behaviour '
                'under test' % START_YAW_OFFSET_DEG_S))
REPEAT = ('a failing part may be run once more after a fix on the SIM side, written first as an amendment (section 8); '
          'both attempts are reported')


def prereg():
    """The machine-readable registration (PREREG_STEER_FOLLOW_GSIM_FINAL4.json)."""
    return dict(schema=SCHEMA + '.prereg', prereg_id=PREREG_ID, result_kind=RESULT_KIND,
                based_on=G3.PREREG_ID, common=COMMON, rtf_host_failure=G3.RTF_HOST_FAILURE,
                operator_timeout_s=G3.OPERATOR_TIMEOUT_S, heartbeat_gap_max_s=G3.HEARTBEAT_GAP_MAX_S,
                link_check_s=G3.LINK_CHECK_S, start_yaw_offset_deg_s=START_YAW_OFFSET_DEG_S,
                parts=dict(A=PART_A, F=PART_F, B=PART_B, C=PART_C), repeat=REPEAT, amendments=[])


judge_follow = G3.judge_follow
judge_sim_auto = G3.judge_sim_auto


def judge_browser(result):
    """Part B: the stage 3 browser items limited to B_ITEMS."""
    out = G3.judge_browser(result)
    items = {k: v for k, v in out['items'].items() if k in B_ITEMS}
    return dict(items=items, passed=all(v['passed'] for v in items.values()))


def start_yaw_offset(ticks, run_id):
    """The median executor yaw rate [deg/s] of the first 20 ACTIVE ticks of a run (the vehicle starts from rest)."""
    values = [r.get('yaw') for r in ticks if r.get('k') == 'tick' and r.get('run') == run_id and r.get('state') == 'ACTIVE'
              and isinstance(r.get('yaw'), (int, float))][:20]
    if not values:
        return None
    values.sort()
    return values[len(values) // 2]
