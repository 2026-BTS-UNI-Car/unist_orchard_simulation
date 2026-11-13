"""Final G-SIM confirmation, stage 5 (registration steer_follow_gsim_final5_v1; no ROS here).

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_GSIM_FINAL5_KO.md and PREREG_STEER_FOLLOW_GSIM_FINAL5.json (the JSON
is prereg() of this module; a test compares them). Stages 2-4 stay as recorded.

What is new against stage 4 (image wf8; the dashboard copy stays a92cfec):
  * the executor container logs its input thread (SpinMonitor: subscription callbacks, its own timer, a GIL probe, the
    main loop's polls; tick 'diag', 'input_gap' events) and the phase times of slow steps (diagnostics only);
  * the follow check waits until the DDS graph is quiet after its own participant joined (its /clock 10 s without a
    wall gap above 0.1 s, at most 60 s) before Start: stage 5 development probes put the stage 4 part A stalls on a
    participant joining the SIM stack (the bridge's /clock and IMU stop about 0.35 s while the SIM time goes on);
  * operator contract revision 5: the session is released at the end of a run and when its heartbeats stop; a session
    that only sent heartbeats does not refuse another session's go (stage 4 part B: session_conflict).
Parts (SIM clock, scene w1_force, a fresh stack per session):
  A  row following, four runs, the limits of steer_follow_gsim_v1 (as stages 3 and 4)
  B  the SIM dashboard, all six items of stage 4 (prepare, one press, heartbeats, tab hidden, screen STOP, start again)
  F  the user's local commands (regression): sim_stack.py start (image wf8), three times sim_auto.py start and reset
Every number is a placeholder of the SIM (자리값 기반 SIM 결과), not field performance.
"""
from __future__ import annotations

from . import gsim_final3 as G3
from . import gsim_final4 as G4

SCHEMA = 'ubts.steer_sim.gsim_final5.v1'
PREREG_ID = 'steer_follow_gsim_final5_v1'
RESULT_KIND = G4.RESULT_KIND
IMAGE = 'ubts-steer-sim:wf8'
CLOCK = 'sim'
B_ITEMS = G4.B_ITEMS
SIM_STALL_GAP_S = .25           # A cause rule: an executor /clock gap above this ...
SIM_STALL_MATCH_S = .1          # ... that the ESP emulator recorded at the same monotonic time (within this) ...
SIM_STALL_BEFORE_END_S = 1.     # ... and that ended within this before the run's end puts a failing run on the SIM side

COMMON = dict(G4.COMMON, image=IMAGE,
              host_quiet='the user\'s stack is stopped and the heavy-measurement lock is held during every session')
PART_A = dict(
    G4.PART_A,
    check=('steering.exec_checks follow --run <name> --out /runs/follow (one_press: Start, Arm, heartbeats, one go); the '
           'check first waits until its own /clock has run 10 s without a wall gap above 0.1 s (at most 60 s) and '
           'records it (discovery_settle)'),
    recorded=('per run: discovery_settle (wait, gaps); the executor input_gap events and the tick diag /clock gaps '
              'above %.2f s of the run; the ESP emulator clock_stalls; the executor loop p50/p99/max and skipped ticks'
              % SIM_STALL_GAP_S),
    cause_rule=('decided before the runs, reported with the verdict (the verdict stays as recorded): a failing run whose '
                'end comes within %.1f s after an executor /clock gap above %.2f s that the ESP emulator recorded at the '
                'same monotonic time (within %.1f s) is put on the SIM side (DDS delivery of the stack); any other '
                'failing run is put on the executor or the controller' % (SIM_STALL_BEFORE_END_S, SIM_STALL_GAP_S,
                                                                           SIM_STALL_MATCH_S)))
PART_B = dict(
    G4.PART_B,
    check=('the SIM dashboard in Chrome, phases auto, stop, restart of the stage 4 script, plus the page\'s own session '
           'digests (page_sessions, from the request bodies; not judged)'),
    recorded=('the executor\'s operator_link events (bind, takeover, release, conflict; session digests) and the page '
              'session digests: a session of another client that bound during the session is reported'))
PART_F = dict(
    G4.PART_F,
    check=G4.PART_F['check'].replace('image wf7', 'image wf8'),
    camera='not taken (the README animation of stage 4 stays)',
    note=('sim_auto.py of the local tool sends its go also when a heartbeat was refused (revision 5: the go takes over a '
          'heartbeat-only session; a session that made the go still refuses it)'))
REPEAT = G4.REPEAT


def prereg():
    """The machine-readable registration (PREREG_STEER_FOLLOW_GSIM_FINAL5.json)."""
    return dict(schema=SCHEMA + '.prereg', prereg_id=PREREG_ID, result_kind=RESULT_KIND,
                based_on=G4.PREREG_ID, common=COMMON, rtf_host_failure=G3.RTF_HOST_FAILURE,
                operator_timeout_s=G3.OPERATOR_TIMEOUT_S, heartbeat_gap_max_s=G3.HEARTBEAT_GAP_MAX_S,
                link_check_s=G3.LINK_CHECK_S, sim_stall_gap_s=SIM_STALL_GAP_S, sim_stall_match_s=SIM_STALL_MATCH_S,
                sim_stall_before_end_s=SIM_STALL_BEFORE_END_S,
                parts=dict(A=PART_A, B=PART_B, F=PART_F), repeat=REPEAT, amendments=[])


judge_follow = G3.judge_follow
judge_sim_auto = G3.judge_sim_auto
judge_browser = G4.judge_browser


def sim_stall_cause(end_mono, exec_gaps, emulator_stalls, gap_s=SIM_STALL_GAP_S, match_s=SIM_STALL_MATCH_S,
                    before_s=SIM_STALL_BEFORE_END_S):
    """Part A cause rule. end_mono: the monotonic time of the run's end (None: no end); exec_gaps: [(gap end monotonic,
    gap s)] of the executor's /clock; emulator_stalls: [(stall end monotonic, gap s)] of the ESP emulator. Returns
    'sim_delivery' when an executor gap above gap_s that the emulator also recorded (end within match_s) ended within
    before_s before the run's end, else 'executor_or_controller'."""
    if end_mono is None:
        return 'executor_or_controller'
    for end, gap in exec_gaps:
        if gap <= gap_s or not 0. <= end_mono - end <= before_s:
            continue
        if any(abs(e - end) <= match_s for e, _ in emulator_stalls):
            return 'sim_delivery'
    return 'executor_or_controller'


def foreign_sessions(link_events, page_digests):
    """Part B record: the session digests that bound / took over in the executor log but are not the page's."""
    seen = []
    for e in link_events:
        digest = e.get('session')
        if e.get('action') in ('bind', 'takeover') and digest and digest not in page_digests and digest not in seen:
            seen.append(digest)
    return seen
