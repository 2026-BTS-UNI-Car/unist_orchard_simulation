"""Final G-SIM confirmation, stage 8b (registration steer_follow_gsim_final8b_v1; no ROS here).

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_GSIM_FINAL8B_KO.md and PREREG_STEER_FOLLOW_GSIM_FINAL8B.json (the
JSON is prereg() of this module; a test compares them). Stages 2-7 and the stage 8 record (not run) stay as recorded.

Stage 8b repeats the stage 7 registration (steer_follow_gsim_final7_v1) with two changes and nothing else:
  * the host CPU placement during the registered sessions (user decision of 2026-11-14 17:40 KST, temporary): the
    WSL VM process (vmmemWSL) is put on the P-core logical CPUs only (process affinity); if that is refused or does not
    take effect, the fallback is the heterogeneous thread scheduling policy of the active power scheme on AC
    (SCHEDPOLICY and SHORTSCHEDPOLICY 'prefer performant processors', then setactive SCHEME_CURRENT). The original
    values are recorded before and restored after the job (also on failure), and the restore is read back. Stage 7 put
    every failure on 1-2 s moments when the efficiency class 0 (E cores) was at 79-80 % while the P cores were 15-29 %;
  * the image ubts-steer-sim:wf11 (sim main 0f07467; against wf10 the dashboard copy e2a9785 with the restored menu and
    the PW-10 online scale of the executor, whose fast-SIM outputs were recorded as unchanged).
Criteria, run set, pre-check, host-fault rule, void reruns, RTF rule, failure events and every safety limit are the
stage 7 ones (the functions below are the stage 7 functions). Every number is a placeholder of the SIM (자리값 기반
SIM 결과), not field performance.
"""
from __future__ import annotations

from . import gsim_final3 as G3
from . import gsim_final7 as G7

SCHEMA = 'ubts.steer_sim.gsim_final8b.v1'
PREREG_ID = 'steer_follow_gsim_final8b_v1'
RESULT_KIND = G7.RESULT_KIND
IMAGE = 'ubts-steer-sim:wf11'
IMAGE_SOURCE_COMMIT = '0f074678ee8745a581d3b223a0e2a0943e1bf830'
CLOCK = G7.CLOCK
B_ITEMS = G7.B_ITEMS
MEASURE_DASHBOARD_PORT = G7.MEASURE_DASHBOARD_PORT

# stage 7 rules, unchanged
QUIET_CPU_PCT = G7.QUIET_CPU_PCT
QUIET_E_PCT = G7.QUIET_E_PCT
QUIET_WINDOW_S = G7.QUIET_WINDOW_S
QUIET_POLL_S = G7.QUIET_POLL_S
QUIET_MAX_WAIT_S = G7.QUIET_MAX_WAIT_S
HOST_FAULT_CPU_PCT = G7.HOST_FAULT_CPU_PCT
HOST_FAULT_LATE_S = G7.HOST_FAULT_LATE_S
HOST_FAULT_E_PCT = G7.HOST_FAULT_E_PCT
HOST_FAULT_WINDOW_S = G7.HOST_FAULT_WINDOW_S
SAMPLER_PERIOD_S = G7.SAMPLER_PERIOD_S
VOID_RERUNS = G7.VOID_RERUNS
E_CLASS = G7.E_CLASS
P_CLASS = '1'
B_SCRIPT_REASONS = G7.B_SCRIPT_REASONS

# the host placement (the only host change; logical CPU map read with GetLogicalProcessorInformationEx before the job)
P_LOGICAL = (0, 1, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19)     # efficiency class 1 (P cores, 6 cores x 2 threads)
E_LOGICAL = (2, 3, 4, 5, 6, 7, 8, 9, 20, 21)                    # efficiency class 0 (8 E cores + 2 low-power E cores)
P_MASK = sum(1 << cpu for cpu in P_LOGICAL)                     # 0xFFC03
SCHEDPOLICY_PREFER_PERFORMANT = 2                               # powercfg index 'prefer performant processors'
HOST_PLACEMENT = dict(
    method_order=['affinity', 'schedpolicy'],
    affinity=dict(process='vmmemWSL (or vmmem when that is the name of the WSL VM process)',
                  mask='0x%X' % P_MASK, logical_cpus=list(P_LOGICAL),
                  how='record the original affinity, set the P-core mask, read it back; restore the original after '
                      'the job and read it back'),
    schedpolicy=dict(when='the affinity is refused (protected process, access denied) or does not take effect',
                     settings=['SUB_PROCESSOR SCHEDPOLICY', 'SUB_PROCESSOR SHORTSCHEDPOLICY'], power_source='AC',
                     scheme='SCHEME_CURRENT (no scheme created, imported or deleted)',
                     value=SCHEDPOLICY_PREFER_PERFORMANT,
                     how='record the original AC indices, set both, powercfg /setactive SCHEME_CURRENT, read back; '
                         'restore both, setactive SCHEME_CURRENT, read back'),
    span='from before the job sampler starts until the end of part F; restored in a finally block (also on failure)',
    not_changed='no .wslconfig edit, no WSL restart, no reboot, no other host setting',
    effect_check=('recorded, not a criterion: the mean efficiency class 0 and class 1 loads of the sampler over every '
                  'judged run and the moments of the class 0 at or above 78 %, next to the stage 7 numbers'),
    if_no_method=('if neither method can be applied the stage 8b sessions are not run (they would repeat stage 7) and '
                  'the record says so'))

EVIDENCE = dict(G7.EVIDENCE, stage7_failures_e_class_pct='79-80 in the 0-2 s before each of the 6 failure events',
                stage7_failures_p_class_pct='15-29', stage7_record='STEER_FOLLOW_GSIM_FINAL_KO.md 16.7')

COMMON = dict(G7.COMMON, image=IMAGE,
              image_note=('built from the clean sim main %s, manifest 680 files = commit blobs, firmware build 04353B49; '
                          'not rebuilt for this stage' % IMAGE_SOURCE_COMMIT[:7]))
PART_A = dict(G7.PART_A)
PART_B = dict(G7.PART_B)
PART_F = dict(
    G7.PART_F,
    check=G7.PART_F['check'].replace('image wf10', 'image wf11'),
    note=('sim_stack.py starts its DEFAULT_IMAGE ubts-steer-sim:wf11 (local tool outside the repository) with the compose '
          'file of the branch main; sim_auto.py sends its go also when a heartbeat was refused (revision 5)'))
REPEAT = G7.REPEAT


def prereg():
    """The machine-readable registration (PREREG_STEER_FOLLOW_GSIM_FINAL8B.json)."""
    return dict(schema=SCHEMA + '.prereg', prereg_id=PREREG_ID, result_kind=RESULT_KIND,
                based_on=G7.PREREG_ID, changes_against_based_on=['host CPU placement (HOST_PLACEMENT)', 'image wf11'],
                common=COMMON, rtf_host_failure=G3.RTF_HOST_FAILURE,
                operator_timeout_s=G3.OPERATOR_TIMEOUT_S, heartbeat_gap_max_s=G3.HEARTBEAT_GAP_MAX_S,
                link_check_s=G3.LINK_CHECK_S,
                host_quiet=dict(cpu_pct=QUIET_CPU_PCT, e_class_pct=QUIET_E_PCT, window_s=QUIET_WINDOW_S,
                                poll_s=QUIET_POLL_S, max_wait_s=QUIET_MAX_WAIT_S),
                host_fault=dict(cpu_pct=HOST_FAULT_CPU_PCT, late_s=HOST_FAULT_LATE_S, e_class_pct=HOST_FAULT_E_PCT,
                                window_s=HOST_FAULT_WINDOW_S, sampler_period_s=SAMPLER_PERIOD_S,
                                void_reruns=VOID_RERUNS),
                e_class_key=E_CLASS, p_class_key=P_CLASS, b_script_reasons=list(B_SCRIPT_REASONS),
                host_placement=HOST_PLACEMENT, evidence=EVIDENCE,
                sim_stall_gap_s=G7.SIM_STALL_GAP_S, sim_stall_before_s=G7.SIM_STALL_BEFORE_S,
                measure_dashboard_port=MEASURE_DASHBOARD_PORT,
                parts=dict(A=PART_A, B=PART_B, F=PART_F), repeat=REPEAT, amendments=[])


# the stage 7 rules and judges, unchanged
judge_follow = G7.judge_follow
judge_sim_auto = G7.judge_sim_auto
judge_browser = G7.judge_browser
foreign_sessions = G7.foreign_sessions
sampler_rows = G7.sampler_rows
browser_failure_event = G7.browser_failure_event
sim_stall_cause = G7.sim_stall_cause
slot_verdict = G7.slot_verdict
judge_part_a = G7.judge_part_a
judge_part_f = G7.judge_part_f
e_load = G7.e_load
quiet_means = G7.quiet_means
is_quiet = G7.is_quiet
host_fault = G7.host_fault
failure_event = G7.failure_event
b_failure_event = G7.b_failure_event
