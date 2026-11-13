"""Final G-SIM confirmation, stage 7 (registration steer_follow_gsim_final7_v1; no ROS here).

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_GSIM_FINAL7_KO.md and PREREG_STEER_FOLLOW_GSIM_FINAL7.json (the JSON
is prereg() of this module; a test compares them). Stages 2-6 stay as recorded.

What is new against stage 6 (image wf10; the dashboard copy stays a92cfec; every safety limit unchanged):
  * the stage 6 part B failure was a steering_status_stale latch of the bridge (no MCU status line for more than
    150 ms of the bridge's SIM clock) after the ESP emulator process (the simulated MCU) was held 0.34 s of wall time
    while the bridge, in the same container, took the queued /clock samples: each process keeps its own SIM clock from
    /clock, so a stall of one process alone parts the two (a real MCU runs by itself). Not a lease TTL expiry, no defect
    of the latch; the Windows efficiency class 0 (E cores) was saturated then. Stage 7 lowers the idle CPU of the steer
    and sim containers without changing a decision: the bridge's SIM clock loop sleeps in the ROS wait until a /clock
    sample or a control request (a guard condition) instead of polling every 2 ms, and the bridge (SIM clock), the
    emulator and the sensor contract nodes have no parameter services, type description service or /rosout;
  * the host-quiet pre-check also needs the mean load of the efficiency class 0 (E cores) below QUIET_E_PCT, and the
    host-fault rule also counts a saturated efficiency class 0 (mean of the HOST_FAULT_WINDOW_S before the failure
    event at or above HOST_FAULT_E_PCT); stage 6 put every failure (B, F 1, F 2, and the void F 1) on moments when the
    E class was at 67-80 % (its ceiling is about 80 %: 8 E cores full, the 2 low-power ones idle) and the P cores were
    half free;
  * the part B failure event is the executor's actual stop: the wall time of the first executor tick after a go whose
    state is not ACTIVE and whose reason is not one the browser script causes (operator_stop, operator_link_lost,
    hold_released); without one, the first failed check of a registered item as before.
Parts (SIM clock, scene w1_force, a fresh stack per session), with the criteria of stages 5 and 6:
  A  row following, four runs, the limits of steer_follow_gsim_v1
  B  the SIM dashboard, all six items (prepare, one press, heartbeats, tab hidden, screen STOP, start again)
  F  the user's local commands (regression): sim_stack.py start (image wf10), three times sim_auto.py start and reset
Every number is a placeholder of the SIM (자리값 기반 SIM 결과), not field performance.
"""
from __future__ import annotations

from . import gsim_final3 as G3
from . import gsim_final6 as G6

SCHEMA = 'ubts.steer_sim.gsim_final7.v1'
PREREG_ID = 'steer_follow_gsim_final7_v1'
RESULT_KIND = G6.RESULT_KIND
IMAGE = 'ubts-steer-sim:wf10'
CLOCK = 'sim'
B_ITEMS = G6.B_ITEMS
MEASURE_DASHBOARD_PORT = G6.MEASURE_DASHBOARD_PORT

# host-quiet pre-check (before every judged run): stage 6 plus the efficiency class 0
QUIET_CPU_PCT = G6.QUIET_CPU_PCT            # 45 %, Windows total CPU, mean of the last QUIET_WINDOW_S, below
QUIET_E_PCT = 50.                           # efficiency class 0 (E cores), mean of the same samples, below
QUIET_WINDOW_S = G6.QUIET_WINDOW_S          # 30 s
QUIET_POLL_S = G6.QUIET_POLL_S              # 5 s
QUIET_MAX_WAIT_S = G6.QUIET_MAX_WAIT_S      # 900 s, then the run starts anyway (recorded)
# host-fault rule (a failed run, the samples of the HOST_FAULT_WINDOW_S before its failure event): stage 6 plus E
HOST_FAULT_CPU_PCT = G6.HOST_FAULT_CPU_PCT  # 70 %, one sample at or above
HOST_FAULT_LATE_S = G6.HOST_FAULT_LATE_S    # 1 s, or one sample that late
HOST_FAULT_E_PCT = 75.                      # or the mean efficiency class 0 load of the window at or above
HOST_FAULT_WINDOW_S = G6.HOST_FAULT_WINDOW_S
SAMPLER_PERIOD_S = G6.SAMPLER_PERIOD_S
VOID_RERUNS = G6.VOID_RERUNS                # 2 per slot (three attempts)
E_CLASS = '0'                               # the sampler's efficiency class key of the E cores (lowest class)
SIM_STALL_GAP_S = G6.SIM_STALL_GAP_S
SIM_STALL_BEFORE_S = G6.SIM_STALL_BEFORE_S
B_SCRIPT_REASONS = ('operator_stop', 'operator_link_lost', 'hold_released')   # stops the part B script causes
# (hold_released: the one tick of the screen STOP before ENDED operator_stop, stage 5 B)

EVIDENCE = dict(
    stage6_sampler='runs/gsim_stage6_20261114/sessions/s6_sampler.jsonl (record folder, outside the repository)',
    e_class_ceiling_pct=80.,
    failure_e_5s_mean_pct=dict(B=79.7, F_slot1_void=67.1, F_slot1_fail=79.0, F_slot2_fail=79.5),
    pass_run_samples=546, pass_run_e_5s_mean_at_or_above_75_pct=10.6, pass_run_e_5s_mean_at_or_above_70_pct=16.1,
    precheck_e_30s_mean_pct_range=[28.7, 44.9])

COMMON = dict(
    G6.COMMON, image=IMAGE,
    host_quiet=('the user\'s stack is stopped (parts A, B) and the heavy-measurement lock is held during every session; '
                'right before every judged run (A: after the 120 s settle, before the follow check; B: after the settle, '
                'before the browser script; F: before every sim_auto.py start) the sampler\'s last %.0f s must have a '
                'mean Windows total CPU below %.0f %% and a mean load of the efficiency class 0 (E cores) below %.0f %%, '
                'checked every %.0f s for at most %.0f s; the wait and the means are recorded; after %.0f s the run '
                'starts anyway and the record says so'
                % (QUIET_WINDOW_S, QUIET_CPU_PCT, QUIET_E_PCT, QUIET_POLL_S, QUIET_MAX_WAIT_S, QUIET_MAX_WAIT_S)),
    host_fault=('a failed run (A, F) or a failed session (B) is recorded as a host fault (void, not counted) only if the '
                'sampler has, in the %.0f s before its failure event, a sample with the Windows total CPU at or above '
                '%.0f %%, or a sample that came %.1f s or more after its due time (interval >= %.0f s + %.1f s), or a '
                'mean load of the efficiency class 0 (E cores) at or above %.0f %% over its samples. Failure event: A '
                'and F, the wall time of the first executor tick after the go whose state is not ACTIVE when the run '
                'did not end at the row end; B, the wall time of the first executor tick after a go whose state is not '
                'ACTIVE and whose reason is not %s (the script\'s own STOP and hidden tab), else the wall time of the '
                'first failed check of a registered item. A run that ends at the row end but fails a criterion is a '
                'failure (no host-fault test). The executor wall time is put on the Windows clock with the offset '
                'measured at the session. A void slot is run again at most %d times on a fresh stack (A, B) or as one '
                'more sim_auto.py start and reset on the same stack (F), after the pre-check; every attempt, void or '
                'not, is recorded and kept; a slot void three times is not judged (reported as not judged, never as '
                'a pass). Failures without a host fault count as failures. The RTF rule (5 s window RTF below 0.5) '
                'stays'
                % (HOST_FAULT_WINDOW_S, HOST_FAULT_CPU_PCT, HOST_FAULT_LATE_S, SAMPLER_PERIOD_S, HOST_FAULT_LATE_S,
                   HOST_FAULT_E_PCT, ' / '.join(B_SCRIPT_REASONS), VOID_RERUNS)))
PART_A = dict(G6.PART_A)
PART_B = dict(G6.PART_B)
PART_F = dict(
    G6.PART_F,
    check=G6.PART_F['check'].replace('image wf9', 'image wf10'),
    note=('sim_stack.py exports the compose file of the branch main (its --ref default) and starts the image wf10 '
          '(DEFAULT_IMAGE of the local tool); until this branch is merged main has the stage 6 compose file; the '
          'image wf10 carries the stage 7 changes (bridge, emulator, sensor contract). sim_auto.py sends its go also '
          'when a heartbeat was refused (revision 5)'))
REPEAT = G6.REPEAT


def prereg():
    """The machine-readable registration (PREREG_STEER_FOLLOW_GSIM_FINAL7.json)."""
    return dict(schema=SCHEMA + '.prereg', prereg_id=PREREG_ID, result_kind=RESULT_KIND,
                based_on=G6.PREREG_ID, common=COMMON, rtf_host_failure=G3.RTF_HOST_FAILURE,
                operator_timeout_s=G3.OPERATOR_TIMEOUT_S, heartbeat_gap_max_s=G3.HEARTBEAT_GAP_MAX_S,
                link_check_s=G3.LINK_CHECK_S,
                host_quiet=dict(cpu_pct=QUIET_CPU_PCT, e_class_pct=QUIET_E_PCT, window_s=QUIET_WINDOW_S,
                                poll_s=QUIET_POLL_S, max_wait_s=QUIET_MAX_WAIT_S),
                host_fault=dict(cpu_pct=HOST_FAULT_CPU_PCT, late_s=HOST_FAULT_LATE_S, e_class_pct=HOST_FAULT_E_PCT,
                                window_s=HOST_FAULT_WINDOW_S, sampler_period_s=SAMPLER_PERIOD_S,
                                void_reruns=VOID_RERUNS),
                e_class_key=E_CLASS, b_script_reasons=list(B_SCRIPT_REASONS), evidence=EVIDENCE,
                sim_stall_gap_s=SIM_STALL_GAP_S, sim_stall_before_s=SIM_STALL_BEFORE_S,
                measure_dashboard_port=MEASURE_DASHBOARD_PORT,
                parts=dict(A=PART_A, B=PART_B, F=PART_F), repeat=REPEAT, amendments=[])


judge_follow = G6.judge_follow
judge_sim_auto = G6.judge_sim_auto
judge_browser = G6.judge_browser
foreign_sessions = G6.foreign_sessions
sampler_rows = G6.sampler_rows
browser_failure_event = G6.browser_failure_event
sim_stall_cause = G6.sim_stall_cause
slot_verdict = G6.slot_verdict
judge_part_a = G6.judge_part_a
judge_part_f = G6.judge_part_f


# --- host sampler -------------------------------------------------------------------------------------------------------
def _window(samples, t_end, window_s):
    return [s for s in samples if t_end - window_s < s['wall'] <= t_end]


def e_load(sample, key=E_CLASS):
    """The efficiency class 0 load of one sampler row (its 'cls' dict), or None when the row has none."""
    value = (sample.get('cls') or {}).get(key)
    return None if value is None else float(value)


def quiet_means(samples, t_end, window_s=QUIET_WINDOW_S):
    """(total mean, E-class mean) of the samples in (t_end - window_s, t_end]; a value is None when fewer than half
    the window has it."""
    w = _window(samples, t_end, window_s)
    need = max(1, int(window_s / SAMPLER_PERIOD_S / 2))
    total = [s['total_pct'] for s in w]
    e = [v for v in (e_load(s) for s in w) if v is not None]
    return (sum(total) / len(total) if len(total) >= need else None,
            sum(e) / len(e) if len(e) >= need else None)


def is_quiet(samples, t_end, cpu_pct=QUIET_CPU_PCT, e_pct=QUIET_E_PCT, window_s=QUIET_WINDOW_S):
    """The stage 7 pre-check at t_end: both means known and below their thresholds."""
    total, e = quiet_means(samples, t_end, window_s)
    return total is not None and e is not None and total < cpu_pct and e < e_pct


def host_fault(samples, event_wall, window_s=HOST_FAULT_WINDOW_S, cpu_pct=HOST_FAULT_CPU_PCT, late_s=HOST_FAULT_LATE_S,
               e_pct=HOST_FAULT_E_PCT, period_s=SAMPLER_PERIOD_S):
    """The stage 7 host-fault test of a failure event (Windows wall time): the stage 6 test on the samples taken in
    [event_wall - window_s, event_wall] plus the mean efficiency class 0 load of those samples. Returns the stage 6
    dict with e_mean and the extra reason; void is False without samples (no evidence)."""
    out = G6.host_fault(samples, event_wall, window_s=window_s, cpu_pct=cpu_pct, late_s=late_s, period_s=period_s)
    if event_wall is None:
        out['e_mean'] = None
        return out
    w = [s for s in samples if event_wall - window_s <= s['wall'] <= event_wall]
    e = [v for v in (e_load(s) for s in w) if v is not None]
    e_mean = sum(e) / len(e) if e else None
    out['e_mean'] = None if e_mean is None else round(e_mean, 1)
    if e_mean is not None and e_mean >= e_pct:
        out['reasons'] = list(out['reasons']) + ['E class mean %.1f %% >= %.0f %%' % (e_mean, e_pct)]
        out['void'] = True
    return out


# --- failure events -----------------------------------------------------------------------------------------------------
failure_event = G6.failure_event


def b_failure_event(ticks, browser_result=None, script_reasons=B_SCRIPT_REASONS):
    """Part B: the first executor tick (in time order) after an ACTIVE tick of the same run whose state is not ACTIVE
    and whose reason is not one the browser script causes. Returns dict(t, wall, mono, state, reason, run, source
    'executor'), else the first failed registered check of browser_result (dict(wall, source 'browser_check')), else
    None."""
    active = set()
    for r in ticks:
        if r.get('k') != 'tick' or r.get('run') is None:
            continue
        run = r.get('run')
        if r.get('state') == 'ACTIVE':
            active.add(run)
            continue
        if run in active:
            active.discard(run)
            if r.get('reason') in script_reasons:
                continue
            diag = r.get('diag') or {}
            return dict(t=r.get('t'), wall=diag.get('ww'), mono=diag.get('wm'), state=r.get('state'),
                        reason=r.get('reason'), run=run, source='executor')
    if browser_result is not None:
        wall = browser_failure_event(browser_result)
        if wall is not None:
            return dict(wall=wall, source='browser_check')
    return None
