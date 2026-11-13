"""Final G-SIM confirmation, stage 6 (registration steer_follow_gsim_final6_v1; no ROS here).

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_GSIM_FINAL6_KO.md and PREREG_STEER_FOLLOW_GSIM_FINAL6.json (the JSON
is prereg() of this module; a test compares them). Stages 2-5 stay as recorded.

What is new against stage 5 (image wf9; the dashboard copy stays a92cfec; every safety limit unchanged):
  * the executor container is less sensitive to a busy host without a change of what it decides: the stage 5 input
    diagnostics are opt-in (UBTS_STEER_EXEC_INPUT_DIAG=1; off, a tick carries only its monotonic and wall time), the
    SIM clock loop waits for the /clock sample that reaches its next grid point instead of polling every 2 ms, the ROS
    node has no parameter services, type description service or /rosout publisher, and the executor service has a CPU
    weight above the other services (cpu_shares 8192 = cgroup v2 cpu.weight 313; quotas unchanged);
  * a host-quiet pre-check before every judged run and an extended host-fault rule (the Windows per-process sampler):
    stage 5 put all its failures (A 2, F 3) on moments when the executor's scan processing took 0.15-0.32 s while the
    Windows host CPU peaked at 74-88 % (RTF above 0.87, so the RTF rule did not apply); stage 6 step 1 (development)
    saw a failing user command with the host at 54 % mean / 71.5 % max in its last 5 s and every executor phase slowed;
  * the measurement stacks of parts A and B publish the dashboard on 127.0.0.1:18091 (a browser tab left open on
    18090 attached to the stage 5 B stack); part F stays the user's command on 18090.
Parts (SIM clock, scene w1_force, a fresh stack per session), with the criteria of stage 5:
  A  row following, four runs, the limits of steer_follow_gsim_v1
  B  the SIM dashboard, all six items (prepare, one press, heartbeats, tab hidden, screen STOP, start again)
  F  the user's local commands (regression): sim_stack.py start (image wf9), three times sim_auto.py start and reset
Every number is a placeholder of the SIM (자리값 기반 SIM 결과), not field performance.
"""
from __future__ import annotations

from . import gsim_final3 as G3
from . import gsim_final4 as G4
from . import gsim_final5 as G5

SCHEMA = 'ubts.steer_sim.gsim_final6.v1'
PREREG_ID = 'steer_follow_gsim_final6_v1'
RESULT_KIND = G5.RESULT_KIND
IMAGE = 'ubts-steer-sim:wf9'
CLOCK = 'sim'
B_ITEMS = G5.B_ITEMS
MEASURE_DASHBOARD_PORT = 18091      # A and B measurement stacks (compose override); F: the user's 18090

# host-quiet pre-check (before every judged run)
QUIET_CPU_PCT = 45.                 # Windows total CPU, mean of the sampler's samples of the last QUIET_WINDOW_S, below
QUIET_WINDOW_S = 30.
QUIET_POLL_S = 5.
QUIET_MAX_WAIT_S = 900.             # then the run starts anyway (recorded)
# extended host-fault rule (a failed run, the samples of the HOST_FAULT_WINDOW_S before its failure event)
HOST_FAULT_CPU_PCT = 70.            # one sample at or above
HOST_FAULT_LATE_S = 1.              # or one sample that came this much after its due time (interval >= period + 1 s)
HOST_FAULT_WINDOW_S = 5.
SAMPLER_PERIOD_S = 1.
VOID_RERUNS = 2                     # a void slot is run again at most twice (three attempts)
SIM_STALL_GAP_S = G5.SIM_STALL_GAP_S
SIM_STALL_BEFORE_S = 1.

COMMON = dict(
    G5.COMMON, image=IMAGE,
    host=('the host-load check of STACK_OPERATIONS_KO.md section 2 is recorded before each session (waiting at most '
          '60 s); the Windows per-process sampler (1 s: total CPU from GetSystemTimes, the sampler lateness, the top '
          'processes) runs through every session; docker stats every 15 s; the stall probe is not run'),
    host_quiet=('the user\'s stack is stopped (parts A, B) and the heavy-measurement lock is held during every session; '
                'right before every judged run (A: after the 120 s settle, before the follow check; B: after the settle, '
                'before the browser script; F: before every sim_auto.py start) the mean Windows total CPU of the '
                'sampler\'s last %.0f s must be below %.0f %%, checked every %.0f s for at most %.0f s; the wait and the '
                'means are recorded; after %.0f s the run starts anyway and the record says so'
                % (QUIET_WINDOW_S, QUIET_CPU_PCT, QUIET_POLL_S, QUIET_MAX_WAIT_S, QUIET_MAX_WAIT_S)),
    host_fault=('a failed run (A, F) or a failed session (B) is recorded as a host fault (void, not counted) only if the '
                'sampler has, in the %.0f s before its failure event, a sample with the Windows total CPU at or above '
                '%.0f %% or a sample that came %.1f s or more after its due time (interval >= %.0f s + %.1f s). Failure '
                'event: A and F, the wall time of the first executor tick after the go whose state is not ACTIVE when '
                'the run did not end at the row end; B, the wall time of the first failed check of a registered item. '
                'A run that ends at the row end but fails a criterion is a failure (no host-fault test). The executor '
                'wall time is put on the Windows clock with the offset measured at the session (docker exec time.time() '
                'between two host readings). A void slot is run again at most %d times on a fresh stack (A, B) or as '
                'one more sim_auto.py start and reset on the same stack (F), after the pre-check; every attempt is '
                'recorded; a slot void three times is not judged. Failures without a host fault count as failures. '
                'The RTF rule (5 s window RTF below 0.5) stays'
                % (HOST_FAULT_WINDOW_S, HOST_FAULT_CPU_PCT, HOST_FAULT_LATE_S, SAMPLER_PERIOD_S, HOST_FAULT_LATE_S,
                   VOID_RERUNS)),
    dashboard_port=('parts A and B: 127.0.0.1:%d (a compose override file of the record folder with ports !override); '
                    'part F: 127.0.0.1:18090 as the user\'s command' % MEASURE_DASHBOARD_PORT))
RECORDED = ('per run: the host-quiet pre-check (wait, means), the sampler over the run (mean, p90, max, the largest '
            'lateness, the top processes of the highest samples), the executor scan processing p50/p99/max (proc_ms of '
            'the ACTIVE ticks), the executor loop p50/p99/max and skipped ticks, the ESP emulator clock_stalls, '
            'discovery_settle (A), the failure event and the host-fault test (failed runs)')
PART_A = dict(
    G5.PART_A, recorded=RECORDED,
    cause_rule=('decided before the runs, reported with the verdict (the verdict stays as recorded): a failing run '
                'whose failure event comes within %.1f s after the end of an ESP emulator /clock stall above %.2f s '
                '(monotonic time; the executor input diagnostics are off by default, so the emulator record alone) is '
                'put on the SIM side (DDS delivery of the stack); any other failing run is put on the executor or the '
                'controller; the host-fault test is reported apart' % (SIM_STALL_BEFORE_S, SIM_STALL_GAP_S)))
PART_B = dict(
    G5.PART_B,
    check=('the SIM dashboard in Chrome on 127.0.0.1:%d, phases auto, stop, restart of the stage 4 script (each check '
           'with its wall time), plus the page\'s own session digests (page_sessions; not judged)'
           % MEASURE_DASHBOARD_PORT))
PART_F = dict(
    G5.PART_F,
    check=G5.PART_F['check'].replace('image wf8', 'image wf9'),
    recorded=RECORDED,
    note=('sim_stack.py exports the compose file of the branch main (its --ref default); until this branch is merged '
          'that file has neither the executor CPU weight nor UBTS_STEER_EXEC_INPUT_DIAG (the image\'s default, off, '
          'applies); the image wf9 carries the executor changes. sim_auto.py sends its go also when a heartbeat was '
          'refused (revision 5)'))
REPEAT = ('a failing part may be run once more after a fix on the SIM or executor side that does not loosen a safety '
          'limit, written first as an amendment (section 8) and committed; both attempts are reported. Host-fault '
          'reruns are not repeats')


def prereg():
    """The machine-readable registration (PREREG_STEER_FOLLOW_GSIM_FINAL6.json)."""
    return dict(schema=SCHEMA + '.prereg', prereg_id=PREREG_ID, result_kind=RESULT_KIND,
                based_on=G5.PREREG_ID, common=COMMON, rtf_host_failure=G3.RTF_HOST_FAILURE,
                operator_timeout_s=G3.OPERATOR_TIMEOUT_S, heartbeat_gap_max_s=G3.HEARTBEAT_GAP_MAX_S,
                link_check_s=G3.LINK_CHECK_S,
                host_quiet=dict(cpu_pct=QUIET_CPU_PCT, window_s=QUIET_WINDOW_S, poll_s=QUIET_POLL_S,
                                max_wait_s=QUIET_MAX_WAIT_S),
                host_fault=dict(cpu_pct=HOST_FAULT_CPU_PCT, late_s=HOST_FAULT_LATE_S, window_s=HOST_FAULT_WINDOW_S,
                                sampler_period_s=SAMPLER_PERIOD_S, void_reruns=VOID_RERUNS),
                sim_stall_gap_s=SIM_STALL_GAP_S, sim_stall_before_s=SIM_STALL_BEFORE_S,
                measure_dashboard_port=MEASURE_DASHBOARD_PORT,
                parts=dict(A=PART_A, B=PART_B, F=PART_F), repeat=REPEAT, amendments=[])


judge_follow = G3.judge_follow
judge_sim_auto = G3.judge_sim_auto
judge_browser = G4.judge_browser
foreign_sessions = G5.foreign_sessions


# --- host sampler -------------------------------------------------------------------------------------------------------
def sampler_rows(rows):
    """The 's' samples of a host_proc_sampler JSONL (dicts with wall, gap_s, total_pct), in time order."""
    return sorted((r for r in rows if r.get('kind') == 's' and r.get('total_pct') is not None), key=lambda r: r['wall'])


def quiet_mean(samples, t_end, window_s=QUIET_WINDOW_S):
    """Mean Windows total CPU of the samples in (t_end - window_s, t_end]; None when fewer than half the window."""
    w = [s['total_pct'] for s in samples if t_end - window_s < s['wall'] <= t_end]
    if len(w) < max(1, int(window_s / SAMPLER_PERIOD_S / 2)):
        return None
    return sum(w) / len(w)


def host_fault(samples, event_wall, window_s=HOST_FAULT_WINDOW_S, cpu_pct=HOST_FAULT_CPU_PCT, late_s=HOST_FAULT_LATE_S,
               period_s=SAMPLER_PERIOD_S):
    """The extended host-fault test of a failure event (Windows wall time): the samples taken in
    [event_wall - window_s, event_wall]. A sample's lateness is its interval minus the period. Returns a dict with
    void (bool), the reason(s), the largest CPU and lateness and the number of samples; void is False without
    samples (no evidence)."""
    if event_wall is None:
        return dict(void=False, reasons=['no failure event'], samples=0, cpu_max=None, late_max=None)
    w = [s for s in samples if event_wall - window_s <= s['wall'] <= event_wall]
    cpu_max = max((s['total_pct'] for s in w), default=None)
    late_max = max((s['gap_s'] - period_s for s in w), default=None)
    reasons = []
    if cpu_max is not None and cpu_max >= cpu_pct:
        reasons.append('cpu %.1f %% >= %.0f %%' % (cpu_max, cpu_pct))
    if late_max is not None and late_max >= late_s - 1e-9:
        reasons.append('sampler late %.3f s >= %.1f s' % (late_max, late_s))
    return dict(void=bool(reasons), reasons=reasons, samples=len(w), cpu_max=cpu_max,
                late_max=None if late_max is None else round(late_max, 3))


# --- failure events -----------------------------------------------------------------------------------------------------
def failure_event(ticks, run_id, end_reason):
    """A and F: the first tick of run run_id after its first ACTIVE tick whose state is not ACTIVE, when the run did not
    end at the row end (end_reason != 'row_end'). ticks: executor tick records (k 'tick'; diag ww / wm: wall and
    monotonic time). Returns dict(t, wall, mono, state, reason) or None (row end, or the run never drove)."""
    if end_reason == 'row_end':
        return None
    active = False
    for r in ticks:
        if r.get('k') != 'tick' or r.get('run') != run_id:
            continue
        if r.get('state') == 'ACTIVE':
            active = True
            continue
        if active:
            diag = r.get('diag') or {}
            return dict(t=r.get('t'), wall=diag.get('ww'), mono=diag.get('wm'), state=r.get('state'),
                        reason=r.get('reason'))
    return None


def browser_failure_event(result):
    """B: the wall time of the first failed check that belongs to a registered item (None when none failed or the
    check has no wall time)."""
    names = {n for item in B_ITEMS for n in G3.BROWSER_ITEMS[item]}
    for c in result.get('checks') or []:
        if c.get('name') in names and not c.get('ok'):
            return c.get('wall')
    return None


def sim_stall_cause(event_mono, emulator_stalls, gap_s=SIM_STALL_GAP_S, before_s=SIM_STALL_BEFORE_S):
    """Part A cause rule (stage 6): 'sim_delivery' when an ESP emulator /clock stall above gap_s (wall) ended within
    before_s before the failure event (monotonic), else 'executor_or_controller'. emulator_stalls: [(end mono, gap s)]."""
    if event_mono is None:
        return 'executor_or_controller'
    for end, gap in emulator_stalls:
        if gap > gap_s and 0. <= event_mono - end <= before_s:
            return 'sim_delivery'
    return 'executor_or_controller'


# --- slots --------------------------------------------------------------------------------------------------------------
def slot_verdict(attempts):
    """One slot from its attempts in order: each 'pass', 'fail', 'void' (host fault) or 'invalid' (RTF or start error,
    the stage 3 rule). The first attempt that is not void / invalid decides; 'not_judged' when every attempt was void or
    invalid. More than 1 + VOID_RERUNS attempts is a usage error."""
    if len(attempts) > 1 + VOID_RERUNS:
        raise ValueError('at most %d attempts per slot' % (1 + VOID_RERUNS))
    for a in attempts:
        if a in ('pass', 'fail'):
            return a
        if a not in ('void', 'invalid'):
            raise ValueError('unknown attempt verdict %r' % a)
    return 'not_judged'


def judge_part_a(slots):
    """Part A: slots = {name: [attempt verdicts]}; passes when every judged slot passes and at least three of four are
    judged."""
    verdicts = {name: slot_verdict(a) for name, a in slots.items()}
    judged = [v for v in verdicts.values() if v != 'not_judged']
    return dict(slots=verdicts, judged=len(judged), passed=len(judged) >= 3 and all(v == 'pass' for v in judged))


def judge_part_f(slots):
    """Part F: three slots, each [attempt verdicts] ('pass' = run_end row_end, 'fail', 'void'); passes when all three
    are judged and pass; 'not_judged' when a slot stays void."""
    verdicts = [slot_verdict(a) for a in slots]
    if len(verdicts) != G3.SIM_AUTO_RUNS:
        raise ValueError('part F has %d slots' % G3.SIM_AUTO_RUNS)
    state = 'not_judged' if 'not_judged' in verdicts else ('pass' if all(v == 'pass' for v in verdicts) else 'fail')
    return dict(slots=verdicts, verdict=state, passed=state == 'pass')
