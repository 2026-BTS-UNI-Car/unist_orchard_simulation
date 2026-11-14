"""Safety script catalogue (design 8.3 P4 'safety scripts'; SIM_ACCEPTANCE 2 = A17, A19, A22).

Every script is an episode spec of tools/row_executor/sim_in_loop_steer.py (world, scripted controller, rig faults,
operator / second-client actions) plus its stated expected outcome ('expect'); acceptance.py runs it and the checks
compute the outcome from the episode's artefacts (measure.py). Times are SIM seconds from the rig start: the operator
emulation starts the run at 0, arms at 0.5 s and presses the hold at 1.2 s; the drive scripts reach a steady speed by
about 3 s, so faults and actions come at FAULT_AT_S.

expect keys (all optional):
  end_reason      the executor's end reason (a string or a tuple of allowed ones); None = the run must not end
  pause_reason    the executor's pause reason (deadman scripts)
  mcu_fault       the MCU F code (A19: the judgement tick within +-1 tick of the spec 6.7 recomputation)
  mcu_watchdog    True: a W line (A17 watchdog item)
  mcu_ttl         True: an F TTL line (A17 TTL item)
  host_latch      host latch name(s) that must appear
  arm_refused     the code of a refused arm (precheck or bridge); then no ARM line may reach the MCU
  stop_at         the time of a STOP request (A17 STOP send / parse / confirmation items)
  stop_unconfirmed True: the resend grid of amendment 7 and the latch at t0 + 400 ms
  steer_off       True: steering output 0 at most 1 tick after the fault judgement (A17)
  hold_check      True: the steering hold error after the stop stays within max(1 deg, 2 x deadband) (A17): from the
                  STOP parse tick, else the W tick (link watchdog, wdmode hold), else the F CRC tick
  no_restart      True (default for stopping scripts): from the first tick with both drive outputs 0 after the stop,
                  no output grows (a release ramp only shrinks), the drive is 0 1.5 s after the stop and no ARM line
                  reaches the MCU (A17)
  operator_rearm_at  the time of an explicit operator re-arm of the script (its ARM line is not an automatic one)
  bridge_restart  True: the rig killed the bridge process and started a new one (fault bridge_restart): the requests
                  failed meanwhile, the new process holds no lease and sends the session STOP, no C line
  boot_after_s    the MCU boots again after this time (a reset when the port is opened again)
  a22             True: no commanded steering move beyond the hold tolerance in the 3 s after the terminal moment, no
                  automatic centring (A22)
  same_as         another script whose trace and serial log must be identical (system clock change)
  client2         expected replies of the second client: [(op, ok, error)]
  late_drive_error  the expected refusal of the late drive (stop_barrier)
  lateral         True: the A17 lateral deviation during a link loss (analytic arc, +-10 % or +-0.01 m)
  rate_counts     the false-detection count group of a normal script ('normal' scripts)
  first_rule      True with a tuple mcu_fault: the found code must be the one the spec 6.7 recomputation trips first
                  (the earliest expected tick of the candidate codes, +-1 tick)
  link_lost_at    one_press: the time the operator screen's heartbeats stop; the drive command must be 0 at most
                  link_timeout_s (the executor's operator_timeout_s, default 1.0) + one tick later (A17 amendment 31/60)
  other_screen    one_press: the expected replies to a second screen [(op, ok, error)] (heartbeat and go)
  go_refused      one_press: the allowed error(s) of the first refused go of the script
  lease_lost      True: the bridge takes the lease while the armed run waits for its go; the executor keeps the run open
                  (event lease_lost, no run_end), the run becomes ACTIVE only after the operator's re-arm
                  (operator_rearm_at, event lease_lost_rearm) (amendment 34)
  stop_phase      True: a STOP phase script (the STOP request off the bridge grid or the status lines shifted;
                  audit AUD3A-09); its STOP items are judged as stop_at and listed in the A17 STOP phase table
Every value of the scripts is a placeholder of the SIM (자리값 기반 SIM 결과).
"""
from __future__ import annotations

import copy

FAULT_AT_S = 5.
POST_END_S = 3.2                                  # A22: 3 s observation after the terminal moment
ROW_POSTS = [[round(-6. + 1.5 * k, 3), side * 1.75, .12] for k in range(30) for side in (1, -1)]   # 60 <= 64 markers
W0_ROWS = {'kind': 'W0', 'markers': ROW_POSTS}    # two lines of posts 3.5 m apart (an orchard-like test floor)
W2_DEV = {'kind': 'W2', 'record': '042301', 'corridor_id': 'c03', 'direction': 'fwd'}   # development corridor
HOLD_40 = {'kind': 'straight_hold', 'drive_pct': 40, 'distance_m': None}                 # drive until stopped


def _seg(duration_s, throttle, steer_deg, label, raw=False):
    return dict(duration_s=float(duration_s), throttle=int(throttle), steer_deg=steer_deg, label=label, raw=raw)


def _drive_then(seconds, throttle=40, steer_deg=0., lead_s=2.):
    return [_seg(lead_s, throttle, 0., 'lead'), _seg(seconds, throttle, steer_deg, 'drive')]


def episode(name, controller=None, world=None, faults=(), actions=(), rig=None, config=None, duration_s=10.,
            post_end_s=POST_END_S, auto_repress=False, seed=0, start_pose=(0., 0., 0.), capture_inputs=0,
            stream_key=None, start_steer_deg=0., start_mode=None):
    """An episode spec. start_mode None = the executor's default (front_steer: one_press, design v2 V2-2); the
    hold-to-run scripts (deadman_release, hold_lost) name 'hold_to_run'."""
    config = dict(config or {})
    if start_mode is not None:
        config.setdefault('start_mode', start_mode)
    return dict(name=name, seed=seed, world=copy.deepcopy(world or W0_ROWS), start_pose=list(start_pose),
                start_steer_deg=float(start_steer_deg),
                controller=copy.deepcopy(controller or HOLD_40), faults=[dict(f) for f in faults],
                actions=[dict(a) for a in actions], rig=dict(rig or {}), config=config,
                duration_s=float(duration_s), post_end_s=post_end_s, auto_repress=auto_repress,
                capture_inputs=capture_inputs, stream_key=stream_key)


def fault(kind, at_s=FAULT_AT_S, **params):
    return dict(kind=kind, at_s=float(at_s), params=params)


def action(op, at_s=FAULT_AT_S, **params):
    return dict(params, op=op, at_s=float(at_s))


def segments(segs, end='stop'):
    return dict(kind='segments', segments=segs, end=end)


SCRIPTS = []


def script(name, group, purpose, spec, expect):
    SCRIPTS.append(dict(name=name, group=group, purpose=purpose, spec=spec, expect=expect))


# --- safety scripts (design 8.3 P4 list) ---------------------------------------------------------------------------------
script('mid_row_stop_w2', 'safety', 'operator STOP in the middle of a development corridor: drive brake, steering hold',
       episode('mid_row_stop_w2', world=W2_DEV, start_pose=(1., 0., 0.), actions=[action('stop', 6.)], duration_s=12.),
       dict(end_reason='operator_stop', stop_at=6., hold_check=True, no_restart=True, a22=True))
script('mid_row_stop_w0', 'safety', 'operator STOP while driving on the W0 test floor (STOP timing, hold, no restart)',
       episode('mid_row_stop_w0', actions=[action('stop', 6.)], duration_s=12.),
       dict(end_reason='operator_stop', stop_at=6., hold_check=True, no_restart=True, a22=True))
# STOP phases (audit AUD3A-09): the executor ticks every 50 ms, so a STOP at 6.05 s waits 10 ms for the 20 ms bridge
# step (6.06 s); the bridge on the odd 10 ms grid moves the 6.0 s STOP off the grid; the boot delay moves the MCU status
# line phase (the STOP confirmation). The rig runs host code on the 10 ms plant grid: a request between two plant steps
# is not representable (T1_RIG_KO.md 2).
script('mid_row_stop_off_grid', 'safety', 'mid_row_stop_w0 with the STOP at 6.05 s (between two bridge steps) and the '
       'status lines 5 ms later (boot delay 75)',
       episode('mid_row_stop_off_grid', actions=[action('stop', 6.05)], rig={'boot_delay_ticks': 75}, duration_s=12.),
       dict(end_reason='operator_stop', stop_at=6.05, stop_phase=True, hold_check=True, no_restart=True))
script('mid_row_stop_bridge_phase', 'safety', 'mid_row_stop_w0 with the bridge on the odd 10 ms grid (the 6.0 s STOP '
       'waits for 6.01 s) and the status lines 10 ms earlier (boot delay 60)',
       episode('mid_row_stop_bridge_phase', actions=[action('stop', 6.)],
               rig={'bridge_phase_ms': 10, 'boot_delay_ticks': 60}, duration_s=12.),
       dict(end_reason='operator_stop', stop_at=6., stop_phase=True, hold_check=True, no_restart=True))
script('deadman_release', 'safety', 'the operator lets the hold go while driving: pause, zero throttle at once, steering '
       'held, no restart (no new press)',
       episode('deadman_release', controller=segments(_drive_then(8., 40, 4.)), actions=[action('release', 5.)],
               duration_s=10., start_mode='hold_to_run'),
       dict(end_reason=None, pause_reason='hold_released', no_restart=True, a22=True, pause_at=5.))
script('hold_lost', 'safety', 'the operator page stops sending heartbeats (page closed): the hold times out',
       episode('hold_lost', actions=[action('hold_lost', 5.)], duration_s=10., start_mode='hold_to_run'),
       dict(end_reason=None, pause_reason='hold_released', no_restart=True, a22=True, pause_at=5.))
# one_press (design v2 V2-2, decision R32, A17 amendment 31; the front_steer default start mode): one go, the screen's
# heartbeats keep the run going; the link timeout (operator_timeout_s 1.0 s since decision R60) pauses it, only a new go
# resumes
script('operator_link_lost', 'safety', 'one_press: the operator screen stops its heartbeats while driving (screen hidden): '
       'pause operator_link_lost, zero drive within the timeout + 1 tick, steering held, no restart',
       episode('operator_link_lost', controller=segments(_drive_then(8., 40, 4.)),
               actions=[action('screen_hidden', 5.)], duration_s=10.),
       dict(end_reason=None, pause_reason='operator_link_lost', link_lost_at=5., link_timeout_s=1., no_restart=True,
            a22=True, pause_at=6.))
script('operator_link_back_no_restart', 'safety', 'one_press: the heartbeats stop and come back 1.3 s later: the run '
       'stays paused (no restart without a new go)',
       episode('operator_link_back_no_restart', actions=[action('screen_hidden', 5.), action('screen_visible', 6.3)],
               duration_s=10.),
       dict(end_reason=None, pause_reason='operator_link_lost', link_lost_at=5., link_timeout_s=1., no_restart=True,
            pause_at=6.))
script('other_screen_refused', 'safety', 'one_press: a second screen sends a heartbeat and a go while the first one drives: '
       'both refused (session_conflict), the run goes on',
       episode('other_screen_refused', actions=[action('other_screen', 5.)], duration_s=8.),
       dict(end_reason=None, other_screen=[('heartbeat', False, 'session_conflict'), ('go', False, 'session_conflict')]))
script('one_press_stop', 'safety', 'one_press: operator STOP while driving ends the run and the go; a go after it is '
       'refused (the latch needs an acknowledgement, a new Start and Arm)',
       episode('one_press_stop', actions=[action('stop', 6.), action('go', 7.)], duration_s=12.),
       dict(end_reason='operator_stop', stop_at=6., hold_check=True, no_restart=True, a22=True,
            go_refused=('latched', 'no_run')))
# amendment 34 (2026-11-13, SIM dashboard sessions s2..s4): the bridge takes the lease of an armed run that waits for its
# go (a status-line gap: steering_status_stale while nothing moves). The run stays open (run_state ready, go blockers
# lease_lost + arm_first); only the operator's Arm press acknowledges that latch and re-arms, then one go starts it.
script('armed_wait_lease_lost', 'safety', 'one_press: status lines lost for 0.4 s while the armed run waits for its go: '
       'the bridge latches steering_status_stale and takes the lease; the run stays open and does not move; the '
       'operator arms again at 3 s and one go starts the run',
       episode('armed_wait_lease_lost', controller=dict(HOLD_40, distance_m=3.),
               faults=[fault('status_loss', at_s=.85, duration_s=.4)], actions=[action('arm', 3.)], duration_s=16.),
       dict(end_reason='script_distance', host_latch='steering_status_stale', lease_lost=True, operator_rearm_at=3.))
script('c_line_loss_ttl', 'safety', 'C lines stop reaching the MCU (link to the MCU lost for 0.4 s, under the watchdog): '
       'MCU C TTL brake, steering kept; the executor ends the run (no restart by the next C)',
       episode('c_line_loss_ttl', faults=[fault('link_loss', duration_s=.4, direction='to_mcu')], duration_s=10.),
       dict(end_reason='link_ttl_expired', mcu_ttl=True, host_latch='mcu_ttl_expired', no_restart=True, a22=True))
# Review RS-01: the MCU status-line phase and the host loop phase are unrelated on the vehicle. With these phases the C
# line written right after the link returns reaches the MCU before the next status line, clears the TTL bit and would
# restart the drive 120 ms after the TTL brake; only the F TTL line (counted by the host) shows the event. The executor
# ends the run on that count in the same tick, so the STOP arrives before the restart could happen.
script('c_line_loss_ttl_status_phase', 'safety', 'c_line_loss_ttl with the MCU status lines 10 ms earlier (boot delay '
       '60 ms) and the link lost 4.97 .. 5.29 s: no status line carries the TTL bit; the executor ends the run on the '
       'host F TTL count, no restart',
       episode('c_line_loss_ttl_status_phase', faults=[fault('link_loss', at_s=4.97, duration_s=.32, direction='to_mcu')],
               rig={'boot_delay_ticks': 60}, duration_s=10.),
       dict(end_reason='link_ttl_expired', mcu_ttl=True, host_latch='mcu_ttl_expired', no_restart=True, a22=True))
script('c_line_loss_ttl_catalogue_phase60', 'safety', 'c_line_loss_ttl (5.0 s, 0.4 s) with the status lines 10 ms '
       'earlier (boot delay 60 ms): the TTL bit appears late; no drive restart before the STOP',
       episode('c_line_loss_ttl_catalogue_phase60', faults=[fault('link_loss', duration_s=.4, direction='to_mcu')],
               rig={'boot_delay_ticks': 60}, duration_s=10.),
       dict(end_reason='link_ttl_expired', mcu_ttl=True, host_latch='mcu_ttl_expired', no_restart=True, a22=True))
script('c_line_loss_ttl_bridge_phase', 'safety', 'c_line_loss_ttl with the host bridge loop on the odd 10 ms grid '
       '(bridge_phase_ms 10)',
       episode('c_line_loss_ttl_bridge_phase', faults=[fault('link_loss', duration_s=.4, direction='to_mcu')],
               rig={'bridge_phase_ms': 10}, duration_s=10.),
       dict(end_reason='link_ttl_expired', mcu_ttl=True, host_latch='mcu_ttl_expired', no_restart=True, a22=True))
# DECISIONS R18 decision 1 (audit AUD3A-04): the executor tick is late right after the C TTL (rig fault executor_stall
# from the time the F TTL line reaches the host). Before the host latch mcu_ttl_expired the bridge's automatic C resends
# drove again 120 ms after the TTL brake while the executor stalled; now the bridge stops on the F TTL line itself. The
# executor ends on the latch, or on its own stale steering status after the stall (both stop reasons are valid).
TTL_STALL_ENDS = ('link_ttl_expired', 'steering_feedback_stale', 'bridge_error')
script('c_line_loss_ttl_executor_stall', 'safety', 'c_line_loss_ttl (5.0 s, 0.32 s) and the executor loop stalled for '
       '0.3 s from the F TTL line: the host latches mcu_ttl_expired and stops; no restart',
       episode('c_line_loss_ttl_executor_stall',
               faults=[fault('link_loss', duration_s=.32, direction='to_mcu'),
                       fault('executor_stall', duration_s=.3, after='f_ttl')], duration_s=10.),
       dict(end_reason=TTL_STALL_ENDS, mcu_ttl=True, host_latch='mcu_ttl_expired', no_restart=True, a22=True))
script('c_line_loss_ttl_executor_stall_phase', 'safety', 'c_line_loss_ttl_executor_stall with the status lines 10 ms '
       'earlier (boot delay 60), the bridge on the odd 10 ms grid, the loss 4.97 .. 5.29 s and a 0.5 s stall that starts '
       '20 ms after the F TTL line',
       episode('c_line_loss_ttl_executor_stall_phase',
               faults=[fault('link_loss', at_s=4.97, duration_s=.32, direction='to_mcu'),
                       fault('executor_stall', at_s=4.97, duration_s=.5, after='f_ttl', delay_s=.02)],
               rig={'boot_delay_ticks': 60, 'bridge_phase_ms': 10}, duration_s=10.),
       dict(end_reason=TTL_STALL_ENDS, mcu_ttl=True, host_latch='mcu_ttl_expired', no_restart=True, a22=True))
script('link_loss_watchdog_brake', 'safety', 'link watchdog 200 ms (CFG wd 200, under the C TTL of 300 ms) and the link '
       'to the MCU lost for 0.5 s while driving: the watchdog itself brakes the moving vehicle (W line, drive 0 at the '
       'W tick, DISARMED), steering hold (review RS-03)',
       episode('link_loss_watchdog_brake', faults=[fault('link_loss', duration_s=.5, direction='to_mcu')],
               rig={'record_overrides': {'wd': 200}}, duration_s=10.),
       dict(end_reason=('link_watchdog', 'mcu_disarmed', 'bridge_error', 'steering_feedback_stale'), mcu_watchdog=True,
            hold_check=True, no_restart=True))
script('executor_stall', 'safety', 'the executor loop stops for 1.5 s (no drive): host input deadline, MCU TTL; the run '
       'ends when the executor comes back',
       episode('executor_stall', actions=[action('executor_stall', 5., duration_s=1.5)], duration_s=10.),
       dict(end_reason=('bridge_error', 'link_ttl_expired'), mcu_ttl=True, no_restart=True))
script('bridge_stall_ttl', 'host', 'the bridge loop stops for 0.335 s while driving (the TTL ends, the watchdog does '
       'not): the bridge reads the F TTL line when it runs again, latches mcu_ttl_expired and stops before any C line',
       episode('bridge_stall_ttl', faults=[fault('bridge_stall', duration_s=.335)], duration_s=10.),
       dict(end_reason=('link_ttl_expired', 'bridge_error'), mcu_ttl=True, host_latch='mcu_ttl_expired',
            no_restart=True))
script('bridge_stall_watchdog', 'safety', 'the bridge loop stops for 1.5 s: MCU C TTL then link watchdog, DISARMED',
       episode('bridge_stall_watchdog', faults=[fault('bridge_stall', duration_s=1.5)], duration_s=10.),
       dict(end_reason=('link_watchdog', 'link_ttl_expired', 'bridge_error', 'mcu_disarmed'), mcu_ttl=True,
            mcu_watchdog=True, hold_check=True, no_restart=True))
for _kind, _code in (('pot_open', 'POT_OPEN'), ('pot_short', 'POT_SHORT')):
    script(_kind, 'fault', 'potentiometer %s while driving: latched stop, steering output 0' % _kind,
           episode(_kind, faults=[fault(_kind)], rig={'report_adc': True}, duration_s=10.),
           dict(end_reason='steering_feedback_invalid', mcu_fault=_code, steer_off=True, host_latch='mcu_disarmed',
                no_restart=True, a22=True))
script('pot_jump', 'fault', 'potentiometer jump of 60 counts for 0.2 s while driving',
       episode('pot_jump', faults=[fault('pot_jump', counts=60, duration_s=.2)], rig={'report_adc': True}),
       dict(end_reason='steering_feedback_invalid', mcu_fault='POT_JUMP', steer_off=True, no_restart=True, a22=True))
script('pot_noise', 'fault', 'potentiometer noise x10 while driving',
       episode('pot_noise', faults=[fault('pot_noise', scale=10.)], rig={'report_adc': True}),
       dict(end_reason='steering_feedback_invalid', mcu_fault='POT_NOISE', steer_off=True, no_restart=True))
script('polarity_reversed', 'fault', 'motor wiring reversed: the first large steering step moves the wrong way',
       episode('polarity_reversed', controller=segments(_drive_then(4., 40, 10.)),
               faults=[fault('polarity_reversed', at_s=0.)]),
       dict(end_reason='steering_polarity_fault', mcu_fault='POLARITY', steer_off=True, no_restart=True))
script('driver_stuck', 'fault', 'steering driver stuck on at 60 % while driving straight',
       episode('driver_stuck', faults=[fault('driver_stuck_on', pwm=.6)]),
       dict(end_reason=('steering_driver_fault', 'steering_polarity_fault'), mcu_fault=('DRV_STUCK', 'POLARITY'),
            steer_off=True, no_restart=True))
script('driver_stuck_isolated', 'fault', 'steering driver stuck on at -60 % (to the right) while driving straight, MCU '
       'POLARITY rule widened (polp 1000): the wheels keep moving while the servo output is 0 or pushes back, so '
       'DRV_STUCK trips (80 permille in 300 ms). The DRV_STUCK rule only counts when the position leaves the brake '
       'reference by more than db before the servo output turns non-zero (same db as the servo deadband), so a stuck '
       'driver that pushes toward the side where cmd lies can be missed and TRACK trips later (see open issues)',
       episode('driver_stuck_isolated', faults=[fault('driver_stuck_on', pwm=-.6)],
               rig={'record_overrides': {'polp': 1000}}),
       dict(end_reason='steering_driver_fault', mcu_fault='DRV_STUCK', steer_off=True, no_restart=True))
# Audit AUD3A-18: faults inside the potentiometer range and a dead driver (no POT_* range rule can see them). The
# expectation comes from the spec 6.7 recomputation of the candidate rules (first_rule): the code found must be the one
# whose rule trips first. A 6 deg step while driving gives a large servo output (gain 15 %/deg: about 230 >= stp 153).
script('pot_stuck_step', 'fault', 'potentiometer reading frozen at 3.0 s, then a 6 deg step while driving: the servo '
       'pushes, the reading stays (STALL, or TRACK under the STALL output level)',
       episode('pot_stuck_step', controller=segments(_drive_then(4., 40, 6.)), faults=[fault('pot_stuck', at_s=3.)],
               rig={'report_adc': True}),
       dict(end_reason=('steering_stalled', 'steering_not_following'), mcu_fault=('STALL', 'TRACK'), first_rule=True,
            steer_off=True, no_restart=True))
script('pot_reversed_step', 'fault', 'potentiometer sign reversed at 3.0 s (wheels near straight: no jump), then a 6 deg '
       'step while driving: the reading moves against the servo output (POLARITY)',
       episode('pot_reversed_step', controller=segments(_drive_then(4., 40, 6.)),
               faults=[fault('pot_reversed', at_s=3.)], rig={'report_adc': True}),
       dict(end_reason=('steering_polarity_fault', 'steering_stalled', 'steering_not_following'),
            mcu_fault=('POLARITY', 'STALL', 'TRACK'), first_rule=True, steer_off=True, no_restart=True))
script('driver_dead_step', 'fault', 'steering driver output dead from 3.0 s, then a 6 deg step while driving: large servo '
       'output, no motion (STALL)',
       episode('driver_dead_step', controller=segments(_drive_then(4., 40, 6.)), faults=[fault('driver_dead', at_s=3.)],
               rig={'report_adc': True}),
       dict(end_reason=('steering_stalled', 'steering_not_following'), mcu_fault=('STALL', 'TRACK'), first_rule=True,
            steer_off=True, no_restart=True))
script('stall_jam', 'fault', 'steering jammed, then a 15 deg step while driving (large output, no motion)',
       episode('stall_jam', controller=segments(_drive_then(4., 40, 15.)), faults=[fault('stall', at_s=2.5)]),
       dict(end_reason='steering_stalled', mcu_fault='STALL', steer_off=True, no_restart=True))
script('track_jam_small', 'fault', 'steering jammed, then a 2.84 deg step: the error stays above trk (120) and the servo '
       'output under the STALL level (153), so TRACK trips (the step is sized to the jam position of this seed; MCU '
       'stm 60 so the +-1 raw count reading noise of the jammed pot is not counted as progress)',
       episode('track_jam_small', controller=segments(_drive_then(4., 40, 2.84)), faults=[fault('stall', at_s=2.5)],
               rig={'record_overrides': {'stm': 60}}),
       dict(end_reason=('steering_not_following', 'steering_stalled'), mcu_fault='TRACK', steer_off=True,
            no_restart=True))
script('thermal', 'fault', 'overloaded (slow) steering actuator with continuous sweeps; host follow rule widened so the '
       'MCU THERMAL proxy trips (as the T1 rig test)',
       episode('thermal', controller=segments([_seg(2., 40, 0., 'lead'),
                                               dict(duration_s=24., throttle=40, label='sweeps',
                                                    steer_deg=dict(fn='triangle', amplitude_deg=20., period_s=2.))]),
               faults=[fault('thermal', at_s=0.)],
               rig={'host_overrides': {'host_faults': {'track_duration_s': 60.}}}, duration_s=30.),
       dict(end_reason='steering_driver_fault', mcu_fault='THERMAL', steer_off=True, no_restart=True))
script('cal_id_mismatch_executor', 'safety', 'the executor expects calibration set 2, the MCU reports 1: arm refused',
       episode('cal_id_mismatch_executor', config={'steer_cal_id': 2}, duration_s=3.),
       dict(arm_refused='calibration_mismatch'))
script('cal_id_mismatch_host', 'host', 'the host profile expects calibration set 2, the MCU has 1: the bridge refuses',
       episode('cal_id_mismatch_host', rig={'host_cal_id': 2}, config={'steer_cal_id': 1}, duration_s=3.),
       dict(arm_refused=('calibration_mismatch', 'steering_fault'), host_latch='calibration_mismatch'))
script('nocal_boot', 'fault', 'blank EEPROM: NOCAL at boot, arm refused (steering uncalibrated)',
       episode('nocal_boot', rig={'eeprom_blank': True}, duration_s=3.),
       dict(arm_refused='steering_uncalibrated', mcu_fault='NOCAL'))
script('stale_steering_status', 'safety', 'status lines lost for 0.4 s: host steering_status_stale, stop',
       episode('stale_steering_status', faults=[fault('status_loss', duration_s=.4)]),
       dict(end_reason='steering_feedback_stale', host_latch='steering_status_stale', no_restart=True, a22=True))
script('stop_unconfirmed', 'safety', 'operator STOP while the MCU loop is stalled for 0.8 s: bounded resends, latch',
       episode('stop_unconfirmed', actions=[action('stop', 6.)], faults=[fault('mcu_stall', at_s=6., duration_s=.8)],
               duration_s=10.),
       dict(end_reason='operator_stop', stop_at=6., stop_unconfirmed=True, host_latch='stop_unconfirmed',
            no_restart=True))
script('mcu_reset', 'mandatory', 'MCU reset while driving: coast, new boot id, host mcu_reset_detected, no restart',
       episode('mcu_reset', faults=[fault('mcu_reset', reason=4)]),
       dict(end_reason=('mcu_reset', 'bridge_error', 'mcu_disarmed'), host_latch='mcu_reset_detected', mcu_fault='BOOT',
            no_restart=True))
script('power_cut', 'safety', 'power cut of the MCU for 0.5 s while driving: no status lines (the host stale rule '
       'stops first), then the new boot (mcu_reset_detected)',
       episode('power_cut', faults=[fault('power_cut', off_s=.5)]),
       dict(end_reason=('steering_feedback_stale', 'mcu_reset', 'bridge_error', 'mcu_disarmed'),
            host_latch='mcu_reset_detected', no_restart=True))
script('softlimit', 'fault', 'standstill with the wheels starting at 23.3 deg (inside the 409 mrad soft limit) and a '
       '25 deg request clamped to the limit (MCU position 923 .. 934 permille, raw 598 .. 600), then the potentiometer '
       'reading set to raw 601 (adc_set: about 945 permille, beyond the MCU soft limit 935 and inside the servo deadband '
       'of the 934 target): the SOFTLIMIT warning bit from the position rule, no stop',
       episode('softlimit', controller=dict(kind='segments', end='hold_end', limit_margin=1.,
                                            segments=[_seg(1., 0, 'hold', 'settle'), _seg(6.5, 0, 25., 'to_limit')]),
               faults=[fault('adc_set', at_s=6.5, raw=601)], rig={'report_adc': True}, duration_s=12.,
               start_steer_deg=23.3),
       dict(end_reason='script_complete', mcu_fault='SOFTLIMIT', no_restart=True))

# --- 09-19 mandatory failure tests ---------------------------------------------------------------------------------------
script('two_screens_and_cli', 'mandatory', 'a second screen tries to arm and to drive while the executor drives; then the '
       'CLI sends STOP: everything stops, the executor latches without stopping a newer lease',
       episode('two_screens_and_cli', actions=[action('client2_arm', 4.), action('client2_drive', 4.5),
                                               action('client2_stop', 6.)], duration_s=10.),
       dict(end_reason='operator_stop', client2=[('client2_arm', False, 'control_in_use'),
                                               ('client2_drive', False, 'control_not_owned'),
                                               ('client2_stop', True, None)], stop_at=6., no_restart=True, a22=True))
script('link_loss_while_held', 'mandatory', 'serial link lost in both directions for 1.0 s while driving with the hold '
       'pressed: MCU TTL and watchdog, no restart when the link returns',
       episode('link_loss_while_held', faults=[fault('link_loss', duration_s=1., direction='both')], duration_s=10.),
       dict(end_reason=('link_watchdog', 'steering_feedback_stale', 'mcu_disarmed', 'link_ttl_expired', 'bridge_error'),
            mcu_watchdog=True, mcu_ttl=True, hold_check=True, no_restart=True))
script('link_loss_lateral_5deg', 'mandatory', 'link lost at 0.4 m/s with the steering held at 5 deg: the sideways shift '
       'until rest against the analytic arc (A17 lateral deviation)',
       episode('link_loss_lateral_5deg', controller=segments(_drive_then(8., 48, 5., lead_s=3.)),
               faults=[fault('link_loss', at_s=6., duration_s=2., direction='both')], duration_s=11.),
       dict(end_reason=('link_watchdog', 'steering_feedback_stale', 'mcu_disarmed', 'link_ttl_expired', 'bridge_error'),
            lateral=True, loss_at=6., no_restart=True))
script('bridge_forced_exit', 'mandatory', 'the bridge process is killed while driving and a new one starts 2 s later '
       '(rig fault bridge_restart: the executor requests get socket errors, the MCU C TTL and link watchdog stop the '
       'vehicle; the new process opens the port, sends the session STOP and holds no lease: no restart). The dashboard '
       'forced exit is hold_lost (the operator page of the executor stops)',
       episode('bridge_forced_exit', faults=[fault('bridge_restart', down_s=2.)], duration_s=10., post_end_s=None),
       dict(end_reason=('bridge_error', 'bridge_latched'), mcu_ttl=True, mcu_watchdog=True, hold_check=True,
            no_restart=True, bridge_restart=True))
script('bridge_forced_exit_port_reset', 'mandatory', 'bridge_forced_exit where opening the port resets the NodeMCU '
       '(DTR / RTS, spec 10.7): the new process sees a new boot; no restart',
       episode('bridge_forced_exit_port_reset', faults=[fault('bridge_restart', down_s=2., port_open_reset=True)],
               duration_s=10., post_end_s=None),
       dict(end_reason=('bridge_error', 'bridge_latched'), mcu_ttl=True, mcu_watchdog=True, no_restart=True,
            bridge_restart=True, boot_after_s=FAULT_AT_S + 2.))
script('usb_unplug_replug', 'mandatory', 'USB unplugged for 1.0 s and plugged back: hardware_fault, reopen, no restart',
       episode('usb_unplug_replug', faults=[fault('usb_unplug', duration_s=1.)], duration_s=10.),
       dict(end_reason=('link_hardware_fault', 'bridge_error', 'link_watchdog'), host_latch='hardware_fault',
            mcu_watchdog=True, no_restart=True))
script('usb_unplug_replug_reset', 'mandatory', 'usb_unplug_replug where opening the port again resets the NodeMCU '
       '(DTR / RTS, spec 10.7; review RS-13): the pins float during the reset, a new boot id, no re-arm, no restart',
       episode('usb_unplug_replug_reset', faults=[fault('usb_unplug', duration_s=1.)], rig={'port_open_reset': True},
               duration_s=10.),
       dict(end_reason=('link_hardware_fault', 'bridge_error', 'link_watchdog'), host_latch='hardware_fault',
            mcu_watchdog=True, no_restart=True, boot_after_s=FAULT_AT_S + 1.))
script('half_frame', 'mandatory', 'half a C frame without LF from another writer: MCU refuses the merged line',
       episode('half_frame', faults=[fault('half_frame')], duration_s=9.),
       dict(end_reason=('link_command_rejected', 'bridge_error'), host_latch='c_rejected', no_restart=True))
script('crc_error_single', 'mandatory', 'one corrupted line: N - crc, no stop, the run goes on',
       episode('crc_error_single', faults=[fault('crc_error', count=1)], duration_s=8., post_end_s=None),
       dict(end_reason=None, no_fault=True))
script('crc_error_burst', 'mandatory', 'five corrupted lines in a row: F CRC, stop',
       episode('crc_error_burst', faults=[fault('crc_error', count=5)], duration_s=9.),
       dict(end_reason=('link_bad_lines', 'bridge_error', 'mcu_disarmed'), mcu_fault='CRC', hold_check=True,
            no_restart=True))
script('range_error', 'mandatory', 'a C line with S outside the range: N q range, host c_rejected, stop',
       episode('range_error', faults=[fault('range_error', count=1)], duration_s=9.),
       dict(end_reason=('link_command_rejected', 'bridge_error'), host_latch='c_rejected', no_restart=True))
script('status_line_loss', 'mandatory', 'status lines lost for 0.4 s (same as stale_steering_status, mandatory list)',
       episode('status_line_loss', faults=[fault('status_loss', duration_s=.4)], seed=1),
       dict(end_reason='steering_feedback_stale', host_latch='steering_status_stale', no_restart=True))
script('late_drive_after_stop', 'mandatory', 'a STOP and, right after it, a drive with the executor lease: stop_barrier',
       episode('late_drive_after_stop', actions=[action('late_drive', 6.)], duration_s=10.),
       dict(end_reason='operator_stop', late_drive_error='stop_barrier', stop_at=6., no_restart=True))
script('clock_change', 'mandatory', 'the system (wall) clock jumps by +1 h while driving: deadlines stay on the monotonic '
       'clock, nothing changes',
       episode('clock_change', faults=[fault('wall_clock_jump', jump_s=3600.)], duration_s=9., post_end_s=None,
               stream_key='clock_pair'),
       dict(end_reason=None, no_fault=True, same_as='clock_reference'))
script('clock_reference', 'normal', 'the same run without the clock jump (reference of clock_change)',
       episode('clock_reference', duration_s=9., post_end_s=None, stream_key='clock_pair'),
       dict(end_reason=None, no_fault=True))

# --- host faults (A19 host part) -----------------------------------------------------------------------------------------
script('host_not_following', 'host', 'slow actuator, MCU TRACK and STALL widened (trk 2000, stm 0): the host '
       'steering_not_following latch',
       episode('host_not_following', controller=segments(_drive_then(5., 40, 14.)),
               faults=[fault('slow', at_s=2.5, rate_scale=.05)], rig={'record_overrides': {'trk': 2000, 'stm': 0}}),
       dict(end_reason='steering_not_following', host_latch='steering_not_following', no_restart=True))
script('host_ineffective', 'host', 'potentiometer drift of 2.9 deg/s while turning at 8 deg and 0.4 m/s (the servo '
       'follows the drifting reading, so the wheels straighten): the gyro does not match v tan(delta_pot) / L (host '
       'steering_ineffective from the executor motion samples)',
       episode('host_ineffective', controller=segments(_drive_then(6., 48, 8., lead_s=3.)),
               faults=[fault('pot_drift', at_s=4.5, rate_rad_per_s=.05)], duration_s=11.),
       dict(end_reason='steering_ineffective', host_latch='steering_ineffective', no_restart=True))
script('host_seq_gap', 'host', 'six single status lines lost within 0.6 s (gaps under the stale limit): status_seq_gap',
       episode('host_seq_gap', faults=[fault('status_loss', at_s=FAULT_AT_S + .1 * k, duration_s=.05)
                                       for k in range(6)]),
       dict(end_reason='steering_feedback_stale', host_latch='status_seq_gap', no_restart=True))
script('host_state_mismatch', 'host', 'another writer arms the MCU before the executor arms: host mcu_state_mismatch, '
       'the executor arm is refused',
       episode('host_state_mismatch', actions=[action('foreign_arm', .3)], duration_s=3.),
       dict(host_latch='mcu_state_mismatch', arm_refused=('steering_fault', 'bridge_state_fault', 'stop_unconfirmed')))
script('arm_unconfirmed', 'host', 'the ARM line is lost on the way to the MCU (link to the MCU down 0.48 .. 0.63 s, '
       'status lines keep coming): no ARMED status within 150 ms',
       episode('arm_unconfirmed', faults=[fault('link_loss', at_s=.48, duration_s=.15, direction='to_mcu')],
               duration_s=4.),
       dict(host_latch='arm_unconfirmed', end_reason=('arm_unconfirmed', 'bridge_error', 'bridge_arm_failed')))
# Review P4R-15: the four host latches the first catalogue left to the host unit tests.
script('host_hw_profile_mismatch', 'host', 'the host profile expects another hardware profile than the MCU banner '
       'reports (rig host_hw_profile): host hardware_profile_mismatch, the arm is refused, no ARM line',
       episode('host_hw_profile_mismatch', rig={'host_hw_profile': 'ubts_steer_other_board'}, duration_s=3.),
       dict(host_latch='hardware_profile_mismatch',
            arm_refused=('hardware_profile_mismatch', 'steering_fault', 'link_identity', 'bridge_state')))
script('host_sim_identity_mismatch', 'host', 'a vehicle host link (rig host_simulated False) meets the SIM MCU (banner '
       'sim=1): host sim_identity_mismatch, the arm is refused, no ARM line (A14 in T1)',
       episode('host_sim_identity_mismatch', rig={'host_simulated': False}, duration_s=3.),
       dict(host_latch='sim_identity_mismatch',
            arm_refused=('sim_identity_mismatch', 'esp_not_connected', 'steering_fault', 'firmware_v3_required',
                         'bridge_state')))
script('host_arm_rejected', 'host', 'potentiometer open while driving (latched stop), then the operator acknowledges, '
       'starts a new run and arms while the open circuit is still there: the MCU refuses the ARM (N - fault), host '
       'arm_rejected, no restart',
       episode('host_arm_rejected', faults=[fault('pot_open')], actions=[action('ack', 7.), action('start', 7.2)],
               rig={'report_adc': True}, duration_s=11., post_end_s=None),
       dict(mcu_fault='POT_OPEN', host_latch='arm_rejected', no_restart=True, operator_rearm_at=7.2))
script('host_internal_error', 'host', 'an injected host bridge defect (rig fault host_defect: one exception inside the '
       'core loop) while driving: host internal_error latch, STOP, the executor ends, no restart',
       episode('host_internal_error', faults=[fault('host_defect')], duration_s=10.),
       dict(end_reason=('bridge_internal_error', 'bridge_error', 'bridge_latched'), host_latch='internal_error',
            no_restart=True))

# --- normal scripts (no fault: 0 false detections; A22; the false-detection rates) -----------------------------------
script('normal_straight_w0', 'normal', 'straight hold 8 m on the W0 floor, stop, 3 s observation',
       episode('normal_straight_w0', controller=dict(HOLD_40, distance_m=8.), duration_s=40.),
       dict(end_reason='script_distance', a22=True, no_fault=True, rate_counts='normal'))
script('normal_straight_w2', 'normal', 'straight hold in a development corridor until the estimator row end or 25 m',
       episode('normal_straight_w2', world=W2_DEV, start_pose=(1., 0., 0.), controller=dict(HOLD_40, distance_m=25.),
               duration_s=100.),
       dict(end_reason=('row_end', 'script_distance'), a22=True, no_fault=True, rate_counts='normal'))
script('normal_zigzag', 'normal', 'zigzag +-6 deg at 40 %, three cycles, stop',
       episode('normal_zigzag', controller=dict(kind='segments', segments=[
           _seg(2., 40, 0., 'lead'), dict(duration_s=12., throttle=40, label='zigzag',
                                          steer_deg=dict(fn='sine', amplitude_deg=6., period_s=4.))]), duration_s=20.),
       dict(end_reason='script_complete', a22=True, no_fault=True, rate_counts='normal'))
script('normal_arcs', 'normal', 'two arcs at 10 deg and 40 % (an S-curve: left 2.5 s, straight 2 s, right 2.5 s), '
       'stop; the heading stays inside the row geometry guard (25 deg), so no full circle in a row',
       episode('normal_arcs', controller=segments([_seg(2., 40, 0., 'lead'), _seg(2.5, 40, 10., 'arc_left'),
                                                   _seg(2., 40, 0., 'straight'), _seg(2.5, 40, -10., 'arc_right'),
                                                   _seg(2., 40, 0., 'straight_end')]), duration_s=18.),
       dict(end_reason='script_complete', a22=True, no_fault=True, rate_counts='normal'))
script('normal_steps_standstill', 'normal', 'standstill steering steps +-5 deg (throttle 0; alignment size), then the end '
       'hold',
       episode('normal_steps_standstill', controller=dict(kind='segments', end='hold_end', segments=[
           _seg(1., 0, 0., 'settle'), _seg(4., 0, 5., 'left'), _seg(5., 0, -5., 'right'), _seg(4., 0, 0., 'centre')]),
               duration_s=20.),      # the end (about 15.2 s) + the 3 s A22 window (review P4R-06: 18 s cut it to 2.8 s)
       dict(end_reason='script_complete', a22=True, no_fault=True, rate_counts='normal'))
script('finding_standstill_reversal', 'finding', 'standstill reversal +10 -> -10 deg at the executor standstill slew: '
       'with the placeholder profile the actuator at standstill (servo 15 %/deg, standstill load 3 N m, start PWM 35 %) '
       'moves at about 4 deg/s after a backlash dead time, so the measured angle lags more than 3 deg for over 1.5 s. '
       'Until 2026-11-13 the host steering_not_following latch tripped here; since SIM_ACCEPTANCE amendment 43 '
       '(7d32154) the host counts the follow error only after the MCU servo ramp reached its goal, so the reversal '
       'completes without a host latch and the MCU TRACK rule (progress-aware) watches the slew (ST-03, 2026-11-14: the '
       'host copy before 7d32154 still latches at 8.14 s in this script)',
       episode('finding_standstill_reversal', controller=dict(kind='segments', end='hold_end', segments=[
           _seg(1., 0, 0., 'settle'), _seg(4., 0, 10., 'left'), _seg(5., 0, -10., 'right'), _seg(4., 0, 0., 'centre')]),
               duration_s=18.),
       dict(end_reason='script_complete', no_fault=True, no_restart=True))
RF03_SWEEP_S = 30.            # the THERMAL window (thw 30 s): sweeps as long as the proxy looks back
for _duty in (0, 4):
    script('rf03_sweeps_duty%d' % _duty, 'rf03', 'standstill full-range sweeps +-20 deg (triangle, 2.5 s period, slewed '
           'by the executor standstill limit) with drive duty %d %% for %d s, host follow rule relaxed as in the thermal '
           'script (RF-03: THERMAL rate; duty 4 %% is inside the drive deadband, so the vehicle does not move and the '
           'executor no-response budget may end the run first)' % (_duty, RF03_SWEEP_S),
           episode('rf03_sweeps_duty%d' % _duty, controller=dict(kind='segments', end='hold_end', segments=[
               _seg(.5, _duty, 0., 'settle'), dict(duration_s=RF03_SWEEP_S, throttle=_duty, label='sweeps',
                                                   steer_deg=dict(fn='triangle', amplitude_deg=20., period_s=2.5))]),
                   rig={'host_overrides': {'host_faults': {'track_duration_s': 60.}}}, duration_s=RF03_SWEEP_S + 4.),
           dict(rate_counts='rf03'))


def by_name():
    return {s['name']: s for s in SCRIPTS}


RATE_GROUPS = dict(normal=('normal_straight_w0', 'normal_zigzag', 'normal_arcs', 'normal_steps_standstill'),
                   rf03=('rf03_sweeps_duty0', 'rf03_sweeps_duty4'))
RATE_SEEDS = tuple(range(100, 110))      # seeds of the false-detection rates (10 per script)
