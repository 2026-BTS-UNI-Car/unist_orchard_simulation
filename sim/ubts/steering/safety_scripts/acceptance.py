"""P4 acceptance checks of the front-steer executor-in-the-loop F-SIM (SIM_ACCEPTANCE A5, A8, A11, A13, A17, A19, A22).

run_all(out_dir) runs the safety script catalogue through tools/row_executor/sim_in_loop_steer.py (one T1 rig per
episode), evaluates every script against its stated expectation (evaluate_script) and assembles:
  A5(F)   sensor format after RealFormat (3240 beams, raw order, angle_min / increment, frame laser, 0 = no return,
          range_min / max, 10 Hz, the yaw-offset convention with a post straight ahead) and statistics at standstill
          (LiDAR range noise sd and IMU gyro noise sd against the profile / plant values +-10 %, IMU rate +-1 %) and
          while driving (audit AUD3A-02: a 40 % straight run on W0; the gyro noise around the true yaw rate and the
          acceleration vibration RMS against the profile driving-vibration model +-10 %, scans 10 Hz, IMU rate +-1 %)
  A8      the same episode twice: serial log, trace and executor log digests equal; another seed differs; a process
          pool of 2 gives the same digests as one worker
  A11     report: episode wall time, executor step latency p50 / p99 / p99.9 (wall, without the rig work inside a
          blocking request), MCU logic cost per FW_STEP; the report records the process pool size (workers): run it
          with 1 and with N (SIM_ACCEPTANCE A11)
  A13(F)  steering.truth_isolation.check_tree() over the work tree (incl. tools/row_executor, tools/field_calib and the
          new files); the result lists every exempted finding (exact key) and their count
  A17(F)  STOP send / parse / confirmation, resend grid and latch (amendment 7), TTL, watchdog, fault -> steering 0, hold
          error, no automatic restart, lateral deviation during a link loss, the 09-19 mandatory failure scripts
  A19(F)  every MCU fault code and host latch from at least one script, judgement tick within +-1 tick of the spec 6.7
          recomputation (measure.expected_fault_tick), the resulting state; 0 false detections in the normal scripts;
          reported rates: STALL and steering_ineffective over the normal scripts x RATE_SEEDS, THERMAL over the RF-03 sweeps
  A22(F)  3 s after every terminal moment: no commanded steering move beyond the hold tolerance, no automatic centring
Evaluation code (A13 role 'evaluator'). Results are placeholder-based SIM results (자리값 기반 SIM 결과).
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

from . import catalogue as CAT
from . import measure as M
from . import states as ST

_REPO = Path(__file__).resolve().parents[4]
if str(_REPO / 'tools') not in sys.path:
    sys.path.insert(0, str(_REPO / 'tools'))

SCHEMA = 'ubts.steer_sim.p4_acceptance.v1'
RESULT_KIND = '자리값 기반 SIM 결과'
STEER_FAULT_CODES = ('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'SOFTLIMIT',
                     'STALL', 'TRACK', 'THERMAL', 'TTL', 'WATCHDOG', 'CRC', 'BOOT')
HOST_LATCHES = ('steering_status_stale', 'mcu_reset_detected', 'calibration_mismatch', 'hardware_profile_mismatch',
                'sim_identity_mismatch', 'steering_not_following', 'steering_ineffective', 'status_seq_gap',
                'stop_unconfirmed', 'mcu_disarmed', 'mcu_state_mismatch', 'c_rejected', 'hardware_fault', 'arm_rejected',
                'arm_unconfirmed', 'internal_error', 'mcu_ttl_expired')
# Every host latch needs a T1 script (A19). Review P4R-15: hardware_profile_mismatch and sim_identity_mismatch come from
# the rig's host link settings (host_hw_profile, host_simulated), arm_rejected from an operator re-arm while the MCU fault
# is still active (N - fault), internal_error from an injected host defect (rig fault host_defect). No exclusion is left.
HOST_LATCHES_NOT_IN_T1 = {}
# The 09-19 mandatory failure tests (SIM_ACCEPTANCE 2, last item) and the scripts that stand for them (audit AUD3A-19:
# the dashboard forced exit is the operator page / hold of the lease holder stopping: hold_lost, and the client loop
# stopping while it holds the lease: executor_stall, input deadline 0.75 s).
MANDATORY_MAP = (
    ('two screens and the CLI driving at once', ('two_screens_and_cli',)),
    ('link lost while the hold is pressed', ('link_loss_while_held', 'link_loss_lateral_5deg')),
    ('dashboard forced exit', ('hold_lost', 'executor_stall')),
    ('bridge forced exit', ('bridge_forced_exit', 'bridge_forced_exit_port_reset')),
    ('USB unplugged and plugged again (pty reopen)', ('usb_unplug_replug', 'usb_unplug_replug_reset')),
    ('MCU reset', ('mcu_reset',)),
    ('half frame', ('half_frame',)),
    ('CRC and range errors', ('crc_error_single', 'crc_error_burst', 'range_error')),
    ('status line loss', ('status_line_loss',)),
    ('late drive right after a STOP', ('late_drive_after_stop',)),
    ('system clock change (monotonic deadlines)', ('clock_change',)),
)


def _sim_in_loop():
    from row_executor import sim_in_loop_steer as S
    return S


def _seconds(t):
    return int(round(float(t) * 1000.))


# --- one script ---------------------------------------------------------------------------------------------------------
def run_script(entry, seed=None, out_dir=None, native_executable=None, build_dir=None, keep_facade=False):
    """Run one catalogue entry (seed: overrides the spec seed). Returns (result, artefacts[, facade])."""
    S = _sim_in_loop()
    spec = dict(entry['spec'])
    if seed is not None:
        spec['seed'] = int(seed)
        spec['name'] = '%s_s%d' % (entry['name'], seed)
    result, facade = S.run_episode(spec, out_dir, native_executable, build_dir, keep=True)
    try:
        art = M.artefacts(facade, S.normalise_spec(spec))
    finally:
        if not keep_facade:
            facade.rig.close()
    return (result, art, facade) if keep_facade else (result, art)


def _item(items, name, ok, measured=None, limit=None, note=None):
    """ok: True / False, or None = not applicable in this script (reported, not judged; review RS-03)."""
    items.append(dict(item=name, ok=None if ok is None else bool(ok), measured=measured, limit=limit, note=note))
    return ok


def _judged(rows):
    """True when every judged row passed (rows with ok None are not applicable)."""
    return all(r['ok'] is not False for r in rows)


def _as_tuple(value):
    return value if isinstance(value, tuple) else (value,)


def _stall_windows(entry):
    """(first tick, end tick) of the script's MCU loop stalls."""
    out = []
    for f in entry['spec']['faults']:
        if f['kind'] == 'mcu_stall':
            a = _seconds(f['at_s'])
            out.append((a, a + _seconds((f.get('params') or {}).get('duration_s', .5))))
    return out


def _onset_tick(entry, art):
    """The tick from which a fault rule can be true: the injection / action time of the script."""
    times = [f['at_s'] for f in entry['spec']['faults']] + [a['at_s'] for a in entry['spec']['actions']]
    return _seconds(min(times)) if times else 0


def evaluate_script(entry, art, result):
    """The checks of one script's expectation: dict(name, passed, items, facts)."""
    exp = entry['expect']
    s = art['summary']
    items, facts = [], {}
    if 'end_reason' in exp:
        want = exp['end_reason']
        if want is None:
            _item(items, 'run not ended', s['end_reason'] is None, s['end_reason'])
        else:
            _item(items, 'end reason', s['end_reason'] in _as_tuple(want), s['end_reason'], list(_as_tuple(want)),
                  s.get('end_note'))
    if 'pause_reason' in exp:
        pauses = [e for e in s['events'] if e[1] == 'PAUSED' and e[2] == exp['pause_reason']]
        _item(items, 'pause reason', bool(pauses), pauses[:1], exp['pause_reason'])
    stop_tick = None
    codes = _as_tuple(exp['mcu_fault']) if 'mcu_fault' in exp else ()
    if codes:
        facts['faults'] = fault_facts(entry, art, codes)
        found = sorted((f for f in facts['faults'] if f['found']), key=lambda f: f['tick'])
        _item(items, 'MCU fault %s' % '/'.join(codes), bool(found), [f['code'] for f in found])
        if exp.get('first_rule') and found:
            # audit AUD3A-18: the code the MCU raised first must be the rule the spec 6.7 recomputation trips first
            onset = _onset_tick(entry, art)
            expected = []
            for code in codes:
                tick, method = M.expected_fault_tick(code, art, onset)
                if tick is not None:
                    expected.append((tick, code))
            facts['first_rule'] = dict(expected=sorted(expected), found=(found[0]['code'], found[0]['tick']))
            first = min(expected) if expected else None
            _item(items, 'first rule by the spec recomputation', first is not None and first[1] == found[0]['code']
                  and abs(first[0] - found[0]['tick']) <= 1, (found[0]['code'], found[0]['tick']),
                  None if first is None else (first[1], first[0]), 'earliest expected tick of %s' % '/'.join(codes))
        for f in found[:1]:
            if f['code'] in ('NOCAL', 'BOOT', 'SOFTLIMIT'):
                ST.mcu_result_items(items, f['code'], art, f['tick'], 'bit')
            if f.get('expected_tick') is not None:
                _item(items, 'judgement tick %s within +-1 of spec 6.7' % f['code'], abs(f['tick'] - f['expected_tick']) <= 1,
                      f['tick'], f['expected_tick'], f.get('method'))
            stop_tick = f['tick'] if f['code'] not in ('SOFTLIMIT', 'NOCAL', 'BOOT') else stop_tick
            if exp.get('steer_off') and f['code'] not in ('SOFTLIMIT', 'NOCAL', 'BOOT', 'TTL'):
                off = M.steer_off_tick(art, f['tick'])
                _item(items, 'fault -> steering output 0 within 1 tick', off is not None and off - f['tick'] <= 1,
                      None if off is None else off - f['tick'], 1, 'steer_pwm 0 and steer EN 0')
                zero = M.drive_zero_tick(art, f['tick'])
                if f['code'] == 'THERMAL':      # THERMAL alone releases the drive (ramp to 0, fw fault step; spec 6.4)
                    bound, d0 = M.release_bound_ticks(art, f['tick'])
                    _item(items, 'fault -> drive release ramp to 0 (THERMAL alone)',
                          zero is not None and zero - f['tick'] <= bound, None if zero is None else zero - f['tick'],
                          bound, 'duty %d at the fault, 255 per s, 10 ms grid' % d0)
                else:
                    _item(items, 'fault -> drive output 0', zero is not None and zero - f['tick'] <= 1,
                          None if zero is None else zero - f['tick'], 1)
                _item(items, 'MCU state after the fault', M.state_at(art, f['tick'] + 2) == 'DISARMED',
                      M.state_at(art, f['tick'] + 2), 'DISARMED')
            # the resulting state of spec 6.4 / 6.8 (audit AUD3A-01): CRC and the latch bit of the latched codes
            # (NOCAL, BOOT, SOFTLIMIT come from status bits: above)
            if f['code'] not in ('NOCAL', 'BOOT', 'SOFTLIMIT'):
                ST.mcu_result_items(items, f['code'], art, f['tick'], 'F')
    if exp.get('mcu_ttl'):
        expected, tick, last = M.expected_ttl_tick(art, _onset_tick(entry, art))
        facts['ttl'] = dict(f_tick=tick, expected=expected)
        _item(items, 'F TTL line', tick is not None, tick)
        if tick is not None and expected is not None:
            _item(items, 'TTL judgement = last C arrival + ttl (+-1)', abs(tick - expected) <= 1, tick, expected)
            zero = M.drive_zero_tick(art, tick - 1)
            _item(items, 'TTL -> drive 0 <= ttl + 1 tick', zero is not None and zero <= expected + 1, zero, expected + 1)
            ST.mcu_result_items(items, 'TTL', art, tick, 'F')
        stop_tick = tick if stop_tick is None else min(stop_tick, tick or stop_tick)
    if exp.get('mcu_watchdog'):
        expected, tick = M.expected_watchdog_tick(art, _onset_tick(entry, art))
        facts['watchdog'] = dict(w_tick=tick, expected=expected)
        _item(items, 'W line (link watchdog)', tick is not None, tick)
        if tick is not None and expected is not None:
            _item(items, 'watchdog = last input + wd + 1 (+-1)', abs(tick - expected) <= 1, tick, expected)
            ch = art['channels']
            before = max(abs(M.value_at(ch['drive_l'], tick - 1)), abs(M.value_at(ch['drive_r'], tick - 1)))
            if before:
                # the watchdog itself must brake: judged only when the drive was not 0 before the W tick (review RS-03)
                zero = M.drive_zero_tick(art, tick)
                _item(items, 'watchdog -> drive 0 <= wd + 1 tick', zero is not None and zero <= expected + 1, zero,
                      expected + 1, 'drive %d before the W tick' % before)
            else:
                _item(items, 'watchdog -> drive 0 <= wd + 1 tick', None, None, expected + 1,
                      'not applicable: the drive was already 0 before the W tick (an earlier stop, e.g. the C TTL '
                      'brake); link_loss_watchdog_brake judges the watchdog brake')
            _item(items, 'watchdog -> DISARMED', M.state_at(art, tick + 1) == 'DISARMED', M.state_at(art, tick + 1))
            # A17 'watchdog -> ..., latch' (audit AUD3A-08): the MCU WATCHDOG latch bit stays in the status lines until
            # the next operator ARM, and the host latches (any latch from the W tick on) or the executor run ended
            kept, until = ST._bit_kept(art, 'WATCHDOG', tick)
            status_bits = [r['fields'].get('fl', 0) & M.bit('WATCHDOG') for r in M.m2h_status(art, since_ms=tick + 1)
                           if until is None or r['queued'] < until]
            # an MCU reset that ends the window before the host could read a status line (the port was closed, e.g.
            # usb_unplug_replug_reset) leaves only the MCU part to judge; otherwise the host must see the bit
            reset_first = until is not None and until == ST._first_reset_after(art, tick)
            _item(items, 'watchdog latch bit kept until the next operator ARM (MCU and status lines)',
                  kept and (bool(status_bits) or reset_first) and all(status_bits),
                  dict(until_arm_tick=until, lines=len(status_bits), ended_by_reset=reset_first))
            latched = [name for name, since, _ in art['latches'] if since is not None and _seconds(since) >= tick]
            _item(items, 'watchdog -> host latch or ended run', bool(latched) or s['end_reason'] is not None,
                  dict(host_latches=latched, end_reason=s['end_reason']))
            mode_items = []
            ST.mcu_result_items(mode_items, 'WATCHDOG', art, tick, 'W')
            for row in mode_items:
                if 'steering' in row['item']:
                    row['item'] = 'watchdog -> ' + row['item'].split(' -> ', 1)[1]
                    items.append(row)
        stop_tick = tick if stop_tick is None else min(stop_tick, tick or stop_tick)
    facts['latches'] = ST.latch_facts(entry, art)
    by_latch = {row['name']: row for row in facts['latches']}
    for name in _as_tuple(exp.get('host_latch', ())):
        when = M.latch_time(art, name)
        _item(items, 'host latch %s' % name, when is not None, when)
        row = by_latch.get(name)
        if row is not None:
            # audit AUD3A-01: the latch time against the design rule (+- one loop period) and the resulting state
            _item(items, 'latch time %s = design (+-1 loop period)' % name, row['time_ok'],
                  row['latch_ms'], row['design_ms'], row['method'])
            _item(items, 'latch state %s (stop, no lease, fault, no C)' % name, row['state_ok'],
                  [(i['item'], i['measured']) for i in row['state_items'] if not i['ok']] or 'ok')
    if 'arm_refused' in exp:
        error = (s.get('start_error') or {}).get('error') if isinstance(s.get('start_error'), dict) else None
        codes_ok = error is not None and any(code in error for code in _as_tuple(exp['arm_refused']))
        _item(items, 'arm refused', codes_ok, error, list(_as_tuple(exp['arm_refused'])))
        arm_lines = [r for r in M.h2m(art, 'ARM') if r['t_ms'] >= _seconds(.45)]
        _item(items, 'no ARM line reached the MCU after the refused arm', not arm_lines, len(arm_lines), 0)
    if 'stop_at' in exp:
        timing = M.stop_timing(art, exp['stop_at'], _stall_windows(entry))
        facts['stop'] = timing
        if timing['found']:
            _item(items, 'STOP request -> STOP byte <= 20 ms', timing['send_delay_ms'] <= art['host']['bridge_period_ms'],
                  timing['send_delay_ms'], art['host']['bridge_period_ms'])
            servo_ticks = int(math.ceil(timing['servo_tick_ms'] - 1e-9))
            ptz = timing['parse_to_zero_ticks']
            _item(items, 'MCU STOP parse -> drive 0 <= 1 servo tick', ptz is not None and ptz <= servo_ticks, ptz,
                  servo_ticks, None if timing['processed_tick'] == timing['parse_tick'] else
                  'arrived inside an MCU stall: parsed at its end, tick %d' % timing['processed_tick'])
            if not exp.get('stop_unconfirmed'):
                cd = timing['confirm_delay_ms']
                _item(items, 'STOP confirmation <= 2 status periods', cd is not None and cd <= timing['confirm_limit_ms'],
                      cd, timing['confirm_limit_ms'])
            stop_tick = timing['parse_tick'] if stop_tick is None else min(stop_tick, timing['parse_tick'])
        else:
            _item(items, 'STOP byte sent', False, None)
    if exp.get('stop_unconfirmed'):
        timing = facts.get('stop') or M.stop_timing(art, exp['stop_at'])
        grid, latch_offset = M.resend_grid(art)
        offsets = timing.get('resend_offsets_ms') or []
        _item(items, 'STOP resends <= 5 on the amendment-7 grid', offsets[:len(grid)] == grid and len(offsets) <= 5,
              offsets, grid)
        when = M.latch_time(art, 'stop_unconfirmed')
        if when is not None and timing.get('found'):
            off = _seconds(when) - timing['sent_ms']
            _item(items, 'stop_unconfirmed latch at t0 + 400 ms', off == latch_offset, off, latch_offset)
        event = M.executor_event(art, 'stop_unconfirmed')
        _item(items, 'executor event stop_unconfirmed', event is not None, None if event is None else event.get('t'))
    if exp.get('hold_check'):
        # the hold stops of A17: an operator STOP, the link watchdog (wdmode hold) and the bad-line burst (CRC: HOLD of
        # the measured position); review RS-09
        hold_t0, source = None, None
        if facts.get('stop', {}).get('found'):
            hold_t0, source = facts['stop']['parse_tick'], 'STOP parse tick'
        elif (facts.get('watchdog') or {}).get('w_tick') is not None:
            hold_t0, source = facts['watchdog']['w_tick'], 'W tick'
        else:
            crc = [f for f in facts.get('faults') or () if f['found'] and f['code'] == 'CRC']
            if crc:
                hold_t0, source = crc[0]['tick'], 'F CRC tick'
        tol = art['hold_tolerance_rad']
        err = None if hold_t0 is None else M.hold_error(art, hold_t0 / 1000., 2.)
        _item(items, 'steering hold error after the stop (2 s)', err is not None and err <= tol + 1e-9,
              None if err is None else round(math.degrees(err), 4), round(math.degrees(tol), 4),
              'deg, true wheel side, from the %s' % (source or 'stop (not found)'))
    if exp.get('no_restart', False):
        t_stop = stop_tick
        if t_stop is None and s.get('ended_at_s') is not None:
            t_stop = _seconds(s['ended_at_s'])
        if t_stop is None and exp.get('pause_at') is not None:
            t_stop = _seconds(exp['pause_at']) + 50
        if t_stop is not None:
            # from the first tick with both drive outputs 0 after the stop to the end: no output may grow (a release
            # ramp only shrinks; review P4R-11 / RS-02); 1.5 s after the stop the drive is 0; no ARM line except an
            # explicit operator re-arm of the script (operator_rearm_at)
            zero, increase = M.drive_increase_after(art, t_stop)
            restart = M.drive_nonzero_after(art, t_stop + 1500)
            rearm = exp.get('operator_rearm_at')
            arms = [r for r in M.h2m(art, 'ARM', since_ms=t_stop + 1)
                    if rearm is None or not _seconds(rearm) <= r['t_ms'] <= _seconds(rearm) + 500]
            _item(items, 'no automatic restart (drive never grows after the stop, 0 after 1.5 s, no ARM)',
                  zero is not None and increase is None and restart is None and not arms,
                  dict(stop_tick=t_stop, drive_zero_tick=zero, drive_increase_tick=increase, drive_nonzero_tick=restart,
                       arm_lines=len(arms)))
        else:
            _item(items, 'no automatic restart (drive never grows after the stop, 0 after 1.5 s, no ARM)', False,
                  None, note='no stop moment found')
    if exp.get('bridge_restart'):
        bridge_restart_items(items, art)
    if exp.get('boot_after_s') is not None:
        since = _seconds(exp['boot_after_s'])
        reboot = M.first_tick(art['channels']['mcu_state'], lambda v: v in (0, 255), since)
        _item(items, 'MCU reset when the port is opened again (a new boot)', reboot is not None, reboot, since)
    if exp.get('a22'):
        t_term = s.get('ended_at_s')
        if t_term is None and exp.get('pause_at') is not None:
            pauses = [e[0] for e in s['events'] if e[1] == 'PAUSED' and e[2] != 'await_hold']
            t_term = pauses[0] if pauses else None
        window = M.command_after(art, t_term, 3.) if t_term is not None else None
        facts['a22'] = window
        _item(items, 'A22: no commanded steering move after the terminal moment, no centring',
              window is not None and window['passed'], window)
    if 'client2' in exp:
        got = [(r['op'], r['ok'], r['error']) for r in s['client2']]
        want = [tuple(x) for x in exp['client2']]
        _item(items, 'second client replies', got == want, got, want)
    if 'other_screen' in exp:
        got = [(r['op'], r['ok'], r['error']) for r in s.get('other_screen') or ()]
        want = [tuple(x) for x in exp['other_screen']]
        _item(items, 'second screen refused (one_press, A17 amendment 31)', got == want, got, want)
    if exp.get('link_lost_at') is not None:
        # A17 amendment 31: the operator link lost (the screen's heartbeats stop at link_lost_at) -> the executor's drive
        # command is 0 at most operator_timeout_s + one tick (50 ms) later
        tr = art['trace']
        t, throttle = np.asarray(tr['t_s'], dtype=float), np.asarray(tr['ctrl_throttle_pct'], dtype=float)
        at = float(exp['link_lost_at'])
        limit = at + float(exp.get('link_timeout_s', 1.)) + .05
        zero = np.flatnonzero((t >= at - 1e-9) & (throttle == 0.))
        first = None if not len(zero) else round(float(t[zero[0]]), 3)
        moving = bool(np.any((t >= at - 1e-9) & (t < at + .01) & (throttle > 0.)))
        _item(items, 'operator link lost -> zero drive command within operator_timeout_s + 1 tick',
              moving and first is not None and first <= limit + 1e-9, first, round(limit, 3),
              'driving at the loss' if moving else 'NOT driving at the loss (script problem)')
    if 'go_refused' in exp:
        errors = [g[1] for g in s.get('go_refused') or ()]
        _item(items, 'go after the stop refused', bool(errors) and errors[0] in _as_tuple(exp['go_refused']), errors,
              list(_as_tuple(exp['go_refused'])))
    if exp.get('lease_lost'):
        # amendment 34: the lease taken while the armed run waits keeps the run open; ACTIVE only after the re-arm
        events = [e.get('event') for e in s.get('executor_events') or ()]
        rearm = exp.get('operator_rearm_at')
        active = s.get('first_active_at_s')
        lost = [e for e in s.get('executor_events') or () if e.get('event') == 'lease_lost']
        _item(items, 'lease lost while waiting: the run stays open (event lease_lost before any run_end)',
              bool(lost) and ('run_end' not in events or events.index('lease_lost') < events.index('run_end')),
              lost[:1])
        _item(items, 'the run moves only after the operator re-arm (lease_lost_rearm, first ACTIVE after it)',
              'lease_lost_rearm' in events and active is not None and rearm is not None and active > float(rearm),
              dict(first_active_at_s=active, rearm_at_s=rearm))
    if 'late_drive_error' in exp:
        errors = [r['reply'].get('error') for r in s['late_drive']]
        _item(items, 'late drive refused', errors[:1] == [exp['late_drive_error']], errors, exp['late_drive_error'])
    if exp.get('lateral'):
        facts['lateral'] = lateral_check(art, exp['loss_at'])
        lat = facts['lateral']
        _item(items, 'lateral shift during the link loss vs the analytic arc (+-10 % or +-0.01 m)',
              lat.get('consistent', False), lat.get('sim_m'), lat.get('analytic_m'))
    if exp.get('no_fault'):
        f_lines = [t for t, q, _ in M.mcu_lines(art, 'F') if t.split(' ')[1] not in ('NOCAL',)]
        latches = [l[0] for l in art['latches']]
        _item(items, 'no MCU fault line', not f_lines, f_lines[:3])
        _item(items, 'no host latch', not latches, latches)
    passed = _judged(items)
    return dict(name=entry['name'], group=entry['group'], passed=passed, items=items, facts=facts,
                end_reason=s['end_reason'], end_note=s.get('end_note'), digests=result['digests'],
                wall_s=result.get('wall_s'), timing=s['timing'])


def bridge_restart_items(items, art):
    """A17 09-19 'bridge forced exit' (review P4R-05 / RS-04): the rig killed the bridge process and started a new one
    (t1_rig fault bridge_restart). The executor's requests failed while no process ran (socket errors); the new process
    holds no lease, sends the session STOP and no C line; the restart and reset items come from the trace and the
    serial log."""
    events = art.get('bridge_events') or []
    exits = [t for kind, t in events if kind == 'exit']
    starts = [t for kind, t in events if kind == 'start']
    _item(items, 'bridge process killed and a new one started', bool(exits) and bool(starts) and starts[0] > exits[0],
          dict(exit_ms=exits, start_ms=starts))
    counters = art.get('rig_counters') or {}
    failed = int(counters.get('requests_without_bridge', 0)) + int(counters.get('requests_lost_with_bridge', 0))
    _item(items, "the executor's requests failed while no bridge process ran (socket error)", failed > 0, failed)
    if not (exits and starts):
        return
    start = starts[0]
    tr = art['trace']
    k = int(np.searchsorted(tr['t_s'], start / 1000. + 1e-9))     # rows after the new process's first step
    lease = int(np.max(tr['host_lease'][k:])) if k < len(tr['t_s']) and 'host_lease' in tr else None
    _item(items, 'the new bridge process holds no lease', lease == 0, lease, 0)
    stops = M.h2m(art, 'STOP', since_ms=start)
    cs = M.h2m(art, 'C', since_ms=start)
    _item(items, 'the new bridge process sends the session STOP and no C line', bool(stops) and not cs,
          dict(stop_lines=len(stops), c_lines=len(cs)))


def fault_facts(entry, art, codes):
    """For each expected code: the MCU's first F line (or status bit) after the onset, the recomputed spec tick."""
    onset = _onset_tick(entry, art)
    out = []
    for code in codes:
        if code == 'NOCAL':
            # at boot NOCAL goes into the latched mask without an F line (only CAL ERASE sends F NOCAL)
            tick = M.first_tick(art['channels']['fault_bits'], lambda v: bool(v & M.bit('NOCAL')), 0)
            out.append(dict(code=code, found=tick is not None, tick=tick, expected_tick=art['boot_delay_ticks'],
                            method='setup tick (latched at boot; status bit)', text=None))
            continue
        if code == 'BOOT':
            tick = M.first_tick(art['channels']['fault_bits'], lambda v: bool(v & M.bit('BOOT')), onset)
            reboot = M.first_tick(art['channels']['mcu_state'], lambda v: v in (0, 255), onset)
            expected = None if reboot is None else reboot + art['boot_delay_ticks']
            out.append(dict(code=code, found=tick is not None, tick=tick, expected_tick=expected,
                            method='reset tick + boot delay (status bit)', text=None))
            continue
        if code == 'SOFTLIMIT':
            soft = art['cal'].get('soft_l')
            pos_tick = None if soft is None else M.first_tick(art['channels']['servo_pos'], lambda v: v > soft, onset)
            tick = M.first_tick(art['channels']['fault_bits'], lambda v: bool(v & M.bit('SOFTLIMIT')), onset)
            out.append(dict(code=code, found=tick is not None, tick=tick, expected_tick=pos_tick,
                            method='first P[n] > soft_l (condition bit)', text=None))
            continue
        if code == 'CRC':
            tick, text = M.f_line(art, 'CRC', onset)
            bad = [r for r in art['serial'] if r['dir'] == 'h2m' and r['note'] == 'bridge:crc_error']
            crcn = int(art['cfg'].get('crcn', 5))
            expected = bad[crcn - 1]['last_tick'] if len(bad) >= crcn else None
            out.append(dict(code=code, found=tick is not None, tick=tick, expected_tick=expected,
                            method='arrival of the crcn-th consecutive bad line', text=text))
            continue
        tick, text = M.f_line(art, code, onset)
        expected, method = M.expected_fault_tick(code, art, onset) if tick is not None else (None, None)
        out.append(dict(code=code, found=tick is not None, tick=tick, expected_tick=expected, method=method,
                        text=text))
    return out


def lateral_check(art, loss_at_s):
    """A17: sideways shift of base_link (relative to the heading at the loss) from the loss to rest, against the
    analytic arc: the path curvature at the loss held until rest, speed v0 until the drive-zero tick, then the brake stop
    model (onset, decel of the profile's brake mode). The rear axle follows the arc (1 - cos(k s)) / k; base_link sits d
    ahead of it (d = -rear_axle_x_in_base_m) and adds d sin(k s). The plant's random lateral slip is not modelled."""
    from steering import drive as D
    from steering.vehicle_profile import load_profile
    tr = art['trace']
    t = tr['t_s']
    k0 = int(np.searchsorted(t, loss_at_s - 1e-9))
    if k0 >= len(t) - 1:
        return dict(evaluated=False)
    v = tr['truth_v_m_s']
    rest = np.flatnonzero((np.arange(len(t)) > k0) & (np.abs(v) < .005))
    if not len(rest):
        return dict(evaluated=False, reason='no rest')
    k1 = int(rest[0])
    x0, y0, yaw0 = float(tr['truth_x_m'][k0]), float(tr['truth_y_m'][k0]), float(tr['truth_yaw_rad'][k0])
    dx, dy = float(tr['truth_x_m'][k1]) - x0, float(tr['truth_y_m'][k1]) - y0
    sim = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
    kappa = float(tr['truth_curvature_1_m'][k0])
    v0 = float(v[k0])
    zero = M.drive_zero_tick(art, _seconds(loss_at_s))
    brake = D.stop_modes(load_profile())['brake']
    t_drive = 0. if zero is None else max(0., zero / 1000. - loss_at_s)
    path = v0 * (t_drive + brake.onset_s) + v0 * v0 / (2. * brake.decel_median_m_s2)
    d = -float(load_profile().value('rear_axle_x_in_base_m'))
    arc = (1. - math.cos(kappa * path)) / kappa if abs(kappa) > 1e-9 else 0.
    analytic = arc + d * math.sin(kappa * path)
    diff = abs(abs(sim) - abs(analytic))
    consistent = diff <= max(.10 * abs(analytic), .01)
    steer_deg = math.degrees(float(tr['truth_delta_eq_rad'][k0]))
    return dict(evaluated=True, sim_m=round(sim, 4), analytic_m=round(analytic, 4), diff_m=round(diff, 4),
                consistent=bool(consistent), v0_m_s=round(v0, 3), kappa_1_m=round(kappa, 5), steer_deg=round(steer_deg, 3),
                rear_axle_arc_m=round(arc, 4), base_offset_m=round(d, 3),
                drive_zero_after_s=round(t_drive, 3), path_m=round(path, 4),
                design_target_ok=bool(abs(sim) <= .05) if abs(steer_deg) <= 5.05 and abs(v0) <= .45 else None)


# --- A5 ----------------------------------------------------------------------------------------------------------------
A5_POST = [6., 0., .12]          # a post straight ahead of the start (yaw offset convention check)


def a5_check(native_executable=None, build_dir=None):
    """A5(F): format of the scan messages the executor receives and standstill statistics (module docstring)."""
    S = _sim_in_loop()
    from steering.s2_format import BEAMS, RAW_AMIN, RAW_INC
    spec = dict(name='a5_standstill', seed=5, world={'kind': 'W0', 'markers': CAT.ROW_POSTS + [A5_POST]},
                controller={'kind': 'hold_still', 'duration_s': 30.}, duration_s=25., capture_inputs=400)
    result, facade = S.run_episode(spec, None, native_executable, build_dir, keep=True)
    try:
        scans, imus = facade.inputs['scans'], facade.inputs['imus']
        plant = facade.rig.plant
        params = plant.params
        offset = facade.cfg.lidar_yaw_offset_deg
        lidar_x = float(facade.rig.profile.value('lidar_x_in_base_m'))
        vib = plant.imu_vibration_rms()
        expected_gyro_sd = math.radians(math.hypot(params.gyro_still_noise_sd_deg_s,
                                                   params.gyro_vibration_deg_s_per_m_s2 * vib))
        noise_sd = float(facade.rig.profile.value('lidar_noise_sd_m'))
        imu_rate = float(facade.rig.imu_rate_hz)
    finally:
        facade.rig.close()
    items = []
    first = scans[0]['msg']
    _item(items, '3240 beams', len(first.ranges) == BEAMS, len(first.ranges), BEAMS)
    _item(items, 'angle_min float32(-pi)', first.angle_min == RAW_AMIN, first.angle_min, RAW_AMIN)
    _item(items, 'angle_increment', first.angle_increment == RAW_INC, first.angle_increment, RAW_INC)
    _item(items, 'frame laser', first.header.frame_id == 'laser', first.header.frame_id)
    _item(items, 'range_min 0.05, range_max 30', (first.range_min, first.range_max) == (.05, 30.),
          [first.range_min, first.range_max])
    zeros = [int(np.sum(np.asarray(m['msg'].ranges) == 0.)) for m in scans]
    _item(items, '0 = no return (beams without a return are 0.0, none negative or non-finite)',
          min(zeros) > 0 and all(np.all(np.isfinite(m['msg'].ranges)) and np.all(np.asarray(m['msg'].ranges) >= 0.)
                                 for m in scans), dict(min_zero_beams=min(zeros)))
    stamps = np.array([m['header'] for m in scans])
    dts = np.diff(stamps)
    _item(items, '10 Hz scan stamps', bool(np.all(np.abs(dts - .1) < 1e-6)), round(float(np.mean(dts)), 6), .1)
    # the yaw-offset convention: raw beam j looks at body angle angle_min + j inc + offset; the post straight ahead
    raw_ahead = math.atan2(math.sin(-math.radians(offset)), math.cos(-math.radians(offset)))
    j0 = int(round((raw_ahead - RAW_AMIN) / RAW_INC)) % BEAMS
    expected_r = A5_POST[0] - lidar_x - A5_POST[2]
    near = [float(m['msg'].ranges[j0]) for m in scans]
    _item(items, 'post straight ahead at raw beam %d (offset %.2f deg, raw order)' % (j0, offset),
          abs(float(np.median(near)) - expected_r) < .03, round(float(np.median(near)), 4), round(expected_r, 4))
    # range noise at standstill: beams finite in every scan, sd over scans, minus the 1 mm rounding variance
    block = np.array([np.asarray(m['msg'].ranges, dtype=float) for m in scans[5:]])
    good = np.all(block > 0., axis=0) & (np.ptp(block, axis=0) < .2)
    sd = np.std(block[:, good], axis=0, ddof=1)
    measured = float(np.sqrt(max(0., float(np.median(sd)) ** 2 - (.001 ** 2) / 12.)))
    _item(items, 'LiDAR range noise sd vs profile +-10 %', abs(measured - noise_sd) <= .1 * noise_sd,
          round(measured, 5), noise_sd, '%d beams x %d scans' % (int(good.sum()), len(block)))
    gyro = np.array([m['gyro_z'] for m in imus[20:]])
    gsd = float(np.std(gyro, ddof=1))
    _item(items, 'IMU gyro noise sd at standstill vs the plant model +-10 %',
          abs(gsd - expected_gyro_sd) <= .1 * expected_gyro_sd, round(gsd, 6), round(expected_gyro_sd, 6),
          'model: hypot(still noise, vibration coefficient x the run still vibration %.3f m/s2)' % vib)
    istamps = np.array([m['header'] for m in imus])
    rate = (len(istamps) - 1) / float(istamps[-1] - istamps[0])
    _item(items, 'IMU rate vs profile +-1 %', abs(rate - imu_rate) <= .01 * imu_rate, round(rate, 3), imu_rate)
    drive = a5_drive_check(native_executable, build_dir)
    for i in drive['items']:
        items.append(dict(i, item='driving: ' + i['item']))
    return dict(check='A5(F)', passed=all(i['ok'] for i in items), items=items, scans=len(scans), imus=len(imus),
                digests=result['digests'], driving=dict((k, v) for k, v in drive.items() if k != 'items'))


A5_DRIVE_S = 30.             # 40 % straight hold on the W0 floor (two rows of posts), driving statistics


def a5_drive_check(native_executable=None, build_dir=None):
    """A5(F) driving statistics (audit AUD3A-02). The plant's IMU model while driving: vibration RMS v_k = hypot(drive
    vibration of the duty (profile imu_vibration_drive_below_90 / at_90 / at_100), steering-motor term x |steer PWM|);
    gyro noise sd_k = hypot(still noise, coupling x v_k) (deg/s); acceleration noise v_k / sqrt(3) per axis. The run
    records, next to every IMU sample, the plant's true state it was made from (evaluation only: accel, v, w, the
    vibration RMS, the per-run gyro scale and bias), so the noise is measured around the truth: gyro residual = gyro -
    (scale x rot_zz x (w + external) + bias), acceleration residual = acc - rot (accel, v w, g). Judged: sd of the gyro
    residual vs sqrt(mean sd_k^2) +-10 %, RMS of the acceleration residual (three axes) vs sqrt(mean v_k^2) +-10 %, the
    driving vibration equals the profile value of the duty, scans every 0.1 s, IMU rate +-1 %."""
    S = _sim_in_loop()
    spec = S.normalise_spec(dict(name='a5_driving', seed=6, world=dict(CAT.W0_ROWS),
                                 controller=dict(kind='straight_hold', drive_pct=40, distance_m=None),
                                 duration_s=A5_DRIVE_S, capture_inputs=4000))
    rig = S.build_rig(spec, native_executable, build_dir)
    truth = []
    try:
        plant = rig.plant
        original = plant.imu_sample

        def imu_sample():
            sample = original()
            truth.append((round(float(sample['stamp_s']), 6), plant.accel, plant.v, plant.w, plant.external_yaw_rate,
                          plant.imu_vibration_rms(), plant.vibration_rms, abs(plant._steer.applied_steer_pwm)))
            return sample
        plant.imu_sample = imu_sample
        cfg = S.steer_executor_config(rig, spec['config'])
        facade = S.SteerExecutorInTheLoop(rig, cfg, spec['controller'], spec['actions'], spec['auto_repress'],
                                          spec['capture_inputs'], operator=spec['operator'])
        facade.run(spec['duration_s'], spec['post_end_s'])
        params = plant.params
        scale, bias = plant.gyro_scale, plant.gyro_bias
        rot = params.imu_rotation
        imus, scans = facade.inputs['imus'], facade.inputs['scans']
        profile = rig.profile
        drive_pct = 40.
        expected_drive = float(params.drive_vibration(drive_pct))
        values = dict(below_90=float(profile.value('imu_vibration_drive_below_90_m_s2')),
                      at_90=float(profile.value('imu_vibration_drive_at_90_m_s2')),
                      at_100=float(profile.value('imu_vibration_drive_at_100_m_s2')),
                      steer=float(profile.value('imu_vibration_steer_m_s2')),
                      gyro_still_deg_s=float(params.gyro_still_noise_sd_deg_s),
                      gyro_coupling_deg_s_per_m_s2=float(params.gyro_vibration_deg_s_per_m_s2))
        imu_rate = float(rig.imu_rate_hz)
        summary = facade.summary()
    finally:
        rig.close()
    by_stamp = {row[0]: row for row in truth}
    gyro_res, acc_res, sd2, vib2, drive_vib, stamps = [], [], [], [], [], []
    for m in imus:
        row = by_stamp.get(round(float(m['header']), 6))
        if row is None:
            continue
        _, accel, v, w, ext, vib, drive_part, _ = row
        if abs(v) < .1:                     # the driving part only (the start and the stop are left out)
            continue
        expected_gyro = scale * rot[2][2] * (w + ext) + bias
        gyro_res.append(m['gyro_z'] - expected_gyro)
        force = (accel, v * w, 9.80665)
        acc = m['accel']
        acc_res.append([acc[i] - sum(rot[i][k] * force[k] for k in range(3)) for i in range(3)])
        sd2.append(math.radians(math.hypot(values['gyro_still_deg_s'], values['gyro_coupling_deg_s_per_m_s2'] * vib))
                   ** 2)
        vib2.append(vib ** 2)
        drive_vib.append(drive_part)
        stamps.append(float(m['header']))
    items = []
    _item(items, 'driving IMU samples (v > 0.1 m/s)', len(gyro_res) >= 300, len(gyro_res), 300)
    if not gyro_res:
        return dict(check='A5(F) driving', passed=False, items=items)
    gyro_res = np.asarray(gyro_res)
    acc_res = np.asarray(acc_res)
    expected_sd = float(np.sqrt(np.mean(sd2)))
    measured_sd = float(np.sqrt(np.mean(gyro_res ** 2)))
    _item(items, 'gyro noise sd while driving vs the profile vibration model +-10 %',
          abs(measured_sd - expected_sd) <= .1 * expected_sd, round(measured_sd, 6), round(expected_sd, 6),
          'rad/s around the true yaw rate (gyro scale and bias of the run removed); model hypot(still %.3f, %.2f x '
          'vibration) deg/s' % (values['gyro_still_deg_s'], values['gyro_coupling_deg_s_per_m_s2']))
    expected_vib = float(np.sqrt(np.mean(vib2)))
    measured_vib = float(np.sqrt(np.mean(np.sum(acc_res ** 2, axis=1))))
    _item(items, 'acceleration vibration RMS while driving vs the profile +-10 %',
          abs(measured_vib - expected_vib) <= .1 * expected_vib, round(measured_vib, 4), round(expected_vib, 4),
          'm/s2, three axes around the true specific force')
    _item(items, 'driving vibration = the profile value of the duty (40 %: below_90)',
          bool(drive_vib) and max(abs(d - expected_drive) for d in drive_vib) < 1e-9,
          round(float(np.median(drive_vib)), 4), expected_drive)
    istamps = np.array(stamps)
    rate = (len(istamps) - 1) / float(istamps[-1] - istamps[0]) if len(istamps) > 1 else 0.
    # the plant draws IMU samples on the 10 ms grid at 33 Hz (30 / 30 / 40 ms): the rate over the driving part
    _item(items, 'IMU rate while driving vs profile +-1 %', abs(rate - imu_rate) <= .01 * imu_rate, round(rate, 3),
          imu_rate)
    sstamps = np.array([m['header'] for m in scans if True])
    moving = [t for t in sstamps if any(abs(t - x) < .06 for x in stamps)]
    dts = np.diff(np.array(moving)) if len(moving) > 1 else np.array([])
    _item(items, 'scans every 0.1 s while driving', len(dts) > 50 and bool(np.all(np.abs(dts - .1) < 1e-6)),
          None if not len(dts) else round(float(np.mean(dts)), 6), .1)
    return dict(check='A5(F) driving', passed=all(i['ok'] for i in items), items=items, samples=len(gyro_res),
                profile_values=values, gyro_residual_mean_rad_s=round(float(np.mean(gyro_res)), 6),
                end_reason=summary['end_reason'], distance_note='straight hold, 40 %% duty, %.0f s' % A5_DRIVE_S)


# --- A8 ----------------------------------------------------------------------------------------------------------------
def a8_check(native_executable=None, build_dir=None):
    """A8: same seed twice equal (serial bytes, trace, executor log), another seed differs, pool of 2 = one worker."""
    S = _sim_in_loop()
    entries = CAT.by_name()
    specs = [dict(entries['normal_zigzag']['spec'], duration_s=12.), dict(entries['mid_row_stop_w0']['spec'])]
    first = [S.run_episode(dict(spec), None, native_executable, build_dir)['digests'] for spec in specs]
    again = [S.run_episode(dict(spec), None, native_executable, build_dir)['digests'] for spec in specs]
    other = [S.run_episode(dict(spec, seed=spec['seed'] + 7), None, native_executable, build_dir)['digests']
             for spec in specs]
    pool = [r['digests'] for r in S.run_episodes([dict(spec) for spec in specs], workers=2,
                                                 native_executable=native_executable, build_dir=build_dir)]
    items = []
    for k, spec in enumerate(specs):
        _item(items, '%s: same seed twice, identical digests' % spec['name'], first[k] == again[k], first[k])
        _item(items, '%s: another seed, different serial log and trace' % spec['name'],
              other[k]['serial_log_sha256'] != first[k]['serial_log_sha256']
              and other[k]['trace_content_sha256'] != first[k]['trace_content_sha256'], other[k])
        _item(items, '%s: process pool of 2 gives the same digests' % spec['name'], pool[k] == first[k], pool[k])
    return dict(check='A8', passed=all(i['ok'] for i in items), items=items)


# --- A13 ---------------------------------------------------------------------------------------------------------------
def a13_check():
    from steering import truth_isolation as TI
    report = TI.check_tree()
    subs = TI.subscription_check()
    items = []
    _item(items, 'truth isolation static check (no error)', report['passed'], len(report['errors']), 0,
          [e for e in report['errors']][:10])
    _item(items, 'executor subscriptions inside the allowlist', subs['passed'], subs['topics'], list(subs['allowlist']),
          subs.get('problems'))
    # the exempted findings are part of the result (review P4R-01): each by its exact (path, function, detail) key
    _item(items, 'no unused exemption key', not report['unused_exemptions'], report['unused_exemptions'], [])
    return dict(check='A13(F)', passed=all(i['ok'] for i in items), items=items, files=report['files'],
                roles=report['roles'], exempted_count=report['exempted_count'],
                exempted=[dict(path=e['path'], line=e['line'], rule=e['rule'], detail=e['detail'], reason=e['reason'])
                          for e in report['exempted']])


# --- the whole run -----------------------------------------------------------------------------------------------------
def _job(item):
    entry, seed, native_executable, build_dir = item
    result, art = run_script(entry, seed, None, native_executable, build_dir)
    return evaluate_script(entry, art, result), _rate_counts(art)


def _rate_counts(art):
    """False-detection counts of one episode: MCU F lines per code, host latches per name, and the episode times the
    RF-03 rate needs (first ACTIVE, end, duration, the MCU THERMAL window thw)."""
    codes = {}
    for text, _, _ in M.mcu_lines(art, 'F'):
        code = text.split('*')[0].split(' ')[1]
        codes[code] = codes.get(code, 0) + 1
    latches = {}
    for name, _, _ in art['latches']:
        latches[name] = latches.get(name, 0) + 1
    s = art['summary']
    return dict(mcu=codes, host=latches, end_reason=s['end_reason'], first_active_at_s=s.get('first_active_at_s'),
                ended_at_s=s.get('ended_at_s'), duration_s=float(art['spec']['duration_s']),
                thermal_window_s=float(art['cfg'].get('thw', 30)))


def thermal_window_filled(counts):
    """RF-03: True when the episode ran its standstill sweeps for at least the MCU THERMAL window (thw s from the first
    ACTIVE tick to the run end) or THERMAL tripped; an episode that ended earlier (e.g. the executor no-response budget
    N9) did not measure THERMAL (review P4R-07 / RS-11)."""
    if counts['mcu'].get('THERMAL'):
        return True
    first = counts.get('first_active_at_s')
    if first is None:
        return False
    end = counts['ended_at_s'] if counts.get('ended_at_s') is not None else counts['duration_s']
    return end - first >= counts['thermal_window_s'] - 1e-9


def run_all(out_dir, only=None, workers=1, repeat=None, native_executable=None, build_dir=None):
    """Run the catalogue and the P4 checks; writes <out_dir>/P4_CHECKS.json and returns the report. only: names,
    groups or check keys (a5, a8, a13, rates); a partial run reports only the checks it ran (verdicts) and complete
    False. workers: process pool size, recorded with the A11 report (run it with 1 and N, SIM_ACCEPTANCE A11)."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    entries = [e for e in CAT.SCRIPTS if only is None or e['name'] in only or e['group'] in only]
    jobs = [(e, None, native_executable, build_dir) for e in entries if e['group'] != 'rf03']
    seeds = CAT.RATE_SEEDS if repeat is None else tuple(range(100, 100 + int(repeat)))
    rate_jobs = []
    if only is None or 'rates' in only:
        for group, names in CAT.RATE_GROUPS.items():
            for name in names:
                entry = CAT.by_name()[name]
                rate_jobs.extend((entry, seed, native_executable, build_dir) for seed in seeds)
    all_jobs = jobs + rate_jobs
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=workers) as pool:
            outcomes = list(pool.map(_job, all_jobs))
    else:
        outcomes = [_job(job) for job in all_jobs]
    scripts = [o[0] for o in outcomes[:len(jobs)]]
    rates = [(job[0]['name'], job[1], o[1]) for job, o in zip(rate_jobs, outcomes[len(jobs):])]
    pairs = []
    by = {s['name']: s for s in scripts}
    for entry in entries:
        other = entry['expect'].get('same_as')
        if other and entry['name'] in by and other in by:
            a, b = by[entry['name']]['digests'], by[other]['digests']
            ok = a['trace_content_sha256'] == b['trace_content_sha256'] and a['serial_log_sha256'] == b['serial_log_sha256']
            pairs.append(dict(item='%s == %s (trace, serial log)' % (entry['name'], other), ok=ok, measured=a, limit=b))
            by[entry['name']]['items'].append(dict(item='same trace and serial log as %s' % other, ok=ok,
                                                   measured=None, limit=None, note=None))
            by[entry['name']]['passed'] = by[entry['name']]['passed'] and ok
    report = dict(schema=SCHEMA, result_kind=RESULT_KIND, field_validated=False, scripts=scripts, pairs=pairs,
                  rates=rate_table(rates) if rate_jobs else None, workers=int(workers),
                  only=None if only is None else sorted(only), complete=only is None)
    if only is None or 'a5' in only:
        report['A5'] = a5_check(native_executable, build_dir)
    if only is None or 'a8' in only:
        report['A8'] = a8_check(native_executable, build_dir)
    if only is None or 'a13' in only:
        report['A13'] = a13_check()
    # the script tables only when scripts ran (a partial run without scripts gives no A17 / A19 / A22; review P4R-10);
    # A19 also needs the rate runs (its 0-false-detection rule covers every rate seed)
    report['A17'] = a17_table(scripts) if scripts else None
    report['A19'] = a19_table(scripts, report['rates']) if scripts and rate_jobs else None
    report['A22'] = a22_table(scripts) if scripts else None
    report['A11'] = a11_table(scripts, workers) if scripts else None
    report['wall_s'] = round(time.perf_counter() - started, 1)
    verdicts = {key: report[key]['passed'] for key in ('A5', 'A8', 'A13', 'A17', 'A19', 'A22')
                if report.get(key) is not None and report[key].get('passed') is not None}
    verdicts['scripts'] = dict(passed=sum(1 for s in scripts if s['passed']), total=len(scripts),
                               failed=[s['name'] for s in scripts if not s['passed']])
    report['verdicts'] = verdicts
    report['all_pass'] = all(v for k, v in verdicts.items() if k != 'scripts') and not verdicts['scripts']['failed']
    raw = json.dumps(report, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False, default=_json_default) + '\n'
    (out / 'P4_CHECKS.json').write_bytes(raw.encode('utf-8'))
    return report


def _json_default(value):
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, np.ndarray):
        return value.tolist()
    return str(value)


def rate_table(rates):
    groups = {}
    false_normal = []
    for name, seed, counts in rates:
        if name in CAT.RATE_GROUPS['normal']:
            codes = sorted(code for code in counts['mcu'] if code != 'NOCAL')
            latches = sorted(counts['host'])
            if codes or latches:
                false_normal.append(dict(script=name, seed=seed, mcu=codes, host=latches, end=counts['end_reason']))
        g = groups.setdefault(name, dict(episodes=0, stall=0, ineffective=0, thermal=0, thermal_measured=0,
                                         other_faults={}, ends={}))
        g['episodes'] += 1
        g['stall'] += 1 if counts['mcu'].get('STALL') else 0
        g['ineffective'] += 1 if counts['host'].get('steering_ineffective') else 0
        g['thermal'] += 1 if counts['mcu'].get('THERMAL') else 0
        g['thermal_measured'] += 1 if 'thermal_window_s' in counts and thermal_window_filled(counts) else 0
        for code, n in counts['mcu'].items():
            if code not in ('STALL', 'THERMAL'):
                g['other_faults'][code] = g['other_faults'].get(code, 0) + n
        end = str(counts['end_reason'])
        g['ends'][end] = g['ends'].get(end, 0) + 1
    normal = [g for n, g in groups.items() if n in CAT.RATE_GROUPS['normal']]
    episodes = sum(g['episodes'] for g in normal)
    thermal = {}
    for n, g in groups.items():
        if n not in CAT.RATE_GROUPS['rf03']:
            continue
        measured = g['thermal_measured']
        # an episode that ended before the THERMAL window could fill is not a measurement (review P4R-07 / RS-11)
        thermal[n] = dict(rate=None if not measured else round(g['thermal'] / measured, 4), thermal=g['thermal'],
                          measured_episodes=measured, not_measured=g['episodes'] - measured, episodes=g['episodes'],
                          ends=g['ends'],
                          note=None if measured == g['episodes'] else
                          'not measured: %d of %d episodes ended before the %s THERMAL window (thw) could fill (see ends)'
                          % (g['episodes'] - measured, g['episodes'], 'MCU'))
    return dict(per_script=groups, normal_episodes=episodes, false_detections_normal=false_normal,
                false_detection_rate_normal=None if not episodes else round(len(false_normal) / episodes, 4),
                stall_false_rate=None if not episodes else round(sum(g['stall'] for g in normal) / episodes, 4),
                ineffective_false_rate=None if not episodes else round(sum(g['ineffective'] for g in normal) / episodes, 4),
                thermal_rf03=thermal)


def _collect(scripts, words):
    rows = []
    for s in scripts:
        for i in s['items']:
            if any(w in i['item'] for w in words):
                rows.append(dict(script=s['name'], group=s['group'], **i))
    return rows


def a17_table(scripts):
    words = ('STOP', 'TTL', 'watchdog', 'fault -> steering', 'hold error', 'automatic restart', 'lateral', 'second client',
             'late drive', 'same trace', 'run not ended', 'end reason', 'bridge process', 'MCU reset when the port')
    rows = _collect([s for s in scripts if s['group'] in ('safety', 'mandatory', 'fault', 'host', 'normal')], words)
    mandatory = [dict(script=s['name'], passed=s['passed']) for s in scripts if s['group'] == 'mandatory']
    judged = [r for r in rows if r['ok'] is not None]
    by = {s['name']: s for s in scripts}
    # the 09-19 mandatory list and the scripts standing for each item (audit AUD3A-19)
    mapping = [dict(item=item, scripts=[dict(script=n, passed=by[n]['passed']) for n in names if n in by],
                    passed=all(by[n]['passed'] for n in names if n in by) if any(n in by for n in names) else None)
               for item, names in MANDATORY_MAP]
    # STOP timing per script: the request phase on the bridge grid and the status line phase (audit AUD3A-09)
    phases = []
    for s in scripts:
        stop = s['facts'].get('stop')
        if not stop or not stop.get('found'):
            continue
        unconfirmed = any(i['item'].startswith('STOP resends') for i in s['items'])
        phases.append(dict(script=s['name'], request_ms=stop['request_ms'], sent_ms=stop['sent_ms'],
                           send_delay_ms=stop['send_delay_ms'], confirm_delay_ms=stop.get('confirm_delay_ms'),
                           parse_to_zero_ticks=stop.get('parse_to_zero_ticks'), unconfirmed_script=unconfirmed))
    delays = [p['send_delay_ms'] for p in phases]
    confirms = [p['confirm_delay_ms'] for p in phases if p['confirm_delay_ms'] is not None
                and not p['unconfirmed_script']]
    # no judged row and no mandatory script: nothing was measured (None, review P4R-10)
    passed = None if not judged and not mandatory else (_judged(rows) and all(m['passed'] for m in mandatory)
                                                        and all(m['passed'] is not False for m in mapping))
    return dict(check='A17(F)', passed=passed, rows=rows, mandatory=mandatory, mandatory_map=mapping,
                stop_phases=dict(rows=phases, worst_send_delay_ms=max(delays) if delays else None,
                                 worst_confirm_delay_ms=max(confirms) if confirms else None,
                                 note='the rig runs host code on the 10 ms plant grid: STOP requests come at executor '
                                      'ticks (50 ms) and the bridge steps every 20 ms, so the request phases are 0 and '
                                      '10 ms; a request between two plant steps is not representable'),
                not_applicable=[dict(script=r['script'], item=r['item'], note=r['note']) for r in rows
                                if r['ok'] is None])


def a19_table(scripts, rates):
    codes = {}
    for s in scripts:
        for f in (s['facts'].get('faults') or []):
            if f['found']:
                entry = codes.setdefault(f['code'], dict(scripts=[], ok=False))
                diff = None if f.get('expected_tick') is None else f['tick'] - f['expected_tick']
                entry['scripts'].append(dict(script=s['name'], tick=f['tick'], expected_tick=f.get('expected_tick'),
                                             diff_ticks=diff, method=f.get('method'), script_passed=s['passed']))
                entry['ok'] = entry['ok'] or (s['passed'] and diff is not None and abs(diff) <= 1)
        for key, code in (('ttl', 'TTL'), ('watchdog', 'WATCHDOG')):
            fact = s['facts'].get(key)
            tick = None if not fact else fact.get('f_tick', fact.get('w_tick'))
            if fact and tick is not None:
                entry = codes.setdefault(code, dict(scripts=[], ok=False))
                diff = None if fact.get('expected') is None else tick - fact['expected']
                entry['scripts'].append(dict(script=s['name'], tick=tick, expected_tick=fact.get('expected'),
                                             diff_ticks=diff, script_passed=s['passed']))
                entry['ok'] = entry['ok'] or (s['passed'] and diff is not None and abs(diff) <= 1)
    host = {}
    for s in scripts:
        for i in s['items']:
            if i['item'].startswith('host latch ') and i['ok']:
                name = i['item'][len('host latch '):]
                host.setdefault(name, []).append(s['name'])
    # latch time against the design rule and the resulting state (audit AUD3A-01): the scripts that expect the latch
    latch_times = {}
    for s in scripts:
        expected = set()
        for i in s['items']:
            if i['item'].startswith('host latch ') and i['ok']:
                expected.add(i['item'][len('host latch '):])
        for row in s['facts'].get('latches') or ():
            if row['name'] in expected:
                latch_times.setdefault(row['name'], []).append(dict(
                    script=s['name'], latch_ms=row['latch_ms'], design_ms=row['design_ms'], diff_ms=row['diff_ms'],
                    method=row['method'], time_ok=row['time_ok'], lease_at_latch=row['lease_at_latch'],
                    state_ok=row['state_ok']))
    latch_time_bad = sorted(name for name, rows in latch_times.items()
                            if not any(r['time_ok'] and r['state_ok'] for r in rows))
    states = {}
    for s in scripts:
        for i in s['items']:
            if ' -> ' in i['item'] or 'latch bit kept' in i['item']:
                head = i['item'].split(' ')[0]
                if head in STEER_FAULT_CODES or head == 'watchdog':
                    states.setdefault(head.upper(), []).append(dict(script=s['name'], item=i['item'], ok=i['ok'],
                                                                    measured=i['measured']))
    state_bad = sorted(code for code, rows in states.items() if any(r['ok'] is False for r in rows))
    normal = [dict(script=s['name'], passed=s['passed']) for s in scripts if s['group'] == 'normal']
    missing = [c for c in STEER_FAULT_CODES if c not in codes or not codes[c]['ok']]
    host_missing = [h for h in HOST_LATCHES if h not in host and h not in HOST_LATCHES_NOT_IN_T1]
    # 0 false detections: the normal scripts (catalogue seed) and every seed of the false-detection rate runs
    false_normal = list((rates or {}).get('false_detections_normal') or [])
    passed = (not missing and not host_missing and all(n['passed'] for n in normal) and not false_normal
              and not latch_time_bad and not state_bad)
    return dict(check='A19(F)', passed=passed, codes=codes, missing_codes=missing, host_latches=host,
                host_missing=host_missing, host_not_in_t1=HOST_LATCHES_NOT_IN_T1, host_latch_times=latch_times,
                host_latch_time_or_state_failed=latch_time_bad, result_states=states,
                result_state_failed=state_bad, normal_scripts=normal,
                false_detections_normal=false_normal, false_detection_rates=rates)


def a22_table(scripts):
    rows = _collect(scripts, ('A22',))
    return dict(check='A22(F)', passed=None if not rows else _judged(rows), rows=rows)


def a11_table(scripts, workers=1):
    steps = [s['timing']['step_ms'] for s in scripts if s['timing']['step_ms']['n']]
    mcu = [s['timing']['mcu_step_ms'] for s in scripts if s['timing']['mcu_step_ms']['n']]

    def worst(rows, key):
        values = [r[key] for r in rows if r.get(key) is not None]
        return None if not values else round(max(values), 4)

    def median(rows, key):
        values = [r[key] for r in rows if r.get(key) is not None]
        return None if not values else round(float(np.median(values)), 4)

    return dict(check='A11 (report)', episodes=len(scripts), workers=int(workers),
                step_ms=dict(p50_median=median(steps, 'p50'), p99_median=median(steps, 'p99'),
                             p99_worst=worst(steps, 'p99'), p999_worst=worst(steps, 'p999'), max_worst=worst(steps, 'max')),
                mcu_step_ms=dict(p50_median=median(mcu, 'p50'), p99_median=median(mcu, 'p99'),
                                 p999_worst=worst(mcu, 'p999'), mean_median=median(mcu, 'mean')),
                wall_s=[dict(script=s['name'], wall_s=s['wall_s']) for s in scripts],
                note='wall-clock values on this PC (shared with other work): report only')
