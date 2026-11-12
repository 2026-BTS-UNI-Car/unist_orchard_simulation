"""Design times and resulting states of the safety script outcomes (SIM_ACCEPTANCE A19 'judgement time and resulting
state are the design values'; audit AUD3A-01, AUD3A-08).

Host latches (HOST_BRIDGE_V3_KO.md 3, 4): latch_facts(entry, art) recomputes, for every host latch an episode raised,
the time the design rule gives from the episode's own records (the serial log: when each MCU line became readable and
each host line was written; the host bridge loop grid; the trace's host view; the executor's motion samples), and
judges the latch time against it (+- one bridge loop period) and the resulting state (the host stopped in the latch
step when it held a lease: STOP written then, no lease and the 'fault' state afterwards, no C line after the latch).

    steering_status_stale   first loop step T with T - (step that read the last status line) > status_gap_s
    status_seq_gap          the step that read the status line whose missing numbers push the 1 s window sum over
                            seq_gap_missing
    steering_not_following  the step that read the status line ending track_duration_s of |requested - measured| >
                            track_error (lease, ARM confirmed, st ARMED; requested = the host request before that step),
                            counted only on lines whose servo ramp has settled (cmd == tgt clipped to the calibration soft
                            limits; amendment 43, steer_bridge_core._track_monitor)
    steering_ineffective    the same with |gyro - v tan(delta_pot) / L| > max(rate_error, frac |pred|) for duration_s
                            (lease holder's op motion samples, at most MOTION_FRESH_S old; v > min_speed, |delta| >
                            min_steer)
    arm_unconfirmed         first loop step T with T - (ARM written) > arm.confirm_ms without an ARMED status line
    stop_unconfirmed        t0 + confirm + (resend_max - 1) x interval + confirm on the loop grid (amendment 7)
    mcu_disarmed            the step that read the first non-ARMED status line after the ARM confirmation
    mcu_state_mismatch      the step that read the first status line with st != DISARMED while the host had no
                            lease, no pending ARM and no pending STOP
    mcu_reset_detected      the step that read the first line of the new boot (banner, ms back, n restart)
    c_rejected, arm_rejected  the step that read the N line
    calibration_mismatch, hardware_profile_mismatch, sim_identity_mismatch  the step that read the banner
    hardware_fault          the first loop step at or after the transport failure (USB unplug)
    internal_error          the first loop step at or after the injected defect
    mcu_ttl_expired         the step that read the F TTL line (R18 decision 1)

MCU fault codes (V3 spec 6.4, 6.8): mcu_result_items(items, code, art, tick, item) adds the resulting state of the rows
the old check left out: CRC (brake, steering HOLD of the measured position for hloss then OFF, DISARMED, CRC latch bit
until the next ARM), TTL (brake, steering keeps its target with the servo on, still ARMED, TTL bit), WATCHDOG (brake,
HOLD for hloss then OFF with wdmode hold, DISARMED, WATCHDOG latch bit until the next ARM; A17 'watchdog -> latch'),
BOOT (coast: drive EN 0 while the MCU is not running, DISARMED after the boot delay, BOOT bit, a banner with a new
boot id), SOFTLIMIT (no drive or state change, no F line, the condition bit), NOCAL (DISARMED, NOCAL bit, no F line at
boot) and, for the latched codes of the old check, the latch bit kept until the next ARM.

Evaluation code (A13 role 'evaluator'). Placeholder values only (자리값 기반 SIM 결과).
"""
from __future__ import annotations

import math

import numpy as np

from . import measure as M

MOTION_FRESH_S = .2        # steer_bridge_core.MOTION_FRESH_S
LATCHED_CODES = ('POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'STALL', 'TRACK',
                 'THERMAL', 'WATCHDOG', 'CRC')        # spec 6.8: the bits an accepted ARM clears (NOCAL at boot)
MODE_OFF, MODE_ACTIVE, MODE_HOLD = 0, 1, 2
STATE_BOOT, STATE_DISARMED, STATE_ARMED, STATE_OFF = 0, 1, 2, 255
HOST_FAULT_STATE = 3       # t1_rig.HOST_STATE_CODES['fault']


# --- the host loop grid ---------------------------------------------------------------------------------------------------
def _seconds(t):
    return int(round(float(t) * 1000.))


def _blocked_windows(art):
    """(t0, t1) ms windows without a host loop step: bridge stalls (rig fault bridge_stall) and bridge_restart outages."""
    out = []
    for f in art['spec']['faults']:
        if f['kind'] == 'bridge_stall':
            a = _seconds(f['at_s'])
            out.append((a, a + _seconds((f.get('params') or {}).get('duration_s', 0.))))
    exits = [t for kind, t in art.get('bridge_events') or () if kind == 'exit']
    starts = [t for kind, t in art.get('bridge_events') or () if kind == 'start']
    for a in exits:
        later = [b for b in starts if b > a]
        out.append((a, later[0] if later else 10 ** 9))
    return out


def loop_step_at_or_after(art, t_ms):
    """The first host loop step at or after t_ms (bridge_phase_ms + k x bridge_period_ms, not in a stall / outage)."""
    period = int(art['host']['bridge_period_ms'])
    phase = int(art['host'].get('bridge_phase_ms', 0))
    t = int(math.ceil(float(t_ms)))
    k = max(0, -(-(t - phase) // period))
    step = phase + k * period
    blocked = _blocked_windows(art)
    for _ in range(100000):
        inside = [b for a, b in blocked if a <= step < b]
        if not inside:
            return step
        k = max(k + 1, -(-(max(inside) - phase) // period))
        step = phase + k * period
    return None


def loop_step_after(art, t_ms):
    """The first host loop step strictly after t_ms."""
    return loop_step_at_or_after(art, int(math.floor(t_ms)) + 1)


# --- records ---------------------------------------------------------------------------------------------------------------
def readable_lines(art, prefix=None):
    """MCU lines the host could read, in order: dict(t_ms readable, queued, read_ms, text)."""
    out = []
    for r in art['serial']:
        if r['dir'] != 'm2h' or r['note'] not in ('', 'garbage+line'):
            continue
        if prefix is not None and not r['text'].startswith(prefix):
            continue
        out.append(dict(t_ms=r['t_ms'], queued=r['first_tick'], read_ms=loop_step_at_or_after(art, r['t_ms']),
                        text=r['text']))
    return out


def status_lines(art):
    """Readable status lines with their fields and the loop step that read them."""
    out = []
    for line in readable_lines(art, 'S '):
        parts = line['text'].split('*')[0].split(' ')
        fields = {}
        for name, token in zip(M.STATUS_FIELDS, parts[1:]):
            try:
                fields[name] = int(token, 16) if name == 'fl' else int(token)
            except ValueError:
                fields[name] = None
        line['fields'] = fields
        out.append(line)
    return out


def _row_at(art, t_ms):
    """Index of the trace row recorded at t_ms (the end of the plant step [t_ms - 10, t_ms)): the host view after the
    last loop step before t_ms."""
    t = art['trace']['t_s']
    k = int(np.searchsorted(t, t_ms / 1000. - 1e-9))
    return min(max(k, 0), len(t) - 1)


def _trace(art, name, t_ms, default=None):
    column = art['trace'].get(name)
    if column is None or not len(column):
        return default
    return column[_row_at(art, t_ms)]


def _mrad_from_permille(art, permille):
    host = art['host']
    if permille is None:
        return None
    if permille >= 0:
        return int(math.floor(abs(permille * host['mark_left_mrad'] / 1000.) + .5))
    return -int(math.floor(abs(-permille * host['mark_right_mrad'] / 1000.) + .5))


def _last_before(values, t_ms, key='t_ms'):
    found = [v for v in values if v[key] <= t_ms]
    return found[-1] if found else None


# --- design times of the host latches -------------------------------------------------------------------------------------
def design_latch_ms(name, art, latch_ms):
    """(design time ms or None, method text) of a host latch raised at latch_ms (module docstring)."""
    host = art['host']
    if name == 'steering_status_stale':
        lines = [s for s in status_lines(art) if s['read_ms'] is not None and s['read_ms'] < latch_ms]
        if not lines:
            return None, 'no status line before the latch'
        last = lines[-1]['read_ms']
        gap = int(round(host['status_gap_s'] * 1000.))
        return loop_step_after(art, last + gap), 'first loop step > read of the last status line (%d) + %d ms' % (
            last, gap)
    if name == 'status_seq_gap':
        window_ms = int(round(host['seq_gap_window_s'] * 1000.))
        prev, entries = None, []
        for s in status_lines(art):
            n = s['fields'].get('n')
            if prev is not None and n is not None:
                dn = (n - prev) % 65536
                if 1 <= dn <= 32767 and dn > 1:
                    entries.append((s['read_ms'], dn - 1))
                elif not 1 <= dn <= 32767:
                    entries = []
            prev = n
            t = s['read_ms']
            entries = [(a, m) for a, m in entries if t - a <= window_ms]
            if sum(m for _, m in entries) > host['seq_gap_missing']:
                return t, 'read of the status line that makes the missing numbers in %d ms exceed %d' % (
                    window_ms, host['seq_gap_missing'])
        return None, 'the missing numbers never exceed the limit'
    if name in ('steering_not_following', 'steering_ineffective'):
        return _monitor_design_ms(name, art, latch_ms)
    if name == 'arm_unconfirmed':
        arms = [r for r in M.h2m(art, 'ARM') if r['t_ms'] < latch_ms]
        if not arms:
            return None, 'no ARM line before the latch'
        t_arm = arms[-1]['t_ms']
        confirm = int(round(host['arm_confirm_s'] * 1000.))
        armed = [s for s in status_lines(art) if s['read_ms'] is not None and s['read_ms'] >= t_arm
                 and s['fields'].get('st') == STATE_ARMED]
        if armed and armed[0]['read_ms'] <= t_arm + confirm:
            return None, 'an ARMED status line was read in time'
        return loop_step_after(art, t_arm + confirm), 'first loop step > ARM written (%d) + %d ms' % (t_arm, confirm)
    if name == 'stop_unconfirmed':
        stops = M.h2m(art, 'STOP')
        firsts = [r['t_ms'] for r in stops if r['t_ms'] <= latch_ms]
        if not firsts:
            return None, 'no STOP before the latch'
        _, offset = M.resend_grid(art)
        # t0: the first STOP of the unconfirmed series (the last STOP that was written more than 400 ms earlier)
        candidates = [t for t in firsts if t + offset >= latch_ms - host['bridge_period_ms']]
        t0 = candidates[0] if candidates else firsts[-1]
        return loop_step_at_or_after(art, t0 + offset), 'first STOP (%d) + %d ms on the loop grid' % (t0, offset)
    if name == 'mcu_disarmed':
        arms = [r['t_ms'] for r in M.h2m(art, 'ARM') if r['t_ms'] < latch_ms]
        if not arms:
            return None, 'no ARM line before the latch'
        lines = [s for s in status_lines(art) if s['read_ms'] is not None and s['read_ms'] >= arms[-1]]
        confirmed = next((s for s in lines if s['fields'].get('st') == STATE_ARMED), None)
        if confirmed is None:
            return None, 'no ARM confirmation'
        after = next((s for s in lines if s['read_ms'] >= confirmed['read_ms'] and s['queued'] > confirmed['queued']
                      and s['fields'].get('st') != STATE_ARMED), None)
        return (None, 'no non-ARMED status line') if after is None else (
            after['read_ms'], 'read of the first non-ARMED status line after the ARM confirmation')
    if name == 'mcu_state_mismatch':
        for s in status_lines(art):
            if s['fields'].get('st') in (STATE_ARMED, 3):
                lease = _trace(art, 'host_lease', s['read_ms'], 0)
                if not lease:
                    return s['read_ms'], 'read of the first ARMED / BENCH status line without a host lease'
        return None, 'no such status line'
    if name == 'mcu_reset_detected':
        reset = _reset_tick(art)
        if reset is None:
            return None, 'no reset in the script'
        lines = [line for line in readable_lines(art) if line['queued'] is not None and line['queued'] >= reset]
        return (None, 'no line of the new boot') if not lines else (
            lines[0]['read_ms'], 'read of the first line of the new boot (reset at %d)' % reset)
    if name == 'c_rejected':
        lines = [line for line in readable_lines(art, 'N ') if line['text'].split(' ')[1][:1].isdigit()]
        return (None, 'no N <q> line') if not lines else (lines[0]['read_ms'], 'read of the first N <q> line')
    if name == 'arm_rejected':
        arms = [r['t_ms'] for r in M.h2m(art, 'ARM') if r['t_ms'] <= latch_ms]
        lines = [line for line in readable_lines(art, 'N - ') if arms and line['t_ms'] >= arms[-1]]
        return (None, 'no N - line after the ARM') if not lines else (
            lines[0]['read_ms'], 'read of the N - line answering the ARM')
    if name in ('calibration_mismatch', 'hardware_profile_mismatch', 'sim_identity_mismatch'):
        banners = readable_lines(art, 'UBTS_STEER_V3 hello')
        return (None, 'no banner') if not banners else (banners[0]['read_ms'], 'read of the session banner')
    if name in ('hardware_fault', 'internal_error'):
        kind = 'usb_unplug' if name == 'hardware_fault' else 'host_defect'
        times = [_seconds(f['at_s']) for f in art['spec']['faults'] if f['kind'] == kind]
        if not times:
            return None, 'no %s fault' % kind
        return loop_step_at_or_after(art, min(times)), 'first loop step at or after the %s fault' % kind
    if name == 'mcu_ttl_expired':
        arms = [r['t_ms'] for r in M.h2m(art, 'ARM') if r['t_ms'] < latch_ms]
        lines = [line for line in readable_lines(art, 'F TTL') if arms and line['t_ms'] > arms[-1]]
        return (None, 'no F TTL line') if not lines else (lines[0]['read_ms'], 'read of the F TTL line')
    return None, 'no design rule here for %s' % name


def _reset_tick(art):
    for f in art['spec']['faults']:
        if f['kind'] in ('mcu_reset', 'power_cut', 'port_open_reset'):
            return _seconds(f['at_s'])
    for kind, t in art.get('bridge_events') or ():
        if kind == 'start':
            return t
    return None


def _monitor_design_ms(name, art, latch_ms):
    """Replay of the host's steering_not_following / steering_ineffective rule on the status lines it read."""
    host = art['host']
    since = None
    motion = sorted(art.get('motion') or ())
    for s in status_lines(art):
        t = s['read_ms']
        if t is None or t > latch_ms + 2 * host['bridge_period_ms']:
            break
        f = s['fields']
        lease = _trace(art, 'host_lease', t, 0)
        armed = _trace(art, 'host_arm_state', t, 0) == 2
        ok = bool(lease) and armed and f.get('st') == STATE_ARMED
        measured = _mrad_from_permille(art, f.get('pos'))
        if name == 'steering_not_following':
            requested = _trace(art, 'cmd_steer_target_mrad', t, float('nan'))
            cal = art.get('cal') or {}
            goal = f.get('tgt')
            if goal is not None and cal.get('soft_l') is not None and cal.get('soft_r') is not None:
                goal = min(cal['soft_l'], max(cal['soft_r'], goal))
            settled = f.get('cmd') is not None and f.get('cmd') == goal
            condition = ok and settled and requested is not None and np.isfinite(requested) and measured is not None \
                and abs(float(requested) - measured) > host['track_error_mrad']
            duration = host['track_duration_s']
        else:
            cfg = host['ineffective']
            sample = None
            for item in motion:
                if item[0] * 1000. < t - 1e-6:
                    sample = item
            condition = False
            if ok and sample is not None and t / 1000. - sample[0] <= MOTION_FRESH_S + 1e-9 and measured is not None:
                delta = measured / 1000.
                yaw, speed = sample[1], sample[2]
                if speed > cfg['min_speed_m_s'] and abs(delta) > math.radians(cfg['min_steer_deg']):
                    predicted = speed * math.tan(delta) / host['wheelbase_m']
                    limit = max(math.radians(cfg['rate_error_deg_s']), cfg['rate_error_frac'] * abs(predicted))
                    condition = abs(yaw - predicted) > limit
            duration = cfg['duration_s']
        if not condition:
            since = None
            continue
        if since is None:
            since = t
        elif (t - since) / 1000. >= duration - 1e-9:
            return t, 'read of the status line ending %.2f s of the condition (from %d)' % (duration, since)
    return None, 'the condition never lasted %s s in the replay' % name


# --- the resulting state of a host latch ---------------------------------------------------------------------------------
def latch_state_items(name, art, latch_ms, end_ms=None):
    """Resulting state after a host latch at latch_ms: with a lease, the STOP of the latch step, then no lease, the
    'fault' state and no C line until end_ms (an operator re-arm, else the end of the record)."""
    items = []
    tr = art['trace']
    k = _row_at(art, latch_ms)                # the row before the latch step: the host view the latch step started with
    leased = bool(tr['host_lease'][k]) if 'host_lease' in tr and len(tr['host_lease']) else False
    end = end_ms if end_ms is not None else _seconds(tr['t_s'][-1]) + 1
    rows = (tr['t_s'] * 1000. > latch_ms + 1e-6) & (tr['t_s'] * 1000. <= end)
    lease_after = int(np.max(tr['host_lease'][rows])) if 'host_lease' in tr and rows.any() else 0
    state_after = sorted(set(int(v) for v in tr['host_state'][rows])) if 'host_state' in tr and rows.any() else []
    stop_written = [r['t_ms'] for r in M.h2m(art, 'STOP', since_ms=latch_ms) if r['t_ms'] <= latch_ms]
    cs = [r['t_ms'] for r in M.h2m(art, 'C', since_ms=latch_ms + 1) if r['t_ms'] < end]
    if leased and name == 'hardware_fault':
        # the transport failed: the host closes the link and writes nothing; the session STOP follows the reopen
        later = [r['t_ms'] for r in M.h2m(art, 'STOP', since_ms=latch_ms)][:1]
        items.append(dict(item='session STOP after the link reopens', ok=bool(later), measured=later, limit=None))
    elif leased:
        items.append(dict(item='STOP in the latch step', ok=bool(stop_written), measured=stop_written, limit=latch_ms))
    items.append(dict(item='no lease after the latch', ok=lease_after == 0, measured=lease_after, limit=0))
    items.append(dict(item="host state 'fault' after the latch", ok=state_after == [HOST_FAULT_STATE],
                      measured=state_after, limit=[HOST_FAULT_STATE]))
    items.append(dict(item='no C line after the latch', ok=not cs, measured=len(cs), limit=0))
    return leased, items


def latch_facts(entry, art):
    """Every host latch the episode raised (first occurrence): design time, latch time, the +- one loop period item and
    the resulting state items (latch_state_items)."""
    out = []
    seen = set()
    period = art['host']['bridge_period_ms']
    rearm = entry['expect'].get('operator_rearm_at')
    for name, since_s, detail in art['latches']:
        if name in seen or since_s is None:
            continue
        seen.add(name)
        latch_ms = _seconds(since_s)
        design, method = design_latch_ms(name, art, latch_ms)
        diff = None if design is None else latch_ms - design
        end = None
        if rearm is not None and _seconds(rearm) > latch_ms:
            end = _seconds(rearm)
        leased, state = latch_state_items(name, art, latch_ms, end)
        out.append(dict(name=name, latch_ms=latch_ms, design_ms=design, diff_ms=diff, method=method,
                        time_ok=None if design is None else abs(diff) <= period, lease_at_latch=leased,
                        state_items=state, state_ok=all(i['ok'] for i in state), detail=detail))
    return out


# --- the resulting state of an MCU fault code (spec 6.4, 6.8) --------------------------------------------------------------
def _value(art, channel, tick, default=0):
    return M.value_at(art['channels'][channel], tick, default)


def _first_arm_after(art, tick):
    """The arrival tick of the first ARM line the MCU accepted after tick (None: none)."""
    for r in M.accepted_inputs(art, ('ARM',), 10 ** 9):
        if r['last_tick'] is not None and r['last_tick'] > tick:
            return r['last_tick']
    return None


def _first_reset_after(art, tick):
    """The first tick after tick at which the MCU does not run (state BOOT or OFF: a reset or a power cut, e.g. the
    NodeMCU reset when the host opens the port again, spec 10.7). The reset clears the RAM latches and the steering
    HOLD; the new boot reports BOOT. None: no reset."""
    return M.first_tick(art['channels']['mcu_state'], lambda v: v in (STATE_BOOT, STATE_OFF), tick + 1,
                        default=STATE_DISARMED)


def _bit_kept(art, code, tick):
    """(kept, until) for a latch bit: set from the judgement tick until the next accepted ARM or the next MCU reset
    (the reset clears the RAM latch, _first_reset_after), or the end."""
    ends = [t for t in (_first_arm_after(art, tick), _first_reset_after(art, tick)) if t is not None]
    until = min(ends) if ends else None
    bit = M.bit(code)
    changes = art['channels']['fault_bits']
    end = until if until is not None else (changes[-1][0] if changes else tick) + 1
    if not _value(art, 'fault_bits', tick + 1) & bit:
        return False, until
    for t, v in changes:
        if tick + 1 < t < end and not v & bit:
            return False, until
    return True, until


def mcu_result_items(items, code, art, tick, fault_tick_kind):
    """The resulting state items of spec 6.4 for one MCU code judged at tick (module docstring); fault_tick_kind:
    'F' (an F line), 'W' (a W line), 'bit' (a status bit: NOCAL, BOOT, SOFTLIMIT)."""
    cfg = art['cfg']

    def add(name, ok, measured=None, limit=None, note=None):
        items.append(dict(item=name, ok=None if ok is None else bool(ok), measured=measured, limit=limit, note=note))

    if code in ('CRC', 'WATCHDOG'):
        zero = M.drive_zero_tick(art, tick)
        add('%s -> drive brake (0 within 1 tick)' % code, zero is not None and zero - tick <= 1,
            None if zero is None else zero - tick, 1)
        mode = _value(art, 'steer_mode', tick)
        hloss = int(cfg.get('hloss', 2000))
        was_active = _value(art, 'steer_mode', tick - 1) == MODE_ACTIVE
        if code == 'WATCHDOG' and int(cfg.get('wdmode', 0)) == 2:
            add('WATCHDOG -> steering OFF (wdmode free)', mode == MODE_OFF, mode, MODE_OFF)
        elif was_active or code == 'WATCHDOG':
            off = M.first_tick(art['channels']['steer_mode'], lambda v: v != MODE_HOLD, tick + 1)
            # an MCU reset inside the hloss window ends the HOLD at the reset (the RAM state is gone, pins OFF)
            reset = _first_reset_after(art, tick)
            end = tick + hloss if reset is None or reset >= tick + hloss else reset
            add('%s -> steering HOLD (measured position) for hloss, then OFF' % code,
                mode == MODE_HOLD and off is not None and abs(off - end) <= 1
                and _value(art, 'steer_mode', off) == MODE_OFF,
                dict(mode=mode, off_tick=off), dict(mode=MODE_HOLD, off_tick=end, reset_tick=reset))
        else:
            add('%s -> steering unchanged (was not ACTIVE)' % code, None, mode, note='not applicable')
        state = _value(art, 'mcu_state', tick + 1, STATE_OFF)
        add('%s -> MCU DISARMED' % code, state == STATE_DISARMED, M.MCU_STATE_NAMES.get(state), 'DISARMED')
        kept, until = _bit_kept(art, code, tick)
        add('%s latch bit kept until the next ARM' % code, kept, dict(until_arm_tick=until))
        return
    if code == 'TTL':
        zero = M.drive_zero_tick(art, tick)
        mode = int(cfg.get('ttlmode', 0))
        if mode == 0:
            add('TTL -> drive brake (0 within 1 tick)', zero is not None and zero - tick <= 1,
                None if zero is None else zero - tick, 1)
        steer_mode = _value(art, 'steer_mode', tick)
        en = _value(art, 'steer_en', tick)
        add('TTL -> steering keeps its target (servo ACTIVE, EN 1)', steer_mode == MODE_ACTIVE and en == 1,
            dict(mode=steer_mode, en=en), dict(mode=MODE_ACTIVE, en=1))
        state = _value(art, 'mcu_state', tick, STATE_OFF)
        add('TTL -> MCU stays ARMED', state == STATE_ARMED, M.MCU_STATE_NAMES.get(state), 'ARMED')
        add('TTL -> TTL condition bit', bool(_value(art, 'fault_bits', tick) & M.bit('TTL')),
            '%04X' % _value(art, 'fault_bits', tick))
        return
    if code == 'BOOT':
        reset = M.first_tick(art['channels']['mcu_state'], lambda v: v in (STATE_BOOT, STATE_OFF), max(0, tick - 10000))
        if reset is None:
            add('BOOT -> reset seen', False, None)
            return
        en = _value(art, 'drive_en', reset)
        add('BOOT -> drive coast (EN 0 while the MCU does not run)', en == 0, en, 0)
        setup = M.first_tick(art['channels']['mcu_state'], lambda v: v == STATE_DISARMED, reset)
        add('BOOT -> DISARMED after the boot delay', setup is not None and abs(setup - (reset + art['boot_delay_ticks']))
            <= 1, None if setup is None else setup - reset, art['boot_delay_ticks'])
        banners = [line for line in readable_lines(art, 'UBTS_STEER_V3 hello') if line['queued'] is not None
                   and line['queued'] >= reset]
        boots = [line['text'].split(' boot=')[1].split(' ')[0] for line in readable_lines(art, 'UBTS_STEER_V3 hello')
                 if ' boot=' in line['text']]
        add('BOOT -> banner with a new boot id', bool(banners) and len(set(boots)) >= 2, boots[-2:])
        return
    if code == 'SOFTLIMIT':
        before = _value(art, 'mcu_state', tick - 1, STATE_OFF)
        after = _value(art, 'mcu_state', tick + 1, STATE_OFF)
        add('SOFTLIMIT -> MCU state unchanged', before == after, [M.MCU_STATE_NAMES.get(before),
                                                                 M.MCU_STATE_NAMES.get(after)])
        lines = [t for t, q, _ in M.mcu_lines(art, 'F', max(0, tick - 5)) if 'SOFTLIMIT' in t]
        add('SOFTLIMIT -> no F line', not lines, lines[:2])
        add('SOFTLIMIT -> drive not stopped by it', M.drive_zero_tick(art, tick) == M.drive_zero_tick(art, tick - 1)
            or _value(art, 'drive_l', tick - 1) == 0, dict(drive_before=_value(art, 'drive_l', tick - 1)))
        return
    if code == 'NOCAL':
        state = _value(art, 'mcu_state', tick + 1, STATE_OFF)
        add('NOCAL -> MCU DISARMED', state == STATE_DISARMED, M.MCU_STATE_NAMES.get(state), 'DISARMED')
        lines = [t for t, q, _ in M.mcu_lines(art, 'F') if 'NOCAL' in t]
        add('NOCAL -> no F line at boot (latched mask only)', not lines, lines[:2])
        return
    if code in LATCHED_CODES:
        kept, until = _bit_kept(art, code, tick)
        add('%s latch bit kept until the next ARM' % code, kept, dict(until_arm_tick=until))
