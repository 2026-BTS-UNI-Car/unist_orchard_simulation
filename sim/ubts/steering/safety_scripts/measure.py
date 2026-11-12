"""Measurements of one executor-in-the-loop episode (safety scripts; SIM_ACCEPTANCE A17, A19, A22).

Artefacts (artefacts()): the rig serial log (both directions, SIM ms / MCU ticks), the MCU lines (text, queued / done
tick), the MCU channel change lists (pins, mcu_state, fault_bits, servo_pos / servo_cmd / servo_tgt / steer_mode, adc_read
when the rig reports it), the MCU CFG values of the calibration record, the host profile timing, the trace, the host latch
log and the executor's events. Times: 1 MCU tick = 1 ms = 1 rig ms.

Judgement ticks (spec ESP_PROTOCOL_V3_DRAFT_KO.md 6.2, 6.3, 6.6, 6.7): expected_fault_tick(code, ...) recomputes the
earliest tick at which the firmware's rule becomes true, from the MCU's own observable channels (P[n] = servo_pos, cmd =
servo_cmd before the tick, u[k] = d[k] = steer_pwm, the steering mode, raw = adc_read) and the CFG values, independently
of the firmware code. The check compares it with the tick of the F / W line (A19: within +-1 tick). Rules with internal
state the channels do not show exactly (the order of steps inside one tick) follow the spec text: a counter updated at
step 11 of tick k is seen by the step-6 judgement of tick k + 1.

Evaluation only (A13 role 'evaluator').
"""
from __future__ import annotations

import bisect
import math

import numpy as np

ACTIVE_MODES = (1, 2)          # steer_mode ACTIVE, HOLD (lockstep_v2 STEER_MODE_NAMES)
CHANNEL_NAMES = ('drive_l', 'drive_r', 'drive_en', 'steer_pwm', 'steer_en', 'mcu_state', 'fault_bits', 'servo_pos',
                 'servo_cmd', 'servo_tgt', 'steer_mode', 'adc_read')
MCU_STATE_NAMES = {0: 'BOOT', 1: 'DISARMED', 2: 'ARMED', 3: 'BENCH', 255: 'OFF'}
FAULT_BIT_NAMES = ('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'SOFTLIMIT',
                   'STALL', 'TRACK', 'THERMAL', 'TTL', 'WATCHDOG', 'CRC', 'BOOT')
# Field order of the S line (ESP_PROTOCOL_V3 5.3; the same tuple as actuator_link.STATUS_FIELDS, checked by the tests).
STATUS_FIELDS = ('ms', 'n', 'st', 'q', 'L', 'R', 'en', 'raw', 'pos', 'tgt', 'cmd', 'sp', 'fl', 'wd', 'tl')


def bit(name):
    return 1 << FAULT_BIT_NAMES.index(name)


# --- artefacts ----------------------------------------------------------------------------------------------------------
def artefacts(facade, spec):
    """Everything the checks need from a finished episode (a picklable dict)."""
    rig = facade.rig
    channels = {}
    for name in CHANNEL_NAMES:
        try:
            channels[name] = [(int(t), int(v)) for t, v in rig.mcu_channel(name)]
        except KeyError:
            channels[name] = []
    lines = [(line.text, int(line.queued_tick), int(line.done_tick)) for line in rig.mcu_lines()]
    hp = rig.hp
    cal = rig.cal or {}
    trace = facade.trace_arrays()
    keep = ('t_s', 'cmd_throttle_pct', 'cmd_steer_target_mrad', 'cmd_stop_mode', 'steer_target_rad', 'drive_applied_pct',
            'drive_request_pct', 'truth_v_m_s', 'truth_x_m', 'truth_y_m', 'truth_yaw_rad', 'truth_delta_eq_rad',
            'truth_hold_position_rad', 'truth_curvature_1_m', 'truth_field_y_m', 'truth_mcu_state', 'drive_en', 'steer_en',
            'host_lease', 'host_state', 'host_arm_state', 'ctrl_throttle_pct', 'ctrl_steer_mrad', 'ctrl_state',
            'ctrl_hold')
    return dict(
        spec=spec, summary=facade.summary(), serial=rig.serial_log(), lines=lines, channels=channels,
        cfg=dict(cal.get('params') or {}), cal=dict((k, v) for k, v in cal.items() if k != 'params'),
        host=dict(status_hz=hp.status_hz, stop_confirm_s=hp.stop_confirm_s,
                  stop_resend_interval_s=hp.stop_resend_interval_s, stop_resend_max=hp.stop_resend_max,
                  c_ttl_max_ms=hp.c_ttl_max_ms, watchdog_ms=hp.watchdog_ms, status_gap_s=hp.status_gap_s,
                  bridge_period_ms=rig.config.bridge_period_ms, bridge_phase_ms=rig.config.bridge_phase_ms,
                  limit_left_mrad=hp.limit_left_mrad, limit_right_mrad=hp.limit_right_mrad,
                  mark_left_mrad=hp.mark_left_mrad, mark_right_mrad=hp.mark_right_mrad, arm_confirm_s=hp.arm_confirm_s,
                  track_error_mrad=hp.track_error_mrad, track_duration_s=hp.track_duration_s,
                  seq_gap_missing=hp.seq_gap_missing, seq_gap_window_s=hp.seq_gap_window_s,
                  ineffective=dict(hp.ineffective), wheelbase_m=hp.wheelbase_m),
        trace={k: np.asarray(trace[k]) for k in keep if k in trace}, latches=list(facade.latch_log),
        boot_delay_ticks=int(rig.config.boot_delay_ticks), servo_hz=float(rig.profile.value('servo_hz')),
        hold_tolerance_rad=max(math.radians(1.), 2. * math.radians(float(rig.profile.value('servo_deadband_deg')))),
        bridge_events=[list(e) for e in getattr(rig, 'bridge_events', ())],
        motion=[tuple(m) for m in getattr(facade, 'motion_log', ())],
        rig_counters=dict(rig.counters))


# --- channels -----------------------------------------------------------------------------------------------------------
def value_at(changes, tick, default=0):
    ticks = [t for t, _ in changes]
    index = bisect.bisect_right(ticks, tick) - 1
    return changes[index][1] if index >= 0 else default


def series(changes, lo, hi, default=0):
    """Per-tick values for ticks lo..hi (inclusive) of a change list."""
    out = np.full(hi - lo + 1, value_at(changes, lo - 1, default), dtype=np.int64)
    for t, v in changes:
        if lo <= t <= hi:
            out[t - lo:] = v
    return out


def first_tick(changes, predicate, since, until=None, default=0):
    """First tick >= since where predicate(value) is true (None if never, up to until)."""
    if predicate(value_at(changes, since, default)):
        return since
    for t, v in changes:
        if t <= since:
            continue
        if until is not None and t > until:
            return None
        if predicate(v):
            return t
    return None


# --- lines ----------------------------------------------------------------------------------------------------------------
def mcu_lines(art, kind=None, since=0):
    out = []
    for text, queued, done in art['lines']:
        head = text.split(' ', 1)[0]
        if queued >= since and (kind is None or head == kind):
            out.append((text, queued, done))
    return out


def f_line(art, code, since=0):
    """(tick, text) of the first F <code> line queued at or after since."""
    for text, queued, _ in mcu_lines(art, 'F', since):
        parts = text.split('*')[0].split(' ')
        if len(parts) > 1 and parts[1] == code:
            return queued, text
    return None, None


def w_line(art, since=0):
    lines = mcu_lines(art, 'W', since)
    return (lines[0][1], lines[0][0]) if lines else (None, None)


def h2m(art, head=None, since_ms=0):
    """Host -> MCU records (dict t_ms, first_tick, last_tick, text, note) whose first token is head."""
    out = []
    for r in art['serial']:
        if r['dir'] != 'h2m' or r['t_ms'] < since_ms:
            continue
        token = r['text'].split('*')[0].split(' ')[0].strip()
        if head is None or token == head:
            out.append(r)
    return out


def m2h_status(art, since_ms=0):
    """Status lines the host could read: dict(t_ms, queued, done, fields) (fields named by STATUS_FIELDS)."""
    out = []
    for r in art['serial']:
        if r['dir'] != 'm2h' or r['t_ms'] < since_ms or not r['text'].startswith('S ') or r['note']:
            continue
        parts = r['text'].split('*')[0].split(' ')
        fields = {}
        for name, token in zip(STATUS_FIELDS, parts[1:]):
            try:
                fields[name] = int(token, 16) if name == 'fl' else int(token)
            except ValueError:
                fields[name] = token
        out.append(dict(t_ms=r['t_ms'], queued=r['first_tick'], done=r['last_tick'], fields=fields))
    return out


def latch_time(art, name):
    for entry in art['latches']:
        if entry[0] == name:
            return entry[1]
    return None


def executor_event(art, name, since_s=0.):
    for e in art['summary']['executor_events']:
        if e.get('event') == name and e.get('t', 0.) >= since_s - 1e-9:
            return e
    return None


# --- spec 6.7 judgement ticks ---------------------------------------------------------------------------------------------
def _window(art, lo, hi):
    ch = art['channels']
    return dict(P=series(ch['servo_pos'], lo, hi), cmd=series(ch['servo_cmd'], lo, hi),
                u=series(ch['steer_pwm'], lo, hi), mode=series(ch['steer_mode'], lo, hi),
                en=series(ch['steer_en'], lo, hi), raw=series(ch['adc_read'], lo, hi))


def _active(mode):
    return np.isin(mode, ACTIVE_MODES)


def expected_fault_tick(code, art, onset, horizon_ms=40000):
    """(expected tick, method) of the earliest judgement of `code` at or after onset (an MCU tick), or (None, why)."""
    cfg = art['cfg']
    lo = max(0, onset - 32000)                 # enough history for the longest window (THERMAL 30 s)
    hi = onset + horizon_ms
    if code in ('POT_OPEN', 'POT_SHORT', 'POT_JUMP'):
        if not art['channels']['adc_read']:
            med = int(cfg.get('med', 5))
            return onset + (med - 1) // 2, 'onset + (med - 1) / 2 (median filter; no adc_read channel)'
        w = _window(art, onset - 8, hi)
        raw = w['raw']
        med = int(cfg.get('med', 5))
        f = np.array([np.median(raw[max(0, i - med + 1):i + 1]) if i >= med - 1 else raw[i] for i in range(len(raw))])
        base = onset - 8
        for i in range(8, len(f)):
            n = base + i
            if code == 'POT_OPEN' and f[i] < cfg.get('pvmin', 20):
                return n, 'f[n] < pvmin (median of %d raw)' % med
            if code == 'POT_SHORT' and f[i] > cfg.get('pvmax', 1003):
                return n, 'f[n] > pvmax (median of %d raw)' % med
            if code == 'POT_JUMP':
                j = int(cfg.get('jmpw', 2))
                if i - j >= 0 and abs(f[i] - f[i - j]) > cfg.get('jmp', 8) and \
                        cfg.get('pvmin', 20) <= f[i] <= cfg.get('pvmax', 1003):
                    return n, '|f[n] - f[n - jmpw]| > jmp'
        return None, 'condition never true'
    if code == 'POT_NOISE':
        if not art['channels']['adc_read']:
            return None, 'needs the adc_read channel'
        width = int(cfg.get('nsw', 50))
        nsd = int(cfg.get('nsd', 50))
        w = _window(art, onset - width, hi)
        raw = w['raw'].astype(np.int64)
        base = onset - width
        for i in range(width, len(raw)):
            x = raw[i - width + 1:i + 1]
            if 100 * (width * int(np.sum(x * x)) - int(np.sum(x)) ** 2) > nsd * nsd * width * width:
                return base + i, '100 (W sum x^2 - (sum x)^2) > nsd^2 W^2 over raw[n - W + 1 .. n]'
        return None, 'condition never true'
    w = _window(art, lo, hi)
    P, cmd, u, active, en = w['P'], w['cmd'], w['u'], _active(w['mode']), w['en']
    start = onset - lo
    if code == 'STALL':
        stp, stt, stm = max(int(cfg.get('stp', 153)), 1), int(cfg.get('stt', 500)), int(cfg.get('stm', 20))
        run = 0
        for i in range(1, len(P)):
            # judgement of tick i (step 6) with the counter as tick i - 1 left it (step 11)
            if i >= start and run >= stt and i - stt >= 0 and abs(P[i] - P[i - stt]) < stm:
                return lo + i, 'runStall >= stt and |P[n] - P[n - stt]| < stm'
            run = run + 1 if (active[i] and abs(u[i]) >= stp) else 0
        return None, 'condition never true'
    if code == 'POLARITY':
        minp, polt, polp = max(int(cfg.get('minp', 102)), 1), int(cfg.get('polt', 100)), int(cfg.get('polp', 40))
        run, sign = 0, 0
        for i in range(1, len(P)):
            if i >= start and run >= polt and i - polt >= 0 and (P[i] - P[i - polt]) * sign < -polp:
                return lo + i, 'runPol >= polt and (P[n] - P[n - polt]) s < -polp'
            if active[i] and abs(u[i]) >= minp:
                s = 1 if u[i] > 0 else -1
                if run and s == sign:
                    run += 1
                else:
                    run, sign = 1, s
            else:
                run, sign = 0, 0
        return None, 'condition never true'
    if code == 'TRACK':
        trk, trt = int(cfg.get('trk', 120)), int(cfg.get('trt', 1000))
        stt, stm = int(cfg.get('stt', 500)), int(cfg.get('stm', 20))
        run = 0
        for i in range(1, len(P)):
            c = cmd[i - 1]                           # cmd before this tick's servo update
            err = c - P[i]
            moved = (P[i] - P[i - stt]) * (1 if err > 0 else -1) if i - stt >= 0 else 0
            # runTrack is updated in the step-6 judgement itself, in the servo mode the tick started with (the mode
            # channel holds the value after the tick, so the previous tick's value; a trip turns it OFF in that tick)
            run = run + 1 if (active[i - 1] and abs(err) > trk and moved <= stm) else 0
            if i >= start and run >= trt:
                return lo + i, 'runTrack >= trt (12.1-69 rule)'
        return None, 'condition never true'
    if code == 'THERMAL':
        thp, thw, tha = max(int(cfg.get('thp', 204)), 1), int(cfg.get('thw', 30)), int(cfg.get('tha', 5000))
        high = (np.abs(u) >= thp).astype(np.int64)
        csum = np.concatenate(([0], np.cumsum(high)))
        span = 1000 * thw
        for i in range(start, len(P)):
            a = max(0, i - span)
            if csum[i] - csum[a] > tha:              # k in [n - span, n - 1]
                return lo + i, 'count of |d[k]| >= thp over the last thw s > tha'
        return None, 'condition never true'
    if code == 'DRV_STUCK':
        db, dst, dsm = int(cfg.get('db', 20)), int(cfg.get('dst', 300)), int(cfg.get('dsm', 80))
        run, phase, ps = 0, None, 0
        for i in range(1, len(P)):
            if i >= start and run >= dst and i - dst >= 0 and abs(P[i] - P[i - dst]) > dsm:
                return lo + i, 'runStuck >= dst and |P[n] - P[n - dst]| > dsm (12.1-70)'
            if not (active[i] and en[i]):
                run, phase = 0, None
                continue
            if u[i] == 0:
                if phase is None:
                    phase, ps, run = 'brake', P[i], 1
                else:
                    if phase == 'drift' and abs(P[i] - ps) <= db:
                        phase = 'brake'
                    run += 1
            elif phase is None:
                run = 0
            elif phase == 'brake' and abs(P[i] - ps) <= db:
                run, phase = 0, None
            else:
                phase = 'drift'
                if (P[i] - ps) * (1 if u[i] > 0 else -1) <= db:
                    run += 1
                else:
                    run, phase = 0, None
        return None, 'condition never true'
    return None, 'no rule here for %s' % code


def _refusals(art):
    """MCU N lines by queued tick: {tick: [q or None, ...]} ('N <q> <reason>' or 'N - <reason>')."""
    out = {}
    for text, queued, _ in art['lines']:
        parts = text.split('*')[0].split(' ')
        if parts[0] != 'N' or len(parts) < 2:
            continue
        try:
            q = None if parts[1] == '-' else int(parts[1])
        except ValueError:
            q = None
        out.setdefault(queued, []).append(q)
    return out


def refused_by_mcu(record, refusals):
    """True when the MCU refused this host line: an N line queued at the tick its LF arrived, with the line's q for a
    C line ('N <q> ...') or 'N - ...' for any line (a bad frame or CRC has no readable q). Spec 6.2 / 6.3: a refused
    line or a frame error does not update last_input and does not start a C TTL."""
    if record['last_tick'] is None:
        return False
    found = refusals.get(record['last_tick'])
    if not found:
        return False
    parts = record['text'].split('*')[0].split(' ')
    q = None
    if parts[0] == 'C' and len(parts) > 1:
        try:
            q = int(parts[1])
        except ValueError:
            q = None
    return None in found or (q is not None and q in found)


def accepted_inputs(art, heads, before_tick, inclusive=False):
    """Host lines with these first tokens that reached the MCU before before_tick (or at it, inclusive) and that the MCU
    did not refuse (refused_by_mcu): what spec 6.2 / 6.3 count as received input."""
    refusals = _refusals(art)
    out = []
    for r in art['serial']:
        if r['dir'] != 'h2m' or r['last_tick'] is None or r['note'].endswith(':lost'):
            continue
        if not (r['last_tick'] < before_tick or (inclusive and r['last_tick'] == before_tick)):
            continue
        if r['text'].split('*')[0].split(' ')[0] not in heads or refused_by_mcu(r, refusals):
            continue
        out.append(r)
    return out


def expected_watchdog_tick(art, since_tick=0):
    """Last input the MCU accepted (PING, C, ARM; not a refused line, spec 6.2) before the W line + wd + 1."""
    w_tick, _ = w_line(art, since_tick)
    if w_tick is None:
        return None, None
    inputs = [r['last_tick'] for r in accepted_inputs(art, ('PING', 'C', 'ARM'), w_tick)]
    if not inputs:
        return None, w_tick
    wd = art['cfg'].get('wd')
    wd = art['host']['watchdog_ms'] if wd is None else wd
    return max(inputs) + int(wd) + 1, w_tick


def expected_ttl_tick(art, since_tick=0):
    """The last C the MCU accepted (not refused, spec 6.3) before the F TTL line: its arrival tick + its ttl."""
    f_tick, _ = f_line(art, 'TTL', since_tick)
    if f_tick is None:
        return None, None, None
    cs = accepted_inputs(art, ('C',), f_tick, inclusive=True)
    if not cs:
        return None, f_tick, None
    last = cs[-1]
    ttl = int(last['text'].split('*')[0].split(' ')[2])
    return last['last_tick'] + ttl, f_tick, last


# --- outcomes -------------------------------------------------------------------------------------------------------------
RELEASE_RAMP_PER_S = 255          # fw_drive.cpp kRampPerS: drive duty units per second of a release (THERMAL alone)
RAMP_GRID_MS = 10                 # fw_drive.cpp kRampGridMs


def release_bound_ticks(art, tick):
    """Ticks a drive release ramp may take from tick to 0: |d| / 255 per s, on the 10 ms ramp grid, + 1 tick."""
    ch = art['channels']
    d0 = max(abs(value_at(ch['drive_l'], tick - 1)), abs(value_at(ch['drive_r'], tick - 1)))
    return int(math.ceil(d0 * 1000. / RELEASE_RAMP_PER_S - 1e-9)) + RAMP_GRID_MS + 1, d0


def drive_zero_tick(art, since):
    ch = art['channels']
    a = first_tick(ch['drive_l'], lambda v: v == 0, since)
    b = first_tick(ch['drive_r'], lambda v: v == 0, since)
    return None if a is None or b is None else max(a, b)


def steer_off_tick(art, since):
    ch = art['channels']
    a = first_tick(ch['steer_pwm'], lambda v: v == 0, since)
    b = first_tick(ch['steer_en'], lambda v: v == 0, since)
    return None if a is None or b is None else max(a, b)


def state_at(art, tick):
    return MCU_STATE_NAMES.get(value_at(art['channels']['mcu_state'], tick, 255), 'UNKNOWN')


def drive_nonzero_after(art, tick):
    """First tick > tick with a drive output other than 0 (an automatic restart if no operator re-arm came)."""
    ch = art['channels']
    a = first_tick(ch['drive_l'], lambda v: v != 0, tick + 1)
    b = first_tick(ch['drive_r'], lambda v: v != 0, tick + 1)
    found = [t for t in (a, b) if t is not None]
    return min(found) if found else None


def drive_increase_after(art, tick):
    """A17 'no automatic restart' from a stop (review P4R-11 / RS-02): the first tick >= tick at which both drive
    outputs are 0 (the stop reached the pins; None: never), and the first later tick at which either output's magnitude
    grows (0 -> non-zero, or a larger |duty|; a release ramp only shrinks). Returns (zero_tick, increase_tick or None)."""
    zero = drive_zero_tick(art, tick)
    if zero is None:
        return None, None
    ch = art['channels']
    found = []
    for name in ('drive_l', 'drive_r'):
        previous = abs(value_at(ch[name], zero))
        for t, v in ch[name]:
            if t <= zero:
                continue
            if abs(v) > previous:
                found.append(t)
                break
            previous = abs(v)
    return zero, (min(found) if found else None)


def arm_lines_after(art, t_ms):
    return [r for r in h2m(art, 'ARM', since_ms=t_ms + 1)]


def stop_timing(art, t_request_s, stalls=()):
    """A17 STOP items for a STOP request at t_request_s: send delay, parse -> drive 0, confirmation, resends.
    stalls: MCU loop stall windows (first tick, end tick); a STOP arriving inside one is parsed at its end."""
    t_req_ms = int(round(t_request_s * 1000.))
    stops = h2m(art, 'STOP', since_ms=t_req_ms)
    if not stops:
        return dict(found=False)
    first = stops[0]
    parse = first['last_tick']
    processed = parse
    for a, b in stalls:
        if parse is not None and a <= parse < b:
            processed = b
    zero = drive_zero_tick(art, parse) if parse is not None else None
    period_ms = 1000. / art['host']['status_hz']
    confirm = None
    for line in m2h_status(art, since_ms=first['t_ms']):
        f = line['fields']
        if parse is not None and line['queued'] is not None and line['queued'] >= parse and f.get('st') == 1 \
                and f.get('L') == 0 and f.get('R') == 0:
            confirm = line
            break
    resends = [r for r in stops[1:] if r['t_ms'] - first['t_ms'] <= 1000]
    return dict(found=True, request_ms=t_req_ms, sent_ms=first['t_ms'], send_delay_ms=first['t_ms'] - t_req_ms,
                parse_tick=parse, processed_tick=processed, drive_zero_tick=zero,
                parse_to_zero_ticks=None if zero is None or processed is None else zero - processed,
                servo_tick_ms=1000. / art['servo_hz'],
                confirm_readable_ms=None if confirm is None else confirm['t_ms'],
                confirm_delay_ms=None if confirm is None else confirm['t_ms'] - first['t_ms'],
                confirm_limit_ms=2 * period_ms, resend_offsets_ms=[r['t_ms'] - first['t_ms'] for r in resends])


def resend_grid(art):
    """Expected resend offsets (ms after the first STOP) on the bridge tick grid and the latch offset (amendment 7)."""
    host = art['host']
    tick = host['bridge_period_ms']
    confirm = host['stop_confirm_s'] * 1000.
    interval = host['stop_resend_interval_s'] * 1000.
    out = []
    for k in range(int(host['stop_resend_max'])):
        due = confirm + k * interval
        out.append(int(math.ceil(due / tick - 1e-9) * tick))
    latch = confirm + (host['stop_resend_max'] - 1) * interval + confirm
    return out, int(math.ceil(latch / tick - 1e-9) * tick)


def hold_error(art, t0_s, duration_s):
    """Largest |true steering - steering at t0| over [t0, t0 + duration] (the hold of a STOP) and the reference."""
    tr = art['trace']
    t = tr['t_s']
    k0 = int(np.searchsorted(t, t0_s - 1e-9))
    k1 = int(np.searchsorted(t, t0_s + duration_s + 1e-9))
    if k0 >= len(t):
        return None
    eq = tr['truth_delta_eq_rad']
    ref = float(eq[max(0, k0 - 1)])
    return float(np.max(np.abs(eq[k0:k1] - ref))) if k1 > k0 else 0.


A22_REST_V_M_S = .02            # 'at rest' of the automatic-centring test (the A22 window, as before audit AUD3A-07)


def command_after(art, t0_s, duration_s=3.):
    """A22: the commanded steering (MCU servo target, steer_target_rad) after the terminal moment t0 for duration_s,
    judged by the evaluator's STEER_AFTER_TERMINAL rule (evaluate_steer.steer_after_terminal; one definition, audit
    AUD3A-07): (a) no move beyond the hold tolerance from the command at t0, (b) a stop that moves the command must move
    it to the steering position at the stop (truth_hold_position_rad: the MCU filtered potentiometer position a
    hold_measured STOP holds, R13-3), also when the MCU applies the stop after t0 (the STOP that lands after the
    executor's terminal moment: the rows where the MCU leaves ARMED, evaluate_steer.mcu_stop_rows), and no move toward 0
    at rest (automatic centring). Plus the executor's own command (ctrl_steer_mrad) does not move beyond the tolerance.
    The whole window must be in the trace (review P4R-06): a trace that ends before t0 + duration_s gives passed False,
    complete False (not measured; the script's duration must be longer). The rule's window is cut at t0 + duration_s."""
    from steering import evaluate_steer as ES
    tr = art['trace']
    t = np.asarray(tr['t_s'], dtype=float)
    k0 = int(np.searchsorted(t, t0_s - 1e-9))
    k1 = int(np.searchsorted(t, t0_s + duration_s + 1e-9))
    if k0 >= len(t) or k1 <= k0:
        return None
    row_dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.
    complete = bool(float(t[-1]) >= t0_s + duration_s - .5 * row_dt - 1e-9)
    tol = art['hold_tolerance_rad']

    def column(name, dtype=float):
        value = tr.get(name)
        return None if value is None else np.asarray(value)[:k1].astype(dtype)

    target = column('steer_target_rad')
    hold = column('truth_hold_position_rad')
    drive = column('drive_request_pct')
    stop_code = column('cmd_stop_mode', np.int64)
    state = column('truth_mcu_state', np.int64)
    stop_rows = [] if state is None else ES.mcu_stop_rows(dict(truth_mcu_state=state))
    rule = ES.steer_after_terminal(t[:k1], target, drive, stop_code, column('truth_v_m_s'), [(k0, 'terminal_t_s')],
                                   tol, A22_REST_V_M_S, hold_position=hold,
                                   hold_source='command_before' if hold is None else 'truth_hold_position',
                                   true_eq=column('truth_delta_eq_rad'), stop_rows=stop_rows)
    window = rule['windows'][0]
    ctrl = np.asarray(tr['ctrl_steer_mrad'], dtype=float)[k0:k1] / 1000.
    ctrl_defined = ctrl[np.isfinite(ctrl)]
    ctrl_move = float(np.max(np.abs(ctrl_defined - ctrl_defined[0]))) if len(ctrl_defined) else 0.
    return dict(window_s=[round(float(t[k0]), 3), round(float(t[k1 - 1]), 3)], required_s=round(float(duration_s), 3),
                complete=complete, target_at_start_deg=window['command_at_start_deg'],
                hold_position_deg=window['hold_position_deg'],
                max_target_move_deg=window['max_dev_from_command_deg'], tolerance_deg=round(math.degrees(tol), 4),
                stop_moved_command=window['stop_moved_command'], mcu_stops=window['mcu_stops'],
                rule='evaluate_steer.steer_after_terminal (STEER_AFTER_TERMINAL a, b)',
                executor_command_move_deg=round(math.degrees(ctrl_move), 4), auto_centre=window['auto_centre'],
                passed=bool(complete and not window['violated'] and not window['auto_centre']
                            and ctrl_move <= tol + 1e-9))
