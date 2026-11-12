"""Firmware monitor rule ticks from a recorded emulator session (A19(G) judgement times; 2026-11-09 review RG-02).

The emulator's record hook (gz_esp_emulator InputRecorder) keeps every FW_STEP response of a session. With
report_adc its changes carry every tick's analogRead value (adc_read); the steering pins are always there
(steer_pwm = A - B = u x mdir, steer_en) with mcu_state and fault_bits. FirmwareTrace rebuilds the inputs of the
firmware's monitors per tick (firmware/ubts_steer_v3/fw_monitor.cpp, ESP spec 6.4 and 6.7) and applies the detector
rules to give the first tick at which each code trips (native session ticks, the ticks of the emulator log):

  per tick n (fw.tick, 0 = the setup tick of the boot)
    raw[n]   the analogRead value (adc_read, carried forward between changes)
    f[n]     median of the last med raw values (the refill at boot pads with raw[0])
    pos[n]   fw_eeprom.raw_to_permille(raw_l, raw_c, raw_r, f[n]) (spec 7.3, the calibration of the session)
    u[n]     the servo output: steer_pwm x mdir (the BRAKE output and the OFF mode give 0)
    en[n]    the steering EN pin after the tick (servo mode with a live servo: 1)
  fault step of tick n (before the servo and the pins of tick n; the run counters are those of the end of tick n-1)
    POT_OPEN f < pvmin, POT_SHORT f > pvmax; else POT_JUMP |f[n] - f[n - jmpw]| > jmp (n >= jmpw) and POT_NOISE
             100 (w s2 - s1^2) > nsd^2 w^2 over raw[n - w + 1 .. n], w = nsw (n >= w - 1)
    servo mode (en[n - 1] = 1): POLARITY run_pol >= polt and (pos[n] - pos[n - polt]) x sign < -polp; DRV_STUCK
             run_stuck >= dst and |pos[n] - pos[n - dst]| > dsm; STALL run_stall >= stt and |pos[n] - pos[n - stt]| < stm
    THERMAL  more than tha hot ticks (|A - B| >= thp) in the last thw s
    SOFTLIMIT (condition bit, no latch) pos[n] > soft_l or pos[n] < soft_r
  end of tick n (monitor_end_of_tick with en[n] as the servo mode): run_pol (|u| >= minp, same sign), run_stall
    (|u| >= stp), the DRV_STUCK run of decision 12.1-70 (stuck_run_step), the thermal ring

TRACK (spec 6.7, decision 12.1-69) needs the firmware's cmd (the servo ramp). With the evaluation-only report_servo
channels of lockstep v2 (servo_pos, servo_tgt, servo_cmd, steer_mode after each tick; decision R18-11) the record
has them, and the rule is (WF-4 g_gaps; fw_monitor.cpp monitor_faults):
    servo mode at the fault step of tick n: steer_mode[n - 1] in (ACTIVE, HOLD)
    cmd at the fault step: servo_cmd[n - 1] in a servo mode, else pos[n] (monitor_adc sets cmd = P outside the servo)
    progress: n >= stt and (pos[n] - pos[n - stt]) x sgn(cmd - pos[n]) > stm
    run_track: +1 when servo and |cmd - pos[n]| > trk and no progress, else 0; TRACK trips when servo and run_track
               >= trt
  pos is the firmware's own P (servo_pos); the recomputed pos (median, raw_to_permille) is compared with it tick by
  tick (pos_mismatch_ticks, a check of this module). The mode at the fault step is the mode after tick n - 1: a C line,
  a STOP or the watchdog that changes the mode inside tick n before step 6 is not seen. Entering a servo mode gives
  |cmd - P| = 0 at that tick either way; a mode left inside tick n can count one tick more (a trip needs trt ticks in
  a row). Without the servo channels (report_servo off, older native programs) TRACK is not computed.
A code that is latched is not reported again; this module gives the first trip only. The servo mode of POLARITY,
DRV_STUCK and STALL is taken from the EN pin of the previous tick (exact for the counters: a servo that is not live
has EN 0 and u 0; the JOG and POL calibration modes are not used by the A19 cases).
Pure Python (no numpy). Placeholder-based SIM results, not field validation.
"""
from __future__ import annotations

from .native import fw_eeprom

NOT_RUNNING = 255
CODES = ('POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'STALL', 'THERMAL', 'SOFTLIMIT',
         'TRACK')
SERVO_MODES = (1, 2)                     # steer_mode ACTIVE, HOLD (lockstep_v2.STEER_MODE_NAMES)
STUCK_IDLE, STUCK_BRAKE, STUCK_DRIFT = 0, 1, 2
SAT16 = 0xFFFF


class TraceError(ValueError):
    """The record cannot be turned into a firmware trace (no adc_read values, no boot, ...)."""


def _sgn(value):
    return (value > 0) - (value < 0)


def channel_series(record, channel, reset=0):
    """(first_tick, last_tick, values) of one channel over the recorded ticks (value at tick = last change at or
    before it)."""
    frames = record['frames']
    if not frames:
        raise TraceError('record has no frames')
    first = int(frames[0]['out']['first_tick'])
    last = max(int(f['out']['first_tick']) + int(f['out']['ticks']) for f in frames) - 1
    changes = sorted((int(t), int(v)) for f in frames for t, c, v in f['out']['changes'] if c == channel)
    values, value, index = [], reset, 0
    for tick in range(first, last + 1):
        while index < len(changes) and changes[index][0] <= tick:
            value = changes[index][1]
            index += 1
        values.append(value)
    return first, last, values


class FirmwareTrace:
    """The monitor inputs and detector states of one boot of a recorded session (module docstring)."""

    def __init__(self, record, at_tick=None):
        session = record.get('session') or {}
        if not (session.get('firmware') or {}).get('report_adc'):
            raise TraceError('the recorded session ran without report_adc (no analogRead values)')
        cal = session.get('calibration') or {}
        if not cal:
            raise TraceError('the recorded session has no calibration (blank EEPROM)')
        self.params = dict(fw_eeprom.cfg_defaults(), **(cal.get('params') or {}))
        self.cal = dict(raw_l=int(cal['raw_l']), raw_c=int(cal['raw_c']), raw_r=int(cal['raw_r']),
                        soft_l=int(cal['soft_l']), soft_r=int(cal['soft_r']), mdir=int(cal.get('mdir', 1)))
        first, last, state = channel_series(record, 'mcu_state', NOT_RUNNING)
        boots = [first + i for i in range(len(state)) if state[i] != NOT_RUNNING and (i == 0 or
                                                                                    state[i - 1] == NOT_RUNNING)]
        if not boots:
            raise TraceError('no firmware boot in the record')
        start = boots[0]
        if at_tick is not None:
            before = [b for b in boots if b <= at_tick]
            start = before[-1] if before else boots[0]
        stop = next((b for b in boots if b > start), last + 1)
        running = [i for i in range(start - first, stop - first) if state[i] != NOT_RUNNING]
        end = start - first + len(running)            # the boot runs until the MCU stops (reset, power off)
        self.setup_tick = start
        self.boots = boots
        self.last_tick = first + end - 1
        series = {}
        for channel, reset in (('adc_read', 0), ('steer_pwm', 0), ('steer_en', 0), ('fault_bits', 0)):
            _, _, values = channel_series(record, channel, reset)
            series[channel] = values[start - first:end]
        if not any(series['adc_read']):
            raise TraceError('no adc_read values in the record (report_adc)')
        self.raw = series['adc_read']
        self.pwm = series['steer_pwm']
        self.en = series['steer_en']
        self.fault_bits = series['fault_bits']
        self.servo_available = bool((session.get('firmware') or {}).get('report_servo'))
        self.servo = None
        if self.servo_available:
            self.servo = {}
            for channel in ('servo_pos', 'servo_tgt', 'servo_cmd', 'steer_mode'):
                _, _, values = channel_series(record, channel, 0)
                self.servo[channel] = values[start - first:end]
        self._evaluate()

    def native_tick(self, n):
        return self.setup_tick + n

    def _evaluate(self):
        p, cal = self.params, self.cal
        raw, pwm, en = self.raw, self.pwm, self.en
        n_ticks = len(raw)
        m = int(p['med'])
        w = int(p['nsw'])
        f, pos = [], []
        for n in range(n_ticks):
            window = [raw[max(k, 0)] for k in range(n - m + 1, n + 1)]
            window.sort()
            f.append(window[m // 2])
            pos.append(fw_eeprom.raw_to_permille(cal['raw_l'], cal['raw_c'], cal['raw_r'], f[n]))
        self.f, self.pos = f, pos
        trips = {code: [] for code in CODES}
        run_pol = run_stall = run_stuck = 0
        pol_sign = 0
        stuck_phase, stuck_ref = STUCK_IDLE, 0
        hot = []
        thermal_cnt = 0
        window_ticks = 1000 * int(p['thw'])
        s1 = s2 = 0
        for n in range(n_ticks):
            x = raw[n]
            s1 += x
            s2 += x * x
            if n >= w:
                old = raw[n - w]
                s1 -= old
                s2 -= old * old
            # fault step of tick n
            fn = f[n]
            is_open, is_short = fn < p['pvmin'], fn > p['pvmax']
            if is_open:
                trips['POT_OPEN'].append(n)
            if is_short:
                trips['POT_SHORT'].append(n)
            if not is_open and not is_short:
                jw = int(p['jmpw'])
                if n >= jw and abs(fn - f[n - jw]) > p['jmp']:
                    trips['POT_JUMP'].append(n)
                if n >= w - 1 and 100 * (w * s2 - s1 * s1) > p['nsd'] * p['nsd'] * w * w:
                    trips['POT_NOISE'].append(n)
            servo = n > 0 and en[n - 1] == 1
            if servo:
                polt = int(p['polt'])
                if run_pol >= polt and n >= polt and (pos[n] - pos[n - polt]) * pol_sign < -p['polp']:
                    trips['POLARITY'].append(n)
                dst = int(p['dst'])
                if run_stuck >= dst and n >= dst and abs(pos[n] - pos[n - dst]) > p['dsm']:
                    trips['DRV_STUCK'].append(n)
                stt = int(p['stt'])
                if run_stall >= stt and n >= stt and abs(pos[n] - pos[n - stt]) < p['stm']:
                    trips['STALL'].append(n)
            if thermal_cnt > p['tha']:
                trips['THERMAL'].append(n)
            if pos[n] > cal['soft_l'] or pos[n] < cal['soft_r']:
                trips['SOFTLIMIT'].append(n)
            # end of tick n
            u = pwm[n] * cal['mdir']
            if en[n] == 1:
                if abs(u) >= max(int(p['minp']), 1):
                    s = _sgn(u)
                    if run_pol > 0 and pol_sign == s:
                        run_pol = min(run_pol + 1, SAT16)
                    else:
                        run_pol, pol_sign = 1, s
                else:
                    run_pol = 0
                run_stall = min(run_stall + 1, SAT16) if abs(u) >= max(int(p['stp']), 1) else 0
            else:
                run_pol, pol_sign, run_stall = 0, 0, 0
            # DRV_STUCK observation run (fw_monitor.cpp stuck_run_step, decision 12.1-70)
            db = int(p['db'])
            if en[n] != 1:
                stuck_phase, run_stuck = STUCK_IDLE, 0
            else:
                moved = pos[n] - stuck_ref
                if u == 0:
                    if stuck_phase == STUCK_IDLE:
                        stuck_phase, stuck_ref, run_stuck = STUCK_BRAKE, pos[n], 0
                    elif stuck_phase == STUCK_DRIFT and abs(moved) <= db:
                        stuck_phase = STUCK_BRAKE
                    run_stuck = min(run_stuck + 1, SAT16)
                else:
                    if stuck_phase == STUCK_BRAKE and abs(moved) > db:
                        stuck_phase = STUCK_DRIFT
                    if stuck_phase == STUCK_DRIFT and moved * _sgn(u) <= db:
                        run_stuck = min(run_stuck + 1, SAT16)
                    else:
                        stuck_phase, run_stuck = STUCK_IDLE, 0
            # thermal ring: the hot ticks of the last window
            is_hot = abs(pwm[n]) >= max(int(p['thp']), 1)
            hot.append(is_hot)
            if n >= window_ticks and hot[n - window_ticks]:
                thermal_cnt -= 1
            if is_hot:
                thermal_cnt += 1
        self.trips = trips
        self.pos_mismatch_ticks = None
        if self.servo is not None:
            self._evaluate_track()

    def _evaluate_track(self):
        """TRACK from the report_servo channels (module docstring); also the recomputed pos against servo_pos."""
        p = self.params
        pos_fw, cmd_fw, mode = self.servo['servo_pos'], self.servo['servo_cmd'], self.servo['steer_mode']
        self.pos_mismatch_ticks = sum(1 for a, b in zip(self.pos, pos_fw) if a != b)
        trk, trt, stt, stm = int(p['trk']), int(p['trt']), int(p['stt']), int(p['stm'])
        run_track = 0
        trips = []
        for n in range(len(pos_fw)):
            servo = n > 0 and mode[n - 1] in SERVO_MODES
            pos = pos_fw[n]
            cmd = cmd_fw[n - 1] if servo else pos
            progress = n >= stt and (pos - pos_fw[n - stt]) * _sgn(cmd - pos) > stm
            if servo and abs(cmd - pos) > trk and not progress:
                run_track = min(run_track + 1, SAT16)
            else:
                run_track = 0
            if servo and run_track >= trt:
                trips.append(n)
        self.trips['TRACK'] = trips

    def first_trip(self, code, after_tick=None):
        """Native tick of the first trip of code at or after after_tick (native), or None."""
        if code == 'TRACK' and self.servo is None:
            raise TraceError('TRACK needs the report_servo channels (servo_cmd, steer_mode) in the record')
        if code not in self.trips:
            raise KeyError(code)
        start = 0 if after_tick is None else after_tick - self.setup_tick
        hit = next((n for n in self.trips[code] if n >= start), None)
        return None if hit is None else self.native_tick(hit)

    def first_change(self, code, after_tick):
        """SOFTLIMIT-like condition: native tick at or after after_tick where the condition starts (it was false the
        tick before), or None."""
        start = max(after_tick - self.setup_tick, 1)
        ticks = set(self.trips[code])
        hit = next((n for n in range(start, len(self.raw)) if n in ticks and (n - 1) not in ticks), None)
        return None if hit is None else self.native_tick(hit)

    def summary(self):
        return dict(setup_tick=self.setup_tick, boots=self.boots, ticks=len(self.raw), last_tick=self.last_tick,
                    first_trips={code: (None if not ticks else self.native_tick(ticks[0]))
                                 for code, ticks in self.trips.items()},
                    servo_channels=self.servo is not None, pos_mismatch_ticks=self.pos_mismatch_ticks,
                    params={k: self.params[k] for k in ('med', 'nsd', 'nsw', 'jmp', 'jmpw', 'pvmin', 'pvmax', 'polp',
                                                        'polt', 'dsm', 'dst', 'stp', 'stt', 'stm', 'thp', 'thw',
                                                        'tha', 'db', 'minp', 'trk', 'trt')},
                    calibration=self.cal)
