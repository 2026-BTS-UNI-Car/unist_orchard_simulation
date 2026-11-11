"""Bench tool for the identification bench tests (B0-B7): ESP protocol V3 lines straight on the serial line.

In the field this is a laptop script on the ESP's USB port while the bridge is stopped (ESP_PROTOCOL_V3_DRAFT_KO.md:
INFO, PING, STOP, ARM, DISARM, C, CAL JOG/MARK/SOFT/POL/SAVE/GET, CFG, BENCH, D<n>). Here it runs in an IdentRig in
bench mode: it writes through IdentRig.bench_write() and reads the MCU lines the rig delivers to its MemoryTransport.
It uses the host's V3 code (esp_motor_v3: line builders, CRC, the strict MCU line parser).

Keepalive like the bridge: while the MCU is ARMED the current C command is sent again every C_PERIOD_S with a new q
and ttl TTL_MS (the MCU TTL never extends a command, spec 4.8); while ARMED or BENCH a PING goes out every
PING_PERIOD_S (link watchdog, spec 6.2).

Everything the tool sends or reads goes into its bench record (recorder.BenchRecord): the serial log with host times,
the operator notes (test, event, parameters), the status lines as steering sidecar rows. No truth is read here.
"""
from __future__ import annotations

import collections
import math

C_PERIOD_S = .05
PING_PERIOD_S = .1
TTL_MS = 300
STEP_S = .01
THERMAL_SHARE = .3          # cool down before a strong output while the THERMAL proxy count is above 30 % of tha


class BenchError(RuntimeError):
    """A command was refused (N line) or an expected answer did not come."""


class BenchTool:
    def __init__(self, rig, record):
        self.rig = rig
        self.v3 = rig.v3
        self.record = record
        self.splitter = self.v3.LineSplitter()
        self.banner = None
        self.boot_id = None
        self.cal = None
        self.status = None
        self.status_t = None
        self.lines = []                 # (host time s, parsed dict) of every MCU line read
        self.rejects = []
        self.faults = []
        self.acks = []
        self.watchdogs = []
        self.q = 0
        self.armed = False
        self.bench = False
        self.drive_lr = (0, 0)
        self.steer = 0
        self._last_c = -math.inf
        self._last_ping = -math.inf
        self.calibration = None         # host-side table {'mark_left_mrad', 'mark_right_mrad'} for angle_mrad
        self.target_mrad = None
        self.test = None
        self.pwm_history = collections.deque(maxlen=4000)    # (MCU ms, steering pin value) of the status lines
        self.thermal = None                                  # (thp PWM, thw s, tha ms) read with CFG when needed
        rig.attach_bench_reader(self)

    # -- clock and stepping -----------------------------------------------------------------------------------------
    def now(self):
        return self.rig.clock()

    def run(self, seconds):
        """Advance the rig by seconds (10 ms steps); lines are read and keepalives written after every step."""
        steps = int(round(float(seconds) / STEP_S))
        for _ in range(max(0, steps)):
            self.rig.step(STEP_S)

    def wait_for(self, predicate, timeout_s):
        end = self.now() + timeout_s
        while self.now() < end - 1e-9:
            if predicate():
                return True
            self.rig.step(STEP_S)
        return bool(predicate())

    # -- notes ----------------------------------------------------------------------------------------------------
    def begin(self, test):
        self.test = test
        self.note('begin')

    def note(self, event, **params):
        self.record.add_note(self.now(), self.rig.wall_clock(), self.test, event, params)

    # -- reading (called by IdentRig after every plant step) ---------------------------------------------------------
    def poll(self):
        transport = self.rig.bench_transport
        if not transport.is_open:
            return
        data = transport.read()
        if data:
            for line in self.splitter.feed(data):
                self._on_line(line)
        self._keepalive()

    def _on_line(self, content):
        t = self.now()
        try:
            parsed = self.v3.parse_mcu_line(content)
        except self.v3.V3LineError as exc:
            parsed = dict(kind='bad', code=exc.code)
        self.record.add_serial(t, self.rig.wall_clock(), 'm2h', content.decode('ascii', 'replace'), parsed.get('kind'))
        self.lines.append((t, parsed))
        kind = parsed['kind']
        if kind == 'banner':
            self.banner = parsed
            self.boot_id = parsed['boot']
            self.armed = self.bench = False
        elif kind == 'cal':
            self.cal = parsed
        elif kind == 'status':
            self.status, self.status_t = parsed, t
            self.pwm_history.append((parsed['ms'], parsed['sp']))
            self.armed = parsed['st'] == 2
            self.bench = parsed['st'] == 3
            self.record.add_status(self._status_row(parsed, t, len(content) + 1))
        elif kind == 'reject':
            self.rejects.append((t, parsed))
        elif kind == 'fault':
            self.faults.append((t, parsed))
        elif kind == 'ack':
            self.acks.append((t, parsed))
        elif kind == 'watchdog':
            self.watchdogs.append((t, parsed))

    def _status_row(self, s, t, line_bytes):
        rig = self.rig
        angle = None
        cal = self.calibration
        if cal is not None:
            mark = cal['mark_left_mrad'] if s['pos'] >= 0 else cal['mark_right_mrad']
            angle = int(round(s['pos'] * mark / 1000.))
        banner = self.banner or {}
        calline = self.cal or {}
        return dict(mcu_ms=s['ms'], mcu_seq=s['n'], boot_id=self.boot_id, reset_reason=banner.get('rst'),
                    cal_id=calline.get('cal_id', banner.get('cal_id')), state=self.v3.STATE_NAMES.get(s['st']),
                    stale=False, raw_adc=s['raw'], pos_permille=s['pos'], angle_mrad=angle, target_permille=s['tgt'],
                    target_mrad=self.target_mrad, cmd_permille=s['cmd'], steer_pwm=s['sp'], drive_l=s['L'],
                    drive_r=s['R'], drive_en=bool(s['en'] & 1), steer_en=bool(s['en'] & 2), applied_q=s['q'],
                    ttl_remaining_ms=s['tl'], wd_ms=s['wd'], fl='%04X' % s['fl'], faults=self.v3.fault_names(s['fl']),
                    stop_mode=None, received_monotonic=t, received_at=rig.wall_clock_at(t), line_bytes=line_bytes,
                    test=self.test)

    def _keepalive(self):
        now = self.now()
        if self.armed and now - self._last_c >= C_PERIOD_S - 1e-9:
            self._send_c()
        if (self.armed or self.bench) and now - self._last_ping >= PING_PERIOD_S - 1e-9:
            self._write(self.v3.ping_line(), 'PING')
            self._last_ping = now

    # -- writing --------------------------------------------------------------------------------------------------
    def _write(self, data, label):
        self.rig.bench_write(data)
        self.record.add_serial(self.now(), self.rig.wall_clock(), 'h2m', data.decode('ascii').rstrip('\n'), label)

    def _send_c(self):
        self.q = self.q + 1 if self.q < 32767 else 1
        left, right = self.drive_lr
        self._write(self.v3.c_line(self.q, TTL_MS, int(left), int(right), int(self.steer)), 'C')
        self._last_c = self.now()

    def _mark(self):
        """Where the answers to the next command start (list indices; times can repeat within one plant step)."""
        return dict(t=self.now(), lines=len(self.lines), acks=len(self.acks), rejects=len(self.rejects))

    def _new(self, kind, mark):
        return getattr(self, kind)[mark[kind]:]

    def _expect(self, predicate, timeout_s, what, mark):
        """Advance until predicate() or an N line for this command (q '-': a C refusal carries its q); raise
        BenchError on N or timeout."""
        def refused():
            return [r for _, r in self._new('rejects', mark) if r.get('q') is None]

        self.wait_for(lambda: predicate() or bool(refused()), timeout_s)
        bad = refused()
        if bad:
            raise BenchError('%s refused: N - %s' % (what, bad[0].get('reason')))
        if not predicate():
            raise BenchError('%s: no answer within %.2f s' % (what, timeout_s))

    def _cal_after(self, mark, check):
        return any(p['kind'] == 'cal' and check(p) for _, p in self._new('lines', mark))

    def _ack_after(self, mark, check):
        return [a for _, a in self._new('acks', mark) if check(a)]

    def _status_after(self, mark, st):
        return any(p['kind'] == 'status' and p['st'] == st for _, p in self._new('lines', mark))

    # -- commands -------------------------------------------------------------------------------------------------
    def wait_banner(self, timeout_s=1.):
        if self.boot_id is None:
            t0 = self.now()
            self.wait_for(lambda: self.boot_id is not None, timeout_s)
            if self.boot_id is None:
                self.info()
            if self.boot_id is None:
                raise BenchError('no banner within %.2f s' % (self.now() - t0))
        return self.boot_id

    def info(self, timeout_s=.3):
        m = self._mark()
        self._write(self.v3.info_line(), 'INFO')
        self._expect(lambda: self._cal_after(m, lambda c: True), timeout_s, 'INFO', m)

    def cal_get(self, timeout_s=.3):
        m = self._mark()
        self._write(self.v3.cal_get_line(), 'CAL GET')
        self._expect(lambda: self._cal_after(m, lambda c: True), timeout_s, 'CAL GET', m)
        return self.cal

    def stop(self):
        self._write(self.v3.stop_line(), 'STOP')
        self.armed = self.bench = False
        self.drive_lr = (0, 0)
        self.target_mrad = None

    def disarm(self):
        self._write(self.v3.disarm_line(), 'DISARM')
        self.armed = self.bench = False
        self.drive_lr = (0, 0)
        self.target_mrad = None

    def arm(self, steer_permille=None, timeout_s=.3):
        """ARM with the boot id; the first C keeps the steering where it is (or steer_permille)."""
        boot = self.wait_banner()
        m = self._mark()
        self._write(self.v3.arm_line(boot), 'ARM')
        self._expect(lambda: self._status_after(m, 2), timeout_s, 'ARM', m)
        self.steer = int(self.status['pos'] if steer_permille is None else steer_permille)
        self.drive_lr = (0, 0)
        self._send_c()

    def enter_bench(self, timeout_s=.3):
        boot = self.wait_banner()
        m = self._mark()
        self._write(self.v3.bench_line(boot), 'BENCH')
        self._expect(lambda: self._status_after(m, 3), timeout_s, 'BENCH', m)

    def pin(self, n, value):
        m = self._mark()
        self._write(self.v3.pin_line(n, value), 'D%d' % n)
        self._expect(lambda: any(p['kind'] == 'pin' and p['pin'] == n for _, p in self._new('lines', m)), .2,
                     'D%d' % n, m)

    def command(self, steer_permille=None, left=None, right=None, target_mrad=None):
        """Change the C command (ARMED): steering target (permille) and drive duty (-255..255). Sent at once."""
        if not self.armed:
            raise BenchError('C command while not ARMED')
        if steer_permille is not None:
            self.steer = int(max(-1000, min(1000, round(steer_permille))))
            self.target_mrad = target_mrad
        if left is not None or right is not None:
            self.drive_lr = (int(left if left is not None else self.drive_lr[0]),
                             int(right if right is not None else self.drive_lr[1]))
        self._send_c()

    def jog(self, pwm, ms, wait=True):
        """CAL JOG <pwm> <ms>: returns the host write time; with wait, advances until the jog has ended."""
        m = self._mark()
        self._write(self.v3.cal_jog_line(int(pwm), int(ms)), 'CAL JOG')
        self._expect(lambda: self._ack_after(m, lambda a: a.get('what') == 'JOG'), .1, 'CAL JOG', m)
        if wait:
            self.run(ms / 1000. + STEP_S)
        return m['t']

    def mark(self, which):
        m = self._mark()
        key = dict(L='left', C='center', R='right')[which]
        self._write(self.v3.cal_mark_line(which), 'CAL MARK')
        self._expect(lambda: self._cal_after(m, lambda c: c[key] is not None), .2, 'CAL MARK', m)
        return self.cal

    def soft(self, left, right):
        m = self._mark()
        self._write(self.v3.cal_soft_line(int(left), int(right)), 'CAL SOFT')
        self._expect(lambda: self._cal_after(m, lambda c: (c['soft_l'], c['soft_r']) == (int(left), int(right))),
                     .2, 'CAL SOFT', m)
        return self.cal

    def pol(self):
        """CAL POL (spec 7.10, 1000 ms): returns the A POL ack (after the calibration report line that follows it)."""
        m = self._mark()
        self._write(self.v3.cal_pol_line(), 'CAL POL')
        self._expect(lambda: self._ack_after(m, lambda a: a.get('what') == 'POL'), 1.2, 'CAL POL', m)
        after = dict(m)
        after['lines'] = next(i for i in range(m['lines'], len(self.lines))
                              if self.lines[i][1]['kind'] == 'ack' and self.lines[i][1].get('what') == 'POL')
        self._expect(lambda: self._cal_after(after, lambda c: True), .2, 'CAL POL report', m)
        return self._ack_after(m, lambda a: a.get('what') == 'POL')[-1]

    def save(self):
        m = self._mark()
        self._write(self.v3.cal_save_line(), 'CAL SAVE')
        self._expect(lambda: self._cal_after(m, lambda c: c['cal_id'] is not None and not c['caldirty']), .3,
                     'CAL SAVE', m)
        return self.cal

    def cfg_set(self, key, value):
        m = self._mark()
        self._write(self.v3.cfg_set_line(key, int(value)), 'CFG')
        self._expect(lambda: self._ack_after(m, lambda a: a.get('what') == 'CFG' and a.get('key') == key), .2,
                     'CFG %s' % key, m)

    def cfg_get(self, key):
        m = self._mark()
        self._write(self.v3.cfg_get_line(key), 'CFG')
        self._expect(lambda: self._ack_after(m, lambda a: a.get('what') == 'CFG' and a.get('key') == key), .2,
                     'CFG %s' % key, m)
        return self._ack_after(m, lambda a: a.get('what') == 'CFG' and a.get('key') == key)[-1]['value']

    # -- THERMAL budget (spec 6.7: THERMAL when |steer PWM| >= thp for more than tha ms in the last thw s) ---------
    def high_pwm_s(self):
        """Seconds with |steering pin| >= thp in the last thw s, estimated from the status lines (MCU time)."""
        if self.thermal is None:
            self.thermal = (self.cfg_get('thp'), self.cfg_get('thw'), self.cfg_get('tha'))
        thp, thw, _ = self.thermal
        if not self.pwm_history:
            return 0.
        rows = list(self.pwm_history)
        last = rows[-1][0]
        total = 0.
        for (m0, _), (m1, sp) in zip(rows[:-1], rows[1:]):
            if m1 < last - thw * 1000.:
                continue
            if abs(sp) >= max(thp, 1):
                total += min(m1 - m0, 60.) / 1000.
        return total

    def cool_down(self, limit_s=None, timeout_s=40.):
        """The operator waits while the THERMAL proxy count is high (keeps strong tests below the MCU budget):
        until the count is at most limit_s (default THERMAL_SHARE x tha)."""
        if self.thermal is None:
            self.high_pwm_s()
        limit = THERMAL_SHARE * self.thermal[2] / 1000. if limit_s is None else float(limit_s)
        if self.high_pwm_s() <= limit:
            return 0.
        t0 = self.now()
        self.note('cool_down', high_pwm_s=round(self.high_pwm_s(), 2), limit_s=limit)
        self.wait_for(lambda: self.high_pwm_s() <= limit, timeout_s)
        return self.now() - t0

    # -- helpers for the procedures ----------------------------------------------------------------------------------
    def status_rows_since(self, since):
        return [row for row in self.record.status_rows if row['received_monotonic'] >= since]

    def settled(self, quiet_s=.3, timeout_s=6., min_s=.25):
        """Advance at least min_s, then until the steering PWM has been 0 in every status line of the last quiet_s
        (servo braked or output off), or timeout."""
        start = self.now()

        def quiet():
            now = self.now()
            if now - start < max(min_s, quiet_s) - 1e-9:
                return False
            rows = self.status_rows_since(now - quiet_s)
            return len(rows) >= 2 and all(r['steer_pwm'] == 0 for r in rows)
        return self.wait_for(quiet, timeout_s)
