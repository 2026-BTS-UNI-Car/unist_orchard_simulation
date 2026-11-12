"""Helpers for the test_steer_* host tests: import path, a host profile literal and a scripted fake V3 MCU.

The fake MCU is a small hand-written model of the V3 behaviour the bridge relies on (banner and cal line at boot,
status lines every 1000/sthz ms from the boot instant, STOP, ARM, DISARM, C with the q rule, TTL expiry, PING,
INFO, N lines). It is not the firmware and not a conformance model; conformance is the firmware team's job.
No hardware, no files. This module has no test cases.
"""
import os
import sys

_DASHBOARD = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _DASHBOARD not in sys.path:
    sys.path.insert(0, _DASHBOARD)

from scripts import esp_motor_v3 as v3  # noqa: E402

BOOT_IDS = ('89025CC1', '658EEC67', 'FB32555E', '7B1DCDAF', 'A1B965F4', '8009454F')


def host_profile_data(**sections):
    """A valid host profile dict with the placeholder numbers of front_steer_v0_placeholder (hand-copied).

    Keyword arguments replace whole sections or top-level values, e.g. link={...}.
    """
    data = {
        'schema': 'ubts.steer_host_profile.v1',
        'vehicle_profile': 'front_steer',
        'hw_profile': 'front_steer_v0',
        'source_profile': {'profile_id': 'front_steer_v0_placeholder', 'profile_sha256': '0' * 64,
                           'provenance_grade': 'placeholder', 'result_kind': 'placeholder SIM result (test literal)',
                           'field_validated': False, 'overrides': None},
        'steer_limit': {'left_mrad': 409, 'right_mrad': 409, 'left_rad': 0.40940306368377555,
                        'right_rad': 0.40940306368377555},
        'calibration': {'cal_id': 7, 'mark_left_mrad': 438.030759092455, 'mark_right_mrad': 438.0307590924551,
                        'permille_per_deg': 39.844901659655896, 'expected_soft_permille': {'left': 935, 'right': -935},
                        'rule': 'test literal'},
        'link': {'baud': 115200, 'status_hz': 20, 'c_ttl_max_ms': 300, 'c_resend_hz': 20.0, 'ping_hz': 10.0,
                 'watchdog_ms': 500, 'banner_timeout_s': 1.5},
        'stop': {'confirm_periods': 2, 'resend_interval_ms': 50.0, 'resend_max': 5, 'steer_policy': 'hold_measured',
                 'hold_after_stop_ms': 2000, 'hold_after_disarm_ms': 2000, 'hold_after_loss_ms': 2000,
                 'watchdog_steer_mode': 'hold', 'ttl_expiry_drive_mode': 'brake'},
        'arm': {'confirm_ms': 150.0},
        'host_faults': {'status_gap_ms': 150.0, 'seq_gap_missing': 5, 'seq_gap_window_s': 1.0,
                        'track_error_deg': 3.0, 'track_duration_s': 1.5,
                        'ineffective': {'min_speed_m_s': 0.2, 'min_steer_deg': 3.0, 'rate_error_deg_s': 3.0,
                                        'rate_error_frac': 0.5, 'duration_s': 1.5}},
        'servo': {'deadband_permille': 20, 'max_pwm': 255},
        'drive': {'lr_mode': 'equal_duty'},
        'vehicle': {'wheelbase_m': 0.76},
        'provenance': {'note': 'test literal'},
    }
    data.update(sections)
    return data


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


class FakeMcu:
    """Scripted V3 device on the device end of a MemoryTransport. Call tick(now) once per test step."""

    def __init__(self, transport, *, sim=True, hw='front_steer_v0', cal_id=7, pol_ok=True, caldirty=False,
                 sthz=20, wd=500, ttlmax=300, reset_on_open=True, boot_delay_s=0.0, start_armed=False,
                 cfgdirty=False):
        self.transport = transport
        self.device = transport.device
        self.sim, self.hw, self.cal_id, self.pol_ok, self.caldirty = sim, hw, cal_id, pol_ok, caldirty
        self.cfgdirty = cfgdirty
        self.sthz, self.wd, self.ttlmax = sthz, wd, ttlmax
        self.boot_delay_s = boot_delay_s
        self.boots = 0
        self.boot_id = None
        self.boot_t = None
        self.pending_boot_t = None
        # behaviour switches used by the tests
        self.ignore_stop = False
        self.send_status = True
        self.drop_status = set()        # status numbers n that are generated but not sent
        self.ms_jump = 0                # added to every ms value (a regression when negative)
        self.follow_rate_pm_s = 4000.0  # pos follows cmd (permille per second)
        self.stuck_pos = None           # pos stays here when set
        self.cmd_rate_pm_s = None       # None: cmd jumps to the target; a rate: the MCU servo ramp (spec 7.1)
        self.watchdog = False           # apply the link watchdog (off by default to keep tests short)
        self.ignore_arm = False         # ARM is silently ignored (no N line, no state change)
        self.arm_reject = None          # ARM is answered with N - <this reason>
        self.start_armed = start_armed  # boot straight into ARMED (a stuck or foreign device)
        self.boot_banner = True         # False: the boot banner is lost on the way (merged with garbage)
        self.boot_cal_line = True       # False: the cal line after the boot banner is lost
        self.sent_lines = []            # (t, line bytes)
        self.rx_lines = []              # (t, host line bytes without LF)
        self.rx_buffer = b''
        self.now = 0.0
        if reset_on_open:
            transport.on_open.append(self.port_open_reset)

    # -- lifecycle
    def port_open_reset(self):
        self.reset(self.transport.clock())

    def reset(self, now):
        self.boot_id = None
        self.pending_boot_t = now + self.boot_delay_s
        self.rx_buffer = b''
        if self.boot_delay_s == 0.0:
            self._boot(now)

    def _boot(self, now):
        self.boot_id = BOOT_IDS[self.boots % len(BOOT_IDS)]
        self.boots += 1
        self.boot_t = now
        self.pending_boot_t = None
        self.st, self.last_q, self.L, self.R = 1, 0, 0, 0
        self.tgt = self.cmd = self.pos = 0
        self.sp = 0
        self.steer_en = 0
        self.fl = 0x4000 | (0 if self.cal_id is not None else 0x0001)
        self.c_t = None
        self.c_ttl = 0
        self.ttl_count = 0
        self.last_input = now
        self.n = 0
        self.next_status = now + 1.0 / self.sthz
        self.hold_until = None
        if self.start_armed:
            self.st, self.steer_en = 2, 1
        if self.boot_banner:
            self._send(now, self.banner())
        if self.boot_cal_line:
            self._send(now, self.cal_line())

    def banner(self):
        return ('UBTS_STEER_V3 hello proto=3 build=0A1B2C3D dev=5A1B2C boot=%s rst=power_on hw=%s '
                'out=drive2,steer1 cal=%s sim=%d wd=%d ttl=%d st=%d drop=0'
                % (self.boot_id, self.hw, 'none' if self.cal_id is None else self.cal_id, int(self.sim), self.wd,
                   self.ttlmax, self.sthz))

    def cal_line(self):
        if self.cal_id is None:
            return ('UBTS_STEER_V3 cal id=none crc=00000000 left=- center=- right=- soft=0,0 pol=no mdir=1 '
                    'caldirty=0 cfgdirty=0')
        return ('UBTS_STEER_V3 cal id=%d crc=61AEB1A4 left=842 center=512 right=181 soft=935,-935 pol=%s mdir=1 '
                'caldirty=%d cfgdirty=%d' % (self.cal_id, 'ok' if self.pol_ok else 'no', int(self.caldirty),
                                             int(self.cfgdirty)))

    def _send(self, now, body):
        line = v3.mcu_line(body)
        self.sent_lines.append((now, line))
        self.device.send(line)

    def ms(self, now):
        return (int(round((now - self.boot_t) * 1000)) + self.ms_jump) % 4294967296

    # -- one step
    def tick(self, now):
        dt = now - self.now
        self.now = now
        if self.pending_boot_t is not None and now >= self.pending_boot_t:
            self._boot(now)
        data = self.device.take()
        if self.boot_id is None:
            return
        self.rx_buffer += data
        while b'\n' in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split(b'\n', 1)
            self.rx_lines.append((now, line))
            self._execute(now, line)
        if self.watchdog and self.st in (2, 3) and now - self.last_input > self.wd / 1000.0:
            self.st, self.L, self.R = 1, 0, 0
            self.fl |= 0x1000
            self.c_t = None
            self._send(now, 'W %d' % self.ms(now))
        if self.st == 2 and self.c_t is not None and now - self.c_t >= self.c_ttl / 1000.0:
            self.L = self.R = 0
            self.c_t = None
            self.fl |= 0x0800
            self.ttl_count += 1
            self._send(now, 'F TTL %d %d' % (self.ttl_count, self.ms(now)))
        if self.hold_until is not None and now >= self.hold_until:
            self.hold_until = None
            self.steer_en = 0
        target = self.cmd if self.stuck_pos is None else self.stuck_pos
        if self.stuck_pos is not None:
            self.pos = self.stuck_pos
        else:
            step = self.follow_rate_pm_s * max(dt, 0.0)
            self.pos = min(target, self.pos + step) if self.pos < target else max(target, self.pos - step)
        if self.st == 2:
            goal = max(-935, min(935, self.tgt))
            if self.cmd_rate_pm_s is None:
                self.cmd = goal
            else:
                step = int(self.cmd_rate_pm_s * max(dt, 0.0))
                self.cmd = min(goal, self.cmd + step) if self.cmd < goal else max(goal, self.cmd - step)
        if now + 1e-9 >= self.next_status:
            self.next_status += 1.0 / self.sthz
            n = self.n
            self.n = (self.n + 1) % 65536
            if self.send_status and n not in self.drop_status:
                self._send(now, self.status_body(now, n))

    def status_body(self, now, n):
        en = 1 | (2 if self.steer_en else 0)
        pos = int(round(self.pos))
        tl = 0
        if self.st == 2 and self.c_t is not None:
            tl = max(0, int(round(self.c_ttl - (now - self.c_t) * 1000)))
        wd = min(9999, int(round((now - self.last_input) * 1000)))
        raw = max(0, min(1023, 512 + int(round(pos * 0.33))))
        tgt = self.tgt if self.st == 2 else pos
        cmd = self.cmd if self.st == 2 else pos
        return 'S %d %d %d %d %d %d %d %d %d %d %d %d %04X %d %d' % (
            self.ms(now), n, self.st, self.last_q, self.L, self.R, en, raw, pos, tgt,
            cmd, self.sp, self.fl, wd, min(1000, tl))

    def _reject(self, now, q, reason):
        self._send(now, 'N %s %s' % ('-' if q is None else q, reason))

    def _execute(self, now, line):
        if line.endswith(b'\r'):
            line = line[:-1]
        if not line:
            return
        text = line.decode('ascii', 'replace')
        if text == 'STOP' or (len(text) == 7 and text.startswith('STOP*')):
            if not self.ignore_stop:
                if self.st == 2:
                    self.hold_until = now + 2.0
                self.st, self.L, self.R = 1, 0, 0
                self.c_t = None
                self.fl &= ~0x0800
            return
        body, star, crc = text.partition('*')
        if star:
            if len(crc) != 2 or v3.crc8(body.encode('ascii')) != int(crc, 16):
                self._reject(now, None, 'crc')
                return
        tokens = body.split(' ')
        head = tokens[0]
        if head in ('ARM', 'DISARM', 'C', 'CAL', 'CFG', 'BENCH') and not star:
            self._reject(now, None, 'nocrc')
            return
        if head == 'PING':
            self.last_input = now
        elif head == 'INFO':
            self._send(now, self.banner())
            self._send(now, self.cal_line())
        elif head == 'ARM':
            if self.ignore_arm:
                return
            if self.arm_reject:
                self._reject(now, None, self.arm_reject)
            elif self.st != 1:
                self._reject(now, None, 'state')
            elif len(tokens) != 2 or tokens[1] != self.boot_id:
                self._reject(now, None, 'boot')
            elif self.cal_id is None:
                self._reject(now, None, 'nocal')
            elif self.caldirty:
                self._reject(now, None, 'cal')
            elif not self.pol_ok:
                self._reject(now, None, 'pol')
            else:
                self.st = 2
                self.tgt = self.cmd = int(round(self.pos))
                self.steer_en = 1
                self.hold_until = None
                self.fl &= 0x0080
                self.last_input = now
        elif head == 'DISARM':
            if self.st == 2:
                self.hold_until = now + 2.0
            self.st, self.L, self.R = 1, 0, 0
            self.c_t = None
        elif head == 'C':
            q = int(tokens[1]) if len(tokens) == 6 and tokens[1].isdigit() else None
            if self.st != 2:
                self._reject(now, q, 'state')
                return
            d = (q - self.last_q) % 65536
            if not 1 <= d <= 32767:
                self._reject(now, q, 'old')
                return
            self.last_q = q
            self.c_ttl = int(tokens[2])
            self.L, self.R, self.tgt = int(tokens[3]), int(tokens[4]), int(tokens[5])
            self.c_t = now
            self.fl &= ~0x0800
            self.last_input = now
        elif head == 'CAL' and tokens[1:] == ['GET']:
            self._send(now, self.cal_line())
        else:
            self._reject(now, None, 'cmd')


class Rig:
    """Clock + MemoryTransport + FakeMcu + EspSteerV3 + SteerBridgeCore, stepped together."""

    def __init__(self, step_s=0.01, latency_s=0.0, simulated=True, profile=None, clock=None, **mcu):
        from scripts import steer_bridge_core as core_mod
        self.core_mod = core_mod
        self.clock = clock or Clock()
        self.step_s = step_s
        self.transport = v3.MemoryTransport(clock=self.clock, latency_s=latency_s)
        self.mcu = FakeMcu(self.transport, **mcu)
        self.hp = core_mod.HostProfile(profile or host_profile_data())
        self.link = v3.EspSteerV3(self.transport, self.clock, simulated=simulated, expect_hw=self.hp.hw_profile,
                                  banner_timeout_s=self.hp.banner_timeout_s)
        self.core = core_mod.SteerBridgeCore(self.link, self.hp, clock=self.clock, wall_clock=lambda: 1.0e9)

    def tick(self):
        self.clock.t = round(self.clock.t + self.step_s, 9)
        self.mcu.tick(self.clock.t)
        self.core.step()

    def run(self, seconds):
        for _ in range(int(round(seconds / self.step_s))):
            self.tick()

    def run_until(self, predicate, limit_s=5.0):
        for _ in range(int(round(limit_s / self.step_s))):
            if predicate():
                return True
            self.tick()
        return predicate()

    def ready(self, limit_s=1.0):
        """Run until the link is up and the session-start STOP is confirmed."""
        ok = self.run_until(lambda: self.core.steering_host()['stop_state'] == 'confirmed', limit_s)
        if not ok:
            raise AssertionError('rig did not become ready: %r' % (self.core.steering_host(),))

    def submit(self, request):
        future = self.core.submit(request)
        self.tick()
        if not future.done():
            raise AssertionError('request not answered in one tick')
        return future.result()

    def tx(self, since=0):
        """Host lines received by the fake MCU after index since (LF removed)."""
        return [line for _, line in self.mcu.rx_lines[since:]]

    def tx_kinds(self, since=0, skip=(b'PING',)):
        out = []
        for line in self.tx(since):
            head = line.split(b' ')[0].split(b'*')[0]
            if head not in skip:
                out.append(head.decode('ascii'))
        return out

    def arm(self, owner='operator-a'):
        reply = self.submit({'op': 'arm', 'owner': owner, 'vehicle_profile': 'front_steer'})
        if not reply.get('ok'):
            raise AssertionError('arm failed: %r' % (reply,))
        self.run_until(lambda: self.core.steering_host()['arm_state'] == 'confirmed', 0.5)
        return reply

    @staticmethod
    def next_request(reply, op, owner='operator-a', **extra):
        return {'op': op, 'owner': owner, 'lease': reply['lease'], 'ticket': reply['ticket'],
                'seq': reply['seq'] + 1, **extra}
