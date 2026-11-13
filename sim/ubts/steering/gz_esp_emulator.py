"""ESP8266 V3 emulator of the G-SIM (design 2.2-2.6, 4.3 and 8.3 P6a; docs/steering/P6A_HEADLESS_KO.md).

The plant side of the T2 path: the host bridge wrapper (dashboard/scripts/steer_rover_bridge.py, PtyTransport) opens
the slave end of a pseudo terminal behind the link /run/ubts-steer/esp-tty. Behind it this process runs the steering
firmware (firmware/ubts_steer_v3) and the shared C++ actuator and potentiometer model in the native lockstep v2
firmware mode (actuator_link.FirmwareSteeringBackend, the Linux build of the image) on the SIM clock:

  /clock (ROS, from ros_gz_bridge)   each message ends one Gazebo physics step (120 Hz: 8,333,333 ns). The MCU runs
                                     to clock + lead_steps x step (default one step ahead, so the joint targets reach
                                     Gazebo before the step that should end at them). The native tick rule keeps the
                                     accumulated tick count exact: 1/120 s gives 8, 9, 8 ticks (LOCKSTEP 5, 11.4).
                                     A gap above 0.1 s is run in FW_STEP frames of at most 0.1 s.
  /ground_truth/odom (truth)          vehicle speed and standstill flag (|v| < 0.01 m/s) of the actuator model: the
                                     emulator is plant side and may read truth (design 2.6). The potentiometer value
                                     comes from the shared model's angle only, never from /sim/truth/joint_states.
  pty master                          host bytes are scheduled at the next MCU tick with the UART arrival rule
                                     (LOCKSTEP 11.7, 115200 baud); MCU bytes are written per whole line at the end of
                                     the FW_STEP in which the line's LF left the wire (as the T1 rig delivers them).
  /sim/act/* (ROS Float64 -> gz Double through bridge_steer.yaml)
                                     the model's wheel angles (delta_L, delta_R; axle_pivot: delta_eq) as steering
                                     joint targets, and per driven wheel the velocity target duty x omega0 (force
                                     mode, gz_model / drive.dc_motor_line; left duty for the left wheels). Drive EN
                                     off (coast, only while the MCU resets) cannot be represented in Gazebo: the
                                     target is 0 (the DC line brakes), counted as coast_steps_as_brake.
  /sim/truth/steering (truth)         true delta_L, delta_R, delta_eq, motor-side angle, driver output, pot raw, the
                                     MCU pins and state (std_msgs/String JSON, schema ubts.steer_truth.v1); for
                                     evaluation and recording only (A13).

Banner: the SIM build of the firmware prints sim=1, the device id (chip id from the seed) and the hardware profile
(compile constant front_steer_v0). The emulator only changes a banner through the banner_rewrite hook (A14 tests).

Port events: the slave starts closed (the emulator does not keep it open). The host opening the slave clears the
master's hangup; with port_open_reset (default on, the NodeMCU DTR/RTS auto reset, ESP spec 10.7) the MCU resets
(reason ext_sys) at the next tick. While no host has the slave open, MCU lines are dropped (counted).
The pty link appears only after --pty-after-clock-s (default 1 s) of SIM time with a steady /clock (no wall gap above
50 ms; at most 20 s wall after the first /clock): the supervisor starts the bridge wrapper once the link exists, so
the bridge opens the port after the emulator's own start (DDS discovery, the native program, the first callbacks).
The MCU runs from the first /clock either way; the port open resets it (WF-4 g_gaps: the start-up host latches of
phase 3 came while the emulator was not yet stepping steadily).

Fault hooks (A17(G), A19(G); a Unix socket outside the shared control volume, default /run/ubts-steer-emu/hooks.sock,
one JSON object per line, one reply line): status, mark, event (mcu_reset, power_cycle, port_open_reset, motor_power,
mcu_stall, adc_set, adc_model), adc_pattern, adc_ramp, link (loss to_mcu / to_host / both for a SIM duration),
status_loss, corrupt (crc / range of the next host lines), half_frame, inject_line (a foreign writer on the MCU RX),
banner_rewrite (sim / hw fields of the next banners), restart (a new native session: EEPROM calibrated or blank, v1
model injections such as polarity_reversed, CFG record overrides, cal_id, report_adc, report_servo), unplug (USB
unplug and replug: the pty master is closed and a new pty is made after the duration; mcu_power cycle when the board
is USB powered), record (A3(G), A19(G) rule ticks: every FW_STEP input and output of N SIM seconds into a JSON file
for an offline replay; the session it starts switches report_adc on by default and report_servo on (the
evaluation-only servo channels for the TRACK rule tick), so that the record has every tick's analogRead value and
the firmware's P, tgt, cmd and steering mode), record_finish (write the running record now) and kill_bridge (A17
mandatory fault test 'bridge forced kill', design 7.2.1: SIGKILL to the bridge wrapper child of the steer supervisor
in this container; the supervisor restarts it, its port open resets the MCU).

Logs: a rotating JSONL event log (serial lines both ways with ticks and the monotonic time of the pty read or write
(mono: the bridge wrapper in the same container uses the same clock in its wall clock mode, for the A19(G) host latch
timing), pin transitions, hooks, port and session events) and /runtime/steering_status.json (emulator state + the
bridge status file, every 0.5 s wall), with the emulator's own load: process CPU, the /clock callback time histogram,
the /sim/act publish counts.

CPU (WF-4 g_gaps): the core costs a few % of one CPU (offline profile); most of the emulator's CPU is the rclpy side
(callbacks at 120 Hz, publishes). A /sim/act/* value that did not change since its last publish is not published
again (the Gazebo joint controllers keep their last target) except every ACT_REFRESH_STEPS steps; the MCU, the model
and the targets Gazebo applies are the same. --truth-every N publishes /sim/truth/steering every N steps (default 1).

The firmware backend here keeps no unbounded history (BoundedFirmwareBackend): a G-SIM run lasts hours.
Every number comes from the placeholder vehicle profile: results are placeholder-based SIM results
(자리값 기반 SIM 결과), not field validation. CLI: python3 -B -m steering.gz_esp_emulator --help (sim/ubts on the path).
"""
from __future__ import annotations

import argparse
import collections
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import socketserver
import sys
import threading
import time

from . import backend_api as ba
from . import drive as drive_model
from . import geometry as geo
from . import gz_model
from .actuator_link import CHANNEL_RESET_VALUES, FirmwareSteeringBackend, crc8, host_line
from .native import fw_eeprom
from .native import lockstep_v2 as v2
from .vehicle_profile import load_profile

SCHEMA_TRUTH = 'ubts.steer_truth.v1'
SCHEMA_STATUS = 'ubts.steer_emulator_status.v1'
SCHEMA_LOG = 'ubts.steer_emulator_log.v1'
SCHEMA_RECORD = 'ubts.steer_emulator_record.v1'
RESULT_KIND = '자리값 기반 SIM 결과'
SIM_ROOT = '/run/ubts-steer'
DEFAULT_PTY_LINK = SIM_ROOT + '/esp-tty'
DEFAULT_HOOK_SOCKET = '/run/ubts-steer-emu/hooks.sock'
DEFAULT_STATUS_FILE = '/runtime/steering_status.json'
DEFAULT_LOG_FILE = '/runtime/steer/emulator_log.jsonl'
DEFAULT_BRIDGE_STATUS_FILE = '/runtime/rover_bridge_status.json'
TRUTH_TOPIC = '/sim/truth/steering'
CLOCK_TOPIC = '/clock'
TRUTH_ODOM_TOPIC = '/ground_truth/odom'
NODE_NAME = 'ubts_steer_esp_emulator'
STANDSTILL_SPEED_M_S = .01               # steered_plant / gz_probe rule: below this the standstill rates apply
BOOT_DELAY_TICKS = 70                    # reset -> setup(); T1 rig placeholder (ESP spec 1: about 70 ms)
MAX_CHUNK_NS = 100000000                 # FW_STEP dt limit 0.1 s (LOCKSTEP 11.4)
PATTERN_CHUNK_NS = 50000000              # with per-tick ADC events a FW_STEP carries at most 50 (limit 64)
STATUS_PERIOD_S = .5
LOG_ROTATE_BYTES = 64 * 1024 * 1024
MAX_HOOK_LINE = 65536
PIN_CHANNELS = ('drive_l', 'drive_r', 'drive_en', 'steer_en', 'mcu_state', 'fault_bits')
CLOCK_GAP_BINS_MS = (20, 50, 100)        # /clock gaps (SIM time between messages) counted in the status
CLOCK_STALL_MS = 100                     # /clock wall gaps above this are kept with their times (clock_stalls)
CLOCK_STALLS_KEPT = 64
SLOW_WINDOW_S = .3                       # SIM time received per monotonic second over this window (clock_slow)
SLOW_RTF = .9                            # below this the window is slow (A10: RTF < 0.9)
SLOW_KEPT = 64
PURPOSES = dict(boot=201, chip=202)      # t1_rig stream purposes (mcu_boot, mcu_chip), same derivation rule
ACT_REFRESH_STEPS = 12                   # unchanged /sim/act/* values are published again every 12 steps (0.1 s)
PTY_AFTER_CLOCK_S = 1.                   # the pty link appears after this much SIM time of steady /clock
PTY_STEADY_GAP_S = .05                   # a /clock wall gap above this restarts the steady time
PTY_FALLBACK_WALL_S = 20.                # the link appears at the latest this long (wall) after the first /clock
CALLBACK_BINS_MS = (1, 2, 5, 10, 20, 50, 100)
DEFAULT_SUPERVISOR_STATE = '/runtime/steer/supervisor_steer.json'
BRIDGE_MODULE = 'scripts.steer_rover_bridge'
FAULT_NAMES = ('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'SOFTLIMIT',
               'STALL', 'TRACK', 'THERMAL', 'TTL', 'WATCHDOG', 'CRC', 'BOOT')


class EmulatorError(RuntimeError):
    """The emulator cannot start or a hook request is wrong."""


def fault_bit_names(bits):
    """Names of the fl bits (ESP spec 5.3: bit i = FAULT_NAMES[i])."""
    return [name for index, name in enumerate(FAULT_NAMES) if bits >> index & 1]


def pwm_level(pwm, levels):
    """Steering PWM level of the pin log: 0 = 0, 1 = below stp, 2 = stp..thp, 3 = at least thp (the STALL and THERMAL
    output thresholds of the CFG record, ESP spec 6.7)."""
    magnitude = abs(int(pwm))
    if magnitude == 0:
        return 0
    return 1 if magnitude < levels[0] else (2 if magnitude < levels[1] else 3)


def stable_u32(seed, purpose):
    """Deterministic 32-bit value from (seed, purpose) without numpy (splitmix64 over a fixed mix)."""
    state = ((int(seed) & 0xFFFFFFFF) << 16 ^ int(purpose)) & 0xFFFFFFFFFFFFFFFF
    return v2.sim_boot_ids(state, 1)[0]


def physics_step_ns(hz):
    """Gazebo's step in ns: 1/hz truncated to whole ns (gz 8.15 keeps 8,333,333 ns at 120 Hz)."""
    return int(1e9 / float(hz))


# --- firmware backend without unbounded history ------------------------------------------------------------------------
class BoundedFirmwareBackend(FirmwareSteeringBackend):
    """FirmwareSteeringBackend for long runs: keeps the last FW_STEP output and the current channel values only
    (the parent keeps every MCU byte and change in lists). lines(), read_new_bytes(), channel() and channel_at() of
    the parent are not used with this class. recorder: an InputRecorder that gets every FW_STEP input and output."""

    def __init__(self, *args, **kwargs):
        if not hasattr(FirmwareSteeringBackend, '_record'):
            raise EmulatorError('actuator_link.FirmwareSteeringBackend has no _record(); this emulator needs it')
        self.recorder = None
        super().__init__(*args, **kwargs)

    def _reset_session_state(self):
        super()._reset_session_state()
        self.channel_values = dict(CHANNEL_RESET_VALUES)

    def _record(self, out):
        self.last = out
        for change in out.changes:
            self.channel_values[change.channel] = change.value

    def fw_step(self, dt_s, *, events=(), rx=(), vehicle_speed_m_s=0., vehicle_stopped=True, supply_scale=1.):
        out = super().fw_step(dt_s, events=events, rx=rx, vehicle_speed_m_s=vehicle_speed_m_s,
                              vehicle_stopped=vehicle_stopped, supply_scale=supply_scale)
        recorder = self.recorder
        if recorder is not None:
            recorder.add(dt_s, events, rx, vehicle_speed_m_s, vehicle_stopped, supply_scale, out)
        return out


def output_record(out):
    """A JSON-friendly copy of one FwStepOutput (replay comparison, A3(G))."""
    b = out.base
    return dict(first_tick=out.first_tick, ticks=out.ticks, sim_time_s=b.sim_time_s, delta_left_rad=b.delta_left_rad,
                delta_right_rad=b.delta_right_rad, delta_eq_rad=b.delta_eq_rad,
                motor_side_angle_rad=b.motor_side_angle_rad, pot_raw=b.pot_raw, applied_steer_pwm=b.applied_steer_pwm,
                steer_en=b.steer_en, fault_bits=b.fault_bits, model_flags=b.model_flags,
                drive_duty_left=b.drive_duty_left, drive_duty_right=b.drive_duty_right, drive_en=b.drive_en,
                mcu_state=out.mcu_state, run_flags=out.run_flags, boot_count=out.boot_count,
                mcu_millis=out.mcu_millis, rx_lost_total=out.rx_lost_total,
                changes=[[c.tick, c.channel, c.value] for c in out.changes],
                tx=[[t.queued_tick, t.done_tick, t.byte, int(t.boot_garbage)] for t in out.tx])


class InputRecorder:
    """Every FW_STEP of a session for `ticks` ticks: the exact request values and the full response (A3(G))."""

    def __init__(self, path, ticks, session_info):
        self.path = Path(path)
        self.ticks = int(ticks)
        self.session = dict(session_info)
        self.frames = []
        self.done = False
        self.counted = 0

    def add(self, dt_s, events, rx, speed, stopped, supply, out):
        if self.done:
            return
        self.frames.append(dict(dt_s=float(dt_s).hex(), dt_s_float=float(dt_s), vehicle_speed_m_s=float(speed).hex(),
                                vehicle_stopped=bool(stopped), supply_scale=float(supply).hex(),
                                events=[[int(o), int(v2.event_code(k)), int(a)] for o, k, a in events],
                                rx=[[int(o), int(b)] for o, b in rx], out=output_record(out)))
        self.counted += out.ticks
        if self.counted >= self.ticks:
            self.finish()

    def finish(self):
        if self.done:
            return
        self.done = True
        payload = dict(schema=SCHEMA_RECORD, result_kind=RESULT_KIND, field_validated=False, session=self.session,
                       frames=self.frames, ticks=self.counted, frame_count=len(self.frames))
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_name(self.path.name + '.tmp')
        tmp.write_bytes((json.dumps(payload, allow_nan=False) + '\n').encode('utf-8'))
        os.replace(tmp, self.path)


def replay_record(record, executable=None):
    """Run a recorded session again in a fresh firmware backend (the F-SIM / T1 path) and compare every FW_STEP
    response with the recorded one. Returns a summary dict (A3(G): same inputs -> same raw values)."""
    session = record['session']
    profile = load_profile(session['profile_path']) if session.get('profile_path') else load_profile()
    if profile.sha256 != session['profile_sha256']:
        raise EmulatorError('replay profile sha256 %s differs from the recorded %s' % (
            profile.sha256, session['profile_sha256']))
    backend = FirmwareSteeringBackend(profile, session['seed'], firmware=session['firmware'],
                                      injections=session['injections'], executable=executable)
    try:
        if backend.config != session['startup']:
            raise EmulatorError('replay startup line differs from the recorded one')
        mismatched, compared, raw_pairs, adc_pairs = [], 0, 0, 0
        for index, frame in enumerate(record['frames']):
            out = backend.fw_step(float.fromhex(frame['dt_s']), events=[tuple(e) for e in frame['events']],
                                  rx=[tuple(r) for r in frame['rx']],
                                  vehicle_speed_m_s=float.fromhex(frame['vehicle_speed_m_s']),
                                  vehicle_stopped=frame['vehicle_stopped'],
                                  supply_scale=float.fromhex(frame['supply_scale']))
            got, want = output_record(out), frame['out']
            compared += 1
            raw_pairs += 1
            adc_pairs += sum(1 for c in want['changes'] if c[1] == 'adc_read')
            if got != want:
                keys = sorted(k for k in want if got.get(k) != want.get(k))
                mismatched.append(dict(frame=index, keys=keys))
                if len(mismatched) >= 20:
                    break
        return dict(frames=len(record['frames']), compared=compared, mismatched_frames=len(mismatched),
                    first_mismatches=mismatched, pot_raw_values_compared=raw_pairs,
                    adc_read_changes_compared=adc_pairs, ticks=record['ticks'],
                    replay_build_id=backend.build_id, recorded_build_id=session.get('build_id'),
                    same_build=backend.build_id == session.get('build_id'),
                    replay_executable_sha256=backend.executable_sha256, passed=not mismatched)
    finally:
        backend.close()


# --- serial line helpers ---------------------------------------------------------------------------------------------
class LineAssembler:
    """MCU bytes (FwTxByte) -> whole lines (data with LF, first queued tick, LF done tick, boot garbage flag)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.buf = bytearray()
        self.first = None
        self.garbage = False

    def feed(self, tx):
        lines = []
        for entry in tx:
            if self.first is None:
                self.first = entry.queued_tick
            self.buf.append(entry.byte)
            self.garbage = self.garbage or entry.boot_garbage
            if entry.byte == 0x0A:
                lines.append((bytes(self.buf), self.first, entry.done_tick, self.garbage))
                self.reset()
        return lines


def line_text(data):
    return data.rstrip(b'\n').decode('ascii', 'replace')


def recrc_line(body):
    """`body*HH\\n` with the CRC of body (body without CRC field and LF)."""
    data = body.encode('ascii') if isinstance(body, str) else bytes(body)
    return data + b'*%02X\n' % crc8(data)


def rewrite_banner(data, fields):
    """A banner line with fields (e.g. {'sim': '0', 'hw': 'front_steer_x'}) replaced and a new CRC. Other lines and
    banners without the field are returned unchanged."""
    text = line_text(data)
    if not text.startswith('UBTS_STEER_V3 hello '):
        return data
    body = text.rsplit('*', 1)[0]
    parts = body.split(' ')
    out = []
    for part in parts:
        key, sep, _ = part.partition('=')
        if sep and key in fields:
            part = '%s=%s' % (key, fields[key])
        out.append(part)
    return recrc_line(' '.join(out))


def corrupt_host_line(data, kind):
    """A corrupted host line (T1 rig rule): 'range' turns the S field of a C line into 2000 with a correct CRC (the
    MCU refuses it as N q range); 'crc' flips one content byte and keeps the CRC field (a line without one gets a
    wrong CRC field). STOP lines are never changed. Returns (data, applied)."""
    if not data.endswith(b'\n') or data.startswith(b'STOP'):
        return data, False
    if kind == 'range':
        if not data.startswith(b'C '):
            return data, False
        parts = data[:-1].split(b'*')[0].split(b' ')
        if len(parts) != 6:
            return data, False
        parts[5] = b'2000'
        return recrc_line(b' '.join(parts)), True
    body, star, crc = data[:-1].partition(b'*')
    if len(body) < 2:
        return data, False
    if not star:
        return body + b'*%02X\n' % ((crc8(body) + 1) & 0xFF), True
    index = len(body) // 2
    value = body[index] ^ 0x01
    flipped = bytes([value]) if 0x21 <= value <= 0x7E and value != 0x2A else b'#'
    return body[:index] + flipped + body[index + 1:] + b'*' + crc + b'\n', True


# --- ports -----------------------------------------------------------------------------------------------------------
class MemoryPort:
    """In-process port for tests: host_write() queues host bytes, host_read() returns the MCU output so far."""

    kind = 'memory'

    def __init__(self, host_open=True):
        self.host_open = host_open
        self.to_mcu = bytearray()
        self.to_host = bytearray()
        self.created = True

    def poll_host_open(self):
        return self.host_open

    def read(self):
        data, self.to_mcu = bytes(self.to_mcu), bytearray()
        return data

    def write(self, data):
        if not self.host_open:
            return False
        self.to_host.extend(data)
        return True

    def host_write(self, data):
        self.to_mcu.extend(data)

    def host_read(self):
        data, self.to_host = bytes(self.to_host), bytearray()
        return data

    def unplug(self):
        self.created = False
        self.host_open = False

    def replug(self):
        self.created = True

    def describe(self):
        return dict(kind=self.kind, created=self.created, host_open=self.host_open)

    def close(self):
        self.created = False


class PtyPort:
    """Linux pty: the master stays with the emulator, the slave is reached through a link under /run/ubts-steer.

    The slave is set raw (no echo, no line editing, no NL translation) and then closed by the emulator, so the
    master reports a hangup until a host opens the slave. poll_host_open() is True while some process holds the
    slave open (no POLLHUP on the master)."""

    kind = 'pty'

    def __init__(self, link_path=DEFAULT_PTY_LINK, allowed_root=SIM_ROOT):
        if not (link_path.startswith(allowed_root.rstrip('/') + '/')):
            raise EmulatorError('pty link %s must be under %s' % (link_path, allowed_root))
        self.link_path = link_path
        self.master = None
        self.slave_name = None
        self.generation = 0
        self.write_errors = 0

    @property
    def created(self):
        return self.master is not None

    def create(self):
        import tty
        master, slave = os.openpty()
        try:
            tty.setraw(slave)
            name = os.ttyname(slave)
        finally:
            os.close(slave)
        os.set_blocking(master, False)
        directory = os.path.dirname(self.link_path)
        os.makedirs(directory, exist_ok=True)
        tmp = '%s.tmp%d' % (self.link_path, os.getpid())
        try:
            os.unlink(tmp)
        except FileNotFoundError:
            pass
        os.symlink(name, tmp)
        os.replace(tmp, self.link_path)
        self.master, self.slave_name = master, name
        self.generation += 1

    def poll_host_open(self):
        import select
        if self.master is None:
            return False
        poller = select.poll()
        poller.register(self.master, select.POLLIN | select.POLLHUP)
        for _, mask in poller.poll(0):
            if mask & select.POLLHUP:
                return False
        return True

    def read(self):
        if self.master is None:
            return b''
        chunks = []
        while True:
            try:
                data = os.read(self.master, 4096)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in (errno.EIO, errno.EAGAIN):    # EIO: no slave open (hangup)
                    break
                raise
            if not data:
                break
            chunks.append(data)
            if len(data) < 4096:
                break
        return b''.join(chunks)

    def write(self, data):
        if self.master is None:
            return False
        view = memoryview(data)
        while view:
            try:
                size = os.write(self.master, view)
            except (BlockingIOError, OSError):
                self.write_errors += 1
                return False
            view = view[size:]
        return True

    def unplug(self):
        """USB unplug: the master goes, so a host holding the slave sees EIO; the link is removed."""
        if self.master is not None:
            os.close(self.master)
            self.master = None
        try:
            os.unlink(self.link_path)
        except FileNotFoundError:
            pass

    def replug(self):
        self.create()

    def describe(self):
        return dict(kind=self.kind, link=self.link_path, slave=self.slave_name, created=self.created,
                    generation=self.generation, write_errors=self.write_errors)

    def close(self):
        self.unplug()


# --- event log ---------------------------------------------------------------------------------------------------------
class EventLog:
    """Rotating JSONL log (one previous file kept as .1). path None: entries are counted only."""

    def __init__(self, path=None, rotate_bytes=LOG_ROTATE_BYTES):
        self.path = None if path is None else Path(path)
        self.rotate_bytes = int(rotate_bytes)
        self.handle = None
        self.size = 0
        self.lines = 0
        self.lock = threading.Lock()
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.handle = open(self.path, 'w', encoding='utf-8', newline='\n')

    def write(self, entry):
        with self.lock:
            self.lines += 1
            if self.handle is None:
                return
            text = json.dumps(entry, allow_nan=False, separators=(',', ':')) + '\n'
            self.handle.write(text)
            self.size += len(text)
            if self.size >= self.rotate_bytes:
                self.handle.close()
                os.replace(self.path, str(self.path) + '.1')
                self.handle = open(self.path, 'w', encoding='utf-8', newline='\n')
                self.size = 0

    def flush(self):
        with self.lock:
            if self.handle is not None:
                self.handle.flush()

    def close(self):
        with self.lock:
            if self.handle is not None:
                self.handle.close()
                self.handle = None


# --- the core ------------------------------------------------------------------------------------------------------------
class EmulatorCore:
    """Firmware session + plant coupling + hooks, without ROS or a real port (both are injected).

    on_clock(clock_ns, speed_m_s) runs the MCU to clock_ns + lead_steps x step_ns and returns the outputs of the last
    FW_STEP (dict steer, drive, truth) or None when nothing ran. apply_hook(request) runs one hook request and
    returns the reply dict; call it from the thread that calls on_clock (the ROS side queues them)."""

    def __init__(self, profile=None, *, seed=0, cal_id=1, eeprom='calibrated', injections=(), record_overrides=None,
                 boot_delay_ticks=BOOT_DELAY_TICKS, port_open_reset=True, report_adc=False, executable=None,
                 step_ns=None, lead_steps=1, port=None, log=None, profile_path=None, boot_seed=None, chip_id=None,
                 report_servo=False):
        self.profile = load_profile(profile_path) if profile is None else profile
        self.profile_path = None if profile_path is None else str(profile_path)
        v = self.profile.value
        if v('steer_actuator_level') != 'L1':
            raise EmulatorError('the firmware mode runs the L1 actuator model; the profile has %s'
                                % v('steer_actuator_level'))
        self.seed = int(seed)
        self.cal_id = int(cal_id)
        self.eeprom_mode = eeprom
        self.injections = ba.validate_injections(list(injections))
        self.record_overrides = dict(record_overrides or {})
        self.boot_delay_ticks = int(boot_delay_ticks)
        self.port_open_reset = bool(port_open_reset)
        self.report_adc = bool(report_adc)
        self.report_servo = bool(report_servo)
        self.executable = executable
        self.step_ns = physics_step_ns(v('gz_physics_hz')) if step_ns is None else int(step_ns)
        self.lead_steps = int(lead_steps)
        if self.lead_steps not in (0, 1, 2):
            raise EmulatorError('lead_steps must be 0, 1 or 2')
        self.port = MemoryPort() if port is None else port
        self.log = EventLog(None) if log is None else log
        self.boot_seed = stable_u32(self.seed, PURPOSES['boot']) if boot_seed is None else int(boot_seed)
        self.chip_id = (stable_u32(self.seed, PURPOSES['chip']) & 0xFFFFFF) if chip_id is None else int(chip_id)
        self.geometry = geo.SteeringGeometry.from_profile(self.profile)
        self.steer_type = v('steer_type')
        self.line = drive_model.dc_motor_line(self.profile)
        self.driven = gz_model.DRIVEN_WHEELS[v('drive_wheels')]
        self.sag = float(v('pot_supply_sag_per_pwm'))
        self.baud = int(v('baud'))
        self.backend = None
        self.session = 0
        self.session_info = None
        self.t0_ns = None              # SIM time of the session's tick 0
        self.done_ns = 0               # SIM ns run since t0 (sum of the FW_STEP dt)
        self.last_clock_ns = None
        self.first_clock_ns = None
        self.clock_messages = 0
        self.max_clock_gap_ns = 0
        self.speed = 0.
        self.host_was_open = False
        self.assembler = LineAssembler()
        self.host_line_buf = bytearray()
        self.host_line_first = None
        self.windows = []              # [kind, start_tick, end_tick, params]
        self.pending_corruptions = collections.deque()
        self.banner_rewrites = collections.deque()
        self.adc_programs = []         # {'kind': 'pattern'|'ramp', ...}
        self.unplug_until_tick = None
        self.unplug_power = False
        self.recorder_request = None
        self.counters = collections.Counter()
        # (wall end, wall gap s, SIM s, monotonic end): the monotonic end is the time base of the attribution (the
        # VM's CLOCK_REALTIME is stepped by about +1.7 s every 30 s on this host, WF-4 g_gaps)
        self.clock_stalls = collections.deque(maxlen=CLOCK_STALLS_KEPT)
        # slow /clock intervals (mono start, mono end, lowest rate, wall end) and the open one (mono start, rate)
        self.clock_slow = collections.deque(maxlen=SLOW_KEPT)
        self.clock_slow_open = None
        self.last_status_line = None
        self.last_banner = None
        self.last_out = None
        self.outputs = None
        self.pin_state = {}
        self.steer_pwm_level = 0
        self.pwm_levels = (153, 204)
        self.closed = False
        self.started_wall = time.time()
        self.load = {}                 # the ROS side's load report (cpu, callback histogram, publish counts)

    # -- sessions
    def _eeprom_image(self, startup, cal_id, record_overrides, eeprom):
        if eeprom == 'blank':
            return None, None
        if eeprom != 'calibrated':
            raise EmulatorError('eeprom must be calibrated or blank')
        image, cal = fw_eeprom.calibrated_eeprom(startup, self.profile, cal_id=cal_id)
        if record_overrides:
            params = dict(cal['params'], **record_overrides)
            image = fw_eeprom.image_hex(fw_eeprom.encode_record(cal_id, cal['raw_l'], cal['raw_c'], cal['raw_r'],
                                                                cal['soft_l'], cal['soft_r'], params=params))
            cal = dict(cal, params=params)
        return image, cal

    def start_session(self, at_ns, reason, *, injections=None, eeprom=None, record_overrides=None, cal_id=None,
                      report_adc=None, report_servo=None):
        """Close the running native session (if any) and power a new MCU on with tick 0 at SIM time at_ns.
        report_adc / report_servo None keep the emulator's setting (--report-adc, --report-servo), True / False set
        it for this session. report_servo: the evaluation-only servo channels of lockstep v2 (decision R18-11)."""
        if injections is not None:
            self.injections = ba.validate_injections(list(injections))
        if eeprom is not None:
            self.eeprom_mode = eeprom
        if record_overrides is not None:
            self.record_overrides = dict(record_overrides)
        if cal_id is not None:
            self.cal_id = int(cal_id)
        if self.backend is not None:
            self._finish_recorder()
            self.backend.close()
            self.backend = None
        startup = v2.firmware_startup_config(self.profile, self.seed)
        image, cal = self._eeprom_image(startup, self.cal_id, self.record_overrides, self.eeprom_mode)
        session_seed = (self.boot_seed + self.session) & 0xFFFFFFFFFFFFFFFF
        adc = self.report_adc if report_adc is None else bool(report_adc)
        servo = self.report_servo if report_servo is None else bool(report_servo)
        firmware = dict(boot_seed=session_seed, chip_id=self.chip_id, eeprom_hex=image, adc_mode='model',
                        boot_delay_ticks=self.boot_delay_ticks, initial_delta_eq_rad=0., report_adc=adc,
                        report_servo=servo)
        self.backend = BoundedFirmwareBackend(self.profile, self.seed, firmware=firmware, injections=self.injections,
                                              executable=self.executable)
        self.session += 1
        self.t0_ns = int(at_ns)
        self.done_ns = 0
        self.assembler.reset()
        self.host_line_buf = bytearray()
        self.host_line_first = None
        self.windows = [w for w in self.windows if w[0] == 'never']
        self.adc_programs = []
        self.pin_state = {}
        self.steer_pwm_level = 0
        params = (cal or {}).get('params') or fw_eeprom.cfg_defaults()
        self.pwm_levels = (int(params['stp']), int(params['thp']))
        backend = self.backend
        self.session_info = dict(
            session=self.session, reason=reason, t0_sim_s=at_ns / 1e9, seed=self.seed, boot_seed=session_seed,
            chip_id='%06X' % self.chip_id, cal_id=None if cal is None else cal['cal_id'], eeprom=self.eeprom_mode,
            injections=self.injections, record_overrides=self.record_overrides, firmware=firmware,
            boot_delay_ticks=self.boot_delay_ticks, build_id=backend.build_id, firmware_build=backend.firmware_build,
            executable_sha256=backend.executable_sha256, profile_id=self.profile.profile_id,
            profile_sha256=self.profile.sha256, profile_path=self.profile_path,
            first_boot_id=backend.boot_id(0), calibration=None if cal is None else dict(
                scale_permille_per_deg=cal['scale_permille_per_deg'], soft_l=cal['soft_l'], soft_r=cal['soft_r'],
                raw_l=cal['raw_l'], raw_c=cal['raw_c'], raw_r=cal['raw_r'], mdir=cal.get('mdir', 1),
            params=dict(cal['params'])),
            startup=backend.config)
        self.counters['sessions'] += 1
        self._log('session', reason=reason, info={k: self.session_info[k] for k in (
            'session', 'reason', 't0_sim_s', 'boot_seed', 'chip_id', 'cal_id', 'eeprom', 'injections',
            'record_overrides', 'build_id', 'firmware_build', 'first_boot_id')})
        if self.recorder_request is not None:
            self._start_recorder()
        return self.session_info

    def _start_recorder(self):
        request, self.recorder_request = self.recorder_request, None
        info = dict(self.session_info)
        self.backend.recorder = InputRecorder(request['path'], int(round(request['seconds'] * 1000.)), info)
        self._log('record', path=str(request['path']), seconds=request['seconds'])

    def _finish_recorder(self):
        if self.backend is not None and self.backend.recorder is not None:
            self.backend.recorder.finish()
            self.backend.recorder = None

    # -- time
    def tick_sim_s(self, tick):
        return (self.t0_ns + tick * 1000000) / 1e9 if self.t0_ns is not None else None

    def now_tick(self):
        return 0 if self.backend is None else self.backend.ticks

    def sim_now_s(self):
        if self.t0_ns is None:
            return None
        return (self.t0_ns + self.done_ns) / 1e9

    def _log(self, kind, **fields):
        tick = self.now_tick()
        entry = dict(kind=kind, session=self.session, tick=tick, t=self.tick_sim_s(tick))
        entry.update(fields)
        self.log.write(entry)

    def on_clock(self, clock_ns, speed_m_s=None):
        """One /clock message (SIM ns). Returns the outputs of the last FW_STEP or None."""
        if self.closed:
            return None
        clock_ns = int(clock_ns)
        if speed_m_s is not None and math.isfinite(speed_m_s):
            self.speed = float(speed_m_s)
        self.clock_messages += 1
        if self.first_clock_ns is None:
            self.first_clock_ns = clock_ns
        if self.last_clock_ns is not None:
            if clock_ns < self.last_clock_ns:
                self.counters['clock_rewinds'] += 1
                self._log('clock_rewind', previous_s=self.last_clock_ns / 1e9, now_s=clock_ns / 1e9)
                self.start_session(clock_ns, 'clock_rewind')
            else:
                gap = clock_ns - self.last_clock_ns
                self.max_clock_gap_ns = max(self.max_clock_gap_ns, gap)
                for limit_ms in CLOCK_GAP_BINS_MS:
                    if gap > limit_ms * 1000000:
                        self.counters['clock_gaps_over_%dms' % limit_ms] += 1
        self.last_clock_ns = clock_ns
        if self.backend is None:
            self.start_session(clock_ns, 'first_clock')
        target = clock_ns + self.lead_steps * self.step_ns - self.t0_ns
        ran = False
        while self.done_ns < target:
            limit = PATTERN_CHUNK_NS if self.adc_programs else MAX_CHUNK_NS
            chunk = min(target - self.done_ns, limit)
            self._run_chunk(chunk)
            ran = True
        if not ran:
            return None
        self.outputs = self._outputs()
        return self.outputs

    def _supply_scale(self):
        b = self.last_out.base if self.last_out is not None else None
        duty = 0. if b is None else .5 * (abs(b.drive_duty_left) + abs(b.drive_duty_right))
        return max(1e-6, 1. - self.sag * duty)

    def _window(self, kind, tick, direction=None):
        for w_kind, start, end, params in self.windows:
            if w_kind == kind and start <= tick < end and (direction is None or params.get('direction') in (
                    direction, 'both')):
                return params
        return None

    def _run_chunk(self, chunk_ns):
        backend = self.backend
        first = backend.ticks
        count = backend.clock.ticks_for(chunk_ns / 1e9)
        self._port_step(first)
        if count > 0:
            self._schedule_adc(first, count)
        speed = self.speed
        backend.step(ba.SteerStepInput(dt_s=chunk_ns / 1e9, mode='firmware', vehicle_speed_m_s=speed,
                                       vehicle_stopped=abs(speed) < STANDSTILL_SPEED_M_S,
                                       supply_scale=self._supply_scale()))
        self.done_ns += chunk_ns
        out = backend.last
        self.last_out = out
        self.counters['fw_steps'] += 1
        self.counters['ticks'] += out.ticks
        self._pins(out)
        self._mcu_lines(out)
        if self.unplug_until_tick is not None and backend.ticks >= self.unplug_until_tick:
            self._replug()
        self.windows = [w for w in self.windows if w[2] > backend.ticks]
        if backend.recorder is not None and backend.recorder.done:
            self._log('record_done', path=str(backend.recorder.path), frames=len(backend.recorder.frames))
            backend.recorder = None

    # -- host side
    def _port_step(self, tick):
        port = self.port
        host_open = port.created and port.poll_host_open()
        if host_open != self.host_was_open:
            self.host_was_open = host_open
            self.counters['host_opens' if host_open else 'host_closes'] += 1
            self._log('port', host_open=host_open, port=port.describe())
            if host_open and self.port_open_reset and self.backend is not None:
                self.backend.event('port_open_reset', tick=tick)
                self._log('event', name='port_open_reset', at_tick=tick, source='host_open')
        data = port.read() if port.created else b''
        if not data:
            return
        mono = time.monotonic()
        self.counters['h2m_bytes_read'] += len(data)
        if self._window('link', tick, 'to_mcu') is not None:
            self.counters['h2m_bytes_lost'] += len(data)
            self._log('h2m_lost', bytes=len(data), mono=round(mono, 6))
            return
        for line in self._host_lines(data):
            self._send_host_line(line, tick, mono=mono)

    def _host_lines(self, data):
        """Whole host lines (with LF) and, at the end, the partial rest (sent as it is: the MCU parser joins it)."""
        out = []
        self.host_line_buf.extend(data)
        while True:
            index = self.host_line_buf.find(b'\n')
            if index < 0:
                break
            out.append(bytes(self.host_line_buf[:index + 1]))
            del self.host_line_buf[:index + 1]
        if self.host_line_buf and len(self.host_line_buf) > 256:
            out.append(bytes(self.host_line_buf))
            self.host_line_buf = bytearray()
        return out

    def _send_host_line(self, line, tick, note='host', mono=None):
        if self.pending_corruptions and note == 'host':
            kind = self.pending_corruptions[0]
            changed, applied = corrupt_host_line(line, kind)
            if applied:
                self.pending_corruptions.popleft()
                line, note = changed, 'host:%s_error' % kind
                self.counters['corrupted_lines'] += 1
        arrivals = self.backend.send(line, tick=tick, pace='uart')
        self.counters['h2m_lines'] += 1
        self._log('h2m', text=line_text(line), first=arrivals[0] if arrivals else None,
                  last=arrivals[-1] if arrivals else None, note=note,
                  mono=None if mono is None else round(mono, 6))

    # -- MCU side
    def _pins(self, out):
        for change in out.changes:
            name = change.channel
            if name in PIN_CHANNELS:
                previous = self.pin_state.get(name)
                if previous != change.value:
                    self.pin_state[name] = change.value
                    entry = dict(channel=name, value=change.value)
                    if name == 'fault_bits':
                        entry['faults'] = fault_bit_names(change.value)
                    self.log.write(dict(kind='pin', session=self.session, tick=change.tick,
                                        t=self.tick_sim_s(change.tick), **entry))
            elif name == 'steer_pwm':
                level = pwm_level(change.value, self.pwm_levels)
                if level != self.steer_pwm_level:
                    self.steer_pwm_level = level
                    self.log.write(dict(kind='pin', session=self.session, tick=change.tick,
                                        t=self.tick_sim_s(change.tick), channel='steer_pwm_level', value=level,
                                        pwm=change.value))

    def _mcu_lines(self, out):
        for data, first, done, garbage in self.assembler.feed(out.tx):
            text = line_text(data)
            kind = text.split(' ', 1)[0]
            if text.startswith('UBTS_STEER_V3 hello '):
                self.counters['banners'] += 1
                if self.banner_rewrites:
                    fields = self.banner_rewrites[0]['fields']
                    data = rewrite_banner(data, fields)
                    self.banner_rewrites[0]['count'] -= 1
                    if self.banner_rewrites[0]['count'] <= 0:
                        self.banner_rewrites.popleft()
                    text = line_text(data)
                    self.counters['banners_rewritten'] += 1
                self.last_banner = text
            elif kind == 'S':
                self.last_status_line = text
                self.counters['status_lines'] += 1
            note = 'garbage+line' if garbage else ''
            delivered = done + 1
            if self._window('link', delivered, 'to_host') is not None:
                note = 'lost'
                self.counters['m2h_lines_lost'] += 1
            elif kind == 'S' and self._window('status_loss', delivered) is not None:
                note = 'status_lost'
                self.counters['status_lines_lost'] += 1
            elif not (self.port.created and self.host_was_open):
                note = 'port_closed'
                self.counters['m2h_lines_port_closed'] += 1
            elif not self.port.write(data):
                note = 'write_failed'
                self.counters['m2h_write_failed'] += 1
            else:
                self.counters['m2h_lines'] += 1
            self.log.write(dict(kind='m2h', session=self.session, tick=first, done=done,
                                t=self.tick_sim_s(first), text=text, note=note, mono=round(time.monotonic(), 6)))

    # -- outputs to Gazebo
    def _outputs(self):
        out = self.last_out
        b = out.base
        if self.steer_type == geo.AXLE_PIVOT:
            steer = {'axle': b.delta_eq_rad}
        else:
            steer = {'left': b.delta_left_rad, 'right': b.delta_right_rad}
        omega0 = self.line.omega0_rad_s
        if b.drive_en:
            drive = {wheel: (b.drive_duty_left if wheel.startswith('left') else b.drive_duty_right) * omega0
                     for wheel in self.driven}
        else:      # EN off (coast: MCU reset, boot, power off) is not representable: the DC line brakes to 0
            drive = {wheel: 0. for wheel in self.driven}
            self.counters['coast_steps_as_brake'] += 1
        truth = dict(schema=SCHEMA_TRUTH, sim_time_s=self.sim_now_s(), session=self.session, tick=out.first_tick +
                     out.ticks, delta_left_rad=b.delta_left_rad, delta_right_rad=b.delta_right_rad,
                     delta_eq_rad=b.delta_eq_rad, motor_side_angle_rad=b.motor_side_angle_rad,
                     applied_steer_pwm=b.applied_steer_pwm, steer_en=b.steer_en, pot_raw=b.pot_raw,
                     drive_duty_left=b.drive_duty_left, drive_duty_right=b.drive_duty_right, drive_en=b.drive_en,
                     mcu_state=out.mcu_state, fault_bits=out.base.fault_bits, model_flags=b.model_flags,
                     boot_count=out.boot_count, vehicle_speed_m_s=self.speed, lead_steps=self.lead_steps,
                     result_kind=RESULT_KIND)
        return dict(steer=steer, drive=drive, truth=truth)

    # -- ADC programs (hooks)
    def _schedule_adc(self, first, count):
        keep = []
        for program in self.adc_programs:
            for tick in range(first, first + count):
                if tick < program['start']:
                    continue
                if program['kind'] == 'pattern':
                    if tick >= program['end']:
                        break
                    values = program['values']
                    value = values[(tick - program['start']) % len(values)]
                else:
                    value = program['from'] + program['step'] * (tick - program['start'] + 1)
                    if (program['step'] > 0 and value >= program['to']) or (program['step'] < 0
                                                                            and value <= program['to']):
                        value = program['to']
                        program['done'] = True
                    value = int(round(value))
                if value != program.get('last'):
                    self.backend.event('adc_set', tick=tick, arg=int(value))
                    program['last'] = value
                if program.get('done'):
                    break
            finished = (program['kind'] == 'pattern' and first + count >= program['end']) or program.get('done')
            if finished:
                if program['kind'] == 'pattern':
                    self.backend.event('adc_model', tick=max(first, program['end']), arg=0)
                    self._log('adc_program_end', program=program['kind'], end_tick=program['end'])
                else:
                    self._log('adc_program_end', program='ramp', value=program['to'])
            else:
                keep.append(program)
        self.adc_programs = keep

    # -- hooks
    def apply_hook(self, request):
        op = request.get('op')
        handler = getattr(self, '_hook_' + str(op), None) if isinstance(op, str) else None
        if handler is None:
            return dict(ok=False, error='unknown_hook')
        if self.backend is None and op not in ('status', 'mark', 'record', 'restart'):
            return dict(ok=False, error='no_session_yet')
        try:
            reply = handler(request)
        except (ValueError, TypeError, KeyError, EmulatorError) as exc:
            return dict(ok=False, error='%s: %s' % (type(exc).__name__, exc))
        self.counters['hooks'] += 1
        self._log('hook', request=request, reply_ok=reply.get('ok', True))
        reply.setdefault('ok', True)
        reply.update(session=self.session, tick=self.now_tick(), sim_time_s=self.sim_now_s())
        return reply

    def _at_tick(self, request):
        tick = request.get('at_tick')
        now = self.now_tick()
        if tick is None:
            return now
        tick = int(tick)
        if tick < now:
            raise ValueError('at_tick %d is in the past (next tick %d)' % (tick, now))
        return tick

    def _hook_status(self, request):
        return dict(status=self.status())

    def _hook_mark(self, request):
        self._log('mark', label=str(request.get('label', '')))
        return {}

    def _hook_event(self, request):
        kind = request['kind']
        if kind not in v2.EVENT_KINDS:
            raise ValueError('event kind %r' % (kind,))
        tick = self._at_tick(request)
        arg = int(request.get('arg', 0))
        if kind == 'adc_model':
            self.adc_programs = []
        self.backend.event(kind, tick=tick, arg=arg)
        return dict(at_tick=tick, at_sim_s=self.tick_sim_s(tick))

    def _hook_adc_pattern(self, request):
        values = [int(v) for v in request['values']]
        if not values or any(not 0 <= v <= 1023 for v in values):
            raise ValueError('values must be raw counts 0..1023')
        start = self._at_tick(request)
        end = start + int(request['duration_ms'])
        self.adc_programs.append(dict(kind='pattern', values=values, start=start, end=end))
        return dict(start_tick=start, end_tick=end)

    def _hook_adc_ramp(self, request):
        start = self._at_tick(request)
        base = request.get('from')
        if base is None:
            base = self.last_out.base.pot_raw if self.last_out is not None else 512
        to = int(request['to'])
        step = float(request['counts_per_tick'])
        if step <= 0.:
            raise ValueError('counts_per_tick must be positive')
        step = step if to >= base else -step
        program = {'kind': 'ramp', 'start': start, 'from': float(base), 'to': to, 'step': step}
        self.adc_programs.append(program)
        return dict(start_tick=start, from_raw=base, to_raw=to, note='the ADC stays scripted at the end value until '
                                                                     'an event adc_model')

    def _hook_link(self, request):
        direction = request.get('direction', 'both')
        if direction not in ('to_mcu', 'to_host', 'both'):
            raise ValueError('direction must be to_mcu, to_host or both')
        start = self._at_tick(request)
        end = start + int(request['duration_ms'])
        self.windows.append(['link', start, end, dict(direction=direction)])
        return dict(start_tick=start, end_tick=end, direction=direction)

    def _hook_status_loss(self, request):
        start = self._at_tick(request)
        end = start + int(request['duration_ms'])
        self.windows.append(['status_loss', start, end, {}])
        return dict(start_tick=start, end_tick=end)

    def _hook_corrupt(self, request):
        kind = request['kind']
        if kind not in ('crc', 'range'):
            raise ValueError('corrupt kind must be crc or range')
        count = int(request.get('count', 1))
        self.pending_corruptions.extend([kind] * count)
        return dict(pending=len(self.pending_corruptions))

    def _hook_half_frame(self, request):
        data = str(request.get('text', 'C 1 3')).encode('ascii')
        if b'\n' in data:
            raise ValueError('a half frame has no LF')
        tick = self._at_tick(request)
        arrivals = self.backend.send(data, tick=tick, pace='uart')
        self._log('h2m', text=data.decode('ascii'), first=arrivals[0], last=arrivals[-1], note='half_frame')
        return dict(at_tick=tick, bytes=len(data))

    def _hook_inject_line(self, request):
        text = str(request['text'])
        data = host_line(text, crc=bool(request.get('crc', True)))
        tick = self._at_tick(request)
        arrivals = self.backend.send(data, tick=tick, pace='uart')
        self._log('h2m', text=line_text(data), first=arrivals[0], last=arrivals[-1], note='foreign_write')
        return dict(at_tick=tick, last_arrival_tick=arrivals[-1])

    def _hook_banner_rewrite(self, request):
        fields = {str(k): str(v) for k, v in dict(request['fields']).items()}
        if set(fields) - {'sim', 'hw', 'dev', 'cal'}:
            raise ValueError('banner_rewrite fields: sim, hw, dev, cal')
        self.banner_rewrites.append(dict(fields=fields, count=int(request.get('count', 1))))
        return dict(pending=len(self.banner_rewrites))

    def _hook_restart(self, request):
        at_ns = self.t0_ns + self.done_ns if self.t0_ns is not None else (self.last_clock_ns or 0)
        injections = request.get('injections')
        report_adc = request.get('report_adc')
        report_servo = request.get('report_servo')
        for name, value in (('report_adc', report_adc), ('report_servo', report_servo)):
            if value is not None and not isinstance(value, bool):
                raise ValueError('%s must be true, false or null' % name)
        info = self.start_session(at_ns, str(request.get('reason', 'hook_restart')), injections=injections,
                                  eeprom=request.get('eeprom'), record_overrides=request.get('record_overrides'),
                                  cal_id=request.get('cal_id'), report_adc=report_adc, report_servo=report_servo)
        return dict(session_info=dict({k: info[k] for k in ('session', 't0_sim_s', 'first_boot_id', 'cal_id', 'eeprom',
                                                            'injections', 'record_overrides', 'build_id')},
                                      report_adc=info['firmware']['report_adc'],
                                      report_servo=info['firmware']['report_servo']))

    def _hook_unplug(self, request):
        duration = int(request['duration_ms'])
        power = request.get('mcu_power', 'keep')
        if power not in ('keep', 'cycle'):
            raise ValueError('mcu_power must be keep or cycle')
        tick = self.now_tick()
        self.port.unplug()
        self.host_was_open = False
        self.counters['unplugs'] += 1
        self._log('port', host_open=False, unplugged=True, port=self.port.describe())
        if power == 'cycle':
            self.backend.event('power_cycle', tick=tick, arg=max(1, duration))
        self.unplug_until_tick = tick + duration
        self.unplug_power = power == 'cycle'
        return dict(unplugged_at_tick=tick, replug_tick=self.unplug_until_tick, mcu_power=power)

    def _replug(self):
        self.unplug_until_tick = None
        self.port.replug()
        self.counters['replugs'] += 1
        self._log('port', replugged=True, port=self.port.describe())

    def _hook_record(self, request):
        seconds = float(request['seconds'])
        path = str(request['path'])
        if not 0. < seconds <= 600.:
            raise ValueError('seconds must be in (0, 600]')
        # A replay starts from READY, so a recording always starts with a new session (power on).
        self.recorder_request = dict(path=path, seconds=seconds)
        if self.backend is None:
            return dict(pending=True)
        return self._hook_restart(dict(reason=str(request.get('reason', 'record')), injections=request.get('injections'),
                                       eeprom=request.get('eeprom'), record_overrides=request.get('record_overrides'),
                                       cal_id=request.get('cal_id'), report_adc=request.get('report_adc'),
                                       report_servo=request.get('report_servo', True)))

    def _hook_record_finish(self, request):
        """Write the running record now (A19(G): the rule ticks are computed from it right after a case)."""
        backend = self.backend
        recorder = None if backend is None else backend.recorder
        if recorder is None:
            return dict(finished=False)
        recorder.finish()
        backend.recorder = None
        self._log('record_done', path=str(recorder.path), frames=len(recorder.frames))
        return dict(finished=True, path=str(recorder.path), frames=len(recorder.frames), ticks=recorder.counted)

    def _hook_kill_bridge(self, request):
        """A17 mandatory fault test 'bridge forced kill' (design 7.2.1): SIGKILL to the bridge wrapper child of the
        steer supervisor (same container, its state file names the pid; /proc/<pid>/cmdline must run the bridge
        module). Nothing else changes here: the MCU keeps its last C until the C TTL, then its link watchdog; the
        supervisor restarts the bridge and its port open resets the MCU."""
        pid = bridge_pid(request.get('supervisor_state') or DEFAULT_SUPERVISOR_STATE)
        mono, wall = time.monotonic(), time.time()
        os.kill(pid, signal.SIGKILL)
        self.counters['bridge_kills'] += 1
        self._log('bridge_kill', pid=pid, mono=round(mono, 6), wall=wall)
        return dict(pid=pid, killed_at_tick=self.now_tick(), mono=mono, wall=wall)

    # -- status
    def status(self):
        out = self.last_out
        b = None if out is None else out.base
        truth = None
        if b is not None:
            truth = dict(delta_eq_deg=math.degrees(b.delta_eq_rad), delta_left_deg=math.degrees(b.delta_left_rad),
                         delta_right_deg=math.degrees(b.delta_right_rad), drive_duty_left=b.drive_duty_left,
                         drive_duty_right=b.drive_duty_right, drive_en=b.drive_en, pot_raw=b.pot_raw,
                         applied_steer_pwm=b.applied_steer_pwm)
        return dict(
            schema=SCHEMA_STATUS, result_kind=RESULT_KIND, field_validated=False, session=self.session,
            session_info=None if self.session_info is None else {k: self.session_info[k] for k in (
                'reason', 't0_sim_s', 'boot_seed', 'chip_id', 'cal_id', 'eeprom', 'injections', 'record_overrides',
                'build_id', 'firmware_build', 'executable_sha256', 'profile_id', 'profile_sha256', 'first_boot_id',
                'calibration')},
            tick=self.now_tick(), sim_time_s=self.sim_now_s(),
            first_clock_s=None if self.first_clock_ns is None else self.first_clock_ns / 1e9,
            last_clock_s=None if self.last_clock_ns is None else self.last_clock_ns / 1e9,
            clock_messages=self.clock_messages, max_clock_gap_ms=self.max_clock_gap_ns / 1e6,
            clock_stalls=[dict(wall_end=w, wall_gap_ms=round(g * 1000., 1), sim_s=t, mono_end=round(m, 6))
                          for w, g, t, m in self.clock_stalls],
            clock_slow=[dict(mono_start=round(a, 4), mono_end=round(b, 4), min_rate=round(r, 4), wall_end=w)
                        for a, b, r, w in self.clock_slow] + ([] if self.clock_slow_open is None else [dict(
                            mono_start=round(self.clock_slow_open[0], 4), mono_end=None,
                            min_rate=round(self.clock_slow_open[1], 4), wall_end=None)]),
            lead_steps=self.lead_steps, step_ns=self.step_ns, port=self.port.describe(), host_open=self.host_was_open,
            port_open_reset=self.port_open_reset, report_servo=self.report_servo, load=dict(self.load),
            mcu=None if out is None else dict(state=out.mcu_state, boot_count=out.boot_count, millis=out.mcu_millis,
                                              fault_bits='%04X' % b.fault_bits, faults=fault_bit_names(b.fault_bits),
                                              rx_lost_total=out.rx_lost_total, run_flags=out.run_flags),
            last_banner=self.last_banner, last_status_line=self.last_status_line, truth=truth,
            vehicle_speed_m_s=self.speed, counters=dict(sorted(self.counters.items())),
            hooks=dict(windows=[list(w) for w in self.windows], pending_corruptions=len(self.pending_corruptions),
                       banner_rewrites=list(self.banner_rewrites), adc_programs=len(self.adc_programs),
                       unplugged_until_tick=self.unplug_until_tick,
                       recording=bool(self.backend is not None and self.backend.recorder is not None)),
            log_lines=self.log.lines)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self._finish_recorder()
        if self.backend is not None:
            self.backend.close()
            self.backend = None
        self.port.close()
        self.log.close()


def bridge_pid(state_path=DEFAULT_SUPERVISOR_STATE):
    """The pid of the running bridge wrapper child of the steer supervisor (state file), checked in /proc."""
    try:
        state = json.loads(Path(state_path).read_bytes().decode('utf-8'))
    except (OSError, ValueError) as exc:
        raise EmulatorError('no supervisor state %s: %s' % (state_path, exc))
    child = next((c for c in state.get('children') or () if c.get('name') == 'bridge'), None)
    if child is None or not child.get('pid') or not child.get('running'):
        raise EmulatorError('the supervisor state has no running bridge child')
    pid = int(child['pid'])
    try:
        cmdline = Path('/proc/%d/cmdline' % pid).read_bytes().replace(b'\0', b' ').decode('utf-8', 'replace')
    except OSError as exc:
        raise EmulatorError('bridge pid %d is not readable: %s' % (pid, exc))
    if BRIDGE_MODULE not in cmdline:
        raise EmulatorError('pid %d does not run %s (%s)' % (pid, BRIDGE_MODULE, cmdline[:120]))
    return pid


# --- hook server -----------------------------------------------------------------------------------------------------------
class _HookHandler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(5.)
        try:
            raw = self.rfile.readline(MAX_HOOK_LINE + 1)
            if len(raw) > MAX_HOOK_LINE:
                raise ValueError('request_too_large')
            request = json.loads(raw)
            if not isinstance(request, dict):
                raise ValueError('object_required')
            reply = self.server.submit(request)
        except (ValueError, OSError) as exc:
            reply = dict(ok=False, error=str(exc))
        try:
            self.wfile.write(json.dumps(reply, allow_nan=False, default=str).encode('utf-8') + b'\n')
        except OSError:
            pass


class HookServer:
    """Unix socket for the fault hooks; requests are applied by the clock thread (queue) within timeout_s."""

    def __init__(self, path, timeout_s=5.):
        self.path = path
        self.timeout_s = timeout_s
        self.requests = queue.Queue(maxsize=64)
        self.server = None

    def start(self):
        base = getattr(socketserver, 'ThreadingUnixStreamServer', None)
        if base is None:
            raise EmulatorError('Unix socket server not available on this platform')
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass

        class Server(base):
            daemon_threads = True
        self.server = Server(self.path, _HookHandler)
        os.chmod(self.path, 0o600)
        self.server.submit = self.submit
        threading.Thread(target=self.server.serve_forever, name='ubts-steer-emulator-hooks', daemon=True).start()

    def submit(self, request):
        done = threading.Event()
        box = {}
        try:
            self.requests.put_nowait((request, box, done))
        except queue.Full:
            return dict(ok=False, error='hook_queue_full')
        if not done.wait(self.timeout_s):
            box['cancelled'] = True
            return dict(ok=False, error='hook_not_applied_within_%.1f_s (no /clock?)' % self.timeout_s)
        return box['reply']

    def drain(self, core):
        while True:
            try:
                request, box, done = self.requests.get_nowait()
            except queue.Empty:
                return
            if box.get('cancelled'):
                continue
            box['reply'] = core.apply_hook(request)
            done.set()

    def close(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass


# --- status file ----------------------------------------------------------------------------------------------------------
def write_json_atomic(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp%d' % os.getpid())
    tmp.write_bytes((json.dumps(payload, allow_nan=False, default=str) + '\n').encode('utf-8'))
    os.replace(tmp, path)


def read_bridge_status(path, max_age_s=3.):
    try:
        data = json.loads(Path(path).read_bytes().decode('utf-8'))
    except (OSError, ValueError):
        return None
    age = time.time() - float(data.get('updated_at') or 0.)
    keep = ('ok', 'state', 'connected', 'armed', 'moving', 'reason', 'last_error', 'transport', 'transport_kind',
            'serial_port', 'serial_open', 'firmware', 'boot_id', 'device_id', 'hw_profile', 'simulated', 'profile',
            'cal_id', 'requested', 'steering')
    out = {key: data.get(key) for key in keep}
    host = data.get('steering_host') or {}
    out['steering_host'] = {key: host.get(key) for key in ('arm_state', 'stop_state', 'mcu_state', 'fl', 'stale',
                                                           'target_mrad', 'latches', 'last_c')}
    out['age_s'] = age
    out['fresh'] = age <= max_age_s
    return out


# --- ROS side ----------------------------------------------------------------------------------------------------------------
class RosEmulator:
    """rclpy wiring: /clock and /ground_truth/odom in, /sim/act/* and /sim/truth/steering out. All stepping happens in
    the /clock callback (single-threaded executor), hooks are drained there too."""

    def __init__(self, core, steer_topics, drive_topics, hooks=None, node_name=NODE_NAME, port=None,
                 pty_after_clock_s=PTY_AFTER_CLOCK_S, truth_every=1, act_refresh_steps=ACT_REFRESH_STEPS):
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Float64, String
        self.rclpy, self.Float64, self.String = rclpy, Float64, String
        self.core, self.hooks = core, hooks
        # No parameter services, type description service or /rosout publisher (G-SIM stage 7): nothing uses them and
        # rclpy adds every service of the node to the wait set of each /clock callback (120 Hz). Topics, QoS and
        # callbacks are unchanged.
        self.node = Node(node_name, parameter_overrides=[Parameter('use_sim_time', value=False),
                                                         Parameter('start_type_description_service', value=False)],
                         start_parameter_services=False, enable_rosout=False)
        act_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.steer_pubs = {key: self.node.create_publisher(Float64, topic, act_qos) for key, topic in steer_topics.items()}
        self.drive_pubs = {wheel: self.node.create_publisher(Float64, topic, act_qos)
                           for wheel, topic in drive_topics.items()}
        self.truth_pub = self.node.create_publisher(String, TRUTH_TOPIC, 10)
        clock_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20, reliability=ReliabilityPolicy.RELIABLE)
        odom_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.speed = None
        self.node.create_subscription(Odometry, TRUTH_ODOM_TOPIC, self.on_odom, odom_qos)
        self.node.create_subscription(Clock, CLOCK_TOPIC, self.on_clock, clock_qos)
        self.errors = 0
        self.last_error = None
        self.last_receipt = None
        self.recent_clock = collections.deque(maxlen=512)        # (monotonic, SIM s) of the last /clock receipts
        # deferred pty (module docstring): the link appears after steady /clock
        self.port = port
        self.pty_after_clock_s = float(pty_after_clock_s)
        self.pty_pending = port is not None and not port.created
        self.steady_since_s = None
        self.first_clock_mono = None
        # publish on change (+ refresh), truth every N steps, load report
        self.truth_every = max(1, int(truth_every))
        self.act_refresh_steps = max(1, int(act_refresh_steps))
        self.last_act = {}
        self.steps = 0
        self.cb_hist = collections.Counter()
        self.cb_max_ms = 0.
        self.cpu0 = (time.monotonic(), self.process_cpu())
        self.cpu_recent = None
        self.odom_messages = 0

    @staticmethod
    def process_cpu():
        times = os.times()
        return times.user + times.system

    def on_odom(self, msg):
        self.odom_messages += 1
        value = float(msg.twist.twist.linear.x)
        if math.isfinite(value):
            self.speed = value

    def maybe_create_pty(self, now, sim_s, gap):
        """Create the pty link once /clock has been steady for pty_after_clock_s of SIM time (no wall gap above
        PTY_STEADY_GAP_S), or PTY_FALLBACK_WALL_S after the first /clock."""
        if self.first_clock_mono is None:
            self.first_clock_mono = now
        if gap is None or gap > PTY_STEADY_GAP_S or self.steady_since_s is None:
            self.steady_since_s = sim_s
        steady = sim_s - self.steady_since_s >= self.pty_after_clock_s
        fallback = now - self.first_clock_mono >= PTY_FALLBACK_WALL_S
        if not (steady or fallback):
            return
        self.port.create()
        self.pty_pending = False
        self.core.counters['pty_created_fallback' if not steady else 'pty_created_steady'] += 1
        self.core._log('pty_created', mono=round(now, 6), sim_s=sim_s, steady_s=round(sim_s - self.steady_since_s, 4),
                       fallback=not steady, wall_since_first_clock_s=round(now - self.first_clock_mono, 3),
                       port=self.port.describe())
        print('gz_esp_emulator: pty %s -> %s after %.2f s SIM of steady /clock (%.2f s wall after the first /clock%s)'
              % (self.port.link_path, self.port.slave_name, sim_s - self.steady_since_s, now - self.first_clock_mono,
                 ', fallback' if not steady else ''), flush=True)

    def load_report(self):
        now, cpu = time.monotonic(), self.process_cpu()
        t0, c0 = self.cpu0
        if now - t0 >= 5.:
            self.cpu_recent = round(100. * (cpu - c0) / (now - t0), 1)
            self.cpu0 = (now, cpu)
        hist = {('<=%dms' % b): self.cb_hist.get(b, 0) for b in CALLBACK_BINS_MS}
        hist['>%dms' % CALLBACK_BINS_MS[-1]] = self.cb_hist.get(None, 0)
        return dict(process_cpu_s=round(cpu, 3), cpu_pct_last_5s=self.cpu_recent, clock_callback_ms_hist=hist,
                    clock_callback_max_ms=round(self.cb_max_ms, 3), odom_messages=self.odom_messages,
                    act_published=self.core.counters['act_published'], act_skipped=self.core.counters['act_skipped'],
                    truth_published=self.core.counters['truth_published'], truth_every=self.truth_every,
                    act_refresh_steps=self.act_refresh_steps, pty_pending=self.pty_pending)

    def track_rate(self, now, sim_s):
        """SIM time received per monotonic second over the last SLOW_WINDOW_S. Windows below SLOW_RTF are kept as
        slow intervals (core.clock_slow, monotonic times): evidence that a host steering_status_stale of the wall clock
        mode came from the SIM side (Gazebo slower than real time, or a stall of this process: A10 invalid)."""
        recent = self.recent_clock
        recent.append((now, sim_s))
        while len(recent) > 2 and now - recent[1][0] >= SLOW_WINDOW_S:
            recent.popleft()
        span = now - recent[0][0]
        if span < SLOW_WINDOW_S:
            return
        rate = (sim_s - recent[0][1]) / span
        core = self.core
        if rate < SLOW_RTF:
            if core.clock_slow_open is None:
                core.clock_slow_open = (recent[0][0], rate)
            elif rate < core.clock_slow_open[1]:
                core.clock_slow_open = (core.clock_slow_open[0], rate)
        elif core.clock_slow_open is not None:
            core.clock_slow.append((core.clock_slow_open[0], now, core.clock_slow_open[1], time.time()))
            core.clock_slow_open = None

    def on_clock(self, msg):
        now = time.monotonic()
        started = time.perf_counter()
        gap = None
        if self.last_receipt is not None:
            gap = now - self.last_receipt
            for limit_ms in CLOCK_GAP_BINS_MS:
                if gap > limit_ms / 1000.:
                    self.core.counters['clock_wall_gaps_over_%dms' % limit_ms] += 1
            self.core.counters['clock_wall_gap_max_us'] = max(self.core.counters['clock_wall_gap_max_us'],
                                                               int(gap * 1e6))
            if gap > CLOCK_STALL_MS / 1000.:      # the wall time of a stall, for the A10 validity of host latches
                self.core.clock_stalls.append((time.time(), gap, int(msg.clock.sec) + int(msg.clock.nanosec) * 1e-9,
                                               now))
        self.last_receipt = now
        sim_s = int(msg.clock.sec) + int(msg.clock.nanosec) * 1e-9
        self.track_rate(now, sim_s)
        try:
            if self.hooks is not None:
                self.hooks.drain(self.core)
            result = self.core.on_clock(int(msg.clock.sec) * 1000000000 + int(msg.clock.nanosec), self.speed)
            if result is not None:
                self.publish(result)
            if self.pty_pending:
                self.maybe_create_pty(now, sim_s, gap)
        except Exception as exc:     # never leave the callback: count, log and stop driving
            self.errors += 1
            self.last_error = '%s: %s' % (type(exc).__name__, exc)
            print('gz_esp_emulator: step error: %s' % self.last_error, file=sys.stderr, flush=True)
            self.publish_zero_drive()
            if self.errors > 50:
                raise
        spent = (time.perf_counter() - started) * 1000.
        self.cb_max_ms = max(self.cb_max_ms, spent)
        self.cb_hist[next((b for b in CALLBACK_BINS_MS if spent <= b), None)] += 1

    def publish(self, result):
        """/sim/act/*: a value is published when it differs from its last published value, and every value every
        act_refresh_steps steps (the Gazebo joint controllers keep their last target, so the joints see the same
        targets); /sim/truth/steering every truth_every steps."""
        self.steps += 1
        refresh = self.steps % self.act_refresh_steps == 0
        counters = self.core.counters
        for pubs, values in ((self.steer_pubs, result['steer']), (self.drive_pubs, result['drive'])):
            for key, value in values.items():
                pub = pubs.get(key)
                if pub is None:
                    continue
                value = float(value)
                if refresh or self.last_act.get(id(pub)) != value:
                    pub.publish(self.Float64(data=value))
                    self.last_act[id(pub)] = value
                    counters['act_published'] += 1
                else:
                    counters['act_skipped'] += 1
        if self.steps % self.truth_every == 0:
            self.truth_pub.publish(self.String(data=json.dumps(result['truth'], allow_nan=False)))
            counters['truth_published'] += 1

    def publish_zero_drive(self):
        for pub in self.drive_pubs.values():
            pub.publish(self.Float64(data=0.))
            self.last_act[id(pub)] = 0.

    def spin(self, stop_event):
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        try:
            while not stop_event.is_set() and self.rclpy.ok():
                executor.spin_once(timeout_sec=.05)
        finally:
            executor.remove_node(self.node)

    def close(self):
        try:
            self.publish_zero_drive()
        except Exception:
            pass
        self.node.destroy_node()


def scene_topics(scene_dir):
    """(steer topics, drive topics, manifest) of a gz_assemble scene folder."""
    manifest = json.loads((Path(scene_dir) / 'scene_manifest.json').read_bytes().decode('utf-8'))
    topics = manifest['model']['topics']
    if manifest['model'].get('drive_mode') != 'force_dc':
        raise EmulatorError('the emulator drives force_dc scenes only (scene drive mode %s)'
                            % manifest['model'].get('drive_mode'))
    return dict(topics['steer']), dict(topics['drive']), manifest


def build_parser():
    p = argparse.ArgumentParser(prog='gz_esp_emulator', description=__doc__.splitlines()[0])
    p.add_argument('--scene-dir', required=True, help='gz_assemble scene folder (scene_manifest.json)')
    p.add_argument('--profile', default=None, help='vehicle profile YAML (default: the shipped default profile)')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--cal-id', type=int, default=1)
    p.add_argument('--eeprom', choices=('calibrated', 'blank'), default='calibrated')
    p.add_argument('--inject', action='append', default=[],
                   help='v1 model injection as JSON {"kind", "at_s", "params"} (at_s from the session start)')
    p.add_argument('--pty-link', default=os.environ.get('UBTS_STEER_PTY_LINK', DEFAULT_PTY_LINK))
    p.add_argument('--hook-socket', default=os.environ.get('UBTS_STEER_HOOK_SOCKET', DEFAULT_HOOK_SOCKET),
                   help="fault hook socket ('' disables the hooks)")
    p.add_argument('--status-file', default=os.environ.get('UBTS_STEER_STATUS_FILE', DEFAULT_STATUS_FILE))
    p.add_argument('--bridge-status-file', default=os.environ.get('ROVER_BRIDGE_STATUS_FILE',
                                                                  DEFAULT_BRIDGE_STATUS_FILE))
    p.add_argument('--log-file', default=os.environ.get('UBTS_STEER_EMULATOR_LOG', DEFAULT_LOG_FILE),
                   help="JSONL event log ('' disables)")
    p.add_argument('--lead-steps', type=int, default=1, choices=(0, 1, 2))
    p.add_argument('--no-port-open-reset', dest='port_open_reset', action='store_false', default=True)
    p.add_argument('--report-adc', action='store_true', help='adc_read channel (A3(G) recordings)')
    p.add_argument('--report-servo', action='store_true',
                   help='evaluation-only servo channels in every session (records switch them on by themselves)')
    p.add_argument('--pty-after-clock-s', type=float,
                   default=float(os.environ.get('UBTS_STEER_PTY_AFTER_CLOCK_S', PTY_AFTER_CLOCK_S)),
                   help='SIM seconds of steady /clock before the pty link appears (0: at start, as before)')
    p.add_argument('--truth-every', type=int, default=int(os.environ.get('UBTS_STEER_TRUTH_EVERY', '1')),
                   help='publish /sim/truth/steering every N physics steps')
    p.add_argument('--act-refresh-steps', type=int, default=ACT_REFRESH_STEPS,
                   help='unchanged /sim/act/* values are published again every N steps (1: every step, as before)')
    p.add_argument('--boot-delay-ticks', type=int, default=BOOT_DELAY_TICKS)
    p.add_argument('--executable', default=None, help='native lockstep program (default: build or the image build)')
    p.add_argument('--max-runtime-s', type=float, default=None)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    steer_topics, drive_topics, manifest = scene_topics(args.scene_dir)
    profile = load_profile(args.profile)
    injections = [json.loads(item) for item in args.inject]
    log = EventLog(args.log_file or None)
    port = PtyPort(args.pty_link)
    try:                                   # a link left by an earlier emulator would let the bridge start too early
        os.unlink(args.pty_link)
    except FileNotFoundError:
        pass
    if args.pty_after_clock_s <= 0.:
        port.create()
    core = EmulatorCore(profile, seed=args.seed, cal_id=args.cal_id, eeprom=args.eeprom, injections=injections,
                        boot_delay_ticks=args.boot_delay_ticks, port_open_reset=args.port_open_reset,
                        report_adc=args.report_adc, executable=args.executable, lead_steps=args.lead_steps,
                        port=port, log=log, profile_path=args.profile, report_servo=args.report_servo)
    hooks = HookServer(args.hook_socket) if args.hook_socket else None
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    ros = RosEmulator(core, steer_topics, drive_topics, hooks, port=port, pty_after_clock_s=args.pty_after_clock_s,
                      truth_every=args.truth_every, act_refresh_steps=args.act_refresh_steps)
    if hooks is not None:
        hooks.start()
    static = dict(scene=manifest.get('scene'), scene_sdf_sha256=manifest.get('sdf_sha256'),
                  steer_topics=steer_topics, drive_topics=drive_topics, truth_topic=TRUTH_TOPIC,
                  hook_socket=args.hook_socket or None, log_file=args.log_file or None,
                  source_commit=os.environ.get('UBTS_STEER_SOURCE_COMMIT'),
                  source_status_sha256=os.environ.get('UBTS_STEER_SOURCE_STATUS_SHA256'),
                  pid=os.getpid(), pty_after_clock_s=args.pty_after_clock_s, truth_every=args.truth_every,
                  act_refresh_steps=args.act_refresh_steps)

    def status_loop():
        while not stop.wait(STATUS_PERIOD_S):
            try:
                core.load = ros.load_report()
                payload = dict(schema='ubts.steering_status.v1', updated_at=time.time(), emulator=core.status(),
                               emulator_static=static, ros_errors=ros.errors, ros_last_error=ros.last_error,
                               bridge=read_bridge_status(args.bridge_status_file))
                write_json_atomic(args.status_file, payload)
                log.flush()
            except Exception as exc:  # the status file is a report; never stop the emulator for it
                print('gz_esp_emulator: status error: %s' % exc, file=sys.stderr, flush=True)
    threading.Thread(target=status_loop, name='ubts-steer-emulator-status', daemon=True).start()
    print('gz_esp_emulator: pty %s -> %s, scene %s, hooks %s' % (
        args.pty_link, port.slave_name if port.created else 'after %.2f s of steady /clock' % args.pty_after_clock_s,
        manifest.get('scene'), args.hook_socket or 'off'), flush=True)
    if args.max_runtime_s:
        threading.Timer(args.max_runtime_s, stop.set).start()
    code = 0
    try:
        ros.spin(stop)
    except Exception as exc:
        print('gz_esp_emulator: fatal: %s: %s' % (type(exc).__name__, exc), file=sys.stderr, flush=True)
        code = 1
    finally:
        try:
            ros.close()
        finally:
            if hooks is not None:
                hooks.close()
            core.close()
            rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
