"""Python link to the native steering actuator and potentiometer process (decision R1).

NativeSteeringBackend implements backend_api.SteeringBackend over the lockstep protocol v1
(docs/steering/LOCKSTEP_PROTOCOL_KO.md): the startup line from native_startup_config(),
READY, then strictly one request frame -> one response frame. The binary is built on demand
by steering.native.build (compiler from UBTS_STEER_CXX or g++ on PATH, output folder from
UBTS_STEER_BUILD_DIR) unless `executable` is given.

Failure handling: any write error, EOF, truncated frame or bad response closes the backend
and raises NativeBackendError with the child's exit code, its stderr line and the phase.
A child that does not read a request or answer it within response_timeout_s (default 30 s
per write and per response; None waits for ever) is killed by a watchdog thread and the
call raises NativeBackendError with timed_out=True, so a hung child cannot hang a matrix run.
The child is always stopped: close(), garbage collection and interpreter exit close its
stdin (the child then exits 0 at the frame boundary) and kill it if it does not exit.

Results built on the placeholder profile are placeholder-based SIM results, not field
validation; result_label() carries the profile identity, mode, seed and the build id.

FirmwareSteeringBackend (subclass) runs the same process in lockstep v2 firmware mode (section 11): the
firmware sources of firmware/ubts_steer_v3, the actuator and the potentiometer model in one subprocess.
It adds the firmware API: fw_step() (one FW_STEP frame), send() / event() (host bytes and events at
absolute ticks), run() / run_until() (steps of N ms), tx_bytes / lines() / read_new_bytes() (MCU bytes with
their ticks), channel() / channel_at() (pin and state channels) and last (drive outputs and truth).
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass
import subprocess
import threading
import time
import weakref

from . import backend_api as ba
from .native import build as native_build
from .native import lockstep_v2 as v2

DEFAULT_EXIT_TIMEOUT_S = 5.0
DEFAULT_RESPONSE_TIMEOUT_S = 30.0


class NativeBackendError(RuntimeError):
    """The native steering process failed.

    phase        'startup', 'reset', 'step' or 'raw'
    exit_code    the child's exit code (protocol section 8), a negative signal number, or None
    exit_name    protocol name of the exit code ('bad_value', ...) or None
    stderr_line  the child's diagnostic line 'steer_native: <name>: <detail>' ('' if none)
    killed       True if the parent killed the child (response it could not accept, or no response)
    timed_out    True if a request write or a response took longer than response_timeout_s
    """

    def __init__(self, message, *, phase, exit_code=None, stderr_line='', killed=False, timed_out=False):
        super().__init__(message)
        self.phase = phase
        self.exit_code = exit_code
        self.exit_name = None if killed or exit_code is None else ba.EXIT_CODES.get(exit_code)
        self.stderr_line = stderr_line
        self.killed = killed
        self.timed_out = timed_out


def _close_streams(proc):
    for stream in (proc.stdin, proc.stdout, proc.stderr):
        if stream is None:
            continue
        try:
            stream.close()
        except (OSError, ValueError):
            pass


def _stop_process(proc, timeout_s):
    """Close stdin (EOF at a frame boundary -> exit 0), wait, kill if needed; returns the exit code."""
    try:
        if proc.stdin is not None and not proc.stdin.closed:
            proc.stdin.close()
    except (OSError, ValueError):
        pass
    try:
        code = proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        proc.kill()
        code = proc.wait()
    _close_streams(proc)
    return code


class _ReadWatchdog:
    """Kills the child when one request write or one response read lasts longer than timeout_s.

    One daemon thread polls every min(0.25 s, timeout / 4); arming and disarming a write or a read are plain
    attribute writes in the calling thread, so a step costs no lock and no thread switch.
    """

    def __init__(self, proc, timeout_s):
        self.proc = proc
        self.timeout_s = float(timeout_s)
        self.started = None       # time.monotonic() when the pending write or read began; None in between
        self.fired = False
        self._stop = threading.Event()
        self._period = min(0.25, self.timeout_s / 4.)
        self._thread = threading.Thread(target=self._run, name='ubts-steer-native-watchdog', daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.wait(self._period):
            started = self.started
            if started is not None and time.monotonic() - started > self.timeout_s:
                self.fired = True
                try:
                    self.proc.kill()
                except OSError:
                    pass
                return

    def stop(self):
        self._stop.set()


def _finalize_process(proc, timeout_s, watchdog=None):
    """weakref finalizer (garbage collection, interpreter exit). Never raises."""
    if watchdog is not None:
        watchdog.stop()
    try:
        _stop_process(proc, timeout_s)
    except Exception:  # pragma: no cover - last resort at interpreter shutdown
        try:
            proc.kill()
        except Exception:
            pass


def _last_line(text):
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    return lines[-1] if lines else ''


class NativeSteeringBackend(ba.SteeringBackend):
    """SteeringBackend over the native lockstep process.

    NativeSteeringBackend(profile=None, seed=0, *, mode='direct_pwm', mcu_tick_hz=None, injections=(),
                          executable=None, compiler=None, build_dir=None, exit_timeout_s=5.0,
                          response_timeout_s=30.0)

    mode            'direct_pwm' or 'test_servo' ('firmware' is refused by the v1 native, exit 2)
    mcu_tick_hz     None -> profile sim_mcu_tick_hz (1000); must be a multiple of servo_hz
    injections      fault injections [{'kind', 'at_s', 'params'}] (backend_api.INJECTION_PARAMS)
    executable      use this binary instead of building one (a list is taken as a full command line,
                    for test doubles; its SHA-256 is then not recorded)
    compiler        C++ compiler for the build (default UBTS_STEER_CXX or g++ on PATH)
    build_dir       build output folder (default UBTS_STEER_BUILD_DIR or the git-ignored _build/)
    exit_timeout_s  how long close() waits for a clean exit before killing the child
    response_timeout_s  longest time for one request write or one response frame (READY, RESET, STEP) before
                    the child is killed
                    and NativeBackendError(timed_out=True) is raised; None waits for ever (debugging)
    With a profile the backend configures (and starts) at once.
    """

    def __init__(self, profile=None, seed=0, *, mode='direct_pwm', mcu_tick_hz=None, injections=(),
                 executable=None, compiler=None, build_dir=None, exit_timeout_s=DEFAULT_EXIT_TIMEOUT_S,
                 response_timeout_s=DEFAULT_RESPONSE_TIMEOUT_S):
        if mode not in ba.MODES:
            raise ValueError('backend_mode_unknown:%r' % (mode,))
        self.mode = mode
        self._tick_option = mcu_tick_hz
        self._injections = ba.validate_injections(injections)
        self._executable = executable
        self._compiler = compiler
        self._build_dir = build_dir
        self._exit_timeout = float(exit_timeout_s)
        if response_timeout_s is not None and not float(response_timeout_s) > 0.:
            raise ValueError('response_timeout_s_must_be_positive_or_None')
        self._response_timeout = None if response_timeout_s is None else float(response_timeout_s)
        self._proc = None
        self._finalizer = None
        self._watchdog = None
        self._seq = 0
        self.config = None          # startup configuration dict actually sent
        self.build = None           # native_build.BuildResult when built here
        self.build_id = None        # READY build id
        self.executable = None
        self.executable_sha256 = None
        self.exit_code = None       # exit code of the last child after close() or a failure
        self.profile_label = None
        if profile is not None:
            self.configure(profile, seed)

    # -- lifecycle
    @property
    def running(self):
        return self._proc is not None

    @property
    def pid(self):
        return None if self._proc is None else self._proc.pid

    def configure(self, profile, seed):
        """Start a native process for this profile and seed (a running one is closed first)."""
        self.close()
        config = self._startup_config(profile, seed)
        line = ba.startup_line(config)
        command = self._resolve_command()
        proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self._proc = proc
        self._watchdog = None if self._response_timeout is None else _ReadWatchdog(proc, self._response_timeout)
        self._finalizer = weakref.finalize(self, _finalize_process, proc, self._exit_timeout, self._watchdog)
        self._seq = 0
        self._send(line, 'startup')
        frame = self._read('startup')
        try:
            _, build_id = self._decode_ready(frame, expected_tick_hz=config['mcu_tick_hz'])
        except ba.ProtocolError as exc:
            raise self._failure('startup', exc, child_exits=False) from None
        self.config = config
        self.build_id = build_id
        self.profile_label = profile.result_label()
        return self

    # hooks of the protocol version (FirmwareSteeringBackend overrides them)
    def _startup_config(self, profile, seed):
        return ba.native_startup_config(profile, seed, mode=self.mode, mcu_tick_hz=self._tick_option,
                                        injections=self._injections)

    @staticmethod
    def _decode_ready(frame, expected_tick_hz):
        return ba.decode_ready(frame, expected_tick_hz=expected_tick_hz)

    @staticmethod
    def _read_frame(stream):
        return ba.read_frame(stream)

    def reset(self, delta_eq_rad=0.):
        """New episode at the wheel-side equivalent angle (inside the equivalent mechanical limits)."""
        self._require_running()
        geometry = self.config['geometry']
        if not -geometry['eq_mech_limit_right_rad'] <= float(delta_eq_rad) <= geometry['eq_mech_limit_left_rad']:
            raise ValueError('reset_delta_outside_equivalent_mechanical_limits:%r' % (delta_eq_rad,))
        seq = self._seq + 1
        frame = ba.encode_reset_request(seq, delta_eq_rad)
        self._seq = seq
        self._send(frame, 'reset')
        response = self._read('reset')
        try:
            return ba.decode_reset_response(response, seq)
        except ba.ProtocolError as exc:
            raise self._failure('reset', exc, child_exits=False) from None

    def step(self, inp):
        self._require_running()
        if inp.mode != self.mode:
            raise ValueError('steer_step_mode_differs_from_backend:%s!=%s' % (inp.mode, self.mode))
        seq = self._seq + 1
        frame = ba.encode_step_request(seq, inp)
        self._seq = seq
        self._send(frame, 'step')
        response = self._read('step')
        try:
            return ba.decode_step_response(response, seq)
        except ba.ProtocolError as exc:
            raise self._failure('step', exc, child_exits=False) from None

    def close(self):
        """Stop the child (EOF, wait, kill if needed). Safe to call twice."""
        proc, self._proc = self._proc, None
        if self._finalizer is not None:
            self._finalizer.detach()
            self._finalizer = None
        self._stop_watchdog()
        if proc is not None:
            self.exit_code = _stop_process(proc, self._exit_timeout)

    def result_label(self):
        """Identity block for results: profile label + backend, mode, seed, tick rate, build id, binary SHA."""
        if self.config is None:
            raise RuntimeError('native_backend_not_configured')
        label = dict(self.profile_label)
        label.update(backend='native_lockstep_v1', mode=self.mode, seed=self.config['seed'],
                     mcu_tick_hz=self.config['mcu_tick_hz'], build_id=self.build_id,
                     executable_sha256=self.executable_sha256,
                     injections=[dict(item) for item in self.config['injections']])
        return label

    # -- test and diagnostics hook
    def _exchange_raw(self, data, phase='raw'):
        """Write raw bytes and read one frame back (protocol tests). Errors raise NativeBackendError."""
        self._require_running()
        self._send(bytes(data), phase)
        return self._read(phase)

    # -- internals
    def _resolve_command(self):
        """Command line of the child: [binary], or a given argument list (test doubles; no SHA then)."""
        if isinstance(self._executable, (list, tuple)):
            self.executable, self.executable_sha256 = [str(part) for part in self._executable], None
            return list(self.executable)
        if self._executable is not None:
            executable = str(self._executable)
        else:
            self.build = native_build.ensure_built(compiler=self._compiler, build_dir=self._build_dir)
            executable = self.build.path
        if executable != self.executable:
            self.executable_sha256 = native_build.file_sha256(executable)   # cached per size and mtime
            self.executable = executable
        return [executable]

    def _require_running(self):
        if self._proc is None:
            raise RuntimeError('native_backend_not_running')

    def _send(self, data, phase):
        watchdog = self._watchdog
        if watchdog is not None:
            watchdog.started = time.monotonic()    # a child that stops reading would block a write on a full pipe
        try:
            self._proc.stdin.write(data)
            self._proc.stdin.flush()
        except (OSError, ValueError) as exc:
            if self._timed_out():
                raise self._timeout_failure(phase) from None
            raise self._failure(phase, exc, child_exits=True) from None
        finally:
            if watchdog is not None:
                watchdog.started = None
        if self._timed_out():
            raise self._timeout_failure(phase) from None

    def _stop_watchdog(self):
        watchdog, self._watchdog = self._watchdog, None
        if watchdog is not None:
            watchdog.stop()

    def _timed_out(self):
        return self._watchdog is not None and self._watchdog.fired

    def _read(self, phase):
        watchdog = self._watchdog
        if watchdog is not None:
            watchdog.started = time.monotonic()
        try:
            frame = self._read_frame(self._proc.stdout)
        except ba.ProtocolError as exc:
            if self._timed_out():
                raise self._timeout_failure(phase) from None
            raise self._failure(phase, exc, child_exits=exc.exit_code == ba.EXIT_TRUNCATED) from None
        except (OSError, ValueError) as exc:
            if self._timed_out():
                raise self._timeout_failure(phase) from None
            raise self._failure(phase, exc, child_exits=True) from None
        finally:
            if watchdog is not None:
                watchdog.started = None
        if self._timed_out():
            raise self._timeout_failure(phase) from None
        if not frame:
            raise self._failure(phase, EOFError('native process closed its stdout'), child_exits=True) from None
        return frame

    def _timeout_failure(self, phase):
        return self._failure(phase, TimeoutError('no response frame within %.3g s (response_timeout_s); the child '
                                                 'was killed' % self._response_timeout), child_exits=False,
                             timed_out=True)

    def _failure(self, phase, cause, child_exits, timed_out=False):
        """Stop the child, collect its exit code and stderr line, and build the exception."""
        proc, self._proc = self._proc, None
        if self._finalizer is not None:
            self._finalizer.detach()
            self._finalizer = None
        self._stop_watchdog()
        code, stderr_text = None, ''
        if proc is not None:
            if child_exits:
                try:
                    code = proc.wait(timeout=self._exit_timeout)
                except subprocess.TimeoutExpired:
                    child_exits = False
            if not child_exits:
                proc.kill()
                code = proc.wait()
            try:
                stderr_text = proc.stderr.read().decode('ascii', 'replace')
            except (OSError, ValueError):
                stderr_text = ''
            _close_streams(proc)
        self.exit_code = code
        line = _last_line(stderr_text)
        if timed_out:
            meaning = 'killed by the parent: no response'
        elif not child_exits:
            meaning = 'killed by the parent'
        elif code is not None and code < 0:
            meaning = 'signal %d' % -code
        else:
            meaning = ba.EXIT_CODES.get(code, 'unknown exit code')
        message = 'native steering process failed during %s: exit code %s (%s); stderr: %s; cause: %s: %s' % (
            phase, code, meaning, line or '(none)', type(cause).__name__, cause)
        return NativeBackendError(message, phase=phase, exit_code=code, stderr_line=line, killed=not child_exits,
                                  timed_out=timed_out)


# --- firmware mode (lockstep v2, section 11) -------------------------------------------------------------------------
LINE_KINDS = ('BANNER', 'CAL', 'S', 'N', 'F', 'W', 'A', 'P', 'OTHER')
STATUS_FIELDS = ('ms', 'n', 'st', 'q', 'L', 'R', 'en', 'raw', 'pos', 'tgt', 'cmd', 'sp', 'fl', 'wd', 'tl')
CHANNEL_RESET_VALUES = dict({name: 0 for name in v2.CHANNELS}, mcu_state=v2.MCU_NOT_RUNNING)


def crc8(data):
    """CRC-8 of the serial lines (ESP protocol 3): poly 0x07, init 0, no reflection."""
    crc = 0
    for byte in bytes(data):
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def host_line(body, crc=True):
    """A host line `body*HH\\n` (crc=True) or `body\\n` as bytes (ESP protocol 2.5; uppercase CRC)."""
    data = body.encode('ascii') if isinstance(body, str) else bytes(body)
    return data + (b'*%02X' % crc8(data) if crc else b'') + b'\n'


def line_kind(text):
    if text.startswith('UBTS_STEER_V3 hello '):
        return 'BANNER'
    if text.startswith('UBTS_STEER_V3 cal '):
        return 'CAL'
    first = text.split(' ', 1)[0]
    return first if first in ('S', 'N', 'F', 'W', 'A', 'P') else 'OTHER'


@dataclass(frozen=True)
class McuLine:
    """One MCU line: text without the LF, tick of its first byte (queued) and of its LF (done)."""

    text: str
    queued_tick: int
    done_tick: int

    @property
    def kind(self):
        return line_kind(self.text)

    @property
    def crc_ok(self):
        t = self.text
        return len(t) >= 3 and t[-3] == '*' and t[-2:] == '%02X' % crc8(t[:-3].encode('ascii', 'replace'))

    @property
    def body(self):
        return self.text[:-3] if len(self.text) >= 3 and self.text[-3] == '*' else self.text

    def fields(self):
        return self.body.split(' ')

    def status(self):
        """Fields of a status line as ints (fl from hex), or None for other lines."""
        parts = self.fields()
        if parts[0] != 'S' or len(parts) != 16:
            return None
        values = {}
        for name, token in zip(STATUS_FIELDS, parts[1:]):
            values[name] = int(token, 16) if name == 'fl' else int(token)
        return values


def assemble_lines(tx_bytes, include_garbage=False):
    """(complete McuLines, bytes of the unfinished last line) from FwTxByte entries in transmission order."""
    lines, current, first_tick = [], bytearray(), None
    for entry in tx_bytes:
        if entry.boot_garbage and not include_garbage:
            continue
        if first_tick is None:
            first_tick = entry.queued_tick
        if entry.byte == 0x0A:
            lines.append(McuLine(current.decode('ascii', 'replace'), first_tick, entry.done_tick))
            current, first_tick = bytearray(), None
        else:
            current.append(entry.byte)
    return lines, bytes(current)


class FirmwareSteeringBackend(NativeSteeringBackend):
    """NativeSteeringBackend in lockstep v2 firmware mode (LOCKSTEP_PROTOCOL_KO.md section 11).

    FirmwareSteeringBackend(profile=None, seed=0, *, firmware=None, injections=(), startup_overrides=None,
                            executable=None, compiler=None, build_dir=None, exit_timeout_s=5.0,
                            response_timeout_s=30.0)

    firmware           keys of the startup 'firmware' object (lockstep_v2.FIRMWARE_KEYS; defaults
                       lockstep_v2.FIRMWARE_DEFAULTS); fw_eeprom.calibrated_eeprom() builds eeprom_hex
    injections         v1 fault injections of the actuator and potentiometer model (at_s from RESET)
    startup_overrides  JSON merge patch (RFC 7386) applied to the whole startup object last

    Low level: reset(delta_eq_rad=None) and fw_step(dt_s, events=, rx=, vehicle_speed_m_s=, vehicle_stopped=,
    supply_scale=) map 1:1 to RESET and FW_STEP frames (event / rx tick offsets inside the step).
    Session level (absolute ticks since RESET; tick n has millis() = n - reset tick + millis_start):
      send(data, tick=None, pace='instant'|'uart', delay_ticks=0)  host bytes (instant: all in RX before the
                                                     tick's loop; uart: section 11.7 arrival rule)
      event(kind, tick=None, arg=0)                  MCU reset, power cycle, port open reset, motor power,
                                                     stall, ADC source (lockstep_v2.EVENT_KINDS)
      run(ticks, step_ticks=10, ...) / run_until(tick, ...)  FW_STEP frames of step_ticks ms with the
                                                     scheduled bytes and events; plant values per call
      tx_bytes, lines(), read_new_bytes()            MCU bytes with queued and done ticks, assembled lines
      channel(name), channel_at(name, tick)          change channels (truth-like: evaluation only)
      last                                           the last FwStepOutput (drive duties, truth, pot_raw)
    step(SteerStepInput) keeps the SteeringBackend contract: serial_in arrives at the first tick of the step,
    serial_out is every byte finished in the step.
    """

    def __init__(self, profile=None, seed=0, *, firmware=None, injections=(), startup_overrides=None,
                 executable=None, compiler=None, build_dir=None, exit_timeout_s=DEFAULT_EXIT_TIMEOUT_S,
                 response_timeout_s=DEFAULT_RESPONSE_TIMEOUT_S):
        self._firmware_options = dict(firmware or {})
        v2.firmware_block(**self._firmware_options)   # refuse bad keys before a process starts
        self._startup_overrides = startup_overrides
        self.clock = v2.TickClock()
        self._reset_session_state()
        super().__init__(None, seed, mode='firmware', mcu_tick_hz=v2.MCU_TICK_HZ, injections=injections,
                         executable=executable, compiler=compiler, build_dir=build_dir, exit_timeout_s=exit_timeout_s,
                         response_timeout_s=response_timeout_s)
        if profile is not None:
            self.configure(profile, seed)

    # -- protocol hooks
    def _startup_config(self, profile, seed):
        return v2.firmware_startup_config(profile, seed, firmware=self._firmware_options, injections=self._injections,
                                          startup_overrides=self._startup_overrides)

    @staticmethod
    def _decode_ready(frame, expected_tick_hz):
        return v2.decode_ready_v2(frame, expected_tick_hz=expected_tick_hz)

    @staticmethod
    def _read_frame(stream):
        return v2.read_frame_v2(stream)

    def _resolve_command(self):
        command = super()._resolve_command()
        if self._executable is None and self.build is not None and self.build.firmware_build_id is None:
            raise native_build.NativeBuildError(
                'firmware mode needs the firmware sources (%s, or set %s); %s was built without them' % (
                    native_build.FIRMWARE_REL, native_build.ENV_FIRMWARE_DIR, self.build.path))
        return command

    # -- lifecycle
    def configure(self, profile, seed):
        super().configure(profile, seed)
        self.clock.reset()
        self._reset_session_state()
        return self

    @property
    def firmware_build(self):
        """The firmware build id (8 hex) of the READY build id, or None."""
        for part in (self.build_id or '').split(' '):
            if part.startswith('fw='):
                return part[3:]
        return None

    def result_label(self):
        label = super().result_label()
        fw = self.config['firmware']
        label.update(backend='native_lockstep_v2_firmware', firmware_build=self.firmware_build,
                     boot_seed=fw['boot_seed'], chip_id=fw['chip_id'], eeprom_blank=fw['eeprom_hex'] is None)
        return label

    def reset(self, delta_eq_rad=None):
        """RESET: model at the angle (default firmware.initial_delta_eq_rad), EEPROM image, RNG, MCU power-on reset."""
        self._require_running()
        if delta_eq_rad is None:
            delta_eq_rad = self.config['firmware']['initial_delta_eq_rad']
        geometry = self.config['geometry']
        if not -geometry['eq_mech_limit_right_rad'] <= float(delta_eq_rad) <= geometry['eq_mech_limit_left_rad']:
            raise ValueError('reset_delta_outside_equivalent_mechanical_limits:%r' % (delta_eq_rad,))
        seq = self._seq + 1
        frame = v2.encode_reset_request_v2(seq, delta_eq_rad)
        self._seq = seq
        self._send(frame, 'reset')
        response = self._read('reset')
        try:
            out = v2.decode_reset_response_v2(response, seq)
        except ba.ProtocolError as exc:
            raise self._failure('reset', exc, child_exits=False) from None
        self.clock.reset()
        self._reset_session_state()
        return out

    def fw_step(self, dt_s, *, events=(), rx=(), vehicle_speed_m_s=0., vehicle_stopped=True, supply_scale=1.):
        """One FW_STEP frame. events: (tick_offset, kind, arg); rx: (tick_offset, byte). Returns FwStepOutput."""
        self._require_running()
        first = self.clock.ticks_total
        expected = self.clock.ticks_for(dt_s)
        seq = self._seq + 1
        frame = v2.encode_fw_step_request(seq, dt_s, vehicle_speed_m_s, supply_scale, vehicle_stopped, events, rx)
        self._seq = seq
        self._send(frame, 'step')
        response = self._read('step')
        try:
            out = v2.decode_fw_step_response(response, seq, first)
        except ba.ProtocolError as exc:
            raise self._failure('step', exc, child_exits=False) from None
        if out.ticks != expected:
            raise self._failure('step', RuntimeError('native ran %d ticks, the tick rule gives %d' % (
                out.ticks, expected)), child_exits=False)
        self.clock.advance(dt_s)
        self._record(out)
        return out

    def step(self, inp):
        """SteeringBackend.step in firmware mode: serial_in at the first tick, serial_out = bytes done in the step."""
        if inp.mode != 'firmware':
            raise ValueError('steer_step_mode_differs_from_backend:%s!=firmware' % (inp.mode,))
        inp.validate()
        if inp.dt_s > v2.MAX_FW_DT_S:
            raise ValueError('firmware_step_dt_above_%g' % v2.MAX_FW_DT_S)
        ticks = self.clock.ticks_for(inp.dt_s)
        if inp.serial_in and ticks <= 0:
            raise ValueError('firmware_step_without_ticks_cannot_carry_serial_in')
        events, rx = self._take_pending(self.ticks, ticks)
        rx = [(0, byte) for byte in bytes(inp.serial_in)] + rx
        rx.sort(key=lambda item: item[0])
        out = self.fw_step(inp.dt_s, events=events, rx=rx, vehicle_speed_m_s=inp.vehicle_speed_m_s,
                           vehicle_stopped=inp.vehicle_stopped, supply_scale=inp.supply_scale)
        return _replace_serial(out.base, bytes(entry.byte for entry in out.tx))

    # -- session level
    @property
    def ticks(self):
        """Ticks run since RESET (the next tick to run)."""
        return self.clock.ticks_total

    def boot_id(self, boot_index=0):
        """SIM boot_id of the boot_index-th setup() since RESET (ESP protocol 10.1), as 8 uppercase hex."""
        return '%08X' % v2.sim_boot_ids(self.config['firmware']['boot_seed'], boot_index + 1)[boot_index]

    def send(self, data, tick=None, pace='instant', delay_ticks=0):
        """Schedule host -> MCU bytes; returns their arrival ticks."""
        data = data.encode('ascii') if isinstance(data, str) else bytes(data)
        tick = self.ticks if tick is None else int(tick)
        if tick < self.ticks:
            raise ValueError('send_tick_in_the_past:%d<%d' % (tick, self.ticks))
        if pace == 'instant':
            arrivals = [tick] * len(data)
        elif pace == 'uart':
            arrivals, self._host_line_end_u = v2.host_arrival_ticks(len(data), tick, self._host_line_end_u, delay_ticks,
                                                                    self.config['mcu']['baud'])
        else:
            raise ValueError('send_pace_unknown:%r' % (pace,))
        for arrival, byte in zip(arrivals, data):
            self._pending_rx.append((arrival, self._order, byte))
            self._order += 1
        self._pending_rx.sort()
        return arrivals

    def event(self, kind, tick=None, arg=0):
        """Schedule a lockstep event (lockstep_v2.EVENT_KINDS) at an absolute tick (default: the next tick)."""
        code = v2.event_code(kind)
        tick = self.ticks if tick is None else int(tick)
        if tick < self.ticks:
            raise ValueError('event_tick_in_the_past:%d<%d' % (tick, self.ticks))
        self._pending_events.append((tick, self._order, code, int(arg)))
        self._order += 1
        self._pending_events.sort()

    def run(self, ticks, step_ticks=10, vehicle_speed_m_s=0., vehicle_stopped=True, supply_scale=1.):
        """Run `ticks` ticks in FW_STEP frames of at most step_ticks (<= 100) ticks; returns the FwStepOutputs."""
        if not 1 <= step_ticks <= 100:
            raise ValueError('step_ticks_out_of_range:%r' % (step_ticks,))
        end = self.ticks + int(ticks)
        outs = []
        while self.ticks < end:
            dt = min(step_ticks, end - self.ticks) / 1000.
            first, count = self.ticks, self.clock.ticks_for(dt)
            events, rx = self._take_pending(first, count)
            outs.append(self.fw_step(dt, events=events, rx=rx, vehicle_speed_m_s=vehicle_speed_m_s,
                                     vehicle_stopped=vehicle_stopped, supply_scale=supply_scale))
        return outs

    def run_until(self, tick, **kwargs):
        """Run until `tick` ticks have run (the next tick to run is `tick`)."""
        return self.run(int(tick) - self.ticks, **kwargs) if tick > self.ticks else []

    def lines(self, include_garbage=False):
        """Complete MCU lines so far (boot garbage bytes left out unless include_garbage)."""
        if include_garbage:
            return assemble_lines(self.tx_bytes, include_garbage=True)[0]
        if self._lines_upto < len(self.tx_bytes):
            for entry in self.tx_bytes[self._lines_upto:]:
                if entry.boot_garbage:
                    continue
                if self._line_first is None:
                    self._line_first = entry.queued_tick
                if entry.byte == 0x0A:
                    self._lines.append(McuLine(self._line_buf.decode('ascii', 'replace'), self._line_first,
                                               entry.done_tick))
                    self._line_buf, self._line_first = bytearray(), None
                else:
                    self._line_buf.append(entry.byte)
            self._lines_upto = len(self.tx_bytes)
        return list(self._lines)

    def read_new_bytes(self):
        """Every MCU byte finished since the last call (boot garbage included, as a host would receive it)."""
        data = bytes(entry.byte for entry in self.tx_bytes[self._read_pos:])
        self._read_pos = len(self.tx_bytes)
        return data

    def channel(self, name):
        """[(tick, value)] changes of a channel since RESET."""
        ticks, values = self._history[name]
        return list(zip(ticks, values))

    def channel_at(self, name, tick):
        """Value of a channel at the end of `tick` (after RESET: 0, mcu_state 255)."""
        ticks, values = self._history[name]
        index = bisect.bisect_right(ticks, tick) - 1
        return values[index] if index >= 0 else CHANNEL_RESET_VALUES[name]

    # -- internals
    def _reset_session_state(self):
        self.tx_bytes = []
        self.changes = []
        self.last = None
        self._history = {name: ([], []) for name in v2.CHANNELS}
        self._pending_rx = []
        self._pending_events = []
        self._order = 0
        self._host_line_end_u = 0
        self._read_pos = 0
        self._lines = []
        self._lines_upto = 0
        self._line_buf = bytearray()
        self._line_first = None

    def _take_pending(self, first, count):
        """Scheduled events and bytes of ticks first .. first + count - 1 as tick offsets."""
        end = first + count
        if any(item[0] < first for item in self._pending_rx) or any(item[0] < first for item in self._pending_events):
            raise ValueError('scheduled_input_before_the_current_tick')
        events = [(t - first, code, arg) for t, _, code, arg in self._pending_events if t < end]
        rx = [(t - first, byte) for t, _, byte in self._pending_rx if t < end]
        self._pending_events = [item for item in self._pending_events if item[0] >= end]
        self._pending_rx = [item for item in self._pending_rx if item[0] >= end]
        return events, rx

    def _record(self, out):
        self.tx_bytes.extend(out.tx)
        self.changes.extend(out.changes)
        for change in out.changes:
            ticks, values = self._history[change.channel]
            ticks.append(change.tick)
            values.append(change.value)
        self.last = out


def _replace_serial(out, serial):
    from dataclasses import replace
    return replace(out, serial_out=serial)
