"""UBTS row executor process: rclpy inputs, 20 Hz single-threaded loop, stall watchdog, JSONL log, operator page.

Run (on the Jetson, through run_executor.sh; never with docker compose):
    PYTHONPATH=/opt/row_executor python3 -B -s -m row_executor.executor_main \
        --config /etc/row_executor/config.json --log-dir /var/log/row_executor [--motor-output] [--shadow-auto]

Modes
  SHADOW (default)  no BridgeClient object exists and nothing ever connects to the ESP bridge socket. The
                    Runner computes proposals (row following, stops) that are only logged/displayed.
  MOTOR             only when ALL of: config motor_output_enabled=true (which already requires the
                    measured breakaway / speed / yaw authority, config.feasibility) AND the --motor-output
                    flag AND an operator Arm on the page for the current run. Arm builds the BridgeClient
                    (first time) and takes the lease; every drive carries deadman:true, also 0/0.
  calibration       (calib_runs.py) config calib_motor_output_enabled=true + --motor-output: MOTOR for the
                    calibration run kinds 'arc' (C5) / 'rolldown' (C3b) only (request calib_start, then Arm, then
                    hold). A row run is armed only when the config also has motor_output_enabled=true.
  --shadow-auto     SHADOW only: the hold is emulated (held while a run is open; a new press is emulated
                    after every pause) and a new run starts by itself after a run ends. For logging the
                    proposals continuously while a human drives manually (MVP-0) and for the replay.

Threads
  ros       rclpy executor spin. Callbacks only append (receipt time.monotonic(), message) to bounded deques
            (InputBuffer); nothing else happens in a callback.
  main      ExecutorLoop.step() at 20 Hz (absolute schedule, missed ticks are skipped, never caught up):
            operator requests -> STOP -> stall pause -> all queued IMU (in order) -> every queued scan is
            stamped (ScanStamper) but only the NEWEST one is processed (older ones are dropped and counted)
            -> Runner.tick(now, hold) -> bridge output (MOTOR) -> status + one JSONL line.
  watchdog  if the main loop has not completed an iteration for stall_guard_s (0.35 s): one drive(0,0)
            through the same BridgeClient (its lock serialises seq/ticket; MOTOR only) and a pause request
            ('stall_guard', the Runner also detects the gap itself). Motion restarts only on a new hold press.
  http      operator page (operator_http.py): hold heartbeats, STOP (immediate op stop from the HTTP
            thread), Start/Arm/ack/params as requests executed by the main loop (the Runner is only ever
            touched by the main loop).
  log       JSONL writer with a bounded queue: the loop never blocks on the disk. Write errors are caught,
            dropped lines are counted from the first failure and the state (ok / failing / stalled / capped,
            dropped N) is in /status and on the page; the writer reopens the file and retries (review S8).

Bridge output (MOTOR, armed): ACTIVE -> the Runner's duties; PAUSED -> 0/0 (lease kept: moving=False so no
0.75 s input expiry; every drive renews the 30 s owner TTL). Sent at send_hz (10 Hz) and at once when the
command becomes 0/0. Run ended normally (row end, backstop, controller terminal hold) -> 0/0 until the
estimator reports stillness or terminal_stop_wait_s, then op stop (lease released, the human can drive
manually from the dashboard). Guard violation, operator STOP, bridge error, clock regression, shutdown ->
op stop at once and LATCHED (operator acknowledgement + new Start + Arm needed; never automatic).

Hold edges (review S3): HoldState tracks press ids (a second device's press while one is held is a conflict
that releases both; released / expired / conflicting ids are ignored forever). _hold_signal hands the Runner a
level that rises only with a genuinely new press, and shows one released tick when one press ended and a new
one began between two ticks, so every new press is its own edge and nothing else ever is.

Day-2 review fixes: EMERGENCY_PREFIXES also hold the bench interlock ends and calib_yaw_rate / calib_overspeed / calib_heading_change
(review_safety_day2 N3: op stop at once instead of the 0/0 fade). A 'calib_stop_test' end (calibration kind 'stop',
C7) keeps the 0/0 for up to calib_runs.STOP_TEST_WAIT_S before the op stop, so the observed stop is the pure fade; the
result is logged as event 'calib_stop'. Every run end is one 'run_end' event and ENDED ticks carry end_note
(review_correctness_day2 R6); ticks carry 'roll' / 'inner_lim' (F2) and 'nomo' (N9) when in use.

Single-row mode (config mode 'single_row', 2026-11-02; SINGLE_ROW_MODE.txt): Start may omit the row length (request
'start' with row_len_m null; a number adds the L - margin bound), the localisation record is SENSOR_ONLY_SINGLE_ROW (the
executor's LiDAR + raw IMU estimator; the config refuses require_native_navigation true), a native provider is an optional
cross-check (reported; only a fully valid COST_OCCUPIED pauses), turn_observe is refused, status carries 'single_row'
(telemetry + stop category), 'single_row_geometry' (operator canvas) and 'single_row_settings', and each JSONL tick 'sr'.
Every hold / freshness / STOP / bridge rule above is unchanged.

Loop exceptions (review S8): MOTOR -> as before: logged, emergency STOP (op stop at once, LATCHED, run ends), the
process ends after more than max_exceptions within exception_window_s. SHADOW -> the process never ends because
of exceptions (the JSONL record is the product of a shadow run): the run ends ('loop_exception'), the
exceptions are counted (status + page), traces are logged for the first ones and then as a rate-limited
summary, and after many consecutive failing steps the Runner is rebuilt (bounded number of times).

Steering branch import (2026-11-09, docs/steering/EXECUTOR_IMPORT_KO.md): the headland-turn observation runner
(turn_observation.py / turn_assist.py) is not part of this package (U-turn is out of scope). A 'turn_observe' request is
refused in every mode ('turn_observe_not_in_single_row_mode' in single-row mode as before, 'turn_observe_not_available'
otherwise) and the status reports turn_observe_available false. Everything else above is unchanged.

Front-wheel steering (config vehicle_profile 'front_steer', steering branch P4, docs/steering/EXECUTOR_STEER_KO.md): the
bridge client is SteerBridgeClient (V3 bridge). Every step hands the Runner the steering status of the last bridge reply
and whether it is required (armed). Output: ACTIVE -> drive(throttle, steer_mrad); PAUSED -> drive(0, held steering); a run
end -> drive(0, held steering) until stillness / terminal_stop_wait_s, then op stop (the MCU STOP holds the measured
steering); no steering target yet -> keepalive (lease only, no C line). After every drive the executor's sensor motion
(estimator gyro yaw rate, LiDAR speed while odometry is valid) goes to the bridge (op motion: its steering_ineffective
monitor). The stall watchdog sends drive(0, last steering) (never a centring). A bridge refusal or latch ends the run with
the design 6.6 steering reason of the bridge status when there is one (config.STEER_FAULT_REASONS), else bridge_error /
bridge_latched. The steering stop reasons (config.STEER_STOP_REASONS, except steering_authority_insufficient) stop at
once (op stop + LATCHED). After every op stop the STOP confirmation of the bridge is watched (event stop_confirmed /
stop_unconfirmed; an unconfirmed STOP latches the client).
"""
import argparse
import collections
import concurrent.futures
import hashlib
import json
import math
import os
import queue
import secrets
import signal
import sys
import threading
import time

import numpy as np

from . import calib_runs as CR
from . import config as C
from .bridge_client import (ARMED, LATCHED, BridgeClient, BridgeError, SteerBridgeClient, UnixSocketTransport,
                            recovery_hint)
from .core import ACTIVE, CALIB_KINDS, ENDED, IDLE, PAUSED, Runner, ScanStamper, verify_vendor
from .navigation_observation import NativeNavigationGate

SCHEMA_LOG = 'ubts.row_executor.log.v1'
PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
SUMS_FILE = 'SHA256SUMS'
# Ends that stop the motors at once (op stop + LATCHED) instead of the 0/0 fade. Day-2 review N3: a bench run that sees
# motion / a blocked front / lost odometry, and a calibration run whose vehicle does something its table does not command
# (yaw above the cap: an inner wheel stalled and the arc became a pivot; overspeed; an unexpected heading change).
EMERGENCY_PREFIXES = ('guard_violation', 'bridge_', 'operator_stop', 'clock_regression', 'shutdown',
                      'bench_motion', 'bench_front', 'bench_odometry',          # not 'bench_pattern_complete'
                      'calib_yaw_rate', 'calib_overspeed', 'calib_heading_change')
REQUEST_TIMEOUT_S = 5.
# front_steer: the steering stop reasons that stop at once (op stop + LATCHED); steering_authority_insufficient is a
# controller-authority end like a terminal hold (zero throttle with the held steering, then op stop)
STEER_EMERGENCY_REASONS = tuple(r for r in C.STEER_STOP_REASONS if r != 'steering_authority_insufficient') + \
    C.STEER_LINK_STOP_REASONS
STOP_WATCH_S = 1.             # front_steer: how long the STOP confirmation of an op stop is watched (bridge 400 ms grid)
MOTION_LIMIT = 10.            # op motion sanity range of the bridge (|yaw_rate_rad_s|, |speed_m_s|)


# --- source integrity -------------------------------------------------------------------------------
def source_files(package_dir=PACKAGE_DIR):
    """Every shipped file of the package (relative, '/'-separated), excluding caches and the sums file."""
    out = []
    for root, dirs, files in os.walk(package_dir):
        dirs[:] = sorted(d for d in dirs if d != '__pycache__' and not d.startswith('.'))
        for name in sorted(files):
            if name == SUMS_FILE or name.endswith(('.pyc', '.pyo')) or name.startswith('.'):
                continue
            out.append(os.path.relpath(os.path.join(root, name), package_dir).replace(os.sep, '/'))
    return sorted(out)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for block in iter(lambda: fh.read(1 << 16), b''):
            h.update(block)
    return h.hexdigest()


def render_sums(package_dir=PACKAGE_DIR):
    """SHA256SUMS text in coreutils format ('<hex>  <path>'), checkable with `sha256sum -c`."""
    return ''.join('%s  %s\n' % (sha256_file(os.path.join(package_dir, *rel.split('/'))), rel)
                   for rel in source_files(package_dir))


def verify_sources(package_dir=PACKAGE_DIR):
    """Compare every file with SHA256SUMS (missing, extra or changed files fail). Returns the sums digest."""
    path = os.path.join(package_dir, SUMS_FILE)
    with open(path, 'rb') as fh:
        recorded = fh.read()
    expected = render_sums(package_dir).encode('utf-8')
    if recorded != expected:
        listed = {line.split('  ', 1)[1] for line in recorded.decode('utf-8', 'replace').splitlines() if '  ' in line}
        actual = set(source_files(package_dir))
        detail = sorted(actual ^ listed) or ['content differs']
        raise RuntimeError('source tree does not match %s: %s' % (SUMS_FILE, ', '.join(detail[:8])))
    return hashlib.sha256(recorded).hexdigest()


# --- message fields (duck-typed: rclpy messages or the replay's fake messages) ------------------------
def stamp_seconds(header):
    try:
        stamp = header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def scan_fields(msg):
    """LaserScan -> plain fields. ranges stays the raw order (0 = no return on the S2 driver)."""
    return dict(ranges=np.asarray(msg.ranges, dtype=np.float32), angle_min=float(msg.angle_min),
                increment=float(msg.angle_increment), range_min=float(msg.range_min),
                range_max=float(msg.range_max), header=stamp_seconds(msg.header))


def imu_fields(msg):
    g, a = msg.angular_velocity, msg.linear_acceleration
    return (float(g.x), float(g.y), float(g.z)), (float(a.x), float(a.y), float(a.z)), stamp_seconds(msg.header)


def human_command(text):
    """/rover/control/requested (std_msgs/String JSON {'left', 'right'}) -> (left, right) or None."""
    try:
        data = json.loads(text)
        left, right = data.get('left'), data.get('right')
    except (ValueError, AttributeError, TypeError):
        return None
    return (left, right) if all(type(v) is int for v in (left, right)) else None


# --- shared state between threads --------------------------------------------------------------------
class InputBuffer:
    """Bounded FIFOs filled by the ROS callbacks; the main loop takes everything once per tick."""

    def __init__(self, scan_depth=16, imu_depth=400):
        self.lock = threading.Lock()
        self.scans = collections.deque(maxlen=scan_depth)
        self.imu = collections.deque(maxlen=imu_depth)
        self.human = None
        self.counts = dict(scans=0, imu=0, human=0, scan_overflow=0, imu_overflow=0)

    def put_scan(self, receipt, msg):
        with self.lock:
            if len(self.scans) == self.scans.maxlen:
                self.counts['scan_overflow'] += 1
            self.scans.append((receipt, msg))
            self.counts['scans'] += 1

    def put_imu(self, receipt, msg):
        with self.lock:
            if len(self.imu) == self.imu.maxlen:
                self.counts['imu_overflow'] += 1
            self.imu.append((receipt, msg))
            self.counts['imu'] += 1

    def put_human(self, receipt, text):
        cmd = human_command(text)
        with self.lock:
            self.counts['human'] += 1
            if cmd is not None:
                self.human = (receipt, cmd)

    def take(self):
        with self.lock:
            scans, imu = list(self.scans), list(self.imu)
            self.scans.clear()
            self.imu.clear()
            return scans, imu, self.human, dict(self.counts)


class HoldState:
    """Operator hold-to-run from the page heartbeats (independent of the bridge deadman field).

    Every press has its own id, chosen by the page at pointerdown and used for that one press only.
      beat(id)     the first beat of an id while NO press is current starts a press (serial + 1). Further beats
                   of the current id keep it alive.
      conflict     a beat of a different id while a press is current (a second device, a second tab): BOTH ids
                   are retired and nothing is current -> the level drops (a running pass pauses). Neither
                   device can continue or revive its press; the operator lifts the finger and presses again.
      release(id)  retires that id, current or not, also an id never seen (a beat that arrives after its own
                   release, e.g. reordered HTTP requests, can not start a press). release(None) retires the
                   current press.
      expiry       a current press without a beat for longer than timeout_s is retired for good (a network
                   hiccup never restarts motion by itself).
    Retired ids are ignored forever (bounded memory: the oldest of RETIRED_MAX are forgotten). The level
    (current()) can therefore only rise with a genuinely new id after every earlier press was released, retired
    or timed out; ExecutorLoop._hold_signal additionally makes every new press a separate edge for the Runner
    even when the previous press ended between two ticks (review S3, F8).
    """

    RETIRED_MAX = 4096

    def __init__(self, timeout_s=.5):
        self.timeout_s = float(timeout_s)
        self.lock = threading.Lock()
        self.press = None
        self.serial = 0                       # increments at every new (eligible) press
        self.last_beat = None
        self.expired = set()                  # retired ids (released, expired, superseded by a conflict)
        self._expired_order = collections.deque()
        self.counts = dict(beats=0, presses=0, releases=0, expiries=0, ignored=0, conflicts=0)

    def _expire_locked(self, now):
        if self.press is not None and now - self.last_beat > self.timeout_s:
            self._retire_locked(self.press)
            self.press = None
            self.counts['expiries'] += 1

    def _retire_locked(self, press):
        if press in self.expired:
            return
        self.expired.add(press)
        self._expired_order.append(press)
        while len(self._expired_order) > self.RETIRED_MAX:          # forget only the oldest ids
            self.expired.discard(self._expired_order.popleft())

    @staticmethod
    def _valid(press):
        return isinstance(press, str) and 4 <= len(press) <= 64

    def beat(self, press, now):
        """True when the beat holds a press (the page keeps the button on), False otherwise (page releases)."""
        return self.beat_detail(press, now)[0]

    def beat_detail(self, press, now):
        """(ok, why): why is None, 'invalid_press_id', 'press_retired' or 'press_conflict'."""
        if not self._valid(press):
            return False, 'invalid_press_id'
        with self.lock:
            self._expire_locked(now)
            if press in self.expired:
                self.counts['ignored'] += 1
                return False, 'press_retired'
            if self.press is not None and press != self.press:
                # second device / tab: retire both, the level drops; everybody presses again
                self._retire_locked(self.press)
                self._retire_locked(press)
                self.press = None
                self.counts['conflicts'] += 1
                return False, 'press_conflict'
            if self.press is None:
                self.press = press
                self.serial += 1
                self.counts['presses'] += 1
            self.last_beat = now
            self.counts['beats'] += 1
            return True, None

    def release(self, press=None):
        with self.lock:
            if press is None:
                if self.press is None:
                    return
                press = self.press
            elif not self._valid(press):
                return
            self._retire_locked(press)
            if press == self.press:
                self.press = None
                self.counts['releases'] += 1

    def current(self, now):
        """(active, serial of the current press or None)."""
        with self.lock:
            self._expire_locked(now)
            return (True, self.serial) if self.press is not None else (False, None)

    def active(self, now):
        return self.current(now)[0]

    def snapshot(self, now):
        with self.lock:
            age = None if self.last_beat is None else round(now - self.last_beat, 3)
            return dict(active=self.press is not None and age is not None and age <= self.timeout_s,
                        beat_age_s=age, serial=self.serial, **self.counts)


class JsonlLog:
    """Bounded asynchronous JSONL writer: put() never blocks; a line that can not be written is dropped and counted.

    Files: <dir>/row_executor_<UTC>_<pid>_<n>.jsonl, rotated at max_file_mb; tick lines stop (events
    continue) once max_total_mb was written by this process.

    Failure handling (review S8): each line goes to the file with os.write (no user-space buffer), so a failed
    write is attributed to exactly that line. Any error in the writer thread (ENOSPC, EIO, a removed directory,
    an unserialisable record, anything else) is caught: the line is counted in `dropped`, the file is closed,
    lines arriving during the next retry_s are dropped (counted) at once, then the same part file is reopened
    (the directory is re-created if it vanished) and writing continues as soon as the disk accepts it again.
    The first line after a failure starts with a newline, so a partially written line stays a separate
    malformed line (readers skip malformed / empty lines). The thread never dies by itself; should it be gone
    anyway, put() counts every line as dropped. health() -> state 'ok' | 'failing' (last write failed or the
    writer is gone) | 'stalled' (lines pending but no write finished for stall_s, e.g. a hung disk) | 'capped'
    (tick lines suppressed at max_total_mb), plus dropped / write_errors / last_error.
    """

    def __init__(self, directory, max_file_mb=100., max_total_mb=2000., queue_size=4000, wall=time.time,
                 retry_s=2., fsync_s=5., stall_s=5., write=None, open_fd=None):
        self.directory = directory
        os.makedirs(directory, exist_ok=True)
        self.max_file = int(max_file_mb * 1e6)
        self.max_total = int(max_total_mb * 1e6)
        self.queue = queue.Queue(maxsize=queue_size)
        self.wall = wall
        self.base = 'row_executor_%s_%d' % (time.strftime('%Y%m%dT%H%M%SZ', time.gmtime(wall())), os.getpid())
        self.retry_s, self.fsync_s, self.stall_s = float(retry_s), float(fsync_s), float(stall_s)
        self._write = write or os.write
        self._open_fd = open_fd or _open_append
        self.part = 0
        self.fd = None
        self.path = None
        self.file_bytes = 0
        self.total_bytes = 0
        self.lines_written = 0
        self.dropped = 0
        self.ticks_suppressed = 0
        self.write_errors = 0
        self.recoveries = 0
        self.last_error = None
        self.failing = False
        self.failing_since = None
        self.paths = []
        self._retry_at = None
        self._last_fsync = time.monotonic()
        self._last_progress = time.monotonic()
        self._count_lock = threading.Lock()
        self._closed = threading.Event()
        self.thread = threading.Thread(target=self._run, name='row-executor-log', daemon=True)
        self.thread.start()

    def _drop(self, n=1):
        with self._count_lock:
            self.dropped += n

    def put(self, record, tick=False):
        if not self.thread.is_alive():
            self._drop()                                 # writer gone: every line is lost, count it
            return
        if tick and self.total_bytes >= self.max_total:
            self.ticks_suppressed += 1
            return
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            self._drop()

    def health(self):
        """Writer state for the status page (see the class docstring)."""
        pending = self.queue.qsize()
        alive = self.thread.is_alive()
        if (not alive and not self._closed.is_set()) or self.failing:
            state = 'failing'
        elif pending and time.monotonic() - self._last_progress > self.stall_s:
            state = 'stalled'
        elif self.total_bytes >= self.max_total:
            state = 'capped'
        else:
            state = 'ok'
        return dict(state=state, dropped=self.dropped, write_errors=self.write_errors, last_error=self.last_error,
                    failing_since=self.failing_since, recoveries=self.recoveries, pending=pending,
                    lines=self.lines_written, bytes=self.total_bytes, ticks_suppressed=self.ticks_suppressed,
                    writer_alive=alive, file=os.path.basename(self.paths[-1]) if self.paths else None)

    def _close_fd(self, sync=False):
        fd, self.fd = self.fd, None
        if fd is None:
            return
        try:
            if sync:
                os.fsync(fd)
        except OSError:
            pass
        try:
            os.close(fd)
        except OSError:
            pass

    def _open(self, new_part=True):
        self._close_fd(sync=True)
        if new_part or self.path is None:
            self.part += 1
            path = os.path.join(self.directory, '%s_%03d.jsonl' % (self.base, self.part))
            self.fd = self._open_fd(path)
            self.path = path
            self.paths.append(path)
            self.file_bytes = 0
        else:                                   # reopen after a failure (append to the same part)
            os.makedirs(self.directory, exist_ok=True)
            self.fd = self._open_fd(self.path)

    def _write_all(self, data):
        view = memoryview(data)
        while view:
            n = self._write(self.fd, view)
            if not n:
                raise OSError('short write (0 bytes)')
            view = view[n:]

    def _fail(self, exc):
        self.write_errors += 1
        self.last_error = ('%s: %s' % (type(exc).__name__, exc))[:200]
        if not self.failing:
            self.failing = True
            self.failing_since = round(self.wall(), 3)
        self._close_fd()
        self._retry_at = time.monotonic() + self.retry_s

    def _write_record(self, record):
        try:
            data = (json.dumps(record, separators=(',', ':'), allow_nan=False, default=_json_default)
                    + '\n').encode('utf-8')
        except (TypeError, ValueError) as exc:          # a bad record: drop that line only
            self._drop()
            self.write_errors += 1
            self.last_error = ('record: %s' % exc)[:200]
            return
        if self.fd is None and self._retry_at is not None and time.monotonic() < self._retry_at:
            self._drop()                                  # failing: count from the first failure, no disk access
            return
        try:
            if self.fd is None:
                reopen = self.failing and self.path is not None
                self._open(new_part=not reopen)
                if reopen:
                    data = b'\n' + data          # terminate a line the failed write may have left half written
            elif self.file_bytes + len(data) > self.max_file:
                self._open()
            self._write_all(data)
        except Exception as exc:                          # ENOSPC, EIO, EROFS, removed directory, ...
            self._drop()
            self._fail(exc)
            return
        self.file_bytes += len(data)
        self.total_bytes += len(data)
        self.lines_written += 1
        self._last_progress = time.monotonic()
        self._retry_at = None
        if self.failing:
            self.failing = False
            self.recoveries += 1

    def _run(self):
        while True:
            try:
                try:
                    record = self.queue.get(timeout=.5)
                except queue.Empty:
                    record = None
                if record is _CLOSE:
                    break
                if record is not None:
                    self._write_record(record)
                else:
                    self._last_progress = time.monotonic()       # idle is not a stall
                if self.fd is not None and time.monotonic() - self._last_fsync >= self.fsync_s:
                    self._last_fsync = time.monotonic()
                    try:
                        os.fsync(self.fd)
                    except OSError as exc:
                        self._fail(exc)
            except Exception as exc:                              # never let the writer thread die
                self._drop()
                try:
                    self._fail(exc)
                except Exception:
                    pass
        self._close_fd(sync=True)
        self._closed.set()

    def close(self, timeout=5.):
        try:
            self.queue.put(_CLOSE, timeout=timeout)
        except queue.Full:
            pass
        self._closed.wait(timeout)


def _open_append(path):
    flags = os.O_WRONLY | os.O_CREAT | os.O_APPEND | getattr(os, 'O_BINARY', 0)   # O_BINARY: no CRLF on Windows
    return os.open(path, flags, 0o644)


_CLOSE = object()


def _json_default(value):
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        value = float(value)
        return value if math.isfinite(value) else None
    if isinstance(value, (set, tuple)):
        return list(value)
    return repr(value)


def _clean(value):
    """JSON-safe copy (non-finite floats -> None)."""
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, (bool, int, str)) or value is None:
        return value
    if isinstance(value, (float, np.floating)):
        value = float(value)
        return round(value, 6) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return repr(value)


class _NullLog:
    def __init__(self):
        self.records = []
        self.dropped = 0
        self.ticks_suppressed = 0
        self.paths = []

    def put(self, record, tick=False):
        self.records.append(record)

    def health(self):
        return dict(state='ok', dropped=0, write_errors=0, last_error=None, failing_since=None, recoveries=0,
                    pending=0, lines=len(self.records), bytes=0, ticks_suppressed=0, writer_alive=True, file=None)

    def close(self, timeout=0.):
        pass


# --- the loop ----------------------------------------------------------------------------------------
class ExecutorLoop:
    """Everything between the inputs and the bridge, without ROS. step(now) is one 20 Hz iteration.

    Only the thread calling step() touches the Runner. Other threads use: on_*_msg (callbacks),
    hold_beat/hold_release, emergency_stop, submit(...) (-> Future resolved by the next step), status().
    """

    def __init__(self, cfg, *, motor_output_flag=False, shadow_auto=False, bridge_factory=None,
                 stop_transport=None, log=None, clock=time.monotonic, wall=time.time, source_digest=None,
                 shadow_auto_restart_s=1., runner_factory=Runner, navigation_gate=None,
                 navigation_test_mode=False):
        cfg.validate()
        self.file_config = cfg
        self.config = cfg
        if navigation_gate is not None and type(navigation_gate) is not NativeNavigationGate:
            raise ValueError('memory_only_typed_native_navigation_gate')
        if navigation_gate is not None and navigation_gate.max_age_s > cfg.lidar_max_age_s:
            raise ValueError('native_map_age_not_looser_than_existing_lidar_age')
        self.navigation_gate = navigation_gate
        if type(navigation_test_mode) is not bool:
            raise ValueError('typed_navigation_test_scope')
        if navigation_test_mode and (navigation_gate is None or not navigation_gate.test_only or
                bridge_factory is None or stop_transport is None or clock is time.monotonic):
            raise ValueError('explicit_inprocess_generated_dependencies_required')
        self._navigation_test_mode = navigation_test_mode
        self._native_navigation = dict(state='WAIT', reason='NOT_EVALUATED', allow_motion=False)
        motor_capable = bool(cfg.motor_output_enabled or cfg.calib_motor_output_enabled)
        if motor_output_flag and not motor_capable:
            raise ValueError('--motor-output given but the config does not enable motor output')
        # MOTOR mode; for a calibration-only config (calib_motor_output_enabled) row runs are never armed (_req_arm)
        self.motor_allowed = bool(motor_capable and motor_output_flag)
        if shadow_auto and self.motor_allowed:
            raise ValueError('shadow_auto is refused with motor output')
        self.shadow_auto = bool(shadow_auto)
        self.shadow_auto_restart_s = float(shadow_auto_restart_s)
        self.clock, self.wall = clock, wall
        self._runner_factory = runner_factory
        self.runner = runner_factory(cfg)
        self.stamper = ScanStamper.from_config(cfg)
        self.inputs = InputBuffer()
        self.hold = HoldState(cfg.hold_timeout_s)
        self.log = log if log is not None else _NullLog()
        self.source_digest = source_digest
        self.steering = C.is_front_steer(cfg)
        if self.steering:
            self._bridge_factory = bridge_factory or (
                lambda: SteerBridgeClient.from_config(cfg, clock=clock, wall=wall))
        else:
            self._bridge_factory = bridge_factory or (
                lambda: BridgeClient(cfg.socket_path, timeout=cfg.bridge_timeout_s))
        self._steer_fed_t = None                # receipt time of the last steering status handed to the Runner
        self._stop_watch = None                 # front_steer: STOP confirmation being watched (dict)
        # SHADOW: no transport object at all (nothing can connect to the bridge socket).
        self._stop_transport = (stop_transport or UnixSocketTransport(cfg.socket_path, cfg.bridge_timeout_s)
                                if self.motor_allowed else None)
        self.bridge = None                      # created by the first Arm (MOTOR only)
        self._requests = queue.Queue()
        self._stop_event = threading.Event()
        self._stop_source = None
        self._pause_request = None
        self._send_lock = threading.Lock()
        self._last_sent = (0, 0)
        self._last_send_t = None
        self._ending_since = None
        self._ended_at = None
        self._auto_toggle = False
        self._hold_level = False                # the hold level the Runner saw last tick (see _hold_signal)
        self._hold_serial = None                # HoldState serial of that press
        self._end_request = None                # end the run at the next step (SHADOW loop exception)
        self._loop_error = None                 # last loop exception text (status / page)
        self._last_decision = None
        self._status = {}
        self._status_lock = threading.Lock()
        self.loop_done = None                   # clock() at the end of the last step (watchdog)
        self._wd_fired = False
        self._config_digest = cfg.digest()
        self.tick_count = 0
        self.counts = dict(scans_processed=0, scans_dropped=0, imu_fed=0, imu_bad=0, scan_bad=0, watchdog_fires=0,
                           watchdog_drives=0, bridge_errors=0, sends=0, emergency_stops=0, requests=0,
                           missed_ticks=0, loop_exceptions=0, runner_resets=0, hold_edges_split=0)
        self._last_scan_info = {}
        self._calib_logged = (None, 0, False)   # (run id, results logged, result event logged)
        self._calib_options = (None, None)      # (config digest, calib_runs.plan_options)
        self._run_end_logged = None             # run id of the last 'run_end' event (R6)
        self._calib_stop_logged = None          # run id of the last 'calib_stop' event (C7)
        self._event('config', config=cfg.to_dict(), config_digest=cfg.digest(), motor_allowed=self.motor_allowed,
                    shadow_auto=self.shadow_auto, feasibility=cfg.feasibility(), source_digest=source_digest,
                    vendor_commit=self.runner.vendor_commit)

    # --- logging -----------------------------------------------------------------------------------
    def _event(self, name, **fields):
        record = dict(k='event', event=name, t=round(self.clock(), 4), wall=round(self.wall(), 3))
        record.update(_clean(fields))
        self.log.put(record)

    # --- inputs (any thread) ---------------------------------------------------------------------------
    def on_scan_msg(self, receipt, msg):
        self.inputs.put_scan(float(receipt), msg)

    def on_imu_msg(self, receipt, msg):
        self.inputs.put_imu(float(receipt), msg)

    def on_human_msg(self, receipt, text):
        self.inputs.put_human(float(receipt), text)

    # --- operator (any thread) -------------------------------------------------------------------------
    def hold_beat(self, press):
        return self.hold.beat(press, self.clock())

    def hold_beat_detail(self, press):
        return self.hold.beat_detail(press, self.clock())

    def hold_release(self, press=None):
        self.hold.release(press)

    def submit(self, kind, **kwargs):
        future = concurrent.futures.Future()
        self._requests.put((kind, kwargs, future))
        return future

    def request(self, kind, timeout=REQUEST_TIMEOUT_S, **kwargs):
        """Submit and wait. A request the loop has not picked up within timeout is cancelled (never executed
        later); one the loop already started is waited for, so the answer is always its real outcome."""
        future = self.submit(kind, **kwargs)
        try:
            return future.result(timeout=timeout)
        except concurrent.futures.TimeoutError:
            if future.cancel():
                return dict(ok=False, error='loop_not_responding')
            try:
                return future.result(timeout=max(timeout, REQUEST_TIMEOUT_S))
            except concurrent.futures.TimeoutError:
                return dict(ok=False, error='request_outcome_unknown_check_status')

    def emergency_stop(self, source='operator'):
        """Immediate, from any thread. MOTOR: op stop to the bridge now (lease or not: a STOP button must
        stop the vehicle even when the dashboard holds the lease). SHADOW: no connection exists; only the
        shadow run ends (the vehicle is NOT stopped by this). The main loop then latches the client and ends
        the run. stop_sent: True / False (MOTOR, the bridge did not confirm) / None (SHADOW)."""
        self.counts['emergency_stops'] += 1
        self._stop_source = str(source)[:40]
        self._stop_event.set()
        sent, error = None, None
        if self.motor_allowed:
            self._watch_stop(self.clock(), 'operator_stop')       # front_steer only: STOP confirmation baseline first
            try:
                response = self._stop_transport({'op': 'stop', 'reason': ('row_executor_stop:' + self._stop_source)[:80]})
                sent = response.get('ok') is True
                if not sent:
                    error = str(response.get('error'))
            except BridgeError as exc:
                sent, error = False, exc.reason
        self._event('emergency_stop', source=self._stop_source, stop_sent=sent, error=error,
                    shadow=not self.motor_allowed)
        note = ('shadow run ended; the vehicle is NOT stopped by this button (dashboard STOP / E-stop)'
                if not self.motor_allowed else ('op stop confirmed by the bridge' if sent else
                                                 'STOP NOT CONFIRMED: use the dashboard STOP / physical E-stop'))
        return dict(ok=True, stop_sent=sent, error=error, shadow=not self.motor_allowed, note=note)

    def request_end(self, why):
        """Any thread: end the open run at the start of the next step (no bridge action of its own)."""
        self._end_request = str(why)[:40]

    def note_loop_exception(self, error):
        """Main thread, from run_loop: count a step exception and show it on the status page at once (the
        failing step never reached _publish)."""
        self.counts['loop_exceptions'] += 1
        self._loop_error = str(error)[:300]
        with self._status_lock:
            if self._status:
                self._status = dict(self._status, loop_exceptions=self.counts['loop_exceptions'],
                                    loop_error=self._loop_error, log=self.log.health())

    def reset_runner(self, why):
        """Main thread (SHADOW recovery after repeated step exceptions): a fresh Runner with the current config;
        run ids stay unique. Never used with motor output."""
        if self.motor_allowed:
            raise RuntimeError('reset_runner is SHADOW only')
        run_id = self.runner.run_id
        self.runner = self._runner_factory(self.config)
        self.runner.run_id = run_id
        self.stamper = ScanStamper.from_config(self.config)
        self._hold_level, self._hold_serial = False, None
        self.counts['runner_resets'] += 1
        self._event('runner_reset', why=why, run_id=run_id)

    def status(self):
        with self._status_lock:
            return dict(self._status)

    # --- watchdog (own thread) ------------------------------------------------------------------------
    def watchdog_check(self, now=None):
        """One watchdog evaluation; returns True when it fired (call every ~50 ms from its thread)."""
        now = self.clock() if now is None else now
        done = self.loop_done
        if done is None or now - done <= self.config.stall_guard_s:
            self._wd_fired = False
            return False
        if self._wd_fired:
            return False
        self._wd_fired = True
        self.counts['watchdog_fires'] += 1
        self._pause_request = 'stall_guard'
        drove = False
        client = self.bridge
        if self.steering:
            # front_steer: zero throttle with the last steering target (a watchdog never centres the steering)
            moving = client is not None and client.state == ARMED and self._last_sent[0] != 0 \
                and self._last_sent[1] is not None
            zero = (0, self._last_sent[1])
        else:
            moving = client is not None and client.state == ARMED and self._last_sent != (0, 0)
            zero = (0, 0)
        if moving:
            if self._send_lock.acquire(timeout=.25):
                try:
                    client.drive(*zero)
                    self._last_sent, self._last_send_t = zero, now
                    self.counts['watchdog_drives'] += 1
                    drove = True
                except BridgeError as exc:
                    self.counts['bridge_errors'] += 1
                    self._event('bridge_error', where='watchdog', reason=exc.reason)
                finally:
                    self._send_lock.release()
        self._event('watchdog_stall', loop_gap_s=round(now - done, 3), drive_zero_sent=drove)
        return True

    # --- main loop ------------------------------------------------------------------------------------
    def step(self, now=None):
        """One iteration. now=None (live): the clock is read again AFTER the inputs were processed, so a long
        on_scan shows up as a tick gap (Runner stall rule -> PAUSED, 0/0) and a watchdog pause raised while this
        step was busy is applied before the tick: a stale non-zero command can never follow a stall."""
        live = now is None
        now = self.clock() if live else float(now)
        c0 = time.perf_counter()
        self.tick_count += 1
        self._process_requests(now)
        if self._stop_event.is_set():
            self._handle_stop(now)
        end, self._end_request = self._end_request, None
        if end and self.runner.state in (PAUSED, ACTIVE):
            self.runner.end_run(now, end)
        scan_info = self._feed_inputs()
        if live:
            now = max(now, self.clock())
        pause, self._pause_request = self._pause_request, None
        if pause:
            self.runner.pause(now, pause)
        # Additional admission only: original raw sensors, output guards, Stop,
        # ownership and hold semantics still apply. No I/O occurs in this gate.
        if self.runner.run_kind == 'row' and self.runner.state in (PAUSED, ACTIVE) and \
                self.config.mode == C.SINGLE_ROW_MODE:
            # Single-row mode: localisation = the executor's own LiDAR + raw IMU estimator (SENSOR_ONLY_SINGLE_ROW; the
            # config refuses require_native_navigation true). A native provider is an optional cross-check: reported,
            # never a motion permit; only a fully valid same-snapshot COST_OCCUPIED report pauses (fail-safe direction).
            self._native_navigation = self._single_row_navigation(now)
            if not self._native_navigation['allow_motion']:
                self.hold.release()
                self.runner.pause(now, 'native_cross_check_cost_occupied')
        elif self.runner.run_kind == 'row' and self.runner.state in (PAUSED, ACTIVE):
            if self.navigation_gate is not None:
                self._native_navigation = self.navigation_gate.evaluate(now)
                if self.navigation_gate.test_only and not self._navigation_test_mode:
                    self._native_navigation.update(state='WAIT', reason='GENERATED_PROVIDER_FORBIDDEN', allow_motion=False)
            elif self.config.require_native_navigation:
                self._native_navigation = dict(state='WAIT', reason='PROVIDER_MISSING', allow_motion=False,
                    provider_present=False, pose_present=False, occupancy_complete=False,
                    forward_hazard='UNKNOWN', source_age_known=False, SDK_adapter_qualified=False)
            else:
                self._native_navigation = dict(state='LEGACY_SENSOR_ONLY_DEVELOPMENT',
                    reason='EXPLICIT_LEGACY_MODE', allow_motion=True, provider_present=False,
                    SDK_adapter_qualified=False)
            if not self._native_navigation['allow_motion']:
                # Retire this press. Map recovery can never manufacture a new
                # rising hold edge from the old button that was already held.
                self.hold.release()
                self.runner.pause(now, 'native_navigation_' + self._native_navigation['reason'].lower())
        if self.steering:
            self._feed_steering()
        hold = self._hold_signal(now)
        decision = self.runner.tick(now, hold)
        self._last_decision = decision
        self._calib_events()
        if self.shadow_auto:
            self._shadow_auto_restart(now)
        sent = self._output(now, decision)
        loop_ms = (time.perf_counter() - c0) * 1e3
        self._publish(now, decision, hold, sent, scan_info, loop_ms)
        self.loop_done = self.clock()
        return decision

    def _single_row_navigation(self, now):
        """Localisation record of a single-row tick: the executor estimator, plus the optional native cross-check."""
        cross = None
        gate = self.navigation_gate
        if gate is not None:
            native = gate.evaluate(now)
            if gate.test_only and not self._navigation_test_mode:
                native.update(state='WAIT', reason='GENERATED_PROVIDER_FORBIDDEN', allow_motion=False)
            cross = {k: native.get(k) for k in ('state', 'reason', 'tracking', 'pose_quality', 'occupancy_complete',
                                                'forward_hazard', 'source_age_known', 'sequence',
                                                'generated_test_provider')}
        veto = cross is not None and cross['reason'] == 'COST_OCCUPIED'
        return dict(state=C.SINGLE_ROW_LOCALISATION,
                    reason='NATIVE_CROSS_CHECK_COST_OCCUPIED' if veto else 'EXECUTOR_LIDAR_IMU_ESTIMATOR',
                    allow_motion=not veto, provider_present=gate is not None, native_required=False,
                    native_cross_check=cross, SDK_adapter_qualified=False,
                    localisation='executor LiDAR + raw IMU estimator: corridor-relative lateral / heading errors, '
                                 'scan-matching speed and resolved_polyline distance')

    def _feed_inputs(self):
        scans, imus, human, counts = self.inputs.take()
        for receipt, msg in imus:
            try:
                gyro, acc, _ = imu_fields(msg)
            except (AttributeError, TypeError, ValueError):
                self.counts['imu_bad'] += 1
                continue
            if self.runner.feed_imu(receipt, gyro, acc):
                self.counts['imu_fed'] += 1
        info = dict(received=len(scans), imu=len(imus))
        if scans:
            stamps = []
            for receipt, msg in scans:
                header = stamp_seconds(getattr(msg, 'header', None))
                stamps.append(self.stamper.stamp(receipt, header))
            self.counts['scans_dropped'] += len(scans) - 1
            receipt, msg = scans[-1]
            try:
                f = scan_fields(msg)
            except (AttributeError, TypeError, ValueError):
                f = None
                self.counts['scan_bad'] += 1
            if f is not None:
                stats = self.runner.feed_scan(receipt, f['ranges'], f['angle_min'], f['increment'],
                                              f['range_max'], f['range_min'], stamp=stamps[-1])
                self.counts['scans_processed'] += 1
                info.update(accepted=bool(stats.get('accepted')), reason=stats.get('reason'),
                            proc_ms=None if stats.get('proc_s') is None else round(stats['proc_s'] * 1e3, 2),
                            near_unconfirmed=stats.get('near_unconfirmed'), masked=stats.get('masked'),
                            age_at_feed_ms=round((self.clock() - receipt) * 1e3, 1),
                            header_offset_s=None if f['header'] is None else round(receipt - f['header'], 4),
                            stamp_lag_s=round(receipt - stamps[-1], 4))
        info['human'] = None if human is None else list(human[1])
        info['human_age_s'] = None if human is None else round(self.clock() - human[0], 3)
        info['buffer'] = counts
        self._last_scan_info = info
        return info

    def _hold_signal(self, now):
        if not self.shadow_auto:
            active, serial = self.hold.current(now)
            if self.motor_allowed and (self.bridge is None or self.bridge.state != ARMED):
                active = False      # MOTOR: Start -> Arm -> hold (a press before Arm never resumes the run)
            if not active:
                self._hold_level = False
                return False
            if self._hold_level and serial != self._hold_serial:
                # The press the Runner saw ended and a NEW press began between two ticks: show the release for
                # this tick (pause), the next tick is the new press's own rising edge.
                self._hold_level = False
                self.counts['hold_edges_split'] += 1
                return False
            self._hold_level, self._hold_serial = True, serial
            return True
        state = self.runner.state
        if state == ACTIVE:
            return True
        if state == PAUSED:
            # emulated operator: release and press again on every tick while paused
            self._auto_toggle = not self._auto_toggle
            return self._auto_toggle
        return False

    def _shadow_auto_restart(self, now):
        if self.runner.state in (IDLE, ENDED):
            if self._ended_at is None:
                self._ended_at = now
            elif now - self._ended_at >= self.shadow_auto_restart_s:
                length = None if self.config.mode == C.SINGLE_ROW_MODE else self.config.max_row_len_m
                self._start(now, length, source='shadow_auto')
        else:
            self._ended_at = None

    # --- requests (executed here, on the main loop thread) -------------------------------------------------
    def _process_requests(self, now):
        while True:
            try:
                kind, kwargs, future = self._requests.get_nowait()
            except queue.Empty:
                return
            if not future.set_running_or_notify_cancel():
                continue
            self.counts['requests'] += 1
            try:
                handler = getattr(self, '_req_' + kind, None)
                result = handler(now, **kwargs) if handler else dict(ok=False, error='unknown_request')
            except Exception as exc:          # a request must never kill the loop
                result = dict(ok=False, error='request_failed:%s' % exc)
            self._event('request', kind=kind, args=kwargs, result=result)
            future.set_result(result)

    def _start_blocked(self):
        if self.runner.state in (PAUSED, ACTIVE):
            return dict(ok=False, error='run_in_progress')
        if self.bridge is not None and self.bridge.state == ARMED:
            return dict(ok=False, error='bridge_still_armed')
        if self.bridge is not None and self.bridge.state == LATCHED:
            return dict(ok=False, error='acknowledge_latch_first', latch=self.bridge.latch_reason)
        return None

    def _start(self, now, row_len_m, source='operator'):
        blocked = self._start_blocked()
        if blocked is not None:
            return blocked
        try:
            if self.config.profile == 'bench':
                run_id = self.runner.start_bench(now)
            else:
                run_id = self.runner.start_run(now, row_len_m)
        except (ValueError, RuntimeError) as exc:
            return dict(ok=False, error=str(exc))
        self._ending_since = None
        self._ended_at = None
        self._event('start', run_id=run_id, row_len_m=row_len_m, source=source,
                    backstop=self.runner.backstop.summary() if self.runner.backstop else None)
        return dict(ok=True, run_id=run_id)

    def _req_start(self, now, row_len_m=None):
        number = (not isinstance(row_len_m, bool) and isinstance(row_len_m, (int, float))
                  and math.isfinite(row_len_m))
        if self.config.profile != 'bench' and self.config.mode == C.SINGLE_ROW_MODE:
            # single-row mode: the row length is optional (None -> distance cap single_row_max_row_length_m and its
            # time cap; a number adds the L - backstop_margin_m bound); anything else is refused
            if row_len_m is not None and not number:
                return dict(ok=False, error='row_len_m_must_be_a_number_or_null')
        elif self.config.profile != 'bench':
            if not number:
                return dict(ok=False, error='row_len_m_required')
        return self._start(now, None if row_len_m is None else float(row_len_m))

    def _req_turn_observe(self, now, params=None):
        """Always refused: the headland-turn observation (turn_observation.py) is not part of this package (U-turn is
        out of scope; docs/steering/EXECUTOR_IMPORT_KO.md). Single-row mode keeps its own reason."""
        if self.config.mode == C.SINGLE_ROW_MODE:
            return dict(ok=False, error='turn_observe_not_in_single_row_mode')   # no headland turn in single-row mode
        return dict(ok=False, error='turn_observe_not_available')

    def _req_calib_start(self, now, calib_kind=None, params=None):
        """A calibration run (calib_runs.py): MOTOR mode and calib_motor_output_enabled only. Then Arm, then hold."""
        if not self.motor_allowed:
            return dict(ok=False, error='calibration_needs_motor_mode')
        problems = C.calib_output_problems(self.config)
        if problems:
            return dict(ok=False, error='calibration_output_not_allowed', problems=problems)
        blocked = self._start_blocked()
        if blocked is not None:
            return blocked
        try:
            plan = CR.make_plan(self.config, calib_kind, {} if params is None else params)
        except CR.CalibPlanError as exc:
            return dict(ok=False, error='invalid_calibration', problems=exc.problems)
        try:
            run_id = self.runner.start_calibration(now, plan)
        except (ValueError, RuntimeError) as exc:
            return dict(ok=False, error=str(exc))
        self._ending_since = None
        self._ended_at = None
        self._event('calib_start', run_id=run_id, plan=plan.to_dict())
        return dict(ok=True, run_id=run_id, plan=plan.to_dict())

    def _req_end(self, now):
        if self.runner.state not in (PAUSED, ACTIVE):
            return dict(ok=False, error='no_run')
        self.runner.end_run(now, 'operator_end')
        return dict(ok=True)

    def _req_arm(self, now):
        if not self.motor_allowed:
            return dict(ok=False, error='motor_output_not_allowed')
        if self.runner.state != PAUSED:
            return dict(ok=False, error='start_a_run_first' if self.runner.state in (IDLE, ENDED) else 'release_hold_first')
        if self.hold.active(now):
            return dict(ok=False, error='release_hold_first')
        if self.runner.run_kind in CALIB_KINDS:
            problems = C.calib_output_problems(self.config)
            if problems:
                return dict(ok=False, error='calibration_output_not_allowed', problems=problems)
        else:
            if self.config.profile == 'field' and not self.config.motor_output_enabled:
                return dict(ok=False, error='row_motor_output_not_enabled',
                            hint='this config enables calibration runs only (calib_motor_output_enabled)')
            feasibility = self.config.feasibility()
            needed = ('ok',) if self.config.profile == 'field' else ('ok', 'unknown')
            if feasibility['status'] not in needed:
                return dict(ok=False, error='config_not_feasible', feasibility=feasibility)
        if self.bridge is None:
            self.bridge = self._bridge_factory()
        if self.bridge.state == LATCHED:
            return dict(ok=False, error='acknowledge_latch_first', latch=self.bridge.latch_reason)
        if self.bridge.state == ARMED:
            return dict(ok=False, error='already_armed')
        try:
            self.bridge.arm()
        except BridgeError as exc:
            self.counts['bridge_errors'] += 1
            self.runner.end_run(now, 'bridge_arm_failed')
            return dict(ok=False, error=exc.reason, hint=recovery_hint(self.bridge.last_status, exc.reason))
        self._last_sent, self._last_send_t = ((0, None) if self.steering else (0, 0)), None
        self._ending_since = None
        return dict(ok=True, owner=self.bridge.owner)

    def _req_ack(self, now):
        state = None if self.bridge is None else self.bridge.acknowledge_latch()
        return dict(ok=True, bridge=state)

    def _req_params(self, now, params=None):
        if self.runner.state in (PAUSED, ACTIVE):
            return dict(ok=False, error='end_the_run_first')
        if self.bridge is not None and self.bridge.state == ARMED:
            return dict(ok=False, error='bridge_armed')
        try:
            new = C.onsite_update(self.file_config, self.config, params)
        except C.ConfigError as exc:
            return dict(ok=False, error='invalid_params', problems=exc.problems)
        old = self.config
        run_id = self.runner.run_id
        self.runner = self._runner_factory(new)
        self.runner.run_id = run_id                 # run ids stay unique in the log across a params change
        self.config = new
        self._config_digest = new.digest()
        keys = C.ONSITE_KEYS + ('measured_yaw_at_max_diff_deg_s', 'max_yaw_rate_deg_s')    # R2: derived keys too
        self._event('params', old={k: getattr(old, k) for k in keys},
                    new={k: getattr(new, k) for k in keys}, config_digest=new.digest(),
                    feasibility=new.feasibility())
        return dict(ok=True, config_digest=new.digest(), feasibility=new.feasibility(),
                    derived={k: getattr(new, k) for k in keys[len(C.ONSITE_KEYS):]})

    # --- STOP -------------------------------------------------------------------------------------------
    def _handle_stop(self, now):
        self._stop_event.clear()
        source = self._stop_source or 'operator'
        if self.bridge is not None and self.bridge.state != LATCHED:
            self._watch_stop(now, 'operator_stop')
            with self._send_lock:
                self.bridge.stop('row_executor_stop:' + source, latch=True)
            self._last_sent = self._zero_sent()
        if self.runner.state in (PAUSED, ACTIVE):
            self.runner.end_run(now, 'operator_stop')

    # --- front_steer helpers ----------------------------------------------------------------------------
    def _zero_sent(self):
        """The zero output record: (0, 0), or (0, last steering target) for front_steer."""
        if self.steering:
            return 0, self._last_sent[1] if isinstance(self._last_sent, tuple) and len(self._last_sent) == 2 else None
        return 0, 0

    def _feed_steering(self):
        """front_steer: the steering status of the last bridge reply to the Runner (once per reply), and whether it is
        required (the client holds the lease)."""
        client = self.bridge
        self.runner.steer_required = client is not None and client.state == ARMED
        seen = None if client is None or not hasattr(client, 'steering') else client.steering()
        if seen is not None and seen[0] != self._steer_fed_t:
            self._steer_fed_t = seen[0]
            self.runner.feed_steering(seen[0], seen[1], seen[2])

    def _end_from_bridge(self, now, default, detail=None):
        """End the open run after a bridge refusal / latch: the design 6.6 steering reason of the bridge status when it
        has one (front_steer), else default (bridge_error / bridge_latched)."""
        if self.runner.state not in (PAUSED, ACTIVE):
            return
        reason = None
        if self.steering and self.bridge is not None and hasattr(self.bridge, 'fault_reason'):
            reason = self.bridge.fault_reason()
        note = None if detail is None else str(detail)[:120]
        if reason is not None:
            note = '%s:%s' % (default, note) if note else default
        self.runner.end_run(now, reason or default, note=note)

    def _watch_stop(self, now, reason):
        """front_steer: watch the bridge's confirmation of the op stop just sent (steering_host stop counters)."""
        if not self.steering or self._stop_transport is None or self._stop_watch is not None:
            return
        counters = {}
        try:
            status = self._stop_transport({'op': 'status'})
            host = status.get('steering_host') if isinstance(status, dict) else None
            counters = dict(host.get('counters') or {}) if isinstance(host, dict) else {}
        except BridgeError:
            pass
        self._stop_watch = dict(t0=float(now), reason=str(reason), sent0=counters.get('stop_sent', 0),
                                confirmed0=counters.get('stop_confirmed', 0))

    def _check_stop_watch(self, now):
        watch = self._stop_watch
        if watch is None:
            return
        try:
            status = self._stop_transport({'op': 'status'})
        except BridgeError:
            status = None
        host = status.get('steering_host') if isinstance(status, dict) else None
        counters = dict(host.get('counters') or {}) if isinstance(host, dict) else {}
        state = host.get('stop_state') if isinstance(host, dict) else None
        sent = counters.get('stop_sent', 0) > watch['sent0']
        elapsed = round(now - watch['t0'], 4)
        if sent and state == 'confirmed' and counters.get('stop_confirmed', 0) > watch['confirmed0']:
            self._event('stop_confirmed', reason=watch['reason'], elapsed_s=elapsed,
                        resent=counters.get('stop_resent', 0))
            self._stop_watch = None
        elif sent and state == 'unconfirmed':
            self._event('stop_unconfirmed', reason=watch['reason'], elapsed_s=elapsed,
                        resent=counters.get('stop_resent', 0))
            client = self.bridge
            if client is not None and client.state != LATCHED:
                client._latch('stop_unconfirmed', send_stop=False)
            self._stop_watch = None
        elif now - watch['t0'] > STOP_WATCH_S:
            self._event('stop_watch_timeout', reason=watch['reason'], elapsed_s=elapsed, stop_state=state)
            self._stop_watch = None

    def _send_motion(self):
        """front_steer: the executor's sensor motion (estimator gyro yaw rate, LiDAR speed with valid odometry) for the
        bridge's steering_ineffective monitor (op motion; never latches)."""
        s = self.runner.last_snapshot or {}
        yaw, speed = s.get('yaw_rate'), s.get('speed')
        if s.get('odom_valid') is not True:
            return False
        for value in (yaw, speed):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
                    or abs(value) > MOTION_LIMIT:
                return False
        if self.bridge.motion(float(yaw), float(speed)):
            self.counts['motion_sent'] = self.counts.get('motion_sent', 0) + 1
            return True
        return False

    def _steer_output(self, now, decision):
        """front_steer output (module docstring). Returns (throttle, steer_mrad) sent this tick or None."""
        client = self.bridge
        if client is None:
            return None
        if client.state == LATCHED:
            if self.runner.state in (PAUSED, ACTIVE):
                self._end_from_bridge(now, 'bridge_latched', client.latch_reason)
            return None
        if client.state != ARMED:
            return None
        state = self.runner.state
        if state in (PAUSED, ACTIVE):
            throttle = decision.throttle if state == ACTIVE else 0
            return self._steer_send_if_due(now, throttle, decision.steer_mrad)
        reason = self.runner.end_reason or 'no_run'
        if reason.startswith(EMERGENCY_PREFIXES) or reason in STEER_EMERGENCY_REASONS:
            self._watch_stop(now, reason)
            with self._send_lock:
                client.stop('row_executor:' + reason, latch=True)
            self._last_sent = self._zero_sent()
            self._event('bridge_stop', reason=reason, latched=True)
            return None
        if self._ending_since is None:
            self._ending_since = now
        snapshot = self.runner.last_snapshot or {}
        still = bool(snapshot.get('still')) and self._last_sent[0] == 0
        if still or now - self._ending_since >= self.config.terminal_stop_wait_s:
            self._watch_stop(now, reason)
            with self._send_lock:
                ok = client.stop('row_executor_end:' + reason)
            self._last_sent = self._zero_sent()
            self._ending_since = None
            self._event('bridge_stop', reason=reason, latched=not ok, observed_still=still)
            return None
        return self._steer_send_if_due(now, 0, decision.steer_mrad)

    def _steer_send_if_due(self, now, throttle, steer_mrad):
        period = 1. / self.config.send_hz - 1e-6
        if steer_mrad is None:
            # no steering target known yet (no steering status since the arm): keep the lease, send no C line
            if self._last_send_t is not None and now - self._last_send_t < period:
                return None
            with self._send_lock:
                try:
                    self.bridge.keepalive()
                except BridgeError as exc:
                    self.counts['bridge_errors'] += 1
                    self._event('bridge_error', where='keepalive', reason=exc.reason)
                    self._end_from_bridge(now, 'bridge_error', exc.reason)
                    return None
                self._last_sent, self._last_send_t = (0, None), now
            return None
        want = (int(throttle), int(steer_mrad))
        due = (self._last_send_t is None or (want[0] == 0 and self._last_sent[0] != 0)
               or now - self._last_send_t >= period)
        if not due:
            return None
        with self._send_lock:
            try:
                self.bridge.drive(want[0], want[1])
            except BridgeError as exc:
                self.counts['bridge_errors'] += 1
                self._last_sent = (0, want[1])
                self._event('bridge_error', where='drive', reason=exc.reason)
                self._end_from_bridge(now, 'bridge_error', exc.reason)
                return None
            self._last_sent, self._last_send_t = want, now
            self.counts['sends'] += 1
        if self._link_event_after_send(now):
            return None
        self._send_motion()
        return self._last_sent

    def _link_event_after_send(self, now):
        """front_steer (review RS-01): the steering status of the drive reply just received may show an MCU C TTL event
        (an F TTL line the host counted, steering_host counters) whose TTL bit a C line sent after it has already
        cleared in the status lines. That C restarts the drive 120 ms after the TTL brake (spec 6.3, 8.2) with no
        operator action, so the run ends here, in the same tick, and the bridge gets the STOP now instead of one
        executor period later. True when it stopped."""
        self._feed_steering()
        monitor = getattr(self.runner, 'steer_monitor', None)
        if monitor is None or not monitor.confirmed() or 'TTL' not in monitor.link_events():
            return False
        if self.runner.state in (PAUSED, ACTIVE):
            self.runner.end_run(now, 'link_ttl_expired', note='mcu_ttl_event_in_drive_reply')
        client = self.bridge
        self._watch_stop(now, 'link_ttl_expired')
        with self._send_lock:
            client.stop('row_executor:link_ttl_expired', latch=True)
        self._last_sent = self._zero_sent()
        self.counts['ttl_event_stops'] = self.counts.get('ttl_event_stops', 0) + 1
        self._event('bridge_stop', reason='link_ttl_expired', latched=True, same_tick=True)
        return True

    # --- output -----------------------------------------------------------------------------------------
    def _output(self, now, decision):
        """MOTOR only. Returns the (left, right) sent this tick or None ((throttle, steer_mrad) for front_steer)."""
        if self.steering:
            sent = self._steer_output(now, decision)
            self._check_stop_watch(now)
            return sent
        client = self.bridge
        if client is None:
            return None
        if client.state == LATCHED:
            if self.runner.state in (PAUSED, ACTIVE):
                self.runner.end_run(now, 'bridge_latched')
            return None
        if client.state != ARMED:
            return None
        state = self.runner.state
        if state in (PAUSED, ACTIVE):
            want = (decision.left, decision.right) if state == ACTIVE else (0, 0)
            return self._send_if_due(now, want)
        reason = self.runner.end_reason or 'no_run'
        if reason.startswith(EMERGENCY_PREFIXES):
            with self._send_lock:
                client.stop('row_executor:' + reason, latch=True)
            self._last_sent = (0, 0)
            self._event('bridge_stop', reason=reason, latched=True)
            return None
        if self._ending_since is None:
            self._ending_since = now
        snapshot = self.runner.last_snapshot or {}
        still = bool(snapshot.get('still')) and self._last_sent == (0, 0)
        wait = self.config.terminal_stop_wait_s
        if reason == 'calib_stop_test':
            wait = max(wait, CR.STOP_TEST_WAIT_S)      # C7: observe the pure 0/0 fade stop (lease kept, 0/0 sent)
        if still or now - self._ending_since >= wait:
            with self._send_lock:
                ok = client.stop('row_executor_end:' + reason)
            self._last_sent = (0, 0)
            self._ending_since = None
            self._event('bridge_stop', reason=reason, latched=not ok, observed_still=still)
            return None
        return self._send_if_due(now, (0, 0))

    def _send_if_due(self, now, want):
        due = (self._last_send_t is None or (want == (0, 0) and self._last_sent != (0, 0))
               or now - self._last_send_t >= 1. / self.config.send_hz - 1e-6)
        if not due:
            return None
        with self._send_lock:
            try:
                self.bridge.drive(int(want[0]), int(want[1]))
            except BridgeError as exc:
                self.counts['bridge_errors'] += 1
                self._last_sent = (0, 0)
                self._event('bridge_error', where='drive', reason=exc.reason,
                            hint=recovery_hint(self.bridge.last_status, exc.reason))
                if self.runner.state in (PAUSED, ACTIVE):
                    self.runner.end_run(now, 'bridge_error')
                return None
            self._last_sent, self._last_send_t = (int(want[0]), int(want[1])), now
            self.counts['sends'] += 1
        return self._last_sent

    # --- status + log -------------------------------------------------------------------------------------
    def _publish(self, now, d, hold, sent, scan_info, loop_ms):
        s = d.summary
        bridge = None if self.bridge is None else self.bridge.snapshot()
        # the decision's own run id: a shadow-auto restart in this same step must not relabel the old run's end
        record = dict(k='tick', t=round(now, 4), run=s.get('run_id', self.runner.run_id), state=d.state, reason=d.reason,
                      L=d.left, R=d.right, sent=None if sent is None else list(sent), hold=bool(hold),
                      ctl=[s.get('controller_state'), s.get('controller_reason')],
                      lidar_age=s.get('lidar_age_s'), imu_age=s.get('imu_age_s'),
                      corridor=s.get('corridor_valid'), corridor_reason=s.get('corridor_reason'),
                      lat=s.get('lateral_offset_m'), head=s.get('heading_error_deg'), width=s.get('corridor_width_m'),
                      end=s.get('row_end_ahead_m'), front=s.get('front_clearance_m'),
                      sideL=s.get('side_clearance_left_m'), sideR=s.get('side_clearance_right_m'),
                      dist=s.get('distance_m'), v=s.get('speed_m_s'), yaw=s.get('yaw_rate_deg_s'),
                      still=s.get('still'), vib=s.get('vibration_rms'), scan=scan_info,
                      loop_ms=round(loop_ms, 2), bridge=None if bridge is None else bridge['state'])
        record['native_navigation'] = dict(self._native_navigation,
            required=self.config.require_native_navigation)
        if 'backstop' in s:
            record['backstop'] = s['backstop']
        if 'calib_tick' in s and self.runner.run_kind in CALIB_KINDS:
            record['calib'] = s['calib_tick']
        if d.state == ENDED or 'guard_violation' in s:
            record['end_reason'] = d.reason if d.state == ENDED else self.runner.end_reason
        if d.state == ENDED and s.get('end_note'):
            record['end_note'] = s['end_note']                  # review_correctness_day2 R6
        if 'rolling' in s:
            record['roll'] = bool(s['rolling'])                 # R6: rolling inner-wheel threshold in use (F2)
        if s.get('inner_limited_ticks'):
            record['inner_lim'] = s['inner_limited_ticks']
        if s.get('no_motion_drive_s'):
            record['nomo'] = s['no_motion_drive_s']             # day-2 N9 budget in use
        sr = s.get('single_row')
        if sr is not None:                                      # single-row mode: compact per-tick telemetry
            record['sr'] = dict(ph=sr.get('phase'), lref=sr.get('lateral_reference_m'), look=sr.get('lookahead_error_deg'),
                                diff=sr.get('differential_pct'), lim=sr.get('differential_limit_pct'),
                                sat=sr.get('saturated'), auth=sr.get('authority_state'), bias=sr.get('bias_pct'),
                                base=sr.get('base_duty_pct'), vt=sr.get('target_speed_m_s'),
                                rem=sr.get('end_remaining_m'), conf=(sr.get('confidence') or {}).get('level'),
                                cat=sr.get('stop_category'))
        if self.steering:                                       # front_steer: the output and the steering feedback
            st = s.get('steering') or {}
            record.update(T=d.throttle, S=d.steer_mrad,
                          steer=dict(meas=st.get('measured_mrad'), age=st.get('age_s'), req=st.get('required'),
                                     arm=st.get('arm_state'), stop=st.get('stop_state'), faults=st.get('faults'),
                                     fault=st.get('fault_reason'), sat=st.get('authority_saturated_s')))
        self.log.put(_clean(record), tick=True)
        self._run_end_event(d)
        status = dict(record)
        armed = bridge is not None and bridge['state'] == ARMED
        status.update(t_wall=round(self.wall(), 3), mode='MOTOR' if self.motor_allowed else 'SHADOW',
                      # what the executor does to the vehicle right now (page banner, review S9)
                      vehicle_control=('armed' if armed else 'not_armed') if self.motor_allowed else 'none',
                      stop_button='vehicle_stop' if self.motor_allowed else 'ends_shadow_run_only',
                      log=self.log.health(), loop_exceptions=self.counts['loop_exceptions'],
                      loop_error=self._loop_error,
                      shadow_auto=self.shadow_auto, profile=self.config.profile, end_reason=self.runner.end_reason,
                      end_note=getattr(self.runner, 'end_note', None),
                      run_kind=self.runner.run_kind, hold_server=self.hold.snapshot(now),
                      bridge_detail=bridge, feasibility=self.config.feasibility(),
                      params={k: getattr(self.config, k) for k in C.ONSITE_KEYS},
                      yaw_params=dict(measured_yaw_at_max_diff_deg_s=self.config.measured_yaw_at_max_diff_deg_s,
                                      max_yaw_rate_deg_s=self.config.max_yaw_rate_deg_s),
                      onsite_limits=C.ONSITE_LIMITS, config_digest=self._config_digest,
                      source_digest=self.source_digest, counts=dict(self.counts),
                      stamper=self.stamper.summary(), log_dropped=self.log.dropped,
                      stale_events=s.get('stale_events'), pauses=s.get('pauses'),
                      row_motor=bool(self.motor_allowed and self.config.motor_output_enabled),
                      calib_allowed=bool(self.motor_allowed and self.config.calib_motor_output_enabled),
                      calib=s.get('calib'), calib_options=self._calib_plan_options())
        status['turn_observe_available'] = False    # no turn observation in this package (U-turn is out of scope)
        status['turn_output_allowed'] = False       # installed integration is INFO_ONLY, irrespective of C6
        status['controller_mode'] = self.config.mode
        if self.config.mode == C.SINGLE_ROW_MODE:
            # operator page single-row panel: telemetry of this tick (or the last run) + the estimator-frame canvas
            status['single_row'] = sr
            status['single_row_geometry'] = getattr(self.runner, 'single_row_geometry', lambda: None)()
            status['single_row_settings'] = dict(
                target_speed_m_s=self.config.single_row_target_speed_m_s,
                approach_speed_m_s=self.config.single_row_approach_speed_m_s,
                base_range_pct=[self.config.single_row_base_min_pct, self.config.single_row_base_max_pct],
                end_stop_offset_m=self.config.single_row_end_stop_offset_m,
                max_row_length_m=self.config.single_row_max_row_length_m,
                localisation=C.SINGLE_ROW_LOCALISATION)
        if self.steering:
            status['vehicle_profile'] = self.config.vehicle_profile
            status['steering'] = s.get('steering')
            status['steer_limits_mrad'] = [self.config.steer_limit_left_mrad, self.config.steer_limit_right_mrad]
        with self._status_lock:
            self._status = _clean(status)

    def _run_end_event(self, d):
        """review_correctness_day2 R6: one 'run_end' event per run with everything needed to read the end from the log
        alone (end_note of a terminal stop kept through a pause, counters, backstop)."""
        s = d.summary
        run = s.get('run_id')
        if d.state != ENDED or run is None or run == self._run_end_logged:
            return
        self._run_end_logged = run
        self._event('run_end', run_id=run, run_kind=s.get('run_kind'), end_reason=d.reason, end_note=s.get('end_note'),
                    pauses=s.get('pauses'), resumes=s.get('resumes'), stale_events=s.get('stale_events'),
                    stall_guard_pauses=s.get('stall_guard_pauses'), controllers_built=s.get('controllers_built'),
                    corridor_lost_pauses=s.get('corridor_lost_pauses'), inner_limited_ticks=s.get('inner_limited_ticks'),
                    no_motion_drive_s=s.get('no_motion_drive_s'), backstop=s.get('backstop'),
                    distance_m=s.get('distance_m'), guard_violation=s.get('guard_violation'))

    def _calib_plan_options(self):
        if not (self.motor_allowed and self.config.calib_motor_output_enabled):
            return None                         # no calibration card in SHADOW or without the config key
        if self._calib_options[0] != self._config_digest:
            try:
                options = CR.plan_options(self.config)
            except Exception as exc:          # display only: never break the status
                options = dict(error=repr(exc)[:200])
            self._calib_options = (self._config_digest, options)
        return self._calib_options[1]

    def _calib_events(self):
        """JSONL events of a calibration run: one 'calib_step' per finished step, one 'calib_result' at its end."""
        calib = self.runner.calib
        if calib is None:
            return
        run_id, logged, done = self._calib_logged
        if run_id != self.runner.run_id:
            run_id, logged, done = self.runner.run_id, 0, False
        for result in calib.results[logged:]:
            self._event('calib_step', run_id=run_id, kind=calib.plan.kind, result=result)
        logged = len(calib.results)
        if calib.end_reason is not None and not done:
            self._event('calib_result', run_id=run_id, kind=calib.plan.kind, end_reason=calib.end_reason,
                        results=calib.results, outcome=calib.outcome, plan=calib.plan.to_dict(),
                        config_digest=self._config_digest)
            done = True
        self._calib_logged = (run_id, logged, done)
        stop = calib.stop
        if stop is not None and stop.get('done') and self._calib_stop_logged != run_id:
            self._calib_stop_logged = run_id
            self._event('calib_stop', run_id=run_id, kind=calib.plan.kind, base_pct=calib.plan.base_pct,
                        terminal_stop_wait_s=(max(self.config.terminal_stop_wait_s, CR.STOP_TEST_WAIT_S)
                                              if calib.end_reason == 'calib_stop_test'
                                              else self.config.terminal_stop_wait_s),
                        emergency=str(calib.end_reason).startswith(EMERGENCY_PREFIXES), stop=stop)

    # --- shutdown --------------------------------------------------------------------------------------------
    def shutdown(self, why='shutdown'):
        """Called by the main thread after the loop stopped: op stop if a lease is held, end the run."""
        now = self.clock()
        if self.runner.state in (PAUSED, ACTIVE):
            self.runner.end_run(now, why)
        client = self.bridge
        if client is not None and client.state == ARMED:
            with self._send_lock:
                client.stop('row_executor_' + why, latch=True)
        self._event('shutdown', why=why, counts=self.counts, runner=self.runner.status(),
                    log_dropped=self.log.dropped, ticks_suppressed=self.log.ticks_suppressed)


# --- threads ---------------------------------------------------------------------------------------------
def run_loop(loop, stop_event, hz=20., max_exceptions=5, exception_window_s=60., shadow_trace_limit=10,
             shadow_summary_s=10., shadow_reset_after=40, shadow_max_resets=5):
    """Absolute-schedule 20 Hz loop on time.monotonic(); a late iteration skips the missed ticks.

    An exception inside a step is a bug.
      MOTOR  (loop.motor_allowed true, or unknown): logged, the emergency STOP path runs (op stop at once, then
             LATCHED and the run ends) and the loop keeps serving the page and logging. More than max_exceptions
             within exception_window_s stop the process (the bridge input TTL and the MCU watchdog stop the motors
             anyway). Unchanged from the first integration.
      SHADOW never stops the process (no motor path exists; the JSONL record is the product): the run ends
             ('loop_exception'), every exception is counted (status/page); the first shadow_trace_limit traces
             are logged, later ones as one summary event per shadow_summary_s. After shadow_reset_after
             consecutive failing steps the Runner is rebuilt (at most shadow_max_resets times; after that the
             loop keeps counting and logging).
    """
    import traceback
    motor = bool(getattr(loop, 'motor_allowed', True))
    period = 1. / hz
    next_t = time.monotonic()
    recent = collections.deque()
    total = consecutive = resets = 0
    last_summary = None
    while not stop_event.is_set():
        try:
            loop.step()
            consecutive = 0
        except Exception as exc:
            now = time.monotonic()
            total += 1
            consecutive += 1
            note = getattr(loop, 'note_loop_exception', None)
            if note is not None:
                note(repr(exc))
            if motor:
                recent.append(now)
                while recent and now - recent[0] > exception_window_s:
                    recent.popleft()
                loop._event('loop_exception', error=repr(exc), trace=traceback.format_exc()[-2000:])
                loop.emergency_stop('loop_exception')
                if len(recent) > max_exceptions:
                    raise
            else:
                if total <= shadow_trace_limit:
                    loop._event('loop_exception', error=repr(exc), trace=traceback.format_exc()[-2000:],
                                count=total, mode='SHADOW')
                elif last_summary is None or now - last_summary >= shadow_summary_s:
                    loop._event('loop_exception_summary', count=total, consecutive=consecutive, last_error=repr(exc))
                    last_summary = now
                loop.request_end('loop_exception')
                if consecutive >= shadow_reset_after and resets < shadow_max_resets:
                    resets += 1
                    consecutive = 0
                    try:
                        loop.reset_runner('loop_exceptions')
                    except Exception as reset_exc:      # keep logging even if the rebuild fails
                        loop._event('runner_reset_failed', error=repr(reset_exc))
        next_t += period
        now = time.monotonic()
        if next_t <= now:                          # late: skip the missed ticks, keep the phase
            missed = int((now - next_t) // period) + 1
            next_t += missed * period
            loop.counts['missed_ticks'] = loop.counts.get('missed_ticks', 0) + missed
        stop_event.wait(max(0., next_t - time.monotonic()))


def run_watchdog(loop, stop_event, period=.05):
    while not stop_event.wait(period):
        try:
            loop.watchdog_check()
        except Exception as exc:          # the watchdog must outlive any error
            loop._event('watchdog_error', error=repr(exc))


class RosInput:
    """rclpy subscriptions -> ExecutorLoop input buffers (receipt = time.monotonic() at callback entry)."""

    def __init__(self, loop, scan_topic='/scan', imu_topic='/imu/data_raw', human_topic='/rover/control/requested'):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Imu, LaserScan
        from std_msgs.msg import String
        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node('ubts_row_executor')

        def sensor_qos(depth):
            return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                              reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        mono = time.monotonic
        self.node.create_subscription(LaserScan, scan_topic, lambda m: loop.on_scan_msg(mono(), m), sensor_qos(2))
        self.node.create_subscription(Imu, imu_topic, lambda m: loop.on_imu_msg(mono(), m), sensor_qos(50))
        if human_topic:
            self.node.create_subscription(String, human_topic, lambda m: loop.on_human_msg(mono(), m.data), 5)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, name='row-executor-ros', daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:      # shutdown races end the spin
            pass

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=1.)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        finally:
            if self.rclpy.ok():
                self.rclpy.shutdown()
        self.thread.join(timeout=2.)


def write_token(path, token):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(token + '\n')


def parse_args(argv=None):
    p = argparse.ArgumentParser(description='UBTS row executor (SHADOW by default)')
    p.add_argument('--config', required=True)
    p.add_argument('--profile', choices=C.PROFILES, default='field')
    p.add_argument('--motor-output', action='store_true',
                   help='allow motor output (also needs motor_output_enabled in the config and an operator Arm)')
    p.add_argument('--shadow-auto', action='store_true', help='SHADOW only: emulated hold + automatic restarts')
    p.add_argument('--log-dir', default='/var/log/row_executor')
    p.add_argument('--max-log-mb', type=float, default=2000.)
    p.add_argument('--scan-topic', default='/scan')
    p.add_argument('--imu-topic', default='/imu/data_raw')
    p.add_argument('--human-topic', default='/rover/control/requested')
    p.add_argument('--no-http', action='store_true', help='SHADOW only: run without the operator page')
    p.add_argument('--skip-source-check', action='store_true', help='development only; refused with --motor-output')
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.motor_output and (args.skip_source_check or args.no_http or args.shadow_auto):
        print('refused: --motor-output cannot be combined with --skip-source-check/--no-http/--shadow-auto',
              file=sys.stderr)
        return 2
    try:
        digest = None if args.skip_source_check else verify_sources()
        verify_vendor()
        cfg = C.load_config(args.config, expected_profile=args.profile)
    except (RuntimeError, OSError, C.ConfigError, ImportError) as exc:
        print('startup refused: %s' % exc, file=sys.stderr)
        return 2
    if (cfg.motor_output_enabled or cfg.calib_motor_output_enabled) and not args.motor_output:
        print('note: config enables motor output but --motor-output is absent: running SHADOW', file=sys.stderr)
    try:
        log = JsonlLog(args.log_dir, max_total_mb=args.max_log_mb)
    except OSError as exc:
        print('startup refused: log directory %s: %s' % (args.log_dir, exc), file=sys.stderr)
        return 2
    try:
        loop = ExecutorLoop(cfg, motor_output_flag=args.motor_output, shadow_auto=args.shadow_auto, log=log,
                            source_digest=digest)
    except ValueError as exc:
        print('startup refused: %s' % exc, file=sys.stderr)
        log.close()
        return 2
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    server = None
    token = secrets.token_urlsafe(18)
    if not args.no_http:
        from .operator_http import make_server
        try:
            server = make_server(loop, cfg.bind_host, cfg.bind_port, token)
        except OSError as exc:
            print('operator page bind %s:%d failed: %s' % (cfg.bind_host, cfg.bind_port, exc), file=sys.stderr)
            log.close()
            return 3
        write_token(os.path.join(args.log_dir, 'operator_token.txt'), token)
        url = 'http://%s:%d/?t=%s' % (cfg.bind_host, cfg.bind_port, token)
        print('operator page: ' + url, flush=True)
        loop._event('operator_page', url_host='%s:%d' % (cfg.bind_host, cfg.bind_port))
        threading.Thread(target=server.serve_forever, name='row-executor-http', daemon=True).start()
    ros = None
    try:
        try:
            ros = RosInput(loop, args.scan_topic, args.imu_topic, args.human_topic or None)
        except ImportError as exc:
            print('rclpy/sensor_msgs not importable (source the ROS setup first): %s' % exc, file=sys.stderr)
            return 4
        watchdog = threading.Thread(target=run_watchdog, args=(loop, stop), name='row-executor-watchdog', daemon=True)
        watchdog.start()
        print('row executor running: mode=%s config=%s digest=%s' % (
            'MOTOR' if loop.motor_allowed else 'SHADOW', args.config, cfg.digest()[:12]), flush=True)
        if not loop.motor_allowed:
            print('SHADOW: the executor does not control the vehicle; the page STOP only ends the shadow run. '
                  'Vehicle stop = dashboard STOP / physical E-stop.', flush=True)
        run_loop(loop, stop, cfg.control_hz)
    finally:
        loop.shutdown('shutdown')
        if server is not None:
            server.shutdown()
            server.server_close()
        if ros is not None:
            ros.close()
        log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
