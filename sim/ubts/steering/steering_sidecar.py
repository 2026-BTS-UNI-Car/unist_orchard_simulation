"""Steering sidecar recording (design 4.5, 8.3 P6b, SIM_ACCEPTANCE A23; docs/steering/STEER_RECORDING_KO.md).

The pinned data recorder (dashboard/scripts/data_recorder_service.py, dashboard_server/motor_pwm.py) cannot record
the steering state: its topic list is fixed and motor_pwm.py keeps the differential fields only (design 4.5). This
separate process records, for every recording of the pinned recorder, a steering timeline next to it:

  <archive_dir>/<recording_id>/steering/        one folder per sidecar segment ('steering', then 'steering-2' ... if
      steering.jsonl                            the sidecar restarted during the recording)
      steering_descriptor.json                  frames: schema orchard.steering.timeline.v1, one per message (20 Hz)
      steering_descriptor.sha256                descriptor (canonical JSON; content_sha256 inside, file SHA here)
      host_profile.json                         byte copy of the host profile the bridge loaded (calibration table)
      bag/                                      separate 'ros2 bag record' of /rover/steering/state (steering_bag_record)
      bag.log, bag.pid                          the bag process output and its pid (orphan recovery)

Where: inside the recording folder, not '<archive_dir>/_steering/<id>/' (design default). The pinned recorder and the
dashboard list every folder of the archive dir as a recording (data_recorder_service.py list_recordings, dashboard
telemetry.recordings), so '_steering' would show as a recording; the finaliser and the media job touch named files
only (the decision and its file/line evidence: STEER_RECORDING_KO.md 2). The sidecar never creates the recording
folder (the recorder creates it with exist_ok=False) and never writes inside 'rosbag2/' or 'mapping_source/' (the
media job hashes those).

Recording-ID handshake (no change to the pinned recorder): poll the recorder's HTTP GET /status (5 Hz). A recording
counts as started when the status says state 'recording' with recording_id R AND <archive_dir>/R/metadata.json says
id R, state 'recording' and the same started_at. It counts as stopped at the first status that is not 'recording'
with R ('finalizing' = the operator's stop; 'idle' = finished or a restarted recorder; another id; the recorder
unreachable for --lost-after-s). The descriptor waits until the recorder has finished R (metadata state 'archived',
or the recorder works on something else) or --finalize-wait-s, so it can record the recorder's end state.

Frames (FRAME_FIELDS, exactly the design 4.5 list): the message fields pass an allow list with type and range checks
like motor_pwm.sanitize_status (anything else is dropped; a bad value becomes null and is counted). Time base: t_s =
archive receive wall time - recording start (metadata started_at), elapsed_monotonic_s = monotonic time since the
same instant, clock_offset_s = their difference; a sample whose wall time goes backwards is skipped and counted (the
motor_pwm rule), so t_s never decreases in the file. stale = the bridge's stale flag, or no MCU status line in the
message, or the archive receive time minus the bridge receive time unknown or above stale_after_s (the host profile
status gap). angle_mrad is computed from pos_permille with the calibration table copied into the descriptor (the host
profile file the bridge loaded, checked by its SHA-256 in every message), and only when the bridge marks the
calibration valid (its own angle_mrad not null). The messages received in the last --buffer-s before the start was
seen are written too (when they are not older than the recording start).

Crash safety (fsync policy): every frame is written and flushed; os.fsync of the frames file at most every
--fsync-interval-s (default 1.0 s, monotonic clock) and at close; the folder is fsynced after files are created; the
descriptor and its SHA file are written to a temporary file, fsynced, renamed, and the folder is fsynced. A process
crash loses no flushed frame; a power cut loses at most about one fsync interval. A segment left without a descriptor
(crash) gets a recovered descriptor (end_reason 'sidecar_restarted_unclean') when the next sidecar opens a segment of
the same recording, or with --recover. The frames file of a crashed segment is never rewritten; a broken last line is
counted as malformed.

  sanitize_state(payload, received_at, stale_after_s, calibration)   the allow list (pure)
  HostCalibration                                                     the calibration table of the host profile JSON
  SteeringSegment                                                     one segment: frames, stats, descriptor
  SidecarController                                                   handshake state machine, segments, bags (pure)
  check_recording(recording_dir)                                      A23 checks of the segments of one recording
  recover_segment(segment_dir)                                        descriptor for a segment left by a crash
  main()                                                              the ROS 2 node (rclpy; recorder container)

ROS: node ubts_steering_sidecar subscribes /rover/steering/state (std_msgs/String JSON, schema ubts.steer_state.v1,
RELIABLE, depth 100). No truth topic, no control topic (A13). The bag process is a second node (ubts_steering_bag).
Placeholder-based SIM results only (자리값 기반 SIM 결과), never field validation.
CLI: python3 -B -m steering.steering_sidecar [--archive-dir D] [--recorder-url U] [--host-profile F] [--status-file F]
     python3 -B -m steering.steering_sidecar --check <recording folder>      (A23 checks, JSON)
     python3 -B -m steering.steering_sidecar --recover <segment folder>      (descriptor for a crashed segment)
"""
from __future__ import annotations

import argparse
import collections
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import threading
import time
import urllib.request

SCHEMA = 'orchard.steering.timeline.v1'
MESSAGE_SCHEMA = 'ubts.steer_state.v1'
HOST_PROFILE_SCHEMA = 'ubts.steer_host_profile.v1'
STATUS_SCHEMA = 'ubts.steering_sidecar_status.v1'
CHECK_SCHEMA = 'ubts.steering_record_check.v1'
TOPIC = '/rover/steering/state'
NODE_NAME = 'ubts_steering_sidecar'
SEGMENT_DIR = 'steering'
MAX_SEGMENTS = 9
FRAMES_FILE = 'steering.jsonl'
DESCRIPTOR_FILE = 'steering_descriptor.json'
DESCRIPTOR_SHA_FILE = 'steering_descriptor.sha256'
BAG_DIR = 'bag'
BAG_LOG = 'bag.log'
BAG_PID = 'bag.pid'
HOST_PROFILE_COPY = 'host_profile.json'      # byte copy of the host profile file the bridge loaded (same SHA)
LOCK_FILE = '.steering_sidecar.lock'
DEFAULT_ARCHIVE_DIR = '/runtime/recordings'
DEFAULT_RECORDER_URL = 'http://127.0.0.1:8790'
DEFAULT_HOST_PROFILE = '/runtime/steer/host_profile.json'
DEFAULT_STATUS_FILE = '/runtime/steering_sidecar_status.json'
DEFAULT_SAMPLE_HZ = 20                       # design 4.5; the host profile link.status_hz replaces it when present
DEFAULT_STALE_AFTER_S = 0.15                 # the placeholder host status gap; the host profile value replaces it
MAX_MESSAGE_CHARS = 65536                    # motor_pwm.MotorTimeline.observe uses the same bound
POLL_S = 0.2
POLL_TIMEOUT_S = 0.5
BUFFER_S = 3.0
LOST_AFTER_S = 5.0
OPEN_TIMEOUT_S = 5.0
FINALIZE_WAIT_S = 30.0                       # the recorder's own stop can take 8 + 2 + 2 s (rosbag SIGINT, TERM, KILL)
SHUTDOWN_S = 5.5                             # inside the supervisor's 6 s grace time
FSYNC_INTERVAL_S = 1.0
PREVIEW_MAX_FRAMES = 600
PREVIEW_INTERVAL_S = 0.5
AGE_SAMPLES_MAX = 200000
RECORDING_ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z')
BOOT_ID = re.compile(r'[0-9A-F]{8}\Z')
HEX64 = re.compile(r'[0-9a-f]{64}\Z')
NAME = re.compile(r'[A-Za-z0-9_]{1,48}\Z')

FRAME_FIELDS = ('t_s', 'elapsed_monotonic_s', 'clock_offset_s', 'source_kind', 'received_at', 'mcu_ms', 'mcu_seq',
                'boot_id', 'reset_reason', 'cal_id', 'state', 'stale', 'raw_adc', 'pos_permille', 'angle_mrad',
                'target_mrad', 'cmd_permille', 'steer_pwm', 'drive_l', 'drive_r', 'drive_en', 'steer_en', 'applied_q',
                'ttl_remaining_ms', 'stop_mode', 'faults')
SOURCE_KINDS = ('mcu_status', 'simulation')
MCU_STATES = ('DISARMED', 'ARMED', 'BENCH')
STOP_MODES = ('release', 'brake', 'coast')
# esp_motor_v3.RESET_NAMES, FAULT_BIT_NAMES and steer_bridge_core.LATCH_NAMES (a test compares them with the host
# modules); mcu_ttl_expired is the host TTL latch of DECISIONS R18 decision 1. Unknown names are dropped and counted.
RESET_REASONS = ('power_on', 'hw_wdt', 'exception', 'soft_wdt', 'soft_restart', 'deep_sleep', 'ext_sys', 'unknown')
MCU_FAULTS = ('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'SOFTLIMIT', 'STALL',
              'TRACK', 'THERMAL', 'TTL', 'WATCHDOG', 'CRC', 'BOOT')
HOST_LATCHES = ('steering_status_stale', 'mcu_reset_detected', 'calibration_mismatch', 'hardware_profile_mismatch',
                'sim_identity_mismatch', 'steering_not_following', 'steering_ineffective', 'status_seq_gap',
                'stop_unconfirmed', 'mcu_disarmed', 'mcu_state_mismatch', 'c_rejected', 'hardware_fault',
                'arm_rejected', 'arm_unconfirmed', 'internal_error', 'mcu_ttl_expired')
FAULT_NAMES = MCU_FAULTS + HOST_LATCHES
MAX_FAULTS = 32
# (low, high) of the integer fields: the V3 status line ranges (esp_motor_v3._parse_status) and the host limits.
INT_RANGES = dict(mcu_ms=(0, 4294967295), mcu_seq=(0, 65535), cal_id=(1, 65535), raw_adc=(0, 1023),
                  pos_permille=(-9999, 9999), target_mrad=(-1570, 1570), cmd_permille=(-9999, 9999),
                  steer_pwm=(-255, 255), drive_l=(-255, 255), drive_r=(-255, 255), applied_q=(0, 65535),
                  ttl_remaining_ms=(0, 1000))
MEANING = ('가변저항은 링크 각의 측정값이며 타이어 접지각이 아니다', 'PWM은 엔코더가 아니다')
CONVERSION = ('angle_mrad = round_half_away(pos_permille * mark_left_mrad / 1000) for pos_permille >= 0, '
              '-round_half_away(-pos_permille * mark_right_mrad / 1000) below 0 (steer_bridge_core.HostProfile.'
              'mrad_from_permille); bicycle-equivalent angle of the potentiometer reading, not the tyre angle')
FSYNC_POLICY = ('each frame is written and flushed; os.fsync of the frames file at most every fsync_interval_s of '
                'the monotonic clock and at close; the folder is fsynced after files are created; the descriptor and '
                'its SHA file go to a temporary file, are fsynced and renamed; the frames file is never rewritten')
STALE_RULE = ('stale = the bridge stale flag (no MCU status line or the line older than the host status gap) or no '
              'MCU status line in the message or (archive receive wall time - bridge receive wall time) unknown, '
              'negative or above stale_after_s')
CLOCK_STEP_POLICY = ('wall_clock_matches_sensor_playhead; backward_samples_skipped_until_caught_up; '
                     'steady_elapsed_retained (dashboard_server/motor_pwm.py rule)')


class SidecarError(RuntimeError):
    pass


# --- small helpers ---------------------------------------------------------------------------------------------------
def _number(value):
    if type(value) not in (int, float):
        return None
    try:
        return float(value) if math.isfinite(value) else None
    except OverflowError:
        return None


def round_half_away(x):
    """Round half away from zero to int (steer_bridge_core.round_half_away)."""
    return int(math.floor(abs(x) + 0.5)) * (1 if x >= 0 else -1)


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False)


def content_sha256(descriptor):
    """SHA-256 of the canonical JSON of the descriptor without its content_sha256 field (key order free)."""
    body = {k: v for k, v in descriptor.items() if k != 'content_sha256'}
    return hashlib.sha256(canonical_json(body).encode('utf-8')).hexdigest()


def descriptor_bytes(descriptor):
    """File bytes of a descriptor: canonical JSON (sorted keys, compact, UTF-8) and one LF."""
    return (canonical_json(descriptor) + '\n').encode('utf-8')


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def parse_started_at(text):
    """Unix seconds of the recorder's started_at (datetime.isoformat with an offset), or None."""
    if not isinstance(text, str) or len(text) > 64:
        return None
    try:
        value = datetime.fromisoformat(text)
    except ValueError:
        return None
    if value.tzinfo is None:
        return None
    return value.timestamp()


def fsync_dir(path):
    """fsync a folder (POSIX; a no-op elsewhere)."""
    if os.name != 'posix':
        return
    fd = os.open(str(path), os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def write_atomic(path, data, fsync=True):
    """Write bytes to path through a temporary file in the same folder (fsync, rename, folder fsync)."""
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    with open(tmp, 'wb') as handle:
        handle.write(data)
        handle.flush()
        if fsync:
            os.fsync(handle.fileno())
    os.replace(tmp, path)
    if fsync:
        fsync_dir(path.parent)


def read_json(path, limit=1 << 20):
    try:
        path = Path(path)
        if path.stat().st_size > limit:
            return {}
        value = json.loads(path.read_bytes().decode('utf-8'))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError, UnicodeDecodeError):
        return {}


def _module_sha256():
    out = {}
    for name in ('steering_sidecar.py', 'steering_bag_record.py'):
        try:
            out[name] = file_sha256(Path(__file__).resolve().with_name(name))
        except OSError:
            out[name] = None
    return out


def _percentiles(values):
    if not values:
        return dict(n=0, p50=None, p95=None, max=None)
    data = sorted(values)

    def pick(q):
        return round(data[min(len(data) - 1, int(math.ceil(q * len(data))) - 1)], 6)
    return dict(n=len(data), p50=pick(0.5), p95=pick(0.95), max=round(data[-1], 6))


# --- calibration table -----------------------------------------------------------------------------------------------
class HostCalibration:
    """The parts of a host profile JSON (schema ubts.steer_host_profile.v1; sim/ubts/steering/host_profile_export.py
    writes it, the bridge loads it with steer_bridge_core.HostProfile) that the descriptor copies: the calibration
    table, the steer limit, the source profile id and SHA, the status rate and the status gap. sha256 is the SHA-256
    of the file bytes, the value the bridge puts into every /rover/steering/state message (host_profile_sha256)."""

    def __init__(self, raw, origin='<memory>'):
        raw = bytes(raw)
        try:
            data = json.loads(raw.decode('utf-8'))
        except (UnicodeDecodeError, ValueError) as exc:
            raise SidecarError('host profile is not UTF-8 JSON: %s' % exc) from None
        if not isinstance(data, dict) or data.get('schema') != HOST_PROFILE_SCHEMA:
            raise SidecarError('host profile schema %r (need %r)' % (
                data.get('schema') if isinstance(data, dict) else None, HOST_PROFILE_SCHEMA))
        cal, link, faults = data.get('calibration'), data.get('link'), data.get('host_faults')
        if not all(isinstance(x, dict) for x in (cal, link, faults)):
            raise SidecarError('host profile needs calibration, link and host_faults objects')
        marks = [_number(cal.get(k)) for k in ('mark_left_mrad', 'mark_right_mrad')]
        if any(m is None or m <= 0 for m in marks):
            raise SidecarError('host profile calibration marks must be positive numbers')
        hz, gap = link.get('status_hz'), _number(faults.get('status_gap_ms'))
        if type(hz) is not int or not 1 <= hz <= 1000 or gap is None or gap <= 0:
            raise SidecarError('host profile link.status_hz / host_faults.status_gap_ms malformed')
        self.data = data
        self.raw = raw
        self.sha256 = hashlib.sha256(raw).hexdigest()
        self.origin = str(origin)
        self.mark_left_mrad, self.mark_right_mrad = marks
        self.cal_id = cal.get('cal_id')
        self.table = json.loads(json.dumps(cal))
        self.steer_limit = json.loads(json.dumps(data.get('steer_limit')))
        self.profile = json.loads(json.dumps(data.get('source_profile')))
        self.hw_profile = data.get('hw_profile')
        self.status_hz = hz
        self.stale_after_s = gap / 1000.0

    @classmethod
    def load(cls, path):
        with open(path, 'rb') as handle:
            return cls(handle.read(), origin=path)

    def mrad_from_permille(self, permille):
        """steer_bridge_core.HostProfile.mrad_from_permille (a test compares them over -9999..9999)."""
        if permille >= 0:
            return round_half_away(permille * self.mark_left_mrad / 1000.0)
        return -round_half_away(-permille * self.mark_right_mrad / 1000.0)

    def describe(self):
        return dict(cal_id=self.cal_id, table=self.table, steer_limit=self.steer_limit, conversion=CONVERSION,
                    host_profile_sha256=self.sha256, host_profile_file=Path(self.origin).name,
                    host_profile_copy=HOST_PROFILE_COPY, hw_profile=self.hw_profile, status_hz=self.status_hz)


# --- message allow list ----------------------------------------------------------------------------------------------
def sanitize_state(payload, received_at, stale_after_s, calibration=None):
    """(sample, info) for one /rover/steering/state payload, or (None, None) when it is not a steering state.

    sample: the FRAME_FIELDS without the three time fields, every key present; a value outside its type or range is
    None (named in info['invalid']). Unknown keys are dropped (allow list, like motor_pwm.sanitize_status).
    info: stale reasons, the receive age, unknown fault names, the angle check, the message's host profile SHA."""
    if not isinstance(payload, dict) or payload.get('schema') != MESSAGE_SCHEMA:
        return None, None
    invalid = []

    def integer(key):
        value = payload.get(key)
        low, high = INT_RANGES[key]
        if value is None:
            return None
        if type(value) is int and low <= value <= high:
            return value
        invalid.append(key)
        return None

    def boolean(key):
        value = payload.get(key)
        if value is None or type(value) is bool:
            return value
        invalid.append(key)
        return None

    def choice(key, allowed):
        value = payload.get(key)
        if value is None:
            return None
        if isinstance(value, str) and value in allowed:
            return value
        invalid.append(key)
        return None
    sample = dict(source_kind=choice('source_kind', SOURCE_KINDS), received_at=None, mcu_ms=integer('mcu_ms'),
                  mcu_seq=integer('mcu_seq'))
    boot = payload.get('boot_id')
    if boot is not None and not (isinstance(boot, str) and BOOT_ID.match(boot) and boot != '00000000'):
        invalid.append('boot_id')
        boot = None
    sample.update(boot_id=boot, reset_reason=choice('reset_reason', RESET_REASONS), cal_id=integer('cal_id'),
                  state=choice('state', MCU_STATES), stale=None, raw_adc=integer('raw_adc'),
                  pos_permille=integer('pos_permille'), angle_mrad=None, target_mrad=integer('target_mrad'),
                  cmd_permille=integer('cmd_permille'), steer_pwm=integer('steer_pwm'), drive_l=integer('drive_l'),
                  drive_r=integer('drive_r'), drive_en=boolean('drive_en'), steer_en=boolean('steer_en'),
                  applied_q=integer('applied_q'), ttl_remaining_ms=integer('ttl_remaining_ms'),
                  stop_mode=choice('stop_mode', STOP_MODES), faults=None)
    unknown = []
    faults = payload.get('faults')
    if isinstance(faults, list):
        kept = []
        for name in faults:
            if isinstance(name, str) and name in FAULT_NAMES:
                if len(kept) < MAX_FAULTS:
                    kept.append(name)
            else:
                unknown.append(name if isinstance(name, str) and NAME.match(name) else '<invalid>')
        sample['faults'] = kept
    elif faults is not None:
        invalid.append('faults')
    # stale (STALE_RULE)
    reasons = []
    flag = payload.get('stale')
    if flag is not False:
        if flag is not True:
            invalid.append('stale')
        reasons.append('bridge_flag')
    if sample['mcu_seq'] is None:
        reasons.append('no_status_line')
    source = _number(payload.get('received_at'))
    age = None if source is None else received_at - source
    if age is None or age < 0:
        reasons.append('age_unknown')
        age = None
    elif age > stale_after_s:
        reasons.append('age_over')
    sample['stale'] = bool(reasons)
    # angle from the recording's calibration table (descriptor copy)
    message_sha = payload.get('host_profile_sha256')
    message_sha = message_sha if isinstance(message_sha, str) and HEX64.match(message_sha) else None
    bridge_angle = payload.get('angle_mrad')
    if calibration is None:
        angle_check = 'no_table'
    elif message_sha != calibration.sha256:
        angle_check = 'table_mismatch'
    elif bridge_angle is None:
        angle_check = 'bridge_marks_invalid'
    elif sample['pos_permille'] is None:
        angle_check = 'no_position'
    else:
        sample['angle_mrad'] = calibration.mrad_from_permille(sample['pos_permille'])
        angle_check = 'ok' if type(bridge_angle) is int and bridge_angle == sample['angle_mrad'] else 'mismatch'
    info = dict(invalid=invalid, unknown_faults=unknown, stale_reasons=reasons, age_s=age, angle_check=angle_check,
                host_profile_sha256=message_sha)
    return sample, info


# --- per-segment counters --------------------------------------------------------------------------------------------
class SeqCounter:
    """Loss counting over the MCU status line number n (16 bit, wraps) per boot id. A repeat of the same (boot, n) is
    the bridge sending the last state again (stale); a forward step d > 1 lost d - 1 lines; a backward step (more than
    half the range) is a regression and starts counting again."""

    def __init__(self):
        self.boot = None
        self.last_seq = self.last_ms = None
        self.unique = self.repeated = self.lost = self.regressions = self.boot_changes = 0
        self.seq_advance = 0
        self.mcu_span_ms = 0
        self.boots = []

    def add(self, boot_id, seq, ms):
        if seq is None:
            return 'no_line'
        if self.last_seq is None or boot_id != self.boot:
            if self.last_seq is not None:
                self.boot_changes += 1
            self.boot, self.last_seq, self.last_ms = boot_id, seq, ms
            self.unique += 1
            if boot_id not in self.boots and len(self.boots) < 8:
                self.boots.append(boot_id)
            return 'first'
        step = (seq - self.last_seq) % 65536
        if step == 0:
            self.repeated += 1
            return 'repeat'
        self.unique += 1
        if step >= 32768:
            self.regressions += 1
            self.last_seq, self.last_ms = seq, ms
            return 'regression'
        self.lost += step - 1
        self.seq_advance += step
        if ms is not None and self.last_ms is not None:
            self.mcu_span_ms += (ms - self.last_ms) % 4294967296
        self.last_seq, self.last_ms = seq, ms
        return 'next' if step == 1 else 'gap'

    def summary(self):
        total = self.unique + self.lost
        rate = self.seq_advance / (self.mcu_span_ms / 1000.0) if self.mcu_span_ms > 0 else None
        return dict(unique_status_lines=self.unique, repeated_status_lines=self.repeated,
                    lost_status_lines=self.lost, loss_rate=round(self.lost / total, 6) if total else None,
                    seq_regressions=self.regressions, boot_changes=self.boot_changes, boot_ids=list(self.boots),
                    mcu_rate_hz=None if rate is None else round(rate, 4))


class Preview:
    """Bounded decimated preview: one frame per interval_s of t_s; when it is full, every second frame is dropped
    and the interval doubles (covers the whole recording; the frames file is authoritative)."""

    def __init__(self, max_frames=PREVIEW_MAX_FRAMES, interval_s=PREVIEW_INTERVAL_S):
        self.max_frames, self.interval_s = int(max_frames), float(interval_s)
        self.frames = []
        self.next_t = None

    def add(self, frame):
        t = frame['t_s']
        if self.next_t is not None and t < self.next_t:
            return
        self.frames.append(frame)
        if len(self.frames) > self.max_frames:
            self.frames = self.frames[::2]
            self.interval_s *= 2.0
        self.next_t = self.frames[-1]['t_s'] + self.interval_s


# --- one segment -----------------------------------------------------------------------------------------------------
class SteeringSegment:
    """Frames of one recording written by one sidecar process (one folder)."""

    def __init__(self, recording_dir, segment_dir, recording, *, calibration=None, calibration_error=None,
                 clock=time.time, steady=time.monotonic, fsync_interval_s=FSYNC_INTERVAL_S,
                 preview_max_frames=PREVIEW_MAX_FRAMES, fsync=os.fsync, recorder_url=None):
        self.recording_dir = Path(recording_dir)
        self.dir = Path(segment_dir)
        self.recording_id = recording['id']
        self.started_at = recording['started_at']
        self.started_epoch = parse_started_at(self.started_at)
        if self.started_epoch is None:
            raise SidecarError('recording started_at %r is not an ISO time with an offset' % (self.started_at,))
        self.calibration = calibration
        self.calibration_error = calibration_error
        self.stale_after_s = calibration.stale_after_s if calibration else DEFAULT_STALE_AFTER_S
        self.sample_hz = calibration.status_hz if calibration else DEFAULT_SAMPLE_HZ
        self.clock, self.steady, self.fsync = clock, steady, fsync
        self.fsync_interval_s = float(fsync_interval_s)
        self.recorder_url = recorder_url
        self.preview = Preview(preview_max_frames)
        self.seq = SeqCounter()
        self.handle = None
        self.state = 'new'
        self.error = None
        self.frame_count = 0
        self.last_t_s = self.first_t_s = None
        self.last_fsync = None
        self.fsync_calls = 0
        self.bag = None
        self.bag_error = None
        self.handshake = dict(recorder_url=recorder_url, recording_started_at=self.started_at,
                              recording_started_unix_s=self.started_epoch)
        self.counts = collections.Counter()
        self.invalid = collections.Counter()
        self.unknown_faults = collections.Counter()
        self.stale_reasons = collections.Counter()
        self.angle_checks = collections.Counter()
        self.source_kinds = set()
        self.sha_seen = []
        self.cal_ids = []
        self.ages = []
        self.started_steady = None
        self.closed_at = None
        self.end_reason = None
        self.descriptor = None

    # life cycle
    def open(self, open_reason='recorder_status_recording'):
        """Create the folder (inside the existing recording folder; never the recording folder) and the frames file."""
        if not self.recording_dir.is_dir() or self.recording_dir.is_symlink():
            raise SidecarError('recording folder missing: %s' % self.recording_dir)
        self.dir.mkdir(exist_ok=False)
        self.handle = open(self.dir / FRAMES_FILE, 'x', encoding='utf-8', newline='\n')
        if self.calibration is not None:
            write_atomic(self.dir / HOST_PROFILE_COPY, self.calibration.raw)
        fsync_dir(self.dir)
        fsync_dir(self.recording_dir)
        wall, steady = self.clock(), self.steady()
        self.started_steady = steady - (wall - self.started_epoch)
        self.last_fsync = steady
        self.state = 'open'
        self.handshake.update(opened_at=wall, open_delay_s=round(wall - self.started_epoch, 6), open_reason=open_reason,
                              sidecar_pid=os.getpid())

    def observe(self, data, received_at, received_steady):
        """One message. Returns the frame written, or None (not open, rejected, clock regression, write error)."""
        if self.state != 'open':
            return None
        self.counts['messages'] += 1
        if not isinstance(data, str) or len(data) > MAX_MESSAGE_CHARS:
            self.counts['rejected_messages'] += 1
            return None
        try:
            sample, info = sanitize_state(json.loads(data), received_at, self.stale_after_s, self.calibration)
        except (ValueError, RecursionError):
            sample = info = None
        if sample is None:
            self.counts['rejected_messages'] += 1
            return None
        elapsed = received_at - self.started_epoch
        steady_elapsed = received_steady - self.started_steady
        if elapsed < 0 or (self.last_t_s is not None and elapsed < self.last_t_s):
            self.counts['dropped_time_frames'] += 1
            self.error = self.error or 'steering_recording_clock_regressed'
            return None
        sample['received_at'] = received_at
        frame = dict(t_s=round(elapsed, 6), elapsed_monotonic_s=round(steady_elapsed, 6),
                     clock_offset_s=round(elapsed - steady_elapsed, 6))
        for key in FRAME_FIELDS[3:]:
            frame[key] = sample[key]
        try:
            self.handle.write(json.dumps(frame, allow_nan=False, separators=(',', ':')) + '\n')
            self.handle.flush()
            now = self.steady()
            if now - self.last_fsync >= self.fsync_interval_s:
                self.fsync(self.handle.fileno())
                self.fsync_calls += 1
                self.last_fsync = now
        except (OSError, ValueError):
            self.counts['write_errors'] += 1
            self.error = 'steering_sidecar_write_failed'
            return None
        self.frame_count += 1
        self.first_t_s = frame['t_s'] if self.first_t_s is None else self.first_t_s
        self.last_t_s = elapsed
        self._account(frame, info)
        self.preview.add(frame)
        return frame

    def _account(self, frame, info):
        kind = self.seq.add(frame['boot_id'], frame['mcu_seq'], frame['mcu_ms'])
        self.counts['frames_%s' % kind] += 1
        if frame['stale']:
            self.counts['stale_frames'] += 1
        for reason in info['stale_reasons']:
            self.stale_reasons[reason] += 1
        for key in info['invalid']:
            self.invalid[key] += 1
        for name in info['unknown_faults']:
            if name in self.unknown_faults or len(self.unknown_faults) < 16:
                self.unknown_faults[name] += 1
        self.angle_checks[info['angle_check']] += 1
        if frame['source_kind']:
            self.source_kinds.add(frame['source_kind'])
        sha = info['host_profile_sha256']
        if sha not in self.sha_seen and len(self.sha_seen) < 4:
            self.sha_seen.append(sha)
        if frame['cal_id'] not in self.cal_ids and len(self.cal_ids) < 8:
            self.cal_ids.append(frame['cal_id'])
        # receive age (archive receive - bridge receive) of new status lines only: the transport delay; a repeat's
        # age is the age of the old line (stale), not a delay
        if kind in ('first', 'next', 'gap', 'regression') and info['age_s'] is not None and \
                len(self.ages) < AGE_SAMPLES_MAX:
            self.ages.append(info['age_s'])

    def close_frames(self, end_reason):
        """Stop taking frames (fsync, close the file). The descriptor comes later with finalize()."""
        if self.state != 'open':
            return
        self.state = 'closing'
        self.closed_at = self.clock()
        self.end_reason = end_reason
        if self.handle is not None:
            try:
                self.handle.flush()
                self.fsync(self.handle.fileno())
                self.fsync_calls += 1
            except (OSError, ValueError):
                self.error = self.error or 'steering_sidecar_close_failed'
            finally:
                self.handle.close()
                self.handle = None
        self.handshake.update(closed_at=self.closed_at, end_reason=end_reason,
                              stop_observed_t_s=round(self.closed_at - self.started_epoch, 6))

    def stats(self):
        seq = self.seq.summary()
        span = None if self.first_t_s is None or self.last_t_s is None else self.last_t_s - self.first_t_s
        rate = None
        if span and span > 0 and seq['unique_status_lines'] > 1:
            rate = round((seq['unique_status_lines'] - 1) / span, 4)
        return dict(messages=self.counts['messages'], rejected_messages=self.counts['rejected_messages'],
                    frames=self.frame_count, replayed_from_buffer=self.counts['replayed'],
                    frames_without_status_line=self.counts['frames_no_line'], **seq,
                    stale_frames=self.counts['stale_frames'], stale_reasons=dict(sorted(self.stale_reasons.items())),
                    archive_rate_hz=rate, first_t_s=self.first_t_s,
                    last_t_s=None if self.last_t_s is None else round(self.last_t_s, 6),
                    dropped_time_frames=self.counts['dropped_time_frames'], write_errors=self.counts['write_errors'],
                    invalid_values=dict(sorted(self.invalid.items())),
                    unknown_fault_names=dict(sorted(self.unknown_faults.items())),
                    angle_checks=dict(sorted(self.angle_checks.items())), host_profile_sha256_seen=list(self.sha_seen),
                    cal_ids_seen=list(self.cal_ids), receive_age_s=_percentiles(self.ages))

    def build_descriptor(self, bag_result=None, recorder_final=None, provenance=None):
        frames_path = self.dir / FRAMES_FILE
        try:
            frames_sha, frames_bytes = file_sha256(frames_path), frames_path.stat().st_size
        except OSError:
            frames_sha = frames_bytes = None
        kinds = sorted(self.source_kinds)
        source_kind = kinds[0] if len(kinds) == 1 else 'mixed' if kinds else 'unknown'
        calibration = self.calibration.describe() if self.calibration else None
        if calibration is not None:
            seen = [s for s in self.sha_seen]
            calibration['matches_messages'] = (seen == [self.calibration.sha256]) if seen else None
        profile = dict(self.calibration.profile) if self.calibration and isinstance(self.calibration.profile, dict) \
            else None
        handshake = dict(self.handshake)
        handshake['recorder_final'] = recorder_final
        duration = (recorder_final or {}).get('duration_s')
        if isinstance(duration, (int, float)) and not isinstance(duration, bool):
            handshake['frames_after_recorder_duration'] = self._frames_after(duration)
        result_kind = None
        if source_kind == 'simulation':
            result_kind = '자리값 기반 SIM 결과'
        elif profile:
            result_kind = profile.get('result_kind')
        descriptor = dict(
            schema=SCHEMA, enabled=True, topic=TOPIC, message_schema=MESSAGE_SCHEMA, recording_id=self.recording_id,
            segment=self.dir.name, location='<archive_dir>/<recording_id>/%s/' % self.dir.name,
            source_kind=source_kind, sample_hz=self.sample_hz,
            sample_hz_semantics='nominal_mcu_status_line_rate_not_measured_archive_rate',
            sampling=('one frame per received message; the bridge publishes one message per MCU status line and sends '
                      'the last state again with stale true every 0.5 s while no line arrives'),
            time_base='host_receive_time_relative_to_recording_start',
            epoch=dict(source='recording metadata.json started_at (recorder wall clock at its start)',
                       started_at=self.started_at, started_at_unix_s=self.started_epoch,
                       note='the pinned recorder takes its own timeline epoch a few ms after started_at'),
            clock_step_policy=CLOCK_STEP_POLICY, stale_after_s=self.stale_after_s,
            stale_after_source=('host profile host_faults.status_gap_ms (the bridge stale threshold)'
                                if self.calibration else 'default %.3f s (host profile unavailable)'
                                % DEFAULT_STALE_AFTER_S),
            stale_rule=STALE_RULE, calibration=calibration, calibration_error=self.calibration_error,
            profile=profile, meaning=list(MEANING), fields=list(FRAME_FIELDS), frames_file=FRAMES_FILE,
            frame_count=self.frame_count, frames_bytes=frames_bytes, frames_sha256=frames_sha,
            preview_max_frames=self.preview.max_frames, preview_interval_s=self.preview.interval_s,
            preview=list(self.preview.frames), stats=self.stats(), bag=bag_result, bag_error=self.bag_error,
            handshake=handshake,
            writer=dict(fsync_policy=FSYNC_POLICY, fsync_interval_s=self.fsync_interval_s,
                        fsync_calls=self.fsync_calls),
            provenance=provenance or {}, result_kind=result_kind, error=self.error)
        descriptor['content_sha256'] = content_sha256(descriptor)
        return descriptor

    def _frames_after(self, duration_s):
        count = 0
        try:
            with open(self.dir / FRAMES_FILE, encoding='utf-8') as handle:
                for line in handle:
                    try:
                        if json.loads(line)['t_s'] > duration_s:
                            count += 1
                    except (ValueError, KeyError, TypeError):
                        continue
        except OSError:
            return None
        return count

    def finalize(self, bag_result=None, recorder_final=None, provenance=None):
        """Write the descriptor and its SHA file (atomic). Returns the descriptor."""
        if self.state == 'open':
            self.close_frames('finalize_without_close')
        descriptor = self.build_descriptor(bag_result, recorder_final, provenance)
        data = descriptor_bytes(descriptor)
        write_atomic(self.dir / DESCRIPTOR_FILE, data)
        write_atomic(self.dir / DESCRIPTOR_SHA_FILE,
                     ('%s  %s\n' % (hashlib.sha256(data).hexdigest(), DESCRIPTOR_FILE)).encode('ascii'))
        self.state = 'done'
        self.descriptor = descriptor
        return descriptor

    def status(self):
        return dict(recording_id=self.recording_id, segment=self.dir.name, state=self.state, frames=self.frame_count,
                    last_t_s=None if self.last_t_s is None else round(self.last_t_s, 3), error=self.error,
                    bag_pid=None if self.bag is None else self.bag.pid, bag_error=self.bag_error)


# --- recorder status -------------------------------------------------------------------------------------------------
_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def fetch_recorder_status(url, timeout_s=POLL_TIMEOUT_S):
    """GET <url>/status of the pinned recorder (no proxy). Returns the dict; raises on any failure."""
    with _OPENER.open(url.rstrip('/') + '/status', timeout=timeout_s) as response:
        body = response.read(1 << 20)
    data = json.loads(body.decode('utf-8'))
    if not isinstance(data, dict):
        raise ValueError('recorder status is not a JSON object')
    return data


def valid_recording_id(value):
    return isinstance(value, str) and bool(RECORDING_ID.match(value))


def segment_dirs(recording_dir):
    """The steering segment folders of a recording folder ('steering', 'steering-2', ...), sorted by number."""
    def number(path):
        return 1 if path.name == SEGMENT_DIR else int(path.name[len(SEGMENT_DIR) + 1:])
    try:
        found = [p for p in Path(recording_dir).iterdir() if p.is_dir() and not p.is_symlink() and (
            p.name == SEGMENT_DIR or re.match(re.escape(SEGMENT_DIR) + r'-[2-9]\Z', p.name))]
    except OSError:
        return []
    return sorted(found, key=number)


def recorder_final(recording_dir, last_status, recording_id):
    """End state of a recording as the recorder left it: metadata.json state (archived = finalised), finished_at,
    duration_s, and complete = the recorder no longer works on it (archived, or its status shows another id or idle)."""
    meta = read_json(Path(recording_dir) / 'metadata.json')
    state = meta.get('state') if isinstance(meta.get('state'), str) else None
    status = last_status if isinstance(last_status, dict) else {}
    busy = status.get('recording_id') == recording_id and status.get('state') in ('recording', 'finalizing')
    duration = meta.get('duration_s')
    return dict(state=state, finished_at=meta.get('finished_at') if isinstance(meta.get('finished_at'), str) else None,
                duration_s=duration if isinstance(duration, (int, float)) and not isinstance(duration, bool) else None,
                kind=meta.get('kind') if isinstance(meta.get('kind'), str) else None,
                complete=state == 'archived' or (bool(status) and not busy))


# --- controller ------------------------------------------------------------------------------------------------------
class SidecarController:
    """Recording-ID handshake and segment life cycle. Pure: the recorder status, the clocks and the bag process
    factory are injected, so the race and restart cases are tested without ROS or HTTP.

    on_message(data[, received_at, received_steady])  every /rover/steering/state message
    on_status(status)                                 every recorder status poll (a dict, or an exception / None)
    tick()                                            progress of closing segments (bag exit, recorder end state)
    shutdown(reason)                                  close everything (bounded)
    bag_factory(output_dir, log_path) -> object with start(), request_stop(), poll(), result(), pid; None = no bag
    """

    def __init__(self, archive_dir, *, host_profile_path=None, recorder_url=DEFAULT_RECORDER_URL, bag_factory=None,
                 clock=time.time, steady=time.monotonic, sleep=time.sleep, buffer_s=BUFFER_S,
                 lost_after_s=LOST_AFTER_S, open_timeout_s=OPEN_TIMEOUT_S, finalize_wait_s=FINALIZE_WAIT_S,
                 fsync_interval_s=FSYNC_INTERVAL_S, fsync=os.fsync, provenance=None):
        self.archive_dir = Path(archive_dir)
        self.host_profile_path = host_profile_path
        self.recorder_url = recorder_url
        self.bag_factory = bag_factory
        self.clock, self.steady, self.sleep = clock, steady, sleep
        self.buffer_s, self.lost_after_s = float(buffer_s), float(lost_after_s)
        self.open_timeout_s, self.finalize_wait_s = float(open_timeout_s), float(finalize_wait_s)
        self.fsync_interval_s, self.fsync = fsync_interval_s, fsync
        self.provenance = dict(provenance or {})
        self.buffer = collections.deque()
        self.segment = None
        self.closing = []
        self.done = []
        self.handled = set()
        self.pending = {}               # recording id -> (first seen steady, last reason)
        self.failed = {}                # recording id -> error (handshake failed; not retried)
        self.unreachable_since = None
        self.last_status = None
        self.counts = collections.Counter()
        self.last_message_steady = None
        self.errors = collections.deque(maxlen=20)
        self.stopping = False           # shutdown: do not wait for the recorder's end state any more
        self.shutting_down = False      # shutdown deadline passed: do not wait for the bags either

    # inputs
    def on_message(self, data, received_at=None, received_steady=None):
        received_at = self.clock() if received_at is None else received_at
        received_steady = self.steady() if received_steady is None else received_steady
        self.counts['messages'] += 1
        self.last_message_steady = received_steady
        self.buffer.append((data, received_at, received_steady))
        while self.buffer and received_steady - self.buffer[0][2] > self.buffer_s:
            self.buffer.popleft()
        if self.segment is not None:
            self.segment.observe(data, received_at, received_steady)

    def on_status(self, status):
        now = self.steady()
        self.counts['status_polls'] += 1
        if not isinstance(status, dict) or status.get('ok') is not True:
            self.counts['status_errors'] += 1
            if self.unreachable_since is None:
                self.unreachable_since = now
            if self.segment is not None and now - self.unreachable_since >= self.lost_after_s:
                self._close('recorder_unreachable')
            return
        self.unreachable_since = None
        self.last_status = {k: status.get(k) for k in ('state', 'recording_id', 'started_at', 'archive_dir')}
        state, rid = status.get('state'), status.get('recording_id')
        if state == 'recording' and valid_recording_id(rid):
            if self.segment is not None:
                if self.segment.recording_id == rid:
                    return
                self._close('recorder_switched')
            self._try_open(rid, status)
        elif state == 'finalizing':
            if self.segment is not None:
                self._close('recorder_stop' if rid in (None, self.segment.recording_id) else 'recorder_switched')
            if valid_recording_id(rid) and rid not in self.handled and rid not in self.failed:
                self._write_missed(rid)
        elif state == 'idle':
            self.pending.clear()
            if self.segment is not None:
                self._close('recorder_idle')
        else:
            self.counts['status_unknown_state'] += 1

    # segments
    def _note(self, text):
        self.errors.append(dict(t=self.clock(), error=text))

    def _calibration(self):
        if not self.host_profile_path:
            return None, 'host_profile_not_configured'
        try:
            return HostCalibration.load(self.host_profile_path), None
        except (OSError, SidecarError) as exc:
            return None, 'host_profile_unavailable: %s' % str(exc)[:200]

    def _handshake(self, rid, status):
        """(recording dict, None) when the recorder's folder proves the id, else (None, reason)."""
        reported = status.get('archive_dir')
        if isinstance(reported, str) and os.path.normpath(reported) != os.path.normpath(str(self.archive_dir)):
            return None, 'archive_dir_mismatch'
        root = self.archive_dir / rid
        if root.is_symlink() or not root.is_dir():
            return None, 'recording_folder_missing'
        try:
            if root.resolve().parent != self.archive_dir.resolve():
                return None, 'recording_folder_outside_archive'
        except OSError:
            return None, 'recording_folder_unresolved'
        meta = read_json(root / 'metadata.json')
        if meta.get('id') != rid:
            return None, 'metadata_id_mismatch'
        if meta.get('state') != 'recording':
            return None, 'metadata_not_recording'
        if meta.get('started_at') != status.get('started_at') or parse_started_at(meta.get('started_at')) is None:
            return None, 'started_at_mismatch'
        return dict(id=rid, started_at=meta['started_at'], kind=meta.get('kind'), root=root), None

    def _recover(self, path):
        """Descriptor for a segment folder of an earlier sidecar process that has none (it crashed)."""
        if path.is_dir() and not path.is_symlink() and not (path / DESCRIPTOR_FILE).exists():
            try:
                recover_segment(path, reason='sidecar_restarted_unclean', provenance=self.provenance)
                self.counts['recovered_segments'] += 1
            except (OSError, SidecarError) as exc:
                self._note('recover %s failed: %s' % (path.name, exc))

    def recover_orphans(self, limit=64):
        """At start: recover the segments without a descriptor of the newest `limit` recordings (a crashed sidecar
        of an earlier run). Returns the number recovered."""
        before = self.counts['recovered_segments']
        try:
            roots = sorted((p for p in self.archive_dir.iterdir() if p.is_dir() and not p.is_symlink()),
                           key=lambda p: p.stat().st_mtime, reverse=True)[:limit]
        except OSError:
            return 0
        for root in roots:
            for path in segment_dirs(root):
                self._recover(path)
        return self.counts['recovered_segments'] - before

    def _segment_path(self, root):
        for index in range(1, MAX_SEGMENTS + 1):
            path = root / (SEGMENT_DIR if index == 1 else '%s-%d' % (SEGMENT_DIR, index))
            if path.exists() or path.is_symlink():
                self._recover(path)
                continue
            return path
        raise SidecarError('more than %d steering segments in %s' % (MAX_SEGMENTS, root))

    def _try_open(self, rid, status):
        if rid in self.handled or rid in self.failed:
            return
        recording, reason = self._handshake(rid, status)
        now = self.steady()
        if recording is None:
            first = self.pending.get(rid, (now, reason))[0]
            self.pending[rid] = (first, reason)
            if now - first >= self.open_timeout_s:
                self.failed[rid] = 'recording_handshake_failed: %s' % reason
                self.pending.pop(rid, None)
                self._note('%s %s' % (rid, self.failed[rid]))
            return
        self.pending.pop(rid, None)
        calibration, cal_error = self._calibration()
        try:
            path = self._segment_path(recording['root'])
            segment = SteeringSegment(recording['root'], path, recording, calibration=calibration,
                                      calibration_error=cal_error, clock=self.clock, steady=self.steady,
                                      fsync_interval_s=self.fsync_interval_s, fsync=self.fsync,
                                      recorder_url=self.recorder_url)
            segment.open()
        except (OSError, SidecarError) as exc:
            self.failed[rid] = 'segment_open_failed: %s' % str(exc)[:200]
            self._note('%s %s' % (rid, self.failed[rid]))
            return
        self.handled.add(rid)
        self.counts['segments_opened'] += 1
        if self.bag_factory is not None:
            try:
                bag = self.bag_factory(path / BAG_DIR, path / BAG_LOG)
                if bag is not None:
                    bag.start()
                    segment.bag = bag
                    write_atomic(path / BAG_PID, (canonical_json(dict(pid=bag.pid, command=bag.command(),
                                                                        started_at=self.clock())) + '\n').encode())
            except Exception as exc:          # the frames go on without a bag; the descriptor says why
                segment.bag_error = 'bag_start_failed: %s' % str(exc)[:200]
                self._note('%s %s' % (rid, segment.bag_error))
        self._replay(segment)
        self.segment = segment

    def _replay(self, segment):
        """Write the buffered messages received since the recording start (they came before the start was seen)."""
        for data, received_at, received_steady in list(self.buffer):
            if received_at >= segment.started_epoch and segment.observe(data, received_at, received_steady):
                segment.counts['replayed'] += 1

    def _write_missed(self, rid):
        """A recording seen only while the recorder finalised it (start and stop between two polls): a segment with
        the buffered messages since its start, no bag, and a descriptor that says so."""
        root = self.archive_dir / rid
        meta = read_json(root / 'metadata.json')
        if root.is_symlink() or not root.is_dir() or meta.get('id') != rid or \
                parse_started_at(meta.get('started_at')) is None:
            self.failed[rid] = 'missed_recording_folder_unusable'
            return
        existing = segment_dirs(root)
        if existing:                      # an earlier sidecar process recorded it: recover, do not add a segment
            for path in existing:
                self._recover(path)
            self.handled.add(rid)
            return
        calibration, cal_error = self._calibration()
        try:
            segment = SteeringSegment(root, root / SEGMENT_DIR, dict(id=rid, started_at=meta['started_at']),
                                      calibration=calibration, calibration_error=cal_error, clock=self.clock,
                                      steady=self.steady, fsync_interval_s=self.fsync_interval_s, fsync=self.fsync,
                                      recorder_url=self.recorder_url)
            segment.open(open_reason='recorder_status_finalizing_only')
            self._replay(segment)
            segment.close_frames('recording_seen_only_while_finalizing')
        except (OSError, SidecarError) as exc:
            self.failed[rid] = 'segment_open_failed: %s' % str(exc)[:200]
            return
        self.handled.add(rid)
        self.counts['missed_recordings'] += 1
        segment.closing_since = self.steady()
        self.closing.append(segment)

    def _close(self, reason):
        segment, self.segment = self.segment, None
        segment.close_frames(reason)
        if segment.bag is not None:
            try:
                segment.bag.request_stop()
            except Exception as exc:
                segment.bag_error = 'bag_stop_failed: %s' % str(exc)[:200]
        segment.closing_since = self.steady()
        self.closing.append(segment)
        self.counts['segments_closed'] += 1

    def tick(self):
        """Finalize closing segments whose bag has exited and whose recording the recorder has finished (or waited
        finalize_wait_s, or shutting down)."""
        for segment in list(self.closing):
            bag_done = True
            if segment.bag is not None:
                try:
                    bag_done = segment.bag.poll()
                except Exception as exc:
                    segment.bag_error = 'bag_poll_failed: %s' % str(exc)[:200]
            final = recorder_final(segment.recording_dir, self.last_status, segment.recording_id)
            waited = self.steady() - segment.closing_since
            if not bag_done and not self.shutting_down:
                continue
            if not (final['complete'] or waited >= self.finalize_wait_s or self.stopping or self.shutting_down):
                continue
            final['waited_s'] = round(waited, 3)
            if segment.end_reason in ('recorder_idle', 'recorder_unreachable') and final['complete']:
                segment.end_reason = 'recorder_stop' if final['state'] == 'archived' else 'recorder_lost'
                segment.handshake['end_reason'] = segment.end_reason
            bag_result = None
            if segment.bag is not None:
                try:
                    bag_result = segment.bag.result()
                except Exception as exc:
                    segment.bag_error = 'bag_result_failed: %s' % str(exc)[:200]
            try:
                segment.finalize(bag_result, final, self._provenance())
            except (OSError, ValueError) as exc:
                self._note('%s descriptor failed: %s' % (segment.recording_id, exc))
            self.closing.remove(segment)
            self.done.append(segment.status())
            del self.done[:-16]
            self.counts['segments_finalized'] += 1

    def _provenance(self):
        return dict(self.provenance, module_sha256=_module_sha256())

    def shutdown(self, reason='sidecar_stopped', limit_s=SHUTDOWN_S):
        """Close the open segment, stop the bags (SIGINT, escalation) and write every descriptor within limit_s."""
        if self.segment is not None:
            self._close(reason)
        self.stopping = True
        deadline = self.steady() + limit_s
        while self.closing and self.steady() < deadline:
            self.tick()
            if self.closing:
                self.sleep(0.05)
        self.shutting_down = True
        self.tick()

    def status(self):
        now = self.steady()
        age = None if self.last_message_steady is None else round(now - self.last_message_steady, 3)
        state = 'recording' if self.segment is not None else 'closing' if self.closing else 'idle'
        return dict(schema=STATUS_SCHEMA, updated_at=self.clock(), state=state, pid=os.getpid(), topic=TOPIC,
                    archive_dir=str(self.archive_dir), recorder_url=self.recorder_url,
                    recorder_reachable=self.unreachable_since is None and self.last_status is not None,
                    recorder=self.last_status, last_message_age_s=age,
                    segment=None if self.segment is None else self.segment.status(),
                    closing=[s.status() for s in self.closing], recent=list(self.done),
                    pending=sorted(self.pending), failed=dict(self.failed), counts=dict(self.counts),
                    errors=list(self.errors))


# --- recovery and checks ---------------------------------------------------------------------------------------------
def _read_frames(path):
    """(frames, malformed lines, total bytes) of a frames file; a line that is not one complete JSON object with
    exactly FRAME_FIELDS is malformed."""
    frames, malformed = [], 0
    with open(path, 'rb') as handle:
        raw = handle.read()
    for line in raw.split(b'\n'):
        if not line:
            continue
        try:
            frame = json.loads(line.decode('utf-8'))
        except (ValueError, UnicodeDecodeError):
            malformed += 1
            continue
        if not isinstance(frame, dict) or tuple(frame) != FRAME_FIELDS:
            malformed += 1
            continue
        frames.append(frame)
    return frames, malformed, len(raw)


def _stop_orphan_bag(segment_dir, wait_s=5.0):
    """SIGINT a bag process left running by a crashed sidecar (bag.pid, matched by its command line) and wait."""
    info = read_json(Path(segment_dir) / BAG_PID)
    pid, command = info.get('pid'), info.get('command')
    if type(pid) is not int or not isinstance(command, list) or os.name != 'posix':
        return None
    proc = Path('/proc') / str(pid) / 'cmdline'
    try:
        current = proc.read_bytes().split(b'\0')
    except OSError:
        return 'not_running'
    wanted = str(Path(segment_dir) / BAG_DIR).encode()
    if wanted not in current:
        return 'pid_reused'
    try:
        os.kill(pid, signal.SIGINT)
    except OSError:
        return 'signal_failed'
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline and proc.exists():
        time.sleep(0.05)
    return 'stopped' if not proc.exists() else 'still_running'


def recover_segment(segment_dir, reason='sidecar_restarted_unclean', provenance=None):
    """Descriptor for a segment without one (its sidecar crashed): counts from the frames file (never rewritten),
    the bag metadata if the bag closed, end_reason reason. Returns the descriptor."""
    from . import steering_bag_record as bag_mod
    segment_dir = Path(segment_dir)
    if (segment_dir / DESCRIPTOR_FILE).exists():
        raise SidecarError('segment has a descriptor already: %s' % segment_dir)
    recording_dir = segment_dir.parent
    meta = read_json(recording_dir / 'metadata.json')
    if parse_started_at(meta.get('started_at')) is None:
        raise SidecarError('recording metadata.json without started_at: %s' % recording_dir)
    orphan = _stop_orphan_bag(segment_dir)
    frames_path = segment_dir / FRAMES_FILE
    frames, malformed = [], 0
    if frames_path.is_file():
        frames, malformed, _ = _read_frames(frames_path)
    calibration, cal_error = None, 'recovered: host profile copy missing'
    try:
        calibration, cal_error = HostCalibration.load(segment_dir / HOST_PROFILE_COPY), None
    except (OSError, SidecarError):
        pass
    segment = SteeringSegment(recording_dir, segment_dir, dict(id=meta.get('id') or recording_dir.name,
                                                               started_at=meta['started_at']),
                              calibration=calibration, calibration_error=cal_error)
    segment.state = 'closing'
    segment.end_reason = reason
    segment.handshake.update(end_reason=reason, recovered_at=time.time(), orphan_bag=orphan,
                             malformed_lines=malformed)
    for frame in frames:
        segment.frame_count += 1
        segment.first_t_s = frame['t_s'] if segment.first_t_s is None else segment.first_t_s
        segment.last_t_s = frame['t_s']
        info = dict(stale_reasons=[], invalid=[], unknown_faults=[], angle_check='recovered',
                    host_profile_sha256=None, age_s=None)
        segment._account(frame, info)
        segment.preview.add(frame)
    bag_dir = segment_dir / BAG_DIR
    bag_result = None
    if bag_dir.exists():
        bag_result = dict(schema=bag_mod.SCHEMA, output_dir=BAG_DIR, recovered=True, metadata=bag_mod.read_metadata(
            bag_dir), pid_file=read_json(segment_dir / BAG_PID) or None)
    segment.error = 'recovered_without_descriptor' + ('; malformed_lines=%d' % malformed if malformed else '')
    final = recorder_final(recording_dir, None, segment.recording_id)
    return segment.finalize(bag_result, final, dict(provenance or {}, module_sha256=_module_sha256()))


def _frame_type_problems(frame):
    problems = []
    for key in ('t_s', 'elapsed_monotonic_s', 'clock_offset_s', 'received_at'):
        if _number(frame.get(key)) is None:
            problems.append(key)
    for key, (low, high) in INT_RANGES.items():
        value = frame.get(key)
        if value is not None and not (type(value) is int and low <= value <= high):
            problems.append(key)
    for key in ('stale',):
        if type(frame.get(key)) is not bool:
            problems.append(key)
    for key in ('drive_en', 'steer_en'):
        if frame.get(key) is not None and type(frame.get(key)) is not bool:
            problems.append(key)
    for key, allowed in (('source_kind', SOURCE_KINDS), ('state', MCU_STATES), ('stop_mode', STOP_MODES),
                         ('reset_reason', RESET_REASONS)):
        if frame.get(key) is not None and frame.get(key) not in allowed:
            problems.append(key)
    if frame.get('boot_id') is not None and not (isinstance(frame['boot_id'], str) and BOOT_ID.match(frame['boot_id'])):
        problems.append('boot_id')
    faults = frame.get('faults')
    if faults is not None and not (isinstance(faults, list) and all(f in FAULT_NAMES for f in faults)):
        problems.append('faults')
    angle = frame.get('angle_mrad')
    if angle is not None and type(angle) is not int:
        problems.append('angle_mrad')
    return problems


def check_segment(segment_dir):
    """A23 checks of one segment folder (pure file reads): schema, rate and loss, stale marking, descriptor SHA,
    calibration copy, separate bag topic and count. Returns a dict; 'passed' needs every judged item true."""
    segment_dir = Path(segment_dir)
    out = dict(segment=segment_dir.name, problems=[])
    raw = None
    try:
        raw = (segment_dir / DESCRIPTOR_FILE).read_bytes()
        descriptor = json.loads(raw.decode('utf-8'))
    except (OSError, ValueError, UnicodeDecodeError) as exc:
        out['problems'].append('descriptor_unreadable: %s' % type(exc).__name__)
        out['passed'] = False
        return out
    file_sha = hashlib.sha256(raw).hexdigest()
    sha_line = None
    try:
        sha_line = (segment_dir / DESCRIPTOR_SHA_FILE).read_text(encoding='ascii').split()
    except (OSError, UnicodeDecodeError):
        pass
    checks = dict(
        descriptor_schema=descriptor.get('schema') == SCHEMA,
        descriptor_content_sha256=descriptor.get('content_sha256') == content_sha256(descriptor),
        descriptor_file_sha256=bool(sha_line) and sha_line[0] == file_sha,
        descriptor_canonical=raw == descriptor_bytes(descriptor))
    frames, malformed, size = [], 0, None
    try:
        frames, malformed, size = _read_frames(segment_dir / FRAMES_FILE)
        frames_sha = file_sha256(segment_dir / FRAMES_FILE)
    except OSError:
        frames_sha = None
    type_problems = collections.Counter()
    for frame in frames:
        for key in _frame_type_problems(frame):
            type_problems[key] += 1
    t = [f['t_s'] for f in frames]
    checks.update(
        frames_schema=malformed == 0 and not type_problems,
        frames_sha256=frames_sha is not None and frames_sha == descriptor.get('frames_sha256'),
        frame_count=len(frames) == descriptor.get('frame_count'),
        t_s_monotonic=all(b >= a for a, b in zip(t, t[1:])),
        stale_marked=all(type(f['stale']) is bool for f in frames) and all(
            f['stale'] for f in frames if f['mcu_seq'] is None))
    calibration = descriptor.get('calibration') or {}
    table = calibration.get('table') or {}
    checks['calibration_copy'] = bool(table) and all(k in table for k in ('cal_id', 'mark_left_mrad',
                                                                          'mark_right_mrad'))
    checks['calibration_matches_messages'] = calibration.get('matches_messages') is True
    bag = descriptor.get('bag') or {}
    meta = bag.get('metadata') or {}
    topics = meta.get('topics') or {}
    checks['bag_finalized'] = meta.get('finalized') is True
    checks['bag_topics'] = list(topics) == [TOPIC]
    checks['bag_message_count_positive'] = bool(topics.get(TOPIC, {}).get('message_count'))
    stats = descriptor.get('stats') or {}
    out.update(checks=checks, frame_count=len(frames), malformed_lines=malformed, frames_bytes=size,
               frame_type_problems=dict(type_problems), stale_frames=sum(1 for f in frames if f['stale']),
               stats={k: stats.get(k) for k in ('unique_status_lines', 'lost_status_lines', 'loss_rate',
                                                   'repeated_status_lines', 'archive_rate_hz', 'mcu_rate_hz',
                                                   'stale_frames', 'stale_reasons', 'replayed_from_buffer',
                                                   'receive_age_s', 'angle_checks')},
               sample_hz=descriptor.get('sample_hz'), end_reason=(descriptor.get('handshake') or {}).get('end_reason'),
               bag=dict(topics=topics, message_count=meta.get('message_count'), finalized=meta.get('finalized'),
                        storage=meta.get('storage'), time_base=bag.get('time_base'), exit_code=bag.get('exit_code'),
                        signals=bag.get('signals')),
               content_sha256=descriptor.get('content_sha256'), descriptor_file_sha256=file_sha,
               error=descriptor.get('error'))
    out['passed'] = all(checks.values())
    out['problems'] += [k for k, v in checks.items() if not v]
    return out


def check_recording(recording_dir):
    """A23 checks of every steering segment of one recording folder (CLI --check)."""
    recording_dir = Path(recording_dir)
    results = [check_segment(p) for p in segment_dirs(recording_dir)]
    return dict(schema=CHECK_SCHEMA, recording=recording_dir.name, segments=results,
                passed=bool(results) and all(r['passed'] for r in results),
                note='placeholder-based SIM result (자리값 기반 SIM 결과); 20 Hz and the loss rate are reported, not judged')


# --- ROS node --------------------------------------------------------------------------------------------------------
class RecorderPoller(threading.Thread):
    """Polls the recorder status in a thread (an HTTP wait never blocks the ROS spin); the main thread takes the
    latest result with latest()."""

    def __init__(self, url, period_s=POLL_S, timeout_s=POLL_TIMEOUT_S, fetch=fetch_recorder_status):
        super().__init__(name='steering-recorder-poller', daemon=True)
        self.url, self.period_s, self.timeout_s, self.fetch = url, float(period_s), float(timeout_s), fetch
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.seq = 0
        self.result = None

    def run(self):
        while not self.stop_event.is_set():
            begin = time.monotonic()
            try:
                result = self.fetch(self.url, self.timeout_s)
            except Exception as exc:          # any failure (refused, timeout, HTTP, JSON) = recorder unreachable
                result = dict(ok=False, error='%s: %s' % (type(exc).__name__, str(exc)[:120]))
            with self.lock:
                self.seq += 1
                self.result = result
            self.stop_event.wait(max(0.0, self.period_s - (time.monotonic() - begin)))

    def latest(self):
        with self.lock:
            return self.seq, self.result


def bag_time_mode(value, environ=os.environ):
    """--bag-time auto: SIM time when DASHBOARD_MODE is simulation (the pinned recorder's own choice)."""
    if value == 'auto':
        return environ.get('DASHBOARD_MODE') == 'simulation'
    return value == 'sim'


def build_parser():
    p = argparse.ArgumentParser(prog='steering_sidecar', description=__doc__.splitlines()[0])
    p.add_argument('--archive-dir', default=os.environ.get('RECORDING_ARCHIVE_DIR', DEFAULT_ARCHIVE_DIR))
    p.add_argument('--recorder-url', default=DEFAULT_RECORDER_URL)
    p.add_argument('--host-profile', default=DEFAULT_HOST_PROFILE)
    p.add_argument('--status-file', default=DEFAULT_STATUS_FILE)
    p.add_argument('--poll-s', type=float, default=POLL_S)
    p.add_argument('--buffer-s', type=float, default=BUFFER_S)
    p.add_argument('--lost-after-s', type=float, default=LOST_AFTER_S)
    p.add_argument('--finalize-wait-s', type=float, default=FINALIZE_WAIT_S)
    p.add_argument('--fsync-interval-s', type=float, default=FSYNC_INTERVAL_S)
    p.add_argument('--bag-time', choices=('auto', 'sim', 'wall'), default='auto')
    p.add_argument('--no-bag', action='store_true', help='frames and descriptor only (no ros2 bag record)')
    p.add_argument('--check', metavar='RECORDING_DIR', help='print the A23 checks of a recording folder and exit')
    p.add_argument('--recover', metavar='SEGMENT_DIR', help='write the descriptor of a crashed segment and exit')
    return p


def single_instance_lock(archive_dir):
    """Exclusive lock file <archive_dir>/.steering_sidecar.lock (a file: the recording listings skip files; the media
    queue keeps its .media_queue.lock there too). Two sidecars would recover each other's open segments. Returns the
    open handle (keep it) or raises SidecarError. POSIX only."""
    import fcntl
    archive_dir = Path(archive_dir)
    archive_dir.mkdir(parents=True, exist_ok=True)   # the recorder's media queue creates it the same way
    handle = open(archive_dir / LOCK_FILE, 'a')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        raise SidecarError('another steering sidecar holds %s' % (archive_dir / LOCK_FILE)) from None
    return handle


def _write_status(path, payload):
    if not path:
        return
    try:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        write_atomic(path, (json.dumps(payload, allow_nan=False, default=str) + '\n').encode('utf-8'), fsync=False)
    except (OSError, ValueError):
        pass


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.check:
        result = check_recording(args.check)
        print(json.dumps(result, ensure_ascii=False, indent=1, sort_keys=True))
        return 0 if result['passed'] else 1
    if args.recover:
        descriptor = recover_segment(args.recover)
        print(json.dumps(dict(recovered=args.recover, content_sha256=descriptor['content_sha256'],
                              frame_count=descriptor['frame_count']), sort_keys=True))
        return 0
    try:
        lock = single_instance_lock(args.archive_dir)
    except (OSError, SidecarError) as exc:
        _write_status(args.status_file, dict(schema=STATUS_SCHEMA, updated_at=time.time(), state='failed',
                                             error=str(exc)))
        print('steering_sidecar: %s' % exc, file=sys.stderr, flush=True)
        return 3
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from . import steering_bag_record

    use_sim_time = bag_time_mode(args.bag_time)

    def bag_factory(output_dir, log_path):
        return steering_bag_record.BagRecord(output_dir, use_sim_time=use_sim_time, log_path=log_path)
    provenance = dict(source_commit=os.environ.get('UBTS_STEER_SOURCE_COMMIT'),
                      source_status_sha256=os.environ.get('UBTS_STEER_SOURCE_STATUS_SHA256'))
    controller = SidecarController(args.archive_dir, host_profile_path=args.host_profile,
                                   recorder_url=args.recorder_url,
                                   bag_factory=None if args.no_bag else bag_factory, buffer_s=args.buffer_s,
                                   lost_after_s=args.lost_after_s, finalize_wait_s=args.finalize_wait_s,
                                   fsync_interval_s=args.fsync_interval_s, provenance=provenance)
    stop = threading.Event()
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:
        rclpy.init(args=[])
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = rclpy.create_node(NODE_NAME, parameter_overrides=[Parameter('use_sim_time', value=False)])
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(String, TOPIC, lambda msg: controller.on_message(msg.data), qos)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    controller.recover_orphans()
    poller = RecorderPoller(args.recorder_url, args.poll_s)
    poller.start()
    seen, last_file = 0, 0.0
    _write_status(args.status_file, dict(controller.status(), state='starting'))
    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.02)
            seq, result = poller.latest()
            if seq != seen:
                seen = seq
                controller.on_status(result)
            controller.tick()
            if time.monotonic() - last_file >= 1.0:
                last_file = time.monotonic()
                _write_status(args.status_file, controller.status())
    finally:
        controller.shutdown('sidecar_stopped')
        poller.stop_event.set()
        _write_status(args.status_file, dict(controller.status(), state='stopped'))
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()
        lock.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
