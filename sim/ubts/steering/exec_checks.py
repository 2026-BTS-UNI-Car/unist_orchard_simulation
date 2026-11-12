"""Executor-in-the-loop and sensing-chain checks of the steering G-SIM (WF-4; design 2.6, 3.7, 5.2, 7.2, 7.2.1, 8.3 P6a;
SIM_ACCEPTANCE A5, A13, A17, A19(G) with amendments 4, 7, 10, 11, 17; docs/steering/S2_NODE_EXEC_KO.md).

Run in the check service of compose.ubts-steer.yaml (sim network and IPC, the control volume with the executor's files
/run/ubts-steer/exec, the emulator hooks, the runtime volume); 'graph' with the executor's node and 'safety' need the
executor service (compose profile executor):

  python3 -B -m steering.exec_checks <check> --out /runs/<folder> [options]

  graph     A13(G) with the executor running: every topic's publishers and subscribers (ROS_GRAPH.json and a plain
            listing GRAPH_LISTING.txt), rules R1-R5 of p6a_graph, R6 for the executor node ubts_row_executor as amended
            (amendment 11: /scan, /imu/data_raw, /rover/control/requested, /parameter_events, /rosout, and /clock in
            the SIM clock mode only), the executor node reads no truth topic, /odom, dynamic /tf, /joint_states,
            /ground_truth/odom, /sim/truth/*, /gps/*; the S2 format node reads only its SIM scan input
  scan      A5(G) on the published /scan of the S2 format node: 3240 beams, angle_min float32(-pi), angle_increment
            RAW_INC, frame laser, range limits, 0 = no return (no negative or non-finite value), 10 Hz (header stamps
            and receipts), raw order = the F-SIM conversion (every /scan equals steering.s2_format.RealFormat of the SIM
            scan with the same stamp, byte for byte), the lidar_yaw_offset_deg convention against geometry (W0: the
            registration pillars seen at the raw beam the convention gives, at the true distance; evaluation reads the
            truth pose), the S2 node's counters (in = out, no rejected input, no stamp gap); the range noise at
            standstill is reported against the profile (the G-SIM LiDAR noise comes from rover.yaml, WORLDS_KO 5)
  imu       A5(G) /imu/data_raw at standstill: rate vs the profile (+-10 %), gyro and acceleration noise sd vs the IMU
            error model of this run (simulation_status.json imu_model, +-10 % as A5(F)), frame imu_link, orientation
            not provided
  display   A5(G) camera and virtual GPS shown (display only): /camera/image_raw 640x360 at about 30 Hz, a JPEG frame
            of the camera WebSocket bridge (ws://127.0.0.1:8770/ws/camera, the stream the dashboard proxies) decoded at
            640x360, /gps/fix near 0 N 0 E, a GPS WebSocket snapshot (ws://127.0.0.1:8778/ws/gps) with a position
  chain     W2 sensing chain as ROS nodes (field1010): /sim/scan (cpu_lidar) -> /sim/scan_clipped (clip node) -> /scan
            (S2 node), matched by header stamp: per-stage latency (receipt of the stage vs the previous stage, and the
            wall time the SIM clock passed the stamp), SIM and wall rates, missing scans per stage, the node counters;
            development values (the judged A10(W2) is the quiet qualification's, amendment 9)
  safety    A17(G) with the executor in the loop (wall or SIM clock): the scripts of SCRIPTS (operator STOP x N,
            deadman release, hold lost, C TTL, link watchdog, fault stop, executor loop stall, executor freeze,
            bridge forced kill, bridge stall, steering status loss, STOP unconfirmed, second screen and CLI, late
            drive after STOP, MCU reset, USB unplug, half frame, CRC, range error, output guard, system time change,
            executor forced kill); each item with its measured value and the registered criterion; A10 validity per
            script in the wall clock mode (p6a_checks.step_validity); the A19(G) normal-script false detections of the
            drive-ups
  drive     one executor run with a scripted controller and the executor's own input acceptance (development)

Every result carries the clock mode, the provenance, the /clock RTF summary and the emulator /clock counters. Times:
SIM ticks of the emulator log (1 ms), the bridge event log (its clock: monotonic in the wall clock mode, SIM time in
the SIM clock mode), the executor's JSONL log (its clock, same rule) and wall time of this process.
Placeholder-based SIM results (자리값 기반 SIM 결과), not field validation.
"""
from __future__ import annotations

import argparse
import base64
import collections
import http.client
import json
import math
import os
from pathlib import Path
import statistics
import struct
import sys
import threading
import time
import uuid

from . import p6a_graph
from .p6a_checks import (BRIDGE_LOG, BRIDGE_TICK_S, EMULATOR_LOG, SIM_STATUS, STATUS_PERIOD_MS, STILL_SPEED_M_S,
                         STOP_CONFIRM_PERIODS, Ctx, analytic_stop_distance, arc_lateral, bridge_stop_latency,
                         counter_delta, criterion, drive_zero_tick, emulator_counters, emulator_slow, emulator_stalls,
                         entries, expected_nodes, f_line, false_detections, first, first_tick_after, invalidate,
                         lateral_in_start_frame, load_json, merge_false_detections, odom_rows, parse_c_line,
                         path_length, pin_first, provenance, rtf_summary, stable_graph, step_start, step_sim_latches,
                         step_validity, truth_rows, verdict, write_json)
from .p6a_client import ClientError, ControlClient, read_jsonl, unix_request
from . import s2_format as S2

RESULT_KIND = '자리값 기반 SIM 결과'
SCHEMA = 'ubts.steer_exec_check.v1'
EXEC_DIR = os.environ.get('UBTS_STEER_EXEC_DIR', '/run/ubts-steer/exec')
SUPERVISOR_HOOKS = os.environ.get('UBTS_STEER_SUPERVISOR_HOOK_SOCKET', '/run/ubts-steer-emu/supervisor.sock')
OPERATOR_HOST, OPERATOR_PORT = '127.0.0.1', int(os.environ.get('UBTS_STEER_EXEC_PORT', '8097'))
EXECUTOR_NODE = p6a_graph.EXECUTOR_NODE
S2_NODE = p6a_graph.S2_NODE
INFRA_ALWAYS = ('/parameter_events', '/rosout')     # amendment 11
INFRA_SIM_CLOCK = ('/clock',)                        # amendment 11: SIM clock mode only
FORBIDDEN_PREFIXES = ('/sim/', '/gps/', '/ground_truth')
FORBIDDEN_EXACT = ('/odom', '/tf', '/joint_states', '/ground_truth/odom')
SCAN_PERIOD_S = .1
RATE_TOL = .10
NOISE_TOL = .10
HOLD_BEAT_S = .1
STATE_ACTIVE, STATE_PAUSED, STATE_ENDED = 'ACTIVE', 'PAUSED', 'ENDED'
CLIENT_LATCHED, CLIENT_ARMED = 'LATCHED', 'ARMED'
# Bridge snapshot states without a fault (rover_control_core: 'armed_manual' with a lease, else 'disarmed'); any other
# state ('fault') needs an operator ack before a new run.
HOST_PLAIN_STATES = ('disarmed', 'armed_manual')
EXEC_RELEASE_WAIT_S = 8.        # an ended run keeps the lease up to terminal_stop_wait_s (1.5 s) before its own STOP
CAMERA_WS = 'ws://127.0.0.1:8770/ws/camera'
GPS_WS = 'ws://127.0.0.1:8778/ws/gps'
CAMERA_BINARY_MAGIC = b'UBCF'                          # scripts.camera_ws_bridge (binary-v1 transport)


# --- pure helpers (tested without ROS) ----------------------------------------------------------------------------------
def _base(name):
    return str(name).rsplit('/', 1)[-1]


def forbidden_topic(topic):
    """A truth or forbidden input topic for the executor (A13: /odom, dynamic /tf, /joint_states, /ground_truth/odom,
    /sim/*, /gps/*)."""
    return topic in FORBIDDEN_EXACT or topic.startswith(FORBIDDEN_PREFIXES)


def judge_executor_subscriptions(graph, clock, node=EXECUTOR_NODE, allowed=p6a_graph.EXECUTOR_ALLOWED):
    """A13(G) R6 with amendment 11: the topics the executor node subscribes. passed None when the node is absent."""
    subscribed = sorted(t for t, info in graph['topics'].items()
                        if any(_base(n) == node for n in info.get('subscribers', [])))
    published = sorted(t for t, info in graph['topics'].items()
                       if any(_base(n) == node for n in info.get('publishers', [])))
    present = node in [_base(n) for n in graph.get('nodes', [])] or bool(subscribed)
    infrastructure = INFRA_ALWAYS + (INFRA_SIM_CLOCK if clock == 'sim' else ())
    permitted = tuple(allowed) + infrastructure
    outside = [t for t in subscribed if t not in permitted]
    truth = [t for t in subscribed if forbidden_topic(t) or t in p6a_graph.TRUTH_TOPICS]
    clock_in_wall_mode = clock != 'sim' and '/clock' in subscribed
    missing = [t for t in allowed if t not in subscribed]
    strict = p6a_graph.check_executor_subscriptions(graph, node_name=node, allowed=tuple(allowed))
    return dict(node=node, present=present, clock=clock, subscribed=subscribed, published=published,
                permitted=list(permitted), not_allowed=outside, truth_or_forbidden=truth,
                clock_in_wall_mode=clock_in_wall_mode, expected_not_subscribed=missing,
                passed=None if not present else (not outside and not truth),
                registered_without_amendment=dict(passed=strict.get('passed'), not_allowed=strict.get('not_allowed')),
                rule='A13 allow list with amendment 11 (/parameter_events, /rosout; /clock only in the SIM clock mode)')


def judge_s2_subscriptions(graph, input_topic, node=S2_NODE):
    """The S2 format node reads only its SIM scan input (+ rclpy infrastructure) and publishes /scan."""
    subscribed = sorted(t for t, info in graph['topics'].items()
                        if any(_base(n) == node for n in info.get('subscribers', [])))
    published = sorted(t for t, info in graph['topics'].items()
                       if any(_base(n) == node for n in info.get('publishers', [])))
    extra = [t for t in subscribed if t not in (input_topic,) + INFRA_ALWAYS]
    present = node in [_base(n) for n in graph.get('nodes', [])] or bool(subscribed)
    return dict(node=node, present=present, subscribed=subscribed, published=published, expected_input=input_topic,
                extra_subscriptions=extra, scan_publishers=sorted(graph['topics'].get('/scan', {}).get('publishers',
                                                                                                      [])),
                passed=None if not present else (not extra and input_topic in subscribed and '/scan' in published))


def graph_listing(graph):
    """Plain text table of a captured graph: topic, types, publishers, subscribers (one topic per line)."""
    lines = ['# ROS graph (%d topics, %d nodes); nodes: %s' % (len(graph['topics']), len(graph.get('nodes', [])),
                                                              ', '.join(graph.get('nodes', [])))]
    for topic in sorted(graph['topics']):
        info = graph['topics'][topic]
        lines.append('%s | %s | pub: %s | sub: %s' % (topic, ','.join(info.get('types', [])),
                                                      ','.join(info.get('publishers', [])) or '-',
                                                      ','.join(info.get('subscribers', [])) or '-'))
    return '\n'.join(lines) + '\n'


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def raw_beam_of_body_bearing(bearing_rad, offset_deg):
    """The raw S2 beam index that looks at a body bearing (raw beam j looks at RAW_AMIN + j RAW_INC + offset)."""
    a = wrap(bearing_rad - math.radians(offset_deg))
    return int(round((a - S2.RAW_AMIN) / S2.RAW_INC)) % S2.BEAMS


def pillar_expectations(pose, mount_xy, pillars_xy, radius, offset_deg, max_range=S2.RANGE_MAX_M):
    """Expected (raw beam, range) of each W0 pillar from a planar pose (x, y, yaw) and the LiDAR mount (x, y) in the
    base frame: the range is the distance to the pillar centre minus its radius along the ray through the centre."""
    x, y, yaw = (float(v) for v in pose)
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy = x + c * mount_xy[0] - s * mount_xy[1], y + s * mount_xy[0] + c * mount_xy[1]
    out = []
    for px, py in pillars_xy:
        dx, dy = float(px) - ox, float(py) - oy
        distance = math.hypot(dx, dy)
        bearing = wrap(math.atan2(dy, dx) - yaw)
        expected = distance - float(radius)
        if not .05 < expected < max_range:
            continue
        out.append(dict(pillar=[px, py], bearing_deg=round(math.degrees(bearing), 4),
                        raw_beam=raw_beam_of_body_bearing(bearing, offset_deg), expected_m=round(expected, 4)))
    return out


def scan_format_problems(beams, angle_min, angle_increment, frame_id, range_min, range_max, ranges):
    """The A5 format items of one /scan message that fail (empty list: all pass)."""
    import numpy as np
    problems = []
    if beams != S2.BEAMS:
        problems.append('beams %d != %d' % (beams, S2.BEAMS))
    if abs(float(angle_min) - S2.RAW_AMIN) > 1e-9:
        problems.append('angle_min %r != float32(-pi)' % angle_min)
    if abs(float(angle_increment) - S2.RAW_INC) > 1e-12:
        problems.append('angle_increment %r != RAW_INC' % angle_increment)
    if frame_id != S2.FRAME_ID:
        problems.append('frame_id %r' % frame_id)
    if (round(float(range_min), 6), round(float(range_max), 6)) != (S2.RANGE_MIN_M, S2.RANGE_MAX_M):
        problems.append('range limits %r, %r' % (range_min, range_max))
    r = np.asarray(ranges, dtype=np.float64)
    if not np.all(np.isfinite(r)):
        problems.append('non-finite ranges')
    if np.any(r < 0.):
        problems.append('negative ranges')
    if np.any(np.isfinite(r) & (r >= S2.RANGE_MAX_M)):
        problems.append('ranges >= 30 m (no return must be 0.0)')
    return problems


def interpolate_wall(clock_rows, sim_t):
    """Wall monotonic time at which the SIM clock reached sim_t (linear between the /clock receipts), or None."""
    rows = clock_rows
    if not rows or sim_t < rows[0][1] or sim_t > rows[-1][1]:
        return None
    lo, hi = 0, len(rows) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if rows[mid][1] < sim_t:
            lo = mid
        else:
            hi = mid
    (w0, s0), (w1, s1) = rows[lo], rows[hi]
    if s1 <= s0:
        return w1
    return w0 + (w1 - w0) * (sim_t - s0) / (s1 - s0)


def interpolate_sim(clock_rows, mono):
    """SIM time of the /clock receipts at a wall monotonic time (linear between receipts), or None outside them."""
    rows = clock_rows
    if not rows or mono < rows[0][0] or mono > rows[-1][0]:
        return None
    lo, hi = 0, len(rows) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if rows[mid][0] < mono:
            lo = mid
        else:
            hi = mid
    (w0, s0), (w1, s1) = rows[lo], rows[hi]
    if w1 <= w0:
        return s1
    return s0 + (s1 - s0) * (mono - w0) / (w1 - w0)


def percentiles(values, qs=(50, 90, 99)):
    if not values:
        return {('p%d' % q): None for q in qs}
    ordered = sorted(values)
    out = {}
    for q in qs:
        k = max(0, min(len(ordered) - 1, int(math.ceil(q / 100. * len(ordered))) - 1))
        out['p%d' % q] = round(ordered[k], 4)
    out['max'] = round(ordered[-1], 4)
    out['mean'] = round(sum(ordered) / len(ordered), 4)
    out['n'] = len(ordered)
    return out


def chain_latency(receipts, clock_rows, stages):
    """Per-stage latency of a sensing chain matched by header stamp.

    receipts: {topic: {stamp_ns: wall monotonic receipt}}; clock_rows: [(wall monotonic, sim s)] of /clock; stages:
    the topics in chain order. Returns per stage: latency from the previous stage's receipt (ms), the first stage from
    the wall time the SIM clock reached the stamp; the total (last stage from the SIM clock); missing stamps (seen at
    the previous stage, not at this one)."""
    out = dict(stages=[], total_ms=None)
    first_topic, last_topic = stages[0], stages[-1]
    for index, topic in enumerate(stages):
        got = receipts.get(topic, {})
        lat, missing = [], 0
        if index == 0:
            for stamp, received in got.items():
                at = interpolate_wall(clock_rows, stamp / 1e9)
                if at is not None:
                    lat.append((received - at) * 1000.)
        else:
            previous = receipts.get(stages[index - 1], {})
            for stamp, received in previous.items():
                if stamp in got:
                    lat.append((got[stamp] - received) * 1000.)
                else:
                    missing += 1
        out['stages'].append(dict(topic=topic, received=len(got), latency_ms=percentiles(lat),
                                  missing_from_previous=None if index == 0 else missing,
                                  reference='SIM clock reached the stamp' if index == 0 else stages[index - 1]))
    total = []
    for stamp, received in receipts.get(last_topic, {}).items():
        at = interpolate_wall(clock_rows, stamp / 1e9)
        if at is not None:
            total.append((received - at) * 1000.)
    out['total_ms'] = percentiles(total)
    out['first_stage'] = first_topic
    return out


def stamp_rate(stamps_ns):
    """(SIM rate Hz from the header stamps, median step s, steps above 1.5 periods) of a stamp list."""
    stamps = sorted(set(int(s) for s in stamps_ns))
    if len(stamps) < 3:
        return dict(sim_hz=None, median_step_s=None, gaps=None, n=len(stamps))
    steps = [(b - a) / 1e9 for a, b in zip(stamps, stamps[1:])]
    span = (stamps[-1] - stamps[0]) / 1e9
    return dict(sim_hz=round((len(stamps) - 1) / span, 4) if span > 0 else None,
                median_step_s=round(statistics.median(steps), 6),
                gaps=sum(1 for s in steps if s > 1.5 * SCAN_PERIOD_S), n=len(stamps))


def wall_rate(receipts_mono):
    values = sorted(receipts_mono)
    if len(values) < 3 or values[-1] <= values[0]:
        return None
    return round((len(values) - 1) / (values[-1] - values[0]), 4)


def imu_expected_noise(model):
    """Expected gyro and acceleration noise sd at standstill of an IMU error model description (p6a_imu_model
    ImuErrorModel.describe(); the steering PWM is 0 at standstill, so the vibration RMS is the still draw)."""
    vib = float(model['still_vibration_rms_m_s2'])
    gyro_deg = math.hypot(float(model['still_noise_sd_deg_s']), float(model['vibration_coupling_deg_s_per_m_s2']) * vib)
    return dict(gyro_sd_rad_s=math.radians(gyro_deg), accel_sd_m_s2=vib / math.sqrt(3.), vibration_rms_m_s2=vib)


def within(value, expected, tol):
    return value is not None and expected is not None and abs(value - expected) <= tol * abs(expected)


def parse_camera_message(raw):
    """(metadata dict, JPEG bytes or None) of a camera WebSocket message (binary-v1 or JSON with base64 data); None
    for a message that is not a frame."""
    if isinstance(raw, (bytes, bytearray)):
        data = bytes(raw)
        if not data.startswith(CAMERA_BINARY_MAGIC) or len(data) < len(CAMERA_BINARY_MAGIC) + 5:
            return None
        offset = len(CAMERA_BINARY_MAGIC) + 1
        (size,) = struct.unpack('!I', data[offset:offset + 4])
        meta = json.loads(data[offset + 4:offset + 4 + size].decode('utf-8'))
        return meta, data[offset + 4 + size:]
    try:
        meta = json.loads(raw)
    except (TypeError, ValueError):
        return None
    if not isinstance(meta, dict) or meta.get('type') != 'frame':
        return None
    payload = meta.pop('data', None)
    return meta, None if payload is None else base64.b64decode(payload)


def jpeg_size(jpeg):
    """(width, height) of a baseline or progressive JPEG from its SOF marker, or None."""
    data = bytes(jpeg or b'')
    if not data.startswith(b'\xff\xd8'):
        return None
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        length = struct.unpack('!H', data[i + 2:i + 4])[0]
        if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            height, width = struct.unpack('!HH', data[i + 5:i + 9])
            return width, height
        i += 2 + length
    return None


# --- clients of the executor container ----------------------------------------------------------------------------------
class OperatorClient:
    """The executor's operator page (operator_http) as the operator's browser uses it: token header, exact Host, Origin
    on every POST, JSON bodies. request() -> (HTTP status, reply dict, wall time, monotonic time) of the send."""

    def __init__(self, exec_dir=EXEC_DIR, host=OPERATOR_HOST, port=OPERATOR_PORT, timeout_s=3.):
        self.host, self.port, self.timeout_s = host, int(port), float(timeout_s)
        self.token_path = Path(exec_dir) / 'log' / 'operator_token.txt'
        self.token = None
        self.log = collections.deque(maxlen=4000)
        self.lock = threading.Lock()

    def load_token(self, wait_s=30.):
        end = time.monotonic() + wait_s
        while True:
            try:
                token = self.token_path.read_bytes().decode('utf-8').strip()
                if len(token) >= 16:
                    self.token = token
                    return token
            except OSError:
                pass
            if time.monotonic() >= end:
                raise ClientError('no operator token at %s' % self.token_path)
            time.sleep(.25)

    def request(self, method, route, body=None, timeout_s=None):
        if self.token is None:
            self.load_token()
        headers = {'X-Row-Token': self.token}
        data = None
        if method == 'POST':
            data = json.dumps(body or {}).encode('utf-8')
            headers.update({'Origin': 'http://%s:%d' % (self.host, self.port), 'Content-Type': 'application/json'})
        conn = http.client.HTTPConnection(self.host, self.port, timeout=timeout_s or self.timeout_s)
        wall, mono = time.time(), time.monotonic()
        try:
            conn.request(method, route, body=data, headers=headers)
            response = conn.getresponse()
            raw, code = response.read(), response.status
        finally:
            conn.close()
        try:
            reply = json.loads(raw.decode('utf-8'))
        except ValueError:
            reply = dict(ok=False, error='non_json_reply', raw=raw[:200].decode('utf-8', 'replace'))
        with self.lock:
            self.log.append(dict(wall=round(wall, 4), route=route, code=code,
                                 ok=reply.get('ok') if isinstance(reply, dict) else None,
                                 error=reply.get('error') if isinstance(reply, dict) else None))
        return code, reply, wall, mono

    def post(self, route, body=None, timeout_s=None):
        return self.request('POST', route, body, timeout_s)

    def status(self):
        return self.request('GET', '/api/status')[1]


class HoldBeater:
    """The hold-to-run button: press() starts a press with a new id and beats every HOLD_BEAT_S (wall) like the page;
    release() sends the release; lose() stops the beats without a release (the page or its link is gone)."""

    def __init__(self, op, period_s=HOLD_BEAT_S):
        self.op, self.period_s = op, float(period_s)
        self.press_id = None
        self.beating = False
        self.stop_event = threading.Event()
        self.thread = None
        self.replies = collections.Counter()
        self.last_beat = None                 # (wall, mono) of the last heartbeat sent
        self.events = []

    def _ensure_thread(self):
        if self.thread is None or not self.thread.is_alive():
            self.stop_event.clear()
            self.thread = threading.Thread(target=self._run, name='exec-check-hold', daemon=True)
            self.thread.start()

    def _beat(self):
        press = self.press_id
        if press is None:
            return
        try:
            code, reply, wall, mono = self.op.post('/api/hold', {'press': press}, timeout_s=1.)
        except (OSError, ClientError) as exc:
            self.replies['error:%s' % type(exc).__name__] += 1
            return
        self.last_beat = (wall, mono)
        self.replies['ok' if reply.get('ok') else 'refused:%s' % reply.get('error')] += 1

    def _run(self):
        while not self.stop_event.wait(self.period_s):
            if self.beating:
                self._beat()

    def press(self):
        self.press_id = 'chk-%s' % uuid.uuid4().hex[:16]
        self.beating = True
        self._ensure_thread()
        self._beat()
        self.events.append(dict(op='press', press=self.press_id, wall=time.time(), mono=time.monotonic()))
        return self.press_id

    def release(self):
        self.beating = False
        press, self.press_id = self.press_id, None
        if press is None:
            return None
        code, reply, wall, mono = self.op.post('/api/release', {'press': press}, timeout_s=1.)
        self.events.append(dict(op='release', press=press, wall=wall, mono=mono, code=code))
        return dict(wall=wall, mono=mono, code=code)

    def lose(self):
        self.beating = False
        last = self.last_beat
        self.events.append(dict(op='lose', press=self.press_id, wall=time.time(), mono=time.monotonic(),
                                last_beat_wall=None if last is None else last[0],
                                last_beat_mono=None if last is None else last[1]))
        self.press_id = None
        return last

    def close(self):
        self.beating = False
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.)


class SocketHooks:
    """One JSON line per connection to a hook socket (the executor's SIM hooks, the steer supervisor's hooks)."""

    def __init__(self, path, timeout_s=4.):
        self.path, self.timeout_s = str(path), float(timeout_s)

    def request(self, payload):
        reply = unix_request(self.path, payload, self.timeout_s)
        if not isinstance(reply, dict):
            raise ClientError('hook reply is not an object')
        return reply

    def call(self, op, **fields):
        reply = self.request(dict(op=op, **fields))
        if not reply.get('ok'):
            raise ClientError('hook %s failed: %s' % (op, reply.get('error')))
        return reply


def executor_log(exec_dir=EXEC_DIR, after_wall=None, run_token=None):
    """Records of the executor JSONL logs (the newest files first written after after_wall - 120 s), in file order.
    Event records carry 'wall'; tick records only the executor clock 't'."""
    folder = Path(exec_dir) / 'log'
    files = sorted(folder.glob('row_executor_*.jsonl'), key=lambda p: p.stat().st_mtime) if folder.is_dir() else []
    if after_wall is not None:
        files = [p for p in files if p.stat().st_mtime >= after_wall - 120.]
    out = []
    for path in files:
        with open(path, 'rb') as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    out.append(json.loads(line.decode('utf-8')))
                except ValueError:
                    continue
    return out


def executor_events(records, names=None, after_wall=None):
    return [r for r in records if r.get('k') == 'event' and (names is None or r.get('event') in names)
            and (after_wall is None or r.get('wall', 0.) >= after_wall)]


def executor_ticks(records, after_t=None):
    return [r for r in records if r.get('k') == 'tick' and (after_t is None or r.get('t', -1.) >= after_t)]


# --- context ------------------------------------------------------------------------------------------------------------
class ExecCtx(Ctx):
    """p6a_checks.Ctx (the check's own control client, emulator hooks, SimObserver) plus the executor's operator page,
    hold button, SIM hooks and the steer supervisor's hooks."""

    def __init__(self, out, clock='wall', owner='exec-check', exec_dir=EXEC_DIR):
        super().__init__(out, owner=owner)
        self.clock = clock
        self.exec_dir = exec_dir
        self.op = OperatorClient(exec_dir)
        self.hold = HoldBeater(self.op)
        self.exec_hooks = SocketHooks(Path(exec_dir) / 'hooks.sock')
        self.sup = SocketHooks(SUPERVISOR_HOOKS)
        self.executor_alive = True

    def exec_status(self):
        return self.exec_hooks.call('status')['status']

    def try_exec_status(self):
        try:
            return self.exec_status()
        except (OSError, ClientError, ValueError, KeyError):
            return None

    def wait_exec(self, predicate, timeout_s=5., period_s=.05):
        end = time.monotonic() + timeout_s
        status = None
        while time.monotonic() < end:
            status = self.try_exec_status()
            if status is not None and predicate(status):
                return status
            self.obs.spin_for(period_s)
        return None

    def close(self):
        self.hold.close()
        super().close()


# --- executor session ---------------------------------------------------------------------------------------------------
class SessionError(RuntimeError):
    pass


def bridge_ready_to_arm(status):
    """The bridge snapshot allows an executor ARM: not armed, STOP confirmed, steering status fresh, nothing moving."""
    host = status.get('steering_host') or {}
    return (status.get('armed') is False and host.get('stop_state') == 'confirmed' and host.get('stale') is False
            and status.get('moving') is False)


def ensure_idle(ctx, timeout_s=20.):
    """The executor ready for a new run (no open run, client not latched) and the vehicle still."""
    ctx.hold.lose()
    status = ctx.exec_status()
    if status['runner']['state'] in (STATE_ACTIVE, STATE_PAUSED):
        ctx.op.post('/api/end')
        ctx.wait_exec(lambda s: s['runner']['state'] not in (STATE_ACTIVE, STATE_PAUSED), 5.)
    # An ended run keeps the lease until the vehicle stands or terminal_stop_wait_s is over, then the executor stops
    # the bridge itself. Wait for that whoever ended the run (the clean-up of a script posts /api/end and returns):
    # a Start inside that time is refused with bridge_still_armed (ST-05: 15 scripts failed at their start this way).
    ctx.wait_exec(lambda s: (s.get('client') or {}).get('state') != CLIENT_ARMED, EXEC_RELEASE_WAIT_S)
    status = ctx.exec_status()
    client = status.get('client') or {}
    if client.get('state') == CLIENT_LATCHED:
        ctx.op.post('/api/ack')
    ready = ctx.wait_host(lambda s: bridge_ready_to_arm(s) or s.get('state') not in HOST_PLAIN_STATES, timeout_s)
    ctx.wait_still(10.)
    return ready


def start_drive(ctx, spec, moving=True, timeout_s=30., arm_attempts=3):
    """Start, Arm and press the hold with the scripted controller spec; wait for ACTIVE (and motion). Returns a dict."""
    ensure_idle(ctx)
    ctx.exec_hooks.call('controller', spec=spec)
    info = dict(spec=spec, arm_attempts=[])
    for attempt in range(arm_attempts):
        status = ctx.exec_status()
        latched = (status.get('client') or {}).get('state') == CLIENT_LATCHED
        if latched or ctx.host().get('state') not in HOST_PLAIN_STATES:
            ctx.op.post('/api/ack')
        code, reply, _, _ = ctx.op.post('/api/start', {'row_len_m': None})
        if not reply.get('ok') and reply.get('error') != 'run_in_progress':
            raise SessionError('start refused: %s' % reply)
        ctx.wait_host(bridge_ready_to_arm, 8.)
        code, reply, wall, _ = ctx.op.post('/api/arm', timeout_s=8.)
        info['arm_attempts'].append(dict(code=code, ok=reply.get('ok'), error=reply.get('error'), wall=wall))
        if reply.get('ok'):
            break
        ctx.obs.spin_for(.5)
    else:
        raise SessionError('arm refused: %s' % info['arm_attempts'][-1])
    ctx.wait_host(lambda s: (s.get('steering_host') or {}).get('arm_state') == 'confirmed', 3.)
    end = time.monotonic() + timeout_s
    presses = 0
    status = None
    while time.monotonic() < end:
        if ctx.hold.press_id is None:
            ctx.hold.press()
            presses += 1
        status = ctx.wait_exec(lambda s: s['runner']['state'] == STATE_ACTIVE or
                               str(s['runner']['reason']).startswith('press_ignored') or
                               s['runner']['state'] == STATE_ENDED, 3.)
        if status is not None and status['runner']['state'] == STATE_ACTIVE:
            break
        if status is not None and status['runner']['state'] == STATE_ENDED:
            raise SessionError('run ended before ACTIVE: %s' % status['runner'])
        ctx.hold.release()
        ctx.obs.spin_for(.6)
    else:
        raise SessionError('no ACTIVE run within %.0f s (last runner %s)' % (
            timeout_s, None if status is None else status['runner']))
    info.update(presses=presses, active_wall=time.time())
    if moving and not ctx.obs.spin_until(lambda: bool(ctx.obs.odom) and abs(ctx.obs.odom[-1][4]) > .05, 10.):
        raise SessionError('the vehicle did not move within 10 s of ACTIVE')
    return info


def drive_spec(throttle=40, steer_deg=5., seconds=60., lead_s=1., end='stop'):
    """A segments controller: a straight lead, then a constant steering angle at the throttle."""
    return dict(kind='segments', end=end, segments=[
        dict(duration_s=float(lead_s), throttle=int(throttle), steer_deg=0., label='lead'),
        dict(duration_s=float(seconds), throttle=int(throttle), steer_deg=float(steer_deg), label='drive')])


# --- emulator log measurements ------------------------------------------------------------------------------------------
def s_sequence(lines):
    """The steering targets (S, permille) of C lines in log order, consecutive repeats removed."""
    out = []
    for c in lines:
        s = c['c']['S']
        if not out or out[-1] != s:
            out.append(s)
    return out


def moves_away_from(sequence, target):
    """True when a value of the sequence lies farther from target than the one before it, or on the other side of it
    (output_guard: the settle segment ramps from the previous steering to 0; a raw step past the guard would move
    away from 0). gsim_final 2026-11-13 w0_a17: 112, 94, 57, 5, 0 (the ramp; the old check wanted 0 only)."""
    for a, b in zip(sequence, sequence[1:]):
        if abs(b - target) > abs(a - target) or (b - target) * (a - target) < 0:
            return True
    return False


def c_lines(emu, after_tick=0):
    out = []
    for e in entries(emu, 'h2m'):
        if e.get('last') is None or e['last'] < after_tick or not e['text'].startswith('C '):
            continue
        parsed = parse_c_line(e['text'])
        if parsed is not None:
            out.append(dict(e, c=parsed))
    return out


def restart_after(emu, zero_tick):
    """Drive pin rises after the drive reached 0 at zero_tick (an automatic restart)."""
    if zero_tick is None:
        return []
    return [e for e in entries(emu, 'pin') if e['channel'] in ('drive_l', 'drive_r') and e['value'] != 0 and
            e['tick'] > zero_tick]


def host_arm_lines(emu, after_tick):
    return [e for e in entries(emu, 'h2m') if e.get('last', -1) >= after_tick and e['text'].startswith('ARM')]


def drive_level_at(emu, tick):
    """The last drive_l pin value at or before tick (None when unknown)."""
    value = None
    for e in entries(emu, 'pin'):
        if e['channel'] == 'drive_l' and e['tick'] <= tick:
            value = e['value']
    return value


def hold_deviation_deg(obs, t0, seconds=1.9):
    rows = truth_rows(obs, t0, t0 + seconds)
    if not rows:
        return None, 0
    return math.degrees(max(abs(r['delta_eq_rad'] - rows[0]['delta_eq_rad']) for r in rows)), len(rows)


def mark_sim_of_tick(mark, tick):
    return mark['sim'] + (tick - mark['tick']) / 1000.


def deadband_deg(ctx):
    info, params = ctx.session_params()
    return (params['db'] / info['calibration']['scale_permille_per_deg']) if params else .5


# --- the scripts (A17(G) executor items) -------------------------------------------------------------------------------
SCRIPTS = []


def script(name, purpose, registered):
    def wrap_fn(fn):
        SCRIPTS.append(dict(name=name, purpose=purpose, registered=registered, fn=fn))
        return fn
    return wrap_fn


def _finish(ctx, name, step0, before, criteria, details, normal_until_tick=None, expected=()):
    """Common end of a script: A10 validity (wall clock), the A19(G) normal false detections of the drive-up (step start
    to the injection), the verdict. details['precondition'] = dict(met=False, reason=...) makes the step invalid too
    (the injection did not meet the state it is meant to test, e.g. the run had already paused)."""
    after = emulator_counters(ctx.hooks)
    sim_made = step_sim_latches(ctx, step0, ctx.clock) if ctx.clock == 'wall' else ()
    validity = step_validity(ctx, ctx.clock, step0['mono'], time.monotonic(), before, after, sim_made=sim_made)
    pre = details.get('precondition')
    if pre is not None and not pre.get('met') and (validity is None or validity.get('valid')):
        validity = dict(validity or {}, valid=False, reason='precondition: %s' % pre.get('reason'))
    criteria = invalidate(criteria, validity)
    emu, _ = ctx.logs(step0, wait=False)
    window = [e for e in emu if normal_until_tick is None or e.get('tick', 0) <= normal_until_tick]
    false = false_detections(window, read_jsonl(BRIDGE_LOG), expected=expected, after_tick=step0['tick'],
                             after_wall=step0['wall'] - .01, stalls=emulator_stalls(), slow=emulator_slow(),
                             clock=ctx.clock)
    ticks = [e.get('tick', 0) for e in window if e.get('tick') is not None]
    false['normal_sim_s'] = round((max(ticks) - step0['tick']) / 1000., 3) if ticks else 0.
    if normal_until_tick is not None:
        false['window'] = 'drive-up: step start to the injection (tick %d)' % normal_until_tick
        # a host latch after the injection wall time belongs to the fault, not to the drive-up
        cut = details.get('injection_wall')
        if cut is not None:
            false['host_latches'] = [r for r in false['host_latches'] if (r.get('wall') or 0) < cut]
            by = {k: v for k, v in false['by_code'].items() if not k.startswith('host:')}
            for row in false['host_latches']:
                by['host:' + row['latch']] = by.get('host:' + row['latch'], 0) + 1
            false['by_code'], false['count'] = by, sum(by.values())
    return dict(name=name, criteria=criteria, passed=verdict(criteria), validity=validity, false_detection=false,
                **details)


def _end_reason(ctx, timeout_s=4.):
    status = ctx.wait_exec(lambda s: s['runner']['state'] == STATE_ENDED, timeout_s)
    status = status or ctx.try_exec_status() or {}
    runner = status.get('runner') or {}
    return runner.get('end_reason'), runner.get('end_note'), (status.get('client') or {})


@script('operator_stop', 'operator STOP on the executor page while driving with a held steering angle: STOP chain, '
        'hold, no restart, the executor latches', 'A17: STOP request -> STOP bytes <= 20 ms; MCU STOP -> drive 0 <= 1 '
        'servo tick; STOP confirm <= 2 status periods; hold error <= max(1 deg, 2 x deadband); no automatic restart')
def s_operator_stop(ctx, index=0):
    before, step0 = step_start(ctx, 'exec_stop_%d_start' % index)
    start = start_drive(ctx, drive_spec(40, 5.))
    ctx.obs.spin_sim_for(3.)
    mark = ctx.mark('exec_stop_%d' % index)
    sim_stop = ctx.obs.sim_time
    code, reply, http_wall, _ = ctx.op.post('/api/stop')
    ctx.obs.spin_sim_for(3.5)
    emu, bridge = ctx.logs(mark)
    req, tx, confirm = bridge_stop_latency(bridge, mark['wall'] - .01)
    stop_line = first(entries(emu, 'h2m'), lambda e: e['text'].startswith('STOP'))
    arrival = stop_line['last'] if stop_line else None
    zero = drive_zero_tick(emu, arrival) if arrival is not None else None
    restart = restart_after(emu, zero)
    arms = host_arm_lines(emu, arrival or mark['tick'])
    late_c = c_lines(emu, arrival) if arrival is not None else []
    hold_dev, hold_n = hold_deviation_deg(ctx.obs, mark_sim_of_tick(mark, arrival)) if arrival is not None else \
        (None, 0)
    end_reason, end_note, client = _end_reason(ctx)
    db = deadband_deg(ctx)
    send = None if (req is None or tx is None) else tx['t'] - req['t']
    host_confirm = None if (tx is None or confirm is None) else confirm['t'] - tx['t']
    criteria = [
        criterion('page_stop_http', code, 200, code == 200, 'operator page POST /api/stop (stop_sent %s)' %
                  reply.get('stop_sent')),
        criterion('stop_request_to_stop_bytes_s', send, '<= 0.020 (bridge clock %s)' % ctx.clock,
                  send is not None and send <= BRIDGE_TICK_S + 1e-9),
        criterion('mcu_stop_to_drive_zero_ticks', None if (zero is None or arrival is None) else zero - arrival,
                  '<= 2 (1 servo tick at 500 Hz)', zero is not None and arrival is not None and zero - arrival <= 2),
        criterion('stop_confirm_host_s', host_confirm, '<= %.3f (2 status periods, STOP bytes to the host confirmation)'
                  % (STOP_CONFIRM_PERIODS * STATUS_PERIOD_MS / 1000.),
                  host_confirm is not None and host_confirm <= STOP_CONFIRM_PERIODS * STATUS_PERIOD_MS / 1000. + 1e-9),
        criterion('hold_error_deg', None if hold_dev is None else round(hold_dev, 4), '<= max(1, 2 x deadband %.2f)'
                  % (2 * db), hold_dev is not None and hold_dev <= max(1., 2 * db)),
        criterion('no_automatic_restart', dict(drive_rises=len(restart), host_arm_lines=len(arms)), '0, 0',
                  not restart and not arms),
        criterion('executor_end_reason', end_reason, 'operator_stop', end_reason == 'operator_stop'),
        criterion('executor_client_latched', client.get('state'), 'LATCHED (operator acknowledgement needed)',
                  client.get('state') == CLIENT_LATCHED),
        criterion('page_to_bridge_request_ms', None if req is None else round((req['wall'] - http_wall) * 1000., 2),
                  'report (operator page POST to the bridge request, wall)', None),
        criterion('c_lines_after_stop', len(late_c), 'report (A22: no commanded steering move after the terminal '
                  'moment; the MCU is DISARMED)', None),
    ]
    details = dict(start=start, mark=mark, http_wall=http_wall, request_t=None if req is None else req['t'],
                   tx_t=None if tx is None else tx['t'], stop_arrival_tick=arrival, drive_zero_tick=zero,
                   hold_samples=hold_n, end_note=end_note, speed_at_stop=None, injection_wall=http_wall)
    rows = odom_rows(ctx.obs, sim_stop, sim_stop + .2)
    details['speed_at_stop'] = abs(rows[0][4]) if rows else None
    # a normal script: the whole window counts for the A19(G) normal false detections (no fault is expected)
    return _finish(ctx, 'operator_stop_%d' % index, step0, before, criteria, details, normal_until_tick=None)


@script('deadman_release', 'the operator lets the hold go while driving: pause, zero throttle at once, steering held, no '
        'restart (no new press)', 'A17: no automatic restart; the steering is held (no centring); report the release '
        'to zero-duty C line time')
def s_deadman_release(ctx):
    before, step0 = step_start(ctx, 'exec_release_start')
    start = start_drive(ctx, drive_spec(40, 4.))
    ctx.obs.spin_sim_for(3.)
    mark = ctx.mark('exec_release')
    rel = ctx.hold.release()
    ctx.obs.spin_sim_for(4.)
    return _deadman_measure(ctx, 'deadman_release', step0, before, start, mark, rel['wall'], 'hold_released')


@script('hold_lost', 'the page stops sending heartbeats while the button is held (the 09-19 test "key held during a '
        'link loss" on the operator link): the hold times out, pause, zero throttle, no restart', 'A17: no automatic '
        'restart; hold timeout <= hold_timeout_s + 1 executor tick (report)')
def s_hold_lost(ctx):
    before, step0 = step_start(ctx, 'exec_hold_lost_start')
    start = start_drive(ctx, drive_spec(40, 4.))
    ctx.obs.spin_sim_for(3.)
    mark = ctx.mark('exec_hold_lost')
    last = ctx.hold.lose()
    ctx.obs.spin_sim_for(4.)
    return _deadman_measure(ctx, 'hold_lost', step0, before, start, mark, last[0] if last else mark['wall'],
                            'hold_released', timeout_s=.5)


def _deadman_measure(ctx, name, step0, before, start, mark, t_wall, pause_reason, timeout_s=None):
    emu, bridge = ctx.logs(mark)
    lines = c_lines(emu, mark['tick'])
    held = [c for c in lines if c['c']['L'] != 0 or c['c']['R'] != 0]
    zero_line = first(lines, lambda c: c['c']['L'] == 0 and c['c']['R'] == 0)
    steer_before = held[-1]['c']['S'] if held else (lines[0]['c']['S'] if lines else None)
    after_zero = [c['c']['S'] for c in lines if zero_line is not None and c['last'] >= zero_line['last']]
    zero = drive_zero_tick(emu, mark['tick'])
    restart = restart_after(emu, zero)
    status = ctx.try_exec_status() or {}
    runner = status.get('runner') or {}
    records = executor_log(ctx.exec_dir, after_wall=mark['wall'])
    pause_ticks = [r for r in executor_ticks(records) if r.get('state') == STATE_PAUSED]
    still = ctx.wait_still(3.)
    zero_ms = None if zero_line is None else zero_line['last'] - mark['tick']
    criteria = [
        criterion('pause_reason', runner.get('reason'), pause_reason, runner.get('state') == STATE_PAUSED and
                  runner.get('reason') == pause_reason, 'the run stays open (PAUSED) until a new press'),
        criterion('zero_throttle_c_line_ms', zero_ms, 'report (mark before the action to the first zero-duty C line '
                  'at the MCU, SIM ms)', None),
        criterion('steering_held_after_release', None if steer_before is None else dict(before=steer_before,
                  after=sorted(set(after_zero))), 'the C lines after the zero throttle keep the steering (no centring)',
                  steer_before is not None and bool(after_zero) and set(after_zero) == {steer_before}),
        criterion('drive_zero_and_vehicle_still', dict(drive_zero_tick=zero, still=bool(still)), 'drive 0, still',
                  zero is not None and bool(still)),
        criterion('no_automatic_restart', len(restart), 0, not restart),
    ]
    if timeout_s is not None:
        expiry = None
        last_mono = None if not ctx.hold.events else ctx.hold.events[-1].get('last_beat_mono')
        if pause_ticks and last_mono is not None:
            first_pause_t = pause_ticks[0].get('t')
            # the executor clock: the kernel CLOCK_MONOTONIC (wall clock mode; the same clock in every container) or
            # the SIM time (SIM clock mode: the last beat mapped to SIM time through this node's /clock receipts)
            reference = last_mono if ctx.clock != 'sim' else interpolate_sim(list(ctx.obs.clock_rows), last_mono)
            if first_pause_t is not None and reference is not None:
                expiry = round(first_pause_t - reference, 3)
        criteria.append(criterion('hold_expiry_after_last_beat_s', expiry, 'report (<= %.2f: hold_timeout_s + 1 '
                                  'executor tick; executor clock %s)' % (timeout_s + .05, ctx.clock), None))
    details = dict(start=start, mark=mark, action_wall=t_wall, zero_line_tick=None if zero_line is None else
                   zero_line['last'], drive_zero_tick=zero, pause_ticks=len(pause_ticks), injection_wall=t_wall)
    out = _finish(ctx, name, step0, before, criteria, details, normal_until_tick=None)      # a normal script
    ctx.op.post('/api/end')
    return out


def _link_loss_measure(ctx, mark, reply, loss_ms):
    """C TTL and watchdog items of a link loss to the MCU (p6a_checks.link_loss rules) with the executor driving."""
    info, params = ctx.session_params()
    wd = int(params['wd']) if params else 500
    ttl_mode = 'release' if params and int(params.get('ttlmode', 0)) == 1 else 'brake'
    emu, bridge = ctx.logs(mark)
    start = reply['start_tick']
    before = [e for e in read_jsonl(EMULATOR_LOG) if e.get('session') == mark['session'] and e['kind'] == 'h2m'
              and e.get('note') == 'host' and e.get('last') is not None and e['last'] < start + 1]
    last_c = first(list(reversed(before)), lambda e: e['text'].startswith('C '))
    last_input = first(list(reversed(before)), lambda e: e['text'].startswith(('C ', 'PING')))
    c = parse_c_line(last_c['text']) if last_c else None
    zero = drive_zero_tick(emu, start)
    ttl_line = f_line(emu, 'TTL', start)
    w_line = first(entries(emu, 'm2h'), lambda e: e['tick'] >= start and e['text'].startswith('W '))
    disarm = pin_first(emu, 'mcu_state', lambda v: v == 1, start)
    ttl_expected = None if c is None else last_c['last'] + c['ttl']
    restart = restart_after(emu, zero)
    t_loss = mark_sim_of_tick(mark, start)
    rows = odom_rows(ctx.obs, t_loss, t_loss + loss_ms / 1000. + 1.5)
    held = truth_rows(ctx.obs, t_loss - .05, t_loss + .05)
    lateral = arc = s_path = s_analytic = ttl_remaining_s = None
    if len(rows) > 2 and held:
        still = first(rows, lambda r: r[0] > t_loss + .05 and abs(r[4]) < STILL_SPEED_M_S)
        rows = [r for r in rows if still is None or r[0] <= still[0]]
        from .vehicle_profile import load_profile
        profile = load_profile()
        wheelbase = float(profile.value('wheelbase_m'))
        rear = float(profile.value('rear_axle_x_in_base_m'))
        rows = [(r[0], r[1] + rear * math.cos(r[3]), r[2] + rear * math.sin(r[3]), r[3], r[4], r[5]) for r in rows]
        s_path = path_length(rows)
        _, lateral = lateral_in_start_frame(rows)
        kappa = math.tan(held[0]['delta_eq_rad']) / wheelbase
        if c is not None:
            ttl_remaining_s = max(0., (last_c['last'] + c['ttl'] - start) / 1000.)
            s_analytic, _ = analytic_stop_distance(profile, rows[0][4], ttl_remaining_s, ttl_mode)
            arc = arc_lateral(kappa, s_analytic)
    return dict(wd=wd, ttl_mode=ttl_mode, start_tick=start, last_c=None if last_c is None else last_c['last'],
                last_input=None if last_input is None else last_input['last'], drive_zero_tick=zero,
                ttl_line=None if ttl_line is None else ttl_line['tick'], ttl_expected=ttl_expected,
                w_line=None if w_line is None else w_line['tick'], disarm_tick=None if disarm is None else disarm['tick'],
                restart=len(restart), lateral_m=lateral, arc_m=arc, s_path_m=s_path, s_analytic_m=s_analytic,
                ttl_remaining_s=ttl_remaining_s, held_delta_deg=math.degrees(held[0]['delta_eq_rad']) if held else None)



def _ttl_criteria(m):
    out = [criterion('ttl_drive_zero_tick_minus_expected', None if (m['drive_zero_tick'] is None or m['ttl_expected']
                     is None) else m['drive_zero_tick'] - m['ttl_expected'], '0..1 (<= ttl + 1 tick)',
                     m['drive_zero_tick'] is not None and m['ttl_expected'] is not None and
                     0 <= m['drive_zero_tick'] - m['ttl_expected'] <= 1),
           criterion('no_automatic_restart', m['restart'], 0, m['restart'] == 0)]
    if m['lateral_m'] is not None and m['arc_m'] is not None:
        out.append(criterion('lateral_deviation_vs_analytic_arc_m', [round(m['lateral_m'], 4), round(m['arc_m'], 4)],
                             '|sim - arc| <= max(10 %%, 0.01 m)', abs(abs(m['lateral_m']) - abs(m['arc_m'])) <=
                             max(.1 * abs(m['arc_m']), .01)))
    return out


@script('c_ttl_link_returns', 'C lines stop reaching the MCU for 0.4 s while the executor drives (under the link '
        'watchdog): MCU C TTL brake; when the link returns, the executor\'s next C must not restart the drive (review '
        'RS-01, DECISIONS R18 decision 1)', 'A17: C TTL -> drive 0 <= ttl + 1 tick; no automatic restart; lateral '
        'deviation vs the analytic arc (+-10 % or 0.01 m)')
def s_c_ttl(ctx):
    before, step0 = step_start(ctx, 'exec_ttl_start')
    start = start_drive(ctx, drive_spec(48, 5.))
    ctx.obs.spin_sim_for(3.)
    mark = ctx.mark('exec_ttl')
    reply = ctx.hooks.link(direction='to_mcu', duration_ms=400)
    injection_wall = time.time()
    ctx.obs.spin_sim_for(3.)
    m = _link_loss_measure(ctx, mark, reply, 400)
    end_reason, end_note, client = _end_reason(ctx)
    criteria = _ttl_criteria(m) + [
        criterion('mcu_ttl_line', m['ttl_line'], 'an F TTL line', m['ttl_line'] is not None),
        criterion('executor_end_reason', end_reason, 'a link stop reason (link_ttl_expired, or the host TTL latch of '
                  'R18 decision 1)', end_reason is not None and str(end_reason).startswith(('link_', 'bridge_',
                                                                                             'mcu_'))),
    ]
    return _finish(ctx, 'c_ttl_link_returns', step0, before, criteria,
                   dict(start=start, mark=mark, measure=m, end_reason=end_reason, end_note=end_note,
                        client=client.get('state'), injection_wall=injection_wall), normal_until_tick=mark['tick'])


@script('link_watchdog', 'the link to the MCU is lost for 1.5 s while the executor drives: C TTL brake, then the link '
        'watchdog (W, DISARMED); no restart', 'A17: C TTL -> drive 0 <= ttl + 1 tick; watchdog <= wd + 1 tick, DISARMED; '
        'no automatic restart; lateral deviation vs the analytic arc')
def s_link_watchdog(ctx):
    before, step0 = step_start(ctx, 'exec_wd_start')
    start = start_drive(ctx, drive_spec(48, 5.))
    ctx.obs.spin_sim_for(3.)
    mark = ctx.mark('exec_wd')
    reply = ctx.hooks.link(direction='to_mcu', duration_ms=1500)
    injection_wall = time.time()
    ctx.obs.spin_sim_for(3.5)
    m = _link_loss_measure(ctx, mark, reply, 1500)
    end_reason, end_note, client = _end_reason(ctx)
    criteria = _ttl_criteria(m) + [
        criterion('watchdog_tick_minus_last_input', None if (m['w_line'] is None or m['last_input'] is None) else
                  m['w_line'] - m['last_input'], '<= wd + 1 = %d' % (m['wd'] + 1), m['w_line'] is not None and
                  m['last_input'] is not None and m['w_line'] - m['last_input'] <= m['wd'] + 1),
        criterion('watchdog_disarmed', m['disarm_tick'], 'DISARMED at the W tick', m['disarm_tick'] is not None and
                  m['w_line'] is not None and m['disarm_tick'] == m['w_line']),
        criterion('executor_end_reason', end_reason, 'a link stop reason', end_reason is not None and
                  str(end_reason).startswith(('link_', 'bridge_', 'mcu_', 'steering_feedback'))),
    ]
    return _finish(ctx, 'link_watchdog', step0, before, criteria,
                   dict(start=start, mark=mark, measure=m, end_reason=end_reason, end_note=end_note,
                        client=client.get('state'), injection_wall=injection_wall), normal_until_tick=mark['tick'])


@script('fault_stop', 'potentiometer open (ADC 0) while the executor steers: MCU fault stop, steering output 0, drive '
        'brake; the executor ends with steering_feedback_invalid', 'A17: fault -> steering output 0 <= 1 tick, drive '
        'brake <= 1 tick; no automatic restart')
def s_fault_stop(ctx):
    before, step0 = step_start(ctx, 'exec_fault_start')
    spec = dict(kind='segments', end='stop', segments=[
        dict(duration_s=1., throttle=30, steer_deg=0., label='lead'),
        dict(duration_s=60., throttle=30, steer_deg=dict(fn='sine', amplitude_deg=6., period_s=4.), label='sine')])
    start = start_drive(ctx, spec)
    ctx.obs.spin_sim_for(2.5)
    mark = ctx.mark('exec_fault')
    reply = ctx.hooks.event(kind='adc_set', arg=0)
    injection_wall = time.time()
    ctx.obs.spin_sim_for(1.5)
    ctx.hooks.event(kind='adc_model')
    ctx.obs.spin_sim_for(1.)
    emu, _ = ctx.logs(mark)
    t = reply['at_tick']
    line = first(entries(emu, 'm2h'), lambda e: e['tick'] >= t and e['text'].startswith('F '))
    off = pin_first(emu, 'steer_en', lambda v: v == 0, t)
    zero = drive_zero_tick(emu, t)
    restart = restart_after(emu, zero)
    end_reason, end_note, client = _end_reason(ctx)
    criteria = [
        criterion('steer_output_zero_ticks_after_fault', None if (line is None or off is None) else
                  off['tick'] - line['tick'], '<= 1', line is not None and off is not None and
                  0 <= off['tick'] - line['tick'] <= 1),
        criterion('drive_brake_at_fault', None if (zero is None or line is None) else zero - line['tick'], '<= 1',
                  zero is not None and line is not None and 0 <= zero - line['tick'] <= 1),
        criterion('no_automatic_restart', len(restart), 0, not restart),
        criterion('executor_end_reason', end_reason, 'steering_feedback_invalid', end_reason ==
                  'steering_feedback_invalid'),
    ]
    return _finish(ctx, 'fault_stop', step0, before, criteria,
                   dict(start=start, mark=mark, trigger_tick=t, f_line=None if line is None else line['text'],
                        end_reason=end_reason, end_note=end_note, client=client.get('state'),
                        injection_wall=injection_wall), normal_until_tick=mark['tick'])


DRIVE_LOOKBACK_S = 1.0          # executor drives come at send_hz 10 Hz (every 0.1 s); the mark window (0.05 s) is shorter


def last_executor_drive(mark_wall, injection_wall, lookback_s=DRIVE_LOOKBACK_S, entries=None):
    """The executor's last op drive request (bridge event log) before the injection. ctx.logs() keeps the bridge
    entries from 0.05 s before the mark only, shorter than the executor's send period (send_hz 10 Hz = 0.1 s): the
    last drive before the injection was often outside that window (ST-05 A17(G) 2nd run: value null twice). Here the
    log is read again from lookback_s before the mark. entries: the bridge log entries (default: read BRIDGE_LOG)."""
    if entries is None:
        entries = read_jsonl(BRIDGE_LOG)
    found = [e for e in entries if e.get('kind') == 'request' and e.get('op') == 'drive' and
             str(e.get('owner') or '').startswith('row-executor') and
             mark_wall - lookback_s <= e.get('wall', 0.) < injection_wall]
    return found[-1] if found else None


STOP_ANSWER_S = .06            # an op stop is written by the bridge loop at its next tick (loop 20 ms; 3 periods)


def stop_writes(bridge, after_t, answer_s=STOP_ANSWER_S):
    """STOP lines the bridge wrote at or after after_t, split into the ones an op stop request caused and the bridge's
    own resends (its loop, on the 50 ms grid from the first STOP). An op stop only sets the bridge core's emergency
    event and is answered at once; the loop writes STOP at its next tick (steer_bridge_core.submit). So the first STOP
    line within answer_s after each op stop request (not yet given to an earlier request) is that request's; two
    requests inside one loop period make one STOP (the second finds none of its own). The executor page's STOP sends
    op stop twice by design: emergency_stop() at once from the HTTP thread, then the main loop's _handle_stop()
    latches its client with one more (ST-05 A17(G) 2nd run: a STOP 20.2 ms after the first was counted as a 6th
    resend; gsim_final w0_a17: requests at -14.0 and +21.7 ms, STOP lines at 0 and 37.4 ms). Returns
    (request_writes, resends, stop_request_times)."""
    requests = sorted(e['t'] for e in bridge if e.get('kind') == 'request' and e.get('op') == 'stop' and
                      e.get('t') is not None)
    tx = [e for e in bridge if e['kind'] == 'tx' and str(e.get('head') or '').startswith('STOP') and
          e['t'] >= after_t]
    taken = set()
    for r in requests:
        for i, e in enumerate(tx):
            if i not in taken and r <= e['t'] <= r + answer_s:
                taken.add(i)
                break
    by_request = [e for i, e in enumerate(tx) if i in taken]
    resends = [e for i, e in enumerate(tx) if i not in taken]
    return by_request, resends, [r for r in requests if r >= after_t]


def driving_precondition(status):
    """Is the executor driving just before an executor-side injection (runner ACTIVE)? The outage scripts measure the
    stop that the injection causes; a run that had already paused (gsim_final 2026-11-13 w0_a17: stale_lidar 1.9 s and
    3.0 s before the injection under host load, drive already 0) leaves nothing to measure: the step is invalid, not
    failed. status: the executor hook status (None: unknown -> not met)."""
    runner = (status or {}).get('runner') or {}
    state, reason = runner.get('state'), runner.get('reason')
    met = state == STATE_ACTIVE
    return dict(met=met, runner_state=state, runner_reason=reason,
                reason=None if met else 'the run was not driving at the injection (%s/%s)' % (state, reason))


def _executor_outage(ctx, name, inject, observe_s=4., expect_watchdog_event=False):
    """Common part of the executor-side outages (loop stall, process freeze): the run, the injection, the
    measurement of the bridge's input deadline / the executor's stall watchdog, no restart."""
    before, step0 = step_start(ctx, 'exec_%s_start' % name)
    start = start_drive(ctx, drive_spec(40, 4.))
    ctx.obs.spin_sim_for(3.)
    precondition = driving_precondition(ctx.try_exec_status())
    mark = ctx.mark('exec_%s' % name)
    reply = inject()
    injection_wall = time.time()
    ctx.obs.spin_sim_for(observe_s)
    emu, bridge = ctx.logs(mark)
    zero = drive_zero_tick(emu, mark['tick'])
    restart = restart_after(emu, zero)
    still = ctx.wait_still(4.)
    records = executor_log(ctx.exec_dir, after_wall=mark['wall'])
    wd_events = executor_events(records, ('watchdog_stall',), after_wall=mark['wall'] - .01)
    states = [e for e in bridge if e.get('kind') == 'state']
    expired = first(states, lambda e: e.get('reason') == 'input_expired')
    last_request = last_executor_drive(mark['wall'], injection_wall)
    status = ctx.try_exec_status() or {}
    runner = status.get('runner') or {}
    criteria = [
        criterion('drive_zero_and_vehicle_still', dict(drive_zero_tick=zero, still=bool(still)), 'drive 0, still',
                  zero is not None and bool(still)),
        criterion('no_automatic_restart', len(restart), 0, not restart),
    ]
    if expect_watchdog_event:
        gap = wd_events[0].get('loop_gap_s') if wd_events else None
        criteria.append(criterion('executor_stall_watchdog', None if not wd_events else dict(
            loop_gap_s=gap, drive_zero_sent=wd_events[0].get('drive_zero_sent')), 'watchdog_stall event with '
            'drive(0, held steering) (stall_guard_s 0.35 s; report the gap)', bool(wd_events)))
    details = dict(start=start, mark=mark, hook=reply, drive_zero_tick=zero, injection_wall=injection_wall,
                   input_expired=None if expired is None else dict(t=expired.get('t'), wall=expired.get('wall')),
                   last_executor_drive=None if last_request is None else dict(t=last_request.get('t'),
                                                                               wall=last_request.get('wall'),
                                                                               op=last_request.get('op')),
                   runner=runner, client=(status.get('client') or {}).get('state'), watchdog_events=len(wd_events),
                   precondition=precondition)
    if expired is not None and last_request is not None:
        details['input_expired_after_last_request_s'] = round(expired['t'] - last_request['t'], 4)
    return step0, before, criteria, details


@script('executor_loop_stall', 'the executor main loop stops stepping for 1.5 s (SIM hook stall): the executor\'s stall '
        'watchdog thread sends drive(0, held steering) and pauses; no restart without a new press', 'A17: no automatic '
        'restart; the watchdog drive 0 (stall_guard_s 0.35 s)')
def s_executor_stall(ctx):
    step0, before, criteria, details = _executor_outage(
        ctx, 'loop_stall', lambda: ctx.exec_hooks.call('stall', duration_s=1.5), expect_watchdog_event=True)
    out = _finish(ctx, 'executor_loop_stall', step0, before, criteria, details, normal_until_tick=details['mark']['tick'])
    ctx.op.post('/api/end')
    return out


@script('executor_freeze', 'the whole executor process stops for 1.5 s (SIM hook freeze: SIGSTOP): the bridge input '
        'deadline (0.75 s) stops the vehicle; after the freeze the executor\'s late drive is refused and it ends the run '
        '(no restart)', 'A17: input deadline -> stop (report: last executor request to input_expired <= 0.75 s + 1 '
        'bridge tick); no automatic restart')
def s_executor_freeze(ctx):
    step0, before, criteria, details = _executor_outage(
        ctx, 'freeze', lambda: ctx.exec_hooks.call('freeze', duration_s=1.5), observe_s=4.)
    value = details.get('input_expired_after_last_request_s')
    criteria.append(criterion('input_expired_after_last_request_s', value, '<= 0.75 + 0.02 (bridge input deadline + 1 '
                              'bridge tick)', value is not None and value <= .75 + BRIDGE_TICK_S + 1e-6))
    end_reason, end_note, client = _end_reason(ctx, 6.)
    criteria.append(criterion('executor_end_reason', end_reason, 'a bridge stop reason (the lease expired)',
                              end_reason is not None and str(end_reason).startswith(('bridge_', 'link_'))))
    details.update(end_reason=end_reason, end_note=end_note)
    return _finish(ctx, 'executor_freeze', step0, before, criteria, details, normal_until_tick=details['mark']['tick'])


def _supervisor_signal(ctx, signal_name, resume_after_s=None):
    fields = dict(child='bridge', signal=signal_name)
    if resume_after_s is not None:
        fields['resume_after_s'] = resume_after_s
    return ctx.sup.call('signal', **fields)


@script('bridge_forced_kill', 'SIGKILL of the bridge wrapper while the executor drives (design 7.2.1 bridge forced '
        'kill, with the executor): the executor\'s requests fail and it ends the run; MCU C TTL then link watchdog; the '
        'supervisor restarts the bridge, its port open resets the MCU; no restart', 'A17: C TTL -> drive 0 <= ttl + 1 '
        'tick; watchdog <= wd + 1, DISARMED; no automatic restart')
def s_bridge_kill(ctx):
    before, step0 = step_start(ctx, 'exec_bridge_kill_start')
    start = start_drive(ctx, drive_spec(40, 4.))
    ctx.obs.spin_sim_for(3.)
    mark = ctx.mark('exec_bridge_kill')
    reply = _supervisor_signal(ctx, 'KILL')
    injection_wall = time.time()
    ctx.obs.spin_sim_for(5.)
    m = _outage_link_measure(ctx, mark)
    end_reason, end_note, client = _end_reason(ctx, 6.)
    sup = ctx.sup.call('status')
    bridge_child = first(sup.get('children') or [], lambda c: c.get('name') == 'bridge') or {}
    criteria = m['criteria'] + [
        criterion('executor_end_reason', end_reason, 'bridge_error (the socket is gone)', end_reason is not None and
                  str(end_reason).startswith(('bridge_', 'link_', 'mcu_'))),
        criterion('bridge_restarted_by_the_supervisor', bridge_child.get('starts'), '>= 2 starts, running',
                  (bridge_child.get('starts') or 0) >= 2 and bool(bridge_child.get('running'))),
    ]
    return _finish(ctx, 'bridge_forced_kill', step0, before, criteria,
                   dict(start=start, mark=mark, hook=reply, measure=m['values'], end_reason=end_reason,
                        end_note=end_note, client=client.get('state'), bridge_child=bridge_child,
                        injection_wall=injection_wall), normal_until_tick=mark['tick'])


def _outage_link_measure(ctx, mark):
    """TTL and watchdog after the host side went silent at mark (no hook window: the last C line before the silence)."""
    info, params = ctx.session_params()
    wd = int(params['wd']) if params else 500
    emu, _ = ctx.logs(mark)
    ttl_line = f_line(emu, 'TTL', mark['tick'])
    w_line = first(entries(emu, 'm2h'), lambda e: e['tick'] >= mark['tick'] and e['text'].startswith('W '))
    lines = c_lines(read_jsonl(EMULATOR_LOG), 0)
    lines = [c for c in lines if c.get('session') == mark['session'] and
             (ttl_line is None or c['last'] <= ttl_line['tick'])]
    last_c = lines[-1] if lines else None
    host = [e for e in read_jsonl(EMULATOR_LOG) if e.get('session') == mark['session'] and e['kind'] == 'h2m'
            and e.get('note') == 'host' and e.get('last') is not None and
            (w_line is None or e['last'] <= w_line['tick']) and e['text'].startswith(('C ', 'PING'))]
    last_input = host[-1] if host else None
    zero = drive_zero_tick(emu, mark['tick'])
    ttl_expected = None if last_c is None else last_c['last'] + last_c['c']['ttl']
    disarm = pin_first(emu, 'mcu_state', lambda v: v == 1, mark['tick'])
    restart = restart_after(emu, zero)
    boots = [e['tick'] for e in entries(emu, 'm2h') if e['text'].startswith('UBTS_STEER_V3 hello')]
    criteria = [
        criterion('ttl_drive_zero_tick_minus_expected', None if (zero is None or ttl_expected is None) else
                  zero - ttl_expected, '0..1 (<= ttl + 1 tick)', zero is not None and ttl_expected is not None and
                  0 <= zero - ttl_expected <= 1),
        criterion('watchdog_tick_minus_last_input', None if (w_line is None or last_input is None) else
                  w_line['tick'] - last_input['last'], '<= wd + 1 = %d' % (wd + 1), w_line is not None and
                  last_input is not None and w_line['tick'] - last_input['last'] <= wd + 1),
        criterion('watchdog_disarmed', None if disarm is None else disarm['tick'], 'DISARMED at the W tick',
                  disarm is not None and w_line is not None and disarm['tick'] == w_line['tick']),
        criterion('no_automatic_restart', len(restart), 0, not restart),
    ]
    return dict(criteria=criteria, values=dict(wd=wd, drive_zero_tick=zero, ttl_expected=ttl_expected,
                                               ttl_line=None if ttl_line is None else ttl_line['tick'],
                                               w_line=None if w_line is None else w_line['tick'],
                                               last_input=None if last_input is None else last_input['last'],
                                               disarm_tick=None if disarm is None else disarm['tick'],
                                               mcu_boots_after=boots))


@script('bridge_stall', 'the bridge wrapper process stops for 1.5 s (SIGSTOP, then SIGCONT) while the executor drives: '
        'MCU C TTL then link watchdog, DISARMED; no restart', 'A17: C TTL -> drive 0 <= ttl + 1 tick; watchdog <= wd + 1, '
        'DISARMED; no automatic restart')
def s_bridge_stall(ctx):
    before, step0 = step_start(ctx, 'exec_bridge_stall_start')
    start = start_drive(ctx, drive_spec(40, 4.))
    ctx.obs.spin_sim_for(3.)
    mark = ctx.mark('exec_bridge_stall')
    reply = _supervisor_signal(ctx, 'STOP', resume_after_s=1.5)
    injection_wall = time.time()
    ctx.obs.spin_sim_for(4.)
    m = _outage_link_measure(ctx, mark)
    end_reason, end_note, client = _end_reason(ctx, 6.)
    criteria = m['criteria'] + [criterion('executor_end_reason', end_reason, 'a link or bridge stop reason',
                                          end_reason is not None and str(end_reason).startswith(
                                              ('bridge_', 'link_', 'mcu_', 'steering_feedback')))]
    return _finish(ctx, 'bridge_stall', step0, before, criteria,
                   dict(start=start, mark=mark, hook=reply, measure=m['values'], end_reason=end_reason,
                        end_note=end_note, client=client.get('state'), injection_wall=injection_wall),
                   normal_until_tick=mark['tick'])


def _latch_script(ctx, name, inject, expect_reason, expect_latch=None, observe_s=2.5, run_continues=False,
                  allow_continue=False):
    """A host-side fault while the executor drives: the expected executor end reason (or the run goes on), no
    restart."""
    before, step0 = step_start(ctx, 'exec_%s_start' % name)
    start = start_drive(ctx, drive_spec(30, 3.))
    ctx.obs.spin_sim_for(2.)
    mark = ctx.mark('exec_%s' % name)
    reply = inject()
    injection_wall = time.time()
    ctx.obs.spin_sim_for(observe_s)
    emu, bridge = ctx.logs(mark)
    status = ctx.try_exec_status() or {}
    runner = status.get('runner') or {}
    host = ctx.host()
    latches = sorted((host.get('steering_host') or {}).get('latches') or {})
    refusals = [e['text'][:40] for e in entries(emu, 'm2h') if e['text'].startswith('N ')]
    zero = drive_zero_tick(emu, mark['tick'])
    restart = restart_after(emu, zero)
    criteria = []
    if run_continues:
        criteria.append(criterion('run_continues', [runner.get('state'), runner.get('reason')], 'ACTIVE (no stop)',
                                  runner.get('state') == STATE_ACTIVE))
    elif allow_continue and runner.get('state') == STATE_ACTIVE:
        # the MCU may take the half frame with the next line (N - crc, still armed) or refuse it (c_rejected): both
        # are safe reactions (P6A_HEADLESS_KO 9.9.1); the run goes on in the first case
        criteria.append(criterion('run_continues_or_ends_with_a_link_reason', [runner.get('state'), runner.get('reason')],
                                  'ACTIVE, or the run ends with ' + ' or '.join(expect_reason), True))
    else:
        end_reason, end_note, client = _end_reason(ctx, 4.)
        allowed = expect_reason if isinstance(expect_reason, tuple) else (expect_reason,)
        criteria += [criterion('executor_end_reason', end_reason, ' or '.join(allowed), end_reason in allowed),
                     criterion('no_automatic_restart', len(restart), 0, not restart)]
        runner = dict(runner, end_reason=end_reason, end_note=end_note)
    if expect_latch:
        criteria.append(criterion('host_latch', latches, expect_latch, expect_latch in latches))
    out = _finish(ctx, name, step0, before, criteria,
                  dict(start=start, mark=mark, hook=reply, runner=runner, host_latches=latches,
                       mcu_refusals=refusals[:5], drive_zero_tick=zero, injection_wall=injection_wall),
                  normal_until_tick=mark['tick'])
    if run_continues or runner.get('state') == STATE_ACTIVE:
        ctx.op.post('/api/end')
    return out


@script('steering_status_loss', 'MCU status lines lost for 0.4 s while the executor drives: host steering_status_stale, '
        'the executor ends with steering_feedback_stale, no restart', 'A17 mandatory: status line loss; no automatic '
        'restart')
def s_status_loss(ctx):
    return _latch_script(ctx, 'steering_status_loss', lambda: ctx.hooks.status_loss(duration_ms=400),
                         ('steering_feedback_stale', 'bridge_latched', 'bridge_error'), 'steering_status_stale')


@script('stop_unconfirmed', 'operator STOP on the executor page while the MCU status lines are lost for 1.5 s: bounded '
        'STOP resends on the 50 ms grid, stop_unconfirmed latch; the executor watches the STOP confirmation',
        'A17 with amendment 7: resends <= 5, each at the first loop tick after its grid time, latch at t0 + 400 ms')
def s_stop_unconfirmed(ctx):
    before, step0 = step_start(ctx, 'exec_stop_unconfirmed_start')
    start = start_drive(ctx, drive_spec(30, 0.))
    ctx.obs.spin_sim_for(2.)
    mark = ctx.mark('exec_stop_unconfirmed')
    ctx.hooks.status_loss(duration_ms=1500)
    code, reply, http_wall, _ = ctx.op.post('/api/stop')
    ctx.obs.spin_sim_for(2.5)
    emu, bridge = ctx.logs(mark)
    req = first(bridge, lambda e: e['kind'] == 'request' and e.get('op') == 'stop')
    stops = [e for e in bridge if e['kind'] == 'tx' and e['head'].startswith('STOP') and req and e['t'] >= req['t']]
    t0 = stops[0]['t'] if stops else None
    # resends = the bridge's own STOP lines; a STOP written by a further op stop (the executor page sends two, see
    # stop_writes) is not a resend. Both lists are kept in the details (all_stop_writes_ms is the old count).
    by_request, own_resends, stop_requests = stop_writes(bridge, req['t']) if req else ([], [], [])
    all_writes = [round((e['t'] - t0) * 1000., 1) for e in stops[1:]] if t0 is not None else []
    stops = [stops[0]] + [e for e in own_resends if e['t'] > stops[0]['t']] if stops else []
    resend = [round((e['t'] - t0) * 1000., 1) for e in stops[1:]] if t0 is not None else []
    latch = first(bridge, lambda e: e['kind'] == 'state' and 'stop_unconfirmed' in e.get('latches', []))
    latch_ms = None if (latch is None or t0 is None) else round((latch['t'] - t0) * 1000., 1)
    grid = [100 + 50 * k for k in range(5)]
    tol = 1e-6 if ctx.clock == 'sim' else 1e-3
    ticks = sorted(e['t'] for e in bridge if e['kind'] == 'tick' and t0 is not None and e['t'] >= t0 - .05)
    resend_ticks = []
    for k, e in enumerate(stops[1:6]):
        ok, before_t = first_tick_after(ticks, t0 + grid[k] / 1000., e['t'], tol)
        resend_ticks.append(dict(k=k, grid_ms=grid[k], sent_ms=resend[k], first_tick_after_grid=ok))
    latch_ok = False if (latch is None or t0 is None) else first_tick_after(ticks, t0 + .4, latch['t'], tol)[0]
    on_grid = len(resend) == 5 and len(ticks) > 5 and all(r['first_tick_after_grid'] for r in resend_ticks)
    records = executor_log(ctx.exec_dir, after_wall=mark['wall'])
    watch = executor_events(records, ('stop_unconfirmed', 'stop_confirmed', 'stop_watch_timeout'),
                            after_wall=mark['wall'] - .01)
    criteria = [criterion('resend_count', len(resend), 5, len(resend) == 5),
                criterion('resends_at_the_first_loop_tick_after_the_50ms_grid', resend, grid, on_grid,
                          'amendment 7 (bridge tick log: %d ticks)' % len(ticks)),
                criterion('latch_stop_unconfirmed_ms', latch_ms, 'first loop tick at or after 400', latch_ok),
                criterion('executor_stop_watch', [e.get('event') for e in watch], 'report (the executor watches the '
                          'STOP confirmation: stop_unconfirmed or stop_watch_timeout)', None)]
    return _finish(ctx, 'stop_unconfirmed', step0, before, criteria,
                   dict(start=start, mark=mark, http_code=code, resend_ms=resend, resend_ticks=resend_ticks,
                        latch_ms=latch_ms, loop_ticks_logged=len(ticks), injection_wall=http_wall,
                        all_stop_writes_ms=all_writes,
                        stop_requests_ms=[None if t0 is None else round((t - t0) * 1000., 1) for t in stop_requests],
                        stop_writes_by_request=len(by_request)),
                   normal_until_tick=mark['tick'])


@script('second_screen_and_cli', 'while the executor drives: a second screen tries to arm and to drive (refused), a CLI '
        'STOP is accepted; the executor\'s next drive comes after the STOP and is refused (09-19 late drive after '
        'STOP); it latches and ends the run; no restart', 'A17 mandatory: two screens and CLI; late drive after STOP; '
        'no automatic restart')
def s_second_screen(ctx):
    before, step0 = step_start(ctx, 'exec_second_screen_start')
    start = start_drive(ctx, drive_spec(30, 3.))
    ctx.obs.spin_sim_for(2.)
    mark = ctx.mark('exec_second_screen')
    other = ControlClient(owner='exec-check-second-screen')
    arm = other.arm()
    other.lease, other.ticket, other.seq = 'not-the-lease', 'x', 0
    drive = other.drive(30, 0)
    cli = ControlClient(owner='exec-check-cli')
    stop = cli.stop('brake')
    injection_wall = time.time()
    ctx.obs.spin_sim_for(2.5)
    emu, bridge = ctx.logs(mark)
    records = executor_log(ctx.exec_dir, after_wall=mark['wall'])
    errors = [e.get('reason') for e in executor_events(records, ('bridge_error',), after_wall=injection_wall - .5)]
    end_reason, end_note, client = _end_reason(ctx, 4.)
    zero = drive_zero_tick(emu, mark['tick'])
    restart = restart_after(emu, zero)
    criteria = [criterion('second_screen_arm', arm.get('error'), 'control_in_use', arm.get('error') == 'control_in_use'),
                criterion('second_screen_drive', drive.get('error'), 'refused (control_not_owned)',
                          not drive.get('ok')),
                criterion('cli_stop_accepted', stop.get('ok'), True, stop.get('ok') is True),
                criterion('executor_late_drive_refused', errors[:3], 'a refusal (stop_barrier / control_not_owned)',
                          bool(errors) and any(('stop_barrier' in str(e)) or ('control_not_owned' in str(e)) or
                                               ('not_owned' in str(e)) for e in errors)),
                criterion('executor_end_reason', end_reason, 'bridge_error or bridge_latched',
                          str(end_reason).startswith('bridge_')),
                criterion('no_automatic_restart', len(restart), 0, not restart)]
    return _finish(ctx, 'second_screen_and_cli', step0, before, criteria,
                   dict(start=start, mark=mark, end_reason=end_reason, end_note=end_note, client=client.get('state'),
                        injection_wall=injection_wall), normal_until_tick=mark['tick'])


@script('mcu_reset', 'MCU reset while the executor drives: BOOT banner, host mcu_reset_detected, the executor ends with '
        'mcu_reset; no restart', 'A17 mandatory: MCU reset; no automatic restart')
def s_mcu_reset(ctx):
    return _latch_script(ctx, 'mcu_reset', lambda: ctx.hooks.event(kind='mcu_reset', arg=4),
                         ('mcu_reset', 'bridge_latched', 'bridge_error'), 'mcu_reset_detected', observe_s=3.)


@script('usb_unplug', 'USB unplug and replug (2.5 s, MCU power cycle) while the executor drives: hardware_fault, the '
        'executor ends; no restart', 'A17 mandatory: USB unplug / replug (pty reopen); no automatic restart')
def s_usb_unplug(ctx):
    return _latch_script(ctx, 'usb_unplug', lambda: ctx.hooks.unplug(duration_ms=2500, mcu_power='cycle'),
                         ('link_hardware_fault', 'bridge_latched', 'bridge_error', 'mcu_reset'), None, observe_s=5.)


@script('half_frame', 'a half frame (no LF) to the MCU while the executor drives', 'A17 mandatory: half frame (report the '
        'MCU and host reaction); no automatic restart when the run ends')
def s_half_frame(ctx):
    return _latch_script(ctx, 'half_frame', lambda: ctx.hooks.half_frame(),
                         ('link_command_rejected', 'link_bad_lines', 'bridge_latched', 'bridge_error'), None,
                         allow_continue=True)


@script('crc_one_line', 'one C line with a bad CRC while the executor drives: the MCU refuses the line and stays armed; '
        'the run goes on', 'A17 mandatory: CRC error (one line); the run continues')
def s_crc_one(ctx):
    return _latch_script(ctx, 'crc_one_line', lambda: ctx.hooks.corrupt(kind='crc', count=1), None, None,
                         run_continues=True)


@script('crc_burst', 'five C lines with a bad CRC while the executor drives: MCU CRC fault, DISARMED, the executor ends',
        'A17 mandatory: CRC errors (burst); no automatic restart')
def s_crc_burst(ctx):
    return _latch_script(ctx, 'crc_burst', lambda: ctx.hooks.corrupt(kind='crc', count=5),
                         ('link_bad_lines', 'link_command_rejected', 'bridge_latched', 'bridge_error'), None)


@script('range_error', 'one C line with a value out of range while the executor drives: MCU refusal, host c_rejected, '
        'the executor ends', 'A17 mandatory: range error; no automatic restart')
def s_range_error(ctx):
    return _latch_script(ctx, 'range_error', lambda: ctx.hooks.corrupt(kind='range', count=1),
                         ('link_command_rejected', 'bridge_latched', 'bridge_error'), 'c_rejected')


@script('output_guard', 'the scripted controller commands a raw steering step above the standstill slew limit: the '
        'executor\'s SteerOutputGuard ends the run (guard_violation, op stop at once, latched); the violating target '
        'never reaches the MCU', 'A17: the executor output guard (design 6.2, 6.6); no automatic restart')
def s_output_guard(ctx):
    before, step0 = step_start(ctx, 'exec_guard_start')
    spec = dict(kind='segments', end='stop', segments=[
        dict(duration_s=1.5, throttle=0, steer_deg=0., label='settle'),
        dict(duration_s=3., throttle=0, steer_deg=20., raw=True, label='raw_step')])
    start = start_drive(ctx, spec, moving=False)
    ctx.obs.spin_sim_for(3.)
    mark = ctx.mark('exec_guard')
    emu, bridge = ctx.logs(step0)
    end_reason, end_note, client = _end_reason(ctx, 4.)
    sequence = s_sequence(c_lines(emu, step0['tick']))
    away = moves_away_from(sequence, 0)
    criteria = [criterion('executor_end_reason', end_reason, 'guard_violation:steer_rate_above_limit',
                          str(end_reason).startswith('guard_violation')),
                criterion('violating_target_not_sent', sequence, 'the S values of the C lines (permille, in order) only '
                          'step toward the settle steering 0 (the run starts from the steering the previous script '
                          'left) and never away from it: the guard held the raw step', not away and
                          (not sequence or sequence[-1] == 0)),
                criterion('executor_client_latched', client.get('state'), 'LATCHED', client.get('state') ==
                          CLIENT_LATCHED)]
    return _finish(ctx, 'output_guard', step0, before, criteria,
                   dict(start=start, mark=mark, end_reason=end_reason, end_note=end_note), normal_until_tick=None)


@script('system_time_change', 'the executor\'s wall clock jumps +1 h while it drives (SIM hook wall_offset; its deadlines '
        'are on the monotonic or SIM clock): the run goes on, no stop and no latch; a STOP afterwards works', 'A17 '
        'mandatory: system time change (executor side; the bridge clock is not changed in the container: no '
        'CAP_SYS_TIME)')
def s_time_change(ctx):
    before, step0 = step_start(ctx, 'exec_time_change_start')
    start = start_drive(ctx, drive_spec(30, 3.))
    ctx.obs.spin_sim_for(1.5)
    mark = ctx.mark('exec_time_change')
    ctx.exec_hooks.call('wall_offset', seconds=3600.)
    ctx.obs.spin_sim_for(3.)
    status = ctx.try_exec_status() or {}
    runner = status.get('runner') or {}
    host = ctx.host()
    latches = sorted((host.get('steering_host') or {}).get('latches') or {})
    code, reply, http_wall, _ = ctx.op.post('/api/stop')
    ctx.obs.spin_sim_for(1.5)
    confirmed = ctx.wait_host(lambda s: (s.get('steering_host') or {}).get('stop_state') == 'confirmed', 2.)
    ctx.exec_hooks.call('wall_offset', seconds=0.)
    criteria = [criterion('run_continues_after_the_jump', [runner.get('state'), runner.get('reason')], 'ACTIVE',
                          runner.get('state') == STATE_ACTIVE),
                criterion('no_host_latch', latches, [], not latches),
                criterion('stop_after_the_jump_confirmed', bool(confirmed), True, bool(confirmed))]
    return _finish(ctx, 'system_time_change', step0, before, criteria,
                   dict(start=start, mark=mark, wall_offset_s=3600., injection_wall=http_wall),
                   normal_until_tick=None)


@script('executor_forced_kill', 'SIGKILL of the executor process while it drives (design 7.2.1 dashboard / controller '
        'forced kill with the executor as the controlling client): the bridge input deadline stops the vehicle; no '
        'restart; the container stays down (restart no)', 'A17 mandatory: forced kill of the controlling process; no '
        'automatic restart (run last: the executor is gone after it)')
def s_executor_kill(ctx):
    step0, before, criteria, details = _executor_outage(ctx, 'kill', lambda: ctx.exec_hooks.call('kill'), observe_s=4.)
    value = details.get('input_expired_after_last_request_s')
    criteria.append(criterion('input_expired_after_last_request_s', value, '<= 0.75 + 0.02', value is not None and
                              value <= .75 + BRIDGE_TICK_S + 1e-6))
    alive = ctx.try_exec_status() is not None
    criteria.append(criterion('executor_process_gone', not alive, True, not alive))
    ctx.executor_alive = False
    return _finish(ctx, 'executor_forced_kill', step0, before, criteria, details,
                   normal_until_tick=details['mark']['tick'])


DEFAULT_ORDER = ('operator_stop', 'deadman_release', 'hold_lost', 'c_ttl_link_returns', 'link_watchdog', 'fault_stop',
                 'executor_loop_stall', 'executor_freeze', 'steering_status_loss', 'stop_unconfirmed',
                 'second_screen_and_cli', 'mcu_reset', 'usb_unplug', 'half_frame', 'crc_one_line', 'crc_burst',
                 'range_error', 'output_guard', 'system_time_change', 'bridge_stall', 'bridge_forced_kill',
                 'executor_forced_kill')


def check_safety(ctx, args):
    by_name = {s['name']: s for s in SCRIPTS}
    order = [n for n in DEFAULT_ORDER if args.only is None or n in args.only]
    results, errors = [], []
    for name in order:
        entry = by_name[name]
        repeats = args.repeat if name == 'operator_stop' else 1
        for index in range(repeats):
            if not ctx.executor_alive:
                break
            started = time.monotonic()
            try:
                result = entry['fn'](ctx, index) if name == 'operator_stop' else entry['fn'](ctx)
            except Exception as exc:
                import traceback
                result = dict(name=name if repeats == 1 else '%s_%d' % (name, index), passed=False, error='%s: %s' % (
                    type(exc).__name__, exc), traceback=traceback.format_exc()[-3000:],
                    criteria=[criterion('no_exception', '%s: %s' % (type(exc).__name__, exc), None, False)])
                errors.append(result['name'])
                try:
                    ensure_idle(ctx)
                except Exception:
                    pass
            result.update(script=name, purpose=entry['purpose'], registered=entry['registered'],
                          wall_s=round(time.monotonic() - started, 2))
            results.append(result)
            write_json(ctx.out / 'scripts' / ('%s.json' % result['name']), result)
    stops = [r for r in results if r['script'] == 'operator_stop']
    valid_stops = [r for r in stops if (r.get('validity') or {}).get('valid', True) and not r.get('error')]
    criteria = [dict(c, script=r['name']) for r in results for c in r.get('criteria', [])]
    blocks = [r['false_detection'] for r in results if r.get('false_detection')]
    false = merge_false_detections(blocks)
    false['rule'] = 'drive-up windows of every script (step start to the injection) and the whole normal scripts ' \
                    '(operator STOP, deadman release, hold lost)'
    normal_s = sum(float(b.get('normal_sim_s') or 0.) for b in blocks)
    by = false.get('by_code') or {}
    false.update(normal_sim_s=round(normal_s, 2),
                 steering_ineffective=dict(count=by.get('host:steering_ineffective', 0),
                                           per_normal_sim_hour=None if not normal_s else
                                           round(by.get('host:steering_ineffective', 0) * 3600. / normal_s, 3)),
                 stall=dict(count=by.get('STALL', 0), per_normal_sim_hour=None if not normal_s else
                            round(by.get('STALL', 0) * 3600. / normal_s, 3)))
    end_status = ctx.try_exec_status() or {}
    motion = dict(executor_motion_sent=(end_status.get('counts') or {}).get('motion_sent'),
                  note='op motion samples of the executor (gyro yaw rate, LiDAR speed with valid odometry) for the '
                       'bridge steering_ineffective monitor; A19(G) reports its false-detection rate')
    return dict(check='A17_G_executor', clock=ctx.clock, criteria=criteria, passed=verdict(criteria),
                scripts=[dict(name=r['name'], passed=r.get('passed'), valid=(r.get('validity') or {}).get('valid'),
                              error=r.get('error'), wall_s=r.get('wall_s')) for r in results],
                stops_valid=len(valid_stops), stops=len(stops), errors=errors, false_detection=false, motion=motion,
                invalid_scripts=[r['name'] for r in results if (r.get('validity') or {}).get('valid') is False],
                validity_rule='A10 / amendment 10 in the wall clock mode; SIM clock steps are in SIM time',
                not_run=dict(system_time_change_bridge='the bridge and the emulator clocks are not changed (no '
                                                       'CAP_SYS_TIME in the containers); the executor side only'))


# --- A13 -------------------------------------------------------------------------------------------------------------------
def check_graph(ctx, args):
    scene = os.environ.get('UBTS_STEER_SCENE')
    expected = tuple(expected_nodes(scene)) + (S2_NODE,) + ((EXECUTOR_NODE,) if not args.no_executor else ())
    graph = stable_graph(ctx, expected=expected, timeout_s=args.discovery_s)
    write_json(ctx.out / 'ROS_GRAPH.json', graph)
    (ctx.out / 'GRAPH_LISTING.txt').write_bytes(graph_listing(graph).encode('utf-8'))
    isolation = p6a_graph.check_truth_isolation(graph)
    executor = judge_executor_subscriptions(graph, ctx.clock)
    s2_input = '/sim/scan_clipped' if scene == 'field1010' else '/sim/scan'
    s2 = judge_s2_subscriptions(graph, s2_input)
    observer = p6a_graph.TRUTH_CONSUMERS
    criteria = [criterion('discovery_complete', graph.get('missing_nodes', []), [], graph['discovery_complete']),
                criterion('rules_R1_R5_no_findings', len(isolation['findings']), 0, isolation['passed']),
                criterion('executor_node_present', executor['present'], True, executor['present']),
                criterion('executor_subscriptions_R6_amendment_11', executor['subscribed'], executor['permitted'],
                          executor['passed']),
                criterion('executor_reads_no_truth_or_forbidden_topic', executor['truth_or_forbidden'], [],
                          not executor['truth_or_forbidden']),
                criterion('executor_clock_only_in_sim_clock_mode', executor['clock_in_wall_mode'], False,
                          not executor['clock_in_wall_mode']),
                criterion('executor_subscribes_its_inputs', executor['expected_not_subscribed'], [],
                          not executor['expected_not_subscribed']),
                criterion('s2_node_reads_only_its_input_and_publishes_scan', s2.get('subscribed'), [s2_input],
                          s2['passed']),
                criterion('scan_publisher_is_the_s2_node', s2['scan_publishers'], [S2_NODE],
                          [_base(n) for n in s2['scan_publishers']] == [S2_NODE]),
                criterion('registered_r6_without_amendment', executor['registered_without_amendment'], 'report',
                          None, 'p6a_graph.check_executor_subscriptions (infrastructure topics not exempt)')]
    return dict(check='A13_G_graph_executor', clock=ctx.clock, scene=scene, criteria=criteria,
                passed=verdict(criteria), isolation=isolation, executor=executor, s2_node=s2, nodes=graph['nodes'],
                topic_count=len(graph['topics']), truth_consumers_allowed=list(observer))


# --- A5 ---------------------------------------------------------------------------------------------------------------------
class TopicRecorder:
    """Receipts and payloads of a few topics on the observer node (evaluation side)."""

    def __init__(self, obs, keep_ranges=200):
        self.obs = obs
        self.keep_ranges = int(keep_ranges)
        self.rows = collections.defaultdict(list)       # topic -> [(mono, stamp_ns, fields...)]
        self.ranges = collections.defaultdict(dict)     # topic -> {stamp_ns: ranges array}
        self.subscriptions = []

    def scan(self, topic, keep=True):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        import numpy as np

        def callback(msg, topic=topic):
            mono = time.monotonic()
            stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            self.rows[topic].append((mono, stamp, len(msg.ranges), float(msg.angle_min), float(msg.angle_increment),
                                     msg.header.frame_id, float(msg.range_min), float(msg.range_max)))
            if keep and len(self.ranges[topic]) < self.keep_ranges:
                self.ranges[topic][stamp] = np.array(msg.ranges, dtype=np.float32)
        self.subscriptions.append(self.obs.node.create_subscription(LaserScan, topic, callback,
                                                                    qos_profile_sensor_data))

    def receipts(self, topic):
        out = {}
        for row in self.rows.get(topic, []):
            out.setdefault(row[1], row[0])
        return out


def check_scan(ctx, args):
    """A5(G) on /scan (module docstring)."""
    import numpy as np
    from .vehicle_profile import load_profile
    from .scan_unobserved_clip import lidar_mount
    profile = load_profile(args.profile)
    offset = float(profile.value('lidar_yaw_offset_deg'))
    scene = os.environ.get('UBTS_STEER_SCENE')
    source = '/sim/scan_clipped' if scene == 'field1010' else '/sim/scan'
    rec = TopicRecorder(ctx.obs, keep_ranges=args.keep)
    rec.scan('/scan')
    rec.scan(source)
    s2_before = load_json_or_none('/runtime/s2_node_status.json')
    ctx.obs.spin_sim_for(args.seconds)
    s2_after = load_json_or_none('/runtime/s2_node_status.json')
    rows = rec.rows.get('/scan', [])
    problems = {}
    for row in rows:
        stamp = row[1]
        found = scan_format_problems(row[2], row[3], row[4], row[5], row[6], row[7],
                                     rec.ranges['/scan'].get(stamp, np.zeros(S2.BEAMS)))
        for p in found:
            problems[p] = problems.get(p, 0) + 1
    real = S2.RealFormat(offset)
    compared = mismatched = 0
    for stamp, out in rec.ranges['/scan'].items():
        src = rec.ranges[source].get(stamp)
        if src is None:
            continue
        compared += 1
        if real.convert(src).tobytes() != out.astype(np.float32).tobytes():
            mismatched += 1
    zeros = [int(np.sum(r == 0.)) for r in rec.ranges['/scan'].values()]
    rate = stamp_rate([r[1] for r in rows])
    wall = wall_rate([r[0] for r in rows])
    rtf = rtf_summary(ctx.obs.clock_rows)
    criteria = [criterion('scan_messages', len(rows), '> 0', bool(rows)),
                criterion('format_items_3240_angles_frame_limits_no_negative_or_nonfinite', problems or 'none', 'none',
                          bool(rows) and not problems),
                criterion('no_return_is_0', min(zeros) if zeros else None, 'beams without a return are 0.0 (min count '
                          'per scan > 0)', bool(zeros) and min(zeros) > 0),
                criterion('scan_rate_sim_hz', rate, '10 Hz (header stamps; median step 0.1 s, no gap)',
                          rate['sim_hz'] is not None and abs(rate['sim_hz'] - S2.SCAN_HZ) <= RATE_TOL * S2.SCAN_HZ and
                          rate['gaps'] == 0),
                criterion('scan_rate_wall_hz', wall, 'report (receipts; RTF median %s)' % rtf.get('median'), None),
                criterion('raw_order_equals_the_f_sim_conversion', dict(compared=compared, mismatched=mismatched),
                          'every /scan = RealFormat(lidar_yaw_offset_deg) of the SIM scan of the same stamp',
                          compared > 0 and mismatched == 0)]
    s2 = None
    if s2_after:
        counts = s2_after.get('counts') or {}
        s2 = dict(counts=counts, rejected_by_reason=s2_after.get('rejected_by_reason'),
                  callback_ms=s2_after.get('callback_ms'), conversion_ms=s2_after.get('conversion_ms'),
                  run_token=s2_after.get('run_token'), lidar_yaw_offset_deg=s2_after.get('lidar_yaw_offset_deg'))
        criteria.append(criterion('s2_node_counters', dict(scans_in=counts.get('scans_in'), scans_out=counts.get(
            'scans_out'), rejected=counts.get('rejected'), stamp_gaps=counts.get('stamp_gaps')),
            'in = out, rejected 0, stamp gaps 0 (no silent drop)', counts.get('scans_in') == counts.get('scans_out')
            and counts.get('rejected') == 0 and counts.get('stamp_gaps') == 0))
        criteria.append(criterion('s2_node_yaw_offset_deg', s2_after.get('lidar_yaw_offset_deg'), offset,
                                  abs(float(s2_after.get('lidar_yaw_offset_deg') or 0.) - offset) < 1e-9))
    # yaw-offset convention against geometry (W0 pillars, the truth pose at the scan stamp; evaluation)
    geometry = None
    if scene in ('w0_force', None, ''):
        geometry = yaw_convention_check(ctx, rec, profile, offset, lidar_mount())
        criteria.append(criterion('yaw_offset_convention_pillars', geometry.get('summary'), 'every visible W0 pillar at '
                                  'its raw beam and true range (|error| <= 0.10 m, >= 3 pillars)',
                                  geometry.get('passed')))
    noise = standstill_noise(rec, ctx)
    criteria.append(criterion('range_noise_sd_m_at_standstill', noise, 'report: profile lidar_noise_sd_m %.3f; the G-SIM '
                              'LiDAR noise is rover.yaml 0.01 (WORLDS_KO 5, a known difference)'
                              % float(profile.value('lidar_noise_sd_m')), None))
    return dict(check='A5_G_scan', scene=scene, source_topic=source, criteria=criteria, passed=verdict(criteria),
                scans=len(rows), rate=rate, wall_rate=wall, rtf=rtf, s2_node=s2, s2_status_before=bool(s2_before),
                geometry=geometry)


def load_json_or_none(path):
    try:
        return load_json(path)
    except (OSError, ValueError):
        return None


def yaw_convention_check(ctx, rec, profile, offset, mount):
    """Each visible W0 pillar at the raw beam the lidar_yaw_offset_deg convention gives, at the true range (truth pose at
    the scan stamp from /ground_truth/odom, evaluation only)."""
    try:
        manifest = load_json('/runtime/steer/scene/scene_manifest.json')
    except (OSError, ValueError) as exc:
        return dict(passed=None, note='no scene manifest: %s' % exc)
    summary = manifest.get('world_summary') or {}
    pillars, radius = summary.get('pillars_xy_m'), summary.get('pillar_radius_m')
    if not pillars:
        return dict(passed=None, note='scene without W0 pillars')
    odom = list(ctx.obs.odom)
    checks = []
    for stamp, ranges in list(rec.ranges['/scan'].items())[:40]:
        t = stamp / 1e9
        near = min(odom, key=lambda r: abs(r[0] - t)) if odom else None
        if near is None or abs(near[0] - t) > .02 or abs(near[4]) > .01:
            continue                                  # a standing vehicle and a pose within 20 ms of the stamp
        for item in pillar_expectations((near[1], near[2], near[3]), mount[:2], pillars, radius, offset):
            j = item['raw_beam']
            got = float(ranges[j])
            checks.append(dict(item, measured_m=round(got, 4), error_m=round(got - item['expected_m'], 4)))
    if not checks:
        return dict(passed=None, note='no standing scan with a pose at its stamp')
    worst = max(abs(c['error_m']) for c in checks)
    pillars_seen = len({tuple(c['pillar']) for c in checks})
    return dict(passed=worst <= .10 and pillars_seen >= 3, summary=dict(pillars=pillars_seen, checks=len(checks),
                                                                         worst_error_m=round(worst, 4)),
                examples=checks[:12])


def standstill_noise(rec, ctx):
    import numpy as np
    block = np.array([r for r in rec.ranges['/scan'].values()], dtype=float)
    if block.shape[0] < 10:
        return None
    good = np.all(block > 0., axis=0) & (np.ptp(block, axis=0) < .2)
    if not good.any():
        return None
    sd = np.std(block[:, good], axis=0, ddof=1)
    return round(float(np.sqrt(max(0., float(np.median(sd)) ** 2 - (.001 ** 2) / 12.))), 5)


def check_imu(ctx, args):
    """A5(G) /imu/data_raw at standstill."""
    import numpy as np
    from .vehicle_profile import load_profile
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    profile = load_profile(args.profile)
    rows = []

    def callback(msg):
        g, a = msg.angular_velocity, msg.linear_acceleration
        rows.append((time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, g.x, g.y, g.z, a.x, a.y,
                     a.z, msg.header.frame_id, msg.orientation_covariance[0]))
    ctx.obs.node.create_subscription(Imu, '/imu/data_raw', callback, qos_profile_sensor_data)
    ctx.obs.spin_sim_for(args.seconds)
    status = load_json_or_none(SIM_STATUS) or {}
    model = status.get('imu_model') or {}
    criteria = []
    moving = [r for r in ctx.obs.odom if abs(r[4]) > STILL_SPEED_M_S]
    data = np.array([r[1:8] for r in rows], dtype=float) if rows else np.zeros((0, 7))
    rate = None
    if len(data) > 2:
        rate = (len(data) - 1) / (data[-1, 0] - data[0, 0])
    expected_rate = float(profile.value('imu_rate_hz'))
    criteria.append(criterion('imu_rate_hz', None if rate is None else round(rate, 3), '%s +-10 %%' % expected_rate,
                              within(rate, expected_rate, RATE_TOL)))
    criteria.append(criterion('imu_rate_within_1_percent', None if rate is None else round(rate, 3),
                              'report (A5(F) used +-1 %% on the T1 clock)', None))
    frames = sorted({r[8] for r in rows})
    criteria.append(criterion('imu_frame_and_orientation_not_provided', [frames, sorted({r[9] for r in rows})],
                              [['imu_link'], [-1.0]], frames == ['imu_link'] and {r[9] for r in rows} == {-1.}))
    out = dict(check='A5_G_imu', samples=len(rows), moving_samples=len(moving), model=model,
               scene=os.environ.get('UBTS_STEER_SCENE'))
    if model and len(data) > 30:
        expected = imu_expected_noise(model)
        g = data[:, 1:4]
        a = data[:, 4:7]
        gsd = [float(np.std(g[:, k], ddof=1)) for k in range(3)]
        asd = [float(np.std(a[:, k], ddof=1)) for k in range(3)]
        out.update(expected=expected, gyro_sd=gsd, accel_sd=asd)
        for k, axis in enumerate('xyz'):
            criteria.append(criterion('gyro_%s_noise_sd_rad_s' % axis, round(gsd[k], 6), '%.6f +-10 %% (IMU model of '
                                      'this run)' % expected['gyro_sd_rad_s'], within(gsd[k], expected['gyro_sd_rad_s'],
                                                                                    NOISE_TOL)))
        for k, axis in enumerate('xyz'):
            criteria.append(criterion('accel_%s_noise_sd_m_s2' % axis, round(asd[k], 5), '%.5f +-10 %% (vibration / '
                                      'sqrt 3)' % expected['accel_sd_m_s2'], within(asd[k], expected['accel_sd_m_s2'],
                                                                                   NOISE_TOL)))
    else:
        criteria.append(criterion('imu_model_in_simulation_status', bool(model), True, bool(model)))
    criteria.append(criterion('vehicle_still_during_the_record', len(moving), 0, not moving))
    out.update(criteria=criteria, passed=verdict(criteria))
    return out


# --- display (camera, virtual GPS) ---------------------------------------------------------------------------------------
def ws_first_messages(url, count=3, timeout_s=8., ack=None):
    """The first messages of a WebSocket stream (the websockets library of the image). ack(message) -> a reply to send
    or None (the GPS stream waits for an ack of each snapshot)."""
    import asyncio
    import websockets

    async def run():
        out = []
        async with websockets.connect(url, max_size=8 * 1024 * 1024, open_timeout=timeout_s) as ws:
            end = time.monotonic() + timeout_s
            while len(out) < count and time.monotonic() < end:
                raw = await asyncio.wait_for(ws.recv(), timeout=max(.1, end - time.monotonic()))
                out.append(raw)
                reply = ack(raw) if ack is not None else None
                if reply is not None:
                    await ws.send(reply)
        return out
    return asyncio.run(run())


def check_display(ctx, args):
    """A5(G) camera and virtual GPS shown (display only)."""
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, NavSatFix
    images, fixes = [], []
    ctx.obs.node.create_subscription(Image, '/camera/image_raw', lambda m: images.append(
        (time.monotonic(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.width, m.height, m.encoding)),
        qos_profile_sensor_data)
    ctx.obs.node.create_subscription(NavSatFix, '/gps/fix', lambda m: fixes.append((m.latitude, m.longitude)),
                                     qos_profile_sensor_data)
    ctx.obs.spin_sim_for(args.seconds)
    criteria = []
    sizes = sorted({(r[2], r[3]) for r in images})
    rate = None
    if len(images) > 2 and images[-1][1] > images[0][1]:
        rate = (len(images) - 1) / (images[-1][1] - images[0][1])
    criteria.append(criterion('camera_topic_640x360', sizes, [[640, 360]], sizes == [(640, 360)]))
    criteria.append(criterion('camera_topic_rate_sim_hz', None if rate is None else round(rate, 2), '30 +-10 %',
                              within(rate, 30., RATE_TOL)))
    frame = None
    try:
        messages = ws_first_messages(CAMERA_WS, count=6)
        for raw in messages:
            parsed = parse_camera_message(raw)
            if parsed is not None and parsed[1]:
                frame = dict(meta={k: parsed[0].get(k) for k in ('width', 'height', 'source_width', 'source_height',
                                                                 'frame_count', 'source_transport')},
                             jpeg_size=jpeg_size(parsed[1]), jpeg_bytes=len(parsed[1]))
                break
    except Exception as exc:              # report the failure as the value
        frame = dict(error='%s: %s' % (type(exc).__name__, exc))
    criteria.append(criterion('camera_websocket_frame_decoded', frame, 'a JPEG frame of the dashboard camera stream, '
                              '640x360', bool(frame) and frame.get('jpeg_size') == (640, 360)))
    fix = fixes[-1] if fixes else None
    criteria.append(criterion('virtual_gps_fix_near_0N_0E', fix, '|lat|, |lon| < 0.01 deg', fix is not None and
                              abs(fix[0]) < .01 and abs(fix[1]) < .01))
    gps = None
    try:
        def ack(raw):
            try:
                seq = json.loads(raw).get('stream_seq')
            except (TypeError, ValueError, AttributeError):
                return None
            return json.dumps({'ack': seq}) if seq is not None else None
        messages = ws_first_messages(GPS_WS, count=2, ack=ack)
        snapshot = json.loads(messages[-1]) if messages else {}
        gps = dict(keys=sorted(snapshot)[:30], latitude=snapshot.get('latitude', snapshot.get('lat')),
                   longitude=snapshot.get('longitude', snapshot.get('lon')), fresh=snapshot.get('fresh'))
    except Exception as exc:
        gps = dict(error='%s: %s' % (type(exc).__name__, exc))
    criteria.append(criterion('gps_websocket_snapshot', gps, 'a GPS snapshot of the dashboard GPS stream (display only)',
                              bool(gps) and 'error' not in gps))
    return dict(check='A5_G_display', criteria=criteria, passed=verdict(criteria), camera_messages=len(images),
                gps_fixes=len(fixes), camera_frame=frame, gps=gps, scene=os.environ.get('UBTS_STEER_SCENE'),
                note='display only: neither the camera nor the GPS is an executor input (A13)')


# --- W2 sensing chain --------------------------------------------------------------------------------------------------
def check_chain(ctx, args):
    """The W2 sensing chain as ROS nodes (development values; module docstring)."""
    stages = ['/sim/scan', '/sim/scan_clipped', '/scan']
    rec = TopicRecorder(ctx.obs, keep_ranges=0)
    for topic in stages:
        rec.scan(topic, keep=False)
    status0 = {name: load_json_or_none('/runtime/%s' % name) for name in (
        'cpu_lidar_status.json', 'scan_clip_status.json', 's2_node_status.json')}
    started = time.monotonic()
    ctx.obs.spin_sim_for(args.seconds, timeout_s=args.seconds * 4. + 60.)
    wall_s = time.monotonic() - started
    status1 = {name: load_json_or_none('/runtime/%s' % name) for name in status0}
    receipts = {topic: rec.receipts(topic) for topic in stages}
    clock_rows = list(ctx.obs.clock_rows)
    latency = chain_latency(receipts, clock_rows, stages)
    rates = {topic: dict(stamp_rate(list(receipts[topic])), wall_hz=wall_rate(list(receipts[topic].values())))
             for topic in stages}
    rtf = rtf_summary(clock_rows)

    def delta(name, *keys):
        a, b = status0.get(name) or {}, status1.get(name) or {}
        for key in keys[:-1]:
            a, b = a.get(key) or {}, b.get(key) or {}
        k = keys[-1]
        if a.get(k) is None or b.get(k) is None:
            return None
        return b[k] - a[k]
    node_counts = dict(
        cpu_lidar=dict(published=delta('cpu_lidar_status.json', 'published_scans'),
                       rate_skips=delta('cpu_lidar_status.json', 'rate_samples_skipped'),
                       last_raycast_ms=(status1.get('cpu_lidar_status.json') or {}).get('last_raycast_ms')),
        clip_node=dict(scans=delta('scan_clip_status.json', 'scans'), published=delta('scan_clip_status.json',
                                                                                      'published'),
                       unmatched=delta('scan_clip_status.json', 'unmatched'),
                       interpolated=delta('scan_clip_status.json', 'interpolated'),
                       mean_ms=(status1.get('scan_clip_status.json') or {}).get('mean_ms')),
        s2_node=dict(scans_in=delta('s2_node_status.json', 'counts', 'scans_in'),
                     scans_out=delta('s2_node_status.json', 'counts', 'scans_out'),
                     rejected=delta('s2_node_status.json', 'counts', 'rejected'),
                     stamp_gaps=delta('s2_node_status.json', 'counts', 'stamp_gaps'),
                     missing_scans=delta('s2_node_status.json', 'counts', 'missing_scans'),
                     callback_ms=(status1.get('s2_node_status.json') or {}).get('callback_ms')))
    scan_rate = rates['/scan']['sim_hz']
    criteria = [criterion('scan_rate_sim_hz', rates['/scan'], 'report (development; 10 Hz = every SIM scan reached '
                          '/scan)', None),
                criterion('scan_missing_per_stage', [s['missing_from_previous'] for s in latency['stages']],
                          'report (scans seen at a stage and not at the next; this node\'s best effort receipt also '
                          'counts)', None),
                criterion('chain_total_latency_ms', latency['total_ms'], 'report (SIM clock at the stamp to the /scan '
                          'receipt, wall)', None),
                criterion('node_counters', node_counts, 'report (cpu_lidar published, clip node published / unmatched, '
                          'S2 node in / out / rejected / stamp gaps)', None),
                criterion('chain_keeps_10_hz', scan_rate, 'report: >= 9.5 Hz of SIM time with no S2 stamp gap',
                          None)]
    return dict(check='W2_chain', scene=os.environ.get('UBTS_STEER_SCENE'), criteria=criteria, passed=None,
                stages=stages, latency=latency, rates=rates, rtf=rtf, wall_s=round(wall_s, 2), node_counts=node_counts,
                status_end=status1, note='development values (amendment 9: the judged A10(W2) is measured in the quiet '
                                         'qualification); this observer subscribes /sim/scan and /sim/scan_clipped '
                                         '(evaluation node, run apart from the A13 graph capture)')


# --- drive (development) ----------------------------------------------------------------------------------------------
def check_drive(ctx, args):
    spec = json.loads(args.controller) if args.controller else dict(kind='straight_hold', drive_pct=40,
                                                                   distance_m=args.distance)
    before, step0 = step_start(ctx, 'exec_drive_start')
    start = start_drive(ctx, spec, moving=True)
    t0 = ctx.obs.sim_time
    pose0 = ctx.obs.last_pose()
    ended = ctx.wait_exec(lambda s: s['runner']['state'] == STATE_ENDED, args.seconds * 3. + 30.)
    ctx.wait_still(10.)
    pose1 = ctx.obs.last_pose()
    status = ctx.try_exec_status() or {}
    records = executor_log(ctx.exec_dir, after_wall=step0['wall'])
    ticks = executor_ticks(records)
    accepted = sum(1 for t in ticks if (t.get('scan') or {}).get('accepted'))
    offsets = [(t.get('scan') or {}).get('header_offset_s') for t in ticks]
    offsets = [o for o in offsets if isinstance(o, (int, float))]
    corridor = sum(1 for t in ticks if t.get('corridor'))
    run = dict(start=start, ended=bool(ended), runner=status.get('runner'), sim_s=round(ctx.obs.sim_time - t0, 2),
               travel_m=None if not (pose0 and pose1) else round(math.hypot(pose1[1] - pose0[1], pose1[2] - pose0[2]),
                                                                  3),
               ticks=len(ticks), scans_accepted=accepted, corridor_valid_ticks=corridor,
               header_offset_s=percentiles(offsets), counts=status.get('counts'))
    ctx.op.post('/api/end')
    after = emulator_counters(ctx.hooks)
    return dict(check='executor_drive', clock=ctx.clock, run=run, emulator_clock=counter_delta(before, after),
                passed=None, note='development run of the executor with a scripted controller (not the P8 row '
                                  'controller)')


# --- G-SIM row following (steer_follow_gsim_v1) -----------------------------------------------------------------------
SCENE_DIR = Path(os.environ.get('UBTS_STEER_SCENE_DIR', '/runtime/steer/scene'))


def check_pose(ctx, args):
    """The truth pose now (evaluation side; used to confirm a pose reset)."""
    ctx.obs.spin_until(lambda: bool(ctx.obs.odom), 5.)
    ctx.obs.spin_for(.5)
    pose = ctx.obs.last_pose()
    return dict(check='pose', pose=None if pose is None else [round(float(v), 4) for v in pose], passed=None)


def _run_records(ctx, step0, run_id):
    """Ticks and the run_end event of this run only. The log folder (control volume) keeps the files of earlier
    sessions and their runs have the same run ids, so only the records after this run's start event are used
    (registration steer_follow_gsim_v1, amendment 1)."""
    from . import gsim_follow as GF
    records = executor_log(ctx.exec_dir, after_wall=step0['wall'])
    starts = [i for i, r in enumerate(records) if r.get('k') == 'event' and r.get('event') == 'start'
              and r.get('run_id') == run_id and r.get('wall', 0.) >= step0['wall'] - 1.]
    own = records[starts[-1]:] if starts else records
    ticks = GF.own_ticks([r for r in executor_ticks(own) if r.get('run') == run_id])
    ends = [r for r in executor_events(own, ('run_end',), after_wall=step0['wall'] - 1.)
            if r.get('run_id') == run_id]
    return ticks, (ends[-1] if ends else None)


def check_follow(ctx, args):
    """One closed-loop run of the row-following controller through the executor, judged on the evaluation side against
    the registration steer_follow_gsim_v1 (steering.gsim_follow). The check is the operator: Start, Arm, one press held
    by heartbeats until the run ends; no second press."""
    from . import gsim_follow as GF
    from . import exec_run
    spec = json.loads(args.controller) if args.controller else dict(GF.CONTROLLER)
    run = next((dict(r) for r in GF.RUNS if r['name'] == args.run), None)
    if run is None:
        raise SessionError('unknown run %r (registered: %s)' % (args.run, ', '.join(r['name'] for r in GF.RUNS)))
    metadata = load_json(SCENE_DIR / 'world_metadata.json')
    manifest = load_json(SCENE_DIR / 'scene_manifest.json')
    geometry = GF.row_geometry(metadata)
    ctx.obs.spin_until(lambda: bool(ctx.obs.odom) and ctx.obs.sim_time is not None, 10.)
    ensure_idle(ctx)
    placed = None
    if run['start'] == 'set_pose':
        along, _, _ = GF.to_row(geometry, geometry['spawn'][0], geometry['spawn'][1], geometry['spawn_yaw'])
        x, y, yaw = GF.from_row(geometry, along, run.get('lateral_offset_m', 0.), 0.)
        z = exec_run.spawn_height(manifest, metadata) + exec_run.OFFSET_DROP_M
        placed = dict(exec_run.set_model_pose(x, y, z, yaw), x=round(x, 4), y=round(y, 4), z=round(z, 4), yaw=yaw)
        ctx.obs.spin_for(2.)
        ctx.wait_still(10.)
    before, step0 = step_start(ctx, 'gsim_follow_' + run['name'])
    ctx.obs.spin_for(.3)
    t0 = ctx.obs.sim_time
    pose0 = ctx.obs.last_pose()
    error, start, timeline, ended = None, None, [], None
    try:
        start = start_drive(ctx, spec, moving=False, timeout_s=30.)
    except (SessionError, ClientError, OSError) as exc:
        error = '%s: %s' % (type(exc).__name__, exc)
    limit = time.monotonic() + (args.limit_s or GF.RUN_LIMIT_S)
    paused_since, last = None, None
    while error is None and time.monotonic() < limit:
        ctx.obs.spin_for(.2)
        status = ctx.try_exec_status()
        if status is None:
            continue
        runner = status['runner']
        key = (runner.get('state'), runner.get('reason'))
        if key != last:
            timeline.append(dict(sim_s=ctx.obs.sim_time, state=key[0], reason=key[1]))
            last = key
        if runner.get('state') == STATE_ENDED:
            ended = dict(runner)
            break
        if runner.get('state') == STATE_PAUSED:
            paused_since = paused_since or time.monotonic()
            if time.monotonic() - paused_since > GF.PAUSE_GIVE_UP_S:
                break
        else:
            paused_since = None
    gave_up = ended is None and error is None
    ctx.hold.release()
    if gave_up:
        ctx.op.post('/api/end')
    ctx.wait_still(15.)
    ctx.obs.spin_for(GF.CRITERIA['after_terminal_window_s'] + .7)
    status = ctx.try_exec_status() or {}
    runner = status.get('runner') or {}
    t1 = ctx.obs.sim_time
    mono1 = time.monotonic()
    odom = [[round(float(v), 5) for v in row] for row in odom_rows(ctx.obs, t0, t1)]
    truth = truth_rows(ctx.obs, t0, t1)
    duty = [(r['sim_time_s'], max(abs(r.get('drive_duty_left') or 0.), abs(r.get('drive_duty_right') or 0.))
             if r.get('drive_en') else 0.) for r in truth]
    ticks, run_end = _run_records(ctx, step0, runner.get('run_id'))
    emu, _ = ctx.logs(step0, wait=False)
    lines = [(mark_sim_of_tick(step0, e['last']), e['c']['S']) for e in c_lines(emu, after_tick=step0['tick'])]
    t_end = None
    if run_end is not None:
        t_end = run_end.get('t') if ctx.clock == 'sim' else interpolate_sim(list(ctx.obs.clock_rows), run_end.get('t'))
    metrics = GF.measure(odom, duty, geometry, settle_m=run.get('settle_m', 0.))
    stats = GF.tick_stats(ticks)
    metrics.update(end_reason=runner.get('end_reason') if run_end is None else run_end.get('end_reason'),
                   final_state=runner.get('state'), final_reason=runner.get('reason'), ended_at_s=t_end,
                   pauses_after_start=sorted({str(item[1]) for item in stats['pauses_after_start']}),
                   after_terminal=GF.after_terminal(lines, t_end, t1))
    checks = GF.judge(run['expect'], metrics)
    after = emulator_counters(ctx.hooks)
    sim_made = step_sim_latches(ctx, step0, ctx.clock) if ctx.clock == 'wall' else ()
    validity = step_validity(ctx, ctx.clock, step0['mono'], mono1, before, after, sim_made=sim_made)
    verdict_text = GF.verdict(checks, validity['valid'])
    counts = status.get('counts') or {}
    steer = [[round(r['sim_time_s'], 3), round(math.degrees(r['delta_eq_rad']), 3)] for r in truth[::6]]
    trace = dict(schema=GF.SCHEMA + '.trace', result_kind=RESULT_KIND, run=run['name'],
                 odom_columns=['sim_s', 'x_m', 'y_m', 'yaw_rad', 'v_m_s', 'w_rad_s'], odom=odom[::3],
                 duty=[[round(t, 3), round(v, 4)] for t, v in duty[::6]], steer_eq_deg=steer,
                 ticks=[dict(t=r.get('t'), state=r.get('state'), reason=r.get('reason'), ctl=r.get('ctl'),
                             corridor=r.get('corridor'), corridor_reason=r.get('corridor_reason'), lat=r.get('lat'),
                             head=r.get('head'), width=r.get('width'), end=r.get('end'), front=r.get('front'),
                             dist=r.get('dist'), v=r.get('v'), T=r.get('T'), S=r.get('S'),
                             lidar_age=r.get('lidar_age')) for r in ticks],
                 geometry=dict((k, geometry[k]) for k in ('lane', 'rows', 'centre_a', 'centre_b', 'axis_rad',
                                                          'half_width_m', 'row_start_s', 'row_end_s', 'markers',
                                                          'rocks', 'spawn', 'spawn_yaw')))
    write_json(Path(ctx.out) / 'GSIM_FOLLOW_TRACE.json', trace)
    criteria = [criterion(item['check'], item['measured'], item['limit'], item['ok'] if validity['valid'] else None)
                for item in checks]
    return dict(check='gsim_follow', prereg_id=GF.PREREG_ID, run=run, controller=spec, clock=ctx.clock,
                verdict=verdict_text, passed=None if verdict_text == 'invalid' else verdict_text == 'pass',
                criteria=criteria, checks=checks, validity=validity, metrics=metrics, tick_stats=stats,
                start=start, start_error=error, placed=placed, gave_up_paused=gave_up, timeline=timeline,
                run_end=run_end, pose_start=None if pose0 is None else [round(float(v), 4) for v in pose0],
                geometry=dict(lane=geometry['lane'], rows=geometry['rows'], axis_deg=math.degrees(geometry['axis_rad']),
                              centre_a_m=geometry['centre_a'], half_width_m=geometry['half_width_m'],
                              row_start_s_m=geometry['row_start_s'], row_end_s_m=geometry['row_end_s'],
                              trunks=geometry['trunks'], rocks_in_lane=len(geometry['rocks'])),
                counts=dict((k, counts.get(k)) for k in ('scans_processed', 'scans_dropped', 'missed_ticks',
                                                         'loop_exceptions', 'watchdog_fires', 'bridge_errors',
                                                         'emergency_stops', 'sends')),
                hold_replies=dict(ctx.hold.replies), samples=dict(odom=len(odom), truth=len(truth), c_lines=len(lines)))


# --- main -----------------------------------------------------------------------------------------------------------------
CHECKS = dict(graph=check_graph, scan=check_scan, imu=check_imu, display=check_display, chain=check_chain,
              safety=check_safety, drive=check_drive, follow=check_follow, pose=check_pose)


def build_parser():
    p = argparse.ArgumentParser(prog='exec_checks', description=__doc__.splitlines()[0])
    p.add_argument('check', choices=sorted(CHECKS))
    p.add_argument('--out', required=True)
    p.add_argument('--clock', choices=('wall', 'sim'), default=os.environ.get('UBTS_STEER_CLOCK', 'wall') or 'wall')
    p.add_argument('--profile', default=None)
    p.add_argument('--seconds', type=float, default=20.)
    p.add_argument('--keep', type=int, default=200, help='scan: SIM scans kept for the byte comparison')
    p.add_argument('--repeat', type=int, default=5, help='safety: operator STOP repeats')
    p.add_argument('--only', nargs='*', default=None, help='safety: script names')
    p.add_argument('--no-executor', action='store_true', help='graph: the executor is not running')
    p.add_argument('--discovery-s', type=float, default=45.)
    p.add_argument('--controller', default=None, help='drive: scripted controller spec JSON')
    p.add_argument('--distance', type=float, default=6.)
    p.add_argument('--run', default='nominal_1', help='follow: a registered run name (gsim_follow.RUNS)')
    p.add_argument('--limit-s', type=float, default=None, help='follow: wall limit of the run')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    out = Path(args.out)
    ctx = ExecCtx(out, clock=args.clock)
    started = time.monotonic()
    counters0 = emulator_counters(ctx.hooks)
    report = dict(schema=SCHEMA, result_kind=RESULT_KIND, field_validated=False, check=args.check,
                  started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                  scene=os.environ.get('UBTS_STEER_SCENE'), clock=args.clock,
                  source_commit=os.environ.get('UBTS_STEER_SOURCE_COMMIT'),
                  source_status_sha256=os.environ.get('UBTS_STEER_SOURCE_STATUS_SHA256'), provenance=provenance(),
                  provenance_dirs_note='sim/ubts, dashboard/scripts, firmware of /opt/ubts (tools/row_executor is '
                                       'the executor container\'s; see its config.json source_check)')
    try:
        report['executor_config'] = load_json_or_none(Path(EXEC_DIR) / 'config.json')
        result = CHECKS[args.check](ctx, args)
        report.update(result)
    except Exception as exc:
        import traceback
        report.update(error='%s: %s' % (type(exc).__name__, exc), traceback=traceback.format_exc()[-4000:],
                      passed=False)
    finally:
        try:
            report['emulator_clock'] = counter_delta(counters0, emulator_counters(ctx.hooks))
            report['rtf'] = rtf_summary(ctx.obs.clock_rows)
            report['wall_s'] = round(time.monotonic() - started, 2)
            report['operator_requests'] = list(ctx.op.log)[-60:]
            report['executor_status_end'] = ctx.try_exec_status()
        finally:
            ctx.close()
    name = dict(graph='A13_G_EXEC.json', scan='A5_G_SCAN.json', imu='A5_G_IMU.json', display='A5_G_DISPLAY.json',
                chain='W2_CHAIN.json', safety='A17_G_EXEC.json', drive='EXEC_DRIVE.json',
                follow='GSIM_FOLLOW_RUN.json', pose='POSE.json')[args.check]
    write_json(out / name, report)
    failed = [c['name'] for c in report.get('criteria', []) if c.get('passed') is False]
    print(json.dumps(dict(check=args.check, passed=report.get('passed'), failed=failed[:40],
                          error=report.get('error'))))
    return 0 if report.get('passed') is not False else 1


if __name__ == '__main__':
    sys.exit(main())
