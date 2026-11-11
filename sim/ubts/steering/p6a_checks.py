"""P6a checks of the steering G-SIM stack (docs/steering/P6A_HEADLESS_KO.md; SIM_ACCEPTANCE A3, A9, A10, A13, A14,
A17, A19). Run inside the check service of compose.ubts-steer.yaml (joins the sim network and IPC, the control socket
/run/ubts-steer/control.sock, the emulator hooks /run/ubts-steer-emu/hooks.sock and the runtime volume):

  python3 -B -m steering.p6a_checks <check> --out /runs/<folder> [options]

  startup     A19 (G) false detections of the stack start (a normal script): every MCU fault line, new fault bit and
              host latch from the start of the logs to now (none expected but BOOT); run first after the health check
  graph       A13 (G graph part): every topic's publishers and subscribers, rules R1-R5 of p6a_graph, the executor
              allow-list check prepared (R6, next phase; judged as registered, infrastructure topics not exempt)
  identity    A14: the SIM wrapper refuses a sim=0 device (banner rewrite hook) and a device of another hardware
              profile (arm refused); the real-hardware transport refuses the SIM path and a pty slave made in the
              check container (no udev identity; with a forged udev record of another ID_SERIAL); the real-role link
              refuses a sim=1 banner; the old control screen (dashboard HTTP, old payloads) cannot arm or move; the
              ROS domains 0 / 42 do not see the domain 43 topics (run in the check container)
  safety      A17 (G): STOP latency chain (5 times; judged from the STOP request to the STOP bytes and from the STOP
              bytes to the host's confirmation), STOP unconfirmed (status lines lost: 5 resends, each at the first loop
              tick after its 50 ms grid time from the bridge's tick log, latch), C TTL and link watchdog (link loss to
              the MCU) with the lateral deviation against the analytic arc (held angle, speed at the loss, the rest of
              the C TTL, the stop mode), fault stop (steering output 0) and hold error, no automatic restart, and the
              mandatory fault tests that need no executor
  faults      A19 (G): each MCU fault code through the emulator hooks (judgement tick vs the spec rule +-1 tick,
              resulting state) and the host faults reachable without the executor. POT_NOISE, SOFTLIMIT, POLARITY,
              DRV_STUCK, STALL and THERMAL are judged against the rule tick that p6a_fw_rules computes from a record
              of the case (every tick's analogRead value and pins); TRACK's tick is an estimate (timing reported,
              not judged)
  a25         A25 (G, emulator path, amendment 2): the steering joint angles of /sim/truth/joint_states against the
              shared model's delta_L, delta_R of /sim/truth/steering at the same SIM time (p99, max, delay in steps,
              joint speed spikes) during a steering pattern at standstill and while driving, with the /clock gaps
  a3          A3 (G): record every FW_STEP of the emulator for N SIM seconds (standstill, a steering sweep while
              driving, an injected potentiometer fault), then replay the inputs in a fresh firmware backend (the
              F-SIM / T1 path) and compare every response; plus the standstill raw noise of the record
  drive       A9 / A10 (W1): one scripted open-loop drive (straight, then a constant steering angle, then STOP) with
              the final pose and lateral deviation from the truth (evaluation), and the RTF windows of /clock
  rtf         A10: the /clock rate against the wall clock (amendment 2: --seconds 240 with the first --warmup-s 20 s
              left out, 5 s windows; RTF source = this node's /clock receipt; the rendering path from the environment)
  summary9    A9: variance over the drive results of several runs (host-side: one stack start per run)
  a19g_normal A19 (G) normal-script false detections over result files (startup, verify, drive, safety, a25) and
              G-SIM lockstep results (gz_probe a25_firmware), with the STALL false-detection rate (host-side)

Every check result has emulator_clock (the emulator's /clock gap counters over the check: SIM gaps > 20 / 50 / 100 ms
= skipped /clock samples, wall gaps between callbacks) and rtf (the 5 s windows of /clock against this node's wall
clock). Time-sensitive steps (A17, A19 timing) carry a validity block: in the wall clock mode a step whose 5 s RTF is
below 0.9 or with an emulator wall gap above 100 ms is invalid (A10) and its criteria are not judged.
Times: SIM ticks of the emulator log (1 ms), the bridge event log (its injected clock: wall monotonic in --clock wall,
SIM time in --clock sim) and wall time of the check process. All values are placeholders: placeholder-based SIM
results (자리값 기반 SIM 결과), not field validation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time

from .p6a_client import ClientError, ControlClient, DriveLoop, HookClient, SimObserver, read_jsonl
from . import p6a_fw_rules
from . import p6a_graph

RESULT_KIND = '자리값 기반 SIM 결과'
SCHEMA = 'ubts.steer_p6a_check.v1'
EMULATOR_LOG = '/runtime/steer/emulator_log.jsonl'
BRIDGE_LOG = '/runtime/steer/bridge_events.jsonl'
STEERING_STATUS = '/runtime/steering_status.json'
SIM_STATUS = '/runtime/simulation_status.json'
BRIDGE_STATUS = '/runtime/rover_bridge_status.json'
IMAGE_ROOT = '/opt/ubts'
IMAGE_MANIFEST = IMAGE_ROOT + '/STEER_IMAGE_MANIFEST.json'
PROVENANCE_DIRS = ('sim/ubts', 'dashboard/scripts', 'firmware')
LOG_FLUSH_WAIT_S = .7                    # the emulator and the bridge flush their logs every 0.5 s wall
STATUS_PERIOD_MS = 50
BRIDGE_TICK_S = .02
STOP_CONFIRM_PERIODS = 2
STILL_SPEED_M_S = .005
RTF_VALID_MIN = .9                       # A10: below this the wall clock mode's time-sensitive results are invalid
EMULATOR_STALL_MS = 100                  # an emulator /clock wall gap above this marks a time-sensitive step invalid
RTF_WINDOW_S = 5.
CLOCK_STEP_S = 1. / 120.
EMULATOR_CLOCK_COUNTERS = ('clock_gaps_over_20ms', 'clock_gaps_over_50ms', 'clock_gaps_over_100ms',
                           'clock_wall_gaps_over_20ms', 'clock_wall_gaps_over_50ms', 'clock_wall_gaps_over_100ms')
FAULT_NAMES = ('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'SOFTLIMIT',
               'STALL', 'TRACK', 'THERMAL', 'TTL', 'WATCHDOG', 'CRC', 'BOOT')
CONDITION_BITS = ('SOFTLIMIT', 'TTL')    # status-line bits that follow a condition (no latch)


# --- small helpers --------------------------------------------------------------------------------------------------------
def provenance(root=IMAGE_ROOT, manifest_path=IMAGE_MANIFEST, dirs=PROVENANCE_DIRS):
    """The image manifest (Dockerfile.ubts-steer) and whether the files the checks run from are the image's own:
    every file under the dirs is hashed and compared with the manifest's list (sources_match_image false when sources
    are mounted over the image, as in the development runs)."""
    try:
        manifest = load_json(manifest_path)
    except (OSError, ValueError):
        return dict(manifest=None, sources_match_image=False)
    listed = {k: v for k, v in (manifest.get('files') or {}).items() if k.startswith(dirs)}
    now = {}
    base = Path(root)
    for folder in dirs:
        for path in sorted((base / folder).rglob('*')):
            rel = path.relative_to(base).as_posix()
            if path.is_file() and not rel.endswith(('.so', '.pyc')) and '__pycache__' not in rel:
                now[rel] = hashlib.sha256(path.read_bytes()).hexdigest()
    differ = sorted(k for k in listed if now.get(k) != listed[k])
    extra = sorted(k for k in now if k not in listed)
    fields = ('image_tag_hint', 'base_image_id', 'source_commit', 'source_status_sha256', 'source_files_sha256',
              'source_file_count', 'built_utc', 'firmware_build_id')
    return dict(manifest={k: manifest.get(k) for k in fields}, dirs=list(dirs), files_checked=len(now),
                differ_count=len(differ), differ=differ[:20], extra_count=len(extra), extra=extra[:20],
                sources_match_image=not differ and not extra and bool(now))


def write_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_bytes((json.dumps(payload, indent=1, allow_nan=False, default=str) + '\n').encode('utf-8'))
    os.replace(tmp, path)


def load_json(path):
    return json.loads(Path(path).read_bytes().decode('utf-8'))


def body(text):
    return text.rsplit('*', 1)[0] if '*' in text else text


def parse_status_line(text):
    parts = body(text).split(' ')
    if parts[0] != 'S' or len(parts) != 16:
        return None
    names = ('ms', 'n', 'st', 'q', 'L', 'R', 'en', 'raw', 'pos', 'tgt', 'cmd', 'sp', 'fl', 'wd', 'tl')
    return {name: (int(tok, 16) if name == 'fl' else int(tok)) for name, tok in zip(names, parts[1:])}


def parse_c_line(text):
    parts = body(text).split(' ')
    if parts[0] != 'C' or len(parts) != 6:
        return None
    return dict(q=int(parts[1]), ttl=int(parts[2]), L=int(parts[3]), R=int(parts[4]), S=int(parts[5]))


def criterion(name, value, limit, passed, note=None):
    out = dict(name=name, value=value, limit=limit, passed=None if passed is None else bool(passed))
    if note:
        out['note'] = note
    return out


def verdict(criteria):
    values = [c['passed'] for c in criteria if c['passed'] is not None]
    if not values:
        return None
    return all(values)


# --- emulator clock and A10 validity of time-sensitive steps ------------------------------------------------------------
def emulator_counters(hooks):
    """The emulator's /clock counters now (hook status), or None."""
    try:
        status = hooks.status()['status']
    except (OSError, ClientError, ValueError, KeyError):
        return None
    counters = status.get('counters') or {}
    out = {k: int(counters.get(k, 0)) for k in EMULATOR_CLOCK_COUNTERS}
    out.update(clock_messages=int(status.get('clock_messages') or 0), last_clock_s=status.get('last_clock_s'),
               clock_wall_gap_max_us=int(counters.get('clock_wall_gap_max_us', 0)), mono=time.monotonic())
    return out


def counter_delta(before, after, step_s=CLOCK_STEP_S):
    """/clock counters between two emulator_counters() snapshots: gap counts, messages received against the SIM
    time span (skipped samples), the wall time between the snapshots."""
    if not before or not after:
        return None
    out = {k: after[k] - before[k] for k in EMULATOR_CLOCK_COUNTERS}
    out['clock_messages'] = after['clock_messages'] - before['clock_messages']
    out['wall_s'] = round(after['mono'] - before['mono'], 3)
    if before.get('last_clock_s') is not None and after.get('last_clock_s') is not None:
        sim = after['last_clock_s'] - before['last_clock_s']
        out['sim_s'] = round(sim, 6)
        expected = int(round(sim / step_s))
        out['clock_messages_expected'] = expected
        out['clock_messages_missing'] = expected - out['clock_messages']
    out['clock_wall_gap_max_us_since_start'] = after['clock_wall_gap_max_us']
    return out


def rtf_between(clock_rows, mono0, mono1):
    """Delta SIM / delta wall of the /clock receipts between two monotonic times (None with too few)."""
    rows = [r for r in clock_rows if mono0 <= r[0] <= mono1]
    if len(rows) < 2 or rows[-1][0] - rows[0][0] < .5:
        return None
    return (rows[-1][1] - rows[0][1]) / (rows[-1][0] - rows[0][0])


def rtf_summary(clock_rows, warmup_s=0., window_s=RTF_WINDOW_S):
    windows = rtf_windows(list(clock_rows), warmup_s=warmup_s, window_s=window_s)
    ordered = sorted(windows)
    return dict(windows=len(windows), median=statistics.median(windows) if windows else None,
                min=ordered[0] if ordered else None, p10=ordered[len(ordered) // 10] if ordered else None,
                below_valid_min=sum(1 for w in windows if w < RTF_VALID_MIN), valid_min=RTF_VALID_MIN,
                window_s=window_s, source='/clock receipts of the check node against its monotonic clock')


def step_validity(ctx, clock, mono0, mono1, before, after, sim_made=()):
    """A10 validity of one time-sensitive step (wall clock mode): the /clock RTF over the step widened to a 5 s window,
    the emulator's /clock wall gaps in the step and the SIM-made host latches of the step (sim_made: a
    steering_status_stale with stall or slow /clock evidence, step_sim_latches; it stops the vehicle, so the step
    measures nothing). In the SIM clock mode the times are SIM times (valid)."""
    half = RTF_WINDOW_S / 2.
    mid = .5 * (mono0 + mono1)
    rtf = rtf_between(list(ctx.obs.clock_rows), min(mono0, mid - half), max(mono1, mid + half))
    delta = counter_delta(before, after)
    stalls = None if delta is None else delta['clock_wall_gaps_over_100ms']
    if clock == 'sim':
        valid, reason = True, 'SIM clock mode (times in SIM time)'
    elif rtf is None:
        valid, reason = False, 'no /clock samples in the window'
    elif rtf < RTF_VALID_MIN:
        valid, reason = False, 'RTF %.3f below %.1f (A10)' % (rtf, RTF_VALID_MIN)
    elif stalls:
        valid, reason = False, '%d emulator /clock wall gaps above %d ms' % (stalls, EMULATOR_STALL_MS)
    elif sim_made:
        valid, reason = False, '%d SIM-made host latch(es) in the step (stall or slow /clock)' % len(sim_made)
    else:
        valid, reason = True, 'RTF %.3f, no emulator wall gap above %d ms' % (rtf, EMULATOR_STALL_MS)
    return dict(valid=valid, reason=reason, rtf_5s=rtf, clock=clock, emulator_clock=delta,
                sim_made_latches=list(sim_made)[:5])


def step_start(ctx, label):
    """Counters and mark at the start of a time-sensitive step, before its ARM: the drive-up belongs to the step (a
    SIM stall there latches the host and stops the vehicle before the measured action; first fix run, A2 STOP 2)."""
    return emulator_counters(ctx.hooks), ctx.mark(label)


def step_sim_latches(ctx, start, clock):
    """The SIM-made host latches (stall or slow /clock evidence) since a step's start mark."""
    emu, _ = ctx.logs(start, wait=False)
    return false_detections(emu, read_jsonl(BRIDGE_LOG), after_tick=start['tick'], after_wall=start['wall'] - .01,
                            stalls=emulator_stalls(), slow=emulator_slow(), clock=clock)['sim_stall_latches']


def invalidate(criteria, validity):
    """Criteria of an invalid step are not judged (passed None, the value kept)."""
    if validity is None or validity['valid']:
        return criteria
    out = []
    for c in criteria:
        if c['passed'] is not None:
            c = dict(c, passed=None, invalid=True, note=(c.get('note', '') + ' [invalid: %s]' % validity['reason']
                                                          ).strip())
        out.append(c)
    return out


# --- A19 (G) false detections of normal scripts (R15-3) -------------------------------------------------------------------
def fault_code_of(text):
    """The fault code of an MCU line: 'F <code> ...' -> code, 'W ...' -> WATCHDOG, else None."""
    parts = body(text).split(' ')
    if parts[0] == 'F' and len(parts) > 1:
        return parts[1]
    if parts[0] == 'W':
        return 'WATCHDOG'
    return None


STALL_ATTRIBUTION_S = 1.                 # a host steering_status_stale within this of an emulator stall is SIM-made


def emulator_stalls():
    """The emulator's /clock wall stalls (wall end, gap) from the status file, or []."""
    try:
        stalls = load_json(STEERING_STATUS)['emulator'].get('clock_stalls') or []
    except (OSError, ValueError, KeyError):
        return []
    return [(float(s['wall_end']), float(s['wall_gap_ms']) / 1000.) for s in stalls]


def emulator_slow():
    """The emulator's slow /clock intervals (monotonic start, end or None while open, lowest SIM rate) from the
    status file, or []: SIM time received per monotonic second over 0.3 s below 0.9."""
    try:
        slow = load_json(STEERING_STATUS)['emulator'].get('clock_slow') or []
    except (OSError, ValueError, KeyError):
        return []
    return [(float(s['mono_start']), None if s.get('mono_end') is None else float(s['mono_end']),
             float(s['min_rate'])) for s in slow]


def false_detections(emu, bridge, expected=(), after_tick=None, after_wall=None, stalls=None, clock='wall',
                     slow=None):
    """Unexpected fault evidence of a normal script (R15-3: a normal script expects no fault code but BOOT; a false
    detection is a fault bit, F or W line or host latch that the script does not expect).

    emu: entries of the emulator log of the script's window; bridge: the whole bridge log (the latches held before
    after_wall are known, not new: a windowed log would show a latch held from before as new). expected: ((kind,
    name, tick0, tick1) ...) with kind 'mcu' (an F code, WATCHDOG or a status bit, ticks of the emulator log) or 'host' (a latch, wall
    times); tick1 None = to the end. stalls: the emulator's /clock wall stalls ((wall end, gap s), emulator_stalls()):
    in the wall clock mode a host steering_status_stale latch inside a stall (or within STALL_ATTRIBUTION_S after it)
    is the SIM's own wall-clock stall (A10: invalid), listed apart and not counted. slow: the emulator's slow /clock
    intervals ((monotonic start, end or None, lowest rate), emulator_slow()): the same for a latch whose bridge
    time (monotonic in the wall clock mode, same kernel clock as the emulator) falls in one (SIM time arrived slower
    than 0.9 x real time: A10). Returns dict(count, by_code, mcu_lines, new_bits, host_latches, expected_seen,
    sim_stall_latches)."""
    def allowed(kind, name, when):
        for k, n, a, b in expected:
            if k == kind and n == name and (a is None or when >= a) and (b is None or when <= b):
                return True
        return False
    mcu, bits, host, seen = [], [], [], []
    for e in entries(emu, 'm2h'):
        if after_tick is not None and e['tick'] < after_tick:
            continue
        code = fault_code_of(e['text'])
        if code is None:
            continue
        (seen if allowed('mcu', code, e['tick']) else mcu).append(dict(tick=e['tick'], code=code, text=e['text'][:48]))
    previous = None
    for e in status_lines(emu, after_tick or 0):
        fl = e['status']['fl']
        if previous is not None:
            for index, name in enumerate(FAULT_NAMES):
                if fl >> index & 1 and not previous >> index & 1 and name != 'BOOT':
                    row = dict(tick=e['tick'], bit=name)
                    (seen if allowed('mcu', name, e['tick']) else bits).append(row)
        previous = fl
    known = ()
    stalled = []

    def in_stall(wall, mono):
        if clock != 'wall':
            return None
        for end, gap in stalls or () if wall is not None else ():
            if end - gap - .05 <= wall <= end + STALL_ATTRIBUTION_S:
                return dict(stall_end=end, stall_ms=round(gap * 1000., 1))
        for start, end, rate in slow or () if mono is not None else ():
            if start - .05 <= mono and (end is None or mono <= end + STALL_ATTRIBUTION_S):
                return dict(slow_start_mono=start, slow_end_mono=end, slow_min_rate=round(rate, 3))
        return None
    for e in bridge:
        if e.get('kind') != 'state':
            continue
        if after_wall is not None and e.get('wall', 0) < after_wall:
            known = tuple(e.get('latches') or ())
            continue
        latches = tuple(e.get('latches') or ())
        for name in latches:
            if name not in known:
                row = dict(wall=e.get('wall'), t=e.get('t'), latch=name, reason=e.get('reason'))
                stall = in_stall(e.get('wall'), e.get('t')) if name == 'steering_status_stale' else None
                if stall is not None:
                    stalled.append(dict(row, **stall))
                else:
                    (seen if allowed('host', name, e.get('wall')) else host).append(row)
        known = latches
    by_code = {}
    for row in mcu:
        by_code[row['code']] = by_code.get(row['code'], 0) + 1
    for row in bits:
        if row['bit'] not in [m['code'] for m in mcu]:       # a latched code gives an F line and a bit: count once
            by_code[row['bit']] = by_code.get(row['bit'], 0) + 1
    for row in host:
        key = 'host:' + row['latch']
        by_code[key] = by_code.get(key, 0) + 1
    return dict(count=sum(by_code.values()), by_code=by_code, mcu_lines=mcu[:20], new_bits=bits[:20],
                host_latches=host[:20], expected_seen=seen[:20], sim_stall_latches=stalled[:20],
                rule='R15-3: unexpected F / W lines, new fault bits (BOOT excepted) and new host latches; a '
                     'steering_status_stale inside an emulator /clock wall stall or a slow /clock interval (SIM time '
                     'below 0.9 x real time over 0.3 s; wall clock mode) is listed apart (A10 invalid)')


class Ctx:
    """Clients, observer and log access of one check run."""

    def __init__(self, out, owner='p6a-check'):
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.client = ControlClient(owner)
        self.hooks = HookClient()
        self.obs = SimObserver()
        self.cfg = None

    # status and marks
    def steering_status(self):
        return load_json(STEERING_STATUS)

    def session_params(self):
        info = self.hooks.status()['status']['session_info']
        return info, (info.get('calibration') or {}).get('params')

    def mark(self, label):
        reply = self.hooks.mark(label=label)
        return dict(label=label, tick=reply['tick'], sim=reply['sim_time_s'], session=reply['session'], wall=time.time(),
                    mono=time.monotonic())

    def logs(self, mark, wait=True):
        if wait:
            self.obs.spin_for(LOG_FLUSH_WAIT_S)
        emu = [e for e in read_jsonl(EMULATOR_LOG) if e.get('session') == mark['session'] and
               e.get('tick', 0) >= mark['tick']]
        bridge = [e for e in read_jsonl(BRIDGE_LOG) if e.get('wall', 0) >= mark['wall'] - .05]
        return emu, bridge

    # control helpers
    def host(self):
        return self.client.status()

    def wait_host(self, predicate, timeout_s=3.):
        end = time.monotonic() + timeout_s
        status = self.host()
        while time.monotonic() < end:
            if predicate(status):
                return status
            self.obs.spin_for(.05)
            status = self.host()
        return status if predicate(status) else None

    def ensure_armed(self, timeout_s=4.):
        """STOP, wait until STOP is confirmed, ARM (front_steer), wait for the MCU confirmation. Returns the reply."""
        self.client.stop('brake')
        self.wait_host(lambda s: s['steering_host']['stop_state'] == 'confirmed', 2.)
        end = time.monotonic() + timeout_s
        reply = None
        while time.monotonic() < end:
            reply = self.client.arm()
            if reply.get('ok'):
                status = self.wait_host(lambda s: s['steering_host']['arm_state'] == 'confirmed', 1.)
                if status is not None:
                    return reply
            self.obs.spin_for(.2)
        raise ClientError('could not arm: %s' % (reply,))

    def wait_still(self, timeout_s=15., hold_s=.3):
        def still():
            if not self.obs.odom:
                return False
            t_end = self.obs.odom[-1][0]
            recent = [row for row in self.obs.odom if row[0] >= t_end - hold_s]
            return len(recent) >= 3 and all(abs(row[4]) < STILL_SPEED_M_S for row in recent)
        return self.obs.spin_until(still, timeout_s)

    def drive_sim(self, throttle, steer_mrad, sim_s):
        loop = DriveLoop(self.client, throttle, steer_mrad).start()
        try:
            self.obs.spin_sim_for(sim_s)
        finally:
            loop.stop()
        return loop

    def close(self):
        try:
            self.client.stop('brake')
        except (OSError, ClientError, ValueError):
            pass
        self.obs.close()


def entries(emu, kind, **match):
    return [e for e in emu if e.get('kind') == kind and all(e.get(k) == v for k, v in match.items())]


def first(seq, predicate):
    for item in seq:
        if predicate(item):
            return item
    return None


def pin_first(emu, channel, predicate, after_tick):
    return first(entries(emu, 'pin', channel=channel), lambda e: e['tick'] >= after_tick and predicate(e['value']))


def drive_zero_tick(emu, after_tick):
    """First tick at or after after_tick at which both drive pins are 0 (pin log transitions)."""
    state = dict(drive_l=None, drive_r=None)
    candidates = []
    for e in entries(emu, 'pin'):
        if e['channel'] in state:
            state[e['channel']] = e['value']
            if e['tick'] >= after_tick and state['drive_l'] in (0, None) and state['drive_r'] in (0, None):
                candidates.append(e['tick'])
    return candidates[0] if candidates else None


def status_lines(emu, after_tick=0):
    out = []
    for e in entries(emu, 'm2h'):
        if e['tick'] >= after_tick and e['text'].startswith('S '):
            s = parse_status_line(e['text'])
            if s is not None:
                out.append(dict(e, status=s))
    return out


def f_line(emu, code, after_tick):
    return first(entries(emu, 'm2h'), lambda e: e['tick'] >= after_tick and body(e['text']).split(' ')[:2] == ['F', code])


def truth_rows(obs, t0, t1):
    return [row for row in obs.truth if row.get('sim_time_s') is not None and t0 <= row['sim_time_s'] <= t1]


def odom_rows(obs, t0, t1):
    return [row for row in obs.odom if t0 <= row[0] <= t1]


def path_length(rows):
    return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(rows, rows[1:]))


def lateral_in_start_frame(rows):
    """(along, lateral) of the last row in the frame of the first row (x along its heading, y left)."""
    t0, x0, y0, yaw0 = rows[0][:4]
    dx, dy = rows[-1][1] - x0, rows[-1][2] - y0
    c, s = math.cos(yaw0), math.sin(yaw0)
    return c * dx + s * dy, -s * dx + c * dy


# --- A13 ----------------------------------------------------------------------------------------------------------------------------
EXPECTED_NODES = ('ros_gz_bridge', 'ubts_cpu_lidar', 'ubts_steer_sensor_contract', 'ubts_steer_esp_emulator',
                  'ubts_steer_rover_bridge', 'ubts_data_recorder', 'ubts_lidar_ws_bridge', 'ubts_imu_ws_bridge',
                  'ubts_gps_websocket_bridge')


def expected_nodes(scene=None):
    """The nodes of the stack (a field_world scene adds the clip node)."""
    scene = scene if scene is not None else os.environ.get('UBTS_STEER_SCENE')
    return EXPECTED_NODES + ((p6a_graph.CLIP_NODE,) if scene == 'field1010' else ())


def stable_graph(ctx, expected=None, timeout_s=40., period_s=2.):
    """Capture the graph until every expected node is there and two captures in a row are equal (DDS discovery of
    ten participants on the loopback takes a few seconds)."""
    expected = expected_nodes() if expected is None else expected
    started = time.monotonic()
    previous, graph = None, None
    while time.monotonic() - started < timeout_s:
        ctx.obs.spin_for(period_s)
        graph = p6a_graph.capture_graph(ctx.obs.node)
        complete = all(n in graph['nodes'] for n in expected) and not any(
            '_NODE_NAME_UNKNOWN_' in n for info in graph['topics'].values()
            for n in info['publishers'] + info['subscribers'])
        if complete and previous is not None and previous['topics'] == graph['topics']:
            graph['discovery_wait_s'] = round(time.monotonic() - started, 1)
            graph['discovery_complete'] = True
            return graph
        previous = graph
    graph['discovery_wait_s'] = round(time.monotonic() - started, 1)
    graph['discovery_complete'] = False
    graph['missing_nodes'] = [n for n in expected if n not in graph['nodes']]
    return graph


def check_graph(ctx, args):
    graph = stable_graph(ctx)
    result = p6a_graph.check_truth_isolation(graph)
    executor = p6a_graph.check_executor_subscriptions(graph)
    write_json(ctx.out / 'ROS_GRAPH.json', graph)
    criteria = [criterion('discovery_complete', graph.get('missing_nodes', []), [], graph['discovery_complete']),
                criterion('rules_R1_R5_no_findings', len(result['findings']), 0, result['passed'])]
    criteria.append(criterion('executor_allowlist_R6', executor.get('not_allowed'), [], executor['passed'],
                              executor.get('note') or executor.get('rule')))
    criteria.append(criterion('executor_infrastructure_topics', executor.get('infrastructure'), 'report',
                              None, 'report: /clock, /parameter_events, /rosout are judged under R6 as registered; '
                                    'passed_if_infrastructure_allowed = %s' %
                              executor.get('passed_if_infrastructure_allowed')))
    criteria.append(criterion('scan_absent_until_s2_node', graph['topics'].get('/scan', {}).get('publishers', []), [],
                              None, 'report: /scan has no publisher in P6a (the S2 format node is the next phase)'))
    return dict(check='A13_G_graph', criteria=criteria, passed=verdict(criteria), isolation=result,
                executor=executor, nodes=graph['nodes'], topic_count=len(graph['topics']))


# --- A14 ----------------------------------------------------------------------------------------------------------------------------
def http_old_screen(base_url):
    """The old differential screen's requests through the dashboard HTTP API: arm without vehicle_profile, then a
    left/right drive. Returns the replies."""
    import http.cookiejar
    import urllib.request
    jar = http.cookiejar.CookieJar()
    opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(jar))

    def post(path, payload):
        request = urllib.request.Request(base_url + path, data=json.dumps(payload).encode('utf-8'), method='POST',
                                         headers={'Content-Type': 'application/json'})
        with opener.open(request, timeout=5) as response:
            return json.loads(response.read().decode('utf-8'))
    with opener.open(base_url + '/api/rover/status', timeout=5) as response:    # session cookie
        response.read()
    arm = post('/api/rover/arm', {})
    lease = arm.get('lease')
    drive = post('/api/rover/command', {'lease': lease, 'ticket': arm.get('ticket'), 'seq': (arm.get('seq') or 0) + 1,
                                         'left': 40, 'right': 40, 'deadman': True})
    stop = post('/api/rover/stop', {'reason': 'old_screen_check'})
    return dict(arm=arm, drive=drive, stop=stop, cookies=len(jar))


def check_identity(ctx, args):
    sys.path.insert(0, '/opt/ubts/dashboard')
    from scripts import esp_motor_v3 as v3
    criteria, details = [], {}
    # 1. SIM wrapper refuses a sim=0 device: banner rewrite on the next boot, MCU reset
    ctx.client.stop('brake')
    ctx.hooks.banner_rewrite(fields={'sim': '0'}, count=1)
    mark = ctx.mark('sim0')
    ctx.hooks.event(kind='mcu_reset', arg=4)
    ok = ctx.wait_host(lambda s: 'sim_identity_mismatch' in s['steering']['faults'], 6.)
    arm = ctx.client.arm()
    # a later banner with sim=1 in the same port session does not lift the refusal (EspSteerV3 keeps the session
    # refused until the port is opened again): a USB unplug / replug gives a new session
    ctx.hooks.event(kind='mcu_reset', arg=4)
    ctx.obs.spin_for(2.)
    arm_again = ctx.client.arm()
    details['sim0'] = dict(latched=ok is not None, arm=arm, arm_after_good_banner_same_session=arm_again)
    criteria.append(criterion('sim_wrapper_refuses_sim0_device', arm.get('error'), 'refused', ok is not None
                              and not arm.get('ok')))
    criteria.append(criterion('refusal_kept_until_the_port_reopens', arm_again.get('error'), 'refused',
                              not arm_again.get('ok')))
    ctx.hooks.unplug(duration_ms=1500, mcu_power='cycle')
    ctx.obs.spin_for(5.)
    # 2. hardware profile mismatch refuses arm
    ctx.hooks.banner_rewrite(fields={'hw': 'front_steer_x9'}, count=1)
    ctx.hooks.event(kind='mcu_reset', arg=4)
    ctx.obs.spin_for(2.5)
    arm = ctx.client.arm()
    details['hw'] = dict(arm=arm)
    criteria.append(criterion('hardware_profile_mismatch_refuses_arm', arm.get('error'), 'hardware_profile_mismatch',
                              arm.get('error') == 'hardware_profile_mismatch'))
    ctx.hooks.event(kind='mcu_reset', arg=4)
    ctx.obs.spin_for(2.5)
    # 3. the real-hardware transport refuses the SIM path (path rule) and a pty slave of this container (udev
    #    identity: none, or a forged record of another ID_SERIAL). The emulator's pty slave lives in the steer
    #    container's devpts and cannot be opened from here (reported, not judged; review RG-08).
    path_rule = try_serial_open(v3, '/run/ubts-steer/esp-tty')
    details['real_transport_sim_path'] = path_rule
    criteria.append(criterion('real_transport_refuses_the_sim_path', path_rule, 'refused by the path rule',
                              path_rule['refused'] and 'SIM device path refused' in path_rule['error']))
    pty = pty_refusals(v3)
    details['real_transport_pty'] = pty
    criteria.append(criterion('real_transport_refuses_a_pty_slave', {k: v.get('error') for k, v in pty.items()
                                                                     if isinstance(v, dict)},
                              'refused: no udev identity; forged record of another ID_SERIAL',
                              pty.get('available') is True and pty['no_udev_record']['refused'] and
                              pty['forged_other_identity']['refused'] and
                              'identity mismatch' in pty['forged_other_identity']['error']))
    other = try_serial_open(v3, os.path.realpath('/run/ubts-steer/esp-tty'))
    details['real_transport_emulator_slave_other_devpts'] = other
    criteria.append(criterion('emulator_pty_slave_from_the_check_container', other['error'], 'report', None,
                              'report: the emulator slave is in the steer container\'s devpts (not visible here)'))
    # 3b. the wrapper processes: the SIM role refuses a device outside /run/ubts-steer (configuration error, exit 2);
    #     the real role (serial transport, a host profile with a fixed cal_id) runs but never opens the SIM pty
    wrappers = wrapper_refusals()
    details['wrappers'] = wrappers
    criteria.append(criterion('sim_wrapper_refuses_a_device_outside_the_sim_root', wrappers['sim_outside_root'],
                              'exit 2, must be under /run/ubts-steer', wrappers['sim_outside_root']['exit'] == 2 and
                              'must be under' in wrappers['sim_outside_root']['stderr']))
    real = wrappers['real_on_the_pty']
    criteria.append(criterion('real_wrapper_never_opens_the_pty', real, 'serial_open false, open refused',
                              real['exit'] == 0 and real.get('serial_open') is False and
                              'SIM device path refused' in str(real.get('last_error'))))
    # 4. the real-role link refuses a sim=1 banner (EspSteerV3 simulated=False on a memory transport fed with the
    #    emulator's last banner)
    banner = ctx.steering_status()['emulator']['last_banner']
    transport = v3.MemoryTransport(clock=time.monotonic)
    link = v3.EspSteerV3(transport, time.monotonic, simulated=False, expect_hw='front_steer_v0', banner_timeout_s=1.5)
    link.open()
    transport.device.send((banner + '\n').encode('ascii'), at=time.monotonic())
    events = link.poll()
    details['real_link'] = dict(banner=banner, identity_error=link.identity_error, ready=link.ready,
                                events=[e.get('type') for e in events])
    criteria.append(criterion('real_link_refuses_sim1_banner', link.identity_error, 'sim_identity_mismatch',
                              link.identity_error == 'sim_identity_mismatch' and not link.ready))
    # 5. old control screen through the dashboard HTTP API
    try:
        old = http_old_screen(args.dashboard_url)
    except OSError as exc:
        old = dict(error=str(exc))
    details['old_screen'] = old
    arm_err = (old.get('arm') or {}).get('error')
    drive_err = (old.get('drive') or {}).get('error')
    criteria.append(criterion('old_screen_cannot_arm', arm_err, 'steer_profile_required',
                              arm_err == 'steer_profile_required'))
    criteria.append(criterion('old_screen_drive_refused', drive_err, 'refused', drive_err is not None,
                              'without a lease the drive is control_not_owned; with one it would be '
                              'differential_command_rejected (checked through the socket below)'))
    # 6. left/right drive with a lease (socket): differential_command_rejected and stop
    ctx.ensure_armed()
    mark = ctx.mark('differential')
    reply = ctx.client.drive_differential(40, 40)
    emu, _ = ctx.logs(mark)
    moved = [e for e in entries(emu, 'pin') if e['channel'] in ('drive_l', 'drive_r') and e['value'] != 0]
    details['differential'] = dict(reply=reply, drive_pin_changes=len(moved))
    criteria.append(criterion('differential_command_rejected', reply.get('error'), 'differential_command_rejected',
                              reply.get('error') == 'differential_command_rejected' and not moved))
    # 7. ROS domains: the domain 43 topics are not seen from 0 / 42
    import subprocess
    domains = {}
    for domain in ('43', '0', '42'):          # 43 first: the positive control with the same listing time
        env = dict(os.environ, ROS_DOMAIN_ID=domain)
        out = subprocess.run(['ros2', 'topic', 'list', '--no-daemon', '--spin-time', '6'], env=env,
                             capture_output=True, text=True, timeout=90)
        domains[domain] = sorted(t for t in out.stdout.split() if t)
    own = [t for t in ('/rover/steering/state', '/sim/truth/steering', '/imu/data_raw', '/sim/act/steer_left_cmd')]
    seen = {d: [t for t in own if t in topics] for d, topics in domains.items()}
    details['domains'] = dict(listing={d: len(t) for d, t in domains.items()}, steer_topics_seen=seen,
                              env_domain=os.environ.get('ROS_DOMAIN_ID'), gz_partition=os.environ.get('GZ_PARTITION'))
    criteria.append(criterion('domain_43_sees_its_topics', seen['43'], own, seen['43'] == own,
                              'positive control of the listing'))
    criteria.append(criterion('domains_0_42_do_not_see_43', {d: seen[d] for d in ('0', '42')}, 'none',
                              not seen['0'] and not seen['42'] and os.environ.get('ROS_DOMAIN_ID') == '43'))
    ctx.client.stop('brake')
    return dict(check='A14_identity', criteria=criteria, passed=verdict(criteria), details=details)


def try_serial_open(v3, path, identity='Silicon_Labs_CP2102_test', udev_dir='/run/udev/data'):
    """Open path with the real-hardware transport; dict(refused, error). Closes it if it opened."""
    transport = v3.SerialTransport(path, identity, udev_dir=udev_dir, lock_dir=os.environ.get('TMPDIR', '/tmp'))
    try:
        transport.open()
    except (OSError, RuntimeError) as exc:
        return dict(path=path, refused=True, error='%s: %s' % (type(exc).__name__, exc))
    transport.close()
    return dict(path=path, refused=False, error=None)


def pty_refusals(v3):
    """A pty pair made in this container: the real-hardware transport must refuse the slave without a udev record
    and with a forged udev record of another ID_SERIAL (a forged record of the expected one would open it; the link
    then refuses the sim=1 banner, item 4)."""
    import tempfile
    try:
        master, slave = os.openpty()
    except OSError as exc:
        return dict(available=False, error='%s: %s' % (type(exc).__name__, exc))
    try:
        path = os.ttyname(slave)
        info = os.stat(path)
        out = dict(available=True, slave=path, major=os.major(info.st_rdev))
        out['no_udev_record'] = try_serial_open(v3, path)
        with tempfile.TemporaryDirectory(prefix='p6a_udev_') as udev:
            name = 'c%d:%d' % (os.major(info.st_rdev), os.minor(info.st_rdev))
            with open(os.path.join(udev, name), 'w', encoding='utf-8', newline='\n') as handle:
                handle.write('E:ID_SERIAL=Some_Other_Device_123\n')
            out['forged_other_identity'] = try_serial_open(v3, path, udev_dir=udev)
        return out
    finally:
        os.close(master)
        os.close(slave)


def wrapper_refusals(host_profile='/runtime/steer/host_profile.json', timeout_s=30.):
    """Run the bridge wrapper CLI twice (no ROS, no socket, bounded run time) in a temporary folder: the SIM role with
    a device outside /run/ubts-steer, and the real role (serial) pointed at the SIM pty."""
    import subprocess
    import tempfile
    out = {}
    with tempfile.TemporaryDirectory(prefix='p6a_wrappers_') as tmp:
        data = load_json(host_profile)
        data['calibration'] = dict(data.get('calibration') or {}, cal_id=1)      # the real role needs a fixed cal_id
        fixed = os.path.join(tmp, 'host_profile_fixed.json')
        with open(fixed, 'wb') as handle:
            handle.write(json.dumps(data).encode('utf-8'))
        status = os.path.join(tmp, 'real_status.json')
        runs = dict(sim_outside_root=['--transport', 'pty', '--device', '/dev/ttyUSB0', '--host-profile', host_profile,
                                      '--no-ros', '--no-socket', '--max-runtime-s', '0.5'],
                    real_on_the_pty=['--transport', 'serial', '--device', '/run/ubts-steer/esp-tty', '--identity',
                                     'Silicon_Labs_CP2102_p6a_check', '--host-profile', fixed, '--no-ros', '--no-socket',
                                     '--status-file', status, '--max-runtime-s', '1.5'])
        for name, argv in runs.items():
            proc = subprocess.run([sys.executable, '-B', '-m', 'scripts.steer_rover_bridge'] + argv,
                                  cwd='/opt/ubts/dashboard', capture_output=True, text=True, timeout=timeout_s)
            result = dict(exit=proc.returncode, stderr=proc.stderr.strip()[-300:])
            if name == 'real_on_the_pty' and os.path.exists(status):
                snap = load_json(status)
                result.update(serial_open=snap.get('serial_open'), state=snap.get('state'),
                              last_error=snap.get('last_error'), transport=snap.get('transport'),
                              transport_kind=snap.get('transport_kind'))
            out[name] = result
    return out


# --- A17 ----------------------------------------------------------------------------------------------------------------------------
def bridge_stop_latency(bridge, after_wall):
    req = first(bridge, lambda e: e['kind'] == 'request' and e.get('op') == 'stop' and e['wall'] >= after_wall)
    if req is None:
        return None, None, None
    tx = first(bridge, lambda e: e['kind'] == 'tx' and e['head'].startswith('STOP') and e['t'] >= req['t'])
    confirm = first(bridge, lambda e: e['kind'] == 'state' and e.get('stop_state') == 'confirmed' and e['t'] >= req['t'])
    return req, tx, confirm


def stop_once(ctx, index, throttle=40, steer_mrad=175, clock='wall'):
    before, step0 = step_start(ctx, 'stop_%d_start' % index)
    ctx.ensure_armed()
    loop = DriveLoop(ctx.client, throttle, steer_mrad).start()
    try:
        ctx.obs.spin_sim_for(3.)
        mark = ctx.mark('stop_%d' % index)
        sim_stop = ctx.obs.sim_time
        ctx.client.stop('brake')
    finally:
        loop.stop()
    ctx.obs.spin_sim_for(3.5)
    after = emulator_counters(ctx.hooks)
    emu, bridge = ctx.logs(mark)
    # a normal STOP from the ARM on: no fault line, bit or host latch is expected (A19(G) false detections)
    emu_step, _ = ctx.logs(step0, wait=False)
    false = false_detections(emu_step, read_jsonl(BRIDGE_LOG), after_tick=step0['tick'],
                             after_wall=step0['wall'] - .01,
                             stalls=emulator_stalls(), slow=emulator_slow(), clock=clock)
    validity = step_validity(ctx, clock, step0['mono'], time.monotonic(), before, after,
                             sim_made=false['sim_stall_latches'] if clock == 'wall' else ())
    req, tx, confirm = bridge_stop_latency(bridge, mark['wall'] - .01)
    stop_line = first(entries(emu, 'h2m'), lambda e: e['text'].startswith('STOP'))
    arrival = stop_line['last'] if stop_line else None
    zero = drive_zero_tick(emu, arrival) if arrival is not None else None
    confirm_line = first(status_lines(emu, arrival or 0), lambda e: e['status']['st'] == 1 and e['status']['L'] == 0
                         and e['status']['R'] == 0) if arrival is not None else None
    restart = [e for e in entries(emu, 'pin') if e['channel'] in ('drive_l', 'drive_r') and e['value'] != 0 and
               zero is not None and e['tick'] > zero]
    # hold: truth delta_eq from the STOP to 2 s later (hstop 2000 ms)
    t_arr = None if arrival is None else mark['sim'] + (arrival - mark['tick']) / 1000.
    hold = truth_rows(ctx.obs, t_arr, t_arr + 1.9) if t_arr is not None else []
    hold_dev = max((abs(r['delta_eq_rad'] - hold[0]['delta_eq_rad']) for r in hold), default=None)
    odom = odom_rows(ctx.obs, sim_stop, sim_stop + 3.4)
    stop_rows = [r for r in odom if abs(r[4]) >= STILL_SPEED_M_S]
    still_from = first(odom, lambda r: r[0] > sim_stop and abs(r[4]) < STILL_SPEED_M_S)
    after = odom_rows(ctx.obs, still_from[0], still_from[0] + 2.) if still_from else []
    drift = path_length(after) if len(after) > 1 else None
    return dict(index=index, mark=mark, step_start=step0, request_t=None if req is None else req['t'], tx_t=None if tx is None else tx['t'],
                stop_send_latency_s=None if (req is None or tx is None) else tx['t'] - req['t'],
                host_confirm_s=None if (req is None or confirm is None) else confirm['t'] - req['t'],
                host_confirm_from_tx_s=None if (tx is None or confirm is None) else confirm['t'] - tx['t'],
                validity=validity, false_detection=false,
                stop_arrival_tick=arrival, drive_zero_tick=zero,
                drive_zero_ticks=None if (zero is None or arrival is None) else zero - arrival,
                confirm_line_tick=None if confirm_line is None else confirm_line['tick'],
                confirm_line_ms=None if (confirm_line is None or arrival is None) else confirm_line['tick'] - arrival,
                restart_pin_changes=len(restart),
                hold_max_dev_deg=None if hold_dev is None else math.degrees(hold_dev), hold_samples=len(hold),
                stop_distance_m=path_length(odom_rows(ctx.obs, sim_stop, still_from[0])) if still_from else None,
                speed_at_stop_m_s=abs(odom[0][4]) if odom else None, drift_after_still_m=drift)


def first_tick_after(ticks, grid_t, sent_t, tol):
    """An action sent at sent_t went out at the first loop tick at or after grid_t: sent_t >= grid_t - tol and the
    loop tick before the one that sent it (the last tick log entry below sent_t - tol) is earlier than grid_t + tol.
    ticks: loop tick times (read before the core step) of the bridge's tick log."""
    own = [t for t in ticks if t <= sent_t + tol]
    if not own:
        return False, None
    previous = [t for t in ticks if t < own[-1] - tol]
    before = previous[-1] if previous else None
    return sent_t >= grid_t - tol and (before is None or before < grid_t + tol), before


def stop_unconfirmed(ctx, clock='wall'):
    before, step0 = step_start(ctx, 'stop_unconfirmed_start')
    ctx.ensure_armed()
    loop = DriveLoop(ctx.client, 30, 0).start()
    try:
        ctx.obs.spin_sim_for(2.)
        mark = ctx.mark('stop_unconfirmed')
        ctx.hooks.status_loss(duration_ms=1500)
        ctx.client.stop('brake')
    finally:
        loop.stop()
    ctx.obs.spin_sim_for(2.5)
    after = emulator_counters(ctx.hooks)
    validity = step_validity(ctx, clock, step0['mono'], time.monotonic(), before, after,
                             sim_made=step_sim_latches(ctx, step0, clock) if clock == 'wall' else ())
    emu, bridge = ctx.logs(mark)
    req = first(bridge, lambda e: e['kind'] == 'request' and e.get('op') == 'stop')
    stops = [e for e in bridge if e['kind'] == 'tx' and e['head'].startswith('STOP') and req and e['t'] >= req['t']]
    t0 = stops[0]['t'] if stops else None
    resend = [round((e['t'] - t0) * 1000., 1) for e in stops[1:]] if t0 is not None else []
    latch = first(bridge, lambda e: e['kind'] == 'state' and 'stop_unconfirmed' in e.get('latches', []))
    latch_ms = None if (latch is None or t0 is None) else round((latch['t'] - t0) * 1000., 1)
    grid = [100 + 50 * k for k in range(5)]
    # amendment 7: each resend at the first loop tick at or after t0 + 100 + 50 k ms; the latch at the first loop
    # tick at or after t0 + 400 ms. The bridge logs every loop tick while the STOP is pending (tick times are read
    # before the core step: 1 ms slack for the wall clock, none for the SIM clock)
    tol = 1e-6 if clock == 'sim' else 1e-3
    ticks = sorted(e['t'] for e in bridge if e['kind'] == 'tick' and t0 is not None and e['t'] >= t0 - .05)
    resend_ticks = []
    for k, e in enumerate(stops[1:6]):
        ok, before_t = first_tick_after(ticks, t0 + grid[k] / 1000., e['t'], tol)
        resend_ticks.append(dict(k=k, grid_ms=grid[k], sent_ms=resend[k], first_tick_after_grid=ok,
                                 previous_tick_ms=None if before_t is None else round((before_t - t0) * 1000., 2)))
    latch_ok, latch_before = (False, None) if (latch is None or t0 is None) else first_tick_after(
        ticks, t0 + .4, latch['t'], tol)
    on_grid = len(resend) == 5 and len(ticks) > 5 and all(r['first_tick_after_grid'] for r in resend_ticks)
    criteria = [criterion('resend_count', len(resend), 5, len(resend) == 5),
                criterion('resends_at_the_first_loop_tick_after_the_50ms_grid', resend, grid, on_grid,
                          'amendment 7: each resend at the first loop tick at or after its grid time (bridge tick '
                          'log: %d ticks)' % len(ticks)),
                criterion('latch_stop_unconfirmed_ms', latch_ms, 'first loop tick at or after 400', latch_ok)]
    criteria = invalidate(criteria, validity)
    return dict(name='stop_unconfirmed', criteria=criteria, passed=verdict(criteria), resend_ms=resend,
                resend_ticks=resend_ticks, latch_ms=latch_ms, latch_previous_tick_ms=None if latch_before is None or
                t0 is None else round((latch_before - t0) * 1000., 2), loop_ticks_logged=len(ticks),
                validity=validity, mcu_stop_lines=len([e for e in entries(emu, 'h2m') if e['text'].startswith('STOP')]))


def analytic_stop_distance(profile, speed_m_s, ttl_remaining_s, mode='brake'):
    """Distance from the link loss to standstill of the A17 analytic arc: the speed is held for the rest of the C TTL
    (the MCU keeps the last duty), then the stop mode of the TTL expiry (drive.stop_modes: onset, constant
    deceleration; ttlmode 0 = brake, 1 = release)."""
    from . import drive
    stop = drive.stop_modes(profile)[mode]
    v = abs(float(speed_m_s))
    return v * max(0., float(ttl_remaining_s)) + stop.distance_m(v), stop


def arc_lateral(kappa, s):
    """Lateral offset after the arc length s on a circle of curvature kappa (signed like kappa)."""
    return (1. - math.cos(kappa * s)) / kappa if abs(kappa) > 1e-9 else 0.


def link_loss(ctx, throttle=48, steer_mrad=87, loss_ms=1500, clock='wall'):
    """C TTL expiry and the link watchdog with a link loss to the MCU while driving; lateral deviation vs the arc."""
    info, params = ctx.session_params()
    wd = int(params['wd']) if params else 500
    ttl_mode = 'release' if params and int(params.get('ttlmode', 0)) == 1 else 'brake'
    before, step0 = step_start(ctx, 'link_loss_start')
    ctx.ensure_armed()
    loop = DriveLoop(ctx.client, throttle, steer_mrad).start()
    try:
        ctx.obs.spin_sim_for(4.)
        mark = ctx.mark('link_loss')
        reply = ctx.hooks.link(direction='to_mcu', duration_ms=loss_ms)
        ctx.obs.spin_sim_for(loss_ms / 1000. + .5)
    finally:
        loop.stop()
    ctx.client.stop('brake')
    ctx.obs.spin_sim_for(1.5)
    after = emulator_counters(ctx.hooks)
    validity = step_validity(ctx, clock, step0['mono'], time.monotonic(), before, after,
                             sim_made=step_sim_latches(ctx, step0, clock) if clock == 'wall' else ())
    emu, bridge = ctx.logs(mark)
    start = reply['start_tick']
    before = [e for e in read_jsonl(EMULATOR_LOG) if e.get('session') == mark['session'] and e['kind'] == 'h2m'
              and e.get('note') == 'host' and e.get('last') is not None and e['last'] < start + 1]
    last_c = first(reversed(before), lambda e: e['text'].startswith('C '))
    last_input = first(reversed(before), lambda e: e['text'].startswith(('C ', 'PING')))
    c = parse_c_line(last_c['text']) if last_c else None
    zero = drive_zero_tick(emu, start)
    ttl_line = f_line(emu, 'TTL', start)
    w_line = first(entries(emu, 'm2h'), lambda e: e['tick'] >= start and e['text'].startswith('W '))
    disarm = pin_first(emu, 'mcu_state', lambda v: v == 1, start)
    ttl_expected = None if c is None else last_c['last'] + c['ttl']
    wd_expected = None if last_input is None else last_input['last'] + wd + 1
    restart = [e for e in entries(emu, 'pin') if e['channel'] in ('drive_l', 'drive_r') and e['value'] != 0 and
               zero is not None and e['tick'] > zero]
    # lateral deviation from the loss start to the stop, against the analytic arc (review RG-06): the held curvature,
    # the speed at the loss, the rest of the C TTL and the stop mode give the arc length (drive.stop_modes); the arc
    # over the SIM's own path length is kept as an auxiliary value
    t_loss = mark['sim'] + (start - mark['tick']) / 1000.
    rows = odom_rows(ctx.obs, t_loss, t_loss + loss_ms / 1000. + 1.5)
    held = truth_rows(ctx.obs, t_loss - .05, t_loss + .05)
    lateral = arc = arc_path = s_path = s_analytic = ttl_remaining_s = None
    stop_model = None
    if len(rows) > 2 and held:
        still = first(rows, lambda r: r[0] > t_loss + .05 and abs(r[4]) < STILL_SPEED_M_S)
        rows = [r for r in rows if still is None or r[0] <= still[0]]
        from .vehicle_profile import load_profile
        profile = load_profile()
        wheelbase = float(profile.value('wheelbase_m'))
        rear = float(profile.value('rear_axle_x_in_base_m'))
        # the rear axle centre moves along the circle of the bicycle model (base_footprint has a side slip angle)
        rows = [(r[0], r[1] + rear * math.cos(r[3]), r[2] + rear * math.sin(r[3]), r[3], r[4], r[5]) for r in rows]
        s_path = path_length(rows)
        _, lateral = lateral_in_start_frame(rows)
        kappa = math.tan(held[0]['delta_eq_rad']) / wheelbase
        arc_path = arc_lateral(kappa, s_path)
        if c is not None:
            ttl_remaining_s = max(0., (last_c['last'] + c['ttl'] - start) / 1000.)
            s_analytic, stop = analytic_stop_distance(profile, rows[0][4], ttl_remaining_s, ttl_mode)
            stop_model = dict(mode=ttl_mode, onset_s=stop.onset_s, decel_m_s2=stop.decel_m_s2())
            arc = arc_lateral(kappa, s_analytic)
    criteria = [
        criterion('ttl_drive_zero_tick_minus_expected', None if (zero is None or ttl_expected is None) else
                  zero - ttl_expected, '0..1 (<= ttl + 1 tick)', zero is not None and ttl_expected is not None
                  and 0 <= zero - ttl_expected <= 1),
        criterion('ttl_line_tick', None if ttl_line is None else ttl_line['tick'], ttl_expected, ttl_line is not None),
        criterion('watchdog_tick_minus_last_input', None if (w_line is None or last_input is None) else
                  w_line['tick'] - last_input['last'], '<= wd + 1 = %d' % (wd + 1), w_line is not None and
                  last_input is not None and w_line['tick'] - last_input['last'] <= wd + 1),
        criterion('watchdog_disarmed', None if disarm is None else disarm['tick'], 'DISARMED at the W tick',
                  disarm is not None and w_line is not None and disarm['tick'] == w_line['tick']),
        criterion('no_automatic_restart', len(restart), 0, not restart),
        criterion('lateral_deviation_vs_analytic_arc_m', None if (lateral is None or arc is None) else
                  [round(lateral, 4), round(arc, 4)], '|sim - arc| <= max(10 %%, 0.01 m); arc from the held angle, the '
                  'speed, the rest of the TTL and the %s stop mode' % ttl_mode, lateral is not None and arc is not None
                  and abs(abs(lateral) - abs(arc)) <= max(.1 * abs(arc), .01)),
        criterion('stop_distance_sim_vs_analytic_m', None if (s_path is None or s_analytic is None) else
                  [round(s_path, 4), round(s_analytic, 4)], 'report', None,
                  'report: the SIM path length of the rear axle centre from the loss to standstill, and the analytic '
                  'distance (G-SIM brakes with the DC line, the analytic stop mode is the F-SIM model)'),
        criterion('lateral_deviation_vs_path_arc_m', None if (lateral is None or arc_path is None) else
                  [round(lateral, 4), round(arc_path, 4)], 'report (auxiliary: arc over the SIM path length)', None),
        criterion('lateral_deviation_design_target_m', None if lateral is None else abs(lateral), '<= 0.05 (report)',
                  None, 'design target (report): |delta| <= 5 deg at 0.4 m/s'),
    ]
    criteria = invalidate(criteria, validity)
    return dict(name='link_loss_ttl_watchdog', criteria=criteria, passed=verdict(criteria), hook=reply,
                validity=validity, ttl_mode=ttl_mode, ttl_remaining_s=ttl_remaining_s, s_path_m=s_path,
                s_analytic_m=s_analytic, stop_model=stop_model, arc_path_m=arc_path,
                last_c=None if last_c is None else dict(text=last_c['text'], arrival=last_c['last'], **(c or {})),
                last_input=None if last_input is None else dict(text=last_input['text'], arrival=last_input['last']),
                drive_zero_tick=zero, ttl_expected_tick=ttl_expected, watchdog_tick=None if w_line is None else
                w_line['tick'], watchdog_expected_tick=wd_expected, wd_ms=wd, held_delta_deg=math.degrees(
                    held[0]['delta_eq_rad']) if held else None, speed_at_loss_m_s=abs(rows[0][4]) if rows else None,
                lateral_m=lateral, arc_m=arc)


def fault_stop(ctx, clock='wall'):
    """POT_OPEN while steering: steering output 0 within 1 tick of the F line, drive brake, no restart."""
    before, step0 = step_start(ctx, 'fault_stop_start')
    ctx.ensure_armed()
    loop = DriveLoop(ctx.client, 30, 150).start()
    try:
        ctx.obs.spin_sim_for(1.5)
        loop.set(30, -150)
        ctx.obs.spin_sim_for(.3)
        mark = ctx.mark('fault_stop')
        reply = ctx.hooks.event(kind='adc_set', arg=0)
        ctx.obs.spin_sim_for(1.)
    finally:
        loop.stop()
    ctx.hooks.event(kind='adc_model')
    ctx.obs.spin_sim_for(1.)
    after = emulator_counters(ctx.hooks)
    validity = step_validity(ctx, clock, step0['mono'], time.monotonic(), before, after,
                             sim_made=step_sim_latches(ctx, step0, clock) if clock == 'wall' else ())
    emu, _ = ctx.logs(mark)
    t = reply['at_tick']
    # the first F line after the trigger: raw 0 enters the noise window at once, so POT_NOISE (tick t) comes two
    # ticks before POT_OPEN (median of 5) - both are latched faults with the same stop action (spec 6.4, 6.7)
    line = first(entries(emu, 'm2h'), lambda e: e['tick'] >= t and e['text'].startswith('F '))
    open_line = f_line(emu, 'POT_OPEN', t)
    off = pin_first(emu, 'steer_en', lambda v: v == 0, t)
    pwm0 = pin_first(emu, 'steer_pwm_level', lambda v: v == 0, t)
    zero = drive_zero_tick(emu, t)
    restart = [e for e in entries(emu, 'pin') if e['channel'] in ('drive_l', 'drive_r') and e['value'] != 0 and
               zero is not None and e['tick'] > zero]
    criteria = [criterion('steer_output_zero_ticks_after_fault', None if (line is None or off is None) else
                          off['tick'] - line['tick'], '<= 1', line is not None and off is not None and
                          0 <= off['tick'] - line['tick'] <= 1),
                criterion('drive_brake_at_fault', None if (zero is None or line is None) else zero - line['tick'], '<= 1',
                          zero is not None and line is not None and 0 <= zero - line['tick'] <= 1),
                criterion('no_automatic_restart', len(restart), 0, not restart)]
    criteria = invalidate(criteria, validity)
    return dict(name='fault_stop_pot_open', criteria=criteria, passed=verdict(criteria), trigger_tick=t,
                validity=validity,
                f_line=None if line is None else line['text'], f_tick=None if line is None else line['tick'],
                pot_open_line=None if open_line is None else open_line['text'],
                pot_open_tick=None if open_line is None else open_line['tick'],
                steer_en_off_tick=None if off is None else off['tick'],
                steer_pwm_zero_tick=None if pwm0 is None else pwm0['tick'])


def mandatory_faults(ctx):
    """Mandatory fault tests of design 7.2.1 that need no executor (report each)."""
    out = {}
    # two clients: a second owner can neither arm nor drive, but its STOP is accepted
    ctx.ensure_armed()
    other = ControlClient(owner='p6a-second-screen')
    out['second_client_arm'] = other.arm().get('error')
    other.lease, other.ticket, other.seq = ctx.client.lease, ctx.client.ticket, ctx.client.seq
    out['second_client_drive'] = other.drive(30, 0).get('error')
    out['second_client_stop'] = other.stop('brake')
    ctx.obs.spin_for(.5)
    out['after_second_stop_state'] = ctx.host().get('state')
    # late drive after STOP with the old ticket
    ctx.ensure_armed()
    lease = (ctx.client.lease, ctx.client.ticket, ctx.client.seq)
    ctx.client.stop('brake')
    ctx.client.lease, ctx.client.ticket, ctx.client.seq = lease
    out['late_drive_after_stop'] = ctx.client.drive(40, 0).get('error')
    # half frame, CRC error (one line), range error (latch), short and long status line loss
    for name, hook, kwargs in (('half_frame', 'half_frame', {}), ('crc_error_one_line', 'corrupt',
                                                                    dict(kind='crc', count=1)),
                               ('range_error', 'corrupt', dict(kind='range', count=1)),
                               ('status_loss_120ms', 'status_loss', dict(duration_ms=120)),
                               ('status_loss_400ms', 'status_loss', dict(duration_ms=400))):
        ctx.ensure_armed()
        loop = DriveLoop(ctx.client, 30, 0).start()
        try:
            ctx.obs.spin_sim_for(1.)
            mark = ctx.mark(name)
            getattr(ctx.hooks, hook)(**kwargs)
            ctx.obs.spin_sim_for(1.)
        finally:
            loop.stop()
        status = ctx.host()
        emu, _ = ctx.logs(mark)
        refusals = [e['text'] for e in entries(emu, 'm2h') if e['text'].startswith('N ')]
        out[name] = dict(state=status.get('state'), reason=status.get('reason'), faults=status['steering']['faults'],
                         mcu_refusals=refusals[:5], loop_errors=dict(loop.errors))
        ctx.client.stop('brake')
        ctx.obs.spin_sim_for(.5)
    # MCU reset while driving
    ctx.ensure_armed()
    loop = DriveLoop(ctx.client, 30, 0).start()
    try:
        ctx.obs.spin_sim_for(1.)
        mark = ctx.mark('mcu_reset')
        ctx.hooks.event(kind='mcu_reset', arg=4)
        ctx.obs.spin_sim_for(2.)
    finally:
        loop.stop()
    status = ctx.host()
    emu, _ = ctx.logs(mark)
    out['mcu_reset_while_driving'] = dict(state=status.get('state'), faults=status['steering']['faults'],
                                          banners=[e['text'][:60] for e in entries(emu, 'm2h')
                                                   if e['text'].startswith('UBTS_STEER_V3 hello')],
                                          drive_restart=[e['tick'] for e in entries(emu, 'pin') if e['channel'] ==
                                                         'drive_l' and e['value'] != 0])
    # USB unplug / replug with the MCU powered from USB
    ctx.ensure_armed()
    loop = DriveLoop(ctx.client, 30, 0).start()
    try:
        ctx.obs.spin_sim_for(1.)
        mark = ctx.mark('usb_unplug')
        ctx.hooks.unplug(duration_ms=2500, mcu_power='cycle')
        ctx.obs.spin_sim_for(5.)
    finally:
        loop.stop()
    status = ctx.host()
    emu, _ = ctx.logs(mark)
    out['usb_unplug_replug'] = dict(state=status.get('state'), faults=status['steering']['faults'],
                                    port_events=[{k: e.get(k) for k in ('tick', 'host_open', 'unplugged', 'replugged')}
                                                 for e in entries(emu, 'port')],
                                    drive_restart=[e['tick'] for e in entries(emu, 'pin') if e['channel'] == 'drive_l'
                                                   and e['value'] != 0 and e['tick'] > mark['tick'] + 50])
    ctx.client.stop('brake')
    return dict(name='mandatory_fault_tests', passed=None, report=out,
                not_run=dict(system_time_change='needs CAP_SYS_TIME in the container; the host clock is monotonic '
                                                '(P3 unit test: a wall clock jump does not move deadlines)',
                             key_held_during_link_loss='needs the steering screen (P6b); the link-loss test drives '
                                                       'the same path from the scripted client',
                             executor_items='STOP and deadman reactions of the executor, its motion op and '
                                            'SteerOutputGuard: next phase (executor container)'))


def check_safety(ctx, args):
    results = dict(stops=[stop_once(ctx, i, clock=args.clock) for i in range(args.repeat)])
    stops = results['stops']
    valid = [s for s in stops if s['validity']['valid']]
    complete = len(valid) == len(stops)
    lat = [s['stop_send_latency_s'] for s in valid if s['stop_send_latency_s'] is not None]
    host = [s['host_confirm_from_tx_s'] for s in valid if s['host_confirm_from_tx_s'] is not None]
    zero = [s['drive_zero_ticks'] for s in valid if s['drive_zero_ticks'] is not None]
    conf = [s['confirm_line_ms'] for s in valid if s['confirm_line_ms'] is not None]
    hold = [s['hold_max_dev_deg'] for s in valid if s['hold_max_dev_deg'] is not None]
    info, params = ctx.session_params()
    db_deg = (params['db'] / info['calibration']['scale_permille_per_deg']) if params else .5

    def judged(values, rule):
        # every STOP valid (A10) and measured: judge; an invalid STOP leaves the item unjudged (rerun)
        if not complete:
            return None
        return len(values) == len(stops) and rule(values)
    note = None if complete else '%d of %d STOP tests valid (A10): not judged' % (len(valid), len(stops))
    criteria = [
        criterion('stop_request_to_stop_bytes_s', lat, '<= 0.020 (bridge clock %s)' % args.clock,
                  judged(lat, lambda v: max(v) <= BRIDGE_TICK_S + 1e-9),
                  note or 'request receipt (socket thread, bridge clock) to the STOP write (bridge clock)'),
        criterion('mcu_stop_to_drive_zero_ticks', zero, '<= 2 (1 servo tick at 500 Hz)',
                  judged(zero, lambda v: max(v) <= 2), note),
        criterion('stop_confirm_host_s', host, '<= %.3f (2 status periods, STOP bytes to the host confirmation)'
                  % (STOP_CONFIRM_PERIODS * STATUS_PERIOD_MS / 1000.),
                  judged(host, lambda v: max(v) <= STOP_CONFIRM_PERIODS * STATUS_PERIOD_MS / 1000. + 1e-9), note),
        criterion('stop_confirm_status_line_ms', conf, 'report (MCU side: the first DISARMED zero-duty status line '
                  'after the STOP arrival)', None),
        criterion('hold_error_deg', hold, '<= max(1, 2 x deadband %.2f)' % (2 * db_deg),
                  judged(hold, lambda v: max(v) <= max(1., 2 * db_deg)), note),
        criterion('no_restart_after_stop', [s['restart_pin_changes'] for s in stops], 0,
                  all(s['restart_pin_changes'] == 0 for s in stops)),
    ]
    results['stop_unconfirmed'] = stop_unconfirmed(ctx, clock=args.clock)
    results['link_loss'] = link_loss(ctx, clock=args.clock)
    results['fault_stop'] = fault_stop(ctx, clock=args.clock)
    results['mandatory'] = mandatory_faults(ctx)
    all_criteria = criteria + results['stop_unconfirmed']['criteria'] + results['link_loss']['criteria'] + \
        results['fault_stop']['criteria']
    false = merge_false_detections([s['false_detection'] for s in stops])
    false.update(by_stop=[s['false_detection'] for s in stops],
                 rule='normal STOP windows only (the fault steps expect their faults)')
    invalid = [name for name, item in (('stops', None if complete else False),
                                       ('stop_unconfirmed', results['stop_unconfirmed']['validity']['valid']),
                                       ('link_loss', results['link_loss']['validity']['valid']),
                                       ('fault_stop', results['fault_stop']['validity']['valid'])) if item is False]
    return dict(check='A17_G', clock=args.clock, criteria=all_criteria, passed=verdict(all_criteria),
                stop_criteria=criteria, results=results, deadband_deg=db_deg, false_detection=false,
                invalid_steps=invalid, validity_rule='A10: wall clock steps with a 5 s RTF below %.1f or an emulator '
                                                     '/clock wall gap above %d ms are not judged' %
                (RTF_VALID_MIN, EMULATOR_STALL_MS),
                executor_items='next phase: STOP / deadman / TTL with the executor container in the loop, the '
                               'executor\'s SteerOutputGuard and motion op, a full row with the executor')


# --- A19 ----------------------------------------------------------------------------------------------------------------------------
RECORD_SECONDS = 60.
RULE_TOLERANCE_TICKS = 1                 # A19: the judgement tick is the design value +-1 tick (A3 wording)


def fault_case(ctx, name, code=None, *, restart=None, arm=True, drive=None, trigger=None, trigger_after_s=1.,
               observe_s=2., expected=None, condition_bit=None, host_latch=None, line_prefix=None, arm_error=None,
               rule=None, timing='judged'):
    """One A19 case (a new session first when restart is given). drive: (throttle, steer_mrad) of the case's one
    DriveLoop (ctx.loop; triggers change it with ctx.loop.set). trigger(ctx) -> trigger tick (None: the restart or
    the injection itself). expected(trigger_tick, emu) -> (tick, tolerance, rule). line_prefix: an MCU line other
    than F (W for the watchdog). arm_error: the arm is expected to be refused with this code. With --cases, a case
    outside the selection is not run (a marker that check_faults drops).
    rule: (code, 'trip' | 'change'): the new session starts through the record hook (report_adc on) and the expected
    tick is the rule tick of p6a_fw_rules over that record (+-1 tick); 'change' is a condition bit (SOFTLIMIT): the
    bit must show in the first status line at or after the tick the condition starts. timing 'estimate': the expected
    tick is an estimate (reported, not judged). In the wall clock mode an invalid case (A10) leaves its timing items
    unjudged."""
    selected = getattr(ctx, 'selected', None)
    if selected and name not in selected:
        return dict(name=name, skipped=True)
    record_path = None
    if restart is not None:
        if rule is not None:
            record_path = '/runtime/steer/a19_record_%s.json' % name
            try:
                os.unlink(record_path)
            except OSError:
                pass
            ctx.hooks.record(seconds=RECORD_SECONDS, path=record_path, report_adc=True, **restart)
        else:
            ctx.hooks.restart(**restart)
        ctx.obs.spin_for(2.)
    before = emulator_counters(ctx.hooks)
    mark = ctx.mark(name)
    ctx.loop = None
    arm_reply = None
    try:
        if arm:
            try:
                arm_reply = ctx.ensure_armed()
            except ClientError as exc:
                arm_reply = dict(ok=False, error=str(exc))
        if drive is not None and arm_reply and arm_reply.get('ok'):
            ctx.loop = DriveLoop(ctx.client, *drive).start()
        ctx.obs.spin_sim_for(trigger_after_s)
        trigger_tick = trigger(ctx) if trigger is not None else None
        ctx.obs.spin_sim_for(observe_s)
    finally:
        if ctx.loop is not None:
            ctx.loop.stop()
            ctx.loop = None
    status = ctx.host()
    after_counters = emulator_counters(ctx.hooks)
    validity = step_validity(ctx, getattr(ctx, 'clock', 'wall'), mark['mono'], time.monotonic(), before,
                             after_counters)
    emu, _ = ctx.logs(mark)
    after = mark['tick'] if trigger_tick is None else trigger_tick
    trace_info = None
    if record_path is not None:
        try:
            ctx.hooks.record_finish()
            trace = p6a_fw_rules.FirmwareTrace(load_json(record_path), at_tick=trigger_tick)
            rule_tick = (trace.first_trip(rule[0], trigger_tick) if rule[1] == 'trip'
                         else trace.first_change(rule[0], trigger_tick))
            trace_info = dict(trace.summary(), rule=list(rule), rule_tick=rule_tick, record=record_path)
            expected = (lambda t, emu, tick=rule_tick: (tick, RULE_TOLERANCE_TICKS,
                                                       'p6a_fw_rules %s %s from the record' % tuple(rule)))
        except (OSError, ValueError, KeyError, ClientError, p6a_fw_rules.TraceError) as exc:
            trace_info = dict(error='%s: %s' % (type(exc).__name__, exc), record=record_path)
            expected = (lambda t, emu: (None, RULE_TOLERANCE_TICKS, 'p6a_fw_rules: %s' % trace_info['error']))
    if line_prefix is not None:
        line = first(entries(emu, 'm2h'), lambda e: e['tick'] >= after and e['text'].startswith(line_prefix))
    elif code:
        line = f_line(emu, code, after)
    else:
        line = None
    first_f = first(entries(emu, 'm2h'), lambda e: e['tick'] >= after and e['text'].startswith('F '))
    exp = expected(trigger_tick, emu) if expected is not None else None
    lines = status_lines(emu)
    bit_seen = first(lines, lambda e: e['status']['fl'] & condition_bit) if condition_bit is not None else None
    state_after = first(lines, lambda e: line is not None and e['tick'] > line['tick'])
    criteria, timing_criteria = [], []
    if code or line_prefix:
        criteria.append(criterion('mcu_line', None if line is None else line['text'], line_prefix or 'F %s ...' % code,
                                  line is not None))
    if exp is not None and rule is not None and rule[1] == 'change':
        # a condition bit: the first status line at or after the rule tick shows it, none before
        due = first(lines, lambda e: exp[0] is not None and e['tick'] >= exp[0])
        early = [e['tick'] for e in lines if exp[0] is not None and e['tick'] < exp[0] and
                 e['status']['fl'] & condition_bit and e['tick'] >= after]
        ok = (exp[0] is not None and due is not None and bit_seen is not None and bit_seen['tick'] == due['tick']
              and not early)
        timing_criteria.append(criterion('condition_bit_in_the_first_status_line_after_the_rule_tick',
                                         None if bit_seen is None or exp[0] is None else
                                         dict(rule_tick=exp[0], first_line_after=None if due is None else due['tick'],
                                              bit_line=bit_seen['tick'], early_lines=early[:3]),
                                         'bit line = first status line at or after the rule tick (%s)' % exp[2], ok))
    elif exp is not None:
        measured = line['tick'] if line is not None else (bit_seen['tick'] if bit_seen else None)
        judged = timing == 'judged'
        name_c = 'judgement_tick_minus_expected' if judged else 'judgement_tick_minus_estimate'
        if exp[0] is None:
            timing_criteria.append(criterion(name_c, None, '|d| <= %s (%s)' % (exp[1], exp[2]),
                                             False if judged else None,
                                             note='expected tick not found in the emulator log'))
        else:
            ok = measured is not None and abs(measured - exp[0]) <= exp[1]
            timing_criteria.append(criterion(name_c, None if measured is None else measured - exp[0],
                                             '|d| <= %s (%s)' % (exp[1], exp[2]), ok if judged else None,
                                             None if judged else 'report: the rule tick is an estimate (the servo '
                                                                 'ramp cmd is not in the record); timing not judged'))
    criteria += invalidate(timing_criteria, validity)
    if condition_bit is not None:
        criteria.append(criterion('condition_bit_in_status', None if bit_seen is None else '%04X' %
                                  bit_seen['status']['fl'], '%04X set' % condition_bit, bit_seen is not None))
    if line is not None and code not in ('TTL',) and line_prefix != 'UBTS_STEER_V3 hello' and state_after is not None:
        criteria.append(criterion('mcu_state_after', state_after['status']['st'], 1, state_after['status']['st'] == 1))
    if host_latch is not None:
        criteria.append(criterion('host_reports', status['steering']['faults'], host_latch,
                                  host_latch in status['steering']['faults'] or host_latch == status.get('last_error')))
    if arm_error is not None:
        got = (arm_reply or {}).get('error') or ''
        criteria.append(criterion('arm_refused', got, arm_error, arm_error in got))
    return dict(name=name, code=code, criteria=criteria, passed=verdict(criteria), trigger_tick=trigger_tick,
                timing=timing if exp is not None else None, validity=validity, trace=trace_info,
                line_tick=None if line is None else line['tick'], line_text=None if line is None else line['text'],
                first_f_line=None if first_f is None else dict(tick=first_f['tick'], text=first_f['text']),
                expected=None if exp is None else dict(tick=exp[0], tolerance=exp[1], rule=exp[2]),
                arm=None if arm_reply is None else {k: arm_reply.get(k) for k in ('ok', 'error')},
                host_state=status.get('state'), host_faults=status['steering']['faults'],
                last_status=None if not lines else lines[-1]['status'])


def pushes_after(emu, tick, level=1):
    """First steering PWM level transition to at least level at or after tick (pin log)."""
    if tick is None:
        return None
    return first(entries(emu, 'pin', channel='steer_pwm_level'), lambda e: e['tick'] >= tick and e['value'] >= level)


def fresh_session(reason, injections=(), record_overrides=None):
    """restart hook arguments of a new, calibrated session (clears MCU latches such as POLARITY / DRV_STUCK)."""
    return dict(reason=reason, injections=list(injections), eeprom='calibrated', cal_id=1,
                record_overrides=dict(record_overrides or {}))


def check_faults(ctx, args):
    ctx.selected = set(getattr(args, 'cases', None) or ())
    info, params = ctx.session_params()
    p = params or {}
    med_delay = (p.get('med', 5) - 1) // 2
    cases = []

    def event(kind, arg=0):
        return lambda c: c.hooks.event(kind=kind, arg=arg)['at_tick']

    def fixed(delay, rule):
        return lambda t, emu: (None if t is None else t + delay, 1, rule)

    def raw_now(c):
        # the model raw of the last physics step (hook status, answered in the clock callback); the status file is
        # up to 0.5 s old (dev run 2026-11-09: a pattern around a stale raw gave a POT_JUMP of 9 at its start)
        return c.hooks.status()['status']['truth']['pot_raw']

    def steer_to(value):
        def run(c):
            tick = c.mark('steer_to_%d' % value)['tick']
            c.loop.set(0, value)
            return tick
        return run

    def motor_off_then(value):
        def run(c):
            c.hooks.event(kind='motor_power', arg=0)
            return steer_to(value)(c)
        return run

    def frozen_step(value):
        def run(c):
            c.hooks.event(kind='motor_power', arg=0)
            c.hooks.event(kind='adc_set', arg=raw_now(c))
            return steer_to(value)(c)
        return run

    def first_push(level, delay_key, default, rule):
        def run(t, emu):
            push = pushes_after(emu, t, level)
            return (None if push is None else push['tick'] + p.get(delay_key, default), 2, rule)
        return run

    # potentiometer faults (scripted ADC through the emulator hook; adc_model gives the model value back)
    cases.append(fault_case(ctx, 'pot_open', 'POT_OPEN', restart=fresh_session('a19_start'), drive=(30, 100),
                            trigger=event('adc_set', 0), expected=fixed(med_delay, 'raw 0 from t: t + (med-1)/2'),
                            host_latch='POT_OPEN'))
    ctx.hooks.event(kind='adc_model')
    cases.append(fault_case(ctx, 'pot_short', 'POT_SHORT', drive=(30, 100), trigger=event('adc_set', 1023),
                            expected=fixed(med_delay, 'raw 1023 from t: t + (med-1)/2'), host_latch='POT_SHORT'))
    ctx.hooks.event(kind='adc_model')
    cases.append(fault_case(ctx, 'pot_jump', 'POT_JUMP', drive=(30, 0),
                            trigger=lambda c: c.hooks.event(kind='adc_set', arg=min(990, raw_now(c) + 4 * p.get(
                                'jmp', 8)))['at_tick'], expected=fixed(med_delay, 'the median moves at t + (med-1)/2'),
                            host_latch='POT_JUMP'))
    ctx.hooks.event(kind='adc_model')

    def noise(c):
        # +-(jmp - 2) alternating: sd jmp - 2 > nsd / 10 (POT_NOISE) while the median step at the start of the
        # pattern stays within jmp with the model's ADC noise (no POT_JUMP)
        r, d = raw_now(c), p.get('jmp', 8) - 2
        return c.hooks.adc_pattern(values=[r - d, r + d], duration_ms=300)['start_tick']
    # POT_NOISE while DISARMED in a new session (the pot detectors run in every state; an armed servo sees the
    # alternating position as a drift under brake and can latch DRV_STUCK first, reported below)
    cases.append(fault_case(ctx, 'pot_noise', 'POT_NOISE', restart=fresh_session('a19_pot_noise'), arm=False,
                            trigger=noise, rule=('POT_NOISE', 'trip'), host_latch='POT_NOISE'))
    ctx.hooks.event(kind='adc_model')
    cases.append(fault_case(ctx, 'pot_noise_armed', None, restart=fresh_session('a19_pot_noise_armed'), drive=(30, 0),
                            trigger=noise, observe_s=1.))
    cases[-1]['passed'] = None
    cases[-1]['note'] = 'report: the codes of the same noise pattern with the servo armed'
    ctx.hooks.event(kind='adc_model')
    # SOFTLIMIT: a slow scripted ramp past the soft limit while DISARMED (an armed servo sees a drift under brake
    # and latches DRV_STUCK); the condition bit only, no state change
    cases.append(fault_case(ctx, 'softlimit', None, restart=fresh_session('a19_softlimit'), arm=False,
                            trigger=lambda c: c.hooks.adc_ramp(to=softlimit_raw(c), counts_per_tick=.2)['start_tick'],
                            observe_s=4., condition_bit=0x0080, rule=('SOFTLIMIT', 'change')))
    ctx.hooks.event(kind='adc_model')
    # actuator faults. POLARITY: v1 model injection; the rule tick (run_pol >= polt, wrong-way motion > polp) comes
    # from the record of the case (p6a_fw_rules), +-1 tick (review RG-02; the earlier 1.5 s window is gone).
    cases.append(fault_case(ctx, 'polarity', 'POLARITY', restart=fresh_session('a19_polarity', injections=[
        dict(kind='polarity_reversed', at_s=3., params={})]), drive=(0, 0), trigger_after_s=2.,
        trigger=steer_to(250), observe_s=2., rule=('POLARITY', 'trip'), host_latch='POLARITY'))
    # DRV_STUCK: the symptom through the scripted ADC (the conformance script fault_drv_stuck_default): the position
    # drifts while the servo brakes in its deadband (1 count every 9 ticks)
    cases.append(fault_case(ctx, 'drv_stuck', 'DRV_STUCK', restart=fresh_session('a19_drv_stuck'), drive=(0, 0),
                            trigger_after_s=1., trigger=lambda c: c.hooks.adc_ramp(to=raw_now(c) + 40,
                                                                                    counts_per_tick=1. / 9.)['start_tick'],
                            observe_s=3., rule=('DRV_STUCK', 'trip'), host_latch='DRV_STUCK'))
    ctx.hooks.event(kind='adc_model')
    # the model's stuck driver (v1 driver_stuck_on): reported (which code the firmware gives first)
    cases.append(fault_case(ctx, 'drv_stuck_model', None, restart=fresh_session('a19_drv_stuck_model', injections=[
        dict(kind='driver_stuck_on', at_s=4., params=dict(pwm=.6))]), drive=(0, 0), trigger_after_s=1.5,
        observe_s=4.))
    cases[-1]['passed'] = None
    cases[-1]['note'] = 'report: the code the firmware latches first for a stuck driver of the model'
    # STALL / TRACK: motor power off and a steering step; the ADC is frozen at its value (scripted, as the
    # conformance scripts do) because the model's pot noise (about 10 permille per count) is near stm (20)
    cases.append(fault_case(ctx, 'stall', 'STALL', restart=fresh_session('a19_stall'), drive=(0, 0),
                            trigger=frozen_step(350), observe_s=2.5, rule=('STALL', 'trip'), host_latch='STALL'))
    ctx.hooks.event(kind='motor_power', arg=1)
    ctx.hooks.event(kind='adc_model')
    cases.append(fault_case(ctx, 'track', 'TRACK', restart=fresh_session('a19_track', record_overrides=dict(maxp=150)),
                            drive=(0, 0), trigger=frozen_step(350), observe_s=2.5,
                            expected=lambda t, emu: track_expected(emu, t, p), host_latch='TRACK', timing='estimate'))
    ctx.hooks.event(kind='motor_power', arg=1)
    ctx.hooks.event(kind='adc_model')
    # the same two with the model's ADC noise: reported (STALL may come late, TRACK may not come before the host's
    # steering_not_following latch of 1.5 s)
    cases.append(fault_case(ctx, 'stall_model_noise', None, restart=fresh_session('a19_stall_noise'), drive=(0, 0),
                            trigger=motor_off_then(350), observe_s=2.5))
    cases[-1]['passed'] = None
    cases[-1]['note'] = 'report: STALL with the model ADC noise'
    ctx.hooks.event(kind='motor_power', arg=1)
    cases.append(fault_case(ctx, 'track_model_noise', None, restart=fresh_session('a19_track_noise',
                                                                                  record_overrides=dict(maxp=150)),
                            drive=(0, 0), trigger=motor_off_then(350), observe_s=3.))
    cases[-1]['passed'] = None
    cases[-1]['note'] = 'report: TRACK with the model ADC noise'
    ctx.hooks.event(kind='motor_power', arg=1)
    # THERMAL: CFG tha 50, thw 1 (the conformance script fault_thermal_release) so that it comes before the host's
    # 1.5 s not-following latch; a large steering step saturates the servo output
    cases.append(fault_case(ctx, 'thermal', 'THERMAL', restart=fresh_session('a19_thermal', record_overrides=dict(
        tha=50, thw=1)), drive=(0, 0), trigger=steer_to(380), observe_s=1.5, host_latch='THERMAL',
        rule=('THERMAL', 'trip')))
    # link and serial faults
    cases.append(fault_case(ctx, 'ttl', 'TTL', restart=fresh_session('a19_link'), drive=(30, 0),
                            trigger=lambda c: c.hooks.link(direction='to_mcu', duration_ms=400)['start_tick'],
                            observe_s=1.5, expected=lambda t, emu: ttl_expected(emu, t)))
    cases.append(fault_case(ctx, 'watchdog', None, line_prefix='W ', drive=(30, 0),
                            trigger=lambda c: c.hooks.link(direction='to_mcu', duration_ms=900)['start_tick'],
                            observe_s=1.5, expected=lambda t, emu: watchdog_expected(emu, t, p.get('wd', 500)),
                            host_latch='mcu_disarmed'))
    cases.append(fault_case(ctx, 'crc', 'CRC', drive=(30, 0),
                            trigger=lambda c: c.hooks.corrupt(kind='crc', count=p.get('crcn', 5))['tick'],
                            observe_s=1.5, expected=lambda t, emu: crc_expected(emu, t, p.get('crcn', 5))))
    cases.append(fault_case(ctx, 'boot', None, line_prefix='UBTS_STEER_V3 hello', drive=(30, 0),
                            trigger=event('mcu_reset', 3), observe_s=2., condition_bit=0x4000,
                            host_latch='mcu_reset_detected'))
    cases.append(fault_case(ctx, 'nocal', None, restart=dict(reason='a19_nocal', eeprom='blank'), trigger=None,
                            observe_s=1., condition_bit=0x0001, arm_error='steering_uncalibrated'))
    # host faults
    cases.append(fault_case(ctx, 'host_status_stale', None, restart=fresh_session('a19_host'), drive=(30, 0),
                            trigger=lambda c: c.hooks.status_loss(duration_ms=400)['start_tick'], observe_s=1.,
                            host_latch='steering_status_stale'))
    cases.append(fault_case(ctx, 'host_calibration_mismatch', None, restart=dict(fresh_session('a19_cal2'), cal_id=2),
                            trigger=None, observe_s=.5, arm_error='calibration_mismatch'))
    cases.append(fault_case(ctx, 'host_c_rejected', None, restart=fresh_session('a19_host2'), drive=(30, 0),
                            trigger=lambda c: c.hooks.corrupt(kind='range', count=1)['tick'], observe_s=1.,
                            host_latch='c_rejected'))
    cases.append(fault_case(ctx, 'host_not_following', None, restart=fresh_session('a19_slow', injections=[
        dict(kind='slow', at_s=0., params=dict(rate_scale=.3))]), drive=(0, 0), trigger=steer_to(300),
        observe_s=2.5, host_latch='steering_not_following'))
    ctx.hooks.restart(**fresh_session('a19_end'))
    ctx.obs.spin_for(2.)
    cases = [c for c in cases if not c.get('skipped')]
    criteria = [criterion(c['name'], c['passed'], True, c['passed']) for c in cases]
    timing = dict(judged_with_rule_tick=sorted(c['name'] for c in cases if c.get('timing') == 'judged'),
                  estimate_not_judged=sorted(c['name'] for c in cases if c.get('timing') == 'estimate'),
                  invalid_cases=sorted(c['name'] for c in cases if not (c.get('validity') or {}).get('valid', True)),
                  tolerance_ticks=RULE_TOLERANCE_TICKS,
                  rules='p6a_fw_rules over a record of the case: POT_NOISE, SOFTLIMIT (condition bit), POLARITY, '
                        'DRV_STUCK, STALL, THERMAL; fixed rules: POT_OPEN / POT_SHORT / POT_JUMP (t + (med-1)/2), '
                        'TTL (last C + ttl), WATCHDOG (last input + wd + 1), CRC (crcn-th bad line); TRACK estimate')
    return dict(check='A19_G', criteria=criteria, passed=verdict(criteria), cases=cases, cfg=params,
                timing=timing, selection=sorted(ctx.selected) or 'all',
                false_detection_note='normal-script false detections are counted by the startup, verify, drive, '
                                     'safety (STOP windows) and a25 checks (a19g_normal summary)',
                not_run=dict(steering_ineffective='needs the executor motion op (next phase)',
                             sim_identity_and_hw_mismatch='in the identity check (A14)',
                             stop_unconfirmed='in the safety check (A17)'))


def watchdog_expected(emu, start, wd):
    before = [e for e in read_jsonl(EMULATOR_LOG) if e['kind'] == 'h2m' and e.get('note') == 'host' and
              e.get('last') is not None and start is not None and e['last'] < start + 1 and
              e['text'].startswith(('C ', 'PING'))]
    if not before:
        return (None, 1, 'last input + wd + 1')
    return (before[-1]['last'] + wd + 1, 1, 'last input (C or PING) + wd + 1')


def crc_expected(emu, start, crcn):
    bad = [e for e in entries(emu, 'h2m') if str(e.get('note', '')).endswith('crc_error') and e['tick'] >= (start or 0)]
    if len(bad) < crcn:
        return (None, 1, 'arrival of the crcn-th bad line')
    return (bad[crcn - 1]['last'], 1, 'arrival of the %d-th consecutive bad line' % crcn)


def ttl_expected(emu, start):
    before = [e for e in read_jsonl(EMULATOR_LOG) if e['kind'] == 'h2m' and e.get('note') == 'host' and
              e.get('last') is not None and start is not None and e['last'] < start + 1 and e['text'].startswith('C ')]
    if not before:
        return (None, 1, 'last C arrival + ttl')
    c = parse_c_line(before[-1]['text'])
    return (before[-1]['last'] + c['ttl'], 1, 'last C arrival + ttl')


def lower_bound(push, delay, window, rule):
    """(middle tick, half window, rule) for a code whose tick depends on the dynamics: fault_case checks
    |measured - middle| <= half window, i.e. earliest <= measured <= earliest + window (earliest = push + delay)."""
    if push is None:
        return (None, window // 2, rule)
    return (push['tick'] + delay + window // 2, window // 2, rule)


def latch_tick(emu, code, after_tick):
    """Tick at which the firmware latched code (fault_bits pin log), or None."""
    hit = first(entries(emu, 'pin', channel='fault_bits'),
                lambda e: after_tick is not None and e['tick'] >= after_tick and code in (e.get('faults') or ()))
    return None if hit is None else hit['tick']


def run_start(emu, trigger_tick, level, delay, rule, code):
    """Start tick of the PWM level run (pin log of the level, at or above level) that is still on when the firmware
    latches code (fault_bits pin log; the pins of that tick already show the stop) + delay."""
    end = latch_tick(emu, code, trigger_tick)
    if trigger_tick is None or end is None:
        return (None, 2, rule)
    start = None
    for e in entries(emu, 'pin', channel='steer_pwm_level'):
        if e['tick'] < trigger_tick:
            continue
        if e['tick'] >= end:
            break
        if e['value'] >= level:
            if start is None:
                start = e['tick']
        else:
            start = None
    return (None if start is None else start + delay, 2, rule)


def thermal_expected(emu, trigger_tick, tha, thw):
    """THERMAL (spec 6.7): the end of each tick counts the tick as hot when |pwm| >= thp (pins of that tick) in a
    window of thw s; step 6 of the next tick trips once the count is above tha. The PWM of a saturating servo goes
    in and out of the hot level, so the hot ticks are summed from the PWM level log (level 3 = |pwm| >= thp)."""
    rule = 'tick after the (tha + 1)-th hot tick (|pwm| >= thp) in thw s; tha %d, thw %d' % (tha, thw)
    end = latch_tick(emu, 'THERMAL', trigger_tick)
    if trigger_tick is None or end is None:
        return (None, 2, rule)
    hot, start = [], None
    for e in entries(emu, 'pin', channel='steer_pwm_level'):
        if e['tick'] >= end:
            break
        if e['value'] >= 3 and start is None:
            start = e['tick']
        elif e['value'] < 3 and start is not None:
            hot.extend(range(start, e['tick']))
            start = None
    if start is not None:
        hot.extend(range(start, end))
    window = 1000 * thw
    for i, tick in enumerate(hot):
        k = tick + 1
        if sum(1 for h in hot[:i + 1] if h >= k - window) > tha:
            return (k, 2, rule)
    return (None, 2, rule)


def track_expected(emu, trigger_tick, p):
    """TRACK (spec 6.7, decision 12.1-69) with the ADC frozen at P: from the arrival of the stepped C line, cmd ramps
    from its value before the step (cmd0) toward the new target at the standstill rate rstd (permille per second,
    servo updates of 2 ms); runTrack starts once |cmd - P| > trk, i.e. after the travel P + (trk + 1) - cmd0 (step
    up; mirrored for a step down), and the code comes trt - 1 ticks later. cmd0 from the last status line before the
    arrival, P from the first one after it."""
    rule = 'step C arrival + ceil(1000 (P + trk + 1 - cmd0) / rstd) + trt - 1'
    c = first(entries(emu, 'h2m'), lambda e: trigger_tick is not None and e['tick'] >= trigger_tick and
              e['text'].startswith('C ') and (parse_c_line(e['text']) or {}).get('S', 0) != 0)
    if c is None:
        return (None, 5, rule)
    lines = status_lines(emu)
    before = [e for e in lines if e['tick'] < c['last']]
    after = first(lines, lambda e: e['tick'] > c['last'])
    if not before or after is None:
        return (None, 5, rule)
    cmd0, pos, target = before[-1]['status']['cmd'], after['status']['pos'], parse_c_line(c['text'])['S']
    trk = p.get('trk', 120)
    travel = (pos + trk + 1 - cmd0) if target > cmd0 else (cmd0 - (pos - trk - 1))
    cross = int(math.ceil(1000. * max(travel, 0) / p.get('rstd', 478)))
    return (c['last'] + cross + p.get('trt', 1000) - 1, 5, '%s; cmd0 %d, P %d' % (rule, cmd0, pos))


def softlimit_raw(ctx):
    info = ctx.hooks.status()['status']['session_info']['calibration']
    raw_l, raw_c, soft = info['raw_l'], info['raw_c'], info['soft_l']
    # permille = 1000 (raw - c) / (l - c) on the left side; 3 % beyond the soft limit
    return int(round(raw_c + (raw_l - raw_c) * (soft + 30) / 1000.))


# --- A3 ----------------------------------------------------------------------------------------------------------------------------
def check_a3(ctx, args):
    path = '/runtime/steer/a3_record.json'
    ctx.hooks.record(seconds=args.seconds, path=path, injections=[dict(kind='pot_jump', at_s=args.seconds * .7,
                                                                        params=dict(counts=40, duration_s=.3))])
    ctx.obs.spin_for(2.)
    try:
        ctx.ensure_armed()
        ctx.obs.spin_sim_for(args.seconds * .35)       # standstill
        loop = DriveLoop(ctx.client, 30, 0).start()
        try:
            for k in range(6):
                loop.set(30, (k % 3 - 1) * 200)
                ctx.obs.spin_sim_for(args.seconds * .05)
        finally:
            loop.stop()
        ctx.client.stop('brake')
        end = time.monotonic() + args.seconds * 3 + 30.
        while time.monotonic() < end and not Path(path).exists():
            ctx.obs.spin_for(.5)
    finally:
        ctx.client.stop('brake')
    if not Path(path).exists():
        return dict(check='A3_G', passed=False, error='record file not written')
    record = load_json(path)
    from .gz_esp_emulator import replay_record
    executable = record['session'].get('executable') or native_executable_path()
    replay = replay_record(record, executable=executable)
    # standstill raw noise (adc_read changes give every tick's analogRead value while report_adc is on)
    values = []
    for frame in record['frames']:
        for tick, channel, value in frame['out']['changes']:
            if channel == 'adc_read':
                values.append((tick, value))
    standstill = [v for t, v in values if 200 <= t < int(args.seconds * 350)]
    sd = statistics.pstdev(standstill) if len(standstill) > 10 else None
    criteria = [criterion('replay_same_responses', replay['mismatched_frames'], 0, replay['passed']),
                criterion('same_native_build', replay['same_build'], True, replay['same_build']),
                criterion('adc_read_values_compared', replay['adc_read_changes_compared'], '> 0',
                          replay['adc_read_changes_compared'] > 0,
                          'adc_read needs report_adc (UBTS_STEER_REPORT_ADC=1)'),
                criterion('standstill_raw_sd_counts', sd, 'report (profile pot_noise_sd_counts)', None)]
    return dict(check='A3_G', criteria=criteria, passed=verdict(criteria), replay=replay, frames=record['frame_count'],
                ticks=record['ticks'], standstill_samples=len(standstill), standstill_sd=sd,
                session=dict((k, record['session'].get(k)) for k in ('seed', 'boot_seed', 'build_id', 'firmware_build',
                                                                     'injections', 'profile_id')))


def native_executable_path():
    folder = os.environ.get('UBTS_STEER_BUILD_DIR')
    if not folder:
        return None
    try:
        return load_json(Path(folder) / 'BUILD_RESULT.json').get('path')
    except (OSError, ValueError):
        return None


# --- A9 / A10 ----------------------------------------------------------------------------------------------------------------
def rtf_windows(rows, warmup_s=20., window_s=5.):
    if len(rows) < 2:
        return []
    start = rows[0][0] + warmup_s
    out, i = [], 0
    rows = [r for r in rows if r[0] >= start]
    while rows and i < len(rows):
        j = i
        while j < len(rows) and rows[j][0] - rows[i][0] < window_s:
            j += 1
        if j >= len(rows):
            break
        out.append((rows[j][1] - rows[i][1]) / (rows[j][0] - rows[i][0]))
        i = j
    return out


def check_drive(ctx, args):
    """One open-loop drive: straight, then a constant steering angle, then STOP (A9), with the /clock RTF (A10)."""
    ctx.obs.spin_for(1.)
    ctx.wait_still(10.)
    ctx.ensure_armed()
    mark = ctx.mark('drive')
    start = ctx.obs.last_pose()
    t0 = ctx.obs.sim_time
    loop = DriveLoop(ctx.client, args.throttle, 0).start()
    try:
        ctx.obs.spin_sim_for(args.straight_s)
        t_straight = ctx.obs.sim_time
        loop.set(args.throttle, args.steer_mrad)
        ctx.obs.spin_sim_for(args.turn_s)
    finally:
        loop.stop()
    ctx.client.stop('brake')
    ctx.wait_still(10.)
    ctx.obs.spin_sim_for(max(0., args.hold_s))
    end = ctx.obs.last_pose()
    rows = odom_rows(ctx.obs, t0, ctx.obs.sim_time)
    straight = odom_rows(ctx.obs, t0, t_straight)
    yaw0 = start[3]
    c, s = math.cos(yaw0), math.sin(yaw0)

    def local(row):
        dx, dy = row[1] - start[1], row[2] - start[2]
        return c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(row[3] - yaw0), math.cos(row[3] - yaw0))
    final = local(end)
    lat_straight = [local(r)[1] for r in straight]
    windows = rtf_windows(list(ctx.obs.clock_rows), warmup_s=min(20., args.straight_s / 2.))
    emu, bridge = ctx.logs(mark)
    false = false_detections(emu, read_jsonl(BRIDGE_LOG), after_tick=mark['tick'],
                             after_wall=mark['wall'] - .01,
                             stalls=emulator_stalls(), slow=emulator_slow(), clock=args.clock)
    return dict(check='drive', scene=load_json(SIM_STATUS).get('scene'), final_x_m=final[0], final_y_m=final[1],
                false_detection=false,
                final_yaw_deg=math.degrees(final[2]), path_m=path_length(rows),
                straight_lateral_end_m=lat_straight[-1] if lat_straight else None,
                straight_lateral_max_m=max((abs(v) for v in lat_straight), default=None),
                rtf_windows=windows, rtf_median=statistics.median(windows) if windows else None,
                rtf_min=min(windows) if windows else None, loop_errors=dict(loop.errors),
                command=dict(throttle=args.throttle, steer_mrad=args.steer_mrad, straight_s=args.straight_s,
                             turn_s=args.turn_s), passed=None, result_kind=RESULT_KIND)


def render_path():
    """The rendering path of the sim service from the environment the stack shares (compose x-environment and the
    sim service's own variables are not visible here; the check service carries the shared ones), and the scene's
    renderer from the scene manifest."""
    env = {k: os.environ.get(k) for k in ('UBTS_RENDER_ENGINE', 'UBTS_HEADLESS_RENDERING', 'LIBGL_ALWAYS_SOFTWARE',
                                          'GALLIUM_DRIVER', 'UBTS_STEER_NO_CAMERA')}
    out = dict(environment=env)
    try:
        manifest = load_json('/runtime/steer/scene/scene_manifest.json')
        out['scene_renderer'] = manifest.get('renderer')
    except (OSError, ValueError):
        out['scene_renderer'] = None
    try:
        out['supervisor_sim_children'] = [c.get('command') for c in load_json('/runtime/steer/supervisor_sim.json')
                                          .get('children', []) if c.get('name') == 'gz']
    except (OSError, ValueError):
        out['supervisor_sim_children'] = None
    renderer = out['scene_renderer'] or {}
    out['path'] = ('software (%s)' % env['GALLIUM_DRIVER'] if env.get('LIBGL_ALWAYS_SOFTWARE') == '1' else
                   'GPU D3D12 (WSLg /dev/dxg)') + ', engine %s, headless %s' % (renderer.get('engine'),
                                                                               renderer.get('headless'))
    return out


def check_rtf(ctx, args):
    """A10 (amendment 2): the /clock rate against the wall clock with the whole stack running (GUI off, the scene
    camera of the stack), args.seconds (240) at standstill of which the first args.warmup_s (20) are left out, 5 s
    windows of delta SIM / delta wall (RTF source: this node's /clock receipts on its monotonic clock); topic rates
    per SIM second from the simulation_status counts; the rendering path; for a field world the clip node's counts."""
    def counts():
        status = load_json(SIM_STATUS)
        return time.monotonic(), status.get('sim_time'), dict(status.get('counts') or {})
    ctx.obs.spin_for(2.)
    ctx.obs.clock_rows.clear()
    clip0 = read_optional('/runtime/scan_clip_status.json')
    lidar0 = read_optional('/runtime/cpu_lidar_status.json')
    w0, s0, c0 = counts()
    ctx.obs.spin_for(args.seconds)
    w1, s1, c1 = counts()
    clip1 = read_optional('/runtime/scan_clip_status.json')
    lidar1 = read_optional('/runtime/cpu_lidar_status.json')
    windows = rtf_windows(list(ctx.obs.clock_rows), warmup_s=args.warmup_s, window_s=5.)
    sim_dt = (s1 - s0) if s0 is not None and s1 is not None else None
    rates = {k: (c1.get(k, 0) - c0.get(k, 0)) / sim_dt for k in sorted(c1)} if sim_dt else {}
    ordered = sorted(windows)
    status = load_json(SIM_STATUS)
    try:
        camera = load_json('/runtime/camera_driver_status.json')
        camera = {k: camera.get(k) for k in ('width', 'height', 'fps', 'state')}
    except (OSError, ValueError):
        camera = None
    clip = None
    if clip0 or clip1:
        clip = dict(start=clip0, end=clip1)
        if clip0 and clip1 and sim_dt:
            clip['published_per_sim_s'] = (clip1.get('published', 0) - clip0.get('published', 0)) / sim_dt
            clip['scans_per_sim_s'] = (clip1.get('scans', 0) - clip0.get('scans', 0)) / sim_dt
    lidar = None
    if lidar0 and lidar1 and sim_dt:
        lidar = dict(published_per_sim_s=(lidar1.get('published_scans', 0) - lidar0.get('published_scans', 0)) /
                     sim_dt, last_raycast_ms=lidar1.get('last_raycast_ms'))
    return dict(check='A10_rtf', scene=status.get('scene'), seconds=args.seconds, warmup_s=args.warmup_s,
                windows=len(windows), rtf_median=statistics.median(windows) if windows else None,
                rtf_min=ordered[0] if ordered else None, rtf_p10=ordered[len(ordered) // 10] if ordered else None,
                rtf_overall=(sim_dt / (w1 - w0)) if sim_dt else None, rtf_windows=windows, rates_per_sim_s=rates,
                camera=camera, gui=False, render=render_path(), clip_node=clip, lidar=lidar,
                rtf_source='this check node: /clock receipts against its monotonic clock (P5 used the gz '
                           'WorldStatistics of the server)',
                definition='amendment 2: %g s, the first %g s left out, 5 s windows, median' % (args.seconds,
                                                                                               args.warmup_s),
                passed=None, note='A10: W0 median >= 0.9 (P5), W1 report, W2 mode R (P7) first measurement; vehicle '
                                  'at standstill')


def read_optional(path):
    try:
        return load_json(path)
    except (OSError, ValueError):
        return None


# --- A19 (G) normal scripts: the stack start and the summary -------------------------------------------------------------
def check_startup(ctx, args):
    """A19 (G) false detections of the stack start (a normal script): from the start of the emulator and bridge logs
    to now. Expected: BOOT (a banner bit), and the port-open reset of the first port open."""
    ctx.obs.spin_for(LOG_FLUSH_WAIT_S)
    emu = read_jsonl(EMULATOR_LOG)
    bridge = read_jsonl(BRIDGE_LOG)
    false = false_detections(emu, bridge, stalls=emulator_stalls(), slow=emulator_slow(), clock=args.clock)
    sessions = sorted({e.get('session') for e in emu if e.get('session') is not None})
    states = [dict(wall=e.get('wall'), latches=e.get('latches'), stop_state=e.get('stop_state'),
                   reason=e.get('reason')) for e in bridge if e.get('kind') == 'state'][:12]
    try:
        emulator = ctx.steering_status()['emulator']
        counters = {k: int(emulator['counters'].get(k, 0))           # a counter never raised is absent (0)
                    for k in EMULATOR_CLOCK_COUNTERS + ('clock_wall_gap_max_us',)}
    except (OSError, ValueError, KeyError):
        counters = None
    criteria = [criterion('startup_false_detections', false['by_code'], 0, false['count'] == 0,
                          'A19(G) normal script: the stack start (R15-3)')]
    return dict(check='A19_G_startup', criteria=criteria, passed=verdict(criteria), false_detection=false,
                sessions=sessions, first_states=states, emulator_clock_since_start=counters,
                log_lines=dict(emulator=len(emu), bridge=len(bridge)))


def gsim_lockstep_false_detections(result):
    """F / W lines of a gz_probe a25_firmware result (G-SIM lockstep, firmware mode; every maneuver is a normal
    script: steps and sines of the steering target, no fault injection)."""
    rows = []
    summaries = ((result.get('run') or {}).get('controller_summaries') or {})
    for name, summary in sorted(summaries.items()):
        kinds = summary.get('line_kinds') or {}
        lines = [(tick, text) for tick, text in summary.get('first_non_status_lines') or []
                 if fault_code_of(text) is not None]
        rows.append(dict(maneuver=name, f_lines=int(kinds.get('F', 0)), w_lines=int(kinds.get('W', 0)),
                         codes=[dict(tick=t, code=fault_code_of(x)) for t, x in lines[:10]],
                         case=((result.get('vehicle_specs') or {}).get(name) or {}).get('tags')))
    return rows


def merge_false_detections(blocks):
    """One false_detection block from several (count, by_code, the SIM-made latches listed apart)."""
    by_code, stalled = {}, []
    for block in blocks:
        for code, n in (block.get('by_code') or {}).items():
            by_code[code] = by_code.get(code, 0) + n
        stalled.extend(block.get('sim_stall_latches') or [])
    return dict(count=sum(by_code.values()), by_code=by_code, sim_stall_latches=stalled[:20])


def summarize_normal(paths, gsim_paths=()):
    """A19 (G) normal-script false detections over P6a result files (each with a false_detection block) and G-SIM
    lockstep results (gz_probe a25_firmware). STALL false-detection rate per hour of normal SIM time; the
    steering_ineffective rate needs the executor's motion op (not measured)."""
    rows, by_code, sim_s, invalid = [], {}, 0., 0
    for path in paths:
        data = load_json(path)
        block = data.get('false_detection')
        if block is None:
            rows.append(dict(file=str(path), check=data.get('check'), missing=True))
            continue
        if block.get('by_code') is None and block.get('by_stop') is not None:     # an A17 file of the first fix run
            block = merge_false_detections(block['by_stop'])
        span = (data.get('emulator_clock') or {}).get('sim_s')
        if span:
            sim_s += span
        stalled = len(block.get('sim_stall_latches') or [])
        invalid += stalled
        rows.append(dict(file=str(path), check=data.get('check'), label=data.get('label'), clock=data.get('clock'),
                         count=block['count'], by_code=block.get('by_code'), sim_s=span,
                         sim_stall_latches=stalled))
        for code, n in (block.get('by_code') or {}).items():
            by_code[code] = by_code.get(code, 0) + n
    gsim = []
    for path in gsim_paths:
        result = load_json(path)
        for row in gsim_lockstep_false_detections(result):
            row['file'] = str(path)
            row['group'] = result.get('group')
            gsim.append(row)
            for item in row['codes']:
                by_code[item['code']] = by_code.get(item['code'], 0) + 1
        sim_s += float(result.get('duration_s') or 0.) * max(1, len((result.get('run') or {}).get(
            'controller_summaries') or {}))
    total = sum(by_code.values())
    stall = by_code.get('STALL', 0)
    criteria = [criterion('normal_script_false_detections', by_code, 0, total == 0 if (rows or gsim) else None,
                          'registered A19: 0 false detections in normal scripts (R15-3 definition)')]
    return dict(check='A19_G_normal', criteria=criteria, passed=verdict(criteria), total=total, by_code=by_code,
                p6a=rows, gsim_lockstep=gsim, normal_sim_s=sim_s,
                sim_stall_latches_not_counted=invalid,
                sim_stall_rule='wall clock mode: a steering_status_stale latch inside an emulator /clock wall stall '
                               'or a slow /clock interval is the SIM\'s own (A10 invalid); listed per file, not '
                               'counted',
                stall_false_detections=stall,
                stall_rate_per_hour=None if not sim_s else stall / sim_s * 3600.,
                steering_ineffective_rate='not measured: needs the executor motion op (next phase)',
                note='report items of A19: STALL and steering_ineffective false-detection rates')


# --- A25 (G, emulator path) ---------------------------------------------------------------------------------------------------
A25_P99_LIMIT_DEG = .2
A25_STEER_SPIKE_FACTOR = 1.5


def joint_errors(joints, truth, keys, lag_steps=0, step_s=CLOCK_STEP_S):
    """|joint angle (stamp T) - shared model angle at T - lag| per key in degrees; rows matched to 1 us."""
    by_time = {}
    for row in truth:
        if row.get('sim_time_s') is not None:
            by_time[int(round(row['sim_time_s'] * 1e6))] = row
    errors = {key: [] for key in keys}
    matched = 0
    for stamp, names in joints:
        t = int(round((stamp - lag_steps * step_s) * 1e6))
        row = by_time.get(t)
        if row is None:
            continue
        matched += 1
        for key, (joint, field) in keys.items():
            if joint in names:
                errors[key].append(math.degrees(abs(names[joint][0] - row[field])))
    return errors, matched


def percentile(values, q):
    ordered = sorted(values)
    if not ordered:
        return None
    k = (len(ordered) - 1) * q
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (k - lo)


def a25_summary(joints, truth, keys, lags=range(0, 4)):
    """Errors per delay (0..3 physics steps): matched samples, p99, median, max (deg) and per joint; the delay with
    the smallest p99; the largest |steering joint speed| (rad/s, finite samples)."""
    per_lag = {}
    for lag in lags:
        errors, matched = joint_errors(joints, truth, keys, lag_steps=lag)
        combined = [v for values in errors.values() for v in values]
        per_lag[lag] = dict(matched=matched, p99_deg=percentile(combined, .99), max_deg=max(combined, default=None),
                            median_deg=percentile(combined, .5),
                            per_joint={k: dict(p99_deg=percentile(v, .99), max_deg=max(v, default=None), n=len(v))
                                       for k, v in errors.items()})
    best = min((lag for lag in per_lag if per_lag[lag]['p99_deg'] is not None),
               key=lambda lag: per_lag[lag]['p99_deg'], default=None)
    speeds = [abs(names[j][1]) for _, names in joints for j, _ in keys.values() if j in names
              and names[j][1] is not None and math.isfinite(names[j][1])]
    return per_lag, best, max(speeds, default=None)


# steps from 0 of at most 150 mrad and a slow sine: the host latches steering_not_following when the error stays
# above 3 deg for 1.5 s (host_track_mismatch); the first wall clock run's +250 -> -250 mrad swing at standstill was
# still at +27 mrad after 1.5 s (a correct host detection, not a test of A25)
A25_STANDSTILL_STEPS_MRAD = (150, 0, -150, 0, 80, 0, -80, 0)
A25_DRIVING_STEPS_MRAD = (150, 0, -150, 0)
A25_SINE_MRAD = 100
A25_SINE_PERIOD_S = 8.


def a25_motion(truth, step_targets):
    """The largest |delta_eq| of the shared model in the run against half of the largest step target (mrad of the
    equivalent angle)."""
    values = [abs(row['delta_eq_rad']) for row in truth if row.get('delta_eq_rad') is not None]
    largest = max((abs(t) for _, targets in step_targets for t in targets), default=0) / 1000.
    top = max(values, default=None)
    required = .5 * largest
    return dict(max_delta_eq_deg=None if top is None else math.degrees(top), required_deg=math.degrees(required),
                passed=top is not None and top >= required)


def check_a25(ctx, args):
    """A25 (G, emulator path; amendment 2): steering joint angles of /sim/truth/joint_states against the shared
    model's delta_L, delta_R (/sim/truth/steering, the emulator's model) at the same SIM time, during steps and a
    sine of the steering target at standstill and while driving. Judged with the A25 numbers (p99 <= 0.2 deg, no
    joint speed spikes); the delay in physics steps that fits best and the /clock gaps of the run are reported.
    The run must have moved the steering (a run without motion would compare zeros): the shared model's largest
    |delta_eq| must reach half of the largest step target, and the drive loop must have had no refused request."""
    from .vehicle_profile import load_profile
    from . import gz_model
    profile = load_profile()
    ctx.obs.enable_joints()
    # the first /clock of this process before arming: the bridge disarms (stale_input) when the first drive input
    # comes later than its input deadline after the ARM (first stack run: 2.3 s of DDS discovery, no motion at all)
    t0 = ctx.obs.spin_sim_for(0.)
    before = emulator_counters(ctx.hooks)
    mark = ctx.mark('a25')
    ctx.ensure_armed()
    loop = DriveLoop(ctx.client, 0, 0).start()
    phases = []
    step_targets = ((0, A25_STANDSTILL_STEPS_MRAD), (30, A25_DRIVING_STEPS_MRAD))
    try:
        for throttle, targets in step_targets:
            for target in targets:
                loop.set(throttle, target)
                ctx.obs.spin_sim_for(args.step_hold_s)
            phases.append(dict(throttle=throttle, steps=list(targets)))
        for k in range(int(round(args.sine_s / .1))):
            loop.set(0, int(round(A25_SINE_MRAD * math.sin(2. * math.pi * k * .1 / A25_SINE_PERIOD_S))))
            ctx.obs.spin_sim_for(.1)
        phases.append(dict(throttle=0, sine_mrad=A25_SINE_MRAD, period_s=A25_SINE_PERIOD_S, seconds=args.sine_s))
    finally:
        loop.stop()
    ctx.client.stop('brake')
    ctx.obs.spin_sim_for(1.)
    after = emulator_counters(ctx.hooks)
    mono1 = time.monotonic()
    t1 = ctx.obs.sim_time
    joints = [row for row in ctx.obs.joints if t0 <= row[0] <= t1]
    truth = truth_rows(ctx.obs, t0 - .1, t1 + .1)
    keys = {'left': (gz_model.STEER_JOINTS['left'], 'delta_left_rad'),
            'right': (gz_model.STEER_JOINTS['right'], 'delta_right_rad')}
    rate_bound = A25_STEER_SPIKE_FACTOR * math.radians(max(float(profile.value('steer_rate_max_deg_s')),
                                                           float(profile.value('steer_rate_standstill_deg_s'))))
    per_lag, best, top = a25_summary(joints, truth, keys)
    same = per_lag[0]
    moved = a25_motion(truth, step_targets)
    loop_errors = dict(loop.errors)
    delta = counter_delta(before, after)
    emu, bridge = ctx.logs(mark)
    false = false_detections(emu, read_jsonl(BRIDGE_LOG), after_tick=mark['tick'],
                             after_wall=mark['wall'] - .01,
                             stalls=emulator_stalls(), slow=emulator_slow(), clock=args.clock)
    validity = step_validity(ctx, args.clock, mark['mono'], mono1, before, after,
                             sim_made=false['sim_stall_latches'] if args.clock == 'wall' else ())
    criteria = [criterion('joint_minus_model_same_sim_time_p99_deg', same['p99_deg'], '<= %.1f' % A25_P99_LIMIT_DEG,
                          same['p99_deg'] is not None and same['p99_deg'] <= A25_P99_LIMIT_DEG,
                          'A25 numbers on the emulator path (amendment 2: measured apart from the lockstep path)'),
                criterion('joint_speed_spikes_rad_s', top, '<= %.3f (1.5 x the largest model rate)' % rate_bound,
                          top is not None and top <= rate_bound),
                criterion('matched_samples', same['matched'], '> 0', same['matched'] > 0),
                criterion('steering_moved_deg', moved['max_delta_eq_deg'], '>= %.2f (half of the largest step '
                          'target)' % moved['required_deg'], moved['passed'],
                          'a run without steering motion compares zeros and must not pass'),
                criterion('drive_loop_refusals', loop_errors, {}, not loop_errors),
                criterion('best_delay_steps', best, 'report (steps of 1/120 s)', None),
                criterion('clock_samples_missing', None if delta is None else delta.get('clock_messages_missing'),
                          'report', None, 'skipped /clock samples during the run (emulator counters)')]
    criteria = invalidate(criteria, validity)          # wall clock mode: RTF < 0.9 or an emulator stall (A10)
    return dict(check='A25_G_emulator_path', criteria=criteria, passed=verdict(criteria), per_lag=per_lag,
                validity=validity, motion=moved, loop_errors=loop_errors,
                best_lag_steps=best, joint_samples=len(joints), truth_samples=len(truth), phases=phases,
                speed_bound_rad_s=rate_bound, top_joint_speed_rad_s=top, emulator_clock_run=delta,
                lead_steps=(ctx.steering_status().get('emulator') or {}).get('lead_steps'),
                false_detection=false, rule='joint angle at stamp T vs the shared model angle with sim_time_s T (the '
                                            'emulator publishes the model at clock + lead steps; lag k compares with '
                                            'the model k steps earlier)')


def summarize_drives(paths):
    runs = [load_json(p) for p in paths]
    keys = ('final_x_m', 'final_y_m', 'final_yaw_deg', 'straight_lateral_end_m', 'straight_lateral_max_m', 'path_m')
    out = {}
    for key in keys:
        values = [r[key] for r in runs if r.get(key) is not None]
        out[key] = dict(values=values, mean=statistics.mean(values) if values else None,
                        sd=statistics.stdev(values) if len(values) > 1 else None,
                        range=(max(values) - min(values)) if values else None)
    rtf = [r['rtf_median'] for r in runs if r.get('rtf_median') is not None]
    return dict(check='A9_summary', runs=len(runs), scene=[r.get('scene') for r in runs], stats=out,
                rtf_medians=rtf, passed=None, note='report (A9: final position and lateral error variance)')


# --- main ---------------------------------------------------------------------------------------------------------------------------
CHECKS = dict(startup=check_startup, graph=check_graph, identity=check_identity, safety=check_safety,
              faults=check_faults, a3=check_a3, drive=check_drive, rtf=check_rtf, a25=check_a25)


def build_parser():
    p = argparse.ArgumentParser(prog='p6a_checks', description=__doc__.splitlines()[0])
    p.add_argument('check', choices=sorted(CHECKS) + ['summary9', 'a19g_normal'])
    p.add_argument('--out', required=True)
    p.add_argument('--label', default=None)
    p.add_argument('--clock', default=os.environ.get('UBTS_STEER_CLOCK', 'wall'))
    p.add_argument('--repeat', type=int, default=5)
    p.add_argument('--seconds', type=float, default=None,
                   help='rtf: measured seconds (default 240, amendment 2); a3: recorded SIM seconds (default 20)')
    p.add_argument('--warmup-s', type=float, default=20., help='rtf: the first seconds left out (amendment 2)')
    p.add_argument('--step-hold-s', type=float, default=2., help='a25: hold time of each steering step')
    p.add_argument('--sine-s', type=float, default=8., help='a25: seconds of the steering sine')
    p.add_argument('--gsim-results', nargs='*', default=[],
                   help='a19g_normal: gz_probe a25_firmware result.json files (G-SIM lockstep, normal scripts)')
    p.add_argument('--dashboard-url', default='http://dashboard:8080')
    p.add_argument('--throttle', type=int, default=40)
    p.add_argument('--steer-mrad', type=int, default=175)
    p.add_argument('--straight-s', type=float, default=15.)
    p.add_argument('--turn-s', type=float, default=10.)
    p.add_argument('--hold-s', type=float, default=1.)
    p.add_argument('--inputs', nargs='*', default=[])
    p.add_argument('--cases', nargs='*', default=[], help='faults: run only these cases (development)')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    out = Path(args.out)
    label = args.label or args.check
    meta = dict(schema=SCHEMA, check=args.check, label=label, started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',
                                                                                       time.gmtime()),
                source_commit=os.environ.get('UBTS_STEER_SOURCE_COMMIT'),
                source_status_sha256=os.environ.get('UBTS_STEER_SOURCE_STATUS_SHA256'),
                image=os.environ.get('UBTS_STEER_IMAGE'), scene=os.environ.get('UBTS_STEER_SCENE'), clock=args.clock,
                result_kind=RESULT_KIND, field_validated=False)
    if args.seconds is None:
        args.seconds = 240. if args.check == 'rtf' else 20.
    if args.check == 'summary9':
        result = summarize_drives(args.inputs)
        write_json(out / ('%s.json' % label), dict(meta, **result))
        print(json.dumps(dict(check='summary9', runs=result['runs'])))
        return 0
    if args.check == 'a19g_normal':
        result = summarize_normal(args.inputs, args.gsim_results)
        write_json(out / ('%s.json' % label), dict(meta, **result))
        print(json.dumps(dict(check='a19g_normal', total=result['total'], passed=result['passed'])))
        return 0
    meta['provenance'] = provenance()
    ctx = Ctx(out)
    ctx.clock = args.clock
    try:
        try:
            meta['steering_status'] = {k: v for k, v in ctx.steering_status()['emulator'].items()
                                       if k in ('session_info', 'step_ns', 'lead_steps', 'port_open_reset')}
        except (OSError, ValueError, KeyError):
            pass
        try:
            meta['bridge_loop'] = load_json(BRIDGE_STATUS).get('bridge_loop')
        except (OSError, ValueError):
            pass
        started = time.monotonic()
        counters0 = emulator_counters(ctx.hooks)
        result = CHECKS[args.check](ctx, args)
        result['wall_s'] = time.monotonic() - started
        result['emulator_clock'] = counter_delta(counters0, emulator_counters(ctx.hooks))
        result['rtf'] = rtf_summary(ctx.obs.clock_rows)
    except Exception as exc:
        result = dict(check=args.check, passed=False, error='%s: %s' % (type(exc).__name__, exc))
        import traceback
        result['traceback'] = traceback.format_exc()
    finally:
        ctx.close()
    payload = dict(meta, **result)
    write_json(out / ('%s.json' % label), payload)
    print(json.dumps(dict(check=args.check, label=label, passed=result.get('passed'),
                          error=result.get('error')), default=str))
    return 0 if result.get('passed') is not False else 1


if __name__ == '__main__':
    sys.exit(main())
