"""Executor container of the steering G-SIM (design 2.2-2.6, 4.3 and 8.3 P6a '실행기 컨테이너'; WF-4).

The real single-row executor (tools/row_executor: executor_main.ExecutorLoop, core.Runner, bridge_client.SteerBridgeClient,
operator_http) in the compose service 'executor' of compose.ubts-steer.yaml: the same base image (rclpy, Python 3.12), the
branch sources read only, network and IPC of the sim service (FastDDS shared memory), the control volume
ubts-steer_control at /run/ubts-steer (control socket /run/ubts-steer/control.sock), ROS domain 43, 0.5 GiB.
It talks to the bridge wrapper only through the control socket (SteerBridgeClient over UnixSocketTransport) and reads
only /scan, /imu/data_raw, /rover/control/requested (and /clock in the SIM clock mode: SIM_ACCEPTANCE amendment 11).
It mounts neither the runtime volume nor the emulator hooks (no SIM status or truth file is reachable).

What differs from executor_main.main (the vehicle entry, unchanged):
  configuration   built in code from the vehicle profile and the host profile export (gsim_executor_config: the
                  values of the T1 configuration tools/row_executor/sim_in_loop_steer.steer_executor_config, plus the
                  SIM socket /run/ubts-steer/control.sock and the operator page on 127.0.0.1); --config-overrides
                  JSON on top; ExecutorConfig.validate() must pass; written to <exec dir>/config.json
  controller      the Runner's controller slot holds a SIM scripted controller (steering.scripted_control; the
                  row-following controller is P8); --controller JSON, or the hook 'controller' between runs
  clock           --clock wall: executor_main.RosInput (receipt time.monotonic) and executor_main.run_loop (20 Hz on
                  the monotonic clock): the vehicle code path. --clock sim: the node also subscribes /clock; receipts,
                  the loop clock, the stall watchdog, the hold timeout, the SteerBridgeClient clock and the steering
                  status age are SIM time; one step per 50 ms of SIM time (a jump over several grid points runs one
                  step, the missed ones are counted, never caught up), polled every 2 ms wall like the bridge
                  wrapper's --clock sim; the bridge wrapper must run with --clock sim too (UBTS_STEER_CLOCK). The
                  vehicle deadlines (bridge input 0.75 s, release 1.1 s) are the bridge's, in its clock
  operator page   operator_http on 127.0.0.1:<--bind-port, 8097> inside the sim network namespace; the token goes to
                  <exec dir>/log/operator_token.txt (0600). The checks of steering.exec_checks drive the page like the
                  operator: Start, Arm, hold heartbeats every 100 ms, STOP, ack
  SIM hooks       <exec dir>/hooks.sock (SIM only; the vehicle has none): one JSON line per connection
                    status                          the executor state (clock, runner, client, hooks, counts)
                    controller {spec}               the scripted controller of the next resume (refused while ACTIVE)
                    stall {duration_s}              the main loop does not step for duration_s (the stall watchdog
                                                    thread keeps running: watchdog drive(0, held steering), pause)
                    freeze {duration_s}             the whole process stops (SIGSTOP; a helper process sends SIGCONT):
                                                    no thread runs, the bridge's input deadline ends the motion
                    kill                            SIGKILL of the executor process (the controlling client dies;
                                                    design 7.2.1 'dashboard / bridge forced kill' with the executor)
                    wall_offset {seconds}           the executor's wall clock (log 'wall', snapshot age check) jumps by
                                                    seconds: the 09-19 'system time change' on the executor side (the
                                                    executor's deadlines are on the monotonic or SIM clock)
  files           <exec dir> (default /run/ubts-steer/exec, inside the control volume): config.json, status.json
                  (1 s), log/ (the executor JSONL log, operator_token.txt), hooks.sock; an earlier status.json is
                  moved to retired/<run token>/ at the start (audit G06)
  diagnostics     SIM clock mode (G-SIM stage 5): SpinMonitor watches the input thread (subscription callbacks, its own
                  20 Hz timer), a GIL probe thread (10 ms sleeps) and the main loop's poll gaps; every executor tick
                  carries 'diag' (callbacks per topic, the longest gaps, timer callbacks, probe lateness, poll gap, the
                  monotonic time) and a subscription gap above 150 ms is an 'input_gap' event. executor_main adds the
                  phase times of a slow step ('ph') and the time of each request ('ms'). Nothing the loop decides changes
  sources         executor_main.verify_sources() (SHA256SUMS of tools/row_executor) and verify_vendor() as on the
                  vehicle; --skip-source-check (development snapshots only) is recorded in config.json and the status

CLI (compose service 'executor'; tools/row_executor and sim/ubts on PYTHONPATH):
  python3 -B -m steering.exec_container_main --clock wall|sim [--controller JSON] [--config-overrides JSON]
      [--profile YAML] [--cal-id N] [--exec-dir DIR] [--socket PATH] [--bind-port N] [--max-log-mb MB]
      [--skip-source-check] [--max-runtime-s S]
Placeholder-based SIM results (자리값 기반 SIM 결과), not field validation.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
from pathlib import Path
import secrets
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid

SCHEMA = 'ubts.steer_exec_container.v1'
EXEC_DIR = '/run/ubts-steer/exec'
SIM_SOCKET = '/run/ubts-steer/control.sock'
BIND_HOST = '127.0.0.1'
BIND_PORT = 8097
CLOCK_MODES = ('wall', 'sim')
SIM_POLL_S = .002                     # SIM clock mode: wall poll of the /clock grid (steer_rover_bridge SIM_POLL_PERIOD_S)
STATUS_PERIOD_S = 1.
LOG_FILES_KEPT = 30
MAX_HOOK_STALL_S = 30.
MAX_WALL_OFFSET_S = 7. * 86400.
DEFAULT_CONTROLLER = {'kind': 'straight_hold', 'drive_pct': 40, 'distance_m': 8.0}
BASE_DUTY_PCT = 55                    # as sim_in_loop_steer.steer_executor_config
SCAN_TOPIC, IMU_TOPIC, HUMAN_TOPIC, CLOCK_TOPIC = '/scan', '/imu/data_raw', '/rover/control/requested', '/clock'


class ContainerError(RuntimeError):
    pass


# --- configuration ------------------------------------------------------------------------------------------------------
def host_profile_of(profile, cal_id):
    """The host profile (scripts.steer_bridge_core.HostProfile) of a vehicle profile, as the steer role exports it."""
    from scripts.steer_bridge_core import HostProfile
    from .host_profile_export import build
    return HostProfile(build(profile, cal_id=cal_id))


def gsim_executor_config(profile, host, cal_id, overrides=None, base_duty_pct=BASE_DUTY_PCT, socket_path=SIM_SOCKET,
                         bind_host=BIND_HOST, bind_port=BIND_PORT):
    """The G-SIM executor configuration: the values of the T1 configuration (sim_in_loop_steer.steer_executor_config:
    vehicle_profile front_steer, single-row mode, MOTOR, SIM 'measured' values from the vehicle profile and the host
    profile: SIM_ASSUMED placeholders) plus the SIM control socket and the operator page address. overrides:
    ExecutorConfig fields on top. Returns the validated config."""
    from dataclasses import replace
    from row_executor import config as XC
    from . import drive as SD
    value = profile.value
    speed = float(SD.steady_speed_m_s(profile, base_duty_pct))
    values = dict(
        vehicle_profile=XC.FRONT_STEER, mode=XC.SINGLE_ROW_MODE, require_native_navigation=False,
        motor_output_enabled=True, base_duty_pct=int(base_duty_pct),
        max_yaw_rate_deg_s=3.6,
        breakaway_duty_pct=20, v_measured_m_s=round(min(speed, .5), 3), max_speed_m_s=.55,
        measured_stop_distance_m=.2,
        steer_limit_left_mrad=int(host.limit_left_mrad), steer_limit_right_mrad=int(host.limit_right_mrad),
        steer_rate_limit_deg_s=round(XC.STEER_RATE_FRACTION * float(value('steer_rate_max_deg_s')), 3),
        steer_rate_limit_standstill_deg_s=round(XC.STEER_RATE_FRACTION * float(value('steer_rate_standstill_deg_s')),
                                                3),
        steer_measured_rate_deg_s=float(value('steer_rate_max_deg_s')),
        steer_measured_rate_standstill_deg_s=float(value('steer_rate_standstill_deg_s')),
        steer_range_measured=True, steer_wheel_soft_margin_deg=float(value('steer_soft_margin_deg')),
        steer_min_pwm_pct=float(value('steer_min_pwm_pct')), steer_zero_offset_deg=0.,
        steer_servo_deadband_deg=float(value('servo_deadband_deg')), steer_polarity_checked=True,
        steer_cal_id=int(cal_id), steer_allow_sim=True,
        steer_rotation_centre_x_m=float(value('rear_axle_x_in_base_m')),
        lidar_yaw_offset_deg=float(value('lidar_yaw_offset_deg')), lidar_x_in_base_m=float(value('lidar_x_in_base_m')),
        footprint_length_m=float(value('footprint_length_m')), footprint_width_m=float(value('footprint_width_m')),
        socket_path=str(socket_path), bind_host=str(bind_host), bind_port=int(bind_port))
    values.update(overrides or {})
    return replace(XC.ExecutorConfig(), **values).validate()


def parse_json_arg(text, what):
    """A JSON object from a command-line value (JSON text, or a JSON file path); None for an empty value."""
    if text is None or not str(text).strip():
        return None
    text = str(text).strip()
    try:
        data = json.loads(text) if text.startswith('{') else json.loads(Path(text).read_bytes().decode('utf-8'))
    except (OSError, ValueError) as exc:
        raise ContainerError('%s: not a JSON object (%s)' % (what, exc)) from None
    if not isinstance(data, dict):
        raise ContainerError('%s: not a JSON object' % what)
    return data


# --- clocks -------------------------------------------------------------------------------------------------------------
class SimClock:
    """SIM time from /clock samples: never decreasing; 0.0 before the first sample (started() False)."""

    def __init__(self):
        self.t = 0.
        self.samples = 0
        self.backwards = 0

    def update(self, sec, nanosec):
        value = int(sec) + int(nanosec) * 1e-9
        self.samples += 1
        if value >= self.t:
            self.t = value
        else:
            self.backwards += 1           # a rewind of the SIM is not followed (the executor clock never goes back)

    def started(self):
        return self.samples > 0

    def __call__(self):
        return self.t


class WallClock:
    """time.time() + an offset the SIM hook 'wall_offset' sets (the executor's wall clock only)."""

    def __init__(self, base=time.time):
        self.base = base
        self.offset_s = 0.

    def __call__(self):
        return self.base() + self.offset_s


# --- controller slot ----------------------------------------------------------------------------------------------------
class ControllerSlot:
    """The Runner's controller_factory: a scripted controller of the current spec, built at every resume."""

    def __init__(self, spec, cfg):
        self.cfg = cfg
        self.spec = None
        self.factory = None
        self.built = 0
        self.set(spec)

    def set(self, spec):
        from . import scripted_control
        spec = dict(spec)
        factory = scripted_control.make_factory(spec, self.cfg)        # raises on a bad spec
        self.spec, self.factory = spec, factory
        self.digest = scripted_control.spec_digest(spec)
        return self.digest

    def __call__(self):
        self.built += 1
        return self.factory()

    def factory_for(self, cfg):
        """The Runner's controller factory for the executor loop's effective config (the field tuning values the loop
        loaded, PW-08): the current spec at every resume, built with that config."""
        from . import scripted_control

        def build():
            self.built += 1
            return scripted_control.make_factory(self.spec, cfg)()
        return build


# --- input thread diagnostics -------------------------------------------------------------------------------------------
class SpinMonitor:
    """Diagnostics of the executor's ROS input thread (G-SIM stage 5, 2026-11-14): in stage 4 two runs ended 0.4-0.6 s
    after the go because no /clock, scan or IMU callback ran for about 0.35 s while the operator HTTP thread answered on
    time. Thread-safe, constant memory; the main loop reads it once per tick (take(): the tick's 'diag').

      data(i)   a subscription callback of topic TOPICS[i] began (the input thread, wall = time.monotonic())
      timer()   the input thread's own 20 Hz timer ran (no DDS data involved: the thread itself is alive)
      probe(s)  the GIL probe thread woke s seconds late from a 10 ms sleep (the whole process: GIL, scheduler)
      poll(s)   the main loop's SIM clock poll came s seconds after the previous one (2 ms nominal)
    A gap of the subscription callbacks longer than gap_event_s is kept as one 'input_gap' event with what the other
    three saw meanwhile: timer callbacks (the thread ran: the data did not come), the probe's worst lateness and the
    main loop's longest poll gap (the process or the main thread did not run)."""

    TOPICS = ('clock', 'scan', 'imu', 'human')
    GAP_EVENT_S = .15
    MAX_EVENTS = 50

    def __init__(self, mono=time.monotonic, gap_event_s=GAP_EVENT_S):
        self.mono = mono
        self.gap_event_s = float(gap_event_s)
        self.lock = threading.Lock()
        self.events = collections.deque(maxlen=self.MAX_EVENTS)
        self.totals = dict(events=0, data=0, timer=0)
        self.last_data = self.last_clock = self.last_timer = None
        self._reset_tick()
        self._reset_gap()

    def _reset_tick(self):
        self.counts = [0] * len(self.TOPICS)
        self.timers = 0
        self.max_data_gap = self.max_clock_gap = self.max_timer_gap = self.max_probe = self.max_poll = 0.

    def _reset_gap(self):
        self.gap_timers = 0
        self.gap_probe = self.gap_poll = 0.

    def data(self, index):
        now = self.mono()
        with self.lock:
            self.counts[index] += 1
            self.totals['data'] += 1
            if self.last_data is not None:
                gap = now - self.last_data
                self.max_data_gap = max(self.max_data_gap, gap)
                if gap > self.gap_event_s:
                    self.totals['events'] += 1
                    self.events.append(dict(gap_ms=round(gap * 1e3, 1), first=self.TOPICS[index],
                                            timer_callbacks=self.gap_timers, probe_max_ms=round(self.gap_probe * 1e3, 1),
                                            poll_max_ms=round(self.gap_poll * 1e3, 1), wm=round(now, 3)))
            if index == 0:
                if self.last_clock is not None:
                    self.max_clock_gap = max(self.max_clock_gap, now - self.last_clock)
                self.last_clock = now
            self.last_data = now
            self._reset_gap()

    def timer(self):
        now = self.mono()
        with self.lock:
            self.timers += 1
            self.gap_timers += 1
            self.totals['timer'] += 1
            if self.last_timer is not None:
                self.max_timer_gap = max(self.max_timer_gap, now - self.last_timer)
            self.last_timer = now

    def probe(self, late_s):
        with self.lock:
            self.max_probe = max(self.max_probe, late_s)
            self.gap_probe = max(self.gap_probe, late_s)

    def poll(self, gap_s):
        with self.lock:
            self.max_poll = max(self.max_poll, gap_s)
            self.gap_poll = max(self.gap_poll, gap_s)

    def take(self):
        """(diag dict for the tick, list of input_gap events since the last take); resets the per-tick values.
        diag (ms, per tick): wm time.monotonic() [s], cb callbacks per topic (TOPICS), gap / cgap the longest
        subscription / clock callback gap, tm timer callbacks and tgap their longest gap, gil the probe's worst
        lateness, poll the main loop's longest poll gap."""
        with self.lock:
            diag = dict(wm=round(self.mono(), 3), cb=list(self.counts), gap=round(self.max_data_gap * 1e3, 1),
                        cgap=round(self.max_clock_gap * 1e3, 1), tm=self.timers, tgap=round(self.max_timer_gap * 1e3, 1),
                        gil=round(self.max_probe * 1e3, 1), poll=round(self.max_poll * 1e3, 1))
            events = list(self.events)
            self.events.clear()
            self._reset_tick()
        return diag, events

    def summary(self):
        with self.lock:
            return dict(self.totals, gap_event_ms=round(self.gap_event_s * 1e3, 1))


def gil_probe(monitor, stop_event, period_s=.01):
    """The GIL probe thread: sleeps period_s and reports how late it woke (the GIL or the scheduler held it back)."""
    while not stop_event.is_set():
        t0 = time.monotonic()
        time.sleep(period_s)
        monitor.probe(max(0., time.monotonic() - t0 - period_s))


def tick_diag(loop, monitor):
    """The ExecutorLoop's tick_extra: the monitor's per-tick values; its input_gap events go to the log as events."""
    def extra():
        diag, events = monitor.take()
        for event in events:
            loop._event('input_gap', **event)
        return diag
    return extra


# --- ROS input (SIM clock mode) -----------------------------------------------------------------------------------------
class SimRosInput:
    """The executor node ubts_row_executor in the SIM clock mode: the subscriptions of executor_main.RosInput (same
    topics and QoS) plus /clock; the receipt of a scan, IMU or /rover/control/requested message is the SIM time of the
    last /clock sample at callback entry. Callbacks only append to the loop's input buffers."""

    def __init__(self, loop, sim_clock, scan_topic=SCAN_TOPIC, imu_topic=IMU_TOPIC, human_topic=HUMAN_TOPIC,
                 clock_topic=CLOCK_TOPIC, monitor=None):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, LaserScan
        from std_msgs.msg import String
        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node('ubts_row_executor')

        def sensor_qos(depth):
            return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                              reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        clock = sim_clock
        mark = (lambda index: None) if monitor is None else monitor.data

        def on_clock(m):
            mark(0)
            clock.update(m.clock.sec, m.clock.nanosec)

        def on_scan(m):
            mark(1)
            loop.on_scan_msg(clock(), m)

        def on_imu(m):
            mark(2)
            loop.on_imu_msg(clock(), m)

        def on_human(m):
            mark(3)
            loop.on_human_msg(clock(), m.data)
        self.node.create_subscription(Clock, clock_topic, on_clock, sensor_qos(10))
        self.node.create_subscription(LaserScan, scan_topic, on_scan, sensor_qos(2))
        self.node.create_subscription(Imu, imu_topic, on_imu, sensor_qos(50))
        if human_topic:
            self.node.create_subscription(String, human_topic, on_human, 5)
        self.timer = None
        if monitor is not None:
            # the input thread's own 20 Hz timer (system clock, no DDS): it runs while the thread runs (SpinMonitor)
            self.timer = self.node.create_timer(.05, monitor.timer)
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


# --- loop ---------------------------------------------------------------------------------------------------------------
class SteppedLoop:
    """The ExecutorLoop as the run loops see it, with the SIM hook 'stall': step() first waits out an active stall
    (the main loop thread hangs; nothing else changes). Every other attribute is the loop's."""

    def __init__(self, loop, hooks_state):
        self._loop = loop
        self._hooks = hooks_state

    def step(self, now=None):
        until = self._hooks.stall_until
        if until is not None:
            delay = until - time.monotonic()
            if delay > 0.:
                time.sleep(delay)
            self._hooks.stall_until = None
            self._hooks.stalls_done += 1
        return self._loop.step(now)

    def __getattr__(self, name):
        return getattr(self._loop, name)


def run_loop_sim(loop, stop_event, sim_clock, hz=20., poll_s=SIM_POLL_S, max_exceptions=5, exception_window_s=60.,
                 grid=None, deadline=None, monitor=None):
    """SIM clock mode loop (module docstring): one loop.step() each time the SIM clock reaches the next 1/hz grid point;
    a jump over several points runs one step and counts the missed ones (loop.counts['missed_ticks'], grid['skipped']).
    An exception in a step: as executor_main.run_loop in MOTOR mode (logged, emergency STOP, the process ends after more
    than max_exceptions within exception_window_s of the monotonic clock). monitor (SpinMonitor, diagnostics): the wall
    gap between two polls (a step in between counts too)."""
    import traceback
    period = 1. / float(hz)
    grid = {} if grid is None else grid
    grid.update(steps=0, skipped=0, polls=0, next_t=None, started=False)
    recent = collections.deque()
    last_poll = time.monotonic()
    while not stop_event.wait(poll_s):
        mono = time.monotonic()
        if monitor is not None:
            monitor.poll(mono - last_poll)
        last_poll = mono
        if deadline is not None and mono >= deadline:
            break
        grid['polls'] += 1
        if not sim_clock.started():
            continue
        now = sim_clock()
        if grid['next_t'] is None:
            grid['next_t'] = math.floor(now / period + 1e-9) * period
            grid['started'] = True
        if now + 1e-9 < grid['next_t']:
            continue
        missed = int(math.floor((now - grid['next_t']) / period + 1e-9))
        if missed > 0:
            grid['skipped'] += missed
            loop.counts['missed_ticks'] = loop.counts.get('missed_ticks', 0) + missed
        grid['next_t'] = grid['next_t'] + (missed + 1) * period
        grid['steps'] += 1
        try:
            loop.step()
        except Exception as exc:
            mono = time.monotonic()
            note = getattr(loop, 'note_loop_exception', None)
            if note is not None:
                note(repr(exc))
            recent.append(mono)
            while recent and mono - recent[0] > exception_window_s:
                recent.popleft()
            loop._event('loop_exception', error=repr(exc), trace=traceback.format_exc()[-2000:])
            loop.emergency_stop('loop_exception')
            if len(recent) > max_exceptions:
                raise
    return grid


# --- SIM hooks ----------------------------------------------------------------------------------------------------------
class HooksState:
    def __init__(self):
        self.stall_until = None
        self.stalls_requested = 0
        self.stalls_done = 0
        self.freezes = 0
        self.events = collections.deque(maxlen=200)


class ExecHooks:
    """SIM-only fault hooks of the executor container (module docstring 'SIM hooks')."""

    def __init__(self, path, state, *, loop=None, slot=None, wall=None, status_fn=None, pid=None, kill=os.kill):
        self.path = str(path)
        self.state = state
        self.loop = loop
        self.slot = slot
        self.wall = wall
        self.status_fn = status_fn
        self.pid = os.getpid() if pid is None else pid
        self.kill = kill
        self.server = None
        self.pending = []              # actions done right after the reply (freeze, kill)

    def _note(self, op, **fields):
        self.state.events.append(dict(op=op, mono=round(time.monotonic(), 4), wall=round(time.time(), 4), **fields))

    @staticmethod
    def _seconds(value, low, high, name):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or \
                not low < float(value) <= high:
            raise ValueError('%s must be a number in (%g, %g]' % (name, low, high))
        return float(value)

    def handle(self, request):
        """One hook request -> reply (dict). A bad request gets ok false."""
        op = request.get('op') if isinstance(request, dict) else None
        try:
            if op == 'status':
                return dict(ok=True, status=None if self.status_fn is None else self.status_fn())
            if op == 'controller':
                if self.slot is None:
                    return dict(ok=False, error='no_controller_slot')
                from row_executor.core import ACTIVE
                if self.loop is not None and self.loop.runner.state == ACTIVE:
                    return dict(ok=False, error='run_active')
                digest = self.slot.set(request.get('spec') or {})
                self._note('controller', digest=digest)
                return dict(ok=True, digest=digest, spec=self.slot.spec)
            if op == 'stall':
                seconds = self._seconds(request.get('duration_s'), 0., MAX_HOOK_STALL_S, 'duration_s')
                self.state.stall_until = time.monotonic() + seconds
                self.state.stalls_requested += 1
                self._note('stall', duration_s=seconds)
                return dict(ok=True, duration_s=seconds, mono=time.monotonic(), wall=time.time())
            if op == 'freeze':
                seconds = self._seconds(request.get('duration_s'), 0., MAX_HOOK_STALL_S, 'duration_s')
                self.state.freezes += 1
                self._note('freeze', duration_s=seconds)
                self.pending.append(('freeze', seconds))
                return dict(ok=True, duration_s=seconds, mono=time.monotonic(), wall=time.time())
            if op == 'kill':
                self._note('kill')
                self.pending.append(('kill', None))
                return dict(ok=True, mono=time.monotonic(), wall=time.time())
            if op == 'wall_offset':
                if self.wall is None:
                    return dict(ok=False, error='no_wall_clock')
                seconds = request.get('seconds')
                if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds) \
                        or abs(seconds) > MAX_WALL_OFFSET_S:
                    raise ValueError('seconds must be a finite number with |seconds| <= %g' % MAX_WALL_OFFSET_S)
                self.wall.offset_s = float(seconds)
                self._note('wall_offset', seconds=float(seconds))
                return dict(ok=True, offset_s=self.wall.offset_s, wall=self.wall())
        except (ValueError, TypeError, KeyError) as exc:
            return dict(ok=False, error=str(exc))
        return dict(ok=False, error='unknown_op')

    def after_reply(self):
        """Run the pending process actions once the reply is out (freeze: a helper process continues this one after
        the time; kill: SIGKILL now)."""
        while self.pending:
            action, seconds = self.pending.pop(0)
            if action == 'freeze':
                code = ('import os, signal, sys, time; time.sleep(float(sys.argv[2])); '
                        'os.kill(int(sys.argv[1]), signal.SIGCONT)')
                helper = subprocess.Popen([sys.executable, '-I', '-c', code, str(self.pid), repr(seconds)],
                                          stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                          stderr=subprocess.DEVNULL)
                threading.Thread(target=helper.wait, name='exec-freeze-helper', daemon=True).start()
                self.kill(self.pid, signal.SIGSTOP)
            elif action == 'kill':
                self.kill(self.pid, signal.SIGKILL)

    def start(self):
        import socketserver
        hooks = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.connection.settimeout(2.)
                try:
                    line = self.rfile.readline(65537)
                    if len(line) > 65536:
                        raise ValueError('request_too_large')
                    request = json.loads(line)
                    if not isinstance(request, dict):
                        raise ValueError('object_required')
                    reply = hooks.handle(request)
                except (ValueError, OSError) as exc:
                    reply = dict(ok=False, error=str(exc))
                try:
                    self.wfile.write((json.dumps(reply, default=str) + '\n').encode('utf-8'))
                    self.wfile.flush()
                except OSError:
                    pass
                hooks.after_reply()

        class Server(socketserver.ThreadingUnixStreamServer):
            daemon_threads = True

        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        if os.path.exists(self.path):
            os.unlink(self.path)
        self.server = Server(self.path, Handler)
        os.chmod(self.path, 0o660)
        threading.Thread(target=self.server.serve_forever, name='exec-sim-hooks', daemon=True).start()
        return self

    def close(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None
        try:
            os.unlink(self.path)
        except OSError:
            pass


# --- files --------------------------------------------------------------------------------------------------------------
def write_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_bytes((json.dumps(payload, indent=1, allow_nan=False, default=str) + '\n').encode('utf-8'))
    os.replace(tmp, path)


def retire_status(exec_dir, token):
    """Move an earlier run's status.json to <exec dir>/retired/<token>/ (audit G06). Returns True when moved."""
    source = Path(exec_dir) / 'status.json'
    if not source.is_file():
        return False
    target = Path(exec_dir) / 'retired' / str(token)
    target.mkdir(parents=True, exist_ok=True)
    os.replace(source, target / 'status.json')
    folders = sorted((p for p in (Path(exec_dir) / 'retired').iterdir() if p.is_dir()), key=lambda p: p.stat().st_mtime)
    for path in folders[:max(0, len(folders) - 5)]:
        shutil.rmtree(path, ignore_errors=True)
    return True


def prune_logs(log_dir, keep=LOG_FILES_KEPT):
    """Keep the newest `keep` executor JSONL files of the log folder (the control volume is kept between runs)."""
    folder = Path(log_dir)
    if not folder.is_dir():
        return 0
    files = sorted(folder.glob('row_executor_*.jsonl'), key=lambda p: p.stat().st_mtime)
    removed = 0
    for path in files[:max(0, len(files) - int(keep))]:
        try:
            path.unlink()
            removed += 1
        except OSError:
            pass
    return removed


# --- main ---------------------------------------------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(prog='exec_container_main', description=__doc__.splitlines()[0])
    p.add_argument('--clock', choices=CLOCK_MODES, default=os.environ.get('UBTS_STEER_CLOCK', 'wall') or 'wall')
    p.add_argument('--controller', default=os.environ.get('UBTS_STEER_EXEC_CONTROLLER') or None,
                   help='scripted controller spec (JSON text or file; default %s)' % json.dumps(DEFAULT_CONTROLLER))
    p.add_argument('--config-overrides', default=os.environ.get('UBTS_STEER_EXEC_OVERRIDES') or None,
                   help='ExecutorConfig fields on top of the G-SIM configuration (JSON text or file)')
    p.add_argument('--profile', default=os.environ.get('UBTS_STEER_PROFILE') or None)
    p.add_argument('--cal-id', type=int, default=int(os.environ.get('UBTS_STEER_CAL_ID', '1') or 1),
                   help='calibration set id the executor expects (the steer role exports and the MCU reports 1)')
    p.add_argument('--exec-dir', default=os.environ.get('UBTS_STEER_EXEC_DIR', EXEC_DIR))
    p.add_argument('--socket', default=os.environ.get('ROVER_CONTROL_SOCKET', SIM_SOCKET))
    p.add_argument('--bind-port', type=int, default=int(os.environ.get('UBTS_STEER_EXEC_PORT', str(BIND_PORT))))
    p.add_argument('--max-log-mb', type=float, default=200.)
    p.add_argument('--skip-source-check', action='store_true',
                   default=os.environ.get('UBTS_STEER_EXEC_SKIP_SOURCE_CHECK') == '1',
                   help='development snapshots only: run although tools/row_executor does not match SHA256SUMS')
    p.add_argument('--max-runtime-s', type=float, default=None, help='stop after this many seconds (tests)')
    return p


class Container:
    """The parts of one executor container run (built by main(); the tests build them without ROS)."""

    def __init__(self, args):
        from row_executor import executor_main as XM
        from row_executor.bridge_client import SteerBridgeClient, UnixSocketTransport
        from row_executor.core import Runner
        from .vehicle_profile import load_profile
        self.XM = XM
        self.args = args
        self.token = uuid.uuid4().hex
        self.exec_dir = Path(args.exec_dir)
        self.log_dir = self.exec_dir / 'log'
        self.started_wall = time.time()
        self.source_check = 'skipped (--skip-source-check: development snapshot)' if args.skip_source_check else 'ok'
        self.source_digest = None
        if not args.skip_source_check:
            try:
                self.source_digest = XM.verify_sources()
            except RuntimeError as exc:
                raise ContainerError('startup refused: %s (development snapshots: --skip-source-check)' % exc) from None
        XM.verify_vendor()
        self.profile = load_profile(args.profile)
        self.host = host_profile_of(self.profile, int(args.cal_id))
        overrides = parse_json_arg(args.config_overrides, '--config-overrides') or {}
        self.cfg = gsim_executor_config(self.profile, self.host, int(args.cal_id), overrides,
                                        socket_path=args.socket, bind_port=args.bind_port)
        spec = parse_json_arg(args.controller, '--controller') or dict(DEFAULT_CONTROLLER)
        self.slot = ControllerSlot(spec, self.cfg)
        self.wall = WallClock()
        self.sim_clock = SimClock() if args.clock == 'sim' else None
        self.clock = self.sim_clock if self.sim_clock is not None else time.monotonic
        self.hooks_state = HooksState()
        self.grid = {}
        self.owner = 'row-executor:gsim-%s' % self.token[:8]
        cfg, clock, wall = self.cfg, self.clock, self.wall
        self.exec_dir.mkdir(parents=True, exist_ok=True)
        self.retired = retire_status(self.exec_dir, self.token)
        self.log_pruned = prune_logs(self.log_dir)
        self.log = XM.JsonlLog(str(self.log_dir), max_total_mb=args.max_log_mb, wall=wall)
        self.loop = XM.ExecutorLoop(
            cfg, motor_output_flag=True, log=self.log, clock=clock, wall=wall, source_digest=self.source_digest,
            bridge_factory=lambda: SteerBridgeClient.from_config(cfg, owner=self.owner, clock=clock, wall=wall),
            stop_transport=UnixSocketTransport(cfg.socket_path, cfg.bridge_timeout_s),
            runner_factory=lambda c: Runner(c, controller_factory=self.slot.factory_for(c)))
        self.cfg = self.loop.config               # with the field tuning the loop loaded (config.json, status digest)
        # diagnostics of the input thread (SIM clock mode; G-SIM stage 5): the tick's 'diag' and 'input_gap' events
        self.monitor = SpinMonitor() if self.sim_clock is not None else None
        if self.monitor is not None:
            self.loop.tick_extra = tick_diag(self.loop, self.monitor)
        self.stepped = SteppedLoop(self.loop, self.hooks_state)
        self.hooks = ExecHooks(self.exec_dir / 'hooks.sock', self.hooks_state, loop=self.loop, slot=self.slot,
                               wall=self.wall, status_fn=self.status)
        write_json(self.exec_dir / 'config.json', self.describe())

    def describe(self):
        cfg = self.cfg
        return dict(schema=SCHEMA, result_kind='자리값 기반 SIM 결과', run_token=self.token, clock=self.args.clock,
                    owner=self.owner, config=cfg.to_dict(), config_digest=cfg.digest(),
                    socket_path=cfg.socket_path, operator_page='http://%s:%d' % (cfg.bind_host, cfg.bind_port),
                    controller=self.slot.spec, controller_digest=self.slot.digest,
                    profile_id=getattr(self.profile, 'profile_id', None),
                    profile_sha256=getattr(self.profile, 'sha256', None), host_profile_cal_id=self.host.cal_id,
                    source_check=self.source_check, source_digest=self.source_digest,
                    tuning=None if self.loop.tuning is None else self.loop.tuning.brief(),
                    topics=dict(scan=SCAN_TOPIC, imu=IMU_TOPIC, human=HUMAN_TOPIC,
                                clock=CLOCK_TOPIC if self.args.clock == 'sim' else None),
                    retired_status=self.retired, log_files_pruned=self.log_pruned,
                    source_commit=os.environ.get('UBTS_STEER_SOURCE_COMMIT'),
                    source_status_sha256=os.environ.get('UBTS_STEER_SOURCE_STATUS_SHA256'),
                    started_wall=self.started_wall)

    def status(self):
        loop = self.loop
        runner = loop.runner
        client = loop.bridge
        return dict(schema=SCHEMA, run_token=self.token, updated_at=time.time(), clock=self.args.clock,
                    executor_clock=round(self.clock(), 4), wall=round(self.wall(), 3),
                    wall_offset_s=self.wall.offset_s,
                    sim_clock=None if self.sim_clock is None else dict(t=self.sim_clock.t,
                                                                       samples=self.sim_clock.samples,
                                                                       backwards=self.sim_clock.backwards),
                    grid=dict(self.grid), runner=dict(state=runner.state, reason=runner.reason, run_id=runner.run_id,
                                                      end_reason=runner.end_reason, end_note=runner.end_note),
                    client=None if client is None else client.snapshot(), counts=dict(loop.counts),
                    hooks=dict(stall_until=self.hooks_state.stall_until, stalls_requested=self.hooks_state.stalls_requested,
                               stalls_done=self.hooks_state.stalls_done, freezes=self.hooks_state.freezes,
                               events=list(self.hooks_state.events)[-20:]),
                    controller=self.slot.spec, controller_digest=self.slot.digest, controllers_built=self.slot.built,
                    input_monitor=None if self.monitor is None else self.monitor.summary(),
                    log=self.log.health(), config_digest=self.loop._config_digest,   # after a field tuning change too
                    tuning=None if self.loop.tuning is None else self.loop.tuning.brief(),
                    source_check=self.source_check)


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        container = Container(args)
    except (ContainerError, ValueError, OSError, ImportError, RuntimeError) as exc:
        print('exec_container_main: startup refused: %s' % exc, file=sys.stderr)
        return 2
    XM = container.XM
    loop, cfg = container.loop, container.cfg
    stop = threading.Event()

    def on_signal(*_):
        stop.set()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, on_signal)
    from row_executor.operator_http import make_server
    token = secrets.token_urlsafe(18)
    try:
        server = make_server(loop, cfg.bind_host, cfg.bind_port, token)
    except OSError as exc:
        print('exec_container_main: operator page bind %s:%d failed: %s' % (cfg.bind_host, cfg.bind_port, exc),
              file=sys.stderr)
        container.log.close()
        return 3
    XM.write_token(str(container.log_dir / 'operator_token.txt'), token)
    threading.Thread(target=server.serve_forever, name='row-executor-http', daemon=True).start()
    loop._event('operator_page', url_host='%s:%d' % (cfg.bind_host, cfg.bind_port))
    loop._event('gsim_container', **{k: v for k, v in container.describe().items() if k != 'config'})
    ros = None
    status_stop = threading.Event()
    try:
        try:
            if args.clock == 'sim':
                ros = SimRosInput(loop, container.sim_clock, monitor=container.monitor)
            else:
                ros = XM.RosInput(loop, SCAN_TOPIC, IMU_TOPIC, HUMAN_TOPIC)
        except ImportError as exc:
            print('exec_container_main: rclpy not importable: %s' % exc, file=sys.stderr)
            return 4
        for sig in (signal.SIGINT, signal.SIGTERM):        # after rclpy.init: this process' own stop handling
            signal.signal(sig, on_signal)
        container.hooks.start()
        watchdog = threading.Thread(target=XM.run_watchdog, args=(loop, stop), name='row-executor-watchdog',
                                    daemon=True)
        watchdog.start()
        if container.monitor is not None:
            threading.Thread(target=gil_probe, args=(container.monitor, status_stop), name='exec-gil-probe',
                             daemon=True).start()

        def status_writer():
            while not status_stop.wait(STATUS_PERIOD_S):
                try:
                    write_json(container.exec_dir / 'status.json', container.status())
                except Exception as exc:       # the status file never stops the executor
                    loop._event('status_file_error', error=repr(exc)[:200])
        threading.Thread(target=status_writer, name='exec-status', daemon=True).start()
        print('exec_container_main: row executor running: clock=%s mode=MOTOR config=%s controller=%s owner=%s' % (
            args.clock, cfg.digest()[:12], container.slot.digest[:12], container.owner), flush=True)
        if args.max_runtime_s is not None:
            threading.Timer(float(args.max_runtime_s), stop.set).start()
        XM.freeze_startup_objects(loop)
        if args.clock == 'sim':
            run_loop_sim(container.stepped, stop, container.sim_clock, cfg.control_hz, grid=container.grid,
                         monitor=container.monitor)
        else:
            XM.run_loop(container.stepped, stop, cfg.control_hz)
    finally:
        status_stop.set()
        try:
            loop.shutdown('shutdown')
        finally:
            container.hooks.close()
            server.shutdown()
            server.server_close()
            if ros is not None:
                ros.close()
            try:
                write_json(container.exec_dir / 'status.json', dict(container.status(), final=True))
            except Exception:
                pass
            container.log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
