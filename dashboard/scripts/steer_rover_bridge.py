#!/usr/bin/env python3
"""Front-steer ESP V3 bridge wrapper: control socket, ROS observation topics, transport and clock selection.

T2 (G-SIM, pty under /run/ubts-steer) and T3 (vehicle, USB by-id serial) use this wrapper; T1 drives
SteerBridgeCore directly. Run from the dashboard root: python3 -m scripts.steer_rover_bridge --help

Socket protocol: the same as scripts/rover_control_bridge.py (not imported): one JSON object line of at most
4096 B per connection, one JSON response line, 0.4 s read timeout, socket mode 0660, a process lock next to the
socket. ROS only publishes observations (rclpy is imported lazily, so the module imports without ROS):
  /rover/status (2 Hz, bridge snapshot with the steering block), /rover/control/requested,
  /rover/control/applied (2 Hz), /rover/steering/state (each new MCU status line, nominal 20 Hz).

Identity (design 2.5, A14): the transport decides the role. serial = real-hardware wrapper, which refuses a device
whose banner says sim=1; pty and memory = SIM wrapper, which refuses sim=0. The SIM pty must be under
/run/ubts-steer and the real transport refuses that root. The role also binds the control socket (the real wrapper
refuses a socket under /run/ubts-steer, the SIM wrapper one under /run/ubts-rover), the clock (--clock sim only in
the SIM role) and the calibration id (the real wrapper needs a host profile with a fixed cal_id).
No automatic re-arm after any refusal or fault. An unexpected exception in a loop tick latches internal_error,
stops and the loop goes on.

Loop timing: --clock wall steps the core every 20 ms of the monotonic clock. --clock sim (G-SIM SIM clock mode,
2026-11-09 P6a) steps it on a 20 ms grid of the SIM clock: the loop polls ROS every 2 ms wall and runs one core step
each time a /clock sample reaches the next multiple of 20 ms (a jump over several grid points runs one step and
counts the skipped ones). /clock comes in 1/120 s samples, so a grid step runs up to one sample after its grid point
and a request on the grid waits up to 20 + 8.3 ms of SIM time. A STOP (the core's emergency flag: op stop, or a full
queue) does not wait for the grid: the next poll runs one extra core step at the current SIM time, and the ROS spin
stops taking /clock samples once a STOP is pending, so a STOP waits at most one /clock sample (8.3 ms SIM; review
RG-04, 2026-11-09). The periodic work (C resends, the STOP resend grid, the status cadence) stays on the 20 ms grid.
--event-log PATH (optional, default off): a rotating JSONL file with every control request (op, owner, time of the
injected clock), every transport write (first bytes, same clock) and, while a STOP is pending, every loop tick
('tick', time read before the core step), for the G-SIM timing checks (A17: a STOP resend at the first loop tick
after its grid time).
"""
import argparse
import json
import math
import os
import signal
import socketserver
import sys
import threading
import time

from scripts import esp_motor_v3 as v3
from scripts.steer_bridge_core import HostProfile, HostProfileError, SteerBridgeCore

SIM_ROOT = '/run/ubts-steer'
REAL_ROOT = '/run/ubts-rover'
SIM_SOCKET = SIM_ROOT + '/control.sock'
REAL_SOCKET = REAL_ROOT + '/control.sock'
REAL_DEVICE = '/dev/ubts-motor'
LOOP_PERIOD_S = 0.02
SIM_POLL_PERIOD_S = 0.002
SIM_SPIN_MAX = 50
STATUS_PERIOD_S = 0.5
STALE_STATE_REPUBLISH_S = 0.5
MAX_REQUEST_BYTES = 4096
EVENT_LOG_ROTATE_BYTES = 32 * 1024 * 1024


class ConfigError(ValueError):
    pass


def build_parser():
    p = argparse.ArgumentParser(prog='steer_rover_bridge', description=__doc__.splitlines()[0])
    p.add_argument('--transport', choices=('memory', 'pty', 'serial'),
                   default=os.environ.get('UBTS_STEER_TRANSPORT', 'serial'),
                   help='serial = real ESP (T3), pty = ESP emulator (T2), memory = embedding/tests only')
    p.add_argument('--device', default=None,
                   help='serial: by-id path (default $ROVER_SERIAL_PORT or %s); pty: default %s'
                        % (REAL_DEVICE, v3.SIM_DEVICE_PATH))
    p.add_argument('--identity', default=os.environ.get('ROVER_ESP_IDENTITY', ''),
                   help='serial only: udev ID_SERIAL of the ESP (default $ROVER_ESP_IDENTITY)')
    p.add_argument('--clock', choices=('wall', 'sim'), default='wall',
                   help='wall = monotonic host clock; sim = ROS /clock (needs ROS, G-SIM SIM clock mode)')
    p.add_argument('--host-profile', default=os.environ.get('UBTS_STEER_HOST_PROFILE'),
                   help='host profile JSON (sim/ubts/steering/host_profile_export.py); default $UBTS_STEER_HOST_PROFILE')
    p.add_argument('--socket', default=None,
                   help='control socket (default $ROVER_CONTROL_SOCKET, else %s for SIM, %s for real)'
                        % (SIM_SOCKET, REAL_SOCKET))
    p.add_argument('--no-socket', action='store_true', help='do not serve the control socket (tests)')
    p.add_argument('--status-file', default=os.environ.get('ROVER_BRIDGE_STATUS_FILE'),
                   help='write the snapshot JSON here every 0.5 s (default $ROVER_BRIDGE_STATUS_FILE, else none)')
    p.add_argument('--ros', dest='ros', action='store_true', default=True, help='publish ROS topics (default)')
    p.add_argument('--no-ros', dest='ros', action='store_false', help='no ROS (tests, bench tools)')
    p.add_argument('--node-name', default='ubts_steer_rover_bridge')
    p.add_argument('--max-runtime-s', type=float, default=None, help='stop after this many seconds (tests)')
    p.add_argument('--event-log', default=os.environ.get('UBTS_STEER_BRIDGE_EVENT_LOG'),
                   help='JSONL log of control requests and transport writes (G-SIM timing checks; default off)')
    return p


def resolve_config(args, have_transport=False):
    """Check the argument combination and return a plain config dict. Raises ConfigError."""
    if not args.host_profile:
        raise ConfigError('host profile JSON required (--host-profile or $UBTS_STEER_HOST_PROFILE)')
    if args.clock == 'sim' and not args.ros:
        raise ConfigError('--clock sim reads ROS /clock; it cannot run with --no-ros')
    simulated = args.transport != 'serial'
    if args.clock == 'sim' and not simulated:
        raise ConfigError('--clock sim is the G-SIM SIM clock mode; the real-hardware (serial) bridge runs on the '
                          'monotonic clock (design 2.4)')
    if args.transport == 'memory' and not have_transport:
        raise ConfigError('--transport memory needs an in-process device (embedding or tests only)')
    device = args.device
    if args.transport == 'serial':
        device = device or os.environ.get('ROVER_SERIAL_PORT', REAL_DEVICE)
        if not args.identity:
            raise ConfigError('--identity (udev ID_SERIAL) required for the serial transport')
    elif args.transport == 'pty':
        device = device or v3.SIM_DEVICE_PATH
        if not v3.path_under(device, v3.SIM_DEVICE_ROOT):
            raise ConfigError('SIM pty device must be under %s' % v3.SIM_DEVICE_ROOT)
    elif args.device:
        raise ConfigError('--device is not used with --transport memory')
    if args.max_runtime_s is not None and not args.max_runtime_s > 0:
        raise ConfigError('--max-runtime-s must be positive')
    socket_path = None
    if not args.no_socket:
        socket_path = args.socket or os.environ.get('ROVER_CONTROL_SOCKET') or (SIM_SOCKET if simulated
                                                                                else REAL_SOCKET)
        other = REAL_ROOT if simulated else SIM_ROOT
        if v3.path_under(socket_path, other):
            raise ConfigError('the %s bridge must not serve a control socket under %s (%s)' % (
                'SIM' if simulated else 'real-hardware', other, socket_path))
    return dict(transport=args.transport, device=device, identity=args.identity if args.transport == 'serial' else '',
                simulated=simulated, clock=args.clock, host_profile=args.host_profile, socket=socket_path,
                status_file=args.status_file, ros=args.ros, node_name=args.node_name,
                max_runtime_s=args.max_runtime_s, event_log=getattr(args, 'event_log', None) or None)


def make_transport(config, memory_transport=None, baud=v3.DEFAULT_BAUD):
    kind = config['transport']
    if kind == 'memory':
        if memory_transport is None:
            raise ConfigError('memory transport not given')
        return memory_transport
    if kind == 'pty':
        return v3.PtyTransport(config['device'], baud=baud)
    return v3.SerialTransport(config['device'], config['identity'], baud=baud)


class EventLog:
    """Rotating JSONL file (one previous file kept as .1); write() is thread safe. path None: nothing is written."""

    def __init__(self, path, clock, rotate_bytes=EVENT_LOG_ROTATE_BYTES):
        self.path = path
        self.clock = clock
        self.rotate_bytes = rotate_bytes
        self.lock = threading.Lock()
        self.size = 0
        self.handle = None
        if path:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            self.handle = open(path, 'w', encoding='utf-8', newline='\n')

    @property
    def enabled(self):
        return self.handle is not None

    def write(self, kind, **fields):
        if self.handle is None:
            return
        entry = dict(kind=kind, t=self.clock(), wall=time.time(), **fields)
        text = json.dumps(entry, allow_nan=False, default=str, separators=(',', ':')) + '\n'
        with self.lock:
            if self.handle is None:
                return
            self.handle.write(text)
            self.size += len(text)
            if self.size >= self.rotate_bytes:
                self.handle.close()
                os.replace(self.path, self.path + '.1')
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


class LoggedTransport:
    """Transport wrapper that logs every write (injected-clock time, size, the first bytes) to an EventLog."""

    def __init__(self, transport, log):
        self._transport = transport
        self._log = log

    def write(self, data):
        self._log.write('tx', size=len(data), head=bytes(data[:48]).decode('ascii', 'replace'))
        return self._transport.write(data)

    def __getattr__(self, name):
        return getattr(self._transport, name)


class LoggedCore:
    """Request front of a SteerBridgeCore that logs each control request (op, owner, injected-clock time)."""

    def __init__(self, core, log):
        self._core = core
        self._log = log

    def request(self, request, timeout=None):
        op = request.get('op') if isinstance(request.get('op'), str) else None
        owner = request.get('owner') if isinstance(request.get('owner'), str) else None
        self._log.write('request', op=op, owner=owner)
        reply = self._core.request(request, timeout)
        self._log.write('reply', op=op, ok=reply.get('ok'), error=reply.get('error'))
        return reply


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(0.4)
        try:
            line = self.rfile.readline(MAX_REQUEST_BYTES + 1)
            if len(line) > MAX_REQUEST_BYTES:
                raise ValueError('request_too_large')
            request = json.loads(line)
            if not isinstance(request, dict):
                raise ValueError('object_required')
            response = self.server.bridge.request(request)
        except (ValueError, OSError) as exc:
            response = {'ok': False, 'error': str(exc)}
        try:
            self.wfile.write(json.dumps(response, allow_nan=False).encode() + b'\n')
        except (OSError, ValueError):
            pass


def _server_class():
    base = getattr(socketserver, 'ThreadingUnixStreamServer', None)
    if base is None:
        raise ConfigError('Unix socket server not available on this platform')

    class Server(base):
        daemon_threads = True
        request_queue_size = 8
    return Server


class RosPublisher:
    """Lazy rclpy publishers (and the /clock time for --clock sim)."""

    def __init__(self, node_name, use_sim_time=False):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        self.rclpy, self.String = rclpy, String
        rclpy.init()
        if use_sim_time:
            from rclpy.parameter import Parameter
            self.node = rclpy.create_node(node_name, parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        else:
            self.node = rclpy.create_node(node_name)
        self.pubs = {topic: self.node.create_publisher(String, topic, 1) for topic in (
            '/rover/status', '/rover/control/requested', '/rover/control/applied', '/rover/steering/state')}
        self._sim_last = 0.0
        self.use_sim_time = use_sim_time
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def publish(self, topic, payload):
        msg = self.String()
        msg.data = json.dumps(payload, allow_nan=False)
        self.pubs[topic].publish(msg)

    def spin_once(self, stop_when=None):
        if not self.use_sim_time:
            self.executor.spin_once(timeout_sec=0.0)
            return
        # SIM clock: take every queued /clock message now (one callback per spin_once), at most SIM_SPIN_MAX; stop
        # early when stop_when() turns true (a pending STOP is stepped at the SIM time it arrived in).
        last = None
        for _ in range(SIM_SPIN_MAX):
            if stop_when is not None and stop_when():
                break
            self.executor.spin_once(timeout_sec=0.0)
            now = self.node.get_clock().now().nanoseconds
            if now == last:
                break
            last = now

    def sim_time(self):
        """ROS /clock seconds, never decreasing; 0.0 before the first /clock message."""
        now = self.node.get_clock().now().nanoseconds / 1e9
        self._sim_last = max(self._sim_last, now)
        return self._sim_last

    def close(self):
        try:
            self.executor.remove_node(self.node)
            self.executor.shutdown(timeout_sec=0.5)
            self.node.destroy_node()
        finally:
            self.rclpy.shutdown()


class BridgeRunner:
    """Owns the core, the socket server, the ROS publishers and the status file. loop_once() is one 20 ms tick."""

    def __init__(self, config, transport=None, clock=None, ros=None, wall_clock=time.time):
        self.config = config
        self.wall_clock = wall_clock
        self.hp = HostProfile.load(config['host_profile'])
        if not config['simulated'] and self.hp.cal_id is None:
            raise ConfigError('host profile calibration.cal_id is null ("any"): only the SIM role may accept any '
                              'cal_id; the real-hardware bridge needs the cal_id of the vehicle calibration')
        if config['clock'] == 'sim' and not config['simulated']:
            raise ConfigError('--clock sim is only for the SIM role')
        self.ros = ros
        if self.ros is None and config['ros']:
            self.ros = RosPublisher(config['node_name'], use_sim_time=config['clock'] == 'sim')
        if clock is None:
            if config['clock'] == 'sim' and self.ros is None:
                raise ConfigError('--clock sim needs the ROS /clock')
            clock = self.ros.sim_time if config['clock'] == 'sim' else time.monotonic
        self.clock = clock
        self.event_log = EventLog(config.get('event_log'), clock)
        transport = make_transport(config, transport, baud=self.hp.baud)
        if self.event_log.enabled:
            transport = LoggedTransport(transport, self.event_log)
        self.transport = transport
        self.link = v3.EspSteerV3(self.transport, clock, simulated=config['simulated'], expect_hw=self.hp.hw_profile,
                                  banner_timeout_s=self.hp.banner_timeout_s)
        self.core = SteerBridgeCore(self.link, self.hp, clock=clock, wall_clock=wall_clock)
        self.server = None
        self.lock = None
        self._published = None
        self._state_published = None
        self.grid = dict(next_t=None, steps=0, skipped=0, polls=0, urgent_steps=0)
        self._latches_logged = ()
        self._stop_pending_logged = False

    def start(self):
        path = self.config['socket']
        if path:
            import fcntl
            directory = os.path.dirname(path) or '.'
            os.makedirs(directory, exist_ok=True)
            self.lock = open(os.path.join(directory, 'bridge.lock'), 'a')
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)   # never delete a live peer's socket
            if os.path.exists(path):
                os.unlink(path)
            self.server = _server_class()(path, Handler)
            os.chmod(path, 0o660)
            self.server.bridge = LoggedCore(self.core, self.event_log) if self.event_log.enabled else self.core
            threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def sim_clock_started(self):
        return self.config['clock'] != 'sim' or self.clock() > 0.0

    def loop_once(self):
        """One loop tick: ROS callbacks, then one core step (wall clock mode and tests)."""
        if self.ros is not None:
            self.ros.spin_once()
        if not self.sim_clock_started():
            return
        self._tick()

    def stop_pending(self):
        """True while the core holds a STOP request that no step has taken yet (op stop or a full queue)."""
        flag = getattr(self.core, 'emergency', None)
        return bool(flag is not None and flag.is_set())

    def _tick(self):
        if self.event_log.enabled and (self._stop_pending_logged or self.stop_pending()):
            self.event_log.write('tick')      # loop tick time before the core step, while a STOP is pending
        self.core.step()
        now = self.clock()
        if self._published is None or now - self._published >= STATUS_PERIOD_S:
            self._published = now
            snapshot = self.core.snapshot
            if self.ros is not None:
                self.ros.publish('/rover/status', snapshot)
                self.ros.publish('/rover/control/requested', snapshot.get('requested', {}))
                self.ros.publish('/rover/control/applied', self.core.applied())
            self._write_status_file(snapshot)
            self.event_log.flush()
        # One /rover/steering/state per MCU status line, also when a loop tick read several (design 4.5, A23);
        # without new lines the last state is sent again (stale: true) every STALE_STATE_REPUBLISH_S.
        taken = self.link.take_status_history()
        for line, received in taken:
            if self.ros is not None:
                self.ros.publish('/rover/steering/state', self.core.steering_state(line, received))
        if taken:
            self._state_published = now
        elif self._state_published is None or now - self._state_published >= STALE_STATE_REPUBLISH_S:
            self._state_published = now
            if self.ros is not None:
                self.ros.publish('/rover/steering/state', self.core.steering_state())
        if self.event_log.enabled:
            host = self.core.steering_host()
            state = (tuple(self.core.latches), host['stop_state'], host['arm_state'], self.core.snapshot.get('state'))
            if state != self._latches_logged:
                self._latches_logged = state
                self.event_log.write('state', latches=list(state[0]), stop_state=state[1], arm_state=state[2],
                                     state=state[3], reason=self.core.snapshot.get('reason'))
            # the loop ticks after a STOP are logged until it is confirmed or given up (A17 resend timing)
            self._stop_pending_logged = host['stop_state'] == 'pending'

    def sim_grid_once(self, period_s=LOOP_PERIOD_S):
        """SIM clock mode: ROS callbacks, then one core step if the SIM time reached the next grid point, or at once
        (an extra step off the grid) when a STOP is pending. Returns True when a step ran."""
        if self.ros is not None and not self.stop_pending():
            self.ros.spin_once(stop_when=self.stop_pending)
        self.grid['polls'] += 1
        if not self.sim_clock_started():
            return False
        now = self.clock()
        due = self.grid['next_t']
        if due is None:
            due = math.floor(now / period_s + 1e-9) * period_s
        if now + 1e-9 < due:
            if not self.stop_pending():
                return False
            self.grid['urgent_steps'] += 1          # STOP: one step now; the grid keeps its next point
            self._tick()
            return True
        missed = int(math.floor((now - due) / period_s + 1e-9))
        if missed > 0:
            self.grid['skipped'] += missed
        self.grid['next_t'] = due + (missed + 1) * period_s
        self.grid['steps'] += 1
        self._tick()
        return True

    def _write_status_file(self, snapshot):
        target = self.config['status_file']
        if not target:
            return
        directory = os.path.dirname(target) or '.'
        os.makedirs(directory, exist_ok=True)
        tmp = target + '.tmp'
        snapshot = dict(snapshot, bridge_loop=dict(clock=self.config['clock'], **self.grid))
        with open(tmp, 'w', encoding='utf-8', newline='\n') as handle:
            handle.write(json.dumps(snapshot, allow_nan=False))
        os.replace(tmp, target)

    def run(self, stop_event):
        deadline = None
        if self.config['max_runtime_s'] is not None:
            deadline = time.monotonic() + self.config['max_runtime_s']
        sim = self.config['clock'] == 'sim'
        # Wall clock: a fixed 20 ms grid of the monotonic clock (a request waits at most one period); a tick that
        # is more than one period late resynchronises the grid and is counted (overruns).
        next_t = time.monotonic()
        while True:
            if sim:
                if stop_event.wait(SIM_POLL_PERIOD_S):
                    break
            else:
                next_t += LOOP_PERIOD_S
                delay = next_t - time.monotonic()
                if delay < -LOOP_PERIOD_S:
                    self.grid['skipped'] += 1
                    next_t = time.monotonic()
                    delay = 0.
                if stop_event.wait(max(0., delay)):
                    break
            try:
                if sim:
                    self.sim_grid_once()
                else:
                    self.grid['steps'] += 1
                    self.loop_once()
            except Exception as exc:   # a defect (ROS publish, status file, ...): latch, stop, keep the loop
                print('steer_rover_bridge: loop error: %s: %s' % (type(exc).__name__, exc), file=sys.stderr)
                self.core._internal_error('loop', exc)
            if deadline is not None and time.monotonic() >= deadline:
                break

    def shutdown(self):
        try:
            self.core.close('shutdown')
        finally:
            if self.server is not None:
                self.server.shutdown()
                self.server.server_close()
                if self.config['socket'] and os.path.exists(self.config['socket']):
                    os.unlink(self.config['socket'])
            if self.ros is not None:
                self.ros.close()
            if self.lock is not None:
                self.lock.close()
            self.event_log.close()


def main(argv=None, transport=None):
    args = build_parser().parse_args(argv)
    try:
        config = resolve_config(args, have_transport=transport is not None)
        runner = BridgeRunner(config, transport=transport)
    except (ConfigError, HostProfileError, OSError) as exc:
        print('steer_rover_bridge: %s' % exc, file=sys.stderr)
        return 2
    stopped = threading.Event()
    if threading.current_thread() is threading.main_thread():
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stopped.set())
    try:
        runner.start()
    except (ConfigError, OSError) as exc:     # e.g. another bridge holds the socket lock
        print('steer_rover_bridge: %s' % exc, file=sys.stderr)
        runner.shutdown()
        return 2
    try:
        runner.run(stopped)
    finally:
        runner.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
