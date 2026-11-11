"""Scripted clients of the steering G-SIM (P6a checks; docs/steering/P6A_HEADLESS_KO.md).

ControlClient   the control API of the bridge wrapper (steer_control_core rules) over the Unix socket
                /run/ubts-steer/control.sock: one JSON object line per connection, one reply line.
                status(), arm(vehicle_profile='front_steer'), drive(throttle, steer_mrad, deadman=True), keepalive(),
                release(), motion(yaw_rate_rad_s, speed_m_s), stop(mode='brake'):
                  brake    op stop (MCU STOP: drive brake, steering hold_measured, DISARMED; no lease needed)
                  release  op release (throttle 0: the MCU ramps the duty down, the steering target is kept)
                  expire   no request at all: the input deadline (0.75 s) runs out, the bridge stops sending C and
                           the MCU's C TTL brakes (the vehicle behaviour of a client that dies)
DriveLoop       a thread that repeats drive(throttle, steer_mrad) every 0.1 s (inside the 0.75 s input deadline)
                until stop(); set() changes the command
HookClient      the emulator's fault hooks (gz_esp_emulator; /run/ubts-steer-emu/hooks.sock)
SimObserver     an evaluation-side rclpy node (ubts_steer_p6a_check): /clock, /ground_truth/odom, /sim/truth/steering,
                /rover/steering/state, /rover/status, /imu/data_raw and on request /sim/truth/joint_states (A25 on the
                emulator path); it reads truth, so it is an evaluation consumer (A13 allow list), never a controller
read_jsonl()    the emulator / bridge event logs

Placeholder-based SIM results, not field validation.
"""
from __future__ import annotations

import collections
import json
import math
import os
from pathlib import Path
import socket
import threading
import time

CONTROL_SOCKET = os.environ.get('ROVER_CONTROL_SOCKET', '/run/ubts-steer/control.sock')
HOOK_SOCKET = os.environ.get('UBTS_STEER_HOOK_SOCKET', '/run/ubts-steer-emu/hooks.sock')
VEHICLE_PROFILE = 'front_steer'
OBSERVER_NODE = 'ubts_steer_p6a_check'
STOP_MODES = ('brake', 'release', 'expire')


class ClientError(RuntimeError):
    pass


def unix_request(path, payload, timeout_s=2.):
    """One JSON line to a Unix socket and the reply line (dict)."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout_s)
        sock.connect(path)
        sock.sendall(json.dumps(payload, allow_nan=False).encode('utf-8') + b'\n')
        data = b''
        while not data.endswith(b'\n'):
            part = sock.recv(65536)
            if not part:
                break
            data += part
    if not data:
        raise ClientError('no reply from %s' % path)
    return json.loads(data.decode('utf-8'))


class ControlClient:
    """Control API client of one owner (module docstring). The last grant (lease, ticket, seq) is kept."""

    def __init__(self, owner='p6a-check', socket_path=CONTROL_SOCKET, timeout_s=2.):
        self.owner = owner
        self.socket_path = socket_path
        self.timeout_s = timeout_s
        self.lease = self.ticket = None
        self.seq = 0
        self.lock = threading.Lock()
        self.log = collections.deque(maxlen=4096)

    def request(self, payload):
        started = time.monotonic()
        reply = unix_request(self.socket_path, payload, self.timeout_s)
        self.log.append(dict(t=time.time(), op=payload.get('op'), ok=reply.get('ok'), error=reply.get('error'),
                             rtt_ms=(time.monotonic() - started) * 1000.))
        return reply

    def status(self):
        return self.request({'op': 'status'})

    def arm(self, vehicle_profile=VEHICLE_PROFILE):
        payload = {'op': 'arm', 'owner': self.owner}
        if vehicle_profile is not None:
            payload['vehicle_profile'] = vehicle_profile
        with self.lock:
            reply = self.request(payload)
            if reply.get('ok'):
                self.lease, self.ticket, self.seq = reply['lease'], reply['ticket'], reply['seq']
            return reply

    def _owned(self, op, **fields):
        with self.lock:
            if not self.lease:
                return {'ok': False, 'error': 'client_has_no_lease'}
            payload = dict(op=op, owner=self.owner, lease=self.lease, ticket=self.ticket, seq=self.seq + 1, **fields)
            reply = self.request(payload)
            if reply.get('ok'):
                self.ticket, self.seq = reply['ticket'], reply['seq']
            elif reply.get('error') not in ('request_expired',):
                self.lease = None
            return reply

    def drive(self, throttle, steer_mrad, deadman=True):
        return self._owned('drive', throttle=int(throttle), steer_mrad=int(steer_mrad), deadman=bool(deadman))

    def drive_differential(self, left, right):
        """The old screen's drive (left/right): refused by the steering bridge (A14)."""
        return self._owned('drive', left=int(left), right=int(right), deadman=True)

    def keepalive(self):
        return self._owned('keepalive')

    def release(self):
        return self._owned('release')

    def motion(self, yaw_rate_rad_s, speed_m_s):
        with self.lock:
            return self.request(dict(op='motion', owner=self.owner, lease=self.lease, yaw_rate_rad_s=yaw_rate_rad_s,
                                     speed_m_s=speed_m_s))

    def stop(self, mode='brake'):
        if mode not in STOP_MODES:
            raise ValueError('stop mode must be one of %s' % (STOP_MODES,))
        if mode == 'brake':
            reply = self.request({'op': 'stop', 'reason': 'p6a_client'})
            with self.lock:
                self.lease = None
            return reply
        if mode == 'release':
            return self.release()
        return {'ok': True, 'state': 'expire: no further requests'}


class DriveLoop:
    """Repeats drive(throttle, steer_mrad) every period_s in a thread (start(), set(), stop())."""

    def __init__(self, client, throttle=0, steer_mrad=0, period_s=.1):
        self.client = client
        self.command = (int(throttle), int(steer_mrad))
        self.period_s = period_s
        self.stopped = threading.Event()
        self.replies = collections.deque(maxlen=2048)
        self.errors = collections.Counter()
        self.thread = None

    def set(self, throttle, steer_mrad):
        self.command = (int(throttle), int(steer_mrad))

    def start(self):
        self.thread = threading.Thread(target=self._run, name='p6a-drive-loop', daemon=True)
        self.thread.start()
        return self

    def _run(self):
        while not self.stopped.is_set():
            throttle, steer = self.command
            try:
                reply = self.client.drive(throttle, steer)
            except (OSError, ValueError, ClientError) as exc:
                reply = {'ok': False, 'error': '%s: %s' % (type(exc).__name__, exc)}
            self.replies.append((time.time(), reply.get('ok'), reply.get('error')))
            if not reply.get('ok'):
                self.errors[reply.get('error')] += 1
            self.stopped.wait(self.period_s)

    def stop(self):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(timeout=2.)


class HookClient:
    """The emulator's fault hooks (gz_esp_emulator docstring)."""

    def __init__(self, socket_path=HOOK_SOCKET, timeout_s=6.):
        self.socket_path = socket_path
        self.timeout_s = timeout_s

    def request(self, payload):
        reply = unix_request(self.socket_path, payload, self.timeout_s)
        if not reply.get('ok'):
            raise ClientError('hook %s failed: %s' % (payload.get('op'), reply.get('error')))
        return reply

    def __getattr__(self, op):
        if op.startswith('_'):
            raise AttributeError(op)
        return lambda **fields: self.request(dict(op=op, **fields))


def read_jsonl(path, since_line=0):
    """Entries of a JSONL log (the .1 rotation file first if it exists and since_line is 0)."""
    path = Path(path)
    out = []
    for item in ([Path(str(path) + '.1')] if since_line == 0 else []) + [path]:
        if not item.exists():
            continue
        with open(item, encoding='utf-8') as handle:
            for line in handle:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        continue
    return out[since_line:] if since_line else out


def yaw_of(q):
    return math.atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z))


class SimObserver:
    """Evaluation-side ROS observer (truth allowed). spin_for(seconds) / spin_until(predicate, timeout_s)."""

    def __init__(self, node_name=OBSERVER_NODE, record_truth=True, imu=False):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String
        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(node_name, parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        best = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sim_time = None
        self.clock_rows = collections.deque(maxlen=200000)        # (wall monotonic, sim s)
        self.odom = collections.deque(maxlen=200000)              # (sim s, x, y, yaw, v, w)
        self.truth = collections.deque(maxlen=200000)             # dict rows of /sim/truth/steering
        self.steering_states = collections.deque(maxlen=20000)
        self.rover_status = None
        self.imu = collections.deque(maxlen=20000)
        self.joints = collections.deque(maxlen=200000)          # (sim s, {name: (position, velocity)})
        self._joint_subscription = None
        self.node.create_subscription(Clock, '/clock', self._clock, best)
        if record_truth:
            self.node.create_subscription(Odometry, '/ground_truth/odom', self._odom, best)
            self.node.create_subscription(String, '/sim/truth/steering', self._truth, 50)
        self.node.create_subscription(String, '/rover/steering/state', self._state, 50)
        self.node.create_subscription(String, '/rover/status', self._status, 10)
        if imu:
            self.node.create_subscription(Imu, '/imu/data_raw', self._imu, best)

    def enable_joints(self):
        """Subscribe /sim/truth/joint_states (truth, evaluation only) once."""
        if self._joint_subscription is None:
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import JointState
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200, reliability=ReliabilityPolicy.RELIABLE)
            self._joint_subscription = self.node.create_subscription(JointState, '/sim/truth/joint_states',
                                                                     self._joint, qos)

    def _joint(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        velocity = list(msg.velocity) + [None] * (len(msg.name) - len(msg.velocity))
        self.joints.append((stamp, {name: (float(pos), None if vel is None else float(vel))
                                    for name, pos, vel in zip(msg.name, msg.position, velocity)}))

    def _clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.sim_time = t
        self.clock_rows.append((time.monotonic(), t))

    def _odom(self, msg):
        p, q, tw = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        self.odom.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, p.x, p.y, yaw_of(q), tw.linear.x,
                          tw.angular.z))

    def _truth(self, msg):
        try:
            self.truth.append(json.loads(msg.data))
        except ValueError:
            pass

    def _state(self, msg):
        try:
            self.steering_states.append((time.monotonic(), json.loads(msg.data)))
        except ValueError:
            pass

    def _status(self, msg):
        try:
            self.rover_status = json.loads(msg.data)
        except ValueError:
            pass

    def _imu(self, msg):
        g, a = msg.angular_velocity, msg.linear_acceleration
        self.imu.append((time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, g.x, g.y, g.z,
                         a.x, a.y, a.z, msg.header.frame_id, list(msg.orientation_covariance)))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=min(.05, max(0., end - time.monotonic())))

    def spin_sim_for(self, sim_seconds, timeout_s=None):
        """Spin until the SIM clock advanced sim_seconds (wall limit timeout_s, default 10 x + 30 s)."""
        self.spin_until(lambda: self.sim_time is not None, 30.)
        start = self.sim_time
        limit = timeout_s if timeout_s is not None else 10. * sim_seconds + 30.
        if not self.spin_until(lambda: self.sim_time - start >= sim_seconds, limit):
            raise ClientError('SIM clock advanced %.2f of %.2f s in %.0f s wall' % (self.sim_time - start, sim_seconds,
                                                                                    limit))
        return self.sim_time

    def spin_until(self, predicate, timeout_s):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if predicate():
                return True
            self.executor.spin_once(timeout_sec=.02)
        return bool(predicate())

    def last_pose(self):
        return self.odom[-1] if self.odom else None

    def close(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=1.)
        self.node.destroy_node()
