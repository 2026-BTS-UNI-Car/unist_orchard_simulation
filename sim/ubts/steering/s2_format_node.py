"""S2 format node of the steering G-SIM (design 2.3, 3.7, 5.2 and 8.3 P6a 'S2 형식 노드'; SIM_ACCEPTANCE A5).

/sim/scan (the 3200-beam Embree scan of the pinned cpu_lidar.py) or /sim/scan_clipped (the unobserved clip node's
output on field world scenes) -> /scan in the real RPLIDAR S2 format. The conversion is steering.s2_format.RealFormat,
the code the F-SIM executor-in-the-loop uses (tools/row_executor/sim_in_loop_steer.scan_message; decision R16):

  ranges          3240 beams in raw driver order, float32, rounded to 1 mm; 0.0 = no return (+inf, NaN, >= 30 m)
  angle_min       float32(-pi) = s2_format.RAW_AMIN; angle_increment = s2_format.RAW_INC; angle_max = angle_min +
                  3239 x increment
  frame_id        'laser'; raw beam j looks at the body angle wrap(angle_min + j x increment + lidar_yaw_offset_deg)
                  (the profile value, 179.08 deg by default: the 10/10 mount estimate, design 3.7)
  range_min/max   0.05 m / 30 m; scan_time 0.1 s; time_increment 0; no intensities (as the F-SIM scan message)
  header.stamp    the input stamp: cpu_lidar stamps a scan with the SIM time of the pose it used and the clip node keeps
                  it, so the executor sees the SIM sensor stamp as in the F-SIM

Input convention (checked, never guessed): the SIM scan of cpu_lidar.ScanSynthesizer, 3200 beams over 2 pi from
angle_min -pi in the body frame, frame 'laser'. A scan with another geometry is not converted: it is counted under its
reason and the status says 'input_rejected' (an unknown geometry would give a wrong beam order).
One input scan gives one output scan. No scan is merged, reused or dropped without a count: a gap of the input header
stamps above 1.5 x the 0.1 s period is counted as missing upstream scans (cpu_lidar, the clip node or the transport),
a stamp that does not increase is counted too. The output QoS is the real driver's (sllidar_ros2: reliable, keep last
10); the input QoS is best effort (the publishers of /sim/scan and /sim/scan_clipped are best effort).

The node reads no truth: no pose, no /ground_truth/*, no /sim/truth/* (A13: p6a_graph.S2_NODE is an allowed reader of
/sim/scan and /sim/scan_clipped; the executor reads /scan only).
Status file (--status-file, about 1 s; state waiting | running | input_rejected | stopped): counts (in, out, rejected
by reason, stamp gaps, missing scans, stamps not increasing), processing ms (last, mean, max: callback entry to the
publish call, wall), the last input and output stamps, the run token of the supervisor (--run-token), the profile id
and lidar_yaw_offset_deg.

  S2Converter            the pure conversion and counters (no ROS; tested directly)
  main()                 the ROS 2 node ubts_steer_s2_format (rclpy; SIM container, supervisor_steer starts it)

CLI (the supervisor's command, supervisor_steer.s2_node_command):
  python3 -B -m steering.s2_format_node --input /sim/scan --output /scan [--status-file F] [--profile P]
      [--yaw-offset-deg D] [--run-token T]
Placeholder-based SIM results (자리값 기반 SIM 결과), not field validation.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time

import numpy as np

from . import s2_format as S2

NODE_NAME = 'ubts_steer_s2_format'            # p6a_graph.S2_NODE
INPUT_TOPIC = '/sim/scan'
OUTPUT_TOPIC = '/scan'
SCHEMA = 'ubts.steer_s2_node_status.v1'
SIM_FRAME_ID = 'laser'                        # cpu_lidar's frame id (body frame values; design 3.7)
SIM_ANGLE_MIN = -math.pi                      # cpu_lidar.ScanSynthesizer.angle_min
SIM_INCREMENT = 2. * math.pi / S2.SIM_BEAMS   # cpu_lidar.ScanSynthesizer.angle_increment
ANGLE_TOL = 1e-6                              # LaserScan angles are float32 (about 1.2e-7 relative)
RAW_ANGLE_MAX = float(np.float32(S2.RAW_AMIN + (S2.BEAMS - 1) * S2.RAW_INC))
SCAN_TIME_S = .1                              # as tools/row_executor/sim_in_loop_steer.scan_message
TIME_INCREMENT_S = 0.
PERIOD_S = 1. / S2.SCAN_HZ
GAP_FACTOR = 1.5                              # a header stamp step above this x the period: missing scans upstream
STATUS_PERIOD_S = 1.
REJECT_REASONS = ('beam_count', 'angle_min', 'angle_increment', 'frame_id', 'stamp_invalid')


def stamp_ns(sec, nanosec):
    """Header stamp (sec, nanosec) -> int ns, or None when it is not a valid ROS time."""
    try:
        sec, nanosec = int(sec), int(nanosec)
    except (TypeError, ValueError):
        return None
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return None
    return sec * 1_000_000_000 + nanosec


class S2Converter:
    """SIM scan -> S2 ranges with counters (module docstring). convert(...) returns (ranges float32 or None, info)."""

    def __init__(self, yaw_offset_deg=S2.DEFAULT_YAW_OFFSET_DEG):
        offset = float(yaw_offset_deg)
        if not math.isfinite(offset):
            raise ValueError('lidar_yaw_offset_deg must be finite')
        self.yaw_offset_deg = offset
        self.real = S2.RealFormat(offset)
        self.counts = dict(scans_in=0, scans_out=0, rejected=0, stamp_gaps=0, missing_scans=0,
                           stamps_not_increasing=0, returns_out=0, no_return_out=0)
        self.rejected = {reason: 0 for reason in REJECT_REASONS}
        self.last_stamp_ns = None
        self.last_out_stamp_ns = None
        self.last_reject = None
        self.ms = dict(last=None, max=None, total=0.)            # conversion (RealFormat and counters)
        self.callback_ms = dict(last=None, max=None, total=0.)   # node callback: entry to the publish call (wall)

    @staticmethod
    def geometry_problem(count, angle_min, angle_increment, frame_id):
        """None when the input follows the SIM convention, else the first reason (REJECT_REASONS)."""
        if count != S2.SIM_BEAMS:
            return 'beam_count'
        if not (math.isfinite(angle_min) and abs(angle_min - SIM_ANGLE_MIN) <= ANGLE_TOL):
            return 'angle_min'
        if not (math.isfinite(angle_increment) and abs(angle_increment - SIM_INCREMENT) <= ANGLE_TOL * SIM_INCREMENT):
            return 'angle_increment'
        if frame_id != SIM_FRAME_ID:
            return 'frame_id'
        return None

    def _note_stamp(self, stamp):
        """Gap and order counters of the input stamps (every input scan, also a rejected one)."""
        previous, self.last_stamp_ns = self.last_stamp_ns, stamp
        if previous is None:
            return
        step = (stamp - previous) / 1e9
        if step <= 0.:
            self.counts['stamps_not_increasing'] += 1
        elif step > GAP_FACTOR * PERIOD_S:
            self.counts['stamp_gaps'] += 1
            self.counts['missing_scans'] += max(1, int(round(step / PERIOD_S)) - 1)

    def convert(self, stamp, frame_id, angle_min, angle_increment, ranges):
        """stamp: int ns (or None); ranges: the SIM ranges (any float sequence). Returns (float32 ranges, info) or
        (None, info) for a rejected scan."""
        started = time.perf_counter()
        self.counts['scans_in'] += 1
        values = np.asarray(ranges, dtype=np.float64).reshape(-1)
        problem = 'stamp_invalid' if stamp is None else self.geometry_problem(
            int(values.shape[0]), float(angle_min), float(angle_increment), frame_id)
        if stamp is not None:
            self._note_stamp(int(stamp))
        if problem is not None:
            self.counts['rejected'] += 1
            self.rejected[problem] += 1
            self.last_reject = dict(reason=problem, beams=int(values.shape[0]), angle_min=float(angle_min),
                                    angle_increment=float(angle_increment), frame_id=frame_id)
            return None, dict(rejected=problem)
        out = self.real.convert(values)
        returns = int(np.count_nonzero(out))
        self.counts['scans_out'] += 1
        self.counts['returns_out'] += returns
        self.counts['no_return_out'] += int(out.shape[0]) - returns
        self.last_out_stamp_ns = int(stamp)
        elapsed = (time.perf_counter() - started) * 1000.
        self._timing(self.ms, elapsed)
        return out, dict(rejected=None, returns=returns, ms=elapsed)

    @staticmethod
    def _timing(block, elapsed_ms):
        block['last'] = float(elapsed_ms)
        block['max'] = block['last'] if block['max'] is None else max(block['max'], block['last'])
        block['total'] += block['last']

    def note_callback_ms(self, elapsed_ms):
        """The node's callback time of a published scan (entry to the publish call, wall)."""
        self._timing(self.callback_ms, elapsed_ms)

    def _summary(self, block):
        n = self.counts['scans_out']
        return dict(last=None if block['last'] is None else round(block['last'], 3),
                    mean=None if not n else round(block['total'] / n, 3),
                    max=None if block['max'] is None else round(block['max'], 3))

    def status(self, state, **extra):
        out = dict(schema=SCHEMA, simulated=True, node=NODE_NAME, state=state, updated_at=time.time(),
                   counts=dict(self.counts), rejected_by_reason=dict(self.rejected), last_reject=self.last_reject,
                   conversion_ms=self._summary(self.ms), callback_ms=self._summary(self.callback_ms),
                   last_input_stamp_ns=self.last_stamp_ns, last_output_stamp_ns=self.last_out_stamp_ns,
                   lidar_yaw_offset_deg=self.yaw_offset_deg,
                   output=dict(beams=S2.BEAMS, angle_min=S2.RAW_AMIN, angle_increment=S2.RAW_INC,
                               angle_max=RAW_ANGLE_MAX, frame_id=S2.FRAME_ID, range_min=S2.RANGE_MIN_M,
                               range_max=S2.RANGE_MAX_M, scan_time=SCAN_TIME_S, time_increment=TIME_INCREMENT_S,
                               no_return=0.0, qos='reliable, keep last 10 (sllidar_ros2)'),
                   rule='steering.s2_format.RealFormat (the F-SIM conversion); one input scan -> one output scan')
        out.update(extra)
        return out


def profile_yaw_offset(profile_path=None):
    """(lidar_yaw_offset_deg, profile id, profile sha256) of a vehicle profile (default: the shipped placeholder)."""
    from .vehicle_profile import load_profile
    profile = load_profile(profile_path)
    return (float(profile.value('lidar_yaw_offset_deg')), getattr(profile, 'profile_id', None),
            getattr(profile, 'sha256', None))


def write_status(path, payload):
    """Atomic JSON status file (UTF-8, LF)."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_name(target.name + '.tmp')
    tmp.write_bytes((json.dumps(payload, allow_nan=False, default=str) + '\n').encode('utf-8'))
    os.replace(tmp, target)


def build_parser():
    parser = argparse.ArgumentParser(prog='s2_format_node', description=__doc__.split('\n')[0])
    parser.add_argument('--input', default=INPUT_TOPIC, help='SIM LaserScan in (default %s)' % INPUT_TOPIC)
    parser.add_argument('--output', default=OUTPUT_TOPIC, help='S2 LaserScan out (default %s)' % OUTPUT_TOPIC)
    parser.add_argument('--status-file', default='/runtime/s2_node_status.json')
    parser.add_argument('--profile', default=None, help='vehicle profile YAML (default: the shipped placeholder)')
    parser.add_argument('--yaw-offset-deg', type=float, default=None,
                        help='lidar_yaw_offset_deg (default: the profile value)')
    parser.add_argument('--run-token', default=None, help='run token of the supervisor (written to the status)')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    yaw, profile_id, profile_sha = profile_yaw_offset(args.profile)
    if args.yaw_offset_deg is not None:
        yaw = float(args.yaw_offset_deg)
    converter = S2Converter(yaw)
    import array
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan

    base = dict(input_topic=args.input, output_topic=args.output, run_token=args.run_token, profile_id=profile_id,
                profile_sha256=profile_sha)
    qos_in = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
    qos_out = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

    class S2FormatNode(Node):
        def __init__(self):
            super().__init__(NODE_NAME, parameter_overrides=[Parameter('use_sim_time', value=False)])
            self.pub = self.create_publisher(LaserScan, args.output, qos_out)
            self.create_subscription(LaserScan, args.input, self.scan, qos_in)
            self.last_status = 0.
            self.state = 'waiting'
            self.report()

        def report(self):
            write_status(args.status_file, converter.status(self.state, **base))
            self.last_status = time.monotonic()

        def scan(self, message):
            started = time.perf_counter()
            stamp = stamp_ns(message.header.stamp.sec, message.header.stamp.nanosec)
            ranges = np.frombuffer(message.ranges, dtype=np.float32) if isinstance(message.ranges, array.array) \
                else message.ranges
            out, _ = converter.convert(stamp, message.header.frame_id, message.angle_min, message.angle_increment,
                                       ranges)
            if out is None:
                self.state = 'input_rejected'
            else:
                scan = LaserScan()
                scan.header.stamp = message.header.stamp
                scan.header.frame_id = S2.FRAME_ID
                scan.angle_min, scan.angle_max, scan.angle_increment = S2.RAW_AMIN, RAW_ANGLE_MAX, S2.RAW_INC
                scan.time_increment, scan.scan_time = TIME_INCREMENT_S, SCAN_TIME_S
                scan.range_min, scan.range_max = S2.RANGE_MIN_M, S2.RANGE_MAX_M
                values = array.array('f')
                values.frombytes(out.astype(np.float32).tobytes())
                scan.ranges = values
                self.pub.publish(scan)
                self.state = 'running'
                converter.note_callback_ms((time.perf_counter() - started) * 1000.)
            if time.monotonic() - self.last_status >= STATUS_PERIOD_S:
                self.report()

    rclpy.init(args=[])
    node = S2FormatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:          # ExternalShutdownException on SIGTERM (rclpy signal handler)
        if type(exc).__name__ != 'ExternalShutdownException':
            raise
    finally:
        node.state = 'stopped'
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
