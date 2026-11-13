"""Sensor contract of the steering G-SIM (design 2.6, 3.7, 4.3 and 8.3 P6a; docs/steering/P6A_HEADLESS_KO.md).

A subclass of the pinned adapter.Adapter (the old SIM sensor contract in its encoderless mode: no /odom, no dynamic
TF). Adapter.__init__ cannot be called: it always builds SimMotor, SimulationControl and AutonomousControl and
advertises /sim/cmd_vel, /scan (the 3200-beam SIM scan) and /rover/status, all of which belong to the old
differential stack (here /rover/status comes from the steering bridge and /scan from the S2 format node of the next
phase). So the subclass replaces the constructor (Node.__init__ only) and reuses the Adapter's sensor methods as they
are: sensor_qos, the quiet create_publisher / create_subscription, observed_callback, touch, prepare-free
publish_imu, and gps_cb (the virtual GPS through sim_telemetry.SensorTelemetry). The pinned file is not changed.

  in   /clock (SIM time, liveness), /sim/imu (Gazebo ideal IMU at the profile rate), /ground_truth/odom (truth:
       virtual GPS and the IMU model's speed), /sim/truth/steering (truth: drive duty and steering PWM for the IMU
       vibration terms), /rover/steering/state (liveness of the steering status lines through the bridge),
       /scan only when an S2 format node is configured (liveness)
  out  /imu/data_raw (p6a_imu_model error model and its game rotation vector, frame imu_link), /gps/data, /gps/fix,
       /gps/driver_status (virtual
       GPS with the obviously virtual origin 0 N 0 E, display only, never a control input; the antenna is the
       profile's gnss_antenna_pose_in_base (0.9 m above base_footprint in v0.1), WF-4 g_gaps / audit G20), /tf_static
       (base_footprint -> base_link -> laser, imu_link, camera_link, gps_link; camera_link -> camera_link_optical)
       The laser frame is the frame of the S2 format /scan (s2_format: frame 'laser', raw beam j at the body angle
       RAW_AMIN + j x RAW_INC + lidar_yaw_offset_deg), so its rotation about z is the profile's lidar_yaw_offset_deg
       (179.08 deg: the travel axis at -179.08 deg in the scan frame; review RG-12). The vehicle stack publishes no
       laser TF (the dashboard sources have no static transform publisher; the executor applies
       lidar_yaw_offset_deg from its own config). The 3200-beam /sim/scan of the pinned cpu_lidar.py also says frame
       'laser' but its beams are in the body frame: it is SIM-internal and must not be drawn with this TF.
  files /runtime/simulation_status.json (the existing schema: updated_at, sim_time, clock_alive, sensors_alive,
       counts, ...; the pinned health.py passes when every sensors_alive value is true), camera_driver_status.json,
       imu_driver_status.json

sensors_alive in this phase: sim_scan (Embree LiDAR /sim/scan from cpu_lidar's status file), imu, camera,
camera_info, gps, steering. /scan is absent until the S2 format node of the next phase: it is not in sensors_alive
(health stays green), the status says so under scan_contract; with an S2 node (--s2-present) 'scan' is added.

Virtual GPS lever arm (audit G20): the pinned sim_telemetry projects an antenna at (0, 0, GPS_HEIGHT_M = 0.70 m) of
the body. The sensor contract gives it an odometry view moved by the difference to the profile antenna
(antenna_shifted_odometry: position p + R (a - (0, 0, h)), body velocity v + w x (a - (0, 0, h))), so the published
virtual GPS is the profile antenna; the pinned file is not changed. gps_link of /tf_static is the profile antenna too.
Display only: the GPS is never a control input.

Pure helpers at module level (static transforms, liveness rules, status payload, the antenna view) import without ROS;
the node class is made by make_node_class() after the ROS imports. Placeholder-based SIM results, not field
validation.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

RESULT_KIND = '자리값 기반 SIM 결과'
NODE_NAME = 'ubts_steer_sensor_contract'
STATUS_SCHEMA = 'ubts.steer_simulation_status.v1'
GPS_ORIGIN = (0.0, 0.0)            # obviously virtual (0 N, 0 E), display only
ALIVE_S = 2.0                      # a sensor is alive with a sample within this wall time (old adapter: 2 s)
CLOCK_ALIVE_S = 1.0
SNAPSHOT_PERIOD_S = .25
RTF_WINDOW_S = 5.0
STATIC_FRAMES = ('base_link', 'laser', 'imu_link', 'camera_link', 'gps_link', 'camera_link_optical')


def quaternion_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.), math.sin(roll / 2.)
    cp, sp = math.cos(pitch / 2.), math.sin(pitch / 2.)
    cy, sy = math.cos(yaw / 2.), math.sin(yaw / 2.)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def gnss_antenna(profile):
    """(x, y, z) of the profile's gnss_antenna_pose_in_base (base_footprint frame, z above the ground)."""
    pose = profile.value('gnss_antenna_pose_in_base')
    return float(pose['x']), float(pose['y']), float(pose['z'])


def antenna_shifted_odometry(msg, antenna, telemetry_height):
    """An odometry view for the pinned sim_telemetry.update_gps, which projects an antenna at (0, 0, telemetry_height)
    of the body: the pose and the twist are moved by d = antenna - (0, 0, telemetry_height) so that its projection gives
    the profile antenna. Position p + R d (world), body linear velocity v + w x d (the twist is in the child frame).
    Header, orientation and angular velocity are the message's own."""
    from types import SimpleNamespace as NS
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    d = (antenna[0], antenna[1], antenna[2] - float(telemetry_height))
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) or 1.
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    rot = ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
           (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
           (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))
    world = [sum(rot[i][j] * d[j] for j in range(3)) for i in range(3)]
    lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
    cross = (ang.y * d[2] - ang.z * d[1], ang.z * d[0] - ang.x * d[2], ang.x * d[1] - ang.y * d[0])
    return NS(header=msg.header,
              pose=NS(pose=NS(position=NS(x=p.x + world[0], y=p.y + world[1], z=p.z + world[2]), orientation=q)),
              twist=NS(twist=NS(linear=NS(x=lin.x + cross[0], y=lin.y + cross[1], z=lin.z + cross[2]),
                                angular=ang)))


def static_transforms(profile, rover_config, gps_height_m=None):
    """(parent, child, (x, y, z), (qx, qy, qz, qw)) rows of /tf_static.

    base_link sits at the generated link height (rover.yaml body_center_height_m; gz_model keeps the link poses).
    laser and camera_link: the profile values (gz_model refuses a profile that differs from the generated links).
    imu_link: the profile IMU position and mount rotation (yaw_sign -1 = mounted upside down: roll + pi), the pose
    gz_model gives the IMU sensor. gps_link: the profile's gnss_antenna_pose_in_base (display only; audit G20). The
    old gps_height_m argument (the pinned sim_telemetry 0.70 m) is ignored and kept for callers only."""
    v = profile.value
    z = float(rover_config['vehicle']['body_center_height_m'])
    imu = v('imu_pose_in_base')
    rpy = v('imu_mount_rpy_deg')
    if rpy['yaw_sign'] not in (1, -1):
        raise ValueError('imu_yaw_sign_must_be_plus_or_minus_one')
    roll = math.radians(rpy['roll']) + (math.pi if rpy['yaw_sign'] == -1 else 0.)
    cam = v('camera_pose_in_base')
    lidar_yaw = math.radians(float(v('lidar_yaw_offset_deg')))
    return [
        ('base_footprint', 'base_link', (0., 0., z), (0., 0., 0., 1.)),
        ('base_link', 'laser', (v('lidar_x_in_base_m'), v('lidar_y_in_base_m'), v('lidar_z_m') - z),
         quaternion_rpy(0., 0., lidar_yaw)),
        ('base_link', 'imu_link', (imu['x'], imu['y'], imu['z'] - z),
         quaternion_rpy(roll, math.radians(rpy['pitch']), math.radians(rpy['yaw']))),
        ('base_link', 'camera_link', (cam['x'], cam['y'], cam['z'] - z), (0., 0., 0., 1.)),
        ('base_link', 'gps_link', (gnss_antenna(profile)[0], gnss_antenna(profile)[1], gnss_antenna(profile)[2] - z),
         (0., 0., 0., 1.)),
        ('camera_link', 'camera_link_optical', (0., 0., 0.), (-.5, .5, -.5, .5)),
    ]


def lidar_alive(status, now_wall, max_age_s=ALIVE_S):
    """Embree LiDAR liveness from cpu_lidar.py's status file (running, fresh, a stamp)."""
    if not isinstance(status, dict):
        return False
    age = now_wall - float(status.get('updated_at') or 0.)
    return status.get('state') == 'running' and 0. <= age <= max_age_s and status.get('last_stamp_ns') is not None


def sensors_alive(now_mono, now_wall, last, clock_seen, camera_health, lidar_status, s2_present, camera_present=True):
    """The sensors_alive map (health.py requires all values true). last: {key: monotonic receipt}. A scene without
    the camera (--no-camera) leaves camera and camera_info out."""
    clock_alive = clock_seen is not None and 0. <= now_mono - clock_seen < CLOCK_ALIVE_S

    def fresh(key):
        return clock_alive and now_mono - last.get(key, -1e9) < ALIVE_S
    alive = dict(sim_scan=clock_alive and lidar_alive(lidar_status, now_wall), imu=fresh('imu'))
    if camera_present:
        alive.update(camera=clock_alive and bool(camera_health['camera']['alive']),
                     camera_info=clock_alive and bool(camera_health['camera_info']['alive']))
    alive.update(gps=fresh('gps'), steering=fresh('steering'))
    if s2_present:
        alive['scan'] = fresh('scan')
    return clock_alive, alive


def scan_contract(s2_present, clip_present):
    return {'/scan': 'S2 format node' if s2_present else
            'absent: the S2 format node (sim/ubts/steering/s2_format.py) is the next phase',
            '/sim/scan': 'cpu_lidar.py (Embree, 3200 SIM beams, pose from /ground_truth/odom); executor forbidden',
            'clip_node': 'configured' if clip_present else 'none (field_world only)',
            's2_node': 'configured' if s2_present else 'none (next phase)'}


def rtf_estimate(samples):
    """Delta SIM / delta wall over the (wall, sim) samples of the last RTF_WINDOW_S, or None."""
    if len(samples) < 2:
        return None
    (w0, s0), (w1, s1) = samples[0], samples[-1]
    return (s1 - s0) / (w1 - w0) if w1 - w0 > .5 else None


def make_node_class():
    """The SteerSensorContract class (imports ROS and the pinned adapter module)."""
    import collections
    from .p6a_imu_model import GameRotationVector
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, LaserScan, NavSatFix
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster
    import adapter                                      # pinned: reused, not changed
    from callback_timing import CallbackTiming
    from scripts.sim_camera_health import CameraHealthReader
    from sim_telemetry import GPS_HEIGHT_M, SensorTelemetry

    class SteerSensorContract(adapter.Adapter):
        """Steering sensor contract (module docstring). The Adapter constructor is replaced (Node only)."""

        encoderless_mode = True
        native_display_mode = False

        def __init__(self, profile, rover_config, world_metadata, *, scene, imu_model, runtime_dir='/runtime',
                     s2_present=False, clip_present=False, camera_present=True, run_token=None):
            # No parameter services, type description service or /rosout publisher (G-SIM stage 7): nothing uses
            # them and rclpy adds every service of the node to the wait set of each callback. Topics, QoS and
            # callbacks are unchanged.
            Node.__init__(self, NODE_NAME, parameter_overrides=[Parameter('use_sim_time', value=False),
                                                                Parameter('start_type_description_service', value=False)],
                          start_parameter_services=False, enable_rosout=False)
            self.profile, self.config = profile, rover_config
            self.scene = scene
            self.runtime_dir = Path(runtime_dir)
            self.s2_present, self.clip_present = bool(s2_present), bool(clip_present)
            self.camera_present = bool(camera_present)
            self.run_token = run_token
            self.imu_model = imu_model
            self.orientation = GameRotationVector()
            self.lock = threading.RLock()
            self.sim_time = 0.
            self.clock_seen = None
            self.last, self.count = {}, {}
            self.callback_timing = CallbackTiming()
            self.callback_generation = 0
            self.world_metadata = world_metadata
            gps_config = dict(rover_config['gps'], origin_latitude=GPS_ORIGIN[0], origin_longitude=GPS_ORIGIN[1])
            self.sensor_telemetry = SensorTelemetry(gps_config, self._save)
            self.sensor_telemetry.world = {'preset': scene, 'note': 'steering G-SIM virtual GPS, display only'}
            self.gps_last_stamp = -999.
            self.camera_health = CameraHealthReader()
            self.truth = dict(duty_pct=0., steer_pwm=0., speed=0., stamp=None)
            self.rtf_samples = collections.deque()
            self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
            self.gps_pub = self.create_publisher(String, '/gps/data', qos_profile_sensor_data)
            self.gps_fix_pub = self.create_publisher(NavSatFix, '/gps/fix', qos_profile_sensor_data)
            self.gps_driver_pub = self.create_publisher(String, '/gps/driver_status', 10)
            self.static = StaticTransformBroadcaster(self)
            self.antenna = gnss_antenna(profile)
            self.static_rows = static_transforms(profile, rover_config)
            self._send_static(TransformStamped)
            best = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            inputs = [(Clock, '/clock', self.clock_cb, self.sensor_qos('/clock')),
                      (Imu, '/sim/imu', self.imu_cb, self.sensor_qos('/sim/imu')),
                      (Odometry, '/ground_truth/odom', self.odom_truth_cb, qos_profile_sensor_data),
                      (String, '/sim/truth/steering', self.truth_steering_cb, best),
                      (String, '/rover/steering/state', self.steering_state_cb, 10)]
            if self.s2_present:
                inputs.append((LaserScan, '/scan', self.scan_cb, best))
            for typ, topic, fn, qos in inputs:
                self.create_subscription(typ, topic, self.observed_callback(topic, fn), qos)

        # -- helpers
        def _save(self, name, payload):
            path = self.runtime_dir / name
            tmp = path.with_name(path.name + '.tmp')
            tmp.write_bytes(json.dumps(payload, allow_nan=False).encode('utf-8'))
            os.replace(tmp, path)

        def _send_static(self, transform_type):
            transforms = []
            for parent, child, xyz, q in self.static_rows:
                t = transform_type()
                t.header.frame_id, t.child_frame_id = parent, child
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
                (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
                 t.transform.rotation.w) = map(float, q)
                transforms.append(t)
            self.static.sendTransform(transforms)

        # -- callbacks (gps_cb, touch, publish_imu, observed_callback: Adapter's)
        def clock_cb(self, msg):
            t = msg.clock.sec + msg.clock.nanosec * 1e-9
            with self.lock:
                if t < self.sim_time:
                    self.count['clock_rewinds'] = self.count.get('clock_rewinds', 0) + 1
                    self.rtf_samples.clear()
                now = time.monotonic()
                if t != self.sim_time:
                    self.clock_seen = now
                self.sim_time = t
                self.rtf_samples.append((now, t))
                while self.rtf_samples and now - self.rtf_samples[0][0] > RTF_WINDOW_S:
                    self.rtf_samples.popleft()

        def odom_truth_cb(self, msg):
            value = float(msg.twist.twist.linear.x)
            if math.isfinite(value):
                self.truth['speed'] = value
            self.gps_cb(antenna_shifted_odometry(msg, self.antenna, GPS_HEIGHT_M))

        def truth_steering_cb(self, msg):
            try:
                data = json.loads(msg.data)
                duty = .5 * (abs(float(data['drive_duty_left'])) + abs(float(data['drive_duty_right']))) * 100.
                self.truth.update(duty_pct=duty if data.get('drive_en') else 0.,
                                  steer_pwm=float(data['applied_steer_pwm']), stamp=data.get('sim_time_s'))
            except (ValueError, KeyError, TypeError):
                self.count['truth_steering_bad'] = self.count.get('truth_steering_bad', 0) + 1

        def steering_state_cb(self, msg):
            try:
                data = json.loads(msg.data)
            except ValueError:
                return
            if data.get('stale') is False:
                self.touch('steering')

        def scan_cb(self, msg):
            self.touch('scan')

        def imu_cb(self, msg):
            g, a = msg.angular_velocity, msg.linear_acceleration
            truth = self.truth
            gyro, accel, _ = self.imu_model.apply((g.x, g.y, g.z), (a.x, a.y, a.z), truth['duty_pct'],
                                                  truth['steer_pwm'], truth['speed'])
            g.x, g.y, g.z = gyro
            a.x, a.y, a.z = accel
            msg.header.frame_id = 'imu_link'
            # BNO085-like game rotation vector from the modelled gyro and accel (p6a_imu_model.GameRotationVector;
            # gsim-final3: the dashboard's live map needs an orientation; Gazebo's own orientation is truth, unused)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ox, oy, oz, ow = self.orientation.update(gyro, accel, stamp)
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = ox, oy, oz, ow
            msg.orientation_covariance = self.orientation.covariance()
            msg.angular_velocity_covariance = [0.] * 9
            msg.linear_acceleration_covariance = [0.] * 9
            self.publish_imu(msg)

        # -- status
        def snapshot(self):
            camera_health = self.camera_health.read(os.environ.get('CAMERA_BRIDGE_STATUS_FILE',
                                                                   str(self.runtime_dir / 'camera_bridge_status.json')))
            try:
                lidar = json.loads((self.runtime_dir / 'cpu_lidar_status.json').read_bytes().decode('utf-8'))
            except (OSError, ValueError):
                lidar = None
            with self.lock:
                now, wall = time.monotonic(), time.time()
                clock_alive, alive = sensors_alive(now, wall, self.last, self.clock_seen, camera_health, lidar,
                                                   self.s2_present, self.camera_present)
                for key in ('camera', 'camera_info'):
                    self.count[key] = camera_health[key]['count']
                rtf = rtf_estimate(list(self.rtf_samples))
                status = dict(
                    schema=STATUS_SCHEMA, updated_at=wall, sim_time=self.sim_time, clock_alive=clock_alive,
                    sensors_alive=alive, counts=dict(self.count), rtf_5s=rtf, scene=self.scene,
                    sim_run=self.run_token,
                    scan_contract=scan_contract(self.s2_present, self.clip_present),
                    control=dict(stack='steering', path='steer_rover_bridge (pty) -> gz_esp_emulator -> /sim/act/*',
                                 control_socket='/run/ubts-steer/control.sock',
                                 details=str(self.runtime_dir / 'steering_status.json'), sim_motor=False,
                                 old_control_socket=False),
                    odometry=dict(published=False, dynamic_tf=False,
                                  note='encoderless sensor contract: no /odom, no dynamic /tf; static TF only'),
                    imu_model=self.imu_model.describe(), imu_orientation=self.orientation.describe(),
                    gps=dict(virtual=True, origin_deg=list(GPS_ORIGIN), display_only=True,
                             lever_arm_m=list(self.antenna), lever_arm_source='profile gnss_antenna_pose_in_base',
                             telemetry_height_m=GPS_HEIGHT_M),
                    lidar=None if lidar is None else {k: lidar.get(k) for k in (
                        'state', 'updated_at', 'last_stamp_ns', 'last_raycast_ms', 'finite_returns', 'errors',
                        'last_error')},
                    camera_health=camera_health, callback_timing=self.callback_timing.snapshot(),
                    result_kind=RESULT_KIND, field_validated=False)
                camera = self.config['camera']
                camera_alive = alive.get('camera', False)
                payloads = {'simulation_status.json': status,
                            'camera_driver_status.json': dict(
                                updated_at=wall, state='streaming' if camera_alive else 'waiting',
                                running=camera_alive, simulated=True, width=camera['width'],
                                height=camera['height'], fps=camera['hz'], pixel_format='RGB8', rotate_180=False),
                            'imu_driver_status.json': dict(
                                updated_at=wall, state='streaming' if alive['imu'] else 'waiting',
                                sensor='BNO085 simulation (p6a_imu_model)', transport='gazebo',
                                orientation_convention='not_simulated', rep145_ready=False, simulated=True)}
            for name, payload in payloads.items():
                self._save(name, payload)
            return status

    return SteerSensorContract


def build_parser():
    p = argparse.ArgumentParser(prog='sensor_contract_steer', description=__doc__.splitlines()[0])
    p.add_argument('--scene-dir', required=True, help='gz_assemble scene folder (world_metadata.json)')
    p.add_argument('--profile', default=None, help='vehicle profile YAML (default: the shipped default profile)')
    p.add_argument('--seed', type=int, default=0, help='seed of the IMU error model draws')
    p.add_argument('--runtime-dir', default='/runtime')
    p.add_argument('--s2-present', action='store_true', help='an S2 format node publishes /scan (next phase)')
    p.add_argument('--clip-present', action='store_true', help='a clip node runs (field_world)')
    p.add_argument('--no-camera', action='store_true', help='the scene has no camera (camera not in sensors_alive)')
    p.add_argument('--run-token', default=None, help='the sim supervisor run token (simulation_status sim_run)')
    p.add_argument('--max-runtime-s', type=float, default=None)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    import signal
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from camera_profile import load_rover_config
    from .p6a_imu_model import ImuErrorModel
    from .vehicle_profile import load_profile
    sim_root = Path(__file__).resolve().parents[1]
    rover_config = load_rover_config(sim_root / 'rover.yaml')
    scene_dir = Path(args.scene_dir)
    manifest = json.loads((scene_dir / 'scene_manifest.json').read_bytes().decode('utf-8'))
    metadata = json.loads((scene_dir / 'world_metadata.json').read_bytes().decode('utf-8'))
    profile = load_profile(args.profile)
    imu_model = ImuErrorModel(profile, seed=args.seed)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = make_node_class()(profile, rover_config, metadata, scene=manifest.get('scene'), imu_model=imu_model,
                             runtime_dir=args.runtime_dir, s2_present=args.s2_present, clip_present=args.clip_present,
                             camera_present=not args.no_camera, run_token=args.run_token)

    def status_loop():
        while not stop.wait(SNAPSHOT_PERIOD_S):
            try:
                node.snapshot()
            except Exception as exc:
                print('sensor_contract_steer: status error: %s: %s' % (type(exc).__name__, exc), file=sys.stderr,
                      flush=True)
    threading.Thread(target=status_loop, name='ubts-steer-contract-status', daemon=True).start()
    if args.max_runtime_s:
        threading.Timer(args.max_runtime_s, stop.set).start()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stop.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
