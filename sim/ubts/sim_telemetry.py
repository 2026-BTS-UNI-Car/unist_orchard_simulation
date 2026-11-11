"""Simulation sensor serialization. GPS is synthetic, never a physical fix."""
from collections import deque
from datetime import datetime, timedelta, timezone
import math
import time
import uuid
from scripts.sim_clock import metadata as clock_metadata

GPS_HEIGHT_M = .70  # Existing SIM-only gps_link mount above base_footprint.


def gps_stamp_seconds(stamp):
    """Read the shared ROS stamp, retaining legacy SIM archive compatibility.

    This float is only for the existing approximate map pairing, never the
    published ROS stamp or nanosecond identity comparisons.
    """
    if isinstance(stamp, dict):
        sec, ns = stamp.get('sec'), stamp.get('nanosec')
        if type(sec) is int and 0 <= sec < 2**31 and type(ns) is int and 0 <= ns < 1_000_000_000:
            return sec + ns * 1e-9
    elif type(stamp) in (int, float):
        try:
            if math.isfinite(stamp) and 0 <= stamp < 2**31:
                return float(stamp)
        except OverflowError:
            pass
    raise ValueError('invalid_gps_stamp')


def virtual_gnss_metadata(stamp, gps_config):
    """Declare a SIM-only factor floor and ideal/no-added-noise clock model.

    The .5 m default matches the existing native-source compatibility scale.
    It is neither measured F9P accuracy nor a calibrated covariance. A nonzero
    noise request is unsupported here; no truth-coordinate perturbation is
    silently invented. Original ROS sec/nanosec are the identity clock.
    """
    gps_stamp_seconds(stamp)
    if not isinstance(stamp, dict):
        raise ValueError('exact_virtual_source_stamp_required')
    model = gps_config.get('virtual_receiver', {})
    if not isinstance(model, dict):
        raise ValueError('invalid_virtual_receiver_accuracy_model')
    floor = _finite_number(model.get('horizontal_accuracy_floor_m', .5))
    noise = _finite_number(model.get('horizontal_noise_stddev_m', 0.))
    if floor is None or not 0 < floor <= 50 or noise != 0.:
        raise ValueError('invalid_virtual_receiver_accuracy_model')
    sec, ns = stamp['sec'], stamp['nanosec']
    # ISO formatting is an interoperability encoding of simulation time from
    # the Unix origin, NOT a real UTC/GNSS date. Avoid float timestamp rounding.
    base = datetime(1970, 1, 1, tzinfo=timezone.utc)+timedelta(seconds=sec)
    return dict(gps_time=base.strftime('%Y-%m-%dT%H:%M:%S')+f'.{ns:09d}+00:00',
                time_source='simulation_clock_not_real_UTC',
                receiver_epoch=f'simulation_clock_ns:{sec*1_000_000_000+ns}',
                position_accuracy_floor_m=floor,
                position_noise_stddev_m=noise,
                position_noise_source='ideal_virtual_antenna_position_no_added_noise')


def _finite_number(value):
    if type(value) not in (int, float):
        return None
    try:
        return float(value) if math.isfinite(value) else None
    except OverflowError:
        return None


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def pose_of(msg):
    p = msg.pose.pose
    return {'x': p.position.x, 'y': p.position.y, 'yaw': yaw_of(p.orientation),
            'stamp': stamp_seconds(msg.header.stamp), 'wall_stamp': time.time()}


def serialized_range(value):
    try:
        value = float(value)
    except (ValueError, TypeError):
        return -1
    return round(value, 4) if math.isfinite(value) else None if value == math.inf else -1


class SensorTelemetry:
    def __init__(self, gps_config, save):
        self.config, self.save = gps_config, save
        self.poses = deque(maxlen=240)
        self.imu = {}; self.gps = {}; self.seq = 0
        self.gps_sequence = 0
        self.world = {}
        self.session = uuid.uuid4().hex
        self.last_scan_stamp = -math.inf

    def odom(self, msg):
        pose = pose_of(msg)
        if not all(math.isfinite(pose[key]) for key in ('x', 'y', 'yaw', 'stamp')):
            return
        if self.poses and pose['stamp'] < self.poses[-1]['stamp']:
            self.poses.clear(); self.session = uuid.uuid4().hex
            self.imu = {}; self.gps = {}
            self.last_scan_stamp = -math.inf
        self.poses.append(pose)

    def update_imu(self, msg):
        def xyz(v): return {'x': v.x, 'y': v.y, 'z': v.z}
        self.imu = {'stamp': stamp_seconds(msg.header.stamp),
                    'orientation': {**xyz(msg.orientation), 'w': msg.orientation.w},
                    'angular_velocity': xyz(msg.angular_velocity),
                    'linear_acceleration': xyz(msg.linear_acceleration)}

    def update_gps(self, msg, spawn):
        """Convert Gazebo truth pose only to a synthetic GNSS observation.

        Direct world/local pose fields are diagnostic only; the native sensor
        source allowlist strips them before localization. No real GNSS/RTK
        accuracy or real UTC clock is claimed.
        """
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        lat0 = _finite_number(self.config.get('origin_latitude', 37.0))
        lon0 = _finite_number(self.config.get('origin_longitude', 127.0))
        now = time.time()
        self.gps_sequence += 1
        stamp = {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec}
        self.gps = {'schema': 'orchard.gps.v1', 'simulated': True,
                    'source': 'gazebo_virtual_gnss', 'protocol': 'SIM_GAZEBO',
                    'valid': False, 'quality': 0, 'fix_type': 'no_fix', 'fix_dimension': 0,
                    'sequence': self.gps_sequence, 'received_at': now, 'stamp': stamp,
                    'frame_id': 'gps_link', 'latitude': None, 'longitude': None,
                    'altitude_msl_m': None, 'altitude_ellipsoid_m': None,
                    'speed_mps': None, 'course_deg': None, 'gps_time': None,
                    'h_acc_m': None, 'v_acc_m': None, 'satellites': None,
                    'hdop': None, 'accuracy_source': None,
                    'local_x': None, 'local_y': None, 'local_z': None, 'yaw': None,
                    'origin_latitude': lat0, 'origin_longitude': lon0,
                    'world': self.world,
                    'description': 'Virtual orchard coordinates; not real GNSS or RTK'}
        try:
            self.gps['sim_stamp_s'] = gps_stamp_seconds(stamp)
        except ValueError:
            self.gps.update(stamp=None, reason='invalid_source_stamp')
            return self.gps
        try:
            self.gps.update(virtual_gnss_metadata(stamp, self.config))
        except ValueError as error:
            self.gps['reason'] = str(error)
            return self.gps
        if self.config.get('fix_available', True) is not True:
            self.gps['reason'] = 'simulated_no_fix'
            return self.gps
        # This simple local ENU projection is intentionally unavailable near
        # a pole. It is not a geodetic model of the real receiver/location.
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, *spawn[:2])
        if (len(spawn) < 2 or any(_finite_number(v) is None for v in values)
                or lat0 is None or lon0 is None or not -89 < lat0 < 89 or not -180 <= lon0 <= 180):
            self.gps['reason'] = 'invalid_source_position_or_origin'
            return self.gps
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if not math.isfinite(norm) or abs(norm-1.) > 1e-6:
            self.gps['reason'] = 'invalid_source_orientation'
            return self.gps
        qx, qy, qz, qw = (v/norm for v in (q.x, q.y, q.z, q.w))
        rotation = ((1-2*(qy*qy+qz*qz), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)),
                    (2*(qx*qy+qw*qz), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx)),
                    (2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx*qx+qy*qy)))
        # Project the existing gps_link antenna, not the model origin. No TF
        # or mounting calibration for the real car is implied by this offset.
        x, y = p.x+GPS_HEIGHT_M*rotation[0][2]-spawn[0], p.y+GPS_HEIGHT_M*rotation[1][2]-spawn[1]
        z = p.z+GPS_HEIGHT_M*rotation[2][2]
        latitude = lat0+math.degrees(y/6378137.0)
        longitude = lon0+math.degrees(x/(6378137.0*math.cos(math.radians(lat0))))
        if not all(math.isfinite(v) for v in (x, y, z, latitude, longitude)) or not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
            self.gps['reason'] = 'projected_position_out_of_range'
            return self.gps
        self.gps.update(valid=True, quality=1, fix_type='simulated', fix_dimension=2,
                        h_acc_m=self.gps['position_accuracy_floor_m'],
                        accuracy_source='virtual_receiver_configured_scale_not_measured',
                        latitude=latitude, longitude=longitude, local_x=x, local_y=y, local_z=z,
                        yaw=math.atan2(rotation[1][0], rotation[0][0]))
        # An arbitrary world Z origin is not an ellipsoid/MSL datum. The old
        # origin_altitude_m key is ambiguous and never supplies NavSatFix altitude.
        for datum in ('ellipsoid', 'msl'):
            origin = _finite_number(self.config.get(f'origin_altitude_{datum}_m'))
            altitude = origin+z if origin is not None else None
            if altitude is not None and math.isfinite(altitude):
                self.gps[f'altitude_{datum}_m'] = altitude
        if self.gps['altitude_ellipsoid_m'] is not None:
            self.gps['fix_dimension'] = 3
        linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
        velocity = (linear.x, linear.y, linear.z, angular.x, angular.y)
        if all(_finite_number(v) is not None for v in velocity):
            # Odometry twist is in child_frame_id. Include the antenna lever
            # arm, rotate to virtual ENU, and report course over ground; at rest
            # course is unknown, not the rover's facing direction.
            body = (linear.x+angular.y*GPS_HEIGHT_M, linear.y-angular.x*GPS_HEIGHT_M, linear.z)
            east, north = (sum(a*b for a, b in zip(row, body)) for row in rotation[:2])
            speed = math.hypot(east, north)
            if math.isfinite(speed):
                self.gps['speed_mps'] = speed
                self.gps['course_deg'] = math.degrees(math.atan2(east, north)) % 360 if speed > 1e-6 else None
        return self.gps

    def gps_driver_status(self):
        from dashboard_server.gps import fresh_snapshot
        return fresh_snapshot(self.gps, self.gps.get('received_at'), driver={
            'state': 'simulated', 'simulated': True, 'device': 'Virtual orchard GNSS',
            # None means not applicable. False means a disconnected physical
            # receiver and must invalidate a hardware sample's freshness.
            'serial_open': None, 'serial_applicable': False,
            'transport': 'simulation_topic', 'configuration_writes': False})

    def scan(self, msg, *, receipt_metadata=None):
        stamp = stamp_seconds(msg.header.stamp)
        if not self.poses or stamp-self.last_scan_stamp < .095:
            return
        # Nearest timestamp, never ground-truth pose; reject unaligned scan/odom.
        pose = min(self.poses, key=lambda p: abs(p['stamp']-stamp))
        if abs(pose['stamp']-stamp) > .12:
            return
        self.last_scan_stamp = stamp; self.seq += 1
        self.save('mapping_sample.json', {
            # An isolated output process can wait briefly for matching odom.
            # Keep the original scan receipt age; writing it later must never
            # renew a stale observation's RAW-clock freshness.
            **(clock_metadata() if receipt_metadata is None else receipt_metadata),
            'session_id': self.session, 'seq': self.seq, 'stamp': stamp,
            'updated_at': time.time(), 'pose': pose,
            # Adapter supplies signed wheel distance integrated with IMU
            # heading. GPS truth is a separate observation, never this pose.
            'pose_source': 'wheel_distance_imu_heading',
            'localization': 'wheel_distance_imu_heading_without_loop_closure',
            'scan': {'angle_min': msg.angle_min, 'angle_increment': msg.angle_increment,
                     'range_min': msg.range_min, 'range_max': msg.range_max,
                     'ranges': [serialized_range(r) for r in msg.ranges]},
            'imu': self.imu, 'gps': self.gps})

    def encoderless(self, envelope):
        """Already atomic exact-endpoint pose/scan; never nearest odom or GPS."""
        from encoderless_runtime_contract import validate_envelope
        validate_envelope(envelope)
        key=(envelope['session_id'],envelope['localization_epoch'])
        if getattr(self,'encoderless_epoch',None)!=key:
            self.encoderless_epoch=key
            self.poses.clear();self.imu={};self.gps={}
            self.encoderless_last=None
        identity=(key,envelope['seq'],envelope['localization_valid'])
        if getattr(self,'encoderless_last',None)==identity:return False
        if (getattr(self,'encoderless_last',None) is not None
                and envelope['seq']<self.encoderless_last[1]):return False
        self.encoderless_last=identity
        value=dict(envelope,updated_at=time.time())
        if envelope['localization_valid']:
            self.poses.append(dict(envelope['pose']))
            value.update(stamp=envelope['scan_stamp_ns']/1e9,imu=self.imu,gps=self.gps)
        self.save('mapping_sample.json',value)
        return True
