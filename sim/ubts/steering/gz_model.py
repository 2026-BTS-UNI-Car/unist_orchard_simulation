"""Steered Gazebo model from the generated rover SDF (design 3.2, 3.5-3.7, 4.3 and 8.3 P5).

steer_model_element() takes the <model name="ubts_rover"> element that
generate_world.generate() (or assemble_world.assemble()) produced and turns the
four-wheel skid model into a front-wheel steered model. The pinned generators are
only called; their output is transformed (the pattern of
verify_wheel_friction.build_fixture()).

What the transform does (values from one vehicle profile, steering/vehicle_profile.py):

  kept       model name ubts_rover, the model origin (base_footprint, ground level),
             the links base_link, laser, imu_link, camera_link (names and poses), the
             four wheel joint names *_wheel_joint, OdometryPublisher (truth, gz topic
             /sim/ground_truth -> ROS /ground_truth/odom) and JointStatePublisher
             (truth, gz topic /sim/truth/joint_states)
  removed    DiffDrive (its wheel odometry is an ideal encoder)
  steering   linkage: per front wheel a <side>_front_steering_link at the kingpin and a
             <side>_front_steering_joint (revolute, axis z tilted by caster and kingpin
             inclination, limits = the per-wheel mechanical limits of the profile);
             the front wheel joints are re-parented to the steering links.
             axle_pivot: one front_axle_steering_link at the front axle centre and one
             front_axle_steering_joint (limits = the smaller wheel limit per direction).
             Steering joint names never end with _wheel_joint.
  steering control  one JointPositionController per steering joint in velocity-command
             mode (use_velocity_commands true, cmd_max = gz_steer_joint_cmd_max_rad_s).
             gz-sim 8.15 then commands the joint velocity -error/dt clipped to cmd_max
             each step, so a target that moves less than cmd_max*dt per step is reached at
             the end of the step; the PID gains are not used in this mode. The steering
             actuator dynamics (rate limits, lag, deadband, backlash, end stops) belong to
             the shared actuator model (backend_api), not to Gazebo (design 3.6).
  drive      force_dc (default, gz_drive_mode): one JointController per driven wheel in
             force mode, the DC motor line of design 3.6 from drive.dc_motor_line():
             p_gain = tau_stall / omega0, cmd_max = tau_stall, cmd_min = -tau_stall,
             i and d gains 0; the velocity target is duty * omega0; the deadband comes
             from the driven joint's Coulomb friction d0 * tau_stall_nominal.
             kinematic_velocity (geometry check A1/A12): a JointController per wheel in
             velocity mode; the caller sends each wheel its kinematic speed
             (kinematic_wheel_speeds()).
             ackermann_check (steer_control='ackermann'): the standard AckermannSteering
             system drives the rear wheels and the two steering joints from a Twist;
             comparison only (design 8.3 P5, risk R5); its odometry is wheel odometry.
  wheels     link poses, radii, widths, masses and inertias (solid cylinder; spin axis
             J = m r^2 / 2, the profile's wheel_inertia_kgm2) from the profile; contact
             friction mu (first direction = lateral, fdir1 0 0 1 = axle in the collision
             frame) = wheel_mu_lat, mu2 = wheel_mu_long. Collision shape (option wheel_collision):
             'sphere' of the wheel radius (default since 2026-11-09, DECISIONS_KO R13 decision 1; the visual
             stays a cylinder, the collision pose, friction and fdir1 are kept) or 'cylinder' (the generated
             shape, kept as an option). A flat cylinder touches the ground at its two edge points; a turning
             wheel then drags (DART Coulomb friction has no slip stiffness) and rolls about an edge: in the P5
             review probe the sphere removed most of the turning speed loss and the kinematic-mode speed bias
             (GZ_MODEL_KO 8). A SIM model choice, not a measured tyre
  body       base_link inertial: mass_kg minus the masses of the other non-wheel links (steering links, front
             axle pivot link, suspension links, the generated sensor links), so the vehicle total is
             mass_kg + 4 wheel masses (drive.vehicle_mass_kg, the F-SIM mass); base_link centre of gravity placed
             so that all non-wheel links together have the profile centre of gravity (cog_x_in_base_m, cog_z_m);
             box inertia of gz_body_box_m with the base_link mass; the body visual and collision: the gz_body_box_m envelope
             with wheel wells (body_boxes(): an upper deck above the wheel tops and lower segments that
             keep body_wheel_clearance_m from every wheel over the steering range). The generated box
             overlapped the wheel tops (5 cm) and a steered front wheel swept 7 cm into it; links of one
             model do not collide (self_collide false), so the overlap was visual and geometric only.
             wheel_body_clearance() measures the gap (tests, steering.gz_direct_drive check)
  front axle front_axle_pivot (default on): a passive roll pivot (front_axle_pivot_link, front_axle_roll_joint,
             axis x, limits +-front_pivot_roll_limit_deg, damping only, no spring) between base_link and the
             steering joints, so the wheels share the load on twisted ground (three-point support). The rigid
             generated chassis lifted a wheel on the original orchard terrain (W1) and stalled there.
             Experimental option suspension (default off): spring-dampers per wheel, see _add_suspension()
  IMU        sensor update_rate = imu_rate_hz; sensor pose inside imu_link = profile IMU
             position minus the link position, rotation = imu_mount_rpy_deg (yaw_sign -1:
             roll + pi, the sensor mounted upside down as in steered_plant)
  sensors    the laser and camera_link positions stay those of the pinned rover.yaml (the Embree LiDAR
             cpu_lidar.py reads its mount from rover.yaml). A profile whose LiDAR or camera position differs is
             refused (SteerModelError sensor_mount_not_applied) unless allow_sensor_mount_mismatch=True; then the
             manifest result_flags carry sensor_mount_not_applied:<link> (and lidar_imu_xy_relation_broken when
             the IMU moved but the laser did not)

Topics (design 4.3): steering targets {prefix}/steer_left_cmd, {prefix}/steer_right_cmd
(axle_pivot {prefix}/steer_axle_cmd), drive {prefix}/<wheel>_cmd with prefix /sim/act
(gz.msgs.Double: steering wheel angle in rad, drive joint velocity target in rad/s).

What Gazebo cannot represent here (F-SIM only, design 3.6): free (non-self-locking)
steering with zero output, coast (EN off) of the drive, self-aligning torque and tyre
slip stiffness. G-SIM results assume a self-locking steering.

Constants below that are not profile values are SIM placeholders (provenance SIM_NUMERIC
or CONFIG); they are listed in PLACEHOLDERS and in the manifest. Results built on the
placeholder profile are placeholder-based SIM results, not field validation.

Pure Python (xml.etree, math). CLI: python -m steering.gz_model --help (sim/ubts on sys.path).
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as E

from . import drive as drive_model
from . import geometry as geo
from .vehicle_profile import drive_wheel_radius, load_profile

SCHEMA = 'ubts.steer_gz_model.v1'
MODEL_NAME = 'ubts_rover'
KEPT_LINKS = ('base_link', 'laser', 'imu_link', 'camera_link')
WHEELS = ('left_front_wheel', 'left_rear_wheel', 'right_front_wheel', 'right_rear_wheel')
FRONT_WHEELS = ('left_front_wheel', 'right_front_wheel')
REAR_WHEELS = ('left_rear_wheel', 'right_rear_wheel')
WHEEL_JOINTS = tuple(name + '_joint' for name in WHEELS)
STEER_LINKS = {'left': 'left_front_steering_link', 'right': 'right_front_steering_link'}
STEER_JOINTS = {'left': 'left_front_steering_joint', 'right': 'right_front_steering_joint'}
AXLE_STEER_LINK = 'front_axle_steering_link'
AXLE_STEER_JOINT = 'front_axle_steering_joint'
DRIVE_MODES = ('force_dc', 'kinematic_velocity')
STEER_CONTROLS = ('position', 'ackermann')
WHEEL_COLLISION_SHAPES = ('cylinder', 'sphere')     # 'sphere': one ground contact under the wheel centre (GZ_MODEL_KO 8)
# Default wheel collision shape: the sphere (DECISIONS_KO R13 decision 1, 2026-11-09; measured again in phase 3,
# GZ_MODEL_KO 12). The generated cylinder stays an option (wheel_collision='cylinder'); the visual is a cylinder in both.
DEFAULT_WHEEL_COLLISION = 'sphere'
WHEEL_COLLISION_DECISION = 'DECISIONS_KO R13 decision 1 (sphere default, cylinder kept as an option)'
# Non-wheel links whose mass the vehicle total includes (base_link gets mass_kg minus these; see _body_mass_split).
SENSOR_LINKS = ('laser', 'imu_link', 'camera_link')
DRIVEN_WHEELS = {'4wd_lr_pairs': WHEELS, 'rear2': REAR_WHEELS, 'front2': FRONT_WHEELS}
DEFAULT_TOPIC_PREFIX = '/sim/act'
TRUTH_ODOM_TOPIC = '/sim/ground_truth'               # bridged to ROS /ground_truth/odom (cpu_lidar.py reads it)
TRUTH_JOINT_STATE_TOPIC = '/sim/truth/joint_states'
ACKERMANN_ODOM_TOPIC = '/sim/truth/ackermann_wheel_odom'
ACKERMANN_TF_TOPIC = '/sim/truth/ackermann_wheel_tf'
REMOVED_PLUGINS = ('DiffDrive',)
KEPT_PLUGINS = ('OdometryPublisher', 'JointStatePublisher')

# SIM placeholders that are not profile parameters (name: (value, provenance, note)).
PLACEHOLDERS = {
    'steer_link_radius_m': (0.03, 'SIM_NUMERIC', 'steering link shape (kingpin post) for visual and inertia; no collision'),
    'steer_link_length_m': (0.10, 'SIM_NUMERIC', 'steering link shape length (vertical)'),
    'steer_link_inertia_min_kgm2': (1.0e-3, 'SIM_NUMERIC',
                                    'lower bound of each principal steering link inertia (design 3.6: not too small)'),
    'steer_joint_effort_limit_nm': (1.0e6, 'SIM_NUMERIC',
                                    'steering joint effort limit; gz-physics 7 (DART) velocity control ignores the '
                                    'position limits of a joint without an effort limit and advises about 1e6'),
    'steer_position_p_gain': (1.0, 'SIM_NUMERIC', 'JointPositionController p_gain; not used in velocity-command mode'),
    'steer_position_i_gain': (0.0, 'SIM_NUMERIC', 'JointPositionController i_gain; not used in velocity-command mode'),
    'steer_position_d_gain': (0.0, 'SIM_NUMERIC', 'JointPositionController d_gain; not used in velocity-command mode'),
    'ackermann_steer_p_gain': (1.0, 'CONFIG', 'AckermannSteering steer_p_gain (plugin default); cross-check scene only'),
    'odometry_publish_hz': (30.0, 'CONFIG', 'OdometryPublisher rate as generated (gz 8.15 truncates the step: ~24 Hz)'),
    'joint_state_publish_hz': (30.0, 'CONFIG', 'JointStatePublisher rate as generated (~24 Hz at 120 Hz physics)'),
    'free_wheel_joint_friction_nm': (0.0, 'SIM_ASSUMED', 'Coulomb friction of a wheel without a drive controller'),
    'body_wheel_clearance_m': (0.01, 'SIM_ASSUMED',
                               'gap kept between the body boxes and every wheel over the whole steering range '
                               '(wheel wells; the generated body box overlapped the wheel tops)'),
    'suspension_static_deflection_m': (0.015, 'SIM_ASSUMED',
                                       'wheel suspension: mean static spring deflection under the sprung weight '
                                       '(sets the spring rate); the spring is preloaded so the rest height is the '
                                       'generated one'),
    'suspension_damping_ratio': (0.5, 'SIM_ASSUMED', 'wheel suspension damping ratio of the sprung corner mass'),
    'suspension_travel_m': (0.04, 'SIM_ASSUMED', 'wheel suspension travel each way from the rest position'),
    'suspension_link_mass_kg': (0.2, 'SIM_NUMERIC', 'mass of the link between the spring and the wheel'),
    'front_pivot_roll_limit_deg': (8.0, 'SIM_ASSUMED', 'front axle roll pivot travel each way (front_axle_pivot, default '
                                                       'on); stands in for unknown chassis compliance'),
    'front_pivot_damping_nms_rad': (2.0, 'SIM_NUMERIC', 'front axle roll pivot damping'),
    'front_pivot_link_mass_kg': (0.5, 'SIM_NUMERIC', 'mass of the front axle pivot link'),
}
# Profile names read by the transform (recorded in the manifest with their provenance).
PROFILE_NAMES = (
    'wheelbase_m', 'track_front_m', 'track_rear_m', 'kingpin_width_m', 'wheel_radius_front_m', 'wheel_radius_rear_m',
    'wheel_width_m', 'rear_axle_x_in_base_m', 'steer_type', 'caster_deg', 'kingpin_inclination_deg',
    'steer_mech_limit_deg', 'steer_ackermann_ratio', 'mass_kg', 'cog_x_in_base_m', 'cog_z_m', 'wheel_mass_kg',
    'wheel_inertia_kgm2', 'steer_link_mass_kg', 'gz_body_box_m', 'gz_physics_hz', 'gz_drive_mode',
    'gz_steer_joint_cmd_max_rad_s', 'wheel_mu_lat', 'wheel_mu_long', 'drive_wheels', 'drive_lr_mode',
    'drive_tau_stall_nm', 'drive_omega0_rad_s', 'drive_deadband_frac', 'battery_voltage_scale', 'forward_curve',
    'imu_rate_hz', 'imu_mount_rpy_deg', 'imu_pose_in_base', 'lidar_x_in_base_m', 'lidar_y_in_base_m', 'lidar_z_m',
    'camera_pose_in_base', 'steer_self_locking')
POSE_CHECK_TOLERANCE_M = 1e-9


class SteerModelError(ValueError):
    """The input SDF is not the expected generated rover, or an option is invalid."""


# --- small XML helpers -----------------------------------------------------------------------------------------
def _num(value):
    """Shortest round-trip text of a float (repr), integers without a fraction."""
    value = float(value)
    if not math.isfinite(value):
        raise SteerModelError('non_finite_value')
    if value == int(value) and abs(value) < 1e15:
        return str(int(value)) if value != 0. else '0'
    return repr(value)


def _sub(parent, tag, text=None, **attrs):
    element = E.SubElement(parent, tag, attrs)
    if text is not None:
        element.text = text if isinstance(text, str) else _num(text)
    return element


def _set(parent, tag, text):
    element = parent.find(tag)
    if element is None:
        element = E.SubElement(parent, tag)
    element.text = text if isinstance(text, str) else _num(text)
    return element


def _pose_text(xyz, rpy=(0., 0., 0.)):
    return ' '.join(_num(v) for v in tuple(xyz) + tuple(rpy))


def _parse_pose(element):
    text = element.findtext('pose') if element is not None else None
    if text is None:
        return (0., 0., 0., 0., 0., 0.)
    values = [float(v) for v in text.split()]
    if len(values) != 6:
        raise SteerModelError('pose_needs_6_values')
    return tuple(values)


def _plugin(parent, short, **params):
    """A gz-sim system plugin element (same file/name convention as generate_world.plugin)."""
    filename = ''.join(('-' + c.lower() if c.isupper() else c) for c in short).lstrip('-')
    element = _sub(parent, 'plugin', filename='gz-sim-%s-system' % filename, name='gz::sim::systems::%s' % short)
    for key, value in params.items():
        if isinstance(value, bool):
            _sub(element, key, 'true' if value else 'false')
        elif isinstance(value, (list, tuple)):
            for item in value:
                _sub(element, key, item if isinstance(item, str) else _num(item))
        else:
            _sub(element, key, value if isinstance(value, str) else _num(value))
    return element


def _plugin_kind(plugin):
    return (plugin.get('name') or '').rsplit('::', 1)[-1]


def _cylinder_inertia(mass, radius, length, axis):
    """Principal inertias (ixx, iyy, izz) of a solid cylinder whose axis is x, y or z."""
    along = .5 * mass * radius * radius
    across = mass * (3. * radius * radius + length * length) / 12.
    return tuple(along if name == axis else across for name in 'xyz')


def _set_inertial(link, mass, diagonal, pose=None):
    inertial = link.find('inertial')
    if inertial is None:
        inertial = E.SubElement(link, 'inertial')
    for old in list(inertial):
        inertial.remove(old)
    if pose is not None:
        pose = tuple(pose)
        _sub(inertial, 'pose', _pose_text(pose[:3], pose[3:6] if len(pose) == 6 else (0., 0., 0.)))
    _sub(inertial, 'mass', mass)
    inertia = _sub(inertial, 'inertia')
    for key, value in zip(('ixx', 'iyy', 'izz'), diagonal):
        _sub(inertia, key, value)
    for key in ('ixy', 'ixz', 'iyz'):
        _sub(inertia, key, '0')


def _link(model, name):
    link = model.find("link[@name='%s']" % name)
    if link is None:
        raise SteerModelError('missing_link:' + name)
    return link


def _joint(model, name):
    joint = model.find("joint[@name='%s']" % name)
    if joint is None:
        raise SteerModelError('missing_joint:' + name)
    return joint


# --- geometry helpers (pure) --------------------------------------------------------------------------------------
def steering_axis(side, caster_rad, kingpin_inclination_rad):
    """Unit steering axis in the body frame (upward). Caster tilts the top rearward (-x); kingpin inclination
    tilts the top toward the vehicle centre (left wheel -y, right wheel +y). 0, 0 gives (0, 0, 1)."""
    inward = -1. if side == 'left' else 1.
    x, y, z = -math.tan(caster_rad), inward * math.tan(kingpin_inclination_rad), 1.
    norm = math.sqrt(x * x + y * y + z * z)
    return (x / norm, y / norm, z / norm)


def wheel_layout(profile):
    """Wheel, steering link and steering joint layout in the model frame (base_footprint) from the profile."""
    v = profile.value
    steer_type = v('steer_type')
    front_x = v('rear_axle_x_in_base_m') + v('wheelbase_m')
    rear_x = v('rear_axle_x_in_base_m')
    r_front, r_rear = v('wheel_radius_front_m'), v('wheel_radius_rear_m')
    t_front, t_rear, w_k = v('track_front_m'), v('track_rear_m'), v('kingpin_width_m')
    limits = v('steer_mech_limit_deg')
    wheels = {
        'left_front_wheel': dict(xyz=(front_x, t_front / 2., r_front), radius=r_front, front=True, side='left'),
        'right_front_wheel': dict(xyz=(front_x, -t_front / 2., r_front), radius=r_front, front=True, side='right'),
        'left_rear_wheel': dict(xyz=(rear_x, t_rear / 2., r_rear), radius=r_rear, front=False, side='left'),
        'right_rear_wheel': dict(xyz=(rear_x, -t_rear / 2., r_rear), radius=r_rear, front=False, side='right'),
    }
    caster = math.radians(v('caster_deg'))
    kpi = math.radians(v('kingpin_inclination_deg'))
    if steer_type == geo.LINKAGE:
        steering = {}
        for side, sign in (('left', 1.), ('right', -1.)):
            lower = -math.radians(limits['%s_wheel_toward_right' % side])
            upper = math.radians(limits['%s_wheel_toward_left' % side])
            steering[side] = dict(link=STEER_LINKS[side], joint=STEER_JOINTS[side], xyz=(front_x, sign * w_k / 2., r_front),
                                  wheels=('%s_front_wheel' % side,), lower=lower, upper=upper,
                                  axis=steering_axis(side, caster, kpi))
    elif steer_type == geo.AXLE_PIVOT:
        lower = -math.radians(min(limits['left_wheel_toward_right'], limits['right_wheel_toward_right']))
        upper = math.radians(min(limits['left_wheel_toward_left'], limits['right_wheel_toward_left']))
        steering = {'axle': dict(link=AXLE_STEER_LINK, joint=AXLE_STEER_JOINT, xyz=(front_x, 0., r_front),
                                 wheels=FRONT_WHEELS, lower=lower, upper=upper, axis=steering_axis('left', caster, 0.))}
    else:
        raise SteerModelError('unknown_steer_type:%r' % (steer_type,))
    return dict(steer_type=steer_type, wheels=wheels, steering=steering, front_axle_x_m=front_x, rear_axle_x_m=rear_x)


def steer_topics(steer_type, prefix=DEFAULT_TOPIC_PREFIX):
    """Steering target topics {key: topic}: linkage left/right, axle_pivot axle."""
    if steer_type == geo.AXLE_PIVOT:
        return {'axle': prefix + '/steer_axle_cmd'}
    return {'left': prefix + '/steer_left_cmd', 'right': prefix + '/steer_right_cmd'}


def wheel_topics(prefix=DEFAULT_TOPIC_PREFIX):
    """Drive topics {wheel link name: topic} (gz.msgs.Double)."""
    return {wheel: '%s/%s_cmd' % (prefix, wheel) for wheel in WHEELS}


def ackermann_topic(prefix=DEFAULT_TOPIC_PREFIX):
    return prefix + '/ackermann_cmd_vel'


def kinematic_wheel_speeds(profile_or_geometry, delta_left_rad, delta_right_rad, rear_speed_m_s):
    """Rolling-without-slip joint speeds (rad/s) of the four wheels for a rear-axle-centre speed.

    The instantaneous centre of rotation is on the rear axle line at R = L / tan(delta_eq) with
    delta_eq from the wheel angles (geometry.equivalent_from_wheels). Each wheel speed is
    omega * |ICR -> wheel centre| / r. The front wheel centres move with the steering
    (geometry.front_wheel_centre). Signs follow the rear speed. Straight: v / r for all wheels.
    Use with ideal Ackermann wheel angles (ratio 1); other wheel angle pairs have no common ICR
    and the speeds then follow the equivalent angle only.
    """
    g = profile_or_geometry if isinstance(profile_or_geometry, geo.SteeringGeometry) else \
        geo.SteeringGeometry.from_profile(profile_or_geometry)
    v = float(rear_speed_m_s)
    delta = g.equivalent_from_wheels(delta_left_rad, delta_right_rad)
    out = {}
    if abs(math.tan(delta)) < 1e-12:
        for wheel in WHEELS:
            radius = g.wheel_radius_front_m if wheel in FRONT_WHEELS else g.wheel_radius_rear_m
            out[wheel + '_joint'] = v / radius
        return out
    radius_icr = g.wheelbase_m / math.tan(delta)             # signed, + left; ICR at (rear axle x, R) in base
    omega = v / radius_icr
    icr = (g.rear_axle_x_in_base_m, radius_icr)
    centres = {
        'left_front_wheel': g.front_wheel_centre(geo.LEFT, delta_left_rad),
        'right_front_wheel': g.front_wheel_centre(geo.RIGHT, delta_right_rad),
        'left_rear_wheel': (g.rear_axle_x_in_base_m, g.track_rear_m / 2.),
        'right_rear_wheel': (g.rear_axle_x_in_base_m, -g.track_rear_m / 2.),
    }
    for wheel, (x, y) in centres.items():
        distance = math.hypot(x - icr[0], y - icr[1])
        radius = g.wheel_radius_front_m if wheel in FRONT_WHEELS else g.wheel_radius_rear_m
        out[wheel + '_joint'] = abs(omega) * distance / radius * (1. if v >= 0. else -1.)
    return out


# --- the transform ------------------------------------------------------------------------------------------------
def _check_generated(model):
    if model.tag != 'model' or model.get('name') is None:
        raise SteerModelError('not_a_model_element')
    for name in KEPT_LINKS + WHEELS:
        _link(model, name)
    for name in WHEEL_JOINTS:
        joint = _joint(model, name)
        if joint.get('type') != 'revolute' or joint.findtext('parent') != 'base_link':
            raise SteerModelError('unexpected_wheel_joint:' + name)
    for side in ('left', 'right'):
        for name in (STEER_LINKS[side], STEER_JOINTS[side]):
            if model.find("*[@name='%s']" % name) is not None:
                raise SteerModelError('already_steered:' + name)
    if model.find("*[@name='%s']" % AXLE_STEER_LINK) is not None:
        raise SteerModelError('already_steered:' + AXLE_STEER_LINK)
    kinds = [_plugin_kind(p) for p in model.findall('plugin')]
    for kind in KEPT_PLUGINS:
        if kinds.count(kind) != 1:
            raise SteerModelError('expected_one_plugin:' + kind)


def _set_wheel(model, wheel, info, profile, mu_lat, mu_long, collision_shape=DEFAULT_WHEEL_COLLISION):
    v = profile.value
    link = _link(model, wheel)
    old = _parse_pose(link)
    link.find('pose').text = _pose_text(info['xyz'])
    radius, width = info['radius'], v('wheel_width_m')
    mass = v('wheel_mass_kg')
    _set_inertial(link, mass, _cylinder_inertia(mass, radius, width, 'y'))
    for element in link.findall('visual'):
        cylinder = element.find('geometry/cylinder')
        if cylinder is not None:
            _set(cylinder, 'radius', radius)
            _set(cylinder, 'length', width)
    for element in link.findall('collision'):
        geometry = element.find('geometry')
        if geometry is None or geometry.find('cylinder') is None:
            raise SteerModelError('missing_wheel_collision_cylinder:' + wheel)
        if collision_shape == 'cylinder':
            cylinder = geometry.find('cylinder')
            _set(cylinder, 'radius', radius)
            _set(cylinder, 'length', width)
        else:          # sphere of the wheel radius about the wheel centre; the collision pose (axle = z) is kept
            for child in list(geometry):
                geometry.remove(child)
            _sub(_sub(geometry, 'sphere'), 'radius', radius)
    friction = link.find('collision/surface/friction/ode')
    if friction is None:
        raise SteerModelError('missing_wheel_friction:' + wheel)
    _set(friction, 'mu', mu_lat)
    _set(friction, 'mu2', mu_long)
    _set(friction, 'fdir1', '0 0 1')
    moved = tuple(info['xyz'][i] - old[i] for i in range(3))
    return dict(wheel=wheel, generated_xyz=list(old[:3]), xyz=list(info['xyz']), moved_m=list(moved), radius_m=radius,
                width_m=width, mass_kg=mass, collision_shape=collision_shape, visual_shape='cylinder')


def _add_steering(model, key, spec, profile, insert_before):
    v = profile.value
    radius, length, floor = (PLACEHOLDERS[n][0] for n in ('steer_link_radius_m', 'steer_link_length_m',
                                                           'steer_link_inertia_min_kgm2'))
    mass = v('steer_link_mass_kg')
    link = E.Element('link', name=spec['link'])
    _sub(link, 'pose', _pose_text(spec['xyz']))
    diagonal = tuple(max(floor, value) for value in _cylinder_inertia(mass, radius, length, 'z'))
    _set_inertial(link, mass, diagonal)
    visual = _sub(link, 'visual', name='kingpin')
    _sub(visual, 'pose', _pose_text((0., 0., 0.)))
    cylinder = _sub(_sub(visual, 'geometry'), 'cylinder')
    _sub(cylinder, 'radius', radius)
    _sub(cylinder, 'length', length)
    material = _sub(visual, 'material')
    _sub(material, 'ambient', '.75 .45 .10 1')
    _sub(material, 'diffuse', '.75 .45 .10 1')
    joint = E.Element('joint', name=spec['joint'], type='revolute')
    _sub(joint, 'parent', 'base_link')
    _sub(joint, 'child', spec['link'])
    axis = _sub(joint, 'axis')
    _sub(axis, 'xyz', ' '.join(_num(c) for c in spec['axis']))
    limit = _sub(axis, 'limit')
    _sub(limit, 'lower', spec['lower'])
    _sub(limit, 'upper', spec['upper'])
    _sub(limit, 'effort', PLACEHOLDERS['steer_joint_effort_limit_nm'][0])
    index = list(model).index(insert_before)
    model.insert(index, joint)
    model.insert(index, link)
    for wheel in spec['wheels']:
        _joint(model, wheel + '_joint').find('parent').text = spec['link']
    return dict(key=key, link=spec['link'], joint=spec['joint'], xyz=list(spec['xyz']),
                axis=[c + 0. for c in spec['axis']], effort_limit_nm=PLACEHOLDERS['steer_joint_effort_limit_nm'][0],
                lower_rad=spec['lower'], upper_rad=spec['upper'], lower_deg=math.degrees(spec['lower']),
                upper_deg=math.degrees(spec['upper']), wheels=list(spec['wheels']), mass_kg=mass,
                inertia_diag_kgm2=list(diagonal))


def _rotate_xy(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return x * c - y * s, x * s + y * c


def _angles(lower, upper, step_rad=math.radians(.25)):
    """Samples of [lower, upper] (both ends included)."""
    count = max(1, int(math.ceil((upper - lower) / step_rad)))
    return [lower + (upper - lower) * i / count for i in range(count + 1)]


def wheel_keepouts(profile, bottom_z, clearance_m, travel_up_m=0.):
    """Top-view zones the body must leave free for the wheels, for a body part from bottom_z (model frame) upward.

    For each wheel whose top is above bottom_z: the rectangle (chord 2a of the wheel circle grown by clearance_m,
    at bottom_z, x the width) of the wheel part above bottom_z, swept over the steering range for the steered
    front wheels (rotation about the kingpin, or about the axle centre for axle_pivot; caster and kingpin
    inclination ignored), the inner side moved in by clearance_m. travel_up_m raises the wheel centres (suspension
    compression; the highest wheel position gives the largest zone). Returns a list of dict(wheel, side, x_min,
    x_max, y_inner (smallest |y| on the wheel's side), top_z (wheel top + clearance)).
    """
    layout = wheel_layout(profile)
    width = profile.value('wheel_width_m')
    zones = []
    for wheel, info in layout['wheels'].items():
        cx, cy, cz = info['xyz']
        cz += travel_up_m
        radius = info['radius']
        grown = radius + clearance_m
        if cz + grown <= bottom_z:
            continue
        dz = bottom_z - cz
        half = grown if dz <= 0. else math.sqrt(max(0., grown * grown - dz * dz))
        rect = [(half, width / 2.), (-half, width / 2.), (-half, -width / 2.), (half, -width / 2.)]
        if info['front']:
            spec = layout['steering']['axle' if 'axle' in layout['steering'] else info['side']]
            px, py = spec['xyz'][0], spec['xyz'][1]
            angles = _angles(spec['lower'], spec['upper'])
        else:
            px, py, angles = cx, cy, [0.]
        points = []
        for angle in angles:
            for rx, ry in rect:
                ox, oy = _rotate_xy(cx - px + rx, cy - py + ry, angle)
                points.append((px + ox, py + oy))
        sign = 1. if info['side'] == 'left' else -1.
        zones.append(dict(wheel=wheel, side=info['side'], x_min=min(p[0] for p in points),
                          x_max=max(p[0] for p in points), y_inner=min(sign * p[1] for p in points) - clearance_m,
                          top_z=cz + grown))
    return zones


def body_boxes(profile, link_xyz, box, clearance_m, travel_up_m=0.):
    """Boxes that fill the body envelope `box` (centred at the base_link origin link_xyz, model frame) except the
    wheel zones: an upper deck above the highest wheel top plus clearance, and lower segments along x whose
    widths stop clearance_m short of the wheels over the steering range (wheel wells).

    Returns (boxes, zones): boxes = [dict(name, centre (link frame), size)], the first is 'body'.
    Without any wheel reaching into the envelope the result is the envelope itself.
    """
    length, width, height = (float(v) for v in box)
    bx, by, bz = link_xyz
    x0, x1, y0, y1, z0, z1 = bx - length / 2., bx + length / 2., by - width / 2., by + width / 2., \
        bz - height / 2., bz + height / 2.
    zones = [z for z in wheel_keepouts(profile, z0, clearance_m, travel_up_m)
             if z['x_max'] > x0 and z['x_min'] < x1
             and (z['y_inner'] < y1 if z['side'] == 'left' else -z['y_inner'] > y0)]
    if not zones:
        return [dict(name='body', centre=[0., 0., 0.], size=[length, width, height])], []
    cut = min(z1, max(z['top_z'] for z in zones))
    boxes = []
    if cut < z1 - 1e-9:
        boxes.append(dict(name='body', centre=[0., 0., (cut + z1) / 2. - bz], size=[length, width, z1 - cut]))
    breaks = sorted({x0, x1} | {min(x1, max(x0, z[k])) for z in zones for k in ('x_min', 'x_max')})
    segments = []
    for xa, xb in zip(breaks, breaks[1:]):
        if xb - xa < 1e-6:
            continue
        mid = (xa + xb) / 2.
        left, right = y1, y0
        for z in zones:
            if z['x_min'] < mid < z['x_max']:
                if z['side'] == 'left':
                    left = min(left, z['y_inner'])
                else:
                    right = max(right, -z['y_inner'])
        if segments and segments[-1][2:] == [right, left] and abs(segments[-1][1] - xa) < 1e-9:
            segments[-1][1] = xb
        else:
            segments.append([xa, xb, right, left])
    lower = []
    for xa, xb, right, left in segments:
        if left - right <= 1e-3 or xb - xa <= 1e-3 or cut - z0 <= 1e-6:
            continue
        lower.append(dict(name='body_lower_%d' % len(lower),
                          centre=[(xa + xb) / 2. - bx, (right + left) / 2. - by, (z0 + cut) / 2. - bz],
                          size=[xb - xa, left - right, cut - z0]))
    boxes.extend(lower)
    if not boxes:
        raise SteerModelError('body_has_no_box_left_outside_the_wheel_zones')
    if boxes[0]['name'] != 'body':                 # no deck: the first lower box keeps the generated name
        boxes[0]['name'] = 'body'
    return boxes, zones


def _link_mass_point(link):
    """(mass, model-frame position of the link's centre of mass) of a generated link (inertial pose in the link frame,
    turned by the link's rpy), or None without an inertial mass."""
    mass = link.findtext('inertial/mass')
    if mass is None:
        return None
    pose = _parse_pose(link)
    offset = _parse_pose(link.find('inertial'))[:3]
    rot = _rpy_matrix(*pose[3:6])
    centre = tuple(pose[i] + sum(rot[i][j] * offset[j] for j in range(3)) for i in range(3))
    return float(mass), centre


def planned_body_links(model, profile, steering, *, front_axle_pivot=True, suspension=False):
    """Non-wheel links other than base_link that the steered model will have: [(name, mass kg, model-frame centre)].
    steering: the _add_steering manifest blocks; the pivot and suspension links are added later at the positions used
    here (_add_front_pivot, _add_suspension); the sensor links are the generated ones."""
    layout = wheel_layout(profile)
    out = [(item['link'], float(item['mass_kg']), tuple(item['xyz'])) for item in steering]
    if front_axle_pivot:
        out.append((PIVOT_LINK, PLACEHOLDERS['front_pivot_link_mass_kg'][0],
                    (layout['front_axle_x_m'], 0., profile.value('wheel_radius_front_m'))))
    if suspension:
        for wheel in WHEELS:
            out.append((SUSPENSION_LINK % wheel, PLACEHOLDERS['suspension_link_mass_kg'][0],
                        tuple(layout['wheels'][wheel]['xyz'])))
    for name in SENSOR_LINKS:
        link = model.find("link[@name='%s']" % name)
        point = None if link is None else _link_mass_point(link)
        if point is not None and point[0] > 0.:
            out.append((name, point[0], point[1]))
    return out


def _body_mass_split(profile, base_pose, other_links):
    """base_link mass and centre of mass (model frame) so that base_link + other_links = mass_kg at the profile
    centre of gravity (cog_x_in_base_m, 0, cog_z_m). The profile mass_kg is the body mass without the wheels
    (measured with axle scales); steering links, pivot, suspension and sensor links are parts of that body."""
    v = profile.value
    total = float(v('mass_kg'))
    others = sum(mass for _, mass, _ in other_links)
    base_mass = total - others
    if not base_mass > .5 * total:
        raise SteerModelError('base_link_mass_too_small: mass_kg %g minus the other body links %g' % (total, others))
    cog = (float(v('cog_x_in_base_m')), 0., float(v('cog_z_m')))
    centre = tuple((total * cog[i] - sum(mass * xyz[i] for _, mass, xyz in other_links)) / base_mass for i in range(3))
    return base_mass, centre, cog


def _set_body(model, profile, wheel_wells=True, travel_up_m=0., other_links=()):
    v = profile.value
    link = _link(model, 'base_link')
    pose = _parse_pose(link)
    box = [float(x) for x in v('gz_body_box_m')]
    mass, centre, profile_cog = _body_mass_split(profile, pose, other_links)
    l_, w_, h_ = box
    diagonal = (mass * (w_ * w_ + h_ * h_) / 12., mass * (l_ * l_ + h_ * h_) / 12., mass * (l_ * l_ + w_ * w_) / 12.)
    cog = (centre[0] - pose[0], centre[1] - pose[1], centre[2] - pose[2])
    _set_inertial(link, mass, diagonal, pose=cog + (0., 0., 0.))
    visual, collision = link.find("visual[@name='body']"), link.find("collision[@name='body_collision']")
    for element in (visual, collision):
        if element is None or element.find('geometry/box') is None:
            raise SteerModelError('missing_body_box')
    clearance = PLACEHOLDERS['body_wheel_clearance_m'][0]
    if wheel_wells:
        boxes, zones = body_boxes(profile, pose[:3], box, clearance, travel_up_m)
    else:
        boxes, zones = [dict(name='body', centre=[0., 0., 0.], size=list(box))], []
    for element, suffix in ((visual, ''), (collision, '_collision')):
        index = list(link).index(element)
        for offset, item in enumerate(boxes):
            target = element if offset == 0 else copy.deepcopy(element)
            target.set('name', item['name'] + suffix)
            pose_element = target.find('pose')
            if pose_element is None:
                pose_element = E.Element('pose')
                target.insert(0, pose_element)
            pose_element.text = _pose_text(item['centre'])
            target.find('geometry/box/size').text = ' '.join(_num(x) for x in item['size'])
            if offset:
                link.insert(index + offset, target)
    others = {name: mass_ for name, mass_, _ in other_links}
    return dict(mass_kg=mass, base_link_mass_kg=mass, profile_body_mass_kg=float(v('mass_kg')),
                other_body_link_masses_kg=others, other_body_links_total_kg=sum(others.values()),
                body_cog_model_m=list(profile_cog), base_link_cog_model_m=list(centre), cog_in_link=list(cog), box_m=box,
                inertia_diag_kgm2=list(diagonal),
                inertia_note='inertia of the full envelope box with the base_link mass (unchanged by the wheel wells)',
                mass_note='base_link = profile mass_kg minus the other non-wheel links (steering, pivot, suspension, '
                          'sensor links); all non-wheel links together keep the profile centre of gravity',
                wheel_wells=bool(wheel_wells and len(boxes) > 1), clearance_m=clearance if wheel_wells else None,
                wheel_travel_up_m=travel_up_m if wheel_wells else None, boxes=boxes, wheel_zones=zones)


SUSPENSION_LINK = '%s_suspension_link'
SUSPENSION_JOINT = '%s_suspension_joint'
PIVOT_LINK = 'front_axle_pivot_link'
PIVOT_JOINT = 'front_axle_roll_joint'


def pivot_rise_m(profile):
    """Largest rise of a front wheel's outer edge when the front axle rolls to the pivot limit (m)."""
    limit = math.radians(PLACEHOLDERS['front_pivot_roll_limit_deg'][0])
    return (profile.value('track_front_m') / 2. + profile.value('wheel_width_m') / 2.) * math.sin(limit)


def _add_front_pivot(model, profile, steering):
    """Passive roll pivot (revolute about x, no spring) between base_link and the front steering joints: the front
    axle can roll so that the four wheels share the load on twisted ground (three-point support). Returns the
    manifest block. Default on (steer_model_element front_axle_pivot)."""
    layout = wheel_layout(profile)
    limit = math.radians(PLACEHOLDERS['front_pivot_roll_limit_deg'][0])
    damping = PLACEHOLDERS['front_pivot_damping_nms_rad'][0]
    mass = PLACEHOLDERS['front_pivot_link_mass_kg'][0]
    floor = PLACEHOLDERS['steer_link_inertia_min_kgm2'][0]
    xyz = (layout['front_axle_x_m'], 0., profile.value('wheel_radius_front_m'))
    link = E.Element('link', name=PIVOT_LINK)
    _sub(link, 'pose', _pose_text(xyz))
    _set_inertial(link, mass, (floor, floor, floor))
    joint = E.Element('joint', name=PIVOT_JOINT, type='revolute')
    _sub(joint, 'parent', 'base_link')
    _sub(joint, 'child', PIVOT_LINK)
    axis = _sub(joint, 'axis')
    _sub(axis, 'xyz', '1 0 0')
    lim = _sub(axis, 'limit')
    _sub(lim, 'lower', -limit)
    _sub(lim, 'upper', limit)
    dynamics = _sub(axis, 'dynamics')
    _sub(dynamics, 'damping', damping)
    _sub(dynamics, 'friction', 0.)
    first = _link(model, steering[0]['link'])
    index = list(model).index(first)
    model.insert(index, joint)
    model.insert(index, link)
    for item in steering:
        _joint(model, item['joint']).find('parent').text = PIVOT_LINK
    return dict(enabled=True, link=PIVOT_LINK, joint=PIVOT_JOINT, xyz=list(xyz), roll_limit_rad=limit,
                damping_nms_rad=damping, link_mass_kg=mass,
                note='passive front axle roll pivot (no spring), default on; SIM_ASSUMED stand-in for unknown '
                     'chassis compliance (three-point support)')


def _add_suspension(model, profile, steering):
    """EXPERIMENTAL (option suspension, default off). Insert a vertical spring-damper (prismatic joint) between each
    wheel and its parent (base_link, or the steering link of a steered wheel). Returns the manifest block.

    Rate k from the mean corner load and the static deflection placeholder; each spring is preloaded
    (spring_reference = -load / k) so that the rest position would be 0. Damping c = 2 zeta sqrt(k m_corner).
    Measured in gz-sim 8.15 / DART 6.13 (p5_demo experiments): a hanging mass gives the exact equilibrium, but
    under ground contact the static spring deflection is 2.23 x load / k at 120 Hz with this damping (1.22 x
    without damping, 1.12 x at 1 ms, 2.88 x with 4 x k). The body then sits about 1.8 cm low; not for evaluation.
    """
    v = profile.value
    deflection, zeta, travel, link_mass = (PLACEHOLDERS[n][0] for n in (
        'suspension_static_deflection_m', 'suspension_damping_ratio', 'suspension_travel_m', 'suspension_link_mass_kg'))
    floor = PLACEHOLDERS['steer_link_inertia_min_kgm2'][0]
    g = 9.80665
    fixed = 0.
    for link in model.findall('link'):
        name = link.get('name')
        if name in WHEELS or name in STEER_LINKS.values() or name == AXLE_STEER_LINK:
            continue
        mass = link.findtext('inertial/mass')
        fixed += float(mass) if mass is not None else 0.
    layout = wheel_layout(profile)
    rear_x, front_x = layout['rear_axle_x_m'], layout['front_axle_x_m']
    cog_x = v('cog_x_in_base_m')
    front_share = min(1., max(0., (cog_x - rear_x) / (front_x - rear_x)))
    steer_mass = sum(item['mass_kg'] for item in steering)
    loads = {}
    for wheel in WHEELS:
        front = wheel in FRONT_WHEELS
        share = front_share if front else 1. - front_share
        sprung = fixed * share / 2. + (steer_mass / 2. if front else 0.)
        loads[wheel] = sprung
    mean_mass = sum(loads.values()) / len(loads)
    rate = mean_mass * g / deflection
    damping = 2. * zeta * math.sqrt(rate * mean_mass)
    rows = []
    for wheel in WHEELS:
        joint = _joint(model, wheel + '_joint')
        parent = joint.findtext('parent')
        wheel_link = _link(model, wheel)
        link = E.Element('link', name=SUSPENSION_LINK % wheel)
        _sub(link, 'pose', wheel_link.findtext('pose'))
        _set_inertial(link, link_mass, (floor, floor, floor))
        prismatic = E.Element('joint', name=SUSPENSION_JOINT % wheel, type='prismatic')
        _sub(prismatic, 'parent', parent)
        _sub(prismatic, 'child', SUSPENSION_LINK % wheel)
        axis = _sub(prismatic, 'axis')
        _sub(axis, 'xyz', '0 0 1')
        limit = _sub(axis, 'limit')
        _sub(limit, 'lower', -travel)
        _sub(limit, 'upper', travel)
        dynamics = _sub(axis, 'dynamics')
        _sub(dynamics, 'damping', damping)
        _sub(dynamics, 'friction', 0.)
        reference = -loads[wheel] * g / rate
        _sub(dynamics, 'spring_reference', reference)
        _sub(dynamics, 'spring_stiffness', rate)
        index = list(model).index(wheel_link)
        model.insert(index, prismatic)
        model.insert(index, link)
        joint.find('parent').text = SUSPENSION_LINK % wheel
        rows.append(dict(wheel=wheel, link=SUSPENSION_LINK % wheel, joint=SUSPENSION_JOINT % wheel, parent=parent,
                         sprung_mass_kg=loads[wheel], spring_reference_m=reference))
    return dict(enabled=True, rate_n_m=rate, damping_ns_m=damping, travel_m=travel, link_mass_kg=link_mass,
                static_deflection_m=deflection, damping_ratio=zeta, fixed_body_mass_kg=fixed, front_share=front_share,
                corners=rows, note='vertical spring-damper per wheel (prismatic joint); preloaded so the rest geometry '
                                   'is the generated one; SIM_ASSUMED placeholders')


def _rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw),
                              math.sin(yaw))
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def wheel_body_clearance(model, profile, *, step_deg=1., rim_samples=96, width_samples=9, cap_rings=8):
    """Smallest signed distance (m) between each wheel cylinder and the base_link collision boxes over the
    steering range (negative = overlap depth). Sampled: wheel surface points against the exact box distance.

    Reads the wheel radius and width from the wheel collision cylinder, or from the visual cylinder when the collision
    is a sphere (wheel_collision='sphere': the wells are for the visible wheel; links of one model do not collide), and
    the boxes from base_link of the given model element (a steered model). Steered front wheels turn about their
    steering joint (planar rotation about the joint position, as wheel_layout). Needs numpy. Returns dict(per_wheel,
    min_clearance_m, overlap_free, wheel_shape_source).
    """
    import numpy as np
    base = _link(model, 'base_link')
    base_pose = _parse_pose(base)
    boxes = []
    for collision in base.findall('collision'):
        size = collision.findtext('geometry/box/size')
        if size is None:
            continue
        pose = _parse_pose(collision)
        boxes.append((collision.get('name'), np.array(pose[:3]) + np.array(base_pose[:3]),
                      np.array(_rpy_matrix(*pose[3:6])), np.array([float(s) for s in size.split()]) / 2.))
    layout = wheel_layout(profile)
    result = {}
    worst = None
    sources = set()
    for wheel, info in layout['wheels'].items():
        link = _link(model, wheel)
        source = 'collision' if link.find('collision/geometry/cylinder') is not None else 'visual'
        sources.add(source)
        radius = float(link.findtext('%s/geometry/cylinder/radius' % source))
        length = float(link.findtext('%s/geometry/cylinder/length' % source))
        centre = np.array(_parse_pose(link)[:3])
        theta = np.linspace(0., 2. * np.pi, rim_samples, endpoint=False)
        ys = np.linspace(-length / 2., length / 2., width_samples)
        rim = np.array([(radius * np.cos(a), y, radius * np.sin(a)) for a in theta for y in ys])
        rings = np.linspace(0., radius, cap_rings + 1)[1:-1]
        caps = np.array([(r * np.cos(a), side * length / 2., r * np.sin(a)) for r in rings for a in theta
                         for side in (-1., 1.)] + [(0., side * length / 2., 0.) for side in (-1., 1.)])
        local = np.vstack([rim, caps])                 # wheel frame: axle y, centre at the origin
        if info['front']:
            spec = layout['steering']['axle' if 'axle' in layout['steering'] else info['side']]
            pivot = np.array(spec['xyz'])
            angles = _angles(spec['lower'], spec['upper'], math.radians(step_deg))
        else:
            pivot, angles = centre, [0.]
        low, high = 0., 0.
        suspension = model.find("joint[@name='%s']" % (SUSPENSION_JOINT % wheel))
        if suspension is not None:                    # the whole suspension travel
            low, high = float(suspension.findtext('axis/limit/lower')), float(suspension.findtext('axis/limit/upper'))
        if info['front'] and model.find("joint[@name='%s']" % PIVOT_JOINT) is not None:
            rise = pivot_rise_m(profile)              # front axle roll (vertical approximation)
            low, high = low - rise, high + rise
        heights = sorted({low, 0., high})
        best = None
        for angle in angles:
            c, s = math.cos(angle), math.sin(angle)
            rot = np.array(((c, -s, 0.), (s, c, 0.), (0., 0., 1.)))
            for lift in heights:
                points = (centre - pivot + local) @ rot.T + pivot + np.array((0., 0., lift))
                for name, box_centre, box_rot, half in boxes:
                    p = np.abs((points - box_centre) @ box_rot) - half
                    outside = np.linalg.norm(np.maximum(p, 0.), axis=1)
                    inside = np.minimum(np.max(p, axis=1), 0.)
                    distance = float(np.min(outside + inside))
                    if best is None or distance < best['clearance_m']:
                        best = dict(clearance_m=distance, steer_deg=math.degrees(angle), box=name, lift_m=lift)
        result[wheel] = best
        if worst is None or best['clearance_m'] < worst:
            worst = best['clearance_m']
    return dict(per_wheel=result, min_clearance_m=worst, overlap_free=bool(worst is not None and worst > 0.),
                boxes=[b[0] for b in boxes], wheel_shape_source=sorted(sources),
                note='sampled wheel cylinder surface vs exact box distance; steering swept in %g deg steps; caster '
                     'and kingpin inclination ignored' % step_deg)


def _set_imu(model, profile):
    v = profile.value
    link = _link(model, 'imu_link')
    sensor = link.find("sensor[@type='imu']")
    if sensor is None:
        return dict(present=False)
    link_pose = _parse_pose(link)
    want = v('imu_pose_in_base')
    offset = (want['x'] - link_pose[0], want['y'] - link_pose[1], want['z'] - link_pose[2])
    rpy = v('imu_mount_rpy_deg')
    if rpy['yaw_sign'] not in (1, -1):
        raise SteerModelError('imu_yaw_sign_must_be_plus_or_minus_one')
    roll = math.radians(rpy['roll']) + (math.pi if rpy['yaw_sign'] == -1 else 0.)
    rotation = (roll, math.radians(rpy['pitch']), math.radians(rpy['yaw']))
    old_rate = sensor.findtext('update_rate')
    _set(sensor, 'update_rate', float(v('imu_rate_hz')))
    pose = sensor.find('pose')
    if pose is None:
        pose = E.Element('pose')
        sensor.insert(0, pose)
    pose.text = _pose_text(offset, rotation)
    return dict(present=True, update_rate_hz=float(v('imu_rate_hz')), generated_update_rate_hz=old_rate,
                sensor_offset_in_link_m=list(offset), sensor_rpy_rad=list(rotation), yaw_sign=rpy['yaw_sign'])


def _pose_checks(model, profile):
    """Profile sensor positions vs the generated (pinned rover.yaml) link positions. Links are not moved."""
    v = profile.value
    wanted = {'laser': (v('lidar_x_in_base_m'), v('lidar_y_in_base_m'), v('lidar_z_m')),
              'camera_link': tuple(v('camera_pose_in_base')[k] for k in 'xyz')}
    checks = {}
    for name, want in wanted.items():
        have = _parse_pose(_link(model, name))[:3]
        diff = [want[i] - have[i] for i in range(3)]
        checks[name] = dict(link_xyz=list(have), profile_xyz=list(want), difference_m=diff,
                            matches=all(abs(d) <= POSE_CHECK_TOLERANCE_M for d in diff))
    return checks


def vehicle_mass_check(model, profile):
    """Sum of every link mass of the model against drive.vehicle_mass_kg (mass_kg + 4 wheels, the F-SIM mass), and the
    centre of mass of the non-wheel links against the profile centre of gravity."""
    total, body_mass, moment = 0., 0., [0., 0., 0.]
    for link in model.findall('link'):
        point = _link_mass_point(link)
        if point is None:
            continue
        mass, centre = point
        total += mass
        if link.get('name') not in WHEELS:
            body_mass += mass
            moment = [moment[i] + mass * centre[i] for i in range(3)]
    expected = drive_model.vehicle_mass_kg(profile)
    cog = [m / body_mass for m in moment] if body_mass > 0. else None
    want = (float(profile.value('cog_x_in_base_m')), 0., float(profile.value('cog_z_m')))
    cog_error = None if cog is None else max(abs(cog[i] - want[i]) for i in range(3))
    return dict(links_total_kg=total, profile_vehicle_mass_kg=expected, body_links_kg=body_mass,
                body_cog_model_m=cog, profile_cog_model_m=list(want), body_cog_error_m=cog_error,
                matches=bool(abs(total - expected) <= 1e-9 * max(1., expected)
                             and cog_error is not None and cog_error <= 1e-9))


def sensor_mount_flags(checks, imu_block):
    """result_flags of sensor links whose profile position is not applied (links stay at rover.yaml)."""
    flags = ['sensor_mount_not_applied:%s' % name for name, check in sorted(checks.items()) if not check['matches']]
    if not checks.get('laser', {}).get('matches', True) and imu_block.get('present'):
        flags.append('lidar_imu_relative_pose_not_the_profile')    # the IMU sensor moved to the profile, the laser did not
    return flags


def steer_model_element(model, profile, *, drive_mode=None, steer_control='position', topic_prefix=DEFAULT_TOPIC_PREFIX,
                        odom_topic=TRUTH_ODOM_TOPIC, joint_state_topic=TRUTH_JOINT_STATE_TOPIC, curve='main',
                        battery_scale=None, tau_stall_nm=None, initial_steer_rad=None, body_wheel_wells=True,
                        suspension=False, front_axle_pivot=True, wheel_collision=None,
                        allow_sensor_mount_mismatch=False, wheel_mu_lat=None, wheel_mu_long=None):
    """Transform a generated <model name="ubts_rover"> element in place. Returns the manifest dict.

    drive_mode      None -> profile gz_drive_mode; 'force_dc' or 'kinematic_velocity'
    steer_control   'position' (JointPositionController, main) or 'ackermann' (AckermannSteering cross-check;
                    linkage only; the rear wheels are driven by the plugin, no drive JointControllers)
    topic_prefix    command topic prefix (default /sim/act); probe scenes with several vehicles use their own
    curve, battery_scale, tau_stall_nm   passed to drive.dc_motor_line() (None: profile values)
    initial_steer_rad  None, or {key: wheel angle} for the JointPositionController initial_position
    body_wheel_wells   True (default): the body box becomes an upper deck and lower segments that keep
                    body_wheel_clearance_m from every wheel over the steering range (the generated box overlapped
                    the wheel tops and a steered front wheel swept through it); mass and inertia are those of the
                    full box. False keeps the single generated box (before/after comparison only).
    front_axle_pivot   True (default): a passive front axle roll pivot (no spring) so that the four wheels share
                    the load on twisted ground; the rigid model lifted one wheel 64 % of the driving time on the
                    original orchard terrain and stalled at duty 0.48 and 0.8. False gives the rigid model.
    suspension      False (default). Experimental vertical spring-damper per wheel. DART joint springs at the
                    120 Hz step carry a wrong static load under ground contact (measured 1.2-2.9 x the corner load,
                    depending on damping, step and stiffness), so the ride height is wrong; not for evaluation.
    wheel_collision None (default) = DEFAULT_WHEEL_COLLISION 'sphere' (one contact under the wheel centre;
                    DECISIONS_KO R13 decision 1), or 'sphere' / 'cylinder' (the generated shape, an option). The
                    visual stays a cylinder. The cylinder's two edge contacts drag a turning wheel and bias the
                    kinematic-mode speed (GZ_MODEL_KO 8). The manifest records the shape (wheel_collision) and
                    whether it came from the default or the option (wheel_collision_choice)
    allow_sensor_mount_mismatch  False (default): refuse a profile whose LiDAR / camera position differs from the
                    generated links (they are not moved: the Embree LiDAR reads rover.yaml). True builds the model
                    and marks the manifest result_flags (sensor_mount_not_applied:<link>)
    wheel_mu_lat, wheel_mu_long  None (default): the profile values. A number overrides the wheel friction of this
                    model only (diagnostics, e.g. the anisotropic fdir1 check of gz_probe; recorded in the manifest)
    """
    if steer_control not in STEER_CONTROLS:
        raise SteerModelError('unknown_steer_control:%r' % (steer_control,))
    drive_mode = profile.value('gz_drive_mode') if drive_mode is None else drive_mode
    if drive_mode not in DRIVE_MODES:
        raise SteerModelError('unknown_drive_mode:%r' % (drive_mode,))
    collision_source = 'default' if wheel_collision is None else 'model_option'
    wheel_collision = DEFAULT_WHEEL_COLLISION if wheel_collision is None else wheel_collision
    if wheel_collision not in WHEEL_COLLISION_SHAPES:
        raise SteerModelError('unknown_wheel_collision:%r (choices %s)' % (wheel_collision,
                                                                           ', '.join(WHEEL_COLLISION_SHAPES)))
    _check_generated(model)
    v = profile.value
    layout = wheel_layout(profile)
    if steer_control == 'ackermann' and layout['steer_type'] != geo.LINKAGE:
        raise SteerModelError('ackermann_check_needs_linkage_steering')
    sensor_checks = _pose_checks(model, profile)
    mismatched = sorted(name for name, check in sensor_checks.items() if not check['matches'])
    if mismatched and not allow_sensor_mount_mismatch:
        raise SteerModelError('sensor_mount_not_applied:%s (the links stay at the pinned rover.yaml positions; the '
                              'profile differs by %s m; allow_sensor_mount_mismatch=True builds it with result flags)'
                              % (','.join(mismatched), ', '.join('%s %s' % (n, [round(d, 6) for d in
                                                                              sensor_checks[n]['difference_m']])
                                                                  for n in mismatched)))
    friction_source = 'profile' if wheel_mu_lat is None and wheel_mu_long is None else 'model_option'
    mu_lat = v('wheel_mu_lat') if wheel_mu_lat is None else float(wheel_mu_lat)
    mu_long = v('wheel_mu_long') if wheel_mu_long is None else float(wheel_mu_long)
    for name, value in (('wheel_mu_lat', mu_lat), ('wheel_mu_long', mu_long)):
        if not (math.isfinite(value) and value > 0.):
            raise SteerModelError('%s_must_be_positive' % name)
    removed = []
    for plugin in list(model.findall('plugin')):
        if _plugin_kind(plugin) in REMOVED_PLUGINS:
            removed.append(_plugin_kind(plugin))
            model.remove(plugin)
    wheels = [_set_wheel(model, wheel, layout['wheels'][wheel], profile, mu_lat, mu_long, wheel_collision)
              for wheel in WHEELS]
    first_wheel_link = _link(model, WHEELS[0])
    steering = [_add_steering(model, key, spec, profile, first_wheel_link) for key, spec in layout['steering'].items()]
    travel_up = PLACEHOLDERS['suspension_travel_m'][0] if suspension else 0.
    if front_axle_pivot:
        travel_up += pivot_rise_m(profile)
    other_links = planned_body_links(model, profile, steering, front_axle_pivot=front_axle_pivot, suspension=suspension)
    body = _set_body(model, profile, wheel_wells=body_wheel_wells, travel_up_m=travel_up, other_links=other_links)
    imu = _set_imu(model, profile)
    suspension_block = _add_suspension(model, profile, steering) if suspension else dict(enabled=False)
    pivot_block = _add_front_pivot(model, profile, steering) if front_axle_pivot else dict(enabled=False)
    mass_check = vehicle_mass_check(model, profile)
    if not mass_check['matches']:
        raise SteerModelError('vehicle_mass_differs_from_the_profile: links %g kg, profile %g kg'
                              % (mass_check['links_total_kg'], mass_check['profile_vehicle_mass_kg']))
    line = drive_model.dc_motor_line(profile, curve, battery_scale=battery_scale, tau_stall_nm=tau_stall_nm)
    driven = DRIVEN_WHEELS[v('drive_wheels')]
    steer_topic_map = steer_topics(layout['steer_type'], topic_prefix)
    drive_topic_map = wheel_topics(topic_prefix)
    cmd_max = float(v('gz_steer_joint_cmd_max_rad_s'))
    plugins = []
    if steer_control == 'position':
        for key, spec in layout['steering'].items():
            params = dict(joint_name=spec['joint'], use_velocity_commands=True, cmd_max=cmd_max, cmd_min=-cmd_max,
                          p_gain=PLACEHOLDERS['steer_position_p_gain'][0], i_gain=PLACEHOLDERS['steer_position_i_gain'][0],
                          d_gain=PLACEHOLDERS['steer_position_d_gain'][0], topic=steer_topic_map[key])
            if initial_steer_rad is not None and key in initial_steer_rad:
                angle = float(initial_steer_rad[key])
                if not spec['lower'] <= angle <= spec['upper']:
                    raise SteerModelError('initial_steer_outside_the_joint_limits:' + key)
                params['initial_position'] = angle
            _plugin(model, 'JointPositionController', **params)
            plugins.append(dict(kind='JointPositionController', joint=spec['joint'], topic=steer_topic_map[key],
                                use_velocity_commands=True, cmd_max=cmd_max))
    joint_friction = {}
    if steer_control == 'ackermann':
        g = geo.SteeringGeometry.from_profile(profile)
        limit = min(math.radians(x) for x in v('steer_mech_limit_deg').values())
        topic = ackermann_topic(topic_prefix)
        _plugin(model, 'AckermannSteering', left_joint=['left_rear_wheel_joint'], right_joint=['right_rear_wheel_joint'],
                left_steering_joint=STEER_JOINTS['left'], right_steering_joint=STEER_JOINTS['right'],
                kingpin_width=g.kingpin_width_m, steering_limit=limit, wheel_base=g.wheelbase_m,
                wheel_separation=g.track_rear_m, wheel_radius=g.wheel_radius_rear_m,
                steer_p_gain=PLACEHOLDERS['ackermann_steer_p_gain'][0], topic=topic, odom_topic=ACKERMANN_ODOM_TOPIC,
                tf_topic=ACKERMANN_TF_TOPIC, frame_id='ackermann_odom', child_frame_id='base_footprint')
        plugins.append(dict(kind='AckermannSteering', topic=topic, odom_topic=ACKERMANN_ODOM_TOPIC,
                            drive_joints=['left_rear_wheel_joint', 'right_rear_wheel_joint'], steering_limit_rad=limit,
                            steer_p_gain=PLACEHOLDERS['ackermann_steer_p_gain'][0], note='comparison only; wheel odometry'))
        for wheel in WHEELS:
            joint_friction[wheel] = PLACEHOLDERS['free_wheel_joint_friction_nm'][0]
    elif drive_mode == 'force_dc':
        for wheel in WHEELS:
            if wheel in driven:
                _plugin(model, 'JointController', joint_name=wheel + '_joint', use_force_commands=True,
                        p_gain=line.p_gain, i_gain=0., d_gain=0., i_max=0., i_min=0., cmd_max=line.cmd_max_nm,
                        cmd_min=line.cmd_min_nm, initial_velocity=0., topic=drive_topic_map[wheel])
                plugins.append(dict(kind='JointController', mode='force', joint=wheel + '_joint',
                                    topic=drive_topic_map[wheel], p_gain=line.p_gain, cmd_max=line.cmd_max_nm,
                                    cmd_min=line.cmd_min_nm))
                joint_friction[wheel] = line.coulomb_friction_nm
            else:
                joint_friction[wheel] = PLACEHOLDERS['free_wheel_joint_friction_nm'][0]
    else:
        for wheel in WHEELS:
            _plugin(model, 'JointController', joint_name=wheel + '_joint', use_force_commands=False,
                    initial_velocity=0., topic=drive_topic_map[wheel])
            plugins.append(dict(kind='JointController', mode='velocity', joint=wheel + '_joint',
                                topic=drive_topic_map[wheel]))
            joint_friction[wheel] = 0.
    for wheel in WHEELS:
        axis = _joint(model, wheel + '_joint').find('axis')
        dynamics = axis.find('dynamics')
        if dynamics is None:
            dynamics = _sub(axis, 'dynamics')
        _set(dynamics, 'friction', joint_friction[wheel])
        _set(dynamics, 'damping', 0.)
    for plugin in model.findall('plugin'):
        kind = _plugin_kind(plugin)
        if kind == 'OdometryPublisher':
            _set(plugin, 'odom_topic', odom_topic)
        elif kind == 'JointStatePublisher':
            _set(plugin, 'topic', joint_state_topic)
    wheel_inertia = _cylinder_inertia(v('wheel_mass_kg'), drive_wheel_radius(v), v('wheel_width_m'), 'y')[1]
    dt = 1. / float(v('gz_physics_hz'))
    manifest = dict(
        schema=SCHEMA, model_name=model.get('name'), profile=profile.result_label(),
        steer_type=layout['steer_type'], drive_mode='ackermann_plugin' if steer_control == 'ackermann' else drive_mode,
        steer_control=steer_control,
        self_locking_assumed=True, profile_steer_self_locking=v('steer_self_locking'),
        removed_plugins=removed, kept_plugins=list(KEPT_PLUGINS), plugins=plugins,
        steering=steering, wheels=wheels, body=body, imu=imu, suspension=suspension_block, front_axle_pivot=pivot_block,
        wheel_collision=wheel_collision,
        wheel_collision_choice=dict(shape=wheel_collision, source=collision_source, default=DEFAULT_WHEEL_COLLISION,
                                    visual='cylinder', decision=WHEEL_COLLISION_DECISION),
        wheel_friction=dict(mu_lat=mu_lat, mu_long=mu_long, fdir1='0 0 1', source=friction_source),
        mass_check=mass_check,
        sensor_link_checks=sensor_checks, result_flags=sensor_mount_flags(sensor_checks, imu),
        topics=dict(steer=steer_topic_map, drive=drive_topic_map if steer_control == 'position' else {},
                    ackermann=ackermann_topic(topic_prefix) if steer_control == 'ackermann' else None,
                    truth_odometry_gz=odom_topic, truth_joint_states_gz=joint_state_topic),
        drive=dict(curve=line.curve, driven_wheels=list(driven) if steer_control == 'position' else list(REAR_WHEELS),
                   lr_mode=v('drive_lr_mode'), omega0_rad_s=line.omega0_rad_s, tau_stall_nm=line.tau_stall_nm,
                   p_gain=line.p_gain, deadband_frac=line.deadband_frac, coulomb_friction_nm=line.coulomb_friction_nm,
                   r_omega0_m_s=line.r_omega0_m_s, battery_scale=line.battery_scale,
                   velocity_target_rule='duty_frac * omega0_rad_s',
                   stability_number=line.stability_number(wheel_inertia, dt),
                   stability_condition='p_gain * dt / J_wheel < 1', wheel_spin_inertia_kgm2=wheel_inertia,
                   physics_dt_s=dt),
        joint_friction_nm=joint_friction,
        placeholders={name: dict(value=value, provenance=tag, note=note)
                      for name, (value, tag, note) in PLACEHOLDERS.items()},
        profile_values={name: dict(value=profile.value(name), provenance=profile.provenance(name))
                        for name in PROFILE_NAMES},
        not_representable=['steering free mode (non-self-locking, zero output): F-SIM only',
                           'drive coast (EN off): F-SIM only (JointController cannot give zero torque)',
                           'self-aligning torque and tyre slip stiffness'])
    return manifest


def steer_world_sdf(sdf_text, profile=None, *, model_name=MODEL_NAME, **options):
    """Transform the ubts_rover model inside an SDF document. Returns (sdf_text, manifest)."""
    profile = load_profile() if profile is None else profile
    root = E.fromstring(sdf_text)
    model = root.find("world/model[@name='%s']" % model_name)
    if model is None:
        raise SteerModelError('model_not_found:' + model_name)
    manifest = steer_model_element(model, profile, **options)
    E.indent(root)
    text = E.tostring(root, encoding='unicode')
    manifest['sdf_sha256'] = hashlib.sha256(text.encode('utf-8')).hexdigest()
    return text, manifest


def describe_model(model):
    """Links, joints and plugins of a model element (for tests and reports)."""
    links = {}
    for link in model.findall('link'):
        links[link.get('name')] = dict(pose=list(_parse_pose(link)),
                                       sensors=[s.get('name') for s in link.findall('sensor')])
    joints = {}
    for joint in model.findall('joint'):
        axis = joint.find('axis')
        joints[joint.get('name')] = dict(
            type=joint.get('type'), parent=joint.findtext('parent'), child=joint.findtext('child'),
            axis=None if axis is None else [float(x) for x in axis.findtext('xyz').split()],
            lower=None if axis is None or axis.find('limit/lower') is None else float(axis.findtext('limit/lower')),
            upper=None if axis is None or axis.find('limit/upper') is None else float(axis.findtext('limit/upper')),
            effort=None if axis is None or axis.find('limit/effort') is None else float(axis.findtext('limit/effort')),
            friction=None if axis is None or axis.find('dynamics/friction') is None
            else float(axis.findtext('dynamics/friction')))
    plugins = []
    for plugin in model.findall('plugin'):
        params = {}
        for child in plugin:
            params.setdefault(child.tag, []).append(child.text)
        plugins.append(dict(kind=_plugin_kind(plugin), filename=plugin.get('filename'),
                            params={k: (vals[0] if len(vals) == 1 else vals) for k, vals in params.items()}))
    return dict(name=model.get('name'), links=links, joints=joints, plugins=plugins)


def _generated_rover_sdf():
    """generate_world.generate() of the pinned rover.yaml (sim/ubts must be on sys.path)."""
    from camera_profile import load_rover_config   # pinned module, imported only
    from generate_world import generate            # pinned module, imported only
    root = Path(__file__).resolve().parents[1]
    return generate(load_rover_config(root / 'rover.yaml'))


def main(argv=None):
    parser = argparse.ArgumentParser(description='Steered ubts_rover SDF from a generated world (design 8.3 P5).')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--input', type=Path, help='SDF file with world/model[@name=ubts_rover]')
    source.add_argument('--generate', action='store_true', help='use generate_world.generate() of rover.yaml')
    parser.add_argument('--output', type=Path, required=True, help='steered SDF file to write')
    parser.add_argument('--manifest', type=Path, help='manifest JSON (default: <output>.manifest.json)')
    parser.add_argument('--profile', type=Path, help='vehicle profile YAML (default: shipped placeholder)')
    parser.add_argument('--drive-mode', choices=DRIVE_MODES)
    parser.add_argument('--steer-control', choices=STEER_CONTROLS, default='position')
    parser.add_argument('--topic-prefix', default=DEFAULT_TOPIC_PREFIX)
    parser.add_argument('--wheel-collision', choices=WHEEL_COLLISION_SHAPES,
                        help='wheel collision shape (default: %s)' % DEFAULT_WHEEL_COLLISION)
    args = parser.parse_args(argv)
    profile = load_profile(args.profile)
    text = _generated_rover_sdf() if args.generate else args.input.read_text(encoding='utf-8')
    out, manifest = steer_world_sdf(text, profile, drive_mode=args.drive_mode, steer_control=args.steer_control,
                                    topic_prefix=args.topic_prefix, wheel_collision=args.wheel_collision)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(out.encode('utf-8'))
    target = args.manifest or args.output.with_name(args.output.name + '.manifest.json')
    target.write_bytes((json.dumps(manifest, indent=1, ensure_ascii=False, allow_nan=False) + '\n').encode('utf-8'))
    print(json.dumps(dict(output=str(args.output), manifest=str(target), sdf_sha256=manifest['sdf_sha256'],
                          drive_mode=manifest['drive_mode'], steer_type=manifest['steer_type'],
                          wheel_collision=manifest['wheel_collision'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
