"""Steered Gazebo model transform, scenes and probe helpers without Gazebo (design 8.3 P5).

Checks the SDF transform of steering/gz_model.py on the output of the pinned
generate_world.generate(): names, axes, per-wheel limits, parents, DiffDrive removed, the
controller plugins and their parameters from the profile, kept plugins, frames and links,
no _wheel_joint suffix on steering joints, wheel friction, the kinematic speed-mode and
AckermannSteering variants and the axle_pivot type; the W0 scene of steering/gz_assemble.py;
and the pure analysis helpers and scenario definitions of steering/gz_probe.py.

unittest style (also collected by pytest) so that it runs on Python 3.10 without pytest.
Files are written only to temporary folders outside the work tree. Placeholder values only.
"""
from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as E

import numpy as np

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from steering import drive as drive_model          # noqa: E402
from steering import geometry as geo               # noqa: E402
from steering import gz_assemble                   # noqa: E402
from steering import gz_model as gm                # noqa: E402
from steering import gz_probe as gp                # noqa: E402
from steering.vehicle_profile import load_profile  # noqa: E402

PROFILE = load_profile()


def generated_world():
    return E.fromstring(gm._generated_rover_sdf())


def rover(root):
    return root.find("world/model[@name='ubts_rover']")


def steered(profile=PROFILE, **options):
    root = generated_world()
    model = rover(root)
    manifest = gm.steer_model_element(model, profile, **options)
    return root, model, manifest


def plugins(model, kind):
    return [p for p in model.findall('plugin') if (p.get('name') or '').endswith('::' + kind)]


def param(plugin, tag):
    return plugin.findtext(tag)


class TempDirCase(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix='ubts_steer_gz_test_'))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)


class TestSteeredModelStructure(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original = rover(generated_world())
        cls.root, cls.model, cls.manifest = steered()
        cls.desc = gm.describe_model(cls.model)

    def test_model_name_origin_and_kept_links(self):
        self.assertEqual(self.model.get('name'), 'ubts_rover')
        self.assertEqual(self.model.findtext('pose'), self.original.findtext('pose'))
        for name in gm.KEPT_LINKS:
            new, old = self.model.find("link[@name='%s']" % name), self.original.find("link[@name='%s']" % name)
            self.assertIsNotNone(new, name)
            self.assertEqual(new.findtext('pose'), old.findtext('pose'), name)
        for name in ('laser', 'imu_link', 'camera_link'):
            joint = self.model.find("joint[@name='%s_fixed']" % name)
            self.assertEqual((joint.get('type'), joint.findtext('parent')), ('fixed', 'base_link'))
        self.assertEqual(self.model.find("link[@name='laser']/sensor").get('type'), 'gpu_lidar')
        self.assertIsNotNone(self.model.find("link[@name='camera_link']/sensor[@type='camera']"))

    def test_wheel_joints_kept_and_front_wheels_reparented(self):
        joints = self.desc['joints']
        for name in gm.WHEEL_JOINTS:
            self.assertIn(name, joints)
            self.assertEqual(joints[name]['type'], 'revolute')
            self.assertEqual(joints[name]['axis'], [0., 1., 0.])
        self.assertEqual(joints['left_front_wheel_joint']['parent'], 'left_front_steering_link')
        self.assertEqual(joints['right_front_wheel_joint']['parent'], 'right_front_steering_link')
        self.assertEqual(joints['left_rear_wheel_joint']['parent'], 'base_link')
        self.assertEqual(joints['right_rear_wheel_joint']['parent'], 'base_link')

    def test_steering_joints_axis_limits_and_names(self):
        limits = PROFILE.value('steer_mech_limit_deg')
        joints = self.desc['joints']
        for side in ('left', 'right'):
            name = gm.STEER_JOINTS[side]
            self.assertFalse(name.endswith('_wheel_joint'))
            joint = joints[name]
            # default model: the steering joints hang from the passive front axle roll pivot
            self.assertEqual((joint['type'], joint['parent'], joint['child']), ('revolute', gm.PIVOT_LINK,
                                                                                 gm.STEER_LINKS[side]))
            self.assertEqual(joint['axis'], [0., 0., 1.])
            self.assertAlmostEqual(joint['lower'], -math.radians(limits['%s_wheel_toward_right' % side]), places=12)
            self.assertAlmostEqual(joint['upper'], math.radians(limits['%s_wheel_toward_left' % side]), places=12)
            self.assertEqual(joint['effort'], gm.PLACEHOLDERS['steer_joint_effort_limit_nm'][0])
        for name in joints:
            if 'steering' in name:
                self.assertFalse(name.endswith('_wheel_joint'), name)

    def test_steering_links_at_the_kingpins_with_inertia_floor(self):
        front_x = PROFILE.value('rear_axle_x_in_base_m') + PROFILE.value('wheelbase_m')
        w_k, r_f = PROFILE.value('kingpin_width_m'), PROFILE.value('wheel_radius_front_m')
        floor = gm.PLACEHOLDERS['steer_link_inertia_min_kgm2'][0]
        for side, sign in (('left', 1.), ('right', -1.)):
            link = self.model.find("link[@name='%s']" % gm.STEER_LINKS[side])
            pose = [float(v) for v in link.findtext('pose').split()]
            self.assertEqual(len(pose), 6)
            for got, want in zip(pose, (front_x, sign * w_k / 2., r_f, 0., 0., 0.)):
                self.assertAlmostEqual(got, want, places=12)
            self.assertEqual(float(link.findtext('inertial/mass')), PROFILE.value('steer_link_mass_kg'))
            for key in ('ixx', 'iyy', 'izz'):
                self.assertGreaterEqual(float(link.findtext('inertial/inertia/' + key)), floor)
            self.assertIsNone(link.find('collision'))

    def test_diffdrive_removed_and_truth_plugins_kept(self):
        self.assertEqual(plugins(self.model, 'DiffDrive'), [])
        self.assertEqual(self.manifest['removed_plugins'], ['DiffDrive'])
        odom = plugins(self.model, 'OdometryPublisher')
        self.assertEqual(len(odom), 1)
        self.assertEqual(param(odom[0], 'odom_topic'), '/sim/ground_truth')
        self.assertEqual(param(odom[0], 'robot_base_frame'), 'base_footprint')
        self.assertEqual(param(odom[0], 'odom_frame'), 'world')
        joint_states = plugins(self.model, 'JointStatePublisher')
        self.assertEqual(len(joint_states), 1)
        self.assertEqual(param(joint_states[0], 'topic'), '/sim/truth/joint_states')

    def test_steering_controllers_in_velocity_command_mode(self):
        controllers = plugins(self.model, 'JointPositionController')
        self.assertEqual(len(controllers), 2)
        cmd_max = PROFILE.value('gz_steer_joint_cmd_max_rad_s')
        topics = {}
        for plugin in controllers:
            self.assertEqual(param(plugin, 'use_velocity_commands'), 'true')
            self.assertAlmostEqual(float(param(plugin, 'cmd_max')), cmd_max)
            self.assertAlmostEqual(float(param(plugin, 'cmd_min')), -cmd_max)
            self.assertEqual(plugin.get('filename'), 'gz-sim-joint-position-controller-system')
            topics[param(plugin, 'joint_name')] = param(plugin, 'topic')
        self.assertEqual(topics, {'left_front_steering_joint': '/sim/act/steer_left_cmd',
                                  'right_front_steering_joint': '/sim/act/steer_right_cmd'})

    def test_drive_controllers_are_the_dc_motor_line(self):
        line = drive_model.dc_motor_line(PROFILE)
        controllers = plugins(self.model, 'JointController')
        self.assertEqual(len(controllers), 4)          # 4wd_lr_pairs
        for plugin in controllers:
            joint = param(plugin, 'joint_name')
            wheel = joint[:-len('_joint')]
            self.assertEqual(param(plugin, 'use_force_commands'), 'true')
            self.assertAlmostEqual(float(param(plugin, 'p_gain')), line.tau_stall_nm / line.omega0_rad_s, places=12)
            self.assertAlmostEqual(float(param(plugin, 'cmd_max')), line.tau_stall_nm)
            self.assertAlmostEqual(float(param(plugin, 'cmd_min')), -line.tau_stall_nm)
            self.assertEqual(float(param(plugin, 'i_gain')), 0.)
            self.assertEqual(float(param(plugin, 'd_gain')), 0.)
            self.assertEqual(param(plugin, 'topic'), '/sim/act/%s_cmd' % wheel)
            friction = self.desc['joints'][joint]['friction']
            self.assertAlmostEqual(friction, line.deadband_frac * line.tau_stall_nominal_nm, places=12)
        self.assertLess(self.manifest['drive']['stability_number'], 1.)
        self.assertAlmostEqual(self.manifest['drive']['stability_number'], .3726, places=3)   # profile v0.1 (tau_stall 4.4); v0 gave 0.1355

    def test_wheels_from_the_profile(self):
        mu_lat, mu_long = PROFILE.value('wheel_mu_lat'), PROFILE.value('wheel_mu_long')
        inertia = PROFILE.value('wheel_inertia_kgm2')
        for wheel in gm.WHEELS:
            link = self.model.find("link[@name='%s']" % wheel)
            friction = link.find('collision/surface/friction/ode')
            self.assertEqual((float(friction.findtext('mu')), float(friction.findtext('mu2'))), (mu_lat, mu_long))
            self.assertEqual(friction.findtext('fdir1'), '0 0 1')
            self.assertAlmostEqual(float(link.findtext('inertial/inertia/iyy')), inertia, places=12)
            radius = PROFILE.value('wheel_radius_front_m' if 'front' in wheel else 'wheel_radius_rear_m')
            # default collision: a sphere of the wheel radius (DECISIONS_KO R13 decision 1); the visual is the cylinder
            self.assertIsNone(link.find('collision/geometry/cylinder'))
            self.assertEqual(float(link.findtext('collision/geometry/sphere/radius')), radius)
            self.assertEqual(float(link.findtext('visual/geometry/cylinder/radius')), radius)
            self.assertEqual(float(link.findtext('visual/geometry/cylinder/length')), PROFILE.value('wheel_width_m'))
            pose = [float(v) for v in link.findtext('pose').split()]
            self.assertAlmostEqual(pose[2], radius, places=12)
        # the placeholder profile equals rover.yaml: wheel centres do not move
        for row in self.manifest['wheels']:
            self.assertEqual(row['moved_m'], [0., 0., 0.])
            self.assertEqual((row['collision_shape'], row['visual_shape']), ('sphere', 'cylinder'))

    def test_body_and_imu_from_the_profile(self):
        base = self.model.find("link[@name='base_link']")
        # review F6: base_link = mass_kg minus the other body links, so the vehicle total is the F-SIM mass (30 kg)
        others = self.manifest['body']['other_body_link_masses_kg']
        self.assertEqual(sorted(others), sorted([gm.STEER_LINKS['left'], gm.STEER_LINKS['right'], gm.PIVOT_LINK,
                                                 'laser', 'imu_link', 'camera_link']))
        base_mass = PROFILE.value('mass_kg') - sum(others.values())
        self.assertAlmostEqual(float(base.findtext('inertial/mass')), base_mass, places=12)
        total = sum(float(link.findtext('inertial/mass')) for link in self.model.findall('link')
                    if link.find('inertial/mass') is not None)
        self.assertAlmostEqual(total, drive_model.vehicle_mass_kg(PROFILE), places=12)
        check = self.manifest['mass_check']
        self.assertTrue(check['matches'], check)
        self.assertAlmostEqual(check['links_total_kg'], 30., places=12)
        self.assertLess(check['body_cog_error_m'], 1e-12)
        for axis, want in ((0, PROFILE.value('cog_x_in_base_m')), (2, PROFILE.value('cog_z_m'))):
            self.assertAlmostEqual(check['body_cog_model_m'][axis], want, places=12)
        # the body boxes (deck + lower segments with wheel wells) fill the profile envelope and nothing outside it
        length, width, height = PROFILE.value('gz_body_box_m')
        boxes = self.manifest['body']['boxes']
        self.assertEqual(boxes[0]['name'], 'body')
        self.assertEqual(boxes[0]['size'][:2], [length, width])           # the deck spans the full top
        for box in boxes:
            for axis, extent in enumerate((length, width, height)):
                self.assertLessEqual(abs(box['centre'][axis]) + box['size'][axis] / 2., extent / 2. + 1e-12)
            visual = base.find("visual[@name='%s']" % box['name'])
            collision = base.find("collision[@name='%s_collision']" % box['name'])
            self.assertEqual(visual.findtext('geometry/box/size'), collision.findtext('geometry/box/size'))
        top = max(b['centre'][2] + b['size'][2] / 2. for b in boxes)
        bottom = min(b['centre'][2] - b['size'][2] / 2. for b in boxes)
        self.assertAlmostEqual(top - bottom, height, places=12)
        inertia = [float(base.findtext('inertial/inertia/' + k)) for k in ('ixx', 'iyy', 'izz')]
        self.assertAlmostEqual(inertia[2], base_mass * (length ** 2 + width ** 2) / 12., places=12)
        imu = self.model.find("link[@name='imu_link']/sensor[@type='imu']")
        self.assertEqual(float(imu.findtext('update_rate')), float(PROFILE.value('imu_rate_hz')))
        self.assertEqual([float(v) for v in imu.findtext('pose').split()], [0.] * 6)
        checks = self.manifest['sensor_link_checks']
        self.assertTrue(checks['laser']['matches'] and checks['camera_link']['matches'])

    def test_every_pose_has_six_numbers_and_the_sdf_round_trips(self):
        text = E.tostring(self.root, encoding='unicode')
        again = E.fromstring(text)
        for pose in again.iter('pose'):
            values = [float(v) for v in pose.text.split()]
            self.assertEqual(len(values), 6, pose.text)
            self.assertTrue(all(math.isfinite(v) for v in values))
        json.dumps(self.manifest, allow_nan=False)

    def test_manifest_records_profile_and_placeholders(self):
        label = self.manifest['profile']
        self.assertEqual(label['profile_sha256'], PROFILE.sha256)
        self.assertFalse(label['field_validated'])
        self.assertIn('steer_link_inertia_min_kgm2', self.manifest['placeholders'])
        self.assertEqual(self.manifest['profile_values']['drive_tau_stall_nm']['provenance'], 'SPEC_PENDING')
        self.assertTrue(self.manifest['self_locking_assumed'])

    def test_transform_twice_or_on_a_wrong_model_is_refused(self):
        model = copy.deepcopy(self.model)
        with self.assertRaises(gm.SteerModelError):
            gm.steer_model_element(model, PROFILE)
        broken = copy.deepcopy(self.original)
        broken.remove(broken.find("link[@name='laser']"))
        with self.assertRaises(gm.SteerModelError):
            gm.steer_model_element(broken, PROFILE)
        with self.assertRaises(gm.SteerModelError):
            gm.steer_model_element(copy.deepcopy(self.original), PROFILE, drive_mode='diff_drive')


class TestModelVariants(unittest.TestCase):
    def test_kinematic_speed_mode(self):
        _, model, manifest = steered(drive_mode='kinematic_velocity')
        controllers = plugins(model, 'JointController')
        self.assertEqual(len(controllers), 4)
        for plugin in controllers:
            self.assertEqual(param(plugin, 'use_force_commands'), 'false')
            self.assertIsNone(plugin.find('p_gain'))
        desc = gm.describe_model(model)
        for name in gm.WHEEL_JOINTS:
            self.assertEqual(desc['joints'][name]['friction'], 0.)
        self.assertEqual(len(plugins(model, 'JointPositionController')), 2)
        self.assertEqual(manifest['drive_mode'], 'kinematic_velocity')

    def test_ackermann_cross_check_variant(self):
        view = PROFILE.with_overrides(gz_assemble.ACKERMANN_CHECK_OVERRIDES, label='test')
        _, model, manifest = steered(view, steer_control='ackermann')
        self.assertEqual(plugins(model, 'JointPositionController'), [])
        self.assertEqual(plugins(model, 'JointController'), [])
        acker = plugins(model, 'AckermannSteering')
        self.assertEqual(len(acker), 1)
        a = acker[0]
        self.assertEqual(param(a, 'left_steering_joint'), 'left_front_steering_joint')
        self.assertEqual(param(a, 'right_steering_joint'), 'right_front_steering_joint')
        self.assertEqual([e.text for e in a.findall('left_joint')], ['left_rear_wheel_joint'])
        self.assertEqual([e.text for e in a.findall('right_joint')], ['right_rear_wheel_joint'])
        self.assertAlmostEqual(float(param(a, 'kingpin_width')), view.value('kingpin_width_m'))
        self.assertAlmostEqual(float(param(a, 'wheel_base')), view.value('wheelbase_m'))
        self.assertAlmostEqual(float(param(a, 'steering_limit')), math.radians(28.))
        self.assertEqual(param(a, 'topic'), '/sim/act/ackermann_cmd_vel')
        self.assertTrue(param(a, 'odom_topic').startswith('/sim/truth/'))
        self.assertEqual(manifest['drive_mode'], 'ackermann_plugin')

    def test_axle_pivot_type(self):
        view = PROFILE.with_overrides({'steer_type': 'axle_pivot', 'steer_mech_limit_deg': {
            'left_wheel_toward_left': 30., 'left_wheel_toward_right': 26., 'right_wheel_toward_left': 27.,
            'right_wheel_toward_right': 29.}}, label='test_axle')
        _, model, manifest = steered(view)
        desc = gm.describe_model(model)
        joint = desc['joints'][gm.AXLE_STEER_JOINT]
        self.assertAlmostEqual(joint['upper'], math.radians(27.))
        self.assertAlmostEqual(joint['lower'], -math.radians(26.))
        self.assertEqual(desc['joints']['left_front_wheel_joint']['parent'], gm.AXLE_STEER_LINK)
        self.assertEqual(desc['joints']['right_front_wheel_joint']['parent'], gm.AXLE_STEER_LINK)
        self.assertNotIn('left_front_steering_joint', desc['joints'])
        controllers = plugins(model, 'JointPositionController')
        self.assertEqual([param(p, 'topic') for p in controllers], ['/sim/act/steer_axle_cmd'])
        with self.assertRaises(gm.SteerModelError):
            steered(view, steer_control='ackermann')

    def test_asymmetric_limits_and_drive_overrides(self):
        view = PROFILE.with_overrides({'steer_mech_limit_deg': {
            'left_wheel_toward_left': 32., 'left_wheel_toward_right': 24., 'right_wheel_toward_left': 22.,
            'right_wheel_toward_right': 34.}, 'drive_wheels': 'rear2', 'wheel_mu_lat': .6}, label='test_limits')
        _, model, manifest = steered(view, tau_stall_nm=3.2, battery_scale=.9)
        desc = gm.describe_model(model)
        left, right = desc['joints']['left_front_steering_joint'], desc['joints']['right_front_steering_joint']
        self.assertAlmostEqual(left['upper'], math.radians(32.))
        self.assertAlmostEqual(left['lower'], -math.radians(24.))
        self.assertAlmostEqual(right['upper'], math.radians(22.))
        self.assertAlmostEqual(right['lower'], -math.radians(34.))
        controllers = plugins(model, 'JointController')
        self.assertEqual(sorted(param(p, 'joint_name') for p in controllers),
                         ['left_rear_wheel_joint', 'right_rear_wheel_joint'])
        line = drive_model.dc_motor_line(view, tau_stall_nm=3.2, battery_scale=.9)
        self.assertAlmostEqual(float(param(controllers[0], 'cmd_max')), .9 * 3.2)
        self.assertAlmostEqual(float(param(controllers[0], 'p_gain')), line.p_gain)
        self.assertEqual(desc['joints']['left_front_wheel_joint']['friction'], 0.)
        self.assertAlmostEqual(desc['joints']['left_rear_wheel_joint']['friction'], line.deadband_frac * 3.2)
        mu = model.find("link[@name='left_front_wheel']/collision/surface/friction/ode/mu")
        self.assertEqual(float(mu.text), .6)

    def test_steering_axis_tilt(self):
        self.assertEqual(gm.steering_axis('left', 0., 0.), (0., 0., 1.))
        x, y, z = gm.steering_axis('left', math.radians(5.), math.radians(8.))
        self.assertAlmostEqual(x * x + y * y + z * z, 1.)
        self.assertLess(x, 0.)       # caster: top rearward
        self.assertLess(y, 0.)       # left kingpin top toward the centre
        self.assertGreater(gm.steering_axis('right', 0., math.radians(8.))[1], 0.)
        view = PROFILE.with_overrides({'caster_deg': 4., 'kingpin_inclination_deg': 6.}, label='tilt')
        _, model, _ = steered(view)
        axis = gm.describe_model(model)['joints']['left_front_steering_joint']['axis']
        self.assertAlmostEqual(axis[0], -math.tan(math.radians(4.)) / math.sqrt(
            1. + math.tan(math.radians(4.)) ** 2 + math.tan(math.radians(6.)) ** 2), places=9)

    def test_imu_mount_from_the_profile(self):
        view = PROFILE.with_overrides({'imu_mount_rpy_deg': {'roll': 2., 'pitch': -3., 'yaw': 0., 'yaw_sign': -1},
                                       'imu_rate_hz': 20}, label='imu')
        _, model, manifest = steered(view)
        imu = model.find("link[@name='imu_link']/sensor[@type='imu']")
        pose = [float(v) for v in imu.findtext('pose').split()]
        self.assertAlmostEqual(pose[3], math.radians(2.) + math.pi)
        self.assertAlmostEqual(pose[4], math.radians(-3.))
        self.assertEqual(float(imu.findtext('update_rate')), 20.)

    def test_initial_steer_and_topic_prefix(self):
        _, model, manifest = steered(topic_prefix='/probe/v1/act', initial_steer_rad={'left': .1, 'right': .08})
        for plugin in plugins(model, 'JointPositionController'):
            self.assertTrue(param(plugin, 'topic').startswith('/probe/v1/act/'))
            self.assertIn(float(param(plugin, 'initial_position')), (.1, .08))
        with self.assertRaises(gm.SteerModelError):
            steered(initial_steer_rad={'left': 1.0})


class TestKinematicWheelSpeeds(unittest.TestCase):
    def setUp(self):
        self.g = geo.SteeringGeometry.from_profile(PROFILE.with_overrides({'steer_ackermann_ratio': 1.}, label='a1'))

    def test_straight(self):
        speeds = gm.kinematic_wheel_speeds(self.g, 0., 0., .5)
        for name, value in speeds.items():
            self.assertAlmostEqual(value, .5 / .14, places=12)
        reverse = gm.kinematic_wheel_speeds(self.g, 0., 0., -.3)
        self.assertTrue(all(v < 0. for v in reverse.values()))

    def test_turning_speeds_follow_the_icr(self):
        g = self.g
        for deg in (5., 10., 20., -15.):
            delta = math.radians(deg)
            left, right = geo.ideal_ackermann_angles(delta, g.wheelbase_m, g.kingpin_width_m)
            speeds = gm.kinematic_wheel_speeds(g, left, right, .4)
            radius = g.wheelbase_m / math.tan(delta)
            omega = .4 / radius
            self.assertAlmostEqual(speeds['left_rear_wheel_joint'] * g.wheel_radius_rear_m,
                                   abs(omega) * abs(radius - g.track_rear_m / 2.), places=12)
            self.assertAlmostEqual(speeds['right_rear_wheel_joint'] * g.wheel_radius_rear_m,
                                   abs(omega) * abs(radius + g.track_rear_m / 2.), places=12)
            # each front wheel rolls perpendicular to the line from the ICR to its centre
            for side, angle in ((geo.LEFT, left), (geo.RIGHT, right)):
                cx, cy = g.front_wheel_centre(side, angle)
                rx, ry = cx - g.rear_axle_x_in_base_m, cy - radius
                heading = (math.cos(angle), math.sin(angle))
                self.assertAlmostEqual(heading[0] * rx + heading[1] * ry, 0., places=9)
            inner = 'left_front_wheel_joint' if deg > 0 else 'right_front_wheel_joint'
            outer = 'right_front_wheel_joint' if deg > 0 else 'left_front_wheel_joint'
            self.assertLess(speeds[inner], speeds[outer])


class TestW0Scene(TempDirCase):
    def test_make_w0_from_the_generated_world(self):
        world = generated_world().find('world')
        summary = gz_assemble.make_w0(world, PROFILE)
        names = [m.get('name') for m in world.findall('model')]
        self.assertFalse(any(n.startswith('tree_') for n in names))
        for name in gz_assemble.W0_REMOVED_MODELS:
            self.assertNotIn(name, names)
        self.assertEqual(sum(n.startswith('w0_pillar_') for n in names), len(gz_assemble.W0_PILLARS_XY))
        ground = world.find("model[@name='ground']")
        self.assertEqual(ground.findtext('.//geometry/box/size'), '200 200 0.2')
        self.assertEqual(float(ground.findtext('.//surface/friction/ode/mu')), PROFILE.value('ground_mu'))
        self.assertEqual(float(ground.findtext('.//surface/friction/ode/mu2')), PROFILE.value('ground_mu'))
        self.assertGreater(summary['removed_models'], 30)
        for x, y in gz_assemble.W0_PILLARS_XY:
            self.assertGreater(math.hypot(x, y), 20.)      # outside the A1 circles (2 R <= 17.4 m)

    def test_build_w0_force_scene(self):
        manifest = gz_assemble.build_scene('w0_force', self.tmp / 'w0')
        sdf = self.tmp / 'w0' / 'world.sdf'
        text = sdf.read_text(encoding='utf-8')
        import hashlib
        self.assertEqual(hashlib.sha256(text.encode('utf-8')).hexdigest(), manifest['sdf_sha256'])
        root = E.fromstring(text)
        world = root.find('world')
        self.assertEqual(world.get('name'), 'ubts_orchard')
        self.assertAlmostEqual(float(world.findtext('physics/max_step_size')), 1. / 120.)
        model = world.find("model[@name='ubts_rover']")
        self.assertIsNone(model.find("link[@name='laser']/sensor"))            # embree backend
        self.assertIsNotNone(model.find("link[@name='camera_link']/sensor[@type='camera']"))
        self.assertEqual(plugins(model, 'DiffDrive'), [])
        self.assertEqual(len(plugins(model, 'JointPositionController')), 2)
        self.assertTrue(any((p.get('name') or '').endswith('::Sensors') for p in world.findall('plugin')))
        self.assertEqual(manifest['world'], 'W0')
        self.assertFalse(manifest['field_validated'])
        saved = json.loads((self.tmp / 'w0' / 'scene_manifest.json').read_text(encoding='utf-8'))
        self.assertEqual(saved['sdf_sha256'], manifest['sdf_sha256'])
        self.assertFalse((self.tmp / 'w0' / 'scene_manifest.json').read_bytes().count(b'\r\n'))
        for name in ('world.sdf', 'world_metadata.json', 'scene_manifest.json'):
            self.assertEqual(gz_assemble.gnss_lint((self.tmp / 'w0' / name).read_text(encoding='utf-8')), [], name)

    def test_gnss_origin_is_stripped(self):
        root = E.fromstring('<sdf><world name="w"><spherical_coordinates><latitude_deg>1</latitude_deg>'
                            '</spherical_coordinates><model name="m"><spherical_coordinates/></model></world></sdf>')
        world = root.find('world')
        self.assertTrue(gz_assemble.gnss_lint(E.tostring(root, encoding='unicode')))
        self.assertEqual(gz_assemble.strip_gnss_world(world), 2)
        self.assertEqual(gz_assemble.gnss_lint(E.tostring(root, encoding='unicode')), [])
        clean, removed = gz_assemble.strip_gnss_metadata(
            {'config': {'latitude': 1., 'longitude': 2., 'elevation': 3., 'rows': 6}, 'trees': [{'x': 1.}]})
        self.assertEqual(clean, {'config': {'rows': 6}, 'trees': [{'x': 1.}]})
        self.assertEqual(sorted(removed), ['config.elevation', 'config.latitude', 'config.longitude'])
        self.assertEqual(gz_assemble.gnss_lint(json.dumps(clean)), [])

    def test_scene_profiles_and_camera_removal(self):
        kin = gz_assemble.scene_profile('w0_kinematic', PROFILE)
        self.assertEqual(kin.value('steer_ackermann_ratio'), 1.)
        self.assertEqual((kin.value('wheel_mu_lat'), kin.value('wheel_mu_long')), (1., 1.))
        self.assertEqual(kin.value('gz_drive_mode'), 'kinematic_velocity')
        self.assertIs(gz_assemble.scene_profile('w0_force', PROFILE), PROFILE)
        manifest = gz_assemble.build_scene('w0_kinematic', self.tmp / 'k', camera=False)
        self.assertEqual(manifest['renderer']['cameras_removed'], 1)
        self.assertEqual(manifest['model']['drive_mode'], 'kinematic_velocity')
        with self.assertRaises(gz_assemble.SceneError):
            gz_assemble.build_scene('w9', self.tmp / 'x')

    @unittest.skipUnless(os.environ.get('UBTS_STEER_GZ_W1_TEST') == '1',
                         'W1 assembly writes the original orchard meshes under /root/.gz; run in the SIM container '
                         'with UBTS_STEER_GZ_W1_TEST=1')
    def test_build_w1_force_scene(self):
        manifest = gz_assemble.build_scene('w1_force', self.tmp / 'w1')
        self.assertEqual(manifest['world'], 'W1')
        root = E.parse(str(self.tmp / 'w1' / 'world.sdf')).getroot()
        self.assertEqual(len(plugins(root.find("world/model[@name='ubts_rover']"), 'JointPositionController')), 2)


class TestProbeHelpers(TempDirCase):
    def test_fit_circle_and_rates(self):
        t = np.linspace(0., 20., 400)
        r, w = 4.31, .07
        x, y = 3. + r * np.sin(w * t), -1. + r * (1. - np.cos(w * t))
        fit = gp.fit_circle(x, y)
        self.assertAlmostEqual(fit['radius'], r, places=9)
        self.assertAlmostEqual(fit['cx'], 3., places=9)
        self.assertAlmostEqual(fit['arc_deg'], math.degrees(w * 20.), places=6)
        self.assertAlmostEqual(gp.yaw_rate_fit(t, np.arctan2(np.sin(w * t), np.cos(w * t))), w, places=12)
        self.assertAlmostEqual(gp.path_speed(t, x, y), r * w, places=3)

    def test_effective_bicycle_fit_recovers_known_values(self):
        l_eff, k = .80, .03
        td = np.tan(np.radians([5., 10., 20., 5., 10., 20.]))
        v = np.array([.3, .3, .3, .5, .5, .5])
        kappa = td / (l_eff + k * v * v)
        fit = gp.fit_effective_bicycle(td, v, kappa, .76)
        self.assertAlmostEqual(fit['L_eff_m'], l_eff, places=10)
        self.assertAlmostEqual(fit['K_v2_s2_m'], k, places=9)
        self.assertAlmostEqual(fit['eta'], .76 / .80, places=10)
        self.assertLess(fit['max_abs_curvature_rel_residual'], 1e-10)

    def test_dc_line_stop_drift_and_rtf(self):
        line = gp.fit_dc_line([.2, .4, .7, 1.], [.937 * (d - .048) for d in (.2, .4, .7, 1.)])
        self.assertAlmostEqual(line['r_omega0_m_s'], .937, places=9)
        self.assertAlmostEqual(line['d0'], .048, places=9)
        t = np.arange(0., 3., .01)
        v = np.where(t < 1., .9, np.maximum(0., .9 - 2. * (t - 1.)))
        x = np.concatenate([[0.], np.cumsum(v[:-1] * .01)])
        stop = gp.stop_distance(t, x, np.zeros_like(t), v, 1.)
        self.assertTrue(stop['stopped'])
        self.assertAlmostEqual(stop['distance_m'], .81 / 4., delta=.02)
        drift = gp.standstill_drift(t, np.full_like(t, 2.), np.zeros_like(t) + .004 * t, np.zeros_like(t), .5, 2.5)
        self.assertAlmostEqual(drift['max_position_change_m'], .008, places=9)
        windows = gp.rtf_windows(np.arange(0., 60., .1) * .9, np.arange(0., 60., .1))
        self.assertTrue(all(abs(w - .9) < 1e-9 for w in windows))

    def test_quaternion_helpers_and_clean(self):
        half = math.radians(30.) / 2.
        q = (math.cos(half), 0., 0., math.sin(half))
        x, y, z = gp.quaternion_rotate(q, (1., 0., 0.32))
        self.assertAlmostEqual(x, math.cos(math.radians(30.)))
        self.assertAlmostEqual(z, .32)
        self.assertAlmostEqual(gp.quaternion_rpy(q)[2], math.radians(30.))
        self.assertEqual(gp._clean({'a': float('nan'), 'b': [np.float64(1.5), np.inf], 'c': np.array([1, 2])}),
                         {'a': None, 'b': [1.5, None], 'c': [1, 2]})

    def test_group_definitions(self):
        for group in gp.GROUPS:
            try:
                definition = gp.group_definition(group, PROFILE)
            except FileNotFoundError as exc:     # a12_w2 builds a catalogue field world: needs the 10/10 data
                if group == 'a12_w2':
                    continue                     # (UBTS_STEER_PATHS not configured, e.g. a container without data)
                raise AssertionError('%s: %s' % (group, exc))
            names = [v['name'] for v in definition['vehicles']]
            self.assertEqual(len(names), len(set(names)), group)
            self.assertIn(definition['scene'], gz_assemble.SCENES)
            positions = [(v['x'], v['y']) for v in definition['vehicles']]
            for i, a in enumerate(positions):
                for b in positions[i + 1:]:
                    self.assertGreaterEqual(math.hypot(a[0] - b[0], a[1] - b[1]), 10., group)
        with self.assertRaises(ValueError):
            gp.group_definition('a99', PROFILE)

    def test_probe_world_has_one_vehicle_per_spec_and_no_sensors(self):
        definition = gp.group_definition('a25_l0', PROFILE)
        path, ctx, manifests = gp.build_probe_world(definition, self.tmp / 'probe', PROFILE)
        world = E.parse(str(path)).getroot().find('world')
        models = {m.get('name'): m for m in world.findall('model')}
        self.assertNotIn('ubts_rover', models)
        for spec in definition['vehicles']:
            model = models[spec['name']]
            topics = [param(p, 'topic') for p in plugins(model, 'JointPositionController')]
            self.assertTrue(all(t.startswith('/probe/%s/act/' % spec['name']) for t in topics))
        self.assertFalse(any((p.get('name') or '').endswith('::Sensors') for p in world.findall('plugin')))
        self.assertEqual(list(world.iter('sensor')), [])
        self.assertEqual(world.findtext('physics/real_time_factor'), '0')
        self.assertAlmostEqual(ctx.dt, .008333333, places=12)
        self.assertAlmostEqual(ctx.duty_for_speed(.4), ctx.line.deadband_frac + .4 / ctx.line.r_omega0_m_s)

    def test_lockstep_controllers_without_gazebo(self):
        definition = gp.group_definition('a25_l0', PROFILE)
        path, ctx, manifests = gp.build_probe_world(definition, self.tmp / 'probe', PROFILE)
        obs = dict(speed=0., wheel_velocity={w + '_joint': 0. for w in gm.WHEELS}, steer_position={})
        controller = definition['vehicles'][0]['factory'](ctx)
        try:
            for k in range(1, 400):
                cmd = controller.command(k + 1, (k + 1) * ctx.dt, obs)
            left, right = cmd['steer']['left'], cmd['steer']['right']
            self.assertEqual(controller.steps, 400)
            # 20 deg step at 2 s, standstill rate 15 deg/s: at 3.33 s the equivalent angle is still rate limited
            self.assertGreater(left, right)
            limits = manifests[definition['vehicles'][0]['name']]['steering']
            self.assertTrue(all(item['lower_rad'] <= cmd['steer'][item['key']] <= item['upper_rad'] for item in limits))
        finally:
            controller.close()
        program = gp.DriveProgram(ctx, 1., .5, 'release', 1.)
        duties = []
        for k in range(300):
            targets = program(k * ctx.dt, obs)
            duties.append(next(iter(targets.values())) / ctx.line.omega0_rad_s)
        self.assertAlmostEqual(duties[100], 1., places=9)
        self.assertAlmostEqual(duties[121] - duties[120], -ctx.ramp_frac_per_s * ctx.dt, places=9)
        self.assertEqual(duties[-1], 0.)


class TestModelFixes(unittest.TestCase):
    """Defects found by the joint direct-drive demo (p5_demo) and their fixes."""

    def test_generated_body_box_overlaps_the_wheels_and_the_wheel_wells_clear_them(self):
        _, rigid_box, _ = steered(body_wheel_wells=False, front_axle_pivot=False)
        before = gm.wheel_body_clearance(rigid_box, PROFILE)
        self.assertFalse(before['overlap_free'])
        self.assertLess(before['per_wheel']['left_rear_wheel']['clearance_m'], -.04)     # wheel tops in the box
        self.assertLess(before['per_wheel']['left_front_wheel']['clearance_m'], -.06)    # steered sweep
        clearance = gm.PLACEHOLDERS['body_wheel_clearance_m'][0]
        for options in (dict(), dict(front_axle_pivot=False), dict(suspension=True)):
            _, model, manifest = steered(**options)
            after = gm.wheel_body_clearance(model, PROFILE)
            self.assertTrue(after['overlap_free'], options)
            self.assertGreater(after['min_clearance_m'], .8 * clearance, options)
            self.assertTrue(manifest['body']['wheel_wells'])
        for overrides in ({'steer_type': 'axle_pivot'}, {'wheel_radius_front_m': .16, 'wheel_radius_rear_m': .16},
                          {'steer_mech_limit_deg': {k: 35. for k in geo.WHEEL_LIMIT_KEYS}}):
            view = PROFILE.with_overrides(overrides, label='wells')
            _, model, _ = steered(view)
            self.assertTrue(gm.wheel_body_clearance(model, view)['overlap_free'], overrides)

    def test_body_boxes_without_wheels_in_the_envelope_is_the_envelope(self):
        boxes, zones = gm.body_boxes(PROFILE, (0., 0., 2.), (1.1, .54, .24), .01)
        self.assertEqual(zones, [])
        self.assertEqual(boxes, [dict(name='body', centre=[0., 0., 0.], size=[1.1, .54, .24])])

    def test_front_axle_pivot_default_and_opt_out(self):
        _, model, manifest = steered()
        desc = gm.describe_model(model)
        pivot = desc['joints'][gm.PIVOT_JOINT]
        limit = math.radians(gm.PLACEHOLDERS['front_pivot_roll_limit_deg'][0])
        self.assertEqual((pivot['type'], pivot['parent'], pivot['child'], pivot['axis']),
                         ('revolute', 'base_link', gm.PIVOT_LINK, [1., 0., 0.]))
        self.assertAlmostEqual(pivot['upper'], limit)
        self.assertAlmostEqual(pivot['lower'], -limit)
        self.assertTrue(manifest['front_axle_pivot']['enabled'])
        self.assertIsNone(model.find("link[@name='%s']/collision" % gm.PIVOT_LINK))
        _, rigid, manifest = steered(front_axle_pivot=False)
        desc = gm.describe_model(rigid)
        self.assertNotIn(gm.PIVOT_JOINT, desc['joints'])
        self.assertEqual(desc['joints']['left_front_steering_joint']['parent'], 'base_link')
        self.assertFalse(manifest['front_axle_pivot']['enabled'])
        view = PROFILE.with_overrides({'steer_type': 'axle_pivot'}, label='axle')
        _, axle, _ = steered(view)
        self.assertEqual(gm.describe_model(axle)['joints'][gm.AXLE_STEER_JOINT]['parent'], gm.PIVOT_LINK)

    def test_experimental_suspension_option(self):
        _, model, manifest = steered(suspension=True)
        block = manifest['suspension']
        self.assertTrue(block['enabled'])
        g = 9.80665
        desc = gm.describe_model(model)
        for row in block['corners']:
            wheel = row['wheel']
            joint = desc['joints'][gm.SUSPENSION_JOINT % wheel]
            self.assertEqual((joint['type'], joint['child'], joint['axis']), ('prismatic', gm.SUSPENSION_LINK % wheel,
                                                                              [0., 0., 1.]))
            self.assertEqual(desc['joints'][wheel + '_joint']['parent'], gm.SUSPENSION_LINK % wheel)
            self.assertAlmostEqual(row['spring_reference_m'], -row['sprung_mass_kg'] * g / block['rate_n_m'])
        self.assertFalse(steered()[2]['suspension']['enabled'])

    def test_w1_spawn_height_fix(self):
        rover = E.fromstring('<model name="ubts_rover"><pose>-3.000 0.000 0.099 0.000 0.000 0.000</pose></model>')
        summary = gz_assemble.fix_spawn_height(rover, {'spawn': [-3.0, 0.0, -0.0507]})
        self.assertAlmostEqual(summary['z_m'], -.0507 + gz_assemble.W1_SPAWN_DROP_M)
        self.assertAlmostEqual(summary['generated_z_m'], .099)
        self.assertEqual([float(v) for v in rover.findtext('pose').split()],
                         [-3., 0., -.0507 + gz_assemble.W1_SPAWN_DROP_M, 0., 0., 0.])
        self.assertIsNone(gz_assemble.fix_spawn_height(rover, {}))

    def test_probe_model_options_reach_every_vehicle(self):
        tmp = Path(tempfile.mkdtemp(prefix='ubts_steer_gz_test_'))
        try:
            definition = gp.group_definition('a21_limits', PROFILE)
            definition['model_options_all'] = {'front_axle_pivot': False}
            path, _, manifests = gp.build_probe_world(definition, tmp, PROFILE)
            self.assertTrue(all(not m['front_axle_pivot']['enabled'] for m in manifests.values()))
            self.assertNotIn(gm.PIVOT_JOINT, path.read_text(encoding='utf-8'))
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
        rate = gp._camera_rate([(10. + i / 30., i / 30., 640, 360) for i in range(100)], 10., 1.)
        self.assertEqual(rate['frames'], 100)
        self.assertAlmostEqual(rate['wall_fps'], 30., places=6)
        self.assertIsNone(gp._camera_rate([], 0., 0.))


class TestDirectDriveDemo(TempDirCase):
    """Pure parts of steering/gz_direct_drive.py (the joint direct-drive demo; no Gazebo)."""

    def setUp(self):
        super().setUp()
        from steering import gz_direct_drive as dd
        self.dd = dd
        self.view = dd.demo_profile(PROFILE)
        self.ctx = gp.Context(self.view, .008333333)

    def test_demo_profile_zero_offset_and_label(self):
        dd = self.dd
        self.assertEqual(self.view.per_run_value(3, 'steer_zero_offset_deg'), 0.)
        self.assertEqual(self.view.result_label()['overrides']['label'], dd.DEMO_OVERRIDE_LABEL)
        self.assertIs(dd.demo_profile(PROFILE, keep_zero_offset=True), PROFILE)
        self.assertIn('관절 직접 조종 시연', dd.LABEL_LINE_KO)
        self.assertEqual(dd.LABEL_KO, '관절 직접 조종 시연')

    def test_model_sha_ignores_indentation(self):
        dd = self.dd
        root, model, _ = steered()
        before = dd.model_sha256(model)
        E.indent(root, space='    ')
        self.assertEqual(dd.model_sha256(model), before)
        model.find('pose').text = '1 0 0 0 0 0'
        self.assertNotEqual(dd.model_sha256(model), before)

    def test_camera_poses(self):
        dd = self.dd
        yaw = math.radians(30.)
        x, y, z, roll, pitch, cam_yaw = dd.follow_pose('top', 1., 2., .1, yaw)
        px, py = dd.VIEWS['top']['point']
        self.assertAlmostEqual(x, 1. + px * math.cos(yaw) - py * math.sin(yaw))
        self.assertAlmostEqual(y, 2. + px * math.sin(yaw) + py * math.cos(yaw))
        self.assertEqual((roll, pitch), (0., math.pi / 2.))
        self.assertAlmostEqual(z, .1 + dd.VIEWS['top']['height'])
        self.assertAlmostEqual(cam_yaw, yaw + math.pi / 2.)          # image up = rover left, image right = forward
        cx, cy, cz, _, pitch, cam_yaw = dd.follow_pose('chase', 0., 0., 0., 0.)
        self.assertLess(cx, 0.)                                       # behind the rover
        self.assertGreater(pitch, 0.)                                 # looking down
        look = dd.VIEWS['chase']['look']
        self.assertAlmostEqual(math.tan(cam_yaw), (look[1] - cy) / (look[0] - cx))
        fixed = dd.follow_pose('overhead', 0., 0., 0., 0.)
        self.assertEqual(fixed[:3], tuple(dd.VIEWS['overhead']['offset']))
        model = dd.camera_model('chase', (640, 360), 15., fixed)
        self.assertEqual(model.findtext('static'), 'true')
        self.assertIsNone(model.find('.//visual'))
        self.assertIsNone(model.find('.//collision'))
        self.assertIsNone(model.find('.//inertial'))
        self.assertEqual(model.findtext('.//sensor/topic'), '/demo/camera/chase')
        grid = dd.ground_grid_model((0., 0., 0.))
        self.assertIsNone(grid.find('.//collision'))
        self.assertEqual(len(grid.findall('.//visual')), 2 * (int(2 * dd.GRID_HALF_M / dd.GRID_STEP_M) + 1))

    def test_build_w0_demo_scene_keeps_the_rover_model(self):
        dd = self.dd
        manifest = dd.build_demo_scene('w0', self.tmp / 'demo', views=('chase', 'top', 'overhead'), size=(320, 180))
        base_root = E.parse(str(self.tmp / 'demo' / 'base' / 'world.sdf')).getroot()
        demo_root = E.parse(str(self.tmp / 'demo' / 'demo_world.sdf')).getroot()
        base_rover = base_root.find("world/model[@name='ubts_rover']")
        demo_rover = demo_root.find("world/model[@name='ubts_rover']")
        self.assertEqual(dd.model_sha256(base_rover), dd.model_sha256(demo_rover))
        self.assertEqual(manifest['demo']['rover_model_sha256'], dd.model_sha256(demo_rover))
        names = [m.get('name') for m in demo_root.find('world').findall('model')]
        for view in ('chase', 'top', 'overhead'):
            self.assertIn(dd.CAMERA_MODEL_PREFIX + view, names)
        self.assertIn('demo_ground_grid', names)
        self.assertFalse(manifest['demo']['evaluation_use'])
        self.assertEqual(manifest['sdf'], 'demo_world.sdf')
        self.assertFalse([c for c in manifest['demo']['cameras'] if c['view'] == 'overhead'][0]['moves'])
        for name in ('demo_world.sdf', 'scene_manifest.json'):
            data = (self.tmp / 'demo' / name).read_bytes()
            self.assertNotIn(b'\r\n', data)
            self.assertEqual(gz_assemble.gnss_lint(data.decode('utf-8')), [])
        check = dd.check_scene(self.tmp / 'demo')
        self.assertTrue(check['rover_model_unchanged'])
        self.assertTrue(check['wheel_body_clearance']['overlap_free'])
        with self.assertRaises(dd.DemoError):
            dd.build_demo_scene('w0', self.tmp / 'x', views=('nope',))

    def test_scripts_and_duty_ramp(self):
        dd = self.dd
        for name in dd.SCRIPTS:
            script = dd.make_script(name, self.ctx, still_every=5.)
            self.assertIn(script['world'], dd.WORLD_SCENES)
            limit = math.degrees(min(self.ctx.geometry.soft_limits_eq()))
            for i in range(int(script['duration_s'] * 10)):
                t = i / 10.
                self.assertLessEqual(abs(script['steer'](t)), limit)
                self.assertTrue(0. <= script['duty'](t) <= 1.)
            self.assertEqual(script['duty'](script['duration_s'] - .01), 0.)       # ends stopped
            if script['video']:
                self.assertLessEqual(script['video']['t1'] - script['video']['t0'], 20.)
        turns = dd.make_script('w0_turns', self.ctx)
        self.assertEqual((turns['steer'](5.), turns['steer'](20.)), (15., -15.))
        ramp = dd.DutyRamp(self.ctx, lambda t: 1.)
        values = [next(iter(ramp(k * self.ctx.dt).values())) / self.ctx.line.omega0_rad_s for k in range(240)]
        self.assertAlmostEqual(values[1] - values[0], self.ctx.ramp_frac_per_s * self.ctx.dt, places=9)
        self.assertAlmostEqual(values[-1], 1., places=9)
        with self.assertRaises(dd.DemoError):
            dd.make_script('nope', self.ctx)

    def test_keys(self):
        dd = self.dd
        state = (0., 0.)
        for key in 'aaw':
            state = dd.apply_key(key, *state, step_deg=2., step_duty=.05, limit_deg=10.)[:2]
        self.assertEqual(state, (4., .05))
        self.assertEqual(dd.apply_key('d', 9., .5, step_deg=2., step_duty=.05, limit_deg=10.)[0], 7.)
        self.assertEqual(dd.apply_key('a', 9., .5, step_deg=2., step_duty=.05, limit_deg=10.)[0], 10.)
        self.assertEqual(dd.apply_key(' ', 9., .5, step_deg=2., step_duty=.05, limit_deg=10.)[1], 0.)
        self.assertEqual(dd.apply_key('q', 9., .5, step_deg=2., step_duty=.05, limit_deg=10.), (9., 0., True))

    def test_overlay_lines(self):
        dd = self.dd
        lines = dd.overlay_lines(True, world='w1', view='top', profile_id='p', t=1., target_deg=5., left_deg=5.2,
                                 right_deg=4.8, duty=1., speed=.7)
        self.assertEqual(lines[0], dd.LABEL_LINE_KO)
        self.assertIn('SIM 자리값', lines[1])
        english = dd.overlay_lines(False, world='w0', view='chase', profile_id='p', t=1., target_deg=-5.,
                                   left_deg=-4.8, right_deg=-5.2, duty=.5, speed=.3, extra='x')
        self.assertTrue(all(line.isascii() for line in english))
        self.assertEqual(len(english), 4)
        try:
            from PIL import Image
        except ImportError:
            self.skipTest('PIL not installed')
        image = dd.draw_overlay(Image.new('RGB', (320, 180), (0, 90, 0)), english, None)
        self.assertEqual(image.size, (320, 180))

    def test_run_analysis_on_a_synthetic_log(self):
        dd = self.dd
        ctx = self.ctx
        dt = ctx.dt
        t = np.arange(1, 2401) * dt
        delta = np.where(t > 2., math.radians(10.), 0.)
        speed = np.where(t > 1., .4, 0.)
        wz = speed * np.tan(delta) / ctx.geometry.wheelbase_m
        yaw = np.cumsum(wz) * dt
        x = np.cumsum(speed * np.cos(yaw)) * dt
        y = np.cumsum(speed * np.sin(yaw)) * dt
        duty = np.where(t > 1., ctx.duty_for_speed(.4), 0.)
        a = dict(t=t, x=x, y=y, z=np.zeros_like(t), roll=np.zeros_like(t), pitch=np.zeros_like(t), yaw=yaw,
                 speed=speed, wz=wz, target_eq=delta, model_eq=delta, model_left=delta, model_right=delta,
                 steer_cmd_left=delta, steer_cmd_right=delta, q_left=delta, q_right=delta,
                 qd_left=np.zeros_like(t), qd_right=np.zeros_like(t), duty=duty)
        noload = duty * ctx.line.omega0_rad_s - ctx.line.coulomb_friction_nm / ctx.line.p_gain
        # flat ground at a steady speed: a loaded wheel also runs at the no-load speed (no rolling resistance), so the
        # loaded wheels here are made slower (climbing) and only the free left rear wheel is flagged
        for wheel in gm.WHEELS:
            a['wheel_%s_qd' % wheel] = noload if wheel == 'left_rear_wheel' else noload - .5
            a['wheel_%s_z' % wheel] = np.full_like(t, .14)
            a['wheel_%s_slip' % wheel] = np.zeros_like(t)
            a['wheel_%s_susp' % wheel] = np.full_like(t, np.nan)
        script = dict(hold=None)
        out = dd.analyze_run(a, ctx, script, 'w0', {w: .14 for w in gm.WHEELS})
        self.assertEqual(out['turning']['yaw_rate_sign_agrees_fraction'], 1.)
        self.assertTrue(out['turning']['left_steer_turns_counter_clockwise'])
        self.assertAlmostEqual(out['turning']['curvature_ratio_median'], 1., places=9)
        self.assertAlmostEqual(out['unloaded_wheels']['fraction_of_driving_time']['left_rear_wheel'], 1.)
        self.assertEqual(out['unloaded_wheels']['fraction_of_driving_time']['left_front_wheel'], 0.)
        self.assertTrue(out['wheel_lift_free'])
        self.assertNotIn('suspension_travel_m', out)


def synthetic_circle_logs(group_def, ctx, duration_s=48., eta_fn=None, loss_fn=None):
    """Fixture-like logs: each vehicle drives its tag circle (R = L / tan delta, rear axle) from its start pose.
    eta_fn(delta_deg) scales the curvature (R = L / (eta tan delta)); loss_fn(delta_deg, v) lowers the speed."""
    dt = ctx.dt
    t = np.arange(1, int(round(duration_s / dt)) + 1) * dt
    logs = {}
    for spec in group_def['vehicles']:
        delta = math.radians(spec['tags']['delta_deg'])
        v = spec['tags']['v'] * (1. - (loss_fn(spec['tags']['delta_deg'], spec['tags']['v']) if loss_fn else 0.))
        eta = eta_fn(spec['tags']['delta_deg']) if eta_fn else 1.
        w = v * eta * math.tan(delta) / ctx.geometry.wheelbase_m
        yaw = spec['yaw'] + w * t
        r = v / w
        rear = ctx.geometry.rear_axle_x_in_base_m
        xr = spec['x'] + rear * math.cos(spec['yaw']) + r * (np.sin(yaw) - math.sin(spec['yaw']))
        yr = spec['y'] + rear * math.sin(spec['yaw']) - r * (np.cos(yaw) - math.cos(spec['yaw']))
        a = dict(t=t, x=xr - rear * np.cos(yaw), y=yr - rear * np.sin(yaw), z=np.zeros_like(t), yaw=yaw,
                 vx=v * np.cos(yaw), vy=v * np.sin(yaw), speed=np.full_like(t, v), wz=np.full_like(t, w))
        for key in ('left', 'right'):
            for item in ('q', 'qd', 'target'):
                a['steer_%s_%s' % (key, item)] = np.zeros_like(t)
        for wheel in gm.WHEELS:
            a['wheel_%s_qd' % wheel] = np.zeros_like(t)
            a['drive_%s_target' % wheel] = np.full_like(t, 1.)
        logs[spec['name']] = a
    return logs


class TestReviewFixes(TempDirCase):
    """Fixes of the P5 review (2026-11-09): wheel collision shape (F1), provenance (F2), fdir1 check (F3), stop distance
    (F4), single-heading start direction (F5), vehicle mass (F6), sensor mount check (F7)."""

    def test_sphere_wheel_collision_default_and_cylinder_option(self):
        """DECISIONS_KO R13 decision 1 (phase 3): the sphere is the default, the generated cylinder an option."""
        self.assertEqual(gm.DEFAULT_WHEEL_COLLISION, 'sphere')
        for options, source in ((dict(), 'default'), (dict(wheel_collision='sphere'), 'model_option')):
            _, model, manifest = steered(**options)
            for wheel in gm.WHEELS:
                link = model.find("link[@name='%s']" % wheel)
                radius = PROFILE.value('wheel_radius_front_m' if 'front' in wheel else 'wheel_radius_rear_m')
                self.assertIsNone(link.find('collision/geometry/cylinder'))
                self.assertEqual(float(link.findtext('collision/geometry/sphere/radius')), radius)
                self.assertEqual(float(link.findtext('visual/geometry/cylinder/radius')), radius)  # visual unchanged
                self.assertEqual(link.findtext('collision/surface/friction/ode/fdir1'), '0 0 1')
                self.assertEqual(link.find('collision/pose').text, link.find('visual/pose').text)  # axle frame kept
            self.assertEqual(manifest['wheel_collision'], 'sphere')
            self.assertEqual(manifest['wheel_collision_choice'],
                             dict(shape='sphere', source=source, default='sphere', visual='cylinder',
                                  decision=gm.WHEEL_COLLISION_DECISION))
            self.assertEqual({row['collision_shape'] for row in manifest['wheels']}, {'sphere'})
            clearance = gm.wheel_body_clearance(model, PROFILE)
            self.assertTrue(clearance['overlap_free'])
            self.assertEqual(clearance['wheel_shape_source'], ['visual'])
        _, model, manifest = steered(wheel_collision='cylinder')
        for wheel in gm.WHEELS:
            link = model.find("link[@name='%s']" % wheel)
            radius = PROFILE.value('wheel_radius_front_m' if 'front' in wheel else 'wheel_radius_rear_m')
            self.assertIsNone(link.find('collision/geometry/sphere'))
            self.assertEqual(float(link.findtext('collision/geometry/cylinder/radius')), radius)
            self.assertEqual(float(link.findtext('collision/geometry/cylinder/length')), PROFILE.value('wheel_width_m'))
            self.assertEqual(link.findtext('collision/surface/friction/ode/fdir1'), '0 0 1')
        self.assertEqual((manifest['wheel_collision'], manifest['wheel_collision_choice']['source']),
                         ('cylinder', 'model_option'))
        self.assertEqual({row['collision_shape'] for row in manifest['wheels']}, {'cylinder'})
        self.assertEqual(gm.wheel_body_clearance(model, PROFILE)['wheel_shape_source'], ['collision'])
        with self.assertRaises(gm.SteerModelError):
            steered(wheel_collision='torus')

    def test_vehicle_mass_is_the_profile_mass_in_every_variant(self):
        for options in (dict(), dict(front_axle_pivot=False), dict(suspension=True), dict(body_wheel_wells=False)):
            _, model, manifest = steered(**options)
            check = manifest['mass_check']
            self.assertTrue(check['matches'], (options, check))
            self.assertAlmostEqual(check['links_total_kg'], drive_model.vehicle_mass_kg(PROFILE), places=12)
        heavy = PROFILE.with_overrides({'mass_kg': 40., 'cog_x_in_base_m': .05}, label='mass')
        _, model, manifest = steered(heavy)
        self.assertTrue(manifest['mass_check']['matches'])
        self.assertAlmostEqual(manifest['mass_check']['body_cog_model_m'][0], .05, places=12)

    def test_sensor_mount_mismatch_is_refused_unless_allowed(self):
        moved = PROFILE.with_overrides({'lidar_x_in_base_m': .30, 'lidar_z_m': .65}, label='mount')
        with self.assertRaisesRegex(gm.SteerModelError, 'sensor_mount_not_applied:laser'):
            steered(moved)
        _, model, manifest = steered(moved, allow_sensor_mount_mismatch=True)
        self.assertIn('sensor_mount_not_applied:laser', manifest['result_flags'])
        self.assertIn('lidar_imu_relative_pose_not_the_profile', manifest['result_flags'])
        self.assertEqual(model.find("link[@name='laser']").findtext('pose'),
                         rover(generated_world()).find("link[@name='laser']").findtext('pose'))
        self.assertEqual(steered()[2]['result_flags'], [])

    def test_wheel_friction_model_option(self):
        _, model, manifest = steered(wheel_mu_lat=.005, wheel_mu_long=1.)
        friction = model.find("link[@name='left_front_wheel']/collision/surface/friction/ode")
        self.assertEqual((float(friction.findtext('mu')), float(friction.findtext('mu2'))), (.005, 1.))
        self.assertEqual(manifest['wheel_friction']['source'], 'model_option')
        self.assertEqual(steered()[2]['wheel_friction']['source'], 'profile')
        with self.assertRaises(gm.SteerModelError):
            steered(wheel_mu_lat=0.)

    def test_stop_distance_includes_the_first_event_step(self):
        dt = int(1e9 / 120.) / 1e9
        t = np.arange(1, 2401) * dt
        t_event, v0, decel = 10., .9, 2.
        k_event = int(np.flatnonzero(t >= t_event)[0])        # index of the end of the first event step
        v = np.full_like(t, v0)
        for i in range(k_event, len(t)):
            v[i] = max(0., v0 - decel * (i - k_event + 1) * dt)
        step = np.concatenate([[0.], .5 * (v[:-1] + v[1:]) * dt])
        x = np.cumsum(step)
        stop = gp.stop_distance(t, x, np.zeros_like(t), v, t_event)
        first_stop = int(np.flatnonzero(v < gp.STOP_SPEED_M_S)[0])
        self.assertAlmostEqual(stop['distance_m'], float(x[first_stop] - x[k_event - 1]), places=12)
        self.assertAlmostEqual(stop['speed_at_event_m_s'], v0, places=12)
        self.assertAlmostEqual(stop['event_start_t_s'], float(t[k_event - 1]), places=12)
        self.assertLess(t[k_event - 1], t_event)
        # the first version started at the end of the first event step and missed its displacement
        self.assertGreater(stop['distance_m'] - float(x[first_stop] - x[k_event]), .007)

    def _probe(self, group):
        definition = gp.group_definition(group, PROFILE)
        path, ctx, manifests = gp.build_probe_world(definition, self.tmp / group, PROFILE)
        return definition, path, ctx, manifests

    def test_single_heading_start_direction_is_not_applicable(self):
        definition, _, ctx, manifests = self._probe('a1_ackermann')
        out = gp.analyze_group('a1_ackermann', definition, ctx, synthetic_circle_logs(definition, ctx), manifests)
        self.assertIsNone(out['checks']['start_direction']['passed'])
        self.assertIn('not applicable', out['checks']['start_direction']['note'])
        definition, _, ctx, manifests = self._probe('a1_force')
        out = gp.analyze_group('a1_force', definition, ctx, synthetic_circle_logs(definition, ctx), manifests)
        check = out['checks']['start_direction']
        self.assertTrue(check['passed'])
        self.assertEqual(sorted(check['headings_deg']), [0., 90., 113.33])
        self.assertIn('isotropic', check['scope'])
        self.assertEqual(check['friction'][0]['mu_lat'], check['friction'][0]['mu_long'])

    def test_fdir_aniso_group(self):
        definition, path, ctx, manifests = self._probe('fdir_aniso')
        self.assertIn('fdir_aniso', gp.DIAGNOSTIC_GROUPS)
        aniso = [s for s in definition['vehicles'] if s['tags']['friction'] == 'anisotropic']
        self.assertEqual(sorted(s['tags']['heading_deg'] for s in aniso), sorted(gp.FDIR_HEADINGS_DEG))
        for spec in definition['vehicles']:
            friction = manifests[spec['name']]['wheel_friction']
            if spec['tags']['friction'] == 'anisotropic':
                self.assertEqual((friction['mu_lat'], friction['mu_long']), (gp.FDIR_MU_LAT, gp.FDIR_MU_LONG))
            else:
                self.assertEqual(friction['source'], 'profile')
        logs = synthetic_circle_logs(definition, ctx, 24.)
        reference = [s['name'] for s in definition['vehicles'] if s['tags']['friction'] == 'profile'][0]
        logs[reference]['x'] = logs[reference]['x'] + .2 * np.linspace(0., 1., len(logs[reference]['x']))
        out = gp.analyze_group('fdir_aniso', definition, ctx, logs, manifests)
        check = out['checks']['fdir1_turns_with_the_wheel']
        self.assertTrue(check['passed'], check)
        self.assertLess(check['max_track_difference_m'], 1e-9)
        bad = dict(logs)
        name = aniso[2]['name']
        bad[name] = dict(logs[name], y=logs[name]['y'] + 1e-3 * np.linspace(0., 1., len(logs[name]['y'])))
        out = gp.analyze_group('fdir_aniso', definition, ctx, bad, manifests)
        self.assertFalse(out['checks']['fdir1_turns_with_the_wheel']['passed'])

    def test_source_identity_and_backend_identity(self):
        source = gz_assemble.source_identity(refresh=True)
        self.assertEqual(len(source['source_files']['files_sha256']), 64)
        self.assertIn('sim/ubts', source['source_files']['per_root'])
        self.assertGreater(source['source_files']['file_count'], 100)
        real = gz_assemble._git_identity
        saved = os.environ.get(gz_assemble.ENV_SOURCE_COMMIT)
        try:
            gz_assemble._git_identity = lambda: (None, None)          # as inside the SIM container
            os.environ[gz_assemble.ENV_SOURCE_COMMIT] = 'f' * 40
            container = gz_assemble.source_identity(refresh=True)
            self.assertEqual((container['source_commit'], container['source_commit_origin']), ('f' * 40, 'environment'))
        finally:
            gz_assemble._git_identity = real
            if saved is None:
                os.environ.pop(gz_assemble.ENV_SOURCE_COMMIT, None)
            else:
                os.environ[gz_assemble.ENV_SOURCE_COMMIT] = saved
            gz_assemble.source_identity(refresh=True)
        from steering.l0_servo import L0ServoBackend
        backend = L0ServoBackend()
        backend.configure(PROFILE, 3)
        identity = gp.backend_identity(backend)
        self.assertEqual(identity['cls'], 'L0ServoBackend')
        self.assertEqual(identity['info']['seed'], 3)

        class FakeBuild:
            path, built, build_key, binary_sha256 = '/b/ubts_steer_native-x', False, 'k' * 16, 'b' * 64
            source_sha256, firmware_build_id = 's' * 64, 'ABCDEF12'

        class FakeNative:
            config, build, executable_sha256 = dict(seed=1), FakeBuild(), 'b' * 64

            def result_label(self):
                return dict(backend='native_lockstep_v1', build_id='x')
        identity = gp.backend_identity(FakeNative())
        self.assertEqual(identity['build']['firmware_build_id'], 'ABCDEF12')
        self.assertEqual(identity['build']['build_key'], 'k' * 16)
        self.assertEqual(identity['label']['backend'], 'native_lockstep_v1')
        block = gp.provenance(None, {'m0': identity})
        self.assertIn('source', block)
        self.assertEqual(block['actuator_backends']['m0']['build']['binary_sha256'], 'b' * 64)

    def test_model_option_parsing(self):
        options = gp.parse_model_options(['wheel_collision=sphere', 'suspension=true', 'tau_stall_nm=4.4',
                                          'front_axle_pivot=False'])
        self.assertEqual(options, dict(wheel_collision='sphere', suspension=True, tau_stall_nm=4.4,
                                       front_axle_pivot=False))
        with self.assertRaises(ValueError):
            gp.parse_model_options(['no_value'])

    def test_image_files_include_the_firmware(self):
        root = HERE.parents[1]
        if not (root / 'docker' / 'Dockerfile.ubts-steer').is_file():
            self.skipTest('docker/Dockerfile.ubts-steer is not here (the SIM image copies sim/ubts, not docker/)')
        ignore = (root / 'docker' / 'Dockerfile.ubts-steer.dockerignore').read_bytes().decode('utf-8').splitlines()
        self.assertIn('!firmware/', ignore)
        self.assertIn('!firmware/**', ignore)
        dockerfile = (root / 'docker' / 'Dockerfile.ubts-steer').read_bytes().decode('utf-8')
        self.assertIn('COPY firmware /opt/ubts/firmware', dockerfile)
        # in the manifest roots (WF-4 added tools/row_executor to that tuple, so the tuple is matched, not one line)
        import re
        roots = re.search(r"roots = \(([^)]*)\)", dockerfile)
        self.assertIsNotNone(roots)
        self.assertIn("'firmware'", roots.group(1))
        self.assertIn('firmware_build_id missing in BUILD_RESULT.json', dockerfile)
        # phase 3: the manifest names the image tag of the build (a build argument; the default moves with the newest
        # stack image, WF-4 switches it), no fixed p5 hint; the image imports the P6a modules
        hint = re.search(r'ARG IMAGE_TAG_HINT=ubts-steer-sim:(\w+)', dockerfile)
        self.assertIsNotNone(hint)
        self.assertNotEqual(hint.group(1), 'p5')
        self.assertIn('from steering import gz_esp_emulator, supervisor_steer, sensor_contract_steer', dockerfile)
        self.assertIn("image_tag_hint='${IMAGE_TAG_HINT}'", dockerfile)
        self.assertNotIn("image_tag_hint='ubts-steer-sim:p5'", dockerfile)


class TestPhase3Probe(TempDirCase):
    """Phase 3 (2026-11-09): the per-delta eta(delta) form with residuals (SIM_ACCEPTANCE amendment 3), the turning
    speed loss against the F-SIM drag loss range, the firmware-mode A25 report group and the demo's collision note."""

    def test_fit_eta_linear(self):
        fit = gp.fit_eta_linear([5., 10., 20.], [1. - .0025 * d for d in (5., 10., 20.)])
        self.assertAlmostEqual(fit['c_per_deg'], .0025, places=12)
        self.assertLess(fit['max_abs_residual'], 1e-12)
        rough = gp.fit_eta_linear([5., -10., 20.], [.99, .97, .96])
        self.assertEqual(rough['delta_deg'], [5., 10., 20.])
        for d, e, r in zip(rough['delta_deg'], rough['eta'], rough['residual']):
            self.assertAlmostEqual(r, e - (1. - rough['c_per_deg'] * d), places=12)
        # least squares through eta(0) = 1: the residuals are orthogonal to |delta|
        self.assertAlmostEqual(sum(d * r for d, r in zip(rough['delta_deg'], rough['residual'])), 0., places=12)

    def test_force_fit_per_delta_residuals_and_turning_speed_loss(self):
        definition = gp.group_definition('a1_force', PROFILE)
        _, ctx, manifests = gp.build_probe_world(definition, self.tmp / 'a1f', PROFILE)
        eta = lambda d: 1. - .0025 * abs(d)
        loss = lambda d, v: .004 * abs(d) * (1. if v < .4 else .5)
        logs = synthetic_circle_logs(definition, ctx, eta_fn=eta, loss_fn=loss)
        out = gp.analyze_group('a1_force', definition, ctx, logs, manifests)
        check = out['checks']['A1G_force_fit']
        summary = check['per_delta_eta_summary']
        self.assertEqual(sorted(summary['eta_by_delta_deg'], key=float), ['5', '10', '20'])
        for d in (5., 10., 20.):
            self.assertAlmostEqual(summary['eta_by_delta_deg']['%g' % d], eta(d), places=6)
            row = check['per_delta_eta_with_zero_understeer']['%g' % d]
            self.assertEqual(len(row['curvature_rel_residual']), len(row['cases_delta_deg_v']))
            self.assertLess(row['max_abs_curvature_rel_residual'], 1e-6)
            self.assertLess(max(abs(x) for x in row['radius_residual_m']), 1e-5)
        self.assertLess(summary['max_abs_curvature_rel_residual'], 1e-6)
        self.assertAlmostEqual(summary['linear_fit']['c_per_deg'], .0025, places=6)
        self.assertIn('K = 0', summary['form'])
        self.assertGreater(check['fit']['max_abs_curvature_rel_residual'], 1e-4)   # the registered form cannot fit it
        speed = out['checks']['turning_speed_loss']
        self.assertIsNone(speed['passed'])
        self.assertEqual(speed['fsim_parameter'], 'scrub_speed_loss_per_deg')
        self.assertEqual(sorted(speed['table'], key=float), ['5', '10', '20'])
        cell = speed['table']['20']
        self.assertAlmostEqual(cell['fsim_drag_loss_draw'][1], .006 * 20., places=12)
        self.assertAlmostEqual(cell['fsim_drag_loss_profile_range_max'], .01 * 20., places=12)
        self.assertAlmostEqual(cell['by_speed']['0.3']['loss'], .08, places=3)
        self.assertAlmostEqual(cell['by_speed']['0.5']['loss'], .04, places=3)
        self.assertTrue(cell['by_speed']['0.3']['inside_fsim_draw'])
        big = synthetic_circle_logs(definition, ctx, loss_fn=lambda d, v: .3)
        out = gp.analyze_group('a1_force', definition, ctx, big, manifests)
        self.assertFalse(out['checks']['turning_speed_loss']['table']['5']['by_speed']['0.3']['inside_fsim_draw'])
        kinematic = gp.group_definition('a1_kinematic', PROFILE)
        _, kctx, kmanifests = gp.build_probe_world(kinematic, self.tmp / 'a1k', PROFILE)
        out = gp.analyze_group('a1_kinematic', kinematic, kctx, synthetic_circle_logs(kinematic, kctx), kmanifests)
        self.assertNotIn('turning_speed_loss', out['checks'])

    def test_firmware_a25_group_definition(self):
        self.assertIn('a25_firmware', gp.GROUPS)
        self.assertIn('a25_firmware', gp.REPORT_GROUPS)
        self.assertNotIn('a25_firmware', gp.DIAGNOSTIC_GROUPS)
        firmware = gp.group_definition('a25_firmware', PROFILE)
        native = gp.group_definition('a25_native', PROFILE)
        self.assertEqual([v['tags']['case'] for v in firmware['vehicles']], [v['tags']['case'] for v in native['vehicles']])
        self.assertEqual({v['tags']['backend'] for v in firmware['vehicles']}, {'firmware'})
        self.assertEqual(firmware['drift_windows'], native['drift_windows'])
        self.assertEqual(firmware['duration_s'], native['duration_s'])

    def test_firmware_steer_controller_without_gazebo(self):
        """The a25_firmware host on the firmware-mode backend: ARM accepted, the C lines turn the model to the target
        at standstill without a fault line, and a drive duty in the C lines reaches the driven wheels."""
        from steering.native import build as native_build
        try:
            present = native_build.firmware_present()
            native_build.find_compiler()
        except native_build.NativeBuildError as exc:
            self.skipTest(str(exc))
        if not present:
            if native_build.firmware_required():
                self.fail(native_build.missing_firmware_message())
            self.skipTest(native_build.missing_firmware_message())
        build_dir = os.environ.get(native_build.ENV_BUILD_DIR) or str(self.tmp / 'native_build')
        build = native_build.ensure_built(build_dir=build_dir)
        ctx = gp.Context(PROFILE, .008333333)
        obs = dict(speed=0., wheel_velocity={w + '_joint': 0. for w in gm.WHEELS}, steer_position={})
        controller = gp.FirmwareSteerController(ctx, 0, lambda t: math.radians(10.) if t >= .5 else 0.,
                                                lambda t: .5 if t >= 2. else 0., executable=build.path)
        try:
            for k in range(1, 360):                       # 3 s SIM
                cmd = controller.command(k + 1, (k + 1) * ctx.dt, obs)
            summary = controller.summary()
        finally:
            controller.close()
        self.assertTrue(summary['crc_ok'])
        self.assertEqual(summary['fault_lines'], [])
        self.assertEqual(summary['refusal_lines'], [])
        self.assertEqual(summary['last_mcu_state'], 2)                      # ARMED
        self.assertEqual(summary['ticks_run'], 3000)
        self.assertEqual(summary['c_lines_sent'], 60)                       # ticks 20, 70, ..., 2970
        self.assertEqual(summary['firmware_build'], build.firmware_build_id)
        self.assertEqual(controller.steps, 360)
        eq = ctx.geometry.equivalent_from_wheels(cmd['steer']['left'], cmd['steer']['right'])
        self.assertAlmostEqual(math.degrees(eq), 10., delta=2.5)
        self.assertGreater(cmd['steer']['left'], cmd['steer']['right'])     # left turn: the inner (left) wheel more
        duty = round(255. * .5) / 255.
        for wheel in ctx.driven:
            self.assertAlmostEqual(cmd['drive'][wheel], duty * ctx.line.omega0_rad_s, delta=1e-6)
        self.assertEqual(summary['last_status']['L'], round(255. * .5))
        self.assertEqual(len(controller.rows), 360)

    def test_overlay_and_plot_note_of_the_wheel_collision(self):
        from steering import gz_direct_drive as dd
        lines = dd.overlay_lines(True, world='w0', view='chase', profile_id='p', t=1., target_deg=5., left_deg=5.2,
                                 right_deg=4.8, duty=1., speed=.7, wheel_collision='sphere')
        self.assertIn('바퀴 충돌 구', lines[1])
        english = dd.overlay_lines(False, world='w0', view='chase', profile_id='p', t=1., target_deg=5., left_deg=5.2,
                                   right_deg=4.8, duty=1., speed=.7, wheel_collision='cylinder')
        self.assertIn('wheel collision cylinder', english[1])
        plain = dd.overlay_lines(True, world='w0', view='chase', profile_id='p', t=1., target_deg=5., left_deg=5.2,
                                 right_deg=4.8, duty=1., speed=.7)
        self.assertNotIn('바퀴 충돌', plain[1])
        self.assertIsNone(dd.collision_note(None, True))


if __name__ == '__main__':
    unittest.main()
