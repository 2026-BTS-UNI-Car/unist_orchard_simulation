"""The vehicle of the user-facing steering stack (decision R91, 2026-11-16; pure parts: no ROS, no Gazebo, no Docker).

  * compose.ubts-steer.yaml passes UBTS_STEER_PROFILE through with an empty default: without the override file every role
    loads the shipped default profile v0.1 (firmware V3 servo law), as the registered G-SIM records and checks did
  * compose.ubts-steer.measured.yaml (added by tools/local_env/sim_stack.py for the local stack) names the measured
    profile front_steer_v1_spec_20261116 for every service, and its executor values agree with the profile and with the
    registered re-check steer_follow_realdims_v1 (GO threshold 4.0 deg/s, controller wheelbase = profile wheelbase); the
    resulting executor configuration validates and the row-following controller builds
  * the Dockerfile copies the override file into the image (COPY compose.ubts*.yaml)
  * steering.gz_views takes --profile (README model pictures of the measured vehicle)

unittest style (pytest on 3.12 / 3.13). Run this file in its own pytest process (it imports tools/row_executor).
Placeholder-based SIM results only (자리값 기반 SIM 결과).
"""
import json
import os
import re
from pathlib import Path
import sys
import unittest

import yaml

SIM_ROOT = Path(__file__).resolve().parent
REPO = SIM_ROOT.parents[1]
for _path in (REPO / 'tools', REPO / 'dashboard'):
    if str(_path) not in sys.path:
        sys.path.append(str(_path))

from steering import exec_container_main as EM  # noqa: E402
from steering import vehicle_profile as VP  # noqa: E402
from steering.safety_scripts import follow_realdims as FR  # noqa: E402

COMPOSE = REPO / 'compose.ubts-steer.yaml'
MEASURED = REPO / 'compose.ubts-steer.measured.yaml'
DOCKERFILE = REPO / 'docker' / 'Dockerfile.ubts-steer'
IMAGE_SIM_ROOT = '/opt/ubts/sim/ubts/'
SERVICES = ('sim', 'steer', 'recorder', 'dashboard', 'executor', 'check')
_DEFAULT = re.compile(r'^\$\{(?P<name>[A-Z0-9_]+):-(?P<default>.*)\}$', re.S)


def default_of(value, name):
    """The default of a compose value '${NAME:-default}' (the name must match)."""
    match = _DEFAULT.match(str(value))
    if not match or match.group('name') != name:
        raise AssertionError('%r is not ${%s:-...}' % (value, name))
    return match.group('default')


class StackVehicleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not (COMPOSE.is_file() and MEASURED.is_file()):
            raise unittest.SkipTest('compose files not next to the work tree (an image without them)')
        cls.base = yaml.safe_load(COMPOSE.read_bytes().decode('utf-8'))
        cls.measured = yaml.safe_load(MEASURED.read_bytes().decode('utf-8'))

    def test_base_compose_keeps_the_shipped_default(self):
        self.assertEqual(self.base['x-environment']['UBTS_STEER_PROFILE'], '${UBTS_STEER_PROFILE:-}')
        executor = self.base['services']['executor']['environment']
        self.assertEqual(executor['UBTS_STEER_PROFILE'], '${UBTS_STEER_PROFILE:-}')
        self.assertEqual(executor['UBTS_STEER_EXEC_CONTROLLER'], '${UBTS_STEER_EXEC_CONTROLLER:-{"kind": "row_follow"}}')
        # an empty value means the shipped default (supervisor_steer and exec_container_main: env or None)
        self.assertEqual(VP.DEFAULT_PROFILE_ID, 'front_steer_v0_1_placeholder')
        if not os.environ.get('UBTS_STEER_PROFILE'):
            self.assertIsNone(EM.build_parser().parse_args([]).profile)

    def test_override_names_the_measured_profile_everywhere(self):
        services = self.measured['services']
        self.assertEqual(set(services), set(SERVICES))
        self.assertTrue(set(services) <= set(self.base['services']))
        paths = {default_of(services[name]['environment']['UBTS_STEER_PROFILE'], 'UBTS_STEER_PROFILE')
                 for name in SERVICES}
        self.assertEqual(len(paths), 1)
        path = paths.pop()
        self.assertTrue(path.startswith(IMAGE_SIM_ROOT), path)
        local = SIM_ROOT / path[len(IMAGE_SIM_ROOT):]
        profile = VP.load_profile(local)
        self.assertEqual(profile.profile_id, FR.PROFILE_ID)
        self.assertEqual(profile.value('steer_fw_mode'), 'abs_hold_8')
        for name in SERVICES:      # the override only sets environment values
            self.assertEqual(set(services[name]), {'environment'})

    def test_executor_values_agree_with_the_profile_and_the_registered_recheck(self):
        env = self.measured['services']['executor']['environment']
        controller = json.loads(default_of(env['UBTS_STEER_EXEC_CONTROLLER'], 'UBTS_STEER_EXEC_CONTROLLER'))
        overrides = json.loads(default_of(env['UBTS_STEER_EXEC_OVERRIDES'], 'UBTS_STEER_EXEC_OVERRIDES'))
        profile = FR.base_profile()
        self.assertEqual(controller, FR.controller(profile))
        self.assertEqual(overrides, FR.EXEC_GATE)
        cal_id = int(EM.build_parser().parse_args([]).cal_id)
        host = EM.host_profile_of(profile, cal_id)
        with self.assertRaises(Exception):            # as the registered gate_as_is part recorded
            EM.gsim_executor_config(profile, host, cal_id)
        cfg = EM.gsim_executor_config(profile, host, cal_id, overrides)
        self.assertAlmostEqual(cfg.steer_rotation_centre_x_m, float(profile.value('rear_axle_x_in_base_m')))
        self.assertAlmostEqual(cfg.lidar_x_in_base_m, float(profile.value('lidar_x_in_base_m')))
        slot = EM.ControllerSlot(controller, cfg)
        self.assertEqual(type(slot()).__name__, 'RowFollowController')

    def test_dockerfile_copies_the_override(self):
        if not DOCKERFILE.is_file():
            raise unittest.SkipTest('Dockerfile not next to the work tree')
        text = DOCKERFILE.read_bytes().decode('utf-8')
        self.assertIn('COPY compose.ubts*.yaml /opt/ubts/', text)
        self.assertTrue(re.fullmatch(r'compose\.ubts.*\.yaml', MEASURED.name))

    def test_gz_views_takes_a_profile(self):
        from steering import gz_views
        import inspect
        source = inspect.getsource(gz_views.main)
        self.assertIn("'--profile'", source)
        self.assertIn('load_profile(args.profile)', source)


if __name__ == '__main__':
    unittest.main()
