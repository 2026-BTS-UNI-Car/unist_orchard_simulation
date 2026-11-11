"""Explicit SIM camera resolution candidate. No ROS, GPU or device operations.

An opt-in native profile changes only camera image dimensions. Renderer and
sensor configuration stay independently pinned by the selected Compose files.
Nothing here certifies achieved cadence or labels a resized preview native.
"""
from copy import deepcopy
import os
from pathlib import Path
import yaml


PROFILE_ENV = 'UBTS_CAMERA_PROFILE'
DEFAULT_PROFILE = 'legacy'
NATIVE_PROFILE = 'native1080_gpu'
REQUIRED_GPU_ENV = {
    'DASHBOARD_MODE': 'simulation',
    'UBTS_RENDER_ENGINE': 'ogre',
    'UBTS_HEADLESS_RENDERING': '0',
    'UBTS_LIDAR_BACKEND': 'embree',
    'LIBGL_ALWAYS_SOFTWARE': '0',
    'GALLIUM_DRIVER': 'd3d12',
}


def selected_profile(environment=None):
    environment = os.environ if environment is None else environment
    name = environment.get(PROFILE_ENV, DEFAULT_PROFILE)
    if name not in (DEFAULT_PROFILE, NATIVE_PROFILE):
        raise ValueError('unsupported_sim_camera_profile')
    return name


def resolve_camera_profile(config, environment=None):
    environment = os.environ if environment is None else environment
    result = deepcopy(config)
    if selected_profile(environment) == DEFAULT_PROFILE:
        return result
    for key, expected in REQUIRED_GPU_ENV.items():
        if environment.get(key) != expected:
            raise ValueError('native_camera_gpu_environment_required:' + key)
    camera = result['camera']
    if camera.get('hardware_max') != '1920x1080@30':
        raise ValueError('native_camera_capability_not_declared')
    if (type(camera.get('hz')) not in (int, float)
            or camera['hz'] != 30.):
        # Preserve configured source cadence; never turn 25Hz into a claimed30.
        raise ValueError('native_camera_source_rate_must_already_be_30')
    camera.update(width=1920, height=1080)
    return result


def load_rover_config(path, environment=None):
    return resolve_camera_profile(
        yaml.safe_load(Path(path).read_text(encoding='utf-8')), environment)


def camera_profile_metadata(config, environment=None):
    camera = config['camera']
    name = selected_profile(environment)
    if name == NATIVE_PROFILE and (camera['width'], camera['height']) != (1920, 1080):
        raise ValueError('native_camera_metadata_requires_resolved_source')
    return dict(profile=name,
                native_width=camera['width'], native_height=camera['height'],
                configured_hz=camera['hz'], source_topic='/camera/image_raw',
                optical_frame='camera_link_optical',
                achieved_cadence_qualified=False,
                original_pixel_content_qualified=False)
