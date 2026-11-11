"""Reuse the upstream orchard assets while replacing only its PX4 rover include."""
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as E

import yaml

from generate_world import generate
from camera_profile import load_rover_config, resolve_camera_profile, camera_profile_metadata, selected_profile

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[1] / 'ros2_ws/src/orchard_gazebo'))


INTERACTIVE_PROFILES = {
    'balanced': dict(leaves_per_tree=160, leaf_clump_every=12,
                     grass_density=.5, stone_density=.5, forest_trees=30,
                     terrain_grid=.5, texture_px_per_m=24.),
    'fast': dict(leaves_per_tree=64, leaf_clump_every=24,
                 grass_density=.15, stone_density=.15, forest_trees=16,
                 terrain_grid=.75, texture_px_per_m=16.),
}


def renderer_settings():
    """Portable CPU default; explicitly selected WSLg server uses Ogre 1/GLX."""
    engine = os.environ.get('UBTS_RENDER_ENGINE', 'ogre2')
    headless_value = os.environ.get('UBTS_HEADLESS_RENDERING', '1')
    if engine not in ('ogre', 'ogre2'):
        raise ValueError('UBTS_RENDER_ENGINE must be ogre or ogre2')
    if headless_value not in ('0', '1'):
        raise ValueError('UBTS_HEADLESS_RENDERING must be 0 or 1')
    headless = headless_value == '1'
    if engine == 'ogre' and headless:
        raise ValueError('Ogre 1 requires UBTS_HEADLESS_RENDERING=0 and an X11 display')
    return engine, headless


def lidar_backend():
    backend = os.environ.get('UBTS_LIDAR_BACKEND', 'gazebo')
    if backend not in ('gazebo', 'embree'):
        raise ValueError('UBTS_LIDAR_BACKEND must be gazebo or embree')
    return backend


def configure_physics(world, frequency):
    """Explicit local dynamics cadence; sensor rates keep their own timestamps."""
    frequency = float(frequency)
    if not math.isfinite(frequency) or not 100 <= frequency <= 1000:
        raise ValueError('physics_hz must be between 100 and 1000')
    physics = world.find('physics')
    if physics is None:
        raise ValueError('world is missing physics settings')
    for name, value in [('max_step_size', 1./frequency), ('real_time_update_rate', frequency),
                         ('real_time_factor', 1.)]:
        element = physics.find(name)
        if element is None:
            element = E.SubElement(physics, name)
        element.text = str(value)
    return frequency


def configure_quality(cfg, options):
    """Original tree/terrain generator, explicit visual-density CPU presets.

    Retains all rows, trunk/branch/apple/post/wire/hose model categories and
    original terrain height parameters. Mesh density changes also consume a
    different upstream RNG sequence: exact tree/rock placement is profile-
    specific, although lane layout and row length stay identical. Collision
    primitives are generated from the same trees/rocks as their visuals.
    """
    quality = options.get('quality', 'lite')
    if quality in ('lite', 'interactive'):
        cfg.apply_lite()
    elif quality != 'full':
        raise ValueError(f'Unknown orchard quality: {quality}')
    if quality == 'interactive':
        detail = options.get('interactive_detail', 'balanced')
        if detail not in INTERACTIVE_PROFILES:
            raise ValueError(f'Unknown interactive detail: {detail}')
        for key, value in INTERACTIVE_PROFILES[detail].items():
            setattr(cfg, key, value)
    return cfg


def configure_spawn(cfg, options):
    """Optional initial-placement fixtures; never move an already running rover.

    Upstream computes terrain height at the selected XY. Missing keys retain
    its default lane-zero, 3 m before row entry, zero-yaw placement.
    """
    changes = {}
    for name in ('spawn_x', 'spawn_y', 'spawn_yaw'):
        if name not in options:
            continue
        value = options[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'world.{name} must be a finite number')
        changes[name] = float(value)
    for name, value in changes.items():
        setattr(cfg, name, value)
    return cfg


def assemble(config, output=Path('/runtime/orchard.sdf')):
    config = resolve_camera_profile(config)
    engine, headless = renderer_settings()
    backend = lidar_backend()
    preset = config.get('world', {}).get('preset', 'original')
    if preset == 'sensor_test':
        if any(key in config.get('world', {}) for key in ('spawn_x', 'spawn_y', 'spawn_yaw')):
            raise ValueError('world spawn overrides require the original orchard preset')
        output.write_text(generate(config), encoding='utf-8')
        metadata = {'preset': preset, 'spawn': [0, 0, 0]}
    elif preset == 'original':
        from orchard_gazebo.world_gen import OrchardConfig, generate_world
        options = config.get('world', {})
        cfg = OrchardConfig(seed=options.get('seed', 7), rover_name='ubts_rover')
        configure_quality(cfg, options)
        configure_spawn(cfg, options)
        source, meta = generate_world(cfg, '/root/.gz/ubts-original-orchard')
        root = E.parse(source).getroot(); world = root.find('world')
        world.set('name', 'ubts_orchard')
        if not options.get('shadows', False):
            for element in world.findall('scene/shadows') + world.findall('light/cast_shadows'):
                element.text='false'
        included = world.find('include'); spawn = included.findtext('pose')
        world.remove(included)
        rover = E.fromstring(generate(config)).find("world/model[@name='ubts_rover']")
        rover.find('pose').text = spawn
        world.append(rover)
        # No inferred trees/segmentation labels, GNSS, magnetometer or PX4 sensors.
        removed = []
        for parent in root.iter():
            for plugin in list(parent.findall('plugin')):
                if any(kind in plugin.get('name', '') for kind in ['Label', 'NavSat', 'AirPressure', 'Magnetometer']):
                    removed.append(plugin.get('name')); parent.remove(plugin)
        E.indent(root); output.write_text(E.tostring(root, encoding='unicode'), encoding='utf-8')
        metadata = json.loads(Path(meta).read_text())
        metadata.update(preset=preset, quality=options.get('quality', 'lite'),
                        interactive_detail=options.get('interactive_detail') if options.get('quality') == 'interactive' else None,
                        render_shadows=options.get('shadows', False),
                        source='orchard_gazebo.world_gen', removed_unused_plugin_count=len(removed))
    else:
        raise ValueError(f'Unknown world preset: {preset}')
    # Both CLI and Sensors plugin select the same engine. An Ogre 1 server can
    # coexist with the existing Ogre 2 software-rendered GUI clients.
    tree = E.parse(output)
    physics_hz = configure_physics(tree.find('world'), config.get('world', {}).get('physics_hz', 250))
    for plugin in tree.findall('.//plugin'):
        if plugin.get('name') == 'gz::sim::systems::Sensors':
            render = plugin.find('render_engine')
            if render is None:
                render = E.SubElement(plugin, 'render_engine')
            render.text = engine
    if backend == 'embree':
        # The independent CPU ray caster replaces just the Gazebo sensor. The
        # fixed laser frame, all rover visuals/collisions and original orchard
        # geometry remain available for TF, Gazebo GUI and ray intersection.
        removed_sensors = 0
        rover = tree.find(".//model[@name='ubts_rover']")
        if rover is not None:
            for link in rover.findall('link'):
                for sensor in list(link.findall("sensor[@type='gpu_lidar']")):
                    link.remove(sensor)
                    removed_sensors += 1
        if removed_sensors != 1:
            raise ValueError('Embree backend requires exactly one UBTS Gazebo LiDAR sensor')
    E.indent(tree); tree.write(output, encoding='unicode')
    metadata.update(render_engine=engine, headless_rendering=headless, lidar_backend=backend,
                    physics_hz=physics_hz)
    if selected_profile() != 'legacy':
        metadata['camera_source'] = camera_profile_metadata(config)
    output.with_name('world_metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    return output


if __name__ == '__main__':
    assemble(load_rover_config(ROOT / 'rover.yaml'))
