"""Gazebo scenes with the steered rover (design 5.1 W0/W1 and 8.3 P5).

build_scene() calls the pinned assemble_world.assemble() (not changed) and then applies
gz_model.steer_model_element() to the ubts_rover model. Scenes:

  w0_force            W0 flat world: the sensor_test world of generate_world with its trees
                      and test obstacles removed, a large flat floor (ground_mu) and a few
                      thin pillars for LiDAR registration; the steered rover in the main
                      G-SIM configuration (steering JointPositionController, drive
                      JointController force mode = DC motor line)
  w0_kinematic        W0 for the geometry check A1(G): ideal Ackermann (ratio 1), high
                      friction (wheel mu 1.0), per-wheel kinematic speed mode
  w0_ackermann_check  W0 with the standard AckermannSteering system driving the rear wheels
                      and the steering joints from a Twist; comparison only (risk R5)
  w1_force            W1 original orchard (orchard_gazebo.world_gen through assemble_world,
                      preset original) with the steered rover, main configuration
  field1010           W2 10/10 reconstructed row (or W3, a procedural orchard: the manifest world says which):
                      the sensor_test world of generate_world with its trees, test obstacles and ground removed,
                      the field world of
                      a field_world spec (field_world.build_field_world: ground + mode R / T geometry in the
                      corridor start frame W) and the steered rover at the spec's start pose (corridor frame
                      s, d, heading), main configuration. The field world files (raster.npz for the clip
                      node, manifest.json, eval_truth.json for evaluation only, field_world.sdf) go to
                      <out>/field_world/. world_metadata.json gets the new schema
                      ubts.steer_field_scene_metadata.v1 (no world truth in it; the truth is eval_truth.json)

The world keeps the name ubts_orchard (existing tools use /world/ubts_orchard/control).
Physics rate = profile gz_physics_hz (120 Hz, as rover.yaml). The LiDAR backend defaults to
embree (assemble_world removes the Gazebo gpu_lidar; cpu_lidar.py makes /sim/scan from
/ground_truth/odom). Renderer settings come from UBTS_RENDER_ENGINE and
UBTS_HEADLESS_RENDERING unless given. The camera (640x360 from rover.yaml) can be removed
with camera=False.

assemble_world writes the original orchard meshes under /root/.gz (inside the SIM
container), so w1_force is built in the container. Outputs go to the folder passed in:
world.sdf, world_metadata.json (assemble_world), scene_manifest.json (this module).

GNSS origin: the original orchard generator writes <spherical_coordinates> with a GPS origin
near a real place and the same latitude/longitude/elevation into its metadata. Steering
scenes carry no GNSS origin: both are removed from world.sdf and world_metadata.json right
after assemble_world, and gnss_lint() must find nothing in the written files (no GNSS
coordinates in any output; design 5.1/5.2). The virtual GPS of P6a is a separate matter.

The rover's wheel collision shape is the gz_model default (sphere, DECISIONS_KO R13 decision 1) unless the model
option wheel_collision (CLI --wheel-collision cylinder) asks for the generated cylinder.

Placeholder-based SIM scenes; results built on them are not field validation.
CLI: python -m steering.gz_assemble --scene w0_force --out <folder> [--wheel-collision cylinder]
     python -m steering.gz_assemble --scene field1010 --out <folder> --record 040927 --corridor c02 [--mode T]
     python -m steering.gz_assemble --scene field1010 --out <folder> --field-spec <spec.json>
(sim/ubts on sys.path).
"""
from __future__ import annotations

import argparse
import contextlib
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as E

from . import gz_model
from .vehicle_profile import load_profile

SCHEMA = 'ubts.steer_gz_scene.v1'
SIM_ROOT = Path(__file__).resolve().parents[1]           # sim/ubts
SCENES = ('w0_force', 'w0_kinematic', 'w0_ackermann_check', 'w1_force', 'field1010')
SCENE_WORLD = {'w0_force': 'W0', 'w0_kinematic': 'W0', 'w0_ackermann_check': 'W0', 'w1_force': 'W1',
               'field1010': 'W2'}
FIELD_SCENES = ('field1010',)
FIELD_METADATA_SCHEMA = 'ubts.steer_field_scene_metadata.v1'
FIELD_REMOVED_MODELS = ('barrier', 'low_obstacle', 'end_wall', 'ground')
ROVER_SPAWN_Z_M = .003                     # generate_world's rover pose height above the floor
# Profile overrides of the geometry-check scenes (design 7.2 A1(G): a = 1, high friction). All inside the ranges.
KINEMATIC_OVERRIDES = {'steer_ackermann_ratio': 1.0, 'wheel_mu_lat': 1.0, 'wheel_mu_long': 1.0,
                       'gz_drive_mode': 'kinematic_velocity'}
ACKERMANN_CHECK_OVERRIDES = {'steer_ackermann_ratio': 1.0, 'wheel_mu_lat': 1.0, 'wheel_mu_long': 1.0}
LIDAR_BACKENDS = ('embree', 'gazebo')
# W0 placeholders (SIM_ASSUMED): floor and LiDAR registration pillars (radius, height, positions in m).
W0_GROUND_SIZE_M = (200., 200., .2)
W0_PILLAR_RADIUS_M = .08
W0_PILLAR_HEIGHT_M = 1.5
W0_PILLARS_XY = ((22., 6.), (22., -6.), (-22., 6.), (-22., -6.), (6., 22.), (-6., -22.))
W0_REMOVED_PREFIXES = ('tree_',)
W0_REMOVED_MODELS = ('barrier', 'low_obstacle', 'end_wall')
# W1 spawn (SIM_ASSUMED): rover origin this far above the generator's terrain height at the spawn point.
W1_SPAWN_DROP_M = .03
# GNSS origin fields removed from every scene and its metadata (no GNSS coordinates in any output).
GNSS_METADATA_KEYS = ('latitude', 'longitude', 'elevation')
GNSS_LINT_WORDS = ('spherical_coordinates', 'latitude_deg', 'longitude_deg', '"latitude"', '"longitude"')


class SceneError(ValueError):
    """Unknown scene or option, or the assembled world is not the expected one."""


@contextlib.contextmanager
def _environment(**values):
    """Temporarily set (or with None leave) environment variables read by assemble_world."""
    saved = {}
    try:
        for key, value in values.items():
            if value is None:
                continue
            saved[key] = os.environ.get(key)
            os.environ[key] = value
        yield
    finally:
        for key, old in saved.items():
            if old is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = old


def _assemble_module():
    """Import the pinned assemble_world (needs sim/ubts on sys.path; it adds orchard_gazebo itself)."""
    if str(SIM_ROOT) not in sys.path:
        sys.path.insert(0, str(SIM_ROOT))
    import assemble_world                      # pinned module, imported only
    return assemble_world


def scene_profile(scene, profile):
    """The profile view used by a scene (overrides for the geometry-check scenes)."""
    if scene == 'w0_kinematic':
        return profile.with_overrides(KINEMATIC_OVERRIDES, label='gz_w0_kinematic_geometry_check')
    if scene == 'w0_ackermann_check':
        return profile.with_overrides(ACKERMANN_CHECK_OVERRIDES, label='gz_w0_ackermann_cross_check')
    return profile


def scene_options(scene):
    if scene == 'w0_kinematic':
        return dict(drive_mode='kinematic_velocity', steer_control='position')
    if scene == 'w0_ackermann_check':
        return dict(drive_mode=None, steer_control='ackermann')
    return dict(drive_mode='force_dc', steer_control='position')


def _num(value):
    return gz_model._num(value)


def _static_cylinder(world, name, xy, radius, height, color='.55 .55 .58 1'):
    model = E.SubElement(world, 'model', name=name)
    E.SubElement(model, 'static').text = 'true'
    E.SubElement(model, 'pose').text = '%s %s %s 0 0 0' % (_num(xy[0]), _num(xy[1]), _num(height / 2.))
    link = E.SubElement(model, 'link', name='body')
    for kind in ('visual', 'collision'):
        element = E.SubElement(link, kind, name='body' if kind == 'visual' else 'body_collision')
        cylinder = E.SubElement(E.SubElement(element, 'geometry'), 'cylinder')
        E.SubElement(cylinder, 'radius').text = _num(radius)
        E.SubElement(cylinder, 'length').text = _num(height)
        if kind == 'visual':
            material = E.SubElement(element, 'material')
            E.SubElement(material, 'ambient').text = color
            E.SubElement(material, 'diffuse').text = color
    return model


def make_w0(world, profile):
    """Turn the assembled sensor_test world into W0 (in place). Returns a summary dict."""
    removed = []
    for model in list(world.findall('model')):
        name = model.get('name', '')
        if name.startswith(W0_REMOVED_PREFIXES) or name in W0_REMOVED_MODELS:
            world.remove(model)
            removed.append(name)
    ground = world.find("model[@name='ground']")
    if ground is None:
        raise SceneError('w0_needs_the_generated_ground')
    size = ' '.join(_num(v) for v in W0_GROUND_SIZE_M)
    for box in ground.findall('.//geometry/box/size'):
        box.text = size
    mu = float(profile.value('ground_mu'))
    for tag in ('mu', 'mu2'):
        for element in ground.findall('.//surface/friction/ode/' + tag):
            element.text = _num(mu)
    pillars = []
    for index, xy in enumerate(W0_PILLARS_XY):
        _static_cylinder(world, 'w0_pillar_%d' % index, xy, W0_PILLAR_RADIUS_M, W0_PILLAR_HEIGHT_M)
        pillars.append(list(xy))
    return dict(removed_models=len(removed), removed_kinds=sorted({n.split('_')[0] for n in removed}),
                ground_size_m=list(W0_GROUND_SIZE_M), ground_mu=mu, pillars_xy_m=pillars,
                pillar_radius_m=W0_PILLAR_RADIUS_M, pillar_height_m=W0_PILLAR_HEIGHT_M,
                provenance='SIM_ASSUMED (W0 floor and pillars)')


def fix_spawn_height(rover, metadata, drop_m=W1_SPAWN_DROP_M):
    """Put the rover origin (base_footprint, at the wheel bottoms) drop_m above the generator's terrain height of
    the spawn point (metadata 'spawn' [x, y, terrain z]). Returns a summary dict, or None without that entry.

    The original orchard generator keeps the pose of its own rover include, about 0.15 m above the terrain;
    with the UBTS rover (origin at the wheel bottoms) that pose dropped the rover 0.15 m at the start."""
    spawn = metadata.get('spawn') if isinstance(metadata, dict) else None
    if not (isinstance(spawn, (list, tuple)) and len(spawn) == 3):
        return None
    pose = [float(v) for v in rover.findtext('pose').split()]
    if len(pose) != 6:
        raise SceneError('rover_pose_needs_6_values')
    generated = pose[2]
    pose[2] = float(spawn[2]) + drop_m
    rover.find('pose').text = ' '.join(_num(v) for v in pose)
    return dict(generated_z_m=generated, terrain_z_m=float(spawn[2]), z_m=pose[2], drop_m=drop_m,
                provenance='SIM_ASSUMED (drop height)')


def strip_gnss_world(world):
    """Remove <spherical_coordinates> (the original world_gen writes a GPS origin near a real place). Returns the count.

    Steering scenes carry no GNSS origin (design 5.1/5.2, rule: no GNSS coordinates in any output)."""
    removed = 0
    for parent in list(world.iter()):
        for element in list(parent.findall('spherical_coordinates')):
            parent.remove(element)
            removed += 1
    return removed


def strip_gnss_metadata(value, path=''):
    """Copy of a metadata value without GNSS_METADATA_KEYS at any depth. Returns (clean value, removed key paths)."""
    removed = []
    if isinstance(value, dict):
        clean = {}
        for key, item in value.items():
            where = '%s.%s' % (path, key) if path else str(key)
            if key in GNSS_METADATA_KEYS:
                removed.append(where)
                continue
            clean[key], sub = strip_gnss_metadata(item, where)
            removed.extend(sub)
        return clean, removed
    if isinstance(value, list):
        clean = []
        for index, item in enumerate(value):
            item_clean, sub = strip_gnss_metadata(item, '%s[%d]' % (path, index))
            clean.append(item_clean)
            removed.extend(sub)
        return clean, removed
    return value, removed


def gnss_lint(text):
    """GNSS field names still present in an SDF or JSON text (empty list = clean)."""
    return [word for word in GNSS_LINT_WORDS if word in text]


def remove_camera(world):
    """Remove the camera sensors (server-only runs). Returns the number removed."""
    count = 0
    for link in world.iter('link'):
        for sensor in list(link.findall("sensor[@type='camera']")):
            link.remove(sensor)
            count += 1
    return count


def _git_identity():
    """(commit, dirty status digest) of the work tree, or (None, None) outside git. Read-only."""
    import subprocess
    try:
        top = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=str(SIM_ROOT), capture_output=True, text=True,
                             timeout=10)
        status = subprocess.run(['git', 'status', '--porcelain'], cwd=str(SIM_ROOT), capture_output=True,
                                text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return None, None
    if top.returncode != 0:
        return None, None
    return top.stdout.strip(), hashlib.sha256(status.stdout.encode('utf-8')).hexdigest()


# --- source identity of results (design 7.3: every result names the repository commit and the MCU build) -----------
# Inside the SIM container git cannot read the work tree (it is mounted at /work, the .git file points to a host path),
# so the host passes the commit and the status digest: docker run -e UBTS_STEER_SOURCE_COMMIT=... -e
# UBTS_STEER_SOURCE_STATUS_SHA256=... . The file SHA bundle is computed here from the files that actually run.
ENV_SOURCE_COMMIT = 'UBTS_STEER_SOURCE_COMMIT'
ENV_SOURCE_STATUS = 'UBTS_STEER_SOURCE_STATUS_SHA256'
SOURCE_ROOTS = ('dashboard', 'sim/ubts', 'ros2_ws/src/orchard_gazebo', 'ros2_ws/src/orchard_navigation', 'firmware')
SOURCE_SKIP_DIRS = ('__pycache__', '.pytest_cache', '_build', 'steer_native_build')
SOURCE_SKIP_SUFFIXES = ('.so', '.pyc', '.pyo', '.exe', '.o', '.obj')
IMAGE_MANIFEST_PATH = Path('/opt/ubts/STEER_IMAGE_MANIFEST.json')
_SOURCE_CACHE = {}


def source_files_identity(root=None):
    """SHA-256 bundle of the source files under SOURCE_ROOTS (+ compose.ubts*.yaml) of the tree `root` (default: the
    tree this module runs from): the image manifest rule ('<sha256> <relative path>' lines sorted by path, hashed),
    per root and over all. Build outputs and caches are left out."""
    root = Path(SIM_ROOT.parents[1] if root is None else root)
    rows = {}
    for top in SOURCE_ROOTS:
        base = root / top
        if not base.is_dir():
            continue
        for path in base.rglob('*'):
            if not path.is_file() or any(part in SOURCE_SKIP_DIRS for part in path.relative_to(root).parts):
                continue
            if path.suffix in SOURCE_SKIP_SUFFIXES:
                continue
            rows[path.relative_to(root).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    for path in sorted(root.glob('compose.ubts*.yaml')):
        rows[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()

    def bundle(items):
        return hashlib.sha256(''.join('%s %s\n' % (v, k) for k, v in sorted(items)).encode('utf-8')).hexdigest()
    per_root = {}
    for top in SOURCE_ROOTS:
        items = [(k, v) for k, v in rows.items() if k.startswith(top + '/')]
        if items:
            per_root[top] = dict(files=len(items), sha256=bundle(items))
    return dict(root=str(root), file_count=len(rows), files_sha256=bundle(rows.items()), per_root=per_root,
                rule="sha256 of sorted '<file sha256> <relative path>' lines (the image manifest rule); skipped: %s, "
                     "*%s" % (', '.join(SOURCE_SKIP_DIRS), ' *'.join(SOURCE_SKIP_SUFFIXES)))


def source_identity(refresh=False):
    """Identity of the code a result came from (cached per process): repository commit and status digest (git in the
    tree, else the host's UBTS_STEER_SOURCE_COMMIT / UBTS_STEER_SOURCE_STATUS_SHA256), the SHA-256 bundle of the
    running source files, and the image manifest summary when running in the SIM image."""
    if _SOURCE_CACHE and not refresh:
        return _SOURCE_CACHE['value']
    commit, status = _git_identity()
    origin = 'git' if commit else None
    if commit is None and os.environ.get(ENV_SOURCE_COMMIT):
        commit, status, origin = os.environ[ENV_SOURCE_COMMIT], os.environ.get(ENV_SOURCE_STATUS), 'environment'
    files = source_files_identity()
    image = None
    if IMAGE_MANIFEST_PATH.is_file():
        try:
            data = json.loads(IMAGE_MANIFEST_PATH.read_text(encoding='utf-8'))
            image = {k: data.get(k) for k in ('image_tag_hint', 'base_image_id', 'source_commit', 'source_status_sha256',
                                              'source_files_sha256', 'source_file_count', 'built_utc')}
            image['running_the_image_sources'] = Path(files['root']).resolve() == IMAGE_MANIFEST_PATH.parent.resolve()
        except (OSError, ValueError) as exc:
            image = dict(error='%s: %s' % (type(exc).__name__, exc))
    value = dict(source_commit=commit, source_commit_origin=origin, source_status_sha256=status,
                 source_files=files, image_manifest=image,
                 note='source_files is computed from the files that ran (a mounted work tree or the image copy); the '
                      'commit names the repository state they came from (dirty if the status digest is not that of '
                      'an empty status)')
    _SOURCE_CACHE['value'] = value
    return value


def field_spec_of(field_spec):
    """FieldWorldSpec from a spec object, a dict, a JSON file path or JSON text."""
    from field_world.scene_spec import FieldWorldSpec
    if isinstance(field_spec, FieldWorldSpec):
        return field_spec
    if isinstance(field_spec, dict):
        return FieldWorldSpec(field_spec)
    if isinstance(field_spec, Path) or (isinstance(field_spec, str) and not field_spec.lstrip().startswith('{')):
        return FieldWorldSpec.from_json(Path(str(field_spec)).read_bytes())
    if isinstance(field_spec, (str, bytes)):
        return FieldWorldSpec.from_json(field_spec)
    raise SceneError('field_spec_needed: a field_world spec (object, dict, JSON file or JSON text)')


def make_field(world, profile, field_spec, out_dir=None):
    """Turn the assembled sensor_test world into a field world scene (in place): remove the generated trees, test
    obstacles and ground, add the field ground and models (field_world.build_field_world) and put the rover at the
    spec start (corridor frame -> W). Writes the field world files to out_dir/field_world when out_dir is given.
    Returns (summary dict, FieldWorld)."""
    from field_world import build_field_world as fwb
    spec = field_spec_of(field_spec)
    fw = fwb.build(spec, profile=profile)
    removed = []
    for model in list(world.findall('model')):
        name = model.get('name', '')
        if name.startswith(W0_REMOVED_PREFIXES) or name in FIELD_REMOVED_MODELS:
            world.remove(model)
            removed.append(name)
    if 'ground' not in removed:
        raise SceneError('field_needs_the_generated_ground')
    rover = world.find("model[@name='%s']" % gz_model.MODEL_NAME)
    index = list(world).index(rover) if rover is not None else len(world)
    for offset, model in enumerate([fw.ground] + list(fw.models)):
        world.insert(index + offset, model)
    start = spec.start
    x, y, heading = fw.frame.pose(start['s_m'], start['d_m'], math.radians(start['heading_deg']))
    wx, wy = fw.transform.to_w(x, y)
    yaw = float(fw.transform.yaw_to_w(heading))
    spawn = (float(wx), float(wy), ROVER_SPAWN_Z_M, yaw)
    if rover is not None:
        rover.find('pose').text = '%s %s %s 0 0 %s' % (_num(spawn[0]), _num(spawn[1]), _num(spawn[2]), _num(spawn[3]))
    files = None
    if out_dir is not None:
        manifest = fwb.write(fw, Path(out_dir) / 'field_world')
        files = dict(folder='field_world', manifest_sha256=fwb._sha256_file(Path(out_dir) / 'field_world' /
                                                                            'manifest.json'),
                     files=manifest['files'], coord_lint=manifest['coord_lint'])
    counts = fwb.counts(fw)
    summary = dict(source=spec.source, field_spec_name=spec.name, field_spec_sha256=spec.sha256(), mode=spec.mode,
                   crop=spec.crop, record=spec.record, corridor_id=spec.corridor_id, direction=spec.direction,
                   removed_models=len(removed), boxes=counts['boxes'], cylinders=counts['cylinders'],
                   collision_shapes=counts['collision_shapes'], triangles_estimate=counts['triangles_estimate'],
                   ground_extent_w_m=[round(v, 3) for v in fw.ground_extent_w], ground_mu=fw.ground_mu,
                   spawn_w=dict(x_m=round(spawn[0], 4), y_m=round(spawn[1], 4), z_m=spawn[2],
                                yaw_rad=round(spawn[3], 6), corridor_start=start),
                   field_world_files=files, world_frame='W: corridor start (s 0, d 0), +x start tangent',
                   provenance='field_world (catalogue = SLAM reconstruction; procedural = W3 generator)')
    return summary, fw


def world_label(scene, summary=None):
    """The design 5.1 world label of a scene: a field scene is W2 for a catalogue spec and W3 for a procedural one
    (review RW-09); the other scenes have a fixed label (SCENE_WORLD)."""
    if scene in FIELD_SCENES and (summary or {}).get('source') == 'procedural':
        return 'W3'
    return SCENE_WORLD[scene]


def field_metadata(scene, base_metadata, summary):
    """world_metadata.json of a field scene (new schema; renderer fields of assemble_world kept, no world truth)."""
    keep = {k: base_metadata.get(k) for k in ('render_engine', 'headless_rendering', 'lidar_backend', 'physics_hz',
                                               'camera_source') if k in base_metadata}
    spawn = summary['spawn_w']
    return dict(keep, schema=FIELD_METADATA_SCHEMA, preset=scene, world=world_label(scene, summary),
                source=summary['source'],
                field_spec_name=summary['field_spec_name'], field_spec_sha256=summary['field_spec_sha256'],
                mode=summary['mode'], spawn=[spawn['x_m'], spawn['y_m'], spawn['z_m']], spawn_yaw=spawn['yaw_rad'],
                field_world_folder='field_world', raster_copy='field_world/raster.npz',
                truth_note='world truth (centreline, rows, end) only in field_world/eval_truth.json, evaluation only')


def profile_lidar_mount(config, view):
    """Put the profile's LiDAR mount (lidar_x_in_base_m, lidar_y_in_base_m, lidar_z_m) into a rover config's lidar
    block in place, so that the generated laser link (generate_world: laser and imu_link at the LiDAR x, y) and the
    Embree LiDAR (cpu_lidar.py reads the same block from the runtime copy the supervisor writes) follow the profile.
    The pinned rover.yaml is not edited. Returns the mount block of the scene manifest. A profile with the rover.yaml
    values (every profile before 2026-11-16) leaves the config, and so the SDF, as it was."""
    lidar = config['lidar']
    want = dict(x_m=float(view.value('lidar_x_in_base_m')), y_m=float(view.value('lidar_y_in_base_m')),
                height_above_ground_m=float(view.value('lidar_z_m')))
    before = {key: float(lidar[key]) for key in want}
    changed = any(abs(want[key] - before[key]) > 1e-9 for key in want)
    if changed:
        lidar.update(want)
    return dict(want, source='profile', rover_yaml=before, differs_from_rover_yaml=changed)


def assemble_base(scene, out_dir, *, profile=None, profile_path=None, lidar_backend='embree', render_engine=None,
                  headless_rendering=None, camera=True, rover_config=None, field_spec=None):
    """Assemble a scene's world with the generated (not yet steered) ubts_rover.

    Writes out_dir/world.sdf and world_metadata.json through assemble_world and returns a dict:
    tree, world (the <world> element, W0 / field changes applied), view (the scene profile view), engine,
    headless, summary, cameras_removed, output (Path), metadata (assemble_world's metadata dict, or the field
    scene metadata), field_world (the FieldWorld of a field scene, else None). field_spec: the field_world spec
    of a field scene (FieldWorldSpec, dict, JSON file or text).
    """
    if scene not in SCENES:
        raise SceneError('unknown_scene:%r (choices %s)' % (scene, ', '.join(SCENES)))
    if scene in FIELD_SCENES and field_spec is None:
        raise SceneError('field_scene_needs_a_field_spec: %s' % scene)
    if scene not in FIELD_SCENES and field_spec is not None:
        raise SceneError('field_spec_only_for_field_scenes: %s' % scene)
    if lidar_backend not in LIDAR_BACKENDS:
        raise SceneError('unknown_lidar_backend:%r' % (lidar_backend,))
    if profile is None:
        profile = load_profile(profile_path)
    view = scene_profile(scene, profile)
    assemble_world = _assemble_module()
    from camera_profile import load_rover_config        # pinned module, imported only
    config = copy.deepcopy(rover_config) if rover_config is not None else load_rover_config(SIM_ROOT / 'rover.yaml')
    lidar_mount = profile_lidar_mount(config, view) if rover_config is None else dict(
        x_m=float(config['lidar']['x_m']), y_m=float(config['lidar']['y_m']),
        height_above_ground_m=float(config['lidar']['height_above_ground_m']), source='rover_config_argument',
        differs_from_rover_yaml=None)
    world_options = config.setdefault('world', {})
    world_options['physics_hz'] = float(view.value('gz_physics_hz'))
    world_options['preset'] = 'sensor_test' if SCENE_WORLD[scene] in ('W0', 'W2') else 'original'
    if SCENE_WORLD[scene] in ('W0', 'W2'):
        for key in ('spawn_x', 'spawn_y', 'spawn_yaw'):
            world_options.pop(key, None)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    output = out_dir / 'world.sdf'
    environment = dict(UBTS_LIDAR_BACKEND=lidar_backend, UBTS_RENDER_ENGINE=render_engine,
                       UBTS_HEADLESS_RENDERING=None if headless_rendering is None else ('1' if headless_rendering else '0'))
    with _environment(**environment):
        assemble_world.assemble(config, output=output)
        engine, headless = assemble_world.renderer_settings()
    tree = E.parse(output)
    world = tree.getroot().find('world')
    gnss_removed = strip_gnss_world(world)
    metadata_path = out_dir / 'world_metadata.json'
    raw_metadata = json.loads(metadata_path.read_text(encoding='utf-8')) if metadata_path.exists() else {}
    fw = None
    if SCENE_WORLD[scene] == 'W0':
        summary = make_w0(world, view)
    elif scene in FIELD_SCENES:
        summary, fw = make_field(world, view, field_spec, out_dir)
        raw_metadata = field_metadata(scene, raw_metadata, summary)
    else:
        summary = dict(source='orchard_gazebo.world_gen')
    cameras_removed = 0 if camera else remove_camera(world)
    rover = world.find("model[@name='%s']" % gz_model.MODEL_NAME)
    if rover is None:
        raise SceneError('assembled_world_has_no_ubts_rover')
    if SCENE_WORLD[scene] == 'W1':
        summary['spawn_height'] = fix_spawn_height(rover, raw_metadata)
    # Rewrite the assembled files at once without the GNSS origin, so no file of the scene folder keeps it.
    E.indent(tree.getroot())
    text = E.tostring(tree.getroot(), encoding='unicode')
    metadata, metadata_removed = strip_gnss_metadata(raw_metadata)
    metadata_text = json.dumps(metadata, indent=1, ensure_ascii=False) + '\n'
    hits = gnss_lint(text) + gnss_lint(metadata_text)
    if hits:
        raise SceneError('gnss_fields_left_in_the_scene:' + ','.join(hits))
    output.write_bytes(text.encode('utf-8'))
    metadata_path.write_bytes(metadata_text.encode('utf-8'))
    return dict(tree=tree, world=world, view=view, engine=engine, headless=headless, summary=summary,
                cameras_removed=cameras_removed, output=output, metadata=metadata, lidar_backend=lidar_backend,
                camera=camera, gnss_removed=dict(origin_elements=gnss_removed, metadata_keys=metadata_removed),
                field_world=fw, lidar_mount=lidar_mount)


def scene_manifest(scene, base, text, model_manifest):
    """Manifest dict of a written scene (base from assemble_base, text = the SDF text written)."""
    world = base['world']
    physics = world.find('physics')
    source = source_identity()
    commit, status_digest = source['source_commit'], source['source_status_sha256']
    view = base['view']
    return dict(
        schema=SCHEMA, scene=scene, world=world_label(scene, base['summary']), world_name=world.get('name'),
        sdf=base['output'].name, sdf_sha256=hashlib.sha256(text.encode('utf-8')).hexdigest(),
        physics=dict(max_step_size=physics.findtext('max_step_size'),
                     real_time_update_rate=physics.findtext('real_time_update_rate'),
                     real_time_factor=physics.findtext('real_time_factor')),
        renderer=dict(engine=base['engine'], headless=base['headless'], lidar_backend=base['lidar_backend'],
                      camera=base['camera'], cameras_removed=base['cameras_removed']),
        world_summary=base['summary'], assemble_metadata_preset=base['metadata'].get('preset'),
        gnss_removed=base['gnss_removed'], gnss_lint=gnss_lint(text),
        scene_profile_overrides=view.result_label().get('overrides'),
        model=model_manifest, source_commit=commit, source_status_sha256=status_digest,
        source_commit_origin=source['source_commit_origin'], source_files_sha256=source['source_files']['files_sha256'],
        source_file_count=source['source_files']['file_count'],
        result_flags=list(model_manifest.get('result_flags') or []),
        result_kind=view.result_label()['result_kind'], field_validated=False,
        lidar_mount=base.get('lidar_mount'))


def build_scene(scene, out_dir, *, profile=None, profile_path=None, lidar_backend='embree', render_engine=None,
                headless_rendering=None, camera=True, rover_config=None, model_options=None, field_spec=None):
    """Assemble one scene into out_dir. Returns the scene manifest (also written to scene_manifest.json).
    field_spec: the field_world spec of a field scene (field1010)."""
    base = assemble_base(scene, out_dir, profile=profile, profile_path=profile_path, lidar_backend=lidar_backend,
                         render_engine=render_engine, headless_rendering=headless_rendering, camera=camera,
                         rover_config=rover_config, field_spec=field_spec)
    model = base['world'].find("model[@name='%s']" % gz_model.MODEL_NAME)
    options = scene_options(scene)
    options.update(model_options or {})
    model_manifest = gz_model.steer_model_element(model, base['view'], **options)
    root = base['tree'].getroot()
    E.indent(root)
    text = E.tostring(root, encoding='unicode')
    if gnss_lint(text):
        raise SceneError('gnss_fields_left_in_the_scene:' + ','.join(gnss_lint(text)))
    base['output'].write_bytes(text.encode('utf-8'))
    manifest = scene_manifest(scene, base, text, model_manifest)
    (Path(out_dir) / 'scene_manifest.json').write_bytes(
        (json.dumps(manifest, indent=1, ensure_ascii=False, allow_nan=False) + '\n').encode('utf-8'))
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description='Build a steering Gazebo scene (design 8.3 P5).')
    parser.add_argument('--scene', choices=SCENES, required=True)
    parser.add_argument('--out', type=Path, required=True, help='output folder (outside the repository)')
    parser.add_argument('--profile', type=Path, help='vehicle profile YAML (default: shipped placeholder)')
    parser.add_argument('--lidar-backend', choices=LIDAR_BACKENDS, default='embree')
    parser.add_argument('--render-engine', choices=('ogre', 'ogre2'))
    parser.add_argument('--headless-rendering', choices=('0', '1'))
    parser.add_argument('--no-camera', action='store_true', help='remove the camera sensor (server-only runs)')
    parser.add_argument('--wheel-collision', choices=gz_model.WHEEL_COLLISION_SHAPES,
                        help='wheel collision shape (default: the model default %s)' % gz_model.DEFAULT_WHEEL_COLLISION)
    parser.add_argument('--field-spec', type=Path, help='field1010: field_world spec JSON file')
    parser.add_argument('--record', help='field1010 without --field-spec: catalogue record (e.g. 040927)')
    parser.add_argument('--corridor', help='field1010 without --field-spec: catalogue corridor id (e.g. c02)')
    parser.add_argument('--direction', choices=('fwd', 'rev'), default='fwd')
    parser.add_argument('--mode', choices=('R', 'T'), default='R', help='field1010: field world mode')
    args = parser.parse_args(argv)
    field_spec = None
    if args.scene in FIELD_SCENES:
        if args.field_spec is not None:
            field_spec = args.field_spec
        elif args.record and args.corridor:
            from field_world.scene_spec import catalogue_spec
            field_spec = catalogue_spec(args.record, args.corridor, args.direction, mode=args.mode)
        else:
            parser.error('field1010 needs --field-spec or --record and --corridor')
    manifest = build_scene(args.scene, args.out, profile_path=args.profile, lidar_backend=args.lidar_backend,
                           render_engine=args.render_engine,
                           headless_rendering=None if args.headless_rendering is None else args.headless_rendering == '1',
                           camera=not args.no_camera,
                           model_options=dict(wheel_collision=args.wheel_collision) if args.wheel_collision else None,
                           field_spec=field_spec)
    print(json.dumps(dict(scene=manifest['scene'], sdf=str(Path(args.out) / manifest['sdf']),
                          sdf_sha256=manifest['sdf_sha256'], drive_mode=manifest['model']['drive_mode'],
                          steer_control=manifest['model']['steer_control'],
                          wheel_collision=manifest['model']['wheel_collision'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
