"""Joint direct-drive demo of the steered rover in Gazebo (design 8.3 P5 'early demo').

관절 직접 조종 시연 (joint direct-drive demo). There is no V3 protocol, no emulator, no bridge and
no safety chain: the steering joint targets come from the shared steering actuator model (backend_api;
the Python L0 servo or the native L1 model in test_servo mode, as in gz_probe) and the drive duty goes
straight to the wheel controllers (velocity target duty x omega0, the MCU ramp of the profile applied).
A maneuver script (fixed-steer turns, a slalom, a straight run with gentle corrections, fixed-steer
circles) or the keyboard gives the equivalent steering target and the duty. It is never used for
evaluation. Every log line, image overlay, plot and caption carries the label.

Demo cameras: separate static models that hold one camera sensor and nothing else (no visual, no
collision, no mass). They are moved every physics step with set_world_pose_cmd so that they follow the
rover (chase, top, front) from its planar pose. The rover model of the scene is not changed (its
canonical SHA-256 is compared before and after) and the rover's own sensor camera stays as it is.

Engines:
  fixture   (run, circles) gz-sim in this process (gz.sim8.TestFixture), stepped in lockstep at the
            physics rate as fast as rendering allows; frames carry SIM time stamps (as gz_probe).
  server    (rtf, keyboard) `gz sim -s -r` at real-time pacing; a GUI client may attach.

Steering of the demo runs: the per-run zero offset and pot calibration residual are set to 0 by default
(DEMO_OVERRIDES, recorded in every result) so that a straight target drives straight and the circles
compare with R = L / tan(delta). --keep-zero-offset uses the profile draw.

Outputs go to a folder outside the repository: demo_world.sdf, scene_manifest.json (the base scene
manifest plus a 'demo' block), run.json, log.npz, PNG stills (overlay and raw), an MP4 (ffmpeg, libx264)
and plots (matplotlib). A Korean overlay font is read from UBTS_DEMO_FONT or --font; without it the
overlay is in English.

Placeholder-based SIM results ("자리값 기반 SIM 결과"); never field validation.
Needs the gz Python bindings (SIM image) for scene runs; the helpers need numpy only.
CLI: python3 -B -m steering.gz_direct_drive {scene,run,circles,plot,rtf,keyboard,check} --help
     (sim/ubts on sys.path)
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as E

import numpy as np

from . import backend_api as ba
from . import geometry as geo
from . import gz_assemble
from . import gz_model
from . import gz_probe
from .l0_servo import L0ServoBackend
from .vehicle_profile import load_profile

SCHEMA = 'ubts.steer_gz_demo.v1'
LABEL_KO = '관절 직접 조종 시연'
LABEL_EN = 'joint direct-drive demo'
LABEL_LINE_KO = '관절 직접 조종 시연 · 안전 사슬 없음 · 평가에 쓰지 않음'
LABEL_LINE_EN = 'JOINT DIRECT-DRIVE DEMO - no safety chain - not for evaluation'
LOG_PREFIX = '[관절 직접 조종 시연]'
RESULT_KIND = gz_probe.RESULT_KIND                  # 자리값 기반 SIM 결과
PLACEHOLDER_KO = 'SIM 자리값'
FONT_ENV = 'UBTS_DEMO_FONT'
WORLD_SCENES = {'w0': 'w0_force', 'w1': 'w1_force'}
WORLD_KO = {'w0': 'W0 평지', 'w1': 'W1 원본 과수원'}
WORLD_EN = {'w0': 'W0 flat', 'w1': 'W1 original orchard'}
CAMERA_MODEL_PREFIX = 'demo_cam_'
CAMERA_TOPIC = '/demo/camera/%s'
ROVER_CAMERA_TOPIC = '/sim/camera'
ROVER_VIEW = 'rover'
DEMO_OVERRIDES = {'steer_zero_offset_deg': 0.0, 'pot_cal_residual_deg': 0.0}
DEMO_OVERRIDE_LABEL = 'gz_demo_zero_offset_0'
STANDSTILL_SPEED_M_S = gz_probe.STANDSTILL_SPEED_M_S
STOPPED_SPEED_M_S = gz_probe.STOP_SPEED_M_S
LIFT_LIMIT_M = .002                 # W0 wheel lift report threshold (centre height - radius above the floor)
UNLOADED_TOL_RAD_S = .03            # a driven wheel this close to its no-load speed gives (almost) no traction
LOADED_MARGIN_RAD_S = .1            # ... while another driven wheel is this much slower (loaded): wheel off the ground
BACKENDS = ('l0', 'native')

# Demo camera views (SIM_ASSUMED display settings, not sensors of the vehicle).
#   follow: camera at `offset` and looking at `look`, both in the rover's planar frame (x forward, y left,
#           z up from base_footprint), moved every step with the rover.
#   top:    straight down above `point` of the rover frame at `height` (above the rover origin), moved with
#           the rover; image_up 'left': image right = rover forward, image up = rover left.
#   fixed:  straight down at `offset` (x, y, height) from the spawn pose in the spawn frame, not moved;
#           image right = spawn forward, image up = spawn left.
VIEWS = {
    'chase': dict(kind='follow', offset=(-2.05, 1.0, 1.2), look=(.35, .05, .2), hfov_deg=60.),
    # top: high and narrow (near orthographic). Lower and wider, the perspective of the 0.44 m deck hid the strips
    # of the wheels that stand out beside the body; from 10 m about 5 cm of each wheel shows.
    'top': dict(kind='top', point=(.0, 0.), height=10., image_up='left', hfov_deg=10.5),
    'front': dict(kind='follow', offset=(2.0, 1.2, .95), look=(.25, 0., .18), hfov_deg=60.),
    'overhead': dict(kind='fixed', offset=(3.3, 2.5, 11.), hfov_deg=60.),
}
# Per-world changes of the view settings: in W1 the follow cameras stay inside the 3.8 m lane (trunk rows at +-1.9 m).
WORLD_VIEW_OVERRIDES = {
    'w1': {'chase': dict(offset=(-2.1, .55, 1.25), look=(.4, 0., .2)),
           'front': dict(offset=(2.1, .6, .9), look=(.25, 0., .2))},
}
VIEW_KO = {'chase': '추격 시점', 'top': '위에서 본 시점', 'front': '앞쪽 시점', 'overhead': '고정 머리 위 시점',
           ROVER_VIEW: '차량 카메라(센서 그대로)'}
VIEW_EN = {'chase': 'chase view', 'top': 'top-down view', 'front': 'front view', 'overhead': 'fixed overhead view',
           ROVER_VIEW: 'rover camera (sensor)'}
# W0 demo ground grid (visual only, no collision): lines every GRID_STEP_M around the spawn, display aid only.
GRID_HALF_M = 10.
GRID_STEP_M = 1.
GRID_LINE_M = (.016, .002)          # line width and height (m); placed GRID_LINE_M[1] / 2 above the floor top
DEFAULT_SIZE = (1280, 720)
DEFAULT_FPS = 30.
VIDEO_SIZE = (960, 540)
INSET_SIZE = (352, 198)
VIDEO_MAX_BYTES = 10 * 1024 * 1024
SCRIPTS = ('w0_turns', 'w0_slalom', 'w0_circle', 'w1_straight')


class DemoError(RuntimeError):
    """A demo scene or run could not be built or run as asked."""


def log(message):
    print('%s %s' % (LOG_PREFIX, message), flush=True)


# =================================================================================================================
# small helpers (pure)
# =================================================================================================================
def _num(value):
    return gz_model._num(value)


def model_sha256(model):
    """SHA-256 of a model element with whitespace-only text and tails removed (indentation-independent)."""
    clean = copy.deepcopy(model)
    for element in clean.iter():
        if element.text is not None and not element.text.strip():
            element.text = None
        if element.tail is not None and not element.tail.strip():
            element.tail = None
    return hashlib.sha256(E.tostring(clean, encoding='unicode').encode('utf-8')).hexdigest()


def follow_pose(view, x, y, z, yaw):
    """World pose (x, y, z, roll, pitch, yaw) of a demo camera for a rover planar pose (for a fixed view: the
    spawn pose). Positive pitch looks down; straight down, the image up direction is the camera yaw."""
    spec = VIEWS[view] if isinstance(view, str) else view
    c, s = math.cos(yaw), math.sin(yaw)
    if spec['kind'] == 'top':
        px, py = spec['point']
        up = yaw + (math.pi / 2. if spec.get('image_up', 'forward') == 'left' else 0.)
        return (x + px * c - py * s, y + px * s + py * c, z + spec['height'], 0., math.pi / 2., up)
    if spec['kind'] == 'fixed':
        ox, oy, oz = spec['offset']
        return (x + ox * c - oy * s, y + ox * s + oy * c, z + oz, 0., math.pi / 2., yaw + math.pi / 2.)
    if spec['kind'] != 'follow':
        raise DemoError('unknown_view_kind:%r' % (spec['kind'],))
    ox, oy, oz = spec['offset']
    lx, ly, lz = spec['look']
    cx, cy, cz = x + ox * c - oy * s, y + ox * s + oy * c, z + oz
    tx, ty, tz = x + lx * c - ly * s, y + lx * s + ly * c, z + lz
    dx, dy, dz = tx - cx, ty - cy, tz - cz
    return (cx, cy, cz, 0., math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx))


def view_spec(view, world=None):
    """The view settings of a demo camera in a world (VIEWS with WORLD_VIEW_OVERRIDES)."""
    spec = dict(VIEWS[view])
    spec.update(WORLD_VIEW_OVERRIDES.get(world, {}).get(view, {}))
    return spec


def camera_model(view, size=DEFAULT_SIZE, fps=DEFAULT_FPS, pose=(0., 0., 3., 0., .5, 0.), hfov_deg=None):
    """A static model with one camera sensor and nothing else (no visual, collision or inertial)."""
    spec = VIEWS[view]
    hfov = math.radians(spec['hfov_deg'] if hfov_deg is None else hfov_deg)
    model = E.Element('model', name=CAMERA_MODEL_PREFIX + view)
    E.SubElement(model, 'static').text = 'true'
    E.SubElement(model, 'pose').text = ' '.join(_num(v) for v in pose)
    link = E.SubElement(model, 'link', name='link')
    sensor = E.SubElement(link, 'sensor', name='camera', type='camera')
    E.SubElement(sensor, 'always_on').text = 'true'
    E.SubElement(sensor, 'update_rate').text = _num(fps)
    E.SubElement(sensor, 'topic').text = CAMERA_TOPIC % view
    camera = E.SubElement(sensor, 'camera')
    E.SubElement(camera, 'horizontal_fov').text = _num(hfov)
    image = E.SubElement(camera, 'image')
    E.SubElement(image, 'width').text = str(int(size[0]))
    E.SubElement(image, 'height').text = str(int(size[1]))
    E.SubElement(image, 'format').text = 'R8G8B8'
    clip = E.SubElement(camera, 'clip')
    E.SubElement(clip, 'near').text = '0.05'
    E.SubElement(clip, 'far').text = '300'
    return model


def ground_grid_model(spawn, half_m=GRID_HALF_M, step_m=GRID_STEP_M):
    """Static model with thin visual-only lines (no collision) every step_m around the spawn point (W0 display aid)."""
    x0, y0, z0 = spawn[0], spawn[1], 0.
    width, height = GRID_LINE_M
    model = E.Element('model', name='demo_ground_grid')
    E.SubElement(model, 'static').text = 'true'
    E.SubElement(model, 'pose').text = '%s %s %s 0 0 0' % (_num(x0), _num(y0), _num(z0 + height / 2.))
    link = E.SubElement(model, 'link', name='link')
    count = int(round(2. * half_m / step_m))
    for axis in ('x', 'y'):
        for i in range(count + 1):
            offset = -half_m + i * step_m
            visual = E.SubElement(link, 'visual', name='grid_%s_%d' % (axis, i))
            E.SubElement(visual, 'cast_shadows').text = 'false'
            pose = (0., offset, 0.) if axis == 'x' else (offset, 0., 0.)
            E.SubElement(visual, 'pose').text = '%s %s %s 0 0 0' % tuple(_num(v) for v in pose)
            size = (2. * half_m, width, height) if axis == 'x' else (width, 2. * half_m, height)
            box = E.SubElement(E.SubElement(visual, 'geometry'), 'box')
            E.SubElement(box, 'size').text = ' '.join(_num(v) for v in size)
            material = E.SubElement(visual, 'material')
            shade = '.92 .92 .86 1' if abs(offset) < 1e-9 else '.62 .66 .55 1'
            E.SubElement(material, 'ambient').text = shade
            E.SubElement(material, 'diffuse').text = shade
    return model


def demo_profile(profile, keep_zero_offset=False):
    """The profile view of the demo runs (DEMO_OVERRIDES unless keep_zero_offset)."""
    if keep_zero_offset:
        return profile
    return profile.with_overrides(DEMO_OVERRIDES, label=DEMO_OVERRIDE_LABEL)


def make_backend(kind, view, seed):
    """The shared steering actuator model in test_servo mode: 'l0' (Python) or 'native' (C++ L1, needs g++)."""
    if kind == 'l0':
        backend = L0ServoBackend()
        backend.configure(view, seed)
        return backend
    if kind == 'native':
        from .actuator_link import NativeSteeringBackend
        return NativeSteeringBackend(view, seed, mode='test_servo')
    raise DemoError('unknown_backend:%r (choices %s)' % (kind, ', '.join(BACKENDS)))


def backend_info(backend):
    """Identity of the shared actuator model of a run (gz_probe.backend_identity): L0 info(), native result_label()
    and its build (build_key, binary_sha256, source_sha256, firmware_build_id). Review F2: the native backend has no
    info(), so the first version kept the class name only."""
    return gz_probe.backend_identity(backend)


class DutyRamp:
    """The MCU duty ramp of the profile (percent per second) applied in both directions."""

    def __init__(self, ctx, script_duty):
        self.ctx, self.script_duty = ctx, script_duty
        self.current = 0.
        self.last_t = None

    def __call__(self, t, obs=None):
        target = float(self.script_duty(t))
        if self.last_t is None:
            step = self.ctx.ramp_frac_per_s * self.ctx.dt
        else:
            step = self.ctx.ramp_frac_per_s * max(0., t - self.last_t)
        self.last_t = t
        self.current += max(-step, min(step, target - self.current))
        return self.ctx.drive_targets(self.current)


# =================================================================================================================
# maneuver scripts
# =================================================================================================================
def piecewise(t, schedule):
    return gz_probe.piecewise(t, schedule)


def make_script(name, ctx, *, steer_deg=None, speed=None, duty=None, still_every=None):
    """A maneuver script: dict(name, world, duration_s, steer(t) -> equivalent target deg, duty(t) -> fraction,
    stills [(t, tag, views)], video dict or None, hold (t0, t1) standstill window or None, description)."""
    if name == 'w0_turns':
        angle = 15. if steer_deg is None else float(steer_deg)
        d = ctx.duty_for_speed(.5 if speed is None else speed) if duty is None else float(duty)
        script = dict(world='w0', duration_s=32.,
                      steer=lambda t: piecewise(t, [(0., 0.), (.5, angle), (12., -angle)]),
                      duty=lambda t: d if 1. <= t < 24. else 0.,
                      stills=[(10.5, 'left%g' % angle, ('chase', 'top', 'front', 'overhead')),
                              (22.5, 'right%g' % angle, ('chase', 'top', 'front', 'overhead')),
                              (27., 'stopped', ('overhead', 'top'))],
                      video=dict(t0=3., t1=18., main='chase', inset='overhead'), hold=(28., 32.),
                      description='δ +%g° 왼쪽 원호 → -%g° 오른쪽 원호 → 정지(조향 유지)' % (angle, angle),
                      params=dict(steer_deg=angle, duty=d))
    elif name == 'w0_slalom':
        angle = 15. if steer_deg is None else float(steer_deg)
        d = ctx.duty_for_speed(.5 if speed is None else speed) if duty is None else float(duty)
        script = dict(world='w0', duration_s=26.,
                      steer=lambda t: angle * math.sin(2. * math.pi * (t - 1.) / 8.) if t >= 1. else 0.,
                      duty=lambda t: d if 1. <= t < 21. else 0.,
                      stills=[(3., 'slalom_left', ('chase', 'top')), (7., 'slalom_right', ('chase', 'top'))],
                      video=dict(t0=2., t1=17., main='chase', inset='top'), hold=(23., 26.),
                      description='사인 슬랄롬 ±%g°, 주기 8 s' % angle, params=dict(steer_deg=angle, duty=d))
    elif name == 'w0_circle':
        angle = 15. if steer_deg is None else float(steer_deg)
        d = ctx.duty_for_speed(.5 if speed is None else speed) if duty is None else float(duty)
        script = dict(world='w0', duration_s=30.,
                      steer=lambda t: angle if t >= .5 else 0.,
                      duty=lambda t: d if 1. <= t < 26. else 0.,
                      stills=[(20., 'circle%g' % angle, ('chase', 'top'))], video=None, hold=(28.5, 30.),
                      description='고정 조향 원 δ %+g°' % angle, params=dict(steer_deg=angle, duty=d))
    elif name == 'w1_straight':
        # duty 1.0 keeps about 0.8 m/s on the W1 slopes. The earlier rigid-axle model stalled here at duty 0.48
        # and 0.8 (a lifted wheel spun at its no-load speed); the front axle roll pivot fixed it (GZ_DEMO_KO.md 6.3)
        angle = 5. if steer_deg is None else float(steer_deg)
        d = 1. if duty is None else float(duty)
        a = angle
        schedule = [(0., 0.), (12., a), (14., -a), (18., a), (20., 0.), (26., -a), (28., a), (32., -a), (34., 0.)]
        script = dict(world='w1', duration_s=52.,
                      steer=lambda t: piecewise(t, schedule),
                      duty=lambda t: d if 1. <= t < 44. else 0.,
                      stills=[(13.2, 'w1_left%g' % a, ('chase', 'top', 'front', ROVER_VIEW)),
                              (27.2, 'w1_right%g' % a, ('chase', 'top', 'front', ROVER_VIEW))],
                      video=dict(t0=9., t1=24., main='chase', inset='top'), hold=(48., 52.),
                      description='열 사이 직진 + 작은 보정(±%g° 두 번) → 정지' % a,
                      params=dict(steer_deg=a, duty=d, schedule=schedule))
    else:
        raise DemoError('unknown_script:%r (choices %s)' % (name, ', '.join(SCRIPTS)))
    if still_every:
        views = sorted({v for _, _, vs in script['stills'] for v in vs})
        t = float(still_every)
        while t < script['duration_s']:
            script['stills'].append((t, 'every_%05.1f' % t, tuple(views)))
            t += float(still_every)
    script['name'] = name
    return script


# =================================================================================================================
# demo scene
# =================================================================================================================
def build_demo_scene(world, out_dir, *, profile=None, views=('chase', 'top'), size=DEFAULT_SIZE, fps=DEFAULT_FPS,
                     rover_camera=True, lidar_backend='embree', model_options=None, grid=None):
    """Build the base steering scene (gz_assemble) and add the demo cameras. Returns the demo scene manifest.

    out_dir/base/ holds the base scene as gz_assemble writes it; out_dir/demo_world.sdf is the demo world and
    out_dir/scene_manifest.json the base manifest with sdf = demo_world.sdf and a 'demo' block.
    model_options go to gz_model.steer_model_element (for example body_wheel_wells=False for a before image).
    grid (default: W0 only) adds visual-only floor lines every GRID_STEP_M as a motion and scale cue.
    """
    if world not in WORLD_SCENES:
        raise DemoError('unknown_world:%r (choices %s)' % (world, ', '.join(WORLD_SCENES)))
    for view in views:
        if view not in VIEWS:
            raise DemoError('unknown_view:%r (choices %s)' % (view, ', '.join(VIEWS)))
    out_dir = Path(out_dir)
    profile = load_profile() if profile is None else profile
    base = gz_assemble.build_scene(WORLD_SCENES[world], out_dir / 'base', profile=profile, camera=rover_camera,
                                   lidar_backend=lidar_backend, model_options=model_options)
    base_text = (out_dir / 'base' / base['sdf']).read_text(encoding='utf-8')
    root = E.fromstring(base_text)
    world_el = root.find('world')
    rover = world_el.find("model[@name='%s']" % gz_model.MODEL_NAME)
    rover_sha = model_sha256(rover)
    spawn = [float(v) for v in rover.findtext('pose').split()]
    cameras = []
    for view in views:
        spec = view_spec(view, world)
        pose = follow_pose(spec, spawn[0], spawn[1], spawn[2], spawn[5])
        world_el.append(camera_model(view, size, fps, pose, hfov_deg=spec['hfov_deg']))
        cameras.append(dict(view=view, model=CAMERA_MODEL_PREFIX + view, topic=CAMERA_TOPIC % view,
                            size=list(size), fps=float(fps), spec=spec, first_pose=list(pose),
                            moves=spec['kind'] != 'fixed'))
    grid = (world == 'w0') if grid is None else bool(grid)
    if grid:
        world_el.append(ground_grid_model(spawn))
    E.indent(root)
    text = E.tostring(root, encoding='unicode')
    again = E.fromstring(text).find("world/model[@name='%s']" % gz_model.MODEL_NAME)
    if model_sha256(again) != rover_sha:
        raise DemoError('the_rover_model_changed_while_adding_the_demo_cameras')
    hits = gz_assemble.gnss_lint(text)
    if hits:
        raise DemoError('gnss_fields_in_the_demo_world:' + ','.join(hits))
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / 'demo_world.sdf').write_bytes(text.encode('utf-8'))
    for name in ('world_metadata.json',):
        source = out_dir / 'base' / name
        if source.exists():
            shutil.copyfile(source, out_dir / name)
    manifest = dict(base)
    manifest['sdf'] = 'demo_world.sdf'
    manifest['sdf_sha256'] = hashlib.sha256(text.encode('utf-8')).hexdigest()
    manifest['demo'] = dict(
        schema=SCHEMA, label=LABEL_KO, label_en=LABEL_EN, world=world, evaluation_use=False,
        safety_chain=False, base_scene_dir='base', base_sdf_sha256=base['sdf_sha256'],
        rover_model_sha256=rover_sha, rover_model_unchanged=True, rover_camera=bool(rover_camera),
        spawn_pose=spawn, cameras=cameras,
        ground_grid=dict(step_m=GRID_STEP_M, half_m=GRID_HALF_M, line_m=list(GRID_LINE_M),
                         note='visual only, no collision; display aid') if grid else None,
        note='demo cameras are separate static models without visual, collision or mass; they follow the rover '
             'pose through set_world_pose_cmd (fixture) and stay at their first pose in server runs')
    _write_json(out_dir / 'scene_manifest.json', manifest)
    log('scene %s (%s): %s, cameras %s, rover model SHA-256 %s... unchanged' % (
        world, WORLD_KO[world], out_dir / 'demo_world.sdf', ','.join(views) or '-', rover_sha[:12]))
    return manifest


def read_demo_manifest(scene_dir):
    path = Path(scene_dir) / 'scene_manifest.json'
    if not path.exists():
        raise DemoError('no scene_manifest.json in %s (build the scene first)' % scene_dir)
    manifest = json.loads(path.read_text(encoding='utf-8'))
    if 'demo' not in manifest:
        raise DemoError('%s is not a demo scene (no demo block); build it with the scene command' % scene_dir)
    return manifest


def fixture_world(scene_dir, manifest):
    """Write demo_world_fixture.sdf: the demo world with real-time pacing off (lockstep fixture). Returns its path."""
    scene_dir = Path(scene_dir)
    tree = E.parse(str(scene_dir / manifest['sdf']))
    physics = tree.getroot().find('world/physics')
    physics.find('real_time_factor').text = '0'
    physics.find('real_time_update_rate').text = '0'
    E.indent(tree.getroot())
    path = scene_dir / 'demo_world_fixture.sdf'
    path.write_bytes(E.tostring(tree.getroot(), encoding='unicode').encode('utf-8'))
    return path, float(physics.findtext('max_step_size'))


# =================================================================================================================
# images: overlay, stills, video
# =================================================================================================================
def font_path(explicit=None):
    path = explicit or os.environ.get(FONT_ENV)
    if path and Path(path).is_file():
        return str(path)
    return None


COLLISION_KO = {'sphere': '바퀴 충돌 구', 'cylinder': '바퀴 충돌 원통'}
COLLISION_EN = {'sphere': 'wheel collision sphere', 'cylinder': 'wheel collision cylinder'}


def collision_note(shape, korean):
    """Text of the rover's wheel collision shape (gz_model wheel_collision) for overlays and plot titles, or None."""
    if not shape:
        return None
    return (COLLISION_KO if korean else COLLISION_EN).get(shape, str(shape))


def overlay_lines(korean, *, world, view, profile_id, t, target_deg, left_deg, right_deg, duty, speed,
                  extra=None, wheel_collision=None):
    """Overlay text lines (Korean if a Korean font is available, else English). wheel_collision: the rover's wheel
    collision shape, shown after the profile id when given."""
    note = collision_note(wheel_collision, korean)
    if korean:
        lines = [LABEL_LINE_KO,
                 '%s (%s)%s · %s · %s' % (PLACEHOLDER_KO, profile_id, ' · ' + note if note else '', WORLD_KO[world],
                                          VIEW_KO.get(view, view)),
                 't %.2f s · 조향 목표(등가) %+.1f° · 바퀴각 좌 %+.1f° / 우 %+.1f° · duty %.2f · %.2f m/s'
                 % (t, target_deg, left_deg, right_deg, duty, speed)]
    else:
        lines = [LABEL_LINE_EN,
                 'SIM placeholder (%s)%s - %s - %s' % (profile_id, ' - ' + note if note else '', WORLD_EN[world],
                                                       VIEW_EN.get(view, view)),
                 't %.2f s - steer target (eq) %+.1f deg - wheels L %+.1f / R %+.1f deg - duty %.2f - %.2f m/s'
                 % (t, target_deg, left_deg, right_deg, duty, speed)]
    if extra:
        lines.append(extra)
    return lines


def draw_overlay(image, lines, font_file=None):
    """PIL image with the text lines on a dark band at the top left."""
    from PIL import Image, ImageDraw, ImageFont
    rgba = image.convert('RGBA')
    layer = Image.new('RGBA', rgba.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    size = max(11, int(round(rgba.size[1] / 36.)))
    if font_file:
        font = ImageFont.truetype(font_file, size)
    else:
        try:
            font = ImageFont.load_default(size=size)
        except TypeError:                         # Pillow < 10.1
            font = ImageFont.load_default()
    pad = max(4, size // 2)
    step = int(round(size * 1.38))
    width = max(draw.textlength(line, font=font) for line in lines)
    draw.rectangle([0, 0, int(width) + 2 * pad, len(lines) * step + pad], fill=(0, 0, 0, 150))
    y = pad // 2 + 1
    for i, line in enumerate(lines):
        draw.text((pad, y), line, font=font, fill=(255, 235, 120, 255) if i == 0 else (255, 255, 255, 255))
        y += step
    return Image.alpha_composite(rgba, layer).convert('RGB')


def _frame_image(frame):
    from PIL import Image
    stamp, width, height, data = frame
    return Image.frombytes('RGB', (width, height), data)


class FrameSink:
    """Collects demo camera frames in the render thread and writes stills and the video in the main thread."""

    def __init__(self, out_dir, script, state_at, *, world, profile_id, font_file, video=True, fps=DEFAULT_FPS,
                 dt=1. / 120., wheel_collision=None):
        self.out_dir = Path(out_dir)
        self.script, self.state_at, self.world, self.profile_id = script, state_at, world, profile_id
        self.wheel_collision = wheel_collision
        self.font_file = font_file
        self.korean = font_file is not None
        self.video = script.get('video') if video else None
        self.fps = float(fps)
        self.dt = dt
        self.lock = threading.Lock()
        self.pending = []
        self.stills_wanted = {}
        for t_req, tag, views in script['stills']:
            for view in views:
                self.stills_wanted.setdefault(view, []).append([t_req, tag, None])
        self.stills = []
        self.inset = {}
        self.ffmpeg = None
        self.video_frames = 0
        self.video_path = None
        self.video_error = None
        self.frame_counts = {}
        self.last_stamp = {}
        self.stamp_gaps = {}

    def wanted(self, view, stamp):
        video = self.video
        if video and view in (video['main'], video.get('inset')) and video['t0'] - .1 <= stamp <= video['t1'] + .1:
            return True
        for t_req, _, done in self.stills_wanted.get(view, ()):
            if done is None and t_req - 1e-6 <= stamp <= t_req + .25:
                return True
        return False

    def callback(self, view):
        def receive(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            with self.lock:
                self.frame_counts[view] = self.frame_counts.get(view, 0) + 1
                last = self.last_stamp.get(view)
                if last is not None:
                    gap = stamp - last
                    self.stamp_gaps.setdefault(view, []).append(gap)
                self.last_stamp[view] = stamp
            if not self.wanted(view, stamp):
                return
            if msg.pixel_format_type not in (0, 3):           # 3 = RGB_INT8 (gz.msgs PixelFormatType)
                return
            frame = (stamp, int(msg.width), int(msg.height), bytes(msg.data))
            with self.lock:
                self.pending.append((view, frame))
        return receive

    def _overlay_for(self, view, stamp, image, extra=None):
        state = self.state_at(stamp)
        lines = overlay_lines(self.korean, world=self.world, view=view, profile_id=self.profile_id, t=stamp,
                              target_deg=state['target_deg'], left_deg=state['left_deg'],
                              right_deg=state['right_deg'], duty=state['duty'], speed=state['speed'], extra=extra,
                              wheel_collision=self.wheel_collision)
        return draw_overlay(image, lines, self.font_file), state

    def process(self, horizon, final=False):
        """Handle frames with stamp <= horizon (all of them when final)."""
        with self.lock:
            ready = [p for p in self.pending if final or p[1][0] <= horizon]
            self.pending = [p for p in self.pending if not (final or p[1][0] <= horizon)]
        ready.sort(key=lambda p: (p[1][0], p[0] != (self.video or {}).get('inset')))
        for view, frame in ready:
            stamp = frame[0]
            for item in self.stills_wanted.get(view, ()):
                t_req, tag, done = item
                if done is None and t_req - 1e-6 <= stamp <= t_req + .25:
                    item[2] = self._save_still(view, tag, frame)
            video = self.video
            if not video or not video['t0'] - 1e-6 <= stamp <= video['t1'] + 1e-6:
                continue
            if view == video.get('inset'):
                self.inset = dict(stamp=stamp, frame=frame)
            elif view == video['main']:
                self._video_frame(frame)

    def _save_still(self, view, tag, frame):
        stills_dir = self.out_dir / 'stills'
        stills_dir.mkdir(parents=True, exist_ok=True)
        image = _frame_image(frame)
        raw = stills_dir / ('%s_%s_raw.png' % (tag, view))
        image.save(raw)
        shown, state = self._overlay_for(view, frame[0], image)
        path = stills_dir / ('%s_%s.png' % (tag, view))
        shown.save(path)
        row = dict(view=view, tag=tag, stamp_s=frame[0], path=str(path), raw_path=str(raw), size=[frame[1], frame[2]],
                   state=state, label=LABEL_KO)
        self.stills.append(row)
        log('still %s %s at t %.2f s (target %+.1f deg, wheels %+.1f/%+.1f deg)' % (
            tag, view, frame[0], state['target_deg'], state['left_deg'], state['right_deg']))
        return path

    def _video_frame(self, frame):
        from PIL import Image
        if self.video_error:
            return
        if self.ffmpeg is None:
            ffmpeg = shutil.which('ffmpeg')
            if ffmpeg is None:
                self.video_error = 'ffmpeg not found'
                return
            self.video_path = self.out_dir / ('%s.mp4' % self.script['name'])
            command = [ffmpeg, '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                       '-s', '%dx%d' % VIDEO_SIZE, '-r', _num(self.fps), '-i', '-', '-an', '-c:v', 'libx264',
                       '-preset', 'medium', '-crf', '23', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                       str(self.video_path)]
            self.ffmpeg = subprocess.Popen(command, stdin=subprocess.PIPE)
        image = _frame_image(frame).resize(VIDEO_SIZE, Image.LANCZOS)
        if self.inset:
            inset = _frame_image(self.inset['frame']).resize(INSET_SIZE, Image.LANCZOS)
            x0, y0 = VIDEO_SIZE[0] - INSET_SIZE[0] - 12, VIDEO_SIZE[1] - INSET_SIZE[1] - 12
            border = Image.new('RGB', (INSET_SIZE[0] + 4, INSET_SIZE[1] + 4), (255, 255, 255))
            image.paste(border, (x0 - 2, y0 - 2))
            image.paste(inset, (x0, y0))
        inset_view = self.video.get('inset')
        extra = (('작은 화면: %s' % VIEW_KO.get(inset_view, inset_view)) if self.korean else
                 ('inset: %s' % VIEW_EN.get(inset_view, inset_view))) if self.inset else None
        shown, _ = self._overlay_for(self.video['main'], frame[0], image, extra=extra)
        try:
            self.ffmpeg.stdin.write(shown.tobytes())
            self.video_frames += 1
        except (BrokenPipeError, OSError) as exc:
            self.video_error = 'ffmpeg pipe: %s' % exc

    def close(self):
        result = dict(frames=self.video_frames, path=str(self.video_path) if self.video_path else None,
                      error=self.video_error, size=list(VIDEO_SIZE), fps=self.fps)
        if self.ffmpeg is not None:
            try:
                self.ffmpeg.stdin.close()
            except OSError:
                pass
            code = self.ffmpeg.wait(120)
            result['ffmpeg_exit'] = code
            if self.video_path and self.video_path.exists():
                result['bytes'] = self.video_path.stat().st_size
                result['duration_s'] = self.video_frames / self.fps
                result['within_limits'] = bool(result['bytes'] <= VIDEO_MAX_BYTES and result['duration_s'] <= 20.)
        gaps = {}
        with self.lock:
            for view, values in self.stamp_gaps.items():
                arr = np.asarray(values, dtype=float)
                gaps[view] = dict(frames=self.frame_counts.get(view, 0), gap_median_s=float(np.median(arr)),
                                  gap_max_s=float(np.max(arr)), gap_min_s=float(np.min(arr)))
        missing = [dict(view=view, t=t_req, tag=tag) for view, rows in self.stills_wanted.items()
                   for t_req, tag, done in rows if done is None]
        return dict(video=result if self.video else None, stills=self.stills, missing_stills=missing, cameras=gaps)


# =================================================================================================================
# fixture run with cameras
# =================================================================================================================
LOG_KEYS = ('k', 't', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'speed', 'wz', 'target_eq', 'model_eq', 'model_left',
            'model_right', 'steer_cmd_left', 'steer_cmd_right', 'q_left', 'q_right', 'qd_left', 'qd_right', 'duty')


def run_demo(scene_dir, script_name, out_dir, *, profile=None, backend='l0', seed=0, video=True, font=None,
             keep_zero_offset=False, still_every=None, script_options=None, chunk_steps=12):
    """Run a maneuver script in the lockstep fixture with the demo cameras. Returns the run record (run.json)."""
    import gz.math7 as gmath
    import gz.sim8 as sim
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.image_pb2 import Image as ImageMsg
    from gz.transport13 import Node

    scene_dir, out_dir = Path(scene_dir), Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = read_demo_manifest(scene_dir)
    demo = manifest['demo']
    base_profile = load_profile() if profile is None else profile
    if manifest['model']['profile']['profile_sha256'] != base_profile.sha256:
        raise DemoError('the scene was built with another profile (%s...)'
                        % manifest['model']['profile']['profile_sha256'][:12])
    view_profile = demo_profile(gz_assemble.scene_profile(manifest['scene'], base_profile), keep_zero_offset)
    sdf, step = fixture_world(scene_dir, manifest)
    dt = int(step * 1e9) / 1e9
    ctx = gz_probe.Context(view_profile, dt)
    script = make_script(script_name, ctx, still_every=still_every, **(script_options or {}))
    if script['world'] != demo['world']:
        raise DemoError('script %s is for %s, the scene is %s' % (script_name, script['world'], demo['world']))
    available = {c['view'] for c in demo['cameras']} | ({ROVER_VIEW} if demo.get('rover_camera') else set())
    script['stills'] = [(t_req, tag, tuple(v for v in views if v in available))
                        for t_req, tag, views in script['stills']]
    script['stills'] = [s for s in script['stills'] if s[2]]
    if script['video'] and script['video']['main'] not in available:
        script['video'] = None
    elif script['video'] and script['video'].get('inset') not in available:
        script['video'] = dict(script['video'], inset=None)
    font_file = font_path(font)
    if font_file is None:
        log('no Korean font (set %s or --font); overlays are in English' % FONT_ENV)
    radius = {w: view_profile.value('wheel_radius_front_m' if 'front' in w else 'wheel_radius_rear_m')
              for w in gz_model.WHEELS}
    topics = manifest['model']['topics']
    steer_backend = make_backend(backend, view_profile, seed)
    info_backend = backend_info(steer_backend)
    duty_ramp = DutyRamp(ctx, script['duty'])
    controller = gz_probe.ModelSteerController(ctx, steer_backend, lambda t: math.radians(script['steer'](t)),
                                               duty_ramp)
    rows = {key: [] for key in LOG_KEYS}
    wheel_rows = {w: dict(qd=[], z=[], slip=[], susp=[]) for w in gz_model.WHEELS}
    state = dict(ready=False, cams={}, cam_pose={}, errors=[], steer_in_force={}, last_k=0,
                 model_in_force=(0, 0., 0., 0., 0., 0., 0., 0, 0.), duty_in_force=0.)
    base_offset = _base_link_offset(scene_dir, manifest)
    lock = threading.Lock()

    def state_at(stamp):
        k = int(round(stamp / dt))
        with lock:
            n = len(rows['k'])
            if n == 0:
                return dict(target_deg=0., left_deg=0., right_deg=0., duty=0., speed=0., k=k)
            i = min(max(0, k - int(rows['k'][0])), n - 1)
            return dict(k=int(rows['k'][i]), t=float(rows['t'][i]), target_deg=math.degrees(rows['target_eq'][i]),
                        left_deg=math.degrees(rows['q_left'][i]), right_deg=math.degrees(rows['q_right'][i]),
                        duty=float(rows['duty'][i]), speed=float(rows['speed'][i]), x=float(rows['x'][i]),
                        y=float(rows['y'][i]), yaw_deg=math.degrees(rows['yaw'][i]))

    wheel_collision = manifest['model'].get('wheel_collision')
    sink = FrameSink(out_dir, script, state_at, world=demo['world'], profile_id=view_profile.profile_id,
                     font_file=font_file, video=video, fps=demo['cameras'][0]['fps'] if demo['cameras'] else 30.,
                     dt=dt, wheel_collision=wheel_collision)
    node = Node()
    views = [c['view'] for c in demo['cameras']]
    moving_views = [c['view'] for c in demo['cameras'] if c.get('moves', True)]
    specs = {c['view']: c.get('spec') or VIEWS[c['view']] for c in demo['cameras']}
    for view in views:
        node.subscribe(ImageMsg, CAMERA_TOPIC % view, sink.callback(view))
    if demo.get('rover_camera'):
        node.subscribe(ImageMsg, ROVER_CAMERA_TOPIC, sink.callback(ROVER_VIEW))
    pub_steer = {key: node.advertise(topic, Double) for key, topic in topics['steer'].items()}
    pub_drive = {wheel: node.advertise(topic, Double) for wheel, topic in topics['drive'].items()}

    def pre(info, ecm):
        try:
            if not state['ready']:
                world = sim.World(sim.world_entity(ecm))
                model = sim.Model(world.model_by_name(ecm, gz_model.MODEL_NAME))
                state['base'] = sim.Link(model.link_by_name(ecm, 'base_link'))
                state['base'].enable_velocity_checks(ecm)
                state['wheels'] = {}
                for wheel in gz_model.WHEELS:
                    link = sim.Link(model.link_by_name(ecm, wheel))
                    link.enable_velocity_checks(ecm)
                    joint = sim.Joint(model.joint_by_name(ecm, wheel + '_joint'))
                    joint.enable_velocity_check(ecm)
                    state['wheels'][wheel] = (link, joint)
                state['susp'] = {}
                for wheel in gz_model.WHEELS:
                    entity = model.joint_by_name(ecm, gz_model.SUSPENSION_JOINT % wheel)
                    if entity != sim.K_NULL_ENTITY:
                        joint = sim.Joint(entity)
                        joint.enable_position_check(ecm)
                        state['susp'][wheel] = joint
                state['steer'] = {}
                for item in manifest['model']['steering']:
                    joint = sim.Joint(model.joint_by_name(ecm, item['joint']))
                    joint.enable_position_check(ecm)
                    joint.enable_velocity_check(ecm)
                    state['steer'][item['key']] = joint
                for view in moving_views:
                    state['cams'][view] = sim.Model(world.model_by_name(ecm, CAMERA_MODEL_PREFIX + view))
                state['ready'] = True
            for view, pose in state['cam_pose'].items():
                state['cams'][view].set_world_pose_cmd(ecm, gmath.Pose3d(*pose))
        except Exception as exc:                  # keep the server alive; report after the run
            if len(state['errors']) < 20:
                state['errors'].append('pre %s: %s' % (type(exc).__name__, exc))

    def post(info, ecm):
        try:
            k = int(info.iterations)
            t = k * dt
            state['last_k'] = k
            pose = state['base'].world_pose(ecm)
            p, r = pose.pos(), pose.rot()
            q = (r.w(), r.x(), r.y(), r.z())
            off = gz_probe.quaternion_rotate(q, base_offset)
            x, y, z = p.x() - off[0], p.y() - off[1], p.z() - off[2]
            roll, pitch, yaw = gz_probe.quaternion_rpy(q)
            lin = state['base'].world_linear_velocity(ecm)
            ang = state['base'].world_angular_velocity(ecm)
            vx, vy = (lin.x(), lin.y()) if lin is not None else (0., 0.)
            wz = ang.z() if ang is not None else 0.
            speed = vx * math.cos(yaw) + vy * math.sin(yaw)
            steer_q, steer_qd = {}, {}
            for key, joint in state['steer'].items():
                pos, vel = joint.position(ecm), joint.velocity(ecm)
                steer_q[key] = pos[0] if pos else float('nan')
                steer_qd[key] = vel[0] if vel else float('nan')
            wheel_now = {}
            for wheel, (link, joint) in state['wheels'].items():
                wpose = link.world_pose(ecm)
                wvel = link.world_linear_velocity(ecm)
                qd = joint.velocity(ecm)
                omega = qd[0] if qd else 0.
                wr = wpose.rot()
                axle = gz_probe.quaternion_rotate((wr.w(), wr.x(), wr.y(), wr.z()), (0., 1., 0.))
                forward = (axle[1], -axle[0], 0.)                 # axle x up
                norm = math.hypot(forward[0], forward[1]) or 1.
                v_roll = ((wvel.x() * forward[0] + wvel.y() * forward[1]) / norm) if wvel is not None else 0.
                rim = omega * radius[wheel]
                slip = (rim - v_roll) / max(abs(v_roll), abs(rim), .05)
                susp = state['susp'].get(wheel)
                travel = susp.position(ecm) if susp is not None else None
                wheel_now[wheel] = (omega, wpose.pos().z(), slip, travel[0] if travel else float('nan'))
            obs = dict(t=t, speed=speed, wheel_velocity={w + '_joint': v[0] for w, v in wheel_now.items()},
                       steer_position=steer_q)
            cmd = controller.command(k + 1, (k + 1) * dt, obs)
            for key, value in cmd['steer'].items():
                msg = Double()
                msg.data = float(value)
                pub_steer[key].publish(msg)
            for wheel, value in (cmd['drive'] or {}).items():
                msg = Double()
                msg.data = float(value)
                pub_drive[wheel].publish(msg)
            # values in force during step k (published after step k-1); the new ones apply to step k+1
            model_row, in_force, duty_now = state['model_in_force'], dict(state['steer_in_force']), state['duty_in_force']
            if controller.rows:
                state['model_in_force'] = controller.rows[-1]
            state['steer_in_force'] = {key: float(value) for key, value in cmd['steer'].items()}
            state['duty_in_force'] = duty_ramp.current
            with lock:
                for name, value in (('k', k), ('t', t), ('x', x), ('y', y), ('z', z), ('roll', roll),
                                    ('pitch', pitch), ('yaw', yaw), ('speed', speed), ('wz', wz),
                                    ('target_eq', math.radians(script['steer'](t))), ('model_eq', model_row[3]),
                                    ('model_left', model_row[4]), ('model_right', model_row[5]),
                                    ('steer_cmd_left', in_force.get('left', in_force.get('axle', 0.))),
                                    ('steer_cmd_right', in_force.get('right', in_force.get('axle', 0.))),
                                    ('q_left', steer_q.get('left', steer_q.get('axle', float('nan')))),
                                    ('q_right', steer_q.get('right', steer_q.get('axle', float('nan')))),
                                    ('qd_left', steer_qd.get('left', steer_qd.get('axle', float('nan')))),
                                    ('qd_right', steer_qd.get('right', steer_qd.get('axle', float('nan')))),
                                    ('duty', duty_now)):
                    rows[name].append(value)
                for wheel, (omega, wz_, slip, travel) in wheel_now.items():
                    wheel_rows[wheel]['qd'].append(omega)
                    wheel_rows[wheel]['z'].append(wz_)
                    wheel_rows[wheel]['slip'].append(slip)
                    wheel_rows[wheel]['susp'].append(travel)
            for view in moving_views:
                state['cam_pose'][view] = follow_pose(specs[view], x, y, z, yaw)
        except Exception as exc:                  # keep the server alive; report after the run
            if len(state['errors']) < 20:
                state['errors'].append('post %s: %s' % (type(exc).__name__, exc))

    log('run %s in %s (%s), backend %s, duration %.1f s SIM, profile %s, overrides %s' % (
        script_name, demo['world'], WORLD_KO[demo['world']], backend, script['duration_s'], view_profile.profile_id,
        'none' if keep_zero_offset else DEMO_OVERRIDE_LABEL))
    fixture = sim.TestFixture(str(sdf))
    fixture.on_pre_update(pre)
    fixture.on_post_update(post)
    fixture.finalize()
    server = fixture.server()
    total = int(round(script['duration_s'] / dt))
    started = time.perf_counter()
    done = 0
    last_report = started
    while done < total:
        n = min(chunk_steps, total - done)
        server.run(True, n, False)
        done += n
        sink.process(done * dt - 3. / sink.fps)
        if state['errors']:
            break
        now = time.perf_counter()
        if now - last_report > 10.:
            last_report = now
            log('%.1f/%.1f s SIM, wall %.1f s' % (done * dt, total * dt, now - started))
    time.sleep(.5)
    sink.process(done * dt, final=True)
    wall = time.perf_counter() - started
    controller.close()
    frames = sink.close()
    arrays = {key: np.asarray(values, dtype=float) for key, values in rows.items()}
    for wheel, values in wheel_rows.items():
        for key, series in values.items():
            arrays['wheel_%s_%s' % (wheel, key)] = np.asarray(series, dtype=float)
    np.savez_compressed(out_dir / 'log.npz', **arrays)
    analysis = analyze_run(arrays, ctx, script, demo['world'], radius) if len(arrays['t']) > 10 else None
    record = dict(schema=SCHEMA, label=LABEL_KO, label_en=LABEL_EN, evaluation_use=False, safety_chain=False,
                  result_kind=RESULT_KIND, field_validated=False, engine='fixture', script=dict(
                      name=script_name, description=script['description'], params=script['params'],
                      duration_s=script['duration_s'], stills=[list(s[:2]) + [list(s[2])] for s in script['stills']],
                      video=script['video'], hold=script['hold']),
                  world=demo['world'], scene_dir=str(scene_dir), scene_sdf_sha256=manifest['sdf_sha256'],
                  rover_model_sha256=demo['rover_model_sha256'], backend=dict(kind=backend, seed=seed,
                                                                                info=info_backend),
                  profile=view_profile.result_label(), demo_overrides=None if keep_zero_offset else DEMO_OVERRIDES,
                  wheel_collision=wheel_collision, model_result_flags=list(manifest.get('result_flags') or []),
                  scene_source=dict(commit=manifest.get('source_commit'),
                                    files_sha256=manifest.get('source_files_sha256')),
                  provenance=gz_probe.provenance(Path(scene_dir) / manifest['sdf'], dict(rover=info_backend)),
                  run=dict(steps=done, sim_s=done * dt, wall_s=wall, sim_per_wall=done * dt / wall if wall else None,
                           dt_s=dt, errors=state['errors']),
                  font=font_file, frames=frames, analysis=analysis)
    _write_json(out_dir / 'run.json', record)
    log('done: %d steps, %.1f s SIM in %.1f s wall, errors %d, stills %d, video %s' % (
        done, done * dt, wall, len(state['errors']), len(frames['stills']),
        (frames['video'] or {}).get('path')))
    try:                                           # plots while the server is alive (see main: no teardown)
        if demo['world'] == 'w1':
            record['plot'] = plot_w1(out_dir, scene_dir, out_dir / 'w1_plot.png', font=font)
        else:
            record['plot'] = plot_run(out_dir, out_dir / 'run_plot.png', font=font)
        log('plot %s' % record['plot'])
    except Exception as exc:                       # plots are optional
        log('plot failed: %s: %s' % (type(exc).__name__, exc))
    _KEEP_ALIVE.append(fixture)
    return record


# The gz-sim server of a rendering fixture is kept until the process ends; the run command then leaves with os._exit
# (outputs are complete by then). Destroying a server with the Sensors system and Ogre ended the process without a
# message inside the SIM container before the plots were written.
_KEEP_ALIVE = []


def _base_link_offset(scene_dir, manifest):
    """base_link position in the rover model frame (base_footprint) from the demo world."""
    tree = E.parse(str(Path(scene_dir) / manifest['sdf']))
    link = tree.getroot().find("world/model[@name='%s']/link[@name='base_link']" % gz_model.MODEL_NAME)
    return tuple(float(v) for v in link.findtext('pose').split()[:3])


# =================================================================================================================
# analysis of a run log (numpy only)
# =================================================================================================================
def _segments(mask):
    """(start, stop) index pairs of the True runs of a boolean array (stop exclusive)."""
    mask = np.asarray(mask, dtype=bool)
    if mask.size == 0:
        return []
    edges = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    return list(zip(np.nonzero(edges == 1)[0], np.nonzero(edges == -1)[0]))


def analyze_run(a, ctx, script, world, radius):
    """Model-defect checks of a demo run (report): turning sign, steering tracking and jitter, wheel lift and
    slip, standstill drift. Never an acceptance verdict."""
    t, dt = a['t'], ctx.dt
    out = dict(note='report only (joint direct-drive demo); acceptance is judged by gz_probe')
    model_eq = a['model_eq']
    moving = np.abs(a['speed']) > .05
    turning = moving & (np.abs(model_eq) > math.radians(2.))
    # steady turning: model angle constant within 0.05 deg over the last 0.5 s
    steady = np.zeros_like(turning)
    w = int(round(.5 / dt))
    for i in range(w, len(t)):
        if turning[i] and np.ptp(model_eq[i - w:i + 1]) < math.radians(.05):
            steady[i] = True
    if steady.any():
        sign_ok = np.sign(a['wz'][steady]) == np.sign(model_eq[steady])
        curvature = a['wz'][steady] / np.where(np.abs(a['speed'][steady]) > 1e-6, a['speed'][steady], np.nan)
        expected = np.tan(model_eq[steady]) / ctx.geometry.wheelbase_m
        out['turning'] = dict(samples=int(steady.sum()), yaw_rate_sign_agrees_fraction=float(np.mean(sign_ok)),
                              left_steer_turns_counter_clockwise=bool(np.all(
                                  (a['wz'][steady & (model_eq > 0)] > 0)) if (steady & (model_eq > 0)).any() else None),
                              curvature_ratio_median=float(np.nanmedian(curvature / expected)),
                              note='curvature ratio = measured wz / v over tan(delta_eq) / L (a = 0.5 linkage, '
                                   'force-mode tyres; see GZ_MODEL_KO.md 7)')
    # steering joint tracking (joint vs the target in force) and jitter at constant targets
    track, jitter = {}, {}
    for side in ('left', 'right'):
        err = np.degrees(a['q_%s' % side][2:] - a['steer_cmd_%s' % side][2:])
        track[side] = gz_probe.percentile_stats(err)
        cmd = a['steer_cmd_%s' % side]
        const = np.zeros(len(t), bool)
        const[1:] = np.abs(np.diff(cmd)) < 1e-12
        holds = [(s, e) for s, e in _segments(const) if e - s >= int(round(1. / dt))]
        sd = [float(np.degrees(np.std(a['q_%s' % side][s + 12:e]))) for s, e in holds if e - s > 24]
        qd = [float(np.max(np.abs(a['qd_%s' % side][s + 12:e]))) for s, e in holds if e - s > 24]
        jitter[side] = dict(hold_windows=len(holds), angle_sd_deg_max=max(sd) if sd else None,
                            joint_speed_abs_max_rad_s=max(qd) if qd else None)
    out['steering_tracking_deg'] = track
    out['steering_jitter_at_constant_target'] = jitter
    out['steering_joint_speed_abs_max_rad_s'] = float(np.nanmax(np.abs(np.concatenate([a['qd_left'], a['qd_right']]))))
    # wheels: lift on flat W0 (centre height - radius), slip ratio while moving, joint speed spread
    wheels = {}
    for wheel in gz_model.WHEELS:
        z = a['wheel_%s_z' % wheel]
        slip = a['wheel_%s_slip' % wheel]
        row = dict(slip_abs_p50=float(np.median(np.abs(slip[moving]))) if moving.any() else None,
                   slip_abs_p95=float(np.percentile(np.abs(slip[moving]), 95)) if moving.any() else None)
        if world == 'w0':
            settled = t > .5
            height = z[settled] - radius[wheel]
            row.update(centre_minus_radius_max_m=float(np.max(height)), centre_minus_radius_min_m=float(np.min(height)),
                       lifted_samples=int(np.sum(height > LIFT_LIMIT_M)), lift_limit_m=LIFT_LIMIT_M)
        wheels[wheel] = row
    # wheel without traction: in force mode a wheel with no longitudinal ground force settles at the no-load speed of
    # the DC line, duty * omega0 - friction / p (time constant J / p, about 0.06 s). On flat ground at a steady speed
    # every wheel is there (no rolling resistance in the SIM), so a wheel is flagged only while another driven wheel
    # is clearly loaded (slower than the no-load speed): the signature of a wheel off the ground on terrain.
    line = ctx.line
    driving = (a['duty'] > line.deadband_frac + .05) & moving
    noload = a['duty'] * line.omega0_rad_s - line.coulomb_friction_nm / line.p_gain
    speeds = {wheel: a['wheel_%s_qd' % wheel] for wheel in ctx.driven}
    unloaded = {}
    any_unloaded = np.zeros(len(t), bool)
    for wheel in ctx.driven:
        others = [noload - speeds[w] for w in ctx.driven if w != wheel]
        loaded_other = np.max(others, axis=0) > LOADED_MARGIN_RAD_S if others else np.zeros(len(t), bool)
        flag = driving & loaded_other & (np.abs(speeds[wheel] - noload) < UNLOADED_TOL_RAD_S)
        any_unloaded |= flag
        unloaded[wheel] = float(np.mean(flag[driving])) if driving.any() else None
    out['unloaded_wheels'] = dict(
        rule='driven wheel within %g rad/s of the no-load speed duty*omega0 - friction/p while another driven wheel is '
             'more than %g rad/s below it' % (UNLOADED_TOL_RAD_S, LOADED_MARGIN_RAD_S),
        fraction_of_driving_time=unloaded,
        any_wheel_fraction=float(np.mean(any_unloaded[driving])) if driving.any() else None,
        driving_samples=int(driving.sum()))
    out['wheels'] = wheels
    susp = {w: a.get('wheel_%s_susp' % w) for w in gz_model.WHEELS}
    if all(s is not None and np.all(np.isfinite(s)) and len(s) for s in susp.values()):
        out['suspension_travel_m'] = {w: dict(min=float(np.min(s)), max=float(np.max(s)), mean=float(np.mean(s)))
                                      for w, s in susp.items()}
    if world == 'w0':
        out['wheel_lift_free'] = bool(all(r['lifted_samples'] == 0 for r in wheels.values()))
    # standstill drift in the hold window (after the vehicle stopped)
    if script.get('hold'):
        t0, t1 = script['hold']
        sel = (t >= t0) & (t <= t1)
        if sel.any():
            drift = gz_probe.standstill_drift(t, a['x'], a['y'], a['yaw'], t0, t1)
            drift['speed_abs_max_m_s'] = float(np.max(np.abs(a['speed'][sel])))
            drift['steer_target_deg'] = float(np.degrees(a['target_eq'][sel][0]))
            drift['joint_left_change_deg'] = float(np.degrees(np.ptp(a['q_left'][sel])))
            drift['pitch_deg_mean'] = float(np.degrees(np.mean(a['pitch'][sel])))
            drift['roll_deg_mean'] = float(np.degrees(np.mean(a['roll'][sel])))
            drift['within_1cm_0p1deg'] = bool(drift['max_position_change_m'] < .01 and drift['max_yaw_change_deg'] < .1)
            out['standstill'] = drift
    out['base_z_range_m'] = [float(np.min(a['z'])), float(np.max(a['z']))]
    out['path_m'] = float(np.sum(np.hypot(np.diff(a['x']), np.diff(a['y']))))
    out['finite'] = bool(all(np.all(np.isfinite(a[k])) for k in ('x', 'y', 'z', 'yaw', 'q_left', 'q_right')))
    return out


# =================================================================================================================
# fixed-steer circles (several rovers, no rendering) through the shared actuator model
# =================================================================================================================
CIRCLE_ANGLES_DEG = (5., 10., 20., -5., -10., -20.)


def run_circles(out_dir, *, profile=None, backend='l0', seed=0, speed=.5, duration_s=125., keep_zero_offset=False,
                angles_deg=CIRCLE_ANGLES_DEG, font=None):
    """Fixed-steer circles of several rovers in one W0 world (fixture, no cameras). Returns the record."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    base_profile = load_profile() if profile is None else profile
    view_profile = demo_profile(base_profile, keep_zero_offset)
    vehicles = []
    for i, angle in enumerate(angles_deg):
        def factory(ctx, angle=angle, i=i):
            duty = ctx.duty_for_speed(speed)
            ramp_drive = DutyRamp(ctx, lambda t: duty if t >= 1. else 0.)
            return gz_probe.ModelSteerController(ctx, make_backend(backend, ctx.view, seed + i),
                                                 lambda t: math.radians(angle) if t >= .5 else 0., ramp_drive)
        vehicles.append(gz_probe.vehicle('c%d' % i, gz_probe._grid(i), 0., factory,
                                         dict(kind='demo_circle', delta_deg=angle, v=speed)))
    group = dict(scene='w0_force', duration_s=float(duration_s), vehicles=vehicles)
    world, ctx, manifests = gz_probe.build_probe_world(group, out_dir, view_profile)
    for spec in vehicles:
        spec['steer_type'] = ctx.steer_type
    log('circles %s deg, %g m/s target, %.0f s SIM, backend %s' % (
        '/'.join('%+g' % a for a in angles_deg), speed, duration_s, backend))
    logs, info = gz_probe.run_fixture(world, group, ctx, manifests,
                                      progress=lambda text: log('circles %s' % text))
    np.savez_compressed(out_dir / 'circles_log.npz', **{'%s__%s' % (name, key): value for name, arrays in logs.items()
                                                        for key, value in arrays.items()})
    rear_x = ctx.geometry.rear_axle_x_in_base_m
    rows = []
    for spec in vehicles:
        a = logs[spec['name']]
        delta = math.radians(spec['tags']['delta_deg'])
        sel = a['t'] >= 8.
        xr, yr = gz_probe.point_track(a['x'][sel], a['y'][sel], a['yaw'][sel], rear_x)
        circle = gz_probe.fit_circle(xr, yr)
        r_geom = abs(ctx.geometry.wheelbase_m / math.tan(delta))
        w = gz_probe.yaw_rate_fit(a['t'][sel], a['yaw'][sel])
        left, right = float(np.median(a['steer_left_q'][sel])), float(np.median(a['steer_right_q'][sel]))
        joint_eq = ctx.geometry.equivalent_from_wheels(left, right)
        r_joint = abs(ctx.geometry.wheelbase_m / math.tan(joint_eq))
        rows.append(dict(name=spec['name'], delta_deg=spec['tags']['delta_deg'], radius_fit_m=circle['radius'],
                         radius_geometry_m=r_geom, radius_error=(circle['radius'] - r_geom) / r_geom,
                         delta_eq_joint_deg=math.degrees(joint_eq), radius_joint_m=r_joint,
                         radius_error_vs_joint=(circle['radius'] - r_joint) / r_joint,
                         yaw_rate_rad_s=w, yaw_sign_ok=bool(np.sign(w) == np.sign(delta)),
                         speed_m_s=gz_probe.path_speed(a['t'][sel], xr, yr), arc_deg=circle['arc_deg'],
                         wheel_left_deg=math.degrees(left), wheel_right_deg=math.degrees(right)))
    record = dict(schema=SCHEMA, label=LABEL_KO, evaluation_use=False, result_kind=RESULT_KIND, field_validated=False,
                  engine='fixture', kind='circles', backend=backend, seed=seed, speed_target_m_s=speed,
                  duration_s=duration_s, profile=view_profile.result_label(),
                  wheel_collision=sorted({m.get('wheel_collision') for m in manifests.values()})[0]
                  if manifests else None,
                  demo_overrides=None if keep_zero_offset else DEMO_OVERRIDES, run=info, circles=rows,
                  wheelbase_m=ctx.geometry.wheelbase_m,
                  provenance=gz_probe.provenance(world, info.get('actuator_backends')))
    _write_json(out_dir / 'circles.json', record)
    try:
        record['plot'] = plot_circles(out_dir, out_dir / 'circles_plot.png', font=font)
        _write_json(out_dir / 'circles.json', record)
    except Exception as exc:                       # the plot is optional
        log('circles plot failed: %s: %s' % (type(exc).__name__, exc))
    for row in rows:
        log('circle %+5.1f deg (joints eq %+.2f deg): R %.3f m (L/tan target %.3f m, %+.2f %%; L/tan joint %.3f m, '
            '%+.2f %%), %.3f m/s, yaw sign %s' % (
                row['delta_deg'], row['delta_eq_joint_deg'], row['radius_fit_m'], row['radius_geometry_m'],
                100. * row['radius_error'], row['radius_joint_m'], 100. * row['radius_error_vs_joint'],
                row['speed_m_s'], 'ok' if row['yaw_sign_ok'] else 'WRONG'))
    return record


# =================================================================================================================
# plots (matplotlib)
# =================================================================================================================
def _plt(font=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    path = font_path(font)
    korean = False
    if path:
        font_manager.fontManager.addfont(path)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=path).get_name()
        korean = True
    plt.rcParams['axes.unicode_minus'] = False
    return plt, korean


def _transform(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + px * c - py * s, y + px * s + py * c) for px, py in points]


def draw_footprint(ax, g, x, y, yaw, left, right, color, alpha=.9, lw=.8, body=True):
    """Body outline (profile footprint) and the four wheels, front wheels at the joint angles."""
    polygons = g.footprint_polygons(left, right, include_hardware_margin=False)
    for name, poly in polygons.items():
        if name == 'body' and not body:
            continue
        pts = _transform(poly, x, y, yaw)
        pts.append(pts[0])
        xs, ys = zip(*pts)
        if name == 'body':
            ax.plot(xs, ys, color=color, lw=lw, alpha=alpha * .8)
        else:
            ax.fill(xs, ys, color=color if name.startswith('front') else '0.25', alpha=alpha, lw=0)


def plot_circles(run_dir, out_png, font=None):
    """Top view of the fixed-steer circles with footprints and the theoretical circles R = L / tan(delta)."""
    run_dir = Path(run_dir)
    record = json.loads((run_dir / 'circles.json').read_text(encoding='utf-8'))
    data = np.load(run_dir / 'circles_log.npz')
    profile = load_profile()
    view = demo_profile(profile, record.get('demo_overrides') is None)
    g = geo.SteeringGeometry.from_profile(view)
    plt, korean = _plt(font)
    fig, ax = plt.subplots(figsize=(10.5, 10.))
    colors = {5.: '#1f77b4', 10.: '#2ca02c', 20.: '#d62728', -5.: '#17becf', -10.: '#8c564b', -20.: '#ff7f0e'}
    rear_x = g.rear_axle_x_in_base_m
    start = None
    theta = np.linspace(0., 2. * math.pi, 361)
    for row in record['circles']:
        name, delta_deg = row['name'], row['delta_deg']
        t, x, y, yaw = (data['%s__%s' % (name, k)] for k in ('t', 'x', 'y', 'yaw'))
        left, right = data['%s__steer_left_q' % name], data['%s__steer_right_q' % name]
        xr, yr = gz_probe.point_track(x, y, yaw, rear_x)
        x0, y0 = xr[0], yr[0]                     # the plot origin is the start of the rear-axle centre
        color = colors.get(delta_deg, 'k')
        radius = g.wheelbase_m / math.tan(math.radians(delta_deg))
        ax.plot(radius * np.sin(theta), radius - radius * np.cos(theta), ls='--', lw=1.2, color=color, alpha=.75)
        joint_deg = row.get('delta_eq_joint_deg', delta_deg)
        if abs(joint_deg - delta_deg) > .05:      # the actuator model did not reach the target: circle of the joints
            joint_r = g.wheelbase_m / math.tan(math.radians(joint_deg))
            ax.plot(joint_r * np.sin(theta), joint_r - joint_r * np.cos(theta), ls=':', lw=1.4, color=color, alpha=.9)
        # one lap of the measured rear-axle path from the start
        heading = np.unwrap(yaw) - yaw[0]
        lap = np.nonzero(np.abs(heading) >= 2. * math.pi)[0]
        end = int(lap[0]) + 1 if lap.size else len(t)
        if korean:
            label = ('δ목표 %+g° (관절 등가 %+.1f°): 측정 R %.2f m\n    L/tan δ목표 %.2f m (%+.1f %%), L/tan δ관절 %.2f m (%+.1f %%)'
                     % (delta_deg, joint_deg, row['radius_fit_m'], row['radius_geometry_m'], 100. * row['radius_error'],
                        row.get('radius_joint_m', row['radius_geometry_m']),
                        100. * row.get('radius_error_vs_joint', row['radius_error'])))
        else:
            label = ('target %+g deg (joints eq %+.1f deg): fitted R %.2f m\n    L/tan target %.2f m (%+.1f %%), '
                     'L/tan joints %.2f m (%+.1f %%)'
                     % (delta_deg, joint_deg, row['radius_fit_m'], row['radius_geometry_m'], 100. * row['radius_error'],
                        row.get('radius_joint_m', row['radius_geometry_m']),
                        100. * row.get('radius_error_vs_joint', row['radius_error'])))
        ax.plot(xr[:end] - x0, yr[:end] - y0, lw=2.2, color=color, label=label)
        for frac in (.25, .5, .75):
            idx = np.nonzero(np.abs(heading[:end]) >= frac * 2. * math.pi)[0]
            if idx.size:
                i = int(idx[0])
                draw_footprint(ax, g, x[i] - x0, y[i] - y0, yaw[i], left[i], right[i], color)
        if start is None:
            start = (x[0] - x0, y[0] - y0, yaw[0])
    if start is not None:
        draw_footprint(ax, g, start[0], start[1], start[2], 0., 0., 'k', alpha=.5)
    ax.plot([0.], [0.], 'ko', ms=4)
    ax.set_aspect('equal')
    ax.grid(True, lw=.4, alpha=.5)
    ax.set_xlabel('x (m, 국소 SIM 좌표: 출발 뒤차축 중심)' if korean else 'x (m, local SIM frame: start rear axle)')
    ax.set_ylabel('y (m, 왼쪽 +)' if korean else 'y (m, left +)')
    speed = record['speed_target_m_s']
    backend = {'l0': 'L0(Python, 이상 서보)', 'native': 'L1(C++, 백래시·데드밴드 자리값)'}.get(record.get('backend'),
                                                                                     record.get('backend'))
    note = collision_note(record.get('wheel_collision'), korean)
    if korean:
        title = ('고정 조향 원 — 관절 직접 조종 시연 (안전 사슬 없음, 평가에 쓰지 않음)\n'
                 '뒤차축 중심 궤적(실선, 첫 한 바퀴), 이론 원 R = L / tan δ(파선: 목표 δ, 점선: 측정 관절 등가각, L = %.2f m), '
                 '발자국은 1/4·1/2·3/4 바퀴\nSIM 자리값 %s%s · W0 평지 · 목표 %.1f m/s · 구동기 모델 %s · 조향 0점 오차 0 · '
                 '자리값 기반 SIM 결과' % (g.wheelbase_m, record['profile']['profile_id'], ' · ' + note if note else '',
                                         speed, backend))
    else:
        title = ('Fixed-steer circles - joint direct-drive demo (no safety chain, not for evaluation)\n'
                 'rear-axle path (solid, first lap), R = L / tan(delta) (dashed: target, dotted: joints, L = %.2f m)\n'
                 'SIM placeholder %s%s, W0 flat, target %.1f m/s, actuator %s, zero offset 0, placeholder-based SIM '
                 'result' % (g.wheelbase_m, record['profile']['profile_id'], ', ' + note if note else '', speed,
                             record.get('backend')))
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=8.5, loc='center left', bbox_to_anchor=(1.02, .5), labelspacing=1.0)
    fig.savefig(out_png, dpi=130, bbox_inches='tight')
    plt.close(fig)
    return str(out_png)


def plot_w1(run_dir, scene_dir, out_png, font=None, footprint_every_s=3.):
    """W1 straight run between the rows: path, footprints, tree trunks (display only) and steering."""
    run_dir, scene_dir = Path(run_dir), Path(scene_dir)
    record = json.loads((run_dir / 'run.json').read_text(encoding='utf-8'))
    a = np.load(run_dir / 'log.npz')
    metadata = json.loads((scene_dir / 'world_metadata.json').read_text(encoding='utf-8'))
    profile = load_profile()
    view = demo_profile(profile, record.get('demo_overrides') is None)
    g = geo.SteeringGeometry.from_profile(view)
    plt, korean = _plt(font)
    fig, (ax, ax2, ax3) = plt.subplots(3, 1, figsize=(15., 7.8), gridspec_kw=dict(height_ratios=[1.05, 1.15, 1.]))
    t, x, y, yaw = a['t'], a['x'], a['y'], a['yaw']
    trees = metadata.get('trees', [])
    x_max = float(np.max(x)) + 2.
    shown = [tr for tr in trees if -6. <= tr['x'] <= x_max + 2. and abs(tr['y']) <= 6.]
    for tree in shown:
        ax.add_patch(plt.Circle((tree['x'], tree['y']), max(tree.get('radius', .05), .05), color='#6b4f2a', alpha=.9))
    ax.plot([], [], 'o', color='#6b4f2a', ms=5, label='줄기(생성된 월드, 표시만)' if korean else 'trunks (generated world, display only)')
    ax.plot(x, y, color='#1f4e9c', lw=1.8, label='차량 원점(base_footprint) 궤적' if korean else 'rover origin path')
    n = len(t)
    i = 0
    while i < n:
        draw_footprint(ax, g, x[i], y[i], yaw[i], a['q_left'][i], a['q_right'][i], '#d62728', alpha=.8)
        nxt = np.nonzero(t >= t[i] + footprint_every_s)[0]
        if not nxt.size:
            break
        i = int(nxt[0])
    ax.set_aspect('equal')
    ax.set_xlim(min(-5., float(np.min(x)) - 1.), x_max)
    ax.set_ylim(-3., 3.)
    ax.grid(True, lw=.4, alpha=.5)
    ax.set_ylabel('y (m)')
    ax.legend(fontsize=8.5, loc='upper left', ncol=2)
    ax2.plot(x, np.degrees(a['target_eq']), color='k', lw=1.2, label='조향 목표(등가)' if korean else 'target (eq)')
    ax2.plot(x, np.degrees(a['model_eq']), color='0.5', lw=1., ls='--', label='공유 구동기 모델(등가)' if korean else 'shared model (eq)')
    ax2.plot(x, np.degrees(a['q_left']), color='#d62728', lw=.9, label='왼쪽 바퀴 관절' if korean else 'left joint')
    ax2.plot(x, np.degrees(a['q_right']), color='#1f77b4', lw=.9, label='오른쪽 바퀴 관절' if korean else 'right joint')
    ax2.set_ylabel('조향각 (°)' if korean else 'steer (deg)')
    ax2.grid(True, lw=.4, alpha=.5)
    ax2.legend(fontsize=8, loc='upper right', ncol=4)
    ax3.plot(x, a['speed'], color='#1f4e9c', lw=1.)
    ax3b = ax3.twinx()
    ax3b.plot(x, a['duty'], color='#ff7f0e', lw=1.)
    ax3.set_ylabel('속도 (m/s)' if korean else 'speed (m/s)', color='#1f4e9c')
    ax3b.set_ylabel('duty', color='#ff7f0e')
    ax3.set_xlabel('x (m, 국소 SIM 좌표)' if korean else 'x (m, local SIM frame)')
    ax3.grid(True, lw=.4, alpha=.5)
    for axis in (ax2, ax3):
        axis.set_xlim(ax.get_xlim())
    params = record['script']['params']
    note = collision_note(record.get('wheel_collision'), korean)
    if korean:
        title = ('W1 원본 과수원: 열 사이 직진과 작은 보정 — 관절 직접 조종 시연 (안전 사슬 없음, 평가에 쓰지 않음)\n'
                 '대본: 열린 고리 시간표(±%g° 두 번, duty %.2f), 발자국 %g s마다 · SIM 자리값 %s%s · 조향 0점 오차 0 · '
                 '자리값 기반 SIM 결과' % (params['steer_deg'], params['duty'], footprint_every_s,
                                         record['profile']['profile_id'], ' · ' + note if note else ''))
    else:
        title = ('W1 original orchard: straight run between rows with gentle corrections - joint direct-drive demo\n'
                 'open-loop schedule (+-%g deg twice, duty %.2f), footprints every %g s, SIM placeholder %s%s'
                 % (params['steer_deg'], params['duty'], footprint_every_s, record['profile']['profile_id'],
                    ', ' + note if note else ''))
    ax.set_title(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    return str(out_png)


def plot_run(run_dir, out_png, font=None, footprint_every_s=2.):
    """Top view of a W0 demo run with footprints."""
    run_dir = Path(run_dir)
    record = json.loads((run_dir / 'run.json').read_text(encoding='utf-8'))
    a = np.load(run_dir / 'log.npz')
    view = demo_profile(load_profile(), record.get('demo_overrides') is None)
    g = geo.SteeringGeometry.from_profile(view)
    plt, korean = _plt(font)
    fig, ax = plt.subplots(figsize=(9., 8.))
    t, x, y, yaw = a['t'], a['x'], a['y'], a['yaw']
    ax.plot(x, y, color='#1f4e9c', lw=1.6)
    i = 0
    while i < len(t):
        draw_footprint(ax, g, x[i], y[i], yaw[i], a['q_left'][i], a['q_right'][i], '#d62728', alpha=.8)
        nxt = np.nonzero(t >= t[i] + footprint_every_s)[0]
        if not nxt.size:
            break
        i = int(nxt[0])
    ax.set_aspect('equal')
    ax.grid(True, lw=.4, alpha=.5)
    ax.set_xlabel('x (m, 국소 SIM 좌표)' if korean else 'x (m, local SIM frame)')
    ax.set_ylabel('y (m, 국소 SIM 좌표)' if korean else 'y (m, local SIM frame)')
    note = collision_note(record.get('wheel_collision'), korean)
    ax.set_title(('%s (안전 사슬 없음, 평가에 쓰지 않음)\n%s · 차 원점 궤적과 %g s마다 발자국(앞바퀴는 측정 관절각) · '
                  '구동기 모델 %s\n%s %s%s · 자리값 기반 SIM 결과'
                  % (LABEL_KO, record['script']['description'], footprint_every_s, record['backend']['kind'],
                     PLACEHOLDER_KO, record['profile']['profile_id'], ' · ' + note if note else ''))
                 if korean else '%s - %s%s' % (LABEL_EN, record['script']['name'], ', ' + note if note else ''),
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    return str(out_png)


# =================================================================================================================
# server engine: real-time factor report and keyboard mode
# =================================================================================================================
def run_rtf(scene_dir, out_dir, *, duration_s=180., steer_deg=0., duty=.8, warmup_s=20., label='rtf', profile=None):
    """Real-time pacing run of a demo or base scene (gz_probe.run_server) with the demo camera rates recorded."""
    manifest = json.loads((Path(scene_dir) / 'scene_manifest.json').read_text(encoding='utf-8'))
    demo = manifest.get('demo') or {}
    extra = [CAMERA_TOPIC % c['view'] for c in demo.get('cameras', [])]
    log('real-time run %s: %.0f s wall, steer %+g deg, duty %.2f, demo cameras %s' % (
        Path(scene_dir).name, duration_s, steer_deg, duty, ','.join(extra) or '-'))
    result = gz_probe.run_server(scene_dir, out_dir, duration_s=duration_s, steer_deg=steer_deg, duty=duty,
                                 warmup_s=warmup_s, label=label, profile=profile, extra_camera_topics=extra)
    result['label'] = LABEL_KO
    result['evaluation_use'] = False
    _write_json(Path(out_dir) / ('%s_result.json' % label), result)
    log('RTF median %s (p10 %s, min %s), rover camera %s, demo cameras %s, truth path %.1f m' % (
        result['rtf_median'], result['rtf_p10'], result['rtf_min'], result['camera'], result.get('extra_cameras'),
        result['truth_path_m']))
    return result


KEY_HELP = 'a/d: steer target +/- step, w/s: duty +/- step, space or x: duty 0, c: steer target 0, q: quit'


def apply_key(key, target_deg, duty, *, step_deg, step_duty, limit_deg):
    """New (target_deg, duty, quit) for one key (pure)."""
    quit_ = False
    if key == 'a':
        target_deg = min(limit_deg, target_deg + step_deg)
    elif key == 'd':
        target_deg = max(-limit_deg, target_deg - step_deg)
    elif key == 'w':
        duty = min(1., duty + step_duty)
    elif key == 's':
        duty = max(0., duty - step_duty)
    elif key in (' ', 'x'):
        duty = 0.
    elif key == 'c':
        target_deg = 0.
    elif key == 'q':
        duty, quit_ = 0., True
    return target_deg, duty, quit_


def run_keyboard(scene_dir, *, profile=None, backend='l0', seed=0, rate_hz=100., max_s=600., step_deg=2.,
                 step_duty=.05, idle_exit_s=5., keep_zero_offset=False, out_dir=None):
    """Keyboard joint direct-drive at real-time pacing (`gz sim -s -r`; attach a GUI client to watch).

    Keys come from stdin: raw single keys on a terminal, or any character stream from a pipe. The shared
    actuator model is stepped on the wall clock at rate_hz with the truth speed of the plant (plant side).
    """
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.transport13 import Node

    scene_dir = Path(scene_dir)
    manifest = json.loads((scene_dir / 'scene_manifest.json').read_text(encoding='utf-8'))
    base_profile = load_profile() if profile is None else profile
    view_profile = demo_profile(gz_assemble.scene_profile(manifest['scene'], base_profile), keep_zero_offset)
    dt = 1. / float(rate_hz)
    ctx = gz_probe.Context(view_profile, dt)
    limit_deg = math.degrees(min(ctx.geometry.soft_limits_eq()))
    backend_obj = make_backend(backend, view_profile, seed)
    backend_obj.reset(0.)
    topics = manifest['model']['topics']
    node = Node()
    pub_steer = {key: node.advertise(topic, Double) for key, topic in topics['steer'].items()}
    pub_drive = {wheel: node.advertise(topic, Double) for wheel, topic in topics['drive'].items()}
    speed = dict(v=0.)

    def on_odom(msg):                 # plant side (truth) speed for the actuator model's standstill rule
        speed['v'] = msg.twist.linear.x
    node.subscribe(Odometry, topics['truth_odometry_gz'], on_odom)
    keys = []
    keys_lock = threading.Lock()
    eof = threading.Event()

    def reader():
        stream = sys.stdin
        try:
            if stream.isatty():
                import termios
                import tty
                fd = stream.fileno()
                saved = termios.tcgetattr(fd)
                try:
                    tty.setcbreak(fd)
                    while True:
                        ch = stream.read(1)
                        if not ch:
                            break
                        with keys_lock:
                            keys.append(ch)
                        if ch == 'q':
                            break
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, saved)
            else:
                while True:
                    ch = stream.read(1)
                    if not ch:
                        break
                    with keys_lock:
                        keys.append(ch)
        finally:
            eof.set()
    command = gz_probe._gz_sim_command(scene_dir / manifest['sdf'], manifest)
    log_dir = Path(out_dir) if out_dir else scene_dir
    log_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    with (log_dir / 'keyboard_gz.log').open('w', encoding='utf-8') as gz_log:
        process = subprocess.Popen(command, stdout=gz_log, stderr=subprocess.STDOUT, start_new_session=True)
        log('keyboard mode (%s). %s' % (KEY_HELP, 'no safety chain; Ctrl+C stops'))
        threading.Thread(target=reader, daemon=True).start()
        target_deg, duty, applied = 0., 0., 0.
        started = time.monotonic()
        next_tick = started
        last_print = 0.
        eof_at = None
        quit_ = False
        try:
            while time.monotonic() - started < max_s and not quit_:
                if process.poll() is not None:
                    raise DemoError('gz sim exited %r' % process.returncode)
                with keys_lock:
                    pending, keys[:] = list(keys), []
                for key in pending:
                    target_deg, duty, quit_ = apply_key(key.lower(), target_deg, duty, step_deg=step_deg,
                                                        step_duty=step_duty, limit_deg=limit_deg)
                    if quit_:
                        break
                if eof.is_set() and eof_at is None:
                    eof_at = time.monotonic()
                if eof_at is not None and time.monotonic() - eof_at > idle_exit_s:
                    break
                ramp = ctx.ramp_frac_per_s * dt
                applied += max(-ramp, min(ramp, duty - applied))
                out = backend_obj.step(ba.SteerStepInput(dt_s=dt, mode='test_servo',
                                                         target_delta_eq_rad=math.radians(target_deg),
                                                         vehicle_speed_m_s=speed['v'],
                                                         vehicle_stopped=abs(speed['v']) < STANDSTILL_SPEED_M_S))
                steer = {'axle': out.delta_eq_rad} if ctx.steer_type == geo.AXLE_PIVOT else \
                    {'left': out.delta_left_rad, 'right': out.delta_right_rad}
                for key, value in steer.items():
                    msg = Double()
                    msg.data = float(value)
                    pub_steer[key].publish(msg)
                for wheel, value in ctx.drive_targets(applied).items():
                    msg = Double()
                    msg.data = float(value)
                    pub_drive[wheel].publish(msg)
                now = time.monotonic()
                rows.append((now - started, target_deg, math.degrees(out.delta_eq_rad), duty, applied, speed['v']))
                if now - last_print >= .5:
                    last_print = now
                    log('t %.1f s · 조향 목표 %+.1f° · 모델 등가각 %+.1f° · duty %.2f(적용 %.2f) · %.2f m/s'
                        % (now - started, target_deg, math.degrees(out.delta_eq_rad), duty, applied, speed['v']))
                next_tick += dt
                time.sleep(max(0., next_tick - time.monotonic()))
        finally:
            for wheel, pub in pub_drive.items():
                msg = Double()
                msg.data = 0.
                pub.publish(msg)
            time.sleep(.2)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            backend_obj.close()
    record = dict(schema=SCHEMA, label=LABEL_KO, evaluation_use=False, engine='server', kind='keyboard',
                  command=command, samples=len(rows), result_kind=RESULT_KIND, field_validated=False,
                  final=dict(zip(('t_s', 'target_deg', 'model_eq_deg', 'duty', 'applied_duty', 'speed_m_s'),
                                 rows[-1])) if rows else None, profile=view_profile.result_label())
    np.savez_compressed(log_dir / 'keyboard_log.npz', rows=np.asarray(rows, dtype=float))
    _write_json(log_dir / 'keyboard.json', record)
    log('keyboard mode ended: %d samples' % len(rows))
    return record


# =================================================================================================================
# static model check
# =================================================================================================================
def check_scene(scene_dir):
    """Static checks of a demo scene: the rover model is unchanged and the wheel-body clearance over the steering range."""
    scene_dir = Path(scene_dir)
    manifest = read_demo_manifest(scene_dir)
    root = E.parse(str(scene_dir / manifest['sdf'])).getroot()
    rover = root.find("world/model[@name='%s']" % gz_model.MODEL_NAME)
    profile = gz_assemble.scene_profile(manifest['scene'], load_profile())
    clearance = gz_model.wheel_body_clearance(rover, profile)
    result = dict(schema=SCHEMA, label=LABEL_KO, rover_model_sha256=model_sha256(rover),
                  rover_model_unchanged=model_sha256(rover) == manifest['demo']['rover_model_sha256'],
                  wheel_body_clearance=clearance)
    _write_json(scene_dir / 'check.json', result)
    log('check: rover model unchanged %s, wheel-body clearance min %.4f m (%s)' % (
        result['rover_model_unchanged'], clearance['min_clearance_m'],
        'no overlap' if clearance['overlap_free'] else 'OVERLAP'))
    return result


# =================================================================================================================
# CLI
# =================================================================================================================
def _write_json(path, payload):
    gz_probe.write_json(path, payload)


def _size(text):
    w, h = (int(v) for v in text.lower().split('x'))
    if not (16 <= w <= 3840 and 16 <= h <= 2160):
        raise argparse.ArgumentTypeError('size out of range')
    return (w, h)


def main(argv=None):
    parser = argparse.ArgumentParser(description='%s (%s): steered rover joint direct-drive demo in Gazebo; '
                                                 'no safety chain, never used for evaluation.' % (LABEL_KO, LABEL_EN))
    sub = parser.add_subparsers(dest='command', required=True)
    p_scene = sub.add_parser('scene', help='build a demo scene (base steering scene + demo cameras)')
    p_scene.add_argument('--world', choices=sorted(WORLD_SCENES), required=True)
    p_scene.add_argument('--out', type=Path, required=True)
    p_scene.add_argument('--profile', type=Path)
    p_scene.add_argument('--views', default='chase,top', help='comma list of %s (empty: none)' % ','.join(VIEWS))
    p_scene.add_argument('--size', type=_size, default=DEFAULT_SIZE)
    p_scene.add_argument('--fps', type=float, default=DEFAULT_FPS)
    p_scene.add_argument('--no-rover-camera', action='store_true')
    p_scene.add_argument('--no-wheel-wells', action='store_true',
                         help='keep the single generated body box (before/after comparison only)')
    p_scene.add_argument('--suspension', choices=('default', 'on', 'off'), default='default',
                         help='wheel suspension option of gz_model (default: the model default)')
    p_scene.add_argument('--front-pivot', choices=('default', 'on', 'off'), default='default',
                         help='front axle roll pivot option of gz_model (default: the model default)')
    p_scene.add_argument('--wheel-collision', choices=('default',) + gz_model.WHEEL_COLLISION_SHAPES, default='default',
                         help='wheel collision shape option of gz_model (default: the model default)')
    p_run = sub.add_parser('run', help='run a maneuver script in the lockstep fixture with the demo cameras')
    p_run.add_argument('--scene-dir', type=Path, required=True)
    p_run.add_argument('--script', choices=SCRIPTS, required=True)
    p_run.add_argument('--out', type=Path, required=True)
    p_run.add_argument('--profile', type=Path)
    p_run.add_argument('--backend', choices=BACKENDS, default='l0')
    p_run.add_argument('--seed', type=int, default=0)
    p_run.add_argument('--steer-deg', type=float)
    p_run.add_argument('--speed', type=float)
    p_run.add_argument('--duty', type=float)
    p_run.add_argument('--still-every', type=float)
    p_run.add_argument('--no-video', action='store_true')
    p_run.add_argument('--keep-zero-offset', action='store_true')
    p_run.add_argument('--font', help='TTF font for Korean overlays (default: $%s)' % FONT_ENV)
    p_circ = sub.add_parser('circles', help='fixed-steer circles of several rovers in W0 (fixture, no cameras)')
    p_circ.add_argument('--out', type=Path, required=True)
    p_circ.add_argument('--profile', type=Path)
    p_circ.add_argument('--backend', choices=BACKENDS, default='l0')
    p_circ.add_argument('--seed', type=int, default=0)
    p_circ.add_argument('--speed', type=float, default=.5)
    p_circ.add_argument('--duration', type=float, default=125.)
    p_circ.add_argument('--keep-zero-offset', action='store_true')
    p_circ.add_argument('--font')
    p_plot = sub.add_parser('plot', help='plots from saved logs')
    p_plot.add_argument('kind', choices=('circles', 'w1', 'run'))
    p_plot.add_argument('--run', type=Path, required=True)
    p_plot.add_argument('--scene-dir', type=Path)
    p_plot.add_argument('--out', type=Path, required=True)
    p_plot.add_argument('--font')
    p_rtf = sub.add_parser('rtf', help='real-time factor of a scene at real-time pacing (report)')
    p_rtf.add_argument('--scene-dir', type=Path, required=True)
    p_rtf.add_argument('--out', type=Path, required=True)
    p_rtf.add_argument('--profile', type=Path)
    p_rtf.add_argument('--duration', type=float, default=180.)
    p_rtf.add_argument('--steer-deg', type=float, default=0.)
    p_rtf.add_argument('--duty', type=float, default=.8)
    p_rtf.add_argument('--label', default='rtf')
    p_key = sub.add_parser('keyboard', help='keyboard joint direct-drive at real-time pacing')
    p_key.add_argument('--scene-dir', type=Path, required=True)
    p_key.add_argument('--profile', type=Path)
    p_key.add_argument('--backend', choices=BACKENDS, default='l0')
    p_key.add_argument('--max-s', type=float, default=600.)
    p_key.add_argument('--out', type=Path)
    p_key.add_argument('--keep-zero-offset', action='store_true')
    p_check = sub.add_parser('check', help='static checks of a demo scene (rover model unchanged, wheel-body clearance)')
    p_check.add_argument('--scene-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.command == 'scene':
        views = tuple(v for v in args.views.split(',') if v)
        model_options = {}
        if args.no_wheel_wells:
            model_options['body_wheel_wells'] = False
        if args.suspension != 'default':
            model_options['suspension'] = args.suspension == 'on'
        if args.front_pivot != 'default':
            model_options['front_axle_pivot'] = args.front_pivot == 'on'
        if args.wheel_collision != 'default':
            model_options['wheel_collision'] = args.wheel_collision
        manifest = build_demo_scene(args.world, args.out, profile=load_profile(args.profile), views=views,
                                    size=args.size, fps=args.fps, rover_camera=not args.no_rover_camera,
                                    model_options=model_options or None)
        print(json.dumps(dict(scene=str(args.out), sdf_sha256=manifest['sdf_sha256'],
                              rover_model_sha256=manifest['demo']['rover_model_sha256'])))
        return 0
    if args.command == 'run':
        options = {k: getattr(args, k) for k in ('steer_deg', 'speed', 'duty') if getattr(args, k) is not None}
        record = run_demo(args.scene_dir, args.script, args.out, profile=load_profile(args.profile),
                          backend=args.backend, seed=args.seed, video=not args.no_video, font=args.font,
                          keep_zero_offset=args.keep_zero_offset, still_every=args.still_every,
                          script_options=options)
        code = 0 if not record['run']['errors'] else 1
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)                              # skip the rendering server teardown (see _KEEP_ALIVE)
    if args.command == 'circles':
        record = run_circles(args.out, profile=load_profile(args.profile), backend=args.backend, seed=args.seed,
                             speed=args.speed, duration_s=args.duration, keep_zero_offset=args.keep_zero_offset,
                             font=args.font)
        return 0 if not record['run']['errors'] else 1
    if args.command == 'plot':
        if args.kind == 'circles':
            print(plot_circles(args.run, args.out, font=args.font))
        elif args.kind == 'w1':
            if args.scene_dir is None:
                parser.error('plot w1 needs --scene-dir')
            print(plot_w1(args.run, args.scene_dir, args.out, font=args.font))
        else:
            print(plot_run(args.run, args.out, font=args.font))
        return 0
    if args.command == 'rtf':
        result = run_rtf(args.scene_dir, args.out, duration_s=args.duration, steer_deg=args.steer_deg,
                         duty=args.duty, label=args.label, profile=load_profile(args.profile))
        return 0 if result['error'] is None else 1
    if args.command == 'keyboard':
        run_keyboard(args.scene_dir, profile=load_profile(args.profile), backend=args.backend, max_s=args.max_s,
                     keep_zero_offset=args.keep_zero_offset, out_dir=args.out)
        return 0
    result = check_scene(args.scene_dir)
    return 0 if result['rover_model_unchanged'] else 1


if __name__ == '__main__':
    sys.exit(main())
