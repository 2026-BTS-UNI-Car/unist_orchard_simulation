"""Pictures of the steering rover model and the orchard world in Gazebo (DOC-13 part; README / notes material).

Fixed demo cameras around the parked rover, rendered in the lockstep fixture of the joint direct-drive demo
(gz_direct_drive: separate static camera models, no visual / collision / mass; the rover model is unchanged and its
SHA-256 is checked). No safety chain, not for evaluation. Every value is a placeholder (자리값 기반 SIM 결과).

  python3 -B -m steering.gz_views w0 --out /runs/views_w0     # model: front, side, top, oblique (+ steered left 15 deg)
  python3 -B -m steering.gz_views w1 --out /runs/views_w1     # orchard: high overview, oblique overview, lane entry,
                                                              # rover camera, model in the lane
  --profile <YAML>  the vehicle profile (default: the shipped default v0.1; the README model pictures of 2026-11-16 use
                    steering/profiles/front_steer_v1_spec_20261116.yaml, the measured dimensions)
Writes <out>/scene/ (the demo scene) and <out>/run/ (run.json, stills/<tag>_<view>.png with the overlay lines and
stills/<tag>_<view>_raw.png without). The container needs the GPU render settings of GZ_DEMO_KO.md 3 (D3D12 / Ogre 1).
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

from . import gz_direct_drive as dd
from .vehicle_profile import load_profile

# Extra demo camera views (rover frame: x forward, y left, z up; metres). 'follow' cameras stay at their first pose
# because the rover stands still (duty 0) in these scripts.
EXTRA_VIEWS = {
    'model_front': dict(kind='follow', offset=(2.7, 0., .75), look=(0., 0., .3), hfov_deg=42.),
    'model_side': dict(kind='follow', offset=(.15, 3.1, .8), look=(.15, 0., .3), hfov_deg=40.),
    'model_oblique': dict(kind='follow', offset=(2.3, 2.0, 1.55), look=(.1, 0., .25), hfov_deg=46.),
    'model_top': dict(kind='top', point=(.1, 0.), height=10., image_up='left', hfov_deg=9.5),
    'orchard_high': dict(kind='top', point=(15., 0.), height=36., image_up='left', hfov_deg=66.),
    'orchard_oblique': dict(kind='follow', offset=(-7.5, 7.5, 7.5), look=(9., 0., 0.), hfov_deg=72.),
    'orchard_lane': dict(kind='follow', offset=(-2.4, 0., 2.0), look=(6., 0., .7), hfov_deg=72.),
}
EXTRA_KO = {'model_front': '앞에서 본 차', 'model_side': '옆에서 본 차', 'model_oblique': '비스듬히 본 차',
            'model_top': '위에서 본 차', 'orchard_high': '과수원 전경(위)', 'orchard_oblique': '과수원 전경(비스듬히)',
            'orchard_lane': '열 사이 출발점(뒤에서)'}
VIEW_SETS = {
    'w0': ('model_front', 'model_side', 'model_oblique', 'model_top'),
    'w1': ('orchard_high', 'orchard_oblique', 'orchard_lane'),
}
SCRIPT_NAMES = {'w0': 'views_w0', 'w1': 'views_w1'}
STEER_DEG = 15.


def install_views():
    """Add the extra views to the demo tool's tables (idempotent)."""
    for name, spec in EXTRA_VIEWS.items():
        dd.VIEWS.setdefault(name, dict(spec))
        dd.VIEW_KO.setdefault(name, EXTRA_KO[name])
        dd.VIEW_EN.setdefault(name, name.replace('_', ' '))


def views_script(name):
    """The parked-rover script of a view set: steering 0 until 3.5 s (stills 'straight' at 3.0 s), then steering
    target +STEER_DEG (left) and stills 'left15' at 6.5 s; duty 0 all the time (the rover stands still)."""
    world = 'w0' if name == 'views_w0' else 'w1'
    views = VIEW_SETS[world] + ((dd.ROVER_VIEW,) if world == 'w1' else ())
    steered = ('model_front', 'model_oblique', 'model_top') if world == 'w0' else ('orchard_lane',)
    return dict(world=world, duration_s=7.,
                steer=lambda t: STEER_DEG if t >= 3.5 else 0.,
                duty=lambda t: 0.,
                stills=[(3.0, 'straight', views), (6.5, 'left%g' % STEER_DEG, steered)],
                video=None, hold=(1., 3.), name=name,
                description='정지한 차(duty 0), 조향 0° 뒤 +%g°(왼쪽): 모델·과수원 그림' % STEER_DEG,
                params=dict(steer_deg=STEER_DEG, duty=0.))


def install_scripts():
    original = dd.make_script
    if getattr(original, '_gz_views', False):
        return

    def make_script(name, ctx, **kwargs):
        if name in SCRIPT_NAMES.values():
            return views_script(name)
        return original(name, ctx, **kwargs)
    make_script._gz_views = True
    dd.make_script = make_script


def main(argv=None):
    p = argparse.ArgumentParser(prog='gz_views', description=__doc__.splitlines()[0])
    p.add_argument('world', choices=sorted(VIEW_SETS))
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--font', default=None)
    p.add_argument('--profile', type=Path, default=None,
                   help='vehicle profile YAML (default: the shipped default profile)')
    args = p.parse_args(argv)
    install_views()
    install_scripts()
    profile = load_profile(args.profile)
    scene = args.out / 'scene'
    manifest = dd.build_demo_scene(args.world, scene, profile=profile, views=VIEW_SETS[args.world],
                                   rover_camera=args.world == 'w1', grid=args.world == 'w0')
    record = dd.run_demo(scene, SCRIPT_NAMES[args.world], args.out / 'run', profile=profile, backend='l0',
                         video=False, font=args.font)
    stills = record['frames']['stills']
    summary = dict(world=args.world, profile_id=profile.profile_id, profile_sha256=profile.sha256,
                   scene_sdf_sha256=manifest['sdf_sha256'],
                   rover_model_sha256=manifest['demo']['rover_model_sha256'], stills=stills,
                   errors=record['run']['errors'], result_kind=dd.RESULT_KIND)
    (args.out / 'VIEWS.json').write_bytes((json.dumps(summary, indent=1, ensure_ascii=False, default=str) + '\n')
                                          .encode('utf-8'))
    print(json.dumps(dict(stills=len(stills), errors=len(record['run']['errors']))), flush=True)
    sys.stdout.flush()
    os._exit(0 if not record['run']['errors'] else 1)      # skip the rendering server teardown (gz_direct_drive)


if __name__ == '__main__':
    main()
