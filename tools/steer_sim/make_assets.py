#!/usr/bin/env python3
"""Make the documentation assets of docs/assets/ from their sources and keep the manifest docs/assets/ASSETS.json
(rules: docs/DOCS_POLICY_KO.md; check: tools/steer_sim/check_assets.py).

  python -B tools/steer_sim/make_assets.py --list
  python -B tools/steer_sim/make_assets.py --runs-root <runs folder> [--only NAME ...]
  python -B tools/steer_sim/make_assets.py --mark-checked NAME [NAME ...] --note "<what was looked at>"

Sources are capture files outside the repository: the 'runs' folder of the steering SIM work (Gazebo stills of
steering.gz_views, camera frames of tools/steer_sim/capture_media.py, plots of a /scan message), given with
--runs-root or the environment variable UBTS_STEER_RUNS. The manifest names them relative to that folder
('<runs>/...'), never with a local absolute path. Needs Pillow; the animation also needs ffmpeg on PATH and, for its
caption band, a Korean font (Malgun Gothic on Windows, Noto Sans CJK on Linux; --font picks another).

Each recipe re-encodes its source without metadata (no text, EXIF or comment blocks), so the same source and the
same tool versions (recorded in the manifest) give the same bytes. A regenerated file whose bytes changed loses its
visual check: look at it and run --mark-checked again (check_assets.py refuses an asset whose visual check names
other bytes). Everything shown is a placeholder-based SIM result (자리값 기반 SIM 결과), never field performance.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST_REL = 'docs/assets/ASSETS.json'
SCHEMA = 'ubts.sim_repo.assets.v1'
SIM_TAG = '[SIM · 자리값]'
GZ_VIEWS = ('steering.gz_views (Gazebo 정지 장면, 이미지 ubts-steer-sim:wf4c, 프로필 front_steer_v0_1_placeholder, '
            '2026-11-13, 그림 위 띠는 gz_views가 넣은 조건 표시)')
W0 = 'gsim_final_20261113/media/gz_views/views_w0/run/stills/'
W1 = 'gsim_final_20261113/media/gz_views/views_w1/run/stills/'
FINAL_N3 = 'gsim_final_20261113/sessions/s2a_nominal_3/media/cam_follow/frames'
FINAL_UI = 'gsim_final_20261113/sessions/s2b_ui2/browser/'
FINAL3_N3 = 'gsim_final_20261113/sessions/s3a_nominal_3/media/cam_follow/frames'
FINAL3_UI = 'gsim_final_20261113/sessions/s3b_ui2/browser/'
FINAL4_F3 = 'gsim_final_20261113/sessions/s4f_simauto/media/cam_run3/frames'
MEDIA = 'gsim_media_20261114/frames/'
STAGE8_CAM = 'gsim_stage8_20261114/sessions/cam_run_1/media/cam/frames'
TOP_VIEW = ('tools/steer_sim/gsim_media.py frames %s (위에서 본 그림, matplotlib; 프레임마다 조건 띠와 상태 줄을 그린다) '
            '<- %s. 참값 경로·조향각은 평가용 그림에만 쓴다')

# The vehicle camera (640x360) sees the top of the rover body in its bottom 58 rows (rows 302..359, the same in
# every G-SIM camera session of 11/12 ~ 11/14). In a README or Notion clip that blue band reads as a cut-off
# frame, so the camera recipes cut it before resizing: crop_bottom is a fraction of the source height.
CAMERA_BODY_CROP = 58 / 360

RECIPES = [
    dict(name='rover_oblique', out='docs/assets/rover/rover_oblique.png', op='png', source=W0 + 'straight_model_oblique.png',
         made_by=GZ_VIEWS,
         caption='Gazebo 조향 차량 모델을 비스듬히 본 모습. 차체 상자, 앞바퀴 조향 링크, LiDAR(기둥 위 원통), 카메라(앞 검은 상자). '
                 '치수·질량은 프로필 v0.1의 자리값'),
    dict(name='rover_side', out='docs/assets/rover/rover_side.png', op='png', source=W0 + 'straight_model_side.png',
         made_by=GZ_VIEWS, caption='옆에서 본 모델. 휠베이스 0.76 m, 바퀴 반경 0.14 m(자리값)'),
    dict(name='rover_top', out='docs/assets/rover/rover_top.png', op='png', source=W0 + 'straight_model_top.png',
         made_by=GZ_VIEWS, caption='위에서 본 모델(오른쪽이 앞). LiDAR와 카메라가 차체 앞쪽 중앙에 있다'),
    dict(name='rover_front_straight', out='docs/assets/rover/rover_front_straight.png', op='png',
         source=W0 + 'straight_model_front.png', made_by=GZ_VIEWS, caption='앞에서 본 모델, 조향 0°'),
    dict(name='rover_front_left15', out='docs/assets/rover/rover_front_left15.png', op='png',
         source=W0 + 'left15_model_front.png', made_by=GZ_VIEWS,
         caption='앞에서 본 모델, 조향 목표(등가각) +15°(왼쪽). 바퀴각 좌 +15.6° / 우 +14.5°(Ackermann 비율 0.5, 자리값)'),
    dict(name='orchard_oblique', out='docs/assets/world/orchard_oblique.jpg', op='jpeg', quality=85,
         source=W1 + 'straight_orchard_oblique.png', made_by=GZ_VIEWS,
         caption='W1 원본 과수원 월드(원본 프로젝트의 생성기)를 비스듬히 본 전경. 오른쪽 아래 파란 차가 출발점에 있다'),
    dict(name='orchard_top', out='docs/assets/world/orchard_top.jpg', op='jpeg', quality=85,
         source=W1 + 'straight_orchard_high.png', made_by=GZ_VIEWS,
         caption='W1 원본 과수원을 위에서 본 전경. 나무 열 6개 가운데 아래쪽 4개가 보인다(열 간격 3.8 m, 나무 간격 1.2 m, '
                 '열 길이 30 m, 생성기 설정값). 왼쪽 길의 파란 점이 차'),
    dict(name='orchard_lane_start', out='docs/assets/world/orchard_lane_start.jpg', op='jpeg', quality=85,
         source=W1 + 'straight_orchard_lane.png', made_by=GZ_VIEWS,
         caption='열 사이 통로 출발점을 차 뒤에서 본 모습. 두 나무 열, 콘크리트 기둥과 철선'),
    dict(name='camera_row_follow', crop_bottom=CAMERA_BODY_CROP, out='docs/assets/sensors/camera_row_follow.jpg', op='jpeg', quality=88,
         source='gsim_final_20261113/media/rover_camera_row_follow.png',
         made_by='tools/steer_sim/capture_media.py camera → still (G-SIM 스택, 장면 w1_force, 이미지 wf4c, 세션 media_w1b, '
                 '2026-11-13)',
         caption='통로 안에서 받은 차량 카메라 한 장(640×360 Gazebo 렌더에서 아래 차체 띠 58줄을 잘라 640×302). 카메라는 표시·기록용이고 위치 추정에 쓰지 않는다'),
    dict(name='lidar_scan_in_row', out='docs/assets/sensors/lidar_scan_in_row.png', op='png',
         source='gsim_final_20261113/media/lidar_scan_w1_in_row.png',
         made_by='G-SIM 스택(w1_force, 이미지 wf4c, 세션 media_w1b)의 /scan 한 장을 차 기준 좌표로 그린 것(matplotlib)',
         caption='통로 안에서 받은 LiDAR /scan 한 장(S2 형식 3240빔, 차 기준 좌표). 빨간 사각형이 차 외곽 1.22 × 0.70 m'),
    dict(name='lidar_scan_start', out='docs/assets/sensors/lidar_scan_start.png', op='png',
         source='gsim_final_20261113/media/lidar_scan_w1_start.png',
         made_by='G-SIM 스택(w1_force, 이미지 wf4c, 세션 media_w1)의 /scan 한 장을 차 기준 좌표로 그린 것(matplotlib)',
         caption='출발점(통로 앞)에서 받은 LiDAR /scan 한 장. 앞쪽 3 m부터 두 줄의 줄기가 보인다'),
    dict(name='row_follow_camera_10x', crop_bottom=CAMERA_BODY_CROP, out='docs/assets/video/row_follow_camera_10x.gif', op='gif',
         source='steer_gsim_follow_20261112/sessions/it1_nominal_3/media/cam_follow/frames',
         frames=(290, 3251, 30), frame_s=0.1, width=480, colors=64,
         label='G-SIM 열 추종 · W1 과수원 · 자리값 프로필 · 10배속',
         made_by='G-SIM 열 추종 확인 반복 1의 run nominal_3(row_end로 끝남, STEER_FOLLOW_GSIM_RESULT_KO.md)을 '
                 'tools/steer_sim/capture_media.py camera로 받은 프레임(29.9 fps). 출발부터 열 끝 정지까지 1 s마다 한 장',
         caption='열 추종 제어기가 G-SIM 과수원(W1) 통로 하나를 처음부터 열 끝까지 달리는 차량 카메라 화면(약 99 s를 10배속, '
                 '480 폭). 실행기 컨테이너가 /scan과 raw IMU만 보고 조향과 구동을 정한다'),
    dict(name='row_follow_wf5_camera_10x', crop_bottom=CAMERA_BODY_CROP, out='docs/assets/video/row_follow_wf5_camera_10x.mp4', op='mp4',
         source=FINAL_N3, frames=(300, 3420, 15), frame_s=0.05, width=480, crf=32,
         label='G-SIM 최종 이미지 wf5 · W1 과수원 · 한 번 눌러 시작 · 10배속',
         made_by='최종 이미지 확인(STEER_FOLLOW_GSIM_FINAL_KO.md) run nominal_3(row_end, e +0.429 m)을 '
                 'tools/steer_sim/capture_media.py camera로 받은 프레임(29.6 fps). 출발 직전부터 열 끝 정지 뒤 5 s까지 0.5 s마다 한 장',
         caption='최종 이미지 wf5로 G-SIM 과수원(W1) 통로 하나를 열 끝까지 달린 차량 카메라 화면(약 104 s를 10배속, 480 폭, '
                 '저화질 H.264). 실행기가 한 번 눌러 시작으로 출발했다'),
    dict(name='row_end_stop_wf5_3x', crop_bottom=CAMERA_BODY_CROP, out='docs/assets/video/row_end_stop_wf5_3x.gif', op='gif',
         source=FINAL_N3, frames=(2550, 3420, 9), frame_s=0.1, width=360, colors=64,
         label='열 끝 다가가기와 정지 · wf5 · 3배속',
         made_by='같은 run nominal_3의 마지막 약 29 s(열 끝 정지 뒤 약 5 s 포함), 0.3 s마다 한 장',
         caption='열 끝에 다가가 서는 차량 카메라 화면(약 29 s를 3배속, 360 폭). 마지막 줄기를 지나 열 끝에서 섰다(e +0.429 m)'),
    dict(name='sim_dashboard_lidar_view', out='docs/assets/dashboard/sim_dashboard_lidar_view.png', op='png',
         source=FINAL_UI + 'drive_lidar_view.png',
         made_by='SIM 대시보드(대시보드 저장소 90094fd 사본 + SIM 확장, 이미지 wf5)를 Chrome(playwright)으로 연 화면, '
                 '세션 s2b_ui2(B 시도 2, 2026-11-13)',
         caption='SIM 대시보드 주행 화면의 LiDAR 보기. 앞쪽에 두 나무 열이 점으로 보인다(Gazebo /scan, 720점 표시). 오른쪽은 '
                 '수동 조향 칸'),
    dict(name='sim_dashboard_autonomy_running', out='docs/assets/dashboard/sim_dashboard_autonomy_running.png', op='png',
         source=FINAL_UI + 'auto_running.png',
         made_by='같은 세션, 자율 칸에서 준비 -> 자율 주행 시작을 한 번 누른 뒤',
         caption='SIM 대시보드의 자율 칸이 "주행 중"을 보이는 화면(한 번 눌러 시작). 화면을 열어 둔 동안 하트비트를 보낸다'),
    dict(name='row_follow_wf6_camera_10x', crop_bottom=CAMERA_BODY_CROP, out='docs/assets/video/row_follow_wf6_camera_10x.mp4', op='mp4',
         source=FINAL3_N3, frames=(130, 3700, 15), frame_s=0.05, width=480, crf=32,
         label='G-SIM 최종 확인 3단계 · wf6 · SIM 시계 · 한 번 눌러 시작 · 10배속',
         made_by='최종 확인 3단계(STEER_FOLLOW_GSIM_FINAL_KO.md 3단계 절) run nominal_3(SIM 시계, row_end, e +0.433 m)을 '
                 'tools/steer_sim/capture_media.py camera로 받은 프레임(29.4 fps). 출발 직전부터 열 끝 정지 뒤까지 0.5 s마다 한 장',
         caption='이미지 wf6, SIM 시계 모드로 G-SIM 과수원(W1) 통로 하나를 열 끝까지 달린 차량 카메라 화면(약 119 s를 10배속, '
                 '480 폭, 저화질 H.264)'),
    dict(name='row_end_stop_wf6_3x', crop_bottom=CAMERA_BODY_CROP, out='docs/assets/video/row_end_stop_wf6_3x.gif', op='gif',
         source=FINAL3_N3, frames=(2700, 3600, 9), frame_s=0.1, width=360, colors=64,
         label='열 끝 다가가기와 정지 · wf6 · 3배속',
         made_by='같은 run nominal_3의 마지막 약 30 s(열 끝 정지 뒤 포함), 0.3 s마다 한 장',
         caption='열 끝에 다가가 서는 차량 카메라 화면(wf6, 약 30 s를 3배속, 360 폭). 마지막 줄기를 지나 열 끝에서 섰다(e +0.433 m)'),
    dict(name='sim_dashboard_live_map', out='docs/assets/dashboard/sim_dashboard_live_map.png', op='png',
         source=FINAL3_UI + 'live_map.png',
         made_by='SIM 대시보드(대시보드 저장소 a92cfec 사본 + SIM 확장, 이미지 wf6, SIM 시계)의 지도 화면을 Chrome(playwright)으로 '
                 '연 것, 세션 s3b_ui2(3단계 B 시도 2), 그리기 시작 뒤 20 s',
         caption='SIM 대시보드 지도 화면에 실시간 격자가 쌓인 모습(대시보드 자신의 지도, SIM LiDAR 원본 스캔과 IMU 자세로 만든다)'),
    dict(name='row_follow_wf7_user_10x', crop_bottom=CAMERA_BODY_CROP, out='docs/assets/video/row_follow_wf7_user_10x.gif', op='gif', role='readme_hero',
         source=FINAL4_F3, frames=(30, 2991, 30), frame_s=0.1, width=480, colors=64,
         label='G-SIM · 사용자 로컬 명령 그대로 · 이미지 wf7 · SIM 시계 · 10배속',
         made_by='최종 확인 4단계 F(STEER_FOLLOW_GSIM_FINAL_KO.md 13절) run 3(사용자 명령 sim_stack.py start --no-browser 뒤 '
                 'sim_auto.py start, ROW_END 33.3 m)을 tools/steer_sim/capture_media.py camera로 받은 프레임(29.98 fps). '
                 '출발 직전부터 열 끝 정지 뒤까지 1 s마다 한 장(reset 앞에서 끊음)',
         caption='사용자 로컬 명령 그대로(sim_stack.py start, sim_auto.py start), 이미지 wf7, SIM 시계로 G-SIM 과수원(W1) 통로 '
                 '하나를 열 끝까지 달린 차량 카메라 화면(약 99 s를 10배속, 480 폭). 실행기가 /scan과 raw IMU만 보고 조향과 '
                 '구동을 정하고 열 끝에서 섰다'),
    dict(name='row_follow_wf7_user_camera_10x', crop_bottom=CAMERA_BODY_CROP, out='docs/assets/video/row_follow_wf7_user_camera_10x.mp4', op='mp4',
         source=FINAL4_F3, frames=(30, 2991, 15), frame_s=0.05, width=480, crf=32,
         label='G-SIM · 사용자 로컬 명령 그대로 · 이미지 wf7 · SIM 시계 · 10배속',
         made_by='같은 F run 3의 프레임, 출발 직전부터 열 끝 정지 뒤까지 0.5 s마다 한 장',
         caption='같은 run(사용자 로컬 명령, wf7, SIM 시계)의 차량 카메라를 480 폭 저화질 H.264로 줄인 것(약 99 s를 10배속)'),
    dict(name='row_follow_wf10_top_10x', out='docs/assets/video/row_follow_wf10_top_10x.gif', op='gif', ext='png',
         label=None, source=MEDIA + 'row_follow_wf10_top_10x', frames=(0, 110, 1), frame_s=0.1, width=480, colors=64,
         made_by=TOP_VIEW % ('row_follow_wf10_top_10x', '최종 확인 7단계 A 등록 run nominal_3(이미지 wf10, SIM 시계, '
                             'row_end, e +0.454 m)의 GSIM_FOLLOW_TRACE.json, 기록 처음(출발 1.2 s 전)부터 정지 3 s 뒤까지 '
                             'SIM 1 s 마다 한 장'),
         caption='이미지 wf10 으로 W1 과수원 통로 하나를 한 번 눌러 시작해 열 끝까지 달린 것을 위에서 본 그림(약 109 s 를 10배속, '
                 '480 폭). 위는 열 전체, 아래는 차를 따라가는 10 m 창(가로세로 같은 비율)'),
    dict(name='recover_side040_wf10_top_10x', out='docs/assets/video/recover_side040_wf10_top_10x.gif', op='gif',
         ext='png', label=None, source=MEDIA + 'recover_side040_wf10_top_10x', frames=(0, 104, 1), frame_s=0.1, width=480,
         colors=64,
         made_by=TOP_VIEW % ('recover_side040_wf10_top_10x', '최종 확인 7단계 A 등록 run lateral_p040(이미지 wf10, SIM 시계, '
                             '중심선 왼쪽 0.40 m 에 놓고 출발, row_end)의 GSIM_FOLLOW_TRACE.json, SIM 1 s 마다 한 장'),
         caption='중심선에서 옆으로 0.40 m 떨어진 자리에서 출발해 중심선으로 돌아오는 차를 위에서 본 그림(이미지 wf10, 약 103 s 를 '
                 '10배속). 외력을 건 것이 아니라 등록된 출발 어긋남이다'),
    dict(name='row_end_stop_wf10_top_3x', out='docs/assets/video/row_end_stop_wf10_top_3x.gif', op='gif', ext='png',
         label=None, source=MEDIA + 'row_end_stop_wf10_top_3x', frames=(0, 112, 1), frame_s=0.1, width=480, colors=64,
         made_by=TOP_VIEW % ('row_end_stop_wf10_top_3x', '같은 run nominal_3, 열 끝 선 8 m 앞부터 정지 4 s 뒤까지 SIM 0.3 s '
                             '마다 한 장'),
         caption='열 끝에 다가가 서는 차를 가까이 위에서 본 그림(이미지 wf10, 약 33 s 를 3배속). 빨간 선이 마지막 줄기 선, 앞 범퍼가 '
                 '그 선을 0.45 m 지나 섰다(등록 한도 ±1.0 m)'),
    dict(name='gnss_dropout_fsim_top_10x', out='docs/assets/video/gnss_dropout_fsim_top_10x.gif', op='gif', ext='png',
         label=None, source=MEDIA + 'gnss_dropout_fsim_top_10x', frames=(0, 69, 1), frame_s=0.1, width=480, colors=64,
         made_by=TOP_VIEW % ('gnss_dropout_fsim_top_10x', '빠른 SIM(F-SIM) perception_compare --variants off gnss_log '
                             '--only gnss_dropout_60s --gnss-seeds 0 의 run follow_gnss_dropout_60s_s0__gnss_log '
                             '(TRACE.npz, EXECUTOR.jsonl), SIM 1 s 마다 한 장'),
         caption='빠른 SIM(F-SIM, W0 평지 기둥 두 줄)에서 가상 GNSS 를 10~70 s 동안 끊은 run 을 위에서 본 그림(약 68 s 를 10배속). '
                 'GNSS 층은 기본 꺼짐이고 이 run 은 기록만(log_only) 해서 조향 입력이 아니다. 차는 LiDAR·IMU 로 열 끝까지 갔다'),
    dict(name='row_follow_wf10_user_camera_10x', crop_bottom=CAMERA_BODY_CROP, out='docs/assets/video/row_follow_wf10_user_camera_10x.gif', op='gif',
         source=STAGE8_CAM, frames=(180, 3720, 60), frame_s=0.2, width=480, colors=64,
         label='G-SIM · 사용자 로컬 명령 그대로 · 이미지 wf10 · SIM 시계 · 10배속',
         made_by='사용자 SIM 스택(ubts-steer, 이미지 wf10, SIM 시계, 장면 w1_force)에서 사용자 로컬 명령 그대로(sim_auto.py reset, '
                 'start, reset) 한 열을 달린 매체용 run(판정 아님, 2026-11-14, ROW_END 33.5 m)을 tools/steer_sim/capture_media.py '
                 'camera로 받은 프레임(29.44 fps). 출발 직전부터 열 끝 정지 뒤 약 4 s까지 약 2 s마다 한 장, 초당 5장으로 재생',
         caption='사용자 로컬 명령 그대로(sim_auto.py start), 이미지 wf10, SIM 시계로 G-SIM 과수원(W1) 통로 하나를 열 끝까지 달린 '
                 '차량 카메라 화면(약 118 s를 10배속, 480×270, 초당 5장). 실행기가 /scan과 raw IMU만 보고 조향과 구동을 정하고 '
                 '열 끝에서 섰다'),
]


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def runs_root(arg):
    value = arg or os.environ.get('UBTS_STEER_RUNS')
    if not value:
        raise SystemExit('give --runs-root or UBTS_STEER_RUNS (the runs folder of the steering SIM work)')
    path = Path(value)
    if not path.is_dir():
        raise SystemExit('runs folder not found: %s' % path)
    return path


def find_font(arg):
    candidates = [arg] if arg else [r'C:\Windows\Fonts\malgun.ttf', '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
                                    '/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc']
    for c in candidates:
        if c and Path(c).is_file():
            return Path(c)
    raise SystemExit('no Korean font found for the caption band; give --font')


def save_png(image, out: Path, width=None):
    from PIL import Image
    if width and image.width > width:
        image = image.resize((width, round(image.height * width / image.width)), Image.LANCZOS)
    if image.mode not in ('RGB', 'RGBA', 'L', 'P'):
        image = image.convert('RGB')
    if image.mode == 'RGBA' and image.getextrema()[3] == (255, 255):
        image = image.convert('RGB')
    out.parent.mkdir(parents=True, exist_ok=True)
    image.save(out, format='PNG', optimize=True)      # no pnginfo: no text chunks


def crop_bottom(image, recipe):
    """Cut the recipe's crop_bottom fraction of the height off the bottom (the rover body in camera frames)."""
    frac = recipe.get('crop_bottom')
    if not frac:
        return image
    return image.crop((0, 0, image.width, image.height - round(image.height * frac)))


def save_jpeg(image, out: Path, quality, width=None):
    from PIL import Image
    if width and image.width > width:
        image = image.resize((width, round(image.height * width / image.width)), Image.LANCZOS)
    out.parent.mkdir(parents=True, exist_ok=True)
    image.convert('RGB').save(out, format='JPEG', quality=quality, optimize=True, progressive=False,
                              subsampling='4:2:0')      # no exif argument: no APP1 segment


def _frames(recipe, src: Path, tmp: Path, font_path: Path):
    """The recipe's camera frames resized to its width with the caption band, as PNG files in tmp, and an ffmpeg
    concat list. Returns (sha256 over the frame names and bytes, number of frames, (width, height))."""
    from PIL import Image, ImageDraw, ImageFont
    first, stop, step = recipe['frames']
    indices = list(range(first, stop, step))
    names = ['%06d.%s' % (i, recipe.get('ext', 'jpg')) for i in indices]
    missing = [n for n in names if not (src / n).is_file()]
    if missing:
        raise SystemExit('missing frames in %s: %s ...' % (src, missing[:3]))
    digest = hashlib.sha256()
    font = ImageFont.truetype(str(font_path), 15)
    label = recipe.get('label')              # one line (the band of 22 px), several lines, or None: frames with their own
    texts = [] if label is None else [label] if isinstance(label, str) else list(label)    # conditions overlay
    small = ImageFont.truetype(str(font_path), 13)
    lines = []
    size = None
    for k, name in enumerate(names):
        digest.update(name.encode() + b'\0' + bytes.fromhex(sha256_file(src / name)))
        image = Image.open(src / name).convert('RGB')
        image = crop_bottom(image, recipe)
        width = recipe['width']
        height = round(image.height * width / image.width)
        if recipe['op'] == 'mp4':
            height += height % 2                       # yuv420p needs even sizes
        image = image.resize((width, height), Image.LANCZOS)
        size = image.size
        draw = ImageDraw.Draw(image)
        if len(texts) == 1:
            draw.rectangle((0, 0, image.width, 22), fill=(20, 28, 36))
            draw.text((6, 2), texts[0], font=font, fill=(255, 255, 255))
        elif texts:
            draw.rectangle((0, 0, image.width, 4 + 17 * len(texts)), fill=(20, 28, 36))
            for row, text in enumerate(texts):
                draw.text((6, 2 + 17 * row), text, font=small, fill=(255, 255, 255))
        frame = tmp / ('f%04d.png' % k)
        image.save(frame, format='PNG')
        lines.append("file '%s'\nduration %.4f\n" % (frame.as_posix(), recipe['frame_s']))
    lines.append("file '%s'\n" % (tmp / ('f%04d.png' % (len(names) - 1))).as_posix())
    (tmp / 'list.txt').write_bytes(''.join(lines).encode('utf-8'))
    return digest.hexdigest(), len(names), size


def make_gif(recipe, src: Path, out: Path, font_path: Path):
    if shutil.which('ffmpeg') is None:
        raise SystemExit('ffmpeg not on PATH')
    with tempfile.TemporaryDirectory(prefix='ubts_assets_') as tmp:
        tmp = Path(tmp)
        digest, count, _ = _frames(recipe, src, tmp, font_path)
        fps = 1.0 / recipe['frame_s']
        vf = ('fps=%g,split[a][b];[a]palettegen=max_colors=%d:stats_mode=diff[p];'
              '[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle' % (fps, recipe['colors']))
        out.parent.mkdir(parents=True, exist_ok=True)
        cmd = ['ffmpeg', '-v', 'error', '-y', '-f', 'concat', '-safe', '0', '-i', str(tmp / 'list.txt'),
               '-vf', vf, '-map_metadata', '-1', '-loop', '0', str(out)]
        subprocess.run(cmd, check=True)
    return digest, count


def make_mp4(recipe, src: Path, out: Path, font_path: Path):
    """A short low-quality H.264 clip (yuv420p, no audio, no metadata, one encoder thread so the same frames give
    the same bytes). Returns (frames sha256, number of frames, (width, height))."""
    if shutil.which('ffmpeg') is None:
        raise SystemExit('ffmpeg not on PATH')
    with tempfile.TemporaryDirectory(prefix='ubts_assets_') as tmp:
        tmp = Path(tmp)
        digest, count, size = _frames(recipe, src, tmp, font_path)
        fps = 1.0 / recipe['frame_s']
        out.parent.mkdir(parents=True, exist_ok=True)
        cmd = ['ffmpeg', '-v', 'error', '-y', '-f', 'concat', '-safe', '0', '-i', str(tmp / 'list.txt'),
               '-vf', 'fps=%g,format=yuv420p' % fps, '-c:v', 'libx264', '-preset', 'slow', '-crf', str(recipe['crf']),
               '-threads', '1', '-an', '-map_metadata', '-1', '-fflags', '+bitexact', '-flags:v', '+bitexact',
               '-movflags', '+faststart', str(out)]
        subprocess.run(cmd, check=True)
    return digest, count, size


def tool_versions():
    import PIL
    versions = dict(python='%d.%d.%d' % sys.version_info[:3], pillow=PIL.__version__)
    if shutil.which('ffmpeg'):
        line = subprocess.run(['ffmpeg', '-version'], capture_output=True, text=True).stdout.splitlines()[:1]
        versions['ffmpeg'] = line[0].split(' Copyright')[0].replace('ffmpeg version ', '') if line else 'unknown'
    return versions


def image_size(path: Path):
    from PIL import Image
    with Image.open(path) as image:
        return image.size


def load_manifest():
    path = ROOT / MANIFEST_REL
    if path.is_file():
        return json.loads(path.read_bytes().decode('utf-8'))
    return dict(schema=SCHEMA, note='', assets=[])


def write_manifest(manifest):
    manifest['note'] = ('docs/assets/ 의 모든 파일과 그 출처. 만드는 도구 tools/steer_sim/make_assets.py, 검사 '
                        'tools/steer_sim/check_assets.py, 규칙 docs/DOCS_POLICY_KO.md. <runs>는 저장소 밖 기록 폴더. '
                        '모든 그림은 자리값 기반 SIM 결과이며 현장 성능이 아니다.')
    order = {r['out']: i for i, r in enumerate(RECIPES)}
    manifest['assets'].sort(key=lambda e: (order.get(e['path'], len(order)), e['path']))
    text = json.dumps(manifest, ensure_ascii=False, indent=1) + '\n'
    (ROOT / MANIFEST_REL).write_bytes(text.encode('utf-8'))


def make(recipes, runs: Path, font_arg):
    from PIL import Image
    manifest = load_manifest()
    by_path = {e['path']: e for e in manifest['assets']}
    versions = tool_versions()
    for recipe in recipes:
        out = ROOT / recipe['out']
        src = runs / recipe['source']
        origin = dict(source='<runs>/' + recipe['source'], made_by=recipe['made_by'],
                      command='python -B tools/steer_sim/make_assets.py --runs-root <runs> --only %s' % recipe['name'],
                      tools=dict(versions))
        size = None
        if recipe['op'] in ('gif', 'mp4'):
            if recipe['op'] == 'gif':
                frames_sha, count = make_gif(recipe, src, out, find_font(font_arg))
                kind = 'animation'
            else:
                frames_sha, count, size = make_mp4(recipe, src, out, find_font(font_arg))
                kind = 'video'
            origin.update(frames='%d장 (%d..%d, %d장마다)' % ((count,) + tuple(recipe['frames'])),
                          frames_sha256=frames_sha, font=find_font(font_arg).name)
        else:
            if not src.is_file():
                raise SystemExit('source not found: %s' % src)
            origin['source_sha256'] = sha256_file(src)
            with Image.open(src) as image:
                image.load()
                image = crop_bottom(image, recipe)
                if recipe['op'] == 'png':
                    save_png(image, out, recipe.get('width'))
                else:
                    save_jpeg(image, out, recipe['quality'], recipe.get('width'))
            kind = 'image'
        width, height = size if size else image_size(out)
        digest = sha256_file(out)
        old = by_path.get(recipe['out'], {})
        visual = (old.get('privacy') or {}).get('visual_check')
        if not visual or visual.get('sha256') != digest:
            visual = None
        entry = dict(path=recipe['out'], kind=kind, bytes=out.stat().st_size, sha256=digest, width=width,
                     height=height, tag=SIM_TAG, caption=recipe['caption'], origin=origin,
                     privacy=dict(metadata='none (다시 인코딩, 문자·EXIF·주석 블록 없음)', visual_check=visual))
        if recipe.get('role'):
            entry['role'] = recipe['role']
        by_path[recipe['out']] = entry
        print('%-28s %8d B  %dx%d  %s' % (recipe['name'], entry['bytes'], width, height,
                                          'visual check kept' if visual else 'NEEDS A LOOK (--mark-checked)'))
    manifest['assets'] = list(by_path.values())
    write_manifest(manifest)


def mark_checked(names, note, date):
    manifest = load_manifest()
    by_name = {r['name']: r['out'] for r in RECIPES}
    by_path = {e['path']: e for e in manifest['assets']}
    for name in names:
        rel = by_name.get(name, name)
        entry = by_path.get(rel)
        if entry is None:
            raise SystemExit('not in the manifest: %s' % name)
        digest = sha256_file(ROOT / rel)
        if digest != entry['sha256']:
            raise SystemExit('%s: the file differs from the manifest; make it again first' % rel)
        entry.setdefault('privacy', {})['visual_check'] = dict(date=date, sha256=digest, note=note)
        print('%s: visual check %s' % (rel, date))
    write_manifest(manifest)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Make docs/assets from their sources and keep ASSETS.json.')
    parser.add_argument('--runs-root')
    parser.add_argument('--only', nargs='+', metavar='NAME')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--font')
    parser.add_argument('--mark-checked', nargs='+', metavar='NAME')
    parser.add_argument('--note', default='')
    parser.add_argument('--date', default=_dt.date.today().isoformat())
    args = parser.parse_args(argv)
    if args.list:
        for r in RECIPES:
            print('%-24s %s <- <runs>/%s' % (r['name'], r['out'], r['source']))
        return 0
    if args.mark_checked:
        if len(args.note.strip()) < 8:
            raise SystemExit('--note: say what was looked at (names, addresses, coordinates, tokens, people)')
        mark_checked(args.mark_checked, args.note.strip(), args.date)
        return 0
    recipes = RECIPES if not args.only else [r for r in RECIPES if r['name'] in args.only]
    unknown = set(args.only or ()) - {r['name'] for r in RECIPES}
    if unknown:
        raise SystemExit('unknown names: %s' % sorted(unknown))
    make(recipes, runs_root(args.runs_root), args.font)
    return 0


if __name__ == '__main__':
    sys.exit(main())
