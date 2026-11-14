#!/usr/bin/env python3
"""Top-view animation frames and the error / steering figures of the G-SIM media set (DOC-15), drawn from the records
of real runs. The truth columns of the records (pose, steering angle) are used for these evaluation pictures only; the
executor never sees them.

Sources (outside the repository, under the runs folder of the steering SIM work, --runs-root or UBTS_STEER_RUNS):
  G-SIM  a folder of steering.exec_checks follow (GSIM_FOLLOW_RUN.json, GSIM_FOLLOW_TRACE.json): the registered runs
         of stage 7 of the final G-SIM check, image ubts-steer-sim:wf10 (STEER_FOLLOW_GSIM_FINAL_KO.md 16)
  F-SIM  a run folder of steering.safety_scripts.perception_compare (RESULT.json, TRACE.npz, EXECUTOR.jsonl): the fast
         SIM with the real executor; the G-SIM stack has no hook that cuts its virtual GPS (GSIM_MEDIA_KO.md)

  python -B tools/steer_sim/gsim_media.py --list
  python -B tools/steer_sim/gsim_media.py --runs-root <runs> frames [NAME ...]   (PNG frames, <runs>/<FRAMES>/NAME/)
  python -B tools/steer_sim/gsim_media.py --runs-root <runs> figures [--out DIR]
  then the animation: python -B tools/steer_sim/make_assets.py --runs-root <runs> --only NAME

Every frame carries the conditions (world, profile, target speed, image or F-SIM, the tag [SIM · 자리값]). Every number
is a placeholder-based SIM result (자리값 기반 SIM 결과), never field performance. Needs numpy and matplotlib.
"""
import argparse
import bisect
import json
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'sim', 'ubts'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.dont_write_bytecode = True
from steering import gsim_follow as GF               # noqa: E402  (row frame, footprint, registered limits)
import gsim_follow_figures as FF                     # noqa: E402  (figure style of the G-SIM follow figures)

FRAMES = 'gsim_media_20261114/frames/'
GSIM_N3 = 'gsim_stage7_20261114/sessions/s7a_nominal_3_1/follow'
GSIM_L40 = 'gsim_stage7_20261114/sessions/s7a_lateral_p040_1/follow'
FSIM_ROOT = 'gsim_media_20261114/fsim/gnss_dropout/runs/'
FSIM_LOG = FSIM_ROOT + 'follow_gnss_dropout_60s_s0__gnss_log'
FSIM_OFF = FSIM_ROOT + 'follow_gnss_dropout_60s_s0__off'
FSIM_DROPOUT_S = (10., 70.)                          # perception_compare GNSS_CONDITIONS gnss_dropout_60s
TARGET = '목표 0.40 m/s'                             # field_tuning.default.json basic target_speed_m_s (both SIMs)
GSIM_LINE = '[SIM · 자리값] Gazebo 이미지 wf10 · W1 과수원 · 프로필 v0.1 · %s' % TARGET
FSIM_LINE = '[SIM · 자리값] F-SIM(빠른 SIM) · W0 평지 기둥 두 줄 · 프로필 v0.1 · %s' % TARGET
TRUTH_NOTE = '참값 경로·조향각은 평가용 그림에만'

CLIPS = {
    'row_follow_wf10_top_10x': dict(
        kind='gsim', source=GSIM_N3, speed=10., fps=10., view='follow', pad=(2., 3.),
        lines=(GSIM_LINE, '한 번 눌러 시작 · 열 끝까지 · 7단계 등록 run nominal_3'),
        note=TRUTH_NOTE),
    'recover_side040_wf10_top_10x': dict(
        kind='gsim', source=GSIM_L40, speed=10., fps=10., view='follow', pad=(2., 3.),
        lines=(GSIM_LINE, 'Gazebo wf10 · 옆 0.40 m 에서 출발해 복귀 · 외력 아님 · 참값은 평가용 그림에만'),
        note='7단계 등록 run lateral_p040 · 출발 자리(초록 테두리)'),
    'row_end_stop_wf10_top_3x': dict(
        kind='gsim', source=GSIM_N3, speed=3., fps=10., view='end', pad=(0., 4.), end_window_m=8.,
        lines=(GSIM_LINE, '열 끝 다가가기와 정지 · 마지막 줄기 선(빨강) · 7단계 등록 run nominal_3'),
        note=TRUTH_NOTE),
    'gnss_dropout_fsim_top_10x': dict(
        kind='fsim', source=FSIM_LOG, speed=10., fps=10., view='follow', pad=(0., 3.),
        lines=(FSIM_LINE, 'GNSS 10~70 s 끊김 · GNSS 층은 기본 꺼짐이라 조향 입력이 아님'),
        note='GNSS 층 기록만(log_only), 명령 열 = 꺼짐 run'),
}

W_PX, H_PX, DPI = 480, 360, 100
BAND_BG, BAND_INK = '#141c24', '#ffffff'
CAR_FACE, CAR_EDGE = '#9cc3ef', '#123f73'


# --- runs ---------------------------------------------------------------------------------------------------------------
def _read_json(path):
    with open(path, 'rb') as handle:
        return json.loads(handle.read().decode('utf-8'))


def _interp(ts, values, t):
    """Linear interpolation of a sorted series (clamped at the ends)."""
    if t <= ts[0]:
        return values[0]
    if t >= ts[-1]:
        return values[-1]
    i = bisect.bisect_right(ts, t)
    t0, t1 = ts[i - 1], ts[i]
    w = 0. if t1 == t0 else (t - t0) / (t1 - t0)
    return values[i - 1] + w * (values[i] - values[i - 1])


def _unwrap(angles):
    out, prev, offset = [], None, 0.
    for a in angles:
        if prev is not None:
            d = a - prev
            if d > math.pi:
                offset -= 2. * math.pi
            elif d < -math.pi:
                offset += 2. * math.pi
        out.append(a + offset)
        prev = a
    return out


def load_gsim(folder):
    report = _read_json(os.path.join(folder, 'GSIM_FOLLOW_RUN.json'))
    trace = _read_json(os.path.join(folder, 'GSIM_FOLLOW_TRACE.json'))
    g = trace['geometry']
    odom = trace['odom']
    t = [r[0] for r in odom]
    m = report['metrics']
    end_x, end_y, _ = GF.from_row(g, g['row_end_s'], 0., 0.)
    markers = [(mk[0], mk[1], mk[2], mk[3]) for mk in g['markers']]
    xs = [r[1] for r in odom] + [mk[0] for mk in markers]
    return dict(
        kind='gsim', name=report['run']['name'], verdict=report.get('verdict'), metrics=m,
        t=t, x=[r[1] for r in odom], y=[r[2] for r in odom], yaw=_unwrap([r[3] for r in odom]),
        steer_t=[r[0] for r in trace['steer_eq_deg']], steer_deg=[r[1] for r in trace['steer_eq_deg']],
        to_row=lambda x, y, yaw: GF.to_row(g, x, y, yaw),
        centre=(g['centre_a'], g['centre_b']), axis=g['axis_rad'], row_start_s=g['row_start_s'],
        row_end_s=g['row_end_s'], row_end_xy=(end_x, end_y), half_width=g['half_width_m'], markers=markers,
        x_range=(min(xs) - 1., max(xs) + 1.), drive=(m['drive_start_s'], m['drive_stop_s']), gnss=None,
        row_end_error_m=m.get('row_end_error_m'), end_reason=m.get('end_reason'))


def load_fsim(folder):
    import numpy as np
    result = _read_json(os.path.join(folder, 'RESULT.json'))
    z = np.load(os.path.join(folder, 'TRACE.npz'))
    t = [float(v) for v in z['t_s']]
    x, y = [float(v) for v in z['truth_x_m']], [float(v) for v in z['truth_y_m']]
    yaw = _unwrap([float(v) for v in z['truth_yaw_rad']])
    steer = [math.degrees(float(v)) for v in z['truth_delta_eq_rad']]
    world = result['spec']['world']
    markers = [(mk[0], mk[1], mk[2], 'post') for mk in world['markers']]
    gnss = []
    with open(os.path.join(folder, 'EXECUTOR.jsonl'), 'rb') as handle:
        for line in handle:
            r = json.loads(line.decode('utf-8'))
            if r.get('k') == 'tick' and 'gnss' in r:
                block = r['gnss']
                gnss.append((r['t'], bool(block.get('present')), (block.get('counts') or {}).get('accepted') or 0))
    drive_on = [i for i, v in enumerate(z['drive_applied_pct']) if abs(float(v)) > 0.]
    drive = (t[drive_on[0]], t[drive_on[-1]]) if drive_on else (t[0], t[-1])
    end_x = max(mk[0] for mk in markers)                 # W0: the last post line (follow_matrix.ROW_END_X_M)
    summary = result['summary']
    rest_e = round(x[-1] + GF.FRONT_BUMPER_M * math.cos(yaw[-1]) - end_x, 3)
    xs = x + [mk[0] for mk in markers]
    return dict(
        kind='fsim', name=result['spec']['name'], verdict=None, metrics={},
        t=t, x=x, y=y, yaw=yaw, steer_t=t, steer_deg=steer,
        to_row=lambda px, py, pyaw: (px, py, math.atan2(math.sin(pyaw), math.cos(pyaw))),
        centre=(0., 0.), axis=0., row_start_s=min(mk[0] for mk in markers), row_end_s=end_x, row_end_xy=(end_x, 0.),
        half_width=max(abs(mk[1]) for mk in markers), markers=markers, x_range=(min(xs) - 1., max(xs) + 1.),
        drive=drive, gnss=gnss, row_end_error_m=rest_e, end_reason=summary.get('end_reason'),
        commands=[(r['t'], r.get('T'), r.get('S')) for r in _ticks(folder)])


def _ticks(folder):
    out = []
    with open(os.path.join(folder, 'EXECUTOR.jsonl'), 'rb') as handle:
        for line in handle:
            r = json.loads(line.decode('utf-8'))
            if r.get('k') == 'tick':
                out.append(r)
    return out


def load(kind, folder):
    return load_gsim(folder) if kind == 'gsim' else load_fsim(folder)


def pose_at(run, t):
    x, y = _interp(run['t'], run['x'], t), _interp(run['t'], run['y'], t)
    yaw = _interp(run['t'], run['yaw'], t)
    return x, y, yaw


def gnss_at(run, t):
    """(present, accepted fixes) of the executor's GNSS block at the last tick at or before t."""
    rows = run['gnss']
    i = bisect.bisect_right([r[0] for r in rows], t) - 1
    return (False, 0) if i < 0 else (rows[i][1], rows[i][2])


# --- frames -------------------------------------------------------------------------------------------------------------
def _box(fig, left_px, bottom_px, width_px, height_px):
    return fig.add_axes((left_px / W_PX, bottom_px / H_PX, width_px / W_PX, height_px / H_PX))


def _plain(ax, face=FF.SURFACE):
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)
    ax.set_facecolor(face)
    for side in ('top', 'right', 'bottom', 'left'):
        ax.spines[side].set_visible(True)
        ax.spines[side].set_color(FF.BAND)


def _limits(centre_x, centre_y, width_px, height_px, x_span):
    """xlim / ylim of an equal-ratio view (1 m on x = 1 m on y) of x_span metres across width_px pixels."""
    y_span = x_span * height_px / width_px
    return (centre_x - x_span / 2., centre_x + x_span / 2.), (centre_y - y_span / 2., centre_y + y_span / 2.)


def _car_xy(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(GF.FRONT_BUMPER_M, GF.HALF_WIDTH_M), (GF.FRONT_BUMPER_M, -GF.HALF_WIDTH_M),
               (-GF.FRONT_BUMPER_M, -GF.HALF_WIDTH_M), (-GF.FRONT_BUMPER_M, GF.HALF_WIDTH_M)]
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in corners]


def _scene(ax, run, marker_size, line_w):
    trunks = [m for m in run['markers'] if m[3] == 'trunk']
    posts = [m for m in run['markers'] if m[3] == 'post']
    if trunks:
        ax.scatter([m[0] for m in trunks], [m[1] for m in trunks], s=marker_size, color=FF.GREEN, zorder=3, lw=0)
    if posts:
        ax.scatter([m[0] for m in posts], [m[1] for m in posts], s=marker_size * 1.2, marker='s', color=FF.MUTED,
                   zorder=3, lw=0)
    a, b = run['centre']
    x0, x1 = run['x_range']
    ax.plot([x0, x1], [a + b * x0, a + b * x1], color=FF.MUTED, lw=line_w, ls='--', zorder=2)
    ex, ey = run['row_end_xy']
    c, s = math.cos(run['axis']), math.sin(run['axis'])
    h = run['half_width'] + .4
    ax.plot([ex + s * h, ex - s * h], [ey - c * h, ey + c * h], color=FF.RED, lw=line_w * 1.4, zorder=2)


def frame_times(run, clip):
    t0, t1 = run['drive'][0] - clip['pad'][0], run['drive'][1] + clip['pad'][1]
    t0, t1 = max(t0, run['t'][0]), min(t1, run['t'][-1])
    if clip['view'] == 'end':
        along = [run['to_row'](x, y, w)[0] for x, y, w in zip(run['x'], run['y'], run['yaw'])]
        start = next(t for t, s in zip(run['t'], along) if s >= run['row_end_s'] - clip['end_window_m'])
        t0 = max(t0, start)
    step = clip['speed'] / clip['fps']
    count = int(math.floor((t1 - t0) / step)) + 1
    return [t0 + k * step for k in range(count)]


def render_frames(name, runs_root, out_root=None):
    from matplotlib.patches import Polygon, Rectangle
    plt = FF._plt()
    clip = CLIPS[name]
    run = load(clip['kind'], os.path.join(runs_root, clip['source']))
    out = os.path.join(out_root or runs_root, FRAMES + name)
    os.makedirs(out, exist_ok=True)
    for old in os.listdir(out):
        if old.endswith('.png'):
            os.remove(os.path.join(out, old))
    times = frame_times(run, clip)
    fig = plt.figure(figsize=(W_PX / DPI, H_PX / DPI), dpi=DPI)
    fig.patch.set_facecolor(FF.SURFACE)
    # band: the conditions (two lines)
    band = _box(fig, 0, H_PX - 44, W_PX, 44)
    band.set_xlim(0, 1)
    band.set_ylim(0, 1)
    band.axis('off')
    band.add_patch(Rectangle((0, 0), 1, 1, color=BAND_BG, transform=band.transAxes, zorder=0))
    band.text(.012, .7, clip['lines'][0], color=BAND_INK, fontsize=8.2, va='center', ha='left')
    band.text(.012, .27, clip['lines'][1], color=BAND_INK, fontsize=8.2, va='center', ha='left')
    t_first = times[0]
    follow = clip['view'] == 'follow'
    # overview (follow views): the whole row, equal ratio
    if follow:
        x0, x1 = run['x_range']
        ov = _box(fig, 10, 250, 460, 52)
        _plain(ov)
        span = max(x1 - x0, 2. * (run['half_width'] + .45) * 460 / 52)   # both rows inside, equal ratio
        xl, yl = _limits((x0 + x1) / 2., run['centre'][0] + run['centre'][1] * (x0 + x1) / 2., 460, 52, span)
        _scene(ov, run, 3., .7)
        ov.set_xlim(*xl)
        ov.set_ylim(*yl)
        ov_trail, = ov.plot([], [], color=FF.BLUE, lw=1.1, zorder=4)
        ov_car = Polygon(_car_xy(run['x'][0], run['y'][0], run['yaw'][0]), closed=True, facecolor=CAR_EDGE,
                         edgecolor=CAR_EDGE, lw=.5, zorder=5)
        ov.add_patch(ov_car)
        ov_win = Rectangle((0, yl[0]), 1, yl[1] - yl[0], facecolor=FF.ORANGE, edgecolor=FF.ORANGE, alpha=.18, lw=.8,
                           zorder=1)                     # the part of the row shown below
        ov.add_patch(ov_win)
        near_bottom, near_height, near_span = 50, 192, 10.
    else:
        near_bottom, near_height, near_span = 50, 250, 10.
    near = _box(fig, 10, near_bottom, 460, near_height)
    _plain(near)
    _scene(near, run, 16., 1.)
    start_pose = pose_at(run, t_first)
    if follow:                                           # the start pose (green outline)
        near.add_patch(Polygon(_car_xy(*start_pose), closed=True, fill=False, edgecolor=FF.GREEN, lw=1.1, zorder=4))
    trail, = near.plot([], [], color=FF.BLUE, lw=1.6, zorder=4)
    car = Polygon(_car_xy(*start_pose), closed=True, facecolor=CAR_FACE, edgecolor=CAR_EDGE, lw=1.2, zorder=6,
                  alpha=.95)
    near.add_patch(car)
    nose, = near.plot([], [], color=CAR_EDGE, lw=2.6, zorder=7, solid_capstyle='butt')
    scale_bar, = near.plot([], [], color=FF.INK, lw=2., zorder=7)
    scale_text = near.text(0, 0, '1 m', fontsize=7.5, color=FF.INK, ha='center', va='bottom', zorder=7)
    if clip['view'] == 'end':
        ex, ey = run['row_end_xy']
        end_centre = (ex - 3.2, ey)
        nxl, nyl = _limits(end_centre[0], end_centre[1], 460, near_height, near_span)
        near.set_xlim(*nxl)
        near.set_ylim(*nyl)
        near.text(ex - .08, nyl[1] - .12, '열 끝(마지막 줄기 선)', color=FF.RED, fontsize=7.5, ha='right', va='top')
    # status (two lines)
    status = fig.add_axes((0, 0, 1, 46 / H_PX))
    status.axis('off')
    s1 = status.text(.012, .68, '', color=FF.INK, fontsize=8.2, va='center', ha='left')
    s2 = status.text(.012, .24, '', color=FF.INK2, fontsize=7.6, va='center', ha='left')
    trail_x, trail_y = [], []
    k_src = 0
    for k, t in enumerate(times):
        while k_src < len(run['t']) and run['t'][k_src] <= t:
            if run['t'][k_src] >= t_first:
                trail_x.append(run['x'][k_src])
                trail_y.append(run['y'][k_src])
            k_src += 1
        x, y, yaw = pose_at(run, t)
        tx, ty = trail_x + [x], trail_y + [y]
        trail.set_data(tx, ty)
        corners = _car_xy(x, y, yaw)
        car.set_xy(corners)
        nose.set_data([corners[0][0], corners[1][0]], [corners[0][1], corners[1][1]])
        if follow:
            ov_trail.set_data(tx, ty)
            ov_car.set_xy(corners)
            a, b = run['centre']
            cx = x + 2.
            nxl, nyl = _limits(cx, a + b * cx, 460, near_height, near_span)
            near.set_xlim(*nxl)
            near.set_ylim(*nyl)
            ov_win.set_x(nxl[0])
            ov_win.set_width(nxl[1] - nxl[0])
        xl_now, yl_now = near.get_xlim(), near.get_ylim()
        bx, by = xl_now[0] + .35, yl_now[0] + .22
        scale_bar.set_data([bx, bx + 1.], [by, by])
        scale_text.set_position((bx + .5, by + .04))
        along, lateral, heading = run['to_row'](x, y, yaw)
        steer = _interp(run['steer_t'], run['steer_deg'], t)
        line = 't %5.1f s · 열 위치 %5.1f m · 횡오차 %+.2f m · 조향각 %+.1f° · %g배속' % (
            t - run['drive'][0], along, lateral, steer, clip['speed'])
        if clip['view'] == 'end':
            front = along + GF.FRONT_BUMPER_M * math.cos(heading) - run['row_end_s']
            line = 't %5.1f s · 앞 범퍼 − 열 끝 선 %+.2f m · 조향각 %+.1f° · %g배속' % (
                t - run['drive'][0], front, steer, clip['speed'])
        s1.set_text(line)
        note = clip['note']
        if run['gnss'] is not None:
            present, accepted = gnss_at(run, t)
            note = ('GNSS fix 받음(%d개) · ' % accepted if present else 'GNSS 없음 → LiDAR·IMU 로 계속 · ') + note
        if t >= run['drive'][1] and run.get('end_reason'):
            note = '정지(%s) · ' % run['end_reason'] + note
        s2.set_text(note)
        fig.savefig(os.path.join(out, '%06d.png' % k), dpi=DPI, metadata={'Software': None})
    plt.close(fig)
    meta = dict(name=name, source='<runs>/' + clip['source'], frames=len(times), sim_step_s=clip['speed'] / clip['fps'],
                t_first=round(times[0], 3), t_last=round(times[-1], 3), result_kind='자리값 기반 SIM 결과')
    with open(os.path.join(out, 'FRAMES.json'), 'wb') as handle:
        handle.write((json.dumps(meta, ensure_ascii=False, indent=1) + '\n').encode('utf-8'))
    return out, len(times)


# --- figures ------------------------------------------------------------------------------------------------------------
FSIM_FOOT = '자리값 기반 SIM 결과 (빠른 SIM F-SIM, W0 평지 기둥 두 줄, 실제 실행기). 실차 성능이 아님.'
GSIM_FOOT = '자리값 기반 SIM 결과 (G-SIM 스택, 이미지 wf10, w1_force 과수원, 실행기 컨테이너). 실차 성능이 아님.'


def _save(plt, fig, out_dir, name, foot):
    os.makedirs(out_dir, exist_ok=True)
    fig.text(.01, .005, foot, fontsize=7.5, color=FF.MUTED, ha='left', va='bottom')
    path = os.path.join(out_dir, name + '.png')
    fig.savefig(path, dpi=150, bbox_inches='tight', metadata={'Software': None})
    plt.close(fig)
    return path


def _series(run):
    """(along, lateral, steering degree) over the driving part (truth)."""
    t0, t1 = run['drive']
    rows = []
    for t, x, y, yaw in zip(run['t'], run['x'], run['y'], run['yaw']):
        if t0 <= t <= t1:
            along, lateral, _ = run['to_row'](x, y, yaw)
            rows.append((along, lateral, _interp(run['steer_t'], run['steer_deg'], t)))
    return rows


def fig_gsim(plt, runs, out_dir):
    c = GF.CRITERIA
    fig, (lat, steer) = plt.subplots(2, 1, figsize=(10.5, 5.8), sharex=True, gridspec_kw=dict(hspace=.16))
    for i, (label, run) in enumerate(runs):
        rows = _series(run)
        color = FF.SERIES[i % len(FF.SERIES)]
        lat.plot([r[0] for r in rows], [r[1] for r in rows], color=color, lw=1.3, label=label)
        steer.plot([r[0] for r in rows], [r[2] for r in rows], color=color, lw=1.1, label=label)
    g = runs[0][1]
    lat.axhline(c['lateral_max_m'], color=FF.RED, lw=1., ls='--')
    lat.axhline(-c['lateral_max_m'], color=FF.RED, lw=1., ls='--', label='횡 최대 한도 ±%.2f m' % c['lateral_max_m'])
    lat.axhline(c['lateral_p95_m'], color=FF.ORANGE, lw=.9, ls=':', label='p95 한도 %.2f m' % c['lateral_p95_m'])
    lat.axhline(-c['lateral_p95_m'], color=FF.ORANGE, lw=.9, ls=':')
    for ax in (lat, steer):
        ax.axhline(0., color=FF.MUTED, lw=.8)
        ax.axvline(g['row_start_s'], color=FF.GREEN, lw=1., ls=':')
        ax.axvline(g['row_end_s'], color=FF.RED, lw=1.1)
    lat.set_ylabel('횡오차 [m] (참값, 왼쪽 +)')
    steer.set_ylabel('조향 등가각 [°] (참값, 왼쪽 +)')
    steer.set_xlabel('열 축 방향 위치 s [m] (초록 점선: 첫 줄기, 빨간 선: 열 끝)')
    lat.set_title('G-SIM wf10 열 추종: 횡오차와 조향각 대 열 위치 (7단계 등록 run, 영상과 같은 기록)')
    lat.legend(loc='upper right', ncol=2)
    steer.legend(loc='upper right')
    return _save(plt, fig, out_dir, 'gsim_fig03_lateral_steer_wf10', GSIM_FOOT)


def fig_fsim(plt, off, log, out_dir):
    fig, (lat, steer, gn) = plt.subplots(3, 1, figsize=(10.5, 6.6), sharex=True,
                                         gridspec_kw=dict(hspace=.18, height_ratios=(1., 1., .38)))
    t_drop = [t for t in log['t'] if FSIM_DROPOUT_S[0] <= t <= FSIM_DROPOUT_S[1]]
    x_drop = (pose_at(log, t_drop[0])[0], pose_at(log, min(t_drop[-1], log['t'][-1]))[0])
    for ax in (lat, steer, gn):
        ax.axvspan(x_drop[0], x_drop[1], color=FF.BAND, alpha=.55, lw=0)
        ax.axvline(log['row_end_s'], color=FF.RED, lw=1.1)
    for run, color, lw, ls, label in ((off, FF.BLUE, 2.4, '-', 'GNSS 층 꺼짐(기본) run'),
                                      (log, FF.ORANGE, 1.1, '--', 'GNSS 층 기록만(log_only) run, 같은 seed')):
        rows = _series(run)
        lat.plot([r[0] for r in rows], [r[1] for r in rows], color=color, lw=lw, ls=ls, label=label)
        steer.plot([r[0] for r in rows], [r[2] for r in rows], color=color, lw=lw, ls=ls, label=label)
    lat.axhline(0., color=FF.MUTED, lw=.8)
    steer.axhline(0., color=FF.MUTED, lw=.8)
    xs, present = [], []
    for t, p, _ in log['gnss']:
        if log['drive'][0] - 1. <= t <= log['t'][-1]:
            xs.append(pose_at(log, t)[0])
            present.append(1 if p else 0)
    gn.step(xs, present, where='post', color=FF.INK2, lw=1.2)
    gn.set_yticks((0, 1))
    gn.set_yticklabels(('없음', '있음'))
    gn.set_ylim(-.3, 1.3)
    gn.set_ylabel('GNSS fix\n(실행기 기록)')
    same = off['commands'] == log['commands']
    lat.set_ylabel('횡오차 [m] (참값, 왼쪽 +)')
    steer.set_ylabel('조향 등가각 [°] (참값, 왼쪽 +)')
    gn.set_xlabel('열 축 방향 위치 x [m] (회색 띠: GNSS 끊김 10~70 s 의 위치, 빨간 선: 열 끝 기둥 선)')
    lat.set_title('F-SIM GNSS 끊김: 횡오차와 조향각 대 열 위치 (두 run 의 실행기 명령 열 %s)' % ('같음' if same else '다름'))
    lat.legend(loc='lower right')
    return _save(plt, fig, out_dir, 'gsim_fig04_gnss_dropout_fsim', FSIM_FOOT)


def figures(runs_root, out_dir):
    plt = FF._plt()
    n3 = load_gsim(os.path.join(runs_root, GSIM_N3))
    l40 = load_gsim(os.path.join(runs_root, GSIM_L40))

    def label(run):
        return '%s (%s, %s, 횡 RMSE %.3f m)' % (run['name'], run['verdict'], run['end_reason'],
                                               run['metrics'].get('lateral_rmse_m'))
    paths = [fig_gsim(plt, [(label(n3), n3), (label(l40), l40)], out_dir)]
    off = load_fsim(os.path.join(runs_root, FSIM_OFF))
    log = load_fsim(os.path.join(runs_root, FSIM_LOG))
    paths.append(fig_fsim(plt, off, log, out_dir))
    return paths, off['commands'] == log['commands']


def runs_root(value):
    value = value or os.environ.get('UBTS_STEER_RUNS')
    if not value or not os.path.isdir(value):
        raise SystemExit('give --runs-root or UBTS_STEER_RUNS (the runs folder of the steering SIM work)')
    return value


def main(argv=None):
    p = argparse.ArgumentParser(prog='gsim_media', description=__doc__.splitlines()[0])
    p.add_argument('--runs-root')
    p.add_argument('--list', action='store_true')
    sub = p.add_subparsers(dest='command')
    f = sub.add_parser('frames')
    f.add_argument('names', nargs='*')
    g = sub.add_parser('figures')
    g.add_argument('--out', default=FF.DEFAULT_OUT)
    args = p.parse_args(argv)
    if args.list:
        for name, clip in CLIPS.items():
            print('%-30s %s <- <runs>/%s' % (name, clip['kind'], clip['source']))
        return 0
    root = runs_root(args.runs_root)
    if args.command == 'frames':
        for name in args.names or list(CLIPS):
            if name not in CLIPS:
                raise SystemExit('unknown clip %s' % name)
            out, count = render_frames(name, root)
            print('%-30s %4d frames -> <runs>/%s%s' % (name, count, FRAMES, name))
        return 0
    if args.command == 'figures':
        paths, same = figures(root, args.out)
        for path in paths:
            print(os.path.relpath(path, REPO))
        print('F-SIM command sequences equal (off vs log_only): %s' % same)
        return 0
    p.print_help()
    return 2


if __name__ == '__main__':
    sys.exit(main())
