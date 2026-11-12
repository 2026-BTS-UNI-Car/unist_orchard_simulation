"""Figures of the single-row following matrix (follow_matrix.py), drawn from the files of real runs.

Input: the output folder of follow_matrix (FOLLOW_MATRIX.json and per run TRACE.npz, EXECUTOR.jsonl). Output: PNG files
(default docs/steering/figures/steer_follow/). Truth columns are read for evaluation only (A13 role 'evaluator').
Every figure says in its footer that the numbers are placeholder-based SIM results.

  fig01_top_view      one nominal run from above: post rows, centre line, driven path, start, stop, row end
  fig02_errors        lateral and heading error against the distance along the row, with the registered limits
  fig03_steer_speed   steering command against the measured steering angle, speed and drive command against time
  fig04_matrix        every run of the matrix: measured values against the limits, pass or fail
  fig05_stale_hold    the LiDAR 1.0 s blackout: scan age, drive command, speed and the pause with its reason
  fig06_obstacle_stop the post on the centre line: front clearance, speed and the end with its reason
  fig07_top_view.gif  (--animation) the nominal run from above, one frame per second of SIM time

CLI (from sim/ubts): python -B -m steering.safety_scripts.follow_figures --runs <matrix folder> [--out <folder>]
                     [--animation]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

from . import follow_matrix as FM

FOOT = '자리값 기반 SIM 결과 (F-SIM T1 리그, W0 기둥 두 줄, 실행기 포함). 실차 성능이 아님.'
SURFACE, INK, INK2, MUTED, GRID, BAND = '#fcfcfb', '#0b0b0b', '#52514e', '#8a8984', '#e4e3df', '#d9d8d4'
BLUE, ORANGE, GREEN, RED = '#2a78d6', '#eb6834', '#1baf7a', '#c93a3a'
BLUES = ('#86b6ef', '#5c9dea', '#3987e5', '#1c5cab', '#0d366b')
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
DEFAULT_OUT = os.path.join(REPO, 'docs', 'steering', 'figures', 'steer_follow')


def _plt():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'font.family': ['Malgun Gothic', 'NanumBarunpen', 'DejaVu Sans'], 'axes.unicode_minus': False,
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'axes.edgecolor': MUTED, 'axes.labelcolor': INK, 'xtick.color': INK2, 'ytick.color': INK2,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': .8, 'axes.spines.top': False,
        'axes.spines.right': False, 'font.size': 9.5, 'axes.titlesize': 10.5, 'axes.titleweight': 'bold',
        'legend.frameon': False, 'legend.fontsize': 8.5})
    return plt


def _save(plt, fig, out_dir, name):
    fig.text(.01, -.01, FOOT, fontsize=7.5, color=INK2, ha='left', va='top')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name + '.png')
    fig.savefig(path, dpi=150, bbox_inches='tight', metadata={'Software': None})
    plt.close(fig)
    return path


class Run:
    """One run folder: the trace columns and the executor ticks."""

    def __init__(self, folder, row):
        self.row = row
        with np.load(os.path.join(folder, 'TRACE.npz')) as z:
            self.a = {k: np.asarray(z[k]) for k in z.files if k != 'meta_json'}
        self.ticks = []
        with open(os.path.join(folder, 'EXECUTOR.jsonl'), 'rb') as fh:
            for line in fh:
                record = json.loads(line)
                if record.get('k') == 'tick':
                    self.ticks.append(record)
        self.t = self.a['t_s'].astype(float)
        self.x, self.y = self.a['truth_x_m'].astype(float), self.a['truth_y_m'].astype(float)
        self.yaw = self.a['truth_yaw_rad'].astype(float)
        self.v = self.a['truth_v_m_s'].astype(float)
        self.throttle = self.a['ctrl_throttle_pct'].astype(float)
        driving = np.flatnonzero(self.throttle > 0)
        self.i0, self.i1 = (int(driving[0]), int(driving[-1])) if len(driving) else (0, len(self.t) - 1)

    def tick(self, key, sub=None):
        out = []
        for r in self.ticks:
            value = r.get(key)
            if sub is not None:
                value = (value or {}).get(sub)
            out.append(np.nan if value is None else float(value))
        return np.array([r['t'] for r in self.ticks], dtype=float), np.array(out, dtype=float)

    def changes(self):
        """(t, state, reason) where the executor state or reason changes."""
        out, last = [], None
        for r in self.ticks:
            key = (r.get('state'), r.get('reason'))
            if key != last:
                out.append((r['t'],) + key)
                last = key
        return out


def load(runs_dir):
    with open(os.path.join(runs_dir, 'FOLLOW_MATRIX.json'), 'rb') as fh:
        report = json.loads(fh.read().decode('utf-8'))
    return report, {row['name']: Run(os.path.join(runs_dir, row['name']), row) for row in report['rows']}


def _footprint(ax, x, y, yaw, **style):
    from matplotlib.patches import Polygon
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(FM.FRONT_BUMPER_M, FM.HALF_WIDTH_M), (FM.FRONT_BUMPER_M, -FM.HALF_WIDTH_M),
               (-FM.FRONT_BUMPER_M, -FM.HALF_WIDTH_M), (-FM.FRONT_BUMPER_M, FM.HALF_WIDTH_M)]
    ax.add_patch(Polygon([(x + c * u - s * w, y + s * u + c * w) for u, w in corners], closed=True, **style))


def _posts(ax, markers):
    from matplotlib.patches import Circle
    for px, py, radius in markers:
        ax.add_patch(Circle((px, py), radius, facecolor=INK2, edgecolor='none', zorder=3))


def fig_top_view(plt, run, out_dir):
    fig, (top, zoom) = plt.subplots(2, 1, figsize=(11., 5.2), gridspec_kw=dict(height_ratios=(1.25, 1.), hspace=.42))
    m = run.row['metrics']
    for ax in (top, zoom):
        ax.axhline(0., color=MUTED, lw=1., ls=(0, (6, 4)), zorder=1, label='통로 중심선')
        ax.axvline(FM.ROW_END_X_M, color=ORANGE, lw=1.2, ls=(0, (3, 3)), zorder=1, label='열 끝(마지막 기둥 선)')
        ax.plot(run.x, run.y, color=BLUE, lw=1.8, zorder=4, label='주행 경로(차체 중심, 참값)')
        ax.plot([run.x[0]], [run.y[0]], marker='o', ms=7, color=GREEN, ls='none', zorder=5, label='출발')
        ax.plot([run.x[-1]], [run.y[-1]], marker='s', ms=7, color=RED, ls='none', zorder=5, label='정지')
        ax.set_xlim(FM.ROW_START_X_M - 1., FM.ROW_END_X_M + 2.5)
    _posts(top, FM.posts())
    _footprint(top, run.x[0], run.y[0], run.yaw[0], facecolor='none', edgecolor=GREEN, lw=1.2, zorder=4)
    _footprint(top, run.x[-1], run.y[-1], run.yaw[-1], facecolor='none', edgecolor=RED, lw=1.2, zorder=4)
    top.set_aspect('equal')
    top.set_ylim(-2.4, 2.4)
    top.set_ylabel('y [m]')
    top.set_title('위에서 본 한 번의 주행 (%s, seed %d): 기둥 두 줄 사이를 따라가 열 끝에서 정지' % (
        run.row['condition'], run.row['seed']))
    top.legend(loc='upper center', bbox_to_anchor=(.5, -.16), ncol=5)
    top.annotate('e = %+.2f m' % m['row_end_error_m'], xy=(run.x[-1] + FM.FRONT_BUMPER_M, 0.),
                 xytext=(FM.ROW_END_X_M + .3, 1.2), fontsize=8.5, color=INK,
                 arrowprops=dict(arrowstyle='-', color=MUTED, lw=.8))
    zoom.axhspan(-FM.CRITERIA['lateral_max_m'], FM.CRITERIA['lateral_max_m'], color=BAND, alpha=.35, lw=0, zorder=0)
    zoom.set_ylim(-.62, .62)
    zoom.set_xlabel('열 방향 위치 x [m]')
    zoom.set_ylabel('횡 위치 y [m] (세로 확대)')
    zoom.set_title('같은 경로의 횡 위치 (회색 띠: 최대 한도 ±%.2f m)' % FM.CRITERIA['lateral_max_m'])
    return _save(plt, fig, out_dir, 'fig01_top_view')


def fig_errors(plt, runs, out_dir):
    c = FM.CRITERIA
    fig, (lat, head) = plt.subplots(2, 1, figsize=(10., 5.6), sharex=True, gridspec_kw=dict(hspace=.16))
    for k, run in enumerate(runs):
        sl = slice(run.i0, run.i1 + 1)
        label = 'seed %d' % run.row['seed']
        lat.plot(run.x[sl], run.y[sl], color=BLUES[k % len(BLUES)], lw=1.4, label=label)
        head.plot(run.x[sl], np.degrees(run.yaw[sl]), color=BLUES[k % len(BLUES)], lw=1.4, label=label)
    for sign in (1, -1):
        lat.axhline(sign * c['lateral_max_m'], color=RED, lw=1.1, label='최대 한도 ±%.2f m' % c['lateral_max_m']
                    if sign > 0 else None)
        lat.axhline(sign * c['lateral_p95_m'], color=ORANGE, lw=1., ls=(0, (5, 3)),
                    label='p95 한도 %.2f m' % c['lateral_p95_m'] if sign > 0 else None)
        lat.axhline(sign * c['lateral_rmse_m'], color=MUTED, lw=1., ls=(0, (1, 2)),
                    label='RMSE 한도 %.2f m' % c['lateral_rmse_m'] if sign > 0 else None)
        head.axhline(sign * c['heading_max_deg'], color=RED, lw=1.1,
                     label='최대 한도 ±%.0f°' % c['heading_max_deg'] if sign > 0 else None)
        head.axhline(sign * c['heading_rmse_deg'], color=MUTED, lw=1., ls=(0, (1, 2)),
                     label='RMSE 한도 %.0f°' % c['heading_rmse_deg'] if sign > 0 else None)
    lat.set_ylabel('횡오차 [m]')
    head.set_ylabel('방향 오차 [°]')
    head.set_xlabel('열 방향 위치 x [m] (출발 0 m, 열 끝 %.1f m)' % FM.ROW_END_X_M)
    lat.set_title('nominal 조건 %d회의 횡오차와 방향 오차 (주행 구간, 참값)' % len(runs))
    lat.legend(loc='upper left', bbox_to_anchor=(1.01, 1.), ncol=1)
    head.legend(loc='upper left', bbox_to_anchor=(1.01, 1.), ncol=1,
                handles=[h for h in head.get_legend_handles_labels()[0][-2:]])
    return _save(plt, fig, out_dir, 'fig02_errors')


def fig_steer_speed(plt, run, out_dir):
    fig, (steer, speed, duty) = plt.subplots(3, 1, figsize=(10., 6.8), sharex=True,
                                             gridspec_kw=dict(hspace=.18, height_ratios=(1.2, 1., .8)))
    t0 = run.t[0]
    tt, s_cmd = run.tick('S')
    _, s_meas = run.tick('steer', 'meas')
    steer.plot(run.t - t0, np.degrees(run.a['truth_delta_eq_rad'].astype(float)), color=MUTED, lw=1.,
               label='실제 조향각(플랜트 참값)')
    steer.plot(tt - t0, np.degrees(s_meas / 1000.), color=ORANGE, lw=1.3, label='측정 조향각(MCU 상태 줄)')
    steer.step(tt - t0, np.degrees(s_cmd / 1000.), where='post', color=BLUE, lw=1.3, label='조향 명령(실행기)')
    steer.set_ylabel('조향각 [°]')
    steer.legend(loc='upper left', bbox_to_anchor=(1.01, 1.))
    steer.set_title('조향 명령과 측정 조향각, 속도, 구동 명령 (%s, seed %d)' % (run.row['condition'], run.row['seed']))
    _, v_est = run.tick('v')
    _, v_ref = run.tick('sr', 'vt')
    speed.axhspan(.20, .50, color=BAND, alpha=.35, lw=0, zorder=0, label='속도 범위 0.20~0.50 m/s')
    speed.plot(run.t - t0, run.v, color=INK2, lw=1.3, label='참 속도')
    speed.plot(tt - t0, v_est, color=ORANGE, lw=1., label='추정 속도(LiDAR 정합)')
    if np.isfinite(v_ref).any():
        speed.step(tt - t0, v_ref, where='post', color=BLUE, lw=1.2, label='목표 속도')
    speed.set_ylabel('속도 [m/s]')
    speed.legend(loc='upper left', bbox_to_anchor=(1.01, 1.))
    _, throttle = run.tick('T')
    duty.step(tt - t0, throttle, where='post', color=BLUE, lw=1.2, label='구동 명령')
    if 'drive_applied_pct' in run.a:
        duty.plot(run.t - t0, run.a['drive_applied_pct'].astype(float), color=MUTED, lw=1., label='적용 duty(MCU)')
    duty.set_ylabel('duty [%]')
    duty.set_xlabel('시간 [s]')
    duty.legend(loc='upper left', bbox_to_anchor=(1.01, 1.))
    return _save(plt, fig, out_dir, 'fig03_steer_speed')


MATRIX_COLUMNS = (('lateral_rmse_m', '횡 RMSE [m]', 'lateral_rmse_m', 'lateral RMSE'),
                  ('lateral_p95_m', '횡 p95 [m]', 'lateral_p95_m', 'lateral p95'),
                  ('lateral_max_m', '횡 최대 [m]', 'lateral_max_m', 'lateral max'),
                  ('heading_rmse_deg', '방향 RMSE [°]', 'heading_rmse_deg', 'heading RMSE'),
                  ('heading_max_deg', '방향 최대 [°]', 'heading_max_deg', 'heading max'),
                  ('row_end_error_m', '정지 위치 e [m]', 'row_end_error_m', 'row end stop position e'))


def fig_matrix(plt, report, out_dir):
    rows = report['rows']
    n = len(rows)
    fig, axes = plt.subplots(1, len(MATRIX_COLUMNS) + 1, figsize=(13., .36 * n + 1.6), sharey=True,
                             gridspec_kw=dict(wspace=.12, width_ratios=(1.,) * len(MATRIX_COLUMNS) + (1.5,)))
    ys = np.arange(n)[::-1]
    for ax, (key, title, limit_key, check_name) in zip(axes, MATRIX_COLUMNS):
        limit = FM.CRITERIA[limit_key]
        signed = key == 'row_end_error_m'
        for y, row in zip(ys, rows):
            value = row['metrics'].get(key)
            if value is None:
                continue
            check = next((c for c in row['checks'] if c['check'] == check_name), None)
            if check is None:
                ax.plot([value], [y], marker='o', ms=6, mfc='none', mec=MUTED, ls='none')       # not judged here
            else:
                ax.plot([value], [y], marker='o' if check['ok'] else 'X', ms=7 if check['ok'] else 9,
                        color=GREEN if check['ok'] else RED, ls='none')
            ax.annotate(('%+.2f' if signed else '%.2f') % value, (value, y), xytext=(0, 7), textcoords='offset points',
                        ha='center', fontsize=7, color=INK2)
        ax.axvline(limit, color=RED, lw=1.1)
        if signed:
            ax.axvline(-limit, color=RED, lw=1.1)
            ax.set_xlim(-1.35 * limit, 1.35 * limit)
        else:
            ax.set_xlim(0., 1.35 * limit)
        ax.set_title(title, fontsize=9.5)
        ax.set_ylim(-.7, n - .3)
    axes[0].set_yticks(ys)
    axes[0].set_yticklabels(['%s  s%d' % (row['condition'], row['seed']) for row in rows], fontsize=8.5)
    text = axes[-1]
    text.set_axis_off()
    for y, row in zip(ys, rows):
        m = row['metrics']
        end = m.get('end_reason') or ('일시정지: %s' % (m.get('first_pause') or [None, '-'])[1])
        failed = [c['check'] for c in row['checks'] if not c['ok']]
        text.text(0., y, '%s   %s' % ('통과' if row['ok'] else '실패', end), va='center', fontsize=8.5,
                  color=GREEN if row['ok'] else RED, fontweight='bold')
        if failed:
            text.text(0., y - .36, '; '.join(failed)[:60], va='center', fontsize=6.5, color=RED)
    text.set_xlim(0., 1.)
    fig.suptitle('조건 행렬 %d회: 잰 값과 등록 한도(빨간 선). 통과 %d회, 실패 %d회. 속 빈 점은 그 조건에서 판정하지 않는 값' % (
        n, report['passed'], n - report['passed']), fontsize=10.5, fontweight='bold', y=1.0)
    return _save(plt, fig, out_dir, 'fig04_matrix')


def _mark_changes(ax, run, t0, lo, hi, y):
    for t, state, reason in run.changes():
        if lo <= t - t0 <= hi and state in ('PAUSED', 'ENDED'):
            left = t - t0 > lo + .7 * (hi - lo)                       # near the right edge: text left of the line
            ax.axvline(t - t0, color=RED, lw=1.1, ls=(0, (4, 3)))
            ax.annotate('%s: %s' % (state, reason), (t - t0, y), xytext=(-6 if left else 6, 0),
                        textcoords='offset points', fontsize=8.5, color=RED, va='top', fontweight='bold',
                        ha='right' if left else 'left')


def fig_stale(plt, run, out_dir):
    t0 = run.t[0]
    first = run.row['metrics'].get('first_pause')
    centre = (first[0] - t0) if first else FM.FAULT_AT_S
    lo, hi = centre - 4., centre + 8.
    fig, (age, duty, speed) = plt.subplots(3, 1, figsize=(10., 6.), sharex=True, gridspec_kw=dict(hspace=.18))
    tt, lidar_age = run.tick('lidar_age')
    age.plot(tt - t0, lidar_age, color=BLUE, lw=1.3, label='실행기가 본 스캔 나이')
    age.axhline(.45, color=RED, lw=1.1, label='신선도 한도 0.45 s')
    age.set_ylabel('스캔 나이 [s]')
    age.set_ylim(0., max(1.3, float(np.nanmax(lidar_age[(tt - t0 >= lo) & (tt - t0 <= hi)])) * 1.1))
    age.legend(loc='upper left', bbox_to_anchor=(1.01, 1.))
    age.set_title('고장 주입: LiDAR 1.0 s 끊김 -> 이유 stale_lidar로 일시정지, 다시 출발하지 않음')
    _mark_changes(age, run, t0, lo, hi, age.get_ylim()[1] * .97)
    _, throttle = run.tick('T')
    duty.step(tt - t0, throttle, where='post', color=BLUE, lw=1.3, label='구동 명령')
    if 'drive_applied_pct' in run.a:
        duty.plot(run.t - t0, run.a['drive_applied_pct'].astype(float), color=MUTED, lw=1., label='적용 duty(MCU)')
    duty.set_ylabel('duty [%]')
    duty.legend(loc='upper left', bbox_to_anchor=(1.01, 1.))
    speed.plot(run.t - t0, run.v, color=INK2, lw=1.3, label='참 속도')
    speed.set_ylabel('속도 [m/s]')
    speed.set_xlabel('시간 [s]')
    speed.legend(loc='upper left', bbox_to_anchor=(1.01, 1.))
    for ax in (age, duty, speed):
        ax.set_xlim(lo, hi)
    return _save(plt, fig, out_dir, 'fig05_stale_hold')


def fig_obstacle(plt, run, out_dir):
    t0 = run.t[0]
    ended = run.row['metrics'].get('ended_at_s')
    hi = (ended - t0 + 4.) if ended else float(run.t[-1] - t0)
    lo = max(0., hi - 30.)
    post_x = 12.0 - FM.POST_RADIUS_M
    fig, (gap, speed, duty) = plt.subplots(3, 1, figsize=(10., 6.), sharex=True, gridspec_kw=dict(hspace=.18))
    bumper = run.x + FM.FRONT_BUMPER_M * np.cos(run.yaw)
    tt, front = run.tick('front')
    gap.plot(run.t - t0, post_x - bumper, color=INK2, lw=1.3, label='앞범퍼에서 기둥 표면까지(참값)')
    gap.plot(tt - t0, front, color=ORANGE, lw=1., label='실행기의 앞 여유(추정)')
    gap.axhline(0., color=RED, lw=1.1, label='접촉')
    gap.set_ylabel('거리 [m]')
    gap.set_ylim(-.3, 6.)
    gap.legend(loc='upper left', bbox_to_anchor=(1.01, 1.))
    gap.set_title('중심선 위의 기둥(x = 12.0 m): 이유 obstacle_ahead로 종결, 남은 거리 %.2f m' % (
        run.row['metrics']['min_clearance_m']))
    _mark_changes(gap, run, t0, lo, hi, 5.8)
    speed.plot(run.t - t0, run.v, color=INK2, lw=1.3, label='참 속도')
    _, v_ref = run.tick('sr', 'vt')
    if np.isfinite(v_ref).any():
        speed.step(tt - t0, v_ref, where='post', color=BLUE, lw=1.2, label='목표 속도')
    speed.set_ylabel('속도 [m/s]')
    speed.legend(loc='upper left', bbox_to_anchor=(1.01, 1.))
    _, throttle = run.tick('T')
    duty.step(tt - t0, throttle, where='post', color=BLUE, lw=1.3, label='구동 명령')
    duty.set_ylabel('duty [%]')
    duty.set_xlabel('시간 [s]')
    duty.legend(loc='upper left', bbox_to_anchor=(1.01, 1.))
    for ax in (gap, speed, duty):
        ax.set_xlim(lo, hi)
    return _save(plt, fig, out_dir, 'fig06_obstacle_stop')


def animation(plt, run, out_dir, step_s=1., fps=10):
    """The nominal run from above as a GIF: one frame per step_s of SIM time (played about 10 times faster)."""
    from matplotlib.animation import FuncAnimation, PillowWriter
    fig, ax = plt.subplots(figsize=(9., 2.3), dpi=80)
    fig.subplots_adjust(left=.06, right=.99, top=.86, bottom=.2)
    _posts(ax, FM.posts())
    ax.axhline(0., color=MUTED, lw=1., ls=(0, (6, 4)), zorder=1)
    ax.axvline(FM.ROW_END_X_M, color=ORANGE, lw=1.2, ls=(0, (3, 3)), zorder=1)
    ax.set_aspect('equal')
    ax.set_xlim(FM.ROW_START_X_M - 1., FM.ROW_END_X_M + 2.5)
    ax.set_ylim(-2.4, 2.4)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    fig.text(.01, .02, FOOT, fontsize=6.5, color=INK2, ha='left', va='bottom')
    path, = ax.plot([], [], color=BLUE, lw=1.8, zorder=4)
    title = ax.set_title('')
    frames = [int(np.searchsorted(run.t, t)) for t in np.arange(run.t[0], run.t[-1], step_s)] + [len(run.t) - 1]
    body = []

    def draw(k):
        while body:
            body.pop().remove()
        path.set_data(run.x[:k + 1], run.y[:k + 1])
        _footprint(ax, run.x[k], run.y[k], run.yaw[k], facecolor='none', edgecolor=RED, lw=1.4, zorder=5)
        body.append(ax.patches[-1])
        title.set_text('시간 %5.1f s   속도 %.2f m/s   횡 위치 %+.2f m' % (run.t[k] - run.t[0], run.v[k], run.y[k]))
        return path, title

    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, 'fig07_top_view.gif')
    FuncAnimation(fig, draw, frames=frames, blit=False).save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out


def make(runs_dir, out_dir=DEFAULT_OUT, animate=False):
    plt = _plt()
    report, runs = load(runs_dir)
    nominal = [r for r in runs.values() if r.row['condition'] == 'nominal']
    out = []
    if nominal:
        out += [fig_top_view(plt, nominal[0], out_dir), fig_errors(plt, nominal, out_dir),
                fig_steer_speed(plt, nominal[0], out_dir)]
        if animate:
            out.append(animation(plt, nominal[0], out_dir))
    out.append(fig_matrix(plt, report, out_dir))
    stale = [r for r in runs.values() if r.row['expect'] == 'stale_lidar']
    if stale:
        out.append(fig_stale(plt, stale[0], out_dir))
    obstacle = [r for r in runs.values() if r.row['expect'] == 'obstacle_ahead']
    if obstacle:
        out.append(fig_obstacle(plt, obstacle[0], out_dir))
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--runs', required=True)
    p.add_argument('--out', default=DEFAULT_OUT)
    p.add_argument('--animation', action='store_true', help='also write fig07_top_view.gif')
    args = p.parse_args(argv)
    for path in make(args.runs, args.out, args.animation):
        print('%s %d bytes' % (os.path.basename(path), os.path.getsize(path)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
