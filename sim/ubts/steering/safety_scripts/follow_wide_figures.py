"""Figures of the wide following matrix (follow_wide.py), drawn from the files of real runs.

Input: an output folder of follow_wide (FOLLOW_WIDE.json and per run runs/<name>/TRACE.npz, EXECUTOR.jsonl, SERIAL.jsonl).
Output: PNG files (default docs/steering/figures/steer_follow_wide/) and, with --gif <path>, a top-view animation.
Truth columns and the world geometry are read for evaluation only (A13 role 'evaluator'). Every figure says in its
footer that the numbers are placeholder-based SIM results.

  fig01_top_views     driven paths of all seeds of a few conditions from above (world frame; W2 in the corridor frame)
  fig02_lateral       lateral error per condition: max and RMSE of every seed against the registered limits
  fig03_matrix        the condition matrix: passed seeds per check group
  fig04_row_end       the row end stop position e against the last structure (R50) and the old reference (last tree)
  fig05_failure       one failed run in time: steering command / measured angle, MCU steering output, lateral offset
  --gif PATH          one run from above, one frame per SIM second, at most 640 px wide

CLI (from sim/ubts): python -B -m steering.safety_scripts.follow_wide_figures --runs <follow_wide folder>
                     [--out <folder>] [--failure <run name>] [--gif <path> --gif-run <run name>]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

from . import follow_figures as FF
from . import follow_wide as FW

FOOT = '자리값 기반 SIM 결과 (F-SIM T1 리그, 실행기 포함, 등록 steer_follow_wide_v2). 실차 성능이 아님.'
SURFACE, INK, INK2, MUTED, GRID = FF.SURFACE, FF.INK, FF.INK2, FF.MUTED, FF.GRID
BLUE, ORANGE, GREEN, RED, PURPLE = FF.BLUE, FF.ORANGE, FF.GREEN, FF.RED, '#7d4fc4'
PASS_FILL, FAIL_FILL, NONE_FILL = '#d6efe4', '#f6d5d2', '#efeeeb'
DEFAULT_OUT = os.path.join(FF.REPO, 'docs', 'steering', 'figures', 'steer_follow_wide')
TOP_VIEW = ('nominal', 'curve_l30', 'curve_r60', 'gap3_both', 'endpost_1p2', 'w2_045553_c03')
GROUPS = (   # column label, check names (any check whose name starts with one of these)
    ('끝·정지 이유', ('ends at the row end', 'ends with', 'first pause is', 'not a row end', '(row end branch) ends',
                   'first steering fault', '(fault branch) first')),
    ('일시정지 없음', ('no pause after', '(row end branch) no pause')),
    ('횡 RMSE·p95', ('lateral RMSE', 'lateral p95', '(row end branch) lateral RMSE', '(row end branch) lateral p95')),
    ('횡 최대', ('lateral max', '(row end branch) lateral max', '(fault branch) lateral max')),
    ('방향', ('heading', '(row end branch) heading')),
    ('속도', ('true speed max', '(row end branch) true speed')),
    ('열 끝 e', ('row end stop position', '(row end branch) row end')),
    ('정지 시간', ('drive command 0', 'steering fault within', '(fault branch) steering fault within')),
    ('재출발 없음', ('no restart',)),
    ('접촉·정지·A22', ('no contact', 'at rest', 'no guard', 'no commanded steering')),
    ('조향 결함 0', ('no steering fault line', 'no host bridge latch')),
    ('GNSS', ('GNSS',)),
)


def _plt():
    return FF._plt()


def _save(plt, fig, out_dir, name):
    fig.text(.01, -.01, FOOT, fontsize=7.5, color=INK2, ha='left', va='top')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name + '.png')
    fig.savefig(path, dpi=110, bbox_inches='tight', metadata={'Software': None})
    plt.close(fig)
    return path


def load(folder):
    with open(os.path.join(folder, 'FOLLOW_WIDE.json'), 'rb') as fh:
        return json.loads(fh.read().decode('utf-8'))


class Run:
    def __init__(self, folder, row):
        self.row, self.folder = row, folder
        with np.load(os.path.join(folder, 'TRACE.npz')) as z:
            self.a = {k: np.asarray(z[k]) for k in z.files if k != 'meta_json'}
        self.t = self.a['t_s'].astype(float)
        self.x, self.y = self.a['truth_x_m'].astype(float), self.a['truth_y_m'].astype(float)
        self.yaw = self.a['truth_yaw_rad'].astype(float)
        self.cond = FW.condition(row['condition'])
        thr = self.a['ctrl_throttle_pct'].astype(float)
        driving = np.flatnonzero(thr > 0)
        self.i0, self.i1 = (int(driving[0]), int(driving[-1])) if len(driving) else (0, len(self.t) - 1)
        if self.cond.get('w2'):
            self.s, self.d = self.a['truth_field_x_m'].astype(float), self.a['truth_field_y_m'].astype(float)
        else:
            self.geo = FW.geometry(self.cond.get('world'))
            self.s, self.d, _ = self.geo.frame(self.x, self.y, self.yaw)


def runs_of(report, folder, condition):
    return [Run(os.path.join(folder, 'runs', r['name']), r) for r in report['rows'] if r['condition'] == condition]


def _footprint(ax, x, y, yaw, color, ls='-'):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(FW.FRONT_BUMPER_M, FW.HALF_WIDTH_M), (FW.FRONT_BUMPER_M, -FW.HALF_WIDTH_M),
           (-FW.FRONT_BUMPER_M, -FW.HALF_WIDTH_M), (-FW.FRONT_BUMPER_M, FW.HALF_WIDTH_M),
           (FW.FRONT_BUMPER_M, FW.HALF_WIDTH_M)]
    ax.plot([x + c * a - s * b for a, b in pts], [y + s * a + c * b for a, b in pts], color=color, lw=1.2, ls=ls)


def fig_top_views(plt, report, folder, out_dir, names=TOP_VIEW):
    names = [n for n in names if any(r['condition'] == n for r in report['rows'])]
    fig, axes = plt.subplots(len(names), 1, figsize=(10.5, 2.35 * len(names)))
    axes = np.atleast_1d(axes)
    for ax, name in zip(axes, names):
        runs = runs_of(report, folder, name)
        if not runs:
            continue
        c = runs[0].cond
        passed = sum(1 for r in runs if r.row['ok'])
        if c.get('w2'):
            from closed_loop import raster_corridor as rc
            cfg = rc.corridor_world_config(c['w2']['record'], c['w2']['corridor_id'], c['w2'].get('direction', 'fwd'))
            for units in (cfg.left_units, cfg.right_units):
                if units:
                    u = np.asarray(units, dtype=float)
                    ax.scatter(u[:, 0], u[:, 1], s=6, color=MUTED, lw=0)
            ax.axhline(0., color=INK2, lw=.8, ls='--')
            last = max(cfg.line_end_s_m)
            ax.axvline(last, color=INK, lw=.8, ls=':')
            for r in runs:
                ax.plot(r.s[r.i0:], r.d[r.i0:], color=BLUE, lw=1.1, alpha=.85)
                ax.plot(r.s[-1], r.d[-1], marker='o', ms=4, color=ORANGE)
            ax.set_xlabel('회랑 s [m]')
            ax.set_ylabel('d [m]')
            ax.set_ylim(-2.6, 2.6)
            title = '%s (10/10 회랑, 회랑 좌표, 옆 확대)' % name
        else:
            geo = runs[0].geo
            posts = np.asarray(geo.posts(), dtype=float)
            for px, py, pr in posts:
                ax.add_patch(plt.Circle((px, py), pr, color=MUTED, lw=0))
            s = np.linspace(geo.start_s_m, geo.end_s_m + 3., 200)
            cx = [geo.centre(v)[0] for v in s]
            cy = [geo.centre(v)[1] for v in s]
            ax.plot(cx, cy, color=INK2, lw=.8, ls='--')
            ex1, ey1 = geo.point(geo.last_structure_s(), geo.half_width_m + .6)
            ex2, ey2 = geo.point(geo.last_structure_s(), -geo.half_width_m - .6)
            ax.plot([ex1, ex2], [ey1, ey2], color=INK, lw=.8, ls=':')
            for r in runs:
                ax.plot(r.x[r.i0:], r.y[r.i0:], color=BLUE, lw=1.1, alpha=.85)
            r = runs[0]
            _footprint(ax, r.x[0], r.y[0], r.yaw[0], GREEN)
            _footprint(ax, r.x[-1], r.y[-1], r.yaw[-1], ORANGE)
            xs = np.concatenate([posts[:, 0]] + [r.x for r in runs])
            ys = np.concatenate([posts[:, 1]] + [r.y for r in runs])
            ax.set_xlim(xs.min() - 1.5, xs.max() + 1.5)
            ax.set_ylim(ys.min() - 1., ys.max() + 1.)
            ax.set_aspect('equal', adjustable='box')
            ax.set_xlabel('x [m]')
            ax.set_ylabel('y [m]')
            title = name
        ax.set_title('%s: seed %d개 중 %d개 통과' % (title, len(runs), passed), loc='left')
    fig.tight_layout()
    fig.text(.01, 1.0, '파란 선: 차체 중심 경로(seed마다), 회색: 기둥(나무·끝 기둥), 점선: 통로 중심선, 점점선: 마지막 구조물, '
             '초록 사각형: 출발, 주황: 정지', fontsize=8, color=INK2, va='bottom')
    return _save(plt, fig, out_dir, 'fig01_top_views')


def fig_lateral(plt, report, out_dir):
    conds = [c['name'] for c in FW.CONDITIONS if any(r['condition'] == c['name'] for r in report['rows'])]
    fig, ax = plt.subplots(figsize=(8.6, .2 * len(conds) + 1.4))
    crit = FW.CRITERIA
    for k, name in enumerate(conds):
        rows = [r for r in report['rows'] if r['condition'] == name]
        ys = k
        for r in rows:
            m = r['metrics']
            mx, rmse = m.get('lateral_max_m'), m.get('lateral_rmse_m')
            if mx is not None:
                ax.plot(mx, ys, marker='o', ms=4.5, mfc='none' if r['ok'] else RED, mec=BLUE if r['ok'] else RED,
                        mew=1.2, ls='none')
            if rmse is not None:
                ax.plot(rmse, ys, marker='|', ms=8, color=ORANGE, mew=1.6, ls='none')
    for limit, label in ((crit['lateral_rmse_m'], 'RMSE 한도 0.20'), (crit['lateral_max_m'], '최대 한도 0.50')):
        ax.axvline(limit, color=INK2, lw=.9, ls='--')
        ax.text(limit, -1.2, label, fontsize=7.5, color=INK2, ha='center', va='bottom')
    ax.set_yticks(range(len(conds)))
    ax.set_yticklabels(conds, fontsize=7.5)
    ax.invert_yaxis()
    ax.set_xlabel('횡오차 [m] (주행 구간 참값, 출발 어긋남 조건의 RMSE는 6 m 뒤부터)')
    ax.set_xlim(0., max(.6, max((r['metrics'].get('lateral_max_m') or 0.) for r in report['rows']) + .05))
    ax.plot([], [], marker='o', ms=4.5, mfc='none', mec=BLUE, ls='none', label='run의 횡 최대(통과 run)')
    ax.plot([], [], marker='o', ms=4.5, color=RED, ls='none', label='run의 횡 최대(실패 run)')
    ax.plot([], [], marker='|', ms=8, color=ORANGE, mew=1.6, ls='none', label='run의 횡 RMSE')
    ax.legend(loc='lower right')
    ax.set_title('조건별 횡오차 (%s seed %s)' % (report['seed_set'], ','.join(str(s) for s in report['seeds'])),
                 loc='left')
    return _save(plt, fig, out_dir, 'fig02_lateral')


def _group_counts(rows):
    out = []
    for label, prefixes in GROUPS:
        judged = passed = 0
        for r in rows:
            mine = [c for c in r['checks'] if any(c['check'].startswith(p) for p in prefixes)]
            if mine:
                judged += 1
                passed += int(all(c['ok'] for c in mine))
        out.append((label, judged, passed))
    return out


def fig_matrix(plt, report, out_dir):
    conds = [c['name'] for c in FW.CONDITIONS if any(r['condition'] == c['name'] for r in report['rows'])]
    fig, ax = plt.subplots(figsize=(11.2, .22 * len(conds) + 1.6))
    ax.set_xlim(0, len(GROUPS) + 1)
    ax.set_ylim(len(conds), -1)
    ax.axis('off')
    ax.grid(False)
    for j, (label, _) in enumerate(GROUPS):
        ax.text(j + 1.5, -.6, label, ha='center', va='bottom', fontsize=7.5, color=INK, rotation=30)
    ax.text(.5, -.6, '전체', ha='center', va='bottom', fontsize=7.5, color=INK, fontweight='bold')
    for i, name in enumerate(conds):
        rows = [r for r in report['rows'] if r['condition'] == name]
        ok = sum(1 for r in rows if r['ok'])
        fill = PASS_FILL if ok == len(rows) else FAIL_FILL
        ax.add_patch(plt.Rectangle((0.04, i + .06), .92, .88, color=fill, lw=0))
        ax.text(.5, i + .5, '%d/%d' % (ok, len(rows)), ha='center', va='center', fontsize=7.5, color=INK,
                fontweight='bold')
        ax.text(-.05, i + .5, name, ha='right', va='center', fontsize=7.5, color=INK)
        for j, (label, judged, passed) in enumerate(_group_counts(rows)):
            if not judged:
                fill, text = NONE_FILL, '-'
            else:
                fill, text = (PASS_FILL if passed == judged else FAIL_FILL), '%d/%d' % (passed, judged)
            ax.add_patch(plt.Rectangle((j + 1.04, i + .06), .92, .88, color=fill, lw=0))
            ax.text(j + 1.5, i + .5, text, ha='center', va='center', fontsize=7, color=INK if judged else MUTED)
    ax.set_title('조건 행렬: 통과한 seed 수 / 판정한 seed 수 (초록 = 모두 통과, 빨강 = 실패 있음, 회색 = 판정 안 함)',
                 loc='left', pad=34)
    return _save(plt, fig, out_dir, 'fig03_matrix')


def fig_row_end(plt, report, out_dir):
    names = [c['name'] for c in FW.CONDITIONS if c['expect'] in ('row_end', 'row_end_or_steer_fault')
             and any(r['condition'] == c['name'] and r['metrics'].get('end_reason') == 'row_end' for r in report['rows'])]
    fig, ax = plt.subplots(figsize=(8.6, .2 * len(names) + 1.4))
    for k, name in enumerate(names):
        for r in report['rows']:
            if r['condition'] != name or r['metrics'].get('end_reason') != 'row_end':
                continue
            m = r['metrics']
            ax.plot(m['row_end_error_m'], k, marker='o', ms=4.5, mfc='none', mec=BLUE, mew=1.2, ls='none')
            if abs(m['row_end_error_last_tree_m'] - m['row_end_error_m']) > 1e-6:
                ax.plot(m['row_end_error_last_tree_m'], k, marker='x', ms=5, color=ORANGE, mew=1.2, ls='none')
    for v in (-1., 1.):
        ax.axvline(v, color=INK2, lw=.9, ls='--')
    ax.axvline(0., color=GRID, lw=1.)
    ax.set_yticks(range(len(names)))
    ax.set_yticklabels(names, fontsize=7.5)
    ax.invert_yaxis()
    ax.set_xlabel('e = 정지 때 앞범퍼 s − 기준 s [m] (띠 ±1.0 m)')
    ax.plot([], [], marker='o', ms=4.5, mfc='none', mec=BLUE, ls='none', label='기준: 열 줄의 마지막 구조물(R50, 판정)')
    ax.plot([], [], marker='x', ms=5, color=ORANGE, ls='none', label='옛 기준: 마지막 나무(끝 기둥이 있을 때만 다름)')
    ax.legend(loc='lower right')
    ax.set_title('열 끝에서 row_end로 선 run의 정지 위치', loc='left')
    return _save(plt, fig, out_dir, 'fig04_row_end')


def fig_failure(plt, report, folder, out_dir, name):
    row = next(r for r in report['rows'] if r['name'] == name)
    run = Run(os.path.join(folder, 'runs', name), row)
    a, t = run.a, run.t
    lines = []
    with open(os.path.join(folder, 'runs', name, 'SERIAL.jsonl'), 'rb') as fh:
        for line in fh:
            lines.append(json.loads(line))
    m = row['metrics']
    faults = m.get('steer_fault_lines') or []
    t_end = (faults[0][1] + 2.) if faults else float(t[-1])
    onset = min((f['at_s'] for f in run.cond.get('faults') or ()), default=None)
    t0 = 0. if onset is None else max(0., onset - 3.)
    sel = (t >= t0) & (t <= t_end)
    fig, axes = plt.subplots(3, 1, figsize=(9.2, 6.6), sharex=True)
    ax = axes[0]
    ax.plot(t[sel], np.degrees(a['ctrl_steer_mrad'][sel] / 1000.), color=BLUE, lw=1.4, label='실행기 조향 명령')
    ax.plot(t[sel], np.degrees(a['truth_mcu_pos_rad'][sel]), color=ORANGE, lw=1.2, label='MCU 가변저항 각(측정)')
    ax.plot(t[sel], np.degrees(a['truth_delta_eq_rad'][sel]), color=PURPLE, lw=1.2, label='참 바퀴 등가각')
    ax.set_ylabel('조향각 [°]')
    ax.legend(loc='upper left', ncol=3)
    ax = axes[1]
    ax.plot(t[sel], 100. * a['truth_mcu_steer_pwm'][sel] / 255., color=BLUE, lw=1.3, label='MCU 조향 출력')
    for v in (60., -60.):
        ax.axhline(v, color=INK2, lw=.9, ls='--')
    ax.text(t[sel][0], 66., 'STALL 문턱 |출력| 60 % (0.5 s 동안 0.5° 미만 움직임)', fontsize=7.5, color=INK2,
            va='bottom')
    ax.set_ylim(-105., 105.)
    ax.set_ylabel('조향 출력 [%]')
    ax = axes[2]
    ax.plot(t[sel], run.d[sel], color=BLUE, lw=1.3, label='횡오차 d')
    ax.plot(t[sel], np.degrees(a['truth_yaw_rad'][sel]) / 10., color=ORANGE, lw=1.2, label='방향 [°] / 10')
    ax.set_ylabel('[m]')
    ax.set_xlabel('SIM 시간 [s]')
    ax.legend(loc='upper left', ncol=2)
    for ax in axes:
        if onset is not None:
            ax.axvspan(onset, onset + max(f['params'].get('duration_s', 0.) for f in run.cond['faults']),
                       color=GRID, alpha=.6, lw=0)
        for code, at in faults:
            ax.axvline(at, color=RED, lw=1.1)
    if faults:
        axes[0].text(faults[0][1], axes[0].get_ylim()[1], ' MCU %s' % faults[0][0], color=INK, fontsize=8, va='top')
    axes[0].set_title('실패 사례 %s: 끝난 이유 %s (회색 띠: 조향축 외란)' % (name, m.get('end_reason')), loc='left')
    fig.tight_layout()
    return _save(plt, fig, out_dir, 'fig05_failure')


def gif(report, folder, run_name, path, width_px=640):
    plt = _plt()
    from matplotlib import animation
    row = next(r for r in report['rows'] if r['name'] == run_name)
    run = Run(os.path.join(folder, 'runs', run_name), row)
    geo = run.geo
    posts = np.asarray(geo.posts(), dtype=float)
    dpi = 80
    fig, ax = plt.subplots(figsize=(width_px / dpi, width_px / dpi * .62), dpi=dpi)
    for px, py, pr in posts:
        ax.add_patch(plt.Circle((px, py), pr, color=MUTED, lw=0))
    s = np.linspace(geo.start_s_m, geo.end_s_m + 3., 200)
    ax.plot([geo.centre(v)[0] for v in s], [geo.centre(v)[1] for v in s], color=INK2, lw=.8, ls='--')
    ax.set_aspect('equal', adjustable='datalim')
    pad = 2.
    ax.set_xlim(min(posts[:, 0]) - pad, max(posts[:, 0]) + pad)
    ax.set_ylim(min(posts[:, 1]) - pad, max(posts[:, 1]) + pad)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    path_line, = ax.plot([], [], color=BLUE, lw=1.6)
    body, = ax.plot([], [], color=ORANGE, lw=1.6)
    label = ax.text(.01, .97, '', transform=ax.transAxes, fontsize=8, color=INK, va='top')
    ax.set_title('열 하나 추종: %s seed %d (자리값 기반 SIM 결과)' % (row['condition'], row['seed']), loc='left',
                 fontsize=9)
    frames = list(range(0, len(run.t), 100)) + [len(run.t) - 1] * 4

    def draw(k):
        path_line.set_data(run.x[:k + 1], run.y[:k + 1])
        c, sn = math.cos(run.yaw[k]), math.sin(run.yaw[k])
        pts = [(FW.FRONT_BUMPER_M, FW.HALF_WIDTH_M), (FW.FRONT_BUMPER_M, -FW.HALF_WIDTH_M),
               (-FW.FRONT_BUMPER_M, -FW.HALF_WIDTH_M), (-FW.FRONT_BUMPER_M, FW.HALF_WIDTH_M),
               (FW.FRONT_BUMPER_M, FW.HALF_WIDTH_M)]
        body.set_data([run.x[k] + c * a - sn * b for a, b in pts], [run.y[k] + sn * a + c * b for a, b in pts])
        label.set_text('t %.0f s  횡오차 %+.2f m  속도 %.2f m/s' % (run.t[k], run.d[k], run.a['truth_v_m_s'][k]))
        return path_line, body, label

    anim = animation.FuncAnimation(fig, draw, frames=frames, blit=False)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    anim.save(path, writer=animation.PillowWriter(fps=8))
    plt.close(fig)
    return path


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--runs', required=True)
    p.add_argument('--out', default=DEFAULT_OUT)
    p.add_argument('--failure')
    p.add_argument('--gif')
    p.add_argument('--gif-run')
    args = p.parse_args(argv)
    report = load(args.runs)
    plt = _plt()
    made = [fig_top_views(plt, report, args.runs, args.out), fig_lateral(plt, report, args.out),
            fig_matrix(plt, report, args.out), fig_row_end(plt, report, args.out)]
    if args.failure:
        made.append(fig_failure(plt, report, args.runs, args.out, args.failure))
    if args.gif:
        made.append(gif(report, args.runs, args.gif_run, args.gif))
    for path in made:
        print('%s %d bytes' % (path, os.path.getsize(path)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
