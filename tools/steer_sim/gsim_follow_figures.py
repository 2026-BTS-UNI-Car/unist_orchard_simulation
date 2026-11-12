#!/usr/bin/env python3
"""Figures of the G-SIM row-following runs (steering.exec_checks follow), drawn from the files of real runs.

Input: run folders, each with GSIM_FOLLOW_RUN.json and GSIM_FOLLOW_TRACE.json (truth path, executor ticks, scene
geometry; written on the evaluation side). Output: PNG files (default docs/steering/figures/steer_follow_gsim/).
Every figure says in its footer that the numbers are placeholder-based SIM results.

  gsim_fig01_top_view   one run from above: the trunks and posts of the two tree rows, the centre line, the driven
                        path (truth), start, stop and the row end line; below, the lateral position enlarged
  gsim_fig02_errors     lateral and heading error of every given run against the position along the row, with the
                        registered limits

  python -B tools/steer_sim/gsim_follow_figures.py --runs <run folder> [<run folder> ...] [--top <run folder>]
                                                   [--out <folder>] [--tag TEXT]
"""
import argparse
import json
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'sim', 'ubts'))
sys.dont_write_bytecode = True
from steering import gsim_follow as GF          # noqa: E402  (the registered definitions)

FOOT = '자리값 기반 SIM 결과 (G-SIM 스택, w1_force 과수원, 실행기 컨테이너). 실차 성능이 아님.'
SURFACE, INK, INK2, MUTED, GRID, BAND = '#fcfcfb', '#0b0b0b', '#52514e', '#8a8984', '#e4e3df', '#d9d8d4'
BLUE, ORANGE, GREEN, RED, BROWN = '#2a78d6', '#eb6834', '#1baf7a', '#c93a3a', '#7a5a3a'
SERIES = ('#2a78d6', '#eb6834', '#1baf7a', '#8a56c2', '#5c9dea', '#c93a3a')
DEFAULT_OUT = os.path.join(REPO, 'docs', 'steering', 'figures', 'steer_follow_gsim')


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
    os.makedirs(out_dir, exist_ok=True)
    fig.text(.01, .005, FOOT, fontsize=7.5, color=MUTED, ha='left', va='bottom')
    path = os.path.join(out_dir, name + '.png')
    fig.savefig(path, dpi=150, bbox_inches='tight', metadata={'Software': None})
    plt.close(fig)
    return path


def load(folder):
    def read(name):
        with open(os.path.join(folder, name), 'rb') as handle:
            return json.loads(handle.read().decode('utf-8'))
    report, trace = read('GSIM_FOLLOW_RUN.json'), read('GSIM_FOLLOW_TRACE.json')
    geometry = trace['geometry']
    odom = trace['odom']
    driving = [t for t, value in trace['duty'] if value > 0.]
    rows = [(r[0],) + GF.to_row(geometry, r[1], r[2], r[3]) + (r[1], r[2], r[3], r[4]) for r in odom]
    if driving:
        drive = [r for r in rows if driving[0] <= r[0] <= driving[-1]]
    else:
        drive = []
    name = (report.get('run') or {}).get('name') or os.path.basename(os.path.normpath(folder))
    again = None                                  # a verdict judged again from the records (registration amendment)
    if os.path.exists(os.path.join(folder, 'GSIM_FOLLOW_REJUDGED.json')):
        again = read('GSIM_FOLLOW_REJUDGED.json').get('verdict')
    return dict(folder=folder, name=name, report=report, trace=trace, geometry=geometry, rows=rows, drive=drive,
                rejudged=again)


def _footprint(ax, x, y, yaw, **style):
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(GF.FRONT_BUMPER_M, GF.HALF_WIDTH_M), (GF.FRONT_BUMPER_M, -GF.HALF_WIDTH_M),
               (-GF.FRONT_BUMPER_M, -GF.HALF_WIDTH_M), (-GF.FRONT_BUMPER_M, GF.HALF_WIDTH_M)]
    xs = [x + c * a - s * b for a, b in corners]
    ys = [y + s * a + c * b for a, b in corners]
    ax.fill(xs, ys, **style)


def _label(run):
    m = run['report'].get('metrics') or {}
    verdict = run['report'].get('verdict')
    if run.get('rejudged') and run['rejudged'] != verdict:
        verdict = '기록 %s, 다시 판정 %s' % (verdict, run['rejudged'])
    return '%s (%s, %s)' % (run['name'], verdict, m.get('end_reason'))


def fig_top_view(plt, run, out_dir, tag=''):
    g = run['geometry']
    fig, (top, zoom) = plt.subplots(2, 1, figsize=(11.5, 5.4), gridspec_kw=dict(height_ratios=(1.3, 1.), hspace=.42))
    trunks = [m for m in g['markers'] if m[3] == 'trunk']
    posts = [m for m in g['markers'] if m[3] == 'post']
    top.scatter([m[0] for m in trunks], [m[1] for m in trunks], s=16, color=GREEN, zorder=3, label='줄기(장면 메타데이터)')
    top.scatter([m[0] for m in posts], [m[1] for m in posts], s=22, marker='s', color=MUTED, zorder=3, label='콘크리트 기둥')
    if g.get('rocks'):
        top.scatter([r['x'] for r in g['rocks']], [r['y'] for r in g['rocks']], s=14, marker='^', color=BROWN,
                    zorder=3, label='낮은 바위(판정 안 함)')
    xs = [r[4] for r in run['rows']]
    x_lo, x_hi = min(xs + [m[0] for m in trunks]) - 1., max(xs + [m[0] for m in posts]) + 1.
    line = lambda x: g['centre_a'] + g['centre_b'] * x                    # noqa: E731
    top.plot([x_lo, x_hi], [line(x_lo), line(x_hi)], color=MUTED, lw=1., ls='--', label='통로 중심선(평가)')
    end_x, end_y, _ = GF.from_row(g, g['row_end_s'], 0., 0.)
    c, s = math.cos(g['axis_rad']), math.sin(g['axis_rad'])
    top.plot([end_x + s * 2.4, end_x - s * 2.4], [end_y - c * 2.4, end_y + c * 2.4], color=RED, lw=1.2,
             label='열 끝(마지막 줄기 선)')
    top.plot(xs, [r[5] for r in run['rows']], color=BLUE, lw=1.6, label='차체 중심 경로(참값)', zorder=4)
    first, last = run['rows'][0], run['rows'][-1]
    _footprint(top, first[4], first[5], first[6], facecolor='none', edgecolor=GREEN, lw=1.4, zorder=5)
    _footprint(top, last[4], last[5], last[6], facecolor='none', edgecolor=RED, lw=1.4, zorder=5)
    top.set_aspect('equal')
    top.set_xlim(x_lo, x_hi)
    top.set_ylim(line(0.) - 2.9, line(0.) + 2.9)
    top.set_xlabel('세계 x [m] (열 방향)')
    top.set_ylabel('세계 y [m]')
    top.set_title('G-SIM 열 추종: 위에서 본 주행 %s%s' % (_label(run), tag))
    top.legend(loc='upper center', bbox_to_anchor=(.5, -.32), ncol=6, fontsize=7.8)
    limit = GF.CRITERIA['lateral_max_m']
    zoom.axhspan(-limit, limit, color=BAND, alpha=.45, lw=0, label='횡 최대 한도 ±%.2f m' % limit)
    zoom.axhline(0., color=MUTED, lw=1., ls='--')
    zoom.plot([r[1] for r in run['rows']], [r[2] for r in run['rows']], color=BLUE, lw=1.4)
    zoom.axvline(g['row_start_s'], color=GREEN, lw=1., ls=':', label='첫 줄기')
    zoom.axvline(g['row_end_s'], color=RED, lw=1.2, label='열 끝')
    zoom.set_xlim(x_lo, x_hi)
    zoom.set_ylim(-.7, .7)
    zoom.set_xlabel('열 축 방향 위치 s [m]')
    zoom.set_ylabel('횡 위치 [m] (왼쪽 +)')
    zoom.legend(loc='upper left', ncol=3)
    return _save(plt, fig, out_dir, 'gsim_fig01_top_view')


def fig_errors(plt, runs, out_dir, tag=''):
    c = GF.CRITERIA
    fig, (lat, head) = plt.subplots(2, 1, figsize=(10.5, 5.8), sharex=True, gridspec_kw=dict(hspace=.16))
    for i, run in enumerate(runs):
        color = SERIES[i % len(SERIES)]
        rows = run['drive'] or run['rows']
        lat.plot([r[1] for r in rows], [r[2] for r in rows], color=color, lw=1.3, label=_label(run))
        head.plot([r[1] for r in rows], [math.degrees(r[3]) for r in rows], color=color, lw=1.1)
    g = runs[0]['geometry']
    for ax, limit, unit in ((lat, c['lateral_max_m'], 'm'), (head, c['heading_max_deg'], '°')):
        ax.axhline(limit, color=RED, lw=1., ls='--')
        ax.axhline(-limit, color=RED, lw=1., ls='--', label='최대 한도 ±%g %s' % (limit, unit))
        ax.axhline(0., color=MUTED, lw=.8)
        ax.axvline(g['row_start_s'], color=GREEN, lw=1., ls=':')
        ax.axvline(g['row_end_s'], color=RED, lw=1.1)
    lat.axhline(c['lateral_p95_m'], color=ORANGE, lw=.9, ls=':', label='p95 한도 %.2f m' % c['lateral_p95_m'])
    lat.axhline(-c['lateral_p95_m'], color=ORANGE, lw=.9, ls=':')
    lat.set_ylabel('횡오차 [m] (참값)')
    head.set_ylabel('방향 오차 [°] (참값)')
    head.set_xlabel('열 축 방향 위치 s [m] (초록 점선: 첫 줄기, 빨간 선: 열 끝)')
    lat.set_title('G-SIM 열 추종: 횡오차와 방향 오차 (주행 구간)%s' % tag)
    lat.legend(loc='upper left', ncol=2)
    head.legend(loc='upper left')
    return _save(plt, fig, out_dir, 'gsim_fig02_errors')


def main(argv=None):
    p = argparse.ArgumentParser(prog='gsim_follow_figures', description=__doc__.splitlines()[0])
    p.add_argument('--runs', nargs='+', required=True)
    p.add_argument('--top', default=None, help='the run of the top view (default: the first of --runs)')
    p.add_argument('--out', default=DEFAULT_OUT)
    p.add_argument('--tag', default='', help='text added to the titles (e.g. the iteration)')
    args = p.parse_args(argv)
    plt = _plt()
    runs = [load(folder) for folder in args.runs]
    top = load(args.top) if args.top else runs[0]
    tag = (' · ' + args.tag) if args.tag else ''
    print(fig_top_view(plt, top, args.out, tag))
    print(fig_errors(plt, runs, args.out, tag))
    return 0


if __name__ == '__main__':
    sys.exit(main())
