"""Figures (PNG + SVG). Every figure says in its footer that the numbers are from the car before the steering
mechanism was fitted. Positions are never plotted."""
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from matplotlib.ticker import FuncFormatter  # noqa: E402
import numpy as np  # noqa: E402

import speed as sp  # noqa: E402
import startstop as ss  # noqa: E402

FOOT = '조향 장치 장착 전 차(좌우 차동 구동)의 2026-10-03·10-10 수동 주행 기록. 조향 차량의 값이 아님.'
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK2 = '#52514e'
MUTED = '#8a8984'
GRID = '#e4e3df'
BAND = '#d9d8d4'
C_ROAD, C_ORCH3, C_ORCH10 = '#2a78d6', '#eb6834', '#1baf7a'
BLUES = ['#86b6ef', '#3987e5', '#1c5cab', '#0d366b']
GROUP_STYLE = {
    sp.GROUP_ROAD: dict(color=C_ROAD, marker='o', fill=True, label='도로·포장 (10/3)'),
    sp.GROUP_ORCH3: dict(color=C_ORCH3, marker='o', fill=True, label='과수원 (10/3, 열 구분 없음)'),
    sp.GROUP_IN: dict(color=C_ORCH10, marker='o', fill=True, label='과수원 열 안 (10/10)'),
    sp.GROUP_OUT: dict(color=C_ORCH10, marker='o', fill=False, label='과수원 그 밖 (10/10)'),
    sp.GROUP_UNK: dict(color=MUTED, marker='o', fill=False, label='과수원 위치 모름 (10/10)'),
}


def _style():
    plt.rcParams.update({
        'font.family': ['Malgun Gothic', 'NanumBarunpen', 'DejaVu Sans'], 'axes.unicode_minus': False,
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'axes.edgecolor': MUTED, 'axes.labelcolor': INK, 'xtick.color': INK2, 'ytick.color': INK2,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': .8, 'axes.spines.top': False,
        'axes.spines.right': False, 'font.size': 9.5, 'axes.titlesize': 10.5, 'axes.titleweight': 'bold',
        'legend.frameon': False, 'legend.fontsize': 8.5, 'svg.fonttype': 'none', 'svg.hashsalt': 'drive_response'})


def _save(fig, out_dir, name):
    fig.text(.01, -.01, FOOT, fontsize=7.5, color=INK2, ha='left', va='top')
    d = os.path.join(out_dir, 'figures')
    os.makedirs(d, exist_ok=True)
    for ext in ('png', 'svg'):
        path = os.path.join(d, '%s.%s' % (name, ext))
        fig.savefig(path, dpi=150 if ext == 'png' else None, bbox_inches='tight', metadata=_meta(ext))
        _lf(path) if ext == 'svg' else None
    plt.close(fig)


def _meta(ext):
    if ext == 'png':
        return {'Software': None}
    return {'Creator': None, 'Date': None}


def _lf(path):
    with open(path, 'rb') as fh:
        data = fh.read()
    with open(path, 'wb') as fh:
        fh.write(data.replace(b'\r\n', b'\n'))


def _bands(ax, missing, label=True):
    for i, (lo, hi) in enumerate(missing):
        ax.axvspan(max(lo - .5, 0), min(hi + .5, 100), color=BAND, alpha=.55, lw=0, zorder=0, label='정착된 직진 구간 없음(외삽)' if (label and i == 0) else None)


def _scatter(ax, x, y, st, size=26, alpha=.85, marker=None, z=3):
    m = marker or st['marker']
    if st['fill']:
        ax.scatter(x, y, s=size, marker=m, color=st['color'], edgecolor=SURFACE, linewidth=.6, alpha=alpha, zorder=z)
    else:
        ax.scatter(x, y, s=size, marker=m, facecolor='none', edgecolor=st['color'], linewidth=1.1, alpha=alpha, zorder=z)


def _jitter(n, seed, width=.9):
    rng = np.random.default_rng(seed)
    return rng.uniform(-width, width, n)


# --------------------------------------------------------------------------------------------------
def fig01(R, out_dir):
    fig, ax = plt.subplots(figsize=(8.6, 5.4))
    _bands(ax, R['missing_ranges'])
    segs = R['main_segments']
    for k, g in enumerate(sp.GROUPS):
        gs = [s for s in segs if s.get('group') == g]
        li = [s for s in gs if s.get('lidar_v') is not None]
        gn = [s for s in gs if s.get('lidar_v') is None and s.get('gnss_v') is not None]
        st = GROUP_STYLE[g]
        if li:
            _scatter(ax, np.array([s['duty'] for s in li]) + _jitter(len(li), k), [s['lidar_v'] for s in li], st, size=22,
                     alpha=.7)
        if gn:
            _scatter(ax, np.array([s['duty'] for s in gn]) + _jitter(len(gn), 10 + k), [s['gnss_v'] for s in gn], st,
                     size=34, alpha=.9, marker='^')
    rows = R['speed_main']['rows']
    meas = [r for r in rows if r['grade'] in ('측정', '참고')]
    ax.plot([r['duty'] for r in rows], [r['v'] for r in rows], color=INK, lw=1.2, ls=(0, (4, 3)), zorder=4)
    ax.plot([r['duty'] for r in meas], [r['v'] for r in meas], ls='none', marker='D', ms=6.5, color=INK, zorder=5)
    for r in meas:
        ax.annotate('%.2f' % r['v'], (r['duty'], r['v']), xytext=(4, 7), textcoords='offset points', fontsize=7.5,
                    color=INK2)
    r90 = next((r for r in meas if r['duty'] == 90), None)
    if r90 is not None:
        ax.annotate('90 %: 구간 대부분이 10/10 과수원\n기록 하나(035952) → 88 %보다 낮음', (90, r90['v']), xytext=(-150, -60),
                    textcoords='offset points', fontsize=7.5, color=INK2,
                    arrowprops=dict(arrowstyle='-', color=MUTED, lw=.8))
    fit = R['speed_main']['fit']
    ax.set_xlim(0, 103)
    ax.set_ylim(0, 1.2)
    ax.set_xlabel('요청 duty (좌우 같음, %)')
    ax.set_ylabel('정상 직진 속도 (m/s)')
    ax.set_title('그림 1. duty → 정상 직진 속도: 구간 점과 기준표')
    handles = [Patch(color=BAND, alpha=.55, label='정착된 직진 구간 없음 → 표에서 외삽·보간')]
    for g in sp.GROUPS:
        if any(s.get('group') == g for s in segs):
            st = GROUP_STYLE[g]
            handles.append(Line2D([], [], ls='none', marker='o', ms=6, color=st['color'],
                                  markerfacecolor=st['color'] if st['fill'] else 'none', label=st['label']))
    handles += [Line2D([], [], ls='none', marker='^', ms=6, color=INK2, markerfacecolor='none',
                       label='삼각형 = LiDAR 값 없어 GNSS'),
                Line2D([], [], ls=(0, (4, 3)), marker='D', ms=6, color=INK,
                       label='기준표(표 2): ◆ 측정·참고, 점선 = 보간·외삽')]
    ax.legend(handles=handles, loc='upper left', ncol=1)
    ax.text(.99, .02, '점 1개 = 2 s 넘게 정착한 직진 구간 하나의 중앙값(첫 1 s 제외)\n외삽 직선 v = %.4f·duty %+.3f' %
            (fit['b'], fit['a']), transform=ax.transAxes, ha='right', va='bottom', fontsize=7.5, color=INK2)
    _save(fig, out_dir, 'fig01_duty_speed')


def fig02(R, out_dir):
    groups = [g for g in sp.GROUPS if R['speed_by_group'].get(g)]
    fig, axes = plt.subplots(1, len(groups), figsize=(3.6 * len(groups), 5.0), sharey=True)
    segs = R['main_segments']
    for ax, g in zip(np.atleast_1d(axes), groups):
        _bands(ax, R['missing_ranges'], label=False)
        st = GROUP_STYLE[g]
        gs = [s for s in segs if s.get('group') == g]
        for key, mk, off in (('lidar_v', 'o', -1.2), ('gnss_v', '^', 0.), ('carto_v', 's', 1.2)):
            pts = [s for s in gs if s.get(key) is not None]
            if pts:
                ax.scatter(np.array([s['duty'] for s in pts]) + off, [s[key] for s in pts], s=12, marker=mk,
                           facecolor='none', edgecolor=st['color'] if key == 'lidar_v' else MUTED, linewidth=.8,
                           alpha=.6, zorder=3)
        cells = R['speed_by_group'][g]
        for d, c in cells.items():
            if not c['n']:
                continue
            ax.errorbar(d, c['v'], yerr=[[c['v'] - c['p10']], [c['p90'] - c['v']]], fmt='D', ms=6,
                        color=st['color'], mfc=st['color'] if st['fill'] else SURFACE, capsize=2.5, lw=1.2, zorder=5)
            ax.text(d, .03, '%.2f %s n%d' % (c['v'], c['grade'] + ('' if c['ref'] == 'LiDAR' else '·' + c['ref']), c['n']),
                    rotation=90, ha='center', va='bottom', fontsize=7, color=INK, zorder=6,
                    bbox=dict(boxstyle='square,pad=0.1', fc=SURFACE, ec='none', alpha=.8))
        ax.set_title(st['label'], fontsize=9.5)
        ax.set_xlim(30, 103)
        ax.set_ylim(0, 1.2)
        ax.set_xlabel('duty (%)')
    np.atleast_1d(axes)[0].set_ylabel('정상 직진 속도 (m/s)')
    handles = [Line2D([], [], ls='none', marker='D', color=INK2, label='칸 값(구간 중앙값들의 중앙값), 막대 p10–p90; 아래 세로 글씨 = 값·등급·구간 수'),
               Line2D([], [], ls='none', marker='o', mfc='none', color=INK2, label='LiDAR 구간'),
               Line2D([], [], ls='none', marker='^', mfc='none', color=MUTED, label='GNSS 구간'),
               Line2D([], [], ls='none', marker='s', mfc='none', color=MUTED, label='Cartographer 구간'),
               Patch(color=BAND, alpha=.55, label='정착된 직진 구간 없음(어느 기록에도)')]
    fig.legend(handles=handles, loc='lower center', ncol=3, bbox_to_anchor=(.5, .03))
    fig.suptitle('그림 2. 지면 묶음별 duty → 정상 직진 속도', fontweight='bold', y=.99)
    fig.subplots_adjust(bottom=.25, wspace=.08)
    _save(fig, out_dir, 'fig02_duty_speed_by_ground')


def fig03(R, out_dir):
    recs = sorted([r for r in R['recordings'] if r['analysed']], key=lambda r: r['kst_start'] or '')
    duties = sorted({d for r in recs for d, s in r['settled_straight_s'].items() if s >= .9}, reverse=True)
    mat = np.zeros((len(duties), len(recs)))
    for j, r in enumerate(recs):
        for i, d in enumerate(duties):
            mat[i, j] = r['settled_straight_s'].get(d, 0.)
    fig, ax = plt.subplots(figsize=(9.6, 5.6))
    from matplotlib.colors import LinearSegmentedColormap, LogNorm
    cmap = LinearSegmentedColormap.from_list('blue', ['#cde2fb', '#86b6ef', '#3987e5', '#1c5cab', '#0d366b'])
    m = np.ma.masked_where(mat < .9, mat)
    im = ax.imshow(m, aspect='auto', cmap=cmap, norm=LogNorm(vmin=1, vmax=max(600, mat.max())))
    for i in range(len(duties)):
        for j in range(len(recs)):
            if mat[i, j] >= .9:
                ax.text(j, i, '%.0f' % mat[i, j], ha='center', va='center', fontsize=7,
                        color='white' if mat[i, j] > 60 else INK)
    ax.set_yticks(range(len(duties)))
    ax.set_yticklabels(['%d %%' % d if d > 0 else '%d %% (후진)' % d for d in duties])
    labels = []
    for r in recs:
        g = {'road': '도로', 'orchard': '과수원'}.get(r['ground'], r['ground'])
        labels.append('%s\n%s\n%s' % (r['short'], r['kst_start'][11:16] if r['kst_start'] else '', g))
    ax.set_xticks(range(len(recs)))
    ax.set_xticklabels(labels, fontsize=7.5)
    ax.grid(False)
    split = sum(1 for r in recs if r['day'] == '1003')
    ax.axvline(split - .5, color=INK2, lw=1)
    ax.set_xlabel('기록 (KST 시작 시각, 지면)    왼쪽: 2026-10-03    오른쪽: 2026-10-10')
    cb = fig.colorbar(im, ax=ax, pad=.01)
    cb.set_label('정착된 같은-duty 직진 시간 (s, 로그 눈금)')
    ax.set_ylabel('요청 duty')
    gaps = ', '.join(('%d~%d %%' % (a, b)) if b > a else '%d %%' % a for a, b in R['missing_ranges'] if b - a >= 2)
    ax.set_title('그림 3. duty × 기록: 정착된 같은-duty 직진 시간(초, 빈 칸 = 1 s 미만), 실제 시각 순', loc='left', pad=16)
    ax.text(0, 1.008, '표에 쓸 구간(2 s 넘게 정착)이 하나도 없는 duty: ' + gaps, transform=ax.transAxes, fontsize=8,
            color=INK2, va='bottom')
    _save(fig, out_dir, 'fig03_duty_seconds_by_record')


def fig04(R, out_dir):
    fig, ax = plt.subplots(figsize=(8.4, 5.2))
    evs = [e for e in R['starts'] if e['cls'] == 'straight_fwd' and e.get('curve')]
    col = {'road': C_ROAD, 'orchard': C_ORCH3}
    for e in evs:
        c = np.array(e['curve'])
        if len(c) < 3:
            continue
        color = C_ORCH10 if e['day'] == '1010' else col.get(e['ground'], MUTED)
        ls = '-' if e['duty'] == 100 else ((0, (2, 1.5)) if e['duty'] >= 88 else (0, (1, 1.2)))
        low = e.get('good_frac', 1.) < ss.MIN_GOOD_FRAC
        ax.plot(c[:, 0], c[:, 1], color=color, lw=1. if low else 1.4, ls=ls, alpha=.45 if low else .85, zorder=3)
        if c[-1, 0] < 3.:
            continue
        ax.annotate('%s %d%%%s' % (e['short'], e['duty'], ' (정합 품질 낮음, 참고)' if low else ''), (c[-1, 0], c[-1, 1]),
                    xytext=(3, 0), textcoords='offset points', fontsize=6.5, color=INK2, va='center')
    rows = {r['duty']: r for r in R['speed_main']['rows'] if r.get('v') is not None}
    tab = sorted((d, r['v']) for d, r in rows.items())
    xs = np.array([d for d, _ in tab], float)
    ys = np.array([v for _, v in tab], float)
    vof = lambda d: float(ys[0] * d / xs[0]) if d <= xs[0] else float(np.interp(d, xs, ys))  # noqa: E731
    br = R['distance']['breakaway_duty']
    T = np.arange(0, 10.01, .1)
    for duty, c in ((100, INK), (50, INK2)):
        y = [ss.model_distance(duty, t, vof, br) if t > 0 else 0. for t in T]
        ax.plot(T, y, color=c, lw=1.2, ls=(0, (4, 3)), zorder=2)
        ax.annotate('모델 %d %%' % duty, (T[-1], y[-1]), xytext=(3, 0), textcoords='offset points', fontsize=7.5, color=c,
                    va='center')
    ax.set_xlim(0, 11.2)
    ax.set_ylim(-.2, 9.5)
    ax.set_xlabel('램프 시작부터 시간 (s)')
    ax.set_ylabel('주행축 방향 이동거리 (m)')
    ax.set_title('그림 4. 정지에서 출발: 시간 대 이동거리(측정 곡선)와 모델')
    handles = [Line2D([], [], color=C_ORCH10, lw=1.4, label='10/10 과수원'),
               Line2D([], [], color=C_ROAD, lw=1.4, label='10/3 도로·포장'),
               Line2D([], [], color=C_ORCH3, lw=1.4, label='10/3 과수원'),
               Line2D([], [], color=INK2, lw=1.4, label='실선 100 %, 짧은 점선 88~95 %, 촘촘한 점선 45 %'),
               Line2D([], [], color=INK, lw=1.2, ls=(0, (4, 3)),
                      label='모델: 램프 100 %%/s, 문턱 %.0f %%, 지연 0.25 s(가정), 정상 속도 = 표 2' % br)]
    ax.legend(handles=handles, loc='upper left')
    ax.text(.99, .02, '곡선은 요청이 바뀌거나 LiDAR 자세가 끊긴 곳에서 끝남(3 s 넘는 곡선만 이름 표시).\n'
            '30~60 % 정지 출발은 10/3 080909의 45 % 한 건뿐. 50 % 곡선은 모델.',
            transform=ax.transAxes, ha='right', va='bottom', fontsize=7.5, color=INK2)
    _save(fig, out_dir, 'fig04_start_distance_time')


def fig05(R, out_dir):
    evs = [e for e in R['starts'] if e['cls'] == 'straight_fwd' and e['precise'] and e.get('onset_s') is not None]
    fig, axes = plt.subplots(1, 2, figsize=(9.4, 4.4))
    for ax, key, lab, lim in ((axes[0], 'duty_at_onset', '움직이기 시작할 때 duty (%)', (0, 105)),
                              (axes[1], 'onset_s', '램프 시작 → 움직임 (s)', (0, 4.5))):
        for row, (day, color, name) in enumerate((('1010', C_ORCH10, '10/10 과수원'), ('1003', C_ROAD, '10/3 (도로·과수원)'))):
            v = np.array([e[key] for e in evs if e['day'] == day])
            y = row + _jitter(len(v), row + (5 if key == 'onset_s' else 0), .12)
            ax.scatter(v, y, s=34, color=color, edgecolor=SURFACE, linewidth=.6, zorder=3)
            if len(v):
                med = float(np.median(v))
                ax.plot([med, med], [row - .3, row + .3], color=INK, lw=2, zorder=4)
                ax.annotate('중앙 %.2g (n %d)' % (med, len(v)) if key == 'onset_s' else '중앙 %.0f %% (n %d)' % (med, len(v)),
                            (med, row + .32), ha='center', fontsize=7.5, color=INK2)
        ax.set_yticks([0, 1])
        ax.set_yticklabels(['10/10 과수원', '10/3'])
        ax.set_ylim(-.6, 1.7)
        ax.set_xlim(*lim)
        ax.set_xlabel(lab)
    axes[0].set_title('출발 문턱 duty (요청 88~100 %에서 램프 도중)')
    axes[1].set_title('램프 시작부터 움직임까지')
    fig.suptitle('그림 5. 정지에서 출발: 문턱 duty와 시간 (±2 스캔 창이라 최대 약 0.2 s 이르게 잡힘)', fontweight='bold')
    fig.subplots_adjust(bottom=.18, wspace=.3)
    _save(fig, out_dir, 'fig05_start_onset')


def fig06(R, out_dir):
    fig, ax = plt.subplots(figsize=(7.6, 5.0))
    ax.axvspan(0, .7, color=BAND, alpha=.55, lw=0, zorder=0)
    v = np.linspace(0, 1.05, 100)
    ax.plot(v, [ss.release_model(x) for x in v], color=INK, lw=1.2, ls=(0, (4, 3)), zorder=2)
    for e in R['stops']:
        if not e.get('clean') or e.get('dist_m') is None:
            continue
        color = C_ORCH10 if e['day'] == '1010' else (C_ROAD if e['ground'] == 'road' else C_ORCH3)
        ok = e.get('speed_consistent') is True
        ax.scatter(e['speed_at_release'], e['dist_m'], s=46, color=color if ok else 'none', edgecolor=color,
                   linewidth=1.2, zorder=3)
        ax.annotate('%s %d%%' % (e['short'], e['prev_duty']), (e['speed_at_release'], e['dist_m']), xytext=(5, -2),
                    textcoords='offset points', fontsize=6.5, color=INK2)
    ax.set_xlim(0, 1.05)
    ax.set_ylim(0, .8)
    ax.set_xlabel('놓을 때 속도 (m/s, LiDAR)')
    ax.set_ylabel('놓기부터 멈춤까지 거리 (m)')
    ax.set_title('그림 6. 키 놓기 정지: 직전 속도 대 정지 거리')
    handles = [Line2D([], [], ls='none', marker='o', color=C_ORCH10, label='10/10 과수원'),
               Line2D([], [], ls='none', marker='o', color=C_ROAD, label='10/3 도로·포장'),
               Line2D([], [], ls='none', marker='o', color=C_ORCH3, label='10/3 과수원'),
               Line2D([], [], ls='none', marker='o', mfc='none', color=INK2,
                      label='속 빈 점: 놓을 때 속도와 직전 1 s 중앙값이 0.15 m/s 넘게 다름'),
               Line2D([], [], color=INK, ls=(0, (4, 3)), label='모델 v·0.25 s + v²/(2·1.3 m/s²)'),
               Patch(color=BAND, alpha=.55, label='0.7 m/s 아래: 측정 사건 없음(모델뿐)')]
    ax.legend(handles=handles, loc='upper left')
    ax.text(.99, .02, 'STOP(즉시 0) 사건은 감속 시작 시각을 알 수 없어 빠짐.', transform=ax.transAxes, ha='right',
            va='bottom', fontsize=7.5, color=INK2)
    _save(fig, out_dir, 'fig06_stop_distance')


def fig07(R, out_dir):
    S = R['session']
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.8), sharey=True)
    for ax, day, title in ((axes[0], '1003', '10/3 (도로 4개 + 과수원 1개)'), (axes[1], '1010', '10/10 (과수원 5개)')):
        segs = [s for s in S['segments'] if s['day'] == day and s['lidar_v'] is not None]
        for g in sp.GROUPS:
            gs = [s for s in segs if s['group'] == g]
            if gs:
                _scatter(ax, [s['hours'] for s in gs], [s['lidar_v'] for s in gs], GROUP_STYLE[g], size=14, alpha=.55)
        for r in [x for x in S['records'] if x['day'] == day]:
            if r['LiDAR']['v'] is not None:
                ax.plot(r['hours_mid'], r['LiDAR']['v'], marker='D', ms=8, color=INK, zorder=5, ls='none')
            if r['GNSS']['v'] is not None:
                ax.plot(r['hours_mid'], r['GNSS']['v'], marker='^', ms=8, mfc=SURFACE, color=INK, zorder=5, ls='none')
            top = max([v for v in (r['LiDAR']['v'], r['GNSS']['v']) if v is not None])
            ax.annotate(r['short'], (r['hours_mid'], top), xytext=(0, 9), textcoords='offset points', ha='center',
                        fontsize=7.5, color=INK2)
        ax.set_title(title, fontsize=9.5)
        ax.set_xlabel('실제 시각 (KST)')
        ax.xaxis.set_major_formatter(FuncFormatter(lambda h, _p: '%d:%02d' % (int(h), int(round((h % 1) * 60)) % 60)))
        ax.set_ylim(.3, 1.2)
    axes[0].set_ylabel('100 % 직진 구간 속도 (m/s)')
    handles = []
    for g in sp.GROUPS:
        if any(s['group'] == g for s in S['segments']):
            st = GROUP_STYLE[g]
            handles.append(Line2D([], [], ls='none', marker='o', color=st['color'],
                                  markerfacecolor=st['color'] if st['fill'] else 'none', label=st['label'] + ' 구간(LiDAR)'))
    handles += [Line2D([], [], ls='none', marker='D', color=INK, label='기록 중앙값 LiDAR'),
                Line2D([], [], ls='none', marker='^', mfc=SURFACE, color=INK, label='기록 중앙값 GNSS')]
    fig.legend(handles=handles, loc='lower center', ncol=4, bbox_to_anchor=(.5, .02))
    fig.suptitle('그림 7. 하루 안에서 100 % 직진 속도의 변화 — 배터리 효과로 단정하지 않음(전압 기록 없음, 지면 섞임)',
                 fontweight='bold')
    fig.subplots_adjust(bottom=.27, wspace=.06)
    _save(fig, out_dir, 'fig07_session_time')


def fig08(R, out_dir):
    segs = R['main_segments']
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 4.9))
    for ax, key, name in ((axes[0], 'gnss_v', 'GNSS 속도'), (axes[1], 'carto_v', 'Cartographer 경로 속도')):
        ax.plot([0, 1.2], [0, 1.2], color=INK2, lw=1, zorder=1)
        ax.plot([0, 1.2], [0, 1.2 * .9], color=MUTED, lw=.8, ls=(0, (3, 3)), zorder=1)
        ax.plot([0, 1.2], [0, 1.2 * 1.1], color=MUTED, lw=.8, ls=(0, (3, 3)), zorder=1)
        ratios = []
        for g in sp.GROUPS:
            gs = [s for s in segs if s.get('group') == g and s.get('lidar_v') and s.get(key)]
            if gs:
                _scatter(ax, [s['lidar_v'] for s in gs], [s[key] for s in gs], GROUP_STYLE[g], size=16, alpha=.65)
                ratios += [s[key] / s['lidar_v'] for s in gs]
        if ratios:
            ax.text(.03, .97, '%s / LiDAR 중앙값 %.3f (구간 %d개)\np10–p90 %.2f–%.2f' % (
                name, np.median(ratios), len(ratios), np.percentile(ratios, 10), np.percentile(ratios, 90)),
                transform=ax.transAxes, va='top', fontsize=8, color=INK)
        ax.set_xlim(.2, 1.15)
        ax.set_ylim(.2, 1.15)
        ax.set_aspect('equal')
        ax.set_xlabel('LiDAR 정합 속도 (m/s)')
        ax.set_ylabel(name + ' (m/s)')
    axes[0].set_title('GNSS 대 LiDAR (같은 구간)')
    axes[1].set_title('Cartographer 대 LiDAR (같은 구간)')
    handles = []
    for g in sp.GROUPS:
        if any(s.get('group') == g for s in segs):
            st = GROUP_STYLE[g]
            handles.append(Line2D([], [], ls='none', marker='o', color=st['color'],
                                  markerfacecolor=st['color'] if st['fill'] else 'none', label=st['label']))
    handles += [Line2D([], [], color=INK2, lw=1, label='같음'), Line2D([], [], color=MUTED, ls=(0, (3, 3)), label='±10 %')]
    fig.legend(handles=handles, loc='lower center', ncol=6, bbox_to_anchor=(.5, .02))
    fig.suptitle('그림 8. 속도 참조 비교 — 어느 쪽도 정답이 아님(줄자 대조 없음)', fontweight='bold')
    fig.subplots_adjust(bottom=.22, wspace=.3)
    _save(fig, out_dir, 'fig08_reference_compare')


def fig10(R, out_dir):
    D = R['distance']
    fig, ax = plt.subplots(figsize=(8.4, 5.2))
    _bands(ax, R['missing_ranges'])
    duties = [r['duty'] for r in D['rows']]
    for T, c in zip(ss.HOLD_TIMES, BLUES):
        key = 'd_%g' % T
        y = [r[key]['model'] if r[key]['model'] is not None else np.nan for r in D['rows']]
        ax.plot(duties, y, color=c, lw=1.6, ls=(0, (4, 3)), marker='o', ms=3.5, zorder=3)
        ax.annotate('%g s' % T, (duties[-1], y[-1]), xytext=(5, 0), textcoords='offset points', color=c, fontsize=8.5,
                    va='center', fontweight='bold')
        for r in D['rows']:
            m = r[key]
            if m['measured'] is not None:
                lo, hi = m['measured_range']
                ax.errorbar(r['duty'], m['measured'], yerr=[[m['measured'] - lo], [hi - m['measured']]], fmt='D', ms=7,
                            color=c, mec=INK, capsize=3, zorder=5)
                ax.annotate('n%d' % m['n'], (r['duty'], m['measured']), xytext=(-14, 4), textcoords='offset points',
                            fontsize=7, color=INK2)
    ax.axvline(D['breakaway_duty'], color=INK2, lw=1, ls=(0, (1, 2)))
    ax.text(D['breakaway_duty'] - .8, 4.7, '출발 문턱 %.0f %% (10/10 정지 출발 %d건 중앙값)' % (
        D['breakaway_duty'], D['breakaway_n']), rotation=90, ha='right', va='center', fontsize=7.5, color=INK2)
    ax.set_xlim(25, 108)
    ax.set_ylim(0, 9.5)
    ax.set_xlabel('요청 duty (정지에서 출발, %)')
    ax.set_ylabel('이동거리 (m)')
    ax.set_title('그림 10. duty와 유지 시간에 따른 이동거리')
    handles = [Line2D([], [], color=INK2, ls=(0, (4, 3)), marker='o', ms=3.5, label='모델(표 6의 가정, 측정 아님)'),
               Line2D([], [], ls='none', marker='D', color=INK2, mec=INK, label='측정: 정지 출발 사건 중앙값, 막대 = 범위'),
               Patch(color=BAND, alpha=.55, label='정착된 직진 구간 없음')]
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(.13, 1.))
    ax.text(.55, .2, '모델 속도는 표 2(지면 합침)를 따른다: 90 %가\n88 %보다 느린 것은 표 2의 90 % 칸 때문(본문 참고).',
            transform=ax.transAxes, ha='left', va='bottom', fontsize=7.5, color=INK2)
    _save(fig, out_dir, 'fig10_duty_hold_distance')


def write_all(R, out_dir):
    _style()
    for f in (fig01, fig02, fig03, fig04, fig05, fig06, fig07, fig08, fig10):
        f(R, out_dir)
