#!/usr/bin/env python3
"""Figures of the steering extension model (ST-10, ST-11): drawn from new T1 rig runs and the profile loader.

  st11_steer_hold      steering deviation (true equivalent angle minus the pre-onset base) in four disturbance cases
                       of SIM_ACCEPTANCE amendment 41 (D1, D4, R1, D5; judgement seed 1000, the same episodes as the
                       judgement run: the rig is deterministic), the disturbance window shaded
  st11_lateral         the driven path (lateral position against the distance) of D1 and D4, seed 1000
  st10_belt_window     the belt ratios the v0.2 profile loader accepts against the mechanical steering limit (the pot
                       window and the 50-count calibration rule), the placeholder point marked

  python -B tools/steer_sim/steer_model_figures.py [--out docs/steering/figures/steering_model] [--build-dir <dir>]
Every figure says in its footer that the numbers are placeholder-based SIM results.
"""
import argparse
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'sim', 'ubts'))
sys.dont_write_bytecode = True
from steering import steer_hold_checks as SHC   # noqa: E402
from steering import vehicle_profile as VP       # noqa: E402

FOOT = '자리값 기반 SIM 결과(T1 장치: 펌웨어 V3 + 호스트 브리지 + L1 구동기, 프로필 v0.2). 실차 성능이 아님.'
SURFACE, INK, INK2, MUTED, GRID, BAND = '#fcfcfb', '#0b0b0b', '#52514e', '#8a8984', '#e4e3df', '#ecebe7'
SERIES = ('#2a78d6', '#eb6834', '#1baf7a', '#8a56c2')
LABELS = dict(D1='+2.5 N·m 계단 3 s', D4='+2.0 N·m 이어지는 힘 10 s', R1='+3.5 N·m 계단 3 s(경계, 보고)',
              D5='+5.0 N·m 계단 5 s(크기 밖)')


def _style(plt):
    for font in ('Malgun Gothic', 'NanumGothic', 'Noto Sans CJK KR'):
        try:
            from matplotlib import font_manager
            font_manager.findfont(font, fallback_to_default=False)
            plt.rcParams['font.family'] = font
            break
        except Exception:
            continue
    plt.rcParams.update({'axes.facecolor': SURFACE, 'figure.facecolor': SURFACE, 'axes.edgecolor': MUTED,
                         'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2, 'axes.grid': True,
                         'grid.color': GRID, 'grid.linewidth': .8, 'axes.spines.top': False,
                         'axes.spines.right': False, 'axes.unicode_minus': False, 'font.size': 9})


def _foot(fig):
    fig.text(.01, .01, FOOT, fontsize=7.5, color=MUTED, ha='left', va='bottom')


def steer_hold(plt, eps, out):
    fig, axes = plt.subplots(2, 2, figsize=(9.6, 5.8))
    for ax, (name, color) in zip(axes.flat, (('D1', SERIES[0]), ('D4', SERIES[1]), ('R1', SERIES[3]),
                                             ('D5', SERIES[2]))):
        ep = eps[name]
        case = SHC.cases_of('disturbance', VP.load_profile(SHC.PROFILE_V02))[name]
        t = ep['trace']['t_s']
        dev = [math.degrees(d) - ep['delta_base_deg'] for d in ep['trace']['truth_delta_eq_rad']]
        ax.axvspan(case['onset_s'], case['end_s'], color=BAND, lw=0)
        ax.axhline(3., color=MUTED, lw=1, ls='--')
        ax.axhline(-3., color=MUTED, lw=1, ls='--')
        ax.plot(t, dev, color=color, lw=2)
        first = ep.get('first_steer_fault')
        status = '결함 %s (t = %.2f s)' % (first[0], first[1] / 1000.) if first else '결함 없음, 복귀 %.2f s' % ep['recovery_s']
        ax.set_title('%s  %s\n%s, 최대 %.1f°' % (name, LABELS[name], status, ep['max_steer_dev_deg']),
                     fontsize=9, color=INK, loc='left')
        ax.set_xlim(4., min(t[-1], case['end_s'] + 4.))
        ax.set_ylim(-6., 30. if first else 6.)
        ax.text(.99, 3., 'TRACK 3°(가변저항 기준)', color=MUTED, fontsize=7.5, va='bottom', ha='right',
                transform=ax.get_yaxis_transform())
    for ax in axes[1]:
        ax.set_xlabel('SIM 시간 (s)')
    for ax in axes[:, 0]:
        ax.set_ylabel('참 조향 등가각 - 기준 (°)')
    fig.suptitle('외란 아래 직진 유지(ST-11): ESP 위치 루프만, seed 1000, 회색 = 외란 구간', fontsize=11, color=INK,
                 x=.01, ha='left')
    fig.tight_layout(rect=(0, .03, 1, .97))
    _foot(fig)
    fig.savefig(os.path.join(out, 'st11_steer_hold.png'), dpi=110)
    plt.close(fig)


def lateral(plt, eps, out):
    fig, ax = plt.subplots(figsize=(8.4, 3.6))
    for name, color in (('D1', SERIES[0]), ('D4', SERIES[1])):
        ep = eps[name]
        x, y = ep['trace']['truth_x_m'], ep['trace']['truth_y_m']
        ax.plot(x, y, color=color, lw=2)
        ax.text(x[-1], y[-1], '  %s (옆 %.2f m)' % (name, ep['max_lateral_dev_m']), color=INK2, fontsize=8.5,
                va='center')
    ax.set_xlabel('앞으로 간 거리 x (m)')
    ax.set_ylabel('옆 위치 y (m)')
    ax.set_title('D1(+2.5 N·m 3 s)와 D4(+2.0 N·m 10 s)의 경로: 바깥 루프 없이 ESP 위치 루프만', fontsize=10, color=INK,
                 loc='left')
    xmax = max(max(eps[n]['trace']['truth_x_m']) for n in ('D1', 'D4'))
    ax.set_xlim(0., xmax * 1.25)
    fig.tight_layout(rect=(0, .05, 1, 1))
    _foot(fig)
    fig.savefig(os.path.join(out, 'st11_lateral.png'), dpi=110)
    plt.close(fig)


def belt_window(plt, out):
    base = VP.load_profile(SHC.PROFILE_V02)
    limits = [20. + .5 * i for i in range(31)]
    lows, highs = [], []
    for lim in limits:
        ok = []
        for k in range(50, 501, 2):        # the profile range 0.5 .. 5, step 0.02
            ratio = k / 100.
            try:
                base.with_overrides({'steer_mech_limit_deg': {key: lim for key in base.value('steer_mech_limit_deg')},
                                     'pot_to_steer_ratio': ratio}, label='window')
                ok.append(ratio)
            except VP.ProfileError:
                continue
        lows.append(min(ok) if ok else float('nan'))
        highs.append(max(ok) if ok else float('nan'))
    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    ax.fill_between(limits, lows, highs, color='#d6e6f7', lw=0)
    ax.plot(limits, lows, color=SERIES[0], lw=2)
    ax.plot(limits, highs, color=SERIES[0], lw=2)
    ax.plot([28.], [1.], 'o', color=INK, ms=7)
    ax.text(28.4, 1.05, '자리값(28°, 비 1)', fontsize=8.5, color=INK)
    ax.text(35., 4.75, '이 선 위: 끝 멈춤이 보정 창(60~963) 밖', fontsize=8, color=INK2, va='bottom', ha='right')
    ax.text(35., .3, '이 선 아래: 보정 표지가 가운데에서 50 count 미만', fontsize=8, color=INK2, va='top', ha='right')
    ax.set_xlabel('바퀴 기계 한계 (°, 네 방향 같게)')
    ax.set_ylabel('벨트 비 (가변저항 축 각 / 조향 각)')
    ax.set_ylim(0., 5.2)
    ax.set_title('프로필 v0.2 읽기가 받는 벨트 비 (270° 10비트 가변저항, 프로필 범위 0.5~5)', fontsize=10, color=INK,
                 loc='left')
    fig.tight_layout(rect=(0, .05, 1, 1))
    _foot(fig)
    fig.savefig(os.path.join(out, 'st10_belt_window.png'), dpi=110)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', default=os.path.join(REPO, 'docs', 'steering', 'figures', 'steering_model'))
    ap.add_argument('--build-dir', default=None)
    args = ap.parse_args(argv)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    _style(plt)
    os.makedirs(args.out, exist_ok=True)
    eps = {name: SHC.run_episode('disturbance', name, 1000, None, args.build_dir, with_trace=True)
           for name in ('D1', 'D4', 'R1', 'D5')}
    steer_hold(plt, eps, args.out)
    lateral(plt, eps, args.out)
    belt_window(plt, args.out)
    for name in sorted(os.listdir(args.out)):
        print(name, os.path.getsize(os.path.join(args.out, name)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
