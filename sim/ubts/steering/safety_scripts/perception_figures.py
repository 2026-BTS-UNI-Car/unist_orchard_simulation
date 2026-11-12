"""Figures of the perception layers (PC-07 local grid, PC-12 GNSS), drawn from the files of real runs.

Inputs: the output folder of tools/row_executor/local_grid_replay.py (SUMMARY.json, grid_<rec>.npz), the output folder
of perception_compare.py (PERCEPTION_COMPARE.json, runs/<name>/EXECUTOR.jsonl) and, for the replay snapshot, the 10/10
records themselves (sim_paths 'field_manual_1010', read only). Output: PNG files (default
docs/steering/figures/perception/). The SIM truth is read for evaluation only (A13 role 'evaluator'). Frames are
run-local (the estimator's odometry frame from the run's first scan; the SIM's virtual GNSS frame): no GNSS coordinate.

  fig01_grid_snapshot   10/10 replay: occupied cells, grid lines and the fast layer's band at two moments of one run
  fig02_replay_agree    10/10 replay: |difference| grid - fast layer (lateral, heading), all valid / steady ticks
  fig03_replay_cost     10/10 replay: grid update wall time per variant (PC) and the estimator's scan time
  fig04_sim_compare     fast SIM: lateral RMSE per condition (off / grid fusion / all layers) and the accuracy of the
                        fast layer and of the grid against the truth
  fig05_gnss_sim        fast SIM: GNSS layer outcomes and position disagreement in the GNSS conditions

CLI (from sim/ubts): python -B -m steering.safety_scripts.perception_figures --replay <replay folder>
                     --sim <compare folder> [--out <folder>] [--snapshot-record 20261010T045553Z_both]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

from . import follow_figures as FF

FOOT_REPLAY = '10/10 수동 주행 기록의 개방 루프 재생(사람이 몲, 조향 장착 전 차). 자율 주행 결과가 아님.'
FOOT_SIM = '자리값 기반 SIM 결과 (F-SIM T1 리그, W0 기둥 두 줄, 실행기 포함, 가상 GNSS). 실차 성능이 아님.'
DEFAULT_OUT = os.path.join(FF.REPO, 'docs', 'steering', 'figures', 'perception')
VARIANT_ORDER = ('c010_n2', 'c005_n2', 'c020_n2', 'c010_n1', 'c010_n4', 'c010_n2_free8', 'c010_n2_l0region')


def _save(plt, fig, out_dir, name, foot, foot_y=-.01):
    fig.text(.01, foot_y, foot, fontsize=7.5, color=FF.INK2, ha='left', va='top')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name + '.png')
    fig.savefig(path, dpi=110, bbox_inches='tight', metadata={'Software': None})
    plt.close(fig)
    return path


def _tools():
    tools = os.path.join(FF.REPO, 'tools')
    for path in (tools, os.path.join(tools, 'row_executor')):
        if path not in sys.path:
            sys.path.insert(0, path)


# --- fig01: replay snapshot ---------------------------------------------------------------------------------------------
def replay_snapshot(record, moments=(12., 28.), mask_dir=None):
    """Run the fast layer and the default grid over the longest forward segment of a record and keep the state at the
    given seconds after the onset: occupied cells, grid lines, the band lines, the base and its path (odometry)."""
    _tools()
    import local_grid_replay as G
    R, F = G.R, G.F
    rec = F.load_record(record, F.BASE_1010, F.DRIVE_1010)
    k0, k1 = math.ceil(rec['t_scan'][0] / G.TICK_S), math.floor(rec['t_scan'][-1] / G.TICK_S)
    h = F.human_ticks(rec, np.arange(k0, k1 + 1, dtype=np.int64) * G.TICK_S)
    seg = max((s for s in F.forward_segments(h) if s['dist_m'] >= 3.), key=lambda s: s['dist_m'])
    adapter, _ = R.make_adapter(G.C_PROPOSED, record, mask_dir or os.path.join(os.getcwd(), 'masks'))
    adapter.transient = R.fresh_transient(G.C_PROPOSED)
    order, w, _ = R.imu_yaw_rates(rec['imu'])
    stamps = R.scan_stamps(rec, G.C_PROPOSED, R.A.SCAN_LATENCY_S)
    est = R.E.SensorEstimator(R.E.EstimatorConfig())
    layer = G.make_layer('c010_n2', G.layer_config(G.VARIANTS['c010_n2']))
    layer.new_run(seg['t_on'] - G.LEAD_S, None)
    imu = rec['imu']
    t_imu = imu[order, 0] / 1e9
    w_sorted, acc_sorted = w[order], imu[order, 5:8]
    events = [(float(t), 0, j) for j, t in enumerate(t_imu) if seg['t_on'] - G.LEAD_S <= t <= seg['t_off']]
    events += [(float(rec['t_scan'][k]), 1, k) for k in range(len(rec['t_scan']))
               if seg['t_on'] - G.LEAD_S <= rec['t_scan'][k] <= seg['t_off']]
    events.sort()
    path, shots, pending = [], [], list(moments)
    for when, typ, idx in events:
        if typ == 0:
            if math.isfinite(w_sorted[idx]):
                est.on_imu(float(t_imu[idx]), float(w_sorted[idx]), tuple(acc_sorted[idx]))
            continue
        try:
            r, a0, inc, rmax, _ = adapter.convert(rec['ranges_mm'][idx].astype(np.float64) / 1000., rec['angle_min'],
                                                  rec['increment'], rec['range_max'])
        except R.X.ScanRejected:
            continue
        stamp = float(stamps[idx])
        est.on_scan(stamp, r, a0, inc, rmax)
        layer.on_scan(est, stamp, r, a0, inc)
        base = est.base_pose(stamp)
        path.append(base[:2])
        if pending and stamp - seg['t_on'] >= pending[0] and layer.grid.fit is not None:
            band = est.band
            snap = est.snapshot(stamp)
            lines = layer.grid.lines(base)
            band_lines = None
            if band is not None:
                c, s = math.cos(band.heading), math.sin(band.heading)
                along0 = c * base[0] + s * base[1]
                band_lines = {}
                for name, off in (('left', band.center + band.width / 2), ('right', band.center - band.width / 2)):
                    band_lines[name] = [[(a * c - off * s), (a * s + off * c)] for a in (along0 - 4., along0 + 10.)]
            lateral, heading = layer.grid.fit.observation(base)
            shots.append(dict(t_rel=round(stamp - seg['t_on'], 2), cells=layer.grid.occupied_points(), base=base,
                              path=np.asarray(path), lines=lines, band=band_lines,
                              grid_obs=(lateral, math.degrees(heading)),
                              l0_obs=None if not snap['corridor_valid'] else
                              (snap['lateral_offset'], math.degrees(snap['heading_error']))))
            pending.pop(0)
        if not pending:
            break
    return dict(record=record, segment_m=round(seg['dist_m'], 1), shots=shots)


def fig01(plt, data, out):
    shots = data['shots']
    fig, axes = plt.subplots(1, len(shots), figsize=(5.2 * len(shots), 5.4))
    axes = np.atleast_1d(axes)
    for ax, shot in zip(axes, shots):
        bx, by, yaw = shot['base']
        c, s = math.cos(-yaw), math.sin(-yaw)

        def local(points):
            p = np.asarray(points, dtype=float) - (bx, by)
            return np.column_stack((c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]))
        cells = local(shot['cells']) if len(shot['cells']) else np.zeros((0, 2))
        ax.scatter(cells[:, 1], cells[:, 0], s=3, c=FF.MUTED, marker='s', lw=0, label='점유 셀(여러 스캔에 남은 셀)')
        path = local(shot['path'])
        ax.plot(path[:, 1], path[:, 0], color=FF.INK2, lw=1., label='지나온 기준점 경로')
        for name in ('left', 'right'):
            seg = local(shot['lines'][name])
            ax.plot(seg[:, 1], seg[:, 0], color=FF.BLUE, lw=2.2, label='격자 열 선(L2)' if name == 'left' else None)
            if shot['band']:
                seg = local(shot['band'][name])
                ax.plot(seg[:, 1], seg[:, 0], color=FF.ORANGE, lw=1.4, ls='--',
                        label='빠른 층 band(L0)' if name == 'left' else None)
        seg = local(shot['lines']['centre'])
        ax.plot(seg[:, 1], seg[:, 0], color=FF.BLUE, lw=1., ls=':', label='격자 중심선')
        ax.plot([0], [0], marker='^', ms=9, color=FF.RED, label='차 기준점(진행 방향 위)')
        ax.set_xlim(6., -6.)
        ax.set_ylim(-6., 12.)
        ax.set_aspect('equal')
        ax.set_xlabel('왼쪽 + [m] (차 기준)')
        ax.set_ylabel('앞 [m]')
        g, l0 = shot['grid_obs'], shot['l0_obs']
        text = '격자: 횡 %+.2f m, 방향 %+.1f°' % g
        if l0:
            text += '\n빠른 층: 횡 %+.2f m, 방향 %+.1f°' % l0
        ax.set_title('출발 뒤 %.1f s' % shot['t_rel'])
        ax.text(.02, .02, text, transform=ax.transAxes, fontsize=8.5, color=FF.INK, va='bottom')
    axes[0].legend(loc='upper left', fontsize=7.5)
    fig.suptitle('10/10 기록 %s, 가장 긴 전진 구간 %.1f m: 국소 격자(0.10 m 셀)와 빠른 층' % (data['record'][9:15],
                                                                              data['segment_m']), fontsize=10.5)
    return _save(plt, fig, out, 'fig01_grid_snapshot', FOOT_REPLAY)


# --- fig02 / fig03: replay statistics ------------------------------------------------------------------------------------
def _replay_rows(folder, variant):
    rows = []
    for name in sorted(os.listdir(folder)):
        if name.startswith('grid_') and name.endswith('.npz'):
            data = np.load(os.path.join(folder, name))
            if variant in data:
                rows.append(data[variant])
    return np.concatenate(rows) if rows else np.zeros((0, 10))


def fig02(plt, folder, summary, out, variant='c010_n2'):
    rows = _replay_rows(folder, variant)
    has = np.isfinite(rows[:, 2])
    steady = (rows[:, 7] <= 3.) & (rows[:, 8] >= 2.5) & (rows[:, 8] <= 5.5)
    fig, axes = plt.subplots(1, 2, figsize=(10., 3.8))
    for ax, col, limit, unit, title, right in ((axes[0], 2, .10, 'm', '횡오차 차이 |격자 - 빠른 층|', .8),
                                               (axes[1], 3, 1.5, 'deg', '방향 오차 차이 |격자 - 빠른 층|', 8.)):
        for mask, color, label in ((has, FF.BLUE, '유효 틱 전체'), (has & steady, FF.GREEN, '직진·통로 폭 틱')):
            values = np.sort(np.abs(rows[mask, col]))
            if not len(values):
                continue
            ax.plot(values, np.arange(1, len(values) + 1) / len(values), color=color, lw=1.8,
                    label='%s (격자 선 있는 %d틱)' % (label, len(values)))
        ax.axvline(limit, color=FF.RED, lw=1., ls='--')
        ax.text(limit, .05, ' %g %s' % (limit, unit), color=FF.RED, fontsize=8)
        ax.set_xlim(0, right)
        ax.set_xlabel('|차이| [%s] (오른쪽 끝을 넘는 값은 잘림)' % unit)
        ax.set_ylabel('누적 비율')
        ax.set_title(title)
        ax.set_ylim(0, 1.)
    v = summary['variants'][variant]
    axes[0].legend(loc='upper left', fontsize=7.5)
    fig.suptitle('10/10 재생, 격자 %s: 유효 틱 %d, 격자 선 있음 %d, 0.10 m·1.5° 안 %.1f %% (선 있는 틱 중 %.1f %%)' % (
        variant, v['valid_ticks'], v['with_fit'], 100. * v['agree_strict'], 100. * v['agree_with_fit']), fontsize=10)
    fig.subplots_adjust(top=.84, wspace=.25)
    return _save(plt, fig, out, 'fig02_replay_agree', FOOT_REPLAY, foot_y=-.02)


def fig03(plt, summary, out):
    """summary: the SUMMARY of the replay whose wall times are shown (--timing, else --replay)."""
    names = [n for n in VARIANT_ORDER if n in summary['variants']]
    fig, ax = plt.subplots(figsize=(9., 3.6))
    x = np.arange(len(names) + 1)
    labels = names + ['추정기 on_scan']
    values = [summary['variants'][n]['update_ms_p50_p90_p99_max'] for n in names] + \
        [summary['estimator_on_scan_ms_p50_p90_p99_max']]
    for k, (q, color) in enumerate(zip(('p50', 'p90', 'p99'), FF.BLUES[1:4])):
        ax.bar(x + (k - 1) * .26, [v[k] for v in values], width=.25, color=color, label=q)
    ax.axhline(15., color=FF.RED, lw=1., ls='--')
    ax.text(len(names) + .45, 15., ' PC p99 목표 15 ms', color=FF.RED, fontsize=8, va='bottom', ha='right')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=20, fontsize=8, ha='right')
    ax.set_ylabel('벽시계 [ms] (PC)')
    ax.set_title('격자 한 번 갱신(넣기 + 선 맞춤)의 시간, 10/10 재생 (%s)' % summary.get('load_note', 'PC'))
    ax.legend(loc='upper left')
    return _save(plt, fig, out, 'fig03_replay_cost', FOOT_REPLAY, foot_y=-.08)


# --- fig04 / fig05: SIM comparison -----------------------------------------------------------------------------------
def fig04(plt, report, out):
    rows = [r for r in report['rows'] if not r['condition'].startswith('gnss_')]
    conditions = []
    for r in rows:
        key = '%s s%d' % (r['condition'], r['seed'])
        if key not in conditions:
            conditions.append(key)
    variants = [v for v in ('off', 'grid_fuse') if any(r['variant'] == v for r in rows)]
    colors = {'off': FF.MUTED, 'grid_fuse': FF.BLUE}
    fig, axes = plt.subplots(1, 2, figsize=(12., 4.), gridspec_kw=dict(width_ratios=(2.2, 1.)))
    ax = axes[0]
    x = np.arange(len(conditions))
    for k, variant in enumerate(variants):
        values = []
        for key in conditions:
            r = next((r for r in rows if '%s s%d' % (r['condition'], r['seed']) == key and r['variant'] == variant),
                     None)
            values.append(np.nan if r is None else (r['metrics'].get('lateral_rmse_m') or np.nan))
        ax.bar(x + (k - .5) * .38, values, width=.36, color=colors[variant],
               label={'off': 'off (층 없음)', 'grid_fuse': 'grid_fuse (격자 이득 0.2; all 과 같음)'}[variant])
    ax.set_xticks(x)
    ax.set_xticklabels(conditions, rotation=60, fontsize=7.5, ha='right')
    ax.set_ylabel('횡 RMSE [m] (참값)')
    ax.set_title('조건별 횡 RMSE (stale·obstacle 은 주행 구간이 짧음)')
    ax.set_ylim(0, ax.get_ylim()[1] * 1.3)
    ax.legend(loc='upper left', ncol=2)
    ax = axes[1]
    acc = (report['summary'].get('grid_log') or {}).get('accuracy') or (report['summary'].get('all') or {}).get(
        'accuracy')
    if acc:
        fused = (report['summary'].get('grid_fuse') or {}).get('accuracy')
        ax.bar(np.arange(2) - .27, acc['l0_lateral_abs_m'], width=.26, color=FF.ORANGE, label='빠른 층 L0 (이득 0)')
        ax.bar(np.arange(2), acc['grid_lateral_abs_m'], width=.26, color=FF.BLUE, label='격자 L2')
        if fused:
            ax.bar(np.arange(2) + .27, fused['l0_lateral_abs_m'], width=.26, color=FF.GREEN,
                   label='이득 0.2로 섞은 L0')
        ax.set_xticks(np.arange(2))
        ax.set_xticklabels(['횡 |오차| p50', '횡 |오차| p90'])
        ax.set_ylabel('|오차| [m] (참값 대비)')
        text = ('방향 |오차| p50 / p90' + chr(10) + 'L0 %.2f / %.2f°' + chr(10) + '격자 %.2f / %.2f°') % (
            tuple(acc['l0_heading_abs_deg']) + tuple(acc['grid_heading_abs_deg']))
        if fused:
            text += (chr(10) + '섞은 L0 %.2f / %.2f°') % tuple(fused['l0_heading_abs_deg'])
        ax.set_ylim(0, ax.get_ylim()[1] * 1.6)
        ax.text(.98, .97, text, transform=ax.transAxes, ha='right', va='top', fontsize=8)
        ax.set_title('같은 틱의 정확도 (run별 값의 중앙값, %d run)' % acc['runs'])
        ax.legend(loc='upper left', fontsize=7.5)
    fig.subplots_adjust(bottom=.3, wspace=.3)
    return _save(plt, fig, out, 'fig04_sim_compare', FOOT_SIM, foot_y=.01)


def fig05(plt, folder, report, out):
    wanted = ('gnss_dropout_60s', 'gnss_jump_15m', 'gnss_hacc_x3', 'gnss_bias_20m')
    fig, axes = plt.subplots(len(wanted), 1, figsize=(10., 2.0 * len(wanted)), sharex=True)
    for ax, cond in zip(axes, wanted):
        row = next((r for r in report['rows'] if r['condition'] == cond and r['seed'] == 0 and r['variant'] == 'all'),
                   None)
        if row is None:
            continue
        path = os.path.join(folder, 'runs', row['name'], 'EXECUTOR.jsonl')
        ticks = [json.loads(line) for line in open(path, 'rb').read().decode('utf-8').splitlines()
                 if '"k":"tick"' in line]
        t = np.array([r['t'] for r in ticks if 'gnss' in r])
        dis = np.array([r['gnss'].get('disagreement_m', np.nan) for r in ticks if 'gnss' in r], dtype=float)
        present = np.array([bool(r['gnss'].get('present')) for r in ticks if 'gnss' in r])
        ax.plot(t, dis, color=FF.BLUE, lw=1.4, label='위치 불일치 [m]')
        ax.fill_between(t, 0, 25, where=~present, color=FF.BAND, step='post', label='GNSS 없음(끊김·품질 미달)')
        stamps = (row.get('receiver') or {}).get('jump_stamps') or []
        for k, s in enumerate(stamps):
            ax.axvline(s, color=FF.RED, lw=1., ls=':', label='튐 주입' if k == 0 else None)
        g = row['layers'].get('gnss') or {}
        counts = g.get('counts') or {}
        ax.set_ylim(0, 25)
        ax.set_ylabel('m')
        ax.set_title('%s: 받아들임 %s, 튐 기각 %s / 주입 %s, 품질 미달 %s, 펜스 %s, 끝 %s' % (
            cond, counts.get('accepted'), counts.get('jump'), len(stamps), counts.get('quality'),
            '작동' if g.get('fence_tripped') else '안 걸림', row['metrics'].get('end_reason')), fontsize=9)
    axes[0].legend(loc='upper right', fontsize=7.5, ncol=3)
    axes[-1].set_xlabel('SIM 시각 [s]')
    return _save(plt, fig, out, 'fig05_gnss_sim', FOOT_SIM)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--replay', required=True)
    p.add_argument('--sim', required=True)
    p.add_argument('--out', default=DEFAULT_OUT)
    p.add_argument('--snapshot-record', default='20261010T045553Z_both')
    p.add_argument('--snapshot-at', nargs='*', type=float, default=[12., 28.])
    p.add_argument('--mask-dir')
    p.add_argument('--load-note', default='PC')
    p.add_argument('--summary', default='SUMMARY.json', help='summary file name inside --replay')
    p.add_argument('--timing', help='replay folder whose SUMMARY.json gives the wall times of fig03 (default --replay)')
    args = p.parse_args(argv)
    plt = FF._plt()
    summary = json.loads(open(os.path.join(args.replay, args.summary), 'rb').read().decode('utf-8'))
    timing = summary if not args.timing else json.loads(open(os.path.join(args.timing, 'SUMMARY.json'), 'rb')
                                                        .read().decode('utf-8'))
    timing['load_note'] = args.load_note
    report = json.loads(open(os.path.join(args.sim, 'PERCEPTION_COMPARE.json'), 'rb').read().decode('utf-8'))
    paths = [fig01(plt, replay_snapshot(args.snapshot_record, tuple(args.snapshot_at), args.mask_dir), args.out),
             fig02(plt, args.replay, summary, args.out), fig03(plt, timing, args.out), fig04(plt, report, args.out),
             fig05(plt, args.sim, report, args.out)]
    for path in paths:
        print(path, os.path.getsize(path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
