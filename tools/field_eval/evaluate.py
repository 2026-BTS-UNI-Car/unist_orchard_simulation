"""One run's evaluation: reference map + executor JSONL + scan source (or an external reference trajectory) ->
REPORT.json, TRAJECTORY.csv and SVG figures in an output folder.

Time: the executor ticks run on the executor clock (time.monotonic on the Jetson). A source on the wall clock
(recorder sidecar, rosbag2) is matched to them through the median wall - t of the executor events; an F-SIM capture
shares the executor's SIM clock.
"""
from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import os

import numpy as np

from . import exec_log as XL
from . import localise as LOC
from . import metrics as MET
from . import refmap as RM
from . import sources as SRC
from . import svg_plot as SP

SCHEMA = 'ubts.field_eval.report.v1'
SIM_KIND = '자리값 기반 SIM 결과'
FIELD_KIND = '실차 기록 평가(기준 궤적 = 지도 기반 LiDAR 위치 추정, 참값 아님)'
REST_WINDOW_S = 5.


def _sha(path):
    return SRC.sha256_file(path)


def vehicle_from(executor, vehicle_file=None):
    """Vehicle values: the vehicle file over the executor's config over the defaults; with the origin of each."""
    values, origin = {}, {}
    defaults = dict(lidar_y_in_base_m=0., footprint_length_m=1.22, footprint_width_m=.70, imu_yaw_sign=1.)
    for k, v in defaults.items():
        values[k], origin[k] = v, 'default'
    for k, v in executor.vehicle().items():
        values[k], origin[k] = v, 'executor_config'
    if vehicle_file:
        with open(vehicle_file, 'rb') as fh:
            doc = json.loads(fh.read().decode('utf-8'))
        for k in XL.VEHICLE_KEYS:
            if doc.get(k) is not None:
                values[k], origin[k] = float(doc[k]), 'vehicle_file'
    for k in ('lidar_x_in_base_m', 'lidar_yaw_offset_deg'):
        if k not in values:
            raise ValueError('vehicle value %s is unknown (not in the executor config; give --vehicle)' % k)
    return LOC.Vehicle(**{k: values[k] for k in ('lidar_x_in_base_m', 'lidar_yaw_offset_deg', 'lidar_y_in_base_m',
                                                 'footprint_length_m', 'footprint_width_m', 'imu_yaw_sign')}), origin


def read_trajectory_csv(path):
    """External reference trajectory: columns x_m, y_m and optionally t_s, yaw_deg (map frame)."""
    with open(path, 'rb') as fh:
        text = fh.read().decode('utf-8-sig')
    rows = list(csv.DictReader(io.StringIO('\n'.join(l for l in text.splitlines() if not l.startswith('#')))))
    for f in rows[0] if rows else ():
        if RM._geo_key(f):
            raise RM.MapError('trajectory column %r looks like a geographic coordinate' % f)
    x = np.array([float(r['x_m']) for r in rows])
    y = np.array([float(r['y_m']) for r in rows])
    t = np.array([float(r['t_s']) for r in rows]) if rows and 't_s' in rows[0] else np.arange(len(rows), dtype=float)
    if rows and 'yaw_deg' in rows[0]:
        yaw = np.radians([float(r['yaw_deg']) for r in rows])
    else:
        yaw = np.arctan2(np.gradient(y), np.gradient(x)) if len(x) > 1 else np.zeros(len(x))
    return dict(t=t, x=x, y=y, yaw=np.asarray(yaw, float), quality=np.full(len(x), LOC.OK, dtype=int),
                timed='t_s' in (rows[0] if rows else {}))


def _write_trajectory(path, traj, refmap):
    s, d, psi = refmap.to_row_arrays(traj['x'], traj['y'], traj['yaw'])
    cols = ['t_s', 'x_m', 'y_m', 'yaw_deg', 'along_m', 'lateral_m', 'heading_deg', 'speed_m_s', 'quality', 'n_match',
            'rms_m']
    lines = [','.join(cols)]
    for i in range(len(traj['t'])):
        vals = [traj['t'][i], traj['x'][i], traj['y'][i], math.degrees(traj['yaw'][i]) if np.isfinite(traj['yaw'][i])
                else math.nan, s[i], d[i], math.degrees(psi[i]) if np.isfinite(psi[i]) else math.nan,
                traj['speed'][i]]
        text = ['' if not np.isfinite(v) else '%.4f' % v for v in vals]
        text.append(LOC.QUALITY_NAMES[int(traj['quality'][i])])
        text.append(str(int(traj.get('n_match', np.zeros(len(traj['t']), int))[i])))
        rms = traj.get('rms_m', np.full(len(traj['t']), np.nan))[i]
        text.append('' if not np.isfinite(rms) else '%.4f' % rms)
        lines.append(','.join(text))
    with open(path, 'wb') as fh:
        fh.write(('\n'.join(lines) + '\n').encode('utf-8'))


def evaluate(map_path, executor_path, scans_path=None, out_dir=None, vehicle_file=None, start_pose=None,
             settle_m=0., run=None, trajectory_csv=None, result_kind=None, use_header=None, loc_config=None,
             truth=None, figures=True, scans_kind=None):
    """Run the evaluation; returns the report dict (and writes the files when out_dir is given).
    truth: optional dict(t, x, y, yaw, label) drawn in the figures (SIM verification only)."""
    refmap = RM.load_map(map_path)
    executor = XL.ExecutorLog(executor_path, run=run)
    vehicle, vehicle_origin = vehicle_from(executor, vehicle_file)
    inputs = dict(map=dict(file=os.path.basename(map_path), sha256=refmap.sha256),
                  executor=dict(files=executor.sha256))
    if vehicle_file:
        inputs['vehicle'] = dict(file=os.path.basename(vehicle_file), sha256=_sha(vehicle_file))
    drive_exec = executor.drive_interval()
    ticks = executor.run_ticks()
    loc_info, traj = None, None
    time_base = 'executor'
    if scans_path is not None:
        scans, imu = SRC.open_source(scans_path, kind=scans_kind, use_header=use_header)
        time_base = scans['time_base']
        inputs['scans'] = dict(source=scans['source'], path=os.path.basename(os.path.normpath(scans_path)),
                               sha256=scans['sha256'], scans=int(len(scans['t'])), beams=int(scans['beams']),
                               imu_samples=int(len(imu['t'])), time_base=time_base,
                               warnings=scans.get('warnings', []))
    shift = 0.
    if time_base == 'wall':
        if executor.wall_minus_t is None:
            raise ValueError('the executor log has no wall-clock events to match a wall-clock source')
        shift = executor.wall_minus_t
    drive = None if drive_exec is None else (drive_exec[0] + shift, drive_exec[1] + shift)
    tick_t = ticks['t'] + shift
    if scans_path is not None:
        loc = LOC.Localiser(refmap, vehicle, loc_config)
        start = start_pose or refmap.start_pose
        traj = loc.run(scans, imu, start, None if drive is None else drive[0])
        rerun = None
        check = traj['along_check']
        if start is not None and check.get('suspect') and check.get('best_shift') not in (0, None):
            # one landmark spacing off: run again from the start moved by the better shift, keep the better run
            axis = refmap.geometry['axis_rad']
            k = check['best_shift'] * check['spacing_m']
            start2 = (start[0] + k * math.cos(axis), start[1] + k * math.sin(axis), start[2])
            traj2 = loc.run(scans, imu, start2, None if drive is None else drive[0])
            kept = 'rerun' if traj2['along_check']['scores']['0'] > check['scores']['0'] else 'first'
            rerun = dict(first_start=list(start), rerun_start=list(start2), first_check=check,
                         rerun_check=traj2['along_check'], kept=kept)
            if kept == 'rerun':
                traj = traj2
        loc_info = dict(method='landmark map: clusters -> circle centres -> nearest-landmark association -> '
                               'weighted SE(2) least squares; IMU yaw-rate prediction',
                        config=loc.cfg, init=traj['init'], imu=traj['imu'], along_check=traj['along_check'],
                        along_rerun=rerun,
                        overall=LOC.summary(traj), in_drive=None if drive is None else LOC.summary(traj, drive))
    elif trajectory_csv is not None:
        traj = read_trajectory_csv(trajectory_csv)
        inputs['trajectory'] = dict(file=os.path.basename(trajectory_csv), sha256=_sha(trajectory_csv),
                                    timed=traj['timed'])
        for k in ('n_match', 'rms_m'):
            traj.setdefault(k, np.zeros(len(traj['t'])) if k == 'n_match' else np.full(len(traj['t']), np.nan))
    else:
        raise ValueError('give a scan source (--scans) or a reference trajectory (--trajectory)')
    valid = (traj['quality'] == LOC.OK) | (traj['quality'] == LOC.WEAK)
    speed = MET.path_speed(traj['t'], np.where(valid, traj['x'], np.nan), np.where(valid, traj['y'], np.nan),
                           traj['yaw'])
    traj['speed'] = speed
    timed = traj.get('timed', True)
    reference = MET.measure(refmap, traj['t'], traj['x'], traj['y'], traj['yaw'], speed,
                            drive if timed else (float(traj['t'][0]), float(traj['t'][-1])), settle_m,
                            vehicle.front_bumper_m, valid=valid, rest_window_s=REST_WINDOW_S if timed else None)
    est, est_series = (None, None)
    if timed:
        est, est_series = MET.estimate_error(refmap, traj['t'], traj['x'], traj['y'], traj['yaw'], valid,
                                             dict(t=tick_t, lat=ticks['lat'], head=ticks['head']), drive)
    kind = result_kind or (SIM_KIND if inputs.get('scans', {}).get('source') == 'fsim' else FIELD_KIND)
    warnings = list(inputs.get('scans', {}).get('warnings') or [])
    if drive is None:
        warnings.append('no_drive_command: the run has no tick with T > 0')
    if loc_info is not None:
        inside = loc_info['in_drive'] or loc_info['overall']
        if (inside.get('ok_fraction') or 0.) < .9:
            warnings.append('localisation_ok_below_90pct: %s' % inside.get('ok_fraction'))
        if (inside.get('measured_fraction') or 0.) < .98:
            warnings.append('localisation_unmeasured_scans: measured fraction %s' % inside.get('measured_fraction'))
        if loc_info['along_rerun'] is not None:
            warnings.append('along_alias_rerun: the first solution was one landmark spacing off along the row; kept '
                            'the %s run (localisation.along_rerun)' % loc_info['along_rerun']['kept'])
        if loc_info['along_check'].get('suspect'):
            warnings.append('along_alias_suspect: %s' % json.dumps(loc_info['along_check'], sort_keys=True))
        if loc_info['init'] is None:
            warnings.append('localisation_never_started')
    if time_base == 'wall' and executor.wall_offset_spread_s > .05:
        warnings.append('executor_wall_clock_spread_s: %.3f' % executor.wall_offset_spread_s)
    if executor.bad_lines:
        warnings.append('executor_bad_lines: %d' % executor.bad_lines)
    report = dict(schema=SCHEMA, result_kind=kind, inputs=inputs, config_digest=executor.config_digests,
                  executor=executor.describe(), vehicle=dict(vehicle.describe(), origin=vehicle_origin),
                  map=refmap.describe(), time=dict(base=time_base, executor_to_source_shift_s=shift,
                                                   drive_interval_executor_s=drive_exec, drive_interval_s=drive),
                  localisation=loc_info, settle_m=settle_m, warnings=warnings,
                  reference_metrics=reference, limits_steer_follow_v1=MET.against_limits(reference),
                  executor_estimate_error=est,
                  definitions=dict(lateral='footprint centre to the reference centre line (left positive)',
                                   heading='vehicle yaw minus the row axis',
                                   executor_heading='minus the tick head (head = corridor heading - vehicle heading)',
                                   drive_interval='first to last executor tick with T > 0',
                                   rest='last measured pose at or before the drive end + %.0f s' % REST_WINDOW_S,
                                   speed='local linear fit of the reference positions over +-0.5 s'),
                  note='limits_steer_follow_v1 lists the SIM limits next to the values; it is not a judgement')
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        _write_trajectory(os.path.join(out_dir, 'TRAJECTORY.csv'), traj, refmap)
        if figures:
            report['figures'] = make_figures(out_dir, refmap, traj, valid, executor, ticks, tick_t, drive, est_series,
                                             vehicle, kind, truth)
        raw = json.dumps(clean(report), indent=1, sort_keys=True, ensure_ascii=False, allow_nan=False,
                         default=_jsonable) + '\n'
        with open(os.path.join(out_dir, 'REPORT.json'), 'wb') as fh:
            fh.write(raw.encode('utf-8'))
    report['_trajectory'] = traj
    report['_estimate_series'] = est_series
    return report


def clean(value):
    """JSON-able copy: NaN / inf -> None, numpy scalars -> Python, private '_' keys dropped."""
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items() if not str(k).startswith('_')}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, np.ndarray):
        return [clean(v) for v in value.tolist()]
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    return value


def _jsonable(v):
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (np.floating,)):
        return None if not np.isfinite(v) else float(v)
    if isinstance(v, np.ndarray):
        return v.tolist()
    return str(v)


def make_figures(out_dir, refmap, traj, valid, executor, ticks, tick_t, drive, est_series, vehicle, kind, truth=None):
    names = []
    g = refmap.geometry
    lm, side = refmap.landmark_array()
    caption = ['출처: %s. 지도 %s (sha256 %s...), 실행기 기록 run %s.' % (kind, refmap.source, (refmap.sha256 or '')[:12],
                                                                    executor.run)]
    # 1. trajectory, whole observed extent (equal axes) + detail
    s0, s1 = g['row_start_s'] - 1., g['row_end_s'] + 1.
    cx0, cy0, _ = refmap.from_row(s0, 0.)
    cx1, cy1, _ = refmap.from_row(s1, 0.)
    full = SP.Panel('전체 관측 범위 (축 비율 1:1)', 'x [m] (지도)', 'y [m] (지도)', equal=True)
    if len(lm):
        full.circles(lm[:, 0], lm[:, 1], np.maximum(lm[:, 2], .05), label='지도 기둥·줄기', colour='#6b6b6b')
    full.line([cx0, cx1], [cy0, cy1], label='기준 중심선', colour='#2e8b57', dash='6 3')
    if truth is not None:
        full.line(truth['x'], truth['y'], label=truth.get('label', '참값'), colour='#d9661f', width=2.4)
    full.line(np.where(valid, traj['x'], np.nan), np.where(valid, traj['y'], np.nan), label='기준 궤적(LiDAR 지도 정합)',
              colour='#1f5fbf')
    s, d, psi = refmap.to_row_arrays(traj['x'], traj['y'], traj['yaw'])
    detail = SP.Panel('상세: 횡 위치 (세로 확대, 축 비율 다름)', '행 방향 위치 s [m]', '횡 위치 d [cm] (왼쪽 +)', height=200)
    detail.line([s0, s1], [0., 0.], label='기준 중심선', colour='#2e8b57', dash='6 3')
    if truth is not None:
        ts, td, _ = refmap.to_row_arrays(truth['x'], truth['y'], truth['yaw'])
        detail.line(ts, td * 100., label=truth.get('label', '참값'), colour='#d9661f', width=2.4)
    detail.line(np.where(valid, s, np.nan), np.where(valid, d * 100., np.nan), label='기준 궤적', colour='#1f5fbf')
    if est_series is not None:
        detail.points(est_series['along'], est_series['lat_exec'] * 100., label='실행기 온라인 추정', colour='#7d3c98',
                      size=1.2)
    path = os.path.join(out_dir, 'fig_trajectory.svg')
    SP.figure(path, [full, detail], '주행 궤적과 기준 중심선', caption=caption)
    names.append(os.path.basename(path))
    # 2. lateral / heading error vs distance, executor estimate error
    if drive is not None:
        inside = valid & (traj['t'] >= drive[0]) & (traj['t'] <= drive[1])
        idx = np.flatnonzero(inside)
        if len(idx) > 1:
            dist = np.concatenate(([0.], np.cumsum(np.hypot(np.diff(traj['x'][idx]), np.diff(traj['y'][idx])))))
            p1 = SP.Panel('횡오차 대 주행 거리', '주행 거리 [m]', '횡오차 [cm]', height=200)
            p1.line(dist, d[idx] * 100., label='기준 궤적', colour='#1f5fbf')
            p2 = SP.Panel('방향 오차 대 주행 거리', '주행 거리 [m]', '방향 오차 [deg]', height=170)
            p2.line(dist, np.degrees(psi[idx]), label='기준 궤적', colour='#1f5fbf')
            panels = [p1, p2]
            if est_series is not None:
                dq = np.interp(est_series['t'], traj['t'][idx], dist)
                p1.points(dq, est_series['lat_exec'] * 100., label='실행기 온라인 추정', colour='#7d3c98', size=1.2)
                p2.points(dq, est_series['head_exec_deg'], label='실행기 온라인 추정', colour='#7d3c98', size=1.2)
                p3 = SP.Panel('실행기 추정 오차 (실행기 - 기준)', '주행 거리 [m]', '오차 [cm] / [deg]', height=170)
                p3.line(dq, (est_series['lat_exec'] - est_series['lat_ref']) * 100., label='횡 [cm]', colour='#c0392b')
                p3.line(dq, est_series['head_exec_deg'] - est_series['head_ref_deg'], label='방향 [deg]',
                        colour='#b8860b')
                panels.append(p3)
            path = os.path.join(out_dir, 'fig_lateral_vs_distance.svg')
            SP.figure(path, panels, '횡오차·방향 오차', caption=caption)
            names.append(os.path.basename(path))
    # 3. target vs LiDAR speed; 4. steering; 5. duty vs speed
    t0 = float(tick_t[0]) if len(tick_t) else 0.
    p = SP.Panel('목표 속도 대 LiDAR 속도', '시간 [s] (기록 시작부터)', '속도 [m/s]', height=200)
    p.line(tick_t - t0, ticks['vt'], label='목표 속도 (실행기 sr.vt)', colour='#2e8b57')
    p.line(tick_t - t0, ticks['v'], label='실행기 LiDAR 속도', colour='#7d3c98', width=1.0)
    p.line(traj['t'] - t0, np.where(valid, traj['speed'], np.nan), label='기준 궤적 속도', colour='#1f5fbf')
    if truth is not None and truth.get('v') is not None:
        p.line(truth['t'] - t0, truth['v'], label='참값 속도', colour='#d9661f', width=1.0)
    path = os.path.join(out_dir, 'fig_speed.svg')
    SP.figure(path, [p], '속도', caption=caption)
    names.append(os.path.basename(path))
    p = SP.Panel('목표 조향각 대 측정 조향각', '시간 [s] (기록 시작부터)', '조향각 [deg]', height=200)
    p.line(tick_t - t0, np.degrees(ticks['S'] / 1000.), label='목표 (실행기 S)', colour='#1f5fbf')
    p.line(tick_t - t0, np.degrees(ticks['meas'] / 1000.), label='측정 (가변저항, steer.meas)', colour='#d9661f')
    path = os.path.join(out_dir, 'fig_steering.svg')
    SP.figure(path, [p], '조향', caption=caption)
    names.append(os.path.basename(path))
    if drive is not None:
        sel = (tick_t >= drive[0]) & (tick_t <= drive[1])
        v_ref = MET.interp_valid(traj['t'], traj['speed'], valid & np.isfinite(traj['speed']), tick_t[sel])
        p = SP.Panel('duty 대 속도 (주행 구간 틱)', '구동 duty T [%]', '속도 [m/s]', height=220)
        p.points(ticks['T'][sel], v_ref, label='기준 궤적 속도', colour='#1f5fbf', size=1.8)
        p.points(ticks['T'][sel], ticks['v'][sel], label='실행기 LiDAR 속도', colour='#7d3c98', size=1.2)
        path = os.path.join(out_dir, 'fig_duty_speed.svg')
        SP.figure(path, [p], 'duty 대 속도', caption=caption)
        names.append(os.path.basename(path))
    return names


def digest_report(report):
    raw = json.dumps(clean(report), sort_keys=True, ensure_ascii=False, default=_jsonable).encode('utf-8')
    return hashlib.sha256(raw).hexdigest()
