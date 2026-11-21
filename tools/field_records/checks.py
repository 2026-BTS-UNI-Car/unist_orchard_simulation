"""Checks of field recordings on the extracted time series (out/<id>.npz, written by bag_extract.py).

Per recording: sensor rates and gaps as they are in the bag, and the executor's own numbers held against sensors it
does not compute them from (GNSS displacement and speed, LiDAR distances to the two tree rows straight from the scan,
the bridge's steering reading, the IMU heading). No GNSS position is printed: only distances between samples.
Output: out/CHECKS.json and a table on stdout. The work folder comes from UBTS_FIELD_OUT (as in bag_extract.py).
usage: python -B checks.py
"""
import json
import os
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

OUT = Path(os.environ['UBTS_FIELD_OUT']) / 'out'
KST = timezone(timedelta(hours=9))
sys.stdout.reconfigure(encoding='utf-8')


def med(v):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    return float(np.median(v)) if len(v) else None


def rnd(v, n=2):
    return None if v is None or not np.isfinite(v) else round(float(v), n)


def gaps(t, limit):
    t = np.asarray(t, dtype=float)
    if len(t) < 2:
        return 0, None
    d = np.diff(t)
    return int((d > limit).sum()), float(d.max())


rows = []
for path in sorted(OUT.glob('2026*.npz')):
    name = path.stem
    z = np.load(path, allow_pickle=False)
    info = json.loads((OUT / (name + '.json')).read_bytes().decode('utf-8'))
    began = datetime.strptime(name[:16], '%Y%m%dT%H%M%SZ').replace(tzinfo=timezone.utc).astimezone(KST)
    row = {'id': name, 'start_kst': began.strftime('%H:%M:%S'), 'truncated': info['truncated'], 'bag_mb': info['bag_mb']}
    ex_t = z['ex_t']
    active = z['ex_state'] == 'ACTIVE' if 'ex_state' in z.files and len(ex_t) else np.zeros(0, bool)
    row['executor_rows'] = int(len(ex_t))
    if active.any():
        a0, a1 = float(ex_t[active][0]), float(ex_t[active][-1])
        row['active_s'] = [rnd(a0), rnd(a1)]
        dist = z['ex_dist_m'][active]
        row['exec_distance_m'] = rnd(np.nanmax(dist) - np.nanmin(dist))
        row['exec_speed_med'] = rnd(med(z['ex_speed_mps'][active]), 3)
        row['exec_target_med'] = rnd(med(z['ex_target_speed_mps'][active]), 3)
        row['exec_throttle_med'] = rnd(med(z['ex_throttle_pct'][active]), 1)
        lat = z['ex_lat_m'][active]
        row['exec_lat_rms_cm'] = rnd(np.sqrt(np.nanmean(lat ** 2)) * 100, 1)
        row['exec_lat_mean_cm'] = rnd(np.nanmean(lat) * 100, 1)
        row['exec_lat_maxabs_cm'] = rnd(np.nanmax(np.abs(lat)) * 100, 1)
        row['exec_head_rms_deg'] = rnd(np.sqrt(np.nanmean(z['ex_head_deg'][active] ** 2)), 2)
        row['exec_width_med'] = rnd(med(z['ex_width_m'][active]), 3)
        row['exec_steer_cmd_med_mrad'] = rnd(med(z['ex_steer_cmd_mrad'][active]), 1)
        row['exec_steer_meas_med_mrad'] = rnd(med(z['ex_steer_meas_mrad'][active]), 1)
        states = [str(s) for s in z['ex_state']]
        reasons = [str(s) for s in z['ex_reason']]
        row['last_state'] = '%s/%s' % (states[-1], reasons[-1])
        stops = [str(s) for s in z['ex_stop_reason'] if str(s) not in ('None', '')]
        row['stop_reason'] = stops[-1] if stops else None
        # distance between the executor's own pose at the start and the end of the drive
        px, py = z['ex_pose_x'][active], z['ex_pose_y'][active]
        ok = np.isfinite(px) & np.isfinite(py)
        if ok.sum() > 2:
            row['exec_pose_straight_m'] = rnd(np.hypot(px[ok][-1] - px[ok][0], py[ok][-1] - py[ok][0]))
            row['exec_pose_yaw_change_deg'] = rnd(float(z['ex_pose_yaw_deg'][active][ok][-1] - z['ex_pose_yaw_deg'][active][ok][0]), 1)
        # GNSS: straight distance between the medians of the first and the last 3 s of the drive, and the path length
        gt, ge, gn, gv = z['gp_t'], z['gp_e'], z['gp_n'], z['gp_valid']
        sel = gv & (gt >= a0) & (gt <= a1)
        row['gnss_samples_in_drive'] = int(sel.sum())
        if sel.sum() >= 8:
            t = gt[sel]
            head, tail = t <= t[0] + 3.0, t >= t[-1] - 3.0
            e0, n0 = np.median(ge[sel][head]), np.median(gn[sel][head])
            e1, n1 = np.median(ge[sel][tail]), np.median(gn[sel][tail])
            row['gnss_straight_m'] = rnd(np.hypot(e1 - e0, n1 - n0))
            row['gnss_hacc_med_m'] = rnd(med(z['gp_hacc'][sel]))
            row['gnss_sat_med'] = rnd(med(z['gp_sat'][sel]), 0)
            row['gnss_speed_med'] = rnd(med(z['gp_speed'][sel]), 3)
            fixes = sorted(set(str(f) for f in z['gp_fix'][sel]))
            row['gnss_fix_types'] = fixes
            # how straight the GNSS track is: RMS distance of the samples from the start-end line
            ux, uy = e1 - e0, n1 - n0
            norm = np.hypot(ux, uy)
            if norm > 5:
                cross = ((ge[sel] - e0) * uy - (gn[sel] - n0) * ux) / norm
                row['gnss_off_line_rms_m'] = rnd(np.sqrt(np.nanmean(cross ** 2)))
        # LiDAR straight from the scan: nearest returns to the left and right within 3 m ahead/behind
        st = z['scan_t']
        sel = (st >= a0) & (st <= a1)
        if sel.any():
            left, right = z['scan_left'][sel], z['scan_right'][sel]
            row['scan_left_med'] = rnd(med(left), 3)
            row['scan_right_med'] = rnd(med(right), 3)
            both = left + right
            row['scan_width_med'] = rnd(med(both), 3)
            row['scan_centre_offset_med_cm'] = rnd(med((right - left) / 2) * 100, 1)      # + : the car is left of the middle between the nearest returns
        # steering as the bridge reports it
        t = z['st_t']
        sel = (t >= a0) & (t <= a1)
        if sel.any():
            row['bridge_steer_meas_med_mrad'] = rnd(med(z['st_meas'][sel]), 1)
            row['bridge_steer_cmd_med_mrad'] = rnd(med(z['st_cmd'][sel]), 1)
            err = z['st_meas'][sel] - z['st_cmd'][sel]
            row['bridge_steer_err_rms_mrad'] = rnd(np.sqrt(np.nanmean(err ** 2)), 1)
            row['bridge_raw_med'] = rnd(med(z['st_raw'][sel]), 0)
            row['bridge_throttle_med'] = rnd(med(z['st_thr'][sel]), 1)
            row['steer_pwm_peak_max'] = rnd(np.nanmax(z['st_peak'][sel]), 0)
            stops = z['st_ttl_stops'][np.isfinite(z['st_ttl_stops'])]
            row['bridge_ttl_stops'] = [rnd(stops[0], 0), rnd(stops[-1], 0)] if len(stops) else None
        # IMU heading over the drive
        t = z['imu_t']
        sel = (t >= a0) & (t <= a1)
        if sel.sum() > 10:
            yaw = np.unwrap(z['imu_yaw'][sel].astype(float))
            row['imu_yaw_change_deg'] = rnd(np.degrees(yaw[-1] - yaw[0]), 1)
            row['imu_yaw_std_deg'] = rnd(np.degrees(np.std(yaw - np.linspace(yaw[0], yaw[-1], len(yaw)))), 2)
    # what the bridge said (whole recording)
    states = [str(s) for s in z['rs_state']]
    changes = [states[0]] if states else []
    for s in states[1:]:
        if s != changes[-1]:
            changes.append(s)
    row['bridge_states'] = changes[:12]
    errs = sorted(set(str(e) for e in z['rs_err']) - {'None'})
    row['bridge_last_errors'] = [e[:90] for e in errs][:4]
    faults = sorted(set(str(f) for f in z['rs_faults']) - {''})
    row['steering_faults'] = faults[:6]
    row['bridge_connected_all'] = bool(z['rs_conn'].all()) if len(z['rs_conn']) else None
    # rates and gaps
    for key, label, limit in (('scan_t', 'scan', 0.25), ('scan_stamp', 'scan_stamp', 0.25), ('imu_t', 'imu', 0.15), ('imu_stamp', 'imu_stamp', 0.15),
                              ('cam_t', 'camera', 0.25), ('cam_stamp', 'camera_stamp', 0.25), ('st_t', 'steer', 0.5), ('ex_t', 'executor', 0.5)):
        if key in z.files and len(z[key]) > 1:
            n, longest = gaps(z[key], limit)
            span = float(z[key][-1] - z[key][0])
            row['%s_hz' % label] = rnd((len(z[key]) - 1) / span if span > 0 else np.nan)
            row['%s_gaps' % label] = n
            row['%s_max_gap_s' % label] = rnd(longest, 3)
    rows.append(row)

(OUT / 'CHECKS.json').write_bytes(json.dumps(rows, ensure_ascii=False, indent=1).encode('utf-8'))
print('start    cut  | drive: exec m  pose m  gnss m (hAcc, sats)  | speed exec/gnss | width exec/scan  | steer meas exec/bridge mrad | lat rms cm | imu yaw change')
for r in rows:
    if 'exec_distance_m' not in r:
        print('%s %-4s | no drive in this recording' % (r['start_kst'], 'cut' if r['truncated'] else ''))
        continue
    print('%s %-4s | %6s %6s %6s (%s, %s) | %s / %s | %s / %s | %s / %s | %s | %s' % (
        r['start_kst'], 'cut' if r['truncated'] else '', r.get('exec_distance_m'), r.get('exec_pose_straight_m'), r.get('gnss_straight_m'), r.get('gnss_hacc_med_m'),
        r.get('gnss_sat_med'), r.get('exec_speed_med'), r.get('gnss_speed_med'), r.get('exec_width_med'), r.get('scan_width_med'),
        r.get('exec_steer_meas_med_mrad'), r.get('bridge_steer_meas_med_mrad'), r.get('exec_lat_rms_cm'), r.get('imu_yaw_change_deg')))
print()
print('start    | scan Hz gaps>0.25 longest (by stamp: gaps longest) | camera fps gaps>0.25 longest (stamp longest) | imu Hz gaps>0.15 longest | steer gaps>0.5 | exec gaps>0.5 longest')
for r in rows:
    print('%s | %s %s %s (%s %s) | %s %s %s (%s) | %s %s %s | %s | %s %s' % (
        r['start_kst'], r.get('scan_hz'), r.get('scan_gaps'), r.get('scan_max_gap_s'), r.get('scan_stamp_gaps'), r.get('scan_stamp_max_gap_s'),
        r.get('camera_hz'), r.get('camera_gaps'), r.get('camera_max_gap_s'), r.get('camera_stamp_max_gap_s'),
        r.get('imu_hz'), r.get('imu_gaps'), r.get('imu_max_gap_s'), r.get('steer_gaps'), r.get('executor_gaps'), r.get('executor_max_gap_s')))
print()
for r in rows:
    print('%s bridge: %s | errors %s | steering faults %s | connected all %s | ttl stops %s | last %s stop %s' % (
        r['start_kst'], '>'.join(r['bridge_states']), r['bridge_last_errors'], r['steering_faults'], r['bridge_connected_all'], r.get('bridge_ttl_stops'), r.get('last_state'), r.get('stop_reason')))
