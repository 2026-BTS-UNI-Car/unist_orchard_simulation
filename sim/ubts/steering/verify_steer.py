"""Integration check of the steering G-SIM stack (design 8.3 P6a; replaces sim/ubts/lifecycle.py and verify.py, which
check four wheel joints, a pivot drive(-35, 35) and left/right commands of the old differential stack).

Drives only the steering control socket /run/ubts-steer/control.sock (the bridge wrapper, the same API as the vehicle)
and reads the truth for evaluation. Run in the check service of compose.ubts-steer.yaml:

  python3 -B -m steering.verify_steer --out /runs/verify [--circle-s 20] [--throttle 40] [--steer-mrad 175]

  health         simulation_status.json: every sensors_alive value true (the pinned health.py rule)
  sensors        /imu/data_raw rate and frame, orientation not provided (covariance[0] -1); /sim/scan from the
                 Embree LiDAR; /scan from the S2 format node (WF-4: 3240 beams, frame laser, 10 Hz; absent with
                 --s2-node none); virtual GPS origin 0 N 0 E; static TF only (no /tf, no /odom publisher)
  arm            arm with vehicle_profile front_steer, MCU confirmation (st ARMED)
  circle         fixed steering angle +/-steer_mrad at the throttle: rear-axle-centre circle fit (8 s .. end), the
                 radius against L / tan(delta) of the commanded and of the true angle (G-SIM force mode; A1(G) gives
                 R / R_geom about 1.03 at 10 deg), yaw rate sign, the host's measured angle (potentiometer)
  stop           op stop while driving straight: STOP confirmed by the host, the vehicle stands, stays still for 3 s
                 with no new drive output (no automatic restart), steering held
  deadman        one drive, then nothing: the input deadline (0.75 s) ends the motion (reason input_expired)
  false detections   A19(G) normal script (R15-3): every MCU fault line, new fault bit and host latch of the run
                 that the script does not expect (only the C TTL of the deadman step is expected); report here, the
                 p6a_checks a19g_normal summary judges
  emulator_clock, rtf   the /clock gaps of the emulator and the 5 s RTF windows over the run (as every P6a check)

Writes <out>/VERIFY_STEER.json. Placeholder-based SIM results (자리값 기반 SIM 결과), not field validation.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time

from .p6a_checks import (BRIDGE_LOG, BRIDGE_STATUS, STILL_SPEED_M_S, Ctx, counter_delta, criterion, emulator_counters,
                         emulator_slow, emulator_stalls, entries, false_detections, load_json, odom_rows, path_length,
                         provenance, rtf_summary, truth_rows, verdict, write_json)
from .p6a_client import DriveLoop, read_jsonl

RESULT_KIND = '자리값 기반 SIM 결과'
SCHEMA = 'ubts.steer_verify.v1'


def wait_health(timeout_s=90.):
    end = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < end:
        try:
            last = load_json('/runtime/simulation_status.json')
            if time.time() - last['updated_at'] < 5 and all(last['sensors_alive'].values()):
                return True, last
        except (OSError, ValueError, KeyError):
            pass
        time.sleep(.5)
    return False, last


def rear_axle_track(rows, rear_x):
    return [(r[0], r[1] + rear_x * math.cos(r[3]), r[2] + rear_x * math.sin(r[3]), r[3], r[4], r[5]) for r in rows]


def fit_circle(points):
    """Algebraic (Kasa) circle fit of (x, y) points -> (cx, cy, r, rms residual)."""
    import numpy as np
    pts = np.asarray(points, dtype=float)
    x, y = pts[:, 0], pts[:, 1]
    a = np.column_stack([x, y, np.ones_like(x)])
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    cx, cy = sol[0] / 2., sol[1] / 2.
    r = math.sqrt(max(0., sol[2] + cx * cx + cy * cy))
    resid = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - r) ** 2)))
    return float(cx), float(cy), float(r), resid


def circle(ctx, profile, throttle, steer_mrad, seconds):
    ctx.ensure_armed()
    t0 = ctx.obs.sim_time
    loop = DriveLoop(ctx.client, throttle, steer_mrad).start()
    measured = []
    try:
        end = t0 + seconds
        while ctx.obs.sim_time < end:
            ctx.obs.spin_for(.5)
            status = ctx.host()
            if status.get('steering', {}).get('measured_mrad') is not None:
                measured.append(status['steering']['measured_mrad'])
    finally:
        loop.stop()
    ctx.client.stop('brake')
    ctx.wait_still(10.)
    rows = rear_axle_track(odom_rows(ctx.obs, t0 + 8., t0 + seconds), float(profile.value('rear_axle_x_in_base_m')))
    truth = truth_rows(ctx.obs, t0 + 8., t0 + seconds)
    wheelbase = float(profile.value('wheelbase_m'))
    delta_cmd = steer_mrad / 1000.
    delta_true = statistics.mean(r['delta_eq_rad'] for r in truth) if truth else None
    result = dict(throttle=throttle, steer_mrad=steer_mrad, samples=len(rows), truth_samples=len(truth),
                  loop_errors=dict(loop.errors))
    if len(rows) > 20:
        cx, cy, radius, resid = fit_circle([(r[1], r[2]) for r in rows])
        yaw_rate = statistics.mean(r[5] for r in rows)
        speed = statistics.mean(r[4] for r in rows)
        r_cmd = wheelbase / math.tan(abs(delta_cmd))
        r_true = wheelbase / math.tan(abs(delta_true)) if delta_true else None
        result.update(radius_m=radius, fit_rms_m=resid, r_geom_cmd_m=r_cmd, r_geom_true_m=r_true,
                      ratio_cmd=radius / r_cmd, ratio_true=None if r_true is None else radius / r_true,
                      yaw_rate_rad_s=yaw_rate, speed_m_s=speed, delta_true_deg=None if delta_true is None else
                      math.degrees(delta_true), delta_cmd_deg=math.degrees(delta_cmd),
                      host_measured_mrad_median=statistics.median(measured) if measured else None)
    return result


def run(args):
    from .vehicle_profile import load_profile
    profile = load_profile(args.profile)
    out = Path(args.out)
    ctx = Ctx(out, owner='verify-steer')
    criteria, report = [], dict(schema=SCHEMA, result_kind=RESULT_KIND, field_validated=False,
                                started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                                scene=os.environ.get('UBTS_STEER_SCENE'), clock=os.environ.get('UBTS_STEER_CLOCK'),
                                source_commit=os.environ.get('UBTS_STEER_SOURCE_COMMIT'),
                                source_status_sha256=os.environ.get('UBTS_STEER_SOURCE_STATUS_SHA256'),
                                provenance=provenance())
    counters0 = emulator_counters(ctx.hooks)
    mark0 = ctx.mark('verify_start')
    deadman_mark = None
    try:
        healthy, status = wait_health()
        criteria.append(criterion('health_all_sensors_alive', None if status is None else status['sensors_alive'],
                                  'all true', healthy))
        s2_present = status is not None and 'scan' in (status.get('sensors_alive') or {})
        report['scan_contract'] = None if status is None else status.get('scan_contract')
        # sensors
        from sensor_msgs.msg import Imu, LaserScan, NavSatFix
        from rclpy.qos import qos_profile_sensor_data
        seen = dict(imu=[], scan=[], fix=[], s2=[])
        ctx.obs.node.create_subscription(Imu, '/imu/data_raw', lambda m: seen['imu'].append(
            (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.header.frame_id, m.orientation_covariance[0])),
            qos_profile_sensor_data)
        ctx.obs.node.create_subscription(LaserScan, '/sim/scan', lambda m: seen['scan'].append(len(m.ranges)),
                                         qos_profile_sensor_data)
        ctx.obs.node.create_subscription(LaserScan, '/scan', lambda m: seen['s2'].append(
            (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, len(m.ranges), m.header.frame_id)),
            qos_profile_sensor_data)
        ctx.obs.node.create_subscription(NavSatFix, '/gps/fix', lambda m: seen['fix'].append(
            (m.latitude, m.longitude)), qos_profile_sensor_data)
        ctx.obs.spin_sim_for(5.)
        # /scan of the S2 format node (WF-4; exec_checks 'scan' checks every A5 item): 3240 beams, frame laser, about
        # 10 Hz of SIM time; without an S2 node (--s2-node none) /scan is absent as in P6a (report)
        s2 = seen['s2']
        s2_hz = (len(s2) - 1) / (s2[-1][0] - s2[0][0]) if len(s2) > 2 and s2[-1][0] > s2[0][0] else None
        if s2_present:
            criteria.append(criterion('scan_from_s2_node', None if not s2 else dict(beams=s2[-1][1], frame=s2[-1][2],
                                                                                    sim_hz=s2_hz),
                                      '3240 beams, frame laser, 10 Hz +-10 %',
                                      bool(s2) and s2[-1][1] == 3240 and s2[-1][2] == 'laser' and s2_hz is not None
                                      and abs(s2_hz - 10.) <= 1.))
        else:
            criteria.append(criterion('scan_absent_without_s2_node', len(s2), 'absent (--s2-node none)', None))
        imu = seen['imu']
        imu_hz = (len(imu) - 1) / (imu[-1][0] - imu[0][0]) if len(imu) > 2 and imu[-1][0] > imu[0][0] else None
        criteria.append(criterion('imu_rate_hz', imu_hz, '%s +-10 %%' % profile.value('imu_rate_hz'),
                                  imu_hz is not None and abs(imu_hz / float(profile.value('imu_rate_hz')) - 1.) <= .1))
        criteria.append(criterion('imu_frame_and_orientation_not_provided', imu[-1][1:] if imu else None,
                                  ['imu_link', -1.0], bool(imu) and imu[-1][1] == 'imu_link' and imu[-1][2] == -1.))
        criteria.append(criterion('sim_scan_3200_beams', seen['scan'][-1] if seen['scan'] else None, 3200,
                                  bool(seen['scan']) and seen['scan'][-1] == 3200))
        fix = seen['fix'][-1] if seen['fix'] else None
        criteria.append(criterion('virtual_gps_near_0N_0E', fix, '|lat|, |lon| < 0.01 deg',
                                  fix is not None and abs(fix[0]) < .01 and abs(fix[1]) < .01))
        pubs = {t: sorted({i.node_name for i in ctx.obs.node.get_publishers_info_by_topic(t)})
                for t in ('/tf', '/odom', '/tf_static', '/scan')}
        criteria.append(criterion('no_tf_no_odom_publisher', pubs, '/tf and /odom without publishers',
                                  not pubs['/tf'] and not pubs['/odom'] and bool(pubs['/tf_static'])))
        report['publishers'] = pubs
        # circles
        report['circle_left'] = left = circle(ctx, profile, args.throttle, args.steer_mrad, args.circle_s)
        report['circle_right'] = right = circle(ctx, profile, args.throttle, -args.steer_mrad, args.circle_s)
        for name, run_ in (('left', left), ('right', right)):
            sign = 1 if run_['steer_mrad'] > 0 else -1
            criteria.append(criterion('circle_%s_yaw_rate_sign' % name, run_.get('yaw_rate_rad_s'), '%+d' % sign,
                                      run_.get('yaw_rate_rad_s') is not None and run_['yaw_rate_rad_s'] * sign > 0))
            criteria.append(criterion('circle_%s_radius_ratio_true' % name, run_.get('ratio_true'),
                                      'report (A1(G) force mode about 1.03 at 10 deg)', None))
            criteria.append(criterion('circle_%s_fit_rms_m' % name, run_.get('fit_rms_m'), '< 0.05',
                                      run_.get('fit_rms_m') is not None and run_['fit_rms_m'] < .05))
        # STOP while driving straight
        ctx.ensure_armed()
        loop = DriveLoop(ctx.client, args.throttle, 0).start()
        try:
            ctx.obs.spin_sim_for(4.)
            mark = ctx.mark('verify_stop')
            t_stop = ctx.obs.sim_time
            stop_reply = ctx.client.stop('brake')
        finally:
            loop.stop()
        confirmed = ctx.wait_host(lambda s: s['steering_host']['stop_state'] == 'confirmed', 2.) is not None
        still = ctx.wait_still(8.)
        t_still = ctx.obs.sim_time
        ctx.obs.spin_sim_for(3.)
        emu, _ = ctx.logs(mark)
        after = odom_rows(ctx.obs, t_still, t_still + 3.)
        restarted = [e for e in entries(emu, 'pin') if e['channel'] in ('drive_l', 'drive_r') and e['value'] != 0]
        hold = truth_rows(ctx.obs, t_stop + .1, t_stop + 1.9)
        hold_dev = max((abs(r['delta_eq_rad'] - hold[0]['delta_eq_rad']) for r in hold), default=None)
        status = ctx.host()
        report['stop'] = dict(reply=stop_reply, stop_distance_m=path_length(odom_rows(ctx.obs, t_stop, t_still)),
                              drift_after_m=path_length(after), host_state=status.get('state'),
                              host_reason=status.get('reason'), hold_dev_deg=None if hold_dev is None else
                              math.degrees(hold_dev))
        criteria.append(criterion('stop_confirmed_by_host', confirmed, True, confirmed))
        criteria.append(criterion('vehicle_stands_after_stop', still, True, still))
        criteria.append(criterion('still_3s_after_stop_m', report['stop']['drift_after_m'], '< 0.01',
                                  report['stop']['drift_after_m'] < .01))
        criteria.append(criterion('no_drive_output_after_stop', len(restarted), 0, not restarted))
        criteria.append(criterion('host_disarmed_operator_stop', [status.get('state'), status.get('reason')],
                                  ['disarmed', 'operator_stop'], status.get('state') == 'disarmed'
                                  and status.get('reason') == 'operator_stop'))
        # deadman (input deadline)
        ctx.ensure_armed()
        deadman_mark = ctx.mark('verify_deadman')
        ctx.client.drive(30, 0)
        expired = ctx.wait_host(lambda s: s.get('reason') == 'input_expired', 3.)
        ctx.wait_still(8.)
        report['deadman'] = dict(reason=None if expired is None else expired.get('reason'),
                                 state=None if expired is None else expired.get('state'))
        criteria.append(criterion('deadman_input_expired', report['deadman']['reason'], 'input_expired',
                                  expired is not None))
    except Exception as exc:
        report['error'] = '%s: %s' % (type(exc).__name__, exc)
        import traceback
        report['traceback'] = traceback.format_exc()
        criteria.append(criterion('no_exception', report['error'], None, False))
    try:
        emu, _ = ctx.logs(mark0)
        expected = () if deadman_mark is None else (('mcu', 'TTL', deadman_mark['tick'], None),)
        bridge = read_jsonl(BRIDGE_LOG)               # the whole log: latches held before the run are not new
        report['false_detection'] = false_detections(emu, bridge, expected=expected, after_tick=mark0['tick'],
                                                     after_wall=mark0['wall'] - .01, stalls=emulator_stalls(),
                                                     slow=emulator_slow(),
                                                     clock=os.environ.get('UBTS_STEER_CLOCK', 'wall'))
        criteria.append(criterion('normal_script_false_detections', report['false_detection']['by_code'],
                                  'report (A19(G) judged in the a19g_normal summary)', None))
        report['emulator_clock'] = counter_delta(counters0, emulator_counters(ctx.hooks))
        report['rtf'] = rtf_summary(ctx.obs.clock_rows)
    except Exception as exc:          # a report block: never turn a verify result into an error
        report['false_detection_error'] = '%s: %s' % (type(exc).__name__, exc)
    finally:
        ctx.close()
    try:
        report['bridge_loop'] = load_json(BRIDGE_STATUS).get('bridge_loop')
    except (OSError, ValueError):
        pass
    report.update(criteria=criteria, passed=verdict(criteria))
    write_json(out / 'VERIFY_STEER.json', report)
    print(json.dumps(dict(passed=report['passed'], failed=[c['name'] for c in criteria if c['passed'] is False])))
    return 0 if report['passed'] else 1


def build_parser():
    p = argparse.ArgumentParser(prog='verify_steer', description=__doc__.splitlines()[0])
    p.add_argument('--out', required=True)
    p.add_argument('--profile', default=None)
    p.add_argument('--circle-s', type=float, default=20.)
    p.add_argument('--throttle', type=int, default=40)
    p.add_argument('--steer-mrad', type=int, default=175)
    return p


def main(argv=None):
    return run(build_parser().parse_args(argv))


if __name__ == '__main__':
    sys.exit(main())
