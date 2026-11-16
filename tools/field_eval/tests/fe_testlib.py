"""Synthetic data for the field_eval tests (no real data, no GNSS): a W0-like lane of posts, a known trajectory,
S2-format scans ray-cast from it (raw beam order, the 179.08 deg mount offset), raw IMU samples and executor JSONL
records."""
import gzip
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
REPO = os.path.dirname(TOOLS)
for _p in (TOOLS,):
    if _p not in sys.path:
        sys.path.insert(0, _p)
sys.dont_write_bytecode = True

RAW_AMIN = -3.1415927410125732
RAW_INC = .0019398535368964076
BEAMS = 3240
OFFSET_DEG = 179.08
LIDAR_X = .18
HALF_WIDTH = 1.75
SPACING = 1.5
RADIUS = .12
X0, X1 = -4.5, 24.


def posts():
    n = int(round((X1 - X0) / SPACING)) + 1
    return [(X0 + SPACING * k, side * HALF_WIDTH, RADIUS) for k in range(n) for side in (1, -1)]


def map_doc(start_pose=(0., 0., 0.), marks=None):
    marks = posts() if marks is None else marks
    return dict(schema='ubts.field_eval.map.v1', frame='row_local', units='m', lane_rows=['right', 'left'],
                landmarks=[dict(id='P%02d' % k, row='left' if y > 0 else 'right', x=x, y=y, radius_m=r, kind='post')
                           for k, (x, y, r) in enumerate(marks)], start_pose=list(start_pose))


def write_json(path, doc):
    with open(path, 'wb') as fh:
        fh.write((json.dumps(doc, indent=1) + '\n').encode('utf-8'))
    return path


def trajectory(duration=40., dt=.1, speed=.4, t_go=2., amp=.12, wavelength=9.):
    """t, x, y, yaw, v of a smooth lane run (stands still until t_go)."""
    t = np.arange(0., duration + 1e-9, dt)
    s = np.clip(t - t_go, 0., None) * speed
    x = s
    y = amp * np.sin(2. * np.pi * s / wavelength) * (s > 0)
    dy = amp * 2. * np.pi / wavelength * np.cos(2. * np.pi * s / wavelength) * (s > 0)
    yaw = np.arctan(dy)
    v = np.where(t > t_go, speed, 0.)
    return t, x, y, yaw, v


def cast(pose, marks, rng=None, noise=0., offset_deg=OFFSET_DEG, lidar_x=LIDAR_X, max_range=30.):
    """Ranges (raw order, float32, 0 = no return) of one scan from base pose (x, y, yaw)."""
    x, y, yaw = pose
    px, py = x + lidar_x * math.cos(yaw), y + lidar_x * math.sin(yaw)
    body = RAW_AMIN + np.arange(BEAMS) * RAW_INC + math.radians(offset_deg)
    ang = yaw + body
    dx, dy = np.cos(ang), np.sin(ang)
    m = np.asarray(marks, float)
    cx, cy, r = m[:, 0] - px, m[:, 1] - py, m[:, 2]
    b = dx[:, None] * cx[None, :] + dy[:, None] * cy[None, :]
    c = (cx ** 2 + cy ** 2 - r ** 2)[None, :]
    disc = b * b - c
    hit = (disc >= 0) & (b > 0)
    t = np.where(hit, b - np.sqrt(np.where(hit, disc, 0.)), np.inf)
    out = t.min(axis=1)
    if rng is not None and noise > 0:
        out = out + rng.normal(0., noise, out.shape)
    out = np.where(np.isfinite(out) & (out < max_range), np.round(out * 1000.) / 1000., 0.)
    return out.astype(np.float32)


def scans_for(t, x, y, yaw, marks=None, noise=.01, seed=0):
    rng = np.random.default_rng(seed)
    marks = posts() if marks is None else marks
    return np.stack([cast((x[i], y[i], yaw[i]), marks, rng, noise) for i in range(len(t))])


def imu_for(t_scan, yaw, rate_hz=33., bias=.002, seed=1):
    rng = np.random.default_rng(seed)
    t = np.arange(t_scan[0], t_scan[-1] + 1e-9, 1. / rate_hz)
    w = np.gradient(np.unwrap(yaw), t_scan)
    gz = np.interp(t, t_scan, w) + bias + rng.normal(0., .002, len(t))
    return t, np.column_stack((np.zeros(len(t)), np.zeros(len(t)), gz)), np.tile([0., 0., 9.81], (len(t), 1))


def write_fsim_folder(folder, t, ranges, imu=None):
    os.makedirs(folder, exist_ok=True)
    mm = np.round(ranges.astype(np.float64) * 1000.).astype(np.uint16)
    np.savez_compressed(os.path.join(folder, 'SCANS.npz'), stamp_s=t, receipt_s=t, ranges_mm=mm,
                        angle_min=np.float64(RAW_AMIN), angle_increment=np.float64(RAW_INC),
                        range_min=np.float64(.05), range_max=np.float64(30.))
    if imu is not None:
        ti, gyro, accel = imu
        np.savez_compressed(os.path.join(folder, 'IMU.npz'), stamp_s=ti, receipt_s=ti, gyro=gyro, accel=accel)


def executor_records(t, x, y, yaw, v, t_go, t_stop, wall0=1.7e9, run=1, lat_bias=0., config_extra=None):
    """Executor JSONL records: config, start, 20 Hz ticks (T = 45 while driving), run_end. lat / head come from the
    truth (+ lat_bias): an 'executor' whose online estimate error is known."""
    config = dict(lidar_x_in_base_m=LIDAR_X, lidar_yaw_offset_deg=OFFSET_DEG, footprint_length_m=1.22,
                  footprint_width_m=.7, imu_yaw_sign=1, **(config_extra or {}))
    recs = [dict(k='event', event='config', t=0., wall=wall0, config=config, config_digest='c' * 64),
            dict(k='event', event='start', t=0., wall=wall0, run_id=run, config_digest='c' * 64)]
    tt = np.arange(t[0], t[-1] + 1e-9, .05)
    xi, yi = np.interp(tt, t, x), np.interp(tt, t, y)
    yawi, vi = np.interp(tt, t, yaw), np.interp(tt, t, v)
    for k, tk in enumerate(tt):
        drive = t_go <= tk <= t_stop
        recs.append(dict(k='tick', t=round(float(tk), 4), run=run, state='ACTIVE' if drive else 'IDLE',
                         T=45 if drive else 0, S=int(round(-yawi[k] * 1000.)), steer=dict(meas=float(-yawi[k] * 900.)),
                         v=float(vi[k]), sr=dict(vt=.4), lat=round(float(yi[k] + lat_bias), 3),
                         head=round(float(-math.degrees(yawi[k])), 2), yaw=0., corridor=True))
    recs.append(dict(k='event', event='run_end', t=float(t_stop), wall=wall0 + float(t_stop), run_id=run,
                     end_reason='row_end'))
    return recs


def write_jsonl(path, records):
    with open(path, 'wb') as fh:
        for r in records:
            fh.write((json.dumps(r, separators=(',', ':')) + '\n').encode('utf-8'))
    return path


def write_sidecar(folder, t_wall, ranges, imu=None):
    """mapping_source/sensors.jsonl.gz rows as mapping_sensor_writer.py writes them (receipt = bag_time_ns)."""
    os.makedirs(os.path.join(folder, 'mapping_source'), exist_ok=True)
    rows = []
    for ti, r in zip(t_wall, ranges):
        sec = int(math.floor(ti))
        msg = dict(header=dict(stamp=dict(sec=sec, nanosec=int(round((ti - sec) * 1e9))), frame_id='laser'),
                   angle_min=RAW_AMIN, angle_max=RAW_AMIN + RAW_INC * (BEAMS - 1), angle_increment=RAW_INC,
                   time_increment=0., scan_time=.1, range_min=.05, range_max=30.,
                   ranges=[None if v <= 0 else float(v) for v in r], intensities=[])
        rows.append((ti, dict(topic='/scan', message=msg, bag_time_ns=int(round(ti * 1e9)))))
    if imu is not None:
        for ti, g, a in zip(*imu):
            sec = int(math.floor(ti))
            msg = dict(header=dict(stamp=dict(sec=sec, nanosec=int(round((ti - sec) * 1e9))), frame_id='imu_link'),
                       angular_velocity=dict(x=g[0], y=g[1], z=g[2]), linear_acceleration=dict(x=a[0], y=a[1], z=a[2]))
            rows.append((ti, dict(topic='/imu/data_raw', message=msg, bag_time_ns=int(round(ti * 1e9)))))
    rows.sort(key=lambda r: r[0])
    with gzip.open(os.path.join(folder, 'mapping_source', 'sensors.jsonl.gz'), 'wb') as fh:
        for _, row in rows:
            fh.write((json.dumps(row, separators=(',', ':')) + '\n').encode('utf-8'))
