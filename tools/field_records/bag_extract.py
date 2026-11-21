"""Reads every message of one recorded bag (rosbag2 mcap, read-only) and writes what the checks and the figures need:
per-topic counts, rates and longest gaps, and time series of the LiDAR, IMU, steering, bridge status, executor
state, GNSS quality and camera frames.

GNSS: positions are turned into metres east/north of the recording's first valid sample at once; no latitude or
longitude is written anywhere (only those offsets, accuracy, fix type and satellite counts).
LiDAR geometry as in the executor's field configuration of that day (the same numbers the recording player uses):
beam angle + 179.08 deg is the angle from the driving direction (not mirrored), the LiDAR sits 0.1185 m ahead of the
base point, returns between 0.05 and 12 m are used.

out/<id>.json   topic statistics and summary numbers
out/<id>.npz    time series (times are seconds since the recording's start in metadata.json)
stills/<id>_<k>.jpg   five camera frames (10, 30, 50, 70, 90 % of the frames), turned upright (the camera is mounted
                upside down: rotation=180 in the frame format)
Folders come from the environment (nothing of this is in the repository):
  UBTS_FIELD_RECORDINGS   folder with one sub-folder per recording (<id>/metadata.json, <id>/rosbag2/rosbag2_0.mcap)
  UBTS_FIELD_OUT          folder for out/ and stills/ (created)
Needs: numpy, opencv-python, mcap, mcap-ros2-support.
usage: python -B bag_extract.py <recording id> [<recording id> ...]
"""
import json
import math
import os
import struct
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from mcap.records import Channel, Message, Schema
from mcap.stream_reader import StreamReader
from mcap_ros2.decoder import DecoderFactory

ROOT = Path(os.environ['UBTS_FIELD_RECORDINGS'])
WORK = Path(os.environ['UBTS_FIELD_OUT'])
OUT = WORK / 'out'
STILLS = WORK / 'stills'
LIDAR_YAW_OFFSET = math.radians(179.08)
LIDAR_X = 0.1185
RANGE_MIN, RANGE_MAX = 0.05, 12.0
BEAM_STEP = 4                      # every fourth beam is kept for the maps (810 of 3240)
EARTH = 6378137.0


def messages(fh, topics=None):
    """Every message in file order, read front to back without the index at the end (so a bag that was never closed
    is read up to its cut; the cut raises and the caller keeps what came before)."""
    schemas, channels = {}, {}
    for record in StreamReader(fh, record_size_limit=None).records:
        if isinstance(record, Schema):
            schemas[record.id] = record
        elif isinstance(record, Channel):
            channels[record.id] = record
        elif isinstance(record, Message):
            channel = channels.get(record.channel_id)
            if channel is None or (topics and channel.topic not in topics):
                continue
            yield schemas.get(channel.schema_id), channel, record


def camera_header(data):
    """(stamp s, format, offset of the JPEG, its length) of a CompressedImage without decoding the whole message."""
    sec, nsec, n = struct.unpack_from('<iII', data, 4)
    at = 4 + 12 + n
    at += (-(at - 4)) % 4
    (m,) = struct.unpack_from('<I', data, at)
    fmt = bytes(data[at + 4:at + 4 + m]).rstrip(b'\0').decode('utf-8', 'replace')
    at = at + 4 + m
    at += (-(at - 4)) % 4
    (size,) = struct.unpack_from('<I', data, at)
    return sec + nsec * 1e-9, fmt, at + 4, size


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def extract(name):
    folder = ROOT / name
    meta = json.loads((folder / 'metadata.json').read_bytes().decode('utf-8'))
    start = datetime.fromisoformat(meta['started_at']).timestamp()
    path = folder / 'rosbag2' / 'rosbag2_0.mcap'
    began = time.time()
    factory = DecoderFactory()
    decoders = {}
    count, size, first, last, gap, prev = Counter(), Counter(), {}, {}, defaultdict(float), {}
    gap_at = {}
    scan_t, scan_left, scan_right, scan_front, scan_pts, scan_stamp = [], [], [], [], [], []
    beam = None
    imu_t, imu_yaw, imu_wz, imu_stamp = [], [], [], []
    st_t, st_cmd, st_meas, st_raw, st_pwm, st_peak, st_thr, st_ttl_stops, st_des_l, st_ack_l = ([] for _ in range(10))
    rs_t, rs_state, rs_err, rs_faults, rs_conn = [], [], [], [], []
    st_stamp = []
    ex_t, ex = [], defaultdict(list)
    gp_t, gp_e, gp_n, gp_hacc, gp_sat, gp_fix, gp_speed, gp_valid = ([] for _ in range(8))
    origin = None
    cam_t, cam_stamp, cam_bytes = [], [], []
    cam_format = None
    cam_offsets = []
    decode_errors = Counter()
    truncated = None
    with open(path, 'rb') as fh:
        try:
            for schema, channel, message in messages(fh):
                topic = channel.topic
                t = message.log_time / 1e9 - start
                count[topic] += 1
                size[topic] += len(message.data)
                first.setdefault(topic, t)
                if topic in prev and t - prev[topic] > gap[topic]:
                    gap[topic] = t - prev[topic]
                    gap_at[topic] = prev[topic]
                prev[topic] = t
                last[topic] = t
                if topic == '/camera/recording/jpeg':
                    try:
                        stamp, fmt, at, n = camera_header(message.data)
                    except struct.error:
                        decode_errors[topic] += 1
                        continue
                    cam_format = cam_format or fmt
                    cam_t.append(t)
                    cam_stamp.append(stamp - start)
                    cam_bytes.append(n)
                    cam_offsets.append((message.log_time, at, n))
                    continue
                if topic not in ('/scan', '/imu/data', '/rover/steering/state', '/rover/status', '/row_executor/state', '/gps/data'):
                    continue
                if topic not in decoders:
                    decoders[topic] = factory.decoder_for(channel.message_encoding, schema)
                try:
                    ros = decoders[topic](message.data)
                except Exception:
                    decode_errors[topic] += 1
                    continue
                if topic == '/scan':
                    r = np.asarray(ros.ranges, dtype=np.float32)
                    if beam is None or len(beam[0]) != len(r):
                        a = ros.angle_min + np.arange(len(r)) * ros.angle_increment + LIDAR_YAW_OFFSET
                        beam = (np.cos(a).astype(np.float32), np.sin(a).astype(np.float32))
                    ok = np.isfinite(r) & (r >= RANGE_MIN) & (r <= RANGE_MAX)
                    x = r * beam[0] + LIDAR_X
                    y = r * beam[1]
                    near = ok & (np.abs(x) < 3.0)
                    left = y[near & (y > 0.6) & (y < 6.0)]
                    right = -y[near & (y < -0.6) & (y > -6.0)]
                    ahead = x[ok & (np.abs(y) < 0.45) & (x > 0.3)]
                    scan_t.append(t)
                    scan_stamp.append(ros.header.stamp.sec + ros.header.stamp.nanosec * 1e-9 - start)
                    scan_left.append(np.percentile(left, 10) if len(left) >= 8 else np.nan)
                    scan_right.append(np.percentile(right, 10) if len(right) >= 8 else np.nan)
                    scan_front.append(ahead.min() if len(ahead) else np.nan)
                    keep = np.zeros(len(r), dtype=bool)
                    keep[::BEAM_STEP] = True
                    keep &= ok
                    pts = np.full((len(r[::BEAM_STEP]), 2), np.nan, dtype=np.float16)
                    sel = keep[::BEAM_STEP]
                    pts[sel, 0] = x[::BEAM_STEP][sel]
                    pts[sel, 1] = y[::BEAM_STEP][sel]
                    scan_pts.append(pts)
                elif topic == '/imu/data':
                    imu_t.append(t)
                    imu_stamp.append(ros.header.stamp.sec + ros.header.stamp.nanosec * 1e-9 - start)
                    imu_yaw.append(yaw_of(ros.orientation))
                    imu_wz.append(ros.angular_velocity.z)
                elif topic == '/gps/data':
                    try:
                        d = json.loads(ros.data)
                    except ValueError:
                        decode_errors[topic] += 1
                        continue
                    lat, lon = d.get('latitude'), d.get('longitude')
                    valid = bool(d.get('valid')) and isinstance(lat, (int, float)) and isinstance(lon, (int, float))
                    if valid and origin is None:
                        origin = (math.radians(lat), math.radians(lon))
                    gp_t.append(t)
                    gp_valid.append(valid)
                    if valid:
                        gp_e.append((math.radians(lon) - origin[1]) * math.cos(origin[0]) * EARTH)
                        gp_n.append((math.radians(lat) - origin[0]) * EARTH)
                    else:
                        gp_e.append(np.nan)
                        gp_n.append(np.nan)
                    gp_hacc.append(d.get('h_acc_m') if isinstance(d.get('h_acc_m'), (int, float)) else np.nan)
                    gp_sat.append(d.get('satellites') if isinstance(d.get('satellites'), (int, float)) else np.nan)
                    gp_fix.append(str(d.get('fix_type')))
                    gp_speed.append(d.get('speed_mps') if isinstance(d.get('speed_mps'), (int, float)) else np.nan)
                else:
                    try:
                        d = json.loads(ros.data)
                    except ValueError:
                        decode_errors[topic] += 1
                        continue
                    if topic == '/rover/steering/state':
                        s, q, drv = d.get('steering') or {}, d.get('requested') or {}, d.get('drive') or {}
                        st_t.append(t)
                        st_stamp.append(d['stamp'] - start if isinstance(d.get('stamp'), (int, float)) else np.nan)
                        st_cmd.append(q.get('steer_mrad', np.nan))
                        st_thr.append(q.get('throttle', np.nan))
                        st_meas.append(s.get('measured_mrad') if s.get('measured_mrad') is not None else np.nan)
                        st_raw.append(s.get('raw') if s.get('raw') is not None else np.nan)
                        st_pwm.append(s.get('pwm') if s.get('pwm') is not None else np.nan)
                        st_peak.append(s.get('pwm_peak') if s.get('pwm_peak') is not None else np.nan)
                        st_ttl_stops.append(drv.get('ttl_stops', np.nan))
                        st_des_l.append((drv.get('left') or {}).get('desired', np.nan))
                        st_ack_l.append((drv.get('left') or {}).get('acked', np.nan))
                    elif topic == '/rover/status':
                        rs_t.append(t)
                        rs_state.append(str(d.get('state')))
                        rs_err.append(str(d.get('last_error')))
                        rs_faults.append(','.join(str(f) for f in ((d.get('steering') or {}).get('faults') or [])))
                        rs_conn.append(bool(d.get('connected')) and bool(d.get('serial_open')))
                    else:
                        ex_t.append(t)
                        ex['stamp'].append(d['stamp'] - start if isinstance(d.get('stamp'), (int, float)) else np.nan)
                        pose = d.get('pose') or {}
                        for key in ('dist_m', 'lat_m', 'head_deg', 'speed_mps', 'target_speed_mps', 'throttle_pct', 'steer_cmd_mrad', 'steer_meas_mrad', 'width_m', 'front_m'):
                            v = d.get(key)
                            ex[key].append(v if isinstance(v, (int, float)) else np.nan)
                        for key, src in (('x', 'x'), ('y', 'y'), ('yaw_deg', 'yaw_deg')):
                            v = pose.get(src)
                            ex['pose_' + key].append(v if isinstance(v, (int, float)) else np.nan)
                        for key in ('state', 'reason', 'run_state', 'stop_reason', 'phase'):
                            ex[key].append(str(d.get(key)))
                        ex['corridor_ok'].append(bool(d.get('corridor_ok')))
        except Exception as err:
            # an unclosed bag ends in a cut chunk: everything before it has been read
            truncated = '%s: %s' % (type(err).__name__, str(err)[:120])
        stills = []
        if cam_offsets:
            STILLS.mkdir(exist_ok=True)
            wanted = {int(len(cam_offsets) * f): k for k, f in enumerate((0.1, 0.3, 0.5, 0.7, 0.9))}
            fh.seek(0)
            index = 0
            try:
                for schema, channel, message in messages(fh, topics=('/camera/recording/jpeg',)):
                    if index in wanted:
                        _, at, n = cam_offsets[index]
                        image = cv2.imdecode(np.frombuffer(message.data[at:at + n], dtype=np.uint8), cv2.IMREAD_COLOR)
                        if image is not None:
                            if 'rotation=180' in (cam_format or ''):
                                image = cv2.rotate(image, cv2.ROTATE_180)
                            target = STILLS / ('%s_%d.jpg' % (name[9:15], wanted[index]))
                            cv2.imwrite(str(target), image, [cv2.IMWRITE_JPEG_QUALITY, 88])
                            stills.append({'file': target.name, 't_s': round(cam_t[index], 2), 'size': list(image.shape[:2])})
                    index += 1
                    if index > max(wanted):
                        break
            except Exception:
                pass
    topics = {}
    for topic in sorted(count):
        span = last[topic] - first[topic]
        topics[topic] = {'n': count[topic], 'hz': round((count[topic] - 1) / span, 2) if span > 0 else None, 'first_s': round(first[topic], 2),
                         'last_s': round(last[topic], 2), 'max_gap_s': round(gap[topic], 3), 'max_gap_at_s': round(gap_at.get(topic, float('nan')), 2) if topic in gap_at else None,
                         'mb': round(size[topic] / 1e6, 1)}
    OUT.mkdir(exist_ok=True)
    arrays = dict(
        scan_t=np.array(scan_t), scan_stamp=np.array(scan_stamp), imu_stamp=np.array(imu_stamp), scan_left=np.array(scan_left, dtype=np.float32), scan_right=np.array(scan_right, dtype=np.float32),
        scan_front=np.array(scan_front, dtype=np.float32), scan_pts=np.array(scan_pts, dtype=np.float16) if scan_pts else np.zeros((0, 0, 2), np.float16),
        imu_t=np.array(imu_t), imu_yaw=np.array(imu_yaw, dtype=np.float32), imu_wz=np.array(imu_wz, dtype=np.float32),
        st_t=np.array(st_t), st_stamp=np.array(st_stamp), st_cmd=np.array(st_cmd, dtype=np.float32), st_meas=np.array(st_meas, dtype=np.float32), st_raw=np.array(st_raw, dtype=np.float32),
        st_pwm=np.array(st_pwm, dtype=np.float32), st_peak=np.array(st_peak, dtype=np.float32), st_thr=np.array(st_thr, dtype=np.float32),
        st_ttl_stops=np.array(st_ttl_stops, dtype=np.float32), st_des_l=np.array(st_des_l, dtype=np.float32), st_ack_l=np.array(st_ack_l, dtype=np.float32),
        rs_t=np.array(rs_t), rs_state=np.array(rs_state), rs_err=np.array(rs_err), rs_faults=np.array(rs_faults), rs_conn=np.array(rs_conn),
        ex_t=np.array(ex_t), gp_t=np.array(gp_t), gp_e=np.array(gp_e, dtype=np.float32), gp_n=np.array(gp_n, dtype=np.float32),
        gp_hacc=np.array(gp_hacc, dtype=np.float32), gp_sat=np.array(gp_sat, dtype=np.float32), gp_fix=np.array(gp_fix), gp_speed=np.array(gp_speed, dtype=np.float32),
        gp_valid=np.array(gp_valid), cam_t=np.array(cam_t), cam_stamp=np.array(cam_stamp), cam_bytes=np.array(cam_bytes))
    for key, values in ex.items():
        arrays['ex_' + key] = (np.array(values) if key in ('state', 'reason', 'run_state', 'stop_reason', 'phase', 'corridor_ok')
                               else np.array(values, dtype=np.float64 if key == 'stamp' else np.float32))
    np.savez_compressed(OUT / (name + '.npz'), **arrays)
    summary = {'id': name, 'started_at': meta['started_at'], 'duration_s': meta.get('duration_s'), 'state': meta.get('state'), 'bag_mb': round(path.stat().st_size / 1e6),
               'read_s': round(time.time() - began, 1), 'truncated': truncated, 'decode_errors': dict(decode_errors), 'topics': topics,
               'camera_format': cam_format, 'stills': stills}
    (OUT / (name + '.json')).write_bytes(json.dumps(summary, ensure_ascii=False, indent=1).encode('utf-8'))
    return summary


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    for name in sys.argv[1:]:
        s = extract(name)
        t = s['topics']
        print('%s %5d MB read %5.1f s | scan %s Hz gap %s | cam %s fps gap %s | imu %s Hz gap %s | steer %s Hz gap %s | exec %s Hz gap %s | cut %s | errors %s' % (
            name[9:15], s['bag_mb'], s['read_s'], t.get('/scan', {}).get('hz'), t.get('/scan', {}).get('max_gap_s'),
            t.get('/camera/recording/jpeg', {}).get('hz'), t.get('/camera/recording/jpeg', {}).get('max_gap_s'),
            t.get('/imu/data', {}).get('hz'), t.get('/imu/data', {}).get('max_gap_s'),
            t.get('/rover/steering/state', {}).get('hz'), t.get('/rover/steering/state', {}).get('max_gap_s'),
            t.get('/row_executor/state', {}).get('hz'), t.get('/row_executor/state', {}).get('max_gap_s'), s['truncated'], s['decode_errors']), flush=True)


main()
