"""Reading layer (PW-02): one function opens a 10/3 or a 10/10 recording the same way.

Motor timeline: field_calib.records (commit 50fc854) - motor_from_jsonl for the 10/3 full extracts (layout
'full', the original motor.jsonl), motor_from_compact for the 10/10 compact drive extracts (layout 'compact').
For 10/3 the compact drive extract of the same recording is also read and its motor rows are compared with the
original motor.jsonl (same count, same receipt times) - a check, the analysis uses motor.jsonl.

LiDAR motion: the existing offline ICP result <rid>.lidar_motion.npz (t = scan receipt time on the host clock,
pose x, y, theta in the chain frame, quality columns inlier, rms, -, cond, -). This is the same input as the
published 10/10 numbers (LIDAR_RESPONSE.json, start_onset_1010.json, from_release.out.txt); the scans are not
matched again here. records.open_compact loads the same file as `reference_motion`.

GNSS: only receipt time, receiver time, valid flag, speed, speed accuracy and the epoch id are read from the
drive extract. The position columns are never read.

Cartographer: trajectory.csv nodes (map-frame metres, scan_index into the recording's scans) are read only for
the in-row test and the path-length speed; scan_index -> host time through lidar_motion t.
"""
import csv
import datetime as dt
import gzip
import json
import math
import os

import numpy as np

import common
from field_calib import records as fc_records

GNSS_KEYS = ('received_at', 'gps_time', 'valid', 'speed_mps', 'speed_acc', 'i_tow_ms', 'fix_type')


class Recording:
    def __init__(self, rid, day, ground, analysed, evidence):
        self.rid = rid
        self.short = rid[9:15]
        self.day = day
        self.ground = ground
        self.analysed = analysed
        self.evidence = evidence
        self.motor = None              # field_calib.records.Motor
        self.motor_layout = None
        self.metadata = {}
        self.lidar = None              # dict t, pose, quality
        self.gnss = []                 # list of dicts (GNSS_KEYS only)
        self.imu = np.zeros((0, 8))
        self.traj = None               # dict t_host, x_map, y_map, scan_index
        self.sources = {}              # relative path -> sha256
        self.checks = {}
        self.warnings = []
        self.receiver_minus_host_s = None

    # --- time ------------------------------------------------------------------------------------
    def to_kst(self, t_host):
        """Host receipt time -> KST datetime through the receiver clock (gps_time); None when unknown."""
        if self.receiver_minus_host_s is None or t_host is None or not math.isfinite(t_host):
            return None
        utc = dt.datetime.fromtimestamp(t_host + self.receiver_minus_host_s, tz=dt.timezone.utc)
        return utc.astimezone(dt.timezone(dt.timedelta(hours=9)))


def _gps_seconds(text):
    if not text:
        return None
    try:
        return dt.datetime.fromisoformat(text.replace('Z', '+00:00')).timestamp()
    except ValueError:
        return None


def _read_drive(path):
    with gzip.open(path, 'rt', encoding='utf-8') as fh:
        return json.load(fh)


def _hash(rec, cfg, path):
    rec.sources[cfg.relative(path)] = common.sha256_file(path)


def open_recording(cfg, entry):
    """Open one recording listed in recordings.json. Raises on a missing motor timeline."""
    rec = Recording(entry['rid'], entry['day'], entry['ground'], entry.get('analysed', True), entry.get('evidence', ''))
    rid = rec.rid
    if rec.day == '1003':
        full_dir = os.path.join(cfg.path_of('day1003_full_extract'), rid)
        motor_path = os.path.join(full_dir, 'motor.jsonl')
        rec.motor = fc_records.motor_from_jsonl(motor_path)
        rec.motor_layout = 'full (records.motor_from_jsonl)'
        rec.metadata = fc_records._metadata(os.path.join(full_dir, 'metadata.json'))
        _hash(rec, cfg, motor_path)
        drive_path = os.path.join(cfg.path_of('day1003_drive_extract'), rid + '.drive.json.gz')
        lidar_dir = cfg.path_of('day1003_lidar')
    else:
        drive_path = os.path.join(cfg.path_of('day1010_drive_extract'), rid + '.drive.json.gz')
        lidar_dir = cfg.path_of('day1010_lidar')
    drive = _read_drive(drive_path)
    _hash(rec, cfg, drive_path)
    compact_motor = fc_records.motor_from_compact(drive['motor'])
    if rec.day == '1003':
        same_n = len(compact_motor) == len(rec.motor)
        same_t = same_n and bool(np.allclose(compact_motor.t, rec.motor.t, atol=1e-6, rtol=0))
        same_req = same_n and bool(np.array_equal(np.nan_to_num(compact_motor.req_l, nan=-999),
                                                  np.nan_to_num(rec.motor.req_l, nan=-999)))
        same_rep = same_n and bool(np.allclose(np.nan_to_num(compact_motor.rep_l, nan=-999),
                                               np.nan_to_num(rec.motor.rep_l, nan=-999)))
        rec.checks['compact_motor_equals_motor_jsonl'] = dict(rows=same_n, receipt_times=same_t, requests=same_req,
                                                             reported=same_rep)
    else:
        rec.motor = compact_motor
        rec.motor_layout = 'compact (records.motor_from_compact)'
        rec.metadata = dict(drive.get('metadata') or {})
    # GNSS (no positions)
    for g in drive.get('gps') or []:
        rec.gnss.append({k: g.get(k) for k in GNSS_KEYS})
    offs = []
    for g in rec.gnss:
        ts = _gps_seconds(g.get('gps_time'))
        if ts is not None and g.get('received_at') is not None:
            offs.append(ts - g['received_at'])
    if offs:
        rec.receiver_minus_host_s = float(np.median(offs))
    # IMU rows as in records.open_compact (receipt s, header s, gyro xyz, acc xyz)
    imu = np.array(drive.get('imu') or [], dtype=np.float64)
    if len(imu):
        rec.imu = np.column_stack([imu[:, 0] / 1e9, imu[:, 1] / 1e9, imu[:, 2:8]])
        rec.imu = rec.imu[np.argsort(rec.imu[:, 0], kind='stable')]
    # LiDAR motion
    lm_path = os.path.join(lidar_dir, rid + '.lidar_motion.npz')
    if os.path.isfile(lm_path):
        z = np.load(lm_path)
        rec.lidar = dict(t=np.asarray(z['t'], dtype=float), pose=np.asarray(z['pose'], dtype=float),
                         quality=np.asarray(z['quality'], dtype=float))
        _hash(rec, cfg, lm_path)
    else:
        rec.warnings.append('no lidar_motion.npz')
    # Cartographer trajectory (map-frame metres; positions only used for the in-row test)
    traj_path = os.path.join(cfg.path_of('cartographer_maps'), rec.short, 'trajectory.csv')
    yaml_path = os.path.join(cfg.path_of('cartographer_maps'), rec.short, '%s_offline.yaml' % rec.short)
    if rec.lidar is not None and os.path.isfile(traj_path) and os.path.isfile(yaml_path):
        with open(traj_path, newline='', encoding='utf-8') as fh:
            rows = list(csv.DictReader(fh))
        si = np.array([int(r['scan_index']) for r in rows], dtype=int)
        ok = (si >= 0) & (si < len(rec.lidar['t']))
        x = np.array([float(r['x_m']) for r in rows])
        y = np.array([float(r['y_m']) for r in rows])
        origin = _yaml_origin(yaml_path)
        rec.traj = dict(t_host=rec.lidar['t'][si[ok]], x_map=x[ok], y_map=y[ok], scan_index=si[ok], origin=origin,
                        dropped=int((~ok).sum()))
        _hash(rec, cfg, traj_path)
        _hash(rec, cfg, yaml_path)
    rec.checks['clock_back_steps'] = int(rec.motor.clock_back)
    rec.checks['clock_forward_jumps'] = int(rec.motor.clock_jump)
    return rec


def _yaml_origin(path):
    with open(path, 'r', encoding='utf-8') as fh:
        for line in fh:
            if line.strip().startswith('origin:'):
                inner = line.split('[', 1)[1].split(']', 1)[0]
                vals = [float(v) for v in inner.split(',')]
                return vals[0], vals[1]
    raise ValueError('origin missing in %s' % path)


def open_all(cfg, log=print):
    listing = common.load_recordings()
    recs = []
    for entry in listing['recordings']:
        rec = open_recording(cfg, entry)
        log('  opened %s: motor %d rows (%s), lidar %s, gnss %d, traj %s' % (
            rec.rid, len(rec.motor), rec.motor_layout, 'yes' if rec.lidar is not None else 'no', len(rec.gnss),
            'yes' if rec.traj is not None else 'no'))
        recs.append(rec)
    return recs, listing
