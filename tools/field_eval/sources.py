"""Scan and IMU sources, read into one form.

Scans  dict(t (s, the time base of the source), receipt (s), ranges (n, beams) float32 metres with NaN = no return,
       angle_min, angle_increment, range_min, range_max, beams, time_base, source, sha256)
Imu    dict(t (s), gyro (n, 3) rad/s, accel (n, 3) m/s^2)   (empty arrays when the source has none)

Sources (open_source detects the kind from the path):
  fsim       an F-SIM run folder written by sim_capture.py: SCANS.npz (+ IMU.npz) next to TRACE.npz / EXECUTOR.jsonl.
             The scans are the executor's real-format S2 messages (3240 beams, raw order). Time base: the SIM clock,
             which is the executor's clock there ('executor').
  sidecar    a recorder archive folder with mapping_source/sensors.jsonl.gz (dashboard data_recorder_service.py and
             mapping_sensor_writer.py; read with tools/field_calib/records.py). Time = receipt time.time() ('wall').
  rosbag2    a rosbag2 folder (or the archive folder holding rosbag2/): bag_reader.py, sqlite3 or MCAP. Time = the
             bag timestamp (receipt, time.time on the Jetson recorder; 'wall').
The time of a scan is its receipt time unless use_header=True (the driver's header stamp; same clock on the Jetson).
"""
from __future__ import annotations

import hashlib
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def _empty_imu():
    return dict(t=np.zeros(0), gyro=np.zeros((0, 3)), accel=np.zeros((0, 3)))


def detect(path):
    if os.path.isfile(os.path.join(path, 'SCANS.npz')):
        return 'fsim'
    if os.path.isfile(os.path.join(path, 'mapping_source', 'sensors.jsonl.gz')):
        return 'sidecar'
    if os.path.isdir(os.path.join(path, 'rosbag2')):
        return 'rosbag2'
    if os.path.isdir(path) and any(n.endswith('.db3') or n.endswith('.mcap') for n in os.listdir(path)):
        return 'rosbag2'
    if os.path.isfile(path) and (path.endswith('.db3') or path.endswith('.mcap')):
        return 'rosbag2'
    return None


def read_fsim(folder, use_header=True):
    path = os.path.join(folder, 'SCANS.npz')
    with np.load(path) as z:
        ranges = z['ranges_mm'].astype(np.float32) / 1000.
        ranges[ranges <= 0.] = np.nan
        t = z['stamp_s'] if use_header else z['receipt_s']
        scans = dict(t=np.asarray(t, dtype=float), receipt=np.asarray(z['receipt_s'], dtype=float), ranges=ranges,
                     angle_min=float(z['angle_min']), angle_increment=float(z['angle_increment']),
                     range_min=float(z['range_min']), range_max=float(z['range_max']), beams=ranges.shape[1],
                     time_base='executor', source='fsim', sha256={'SCANS.npz': sha256_file(path)})
    imu = _empty_imu()
    ipath = os.path.join(folder, 'IMU.npz')
    if os.path.isfile(ipath):
        with np.load(ipath) as z:
            t = z['stamp_s'] if use_header else z['receipt_s']
            imu = dict(t=np.asarray(t, dtype=float), gyro=np.asarray(z['gyro'], dtype=float),
                       accel=np.asarray(z['accel'], dtype=float))
        scans['sha256']['IMU.npz'] = sha256_file(ipath)
    return scans, imu


def read_sidecar(folder, use_header=False):
    if TOOLS not in sys.path:
        sys.path.insert(0, TOOLS)
    from field_calib import records as R
    path = os.path.join(folder, 'mapping_source', 'sensors.jsonl.gz')
    rec = R.Recording(os.path.basename(folder.rstrip('/\\')), 'jetson')
    R.read_sidecar(rec, path)
    mm = rec.ranges_mm.astype(np.float32)
    ranges = mm / 1000.
    ranges[mm == 0] = np.nan
    geo = rec.geometry or {}
    t = rec.scan_header_t if use_header else rec.scan_t
    scans = dict(t=np.asarray(t, dtype=float), receipt=np.asarray(rec.scan_t, dtype=float), ranges=ranges,
                 angle_min=float(geo.get('angle_min', 0.)), angle_increment=float(geo.get('angle_increment', 0.)),
                 range_min=float(geo.get('range_min', 0.)), range_max=float(geo.get('range_max', 0.)),
                 beams=ranges.shape[1] if ranges.ndim == 2 else 0, time_base='wall', source='sidecar',
                 sha256={'mapping_source/sensors.jsonl.gz': sha256_file(path)}, warnings=list(rec.warnings))
    imu = _empty_imu()
    if len(rec.imu):
        col = 1 if use_header else 0
        imu = dict(t=rec.imu[:, col].copy(), gyro=rec.imu[:, 2:5].copy(), accel=rec.imu[:, 5:8].copy())
    return scans, imu


def read_rosbag2(path, use_header=False, scan_topic='/scan', imu_topic='/imu/data_raw'):
    from . import bag_reader as B
    folder = os.path.join(path, 'rosbag2') if os.path.isdir(os.path.join(path, 'rosbag2')) else path
    msgs, imus = B.read_bag(folder, scan_topic, imu_topic)
    if not msgs:
        raise ValueError('%s: no %s messages' % (folder, scan_topic))
    first = msgs[0]
    beams = len(first['ranges'])
    keep = [m for m in msgs if len(m['ranges']) == beams and m['angle_min'] == first['angle_min']
            and m['angle_increment'] == first['angle_increment']]
    ranges = np.stack([m['ranges'] for m in keep]).astype(np.float32)
    ranges[~np.isfinite(ranges) | (ranges <= 0.)] = np.nan
    key = 'stamp_s' if use_header else 'receipt_s'
    files = B._bag_files(folder)
    scans = dict(t=np.array([m[key] for m in keep]), receipt=np.array([m['receipt_s'] for m in keep]), ranges=ranges,
                 angle_min=float(first['angle_min']), angle_increment=float(first['angle_increment']),
                 range_min=float(first['range_min']), range_max=float(first['range_max']), beams=beams,
                 time_base='wall', source='rosbag2', sha256={os.path.basename(f): sha256_file(f) for f in files},
                 warnings=[] if len(keep) == len(msgs) else ['scan_geometry_changed:%d' % (len(msgs) - len(keep))])
    imu = _empty_imu()
    if imus:
        imu = dict(t=np.array([m[key] for m in imus]), gyro=np.array([m['gyro'] for m in imus], dtype=float),
                   accel=np.array([m['accel'] for m in imus], dtype=float))
    return scans, imu


def open_source(path, kind=None, use_header=None):
    """(scans, imu) of a source (module docstring); kind None = detect."""
    kind = kind or detect(path)
    if kind == 'fsim':
        return read_fsim(path, True if use_header is None else use_header)
    if kind == 'sidecar':
        return read_sidecar(path, bool(use_header))
    if kind == 'rosbag2':
        return read_rosbag2(path, bool(use_header))
    raise ValueError('%s: not a known scan source (F-SIM capture, recorder archive, rosbag2)' % path)
