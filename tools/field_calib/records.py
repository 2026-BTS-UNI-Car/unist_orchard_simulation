"""Read-only recording readers (no ROS, numpy only). Three layouts, one Recording object.

  jetson   <root>/<rid>/metadata.json, motor.jsonl, mapping_source/sensors.jsonl.gz (+ manifest.json)
           = what data_recorder_service.py / mapping_sensor_writer.py write on the Jetson
           (the dashboard's runtime/recordings folder, mounted into the recorder container; the steering SIM
           writes the same layout, sim/ubts/steering/ident). Sidecar rows: {"topic", "message", "bag_time_ns"} with
           bag_time_ns = host time.time() at the recorder callback (receipt). Topics: /scan (LaserScan,
           ranges with null for non-finite), /imu/data_raw (raw_imu_payload + header), /gps/data (unused).
  full     <root>/<rid>/sensors.jsonl.gz, motor.jsonl, metadata.json: the 10/3 full extracts (rosbag MCAP
           rows {"topic", "bag_time_ns", "message"}; same field names).
  compact  <base>/lidar/<rid>.scans.npz + <drive_dir>/<rid>.drive.json.gz: the 10/10 compact
           extracts (extract_scans.py / extract_drive.py). drive_dir is given by the caller or by the
           environment variable UBTS_RD_1010_DRIVE (the repository does not name that data folder).

motor.jsonl (orchard.motor_pwm.timeline.v1) = /rover/status snapshots at ~2 Hz: received_at (host
time.time(), same clock as the sidecar), status.state/armed/moving, requested.left/right (signed %),
pins.<pin>.reported_pwm (last MCU report, 0-255). Signed reported duty: left = (D8 - D5) / 255 * 100,
right = (D3 - D0) / 255 * 100 (motor_pwm.MOTOR_DIRECTION_PINS). /rover/control/requested in the rosbag
carries the same requested values at the same 2 Hz, so the analyser segments on motor.jsonl only.

Scan ranges are kept as uint16 millimetres (0 = no return), exactly like extract_scans.py. Only scans
inside the requested time windows are parsed (the slow part is the 3240 float strings per scan).
"""
import gzip
import hashlib
import json
import math
import os
import zlib

import numpy as np

LAYOUTS = ('jetson', 'full', 'compact')
DIRECTION_PINS = dict(left=('D8', 'D5'), right=('D3', 'D0'))
SCAN_PREFIX = b'{"topic":"/scan"'
IMU_PREFIX = b'{"topic":"/imu/data_raw"'


class RecordingError(ValueError):
    pass


def sha256_file(path, limit=None):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def detect_layout(path):
    if os.path.isfile(os.path.join(path, 'mapping_source', 'sensors.jsonl.gz')):
        return 'jetson'
    if os.path.isfile(os.path.join(path, 'sensors.jsonl.gz')):
        return 'full'
    return None


def _signed(pins, a, b):
    try:
        pa, pb = pins[a], pins[b]
    except (KeyError, TypeError):
        return math.nan
    if isinstance(pa, dict):
        pa, pb = pa.get('reported_pwm'), (pb or {}).get('reported_pwm')
    if pa is None or pb is None:
        return math.nan
    return (pa - pb) / 255. * 100.


CLOCK_BACK_TOL_S = .2           # file-order step back larger than this = host clock stepped backward
CLOCK_JUMP_S = 30.              # file-order step forward larger than this = clock step (or a long pause)


def clock_steps(t_file_order, tol=CLOCK_BACK_TOL_S, jump=CLOCK_JUMP_S):
    """(backward steps, forward jumps) of receipt times in FILE order (before any sorting)."""
    t = np.asarray(t_file_order, dtype=np.float64)
    if len(t) < 2:
        return 0, 0
    d = np.diff(t)
    return int((d < -tol).sum()), int((d > jump).sum())


class Motor:
    """Motor status timeline as arrays (received time order).

    The rows are sorted by receipt time for the analysis; clock_back / clock_jump count the steps in the
    original file order (a sorted array can not show a backward clock step any more)."""

    def __init__(self, rows):
        self.clock_back, self.clock_jump = clock_steps([r['t'] for r in rows])
        rows = sorted(rows, key=lambda r: r['t'])
        self.t = np.array([r['t'] for r in rows], dtype=np.float64)
        self.req_l = np.array([np.nan if r['req_l'] is None else r['req_l'] for r in rows], dtype=np.float64)
        self.req_r = np.array([np.nan if r['req_r'] is None else r['req_r'] for r in rows], dtype=np.float64)
        self.rep_l = np.array([r['rep_l'] for r in rows], dtype=np.float64)
        self.rep_r = np.array([r['rep_r'] for r in rows], dtype=np.float64)
        self.state = [r.get('state') for r in rows]
        self.armed = np.array([bool(r.get('armed')) for r in rows])
        self.moving = np.array([bool(r.get('moving')) for r in rows])
        self.stale = np.array([bool(r.get('stale')) for r in rows])
        self.malformed = 0

    def __len__(self):
        return len(self.t)


def motor_from_jsonl(path):
    rows, malformed = [], 0
    with open(path, 'r', encoding='utf-8') as fh:
        for line in fh:
            try:
                r = json.loads(line)
                t = r['received_at']
                if not isinstance(t, (int, float)) or not math.isfinite(t):
                    raise ValueError
            except (ValueError, KeyError, TypeError):
                malformed += 1
                continue
            st = r.get('status') or {}
            rq = r.get('requested') or {}
            pins = r.get('pins') or {}
            rows.append(dict(t=float(t), req_l=rq.get('left'), req_r=rq.get('right'),
                             rep_l=_signed(pins, 'D8', 'D5'), rep_r=_signed(pins, 'D3', 'D0'),
                             state=st.get('state'), armed=st.get('armed'), moving=st.get('moving'),
                             stale=r.get('source_stale')))
    motor = Motor(rows)
    motor.malformed = malformed
    return motor


def motor_from_compact(rows):
    out = []
    for r in rows:
        rep = r.get('rep') or {}
        state = r.get('state')
        out.append(dict(t=float(r['received_at']), req_l=r.get('req_l'), req_r=r.get('req_r'),
                        rep_l=_signed(rep, 'D8', 'D5'), rep_r=_signed(rep, 'D3', 'D0'), state=state,
                        armed=state == 'armed_manual', moving=r.get('moving'), stale=r.get('source_stale')))
    return Motor(out)


class Recording:
    def __init__(self, rid, layout, label=''):
        self.rid = rid
        self.layout = layout
        self.label = label
        self.metadata = {}
        self.sources = {}
        self.warnings = []
        self.motor = None
        self.imu = np.zeros((0, 8))      # t_receipt, t_header, gx, gy, gz, ax, ay, az (rad/s, m/s^2)
        self.scan_t = np.zeros(0)        # receipt [s, host clock]
        self.scan_header_t = np.zeros(0)
        self.ranges_mm = np.zeros((0, 0), dtype=np.uint16)
        self.geometry = {}
        self.scan_total = 0              # scans in the file (parsed or not)
        self.scan_geometry_rejected = 0
        self.scan_windows_s = None       # time windows that were parsed (None = all)
        self.sidecar_manifest = None
        self.reference_motion = None     # optional precomputed lidar_motion.npz (validation only)
        self.cached_odometry = None      # (pose, quality, chain) from the cache (sidecar not read)
        self.cache_key = None
        self.read_warnings = []
        self.clock_back = 0              # backward host clock steps in file order (motor + IMU + scans)
        self.sidecar_clock = (0, 0)      # (backward, forward jumps) per-topic file order in the sidecar

    def check_clock(self):
        """Fold the file-order clock steps of the motor rows and the sidecar into clock_back + warnings."""
        back = int(getattr(self.motor, 'clock_back', 0) or 0) + int(self.sidecar_clock[0])
        jump = int(getattr(self.motor, 'clock_jump', 0) or 0) + int(self.sidecar_clock[1])
        self.clock_back = back
        if back:
            self.warnings.append('clock_backward_steps:%d (recording unusable: data order is not time order)' % back)
        if jump:
            self.warnings.append('clock_forward_jumps:%d (> %.0f s)' % (jump, CLOCK_JUMP_S))

    @property
    def duration_s(self):
        ts = [x for x in (self.motor.t if self.motor is not None and len(self.motor) else [],
                          self.imu[:, 0] if len(self.imu) else [], self.scan_t) if len(x)]
        if not ts:
            return 0.
        return float(max(np.max(x) for x in ts) - min(np.min(x) for x in ts))


def _inside(t, windows):
    if windows is None:
        return True
    for a, b in windows:
        if a <= t <= b:
            return True
    return False


def _bag_time(line):
    k = line.rfind(b'"bag_time_ns":')
    if k < 0:
        return None
    k += 14
    end = k
    while end < len(line) and line[end:end + 1] in b'0123456789':
        end += 1
    try:
        return int(line[k:end]) / 1e9
    except ValueError:
        return None


def _parse_scan(line):
    """(geometry dict, header_t, ranges float64 with nan) from one /scan sidecar line."""
    i = line.index(b'"ranges":[') + 10
    j = line.index(b']', i)
    ranges = np.array(line[i:j].replace(b'null', b'nan').split(b','), dtype=np.float64)
    rest = json.loads(line[:i - 10] + b'"ranges":[]' + line[j + 1:])
    m = rest['message']
    st = m['header']['stamp']
    geo = dict(angle_min=float(m['angle_min']), angle_increment=float(m['angle_increment']),
               range_min=float(m['range_min']), range_max=float(m['range_max']),
               scan_time=m.get('scan_time'), time_increment=m.get('time_increment'),
               frame_id=m['header'].get('frame_id'), beams=len(ranges))
    return geo, int(st['sec']) + int(st['nanosec']) / 1e9, ranges, rest.get('bag_time_ns')


def _imu_row(line):
    r = json.loads(line)
    m = r['message']
    st = m['header']['stamp'] if 'header' in m else m['stamp']
    av, la = m['angular_velocity'], m['linear_acceleration']
    return [r['bag_time_ns'] / 1e9, int(st['sec']) + int(st['nanosec']) / 1e9,
            av['x'], av['y'], av['z'], la['x'], la['y'], la['z']]


def _scan_to_mm(line):
    geo, th, a, _ = _parse_scan(line)
    key = (geo['angle_min'], geo['angle_increment'], geo['range_min'], geo['range_max'], geo['beams'])
    ok = np.isfinite(a) & (a >= geo['range_min']) & (a <= geo['range_max'])
    mm = np.zeros(a.shape, dtype=np.uint16)
    mm[ok] = np.clip(np.round(a[ok] * 1000.), 1, 65535).astype(np.uint16)
    return key, geo, th, mm


def _parse_batch(lines):
    """Worker: [(t, line)] -> [(t, key, geo, header_t, mm)]; key None = malformed row."""
    out = []
    for t, line in lines:
        try:
            key, geo, th, mm = _scan_to_mm(line)
            out.append((t, key, geo, th, mm))
        except (ValueError, KeyError, TypeError) as exc:
            out.append((t, None, str(exc)[:80], None, None))
    return out


BATCH_LINES = 96


def read_sidecar(rec, path, scan_windows=None, pool=None, workers=1, skip_scans=False):
    """Stream a sensors.jsonl.gz once: all IMU rows, scans only inside scan_windows (None = all).

    The float parsing of the selected scan lines (the slow part) runs in `pool` when one is given, in
    batches of BATCH_LINES lines with at most 2 * workers batches in flight (bounded memory); results
    are consumed in file order, so the output is identical to the serial path."""
    imu, scan_t, scan_h, rows, scan_file_t = [], [], [], [], []
    state = dict(geometry=None, key=None, rejected=0)
    pending, batch = [], []

    def consume(results):
        for t, key, geo, th, mm in results:
            if key is None:
                rec.warnings.append('scan_row_malformed')
                continue
            if state['geometry'] is None:
                state['geometry'], state['key'] = geo, key
            elif key != state['key']:
                state['rejected'] += 1
                continue
            rows.append(mm)
            scan_t.append(t)
            scan_h.append(th)

    def flush(force=False):
        if batch:
            if pool is None:
                consume(_parse_batch(list(batch)))
            else:
                pending.append(pool.submit(_parse_batch, list(batch)))
            del batch[:]
        while pending and (force or len(pending) > 2 * max(1, workers)):
            consume(pending.pop(0).result())

    total = 0
    try:
        with gzip.open(path, 'rb') as fh:
            for line in fh:
                if line.startswith(IMU_PREFIX):
                    try:
                        imu.append(_imu_row(line))
                    except (ValueError, KeyError, TypeError):
                        rec.warnings.append('imu_row_malformed')
                    continue
                if not line.startswith(SCAN_PREFIX):
                    continue
                total += 1
                if skip_scans:
                    continue                  # scans come from the odometry cache
                t = _bag_time(line)
                if t is not None:
                    scan_file_t.append(t)
                if t is None or not _inside(t, scan_windows):
                    continue
                batch.append((t, line))
                if len(batch) >= BATCH_LINES:
                    flush()
    except (EOFError, zlib.error, OSError) as exc:
        # An interrupted recording ends in a truncated gzip member: keep what was read, say so.
        rec.warnings.append('sidecar_truncated:%s' % type(exc).__name__)
    flush(force=True)
    imu = np.array(imu, dtype=np.float64).reshape(-1, 8)
    ib, ij = clock_steps(imu[:, 0])                  # per topic, file order (before sorting)
    sb, sj = clock_steps(scan_file_t)
    rec.sidecar_clock = (ib + sb, ij + sj)
    if len(imu):
        imu = imu[np.argsort(imu[:, 0], kind='stable')]
    rec.imu = imu
    order = np.argsort(np.array(scan_t), kind='stable') if scan_t else np.zeros(0, dtype=int)
    rec.scan_t = np.array(scan_t, dtype=np.float64)[order] if scan_t else np.zeros(0)
    rec.scan_header_t = np.array(scan_h, dtype=np.float64)[order] if scan_t else np.zeros(0)
    rec.ranges_mm = np.stack(rows)[order] if rows else np.zeros((0, 0), dtype=np.uint16)
    rec.geometry = state['geometry'] or {}
    rec.scan_total = total
    rec.scan_geometry_rejected = state['rejected']
    rec.scan_windows_s = None if scan_windows is None else [list(w) for w in scan_windows]
    if state['rejected']:
        rec.warnings.append('scan_geometry_changed:%d' % state['rejected'])


def _metadata(path):
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            meta = json.load(fh)
    except (OSError, ValueError):
        return {}
    keep = ('id', 'kind', 'state', 'started_at', 'finished_at', 'duration_s', 'include_imu', 'include_gps')
    out = {k: meta.get(k) for k in keep if k in meta}
    dur = meta.get('durability') or {}
    for k in ('mapping_sidecar_complete', 'mapping_sidecar_error', 'mapping_sidecar_worker_closed'):
        if k in dur:
            out[k] = dur[k]
    return out


CACHE_VERSION = 'field_calib.cache.v3 (lidar_motion.py ICP, chunked; scan times, IMU rows, sidecar clock steps)'


def cache_path(cache_dir, rid):
    return os.path.join(cache_dir, '%s.odometry.npz' % rid)


def load_cache(cache_dir, rid, key):
    path = cache_path(cache_dir, rid)
    if not os.path.isfile(path):
        return None
    try:
        z = np.load(path)
        if str(z['key']) != key:
            return None
        return dict(pose=z['pose'], quality=z['quality'], chain=z['chain'], scan_t=z['scan_t'],
                    scan_header_t=z['scan_header_t'], geometry=json.loads(str(z['geometry'])), imu=z['imu'],
                    scan_total=int(z['scan_total']), scan_geometry_rejected=int(z['scan_geometry_rejected']),
                    read_warnings=json.loads(str(z['read_warnings'])),
                    sidecar_clock=tuple(int(x) for x in z['sidecar_clock']))
    except (OSError, ValueError, KeyError):
        return None


def save_cache(cache_dir, rec, pose, quality, chain):
    os.makedirs(cache_dir, exist_ok=True)
    path = cache_path(cache_dir, rec.rid)
    tmp = path + '.tmp.npz'
    np.savez_compressed(tmp, key=rec.cache_key, pose=pose, quality=quality, chain=chain, scan_t=rec.scan_t,
                        scan_header_t=rec.scan_header_t, geometry=json.dumps(rec.geometry), imu=rec.imu,
                        scan_total=rec.scan_total, scan_geometry_rejected=rec.scan_geometry_rejected,
                        read_warnings=json.dumps(rec.read_warnings), sidecar_clock=np.array(rec.sidecar_clock))
    os.replace(tmp, path)


def open_archive(root, rid, label='', scan_windows_fn=None, hashes=True, pool=None, workers=1, cache_dir=None,
                 chunk_scans=600):
    """Jetson or full-extract layout. scan_windows_fn(motor) -> windows (s) or None for all scans.

    With cache_dir: the key is the sha256 of motor.jsonl and the sidecar plus the scan windows and the
    chunking; on a hit the sidecar is not read at all (the cache holds the ICP result, the scan times and
    the IMU rows), so a re-run with other labels or targets costs the two file hashes and the analysis."""
    path = os.path.join(root, rid)
    layout = detect_layout(path)
    if layout is None:
        raise RecordingError('%s: no mapping_source/sensors.jsonl.gz or sensors.jsonl.gz' % path)
    rec = Recording(rid, layout, label)
    rec.metadata = _metadata(os.path.join(path, 'metadata.json'))
    if rec.metadata.get('state') not in (None, 'archived'):
        rec.warnings.append('recording_state:%s' % rec.metadata.get('state'))
    motor_path = os.path.join(path, 'motor.jsonl')
    if not os.path.isfile(motor_path):
        raise RecordingError('%s: motor.jsonl missing (recordings before the motor timeline cannot be used)' % path)
    rec.motor = motor_from_jsonl(motor_path)
    if rec.motor.malformed:
        rec.warnings.append('motor_rows_malformed:%d' % rec.motor.malformed)
    sidecar = os.path.join(path, 'mapping_source', 'sensors.jsonl.gz') if layout == 'jetson' else \
        os.path.join(path, 'sensors.jsonl.gz')
    manifest = os.path.join(os.path.dirname(sidecar), 'manifest.json') if layout == 'jetson' else None
    if manifest and os.path.isfile(manifest):
        try:
            with open(manifest, 'r', encoding='utf-8') as fh:
                m = json.load(fh)
            rec.sidecar_manifest = {k: m.get(k) for k in ('complete', 'dropped', 'error', 'topic_counts')}
            if m.get('complete') is False or m.get('dropped'):
                rec.warnings.append('sidecar_incomplete:dropped=%s,error=%s' % (m.get('dropped'), m.get('error')))
        except (OSError, ValueError):
            rec.warnings.append('sidecar_manifest_unreadable')
    elif layout == 'jetson':
        rec.warnings.append('sidecar_manifest_missing(recording not closed?)')
    windows = scan_windows_fn(rec.motor) if scan_windows_fn else None
    digests = {}
    if hashes or cache_dir:
        digests = {motor_path: sha256_file(motor_path), sidecar: sha256_file(sidecar)}
    cached = None
    rec.cache_key = None
    if cache_dir:
        h = hashlib.sha256()
        h.update(json.dumps([CACHE_VERSION, rid, digests[motor_path], digests[sidecar], windows,
                             int(chunk_scans)]).encode())
        rec.cache_key = h.hexdigest()
        cached = load_cache(cache_dir, rid, rec.cache_key)
    if cached is None:
        before = len(rec.warnings)
        read_sidecar(rec, sidecar, windows, pool=pool, workers=workers)
        rec.read_warnings = rec.warnings[before:]
    else:
        # the sidecar is not read at all: scan times, IMU rows and the read warnings come from the cache
        rec.scan_t, rec.scan_header_t, rec.geometry = cached['scan_t'], cached['scan_header_t'], cached['geometry']
        rec.imu, rec.scan_total = cached['imu'], cached['scan_total']
        rec.scan_geometry_rejected = cached['scan_geometry_rejected']
        rec.read_warnings = cached['read_warnings']
        rec.sidecar_clock = cached['sidecar_clock']
        rec.warnings.extend(rec.read_warnings)
        rec.ranges_mm = np.zeros((len(rec.scan_t), 0), dtype=np.uint16)     # not needed after matching
        rec.scan_windows_s = None if windows is None else [list(w) for w in windows]
        rec.cached_odometry = (cached['pose'], cached['quality'], cached['chain'])
    rec.sources = {'motor.jsonl': dict(bytes=os.path.getsize(motor_path)),
                   os.path.relpath(sidecar, path).replace(os.sep, '/'): dict(bytes=os.path.getsize(sidecar))}
    if hashes:
        rec.sources['motor.jsonl']['sha256'] = digests[motor_path]
        rec.sources[os.path.relpath(sidecar, path).replace(os.sep, '/')]['sha256'] = digests[sidecar]
    rec.check_clock()
    return rec


def open_compact(base, rid, label='', scan_windows_fn=None, with_reference=False, drive_dir=None):
    """10/10 compact extract (validation): scans.npz + drive.json.gz (+ lidar_motion.npz reference).

    drive_dir: folder of <rid>.drive.json.gz; default the environment variable UBTS_RD_1010_DRIVE."""
    if drive_dir is None:
        drive_dir = os.environ.get('UBTS_RD_1010_DRIVE', '').strip() or None
    if drive_dir is None:
        raise FileNotFoundError('open_compact: the drive extract folder is not given (drive_dir argument or '
                                'UBTS_RD_1010_DRIVE)')
    z = np.load(os.path.join(base, 'lidar', '%s.scans.npz' % rid))
    with gzip.open(os.path.join(drive_dir, '%s.drive.json.gz' % rid), 'rt', encoding='utf-8') as fh:
        drive = json.load(fh)
    rec = Recording(rid, 'compact', label)
    rec.metadata = dict((drive.get('metadata') or {}))
    rec.motor = motor_from_compact(drive['motor'])
    imu = np.array(drive['imu'], dtype=np.float64)
    rec.imu = np.column_stack([imu[:, 0] / 1e9, imu[:, 1] / 1e9, imu[:, 2:8]]) if len(imu) else np.zeros((0, 8))
    rec.imu = rec.imu[np.argsort(rec.imu[:, 0], kind='stable')]
    t = z['bag_time_ns'] / 1e9
    keep = np.ones(len(t), dtype=bool)
    windows = scan_windows_fn(rec.motor) if scan_windows_fn else None
    if windows is not None:
        keep[:] = False
        for a, b in windows:
            keep |= (t >= a) & (t <= b)
    rec.scan_total = len(t)
    rec.scan_t = t[keep]
    rec.scan_header_t = z['header_ns'][keep] / 1e9
    rec.ranges_mm = z['ranges_mm'][keep]
    rec.scan_windows_s = None if windows is None else [list(w) for w in windows]
    rec.geometry = dict(angle_min=float(z['angle_min']), angle_increment=float(z['angle_increment']),
                        range_min=float(z['range_min']), range_max=float(z['range_max']),
                        beams=int(z['ranges_mm'].shape[1]), frame_id=str(z['frame_id']))
    rec.sources = {'scans.npz': dict(sha256_of_original=str(z['source_sha256'])),
                   'drive.json.gz': dict(sha256_of_original=drive.get('source_sha256'))}
    if with_reference:
        ref_path = os.path.join(base, 'lidar', '%s.lidar_motion.npz' % rid)
        if os.path.isfile(ref_path):
            m = np.load(ref_path)
            rec.reference_motion = dict(t=m['t'][keep], pose=m['pose'][keep], quality=m['quality'][keep][:, :4])
    rec.check_clock()
    return rec


def list_recordings(root):
    """Recording ids under root (sorted; id = UTC start time, so sorted = chronological)."""
    try:
        names = os.listdir(root)
    except OSError as exc:
        raise RecordingError('cannot list %s: %s' % (root, exc)) from None
    return sorted(n for n in names if detect_layout(os.path.join(root, n)))
