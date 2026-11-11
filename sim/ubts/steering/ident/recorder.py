"""Record writers of the identification runs (field record formats; design 4.5).

Drive records (F1-F5), ArchiveRecorder, recorder archive layout (what tools/field_calib/records.py reads):
  <root>/<rid>/metadata.json                    id, kind, state archived, start/finish (wall), duration
  <root>/<rid>/motor.jsonl                      orchard.motor_pwm.timeline.v1 rows at 2 Hz from the bridge snapshot
                                                (/rover/status): received_at, status, requested.left/right (the
                                                throttle duty, design 4.2), pins D8/D5/D3/D0 reported_pwm from the MCU
                                                applied duty of the last status line, D1 enable
  <root>/<rid>/mapping_source/sensors.jsonl.gz  /scan (real S2 format: 3240 beams in raw order, 0.0 = no return,
                                                steering/s2_format.py) and /imu/data_raw rows; bag_time_ns = receipt
  <root>/<rid>/mapping_source/manifest.json     ubts.field_extraction.v1 (complete, dropped, topic_counts)
  <root>/_steering/<rid>/steering.jsonl         orchard.steering.timeline.v1 (design 4.5): one row per MCU status line
  <root>/_steering/<rid>/descriptor.json        topic, sample_hz, time_base, stale_after_s, calibration table copy,
                                                profile id and SHA (the operator's copy), host profile copy and SHA,
                                                meaning notes, frames_file, frame_count, preview
Bench records (B0-B7), BenchRecord: <root>/_bench/<rid>/
  bench.jsonl      serial log (h2m / m2h with host time and wall time) and the operator notes (test, event, params)
  steering.jsonl   the status lines as orchard.steering.timeline.v1 rows (bench tool decode)
  gauge.jsonl      instrument readings (digital angle gauge: left and right wheel angle, resolution)
  descriptor.json  test list, profile copy, B1 tape measurements, calibration table in effect, counts, file SHA-256

Times: receipt times are the host clock (SIM: the rig's injected clock); wall = epoch + SIM time (t1_rig wall_clock).
t_s and elapsed_monotonic_s count from the start of the record; clock_offset_s = wall - monotonic at the row.
No truth is written into a record: the operator's instruments write their readings (instruments.py).
"""
from __future__ import annotations

import gzip
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np

from .. import s2_format

STEERING_SCHEMA = 'orchard.steering.timeline.v1'
MOTOR_SCHEMA = 'orchard.motor_pwm.timeline.v1'
DESCRIPTOR_SCHEMA = 'orchard.steering.descriptor.v1'
BENCH_SCHEMA = 'ubts.steer_ident.bench_record.v1'
MEANING = ['가변저항은 링크 각의 측정값이며 타이어 접지각이 아니다',
           'PWM은 엔코더가 아니다', 'requested.left/right는 throttle에서 만든 구동 duty다(좌우 같음)']
STALE_AFTER_S = .15             # host_status_gap_ms (placeholder profile)


def _dumps(value):
    return json.dumps(value, separators=(',', ':'), sort_keys=False, ensure_ascii=True, allow_nan=False)


def _sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def _clean(value):
    """JSON-safe copy: numpy scalars to Python, non-finite floats to None."""
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _stamp(wall_s):
    sec = int(math.floor(wall_s))
    return dict(sec=sec, nanosec=int(round((wall_s - sec) * 1e9)) % 1000000000)


class ArchiveRecorder:
    """One drive recording (F test) in the recorder archive layout; rows are streamed, files closed by finish()."""

    wants_steering = True

    def __init__(self, root, rid, *, mono_start_s, wall_start_s, lidar_yaw_offset_deg, profile_copy, host_profile_bytes,
                 calibration, test, notes=None):
        self.root = Path(root)
        self.rid = rid
        self.dir = self.root / rid
        self.steer_dir = self.root / '_steering' / rid
        (self.dir / 'mapping_source').mkdir(parents=True, exist_ok=False)
        self.steer_dir.mkdir(parents=True, exist_ok=False)
        self.mono_start = float(mono_start_s)
        self.wall_start = float(wall_start_s)
        self.fmt = s2_format.RealFormat(lidar_yaw_offset_deg)
        self.profile_copy = profile_copy
        self.host_profile_bytes = host_profile_bytes
        self.calibration = calibration
        self.test = test
        self.notes = list(notes or ())
        self._sensors = gzip.open(self.dir / 'mapping_source' / 'sensors.jsonl.gz', 'wb', compresslevel=4)
        self._motor = open(self.dir / 'motor.jsonl', 'wb')
        self._steering = open(self.steer_dir / 'steering.jsonl', 'wb')
        self.counts = dict(scan=0, imu=0, motor=0, steering=0)
        self.preview = []
        self.closed = False
        self.last_wall = self.wall_start

    # -- rows -------------------------------------------------------------------------------------------------------
    def add_packet(self, packet, wall_s):
        if packet['kind'] == 'scan':
            self._add_scan(packet, wall_s)
        elif packet['kind'] == 'imu':
            self._add_imu(packet, wall_s)
        self.last_wall = max(self.last_wall, wall_s)

    def _add_scan(self, packet, wall_s):
        ranges = self.fmt.convert(packet['ranges'])
        text = ','.join('%.3f' % float(r) if r else '0.0' for r in ranges.tolist())
        header = dict(stamp=_stamp(self.wall_start + packet['stamp_s'] - self.mono_start), frame_id=s2_format.FRAME_ID)
        head = _dumps(dict(header=header, angle_min=s2_format.RAW_AMIN, angle_max=s2_format.RAW_AMIN + (
            s2_format.BEAMS - 1) * s2_format.RAW_INC, angle_increment=s2_format.RAW_INC, time_increment=.1 / s2_format.BEAMS,
            scan_time=.1, range_min=s2_format.RANGE_MIN_M, range_max=s2_format.RANGE_MAX_M))
        # message.ranges last inside the message object (records._parse_scan finds the first "ranges":[ ... ] span)
        message = head[:-1] + ',"ranges":[' + text + ']}'
        line = '{"topic":"/scan","message":%s,"bag_time_ns":%d}' % (message, int(round(wall_s * 1e9)))
        self._sensors.write(line.encode('ascii') + b'\n')
        self.counts['scan'] += 1

    def _add_imu(self, packet, wall_s):
        ax, ay, az = (float(v) for v in packet['accel_m_s2'])
        stamp = _stamp(self.wall_start + packet['stamp_s'] - self.mono_start)
        msg = dict(header=dict(stamp=stamp, frame_id='imu_link'), orientation=dict(x=0., y=0., z=0., w=1.),
                   angular_velocity=dict(x=0., y=0., z=float(packet['gyro_z_rad_s'])),
                   linear_acceleration=dict(x=ax, y=ay, z=az))
        line = '{"topic":"/imu/data_raw","message":%s,"bag_time_ns":%d}' % (_dumps(msg), int(round(wall_s * 1e9)))
        self._sensors.write(line.encode('ascii') + b'\n')
        self.counts['imu'] += 1

    def add_motor(self, snapshot, wall_s, mono_s, status_line):
        status = dict(ok=bool(snapshot.get('ok', True)), connected=bool(snapshot.get('connected')),
                      armed=bool(snapshot.get('armed')), moving=bool(snapshot.get('moving')),
                      state=snapshot.get('state'), firmware=None)
        req = snapshot.get('requested') or {}
        pins = {}
        if status_line is not None:
            for side, (fw, rv) in (('L', ('D8', 'D5')), ('R', ('D3', 'D0'))):
                d = int(status_line[side])
                pins[fw] = dict(reported_pwm=d if d > 0 else 0)
                pins[rv] = dict(reported_pwm=-d if d < 0 else 0)
            pins['D1'] = dict(reported_pwm=255 if status_line['en'] & 1 else 0)
        updated = snapshot.get('updated_at')
        row = dict(t_s=round(mono_s - self.mono_start, 6), received_at=wall_s, source_kind='simulation',
                   source_updated_at=updated, source_stale=False, status=status,
                   requested=dict(left=req.get('left'), right=req.get('right'), throttle=req.get('throttle'),
                                  steer_mrad=req.get('steer_mrad')), pins=pins)
        self._motor.write(_dumps(_clean(row)).encode('ascii') + b'\n')
        self.counts['motor'] += 1
        self.last_wall = max(self.last_wall, wall_s)

    def add_steering(self, state):
        received_mono = state.get('received_monotonic')
        received_at = state.get('received_at')
        row = dict(t_s=None if received_mono is None else round(received_mono - self.mono_start, 6),
                   elapsed_monotonic_s=None if received_mono is None else round(received_mono - self.mono_start, 6),
                   clock_offset_s=None if received_at is None or received_mono is None else received_at - received_mono)
        row.update(state)
        row['schema'] = STEERING_SCHEMA
        self._steering.write(_dumps(_clean(row)).encode('ascii') + b'\n')
        self.counts['steering'] += 1
        if len(self.preview) < 5:
            self.preview.append(_clean(row))

    # -- finish -----------------------------------------------------------------------------------------------------
    def finish(self, mono_end_s, wall_end_s):
        if self.closed:
            return
        self.closed = True
        self._sensors.close()
        self._motor.close()
        self._steering.close()
        duration = float(mono_end_s) - self.mono_start
        meta = dict(id=self.rid, kind='both', state='archived', started_at=self.wall_start, finished_at=wall_end_s,
                    duration_s=round(duration, 3), include_imu=True, include_gps=False,
                    durability=dict(mapping_sidecar_complete=True, mapping_sidecar_error=None,
                                    mapping_sidecar_worker_closed=True))
        (self.dir / 'metadata.json').write_bytes((json.dumps(meta, indent=1) + '\n').encode('ascii'))
        manifest = dict(schema='ubts.field_extraction.v1', complete=True, dropped=0, error=None,
                        topic_counts={'/scan': self.counts['scan'], '/imu/data_raw': self.counts['imu']})
        (self.dir / 'mapping_source' / 'manifest.json').write_bytes((json.dumps(manifest, indent=1) + '\n').encode('ascii'))
        host_name = 'host_profile.json'
        (self.steer_dir / host_name).write_bytes(self.host_profile_bytes)
        descriptor = dict(
            schema=DESCRIPTOR_SCHEMA, topic='/rover/steering/state', sample_hz=20, time_base='host receipt (monotonic)',
            stale_after_s=STALE_AFTER_S, recording_id=self.rid, test=self.test, notes=self.notes,
            monotonic_start_s=self.mono_start, wall_start_s=self.wall_start, calibration=self.calibration,
            profile=self.profile_copy, host_profile=dict(file=host_name,
                                                         sha256=hashlib.sha256(self.host_profile_bytes).hexdigest()),
            meaning=MEANING, frames_file='steering.jsonl', frame_count=self.counts['steering'], preview=self.preview,
            counts=self.counts,
            files={name: _sha(path) for name, path in (('steering.jsonl', self.steer_dir / 'steering.jsonl'),
                                                     ('motor.jsonl', self.dir / 'motor.jsonl'),
                                                     ('sensors.jsonl.gz', self.dir / 'mapping_source' /
                                                      'sensors.jsonl.gz'))})
        (self.steer_dir / 'descriptor.json').write_bytes(
            (json.dumps(_clean(descriptor), indent=1, ensure_ascii=False) + '\n').encode('utf-8'))


class BenchRecord:
    """One bench session record (B0-B7) under <root>/_bench/<rid>/ (module docstring)."""

    def __init__(self, root, rid, *, mono_start_s, wall_start_s, profile_copy, b1=None):
        self.root = Path(root)
        self.rid = rid
        self.dir = self.root / '_bench' / rid
        self.dir.mkdir(parents=True, exist_ok=False)
        self.mono_start = float(mono_start_s)
        self.wall_start = float(wall_start_s)
        self.profile_copy = profile_copy
        self.b1 = b1
        self.calibration = None
        self.status_rows = []
        self.serial = []
        self.notes = []
        self.gauge = []
        self.tests = []
        self.closed = False

    def add_serial(self, t, wall, direction, text, kind):
        self.serial.append(dict(t=round(t, 6), wall=wall, dir=direction, text=text, kind=kind))

    def add_note(self, t, wall, test, event, params):
        if event == 'begin' and test not in self.tests:
            self.tests.append(test)
        row = dict(t=round(t, 6), wall=wall, dir='note', test=test, event=event, params=_clean(params))
        self.serial.append(row)
        self.notes.append(row)

    def add_status(self, row):
        t = row['received_monotonic']
        out = dict(t_s=round(t - self.mono_start, 6), elapsed_monotonic_s=round(t - self.mono_start, 6),
                   clock_offset_s=row['received_at'] - t, source_kind='simulation', schema=STEERING_SCHEMA)
        out.update(row)
        self.status_rows.append(out)

    def add_gauge(self, t, wall, test, point, reading):
        self.gauge.append(dict(t=round(t, 6), wall=wall, test=test, point=point, **_clean(reading)))

    def set_calibration(self, calibration):
        self.calibration = calibration

    def finish(self):
        if self.closed:
            return
        self.closed = True
        files = {}
        for name, rows in (('bench.jsonl', self.serial), ('steering.jsonl', self.status_rows), ('gauge.jsonl', self.gauge)):
            path = self.dir / name
            path.write_bytes(b''.join(_dumps(_clean(r)).encode('ascii', 'backslashreplace') + b'\n' for r in rows))
            files[name] = _sha(path)
        descriptor = dict(schema=BENCH_SCHEMA, recording_id=self.rid, tests=self.tests, monotonic_start_s=self.mono_start,
                          wall_start_s=self.wall_start, profile=self.profile_copy, b1_measurements=self.b1,
                          calibration=self.calibration, counts=dict(serial=len(self.serial), status=len(self.status_rows),
                                                                    gauge=len(self.gauge), notes=len(self.notes)),
                          files=files, meaning=MEANING[:2])
        (self.dir / 'descriptor.json').write_bytes(
            (json.dumps(_clean(descriptor), indent=1, ensure_ascii=False) + '\n').encode('utf-8'))


def load_jsonl(path):
    with open(path, 'r', encoding='utf-8') as fh:
        return [json.loads(line) for line in fh if line.strip()]


def write_json(path, data):
    Path(path).write_bytes((json.dumps(_clean(data), indent=1, ensure_ascii=False, sort_keys=True) + '\n').encode('utf-8'))
