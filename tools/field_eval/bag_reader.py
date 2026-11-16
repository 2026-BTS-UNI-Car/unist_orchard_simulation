"""rosbag2 reader for /scan (sensor_msgs/msg/LaserScan) and /imu/data_raw (sensor_msgs/msg/Imu), standard library +
numpy, no ROS installation.

Storage:
  sqlite3   <bag>/*.db3 (tables topics, messages; timestamp = recorder receipt time, ns). Implemented.
  mcap      <bag>/*.mcap (records Schema / Channel / Message, in Chunk records or not). Implemented for chunks
            without compression. A chunk compressed with zstd or lz4 (the rosbag2 MCAP writer can do both; which
            one the Jetson's `ros2 bag record` uses has not been checked on a real bag) needs the optional module
            'zstandard' (or Python 3.14 compression.zstd) or 'lz4'; without it BagFormatError names the gap.
  A per-file compressed bag (*.db3.zstd, *.mcap.zstd) is not read (BagFormatError).
Messages are ROS 2 CDR (XCDR1, little or big endian by the encapsulation header); only the two message types above
are decoded (decode_laser_scan, decode_imu). Neither reader has been run on a bag written by the Jetson recorder:
the tests write bags with this module's own encoder (tests/fe_testlib.py), so a field mismatch would show only on
the first real bag. The recorder's sidecar (mapping_source/sensors.jsonl.gz) is the checked path (sources.py).
"""
from __future__ import annotations

import os
import sqlite3
import struct

import numpy as np

SCAN_TYPE = 'sensor_msgs/msg/LaserScan'
IMU_TYPE = 'sensor_msgs/msg/Imu'
MCAP_MAGIC = b'\x89MCAP0\r\n'


class BagFormatError(ValueError):
    pass


# --- CDR ---------------------------------------------------------------------------------------------------------------
class _Cdr:
    def __init__(self, data):
        if len(data) < 4:
            raise BagFormatError('cdr: short message')
        kind = data[1]
        if kind not in (0, 1):
            raise BagFormatError('cdr: encapsulation %r not plain CDR' % data[:2])
        self.e = '<' if kind == 1 else '>'
        self.buf = memoryview(data)[4:]
        self.pos = 0

    def align(self, n):
        self.pos += (-self.pos) % n

    def read(self, fmt, size):
        self.align(size)
        value = struct.unpack_from(self.e + fmt, self.buf, self.pos)
        self.pos += struct.calcsize(fmt)
        return value

    def u32(self):
        return self.read('I', 4)[0]

    def i32(self):
        return self.read('i', 4)[0]

    def f32(self, n=1):
        v = self.read('%df' % n, 4)
        return v if n > 1 else v[0]

    def f64(self, n=1):
        v = self.read('%dd' % n, 8)
        return v if n > 1 else v[0]

    def string(self):
        n = self.u32()
        raw = bytes(self.buf[self.pos:self.pos + n])
        self.pos += n
        return raw.rstrip(b'\x00').decode('utf-8', 'replace')

    def f32_seq(self):
        n = self.u32()
        self.align(4)
        arr = np.frombuffer(self.buf, dtype=self.e + 'f4', count=n, offset=self.pos).astype(np.float32)
        self.pos += 4 * n
        return arr

    def header(self):
        sec, nsec = self.i32(), self.u32()
        return sec + nsec * 1e-9, self.string()


def decode_laser_scan(data):
    """dict(stamp_s, frame_id, angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max,
    ranges float32) of one CDR LaserScan."""
    c = _Cdr(data)
    stamp, frame = c.header()
    amin, amax, inc, tinc, stime, rmin, rmax = c.f32(7)
    ranges = c.f32_seq()
    return dict(stamp_s=stamp, frame_id=frame, angle_min=amin, angle_max=amax, angle_increment=inc,
                time_increment=tinc, scan_time=stime, range_min=rmin, range_max=rmax, ranges=ranges)


def decode_imu(data):
    """dict(stamp_s, frame_id, gyro (x, y, z) rad/s, accel (x, y, z) m/s^2) of one CDR Imu."""
    c = _Cdr(data)
    stamp, frame = c.header()
    c.f64(4)                      # orientation
    c.f64(9)
    gyro = c.f64(3)
    c.f64(9)
    accel = c.f64(3)
    return dict(stamp_s=stamp, frame_id=frame, gyro=tuple(gyro), accel=tuple(accel))


def encode_laser_scan(stamp_s, ranges, angle_min, angle_increment, range_min=.05, range_max=30., frame_id='laser',
                      scan_time=.1):
    """CDR bytes of a LaserScan (little endian; for the tests and for converting SIM captures)."""
    out = bytearray(b'\x00\x01\x00\x00')
    sec = int(np.floor(stamp_s))
    nsec = int(round((stamp_s - sec) * 1e9))

    def pad(n):
        out.extend(b'\x00' * ((-(len(out) - 4)) % n))
    out += struct.pack('<iI', sec, nsec)
    name = frame_id.encode('utf-8') + b'\x00'
    out += struct.pack('<I', len(name)) + name
    pad(4)
    ranges = np.asarray(ranges, dtype='<f4')
    amax = angle_min + angle_increment * (len(ranges) - 1)
    out += struct.pack('<7f', angle_min, amax, angle_increment, 0., scan_time, range_min, range_max)
    out += struct.pack('<I', len(ranges)) + ranges.tobytes()
    out += struct.pack('<I', 0)
    return bytes(out)


def encode_imu(stamp_s, gyro, accel, frame_id='imu_link'):
    out = bytearray(b'\x00\x01\x00\x00')
    sec = int(np.floor(stamp_s))
    nsec = int(round((stamp_s - sec) * 1e9))

    def pad(n):
        out.extend(b'\x00' * ((-(len(out) - 4)) % n))
    out += struct.pack('<iI', sec, nsec)
    name = frame_id.encode('utf-8') + b'\x00'
    out += struct.pack('<I', len(name)) + name
    pad(8)
    out += struct.pack('<4d', 0., 0., 0., 1.) + struct.pack('<9d', *([0.] * 9))
    out += struct.pack('<3d', *gyro) + struct.pack('<9d', *([0.] * 9))
    out += struct.pack('<3d', *accel) + struct.pack('<9d', *([0.] * 9))
    return bytes(out)


# --- storage -----------------------------------------------------------------------------------------------------------
def _bag_files(path):
    if os.path.isfile(path):
        return [path]
    if not os.path.isdir(path):
        raise FileNotFoundError(path)
    names = sorted(os.listdir(path))
    for n in names:
        if n.endswith('.zstd') or n.endswith('.zst'):
            raise BagFormatError('%s: a per-file compressed bag is not read (decompress it first)' % n)
    files = [os.path.join(path, n) for n in names if n.endswith('.db3') or n.endswith('.mcap')]
    if not files:
        raise BagFormatError('%s: no .db3 or .mcap file' % path)
    return files


def _sqlite_messages(path, topics):
    con = sqlite3.connect('file:%s?mode=ro' % path.replace('\\', '/'), uri=True)
    try:
        rows = con.execute('SELECT id, name, type FROM topics').fetchall()
        ids = {r[0]: (r[1], r[2]) for r in rows if r[1] in topics}
        if not ids:
            return
        marks = ','.join('?' * len(ids))
        cur = con.execute('SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (%s) ORDER BY timestamp'
                          % marks, tuple(ids))
        for topic_id, stamp, data in cur:
            name, kind = ids[topic_id]
            yield name, kind, int(stamp), bytes(data)
    finally:
        con.close()


def _decompress(kind, data, size):
    if kind in ('', None):
        return data
    if kind == 'zstd':
        try:
            from compression import zstd           # Python >= 3.14
            return zstd.decompress(data)
        except ImportError:
            pass
        try:
            import zstandard
        except ImportError:
            raise BagFormatError('mcap chunk compressed with zstd: install the module zstandard or decompress the bag '
                                 '(ros2 bag convert / mcap recover) first') from None
        return zstandard.ZstdDecompressor().decompress(data, max_output_size=size)
    if kind == 'lz4':
        try:
            import lz4.frame
        except ImportError:
            raise BagFormatError('mcap chunk compressed with lz4: install the module lz4 or decompress the bag '
                                 'first') from None
        return lz4.frame.decompress(data)
    raise BagFormatError('mcap chunk compression %r is not known' % kind)


def _mcap_string(buf, pos):
    n = struct.unpack_from('<I', buf, pos)[0]
    return bytes(buf[pos + 4:pos + 4 + n]).decode('utf-8'), pos + 4 + n


def _mcap_records(buf, start, end, state, topics):
    pos = start
    while pos + 9 <= end:
        op = buf[pos]
        length = struct.unpack_from('<Q', buf, pos + 1)[0]
        body = pos + 9
        pos = body + length
        if pos > end:
            raise BagFormatError('mcap: truncated record')
        if op == 0x03:                                  # Schema
            sid = struct.unpack_from('<H', buf, body)[0]
            name, _ = _mcap_string(buf, body + 2)
            state['schemas'][sid] = name
        elif op == 0x04:                                # Channel
            cid, sid = struct.unpack_from('<HH', buf, body)
            topic, p = _mcap_string(buf, body + 4)
            encoding, _ = _mcap_string(buf, p)
            state['channels'][cid] = (topic, sid, encoding)
        elif op == 0x05:                                # Message
            cid, _seq, log_time, _pub = struct.unpack_from('<HIQQ', buf, body)
            topic, sid, encoding = state['channels'].get(cid, (None, None, None))
            if topic in topics:
                if encoding != 'cdr':
                    raise BagFormatError('mcap: channel %s has message encoding %r (cdr expected)' % (topic, encoding))
                yield topic, state['schemas'].get(sid), int(log_time), bytes(buf[body + 22:pos])
        elif op == 0x06:                                # Chunk
            _s, _e, size, _crc = struct.unpack_from('<QQQI', buf, body)
            kind, p = _mcap_string(buf, body + 28)
            n = struct.unpack_from('<Q', buf, p)[0]
            inner = _decompress(kind, bytes(buf[p + 8:p + 8 + n]), size)
            yield from _mcap_records(memoryview(inner), 0, len(inner), state, topics)
        elif op in (0x0F, 0x02):                        # DataEnd, Footer: the summary section follows
            return


def _mcap_messages(path, topics):
    with open(path, 'rb') as fh:
        buf = fh.read()
    if not buf.startswith(MCAP_MAGIC):
        raise BagFormatError('%s: not an MCAP file' % os.path.basename(path))
    state = dict(schemas={}, channels={})
    out = list(_mcap_records(memoryview(buf), len(MCAP_MAGIC), len(buf), state, topics))
    out.sort(key=lambda m: m[2])
    yield from out


def read_bag(path, scan_topic='/scan', imu_topic='/imu/data_raw'):
    """(scans, imus): scans = list of decode_laser_scan dicts with receipt_s; imus = list of decode_imu dicts with
    receipt_s (receipt = the recorder's message timestamp)."""
    scans, imus = [], []
    topics = {scan_topic, imu_topic}
    for name in _bag_files(path):
        reader = _sqlite_messages if name.endswith('.db3') else _mcap_messages
        for topic, kind, stamp_ns, data in reader(name, topics):
            if topic == scan_topic:
                if kind not in (None, SCAN_TYPE):
                    raise BagFormatError('%s has type %s' % (topic, kind))
                msg = decode_laser_scan(data)
                msg['receipt_s'] = stamp_ns * 1e-9
                scans.append(msg)
            else:
                if kind not in (None, IMU_TYPE):
                    raise BagFormatError('%s has type %s' % (topic, kind))
                msg = decode_imu(data)
                msg['receipt_s'] = stamp_ns * 1e-9
                imus.append(msg)
    return scans, imus


# --- writers for the tests -------------------------------------------------------------------------------------------
def write_sqlite_bag(folder, messages):
    """A minimal rosbag2 sqlite3 bag: messages = [(topic, type, receipt_ns, cdr bytes)] (tests)."""
    os.makedirs(folder, exist_ok=True)
    con = sqlite3.connect(os.path.join(folder, os.path.basename(folder.rstrip('/\\')) + '_0.db3'))
    try:
        con.execute('CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL)')
        con.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, '
                    'serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)')
        con.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, '
                    'timestamp INTEGER NOT NULL, data BLOB NOT NULL)')
        ids = {}
        for topic, kind, _, _ in messages:
            if topic not in ids:
                ids[topic] = len(ids) + 1
                con.execute('INSERT INTO topics VALUES (?, ?, ?, ?, ?)', (ids[topic], topic, kind, 'cdr', ''))
        for topic, _, stamp, data in messages:
            con.execute('INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?)', (ids[topic], stamp, data))
        con.commit()
    finally:
        con.close()


def write_mcap_bag(path, messages, chunked=True, compression=''):
    """A minimal MCAP file (tests): messages = [(topic, type, receipt_ns, cdr bytes)]; compression '' or 'zstd'
    (the latter needs the module zstandard)."""
    def rec(op, body):
        return bytes([op]) + struct.pack('<Q', len(body)) + body

    def s(text):
        raw = text.encode('utf-8')
        return struct.pack('<I', len(raw)) + raw
    schemas, channels, body = {}, {}, bytearray()
    for topic, kind, stamp, data in messages:
        if kind not in schemas:
            schemas[kind] = len(schemas) + 1
            body += rec(0x03, struct.pack('<H', schemas[kind]) + s(kind) + s('ros2msg') + struct.pack('<I', 0))
        if topic not in channels:
            channels[topic] = len(channels) + 1
            body += rec(0x04, struct.pack('<HH', channels[topic], schemas[kind]) + s(topic) + s('cdr')
                        + struct.pack('<I', 0))
        body += rec(0x05, struct.pack('<HIQQ', channels[topic], 0, stamp, stamp) + data)
    out = bytearray(MCAP_MAGIC) + rec(0x01, s('ros2') + s('field_eval_test'))
    if chunked:
        t0 = min(m[2] for m in messages) if messages else 0
        t1 = max(m[2] for m in messages) if messages else 0
        data = bytes(body)
        if compression == 'zstd':
            import zstandard
            data = zstandard.ZstdCompressor().compress(data)
        out += rec(0x06, struct.pack('<QQQI', t0, t1, len(body), 0) + s(compression) + struct.pack('<Q', len(data))
                   + data)
    else:
        out += body
    out += rec(0x0F, struct.pack('<I', 0)) + rec(0x02, struct.pack('<QQI', 0, 0, 0)) + MCAP_MAGIC
    with open(path, 'wb') as fh:
        fh.write(bytes(out))
