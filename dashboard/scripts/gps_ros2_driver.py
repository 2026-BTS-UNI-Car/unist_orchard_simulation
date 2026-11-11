#!/usr/bin/env python3
"""One UART owner; NMEA/UBX input to NavSatFix and a lossless quality payload.

Only read-only UBX/NMEA polls are sent. No CFG, reset, rate or flash writes.
"""
from __future__ import annotations

import argparse
import array
import errno
import fcntl
import glob
import json
import math
import os
from pathlib import Path
import select
import stat
import sys
import termios
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0, str(ROOT))
from dashboard_server.gps import GnssDecoder, fresh_snapshot, ubx_packet, valid_position
from dashboard_server.gps_reception import RECEPTION_POLL_SECONDS


SERIAL_ERRORS = (OSError, termios.error)


def serial_device_name(port, sysfs_root=Path('/sys')):
    """Resolve Docker's renamed device node through its kernel major/minor."""
    device = os.stat(port)
    if not stat.S_ISCHR(device.st_mode): raise OSError('GPS port is not a character device')
    entry = sysfs_root / 'dev/char' / f'{os.major(device.st_rdev)}:{os.minor(device.st_rdev)}'
    return entry.resolve().name if entry.exists() else Path(port).resolve().name


def serial_counters(fd):
    """Linux serial_icounter_struct; unsupported USB/PTY drivers return None."""
    values = array.array('i', [0]*20)
    try: fcntl.ioctl(fd, termios.TIOCGICOUNT, values, True)
    except OSError: return None
    names = ('cts', 'dsr', 'rng', 'dcd', 'rx', 'tx', 'frame', 'overrun', 'parity', 'brk', 'buf_overrun')
    return {name: value & 0xffffffff for name, value in zip(names, values) if name in names[4:]}


def nmea_poll(talker, message):
    body = f'EI{talker}Q,{message}'.encode('ascii')
    checksum = 0
    for value in body: checksum ^= value
    return b'$'+body+f'*{checksum:02X}\r\n'.encode('ascii')


def serial_owners(port):
    target = os.stat(port).st_rdev
    owners = set()
    for fdpath in glob.glob('/proc/[0-9]*/fd/*'):
        pid = int(fdpath.split('/')[2])
        if pid == os.getpid(): continue
        try:
            item = os.stat(fdpath)
            if stat.S_ISCHR(item.st_mode) and item.st_rdev == target: owners.add(pid)
        except OSError: pass
    return sorted(owners)


class GpsSerial:
    BAUDS = (38400, 9600, 115200, 57600, 19200, 230400, 460800, 921600)

    def __init__(self, port, baud=0, lock_path='/run/ubts-gps/serial.lock', poll_ubx=True):
        self.port, self.fixed_baud, self.poll_ubx = port, baud, poll_ubx
        self.lock_path = Path(lock_path)
        self.fd = self.lock_fd = None
        self.original = None
        self.baud_index = 0
        self.baud = baud or self.BAUDS[0]
        self.decoder = GnssDecoder()
        self.error = None
        self.bytes_received = 0
        self.connected_at = 0
        self.next_poll = 0
        self.next_reception_poll = 0
        self.pending_poll = b''
        self.last_good_packet = None
        self.device_name = None
        self.counter_baseline = None
        self.opened_at = None
        self.baud_attempts = []
        self.next_open = 0
        self.cleanup_errors = []

    def open(self):
        if self.fd is not None or self.lock_fd is not None:
            raise RuntimeError('GPS serial is already open')
        self.device_name = serial_device_name(self.port)
        consoles = {item.split('=', 1)[1].split(',', 1)[0]
                    for item in Path('/proc/cmdline').read_text().split() if item.startswith('console=')}
        if self.device_name in consoles:
            raise OSError('GPS port is the system console')
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        self.lock_fd = os.open(self.lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            owners = serial_owners(self.port)
            if owners: raise OSError('GPS UART already owned by PID(s): '+','.join(map(str, owners)))
            self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            self.original = termios.tcgetattr(self.fd)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            self.counter_baseline = serial_counters(self.fd)
            self.opened_at = time.monotonic()
            self.baud_attempts = []
            self.set_baud(self.baud)
            # A reopened USB path may refer to a new receiver. Never carry its
            # predecessor's partial packet, identity, fix or NMEA aiding state.
            self.decoder = GnssDecoder()
            self.bytes_received = 0
            self.cleanup_errors = []
        except Exception:
            self.close(); raise

    def set_baud(self, baud):
        cfg = termios.tcgetattr(self.fd)
        cfg[0] = cfg[1] = cfg[3] = 0
        cfg[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        cfg[4] = cfg[5] = getattr(termios, 'B'+str(baud))
        cfg[6][termios.VMIN] = cfg[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, cfg)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.baud = baud
        if baud not in self.baud_attempts: self.baud_attempts.append(baud)
        self.connected_at = time.monotonic()
        self.decoder.buffer.clear()
        self.last_good_packet = None
        self.next_poll = 0
        self.next_reception_poll = 0
        self.pending_poll = b''

    def read(self):
        # With VMIN=0 an empty read alone is not a reliable disconnect signal.
        # POLLHUP also detects removal in passive mode, without a poll write.
        readiness = select.poll()
        readiness.register(self.fd, select.POLLIN | select.POLLERR | select.POLLHUP)
        if any(flags & (select.POLLERR | select.POLLHUP | select.POLLNVAL)
               for _, flags in readiness.poll(0)):
            raise OSError(errno.EIO, 'GPS serial device disconnected')
        frames = []
        before = self.decoder.packets
        for _ in range(8):
            try: data = os.read(self.fd, 8192)
            except BlockingIOError: break
            if not data: break
            self.bytes_received += len(data)
            frames.extend(self.decoder.feed(data))
        now = time.monotonic()
        if self.decoder.packets > before: self.last_good_packet = now
        if not self.fixed_baud and now - (self.last_good_packet or self.connected_at) > 5:
            self.baud_index = (self.baud_index + 1) % len(self.BAUDS)
            self.set_baud(self.BAUDS[self.baud_index])
        if self.poll_ubx and now >= self.next_poll and not self.pending_poll:
            # Query data only, including receivers configured without periodic output.
            polls = ubx_packet(1, 7) + ubx_packet(1, 20)
            if not self.decoder.identity: polls = ubx_packet(10, 4) + polls
            # A receiver can have UBX input disabled and periodic NMEA output off.
            # Query both common talker IDs until it provides a navigation sample.
            if self.decoder.last_sample_at is None or time.time()-self.decoder.last_sample_at > 3:
                polls += b''.join(nmea_poll(talker, message)
                                  for talker in ('GN', 'GP') for message in ('GGA', 'RMC'))
            if now >= self.next_reception_poll:
                # Empty NAV-SAT/NAV-SIG polls only, in this same serial owner.
                # Keep a fixed bounded cadence even when unsupported or unanswered.
                polls += ubx_packet(1, 0x35) + ubx_packet(1, 0x43)
                self.next_reception_poll = now + RECEPTION_POLL_SECONDS
            self.pending_poll = polls
            self.next_poll = now + 1.0
        if self.poll_ubx and self.pending_poll:
            # Nonblocking short writes resume the same bounded packet batch;
            # never append another batch or interleave a new packet mid-frame.
            try: self.pending_poll = self.pending_poll[os.write(self.fd, self.pending_poll):]
            except BlockingIOError: pass
        if self.decoder.packets > before: self.error = None
        return frames

    def poll(self):
        """One retryable serial step; publishing/programming errors stay visible."""
        if self.fd is None and time.monotonic() < self.next_open: return []
        try:
            if self.fd is None: self.open()
            return self.read()
        except SERIAL_ERRORS as error:
            self.error = str(error)
            self.close()
            self.next_open = time.monotonic() + 5
            return []

    def snapshot(self):
        counts = serial_counters(self.fd) if self.fd is not None else None
        if counts is not None and self.counter_baseline is not None:
            counts = {name: (value-self.counter_baseline[name]) & 0xffffffff for name, value in counts.items()}
        else: counts = None
        driver = dict(serial_port=self.port, serial_open=self.fd is not None, baud=self.baud,
                      auto_baud=not self.fixed_baud, bytes_received=self.bytes_received,
                      valid_packets=self.decoder.packets, rejected_packets=self.decoder.rejected,
                      receiver=self.decoder.identity, error=self.error, configuration_writes=False,
                      cleanup_errors=list(self.cleanup_errors),
                      serial_device=self.device_name, baud_attempts=list(self.baud_attempts),
                      serial_open_s=round(time.monotonic()-self.opened_at, 1) if self.opened_at is not None else None,
                      uart_counters=counts, uart_counters_scope='since_uart_open' if counts is not None else None)
        # A recently valid fix must become invalid immediately on disconnect.
        received_at = self.decoder.last_sample_at if self.fd is not None else None
        return fresh_snapshot(self.decoder.sample, received_at, driver=driver,
                              reception=self.decoder.reception)

    def close(self):
        fd, lock_fd, original = self.fd, self.lock_fd, self.original
        # Detach first: repeated close must never act on a recycled descriptor.
        self.fd = self.lock_fd = self.original = None
        self.opened_at = None
        self.counter_baseline = None
        if fd is None and lock_fd is None: return
        self.cleanup_errors = []

        def cleanup(label, action):
            try: action()
            except SERIAL_ERRORS as error:
                self.cleanup_errors.append(f'{label}: {error}')

        try:
            if fd is not None:
                try:
                    if original is not None:
                        cleanup('restore termios', lambda: termios.tcsetattr(fd, termios.TCSANOW, original))
                finally:
                    cleanup('release serial exclusive mode', lambda: fcntl.ioctl(fd, termios.TIOCNXCL))
        finally:
            try:
                if fd is not None: cleanup('close serial descriptor', lambda: os.close(fd))
            finally:
                if lock_fd is not None: cleanup('close lock descriptor', lambda: os.close(lock_fd))


def make_navsat(sample, message_class):
    msg = message_class()
    stamp = sample.get('stamp') or {}
    msg.header.stamp.sec, msg.header.stamp.nanosec = int(stamp.get('sec', 0)), int(stamp.get('nanosec', 0))
    msg.header.frame_id = 'gps_link'
    ok = valid_position(sample)
    msg.status.status = (2 if sample.get('quality') in (4, 5) else 0) if ok else -1
    # PVT/GGA do not identify which constellations actually supplied the fix.
    msg.status.service = 0
    msg.latitude = float(sample['latitude']) if ok else math.nan
    msg.longitude = float(sample['longitude']) if ok else math.nan
    msg.altitude = float(sample['altitude_ellipsoid_m']) if ok and sample.get('altitude_ellipsoid_m') is not None else math.nan
    h, v = sample.get('h_acc_m'), sample.get('v_acc_m')
    if ok and h is not None and v is not None:
        msg.position_covariance = [h*h, 0., 0., 0., h*h, 0., 0., 0., v*v]
        msg.position_covariance_type = 1  # Approximated, not a surveyed covariance.
    return msg


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=os.environ.get('GPS_SERIAL_PORT', '/dev/ubts-gps'))
    parser.add_argument('--baud', type=int, default=int(os.environ.get('GPS_BAUD', '0')))
    parser.add_argument('--status-file', default=os.environ.get('GPS_DRIVER_STATUS_FILE', '/app/runtime/gps_driver_status.json'))
    parser.add_argument('--poll-ubx', action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    if args.baud and args.baud not in GpsSerial.BAUDS: parser.error('Unsupported baud')
    rclpy.init()
    node = Node('ubts_gps_driver')
    serial = GpsSerial(args.port, args.baud, poll_ubx=args.poll_ubx)
    fix_pub = node.create_publisher(NavSatFix, '/gps/fix', qos_profile_sensor_data)
    data_pub = node.create_publisher(String, '/gps/data', qos_profile_sensor_data)
    status_pub = node.create_publisher(String, '/gps/driver_status', 10)
    def poll():
        for frame in serial.poll():
            stamp = node.get_clock().now().to_msg()
            frame['stamp'] = dict(sec=stamp.sec, nanosec=stamp.nanosec)
            frame['stamp_role'] = 'host_ROS_publication_not_receiver_measurement'
            serial.decoder.sample['stamp'] = frame['stamp']
            serial.decoder.sample['stamp_role'] = frame['stamp_role']
            fix_pub.publish(make_navsat(frame, NavSatFix))
            data_pub.publish(String(data=json.dumps(frame, allow_nan=False, separators=(',', ':'))))

    def status():
        payload = serial.snapshot()
        encoded = json.dumps(payload, allow_nan=False, separators=(',', ':'))
        status_pub.publish(String(data=encoded))
        try:
            path = Path(args.status_file); path.parent.mkdir(parents=True, exist_ok=True)
            temp = path.with_suffix('.tmp'); temp.write_text(encoded); temp.replace(path)
        except OSError as error: node.get_logger().warning(str(error))

    node.create_timer(.02, poll)
    node.create_timer(1., status)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        try: serial.close()
        finally:
            try: node.destroy_node()
            finally: rclpy.try_shutdown()
    return 0


if __name__ == '__main__': raise SystemExit(main())
