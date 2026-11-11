"""One owner for an explicitly selected ESP USB device; V2 firmware only."""
import fcntl
import os
import re
import select
import termios
import time
from pathlib import Path

from scripts.esp_wheel_controller import PulseController, WheelController


class EspMotor:
    def __init__(self, port, identity):
        self.port, self.identity = port, identity
        self.fd = self.lock = None
        self.firmware = None
        self.error = None
        self.outputs = {}
        self.targets = {}
        self.buffer = b""
        self.last_rx = self.last_info = self.last_ping = 0.0
        self.settled_at = 0.0
        self.controller = PulseController(self.write, self.event)
        self.wheels = WheelController(self.controller)

    def event(self, kind, **fields):
        if kind == "firmware":
            self.firmware = fields["name"]
        if kind == "watchdog":
            self.error = "mcu_watchdog_rearm_required"

    @property
    def ready(self):
        return self.fd is not None and self.firmware == "UBTS_WHEEL_PWM_V2" and time.monotonic() - self.last_rx < 1.5 and not self.error

    @property
    def stopped(self):
        return all(self.outputs.get(p) == 0 and self.targets.get(p) == 0 for p in ("D8", "D5", "D3", "D0"))

    def open(self):
        # Deployment pins the by-id path and checks its udev ID_SERIAL. Never scan ttyUSB*.
        if not self.identity or not self.port or self.port == "auto":
            raise RuntimeError("explicit ESP identity and device path required")
        info = os.stat(self.port)
        properties = Path(f"/run/udev/data/c{os.major(info.st_rdev)}:{os.minor(info.st_rdev)}").read_text()
        if f"E:ID_SERIAL={self.identity}" not in properties.splitlines():
            raise RuntimeError("live udev USB identity mismatch")
        self.lock = open(f"/run/lock/ubts-esp-{os.major(info.st_rdev)}-{os.minor(info.st_rdev)}.lock", "a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            self.settled_at = time.monotonic() + 1.5
        except Exception:
            self.close()
            raise

    def write(self, data):
        if self.fd is None:
            raise OSError("ESP disconnected")
        deadline = time.monotonic() + 0.1
        while data:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [self.fd], [], remaining)[1]:
                raise OSError("ESP write deadline")
            try:
                size = os.write(self.fd, data)
            except BlockingIOError:
                continue
            if size <= 0:
                raise OSError("ESP write failed")
            data = data[size:]

    def poll(self):
        if self.fd is None:
            return
        now = time.monotonic()
        if now < self.settled_at:
            return
        if select.select([self.fd], [], [], 0)[0]:
            data = os.read(self.fd, 4096)
            if not data:
                raise OSError("ESP disconnected")
            self.buffer += data
            if len(self.buffer) > 16384:
                raise OSError("ESP response overflow")
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                line = line.rstrip(b"\r")
                if self.wheels.observe_firmware(line):
                    self.last_rx = now
                match = re.fullmatch(rb"UBTS_WHEEL_PWM_V2 pin=(D[0-8]) PWM=(\d+) TARGET=(\d+)", line)
                if match:
                    pin = match[1].decode()
                    self.outputs[pin], self.targets[pin] = int(match[2]), int(match[3])
        if now - self.last_info > 0.5:
            self.write(b"INFO\n")
            self.last_info = now
        if self.wheels.enabled and now - self.last_ping > 0.2:
            self.wheels.heartbeat()
            self.write(b"PING\n")
            self.last_ping = now
        self.wheels.tick()

    def enable(self):
        if not self.ready:
            raise RuntimeError("verified ESP V2 required")
        self.wheels.heartbeat()
        self.wheels.enable(True)

    def drive(self, left, right):
        self.wheels.heartbeat()
        self.wheels.command(left="forward" if left > 0 else "reverse" if left < 0 else "stop",
                            right="forward" if right > 0 else "reverse" if right < 0 else "stop",
                            left_speed=abs(left), right_speed=abs(right))

    def release(self):
        self.wheels.stop()

    def hard_stop(self):
        if self.fd is not None and self.firmware == "UBTS_WHEEL_PWM_V2":
            self.wheels.stop_all("web_stop")

    def close(self):
        self.firmware = None
        self.outputs.clear()
        self.targets.clear()
        if self.fd is not None:
            try:
                fcntl.ioctl(self.fd, termios.TIOCNXCL)
            except OSError:
                pass
            os.close(self.fd)
            self.fd = None
        if self.lock is not None:
            self.lock.close()
            self.lock = None
