#!/usr/bin/env python3
"""ESP motor owner. Unix socket control; ROS publishes observations only."""
import concurrent.futures
import json
import os
from pathlib import Path
import queue
import signal
import socketserver
import threading
import time

from scripts.esp_motor import EspMotor
from scripts.rover_control_core import RoverControl, ControlError


class Bridge:
    def __init__(self, motor):
        self.motor = motor
        self.control = RoverControl(motor)
        self.pending = queue.Queue(maxsize=8)
        self.emergency = threading.Event()
        self.snapshot = {"ok": True, "connected": False, "state": "starting"}

    def request(self, request):
        if request.get("op") == "status":
            return dict(self.snapshot)
        if request.get("op") == "stop":
            self.emergency.set()
            return {"ok": True, "state": "stop_requested"}
        future = concurrent.futures.Future()
        try:
            self.pending.put_nowait((time.monotonic(), request, future))
            return future.result(timeout=0.4)
        except (queue.Full, concurrent.futures.TimeoutError):
            future.cancel()
            self.emergency.set()
            return {"ok": False, "error": "control_busy_stopped"}

    def step(self):
        if self.emergency.is_set():
            self.control.stop("operator_stop")
            while not self.pending.empty():
                _, _, future = self.pending.get_nowait()
                if not future.done():
                    future.set_result({"ok": False, "error": "stop_barrier"})
            self.emergency.clear()
        self.control.tick()
        self.motor.poll()
        self.control.tick()
        # One bounded request per tick, after stop/expiry and serial feedback.
        if not self.pending.empty():
            received, request, future = self.pending.get_nowait()
            if not future.cancelled():
                if time.monotonic() - received > 0.25:
                    result = {"ok": False, "error": "request_expired"}
                else:
                    try:
                        result = self.control.dispatch(request)
                    except (ControlError, ValueError) as exc:
                        result = {"ok": False, "error": str(exc)}
                if not future.done():
                    future.set_result(result)
        self.snapshot = {**self.control.status(), "updated_at": time.time(),
                         "transport": "esp_usb", "serial_port": self.motor.port,
                         "serial_open": self.motor.fd is not None, "mode": "manual",
                         "device_identity": self.motor.identity}


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(0.4)
        try:
            line = self.rfile.readline(4097)
            if len(line) > 4096:
                raise ValueError("request_too_large")
            request = json.loads(line)
            if not isinstance(request, dict):
                raise ValueError("object_required")
            response = self.server.bridge.request(request)
        except (ValueError, OSError) as exc:
            response = {"ok": False, "error": str(exc)}
        try:
            self.wfile.write(json.dumps(response, allow_nan=False).encode() + b"\n")
        except OSError:
            pass


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    request_queue_size = 8


def main():
    motor = EspMotor(os.environ.get("ROVER_SERIAL_PORT", "/dev/ubts-motor"), os.environ.get("ROVER_ESP_IDENTITY", ""))
    bridge = Bridge(motor)
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.set())
    path = Path(os.environ.get("ROVER_CONTROL_SOCKET", "/run/ubts-rover/control.sock"))
    path.parent.mkdir(parents=True, exist_ok=True)
    # A process lock avoids deleting a live peer's socket.
    import fcntl
    lock = open(path.parent / "bridge.lock", "a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    path.unlink(missing_ok=True)
    server = Server(str(path), Handler)
    os.chmod(path, 0o660)
    server.bridge = bridge
    threading.Thread(target=server.serve_forever, daemon=True).start()
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node("ubts_esp_rover_bridge")
    status_pub = node.create_publisher(String, "/rover/status", 1)
    requested_pub = node.create_publisher(String, "/rover/control/requested", 1)
    applied_pub = node.create_publisher(String, "/rover/control/applied", 1)
    fault = None
    try:
        try:
            motor.open()
        except Exception as exc:
            fault = str(exc)
        published = 0.0
        while not stopped.wait(0.02):
            try:
                if not fault:
                    bridge.step()
                else:
                    bridge.snapshot = {**bridge.control.status(), "ok": False, "state": "fault", "last_error": fault, "updated_at": time.time()}
            except Exception as exc:
                fault = str(exc)
                try:
                    bridge.control.stop("hardware_fault")
                except OSError:
                    pass
                finally:
                    motor.close()
            if time.monotonic() - published >= 0.5:
                payload = bridge.snapshot
                for publisher, obj in ((status_pub, payload), (requested_pub, payload.get("requested", {})), (applied_pub, payload.get("outputs", {}))):
                    msg = String()
                    msg.data = json.dumps(obj, allow_nan=False)
                    publisher.publish(msg)
                target = Path(os.environ.get("ROVER_BRIDGE_STATUS_FILE", "/app/runtime/rover_bridge_status.json"))
                target.parent.mkdir(parents=True, exist_ok=True)
                tmp = target.with_suffix(".tmp")
                tmp.write_text(json.dumps(payload))
                tmp.replace(target)
                published = time.monotonic()
    finally:
        try:
            bridge.control.stop("shutdown")
        finally:
            motor.close()
            server.shutdown()
            server.server_close()
            path.unlink(missing_ok=True)
            node.destroy_node()
            rclpy.shutdown()
            lock.close()


if __name__ == "__main__":
    main()
