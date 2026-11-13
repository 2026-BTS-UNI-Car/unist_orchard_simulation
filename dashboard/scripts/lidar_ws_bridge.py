from __future__ import annotations
import argparse
import asyncio
import json
import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


@dataclass
class ScanState:
    frame_id: str = "laser"
    source: str = "waiting"
    stamp: float | None = None
    points: list[dict[str, float | int]] = field(default_factory=list)
    last_scan_at: float | None = None
    error: str | None = None
    # The original LaserScan fields of the newest scan (profile=mapping: the dashboard's own live map,
    # dashboard_server/source_live_mapping.py, needs them; same shape as the dashboard repository's bridge).
    raw_scan: dict = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update(
        self,
        frame_id: str,
        stamp: float,
        points: list[dict[str, float | int]],
        source: str,
        raw_scan: dict | None = None,
    ) -> None:
        with self.lock:
            self.frame_id = frame_id or "laser"
            self.source = source
            self.stamp = stamp
            self.points = points
            self.raw_scan = raw_scan or {}
            self.last_scan_at = time.time()
            self.error = None

    def set_error(self, message: str) -> None:
        with self.lock:
            self.error = message

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            age = (
                None
                if self.last_scan_at is None
                else round(time.time() - self.last_scan_at, 3)
            )
            return {
                "frame_id": self.frame_id,
                "source": self.source,
                "stamp": self.stamp,
                "points": list(self.points),
                "last_scan_age_sec": age,
                "point_count": len(self.points),
                "error": self.error,
                "raw_scan": self.raw_scan,
            }


def bridge_status_payload(state: ScanState) -> dict[str, Any]:
    snap = state.snapshot()
    alive = snap["last_scan_age_sec"] is not None and snap["last_scan_age_sec"] < 2.0
    return {
        "type": "lidar_bridge_status",
        "updated_at": time.time(),
        "scan_topic_alive": alive,
        "last_scan_age_sec": snap["last_scan_age_sec"],
        "point_count": snap["point_count"],
        "source": snap["source"],
        "frame_id": snap["frame_id"],
        "error": snap["error"] or (None if alive else "ROS2 /scan 대기 중"),
    }


def write_bridge_status(path: Path, payload: dict[str, Any]) -> None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(f"{path.suffix}.tmp")
        tmp.write_text(
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
        tmp.replace(path)
    except OSError:
        pass


def bridge_log(event: str, **fields: Any) -> None:
    payload = {"ts": round(time.time(), 3), "event": f"lidar_bridge.{event}", **fields}
    print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")), flush=True)


def status_writer_thread(
    state: ScanState, path: Path, interval_s: float, stop: threading.Event
) -> None:
    while not stop.is_set():
        write_bridge_status(path, bridge_status_payload(state))
        stop.wait(max(0.2, interval_s))
    write_bridge_status(path, bridge_status_payload(state))


def scan_to_points(msg: Any, max_points: int) -> list[dict[str, float | int]]:
    ranges = list(getattr(msg, "ranges", []) or [])
    intensities = list(getattr(msg, "intensities", []) or [])
    angle_min = float(getattr(msg, "angle_min", 0.0))
    angle_increment = float(getattr(msg, "angle_increment", 0.0))
    range_min = float(getattr(msg, "range_min", 0.0) or 0.0)
    range_max = float(getattr(msg, "range_max", 30.0) or 30.0)
    valid = []
    for index, distance in enumerate(ranges):
        r = float(distance)
        if not math.isfinite(r) or r <= 0:
            continue
        if range_min and r < range_min:
            continue
        if range_max and r > range_max:
            continue
        theta = angle_min + index * angle_increment
        angle_deg = math.degrees(theta) % 360.0
        quality = (
            int(intensities[index])
            if index < len(intensities) and math.isfinite(float(intensities[index]))
            else 0
        )
        valid.append(
            {
                "angle_deg": round(angle_deg, 3),
                "distance_m": round(r, 3),
                "quality": quality,
                "x_m": round(math.sin(theta) * r, 3),
                "y_m": round(math.cos(theta) * r, 3),
            }
        )
    return decimate_points(valid, max_points)


def decimate_points(
    points: list[dict[str, float | int]], max_points: int
) -> list[dict[str, float | int]]:
    max_points = max(50, int(max_points or 900))
    if len(points) <= max_points:
        return points
    step = len(points) / max_points
    return [points[min(len(points) - 1, int(i * step))] for i in range(max_points)]


def client_max_points(actual_path: str, default: int, source_max: int) -> int:
    parsed = urlparse(actual_path)
    raw = (parse_qs(parsed.query).get("max_points") or [default])[0]
    try:
        value = int(raw)
    except (TypeError, ValueError):
        value = default
    return max(50, min(max(source_max, default), value))


def ros_thread(
    state: ScanState,
    topic: str,
    max_points: int,
    ready: threading.Event,
    stop: threading.Event,
) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import LaserScan
    except ImportError as exc:
        state.set_error(f"ROS2 Python packages are required on Jetson: {exc}")
        bridge_log("ros_import_error", error=str(exc))
        ready.set()
        return

    from scripts.ros_executor import SimulationDiagnosticEvents

    class BridgeNode(SimulationDiagnosticEvents, Node):
        def __init__(self):
            # Preview timestamps come from LaserScan.header, not node.get_clock.
            # Keep the existing live-mode False override and avoid an unused
            # 120Hz /clock callback when running the simulation preview.
            super().__init__("ubts_lidar_ws_bridge", parameter_overrides=[Parameter("use_sim_time", value=False)])
            preview_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(LaserScan, topic, self.on_scan, preview_qos)
            self.scan_count = 0
            self.last_log_at = 0.0
            bridge_log("ros_subscribed", topic=topic, max_points=max_points)

        def on_scan(self, msg):
            stamp = (
                float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) / 1000000000
            )
            points = scan_to_points(msg, max_points)
            state.update(msg.header.frame_id or "laser", stamp, points, "ros2", raw_scan_of(msg))
            self.scan_count += 1
            now = time.time()
            if self.scan_count == 1 or now - self.last_log_at >= 10:
                self.last_log_at = now
                bridge_log(
                    "scan_received",
                    topic=topic,
                    frame_id=msg.header.frame_id or "laser",
                    raw_ranges=len(getattr(msg, "ranges", []) or []),
                    points=len(points),
                    count=self.scan_count,
                )

    rclpy.init()
    node = BridgeNode()
    ready.set()
    try:
        from scripts.ros_executor import persistent_executor
        with persistent_executor(node) as executor:
            while rclpy.ok() and (not stop.is_set()):
                executor.spin_once(timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()






def raw_scan_of(msg) -> dict:
    """The original LaserScan fields for profile=mapping (the dashboard repository's bridge, ui-20261112 90094fd):
    header, every range (non-finite -> None), angles, limits, scan and beam times."""
    return dict(header=dict(stamp=dict(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec),
                            frame_id=msg.header.frame_id),
                ranges=[float(v) if math.isfinite(v) else None for v in msg.ranges],
                angle_min=float(msg.angle_min), angle_increment=float(msg.angle_increment),
                range_min=float(msg.range_min), range_max=float(msg.range_max),
                scan_time=float(msg.scan_time), time_increment=float(msg.time_increment))


def mapping_payload(snap, points, sequence) -> dict:
    """profile=mapping message: the scan header fields, no display points, the original scan in raw_scan
    (source source_native_LaserScan), as the dashboard repository's bridge sends it."""
    payload = compact_scan_payload(snap, points, sequence, True, False)
    payload["raw_scan"] = snap.get("raw_scan") or {}
    payload["points"] = []
    payload["source"] = "source_native_LaserScan"
    return payload


def compact_scan_payload(snap, points, sequence, compact=True, overlay=True):
    payload = {
        "type": "scan",
        "frame_id": snap["frame_id"],
        "source": snap["source"],
        "stamp": snap["stamp"],
        "sequence": sequence,
        "point_count": len(points),
        "source_point_count": snap["point_count"],
        "requested_max_points": len(points),
        "coverage_deg": len(
            {int(p["angle_deg"]) % 360 for p in snap.get("points", points)}
        ),
    }
    if compact:
        payload["point_encoding"] = "polar-v1"
        payload["points"] = [
            [p["angle_deg"], p["distance_m"], p.get("quality", 0)] for p in points
        ]
    else:
        payload["points"] = points
    return payload


async def serve_websocket(
    state: ScanState,
    host: str,
    port: int,
    path: str,
    max_hz: float,
    source_max_points: int,
):
    import websockets
    from scripts.stream_policy import StreamBudget, wait_for_ack

    async def handler(websocket, legacy_path=None):
        actual_path = (
            getattr(getattr(websocket, "request", None), "path", None)
            or legacy_path
            or getattr(websocket, "path", path)
        )
        if urlparse(actual_path).path != path:
            await websocket.close(code=1008, reason="unsupported path")
            return
        params = parse_qs(urlparse(actual_path).query)
        compact = (params.get("format") or [""])[0] == "polar-v1"
        ack_flow = (params.get("flow") or [""])[0] == "ack"
        mapping = (params.get("profile") or [""])[0] == "mapping"
        budget = StreamBudget((params.get("profile") or ["auto"])[0])
        requested = client_max_points(
            actual_path, default=900, source_max=source_max_points
        )
        last_stamp = None
        sequence = 0
        last_status = 0.0
        try:
            await websocket.send(
                json.dumps(
                    {
                        "type": "socket",
                        "ok": True,
                        "latest_only": True,
                        "max_inflight_frames": 1,
                    }
                )
            )
            while True:
                started = time.monotonic()
                snap = state.snapshot()
                alive = (
                    snap["last_scan_age_sec"] is not None
                    and snap["last_scan_age_sec"] < 2
                )
                limit = (
                    60
                    if budget.values[0] <= 160
                    else 180
                    if budget.values[0] <= 320
                    else 450
                    if budget.values[0] <= 640
                    else source_max_points
                )
                points = decimate_points(snap["points"], min(requested, limit))
                if started - last_status >= 1:
                    await asyncio.wait_for(
                        websocket.send(
                            json.dumps(
                                {
                                    "type": "status",
                                    "device_found": alive,
                                    "scan_topic_alive": alive,
                                    "last_scan_age_sec": snap["last_scan_age_sec"],
                                    "point_count": len(points),
                                    "source_point_count": snap["point_count"],
                                    "requested_max_points": min(requested, limit),
                                    "source": snap["source"],
                                    "error": snap["error"],
                                    "latest_only": True,
                                    "profile": budget.profile,
                                    "effective_profile": budget.effective_profile,
                                    "ack_latency_ms": budget.ack_latency_ms,
                                },
                                separators=(",", ":"),
                            )
                        ),
                        timeout=2,
                    )
                    last_status = started
                if not alive or not points or snap["stamp"] == last_stamp:
                    await asyncio.sleep(0.04)
                    continue
                sequence += 1
                if mapping:
                    payload = mapping_payload(snap, points, sequence)
                else:
                    payload = compact_scan_payload(
                        snap, points, sequence, compact, budget.values[0] > 320
                    )
                message = json.dumps(payload, allow_nan=False, separators=(",", ":"))
                await asyncio.wait_for(websocket.send(message), timeout=2)
                last_stamp = snap["stamp"]
                if ack_flow:
                    await wait_for_ack(websocket, lambda reply:
                                       reply.get("type") == "scan_ack" and reply.get("sequence") == sequence)
                    budget.acknowledge(time.monotonic() - started)
                hz = min(
                    max_hz,
                    1
                    if budget.values[0] <= 160
                    else 3
                    if budget.values[0] <= 320
                    else 5
                    if budget.values[0] <= 640
                    else 10,
                )
                if mapping:
                    hz = max_hz          # every scan for the live map (local stream, not the preview budget)
                delay = max(
                    1 / max(0.1, hz), 0 if mapping else len(message) / max(2000, budget.values[4] / 4)
                ) - (time.monotonic() - started)
                if delay > 0:
                    await asyncio.sleep(delay)
        except Exception as exc:
            bridge_log("preview_closed", reason=type(exc).__name__)
        finally:
            await websocket.close()

    async with websockets.serve(
        handler,
        host,
        port,
        max_size=4096,
        max_queue=2,
        write_limit=8192,
        compression=None,
        close_timeout=1,
    ):
        await asyncio.Future()


def main():
    parser = argparse.ArgumentParser(description="Raw ROS LaserScan to WebSocket (no device fallback)")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--path", default="/ws/lidar")
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--source", choices=["ros2"], default="ros2")
    parser.add_argument("--max-points", type=int, default=3200)
    parser.add_argument("--max-hz", type=float, default=10)
    parser.add_argument("--status-file", default=os.environ.get("LIDAR_BRIDGE_STATUS_FILE", "/runtime/lidar_bridge_status.json"))
    args=parser.parse_args()
    state, stop, ready = ScanState(), threading.Event(), threading.Event()
    workers=[threading.Thread(target=ros_thread,args=(state,args.topic,args.max_points,ready,stop),daemon=True),
             threading.Thread(target=status_writer_thread,args=(state,Path(args.status_file),.5,stop),daemon=True)]
    for t in workers:t.start()
    try:asyncio.run(serve_websocket(state,args.host,args.port,args.path,args.max_hz,args.max_points))
    finally:
        stop.set()
        for t in workers:t.join(timeout=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
