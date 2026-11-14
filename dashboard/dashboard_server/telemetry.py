from __future__ import annotations
import base64
import hashlib
import http.client
import ipaddress
import json
import fcntl
import math
import os
import platform
import re
import shutil
import socket
import struct
import subprocess
import termios
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import urllib.request
from collections import OrderedDict
from concurrent.futures import Future, TimeoutError as FutureTimeoutError
from dashboard_server.motor_pwm import with_recorded_motor
from dashboard_server.playback_profile import POLAR_PROFILE, point_fields
from dashboard_server.telemetry_cache import light_timeline, TimelineLoadUnavailable

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config" / "dashboard.json"
DEFAULT_TELEMETRY_FILE = ROOT / "runtime" / "telemetry_snapshot.json"
DEFAULT_LIDAR_BRIDGE_STATUS_FILE = ROOT / "runtime" / "lidar_bridge_status.json"
DEFAULT_CAMERA_DRIVER_STATUS_FILE = ROOT / "runtime" / "camera_driver_status.json"
DEFAULT_CAMERA_BRIDGE_STATUS_FILE = ROOT / "runtime" / "camera_bridge_status.json"
DEFAULT_IMU_DRIVER_STATUS_FILE = ROOT / "runtime" / "imu_driver_status.json"
DEFAULT_IMU_BRIDGE_STATUS_FILE = ROOT / "runtime" / "imu_bridge_status.json"
DEFAULT_CAMERA_SETTINGS_FILE = ROOT / "runtime" / "camera_settings.json"
DEFAULT_ROVER_BRIDGE_STATUS_FILE = ROOT / "runtime" / "rover_bridge_status.json"
DEFAULT_DATA_RECORDER_STATUS_FILE = ROOT / "runtime" / "data_recorder_status.json"
DEFAULT_RECORDING_ARCHIVE_DIR = ROOT / "runtime" / "recordings"
PLAYBACK_CACHE_SCHEMA = "orchard.timeline.playback_cache.v1"
PLAYBACK_CACHE_REVISION = "raw_sensor_segment_v2"
PLAYBACK_DASHBOARD_PROFILE = "dashboard-compact-v1"
PLAYBACK_LIDAR_POINT_FIELDS = ("x_m", "y_m", "distance_m", "quality", "angle_deg")
PLAYBACK_SEGMENT_SECONDS = 4.0
DEFAULT_MEMORY_DISPLAY_TOTAL_BYTES = 8 * 1024 * 1024 * 1024
DEFAULT_CAMERA_VIEW_MODES = [
    {
        "resolution": "1920x1080",
        "width": 1920,
        "height": 1080,
        "fps": [30],
        "label": "1080p 16:9",
    },
    {
        "resolution": "1280x720",
        "width": 1280,
        "height": 720,
        "fps": [60],
        "label": "720p 16:9",
    },
    {
        "resolution": "1024x768",
        "width": 1024,
        "height": 768,
        "fps": [30],
        "label": "768p 4:3",
    },
    {
        "resolution": "800x600",
        "width": 800,
        "height": 600,
        "fps": [60],
        "label": "600p 4:3",
    },
    {
        "resolution": "640x480",
        "width": 640,
        "height": 480,
        "fps": [30],
        "label": "480p 4:3",
    },
    {
        "resolution": "320x240",
        "width": 320,
        "height": 240,
        "fps": [30],
        "label": "240p 4:3",
    },
    {
        "resolution": "1280x1024",
        "width": 1280,
        "height": 1024,
        "fps": [30],
        "label": "1024p 5:4",
    },
]
DEFAULT_LIDAR_BAUDS = [1000000, 460800, 256000, 115200]
DEFAULT_LIDAR_SCAN_DURATION_S = 3.0
DEFAULT_LIDAR_SCAN_MAX_POINTS = 1800
DEFAULT_FAN_STATE_FILE = ROOT / "data" / "fan_state.json"
RPLIDAR_STOP = bytes([165, 37])
RPLIDAR_SCAN = bytes([165, 32])
RPLIDAR_SCAN_DESCRIPTOR = bytes([165, 90, 5, 0, 0, 64, 129])
TIOCMGET = 21525
TIOCMSET = 21528
TIOCM_DTR = 2
TIOCM_RTS = 4


def _read_text(path: str | Path) -> str | None:
    try:
        return Path(path).read_text(encoding="utf-8").strip()
    except (OSError, TypeError, UnicodeDecodeError):
        return None


def _load_json(path: str | Path) -> dict[str, Any]:
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def _write_json_atomic(path: str | Path, payload: dict[str, Any]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(f"{target.suffix}.tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(target)


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _safe_int(value: str | None) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return None


def _pct(used: int, total: int) -> float | None:
    if total <= 0:
        return None
    return round(used / total * 100.0, 1)


def _cpu_display_label(name: str) -> str:
    if name.startswith("cpu") and name[3:].isdigit():
        return f"CPU {int(name[3:]) + 1}"
    return name.upper()


class DockerSocketHTTPConnection(http.client.HTTPConnection):
    def __init__(self, socket_path: str):
        super().__init__("localhost")
        self.socket_path = socket_path

    def connect(self) -> None:
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(self.socket_path)


class TelemetryProvider:
    def __init__(self, config_path: str | Path | None = None):
        self.started_at = time.time()
        self.config_path = Path(
            config_path or os.environ.get("DASHBOARD_CONFIG", DEFAULT_CONFIG)
        )
        self.config = _load_json(self.config_path)
        self.mode = os.environ.get("DASHBOARD_MODE", self.config.get("mode", "live"))
        telemetry_file = Path(
            os.environ.get(
                "TELEMETRY_FILE",
                self.config.get("telemetry_file", str(DEFAULT_TELEMETRY_FILE)),
            )
        )
        self.telemetry_file = (
            telemetry_file if telemetry_file.is_absolute() else ROOT / telemetry_file
        )
        lidar_bridge_file = Path(
            os.environ.get(
                "LIDAR_BRIDGE_STATUS_FILE", str(DEFAULT_LIDAR_BRIDGE_STATUS_FILE)
            )
        )
        self.lidar_bridge_status_file = (
            lidar_bridge_file
            if lidar_bridge_file.is_absolute()
            else ROOT / lidar_bridge_file
        )
        camera_driver_file = Path(
            os.environ.get(
                "CAMERA_DRIVER_STATUS_FILE", str(DEFAULT_CAMERA_DRIVER_STATUS_FILE)
            )
        )
        self.camera_driver_status_file = (
            camera_driver_file
            if camera_driver_file.is_absolute()
            else ROOT / camera_driver_file
        )
        camera_bridge_file = Path(
            os.environ.get(
                "CAMERA_BRIDGE_STATUS_FILE", str(DEFAULT_CAMERA_BRIDGE_STATUS_FILE)
            )
        )
        self.camera_bridge_status_file = (
            camera_bridge_file
            if camera_bridge_file.is_absolute()
            else ROOT / camera_bridge_file
        )
        imu_driver_file = Path(
            os.environ.get(
                "IMU_DRIVER_STATUS_FILE", str(DEFAULT_IMU_DRIVER_STATUS_FILE)
            )
        )
        self.imu_driver_status_file = (
            imu_driver_file if imu_driver_file.is_absolute() else ROOT / imu_driver_file
        )
        imu_bridge_file = Path(
            os.environ.get(
                "IMU_BRIDGE_STATUS_FILE", str(DEFAULT_IMU_BRIDGE_STATUS_FILE)
            )
        )
        self.imu_bridge_status_file = (
            imu_bridge_file if imu_bridge_file.is_absolute() else ROOT / imu_bridge_file
        )
        camera_settings_file = Path(
            os.environ.get("CAMERA_SETTINGS_FILE", str(DEFAULT_CAMERA_SETTINGS_FILE))
        )
        self.camera_settings_file = (
            camera_settings_file
            if camera_settings_file.is_absolute()
            else ROOT / camera_settings_file
        )
        rover_bridge_file = Path(
            os.environ.get(
                "ROVER_BRIDGE_STATUS_FILE", str(DEFAULT_ROVER_BRIDGE_STATUS_FILE)
            )
        )
        self.rover_bridge_status_file = (
            rover_bridge_file
            if rover_bridge_file.is_absolute()
            else ROOT / rover_bridge_file
        )
        recorder_status_file = Path(
            os.environ.get(
                "DATA_RECORDER_STATUS_FILE", str(DEFAULT_DATA_RECORDER_STATUS_FILE)
            )
        )
        self.data_recorder_status_file = (
            recorder_status_file
            if recorder_status_file.is_absolute()
            else ROOT / recorder_status_file
        )
        archive_dir = Path(
            os.environ.get("RECORDING_ARCHIVE_DIR", str(DEFAULT_RECORDING_ARCHIVE_DIR))
        )
        self.recording_archive_dir = (
            archive_dir if archive_dir.is_absolute() else ROOT / archive_dir
        )
        playback_cache_dir = Path(
            os.environ.get(
                "RECORDING_PLAYBACK_CACHE_DIR",
                str(self.recording_archive_dir.parent / "playback_cache"),
            )
        )
        self.recording_playback_cache_dir = (
            playback_cache_dir
            if playback_cache_dir.is_absolute()
            else ROOT / playback_cache_dir
        )
        self.recorder_api_url = os.environ.get(
            "RECORDER_API_URL", "http://127.0.0.1:8790"
        ).rstrip("/")
        self.network_control_enabled = _env_bool("NETWORK_CONTROL_ENABLED", False)
        self.network_mode_helper = os.environ.get(
            "NETWORK_MODE_HELPER", "/usr/local/sbin/ubts-network-mode"
        )
        self.host_root = Path(os.environ.get("HOST_ROOT", "/host-root"))
        if not self.host_root.exists():
            self.host_root = Path("/")
        self.proc_root = Path(os.environ.get("HOST_PROC", "/proc"))
        self.sys_root = Path(os.environ.get("HOST_SYS", "/sys"))
        self.docker_socket = os.environ.get("DOCKER_SOCKET", "/var/run/docker.sock")
        self._self_container_names = self._dashboard_container_names()
        self._self_container_id = self._detect_self_container_id()
        self._last_cpu = self._read_cpu_times()
        self._last_net = self._read_net_counters()
        self._last_sample_at = time.time()
        self._external_ip_value: str | None = None
        self._external_ip_error: str | None = None
        self._external_ip_next_refresh = 0.0
        self._external_ip_thread: threading.Thread | None = None
        # Slow collectors are refreshed in the background (S03/S23); requests
        # read the last value. Docker stats take about 1 s per container.
        self._docker_lock = threading.Lock()
        self._docker_value: dict[str, Any] | None = None
        self._docker_at = 0.0
        self._docker_thread: threading.Thread | None = None
        self._docker_refresh_s = max(1.0, float(os.environ.get("TELEMETRY_DOCKER_REFRESH_S", "10")))
        self._docker_yield_refresh_s = max(1.0, float(os.environ.get("TELEMETRY_DOCKER_YIELD_REFRESH_S", "60")))
        self.docker_yield: Any = None  # callable -> True while background work must yield
        self._disks_cache: tuple[float, list[dict[str, Any]]] | None = None
        self._disks_ttl_s = max(0.0, float(os.environ.get("TELEMETRY_DISKS_TTL_S", "30")))
        self._holders_cache: dict[str, tuple[float, list[dict[str, Any]]]] = {}
        self._holders_ttl_s = max(0.0, float(os.environ.get("TELEMETRY_DEVICE_HOLDERS_TTL_S", "5")))
        self._last_lidar_test: dict[str, Any] | None = None
        self._last_lidar_scan: dict[str, Any] | None = None
        self._timeline_cache: OrderedDict[str, tuple[int, int, dict[str, Any]]] = (
            OrderedDict()
        )
        self._timeline_cache_lock = threading.RLock()
        self._timeline_cache_limit = max(
            1, int(os.environ.get("ARCHIVE_TIMELINE_CACHE_COUNT", "2"))
        )
        # Owners run in existing HTTP requests. No executor/background thread is
        # created; even distinct recordings cannot accumulate unbounded parses.
        self._timeline_loads: dict[tuple[str, int, int], Future] = {}
        self._timeline_load_slots = threading.BoundedSemaphore(self._timeline_cache_limit)
        self._timeline_wait_slots = threading.BoundedSemaphore(4 * self._timeline_cache_limit)
        self._light_timeline_cache: OrderedDict[str, tuple[int, int, dict[str, Any]]] = OrderedDict()
        self._playback_cache_lock = threading.RLock()
        self._rover_status_cache: dict[str, Any] | None = None
        self._rover_status_next_refresh = 0.0
        self._rover_status_timeout_s = max(
            0.1, float(os.environ.get("ROVER_STATUS_TIMEOUT_S", "0.35"))
        )
        self._rover_status_refresh_s = max(
            1.0, float(os.environ.get("ROVER_STATUS_REFRESH_S", "5.0"))
        )
        self._fan_hwmon_cache: dict[str, Any] | None = None
        self._power_hwmon_cache: Path | None = None
        self._fan_hold_interval_s = max(
            0.0, float(os.environ.get("FAN_HOLD_INTERVAL_S", "1.5"))
        )
        self._fan_hold_thread: threading.Thread | None = None
        if self._fan_hold_interval_s > 0:
            self._fan_hold_thread = threading.Thread(
                target=self._fan_hold_loop, daemon=True
            )
            self._fan_hold_thread.start()

    def health(self) -> dict[str, Any]:
        return {
            "ok": True,
            "service": "unist_orachard_dashboard",
            "mode": self.mode,
            "uptime_s": round(time.time() - self.started_at, 1),
            "host": self._hostname(),
        }

    def telemetry(self, include_rover: bool = True) -> dict[str, Any]:
        # include_rover=False is used by the shared snapshot, which reads the
        # rover state again for every request (telemetry_snapshot.py).
        external = (
            _load_json(self.telemetry_file) if self.telemetry_file.exists() else {}
        )
        connection = self.connection(external, include_rover=include_rover)
        payload = {
            "meta": self._meta(),
            "system": self.system(),
            "network": self.network(),
            "docker": self.docker_cached(),
            "fan": self.fan_status(),
            "connection": connection,
            "rover": external["rover"] if "rover" in external else (self.rover_status() if include_rover else None),
            "rover_source": "external" if "rover" in external else "rover_bridge",
            "sensors": connection["sensors"],
            "experiment": external.get("experiment", self._not_connected("experiment")),
            "mission": external.get("mission", self._mission_status()),
            "services": connection["services"],
            "alerts": self._alerts(external),
        }
        return payload

    def connection(self, external: dict[str, Any] | None = None, include_rover: bool = True) -> dict[str, Any]:
        external = external or (
            _load_json(self.telemetry_file) if self.telemetry_file.exists() else {}
        )
        lidar_bridge = self._lidar_bridge_status()
        camera_bridge = self._camera_bridge_status()
        imu_bridge = self._imu_bridge_status()
        rover_bridge = self._rover_bridge_status()
        return {
            "lidar": self.lidar_status(),
            "camera": self.camera_status(),
            "lidar_bridge": lidar_bridge,
            "camera_bridge": camera_bridge,
            "imu_bridge": imu_bridge,
            "rover_bridge": rover_bridge,
            "rover": self.rover_status() if include_rover else None,
            "services": self._services(
                external, lidar_bridge, camera_bridge, imu_bridge, rover_bridge
            ),
            "sensors": self._sensors(external, lidar_bridge, camera_bridge, imu_bridge),
        }

    def _disks_cached(self) -> list[dict[str, Any]]:
        now = time.monotonic()
        cached = self._disks_cache
        if cached is not None and now - cached[0] < self._disks_ttl_s:
            return cached[1]
        disks = self._disks()
        self._disks_cache = (now, disks)
        return disks

    def docker_cached(self) -> dict[str, Any]:
        """Last Docker summary, refreshed by one background thread.

        Refresh period: 10 s, or 60 s while autonomy has resource priority."""
        now = time.monotonic()
        try:
            yielding = bool(self.docker_yield()) if callable(self.docker_yield) else False
        except Exception:
            yielding = False
        period = self._docker_yield_refresh_s if yielding else self._docker_refresh_s
        with self._docker_lock:
            value, at = self._docker_value, self._docker_at
            running = self._docker_thread is not None and self._docker_thread.is_alive()
            if (value is None or now - at >= period) and not running:
                self._docker_thread = threading.Thread(target=self._refresh_docker, daemon=True,
                                                       name="telemetry-docker")
                self._docker_thread.start()
        if value is None:
            return {"available": False, "pending": True, "error": "Docker 상태 수집 중", "containers": []}
        return {**value, "age_s": round(max(0.0, now - at), 1)}

    def _refresh_docker(self) -> None:
        try:
            value = self.docker()
        except Exception as exc:  # never let the background thread die silently with a stale lock
            value = {"available": False, "error": str(exc), "containers": []}
        with self._docker_lock:
            self._docker_value = value
            self._docker_at = time.monotonic()

    def system(self) -> dict[str, Any]:
        memory = self._memory()
        swap = self._swap()
        disks = self._disks_cached()
        cpu = self._cpu_percent()
        return {
            "hostname": self._hostname(),
            "platform": platform.platform(),
            "architecture": platform.machine(),
            "jetson_release": _read_text(
                os.environ.get("JETSON_RELEASE_FILE", "/etc/nv_tegra_release")
            ),
            "service_uptime_s": round(time.time() - self.started_at, 1),
            "os_uptime_s": self._os_uptime_s(),
            "loadavg": os.getloadavg()
            if hasattr(os, "getloadavg")
            else [0.0, 0.0, 0.0],
            "cpu": cpu,
            "memory": memory,
            "swap": swap,
            "power": self._power(),
            "temperatures": self._temperatures(),
            "temperature_c": self._max_temperature(),
            "disks": disks,
        }

    def network(self) -> dict[str, Any]:
        now = time.time()
        counters = self._read_net_counters()
        elapsed = max(0.001, now - self._last_sample_at)
        interfaces = []
        for name, values in counters.items():
            previous = self._last_net.get(name, values)
            rx_bps = max(0.0, (values["rx_bytes"] - previous["rx_bytes"]) / elapsed)
            tx_bps = max(0.0, (values["tx_bytes"] - previous["tx_bytes"]) / elapsed)
            interfaces.append(
                {
                    "name": name,
                    "state": self._iface_text(name, "operstate"),
                    "carrier": self._iface_text(name, "carrier"),
                    "mac": self._iface_text(name, "address"),
                    "speed_mbps": _safe_int(self._iface_text(name, "speed")),
                    "mtu": _safe_int(self._iface_text(name, "mtu")),
                    "ipv4": self._ipv4_for_interface(name),
                    "rx_bytes": values["rx_bytes"],
                    "tx_bytes": values["tx_bytes"],
                    "rx_bps": round(rx_bps, 1),
                    "tx_bps": round(tx_bps, 1),
                    "rx_packets": values["rx_packets"],
                    "tx_packets": values["tx_packets"],
                }
            )
        self._last_net = counters
        self._last_sample_at = now
        active = [
            iface
            for iface in interfaces
            if iface.get("state") == "up" and iface["name"] != "lo"
        ]
        primary_ipv4 = self._primary_ipv4(interfaces)
        primary_interface = self._primary_interface(primary_ipv4, interfaces)
        traffic_interfaces = [iface for iface in interfaces if iface["name"] != "lo"]
        return {
            "hostname": self._hostname(),
            "interfaces": sorted(
                interfaces, key=lambda item: (item["name"] == "lo", item["name"])
            ),
            "active_interfaces": [iface["name"] for iface in active],
            "primary_ipv4": primary_ipv4,
            "primary_interface": primary_interface,
            "external_ip": self._cached_external_ip(),
            "external_ip_error": self._external_ip_error,
            "total_rx_bps": round(
                sum((iface["rx_bps"] for iface in traffic_interfaces)), 1
            ),
            "total_tx_bps": round(
                sum((iface["tx_bps"] for iface in traffic_interfaces)), 1
            ),
            "control": self.network_control_status(),
        }

    def network_control_status(self) -> dict[str, Any]:
        if not self.network_control_enabled:
            return {
                "ok": False,
                "enabled": False,
                "state": "disabled",
                "can_request_ap": False,
                "can_request_wifi": False,
            }
        result = self._network_mode_command("status")
        result.setdefault("enabled", True)
        return result

    def set_network_mode(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not self.network_control_enabled:
            return {
                "ok": False,
                "state": "disabled",
                "error": "Network control is disabled.",
            }
        action = str(payload.get("action") or "").strip().lower()
        confirmations = {
            "ap": "disconnect_wifi_and_enable_ap",
            "wifi": "leave_ap_and_scan_known_wifi",
        }
        if action not in confirmations:
            return {
                "ok": False,
                "state": "bad_action",
                "error": "action must be ap or wifi",
            }
        if str(payload.get("confirm") or "") != confirmations[action]:
            return {"ok": False, "state": "confirmation_required", "action": action}
        result = self._network_mode_command(f"request-{action}")
        result["requested_action"] = action
        return result

    def _network_mode_command(self, command: str) -> dict[str, Any]:
        if command not in {"status", "request-ap", "request-wifi"}:
            return {"ok": False, "state": "bad_command"}
        helper = Path(self.network_mode_helper)
        host_helper = self.host_root / str(helper).lstrip("/")
        nsenter = shutil.which("nsenter")
        if self.host_root == Path("/") and helper.exists():
            argv = [str(helper), command, "--json"]
            runner = "host"
        elif nsenter and host_helper.exists():
            argv = [
                nsenter,
                "-t",
                "1",
                "-m",
                "-u",
                "-i",
                "-n",
                "-p",
                "--",
                str(helper),
                command,
                "--json",
            ]
            runner = "nsenter"
        else:
            return {
                "ok": False,
                "enabled": True,
                "state": "helper_unavailable",
                "error": f"Host helper not found: {helper}",
            }
        try:
            proc = subprocess.run(
                argv, check=False, capture_output=True, text=True, timeout=5
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return {
                "ok": False,
                "enabled": True,
                "state": "helper_error",
                "error": str(exc),
            }
        output = (proc.stdout or "").strip().splitlines()
        try:
            result = json.loads(output[-1]) if output else {}
        except json.JSONDecodeError:
            result = {}
        if not isinstance(result, dict):
            result = {}
        result.setdefault("ok", proc.returncode == 0)
        result.setdefault("state", "ok" if proc.returncode == 0 else "helper_failed")
        if proc.returncode != 0 and (not result.get("error")):
            result["error"] = (
                proc.stderr or proc.stdout or f"exit {proc.returncode}"
            ).strip()[:500]
        result["enabled"] = True
        result["runner"] = runner
        return result

    def docker(self) -> dict[str, Any]:
        if not Path(self.docker_socket).exists():
            return {
                "available": False,
                "error": "docker socket not mounted",
                "containers": [],
            }
        try:
            info = self._docker_get_json("/info")
            version = self._docker_get_json("/version")
            containers = self._docker_get_json("/containers/json?all=1")
        except OSError as exc:
            return {"available": False, "error": str(exc), "containers": []}
        rows = []
        for container in containers:
            container_id = container.get("Id", "")
            stats = self._docker_container_stats(container_id) if container_id else {}
            rows.append(
                {
                    "id": container_id[:12],
                    "full_id": container_id,
                    "name": (container.get("Names") or [""])[0].lstrip("/"),
                    "image": container.get("Image"),
                    "state": container.get("State"),
                    "status": container.get("Status"),
                    "ports": container.get("Ports", []),
                    "cpu_percent": stats.get("cpu_percent"),
                    "memory_percent": stats.get("memory_percent"),
                    "memory_usage": stats.get("memory_usage"),
                    "memory_limit": stats.get("memory_limit"),
                    "rx_bytes": stats.get("rx_bytes"),
                    "tx_bytes": stats.get("tx_bytes"),
                    "self": self._is_self_container(
                        container_id, container.get("Names") or []
                    ),
                }
            )
        running = [row for row in rows if row.get("state") == "running"]
        return {
            "available": True,
            "engine": {
                "server_version": info.get("ServerVersion"),
                "docker_root_dir": info.get("DockerRootDir"),
                "driver": info.get("Driver"),
                "operating_system": info.get("OperatingSystem"),
                "architecture": info.get("Architecture"),
                "containers": info.get("Containers"),
                "containers_running": info.get("ContainersRunning"),
                "images": info.get("Images"),
                "version": version.get("Version"),
                "api_version": version.get("ApiVersion"),
            },
            "summary": {
                "total": len(rows),
                "running": len(running),
                "stopped": len(rows) - len(running),
                "cpu_percent": round(
                    sum((row.get("cpu_percent") or 0.0 for row in rows)), 1
                ),
                "memory_usage": sum((row.get("memory_usage") or 0 for row in rows)),
                "memory_limit": max(
                    [row.get("memory_limit") or 0 for row in rows] or [0]
                ),
            },
            "containers": rows,
        }

    def docker_container_detail(self, container_id: str) -> dict[str, Any]:
        safe_id = str(container_id or "").strip().lstrip("/")
        if not safe_id:
            return {"ok": False, "error": "container_id_required"}
        if not Path(self.docker_socket).exists():
            return {"ok": False, "error": "docker socket not mounted"}
        try:
            detail = self._docker_get_json(f"/containers/{safe_id}/json")
            stats = self._docker_container_stats(safe_id)
        except (OSError, json.JSONDecodeError) as exc:
            return {"ok": False, "id": safe_id, "error": str(exc)}
        config = detail.get("Config", {})
        state = detail.get("State", {})
        host_config = detail.get("HostConfig", {})
        network_settings = detail.get("NetworkSettings", {})
        return {
            "ok": True,
            "id": (detail.get("Id") or safe_id)[:12],
            "full_id": detail.get("Id"),
            "name": str(detail.get("Name") or "").lstrip("/"),
            "image": config.get("Image"),
            "created": detail.get("Created"),
            "state": state,
            "restart_policy": host_config.get("RestartPolicy"),
            "network_mode": host_config.get("NetworkMode"),
            "mounts": detail.get("Mounts", []),
            "ports": network_settings.get("Ports") or {},
            "stats": stats,
            "self": self._is_self_container(
                detail.get("Id") or safe_id, [detail.get("Name") or ""]
            ),
        }

    def docker_container_action(self, container_id: str, action: str) -> dict[str, Any]:
        safe_id = str(container_id or "").strip().lstrip("/")
        if action not in {"start", "stop", "restart"}:
            return {"ok": False, "error": "bad_action", "action": action}
        if not safe_id:
            return {"ok": False, "error": "container_id_required", "action": action}
        if not Path(self.docker_socket).exists():
            return {"ok": False, "error": "docker socket not mounted", "action": action}
        try:
            detail = self._docker_get_json(f"/containers/{safe_id}/json")
        except (OSError, json.JSONDecodeError) as exc:
            return {"ok": False, "id": safe_id, "action": action, "error": str(exc)}
        full_id = detail.get("Id") or safe_id
        if self._is_self_container(full_id):
            return {
                "ok": False,
                "id": full_id[:12],
                "action": action,
                "state": "self_container_blocked",
                "error": "Dashboard container cannot control itself from the dashboard UI.",
            }
        suffix = {"start": "/start", "stop": "/stop?t=10", "restart": "/restart?t=10"}[
            action
        ]
        try:
            status, body = self._docker_post_raw(f"/containers/{safe_id}{suffix}")
        except OSError as exc:
            return {
                "ok": False,
                "id": full_id[:12],
                "action": action,
                "error": str(exc),
            }
        ok = status in {204, 304}
        return {
            "ok": ok,
            "id": full_id[:12],
            "action": action,
            "status": status,
            "state": "applied" if ok else "docker_api_error",
            "message": body.decode("utf-8", errors="replace")[:500] if body else "",
        }

    def docker_container_logs(
        self, container_id: str, tail: int = 160
    ) -> dict[str, Any]:
        safe_id = str(container_id or "").strip().lstrip("/")
        if not safe_id:
            return {"ok": False, "error": "container_id_required", "lines": []}
        if not Path(self.docker_socket).exists():
            return {"ok": False, "error": "docker socket not mounted", "lines": []}
        try:
            tail = max(20, min(500, int(tail)))
        except (TypeError, ValueError):
            tail = 160
        try:
            raw = self._docker_get_raw(
                f"/containers/{safe_id}/logs?stdout=1&stderr=1&timestamps=1&tail={tail}"
            )
        except OSError as exc:
            return {"ok": False, "id": safe_id, "error": str(exc), "lines": []}
        text = self._decode_docker_log_stream(raw)
        lines = [line for line in text.splitlines() if line.strip()]
        return {"ok": True, "id": safe_id, "tail": tail, "lines": lines[-tail:]}

    def lidar_status(self) -> dict[str, Any]:
        candidates = self._lidar_candidates()
        present = [item for item in candidates if item["exists"]]
        selected = present[0] if present else candidates[0] if candidates else None
        holders = (
            self._device_holders(selected["open_path"])
            if selected and selected["exists"]
            else []
        )
        state = "detected" if present else "not_found"
        if holders:
            state = "in_use"
        return {
            "present": bool(present),
            "state": state,
            "device": selected["display_path"] if selected else None,
            "open_path": selected["open_path"] if selected else None,
            "by_id": selected["by_id"] if selected else None,
            "driver_hint": self._lidar_driver_hint(
                selected["display_path"] if selected else ""
            ),
            "holders": holders,
            "test_supported": False,
            "test_mode": "ros2_subscribe_only",
            "conflict_policy": "Dashboard는 LiDAR serial port를 열지 않습니다. ROS2 driver가 포트를 단독 소유하고 /scan을 발행합니다.",
            "last_test": self._last_lidar_test,
            "last_scan": self._last_lidar_scan,
        }

    def lidar_test(self) -> dict[str, Any]:
        status = self.lidar_status()
        if not status["present"]:
            result = {
                "ok": False,
                "state": "not_found",
                "message": "LiDAR 장치를 찾지 못했습니다. USB 연결과 전원을 확인하세요.",
                "status": status,
            }
            self._last_lidar_test = result
            return result
        if status["holders"]:
            result = {
                "ok": True,
                "state": "ros2_owned",
                "message": "LiDAR 포트가 다른 프로세스에 의해 열려 있습니다. ROS2 driver가 소유 중이면 정상입니다.",
                "status": status,
            }
            self._last_lidar_test = result
            return result
        result = {
            "ok": True,
            "state": "device_ready",
            "message": "LiDAR 장치는 보이지만 Dashboard는 포트를 열지 않습니다. ROS2 driver가 /scan을 발행하면 지도가 표시됩니다.",
            "status": status,
        }
        self._last_lidar_test = result
        return result

    def lidar_scan(self) -> dict[str, Any]:
        status = self.lidar_status()
        result = {
            "ok": False,
            "state": "disabled",
            "message": "Dashboard serial scan은 비활성화했습니다. ROS2 driver가 /scan을 발행하고 WebSocket bridge가 이를 구독해야 합니다.",
            "status": status,
            "points": [],
        }
        self._last_lidar_scan = result
        return result

    def camera_settings(self) -> dict[str, Any]:
        settings = _load_json(self.camera_settings_file)
        if not settings:
            driver = self._camera_driver_status()
            requested = (
                driver.get("requested")
                if isinstance(driver.get("requested"), dict)
                else {}
            )
            width = requested.get("width") or int(
                os.environ.get("CAMERA_WIDTH", "1280")
            )
            height = requested.get("height") or int(
                os.environ.get("CAMERA_HEIGHT", "720")
            )
            fps = requested.get("fps") or float(os.environ.get("CAMERA_FPS", "60"))
            settings = {
                "resolution": f"{int(width)}x{int(height)}",
                "width": int(width),
                "height": int(height),
                "fps": fps,
                "fourcc": requested.get("fourcc")
                or os.environ.get("CAMERA_FOURCC", "MJPG"),
                "source": "driver_default",
            }
        return {
            "ok": True,
            "settings": settings,
            "settings_file": str(self.camera_settings_file),
        }

    def set_camera_settings(self, payload: dict[str, Any]) -> dict[str, Any]:
        driver_status = self._camera_driver_status()
        modes = self._camera_supported_view_modes(driver_status)
        mode_by_resolution = {item["resolution"]: item for item in modes}
        resolution = str(payload.get("resolution") or "").strip().lower()
        if not resolution:
            try:
                width = int(payload.get("width"))
                height = int(payload.get("height"))
                resolution = f"{width}x{height}"
            except (TypeError, ValueError):
                return {
                    "ok": False,
                    "error": "camera_resolution_required",
                    "supported_view_modes": modes,
                }
        mode = mode_by_resolution.get(resolution)
        if mode is None:
            return {
                "ok": False,
                "error": "unsupported_camera_resolution",
                "resolution": resolution,
                "supported_view_modes": modes,
            }
        allowed_fps = [float(value) for value in mode.get("fps", [])]
        try:
            requested_fps = float(payload.get("fps"))
        except (TypeError, ValueError):
            requested_fps = allowed_fps[-1] if allowed_fps else 30.0
        if not any((abs(requested_fps - value) < 0.05 for value in allowed_fps)):
            return {
                "ok": False,
                "error": "unsupported_camera_fps",
                "resolution": resolution,
                "fps": requested_fps,
                "allowed_fps": mode.get("fps", []),
                "supported_view_modes": modes,
            }
        fps: int | float = (
            int(round(requested_fps))
            if abs(requested_fps - round(requested_fps)) < 0.05
            else round(requested_fps, 2)
        )
        fourcc = (
            str(payload.get("fourcc") or os.environ.get("CAMERA_FOURCC", "MJPG"))
            .strip()
            .upper()[:4]
            or "MJPG"
        )
        locked = self._camera_size_change_locked(int(mode["width"]), int(mode["height"]))
        if locked is not None:
            return {**locked, "supported_view_modes": modes}
        settings = {
            "updated_at": time.time(),
            "source": "dashboard",
            "resolution": mode["resolution"],
            "width": int(mode["width"]),
            "height": int(mode["height"]),
            "fps": fps,
            "fourcc": fourcc,
        }
        try:
            _write_json_atomic(self.camera_settings_file, settings)
        except OSError as exc:
            return {
                "ok": False,
                "error": str(exc),
                "settings_file": str(self.camera_settings_file),
            }
        return {
            "ok": True,
            "message": "camera capture settings saved; ROS2 camera driver will reopen the device",
            "settings": settings,
            "settings_file": str(self.camera_settings_file),
            "supported_view_modes": modes,
        }

    def _camera_size_change_locked(self, width: int, height: int) -> dict[str, Any] | None:
        """A frame-size change is refused unless the recorder is known not to record camera.

        The camera export keeps one frame size per recording and rejects a size
        change inside it, so a mid-recording resolution change would lose that
        recording's video. FPS-only changes keep the size and stay allowed.
        """
        current = self.camera_settings().get("settings", {})
        try:
            if (int(current.get("width")), int(current.get("height"))) == (width, height):
                return None
        except (TypeError, ValueError):
            pass
        recording = self.recording_status()
        if recording.get("active") is False:
            return None
        if recording.get("active") is True and recording.get("kind") == "lidar":
            return None
        error = (
            "camera_resolution_locked_while_recording"
            if recording.get("active") is True
            else "camera_resolution_change_needs_idle_recorder"
        )
        return {
            "ok": False,
            "error": error,
            "recording_id": recording.get("recording_id"),
            "recorder_state": recording.get("state"),
            "current_resolution": current.get("resolution"),
        }

    def camera_status(self) -> dict[str, Any]:
        driver_status = self._camera_driver_status()
        candidates = self._camera_candidates()
        present = [item for item in candidates if item["exists"]]
        selected = present[0] if present else candidates[0] if candidates else None
        holders = (
            self._device_holders(selected["open_path"])
            if selected and selected["exists"]
            else []
        )
        state = "detected" if present else "not_found"
        if holders:
            state = "in_use"
        elif not present and driver_status.get("usb_present"):
            state = "usb_detected_video_node_missing"
        supported_modes = self._camera_supported_view_modes(driver_status)
        return {
            "present": bool(present),
            "usb_present": bool(driver_status.get("usb_present")),
            "state": state,
            "device": selected["display_path"] if selected else None,
            "open_path": selected["open_path"] if selected else None,
            "by_id": selected["by_id"] if selected else None,
            "driver_hint": "UVC USB camera",
            "holders": holders,
            "driver_state": driver_status.get("state"),
            "driver_status_alive": driver_status.get("alive", False),
            "driver_status_age_sec": driver_status.get("age_sec"),
            "driver_last_error": driver_status.get("last_error"),
            "driver_last_recovery": driver_status.get("last_recovery"),
            "driver_requested": driver_status.get("requested", {}),
            "driver_actual": driver_status.get("actual", {}),
            "settings": self.camera_settings().get("settings", {}),
            "settings_file": str(self.camera_settings_file),
            "usb_devices": driver_status.get("usb_devices", []),
            "video_nodes": driver_status.get("video_nodes", []),
            "test_supported": False,
            "test_mode": "ros2_subscribe_only",
            "topic": "/camera/image_raw",
            "camera_info_topic": "/camera/camera_info",
            "supported_view_modes": supported_modes,
            "supported_view_modes_source": "v4l2"
            if driver_status.get("supported_view_modes")
            else "fallback",
            "conflict_policy": "Dashboard는 /dev/video 장치를 열지 않습니다. ROS2 camera driver가 장치를 소유하고 /camera/image_raw를 발행합니다.",
        }

    def rover_status(self) -> dict[str, Any]:
        from dashboard_server.rover_api import request

        return {
            **request({"op": "status"}),
            "control_page": "/rover-control",
            "mode": "esp_manual",
        }

    def set_rover_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        return {"ok": False, "error": "legacy_command_file_removed"}

    def stop_rover(self, payload=None):
        from dashboard_server.rover_api import request

        return request({"op": "stop"})

    def recording_status(self) -> dict[str, Any]:
        remote = self._recorder_get("/status")
        if remote.get("ok"):
            return {**remote, "status_fresh": True, "status_source": "recorder_api"}
        data = _load_json(self.data_recorder_status_file)
        if data:
            # This file is an event snapshot, not a heartbeat. Even a recent
            # mtime cannot prove that the recorder is still running or idle.
            updated = data.get("updated_at")
            age = time.time() - updated if type(updated) in (int, float) else None
            if age is not None and (not math.isfinite(age) or age < 0): age = None
            return {
                "ok": False, "active": None, "state": "unknown",
                "status_fresh": False, "status_source": "event_snapshot",
                "status_file": str(self.data_recorder_status_file),
                "status_url": f"{self.recorder_api_url}/status",
                "archive_count": self._recording_archive_count(),
                "last_known": data, "last_known_age_s": age,
                "error": remote.get("error") or "recorder_unavailable",
            }
        return {
            "ok": False,
            "active": None,
            "state": "not_connected",
            "status_fresh": False,
            "status_source": "unavailable",
            "status_url": f"{self.recorder_api_url}/status",
            "status_file": str(self.data_recorder_status_file),
            "archive_dir": str(self.recording_archive_dir),
            "archive_count": self._recording_archive_count(),
            "error": remote.get("error"),
        }

    def start_recording(self, payload: dict[str, Any]) -> dict[str, Any]:
        kind = str(payload.get("kind") or "").strip().lower()
        if kind not in {"lidar", "camera", "both"}:
            return {
                "ok": False,
                "state": "invalid_kind",
                "error": "kind must be lidar, camera, or both",
            }
        raw_include_imu = payload.get("include_imu", True)
        include_imu = (
            raw_include_imu
            if isinstance(raw_include_imu, bool)
            else str(raw_include_imu).strip().lower() in {"1", "true", "yes", "on"}
        )
        raw_gps = payload.get("include_gps", True)
        include_gps = (
            raw_gps
            if isinstance(raw_gps, bool)
            else str(raw_gps).lower() in {"1", "true", "yes", "on"}
        )
        result = self._recorder_post(
            "/start",
            {"kind": kind, "include_imu": include_imu, "include_gps": include_gps},
        )
        if result.get("ok"):
            return result
        return {
            "ok": False,
            "state": "recorder_unavailable",
            "error": result.get("error"),
            "status": self.recording_status(),
        }

    def zero_imu(self) -> dict[str, Any]:
        from dashboard_server.imu_processing import imu_zero_removed
        return imu_zero_removed()

    def stop_recording(self) -> dict[str, Any]:
        result = self._recorder_post("/stop", {})
        if result.get("ok"):
            recording_id = str(result.get("recording_id") or "")
            if recording_id:
                threading.Thread(
                    target=self._prewarm_playback_cache,
                    args=(recording_id,),
                    daemon=True,
                    name=f"playback-cache-{recording_id[:32]}",
                ).start()
            return result
        return {
            "ok": False,
            "state": "recorder_unavailable",
            "error": result.get("error"),
            "status": self.recording_status(),
        }

    def recordings(self) -> dict[str, Any]:
        rows: list[dict[str, Any]] = []
        if self.recording_archive_dir.exists():
            for path in self.recording_archive_dir.iterdir():
                if not path.is_dir():
                    continue
                metadata = _load_json(path / "metadata.json")
                timeline_summary = self._recording_timeline_summary(path, metadata)
                rows.append(
                    {
                        "id": metadata.get("id") or path.name,
                        "kind": metadata.get("kind") or "unknown",
                        "state": metadata.get("state") or "archived",
                        "started_at": metadata.get("started_at"),
                        **__import__("dashboard_server.recording_clock", fromlist=["display_clock"]).display_clock(path, metadata),
                        "finished_at": metadata.get("finished_at"),
                        "duration_s": metadata.get("duration_s"),
                        "path": str(path),
                        "timeline": timeline_summary,
                    }
                )
        return {
            "ok": True,
            "archive_dir": str(self.recording_archive_dir),
            "recordings": sorted(rows, key=__import__("dashboard_server.recording_clock", fromlist=["sort_key"]).sort_key, reverse=True),
        }

    def _recording_archive_count(self) -> int:
        if not self.recording_archive_dir.exists():
            return 0
        try:
            return sum(
                (1 for path in self.recording_archive_dir.iterdir() if path.is_dir())
            )
        except OSError:
            return 0

    def _recording_timeline_summary(
        self,
        path: Path,
        metadata: dict[str, Any],
        preview_scan: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        metadata_summary = (
            metadata.get("timeline")
            if isinstance(metadata.get("timeline"), dict)
            else {}
        )
        topics = (
            metadata.get("topics") if isinstance(metadata.get("topics"), list) else []
        )
        metadata_imu_frames = metadata_summary.get("imu_frames")
        if metadata_imu_frames is None and (
            not any(
                (
                    str(topic).startswith("/imu/")
                    or str(topic) == "/orchard/imu/calibration"
                    for topic in topics
                )
            )
        ):
            metadata_imu_frames = 0
        summary: dict[str, Any] = {
            "ok": True,
            "duration_s": metadata.get("duration_s"),
            "lidar_frames": metadata_summary.get("lidar_frames"),
            "camera_frames": metadata_summary.get("camera_frames"),
            "imu_frames": metadata_imu_frames,
            "gps_frames": metadata_summary.get("gps_frames", 0),
            "gps_valid_frames": metadata_summary.get("gps_valid_frames", 0),
            "gps_enabled": metadata.get("include_gps", False),
            "motor_frames": metadata_summary.get("motor_frames", 0),
            "motor_enabled": metadata_summary.get("motor_enabled", False),
            "motor_error": metadata_summary.get("motor_error"),
            "imu_calibration_valid": metadata_summary.get("imu_calibration_valid"),
            "imu_timeline_hz": metadata_summary.get("imu_timeline_hz"),
            "sample_hz": metadata_summary.get("sample_hz"),
            "algorithm": metadata.get("algorithm"),
            "coordinate_frame": (preview_scan or {}).get("coordinate_frame"),
            "source": "metadata",
        }
        if (
            summary["lidar_frames"] is not None or summary["camera_frames"] is not None
        ) and summary["imu_frames"] is not None:
            return summary
        timeline_path = path / "timeline.json"

        if not timeline_path.exists():
            summary.update({"ok": False, "source": "missing"})
            return summary
        try:
            if timeline_path.stat().st_size > 2000000:
                summary.update({"source": "deferred", "deferred": True})
                return summary
        except OSError:
            summary.update({"source": "deferred", "deferred": True})
            return summary
        timeline = _load_json(timeline_path)
        if not timeline:
            summary.update({"ok": False, "source": "unreadable"})
            return summary
        summary.update(
            {
                "ok": bool(timeline.get("ok")),
                "duration_s": timeline.get("duration_s") or summary.get("duration_s"),
                "lidar_frames": len(timeline.get("lidar", {}).get("frames", [])),
                "camera_frames": len(timeline.get("camera", {}).get("frames", [])),
                "imu_frames": len((timeline.get("imu") or {}).get("frames", [])),
                "gps_frames": (timeline.get("gps") or {}).get(
                    "frame_count", len((timeline.get("gps") or {}).get("frames", []))
                ),
                "motor_frames": (timeline.get("motor") or {}).get("frame_count", 0),
                "motor_enabled": bool((timeline.get("motor") or {}).get("enabled")),
                "motor_error": (timeline.get("motor") or {}).get("error"),
                "imu_calibration_valid": bool(
                    (timeline.get("imu") or {}).get("calibration", {}).get("valid")
                    or (timeline.get("imu") or {})
                    .get("calibration", {})
                    .get("calibration", {})
                    .get("valid")
                ),
                "algorithm": timeline.get("algorithm") or summary.get("algorithm"),
                "coordinate_frame": timeline.get("coordinate_frame")
                or summary.get("coordinate_frame"),
                "source": "timeline",
            }
        )
        return summary

    def recording_detail(self, recording_id: str) -> dict[str, Any]:
        path = self.recording_archive_path(recording_id)
        if path is None or not path.exists():
            return {"ok": False, "error": "recording_not_found", "id": recording_id}
        metadata = _load_json(path / "metadata.json")
        preview_scan = _load_json(path / "preview_scan.json")
        timeline_summary = self._recording_timeline_summary(
            path, metadata, preview_scan=preview_scan
        )
        source_video = self.recording_video(recording_id)
        adaptive = self.recording_adaptive_video(recording_id)
        if not source_video and adaptive:
            source_video = dict(fps=adaptive.get('fps'), width=adaptive.get('source_width'),
                                height=adaptive.get('source_height'), file='native.mp4',
                                timeline_offset_s=adaptive.get('timeline_offset_s'))
        public_video = {
            key: value for key, value in (source_video or {}).items() if key != "path"
        }
        return {
            "ok": True,
            "recording": {
                "id": metadata.get("id") or path.name,
                "kind": metadata.get("kind"),
                "metadata": metadata,
                "preview_scan": preview_scan,
                **__import__("dashboard_server.recording_clock", fromlist=["display_clock"]).display_clock(path, metadata),
                "timeline_summary": timeline_summary,
                "download_url": f"/api/recordings/{path.name}/download",
                "source_video": public_video or None,
                "adaptive_video": adaptive,
                "video_preparation": _load_json(path / "video_preparation.json"),
                "video_url": f"/api/recordings/{path.name}/video?v={source_video.get('bytes')}"
                if source_video and not adaptive
                else adaptive.get('fallback_url') if adaptive else None,
            },
        }

    def recording_adaptive_video(self, recording_id: str):
        from dashboard_server.adaptive_video import manifest, public_info
        archive = self.recording_archive_path(recording_id)
        if archive is None or not archive.is_dir(): return None
        value = manifest(archive)
        return public_info(archive, value) if value else None

    def recording_adaptive_asset(self, recording_id: str, filename: str):
        from dashboard_server.adaptive_video import manifest, asset_path, MIMES
        archive = self.recording_archive_path(recording_id)
        if archive is None or not archive.is_dir(): return None
        value = manifest(archive)
        path = asset_path(archive, value, filename) if value else None
        return dict(path=path, mime=MIMES[path.suffix]) if path else None

    def recording_video(self, recording_id: str) -> dict[str, Any] | None:
        archive = self.recording_archive_path(recording_id)
        if archive is None or not archive.is_dir():
            return None
        metadata = _load_json(archive / "metadata.json")
        source = metadata.get("source_video")
        if not isinstance(source, dict):
            return None
        raw_name = str(source.get("file") or "")
        filename = Path(raw_name).name
        if not filename or filename != raw_name:
            return None
        path = (archive / filename).resolve()
        try:
            if archive.resolve() not in path.parents or not path.is_file():
                return None
        except OSError:
            return None
        return {
            **source,
            "file": filename,
            "mime": str(source.get("mime") or "video/mp4"),
            "bytes": path.stat().st_size,
            "path": path,
        }

    def recording_timeline(
        self,
        recording_id: str,
        start_s: Any = None,
        duration_s: Any = None,
        profile: Any = None,
    ) -> dict[str, Any]:
        path = self.recording_archive_path(recording_id)
        if path is None or not path.exists():
            return {"ok": False, "error": "recording_not_found", "id": recording_id}
        timeline_path = path / "timeline.json"

        if profile == 'dashboard-telemetry-v1':
            return self._recording_light_telemetry(path,timeline_path,start_s,duration_s)

        def response(timeline):
            if not (timeline.get('lidar') or {}).get('frames'):
                from .lidar_preview_recovery import window
                try: recovered = window(path,self.recording_playback_cache_dir,start_s,duration_s)
                except (OSError,ValueError,TypeError,KeyError): recovered = None
                if recovered:
                    recovered = dict(recovered);recovered.pop('telemetry',None)
                    timeline = dict(timeline,lidar=recovered)
            timeline = self._with_recorded_gps(path, timeline, start_s, duration_s)
            timeline = with_recorded_motor(path, timeline, start_s, duration_s)
            if profile in (PLAYBACK_DASHBOARD_PROFILE, POLAR_PROFILE) and self.recording_adaptive_video(recording_id):
                timeline = dict(timeline, camera=dict(timeline.get('camera') or {}, frames=[]))
            return {
                "ok": True,
                "timeline": self._playback_response_profile(timeline, profile),
            }

        playback_request = self._playback_segment_request(start_s, duration_s)
        if playback_request is not None:
            cached_segment = self._cached_playback_segment(
                path, timeline_path, playback_request
            )
            if cached_segment:
                return response(cached_segment)
        try:
            timeline = self._load_recording_timeline(timeline_path)
        except TimelineLoadUnavailable as error:
            return {"ok": False, "error": str(error), "id": recording_id}
        if not timeline:
            # Missing previews are prepared explicitly offline. Never scan a
            # complete raw recording in this HTTP handler or steal live CPU.
            from .lidar_preview_recovery import window
            try: prepared = window(path,self.recording_playback_cache_dir,start_s,duration_s)
            except (OSError,ValueError,TypeError,KeyError): prepared = None
            if prepared is None:
                return {"ok": False, "error": "preview_recovery_required", "id": recording_id}
            timeline = prepared['telemetry']
            if not timeline:
                return {"ok": False, "error": "timeline_not_found", "id": recording_id}
            return response(self._recording_timeline_segment(timeline,start_s or 0,duration_s or 8))
        if start_s is not None or duration_s is not None:
            if playback_request is not None:
                cached_segment = self._build_playback_cache(
                    path, timeline_path, timeline, playback_request
                )
                if cached_segment:
                    self._drop_timeline_cache(timeline_path)
                    return response(cached_segment)
            segment = self._recording_timeline_segment(timeline, start_s, duration_s)
            return response(segment)
        return response(timeline)

    def _recording_light_telemetry(self,path,timeline_path,start_s,duration_s):
        from dashboard_server.telemetry_cache import light_segment
        try:
            start=float(start_s or 0);duration=float(duration_s or 24)
            if not math.isfinite(start) or not math.isfinite(duration):raise ValueError()
            start=max(0.0,start);duration=min(24.0,max(.5,duration))
        except (ValueError,TypeError):return dict(ok=False,error='invalid_telemetry_window')
        cache_dir=self._playback_cache_path(path,timeline_path)
        manifest=self._valid_playback_manifest(cache_dir,timeline_path) if cache_dir is not None else None
        first=max(0,int(start//PLAYBACK_SEGMENT_SECONDS)-1)
        last=int((start+duration)//PLAYBACK_SEGMENT_SECONDS)
        parts=[]
        if manifest and isinstance(manifest.get('segment_count'),int):
            # Existing small files remain compatible. Never manufacture them by
            # reparsing LiDAR-rich segment JSON on the light request path.
            indices=range(first,min(last+1,manifest['segment_count']))
            parts=[light_segment(cache_dir,index,cached_only=True) for index in indices]
        if not parts or any(part is None for part in parts):
            try:timeline=self._load_recording_light_timeline(timeline_path)
            except TimelineLoadUnavailable as error:return dict(ok=False,error=str(error))
            if not timeline:
                from .lidar_preview_recovery import window
                try: prepared=window(path,self.recording_playback_cache_dir,start,duration)
                except (OSError,ValueError,TypeError,KeyError): prepared=None
                if prepared is None:return dict(ok=False,error='preview_recovery_required')
                timeline=light_timeline(prepared['telemetry'])
            # Reuse the existing dedup/window response contract, but only over
            # the requested small sensor slice, never the LiDAR-rich timeline.
            parts=[self._recording_timeline_segment(timeline,start,duration)]
        combined={};frames={key:{} for key in ('imu','gps','motor')}
        for part in parts:
            combined.update({k:v for k,v in part.items() if k not in frames})
            for kind in frames:
                section=part.get(kind) or {}
                combined[kind]={k:v for k,v in section.items() if k!='frames'}
                for frame in section.get('frames',[]):frames[kind][json.dumps([frame.get('t_s'),frame.get('stamp')],sort_keys=True)]=frame
        for kind,rows in frames.items():combined.setdefault(kind,{})['frames']=sorted(rows.values(),key=lambda f:float(f.get('t_s',0)))
        result=self._recording_timeline_segment(combined,start,duration)
        result=self._with_recorded_gps(path,result,start,duration)
        result=with_recorded_motor(path,result,start,duration)
        result=self._playback_response_profile(result,POLAR_PROFILE)
        result['playback']['profile']='dashboard-telemetry-v1'
        return dict(ok=True,timeline=result)

    def _with_recorded_gps(self, path, timeline, start_s, duration_s):
        """Read the full GPS sidecar by time window, including old samples past the RAM tail."""
        gps_path = path / "gps.jsonl"
        if not gps_path.is_file():
            return timeline
        try:
            start = max(0.0, float(start_s)) if start_s is not None else 0.0
            end = (
                start + max(0.1, float(duration_s))
                if duration_s is not None
                else float("inf")
            )
        except (ValueError, TypeError):
            return timeline
        frames = []
        before = None
        try:
            with gps_path.open(encoding="utf-8") as source:
                for line in source:
                    try:
                        frame = json.loads(line)
                        t = float(frame["t_s"])
                        if not math.isfinite(t) or not isinstance(
                            frame.get("sample"), dict
                        ):
                            continue
                    except (ValueError, TypeError, KeyError):
                        continue
                    if t < start:
                        before = frame
                        continue
                    if not frames and before is not None:
                        frames.append(before)
                    frames.append(frame)
                    if t > end:
                        break
            if not frames and before is not None:
                frames.append(before)
        except OSError:
            return timeline
        return {
            **timeline,
            "gps": {**(timeline.get("gps") or {}), "frames": frames},
            "sync": {**(timeline.get("sync") or {}), "segment_gps_frames": len(frames)},
        }

    def _playback_response_profile(
        self, timeline: dict[str, Any], profile: Any
    ) -> dict[str, Any]:
        """Compact only the dashboard playback response; the archive stays lossless.

        The default timeline API remains unchanged for ROS/research consumers.  The
        browser asks for this profile because repeated JSON object keys in LiDAR and
        complete IMU covariance/orientation records add substantial parse work while
        not being used by the playback canvases.
        """
        if str(profile or "") not in (PLAYBACK_DASHBOARD_PROFILE, POLAR_PROFILE):
            lidar = timeline.get('lidar') or {}
            if lidar.get('source') == 'verified_raw_preview_recovery':
                from .playback_profile import FULL_FIELDS
                return dict(timeline,lidar=dict(lidar,frames=[dict(f,points=[
                    dict(zip(FULL_FIELDS,p)) for p in f.get('points',[])]) for f in lidar.get('frames',[])]))
            return timeline
        result = dict(timeline)
        lidar = dict(timeline.get("lidar") or {})
        fields = point_fields(lidar.get('frames') or [], profile)
        compact_lidar_frames: list[dict[str, Any]] = []
        for source_frame in lidar.get("frames", []) or []:
            frame = dict(source_frame)
            points = frame.get("points") or []
            if points and isinstance(points[0], dict):
                frame["points"] = [
                    [point.get(field) for field in fields]
                    for point in points
                ]
            compact_lidar_frames.append(frame)
        lidar["frames"] = compact_lidar_frames
        result["lidar"] = lidar
        imu = dict(timeline.get("imu") or {})
        compact_imu_frames: list[dict[str, Any]] = []
        for source_frame in imu.get("frames", []) or []:
            frame = dict(source_frame)
            for sample_name in ("raw", "corrected"):
                sample = frame.get(sample_name)
                if not isinstance(sample, dict):
                    continue
                frame[sample_name] = {
                    key: sample[key]
                    for key in (
                        "frame_id",
                        "euler_deg",
                        "angular_velocity",
                        "linear_acceleration",
                    )
                    if key in sample
                }
            magnetic = frame.get("magnetic")
            if isinstance(magnetic, dict):
                frame["magnetic"] = {
                    key: magnetic[key]
                    for key in ("frame_id", "magnetic_field_uT")
                    if key in magnetic
                }
            compact_imu_frames.append(frame)
        imu["frames"] = compact_imu_frames
        result["imu"] = imu
        playback = dict(timeline.get("playback") or {})
        playback.update(
            {
                "profile": profile,
                "lidar_point_fields": list(fields),
                "archive_source_preserved": True,
            }
        )
        result["playback"] = playback
        return result

    def recording_camera_frame(
        self, recording_id: str, filename: str
    ) -> dict[str, Any] | None:
        if not re.fullmatch("frame-[0-9a-f]{24}\\.jpg", str(filename or "")):
            return None
        archive = self.recording_archive_path(recording_id)
        if archive is None or not archive.is_dir():
            return None
        timeline_path = archive / "timeline.json"
        cache_dir = self._playback_cache_path(archive, timeline_path)
        if cache_dir is None:
            return None
        path = (cache_dir / "frames" / filename).resolve()
        try:
            frame_root = (cache_dir / "frames").resolve()
            if frame_root not in path.parents or not path.is_file():
                return None
        except OSError:
            return None
        return {
            "path": path,
            "mime": "image/jpeg",
            "bytes": path.stat().st_size,
            "etag": filename.removeprefix("frame-").removesuffix(".jpg"),
        }

    def _playback_segment_request(
        self, start_s: Any, duration_s: Any
    ) -> tuple[float, int] | None:
        if start_s is None or duration_s is None:
            return None
        try:
            start = max(0.0, float(start_s))
            duration = float(duration_s)
        except (TypeError, ValueError):
            return None
        if abs(duration - PLAYBACK_SEGMENT_SECONDS) > 0.001:
            return None
        index = int(round(start / PLAYBACK_SEGMENT_SECONDS))
        if abs(start - index * PLAYBACK_SEGMENT_SECONDS) > 0.001:
            return None
        return (start, index)

    def _playback_source_signature(self, timeline_path: Path) -> tuple[int, int] | None:
        try:
            stat = timeline_path.stat()
        except OSError:
            return None
        return (stat.st_mtime_ns, stat.st_size)

    def _playback_cache_path(self, archive: Path, timeline_path: Path) -> Path | None:
        signature = self._playback_source_signature(timeline_path)
        if signature is None:
            return None
        token = f"{signature[0]}-{signature[1]}-{PLAYBACK_CACHE_REVISION}"
        return self.recording_playback_cache_dir / archive.name / token

    def _valid_playback_manifest(
        self, cache_dir: Path, timeline_path: Path
    ) -> dict[str, Any] | None:
        manifest = _load_json(cache_dir / "manifest.json")
        signature = self._playback_source_signature(timeline_path)
        if (
            not manifest
            or signature is None
            or manifest.get("schema") != PLAYBACK_CACHE_SCHEMA
            or (manifest.get("revision") != PLAYBACK_CACHE_REVISION)
            or (manifest.get("source_mtime_ns") != signature[0])
            or (manifest.get("source_bytes") != signature[1])
            or (
                float(manifest.get("segment_duration_s") or 0.0)
                != PLAYBACK_SEGMENT_SECONDS
            )
        ):
            return None
        return manifest

    def _cached_playback_segment(
        self, archive: Path, timeline_path: Path, request: tuple[float, int]
    ) -> dict[str, Any] | None:
        cache_dir = self._playback_cache_path(archive, timeline_path)
        if (
            cache_dir is None
            or self._valid_playback_manifest(cache_dir, timeline_path) is None
        ):
            return None
        _, index = request
        return _load_json(cache_dir / "segments" / f"segment-{index:06d}.json") or None

    def _build_playback_cache(
        self,
        archive: Path,
        timeline_path: Path,
        timeline: dict[str, Any],
        request: tuple[float, int],
    ) -> dict[str, Any] | None:
        with self._playback_cache_lock:
            cached = self._cached_playback_segment(archive, timeline_path, request)
            if cached:
                return cached
            cache_dir = self._playback_cache_path(archive, timeline_path)
            signature = self._playback_source_signature(timeline_path)
            if cache_dir is None or signature is None:
                return None
            frames_dir = cache_dir / "frames"
            segments_dir = cache_dir / "segments"
            frames_dir.mkdir(parents=True, exist_ok=True)
            segments_dir.mkdir(parents=True, exist_ok=True)
            duration = float(timeline.get("duration_s") or 0.0)
            if duration <= 0.0:
                all_frames = [
                    *(timeline.get("lidar", {}).get("frames", []) or []),
                    *(timeline.get("camera", {}).get("frames", []) or []),
                    *((timeline.get("imu") or {}).get("frames", []) or []),
                ]
                duration = max(
                    (float(frame.get("t_s") or 0.0) for frame in all_frames),
                    default=0.0,
                )
            segment_count = max(
                1, int(math.ceil(max(0.001, duration) / PLAYBACK_SEGMENT_SECONDS))
            )
            camera_files: set[str] = set()
            for index in range(segment_count):
                start = index * PLAYBACK_SEGMENT_SECONDS
                segment = self._recording_timeline_segment(
                    timeline, start, PLAYBACK_SEGMENT_SECONDS
                )
                for frame in segment.get("camera", {}).get("frames", []):
                    encoded = frame.get("data")
                    if not encoded:
                        continue
                    try:
                        jpeg = base64.b64decode(encoded, validate=True)
                    except (ValueError, TypeError):
                        continue
                    frame.pop("data", None)
                    digest = hashlib.sha256(jpeg).hexdigest()[:24]
                    filename = f"frame-{digest}.jpg"
                    frame_path = frames_dir / filename
                    if not frame_path.exists():
                        tmp_path = frame_path.with_suffix(".jpg.tmp")
                        tmp_path.write_bytes(jpeg)
                        tmp_path.replace(frame_path)
                    camera_files.add(filename)
                    frame["frame_url"] = (
                        f"/api/recordings/{archive.name}/camera-frame/{filename}?v={signature[0]}-{signature[1]}"
                    )
                    frame["transport"] = "binary-jpeg-http-cache-v1"
                segment["playback"] = {
                    "schema": PLAYBACK_CACHE_SCHEMA,
                    "revision": PLAYBACK_CACHE_REVISION,
                    "segment_duration_s": PLAYBACK_SEGMENT_SECONDS,
                    "camera_transport": "binary-jpeg-http-cache-v1",
                    "source_timeline_preserved": True,
                }
                segment_path = segments_dir / f"segment-{index:06d}.json"
                tmp_segment = segment_path.with_suffix(".json.tmp")
                tmp_segment.write_text(
                    json.dumps(
                        segment,
                        ensure_ascii=False,
                        separators=(",", ":"),
                        allow_nan=False,
                    ),
                    encoding="utf-8",
                )
                tmp_segment.replace(segment_path)
            manifest = {
                "schema": PLAYBACK_CACHE_SCHEMA,
                "revision": PLAYBACK_CACHE_REVISION,
                "recording_id": archive.name,
                "source_mtime_ns": signature[0],
                "source_bytes": signature[1],
                "segment_duration_s": PLAYBACK_SEGMENT_SECONDS,
                "segment_count": segment_count,
                "camera_file_count": len(camera_files),
                "created_at": time.time(),
                "source_timeline_preserved": True,
            }
            _write_json_atomic(cache_dir / "manifest.json", manifest)
            return self._cached_playback_segment(archive, timeline_path, request)

    def _drop_timeline_cache(self, timeline_path: Path) -> None:
        try:
            key = str(timeline_path.resolve())
        except OSError:
            key = str(timeline_path)
        with self._timeline_cache_lock:
            self._timeline_cache.pop(key, None)
            # The bounded light projection retains no LiDAR/camera references.
            # Dropping the expensive full view must not force another cold parse.

    def _prewarm_playback_cache(self, recording_id: str) -> None:
        archive = self.recording_archive_path(recording_id)
        if archive is None or not archive.is_dir():
            return
        timeline_path = archive / "timeline.json"
        try:
            timeline = self._load_recording_timeline(timeline_path)
        except TimelineLoadUnavailable:
            return
        if not timeline:
            return
        try:
            self._build_playback_cache(archive, timeline_path, timeline, (0.0, 0))
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            return
        finally:
            self._drop_timeline_cache(timeline_path)

    def _load_recording_timeline(self, timeline_path: Path) -> dict[str, Any]:
        deadline = time.monotonic() + 30.0
        signature = self._playback_source_signature(timeline_path)
        if signature is None:
            return {}
        key = str(timeline_path.resolve())
        flight_key = (key, *signature)
        with self._timeline_cache_lock:
            cached = self._timeline_cache.get(key)
            if cached and cached[:2] == signature:
                self._timeline_cache.move_to_end(key)
                return cached[2]
            future = self._timeline_loads.get(flight_key)
        if future is not None:
            return self._wait_for_timeline_load(future, timeline_path, signature, deadline)
        owns_slot = self._timeline_load_slots.acquire(blocking=False)
        if not owns_slot:
            if not self._timeline_wait_slots.acquire(blocking=False):
                raise TimelineLoadUnavailable('timeline_load_busy')
            try:
                owns_slot = self._timeline_load_slots.acquire(timeout=max(0.0, deadline - time.monotonic()))
            finally:
                self._timeline_wait_slots.release()
            if not owns_slot:
                raise TimelineLoadUnavailable('timeline_load_timeout')
        future = None
        owner = False
        try:
            # Admission wait is outside the shared lock. Never publish a parse
            # against the signature seen before a source replacement.
            if self._playback_source_signature(timeline_path) != signature:
                raise TimelineLoadUnavailable('timeline_source_changed')
            with self._timeline_cache_lock:
                cached = self._timeline_cache.get(key)
                if cached and cached[:2] == signature:
                    self._timeline_cache.move_to_end(key)
                    return cached[2]
                future = self._timeline_loads.get(flight_key)
                if future is None:
                    future = Future()
                    self._timeline_loads[flight_key] = future
                    owner = True
            if not owner:
                self._timeline_load_slots.release()
                owns_slot = False
                return self._wait_for_timeline_load(future, timeline_path, signature, deadline)
            timeline = _load_json(timeline_path)
            # Loading/parsing occurs in this request, outside all cache locks.
            with self._timeline_cache_lock:
                # The metadata check belongs at publication, after acquiring
                # this short lock; an older owner cannot overwrite a newer
                # generation that finished while it waited for the lock.
                if self._playback_source_signature(timeline_path) != signature:
                    raise TimelineLoadUnavailable('timeline_source_changed')
                if timeline:
                    self._timeline_cache[key] = (*signature, timeline)
                    self._timeline_cache.move_to_end(key)
                    self._light_timeline_cache[key] = (*signature, light_timeline(timeline))
                    self._light_timeline_cache.move_to_end(key)
                    while len(self._timeline_cache) > self._timeline_cache_limit:
                        self._timeline_cache.popitem(last=False)
                    while len(self._light_timeline_cache) > self._timeline_cache_limit:
                        self._light_timeline_cache.popitem(last=False)
                future.set_result(timeline)
            if self._playback_source_signature(timeline_path) != signature:
                raise TimelineLoadUnavailable('timeline_source_changed')
            return timeline
        except BaseException as error:
            if owner and future is not None and not future.done():
                future.set_exception(error)
            raise
        finally:
            if owner:
                with self._timeline_cache_lock:
                    if self._timeline_loads.get(flight_key) is future:
                        self._timeline_loads.pop(flight_key)
            if owns_slot:
                self._timeline_load_slots.release()

    def _wait_for_timeline_load(self, future, timeline_path, signature, deadline=None):
        if deadline is None:
            deadline = time.monotonic() + 30.0
        if not self._timeline_wait_slots.acquire(blocking=False):
            raise TimelineLoadUnavailable('timeline_load_busy')
        try:
            try:
                timeline = future.result(timeout=max(0.0, deadline - time.monotonic()))
            except FutureTimeoutError:
                if not future.done():
                    raise TimelineLoadUnavailable('timeline_load_timeout')
                # An owner's TimeoutError is an owner failure, not our wait
                # deadline. Re-read a completed Future to preserve its outcome.
                timeline = future.result()
            if self._playback_source_signature(timeline_path) != signature:
                raise TimelineLoadUnavailable('timeline_source_changed')
            return timeline
        finally:
            self._timeline_wait_slots.release()

    def _load_recording_light_timeline(self, timeline_path: Path) -> dict[str, Any]:
        signature = self._playback_source_signature(timeline_path)
        if signature is None:
            return {}
        key = str(timeline_path.resolve())
        with self._timeline_cache_lock:
            cached = self._light_timeline_cache.get(key)
            if cached and cached[:2] == signature:
                self._light_timeline_cache.move_to_end(key)
                return cached[2]
        timeline = self._load_recording_timeline(timeline_path)
        if self._playback_source_signature(timeline_path) != signature:
            raise TimelineLoadUnavailable('timeline_source_changed')
        return light_timeline(timeline)

    def _recording_timeline_segment(
        self, timeline: dict[str, Any], start_s: Any = None, duration_s: Any = None
    ) -> dict[str, Any]:
        try:
            start = max(0.0, float(start_s if start_s is not None else 0.0))
        except (TypeError, ValueError):
            start = 0.0
        try:
            duration = float(duration_s if duration_s is not None else 8.0)
        except (TypeError, ValueError):
            duration = 8.0
        duration = max(0.5, min(60.0, duration))
        end = start + duration
        payload = {
            key: value
            for key, value in timeline.items()
            if key not in {"lidar", "camera", "imu"}
        }
        payload["segment"] = {
            "start_s": round(start, 3),
            "end_s": round(end, 3),
            "duration_s": round(duration, 3),
            "partial": True,
        }
        lidar_frames = self._timeline_frames_between(
            timeline.get("lidar", {}).get("frames", []), start, end
        )
        payload["lidar"] = {"frames": self._raw_sensor_frames(lidar_frames)}
        camera_frames = self._timeline_frames_between(
            timeline.get("camera", {}).get("frames", []), start, end
        )
        camera_rows = self._raw_sensor_frames(camera_frames)
        native_video = bool(
            timeline.get("video") or (timeline.get("sync") or {}).get("native_video")
        )
        if native_video:
            camera_rows = [
                {key: value for key, value in frame.items() if key != "data"}
                for frame in camera_rows
            ]
        payload["camera"] = {"frames": camera_rows}
        imu_frames = self._timeline_frames_between(
            timeline.get("imu", {}).get("frames", []), start, end
        )
        payload["imu"] = {
            **{
                key: value
                for key, value in (timeline.get("imu") or {}).items()
                if key != "frames"
            },
            "frames": imu_frames,
        }
        payload["gps"] = {
            **{
                key: value
                for key, value in (timeline.get("gps") or {}).items()
                if key != "frames"
            },
            "frames": self._timeline_frames_between(
                (timeline.get("gps") or {}).get("frames", []), start, end
            ),
        }
        if isinstance(timeline.get("motor"), dict):
            motor = timeline["motor"]
            rows, previous = [], None
            for frame in motor.get("frames", []):
                try:
                    stamp = float(frame["t_s"])
                except (KeyError, TypeError, ValueError):
                    continue
                if not math.isfinite(stamp):
                    continue
                if stamp < start:
                    previous = frame
                    continue
                if stamp > end:
                    break
                if not rows and previous is not None:
                    rows.append(previous)
                rows.append(frame)
            if not rows and previous is not None:
                rows.append(previous)
            payload["motor"] = {**motor, "frames": rows}
        payload["sync"] = {
            **(timeline.get("sync") or {}),
            "segment_lidar_frames": len(payload["lidar"]["frames"]),
            "segment_camera_frames": len(payload["camera"]["frames"]),
            "segment_imu_frames": len(payload["imu"]["frames"]),
            "segment_gps_frames": len(payload["gps"]["frames"]),
            "segment_motor_frames": len((payload.get("motor") or {}).get("frames", [])),
        }
        return payload

    def _raw_sensor_frames(self, frames):
        return [
            {
                k: v
                for k, v in frame.items()
                if k
                not in {"overlay", "analysis", "vision_sync", "inference_input_policy"}
            }
            for frame in frames
        ]

    def _timeline_frames_between(
        self, frames: list[dict[str, Any]], start: float, end: float
    ) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        previous: dict[str, Any] | None = None
        for frame in frames:
            try:
                stamp = float(frame.get("t_s") or 0.0)
            except (TypeError, ValueError):
                continue
            if stamp < start:
                previous = frame
                continue
            if stamp >= end:
                break
            if previous and (not rows):
                rows.append(previous)
            rows.append(frame)
        return rows

    def recording_archive_path(self, recording_id: str) -> Path | None:
        safe = re.sub("[^A-Za-z0-9_.-]", "", str(recording_id or ""))
        if not safe:
            return None
        path = (self.recording_archive_dir / safe).resolve()
        try:
            root = self.recording_archive_dir.resolve()
        except OSError:
            root = self.recording_archive_dir
        if root != path and root not in path.parents:
            return None
        return path

    def _recorder_get(self, path: str) -> dict[str, Any]:
        try:
            with urllib.request.urlopen(
                f"{self.recorder_api_url}{path}", timeout=1.0
            ) as res:
                data = json.loads(res.read().decode("utf-8"))
                data.setdefault("ok", 200 <= int(res.status) < 300)
                return data
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def _recorder_post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        req = urllib.request.Request(
            f"{self.recorder_api_url}{path}",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=2.0) as res:
                data = json.loads(res.read().decode("utf-8"))
                data.setdefault("ok", 200 <= int(res.status) < 300)
                return data
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def _bounded_float(
        self, value: Any, low: float, high: float, default: float = 0.0
    ) -> float:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            numeric = default
        if not math.isfinite(numeric):
            numeric = default
        return round(max(low, min(high, numeric)), 3)

    def fan_status(self) -> dict[str, Any]:
        self._enforce_fan_state()
        return self._fan_status_snapshot()

    def _fan_status_snapshot(self) -> dict[str, Any]:
        hwmon = self._fan_hwmon()
        nvfancontrol = self._fan_nvfancontrol_status()
        temperature_c = self._max_temperature()
        desired = self._fan_desired_state()
        if not hwmon:
            return {
                "ok": False,
                "available": False,
                "state": "not_found",
                "message": "Jetson 팬 제어 장치를 찾지 못했습니다.",
                "mode": "auto" if nvfancontrol.get("active") else "unknown",
                "nvfancontrol": nvfancontrol,
                "temperature_c": temperature_c,
                "rpm": None,
                "pwm": None,
                "pwm_enable": None,
                "paths": {},
                "desired": desired,
            }
        pwm = _safe_int(_read_text(hwmon.get("pwm_path")))
        pwm_enable = _safe_int(_read_text(hwmon.get("enable_path")))
        rpm = _safe_int(_read_text(hwmon.get("rpm_path")))
        mode = "auto" if nvfancontrol.get("active") else "manual"
        if not desired and mode == "manual" and (pwm is not None):
            self._save_fan_desired_state(
                {"mode": "manual", "pwm": pwm, "adopted": True}
            )
            desired = self._fan_desired_state()
        return {
            "ok": True,
            "available": True,
            "state": "ready",
            "message": "팬 제어 장치를 확인했습니다.",
            "mode": mode,
            "nvfancontrol": nvfancontrol,
            "temperature_c": temperature_c,
            "rpm": rpm,
            "pwm": pwm,
            "pwm_enable": pwm_enable,
            "paths": {
                "name": str(hwmon.get("name_path") or ""),
                "pwm": str(hwmon.get("pwm_path") or ""),
                "enable": str(hwmon.get("enable_path") or ""),
                "rpm": str(hwmon.get("rpm_path") or ""),
                "hwmon": str(hwmon.get("root") or ""),
                "name_value": hwmon.get("name"),
            },
            "presets": self._fan_presets(),
            "safety": self._fan_safety(temperature_c),
            "desired": desired,
        }

    def set_fan_mode(self, auto: bool) -> dict[str, Any]:
        if auto:
            self._save_fan_desired_state({"mode": "auto"})
            result = self._fan_systemctl("start")
            status = self._fan_status_snapshot()
            return {
                "ok": bool(result.get("ok")),
                "action": "auto",
                "systemctl": result,
                "status": status,
            }
        hwmon = self._fan_hwmon()
        if not hwmon:
            return {
                "ok": False,
                "state": "not_found",
                "message": "팬 제어 장치를 찾지 못했습니다.",
                "status": self.fan_status(),
            }
        stop = self._fan_systemctl("stop")
        enable = self._write_fan_value(hwmon.get("enable_path"), "1")
        current = self._fan_status_snapshot()
        pwm = current.get("pwm")
        self._save_fan_desired_state({"mode": "manual", "pwm": pwm})
        ok = bool(enable.get("ok"))
        return {
            "ok": ok,
            "action": "manual",
            "systemctl": stop,
            "enable": enable,
            "status": self._fan_status_snapshot(),
        }

    def set_fan_pwm(self, pwm: Any, preset: str | None = None) -> dict[str, Any]:
        hwmon = self._fan_hwmon()
        if not hwmon:
            return {
                "ok": False,
                "state": "not_found",
                "message": "팬 제어 장치를 찾지 못했습니다.",
                "status": self.fan_status(),
            }
        try:
            value = int(pwm)
        except (TypeError, ValueError):
            value = 0
        value = max(0, min(255, value))
        status = self._fan_status_snapshot()
        safety = self._fan_pwm_safety(value, status.get("temperature_c"))
        if not safety["ok"]:
            return {"ok": False, **safety, "status": status}
        stop = (
            self._fan_systemctl("stop")
            if status.get("mode") == "auto"
            else {"ok": True, "state": "already_manual"}
        )
        enable = self._write_fan_value(hwmon.get("enable_path"), "1")
        write = self._write_fan_value(hwmon.get("pwm_path"), str(value))
        if write.get("ok"):
            self._save_fan_desired_state(
                {"mode": "manual", "pwm": value, "preset": preset}
            )
        return {
            "ok": bool(write.get("ok")),
            "pwm": value,
            "preset": preset,
            "systemctl": stop,
            "enable": enable,
            "write": write,
            "status": self._fan_status_snapshot(),
        }

    def set_fan_preset(self, preset: str) -> dict[str, Any]:
        presets = self._fan_presets()
        if preset not in presets:
            return {
                "ok": False,
                "state": "bad_preset",
                "message": f"Unknown fan preset: {preset}",
                "presets": presets,
            }
        result = self.set_fan_pwm(presets[preset], preset=preset)
        result["preset"] = preset
        return result

    def _fan_hwmon(self) -> dict[str, Any] | None:
        if self._fan_hwmon_cache:
            pwm_path = self._fan_hwmon_cache.get("pwm_path")
            if pwm_path and Path(pwm_path).exists():
                return self._fan_hwmon_cache
            self._fan_hwmon_cache = None

        def inspect(root: Path) -> dict[str, Any] | None:
            name_path = root / "name"
            name = _read_text(name_path)
            pwm_path = root / "pwm1"
            if not pwm_path.exists():
                return None
            if name not in {"pwmfan", "pwm-fan"} and "pwm" not in (name or ""):
                return None
            rpm_path = root / "fan1_input"
            if not rpm_path.exists():
                fallback = root / "rpm"
                rpm_path = fallback if fallback.exists() else rpm_path
            return {
                "root": root,
                "name": name,
                "name_path": name_path,
                "pwm_path": pwm_path,
                "enable_path": root / "pwm1_enable",
                "rpm_path": rpm_path,
            }

        hwmon_root = self.sys_root / "class/hwmon"
        if hwmon_root.exists():
            for root in sorted(hwmon_root.glob("hwmon*")):
                found = inspect(root)
                if found:
                    self._fan_hwmon_cache = found
                    return found
        for root in sorted(
            self.sys_root.glob("devices/platform/*pwm*fan*/hwmon/hwmon*")
        ):
            found = inspect(root)
            if found:
                self._fan_hwmon_cache = found
                return found
        return None

    def _fan_nvfancontrol_status(self) -> dict[str, Any]:
        return self._fan_systemctl("is-active")

    def _fan_systemctl(self, action: str) -> dict[str, Any]:
        if action not in {"start", "stop", "is-active"}:
            return {"ok": False, "state": "bad_action", "action": action}
        systemctl = shutil.which("systemctl")
        nsenter = shutil.which("nsenter")
        if systemctl:
            command = [systemctl, action, "nvfancontrol"]
            runner = "systemctl"
        elif nsenter:
            command = [
                nsenter,
                "-t",
                "1",
                "-m",
                "-u",
                "-i",
                "-n",
                "-p",
                "systemctl",
                action,
                "nvfancontrol",
            ]
            runner = "nsenter-systemctl"
        else:
            return {
                "ok": False,
                "known": False,
                "active": False,
                "state": "systemctl_missing",
                "action": action,
            }
        try:
            proc = subprocess.run(
                command, check=False, capture_output=True, text=True, timeout=4
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return {
                "ok": False,
                "known": False,
                "active": False,
                "state": "systemctl_error",
                "error": str(exc),
                "action": action,
            }
        output = (proc.stdout or proc.stderr or "").strip()
        active = action == "is-active" and output == "active" and (proc.returncode == 0)
        return {
            "ok": proc.returncode == 0,
            "known": True,
            "active": active or (action == "start" and proc.returncode == 0),
            "state": output or ("ok" if proc.returncode == 0 else "failed"),
            "returncode": proc.returncode,
            "action": action,
            "runner": runner,
        }

    def _write_fan_value(self, path: Path | None, value: str) -> dict[str, Any]:
        hwmon = self._fan_hwmon()
        allowed = (
            {str(hwmon.get("pwm_path")), str(hwmon.get("enable_path"))}
            if hwmon
            else set()
        )
        if not path or str(path) not in allowed:
            return {"ok": False, "state": "path_not_allowed", "path": str(path or "")}
        target = Path(path)
        if target.name == "pwm1_enable" and (not target.exists()):
            return {
                "ok": True,
                "state": "not_present",
                "path": str(path),
                "skipped": True,
            }
        try:
            target.write_text(str(value), encoding="utf-8")
        except OSError as exc:
            return {
                "ok": False,
                "state": "write_error",
                "path": str(path),
                "error": str(exc),
            }
        return {"ok": True, "path": str(path), "value": str(value)}

    def _fan_presets(self) -> dict[str, int]:
        defaults = {"off": 0, "quiet": 80, "normal": 160, "max": 255}
        result = {}
        for key, default in defaults.items():
            env_key = f"FAN_PWM_{key.upper()}"
            try:
                result[key] = max(0, min(255, int(os.environ.get(env_key, default))))
            except ValueError:
                result[key] = default
        return result

    def _fan_safety(self, temperature_c: float | None) -> dict[str, Any]:
        if temperature_c is None:
            return {"level": "unknown", "message": "Temperature sensor unavailable."}
        if temperature_c >= 80:
            return {
                "level": "critical",
                "message": "80°C 이상입니다. 자동 제어 또는 PWM 255를 권장합니다.",
            }
        if temperature_c >= 75:
            return {
                "level": "hot",
                "message": "75°C 이상에서는 낮은 PWM 설정을 제한합니다.",
            }
        if temperature_c >= 50:
            return {
                "level": "warm",
                "message": "50°C 이상에서는 fan off preset을 제한합니다.",
            }
        return {"level": "normal", "message": "현재 온도는 정상 범위입니다."}

    def _fan_pwm_safety(self, pwm: int, temperature_c: float | None) -> dict[str, Any]:
        if temperature_c is None:
            return {"ok": True, "state": "temperature_unknown"}
        if temperature_c >= 75 and pwm < 160:
            return {
                "ok": False,
                "state": "temperature_guard",
                "message": "75°C 이상에서는 PWM 160 미만을 적용하지 않습니다.",
            }
        if temperature_c >= 50 and pwm == 0:
            return {
                "ok": False,
                "state": "temperature_guard",
                "message": "50°C 이상에서는 팬 끄기를 적용하지 않습니다.",
            }
        return {"ok": True, "state": "safe"}

    def _fan_state_path(self) -> Path:
        raw = os.environ.get("FAN_STATE_FILE", str(DEFAULT_FAN_STATE_FILE))
        path = Path(raw)
        return path if path.is_absolute() else ROOT / path

    def _host_boot_id(self) -> str | None:
        return _read_text(self.proc_root / "sys/kernel/random/boot_id") or _read_text(
            "/proc/sys/kernel/random/boot_id"
        )

    def _fan_desired_state(self) -> dict[str, Any]:
        data = _load_json(self._fan_state_path())
        if data.get("mode") not in {"auto", "manual"}:
            return {"mode": "auto", "default": True}
        if data.get("mode") == "manual":
            current_boot = self._host_boot_id()
            saved_boot = data.get("boot_id")
            if current_boot and saved_boot and (saved_boot != current_boot):
                self._save_fan_desired_state(
                    {"mode": "auto", "recovered_after_reboot": True}
                )
                return {"mode": "auto", "recovered_after_reboot": True}
            if current_boot and (not saved_boot):
                self._save_fan_desired_state(
                    {"mode": "auto", "recovered_from_legacy_manual_state": True}
                )
                return {"mode": "auto", "recovered_from_legacy_manual_state": True}
        return data

    def _save_fan_desired_state(self, payload: dict[str, Any]) -> None:
        data = {
            **payload,
            "boot_id": self._host_boot_id(),
            "updated_at": datetime.now(timezone.utc)
            .astimezone()
            .isoformat(timespec="seconds"),
        }
        path = self._fan_state_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(f"{path.suffix}.tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(path)

    def _enforce_fan_state(self) -> None:
        desired = self._fan_desired_state()
        if not desired:
            return
        mode = desired.get("mode")
        if mode == "auto":
            status = self._fan_status_snapshot()
            if not status.get("available"):
                return
            if status.get("mode") != "auto":
                self._fan_systemctl("start")
            return
        if mode != "manual":
            return
        pwm = desired.get("pwm")
        if pwm is None:
            return
        try:
            value = max(0, min(255, int(pwm)))
        except (TypeError, ValueError):
            return
        hwmon = self._fan_hwmon()
        if not hwmon:
            return
        safety = self._fan_pwm_safety(value, self._max_temperature())
        if not safety.get("ok"):
            return
        current_pwm = _safe_int(_read_text(hwmon.get("pwm_path")))
        if current_pwm != value:
            self._fan_systemctl("stop")
            self._write_fan_value(hwmon.get("enable_path"), "1")
            self._write_fan_value(hwmon.get("pwm_path"), str(value))

    def _fan_hold_loop(self) -> None:
        while True:
            try:
                self._enforce_fan_state()
            except Exception:
                pass
            time.sleep(self._fan_hold_interval_s)

    def _lidar_device_root(self) -> Path:
        root = Path(os.environ.get("LIDAR_DEV_ROOT", "/host-dev"))
        return root if root.exists() else Path("/dev")

    def _camera_device_root(self) -> Path:
        root = Path(
            os.environ.get(
                "CAMERA_DEV_ROOT", os.environ.get("LIDAR_DEV_ROOT", "/host-dev")
            )
        )
        return root if root.exists() else Path("/dev")

    def _host_device_label(self, path: Path, resolve: bool = True) -> str:
        root = self._lidar_device_root()
        try:
            target = path.resolve() if resolve else path.absolute()
            relative = target.relative_to(root.resolve())
            return str(Path("/dev") / relative)
        except (OSError, ValueError):
            return str(path)

    def _open_device_path(self, path: str | Path) -> Path:
        raw = Path(path)
        root = self._lidar_device_root()
        if (
            raw.is_absolute()
            and str(raw).startswith("/dev/")
            and (root != Path("/dev"))
        ):
            return root / raw.relative_to("/dev")
        return raw

    def _open_camera_device_path(self, path: str | Path) -> Path:
        raw = Path(path)
        root = self._camera_device_root()
        if (
            raw.is_absolute()
            and str(raw).startswith("/dev/")
            and (root != Path("/dev"))
        ):
            return root / raw.relative_to("/dev")
        return raw

    def _lidar_candidates(self) -> list[dict[str, Any]]:
        root = self._lidar_device_root()
        paths: list[Path] = []
        configured = os.environ.get("LIDAR_DEVICE")
        if configured:
            paths.append(self._open_device_path(configured))
        by_id_root = Path(
            os.environ.get("LIDAR_BY_ID_ROOT", str(root / "serial/by-id"))
        )
        if by_id_root.exists():
            paths.extend(sorted(by_id_root.glob("*")))
        for pattern in ("ttyUSB*", "ttyACM*"):
            paths.extend(sorted(root.glob(pattern)))
        rows = []
        seen: set[str] = set()
        for path in paths:
            try:
                resolved = path.resolve()
            except OSError:
                resolved = path
            key = str(resolved)
            if key in seen:
                continue
            seen.add(key)
            rows.append(
                {
                    "display_path": self._host_device_label(resolved),
                    "open_path": str(resolved),
                    "by_id": self._host_device_label(path, resolve=False)
                    if "/serial/by-id/" in str(path)
                    else None,
                    "exists": resolved.exists(),
                }
            )
        return rows

    def _lidar_driver_hint(self, path: str) -> str | None:
        lower = path.lower()
        if "cp210" in lower or "ttyusb" in lower:
            return "USB-UART serial LiDAR candidate, CP210x 계열"
        if "ttyacm" in lower:
            return "USB CDC ACM LiDAR candidate"
        return None

    def _camera_candidates(self) -> list[dict[str, Any]]:
        root = self._camera_device_root()
        paths: list[Path] = []
        configured = os.environ.get("CAMERA_DEVICE")
        if configured and configured != "auto":
            paths.append(self._open_camera_device_path(configured))
        by_id_root = Path(os.environ.get("CAMERA_BY_ID_ROOT", str(root / "v4l/by-id")))
        if by_id_root.exists():
            paths.extend(sorted(by_id_root.glob("*Camera*")))
            paths.extend(sorted(by_id_root.glob("*camera*")))
            paths.extend(sorted(by_id_root.glob("*USB*")))
        paths.extend(sorted(root.glob("video*")))
        rows = []
        seen: set[str] = set()
        for path in paths:
            try:
                resolved = path.resolve()
            except OSError:
                resolved = path
            key = str(resolved)
            if key in seen:
                continue
            seen.add(key)
            rows.append(
                {
                    "display_path": self._host_device_label_for_root(resolved, root),
                    "open_path": str(resolved),
                    "by_id": self._host_device_label_for_root(path, root, resolve=False)
                    if "/v4l/by-id/" in str(path)
                    else None,
                    "exists": resolved.exists(),
                }
            )
        return rows

    def _camera_driver_status(self) -> dict[str, Any]:
        data = _load_json(self.camera_driver_status_file)
        updated_at = data.get("updated_at")
        if isinstance(updated_at, (int, float)):
            age = max(0.0, time.time() - float(updated_at))
            data["age_sec"] = round(age, 2)
            data["alive"] = age < 5.0
        else:
            data["age_sec"] = None
            data["alive"] = False
        return data

    def _camera_supported_view_modes(
        self, driver_status: dict[str, Any]
    ) -> list[dict[str, Any]]:
        raw_modes = driver_status.get("supported_view_modes")
        if raw_modes:
            raw_resolutions = set()
            for item in raw_modes:
                if not isinstance(item, dict):
                    continue
                resolution = str(item.get("resolution") or "")
                if "x" not in resolution and item.get("width") and item.get("height"):
                    resolution = f"{item.get('width')}x{item.get('height')}"
                if "x" in resolution:
                    raw_resolutions.add(resolution.lower())
            product_modes = self._normalized_camera_modes(DEFAULT_CAMERA_VIEW_MODES)
            filtered = [
                item
                for item in product_modes
                if item["resolution"].lower() in raw_resolutions
            ]
            if filtered:
                return filtered
        if not raw_modes:
            raw_modes = (
                driver_status.get("default_view_modes") or DEFAULT_CAMERA_VIEW_MODES
            )
        return self._normalized_camera_modes(raw_modes)

    def _normalized_camera_modes(self, raw_modes: list[Any]) -> list[dict[str, Any]]:
        rows = []
        seen: set[str] = set()
        for item in raw_modes:
            if not isinstance(item, dict):
                continue
            resolution = str(item.get("resolution") or "")
            if "x" not in resolution:
                width = item.get("width")
                height = item.get("height")
                if isinstance(width, int) and isinstance(height, int):
                    resolution = f"{width}x{height}"
            try:
                width_text, height_text = resolution.lower().split("x", 1)
                width = int(width_text)
                height = int(height_text)
            except (ValueError, TypeError):
                continue
            if resolution in seen:
                continue
            seen.add(resolution)
            fps_values = []
            for fps in item.get("fps", []):
                try:
                    value = float(fps)
                except (TypeError, ValueError):
                    continue
                if value >= 1:
                    fps_values.append(
                        int(round(value))
                        if abs(value - round(value)) < 0.05
                        else round(value, 2)
                    )
            fps_values = sorted({value for value in fps_values if value >= 1})
            if not fps_values:
                fps_values = [30]
            row = {
                "resolution": resolution,
                "width": width,
                "height": height,
                "fps": fps_values,
            }
            if item.get("label"):
                row["label"] = str(item.get("label"))
            rows.append(row)
        return rows

    def _host_device_label_for_root(
        self, path: Path, root: Path, resolve: bool = True
    ) -> str:
        try:
            target = path.resolve() if resolve else path.absolute()
            relative = target.relative_to(root.resolve())
            return str(Path("/dev") / relative)
        except (OSError, ValueError):
            return str(path)

    def _device_holders(self, device_path: str | None) -> list[dict[str, Any]]:
        # Scanning every /proc/<pid>/fd is the most expensive part of a
        # telemetry build; holders change rarely, so reuse the result briefly.
        if not device_path:
            return []
        now = time.monotonic()
        cached = self._holders_cache.get(device_path)
        if cached is not None and now - cached[0] < self._holders_ttl_s:
            return cached[1]
        holders = self._device_holders_uncached(device_path)
        self._holders_cache[device_path] = (now, holders)
        return holders

    def _device_holders_uncached(self, device_path: str) -> list[dict[str, Any]]:
        try:
            device_name = Path(device_path).resolve().name
        except OSError:
            device_name = Path(device_path).name
        holders = []
        proc_root = Path("/proc")
        for pid_dir in proc_root.iterdir():
            if not pid_dir.name.isdigit():
                continue
            fd_dir = pid_dir / "fd"
            try:
                fds = list(fd_dir.iterdir())
            except OSError:
                continue
            matched = False
            for fd in fds:
                try:
                    target = os.readlink(fd)
                except OSError:
                    continue
                if target == device_path or target.endswith(f"/{device_name}"):
                    matched = True
                    break
            if not matched:
                continue
            cmdline = (
                _read_text(pid_dir / "cmdline") or _read_text(pid_dir / "comm") or ""
            )
            holders.append(
                {"pid": int(pid_dir.name), "cmd": cmdline.replace("\x00", " ").strip()}
            )
        return holders

    def _rplidar_probe(self, path: str) -> dict[str, Any]:
        bauds = self._lidar_bauds()
        errors = []
        for baud in bauds:
            speed = getattr(termios, f"B{baud}", None)
            if speed is None:
                errors.append({"baud": baud, "error": "baud_not_supported_by_termios"})
                continue
            try:
                response = self._rplidar_probe_at_baud(path, baud, speed)
            except OSError as exc:
                errors.append({"baud": baud, "error": str(exc)})
                continue
            if response["ok"]:
                return response
            errors.append(
                {"baud": baud, "error": response.get("message", "no_response")}
            )
        return {
            "ok": False,
            "state": "no_protocol_response",
            "message": "Serial device exists, but RPLIDAR info/health response was not received.",
            "device": self._host_device_label(Path(path)),
            "tried_baud": bauds,
            "errors": errors,
        }

    def _rplidar_scan(self, path: str) -> dict[str, Any]:
        bauds = self._lidar_bauds()
        duration_s = self._lidar_scan_duration()
        max_points = self._lidar_scan_max_points()
        errors = []
        for baud in bauds:
            speed = getattr(termios, f"B{baud}", None)
            if speed is None:
                errors.append({"baud": baud, "error": "baud_not_supported_by_termios"})
                continue
            try:
                response = self._rplidar_normal_scan_at_baud(
                    path, baud, speed, duration_s, max_points
                )
            except OSError as exc:
                errors.append({"baud": baud, "error": str(exc)})
                continue
            if response["ok"]:
                return response
            error = {
                "baud": baud,
                "error": response.get("message", "no_scan_data"),
                "state": response.get("state"),
                "raw_bytes": response.get("raw_bytes"),
                "motor_line": response.get("motor_line"),
            }
            errors.append(
                {key: value for key, value in error.items() if value is not None}
            )
        return {
            "ok": False,
            "state": "no_scan_data",
            "message": "Serial device exists, but RPLIDAR normal scan points were not received.",
            "device": self._host_device_label(Path(path)),
            "tried_baud": bauds,
            "duration_s": duration_s,
            "errors": errors,
            "points": [],
        }

    def _rplidar_normal_scan_at_baud(
        self, path: str, baud: int, speed: int, duration_s: float, max_points: int
    ) -> dict[str, Any]:
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        raw = bytearray()
        motor_line = os.environ.get("LIDAR_MOTOR_LINE", "dtr").strip().lower() or "dtr"
        line_report: dict[str, Any] | None = None
        try:
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            attrs[4] = speed
            attrs[5] = speed
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)
            line_report = self._set_lidar_modem_lines(fd, motor_line)
            os.write(fd, RPLIDAR_STOP)
            time.sleep(0.08)
            termios.tcflush(fd, termios.TCIFLUSH)
            os.write(fd, RPLIDAR_SCAN)
            deadline = time.time() + duration_s
            while time.time() < deadline and len(raw) < 240000:
                try:
                    chunk = os.read(fd, 8192)
                except BlockingIOError:
                    time.sleep(0.01)
                    continue
                if chunk:
                    raw.extend(chunk)
                else:
                    time.sleep(0.01)
        finally:
            try:
                os.write(fd, RPLIDAR_STOP)
                time.sleep(0.03)
            except OSError:
                pass
            self._set_lidar_modem_lines(fd, "off")
            os.close(fd)
        payload = self._scan_payload(bytes(raw))
        nodes = self._parse_normal_scan_nodes(payload)
        if not nodes:
            return {
                "ok": False,
                "state": "no_scan_points",
                "message": f"RPLIDAR scan descriptor or valid points were not received at {baud} bps.",
                "device": self._host_device_label(Path(path)),
                "baud": baud,
                "protocol": "RPLIDAR normal scan",
                "duration_s": duration_s,
                "raw_bytes": len(raw),
                "motor_line": line_report,
                "points": [],
            }
        rendered = self._decimate_points(nodes, max_points)
        distances = [point["distance_m"] for point in nodes]
        angles = [point["angle_deg"] for point in nodes]
        metrics = {
            "points": len(nodes),
            "rendered_points": len(rendered),
            "raw_bytes": len(raw),
            "min_distance_m": round(min(distances), 3),
            "max_distance_m": round(max(distances), 3),
            "angle_min_deg": round(min(angles), 2),
            "angle_max_deg": round(max(angles), 2),
            "coverage_deg": self._angle_coverage(nodes),
            "range_limit_m": 30.0,
        }
        return {
            "ok": True,
            "state": "scan_ready",
            "message": "RPLIDAR produced normal scan points; port was closed after STOP.",
            "device": self._host_device_label(Path(path)),
            "baud": baud,
            "protocol": "RPLIDAR normal scan",
            "duration_s": duration_s,
            "points": rendered,
            "metrics": metrics,
            "tested_at": int(time.time()),
            "motor_line": line_report,
            "conflict_policy": "Short manual scan only; DTR/RTS motor line is disabled after STOP and the serial port is closed.",
        }

    def _scan_payload(self, raw: bytes) -> bytes:
        exact = raw.find(RPLIDAR_SCAN_DESCRIPTOR)
        if exact >= 0:
            return raw[exact + len(RPLIDAR_SCAN_DESCRIPTOR) :]
        marker = raw.find(b"\xa5Z")
        if marker >= 0 and len(raw) >= marker + 7:
            return raw[marker + 7 :]
        return raw

    def _parse_normal_scan_nodes(self, data: bytes) -> list[dict[str, Any]]:
        nodes: list[dict[str, Any]] = []
        i = 0
        while i <= len(data) - 5:
            b0, b1, b2, b3, b4 = data[i : i + 5]
            start_flag = b0 & 1
            inverse_start_flag = b0 >> 1 & 1
            check_bit = b1 & 1
            if start_flag ^ inverse_start_flag != 1 or check_bit != 1:
                i += 1
                continue
            angle_deg = (b1 >> 1 | b2 << 7) / 64.0
            distance_m = (b3 | b4 << 8) / 4.0 / 1000.0
            if 0.05 <= distance_m <= 30.0 and 0.0 <= angle_deg < 360.0:
                rad = math.radians(angle_deg)
                nodes.append(
                    {
                        "angle_deg": round(angle_deg, 3),
                        "distance_m": round(distance_m, 3),
                        "quality": b0 >> 2,
                        "x_m": round(math.sin(rad) * distance_m, 3),
                        "y_m": round(math.cos(rad) * distance_m, 3),
                    }
                )
                i += 5
                continue
            i += 1
        return nodes

    def _decimate_points(
        self, points: list[dict[str, Any]], max_points: int
    ) -> list[dict[str, Any]]:
        if len(points) <= max_points:
            return points
        step = len(points) / max_points
        return [
            points[min(len(points) - 1, int(index * step))]
            for index in range(max_points)
        ]

    def _angle_coverage(self, points: list[dict[str, Any]]) -> float:
        if not points:
            return 0.0
        bins = {int(point["angle_deg"]) % 360 for point in points}
        return round(float(len(bins)), 1)

    def _set_lidar_modem_lines(self, fd: int, mode: str) -> dict[str, Any]:
        try:
            flags = struct.unpack("I", fcntl.ioctl(fd, TIOCMGET, struct.pack("I", 0)))[
                0
            ]
            if mode == "dtr_rts":
                flags |= TIOCM_DTR | TIOCM_RTS
            elif mode == "rts":
                flags = (flags | TIOCM_RTS) & ~TIOCM_DTR
            elif mode == "off":
                flags &= ~(TIOCM_DTR | TIOCM_RTS)
            else:
                mode = "dtr"
                flags = (flags | TIOCM_DTR) & ~TIOCM_RTS
            fcntl.ioctl(fd, TIOCMSET, struct.pack("I", flags))
        except OSError as exc:
            return {"ok": False, "mode": mode, "error": str(exc)}
        return {"ok": True, "mode": mode}

    def _lidar_scan_duration(self) -> float:
        try:
            value = float(
                os.environ.get("LIDAR_SCAN_DURATION_S", DEFAULT_LIDAR_SCAN_DURATION_S)
            )
        except ValueError:
            value = DEFAULT_LIDAR_SCAN_DURATION_S
        return round(max(0.3, min(4.0, value)), 2)

    def _lidar_scan_max_points(self) -> int:
        try:
            value = int(
                os.environ.get("LIDAR_SCAN_MAX_POINTS", DEFAULT_LIDAR_SCAN_MAX_POINTS)
            )
        except ValueError:
            value = DEFAULT_LIDAR_SCAN_MAX_POINTS
        return max(120, min(5000, value))

    def _lidar_bauds(self) -> list[int]:
        raw = os.environ.get("LIDAR_BAUDS")
        if not raw:
            return DEFAULT_LIDAR_BAUDS
        values = []
        for item in raw.split(","):
            try:
                values.append(int(item.strip()))
            except ValueError:
                continue
        return values or DEFAULT_LIDAR_BAUDS

    def _rplidar_probe_at_baud(
        self, path: str, baud: int, speed: int
    ) -> dict[str, Any]:
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            attrs[4] = speed
            attrs[5] = speed
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)
            os.write(fd, bytes([165, 37]))
            time.sleep(0.05)
            info_raw = self._serial_command(fd, bytes([165, 80]), 27, 0.9)
            health_raw = self._serial_command(fd, bytes([165, 82]), 10, 0.7)
        finally:
            os.close(fd)
        if not info_raw.startswith(b"\xa5Z"):
            return {"ok": False, "message": f"no RPLIDAR response at {baud} bps"}
        info = self._parse_rplidar_info(info_raw)
        health = self._parse_rplidar_health(health_raw)
        return {
            "ok": True,
            "state": "responding",
            "message": "RPLIDAR responded to GET_INFO without starting scan.",
            "device": self._host_device_label(Path(path)),
            "baud": baud,
            "protocol": "RPLIDAR",
            "info": info,
            "health": health,
            "raw_info": info_raw.hex(" "),
            "raw_health": health_raw.hex(" "),
            "tested_at": int(time.time()),
            "conflict_policy": "GET_INFO/GET_HEALTH only; no scan command; serial port closed immediately after test.",
        }

    def _serial_command(
        self, fd: int, command: bytes, expected: int, timeout_s: float
    ) -> bytes:
        termios.tcflush(fd, termios.TCIOFLUSH)
        os.write(fd, command)
        deadline = time.time() + timeout_s
        data = bytearray()
        while time.time() < deadline and len(data) < expected:
            try:
                chunk = os.read(fd, expected - len(data))
            except BlockingIOError:
                time.sleep(0.02)
                continue
            if chunk:
                data.extend(chunk)
            else:
                time.sleep(0.02)
        return bytes(data)

    def _parse_rplidar_info(self, data: bytes) -> dict[str, Any]:
        payload = data[7:27]
        if len(payload) < 20:
            return {"raw_payload": payload.hex(" ")}
        return {
            "model_id": payload[0],
            "firmware": f"{payload[2]}.{payload[1]}",
            "hardware": payload[3],
            "serial": payload[4:20].hex(),
        }

    def _parse_rplidar_health(self, data: bytes) -> dict[str, Any] | None:
        if not data.startswith(b"\xa5Z") or len(data) < 10:
            return None
        payload = data[7:10]
        status = {0: "good", 1: "warning", 2: "error"}.get(
            payload[0], f"unknown_{payload[0]}"
        )
        return {"status": status, "error_code": payload[1] | payload[2] << 8}

    def plan(self):
        return {"research_topic": "2D LiDAR 기반 UBTS 로버의 원본 센서 대시보드", "team": "유니주행차", "partner": "UNIST",
                "focus": ["RPLIDAR S2 단일 평면 스캔", "UVC 카메라 원본 영상", "BNO085 자세와 가속도", "수동 조종 및 센서 상태 관찰", "GPS 미연결 상태 유지"],
                "milestones": [{"month": "1단계", "label": "원본 센서 대시보드", "status": "done"}, {"month": "2단계", "label": "Gazebo 2D 센서 및 가상 주행", "status": "planned"}, {"month": "추후", "label": "시연 녹화와 실차 보정", "status": "planned"}]}

    def site_map(self) -> dict[str, Any]:
        """The eight menu pages as before the redesign (docs/SCREENS_KO.md, UI-16); six-screen addresses are aliases."""
        return {
            "routes": [
                {"path": "/", "label": "Overview", "summary": "핵심 상태"},
                {"path": "/connection", "label": "Connection", "summary": "LiDAR, ROS2 bridge, sensor links"},
                {"path": "/system", "label": "System", "summary": "CPU, Memory, Swap, Network, Power, Disk"},
                {"path": "/network", "label": "Network", "summary": "IP, interface, RX/TX throughput"},
                {"path": "/docker", "label": "Docker", "summary": "Docker engine and containers"},
                {"path": "/fan", "label": "Fan Control", "summary": "Jetson 온도와 수동 PWM"},
                {"path": "/rover-control", "label": "Rover Control",
                 "summary": "센서, 지도, 수동 조종(차동·앞바퀴 조향), 자율주행, 현장 조정 값, 기록"},
                {"path": "/docs", "label": "Docs", "summary": "운영 문서와 Markdown viewer"},
            ],
            "aliases": {"/drive": "/rover-control", "/map": "/rover-control", "/records": "/rover-control",
                        "/tuning": "/rover-control", "/sensors": "/connection", "/system/resources": "/system",
                        "/system/network": "/network", "/system/docker": "/docker", "/system/fan": "/fan",
                        "/research": "/"},
        }

    def _meta(self) -> dict[str, Any]:
        return {
            "generated_at": int(time.time()),
            "mode": self.mode,
            "project": "unist_orachard_dashboard",
            "telemetry_file": str(self.telemetry_file),
        }

    def _not_connected(self, name: str) -> dict[str, Any]:
        return {"connected": False, "state": "not_connected", "source": name}

    def _default_sensors(self) -> dict[str, Any]:
        return {
            "lidar": {
                "ok": False,
                "state": "waiting_for_ros2",
                "severity": "waiting",
                "topic": "/scan",
                "frame_id": "laser",
                "link": "laser",
                "parent_link": "base_link",
                "owner": "ros2_driver",
                "dashboard_access": "subscribe_only",
            },
            "camera": {
                "ok": False,
                "state": "not_connected",
                "topic": "/camera/image_raw",
                "camera_info_topic": "/camera/camera_info",
                "frame_id": "camera_link_optical",
                "link": "camera_link",
                "optical_link": "camera_link_optical",
                "parent_link": "base_link",
                "owner": "ros2_camera_driver",
                "dashboard_access": "subscribe_only",
            },
            "imu": {
                "ok": False,
                "state": "not_connected",
                "topic": "/imu/data_raw",
                "raw_topic": "/imu/data_raw",
                "mag_topic": "/imu/mag",
                "frame_id": "imu_link",
                "link": "imu_link",
                "parent_link": "base_link",
                "owner": "ros2_driver_and_recorder",
                "dashboard_access": "subscribe_only",
            },
            "encoder": {
                "ok": False,
                "state": "not_connected",
                "topic": "/wheel/encoder",
                "frame_id": "base_link",
            },
            "mavlink": {"ok": False, "state": "not_connected"},
        }

    def _sensors(
        self,
        external: dict[str, Any],
        lidar_bridge: dict[str, Any] | None = None,
        camera_bridge: dict[str, Any] | None = None,
        imu_bridge: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        sensors = self._default_sensors()
        for key, value in external.get("sensors", {}).items():
            if isinstance(value, dict) and isinstance(sensors.get(key), dict):
                sensors[key] = {**sensors[key], **value}
            else:
                sensors[key] = value
        bridge = lidar_bridge or self._lidar_bridge_status()
        lidar = dict(sensors.get("lidar", {}))
        if bridge.get("scan_topic_alive"):
            lidar.update(
                {
                    "ok": True,
                    "connected": True,
                    "state": "scan_live",
                    "severity": "ok",
                    "source": bridge.get("source", "ros2"),
                    "point_count": bridge.get("point_count", 0),
                    "last_scan_age_sec": bridge.get("last_scan_age_sec"),
                    "frame_id": bridge.get("frame_id")
                    or lidar.get("frame_id")
                    or "laser",
                }
            )
        elif bridge.get("alive"):
            lidar.update(
                {
                    "ok": False,
                    "connected": False,
                    "state": "waiting_for_scan",
                    "severity": "waiting",
                    "source": bridge.get("source", "ros2"),
                    "point_count": bridge.get("point_count", 0),
                    "last_scan_age_sec": bridge.get("last_scan_age_sec"),
                }
            )
        sensors["lidar"] = lidar
        camera_status = self.camera_status()
        bridge_camera = camera_bridge or self._camera_bridge_status()
        camera = dict(sensors.get("camera", {}))
        camera.update(
            {
                "present": camera_status.get("present"),
                "usb_present": camera_status.get("usb_present"),
                "device": camera_status.get("device"),
                "by_id": camera_status.get("by_id"),
                "holders": camera_status.get("holders", []),
                "driver_state": camera_status.get("driver_state"),
                "driver_last_error": camera_status.get("driver_last_error"),
            }
        )
        if bridge_camera.get("frame_topic_alive"):
            camera.update(
                {
                    "ok": True,
                    "connected": True,
                    "state": "frame_live",
                    "severity": "ok",
                    "frame_id": bridge_camera.get("frame_id")
                    or camera.get("frame_id")
                    or "camera_link_optical",
                    "source_width": bridge_camera.get("source_width"),
                    "source_height": bridge_camera.get("source_height"),
                    "last_frame_age_sec": bridge_camera.get("last_frame_age_sec"),
                    "frame_count": bridge_camera.get("frame_count", 0),
                }
            )
        elif camera_status.get("state") == "usb_detected_video_node_missing":
            camera.update(
                {
                    "ok": False,
                    "connected": False,
                    "state": "video_node_missing",
                    "severity": "error",
                    "error": camera_status.get("driver_last_error")
                    or "USB camera detected, but /dev/video node is missing",
                }
            )
        elif bridge_camera.get("alive"):
            camera.update(
                {
                    "ok": False,
                    "connected": False,
                    "state": "waiting_for_frame",
                    "severity": "waiting",
                    "last_frame_age_sec": bridge_camera.get("last_frame_age_sec"),
                    "frame_count": bridge_camera.get("frame_count", 0),
                }
            )
        elif camera_status.get("present"):
            camera.update({"state": "waiting_for_ros2", "severity": "waiting"})
        sensors["camera"] = camera
        bridge_imu = imu_bridge or self._imu_bridge_status()
        driver_imu = self._imu_driver_status()
        recorder_imu = self.recording_status().get("imu_pipeline") or {}
        imu = dict(sensors.get("imu", {}))
        imu.update(
            {
                "driver_state": driver_imu.get("state", "not_connected"),
                "driver_revision": driver_imu.get("revision"),
                "sensor": driver_imu.get("sensor", "BNO085"),
                "transport": driver_imu.get("transport", "i2c"),
                "raw_topic": driver_imu.get("raw_topic", "/imu/data_raw"),
                "relay_topic": "/imu/data",
                "processing_mode": "sensor_native_raw",
                "mag_topic": driver_imu.get("mag_topic", "/imu/mag"),
                "frame_id": driver_imu.get("frame_id")
                or (bridge_imu.get("raw") or {}).get("frame_id")
                or "imu_link",
                "orientation_convention": driver_imu.get(
                    "orientation_convention", "sensor_native_unverified_mount"
                ),
                "rep145_ready": driver_imu.get("rep145_ready", False),
                "rep145_blocker": driver_imu.get("rep145_blocker"),
                "raw": bridge_imu.get("raw") or recorder_imu.get("latest_raw") or {},
                "magnetic": bridge_imu.get("magnetic")
                or recorder_imu.get("latest_magnetic")
                or {},
                "last_imu_age_s": bridge_imu.get(
                    "last_imu_age_s", recorder_imu.get("last_raw_age_s")
                ),
                "sample_count": bridge_imu.get(
                    "raw_count", recorder_imu.get("raw_received_count", 0)
                ),
            }
        )
        if bridge_imu.get("imu_topic_alive") or recorder_imu.get("raw_topic_alive"):
            imu.update(
                {"ok": True, "connected": True, "state": "imu_live", "severity": "ok"}
            )
        elif bridge_imu.get("updated_at") or driver_imu.get("updated_at"):
            imu.update(
                {
                    "ok": False,
                    "connected": False,
                    "state": "waiting_for_imu",
                    "severity": "waiting",
                }
            )
        sensors["imu"] = imu
        return sensors

    def _mission_status(self) -> dict[str, Any]:
        return {
            "name": "Orchard rover dashboard",
            "stage": "infrastructure",
            "operator": "unist",
            "connected": False,
        }

    def _services(
        self,
        external: dict[str, Any],
        lidar_bridge: dict[str, Any] | None = None,
        camera_bridge: dict[str, Any] | None = None,
        imu_bridge: dict[str, Any] | None = None,
        rover_bridge: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        services = {
            "dashboard": {"ok": True, "state": "serving", "port": 8080},
            "cloudflared": {
                "ok": True,
                "state": "tunnel_configured",
                "host": "bts.sarihub.com",
            },
            "ros2_bridge": {"ok": False, "state": "not_connected"},
            "lidar_bridge": {"ok": False, "state": "not_connected", "topic": "/scan"},
            "camera_bridge": {
                "ok": False,
                "state": "not_connected",
                "topic": "/camera/image_raw",
            },
            "imu_bridge": {
                "ok": False,
                "state": "not_connected",
                "topic": "/imu/data_raw",
            },
            "rover_bridge": {
                "ok": False,
                "state": "not_connected",
                "host": None,
                "mode": "serial_only",
            },
            "data_recorder": {"ok": False, "state": "not_connected"},
        }
        services.update(external.get("services", {}))
        bridge = lidar_bridge or self._lidar_bridge_status()
        if bridge.get("alive"):
            lidar_service = {
                "ok": bool(bridge.get("scan_topic_alive")),
                "state": "scan_live"
                if bridge.get("scan_topic_alive")
                else "bridge_online",
                "topic": "/scan",
                "source": bridge.get("source", "ros2"),
                "point_count": bridge.get("point_count", 0),
                "last_scan_age_sec": bridge.get("last_scan_age_sec"),
            }
            services["ros2_bridge"] = lidar_service
            services["lidar_bridge"] = lidar_service
        camera = camera_bridge or self._camera_bridge_status()
        if camera.get("alive"):
            services["camera_bridge"] = {
                "ok": bool(camera.get("frame_topic_alive")),
                "state": "frame_live"
                if camera.get("frame_topic_alive")
                else "bridge_online",
                "topic": "/camera/image_raw",
                "frame_count": camera.get("frame_count", 0),
                "last_frame_age_sec": camera.get("last_frame_age_sec"),
                "resolution": self._resolution_text(
                    camera.get("source_width"), camera.get("source_height")
                ),
                "source_fps": camera.get("source_fps"),
            }
        imu = imu_bridge or self._imu_bridge_status()
        services["imu_bridge"] = {
            "ok": bool(imu.get("imu_topic_alive")),
            "state": "imu_live"
            if imu.get("imu_topic_alive")
            else "bridge_online"
            if imu.get("updated_at")
            else "not_connected",
            "topic": "/imu/data_raw",
            "relay_topic": "/imu/data",
            "processing_mode": "sensor_native_raw",
            "raw_count": imu.get("raw_count", 0),
            "last_imu_age_s": imu.get("last_imu_age_s"),
        }
        rover = self.rover_status()
        rover_bridge = rover_bridge or self._rover_bridge_status()
        rover_bridge_ok = (
            bool(rover_bridge.get("alive"))
            and (
                rover_bridge.get("transport") != "serial"
                or bool(rover_bridge.get("serial_open"))
            )
            and (not rover_bridge.get("last_error"))
        )
        services["rover_bridge"] = {
            "ok": rover_bridge_ok,
            "state": rover_bridge.get("state", "not_connected"),
            "host": rover.get("base_url"),
            "proxy": rover.get("proxy_url"),
            "mode": rover.get("mode", "serial_only"),
            "transport": rover_bridge.get("transport"),
            "serial_port": rover_bridge.get("serial_port"),
            "status_topic": rover_bridge.get("status_topic"),
            "serial_open": rover_bridge.get("serial_open"),
            "last_error": rover_bridge.get("last_error"),
        }
        recorder = self.recording_status()
        services["data_recorder"] = {
            "ok": bool(recorder.get("ok")),
            "state": "recording"
            if recorder.get("active")
            else recorder.get("state", "not_connected"),
            "active": bool(recorder.get("active")),
            "kind": recorder.get("kind"),
            "recording_id": recorder.get("recording_id"),
            "archive_count": recorder.get("archive_count"),
            "started_at": recorder.get("started_at"),
            "duration_s": recorder.get("duration_s"),
            "samples": recorder.get("samples"),
            "include_imu": recorder.get("include_imu", True),
            "include_gps": recorder.get("include_gps", True),
            "imu_pipeline": recorder.get("imu_pipeline"),
            "status_url": recorder.get("status_url")
            or f"{self.recorder_api_url}/status",
        }
        return services

    def _lidar_bridge_status(self) -> dict[str, Any]:
        data = _load_json(self.lidar_bridge_status_file)
        now = time.time()
        try:
            updated_at = float(data.get("updated_at", 0.0))
        except (TypeError, ValueError):
            updated_at = 0.0
        status_age = round(now - updated_at, 3) if updated_at else None
        alive = status_age is not None and status_age <= 3.0
        last_scan_age = data.get("last_scan_age_sec")
        try:
            last_scan_age = None if last_scan_age is None else float(last_scan_age)
        except (TypeError, ValueError):
            last_scan_age = None
        scan_alive = bool(alive and last_scan_age is not None and (last_scan_age < 2.0))
        return {
            "alive": alive,
            "scan_topic_alive": scan_alive,
            "status_age_sec": status_age,
            "last_scan_age_sec": round(last_scan_age, 3)
            if last_scan_age is not None
            else None,
            "point_count": int(data.get("point_count") or 0),
            "source": data.get("source") or "ros2",
            "frame_id": data.get("frame_id") or "laser",
            "error": None if scan_alive else data.get("error"),
            "updated_at": updated_at or None,
            "status_file": str(self.lidar_bridge_status_file),
        }

    def _camera_bridge_status(self) -> dict[str, Any]:
        data = _load_json(self.camera_bridge_status_file)
        now = time.time()
        try:
            updated_at = float(data.get("updated_at", 0.0))
        except (TypeError, ValueError):
            updated_at = 0.0
        status_age = round(now - updated_at, 3) if updated_at else None
        alive = status_age is not None and status_age <= 3.0
        last_frame_age = data.get("last_frame_age_sec")
        try:
            last_frame_age = None if last_frame_age is None else float(last_frame_age)
        except (TypeError, ValueError):
            last_frame_age = None
        frame_alive = bool(
            alive and last_frame_age is not None and (last_frame_age < 2.0)
        )
        return {
            "alive": alive,
            "frame_topic_alive": frame_alive,
            "status_age_sec": status_age,
            "last_frame_age_sec": round(last_frame_age, 3)
            if last_frame_age is not None
            else None,
            "frame_count": int(data.get("frame_count") or 0),
            "source_width": data.get("source_width"),
            "source_height": data.get("source_height"),
            "source_fps": data.get("source_fps"),
            "encoding": data.get("encoding"),
            "frame_id": data.get("frame_id") or "camera_link_optical",
            "error": None if frame_alive else data.get("error"),
            "updated_at": updated_at or None,
            "status_file": str(self.camera_bridge_status_file),
        }

    def _imu_driver_status(self) -> dict[str, Any]:
        data = _load_json(self.imu_driver_status_file)
        now = time.time()
        try:
            updated_at = float(data.get("updated_at", 0.0))
        except (TypeError, ValueError):
            updated_at = 0.0
        age = round(now - updated_at, 3) if updated_at else None
        alive = bool(age is not None and age <= 3.0 and data.get("alive"))
        return {
            **data,
            "alive": alive,
            "state": data.get("state")
            if age is not None and age <= 3.0
            else "not_connected",
            "status_age_sec": age,
            "updated_at": updated_at or None,
            "status_file": str(self.imu_driver_status_file),
        }

    def _imu_bridge_status(self) -> dict[str, Any]:
        data = _load_json(self.imu_bridge_status_file)
        now = time.time()
        try:
            updated_at = float(data.get("updated_at", 0.0))
        except (TypeError, ValueError):
            updated_at = 0.0
        status_age = round(now - updated_at, 3) if updated_at else None
        alive = status_age is not None and status_age <= 3.0
        try:
            sample_age = (
                float(data.get("last_imu_age_s"))
                if data.get("last_imu_age_s") is not None
                else None
            )
        except (TypeError, ValueError):
            sample_age = None
        return {
            **data,
            "alive": alive,
            "imu_topic_alive": bool(
                alive and sample_age is not None and (sample_age < 1.0)
            ),
            "corrected_topic_alive": bool(alive and data.get("corrected_topic_alive")),
            "last_imu_age_s": round(sample_age, 4) if sample_age is not None else None,
            "updated_at": updated_at or None,
            "status_age_sec": status_age,
            "status_file": str(self.imu_bridge_status_file),
        }

    def _rover_bridge_status(self) -> dict[str, Any]:
        data = _load_json(self.rover_bridge_status_file)
        try:
            age = time.time() - float(data.get("updated_at") or 0)
        except (TypeError, ValueError):
            age = float("inf")
        alive = 0 <= age < 3
        return {
            **data,
            "alive": alive,
            "ok": bool(alive and data.get("connected")),
            "connected": bool(alive and data.get("connected")),
            "serial_open": bool(alive and data.get("serial_open")),
            "state": data.get("state", "disarmed") if alive else "unavailable",
            "mode": "esp_manual",
            "automatic_input_enabled": False,
        }

    def _resolution_text(self, width: Any, height: Any) -> str | None:
        try:
            if int(width) > 0 and int(height) > 0:
                return f"{int(width)}x{int(height)}"
        except (TypeError, ValueError):
            return None
        return None

    def _alerts(self, external: dict[str, Any]) -> list[dict[str, str]]:
        alerts = list(external.get("alerts", []))
        if not external.get("rover"):
            alerts.append(
                {
                    "level": "info",
                    "message": "ROS2/로버 텔레메트리 파일이 아직 연결되지 않았습니다.",
                }
            )
        if not Path(self.docker_socket).exists():
            alerts.append(
                {
                    "level": "warn",
                    "message": "Docker socket이 컨테이너에 마운트되지 않았습니다.",
                }
            )
        return alerts

    def _hostname(self) -> str:
        return (
            _read_text(
                os.environ.get(
                    "DASHBOARD_HOSTNAME_FILE", str(self.host_root / "etc/hostname")
                )
            )
            or socket.gethostname()
        )

    def _cpu_percent(self) -> dict[str, Any]:
        current = self._read_cpu_times()
        if not current or not self._last_cpu:
            return {"total_percent": None, "cores": []}
        result = {"total_percent": None, "cores": []}
        for name, values in current.items():
            previous = self._last_cpu.get(name)
            if not previous:
                continue
            idle_delta = values["idle"] - previous["idle"]
            total_delta = values["total"] - previous["total"]
            percent = (
                None
                if total_delta <= 0
                else round(
                    max(0.0, min(100.0, 100.0 * (1.0 - idle_delta / total_delta))), 1
                )
            )
            if name == "cpu":
                result["total_percent"] = percent
            else:
                index = (
                    int(name[3:])
                    if name.startswith("cpu") and name[3:].isdigit()
                    else len(result["cores"])
                )
                result["cores"].append(
                    {
                        "name": name,
                        "label": _cpu_display_label(name),
                        "index": index,
                        "percent": percent,
                    }
                )
        self._last_cpu = current
        result["cores"].sort(key=lambda row: row.get("index", 0))
        return result

    def _read_cpu_times(self) -> dict[str, dict[str, int]] | None:
        text = _read_text(self.proc_root / "stat")
        if not text:
            return None
        rows = {}
        for line in text.splitlines():
            parts = line.split()
            if not parts or not parts[0].startswith("cpu"):
                continue
            try:
                values = [int(value) for value in parts[1:]]
            except ValueError:
                continue
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            rows[parts[0]] = {"idle": idle, "total": sum(values)}
        return rows

    def _memory(self) -> dict[str, Any]:
        values = self._meminfo()
        total = values.get("MemTotal", 0) * 1024
        available = values.get("MemAvailable", 0) * 1024
        used = max(0, total - available)
        display_total = int(
            os.environ.get(
                "MEMORY_DISPLAY_TOTAL_BYTES", DEFAULT_MEMORY_DISPLAY_TOTAL_BYTES
            )
        )
        return {
            "total": total,
            "available": available,
            "used": used,
            "percent": _pct(used, total),
            "display_total": display_total,
            "display_percent": _pct(used, display_total),
            "display_label": "8.0 GB 기준",
        }

    def _swap(self) -> dict[str, Any]:
        values = self._meminfo()
        total = values.get("SwapTotal", 0) * 1024
        free = values.get("SwapFree", 0) * 1024
        used = max(0, total - free)
        return {
            "total": total,
            "free": free,
            "used": used,
            "percent": _pct(used, total),
        }

    def _meminfo(self) -> dict[str, int]:
        text = _read_text(self.proc_root / "meminfo")
        values = {}
        if not text:
            return values
        for line in text.splitlines():
            key, raw = line.split(":", 1)
            try:
                values[key] = int(raw.strip().split()[0])
            except (IndexError, ValueError):
                continue
        return values

    def _disks(self) -> list[dict[str, Any]]:
        mounted = self._host_mounted_partitions()
        if mounted:
            return mounted
        return self._fallback_disks()

    def _fallback_disks(self) -> list[dict[str, Any]]:
        targets = [("/", self.host_root, None, None)]
        workspace = self.host_root / "home/unist/workspace"
        if workspace.exists():
            targets.append(("/home/unist/workspace", workspace, None, None))
        disks = []
        for label, path, source, fstype in targets:
            try:
                usage = shutil.disk_usage(path)
            except OSError:
                continue
            disks.append(
                {
                    "mount": label,
                    "source": source,
                    "fstype": fstype,
                    "total": usage.total,
                    "used": usage.used,
                    "free": usage.free,
                    "percent": _pct(usage.used, usage.total),
                }
            )
        return disks

    def _host_mounted_partitions(self) -> list[dict[str, Any]]:
        findmnt = shutil.which("findmnt")
        if not findmnt:
            return []
        command = [
            findmnt,
            "-b",
            "-J",
            "-o",
            "TARGET,SOURCE,FSTYPE,SIZE,USED,AVAIL,USE%",
        ]
        nsenter = shutil.which("nsenter")
        if nsenter:
            command = [nsenter, "-t", "1", "-m", "--", *command]
        try:
            proc = subprocess.run(
                command, check=False, capture_output=True, text=True, timeout=4
            )
        except (OSError, subprocess.TimeoutExpired):
            return []
        if proc.returncode != 0:
            return []
        try:
            payload = json.loads(proc.stdout)
        except json.JSONDecodeError:
            return []
        rows: list[dict[str, Any]] = []
        seen: set[tuple[str, str]] = set()
        for item in self._flatten_findmnt(payload.get("filesystems", [])):
            target = str(item.get("target") or "")
            source = str(item.get("source") or "")
            fstype = str(item.get("fstype") or "")
            total = _safe_int(str(item.get("size"))) or 0
            used = _safe_int(str(item.get("used"))) or 0
            free = _safe_int(str(item.get("avail"))) or 0
            if not self._is_storage_mount(target, source, fstype, total):
                continue
            key = (target, source)
            if key in seen:
                continue
            seen.add(key)
            if total <= 0:
                host_path = self.host_root / target.lstrip("/")
                if target == "/":
                    host_path = self.host_root
                try:
                    usage = shutil.disk_usage(host_path)
                except OSError:
                    continue
                total, used, free = (usage.total, usage.used, usage.free)
            rows.append(
                {
                    "mount": target,
                    "source": source or None,
                    "fstype": fstype or None,
                    "total": total,
                    "used": used,
                    "free": free,
                    "percent": _pct(used, total),
                }
            )
        rows.sort(key=lambda row: (0 if row["mount"] == "/" else 1, row["mount"]))
        return rows

    def _flatten_findmnt(self, items: list[dict[str, Any]]) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for item in items:
            rows.append(item)
            children = item.get("children")
            if isinstance(children, list):
                rows.extend(self._flatten_findmnt(children))
        return rows

    def _is_storage_mount(
        self, target: str, source: str, fstype: str, total: int
    ) -> bool:
        if not target or total <= 0:
            return False
        ignored_fs = {
            "autofs",
            "binfmt_misc",
            "bpf",
            "cgroup",
            "cgroup2",
            "configfs",
            "debugfs",
            "devpts",
            "devtmpfs",
            "fusectl",
            "mqueue",
            "nsfs",
            "overlay",
            "proc",
            "pstore",
            "securityfs",
            "squashfs",
            "sysfs",
            "tmpfs",
            "tracefs",
        }
        if fstype in ignored_fs:
            return False
        ignored_targets = (
            "/dev",
            "/etc/hostname",
            "/etc/hosts",
            "/etc/resolv.conf",
            "/mnt/host",
            "/proc",
            "/run",
            "/sys",
            "/var/lib/docker/overlay2",
            "/var/lib/docker/containers",
            "/snap",
        )
        if target != "/" and target.startswith(ignored_targets):
            return False
        if "/docker/containers/" in source:
            return False
        if source.startswith("/dev/loop"):
            return False
        if source.startswith("/dev/"):
            return True
        return target in {"/", "/boot", "/boot/efi"} or target.startswith(
            ("/media/", "/mnt/", "/home/")
        )

    def _os_uptime_s(self) -> float | None:
        text = _read_text(self.proc_root / "uptime")
        if not text:
            return None
        try:
            return round(float(text.split()[0]), 1)
        except (IndexError, ValueError):
            return None

    def _power(self) -> dict[str, Any]:
        root = self._power_hwmon()
        if not root:
            return {"available": False, "total_w": None, "rails": []}
        rails = []
        for curr_path in sorted(root.glob("curr*_input")):
            suffix = curr_path.name.removeprefix("curr").removesuffix("_input")
            current_ma = _safe_int(_read_text(curr_path))
            voltage_mv = _safe_int(_read_text(root / f"in{suffix}_input"))
            label = _read_text(root / f"in{suffix}_label") or f"rail_{suffix}"
            if current_ma is None:
                continue
            power_w = None
            if voltage_mv is not None:
                power_w = round(voltage_mv * current_ma / 1000000.0, 3)
            rails.append(
                {
                    "name": label,
                    "current_ma": current_ma,
                    "voltage_mv": voltage_mv,
                    "power_w": power_w,
                }
            )
        total_rail = next(
            (
                rail
                for rail in rails
                if rail["name"] == "VDD_IN" and rail["power_w"] is not None
            ),
            None,
        )
        if total_rail:
            total_w = total_rail["power_w"]
            source = "VDD_IN"
        else:
            total_w = (
                round(sum((rail["power_w"] or 0.0 for rail in rails)), 3)
                if rails
                else None
            )
            source = "sum"
        return {
            "available": bool(rails),
            "total_w": total_w,
            "source": source,
            "rails": rails,
        }

    def _power_hwmon(self) -> Path | None:
        if self._power_hwmon_cache and self._power_hwmon_cache.exists():
            return self._power_hwmon_cache
        hwmon_root = self.sys_root / "class/hwmon"
        if not hwmon_root.exists():
            return None
        for root in sorted(hwmon_root.glob("hwmon*")):
            name = _read_text(root / "name")
            if name == "ina3221" and list(root.glob("curr*_input")):
                self._power_hwmon_cache = root
                return root
        return None

    def _temperatures(self) -> list[dict[str, Any]]:
        thermal_root = self.sys_root / "class/thermal"
        temps = []
        if not thermal_root.exists():
            return temps
        for zone in thermal_root.glob("thermal_zone*"):
            raw = _read_text(zone / "temp")
            value = _safe_int(raw)
            if value is None:
                continue
            label = _read_text(zone / "type") or zone.name
            temp = value / 1000.0 if abs(value) > 200 else float(value)
            temps.append({"name": label, "temperature_c": round(temp, 1)})
        return temps

    def _max_temperature(self) -> float | None:
        temps = [row["temperature_c"] for row in self._temperatures()]
        return round(max(temps), 1) if temps else None

    def _read_net_counters(self) -> dict[str, dict[str, int]]:
        text = _read_text(self.proc_root / "net/dev")
        counters = {}
        if not text:
            return counters
        for line in text.splitlines()[2:]:
            if ":" not in line:
                continue
            name, raw = line.split(":", 1)
            fields = raw.split()
            if len(fields) < 16:
                continue
            counters[name.strip()] = {
                "rx_bytes": int(fields[0]),
                "rx_packets": int(fields[1]),
                "tx_bytes": int(fields[8]),
                "tx_packets": int(fields[9]),
            }
        return counters

    def _cached_external_ip(self) -> str | None:
        # Never block a telemetry request on DNS/HTTP (S23): the lookup runs in
        # one background thread and the next refresh time is set before it starts.
        now = time.time()
        if now < self._external_ip_next_refresh:
            return self._external_ip_value
        if not os.environ.get("EXTERNAL_IP_URL", "https://api64.ipify.org"):
            return self._external_ip_value
        if self._external_ip_thread is not None and self._external_ip_thread.is_alive():
            return self._external_ip_value
        self._external_ip_next_refresh = now + int(os.environ.get("EXTERNAL_IP_RETRY_S", "60"))
        self._external_ip_thread = threading.Thread(target=self._lookup_external_ip, daemon=True,
                                                    name="telemetry-external-ip")
        self._external_ip_thread.start()
        return self._external_ip_value

    def _lookup_external_ip(self) -> None:
        now = time.time()
        url = os.environ.get("EXTERNAL_IP_URL", "https://api64.ipify.org")
        timeout = float(os.environ.get("EXTERNAL_IP_TIMEOUT_S", "0.8"))
        try:
            with urllib.request.urlopen(url, timeout=timeout) as response:
                value = response.read(96).decode("utf-8").strip().splitlines()[0]
            ipaddress.ip_address(value)
            self._external_ip_value = value
            self._external_ip_error = None
            self._external_ip_next_refresh = now + int(
                os.environ.get("EXTERNAL_IP_CACHE_S", "300")
            )
        except (OSError, UnicodeDecodeError, ValueError, IndexError) as exc:
            self._external_ip_error = str(exc)
            self._external_ip_next_refresh = now + int(
                os.environ.get("EXTERNAL_IP_RETRY_S", "60")
            )

    def _primary_ipv4(self, interfaces: list[dict[str, Any]]) -> str | None:
        target = os.environ.get("PRIMARY_IP_ROUTE_TARGET", "1.1.1.1")
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.settimeout(0.2)
                sock.connect((target, 80))
                value = sock.getsockname()[0]
                if value and (not value.startswith("127.")):
                    return value
        except OSError:
            pass
        for iface in interfaces:
            if iface["name"] == "lo":
                continue
            for value in iface.get("ipv4") or []:
                if value and (not value.startswith("127.")):
                    return value
        return None

    def _primary_interface(
        self, primary_ipv4: str | None, interfaces: list[dict[str, Any]]
    ) -> str | None:
        if not primary_ipv4:
            return None
        for iface in interfaces:
            if primary_ipv4 in (iface.get("ipv4") or []):
                return iface["name"]
        return None

    def _iface_text(self, name: str, field: str) -> str | None:
        return _read_text(self.sys_root / "class/net" / name / field)

    def _ipv4_for_interface(self, name: str) -> list[str]:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            request = struct.pack("256s", name[:15].encode("utf-8"))
            response = fcntl_ioctl(sock, 35093, request)
            return [socket.inet_ntoa(response[20:24])]
        except OSError:
            return []
        finally:
            if "sock" in locals():
                sock.close()

    def _docker_get_json(self, path: str) -> Any:
        body = self._docker_get_raw(path)
        return json.loads(body.decode("utf-8"))

    def _docker_get_raw(self, path: str) -> bytes:
        conn = DockerSocketHTTPConnection(self.docker_socket)
        try:
            conn.request("GET", path)
            response = conn.getresponse()
            body = response.read()
            if response.status >= 400:
                raise OSError(f"Docker API {path} returned {response.status}")
            return body
        finally:
            conn.close()

    def _docker_post_raw(self, path: str) -> tuple[int, bytes]:
        conn = DockerSocketHTTPConnection(self.docker_socket)
        try:
            conn.request("POST", path, body=b"")
            response = conn.getresponse()
            body = response.read()
            if response.status >= 400:
                raise OSError(
                    f"Docker API {path} returned {response.status}: {body.decode('utf-8', errors='replace')[:300]}"
                )
            return (response.status, body)
        finally:
            conn.close()

    def _dashboard_container_names(self) -> set[str]:
        names = {
            "unist-orachard-dashboard",
            "dashboard",
            os.environ.get("DASHBOARD_CONTAINER_NAME", ""),
            os.environ.get("COMPOSE_SERVICE", ""),
        }
        return {
            str(name).strip().lstrip("/") for name in names if str(name or "").strip()
        }

    def _detect_self_container_id(self) -> str | None:
        explicit = os.environ.get("DASHBOARD_CONTAINER_ID")
        if explicit and re.fullmatch("[0-9a-f]{12,64}", explicit):
            return explicit
        by_name = self._container_id_by_known_name()
        if by_name:
            return by_name
        candidates: list[str] = []
        hostname = os.environ.get("HOSTNAME") or socket.gethostname()
        if re.fullmatch("[0-9a-f]{12,64}", hostname or ""):
            candidates.append(hostname)
        text = _read_text("/proc/self/cgroup") or ""
        candidates.extend(re.findall("[0-9a-f]{64}", text))
        candidates.extend(re.findall("docker-([0-9a-f]{64})\\\\.scope", text))
        for candidate in candidates:
            matched = self._inspect_container_name_matches(candidate)
            if matched:
                return matched
        if len(candidates) == 1 and str(candidates[0]).startswith(str(hostname or "")):
            return candidates[0]
        return None

    def _container_id_by_known_name(self) -> str | None:
        if not Path(self.docker_socket).exists():
            return None
        try:
            containers = self._docker_get_json("/containers/json?all=1")
        except (OSError, json.JSONDecodeError):
            return None
        for container in containers:
            names = container.get("Names") or []
            labels = container.get("Labels") or {}
            if (
                self._container_names_match(names)
                or labels.get("com.docker.compose.service")
                in self._self_container_names
            ):
                return container.get("Id")
        return None

    def _inspect_container_name_matches(self, container_id: str) -> str | None:
        if not Path(self.docker_socket).exists():
            return None
        try:
            detail = self._docker_get_json(f"/containers/{container_id}/json")
        except (OSError, json.JSONDecodeError):
            return None
        config = detail.get("Config") or {}
        labels = config.get("Labels") or {}
        if (
            self._container_names_match([detail.get("Name") or ""])
            or labels.get("com.docker.compose.service") in self._self_container_names
        ):
            return detail.get("Id") or container_id
        return None

    def _container_names_match(self, names: list[str] | tuple[str, ...]) -> bool:
        normalized = {
            str(name).strip().lstrip("/") for name in names if str(name or "").strip()
        }
        return bool(normalized & self._self_container_names)

    def _detect_container_ids_from_mountinfo(self) -> list[str]:
        ids: list[str] = []
        for path in ("/proc/self/mountinfo",):
            text = _read_text(path) or ""
            for line in text.splitlines():
                parts = line.split()
                if len(parts) > 4 and parts[4] == "/":
                    ids.extend(re.findall("[0-9a-f]{64}", line))
                    ids.extend(re.findall("docker-([0-9a-f]{64})\\\\.scope", line))
                    break
        return ids

    def _is_self_container(
        self, container_id: str | None, names: list[str] | tuple[str, ...] | None = None
    ) -> bool:
        if names and self._container_names_match(names):
            return True
        if not container_id or not self._self_container_id:
            return False
        full = str(container_id)
        own = self._self_container_id
        return full.startswith(own) or own.startswith(full) or full.startswith(own[:12])

    def _decode_docker_log_stream(self, raw: bytes) -> str:
        chunks: list[bytes] = []
        index = 0
        while index + 8 <= len(raw):
            stream_type = raw[index]
            size = int.from_bytes(raw[index + 4 : index + 8], "big")
            if stream_type not in {0, 1, 2} or size < 0 or index + 8 + size > len(raw):
                break
            chunks.append(raw[index + 8 : index + 8 + size])
            index += 8 + size
        if chunks and index == len(raw):
            raw = b"".join(chunks)
        return raw.decode("utf-8", errors="replace")

    def _docker_container_stats(self, container_id: str) -> dict[str, Any]:
        try:
            stats = self._docker_get_json(
                f"/containers/{container_id}/stats?stream=false&one-shot=true"
            )
        except (OSError, json.JSONDecodeError):
            return {}
        cpu_stats = stats.get("cpu_stats", {})
        precpu_stats = stats.get("precpu_stats", {})
        cpu_delta = cpu_stats.get("cpu_usage", {}).get(
            "total_usage", 0
        ) - precpu_stats.get("cpu_usage", {}).get("total_usage", 0)
        system_delta = cpu_stats.get("system_cpu_usage", 0) - precpu_stats.get(
            "system_cpu_usage", 0
        )
        online_cpus = (
            cpu_stats.get("online_cpus")
            or len(cpu_stats.get("cpu_usage", {}).get("percpu_usage") or [])
            or 1
        )
        cpu_percent = 0.0
        if cpu_delta > 0 and system_delta > 0:
            cpu_percent = cpu_delta / system_delta * online_cpus * 100.0
        memory = stats.get("memory_stats", {})
        memory_usage = max(
            0, memory.get("usage", 0) - memory.get("stats", {}).get("cache", 0)
        )
        memory_limit = memory.get("limit", 0)
        networks = stats.get("networks", {})
        rx_bytes = sum((item.get("rx_bytes", 0) for item in networks.values()))
        tx_bytes = sum((item.get("tx_bytes", 0) for item in networks.values()))
        return {
            "cpu_percent": round(cpu_percent, 1),
            "memory_usage": memory_usage,
            "memory_limit": memory_limit,
            "memory_percent": _pct(memory_usage, memory_limit),
            "rx_bytes": rx_bytes,
            "tx_bytes": tx_bytes,
        }


def fcntl_ioctl(sock: socket.socket, request: int, args: bytes) -> bytes:
    import fcntl

    return fcntl.ioctl(sock.fileno(), request, args)
