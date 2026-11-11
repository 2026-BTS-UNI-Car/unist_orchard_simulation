from __future__ import annotations

import argparse
import base64
import gzip
import hashlib
import json
import mimetypes
import os
import secrets
import select
import socket
import tarfile
import threading
import time
from http.cookies import CookieError, SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qs, unquote, urljoin, urlparse
from urllib.request import Request, urlopen

from dashboard_server.telemetry import ROOT, TelemetryProvider


STATIC_ROOT = ROOT / "static"
DOCS_ROOT = ROOT / "docs"
APP_ROUTES = {"/", "/connection", "/system", "/network", "/docker", "/fan", "/research", "/rover-control", "/docs"}
SESSION_COOKIE_NAME = "ubts_session"
SESSION_TTL_S = int(os.environ.get("DASHBOARD_SESSION_TTL_S", "3600"))
_SESSION_LOCK = threading.Lock()
_SESSIONS: dict[str, dict[str, float]] = {}
_ACTIVE_LIDAR_WS: dict[str, dict[str, float | str]] = {}
_ACTIVE_CAMERA_WS: dict[str, dict[str, float | str]] = {}
_ACTIVE_IMU_WS: dict[str, dict[str, float | str]] = {}
_ACTIVE_GPS_WS: dict[str, dict[str, float | str]] = {}


def dashboard_log(event: str, **fields):
    payload = {"ts": round(time.time(), 3), "event": event, **fields}
    print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")), flush=True)


def _prune_sessions(now: float | None = None):
    now = now or time.time()
    expired = [token for token, item in _SESSIONS.items() if item.get("expires_at", 0.0) <= now]
    for token in expired:
        _SESSIONS.pop(token, None)
    for connection_id, item in list(_ACTIVE_LIDAR_WS.items()):
        if item.get("session") in expired:
            _ACTIVE_LIDAR_WS.pop(connection_id, None)
    for connection_id, item in list(_ACTIVE_CAMERA_WS.items()):
        if item.get("session") in expired:
            _ACTIVE_CAMERA_WS.pop(connection_id, None)
    for connection_id, item in list(_ACTIVE_IMU_WS.items()):
        if item.get("session") in expired:
            _ACTIVE_IMU_WS.pop(connection_id, None)
    for connection_id, item in list(_ACTIVE_GPS_WS.items()):
        if item.get("session") in expired:
            _ACTIVE_GPS_WS.pop(connection_id, None)


def lidar_websocket_stats() -> dict[str, object]:
    return websocket_stats(_ACTIVE_LIDAR_WS)


def camera_websocket_stats() -> dict[str, object]:
    return websocket_stats(_ACTIVE_CAMERA_WS)


def imu_websocket_stats() -> dict[str, object]:
    return websocket_stats(_ACTIVE_IMU_WS)


def websocket_stats(registry: dict[str, dict[str, float | str]]) -> dict[str, object]:
    now = time.time()
    with _SESSION_LOCK:
        _prune_sessions(now)
        sessions = [
            {
                "id": token[:8],
                "age_sec": round(now - float(item.get("connected_at", now)), 1),
                "remote": item.get("remote", ""),
                "session": str(item.get("session", ""))[:8],
            }
            for token, item in registry.items()
        ]
        return {
            "active": len(sessions),
            "sessions": sessions,
            "session_ttl_s": SESSION_TTL_S,
            "known_sessions": len(_SESSIONS),
        }


class _TeeWriter:
    def __init__(self, response_target, cache_target):
        self.response_target = response_target
        self.cache_target = cache_target
        self.response_broken = False

    def write(self, data: bytes) -> int:
        if not self.response_broken:
            try:
                self.response_target.write(data)
            except (BrokenPipeError, ConnectionResetError):
                self.response_broken = True
        self.cache_target.write(data)
        return len(data)

    def flush(self):
        if not self.response_broken:
            try:
                self.response_target.flush()
            except (BrokenPipeError, ConnectionResetError):
                self.response_broken = True
        self.cache_target.flush()


DOCS_CATALOG = [{'slug': 'gps-offline-map', 'title': 'GPS · 대한민국 오프라인 지도', 'category': 'Sensors', 'summary': 'GPS UART 드라이버, 지도 추적, 동기 기록 및 배선 진단', 'file': 'gps_offline_map.md'}, {'slug': 'architecture', 'title': '대시보드 아키텍처', 'category': 'System', 'summary': 'Cloudflare Tunnel, Python 서버, telemetry 파일, 프론트엔드 흐름', 'file': 'architecture.md'}, {'slug': 'jetson-deployment', 'title': 'Jetson 배포', 'category': 'Deployment', 'summary': 'Jetson Orin Nano에서 Docker Compose와 자동 재시작으로 운영하는 절차', 'file': 'jetson_deployment.md'}, {'slug': 'simulation-run', 'title': 'ROS2 시뮬레이션 실행', 'category': 'ROS2', 'summary': '격리된 2D 과수원 시뮬레이션, GUI, 점유 지도, 가상 GPS, 기록 및 자율주행', 'file': 'simulation_run.md'}, {'slug': 'lidar-connection', 'title': 'LiDAR 연결과 테스트', 'category': 'Hardware', 'summary': 'RPLIDAR S2, ROS2 /scan owner 구조, WebSocket live map, serial 충돌 방지', 'file': 'lidar_connection.md'}, {'slug': 'lidar-drive-test-recording', 'title': 'LiDAR 주행 테스트 기록', 'category': 'Data', 'summary': '주행 테스트 중 /scan, TF, cmd_vel, Rover 상태를 rosbag2로 수집하는 절차', 'file': 'lidar_drive_test_recording.md'}, {'slug': 'usb-camera-connection', 'title': 'USB 카메라 연결', 'category': 'Hardware', 'summary': 'UVC USB 카메라 인식, ROS2 image topic, dashboard live view 설정', 'file': 'usb_camera_connection.md'}, {'slug': 'imu-ros2-recording', 'title': 'ROS2 IMU 실시간 표시와 기록', 'category': 'Hardware', 'summary': 'BNO085 ROS2 원본 토픽, 소프트웨어 보정 없는 표시·기록 계약', 'file': 'imu_ros2_recording.md'}, {'slug': 'jetson-offline-access-20260827', 'title': 'Jetson 오프라인 Wi-Fi 접속', 'category': 'Network', 'summary': '단일 무선랜 Wi-Fi/AP 자동 전환, 로컬 DNS, Dashboard·SSH·Portainer 접속 계약', 'file': 'jetson_offline_access_20260827.md'}, {'slug': 'esp-rover-control', 'title': 'ESP8266 연동', 'category': 'Hardware', 'summary': 'ESP USB motor owner, manual lease and ROS status 기준', 'file': 'esp_rover_control.md'}, {'slug': 'rover-control-dashboard', 'title': 'Rover Control Dashboard', 'category': 'Data', 'summary': '주행 제어, live 센서, LiDAR/카메라 기록, archive 조회와 다운로드', 'file': 'rover_control_dashboard.md'}, {'slug': 'rover-control-todo', 'title': 'Rover Control TODO', 'category': 'Data', 'summary': 'Rover Control, 카메라/LiDAR 디버깅, 아카이브 재생 개선 작업 목록', 'file': 'rover_control_todo.md'}, {'slug': 'jetson-runtime-architecture', 'title': 'Jetson 런타임 아키텍처', 'category': 'Deployment', 'summary': 'ROS2 real bringup, LiDAR bridge, dashboard, fan control 서비스 구성', 'file': 'jetson_runtime_architecture.md'}, {'slug': 'fan-control', 'title': 'Fan Control', 'category': 'Hardware', 'summary': 'Jetson 온도 확인과 수동 PWM 유지', 'file': 'fan_control.md'}, {'slug': 'cloudflared', 'title': 'Cloudflare Tunnel', 'category': 'Network', 'summary': 'bts.sarihub.com 터널과 localhost:8080 연결 구성', 'file': 'cloudflared.md'}, {'slug': 'telemetry-schema', 'title': '텔레메트리 스키마', 'category': 'Data', 'summary': 'ROS2 bridge가 dashboard server로 넘길 JSON 구조', 'file': 'telemetry_schema.md'}, {'slug': 'research-metrics', 'title': '연구 지표', 'category': 'Research', 'summary': '원본 카메라·LiDAR·IMU 수신 상태와 수동 조종 지표', 'file': 'research_metrics.md'}]


class DashboardHandler(BaseHTTPRequestHandler):
    provider: TelemetryProvider
    protocol_version = "HTTP/1.1"
    _last_telemetry_log_at = 0.0

    def _send_index(self):
        body = (STATIC_ROOT / "index.html").read_text(encoding="utf-8")
        if self.provider.mode == "simulation":
            if getattr(self.provider, 'external_native_pc', False):
                body=body.replace('data-runtime-mode="live"','data-runtime-mode="simulation" data-external-native-pc="true"')
            body = body.replace('data-runtime-mode="live"', 'data-runtime-mode="simulation"')
            body = body.replace('<title>UBTS Dashboard</title>', '<title>SIM · UBTS Dashboard</title>')
            body = body.replace('<div hidden="" class="simulation-banner"', '<div class="simulation-banner"')
            if getattr(self.provider, 'native_owner_enabled', False):
                body = body.replace('data-native-owner-enabled="false"', 'data-native-owner-enabled="true"')
            if getattr(self.provider, 'native_view_enabled', False):
                body = body.replace('data-native-view-enabled="false"', 'data-native-view-enabled="true"')
        payload = body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(payload)

    def do_HEAD(self):
        self.do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path.startswith("/api/sim/"):
            self._simulation_mapping_request(parsed, action=True)
        elif path.startswith("/api/rover/"):
            self._rover_action(path)
        elif path.startswith("/occupancy-api"):
            self._proxy_occupancy_request(parsed)
        elif path == "/api/lidar/test":
            self._send_json(self.provider.lidar_test())
        elif path == "/api/lidar/scan":
            self._send_json(self.provider.lidar_scan())
        elif path == "/api/camera/settings":
            payload = self._read_json_body()
            self._send_json(self.provider.set_camera_settings(payload))
        elif path == "/api/recordings/media-jobs/action":
            self._recording_media_action()
        elif path == "/api/recordings/start":
            payload = self._read_json_body()
            self._send_json(self.provider.start_recording(payload))
        elif path == "/api/recordings/stop":
            self._send_json(self.provider.stop_recording())
        elif path == "/api/imu/zero":
            self._send_json(self.provider.zero_imu(), status=410)
        elif path == "/api/network/mode":
            payload = self._read_json_body()
            self._send_json(self.provider.set_network_mode(payload))
        elif path.startswith("/rover-proxy/") or path == "/rover-proxy":
            self._proxy_rover_request(parsed)
        elif path == "/api/fan/mode":
            payload = self._read_json_body()
            self._send_json(self.provider.set_fan_mode(bool(payload.get("auto"))))
        elif path == "/api/fan/pwm":
            payload = self._read_json_body()
            self._send_json(self.provider.set_fan_pwm(payload.get("pwm")))
        elif path == "/api/fan/preset":
            payload = self._read_json_body()
            self._send_json(self.provider.set_fan_preset(str(payload.get("preset", ""))))
        elif path.startswith("/api/docker/container/"):
            self._post_docker_container_action(path)
        else:
            self.send_error(404, "Not found")

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path.startswith("/api/sim/"):
            self._simulation_mapping_request(parsed)
        elif path.startswith("/occupancy-api"):
            self._proxy_occupancy_request(parsed)
        elif path == "/ws/lidar":
            self._proxy_lidar_websocket()
        elif path == "/ws/camera":
            self._proxy_camera_websocket()
        elif path == "/ws/imu":
            self._proxy_imu_websocket()
        elif path == "/ws/gps":
            self._proxy_local_websocket(label="gps", registry=_ACTIVE_GPS_WS,
                default_upstream=os.environ.get("GPS_WS_UPSTREAM", "ws://127.0.0.1:8778/ws/gps"),
                unavailable_payload={"type": "gps", "state": "unavailable", "valid": False, "connected": False})
        elif path == "/api/maps/status":
            from dashboard_server.offline_maps import map_status
            self._send_json(map_status())
        elif path.startswith("/maps/"):
            self._send_map_asset(unquote(path.removeprefix("/maps/")))
        elif path == "/api/gps":
            from dashboard_server.gps import fresh_snapshot
            try:
                gps = json.loads(Path(os.environ.get("GPS_BRIDGE_STATUS_FILE", str(ROOT / "runtime/gps_bridge_status.json"))).read_text())
            except (OSError, ValueError):
                gps = {}
            self._send_json(fresh_snapshot(gps, gps.get("received_at"), driver=gps.get("driver")))
        elif path in APP_ROUTES or path.startswith("/docs/"):
            self._send_index()
        elif path == "/api/health":
            self._send_json(self.provider.health())
        elif path == "/api/telemetry":
            payload = self.provider.telemetry()
            payload["connection"]["lidar_websocket"] = lidar_websocket_stats()
            payload["connection"]["camera_websocket"] = camera_websocket_stats()
            payload["connection"]["imu_websocket"] = imu_websocket_stats()
            self._send_json(payload)
        elif path == "/api/system":
            self._send_json(self.provider.system())
        elif path == "/api/network":
            self._send_json(self.provider.network())
        elif path == "/api/docker":
            self._send_json(self.provider.docker())
        elif path.startswith("/api/docker/container/"):
            self._send_docker_container_payload(path, parsed.query)
        elif path == "/api/connection":
            payload = self.provider.connection()
            payload["lidar_websocket"] = lidar_websocket_stats()
            payload["camera_websocket"] = camera_websocket_stats()
            payload["imu_websocket"] = imu_websocket_stats()
            self._send_json(payload)
        elif path == "/api/rover/status":
            self._send_json(self.provider.rover_status())
        elif path == "/api/recordings/media-jobs":
            self._send_json(self.provider._recorder_get("/media-jobs"))
        elif path == "/api/recordings/status":
            self._send_json(self.provider.recording_status())
        elif path == "/api/recordings":
            self._send_json(self.provider.recordings())
        elif path.startswith("/api/recordings/"):
            self._send_recording_payload(path, parsed.query)
        elif path == "/api/plan":
            self._send_json(self.provider.plan())
        elif path == "/api/site-map":
            self._send_json(self.provider.site_map())
        elif path == "/api/fan/status":
            self._send_json(self.provider.fan_status())
        elif path == "/api/docs":
            self._send_json(self._docs_index())
        elif path.startswith("/api/docs/"):
            self._send_json(self._doc_payload(path.removeprefix("/api/docs/")))
        elif path.startswith("/docs-assets/"):
            self._send_file(DOCS_ROOT / "assets" / path.removeprefix("/docs-assets/"), allowed_root=DOCS_ROOT / "assets")
        elif path.startswith("/rover-proxy/") or path == "/rover-proxy":
            self._proxy_rover_request(parsed)
        elif path.startswith("/static/"):
            self._send_file(STATIC_ROOT / path.removeprefix("/static/"))
        else:
            self.send_error(404, "Not found")

    def do_DELETE(self):
        parsed = urlparse(self.path)
        if parsed.path.startswith("/occupancy-api"):
            self._proxy_occupancy_request(parsed)
        else:
            self.send_error(404, "Not found")

    def _simulation_mapping_request(self, parsed, action=False):
        if self.provider.mode != "simulation":
            self.send_error(404, "Simulation only")
            return
        if getattr(self.provider, 'external_native_pc', False):
            from .external_native_pc import admit_request,managed
            if parsed.path=='/api/sim/external-native-pc-view' and not action:
                try:
                    if parsed.query or len(self.headers.get_all('Host',[]))!=1 or len(self.headers.get_all('Origin',[]))>1 or len(self.headers.get_all('Sec-Fetch-Site',[]))>1:
                        raise ValueError('external_view_exact_request_headers')
                    admit_request(self.command,self.headers.get('Host'),self.headers.get('Origin'),self.headers.get('Sec-Fetch-Site'))
                    value=self.provider.mapping.view()
                except (ValueError,OSError,UnicodeError,RecursionError):
                    self._send_json({'ok':False,'error':'external_native_view_unavailable_or_request_rejected','map_health_observed':False},status=403)
                    return
                self._send_json(value)
                return
            self._send_json(managed(),status=409)
            return
        if parsed.path.startswith(('/api/sim/derived-viewport', '/api/sim/display-overview')):
            self._derived_owner_request(parsed, action)
            return
        if parsed.path.startswith('/api/sim/native-observer/'):
            self._native_owner_request(parsed, action)
            return
        if parsed.path == '/api/sim/native-map' or parsed.path.startswith('/api/sim/native-map/'):
            proxy = getattr(self.provider, 'native_view', None)
            if proxy is None:
                self.send_error(404, 'Optional native view disabled')
                return
            target = parsed.path + ('?' + parsed.query if parsed.query else '')
            with proxy.response(self.command, target, self.headers) as response:
                self._send_native_view_response(response)
            return
        if action:
            origin = self.headers.get("Origin")
            if origin and urlparse(origin).netloc != self.headers.get("Host"):
                self.send_error(403, "Origin mismatch")
                return
            payload = self._read_json_body()
        path = parsed.path
        if path=='/api/sim/mapping-jobs' and not action:
            if not getattr(self.provider,'source_only_mapping',False):self.send_error(404);return
            self._send_json(self.provider.mapping_operations.status())
        elif path=='/api/sim/mapping-jobs/stop' and action:
            if not getattr(self.provider,'source_only_mapping',False):self.send_error(404);return
            if not self._ensure_session(create=False):self.send_error(403);return
            if not isinstance(payload,dict):self.send_error(400);return
            if payload.get('job_id') == 'native-observer':
                if set(payload) != {'job_id','expected_epoch','source_session'}:self.send_error(400);return
                self._native_owner_action('stop', {'expected_control_epoch':payload['expected_epoch'],'source_session':payload['source_session']})
                return
            self._send_json(self.provider.mapping_operations.stop(payload.get('job_id'),payload.get('expected_epoch')))
        elif path == '/api/sim/navigation' and not action:
            if not getattr(self.provider, 'source_only_mapping', False):self.send_error(404);return
            self._send_json(self.provider.mapping.navigation.snapshot())
        elif path == '/api/sim/autonomy/preflight' and not action:
            from .field_preflight import report
            live=self.provider.mapping.task_status() if getattr(self.provider,'source_only_mapping',False) else None
            self._send_json(report(ROOT/'config/field_autonomy_preparation.json',live))
        elif path == "/api/sim/autonomy" or path.startswith("/api/sim/autonomy/"):
            from dashboard_server.rover_api import request
            session = self._ensure_session(create=False)
            if not session:
                self.send_error(403, "Dashboard session required"); return
            op = path.rsplit("/", 1)[-1] if action else "status"
            if op not in {"start", "stop", "heartbeat", "status", "speed"}:
                self.send_error(404); return
            if action and not isinstance(payload, dict):
                self.send_error(400, "Object required"); return
            if getattr(self.provider, 'source_only_mapping', False):
                if op == 'start':
                    self._send_json({'ok':False,'error':'native_mapper_observation_only',
                        'reason':'센서 기반 지도의 자율 제어 검증이 필요합니다. 수동 시뮬레이션은 사용할 수 있습니다.'})
                    return
                if op == 'status':
                    from .field_preflight import report
                    status=request({'op':'autonomy_status','owner':session})
                    self._send_json({**status,'start_available':False,
                        'mapping_profile':'native_sensors','runtime_authority':False,
                        'navigation':self.provider.mapping.navigation.snapshot(),
                        'preparation':report(ROOT/'config/field_autonomy_preparation.json',self.provider.mapping.task_status()),
                        'reason':'LiDAR·IMU·GPS 지도 검증 중 · 자율 제어 보류'})
                    return
            self._send_json(request({**(payload if action else {}), "op": "autonomy_"+op, "owner": session}))
        elif not action and path == "/api/sim/map":
            query=parse_qs(parsed.query);since=query.get('since',[None])[0]
            if query.get('transport',[None])[0]=='tiles':
                self._send_json(self.provider.simulation_map(since,transport='tiles',
                    tile_since=query.get('tile_since',[None])[0],tile_epoch=query.get('tile_epoch',[None])[0]))
            else:self._send_json(self.provider.simulation_map(since))
        elif not action and path == "/api/sim/map-history":
            self._send_json(self.provider.simulation_map_history())
        elif not action and path.startswith("/api/sim/map-history/"):
            self._send_json(self.provider.simulation_map_history(unquote(path.removeprefix("/api/sim/map-history/"))))
        elif action and path in {"/api/sim/map/start", "/api/sim/map/stop", "/api/sim/map/reset"}:
            options={'expected_epoch':payload.get('expected_epoch')} if getattr(self.provider,'source_only_mapping',False) and isinstance(payload,dict) else {}
            self._send_json(self.provider.simulation_map_action(path.rsplit("/", 1)[-1],transport='tiles',**options))
        elif path.startswith("/api/sim/recordings/"):
            parts = path.removeprefix("/api/sim/recordings/").split("/")
            if len(parts) != 2:
                self.send_error(404, "Not found")
            elif not action and parts[1] == "map":
                self._send_json(self.provider.simulation_archive_map(unquote(parts[0])))
            elif action and parts[1] == "regenerate":
                self._send_json(self.provider.simulation_archive_regenerate(unquote(parts[0])))
            else:
                self.send_error(404, "Not found")
        else:
            self.send_error(404, "Not found")

    def _derived_owner_request(self, parsed, action=False):
        service_name = 'overview_service' if parsed.path.startswith('/api/sim/display-overview') else 'derived_owner_service'
        service = getattr(self.provider, service_name, None)
        if service is None:
            self.send_error(404, 'Optional derived view disabled')
            return
        payload = None
        if action:
            origin = self.headers.get('Origin')
            if origin and urlparse(origin).netloc != self.headers.get('Host'):
                self.send_error(403, 'Origin mismatch')
                return
            if not self._ensure_session(create=False):
                self.send_error(403, 'Session required')
                return
            lengths = self.headers.get_all('Content-Length', [])
            length = lengths[0] if len(lengths) == 1 else ''
            if (self.headers.get('Transfer-Encoding') is not None or
                    not 1 <= len(length) <= 4 or not length.isascii() or
                    not length.isdigit() or not 1 <= int(length) <= 4096):
                self.send_error(413, 'Bounded Stop body required')
                return
            self.close_connection = True
            deadline = time.monotonic() + 3
            data = bytearray()
            try:
                while len(data) < int(length):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('Stop body deadline')
                    self.connection.settimeout(remaining)
                    chunk = self.rfile.read1(int(length) - len(data))
                    if not chunk:
                        raise ValueError('Incomplete Stop body')
                    data.extend(chunk)
                if time.monotonic() >= deadline:
                    raise TimeoutError('Stop body deadline')
                def exact_pairs(pairs):
                    result = {}
                    for key, value in pairs:
                        if key in result:
                            raise ValueError('Duplicate Stop key')
                        result[key] = value
                    return result
                def invalid_constant(value):
                    raise ValueError('Nonfinite Stop JSON')
                payload = json.loads(data.decode('utf-8'), object_pairs_hook=exact_pairs,
                                     parse_constant=invalid_constant)
            except (TimeoutError, OSError):
                self.send_error(408, 'Stop body deadline')
                return
            except (ValueError, RecursionError):
                self.send_error(400, 'Invalid Stop JSON')
                return
        target = parsed.path + ('?' + parsed.query if parsed.query else '')
        response = service.route(self.command, target, payload)
        body = response['body']
        if type(body) is not bytes or len(body) > 8 * 1024 * 1024:
            self.send_error(502, 'Derived response bound')
            return
        self.close_connection = True
        self.connection.settimeout(3)
        token = self._ensure_session(create=True)
        self.send_response(response['status'])
        for name, value in response['headers'].items():
            self.send_header(name, value)
        self.send_header('Connection', 'close')
        self._send_session_cookie(token)
        self.end_headers()
        if self.command == 'GET' or self.command == 'POST':
            self.wfile.write(body)

    def _native_owner_request(self, parsed, action=False):
        owner = getattr(self.provider, 'native_owner', None)
        if owner is None or not getattr(self.provider, 'native_owner_enabled', False):self.send_error(404);return
        if parsed.query:self.send_error(400);return
        if not action and parsed.path == '/api/sim/native-observer/status' and self.command == 'GET':
            result = owner.status();self._send_json(result, status=200 if result.get('ok') else 503);return
        if action and self.command == 'POST' and parsed.path in {'/api/sim/native-observer/start','/api/sim/native-observer/stop'}:
            self._native_owner_action(parsed.path.rsplit('/',1)[1], self._read_json_body());return
        self.send_error(405)

    def _native_owner_action(self, op, payload):
        owner = getattr(self.provider, 'native_owner', None)
        if owner is None or not getattr(self.provider, 'native_owner_enabled', False):self.send_error(404);return
        origin = self.headers.get('Origin');host = self.headers.get('Host')
        if not origin or not host or urlparse(origin).scheme not in {'http','https'} or origin != urlparse(origin).scheme+'://'+host:
            self.send_error(403, 'Same-origin request required');return
        if not self._ensure_session(create=False):self.send_error(403, 'Dashboard session required');return
        from .sim_native_owner_proxy import token
        if (op not in {'start','stop'} or type(payload) is not dict or
                set(payload) != {'expected_control_epoch','source_session'} or
                not token(payload['expected_control_epoch']) or not token(payload['source_session'])):
            self.send_error(400);return
        result = (owner.start if op=='start' else owner.stop)(payload['expected_control_epoch'],payload['source_session'])
        self._send_json(result, status=202 if result.get('ok') else 409)

    def _recording_media_action(self):
        origin = self.headers.get("Origin")
        if origin and urlparse(origin).netloc != self.headers.get("Host"):
            self.send_error(403, "Origin mismatch"); return
        if not self._ensure_session(create=False):
            self.send_error(403, "Dashboard session required"); return
        payload = self._read_json_body()
        if (not isinstance(payload, dict) or payload.get('action') not in ('pause','resume','cancel')
                or not isinstance(payload.get('recording_id'), str)
                or not 1 <= len(payload['recording_id']) <= 128):
            self._send_json(dict(ok=False,error='invalid_media_action'),400); return
        result = self.provider._recorder_post('/media-jobs/action',
            {key:payload[key] for key in ('recording_id','action')})
        self._send_json(result,200 if result.get('ok') else 409)

    def _rover_action(self, path):
        from dashboard_server.rover_api import request
        origin = self.headers.get("Origin")
        if origin and urlparse(origin).netloc != self.headers.get("Host"):
            self.send_error(403, "Origin mismatch")
            return
        session = self._ensure_session(create=False)
        if not session:
            self.send_error(403, "Dashboard session required")
            return
        payload = self._read_json_body()
        if not isinstance(payload, dict):
            self.send_error(400, "Object required")
            return
        operations = {"/api/rover/arm": "arm", "/api/rover/command": "drive", "/api/rover/release": "release",
                      "/api/rover/keepalive": "keepalive", "/api/rover/stop": "stop"}
        if path not in operations:
            self.send_error(404)
            return
        self._send_json(request({**payload, "op": operations[path], "owner": session}))

    def log_message(self, fmt, *args):
        if os.environ.get("DASHBOARD_ACCESS_LOG", "0") == "1":
            super().log_message(fmt, *args)

    def _send_native_view_response(self, response):
        # Proxy permit remains held through this bounded byte write.
        self.close_connection = True
        remaining = response.deadline - time.monotonic()
        if remaining <= 0:
            return
        try:
            self.connection.settimeout(remaining)
            session_token = self._ensure_session(create=True)
            self.send_response(response.status)
            for name, value in response.headers:
                self.send_header(name, value)
            self.send_header('Connection', 'close')
            self._send_session_cookie(session_token)
            remaining = response.deadline - time.monotonic()
            if remaining <= 0:
                return
            self.connection.settimeout(remaining)
            self.end_headers()
            if self.command == 'GET':
                remaining = response.deadline - time.monotonic()
                if remaining <= 0:
                    return
                self.connection.settimeout(remaining)
                self.wfile.write(response.body)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def _send_json(self, payload, status=200):
        session_token = self._ensure_session(create=True)
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        accepts_gzip = "gzip" in self.headers.get("Accept-Encoding", "").lower()
        encoded = gzip.compress(body, compresslevel=1) if accepts_gzip and len(body) >= 1024 else body
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.send_header("Vary", "Accept-Encoding")
        if encoded is not body:
            self.send_header("Content-Encoding", "gzip")
        self._send_session_cookie(session_token)
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        if self.command != "HEAD":
            try:
                self.wfile.write(encoded)
            except (BrokenPipeError, ConnectionResetError, OSError):
                self.close_connection = True
                return
        self.close_connection = True
        self._log_served_json()

    def _send_bytes(self, body: bytes, content_type: str, filename: str | None = None):
        session_token = self._ensure_session(create=True)
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self._send_session_cookie(session_token)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)
        self.close_connection = True

    def _read_json_body(self):
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return {}

    def _docs_index(self):
        return {"documents": [self._doc_meta(item) for item in DOCS_CATALOG]}

    def _doc_payload(self, raw_slug: str):
        slug = unquote(raw_slug).strip("/")
        item = next((row for row in DOCS_CATALOG if row["slug"] == slug), None)
        if not item:
            return {"ok": False, "error": "document_not_found", "slug": slug}
        path = (DOCS_ROOT / item["file"]).resolve()
        try:
            if DOCS_ROOT.resolve() not in path.parents:
                return {"ok": False, "error": "document_outside_root", "slug": slug}
            markdown = path.read_text(encoding="utf-8")
        except OSError as exc:
            return {"ok": False, "error": str(exc), "slug": slug}
        return {"ok": True, **self._doc_meta(item), "markdown": markdown}

    def _doc_meta(self, item):
        path = DOCS_ROOT / item["file"]
        mtime = int(path.stat().st_mtime) if path.exists() else None
        return {
            "slug": item["slug"],
            "title": item["title"],
            "category": item["category"],
            "summary": item["summary"],
            "source": f"docs/{item['file']}",
            "updated_at": mtime,
            "url": f"/docs/{item['slug']}",
        }

    def _send_file(self, path: Path, allowed_root: Path = STATIC_ROOT):
        session_token = self._ensure_session(create=True)
        try:
            resolved = path.resolve()
            root = allowed_root.resolve()
            if root not in resolved.parents and resolved != root:
                self.send_error(403, "Forbidden")
                return
            info = resolved.stat()
        except OSError:
            self.send_error(404, "Not found")
            return
        content_type = mimetypes.guess_type(str(resolved))[0] or "application/octet-stream"
        if resolved.suffix in (".js", ".mjs"):
            content_type = "application/javascript; charset=utf-8"
        elif resolved.suffix == ".css":
            content_type = "text/css; charset=utf-8"
        elif resolved.suffix == ".html":
            content_type = "text/html; charset=utf-8"
        compressed = 'gzip' in self.headers.get('Accept-Encoding', '') and resolved.suffix in ('.js', '.mjs', '.css', '.html', '.json', '.svg')
        etag = f'"{info.st_mtime_ns:x}-{info.st_size:x}-{"gz" if compressed else "raw"}"'
        if self.headers.get('If-None-Match') == etag:
            self.send_response(304)
            self.send_header('ETag', etag)
            self.send_header('Cache-Control', 'private, max-age=0, must-revalidate')
            self.send_header('Vary', 'Accept-Encoding')
            self._send_session_cookie(session_token)
            self.end_headers()
            return
        try:
            body = resolved.read_bytes()
        except OSError:
            self.send_error(404, 'Not found')
            return
        if compressed: body = gzip.compress(body, compresslevel=6, mtime=0)
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "private, max-age=0, must-revalidate")
        self.send_header('ETag', etag)
        self.send_header('Vary', 'Accept-Encoding')
        if compressed: self.send_header('Content-Encoding', 'gzip')
        self.send_header("Connection", "close")
        self._send_session_cookie(session_token)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if self.command != "HEAD":
            try: self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, OSError): pass
        self.close_connection = True

    def _ensure_session(self, create: bool) -> str | None:
        now = time.time()
        try:
            cookie = SimpleCookie(self.headers.get("Cookie", ""))
        except CookieError:
            cookie = SimpleCookie()
        token = cookie.get(SESSION_COOKIE_NAME).value if SESSION_COOKIE_NAME in cookie else None
        with _SESSION_LOCK:
            _prune_sessions(now)
            if token and token in _SESSIONS:
                return token
            if not create:
                return None
            token = secrets.token_urlsafe(24)
            _SESSIONS[token] = {"created_at": now, "expires_at": now + SESSION_TTL_S}
        dashboard_log("session.created", session=token[:8], remote=self.client_address[0])
        return token

    def _send_session_cookie(self, token: str | None):
        if not token:
            return
        self.send_header(
            "Set-Cookie",
            f"{SESSION_COOKIE_NAME}={token}; Path=/; Max-Age={SESSION_TTL_S}; HttpOnly; SameSite=Lax",
        )

    def _log_served_json(self):
        path = urlparse(self.path).path
        now = time.time()
        if path == "/api/telemetry":
            if now - DashboardHandler._last_telemetry_log_at < 30:
                return
            DashboardHandler._last_telemetry_log_at = now
            dashboard_log(
                "api.telemetry",
                active_lidar_ws=lidar_websocket_stats()["active"],
                active_camera_ws=camera_websocket_stats()["active"],
            )
            return
        if path not in {"/api/health"}:
            dashboard_log(
                "api.request",
                path=path,
                active_lidar_ws=lidar_websocket_stats()["active"],
                active_camera_ws=camera_websocket_stats()["active"],
            )

    def _send_recording_payload(self, path: str, query: str = ""):
        raw = path.removeprefix("/api/recordings/").strip("/")
        if not raw:
            self.send_error(404, "Recording id required")
            return
        if "/adaptive-video/" in raw:
            recording_id, filename = raw.split("/adaptive-video/", 1)
            asset = self.provider.recording_adaptive_asset(unquote(recording_id), unquote(filename))
            if not asset:
                self.send_error(404, "Adaptive recording asset not found")
                return
            self._send_range_file(asset)
            return
        if "/camera-frame/" in raw:
            recording_id, filename = raw.split("/camera-frame/", 1)
            frame = self.provider.recording_camera_frame(unquote(recording_id), unquote(filename))
            if not frame:
                self.send_error(404, "Recording camera frame not found")
                return
            self._send_recording_camera_frame(frame)
            return
        if raw.endswith("/video"):
            recording_id = unquote(raw.removesuffix("/video").strip("/"))
            video = self.provider.recording_video(recording_id)
            if not video:
                self.send_error(404, "Recording video not found")
                return
            self._send_recording_video(video)
            return
        if raw.endswith("/download"):
            recording_id = unquote(raw.removesuffix("/download").strip("/"))
            archive = self.provider.recording_archive_path(recording_id)
            if archive is None or not archive.exists():
                self.send_error(404, "Recording not found")
                return
            self._send_recording_archive_download(archive, recording_id)
            return
        if raw.endswith("/timeline"):
            recording_id = unquote(raw.removesuffix("/timeline").strip("/"))
            params = parse_qs(query or "")
            start_s = (params.get("start_s") or [None])[0]
            duration_s = (params.get("duration_s") or [None])[0]
            profile = (params.get("profile") or [None])[0]
            self._send_json(
                self.provider.recording_timeline(
                    recording_id,
                    start_s=start_s,
                    duration_s=duration_s,
                    profile=profile,
                )
            )
            return
        if raw.endswith("/orchard-map"):
            self.send_error(404)
            return
        self._send_json(self.provider.recording_detail(unquote(raw)))

    def _send_recording_camera_frame(self, frame: dict[str, object]):
        path = frame.get("path")
        if not isinstance(path, Path):
            self.send_error(404, "Recording camera frame not found")
            return
        try:
            body = path.read_bytes()
        except OSError:
            self.send_error(404, "Recording camera frame not found")
            return
        etag = f'"{frame.get("etag") or path.stem}"'
        if self.headers.get("If-None-Match") == etag:
            self.send_response(304)
            self.send_header("ETag", etag)
            self.send_header("Cache-Control", "private, max-age=86400, immutable")
            self.end_headers()
            return
        session_token = self._ensure_session(create=True)
        self.send_response(200)
        self.send_header("Content-Type", str(frame.get("mime") or "image/jpeg"))
        self.send_header("Cache-Control", "private, max-age=86400, immutable")
        self.send_header("ETag", etag)
        self.send_header("Content-Length", str(len(body)))
        self._send_session_cookie(session_token)
        self.end_headers()
        if self.command != "HEAD":
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, OSError):
                self.close_connection = True

    def _send_map_asset(self, relative):
        from dashboard_server.offline_maps import asset_path
        path = asset_path(relative)
        if path is None:
            self.send_error(404, "Offline map asset not found")
            return
        if path.suffix == ".pmtiles" and not self.headers.get("Range") and self.command != "HEAD":
            self.send_error(400, "A byte range is required for this map archive")
            return
        mime = {".pmtiles": "application/octet-stream", ".pbf": "application/x-protobuf",
                ".woff2": "font/woff2"}.get(path.suffix) or mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        self._send_range_file({"path": path, "mime": mime})

    def _send_recording_video(self, video: dict[str, object]):
        self._send_range_file(video)

    def _send_range_file(self, video: dict[str, object]):
        path = video.get("path")
        if not isinstance(path, Path):
            self.send_error(404, "Recording video not found")
            return
        try:
            info = path.stat()
            size = info.st_size
        except OSError:
            self.send_error(404, "Recording video not found")
            return
        start, end, partial = self._requested_byte_range(size)
        if start is None or end is None:
            self.send_response(416)
            self.send_header("Content-Range", f"bytes */{size}")
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        length = end - start + 1
        etag = f'"{info.st_mtime_ns:x}-{size:x}"'
        if self.headers.get('If-Match') not in (None, '*', etag):
            self.send_response(412)
            self.send_header('ETag', etag)
            self.send_header('Content-Length', '0')
            self.end_headers()
            return
        session_token = self._ensure_session(create=True)
        self.send_response(206 if partial else 200)
        self.send_header("Content-Type", str(video.get("mime") or "video/mp4"))
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("ETag", etag)
        self.send_header("Content-Length", str(length))
        self.send_header("Cache-Control", "private, max-age=3600")
        self.send_header("Content-Disposition", f'inline; filename="{path.name}"')
        if partial:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self._send_session_cookie(session_token)
        self.end_headers()
        if self.command == "HEAD":
            return
        try:
            with path.open("rb") as handle:
                handle.seek(start)
                remaining = length
                while remaining > 0:
                    block = handle.read(min(1024 * 1024, remaining))
                    if not block:
                        break
                    self.wfile.write(block)
                    remaining -= len(block)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.close_connection = True

    def _requested_byte_range(self, size: int) -> tuple[int | None, int | None, bool]:
        header = str(self.headers.get("Range") or "").strip()
        if not header:
            return 0, max(0, size - 1), False
        if not header.startswith("bytes=") or "," in header or size <= 0:
            return None, None, True
        value = header.removeprefix("bytes=").strip()
        try:
            raw_start, raw_end = value.split("-", 1)
            if raw_start:
                start = int(raw_start)
                end = int(raw_end) if raw_end else size - 1
            else:
                suffix = int(raw_end)
                if suffix <= 0:
                    raise ValueError("invalid suffix")
                start = max(0, size - suffix)
                end = size - 1
        except (TypeError, ValueError):
            return None, None, True
        if start < 0 or start >= size or end < start:
            return None, None, True
        return start, min(end, size - 1), True

    def _send_recording_archive_download(self, archive: Path, recording_id: str):
        filename = f"{archive.name}.tar.gz"
        cache_path = self._recording_download_cache_path(archive)
        if self._recording_download_cache_current(cache_path, archive):
            self._send_download_file(cache_path, "application/gzip", filename=filename)
            return
        if self.command == "HEAD":
            self._send_download_headers("application/gzip", filename=filename)
            return
        tmp_path = cache_path.with_suffix(f".{secrets.token_hex(6)}.tmp")
        try:
            cache_path.parent.mkdir(parents=True, exist_ok=True)
            with tmp_path.open("wb") as cache_file:
                self._send_download_headers("application/gzip", filename=filename)
                writer = _TeeWriter(self.wfile, cache_file)
                with gzip.GzipFile(fileobj=writer, mode="wb", compresslevel=1) as gzip_file:
                    with tarfile.open(fileobj=gzip_file, mode="w|") as tar:
                        tar.add(archive, arcname=archive.name)
                writer.flush()
            tmp_path.replace(cache_path)
        except OSError as exc:
            tmp_path.unlink(missing_ok=True)
            dashboard_log("recording.download.error", id=recording_id, error=str(exc))
        finally:
            self.close_connection = True

    def _recording_download_cache_path(self, archive: Path) -> Path:
        return archive.parent.parent / "recording_downloads" / f"{archive.name}.tar.gz"

    def _recording_download_cache_current(self, cache_path: Path, archive: Path) -> bool:
        try:
            if not cache_path.exists() or cache_path.stat().st_size <= 0:
                return False
            newest = archive.stat().st_mtime
            for item in archive.rglob("*"):
                if item.is_file():
                    newest = max(newest, item.stat().st_mtime)
            return cache_path.stat().st_mtime >= newest
        except OSError:
            return False

    def _send_download_file(self, path: Path, content_type: str, filename: str | None = None):
        try:
            size = path.stat().st_size
        except OSError:
            self.send_error(404, "File not found")
            return
        self._send_download_headers(content_type, filename=filename, content_length=size)
        if self.command != "HEAD":
            with path.open("rb") as handle:
                while True:
                    chunk = handle.read(1024 * 1024)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
        self.close_connection = True

    def _send_download_headers(self, content_type: str, filename: str | None = None, content_length: int | None = None):
        session_token = self._ensure_session(create=True)
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        if content_length is not None:
            self.send_header("Content-Length", str(content_length))
        self._send_session_cookie(session_token)
        self.end_headers()

    def _send_docker_container_payload(self, path: str, query: str):
        raw = path.removeprefix("/api/docker/container/").strip("/")
        if not raw:
            self.send_error(404, "Container id required")
            return
        if raw.endswith("/logs"):
            container_id = unquote(raw.removesuffix("/logs").strip("/"))
            params = parse_qs(query)
            tail = (params.get("tail") or ["160"])[0]
            self._send_json(self.provider.docker_container_logs(container_id, tail=tail))
            return
        self._send_json(self.provider.docker_container_detail(unquote(raw)))

    def _post_docker_container_action(self, path: str):
        raw = path.removeprefix("/api/docker/container/").strip("/")
        parts = [unquote(item) for item in raw.split("/") if item]
        if len(parts) != 2 or parts[1] not in {"start", "stop", "restart"}:
            self.send_error(404, "Unknown container action")
            return
        container_id, action = parts
        result = self.provider.docker_container_action(container_id, action)
        dashboard_log("docker.action", action=action, container=container_id[:12], ok=bool(result.get("ok")), state=result.get("state"))
        self._send_json(result)

    def _proxy_lidar_websocket(self):
        self._proxy_local_websocket(
            label="lidar",
            registry=_ACTIVE_LIDAR_WS,
            default_upstream=os.environ.get("LIDAR_WS_UPSTREAM", "ws://127.0.0.1:8765/ws/lidar"),
            unavailable_payload={
                "type": "status",
                "device_found": False,
                "scan_topic_alive": False,
                "last_scan_age_sec": None,
                "point_count": 0,
                "source": "bridge",
            },
        )

    def _proxy_camera_websocket(self):
        self._proxy_local_websocket(
            label="camera",
            registry=_ACTIVE_CAMERA_WS,
            default_upstream=os.environ.get("CAMERA_WS_UPSTREAM", "ws://127.0.0.1:8770/ws/camera"),
            unavailable_payload={
                "type": "status",
                "frame_topic_alive": False,
                "last_frame_age_sec": None,
                "frame_count": 0,
            },
        )

    def _proxy_imu_websocket(self):
        self._proxy_local_websocket(
            label="imu",
            registry=_ACTIVE_IMU_WS,
            default_upstream=os.environ.get("IMU_WS_UPSTREAM", "ws://127.0.0.1:8775/ws/imu"),
            unavailable_payload={
                "type": "imu",
                "imu_topic_alive": False,
                "corrected_topic_alive": False,
                "last_imu_age_s": None,
                "raw": {},
                "corrected": {},
                "magnetic": {},
                "calibration": {"state": "unavailable", "valid": False},
            },
        )

    def _proxy_local_websocket(
        self,
        label: str,
        registry: dict[str, dict[str, float | str]],
        default_upstream: str,
        unavailable_payload: dict,
    ):
        if self.headers.get("Upgrade", "").lower() != "websocket":
            self.send_error(426, "WebSocket upgrade required")
            return
        client_key = self.headers.get("Sec-WebSocket-Key", "")
        if not client_key:
            self.send_error(400, "Missing Sec-WebSocket-Key")
            return
        session_token = self._ensure_session(create=False)
        if not session_token:
            dashboard_log(f"ws.{label}.rejected", reason="missing_session", remote=self.client_address[0])
            self.send_error(403, f"{label} WebSocket session required")
            return
        # Avoid Nagle/delayed-ACK stalls on JPEG tails and small telemetry ACKs.
        # Limit this to authenticated upgraded connections.
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        now = time.time()
        with _SESSION_LOCK:
            session = _SESSIONS.get(session_token)
            session_deadline = float(session.get("expires_at", now + SESSION_TTL_S)) if session else now
            connection_id = secrets.token_urlsafe(8)
            registry[connection_id] = {"session": session_token, "connected_at": now, "remote": self.client_address[0]}

        accept = base64.b64encode(
            hashlib.sha1((client_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()
        ).decode("ascii")
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.close_connection = True
        dashboard_log(f"ws.{label}.open", session=session_token[:8], connection=connection_id[:8], active=websocket_stats(registry)["active"])

        upstream_url = self._upstream_with_client_query(default_upstream)
        try:
            unavailable_logged = False
            while time.time() < session_deadline:
                try:
                    upstream, initial_upstream = self._connect_ws_upstream(upstream_url, label.upper())
                except OSError as exc:
                    if not unavailable_logged:
                        dashboard_log(f"ws.{label}.upstream_unavailable", session=session_token[:8], error=str(exc))
                        unavailable_logged = True
                    payload = dict(unavailable_payload)
                    payload["error"] = f"{label} WebSocket bridge unavailable: {exc}"
                    ok = self._ws_send_json(
                        self.connection,
                        payload,
                    )
                    if not ok:
                        return
                    time.sleep(min(2.0, max(0.1, session_deadline - time.time())))
                    continue
                try:
                    self._relay_websocket(upstream, initial_upstream, session_deadline)
                finally:
                    try:
                        upstream.close()
                    except OSError:
                        pass
                return
            self._ws_send_json(
                self.connection,
                {"type": "status", "error": f"{label} WebSocket session expired."},
            )
            self._ws_send_close(self.connection)
        finally:
            with _SESSION_LOCK:
                registry.pop(connection_id, None)
            dashboard_log(f"ws.{label}.close", session=session_token[:8], connection=connection_id[:8], active=websocket_stats(registry)["active"])

    def _connect_ws_upstream(self, upstream_url: str, env_prefix: str = "LOCAL") -> tuple[socket.socket, bytes]:
        parsed = urlparse(upstream_url)
        if parsed.scheme != "ws":
            raise OSError(f"{env_prefix}_WS_UPSTREAM must use ws:// for the local bridge")
        host = parsed.hostname or "127.0.0.1"
        port = parsed.port or 80
        path = parsed.path or "/ws/lidar"
        if parsed.query:
            path = f"{path}?{parsed.query}"
        sock = socket.create_connection((host, port), timeout=3)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        key = base64.b64encode(secrets.token_bytes(16)).decode("ascii")
        host_header = f"{host}:{port}"
        request = (
            f"GET {path} HTTP/1.1\r\n"
            f"Host: {host_header}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        ).encode("ascii")
        sock.sendall(request)
        response = b""
        while b"\r\n\r\n" not in response and len(response) < 8192:
            chunk = sock.recv(1024)
            if not chunk:
                break
            response += chunk
        response_head, _, initial = response.partition(b"\r\n\r\n")
        if b" 101 " not in response_head.split(b"\r\n", 1)[0]:
            sock.close()
            raise OSError("local bridge did not accept WebSocket upgrade")
        sock.settimeout(None)
        return sock, initial

    def _upstream_with_client_query(self, upstream_url: str) -> str:
        client_query = urlparse(self.path).query
        if not client_query:
            return upstream_url
        separator = "&" if urlparse(upstream_url).query else "?"
        return f"{upstream_url}{separator}{client_query}"

    def _proxy_rover_request(self, parsed):
        self.send_error(410, "Legacy rover proxy removed")

    def _proxy_occupancy_request(self, parsed):
        """Forward binary map frames and control requests to the local Jetson renderer."""
        base_url = os.environ.get("OCCUPANCY_GRID_URL", "http://127.0.0.1:8810").rstrip("/")
        suffix = parsed.path.removeprefix("/occupancy-api") or "/"
        target_url = f"{base_url}{suffix}"
        if parsed.query:
            target_url = f"{target_url}?{parsed.query}"
        body = None
        headers = {"User-Agent": "UBTS-Dashboard-Occupancy-Proxy/1.0"}
        if self.command in {"POST", "PUT", "PATCH"}:
            body = self.rfile.read(int(self.headers.get("Content-Length", "0") or 0))
            content_type = self.headers.get("Content-Type")
            if content_type:
                headers["Content-Type"] = content_type
        try:
            req = Request(target_url, data=body, headers=headers, method=self.command)
            with urlopen(req, timeout=90) as res:
                self._send_occupancy_proxy_response(res.status, res.headers, res.read())
        except HTTPError as exc:
            self._send_occupancy_proxy_response(exc.code, exc.headers, exc.read())
        except (OSError, URLError) as exc:
            payload = json.dumps(
                {
                    "ok": False,
                    "state": "occupancy_renderer_unreachable",
                    "error": str(exc),
                },
                ensure_ascii=False,
                separators=(",", ":"),
            ).encode("utf-8")
            self.send_response(502)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(payload)
            self.close_connection = True

    def _send_occupancy_proxy_response(self, status: int, upstream_headers, payload: bytes):
        self.send_response(status)
        self.send_header("Content-Type", upstream_headers.get("Content-Type", "application/octet-stream"))
        self.send_header("Cache-Control", upstream_headers.get("Cache-Control", "no-store"))
        self.send_header("Connection", "close")
        metadata = upstream_headers.get("X-OGM-Meta")
        if metadata:
            self.send_header("X-OGM-Meta", metadata)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(payload)
        self.close_connection = True

    def _relay_websocket(self, upstream: socket.socket, initial_upstream: bytes = b"", deadline: float | None = None):
        if initial_upstream:
            try:
                self.connection.sendall(initial_upstream)
            except OSError:
                return
        sockets = [self.connection, upstream]
        while True:
            timeout = 30.0
            if deadline is not None:
                remaining = deadline - time.time()
                if remaining <= 0:
                    self._ws_send_close(self.connection)
                    return
                timeout = min(timeout, max(0.1, remaining))
            readable, _, errored = select.select(sockets, [], sockets, timeout)
            if errored:
                break
            if not readable:
                if deadline is not None and time.time() >= deadline:
                    self._ws_send_close(self.connection)
                    return
                self._ws_send_json(
                    self.connection,
                    {
                        "type": "status",
                        "device_found": False,
                        "scan_topic_alive": False,
                        "last_scan_age_sec": None,
                        "point_count": 0,
                        "error": "LiDAR WebSocket bridge timeout",
                    },
                )
                continue
            for sock in readable:
                try:
                    data = sock.recv(65536)
                except OSError:
                    return
                if not data:
                    return
                target = upstream if sock is self.connection else self.connection
                try:
                    target.sendall(data)
                except OSError:
                    return

    def _ws_send_json(self, sock: socket.socket, payload: dict) -> bool:
        return self._ws_send_frame(sock, 0x1, json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8"))

    def _ws_send_close(self, sock: socket.socket):
        self._ws_send_frame(sock, 0x8, b"\x03\xe8")

    def _ws_send_frame(self, sock: socket.socket, opcode: int, payload: bytes) -> bool:
        length = len(payload)
        header = bytearray([0x80 | opcode])
        if length < 126:
            header.append(length)
        elif length <= 0xFFFF:
            header.extend([126, (length >> 8) & 0xFF, length & 0xFF])
        else:
            header.append(127)
            header.extend(length.to_bytes(8, "big"))
        try:
            sock.sendall(bytes(header) + payload)
        except OSError:
            return False
        return True


class DashboardServer(ThreadingHTTPServer):
    # Simultaneous sensor/map/video clients must not overflow the default
    # five-entry TCP accept backlog and delay manual requests by a SYN retry.
    request_queue_size = 128
    daemon_threads = True
    allow_reuse_address = True


def create_server(host: str, port: int, config: str | None = None) -> ThreadingHTTPServer:
    if os.environ.get("DASHBOARD_MODE") == "simulation":
        from dashboard_server.simulation import SimulationProvider
        DashboardHandler.provider = SimulationProvider(config)
    else:
        DashboardHandler.provider = TelemetryProvider(config)
    dashboard_log("server.create", host=host, port=port, session_ttl_s=SESSION_TTL_S)
    return DashboardServer((host, port), DashboardHandler)


def main():
    parser = argparse.ArgumentParser(description="UNIST orchard rover dashboard server")
    parser.add_argument("--host", default=os.environ.get("DASHBOARD_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("DASHBOARD_PORT", "8080")))
    parser.add_argument("--config", default=os.environ.get("DASHBOARD_CONFIG"))
    args = parser.parse_args()
    server = create_server(args.host, args.port, args.config)
    dashboard_log("server.listen", url=f"http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
