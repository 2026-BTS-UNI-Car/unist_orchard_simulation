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

from dashboard_server.extensions import DashboardSettings, RouteTable
from dashboard_server.static_assets import StaticAssets
from dashboard_server.telemetry import ROOT, TelemetryProvider
from dashboard_server.ws_relay import relay_sensor_stream


STATIC_ROOT = ROOT / "static"
DOCS_ROOT = ROOT / "docs"
# Content-hash addresses, long caching and kept gzip results for /static/ and the page (NET-04).
STATIC_ASSETS = StaticAssets(STATIC_ROOT)
# The eight menu pages (docs/SCREENS_KO.md, UI-16) and the addresses of the six-screen layout and of the removed
# research page, which the page rewrites to the menu page that holds those panels.
APP_ROUTES = {"/", "/connection", "/system", "/network", "/docker", "/fan", "/rover-control", "/docs",
              "/drive", "/map", "/sensors", "/records", "/tuning",
              "/system/resources", "/system/network", "/system/docker", "/system/fan", "/research"}
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


DOCS_CATALOG = [{'slug': 'docs-index', 'title': '문서 색인', 'category': 'System', 'summary': 'docs 폴더의 문서 목록과 지금 내용과 맞는지 표시', 'file': 'README_KO.md'}, {'slug': 'gps-offline-map', 'title': 'GPS · 대한민국 오프라인 지도', 'category': 'Sensors', 'summary': 'GPS UART 드라이버, 지도 추적, 동기 기록 및 배선 진단', 'file': 'gps_offline_map.md'}, {'slug': 'lidar-connection', 'title': 'LiDAR 연결과 테스트', 'category': 'Hardware', 'summary': 'RPLIDAR S2, ROS2 /scan owner 구조, WebSocket live map, serial 충돌 방지', 'file': 'lidar_connection.md'}, {'slug': 'usb-camera-connection', 'title': 'USB 카메라 연결', 'category': 'Hardware', 'summary': 'UVC USB 카메라 인식, ROS2 image topic, dashboard live view 설정', 'file': 'usb_camera_connection.md'}, {'slug': 'imu-ros2-recording', 'title': 'ROS2 IMU 실시간 표시와 기록', 'category': 'Hardware', 'summary': 'BNO085 ROS2 원본 토픽, 소프트웨어 보정 없는 표시·기록 계약', 'file': 'imu_ros2_recording.md'}, {'slug': 'jetson-offline-access', 'title': 'Jetson 오프라인 Wi-Fi 접속', 'category': 'Network', 'summary': '단일 무선랜 Wi-Fi/AP 자동 전환, 로컬 DNS, Dashboard·SSH·Portainer 접속 계약', 'file': 'jetson_offline_access.md'}, {'slug': 'wifi-ap-checklist', 'title': '차의 무선 AP 점검표', 'category': 'Network', 'summary': '채널·대역·출력·안테나·거리 현장 점검, 연결 기록(?link_log=1)과 등급 경계 맞추기', 'file': 'WIFI_AP_CHECKLIST_KO.md'}, {'slug': 'esp-rover-control', 'title': 'ESP32-S3 · ESP8266 연동', 'category': 'Hardware', 'summary': 'ESP32-S3(BTS7960 3개) 조종 브리지와 구동 안전, 옛 ESP8266 V2 조종권과 정지', 'file': 'esp_rover_control.md'}, {'slug': 'rover-control-dashboard', 'title': 'Rover Control Dashboard', 'category': 'Data', 'summary': '주행 제어, live 센서, LiDAR/카메라 기록, archive 조회와 다운로드', 'file': 'rover_control_dashboard.md'}, {'slug': 'fan-control', 'title': 'Fan Control', 'category': 'Hardware', 'summary': 'Jetson 온도 확인과 수동 PWM 유지', 'file': 'fan_control.md'}, {'slug': 'cloudflared', 'title': 'Cloudflare Tunnel', 'category': 'Network', 'summary': 'bts.sarihub.com 터널과 localhost:8080 연결 구성', 'file': 'cloudflared.md'}]


class DashboardHandler(BaseHTTPRequestHandler):
    provider: TelemetryProvider
    protocol_version = "HTTP/1.1"

    def version_string(self):
        # A short Server header (NET-06): every answer carried "BaseHTTP/0.6 Python/3.x.y".
        return "ubts"
    # D-12 extension points (dashboard_server/extensions.py). create_server() sets them from the environment.
    dashboard_settings = DashboardSettings()
    extension_routes = RouteTable()
    page_filter = None          # changes the page text (simulation mode, vehicle kind); None = the page as it is
    # Relay of the row executor's operator API (dashboard_server/executor_proxy.py); None = not configured.
    executor_client = None
    # Change log of the tuning screen (dashboard_server/steer_tuning.py); create_server() sets it.
    tuning_log = None
    _last_telemetry_log_at = 0.0

    def do_HEAD(self):
        self.do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path.startswith('/api/live/') or path.startswith('/api/autonomy/'):
            self._live_request(parsed, action=True)
        elif path.startswith("/api/rover/"):
            self._rover_action(path)
        elif path.startswith("/api/executor/"):
            self._executor_request(parsed, action=True)
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
            result = self.provider.stop_recording()
            if result.get('ok') and result.get('recording_id'):
                try:self.archive_mapping.regenerate(str(result['recording_id']))
                except ValueError:pass
            self._send_json(result)
        elif path == "/api/imu/zero":
            self._send_json(self.provider.zero_imu(), status=410)
        elif path == "/api/link-log":
            self._link_log()
        elif path == "/api/network/mode":
            if self._network_request_allowed():
                payload = self._read_json_body()
                self._send_json(self.provider.set_network_mode(payload))
        elif path.startswith("/api/network/wifi/"):
            self._network_wifi_action(path.removeprefix("/api/network/wifi/"))
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
        elif not self._extension_request(parsed):
            self.send_error(404, "Not found")

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/api/ping":
            self._send_ping()
            return
        if path.startswith('/api/live/') or path.startswith('/api/autonomy/native-map/') or path in ('/api/autonomy','/api/autonomy/preflight','/api/autonomy/observe','/api/autonomy/supervisor'):
            self._live_request(parsed)
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
            self._send_static("index.html")
        elif path == "/api/health":
            self._send_json(self.provider.health())
        elif path == "/api/telemetry" and parse_qs(parsed.query).get("bundle") == ["drive"]:
            # NET-06: the drive screen's status only (sensors, services, rover).
            if isinstance(self.provider, TelemetryProvider):
                payload = self._shared_telemetry("drive").get()
            else:
                payload = self.provider.telemetry()
            self._send_json(payload)
        elif path == "/api/telemetry":
            payload = self._shared_telemetry().get() if isinstance(self.provider, TelemetryProvider) else self.provider.telemetry()
            payload["connection"]["lidar_websocket"] = lidar_websocket_stats()
            payload["connection"]["camera_websocket"] = camera_websocket_stats()
            payload["connection"]["imu_websocket"] = imu_websocket_stats()
            self._send_json(payload)
        elif path == "/api/system":
            self._send_json(self.provider.system())
        elif path == "/api/network":
            self._send_json(self.provider.network())
        elif path == "/api/network/wifi":
            wifi = getattr(self.provider, "wifi", None)
            if wifi is None:
                self._send_json({"ok": False, "enabled": False, "state": "disabled", "saved": [], "scan": None})
            else:
                self._send_json(wifi.overview(refresh_saved=parse_qs(parsed.query).get("refresh") == ["1"]))
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
            self._send_static(path.removeprefix("/static/"), parse_qs(parsed.query).get("v", [None])[0])
        elif path == "/api/dashboard/config":
            self._send_json({"ok": True, **self.dashboard_settings.public(), "executor_configured": self.executor_client is not None})
        elif path.startswith("/api/executor/"):
            self._executor_request(parsed)
        elif path.startswith("/ext-static/") and self.dashboard_settings.extra_static is not None:
            folder = self.dashboard_settings.extra_static
            self._send_file(folder / unquote(path.removeprefix("/ext-static/")), allowed_root=folder)
        elif not self._extension_request(parsed):
            self.send_error(404, "Not found")

    def do_DELETE(self):
        parsed = urlparse(self.path)
        if parsed.path.startswith("/occupancy-api"):
            self._proxy_occupancy_request(parsed)
        elif not self._extension_request(parsed):
            self.send_error(404, "Not found")

    def _executor_request(self, parsed, action=False):
        """/api/executor/*: the row executor's operator API on behalf of the page (docs/STEERING_SERVER_KO.md 6)."""
        from dashboard_server import executor_proxy as proxy
        from dashboard_server import steer_tuning
        client = self.executor_client
        name = parsed.path.removeprefix("/api/executor/")
        if not action:
            if name == "status":
                grid = parse_qs(parsed.query).get("grid") == ["1"]          # the map screen's local grid layer
                self._send_json(client.status(grid) if client is not None else proxy.not_configured_status())
            elif name == "tuning":
                self._send_json(steer_tuning.view(client.status() if client is not None else proxy.not_configured_status()))
            elif name == "tuning/log":
                log = self.tuning_log
                self._send_json({"ok": True, "entries": log.recent() if log is not None else []})
            else:
                self.send_error(404, "Not found")
            return
        if name not in proxy.ACTIONS and name != "params":
            self.send_error(404, "Not found")
            return
        origin = self.headers.get("Origin")
        if origin and urlparse(origin).netloc != self.headers.get("Host"):
            self.send_error(403, "Origin mismatch")
            return
        session = self._ensure_session(create=False)
        if not session:
            self.send_error(403, "Dashboard session required")
            return
        try:
            body = proxy.read_body(self)
            if client is None:
                raise proxy.ExecutorError(503, "executor_not_configured")
            if name == "params":
                if self.tuning_log is None:
                    raise proxy.ExecutorError(503, "tuning_log_not_configured")
                status, reply = steer_tuning.apply(client, body, self.tuning_log, session[:8])
            else:
                status, reply = client.action(name, body)
        except proxy.ExecutorError as exc:
            if name == "stop":
                dashboard_log("executor.stop_failed", error=exc.error)
            self._send_json(exc.payload(), exc.status)
            return
        self._send_json(reply, status)

    def _extension_request(self, parsed) -> bool:
        """An extra route of a simulation extension (D-12). Only paths the dashboard itself does not serve."""
        handler = self.extension_routes.find(self.command, parsed.path)
        if handler is None:
            return False
        handler(self, parsed)
        return True

    def _live_request(self, parsed, action=False):
        if action:
            origin=self.headers.get('Origin')
            if origin and urlparse(origin).netloc != self.headers.get('Host'):
                self.send_error(403,'Origin mismatch');return
            if not self._ensure_session(create=False):
                self.send_error(403,'Dashboard session required');return
            payload=self._read_json_body()
            if not isinstance(payload,dict):self.send_error(400,'Object required');return
        path=parsed.path
        try:
            if path=='/api/live/mapping-jobs' and not action:
                result=self.mapping_operations.status()
            elif path=='/api/live/mapping-jobs/stop' and action:
                result=self.mapping_operations.stop(payload.get('job_id'),payload.get('expected_epoch'))
            elif path=='/api/autonomy' and not action:
                live=self.live_mapping.task_status() if hasattr(self.live_mapping,'task_status') else {'state':'unavailable'}
                result=self.autonomy_settings.status(live)
                result={**result,'supervisor':self.autonomy_supervisor.status()}
            elif path=='/api/autonomy/preflight' and not action:
                from dashboard_server.field_preflight import report
                result=report(ROOT/'config/field_autonomy_preparation.json',self.live_mapping.task_status())
            elif path in ('/api/autonomy/native-map/status','/api/autonomy/native-map/tiles','/api/autonomy/native-map/release') and not action:
                native=getattr(self,'native_observation_client',None)
                if not getattr(self,'native_map_preview_enabled',False) or native is None:
                    result=dict(ok=True,available=False,status=dict(enabled=False,reason='native_map_default_OFF'),
                        runtime_authority=False,motor_command_sent=False,autonomy_output_enabled=False,
                        execution_eligible=False,physical_accuracy_verified=False,live_input_eligible=False)
                else:
                    query=parse_qs(parsed.query,keep_blank_values=True)
                    if set(query)-{'cursor','lease'} or any(len(v)!=1 for v in query.values()):
                        raise ValueError('native_map_get_query_required')
                    if path.endswith('/status'):
                        if query:raise ValueError('native_map_status_has_no_query')
                        result=native.map_status()
                    elif path.endswith('/release'):
                        if set(query)!={'lease'}:raise ValueError('native_map_lease_required')
                        result=native.map_release(query['lease'][0])
                    else:
                        if set(query)-{'cursor'}:raise ValueError('native_map_tiles_query_required')
                        result=native.map_tiles(query.get('cursor',[None])[0])
                    if (not isinstance(result,dict) or any(result.get(k) is not False for k in
                            ('runtime_authority','motor_command_sent','autonomy_output_enabled'))):
                        raise ValueError('native_map_observation_only_reply_required')
                    if len(json.dumps(result,allow_nan=False,separators=(',',':')).encode())>131072:
                        raise ValueError('native_map_page_capacity')
            elif path=='/api/autonomy/native-map/new' and action:
                native=getattr(self,'native_observation_client',None)
                if (not getattr(self,'native_map_preview_enabled',False) or native is None or
                        parsed.query or set(payload)!={'map_epoch'}):
                    raise ValueError('explicit_enabled_native_new_map_required')
                result=native.map_new(payload['map_epoch'])
                if (not isinstance(result,dict) or any(result.get(k) is not False for k in
                        ('runtime_authority','motor_command_sent','autonomy_output_enabled'))):
                    raise ValueError('native_map_observation_only_reply_required')
            elif path=='/api/autonomy/observe' and not action:
                result=self._autonomy_observe_status()
            elif path in ('/api/autonomy/observe/start','/api/autonomy/observe/stop') and action:
                if payload:raise ValueError('observation_actions_do_not_accept_sensor_or_control_inputs')
                self._autonomy_observe_action(path.rsplit('/',1)[-1])
                result=self._autonomy_observe_status()
            elif path=='/api/autonomy/speed' and action:
                with self.autonomy_observe_lock:
                    observer=getattr(self,'autonomy_observation',None)
                    observing=observer is not None and observer.status().get('active') is True
                    percent=payload.get('speed_percent')
                    if observing and (isinstance(percent,bool) or not isinstance(percent,(int,float)) or not 0<=percent<=30):
                        raise ValueError('active_observation_speed_requires_0_to_30_percent')
                    result=self.autonomy_settings.set_speed(percent)
                    if observing:observer.set_cruise_percent(percent)
            elif path=='/api/autonomy/supervisor' and not action:
                result=self.autonomy_supervisor.status()
            elif path in ('/api/autonomy/start','/api/autonomy/stop') and action:
                result=self._native_autonomy_action(path.rsplit('/',1)[-1],payload)
                if path.endswith('/start') and not result.get('active'):
                    self._send_json({**result,'ok':False},409);return
            elif path=='/api/live/map' and not action:
                query=parse_qs(parsed.query)
                if query.get('transport',[None])[0]=='tiles':
                    extra={}
                    if getattr(self.live_mapping,'supports_path_delta',False) and 'path_token' in query:
                        extra=dict(path_token=query['path_token'][0],path_count=query.get('path_count',[None])[0],
                            path_stamp=query.get('path_stamp',[None])[0])
                    result=self.live_mapping.status(query.get('since',[None])[0],transport='tiles',
                        tile_since=query.get('tile_since',[None])[0],tile_epoch=query.get('tile_epoch',[None])[0],**extra)
                else:result=self.live_mapping.status(query.get('since',[None])[0])
            elif path.startswith('/api/live/map/') and action:
                operation=path.rsplit('/',1)[-1]
                if hasattr(self.live_mapping,'task_status'):
                    result=self.live_mapping.action(operation,expected_epoch=payload.get('expected_epoch'))
                else:result=self.live_mapping.action(operation)
            elif path.startswith('/api/live/recordings/'):
                parts=path.split('/')
                if len(parts)!=6:raise ValueError('invalid_archive_route')
                archive_id=unquote(parts[4])
                if parts[5]=='map' and not action:
                    query=parse_qs(parsed.query)
                    if query.get('transport',[None])[0]=='tiles':
                        result=self.archive_mapping.status(archive_id,query.get('t_s',[None])[0],query.get('since',[None])[0],
                            transport='tiles',tile_since=query.get('tile_since',[None])[0],tile_epoch=query.get('tile_epoch',[None])[0])
                    else:result=self.archive_mapping.status(archive_id,query.get('t_s',[None])[0],query.get('since',[None])[0])
                    if query.get('codec',[None])[0]=='xor2-deflate-v1' and 'tiles' in result:
                        from dashboard_server.window_cells import compact_window
                        result=compact_window(dict(ok=True,map_frame=True,seed=result,events=[]),'xor2-deflate-v1')
                elif parts[5]=='map-window' and not action:
                    query=parse_qs(parsed.query)
                    result=self.archive_mapping.window(archive_id,query.get('start_s',['0'])[0],query.get('span_s',['12'])[0],query.get('codec',[None])[0])
                elif parts[5]=='new-map' and action:
                    if getattr(self.archive_mapping,'controlled',False):
                        result=self.archive_mapping.new_map(archive_id,expected_epoch=payload.get('expected_epoch'),client=payload.get('client'))
                    else:result=self.archive_mapping.new_map(archive_id)
                elif parts[5]=='map-playback' and action:
                    kwargs={'resume':payload.get('resume_mapping') is True,'generation':payload.get('mapping_generation')} if getattr(self.archive_mapping,'controlled',False) else {}
                    result=self.archive_mapping.playback(archive_id,payload.get('t_s'),payload.get('playing'),payload.get('control'),**kwargs)
                    # One round trip carries the ordered playback cursor and
                    # its historical cell changes. Pure reads/older clients
                    # retain the independent map endpoint and full snapshots.
                    if (payload.get('transport')=='tiles' and result.get('playback_driven')
                            and not result.get('stale_control_ignored')):
                        snapshot=self.archive_mapping.status(archive_id,payload.get('t_s'),transport='tiles',
                            tile_since=payload.get('tile_since'),tile_epoch=payload.get('tile_epoch'))
                        if payload.get('codec')=='xor2-deflate-v1' and 'tiles' in snapshot:
                            from dashboard_server.window_cells import compact_window
                            snapshot=compact_window(dict(ok=True,map_frame=True,seed=snapshot,events=[]),'xor2-deflate-v1')
                        result={**result,'map_snapshot':snapshot}
                elif parts[5]=='regenerate' and action:result=self.archive_mapping.regenerate(archive_id)
                else:self.send_error(404);return
            else:self.send_error(404);return
            self._send_json(result)
        except ValueError as error:
            from dashboard_server import mapping_operations as operations
            MappingConflict=operations.MappingConflict
            deferred=getattr(operations,'MappingDeferred',None)
            if isinstance(deferred,type) and isinstance(error,deferred):
                # A deferred read (autonomy priority, busy window slot) is a wait, not an error.
                self._send_json({'ok':False,'error':str(error),'deferred':True,'retry_after_s':error.retry_after_s},503,
                                headers={'Retry-After':str(error.retry_after_s)});return
            self._send_json({'ok':False,'error':str(error)},409 if isinstance(error,MappingConflict) or str(error)=='mapping_epoch_changed_refresh_before_action' else 400)

    def _native_autonomy_action(self,action,payload):
        # Actual arming remains local and requires the existing authenticated session.
        origin=self.headers.get('Origin')
        if (not origin or urlparse(origin).netloc!=self.headers.get('Host') or
                self.client_address[0] not in ('127.0.0.1','::1')):
            raise ValueError('explicit_local_same_origin_operator_required')
        session=self._ensure_session(create=False)
        if not session:raise ValueError('authenticated_local_operator_required')
        if action=='stop':
            if payload:raise ValueError('autonomy_stop_accepts_no_control_or_sensor_inputs')
            return self.autonomy_supervisor.stop()
        if set(payload)!={'operator_arm','deadman'} or payload.get('operator_arm') is not True or payload.get('deadman') is not True:
            raise ValueError('explicit_operator_arm_and_deadman_required')
        return self.autonomy_supervisor.start(session,explicit=True,cruise_percent=self.autonomy_settings.percent)

    def _autonomy_observe_status(self):
        result=self.autonomy_observation.status()
        if (not isinstance(result,dict) or result.get('runtime_authority') is not False
                or result.get('motor_command_sent') is not False
                or result.get('autonomy_output_enabled') is not False):
            raise ValueError('observation_only_status_required')
        # The existing percentage is a preference. No measured m/s conversion,
        # motor sender, lease or mapping action is introduced by these routes.
        native=getattr(self,'native_observation_client',None)
        return {**result,'speed_percent':self.autonomy_settings.percent,
            'native_source':native.status() if native is not None else {
                'enabled':False,'reason':'legacy_web_ws_provenance_unverified'},
            'speed_units':'percent_of_configured_cruise','calibrated_speed_m_s':None}

    def _autonomy_observe_action(self,action):
        with self.autonomy_observe_lock:
            native=self.native_observation_client
            if action=='start':
                percent=self.autonomy_settings.percent
                if isinstance(percent,bool) or not isinstance(percent,(int,float)) or not 0<=percent<=30:
                    raise ValueError('observation_requires_existing_speed_preference_0_to_30_percent')
                if native is not None:
                    started=native.start()
                    if not isinstance(started,dict) or started.get('ok') is not True:
                        raise ValueError('native_observation_start_failed:'+str(started.get('error','unknown') if isinstance(started,dict) else 'invalid_response'))
                try:
                    observed=self.autonomy_observation.start(cruise_percent=percent)
                    if not isinstance(observed,dict) or observed.get('active') is not True or observed.get('first_failure_reason'):
                        raise ValueError('autonomy_observation_start_failed:'+str(observed.get('first_failure_reason') or observed.get('state','unknown') if isinstance(observed,dict) else 'invalid_response'))
                except Exception:
                    if native is not None:native.stop()
                    raise
            else:
                try:self.autonomy_observation.stop()
                finally:
                    if native is not None:
                        stopped=native.stop()
                        if not isinstance(stopped,dict) or stopped.get('ok') is not True:
                            raise ValueError('native_observation_stop_failed:'+str(stopped.get('error','unknown') if isinstance(stopped,dict) else 'invalid_response'))

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
        supervisor=getattr(self,'autonomy_supervisor',None)
        if supervisor is not None and (supervisor.status().get('active') or supervisor.status().get('loop_cleanup_pending')):
            handoff=supervisor.manual_takeover()
            if path!='/api/rover/stop' and not handoff.get('loop_joined'):
                self._send_json({'ok':False,'error':'autonomy_manual_handoff_cleanup_pending'},409);return
        self._send_json(request({**payload, "op": operations[path], "owner": session}))

    def log_message(self, fmt, *args):
        if os.environ.get("DASHBOARD_ACCESS_LOG", "0") == "1":
            super().log_message(fmt, *args)

    # Persistent connections (NET-05). JSON and static answers keep the TCP
    # connection open, so a control command on a slow link no longer pays one
    # extra round trip for a new connection. Only the wait for the next request
    # line has a time limit; once a request has started the socket is blocking
    # as before (WebSocket relays and large bodies are unchanged).
    KEEP_ALIVE_IDLE_S = 30.0

    def handle_one_request(self):
        try:
            self.connection.settimeout(self.KEEP_ALIVE_IDLE_S)
        except OSError:
            pass
        self._body_consumed = False
        self._session_created = None
        super().handle_one_request()

    def parse_request(self):
        # Start of the server-side time reported in Server-Timing (E0).
        self._request_started = time.perf_counter()
        try:
            self.connection.settimeout(None)
        except OSError:
            pass
        return super().parse_request()

    def _keep_alive(self) -> bool:
        """True when this answer may leave the connection open for the next request.

        A request body that was not read would be taken for the next request line,
        so such a connection is closed (as every answer did before)."""
        if self.close_connection:
            return False
        if "chunked" in self.headers.get("Transfer-Encoding", "").lower():
            return False
        try:
            length = int(self.headers.get("Content-Length", "0") or 0)
        except ValueError:
            return False
        return length <= 0 or bool(getattr(self, "_body_consumed", False))

    def _send_connection_header(self):
        self.send_header("Connection", "keep-alive" if self._keep_alive() else "close")

    def _server_timing_header(self):
        started = getattr(self, "_request_started", None)
        if started is not None:
            self.send_header("Server-Timing", f"app;dur={(time.perf_counter() - started) * 1000:.1f}")

    def _shared_telemetry(self, bundle: str = "full"):
        from dashboard_server.telemetry_snapshot import SharedTelemetry, DriveTelemetry
        attribute = "_telemetry_snapshot" if bundle == "full" else "_telemetry_snapshot_drive"
        shared = getattr(DashboardHandler, attribute, None)
        if shared is None or getattr(shared, "owner", shared.provider) is not self.provider:
            source = self.provider if bundle == "full" else DriveTelemetry(self.provider)
            shared = SharedTelemetry(source, ttl_s=float(os.environ.get("TELEMETRY_SNAPSHOT_TTL_S", "1.0")))
            shared.owner = self.provider
            setattr(DashboardHandler, attribute, shared)
        return shared

    # Conditional GET for JSON (NET-06). A polled status answer that has not
    # changed is answered "304 Not Modified" without a body when the page names
    # the ETag of the copy it holds. The body is hashed after it is built, so the
    # answer is the same JSON either way; large answers are not hashed.
    JSON_ETAG_MAX_BYTES = 256 * 1024

    def _send_json(self, payload, status=200, headers=None):
        session_token = self._ensure_session(create=True)
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        etag = None
        if status == 200 and self.command in ("GET", "HEAD") and len(body) <= self.JSON_ETAG_MAX_BYTES:
            etag = '"j' + hashlib.sha1(body).hexdigest()[:20] + '"'
            if self.headers.get("If-None-Match") == etag:
                self.send_response(304)
                self._server_timing_header()
                self.send_header("ETag", etag)
                self.send_header("Cache-Control", "no-store")
                self._send_connection_header()
                self._send_session_cookie(session_token)
                self.end_headers()
                return
        accepts_gzip = "gzip" in self.headers.get("Accept-Encoding", "").lower()
        encoded = gzip.compress(body, compresslevel=1) if accepts_gzip and len(body) >= 1024 else body
        self.send_response(status)
        self._server_timing_header()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self._send_connection_header()
        self.send_header("Vary", "Accept-Encoding")
        if encoded is not body:
            self.send_header("Content-Encoding", "gzip")
        if etag:
            self.send_header("ETag", etag)
        for key, value in (headers or {}).items():
            self.send_header(key, value)
        self._send_session_cookie(session_token)
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        if self.command != "HEAD":
            try:
                self.wfile.write(encoded)
            except (BrokenPipeError, ConnectionResetError, OSError):
                self.close_connection = True
                return
        self._log_served_json()

    def _send_ping(self):
        """Link probe of the page (NET-10): 204 without a body, the smallest answer
        there is. The page measures the round trip; Server-Timing gives the
        server's own share so it can be taken off."""
        self.send_response(204)
        self._server_timing_header()
        self.send_header("Cache-Control", "no-store")
        self._send_connection_header()
        self.end_headers()

    LINK_LOG_MAX_BODY = 256 * 1024
    LINK_LOG_MAX_FILE = 20 * 1024 * 1024

    def _link_log(self):
        """Optional field log of the page's link samples (NET-10 (5)).

        The page sends round trips and grades in batches while the log is turned
        on; they are kept as one JSONL file a day in runtime/link_log/ (20 MB a
        day at most). Only numbers and short words are kept."""
        origin = self.headers.get("Origin")
        if origin and urlparse(origin).netloc != self.headers.get("Host"):
            self.send_error(403, "Origin mismatch")
            return
        if not self._ensure_session(create=False):
            self.send_error(403, "Dashboard session required")
            return
        try:
            length = int(self.headers.get("Content-Length", "0") or 0)
        except ValueError:
            length = -1
        if length <= 0 or length > self.LINK_LOG_MAX_BODY:
            self.send_error(413 if length > 0 else 400, "Link log batch size")
            return
        payload = self._read_json_body()
        samples = payload.get("samples") if isinstance(payload, dict) else None
        if not isinstance(samples, list) or len(samples) > 2000:
            self._send_json({"ok": False, "error": "samples_required"}, 400)
            return
        recording = payload.get("recording_id")
        recording = recording if isinstance(recording, str) and len(recording) <= 80 else None
        profile = payload.get("profile") if payload.get("profile") in ("auto", "tiny", "slow", "balanced", "detail") else None
        words = {"ping", "control", "good", "normal", "slow", "very_slow", "offline", "unknown"}
        rows = []
        for sample in samples:
            if not isinstance(sample, dict):
                continue
            t, rtt = sample.get("t"), sample.get("rtt_ms")
            rows.append({
                "t": t if isinstance(t, (int, float)) and not isinstance(t, bool) else None,
                "kind": sample.get("kind") if sample.get("kind") in words else None,
                "ok": sample.get("ok") is True,
                "rtt_ms": rtt if isinstance(rtt, (int, float)) and not isinstance(rtt, bool) and 0 <= rtt < 600000 else None,
                "grade": sample.get("grade") if sample.get("grade") in words else None,
                "recording_id": recording,
                "profile": profile,
            })
        folder = Path(os.environ.get("DASHBOARD_LINK_LOG_DIR", str(ROOT / "runtime" / "link_log")))
        written = 0
        try:
            folder.mkdir(parents=True, exist_ok=True)
            target = folder / time.strftime("%Y%m%d.jsonl", time.gmtime())
            size = target.stat().st_size if target.exists() else 0
            data = "".join(json.dumps(row, separators=(",", ":")) + "\n" for row in rows).encode("utf-8")
            if size + len(data) <= self.LINK_LOG_MAX_FILE:
                with open(target, "ab") as stream:
                    stream.write(data)
                written = len(rows)
        except OSError as exc:
            self._send_json({"ok": False, "error": f"link_log_write_failed: {exc.strerror or exc}"}, 500)
            return
        self._send_json({"ok": True, "written": written, "dropped": len(rows) - written})

    # ---- Network page: AP switch and Wi-Fi management (dashboard_server/wifi_manager.py) ----
    NETWORK_MAX_BODY = 4096

    def _network_request_allowed(self) -> bool:
        """Changes of the Jetson's network need this page's own origin and a dashboard session."""
        origin = self.headers.get("Origin")
        if origin and urlparse(origin).netloc != self.headers.get("Host"):
            self.send_error(403, "Origin mismatch")
            return False
        if not self._ensure_session(create=False):
            self.send_error(403, "Dashboard session required")
            return False
        try:
            length = int(self.headers.get("Content-Length", "0") or 0)
        except ValueError:
            length = -1
        if length < 0 or length > self.NETWORK_MAX_BODY:
            self.send_error(413, "Network request size")
            return False
        return True

    def _network_wifi_action(self, name: str):
        from dashboard_server.wifi_manager import http_status
        actions = {"scan": "scan", "connect": "connect", "confirm": "confirm", "revert": "revert", "profile": "update_profile"}
        wifi = getattr(self.provider, "wifi", None)
        if name not in actions:
            self.send_error(404, "Not found")
            return
        if not self._network_request_allowed():
            return
        payload = self._read_json_body()
        if wifi is None:
            self._send_json({"ok": False, "state": "disabled"}, 503)
            return
        result = wifi.scan() if name == "scan" else getattr(wifi, actions[name])(payload)
        payload = None  # the request may hold a password; it is not kept or logged
        self._send_json(result, http_status(result))

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
        raw = self.rfile.read(length)
        self._body_consumed = len(raw) == length
        try:
            return json.loads(raw.decode("utf-8"))
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

    def _send_static(self, rel: str, requested_version: str | None = None):
        """A file below static/ (or the page) with a content-hash ETag. A request naming the current version
        (?v=...) is cached for a year; anything else is checked again next time (see static_assets.py)."""
        session_token = self._ensure_session(create=True)
        accept_gzip = "gzip" in self.headers.get("Accept-Encoding", "").lower()
        # D-12: read through the class (a function stored on the class would bind to the instance); no work when it is
        # the filter already in use.
        STATIC_ASSETS.set_page_filter(type(self).page_filter)
        started = time.monotonic()
        try:
            served = STATIC_ASSETS.serve(rel, requested_version, accept_gzip)
        except OSError:
            served = None
        serve_ms = (time.monotonic() - started) * 1000
        if served is None:
            self.send_error(404, "Not found")
            return
        if self.headers.get("If-None-Match") == served.etag:
            self.send_response(304)
            self.send_header("ETag", served.etag)
            self.send_header("Cache-Control", served.cache_control)
            self.send_header("Vary", "Accept-Encoding")
            self._send_session_cookie(session_token)
            self.end_headers()
            return
        content_type = mimetypes.guess_type(str(served.path))[0] or "application/octet-stream"
        suffix = served.path.suffix.lower()
        if suffix in (".js", ".mjs"):
            content_type = "application/javascript; charset=utf-8"
        elif suffix == ".css":
            content_type = "text/css; charset=utf-8"
        elif suffix == ".html":
            content_type = "text/html; charset=utf-8"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", served.cache_control)
        self.send_header("ETag", served.etag)
        self.send_header("Vary", "Accept-Encoding")
        if served.gzip:
            self.send_header("Content-Encoding", "gzip")
        self._send_connection_header()
        self._send_session_cookie(session_token)
        self.send_header("Content-Length", str(len(served.body)))
        self.end_headers()
        if self.command != "HEAD":
            try:
                self.wfile.write(served.body)
            except (BrokenPipeError, ConnectionResetError, OSError):
                self.close_connection = True
        took_ms = (time.monotonic() - started) * 1000
        if took_ms > 500:
            dashboard_log("static.slow", path=rel, ms=round(took_ms), serve_ms=round(serve_ms))

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
                # NET-11: a session in use slides. Past half of its lifetime it is
                # extended and the cookie sent again, so an open page (and its
                # sensor streams) is not cut off when the first lifetime ends.
                item = _SESSIONS[token]
                if item.get("expires_at", 0.0) - now < SESSION_TTL_S / 2:
                    item["expires_at"] = now + SESSION_TTL_S
                    self._session_created = token
                return token
            if not create:
                return None
            token = secrets.token_urlsafe(24)
            _SESSIONS[token] = {"created_at": now, "expires_at": now + SESSION_TTL_S}
        self._session_created = token
        dashboard_log("session.created", session=token[:8], remote=self.client_address[0])
        return token

    def _send_session_cookie(self, token: str | None):
        # The cookie is sent when the session is created (NET-06). The session
        # does not slide, so sending it again on every answer only added about
        # 100 bytes to each one.
        if not token or getattr(self, "_session_created", None) != token:
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
        # High-rate polls of the steering screens are not logged one by one.
        if path not in {"/api/health", "/api/rover/status", "/api/executor/status", "/api/executor/heartbeat"}:
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
        self._server_timing_header()
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

        def current_deadline(token=session_token, fallback=session_deadline):
            # The session's expiry as it is now (it slides while the page uses it).
            with _SESSION_LOCK:
                item = _SESSIONS.get(token)
                return float(item.get("expires_at", fallback)) if item else fallback

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
            while time.time() < current_deadline():
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
                    # Wait for the next try, but notice a browser that leaves meanwhile (close frame or EOF):
                    # otherwise the socket stays CLOSING until the session deadline.
                    if self._ws_client_left(min(2.0, max(0.1, current_deadline() - time.time()))):
                        self._ws_send_close(self.connection)
                        return
                    continue
                try:
                    self._relay_websocket(upstream, initial_upstream, current_deadline)
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

    def _relay_websocket(self, upstream: socket.socket, initial_upstream: bytes = b"", deadline=None):
        # Sensor previews only. A slow reader must not block reverse ACKs or
        # retain this registry entry/upstream after the write/session deadline.
        # Opaque TCP chunks do not establish WS frame boundaries. Injecting an
        # idle JSON frame could corrupt a delayed JPEG/scan payload; source
        # bridge status and the existing client sensor-age display remain.
        result = relay_sensor_stream(self.connection, upstream, initial_upstream,
                                     session_deadline=deadline)
        dashboard_log("ws.relay.end", reason=result.reason,
                      to_client_bytes=result.to_client_bytes,
                      to_upstream_bytes=result.to_upstream_bytes,
                      peak_to_client_bytes=result.peak_to_client_bytes,
                      peak_to_upstream_bytes=result.peak_to_upstream_bytes)

    def _ws_send_json(self, sock: socket.socket, payload: dict) -> bool:
        return self._ws_send_frame(sock, 0x1, json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8"))

    def _ws_client_left(self, wait_s: float) -> bool:
        """Wait up to wait_s for the browser while no bridge is connected; True when it sent a close frame or closed.

        Whole frames are read (header, extended length, mask, payload) so a later relay starts on a frame boundary;
        other frames from the browser during the wait are dropped (there is no bridge to give them to)."""
        sock = self.connection
        end = time.monotonic() + max(0.0, wait_s)
        while True:
            left = end - time.monotonic()
            if left <= 0:
                return False
            try:
                readable, _, _ = select.select([sock], [], [], left)
            except (OSError, ValueError):
                return True
            if not readable:
                return False
            frame = self._ws_read_client_frame(sock)
            if frame is None or frame == 0x8:
                return True

    @staticmethod
    def _ws_read_client_frame(sock: socket.socket, limit: int = 1 << 20):
        """Read one client frame and return its opcode; None on EOF, error or an oversized frame."""
        def exact(n):
            data = b""
            while len(data) < n:
                try:
                    chunk = sock.recv(n - len(data))
                except OSError:
                    return None
                if not chunk:
                    return None
                data += chunk
            return data
        previous = sock.gettimeout()
        sock.settimeout(2.0)
        try:
            head = exact(2)
            if head is None:
                return None
            opcode, length = head[0] & 0x0F, head[1] & 0x7F
            if length == 126:
                ext = exact(2)
                length = None if ext is None else int.from_bytes(ext, "big")
            elif length == 127:
                ext = exact(8)
                length = None if ext is None else int.from_bytes(ext, "big")
            if length is None or length > limit:
                return None
            if head[1] & 0x80 and exact(4) is None:
                return None
            if length and exact(length) is None:
                return None
            return opcode
        finally:
            try:
                sock.settimeout(previous)
            except OSError:
                pass

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
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, server_address, RequestHandlerClass, bind_and_activate=True, *,
                 autonomy_supervisor=None, autonomy_observation=None,
                 native_observation_client=None, autonomy_observe_lock=None):
        # TCPServer.__init__ may call our server_close() if bind/activate fails.
        # Establish instance ownership BEFORE that call, including direct
        # DashboardServer users which intentionally have no observation worker.
        self.autonomy_supervisor = autonomy_supervisor
        self.autonomy_observation = autonomy_observation
        self.native_observation_client = native_observation_client
        self.autonomy_observe_lock = (threading.RLock() if autonomy_observe_lock is None
                                      else autonomy_observe_lock)
        super().__init__(server_address, RequestHandlerClass, bind_and_activate)

    def server_close(self):
        supervisor=getattr(self,'autonomy_supervisor',None)
        if supervisor is not None:
            closed=supervisor.close()
            if not closed.get('loop_joined'):
                raise RuntimeError('autonomy_supervisor_cleanup_unproven')
        try:
            with self.autonomy_observe_lock:
                try:
                    observer=getattr(self,'autonomy_observation',None)
                    if observer is not None:observer.stop(reason='dashboard_server_closed')
                finally:
                    native=getattr(self,'native_observation_client',None)
                    if native is not None:native.stop()
        finally:
            super().server_close()


def create_server(host: str, port: int, config: str | None = None, *, autonomy_bindings=None) -> ThreadingHTTPServer:
    from dashboard_server import extensions
    from dashboard_server.executor_proxy import ExecutorClient
    from dashboard_server.steer_tuning import TuningLog
    settings = extensions.DashboardSettings.from_env(os.environ)
    if settings.ignored:
        dashboard_log("settings.ignored_in_live_mode", names=list(settings.ignored))
    extension = extensions.load_extension(settings)
    DashboardHandler.dashboard_settings = settings
    DashboardHandler.extension_routes = extensions.routes_of(extension)
    DashboardHandler.page_filter = extensions.page_filter(settings, extension)
    DashboardHandler.provider = extensions.load_provider(settings, config) or TelemetryProvider(config)
    DashboardHandler.executor_client = ExecutorClient.from_env(os.environ)
    DashboardHandler.tuning_log = TuningLog(os.environ.get("UBTS_EXECUTOR_TUNING_LOG") or ROOT / "runtime/executor_tuning_log.jsonl")
    from dashboard_server.source_live_mapping import SourceLiveMapping as LiveMapping
    from dashboard_server.mapping_operations import ManagedArchiveMapping,MappingOperations
    from dashboard_server.autonomy_settings import AutonomySettings
    from dashboard_server.autonomy_observation import AutonomyObservation
    from dashboard_server.record_based_autonomy import RecordBasedAutonomy
    DashboardHandler.live_mapping=LiveMapping(ROOT/'runtime/live-map',start_worker=os.environ.get('DASHBOARD_LIVE_MAP_WORKER','1')=='1')
    DashboardHandler.archive_mapping=ManagedArchiveMapping(DashboardHandler.provider.recording_archive_dir,DashboardHandler.live_mapping)
    DashboardHandler.mapping_operations=MappingOperations(DashboardHandler.live_mapping,DashboardHandler.archive_mapping)
    DashboardHandler.autonomy_settings=AutonomySettings(ROOT/'runtime/autonomy_settings.json')
    native_path=os.environ.get('UBTS_NATIVE_STREAM_SOCKET')
    DashboardHandler.native_observation_client=None
    DashboardHandler.native_map_preview_enabled=os.environ.get('UBTS_NATIVE_MAP_PREVIEW','0')=='1'
    if native_path:
        from dashboard_server.native_observation_client import NativeObservationClient
        DashboardHandler.native_observation_client=NativeObservationClient(native_path)
    from dashboard_server.native_autonomy_application import NativeAutonomyApplication
    def native_session_valid(token):
        with _SESSION_LOCK:
            _prune_sessions();return token in _SESSIONS
    DashboardHandler.autonomy_supervisor=NativeAutonomyApplication(
        DashboardHandler.native_observation_client,session_valid=native_session_valid,bindings=autonomy_bindings)
    DashboardHandler.autonomy_observe_lock=threading.RLock()
    from dashboard_server.field10_shadow import make_field10_shadow_planner
    # Enabled only by a release that pins BOTH roles to the scalar protocol.
    # An old/unsupported server returns None; it is never a full-getter fallback.
    comparison_protocol=os.environ.get('UBTS_NATIVE_CONTROL_COMPARISON_V1','0')
    if comparison_protocol not in ('0','1'):
        raise ValueError('explicit_native_comparison_protocol_required')
    if comparison_protocol=='1' and DashboardHandler.native_observation_client is None:
        raise ValueError('native_comparison_protocol_requires_native_source')
    DashboardHandler.autonomy_observation=AutonomyObservation(
        planner=make_field10_shadow_planner(),input_provider=DashboardHandler.native_observation_client.input
            if DashboardHandler.native_observation_client is not None else DashboardHandler.live_mapping.observation_input,
        comparison_provider=DashboardHandler.native_observation_client.control_comparison
            if comparison_protocol=='1' else None)
    dashboard_log("server.create", host=host, port=port, session_ttl_s=SESSION_TTL_S)
    server=DashboardServer((host, port), DashboardHandler,
        autonomy_supervisor=DashboardHandler.autonomy_supervisor,
        autonomy_observation=DashboardHandler.autonomy_observation,
        native_observation_client=DashboardHandler.native_observation_client,
        autonomy_observe_lock=DashboardHandler.autonomy_observe_lock)
    server.autonomy_supervisor=DashboardHandler.autonomy_supervisor
    server.autonomy_observation=DashboardHandler.autonomy_observation
    server.native_observation_client=DashboardHandler.native_observation_client
    server.autonomy_observe_lock=DashboardHandler.autonomy_observe_lock
    return server


def main():
    parser = argparse.ArgumentParser(description="UNIST orchard rover dashboard server")
    parser.add_argument("--host", default=os.environ.get("DASHBOARD_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("DASHBOARD_PORT", "8080")))
    parser.add_argument("--config", default=os.environ.get("DASHBOARD_CONFIG"))
    args = parser.parse_args()
    server = create_server(args.host, args.port, args.config)
    # Prepare the page's versioned files now rather than on the first visitor's request (static_assets.warm).
    def warm_static():
        started = time.monotonic()
        # The first mimetypes lookup builds its table; on Windows it reads the registry (0.6-0.8 s on the bench PC,
        # seconds when the PC is busy). Do it here, not in the first visitor's page answer.
        mimetypes.guess_type("index.html")
        mime_ms = round((time.monotonic() - started) * 1000)
        try:
            version = STATIC_ASSETS.warm(DashboardHandler.page_filter)
        except Exception as exc:  # the first request does the same work
            dashboard_log("static.warm_failed", error=type(exc).__name__)
            return
        dashboard_log("static.warm", ms=round((time.monotonic() - started) * 1000), mime_ms=mime_ms, page_version=version)
    threading.Thread(target=warm_static, name="static-assets-warm", daemon=True).start()
    dashboard_log("server.listen", url=f"http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        DashboardHandler.live_mapping.close()
        DashboardHandler.archive_mapping.close()
        server.server_close()


if __name__ == "__main__":
    main()
