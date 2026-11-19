"""Relay of the row executor's operator API for the dashboard (docs/STEERING_SERVER_KO.md 6 and 7, contract v0).

The dashboard calls the executor's operator HTTP on behalf of the page, so the page never sees the executor token:

  UBTS_EXECUTOR_URL         http://host:port of the executor operator page (tools/row_executor/operator_http.py)
  UBTS_EXECUTOR_TOKEN_FILE  the token file the executor writes at every start (operator_token.txt)

Without UBTS_EXECUTOR_URL the dashboard reads the link file of the Jetson executor kit (deploy/jetson_row_executor):
  UBTS_EXECUTOR_LINK_FILE   default <dashboard>/runtime/row_executor/dashboard_link.json, written by the kit's installer
                            {"schema": "ubts.row_executor.dashboard_link.v1", "url": "http://127.0.0.1:<port>",
                             "token_file": "<the same folder>/operator_token.txt"}
                            read again when it changes (no container restart); only a loopback URL and a token file in
                            the folder of the link file are taken
With neither the relay is "not configured": the status says so and every action answers 503. The
executor checks Host, Origin and the X-Row-Token header and takes JSON bodies of at most 4 KiB; this module sends
exactly that, with only the fields of the contract. The status is reduced to the fields the screens show (no GNSS
coordinates). Only the map screen's ?grid=1 read carries the run's own odometry geometry (local grid, estimator band,
the recent path of this run: run-local metres from the start pose, never GNSS). Every read carries "pose", the
vehicle's place in that odometry (x m, y m, yaw rad; three numbers, 2026-11-20): the moving picture of the monitoring
view glides between the autonomy panel's two reads a second instead of asking for the geometry more often. Standard
library only.

GPS per run (2026-11-19, user request 'gps를 자율주행에 쓸지 안쓸지를 이제 결정해서 자율주행을 굴릴 수 있으면'): POST
/api/executor/start may carry "gnss_mode" (off | log_only | geofence | weak_constraint); null or absent = the executor's
configured mode (the field is then not sent, as before). Any other value: 400 invalid_gnss_mode. The executor decides
(weak_constraint needs RTK FIXED) and answers its own refusal, which the page shows.
"""
from __future__ import annotations

import http.client
import json
import math
import os
import re
import threading
import time
from urllib.parse import urlsplit

MAX_BODY = 4096
SESSION = re.compile(r"[A-Za-z0-9_-]{4,64}")      # contract v0 revision 2: 4 to 64 characters
TIMEOUT_S = {"heartbeat": 0.8, "stop": 2.5}       # placeholders (STEERING_SERVER_KO.md 6); others 2.0 s
DEFAULT_TIMEOUT_S = 2.0
ACTIONS = ("start", "arm", "go", "heartbeat", "stop", "ack", "end")
GNSS_MODES = ("off", "log_only", "geofence", "weak_constraint")    # tools/row_executor/config.py gnss_mode

# GET /api/status fields passed to the page (STEERING_SERVER_KO.md 6). None = the whole value (a scalar or a list of
# strings), a tuple = only these keys of the object. "tuning" and "local_grid_map" have their own readers below.
STATUS_FIELDS = {
    "run_state": None, "start_mode": None, "stop_reason": None, "stop_detail": None, "go_ready": None,
    "go_blockers": None, "mode": None, "state": None, "reason": None, "end_reason": None, "controller_mode": None,
    "profile": None, "vehicle_control": None, "lidar_age": None, "imu_age": None,
    # the tick record of a front_steer executor: run id, throttle T (%), steering target S (mrad), corridor width
    "run": None, "T": None, "S": None, "width": None, "corridor": None,
    "steer": ("meas", "age", "sat"),
    "operator_link": ("fresh", "age_s", "timeout_s"),
    "speed": ("target_mps", "limit_mps", "limit_reason"),
    "single_row": ("phase", "lateral_error_m", "heading_error_deg", "distance_m", "speed_m_s", "target_speed_m_s",
                   "stop_reason", "stop_category", "confidence", "throttle_pct", "steer_mrad", "end_remaining_m",
                   "lookahead_error_deg", "lateral_reference_m"),
    # the GNSS layer of the run (gnss_layer.py summary; 2026-11-19 per-run mode): no coordinates (rel_m stays here)
    "gnss": ("mode", "configured_mode", "requested_mode", "state", "fix", "fix_type", "present", "outcome", "age_s",
             "h_acc_m", "hacc_m", "used", "accepted", "rejected", "aligned", "fence_tripped", "fence_inside", "reason",
             "counts", "rtk"),
    "bridge_detail": ("state", "latch_reason"),
    # contract revision 3: the bridge took the lease back before the first go (the run stays open, run_state ready)
    "lease_lost": ("where", "error", "bridge_reason", "bridge_latches", "steering_fault"),
    # contract revision 6 (PW-10): the online scale LiDAR speed / table speed of the run and the offer after it ended
    "online_scale": ("enabled", "run_id", "value", "window_s", "target_s", "min_s", "frozen", "hold", "age_s", "mismatch",
                     "speed_scale", "prior_scale", "offer"),
}
NESTED = {("single_row", "confidence"): ("level",),
          ("gnss", "counts"): ("accepted", "restarted", "reanchored", "invalid", "order", "no_fix", "quality", "jump"),
          ("gnss", "rtk"): ("state", "fix_type", "fix_age_s", "fix_hacc_m", "correction_m", "fix_reason"),
          ("online_scale", "offer"): ("run_id", "value", "window_s", "end_reason", "speed_scale_now", "speed_scale_new",
                                      "ok", "why_not")}
# tools/row_executor/field_tuning.py State.status() (OPERATOR_API_KO.md 6). The file path and digests stay here.
TUNING_SCALARS = ("preset", "revision", "editable", "not_editable_because", "modified", "writable", "unsaved")
TUNING_GROUPS = ("basic", "advanced", "locked")
MAX_TUNING_KEYS = 96
# tools/row_executor/local_grid.py export(), schema ubts.row_executor.local_grid.v1 (LOCAL_GRID_KO.md): run odometry
# frame, no GNSS. Bitmap text at most 24 KiB (the executor keeps it under 16 KiB), at most 2,000 cells.
GRID_SCHEMA = "ubts.row_executor.local_grid.v1"
GRID_SCALARS = ("schema", "frame", "updates", "axis_rad", "occupied", "encoding", "age_s", "cell_m", "truncated")
MAX_GRID_BITS = 24576
MAX_GRID_CELLS = 2000
# status.metrics (2026-11-19): the run's reference centreline against the travelled path (lat_rms_m, within_0_10_pct,
# head_rms_deg, grid_* ...). The names may still change, so every short snake_case key with a number, a bool or a short
# text is passed (aggregates only, no coordinates).
METRIC_KEY = re.compile(r"[a-z][a-z0-9_]{0,47}")
MAX_METRICS = 64
# status.single_row_geometry (estimator band and the recent path, run odometry frame) on the ?grid=1 read only.
MAX_PATH_POINTS = 200


def _scalar(value):
    if value is None or isinstance(value, (bool, str)):
        return value if not isinstance(value, str) else value[:200]
    if isinstance(value, (int, float)):
        return value if math.isfinite(value) else None
    if isinstance(value, list):
        return [item[:200] for item in value[:32] if isinstance(item, str)]
    return None


def _number(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


def _numbers(value, count):
    """A list of exactly `count` finite numbers, or None."""
    if not isinstance(value, list) or len(value) != count:
        return None
    out = [_number(item) for item in value]
    return None if any(item is None for item in out) else out


def _tuning_value(value):
    """A tuning value: a number, a string, or a table of [number, number] rows (speed_table)."""
    if isinstance(value, list):
        rows = [_numbers(row, 2) for row in value[:32]]
        return [row for row in rows if row is not None]
    return _scalar(value)


def page_tuning(raw) -> dict:
    """status.tuning without the file path and digests: names, values, ranges and units come from the executor."""
    out = {key: _scalar(raw[key]) for key in TUNING_SCALARS if key in raw}
    if isinstance(raw.get("presets"), list):
        out["presets"] = [name[:64] for name in raw["presets"][:32] if isinstance(name, str)]
    if isinstance(raw.get("basic"), list):
        rows = []
        for item in raw["basic"][:MAX_TUNING_KEYS]:
            if isinstance(item, dict) and isinstance(item.get("name"), str):
                unit = item.get("unit")
                rows.append({"name": item["name"][:64], "value": _tuning_value(item.get("value")),
                             "low": _number(item.get("low")), "high": _number(item.get("high")),
                             "unit": unit[:16] if isinstance(unit, str) else ""})
        out["basic"] = rows
    values = raw.get("values")
    if isinstance(values, dict):
        out["values"] = {group: {str(name)[:64]: _tuning_value(value) for name, value in list(values[group].items())[:MAX_TUNING_KEYS]}
                         for group in TUNING_GROUPS if isinstance(values.get(group), dict)}
    return out


def page_grid(raw):
    """The executor's local grid for the map screen, or None when it is not a v1 export."""
    if not isinstance(raw, dict) or raw.get("schema") != GRID_SCHEMA:
        return None
    out = {key: _scalar(raw[key]) for key in GRID_SCALARS if key in raw}
    for key, count in (("origin_uv", 2), ("shape", 2), ("base_pose", 3)):
        value = _numbers(raw.get(key), count)
        if value is not None:
            out[key] = value
    if isinstance(raw.get("bits"), str) and len(raw["bits"]) <= MAX_GRID_BITS:
        out["bits"] = raw["bits"]
    if isinstance(raw.get("cells"), list):
        out["cells"] = [cell for cell in (_numbers(item, 2) for item in raw["cells"][:MAX_GRID_CELLS]) if cell is not None]
    lines = raw.get("lines")
    if isinstance(lines, dict):
        part = {}
        for name in ("left", "right", "centre"):
            segment = lines.get(name)
            if isinstance(segment, list) and len(segment) == 2:
                points = [_numbers(point, 2) for point in segment]
                if all(points):
                    part[name] = points
        for key in ("quality", "stamp"):
            if _number(lines.get(key)) is not None:
                part[key] = lines[key]
        out["lines"] = part
    return out


def page_metrics(raw):
    """status.metrics: the scalar aggregates (numbers, bools, short texts) under short snake_case names."""
    if not isinstance(raw, dict):
        return None
    out = {}
    for key, value in raw.items():
        if len(out) >= MAX_METRICS:
            break
        if not isinstance(key, str) or not METRIC_KEY.fullmatch(key):
            continue
        if isinstance(value, bool) or _number(value) is not None:
            out[key] = value
        elif isinstance(value, str):
            out[key] = value[:40]
    return out


def page_geometry(raw):
    """status.single_row_geometry for the monitoring view: base pose, band, recent path, optional lookahead point."""
    if not isinstance(raw, dict):
        return None
    out = {}
    if isinstance(raw.get("frame"), str):
        out["frame"] = raw["frame"][:64]
    base = _numbers(raw.get("base"), 3)
    if base is not None:
        out["base"] = base
    band = raw.get("band")
    if isinstance(band, dict):
        part = {k: band[k] for k in ("heading_rad", "centre_m", "width_m", "end_left_m", "end_right_m")
                if _number(band.get(k)) is not None}
        if part:
            out["band"] = part
    path = raw.get("path")
    if isinstance(path, list):
        points = [_numbers(point, 2) for point in path[-MAX_PATH_POINTS:]]
        out["path"] = [point for point in points if point is not None]
    for key in ("lookahead", "target"):
        point = _numbers(raw.get(key), 2)
        if point is not None:
            out[key] = point
    footprint = _numbers(raw.get("footprint_m"), 2)
    if footprint is not None:
        out["footprint_m"] = footprint
    return out


def page_status(raw, grid=False) -> dict:
    """The executor status reduced to the fields the screens show; the local grid and the run geometry only when
    asked (map screen, monitoring view)."""
    out = {}
    if not isinstance(raw, dict):
        return out
    for name, keys in STATUS_FIELDS.items():
        if name not in raw:
            continue
        value = raw[name]
        if keys is None:
            out[name] = _scalar(value)
            continue
        if not isinstance(value, dict):
            continue
        part = {}
        for key in keys:
            if key not in value:
                continue
            inner = NESTED.get((name, key))
            if inner is not None and isinstance(value[key], dict):
                part[key] = {k: _scalar(value[key][k]) for k in inner if k in value[key]}
            else:
                part[key] = _scalar(value[key])
        out[name] = part
    if isinstance(raw.get("tuning"), dict):
        out["tuning"] = page_tuning(raw["tuning"])
    metrics = page_metrics(raw.get("metrics"))
    if metrics is not None:
        out["metrics"] = metrics
    geometry = raw.get("single_row_geometry")
    pose = _numbers(geometry.get("base"), 3) if isinstance(geometry, dict) else None
    if pose is not None:
        out["pose"] = pose                 # run odometry (x m, y m, yaw rad), not GNSS
    if grid:
        local = page_grid(raw.get("local_grid_map"))
        if local is not None:
            out["local_grid_map"] = local
        geometry = page_geometry(raw.get("single_row_geometry"))
        if geometry is not None:
            out["single_row_geometry"] = geometry
    return out


class ExecutorError(Exception):
    def __init__(self, status: int, error: str, **extra):
        super().__init__(error)
        self.status, self.error, self.extra = status, error, extra

    def payload(self) -> dict:
        return {"ok": False, "error": self.error, **self.extra}


def parse_action_body(action: str, body) -> dict:
    """The body sent to the executor for one action, or ExecutorError(400)."""
    if not isinstance(body, dict):
        raise ExecutorError(400, "object_required")
    if action == "start":
        row = body.get("row_len_m")
        if row is not None and (isinstance(row, bool) or not isinstance(row, (int, float)) or not math.isfinite(row)
                                or not 0 < row <= 1000):
            raise ExecutorError(400, "invalid_row_len")
        out = {"row_len_m": row}
        mode = body.get("gnss_mode")
        if mode is not None:                         # absent or null: the executor's configured mode
            if not isinstance(mode, str) or mode not in GNSS_MODES:
                raise ExecutorError(400, "invalid_gnss_mode")
            out["gnss_mode"] = mode
        return out
    if action in ("go", "heartbeat"):
        session = body.get("session")
        if not isinstance(session, str) or not SESSION.fullmatch(session):
            raise ExecutorError(400, "invalid_session")
        return {"session": session}
    return {}


class ExecutorClient:
    """HTTP client of one executor operator API."""

    def __init__(self, url: str, token_file: str | None):
        parts = urlsplit(url)
        if parts.scheme != "http" or not parts.hostname or parts.port is None or parts.path not in ("", "/") \
                or parts.query or parts.fragment or parts.username or parts.password:
            raise ValueError("UBTS_EXECUTOR_URL must be http://host:port")
        self.host, self.port = parts.hostname, parts.port
        self.netloc = parts.netloc
        self.origin = f"http://{parts.netloc}"
        self.token_file = token_file
        self._token = None
        self._token_stamp = None
        self._lock = threading.Lock()

    @classmethod
    def from_env(cls, env=None) -> "ExecutorClient | None":
        env = os.environ if env is None else env
        url = (env.get("UBTS_EXECUTOR_URL") or "").strip()
        if not url:
            return None
        return cls(url, (env.get("UBTS_EXECUTOR_TOKEN_FILE") or "").strip() or None)

    def token(self) -> str:
        """The current token; the file is read again when it changes (the executor writes a new one at every start)."""
        if not self.token_file:
            raise ExecutorError(503, "executor_token_not_configured")
        try:
            info = os.stat(self.token_file)
        except OSError:
            raise ExecutorError(502, "executor_token_missing") from None
        stamp = (info.st_mtime_ns, info.st_size)
        with self._lock:
            if stamp != self._token_stamp:
                try:
                    with open(self.token_file, "r", encoding="utf-8") as handle:
                        text = handle.read(512).strip()
                except OSError:
                    raise ExecutorError(502, "executor_token_missing") from None
                self._token, self._token_stamp = text, stamp
            if not self._token:
                raise ExecutorError(502, "executor_token_missing")
            return self._token

    def call(self, method: str, path: str, body: dict | None = None, timeout: float = DEFAULT_TIMEOUT_S):
        """(HTTP status, JSON object) from the executor, or ExecutorError(502) when it cannot be reached."""
        headers = {"X-Row-Token": self.token(), "Accept": "application/json"}
        data = None
        if method == "POST":
            data = json.dumps(body or {}, allow_nan=False, separators=(",", ":")).encode("utf-8")
            headers.update({"Content-Type": "application/json", "Origin": self.origin})
        connection = http.client.HTTPConnection(self.host, self.port, timeout=timeout)
        try:
            connection.request(method, path, body=data, headers=headers)
            response = connection.getresponse()
            raw = response.read(256 * 1024)
            status = response.status
        except (OSError, http.client.HTTPException):
            raise ExecutorError(502, "executor_unreachable") from None
        finally:
            connection.close()
        if status == 403:
            raise ExecutorError(502, "executor_token_rejected")
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            raise ExecutorError(502, "executor_bad_reply", executor_status=status) from None
        if not isinstance(payload, dict):
            raise ExecutorError(502, "executor_bad_reply", executor_status=status)
        return status, payload

    def status(self, grid=False) -> dict:
        """Always a dict for GET /api/executor/status (the page polls it; HTTP 200 even when the executor is away).
        grid: also the local grid export (the map screen asks with ?grid=1)."""
        received = time.time()
        try:
            code, raw = self.call("GET", "/api/status")
        except ExecutorError as exc:
            return {"ok": False, "configured": True, "reachable": exc.error not in ("executor_unreachable",),
                    "error": exc.error, "received_at": received}
        if code != 200:
            return {"ok": False, "configured": True, "reachable": True, "error": raw.get("error") or f"executor_http_{code}",
                    "received_at": received}
        return {"ok": True, "configured": True, "reachable": True, "received_at": received, "status": page_status(raw, grid)}

    def action(self, action: str, body) -> tuple[int, dict]:
        """(HTTP status for the page, reply) of one POST action."""
        if action not in ACTIONS:
            raise ExecutorError(404, "unknown_action")
        sent = parse_action_body(action, body)
        code, reply = self.call("POST", f"/api/{action}", sent, TIMEOUT_S.get(action, DEFAULT_TIMEOUT_S))
        if isinstance(reply.get("status"), dict):        # a start reply carries the status: the same fields as GET
            reply["status"] = page_status(reply["status"])
        return code, reply


def hand_over_manual_lease(request_fn, sleep=None) -> str | None:
    """MOTOR '준비' (executor arm, 2026-11-19 corridor): the executor takes the bridge lease, and a manual Arm of the
    drive page holds it (the bridge answers control_in_use). The operator's press hands it over: when the bridge is
    armed_manual and not moving, a bridge stop of that lease first. A moving vehicle is left alone (the arm is then
    refused as before). Returns 'released', 'moving' or None (nothing to do / bridge not reachable)."""
    try:
        status = request_fn({"op": "status"})
    except Exception:
        return None
    if not isinstance(status, dict) or not status.get("armed") or status.get("state") != "armed_manual":
        return None
    if status.get("moving"):
        return "moving"
    try:
        request_fn({"op": "stop", "reason": "executor_takeover"})
    except Exception:
        return None
    (sleep or time.sleep)(0.3)
    return "released"


LINK_SCHEMA = "ubts.row_executor.dashboard_link.v1"
LINK_FILE = "runtime/row_executor/dashboard_link.json"
LOOPBACK_HOSTS = ("127.0.0.1", "localhost", "::1")
MAX_LINK_BYTES = 2048


class LinkedExecutor:
    """The executor of the kit's link file (module docstring). current() -> ExecutorClient or None; the file is read
    again when its (mtime, size) changes; a missing or refused file means 'not configured' (error() says why)."""

    def __init__(self, path):
        self.path = str(path)
        self._stamp = None
        self._client = None
        self._error = "link_file_missing"
        self._lock = threading.Lock()

    def error(self):
        return self._error

    def current(self):
        try:
            info = os.stat(self.path)
        except OSError:
            with self._lock:
                self._stamp, self._client, self._error = None, None, "link_file_missing"
            return None
        stamp = (info.st_mtime_ns, info.st_size)
        with self._lock:
            if stamp != self._stamp:
                self._stamp = stamp
                self._client, self._error = self._load(info.st_size)
            return self._client

    def _load(self, size):
        if size > MAX_LINK_BYTES:
            return None, "link_file_too_large"
        try:
            with open(self.path, "r", encoding="utf-8") as handle:
                data = json.loads(handle.read(MAX_LINK_BYTES + 1))
        except (OSError, UnicodeDecodeError, ValueError):
            return None, "link_file_unreadable"
        if not isinstance(data, dict) or data.get("schema") != LINK_SCHEMA or set(data) - {"schema", "url", "token_file", "note"}:
            return None, "link_file_schema"
        url, token = data.get("url"), data.get("token_file")
        if not isinstance(url, str) or not isinstance(token, str):
            return None, "link_file_schema"
        if urlsplit(url).hostname not in LOOPBACK_HOSTS:
            return None, "link_url_not_loopback"
        folder = os.path.dirname(os.path.realpath(self.path))
        if os.path.dirname(os.path.realpath(token)) != folder:
            return None, "link_token_outside_folder"
        try:
            return ExecutorClient(url, token), None
        except ValueError:
            return None, "link_url_invalid"


def executor_from_env(env, root):
    """The dashboard's executor relay: ExecutorClient (UBTS_EXECUTOR_URL) or LinkedExecutor (the kit's link file)."""
    client = ExecutorClient.from_env(env)
    if client is not None:
        return client
    path = (env.get("UBTS_EXECUTOR_LINK_FILE") or "").strip() or os.path.join(str(root), LINK_FILE)
    return LinkedExecutor(path)


def not_configured_status() -> dict:
    return {"ok": False, "configured": False, "reachable": False, "error": "executor_not_configured",
            "received_at": time.time()}


def read_body(handler):
    """The JSON object body of a dashboard request (at most 4 KiB), or ExecutorError."""
    length = handler.headers.get("Content-Length")
    if length is None or not length.isdigit() or int(length) > MAX_BODY:
        raise ExecutorError(413, "body_size")
    raw = handler.rfile.read(int(length)) if int(length) else b"{}"
    # The whole body was read: the connection may stay open for the next request (NET-05 keep-alive).
    handler._body_consumed = int(length) == 0 or len(raw) == int(length)

    def refuse(_value):
        raise ValueError("non-finite number")
    try:
        body = json.loads(raw.decode("utf-8"), parse_constant=refuse)
    except (UnicodeDecodeError, ValueError):
        raise ExecutorError(400, "invalid_json") from None
    if not isinstance(body, dict):
        raise ExecutorError(400, "object_required")
    return body
