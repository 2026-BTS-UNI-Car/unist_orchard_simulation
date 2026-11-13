"""Relay of the row executor's operator API for the dashboard (docs/STEERING_SERVER_KO.md 6 and 7, contract v0).

The dashboard calls the executor's operator HTTP on behalf of the page, so the page never sees the executor token:

  UBTS_EXECUTOR_URL         http://host:port of the executor operator page (tools/row_executor/operator_http.py)
  UBTS_EXECUTOR_TOKEN_FILE  the token file the executor writes at every start (operator_token.txt)

Without UBTS_EXECUTOR_URL the relay is "not configured": the status says so and every action answers 503. The
executor checks Host, Origin and the X-Row-Token header and takes JSON bodies of at most 4 KiB; this module sends
exactly that, with only the fields of the contract. The status is reduced to the fields the screens show (no GNSS
coordinates, no paths). Standard library only.
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

# GET /api/status fields passed to the page (STEERING_SERVER_KO.md 6). None = the whole value (a scalar or a list of
# strings), a tuple = only these keys of the object.
STATUS_FIELDS = {
    "run_state": None, "start_mode": None, "stop_reason": None, "stop_detail": None, "go_ready": None,
    "go_blockers": None, "mode": None, "state": None, "reason": None, "end_reason": None, "controller_mode": None,
    "profile": None, "vehicle_control": None, "lidar_age": None, "imu_age": None,
    "operator_link": ("fresh", "age_s", "timeout_s"),
    "speed": ("target_mps", "limit_mps", "limit_reason"),
    "tuning": ("preset", "revision", "editable", "values", "editable_names", "limits"),
    "single_row": ("phase", "lateral_error_m", "heading_error_deg", "distance_m", "speed_m_s", "target_speed_m_s",
                   "stop_reason", "stop_category", "confidence"),
    "gnss": ("state", "fix", "age_s", "h_acc_m", "used"),
    "bridge_detail": ("state", "latch_reason"),
}
NESTED = {("single_row", "confidence"): ("level",)}


def _scalar(value):
    if value is None or isinstance(value, (bool, str)):
        return value if not isinstance(value, str) else value[:200]
    if isinstance(value, (int, float)):
        return value if math.isfinite(value) else None
    if isinstance(value, list):
        return [item[:200] for item in value[:32] if isinstance(item, str)]
    return None


def page_status(raw) -> dict:
    """The executor status reduced to the fields the screens show."""
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
            elif name == "tuning" and key in ("values", "limits") and isinstance(value[key], dict):
                part[key] = {str(k)[:64]: (_scalar(v) if not isinstance(v, list) else [_scalar(x) for x in v[:2]])
                             for k, v in list(value[key].items())[:64]}
            else:
                part[key] = _scalar(value[key])
        out[name] = part
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
        return {"row_len_m": row}
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

    def status(self) -> dict:
        """Always a dict for GET /api/executor/status (the page polls it; HTTP 200 even when the executor is away)."""
        received = time.time()
        try:
            code, raw = self.call("GET", "/api/status")
        except ExecutorError as exc:
            return {"ok": False, "configured": True, "reachable": exc.error not in ("executor_unreachable",),
                    "error": exc.error, "received_at": received}
        if code != 200:
            return {"ok": False, "configured": True, "reachable": True, "error": raw.get("error") or f"executor_http_{code}",
                    "received_at": received}
        return {"ok": True, "configured": True, "reachable": True, "received_at": received, "status": page_status(raw)}

    def action(self, action: str, body) -> tuple[int, dict]:
        """(HTTP status for the page, reply) of one POST action."""
        if action not in ACTIONS:
            raise ExecutorError(404, "unknown_action")
        sent = parse_action_body(action, body)
        return self.call("POST", f"/api/{action}", sent, TIMEOUT_S.get(action, DEFAULT_TIMEOUT_S))


def not_configured_status() -> dict:
    return {"ok": False, "configured": False, "reachable": False, "error": "executor_not_configured",
            "received_at": time.time()}


def read_body(handler):
    """The JSON object body of a dashboard request (at most 4 KiB), or ExecutorError."""
    length = handler.headers.get("Content-Length")
    if length is None or not length.isdigit() or int(length) > MAX_BODY:
        raise ExecutorError(413, "body_size")
    raw = handler.rfile.read(int(length)) if int(length) else b"{}"

    def refuse(_value):
        raise ValueError("non-finite number")
    try:
        body = json.loads(raw.decode("utf-8"), parse_constant=refuse)
    except (UnicodeDecodeError, ValueError):
        raise ExecutorError(400, "invalid_json") from None
    if not isinstance(body, dict):
        raise ExecutorError(400, "object_required")
    return body
