"""Tuning values of the row executor as the dashboard shows and changes them (UI-12, docs/STEERING_SERVER_KO.md 7).

Decision 16 (todo 4): a basic group the screen can change, an advanced group, and safety limits that are locked on the
screen; values are applied only while the vehicle is stopped. The executor is the final judge (its POST /api/params
refuses what it does not take); the dashboard checks first so the page can say why, and keeps a change log.

The catalogue below gives names, units, ranges and sources while the executor does not report them. Every range and
default is a placeholder with its source; names follow the executor's settings when it reports them
(status tuning.values / editable_names / limits, contract requests in STEERING_SERVER_KO.md 11).
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path

from dashboard_server.executor_proxy import ExecutorError

SCHEMA = "ubts.executor_tuning.v0"
STOPPED_STATES = ("idle", "ready", "ended")
LOG_KEEP = 500
LOG_SHOW = 50

# name, group, label, unit, min, max, step, default, source
CATALOG = (
    ("speed_table_scale", "basic", "속도 표 배율", "배", 0.80, 1.20, 0.01, 1.00, "자리값: PWM-속도 표(PW-07) 대기"),
    ("target_speed_m_s", "basic", "목표 속도", "m/s", 0.20, 0.50, 0.05, 0.40, "자리값: 결정 나10"),
    ("lookahead_m", "basic", "주시거리", "m", 1.0, 4.0, 0.1, 2.0, "자리값: PREREG_STEER_FOLLOW 2.0 m"),
    ("lateral_gain", "basic", "횡 이득", "rad/m", 0.10, 1.50, 0.05, 0.40, "자리값: StraightHold k_d"),
    ("heading_gain", "basic", "방향 이득", "rad/rad", 0.10, 3.00, 0.05, 0.90, "자리값: StraightHold k_psi"),
    ("steer_rate_limit_deg_s", "basic", "조향 속도 제한", "°/s", 4.0, 30.0, 1.0, 24.0, "자리값: config.py 0.8 × B5"),
    ("steer_rate_limit_standstill_deg_s", "advanced", "정지 중 조향 속도 제한", "°/s", 4.0, 20.0, 1.0, 12.0,
     "자리값: config.py 0.8 × B5 정지"),
    ("min_speed_m_s", "advanced", "최저 목표 속도", "m/s", 0.10, 0.40, 0.05, 0.20, "자리값: single_row_min_speed_m_s"),
    ("approach_speed_m_s", "advanced", "열 끝 접근 속도", "m/s", 0.10, 0.40, 0.05, 0.20,
     "자리값: single_row_approach_speed_m_s"),
    ("corridor_lost_s", "advanced", "통로 잃음 판정", "s", 0.5, 5.0, 0.1, None, "자리값: CONTROLLER_OVERRIDE_LIMITS"),
    ("operator_timeout_s", "safety", "화면 연결 끊김 정지", "s", 0.5, 3.0, 0.1, 0.5, "규약 v0 개정 1(A17 개정 31)"),
    ("max_speed_m_s", "safety", "최고 속도", "m/s", 0.05, 0.50, 0.01, 0.50, "자리값: config.py max_speed_m_s"),
    ("steer_limit_left_mrad", "safety", "왼쪽 조향 한계", "mrad", 0, 409, 1, 409, "자리값: 호스트 프로필 v0"),
    ("steer_limit_right_mrad", "safety", "오른쪽 조향 한계", "mrad", 0, 409, 1, 409, "자리값: 호스트 프로필 v0"),
    ("steer_status_max_age_s", "safety", "조향 상태 최대 나이", "s", 0.1, 1.0, 0.05, 0.30, "자리값: R18 결정 6"),
)
GROUPS = ("basic", "advanced", "safety")


def _number(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


def view(executor_status: dict) -> dict:
    """GET /api/executor/tuning: the catalogue joined with what the executor reports (STEERING_SERVER_KO.md 7)."""
    status = executor_status.get("status") if isinstance(executor_status, dict) else None
    reachable = bool(isinstance(executor_status, dict) and executor_status.get("ok") and isinstance(status, dict))
    tuning = status.get("tuning") if reachable and isinstance(status.get("tuning"), dict) else {}
    values = tuning.get("values") if isinstance(tuning.get("values"), dict) else {}
    limits = tuning.get("limits") if isinstance(tuning.get("limits"), dict) else {}
    names = tuning.get("editable_names")
    names = set(names) if isinstance(names, list) else None
    run_state = status.get("run_state") if reachable else None
    applicable = reachable and run_state in STOPPED_STATES and tuning.get("editable") is not False
    params = []
    for name, group, label, unit, low, high, step, default, source in CATALOG:
        limit = limits.get(name)
        if isinstance(limit, list) and len(limit) == 2 and all(_number(x) is not None for x in limit) and limit[0] < limit[1]:
            low, high = limit
        if group == "safety":
            editable = False
        elif names is not None:
            editable = name in names
        else:
            editable = group == "basic"
        params.append({"name": name, "group": group, "label": label, "unit": unit, "min": low, "max": high, "step": step,
                       "default": default, "value": _number(values.get(name)), "source": source,
                       "editable": bool(editable and applicable), "editable_when_stopped": bool(editable)})
    return {"ok": True, "schema": SCHEMA, "configured": executor_status.get("configured", False) if isinstance(executor_status, dict) else False,
            "reachable": reachable, "error": None if reachable else (executor_status or {}).get("error"),
            "run_state": run_state, "applicable": bool(applicable),
            "tuning": {key: tuning.get(key) for key in ("preset", "revision", "editable") if key in tuning}, "params": params}


def validate(body, tuning_view: dict) -> dict:
    """The values to send (POST /api/params), or ExecutorError with the reason and the field."""
    if not isinstance(body, dict) or not body:
        raise ExecutorError(400, "no_change")
    by_name = {param["name"]: param for param in tuning_view["params"]}
    if not tuning_view["reachable"]:
        raise ExecutorError(502, tuning_view.get("error") or "executor_unreachable")
    if not tuning_view["applicable"]:
        raise ExecutorError(409, "not_stopped", run_state=tuning_view.get("run_state"))
    out = {}
    for name, value in body.items():
        param = by_name.get(name)
        if param is None:
            raise ExecutorError(400, "unknown_parameter", field=str(name)[:64])
        if param["group"] == "safety" or not param["editable"]:
            raise ExecutorError(400, "locked_parameter", field=name)
        number = _number(value)
        if number is None or not param["min"] <= number <= param["max"]:
            raise ExecutorError(400, "out_of_range", field=name, min=param["min"], max=param["max"])
        out[name] = number
    return out


class TuningLog:
    """Change log of the tuning screen: one JSON line per attempt, the last LOG_KEEP lines kept."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self._lock = threading.Lock()

    def append(self, entry: dict) -> None:
        line = json.dumps({"at": round(time.time(), 3), **entry}, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            lines = self._read()
            lines.append(line)
            lines = lines[-LOG_KEEP:]
            temp = self.path.with_suffix(".tmp")
            temp.write_text("\n".join(lines) + "\n", encoding="utf-8")
            os.replace(temp, self.path)

    def recent(self, count: int = LOG_SHOW) -> list:
        with self._lock:
            lines = self._read()
        out = []
        for line in reversed(lines[-count:]):
            try:
                out.append(json.loads(line))
            except ValueError:
                continue
        return out

    def _read(self) -> list:
        try:
            return [line for line in self.path.read_text(encoding="utf-8").splitlines() if line.strip()]
        except OSError:
            return []


def apply(client, body, log: TuningLog, who: str) -> tuple[int, dict]:
    """POST /api/executor/params: check, send, log. Returns (HTTP status, reply)."""
    before = view(client.status())
    old = {param["name"]: param["value"] for param in before["params"]}
    revision_before = before["tuning"].get("revision")
    try:
        params = validate(body, before)
    except ExecutorError as exc:
        log.append({"by": who, "changes": {str(k)[:64]: [old.get(k), v if _number(v) is not None else None]
                                           for k, v in (body.items() if isinstance(body, dict) else ())},
                    "ok": False, "error": exc.error, "revision_before": revision_before, "revision_after": revision_before})
        raise
    code, reply = client.call("POST", "/api/params", params)
    ok = code == 200 and reply.get("ok") is not False
    after = view(client.status())
    log.append({"by": who, "changes": {name: [old.get(name), value] for name, value in params.items()}, "ok": ok,
                "error": None if ok else (reply.get("error") or f"executor_http_{code}"),
                "revision_before": revision_before, "revision_after": after["tuning"].get("revision")})
    return code, reply
