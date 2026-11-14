"""Field tuning of the row executor as the dashboard shows and changes it (UI-12, docs/STEERING_SERVER_KO.md 7).

The executor is the reference: tools/row_executor/field_tuning.py of the simulation repository (FIELD_TUNING_KO.md,
OPERATOR_API_KO.md 6). Its status `tuning` carries the bundle: the basic group with names, values, ranges and units,
the advanced and locked groups with their values, the preset list, the revision and why it can not change now. The
dashboard draws what the executor sends (it keeps no ranges of its own), adds Korean labels, checks a request against
the executor's own ranges so the page can say why first, relays it to POST /api/params and keeps a change log.

What the screen may send (the executor's forms): basic keys only {name: number}; {"preset": name} alone;
{"save": true}; {"save_preset": name, "note": text}; the answer to the online-scale offer after a run (PW-10,
contract revision 6) {"online_scale": "accept" | "dismiss", "run_id": N}. Advanced and locked keys change in the file
only.
"""
from __future__ import annotations

import json
import math
import os
import re
import threading
import time
from pathlib import Path

from dashboard_server.executor_proxy import ExecutorError

SCHEMA = "ubts.executor_tuning.v1"
GROUPS = ("basic", "advanced", "locked")
LOG_KEEP = 500
LOG_SHOW = 50
PRESET_NAME = re.compile(r"[A-Za-z0-9_.-]{1,40}")       # field_tuning._name_ok
DEFAULT_PRESET = "default"
NOTE_MAX = 200
SCALE_ANSWERS = ("accept", "dismiss")                    # PW-10: the online-scale offer (OPERATOR_API_KO.md 6.1)

# Korean labels of the executor's keys (field_tuning.KEYS). A key the executor adds later is shown by its name.
LABELS = {
    "speed_scale": "속도 배율(k_field)", "target_speed_m_s": "목표 속도", "lookahead_base_m": "주시거리 바닥",
    "lookahead_per_speed_s": "속도당 주시거리", "lookahead_max_m": "주시거리 상한", "k_d": "횡오차 이득(k_d)",
    "k_psi": "방향 이득(k_psi)", "steer_rate_fraction": "조향 변화 속도 비율",
    "speed_table": "duty→속도 표", "surface_scale": "노면 배율", "battery_scale": "배터리 배율",
    "min_speed_m_s": "최저 속도", "approach_speed_m_s": "열 끝 접근 속도", "approach_window_m": "열 끝 접근 거리",
    "end_stop_offset_m": "열 끝 앞 정지 거리", "duty_max_pct": "구동 duty 상한", "speed_kp_pct_s_per_m": "속도 PI 비례 이득",
    "speed_ki_pct_per_m": "속도 PI 적분 이득", "speed_integral_limit_pct": "속도 PI 적분 한도",
    "duty_rise_pct_s": "duty 올림 속도", "duty_fall_pct_s": "duty 내림 속도", "k_r": "요 각속도 이득(k_r)",
    "trim_gain_rad_per_m_s": "조향 trim 이득", "trim_limit_deg": "조향 trim 한도", "steer_follow_limit_deg": "추종 조향 한도",
    "steer_filter_s": "조향 목표 필터", "d_scan_m": "스캔당 이동 한도", "d_blind_m": "루프 지연당 이동 한도",
    "loop_fixed_s": "루프 고정 지연", "corridor_weak_age_s": "통로 갱신 나이 한도", "lateral_slow_start_m": "횡오차 감속 시작",
    "lateral_slow_full_m": "횡오차 최저 속도 지점", "lateral_accel_max_m_s2": "횡가속도 상한",
    "steer_slow_start": "조향 요구 감속 시작", "fresh_slow_age_s": "스캔 나이 감속 시작",
    "max_speed_m_s": "최고 속도", "front_margin_m": "앞 여유", "overspeed_margin_m_s": "과속 여유", "overspeed_s": "과속 지속",
    "duty_cap_pct": "출력 가드 duty 상한", "lidar_max_age_s": "스캔 신선도 문턱", "imu_max_age_s": "IMU 신선도 문턱",
    "steer_status_max_age_s": "조향 상태 신선도 문턱", "stall_guard_s": "루프 틈 문턱",
    "operator_timeout_s": "화면 연결 끊김 정지", "guard_heading_deg": "방향 가드",
    "prior_predict_max_s": "사전값 예측 한도(시간)", "prior_predict_max_m": "사전값 예측 한도(거리)",
}
# Units of the advanced and locked keys, read from the name (the executor's status gives units for the basic group
# only). Longest suffix first.
UNIT_SUFFIXES = (("_pct_s_per_m", "%/(m/s)"), ("_rad_per_m_s", "rad/(m·s)"), ("_pct_per_m", "%/m"), ("_m_s2", "m/s²"),
                 ("_pct_s", "%/s"), ("_m_s", "m/s"), ("_pct", "%"), ("_deg", "°"), ("_m", "m"), ("_s", "s"))


def _number(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


def unit_of(name: str) -> str:
    if name == "speed_table":
        return "[duty %, m/s]"
    for suffix, unit in UNIT_SUFFIXES:
        if name.endswith(suffix):
            return unit
    return ""


def step_of(low, high) -> float:
    """Input step of a basic key from its range (the executor gives none)."""
    return 1 if low is not None and high is not None and high - low > 20 else 0.01


def _value(value):
    """A tuning value fit for the page and the log: a finite number, a short string, or a [[n, n], ...] table."""
    if isinstance(value, list):
        return [[_number(x) for x in row[:2]] for row in value[:32] if isinstance(row, list)]
    if isinstance(value, str):
        return value[:64]
    return _number(value)


def view(executor_status: dict) -> dict:
    """GET /api/executor/tuning: the executor's bundle as the screen draws it."""
    status = executor_status.get("status") if isinstance(executor_status, dict) else None
    reachable = bool(isinstance(executor_status, dict) and executor_status.get("ok") and isinstance(status, dict))
    tuning = status.get("tuning") if reachable and isinstance(status.get("tuning"), dict) else None
    run_state = status.get("run_state") if reachable else None
    available = tuning is not None
    applicable = bool(available and tuning.get("editable") is True)
    why_not = None
    if not reachable:
        why_not = executor_status.get("error") if isinstance(executor_status, dict) else None
    elif not available:
        why_not = "no_field_tuning"
    elif not applicable:
        why_not = tuning.get("not_editable_because") or "tuning_locked"
    params = []
    if available:
        for item in tuning.get("basic") if isinstance(tuning.get("basic"), list) else []:
            if not isinstance(item, dict) or not isinstance(item.get("name"), str):
                continue
            low, high = _number(item.get("low")), _number(item.get("high"))
            ranged = low is not None and high is not None and low <= high
            params.append({"name": item["name"], "group": "basic", "label": LABELS.get(item["name"], item["name"]),
                           "unit": item.get("unit") if isinstance(item.get("unit"), str) else "", "min": low, "max": high,
                           "step": step_of(low, high), "value": _number(item.get("value")),
                           "editable": applicable and ranged, "editable_when_stopped": ranged})
        values = tuning.get("values") if isinstance(tuning.get("values"), dict) else {}
        for group in ("advanced", "locked"):
            entries = values.get(group) if isinstance(values.get(group), dict) else {}
            for name, value in entries.items():
                params.append({"name": name, "group": group, "label": LABELS.get(name, name), "unit": unit_of(name),
                               "min": None, "max": None, "step": None, "value": _value(value), "editable": False,
                               "editable_when_stopped": False})
    brief = {}
    if available:
        brief = {key: tuning.get(key) for key in ("preset", "revision", "modified", "presets", "writable", "unsaved")
                 if key in tuning}
    scale = status.get("online_scale") if reachable and isinstance(status.get("online_scale"), dict) else None
    offer = scale.get("offer") if scale is not None and isinstance(scale.get("offer"), dict) else None
    return {"ok": True, "schema": SCHEMA,
            "configured": executor_status.get("configured", False) if isinstance(executor_status, dict) else False,
            "reachable": reachable, "error": None if reachable else (executor_status or {}).get("error"),
            "run_state": run_state, "available": available, "applicable": applicable, "why_not": why_not,
            "tuning": brief, "params": params, "online_scale_offer": offer}


def validate(body, tuning_view: dict) -> dict:
    """The body to send to POST /api/params (one of the executor's forms), or ExecutorError with the reason."""
    if not isinstance(body, dict) or not body:
        raise ExecutorError(400, "no_change")
    if not tuning_view["reachable"]:
        raise ExecutorError(502, tuning_view.get("error") or "executor_unreachable")
    if not tuning_view["available"]:
        raise ExecutorError(409, "no_field_tuning")
    if "online_scale" in body:
        return _scale_answer(body, tuning_view)
    if not tuning_view["applicable"]:
        raise ExecutorError(409, tuning_view.get("why_not") or "tuning_locked", run_state=tuning_view.get("run_state"))
    brief = tuning_view["tuning"]
    if "preset" in body:
        name = body["preset"]
        presets = brief.get("presets") if isinstance(brief.get("presets"), list) else []
        if len(body) != 1:
            raise ExecutorError(400, "preset_alone")
        if not isinstance(name, str) or name not in presets:
            raise ExecutorError(400, "unknown_preset", field="preset")
        return {"preset": name}
    if "save" in body or "save_preset" in body:
        if set(body) - {"save", "save_preset", "note"} or body.get("save", True) is not True:
            raise ExecutorError(400, "save_form")
        if brief.get("writable") is not True:
            raise ExecutorError(409, "tuning_not_writable")
        if "save_preset" not in body:
            return {"save": True}
        name = body["save_preset"]
        if not isinstance(name, str) or not PRESET_NAME.fullmatch(name) or name == DEFAULT_PRESET:
            raise ExecutorError(400, "bad_preset_name", field="save_preset")
        note = body.get("note", "")
        if not isinstance(note, str) or len(note) > NOTE_MAX:
            raise ExecutorError(400, "bad_note", field="note")
        return {"save_preset": name, "note": note}
    by_name = {param["name"]: param for param in tuning_view["params"]}
    out = {}
    for name, value in body.items():
        param = by_name.get(name)
        if param is None:
            raise ExecutorError(400, "unknown_parameter", field=str(name)[:64])
        if param["group"] != "basic" or not param["editable"]:
            raise ExecutorError(400, "locked_parameter", field=name)
        number = _number(value)
        if number is None or not param["min"] <= number <= param["max"]:
            raise ExecutorError(400, "out_of_range", field=name, min=param["min"], max=param["max"])
        out[name] = number
    return out


def _scale_answer(body: dict, tuning_view: dict) -> dict:
    """The answer to the executor's online-scale offer (PW-10): {"online_scale": "accept" | "dismiss", "run_id": N}.
    The offer of the status must be there and be for that run; accept also needs an offer that can be taken and a
    tuning that may change now (the executor checks both again)."""
    answer, run = body.get("online_scale"), body.get("run_id")
    if set(body) - {"online_scale", "run_id"}:
        raise ExecutorError(400, "online_scale_form")
    if answer not in SCALE_ANSWERS or isinstance(run, bool) or not isinstance(run, int):
        raise ExecutorError(400, "online_scale_form")
    offer = tuning_view.get("online_scale_offer")
    if not isinstance(offer, dict):
        raise ExecutorError(409, "no_online_scale_offer")
    if offer.get("run_id") != run:
        raise ExecutorError(409, "online_scale_offer_changed", run_id=offer.get("run_id"))
    if answer == "accept":
        if offer.get("ok") is not True:
            raise ExecutorError(409, offer.get("why_not") or "online_scale_not_offered")
        if not tuning_view["applicable"]:
            raise ExecutorError(409, tuning_view.get("why_not") or "tuning_locked", run_state=tuning_view.get("run_state"))
    return {"online_scale": answer, "run_id": run}


def page_reply(reply: dict) -> dict:
    """The executor's POST /api/params reply without the file path and digests."""
    out = {"ok": reply.get("ok") is True}
    for key in ("action", "error"):
        if isinstance(reply.get(key), str):
            out[key] = reply[key][:64]
    if isinstance(reply.get("problems"), list):
        out["problems"] = [str(text)[:200] for text in reply["problems"][:20]]
    if isinstance(reply.get("changes"), list):
        out["changes"] = [{"key": str(change.get("key"))[:64], "old": _value(change.get("old")), "new": _value(change.get("new"))}
                          for change in reply["changes"][:64] if isinstance(change, dict)]
    tuning = reply.get("tuning")
    if isinstance(tuning, dict):
        out["tuning"] = {key: tuning.get(key) for key in ("preset", "revision") if key in tuning}
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
            temp.write_bytes(("\n".join(lines) + "\n").encode("utf-8"))
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


def _action_of(body) -> str:
    if not isinstance(body, dict):
        return "set"
    if "online_scale" in body:
        return "online_scale_dismiss" if body.get("online_scale") == "dismiss" else "online_scale"
    if "preset" in body:
        return "load_preset"
    if "save_preset" in body:
        return "save_preset"
    return "save" if "save" in body else "set"


def apply(client, body, log: TuningLog, who: str) -> tuple[int, dict]:
    """POST /api/executor/params: check, send, log. Returns (HTTP status, reply for the page)."""
    before = view(client.status())
    old = {param["name"]: param["value"] for param in before["params"]}
    revision_before = before["tuning"].get("revision")
    action = _action_of(body)
    target = None
    if isinstance(body, dict) and action in ("load_preset", "save_preset"):
        target = body.get("preset" if action == "load_preset" else "save_preset")
        target = target[:40] if isinstance(target, str) else None
    if isinstance(body, dict) and action.startswith("online_scale"):
        run = body.get("run_id")
        target = f"run {run}" if isinstance(run, int) and not isinstance(run, bool) else None
    try:
        sent = validate(body, before)
    except ExecutorError as exc:
        changes = {}
        if action == "set" and isinstance(body, dict):
            changes = {str(k)[:64]: [_value(old.get(k)), _value(v)] for k, v in list(body.items())[:16]}
        log.append({"by": who, "action": action, "target": target, "changes": changes, "ok": False, "error": exc.error,
                    "problems": [], "revision_before": revision_before, "revision_after": revision_before})
        raise
    code, raw = client.call("POST", "/api/params", sent)
    reply = page_reply(raw)
    ok = code == 200 and reply["ok"]
    after = view(client.status())
    if ok:
        changes = {change["key"]: [change["old"], change["new"]] for change in reply.get("changes", [])}
    else:
        changes = {name: [_value(old.get(name)), _value(value)] for name, value in sent.items()} if action == "set" else {}
    log.append({"by": who, "action": action, "target": target, "changes": changes, "ok": ok,
                "error": None if ok else (reply.get("error") or f"executor_http_{code}"),
                "problems": reply.get("problems", [])[:5], "revision_before": revision_before,
                "revision_after": after["tuning"].get("revision")})
    return code, reply
