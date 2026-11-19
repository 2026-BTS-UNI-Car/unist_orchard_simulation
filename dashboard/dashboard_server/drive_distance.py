"""Distance moves of the front-steer vehicle (2026-11-17, user request: choose the terrain, send '1 m forward').

Data: config/drive_terrain_table_202610.json, copied with its provenance from the October 2026 drive records
(simulation repository docs/drive_response/: T3 speed by ground, T4s starts, T5s stops, T6 distance from a stop). They
are the values of the vehicle BEFORE the steering unit (ESP8266, differential drive, firmware drive ramp 100 %/s); the
steering vehicle has not been measured.

Speed (2026-11-19, user request '거리 이동하는거 속도 조절 되게'): the operator picks any integer duty from the terrain's
lowest to its highest table duty. A table duty uses its row; a duty between two rows uses the straight line between
them (grade '모델', never outside the table: no extrapolation). Without a duty the move uses the terrain's lowest table
duty, the first choice of the 2026-11-17 page (where the operator's tape corrections so far were taken).

Planner (docs/esp_rover_control.md 1-6). For terrain g and duty d with steady speed v (T3), a hold of T seconds from a
stop followed by a cut of the drive is expected to move
    E(T) = c_gd * (v * (T - L) + C(v)),   valid for T >= DRIVE_MIN_S
  L    start lag = 2 s - s100(2 s) / v100 = 2 - 1.29 / 0.928 = 0.61 s: the distance after 2 s from a stop at 100 % (T6,
       9 events) against the steady 100 % speed (T2). Measured at 100 % only; other duties use it as an assumption.
  C(v) coast after the cut: 0.55 m measured at 100 % (T6, 7 key releases); otherwise the analysis model
       v * 0.25 + v^2 / 2.6 (T5s; measured/model p50 0.97).
  c_gd correction factor of terrain g at duty d from the operator's tape measurements (median of the last
       FACTOR_SAMPLES ratios actual / model taken at that duty, clamped FACTOR_MIN..FACTOR_MAX). A duty without its
       own measurement borrows the nearest measured duty of the terrain (the plan says which); 1.0 without any.
Inverted: T = L + (D / c_gd - C(v)) / v. Refused when T < DRIVE_MIN_S (1.3 s, T4s: 90 % of the steady speed after
p50 1.28 s; the start itself is outside the data) or T > DRIVE_MAX_S, and when the distance is outside 0.2..3.0 m.
The ESP32-S3 bridge's soft start and release (since 3dba62c, scripts/esp32_abs_hold.py DRIVE_RAMP_*: a start jumps to
PWM 60 and rises 150 PWM/s, a drive of 0 falls 320 PWM/s) are NOT in E(T): L and C are the old vehicle's. How much of a
move is spent ramping depends on the duty, so a correction only holds at the duty it was measured at; that is why the
corrections are kept per duty and a borrowed one is shown as borrowed.

Move (MoveRunner): the page holds the button (deadman) and its own lease; the server drives with that lease through the
bridge's normal 'drive' op (owner, lease, one-use ticket, seq + 1, deadman, input TTL 0.75 s, the bridge's max_pwm cap
and the firmware watchdog all unchanged): a drive request every SEND_PERIOD_S at the plan's duty with the steering
target held, and a request with throttle 0 at the planned time (or at once on release, a missing hold heartbeat for
HOLD_TIMEOUT_S, or STOP). Meanwhile the page sends no drive of its own (the server owns the ticket chain) and takes the
chain back afterwards; a chain nobody takes within TAKE_TIMEOUT_S is closed with STOP. No LiDAR loop (v2).

Backward (2026-11-19, user request '특정거리 앞으로 가는거 뿐만 아니라 뒤로 가는 기능'): direction 'backward' drives the same
duty with a negative throttle (the bridge's drive op takes -100..100; its cap applies to |throttle|). Every rule of the
forward move holds (Arm, held button, STOP, drive time 1.3..10 s, the bridge cap); the distance is at most
DISTANCE_MAX_BACKWARD_M because nothing looks behind the vehicle. The speed table was driven forward only: a backward
plan uses it as an assumption (grade '가정'). Tape corrections are kept per direction as well as per duty: a backward
move never borrows a forward correction (without a backward sample the factor is 1.0). Samples of the 2026-11-17/19
files have no direction and count as forward.
"""
from __future__ import annotations

import json
import math
import os
import statistics
import threading
import time
from pathlib import Path

TABLE_SCHEMA = "ubts.drive_terrain_table.v1"
STORE_SCHEMA = "ubts.drive_distance_corrections.v1"
DEFAULT_TABLE = Path(__file__).resolve().parent.parent / "config" / "drive_terrain_table_202610.json"
DISTANCE_MIN_M, DISTANCE_MAX_M, DISTANCE_DEFAULT_M = 0.2, 3.0, 1.0
DISTANCE_MAX_BACKWARD_M = 2.0           # no rear sensing: a shorter limit than forward
DIRECTIONS = ("forward", "backward")
DRIVE_MIN_S = 1.3
DRIVE_MAX_S = 10.0
FACTOR_MIN, FACTOR_MAX = 0.5, 1.5
FACTOR_SAMPLES = 5
SAMPLES_KEEP = 20
ACTUAL_MAX_M = 10.0
GRADE_ORDER = ("측정", "참고", "모델", "가정")

SEND_PERIOD_S = 0.1
HOLD_TIMEOUT_S = 0.4
TAKE_TIMEOUT_S = 2.0
KEEPALIVE_S = 0.4


def duty_pwm(duty: int) -> int:
    """Firmware PWM of a duty % (the bridge's throttle_to_pwm without its cap)."""
    return (abs(int(duty)) * 255 + 50) // 100


def cap_duty(max_pwm) -> int | None:
    """The highest duty % whose PWM is within the bridge's drive cap (None for an unknown cap)."""
    if not isinstance(max_pwm, int) or isinstance(max_pwm, bool):
        return None
    return max((d for d in range(0, 101) if duty_pwm(d) <= max_pwm), default=0)


def load_table(path=None) -> dict:
    data = json.loads(Path(path or DEFAULT_TABLE).read_bytes().decode("utf-8"))
    if data.get("schema") != TABLE_SCHEMA or not data.get("terrains"):
        raise ValueError("drive terrain table: unknown schema")
    return data


def _lag_point(table: dict) -> dict:
    start = table["start"]
    return next(p for p in start["distance_from_stop_100"] if p["t_s"] == start["lag_point_t_s"])


def start_lag_s(table: dict) -> float:
    point = _lag_point(table)
    return round(point["t_s"] - point["distance_m"] / table["start"]["steady_speed_100"]["speed_m_s"], 3)


def coast_m(table: dict, duty: int, speed: float) -> tuple[float, str]:
    coast = table["release_coast"]
    if duty == 100:
        return coast["duty_100"]["distance_m"], coast["duty_100"]["grade"]
    model = coast["model"]
    return round(speed * model["reaction_s"] + speed * speed / (2.0 * model["decel_m_s2"]), 3), model["grade"]


def terrain_of(table: dict, key) -> dict | None:
    return next((t for t in table["terrains"] if t["key"] == key), None)


def duty_range(terrain: dict) -> tuple[int, int]:
    """The lowest and the highest table duty of a terrain: the speeds the operator can choose."""
    duties = [r["duty"] for r in terrain["speeds"]]
    return min(duties), max(duties)


def default_duty(terrain: dict) -> int:
    """The duty of a request without one: the lowest of the terrain (the 2026-11-17 page's first choice)."""
    return duty_range(terrain)[0]


def speed_at(terrain: dict, duty: int) -> dict | None:
    """The table speed at a duty: its own row, or the straight line between the two rows around it (grade '모델');
    None outside the table (no extrapolation)."""
    rows = sorted(terrain["speeds"], key=lambda r: r["duty"])
    for row in rows:
        if row["duty"] == duty:
            return {"value": row["speed_m_s"], "grade": row["grade"], "reference": row["reference"],
                    "segments": row["segments"], "interpolated": False, "between": None}
    for lo, hi in zip(rows, rows[1:]):
        if lo["duty"] < duty < hi["duty"]:
            w = (duty - lo["duty"]) / (hi["duty"] - lo["duty"])
            reference = lo["reference"] if lo["reference"] == hi["reference"] else \
                "%s/%s" % (lo["reference"], hi["reference"])
            return {"value": round(lo["speed_m_s"] + w * (hi["speed_m_s"] - lo["speed_m_s"]), 3), "grade": "모델",
                    "reference": reference, "segments": None, "interpolated": True,
                    "between": [lo["duty"], hi["duty"]]}
    return None


def _worst(*grades: str) -> str:
    return max(grades, key=lambda g: GRADE_ORDER.index(g) if g in GRADE_ORDER else len(GRADE_ORDER))


def model_distance(v: float, lag: float, coast: float, drive_s: float) -> float:
    return v * (drive_s - lag) + coast


def no_correction(factor: float = 1.0) -> dict:
    return {"factor": factor, "samples": 0, "from_duty": None, "exact": False}


def direction_of(value) -> str | None:
    """'forward' (also for a request without one) or 'backward'; None for anything else."""
    if value is None:
        return "forward"
    return value if isinstance(value, str) and value in DIRECTIONS else None


def distance_limit(direction: str) -> float:
    return DISTANCE_MAX_BACKWARD_M if direction == "backward" else DISTANCE_MAX_M


def plan(table: dict, terrain, duty, distance_m, factor: float = 1.0, correction=None, direction=None) -> dict:
    """The plan of one move (pure). 'problem' is None when it may run; the numbers are given either way.
    duty: an integer % within the terrain's table range, None for the terrain's default. correction: a callable
    (terrain key, duty, direction) -> the correction of that duty and direction (CorrectionStore.correction); without
    it 'factor' applies. direction: 'forward' (None) or 'backward' (negative throttle, at most DISTANCE_MAX_BACKWARD_M)."""
    way = direction_of(direction)
    if way is None:
        return {"ok": False, "problem": "distance_direction_invalid"}
    t = terrain_of(table, terrain)
    if t is None:
        return {"ok": False, "problem": "distance_terrain_unknown", "direction": way}
    lo, hi = duty_range(t)
    span = {"terrain": t["key"], "direction": way, "duty_min": lo, "duty_max": hi, "default_duty": default_duty(t),
            "duties": [r["duty"] for r in t["speeds"]], "distance_min_m": DISTANCE_MIN_M,
            "distance_limit_m": distance_limit(way)}
    if duty is None:
        duty = default_duty(t)
    if type(duty) is not int:
        return {"ok": False, "problem": "distance_duty_invalid", **span}
    if not lo <= duty <= hi:
        return {"ok": False, "problem": "distance_duty_out_of_range", "duty": duty, **span}
    if isinstance(distance_m, bool) or not isinstance(distance_m, (int, float)) or not math.isfinite(distance_m):
        return {"ok": False, "problem": "distance_invalid", "duty": duty, **span}
    speed = speed_at(t, duty)
    v = speed["value"]
    lag = start_lag_s(table)
    coast, coast_grade = coast_m(table, duty, v)
    lag_grade = _lag_point(table)["grade"] if duty == 100 else "가정"
    corr = dict(correction(t["key"], duty, way)) if correction is not None else no_correction(float(factor))
    corr["direction"] = way
    factor = min(FACTOR_MAX, max(FACTOR_MIN, float(corr["factor"])))
    corr["factor"] = round(factor, 3)
    drive_s = lag + (float(distance_m) / factor - coast) / v
    min_m = factor * model_distance(v, lag, coast, DRIVE_MIN_S)
    limit = distance_limit(way)
    max_m = min(limit, factor * model_distance(v, lag, coast, DRIVE_MAX_S))
    problem = None
    if not DISTANCE_MIN_M <= distance_m <= limit:
        problem = "distance_out_of_range_backward" if way == "backward" else "distance_out_of_range"
    elif drive_s < DRIVE_MIN_S:
        problem = "distance_too_short_for_duty"
    elif drive_s > DRIVE_MAX_S:
        problem = "distance_too_long_for_duty"
    return {
        "ok": problem is None, "problem": problem,
        **span, "terrain_label": t["label"], "day": t["day"],
        "duty": duty, "throttle": -duty if way == "backward" else duty, "pwm": duty_pwm(duty),
        "distance_m": round(float(distance_m), 3),
        "drive_s": round(drive_s, 3), "expected_m": round(factor * model_distance(v, lag, coast, drive_s), 3),
        "min_distance_m": round(min_m, 2), "max_distance_m": round(max_m, 2),
        "factor": round(factor, 3), "correction": corr,
        "speed": speed,
        "lag": {"value": lag, "grade": lag_grade},
        "coast": {"value": coast, "grade": coast_grade},
        # the table was driven forward only: backward it is an assumption
        "grade": _worst(speed["grade"], lag_grade, coast_grade, *(("가정",) if way == "backward" else ())),
        "source": "2026-10 기록(조향 장치 전 차, ESP8266): %s %s%s" % (
            t["label"], t["day"], " · 뒤로는 앞으로 잰 표를 그대로 씀(가정)" if way == "backward" else ""),
    }


def table_view(table: dict) -> dict:
    """What the page needs to offer the choices: the table rows (measured duties) and the duty range of each terrain."""
    return {
        "version": table["version"], "title": table["title"], "vehicle": table["source"]["vehicle"],
        "not_measured": table["source"]["not_measured"],
        "terrains": [{"key": t["key"], "label": t["label"], "day": t["day"], "note": t["note"],
                      "duty_min": duty_range(t)[0], "duty_max": duty_range(t)[1], "default_duty": default_duty(t),
                      "duties": [{k: r[k] for k in ("duty", "speed_m_s", "grade", "reference", "segments")}
                                 for r in t["speeds"]]} for t in table["terrains"]],
        "limits": {"distance_min_m": DISTANCE_MIN_M, "distance_max_m": DISTANCE_MAX_M,
                   "distance_max_backward_m": DISTANCE_MAX_BACKWARD_M, "directions": list(DIRECTIONS),
                   "distance_default_m": DISTANCE_DEFAULT_M, "drive_min_s": DRIVE_MIN_S, "drive_max_s": DRIVE_MAX_S,
                   "factor_min": FACTOR_MIN, "factor_max": FACTOR_MAX, "hold_timeout_s": HOLD_TIMEOUT_S},
        "start_lag_s": start_lag_s(table),
    }


class CorrectionStore:
    """Correction factors from the operator's tape measurements per terrain, direction and duty (a small JSON file,
    like the tuning log). The file format is the one of 2026-11-17: a list of samples per terrain, each with the duty it
    was taken at; the factor of a duty uses only that duty's samples (2026-11-19). A backward sample carries
    "direction": "backward"; a sample without a direction is a forward one (the files of 2026-11-17/19). A sample
    without an integer duty is kept in the file but not used. Each direction and duty keeps its last SAMPLES_KEEP
    samples."""

    def __init__(self, path):
        self.path = Path(path)
        self._lock = threading.Lock()

    def _read(self) -> dict:
        try:
            data = json.loads(self.path.read_bytes().decode("utf-8"))
            if data.get("schema") == STORE_SCHEMA and isinstance(data.get("terrains"), dict):
                return data
        except (OSError, ValueError):
            pass
        return {"schema": STORE_SCHEMA, "terrains": {}}

    def _write(self, data: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.path.with_suffix(".tmp")
        temp.write_bytes((json.dumps(data, ensure_ascii=False, indent=1, allow_nan=False) + "\n").encode("utf-8"))
        os.replace(temp, self.path)

    @staticmethod
    def _factor(samples: list) -> float:
        ratios = [s["ratio"] for s in samples[-FACTOR_SAMPLES:]]
        if not ratios:
            return 1.0
        return round(min(FACTOR_MAX, max(FACTOR_MIN, statistics.median(ratios))), 3)

    @staticmethod
    def _duty_of(sample) -> int | None:
        duty = sample.get("duty") if isinstance(sample, dict) else None
        return duty if type(duty) is int else None

    @staticmethod
    def _direction_of(sample):
        """'forward' (also without a direction), 'backward', or None for a damaged direction (not used)."""
        value = sample.get("direction") if isinstance(sample, dict) else None
        return "forward" if value is None else value if value in DIRECTIONS else None

    @classmethod
    def _by_duty(cls, samples: list, direction: str = "forward") -> dict:
        """The usable samples of one terrain and direction by duty (an integer duty and a finite positive ratio)."""
        out = {}
        for s in samples:
            ratio = s.get("ratio") if isinstance(s, dict) else None
            if cls._duty_of(s) is None or isinstance(ratio, bool) or not isinstance(ratio, (int, float)) \
                    or not math.isfinite(ratio) or ratio <= 0 or cls._direction_of(s) != direction:
                continue
            out.setdefault(s["duty"], []).append(s)
        return out

    @classmethod
    def correction_of(cls, samples: list, duty, direction: str = "forward") -> dict:
        """The correction at a duty in one direction: its own samples, else the nearest measured duty's of the same
        direction (a tie goes to the duty with more samples, then the lower one), else 1.0. from_duty names the duty
        whose samples were used. The other direction's samples are never borrowed."""
        if direction not in DIRECTIONS:
            return no_correction()
        by = cls._by_duty(samples, direction)
        if type(duty) is not int or not by:
            return no_correction()
        source = min(by, key=lambda d: (abs(d - duty), -len(by[d]), d))
        return {"factor": cls._factor(by[source]), "samples": len(by[source]), "from_duty": source,
                "exact": source == duty}

    def _samples(self, terrain) -> list:
        entry = self._read()["terrains"].get(terrain) if isinstance(terrain, str) else None
        return entry.get("samples", []) if isinstance(entry, dict) else []

    def correction(self, terrain, duty, direction: str = "forward") -> dict:
        with self._lock:
            samples = self._samples(terrain)
        return self.correction_of(samples, duty, direction)

    def factor(self, terrain, duty, direction: str = "forward") -> float:
        return self.correction(terrain, duty, direction)["factor"]

    def view(self) -> dict:
        with self._lock:
            data = self._read()
        out = {}
        for key, entry in data["terrains"].items():
            samples = entry.get("samples", []) if isinstance(entry, dict) else []
            out[key] = {"samples": len(samples), "last": samples[-FACTOR_SAMPLES:],
                        "duties": [{"duty": duty, "factor": self._factor(rows), "samples": len(rows)}
                                   for duty, rows in sorted(self._by_duty(samples).items())],
                        "backward_duties": [{"duty": duty, "factor": self._factor(rows), "samples": len(rows)}
                                            for duty, rows in sorted(self._by_duty(samples, "backward").items())]}
        return out

    def add(self, terrain: str, sample: dict) -> dict:
        with self._lock:
            data = self._read()
            entry = data["terrains"].setdefault(terrain, {"samples": []})
            samples = entry.get("samples", []) + [sample]
            count, keep = {}, []
            for s in reversed(samples):                 # the last SAMPLES_KEEP of each direction and duty
                group = (self._direction_of(s), self._duty_of(s))
                count[group] = count.get(group, 0) + 1
                if count[group] <= SAMPLES_KEEP:
                    keep.append(s)
            entry["samples"] = keep[::-1]
            self._write(data)
        return self.view()

    def reset(self, terrain: str | None = None, direction: str | None = None) -> dict:
        """Forget the samples of one terrain (all of them, or only one direction's), or of every terrain."""
        with self._lock:
            data = self._read()
            if terrain is None:
                data["terrains"] = {}
            elif direction is None:
                data["terrains"].pop(terrain, None)
            else:
                entry = data["terrains"].get(terrain)
                if isinstance(entry, dict):
                    entry["samples"] = [s for s in entry.get("samples", []) if self._direction_of(s) != direction]
                    if not entry["samples"]:
                        data["terrains"].pop(terrain, None)
            self._write(data)
        return self.view()


class MoveRunner:
    """One distance move at a time. request(payload) -> dict is the control-socket client (rover_api.request).
    threaded=False: the caller runs step(now) itself (tests with an injected clock)."""

    def __init__(self, request, clock=time.monotonic, threaded=True):
        self.request = request
        self.clock = clock
        self.threaded = threaded
        self.lock = threading.RLock()
        self.wakeup = threading.Event()
        self.move = None
        self.moves = 0
        self.thread = None

    # ---- state ------------------------------------------------------------------------------------------------
    def busy(self) -> bool:
        """True while the server owns the lease chain (driving, or finished and not taken back yet)."""
        with self.lock:
            m = self.move
            return m is not None and (m["state"] == "driving" or (m["state"] == "finished" and not m["taken"]))

    def view(self, session=None, now=None) -> dict | None:
        with self.lock:
            m = self.move
            if m is None or (session is not None and m["session"] != session):
                return None
            now = self.clock() if now is None else now
            end = m["stopped_at"] if m["stopped_at"] is not None else now
            elapsed = max(0.0, end - m["started_at"])
            p = m["plan"]
            out = {"id": m["id"], "state": m["state"], "reason": m["reason"], "terrain": p["terrain"],
                   "direction": p.get("direction", "forward"),
                   "duty": p["duty"], "distance_m": p["distance_m"], "planned_s": p["drive_s"],
                   "elapsed_s": round(elapsed, 3), "expected_m": p["expected_m"], "factor": p["factor"],
                   "taken": m["taken"], "feedback_open": m["state"] in ("finished", "closed") and
                   m["reason"] == "planned" and not m["feedback_done"]}
            if m["stopped_at"] is not None and elapsed >= DRIVE_MIN_S:
                out["expected_for_elapsed_m"] = round(p["factor"] * self._model(m, elapsed), 3)
            return out

    @staticmethod
    def _model(m, drive_s):
        p = m["plan"]
        return model_distance(p["speed"]["value"], p["lag"]["value"], p["coast"]["value"], drive_s)

    # ---- requests from the page ---------------------------------------------------------------------------------
    def start(self, session: str, the_plan: dict, chain: dict, steer_mrad: int) -> dict:
        with self.lock:
            if self.busy():
                return {"ok": False, "error": "distance_move_running"}
            m = {"id": self.moves + 1, "session": session, "plan": the_plan, "chain": dict(chain),
                 "steer_mrad": steer_mrad, "state": "driving", "reason": None, "started_at": None,
                 "stopped_at": None, "last_hold": None, "release": False, "abort": None, "next_send": None,
                 "taken": False, "feedback_done": False, "finished_at": None, "last_keepalive": None, "error": None}
            reply = self._drive(m, the_plan["throttle"])
            if not reply.get("ok"):
                return {"ok": False, "error": reply.get("error") or "distance_drive_refused"}
            self.moves += 1
            now = self.clock()
            m.update(started_at=now, last_hold=now, next_send=now + SEND_PERIOD_S)
            m["end_at"] = now + the_plan["drive_s"]
            self.move = m
            if self.threaded:
                self.thread = threading.Thread(target=self._loop, args=(m["id"],), name="drive-distance", daemon=True)
                self.thread.start()
            return {"ok": True, "move": self.view(session, now)}

    def hold(self, session: str, move_id) -> dict:
        with self.lock:
            m = self._own(session, move_id)
            if m is None:
                return {"ok": False, "error": "distance_move_unknown"}
            if m["state"] == "driving":
                m["last_hold"] = self.clock()
            return self._reply(m)

    def release(self, session: str, move_id) -> dict:
        with self.lock:
            m = self._own(session, move_id)
            if m is None:
                return {"ok": False, "error": "distance_move_unknown"}
            if m["state"] == "driving":
                m["release"] = True
                self._step_locked(self.clock())
            return self._reply(m)

    def abort(self, reason: str = "operator_stop") -> None:
        """STOP (the route sends the bridge's stop op itself, which ends the lease): no further request."""
        with self.lock:
            m = self.move
            if m is not None and (m["state"] == "driving" or (m["state"] == "finished" and not m["taken"])):
                if m["state"] == "driving":
                    m["stopped_at"] = self.clock()
                m["state"], m["reason"] = "stopped", m["reason"] or reason
        self.wakeup.set()

    def feedback(self, session: str, move_id, actual_m, store: CorrectionStore) -> dict:
        with self.lock:
            m = self._own(session, move_id)
            if m is None:
                return {"ok": False, "error": "distance_move_unknown"}
            if not (m["state"] in ("finished", "closed") and m["reason"] == "planned"):
                return {"ok": False, "error": "distance_feedback_needs_full_move"}
            if m["feedback_done"]:
                return {"ok": False, "error": "distance_feedback_done"}
            if isinstance(actual_m, bool) or not isinstance(actual_m, (int, float)) or not math.isfinite(actual_m) \
                    or not 0 < actual_m <= ACTUAL_MAX_M:
                return {"ok": False, "error": "distance_actual_invalid"}
            elapsed = m["stopped_at"] - m["started_at"]
            model = self._model(m, elapsed)
            p = m["plan"]
            sample = {"at": round(time.time(), 1), "ratio": round(float(actual_m) / model, 4),
                      "actual_m": round(float(actual_m), 3), "model_m": round(model, 3),
                      "drive_s": p["drive_s"], "elapsed_s": round(elapsed, 3), "duty": p["duty"],
                      "requested_m": p["distance_m"], "factor": p["factor"]}
            direction = p.get("direction", "forward")
            if direction != "forward":
                sample["direction"] = direction           # forward samples stay as in the 2026-11-17 files
            m["feedback_done"] = True
        view = store.add(p["terrain"], sample)
        return {"ok": True, "sample": sample, "corrections": view,
                "correction": store.correction(p["terrain"], p["duty"], direction)}

    def _own(self, session, move_id):
        m = self.move
        return m if m is not None and m["session"] == session and m["id"] == move_id else None

    def _reply(self, m) -> dict:
        out = {"ok": True, "move": self.view(m["session"])}
        if m["state"] in ("finished", "closed") and not m["taken"]:
            # The page takes the lease chain back now (it sends its own keepalives again).
            m["taken"] = True
            m["state"] = "closed"
            out["chain"] = dict(m["chain"])
            out["move"] = self.view(m["session"])
        elif m["state"] in ("failed", "stopped"):
            out["lease_lost"] = True
        self.wakeup.set()
        return out

    # ---- the loop -----------------------------------------------------------------------------------------------
    def _send(self, m, payload) -> dict:
        reply = self.request(dict(payload, owner=m["session"]))
        if isinstance(reply, dict) and reply.get("ok") and isinstance(reply.get("ticket"), str):
            m["chain"] = {"lease": reply.get("lease", m["chain"]["lease"]), "ticket": reply["ticket"],
                          "seq": reply.get("seq", m["chain"]["seq"] + 1)}
        return reply if isinstance(reply, dict) else {"ok": False, "error": "distance_bad_reply"}

    def _drive(self, m, throttle) -> dict:
        c = m["chain"]
        return self._send(m, {"op": "drive", "lease": c["lease"], "ticket": c["ticket"], "seq": c["seq"] + 1,
                              "deadman": True, "throttle": throttle, "steer_mrad": m["steer_mrad"]})

    def _fail(self, m, now, error):
        m["state"], m["reason"], m["error"] = "failed", "bridge_error", error
        if m["stopped_at"] is None:
            m["stopped_at"] = now
        self.request({"op": "stop", "reason": "distance_move_failed"})

    def step(self, now=None):
        with self.lock:
            return self._step_locked(self.clock() if now is None else now)

    def _step_locked(self, now):
        """One step; returns the next time it is due, or None when nothing more will happen."""
        m = self.move
        if m is None:
            return None
        if m["state"] == "driving":
            reason = "released" if m["release"] else "heartbeat_lost" if now - m["last_hold"] > HOLD_TIMEOUT_S \
                else "planned" if now >= m["end_at"] else None
            if reason:
                reply = self._drive(m, 0)
                if not reply.get("ok"):
                    self._fail(m, now, reply.get("error"))
                    return None
                m.update(state="finished", reason=reason, stopped_at=self.clock(), finished_at=self.clock(),
                         last_keepalive=self.clock())
                return self.clock() + KEEPALIVE_S
            if now >= m["next_send"]:
                reply = self._drive(m, m["plan"]["throttle"])
                if not reply.get("ok"):
                    self._fail(m, now, reply.get("error"))
                    return None
                m["next_send"] = max(m["next_send"] + SEND_PERIOD_S, now + SEND_PERIOD_S / 2)
            return min(m["next_send"], m["end_at"], m["last_hold"] + HOLD_TIMEOUT_S + 0.01)
        if m["state"] == "finished" and not m["taken"]:
            if now - m["finished_at"] > TAKE_TIMEOUT_S:
                m["state"], m["reason"] = "stopped", m["reason"] + "+unclaimed"
                self.request({"op": "stop", "reason": "distance_move_unclaimed"})
                return None
            if now - m["last_keepalive"] >= KEEPALIVE_S:
                c = m["chain"]
                reply = self._send(m, {"op": "keepalive", "lease": c["lease"], "ticket": c["ticket"],
                                       "seq": c["seq"] + 1})
                if not reply.get("ok"):
                    m["state"] = "failed"
                    m["error"] = reply.get("error")
                    return None
                m["last_keepalive"] = now
            return min(m["last_keepalive"] + KEEPALIVE_S, m["finished_at"] + TAKE_TIMEOUT_S + 0.01)
        return None

    def _loop(self, move_id):
        while True:
            with self.lock:
                if self.move is None or self.move["id"] != move_id:
                    return                         # a newer move has its own loop
                due = self._step_locked(self.clock())
            if due is None:
                return
            self.wakeup.wait(max(0.0, due - self.clock()))
            self.wakeup.clear()
