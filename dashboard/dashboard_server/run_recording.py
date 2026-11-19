"""Recordings of an autonomy run: one recording per driven stretch, its record, and a keeper (2026-11-20).

User request of 2026-11-20: a run that stops must be recorded up to that place, a start again after a pause must begin a
new recording, the recording must say later that the start-again was used, and the driving time must show.

One recording = one stretch: from a go (the first one or one after a pause) to the place the vehicle stops, whatever
stops it. The page starts the recording right before the go and stops it when the run does not drive any more
(static/steer/auto-panel.mjs). This module is the server's part:

- The record of a stretch, written next to the recorder's files as <archive>/<recording_id>/autonomy_segment.json when
  the recording starts and written again when it stops (schema SCHEMA below; UTF-8, LF, written to a temporary file and
  renamed). The recorder's own files are not touched (its metadata.json is one of the files whose hash the camera
  post-processing checks), and the recorder itself is not changed. When the folder cannot be written the record goes to
  the fallback folder (<dashboard>/runtime/autonomy_segments/<recording_id>.json). No GNSS position is read or written.
- The keeper: while a recording that was started with the tag "autonomy" is running, the executor's status is read
  every KEEPER_PERIOD_S. When the run has not been driving for NOT_DRIVING_STOP_S (after its go), for
  NEVER_DROVE_STOP_S (the go never came) or the executor has not answered for UNREACHABLE_STOP_S, the server stops the
  recording, so a page that was closed or reloaded does not leave it running. A recording the page already stopped needs
  nothing; a recording with the tag "manual" is never looked at.

Driving time and distance come from the executor's run metrics (status.metrics time_s / distance_m: ticks with a drive
command only, so standing still does not count; simulation repository tools/row_executor/run_metrics.py). The record of
a stretch keeps the run's totals at its start and at its end; the stretch is the difference. An executor without the
metrics: the wall clock from the go to the stop (time_source says which).

Standard library only.
"""
from __future__ import annotations

import json
import math
import os
import re
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

SCHEMA = "ubts.autonomy_segment.v1"
FILE_NAME = "autonomy_segment.json"
KEEPER_PERIOD_S = 2.0
NOT_DRIVING_STOP_S = 5.0        # after its go the run has not been driving for this long: the server stops the recording
NEVER_DROVE_STOP_S = 15.0       # the recording runs and the run never drove (the go did not come)
UNREACHABLE_STOP_S = 15.0       # the executor does not answer
STOP_RETRY_S = 10.0             # after a stop the recorder refused
RECOVER_AFTER_S = 3.0           # after the server start: is a recording of a run already running?
PREVIOUS_SCAN = 40              # recording folders looked at for the stretch before (newest first)
SAME_PLACE_M = 0.5              # a stretch continues the one before when the run's distance has not changed by more
SAFE_ID = re.compile(r"[A-Za-z0-9_.-]{1,128}")

# Why a run stopped, in the words of the autonomy tab (the same tables as static/steer/auto-model.mjs STOP_DETAIL and
# STOP_REASON; tests/test_run_recording_words.mjs compares them).
STOP_DETAIL = {
    "operator_stop": "STOP 누름", "operator_link_lost": "화면 연결 끊김", "row_end": "열 끝 도착", "lease_lost": "조종 권한 거둠",
    "stale_lidar": "LiDAR 입력 끊김", "stale_imu": "IMU 입력 끊김", "speed_cap_below_min": "속도 제한이 최저 속도보다 낮음",
    "overspeed": "과속", "prediction_limit": "예측 한도 도달", "obstacle_ahead": "앞 장애물", "stale_repeat_limit": "잠시 멈춤 반복",
    "gnss_geofence": "GPS 범위 벗어남", "bridge_error": "브리지 오류", "bridge_latched": "브리지 멈춤 상태",
    "bridge_arm_failed": "조종 권한 받기 실패",
}
STOP_REASON = {
    "ROW_END": "열 끝 도착", "OBSTACLE": "열 안 장애물", "END_STRUCTURE": "열 끝 구조물",
    "PERCEPTION_OR_LOCALISATION": "센서·위치 추정 부족", "CONTROL_AUTHORITY": "조향·구동 응답 부족",
    "OPERATOR_OR_BACKSTOP": "STOP 또는 거리 한도", "OPERATOR_LINK_LOST": "화면 연결 끊김",
    "SYSTEM_FAULT": "시스템 오류", "UNCLASSIFIED": "이유 모름",
}
NOT_STARTED = "출발하지 않음"


def _num(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


def _iso(epoch):
    return datetime.fromtimestamp(epoch, timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def stop_words(category, detail):
    """The stop reason in words: the known detail, else the category (static/steer/auto-model.mjs stopWords)."""
    if category == "ROW_END":
        return STOP_REASON["ROW_END"]
    if isinstance(detail, str) and detail in STOP_DETAIL:
        return STOP_DETAIL[detail]
    if isinstance(category, str) and category:
        return STOP_REASON.get(category) or f"기타({category[:48]})"
    return None


def sample(raw):
    """What a stretch needs of one executor status (GET /api/status of the executor), or None."""
    if not isinstance(raw, dict):
        return None
    row = raw.get("single_row") if isinstance(raw.get("single_row"), dict) else {}
    link = raw.get("operator_link") if isinstance(raw.get("operator_link"), dict) else {}
    speed = raw.get("speed") if isinstance(raw.get("speed"), dict) else {}
    gnss = raw.get("gnss") if isinstance(raw.get("gnss"), dict) else {}
    run = raw.get("run") if isinstance(raw.get("run"), int) and not isinstance(raw.get("run"), bool) else None
    metrics = raw.get("metrics") if isinstance(raw.get("metrics"), dict) else None
    if metrics is not None and run is not None and metrics.get("run_id") not in (None, run):
        metrics = None                       # the numbers of another run
    category = raw.get("stop_reason") or row.get("stop_category")
    detail = raw.get("stop_detail")
    distance = _num(row.get("distance_m"))
    if distance is None and metrics is not None:
        distance = _num(metrics.get("distance_m"))
    state = raw.get("run_state") if isinstance(raw.get("run_state"), str) else None
    return {
        "run": run, "run_state": state,
        "reason": category if isinstance(category, str) else None, "detail": detail[:80] if isinstance(detail, str) else None,
        "time_s": _num(metrics.get("time_s")) if metrics is not None else None, "distance_m": distance,
        "go_active": link.get("go_active") is True,
        "gnss_mode": gnss.get("mode") if isinstance(gnss.get("mode"), str) else None,
        "target_speed_m_s": _num(speed.get("target_mps")) if _num(speed.get("target_mps")) is not None else _num(row.get("target_speed_m_s")),
    }


def _stop_of(view):
    """{run_state, reason, detail, text} of a sample that does not drive."""
    return {"run_state": view.get("run_state"), "reason": view.get("reason"), "detail": view.get("detail"),
            "text": stop_words(view.get("reason"), view.get("detail"))}


def _write_atomic(path: Path, record: dict) -> None:
    data = (json.dumps(record, ensure_ascii=False, allow_nan=False, indent=2) + "\n").encode("utf-8")
    tmp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
    with open(tmp, "wb") as handle:
        handle.write(data)
        handle.flush()
        try:
            os.fsync(handle.fileno())
        except OSError:
            pass
    os.replace(tmp, path)


def _read(path: Path):
    try:
        if path.stat().st_size > 64 * 1024:
            return None
        value = json.loads(path.read_bytes().decode("utf-8"))
    except (OSError, ValueError, UnicodeDecodeError):
        return None
    return value if isinstance(value, dict) and value.get("schema") == SCHEMA else None


def list_fields(record):
    """The fields of a record that the recording list shows."""
    end = record.get("end") if isinstance(record.get("end"), dict) else None
    previous = record.get("previous") if isinstance(record.get("previous"), dict) else None
    return {
        "run": record.get("run"), "segment": record.get("segment"), "resumed": record.get("resumed"),
        "ended": end is not None, "end_text": end.get("text") if end else None, "stopped_by": end.get("stopped_by") if end else None,
        "drive_time_s": record.get("drive_time_s"), "distance_m": record.get("distance_m"), "time_source": record.get("time_source"),
        "previous_recording_id": previous.get("recording_id") if previous else None,
        "previous_stop_text": (previous.get("stop") or {}).get("text") if previous else None,
    }


class _Active:
    """The stretch whose recording is running."""

    def __init__(self, record, now):
        self.record = record
        self.id = record["recording_id"]
        self.since = now                     # steady clock
        self.drove = False
        self.not_driving_since = None
        self.unknown_since = None
        self.end_sample = None               # the first status of the run after it stopped driving
        self.end_epoch = None
        self.last = None                     # the newest status of the run
        self.retry_at = None
        self.closing = False


class RunRecordings:
    """The records of the stretches and the keeper. executor_raw() -> the executor's status dict or None;
    recorder_status() -> the recorder status of the page; recorder_stop() -> the result of a recorder stop."""

    def __init__(self, archive_dir, fallback_dir, executor_raw, recorder_status, recorder_stop, log=None,
                 clock=time.time, steady=time.monotonic):
        self.archive_dir = Path(archive_dir)
        self.fallback_dir = Path(fallback_dir)
        self.executor_raw, self.recorder_status, self.recorder_stop = executor_raw, recorder_status, recorder_stop
        self.log = log or (lambda *args, **kwargs: None)
        self.clock, self.steady = clock, steady
        self.lock = threading.Lock()
        self.active = None
        self.last = None                     # the record of the stretch that closed last
        self.closed = deque(maxlen=16)
        self.run_settings = {}               # {run id: {row_len_m, gnss_mode}} of the starts this server relayed
        self.wake = threading.Event()
        self.halt = threading.Event()
        self.thread = None
        self._cache = {}                     # {recording id: ((mtime_ns, size), list fields)}

    # ---- where a record lives ---------------------------------------------------------------------------------------
    def _paths(self, recording_id):
        """[the file in the recording's folder, the fallback file]; [] for an id that is not a plain name."""
        if not isinstance(recording_id, str) or not SAFE_ID.fullmatch(recording_id) or recording_id in (".", ".."):
            return []
        return [self.archive_dir / recording_id / FILE_NAME, self.fallback_dir / f"{recording_id}.json"]

    def _save(self, record):
        paths = self._paths(record.get("recording_id"))
        if not paths:
            return None
        folder_file, fallback_file = paths
        try:
            if folder_file.parent.is_dir():
                _write_atomic(folder_file, record)
                return folder_file
        except OSError as exc:
            self.log("run_recording.write_failed", path=str(folder_file), error=type(exc).__name__)
        try:
            fallback_file.parent.mkdir(parents=True, exist_ok=True)
            _write_atomic(fallback_file, record)
            return fallback_file
        except OSError as exc:
            self.log("run_recording.write_failed", path=str(fallback_file), error=type(exc).__name__)
            return None

    def read(self, recording_id):
        for path in self._paths(recording_id):
            record = _read(path)
            if record is not None:
                return record
        return None

    # ---- what the relay tells ---------------------------------------------------------------------------------------
    def run_started(self, body, reply):
        """A start of a run went through the relay: its row length and GPS mode belong to that run id."""
        run = reply.get("run_id") if isinstance(reply, dict) else None
        if not isinstance(run, int) or isinstance(run, bool):
            return
        body = body if isinstance(body, dict) else {}
        mode = reply.get("gnss_mode") if isinstance(reply.get("gnss_mode"), str) else body.get("gnss_mode")
        with self.lock:
            self.run_settings = {run: {"row_len_m": _num(body.get("row_len_m")), "gnss_mode": mode if isinstance(mode, str) else None}}

    def go_sent(self, reply):
        """A go went through the relay while a stretch is being recorded: the stretch starts now."""
        with self.lock:
            seg = self.active
            if seg is None or seg.record.get("go") is not None:
                return
            now = self.clock()
            serial = reply.get("go_serial") if isinstance(reply, dict) else None
            seg.record["go"] = {"at": _iso(now), "epoch_s": round(now, 3),
                                "serial": serial if isinstance(serial, int) and not isinstance(serial, bool) else None}
            record = dict(seg.record)
        self._save(record)

    def _read_executor(self):
        try:
            return sample(self.executor_raw())
        except Exception as exc:                 # the keeper must not die of a reader
            self.log("run_recording.executor_read_failed", error=type(exc).__name__)
            return None

    def _previous(self, view):
        """The record of the stretch before this one of the same run (memory, else the newest folders), or None. It
        must have ended where the run is now: a run number is used again after the executor starts again."""
        if view.get("run") is None:
            return None

        def fits(record):
            if record.get("run") != view["run"] or not isinstance(record.get("end"), dict):
                return False
            was, now = _num(record.get("run_distance_m")), view.get("distance_m")
            return was is None or now is None or abs(was - now) <= SAME_PLACE_M
        with self.lock:
            for record in reversed(self.closed):
                if fits(record):
                    return record
        try:
            names = sorted((name for name in os.listdir(self.archive_dir) if SAFE_ID.fullmatch(name)), reverse=True)[:PREVIOUS_SCAN]
        except OSError:
            names = []
        for name in names:
            record = self.read(name)
            if record is not None and fits(record):
                return record
        return None

    def recording_started(self, payload, result):
        """After POST /api/recordings/start: a recording with the tag "autonomy" that started is a stretch."""
        if not isinstance(payload, dict) or payload.get("tag") != "autonomy" or not isinstance(result, dict) or not result.get("ok"):
            return None
        status = result.get("status") if isinstance(result.get("status"), dict) else {}
        recording_id = result.get("recording_id") or status.get("recording_id")
        if not self._paths(recording_id):
            return None
        view = self._read_executor()
        record = self._new_record(recording_id, view)
        where = self._save(record)
        with self.lock:
            self.active = _Active(record, self.steady())
        self.wake.set()
        self.log("run_recording.start", recording_id=recording_id, run=record["run"], segment=record["segment"],
                 resumed=record["resumed"], file=str(where) if where else None)
        return record

    def _new_record(self, recording_id, view, recovered=False):
        now = self.clock()
        view = view or {}
        run = view.get("run")
        resumed = None if not view else view.get("run_state") == "paused"
        previous = self._previous(view) if resumed else None
        if resumed is False:
            index = 1
        elif previous is not None and isinstance(previous.get("segment"), int):
            index = previous["segment"] + 1
        else:
            index = None
        with self.lock:
            settings = dict(self.run_settings.get(run) or {})
        run_time = view.get("time_s")
        if run_time is None and previous is not None:
            run_time = _num(previous.get("run_drive_time_s"))       # an executor without the metrics: the clock's sum
        if run_time is None and resumed is False:
            run_time = 0.0
        record = {
            "schema": SCHEMA, "recording_id": recording_id, "run": run, "segment": index, "resumed": resumed,
            "previous": None if not resumed else {
                "recording_id": previous.get("recording_id") if previous else None,
                "stop": _stop_of(view)},
            "settings": {"row_len_m": settings.get("row_len_m"),
                         "gnss_mode": view.get("gnss_mode") or settings.get("gnss_mode"),
                         "target_speed_m_s": view.get("target_speed_m_s")},
            "start": {"at": _iso(now), "epoch_s": round(now, 3), "run_state": view.get("run_state"),
                      "run_drive_time_s": run_time, "run_distance_m": view.get("distance_m")},
            "go": None, "moved": None, "end": None,
            "drive_time_s": None, "distance_m": None, "run_drive_time_s": None, "run_distance_m": None, "time_source": None,
        }
        if recovered:
            record["recovered"] = True          # the server started while this recording was running: the start is not known
        return record

    # ---- the stop ---------------------------------------------------------------------------------------------------
    def stop(self, by="page"):
        """The stop of POST /api/recordings/stop (and of the keeper): the recorder's result. A stretch that was being
        recorded gets its end written."""
        with self.lock:
            seg = self.active
            mine = seg is not None and not seg.closing
            if mine:
                seg.closing = True
        before = self._read_executor() if mine else None       # before the recorder stops: that takes seconds
        try:
            result = self.recorder_stop()
        except Exception as exc:
            result = {"ok": False, "state": "stop_failed", "error": type(exc).__name__}
        if not isinstance(result, dict):
            result = {"ok": False, "state": "stop_failed", "error": "no_reply"}
        if mine:
            stopped = result.get("recording_id")
            if result.get("ok") and stopped in (None, "", seg.id):
                self._close(seg, before, by, str(result.get("state") or "stopped"))
            else:
                with self.lock:
                    seg.closing = False
                    seg.retry_at = self.steady() + STOP_RETRY_S
                self.log("run_recording.stop_failed", recording_id=seg.id, by=by, error=result.get("error"), state=result.get("state"))
        return result

    def _close(self, seg, before, by, recorder_state):
        now = self.clock()
        record = seg.record
        run = record.get("run")
        same = lambda view: view is not None and (run is None or view.get("run") == run)
        # the state the stretch ended in: the status now when it is still of this run and does not drive, else the first
        # one the keeper saw after the driving, else the newest of the run
        if same(before) and before.get("run_state") != "running":
            end_view = before
        else:
            end_view = seg.end_sample or (before if same(before) else None) or seg.last or {}
        totals = end_view if end_view.get("time_s") is not None or end_view.get("distance_m") is not None else (seg.last or end_view)
        start = record["start"]
        end_time, start_time = _num(totals.get("time_s")), _num(start.get("run_drive_time_s"))
        end_distance, start_distance = _num(totals.get("distance_m")), _num(start.get("run_distance_m"))
        distance = round(max(0.0, end_distance - start_distance), 3) if end_distance is not None and start_distance is not None else None
        metrics = end_time is not None and start_time is not None and end_time >= start_time
        if metrics:
            drive, source = round(end_time - start_time, 2), "executor_metrics"
        else:                                    # an executor without the run metrics: the clock from the go to the stop
            began = _num((record.get("go") or {}).get("epoch_s"))
            ended = seg.end_epoch if seg.end_epoch is not None else now
            drive = round(max(0.0, ended - began), 2) if began is not None else 0.0
            source = "wall_clock"
        stop = _stop_of(end_view) if end_view else {"run_state": None, "reason": None, "detail": None, "text": None}
        moved = bool(seg.drove or (metrics and drive > 0) or (distance or 0.0) > 0.005)
        if not moved:
            # the go never came, or the executor did not take it, or the run was stopped before it left
            if stop["run_state"] in (None, "ready", "armed", "paused"):
                stop.update(reason=None, detail="not_started")      # the reason of the pause before is not this stretch's
            stop["text"] = NOT_STARTED
            drive = 0.0
        run_time = end_time if metrics else round((start_time or 0.0) + drive, 2)
        record.update(
            moved=moved,
            end={"at": _iso(now), "epoch_s": round(now, 3), **stop, "stopped_by": by, "recorder": recorder_state},
            drive_time_s=drive, distance_m=distance, run_drive_time_s=run_time, run_distance_m=end_distance, time_source=source)
        where = self._save(dict(record))
        with self.lock:
            if self.active is seg:
                self.active = None
            self.last = dict(record)
            self.closed.append(dict(record))
        self.log("run_recording.stop", recording_id=seg.id, by=by, run=run, segment=record.get("segment"),
                 run_state=stop["run_state"], drive_time_s=drive, file=str(where) if where else None)

    # ---- the keeper -------------------------------------------------------------------------------------------------
    def recover(self):
        """After the server start: a recording of a run that is running is kept from now on."""
        with self.lock:
            if self.active is not None:
                return None
        try:
            status = self.recorder_status()
        except Exception:
            return None
        if not isinstance(status, dict) or not status.get("ok") or status.get("active") is not True or status.get("tag") != "autonomy":
            return None
        recording_id = status.get("recording_id")
        if not self._paths(recording_id):
            return None
        record = self.read(recording_id)
        if record is None or record.get("end") is not None:
            record = self._new_record(recording_id, self._read_executor(), recovered=True)
            self._save(record)
        with self.lock:
            if self.active is None:
                self.active = _Active(record, self.steady())
        self.wake.set()
        self.log("run_recording.recovered", recording_id=recording_id)
        return record

    def tick(self):
        """One look of the keeper. Returns what it found: idle, driving, waiting, unreachable, stopped, gone, failed."""
        with self.lock:
            seg = self.active
            if seg is None or seg.closing:
                return "idle"
        now = self.steady()
        view = self._read_executor()
        run = seg.record.get("run")
        with self.lock:
            if view is None:
                seg.unknown_since = seg.unknown_since if seg.unknown_since is not None else now
                waited, limit, found = now - seg.unknown_since, UNREACHABLE_STOP_S, "unreachable"
            else:
                seg.unknown_since = None
                of_run = run is None or view.get("run") == run
                driving = of_run and (view.get("run_state") == "running" or view.get("go_active"))
                if of_run:
                    seg.last = view
                if driving:
                    if view.get("run_state") == "running":
                        seg.drove = True
                    seg.not_driving_since, seg.end_sample, seg.end_epoch = None, None, None
                    return "driving"
                if seg.not_driving_since is None:
                    seg.not_driving_since = now
                    seg.end_epoch = self.clock()
                    if of_run:
                        seg.end_sample = view
                waited = now - seg.not_driving_since
                # after a go the stretch is over once the run does not drive (it drove, or the executor let the
                # press go); before a go the page may still be on its way to it
                went = seg.drove or seg.record.get("go") is not None
                limit, found = (NOT_DRIVING_STOP_S if went else NEVER_DROVE_STOP_S), "waiting"
            if waited < limit or (seg.retry_at is not None and now < seg.retry_at):
                return found
        # nobody stopped it: is it still the recording that runs?
        try:
            status = self.recorder_status()
        except Exception:
            status = None
        if isinstance(status, dict) and status.get("ok") and (status.get("active") is not True or status.get("recording_id") != seg.id):
            with self.lock:
                if seg.closing:
                    return "idle"
                seg.closing = True
            self._close(seg, view, "elsewhere", str(status.get("state") or "not_active"))
            return "gone"
        result = self.stop("server")
        return "stopped" if result.get("ok") else "failed"

    def start_thread(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self._loop, name="run-recording-keeper", daemon=True)
            self.thread.start()
        return self.thread

    def close(self):
        self.halt.set()
        self.wake.set()

    def _loop(self):
        if self.halt.wait(RECOVER_AFTER_S):
            return
        try:
            self.recover()
        except Exception as exc:
            self.log("run_recording.keeper_error", error=type(exc).__name__)
        while not self.halt.is_set():
            with self.lock:
                busy = self.active is not None
            if not busy:
                self.wake.wait(60.0)
                self.wake.clear()
                continue
            try:
                self.tick()
            except Exception as exc:             # never let the keeper die: it is what stops an orphaned recording
                self.log("run_recording.keeper_error", error=type(exc).__name__)
            self.halt.wait(KEEPER_PERIOD_S)

    # ---- what the page gets -----------------------------------------------------------------------------------------
    def status_fields(self):
        """For the recorder status of the page: the stretch that is being recorded, else the one that closed last."""
        with self.lock:
            record = self.active.record if self.active is not None else self.last
            return {"autonomy_segment": json.loads(json.dumps(record))} if record else {}

    def with_segment(self, result):
        """A recorder reply for the page with the stretch record beside it (also inside its "status")."""
        if isinstance(result, dict):
            fields = self.status_fields()
            if fields:
                result = {**result, **fields}
                if isinstance(result.get("status"), dict):
                    result["status"] = {**result["status"], **fields}
        return result

    def decorate(self, listing):
        """GET /api/recordings: every recording with the tag "autonomy" gets "segment" (list_fields) when it has a record."""
        rows = listing.get("recordings") if isinstance(listing, dict) else None
        if not isinstance(rows, list):
            return listing
        for row in rows:
            if not isinstance(row, dict) or row.get("tag") != "autonomy":
                continue
            fields = self._list_fields(row.get("id"))
            if fields is not None:
                row["segment"] = fields
        return listing

    def _list_fields(self, recording_id):
        with self.lock:
            if self.active is not None and self.active.id == recording_id:
                return list_fields(self.active.record)
        for path in self._paths(recording_id):
            try:
                info = path.stat()
            except OSError:
                continue
            stamp = (info.st_mtime_ns, info.st_size)
            cached = self._cache.get(recording_id)
            if cached is not None and cached[0] == stamp:
                return cached[1]
            record = _read(path)
            if record is None:
                continue
            fields = list_fields(record)
            if len(self._cache) > 2000:
                self._cache.clear()
            self._cache[recording_id] = (stamp, fields)
            return fields
        return None
