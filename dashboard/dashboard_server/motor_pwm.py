"""Read-only motor status archives. Never opens a motor device or sends commands."""
from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
from typing import Any

MOTOR_TIMELINE_SCHEMA = "orchard.motor_pwm.timeline.v1"
MOTOR_STALE_AFTER_S = 1.5
MOTOR_REQUESTED_TOPIC = "/rover/control/requested"
MOTOR_APPLIED_TOPIC = "/rover/control/applied"
MOTOR_PREVIEW_MAX_FRAMES = 1800
# Declarative wiring only: checked against PIN_GPIO and WheelController constants
# by test_motor_pwm.py. The web service must not import a motor controller.
MOTOR_PIN_GPIO = {"D0": 16, "D1": 5, "D2": 4, "D3": 0, "D5": 14, "D6": 12, "D7": 13, "D8": 15}
MOTOR_DIRECTION_PINS = {"left": ("D8", "D5"), "right": ("D3", "D0")}
MOTOR_POWER_PINS = ("D1", "D2", "D6", "D7")
MOTOR_PIN_PROVENANCE = "scripts/esp_wheel_controller.py:PIN_GPIO,WheelController.DIRECTION_PINS,WheelController.POWER_PINS"


def pin_metadata() -> list[dict[str, Any]]:
    """Logical wiring follows the same constants as the actual ESP controller."""
    rows = [
        {"pin": pin, "gpio": MOTOR_PIN_GPIO[pin], "role": f"{side}_{direction}"}
        for side, pins in MOTOR_DIRECTION_PINS.items()
        for pin, direction in zip(pins, ("forward", "reverse"))
    ]
    return rows + [
        {"pin": pin, "gpio": MOTOR_PIN_GPIO[pin], "role": "driver_enable"}
        for pin in MOTOR_POWER_PINS
    ]


def _number(value: Any) -> float | None:
    if type(value) not in (int, float):
        return None
    try:
        return float(value) if math.isfinite(value) else None
    except OverflowError:
        return None


def _integer(value: Any, low: int, high: int) -> int | None:
    return value if type(value) is int and low <= value <= high else None


def sanitize_status(payload: Any, received_at: float) -> dict[str, Any] | None:
    """Whitelist known scalar fields; never archive leases, tickets or free text."""
    if not isinstance(payload, dict) or not (
        isinstance(payload.get("requested"), dict)
        or isinstance(payload.get("outputs"), dict)
    ):
        return None
    source_updated = _number(payload.get("updated_at"))
    source_age = received_at - source_updated if source_updated is not None else None
    if source_age is not None and source_age < 0:
        source_age = None
    requested = payload.get("requested")
    requested = requested if isinstance(requested, dict) else {}
    requested = {side: _integer(requested.get(side), -100, 100) for side in ("left", "right")}
    outputs = payload.get("outputs")
    outputs = outputs if isinstance(outputs, dict) else {}
    derived = {}
    for side, (forward, reverse) in MOTOR_DIRECTION_PINS.items():
        percent = requested[side]
        value = (abs(percent) * 255 + 50) // 100 if percent is not None else None
        derived[forward] = None if percent is None else value if percent > 0 else 0
        derived[reverse] = None if percent is None else value if percent < 0 else 0
    state = payload.get("state")
    firmware = payload.get("firmware")
    simulated = (
        payload.get("simulated") is True or firmware == "SIMULATED_GAZEBO_SKID"
        or payload.get("transport") == "simulation" or payload.get("mode") == "simulation"
    )
    return {
        "source_kind": "simulation" if simulated else "mcu_status",
        "received_at": received_at,
        "source_updated_at": source_updated,
        "source_age_s": round(source_age, 6) if source_age is not None else None,
        "source_stale": source_age is None or source_age > MOTOR_STALE_AFTER_S,
        "status": {
            **{key: payload.get(key) if type(payload.get(key)) is bool else None
               for key in ("ok", "connected", "armed", "moving")},
            "state": state if state in ("armed_manual", "disarmed", "fault") else None,
            "firmware": firmware if firmware in ("UBTS_WHEEL_PWM_V2", "UBTS_WHEEL_PWM_V1", "UBTS_PWM_V1", "SIMULATED_GAZEBO_SKID") else None,
            "simulated": simulated,
        },
        "requested": requested,
        "pins": {
            row["pin"]: {
                "requested_pwm_derived": derived.get(row["pin"]),
                "reported_pwm": None if simulated else _integer(outputs.get(row["pin"]), 0, 255),
                "simulated_output_pwm": _integer(outputs.get(row["pin"]), 0, 255) if simulated else None,
                "target_pwm": None,
                "reported_age_s": None,
            }
            for row in pin_metadata()
        },
    }


class MotorTimeline:
    """The recorder's existing state lock serializes all calls to this class."""

    def __init__(self, status_topic: str, max_frames: int = MOTOR_PREVIEW_MAX_FRAMES):
        self.status_topic = status_topic
        self.frames: deque[dict[str, Any]] = deque(maxlen=max(1, max_frames))
        self.frame_count = 0
        self.error: str | None = None
        self.log = None
        self.started_steady: float | None = None
        self.started_at: float | None = None
        self.last_frame_t_s: float | None = None
        self.dropped_time_frames = 0
        self.source_kinds: set[str] = set()
        self.last_received_steady: float | None = None
        self.latest: dict[str, Any] | None = None

    def start(self, root: Path, started_steady: float, started_at: float) -> None:
        self.close()
        self.frames.clear()
        self.frame_count = 0
        self.error = None
        self.started_steady = started_steady
        self.started_at = started_at
        self.last_frame_t_s = None
        self.dropped_time_frames = 0
        self.source_kinds.clear()
        try:
            self.log = (root / "motor.jsonl").open("w", encoding="utf-8")
        except OSError:
            self.error = "motor_sidecar_open_failed"

    def close(self) -> None:
        self.started_steady = None
        self.started_at = None
        if self.log is not None:
            try:
                self.log.close()
            except OSError:
                self.error = "motor_sidecar_close_failed"
            self.log = None

    def observe(self, data: Any, *, received_at: float, received_steady: float) -> None:
        if not isinstance(data, str) or len(data) > 65536:
            return
        try:
            sample = sanitize_status(json.loads(data), received_at)
        except (TypeError, ValueError, RecursionError):
            return
        if sample is None:
            return
        self.latest = sample
        self.last_received_steady = received_steady
        if self.started_steady is None or self.started_at is None or received_steady < self.started_steady:
            return
        elapsed = received_at - self.started_at
        steady_elapsed = received_steady - self.started_steady
        # Match the existing camera/LiDAR/IMU/GPS playback epoch. Keep independent
        # steady provenance, but never silently reorder or zero a regressed clock.
        if elapsed < 0 or (self.last_frame_t_s is not None and elapsed < self.last_frame_t_s):
            self.dropped_time_frames += 1
            self.error = "motor_recording_clock_regressed"
            return
        frame = {"t_s": round(elapsed, 6), "elapsed_monotonic_s": round(steady_elapsed, 6),
                 "clock_offset_s": round(elapsed - steady_elapsed, 6), **sample}
        # The file is authoritative; the bounded RAM tail is only a preview.
        if self.log is not None:
            try:
                self.log.write(json.dumps(frame, allow_nan=False, separators=(",", ":")) + "\n")
                self.log.flush()
            except OSError:
                self.error = "motor_sidecar_write_failed"
                return
        elif self.error:
            return
        self.frames.append(frame)
        self.source_kinds.add(frame["source_kind"])
        self.frame_count += 1
        self.last_frame_t_s = elapsed

    def status(self, now_steady: float) -> dict[str, Any]:
        age = None if self.last_received_steady is None else max(0.0, now_steady - self.last_received_steady)
        return {
            "schema": MOTOR_TIMELINE_SCHEMA,
            "status_topic": self.status_topic,
            "enabled": True,
            "topic_alive": age is not None and age <= MOTOR_STALE_AFTER_S,
            "last_received_age_s": round(age, 4) if age is not None else None,
            "frames": self.frame_count,
            "dropped_time_frames": self.dropped_time_frames,
            "error": self.error,
        }

    def descriptor(self) -> dict[str, Any]:
        source_kind = next(iter(self.source_kinds)) if len(self.source_kinds) == 1 else "mixed" if self.source_kinds else "unknown"
        return {
            "schema": MOTOR_TIMELINE_SCHEMA,
            "enabled": True,
            "status_topic": self.status_topic,
            "requested_topic": MOTOR_REQUESTED_TOPIC,
            "applied_topic": MOTOR_APPLIED_TOPIC,
            "source_kind": source_kind,
            "sample_hz": {"simulation": 4, "mcu_status": 2}.get(source_kind),
            "sample_hz_semantics": "nominal_publisher_rate_not_measured_archive_rate",
            "sampling": "status_snapshots_not_all_serial_edges",
            "stale_after_s": MOTOR_STALE_AFTER_S,
            "time_base": "host_receive_time_relative_to_recording_start",
            "clock_step_policy": "wall_clock_matches_sensor_playhead; backward_samples_skipped_until_caught_up; steady_elapsed_retained",
            "source_time_base": "bridge_host_unix_seconds",
            "pwm_min": 0,
            "pwm_max": 255,
            "pwm_frequency_hz": None,
            "mcu_report_age_s": None,
            "requested_pwm_semantics": "derived_from_requested_signed_percent_not_serial_transmit_or_mcu_target",
            "reported_pwm_semantics": "last_cached_mcu_report_not_voltage_or_measured_speed",
            "simulated_output_semantics": "round(max(0,directional_applied_commanded_wheel_m_s)*255/max_wheel_speed_m_s); not_mcu_report_or_measured_speed",
            "pins": pin_metadata(),
            "pin_mapping_source": MOTOR_PIN_PROVENANCE,
            "frames_file": "motor.jsonl",
            "frame_count": self.frame_count,
            "dropped_time_frames": self.dropped_time_frames,
            "preview_max_frames": self.frames.maxlen,
            "frames": list(self.frames),
            "error": self.error,
        }


def with_recorded_motor(path: Path, timeline: dict, start_s: Any, duration_s: Any) -> dict:
    """Recover early history from the sidecar, including the seek carry-in sample."""
    descriptor = timeline.get("motor")
    if not isinstance(descriptor, dict) or descriptor.get("schema") != MOTOR_TIMELINE_SCHEMA:
        return timeline  # Legacy recordings cannot reconstruct motor signals.
    source = path / "motor.jsonl"
    try:
        start = max(0.0, float(start_s)) if start_s is not None else 0.0
        end = start + max(0.1, float(duration_s)) if duration_s is not None else float("inf")
        if not math.isfinite(start) or math.isnan(end):
            return timeline
    except (TypeError, ValueError):
        return timeline
    frames, before = [], None
    malformed = 0
    try:
        with source.open(encoding="utf-8") as stream:
            for line in stream:
                try:
                    frame = json.loads(line)
                    stamp = _number(frame.get("t_s")) if isinstance(frame, dict) else None
                    if stamp is None or stamp < 0 or not isinstance(frame.get("pins"), dict):
                        raise ValueError("invalid motor frame")
                except (ValueError, TypeError, RecursionError):
                    malformed += 1
                    continue
                if stamp < start:
                    before = frame
                    continue
                if stamp > end:
                    break
                if not frames and before is not None:
                    frames.append(before)
                frames.append(frame)
        if not frames and before is not None:
            frames.append(before)
    except (OSError, UnicodeError):
        return {**timeline, "motor": {**descriptor, "sidecar_read_error": "motor_sidecar_unavailable"}}
    return {
        **timeline,
        "motor": {**descriptor, "frames": frames, "sidecar_malformed_rows": malformed},
        "sync": {**(timeline.get("sync") or {}), "segment_motor_frames": len(frames)},
    }
