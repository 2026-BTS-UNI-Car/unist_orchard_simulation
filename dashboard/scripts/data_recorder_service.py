from __future__ import annotations
import argparse
import base64
import copy
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu, LaserScan, MagneticField
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from dashboard_server.imu_processing import (
    IMU_TIMELINE_SCHEMA,
    imu_processing_status,
    imu_zero_removed,
    raw_imu_payload,
)
from dashboard_server.gps import GPS_SCHEMA, GPS_TIMELINE_SCHEMA, valid_position, finite
from dashboard_server.recording_clock import (
    clock_sample, clock_epoch, start_clock_snapshot, display_clock, sort_key,
)
from dashboard_server.motor_pwm import (
    MOTOR_APPLIED_TOPIC,
    MOTOR_REQUESTED_TOPIC,
    MOTOR_TIMELINE_SCHEMA,
    MotorTimeline,
)
from scripts.sim_video_clock import AnchorWriter,ANCHOR_FILE,TOPIC,camera_header_ns,new_contract


def iso_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def log(event: str, **fields: Any) -> None:
    print(
        json.dumps(
            {"ts": round(time.time(), 3), "event": f"data_recorder.{event}", **fields},
            ensure_ascii=False,
        ),
        flush=True,
    )


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(f"{path.suffix}.tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)


def read_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def safe_id(value: str) -> str:
    return "".join((ch for ch in value if ch.isalnum() or ch in {"_", "-", "."}))


class RecordingState:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.simulated = os.environ.get("DASHBOARD_MODE") == "simulation"
        self.sim_video_clock = None
        self.sim_video_clock_status = None
        self.lock = threading.Lock()
        self.archive_dir = Path(args.archive_dir)
        self.status_file = Path(args.status_file)
        self.active: dict[str, Any] | None = None
        self.process: subprocess.Popen | None = None
        self.preview_scan: dict[str, Any] = {}
        self.lidar_frames: list[dict[str, Any]] = []
        self.camera_frames: list[dict[str, Any]] = []
        self.next_lidar_sample_at = 0.0
        self.next_camera_sample_at = 0.0
        self.camera_pending_sample: dict[str, Any] | None = None
        self.camera_pending_event = threading.Event()
        self.camera_idle_event = threading.Event()
        self.camera_idle_event.set()
        self.camera_enqueued_count = 0
        self.camera_coalesced_frame_count = 0
        self.camera_archive_count = 0
        self.camera_last_archive_duration_ms: float | None = None
        self.imu_frames: list[dict[str, Any]] = []
        self.imu_latest_raw: dict[str, Any] = {}
        self.imu_latest_magnetic: dict[str, Any] = {}
        self.imu_last_received_monotonic: float | None = None
        self.imu_raw_received_count = 0
        self.imu_recording_raw_count = 0
        self.imu_timeline_count = 0
        self.next_imu_sample_at = 0.0
        self.gps_frames = []
        self.gps_frame_count = 0
        self.gps_valid_count = 0
        self.gps_latest = {}
        self.gps_last_at = None
        # HPPOSLLH carries copied NAV-PVT UTC, not a new receiver time epoch.
        self.gps_clock_latest = {}
        self.gps_clock_last_at = None
        self.gps_clock_epoch_watermark = None
        self.gps_write_error = None
        self.next_gps_sample_at = 0.0
        self.gps_log = None
        self.motor_timeline = MotorTimeline(args.rover_status_topic)
        self.media_queue = None
        self.finalizing = False
        self.finalizing_id = None
        self.write_status("start")

    def status(self) -> dict[str, Any]:
        with self.lock:
            active = dict(self.active or {})
            if active:
                active["duration_s"] = round(
                    time.time() - float(active.get("started_monotonic", time.time())), 2
                )
            return {
                "ok": True,
                "state": "finalizing" if self.finalizing else "recording" if active else "idle",
                "active": bool(active) or self.finalizing,
                "finalizing": self.finalizing,
                "recording_id": active.get("id") or self.finalizing_id,
                "kind": active.get("kind"),
                "include_imu": bool(active.get("include_imu", True))
                if active
                else True,
                "include_gps": bool(active.get("include_gps", True))
                if active
                else True,
                "started_at": active.get("started_at"),
                **(display_clock(self.archive_dir / safe_id(str(active.get('id') or '')),
                                 active) if active else {}),
                "duration_s": active.get("duration_s"),
                "rosbag_topics": list(active.get("topics") or []),
                "archive_dir": str(self.archive_dir),
                "archive_count": len(self.list_recordings()["recordings"]),
                "rosbag_pid": self.process.pid
                if self.process and self.process.poll() is None
                else None,
                "samples": {
                    "lidar_frames": len(self.lidar_frames),
                    "camera_frames": len(self.camera_frames),
                    "imu_frames": len(self.imu_frames),
                    "gps_frames": self.gps_frame_count,
                    "motor_frames": self.motor_timeline.frame_count,
                },
                "motor_pipeline": self.motor_timeline.status(time.monotonic()),
                "gps_pipeline": {
                    "schema": GPS_TIMELINE_SCHEMA,
                    "data_topic": "/gps/data",
                    "fix_topic": "/gps/fix",
                    "topic_alive": self.gps_last_at is not None
                    and time.monotonic() - self.gps_last_at < 3,
                    "valid": valid_position(self.gps_latest)
                    and self.gps_last_at is not None
                    and (time.monotonic() - self.gps_last_at < 3),
                    "frames": self.gps_frame_count,
                    "valid_frames": self.gps_valid_count,
                    "error": self.gps_write_error,
                },
                "imu_pipeline": {
                    "schema": IMU_TIMELINE_SCHEMA,
                    "raw_topic": self.args.imu_raw_topic,
                    "relay_topic": self.args.imu_relay_topic,
                    "processing": imu_processing_status(),
                    "mag_topic": self.args.imu_mag_topic,
                    "default_recording_enabled": True,
                    "recording_enabled": bool(active.get("include_imu", True))
                    if active
                    else False,
                    "raw_topic_alive": self.imu_last_received_monotonic is not None
                    and time.monotonic() - self.imu_last_received_monotonic < 1.0,
                    "last_raw_age_s": round(
                        time.monotonic() - self.imu_last_received_monotonic, 4
                    )
                    if self.imu_last_received_monotonic is not None
                    else None,
                    "raw_received_count": self.imu_raw_received_count,
                    "recording_raw_count": self.imu_recording_raw_count,
                    "timeline_frame_count": self.imu_timeline_count,
                    "timeline_hz": float(self.args.imu_timeline_hz),
                    "latest_raw": dict(self.imu_latest_raw),
                    "latest_magnetic": dict(self.imu_latest_magnetic),
                },
                "camera_pipeline": {
                    "native_clock_anchors": (self.sim_video_clock.status()
                        if self.sim_video_clock is not None else self.sim_video_clock_status),
                    "preview_width": int(self.args.camera_preview_width),
                    "preview_jpeg_quality": int(self.args.camera_jpeg_quality),
                    "preview_decoupled": True,
                    "latest_only": True,
                    "coalescing": "single_slot_archive_worker_and_dds_best_effort_keep_last_1",
                    "enqueued_frame_count": self.camera_enqueued_count,
                    "coalesced_frame_count": self.camera_coalesced_frame_count,
                    "archived_frame_count": self.camera_archive_count,
                    "last_duration_ms": self.camera_last_archive_duration_ms,
                },
            }

    def write_status(self, event: str) -> None:
        payload = self.status()
        payload["event"] = event
        payload["updated_at"] = time.time()
        try:
            write_json_atomic(self.status_file, payload)
        except OSError as exc:
            log("status_write_error", error=str(exc), path=str(self.status_file))

    def list_recordings(self) -> dict[str, Any]:
        rows = []
        if self.archive_dir.exists():
            for path in sorted(
                self.archive_dir.iterdir(),
                key=lambda item: item.stat().st_mtime if item.exists() else 0,
                reverse=True,
            ):
                if not path.is_dir():
                    continue
                metadata = read_json(path / "metadata.json")
                rows.append(
                    {
                        "id": metadata.get("id") or path.name,
                        "kind": metadata.get("kind") or "unknown",
                        "state": metadata.get("state") or "archived",
                        "started_at": metadata.get("started_at"),
                        **display_clock(path, metadata),
                        "finished_at": metadata.get("finished_at"),
                        "duration_s": metadata.get("duration_s"),
                    }
                )
        return {"ok": True, "recordings": sorted(rows, key=sort_key, reverse=True),
                "archive_dir": str(self.archive_dir)}

    def detail(self, recording_id: str) -> dict[str, Any]:
        path = self.archive_dir / safe_id(recording_id)
        if not path.exists():
            return {"ok": False, "error": "recording_not_found", "id": recording_id}
        metadata = read_json(path / "metadata.json")
        return {
            "ok": True,
            "recording": {
                "id": path.name,
                "metadata": metadata,
                **display_clock(path, metadata),
                "preview_scan": read_json(path / "preview_scan.json"),
                "timeline_summary": self.timeline_summary(path),
            },
        }

    def timeline(self, recording_id: str) -> dict[str, Any]:
        path = self.archive_dir / safe_id(recording_id)
        if not path.exists():
            return {"ok": False, "error": "recording_not_found", "id": recording_id}
        timeline = read_json(path / "timeline.json")
        if not timeline:
            return {"ok": False, "error": "timeline_not_found", "id": recording_id}
        return {"ok": True, "timeline": timeline}

    def timeline_summary(self, path: Path) -> dict[str, Any]:
        timeline = read_json(path / "timeline.json")
        if not timeline:
            return {"ok": False, "lidar_frames": 0, "camera_frames": 0, "imu_frames": 0}
        return {
            "ok": bool(timeline.get("ok")),
            "duration_s": timeline.get("duration_s"),
            "lidar_frames": len(timeline.get("lidar", {}).get("frames", [])),
            "camera_frames": len(timeline.get("camera", {}).get("frames", [])),
            "imu_frames": len(timeline.get("imu", {}).get("frames", [])),
            "gps_frames": (timeline.get("gps") or {}).get("frame_count", 0),
            "motor_frames": (timeline.get("motor") or {}).get("frame_count", 0),
            "motor_enabled": bool((timeline.get("motor") or {}).get("enabled")),
            "imu_calibration_valid": bool(
                timeline.get("imu", {}).get("calibration", {}).get("valid")
                or timeline.get("imu", {})
                .get("calibration", {})
                .get("calibration", {})
                .get("valid")
            ),
            "coordinate_frame": timeline.get("coordinate_frame"),
        }

    def start(
        self, kind: str, include_imu: bool = True, include_gps: bool = True
    ) -> dict[str, Any]:
        kind = kind.strip().lower()
        if self.simulated and kind in {'camera','both'} and self.args.camera_topic!=TOPIC:
            return {'ok':False,'state':'unsupported_camera_clock',
                    'error':'sim_native_video_clock_requires_camera_image_raw'}
        if kind not in {"lidar", "camera", "both"}:
            return {
                "ok": False,
                "state": "invalid_kind",
                "error": "kind must be lidar, camera, or both",
            }
        with self.lock:
            if self.finalizing:
                return {"ok": False, "state": "finalizing", "error": "recording_finalization_in_progress",
                        "recording_id": self.finalizing_id}
            if self.active:
                already_active = True
            else:
                already_active = False
        if already_active:
            return {"ok": False, "state": "already_recording", "status": self.status()}
        with self.lock:
            if self.finalizing or self.active:
                return {"ok": False, "state": "finalizing" if self.finalizing else "already_recording",
                        "error": "recording_busy"}
            recording_id = (
                f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}_{kind}"
            )
            root = self.archive_dir / recording_id
            root.mkdir(parents=True, exist_ok=False)
            include_imu = bool(include_imu)
            include_gps = bool(include_gps)
            topics = self.topics_for(
                kind, include_imu=include_imu, include_gps=include_gps
            )
            metadata = {
                "id": recording_id,
                "kind": kind,
                "include_imu": include_imu,
                "include_gps": include_gps,
                "include_motor": True,
                "state": "recording",
                "started_at": iso_now(),
                "topics": topics,
                "archive_path": str(root),
                **({'video_timing_contract':new_contract()}
                   if self.simulated and kind in {'camera','both'} else {}),
                **({"rosbag_timebase": {
                    "domain": "ros_simulation_time",
                    "clock_topic": "/clock",
                    "record_timestamp": "latest_received_clock",
                    "waits_for_first_clock": True,
                    "message_header_stamps": "preserved",
                    "dashboard_timeline_timebase": "host_wall_elapsed_unchanged",
                }} if self.simulated else {}),
                "timeline": {
                    "sample_hz": self.args.timeline_hz,
                    "camera_preview_width": self.args.camera_preview_width,
                    "camera_preview_decoupled": True,
                    "camera_latest_only": True,
                    "max_frames": self.args.timeline_max_frames,
                    "imu_enabled": include_imu,
                    "imu_schema": IMU_TIMELINE_SCHEMA,
                    "imu_processing": imu_processing_status(),
                    "imu_timeline_hz": self.args.imu_timeline_hz,
                    "imu_max_frames": self.args.imu_timeline_max_frames,
                    "motor_enabled": True,
                    "motor_schema": MOTOR_TIMELINE_SCHEMA,
                    "motor_frames_file": "motor.jsonl",
                },
            }
            metadata['clock_quality_snapshot'] = start_clock_snapshot(
                metadata['started_at'], self.gps_clock_latest, self.gps_clock_last_at,
                now_monotonic=time.monotonic(), host_now_s=time.time())
            write_json_atomic(root / "metadata.json", metadata)
            self.preview_scan = {}
            self.lidar_frames = []
            self.camera_frames = []
            self.next_lidar_sample_at = 0.0
            self.next_camera_sample_at = 0.0
            self.camera_pending_sample = None
            self.camera_pending_event.clear()
            self.camera_idle_event.set()
            self.camera_enqueued_count = 0
            self.camera_coalesced_frame_count = 0
            self.camera_archive_count = 0
            self.camera_last_archive_duration_ms = None
            self.imu_frames = []
            self.gps_frames = []
            self.gps_frame_count = self.gps_valid_count = 0
            self.gps_write_error = None
            self.next_gps_sample_at = 0.0
            self.gps_log = (
                (root / "gps.jsonl").open("w", encoding="utf-8")
                if include_gps
                else None
            )
            self.imu_recording_raw_count = 0
            self.imu_timeline_count = 0
            self.next_imu_sample_at = 0.0
            started_epoch = time.time()
            started_steady = time.monotonic()
            self.sim_video_clock = None
            self.sim_video_clock_status = None
            if self.simulated and kind in {'camera','both'}:
                try:
                    self.sim_video_clock = AnchorWriter(root/ANCHOR_FILE,round(started_epoch*1e9))
                    self.sim_video_clock_status = self.sim_video_clock.status()
                except (OSError,ValueError) as error:
                    # Preserve raw recording even when synchronization proof
                    # cannot be captured; generated synchronized media fails.
                    self.sim_video_clock_status = dict(complete=False,error=str(error),file=ANCHOR_FILE)
                metadata['video_clock_anchors'] = self.sim_video_clock_status
                write_json_atomic(root/'metadata.json',metadata)
            self.motor_timeline.start(root, started_steady, started_epoch)
            self.active = {
                **metadata,
                "path": str(root),
                "started_monotonic": started_epoch,
                "started_steady": started_steady,
            }
            self.process = self.start_rosbag(root / "rosbag2", topics)
        log(
            "start",
            id=recording_id,
            kind=kind,
            include_imu=include_imu,
            topics=topics,
            rosbag_pid=self.process.pid if self.process else None,
        )
        self.write_status("start_recording")
        return {"ok": True, "state": "recording", "status": self.status()}

    def stop(self) -> dict[str, Any]:
        with self.lock:
            if self.finalizing:
                return {"ok": True, "state": "finalizing", "recording_id": self.finalizing_id}
            active = dict(self.active or {})
            proc = self.process
            if self.active:
                self.active["stopping"] = True
                self.finalizing = True
                self.finalizing_id = active.get('id')
        if not active:
            self.write_status("stop_idle")
            return {"ok": True, "state": "idle", "status": self.status()}
        self.camera_idle_event.wait(
            timeout=max(0.1, float(self.args.camera_archive_flush_timeout_s))
        )
        with self.lock:
            self.active = None
            self.process = None
            self.motor_timeline.close()
            if self.gps_log is not None:
                self.gps_log.close()
                self.gps_log = None
        if proc and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=self.args.rosbag_stop_timeout_s)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2.)
        # Raw header anchors remain accepted after preview state is cleared,
        # until rosbag has finalized. Flush their append worker before marking
        # the recording archived or allowing the converter to read it.
        with self.lock:
            clock_writer = self.sim_video_clock
        if clock_writer is not None:
            clock_status = clock_writer.close(timeout_s=max(.1,min(10.,float(self.args.camera_archive_flush_timeout_s))))
            with self.lock:
                self.sim_video_clock_status = clock_status
                self.sim_video_clock = None
        root = Path(str(active["path"]))
        finished_at = iso_now()
        duration_s = round(
            time.time() - float(active.get("started_monotonic", time.time())), 2
        )
        metadata = read_json(root / "metadata.json")
        if self.simulated and metadata.get('video_timing_contract'):
            metadata['video_clock_anchors'] = self.sim_video_clock_status
        metadata.update(
            {
                "state": "archived",
                "finished_at": finished_at,
                "duration_s": duration_s,
                "timeline": {
                    "lidar_frames": len(self.lidar_frames),
                    "camera_frames": len(self.camera_frames),
                    "imu_frames": len(self.imu_frames),
                    "gps_frames": self.gps_frame_count,
                    "gps_valid_frames": self.gps_valid_count,
                    "gps_enabled": bool(active.get("include_gps", True)),
                    "gps_error": self.gps_write_error,
                    "motor_enabled": True,
                    "motor_schema": MOTOR_TIMELINE_SCHEMA,
                    "motor_frames": self.motor_timeline.frame_count,
                    "motor_frames_file": "motor.jsonl",
                    "motor_error": self.motor_timeline.error,
                    "sample_hz": self.args.timeline_hz,
                    "camera_preview_width": self.args.camera_preview_width,
                    "camera_preview_decoupled": True,
                    "camera_latest_only": True,
                    "imu_enabled": bool(active.get("include_imu", True)),
                    "imu_schema": IMU_TIMELINE_SCHEMA,
                    "imu_processing": imu_processing_status(),
                    "imu_timeline_hz": self.args.imu_timeline_hz,
                },
            }
        )
        write_json_atomic(root / "metadata.json", metadata)
        write_json_atomic(
            root / "preview_scan.json",
            self.preview_scan
            or {"ok": False, "points": [], "message": "no lidar scan captured"},
        )
        write_json_atomic(
            root / "timeline.json", self.timeline_payload(active, metadata, duration_s)
        )
        if active.get('kind') in ('camera','both'):
            from scripts.recording_postprocess import enqueue
            result = enqueue(root, self.media_queue)
            if not result.get('ok'):
                log('media_queue_rejected', id=active.get('id'), error=result.get('error'))
        with self.lock:
            self.finalizing = False
            self.finalizing_id = None
        log("stop", id=active.get("id"), duration_s=duration_s)
        self.write_status("stop_recording")
        return {
            "ok": True,
            "state": "archived",
            "recording_id": active.get("id"),
            "status": self.status(),
        }

    def timeline_payload(
        self, active: dict[str, Any], metadata: dict[str, Any], duration_s: float
    ) -> dict[str, Any]:
        return {
            "ok": True,
            "version": 1,
            "id": active.get("id") or metadata.get("id"),
            "kind": active.get("kind") or metadata.get("kind"),
            "coordinate_frame": "lidar_bridge_xy_v1",
            "generated_at": iso_now(),
            "started_at": metadata.get("started_at"),
            "finished_at": metadata.get("finished_at"),
            "duration_s": duration_s,
            "sample_hz": self.args.timeline_hz,
            "lidar": {"frames": list(self.lidar_frames)},
            "camera": {
                "frames": list(self.camera_frames),
                "preview_width": int(self.args.camera_preview_width),
                "preview_jpeg_quality": int(self.args.camera_jpeg_quality),
                "preview_decoupled": True,
                "latest_only": True,
                "coalescing": "single_slot_archive_worker_and_dds_best_effort_keep_last_1",
                "enqueued_frame_count": self.camera_enqueued_count,
                "coalesced_frame_count": self.camera_coalesced_frame_count,
            },
            "imu": {
                "schema": IMU_TIMELINE_SCHEMA,
                "enabled": bool(active.get("include_imu", True)),
                "raw_topic": self.args.imu_raw_topic,
                "relay_topic": self.args.imu_relay_topic,
                "processing": imu_processing_status(),
                "mag_topic": self.args.imu_mag_topic,
                "sample_hz": float(self.args.imu_timeline_hz),
                "frames": list(self.imu_frames),
                "raw_received_count": self.imu_recording_raw_count,
            },
            "sync": {
                "time_base": "seconds_since_recording_start",
                "lidar_frames": len(self.lidar_frames),
                "camera_frames": len(self.camera_frames),
                "imu_frames": len(self.imu_frames),
                "gps_frames": self.gps_frame_count,
                "motor_frames": self.motor_timeline.frame_count,
            },
            "motor": self.motor_timeline.descriptor(),
            "gps": {
                "schema": GPS_TIMELINE_SCHEMA,
                "enabled": bool(active.get("include_gps", True)),
                "data_topic": "/gps/data",
                "fix_topic": "/gps/fix",
                "coordinate_frame": "WGS84",
                "sample_hz": 5,
                "frame_count": self.gps_frame_count,
                "valid_frames": self.gps_valid_count,
                "frames_file": "gps.jsonl",
                "frames": list(self.gps_frames),
                "error": self.gps_write_error,
                "time_base": "host_receive_time_relative_to_recording_start",
                "gps_utc_preserved": True,
            },
        }

    def start_rosbag(
        self, output_dir: Path, topics: list[str]
    ) -> subprocess.Popen | None:
        if self.args.disable_rosbag:
            return None
        output_dir.parent.mkdir(parents=True, exist_ok=True)
        log_path = output_dir.parent / "rosbag2.log"
        log_file = log_path.open("ab")
        cmd = ["ros2", "bag", "record"]
        if self.simulated:
            # Bag reception time must share the simulation epoch used by scan
            # and TF headers. JSON archive playback keeps its existing timebase.
            cmd.append("--use-sim-time")
            topics = list(dict.fromkeys([*topics, "/clock"]))
        cmd.extend(["-o", str(output_dir), *topics])
        try:
            return subprocess.Popen(
                cmd, stdout=log_file, stderr=subprocess.STDOUT, start_new_session=True
            )
        except OSError as exc:
            log("rosbag_start_error", error=str(exc), cmd=cmd)
            return None
        finally:
            log_file.close()

    def topics_for(
        self, kind: str, *, include_imu: bool = True, include_gps: bool = True
    ) -> list[str]:
        topics = [
            self.args.cmd_topic,
            self.args.rover_status_topic,
            MOTOR_REQUESTED_TOPIC,
            MOTOR_APPLIED_TOPIC,
            self.args.odom_topic,
            self.args.tf_topic,
            self.args.tf_static_topic,
        ]
        if self.simulated:
            topics.append("/clock")
        if include_imu:
            topics.extend(
                [
                    self.args.imu_raw_topic,
                    self.args.imu_relay_topic,
                    self.args.imu_mag_topic,
                    self.args.imu_driver_status_topic,
                ]
            )
        if include_gps:
            topics.extend(["/gps/fix", "/gps/data", "/gps/driver_status"])
        if kind in {"lidar", "both"}:
            topics.append(self.args.scan_topic)
        if kind in {"camera", "both"}:
            topics.extend(
                [
                    self.args.camera_topic,
                    self.args.camera_compressed_topic,
                    self.args.camera_info_topic,
                    self.args.camera_preview_info_topic,
                ]
            )
        return list(
            dict.fromkeys((topic for topic in topics if str(topic or "").strip()))
        )

    def observe_motor(self, msg: String) -> None:
        received_at, received_steady = time.time(), time.monotonic()
        with self.lock:
            if self.active and self.active.get("stopping"):
                return
            self.motor_timeline.observe(
                msg.data, received_at=received_at, received_steady=received_steady
            )

    def observe_gps(self, msg, *, status=False):
        try:
            sample = json.loads(msg.data)
            if not isinstance(sample, dict) or sample.get("schema") != GPS_SCHEMA:
                return
            now = time.monotonic()
            if status:
                if sample.get("valid") or sample.get("state") not in {
                    "stale",
                    "no_data",
                    "unavailable",
                }:
                    return
                sample.update(
                    valid=False, latitude=None, longitude=None, received_at=time.time()
                )
                sample.pop("driver", None)
            json.dumps(sample, allow_nan=False)
            received = finite(sample.get("received_at"))
            if received is None:
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            self.gps_latest = sample
            self.gps_last_at = now
            if status:
                self.gps_clock_latest = {}
                self.gps_clock_last_at = None
            elif sample.get('protocol') == 'UBX_NAV_PVT':
                incoming = clock_sample(sample)
                incoming_epoch = clock_epoch(incoming)
                previous_epoch = self.gps_clock_epoch_watermark
                if (incoming_epoch is None or previous_epoch is None
                        or incoming_epoch > previous_epoch):
                    self.gps_clock_latest = incoming
                    self.gps_clock_last_at = now
                    if incoming_epoch is not None:
                        self.gps_clock_epoch_watermark = incoming_epoch
                elif incoming_epoch < previous_epoch:
                    self.gps_clock_latest = {}
                    self.gps_clock_last_at = None
            active = self.active
            if (
                not active
                or not active.get("include_gps", True)
                or active.get("stopping")
                or (now < self.next_gps_sample_at)
            ):
                return
            self.next_gps_sample_at = now + 0.2
            started = float(active.get("started_monotonic", time.time()))
            if received < started or received > time.time() + 1:
                return
            frame = {"t_s": round(received - started, 6), "sample": sample}
            if self.gps_log is not None:
                try:
                    self.gps_log.write(
                        json.dumps(frame, allow_nan=False, separators=(",", ":")) + "\n"
                    )
                    self.gps_log.flush()
                except OSError as error:
                    self.gps_write_error = str(error)
                    return
            self.gps_frames.append(frame)
            if len(self.gps_frames) > 18000:
                del self.gps_frames[: len(self.gps_frames) - 18000]
            self.gps_frame_count += 1
            self.gps_valid_count += int(valid_position(sample))

    def observe_scan(self, msg: LaserScan) -> None:
        with self.lock:
            if not self.active or self.active.get("kind") not in {"lidar", "both"}:
                return
            started_monotonic = float(self.active.get("started_monotonic", time.time()))
        points = []
        angle = float(msg.angle_min)
        max_points = max(50, int(self.args.preview_max_points))
        stride = max(1, math.ceil(len(msg.ranges) / max_points))
        for index, distance in enumerate(msg.ranges):
            if index % stride != 0:
                angle += float(msg.angle_increment)
                continue
            r = float(distance)
            if math.isfinite(r) and max(float(msg.range_min), 0.02) <= r <= float(
                msg.range_max
            ):
                x = r * math.sin(angle)
                y = r * math.cos(angle)
                points.append(
                    {
                        "angle_deg": round((math.degrees(angle) + 360.0) % 360.0, 2),
                        "distance_m": round(r, 3),
                        "x_m": round(x, 3),
                        "y_m": round(y, 3),
                        "quality": 32,
                    }
                )
            angle += float(msg.angle_increment)
        preview = {
            "ok": True,
            "state": "archived_preview",
            "protocol": "ROS2 /scan",
            "frame_id": msg.header.frame_id or self.args.lidar_frame_id,
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "tested_at": int(time.time()),
            "points": points,
            "coordinate_frame": "lidar_bridge_xy_v1",
            "metrics": {
                "rendered_points": len(points),
                "source_points": len(msg.ranges),
                "coverage_deg": self.coverage(points),
                "range_limit_m": float(msg.range_max),
                "scan_time_s": float(msg.scan_time),
                "time_increment_s": float(msg.time_increment),
                "angle_min_rad": float(msg.angle_min),
                "angle_increment_rad": float(msg.angle_increment),
                "point_time_direction": -1.0,
            },
            "source": "archive_recorder",
        }
        now = time.time()
        frame = None
        if now >= self.next_lidar_sample_at:
            self.next_lidar_sample_at = now + self.sample_interval_s()
            frame = {
                "t_s": round(now - started_monotonic, 3),
                "frame_id": preview["frame_id"],
                "stamp": preview["stamp"],
                "points": points,
                "coordinate_frame": "lidar_bridge_xy_v1",
                "metrics": preview["metrics"],
            }
        with self.lock:
            self.preview_scan = preview
            if frame:
                self.append_limited(self.lidar_frames, frame)

    def observe_imu(self, msg: Imu) -> dict[str, Any] | None:
        """Observe sensor-native data without modifying orientation, axes or bias."""
        now_epoch = time.time()
        now_steady = time.monotonic()
        try:
            raw = raw_imu_payload(msg)
        except (TypeError, ValueError, AttributeError, OverflowError) as exc:
            log("imu_message_rejected", error=str(exc))
            return None
        with self.lock:
            self.imu_latest_raw = raw
            self.imu_last_received_monotonic = now_steady
            self.imu_raw_received_count += 1
            active = dict(self.active or {})
            recording_imu_active = bool(
                active
                and active.get("include_imu", True)
                and (not active.get("stopping"))
            )
            if recording_imu_active:
                self.imu_recording_raw_count += 1
            if recording_imu_active and now_steady >= self.next_imu_sample_at:
                self.next_imu_sample_at = now_steady + 1.0 / max(
                    0.1, float(self.args.imu_timeline_hz)
                )
                magnetic = dict(self.imu_latest_magnetic)
                magnetic_age_s = None
                if magnetic.get("received_monotonic") is not None:
                    magnetic_age_s = max(
                        0.0, now_steady - float(magnetic["received_monotonic"])
                    )
                magnetic.pop("received_monotonic", None)
                frame = {
                    "t_s": round(
                        now_epoch - float(active.get("started_monotonic", now_epoch)), 6
                    ),
                    "stamp": dict(raw["stamp"]),
                    "raw": copy.deepcopy(raw),
                    "corrected": None,
                    "magnetic": magnetic
                    if magnetic
                    and magnetic_age_s is not None
                    and (magnetic_age_s <= 0.25)
                    else {},
                    "magnetic_age_s": round(magnetic_age_s, 6)
                    if magnetic_age_s is not None
                    else None,
                    "processing": imu_processing_status(),
                }
                self.append_imu_limited(frame)
                self.imu_timeline_count += 1
            return copy.deepcopy(raw)

    def zero_imu(self) -> dict[str, Any]:
        # Retired endpoint: no I/O, state mutation or reference creation.
        return imu_zero_removed()

    def observe_magnetic(self, msg: MagneticField) -> None:
        try:
            magnetic_t = {
                "x": float(msg.magnetic_field.x),
                "y": float(msg.magnetic_field.y),
                "z": float(msg.magnetic_field.z),
            }
            if not all((math.isfinite(value) for value in magnetic_t.values())):
                return
            row = {
                "frame_id": str(msg.header.frame_id or "imu_link"),
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
                "magnetic_field_t": magnetic_t,
                "magnetic_field_uT": {
                    key: value * 1000000.0 for key, value in magnetic_t.items()
                },
                "source": "ros2_sensor_msgs_magnetic_field",
                "received_monotonic": time.monotonic(),
            }
        except (TypeError, ValueError, AttributeError):
            return
        with self.lock:
            self.imu_latest_magnetic = row

    def _reserve_camera_sample(self):
        """Atomically admit only an active, due archive-preview sample."""
        with self.lock:
            if (
                not self.active
                or self.active.get("kind") not in {"camera", "both"}
                or self.active.get("stopping")
            ):
                return
            started_monotonic = float(self.active.get("started_monotonic", time.time()))
            recording_id = str(self.active.get("id") or "")
            now = time.time()
            if now < self.next_camera_sample_at:
                return
            self.next_camera_sample_at = now + self.sample_interval_s()
            return recording_id, started_monotonic, now

    def observe_camera_raw(self, serialized: bytes) -> None:
        """Preserve native header anchors; sample preview without decoding idle."""
        # Header-only work at native camera cadence, independently of the
        # coalesced 2-Hz JPEG preview. No image payload deserialization/copy.
        with self.lock:
            clock_writer = self.sim_video_clock
        if clock_writer is not None:
            try:
                wall_ns = time.time_ns()
                steady_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
                clock_writer.submit(camera_header_ns(serialized),wall_ns,steady_ns)
            except (ValueError,AttributeError,OSError) as error:
                clock_writer.fail('sim_video_anchor_capture_failed:'+str(error))
        context = self._reserve_camera_sample()
        if context is None:
            return
        try:
            msg = deserialize_message(serialized, Image)
        except Exception as exc:
            log("camera_deserialize_error", error=str(exc))
            return
        self._enqueue_camera_sample(msg, context)

    def observe_camera(self, msg: Image) -> None:
        """Copy a due ROS image into one slot; the worker later encodes JPEG."""
        context = self._reserve_camera_sample()
        if context is not None:
            self._enqueue_camera_sample(msg, context)

    def _enqueue_camera_sample(self, msg: Image, context) -> None:
        recording_id, started_monotonic, now = context
        try:
            sample = {
                "recording_id": recording_id,
                "started_monotonic": started_monotonic,
                "received_at": now,
                "frame_id": str(msg.header.frame_id or "camera_link_optical"),
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
                "height": int(getattr(msg, "height", 0) or 0),
                "width": int(getattr(msg, "width", 0) or 0),
                "encoding": str(getattr(msg, "encoding", "") or ""),
                "data": bytes(getattr(msg, "data", b"") or b""),
            }
            if sample["height"] <= 0 or sample["width"] <= 0 or (not sample["data"]):
                raise ValueError("empty image message")
        except Exception as exc:
            log("camera_enqueue_error", error=str(exc))
            return
        with self.lock:
            if (
                not self.active
                or str(self.active.get("id") or "") != recording_id
                or self.active.get("stopping")
            ):
                return
            if self.camera_pending_sample is not None:
                self.camera_coalesced_frame_count += 1
            self.camera_pending_sample = sample
            self.camera_enqueued_count += 1
            self.camera_idle_event.clear()
            self.camera_pending_event.set()

    def camera_archive_worker(self, shutdown: threading.Event) -> None:
        while not shutdown.is_set():
            self.camera_pending_event.wait(timeout=0.1)
            if shutdown.is_set():
                break
            while not shutdown.is_set():
                with self.lock:
                    sample = self.camera_pending_sample
                    self.camera_pending_sample = None
                    if sample is None:
                        self.camera_pending_event.clear()
                        self.camera_idle_event.set()
                        break
                self.process_camera_sample(sample)
                with self.lock:
                    if self.camera_pending_sample is None:
                        self.camera_pending_event.clear()
                        self.camera_idle_event.set()
                        break

    def process_camera_sample(self, sample: dict[str, Any]) -> None:
        started = time.perf_counter()
        stamp = sample.get("stamp") or {}
        stamp_ns = int(stamp.get("sec") or 0) * 1000000000 + int(
            stamp.get("nanosec") or 0
        )
        try:
            source_frame = self.image_sample_to_bgr(sample)
            source_height, source_width = source_frame.shape[:2]
            preview_width = max(160, int(self.args.camera_preview_width))
            if source_width > preview_width:
                scale = preview_width / source_width
                preview_height = max(1, int(round(source_height * scale)))
                preview_frame = cv2.resize(
                    source_frame,
                    (preview_width, preview_height),
                    interpolation=cv2.INTER_AREA,
                )
            else:
                preview_frame = source_frame
            ok, encoded = cv2.imencode(
                ".jpg",
                preview_frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.args.camera_jpeg_quality)],
            )
            if not ok:
                return
        except Exception as exc:
            log("camera_archive_error", error=str(exc))
            return
        payload = {
            "t_s": round(
                float(sample["received_at"]) - float(sample["started_monotonic"]), 3
            ),
            "frame_id": str(sample.get("frame_id") or "camera_link_optical"),
            "stamp": {
                "sec": int(stamp.get("sec") or 0),
                "nanosec": int(stamp.get("nanosec") or 0),
            },
            "mime": "image/jpeg",
            "data": base64.b64encode(encoded.tobytes()).decode("ascii"),
            "width": int(preview_frame.shape[1]),
            "height": int(preview_frame.shape[0]),
            "source_width": int(source_width),
            "source_height": int(source_height),
        }
        with self.lock:
            active_id = str((self.active or {}).get("id") or "")
            if active_id != str(sample.get("recording_id") or ""):
                return
            self.camera_archive_count += 1
            self.camera_last_archive_duration_ms = round(
                (time.perf_counter() - started) * 1000.0, 1
            )
            self.append_limited(self.camera_frames, payload)

    def sample_interval_s(self) -> float:
        return 1.0 / max(0.1, float(self.args.timeline_hz))

    def append_limited(self, rows: list[dict[str, Any]], item: dict[str, Any]) -> None:
        rows.append(item)
        limit = max(1, int(self.args.timeline_max_frames))
        if len(rows) > limit:
            del rows[: len(rows) - limit]

    def append_imu_limited(self, item: dict[str, Any]) -> None:
        self.imu_frames.append(item)
        limit = max(1, int(self.args.imu_timeline_max_frames))
        if len(self.imu_frames) > limit:
            del self.imu_frames[: len(self.imu_frames) - limit]

    def image_sample_to_bgr(self, sample: dict[str, Any]) -> Any:
        height = int(sample.get("height") or 0)
        width = int(sample.get("width") or 0)
        encoding = str(sample.get("encoding") or "").lower()
        data = bytes(sample.get("data") or b"")
        if height <= 0 or width <= 0 or (not data):
            raise ValueError("empty image message")
        if encoding in {"bgr8", "rgb8"}:
            arr = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
            return (
                arr.copy()
                if encoding == "bgr8"
                else cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            )
        if encoding in {"bgra8", "rgba8"}:
            arr = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 4))
            code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
            return cv2.cvtColor(arr, code)
        if encoding in {"mono8", "8uc1"}:
            arr = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"unsupported image encoding: {encoding}")

    def coverage(self, points: list[dict[str, Any]]) -> float:
        if len(points) < 2:
            return 0.0
        angles = sorted((float(point["angle_deg"]) for point in points))
        gaps = [angles[i] - angles[i - 1] for i in range(1, len(angles))]
        gaps.append(angles[0] + 360.0 - angles[-1])
        return round(max(0.0, 360.0 - max(gaps)), 1)


class RecorderNode(Node):
    def __init__(self, args: argparse.Namespace, state: RecordingState):
        super().__init__("ubts_data_recorder")
        self.state = state
        self.args = args
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # A BEST_EFFORT sensor publisher cannot match a RELIABLE subscriber.
        # Latest-only previews also work with reliable physical scan publishers;
        # the independent rosbag recorder retains raw topic history.
        self.create_subscription(
            LaserScan, args.scan_topic, self.state.observe_scan, camera_qos
        )
        simulation = os.environ.get("DASHBOARD_MODE") == "simulation"
        self.create_subscription(
            Image, args.camera_topic,
            self.state.observe_camera_raw if simulation else self.state.observe_camera,
            camera_qos, raw=simulation,
        )
        self.imu_relay_publisher = self.create_publisher(
            Imu, args.imu_relay_topic, camera_qos
        )
        self.create_subscription(Imu, args.imu_raw_topic, self.observe_imu, camera_qos)
        self.create_subscription(
            MagneticField, args.imu_mag_topic, self.state.observe_magnetic, camera_qos
        )
        self.create_subscription(
            String, "/gps/data", self.state.observe_gps, camera_qos
        )
        self.create_subscription(
            String, args.rover_status_topic, self.state.observe_motor, 10
        )
        self.create_subscription(
            String,
            "/gps/driver_status",
            lambda msg: self.state.observe_gps(msg, status=True),
            10,
        )

    def observe_imu(self, msg: Imu) -> None:
        if self.state.observe_imu(msg) is not None:
            # Compatibility topic only: relay the complete message unchanged,
            # including frame, native clock, quaternion and all covariance terms.
            self.imu_relay_publisher.publish(msg)


class Handler(BaseHTTPRequestHandler):
    state: RecordingState
    protocol_version = "HTTP/1.1"

    def do_GET(self):
        if self.path == "/media-jobs":
            queue = self.state.media_queue
            self.send_json(queue.status() if queue else {"ok": False, "error": "media_queue_unavailable"})
        elif self.path == "/status":
            self.send_json(self.state.status())
        elif self.path == "/recordings":
            self.send_json(self.state.list_recordings())
        elif self.path.startswith("/recordings/"):
            raw = self.path.removeprefix("/recordings/").strip("/")
            if raw.endswith("/timeline"):
                self.send_json(
                    self.state.timeline(raw.removesuffix("/timeline").strip("/"))
                )
            else:
                self.send_json(self.state.detail(raw))
        else:
            self.send_error(404, "Not found")

    def do_POST(self):
        if self.path == "/media-jobs/action":
            payload = self.read_json()
            try:
                queue = self.state.media_queue
                if queue is None:
                    raise ValueError('media_queue_unavailable')
                result = queue.action(payload.get('recording_id'), payload.get('action'))
                self.send_json(result)
            except ValueError as error:
                self.send_json({"ok": False, "error": str(error)}, status=409)
        elif self.path == "/start":
            payload = self.read_json()
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
            self.send_json(
                self.state.start(
                    str(payload.get("kind") or ""),
                    include_imu=include_imu,
                    include_gps=include_gps,
                )
            )
        elif self.path == "/stop":
            self.send_json(self.state.stop())
        elif self.path == "/imu/zero":
            self.send_json(self.state.zero_imu(), status=410)
        else:
            self.send_error(404, "Not found")

    def read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return {}

    def send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)
        self.close_connection = True

    def log_message(self, fmt, *args):
        if os.environ.get("RECORDER_ACCESS_LOG", "0") == "1":
            super().log_message(fmt, *args)


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Archive rover LiDAR/camera drive-test data."
    )
    parser.add_argument("--host", default=os.environ.get("RECORDER_HOST", "127.0.0.1"))
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("RECORDER_PORT", "8790"))
    )
    parser.add_argument(
        "--archive-dir",
        default=os.environ.get("RECORDING_ARCHIVE_DIR", "/app/runtime/recordings"),
    )
    parser.add_argument(
        "--status-file",
        default=os.environ.get(
            "DATA_RECORDER_STATUS_FILE", "/app/runtime/data_recorder_status.json"
        ),
    )
    parser.add_argument(
        "--scan-topic", default=os.environ.get("LIDAR_SCAN_TOPIC", "/scan")
    )
    parser.add_argument(
        "--camera-topic",
        default=os.environ.get("CAMERA_IMAGE_TOPIC", "/camera/image_raw"),
    )
    parser.add_argument(
        "--camera-compressed-topic",
        default=os.environ.get("CAMERA_COMPRESSED_TOPIC", "/camera/image_jpeg"),
    )
    parser.add_argument(
        "--camera-info-topic",
        default=os.environ.get("CAMERA_INFO_TOPIC", "/camera/camera_info"),
    )
    parser.add_argument(
        "--camera-preview-info-topic",
        default=os.environ.get(
            "CAMERA_PREVIEW_INFO_TOPIC", "/camera/preview/camera_info"
        ),
    )
    parser.add_argument(
        "--cmd-topic", default=os.environ.get("ROVER_CMD_TOPIC", "/cmd_vel")
    )
    parser.add_argument(
        "--rover-status-topic",
        default=os.environ.get("ROVER_STATUS_TOPIC", "/rover/status"),
    )
    parser.add_argument("--odom-topic", default=os.environ.get("ODOM_TOPIC", "/odom"))
    parser.add_argument(
        "--imu-raw-topic", default=os.environ.get("IMU_RAW_TOPIC", "/imu/data_raw")
    )
    parser.add_argument(
        "--imu-relay-topic",
        "--imu-topic",
        dest="imu_relay_topic",
        default=os.environ.get(
            "IMU_RELAY_TOPIC", os.environ.get("IMU_TOPIC", "/imu/data")
        ),
    )
    parser.add_argument(
        "--imu-mag-topic", default=os.environ.get("IMU_MAG_TOPIC", "/imu/mag")
    )
    parser.add_argument(
        "--imu-driver-status-topic",
        default=os.environ.get("IMU_DRIVER_STATUS_TOPIC", "/orchard/imu/driver_status"),
    )
    parser.add_argument("--tf-topic", default=os.environ.get("TF_TOPIC", "/tf"))
    parser.add_argument(
        "--tf-static-topic", default=os.environ.get("TF_STATIC_TOPIC", "/tf_static")
    )
    parser.add_argument(
        "--lidar-frame-id", default=os.environ.get("LIDAR_FRAME_ID", "laser")
    )
    parser.add_argument(
        "--preview-max-points",
        type=int,
        default=int(os.environ.get("RECORDER_PREVIEW_MAX_POINTS", "1200")),
    )
    parser.add_argument(
        "--timeline-hz",
        type=float,
        default=float(os.environ.get("RECORDER_TIMELINE_HZ", "2.0")),
    )
    parser.add_argument(
        "--timeline-max-frames",
        type=int,
        default=int(os.environ.get("RECORDER_TIMELINE_MAX_FRAMES", "1800")),
    )
    parser.add_argument(
        "--imu-timeline-hz",
        type=float,
        default=float(os.environ.get("RECORDER_IMU_TIMELINE_HZ", "10.0")),
    )
    parser.add_argument(
        "--imu-timeline-max-frames",
        type=int,
        default=int(os.environ.get("RECORDER_IMU_TIMELINE_MAX_FRAMES", "18000")),
    )
    parser.add_argument(
        "--camera-preview-width",
        type=int,
        default=int(os.environ.get("RECORDER_CAMERA_PREVIEW_WIDTH", "640")),
    )
    parser.add_argument(
        "--camera-jpeg-quality",
        type=int,
        default=int(os.environ.get("RECORDER_CAMERA_JPEG_QUALITY", "68")),
    )
    parser.add_argument(
        "--camera-archive-flush-timeout-s",
        type=float,
        default=float(
            os.environ.get("RECORDER_CAMERA_ARCHIVE_FLUSH_TIMEOUT_S", "0.75")
        ),
    )
    parser.add_argument(
        "--rosbag-stop-timeout-s",
        type=float,
        default=float(os.environ.get("RECORDER_ROSBAG_STOP_TIMEOUT_S", "8.0")),
    )
    parser.add_argument(
        "--disable-rosbag",
        default=os.environ.get("RECORDER_DISABLE_ROSBAG", "0").strip().lower()
        in {"1", "true", "yes", "on"},
        action=argparse.BooleanOptionalAction,
    )
    args = parser.parse_args()
    args.imu_timeline_hz = max(0.5, min(100.0, float(args.imu_timeline_hz)))
    args.imu_timeline_max_frames = max(
        100, min(360000, int(args.imu_timeline_max_frames))
    )
    args.camera_archive_flush_timeout_s = max(
        0.1, min(3.0, args.camera_archive_flush_timeout_s)
    )
    args.camera_preview_width = max(160, min(3840, args.camera_preview_width))
    args.camera_jpeg_quality = max(40, min(95, args.camera_jpeg_quality))
    if args.imu_raw_topic == args.imu_relay_topic:
        parser.error("IMU raw and relay topics must differ")
    state = RecordingState(args)
    Handler.state = state
    server = Server((args.host, args.port), Handler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    camera_shutdown = threading.Event()
    camera_thread = threading.Thread(
        target=state.camera_archive_worker, args=(camera_shutdown,), daemon=True
    )
    camera_thread.start()
    log("listen", host=args.host, port=args.port, archive_dir=args.archive_dir)
    rclpy.init()
    from scripts.recording_postprocess import initialize
    media_queue = initialize(state)
    node = RecorderNode(args, state)
    try:
        rclpy.spin(node)
    finally:
        state.stop()
        media_queue.close()
        camera_shutdown.set()
        state.camera_pending_event.set()
        camera_thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
