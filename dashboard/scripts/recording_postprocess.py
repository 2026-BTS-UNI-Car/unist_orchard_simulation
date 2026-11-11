"""Bounded native-camera/HLS queue. No ROS replay or motor transport.

Only recorder-finalized camera archives enter automatically. One Linux child
owns all export/encoding work; recording, control and live mapping preempt it.
Raw files are hashed in that child, never in HTTP handlers.
"""
from __future__ import annotations
import argparse
from collections import OrderedDict
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import urllib.request
import uuid

SCHEMA = "orchard.media_queue.v2"
TERMINAL = {"complete", "failed", "cancelled"}


def read_json(path):
    try:
        path = Path(path)
        if path.stat().st_size > 1024 * 1024:
            return {}
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def atomic_json(path, value):
    path = Path(path)
    temp = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    temp.replace(path)


def source_files(archive):
    """Explicit raw-file roots; generated media/job files are excluded."""
    archive = Path(archive).resolve(strict=True)
    metadata = read_json(archive / "metadata.json")
    if metadata.get("state") != "archived" or metadata.get("id") != archive.name:
        raise ValueError("finished_archive_required")
    if metadata.get("kind") not in ("camera", "both"):
        raise ValueError("camera_archive_required")
    if metadata.get('video_timing_contract') is not None:
        from scripts.sim_video_clock import timing_contract
        timing_contract(metadata)
        clock=metadata.get('video_clock_anchors')
        if not isinstance(clock,dict) or clock.get('complete') is not True or clock.get('error'):
            # Do not hash/launch an encoder against a sidecar whose append
            # worker failed or missed its bounded flush deadline.
            raise ValueError('sim_video_anchors_not_complete')
        if not (archive/'native_camera_clock.jsonl').is_file():
            raise ValueError('sim_video_anchor_source_invalid')
    files = [archive / "metadata.json"]
    for name in ("timeline.json", "preview_scan.json", "gps.jsonl", "motor.jsonl", "native_camera_clock.jsonl"):
        if (archive / name).is_file():
            files.append(archive / name)
    for name in ("rosbag2", "mapping_source"):
        root = archive / name
        if root.is_symlink():
            raise ValueError("raw_source_symlink")
        if root.exists():
            entries = list(root.rglob("*"))
            if any(path.is_symlink() for path in entries):
                raise ValueError("raw_source_symlink")
            files.extend(sorted(path for path in entries if path.is_file()))
    if not (archive / "rosbag2/metadata.yaml").is_file():
        raise ValueError("finished_native_bag_required")
    for path in files:
        if path.is_symlink() or archive not in path.resolve(strict=True).parents:
            raise ValueError("raw_source_symlink")
    return files


def source_hashes(archive):
    archive = Path(archive).resolve(strict=True)
    result = {}
    for path in source_files(archive):
        digest = hashlib.sha256()
        before = (path.stat().st_size, path.stat().st_mtime_ns)
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        if before != (path.stat().st_size, path.stat().st_mtime_ns):
            raise ValueError("raw_source_changed_during_hash")
        result[path.relative_to(archive).as_posix()] = digest.hexdigest()
    return result


def verify_sources(archive, expected):
    if source_hashes(archive) != expected:
        raise ValueError("raw_source_changed")


def process(archive, attempt):
    """Constrained child only; incomplete attempts stay as evidence."""
    from scripts.export_recording_camera import export
    from scripts.build_adaptive_recording import build
    from scripts.install_adaptive_recording import install
    archive = Path(archive).resolve(strict=True)
    if not re.fullmatch(r"[a-f0-9]{32}", attempt):
        raise ValueError("invalid_attempt")
    workspace = archive / "generated_video_work" / attempt
    if workspace.parent.is_symlink():
        raise ValueError("generated_workspace_symlink")
    workspace.mkdir(parents=True, exist_ok=False)
    progress = workspace / "progress.json"
    def report(stage, **fields):
        atomic_json(progress, dict(stage=stage, updated_at=time.time(), **fields))
    try:
        report("verifying_raw_sources")
        protected = source_hashes(archive)
        atomic_json(workspace / "source_sha256.json", protected)
        # Existing sidecars are never silently replaced or assumed valid.
        if (archive / "adaptive_video").exists():
            from dashboard_server.adaptive_video import manifest
            existing = manifest(archive)
            native_sources = {name for name in protected if name == 'metadata.json' or name.startswith('rosbag2/')}
            recorded_sources = existing.get('source_sha256') if existing else None
            if not isinstance(recorded_sources, dict) or not native_sources <= recorded_sources.keys() or any(
                    protected.get(name) != sha for name, sha in recorded_sources.items()):
                raise ValueError("existing_adaptive_sidecar_requires_review")
            for name, row in existing["assets"].items():
                digest = hashlib.sha256()
                with (archive / "adaptive_video" / name).open("rb") as stream:
                    for block in iter(lambda: stream.read(1024 * 1024), b""):
                        digest.update(block)
                if digest.hexdigest() != row["sha256"]:
                    raise ValueError("existing_adaptive_asset_changed")
            verify_sources(archive, protected)
            report("complete", already_installed=True, raw_source_preserved=True)
            return
        report("exporting_native_frames")
        exported = export(archive, workspace / "source.zip")
        verify_sources(archive, protected)
        media = {key: exported.get(key) for key in ('frames', 'width', 'height')}
        report("encoding_adaptive_video", **media)
        prepared = build(workspace / "source.zip", workspace / "package", "ffmpeg", archive / "metadata.json")
        report("verifying_generated_video", **media,
               duration_s=prepared.get('duration_s'), encode_elapsed_s=prepared.get('build_elapsed_s'))
        verify_sources(archive, protected)
        result = install(workspace / "package", archive.parent)
        verify_sources(archive, protected)
        report("complete", result=result, raw_source_preserved=True, **media,
               duration_s=prepared.get('duration_s'), encode_elapsed_s=prepared.get('build_elapsed_s'))
    except Exception as error:
        report("failed", error=str(error)[:300])
        raise


def child_main(archive, attempt, expected_parent=None):
    # FFmpeg descendants inherit one-core affinity. OpenCV/BLAS cannot consume
    # a second core, including in export/hash stages before FFmpeg starts.
    import ctypes
    if os.getpgrp() != os.getpid():
        raise ValueError('dedicated_media_process_group_required')
    # A recorder crash must not leave an orphan encoder. The process-group
    # handler kills FFmpeg descendants too; partial outputs remain untouched.
    def terminate_group(_signal, _frame):
        os.killpg(os.getpgrp(), signal.SIGKILL)
    signal.signal(signal.SIGTERM, terminate_group)
    parent = os.getppid()
    if expected_parent is not None and parent != expected_parent:
        terminate_group(None, None)
    def parent_or_resume(_signal, _frame):
        # SIGCONT wakes a stopped group leader after parent death. SIGTERM
        # alone would stay pending on a SIGSTOP-frozen child and leak the
        # inherited flock forever. Intentional resume retains the same parent.
        if os.getppid() != parent:
            terminate_group(None, None)
    signal.signal(signal.SIGCONT, parent_or_resume)
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGCONT, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), 'parent_death_signal_unavailable')
    # Container PID 1 can be the legitimate recorder parent.
    if os.getppid() != parent:
        terminate_group(None, None)
    cpus = os.sched_getaffinity(0)
    os.sched_setaffinity(0, {min(cpus)})
    os.nice(10)
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[name] = "1"
    os.environ["UBTS_MEDIA_THREADS"] = "1"
    import cv2
    cv2.setNumThreads(1)
    process(archive, attempt)


class MediaQueue:
    def __init__(self, root, *, busy, limit=8, start_worker=True, launcher=None):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.limit = max(1, min(8, int(limit)))
        self.busy = busy
        self.launcher = launcher or self._launch
        self.lock = threading.RLock()
        self.jobs = OrderedDict()
        self.process = None
        self.running_id = None
        self.frozen = False
        self.stop_event = threading.Event()
        self.thread = None
        self.owner = (self.root / ".media_queue.lock").open("a")
        try:
            fcntl.flock(self.owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.owner.close()
            raise ValueError("media_queue_owner_already_running")
        try:
            self._load()
        except Exception:
            fcntl.flock(self.owner, fcntl.LOCK_UN)
            self.owner.close()
            raise
        if start_worker:
            self.thread = threading.Thread(target=self._run, daemon=True, name="bounded-media-queue")
            self.thread.start()

    def _archive(self, name):
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", name):
            raise ValueError("invalid_recording_id")
        archive = self.root / name
        if archive.is_symlink() or archive.resolve().parent != self.root or not archive.is_dir():
            raise ValueError("invalid_archive_path")
        return archive

    def _load(self):
        # Only queue-marked jobs resume. Legacy recordings are never queued by
        # age/absence of HLS. Failed/cancelled jobs need an explicit action.
        rows = []
        for archive in self.root.iterdir():
            if not archive.is_dir() or archive.is_symlink():
                continue
            value = read_json(archive / "video_preparation.json")
            if (value.get("schema") == SCHEMA and value.get("recording_id") == archive.name
                    and re.fullmatch(r'[A-Za-z0-9_-]{1,128}', archive.name)
                    and value.get('state') in TERMINAL | {'queued', 'paused', 'running', 'waiting_priority'}
                    and type(value.get('queued_at')) in (int, float) and math.isfinite(value['queued_at'])):
                rows.append(value)
        rows.sort(key=lambda row: float(row.get("queued_at", 0)))
        for row in rows:
            if row.get("state") not in TERMINAL and row.get("state") != "paused":
                row.update(state="queued", reason="service_restarted")
            self.jobs[row["recording_id"]] = row
            if sum(other['state'] not in TERMINAL for other in self.jobs.values()) > self.limit:
                row.update(state='failed', error='media_queue_restore_capacity', reason='explicit_resume_required')
                self._save(row)
        self._prune()

    def _prune(self):
        while (len(self.jobs) > self.limit + 16 or
               sum(row['state'] in TERMINAL for row in self.jobs.values()) > 16):
            terminal = next((name for name, row in self.jobs.items() if row["state"] in TERMINAL), None)
            if terminal is None:
                break
            self.jobs.pop(terminal)

    def _save(self, row):
        row["updated_at"] = time.time()
        atomic_json(self._archive(row["recording_id"]) / "video_preparation.json", row)

    def enqueue(self, name):
        archive = self._archive(name)
        metadata = read_json(archive / "metadata.json")
        if metadata.get("id") != name or metadata.get("state") != "archived":
            raise ValueError("finished_archive_required")
        if metadata.get("kind") not in ("camera", "both") or not (archive / "rosbag2/metadata.yaml").is_file():
            return dict(ok=False, error="native_camera_bag_required", recording_id=name)
        with self.lock:
            if name in self.jobs:
                return dict(ok=True, existing=True, job=dict(self.jobs[name]))
            if sum(row["state"] not in TERMINAL for row in self.jobs.values()) >= self.limit:
                # Preserve a visible per-archive rejection instead of silently
                # losing the new recording's preparation request.
                row = dict(schema=SCHEMA, recording_id=name, state='failed', stage='queue_admission',
                           queued_at=time.time(), attempt=None, reason='explicit_resume_required',
                           error='media_queue_full', raw_source_preserved=True)
                self.jobs[name] = row
                self._save(row); self._prune()
                return dict(ok=False, error="media_queue_full", max_jobs=self.limit, job=dict(row))
            row = dict(schema=SCHEMA, recording_id=name, state="queued", stage="queued", queued_at=time.time(),
                       attempt=None, reason=None, error=None, raw_source_preserved=True)
            self.jobs[name] = row
            self._save(row)
            self._prune()
            return dict(ok=True, job=dict(row))

    def status(self):
        with self.lock:
            return dict(ok=True, schema=SCHEMA, max_jobs=self.limit, max_workers=1,
                        cpu_core_limit=1, live_first=True, running_id=self.running_id,
                        counts={state:sum(row['state'] == state for row in self.jobs.values())
                                for state in ('queued', 'waiting_priority', 'running', 'paused', 'complete', 'failed', 'cancelled')},
                        jobs=[dict(row) for row in self.jobs.values()])

    def action(self, name, action):
        if action not in ("pause", "resume", "cancel"):
            raise ValueError("invalid_media_action")
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', name):
            raise ValueError('invalid_recording_id')
        with self.lock:
            row = self.jobs.get(name)
            if row is None:
                # Explicit recovery of a pruned terminal job is allowed only
                # when this queue previously marked that exact recording.
                row = read_json(self._archive(name) / 'video_preparation.json')
                if row.get('schema') != SCHEMA or row.get('recording_id') != name or row.get('state') not in TERMINAL:
                    raise ValueError("unknown_media_job")
                if action == 'resume' and sum(other['state'] not in TERMINAL for other in self.jobs.values()) >= self.limit:
                    raise ValueError('media_queue_full')
                if row['state'] != 'complete':
                    self.jobs[name] = row
            if row["state"] == "complete":
                return dict(ok=True, unchanged=True, job=dict(row))
            if action == "cancel":
                if name == self.running_id:
                    self._terminate()
                row.update(state="cancelled", reason="operator_cancelled")
            elif action == "pause":
                if row["state"] not in TERMINAL:
                    row.update(state="paused", reason="operator_paused")
                    if name == self.running_id:
                        self._freeze(True)
            else:
                if sum(other["state"] not in TERMINAL for other in self.jobs.values()) >= self.limit and row["state"] in TERMINAL:
                    raise ValueError("media_queue_full")
                row.update(state="queued", reason="operator_resumed", error=None)
            self._save(row)
            self._prune()
            return dict(ok=True, job=dict(row))

    def _launch(self, archive, attempt):
        log = (archive / "generated_video_work").resolve()
        if archive not in log.parents:
            raise ValueError("generated_workspace_symlink")
        log.mkdir(exist_ok=True)
        with (log / (attempt + ".log")).open("xb") as stream:
            return subprocess.Popen([sys.executable, "-m", "scripts.recording_postprocess", "--worker",
                                     str(archive), "--attempt", attempt, '--parent-pid', str(os.getpid())], stdout=stream, stderr=subprocess.STDOUT,
                                    start_new_session=True, pass_fds=(self.owner.fileno(),),
                                    cwd=Path(__file__).resolve().parents[1])

    def _freeze(self, freeze):
        if self.process is not None and self.process.poll() is None and freeze != self.frozen:
            os.killpg(self.process.pid, signal.SIGSTOP if freeze else signal.SIGCONT)
            self.frozen = freeze

    def _terminate(self):
        process = self.process
        if process is not None and process.poll() is None:
            self._freeze(False)
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=1)
        self.process = None
        self.running_id = None
        self.frozen = False

    def tick(self):
        with self.lock:
            # Idle dashboards must not trigger HTTP polling or background work.
            if self.process is None and not any(row['state'] in ('queued', 'waiting_priority') for row in self.jobs.values()):
                return
            operator_paused = bool(self.running_id and self.jobs[self.running_id]['state'] == 'paused')
        try:
            reason = None if operator_paused else self.busy()
        except Exception:
            reason = "priority_state_unavailable"
        with self.lock:
            if self.process is not None:
                row = self.jobs[self.running_id]
                progress = read_json(self._archive(self.running_id) / "generated_video_work" / row["attempt"] / "progress.json")
                for key in ('frames', 'width', 'height', 'duration_s', 'encode_elapsed_s'):
                    if key in progress:
                        row[key] = progress[key]
                stage = progress.get("stage") or row["stage"]
                code = self.process.poll()
                if code is not None:
                    row.update(state="complete" if code == 0 and stage == "complete" else "failed",
                               stage=stage, error=progress.get("error") or (
                                   None if code == 0 and stage == 'complete' else 'incomplete_generated_video' if code == 0 else 'media_child_failed'),
                               reason=None, finished_at=time.time())
                    self._save(row)
                    self._prune()
                    self.process = None
                    self.running_id = None
                    self.frozen = False
                else:
                    paused = row["state"] == "paused"
                    self._freeze(bool(reason or paused))
                    state = "paused" if paused else "waiting_priority" if reason else "running"
                    if (state, stage, row.get("reason")) != (row["state"], row["stage"], "operator_paused" if paused else reason):
                        row.update(state=state, stage=stage, reason="operator_paused" if paused else reason)
                        self._save(row)
                    return
            if reason:
                for row in self.jobs.values():
                    if row["state"] in ("queued", "waiting_priority") and row.get("reason") != reason:
                        row.update(state="waiting_priority", reason=reason)
                        self._save(row)
                return
            row = next((row for row in self.jobs.values() if row["state"] in ("queued", "waiting_priority")), None)
            if row is None:
                return
            try:
                attempt = uuid.uuid4().hex
                self.process = self.launcher(self._archive(row["recording_id"]), attempt)
                self.running_id = row["recording_id"]
                row.update(state="running", stage="starting", attempt=attempt, reason=None)
            except Exception as error:
                row.update(state="failed", error=str(error)[:200])
            self._save(row)

    def _run(self):
        while not self.stop_event.wait(.2):
            try:
                self.tick()
            except Exception:
                # No spin/second child after an I/O failure.
                with self.lock:
                    self._freeze(True)

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
        with self.lock:
            if self.running_id:
                row = self.jobs[self.running_id]
                if row["state"] != "paused":
                    row.update(state="queued", reason="service_stopped")
                self._save(row)
            self._terminate()
            fcntl.flock(self.owner, fcntl.LOCK_UN)
            self.owner.close()


def priority_reason(state):
    # Never hold the recording callback lock during network I/O.
    with state.lock:
        if state.active or getattr(state, 'finalizing', False):
            return "recording_active"
        latest = getattr(getattr(state, 'motor_timeline', None), 'latest', None) or {}
        last_at = getattr(getattr(state, 'motor_timeline', None), 'last_received_steady', None)
        if last_at is not None and time.monotonic() - last_at < 1.5:
            control = latest.get('status') or {}
            if control.get('armed') or control.get('moving'):
                return 'rover_control_active'
    runtime = state.archive_dir.parent
    rover = read_json(Path(os.environ.get("ROVER_BRIDGE_STATUS_FILE", runtime / "rover_bridge_status.json")))
    if rover.get("armed") or rover.get("moving"):
        return "rover_control_active"
    if not isinstance(rover.get("updated_at"), (int, float)) or not 0 <= time.time() - rover["updated_at"] <= 3:
        return "rover_status_unavailable"
    try:
        request = urllib.request.Request(os.environ.get("MEDIA_MAPPING_JOBS_URL", "http://127.0.0.1:8080/api/live/mapping-jobs"))
        with urllib.request.urlopen(request, timeout=.2) as response:
            mapping = json.loads(response.read(128 * 1024))
        if not mapping.get("ok"):
            return "mapping_status_unavailable"
        live = next((job for job in mapping.get("jobs", []) if job.get("id") == "live"), None)
        if live is None or not isinstance(live.get("tracking"), bool):
            return "mapping_status_unavailable"
        if live["tracking"] or live.get("active") or live.get("processing"):
            return "live_mapping_active"
        if any(job.get("processing") for job in mapping.get("jobs", [])):
            return "recorded_mapping_active"
    except (OSError, ValueError, TypeError):
        return "mapping_status_unavailable"
    return None


def initialize(state):
    state.media_queue = MediaQueue(state.archive_dir, busy=lambda: priority_reason(state))
    return state.media_queue


def enqueue(archive, queue=None):
    if os.environ.get("RECORDER_ADAPTIVE_VIDEO_AUTO", "1") != "1":
        return dict(ok=True, state="disabled")
    if queue is None:
        return dict(ok=False, error="media_queue_not_initialized")
    return queue.enqueue(Path(archive).name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker", type=Path, required=True)
    parser.add_argument("--attempt", required=True)
    parser.add_argument('--parent-pid', required=True, type=int)
    args = parser.parse_args()
    child_main(args.worker, args.attempt, args.parent_pid)
