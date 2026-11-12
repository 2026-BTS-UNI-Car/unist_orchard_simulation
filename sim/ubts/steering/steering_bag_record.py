"""Separate bag process for /rover/steering/state (design 4.5, 8.3 P6b, SIM_ACCEPTANCE A23).

The pinned data recorder (dashboard/scripts/data_recorder_service.py) records a fixed topic list with one argument
per topic and has no argument for an extra topic (design 4.5). So the steering topic goes to its own bag, written
by its own 'ros2 bag record' process. This module wraps that process; steering_sidecar.py starts it when a recording
starts and stops it when the recording stops (the recording-ID handshake lives there, not here).

  BagRecord(output_dir, ...)   one 'ros2 bag record' process: command(), start(), request_stop(), poll(), stop(),
                               result(). Stop: SIGINT (the recorder writes metadata.yaml and closes the file), then
                               SIGTERM after stop_timeout_s, then SIGKILL after kill_after_s more.
  read_metadata(bag_dir)       topics, message counts, storage and duration of metadata.yaml (None fields when the
                               bag was not finalized, for example after SIGKILL)
  main()                       CLI: record into --out until SIGINT / SIGTERM or --duration, print the result JSON

Command (Jazzy rosbag2; the positional topic list is deprecated there, so --topics is used):
  [setpriv --pdeathsig INT --] ros2 bag record -o <output_dir> -s mcap --topics /rover/steering/state
      --node-name ubts_steering_bag --max-cache-size 1048576 --disable-keyboard-controls [--use-sim-time]

- setpriv --pdeathsig INT (util-linux, present in the steering image): the bag process gets SIGINT when the process
  that started it ends (also after SIGKILL of that process), so a crashed sidecar does not leave an orphan recorder
  that never closes its file. The kernel sends the signal when the starting THREAD ends: start the bag from a thread
  that lives as long as the bag (steering_sidecar starts it from its main thread). Without setpriv the guard is
  'none' (recorded in result()).
- --use-sim-time: the same choice as the pinned recorder in simulation mode (its bag uses the SIM clock, metadata
  rosbag_timebase); the steering bag then stamps messages with /clock time and writes nothing before the first
  /clock message (rosbag2 help text). /clock itself is not recorded in this bag (the main bag already has it).
- --node-name ubts_steering_bag: the main bag's recorder keeps the default node name rosbag2_recorder; two nodes of
  one name would collide in the ROS graph.
- --max-cache-size 1 MiB instead of the 100 MiB default: the recorder container has a 512 MiB limit and this topic
  is about 12 kB/s (20 Hz JSON of about 600 B), so the cache flushes about every 85 s.
- The output folder must not exist (rosbag2 refuses an existing folder); the wrapper refuses it first.

Linux only for the process part (POSIX signals, ROS). The pure parts (command, read_metadata) run everywhere.
Placeholder-based SIM results only (자리값 기반 SIM 결과), never field validation.
CLI: python3 -B -m steering.steering_bag_record --out <new folder> [--topic T] [--use-sim-time] [--duration S]
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

TOPIC = '/rover/steering/state'
NODE_NAME = 'ubts_steering_bag'
STORAGE = 'mcap'
MAX_CACHE_SIZE = 1048576
STOP_TIMEOUT_S = 4.0          # SIGINT -> SIGTERM (the supervisor's grace time is 6 s: stay inside it)
KILL_AFTER_S = 1.0            # SIGTERM -> SIGKILL
METADATA_FILE = 'metadata.yaml'
SCHEMA = 'ubts.steering_bag_record.v1'


class BagRecordError(RuntimeError):
    pass


def pdeathsig_prefix(which=shutil.which):
    """['setpriv', '--pdeathsig', 'INT', '--'] when util-linux setpriv is on PATH (Linux), else []."""
    if os.name != 'posix':
        return []
    path = which('setpriv')
    return [path, '--pdeathsig', 'INT', '--'] if path else []


def read_metadata(bag_dir):
    """Summary of <bag_dir>/metadata.yaml. finalized False (and None counts) when the file is missing or unreadable:
    rosbag2 writes it when the recorder closes the bag normally."""
    import yaml
    path = Path(bag_dir) / METADATA_FILE
    out = dict(finalized=False, metadata_file=METADATA_FILE, storage=None, message_count=None, topics={},
               duration_s=None, starting_time_ns=None, files=[], error=None)
    try:
        raw = path.read_bytes()
    except FileNotFoundError:
        out['error'] = 'metadata_missing'
        return out
    except OSError as exc:
        out['error'] = 'metadata_unreadable: %s' % type(exc).__name__
        return out
    try:
        info = yaml.safe_load(raw.decode('utf-8'))['rosbag2_bagfile_information']
        topics = {}
        for row in info.get('topics_with_message_count') or []:
            meta = row.get('topic_metadata') or {}
            name = meta.get('name')
            if isinstance(name, str):
                topics[name] = dict(type=meta.get('type'), message_count=int(row.get('message_count') or 0))
        duration = (info.get('duration') or {}).get('nanoseconds')
        start = (info.get('starting_time') or {}).get('nanoseconds_since_epoch')
        files = [str(f) for f in (info.get('relative_file_paths') or [])]
        out.update(finalized=True, storage=info.get('storage_identifier'),
                   message_count=int(info.get('message_count') or 0), topics=dict(sorted(topics.items())),
                   duration_s=None if duration is None else int(duration) / 1e9,
                   starting_time_ns=None if start is None else int(start), files=files)
    except (ValueError, TypeError, AttributeError, KeyError, UnicodeDecodeError, yaml.YAMLError) as exc:
        out['error'] = 'metadata_malformed: %s' % type(exc).__name__
    return out


class BagRecord:
    """One 'ros2 bag record' process for the steering topic. Not thread safe: call it from one thread (the thread
    that called start(), so the pdeathsig guard follows the right thread)."""

    def __init__(self, output_dir, *, topics=(TOPIC,), use_sim_time=False, node_name=NODE_NAME, storage=STORAGE,
                 max_cache_size=MAX_CACHE_SIZE, ros2=('ros2',), log_path=None, guard=None,
                 stop_timeout_s=STOP_TIMEOUT_S, kill_after_s=KILL_AFTER_S, clock=time.monotonic,
                 popen=subprocess.Popen, env=None):
        self.output_dir = Path(output_dir)
        self.topics = [str(t) for t in topics]
        if not self.topics or any(not t.startswith('/') for t in self.topics):
            raise BagRecordError('topics must be absolute ROS topic names: %r' % (self.topics,))
        self.use_sim_time = bool(use_sim_time)
        self.node_name = node_name
        self.storage = storage
        self.max_cache_size = int(max_cache_size)
        self.ros2 = [ros2] if isinstance(ros2, str) else list(ros2)
        self.log_path = None if log_path is None else Path(log_path)
        self.guard = pdeathsig_prefix() if guard is None else list(guard)
        self.stop_timeout_s, self.kill_after_s = float(stop_timeout_s), float(kill_after_s)
        self.clock = clock
        self.popen = popen
        self.env = env
        self.process = None
        self.started_at = self.started_monotonic = None
        self.stop_requested_monotonic = None
        self.signals = []            # (signal name, seconds after the stop request)
        self.exit_code = None
        self.exited_monotonic = None
        self.error = None

    def command(self):
        cmd = self.guard + self.ros2 + ['bag', 'record', '-o', str(self.output_dir), '-s', self.storage,
                                        '--topics', *self.topics, '--node-name', self.node_name,
                                        '--max-cache-size', str(self.max_cache_size), '--disable-keyboard-controls']
        if self.use_sim_time:
            cmd.append('--use-sim-time')
        return cmd

    def start(self):
        """Start the process (the output folder must not exist; its parent must). Returns the result dict."""
        if self.process is not None:
            raise BagRecordError('bag process already started')
        if os.name != 'posix':
            raise BagRecordError('ros2 bag record runs on Linux only (POSIX signals)')
        if self.output_dir.exists() or self.output_dir.is_symlink():
            raise BagRecordError('bag output folder exists already: %s' % self.output_dir)
        if not self.output_dir.parent.is_dir():
            raise BagRecordError('bag output parent folder missing: %s' % self.output_dir.parent)
        log = subprocess.DEVNULL
        handle = None
        if self.log_path is not None:
            handle = open(self.log_path, 'ab')
            log = handle
        try:
            self.process = self.popen(self.command(), stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True, env=self.env)
        except OSError as exc:
            self.error = 'bag_start_failed: %s' % exc
            raise BagRecordError(self.error) from None
        finally:
            if handle is not None:
                handle.close()
        self.started_at = time.time()
        self.started_monotonic = self.clock()
        return self.result()

    @property
    def pid(self):
        return None if self.process is None else self.process.pid

    def running(self):
        return self.process is not None and self.process.poll() is None

    def _signal(self, sig, name):
        try:
            os.killpg(self.process.pid, sig)
        except ProcessLookupError:
            return
        except PermissionError:
            try:
                self.process.send_signal(sig)
            except ProcessLookupError:
                return
        base = self.stop_requested_monotonic if self.stop_requested_monotonic is not None else self.clock()
        self.signals.append((name, round(self.clock() - base, 3)))

    def request_stop(self):
        """Send SIGINT once (graceful close). Later poll() calls escalate when the deadlines pass."""
        if self.process is None or self.stop_requested_monotonic is not None:
            return
        self.stop_requested_monotonic = self.clock()
        if self.process.poll() is None:
            self._signal(signal.SIGINT, 'SIGINT')

    def poll(self):
        """True once the process has exited (or was never started). Escalates SIGTERM / SIGKILL after a stop request."""
        if self.process is None:
            return True
        code = self.process.poll()
        if code is not None:
            if self.exit_code is None:
                self.exit_code = code
                self.exited_monotonic = self.clock()
            return True
        if self.stop_requested_monotonic is not None:
            waited = self.clock() - self.stop_requested_monotonic
            names = [s[0] for s in self.signals]
            if waited >= self.stop_timeout_s and 'SIGTERM' not in names:
                self._signal(signal.SIGTERM, 'SIGTERM')
            elif waited >= self.stop_timeout_s + self.kill_after_s and 'SIGKILL' not in names:
                self._signal(signal.SIGKILL, 'SIGKILL')
        return False

    def stop(self, poll_s=0.05, sleep=time.sleep):
        """Blocking stop: SIGINT, escalation, wait for the exit. Returns result()."""
        if self.process is None:
            return self.result()
        self.request_stop()
        limit = self.stop_timeout_s + self.kill_after_s + 5.0
        while not self.poll():
            if self.clock() - self.stop_requested_monotonic > limit:
                self.error = 'bag_process_did_not_exit'
                break
            sleep(poll_s)
        return self.result()

    def result(self):
        """Process facts and (after the exit) the metadata.yaml summary. finalized: metadata.yaml was written."""
        exited = self.process is not None and self.exit_code is not None
        meta = read_metadata(self.output_dir) if exited or self.process is None else None
        stop_s = None
        if self.stop_requested_monotonic is not None and self.exited_monotonic is not None:
            stop_s = round(self.exited_monotonic - self.stop_requested_monotonic, 3)
        return dict(schema=SCHEMA, output_dir=self.output_dir.name, command=self.command(), topics=list(self.topics),
                    use_sim_time=self.use_sim_time, time_base='ros_simulation_time' if self.use_sim_time
                    else 'host_wall_time', node_name=self.node_name, storage_requested=self.storage,
                    max_cache_size=self.max_cache_size,
                    orphan_guard='setpriv --pdeathsig INT' if self.guard else 'none',
                    started=self.process is not None, pid=self.pid, started_at=self.started_at,
                    stop_requested=self.stop_requested_monotonic is not None, signals=[list(s) for s in self.signals],
                    exit_code=self.exit_code, stop_duration_s=stop_s, error=self.error, metadata=meta)


def build_parser():
    p = argparse.ArgumentParser(prog='steering_bag_record', description=__doc__.splitlines()[0])
    p.add_argument('--out', required=True, help='bag folder to create (must not exist)')
    p.add_argument('--topic', action='append', default=None, help='topic (repeatable; default %s)' % TOPIC)
    p.add_argument('--use-sim-time', action='store_true', help='stamp with /clock (the pinned recorder does this '
                   'in simulation mode)')
    p.add_argument('--duration', type=float, default=None, help='stop after this many seconds (default: SIGINT)')
    p.add_argument('--log', default=None, help='append the ros2 output to this file')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(1))
    bag = BagRecord(args.out, topics=args.topic or (TOPIC,), use_sim_time=args.use_sim_time, log_path=args.log)
    bag.start()
    begin = time.monotonic()
    while not stop and bag.running() and (args.duration is None or time.monotonic() - begin < args.duration):
        time.sleep(0.1)
    print(json.dumps(bag.stop(), sort_keys=True), flush=True)
    return 0 if bag.exit_code == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
