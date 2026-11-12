"""Process supervisor of the steering G-SIM stack (design 2.2, 2.5, 4.3 and 8.3 P6a; docs/steering/P6A_HEADLESS_KO.md).

One supervisor per container role of compose.ubts-steer.yaml (the old sim/ubts/supervisor.py is not used):

  sim        assemble the scene (gz_assemble: w0_force, w1_force or field1010 with a field world spec) -> gz sim
             (server; GUI is a separate optional client) -> ros_gz_bridge with bridge_steer.yaml (checked against the
             scene manifest topics) -> Embree LiDAR (pinned cpu_lidar.py: /sim/scan from /ground_truth/odom) ->
             [clip node: field1010 scenes, steering.scan_unobserved_clip by default] -> S2 format node
             (steering.s2_format_node by default, WF-4: /sim/scan or /sim/scan_clipped -> /scan in the real S2 format;
             --s2-node none leaves /scan absent as in P6a) -> sensor contract (sensor_contract_steer: IMU error model
             /imu/data_raw, virtual GPS, static TF, status files; --s2-present when the S2 node runs) -> WebSocket
             bridges of the dashboard (camera, lidar on /scan, imu, gps)
  steer      waits for the sim role of this run (READY stamp and a fresh simulation_status.json with a live /clock and
             the same run token) -> host profile JSON (host_profile_export) -> ESP emulator (gz_esp_emulator: pty
             /run/ubts-steer/esp-tty, MCU firmware + actuator model on the SIM clock) -> bridge wrapper
             (scripts.steer_rover_bridge, pty transport, SIM role, --clock wall|sim, control socket
             /run/ubts-steer/control.sock). The bridge is restarted after an exit (at most 5 times per 60 s, like a
             service manager); the emulator is not
  recorder   the existing data recorder service (P6b uses it; it records the topics it already knows)

Run token (2026-11-09 review RG-10): the sim supervisor makes a new token at its start, writes it into the scene
READY stamp (JSON) and passes it to the sensor contract (simulation_status.json sim_run). A stamp left by a run that
was killed (SIGKILL: no clean-up) has another token, so the steer role never takes the scene of an earlier run, and
it starts its children only once the sim role's /clock is live (the startup load of gz, the Embree BVH and the
sensor contract is over).

Children start in order; each has a readiness check (a topic seen by gz, a status file, a pty link) with a time
limit. An exit of a child that is not restartable stops the whole role (the container exits non-zero). SIGTERM or
SIGINT stops the children in reverse order (SIGTERM to the process group, SIGKILL after the grace time) and the
supervisor exits 0. Child output goes to <runtime>/steer/logs/<role>_<name>.log; the supervisor state to
<runtime>/steer/supervisor_<role>.json.

Hooks: --s2-node MODULE (default steering.s2_format_node, WF-4; 'none' runs no S2 node; run as python3 -B -m MODULE
--input <topic> --output /scan --status-file <runtime>/s2_node_status.json --run-token T [--profile P]; ready when its
status file of this run token says waiting or running; it makes 'scan' part of sensors_alive) and --clip-node MODULE
(field1010 only; default steering.scan_unobserved_clip; run as python3 -B -m MODULE --input /sim/scan --output
/sim/scan_clipped --raster <scene>/field_world/raster.npz --status-file <runtime>/scan_clip_status.json --noise-sd SD
--noise-seed N; the S2 node then reads the clipped topic). With the clip node, cpu_lidar runs on a runtime copy of
rover.yaml with lidar.noise_stddev_m 0 and the clip node adds the rover.yaml range noise after the clip: the F-SIM
order (UNOBSERVED end on exact ranges, then noise; review RW-04). The W2 sensing chain is cpu_lidar -> clip node -> S2
node -> /scan (three ROS nodes, each scan keeps its header stamp).
The executor container is not started here: it is its own compose service (profile executor, WF-4,
steering.exec_container_main; 0.5 GiB of the 6 GiB budget).

Stale status files (audit G06; the runtime volume is kept between runs): every role moves the status files of its own
children (ROLE_STATUS_FILES) that a run left behind into <runtime>/steer/retired/<run token of this start>/ before it
starts its children, so a file of a node that this scene does not run (the clip node in W0, the S2 node with --s2-node
none) or of an earlier run is never read as this run's (readiness probes, checks). The newest RETIRED_KEEP folders stay.

Supervisor hooks (steer role, SIM only, WF-4 A17(G) with the executor): a Unix socket in the hooks volume
(--supervisor-hook-socket, default /run/ubts-steer-emu/supervisor.sock; the executor container does not mount it), one
JSON line per connection: {"op": "status"} -> the children and their pids; {"op": "signal", "child": "bridge",
"signal": "KILL" | "TERM" | "STOP" | "CONT", "resume_after_s": s (STOP only, optional)} -> the signal to the bridge
wrapper process (the 09-19 tests 'bridge forced kill' and a stalled bridge loop); a killed bridge is restarted by the
supervisor as before (BRIDGE_RESTARTS).
Placeholder-based SIM results, not field validation. CLI: python3 -B -m steering.supervisor_steer --help.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid

import yaml

SIM_ROOT = Path(__file__).resolve().parents[1]              # sim/ubts
REPO_ROOT = SIM_ROOT.parents[1]
DASHBOARD_ROOT = Path(os.environ.get('UBTS_STEER_DASHBOARD_ROOT', str(REPO_ROOT / 'dashboard')))
BRIDGE_TEMPLATE = Path(__file__).resolve().with_name('bridge_steer.yaml')
ROLES = ('sim', 'steer', 'recorder')
SCENES = ('w0_force', 'w1_force', 'field1010')
FIELD_SCENES = ('field1010',)
DEFAULT_CLIP_NODE = 'steering.scan_unobserved_clip'
DEFAULT_S2_NODE = 'steering.s2_format_node'
S2_NODE_OFF = ('none', 'off', '0')                          # --s2-node none: no S2 node, /scan absent (P6a)
S2_STATUS = 's2_node_status.json'
GRACE_S = 6.
STATUS_FRESH_S = 3.                                         # simulation_status.json age for a live sim role
BRIDGE_RESTARTS = (5, 60.)                                  # at most 5 restarts in 60 s
SCHEMA = 'ubts.steer_supervisor.v1'
ACT_PREFIX = '/sim/act/'
# Status files of each role's children in the runtime folder (audit G06: moved away at the role's start).
ROLE_STATUS_FILES = {
    'sim': ('simulation_status.json', 'cpu_lidar_status.json', 'scan_clip_status.json', S2_STATUS,
            'camera_driver_status.json', 'imu_driver_status.json', 'lidar_bridge_status.json',
            'camera_bridge_status.json', 'imu_bridge_status.json', 'gps_bridge_status.json'),
    'steer': ('steering_status.json', 'rover_bridge_status.json'),
    'recorder': ('data_recorder_status.json',),
}
RETIRED_DIR = 'retired'                                     # <runtime>/steer/retired/<run token>/
RETIRED_KEEP = 5
SUPERVISOR_HOOK_SOCKET = '/run/ubts-steer-emu/supervisor.sock'
HOOK_CHILDREN = ('bridge',)
HOOK_SIGNALS = {'KILL': signal.SIGKILL, 'TERM': signal.SIGTERM, 'STOP': signal.SIGSTOP, 'CONT': signal.SIGCONT} \
    if hasattr(signal, 'SIGKILL') else {}
HOOK_MAX_STALL_S = 30.


class SupervisorError(RuntimeError):
    pass


# --- bridge configuration -------------------------------------------------------------------------------------------------
def load_bridge_template(path=BRIDGE_TEMPLATE):
    entries = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(entries, list) or not all(isinstance(e, dict) for e in entries):
        raise SupervisorError('invalid_bridge_configuration')
    return entries


def bridge_config(manifest, template=None):
    """bridge_steer.yaml checked against a scene manifest: the actuator entries must be exactly the manifest's steer
    and drive topics (ROS_TO_GZ, Float64 -> Double); the camera entries are dropped when the scene has no camera.
    Returns the entry list. Raises SupervisorError on a mismatch."""
    entries = load_bridge_template() if template is None else [dict(e) for e in template]
    topics = manifest['model']['topics']
    want = sorted(list(topics['steer'].values()) + list(topics['drive'].values()))
    have = sorted(e['gz_topic_name'] for e in entries if str(e.get('gz_topic_name', '')).startswith(ACT_PREFIX))
    if manifest['model'].get('steer_type') == 'axle_pivot':
        # The template carries the linkage (left/right) steering topics; an axle_pivot scene has one axle topic.
        entries = [e for e in entries if not (e['gz_topic_name'].startswith(ACT_PREFIX + 'steer_')
                                              and e['gz_topic_name'] not in want)]
        for topic in topics['steer'].values():
            if topic not in have:
                entries.append(dict(ros_topic_name=topic, publisher_queue=1, subscriber_queue=1, gz_topic_name=topic,
                                    ros_type_name='std_msgs/msg/Float64', gz_type_name='gz.msgs.Double',
                                    direction='ROS_TO_GZ'))
        have = sorted(e['gz_topic_name'] for e in entries if e['gz_topic_name'].startswith(ACT_PREFIX))
    if want != have:
        raise SupervisorError('bridge_steer.yaml actuator topics %s differ from the scene manifest %s' % (have, want))
    for e in entries:
        if e['gz_topic_name'].startswith(ACT_PREFIX):
            if (e.get('direction') != 'ROS_TO_GZ' or e.get('ros_type_name') != 'std_msgs/msg/Float64'
                    or e.get('gz_type_name') != 'gz.msgs.Double' or e.get('ros_topic_name') != e['gz_topic_name']):
                raise SupervisorError('actuator bridge entry %s must be Float64 -> Double ROS_TO_GZ' % e['gz_topic_name'])
    truth = {e['gz_topic_name']: e['ros_topic_name'] for e in entries}
    if truth.get(topics['truth_odometry_gz']) != '/ground_truth/odom':
        raise SupervisorError('the truth odometry must reach ROS as /ground_truth/odom')
    if truth.get(topics['truth_joint_states_gz']) != '/sim/truth/joint_states':
        raise SupervisorError('the truth joint states must reach ROS as /sim/truth/joint_states')
    forbidden = [e['ros_topic_name'] for e in entries if e['ros_topic_name'] in ('/odom', '/tf', '/joint_states',
                                                                                 '/sim/cmd_vel', '/sim/wheel_odom',
                                                                                 '/scan')]
    if forbidden:
        raise SupervisorError('bridge entries that the steering stack must not have: %s' % forbidden)
    if not manifest['renderer'].get('camera'):
        entries = [e for e in entries if e['ros_topic_name'] not in ('/camera/image_raw', '/camera/camera_info')]
    return entries


def write_bridge_config(manifest, target):
    entries = bridge_config(manifest)
    target = Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix('.tmp')
    tmp.write_text(yaml.safe_dump(entries, sort_keys=False), encoding='utf-8')
    tmp.replace(target)
    return target


# --- children -------------------------------------------------------------------------------------------------------------------
class Child:
    def __init__(self, name, command, *, cwd=None, env=None, ready=None, ready_timeout_s=60., restart=False):
        self.name, self.command, self.cwd, self.env = name, list(command), cwd, env
        self.ready, self.ready_timeout_s, self.restart = ready, float(ready_timeout_s), restart
        self.process = None
        self.started = []
        self.exits = []
        self.log_path = None

    def describe(self):
        return dict(name=self.name, command=self.command, pid=None if self.process is None else self.process.pid,
                    running=self.process is not None and self.process.poll() is None, starts=len(self.started),
                    exits=self.exits[-5:], restart=self.restart, log=None if self.log_path is None else
                    str(self.log_path))


class Supervisor:
    def __init__(self, role, runtime_dir, log_dir):
        self.role = role
        self.runtime_dir = Path(runtime_dir)
        self.log_dir = Path(log_dir)
        self.children = []
        self.stopping = False
        self.state_path = self.runtime_dir / 'steer' / ('supervisor_%s.json' % role)
        self.events = []

    def note(self, text, **fields):
        entry = dict(t=time.time(), text=text, **fields)
        self.events.append(entry)
        del self.events[:-200]
        print('[supervisor_steer %s] %s %s' % (self.role, text, json.dumps(fields, default=str) if fields else ''),
              flush=True)

    def write_state(self, final=None):
        payload = dict(schema=SCHEMA, role=self.role, updated_at=time.time(), pid=os.getpid(),
                       run_token=getattr(self, 'run_token', None), all_ready=getattr(self, 'all_ready', False),
                       retired_status_files=getattr(self, 'retired_status_files', None),
                       children=[c.describe() for c in self.children], events=self.events[-40:], final=final,
                       source_commit=os.environ.get('UBTS_STEER_SOURCE_COMMIT'),
                       source_status_sha256=os.environ.get('UBTS_STEER_SOURCE_STATUS_SHA256'))
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.state_path.with_name(self.state_path.name + '.tmp')
        tmp.write_bytes((json.dumps(payload, default=str) + '\n').encode('utf-8'))
        os.replace(tmp, self.state_path)

    def start(self, child):
        self.log_dir.mkdir(parents=True, exist_ok=True)
        child.log_path = self.log_dir / ('%s_%s.log' % (self.role, child.name))
        mode = 'a' if child.started else 'w'
        handle = open(child.log_path, mode, encoding='utf-8', newline='\n')
        handle.write('# %s start %s: %s\n' % (time.strftime('%Y-%m-%dT%H:%M:%S'), child.name, ' '.join(child.command)))
        handle.flush()
        env = dict(os.environ)
        env.update(child.env or {})
        child.process = subprocess.Popen(child.command, cwd=child.cwd, env=env, stdout=handle, stderr=subprocess.STDOUT,
                                         start_new_session=True)
        handle.close()
        child.started.append(time.time())
        if child not in self.children:
            self.children.append(child)
        self.note('START', child=child.name, pid=child.process.pid)
        self.write_state()

    def wait_ready(self, child):
        if child.ready is None:
            return
        deadline = time.monotonic() + child.ready_timeout_s
        last_error = None
        started = time.monotonic()
        while time.monotonic() < deadline and not self.stopping:
            if child.process.poll() is not None:
                raise SupervisorError('%s exited (%s) before it was ready; see %s' % (
                    child.name, child.process.returncode, child.log_path))
            try:
                if child.ready():
                    self.note('READY', child=child.name, after_s=round(time.monotonic() - started, 2))
                    return
            except Exception as exc:          # a readiness probe error is "not ready yet"
                last_error = '%s: %s' % (type(exc).__name__, exc)
            time.sleep(.25)
        if not self.stopping:
            raise SupervisorError('%s not ready within %.0f s (last probe error %s); see %s' % (
                child.name, child.ready_timeout_s, last_error, child.log_path))

    def start_all(self, children):
        for child in children:
            if self.stopping:
                return
            self.start(child)
            self.wait_ready(child)

    def watch(self):
        last_state = 0.
        while not self.stopping:
            for child in list(self.children):
                code = child.process.poll()
                if code is None:
                    continue
                child.exits.append(dict(t=time.time(), code=code))
                self.note('EXIT', child=child.name, code=code)
                recent = [t for t in child.started if time.time() - t < BRIDGE_RESTARTS[1]]
                if child.restart and len(recent) <= BRIDGE_RESTARTS[0] and not self.stopping:
                    time.sleep(1.)
                    self.start(child)
                    continue
                raise SupervisorError('child %s exited %s' % (child.name, code))
            if time.time() - last_state > 1.:
                last_state = time.time()
                self.write_state()
            time.sleep(.25)

    def stop_all(self):
        self.stopping = True
        for child in reversed(self.children):
            p = child.process
            if p is not None and p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                    os.killpg(p.pid, signal.SIGCONT)     # a child stopped by a hook gets the TERM too
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + GRACE_S
        for child in reversed(self.children):
            p = child.process
            if p is None:
                continue
            try:
                p.wait(timeout=max(.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                p.wait()
                self.note('KILLED', child=child.name)
            self.note('STOPPED', child=child.name, code=p.returncode)


class SupervisorHooks:
    """Steer-role fault hooks (module docstring 'Supervisor hooks'): signal the bridge wrapper child. SIM only."""

    def __init__(self, supervisor, path, children=HOOK_CHILDREN, signals=None):
        self.supervisor = supervisor
        self.path = str(path)
        self.children = tuple(children)
        self.signals = dict(HOOK_SIGNALS if signals is None else signals)
        self.server = None

    def _child(self, name):
        for child in self.supervisor.children:
            if child.name == name:
                return child
        return None

    def handle(self, request):
        """One hook request (dict) -> reply (dict). Never raises for a bad request."""
        op = request.get('op') if isinstance(request, dict) else None
        if op == 'status':
            return dict(ok=True, role=self.supervisor.role, run_token=getattr(self.supervisor, 'run_token', None),
                        children=[c.describe() for c in self.supervisor.children])
        if op != 'signal':
            return dict(ok=False, error='unknown_op')
        name, sig = request.get('child'), request.get('signal')
        if name not in self.children:
            return dict(ok=False, error='child_not_allowed', allowed=list(self.children))
        if sig not in self.signals:
            return dict(ok=False, error='signal_not_allowed', allowed=sorted(self.signals))
        resume = request.get('resume_after_s')
        if resume is not None:
            if sig != 'STOP' or isinstance(resume, bool) or not isinstance(resume, (int, float)) or \
                    not 0. < float(resume) <= HOOK_MAX_STALL_S:
                return dict(ok=False, error='resume_after_s_only_with_STOP_in_(0,%g]' % HOOK_MAX_STALL_S)
        child = self._child(name)
        if child is None or child.process is None or child.process.poll() is not None:
            return dict(ok=False, error='child_not_running')
        pid = child.process.pid
        wall, mono = time.time(), time.monotonic()
        try:
            os.kill(pid, self.signals[sig])
        except ProcessLookupError:
            return dict(ok=False, error='child_not_running')
        self.supervisor.note('HOOK_SIGNAL', child=name, signal=sig, pid=pid, resume_after_s=resume)
        if resume is not None:
            timer = threading.Timer(float(resume), self._resume, (pid, name))
            timer.daemon = True
            timer.start()
        return dict(ok=True, child=name, pid=pid, signal=sig, wall=wall, mono=mono, resume_after_s=resume)

    def _resume(self, pid, name):
        try:
            os.kill(pid, self.signals.get('CONT', signal.SIGCONT))
            self.supervisor.note('HOOK_SIGNAL', child=name, signal='CONT', pid=pid, timer=True)
        except (ProcessLookupError, AttributeError):
            pass

    def start(self):
        import socketserver
        hooks = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.connection.settimeout(2.)
                try:
                    line = self.rfile.readline(4097)
                    if len(line) > 4096:
                        raise ValueError('request_too_large')
                    request = json.loads(line)
                    if not isinstance(request, dict):
                        raise ValueError('object_required')
                    reply = hooks.handle(request)
                except (ValueError, OSError) as exc:
                    reply = dict(ok=False, error=str(exc))
                try:
                    self.wfile.write((json.dumps(reply, default=str) + '\n').encode('utf-8'))
                except OSError:
                    pass

        class Server(socketserver.ThreadingUnixStreamServer):
            daemon_threads = True

        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        if os.path.exists(self.path):
            os.unlink(self.path)
        self.server = Server(self.path, Handler)
        threading.Thread(target=self.server.serve_forever, name='supervisor-hooks', daemon=True).start()
        return self

    def close(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None
        try:
            os.unlink(self.path)
        except OSError:
            pass


# --- readiness probes ---------------------------------------------------------------------------------------------------
def gz_topic_ready(topic):
    def probe():
        out = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, timeout=10)
        return topic in out.stdout.split()
    return probe


def json_file_ready(path, predicate=None, max_age_s=10.):
    def probe():
        data = json.loads(Path(path).read_bytes().decode('utf-8'))
        age = time.time() - float(data.get('updated_at') or 0.)
        return age <= max_age_s and (predicate is None or predicate(data))
    return probe


def path_ready(path):
    return lambda: os.path.lexists(path)


def alive_after(seconds):
    started = []

    def probe():
        if not started:
            started.append(time.monotonic())
        return time.monotonic() - started[0] >= seconds
    return probe


# --- roles ------------------------------------------------------------------------------------------------------------------
def py_module(module, *args):
    return [sys.executable, '-B', '-m', module] + [str(a) for a in args]


def lidar_config_without_noise(source, target):
    """A runtime copy of rover.yaml for cpu_lidar with lidar.noise_stddev_m 0. Returns (path, the original sd)."""
    config = yaml.safe_load(Path(source).read_text(encoding='utf-8'))
    noise = float(config['lidar'].get('noise_stddev_m', .01))
    out = copy.deepcopy(config)
    out['lidar']['noise_stddev_m'] = 0.
    target = Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix('.tmp')
    tmp.write_bytes(yaml.safe_dump(out, sort_keys=False).encode('utf-8'))
    tmp.replace(target)
    return target, noise


def retire_stale_status(runtime_dir, role, token, keep=RETIRED_KEEP, names=None):
    """Move the status files of the role's children (ROLE_STATUS_FILES) that an earlier run left in the runtime folder
    to <runtime>/steer/retired/<token>/ (audit G06). Keeps the newest `keep` retired folders. Returns the moved names."""
    runtime = Path(runtime_dir)
    names = ROLE_STATUS_FILES.get(role, ()) if names is None else tuple(names)
    target = runtime / 'steer' / RETIRED_DIR / str(token)
    moved = []
    for name in names:
        source = runtime / name
        if not source.is_file():
            continue
        target.mkdir(parents=True, exist_ok=True)
        os.replace(source, target / name)
        moved.append(name)
    folders = runtime / 'steer' / RETIRED_DIR
    if folders.is_dir():
        old = sorted((p for p in folders.iterdir() if p.is_dir()), key=lambda p: p.stat().st_mtime)
        for path in old[:max(0, len(old) - int(keep))]:
            shutil.rmtree(path, ignore_errors=True)
    return moved


def s2_node_of(args):
    """The S2 format node module of this run (default DEFAULT_S2_NODE), or None for --s2-node none."""
    module = (getattr(args, 's2_node', None) or DEFAULT_S2_NODE).strip()
    return None if module.lower() in S2_NODE_OFF else module


def s2_node_command(module, scan_input, runtime, run_token=None, profile=None):
    """The S2 format node's command line (the contract of steering.s2_format_node)."""
    command = py_module(module, '--input', scan_input, '--output', '/scan', '--status-file',
                        Path(runtime) / S2_STATUS)
    if run_token:
        command += ['--run-token', str(run_token)]
    if profile:
        command += ['--profile', str(profile)]
    return command


def s2_node_ready(path, run_token):
    """Ready: the S2 node's status file of this run token says waiting or running (fresh)."""
    return json_file_ready(path, lambda d: d.get('state') in ('waiting', 'running') and
                           (run_token is None or d.get('run_token') == run_token))


def clip_node_command(module, scene_dir, runtime, noise_sd, seed):
    """The clip node's command line (the contract of steering.scan_unobserved_clip)."""
    return py_module(module, '--input', '/sim/scan', '--output', '/sim/scan_clipped', '--raster',
                     Path(scene_dir) / 'field_world' / 'raster.npz', '--status-file',
                     Path(runtime) / 'scan_clip_status.json', '--noise-sd', repr(float(noise_sd)), '--noise-seed',
                     int(seed))


def clip_node_of(args, manifest):
    """The clip node module of the scene: field1010 needs one (default steering.scan_unobserved_clip); other scenes
    have no raster copy and refuse one."""
    field = manifest.get('scene') in FIELD_SCENES
    if field:
        return args.clip_node or DEFAULT_CLIP_NODE
    if args.clip_node:
        raise SupervisorError('--clip-node is only for field world scenes (scene %s has no raster copy)'
                              % manifest.get('scene'))
    return None


def sim_children(args, scene_dir, manifest):
    runtime = Path(args.runtime_dir)
    sdf = scene_dir / manifest['sdf']
    clip_node = clip_node_of(args, manifest)
    lidar_config, noise_sd = SIM_ROOT / 'rover.yaml', None
    if clip_node:
        lidar_config, noise_sd = lidar_config_without_noise(SIM_ROOT / 'rover.yaml',
                                                            runtime / 'steer' / 'rover_lidar_no_noise.yaml')
    bridge_yaml = write_bridge_config(manifest, runtime / 'steer' / 'bridge_steer.runtime.yaml')
    renderer = manifest['renderer']
    gz = ['gz', 'sim', '-r', '-s', '-v', str(args.gz_verbosity), '--render-engine-server', renderer['engine']]
    if renderer['headless']:
        gz.append('--headless-rendering')
    gz.append(str(sdf))
    lidar_env = dict(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    children = [
        Child('gz', gz, ready=gz_topic_ready('/clock'), ready_timeout_s=180.),
        Child('ros_gz_bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '--ros-args', '-p',
                                'config_file:=%s' % bridge_yaml, '-p', 'use_sim_time:=true'], ready=alive_after(2.)),
        Child('cpu_lidar', [sys.executable, '-B', str(SIM_ROOT / 'cpu_lidar.py'), '--world', str(sdf), '--config',
                            str(lidar_config), '--status-file', str(runtime / 'cpu_lidar_status.json')], env=lidar_env,
              ready=json_file_ready(runtime / 'cpu_lidar_status.json',
                                    lambda d: d.get('state') in ('waiting_for_pose', 'running')),
              ready_timeout_s=300.),
    ]
    scan_input = '/sim/scan'
    if clip_node:
        children.append(Child('clip_node', clip_node_command(clip_node, scene_dir, runtime, noise_sd, args.seed),
                              cwd=str(SIM_ROOT), env=lidar_env,
                              ready=json_file_ready(runtime / 'scan_clip_status.json',
                                                    lambda d: d.get('state') in ('waiting', 'running')),
                              ready_timeout_s=120.))
        scan_input = '/sim/scan_clipped'
    s2_node = s2_node_of(args)
    run_token = getattr(args, 'run_token', None)
    if s2_node:
        children.append(Child('s2_node', s2_node_command(s2_node, scan_input, runtime, run_token, args.profile),
                              cwd=str(SIM_ROOT), env=lidar_env, ready=s2_node_ready(runtime / S2_STATUS, run_token),
                              ready_timeout_s=60.))
    contract = ['--scene-dir', scene_dir, '--seed', args.seed, '--runtime-dir', runtime]
    if run_token:
        contract += ['--run-token', run_token]
    if args.profile:
        contract += ['--profile', args.profile]
    if s2_node:
        contract.append('--s2-present')
    if clip_node:
        contract.append('--clip-present')
    if not renderer.get('camera'):
        contract.append('--no-camera')
    children.append(Child('sensor_contract', py_module('steering.sensor_contract_steer', *contract),
                          cwd=str(SIM_ROOT),
                          ready=json_file_ready(runtime / 'simulation_status.json',
                                                lambda d: d.get('schema') == 'ubts.steer_simulation_status.v1'),
                          ready_timeout_s=60.))
    dash = str(DASHBOARD_ROOT)
    if renderer.get('camera'):
        children.append(Child('camera_ws_bridge', py_module('scripts.camera_ws_bridge'), cwd=dash,
                              ready=alive_after(1.)))
    children += [
        Child('lidar_ws_bridge', py_module('scripts.lidar_ws_bridge', '--source', 'ros2', '--topic', '/scan',
                                           '--max-points', '3240', '--max-hz', '10'), cwd=dash, ready=alive_after(1.)),
        Child('imu_ws_bridge', py_module('scripts.imu_ws_bridge'), cwd=dash, ready=alive_after(1.)),
        Child('gps_ws_bridge', py_module('scripts.gps_ws_bridge', '--host', '0.0.0.0'), cwd=dash,
              ready=alive_after(1.)),
    ]
    return children


def steer_children(args, scene_dir, manifest):
    runtime = Path(args.runtime_dir)
    host_profile = runtime / 'steer' / 'host_profile.json'
    export = py_module('steering.host_profile_export', '--out', host_profile, '--cal-id', args.cal_id)
    if args.profile:
        export += ['--profile', args.profile]
    result = subprocess.run(export, cwd=str(SIM_ROOT), capture_output=True, text=True)
    if result.returncode != 0:
        raise SupervisorError('host profile export failed: %s' % (result.stderr.strip() or result.stdout.strip()))
    emulator = ['--scene-dir', scene_dir, '--seed', args.seed, '--cal-id', args.cal_id, '--pty-link', args.pty_link,
                '--hook-socket', args.hook_socket, '--status-file', runtime / 'steering_status.json',
                '--bridge-status-file', runtime / 'rover_bridge_status.json',
                '--log-file', runtime / 'steer' / 'emulator_log.jsonl', '--lead-steps', args.lead_steps]
    if args.profile:
        emulator += ['--profile', args.profile]
    if args.report_adc:
        emulator.append('--report-adc')
    if not args.port_open_reset:
        emulator.append('--no-port-open-reset')
    executable = native_executable()
    if executable:
        emulator += ['--executable', executable]
    bridge = ['--transport', 'pty', '--device', args.pty_link, '--clock', args.clock, '--host-profile', host_profile,
              '--socket', args.control_socket, '--status-file', runtime / 'rover_bridge_status.json',
              '--event-log', runtime / 'steer' / 'bridge_events.jsonl']
    return [
        Child('emulator', py_module('steering.gz_esp_emulator', *emulator), cwd=str(SIM_ROOT),
              ready=path_ready(args.pty_link), ready_timeout_s=60.),
        Child('bridge', py_module('scripts.steer_rover_bridge', *bridge), cwd=str(DASHBOARD_ROOT),
              ready=json_file_ready(runtime / 'rover_bridge_status.json', max_age_s=5.), ready_timeout_s=30.,
              restart=True),
    ]


def recorder_children(args):
    runtime = Path(args.runtime_dir)
    return [Child('data_recorder', py_module('scripts.data_recorder_service', '--host', '0.0.0.0', '--archive-dir',
                                             runtime / 'recordings', '--status-file',
                                             runtime / 'data_recorder_status.json'), cwd=str(DASHBOARD_ROOT),
                  ready=alive_after(2.))]


def native_executable():
    """The image's prebuilt native lockstep program (Dockerfile BUILD_RESULT.json), if any."""
    folder = os.environ.get('UBTS_STEER_BUILD_DIR')
    if not folder:
        return None
    try:
        result = json.loads((Path(folder) / 'BUILD_RESULT.json').read_bytes().decode('utf-8'))
    except (OSError, ValueError):
        return None
    path = result.get('path')
    return path if path and result.get('firmware_build_id') and os.path.isfile(path) else None


def clear_scene_stamp(scene_dir):
    """The sim role removes the READY stamp when it starts and when it stops, so another role never takes the scene
    of an earlier run (the steer role waits for the stamp of the current run token)."""
    try:
        (Path(scene_dir) / 'READY').unlink()
    except FileNotFoundError:
        pass


def write_scene_stamp(scene_dir, token, manifest):
    stamp = dict(token=token, scene=manifest.get('scene'), sdf_sha256=manifest.get('sdf_sha256'), pid=os.getpid(),
                 written_at=time.time())
    path = Path(scene_dir) / 'READY'
    tmp = path.with_name('READY.tmp')
    tmp.write_bytes((json.dumps(stamp) + '\n').encode('utf-8'))
    os.replace(tmp, path)


def read_scene_stamp(scene_dir):
    """The READY stamp (dict) or None (missing, or the plain stamp of an earlier image)."""
    try:
        stamp = json.loads((Path(scene_dir) / 'READY').read_bytes().decode('utf-8'))
    except (OSError, ValueError):
        return None
    return stamp if isinstance(stamp, dict) and stamp.get('token') else None


def sim_role_live(stamp, status, now_wall, fresh_s=STATUS_FRESH_S, supervisor=None):
    """The steer role's start rule: the READY stamp and simulation_status.json of the same run token, the status
    fresh and its /clock alive, and (when supervisor, the sim role's supervisor_sim.json, is given) every child of
    the sim role started (all_ready of the same token: the start load of the WebSocket bridges and the camera is
    over, so the emulator and the bridge do not start inside it)."""
    if not stamp or not isinstance(status, dict):
        return False
    age = now_wall - float(status.get('updated_at') or 0.)
    live = (status.get('sim_run') == stamp.get('token') and 0. <= age <= fresh_s and
            bool(status.get('clock_alive')))
    if supervisor is not None:
        live = live and supervisor.get('run_token') == stamp.get('token') and bool(supervisor.get('all_ready'))
    return live


def field_spec_value(text):
    """--field-spec: JSON text or a JSON file path (None: none)."""
    if not text:
        return None
    text = text.strip()
    if text.startswith('{'):
        return json.loads(text)
    return json.loads(Path(text).read_bytes().decode('utf-8'))


def ensure_scene(args):
    """Build the scene (sim role) or wait for the sim role of this run (other roles). Returns (scene_dir, manifest)."""
    scene_dir = Path(args.scene_dir)
    manifest_path = scene_dir / 'scene_manifest.json'
    if args.role == 'sim':
        clear_scene_stamp(scene_dir)
        field_spec = field_spec_value(args.field_spec)
        if (args.scene in FIELD_SCENES) != (field_spec is not None):
            raise SupervisorError('scene %s %s a field world spec (--field-spec / UBTS_STEER_FIELD_SPEC)' % (
                args.scene, 'needs' if args.scene in FIELD_SCENES else 'takes no'))
        if args.reuse_scene and manifest_path.exists() and field_spec is None:
            manifest = json.loads(manifest_path.read_bytes().decode('utf-8'))
            if manifest.get('scene') == args.scene and manifest.get('renderer', {}).get('camera') == (not args.no_camera):
                write_scene_stamp(scene_dir, args.run_token, manifest)
                return scene_dir, manifest
        from . import gz_assemble
        from .vehicle_profile import load_profile
        profile = load_profile(args.profile)
        manifest = gz_assemble.build_scene(args.scene, scene_dir, profile=profile, profile_path=args.profile,
                                           camera=not args.no_camera, field_spec=field_spec)
        write_scene_stamp(scene_dir, args.run_token, manifest)
        return scene_dir, manifest
    deadline = time.monotonic() + args.scene_wait_s
    status_path = Path(args.runtime_dir) / 'simulation_status.json'
    supervisor_path = Path(args.runtime_dir) / 'steer' / 'supervisor_sim.json'
    while time.monotonic() < deadline:
        stamp = read_scene_stamp(scene_dir)
        try:
            status = json.loads(status_path.read_bytes().decode('utf-8'))
            supervisor = json.loads(supervisor_path.read_bytes().decode('utf-8'))
        except (OSError, ValueError):
            status = supervisor = None
        if stamp and manifest_path.exists() and sim_role_live(stamp, status, time.time(), supervisor=supervisor or {}):
            manifest = json.loads(manifest_path.read_bytes().decode('utf-8'))
            if manifest.get('sdf_sha256') == stamp.get('sdf_sha256'):
                return scene_dir, manifest
        time.sleep(.5)
    raise SupervisorError('scene %s and a live sim role (fresh simulation_status.json with the READY run token, '
                          'clock_alive, supervisor_sim.json all_ready) not seen within %.0f s'
                          % (scene_dir, args.scene_wait_s))


def build_parser():
    p = argparse.ArgumentParser(prog='supervisor_steer', description=__doc__.splitlines()[0])
    p.add_argument('--role', choices=ROLES, required=True)
    p.add_argument('--scene', choices=SCENES, default=os.environ.get('UBTS_STEER_SCENE', 'w0_force'))
    p.add_argument('--scene-dir', default=os.environ.get('UBTS_STEER_SCENE_DIR', '/runtime/steer/scene'))
    p.add_argument('--reuse-scene', action='store_true', help='keep an existing scene of the same name')
    p.add_argument('--no-camera', action='store_true', default=os.environ.get('UBTS_STEER_NO_CAMERA') == '1')
    p.add_argument('--profile', default=os.environ.get('UBTS_STEER_PROFILE') or None)
    p.add_argument('--seed', type=int, default=int(os.environ.get('UBTS_STEER_SEED', '0')))
    p.add_argument('--cal-id', type=int, default=1)
    p.add_argument('--clock', choices=('wall', 'sim'), default=os.environ.get('UBTS_STEER_CLOCK', 'wall'))
    p.add_argument('--lead-steps', type=int, default=int(os.environ.get('UBTS_STEER_LEAD_STEPS', '1')))
    p.add_argument('--report-adc', action='store_true', default=os.environ.get('UBTS_STEER_REPORT_ADC') == '1')
    p.add_argument('--no-port-open-reset', dest='port_open_reset', action='store_false',
                   default=os.environ.get('UBTS_STEER_PORT_OPEN_RESET', '1') != '0')
    p.add_argument('--runtime-dir', default='/runtime')
    p.add_argument('--pty-link', default='/run/ubts-steer/esp-tty')
    p.add_argument('--control-socket', default=os.environ.get('ROVER_CONTROL_SOCKET', '/run/ubts-steer/control.sock'))
    p.add_argument('--hook-socket', default=os.environ.get('UBTS_STEER_HOOK_SOCKET', '/run/ubts-steer-emu/hooks.sock'))
    p.add_argument('--s2-node', default=os.environ.get('UBTS_STEER_S2_NODE') or DEFAULT_S2_NODE,
                   help='module of the S2 format node (default %s; none: no S2 node, /scan absent): python3 -m '
                        'MODULE --input T --output /scan --status-file F --run-token T' % DEFAULT_S2_NODE)
    p.add_argument('--supervisor-hook-socket', default=os.environ.get('UBTS_STEER_SUPERVISOR_HOOK_SOCKET',
                                                                      SUPERVISOR_HOOK_SOCKET),
                   help='steer role: Unix socket of the supervisor fault hooks (SIM only; empty: none)')
    p.add_argument('--clip-node', default=os.environ.get('UBTS_STEER_CLIP_NODE') or None,
                   help='module of the unobserved clip node (field1010 only; default %s there)' % DEFAULT_CLIP_NODE)
    p.add_argument('--field-spec', default=os.environ.get('UBTS_STEER_FIELD_SPEC') or None,
                   help='field1010: field world spec (JSON text or a JSON file path)')
    p.add_argument('--gz-verbosity', type=int, default=2)
    p.add_argument('--scene-wait-s', type=float, default=600.)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    args.run_token = uuid.uuid4().hex
    supervisor = Supervisor(args.role, args.runtime_dir, Path(args.runtime_dir) / 'steer' / 'logs')
    supervisor.run_token = args.run_token
    supervisor.write_state()

    def stop(*_):
        supervisor.stopping = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    final, code = None, 0
    hooks = None
    try:
        supervisor.retired_status_files = retire_stale_status(args.runtime_dir, args.role, args.run_token)
        supervisor.note('RETIRED_STATUS', files=supervisor.retired_status_files,
                        folder='%s/%s' % (RETIRED_DIR, args.run_token))
    except OSError as exc:                    # never block a start on the clean-up; the checks see the note
        supervisor.note('RETIRE_FAILED', error=str(exc))
    try:
        if args.role == 'steer' and args.supervisor_hook_socket:
            try:
                hooks = SupervisorHooks(supervisor, args.supervisor_hook_socket).start()
                supervisor.note('HOOKS', socket=args.supervisor_hook_socket)
            except OSError as exc:
                supervisor.note('HOOKS_UNAVAILABLE', error=str(exc))
                hooks = None
        if args.role == 'recorder':
            children = recorder_children(args)
        else:
            scene_dir, manifest = ensure_scene(args)
            supervisor.note('SCENE', scene=manifest.get('scene'), sdf_sha256=manifest.get('sdf_sha256'),
                            camera=manifest['renderer'].get('camera'))
            children = sim_children(args, scene_dir, manifest) if args.role == 'sim' else \
                steer_children(args, scene_dir, manifest)
        supervisor.start_all(children)
        supervisor.all_ready = True
        supervisor.note('ALL_READY', children=[c.name for c in children])
        supervisor.write_state()
        supervisor.watch()
        final = 'stopped'
    except SupervisorError as exc:
        supervisor.note('FAILED', error=str(exc))
        final, code = 'failed: %s' % exc, 1
    finally:
        if hooks is not None:
            hooks.close()
        supervisor.stop_all()
        if args.role == 'sim':
            clear_scene_stamp(args.scene_dir)
        supervisor.write_state(final=final)
    return code


if __name__ == '__main__':
    sys.exit(main())
