"""Start, watch and stop one executor run of the steering G-SIM stack from the host (docker exec; no extra container).

The executor container runs the real row executor with a controller in its slot (steering.exec_container_main). A run
is started on the executor's operator page (127.0.0.1:8097 in the sim network namespace, token in the control volume).
That page is hold-to-run: the run goes on only while heartbeats of one press arrive. The one-press start of design v2
(R32) is not implemented. This module is the operator for a SIM run: it sends Start and Arm, presses once, sends the
heartbeats itself until the run ends, and prints one status line per second (the executor's own estimates, not
truth).

  in the executor container (control volume, no truth):
    docker exec -it <project>-executor-1 python3 -B -m steering.exec_run start [--controller JSON]
        Start, Arm, one press, heartbeats until the run ends (row end, a hold of the controller, an operator stop or
        --limit-s). Ctrl-C sends an operator stop. Exit code 0: ended with row_end; 3: ended otherwise; 4: no start.
    docker exec <project>-executor-1 python3 -B -m steering.exec_run stop      operator stop at once (/api/stop)
    docker exec <project>-executor-1 python3 -B -m steering.exec_run status    one status line
  in the sim container (runtime volume with the scene; SIM only, the vehicle must stand):
    docker exec <project>-sim-1 /ros_entrypoint.sh python3 -B -m steering.exec_run reset [--lateral M] [--heading DEG]
        puts the rover back to the start pose of the scene with the Gazebo set_pose service (evaluation / SIM side:
        it reads the scene files, which the executor never sees)

Default controller: {"kind": "row_follow"} (closed_loop/steer_control.py). Every value of the stack is a placeholder
(자리값 기반 SIM 결과), never field performance. Nothing here talks to a vehicle.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

RESULT_KIND = '자리값 기반 SIM 결과'
DEFAULT_CONTROLLER = {'kind': 'row_follow'}
EXEC_DIR = os.environ.get('UBTS_STEER_EXEC_DIR', '/run/ubts-steer/exec')
SCENE_DIR = os.environ.get('UBTS_STEER_SCENE_DIR', '/runtime/steer/scene')
WORLD_NAME = 'ubts_orchard'
MODEL_NAME = 'ubts_rover'
OFFSET_DROP_M = .05                 # extra height of a shifted start (the terrain differs beside the spawn point)
ARM_ATTEMPTS = 6
ARM_RETRY_S = .6
RELEASE_WAIT_S = 8.                 # an ended run keeps the lease up to terminal_stop_wait_s before its own STOP
PAUSE_GIVE_UP_S = 15.
EXIT_ROW_END, EXIT_OTHER_END, EXIT_NO_START = 0, 3, 4


# --- executor side ------------------------------------------------------------------------------------------------------
def last_tick(exec_dir=EXEC_DIR, tail_bytes=16384):
    """The newest tick record of the executor JSONL log (the executor's estimates), or None."""
    folder = Path(exec_dir) / 'log'
    try:
        files = sorted(folder.glob('row_executor_*.jsonl'), key=lambda p: p.stat().st_mtime)
    except OSError:
        return None
    if not files:
        return None
    try:
        with open(files[-1], 'rb') as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            handle.seek(max(0, size - tail_bytes))
            lines = handle.read().splitlines()
    except OSError:
        return None
    for raw in reversed(lines):
        try:
            record = json.loads(raw.decode('utf-8'))
        except (ValueError, UnicodeDecodeError):
            continue
        if isinstance(record, dict) and record.get('k') == 'tick':
            return record
    return None


def _number(value, pattern, scale=1.):
    return '-' if not isinstance(value, (int, float)) or isinstance(value, bool) else pattern % (value * scale)


def status_line(elapsed_s, status, tick):
    """One line: run state and the executor's estimates of the newest tick (the log has the lateral offset in metres
    and the heading error in degrees; shown as cm and degrees)."""
    runner = (status or {}).get('runner') or {}
    client = (status or {}).get('client') or {}
    tick = tick or {}
    control = tick.get('ctl') or [None, None]
    return ('t=%5.1fs run=%s %s/%s ctl=%s corridor=%s lat=%s cm head=%s deg dist=%s m v=%s m/s T=%s S=%s '
            'lidar_age=%s s client=%s' % (
                elapsed_s, runner.get('run_id'), runner.get('state'), runner.get('reason'), control[1],
                {True: 'ok', False: 'no'}.get(tick.get('corridor'), '-'), _number(tick.get('lat'), '%+.1f', 100.),
                _number(tick.get('head'), '%+.1f'), _number(tick.get('dist'), '%.2f'),
                _number(tick.get('v'), '%.2f'), tick.get('T'), tick.get('S'), _number(tick.get('lidar_age'), '%.2f'),
                client.get('state')))


class Operator:
    """The operator page and the SIM hooks of the executor (the classes of exec_checks, no ROS)."""

    def __init__(self, exec_dir=EXEC_DIR, op=None, hold=None, hooks=None):
        from . import exec_checks as X
        self.x = X
        self.exec_dir = exec_dir
        self.op = op or X.OperatorClient(exec_dir)
        self.hold = hold or X.HoldBeater(self.op)
        self.hooks = hooks or X.SocketHooks(Path(exec_dir) / 'hooks.sock')

    def status(self):
        try:
            return self.hooks.call('status')['status']
        except (OSError, ValueError, KeyError, self.x.ClientError):
            return None

    def wait(self, predicate, timeout_s, period_s=.1, sleep=time.sleep):
        end = time.monotonic() + timeout_s
        status = self.status()
        while not (status is not None and predicate(status)):
            if time.monotonic() >= end:
                return None
            sleep(period_s)
            status = self.status()
        return status


def start_run(operator, spec, out=print, limit_s=600., period_s=1., sleep=time.sleep, tick_reader=last_tick):
    """Start, Arm, one press with heartbeats until the run ends. Returns (exit code, summary dict)."""
    X = operator.x
    status = operator.status()
    if status is None:
        out('no executor status: is the executor container running (compose profile executor)?')
        return EXIT_NO_START, dict(error='no_executor_status')
    state = status['runner'].get('state')
    if state in (X.STATE_ACTIVE, X.STATE_PAUSED):
        out('a run is open (%s / %s): stop it first (exec_run stop)' % (state, status['runner'].get('reason')))
        return EXIT_NO_START, dict(error='run_in_progress')
    operator.wait(lambda s: (s.get('client') or {}).get('state') != X.CLIENT_ARMED, RELEASE_WAIT_S, sleep=sleep)
    status = operator.status() or status
    if (status.get('client') or {}).get('state') == X.CLIENT_LATCHED:
        out('the executor is latched (%s): acknowledging' % (status.get('client') or {}).get('reason'))
        operator.op.post('/api/ack')
    reply = operator.hooks.request(dict(op='controller', spec=spec))
    if not reply.get('ok'):
        out('controller refused: %s' % reply.get('error'))
        return EXIT_NO_START, dict(error='controller_refused:%s' % reply.get('error'))
    out('controller %s' % json.dumps(spec, sort_keys=True))
    code, reply, _, _ = operator.op.post('/api/start', {'row_len_m': None})
    if not reply.get('ok') and reply.get('error') != 'run_in_progress':
        out('start refused: %s' % reply.get('error'))
        return EXIT_NO_START, dict(error='start_refused:%s' % reply.get('error'))
    armed = None
    for attempt in range(ARM_ATTEMPTS):
        code, armed, _, _ = operator.op.post('/api/arm', timeout_s=8.)
        if armed.get('ok'):
            break
        out('arm attempt %d refused: %s' % (attempt + 1, armed.get('error')))
        sleep(ARM_RETRY_S)
    if not (armed or {}).get('ok'):
        operator.op.post('/api/end')
        return EXIT_NO_START, dict(error='arm_refused:%s' % (armed or {}).get('error'))
    operator.hold.press()
    out('started: one press is held by this command (heartbeats). Ctrl-C or "exec_run stop" = operator stop.')
    t0 = time.monotonic()
    paused_since, final, seen_active = None, None, False
    try:
        while time.monotonic() - t0 < limit_s:
            sleep(period_s)
            status = operator.status()
            out(status_line(time.monotonic() - t0, status, tick_reader(operator.exec_dir)))
            if status is None:
                continue
            runner = status['runner']
            state = runner.get('state')
            seen_active = seen_active or state == X.STATE_ACTIVE
            if state == X.STATE_ENDED:
                final = dict(runner)
                break
            if (status.get('client') or {}).get('state') == X.CLIENT_LATCHED:
                final = dict(runner, latched=(status.get('client') or {}).get('reason'))
                break
            if state == X.STATE_PAUSED:
                paused_since = paused_since or time.monotonic()
                if time.monotonic() - paused_since > PAUSE_GIVE_UP_S:
                    out('paused for %.0f s (%s): ending the run' % (PAUSE_GIVE_UP_S, runner.get('reason')))
                    final = dict(runner, gave_up_paused=True)
                    break
            else:
                paused_since = None
        else:
            out('limit %.0f s reached: ending the run' % limit_s)
            final = dict((operator.status() or {}).get('runner') or {}, limit_reached=True)
    except KeyboardInterrupt:
        operator.hold.lose()
        code, reply, _, _ = operator.op.post('/api/stop')
        out('interrupted: operator stop sent (HTTP %s)' % code)
        final = dict((operator.status() or {}).get('runner') or {}, interrupted=True)
    finally:
        operator.hold.release()
        operator.hold.close()
    if final.get('state') != X.STATE_ENDED:
        operator.op.post('/api/end')
        ended = operator.wait(lambda s: s['runner'].get('state') == X.STATE_ENDED, 5., sleep=sleep)
        if ended is not None:
            final = dict(final, state=ended['runner'].get('state'), end_reason=ended['runner'].get('end_reason'))
    reason = final.get('end_reason')
    out('ended: %s (state %s). %s, not field performance.' % (reason, final.get('state'), RESULT_KIND))
    return (EXIT_ROW_END if reason == 'row_end' else EXIT_OTHER_END), dict(runner=final, seen_active=seen_active)


def stop_run(operator, out=print):
    code, reply, _, _ = operator.op.post('/api/stop')
    out('operator stop: HTTP %s %s' % (code, json.dumps(reply, sort_keys=True)[:200]))
    return 0 if code == 200 else 1


# --- SIM side (the sim container: scene files and Gazebo) -----------------------------------------------------------
def spawn_height(manifest, metadata):
    """z of the rover origin at the scene start (W1: terrain height + drop; other scenes: the terrain entry or 0)."""
    summary = (manifest or {}).get('world_summary') or {}
    height = summary.get('spawn_height') or {}
    if isinstance(height.get('z_m'), (int, float)):
        return float(height['z_m'])
    spawn = (metadata or {}).get('spawn')
    return float(spawn[2]) if isinstance(spawn, (list, tuple)) and len(spawn) == 3 else 0.


def start_pose(scene_dir=SCENE_DIR, lateral_m=0., heading_deg=0.):
    """(x, y, z, yaw) of the scene start, shifted sideways (left positive) and turned if asked."""
    folder = Path(scene_dir)
    metadata = json.loads((folder / 'world_metadata.json').read_bytes().decode('utf-8'))
    manifest = json.loads((folder / 'scene_manifest.json').read_bytes().decode('utf-8'))
    spawn = metadata.get('spawn')
    if not (isinstance(spawn, (list, tuple)) and len(spawn) == 3):
        raise ValueError('the scene metadata has no spawn point (reset is for the w1_force scene)')
    yaw = float((metadata.get('config') or {}).get('spawn_yaw') or 0.)
    x = float(spawn[0]) - math.sin(yaw) * lateral_m
    y = float(spawn[1]) + math.cos(yaw) * lateral_m
    z = spawn_height(manifest, metadata) + (OFFSET_DROP_M if lateral_m or heading_deg else 0.)
    return x, y, z, yaw + math.radians(heading_deg)


def set_model_pose(x, y, z, yaw, world=WORLD_NAME, model=MODEL_NAME, timeout_ms=3000):
    """Gazebo set_pose service (gz.transport13; needs the ROS / Gazebo environment of /ros_entrypoint.sh)."""
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node
    request = Pose()
    request.name = model
    request.position.x, request.position.y, request.position.z = float(x), float(y), float(z)
    request.orientation.w, request.orientation.z = math.cos(yaw / 2.), math.sin(yaw / 2.)
    service = '/world/%s/set_pose' % world
    result, reply = Node().request(service, request, Pose, Boolean, int(timeout_ms))
    return dict(service=service, requested=bool(result), accepted=bool(result and reply.data))


def reset_pose(args, out=print):
    x, y, z, yaw = start_pose(args.scene_dir, args.lateral, args.heading)
    reply = set_model_pose(x, y, z, yaw)
    out('set_pose %s: x %.3f y %.3f z %.3f yaw %.2f deg -> %s' % (
        MODEL_NAME, x, y, z, math.degrees(yaw), 'accepted' if reply['accepted'] else 'NOT accepted'))
    return 0 if reply['accepted'] else 1


# --- main ---------------------------------------------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(prog='exec_run', description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='command', required=True)
    c = sub.add_parser('start')
    c.add_argument('--controller', default=None, help='controller spec JSON (default %s)' % json.dumps(
        DEFAULT_CONTROLLER))
    c.add_argument('--limit-s', type=float, default=600.)
    sub.add_parser('stop')
    sub.add_parser('status')
    c = sub.add_parser('reset')
    c.add_argument('--lateral', type=float, default=0., help='start this far left of the scene start [m]')
    c.add_argument('--heading', type=float, default=0., help='start turned by this angle [deg]')
    c.add_argument('--scene-dir', default=SCENE_DIR)
    return p


def _terminate(signum, frame):
    raise KeyboardInterrupt()


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.command == 'reset':
        return reset_pose(args)
    operator = Operator()
    if args.command == 'status':
        print(status_line(0., operator.status(), last_tick()))
        return 0
    if args.command == 'stop':
        return stop_run(operator)
    signal.signal(signal.SIGTERM, _terminate)
    spec = json.loads(args.controller) if args.controller else dict(DEFAULT_CONTROLLER)
    code, _ = start_run(operator, spec, limit_s=args.limit_s)
    return code


if __name__ == '__main__':
    sys.exit(main())
