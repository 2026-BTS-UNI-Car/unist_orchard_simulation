#!/usr/bin/env python3
"""Start, use and stop the steering G-SIM stack (compose.ubts-steer.yaml) under the stack operating rules.

The rules are docs/steering/STACK_OPERATIONS_KO.md (DECISIONS R22). This helper enforces the ones a program can check:

  one stack at a time      a lock file in the state folder names the project that owns the stack; "up" refuses while
                           the lock names another project, or while any container of another ubts-steer* compose
                           project runs (also one started without this helper)
  a project name per job   --project is required and must be ubts-steer-<job> (lower case letters, digits, - and _);
                           the bare default name ubts-steer of the compose file is refused
  short budget wait        "up" reserves memory with docker_budget.py (label = project, default 6 GiB) and waits at
                           most --budget-wait-s (default 600 s); a timeout stops before any container starts
  the starter stops it     "down" refuses a project that the lock does not name; it runs compose down -t 20 for its
                           own project only (never -v: the named volumes stay), releases the budget and the lock
  leftovers are recorded   "down" writes a receipt: the containers and networks of the project left after the down
  host load before up      "up" refuses while the hold file exists (UBTS_STEER_HOLD_FILE) and records the host-load
                           check file given with --load-check (the check itself is a separate step of the rules)

Commands (every docker call names the project with -p; the docker program is UBTS_STEER_DOCKER or 'docker'):
  python -I tools/steer_sim/stack.py up     --project ubts-steer-<job> [--profile executor] [--gib 6]
                                            [--budget-wait-s 600] [--load-check FILE] [SERVICE ...]
  python -I tools/steer_sim/stack.py run    --project ubts-steer-<job> -- COMMAND ...   (compose run --rm --no-deps check)
  python -I tools/steer_sim/stack.py ps     --project ubts-steer-<job>
  python -I tools/steer_sim/stack.py down   --project ubts-steer-<job> [--receipt FILE]
  python -I tools/steer_sim/stack.py status
Folders: --state-dir or UBTS_STEER_STACK_STATE (the lock); --budget-tool or UBTS_STEER_BUDGET_TOOL (docker_budget.py,
outside the repository). The compose environment (UBTS_STEER_SCENE, UBTS_STEER_CLOCK, UBTS_STEER_IMAGE, ...) is taken
from the caller's environment at "up" and stored in the lock, so "run" and "down" use the same values (compose must
not recreate the sim service). Exit codes: 0 ok, 2 usage, 3 budget timeout, 4 refused by a rule, 5 docker failed.
Placeholder-based SIM tooling; it never touches a vehicle.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
COMPOSE_FILE = ROOT / 'compose.ubts-steer.yaml'
PROJECT_RE = re.compile(r'^ubts-steer-[a-z0-9][a-z0-9_-]{0,40}$')
PROJECT_PREFIX = 'ubts-steer'
LOCK_NAME = 'stack.lock.json'
SCHEMA = 'ubts.steer_stack.v1'
DEFAULT_GIB = 6.
DEFAULT_BUDGET_WAIT_S = 600.
DEFAULT_BUDGET_TTL_S = 14400.
DOWN_TIMEOUT_S = 20
ENV_KEYS_PREFIX = ('UBTS_STEER_',)
ENV_KEYS_SKIP = ('UBTS_STEER_STACK_STATE', 'UBTS_STEER_BUDGET_TOOL', 'UBTS_STEER_DOCKER', 'UBTS_STEER_HOLD_FILE')
EXIT_USAGE, EXIT_BUDGET, EXIT_REFUSED, EXIT_DOCKER = 2, 3, 4, 5


class Refused(Exception):
    """A stack rule refuses the command (exit EXIT_REFUSED)."""


def check_project(name):
    if not name or not PROJECT_RE.match(name):
        raise Refused('project %r: use ubts-steer-<job> (lower case letters, digits, - and _; the bare compose default '
                      '%r is refused, every job names its own project)' % (name, PROJECT_PREFIX))
    return name


def docker_program():
    """The docker command as a list: UBTS_STEER_DOCKER (a program, or a JSON list such as a test double) or docker."""
    value = os.environ.get('UBTS_STEER_DOCKER') or 'docker'
    return json.loads(value) if value.startswith('[') else [value]


def run_docker(args, check=True, capture=True, env=None):
    proc = subprocess.run(docker_program() + list(args), capture_output=capture, text=True, encoding='utf-8',
                          errors='replace', env=env)
    if check and proc.returncode != 0:
        raise RuntimeError('docker %s failed (%d): %s' % (' '.join(args[:3]), proc.returncode,
                                                         (proc.stderr or '').strip()[-400:]))
    return proc


def running_projects():
    """Compose project names of the running containers (label com.docker.compose.project)."""
    proc = run_docker(['ps', '--format', '{{.Label "com.docker.compose.project"}}'])
    return sorted({line.strip() for line in proc.stdout.splitlines() if line.strip()})


def project_leftovers(project):
    """(container ids, network ids) of the project that docker still lists (all states)."""
    label = 'label=com.docker.compose.project=%s' % project
    containers = run_docker(['ps', '-a', '-q', '--filter', label]).stdout.split()
    networks = run_docker(['network', 'ls', '-q', '--filter', label]).stdout.split()
    return containers, networks


def state_dir(args):
    folder = args.state_dir or os.environ.get('UBTS_STEER_STACK_STATE')
    if not folder:
        tool = budget_tool(args, required=False)
        if tool is not None:
            folder = str(Path(tool).resolve().parent.parent / 'locks' / 'steer_stack')
    if not folder:
        raise Refused('no state folder: give --state-dir or UBTS_STEER_STACK_STATE (outside the repository)')
    path = Path(folder)
    path.mkdir(parents=True, exist_ok=True)
    return path


def budget_tool(args, required=True):
    tool = getattr(args, 'budget_tool', None) or os.environ.get('UBTS_STEER_BUDGET_TOOL')
    if not tool and required:
        raise Refused('no budget tool: give --budget-tool or UBTS_STEER_BUDGET_TOOL (docker_budget.py)')
    return tool or None


def read_lock(folder):
    path = folder / LOCK_NAME
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_bytes().decode('utf-8'))
    except ValueError:
        return {'project': None, 'note': 'unreadable lock file: treated as foreign'}


def write_json(path, payload):
    tmp = Path(str(path) + '.tmp')
    tmp.write_bytes((json.dumps(payload, indent=1, sort_keys=True) + '\n').encode('utf-8'))
    os.replace(tmp, path)


def compose_args(lock):
    args = ['compose', '-p', lock['project'], '-f', lock['compose_file']]
    for profile in lock.get('profiles') or ():
        args += ['--profile', profile]
    return args


def compose_env(lock):
    env = dict(os.environ)
    env.update(lock.get('env') or {})
    return env


def stack_env():
    return {k: v for k, v in os.environ.items() if k.startswith(ENV_KEYS_PREFIX) and k not in ENV_KEYS_SKIP}


def budget(tool, action, label, gib=None, wait_s=None, ttl_s=None):
    command = [sys.executable, '-I', str(tool), action, '--label', label]
    if action == 'acquire':
        command += ['--gib', '%g' % gib, '--wait-s', '%g' % wait_s, '--ttl-s', '%g' % ttl_s]
    proc = subprocess.run(command, capture_output=True, text=True, encoding='utf-8', errors='replace')
    return proc.returncode, (proc.stdout + proc.stderr).strip()


# --- commands -----------------------------------------------------------------------------------------------------------
def cmd_up(args):
    project = check_project(args.project)
    folder = state_dir(args)
    hold = os.environ.get('UBTS_STEER_HOLD_FILE')
    if hold and Path(hold).exists():
        raise Refused('hold file %s exists: another job holds the host (rules 2)' % hold)
    lock = read_lock(folder)
    if lock is not None:
        raise Refused('the stack lock names project %r (started %s): one stack at a time; that job stops it first'
                      % (lock.get('project'), lock.get('started_utc')))
    others = [p for p in running_projects() if p.startswith(PROJECT_PREFIX) and p != project]
    if others:
        raise Refused('running containers of other steering projects %s: one stack at a time' % others)
    load = None
    if args.load_check:
        path = Path(args.load_check)
        if not path.is_file():
            raise Refused('host-load check file %s not found (rules 2: check the host before the start)' % path)
        load = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    tool = budget_tool(args)
    code, text = budget(tool, 'acquire', project, args.gib, args.budget_wait_s, args.budget_ttl_s)
    if code != 0:
        print(text)
        print('budget not reserved within %g s (exit %d): nothing started' % (args.budget_wait_s, code))
        return EXIT_BUDGET
    lock = dict(schema=SCHEMA, project=project, compose_file=str(Path(args.compose_file).resolve()),
                profiles=list(args.profile or ()), services=list(args.services or ()), env=stack_env(),
                budget=dict(tool=str(tool), label=project, gib=args.gib), load_check=load, pid=os.getpid(),
                started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))
    write_json(folder / LOCK_NAME, lock)
    proc = run_docker(compose_args(lock) + ['up', '-d'] + list(args.services or ()), check=False, capture=False,
                      env=compose_env(lock))
    if proc.returncode != 0:
        print('compose up failed (%d): the lock stays; run "down --project %s" to clean up' % (proc.returncode, project))
        return EXIT_DOCKER
    print('up: project %s, lock %s' % (project, folder / LOCK_NAME))
    return 0


def own_lock(args):
    project = check_project(args.project)
    folder = state_dir(args)
    lock = read_lock(folder)
    if lock is None:
        raise Refused('no stack lock: project %s was not started with this helper (nothing is stopped; a foreign or '
                      'stray stack is the job of whoever started it)' % project)
    if lock.get('project') != project:
        raise Refused('the stack lock names project %r, not %r: a job stops only the stack it started'
                      % (lock.get('project'), project))
    return folder, lock


def cmd_run(args):
    folder, lock = own_lock(args)
    command = list(args.command or ())
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        raise Refused('run needs a command after --')
    proc = run_docker(compose_args(lock) + ['run', '--rm', '--no-deps', 'check'] + command, check=False,
                      capture=False, env=compose_env(lock))
    return 0 if proc.returncode == 0 else EXIT_DOCKER


def cmd_ps(args):
    folder, lock = own_lock(args)
    proc = run_docker(compose_args(lock) + ['ps'], check=False, capture=False, env=compose_env(lock))
    return 0 if proc.returncode == 0 else EXIT_DOCKER


def cmd_down(args):
    folder, lock = own_lock(args)
    project = lock['project']
    proc = run_docker(compose_args(lock) + ['down', '-t', str(DOWN_TIMEOUT_S)], check=False, capture=False,
                      env=compose_env(lock))
    containers, networks = project_leftovers(project)
    code, text = budget(lock['budget']['tool'], 'release', project) if lock.get('budget') else (0, 'no budget')
    receipt = dict(schema=SCHEMA, project=project, down_exit=proc.returncode, budget_release_exit=code,
                   budget_release=text[-400:], leftover_containers=len(containers), leftover_networks=len(networks),
                   leftover_container_ids=containers, leftover_network_ids=networks, lock=lock,
                   stopped_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                   note='volumes are kept on purpose (never down -v)')
    if args.receipt:
        write_json(Path(args.receipt), receipt)
    if proc.returncode == 0 and not containers and not networks:
        (folder / LOCK_NAME).unlink()
        print('down: project %s, 0 containers and 0 networks left, lock released' % project)
        return 0
    print('down: project %s, exit %d, %d containers and %d networks left: the lock stays'
          % (project, proc.returncode, len(containers), len(networks)))
    return EXIT_DOCKER


def cmd_status(args):
    folder = state_dir(args)
    lock = read_lock(folder)
    others = running_projects()
    print(json.dumps(dict(lock=lock, running_projects=others), indent=1, sort_keys=True))
    return 0


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--state-dir', default=None)
    common.add_argument('--budget-tool', default=None)
    up = sub.add_parser('up', parents=[common])
    up.add_argument('--project', required=True)
    up.add_argument('--profile', action='append')
    up.add_argument('--gib', type=float, default=DEFAULT_GIB)
    up.add_argument('--budget-wait-s', type=float, default=DEFAULT_BUDGET_WAIT_S)
    up.add_argument('--budget-ttl-s', type=float, default=DEFAULT_BUDGET_TTL_S)
    up.add_argument('--compose-file', default=str(COMPOSE_FILE))
    up.add_argument('--load-check', default=None, help='host-load check record of this start (rules 2)')
    up.add_argument('services', nargs='*')
    for name in ('run', 'ps', 'down'):
        sp = sub.add_parser(name, parents=[common])
        sp.add_argument('--project', required=True)
        if name == 'run':
            sp.add_argument('command', nargs=argparse.REMAINDER)
        if name == 'down':
            sp.add_argument('--receipt', default=None)
    sub.add_parser('status', parents=[common])
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        return dict(up=cmd_up, run=cmd_run, ps=cmd_ps, down=cmd_down, status=cmd_status)[args.cmd](args)
    except Refused as exc:
        print('refused: %s' % exc)
        return EXIT_REFUSED
    except RuntimeError as exc:
        print('docker: %s' % exc)
        return EXIT_DOCKER


if __name__ == '__main__':
    sys.exit(main())
