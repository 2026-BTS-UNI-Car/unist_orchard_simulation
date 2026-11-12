"""Stack helper rules (tools/steer_sim/stack.py; docs/steering/STACK_OPERATIONS_KO.md, DECISIONS R22).

No Docker: a test double for the docker program (a Python script that logs every call and keeps the running projects,
containers and networks in a JSON file) and one for docker_budget.py. Checks: a project name per job, one stack at a
time (lock and running containers of other steering projects), a short budget wait that starts nothing on timeout, a
job cannot take down (or use) another project's stack, down never removes volumes, the receipt counts leftovers, the
hold file and the host-load record. Temporary folders only.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools' / 'steer_sim'))
import stack  # noqa: E402

FAKE_DOCKER = r'''
import json, os, sys
args = sys.argv[1:]
log, state_path = os.environ['FAKE_DOCKER_LOG'], os.environ['FAKE_DOCKER_STATE']
with open(log, 'a', encoding='utf-8') as fh:
    fh.write(json.dumps(args) + '\n')
state = json.load(open(state_path, encoding='utf-8')) if os.path.exists(state_path) else {}
running = state.setdefault('running', {})


def label_of(args):
    for a in args:
        if a.startswith('label=com.docker.compose.project='):
            return a.split('=', 2)[2]
    return None


if args[:1] == ['ps'] and '--format' in args:
    for project, n in running.items():
        for _ in range(n):
            print(project)
elif args[:1] == ['ps']:
    for k in range(running.get(label_of(args), 0)):
        print('c%d' % k)
elif args[:2] == ['network', 'ls']:
    if running.get(label_of(args), 0):
        print('n0')
elif args[:1] == ['compose']:
    project = args[args.index('-p') + 1]
    if 'up' in args:
        running[project] = 3
    elif 'down' in args:
        if os.environ.get('FAKE_DOCKER_DOWN_LEAVES') == '1':
            running[project] = 1
        else:
            running.pop(project, None)
json.dump(state, open(state_path, 'w', encoding='utf-8'))
'''
FAKE_BUDGET = r'''
import json, os, sys
with open(os.environ['FAKE_BUDGET_LOG'], 'a', encoding='utf-8') as fh:
    fh.write(json.dumps(sys.argv[1:]) + '\n')
sys.exit(int(os.environ.get('FAKE_BUDGET_EXIT', '0')) if sys.argv[1] == 'acquire' else 0)
'''


class StackRulesTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(prefix='ubts_stack_')
        self.addCleanup(tmp.cleanup)
        self.tmp = Path(tmp.name)
        (self.tmp / 'docker.py').write_bytes(FAKE_DOCKER.encode('utf-8'))
        (self.tmp / 'budget.py').write_bytes(FAKE_BUDGET.encode('utf-8'))
        self.state_dir = self.tmp / 'state'
        env = dict(UBTS_STEER_DOCKER=json.dumps([sys.executable, '-I', str(self.tmp / 'docker.py')]),
                   UBTS_STEER_BUDGET_TOOL=str(self.tmp / 'budget.py'), UBTS_STEER_STACK_STATE=str(self.state_dir),
                   FAKE_DOCKER_LOG=str(self.tmp / 'docker.log'), FAKE_DOCKER_STATE=str(self.tmp / 'docker.json'),
                   FAKE_BUDGET_LOG=str(self.tmp / 'budget.log'), UBTS_STEER_SCENE='w0_force')
        patcher = mock.patch.dict(os.environ, env)
        patcher.start()
        self.addCleanup(patcher.stop)
        for name in ('UBTS_STEER_HOLD_FILE', 'FAKE_BUDGET_EXIT', 'FAKE_DOCKER_DOWN_LEAVES'):
            os.environ.pop(name, None)

    # -- helpers
    def calls(self, name='docker.log'):
        path = self.tmp / name
        return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()] if path.exists() else []

    def compose_calls(self):
        return [c for c in self.calls() if c[:1] == ['compose']]

    def running(self):
        path = self.tmp / 'docker.json'
        return json.loads(path.read_text(encoding='utf-8')).get('running', {}) if path.exists() else {}

    def main(self, *argv):
        with mock.patch('sys.stdout'):
            return stack.main(list(argv))

    # -- tests
    def test_every_job_names_its_own_project(self):
        for bad in ('ubts-steer', 'UBTS-steer-a', 'other-a', 'ubts-steer-', 'ubts-steer-a b', 'ubts-steer-' + 'x' * 42):
            with self.subTest(project=bad):
                self.assertEqual(self.main('up', '--project', bad), stack.EXIT_REFUSED)
        self.assertEqual(self.compose_calls(), [])
        self.assertEqual(self.calls('budget.log'), [])
        self.assertEqual(stack.check_project('ubts-steer-st05_a'), 'ubts-steer-st05_a')

    def test_up_run_down_of_the_own_project(self):
        receipt = self.tmp / 'receipt.json'
        self.assertEqual(self.main('up', '--project', 'ubts-steer-a', '--profile', 'executor', 'sim', 'steer'), 0)
        self.assertTrue((self.state_dir / stack.LOCK_NAME).is_file())
        acquire = self.calls('budget.log')[0]
        self.assertEqual(acquire[:3], ['acquire', '--label', 'ubts-steer-a'])
        self.assertEqual(acquire[acquire.index('--wait-s') + 1], '600')         # short wait (rules 3)
        up = self.compose_calls()[0]
        self.assertEqual(up[:3], ['compose', '-p', 'ubts-steer-a'])
        self.assertEqual(up[-4:], ['up', '-d', 'sim', 'steer'])
        self.assertIn('--profile', up)
        self.assertEqual(self.main('run', '--project', 'ubts-steer-a', '--', 'python3', '-B', '-m', 'x'), 0)
        run = self.compose_calls()[1]
        self.assertEqual(run[run.index('run'):], ['run', '--rm', '--no-deps', 'check', 'python3', '-B', '-m', 'x'])
        self.assertEqual(self.main('down', '--project', 'ubts-steer-a', '--receipt', str(receipt)), 0)
        down = self.compose_calls()[2]
        self.assertEqual(down[:3], ['compose', '-p', 'ubts-steer-a'])
        self.assertEqual(down[-3:], ['down', '-t', '20'])
        self.assertEqual(self.calls('budget.log')[1], ['release', '--label', 'ubts-steer-a'])
        self.assertFalse((self.state_dir / stack.LOCK_NAME).exists())
        rec = json.loads(receipt.read_text(encoding='utf-8'))
        self.assertEqual((rec['leftover_containers'], rec['leftover_networks']), (0, 0))
        self.assertEqual(rec['lock']['env']['UBTS_STEER_SCENE'], 'w0_force')    # the values of "up" are kept
        for call in self.calls():
            self.assertNotIn('-v', call)                                         # never removes volumes
            self.assertNotIn('--volumes', call)
            if call[:1] == ['compose']:
                self.assertEqual(call[1:3], ['-p', 'ubts-steer-a'])

    def test_override_files_follow_the_stack_file_in_every_command(self):
        override = self.tmp / 'compose.field.yaml'
        override.write_bytes(b'services: {}' + bytes([10]))
        self.assertEqual(self.main('up', '--project', 'ubts-steer-a', '--override', str(self.tmp / 'missing.yaml')),
                         stack.EXIT_REFUSED)
        self.assertEqual(self.compose_calls(), [])
        self.assertEqual(self.main('up', '--project', 'ubts-steer-a', '--override', str(override)), 0)
        self.assertEqual(self.main('run', '--project', 'ubts-steer-a', '--', 'true'), 0)
        self.assertEqual(self.main('down', '--project', 'ubts-steer-a'), 0)
        calls = self.compose_calls()
        self.assertEqual(len(calls), 3)
        for call in calls:
            files = [call[i + 1] for i, a in enumerate(call) if a == '-f']
            self.assertEqual(len(files), 2)
            self.assertTrue(files[0].endswith('compose.ubts-steer.yaml'))
            self.assertEqual(files[1], str(override.resolve()))

    def test_one_stack_at_a_time(self):
        self.assertEqual(self.main('up', '--project', 'ubts-steer-a'), 0)
        self.assertEqual(self.main('up', '--project', 'ubts-steer-b'), stack.EXIT_REFUSED)
        self.assertEqual([c[2] for c in self.compose_calls()], ['ubts-steer-a'])
        self.assertEqual([c[2] for c in self.calls('budget.log')], ['ubts-steer-a'])

    def test_a_job_cannot_take_down_or_use_another_projects_stack(self):
        self.assertEqual(self.main('up', '--project', 'ubts-steer-a'), 0)
        before = len(self.compose_calls())
        for command in (('down', '--project', 'ubts-steer-b'), ('run', '--project', 'ubts-steer-b', '--', 'true'),
                        ('ps', '--project', 'ubts-steer-b')):
            with self.subTest(command=command[0]):
                self.assertEqual(self.main(*command), stack.EXIT_REFUSED)
        self.assertEqual(len(self.compose_calls()), before)                      # no compose call at all
        self.assertEqual(self.running(), {'ubts-steer-a': 3})                    # project a still runs
        self.assertTrue((self.state_dir / stack.LOCK_NAME).is_file())
        # without a lock (a stack started by hand or by another helper) nothing is stopped either
        (self.state_dir / stack.LOCK_NAME).unlink()
        self.assertEqual(self.main('down', '--project', 'ubts-steer-a'), stack.EXIT_REFUSED)
        self.assertEqual(self.running(), {'ubts-steer-a': 3})

    def test_running_containers_of_another_steering_project_block_up(self):
        for other in ('ubts-steer', 'ubts-steer-x'):
            with self.subTest(other=other):
                (self.tmp / 'docker.json').write_text(json.dumps({'running': {other: 2, 'unrelated': 1}}),
                                                      encoding='utf-8')
                self.assertEqual(self.main('up', '--project', 'ubts-steer-a'), stack.EXIT_REFUSED)
        self.assertEqual(self.compose_calls(), [])
        (self.tmp / 'docker.json').write_text(json.dumps({'running': {'unrelated': 1}}), encoding='utf-8')
        self.assertEqual(self.main('up', '--project', 'ubts-steer-a'), 0)    # other projects do not block

    def test_budget_timeout_starts_nothing(self):
        os.environ['FAKE_BUDGET_EXIT'] = '3'
        self.assertEqual(self.main('up', '--project', 'ubts-steer-a', '--budget-wait-s', '1'), stack.EXIT_BUDGET)
        self.assertEqual(self.compose_calls(), [])
        self.assertFalse((self.state_dir / stack.LOCK_NAME).exists())

    def test_hold_file_and_host_load_record(self):
        hold = self.tmp / 'HOLD'
        hold.write_text('{}', encoding='utf-8')
        os.environ['UBTS_STEER_HOLD_FILE'] = str(hold)
        self.assertEqual(self.main('up', '--project', 'ubts-steer-a'), stack.EXIT_REFUSED)
        hold.unlink()
        self.assertEqual(self.main('up', '--project', 'ubts-steer-a', '--load-check', str(self.tmp / 'none.json')),
                         stack.EXIT_REFUSED)
        self.assertEqual(self.compose_calls(), [])
        check = self.tmp / 'load.json'
        check.write_text('{"ok": true}', encoding='utf-8')
        self.assertEqual(self.main('up', '--project', 'ubts-steer-a', '--load-check', str(check)), 0)
        lock = json.loads((self.state_dir / stack.LOCK_NAME).read_text(encoding='utf-8'))
        self.assertEqual(lock['load_check']['path'], str(check))
        self.assertEqual(len(lock['load_check']['sha256']), 64)

    def test_leftovers_after_down_keep_the_lock(self):
        receipt = self.tmp / 'receipt.json'
        self.assertEqual(self.main('up', '--project', 'ubts-steer-a'), 0)
        os.environ['FAKE_DOCKER_DOWN_LEAVES'] = '1'
        self.assertEqual(self.main('down', '--project', 'ubts-steer-a', '--receipt', str(receipt)), stack.EXIT_DOCKER)
        rec = json.loads(receipt.read_text(encoding='utf-8'))
        self.assertEqual((rec['leftover_containers'], rec['leftover_networks']), (1, 1))
        self.assertTrue((self.state_dir / stack.LOCK_NAME).is_file())             # a new up stays blocked
        self.assertEqual(self.main('up', '--project', 'ubts-steer-b'), stack.EXIT_REFUSED)


if __name__ == '__main__':
    unittest.main()
