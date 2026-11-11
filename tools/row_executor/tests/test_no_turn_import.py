"""The steering-branch import of the row executor carries no headland-turn code (U-turn is out of scope).

Importing executor_main, sim_in_loop or single_row_sim loads no turn_* module (checked in a fresh interpreter per
module, so other tests cannot hide or cause it), no package source names a turn_* module in an import, and the turn
files are absent. A 'turn_observe' request is refused in every mode. sim_in_loop / single_row_sim need the SIM tree
(numpy, scipy); they are skipped with that reason where scipy cannot be imported (WSL Python 3.10, DECISIONS R10).
Runs under pytest and plain unittest. No file is written.
"""
import ast
import json
import os
import subprocess
import sys
import unittest

import rowexec_testlib as L

PACKAGE = os.path.join(L.TOOLS, 'row_executor')
TURN_FILES = ('turn_assist.py', 'turn_bridge.py', 'turn_observation.py', 'turn_real_replay.py', 'turn_sim_in_loop.py',
              'TURN_ASSIST.txt', os.path.join('tests', 'turn_testlib.py'))
PROBE = r'''
import json, sys
sys.dont_write_bytecode = True
sys.path.insert(0, sys.argv[1])
import importlib
importlib.import_module(sys.argv[2])
loaded = sorted(n for n in sys.modules if n.split('.')[-1].startswith('turn_'))
print(json.dumps(dict(loaded=loaded)))
'''


def have_scipy():
    try:
        import scipy.spatial  # noqa: F401
        return True
    except Exception:            # the WSL 3.10 scipy is built for another numpy (DECISIONS R10)
        return False


def probe(module):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    result = subprocess.run([sys.executable, '-B', '-c', PROBE, L.TOOLS, module], capture_output=True, text=True,
                            env=env, timeout=300)
    if result.returncode:
        raise AssertionError('import of %s failed:\n%s' % (module, result.stderr[-3000:]))
    return json.loads(result.stdout.strip().splitlines()[-1])['loaded']


class NoTurnModuleLoaded(unittest.TestCase):
    def test_executor_main_loads_no_turn_module(self):
        self.assertEqual(probe('row_executor.executor_main'), [])

    @unittest.skipUnless(have_scipy(), 'scipy not importable here (the SIM tree estimator needs it)')
    def test_sim_in_loop_loads_no_turn_module(self):
        self.assertEqual(probe('row_executor.sim_in_loop'), [])

    @unittest.skipUnless(have_scipy(), 'scipy not importable here (the SIM tree estimator needs it)')
    def test_single_row_sim_loads_no_turn_module(self):
        self.assertEqual(probe('row_executor.single_row_sim'), [])


class NoTurnSource(unittest.TestCase):
    def test_turn_files_are_absent(self):
        present = [name for name in TURN_FILES if os.path.exists(os.path.join(PACKAGE, name))]
        self.assertEqual(present, [])

    def test_no_package_source_imports_a_turn_module(self):
        offenders = []
        for root, dirs, files in os.walk(PACKAGE):
            dirs[:] = [d for d in dirs if d != '__pycache__']
            for name in files:
                if not name.endswith('.py'):
                    continue
                path = os.path.join(root, name)
                with open(path, 'rb') as fh:
                    tree = ast.parse(fh.read(), path)
                for node in ast.walk(tree):
                    if isinstance(node, ast.Import):
                        names = [a.name for a in node.names]
                    elif isinstance(node, ast.ImportFrom):
                        names = [node.module or ''] + [a.name for a in node.names]
                    else:
                        continue
                    if any(part.startswith('turn_') for n in names for part in n.split('.')):
                        offenders.append('%s:%d' % (os.path.relpath(path, PACKAGE), node.lineno))
        self.assertEqual(offenders, [])


class TurnObserveRefused(unittest.TestCase):
    def test_refused_in_every_mode(self):
        from row_executor import executor_main as M
        from row_executor import config as C
        loop = M.ExecutorLoop.__new__(M.ExecutorLoop)            # only the request handler is exercised
        for mode, error in ((C.SINGLE_ROW_MODE, 'turn_observe_not_in_single_row_mode'),
                            ('continuous', 'turn_observe_not_available'),
                            ('continuous_side', 'turn_observe_not_available')):
            loop.config = C.ExecutorConfig(mode=mode)
            self.assertEqual(loop._req_turn_observe(0., params={}), dict(ok=False, error=error))


if __name__ == '__main__':
    unittest.main()
