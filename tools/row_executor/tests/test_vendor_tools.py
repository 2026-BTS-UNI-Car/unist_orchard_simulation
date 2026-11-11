"""tools/steer_sim/regen_vendor.py and check_vendor_sync.py on a temporary copy of the package + SIM-tree sources.

The copy lives in the system temporary folder (never the work tree). check_vendor_sync must pass on an unchanged copy
and fail, with a named problem, on every kind of drift; regen_vendor --worktree must bring the copy back in sync, keep
the derivation of the last committed vendor, and be idempotent. The --commit mode needs git and is checked by hand
(docs/steering/EXECUTOR_IMPORT_KO.md). Runs under pytest and plain unittest.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

import rowexec_testlib as L

REPO = os.path.dirname(L.TOOLS)
PACKAGE = os.path.join(L.TOOLS, 'row_executor')
TOOLS = ('tools/steer_sim/regen_vendor.py', 'tools/steer_sim/check_vendor_sync.py')


def run(root, script, *args):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    result = subprocess.run([sys.executable, '-B', os.path.join(root, *script.split('/'))] + list(args) + ['--json'],
                            cwd=root, capture_output=True, text=True, env=env, timeout=300)
    try:
        report = json.loads(result.stdout)
    except ValueError:
        report = dict(raw=result.stdout[-2000:], stderr=result.stderr[-2000:])
    return result.returncode, report


class VendorToolsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(os.path.join(PACKAGE, 'vendor', 'SOURCE_MANIFEST.json'), 'rb') as fh:
            cls.manifest = json.loads(fh.read().decode('utf-8'))

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='ubts-rowexec-vendor-')
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.root = os.path.join(self.tmp, 'tree')
        shutil.copytree(PACKAGE, os.path.join(self.root, 'tools', 'row_executor'),
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        for rel in TOOLS + tuple(e['source_path'] for e in self.manifest['files']):
            target = os.path.join(self.root, *rel.split('/'))
            os.makedirs(os.path.dirname(target), exist_ok=True)
            shutil.copyfile(os.path.join(REPO, *rel.split('/')), target)

    def path(self, rel):
        return os.path.join(self.root, *rel.split('/'))

    def append(self, rel, data=b'\n# drift\n'):
        with open(self.path(rel), 'ab') as fh:
            fh.write(data)

    def check(self):
        return run(self.root, 'tools/steer_sim/check_vendor_sync.py')

    def assert_problem(self, needle):
        code, report = self.check()
        self.assertEqual(code, 1, report)
        self.assertTrue(any(needle in p for p in report.get('problems', [])), report)

    def test_unchanged_copy_passes(self):
        code, report = self.check()
        self.assertEqual(code, 0, report)
        self.assertEqual(report['vendored_files'], len(self.manifest['files']))

    def test_vendored_file_edit_is_found(self):
        self.append('tools/row_executor/vendor/closed_loop_copy/controllers.py')
        self.assert_problem('differs from the manifest sha256/bytes: closed_loop_copy/controllers.py')
        self.assert_problem('differs from the SIM tree sim/ubts/closed_loop/controllers.py')
        self.assert_problem('verify_vendor / executor_main.verify_sources failed')

    def test_sim_tree_change_is_found_and_regen_fixes_it(self):
        self.append('sim/ubts/closed_loop/estimator.py')
        self.assert_problem('vendored closed_loop_copy/estimator.py differs from the SIM tree')
        code, report = run(self.root, 'tools/steer_sim/regen_vendor.py', '--worktree')
        self.assertEqual(code, 0, report)
        self.assertEqual(report['written'], ['closed_loop_copy/estimator.py'])
        self.assertTrue(report['verify']['ok'], report)
        code, check = self.check()
        self.assertEqual(code, 0, check)
        with open(self.path('tools/row_executor/vendor/SOURCE_MANIFEST.json'), 'rb') as fh:
            first = fh.read()
        manifest = json.loads(first.decode('utf-8'))
        self.assertEqual(manifest['source_commit'], 'WORKTREE-UNCOMMITTED')
        if self.manifest['source_commit'] == 'WORKTREE-UNCOMMITTED':
            self.assertEqual(manifest['derivation'], self.manifest['derivation'])     # still names the last commit
        else:
            self.assertEqual(manifest['derivation']['previous_source_commit'], self.manifest['source_commit'])
        self.assertNotIn(b'\r\n', first)
        code, again = run(self.root, 'tools/steer_sim/regen_vendor.py', '--worktree')
        self.assertEqual((code, again['written'], again['manifest_changed']), (0, [], False), again)

    def test_wrong_manifest_digest_is_found(self):
        rel = 'tools/row_executor/vendor/SOURCE_MANIFEST.json'
        with open(self.path(rel), 'rb') as fh:
            manifest = json.loads(fh.read().decode('utf-8'))
        manifest['files'][0]['sha256'] = '0' * 64
        with open(self.path(rel), 'wb') as fh:
            fh.write((json.dumps(manifest, indent=1, sort_keys=True) + '\n').encode('utf-8'))
        self.assert_problem('differs from the manifest sha256/bytes: %s' % manifest['files'][0]['vendored_path'])

    def test_package_drift_is_found_like_verify_sources(self):
        with open(self.path('tools/row_executor/extra_note.txt'), 'wb') as fh:
            fh.write(b'x\n')
        self.assert_problem('SHA256SUMS does not list: extra_note.txt')
        os.remove(self.path('tools/row_executor/extra_note.txt'))
        self.append('tools/row_executor/guard.py')
        self.assert_problem('SHA256SUMS differs for: guard.py')
        os.remove(self.path('tools/row_executor/body_mask_fit.py'))
        self.assert_problem('SHA256SUMS lists a missing file: body_mask_fit.py')

    def test_unlisted_vendor_file_is_found_and_refused_by_regen(self):
        with open(self.path('tools/row_executor/vendor/closed_loop_copy/stray.py'), 'wb') as fh:
            fh.write(b'# not listed\n')
        self.assert_problem('vendor/ holds a file the manifest does not list: closed_loop_copy/stray.py')
        code, report = run(self.root, 'tools/steer_sim/regen_vendor.py', '--worktree')
        self.assertNotEqual(code, 0)

    def test_regen_adds_a_new_sim_file(self):
        source = 'sim/ubts/closed_loop/new_part.py'
        with open(self.path(source), 'wb') as fh:
            fh.write(b'VALUE = 1\n')
        code, report = run(self.root, 'tools/steer_sim/regen_vendor.py', '--worktree', '--add', source,
                           'closed_loop_copy/new_part.py')
        self.assertEqual(code, 0, report)
        self.assertIn('closed_loop_copy/new_part.py', report['written'])
        code, check = self.check()
        self.assertEqual(code, 0, check)
        self.assertEqual(check['vendored_files'], len(self.manifest['files']) + 1)

    def test_regen_moves_a_vendored_file_and_prunes_the_old_folder(self):
        code, report = run(self.root, 'tools/steer_sim/regen_vendor.py', '--worktree', '--remove',
                           'orchard_perception_copy/scan_corridor.py', '--remove', 'orchard_perception_copy/__init__.py',
                           '--add', 'sim/ubts/orchard_perception/scan_corridor.py', 'perception_moved/scan_corridor.py',
                           '--add', 'sim/ubts/orchard_perception/__init__.py', 'perception_moved/__init__.py')
        self.assertEqual(code, 0, report)
        self.assertFalse(os.path.exists(self.path('tools/row_executor/vendor/orchard_perception_copy')))
        self.assertTrue(os.path.isfile(self.path('tools/row_executor/vendor/perception_moved/scan_corridor.py')))


class CheckGitTest(unittest.TestCase):
    """Review P4R-12: with git present in a work tree, a manifest commit or path git does not know is a problem; only a
    missing git or a folder outside a work tree is a note."""

    @classmethod
    def setUpClass(cls):
        import importlib.util
        spec = importlib.util.spec_from_file_location('check_vendor_sync_under_test',
                                                      os.path.join(REPO, 'tools', 'steer_sim', 'check_vendor_sync.py'))
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)
        cls.manifest = dict(source_commit='0' * 40, files=[dict(vendored_path='x/y.py', source_path='sim/ubts/x/y.py')])

    def fake(self, rev_parse=(0, b'true\n'), cat_file=(0, b''), show=(0, b'data')):
        replies = {'rev-parse': rev_parse, 'cat-file': cat_file, 'show': show}

        def run(args, **kwargs):
            code, out = replies[args[1]]
            return subprocess.CompletedProcess(args, code, out, b'fatal: test reply')
        return run

    def test_unknown_commit_is_a_problem(self):
        problems, _ = self.module.check_git(self.manifest, True, run=self.fake(cat_file=(128, b'')))
        self.assertTrue(problems and 'is not a commit of this repository' in problems[0], problems)

    def test_unknown_path_is_a_problem(self):
        problems, _ = self.module.check_git(self.manifest, True, run=self.fake(show=(128, b'')))
        self.assertTrue(problems and 'git show' in problems[0] and 'failed' in problems[0], problems)

    def test_no_git_or_no_work_tree_is_a_note(self):
        def missing(args, **kwargs):
            raise FileNotFoundError('git')
        problems, note = self.module.check_git(self.manifest, True, run=missing)
        self.assertEqual(problems, [])
        self.assertIn('not possible', note)
        problems, note = self.module.check_git(self.manifest, True, run=self.fake(rev_parse=(128, b'')))
        self.assertEqual(problems, [])
        self.assertIn('not possible', note)


if __name__ == '__main__':
    unittest.main()
