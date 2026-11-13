"""The dashboard copy and its wiring in the steering stack (D-13, docs/steering/SIM_DASHBOARD_KO.md).

  CopyTest      dashboard/ matches dashboard/DASHBOARD_SOURCE.json (tools/steer_sim/sync_dashboard.py --check); with git,
                the committed blob ids of the copied files are the dashboard repository's blob ids.
  SyncToolTest  the tool on a small temporary git repository: copies the three folders byte for byte, removes what the
                commit does not have under them, leaves scripts/ alone, writes the same manifest when run again, and
                --check finds a changed byte, an extra file and a manifest that names another commit.
  WiringTest    compose.ubts-steer.yaml, the Dockerfile and the executor container agree: the dashboard runs in the sim
                network namespace (the executor's operator page is 127.0.0.1 there), the sim service publishes the page,
                the relay URL and the token file are the executor's, the extension modules exist in dashboard_sim/.
No Docker, temporary folders only. Placeholder-based SIM results only, never field validation.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools' / 'steer_sim'))
import sync_dashboard as SD  # noqa: E402

GIT = shutil.which('git')


def git(cwd, *args):
    return subprocess.run(['git', '-c', 'core.autocrlf=false', *args], cwd=str(cwd), capture_output=True, check=True)


class CopyTest(unittest.TestCase):
    def test_the_copy_matches_its_manifest(self):
        report = SD.check(ROOT / 'dashboard')
        self.assertTrue(report['ok'], report)
        self.assertGreater(report['file_count'], 100)
        manifest = json.loads((ROOT / 'dashboard' / SD.MANIFEST_NAME).read_bytes())
        self.assertEqual(manifest['repository'], SD.REPOSITORY)
        self.assertRegex(manifest['commit'], r'^[0-9a-f]{40}$')
        for path in ('dashboard_server/app.py', 'dashboard_server/extensions.py', 'dashboard_server/executor_proxy.py',
                     'static/index.html', 'static/steer/steer-control-rules.js', 'static/steer/auto-panel.mjs'):
            self.assertIn(path, manifest['files'])

    @unittest.skipUnless(GIT, 'git not found')
    def test_committed_blobs_are_the_dashboard_repository_blobs(self):
        listed = subprocess.run(['git', 'ls-files', '-s', '--'] + ['dashboard/' + r for r in SD.SYNCED_ROOTS],
                                cwd=str(ROOT), capture_output=True, text=True)
        if listed.returncode != 0 or not listed.stdout.strip():
            self.skipTest('not a git checkout with the copy staged or committed')
        index = {}
        for line in listed.stdout.splitlines():
            meta, path = line.split('\t', 1)
            mode, blob, _ = meta.split()
            index[path[len('dashboard/'):]] = (mode, blob)
        files = json.loads((ROOT / 'dashboard' / SD.MANIFEST_NAME).read_bytes())['files']
        self.assertEqual(sorted(index), sorted(files))
        for path, row in files.items():
            self.assertEqual(index[path], (row['mode'], row['blob']), path)

    def test_scripts_are_not_part_of_the_copy(self):
        files = json.loads((ROOT / 'dashboard' / SD.MANIFEST_NAME).read_bytes())['files']
        self.assertFalse([p for p in files if not p.startswith(tuple(r + '/' for r in SD.SYNCED_ROOTS))])
        self.assertTrue((ROOT / 'dashboard' / 'scripts' / 'steer_rover_bridge.py').is_file())


@unittest.skipUnless(GIT, 'git not found')
class SyncToolTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix='ubts_sync_'))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.src = self.tmp / 'dashboard_repo'
        self.src.mkdir()
        git(self.src, 'init', '-q')
        git(self.src, 'config', 'user.name', 'test')
        git(self.src, 'config', 'user.email', 'test@example.invalid')
        self.write(self.src, {'config/dashboard.json': b'{"a": 1}\n', 'static/app.js': b'console.log(1);\n',
                              'static/img/logo.png': b'\x89PNG\r\n\x1a\n\x00binary',
                              'dashboard_server/__init__.py': b'', 'dashboard_server/app.py': b'X = 1\n',
                              'tests/test_x.py': b'# not copied\n', 'README.md': b'not copied\n'})
        git(self.src, 'add', '-A')
        git(self.src, 'commit', '-q', '-m', 'one')
        self.first = git(self.src, 'rev-parse', 'HEAD').stdout.decode().strip()
        self.target = self.tmp / 'dashboard'
        self.write(self.target, {'static/old-sim-page.js': b'old\n', 'dashboard_server/simulation.py': b'old\n',
                                 'scripts/lidar_ws_bridge.py': b'kept\n', 'UPSTREAM.md': b'kept\n'})

    @staticmethod
    def write(root, files):
        for rel, data in files.items():
            path = root / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)

    def test_copy_removes_what_the_commit_lacks_and_keeps_scripts(self):
        report = SD.sync(self.src, 'HEAD', self.target)
        self.assertEqual((report['add'], report['change'], report['remove']), (5, 0, 2))
        self.assertEqual((self.target / 'static/img/logo.png').read_bytes(), b'\x89PNG\r\n\x1a\n\x00binary')
        self.assertFalse((self.target / 'static/old-sim-page.js').exists())
        self.assertFalse((self.target / 'dashboard_server/simulation.py').exists())
        self.assertEqual((self.target / 'scripts/lidar_ws_bridge.py').read_bytes(), b'kept\n')
        self.assertFalse((self.target / 'tests').exists() or (self.target / 'README.md').exists())
        manifest = json.loads((self.target / SD.MANIFEST_NAME).read_bytes())
        self.assertEqual(manifest['commit'], self.first)
        self.assertEqual(manifest['file_count'], 5)
        self.assertTrue(SD.check(self.target, self.src)['ok'])

    def test_running_again_gives_the_same_bytes(self):
        SD.sync(self.src, 'HEAD', self.target)
        first = (self.target / SD.MANIFEST_NAME).read_bytes()
        again = SD.sync(self.src, self.first[:10], self.target)
        self.assertEqual((again['add'], again['change'], again['remove'], again['manifest_changed']), (0, 0, 0, False))
        self.assertEqual((self.target / SD.MANIFEST_NAME).read_bytes(), first)

    def test_check_finds_changes_extras_and_another_commit(self):
        SD.sync(self.src, 'HEAD', self.target)
        (self.target / 'static/app.js').write_bytes(b'console.log(2);\n')
        (self.target / 'static/extra.js').write_bytes(b'x\n')
        report = SD.check(self.target)
        self.assertFalse(report['ok'])
        self.assertEqual((report['changed'], report['extra']), (['static/app.js'], ['static/extra.js']))
        SD.sync(self.src, 'HEAD', self.target)
        self.assertTrue(SD.check(self.target)['ok'])
        self.write(self.src, {'static/app.js': b'console.log(3);\n'})
        git(self.src, 'commit', '-q', '-am', 'two')
        manifest = json.loads((self.target / SD.MANIFEST_NAME).read_bytes())
        self.assertEqual(manifest['commit'], self.first)
        self.assertTrue(SD.check(self.target, self.src)['ok'], 'the manifest names the first commit')
        dry = SD.sync(self.src, 'HEAD', self.target, dry_run=True)
        self.assertEqual((dry['change'], dry['manifest_changed']), (1, True))
        self.assertEqual((self.target / 'static/app.js').read_bytes(), b'console.log(1);\n', 'dry run writes nothing')

    def test_a_hand_edited_manifest_is_refused(self):
        SD.sync(self.src, 'HEAD', self.target)
        path = self.target / SD.MANIFEST_NAME
        path.write_bytes(path.read_bytes().replace(b'\n', b'\r\n'))
        self.assertFalse(SD.check(self.target)['ok'])


class WiringTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import yaml
        except ImportError:
            raise unittest.SkipTest('PyYAML not installed')
        cls.compose = yaml.safe_load((ROOT / 'compose.ubts-steer.yaml').read_text(encoding='utf-8'))
        cls.services = cls.compose['services']
        cls.env = cls.services['dashboard']['environment']

    def test_dashboard_is_in_the_sim_network_namespace_and_sim_publishes_the_page(self):
        dashboard, sim = self.services['dashboard'], self.services['sim']
        self.assertEqual(dashboard['network_mode'], 'service:sim')
        self.assertNotIn('ports', dashboard)
        self.assertEqual(sim['ports'], ['127.0.0.1:18090:8080'])
        self.assertEqual(self.env['DASHBOARD_PORT'], '8080')
        self.assertEqual(self.services['executor']['network_mode'], 'service:sim')

    def test_relay_points_at_the_executor_operator_page_and_token(self):
        sys.path.insert(0, str(ROOT / 'sim' / 'ubts'))
        try:
            from steering import exec_container_main as X
        finally:
            sys.path.remove(str(ROOT / 'sim' / 'ubts'))
        self.assertEqual(self.env['UBTS_EXECUTOR_URL'], 'http://%s:%d' % (X.BIND_HOST, X.BIND_PORT))
        self.assertEqual(self.env['UBTS_EXECUTOR_TOKEN_FILE'], X.EXEC_DIR + '/log/operator_token.txt')
        mounts = self.services['dashboard']['volumes']
        self.assertIn('control:/run/ubts-steer', mounts)
        self.assertTrue(X.EXEC_DIR.startswith('/run/ubts-steer/'))
        self.assertTrue(self.env['UBTS_EXECUTOR_TUNING_LOG'].startswith('/runtime/'))

    def test_extension_settings_name_files_of_dashboard_sim(self):
        self.assertEqual(self.env['DASHBOARD_MODE'] if 'DASHBOARD_MODE' in self.env else
                         self.compose['x-environment']['DASHBOARD_MODE'], 'simulation')
        self.assertEqual(self.env['DASHBOARD_VEHICLE'], 'front_steer')
        module, _, cls = self.env['DASHBOARD_PROVIDER'].partition(':')
        self.assertTrue((ROOT / 'dashboard_sim' / (module.replace('.', '/') + '.py')).is_file())
        self.assertIn('class %s(' % cls, (ROOT / 'dashboard_sim' / (module.replace('.', '/') + '.py')).read_text(
            encoding='utf-8'))
        self.assertTrue((ROOT / 'dashboard_sim' / (self.env['DASHBOARD_EXTENSION'].replace('.', '/') + '.py')).is_file())
        self.assertEqual(self.env['DASHBOARD_EXTRA_STATIC'], '/opt/ubts/dashboard_sim/static')
        self.assertTrue((ROOT / 'dashboard_sim' / 'static' / 'sim-info.js').is_file())
        self.assertIn('/opt/ubts/dashboard_sim', self.env['PYTHONPATH'].split(':'))
        for name in ('LIDAR', 'CAMERA', 'IMU', 'GPS'):
            self.assertTrue(self.env[name + '_WS_UPSTREAM'].startswith('ws://127.0.0.1:'), name)

    def test_image_copies_dashboard_sim_and_checks_the_copy(self):
        dockerfile = (ROOT / 'docker' / 'Dockerfile.ubts-steer').read_text(encoding='utf-8')
        self.assertIn('COPY dashboard_sim /opt/ubts/dashboard_sim', dockerfile)
        self.assertIn('DASHBOARD_SOURCE.json', dockerfile)
        self.assertIn('dashboard_server/native_raster.cc', dockerfile)
        self.assertTrue((ROOT / 'dashboard' / 'dashboard_server' / 'native_raster.cc').is_file())
        ignore = (ROOT / 'docker' / 'Dockerfile.ubts-steer.dockerignore').read_text(encoding='utf-8').splitlines()
        self.assertIn('!dashboard_sim/**', ignore)

    def test_checks_reach_the_dashboard_in_the_namespace(self):
        text = (ROOT / 'sim' / 'ubts' / 'steering' / 'p6a_checks.py').read_text(encoding='utf-8')
        self.assertIn("p.add_argument('--dashboard-url', default='http://127.0.0.1:8080')", text)


if __name__ == '__main__':
    unittest.main()
