"""row_executor.sim_paths: external locations of the SIM-side tools (environment variable, path config file next to the
steering config or named by UBTS_ROW_EXECUTOR_PATHS, repository-relative defaults; no machine-specific path).

Config files are written to a temporary folder outside the work tree. Runs under pytest and plain unittest.
"""
import json
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import rowexec_testlib  # noqa: F401  (path setup)
from row_executor import sim_paths as P

ENV_NAMES = tuple(P.KEYS.values()) + (P.CONFIG_ENV, P.STEER_CONFIG_ENV)


def clean_env(**values):
    env = {k: v for k, v in os.environ.items() if k not in ENV_NAMES}
    env.update(values)
    return mock.patch.dict(os.environ, env, clear=True)


class SimPathsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='ubts-rowexec-paths-')
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.data = os.path.join(self.tmp, 'data')
        os.makedirs(os.path.join(self.data, 'lidar'))
        self.prereg = os.path.join(self.tmp, 'PREREG.json')
        with open(self.prereg, 'wb') as fh:
            fh.write(b'{}\n')

    def write(self, name, doc):
        path = os.path.join(self.tmp, name)
        with open(path, 'wb') as fh:
            fh.write(json.dumps(doc).encode('utf-8'))
        return path

    def test_environment_variable_wins(self):
        cfg = self.write('cfg.json', dict(schema=P.SCHEMA, single_row_prereg='other.json'))
        with clean_env(UBTS_SINGLE_ROW_PREREG=self.prereg, UBTS_ROW_EXECUTOR_PATHS=cfg):
            value, source = P.lookup('single_row_prereg')
        self.assertEqual(value, self.prereg)
        self.assertIn('UBTS_SINGLE_ROW_PREREG', source)

    def test_named_config_file_string_object_and_relative_values(self):
        cfg = self.write('cfg.json', {'schema': P.SCHEMA, 'single_row_prereg': 'PREREG.json',
                                      'field_manual_1010': {os.name: self.data}})
        with clean_env(UBTS_ROW_EXECUTOR_PATHS=cfg):
            self.assertEqual(P.configured_path('single_row_prereg', 'file'), Path(self.tmp) / 'PREREG.json')
            self.assertEqual(P.configured_path('field_manual_1010', 'dir'), Path(self.data))

    def test_drive_extract_folder_is_its_own_location(self):
        drive = os.path.join(self.tmp, 'drive_rows')
        os.makedirs(drive)
        cfg = self.write('cfg.json', {'schema': P.SCHEMA, 'field_manual_1010': {os.name: self.data},
                                      'field_manual_1010_drive': {os.name: drive}})
        with clean_env(UBTS_ROW_EXECUTOR_PATHS=cfg):
            self.assertEqual(P.configured_path('field_manual_1010_drive', 'dir'), Path(drive))
        with clean_env(UBTS_RD_1010_DRIVE=drive):
            self.assertEqual(P.lookup('field_manual_1010_drive')[0], drive)
        with clean_env():
            self.assertIsNone(P.optional_path('field_manual_1010_drive'))

    def test_sibling_of_the_steering_config_is_used(self):
        steer = self.write('paths.json', dict(schema='ubts.steer_sim.paths.v1'))
        self.write(P.SIBLING_NAME, dict(schema=P.SCHEMA, single_row_prereg=self.prereg))
        with clean_env(UBTS_STEER_PATHS=steer):
            path, how = P.config_file()
            self.assertEqual(path, Path(self.tmp) / P.SIBLING_NAME)
            self.assertIn(P.SIBLING_NAME, how)
            self.assertEqual(P.lookup('single_row_prereg')[0], self.prereg)

    def test_named_file_wins_over_the_sibling(self):
        steer = self.write('paths.json', dict(schema='ubts.steer_sim.paths.v1'))
        self.write(P.SIBLING_NAME, dict(schema=P.SCHEMA, single_row_prereg='sibling.json'))
        named = self.write('named.json', dict(schema=P.SCHEMA, single_row_prereg=self.prereg))
        with clean_env(UBTS_STEER_PATHS=steer, UBTS_ROW_EXECUTOR_PATHS=named):
            self.assertEqual(P.lookup('single_row_prereg')[0], self.prereg)

    def test_not_configured_is_a_clear_file_not_found_error(self):
        with clean_env():
            with self.assertRaises(P.PathNotConfigured) as ctx:
                P.lookup('single_row_prereg')
            self.assertTrue(issubclass(P.PathNotConfigured, FileNotFoundError))
            for needle in ('UBTS_SINGLE_ROW_PREREG', 'single_row_prereg', P.CONFIG_ENV, P.DOC):
                self.assertIn(needle, str(ctx.exception))
            self.assertIsNone(P.optional_path('field_manual_1010'))

    def test_config_without_the_key_falls_to_the_default_or_fails(self):
        cfg = self.write('cfg.json', dict(schema=P.SCHEMA))
        with clean_env(UBTS_ROW_EXECUTOR_PATHS=cfg):
            self.assertEqual(P.lookup('dashboard_dir'), (str(P.REPO_ROOT / 'dashboard'), 'default'))
            with self.assertRaises(P.PathNotConfigured):
                P.lookup('field_manual_1010')

    def test_dashboard_default_is_this_repository(self):
        with clean_env():
            self.assertEqual(P.configured_path('dashboard_dir', 'dir'), P.REPO_ROOT / 'dashboard')
            self.assertTrue((P.REPO_ROOT / 'dashboard' / 'scripts' / 'rover_control_core.py').is_file())

    def test_configured_but_missing_path_names_its_source(self):
        with clean_env(UBTS_RD_1010=os.path.join(self.tmp, 'absent')):
            with self.assertRaises(FileNotFoundError) as ctx:
                P.configured_path('field_manual_1010', 'dir')
            self.assertNotIsInstance(ctx.exception, P.PathNotConfigured)
            self.assertIn('UBTS_RD_1010', str(ctx.exception))
            with self.assertRaises(FileNotFoundError):
                P.optional_path('field_manual_1010', 'dir')       # configured: never silently None

    def test_bad_config_files_are_refused(self):
        for doc in (dict(schema='other.v1'), dict(schema=P.SCHEMA, typo_key='x'),
                    dict(schema=P.SCHEMA, ffmpeg=3), dict(schema=P.SCHEMA, ffmpeg={'dos': 'x'}), [1, 2]):
            cfg = self.write('bad.json', doc)
            with clean_env(UBTS_ROW_EXECUTOR_PATHS=cfg):
                with self.assertRaises(ValueError):
                    P.lookup('ffmpeg')
        with clean_env(UBTS_ROW_EXECUTOR_PATHS=os.path.join(self.tmp, 'missing.json')):
            with self.assertRaises(FileNotFoundError):
                P.lookup('ffmpeg')

    def test_unknown_location(self):
        with self.assertRaises(KeyError):
            P.lookup('maps')

    def test_shell_on_path_skips_the_windows_system32_launcher(self):
        system_root = os.path.join(self.tmp, 'Windows')
        launcher = os.path.join(system_root, 'System32', 'bash.exe')
        found = {'sh': None, 'bash': launcher}
        with mock.patch.object(P.os, 'name', 'nt'), mock.patch.dict(os.environ, {'SystemRoot': system_root}), \
                mock.patch.object(P.shutil, 'which', side_effect=lambda name: found[name]):
            self.assertIsNone(P.posix_shell_on_path())
        found = {'sh': os.path.join(self.tmp, 'git', 'usr', 'bin', 'sh.exe'), 'bash': launcher}
        with mock.patch.object(P.os, 'name', 'nt'), mock.patch.dict(os.environ, {'SystemRoot': system_root}), \
                mock.patch.object(P.shutil, 'which', side_effect=lambda name: found[name]):
            self.assertEqual(P.posix_shell_on_path(), found['sh'])

    def test_measurement_hold_file(self):
        """The replay tools' HOLD_MEASUREMENT file: the SIM tree resolver's 'hold_dir' when the SIM tree is there."""
        self.assertIsNone(P.measurement_hold_file(os.path.join(self.tmp, 'no_sim_tree')))     # a copy without sim/ubts
        hold = os.path.join(self.tmp, 'hold')
        os.makedirs(hold)
        before = set(sys.modules)
        with clean_env(UBTS_HOLD_DIR=hold):
            found = P.measurement_hold_file()
        self.assertEqual(found, Path(hold) / 'HOLD_MEASUREMENT')
        self.assertFalse(found.exists())
        self.assertFalse({'closed_loop', 'closed_loop.external_paths'} & (set(sys.modules) - before))   # by file path
        bare = {k: v for k, v in os.environ.items() if k not in ENV_NAMES + ('UBTS_HOLD_DIR',)}
        with mock.patch.dict(os.environ, bare, clear=True):  # SIM tree present, hold folder not configured: an error
            with self.assertRaises(FileNotFoundError):
                P.measurement_hold_file()
        with clean_env(UBTS_HOLD_DIR=os.path.join(self.tmp, 'missing')):
            with self.assertRaises(FileNotFoundError):
                P.measurement_hold_file()

    def test_no_machine_specific_path_in_the_module(self):
        with open(P.__file__, 'rb') as fh:
            text = fh.read().decode('utf-8')
        for needle in ('D:/', 'C:/', '/mnt/', 'Program Files', 'chocolatey'):
            self.assertNotIn(needle, text)


if __name__ == '__main__':
    unittest.main()
