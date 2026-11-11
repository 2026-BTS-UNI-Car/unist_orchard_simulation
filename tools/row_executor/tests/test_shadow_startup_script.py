"""Focused launcher lifecycle checks using only stub shell commands, never Docker."""
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import rowexec_testlib as L
from test_run_script import preferred_shell

SCRIPT = Path(L.TOOLS) / 'row_executor/run_executor.sh'
CID = 'a' * 64
DOCKER = '''#!/bin/sh
echo "$*" >> "$STUB_CALLS"
case "$1" in
  image|ps|stop) exit 0 ;;
  run)
    [ "$STUB_SCENARIO" != run_fail ] || exit 125
    [ "$STUB_SCENARIO" != bad_id ] || { echo bad-id; exit 0; }
    echo aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa ;;
  logs)
    [ "$STUB_SCENARIO" != logs_fail ] || { echo preserved-log-error >&2; exit 1; }
    echo 'operator page: http://127.0.0.1:8097/?t=PRIVATE_TEST_ONLY' ;;
  inspect)
    n=0; [ ! -f "$STUB_INSPECT" ] || n=$(cat "$STUB_INSPECT")
    n=$((n+1)); echo "$n" > "$STUB_INSPECT"
    [ "$STUB_SCENARIO" != inspect_fail ] || exit 1
    if [ "$STUB_SCENARIO" = exited ] || { [ "$STUB_SCENARIO" = late_exit ] && [ "$n" -ge 2 ]; }; then
      echo false
    else echo true; fi ;;
  *) exit 91 ;;
esac
'''


def shell_path(p):
    return Path(p).as_posix()


class ShadowStartupScriptTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.shell = preferred_shell()
        if not cls.shell:
            raise unittest.SkipTest('POSIX shell unavailable')

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='ubts-shadow-start-')
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        package = self.base / 'src/row_executor'; package.mkdir(parents=True)
        (package / 'a.txt').write_bytes(b'x\n')
        (package / 'SHA256SUMS').write_bytes((hashlib.sha256(b'x\n').hexdigest() + '  a.txt\n').encode())
        (self.base / 'config').mkdir()
        (self.base / 'config/config.json').write_bytes(b'{"profile":"field","motor_output_enabled":false}\n')
        self.bin = self.base / 'bin'; self.bin.mkdir()
        for name, body in (('docker', DOCKER), ('sleep', '#!/bin/sh\nexit 0\n')):
            path = self.bin / name; path.write_bytes(body.encode()); path.chmod(0o755)
        # Git Bash includes the standard sha256sum/grep/tail tools. The first PATH
        # entry is our executable stub; there is no fallback Docker invocation.
        self.calls = self.base / 'calls.txt'
        self.env = dict(os.environ, UBTS_ROW_EXECUTOR_HOME=shell_path(self.base),
                        UBTS_ROW_EXECUTOR_IMAGE='sha256:' + '0' * 64,
                        STUB_CALLS=shell_path(self.calls), STUB_INSPECT=shell_path(self.base / 'inspect-count'),
                        PATH=str(self.bin) + os.pathsep + os.environ.get('PATH', ''))

    def run_case(self, scenario, mode='shadow'):
        env = dict(self.env, STUB_SCENARIO=scenario)
        result = subprocess.run([self.shell, shell_path(SCRIPT), mode], env=env,
                                capture_output=True, text=True, encoding='utf-8', timeout=15)
        calls = self.calls.read_text(encoding='utf-8').splitlines() if self.calls.exists() else []
        return result, calls

    @property
    def log(self):
        return self.base / 'logs' / ('start-' + CID + '.log')

    def test_exited_before_logs_is_failure_with_retained_private_evidence(self):
        result, calls = self.run_case('exited')
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertNotIn('started ubts-row-executor', result.stdout)
        self.assertIn('exited before startup confirmation', result.stderr)
        self.assertIn('PRIVATE_TEST_ONLY', self.log.read_text())
        self.assertFalse(any(c.startswith(('rm ', 'stop ')) for c in calls))

    def test_running_requires_both_state_checks_and_has_no_bridge_mount(self):
        result, calls = self.run_case('running')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(sum(c.startswith('inspect ') for c in calls), 2)
        run = next(c for c in calls if c.startswith('run '))
        self.assertNotIn('--rm', run)
        self.assertNotIn('/run/ubts-rover', run)
        self.assertNotIn('--motor-output', run)
        self.assertIn('started ubts-row-executor (shadow)', result.stdout)

    def test_log_failure_cannot_be_hidden_by_tail_success(self):
        result, _ = self.run_case('logs_fail')
        self.assertEqual(result.returncode, 2)
        self.assertIn('logs unavailable', result.stderr)
        self.assertIn('preserved-log-error', self.log.read_text())

    def test_exit_during_log_read_is_not_success(self):
        result, _ = self.run_case('late_exit')
        self.assertEqual(result.returncode, 2)
        self.assertIn('exited while reading startup logs', result.stderr)

    def test_missing_inspect_and_failed_run_are_not_success(self):
        for scenario in ('inspect_fail', 'run_fail'):
            with self.subTest(scenario=scenario):
                result, _ = self.run_case(scenario)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn('started ubts-row-executor', result.stdout)

    def test_invalid_container_id_is_not_accepted(self):
        result, _ = self.run_case('bad_id')
        self.assertEqual(result.returncode, 2)
        self.assertFalse(self.log.exists())

    def test_startup_evidence_is_never_overwritten(self):
        self.log.parent.mkdir(); self.log.write_bytes(b'KEEP-OLD-EVIDENCE')
        result, _ = self.run_case('running')
        self.assertEqual(result.returncode, 2)
        self.assertEqual(self.log.read_bytes(), b'KEEP-OLD-EVIDENCE')

    def test_stop_command_still_stops_only_the_original_named_service(self):
        result, calls = self.run_case('running', mode='stop')
        self.assertEqual(result.returncode, 0)
        self.assertEqual(calls, ['stop -t 5 ubts-row-executor'])


if __name__ == '__main__':
    unittest.main()
