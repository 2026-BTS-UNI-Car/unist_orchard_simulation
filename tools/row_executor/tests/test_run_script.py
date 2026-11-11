"""Checks of run_executor.sh.

Static checks of the docker run lines, and (review S4) the bench gates executed for real with a STUB `docker`
first on PATH (it only records its arguments; the real docker CLI is never reached and nothing is started):
bench needs --wheels-off-ground-confirmed, refuses a bench.json whose profile is "field" (or anything but
"bench") and refuses when config.json enables motor output (the field setup).
"""
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

import rowexec_testlib as L

SCRIPT = os.path.join(L.TOOLS, 'row_executor', 'run_executor.sh')


def preferred_shell():
    # Windows' System32/bash.exe can be WSL, which cannot open a D:/ argv path. Windows-native stub tests need a
    # Git POSIX shell: row_executor.sim_paths 'posix_shell' (UBTS_POSIX_SHELL, the path config file, or sh / bash on
    # PATH except the System32 launcher). None -> the shell cases are skipped (no built-in machine path).
    from row_executor import sim_paths
    found = sim_paths.optional_path('posix_shell', 'file')
    return None if found is None else str(found)


class RunScriptTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(SCRIPT, 'rb') as fh:
            data = fh.read()
        cls.raw = data
        cls.text = data.decode('utf-8')
        cls.lines = cls.text.splitlines()

    def runs(self, mode):
        """The docker run line(s) of one case branch."""
        start = self.text.index('  %s)\n' % mode)
        end = self.text.index(';;', start)
        return [line for line in self.text[start:end].splitlines() if 'docker run' in line]

    def test_lf_only_and_posix_syntax(self):
        self.assertNotIn(b'\r\n', self.raw)
        sh = preferred_shell()
        if sh is None:
            self.skipTest('no POSIX shell on this host')
        result = subprocess.run([sh, '-n', posix(SCRIPT)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_never_compose_and_common_options(self):
        code = '\n'.join(line for line in self.lines if not line.lstrip().startswith('#'))
        self.assertNotRegex(code, r'docker[ -]compose')
        common = re.search(r'COMMON="([^"]*)"', self.text, re.S).group(1)
        for option in ('--restart no', '--ipc host', '--user 0:0', '--security-opt no-new-privileges',
                       '--entrypoint /bin/bash', ':/opt/row_executor:ro', ':/etc/row_executor:ro'):
            self.assertIn(option, common)
        self.assertNotIn('ubts-rover', common)
        self.assertNotIn('--rm', common)        # SHADOW retains exited-container evidence
        for mode in ('motor', 'bench', 'check'):
            self.assertIn('--rm', self.runs(mode)[0])
        self.assertNotIn('--rm', self.runs('shadow')[0])

    def test_shadow_has_no_bridge_socket_and_motor_has_it(self):
        shadow, motor, check = self.runs('shadow'), self.runs('motor'), self.runs('check')
        self.assertEqual(len(shadow), 1)
        self.assertEqual(len(motor), 1)
        self.assertNotIn('SOCKET_DIR', shadow[0])
        self.assertNotIn('--motor-output', shadow[0])
        self.assertIn('--network host', shadow[0])
        self.assertIn('-v "$SOCKET_DIR:$SOCKET_DIR:ro"', motor[0])     # connect works through a ro bind (S10)
        self.assertIn('--motor-output', motor[0])
        self.assertIn('--network none', check[0])
        self.assertNotIn('SOCKET_DIR', check[0])
        bench = self.runs('bench')
        self.assertEqual(len(bench), 1)
        for needle in ('--profile bench', '/etc/row_executor/bench.json', '--motor-output',
                       '-v "$SOCKET_DIR:$SOCKET_DIR:ro"'):
            self.assertIn(needle, bench[0])
        self.assertEqual(self.text.count('$SOCKET_DIR:$SOCKET_DIR'), 2)
        self.assertEqual(self.text.count('$SOCKET_DIR:$SOCKET_DIR:ro'), 2)

    def test_bench_gates_come_before_any_docker_call(self):
        first_docker = self.text.index('docker image inspect')
        self.assertLess(self.text.index('--wheels-off-ground-confirmed" ] || die'), first_docker)
        first_run = self.text.index('docker run')
        for needle in ('bench_profile" != field ] || die', 'bench_profile" = bench ] || die',
                       '"motor_output_enabled":true'):
            self.assertLess(self.text.index(needle), first_run, needle)
        self.assertIn('delete $CFG_DIR/bench.json', self.text)

    def test_source_and_image_checks_before_any_run(self):
        first_run = self.text.index('docker run')
        for needle in ('sha256sum -c --quiet --strict SHA256SUMS', 'SUMS_SHA256', 'sha256:*)', 'docker image inspect',
                       'already exists'):
            self.assertLess(self.text.index(needle), first_run, needle)


STUB_DOCKER = """#!/bin/sh
echo "$*" >> "$STUB_DOCKER_LOG"
case "$1" in
  image) exit 0 ;;
  ps) exit 0 ;;
esac
exit 0
"""


def posix(path):
    return path.replace(os.sep, '/')


class BenchGateExecutionTest(unittest.TestCase):
    """Runs run_executor.sh with a stub docker; every refusal must happen before any 'docker run'."""

    @classmethod
    def setUpClass(cls):
        cls.sh = preferred_shell()
        if cls.sh is None:
            raise unittest.SkipTest('no POSIX sh / sha256sum on this host')

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='rowexec-sh-')
        self.addCleanup(shutil.rmtree, self.tmp, True)
        base = os.path.join(self.tmp, 'home')
        pkg = os.path.join(base, 'src', 'row_executor')
        os.makedirs(pkg)
        with open(os.path.join(pkg, 'a.txt'), 'wb') as fh:
            fh.write(b'x\n')
        digest = hashlib.sha256(b'x\n').hexdigest()
        with open(os.path.join(pkg, 'SHA256SUMS'), 'wb') as fh:
            fh.write(('%s  a.txt\n' % digest).encode())
        os.makedirs(os.path.join(base, 'config'))
        stub_dir = os.path.join(self.tmp, 'bin')
        os.makedirs(stub_dir)
        stub = os.path.join(stub_dir, 'docker')
        with open(stub, 'wb') as fh:
            fh.write(STUB_DOCKER.encode())
        os.chmod(stub, 0o755)
        self.base, self.log = base, os.path.join(self.tmp, 'docker.log')
        self.env = dict(os.environ, UBTS_ROW_EXECUTOR_HOME=posix(base), UBTS_ROW_EXECUTOR_IMAGE='sha256:' + '0' * 64,
                        STUB_DOCKER_LOG=posix(self.log), PATH=stub_dir + os.pathsep + os.environ.get('PATH', ''))

    def config(self, name, text):
        with open(os.path.join(self.base, 'config', name), 'wb') as fh:
            fh.write(text.encode())

    def run_script(self, *args):
        result = subprocess.run([self.sh, posix(SCRIPT)] + list(args), env=self.env, capture_output=True, text=True,
                                timeout=60)
        calls = []
        if os.path.exists(self.log):
            with open(self.log, encoding='utf-8') as fh:
                calls = fh.read().splitlines()
        self.assertFalse([c for c in calls if c.startswith('run')], calls)      # never starts a container
        return result

    def test_stub_docker_is_the_one_found(self):
        self.config('bench.json', '{"profile": "bench"}')
        self.run_script('bench', '--wheels-off-ground-confirmed')
        self.assertTrue(os.path.exists(self.log))                                # the stub answered

    def test_bench_without_the_flag_is_refused(self):
        self.config('bench.json', '{"profile": "bench"}')
        for args in ((), ('--wheels-off-ground',), ('yes',)):
            with self.subTest(args=args):
                result = self.run_script('bench', *args)
                self.assertEqual(result.returncode, 2)
                self.assertIn('--wheels-off-ground-confirmed', result.stderr)
                self.assertFalse(os.path.exists(self.log))                       # refused before any docker call

    def test_bench_refuses_a_field_profile(self):
        self.config('bench.json', '{\n  "profile" : "field",\n  "motor_output_enabled": true\n}\n')
        result = self.run_script('bench', '--wheels-off-ground-confirmed')
        self.assertEqual(result.returncode, 2)
        self.assertIn('profile "field"', result.stderr)
        self.config('bench.json', '{"profile": "other"}')
        result = self.run_script('bench', '--wheels-off-ground-confirmed')
        self.assertEqual(result.returncode, 2)
        self.assertIn('must have profile "bench"', result.stderr)

    def test_bench_refused_when_the_field_config_enables_motor_output(self):
        self.config('bench.json', '{"profile": "bench", "bench_ack": "WHEELS_OFF_GROUND"}')
        self.config('config.json', '{"profile": "field",\r\n "motor_output_enabled" : true}')
        result = self.run_script('bench', '--wheels-off-ground-confirmed')
        self.assertEqual(result.returncode, 2)
        self.assertIn('enables motor output', result.stderr)

    def test_bench_refused_when_the_field_config_enables_calibration_output(self):
        self.config('bench.json', '{"profile": "bench", "bench_ack": "WHEELS_OFF_GROUND"}')
        self.config('config.json', '{"profile": "field", "motor_output_enabled": false,\n "calib_motor_output_enabled": true}')
        result = self.run_script('bench', '--wheels-off-ground-confirmed')
        self.assertEqual(result.returncode, 2)
        self.assertIn('enables calibration motor output', result.stderr)
        self.assertFalse(os.path.exists(self.log) and 'run' in open(self.log, encoding='utf-8').read().split())

    def test_bench_with_flag_and_bench_profile_reaches_the_socket_check(self):
        self.config('bench.json', '{"profile": "bench", "bench_ack": "WHEELS_OFF_GROUND"}')
        self.config('config.json', '{"profile": "field", "motor_output_enabled": false}')
        result = self.run_script('bench', '--wheels-off-ground-confirmed')
        if os.path.exists('/run/ubts-rover/control.sock'):
            self.skipTest('a real bridge socket exists on this host')
        self.assertEqual(result.returncode, 2)
        self.assertIn('bridge socket', result.stderr)                             # every bench gate passed


if __name__ == '__main__':
    unittest.main()
