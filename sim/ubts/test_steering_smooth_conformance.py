"""Conformance scripts of the SMOOTH r5 steering law (steering/conformance/scripts_smooth_r5;
docs/steering/SMOOTH_R5_SIM_KO.md).

Always: every SMOOTH script validates against ubts.steer_conformance.v1, runs on the profile
front_steer_v2_field_20261116 (schema v5, steer_fw_mode smooth_r5), and the set covers the host-harness scenarios of the r5 firmware (continuous run
with ramp and quiet reads, takeover, reversal pause, stall cancel without force retry, floor boost under a heavy load,
powered offset with QUIET_MS on and off, current bursts, brake on / off, drift correction, the soft limit clamp). The
default script folder still holds only the V3 scripts.

With the native firmware build: every SMOOTH script passes through the runner's own lockstep v2 codec (native) and
through actuator_link.FirmwareSteeringBackend (backend, the package codec) with the same trace, and a wrong
expectation fails. Skipped with the reason while the firmware mode is unavailable (UBTS_STEER_REQUIRE_FIRMWARE turns
the skip into a failure). Builds go to UBTS_STEER_BUILD_DIR or a temporary folder. All values are placeholders:
passing is a placeholder-based SIM result (자리값 기반 SIM 결과), not field validation.
"""
import json
import os
import re
import shutil
import tempfile
import unittest

from steering.conformance import run_conformance as rc
from steering.native import build as native_build

PROFILE = 'front_steer_v2_field_20261116'
NAMES = ('smooth_run_ramp_quiet_brake', 'smooth_takeover_same_direction', 'smooth_reversal_pause',
         'smooth_stall_cancel_no_retry', 'smooth_floor_boost_heavy', 'smooth_powered_offset_quiet_on',
         'smooth_powered_offset_quiet_off', 'smooth_current_bursts', 'smooth_brake_off', 'smooth_drift_correction',
         'smooth_soft_limit_clamp')


def smooth_paths():
    return rc.script_paths(rc.SMOOTH_SCRIPTS_DIR)


class ScriptFiles(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.paths = smooth_paths()
        cls.scripts = [rc.load_script(path) for path in cls.paths]

    def test_the_set_of_scripts(self):
        self.assertEqual(sorted(p.stem for p in self.paths), sorted(NAMES))
        self.assertFalse([p for p in rc.script_paths() if p.stem.startswith('smooth_')])   # V3 folder unchanged

    def test_files_are_utf8_lf_without_bom_and_forbidden_words(self):
        pattern = re.compile(br'(?i)cl' br'aude|anth' br'ropic|cod' br'ex|chat' br'gpt')
        for path in self.paths:
            data = path.read_bytes()
            self.assertFalse(data.startswith(b'\xef\xbb\xbf'), path.name)
            self.assertNotIn(b'\r', data, path.name)
            data.decode('utf-8')
            self.assertIsNone(pattern.search(data), path.name)

    def test_every_script_runs_the_smooth_profile(self):
        for script in self.scripts:
            self.assertEqual(script.setup['profile'], PROFILE, script.name)
            self.assertEqual(rc.startup_config(script)['steer_law']['kind'], 'smooth_r5', script.name)
            self.assertTrue(script.description, script.name)
            kinds = {item['type'] for item in script.expect}
            self.assertIn('no_line', kinds, script.name)             # no V3 fault, refusal or alarm line
            self.assertTrue(kinds & {'change', 'pins_window'}, script.name)

    def test_overrides_touch_only_the_smooth_block(self):
        for script in self.scripts:
            overrides = script.setup['startup_overrides']
            self.assertLessEqual(set(overrides), {'steer_law'}, script.name)
            config = rc.startup_config(script)
            for key, value in overrides.get('steer_law', {}).items():
                self.assertEqual(config['steer_law'][key], value, script.name)


class RealFirmware(unittest.TestCase):
    """Every SMOOTH script against the native lockstep program in firmware mode, both codecs."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = None
        build_dir = os.environ.get('UBTS_STEER_BUILD_DIR')
        if not build_dir:
            cls.tmp = tempfile.mkdtemp(prefix='ubts_smooth_conformance_build_')
            build_dir = cls.tmp
        try:
            cls.executable = rc.resolve_executable(build_dir=build_dir)
            cls.available, cls.reason = rc.probe_firmware_mode(cls.executable)
        except rc.TransportError as exc:
            cls.executable, cls.available, cls.reason = None, False, str(exc)

    @classmethod
    def tearDownClass(cls):
        if cls.tmp:
            shutil.rmtree(cls.tmp, ignore_errors=True)

    def skip_or_fail(self):
        if native_build.firmware_required():
            self.fail('%s is set but the native firmware mode is unavailable: %s' % (
                native_build.ENV_REQUIRE_FIRMWARE, self.reason))
        self.skipTest('native firmware mode unavailable: %s' % self.reason)

    def test_all_smooth_scripts_pass_through_both_codecs(self):
        if not self.available:
            self.skip_or_fail()
        failures = []
        for path in smooth_paths():
            script = rc.load_script(path)
            for name in ('native', 'backend'):
                result = rc.run_script(script, rc.TRANSPORT_FACTORIES[name](script, self.executable))
                if not result['pass']:
                    failures.append('%s: %s' % (name, rc.format_result(result)))
        self.assertEqual(failures, [], '\n'.join(failures))

    def test_the_package_codec_gives_the_same_trace(self):
        if not self.available:
            self.skip_or_fail()
        failures = []
        for path in smooth_paths():
            script = rc.load_script(path)
            schedule = rc.schedule_inputs(script)
            traces = []
            for factory in (rc.native_transport_factory, rc.backend_transport_factory):
                transport = factory(script, self.executable)()
                try:
                    traces.append(rc.execute(script, transport, rc.DEFAULT_STEP_TICKS, schedule))
                finally:
                    transport.close()
            difference = rc.compare_traces(*traces)
            if difference is not None:
                failures.append('%s: %s' % (script.name, difference))
        self.assertEqual(failures, [], '\n'.join(failures))

    def test_a_restart_instead_of_a_takeover_fails(self):
        # The takeover script pins PWM 80 over ticks 403..416; a firmware that stopped and restarted at the new target
        # would show PWM 60 there. Expecting 60 must fail.
        if not self.available:
            self.skip_or_fail()
        path = [p for p in smooth_paths() if p.stem == 'smooth_takeover_same_direction'][0]
        data = json.loads(path.read_bytes().decode('utf-8'))
        for item in data['expect']:
            if item.get('id') == 'no_stop_at_takeover':
                item['steer_pwm'] = 60
        script = rc.Script(data, data['name'])
        result = rc.run_script(script, rc.native_transport_factory(script, self.executable))
        self.assertFalse(result['pass'])
        self.assertEqual([item['id'] for item in result['expectations'] if not item['pass']], ['no_stop_at_takeover'])


if __name__ == '__main__':
    unittest.main()
