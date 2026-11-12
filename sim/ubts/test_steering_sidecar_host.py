"""Steering sidecar recording against the real host code (design 4.5, A23; dashboard/scripts, no ROS).

  * the sidecar's allow lists cover the host names (esp_motor_v3 fault bits, reset names, MCU states,
    steer_bridge_core latch names and schemas); the calibration conversion equals HostProfile.mrad_from_permille over
    -9999..9999 (exported placeholder profile and the host test literal)
  * real /rover/steering/state payloads: SteerBridgeCore.steering_state() of the host test rig (scripts/
    test_steer_support.Rig: fake V3 MCU + the real core and link) through a SteeringSegment, including dropped status
    lines (loss count) and the wrapper's stale repeat while no line arrives
  * what the SIM motor.jsonl contains (design 4.5 first bullet): a real bridge snapshot through the pinned
    dashboard_server.motor_pwm.MotorTimeline (the recorder's code path), one written line checked field by field

The dashboard root is put on sys.path only inside dashboard_imports(); the earlier 'scripts' / 'dashboard_server'
modules and sys.path are restored afterwards (an image may carry another dashboard copy; t1_rig.host_modules does the
same). unittest style. Temporary folders only. Placeholder-based SIM results only.
"""
import contextlib
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

from steering import host_profile_export as hpe
from steering import steering_sidecar as sc
from steering.vehicle_profile import load_profile

DASHBOARD = Path(__file__).resolve().parents[2] / 'dashboard'
PREFIXES = ('scripts', 'dashboard_server')
WALL0 = 1.0e9


@contextlib.contextmanager
def dashboard_imports():
    """Import the host modules of this work tree's dashboard; restore sys.path and sys.modules afterwards."""
    if not (DASHBOARD / 'scripts' / 'steer_bridge_core.py').is_file():
        raise unittest.SkipTest('dashboard host code not found next to sim/ubts')
    saved_path = list(sys.path)
    saved = {n: m for n, m in sys.modules.items() if n.split('.')[0] in PREFIXES}
    for name in saved:
        del sys.modules[name]
    sys.path.insert(0, str(DASHBOARD))
    try:
        import scripts.esp_motor_v3 as v3
        import scripts.steer_bridge_core as bc
        import scripts.test_steer_support as ts
        yield v3, bc, ts
    finally:
        sys.path[:] = saved_path
        for name in [n for n in sys.modules if n.split('.')[0] in PREFIXES]:
            del sys.modules[name]
        sys.modules.update(saved)


def literal_bytes(ts):
    return json.dumps(ts.host_profile_data(), sort_keys=True).encode('utf-8')


def drive(rig, reply, throttle, steer_mrad):
    return rig.submit(rig.next_request(reply, 'drive', deadman=True, throttle=throttle, steer_mrad=steer_mrad))


class HostConstantsTest(unittest.TestCase):
    def test_allow_lists_cover_the_host_names(self):
        with dashboard_imports() as (v3, bc, ts):
            self.assertEqual(sc.MCU_FAULTS, tuple(v3.FAULT_BIT_NAMES))
            self.assertLessEqual(set(bc.LATCH_NAMES), set(sc.HOST_LATCHES))
            self.assertEqual(sc.RESET_REASONS, tuple(v3.RESET_NAMES))
            self.assertEqual(set(sc.MCU_STATES), set(v3.STATE_NAMES.values()))
            self.assertEqual((sc.MESSAGE_SCHEMA, sc.HOST_PROFILE_SCHEMA),
                             (bc.STEER_STATE_SCHEMA, bc.HOST_PROFILE_SCHEMA))

    def test_conversion_equals_the_host_profile(self):
        exported = hpe.to_bytes(hpe.build(load_profile()))
        with dashboard_imports() as (v3, bc, ts):
            for raw in (exported, literal_bytes(ts)):
                host = bc.HostProfile.from_bytes(raw)
                mine = sc.HostCalibration(raw)
                self.assertEqual(mine.sha256, host.sha256)
                self.assertEqual((mine.status_hz, mine.stale_after_s), (host.status_hz, host.status_gap_s))
                bad = [p for p in range(-9999, 10000) if mine.mrad_from_permille(p) != host.mrad_from_permille(p)]
                self.assertEqual(bad, [])


class RealMessagesTest(unittest.TestCase):
    def collect(self):
        """Messages of a host rig run: drive, two dropped status lines, then 1.2 s without status lines (the
        wrapper's 0.5 s stale repeat, emulated as steer_rover_bridge._tick does it)."""
        with dashboard_imports() as (v3, bc, ts):
            raw = literal_bytes(ts)
            rig = ts.Rig()
            rig.core.hp.sha256 = hashlib.sha256(raw).hexdigest()   # the value a file-loaded profile carries
            rig.core.wall_clock = lambda: WALL0 + rig.clock.t      # a wall clock that moves with the rig clock
            rig.ready()
            reply = rig.arm()
            rig.link.take_status_history()
            messages, published = [], [rig.clock.t]
            boot = rig.mcu.boot_id
            first_n = rig.mcu.n
            rig.mcu.drop_status = {first_n + 10, first_n + 11}

            def step():
                rig.tick()
                lines = rig.link.take_status_history()
                for line, received in lines:
                    messages.append((rig.core.steering_state(line, received), rig.clock.t))
                if lines:
                    published[0] = rig.clock.t
                elif rig.clock.t - published[0] >= 0.5 - 1e-9:
                    published[0] = rig.clock.t
                    messages.append((rig.core.steering_state(), rig.clock.t))
            for i in range(150):                  # 1.5 s, the drive request renewed every 0.2 s
                if i % 20 == 0:
                    reply = drive(rig, reply, 40, -120)
                step()
            rig.mcu.send_status = False
            for _ in range(120):
                step()
            return raw, messages, boot

    def test_bridge_messages_through_a_segment(self):
        raw, messages, boot = self.collect()
        cal = sc.HostCalibration(raw)
        self.assertGreater(len(messages), 25)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / '20261109T010203Z_both'
            root.mkdir()
            start = WALL0 - 10.0
            started_at = datetime.fromtimestamp(start, timezone.utc).isoformat()
            seg = sc.SteeringSegment(root, root / 'steering', dict(id=root.name, started_at=started_at),
                                     calibration=cal, clock=lambda: start + 1.0, steady=lambda: 50.0)
            seg.open()
            for m, published in messages:     # archive receive 2 ms after the bridge published it
                self.assertIsNotNone(seg.observe(json.dumps(m), WALL0 + published + 0.002, 59.0 + published))
            seg.close_frames('test')
            frames = [json.loads(line) for line in (seg.dir / sc.FRAMES_FILE).read_text(encoding='utf-8').splitlines()]
        stats = seg.stats()
        self.assertEqual(stats['invalid_values'], {})
        self.assertEqual(stats['unknown_fault_names'], {})
        self.assertEqual(stats['rejected_messages'], 0)
        self.assertNotIn('mismatch', stats['angle_checks'])
        self.assertGreater(stats['angle_checks']['ok'], 20)
        self.assertEqual(stats['lost_status_lines'], 2)
        self.assertEqual(stats['boot_ids'], [boot])
        self.assertGreaterEqual(stats['repeated_status_lines'], 2)                 # the stale repeats
        self.assertTrue(all(list(f) == list(sc.FRAME_FIELDS) for f in frames))
        driving = [f for f in frames if f['drive_l'] and not f['stale']]    # the stale repeats: host stopped
        self.assertTrue(driving)
        self.assertTrue(all(f['drive_l'] == f['drive_r'] == 102 for f in driving if f['state'] == 'ARMED'))
        self.assertEqual({f['target_mrad'] for f in driving}, {-120})
        self.assertEqual({f['source_kind'] for f in frames}, {'simulation'})
        self.assertEqual({f['cal_id'] for f in frames}, {7})
        settled = [f for f in frames if f['pos_permille'] == -274]
        self.assertTrue(settled and all(f['angle_mrad'] == -120 for f in settled))
        repeats = frames[-2:]                                                     # after the lines stopped
        self.assertTrue(all(f['stale'] for f in repeats))
        self.assertTrue(all(f['mcu_seq'] == frames[-3]['mcu_seq'] for f in repeats))
        self.assertTrue(all(not f['stale'] for f in frames[:-2]))
        self.assertEqual(stats['stale_reasons'], {'age_over': 2, 'bridge_flag': 2})
        self.assertLess(stats['receive_age_s']['max'], 0.02)                     # the fresh ages only

    def test_message_carries_every_design_field(self):
        raw, messages, boot = self.collect()
        line_messages = [m for m, _ in messages if m.get('mcu_seq') is not None]
        self.assertLessEqual(set(sc.FRAME_FIELDS[3:]), set(line_messages[0]))


class RecorderRoleTest(unittest.TestCase):
    """The recorder role of the stack supervisor runs the sidecar next to the pinned recorder (ST-06)."""

    def test_sidecar_is_a_child_of_the_recorder_role(self):
        import os
        from steering import supervisor_steer as sup
        args = sup.build_parser().parse_args(['--role', 'recorder', '--runtime-dir', '/runtime'])
        if not os.environ.get('UBTS_STEER_SIDECAR'):
            self.assertEqual(sup.sidecar_of(args), sup.DEFAULT_SIDECAR)
        children = sup.recorder_children(args)
        self.assertEqual([c.name for c in children][0], 'data_recorder')
        if not os.environ.get('UBTS_STEER_SIDECAR'):
            self.assertEqual([c.name for c in children], ['data_recorder', 'steering_sidecar'])
        for off in ('none', 'off', '0', 'NONE'):
            args = sup.build_parser().parse_args(['--role', 'recorder', '--sidecar', off])
            self.assertIsNone(sup.sidecar_of(args))
            self.assertEqual([c.name for c in sup.recorder_children(args)], ['data_recorder'])

    def test_command_parses_with_the_sidecars_parser(self):
        from steering import supervisor_steer as sup
        command = sup.sidecar_command(sup.DEFAULT_SIDECAR, Path('/runtime'))
        self.assertEqual(command[1:4], ['-B', '-m', 'steering.steering_sidecar'])
        args = sc.build_parser().parse_args(command[4:])
        self.assertEqual(Path(args.archive_dir).as_posix(), '/runtime/recordings')
        self.assertEqual(args.recorder_url, sc.DEFAULT_RECORDER_URL)
        self.assertEqual(Path(args.host_profile).as_posix(), sc.DEFAULT_HOST_PROFILE)
        self.assertEqual(Path(args.status_file).as_posix(), sc.DEFAULT_STATUS_FILE)
        self.assertFalse(args.no_bag)
        # the status file is one of the recorder role's files (moved away at the role's start, audit G06)
        self.assertIn(sup.SIDECAR_STATUS, sup.ROLE_STATUS_FILES['recorder'])
        self.assertEqual('/runtime/' + sup.SIDECAR_STATUS, sc.DEFAULT_STATUS_FILE)

    def test_readiness_states_are_the_sidecars_live_states(self):
        from steering import supervisor_steer as sup
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / sup.SIDECAR_STATUS
            probe = sup.json_file_ready(path, lambda d: d.get('state') in sup.SIDECAR_READY_STATES)
            import time
            for state, ready in (('idle', True), ('recording', True), ('starting', True), ('failed', False),
                                 ('stopped', False)):
                path.write_text(json.dumps(dict(state=state, updated_at=time.time())), encoding='utf-8')
                self.assertEqual(bool(probe()), ready, state)

    def test_compose_passes_the_switch_to_the_recorder(self):
        text = (Path(__file__).resolve().parents[2] / 'compose.ubts-steer.yaml').read_text(encoding='utf-8')
        self.assertIn('UBTS_STEER_SIDECAR: ${UBTS_STEER_SIDECAR:-}', text)


class MotorJsonlTest(unittest.TestCase):
    """What the pinned recorder writes into motor.jsonl in the steering SIM (design 4.5 first bullet)."""

    def test_sim_motor_jsonl_line(self):
        with dashboard_imports() as (v3, bc, ts):
            from dashboard_server import motor_pwm
            import scripts.steer_rover_bridge as wrapper
            rig = ts.Rig()
            rig.ready()
            reply = rig.arm()
            reply = drive(rig, reply, 40, 100)
            rig.run(0.3)
            snapshot = json.loads(json.dumps(rig.core.snapshot))
            timeline = motor_pwm.MotorTimeline('/rover/status')
            with tempfile.TemporaryDirectory() as tmp:
                timeline.start(Path(tmp), started_steady=100.0, started_at=snapshot['updated_at'] - 1.0)
                timeline.observe(json.dumps(snapshot), received_at=snapshot['updated_at'] + 0.05,
                                 received_steady=101.05)
                timeline.close()
                lines = (Path(tmp) / 'motor.jsonl').read_text(encoding='utf-8').splitlines()
            descriptor = timeline.descriptor()
            status_period = wrapper.STATUS_PERIOD_S
        self.assertEqual(len(lines), 1)
        frame = json.loads(lines[0])
        self.assertEqual(sorted(frame), sorted(['t_s', 'elapsed_monotonic_s', 'clock_offset_s', 'source_kind',
                                                'received_at', 'source_updated_at', 'source_age_s', 'source_stale',
                                                'status', 'requested', 'pins']))
        self.assertEqual(frame['source_kind'], 'simulation')
        self.assertEqual(frame['status']['firmware'], None)              # UBTS_STEER_V3 is not in the allow list
        self.assertEqual(snapshot['firmware'], 'UBTS_STEER_V3')
        self.assertEqual((frame['status']['state'], frame['status']['simulated']), ('armed_manual', True))
        self.assertEqual(frame['requested'], {'left': 40, 'right': 40})   # throttle, not the steering request
        self.assertNotIn('steer', json.dumps(frame))                     # no steering field survives the allow list
        outputs = snapshot['outputs']
        for pin, row in frame['pins'].items():
            self.assertIsNone(row['reported_pwm'], pin)                   # SIM: values go to simulated_output_pwm
            self.assertEqual(row['simulated_output_pwm'], outputs.get(pin), pin)
        self.assertEqual(frame['pins']['D8']['requested_pwm_derived'], 102)
        self.assertEqual(frame['pins']['D3']['requested_pwm_derived'], 102)
        self.assertEqual(frame['pins']['D2']['simulated_output_pwm'], 255)          # steering EN on
        roles = {row['pin']: row['role'] for row in descriptor['pins']}
        self.assertEqual((roles['D2'], roles['D6'], roles['D7']), ('driver_enable',) * 3)   # V2 labels (mislabel)
        self.assertEqual(descriptor['sample_hz'], 4)                      # nominal 'simulation' rate of motor_pwm
        self.assertEqual(status_period, 0.5)                              # the steer bridge publishes 2 Hz


if __name__ == '__main__':
    unittest.main()
