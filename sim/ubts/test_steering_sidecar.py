"""Steering sidecar recording (design 4.5, 8.3 P6b, A23): pure parts, no ROS, no HTTP, no host import.

  * the frame field list is exactly the design 4.5 list; the allow list drops unknown keys and bad values
    (motor_pwm style); the stale rule; the angle from the recording's calibration table
  * rate and loss counting on a synthetic 20 Hz status stream (gaps, 16-bit wrap, repeats, boot change)
  * monotonic time base (a wall clock step back is skipped and counted), fsync policy (bounded fsync rate)
  * descriptor: contents, SHA stability (key order, same input = same bytes), check_segment detects tampering
  * the recording-ID handshake and start/stop races with a fake recorder status and a fake bag: normal stop, stop
    before the first sample, a recording seen only while finalising, buffer replay, recorder restart (quick and
    unreachable), recording switch, missing folder / metadata / archive dir, sidecar crash and restart (recovered
    descriptor, frames file unchanged, segment steering-2), shutdown during a recording, bag start failure, no host
    profile

unittest style (pytest on 3.12 / 3.13, python3 -m unittest on 3.10). Temporary folders only (tempfile).
Placeholder-based SIM results only.
"""
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest

from steering import steering_bag_record as bag_mod
from steering import steering_sidecar as sc

DESIGN_FIELDS = ['t_s', 'elapsed_monotonic_s', 'clock_offset_s', 'source_kind', 'received_at', 'mcu_ms', 'mcu_seq',
                 'boot_id', 'reset_reason', 'cal_id', 'state', 'stale', 'raw_adc', 'pos_permille', 'angle_mrad',
                 'target_mrad', 'cmd_permille', 'steer_pwm', 'drive_l', 'drive_r', 'drive_en', 'steer_en',
                 'applied_q', 'ttl_remaining_ms', 'stop_mode', 'faults']
START_WALL = 1_800_000_000.0
BOOT = '89025CC1'


def host_profile_bytes(status_hz=20, gap_ms=150.0, cal_id=1):
    """A minimal host profile (the parts HostCalibration reads, plus a source profile)."""
    data = {
        'schema': 'ubts.steer_host_profile.v1', 'vehicle_profile': 'front_steer', 'hw_profile': 'front_steer_v0',
        'source_profile': {'profile_id': 'front_steer_v0_1_placeholder', 'profile_sha256': 'a' * 64,
                           'provenance_grade': 'placeholder', 'result_kind': 'placeholder SIM result',
                           'field_validated': False, 'overrides': None},
        'steer_limit': {'left_mrad': 409, 'right_mrad': 409, 'left_rad': 0.4094, 'right_rad': 0.4094},
        'calibration': {'cal_id': cal_id, 'mark_left_mrad': 438.030759092455, 'mark_right_mrad': 438.0307590924551,
                        'permille_per_deg': 39.84, 'expected_soft_permille': {'left': 935, 'right': -935},
                        'rule': 'test literal'},
        'link': {'status_hz': status_hz}, 'host_faults': {'status_gap_ms': gap_ms},
    }
    return (json.dumps(data, sort_keys=True, indent=1) + '\n').encode('utf-8')


class FakeClock:
    """Wall and monotonic clocks moved together (wall_step moves the wall clock alone)."""

    def __init__(self, wall=START_WALL - 5.0, steady=1000.0):
        self.wall, self.mono = wall, steady

    def clock(self):
        return self.wall

    def steady(self):
        return self.mono

    def advance(self, dt):
        self.wall += dt
        self.mono += dt

    def sleep(self, dt):
        self.advance(dt)


def message(seq, ms, *, boot=BOOT, stale=False, received_at=None, pos=-274, angle=-120, sha=None, extra=None,
            line=True):
    m = dict(schema='ubts.steer_state.v1', source_kind='simulation', received_at=received_at,
             received_monotonic=12.5, stale=stale, boot_id=boot, reset_reason='power_on', cal_id=1, target_mrad=-120,
             stop_mode=None, host_faults=[], host_profile_sha256=sha)
    if line:
        m.update(mcu_ms=ms, mcu_seq=seq, state='ARMED', raw_adc=611, pos_permille=pos, angle_mrad=angle,
                 target_permille=-274, cmd_permille=-270, steer_pwm=-64, drive_l=102, drive_r=102, drive_en=True,
                 steer_en=True, applied_q=1842, ttl_remaining_ms=260, fl='0000', faults=[])
    else:
        m.update(dict.fromkeys(('mcu_ms', 'mcu_seq', 'state', 'raw_adc', 'pos_permille', 'angle_mrad',
                                'target_permille', 'cmd_permille', 'steer_pwm', 'drive_l', 'drive_r', 'drive_en',
                                'steer_en', 'applied_q', 'ttl_remaining_ms', 'fl')), faults=[])
    m.update(extra or {})
    return m


def iso(t):
    return datetime.fromtimestamp(t, timezone.utc).isoformat()


class FakeRecorder:
    """The pinned recorder's folder and status behaviour (start creates <archive>/<id>/metadata.json first)."""

    def __init__(self, archive):
        self.archive = Path(archive)
        self.archive.mkdir(parents=True, exist_ok=True)

    def start(self, rid, started):
        root = self.archive / rid
        root.mkdir(parents=True, exist_ok=False)
        meta = dict(id=rid, kind='both', state='recording', started_at=iso(started), topics=['/scan'])
        (root / 'metadata.json').write_bytes(json.dumps(meta).encode())
        return root

    def archive_rec(self, rid, duration):
        root = self.archive / rid
        meta = json.loads((root / 'metadata.json').read_bytes())
        meta.update(state='archived', finished_at=iso(sc.parse_started_at(meta['started_at']) + duration),
                    duration_s=duration)
        (root / 'metadata.json').write_bytes(json.dumps(meta).encode())

    def status(self, state, rid=None):
        out = dict(ok=True, state=state, recording_id=rid, archive_dir=str(self.archive), started_at=None)
        if rid is not None:
            meta = json.loads((self.archive / rid / 'metadata.json').read_bytes())
            out['started_at'] = meta['started_at']
        return out


class FakeBag:
    """Stands in for steering_bag_record.BagRecord: makes the folder, counts messages it is told about, writes a
    rosbag2-like metadata.yaml when it 'exits' (after exit_polls polls following the stop request)."""
    created = []

    def __init__(self, output_dir, log_path, exit_polls=1, count=5, fail=False):
        self.output_dir, self.log_path = Path(output_dir), Path(log_path)
        self.exit_polls, self.count, self.fail = exit_polls, count, fail
        self.pid = 4242
        self.stop_requested = False
        self.polls = 0
        self.exit_code = None
        FakeBag.created.append(self)

    def command(self):
        return ['ros2', 'bag', 'record', '-o', str(self.output_dir)]

    def start(self):
        if self.fail:
            raise bag_mod.BagRecordError('fake start failure')
        self.output_dir.mkdir()

    def request_stop(self):
        self.stop_requested = True

    def poll(self):
        if not self.stop_requested:
            return False
        self.polls += 1
        if self.polls >= self.exit_polls and self.exit_code is None:
            text = ('rosbag2_bagfile_information:\n  version: 9\n  storage_identifier: mcap\n'
                    '  duration:\n    nanoseconds: 1000000000\n  starting_time:\n    nanoseconds_since_epoch: 5\n'
                    '  message_count: %d\n  topics_with_message_count:\n    - topic_metadata:\n'
                    '        name: /rover/steering/state\n        type: std_msgs/msg/String\n'
                    '      message_count: %d\n  relative_file_paths:\n    - bag_0.mcap\n') % (self.count, self.count)
            (self.output_dir / 'metadata.yaml').write_text(text, encoding='utf-8')
            self.exit_code = 0
        return self.exit_code is not None

    def result(self):
        return dict(schema=bag_mod.SCHEMA, output_dir=self.output_dir.name, exit_code=self.exit_code,
                    time_base='ros_simulation_time', signals=[['SIGINT', 0.0]] if self.stop_requested else [],
                    metadata=bag_mod.read_metadata(self.output_dir))


class Harness:
    """Temporary archive, fake recorder, fake clock, host profile file and a SidecarController."""

    def __init__(self, test, *, host_profile=True, bag=True, bag_kwargs=None, **kwargs):
        self.tmp = tempfile.TemporaryDirectory()
        test.addCleanup(self.tmp.cleanup)
        base = Path(self.tmp.name)
        self.archive = base / 'recordings'
        self.recorder = FakeRecorder(self.archive)
        self.clock = FakeClock()
        self.profile_path = base / 'host_profile.json'
        self.profile_raw = host_profile_bytes()
        self.profile_path.write_bytes(self.profile_raw)
        self.sha = hashlib.sha256(self.profile_raw).hexdigest()
        self.fsyncs = []
        self.bags = []
        bag_kwargs = bag_kwargs or {}

        def factory(output_dir, log_path):
            item = FakeBag(output_dir, log_path, **bag_kwargs)
            self.bags.append(item)
            return item
        self.controllers = []
        test.addCleanup(self.close_open_files)          # runs before the folder clean-up (LIFO)
        self.controller = self.make(host_profile=host_profile, bag=factory if bag else None, **kwargs)

    def close_open_files(self):
        for controller in self.controllers:
            if controller.segment is not None:
                controller.segment.close_frames('test_cleanup')

    def make(self, host_profile=True, bag=None, **kwargs):
        profile = str(self.profile_path) if host_profile else None
        controller = sc.SidecarController(self.archive, host_profile_path=profile,
                                          recorder_url='http://recorder.invalid', bag_factory=bag,
                                          clock=self.clock.clock, steady=self.clock.steady, sleep=self.clock.sleep,
                                          fsync=self.fsyncs.append, **kwargs)
        self.controllers.append(controller)
        return controller

    def send(self, controller, seq, ms, **kw):
        kw.setdefault('sha', self.sha)
        kw.setdefault('received_at', self.clock.wall - 0.01)
        controller.on_message(json.dumps(message(seq, ms, **kw)))

    def stream(self, controller, seconds, seq0=0, ms0=0, hz=20, skip=(), **kw):
        n = int(round(seconds * hz))
        for i in range(n):
            self.clock.advance(1.0 / hz)
            if i in skip:
                continue
            self.send(controller, (seq0 + i) % 65536, ms0 + int(round(i * 1000 / hz)), **kw)
        return seq0 + n, ms0 + int(round(n * 1000 / hz))


def segment_for(harness, rid, name='steering'):
    return harness.archive / rid / name


def descriptor_of(path):
    return json.loads((Path(path) / sc.DESCRIPTOR_FILE).read_bytes().decode('utf-8'))


class SanitizeTest(unittest.TestCase):
    def setUp(self):
        self.cal = sc.HostCalibration(host_profile_bytes())

    def test_frame_fields_are_exactly_the_design_list(self):
        self.assertEqual(list(sc.FRAME_FIELDS), DESIGN_FIELDS)
        self.assertEqual(sc.SCHEMA, 'orchard.steering.timeline.v1')
        self.assertEqual(sc.MEANING, ('가변저항은 링크 각의 측정값이며 타이어 접지각이 아니다', 'PWM은 엔코더가 아니다'))

    def test_allow_list_drops_unknown_keys_and_free_text(self):
        m = message(5, 1000, received_at=99.99, sha=self.cal.sha256,
                    extra=dict(lease='secret-lease', ticket='t', note='free text', host_faults=['x'], fl='0000'))
        sample, info = sc.sanitize_state(m, 100.0, 0.15, self.cal)
        self.assertEqual(sorted(sample), sorted(DESIGN_FIELDS[3:]))
        text = json.dumps(sample)
        for word in ('secret-lease', 'free text', 'received_monotonic', 'target_permille', 'host_profile_sha256'):
            self.assertNotIn(word, text)
        self.assertEqual(info['invalid'], [])

    def test_bad_values_become_null_and_are_counted(self):
        bad = dict(mcu_seq=True, mcu_ms=-1, raw_adc=1024, pos_permille=10000, steer_pwm=256.0, drive_l='102',
                   drive_en=1, steer_en='yes', state='ARMED_MANUAL', stop_mode='free', reset_reason='other',
                   boot_id='89025cc1', cal_id=0, target_mrad=2000, ttl_remaining_ms=1001, applied_q=70000,
                   source_kind='real', faults='POT_OPEN', cmd_permille=None)
        sample, info = sc.sanitize_state(message(5, 1000, received_at=99.99, extra=bad), 100.0, 0.15, self.cal)
        for key in bad:
            self.assertIsNone(sample[key], key)
        self.assertEqual(sorted(info['invalid']), sorted(k for k in bad if k != 'cmd_permille'))
        self.assertTrue(sample['stale'])          # no valid mcu_seq -> no status line

    def test_faults_keep_known_names_only(self):
        m = message(5, 1000, received_at=99.99, extra=dict(faults=['POT_OPEN', 'steering_status_stale', 'NEW_THING',
                                                                    'mcu_ttl_expired', 'bad name!', 7]))
        sample, info = sc.sanitize_state(m, 100.0, 0.15, self.cal)
        self.assertEqual(sample['faults'], ['POT_OPEN', 'steering_status_stale', 'mcu_ttl_expired'])
        self.assertEqual(info['unknown_faults'], ['NEW_THING', '<invalid>', '<invalid>'])

    def test_other_schema_or_type_is_rejected(self):
        for payload in (None, [], 'x', dict(message(1, 1), schema='ubts.other.v1'),
                        {k: v for k, v in message(1, 1).items() if k != 'schema'}):
            self.assertEqual(sc.sanitize_state(payload, 1.0, 0.15, self.cal), (None, None))

    def test_stale_rule(self):
        def stale(received_at, now=100.0, **kw):
            sample, info = sc.sanitize_state(message(5, 1000, received_at=received_at, sha=self.cal.sha256, **kw),
                                             now, 0.15, self.cal)
            return sample['stale'], info['stale_reasons']
        self.assertEqual(stale(99.99), (False, []))
        self.assertEqual(stale(99.80), (True, ['age_over']))
        self.assertEqual(stale(100.05), (True, ['age_unknown']))          # bridge time after ours: unknown
        self.assertEqual(stale(None), (True, ['age_unknown']))
        self.assertEqual(stale(99.99, stale=True), (True, ['bridge_flag']))
        self.assertEqual(stale(None, stale=True, line=False), (True, ['bridge_flag', 'no_status_line', 'age_unknown']))
        self.assertEqual(stale(99.99, stale='no'), (True, ['bridge_flag']))  # a non-bool flag counts as stale

    def test_angle_from_the_recording_table(self):
        def angle(**kw):
            sample, info = sc.sanitize_state(message(5, 1000, received_at=99.99, **kw), 100.0, 0.15, self.cal)
            return sample['angle_mrad'], info['angle_check']
        self.assertEqual(angle(sha=self.cal.sha256, pos=-274, angle=-120), (-120, 'ok'))
        self.assertEqual(angle(sha=self.cal.sha256, pos=935, angle=410), (410, 'ok'))
        self.assertEqual(angle(sha=self.cal.sha256, pos=-274, angle=-119), (-120, 'mismatch'))
        self.assertEqual(angle(sha='b' * 64, pos=-274, angle=-120), (None, 'table_mismatch'))
        self.assertEqual(angle(sha=self.cal.sha256, pos=-274, angle=None), (None, 'bridge_marks_invalid'))
        sample, info = sc.sanitize_state(message(5, 1000, received_at=99.99, sha=self.cal.sha256), 100.0, 0.15, None)
        self.assertEqual((sample['angle_mrad'], info['angle_check']), (None, 'no_table'))

    def test_host_calibration_reads_the_profile(self):
        self.assertEqual((self.cal.status_hz, self.cal.stale_after_s, self.cal.cal_id), (20, 0.15, 1))
        self.assertEqual(self.cal.sha256, hashlib.sha256(host_profile_bytes()).hexdigest())
        self.assertEqual(self.cal.mrad_from_permille(0), 0)
        for broken in (b'{', json.dumps(dict(schema='x')).encode(),
                       host_profile_bytes().replace(b'438.030759092455', b'-1')):
            with self.assertRaises(sc.SidecarError):
                sc.HostCalibration(broken)

    def test_parse_started_at(self):
        self.assertEqual(sc.parse_started_at(iso(START_WALL)), START_WALL)
        for text in (None, '', 'yesterday', '2026-11-09T01:02:03', 5):
            self.assertIsNone(sc.parse_started_at(text))


class SegmentTest(unittest.TestCase):
    def open_segment(self, h, rid='20261109T010203Z_both', started=START_WALL, **kw):
        root = h.recorder.start(rid, started)
        cal = sc.HostCalibration.load(h.profile_path)
        seg = sc.SteeringSegment(root, root / 'steering', dict(id=rid, started_at=iso(started)), calibration=cal,
                                 clock=h.clock.clock, steady=h.clock.steady, fsync=h.fsyncs.append, **kw)
        h.clock.wall = started + 0.25
        seg.open()
        self.addCleanup(seg.close_frames, 'test_cleanup')     # no-op when the test closed it
        return seg

    def feed(self, h, seg, seq, ms, **kw):
        kw.setdefault('received_at', h.clock.wall - 0.01)
        kw.setdefault('sha', h.sha)
        return seg.observe(json.dumps(message(seq, ms, **kw)), h.clock.wall, h.clock.mono)

    def test_rate_and_loss_counting(self):
        h = Harness(self, bag=False)
        seg = self.open_segment(h)
        seq, ms = 65500, 0
        lost = 0
        for i in range(200):                    # 10 s at 20 Hz: 4 lines lost, a 16-bit wrap in the middle
            h.clock.advance(0.05)
            if i in (10, 11, 50, 120):
                lost += 1
            else:
                self.feed(h, seg, (seq + i) % 65536, ms + 50 * i)
            if i in (30, 31):                   # the bridge repeats the last state (stale) twice
                self.feed(h, seg, (seq + i) % 65536, ms + 50 * i, stale=True)
        for i in range(20):                     # MCU reset: new boot id, n starts again
            h.clock.advance(0.05)
            self.feed(h, seg, i, 5000 + 50 * i, boot='658EEC67')
        stats = seg.stats()
        self.assertEqual(stats['unique_status_lines'], 196 + 20)
        self.assertEqual(stats['lost_status_lines'], 4)
        self.assertEqual(stats['repeated_status_lines'], 2)
        self.assertEqual(stats['boot_changes'], 1)
        self.assertEqual(stats['boot_ids'], [BOOT, '658EEC67'])
        self.assertEqual(stats['seq_regressions'], 0)
        self.assertAlmostEqual(stats['loss_rate'], 4 / 220, places=6)
        self.assertAlmostEqual(stats['mcu_rate_hz'], 20.0, places=3)      # MCU emission rate (lost lines included)
        # archive rate: unique lines received per second between the first and the last frame (losses lower it)
        self.assertAlmostEqual(stats['archive_rate_hz'], (216 - 1) / (11.25 - 0.3), delta=0.01)
        self.assertEqual(stats['stale_frames'], 2)
        self.assertEqual(stats['stale_reasons'], {'bridge_flag': 2})
        self.assertEqual(seg.frame_count, 196 + 2 + 20)

    def test_sequence_regression_without_boot_change(self):
        h = Harness(self, bag=False)
        seg = self.open_segment(h)
        for seq in (10, 11, 12, 3, 4):
            h.clock.advance(0.05)
            self.feed(h, seg, seq, 1000 + seq * 50)
        stats = seg.stats()
        self.assertEqual((stats['seq_regressions'], stats['lost_status_lines'], stats['unique_status_lines']), (1, 0, 5))

    def test_time_base_is_monotonic(self):
        h = Harness(self, bag=False)
        seg = self.open_segment(h)
        h.clock.advance(0.05)
        first = self.feed(h, seg, 1, 50)
        self.assertAlmostEqual(first['t_s'], 0.3, places=6)
        self.assertAlmostEqual(first['elapsed_monotonic_s'], 0.3, places=6)
        self.assertEqual(first['clock_offset_s'], 0.0)
        h.clock.advance(0.05)
        h.clock.wall -= 1.0                     # wall clock steps back 1 s
        self.assertIsNone(self.feed(h, seg, 2, 100, received_at=h.clock.wall - 0.01))
        for i in range(25):                     # skipped until the wall time is past the last frame again
            h.clock.advance(0.05)
            self.feed(h, seg, 3 + i, 150 + 50 * i, received_at=h.clock.wall - 0.01)
        seg.close_frames('test')
        frames = [json.loads(line) for line in (seg.dir / sc.FRAMES_FILE).read_text(encoding='utf-8').splitlines()]
        t = [f['t_s'] for f in frames]
        self.assertTrue(all(b >= a for a, b in zip(t, t[1:])))
        mono = [f['elapsed_monotonic_s'] for f in frames]
        self.assertTrue(all(b > a for a, b in zip(mono, mono[1:])))
        self.assertGreaterEqual(seg.counts['dropped_time_frames'], 19)
        self.assertEqual(seg.error, 'steering_recording_clock_regressed')
        self.assertAlmostEqual(frames[-1]['clock_offset_s'], -1.0, places=5)

    def test_fsync_rate_is_bounded(self):
        h = Harness(self, bag=False)
        seg = self.open_segment(h)
        for i in range(100):                    # 5 s at 20 Hz
            h.clock.advance(0.05)
            self.feed(h, seg, i, 50 * i)
        writes = len(h.fsyncs)
        self.assertGreaterEqual(writes, 4)
        self.assertLessEqual(writes, 6)
        seg.close_frames('test')
        self.assertEqual(len(h.fsyncs), writes + 1)     # one more at close
        self.assertEqual(seg.fsync_calls, writes + 1)

    def test_preview_is_bounded_and_spans_the_recording(self):
        preview = sc.Preview(max_frames=10, interval_s=0.5)
        for i in range(2000):
            preview.add(dict(t_s=i * 0.05))
        self.assertLessEqual(len(preview.frames), 10)
        self.assertEqual(preview.frames[0]['t_s'], 0.0)
        self.assertGreater(preview.frames[-1]['t_s'], 80.0)

    def finished_segment(self, h, rid='20261109T010203Z_both'):
        seg = self.open_segment(h, rid=rid)
        for i in range(40):
            h.clock.advance(0.05)
            self.feed(h, seg, i, 50 * i)
        seg.close_frames('recorder_stop')
        return seg

    def test_descriptor_contents(self):
        h = Harness(self, bag=False)
        seg = self.finished_segment(h)
        d = seg.finalize(dict(metadata=dict(finalized=True)), dict(state='archived', duration_s=2.5, complete=True),
                         dict(source_commit='abc'))
        self.assertEqual(d['schema'], sc.SCHEMA)
        self.assertEqual((d['topic'], d['sample_hz'], d['stale_after_s']), ('/rover/steering/state', 20, 0.15))
        self.assertEqual(d['time_base'], 'host_receive_time_relative_to_recording_start')
        self.assertEqual(d['calibration']['table'], json.loads(h.profile_raw)['calibration'])
        self.assertEqual(d['calibration']['cal_id'], 1)
        self.assertEqual(d['calibration']['host_profile_sha256'], h.sha)
        self.assertTrue(d['calibration']['matches_messages'])
        self.assertEqual(d['profile']['profile_id'], 'front_steer_v0_1_placeholder')
        self.assertEqual(d['profile']['profile_sha256'], 'a' * 64)
        self.assertEqual(d['meaning'], list(sc.MEANING))
        self.assertEqual((d['frames_file'], d['frame_count'], d['fields']), ('steering.jsonl', 40, DESIGN_FIELDS))
        self.assertEqual(d['frames_sha256'], sc.file_sha256(seg.dir / sc.FRAMES_FILE))
        self.assertTrue(0 < len(d['preview']) <= d['preview_max_frames'])
        self.assertEqual(d['result_kind'], '자리값 기반 SIM 결과')
        self.assertEqual(d['handshake']['frames_after_recorder_duration'], 0)
        later = seg.build_descriptor(None, dict(state='archived', duration_s=2.02, complete=True))
        self.assertEqual(later['handshake']['frames_after_recorder_duration'], 5)    # t_s 2.05 .. 2.25
        self.assertEqual((seg.dir / sc.HOST_PROFILE_COPY).read_bytes(), h.profile_raw)
        raw = (seg.dir / sc.DESCRIPTOR_FILE).read_bytes()
        self.assertEqual(raw, sc.descriptor_bytes(d))
        self.assertNotIn(b'\r\n', raw)
        line = (seg.dir / sc.DESCRIPTOR_SHA_FILE).read_text(encoding='ascii').split()
        self.assertEqual(line, [hashlib.sha256(raw).hexdigest(), sc.DESCRIPTOR_FILE])

    def test_descriptor_sha_is_stable(self):
        digests = []
        for _ in range(2):                      # same inputs and clocks -> same bytes and SHA
            h = Harness(self, bag=False)
            seg = self.finished_segment(h)
            d = seg.finalize(None, dict(state='archived', duration_s=2.0, complete=True), dict(source_commit='abc'))
            digests.append((d['content_sha256'], hashlib.sha256((seg.dir / sc.DESCRIPTOR_FILE).read_bytes())
                            .hexdigest(), d['frames_sha256']))
        self.assertEqual(digests[0][0], digests[1][0])
        self.assertEqual(digests[0][1], digests[1][1])
        shuffled = dict(reversed(list(d.items())))
        self.assertEqual(sc.content_sha256(shuffled), d['content_sha256'])
        changed = copy.deepcopy(d)
        changed['stats']['frames'] += 1
        self.assertNotEqual(sc.content_sha256(changed), d['content_sha256'])

    def test_check_segment_detects_tampering(self):
        h = Harness(self, bag=False)
        seg = self.finished_segment(h)
        bag_dir = seg.dir / sc.BAG_DIR
        fake = FakeBag(bag_dir, seg.dir / sc.BAG_LOG, count=33)
        fake.start()
        fake.request_stop()
        fake.poll()
        seg.finalize(fake.result(), dict(state='archived', duration_s=2.0, complete=True))
        good = sc.check_segment(seg.dir)
        self.assertTrue(good['passed'], good['problems'])
        self.assertEqual((good['frame_count'], good['bag']['message_count']), (40, 33))
        frames = seg.dir / sc.FRAMES_FILE
        original = frames.read_bytes()
        frames.write_bytes(original.replace(b'"drive_l":102', b'"drive_l":103', 1))
        self.assertIn('frames_sha256', sc.check_segment(seg.dir)['problems'])
        frames.write_bytes(original)
        desc = seg.dir / sc.DESCRIPTOR_FILE
        raw = desc.read_bytes()
        desc.write_bytes(raw.replace(b'"frame_count":40', b'"frame_count":41'))
        problems = sc.check_segment(seg.dir)['problems']
        self.assertIn('descriptor_content_sha256', problems)
        self.assertIn('descriptor_file_sha256', problems)
        desc.write_bytes(raw)
        frames.write_bytes(original + b'{"t_s":1}\n')
        self.assertIn('frames_schema', sc.check_segment(seg.dir)['problems'])


class ControllerTest(unittest.TestCase):
    RID = '20261109T010203Z_both'

    def start(self, h, rid=None, started=None):
        rid = rid or self.RID
        started = h.clock.wall if started is None else started
        h.recorder.start(rid, started)
        return rid

    def poll(self, h, state, rid=None, controller=None):
        (controller or h.controller).on_status(h.recorder.status(state, rid))

    def finish(self, h, rid=None, duration=None, controller=None):
        rid = rid or self.RID
        controller = controller or h.controller
        self.poll(h, 'finalizing', rid, controller)
        h.recorder.archive_rec(rid, duration if duration is not None else 5.0)
        self.poll(h, 'idle', None, controller)
        for _ in range(5):
            controller.tick()

    def test_normal_recording(self):
        h = Harness(self)
        self.poll(h, 'idle')
        rid = self.start(h)
        h.clock.advance(0.1)
        h.stream(h.controller, 0.1)             # before the start was seen: kept in the buffer
        self.poll(h, 'recording', rid)
        self.assertIsNotNone(h.controller.segment)
        self.assertEqual(h.controller.status()['state'], 'recording')
        h.stream(h.controller, 5.0, seq0=2, ms0=100)
        self.finish(h, duration=5.3)
        d = descriptor_of(segment_for(h, rid))
        self.assertEqual(d['handshake']['end_reason'], 'recorder_stop')
        self.assertEqual(d['handshake']['recorder_final']['state'], 'archived')
        self.assertEqual(d['frame_count'], 102)
        self.assertEqual(d['stats']['replayed_from_buffer'], 2)
        self.assertEqual(d['stats']['lost_status_lines'], 0)
        self.assertTrue(h.bags[0].stop_requested)
        check = sc.check_recording(h.archive / rid)
        self.assertTrue(check['passed'], check['segments'][0]['problems'])
        self.assertEqual(h.controller.status()['state'], 'idle')
        self.assertEqual(sorted(p.name for p in (h.archive / rid).iterdir()), ['metadata.json', 'steering'])
        self.assertEqual(sorted(p.name for p in (h.archive / rid / 'steering').iterdir()),
                         sorted([sc.BAG_DIR, sc.BAG_PID, sc.DESCRIPTOR_FILE, sc.DESCRIPTOR_SHA_FILE, sc.FRAMES_FILE,
                                 sc.HOST_PROFILE_COPY]))

    def test_messages_before_the_recording_start_are_not_written(self):
        h = Harness(self)
        h.stream(h.controller, 1.0)             # 20 messages before the start (buffer 3 s)
        h.clock.advance(0.01)
        rid = self.start(h)
        h.stream(h.controller, 0.2, seq0=20)    # 4 after the start, before it is seen
        self.poll(h, 'recording', rid)
        self.assertEqual(h.controller.segment.frame_count, 4)
        self.assertTrue(all(f['t_s'] >= 0 for f in h.controller.segment.preview.frames))

    def test_stop_before_the_first_sample(self):
        h = Harness(self)
        rid = self.start(h)
        self.poll(h, 'recording', rid)
        self.finish(h, duration=0.2)
        d = descriptor_of(segment_for(h, rid))
        self.assertEqual((d['frame_count'], d['handshake']['end_reason']), (0, 'recorder_stop'))
        self.assertEqual(d['stats']['unique_status_lines'], 0)
        check = sc.check_recording(h.archive / rid)['segments'][0]
        self.assertTrue(check['checks']['frames_schema'] and check['checks']['descriptor_content_sha256'])
        self.assertEqual((h.archive / rid / 'steering' / sc.FRAMES_FILE).read_bytes(), b'')

    def test_recording_seen_only_while_finalizing(self):
        h = Harness(self)
        h.stream(h.controller, 0.5)             # before the start: not written
        h.clock.advance(0.01)
        rid = self.start(h)
        h.stream(h.controller, 0.1, seq0=10)    # start and stop between two polls: the buffer keeps these two
        self.finish(h, duration=0.1)
        d = descriptor_of(segment_for(h, rid))
        self.assertEqual(d['handshake']['end_reason'], 'recording_seen_only_while_finalizing')
        self.assertEqual(d['handshake']['open_reason'], 'recorder_status_finalizing_only')
        self.assertEqual((d['frame_count'], d['stats']['replayed_from_buffer'], d['bag']), (2, 2, None))
        self.assertEqual(h.bags, [])
        self.assertEqual(h.controller.counts['missed_recordings'], 1)

    def test_recorder_restart_quick(self):
        h = Harness(self)
        rid = self.start(h)
        self.poll(h, 'recording', rid)
        h.stream(h.controller, 1.0)
        self.poll(h, 'idle')                    # restarted recorder: idle, metadata still 'recording'
        h.controller.tick()
        d = descriptor_of(segment_for(h, rid))
        self.assertEqual(d['handshake']['end_reason'], 'recorder_lost')
        self.assertEqual(d['handshake']['recorder_final']['state'], 'recording')

    def test_recorder_unreachable(self):
        h = Harness(self)
        rid = self.start(h)
        self.poll(h, 'recording', rid)
        for i in range(21):                     # 0.25 s steps (exact): the 21st failed poll is 5.0 s after the first
            h.clock.advance(0.25)
            h.send(h.controller, i, 250 * i)
            self.assertIsNotNone(h.controller.segment)
            h.controller.on_status(dict(ok=False, error='ConnectionRefusedError') if i % 2 else None)
        self.assertIsNone(h.controller.segment)
        h.controller.tick()
        self.assertFalse((segment_for(h, rid) / sc.DESCRIPTOR_FILE).exists())   # waits for the recorder's end
        self.poll(h, 'idle')
        h.controller.tick()
        d = descriptor_of(segment_for(h, rid))
        self.assertEqual(d['handshake']['end_reason'], 'recorder_lost')
        self.assertEqual(d['frame_count'], 21)

    def test_recorder_never_answers_again(self):
        h = Harness(self, finalize_wait_s=10.0)
        rid = self.start(h)
        self.poll(h, 'recording', rid)
        for _ in range(100):                    # closed after 5 s, descriptor 10 s later without the recorder
            h.clock.advance(0.2)
            h.controller.on_status(None)
            h.controller.tick()
        d = descriptor_of(segment_for(h, rid))
        self.assertEqual(d['handshake']['end_reason'], 'recorder_unreachable')
        self.assertFalse(d['handshake']['recorder_final']['complete'])

    def test_recording_switch(self):
        h = Harness(self)
        rid = self.start(h)
        self.poll(h, 'recording', rid)
        h.stream(h.controller, 0.5)
        h.recorder.archive_rec(rid, 0.5)
        rid2 = '20261109T010210Z_both'
        self.start(h, rid2)
        self.poll(h, 'recording', rid2)
        self.assertEqual(h.controller.segment.recording_id, rid2)
        h.controller.tick()
        self.assertEqual(descriptor_of(segment_for(h, rid))['handshake']['end_reason'], 'recorder_switched')

    def test_handshake_never_creates_the_recording_folder(self):
        h = Harness(self, open_timeout_s=1.0)
        rid = self.RID
        status = dict(ok=True, state='recording', recording_id=rid, archive_dir=str(h.archive),
                      started_at=iso(START_WALL))
        for _ in range(8):
            h.clock.advance(0.2)
            h.controller.on_status(status)
        self.assertFalse((h.archive / rid).exists())
        self.assertIsNone(h.controller.segment)
        self.assertEqual(h.controller.failed[rid], 'recording_handshake_failed: recording_folder_missing')

    def test_handshake_checks_metadata_and_archive_dir(self):
        h = Harness(self)
        rid = self.start(h)
        status = h.recorder.status('recording', rid)
        for change, reason in ((dict(started_at=iso(START_WALL - 60)), 'started_at_mismatch'),
                               (dict(archive_dir='/elsewhere/recordings'), 'archive_dir_mismatch')):
            recording, why = h.controller._handshake(rid, dict(status, **change))
            self.assertEqual((recording, why), (None, reason))
        meta_path = h.archive / rid / 'metadata.json'
        meta = json.loads(meta_path.read_bytes())
        meta_path.write_bytes(json.dumps(dict(meta, id='other')).encode())
        self.assertEqual(h.controller._handshake(rid, status), (None, 'metadata_id_mismatch'))
        meta_path.write_bytes(json.dumps(dict(meta, state='archived')).encode())
        self.assertEqual(h.controller._handshake(rid, status), (None, 'metadata_not_recording'))
        for bad in ('../x', '', None, 'a/b'):
            h.controller.on_status(dict(status, recording_id=bad))
        self.assertIsNone(h.controller.segment)

    def test_sidecar_crash_and_restart(self):
        h = Harness(self)
        rid = self.start(h)
        self.poll(h, 'recording', rid)
        h.stream(h.controller, 2.0)
        first = h.controller.segment
        first.handle.write('{"t_s":2.1,"elapsed_mono')      # torn last line (power cut while writing)
        first.handle.close()                                  # the process is gone: no descriptor
        before = (segment_for(h, rid) / sc.FRAMES_FILE).read_bytes()
        restarted = h.make(bag=lambda out, log: FakeBag(out, log))
        self.assertEqual(restarted.recover_orphans(), 1)
        d = descriptor_of(segment_for(h, rid))
        self.assertEqual(d['handshake']['end_reason'], 'sidecar_restarted_unclean')
        self.assertEqual(d['handshake']['malformed_lines'], 1)
        self.assertEqual(d['frame_count'], 40)
        self.assertEqual(d['calibration']['host_profile_sha256'], h.sha)    # from the segment's host profile copy
        self.assertTrue(d['error'].startswith('recovered_without_descriptor'))
        self.assertEqual((segment_for(h, rid) / sc.FRAMES_FILE).read_bytes(), before)    # never rewritten
        self.poll(h, 'recording', rid, restarted)
        self.assertEqual(restarted.segment.dir.name, 'steering-2')
        h.stream(restarted, 1.0, seq0=60)
        self.finish(h, controller=restarted)
        names = [p.name for p in sc.segment_dirs(h.archive / rid)]
        self.assertEqual(names, ['steering', 'steering-2'])
        second = descriptor_of(segment_for(h, rid, 'steering-2'))
        self.assertEqual((second['frame_count'], second['handshake']['end_reason']), (20, 'recorder_stop'))
        result = sc.check_recording(h.archive / rid)
        self.assertEqual([s['passed'] for s in result['segments']], [False, True])   # the recovered one is marked

    def test_restart_while_finalizing_recovers_without_a_new_segment(self):
        h = Harness(self)
        rid = self.start(h)
        self.poll(h, 'recording', rid)
        h.stream(h.controller, 1.0)
        h.controller.segment.handle.close()
        restarted = h.make()
        self.poll(h, 'finalizing', rid, restarted)
        self.assertEqual([p.name for p in sc.segment_dirs(h.archive / rid)], ['steering'])
        self.assertEqual(descriptor_of(segment_for(h, rid))['handshake']['end_reason'], 'sidecar_restarted_unclean')

    def test_shutdown_during_a_recording(self):
        h = Harness(self, bag_kwargs=dict(exit_polls=3))
        rid = self.start(h)
        self.poll(h, 'recording', rid)
        h.stream(h.controller, 1.0)
        h.controller.shutdown('sidecar_stopped', limit_s=2.0)
        d = descriptor_of(segment_for(h, rid))
        self.assertEqual(d['handshake']['end_reason'], 'sidecar_stopped')
        self.assertFalse(d['handshake']['recorder_final']['complete'])
        self.assertEqual(d['bag']['exit_code'], 0)
        self.assertEqual(h.controller.closing, [])

    def test_bag_start_failure_keeps_the_frames(self):
        h = Harness(self, bag_kwargs=dict(fail=True))
        rid = self.start(h)
        self.poll(h, 'recording', rid)
        h.stream(h.controller, 1.0)
        self.finish(h)
        d = descriptor_of(segment_for(h, rid))
        self.assertEqual(d['frame_count'], 20)
        self.assertTrue(d['bag_error'].startswith('bag_start_failed'))
        self.assertIn('bag_finalized', sc.check_recording(h.archive / rid)['segments'][0]['problems'])

    def test_without_host_profile(self):
        h = Harness(self, host_profile=False)
        rid = self.start(h)
        self.poll(h, 'recording', rid)
        h.stream(h.controller, 1.0)
        self.finish(h)
        d = descriptor_of(segment_for(h, rid))
        self.assertIsNone(d['calibration'])
        self.assertEqual(d['calibration_error'], 'host_profile_not_configured')
        self.assertEqual(d['stats']['angle_checks'], {'no_table': 20})
        self.assertEqual((d['stale_after_s'], d['sample_hz']), (sc.DEFAULT_STALE_AFTER_S, sc.DEFAULT_SAMPLE_HZ))
        frames = (segment_for(h, rid) / sc.FRAMES_FILE).read_text(encoding='utf-8').splitlines()
        self.assertTrue(all(json.loads(line)['angle_mrad'] is None for line in frames))

    def test_bag_time_mode(self):
        self.assertTrue(sc.bag_time_mode('auto', {'DASHBOARD_MODE': 'simulation'}))
        self.assertFalse(sc.bag_time_mode('auto', {}))
        self.assertTrue(sc.bag_time_mode('sim', {}))
        self.assertFalse(sc.bag_time_mode('wall', {'DASHBOARD_MODE': 'simulation'}))

    def test_status_file_shape(self):
        h = Harness(self)
        status = h.controller.status()
        self.assertEqual(status['schema'], sc.STATUS_SCHEMA)
        self.assertEqual(status['state'], 'idle')
        json.dumps(status, allow_nan=False)
        target = Path(h.tmp.name) / 'status.json'
        sc._write_status(str(target), status)
        self.assertEqual(json.loads(target.read_bytes())['schema'], sc.STATUS_SCHEMA)


class BagMetadataTest(unittest.TestCase):
    def test_read_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / 'bag'
            self.assertEqual(bag_mod.read_metadata(folder)['error'], 'metadata_missing')
            fake = FakeBag(folder, Path(tmp) / 'log', count=12)
            fake.start()
            fake.request_stop()
            fake.poll()
            meta = bag_mod.read_metadata(folder)
            self.assertTrue(meta['finalized'])
            self.assertEqual(meta['topics'], {'/rover/steering/state': {'type': 'std_msgs/msg/String',
                                                                        'message_count': 12}})
            self.assertEqual((meta['message_count'], meta['storage'], meta['duration_s']), (12, 'mcap', 1.0))
            (folder / 'metadata.yaml').write_text('rosbag2_bagfile_information: [1, 2', encoding='utf-8')
            self.assertTrue(bag_mod.read_metadata(folder)['error'].startswith('metadata_malformed'))

    def test_command_line(self):
        bag = bag_mod.BagRecord('/x/steering/bag', use_sim_time=True, guard=['setpriv', '--pdeathsig', 'INT', '--'])
        cmd = bag.command()
        self.assertEqual(cmd[:4], ['setpriv', '--pdeathsig', 'INT', '--'])
        self.assertEqual(cmd[4:7], ['ros2', 'bag', 'record'])
        for pair in (['-o', str(Path('/x/steering/bag'))], ['-s', 'mcap'], ['--topics', '/rover/steering/state'],
                     ['--node-name', 'ubts_steering_bag'], ['--max-cache-size', '1048576']):
            i = cmd.index(pair[0])
            self.assertEqual(cmd[i:i + 2], pair)
        self.assertIn('--use-sim-time', cmd)
        self.assertIn('--disable-keyboard-controls', cmd)
        self.assertNotIn('--use-sim-time', bag_mod.BagRecord('/x/b', guard=[]).command())
        with self.assertRaises(bag_mod.BagRecordError):
            bag_mod.BagRecord('/x/b', topics=('rover/steering/state',))

    def test_result_before_start(self):
        result = bag_mod.BagRecord('/nonexistent-steering-bag', guard=[]).result()
        self.assertEqual((result['started'], result['exit_code'], result['orphan_guard']), (False, None, 'none'))


if __name__ == '__main__':
    unittest.main()
