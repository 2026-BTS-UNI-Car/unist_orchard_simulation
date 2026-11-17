"""Bridge core with a scripted fake V3 MCU on a MemoryTransport and an injected clock (no hardware, no ROS).

Covers STOP confirm/resend/latch timing, C resend and TTL, request expiry, busy, the queue limit, the host latches
and the operator re-arm, every error code of design 4.2, the steering block and requested.left/right.
All numbers are placeholder values (placeholder-based SIM test, not field data).
"""
import json
import math
import unittest

try:
    import test_steer_support as ts      # unittest discover -s dashboard/scripts
except ImportError:                     # pytest imports these modules as scripts.test_steer_*
    from scripts import test_steer_support as ts
from scripts import esp_motor_v3 as v3
from scripts import steer_bridge_core as core_mod

ARM = {'op': 'arm', 'owner': 'operator-a', 'vehicle_profile': 'front_steer'}


def drive(reply, throttle=0, steer_mrad=0, owner='operator-a', **extra):
    request = ts.Rig.next_request(reply, 'drive', owner=owner, deadman=True, throttle=throttle, steer_mrad=steer_mrad)
    request.update(extra)
    return request


def c_lines(rig, since_t=None):
    """(t, q, ttl, L, R, S) of every C line the host wrote (from the link tx log)."""
    out = []
    for t, label, data in rig.link.tx_log:
        if label == 'C' and (since_t is None or t > since_t):
            body, crc = data[:-1].split(b'*')
            assert crc == b'%02X' % v3.crc8(body)
            _, q, ttl, left, right, steer = body.split(b' ')
            out.append((t, int(q), int(ttl), int(left), int(right), int(steer)))
    return out


def tx_times(rig, label):
    return [t for t, lab, _ in rig.link.tx_log if lab == label]


class StartupTest(unittest.TestCase):
    def test_session_stop_ping_and_no_c_before_arm(self):
        rig = ts.Rig()
        rig.ready()
        self.assertEqual(rig.tx_kinds(skip=())[:2], ['STOP', 'PING'])
        rig.run(1.0)
        pings = rig.tx_kinds(skip=()).count('PING')
        self.assertTrue(10 <= pings <= 12, pings)
        self.assertNotIn('C', rig.tx_kinds())
        self.assertNotIn('ARM', rig.tx_kinds())
        snap = rig.core.snapshot
        self.assertEqual((snap['state'], snap['armed'], snap['connected'], snap['reason']),
                         ('disarmed', False, True, 'startup'))


class ArmTest(unittest.TestCase):
    def test_arm_needs_the_vehicle_profile(self):
        rig = ts.Rig()
        rig.ready()
        for request in ({'op': 'arm', 'owner': 'operator-a'},
                        {'op': 'arm', 'owner': 'operator-a', 'vehicle_profile': 'differential'}):
            self.assertEqual(rig.submit(request), {'ok': False, 'error': 'steer_profile_required'})
        self.assertEqual(rig.submit({'op': 'arm', 'vehicle_profile': 'front_steer'})['error'], 'operator_required')
        self.assertIsNone(rig.core.control.lease)
        self.assertNotIn('ARM', rig.tx_kinds())

    def test_arm_sends_stop_then_arm_and_waits_for_confirmation(self):
        rig = ts.Rig()
        rig.ready()
        n0 = len(rig.link.tx_log)
        reply = rig.submit(ARM)
        self.assertTrue(reply['ok'])
        self.assertTrue(reply['lease'] and reply['ticket'])
        self.assertEqual(reply['seq'], 0)
        self.assertEqual(reply['status']['steering_host']['arm_state'], 'pending')
        lines = [data[:-1] for _, label, data in list(rig.link.tx_log)[n0:] if label != 'PING']
        self.assertEqual(lines[:2], [b'STOP', b'ARM ' + ts.BOOT_IDS[0].encode() + b'*%02X'
                                     % v3.crc8(b'ARM ' + ts.BOOT_IDS[0].encode())])
        self.assertTrue(rig.run_until(lambda: rig.core.steering_host()['arm_state'] == 'confirmed', 0.1))
        self.assertEqual(rig.core.snapshot['state'], 'armed_manual')
        self.assertEqual(rig.core.snapshot['steering']['faults'], [])    # ARM cleared BOOT

    def test_arm_blockers(self):
        def stale(rig):
            rig.mcu.send_status = False

        def ignore_stop(rig):
            rig.mcu.ignore_stop = True

        def polarity_latch(rig):
            rig.mcu.fl |= 0x0020           # POLARITY latched in the MCU (status fl)
        # (expected error, fake MCU options, change before the run, change after 0.2 s)
        cases = [
            ('firmware_v3_required', dict(reset_on_open=False), None, None),
            ('steering_uncalibrated', dict(cal_id=None), None, None),
            ('calibration_mismatch', dict(cal_id=8), None, None),
            ('calibration_mismatch', dict(caldirty=True), None, None),
            ('polarity_check_missing', dict(pol_ok=False), None, None),
            ('polarity_check_missing', {}, None, polarity_latch),     # spec 12.1-71: CAL POL first
            ('config_unsaved', dict(cfgdirty=True), None, None),       # spec 12.1-74
            ('hardware_profile_mismatch', dict(hw='front_steer_v9'), None, None),
            ('sim_identity_mismatch', dict(sim=False), None, None),
            ('link_config_mismatch', dict(wd=600), None, None),
            ('link_config_mismatch', dict(ttlmax=250), None, None),
            ('steering_status_stale', {}, None, stale),
            ('stop_unconfirmed', dict(start_armed=True), ignore_stop, None),
        ]
        for code, mcu, before, after in cases:
            with self.subTest(code=code, mcu=mcu):
                rig = ts.Rig(**mcu)
                if before is not None:
                    before(rig)
                rig.run(0.2)
                if after is not None:
                    after(rig)
                    rig.run(0.3)
                self.assertEqual(rig.submit(ARM), {'ok': False, 'error': code})
                self.assertNotIn('ARM', rig.tx_kinds())
                self.assertIsNone(rig.core.control.lease)

    def test_mcu_rejection_of_arm_stops(self):
        rig = ts.Rig()
        rig.ready()
        rig.mcu.arm_reject = 'fault'
        self.assertTrue(rig.submit(ARM)['ok'])
        rig.run(0.05)
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'steering_fault_latched')
        self.assertEqual(rig.core.last_error, 'arm_rejected:fault')

    def test_refused_or_unconfirmed_rearm_keeps_the_latches(self):
        # Spec 12.1-75 (2026-11-09 review): the operator re-arm clears the host latches only when the MCU confirms the
        # ARM. A refused ARM (N - fault), an ARM without confirmation or a failed ARM write keeps them, adds its own
        # latch and leaves the state at fault.
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        rig.submit(drive(reply, 0, 100))
        rig.mcu.st, rig.mcu.L, rig.mcu.R = 1, 0, 0                      # the MCU disarms itself on a fault
        rig.mcu.fl |= 0x0002
        rig.mcu._send(rig.clock.t, 'F POT_OPEN 3 %d' % rig.mcu.ms(rig.clock.t))
        rig.run(0.2)
        self.assertEqual(list(rig.core.latches), ['mcu_disarmed'])
        self.assertEqual(rig.core.snapshot['state'], 'fault')
        rig.mcu.arm_reject = 'fault'                                    # the pot condition is still there
        self.assertTrue(rig.submit(ARM)['ok'])
        rig.run(0.2)
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'steering_fault_latched')
        self.assertEqual(list(rig.core.latches), ['mcu_disarmed', 'arm_rejected'])
        self.assertEqual(rig.core.snapshot['state'], 'fault')
        self.assertIn('mcu_disarmed', rig.core.snapshot['steering']['faults'])
        rig.mcu.arm_reject = None
        rig.mcu.ignore_arm = True                                       # the next ARM is lost on the way
        self.assertTrue(rig.submit(ARM)['ok'])
        rig.run(0.3)
        self.assertEqual(rig.core.control.reason, 'arm_unconfirmed')
        self.assertEqual(list(rig.core.latches), ['mcu_disarmed', 'arm_rejected', 'arm_unconfirmed'])
        self.assertEqual(rig.core.snapshot['state'], 'fault')
        rig.mcu.ignore_arm = False
        rig.transport.fail_writes = True                                # the ARM write fails
        self.assertEqual(rig.submit(ARM), {'ok': False, 'error': 'hardware_fault'})
        self.assertEqual(set(rig.core.latches), {'mcu_disarmed', 'arm_rejected', 'arm_unconfirmed', 'hardware_fault'})
        rig.transport.fail_writes = False
        rig.run(1.2)                                                    # reopened (a new boot), no automatic re-arm
        rig.ready()
        self.assertIsNone(rig.core.control.lease)
        self.assertTrue(rig.core.latches)
        rig.arm()                                                       # confirmed: now the latches go
        self.assertEqual(rig.core.steering_host()['arm_state'], 'confirmed')
        self.assertEqual(rig.core.latches, {})
        self.assertEqual(rig.core.snapshot['state'], 'armed_manual')

    def test_arm_unconfirmed_stops(self):
        rig = ts.Rig()
        rig.ready()
        rig.mcu.ignore_arm = True
        self.assertTrue(rig.submit(ARM)['ok'])
        rig.run(0.13)
        self.assertIsNotNone(rig.core.control.lease)
        rig.run(0.05)
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'arm_unconfirmed')

    def test_second_operator_cannot_take_control(self):
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        self.assertEqual(rig.submit(dict(ARM, owner='operator-b'))['error'], 'control_in_use')
        self.assertEqual(rig.submit(drive(reply, 30, 0, owner='operator-b'))['error'], 'control_not_owned')
        self.assertIsNotNone(rig.core.control.lease)


class DriveTest(unittest.TestCase):
    def armed(self, **mcu):
        rig = ts.Rig(**mcu)
        rig.ready()
        return rig, rig.arm()

    def test_c_frame_content_and_q(self):
        rig, reply = self.armed()
        t0 = rig.clock.t
        reply = rig.submit(drive(reply, 40, -120))
        self.assertTrue(reply['ok'])
        rig.run(0.2)
        cs = c_lines(rig, t0)
        self.assertGreaterEqual(len(cs), 4)
        self.assertEqual([c[1] for c in cs], list(range(1, len(cs) + 1)))   # new boot: q from 1, +1 per C line
        self.assertTrue(all(c[3:] == (102, 102, -274) for c in cs))       # duty (40*255+50)//100, S from the marks
        for throttle, steer, expected in ((100, 409, (255, 255, 934)), (-100, -409, (-255, -255, -934)),
                                          (1, 1, (3, 3, 2)), (0, 0, (0, 0, 0))):
            t1 = rig.clock.t
            reply = rig.submit(drive(reply, throttle, steer))
            self.assertTrue(reply['ok'])
            self.assertEqual(c_lines(rig, t1)[0][3:], expected)

    def test_c_resend_rate_and_ttl_bound(self):
        rig, reply = self.armed()
        t0 = rig.clock.t
        reply = rig.submit(drive(reply, 30, 50))
        deadline = rig.core.control.input_until
        rig.run(1.0)
        cs = c_lines(rig, t0)
        self.assertTrue(14 <= len(cs) <= 16, len(cs))
        for t, q, ttl, *_ in cs:
            self.assertLess(t, deadline)
            self.assertEqual(ttl, min(300, int(math.floor((deadline - t) * 1000.0))))
        gaps = [b[0] - a[0] for a, b in zip(cs, cs[1:])]
        self.assertTrue(all(0.039 <= g <= 0.061 for g in gaps), gaps)
        self.assertLess(cs[-1][2], 300)                 # the last C ends no later than the input deadline
        self.assertEqual(rig.core.control.reason, 'input_expired')

    def test_drive_before_arm_confirmation_waits(self):
        rig = ts.Rig(latency_s=0.03)
        rig.ready()
        reply = rig.submit(ARM)
        t0 = rig.clock.t
        reply = rig.submit(drive(reply, 20, 100))
        self.assertTrue(reply['ok'])
        self.assertEqual(rig.core.steering_host()['arm_state'], 'pending')
        self.assertEqual(c_lines(rig, t0), [])
        self.assertTrue(rig.run_until(lambda: rig.core.steering_host()['arm_state'] == 'confirmed', 0.3))
        t_confirm = rig.clock.t
        cs = c_lines(rig, t0)
        self.assertTrue(cs and abs(cs[0][0] - t_confirm) < 1e-9)          # sent in the confirming step

    def test_invalid_drive_values_stop(self):
        bad = [(True, 0), (40.0, 0), ('40', 0), (101, 0), (-101, 0), (None, 0), (40, 410), (40, -410), (40, 1.5),
               (40, None), (40, False)]
        for throttle, steer in bad:
            with self.subTest(throttle=throttle, steer=steer):
                rig, reply = self.armed()
                self.assertEqual(rig.submit(drive(reply, throttle, steer))['error'], 'invalid_drive')
                self.assertIsNone(rig.core.control.lease)
                self.assertEqual(rig.core.control.reason, 'invalid_drive')
        for deadman in (False, None, 1):
            rig, reply = self.armed()
            self.assertEqual(rig.submit(drive(reply, 10, 0, deadman=deadman))['error'], 'invalid_drive')
        rig, reply = self.armed()
        self.assertTrue(rig.submit(drive(reply, 10, 409))['ok'])
        rig, reply = self.armed()
        self.assertTrue(rig.submit(drive(reply, 10, -409))['ok'])

    def test_differential_drive_is_rejected_even_from_a_non_owner(self):
        rig, reply = self.armed()
        old = {'op': 'drive', 'owner': 'operator-b', 'deadman': True, 'left': 30, 'right': 30}
        self.assertEqual(rig.submit(old)['error'], 'differential_command_rejected')
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'differential_command_rejected')
        rig, reply = self.armed()
        self.assertEqual(rig.submit(drive(reply, 30, 0, left=30))['error'], 'differential_command_rejected')

    def test_keepalive_does_not_extend_the_motion_deadline(self):
        rig, reply = self.armed()
        reply = rig.submit(drive(reply, 30, 0))
        t_drive = rig.clock.t
        rig.run(0.58)
        reply = rig.submit(rig.next_request(reply, 'keepalive'))
        self.assertTrue(reply['ok'])
        rig.run_until(lambda: rig.core.control.lease is None, 0.5)
        self.assertEqual(rig.core.control.reason, 'input_expired')
        self.assertAlmostEqual(rig.clock.t - t_drive, 0.75, delta=0.011)
        self.assertTrue(all(c[0] < t_drive + 0.75 for c in c_lines(rig)))

    def test_release_keeps_the_steering_target_and_ramps_out(self):
        rig, reply = self.armed()
        reply = rig.submit(drive(reply, 50, 200))
        rig.run(0.1)
        t_release = rig.clock.t
        reply = rig.submit(rig.next_request(reply, 'release'))
        self.assertTrue(reply['ok'])
        self.assertEqual(reply['status']['requested'], {'throttle': 0, 'steer_mrad': 200, 'left': 0, 'right': 0})
        rig.run(1.3)
        after = c_lines(rig, t_release)
        self.assertTrue(after and all(c[3:] == (0, 0, 457) for c in after))
        self.assertTrue(all(c[0] < t_release + 1.11 for c in after))
        self.assertIsNotNone(rig.core.control.lease)      # stopped within 1.1 s: lease kept (no automatic stop)
        self.assertFalse(rig.core.control.moving)

    def test_steer_only_commands_need_lease_deadman_and_arm(self):
        rig = ts.Rig()
        rig.ready()
        self.assertEqual(rig.submit({'op': 'drive', 'owner': 'operator-a', 'lease': None, 'ticket': None, 'seq': 1,
                                     'deadman': True, 'throttle': 0, 'steer_mrad': 100})['error'],
                         'control_not_owned')
        reply = rig.arm()
        self.assertEqual(rig.submit(drive(reply, 0, 100, deadman=False))['error'], 'invalid_drive')
        rig, reply = self.armed()
        t0 = rig.clock.t
        self.assertTrue(rig.submit(drive(reply, 0, 100))['ok'])
        rig.run(1.0)
        cs = c_lines(rig, t0)
        self.assertTrue(cs and all(c[3:] == (0, 0, 228) for c in cs))
        self.assertIsNotNone(rig.core.control.lease)       # not moving: no input_expired stop
        self.assertGreaterEqual(rig.core.counters['mcu_fault_TTL'], 1)   # the MCU TTL ended the drive window
        self.assertEqual(rig.core.snapshot['steering']['stop_mode'], 'brake')


class StopTest(unittest.TestCase):
    def test_stop_skips_the_queue_and_goes_out_within_one_tick(self):
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        pending = rig.core.submit(drive(reply, 40, 0))
        stop = rig.core.submit({'op': 'stop', 'reason': 'ignored'})
        self.assertEqual(stop.result(), {'ok': True, 'state': 'stop_requested'})
        n0 = len(rig.link.tx_log)
        t_request = rig.clock.t
        rig.tick()
        self.assertEqual(pending.result(), {'ok': False, 'error': 'stop_barrier'})
        sent = [(t, label) for t, label, _ in list(rig.link.tx_log)[n0:]]
        self.assertEqual(sent[0][1], 'STOP')
        self.assertLessEqual(sent[0][0] - t_request, 0.02)
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'operator_stop')
        self.assertEqual(c_lines(rig), [])

    def test_stop_confirmed_within_two_status_periods(self):
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        rig.submit(drive(reply, 40, 100))
        rig.run(0.07)
        rig.core.submit({'op': 'stop'})
        rig.tick()
        t_stop = tx_times(rig, 'STOP')[-1]
        self.assertTrue(rig.run_until(lambda: rig.core.steering_host()['stop_state'] == 'confirmed', 0.2))
        self.assertLessEqual(rig.clock.t - t_stop, 2 * 0.05 + 1e-9)
        self.assertEqual(tx_times(rig, 'STOP')[-1], t_stop)                 # no resend needed
        block = rig.core.snapshot['steering']
        self.assertEqual((block['stop_mode'], block['drive_en'], block['steer_en']), ('brake', True, True))

    def test_stop_resend_every_50_ms_five_times_then_latch(self):
        rig = ts.Rig(start_armed=True)
        rig.mcu.ignore_stop = True
        rig.run(0.6)
        times = tx_times(rig, 'STOP')
        self.assertEqual(len(times), 6)
        t0 = times[0]
        expected = [0.0, 0.10, 0.15, 0.20, 0.25, 0.30]
        for got, want in zip(times, expected):
            self.assertAlmostEqual(got - t0, want, delta=0.0105)
        latch = rig.core.latches['stop_unconfirmed']
        self.assertAlmostEqual(latch['since'] - t0, 0.40, delta=0.0105)
        self.assertEqual(rig.core.snapshot['state'], 'fault')
        self.assertIn('stop_unconfirmed', rig.core.snapshot['steering']['faults'])
        self.assertEqual(rig.core.steering_host()['stop_state'], 'unconfirmed')
        rig.run(0.5)
        self.assertEqual(len(tx_times(rig, 'STOP')), 6)                    # no more resends after the latch
        self.assertEqual(rig.submit(ARM)['error'], 'stop_unconfirmed')

    def test_stop_resend_grid_with_the_20_ms_wrapper_loop(self):
        # The grid is fixed from t0 (spec 12.1-62): a 20 ms loop rounds each resend up to its next step but does
        # not stretch the grid (re-measuring from the previous resend would give 60 ms steps and t0 + 440 ms).
        rig = ts.Rig(step_s=0.02, start_armed=True)
        rig.mcu.ignore_stop = True
        rig.run(0.7)
        times = tx_times(rig, 'STOP')
        t0 = times[0]
        self.assertEqual([int(round((t - t0) * 1000)) for t in times], [0, 100, 160, 200, 260, 300])
        self.assertEqual(int(round((rig.core.latches['stop_unconfirmed']['since'] - t0) * 1000)), 400)

    def test_stop_while_the_arm_is_in_flight(self):
        # Spec 12.1-77 (2026-11-09 review): with 30 ms of serial delay a status line made before the MCU took the ARM
        # can be read after the STOP. It must not confirm that STOP: here the STOP is lost and the MCU is ARMED.
        for phase in range(5):
            with self.subTest(phase=phase):
                rig = ts.Rig(step_s=0.01, latency_s=0.03)
                rig.ready()
                rig.run(0.01 * phase)
                self.assertTrue(rig.submit(ARM)['ok'])                   # STOP + ARM written
                rig.core.submit({'op': 'stop'})
                confirmed = rig.core.counters['stop_confirmed']
                rig.tick()                                               # MCU: ARMED; host: STOP written
                rig.mcu.ignore_stop = True                               # that STOP never takes effect
                for _ in range(30):
                    self.assertFalse(rig.mcu.st == 2 and rig.core.steering_host()['stop_state'] == 'confirmed')
                    rig.tick()
                self.assertEqual(rig.core.counters['stop_confirmed'], confirmed)
                rig.run(0.2)
                self.assertIn('stop_unconfirmed', rig.core.latches)      # the resends could not stop it
        # When the STOP does take effect it is confirmed by a later line, within the confirmation window.
        rig = ts.Rig(step_s=0.01, latency_s=0.03)
        rig.ready()
        rig.run(0.03)
        self.assertTrue(rig.submit(ARM)['ok'])
        rig.core.submit({'op': 'stop'})
        rig.tick()
        t_stop = rig.clock.t
        self.assertTrue(rig.run_until(lambda: rig.core.steering_host()['stop_state'] == 'confirmed', 0.3))
        self.assertEqual(rig.mcu.st, 1)
        self.assertLessEqual(rig.clock.t - t_stop, 0.15 + 1e-9)

    def test_late_drive_after_stop_is_refused(self):
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        reply = rig.submit(drive(reply, 30, 0))
        rig.core.submit({'op': 'stop'})
        rig.tick()
        t_stop = rig.clock.t
        self.assertEqual(rig.submit(drive(reply, 30, 0))['error'], 'control_not_owned')
        rig.run(0.2)
        self.assertEqual(c_lines(rig, t_stop), [])


class QueueTest(unittest.TestCase):
    def test_request_expiry_does_not_stop(self):
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        future = rig.core.submit(drive(reply, 20, 0))
        rig.clock.t += 0.3
        rig.mcu.tick(rig.clock.t)
        rig.core.step()
        self.assertEqual(future.result(), {'ok': False, 'error': 'request_expired'})
        self.assertIsNotNone(rig.core.control.lease)

    def test_busy_queue_stops(self):
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        futures = [rig.core.submit(rig.next_request(reply, 'keepalive')) for _ in range(8)]
        ninth = rig.core.submit(rig.next_request(reply, 'keepalive'))
        self.assertEqual(ninth.result(), {'ok': False, 'error': 'control_busy_stopped'})
        self.assertTrue(rig.core.emergency.is_set())
        rig.tick()
        self.assertEqual([f.result()['error'] for f in futures], ['stop_barrier'] * 8)
        self.assertIsNone(rig.core.control.lease)

    def test_one_request_per_tick(self):
        rig = ts.Rig()
        rig.ready()
        futures = [rig.core.submit({'op': 'keepalive', 'owner': 'x'}) for _ in range(3)]
        rig.tick()
        self.assertEqual([f.done() for f in futures], [True, False, False])
        rig.tick()
        rig.tick()
        self.assertTrue(all(f.result() == {'ok': False, 'error': 'control_not_owned'} for f in futures))

    def test_blocking_request_timeout_is_busy_stop(self):
        rig = ts.Rig()
        rig.ready()
        self.assertEqual(rig.core.request({'op': 'keepalive', 'owner': 'x'}, timeout=0.01),
                         {'ok': False, 'error': 'control_busy_stopped'})
        self.assertTrue(rig.core.emergency.is_set())
        self.assertEqual(rig.core.request({'op': 'status'})['state'], 'disarmed')
        self.assertEqual(rig.core.request(['not', 'a', 'dict']), {'ok': False, 'error': 'object_required'})


class HostFaultTest(unittest.TestCase):
    def armed_driving(self, **mcu):
        rig = ts.Rig(**mcu)
        rig.ready()
        reply = rig.arm()
        reply = rig.submit(drive(reply, 0, 100))
        return rig, reply

    def test_status_gap_latch_and_operator_rearm(self):
        rig, reply = self.armed_driving()
        rig.mcu.send_status = False
        last = rig.link.status_t
        rig.run(0.25)
        latch = rig.core.latches['steering_status_stale']
        self.assertLessEqual(latch['since'] - last, 0.15 + 0.011)
        self.assertGreater(latch['since'] - last, 0.15)
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'steering_status_stale')
        self.assertEqual(rig.core.snapshot['state'], 'fault')
        self.assertEqual(rig.submit(ARM)['error'], 'steering_status_stale')
        rig.mcu.send_status = True
        rig.run(0.5)
        self.assertIsNone(rig.core.control.lease)                          # no automatic re-arm
        self.assertIn('steering_status_stale', rig.core.latches)            # latch kept until the operator re-arms
        rig.arm()
        self.assertEqual(rig.core.latches, {})
        self.assertEqual(rig.core.snapshot['state'], 'armed_manual')

    def test_mcu_armed_or_bench_without_a_host_lease_latches_and_stops(self):
        # Open P3 item 1 (A19 host part): another writer on the port (a CLI, a second process) armed the MCU. The next
        # status line shows ARMED / BENCH while the host holds no lease, waits for no ARM and has no STOP pending.
        for st, name in ((2, 'ARMED'), (3, 'BENCH')):
            with self.subTest(state=name):
                rig = ts.Rig()
                rig.ready()
                rig.run(0.1)
                self.assertEqual(rig.core.latches, {})
                since = len(rig.mcu.rx_lines)
                t0 = rig.clock.t
                rig.mcu.st = st                              # behind the host's back
                rig.run(0.1)
                latch = rig.core.latches['mcu_state_mismatch']
                self.assertEqual(latch['detail'], 'MCU %s without a host lease' % name)
                self.assertLessEqual(latch['since'] - t0, 0.05 + 0.011)       # the next status line
                self.assertIn('STOP', rig.tx_kinds(since))
                self.assertEqual(rig.mcu.st, 1)                                # the bridge's STOP disarmed it
                self.assertEqual(rig.core.snapshot['state'], 'fault')
                self.assertIsNone(rig.core.control.lease)
                self.assertIn('mcu_state_mismatch', rig.core.snapshot['steering']['faults'])
                rig.run(0.3)
                self.assertNotIn('ARM', rig.tx_kinds(since))                   # no automatic re-arm
                rig.arm()                                                      # the operator re-arm clears it
                self.assertEqual(rig.core.latches, {})

    def test_mcu_state_mismatch_not_raised_while_the_host_expects_the_state(self):
        # The same status lines are not a mismatch while an ARM is pending or confirmed, or a STOP is unconfirmed.
        rig = ts.Rig()
        rig.ready()
        rig.arm()                                    # ARM sent and confirmed: st 2 with a lease
        rig.run(0.3)
        rig.mcu.ignore_stop = True                   # st stays 2 while the STOP is pending (stop_unconfirmed path)
        rig.submit({'op': 'stop'})
        rig.run(0.5)                                 # past the t0 + 400 ms latch of the STOP resend grid
        self.assertNotIn('mcu_state_mismatch', rig.core.latches)
        self.assertIn('stop_unconfirmed', rig.core.latches)

    def test_motion_op_of_the_lease_holder(self):
        # Open P3 item 4: op motion is the executor-side source of steering_ineffective (gyro yaw rate, sensor speed).
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        sample = {'op': 'motion', 'owner': 'operator-a', 'lease': reply['lease'], 'yaw_rate_rad_s': 0.1,
                  'speed_m_s': 0.4}
        future = rig.core.submit(sample)
        self.assertTrue(future.done())                                         # answered at once, no queue
        self.assertEqual(future.result(), {'ok': True})
        self.assertEqual(rig.core._motion[1:], (0.1, 0.4))
        self.assertEqual(rig.core._motion[0], rig.clock.t)
        for change, error in ((dict(owner='operator-b'), 'control_not_owned'), (dict(lease='other'), 'control_not_owned'),
                              (dict(yaw_rate_rad_s=float('nan')), 'invalid_motion'),
                              (dict(speed_m_s=20.0), 'invalid_motion'), (dict(yaw_rate_rad_s=True), 'invalid_motion'),
                              (dict(speed_m_s='0.4'), 'invalid_motion'), (dict(yaw_rate_rad_s=None), 'invalid_motion')):
            with self.subTest(change=change):
                self.assertEqual(rig.core.submit(dict(sample, **change)).result(), {'ok': False, 'error': error})
        self.assertIsNotNone(rig.core.control.lease)                           # a refused sample never stops
        self.assertEqual(rig.core.counters['motion_samples'], 1)
        self.assertEqual(rig.core.counters['motion_refused'], 2)
        self.assertEqual(rig.core.counters['motion_invalid'], 5)
        rig.submit({'op': 'stop'})
        self.assertEqual(rig.core.submit(sample).result(), {'ok': False, 'error': 'control_not_owned'})  # no lease
        self.assertIn('invalid_motion', core_mod.ERROR_CODES)

    def test_steering_ineffective_through_the_motion_op(self):
        for yaw_rate, expect in ((0.0, True), (0.4 * math.tan(0.2) / 0.76, False)):
            with self.subTest(yaw_rate=yaw_rate):
                rig = ts.Rig()
                rig.ready()
                reply = rig.arm()
                t0 = rig.clock.t
                while rig.core.control.lease and rig.clock.t - t0 < 2.5:
                    reply = rig.submit(drive(reply, 20, 200))
                    for _ in range(20):
                        rig.core.submit({'op': 'motion', 'owner': 'operator-a', 'lease': reply['lease'],
                                         'yaw_rate_rad_s': yaw_rate, 'speed_m_s': 0.4})
                        rig.tick()
                self.assertEqual('steering_ineffective' in rig.core.latches, expect)
                self.assertEqual(rig.core.control.lease is None, expect)

    def test_mcu_reset_detected_by_a_new_boot_id(self):
        rig, reply = self.armed_driving()
        rig.mcu.reset(rig.clock.t)
        rig.run(0.1)
        self.assertIn('mcu_reset_detected', rig.core.latches)
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'mcu_reset_detected')
        self.assertEqual(rig.link.boot_id, ts.BOOT_IDS[1])
        n0 = len(rig.mcu.rx_lines)
        reply = rig.arm()
        self.assertIn(b'ARM ' + ts.BOOT_IDS[1].encode(), [line.split(b'*')[0] for line in rig.tx(n0)])
        t0 = rig.clock.t
        rig.submit(drive(reply, 10, 0))
        self.assertEqual(c_lines(rig, t0)[0][1], 1)                         # q restarts at 1 on a new boot

    def test_mcu_reset_detected_by_ms_going_back(self):
        rig, reply = self.armed_driving()
        rig.mcu.ms_jump = -100000
        rig.run(0.1)
        self.assertIn('mcu_reset_detected', rig.core.latches)
        self.assertIsNone(rig.core.control.lease)

    def test_mcu_reset_with_a_lost_banner_asks_info(self):
        # The new boot banner merged with a cut line or boot garbage (spec 12.1-61): the host sees the reset in
        # the status lines (ms back), stops, asks INFO and refuses arm until the banner with the new boot id.
        rig, reply = self.armed_driving()
        rig.mcu.boot_banner = rig.mcu.boot_cal_line = False
        rig.mcu.reset(rig.clock.t)
        rig.mcu.boot_banner = rig.mcu.boot_cal_line = True                   # INFO answers come through
        t_reset = rig.clock.t
        n0 = len(rig.mcu.rx_lines)
        replies = []
        while 'mcu_reset_detected' not in rig.core.latches and rig.clock.t - t_reset < 0.2:
            future = rig.core.submit(ARM)                                     # an operator pressing arm
            rig.tick()
            replies.append(future.result())
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.link.boot_id, ts.BOOT_IDS[0])                   # not known yet
        # The arm dispatched in the detection step (after the status line) is refused: no boot id yet.
        self.assertEqual(replies[-1], {'ok': False, 'error': 'mcu_reset_detected'})
        self.assertTrue(all(r == {'ok': False, 'error': 'control_in_use'} for r in replies[:-1]), replies)
        self.assertNotIn('ARM', rig.tx_kinds(n0))
        self.assertTrue(rig.run_until(lambda: rig.link.boot_id == ts.BOOT_IDS[1], 0.1))
        self.assertIn('INFO', rig.tx_kinds(n0))
        self.assertEqual(rig.core.counters['info_after_reset'], 1)
        self.assertLess(rig.clock.t - t_reset, 0.15)
        rig.ready()                                       # a status line of the new boot and the STOP confirmed
        reply = rig.arm()
        self.assertTrue(reply['ok'])
        self.assertEqual(rig.core.latches, {})

    def test_arm_refused_with_n_boot_asks_for_the_banner(self):
        # Spec 12.1-76 (2026-11-09 review): the MCU rebooted after the session banner and before its first status
        # line, and the new banner was lost. The host knows only the old boot id and nothing in the status lines
        # shows the reset (there was no earlier line): ARM gets N - boot. That refusal latches mcu_reset_detected and
        # asks for the banner with INFO; the next arm uses the new boot id. Before, every arm got N - boot again.
        rig = ts.Rig()
        rig.tick()                                                       # port-open reset, banner of boot 0
        self.assertEqual(rig.link.boot_id, ts.BOOT_IDS[0])
        self.assertIsNone(rig.link.status)
        rig.mcu.boot_banner = rig.mcu.boot_cal_line = False
        rig.mcu.reset(rig.clock.t)                                       # boot 1, its banner is lost
        rig.mcu.boot_banner = rig.mcu.boot_cal_line = True
        rig.ready()
        self.assertEqual(rig.link.boot_id, ts.BOOT_IDS[0])               # stale
        n0 = len(rig.mcu.rx_lines)
        self.assertTrue(rig.submit(ARM)['ok'])
        rig.run(0.05)
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'mcu_reset_detected')
        self.assertIn('mcu_reset_detected', rig.core.latches)
        self.assertIn('arm_rejected', rig.core.latches)
        self.assertIn('INFO', rig.tx_kinds(n0))
        self.assertTrue(rig.run_until(lambda: rig.link.boot_id == ts.BOOT_IDS[1], 0.3))
        rig.ready()
        rig.arm()
        self.assertEqual(rig.core.steering_host()['arm_state'], 'confirmed')
        self.assertIn(b'ARM ' + ts.BOOT_IDS[1].encode(), [line.split(b'*')[0] for line in rig.tx(n0)])
        self.assertEqual(rig.core.latches, {})

    def test_mcu_reset_detected_by_status_n_restarting(self):
        rig, reply = self.armed_driving()
        rig.mcu.n = max(0, rig.mcu.n - 3)                                     # n does not follow: a new boot
        self.assertTrue(rig.run_until(lambda: 'mcu_reset_detected' in rig.core.latches, 0.1))
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.counters['status_n_anomaly'], 1)
        self.assertTrue(rig.run_until(lambda: rig.core.counters['info_after_reset'] == 1, 0.05))
        rig.run(0.1)                                                          # same boot: the INFO banner clears
        self.assertEqual(rig.core._banner_wanted, False)
        self.assertTrue(rig.arm()['ok'])

    def test_cal_get_only_when_the_cal_line_is_missing(self):
        rig = ts.Rig()
        rig.ready()
        self.assertNotIn('CAL', rig.tx_kinds())                              # it came with the banner
        rig = ts.Rig()
        rig.mcu.boot_cal_line = False
        rig.run(0.3)
        cal_get = [t for t, label, _ in rig.link.tx_log if label == 'CAL GET']
        banner_t = rig.link.banner_t
        self.assertEqual(len(cal_get), 1)
        self.assertGreaterEqual(cal_get[0] - banner_t, rig.core.CAL_QUERY_GRACE_S - 1e-9)
        self.assertLess(cal_get[0] - banner_t, rig.core.CAL_QUERY_GRACE_S + 0.011)
        self.assertIsNotNone(rig.link.cal)

    def test_too_many_missing_status_lines(self):
        rig, reply = self.armed_driving()
        n = rig.mcu.n
        rig.mcu.drop_status = set(range(n, n + 60, 2))   # every other line: gaps of 100 ms, below the 150 ms limit
        rig.run(0.75)
        self.assertIn('status_seq_gap', rig.core.latches)
        self.assertNotIn('steering_status_stale', rig.core.latches)
        self.assertEqual(rig.core.control.reason, 'status_seq_gap')

    def test_identity_and_calibration_latches(self):
        for mcu, latch in ((dict(cal_id=8), 'calibration_mismatch'), (dict(hw='front_steer_v9'),
                                                                        'hardware_profile_mismatch'),
                           (dict(sim=False), 'sim_identity_mismatch')):
            with self.subTest(latch=latch):
                rig = ts.Rig(**mcu)
                rig.run(0.2)
                self.assertIn(latch, rig.core.latches)
                self.assertEqual(rig.core.snapshot['state'], 'fault')
                self.assertEqual(rig.submit(ARM)['error'], latch)

    def test_sim_flag_rule_in_both_roles(self):
        sim_wrapper = ts.Rig(simulated=True, sim=False)
        sim_wrapper.run(0.5)
        self.assertEqual(sim_wrapper.link.state, 'refused')
        self.assertNotIn('PING', sim_wrapper.tx_kinds(skip=()))
        real_wrapper = ts.Rig(simulated=False, sim=True)
        real_wrapper.run(0.5)
        self.assertEqual(real_wrapper.submit(ARM)['error'], 'sim_identity_mismatch')
        self.assertEqual(real_wrapper.core.snapshot['transport'], 'esp_usb')
        good = ts.Rig(simulated=False, sim=False)
        good.ready()
        good.arm()
        self.assertFalse(good.core.snapshot['simulated'])

    def test_measured_angle_not_following_the_request(self):
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        rig.mcu.stuck_pos = 0
        t0 = rig.clock.t
        while rig.core.control.lease and rig.clock.t - t0 < 3.0:
            reply = rig.submit(drive(reply, 0, 300))
            rig.run(0.2)
        latch = rig.core.latches['steering_not_following']
        self.assertGreaterEqual(latch['since'] - t0, 1.5)
        self.assertLess(latch['since'] - t0, 1.5 + 0.12)
        self.assertEqual(rig.core.control.reason, 'steering_not_following')

    def test_not_following_counts_only_after_the_mcu_ramp_settles(self):
        # amendment 43: a slow MCU ramp (480 permille/s = 12 deg/s at 40 permille/deg) toward 300 mrad (about 688
        # permille) takes about 1.4 s; the 1.5 s count starts when cmd reaches the target, not at the request
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        rig.mcu.stuck_pos = 0
        rig.mcu.cmd_rate_pm_s = 480.0
        t0 = rig.clock.t
        while rig.core.control.lease and rig.clock.t - t0 < 5.0:
            reply = rig.submit(drive(reply, 0, 300))
            rig.run(0.2)
        latch = rig.core.latches['steering_not_following']
        self.assertGreaterEqual(latch['since'] - t0, 1.4 + 1.5)    # the old rule latched at 1.5 .. 1.62 s
        self.assertLess(latch['since'] - t0, 3.6)

    def test_slewing_that_never_settles_does_not_latch(self):
        # a request that reverses every 1.2 s with a ramp needing about 4 s: the error stays large but the MCU is still
        # slewing (its TRACK rule watches that phase), so the host does not latch
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        rig.mcu.stuck_pos = 0
        rig.mcu.cmd_rate_pm_s = 480.0
        t0 = rig.clock.t
        k = 0
        while rig.clock.t - t0 < 6.0:
            target = 400 if (k // 6) % 2 == 0 else -400
            reply = rig.submit(drive(reply, 0, target))
            rig.run(0.2)
            k += 1
        self.assertNotIn('steering_not_following', rig.core.latches)
        self.assertTrue(rig.core.control.lease)

    def test_esp32_adapter_track_rule_counts_without_the_ramp_gate(self):
        # host profile host_faults.track_rule esp32_adapter (vehicle profile steer_monitor_stack esp32_r5): the
        # vehicle's ESP32 stack has no MCU ramp, so |sent target (status cmd) - measured| counts at once. The slow ramp
        # of the test above toward 300 mrad: cmd passes 3 deg (about 120 permille) after about 0.25 s, the latch
        # follows 1.5 s later (the ramp-gated rule: >= 2.9 s); the never-settling reversal now latches too
        faults = dict(ts.host_profile_data()['host_faults'], track_rule='esp32_adapter')
        profile = ts.host_profile_data(host_faults=faults)
        self.assertEqual(core_mod.HostProfile(profile).track_rule, 'esp32_adapter')
        self.assertEqual(core_mod.HostProfile(ts.host_profile_data()).track_rule, 'v3_ramp_gate')
        rig = ts.Rig(profile=profile)
        rig.ready()
        reply = rig.arm()
        rig.mcu.stuck_pos = 0
        rig.mcu.cmd_rate_pm_s = 480.0
        t0 = rig.clock.t
        while rig.core.control.lease and rig.clock.t - t0 < 5.0:
            reply = rig.submit(drive(reply, 0, 300))
            rig.run(0.2)
        latch = rig.core.latches['steering_not_following']
        self.assertGreaterEqual(latch['since'] - t0, 0.2 + 1.5)
        self.assertLess(latch['since'] - t0, 2.2)
        rig = ts.Rig(profile=profile)
        rig.ready()
        reply = rig.arm()
        rig.mcu.stuck_pos = 0
        rig.mcu.cmd_rate_pm_s = 480.0
        t0 = rig.clock.t
        k = 0
        while rig.core.control.lease and rig.clock.t - t0 < 6.0:
            target = 400 if (k // 6) % 2 == 0 else -400
            reply = rig.submit(drive(reply, 0, target))
            rig.run(0.2)
            k += 1
        self.assertIn('steering_not_following', rig.core.latches)
        bad = dict(faults, track_rule='other')
        with self.assertRaises(core_mod.HostProfileError):
            core_mod.HostProfile(ts.host_profile_data(host_faults=bad))
        with self.assertRaises(core_mod.HostProfileError):
            core_mod.HostProfile(ts.host_profile_data(host_faults=dict(faults, surprise=1)))

    def test_steering_ineffective_with_a_motion_source(self):
        for yaw_rate, expect in ((0.0, True), (0.4 * math.tan(0.2) / 0.76, False)):
            with self.subTest(yaw_rate=yaw_rate):
                rig = ts.Rig()
                rig.ready()
                reply = rig.arm()
                t0 = rig.clock.t
                while rig.core.control.lease and rig.clock.t - t0 < 2.5:
                    reply = rig.submit(drive(reply, 20, 200))
                    for _ in range(20):
                        rig.core.observe_motion(yaw_rate, 0.4)
                        rig.tick()
                self.assertEqual('steering_ineffective' in rig.core.latches, expect)
                self.assertEqual(rig.core.control.lease is None, expect)
        with self.assertRaises(ValueError):
            rig.core.observe_motion(float('nan'), 0.4)

    def test_mcu_disarming_itself_latches(self):
        for line, cause in (('F POT_OPEN 3 %d', 'MCU fault POT_OPEN'), ('W %d', 'MCU watchdog')):
            with self.subTest(cause=cause):
                rig, reply = self.armed_driving()
                rig.mcu.st, rig.mcu.L, rig.mcu.R = 1, 0, 0
                rig.mcu.fl |= 0x0002 if line.startswith('F') else 0x1000
                rig.mcu._send(rig.clock.t, line % rig.mcu.ms(rig.clock.t))
                rig.run(0.1)
                self.assertEqual(rig.core.latches['mcu_disarmed']['detail'], cause)
                self.assertIsNone(rig.core.control.lease)
                faults = rig.core.snapshot['steering']['faults']
                self.assertIn('mcu_disarmed', faults)
                self.assertIn('POT_OPEN' if line.startswith('F') else 'WATCHDOG', faults)

    def test_rejected_c_line_latches(self):
        rig, reply = self.armed_driving()
        rig.run(0.06)
        rig.link.next_q = rig.mcu.last_q          # the next C repeats a q: the MCU answers N <q> old
        rig.run(0.1)
        self.assertIn('c_rejected', rig.core.latches)
        self.assertIsNone(rig.core.control.lease)

    def test_write_failure_latches_closes_and_reopens_without_rearm(self):
        rig, reply = self.armed_driving()
        rig.transport.fail_writes = True
        rig.run(0.1)
        self.assertIn('hardware_fault', rig.core.latches)
        self.assertEqual(rig.link.state, 'closed')
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'hardware_fault')
        rig.transport.fail_writes = False
        rig.run(1.2)
        self.assertTrue(rig.link.ready)
        self.assertEqual(rig.link.sessions, 2)
        self.assertIsNone(rig.core.control.lease)
        rig.run_until(lambda: rig.core.steering_host()['stop_state'] == 'confirmed', 0.3)
        rig.arm()
        self.assertEqual(rig.core.latches, {})

    def test_usb_unplug_and_replug(self):
        rig, reply = self.armed_driving()
        rig.transport.device.unplug()
        rig.run(0.05)
        self.assertEqual(rig.link.state, 'closed')
        self.assertIsNone(rig.core.control.lease)
        rig.run(2.0)                                   # reopen attempts fail while unplugged
        self.assertEqual(rig.link.state, 'closed')
        rig.transport.device.replug()
        rig.run(1.1)
        self.assertTrue(rig.link.ready)
        self.assertNotEqual(rig.link.boot_id, ts.BOOT_IDS[0])     # port-open reset gave a new boot
        self.assertIsNone(rig.core.control.lease)

    def test_bad_mcu_lines_are_counted_not_fatal(self):
        rig = ts.Rig()
        rig.ready()
        rig.transport.device.send(b'S 1 2 3*00\nhalf frame without end')
        rig.run(0.1)
        self.assertGreaterEqual(rig.core.counters['rx_bad_lines'], 1)
        self.assertTrue(rig.link.ready)
        self.assertEqual(rig.core.latches, {})

    def test_wall_clock_change_has_no_effect_on_deadlines(self):
        rig, reply = self.armed_driving()
        rig.core.wall_clock = lambda: 0.0
        reply = rig.submit(drive(reply, 30, 0))
        t0 = rig.clock.t
        rig.run_until(lambda: rig.core.control.lease is None, 1.0)
        self.assertAlmostEqual(rig.clock.t - t0, 0.75, delta=0.011)
        self.assertEqual(rig.core.snapshot['updated_at'], 0.0)


class TtlLatchTest(unittest.TestCase):
    """DECISIONS R18 decision 1: an MCU C TTL expiry while the host still had a valid operator input (C lines lost or
    late) latches mcu_ttl_expired and stops at once: no C line until an operator re-arm, so the next C cannot restart
    the drive after the TTL brake. The TTL at the end of the host's own input deadline does not latch."""

    @staticmethod
    def lose_host_lines(rig, start_t, end_t):
        """Host -> MCU bytes written in [start_t, end_t) never reach the fake MCU (the link to the MCU is down)."""
        original = rig.mcu.tick

        def tick(now, _original=original):
            device = rig.mcu.device
            kept = [(w, d) for w, d in device.take_timed() if not start_t - 1e-9 <= w < end_t - 1e-9]
            device.transport._to_device.extend(kept)
            _original(now)
        rig.mcu.tick = tick

    @staticmethod
    def drop_mcu_lines(rig, prefix):
        """MCU lines that start with prefix are lost on the way to the host."""
        original = rig.mcu._send

        def send(now, body, _original=original):
            if body.startswith(prefix):
                rig.mcu.sent_lines.append((now, b'(lost) ' + body.encode('ascii')))
                return
            _original(now, body)
        rig.mcu._send = send

    def driving(self, latency_s=0.0, throttle=30, steer_mrad=100):
        rig = ts.Rig(latency_s=latency_s)
        rig.ready()
        reply = rig.arm()
        reply = rig.submit(drive(reply, throttle, steer_mrad))
        return rig, reply

    def keep_driving(self, rig, reply, until_t, throttle=30, steer_mrad=100, period_s=0.1):
        """A fresh drive every period_s (like the executor) while the lease lasts; returns the last reply."""
        while rig.clock.t < until_t and rig.core.control.lease:
            reply = rig.submit(drive(reply, throttle, steer_mrad))
            if not reply.get('ok'):
                break
            rig.run_until(lambda: rig.clock.t >= until_t or not rig.core.control.lease, period_s - rig.step_s)
        return reply

    def test_lost_c_lines_latch_and_stop_at_the_f_ttl_line(self):
        rig, reply = self.driving()
        t_loss = rig.clock.t + 0.105
        self.lose_host_lines(rig, t_loss, t_loss + 0.4)
        self.keep_driving(rig, reply, t_loss + 0.6)
        latch = rig.core.latches['mcu_ttl_expired']
        f_ttl = [t for t, line in rig.mcu.sent_lines if line.startswith(b'F TTL')]
        self.assertEqual(len(f_ttl), 1)
        self.assertAlmostEqual(latch['since'], f_ttl[0], delta=1e-9)       # read and latched in the same step
        self.assertIn('F TTL line', latch['detail'])
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.control.reason, 'mcu_ttl_expired')
        self.assertIn(latch['since'], tx_times(rig, 'STOP'))                # the host's own STOP in that step
        self.assertEqual(c_lines(rig, latch['since'] - 1e-9), [])          # no C line after the latch
        rig.run(0.5)                                                        # the link is back: a STOP resend lands
        self.assertEqual((rig.mcu.st, rig.mcu.L, rig.mcu.R), (1, 0, 0))
        self.assertEqual(rig.core.snapshot['state'], 'fault')
        self.assertIn('mcu_ttl_expired', rig.core.snapshot['steering']['faults'])
        self.assertIsNone(rig.core.control.lease)                           # no automatic re-arm
        self.assertEqual(c_lines(rig, latch['since'] - 1e-9), [])
        rig.arm()                                                           # operator re-arm clears the latch
        self.assertEqual(rig.core.latches, {})

    def test_c_sent_after_the_expiry_is_stopped_well_inside_the_rest(self):
        # the F TTL line reaches the host 30 ms late; a fresh drive in between sends a C that clears the TTL at the
        # MCU (the real MCU would drive again 120 ms after the TTL brake, spec 8.2): the host still stops on the F line
        rig, reply = self.driving(latency_s=0.03)
        rig.run_until(lambda: rig.clock.t > c_lines(rig)[-1][0] + 1e-9, 0.1)   # just after a C write
        t_c = c_lines(rig)[-1][0]
        self.lose_host_lines(rig, t_c + 0.005, t_c + 0.32)
        reply = self.keep_driving(rig, reply, t_c + 0.3)
        rig.run_until(lambda: rig.mcu.ttl_count >= 1, 0.2)
        e = [t for t, line in rig.mcu.sent_lines if line.startswith(b'F TTL')][0]
        rig.run_until(lambda: rig.clock.t >= e + 0.009, 0.1)
        self.assertIsNotNone(rig.core.control.lease)                        # the F line is not read yet
        reply = rig.submit(drive(reply, 30, 100))                           # a C after the expiry (link back)
        self.assertTrue(reply['ok'])
        first_c_after = [c for c in c_lines(rig, e)][0][0]
        rig.run(0.05)
        latch = rig.core.latches['mcu_ttl_expired']
        stop_t = [t for t in tx_times(rig, 'STOP') if t >= first_c_after][0]
        self.assertLessEqual(stop_t - first_c_after, 0.03 + 0.011)           # far inside the 120 ms rest
        self.assertAlmostEqual(latch['since'], stop_t, delta=1e-9)
        self.assertEqual(c_lines(rig, stop_t), [])

    def test_ttl_bit_in_the_status_lines_latches_when_the_f_line_is_lost(self):
        rig, reply = self.driving()
        self.drop_mcu_lines(rig, 'F TTL')
        t_loss = rig.clock.t + 0.105
        self.lose_host_lines(rig, t_loss, t_loss + 0.5)
        self.keep_driving(rig, reply, t_loss + 0.6)
        latch = rig.core.latches['mcu_ttl_expired']
        self.assertIn('status line TTL bit', latch['detail'])
        self.assertEqual(rig.core.counters['mcu_fault_TTL'], 0)
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(c_lines(rig, latch['since'] - 1e-9), [])

    def test_ttl_at_the_end_of_the_input_deadline_does_not_latch(self):
        # throttle 0 at standstill: the lease is kept past the input deadline (no C resend), the MCU C TTL ends the
        # command (HOST_BRIDGE_V3_KO.md 2, spec 6.3): no latch, the lease stays
        rig, reply = self.driving(throttle=0)
        rig.run(1.2)
        self.assertGreaterEqual(rig.core.counters['mcu_fault_TTL'], 1)
        self.assertGreaterEqual(rig.core.counters['ttl_expected'], 1)
        self.assertNotIn('mcu_ttl_expired', rig.core.latches)
        self.assertIsNotNone(rig.core.control.lease)
        reply = rig.submit(drive(reply, 30, 100))                            # the operator drives on: no re-arm
        self.assertTrue(reply['ok'])
        rig.run(0.2)
        self.assertNotIn('mcu_ttl_expired', rig.core.latches)
        self.assertGreater(len(c_lines(rig, rig.clock.t - 0.2)), 2)

    def test_a_fresh_request_at_the_deadline_expiry_does_not_latch(self):
        # the operator drives on in the very step of the deadline expiry (F line read in that step, or 30 ms late):
        # the request came after the expiry, so the expiry was the host's own deadline
        for latency_s in (0.0, 0.03):
            with self.subTest(latency_s=latency_s):
                rig, reply = self.driving(latency_s=latency_s, throttle=0)
                mcu = rig.mcu

                def ttl_in_the_next_step():
                    pending = any(d.startswith(b'C ') for _, d in rig.transport._to_device)
                    return (not pending and mcu.c_t is not None          # the fake MCU's own TTL expression
                            and round(rig.clock.t + rig.step_s, 9) - mcu.c_t >= mcu.c_ttl / 1000.0)
                self.assertTrue(rig.run_until(ttl_in_the_next_step, 1.0))
                reply = rig.submit(drive(reply, 30, 100))                     # MCU TTL fires in this step
                self.assertTrue(reply['ok'])
                self.assertEqual(mcu.ttl_count, 1)
                rig.run(0.1)
                self.assertNotIn('mcu_ttl_expired', rig.core.latches)
                self.assertIsNotNone(rig.core.control.lease)
                self.assertGreaterEqual(rig.core.counters['ttl_expected'], 1)

    def test_no_lease_no_latch(self):
        rig, reply = self.driving()
        rig.submit({'op': 'stop'})
        rig.run(0.2)
        rig.mcu.st = 2                                     # a stray TTL line of a device the host does not drive
        rig.mcu._send(rig.clock.t, 'F TTL 9 %d' % rig.mcu.ms(rig.clock.t))
        rig.run(0.05)
        self.assertNotIn('mcu_ttl_expired', rig.core.latches)


class RobustnessTest(unittest.TestCase):
    def test_bad_ops_and_defects_never_leave_step(self):
        # 2026-11-09 review: a lease owner sending op as a list ended the wrapper loop (TypeError out of step()).
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        for op in (['drive'], {'x': 1}, 5):
            request = dict(drive(reply, 0, 0), op=op)
            self.assertEqual(rig.submit(request), {'ok': False, 'error': 'unknown_operation'})
        self.assertIsNotNone(rig.core.control.lease)        # refused like RoverControl's unknown op, not a stop
        # A defect inside a request: internal_error latch, stop, an answer; the loop goes on.

        def broken(request):
            raise ZeroDivisionError('test defect')
        rig.core.control.dispatch = broken
        self.assertEqual(rig.submit({'op': 'keepalive', 'owner': 'operator-a'}),
                         {'ok': False, 'error': 'internal_error'})
        del rig.core.control.dispatch
        self.assertIn('internal_error', rig.core.latches)
        self.assertIsNone(rig.core.control.lease)
        self.assertEqual(rig.core.snapshot['state'], 'fault')
        # A defect inside a step: the same, and step() returns.
        original = rig.core._drive_link

        def broken_step(now):
            raise KeyError('test defect')
        rig.core._drive_link = broken_step
        rig.tick()
        rig.core._drive_link = original
        self.assertEqual(rig.core.counters['internal_errors'], 2)
        # An answer for a request the socket thread cancelled meanwhile (0.4 s timeout).
        box = []

        def cancelling(request):
            box[0].cancel()
            return {'ok': True}
        rig.core.control.dispatch = cancelling
        box.append(rig.core.submit({'op': 'keepalive', 'owner': 'x'}))
        rig.tick()
        del rig.core.control.dispatch
        self.assertTrue(box[0].cancelled())
        self.assertEqual(rig.core.counters['answers_after_cancel'], 1)
        rig.run(0.3)
        self.assertTrue(rig.link.ready)


class StatusTest(unittest.TestCase):
    def test_steering_block_follows_design_4_2(self):
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        rig.submit(drive(reply, 40, -120))
        rig.run(0.2)
        snap = rig.core.snapshot
        json.dumps(snap, allow_nan=False)
        self.assertEqual(tuple(snap['steering']), core_mod.STEERING_BLOCK_KEYS)
        self.assertEqual({k: snap[k] for k in ('firmware', 'protocol', 'boot_id', 'device_id', 'reset_reason',
                                               'hw_profile', 'simulated', 'profile', 'cal_id')},
                         dict(firmware='UBTS_STEER_V3', protocol=3, boot_id=ts.BOOT_IDS[0], device_id='5A1B2C',
                              reset_reason='power_on', hw_profile='front_steer_v0', simulated=True,
                              profile='front_steer', cal_id=7))
        self.assertEqual(snap['requested'], {'throttle': 40, 'steer_mrad': -120, 'left': 40, 'right': 40})
        block = snap['steering']
        self.assertEqual((block['measured_mrad'], block['measured_permille'], block['target_permille']),
                         (-120, -274, -274))
        self.assertEqual((block['drive_en'], block['steer_en'], block['calibrated'], block['saturated']),
                         (True, True, True, False))
        self.assertIn(block['servo_state'], ('off', 'hold', 'slewing', 'tracking', 'settled'))
        self.assertIsInstance(block['age_ms'], int)
        self.assertLessEqual(block['age_ms'], 50)
        self.assertIsNone(block['stop_mode'])
        self.assertEqual(snap['outputs'], {'D8': 102, 'D5': 0, 'D3': 102, 'D0': 0, 'D1': 255, 'D2': 255, 'D6': 0,
                                           'D7': 0})
        self.assertEqual(snap['transport'], 'simulation')
        applied = rig.core.applied()
        self.assertEqual((applied['drive_l'], applied['drive_r'], applied['steer_en']), (102, 102, True))
        state = rig.core.steering_state()
        json.dumps(state, allow_nan=False)
        self.assertEqual((state['schema'], state['angle_mrad'], state['target_mrad'], state['source_kind']),
                         ('ubts.steer_state.v1', -120, -120, 'simulation'))

    def test_pinned_motor_pwm_records_the_drive_duty(self):
        from dashboard_server import motor_pwm
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        rig.submit(drive(reply, 40, 100))
        rig.run(0.1)
        snap = rig.core.snapshot
        sample = motor_pwm.sanitize_status(json.loads(json.dumps(snap)), received_at=snap['updated_at'] + 0.1)
        self.assertEqual(sample['requested'], {'left': 40, 'right': 40})
        self.assertEqual(sample['pins']['D8']['requested_pwm_derived'], 102)
        self.assertEqual(sample['pins']['D3']['requested_pwm_derived'], 102)
        self.assertEqual(sample['pins']['D8']['simulated_output_pwm'], 102)
        self.assertEqual((sample['source_kind'], sample['status']['state'], sample['status']['simulated']),
                         ('simulation', 'armed_manual', True))
        self.assertIsNone(sample['status']['firmware'])     # V3 is not in the pinned allow list (design 4.5)

    def test_hold_estimate_after_stop(self):
        rig = ts.Rig()
        rig.ready()
        reply = rig.arm()
        rig.submit(drive(reply, 0, 100))
        rig.run(0.2)
        rig.core.submit({'op': 'stop'})
        rig.run(0.11)
        hold = rig.core.snapshot['steering']['hold_remaining_ms']
        self.assertTrue(1800 <= hold <= 2000, hold)
        self.assertEqual(rig.core.snapshot['steering']['servo_state'], 'hold')
        rig.run(2.0)
        self.assertIsNone(rig.core.snapshot['steering']['hold_remaining_ms'])
        self.assertEqual(rig.core.snapshot['steering']['servo_state'], 'off')

    def test_error_code_list_contains_the_design_codes(self):
        design = {'firmware_v3_required', 'steering_uncalibrated', 'calibration_mismatch', 'hardware_profile_mismatch',
                  'polarity_check_missing', 'steer_profile_required', 'differential_command_rejected',
                  'steering_fault_latched', 'steering_status_stale', 'stop_unconfirmed'}
        self.assertLessEqual(design, set(core_mod.ERROR_CODES))
        self.assertEqual(len(core_mod.LATCH_NAMES), len(set(core_mod.LATCH_NAMES)))


class HostProfileTest(unittest.TestCase):
    def test_validation(self):
        hp = core_mod.HostProfile(ts.host_profile_data())
        self.assertEqual((hp.permille_from_mrad(-120), hp.permille_from_mrad(409), hp.mrad_from_permille(-274)),
                         (-274, 934, -120))
        self.assertEqual(core_mod.duty_from_throttle(-40), -102)
        bad = [dict(schema='x'), dict(vehicle_profile='differential'), dict(extra=1),
               dict(link=dict(ts.host_profile_data()['link'], status_hz=0)),
               dict(drive={'lr_mode': 'electronic_diff'}),
               dict(steer_limit=dict(ts.host_profile_data()['steer_limit'], left_mrad=410)),
               dict(calibration=dict(ts.host_profile_data()['calibration'], cal_id=0)),
               dict(calibration=dict(ts.host_profile_data()['calibration'], mark_left_mrad=300.0)),
               # timing relations (2026-11-09 review): each value alone is in range
               dict(link=dict(ts.host_profile_data()['link'], ping_hz=1.0)),        # PING 1 s, watchdog 500 ms
               dict(link=dict(ts.host_profile_data()['link'], c_resend_hz=1.0)),    # C every 1 s, ttl 300 ms
               dict(host_faults=dict(ts.host_profile_data()['host_faults'], status_gap_ms=40.0)),
               dict(arm={'confirm_ms': 10.0})]
        for change in bad:
            with self.subTest(change=sorted(change)):
                with self.assertRaises(core_mod.HostProfileError):
                    core_mod.HostProfile(ts.host_profile_data(**change))
        with self.assertRaises(core_mod.HostProfileError):
            core_mod.HostProfile.from_bytes(b'{"schema": NaN}')
        raw = json.dumps(ts.host_profile_data()).encode('utf-8')
        self.assertEqual(len(core_mod.HostProfile.from_bytes(raw).sha256), 64)
        edge = ts.host_profile_data(link=dict(ts.host_profile_data()['link'], watchdog_ms=200))  # PING 100 ms = wd / 2
        self.assertEqual(core_mod.HostProfile(edge).watchdog_ms, 200)


if __name__ == '__main__':
    unittest.main()
