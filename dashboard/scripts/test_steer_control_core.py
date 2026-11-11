"""SteerControl rules (design 4.2) with a recording fake motor adapter. No hardware, no link."""
import unittest

try:
    import test_steer_support as ts  # noqa: F401  (unittest discover -s dashboard/scripts; sets the import path)
except ImportError:                     # pytest imports these modules as scripts.test_steer_*
    from scripts import test_steer_support as ts  # noqa: F401
from scripts.rover_control_core import ControlError, RoverControl
from scripts.steer_control_core import VEHICLE_PROFILE, SteerControl


class Motor:
    ready = True
    firmware = 'UBTS_STEER_V3'
    error = None
    outputs = {}
    stopped = True

    def __init__(self):
        self.calls = []
        self.blocker = None

    def hard_stop(self, for_arm=False):
        self.calls.append('STOP(arm)' if for_arm else 'STOP')
        self.stopped = True

    def enable(self):
        self.calls.append('enable')

    def release(self):
        self.calls.append('release')

    def drive_steer(self, throttle, steer):
        self.calls.append(('drive', throttle, steer))
        self.stopped = False

    def arm_blocker(self):
        return self.blocker

    def clear_latches(self):
        self.calls.append('clear_latches')


class SteerControlTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.motor = Motor()
        self.core = SteerControl(self.motor, lambda: self.now, steer_limit_left_mrad=409, steer_limit_right_mrad=380,
                                 status_extra=lambda: {'steering': {'marker': 1}})
        self.reply = self.core.dispatch({'op': 'arm', 'owner': 'operator-a', 'vehicle_profile': VEHICLE_PROFILE})

    def request(self, op, owner='operator-a', **extra):
        return {'op': op, 'owner': owner, 'lease': self.reply['lease'], 'ticket': self.reply['ticket'],
                'seq': self.reply['seq'] + 1, **extra}

    def send(self, op, **extra):
        self.reply = self.core.dispatch(self.request(op, **extra))
        return self.reply

    def test_is_a_rover_control(self):
        self.assertIsInstance(self.core, RoverControl)
        self.assertEqual((self.core.INPUT_TTL, self.core.TICKET_TTL, self.core.OWNER_TTL), (0.75, 1.5, 30.0))

    def test_arm_rules(self):
        self.assertEqual(self.motor.calls, ['clear_latches', 'STOP(arm)', 'enable'])
        for request, code in (({'op': 'arm', 'owner': 'b'}, 'steer_profile_required'),
                              ({'op': 'arm', 'owner': 'b', 'vehicle_profile': 'differential'}, 'steer_profile_required'),
                              ({'op': 'arm', 'vehicle_profile': VEHICLE_PROFILE}, 'operator_required'),
                              ({'op': 'arm', 'owner': 'b', 'vehicle_profile': VEHICLE_PROFILE}, 'control_in_use')):
            with self.assertRaisesRegex(ControlError, code):
                self.core.dispatch(request)
        self.core.stop()
        self.motor.blocker = 'steering_uncalibrated'
        with self.assertRaisesRegex(ControlError, 'steering_uncalibrated'):
            self.core.dispatch({'op': 'arm', 'owner': 'b', 'vehicle_profile': VEHICLE_PROFILE})
        self.assertIsNone(self.core.lease)

    def test_drive_sets_the_request(self):
        reply = self.send('drive', deadman=True, throttle=40, steer_mrad=-120)
        self.assertEqual(self.motor.calls[-1], ('drive', 40, -120))
        self.assertEqual(reply['status']['requested'], {'throttle': 40, 'steer_mrad': -120, 'left': 40, 'right': 40})
        self.assertEqual(reply['status']['steering'], {'marker': 1})
        self.assertEqual(reply['status']['vehicle_profile'], 'front_steer')
        self.assertEqual(reply['status']['steer_limit_mrad'], {'left': 409, 'right': 380})
        self.assertTrue(self.core.moving)
        self.assertAlmostEqual(self.core.input_until, 100.75)

    def test_steer_limits_and_types(self):
        for throttle, steer in ((0, 409), (0, -380), (100, 0), (-100, 0)):
            self.send('drive', deadman=True, throttle=throttle, steer_mrad=steer)
        for throttle, steer in ((0, 410), (0, -381), (101, 0), (True, 0), (0, False), (0.0, 0), (0, 1.0), ('1', 0),
                                (None, 0), (0, None)):
            with self.subTest(throttle=throttle, steer=steer):
                self.setUp()
                with self.assertRaisesRegex(ControlError, 'invalid_drive'):
                    self.send('drive', deadman=True, throttle=throttle, steer_mrad=steer)
                self.assertIsNone(self.core.lease)
                self.assertEqual(self.core.reason, 'invalid_drive')
        for deadman in (False, None, 'yes', 1):
            self.setUp()
            with self.assertRaisesRegex(ControlError, 'invalid_drive'):
                self.send('drive', deadman=deadman, throttle=0, steer_mrad=0)

    def test_differential_format_is_rejected_and_stops(self):
        for extra in ({'left': 30, 'right': 30}, {'left': 30}, {'right': 0, 'throttle': 0, 'steer_mrad': 0}):
            with self.subTest(extra=extra):
                self.setUp()
                with self.assertRaisesRegex(ControlError, 'differential_command_rejected'):
                    self.core.dispatch({'op': 'drive', 'owner': 'someone-else', 'deadman': True, **extra})
                self.assertIsNone(self.core.lease)
                self.assertEqual(self.motor.calls[-1], 'STOP')

    def test_steer_only_needs_ownership(self):
        with self.assertRaisesRegex(ControlError, 'control_not_owned'):
            self.core.dispatch(self.request('drive', owner='operator-b', deadman=True, throttle=0, steer_mrad=100))
        stale = self.request('drive', deadman=True, throttle=0, steer_mrad=100)
        self.send('drive', deadman=True, throttle=0, steer_mrad=50)
        with self.assertRaisesRegex(ControlError, 'stale_input_rearm_required'):
            self.core.dispatch(stale)
        self.assertIsNone(self.core.lease)
        self.setUp()
        with self.assertRaisesRegex(ControlError, 'invalid_sequence'):
            self.core.dispatch(dict(self.request('drive', deadman=True, throttle=0, steer_mrad=1), seq=5))

    def test_steer_only_does_not_expire_the_lease(self):
        self.send('drive', deadman=True, throttle=0, steer_mrad=200)
        self.assertFalse(self.core.moving)
        self.now += 5.0
        self.core.tick()
        self.assertIsNotNone(self.core.lease)
        self.assertEqual(self.core.steer_mrad, 200)

    def test_keepalive_does_not_extend_the_motion_deadline(self):
        self.send('drive', deadman=True, throttle=30, steer_mrad=0)
        self.now += 0.6
        self.send('keepalive')
        self.now += 0.16
        self.core.tick()
        self.assertIsNone(self.core.lease)
        self.assertEqual(self.core.reason, 'input_expired')
        self.assertEqual(self.motor.calls[-1], 'STOP')

    def test_release_keeps_steering_and_has_the_completion_bound(self):
        self.send('drive', deadman=True, throttle=60, steer_mrad=150)
        reply = self.send('release')
        self.assertEqual(self.motor.calls[-1], 'release')
        self.assertEqual(reply['status']['requested'], {'throttle': 0, 'steer_mrad': 150, 'left': 0, 'right': 0})
        self.assertTrue(self.core.moving)
        self.now += 1.11
        self.core.tick()
        self.assertIsNone(self.core.lease)           # still not stopped after 1.1 s -> stop
        self.setUp()
        self.send('drive', deadman=True, throttle=60, steer_mrad=150)
        self.send('release')
        self.motor.stopped = True
        self.now += 0.5
        self.core.tick()
        self.assertFalse(self.core.moving)
        self.assertIsNotNone(self.core.lease)

    def test_stop_and_no_auto_rearm(self):
        self.send('drive', deadman=True, throttle=20, steer_mrad=10)
        self.core.stop('operator_stop')
        self.assertEqual((self.core.throttle, self.core.steer_mrad, self.core.requested),
                         (0, None, {'left': 0, 'right': 0}))
        for _ in range(10):
            self.now += 1.0
            self.core.tick()
        self.assertIsNone(self.core.lease)
        self.assertNotIn('enable', self.motor.calls[3:])

    def test_no_pivot_or_unknown_ops(self):
        for op in ('pivot', 'turn', 'differential'):
            self.setUp()
            with self.assertRaisesRegex(ControlError, 'unknown_operation'):
                self.send(op, deadman=True, throttle=0, steer_mrad=0)

    def test_limits_must_be_positive_ints(self):
        for left, right in ((0, 1), (1, 0), (1.0, 1), (True, 1), (2000, 1)):
            with self.assertRaises(ValueError):
                SteerControl(Motor(), steer_limit_left_mrad=left, steer_limit_right_mrad=right)


if __name__ == '__main__':
    unittest.main()
