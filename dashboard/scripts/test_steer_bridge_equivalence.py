"""Equivalence with the fixed V2 Bridge (design 2.4 and 4.2, P3): the same request sequences (normal, expired, busy,
STOP priority, plus input expiry, replay, sequence and owner errors) go to the V2 Bridge with a fake V2 motor and to
the steer bridge core with the fake V3 MCU. Responses, stop reasons and the motor call order must be equal for the
common ops once the steering fields are removed.

Linux only: scripts/rover_control_bridge.py imports scripts.esp_motor, which imports fcntl and termios at module
level. The V2 Bridge is used as it is (not modified); only its blocking request() wait is replaced by putting the
same (time, request, future) tuple into its queue, which is the body of Bridge.request without the 0.4 s wait.
"""
import concurrent.futures
import queue
import sys
import time
import unittest

try:
    import test_steer_support as ts      # unittest discover -s dashboard/scripts
except ImportError:                     # pytest imports these modules as scripts.test_steer_*
    from scripts import test_steer_support as ts

LINUX = sys.platform.startswith('linux')
STATUS_KEYS = ('ok', 'state', 'connected', 'armed', 'moving', 'reason', 'input_ttl_s', 'automatic_input_enabled',
               'last_error')
TICK_S = 0.02


class V2Motor:
    """The fake motor of tests/test_esp_rover.py, with the attributes Bridge.step and the snapshot read."""
    ready = True
    firmware = 'UBTS_WHEEL_PWM_V2'
    error = None
    outputs = {}
    stopped = True
    port, fd, identity = '/fake', 1, 'test'

    def __init__(self):
        self.calls = []

    def hard_stop(self):
        self.calls.append('STOP')
        self.stopped = True

    def enable(self):
        self.calls.append('enable')

    def drive(self, left, right):
        self.calls.append((left, right))
        self.stopped = not (left or right)

    def release(self):
        self.calls.append('release')
        self.stopped = True        # the fake V3 MCU applies a zero target at once as well

    def poll(self):
        pass


def normalise(reply):
    """Common-op view of a reply: steering-only fields removed, tokens replaced by their presence."""
    out = {key: reply.get(key) for key in ('ok', 'error', 'seq', 'ticket_ttl_s', 'state')}
    out['lease'] = bool(reply.get('lease'))
    out['ticket'] = bool(reply.get('ticket'))
    status = reply.get('status')
    if isinstance(status, dict):
        out['status'] = {key: status.get(key) for key in STATUS_KEYS}
        out['status']['requested'] = {side: status['requested'][side] for side in ('left', 'right')}
    return out


def v3_calls(action_log):
    out = []
    for item in action_log:
        if isinstance(item, tuple):
            out.append((item[1], item[1]))       # equal duty: drive(throttle) is drive(left=right=throttle)
        else:
            out.append(item)
    return out


@unittest.skipUnless(LINUX, 'the V2 Bridge imports fcntl and termios (Linux only)')
class EquivalenceTest(unittest.TestCase):
    def setUp(self):
        from scripts.rover_control_bridge import Bridge
        self.t = ts.Clock(500.0)
        self.v2_motor = V2Motor()
        self.v2 = Bridge(self.v2_motor)
        self.v2.control.clock = self.t            # RoverControl takes its clock from this attribute
        self.rig = ts.Rig(step_s=TICK_S, clock=self.t)       # one clock for both bridges
        self.rig.ready()
        self.rig.core.action_log.clear()     # the session-start STOP of the V3 link is not a request
        self.v2.step()
        self.v2_replies = {}
        self.v3_replies = {}

    # -- driving both bridges
    def v2_submit(self, request, received=None):
        if request.get('op') in ('status', 'stop'):
            future = concurrent.futures.Future()
            future.set_result(self.v2.request(request))
            return future
        future = concurrent.futures.Future()
        try:
            self.v2.pending.put_nowait((time.monotonic() if received is None else received, request, future))
        except queue.Full:
            future.cancel()
            self.v2.emergency.set()
            done = concurrent.futures.Future()
            done.set_result({'ok': False, 'error': 'control_busy_stopped'})
            return done
        return future

    def v3_submit(self, request, received=None):
        future = self.rig.core.submit(request)
        if received is not None and not future.done():
            # replace the queued time stamp like the V2 case (the newest queue entry is this request)
            items = list(self.rig.core.pending.queue)
            items[-1] = (received, items[-1][1], items[-1][2])
            self.rig.core.pending.queue.clear()
            self.rig.core.pending.queue.extend(items)
        return future

    def tick(self, n=1):
        for _ in range(n):
            self.t.t = round(self.t.t + TICK_S, 9)
            self.v2.step()
            self.rig.mcu.tick(self.t.t)
            self.rig.core.step()
            self.compare_state()

    def compare_state(self):
        a, b = self.v2.snapshot, self.rig.core.snapshot
        for key in ('armed', 'reason'):
            self.assertEqual(a.get(key), b.get(key), key)
        self.assertEqual(a['requested'], {side: b['requested'][side] for side in ('left', 'right')})

    def both(self, name, op, v2_extra=None, v3_extra=None, received_age=None, ticks=1, check=True):
        """Send one request to both bridges (built from each side's last reply), tick, compare the replies."""
        def build(replies, extra):
            last = replies.get('last')
            request = {'op': op, 'owner': 'operator-a'}
            if last is not None and last.get('lease'):
                request.update(lease=last['lease'], ticket=last['ticket'], seq=last['seq'] + 1)
            request.update(extra or {})
            return request
        v2_request = build(self.v2_replies, v2_extra)
        v3_request = build(self.v3_replies, v3_extra)
        v2_future = self.v2_submit(v2_request, None if received_age is None else time.monotonic() - received_age)
        v3_future = self.v3_submit(v3_request, None if received_age is None else self.t.t - received_age)
        self.tick(ticks)
        a, b = v2_future.result(timeout=0), v3_future.result(timeout=0)
        if a.get('ticket'):
            self.v2_replies['last'] = a
        if b.get('ticket'):
            self.v3_replies['last'] = b
        if check:
            self.assertEqual(normalise(a), normalise(b), name)
        return a, b

    def assertCallsEqual(self):
        self.assertEqual(self.v2_motor.calls, v3_calls(self.rig.core.action_log))

    # -- sequences
    def test_normal_sequence(self):
        self.both('arm', 'arm', v3_extra={'vehicle_profile': 'front_steer'})
        self.tick(5)
        for throttle in (30, 30, 45, 0, 40):
            self.both('drive %d' % throttle, 'drive', {'deadman': True, 'left': throttle, 'right': throttle},
                      {'deadman': True, 'throttle': throttle, 'steer_mrad': 0})
            self.tick(3)
        self.both('keepalive', 'keepalive')
        self.both('release', 'release')
        self.tick(5)
        self.assertEqual(normalise(self.v2.request({'op': 'status'})), normalise(self.rig.core.request({'op': 'status'})))
        self.tick(80)
        self.assertCallsEqual()
        self.assertEqual(self.v2_motor.calls, ['STOP', 'enable', (30, 30), (30, 30), (45, 45), (0, 0), (40, 40),
                                               'release'])
        self.assertTrue(self.v2.control.lease and self.rig.core.control.lease)

    def test_expired_request(self):
        self.both('arm', 'arm', v3_extra={'vehicle_profile': 'front_steer'})
        self.tick(5)
        a, b = self.both('late drive', 'drive', {'deadman': True, 'left': 20, 'right': 20},
                         {'deadman': True, 'throttle': 20, 'steer_mrad': 0}, received_age=0.3)
        self.assertEqual(a, {'ok': False, 'error': 'request_expired'})
        self.assertTrue(self.v2.control.lease and self.rig.core.control.lease)
        self.assertCallsEqual()

    def test_busy_queue(self):
        self.both('arm', 'arm', v3_extra={'vehicle_profile': 'front_steer'})
        self.tick(5)
        a_last, b_last = self.v2_replies['last'], self.v3_replies['last']
        keep = lambda last: {'op': 'keepalive', 'owner': 'operator-a', 'lease': last['lease'],  # noqa: E731
                             'ticket': last['ticket'], 'seq': last['seq'] + 1}
        a_futures = [self.v2_submit(keep(a_last)) for _ in range(8)]
        b_futures = [self.v3_submit(keep(b_last)) for _ in range(8)]
        self.assertEqual(self.v2.request(keep(a_last)), self.rig.core.request(keep(b_last)))
        self.assertTrue(self.v2.emergency.is_set() and self.rig.core.emergency.is_set())
        self.tick()
        self.assertEqual([f.result() for f in a_futures], [f.result() for f in b_futures])
        self.assertEqual(a_futures[0].result(), {'ok': False, 'error': 'stop_barrier'})
        self.assertEqual(self.v2.control.reason, 'operator_stop')
        self.assertCallsEqual()

    def test_stop_priority(self):
        self.both('arm', 'arm', v3_extra={'vehicle_profile': 'front_steer'})
        self.tick(5)
        a_future = self.v2_submit({'op': 'drive', 'owner': 'operator-a', 'lease': self.v2_replies['last']['lease'],
                                   'ticket': self.v2_replies['last']['ticket'], 'seq': 1, 'deadman': True,
                                   'left': 60, 'right': 60})
        b_future = self.v3_submit({'op': 'drive', 'owner': 'operator-a', 'lease': self.v3_replies['last']['lease'],
                                   'ticket': self.v3_replies['last']['ticket'], 'seq': 1, 'deadman': True,
                                   'throttle': 60, 'steer_mrad': 0})
        self.assertEqual(self.v2.request({'op': 'stop'}), self.rig.core.request({'op': 'stop'}))
        self.tick()
        self.assertEqual(a_future.result(), b_future.result())
        self.assertEqual(a_future.result()['error'], 'stop_barrier')
        self.assertCallsEqual()
        self.assertNotIn((60, 60), self.v2_motor.calls)

    def test_input_expiry_replay_sequence_and_owner(self):
        self.both('arm', 'arm', v3_extra={'vehicle_profile': 'front_steer'})
        self.tick(5)
        self.both('drive', 'drive', {'deadman': True, 'left': 30, 'right': 30},
                  {'deadman': True, 'throttle': 30, 'steer_mrad': 0})
        self.tick(40)
        self.assertEqual(self.v2.control.reason, 'input_expired')
        self.assertCallsEqual()
        self.v2_replies.clear()
        self.v3_replies.clear()
        self.tick(5)                         # V3: arm needs the STOP confirmed by a DISARMED status line first
        self.both('re-arm', 'arm', v3_extra={'vehicle_profile': 'front_steer'})
        self.tick(5)
        replay_a, replay_b = self.v2_replies['last'], self.v3_replies['last']
        self.both('drive', 'drive', {'deadman': True, 'left': 10, 'right': 10},
                  {'deadman': True, 'throttle': 10, 'steer_mrad': 0})
        a = self.v2_submit({'op': 'drive', 'owner': 'operator-a', 'lease': replay_a['lease'],
                            'ticket': replay_a['ticket'], 'seq': 1, 'deadman': True, 'left': 10, 'right': 10})
        b = self.v3_submit({'op': 'drive', 'owner': 'operator-a', 'lease': replay_b['lease'],
                            'ticket': replay_b['ticket'], 'seq': 1, 'deadman': True, 'throttle': 10, 'steer_mrad': 0})
        self.tick()
        self.assertEqual(a.result(), b.result())
        self.assertEqual(a.result()['error'], 'stale_input_rearm_required')
        for extra_a, extra_b, error in (({'seq': 9}, {'seq': 9}, 'invalid_sequence'),
                                        ({'owner': 'operator-b'}, {'owner': 'operator-b'}, 'control_not_owned')):
            self.v2_replies.clear()
            self.v3_replies.clear()
            self.tick(5)
            self.both('arm', 'arm', v3_extra={'vehicle_profile': 'front_steer'})
            self.tick(5)
            a, b = self.both(error, 'drive', dict({'deadman': True, 'left': 5, 'right': 5}, **extra_a),
                             dict({'deadman': True, 'throttle': 5, 'steer_mrad': 0}, **extra_b))
            self.assertEqual(a['error'], error)
        self.assertCallsEqual()


if __name__ == '__main__':
    unittest.main()
