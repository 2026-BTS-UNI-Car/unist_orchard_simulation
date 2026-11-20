"""Esp32AbsHoldAdapter, esp32_arm_precheck and the ESP32 configuration rules (docs/steering/EXECUTOR_ESP32_KO.md) with a
hand-written ESP32 bridge status (the fields of the dashboard's scripts/esp32_abs_hold.Esp32SteerMotor.status_extra).
No dashboard, socket or ROS. Test numbers only."""
import math
import os
import unittest
from dataclasses import replace

import rowexec_testlib as L  # noqa: F401  (path setup)
from row_executor import config as C
from row_executor.bridge_client import (ESP32_INEFFECTIVE, ESP32_POLICY, ESP32_SMOOTH_POLICY, ESP32_TRACK,
                                        Esp32AbsHoldAdapter, Esp32SteerBridgeClient, LATCHED, esp32_arm_precheck,
                                        esp32_cal_accepted,
                                        esp32_smooth_hold, steer_arm_precheck)

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAL = dict(right=1681, center=1900, left=2180, max_command=20)
MARK = 17.831
SCALE = 20. / MARK


def esp_status(cmd=0., pot=1900, target_pot=None, hold=False, armed=False, faults=(), latches=(), age_ms=40,
               stop_state='confirmed', direction_verified=True, watchdog=None, target_cmd=None, **over):
    """An ESP32 bridge status (esp32_steer_bridge snapshot fields)."""
    status = dict(ok=True, connected=True, state='armed_manual' if armed else 'disarmed', armed=armed, moving=False,
                  firmware='ABS-HOLD-8-REBOUND-LIMITS', esp_kind='esp32_abs_hold', vehicle_profile='front_steer',
                  steer_limit_mrad=dict(left=349, right=349), reason='operator_armed' if armed else 'startup',
                  updated_at=100., outputs=dict(D8=0, D5=0, D3=0, D0=0, D1=0, D2=0),
                  steer_control=dict(kind='esp32_abs_hold', angle_quantum_deg=1.0),
                  steering=dict(measured_mrad=int(round(-math.radians(cmd) * 1000)), command_equiv_deg=cmd, raw=pot,
                                target_pot=target_pot, hold=hold, age_ms=age_ms, calibrated=True, cal=dict(CAL),
                                faults=list(faults)),
                  steering_host=dict(link='ready', stale=False, stop_state=stop_state,
                                     arm_state='armed' if armed else 'disarmed', latches=list(latches),
                                     target_command_deg=target_cmd),
                  drive=dict(direction_verified=direction_verified, firmware_watchdog_ms=watchdog))
    status.update(over)
    return status


def smooth_status(mode='SMOOTH', firmware=True, quantum=.25, **kw):
    """An ESP32 bridge status of the SMOOTH firmware (dashboard da39e8a or later): steer_control with the bridge's
    pacing and steering.smooth (firmware = FW token SMOOTH, mode = the board's SM line)."""
    status = esp_status(steer_control=dict(kind='esp32_abs_hold', angle_quantum_deg=quantum,
                                           angle_min_interval_ms=200 if firmware else 500, smooth=firmware), **kw)
    status['steering']['smooth'] = dict(firmware=firmware, mode=mode, params=None, store='NVS', age_s=0.,
                                        last_reply=None, data_lines=0)
    return status


class Clock:
    def __init__(self, t=10.):
        self.t = t

    def __call__(self):
        return self.t


class FakeBridge:
    """Answers like the ESP32 bridge: status, stop, arm, drive (lease rules reduced to what the adapter sees)."""

    def __init__(self):
        self.status = esp_status()
        self.requests = []
        self.lease = None

    def __call__(self, request):
        self.requests.append(dict(request))
        op = request['op']
        if op == 'status':
            return dict(self.status)
        if op == 'stop':
            self.lease = None
            self.status = dict(self.status, armed=False, state='disarmed', reason=request.get('reason'),
                               steering_host=dict(self.status['steering_host'], arm_state='disarmed'))
            return dict(ok=True, state='stop_requested')
        if op == 'arm':
            self.lease = 'L'
            self.status = dict(self.status, armed=True, state='armed_manual',
                               steering_host=dict(self.status['steering_host'], arm_state='armed'))
            return dict(ok=True, lease='L', ticket='T0', seq=0, status=dict(self.status))
        if op in ('drive', 'keepalive'):
            if self.lease is None:
                return dict(ok=False, error='control_not_owned')
            return dict(ok=True, lease='L', ticket='T%d' % request['seq'], seq=request['seq'], status=dict(self.status))
        return dict(ok=False, error='unknown_operation')


def adapter(bridge=None, clock=None):
    return Esp32AbsHoldAdapter(bridge or FakeBridge(), mark_deg=MARK, wheelbase_m=.657, clock=clock or Clock(),
                               wall=lambda: 100.)


class Units(unittest.TestCase):
    def test_scale_and_sign(self):
        a = adapter()
        self.assertAlmostEqual(a.command_deg_of(100), -math.degrees(.1) * SCALE)        # left + -> command minus
        self.assertAlmostEqual(a.equivalent_mrad_of(a.command_deg_of(-77)), -77)
        self.assertEqual(a.bridge_mrad_of(-6.), 105)                                    # bridge left + mrad
        self.assertEqual(a.bridge_mrad_of(30.), -349)                                   # clamped to the bridge range

    def test_normalised_status(self):
        a = adapter()
        out = a.normalise(esp_status(cmd=-5., faults=['steer_hold_cancelled'], latches=['hardware_fault'], armed=True))
        self.assertEqual(out['steering']['measured_mrad'], round(math.radians(5. / SCALE) * 1000))
        self.assertEqual(out['steering']['faults'], [])                     # the firmware event is not a fault
        self.assertEqual(out['steering']['events'], ['steer_hold_cancelled'])
        self.assertEqual(out['steering_host']['latches'], {'hardware_fault': True})
        self.assertEqual(out['steering_host']['arm_state'], 'confirmed')
        self.assertEqual(out['steering_host']['counters'], dict(stop_sent=0, stop_confirmed=0))
        self.assertEqual(out['steering']['esp32']['command_equiv_deg'], -5.)


class Policy(unittest.TestCase):
    def drive(self, a, bridge, steer):
        return a(dict(op='drive', owner='o', lease='L', ticket='T', seq=1, deadman=True, throttle=0, steer_mrad=steer))

    def test_targets_change_at_most_once_per_interval_while_the_board_holds(self):
        clock, bridge = Clock(), FakeBridge()
        a = adapter(bridge, clock)
        a(dict(op='arm', owner='o', vehicle_profile='front_steer'))
        sent = []
        steer = 0.
        bridge.status = esp_status(armed=True, hold=True, pot=1900, target_pot=1990)     # holding, not reached
        for _ in range(40):                                                            # 4 s at 10 Hz, 50 mrad/s
            self.drive(a, bridge, int(round(steer)))
            sent.append((clock.t, bridge.requests[-1]['steer_mrad']))
            steer += 5.
            clock.t += .1
        changes = [sent[0]] + [b for a_, b in zip(sent, sent[1:]) if a_[1] != b[1]]
        gaps = [b[0] - a_[0] for a_, b in zip(changes, changes[1:])]
        self.assertTrue(all(g >= ESP32_POLICY['min_interval_s'] - 1e-6 for g in gaps), gaps)
        self.assertGreaterEqual(len(changes), 3)

    def test_reached_or_idle_board_takes_the_next_target_at_the_next_poll(self):
        clock, bridge = Clock(), FakeBridge()
        a = adapter(bridge, clock)
        a(dict(op='arm', owner='o', vehicle_profile='front_steer'))
        self.drive(a, bridge, 0)
        bridge.status = esp_status(armed=True, hold=True, pot=1990, target_pot=1995)      # reached (5 counts)
        a.normalise(bridge.status)
        clock.t += .2
        self.drive(a, bridge, 120)
        self.assertEqual(bridge.requests[-1]['steer_mrad'], a.bridge_mrad_of(round(a.command_deg_of(120))))

    def ramp(self, a, bridge, status, clock, seconds=4., rate_mrad_s=50.):
        """The executor's 10 Hz drives on a steering ramp (left) while the board holds a target it has not reached
        (its measured angle follows the last target, so the tracking rule stays quiet); returns the target changes
        [(t, command deg)] the adapter let through."""
        a(dict(op='arm', owner='o', vehicle_profile='front_steer'))
        sent, steer = [], 0.
        for _ in range(int(round(seconds / .1))):
            followed = a.sent_cmd or 0.
            bridge.status = dict(status, steering=dict(status['steering'], command_equiv_deg=followed,
                                                       measured_mrad=int(round(-math.radians(followed) * 1000))))
            a(dict(op='status'))                                    # the client's status poll: the adapter's board
            self.drive(a, bridge, int(round(steer)))
            self.assertEqual(bridge.requests[-1]['steer_mrad'], a.bridge_mrad_of(a.sent_cmd))
            sent.append((clock.t, a.sent_cmd))
            steer += rate_mrad_s * .1
            clock.t += .1
        return [sent[0]] + [b for a_, b in zip(sent, sent[1:]) if a_[1] != b[1]]

    def test_smooth_hold_status(self):
        good = dict(firmware=True, mode='SMOOTH', params=None, store='NVS')
        self.assertTrue(esp32_smooth_hold(dict(smooth=good)))
        for block in (dict(good, mode='PULSE'), dict(good, firmware=False), dict(good, firmware=None),
                      dict(good, mode=None), dict(good, firmware='true'), dict(mode='SMOOTH'), None, 'SMOOTH'):
            self.assertFalse(esp32_smooth_hold(dict(smooth=block)), block)
        self.assertFalse(esp32_smooth_hold({}))                     # a bridge older than da39e8a: no smooth block
        self.assertFalse(esp32_smooth_hold(None))

    def test_smooth_policy_follows_the_ramp_in_quarter_degrees(self):
        clock, bridge = Clock(), FakeBridge()
        a = adapter(bridge, clock)
        changes = self.ramp(a, bridge, smooth_status(armed=True, hold=True, pot=1900, target_pot=1990), clock)
        gaps = [b[0] - a_[0] for a_, b in zip(changes, changes[1:])]
        self.assertTrue(all(g >= ESP32_SMOOTH_POLICY['min_interval_s'] - 1e-6 for g in gaps), gaps)
        self.assertGreaterEqual(len(changes), 15)                   # about one target per 0.2 s over 4 s (pulse: 4)
        steps = [abs(b[1] - a_[1]) for a_, b in zip(changes, changes[1:])]
        self.assertTrue(all(st >= ESP32_SMOOTH_POLICY['deadband_deg'] - 1e-9 for st in steps), steps)
        self.assertTrue(all(abs(c / .25 - round(c / .25)) < 1e-9 and c <= 0 for _, c in changes), changes)
        self.assertTrue(any(abs(c - round(c)) > 1e-9 for _, c in changes), changes)   # finer than whole degrees
        self.assertEqual(a.policy_name(), 'smooth')
        self.assertEqual(a.counts['smooth_drives'], a.counts['drives'])
        self.assertEqual(a.normalise(bridge.status)['esp32_adapter']['policy'], 'smooth')

    def test_pulse_policy_when_smooth_is_off_or_missing(self):
        held = dict(armed=True, hold=True, pot=1900, target_pot=1990)
        cases = {'smooth_off': (smooth_status(mode='PULSE', firmware=False, quantum=1., **held), 1.),
                 'sm_line_not_yet': (smooth_status(mode=None, **held), .5),     # SMOOTH=1, no SM line: still pulse
                 'old_bridge': (esp_status(**held), 1.)}
        for name, (status, quantum) in cases.items():
            with self.subTest(name):
                clock, bridge = Clock(), FakeBridge()
                a = adapter(bridge, clock)
                changes = self.ramp(a, bridge, status, clock)
                gaps = [b[0] - a_[0] for a_, b in zip(changes, changes[1:])]
                self.assertTrue(all(g >= ESP32_POLICY['min_interval_s'] - 1e-6 for g in gaps), gaps)
                self.assertTrue(3 <= len(changes) <= 5, changes)
                self.assertTrue(all(abs(c / quantum - round(c / quantum)) < 1e-9 for _, c in changes), changes)
                self.assertEqual((a.policy_name(), a.counts['smooth_drives']), ('pulse', 0))
                self.assertEqual(a.normalise(bridge.status)['esp32_adapter']['policy'], 'pulse')

    def test_policy_follows_the_status_and_the_snapshot_shows_it(self):
        a = adapter()
        self.assertEqual(a.policy_name(), 'pulse')                  # no status yet
        a.normalise(smooth_status())
        self.assertEqual(a.policy_name(), 'smooth')
        a.normalise(smooth_status(mode='PULSE', firmware=False, quantum=1.))   # an operator's 'SM OFF'
        self.assertEqual(a.policy_name(), 'pulse')
        client = Esp32SteerBridgeClient(a, limit_left_mrad=280, limit_right_mrad=280, expect_cal=None)
        a.normalise(smooth_status())
        snap = client.snapshot()
        self.assertEqual(snap['adapter']['policy'], 'smooth')
        self.assertIn('smooth_drives', snap['adapter']['counts'])

    def test_smooth_quantum_is_the_bridges(self):
        for status, want in ((smooth_status(armed=True), -6.5),    # -6.43 command deg in 0.25 steps
                             (esp_status(armed=True), -6.)):        # pulse: the bridge's 1 deg quantum
            bridge = FakeBridge()
            a = adapter(bridge)
            a(dict(op='arm', owner='o', vehicle_profile='front_steer'))
            bridge.status = status
            a(dict(op='status'))
            self.drive(a, bridge, 100)
            self.assertAlmostEqual(a.command_deg_of(100), -6.4265, places=3)
            self.assertEqual(a.sent_cmd, want)

    def test_motion_never_reaches_the_bridge(self):
        bridge = FakeBridge()
        a = adapter(bridge)
        self.assertEqual(a(dict(op='motion', owner='o', lease='L', yaw_rate_rad_s=0., speed_m_s=0.))['error'],
                         'control_not_owned')
        a(dict(op='arm', owner='o', vehicle_profile='front_steer'))
        self.assertTrue(a(dict(op='motion', owner='o', lease='L', yaw_rate_rad_s=0., speed_m_s=0.))['ok'])
        self.assertNotIn('motion', [r['op'] for r in bridge.requests])


class Monitors(unittest.TestCase):
    def test_not_following_latches_and_stops_once_while_holding(self):
        clock, bridge = Clock(), FakeBridge()
        a = adapter(bridge, clock)
        a(dict(op='arm', owner='o', vehicle_profile='front_steer'))
        status = esp_status(armed=True, cmd=0., target_cmd=-6., hold=False)
        for _ in range(int(ESP32_TRACK['duration_s'] / .1) + 2):
            out = a.normalise(status)
            clock.t += .1
        self.assertIn('steering_not_following', a.latches)
        self.assertIn('steering_not_following', out['steering']['faults'])
        stops = [r for r in bridge.requests if r['op'] == 'stop']
        self.assertEqual(len(stops), 1)
        self.assertEqual(stops[0]['reason'], 'row_executor_esp32:steering_not_following')
        self.assertEqual(C.steer_fault_reason(out['steering']['faults']), 'steering_not_following')
        # the operator's re-arm clears the adapter latch
        a(dict(op='arm', owner='o', vehicle_profile='front_steer'))
        self.assertEqual(a.latches, {})

    def test_small_error_or_another_owner_never_latches(self):
        clock, bridge = Clock(), FakeBridge()
        a = adapter(bridge, clock)
        for _ in range(40):                                  # not holding: somebody else drives
            a.normalise(esp_status(armed=True, cmd=0., target_cmd=-10.))
            clock.t += .1
        a(dict(op='arm', owner='o', vehicle_profile='front_steer'))
        for _ in range(40):                                  # holding, 2.5 command deg = 2.2 equivalent deg off
            a.normalise(esp_status(armed=True, cmd=-2.5, target_cmd=-5.))
            clock.t += .1
        self.assertEqual(a.latches, {})
        self.assertEqual([r for r in bridge.requests if r['op'] == 'stop'], [])

    def test_ineffective_rule(self):
        clock, bridge = Clock(), FakeBridge()
        a = adapter(bridge, clock)
        a(dict(op='arm', owner='o', vehicle_profile='front_steer'))
        cmd = -8.                                            # 7.1 deg equivalent to the left
        delta = math.radians(8. / SCALE)
        predicted = .4 * math.tan(delta) / .657
        for _ in range(int(ESP32_INEFFECTIVE['duration_s'] / .1) + 2):
            a.normalise(esp_status(armed=True, cmd=cmd, target_cmd=cmd, hold=True, pot=2010, target_pot=2012))
            a(dict(op='motion', owner='o', lease='L', yaw_rate_rad_s=predicted, speed_m_s=.4))
            clock.t += .1
        self.assertEqual(a.latches, {})                      # the gyro agrees with v tan(delta) / L
        for _ in range(int(ESP32_INEFFECTIVE['duration_s'] / .1) + 2):
            a.normalise(esp_status(armed=True, cmd=cmd, target_cmd=cmd, hold=True, pot=2010, target_pot=2012))
            a(dict(op='motion', owner='o', lease='L', yaw_rate_rad_s=0., speed_m_s=.4))
            clock.t += .1
        self.assertIn('steering_ineffective', a.latches)
        self.assertEqual([r['reason'] for r in bridge.requests if r['op'] == 'stop'],
                         ['row_executor_esp32:steering_ineffective'])

    def test_stop_confirmation_counters(self):
        bridge = FakeBridge()
        a = adapter(bridge)
        a(dict(op='arm', owner='o', vehicle_profile='front_steer'))
        a(dict(op='stop', reason='row_executor_stop:operator_page'))
        out = a(dict(op='status'))
        self.assertEqual(out['steering_host']['counters'], dict(stop_sent=1, stop_confirmed=1))
        bridge.status = dict(bridge.status, steering_host=dict(bridge.status['steering_host'], stop_state='pending'))
        a(dict(op='stop', reason='again'))
        out = a(dict(op='status'))
        self.assertEqual(out['steering_host']['counters'], dict(stop_sent=2, stop_confirmed=1))


class Precheck(unittest.TestCase):
    def check(self, status, **over):
        kw = dict(expect_cal=(1681, 1900, 2180, 20), require_fw_watchdog=False, now=100.5)
        kw.update(over)
        return esp32_arm_precheck(adapter().normalise(status), **kw)

    def test_good_status_passes_and_the_v3_precheck_refuses_it(self):
        self.assertIsNone(self.check(esp_status()))
        self.assertEqual(steer_arm_precheck(adapter().normalise(esp_status()), expect_cal_id=1, allow_sim=False,
                                            limit_left_mrad=281, limit_right_mrad=281, now=100.5),
                         'firmware_v3_required')

    def test_smooth_firmware_status_passes(self):
        # the bridge of the SMOOTH firmware (FW line word SMOOTH=1): the same firmware name, a smooth block in the
        # steering status and finer pacing; the executor's checks do not depend on it
        status = esp_status(watchdog=300, steer_control=dict(kind='esp32_abs_hold', angle_quantum_deg=.25,
                                                             angle_min_interval_ms=200, smooth=True))
        status['steering']['smooth'] = dict(firmware=True, mode='SMOOTH', params=dict(STOP_TOL=6, RESTART_TOL=16),
                                            store='NVS', age_s=0., last_reply=None, data_lines=0)
        self.assertIsNone(self.check(status, require_fw_watchdog=True))
        status['steering']['smooth'] = dict(firmware=False, mode='PULSE', params=None, store='NVS')   # SMOOTH=0
        self.assertIsNone(self.check(status, require_fw_watchdog=True))

    def test_each_rule(self):
        cases = [
            (dict(ok=False), {}, 'status_not_ok'),
            (dict(connected=False), {}, 'esp_not_connected'),
            (dict(esp_kind='esp8266_v2'), {}, 'esp32_bridge_required'),
            (dict(firmware='OTHER'), {}, 'firmware_mismatch'),
            (dict(steer_limit_mrad=dict(left=300, right=349)), {}, 'steer_limit_mismatch'),
            (dict(direction_verified=False), {}, 'drive_direction_unverified'),
            ({}, dict(require_fw_watchdog=True), 'firmware_drive_watchdog_missing'),
            ({}, dict(expect_cal=None), 'calibration_id_not_configured'),
            ({}, dict(expect_cal=(1681, 1900, 2181, 20)), 'calibration_mismatch'),
            (dict(faults=['pot_invalid']), {}, 'steering_fault:pot_invalid'),
            (dict(age_ms=400), {}, 'steering_status_stale'),
            (dict(stop_state='pending'), {}, 'stop_unconfirmed'),
            (dict(armed=True), {}, 'bridge_armed_by_other_client'),
            (dict(moving=True), {}, 'bridge_reports_moving'),
            (dict(updated_at=90.), {}, 'bridge_snapshot_stale'),
        ]
        for status_over, kw, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(self.check(esp_status(**status_over), **kw), expected)
        self.assertIsNone(self.check(esp_status(watchdog=300), require_fw_watchdog=True))
        self.assertIsNone(self.check(esp_status(faults=['steer_hold_cancelled'])))       # an event, not a fault
        self.assertIsNone(self.check(esp_status(faults=['pot_invalid']), rearm=True))   # the bridge judges the re-arm

    def test_cal_tolerance(self):
        # 2026-11-20 (orchard): the centre was moved on the calibration tab and every Arm was refused. With a
        # tolerance the configured line may be off by that many counts per mark; without one nothing changes.
        board = (1681, 1900, 2180, 20)                                   # the line esp_status() reports
        for expect, tolerance, ok in (
                (board, 0, True), (board, 300, True),
                ((1681, 1860, 2180, 20), 0, False),                      # centre 40 off, exact match asked
                ((1681, 1860, 2180, 20), 39, False), ((1681, 1860, 2180, 20), 40, True),
                ((1651, 1860, 2230, 20), 50, True), ((1651, 1860, 2231, 20), 50, False),   # every mark counts
                ((1681, 1900, 2180, 18), 300, False),                    # another command scale
                ((1681, 1700, 2180, 20), 300, True),                     # left side 280 of the configured 480: kept
                ((1081, 1700, 2180, 20), 600, False),                    # right side 219, less than half of 619
                ((1681, 1700, 2780, 20), 600, False)):                   # left side 280, less than half of 1080
            problem = self.check(esp_status(), expect_cal=expect, cal_tolerance_counts=tolerance)
            self.assertEqual(problem, None if ok else 'calibration_mismatch', (expect, tolerance))
        self.assertFalse(esp32_cal_accepted((1681, 1900.0, 2180, 20), (1681, 1890, 2180, 20), 50))   # not an int
        self.assertFalse(esp32_cal_accepted((1681, None, 2180, 20), (1681, 1890, 2180, 20), 50))
        self.assertFalse(esp32_cal_accepted((1950, 1900, 2180, 20), (1681, 1900, 2180, 20), 600))    # RIGHT > CENTER
        self.assertFalse(esp32_cal_accepted((1681, 1890, 2180, 20), (1681, 1900, 2180, 20), True))   # a bool is no tolerance
        self.assertTrue(esp32_cal_accepted((1330, 2005, 2440, 20), (1330, 1885, 2440, 20), 300))     # the orchard case


class Client(unittest.TestCase):
    def test_arm_drive_and_latch(self):
        bridge = FakeBridge()
        a = adapter(bridge)
        client = Esp32SteerBridgeClient(a, owner='o', limit_left_mrad=281, limit_right_mrad=281,
                                        expect_cal=(1681, 1900, 2180, 20), require_fw_watchdog=False, wall=lambda: 100.5)
        client.arm()
        self.assertEqual(bridge.requests[-1], dict(op='arm', owner='o', vehicle_profile='front_steer'))
        client.drive(30, 100)
        sent = bridge.requests[-1]
        self.assertEqual((sent['throttle'], sent['steer_mrad']), (30, a.bridge_mrad_of(round(a.command_deg_of(100)))))
        self.assertTrue(client.motion(0., 0.))
        bridge.lease = None                                  # somebody else's STOP took the lease
        with self.assertRaises(Exception):
            client.drive(30, 100)
        self.assertEqual(client.state, LATCHED)
        self.assertFalse(a.holding)

    def test_precheck_refusal_sends_no_arm(self):
        bridge = FakeBridge()
        bridge.status = esp_status(direction_verified=False)
        client = Esp32SteerBridgeClient(adapter(bridge), owner='o', limit_left_mrad=281, limit_right_mrad=281,
                                        expect_cal=(1681, 1900, 2180, 20), require_fw_watchdog=False, wall=lambda: 100.5)
        with self.assertRaises(Exception):
            client.arm()
        self.assertEqual([r['op'] for r in bridge.requests], ['status'])

    def test_limits_beyond_the_bridge_range_are_refused(self):
        with self.assertRaises(ValueError):
            Esp32SteerBridgeClient(adapter(), limit_left_mrad=320, limit_right_mrad=281, expect_cal=None)


class Config(unittest.TestCase):
    def base(self):
        return C.load_config(os.path.join(PACKAGE, 'config.vehicle_esp32_shadow.json'))

    def test_vehicle_shadow_config(self):
        cfg = self.base()
        self.assertEqual((cfg.steer_bridge_kind, cfg.motor_output_enabled, cfg.bind_host, cfg.socket_path),
                         ('esp32_abs_hold', False, '127.0.0.1', '/run/ubts-rover/control.sock'))
        feasibility = cfg.feasibility()
        self.assertEqual(feasibility['status'], 'unknown')
        self.assertTrue(any('steer_polarity_checked' in u for u in feasibility['unknown']))
        self.assertFalse(any('steer_cal_id' in u for u in feasibility['unknown']))
        with self.assertRaises(C.ConfigError):               # motor output needs every field measurement
            replace(cfg, motor_output_enabled=True).validate()

    def test_rules(self):
        cfg = self.base()
        problems = lambda **over: C.esp32_rule_problems(replace(cfg, **over))  # noqa: E731
        self.assertEqual(problems(), [])
        self.assertTrue(problems(steer_bridge_kind='other'))
        self.assertTrue(problems(steer_esp32_mark_deg=None))
        self.assertTrue(problems(steer_wheelbase_m=None))
        beyond = int(C.ESP32_BRIDGE_LIMIT_MRAD / C.esp32_scale(cfg)) + 1   # the first soft limit past +-20 command deg
        self.assertTrue(problems(steer_limit_left_mrad=beyond))
        self.assertTrue(problems(steer_limit_right_mrad=beyond))
        self.assertEqual(problems(steer_limit_left_mrad=beyond - 1, steer_limit_right_mrad=beyond - 1), [])
        self.assertTrue(problems(steer_allow_sim=True))
        self.assertTrue(C.esp32_rule_problems(replace(cfg, steer_bridge_kind='v3')))   # esp32 keys without the kind
        self.assertEqual(C.steer_fault_reason(['pot_invalid']), 'steering_feedback_invalid')
        self.assertEqual(C.steer_fault_reason(['firmware_mismatch']), 'link_identity_mismatch')

    def test_untouched_block_digest_unchanged(self):
        self.assertEqual(C.ExecutorConfig().digest(), '700ee41c0188dffa058a2e1ce1f85411aee3a2ca38646d09451de79620df4340')

    def test_controller_takes_the_configured_wheelbase(self):
        from row_executor.core import default_steer_controller
        self.assertEqual(default_steer_controller(self.base()), {'kind': 'row_follow', 'wheelbase_m': .657})
        self.assertEqual(default_steer_controller(C.ExecutorConfig()), {'kind': 'row_follow'})


if __name__ == '__main__':
    unittest.main()
