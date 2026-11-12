"""Safety scripts and P4 acceptance measurements (steering/safety_scripts; SIM_ACCEPTANCE A17, A19, A22).

Pure parts on hand-made artefacts: the catalogue (every required safety / 09-19 script present, stated expectations,
no sealed corridor, the false-detection groups), the spec 6.7 recomputation of the judgement ticks (STALL, POLARITY,
TRACK, THERMAL, DRV_STUCK, POT_*), the status-line field order, the STOP timing with an MCU stall, the amendment-7 resend
grid, the release ramp bound, the A22 command window, the analytic lateral shift of a link loss (rear-axle arc plus the
base_link offset), the A19 false-detection rule. Two scripts run end to end through the T1 executor-in-the-loop when
the native steering program can be built (skipped with the reason otherwise).

unittest style (also collected by pytest); no file is written inside the work tree. Placeholder values only
(자리값 기반 SIM 결과).
Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_safety_scripts.py
                    python3 -B -m unittest test_steering_safety_scripts     (Python 3.10, no pytest)
"""
import atexit
import math
import os
import shutil
import tempfile
import unittest

import numpy as np

from steering import actuator_link as al
from steering.safety_scripts import acceptance as A
from steering.safety_scripts import catalogue as CAT
from steering.safety_scripts import measure as M


def sim_in_loop_or_skip(case):
    """The T1 executor-in-the-loop module (the SIM tree estimator needs scipy), or skip with the reason (R10)."""
    try:
        return A._sim_in_loop()
    except Exception as exc:
        case.skipTest('sim_in_loop_steer not importable here (decision R10): %s: %s' % (type(exc).__name__, exc))

CFG = dict(stp=153, stt=500, stm=20, trk=120, trt=1000, minp=102, polt=100, polp=40, thp=204, thw=30, tha=5000,
           db=20, dst=300, dsm=80, med=5, pvmin=20, pvmax=1003, jmp=8, jmpw=2, nsd=50, nsw=50)
EXPECT_KEYS = {'end_reason', 'pause_reason', 'pause_at', 'mcu_fault', 'mcu_watchdog', 'mcu_ttl', 'host_latch',
               'arm_refused', 'stop_at', 'stop_unconfirmed', 'steer_off', 'hold_check', 'no_restart', 'a22', 'same_as',
               'client2', 'late_drive_error', 'lateral', 'loss_at', 'rate_counts', 'no_fault', 'operator_rearm_at',
               'bridge_restart', 'boot_after_s', 'first_rule', 'stop_phase',
               # one_press (design v2 V2-2, A17 amendment 31)
               'link_lost_at', 'link_timeout_s', 'other_screen', 'go_refused'}


def art_of(channels, cfg=None, **extra):
    art = dict(channels={name: list(channels.get(name, [])) for name in M.CHANNEL_NAMES}, cfg=dict(cfg or CFG),
               serial=[], lines=[], latches=[], summary=dict(executor_events=[], events=[]),
               host=dict(status_hz=20., stop_confirm_s=.1, stop_resend_interval_s=.05, stop_resend_max=5,
                         bridge_period_ms=20), servo_hz=500., boot_delay_ticks=70, hold_tolerance_rad=math.radians(1.))
    art.update(extra)
    return art


class CatalogueTest(unittest.TestCase):
    def test_required_scripts_and_expectations(self):
        names = [s['name'] for s in CAT.SCRIPTS]
        self.assertEqual(len(names), len(set(names)))
        required = ('mid_row_stop_w2', 'mid_row_stop_w0', 'deadman_release', 'hold_lost', 'c_line_loss_ttl',
                    'bridge_stall_watchdog', 'pot_open', 'pot_short', 'pot_jump', 'pot_noise', 'polarity_reversed',
                    'driver_stuck', 'cal_id_mismatch_executor', 'cal_id_mismatch_host', 'stale_steering_status',
                    'stop_unconfirmed', 'mcu_reset', 'power_cut',
                    # 09-19 mandatory failure tests
                    'two_screens_and_cli', 'link_loss_while_held', 'link_loss_lateral_5deg', 'bridge_forced_exit',
                    'usb_unplug_replug', 'half_frame', 'crc_error_single', 'crc_error_burst', 'range_error',
                    'status_line_loss', 'late_drive_after_stop', 'clock_change',
                    # review fixes: TTL phases (RS-01), watchdog brake (RS-03), reset on reopen (RS-13), host latches
                    'c_line_loss_ttl_status_phase', 'c_line_loss_ttl_bridge_phase', 'link_loss_watchdog_brake',
                    'bridge_forced_exit_port_reset', 'usb_unplug_replug_reset', 'host_hw_profile_mismatch',
                    'host_sim_identity_mismatch', 'host_arm_rejected', 'host_internal_error')
        for name in required:
            self.assertIn(name, names)
        by = CAT.by_name()
        # review P4R-05 / RS-04: the forced exit restarts the bridge process (not a stall of the same core)
        self.assertEqual([f['kind'] for f in by['bridge_forced_exit']['spec']['faults']], ['bridge_restart'])
        self.assertTrue(by['bridge_forced_exit']['expect']['bridge_restart'])
        self.assertTrue(by['usb_unplug_replug_reset']['spec']['rig']['port_open_reset'])
        # every host latch has a T1 script (review P4R-15)
        latched = set()
        for s in CAT.SCRIPTS:
            latched.update(A._as_tuple(s['expect'].get('host_latch', ())))
        self.assertEqual(set(A.HOST_LATCHES) - latched, set())
        self.assertEqual(A.HOST_LATCHES_NOT_IN_T1, {})
        # the A22 window fits into every A22 script (review P4R-06): duration >= the end of the script + 3 s is
        # checked at run time (command_after complete); the standstill script got its time back
        self.assertGreaterEqual(by['normal_steps_standstill']['spec']['duration_s'], 15.2 + 3.)
        for s in CAT.SCRIPTS:
            self.assertTrue(s['purpose'] and s['expect'] is not None, s['name'])
            self.assertLessEqual(set(s['expect']), EXPECT_KEYS, s['name'])
            self.assertIn(s['group'], ('safety', 'fault', 'mandatory', 'host', 'normal', 'rf03', 'finding'))
        for group, members in CAT.RATE_GROUPS.items():
            for name in members:
                self.assertIn(name, names)
        mandatory = {s['name'] for s in CAT.SCRIPTS if s['group'] == 'mandatory'}
        self.assertGreaterEqual(len(mandatory), 12)

    def test_no_sealed_corridor_and_development_worlds_only(self):
        from steering.scene_spec import sealed_status
        for s in CAT.SCRIPTS:
            world = s['spec']['world']
            self.assertIn(world['kind'], ('W0', 'W2'), s['name'])
            if world['kind'] == 'W2':
                self.assertIsNone(sealed_status(world['record'], world['corridor_id'], world['direction']), s['name'])
                self.assertFalse(world.get('allow_sealed', False))
            else:
                self.assertLessEqual(len(world.get('markers') or ()), 64)

    def test_specs_normalise(self):
        S = sim_in_loop_or_skip(self)
        for s in CAT.SCRIPTS:
            spec = S.normalise_spec(s['spec'])
            self.assertEqual(spec['name'], s['name'])
            for action in spec['actions']:
                self.assertIn(action['op'], S.ACTIONS)


class JudgementTest(unittest.TestCase):
    def test_stall(self):
        art = art_of(dict(servo_pos=[(0, 100)], servo_cmd=[(0, 400)], steer_pwm=[(1000, 200)], steer_mode=[(0, 1)],
                          steer_en=[(0, 1)]))
        tick, _ = M.expected_fault_tick('STALL', art, 1000, horizon_ms=2000)
        self.assertEqual(tick, 1500)            # the counter of tick k is seen by the judgement of tick k + 1
        art['channels']['steer_pwm'] = [(1000, 150)]
        self.assertIsNone(M.expected_fault_tick('STALL', art, 1000, horizon_ms=2000)[0])     # under stp

    def test_polarity(self):
        pos = [(t, 100 + (t - 1000) // 2) for t in range(1000, 1400)]     # moving up while the output pushes down
        art = art_of(dict(servo_pos=[(0, 100)] + pos, servo_cmd=[(0, 0)], steer_pwm=[(1000, -150)],
                          steer_mode=[(0, 1)], steer_en=[(0, 1)]))
        tick, _ = M.expected_fault_tick('POLARITY', art, 1000, horizon_ms=1000)
        # runPol counts ticks 1000 .. 1099 (100); the judgement of tick 1100 sees it, P[1100] - P[1000] = 50 > polp
        self.assertEqual(tick, 1100)

    def test_track_uses_the_mode_the_tick_started_with(self):
        art = art_of(dict(servo_pos=[(0, 0)], servo_cmd=[(0, 0), (1000, 200)], steer_pwm=[(1000, 140)],
                          steer_mode=[(0, 1), (2000, 0)], steer_en=[(0, 1)]))
        tick, _ = M.expected_fault_tick('TRACK', art, 1000, horizon_ms=3000)
        self.assertEqual(tick, 2000)            # 1000 ticks with |cmd - P| > trk from tick 1001 (cmd of tick k - 1)

    def test_thermal(self):
        art = art_of(dict(servo_pos=[(0, 0)], steer_pwm=[(1000, 210)], steer_mode=[(0, 1)], steer_en=[(0, 1)]))
        tick, _ = M.expected_fault_tick('THERMAL', art, 1000, horizon_ms=8000)
        self.assertEqual(tick, 6001)            # count of |d[k]| >= thp over k in [n - 30 s, n - 1] > 5000

    def test_drv_stuck(self):
        # brake (u = 0) from tick 1000 at P = 0, the position runs away while the output pushes back
        pos = [(0, 0)] + [(t, (t - 1100) // 2) for t in range(1100, 1700)]
        art = art_of(dict(servo_pos=pos, servo_cmd=[(0, 0)], steer_pwm=[(0, 0), (1150, -110)], steer_mode=[(0, 1)],
                          steer_en=[(0, 1)]))
        tick, method = M.expected_fault_tick('DRV_STUCK', art, 1000, horizon_ms=1000)
        self.assertIsNotNone(tick, method)
        self.assertGreater(abs(M.value_at(art['channels']['servo_pos'], tick)
                               - M.value_at(art['channels']['servo_pos'], tick - CFG['dst'])), CFG['dsm'])

    def test_pot_rules(self):
        raw = [(0, 500), (1000, 10)]
        art = art_of(dict(adc_read=raw))
        self.assertEqual(M.expected_fault_tick('POT_OPEN', art, 1000, horizon_ms=50)[0], 1002)   # median of 5
        art = art_of(dict(adc_read=[(0, 500), (1000, 1020)]))
        self.assertEqual(M.expected_fault_tick('POT_SHORT', art, 1000, horizon_ms=50)[0], 1002)
        art = art_of(dict(adc_read=[(0, 500), (1000, 530)]))
        tick, _ = M.expected_fault_tick('POT_JUMP', art, 1000, horizon_ms=50)
        self.assertEqual(tick, 1002)
        art = art_of({})
        self.assertEqual(M.expected_fault_tick('POT_NOISE', art, 1000)[0], None)               # needs adc_read


class HelperTest(unittest.TestCase):
    def test_status_fields_match_the_link(self):
        self.assertEqual(M.STATUS_FIELDS, al.STATUS_FIELDS)
        self.assertEqual(M.FAULT_BIT_NAMES[M.FAULT_BIT_NAMES.index('SOFTLIMIT')], 'SOFTLIMIT')
        text = 'S 1200 7 1 3 0 0 3 512 10 12 12 0 0100 500 100'
        art = art_of({}, serial=[dict(dir='m2h', t_ms=1200, first_tick=1199, last_tick=1201, text=text + '*00', note='')])
        fields = M.m2h_status(art)[0]['fields']
        self.assertEqual((fields['ms'], fields['n'], fields['st'], fields['L'], fields['fl'], fields['tl']),
                         (1200, 7, 1, 0, 0x100, 100))

    def test_stop_timing_with_an_mcu_stall(self):
        serial = [dict(dir='h2m', t_ms=6000, first_tick=6000, last_tick=6001, text='STOP 1*00', note=''),
                  dict(dir='m2h', t_ms=6850, first_tick=6840, last_tick=6845,
                       text='S 6840 9 1 3 0 0 3 512 10 12 12 0 0000 500 0*00', note='')]
        art = art_of(dict(drive_l=[(0, 100), (6800, 0)], drive_r=[(0, 100), (6800, 0)]), serial=serial)
        plain = M.stop_timing(art, 6.)
        self.assertEqual(plain['parse_to_zero_ticks'], 799)
        stalled = M.stop_timing(art, 6., stalls=[(6000, 6800)])
        self.assertEqual((stalled['processed_tick'], stalled['parse_to_zero_ticks']), (6800, 0))
        self.assertEqual(stalled['confirm_delay_ms'], 850)

    def test_resend_grid_amendment_7(self):
        art = art_of({})
        grid, latch = M.resend_grid(art)
        self.assertEqual(grid, [100, 160, 200, 260, 300])
        self.assertEqual(latch, 400)

    def test_release_bound(self):
        art = art_of(dict(drive_l=[(0, 102)], drive_r=[(0, 102)]))
        bound, d0 = M.release_bound_ticks(art, 1000)
        self.assertEqual((bound, d0), (400 + 10 + 1, 102))

    def test_command_after_and_hold_error(self):
        t = np.arange(0., 6., .01)
        target = np.where(t < 3., math.radians(5.), math.radians(5.))
        trace = dict(t_s=t, steer_target_rad=target, truth_hold_position_rad=target.copy(), truth_v_m_s=np.zeros_like(t),
                     ctrl_steer_mrad=np.full_like(t, 87.), truth_delta_eq_rad=target.copy())
        art = art_of({}, trace=trace)
        window = M.command_after(art, 2., 3.)
        self.assertTrue(window['passed'])
        trace['steer_target_rad'] = np.where(t < 3., math.radians(5.), 0.)                      # centring at rest
        window = M.command_after(art_of({}, trace=trace), 2., 3.)
        self.assertFalse(window['passed'])
        self.assertTrue(window['auto_centre'])
        self.assertAlmostEqual(M.hold_error(art, 2., 2.), 0.)


class ReviewFixTest(unittest.TestCase):
    """The review fixes of the measurement rules (P4R-06, P4R-07, P4R-10, P4R-11 / RS-02, RS-03, RS-10)."""

    def test_a22_window_must_be_complete(self):
        t = np.arange(0., 4.6, .01)
        trace = dict(t_s=t, steer_target_rad=np.zeros_like(t), truth_hold_position_rad=np.zeros_like(t),
                     truth_v_m_s=np.zeros_like(t), ctrl_steer_mrad=np.zeros_like(t))
        window = M.command_after(art_of({}, trace=trace), 2., 3.)      # the trace ends at 4.59 s, 2.6 s observed
        self.assertFalse(window['complete'])
        self.assertFalse(window['passed'])
        window = M.command_after(art_of({}, trace=trace), 1.5, 3.)
        self.assertTrue(window['complete'] and window['passed'])

    def test_restart_inside_the_first_1500_ms_is_found(self):
        # TTL brake at 5262, a C restarts the drive at 5382 (120 ms rest), a STOP brakes again at 6001: the old check
        # (drive 0 after t_stop + 1500) passed this
        drive = [(0, 102), (5262, 0), (5382, 2), (5392, 5), (5442, 17), (6001, 0)]
        art = art_of(dict(drive_l=drive, drive_r=drive))
        self.assertIsNone(M.drive_nonzero_after(art, 5262 + 1500))
        self.assertEqual(M.drive_increase_after(art, 5262), (5262, 5382))
        # a release ramp only shrinks: no increase
        ramp = [(0, 102)] + [(5000 + 50 * k, max(0, 102 - 13 * k)) for k in range(1, 10)]
        art = art_of(dict(drive_l=ramp, drive_r=ramp))
        zero, increase = M.drive_increase_after(art, 5000)
        self.assertIsNotNone(zero)
        self.assertIsNone(increase)

    def test_refused_lines_are_no_input(self):
        # spec 6.2: a refused line (N - crc at its arrival tick) does not update last_input
        serial = [dict(dir='h2m', t_ms=4860, first_tick=4861, last_tick=4862, text='C 7 300 102 102 0*AA', note='bridge'),
                  dict(dir='h2m', t_ms=4900, first_tick=4901, last_tick=4901, text='PING*00', note='bridge:crc_error'),
                  dict(dir='h2m', t_ms=4900, first_tick=4902, last_tick=4903, text='C 8 300 102 102 0*00',
                       note='bridge:crc_error')]
        lines = [('N - crc*00', 4901, 4902), ('N - crc*00', 4903, 4904), ('W 5363*00', 5363, 5364)]
        art = art_of({}, serial=serial, lines=lines, cfg=dict(CFG, wd=500))
        self.assertEqual(M.expected_watchdog_tick(art), (4862 + 500 + 1, 5363))
        refusals = M._refusals(art)
        self.assertTrue(M.refused_by_mcu(serial[1], refusals))
        self.assertFalse(M.refused_by_mcu(serial[0], refusals))
        # a C refused with its q (N 8 range) is no TTL start either
        serial[2] = dict(serial[2], note='bridge:range_error')
        lines = [('N 8 range*00', 4903, 4904), ('F TTL 1 5162*00', 5162, 5163)]
        art = art_of({}, serial=serial, lines=lines)
        expected, f_tick, last = M.expected_ttl_tick(art)
        self.assertEqual((expected, f_tick, last['text'][:3]), (4862 + 300, 5162, 'C 7'))

    def test_watchdog_brake_item_only_when_moving(self):
        serial = [dict(dir='h2m', t_ms=4860, first_tick=4861, last_tick=4862, text='C 7 300 102 102 0*AA', note='')]
        # the watchdog result of spec 6.4 (audit AUD3A-08 items): WATCHDOG latch bit from the W tick in the fault mask
        # and in the status lines the host reads, steering HOLD for hloss (default 2000 ms) then OFF, the run ended
        serial += [dict(dir='m2h', t_ms=t, first_tick=t, last_tick=t + 2, note='',
                        text='S %d %d 1 7 0 0 3 512 10 12 12 0 1000 500 0*00' % (t, t // 50)) for t in (5400, 5450)]
        mcu_result = dict(fault_bits=[(0, 0), (5363, 0x1000)], steer_mode=[(0, 1), (5363, 2), (7363, 0)])
        base = dict(spec=dict(faults=[], actions=[]), expect=dict(mcu_watchdog=True), name='x', group='safety')
        lines = [('W 5363*00', 5363, 5364)]
        result = dict(digests={}, wall_s=0.)
        summary = dict(end_reason='link_watchdog', events=[], executor_events=[], timing={})
        cfg = dict(CFG, wd=500)
        moving = art_of(dict(drive_l=[(0, 102), (5363, 0)], drive_r=[(0, 102), (5363, 0)],
                             mcu_state=[(0, 2), (5363, 1)], **mcu_result), cfg=cfg, serial=serial, lines=lines,
                        summary=summary)
        ev = A.evaluate_script(base, moving, result)
        item = [i for i in ev['items'] if i['item'].startswith('watchdog -> drive 0')][0]
        self.assertIs(item['ok'], True)
        braked = art_of(dict(drive_l=[(0, 102), (5162, 0)], drive_r=[(0, 102), (5162, 0)],
                             mcu_state=[(0, 2), (5363, 1)], **mcu_result), cfg=cfg, serial=serial, lines=lines,
                        summary=summary)
        ev = A.evaluate_script(base, braked, result)
        item = [i for i in ev['items'] if i['item'].startswith('watchdog -> drive 0')][0]
        self.assertIsNone(item['ok'])                 # not applicable: the TTL brake came first
        self.assertTrue(ev['passed'])
        # the same run without the latch bit in the status lines fails the A17 'latch' clause (AUD3A-08)
        unlatched = [dict(r, text=r['text'].replace(' 1000 500 ', ' 0000 500 ')) for r in serial]
        ev_bad = A.evaluate_script(base, dict(braked, serial=unlatched), result)
        self.assertFalse(ev_bad['passed'])
        self.assertIs([i for i in ev_bad['items'] if i['item'].startswith('watchdog latch bit kept')][0]['ok'], False)
        table = A.a17_table([dict(ev, group='safety')])
        self.assertTrue(table['passed'])
        self.assertEqual(len(table['not_applicable']), 1)

    def test_watchdog_latch_and_hold_end_at_an_mcu_reset(self):
        # usb_unplug_replug_reset: opening the port again resets the NodeMCU (spec 10.7) inside the hloss window; the
        # reset clears the RAM latch bit and ends the steering HOLD (pins OFF), the new boot reports BOOT
        serial = [dict(dir='h2m', t_ms=4960, first_tick=4961, last_tick=4962, text='C 7 300 102 102 0*AA', note='')]
        serial += [dict(dir='m2h', t_ms=t, first_tick=t, last_tick=t + 2, note='',
                        text='S %d %d 1 7 0 0 3 512 10 12 12 0 1000 500 0*00' % (t, t // 50)) for t in (5500, 5550)]
        base = dict(spec=dict(faults=[], actions=[]), expect=dict(mcu_watchdog=True), name='x', group='mandatory')
        summary = dict(end_reason='link_hardware_fault', events=[], executor_events=[], timing={})
        channels = dict(drive_l=[(0, 102), (5463, 0)], drive_r=[(0, 102), (5463, 0)],
                        mcu_state=[(0, 2), (5463, 1), (6000, 255), (6070, 1)],
                        fault_bits=[(0, 0), (5463, 0x1000), (6000, 0), (6070, 0x4000)],
                        steer_mode=[(0, 1), (5463, 2), (6000, 0)])
        art = art_of(channels, cfg=dict(CFG, wd=500), serial=serial, lines=[('W 5463*00', 5463, 5463)],
                     summary=summary)
        ev = A.evaluate_script(base, art, dict(digests={}, wall_s=0.))
        by = dict((i['item'], i) for i in ev['items'])
        self.assertIs(by['watchdog latch bit kept until the next operator ARM (MCU and status lines)']['ok'], True)
        hold = by['watchdog -> steering HOLD (measured position) for hloss, then OFF']
        self.assertIs(hold['ok'], True)
        self.assertEqual(hold['limit']['off_tick'], 6000)
        self.assertTrue(ev['passed'], [i for i in ev['items'] if i['ok'] is False])
        # the port closed until the reset: no status line reached the host, only the MCU part is judged; without the
        # reset the host must see the bit (no line is then a failure)
        quiet = dict(art, serial=serial[:1])
        ev = A.evaluate_script(base, quiet, dict(digests={}, wall_s=0.))
        item = [i for i in ev['items'] if i['item'].startswith('watchdog latch bit kept')][0]
        self.assertIs(item['ok'], True)
        self.assertEqual(item['measured'], dict(until_arm_tick=6000, lines=0, ended_by_reset=True))
        no_reset = dict(channels, mcu_state=[(0, 2), (5463, 1)], fault_bits=[(0, 0), (5463, 0x1000)])
        ev = A.evaluate_script(base, art_of(no_reset, cfg=dict(CFG, wd=500), serial=serial[:1],
                                            lines=[('W 5463*00', 5463, 5463)], summary=summary),
                               dict(digests={}, wall_s=0.))
        self.assertIs([i for i in ev['items'] if i['item'].startswith('watchdog latch bit kept')][0]['ok'], False)
        # without the reset the HOLD must last hloss: OFF at the reset tick is then a failure
        early = dict(channels, mcu_state=[(0, 2), (5463, 1)], fault_bits=[(0, 0), (5463, 0x1000)])
        ev = A.evaluate_script(base, art_of(early, cfg=dict(CFG, wd=500), serial=serial,
                                            lines=[('W 5463*00', 5463, 5463)], summary=summary),
                               dict(digests={}, wall_s=0.))
        hold = [i for i in ev['items'] if i['item'].startswith('watchdog -> steering HOLD')][0]
        self.assertIs(hold['ok'], False)
        self.assertEqual(hold['limit']['off_tick'], 5463 + 2000)

    def test_partial_runs_report_only_what_ran(self):
        self.assertIsNone(A.a17_table([])['passed'])
        self.assertIsNone(A.a22_table([])['passed'])

    def test_thermal_rate_counts_only_filled_windows(self):
        early = dict(mcu={}, host={}, end_reason='drive_no_response_repeat', first_active_at_s=1.2, ended_at_s=7.2,
                     duration_s=34., thermal_window_s=30.)
        full = dict(mcu={}, host={}, end_reason='script_complete', first_active_at_s=1.2, ended_at_s=32.,
                    duration_s=34., thermal_window_s=30.)
        tripped = dict(full, mcu={'THERMAL': 1}, ended_at_s=8., end_reason='steering_driver_fault')
        table = A.rate_table([('rf03_sweeps_duty4', 100 + k, early) for k in range(10)])
        row = table['thermal_rf03']['rf03_sweeps_duty4']
        self.assertIsNone(row['rate'])
        self.assertEqual((row['measured_episodes'], row['not_measured']), (0, 10))
        table = A.rate_table([('rf03_sweeps_duty0', 100, full), ('rf03_sweeps_duty0', 101, tripped),
                              ('rf03_sweeps_duty0', 102, early)])
        row = table['thermal_rf03']['rf03_sweeps_duty0']
        self.assertEqual((row['thermal'], row['measured_episodes'], row['rate']), (1, 2, .5))


class LateralTest(unittest.TestCase):
    def test_analytic_arc_includes_the_base_offset(self):
        from steering import drive as D
        from steering.vehicle_profile import load_profile
        profile = load_profile()
        brake = D.stop_modes(profile)['brake']
        d = -float(profile.value('rear_axle_x_in_base_m'))
        kappa, v0, loss, zero_s = .08, .43, 6., .263
        dt = .01
        t = np.arange(0., 8., dt)
        t1 = loss + zero_s + brake.onset_s
        v = np.where(t < t1, v0, np.maximum(0., v0 - brake.decel_median_m_s2 * (t - t1)))
        s = np.concatenate(([0.], np.cumsum(.5 * (v[1:] + v[:-1]) * dt)))
        s_rel = s - s[int(round(loss / dt))]
        psi = kappa * s_rel
        rx, ry = np.sin(psi) / kappa, (1. - np.cos(psi)) / kappa
        x, y = rx + d * np.cos(psi), ry + d * np.sin(psi)
        trace = dict(t_s=t, truth_v_m_s=v, truth_x_m=x, truth_y_m=y, truth_yaw_rad=psi,
                     truth_curvature_1_m=np.full_like(t, kappa), truth_delta_eq_rad=np.full_like(t, math.radians(5.)))
        zero_tick = int(round((loss + zero_s) * 1000.))
        art = art_of(dict(drive_l=[(0, 120), (zero_tick, 0)], drive_r=[(0, 120), (zero_tick, 0)]), trace=trace)
        out = A.lateral_check(art, loss)
        self.assertTrue(out['evaluated'])
        self.assertTrue(out['consistent'], out)
        self.assertAlmostEqual(out['sim_m'], out['analytic_m'], delta=.002)
        self.assertGreater(out['analytic_m'], out['rear_axle_arc_m'])       # the base offset term


class A19RuleTest(unittest.TestCase):
    def test_false_detections_of_every_normal_seed_fail_a19(self):
        clean = dict(mcu={}, host={}, end_reason='script_complete')
        rates = [(name, seed, clean) for name in CAT.RATE_GROUPS['normal'] for seed in (100, 101)]
        table = A.rate_table(rates)
        self.assertEqual(table['false_detections_normal'], [])
        rates[1] = (rates[1][0], 101, dict(mcu={'POLARITY': 1}, host={'mcu_disarmed': 1},
                                           end_reason='steering_polarity_fault'))
        table = A.rate_table(rates)
        self.assertEqual(len(table['false_detections_normal']), 1)
        self.assertAlmostEqual(table['false_detection_rate_normal'], 1. / len(rates))
        result = A.a19_table([], table)
        self.assertFalse(result['passed'])
        self.assertEqual(result['false_detections_normal'][0]['mcu'], ['POLARITY'])


_STATE = {}


def _cleanup(path):
    shutil.rmtree(path, ignore_errors=True)


def native_or_skip():
    from steering import t1_rig as tr
    from steering.native import build as native_build
    if 'native' not in _STATE:
        try:
            tr.host_modules()
            if not native_build.firmware_present():
                raise native_build.NativeBuildError(native_build.missing_firmware_message())
            native_build.find_compiler()
            folder = os.environ.get(native_build.ENV_BUILD_DIR)
            if not folder:
                folder = tempfile.mkdtemp(prefix='steer_safety_build_')
                atexit.register(_cleanup, folder)
            _STATE['native'] = native_build.ensure_built(build_dir=folder).path
        except (native_build.NativeBuildError, tr.RigError) as exc:
            _STATE['native'] = 'native steering program or host modules not available: %s' % exc
    value = _STATE['native']
    if not os.path.isfile(value):
        if native_build.firmware_required():
            raise AssertionError('%s is set: %s' % (native_build.ENV_REQUIRE_FIRMWARE, value))
        raise unittest.SkipTest(value)
    return value


class EndToEndTest(unittest.TestCase):
    def test_two_scripts_meet_their_expectation(self):
        sim_in_loop_or_skip(self)
        native = native_or_skip()
        for name in ('cal_id_mismatch_executor', 'c_line_loss_ttl'):
            entry = CAT.by_name()[name]
            result, art = A.run_script(entry, native_executable=native)
            ev = A.evaluate_script(entry, art, result)
            self.assertTrue(ev['passed'], [i for i in ev['items'] if not i['ok']])
            self.assertTrue(ev['items'])


if __name__ == '__main__':
    unittest.main()
