"""Identification test scripts B0-B7 and F1-F5 in an IdentRig (design 3.9; plant side, SIM operator).

The field tests are manual: a person runs the bench tool (B tests, bridge stopped) or drives with the dashboard
(F tests, through the control API) and walks next to the vehicle. The scripts below stand in for that person. They
use only what the person has: the bench tool lines and the status lines, the dashboard control API, the tape and the
angle gauge (instruments.py), and their eyes for coarse driving decisions (operator_view(): where the car is, how far
it has turned; never an analysis input). Every test writes its own record (recorder.py); the analysis reads records.

Order (design 3.9 default order, B1 first since the analysis needs its lengths):
  bench phase   B1 tape, B7 cold, B0 polarity (JOG direction, EN cut), B2 end points / centre (CAL JOG, MARK, SOFT),
                B0 CAL POL, CAL SAVE, B3 static table (the host calibration comes from its analysis), B4 slow PWM ramp
                (JOG staircase, standstill), B5 steps (MCU rate limit lifted, PWM cap levels; standstill and moving),
                B6 back and forth (hysteresis), B7 warm and with drive PWM on (wheels lifted stand-in)
  drive phase   host rebuilt with the B2/B3 calibration, then F5 standstill steering, F4 slow zigzag, F2 fixed-steer
                circles, F3 30-60 % straight starts / stops (release and STOP) and short reverse, F1 straight back and
                forth (each round trip from the start mark; F1 last since its long legs move the car the most)
run_session() runs everything for one profile view and seed and writes <out>/records (the field records),
<out>/eval (truth for A18, evaluation only) and <out>/session.json. Placeholder-based SIM results only.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import time

import numpy as np

from .. import t1_rig as tr
from .. import geometry as geo_mod
from ..vehicle_profile import load_profile
from . import analysis
from . import bench as bt
from . import instruments
from . import recorder as rc
from . import rig as ir
from . import yard as yard_mod

SESSION_SCHEMA = 'ubts.steer_ident.session.v1'
START_POSE = (-12., 0., 0.)
# Start marks in the calibration yard (yard.py; free disk of about 17 m radius around the centre). Between drives the
# operator puts the stopped car back on a mark (in the field: drives or pushes it there; the SIM places it, rig.place()).
F1_START = (-20., 0., 0.)          # F1 lane along +x (a 20 m leg ends near x = 0)
F3_START = (-10., 0., 0.)
F4_START = (-10., 0., 0.)
F2_CENTRE = (0., 0.)               # every F2 circle starts so that its nominal centre is the yard centre
OPERATOR_CLEARANCE_M = 2.0      # the operator stops a drive when the car comes this close to a wall or post
DASHBOARD_STEER_RATE_DEG_S = 20.   # design 4.4: the dashboard limits the A/D target change (proposal 20 deg/s)
# Standing still the operator turns in short A/D pushes and watches the follow error on the dashboard (design 4.4
# shows target, measured angle and follow error). A slow continuous ramp does not work: the standstill servo lags by
# about (start PWM + rate share) / kp, 4 to 5 deg at 8 deg/s with the placeholders, and the host latches
# steering_not_following when the error stays above 3 deg for 1.5 s. So: push at most STANDSTILL_CHUNK_DEG beyond the
# measured angle, wait until the measured angle is within STANDSTILL_SETTLE_DEG of the request, push again; if the
# error has shown red (above the host 3 deg) for STANDSTILL_RED_S, back off to STANDSTILL_SAFE_LEAD_DEG ahead.
STANDSTILL_CHUNK_DEG = 5.
STANDSTILL_SETTLE_DEG = 1.
STANDSTILL_RED_S = 1.
STANDSTILL_SAFE_LEAD_DEG = 2.5
STANDSTILL_TURN_TIMEOUT_S = 120.
OPERATOR_PERIOD_S = .1          # operator input at 10 Hz (browser 7 Hz, executor 10 Hz)
POL_MIN_COUNTS = 4              # spec 7.10: CAL POL passes with |d1|, |d2| >= 4 raw counts [firmware placeholder]


@dataclass(frozen=True)
class Plan:
    """What the session runs (durations, levels). The defaults are the A18 plan; tests use shorter plans."""

    cold_s: float = 60.
    warm_s: float = 60.
    load_s: float = 20.
    b4_levels_pct: tuple = tuple(range(30, 101, 5))
    b4_reps: int = 2
    b4_jog_ms: int = 500
    b5_moving_levels_pct: tuple = (100, 80, 60, 50, 40, 35, 30)
    b5_standstill_levels_pct: tuple = (100, 60)
    b5_amplitude_permille: int = 300
    b5_drive_pct: int = 40
    b6_step_permille: int = 40
    b6_span_permille: int = 320
    b6_overshoot_permille: int = 300      # first point of a sweep approached from this far back (more than the play)
    b3_points_permille: tuple = (-750, -560, -375, -190, 0, 190, 375, 560, 750)
    b3_overshoot_permille: int = 300      # (the MCU clips the start at its soft limit: about 190 permille at the ends)
    f5_angles_deg: tuple = (12., 0., -12., 0., 6., 0.)    # moves kept inside the THERMAL budget (8 s holds)
    f5_hold_s: float = 8.
    f4_drive_pct: int = 30
    f4_amplitude_deg: float = 6.
    f4_half_period_s: float = 5.
    f4_leg_s: float = 30.
    f1_drive_pct: int = 40
    f1_leg_m: float = 20.
    f1_round_trips: int = 12              # 480 m at steering 0: the curvature wander averages to about 0.13 deg
    f2_drive_pct: int = 40
    f2_circles: tuple = ((8., 2.), (-8., 2.), (8., 2.), (-8., 2.), (13., 3.), (-13., 3.), (18., 3.), (-18., 3.))
    f3_duties_pct: tuple = (30, 40, 50, 60)
    f3_runs: tuple = ('release', 'brake', 'release')
    f3_drive_s: float = 10.
    f3_reverse: bool = True
    drive_tests: tuple = ('F5', 'F4', 'F2', 'F3', 'F1')   # F1 last: its long legs move the car the most
    # B7 load, third part (audit AUD3A-16): ARMED with the drive duty on and a slow steering target sweep, so the
    # steering PWM is on while the noise is measured (0: not run; the registered A18 plan ran without it)
    b7_servo_sweep_s: float = 0.
    b7_servo_sweep_permille: int = 150
    b7_servo_sweep_period_s: float = 10.


class SessionError(RuntimeError):
    pass


class StopEventLog:
    """Evaluation only (eval/truth.json, never a record or an analysis input; SIM_ACCEPTANCE amendment 80): the plant's
    slow-down events during F3. Wraps plant.step (an instance attribute over the method; nothing in the plant changes,
    no random draw). For each new event (plant._decel_event with a new index): mode, the event's own values (since,
    speed, deceleration, onset) and the plant pose at the start of the plant step that started it; then the pose at the
    end of the first later step with v == 0 (the stop). travel_along_m = the travel between the two poses projected on
    the heading at the start (the lateral slip drops out). tag(): what the operator was doing (duty, run kind)."""

    def __init__(self, plant):
        self.plant = plant
        self.events = []
        self.current = {}

    def tag(self, **what):
        self.current = dict(what)

    def install(self):
        plant = self.plant
        original = plant.step

        def step(dt, _original=original, _plant=plant):
            before = (_plant.x, _plant.y, _plant.yaw, _plant.time_s, _plant._decel_events)
            out = _original(dt)
            self._after_step(before)
            return out
        plant.step = step

    def remove(self):
        self.plant.__dict__.pop('step', None)

    def _after_step(self, before):
        plant = self.plant
        x0, y0, yaw0, t0, count = before
        event = plant._decel_event
        if plant._decel_events != count and event is not None:
            if self.events and self.events[-1].get('stop_s') is None:
                self.events[-1]['superseded'] = True
            self.events.append(dict(self.current, mode=event['mode'], index=int(event['index']),
                                    since_s=float(event['since']), speed_m_s=float(event['speed_m_s']),
                                    decel_m_s2=float(event['decel_m_s2']), onset_s=float(event['onset_s']),
                                    step_start_s=float(t0), x0=float(x0), y0=float(y0), yaw0=float(yaw0),
                                    stop_s=None, travel_along_m=None))
            return
        last = self.events[-1] if self.events else None
        if last is not None and last['stop_s'] is None and not last.get('superseded') and plant.v == 0.:
            dx, dy = plant.x - last['x0'], plant.y - last['y0']
            last.update(stop_s=float(plant.time_s), x1=float(plant.x), y1=float(plant.y),
                        travel_along_m=float(dx * math.cos(last['yaw0']) + dy * math.sin(last['yaw0'])))


def _deg(rad):
    return math.degrees(rad)


# --- bench phase ------------------------------------------------------------------------------------------------------
class BenchSession:
    """The bench tests on one IdentRig in bench mode; each test is its own bench record."""

    def __init__(self, rig, root, session_id, profile_copy, b1, gauge, nominal_profile, plan):
        self.rig = rig
        self.root = Path(root)
        self.session_id = session_id
        self.profile_copy = profile_copy
        self.b1 = b1
        self.gauge = gauge
        self.nominal = nominal_profile
        self.plan = plan
        self.records = {}
        self.tool = None
        self.record = None
        self.calibration = None
        self.cfg_saved = {}

    # -- record per test
    def open(self, test):
        rid = '%s_%s' % (self.session_id, test)
        record = rc.BenchRecord(self.root, rid, mono_start_s=self.rig.clock(), wall_start_s=self.rig.wall_clock(),
                                profile_copy=self.profile_copy, b1=self.b1)
        record.set_calibration(self.calibration)
        if self.tool is None:
            self.tool = bt.BenchTool(self.rig, record)
        else:
            self.tool.record = record
        self.record = record
        self.rig.set_phase(test)
        self.tool.begin(test)
        return record

    def close(self, test):
        self.tool.note('end')
        self.record.finish()
        self.records[test] = '_bench/%s' % self.record.rid
        self.record = None

    def folder(self, test):
        return str(self.root / self.records[test])

    def abort(self):
        """Close the open record after an error (the partial record is kept for inspection)."""
        if self.record is not None and not self.record.closed:
            self.tool.note('aborted', faults=[f for _, f in self.tool.faults[-5:]],
                           rejects=[r for _, r in self.tool.rejects[-5:]])
            self.record.finish()

    def read_gauge(self, test, point):
        p = self.rig.plant
        st = p._steer
        reading = self.gauge.read(st.delta_left_rad, st.delta_right_rad)
        self.record.add_gauge(self.rig.clock(), self.rig.wall_clock(), test, point, reading)
        return reading

    def mean_pos(self, seconds=.3):
        rows = self.tool.status_rows_since(self.rig.clock() - seconds)
        return float(np.mean([r['pos_permille'] for r in rows])) if rows else float(self.tool.status['pos'])

    # -- tests
    def b7(self, name, seconds):
        self.open(name)
        self.tool.wait_banner()
        self.tool.run(seconds)
        self.close(name)

    def b0(self):
        t = self.tool
        self.open('B0')
        t.wait_banner()
        found = {}
        for pct in (20, 30, 40, 50, 60):
            for sign in (1, -1):
                if sign in found:
                    continue
                pwm = sign * int(round(pct * 2.55))
                before = t.status['raw']
                t.note('jog', pwm=pwm, ms=300)
                t.jog(pwm, 300)
                t.run(.4)
                change = t.status['raw'] - before
                t.note('jog_result', pwm=pwm, raw_change=change)
                if abs(change) >= 4:
                    found[sign] = change
            if len(found) == 2:
                break
        # EN cut: STOP during a JOG must end the output at once
        t.note('en_cut_jog', pwm=153, ms=500)
        t.jog(153, 500, wait=False)
        t.run(.15)
        t.note('en_cut_stop')
        t.stop()
        t.run(.6)
        t.note('result', directions=found)
        self.close('B0')
        return found

    def _jog_to_end(self, sign, pwm=153, ms=500, max_jogs=40):
        t = self.tool
        last = None
        for _ in range(max_jogs):
            t.note('jog', pwm=sign * pwm, ms=ms)
            t.jog(sign * pwm, ms)
            t.run(.3)
            raw = t.status['raw']
            if last is not None and abs(raw - last) <= 1:
                return raw
            last = raw
        raise SessionError('B2: end stop not reached in %d jogs' % max_jogs)

    def b2(self):
        """End points and centre (CAL JOG / MARK), soft limits, CAL POL (B0), CAL SAVE."""
        t = self.tool
        self.open('B2')
        t.wait_banner()
        raw_l = self._jog_to_end(1)
        gl = self.read_gauge('B2', 'left_end')
        cal = t.mark('L')
        t.note('mark', which='L', raw=raw_l)
        raw_r = self._jog_to_end(-1)
        gr = self.read_gauge('B2', 'right_end')
        cal = t.mark('R')
        t.note('mark', which='R', raw=raw_r)
        # centre: from the right end move left in jogs, then in short jogs until the gauge reads straight (+-0.15 deg)
        for attempt in range(3):
            for _ in range(40):
                g = self.read_gauge('B2', 'centre_search')
                if .5 * (g['left_deg'] + g['right_deg']) >= -3.:
                    break
                t.jog(153, 300)
                t.run(.3)
            for _ in range(120):
                g = self.read_gauge('B2', 'centre_search')
                mean = .5 * (g['left_deg'] + g['right_deg'])
                if mean >= -.15:
                    break
                t.jog(115, 40)
                t.run(.25)
            g = self.read_gauge('B2', 'centre')
            mean = .5 * (g['left_deg'] + g['right_deg'])
            if abs(mean) <= .15:
                break
            t.note('centre_overshoot', gauge_mean_deg=mean)
            for _ in range(3):                                   # back to the right of the centre, approach again
                t.jog(-153, 300)
                t.run(.3)
        else:
            raise SessionError('B2: centre not found')
        cal = t.mark('C')
        t.note('mark', which='C', raw=t.status['raw'], gauge_mean_deg=mean)
        soft_l, soft_r = self._soft_from_gauge(gl, gr)
        cal = t.soft(soft_l, soft_r)
        t.note('soft', soft_l=soft_l, soft_r=soft_r)
        ack = t.pol()
        t.note('pol', ok=ack['ok'], d1=ack['d1'], d2=ack['d2'])
        if not ack['ok'] and ack['d1'] * ack['d2'] < 0 and min(abs(ack['d1']), abs(ack['d2'])) < POL_MIN_COUNTS:
            # the pulses moved the right way but less than the firmware minimum (spec 7.10: 4 counts): a slow actuator
            # at standstill (A18 variant B: 3 / -3 counts, R18 decision 18). The field procedure runs B0 / B2 with the
            # front lifted (design 3.9); the stand-in lifts the steered wheels and repeats CAL POL once
            t.note('pol_retry_lifted', d1=ack['d1'], d2=ack['d2'])
            self.rig.set_steering_lifted(True)
            try:
                ack = t.pol()
            finally:
                self.rig.set_steering_lifted(False)
            t.note('pol', ok=ack['ok'], d1=ack['d1'], d2=ack['d2'], lifted=True)
        if not ack['ok']:
            raise SessionError('B0 CAL POL failed: %r' % (ack,))
        cal = t.save()
        t.note('save', cal_id=cal['cal_id'], marks=[cal['left'], cal['center'], cal['right']])
        self.close('B2')
        return cal

    def _soft_from_gauge(self, gl, gr):
        """Soft limits in permille. At an end stop the wheel that turns more is against its limit (the gauge reads
        it); the equivalent soft limit is that wheel's limit minus the soft margin, as an equivalent angle (B1 lengths,
        nominal Ackermann ratio and margin), over the equivalent angle of the mark (the same wheel at the stop)."""
        v = self.b1['values']
        nominal = self.nominal.value
        a, margin = float(nominal('steer_ackermann_ratio')), float(nominal('steer_soft_margin_deg'))
        L, wk, kind = v['wheelbase_m'], v['kingpin_width_m'], nominal('steer_type')

        def side(g, sign):
            mags = {'left': sign * g['left_deg'], 'right': sign * g['right_deg']}
            wheel = max(mags, key=lambda k: mags[k])
            mark = sign * geo_mod.equivalent_from_wheel(sign * math.radians(mags[wheel]), wheel, kind, L, wk, a)
            soft = sign * geo_mod.equivalent_from_wheel(sign * math.radians(mags[wheel] - margin), wheel, kind, L, wk, a)
            return soft, mark
        soft_left, mark_left = side(gl, 1)
        soft_right, mark_right = side(gr, -1)
        soft_l = int(min(999, max(1, round(1000. * soft_left / mark_left))))
        soft_r = -int(min(999, max(1, round(1000. * soft_right / mark_right))))
        return soft_l, soft_r

    def b3(self):
        t = self.tool
        self.open('B3')
        t.arm(steer_permille=0)
        t.settled()
        for direction, points in (('up', self.plan.b3_points_permille),
                                  ('down', tuple(reversed(self.plan.b3_points_permille)))):
            over = self.plan.b3_overshoot_permille         # take up the play in the sweep direction first
            start = points[0] - over if direction == 'up' else points[0] + over
            t.command(steer_permille=max(-1000, min(1000, start)))
            t.settled()
            for p in points:
                t.cool_down()
                t.command(steer_permille=p)
                t.note('point', target_permille=p, direction=direction)
                t.settled(quiet_s=.4, timeout_s=8.)
                pos = self.mean_pos()
                g = self.read_gauge('B3', dict(target_permille=p, direction=direction, pos_permille=pos))
                t.note('reading', target_permille=p, direction=direction, pos_permille=pos, left_deg=g['left_deg'],
                       right_deg=g['right_deg'])
        t.command(steer_permille=0)
        t.settled()
        t.disarm()
        t.run(2.2)
        self.close('B3')
        self.calibration = analysis.b3_calibration(self.folder('B3'))
        t.calibration = self.calibration
        return self.calibration

    def _cfg(self, key, value):
        if key not in self.cfg_saved:
            self.cfg_saved[key] = self.tool.cfg_get(key)
        self.tool.cfg_set(key, value)
        self.tool.note('cfg', key=key, value=value)

    def restore_cfg(self):
        """Put every changed CFG value back. Bound pairs (minp <= maxp, rstd <= rmov, spec 4.10) can refuse a value
        until its partner is back, so a refused key ('range') is tried again after the others."""
        pending = list(self.cfg_saved)
        for _ in range(len(pending) + 1):
            refused = []
            for key in pending:
                value = self.cfg_saved[key]
                try:
                    self.tool.cfg_set(key, value)
                except bt.BenchError as exc:
                    if 'range' not in str(exc):
                        raise
                    refused.append(key)
                    continue
                self.tool.note('cfg_restore', key=key, value=value)
            pending = refused
            if not pending:
                break
        if pending:
            raise SessionError('CFG restore refused for %s' % pending)
        self.cfg_saved = {}
        cal = self.tool.cal_get()
        if cal['cfgdirty'] or cal['caldirty']:
            raise SessionError('CFG not restored: cal line %r' % (cal,))

    def b4(self):
        """Slow PWM ramp at standstill as a JOG staircase (alternating direction, so the steering stays central)."""
        t = self.tool
        self.open('B4')
        self._cfg('sthz', 50)
        t.run(.2)
        for rep in range(self.plan.b4_reps):
            for pct in self.plan.b4_levels_pct:
                for sign in (1, -1):
                    pwm = sign * min(255, int(round(pct * 2.55)))
                    t.cool_down()
                    t.run(.1)
                    t_w = t.jog(pwm, self.plan.b4_jog_ms, wait=False)
                    t.note('jog', pwm=pwm, ms=self.plan.b4_jog_ms, write_t=t_w, rep=rep)
                    t.run(self.plan.b4_jog_ms / 1000. + 1.)
        self.close('B4')

    def b5(self):
        """Steps with the MCU rate limit lifted (CFG rmov/rstd 5000; servo gain kp left at its default, a higher gain
        overshoots with a slow motor and trips POLARITY) at PWM cap levels (maxp; minp = maxp below the default minp):
        standstill and while driving (40 %, forward and reverse in turn)."""
        t = self.tool
        self.open('B5')
        current = {}
        for key, value in (('rmov', 5000), ('rstd', 5000), ('sthz', 50)):
            self._cfg(key, value)
            current[key] = value
        current['minp'] = t.cfg_get('minp')
        current['maxp'] = t.cfg_get('maxp')

        def set_level(level):
            if level < current['minp']:                    # keep minp <= maxp at every step (spec 4.10)
                self._cfg('minp', level)
                current['minp'] = level
            if current['maxp'] != level:
                self._cfg('maxp', level)
                current['maxp'] = level

        def run_level(pct, moving, direction):
            level = min(255, int(round(pct * 2.55)))
            set_level(level)
            if moving:
                t.cool_down(limit_s=.2)                     # the steps while driving must fit in the THERMAL budget
            t.arm()
            amp = self.plan.b5_amplitude_permille
            duty = direction * int(round(self.plan.b5_drive_pct * 2.55)) if moving else 0
            if moving:
                t.command(left=duty, right=duty)
                t.run(1.5)
            for target in (-amp, amp, -amp):
                if not moving:
                    t.cool_down()
                t.command(steer_permille=target)
                t.note('step', level=level, target_permille=target, moving=moving, direction=direction,
                       write_t=t.now())
                if self._step_progress(target) == 'no_progress':
                    t.note('no_motion', level=level, moving=moving, direction=direction)
                    t.run(.5)
                    break
                t.run(.4)
            if moving:
                t.command(left=0, right=0)
                t.run(1.2)
            t.disarm()
            t.run(2.3)
            if t.status['st'] != 1:
                raise SessionError('B5: not DISARMED after the level %d' % level)

        for pct in self.plan.b5_standstill_levels_pct:
            run_level(pct, False, 0)
        direction = 1
        for pct in self.plan.b5_moving_levels_pct:
            run_level(pct, True, direction)
            direction = -direction
        self.restore_cfg()
        self.close('B5')

    def _step_progress(self, target, timeout_s=6.):
        """Wait for a B5 step (within 30 permille of target). A cap at or below the start PWM does not move the motor,
        and the MCU latches TRACK after 1 s more than 3 deg (trk 120 permille) off cmd without progress (more than
        stm 20 permille toward cmd within stt 0.5 s). The operator watches the same: from 0.6 s on, every 0.2 s, the
        motion toward the target over the last 0.5 s (means of 0.1 s of status lines); two checks in a row under
        14 permille -> the target is taken back to the position before the MCU counts its second. A level whose
        motor moves 0.7 to 1 deg/s would pass this check but not the MCU's; the plan levels stay out of that band."""
        t = self.tool
        t0 = t.now()
        sign = 1 if target > t.status['pos'] else -1

        def mean_between(a, b):
            rows = [r['pos_permille'] for r in t.status_rows_since(a) if r['received_monotonic'] <= b]
            return float(np.mean(rows)) if rows else None
        low = 0
        next_check = t0 + .6
        while t.now() - t0 < timeout_s:
            t.run(.02)
            pos = t.status['pos']
            if abs(pos - target) <= 30:
                return 'reached'
            now = t.now()
            if now + 1e-9 < next_check:
                continue
            next_check = now + .2
            if abs(target - pos) <= 100:                     # within about trk: TRACK does not count
                low = 0
                continue
            recent, past = mean_between(now - .1, now), mean_between(now - .6, now - .5)
            if recent is None or past is None:
                continue
            low = low + 1 if (recent - past) * sign < 14 else 0
            if low >= 2:
                t.command(steer_permille=int(round(recent)))
                return 'no_progress'
        return 'timeout'

    def b6(self):
        t = self.tool
        self.open('B6')
        t.arm(steer_permille=0)
        t.settled()
        step, span = self.plan.b6_step_permille, self.plan.b6_span_permille
        up = list(range(-span, span + 1, step))
        for direction, points in (('up', up), ('down', list(reversed(up)))):
            over = self.plan.b6_overshoot_permille
            start = points[0] - over if direction == 'up' else points[0] + over
            t.command(steer_permille=start)
            t.settled(quiet_s=.4, timeout_s=8.)
            for p in points:
                t.cool_down()
                t.command(steer_permille=p)
                t.settled(quiet_s=.4, timeout_s=8.)
                pos = self.mean_pos()
                g = self.read_gauge('B6', dict(target_permille=p, direction=direction, pos_permille=pos))
                t.note('reading', target_permille=p, direction=direction, pos_permille=pos, left_deg=g['left_deg'],
                       right_deg=g['right_deg'])
        t.command(steer_permille=0)
        t.settled()
        t.disarm()
        t.run(2.2)
        self.close('B6')

    def b7_load(self, seconds):
        """Standstill noise under load: the drive PWM on (wheels lifted stand-in, BENCH pins), then the drive and the
        steering servo on together (ARMED, C with the drive duty and the target at the present position: the servo
        holds with short steering PWM pulses), then off again. Each part lasts `seconds`."""
        t = self.tool
        self.open('B7_load')
        t.run(2.)
        t.note('load_on', duty=153)
        self.rig.set_wheels_lifted(True)
        t.enter_bench()
        t.pin(8, 153)
        t.pin(3, 153)
        t.run(seconds)
        t.pin(8, 0)
        t.pin(3, 0)
        t.run(1.)
        t.note('load_off')
        t.disarm()
        t.run(2.3)
        t.arm()
        t.note('servo_load_on', duty=153)
        t.command(left=153, right=153)
        t.run(seconds)
        t.command(left=0, right=0)
        t.run(1.)
        t.note('servo_load_off')
        sweep_s = float(self.plan.b7_servo_sweep_s)
        if sweep_s > 0.:
            # third part (audit AUD3A-16): the drive duty on and the steering target moving slowly (triangle), so the
            # servo drives the steering motor: the noise with the steering PWM on (analysis: local line residual)
            amp, period = int(self.plan.b7_servo_sweep_permille), float(self.plan.b7_servo_sweep_period_s)
            centre = int(self.tool.status['pos'])
            t.note('servo_sweep_on', duty=153, amplitude_permille=amp, period_s=period)
            t.command(left=153, right=153)
            start = self.rig.now
            while self.rig.now - start < sweep_s:
                phase = ((self.rig.now - start) / period) % 1.
                tri = 4. * phase - 1. if phase < .5 else 3. - 4. * phase
                t.command(steer_permille=centre + amp * tri)
                t.run(.05)
            t.command(steer_permille=centre, left=0, right=0)
            t.run(1.)
            t.note('servo_sweep_off')
        t.disarm()
        t.run(2.)
        self.rig.set_wheels_lifted(False)
        self.close('B7_load')


# --- drive phase ------------------------------------------------------------------------------------------------------
class DriveSession:
    """F tests through the control API (the bridge in drive mode); each test is one archive recording."""

    def __init__(self, rig, root, session_id, profile_copy, calibration, plan, wheelbase_m=.76):
        self.rig = rig
        self.root = Path(root)
        self.session_id = session_id
        self.profile_copy = profile_copy
        self.calibration = calibration
        self.plan = plan
        self.wheelbase_m = float(wheelbase_m)       # the B1 tape length (the operator's circle size guess)
        self.client = None
        self.records = {}
        self.steer_mrad = 0
        self.events = []
        self.f3_stop_events = []        # evaluation only (StopEventLog, SIM_ACCEPTANCE amendment 80)

    def ready(self):
        rig = self.rig
        ok = rig.run_until_true(lambda: rig.core.steering_host()['stop_state'] == 'confirmed', 4.)
        if not ok:
            raise SessionError('bridge not ready: %r' % (rig.core.steering_host(),))
        self.client = rig.client('field-operator')

    def arm(self):
        status = self.client.status()
        if self.client.lease is not None and status.get('armed') and status.get('state') == 'armed_manual':
            return                                               # still armed (a release keeps the lease)
        problem = self.client.arm_precheck(status)
        if problem:
            self.rig.run_until_true(lambda: self.client.arm_precheck() is None, 1.)
            problem = self.client.arm_precheck()
        if problem:
            raise SessionError('arm precheck: %s' % problem)
        reply = self.client.arm()
        if not reply.get('ok') or not self.client.wait_armed():
            raise SessionError('arm failed: %r' % (reply,))

    def start(self, test, lidar=True):
        rig = self.rig
        rid = '%s_%s' % (self.session_id, test)
        record = rc.ArchiveRecorder(self.root, rid, mono_start_s=rig.clock(), wall_start_s=rig.wall_clock(),
                                    lidar_yaw_offset_deg=float(rig.profile.value('lidar_yaw_offset_deg')),
                                    profile_copy=self.profile_copy, host_profile_bytes=rig.host_profile_bytes,
                                    calibration=self.calibration, test=test)
        rig.set_lidar(lidar)
        rig.attach_recorder(record)
        rig.set_phase(test)
        return record

    def finish(self, test, record):
        rig = self.rig
        rig.step(.2)
        rig.attach_recorder(None)
        rig.set_lidar(False)
        record.finish(rig.clock(), rig.wall_clock())
        self.records[test] = record.rid

    def event(self, what, **params):
        self.events.append(dict(t=self.rig.clock(), what=what, **params))

    # -- operator input helpers (10 Hz, dashboard target rate limit)
    def _send(self, throttle):
        """One dashboard input (W/S throttle and the A/D steering target); re-arms after a stale-input stop."""
        reply = self.client.drive(int(throttle), int(self.steer_mrad))
        if not reply.get('ok') and reply.get('error') == 'stale_input_rearm_required':
            self.event('rearm', why=reply.get('error'))      # the operator presses Arm again after a pause
            self.arm()
            reply = self.client.drive(int(throttle), int(self.steer_mrad))
        if not reply.get('ok'):
            status = self.client.status()
            raise SessionError('drive refused: %r (bridge state %s, reason %s, latches %s)' % (
                reply, status.get('state'), status.get('reason'),
                sorted((status.get('steering_host') or {}).get('latches') or ())))

    def _measured_mrad(self):
        """The measured angle the dashboard shows (host calibration of the last status line)."""
        view = self.rig.core.steering_state()
        if view.get('angle_mrad') is None or view.get('stale'):
            raise SessionError('no measured steering angle on the dashboard: %r' % ({k: view.get(k) for k in (
                'state', 'stale', 'faults')},))
        return int(view['angle_mrad'])

    def hold(self, seconds, throttle, steer_deg=None, stop_when=None):
        """Keep the keys pressed: drive(throttle, steer) every 0.1 s for seconds; the steering target moves toward
        steer_deg at the dashboard rate limit (standing still: turn_standstill() first, on top of seconds).
        stop_when(): the operator's own stop condition (what they see)."""
        rig = self.rig
        target = self.steer_mrad if steer_deg is None else int(round(math.radians(steer_deg) * 1000.))
        if not throttle and target != self.steer_mrad:
            self.turn_standstill(math.degrees(target / 1000.))
        end = rig.clock() + seconds
        step = int(round(math.radians(DASHBOARD_STEER_RATE_DEG_S) * OPERATOR_PERIOD_S * 1000.))
        while rig.clock() < end - 1e-9:
            if self.steer_mrad != target:
                delta = target - self.steer_mrad
                self.steer_mrad += max(-step, min(step, delta))
            self._send(throttle)
            rig.run_until(min(end, rig.clock() + OPERATOR_PERIOD_S))
            if rig.clearance_m() < OPERATOR_CLEARANCE_M:
                self.event('operator_clearance_stop', clearance_m=rig.clearance_m())
                return 'clearance'
            if stop_when is not None and stop_when():
                return 'condition'
        return 'time'

    def turn_standstill(self, steer_deg, timeout_s=STANDSTILL_TURN_TIMEOUT_S):
        """Standing still: turn to steer_deg in short pushes (see STANDSTILL_CHUNK_DEG); returns the time taken."""
        rig = self.rig

        def mrad(deg):
            return int(round(math.radians(deg) * 1000.))
        target, chunk, settle, safe = (mrad(steer_deg), mrad(STANDSTILL_CHUNK_DEG), mrad(STANDSTILL_SETTLE_DEG),
                                       mrad(STANDSTILL_SAFE_LEAD_DEG))
        red = rig.core.hp.track_error_mrad               # the dashboard shows the follow error red above the host limit
        step = mrad(DASHBOARD_STEER_RATE_DEG_S * OPERATOR_PERIOD_S)
        t0 = rig.clock()
        goal, red_since, pushes, backoffs = None, None, 0, 0
        while rig.clock() < t0 + timeout_s:
            now, measured = rig.clock(), self._measured_mrad()
            if abs(self.steer_mrad - measured) > red:
                red_since = now if red_since is None else red_since
            else:
                red_since = None
            if goal is None or (self.steer_mrad == goal and abs(goal - measured) <= settle):
                if self.steer_mrad == target and abs(target - measured) <= settle:
                    self.event('standstill_turn', steer_deg=steer_deg, s=round(now - t0, 2), pushes=pushes,
                               backoffs=backoffs)
                    return now - t0
                left = target - measured
                goal = target if abs(left) <= chunk else measured + (chunk if left > 0 else -chunk)
                pushes += 1
            if red_since is not None and now - red_since >= STANDSTILL_RED_S:
                sign = 1 if goal > measured else -1
                goal = measured + sign * min(safe, abs(goal - measured))
                self.steer_mrad, red_since = goal, None
                backoffs += 1
            elif self.steer_mrad != goal:
                delta = goal - self.steer_mrad
                self.steer_mrad += max(-step, min(step, delta))
            self._send(0)
            rig.run_until(rig.clock() + OPERATOR_PERIOD_S)
        raise SessionError('standstill turn to %.1f deg did not settle within %.0f s (request %d mrad, measured %d '
                           'mrad)' % (steer_deg, timeout_s, self.steer_mrad, self._measured_mrad()))

    def idle(self, seconds):
        """Armed without input: keepalive every 0.5 s (the dashboard keeps its ticket; no motion is extended)."""
        rig = self.rig
        end = rig.clock() + seconds
        while rig.clock() < end - 1e-9:
            if self.client.lease is not None:
                self.client.keepalive()
            rig.run_until(min(end, rig.clock() + .5))

    def reposition(self, pose, timeout_s=20.):
        """Wait until the car stands still, then put it on the start mark pose (x, y, heading) (module START marks)."""
        rig = self.rig
        ok = rig.run_until_true(lambda: abs(rig.operator_view()['v']) < 1e-3, timeout_s)
        if not ok:
            raise SessionError('the car does not come to rest for the repositioning')
        rig.place(*pose)
        self.event('reposition', x=pose[0], y=pose[1], heading_deg=math.degrees(pose[2]))

    def release(self, wait_s=2.):
        self.client.stop('release')
        self.idle(round(wait_s, 2))

    def stop(self, wait_s=2.):
        self.client.stop('brake')
        self.rig.step(round(wait_s, 2))

    # -- tests
    def f5(self):
        rec = self.start('F5', lidar=False)
        self.arm()
        self.hold(1., 0, 0.)
        for angle in self.plan.f5_angles_deg:
            self.event('target', steer_deg=angle)
            self.hold(self.plan.f5_hold_s, 0, angle)
        self.release(1.)
        self.finish('F5', rec)

    def f4(self):
        rec = self.start('F4', lidar=False)
        self.reposition(F4_START)
        self.arm()
        p = self.plan
        for leg, sign in (('forward', 1), ('reverse', -1)):
            angle = p.f4_amplitude_deg
            end = self.rig.clock() + p.f4_leg_s
            while self.rig.clock() < end - 1e-9:
                self.event('zigzag', leg=leg, steer_deg=angle)
                why = self.hold(min(p.f4_half_period_s, end - self.rig.clock()), sign * p.f4_drive_pct, angle)
                if why == 'clearance':
                    break
                angle = -angle
            self.hold(.5, 0, 0.)
            self.release(1.5)
        self.finish('F4', rec)

    def _speed_guess(self, pct):
        return .0082 * pct          # the operator's rough walking guess of the speed (m/s); only for leg durations

    def f1(self):
        """Straight (steering target 0) round trips: before each one the operator puts the car back on the start mark,
        aimed along the lane (the straight curvature wander turns the car by about 15 deg per 20 m leg, so the legs
        would leave the yard). Forward for the leg length (the operator's speed guess), back for the same distance.
        A first version re-aligned the car by driving with steering; returning a larger angle toward the centre while
        driving tripped DRV_STUCK and a slow actuator tripped the host follow rule (IDENT_KO.md 7)."""
        rec = self.start('F1', lidar=True)
        p = self.plan
        self.arm()
        speed = self._speed_guess(p.f1_drive_pct)
        for k in range(p.f1_round_trips):
            self.reposition(F1_START)
            self.hold(1., 0, 0.)
            for sign in (1, -1):
                self.event('leg', direction=sign, k=k)
                self.hold(p.f1_leg_m / (speed * (1. if sign > 0 else .8)), sign * p.f1_drive_pct, 0.)
                self.release(2.)
        self.finish('F1', rec)

    def f2(self):
        """Fixed-steer circles. Each circle starts on a mark chosen so that its centre is the yard centre: first with
        the operator's size guess L / tan(angle); after the first 60 deg of the circle the operator sees its real size
        (chord over heading change) and, when it differs by more than 1.5 m, stops and puts the car on the mark for that
        size (once per circle; the short first arc is a circle of its own in the record). Then the laps."""
        rec = self.start('F2', lidar=True)
        self.arm()
        p = self.plan
        rig = self.rig
        for angle, laps in p.f2_circles:
            self.event('circle', steer_deg=angle, laps=laps)
            radius = self.wheelbase_m / math.tan(math.radians(abs(angle)))     # the operator's size guess
            side = -1. if angle > 0 else 1.                       # a left circle has its centre on the left
            for attempt in range(2):
                self.reposition((F2_CENTRE[0], F2_CENTRE[1] + side * radius, 0.))
                self.turn_standstill(angle)                       # turn the wheels at standstill first
                self.hold(1., 0, angle)
                start = rig.plant.yaw_unwrapped
                view0 = rig.operator_view()
                if attempt == 0:
                    def seen(start=start):                        # the first 60 deg of the circle
                        return abs(rig.plant.yaw_unwrapped - start) >= math.radians(60.)
                    why = self.hold(600., p.f2_drive_pct, angle, stop_when=seen)
                    view1 = rig.operator_view()
                    turned = abs(rig.plant.yaw_unwrapped - start)
                    chord = math.hypot(view1['x'] - view0['x'], view1['y'] - view0['y'])
                    seen_radius = chord / (2. * math.sin(turned / 2.)) if turned > 1e-3 else radius
                    if why == 'condition' and abs(seen_radius - radius) > 1.5:
                        self.event('circle_resize', steer_deg=angle, guess_m=radius, seen_m=seen_radius)
                        radius = seen_radius
                        self.release(2.)
                        continue
                    # the size fits: the laps go on from the same start (no stop)

                def done(start=start, laps=laps):                 # the operator sees the car back at the start
                    return abs(rig.plant.yaw_unwrapped - start) >= 2. * math.pi * laps
                self.hold(3600., p.f2_drive_pct, angle, stop_when=done)
                self.release(2.)
                break
        self.hold(2., 0, 0.)
        self.release(.5)
        self.finish('F2', rec)

    def f3(self):
        rec = self.start('F3', lidar=True)
        p = self.plan
        self.reposition(F3_START)
        log = StopEventLog(self.rig.plant)       # evaluation only: the plant's slow-down events (A18 brake rows)
        log.install()
        try:
            self.arm()
            self.hold(1., 0, 0.)
            for duty in p.f3_duties_pct:
                for mode in p.f3_runs:
                    self.event('run', duty=duty, stop=mode)
                    log.tag(duty=duty, run=mode, part='start')
                    self.hold(2., 0, 0.)
                    self.hold(p.f3_drive_s, duty, 0.)
                    log.tag(duty=duty, run=mode, part='stop')
                    if mode == 'brake':
                        self.stop(3.)
                        self.arm()
                    else:
                        self.release(3.)
                    if p.f3_reverse:
                        self.event('reverse', duty=duty)
                        log.tag(duty=duty, run=mode, part='reverse')
                        self.hold(p.f3_drive_s / .8, -duty, 0.)
                        self.release(2.)
        finally:
            log.remove()
            self.f3_stop_events = log.events
        self.finish('F3', rec)


# --- session ---------------------------------------------------------------------------------------------------------
def nominal_profile_copy():
    """The operator's profile copy (the shipped placeholder), as written into every record."""
    nominal = load_profile()
    return nominal, dict(profile_id=nominal.profile_id, profile_sha256=nominal.sha256,
                         note='operator copy of the vehicle profile (placeholder); B1/B2/B3 measurements are separate')


def run_session(out_dir, profile, seed, *, stream_key='ident', native_executable=None, plan=None, label='',
                log=print):
    """One identification session (bench + drive phase) for a profile view and seed (module docstring)."""
    plan = Plan() if plan is None else plan
    out = Path(out_dir)
    root = out / 'records'
    eval_dir = out / 'eval'
    root.mkdir(parents=True, exist_ok=False)
    eval_dir.mkdir(parents=True, exist_ok=False)
    nominal, copy = nominal_profile_copy()
    config = tr.RigConfig(seed=int(seed), stream_key=stream_key, world=yard_mod.YardSpec(), start_pose=START_POSE,
                          profile=profile, eeprom_blank=True, trace_every=5, serial_log=False)
    wall0 = time.time()
    rig = ir.IdentRig(config, native_executable=native_executable)
    timing = {}
    session_id = 'ident_s%d' % int(seed)
    try:
        b1 = instruments.Tape(seed, stream_key).measure(rig.profile)
        gauge = instruments.AngleGauge(seed, stream_key)
        bench = BenchSession(rig, root, session_id, copy, b1, gauge, nominal, plan)
        t0 = time.time()
        try:
            bench.b7('B7_cold', plan.cold_s)
            bench.b0()
            cal = bench.b2()
            calibration = bench.b3()
            bench.b4()
            bench.b5()
            bench.b6()
            bench.b7('B7_warm', plan.warm_s)
            bench.b7_load(plan.load_s)
        except Exception:
            bench.abort()
            raise
        timing['bench_wall_s'] = round(time.time() - t0, 2)
        timing['bench_sim_s'] = round(rig.now, 2)
        host_cal = dict(cal_id=int(cal['cal_id']), mark_left_mrad=calibration['mark_left_mrad'],
                        mark_right_mrad=calibration['mark_right_mrad'],
                        permille_per_deg=calibration['permille_per_deg'],
                        expected_soft_permille={'left': int(cal['soft_l']), 'right': int(cal['soft_r'])},
                        rule='B2 marks (end stops, gauge centre), B3 table (gauge, permille -> deg per side)')
        rig.enter_drive_mode(calibration=host_cal)
        drive = DriveSession(rig, root, session_id, copy, dict(host_cal, b3=calibration), plan,
                             wheelbase_m=b1['values']['wheelbase_m'])
        t1 = time.time()
        drive.ready()
        for test in plan.drive_tests:
            getattr(drive, test.lower())()
            log('  %s done at SIM %.1f s, wall %.1f s' % (test, rig.now, time.time() - wall0))
        timing['drive_wall_s'] = round(time.time() - t1, 2)
        timing['sim_s'] = round(rig.now, 2)
        truth = session_truth(rig, plan)
        truth['f3_stop_events'] = drive.f3_stop_events
        truth['wall_minus_sim_s'] = float(rig.wall_clock() - rig.clock())
        arrays = rig.trace_arrays()
        tr.write_npz(eval_dir / 'trace.npz', arrays)
        rc.write_json(eval_dir / 'truth.json', truth)
        rc.write_json(eval_dir / 'operator_events.json', drive.events)
        session = dict(schema=SESSION_SCHEMA, label=label, seed=int(seed), stream_key=stream_key,
                       records=dict(bench=bench.records, drive=drive.records), root='records',
                       calibration=host_cal, b3=calibration, plan=_plan_dict(plan), timing=timing,
                       identity=rig.identity(), result_kind=tr.RESULT_KIND, field_validated=False)
        rc.write_json(out / 'session.json', session)
        return session
    finally:
        rig.close()


def _plan_dict(plan):
    return {k: (list(v) if isinstance(v, tuple) else v) for k, v in plan.__dict__.items()}


def session_truth(rig, plan):
    """Known values of the session (evaluation only; written to eval/, never into a record)."""
    from field_response import forward_curve
    p = rig.plant
    v = rig.profile.value
    startup = rig.link_backend.startup
    act = startup['actuator']
    duties = {}
    for pct in plan.f3_duties_pct:
        applied = (pct * 255 + 50) // 100 * 100. / 255.
        duties[str(pct)] = dict(applied_duty_pct=applied,
                                speed_m_s=forward_curve(applied * rig.params.battery_voltage_scale,
                                                        rig.params.curve_params) * p.speed_run_scale)
    return dict(
        profile_id=rig.profile.profile_id, profile_sha256=rig.profile.sha256,
        profile_overrides=rig.profile.result_label().get('overrides'),
        steer_rate_max_deg_s=float(v('steer_rate_max_deg_s')),
        steer_rate_standstill_deg_s=float(v('steer_rate_standstill_deg_s')),
        steer_motor_tau_s=float(v('steer_motor_tau_s')), steer_min_pwm_pct=float(v('steer_min_pwm_pct')),
        steer_min_pwm_standstill_pct=float(v('steer_min_pwm_standstill_pct')),
        steer_backlash_deg=float(v('steer_backlash_deg')),
        steer_zero_offset_deg=math.degrees(act['zero_offset_rad']),
        pot_cal_residual_deg=math.degrees(startup['pot']['cal_residual_rad']),
        draws=dict(p.draws), speed_run_scale=p.speed_run_scale, gyro_scale=p.gyro_scale,
        battery_voltage_scale=rig.params.battery_voltage_scale, curve=rig.params.curve, duty_speed=duties,
        wheelbase_m=float(v('wheelbase_m')), eq_mech_limit_left_deg=math.degrees(
            startup['geometry']['eq_mech_limit_left_rad']),
        eq_mech_limit_right_deg=math.degrees(startup['geometry']['eq_mech_limit_right_rad']),
        ackermann_ratio=float(v('steer_ackermann_ratio')),
        curvature_wander=dict(v('curvature_wander')), result_kind=tr.RESULT_KIND)
