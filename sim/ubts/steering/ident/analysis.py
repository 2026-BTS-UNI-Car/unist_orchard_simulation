"""Identification analysis from the records only (design 3.9; A18). No SIM state is read here.

Inputs are the record folders written by the bench tool and the recorder (recorder.py; the field formats):
bench records <root>/_bench/<rid>/ (serial log with host times, operator notes, status lines, gauge readings, B1 tape
lengths) and drive records <root>/<rid>/ (recorder archive: motor.jsonl, mapping_source/sensors.jsonl.gz) with the
steering sidecar <root>/_steering/<rid>/. Every estimate carries its sample counts.

  b3_calibration    B3 static table: per side permille -> equivalent angle (host mark angles), the zero at
                    standstill, the Ackermann ratio from the two wheel readings, left / right asymmetry
  b4_jog            B4 slow PWM ramp (JOG staircase at standstill): displacement per JOG = rate * T * (u - m) / (1 - m)
                    for u > m (the L1 speed line) -> minimum PWM m and the standstill steering rate; tau from the
                    response of the strong JOGs (first-order speed lag fitted to the 50 Hz status lines)
  b5_steps          B5 steps with the MCU rate limit lifted and the PWM capped: slope of the position while the steering
                    PWM sits at the cap -> speed per cap level -> rate and minimum PWM (moving and standstill)
  b6_backlash       B6 back and forth: wheel angle (gauge) against the potentiometer angle on the up and the down
                    sweep; the gap between the two lines is the backlash
  b7_noise          B7 standstill noise (filtered ADC of the status lines), drift between cold and warm, shift with load
  f1_zero_offset    F1 straight (steering target 0): path curvature from the LiDAR odometry -> equivalent angle
  f2_kappa_table    F2 fixed-steer circles: curvature per steering target (LiDAR heading change / rear-axle path)
  f3_duty_speed     F3 through tools/field_calib (drive_calib: C2 ladder, C3 v(duty), C7 stop)
  f4_f5_steering    F4 / F5 steering slew and tracking seen in the status lines

Clock of the bench timing: status lines carry the MCU millis(); the host receive time of a line is at least its
transmission time (115200 baud) plus 1 ms after the MCU made it, so host - MCU offset = min over the lines of
(receive - ms - transmission - 1 ms) (a few ms late at most). A command written at host time t reaches the MCU at
t - offset + its own transmission time.

The LiDAR odometry and the drive steps are the imported tools/field_calib (records, odometry, steps, drive_calib);
they need scipy (cKDTree) and are imported only when F1, F2 or F3 are analysed.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys

import numpy as np

from .. import geometry as geo

SCHEMA = 'ubts.steer_ident.analysis.v1'
BAUD = 115200.
LINE_DONE_MS = 1.
FILTER_DELAY_MS = 2.            # median of 5 ADC samples (1 kHz): the filtered position lags about 2 ms
DEFAULT_LIDAR_YAW_OFFSET_DEG = 179.08
SUBSAMPLE_SCANS = 3             # F1 / F2 odometry: every 3rd scan (0.3 s; at most ~3 deg per match in F2)


class AnalysisError(ValueError):
    pass


# --- small helpers -----------------------------------------------------------------------------------------------------
def _load_jsonl(path):
    with open(path, 'r', encoding='utf-8') as fh:
        return [json.loads(line) for line in fh if line.strip()]


def _load_json(path):
    with open(path, 'r', encoding='utf-8') as fh:
        return json.load(fh)


def _r(x, nd=4):
    return None if x is None or not np.isfinite(x) else round(float(x), nd)


def tools_path():
    """<repository>/tools (the imported field_calib package); put on sys.path when needed."""
    here = Path(__file__).resolve()
    tools = here.parents[4] / 'tools'
    if not (tools / 'field_calib' / 'odometry.py').is_file():
        raise AnalysisError('tools/field_calib not found next to sim/ubts (%s)' % tools)
    if str(tools) not in sys.path:
        sys.path.insert(0, str(tools))
    return tools


def tx_s(nbytes):
    """UART time of a line of nbytes (LF included), 8N1 at 115200 baud."""
    return nbytes * 10. / BAUD


class Calibration:
    """Host calibration table: permille -> bicycle equivalent angle (deg), one line per side (marks in mrad)."""

    def __init__(self, mark_left_mrad, mark_right_mrad):
        self.left = float(mark_left_mrad) / 1000.
        self.right = float(mark_right_mrad) / 1000.

    @classmethod
    def from_dict(cls, data):
        return cls(data['mark_left_mrad'], data['mark_right_mrad'])

    def deg(self, permille):
        p = np.asarray(permille, dtype=float)
        return np.degrees(np.where(p >= 0., p * self.left, p * self.right) / 1000.)


class BenchData:
    """One bench record (recorder.BenchRecord layout)."""

    def __init__(self, folder):
        self.dir = Path(folder)
        self.descriptor = _load_json(self.dir / 'descriptor.json')
        self.serial = _load_jsonl(self.dir / 'bench.jsonl')
        self.status = _load_jsonl(self.dir / 'steering.jsonl')
        self.gauge = _load_jsonl(self.dir / 'gauge.jsonl')
        self.notes = [r for r in self.serial if r.get('dir') == 'note']
        self.t = np.array([r['received_monotonic'] for r in self.status], dtype=float)
        self.ms = np.array([r['mcu_ms'] for r in self.status], dtype=float)
        self.pos = np.array([r['pos_permille'] for r in self.status], dtype=float)
        self.raw = np.array([r['raw_adc'] for r in self.status], dtype=float)
        self.sp = np.array([r['steer_pwm'] for r in self.status], dtype=float)
        self.lines = np.array([r.get('line_bytes') or 0 for r in self.status], dtype=float)
        self.offset_s = self._offset()

    def _offset(self):
        """host - MCU clock offset (s) from the status lines (module docstring)."""
        if not len(self.t):
            return None
        return float(np.min(self.t - self.ms / 1000. - tx_s(self.lines) - LINE_DONE_MS / 1000.))

    def mcu_s(self, host_t, nbytes):
        """MCU time (s) at which a host line written at host_t has fully arrived."""
        return host_t - self.offset_s + tx_s(nbytes)

    def events(self, event):
        return [n for n in self.notes if n.get('event') == event]

    def b1(self):
        return (self.descriptor.get('b1_measurements') or {}).get('values') or {}

    def window(self, t0, t1):
        m = (self.t >= t0) & (self.t <= t1)
        return np.nonzero(m)[0]


def _jog_line_bytes(pwm, ms, notes):
    return len('CAL JOG %d %d*00' % (pwm, ms)) + 1


# --- B3 ---------------------------------------------------------------------------------------------------------------
def _ackermann_ratio(left_deg, right_deg, b1, steer_type):
    """Ackermann ratio a in [0, 1] that makes the left and the right single-wheel inversions agree best (LS)."""
    L, wk = b1['wheelbase_m'], b1['kingpin_width_m']
    left, right = np.radians(left_deg), np.radians(right_deg)
    use = np.abs(left) + np.abs(right) > np.radians(4.)
    if use.sum() < 3:
        return None, None
    grid = np.linspace(0., 1., 101)
    cost = []
    for a in grid:
        d = [geo.equivalent_from_wheel(float(l), 'left', steer_type, L, wk, float(a)) -
             geo.equivalent_from_wheel(float(r), 'right', steer_type, L, wk, float(a))
             for l, r in zip(left[use], right[use])]
        cost.append(float(np.mean(np.square(d))))
    k = int(np.argmin(cost))
    lo, hi = max(0, k - 1), min(len(grid) - 1, k + 1)
    if 0 < k < len(grid) - 1:                        # parabola through the three best grid points
        c0, c1, c2 = cost[lo], cost[k], cost[hi]
        den = c0 - 2. * c1 + c2
        a = grid[k] + (.5 * (c0 - c2) / den * (grid[1] - grid[0]) if den > 0 else 0.)
    else:
        a = grid[k]
    return float(min(1., max(0., a))), float(np.sqrt(cost[k]))


def b3_calibration(folder, steer_type='linkage'):
    """B3 static table -> host calibration (mark angles), zero, Ackermann ratio (module docstring)."""
    data = BenchData(folder)
    b1 = data.b1()
    rows = [n['params'] for n in data.events('reading')]
    if len(rows) < 6:
        raise AnalysisError('B3: %d readings (need 6)' % len(rows))
    pos = np.array([r['pos_permille'] for r in rows], dtype=float)
    left = np.array([r['left_deg'] for r in rows], dtype=float)
    right = np.array([r['right_deg'] for r in rows], dtype=float)
    a, mismatch = _ackermann_ratio(left, right, b1, steer_type)
    if a is None:
        raise AnalysisError('B3: too few readings away from the centre for the Ackermann ratio')
    L, wk = b1['wheelbase_m'], b1['kingpin_width_m']
    eq = np.degrees([.5 * (geo.equivalent_from_wheel(math.radians(l), 'left', steer_type, L, wk, a) +
                           geo.equivalent_from_wheel(math.radians(r), 'right', steer_type, L, wk, a))
                     for l, r in zip(left, right)])
    # eq = c0 + kL * max(pos, 0) + kR * min(pos, 0): least squares over both sweeps (the play averages out)
    design = np.column_stack([np.ones_like(pos), np.maximum(pos, 0.), np.minimum(pos, 0.)])
    coef, *_ = np.linalg.lstsq(design, eq, rcond=None)
    c0, kl, kr = (float(v) for v in coef)
    resid = eq - design @ coef
    mark_left_deg, mark_right_deg = 1000. * kl, 1000. * kr
    return dict(schema=SCHEMA, test='B3', mark_left_mrad=math.radians(mark_left_deg) * 1000.,
                mark_right_mrad=math.radians(mark_right_deg) * 1000., mark_left_deg=_r(mark_left_deg),
                mark_right_deg=_r(mark_right_deg), permille_per_deg=1000. / (.5 * (mark_left_deg + mark_right_deg)),
                zero_deg=_r(c0), ackermann_ratio=_r(a, 3), ackermann_mismatch_deg=_r(mismatch and math.degrees(mismatch)),
                lr_asymmetry=_r((kl - kr) / (kl + kr)), residual_deg_rms=_r(float(np.sqrt(np.mean(resid ** 2)))),
                residual_deg_max=_r(float(np.max(np.abs(resid)))), readings=len(rows),
                note='marks: per-side slope through the zero of the up and down sweeps (play averaged); zero_deg is '
                     'the wheel angle at permille 0 at standstill (host table has the centre at 0)')


# --- B4: JOG staircase ---------------------------------------------------------------------------------------------------
def _median_between(data, t0, t1):
    idx = data.window(t0, t1)
    return (float(np.median(data.pos[idx])), len(idx)) if len(idx) else (None, 0)


def _speed_line(u, speed):
    """Least squares speed = a u + c -> (minimum PWM -c/a, rate a + c, residual rms, n)."""
    u, speed = np.asarray(u, float), np.asarray(speed, float)
    if len(u) < 3 or np.ptp(u) < .1:
        return None
    a, c = np.polyfit(u, speed, 1)
    if a <= 0:
        return None
    resid = speed - (a * u + c)
    return dict(min_pwm=float(-c / a), rate=float(a + c), slope=float(a), intercept=float(c),
                residual_rms=float(np.sqrt(np.mean(resid ** 2))), n=int(len(u)))


def _first_order_response(t, T, tau):
    """Displacement of a first-order speed (lag tau) driven at unit speed for T, at times t after the start."""
    t = np.asarray(t, float)
    out = np.zeros_like(t)
    on = (t > 0) & (t <= T)
    out[on] = t[on] - tau * (1. - np.exp(-t[on] / tau))
    after = t > T
    if np.any(after):
        end = T - tau * (1. - math.exp(-T / tau))
        v_end = 1. - math.exp(-T / tau)
        out[after] = end + v_end * tau * (1. - np.exp(-(t[after] - T) / tau))
    return out


def _fit_tau(t_rel, disp_deg, T, taus=None):
    """(tau, speed, offset, rms): grid search over tau with a linear fit of (speed, offset) for each."""
    taus = np.linspace(.01, .5, 491) if taus is None else taus
    best = None
    for tau in taus:
        g = _first_order_response(t_rel, T, tau)
        design = np.column_stack([g, np.ones_like(g)])
        coef, *_ = np.linalg.lstsq(design, disp_deg, rcond=None)
        rms = float(np.sqrt(np.mean((design @ coef - disp_deg) ** 2)))
        if best is None or rms < best[3]:
            best = (float(tau), float(coef[0]), float(coef[1]), rms)
    return best


def b4_jog(folder, calibration):
    """B4: minimum PWM and steering rate at standstill from the JOG displacements; tau from the strong JOGs."""
    data = BenchData(folder)
    cal = Calibration.from_dict(calibration)
    jogs = [n['params'] for n in data.events('jog')]
    rows = []
    for j in jogs:
        pwm, ms, t_w = int(j['pwm']), int(j['ms']), float(j['write_t'])
        before, nb = _median_between(data, t_w - .15, t_w)
        after, na = _median_between(data, t_w + ms / 1000. + .6, t_w + ms / 1000. + .95)
        if before is None or after is None:
            continue
        disp = float(cal.deg(after) - cal.deg(before))
        rows.append(dict(pwm=pwm, u=abs(pwm) / 255., ms=ms, write_t=t_w, disp_deg=disp,
                         speed_deg_s=abs(disp) / (ms / 1000.), sign=1 if pwm > 0 else -1, before=before, after=after))
    moving = [r for r in rows if r['speed_deg_s'] > 1.5 and np.sign(r['disp_deg']) == r['sign']]
    line = _speed_line([r['u'] for r in moving], [r['speed_deg_s'] for r in moving])
    full = [r['speed_deg_s'] for r in rows if r['pwm'] in (255, -255)]
    # tau: fit the strong JOGs (u >= 0.7) with the first-order model on the status lines
    taus = []
    for r in rows:
        if r['u'] < .7 or data.offset_s is None:
            continue
        t_mcu = data.mcu_s(r['write_t'], _jog_line_bytes(r['pwm'], r['ms'], None)) + FILTER_DELAY_MS / 1000.
        idx = data.window(r['write_t'] - .1, r['write_t'] + r['ms'] / 1000. + .6)
        if len(idx) < 15:
            continue
        t_rel = data.ms[idx] / 1000. - t_mcu
        disp = (cal.deg(data.pos[idx]) - cal.deg(r['before'])) * r['sign']
        fit = _fit_tau(t_rel, disp, r['ms'] / 1000.)
        if fit is not None and fit[1] > 1.:
            taus.append(dict(pwm=r['pwm'], tau_s=fit[0], speed_deg_s=fit[1], rms_deg=fit[3], samples=len(idx)))
    out = dict(schema=SCHEMA, test='B4', jogs=len(rows), moving_jogs=len(moving),
               min_pwm_standstill_pct=_r(line and 100. * line['min_pwm'], 3),
               rate_standstill_deg_s_line=_r(line and line['rate'], 3),
               rate_standstill_deg_s_full_pwm=_r(float(np.median(full)) if full else None, 3),
               line=line, tau_s=_r(float(np.median([x['tau_s'] for x in taus])) if taus else None, 4),
               tau_fits=taus, tau_n=len(taus), clock_offset_s=_r(data.offset_s, 6),
               rows=rows, note='speed per JOG = displacement / T (a first-order speed gives exactly rate * T); the line '
                               'uses the JOGs with > 1.5 deg/s in the commanded direction')
    return out


# --- B5: capped steps ----------------------------------------------------------------------------------------------------
def _c_line_bytes(data, write_t):
    """Length (bytes with the newline) of the last C line written at or before write_t (serial log)."""
    best = None
    for r in data.serial:
        if r.get('dir') == 'h2m' and r.get('kind') == 'C' and r['t'] <= write_t + 1e-9:
            best = r
    return len(best['text']) + 1 if best is not None else 32


def _step_fit(t_rel, disp, tau, delays):
    """Fit disp = v * g(t - d; tau) + c (first-order speed from rest at d, g = t - tau (1 - exp(-t / tau))) over the
    saturated samples; returns (v, d, c, rms) of the best start delay d."""
    best = None
    for d in delays:
        g = _first_order_response(t_rel - d, 1e9, tau)
        if np.ptp(g) <= 0:
            continue
        design = np.column_stack([g, np.ones_like(g)])
        coef, *_ = np.linalg.lstsq(design, disp, rcond=None)
        rms = float(np.sqrt(np.mean((design @ coef - disp) ** 2)))
        if best is None or rms < best[3]:
            best = (float(coef[0]), float(d), float(coef[1]), rms)
    return best


def b5_steps(folder, calibration, tau_s=None):
    """B5: per PWM cap level the steady steering speed while the output sits at the cap (moving / standstill).

    The speed comes from a first-order fit of the saturated part of each step (motor from rest, lag tau from B4; the
    start delay after the C line reached the MCU is fitted): a straight line over the saturated samples would read
    the speed while it is still rising and come out low by a few tau / (saturated time)."""
    data = BenchData(folder)
    cal = Calibration.from_dict(calibration)
    steps = [n['params'] for n in data.events('step')]
    taus = [float(tau_s)] if tau_s else list(np.linspace(.02, .4, 39))
    delays = np.arange(0., .081, .002)
    per_level = {}
    for k, s in enumerate(steps):
        t0 = float(s['write_t'])
        t1 = float(steps[k + 1]['write_t']) if k + 1 < len(steps) else t0 + 6.5
        idx = data.window(t0, min(t1, t0 + 6.5))
        level = int(s['level'])
        sat = idx[np.abs(data.sp[idx]) == level]
        if len(sat) < 6:
            continue
        sign = np.sign(data.sp[sat])
        main = sat[sign == np.sign(np.median(sign))]
        breaks = np.nonzero(np.diff(main) != 1)[0]
        if len(breaks):
            main = main[:breaks[0] + 1]                     # the first saturated run of status lines only
        if len(main) < 5 or data.offset_s is None:
            continue
        t_arr = data.mcu_s(t0, _c_line_bytes(data, t0)) + FILTER_DELAY_MS / 1000.
        t_rel = data.ms[main] / 1000. - t_arr
        before, _ = _median_between(data, t0 - .2, t0)
        if before is None:
            continue
        disp = (cal.deg(data.pos[main]) - cal.deg(before)) * float(np.median(sign))
        fits = [(tau,) + _step_fit(t_rel, disp, tau, delays) for tau in taus]
        fits = [f for f in fits if f[1] is not None]
        if not fits:
            continue
        tau, v, d, c, rms = min(fits, key=lambda f: f[4])
        keep = main[t_rel >= t_rel[0] + .15]
        slope = (abs(float(np.polyfit(data.ms[keep] / 1000., cal.deg(data.pos[keep]), 1)[0]))
                 if len(keep) >= 3 else None)
        key = ('moving' if s['moving'] else 'standstill', level)
        per_level.setdefault(key, []).append(dict(speed_deg_s=float(abs(v)), start_delay_s=d, tau_s=tau, rms_deg=rms,
                                                  line_slope_deg_s=slope, samples=int(len(main)),
                                                  saturated_s=float(t_rel[-1] - t_rel[0]),
                                                  target_permille=s['target_permille'],
                                                  direction=s.get('direction')))
    out = dict(schema=SCHEMA, test='B5', levels={}, tau_s_used=_r(tau_s, 4) if tau_s else None)
    for regime in ('moving', 'standstill'):
        levels = sorted(l for r, l in per_level if r == regime)
        table = []
        for level in levels:
            fits = per_level[(regime, level)]
            table.append(dict(level=level, u=level / 255.,
                              speed_deg_s=float(np.median([f['speed_deg_s'] for f in fits])),
                              line_slope_deg_s=_r(float(np.median([f['line_slope_deg_s'] for f in fits
                                                                   if f['line_slope_deg_s'] is not None]))
                                                  if any(f['line_slope_deg_s'] is not None for f in fits) else None, 3),
                              steps=len(fits), samples=int(sum(f['samples'] for f in fits)),
                              saturated_s=_r(float(np.median([f['saturated_s'] for f in fits])), 3)))
        moving = [x for x in table if x['speed_deg_s'] > .5]
        line = _speed_line([x['u'] for x in moving], [x['speed_deg_s'] for x in moving])
        full = [x['speed_deg_s'] for x in table if x['level'] == 255]
        out['levels'][regime] = dict(table=table, line=line,
                                     min_pwm_pct=_r(line and 100. * line['min_pwm'], 3),
                                     rate_deg_s_line=_r(line and line['rate'], 3),
                                     rate_deg_s_full_pwm=_r(full[0] if full else None, 3))
    out['note'] = ('speed per step = first-order fit of the saturated part (|steer PWM| equal to the cap; lag tau '
                   'from B4); line speed = a u + c over the levels with motion: minimum PWM = -c / a, rate = a + c')
    return out


# --- B6 ------------------------------------------------------------------------------------------------------------------
def b6_backlash(folder, calibration):
    data = BenchData(folder)
    cal = Calibration.from_dict(calibration)
    rows = [n['params'] for n in data.events('reading')]
    if len(rows) < 8:
        raise AnalysisError('B6: %d readings' % len(rows))
    motor = cal.deg([r['pos_permille'] for r in rows])
    wheel = np.array([.5 * (r['left_deg'] + r['right_deg']) for r in rows], dtype=float)
    up = np.array([r['direction'] == 'up' for r in rows])
    design = np.column_stack([motor, up.astype(float), (~up).astype(float)])
    coef, *_ = np.linalg.lstsq(design, wheel, rcond=None)
    k, c_up, c_down = (float(v) for v in coef)
    resid = wheel - design @ coef
    return dict(schema=SCHEMA, test='B6', backlash_deg=_r(c_down - c_up), gain=_r(k), offset_up_deg=_r(c_up),
                offset_down_deg=_r(c_down), residual_deg_rms=_r(float(np.sqrt(np.mean(resid ** 2)))),
                readings_up=int(up.sum()), readings_down=int((~up).sum()),
                note='wheel (gauge mean of both wheels) = k * potentiometer angle + offset per sweep direction; '
                     'backlash = offset(down) - offset(up)')


# --- B0, B2, B7 ---------------------------------------------------------------------------------------------------------
def b0_polarity(folder, b2_folder=None):
    data = BenchData(folder)
    results = [n['params'] for n in data.events('jog_result')]
    found = {}
    for r in results:
        if abs(r['raw_change']) >= 4:
            found.setdefault('plus' if r['pwm'] > 0 else 'minus', r)
    ok = ('plus' in found and 'minus' in found and np.sign(found['plus']['raw_change']) !=
          np.sign(found['minus']['raw_change']))
    stop_t = [n['t'] for n in data.events('en_cut_stop')]
    en_cut = None
    if stop_t:
        idx = data.window(stop_t[0] + .03, stop_t[0] + .5)
        en_cut = dict(steer_pwm_after_stop=[int(v) for v in data.sp[idx][:5]],
                      output_off=bool(len(idx) and np.all(data.sp[idx] == 0)),
                      raw_span_after=float(np.ptp(data.raw[idx[3:]])) if len(idx) > 4 else None)
    out = dict(schema=SCHEMA, test='B0', directions=found, opposite=bool(ok), en_cut=en_cut)
    if b2_folder:
        pol = BenchData(b2_folder).events('pol')
        out['cal_pol'] = pol[-1]['params'] if pol else None
    return out


def b2_marks(folder, steer_type='linkage', ackermann_ratio=None):
    data = BenchData(folder)
    b1 = data.b1()
    marks = {n['params']['which']: n['params'] for n in data.events('mark')}
    gauges = {g['point']: g for g in data.gauge if isinstance(g.get('point'), str)}
    out = dict(schema=SCHEMA, test='B2', marks=marks)
    a = 0.5 if ackermann_ratio is None else ackermann_ratio
    for name, key in (('left_end', 'eq_mech_left_deg'), ('right_end', 'eq_mech_right_deg')):
        g = gauges.get(name)
        if g is None:
            continue
        left, right = math.radians(g['left_deg']), math.radians(g['right_deg'])
        if name == 'left_end':
            value = max(geo.equivalent_from_wheel(left, 'left', steer_type, b1['wheelbase_m'], b1['kingpin_width_m'], a),
                        geo.equivalent_from_wheel(right, 'right', steer_type, b1['wheelbase_m'], b1['kingpin_width_m'], a))
        else:
            value = -min(geo.equivalent_from_wheel(left, 'left', steer_type, b1['wheelbase_m'], b1['kingpin_width_m'], a),
                         geo.equivalent_from_wheel(right, 'right', steer_type, b1['wheelbase_m'], b1['kingpin_width_m'], a))
        out[key] = _r(math.degrees(value))
    c = gauges.get('centre')
    out['centre_gauge_deg'] = None if c is None else _r(.5 * (c['left_deg'] + c['right_deg']))
    soft = data.events('soft')
    out['soft'] = soft[-1]['params'] if soft else None
    save = data.events('save')
    out['save'] = save[-1]['params'] if save else None
    return out


def b7_noise(folders):
    """folders: {'cold': dir, 'warm': dir, 'load': dir}: filtered ADC noise sd, drift, load shift (report)."""
    out = dict(schema=SCHEMA, test='B7')
    means = {}
    for name in ('cold', 'warm'):
        if name not in folders:
            continue
        data = BenchData(folders[name])
        raw = data.raw[data.t >= data.t[0] + 1.] if len(data.t) else data.raw
        means[name] = float(np.mean(raw)) if len(raw) else None
        out[name] = dict(samples=int(len(raw)), raw_mean=_r(means[name], 3),
                         raw_sd_filtered=_r(float(np.std(raw)) if len(raw) else None, 3),
                         seconds=_r(float(np.ptp(data.t)) if len(data.t) else None, 1))
    if means.get('cold') is not None and means.get('warm') is not None:
        out['drift_counts'] = _r(means['warm'] - means['cold'], 3)
    if 'load' in folders:
        data = BenchData(folders['load'])
        on = [n['t'] for n in data.events('load_on')]
        off = [n['t'] for n in data.events('load_off')]
        if on and off:
            loaded = data.raw[(data.t > on[0] + 1.) & (data.t < off[0] - 1.)]
            quiet = data.raw[data.t < on[0]]
            out['load'] = dict(samples=int(len(loaded)), raw_mean_loaded=_r(float(np.mean(loaded)), 3) if len(loaded) else None,
                               raw_sd_loaded=_r(float(np.std(loaded)), 3) if len(loaded) else None,
                               raw_mean_unloaded=_r(float(np.mean(quiet)), 3) if len(quiet) else None,
                               shift_counts=_r(float(np.mean(loaded) - np.mean(quiet)), 3) if len(loaded) and len(quiet)
                               else None)
        son = [n['t'] for n in data.events('servo_load_on')]
        soff = [n['t'] for n in data.events('servo_load_off')]
        if son and soff:
            idx = (data.t > son[0] + 1.) & (data.t < soff[0] - 1.)
            held = data.raw[idx]
            out['servo_load'] = dict(samples=int(len(held)), raw_sd=_r(float(np.std(held)), 3) if len(held) else None,
                                     steer_pwm_on_fraction=_r(float(np.mean(data.sp[idx] != 0)), 3) if len(held) else None,
                                     note='drive duty on and the servo holding (steering PWM pulses); raw counts')
    return out


# --- drive records: steering sidecar and LiDAR odometry -------------------------------------------------------------------
class DriveData:
    """One drive record: steering sidecar rows, motor rows (throttle requests), descriptor; scans on demand."""

    def __init__(self, root, rid):
        self.root = Path(root)
        self.rid = rid
        self.steer_dir = self.root / '_steering' / rid
        self.descriptor = _load_json(self.steer_dir / 'descriptor.json')
        self.steering = _load_jsonl(self.steer_dir / 'steering.jsonl')
        self.motor = _load_jsonl(self.root / rid / 'motor.jsonl')
        self.mono0 = float(self.descriptor['monotonic_start_s'])
        self.wall0 = float(self.descriptor['wall_start_s'])
        st = [r for r in self.steering if r.get('mcu_ms') is not None]
        self.st_t = np.array([r['received_monotonic'] for r in st], dtype=float)
        self.st_ms = np.array([r['mcu_ms'] for r in st], dtype=float)
        self.st_pos = np.array([r['pos_permille'] for r in st], dtype=float)
        self.st_target = np.array([np.nan if r.get('target_mrad') is None else r['target_mrad'] for r in st], float)
        self.st_sp = np.array([r['steer_pwm'] for r in st], dtype=float)
        self.st_tgt = np.array([r['target_permille'] for r in st], dtype=float)
        self.st_cmd = np.array([r['cmd_permille'] for r in st], dtype=float)
        self.st_drive = np.array([r['drive_l'] for r in st], dtype=float)
        self.st_faults = [r.get('faults') or [] for r in st]
        self.mo_t = np.array([r['t_s'] + self.mono0 for r in self.motor], dtype=float)
        self.mo_throttle = np.array([np.nan if (r.get('requested') or {}).get('throttle') is None
                                     else r['requested']['throttle'] for r in self.motor], dtype=float)

    def wall_of(self, mono):
        return self.wall0 + (np.asarray(mono, float) - self.mono0)

    def segments(self, min_s, settle_s, nonzero_throttle=True):
        """[(t0, t1, target_mrad, throttle)] runs of constant steering target and constant throttle (status rows)."""
        if not len(self.st_t):
            return []
        if not len(self.mo_t):
            return []
        # throttle at each status row: the last motor row at or before it
        k = np.searchsorted(self.mo_t, self.st_t, side='right') - 1
        thr = np.where(k >= 0, np.nan_to_num(self.mo_throttle[np.clip(k, 0, len(self.mo_t) - 1)]), 0.)
        out = []
        i = 0
        n = len(self.st_t)
        while i < n:
            j = i
            while (j + 1 < n and self.st_target[j + 1] == self.st_target[i] and thr[j + 1] == thr[i]
                   and self.st_t[j + 1] - self.st_t[j] < .5):
                j += 1
            t0, t1 = self.st_t[i] + settle_s, self.st_t[j]
            if np.isfinite(self.st_target[i]) and t1 - t0 >= min_s and (thr[i] != 0 or not nonzero_throttle):
                out.append((float(t0), float(t1), float(self.st_target[i]), float(thr[i])))
            i = j + 1
        return out

    def scans(self, windows_mono, every=SUBSAMPLE_SCANS):
        """Scans (receipt mono time, ranges_mm, geometry) inside the windows, every `every`-th scan of each window."""
        tools_path()
        from field_calib import records
        import gzip
        windows = [(float(self.wall_of(a)), float(self.wall_of(b))) for a, b in windows_mono]
        out = [[] for _ in windows]
        counters = [0 for _ in windows]
        geometry = None
        with gzip.open(self.root / self.rid / 'mapping_source' / 'sensors.jsonl.gz', 'rb') as fh:
            for line in fh:
                if not line.startswith(records.SCAN_PREFIX):
                    continue
                t = records._bag_time(line)
                if t is None:
                    continue
                for w, (a, b) in enumerate(windows):
                    if a <= t <= b:
                        counters[w] += 1
                        if (counters[w] - 1) % every == 0:
                            key, geo_, th, mm = records._scan_to_mm(line)
                            geometry = geometry or geo_
                            out[w].append((self.mono0 + (t - self.wall0), mm))
                        break
        return out, geometry


def _lever(b1, axis_rad):
    """Rear-axle centre relative to the LiDAR in the LiDAR scan frame (m)."""
    rx, lx, ly = b1['rear_axle_x_in_base_m'], b1['lidar_x_in_base_m'], b1.get('lidar_y_in_base_m', 0.)
    ax, ay = math.cos(axis_rad), math.sin(axis_rad)            # body x in the scan frame
    px, py = -ay, ax                                          # body y in the scan frame
    dx, dy = rx - lx, -ly
    return np.array([dx * ax + dy * px, dx * ay + dy * py])


def path_curvature(scans, geometry, b1, axis_rad, workers=1):
    """Heading change, signed rear-axle path length and curvature of one window from the LiDAR odometry."""
    tools_path()
    from field_calib import odometry as odo
    if len(scans) < 5:
        return None
    t = np.array([s[0] for s in scans], dtype=float)
    mm = np.stack([s[1] for s in scans])
    pose, quality, chain = odo.odometry(mm, geometry['angle_min'], geometry['angle_increment'], [(0, len(mm))],
                                        workers=workers, chunk_scans=200)
    lever = _lever(b1, axis_rad)
    heading = 0.
    path = 0.
    pairs = 0
    gaps = 0
    ax = np.array([math.cos(axis_rad), math.sin(axis_rad)])
    for i in range(1, len(t)):
        if chain[i] < 0 or chain[i - 1] != chain[i] or not np.isfinite(pose[i, 0]) or not np.isfinite(pose[i - 1, 0]):
            gaps += 1
            continue
        c0, s0 = math.cos(pose[i - 1, 2]), math.sin(pose[i - 1, 2])
        c1, s1 = math.cos(pose[i, 2]), math.sin(pose[i, 2])
        r0 = pose[i - 1, :2] + np.array([c0 * lever[0] - s0 * lever[1], s0 * lever[0] + c0 * lever[1]])
        r1 = pose[i, :2] + np.array([c1 * lever[0] - s1 * lever[1], s1 * lever[0] + c1 * lever[1]])
        step = r1 - r0
        body_x = np.array([c0 * ax[0] - s0 * ax[1], s0 * ax[0] + c0 * ax[1]])
        ds = float(np.linalg.norm(step)) * (1. if float(step @ body_x) >= 0. else -1.)
        dth = float(math.atan2(math.sin(pose[i, 2] - pose[i - 1, 2]), math.cos(pose[i, 2] - pose[i - 1, 2])))
        heading += dth
        path += ds
        pairs += 1
    if pairs < 3 or abs(path) < .5:
        return None
    return dict(heading_deg=math.degrees(heading), path_m=path, curvature_1_m=heading / path, pairs=pairs, gaps=gaps,
                seconds=float(t[-1] - t[0]), speed_m_s=abs(path) / max(float(t[-1] - t[0]), 1e-6),
                matched_fraction=float(np.mean(chain >= 0)))


def _axis_rad(descriptor):
    return math.radians(-DEFAULT_LIDAR_YAW_OFFSET_DEG)


def combine_curvature(parts):
    """Path-weighted mean curvature (1/m) of segments [dict(heading_deg, path_m)]: heading_i = kappa path_i with the
    path signed (negative in reverse), so kappa = sum heading_i sign(path_i) / sum |path_i| (a plain sum / sum would
    divide by about zero over forward and reverse passes of the same length)."""
    heading = sum(p['heading_deg'] * (1. if p['path_m'] > 0 else -1.) for p in parts)
    path = sum(abs(p['path_m']) for p in parts)
    if path <= 0:
        raise AnalysisError('no path')
    return math.radians(heading) / path


def f1_zero_offset(root, rid, b1, workers=1, every=SUBSAMPLE_SCANS):
    """F1: path curvature with the steering target 0 -> equivalent angle atan(kappa L) (zero offset; it includes the
    straight-line curvature bias of the drive, which F1 cannot tell apart from the steering zero)."""
    data = DriveData(root, rid)
    segs = [s for s in data.segments(min_s=4., settle_s=2.) if s[2] == 0.]
    scans, geometry = data.scans([(a, b) for a, b, _, _ in segs], every=every)
    axis = _axis_rad(data.descriptor)
    parts = []
    for (a, b, target, thr), sc in zip(segs, scans):
        res = path_curvature(sc, geometry, b1, axis, workers=workers)
        if res is not None:
            parts.append(dict(res, t0=a, t1=b, throttle=thr, target_mrad=target))
    if not parts:
        raise AnalysisError('F1: no straight segment with odometry')
    kappa = combine_curvature(parts)
    L = b1['wheelbase_m']
    per = [math.degrees(math.atan(math.radians(p['heading_deg']) / p['path_m'] * L)) for p in parts]
    return dict(schema=SCHEMA, test='F1', zero_offset_deg=_r(math.degrees(math.atan(kappa * L))),
                curvature_1_m=_r(kappa, 6), curvature_deg_per_m=_r(math.degrees(kappa), 4),
                path_m=_r(sum(abs(p['path_m']) for p in parts), 2), segments=parts,
                per_segment_zero_offset_deg=[_r(v) for v in per],
                segment_spread_deg=_r(float(np.std(per)) if len(per) > 1 else None),
                windows=[(p['t0'], p['t1']) for p in parts], wheelbase_m=L,
                note='zero offset = atan(kappa L), kappa = sum heading sign(path) / sum |path| over the forward and '
                     'reverse passes (rear-axle path from the B1 lever arm)')


def f2_kappa_table(root, rid, b1, workers=1, every=SUBSAMPLE_SCANS):
    """F2: one row per steering target (the circles of the same target merged, path weighted): curvature from the
    LiDAR odometry over each circle after 3 s settling; 'runs' keeps every circle."""
    data = DriveData(root, rid)
    segs = [s for s in data.segments(min_s=8., settle_s=3.) if s[2] != 0. and s[3] > 0]
    scans, geometry = data.scans([(a, b) for a, b, _, _ in segs], every=every)
    axis = _axis_rad(data.descriptor)
    runs = []
    for (a, b, target, thr), sc in zip(segs, scans):
        res = path_curvature(sc, geometry, b1, axis, workers=workers)
        if res is None:
            continue
        idx = (data.st_t >= a) & (data.st_t <= b)
        runs.append(dict(target_mrad=target, kappa_1_m=_r(res['curvature_1_m'], 5), path_m=_r(res['path_m'], 2),
                         heading_deg=_r(res['heading_deg'], 3), speed_m_s=_r(res['speed_m_s'], 4), t0=a, t1=b,
                         pairs=res['pairs'], gaps=res['gaps'],
                         measured_pos_permille_median=_r(float(np.median(data.st_pos[idx])) if idx.any() else None, 1),
                         throttle=thr))
    table = []
    for target in sorted({r['target_mrad'] for r in runs}):
        group = [r for r in runs if r['target_mrad'] == target]
        kappa = combine_curvature(group)
        table.append(dict(target_mrad=target, target_deg=_r(math.degrees(target / 1000.), 3), kappa_1_m=_r(kappa, 5),
                          radius_m=_r(1. / kappa, 3), path_m=_r(sum(abs(r['path_m']) for r in group), 2),
                          circles=len(group), windows=[(r['t0'], r['t1']) for r in group],
                          speed_m_s=_r(float(np.mean([r['speed_m_s'] for r in group])), 4),
                          kappa_spread_1_m=_r(float(np.std([r['kappa_1_m'] for r in group])) if len(group) > 1
                                              else None, 5)))
    L = b1['wheelbase_m']
    fit = None
    if len(table) >= 2:
        x = np.array([math.tan(r['target_mrad'] / 1000.) / L for r in table])
        y = np.array([r['kappa_1_m'] for r in table])
        eta = float(x @ y / (x @ x))
        fit = dict(eta=_r(eta), residual_rms_1_m=_r(float(np.sqrt(np.mean((y - eta * x) ** 2))), 5))
    return dict(schema=SCHEMA, test='F2', table=table, runs=runs, fit=fit, wheelbase_m=L,
                note='kappa = LiDAR heading change / signed rear-axle path (B1 lever arm), path weighted over the '
                     'circles of one target; eta = LS of kappa on tan(target) / L through the origin (report)')


def f3_duty_speed(root, rid, workers=1, cache_dir=None):
    """F3 through the imported drive tools (drive_calib.analyse: C2, C3 v(duty), C7)."""
    tools_path()
    from field_calib import drive_calib
    recs, skipped = drive_calib.open_recordings(str(root), [rid], workers=1, hashes=False, cache_dir=cache_dir,
                                                log=lambda *a: None)
    results, _ = drive_calib.analyse(recs, workers=workers, log=lambda *a: None, skipped=skipped, cache_dir=cache_dir)
    table = {d: dict(v_p50_m_s=_r((row.get('v_p10_p50_p90') or [None, None, None])[1]),
                     segment_p50_m_s=_r((row.get('segment_speed_p10_p50_p90') or [None, None, None])[1]),
                     n_segments=row.get('n_segments'), samples=row.get('samples'))
             for d, row in results['c3']['table'].items()}
    lad = results['c2']['labels'].get('all', {}).get('ladder', {})
    c7 = results['c7']['kinds']
    return dict(schema=SCHEMA, test='F3', duty_speed=table, axis=results['axis'], lidar_imu=results['lidar_imu'],
                c2=dict(b_fwd_pct=lad.get('b_fwd_pct'), bracketed=lad.get('bracketed')),
                c7={k: dict(clean=v['clean'], distance_m_p50=(v['distance_m_p10_p50_p90'] or [None, None])[1])
                    for k, v in c7.items()},
                drive_result=drive_calib._json_safe(results))


def f4_f5_steering(root, rid, calibration):
    data = DriveData(root, rid)
    cal = Calibration.from_dict(calibration)
    if len(data.st_ms) < 10:
        return dict(schema=SCHEMA, test=data.descriptor.get('test'), rows=0)
    deg = cal.deg(data.st_pos)
    dt = np.diff(data.st_ms) / 1000.
    rate = np.abs(np.diff(deg)) / np.maximum(dt, 1e-3)
    busy = (np.abs(data.st_sp[1:]) > 0) & (dt > 0)
    target = cal.deg(data.st_tgt)
    err = np.abs(target - deg)
    faults = sorted({f for row in data.st_faults for f in row if f not in ('BOOT', 'SOFTLIMIT')})
    return dict(schema=SCHEMA, test=data.descriptor.get('test'), rows=int(len(deg)),
                slew_deg_s_p50=_r(float(np.median(rate[busy])) if busy.any() else None, 3),
                slew_deg_s_p90=_r(float(np.percentile(rate[busy], 90)) if busy.any() else None, 3),
                slew_deg_s_max=_r(float(np.max(rate[busy])) if busy.any() else None, 3),
                tracking_error_deg_p95=_r(float(np.percentile(err, 95)), 3), faults=faults,
                note='slew between consecutive status lines while the steering PWM is on (MCU limited unless the '
                     'actuator is slower); tracking = |MCU target - measured|')


# --- whole session ------------------------------------------------------------------------------------------------------
def analyse_session(session_dir, workers=1, log=print):
    """Every estimate of one session from its records (session.json names the records)."""
    session_dir = Path(session_dir)
    session = _load_json(session_dir / 'session.json')
    root = session_dir / session['root']
    bench = {k: str(root / v) for k, v in session['records']['bench'].items()}
    drive = session['records']['drive']
    out = dict(schema=SCHEMA, session=session.get('label'), seed=session.get('seed'))
    cal = b3_calibration(bench['B3'])
    out['B3'] = cal
    out['B0'] = b0_polarity(bench['B0'], bench.get('B2'))
    out['B2'] = b2_marks(bench['B2'], ackermann_ratio=cal['ackermann_ratio'])
    out['B4'] = b4_jog(bench['B4'], cal)
    out['B5'] = b5_steps(bench['B5'], cal, tau_s=out['B4']['tau_s'])
    out['B6'] = b6_backlash(bench['B6'], cal)
    out['B7'] = b7_noise({k: bench[n] for k, n in (('cold', 'B7_cold'), ('warm', 'B7_warm'), ('load', 'B7_load'))
                          if n in bench})
    b1 = BenchData(bench['B3']).b1()
    host_cal = session['calibration']
    if 'F5' in drive:
        out['F5'] = f4_f5_steering(root, drive['F5'], host_cal)
    if 'F4' in drive:
        out['F4'] = f4_f5_steering(root, drive['F4'], host_cal)
    if 'F1' in drive:
        log('    F1 odometry')
        out['F1'] = f1_zero_offset(root, drive['F1'], b1, workers=workers)
    if 'F2' in drive:
        log('    F2 odometry')
        out['F2'] = f2_kappa_table(root, drive['F2'], b1, workers=workers)
    if 'F3' in drive:
        log('    F3 drive tools')
        out['F3'] = f3_duty_speed(root, drive['F3'], workers=workers)
    return out
