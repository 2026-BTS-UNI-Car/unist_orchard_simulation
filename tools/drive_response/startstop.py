"""PW-04: start from rest, stop events, and duty x hold time -> distance.

Start events follow _archive/.../drive-evidence-scripts/start_onset_1010.py line by line (the 10/10 reference
values come from it): a non-zero request after >= 2 statuses (~1 s) with request 0 and reported duty 0 on both
sides, LiDAR still in between (max centred speed <= 0.05 m/s); the ramp start t0 is back-calculated from the
first status whose MCU-reported duty is strictly between 0 and the request (t0 = t_status - duty / 100 %/s;
'precise'); motion onset = first of 3 consecutive scans with LiDAR translation speed > 0.05 m/s (|yaw| > 5 deg/s
for pivots), searched from t0 - 0.3 s; the centred +-2 scan window makes the onset up to ~0.2 s early.
duty at onset = min(request, 100 %/s x (onset - t0)). Here it runs on both days (10/3 too), straight forward
requests only for the tables (pivots are listed in the event file but not used: the steered car has none).

Stop events follow resume-audit-20261013/sim-review/fix/from_release.py: request (0, 0) after a straight
forward request; the fade start (key release, MCU ramps the duty down at 100 %/s) is back-calculated from the
reported duty at that status or the next one; stop = first scan j with |p(j+5) - p(j)| / dt < 0.03 m/s;
distance along the travel axis from the fade start. 'clean' = no tag (state armed_manual, next request still 0,
forward request held >= ~1 s before). Hard stops (STOP button / disarm: duty set to 0 at once) are tagged by the
state; the fade start cannot be recovered for them.

Distance table: measured cells from the start events (distance along the travel axis from the ramp start after
T s, request still held at the last status >= T s after t0); model cells from a simple start model (ramp 100 %/s,
motion from the breakaway duty, first-order lag tau, steady speed from the speed table) - every model input is
listed in the output and the cells are marked 모델.
"""
import math

import numpy as np

from field_calib import timeline as tl

GOOD = dict(inlier=.5, rms=.08, cond=60.)
HOLD_TIMES = (1., 2., 5., 10.)
MIN_GOOD_FRAC = .5               # start curves with fewer ICP-good scans are listed but not used as measured cells
MODEL = dict(ramp_pct_per_s=100., tau_s=.25, release_hold_s=.25, release_decel=1.3)


def _signed_cls(l, r):
    return tl.classify(l, r)


def _lidar_speed_yaw(mot):
    """start_onset_1010.py: translation magnitude and yaw rate (deg/s) per scan, same validity rule."""
    t, pose = mot.t, mot.pose
    good = mot.good
    th = mot.theta
    n = len(t)
    spd = np.full(n, np.nan)
    yaw = np.full(n, np.nan)
    if n > 4:
        i = np.arange(2, n - 2)
        a, b = i - 2, i + 2
        dtt = t[b] - t[a]
        ok = good[a] & good[b] & good[i] & (dtt >= .25) & (dtt <= .6)
        ii, aa, bb, dd = i[ok], a[ok], b[ok], dtt[ok]
        spd[ii] = np.hypot(pose[bb, 0] - pose[aa, 0], pose[bb, 1] - pose[aa, 1]) / dd
        yaw[ii] = np.degrees(th[bb] - th[aa]) / dd
    return spd, yaw


def start_events(rec, mot):
    m = rec.motor
    if mot is None:
        return []
    t = mot.t
    spd, yaw = _lidar_speed_yaw(mot)
    mt = m.t
    rq = lambda j: (m.req_l[j], m.req_r[j])  # noqa: E731
    events = []
    for k in range(3, len(m)):
        l, r = rq(k)
        if (l, r) == (0, 0) or not (math.isfinite(l) and math.isfinite(r)):
            continue
        if any(rq(j) != (0, 0) for j in (k - 2, k - 1)):
            continue
        if any(abs(np.nan_to_num(m.rep_l[j])) > 0 or abs(np.nan_to_num(m.rep_r[j])) > 0 for j in (k - 2, k - 1)):
            continue
        pre = (t > mt[k - 2]) & (t < mt[k - 1])
        if not pre.any() or np.nanmax(np.nan_to_num(spd[pre], nan=0.)) > .05:
            continue
        cls = _signed_cls(l, r)
        t0 = None
        precise = False
        for j in range(k, min(k + 6, len(m))):
            if rq(j) != (l, r):
                break
            sl, sr = m.rep_l[j], m.rep_r[j]
            mx = max(abs(np.nan_to_num(sl)), abs(np.nan_to_num(sr)))
            if mx > 0:
                req_mx = max(abs(l), abs(r))
                t0 = mt[j] - mx / 100.
                precise = mx < req_mx - 2
                if not precise:
                    t0 = max(mt[j] - mx / 100., mt[j - 1]) if j > k else mt[j] - mx / 100.
                break
        if t0 is None:
            continue
        h = k
        while h + 1 < len(m) and rq(h + 1) == (l, r):
            h += 1
        t_end = mt[h] + .5
        w = (t >= t0 - .3) & (t <= min(t_end, t0 + 6.))
        idx = np.nonzero(w)[0]
        pivot = cls.startswith('pivot')
        sig = np.abs(yaw) if pivot else spd
        thr = 5. if pivot else .05
        on = None
        for ii in range(len(idx) - 2):
            s3 = sig[idx[ii:ii + 3]]
            if np.all(np.isfinite(s3)) and np.all(s3 > thr):
                on = t[idx[ii]]
                break
        ev = dict(rid=rec.rid, short=rec.short, day=rec.day, ground=rec.ground, cls=cls, req=[float(l), float(r)],
                  duty=int(round(max(abs(l), abs(r)))), t0=float(t0), t_rel=float(t0 - t[0]), precise=bool(precise),
                  hold_s=float(t_end - mt[k]), last_status_after_t0_s=float(mt[h] - t0))
        kst = rec.to_kst(t0)
        ev['kst'] = kst.strftime('%H:%M:%S') if kst else None
        if on is not None:
            lag = float(on - t0)
            ev['onset_s'] = lag
            ev['duty_at_onset'] = float(round(min(max(abs(l), abs(r)), max(0., lag * 100.)), 0))
            if not pivot:
                late = (t >= t0 + 2.) & (t <= min(t_end, t0 + 6.)) & np.isfinite(spd)
                if late.sum() >= 5:
                    vss = float(np.median(spd[late]))
                    ev['steady_mps'] = vss
                    for ii in idx:
                        if np.isfinite(spd[ii]) and spd[ii] >= .9 * vss and t[ii] >= on:
                            ev['t90_s'] = float(t[ii] - t0)
                            break
        else:
            ev['onset_s'] = None
        if not pivot and mot.axis is not None:
            ev['curve'] = _distance_curve(mot, t0, mt[h])
            span = (t >= t0) & (t <= t0 + (ev['curve'][-1][0] if ev['curve'] else 0.))
            # share of scans passing the ICP quality gate over the curve: below MIN_GOOD_FRAC the distances are 참고
            ev['good_frac'] = float(mot.good[span].mean()) if span.any() else 0.
            for T in HOLD_TIMES:
                ev['d_%g' % T] = _distance_at(mot, t0, T, mt[h])
        events.append(ev)
    return events


def _axis_at_time(mot, t0):
    i = int(np.clip(np.searchsorted(mot.t, t0), 0, len(mot.t) - 1))
    return mot.axis_vector(i)


def _distance_at(mot, t0, T, t_last_status):
    """Distance along the travel axis at t0 + T (None when the request did not last or the pose is missing)."""
    if t_last_status - t0 < T:
        return None
    p0 = mot.position_at(t0)
    p1 = mot.position_at(t0 + T)
    if p0 is None or p1 is None:
        return None
    return float((p1 - p0) @ _axis_at_time(mot, t0))


def _distance_curve(mot, t0, t_last_status, step=.1, horizon=10.):
    """[(tau, distance)] from the ramp start while the request lasts (<= horizon), pose gaps skipped."""
    p0 = mot.position_at(t0)
    if p0 is None:
        return []
    ax = _axis_at_time(mot, t0)
    out = []
    end = min(horizon, t_last_status - t0 + .5)
    for tau in np.arange(0., end + 1e-9, step):
        p = mot.position_at(t0 + tau)
        if p is not None:
            out.append((round(float(tau), 2), round(float((p - p0) @ ax), 4)))
    return out


def stop_events(rec, mot):
    m = rec.motor
    if mot is None:
        return []
    t, pose, good = mot.t, mot.pose, mot.good
    p = pose[:, :2]
    mt = m.t
    rq = lambda j: (m.req_l[j], m.req_r[j])  # noqa: E731
    out = []
    for k in range(1, len(m) - 2):
        prev, cur = rq(k - 1), rq(k)
        if prev == cur or cur != (0, 0):
            continue
        if not (prev[0] == prev[1] and prev[0] > 0):
            continue
        t0 = mt[k]
        i0 = int(np.searchsorted(t, t0))
        if i0 < 5 or i0 + 40 >= len(t) or not good[i0 - 5:i0 + 40].mean() > .8:
            continue
        tags = []
        if m.state[k] != 'armed_manual':
            tags.append('state=%s' % m.state[k])
        if rq(k + 1) != (0, 0):
            tags.append('next request %s' % (tuple(int(x) for x in rq(k + 1)),))
        if rq(k - 2) != prev:
            tags.append('forward request held < ~0.5-1 s (from %s)' % (tuple(int(x) for x in rq(k - 2)),))
        prev_duty = float(prev[0])
        d0 = .5 * (m.rep_l[k] + m.rep_r[k])
        d1 = .5 * (m.rep_l[k + 1] + m.rep_r[k + 1])
        # from_release.py used 100 % as the start of the fade (all its events came from 100 %); generalised to the
        # previous settled duty, identical for 100 %.
        top = prev_duty - .5
        if 0 < d0 < top:
            t_r = t0 - (prev_duty - d0) / 100.
            basis = 'duty %.1f%% at t0' % d0
        elif 0 < d1 < top and d0 >= top:
            t_r = mt[k + 1] - (prev_duty - d1) / 100.
            basis = 'duty %.0f%% at t0, %.1f%% at next status' % (d0, d1)
        else:
            t_r = None
            basis = 'duty %.1f%%/%.1f%%: fade start not recoverable' % (d0, d1)
        ax = mot.axis_vector(i0)
        start = int(np.searchsorted(t, t_r)) if t_r is not None else i0
        stop_i = None
        for j in range(start, i0 + 40):
            if j + 5 < len(t) and np.all(np.isfinite(p[j:j + 6, 0])):
                if np.linalg.norm(p[j + 5] - p[j]) / (t[j + 5] - t[j]) < .03:
                    stop_i = j
                    break
        if m.state[k] != 'armed_manual':
            kind = 'STOP/해제'
        elif t_r is not None:
            kind = '놓기'
        elif (abs(d0) < .5 and k >= 1 and .5 * (m.rep_l[k - 1] + m.rep_r[k - 1]) > top
              and mt[k] - mt[k - 1] < prev_duty / 100.):
            kind = '즉시 0(STOP 계열 추정)'      # the duty fell faster than the 100 %/s fade allows
        else:
            kind = '알 수 없음'
        ev = dict(rid=rec.rid, short=rec.short, day=rec.day, ground=rec.ground, prev_duty=int(prev_duty),
                  kind=kind, tags=tags, basis=basis, clean=(not tags and t_r is not None and stop_i is not None),
                  lag_s=None if t_r is None else float(t0 - t_r))
        kst = rec.to_kst(t0)
        ev['kst'] = kst.strftime('%H:%M:%S') if kst else None
        if stop_i is not None:
            ev['dist_from_first_zero_status_m'] = float((p[stop_i] - p[i0]) @ ax)
            ev['time_from_first_zero_status_s'] = float(t[stop_i] - t0)
        if t_r is not None and stop_i is not None:
            pr = np.array([np.interp(t_r, t, p[:, 0]), np.interp(t_r, t, p[:, 1])])
            j5 = int(np.searchsorted(t, t_r))
            ev['speed_at_release'] = float((p[j5] - p[j5 - 5]) @ ax / (t[j5] - t[j5 - 5]))
            ev['dist_m'] = float((p[stop_i] - pr) @ ax)
            ev['time_s'] = float(t[stop_i] - t_r)
            ev['model_dist_m'] = release_model(ev['speed_at_release'])
            before = mot.valid & (t >= t_r - 1.) & (t <= t_r)
            if before.sum() >= 3:
                ev['speed_1s_before'] = float(np.median(mot.u[before]))
                ev['speed_consistent'] = abs(ev['speed_1s_before'] - ev['speed_at_release']) < .15
        out.append(ev)
    return out


def release_model(v):
    """Key-release stop model of the SIM plant (speed held release_hold_s, then constant deceleration)."""
    v = max(v, 0.)
    return v * MODEL['release_hold_s'] + v * v / (2. * MODEL['release_decel'])


def model_distance(duty, seconds, v_of_duty, breakaway, tau=MODEL['tau_s'], ramp=MODEL['ramp_pct_per_s'], dt=.005):
    """Distance from rest after `seconds` for a duty request: ramp, start at the breakaway duty, lag tau.
    v_of_duty(d) gives the steady speed (from the speed table). None when the duty never reaches the breakaway."""
    if duty < breakaway:
        return None
    t = d = v = x = 0.
    moving = False
    while t < seconds - 1e-12:
        d = min(duty, d + ramp * dt)
        if not moving and d >= breakaway:
            moving = True
        target = v_of_duty(d) if moving else 0.
        v += (target - v) * dt / tau
        x += v * dt
        t += dt
    return x
