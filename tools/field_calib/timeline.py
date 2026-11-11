"""Motor status timeline helpers (no I/O): request runs, settled samples, ramp back-calculation, windows.

The bridge publishes /rover/status every 0.5 s (rover_control_bridge.py), so every command time read
from motor.jsonl is uncertain by up to ~0.5 s. The firmware ramp is deterministic (255/s = 100 %/s per
side, wheel_pwm.ino rampWheel; drive(0,0) and key release fade the same way; STOP / input expiry set the
duty to 0 at once), so a status that shows a duty strictly between the old and the new value gives the
ramp start to ~0.1 s (t_ramp = t_status - |change so far| / 100). The MCU report age (0-0.1 s) makes
that estimate slightly late.
"""
import math

import numpy as np

RAMP_PCT_PER_S = 100.
STATUS_PERIOD_S = .5
SETTLED_TOL_PCT = 2.


def classify(l, r):
    """lidar_response.py command classes."""
    if l is None or r is None or (isinstance(l, float) and math.isnan(l)) or (isinstance(r, float) and math.isnan(r)):
        return 'unknown'
    if l == 0 and r == 0:
        return 'stop'
    if l == r:
        return 'straight_fwd' if l > 0 else 'straight_rev'
    if l < 0 < r:
        return 'pivot_left'
    if r < 0 < l:
        return 'pivot_right'
    if 0 < l < r:
        return 'arc_left_fwd'
    if 0 < r < l:
        return 'arc_right_fwd'
    return 'other'


def status_index(motor, t):
    """Index of the latest status received at or before each t (-1 = none)."""
    return np.searchsorted(motor.t, t, side='right') - 1


def is_settled(motor, k, tol=SETTLED_TOL_PCT):
    if k < 0:
        return False
    l, r, a, b = motor.req_l[k], motor.req_r[k], motor.rep_l[k], motor.rep_r[k]
    if not all(math.isfinite(x) for x in (l, r, a, b)):
        return False
    return abs(a - l) <= tol and abs(b - r) <= tol


def reported_zero(motor, k):
    a, b = motor.rep_l[k], motor.rep_r[k]
    return math.isfinite(a) and math.isfinite(b) and abs(a) < .5 and abs(b) < .5


def request_zero(motor, k):
    return motor.req_l[k] == 0 and motor.req_r[k] == 0


def request_runs(motor):
    """Maximal runs of statuses with the same (req_l, req_r): dicts k0, k1, l, r, t0, t_last, t_next."""
    runs = []
    n = len(motor)
    i = 0
    while i < n:
        l, r = motor.req_l[i], motor.req_r[i]
        j = i
        while j + 1 < n and _same(motor.req_l[j + 1], l) and _same(motor.req_r[j + 1], r):
            j += 1
        t_next = motor.t[j + 1] if j + 1 < n else motor.t[j] + STATUS_PERIOD_S
        runs.append(dict(k0=i, k1=j, l=l, r=r, t0=float(motor.t[i]), t_last=float(motor.t[j]),
                         t_next=float(t_next), cls=classify(l, r)))
        i = j + 1
    return runs


def _same(a, b):
    return (a == b) or (isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b))


def ramp_start_from_rest(motor, run):
    """Ramp start after a request from rest: (t0, precise, basis). None when no non-zero report arrives."""
    req = max(abs(run['l']), abs(run['r']))
    for j in range(run['k0'], min(run['k1'] + 1, run['k0'] + 6)):
        a, b = motor.rep_l[j], motor.rep_r[j]
        if not (math.isfinite(a) and math.isfinite(b)):
            continue
        mx = max(abs(a), abs(b))
        if mx > 0:
            t0 = motor.t[j] - mx / RAMP_PCT_PER_S
            if mx < req - SETTLED_TOL_PCT:
                return float(t0), True, 'duty %.1f%% of %d%% at a status' % (mx, req)
            earliest = motor.t[j - 1] if j > 0 else t0
            return float(max(t0, earliest)), False, 'ramp already complete at the first non-zero status'
    return None


def fade_start(motor, k, previous_duty):
    """Request-to-zero at status k: estimated fade start from the MCU-reported fading duty.

    Returns (t, basis) or (None, basis). previous_duty = settled duty before the change (mean of sides).
    from_release.py rule generalised from 100 % to any previous duty."""
    for j in (k, k + 1):
        if j >= len(motor):
            break
        d = .5 * (abs(motor.rep_l[j]) + abs(motor.rep_r[j]))
        if not math.isfinite(d):
            continue
        if 0 < d < previous_duty - .5:
            return float(motor.t[j] - (previous_duty - d) / RAMP_PCT_PER_S), 'duty %.1f%% at status %+d' % (d, j - k)
        if d < .5:
            return None, 'reported 0 already at status %+d' % (j - k)
    return None, 'fade not visible in the reported duty'


def interest_windows(motor, pre_s=3., post_s=6., merge_gap_s=2., still_min_s=10., still_cap_s=40.):
    """Scan time windows worth matching: around every non-zero request / non-zero reported duty, plus
    the first still_cap_s of every standstill of >= still_min_s (C1). Merged, sorted, in seconds."""
    if len(motor) == 0:
        return []
    active = np.zeros(len(motor), dtype=bool)
    for k in range(len(motor)):
        req = (math.isfinite(motor.req_l[k]) and motor.req_l[k] != 0) or \
              (math.isfinite(motor.req_r[k]) and motor.req_r[k] != 0)
        rep = not reported_zero(motor, k) and (math.isfinite(motor.rep_l[k]) or math.isfinite(motor.rep_r[k]))
        active[k] = req or rep
    windows = []
    k = 0
    n = len(motor)
    while k < n:
        if not active[k]:
            k += 1
            continue
        j = k
        while j + 1 < n and active[j + 1]:
            j += 1
        windows.append([motor.t[k] - pre_s, motor.t[j] + post_s])
        k = j + 1
    for a, b in still_windows(motor, min_s=still_min_s, head_s=0., tail_s=0.):
        windows.append([a, min(b, a + still_cap_s)])
    windows.sort()
    merged = []
    for a, b in windows:
        if merged and a <= merged[-1][1] + merge_gap_s:
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    return [(float(a), float(b)) for a, b in merged]


def still_windows(motor, min_s=10., head_s=2., tail_s=0., max_gap_s=1.5):
    """Status runs with request 0/0 and reported duty 0 on both sides (C1). [(t_from, t_to)] of >= min_s."""
    out = []
    n = len(motor)
    k = 0
    while k < n:
        if not (request_zero(motor, k) and reported_zero(motor, k)):
            k += 1
            continue
        j = k
        while (j + 1 < n and request_zero(motor, j + 1) and reported_zero(motor, j + 1)
               and motor.t[j + 1] - motor.t[j] <= max_gap_s):
            j += 1
        a, b = motor.t[k] + head_s, motor.t[j] - tail_s
        if b - a >= min_s:
            out.append((float(a), float(b)))
        k = j + 1
    return out


def scan_windows_to_indices(scan_t, windows):
    """[(first, last_exclusive)] scan index ranges covered by time windows (sorted, non-overlapping)."""
    out = []
    for a, b in windows:
        i0 = int(np.searchsorted(scan_t, a, side='left'))
        i1 = int(np.searchsorted(scan_t, b, side='right'))
        if i1 - i0 >= 2:
            if out and i0 <= out[-1][1]:
                out[-1] = (out[-1][0], max(out[-1][1], i1))
            else:
                out.append((i0, i1))
    return out
