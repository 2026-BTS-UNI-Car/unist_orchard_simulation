"""C1 gate: OPEN-LOOP shadow replay of the SIM RowController on real 2026-10-10 manual orchard records.

No motor, no ROS, no Jetson. Read-only on the records. The vendored SIM estimator/controller (vendor/,
vendor/SOURCE_MANIFEST.json) run unmodified, loaded by core.load_vendor() (sha256-verified); only
executor-level inputs and configs change.

Inputs to the estimator (nothing else), converted by the EXECUTOR's own core.ScanAdapter / core.ImuYawRate
(row_executor/core.py; --spec '{"adapter": "replay"}' selects the equivalent replay_adapters copies): the real
scans rotated by the LiDAR mount offset (+179.08 deg so +x = travel), 0/invalid -> inf, an optional
leave-one-out per-beam body mask (body_mask_fit.py); the gravity-projected IMU yaw rate (gyro . up, accel EMA
tau 1 s) and raw accel. Stamps: scan = receive - 0.08 s, IMU = receive. Controller ticks at 20 Hz on the
receive clock.

Runs (what an operator would do with the hold-to-run executor):
  standing  the human starts driving forward after >= 1 s of stillness (the task definition). A new
            estimator starts at onset - min(stillness, 3 s) (Start pressed while still), the controller
            (hold pressed) at onset - 0.5 s; it must reach FOLLOW by onset + 0.5 s.
  flying    (secondary, labelled) every other forward segment (e.g. right after a manual pivot). New
            estimator at onset - 2 s, controller at onset; stillness is INJECTED into the controller's
            snapshot only while it is in REACQUIRE, for at most 1 s, so it can accept the corridor while the
            human is already moving. Nothing else is altered.
  A run ends when the human stops driving forward (reference ICP) or reverses.
Open loop: the commands are never applied; the human's motion continues regardless. Therefore only the
FIRST stop decision of a controller in a run is meaningful (the real vehicle would stop there); the run is
scored up to that point. drive_no_response cannot be judged open loop (the human, not the command, moves).

Variants: estimator config overrides, controller overrides, mask, an opt-in near-speckle filter, and a
'drop scans that arrive while busy' timing mode (virtual clock = measured processing time x scale; only the
newest pending scan is processed; ticks are delayed while busy; IMU is loaded before and after on_scan).

Usage: python -I -B row_shadow_replay.py --variant base --out-dir <runs folder outside the repository>
       [--records ...] [--starts standing flying] [--workers 4] [--est KEY=VAL ...] [--ctl KEY=VAL ...]
       [--name NAME] [--mask-dir DIR]
External locations (steering branch, docs/steering/REPLAY_BASELINE_KO.md): the 10/10 records come from sim_paths.py
('field_manual_1010', 'field_manual_1010_drive'); the leave-one-out body masks are written to --mask-dir (default
<out-dir>/masks); new work waits while the HOLD_MEASUREMENT file of the configured hold folder exists
(sim_paths.measurement_hold_file(): the SIM tree resolver's 'hold_dir'; a copy without the SIM tree has no hold file).
"""
import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))     # tools/ -> the row_executor package (executor core)
sys.path.insert(0, str(HERE))
from row_executor import core as X       # noqa: E402  executor core: adapters + hash-verified vendor loader
import field_records as F                # noqa: E402
import sim_paths as P                    # noqa: E402  (HOLD_MEASUREMENT file of the configured hold folder)
import replay_adapters as A              # noqa: E402

E, C = X.load_vendor()                   # vendored SIM estimator/controller, sha256-verified, unmodified

TICK_S = .05
STANDING_EST_LEAD_MAX_S = 3.
STANDING_HOLD_LEAD_S = .5
STANDING_ACCEPT_S = .5
FLYING_EST_LEAD_S = 2.
FLYING_INJECT_S = 1.
AT_END_M = 3.             # a stop with less than this straight human driving left is "at a turn / segment end"

WEEKEND = dict(base_duty_pct=55, reduce_duty_pct=45, max_differential_pct=20, arc_yaw_deg_s_per_pct_diff=.15)
CONTROLLERS = {
    'continuous/sim': ('continuous', {}),
    'continuous/weekend': ('continuous', WEEKEND),
    'continuous_side/sim': ('continuous_side', {}),
    'continuous_side/weekend': ('continuous_side', WEEKEND),
}
VARIANTS = {
    'base': dict(),
    'n3': dict(est=dict(corridor_every_n_scans=3)),
    'drop': dict(drop=True),
    'drop_n3': dict(est=dict(corridor_every_n_scans=3), drop=True),
    'drop_x3': dict(drop=True, scale=3.),
    'drop_n3_x3': dict(est=dict(corridor_every_n_scans=3), drop=True, scale=3.),
    'nomask': dict(mask='none'),
    'speckle': dict(speckle=dict(near_m=1.2, window=2, min_neighbours=1, tolerance_m=.15)),
    'transient': dict(transient=dict(near_m=2., tol_m=.35)),
}
STATES = ('REACQUIRE', 'FOLLOW', 'REDUCE', 'STOP_REQUESTED', 'STOP_OBSERVED', 'TURN', 'SETTLE', 'HOLD')


def wait_for_measurement_hold():
    hold = P.measurement_hold_file()
    while hold is not None and hold.exists():
        print('HOLD_MEASUREMENT present, waiting 60 s', flush=True)
        time.sleep(60)


class Codes:
    def __init__(self):
        self.names = []

    def __call__(self, name):
        if name not in self.names:
            self.names.append(name)
        return self.names.index(name)


def nan_or(v):
    return np.nan if v is None else float(v)


def parse_kv(items):
    out = {}
    for item in items or []:
        key, value = item.split('=', 1)
        out[key] = json.loads(value) if value not in ('None', 'none', 'null') else None
    return out


def make_adapter(variant, rid, mask_dir):
    mask = None
    mode = variant.get('mask', 'loo')
    if mode == 'loo':
        import body_mask_fit as M
        path = Path(mask_dir) / ('mask_loo_%s.json' % rid[9:15])
        if not path.exists():
            result = M.fit_mask([r for r in F.RECORDS_1010 if r != rid])
            result['held_out'] = [rid]
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, indent=1) + '\n', encoding='utf-8')
        mask, _ = M.load_mask(path)
    elif mode not in (None, 'none'):
        import body_mask_fit as M
        mask, _ = M.load_mask(mode)
    speckle = variant.get('speckle')
    masked = None if mask is None else int((mask > 0).sum())
    if variant.get('adapter', 'core') == 'replay':
        return A.ScanAdapter(mask_r=mask, speckle=None if not speckle else A.NearSpeckleFilter(**speckle)), masked
    return CoreScanAdapter(mask, None if not speckle else A.NearSpeckleFilter(**speckle)), masked


def mask_segments(mask):
    """Per-beam mask radii -> core.ScanAdapter body_mask segments (first, last, max_range)."""
    out = []
    if mask is None:
        return out
    k = 0
    while k < len(mask):
        if mask[k] > 0:
            j = k
            while j + 1 < len(mask) and mask[j + 1] == mask[k]:
                j += 1
            out.append((k, j, float(mask[k])))
            k = j + 1
        else:
            k += 1
    return out


class CoreScanAdapter:
    """The executor's own core.ScanAdapter (+ the opt-in replay filters applied after it)."""
    def __init__(self, mask, speckle):
        self.core = X.ScanAdapter(yaw_offset_deg=A.LIDAR_YAW_OFFSET_DEG, body_mask=mask_segments(mask))
        self.speckle = speckle
        self.transient = None

    def convert(self, ranges, angle_min, increment, range_max):
        r, amin, inc, rmax, st = self.core.convert(ranges, angle_min, increment, range_max)
        st = dict(st, speckle=0, transient=0, mask_mismatch=False)
        if self.speckle is not None:
            st['speckle'] = self.speckle.apply(r)
        if self.transient is not None:
            # same frame on every scan: the filter only compares near returns between consecutive scans
            st['transient'] = self.transient.apply(r, amin, inc)
        return r, amin, inc, rmax, st


def geometry_guard(guard, snap, cfg):
    """Executor-level sanity stop while driving (opt-in): the band says the vehicle is not aligned with / inside
    its corridor. Continuous mode has no large-error limit in FOLLOW (it would keep a saturated arc), and the
    tracked band can stay 'valid' with heading errors up to 180 deg after the vehicle turned (10/10 replay)."""
    if not snap['corridor_valid']:
        return None
    if abs(math.degrees(snap['heading_error'])) > guard.get('heading_deg', 25.):
        return 'guard_heading'
    half = .5 * snap['corridor_width'] - guard.get('body_half_width_m', .35)
    if abs(snap['lateral_offset']) > half + guard.get('lateral_margin_m', 0.):
        return 'guard_lateral'
    return None


def scan_stamps(rec, variant, latency):
    """Estimator scan stamps on the receive clock.

    'receipt' (default, WEEKEND_PLAN 1.3): receive - latency. Bursty delivery (several scans received within a
      few ms after a hiccup; 10/03 records: 55-76 receive intervals < 30 ms, minimum 0 ms) gives near-zero dt
      and the estimator's speed (median of step/dt) explodes.
    'header_offset' (opt-in, FEASIBILITY_CRITIQUE M3): header + causal running median over the last 50 scans
      of (receive - header) - latency. Keeps the driver's regular 0.1 s spacing; a header clock step (GNSS
      sync) moves the offset median within ~25 scans."""
    t_r, t_h = rec['t_scan'], rec['t_header']
    if variant.get('stamp', 'receipt') == 'receipt':
        return t_r - latency
    window = int(variant.get('stamp_window', 50))
    d = t_r - t_h
    out = np.empty(len(t_r))
    for k in range(len(t_r)):
        out[k] = t_h[k] + float(np.median(d[max(0, k - window + 1):k + 1])) - latency
    return out


def fresh_transient(variant):
    spec = variant.get('transient')
    return None if not spec else A.NearTransientFilter(**spec)


def imu_yaw_rates(imu):
    """core.ImuYawRate over the whole record (the executor process keeps it across runs)."""
    order = np.argsort(imu[:, 0], kind='stable')
    yaw = X.ImuYawRate()
    w = np.full(len(imu), np.nan)
    for k in order:
        out = yaw.update(imu[k, 0] / 1e9, imu[k, 2:5], imu[k, 5:8])
        if out is not None:
            w[k] = out[0]
    st = yaw.state
    tilt = None if st is None else math.degrees(math.acos(max(-1., min(1., st[2] / math.sqrt(sum(v * v for v in st))))))
    return order, w, tilt


class ControllerRun:
    """One controller instance for one run, scored until its first stop decision.

    RowController keeps its own timers relative to since = 0. (set in __init__); an absolute clock (receive
    time here, time.monotonic() on the vehicle) makes REACQUIRE time out on the first tick
    (corridor_not_acquired). The controller therefore gets a RUN-RELATIVE clock (now - creation time); the
    estimator keeps the absolute stamps (its ages are computed internally). The executor must do the same.
    """
    def __init__(self, name, mode, overrides, t0):
        self.t0 = t0
        self.name = name
        self.ctl = C.RowController(C.config_for(mode, **overrides))
        self.cfg = self.ctl.config
        self.active = True
        self.follow_t = None
        self.follow_d = None
        self.stop = None
        self.start_fail = None
        self.ticks_follow = 0
        self.fwd_ticks_follow = 0
        self.correcting = 0
        self.agree = 0
        self.saturated = 0
        self.max_sat_run = 0
        self._sat_run = 0
        self.row_end_posted = 0
        self.row_end_early = 0
        self.max_abs_lateral = 0.
        self.lateral = []
        self.heading = []
        self.guard_reason = None

    def step(self, now, snap, inject):
        s = snap
        if inject and self.ctl.state == 'REACQUIRE':
            s = dict(snap, still=True, still_s=max(snap['still_s'], 1.))
        return self.ctl.step(now - self.t0, s)


def run_one(rec, h, seg, kind, variant, adapter, imu_order, w_imu, codes, log, timing):
    t_on, t_off = seg['t_on'], seg['t_off']
    t0_rec = rec['t_scan'][0]
    if kind == 'standing':
        t_est = t_on - min(seg['still_s'], STANDING_EST_LEAD_MAX_S)
        t_hold = max(t_est, t_on - STANDING_HOLD_LEAD_S)
        inject_until = None
        accept_by = t_on + STANDING_ACCEPT_S
    else:
        t_est = t_on - FLYING_EST_LEAD_S
        t_hold = t_on
        inject_until = t_on + FLYING_INJECT_S
        accept_by = inject_until
    t_est = max(t_est, t0_rec - 1e-6)
    guard = variant.get('guard')
    est = E.SensorEstimator(E.EstimatorConfig(**variant.get('est', {})))
    adapter.transient = fresh_transient(variant)          # per-run state, like the estimator
    ctl_overrides = variant.get('ctl', {})
    runs = [ControllerRun(name, mode, {**ov, **ctl_overrides}, t_hold) for name, (mode, ov) in CONTROLLERS.items()]
    imu = rec['imu']
    t_imu = imu[imu_order, 0] / 1e9
    w_sorted = w_imu[imu_order]
    acc_sorted = imu[imu_order, 5:8]
    ji = int(np.searchsorted(t_imu, t_est, side='left'))
    ts = rec['t_scan']
    ks = int(np.searchsorted(ts, t_est, side='left'))
    k_end = int(np.searchsorted(ts, t_off, side='right'))
    latency = variant.get('scan_latency_s', A.SCAN_LATENCY_S)
    stamps = scan_stamps(rec, variant, latency)
    amin, inc, rmax = rec['angle_min'], rec['increment'], rec['range_max']
    i_on = seg['i0']
    d_on = h['dist'][i_on]
    k_grid0 = int(round(h['t'][0] / TICK_S))          # ticks are integer multiples of TICK_S (no float drift)
    scan_times, tick_costs, tick_delays = [], [], []
    last_now = [None]
    stats = dict(scans_processed=0, scans_dropped=0, masked=0, trimmed=0, speckle=0, transient=0, mask_mismatch=0,
                 stall_ticks=0,
                 ticks=0, stale_lidar_ticks=0, stale_imu_ticks=0)

    def feed_imu(until):
        nonlocal ji
        while ji < len(t_imu) and t_imu[ji] <= until:
            if math.isfinite(w_sorted[ji]):
                est.on_imu(float(t_imu[ji]), float(w_sorted[ji]), tuple(acc_sorted[ji]))
            ji += 1

    def process_scan(k):
        c0 = time.perf_counter()
        try:
            r, a0, inc_, rmax_, st = adapter.convert(rec['ranges_mm'][k].astype(np.float64) / 1000., amin, inc, rmax)
        except X.ScanRejected as exc:
            stats['rejected'] = stats.get('rejected', 0) + 1
            stats['rejected_reason'] = exc.reason
            return 0.
        est.on_scan(float(stamps[k]), r, a0, inc_, rmax_)
        c = time.perf_counter() - c0
        stats['scans_processed'] += 1
        for key in ('masked', 'trimmed', 'speckle', 'transient'):
            stats[key] += st[key]
        stats['mask_mismatch'] += int(st['mask_mismatch'])
        scan_times.append(c)
        return c

    def tick(now, t_grid):
        c0 = time.perf_counter()
        snap = est.snapshot(now)
        stats['ticks'] += 1
        cfg0 = runs[0].cfg
        stats['stale_lidar_ticks'] += int(snap['lidar_age_s'] > cfg0.lidar_max_age_s)
        stats['stale_imu_ticks'] += int(snap['imu_age_s'] > cfg0.imu_max_age_s)
        i = int(round(t_grid / TICK_S)) - k_grid0
        i = i if 0 <= i < len(h['t']) else None
        human_fwd = bool(h['forward'][i]) if i is not None else False
        d_now = float(h['dist'][i]) if i is not None else math.nan
        inject = inject_until is not None and now <= inject_until
        row = [now - t_on, codes('kind:' + kind), nan_or(h['vf'][i] if i is not None else None), d_now - d_on,
               float(human_fwd), snap['lidar_age_s'], snap['imu_age_s'], snap['speed'], float(snap['corridor_valid']),
               codes(snap['corridor_reason']), nan_or(snap.get('heading_error')), nan_or(snap.get('lateral_offset')),
               nan_or(snap.get('corridor_width')), nan_or(snap.get('row_end_ahead_m')), snap['front_clearance_m'],
               snap['side_clearance_left_m'], snap['side_clearance_right_m'], float(snap['still']),
               snap['distance_m'], nan_or(snap.get('corridor_age_s'))]
        first = True
        # Runner rule (core.Runner.tick): a loop gap > stall_guard_s while ACTIVE pauses the run; motion
        # restarts only on a new hold press -> scored as a stop. Only the drop mode has compute time.
        stall = (bool(variant.get('drop')) and last_now[0] is not None
                 and now - last_now[0] > variant.get('stall_guard_s', .35))
        stats['stall_ticks'] += int(stall)
        last_now[0] = now
        for run in runs:
            if not run.active:
                row += [codes('state:-'), codes('reason:-'), 0, 0, np.nan]
                continue
            cmd = run.step(now, snap, inject)
            if run.follow_t is not None and run.ctl.state in C.DRIVING_STATES:
                why = 'stall_guard' if stall else (geometry_guard(guard, snap, run.cfg) if guard else None)
                if why:
                    cmd = C.Command(0, 0, 'HOLD', why)        # executor-level stop (the controller is not edited)
                    run.guard_reason = why
            if first:
                tick_costs.append(time.perf_counter() - c0)
                first = False
            terr = math.degrees(run.ctl.target_error(snap)) if snap['corridor_valid'] else math.nan
            row += [codes('state:' + cmd.state), codes('reason:' + cmd.reason), cmd.left, cmd.right, terr]
            score(run, cmd, snap, now, i, human_fwd, d_now, terr, accept_by)
        log.append(row)

    def score(run, cmd, snap, now, i, human_fwd, d_now, terr, accept_by_):
        state = run.ctl.state if run.guard_reason is None else 'HOLD'
        if run.follow_t is None:
            if state in C.DRIVING_STATES:
                run.follow_t, run.follow_d = now - t_on, d_now
            elif state == 'HOLD' or now > accept_by_:
                run.start_fail = dict(t_rel=round(now - t_on, 3), state=state, reason=run.ctl.reason,
                                      corridor_valid=bool(snap['corridor_valid']),
                                      corridor_reason=snap['corridor_reason'], still=bool(snap['still']),
                                      front_clearance_m=round(snap['front_clearance_m'], 3))
                run.active = False
                return
            else:
                return
        if state in C.DRIVING_STATES:
            run.ticks_follow += 1
            run.fwd_ticks_follow += int(human_fwd)
            if cmd.left != cmd.right:
                run.correcting += 1
                if math.isfinite(terr) and terr != 0 and (cmd.right - cmd.left) * terr > 0:
                    run.agree += 1
            limit = min(run.cfg.max_differential_pct, 100 - run.cfg.base_duty_pct, run.cfg.base_duty_pct)
            if abs(cmd.right - cmd.left) >= 2 * limit:
                run.saturated += 1
                run._sat_run += 1
                run.max_sat_run = max(run.max_sat_run, run._sat_run)
            else:
                run._sat_run = 0
            lat = snap.get('lateral_offset')
            if lat is not None:
                run.lateral.append(lat)
                run.heading.append(math.degrees(snap['heading_error']))
            end = snap.get('row_end_ahead_m')
            if end is not None and i is not None:
                run.row_end_posted += 1
                ahead = F.straight_ahead_m(h, i)
                if ahead is not None and ahead > end + AT_END_M:
                    run.row_end_early += 1
            return
        # first stop decision of this run
        kind_ = run.ctl.pending[0] if state == 'STOP_REQUESTED' and run.ctl.pending else 'hold'
        ahead = F.straight_ahead_m(h, i) if i is not None else None
        remaining = float(h['dist'][seg['i1']] - d_now) if i is not None else None
        at_end = (remaining is not None and remaining < AT_END_M) or (ahead is not None and ahead < AT_END_M)
        if run.guard_reason is not None:
            kind_ = 'executor_guard'
        run.stop = dict(t_rel=round(now - t_on, 3), state=state,
                        reason=run.ctl.reason if run.guard_reason is None else run.guard_reason, pending=kind_,
                        dist_from_follow_m=round(d_now - run.follow_d, 3) if math.isfinite(d_now) else None,
                        dist_from_onset_m=round(d_now - d_on, 3) if math.isfinite(d_now) else None,
                        segment_remaining_m=None if remaining is None else round(remaining, 3),
                        straight_ahead_m=None if ahead is None else round(ahead, 3), at_turn_or_end=bool(at_end),
                        est_distance_m=round(snap['distance_m'], 3), speed=round(snap['speed'], 3),
                        row_end_ahead_m=snap.get('row_end_ahead_m'), front_clearance_m=round(snap['front_clearance_m'], 3),
                        side_left_m=round(snap['side_clearance_left_m'], 3), side_right_m=round(snap['side_clearance_right_m'], 3),
                        corridor_valid=bool(snap['corridor_valid']), corridor_reason=snap['corridor_reason'],
                        corridor_age_s=None if snap.get('corridor_age_s') is None else round(snap['corridor_age_s'], 3),
                        lidar_age_s=round(snap['lidar_age_s'], 3), imu_age_s=round(snap['imu_age_s'], 3),
                        human_vf=nan_or(h['vf'][i]) if i is not None else None)
        run.active = False

    # ---- main loop -------------------------------------------------------------------------------------
    k_first = math.ceil(t_est / TICK_S - 1e-6)
    k_last = math.floor(t_off / TICK_S + 1e-6)
    if not variant.get('drop'):
        events = []
        j = ji
        while j < len(t_imu) and t_imu[j] <= t_off:
            events.append((float(t_imu[j]), 0, j))
            j += 1
        for k in range(ks, k_end):
            events.append((float(ts[k]), 1, k))
        for kt in range(k_first, k_last + 1):
            events.append((kt * TICK_S, 2, -1))
        events.sort()
        for when, typ, idx in events:
            if typ == 0:
                if math.isfinite(w_sorted[idx]):
                    est.on_imu(float(t_imu[idx]), float(w_sorted[idx]), tuple(acc_sorted[idx]))
                ji = idx + 1
            elif typ == 1:
                process_scan(idx)
            elif when >= t_hold - 1e-9:
                if not any(r.active for r in runs):
                    break
                tick(when, when)
    else:
        scale = float(variant.get('scale', 1.))
        busy = t_est
        kt = k_first
        tick_t = kt * TICK_S
        k = ks
        while kt <= k_last and any(r.active for r in runs):
            t = max(tick_t, busy)
            tick_delays.append(t - tick_t)
            feed_imu(t)
            last = int(np.searchsorted(ts, t, side='right')) - 1
            c = 0.
            if last >= k and last < k_end:
                stats['scans_dropped'] += last - k
                c = process_scan(last) * scale
                k = last + 1
                feed_imu(t + c)          # executor re-loads the IMU queue before the snapshot
            now = t + c
            if now >= t_hold - 1e-9:
                c1 = time.perf_counter()
                tick(now, tick_t)
                now += (time.perf_counter() - c1) * scale
            busy = now
            kt = max(kt + 1, math.floor(t / TICK_S + 1e-6) + 1)
            tick_t = kt * TICK_S
    # censor still-active runs at the human end
    out = []
    for run in runs:
        d_end = float(h['dist'][seg['i1']])
        exposure = None
        if run.follow_d is not None:
            d_stop = d_on + run.stop['dist_from_onset_m'] if run.stop and run.stop['dist_from_onset_m'] is not None \
                else d_end
            exposure = round(d_stop - run.follow_d, 3)
        lat = np.abs(np.asarray(run.lateral)) if run.lateral else None
        out.append(dict(controller=run.name, followed=run.follow_t is not None,
                        follow_t_rel=None if run.follow_t is None else round(run.follow_t, 3),
                        start_fail=run.start_fail, stop=run.stop, exposure_m=exposure,
                        follow_ticks=run.ticks_follow, follow_forward_ticks=run.fwd_ticks_follow,
                        correcting_ticks=run.correcting, agree_ticks=run.agree, saturated_ticks=run.saturated,
                        max_saturated_run_s=round(run.max_sat_run * TICK_S, 2),
                        row_end_posted_ticks=run.row_end_posted, row_end_early_ticks=run.row_end_early,
                        lateral_abs_p50_p90_max=None if lat is None else
                        [round(float(np.percentile(lat, p)), 3) for p in (50, 90, 100)],
                        side_inhibit=run.ctl.counts.get('side_inhibit', 0)))
    timing['scan'].extend(scan_times)
    timing['tick'].extend(tick_costs)
    timing['delay'].extend(tick_delays)
    seg_fwd_ticks = int(h['forward'][seg['i0']:seg['i1'] + 1].sum())
    return dict(kind=kind, t_on_rel=round(t_on - t0_rec, 2), t_off_rel=round(t_off - t0_rec, 2),
                segment_m=round(seg['dist_m'], 3), forward_ticks=seg_fwd_ticks, still_s=round(seg['still_s'], 2),
                est_window_s=round(t_off - t_est, 2), stats=stats, controllers=out)


def replay_record(rid, name, variant, out_dir, starts, mask_dir, limit_runs=None, min_segment_m=3.):
    wait_for_measurement_hold()
    t_wall = time.perf_counter()
    rec = F.load_record(rid, os.environ.get('ROW_REPLAY_BASE') or F.BASE_1010,
                        os.environ.get('ROW_REPLAY_DRIVE') or F.DRIVE_1010)
    k0, k1 = math.ceil(rec['t_scan'][0] / TICK_S), math.floor(rec['t_scan'][-1] / TICK_S)
    t_grid = np.arange(k0, k1 + 1, dtype=np.int64) * TICK_S     # integer ticks: np.arange on floats drifts
    h = F.human_ticks(rec, t_grid)
    segments = F.forward_segments(h)
    adapter, masked_beams = make_adapter(variant, rid, mask_dir)
    imu_order, w_imu, tilt = imu_yaw_rates(rec['imu'])
    codes = Codes()
    log, results = [], []
    timing = dict(scan=[], tick=[], delay=[])
    # standing: standing segments with standing semantics (task definition); flying: EVERY segment (standing
    # ones included) with flying semantics, so all forward driving is used. Pivot jiggles < min_segment_m are
    # not row-following attempts and are skipped (counted in the summary).
    eligible = [seg for seg in segments if seg['dist_m'] >= min_segment_m]
    todo = []
    if 'standing' in starts:
        todo += [(seg, 'standing') for seg in eligible if seg['standing']]
    if 'flying' in starts:
        todo += [(seg, 'flying') for seg in eligible]
    if limit_runs:
        todo = todo[:limit_runs]
    for n, (seg, kind) in enumerate(todo):
        first = len(log)
        res = run_one(rec, h, seg, kind, variant, adapter, imu_order, w_imu, codes, log, timing)
        res['run'] = n
        for row in log[first:]:
            row.insert(0, n)
        results.append(res)
    total_fwd = int(h['forward'].sum())
    pct = (lambda v: None if not len(v) else [round(float(np.percentile(v, p)) * 1e3, 2) for p in (50, 90, 99, 100)])
    summary = dict(schema='ubts.row_executor.shadow_replay.v1', rid=rid, variant=name, spec=variant,
                   controllers={k: dict(mode=m, overrides=o) for k, (m, o) in CONTROLLERS.items()},
                   adapter=variant.get('adapter', 'core'), vendor_commit=getattr(E, 'VENDOR_COMMIT', None),
                   starts=list(starts), masked_beams=masked_beams, imu_tilt_deg=None if tilt is None else round(tilt, 2),
                   record_forward_ticks=total_fwd, tick_s=TICK_S, segments=len(segments),
                   standing_segments=sum(1 for s in segments if s['standing']),
                   min_segment_m=min_segment_m, eligible_segments=len(eligible),
                   eligible_standing=sum(1 for s in eligible if s['standing']),
                   segments_m=round(sum(s['dist_m'] for s in segments), 1),
                   eligible_m=round(sum(s['dist_m'] for s in eligible), 1),
                   eligible_forward_ticks=int(sum(h['forward'][s['i0']:s['i1'] + 1].sum() for s in eligible)),
                   on_scan_ms_p50_p90_p99_max=pct(timing['scan']), tick_ms_p50_p90_p99_max=pct(timing['tick']),
                   tick_delay_ms_p50_p90_p99_max=pct(timing['delay']), scans_timed=len(timing['scan']),
                   wall_s=round(time.perf_counter() - t_wall, 1), cpu_count=os.cpu_count(),
                   workers=int(os.environ.get('ROW_REPLAY_WORKERS', '1')), runs=results, codes=codes.names,
                   constants=dict(standing_est_lead_max_s=STANDING_EST_LEAD_MAX_S, standing_hold_lead_s=STANDING_HOLD_LEAD_S,
                                  standing_accept_s=STANDING_ACCEPT_S, flying_est_lead_s=FLYING_EST_LEAD_S,
                                  flying_inject_s=FLYING_INJECT_S, at_end_m=AT_END_M,
                                  scan_latency_s=variant.get('scan_latency_s', A.SCAN_LATENCY_S),
                                  lidar_yaw_offset_deg=A.LIDAR_YAW_OFFSET_DEG))
    out = Path(out_dir) / name
    out.mkdir(parents=True, exist_ok=True)
    tag = rid[9:15]
    columns = ['run', 't_rel', 'kind', 'human_vf', 'human_dist', 'human_fwd', 'lidar_age', 'imu_age', 'speed',
               'corridor_valid', 'corridor_reason', 'heading_error', 'lateral', 'width', 'row_end_ahead',
               'front_clear', 'side_left', 'side_right', 'still', 'est_distance', 'corridor_age']
    for cname in CONTROLLERS:
        columns += [cname + ':' + c for c in ('state', 'reason', 'left', 'right', 'target_err_deg')]
    arr = np.array(log, dtype=np.float64) if log else np.zeros((0, len(columns)))
    np.savez_compressed(out / f'ticks_{tag}.npz', ticks=arr, columns=np.array(columns), codes=np.array(codes.names))
    np.save(out / f'scan_ms_{tag}.npy', np.asarray(timing['scan']) * 1e3)
    (out / f'summary_{tag}.json').write_text(json.dumps(summary, indent=1), encoding='utf-8')
    return dict(rid=rid, runs=len(results), wall_s=summary['wall_s'], on_scan=summary['on_scan_ms_p50_p90_p99_max'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--variant', default='base')
    ap.add_argument('--name')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--mask-dir', help='folder of the leave-one-out body masks (default: <out-dir>/masks)')
    ap.add_argument('--records', nargs='*', default=list(F.RECORDS_1010))
    ap.add_argument('--starts', nargs='*', default=['standing', 'flying'])
    ap.add_argument('--workers', type=int, default=1)
    ap.add_argument('--est', nargs='*', help='EstimatorConfig KEY=JSON overrides')
    ap.add_argument('--ctl', nargs='*', help='ControlConfig KEY=JSON overrides for every controller')
    ap.add_argument('--spec', help='JSON object merged into the variant (e.g. {"drop": true, "scale": 3})')
    ap.add_argument('--limit-runs', type=int)
    ap.add_argument('--min-segment-m', type=float, default=3.)
    ap.add_argument('--base', help='record layout root holding lidar/ (default: sim_paths field_manual_1010)')
    ap.add_argument('--drive-dir', help='folder of the <rid>.drive.json.gz files of --base (default: sim_paths '
                                        'field_manual_1010_drive)')
    ap.add_argument('--ref-inlier', type=float, help='reference ICP inlier threshold (human-motion labels only)')
    args = ap.parse_args()
    variant = json.loads(json.dumps(VARIANTS[args.variant]))
    variant.setdefault('est', {}).update(parse_kv(args.est))
    variant.setdefault('ctl', {}).update(parse_kv(args.ctl))
    if args.spec:
        variant.update(json.loads(args.spec))
    name = args.name or args.variant
    if args.mask_dir is None:
        args.mask_dir = os.path.join(args.out_dir, 'masks')
    E.EstimatorConfig(**variant['est'])               # fail fast on an invalid override
    C.config_for('continuous', **variant['ctl'])
    os.environ['ROW_REPLAY_WORKERS'] = str(args.workers)
    if args.base:
        os.environ['ROW_REPLAY_BASE'] = args.base      # inherited by the worker processes
    if args.drive_dir:
        os.environ['ROW_REPLAY_DRIVE'] = args.drive_dir
    if args.ref_inlier is not None:
        os.environ['ROW_REPLAY_REF_INLIER'] = str(args.ref_inlier)
        F.GOOD['inlier'] = args.ref_inlier
    print(json.dumps(dict(name=name, variant=variant, records=args.records, starts=args.starts)), flush=True)
    if args.workers > 1:
        with ProcessPoolExecutor(args.workers) as pool:
            futures = [pool.submit(replay_record, rid, name, variant, args.out_dir, args.starts, args.mask_dir,
                                   args.limit_runs, args.min_segment_m) for rid in args.records]
            for fut in futures:
                print(json.dumps(fut.result()), flush=True)
    else:
        for rid in args.records:
            print(json.dumps(replay_record(rid, name, variant, args.out_dir, args.starts, args.mask_dir,
                                           args.limit_runs, args.min_segment_m)), flush=True)


if __name__ == '__main__':
    main()
