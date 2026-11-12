"""Shadow replay of the real 10/10 records through executor_main's code path with a fake ROS feeder.

No ROS, no motor, no Jetson; the records are read-only (field_records.load_record). The feeder plays the
role of executor_main.RosInput: for every recorded message it builds a duck-typed sensor_msgs object
(LaserScan: header.stamp, ranges [m, 0 = no return], angle_min, angle_increment, range_min, range_max;
Imu: header.stamp, angular_velocity, linear_acceleration; /rover/control/requested: the JSON string) and
calls the SAME ExecutorLoop.on_scan_msg / on_imu_msg / on_human_msg the rclpy callbacks call, with the
recorded host receipt time shifted onto a monotonic-like clock (receipt - first receipt + 1000 s; headers keep
their epoch time, as live). ExecutorLoop.step() runs on an integer 20 Hz grid of that clock, in SHADOW with
--shadow-auto behaviour (emulated hold, automatic restarts: no operator is in the records), writing the
real JSONL log (JsonlLog). Everything between the callbacks and the decision is the field code path:
InputBuffer, ScanStamper, newest-scan-only feeding, Runner (adapter, near-confirm filter, vendored
estimator/controller, geometry guard, backstop with row length max_row_len_m, OutputGuard), status, log.

--compute-scale X (optional): the virtual clock advances by max(50 ms, X * measured step time), so scans
arrive while the loop is 'busy' (dropped except the newest) and the stall watchdog is evaluated as its thread
would (ExecutorLoop.watchdog_check) -> a coarse Jetson-speed stand-in (X = 3: 'Jetson ~3x slower', an assumption).

Scoring uses the offline ICP reference of the human motion (field_records; never fed to the executor):
FOLLOW share of the human forward time, terminal ends per km of human forward driving, ends while the human
kept driving straight >= 3 m ('mid-straight', the gate's premature-stop proxy), proposal duties and their guard.
Usage: python -B tools/row_executor/replay_executor_shadow.py --out-dir <dir> [--records ...] [--compute-scale 3]
"""
import argparse
import collections
import json
import math
import os
import sys
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import field_records as F  # noqa: E402
import sim_paths as P  # noqa: E402  (HOLD_MEASUREMENT file)
from row_executor import config as C  # noqa: E402
from row_executor import executor_main as M  # noqa: E402
from row_executor.core import ACTIVE, ENDED, PAUSED  # noqa: E402

TICK_S = .05
MONO0 = 1000.


def ns(**kw):
    return types.SimpleNamespace(**kw)


def header_from_seconds(t):
    sec = math.floor(t)
    return ns(stamp=ns(sec=int(sec), nanosec=min(999999999, int(round((t - sec) * 1e9)))))


def header_from_ns(t_ns):
    t_ns = int(t_ns)
    return ns(stamp=ns(sec=t_ns // 1000000000, nanosec=t_ns % 1000000000))


class FakeRosFeeder:
    """Plays executor_main.RosInput: recorded messages -> fake sensor_msgs -> the loop's callback entry points."""

    def __init__(self, rec, loop):
        self.rec, self.loop = rec, loop
        self.offset = MONO0 - float(rec['t_scan'][0])
        imu = rec['imu']
        self.imu = imu[np.argsort(imu[:, 0], kind='stable')]
        self.human = [(float(m['received_at']), m['req_l'], m['req_r']) for m in rec['motor']
                      if m.get('received_at') is not None]
        self.human.sort(key=lambda x: x[0])
        self.i_scan = self.i_imu = self.i_human = 0
        self.delivered = collections.Counter()

    def mono(self, t_epoch):
        return float(t_epoch) + self.offset

    def deliver_until(self, now):
        rec = self.rec
        t_scan = rec['t_scan']
        while self.i_imu < len(self.imu) and self.mono(self.imu[self.i_imu, 0] / 1e9) <= now + 1e-9:
            row = self.imu[self.i_imu]
            msg = ns(header=header_from_ns(row[1]), angular_velocity=ns(x=row[2], y=row[3], z=row[4]),
                     linear_acceleration=ns(x=row[5], y=row[6], z=row[7]))
            self.loop.on_imu_msg(self.mono(row[0] / 1e9), msg)
            self.i_imu += 1
            self.delivered['imu'] += 1
        while self.i_scan < len(t_scan) and self.mono(t_scan[self.i_scan]) <= now + 1e-9:
            k = self.i_scan
            msg = ns(header=header_from_seconds(rec['t_header'][k]),
                     ranges=rec['ranges_mm'][k].astype(np.float32) / np.float32(1000.),
                     angle_min=rec['angle_min'], angle_increment=rec['increment'],
                     range_min=rec['range_min'], range_max=rec['range_max'])
            self.loop.on_scan_msg(self.mono(t_scan[k]), msg)
            self.i_scan += 1
            self.delivered['scan'] += 1
        while self.i_human < len(self.human) and self.mono(self.human[self.i_human][0]) <= now + 1e-9:
            t, left, right = self.human[self.i_human]
            if left is not None and right is not None:
                self.loop.on_human_msg(self.mono(t), json.dumps({'left': int(left), 'right': int(right)}))
            self.i_human += 1
            self.delivered['human'] += 1

    def done(self):
        return self.i_scan >= len(self.rec['t_scan'])


def replay(rid, cfg, out_dir, compute_scale=0., limit_s=None):
    rec = F.load_record(rid)
    clock = [0.]
    log = M.JsonlLog(os.path.join(out_dir, 'logs', rid[:15]), max_file_mb=200., wall=lambda: 1.79e9)
    loop = M.ExecutorLoop(cfg, shadow_auto=True, clock=lambda: clock[0], log=log,
                          wall=lambda: clock[0] - MONO0 + float(rec['t_scan'][0]), shadow_auto_restart_s=1.)
    feeder = FakeRosFeeder(rec, loop)
    k_tick = int(math.ceil(MONO0 / TICK_S))
    end_t = feeder.mono(rec['t_scan'][-1]) if limit_s is None else MONO0 + limit_s
    ticks = []
    step_ms = []
    exc = None
    watchdog_fires = 0
    started = time.perf_counter()
    while True:
        now = k_tick * TICK_S
        if now > end_t:
            break
        clock[0] = now
        feeder.deliver_until(now)
        c0 = time.perf_counter()
        try:
            d = loop.step(now)
        except Exception as error:            # recorded and the replay stops: an exception is a finding
            exc = repr(error)
            break
        cost = time.perf_counter() - c0
        step_ms.append(cost * 1e3)
        s = d.summary
        ticks.append((now, d.state, d.reason, d.left, d.right, s.get('controller_state'), s.get('run_id'),
                      loop.runner.end_reason if d.state == ENDED else None))
        if compute_scale > 0:
            busy = cost * compute_scale
            advance = max(1, int(math.ceil(busy / TICK_S - 1e-9)))
            if busy > cfg.stall_guard_s:
                clock[0] = now + busy
                if loop.watchdog_check(now + busy):
                    watchdog_fires += 1
            k_tick += advance
        else:
            k_tick += 1
    wall = time.perf_counter() - started
    log.close()
    return rec, loop, feeder, ticks, step_ms, exc, wall, watchdog_fires, log


def score(rid, rec, loop, feeder, ticks, step_ms, exc, wall, watchdog_fires, log):
    t = np.array([x[0] for x in ticks])
    t_epoch = t - MONO0 + float(rec['t_scan'][0])
    h = F.human_ticks(rec, t_epoch)
    states = [x[1] for x in ticks]
    ctl = [x[5] for x in ticks]
    fwd = h['forward']
    follow = np.array([c in ('FOLLOW', 'REDUCE') and s == ACTIVE for s, c in zip(states, ctl)])
    ends, mid_straight = collections.Counter(), collections.Counter()
    end_events = []
    prev_state = None
    for i, x in enumerate(ticks):
        if x[1] == ENDED and prev_state != ENDED:
            reason = x[7]
            ends[reason] += 1
            ahead = F.straight_ahead_m(h, i) if bool(fwd[i]) else None
            if ahead is not None and ahead >= 3.:
                mid_straight[reason] += 1
            end_events.append(dict(t_rel=round(x[0] - MONO0, 2), reason=reason, run=x[6],
                                   human_forward=bool(fwd[i]), human_straight_ahead_m=None if ahead is None else round(ahead, 1)))
        prev_state = x[1]
    left = np.array([x[3] for x in ticks])
    right = np.array([x[4] for x in ticks])
    driving = (left > 0) | (right > 0)
    dt = TICK_S
    fwd_s = float(fwd.sum() * dt)
    fwd_km = float(np.nansum(np.where(fwd, np.clip(h['vf'], 0, None), 0.)) * dt) / 1000.
    runner = loop.runner
    steps = np.asarray(step_ms) if step_ms else np.zeros(1)
    status = loop.status()
    return dict(
        record=rid, exception=exc, replay_wall_s=round(wall, 1), ticks=len(ticks),
        record_s=round(float(rec['t_scan'][-1] - rec['t_scan'][0]), 1),
        delivered=dict(feeder.delivered), loop_counts=dict(loop.counts), stamper=loop.stamper.summary(),
        runner_stats=dict(runner.stats, scans_rejected=dict(runner.stats['scans_rejected'])),
        runs=int(max((x[6] or 0) for x in ticks)) if ticks else 0, terminal_ends=dict(ends),
        ends_while_human_straight_3m=dict(mid_straight),
        human_forward_s=round(fwd_s, 1), human_forward_km=round(fwd_km, 3),
        follow_share_of_human_forward=round(float((follow & fwd).sum() / max(1, fwd.sum())), 3),
        terminal_ends_per_km_forward=round(sum(ends.values()) / fwd_km, 1) if fwd_km > 0 else None,
        mid_straight_ends_per_km=round(sum(mid_straight.values()) / fwd_km, 1) if fwd_km > 0 else None,
        proposal_ticks=int(driving.sum()), proposal_duty_left_minmax=[int(left[driving].min()), int(left[driving].max())]
        if driving.any() else None, proposal_max_abs_diff=int(np.abs(left - right)[driving].max()) if driving.any() else 0,
        output_guard_violations=runner.guard.violations,
        bridge_object=loop.bridge is not None, step_ms=dict(p50=round(float(np.percentile(steps, 50)), 2),
                                                           p99=round(float(np.percentile(steps, 99)), 2),
                                                           max=round(float(steps.max()), 2)),
        watchdog_fires=watchdog_fires, log_files=[os.path.basename(p) for p in log.paths],
        log_bytes=log.total_bytes, log_dropped=log.dropped, status_keys=sorted(status)[:60],
        first_end_events=end_events[:40])


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    p.add_argument('--out-dir', required=True)
    p.add_argument('--records', nargs='*', default=list(F.RECORDS_1010))
    p.add_argument('--compute-scale', type=float, default=0.)
    p.add_argument('--limit-s', type=float)
    p.add_argument('--config', help='executor config JSON (default: the field default = ExecutorConfig())')
    p.add_argument('--name', default='default')
    args = p.parse_args(argv)
    hold = P.measurement_hold_file()
    while hold is not None and hold.exists():
        print('HOLD_MEASUREMENT present, waiting 60 s', flush=True)
        time.sleep(60)
    cfg = C.load_config(args.config) if args.config else C.ExecutorConfig().validate()
    out_dir = os.path.join(args.out_dir, args.name)
    os.makedirs(out_dir, exist_ok=True)
    results = []
    for rid in args.records:
        out = replay(rid, cfg, out_dir, args.compute_scale, args.limit_s)
        row = score(rid, *out)
        results.append(row)
        print(json.dumps({k: row[k] for k in ('record', 'exception', 'ticks', 'runs', 'terminal_ends',
                                               'follow_share_of_human_forward', 'mid_straight_ends_per_km',
                                               'step_ms', 'watchdog_fires', 'replay_wall_s')}), flush=True)
    total_km = sum(r['human_forward_km'] for r in results)
    total = dict(records=len(results), exceptions=sum(r['exception'] is not None for r in results),
                 human_forward_km=round(total_km, 3),
                 terminal_ends=sum(sum(r['terminal_ends'].values()) for r in results),
                 mid_straight_ends=sum(sum(r['ends_while_human_straight_3m'].values()) for r in results),
                 output_guard_violations=sum(r['output_guard_violations'] for r in results),
                 bridge_objects=sum(r['bridge_object'] for r in results),
                 watchdog_fires=sum(r['watchdog_fires'] for r in results),
                 scans_dropped=sum(r['loop_counts']['scans_dropped'] for r in results),
                 log_dropped=sum(r['log_dropped'] for r in results))
    reasons = collections.Counter()
    mid = collections.Counter()
    for r in results:
        reasons.update(r['terminal_ends'])
        mid.update(r['ends_while_human_straight_3m'])
    total.update(terminal_end_reasons=dict(reasons.most_common()), mid_straight_end_reasons=dict(mid.most_common()),
                 mid_straight_m_per_end=round(total_km * 1000 / max(1, total['mid_straight_ends']), 1))
    doc = dict(schema='ubts.row_executor.shadow_replay.v1', created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
               name=args.name, compute_scale=args.compute_scale, config=cfg.to_dict(), config_digest=cfg.digest(),
               total=total, records=results)
    with open(os.path.join(out_dir, 'SHADOW_REPLAY.json'), 'w', encoding='utf-8', newline='\n') as fh:
        json.dump(doc, fh, indent=1, default=str)
        fh.write('\n')
    print(json.dumps(total, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
