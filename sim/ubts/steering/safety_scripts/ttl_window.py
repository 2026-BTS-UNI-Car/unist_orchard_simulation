"""C TTL restart window sweep (DECISIONS R18 decision 1; audit AUD3A-04; SIM_ACCEPTANCE A17 'no automatic restart').

The window: C lines stop reaching the MCU (link to the MCU down), the MCU C TTL brakes the drive and sends F TTL; when
the link is back, the next C line clears TTL and the MCU drives again after the 120 ms rest (V3 spec 6.3, 8.2). Before
R18 decision 1 only the executor ended the run on that event, from its next drive reply; a late executor tick (rig
fault executor_stall right after the F TTL line) let the bridge's automatic C resends restart the drive. Now the host
bridge stops on the F TTL line itself (latch mcu_ttl_expired).

sweep() runs the c_line_loss_ttl episode over a grid of
  loss_start_s      start of the link loss (host -> MCU)
  loss_ms           length of the link loss
  stall_s           executor stall length (rig fault executor_stall, after='f_ttl')
  stall_delay_s     stall start after the F TTL line becomes readable by the host
  bridge_phase_ms   host bridge loop phase on the 10 ms rig grid
  boot_delay_ticks  MCU boot delay (moves the MCU status line phase)
and judges each run: the F TTL tick, the first tick with both drive outputs 0 after it, the first later tick at which a
drive output grows (a restart; measure.drive_increase_after), any drive output 1.5 s after the TTL, ARM lines after it,
the host latch mcu_ttl_expired and the executor end reason. The host code comes from the work tree's dashboard (or env
UBTS_STEER_DASHBOARD_DIR: e.g. an older host copy to show the window without the latch).

CLI (from sim/ubts): python -B -m steering.safety_scripts.ttl_window --out <json outside the repository>
                     [--workers N] [--quick] [--limit N]
Evaluation code (A13 role 'evaluator'). Every number is a placeholder of the SIM (자리값 기반 SIM 결과).
"""
from __future__ import annotations

import argparse
import copy
import itertools
import json
import os
import sys
import time

from . import acceptance as A
from . import catalogue as CAT
from . import measure as M

SCHEMA = 'ubts.steer_sim.ttl_window_sweep.v1'
GRID = dict(loss_start_s=(4.95, 4.97, 5.0, 5.03), loss_ms=(320, 400), stall_s=(.1, .2, .3, .4, .5),
            stall_delay_s=(0., .02), bridge_phase_ms=(0, 10), boot_delay_ticks=(60, 70))
QUICK_GRID = dict(loss_start_s=(4.97, 5.0), loss_ms=(320,), stall_s=(.1, .3, .5), stall_delay_s=(0.,),
                  bridge_phase_ms=(0, 10), boot_delay_ticks=(60,))
DURATION_S = 7.5              # the loss ends by 5.43 s, the stall by about 6.0 s, + the 1.5 s 'drive 0' check
KEYS = ('loss_start_s', 'loss_ms', 'stall_s', 'stall_delay_s', 'bridge_phase_ms', 'boot_delay_ticks')


def entry(loss_start_s, loss_ms, stall_s, stall_delay_s, bridge_phase_ms, boot_delay_ticks, duration_s=DURATION_S):
    """The catalogue entry of one grid point (c_line_loss_ttl with the loss, the stall and the phases set)."""
    base = copy.deepcopy(CAT.by_name()['c_line_loss_ttl'])
    name = 'ttl_window_l%d_%d_s%d_d%d_p%d_b%d' % (int(round(loss_start_s * 1000)), loss_ms, int(round(stall_s * 1000)),
                                                   int(round(stall_delay_s * 1000)), bridge_phase_ms, boot_delay_ticks)
    spec = base['spec']
    spec['name'] = name
    spec['faults'] = [CAT.fault('link_loss', at_s=loss_start_s, duration_s=loss_ms / 1000., direction='to_mcu'),
                      CAT.fault('executor_stall', at_s=loss_start_s, duration_s=stall_s, after='f_ttl',
                                delay_s=stall_delay_s)]
    spec['rig'] = dict(bridge_phase_ms=int(bridge_phase_ms), boot_delay_ticks=int(boot_delay_ticks))
    spec['duration_s'] = float(duration_s)
    spec['post_end_s'] = None
    base['name'] = name
    return base


def judge(art):
    """One run's outcome (module docstring)."""
    s = art['summary']
    f_tick, _ = M.f_line(art, 'TTL', 0)
    row = dict(f_ttl_tick=f_tick, end_reason=s['end_reason'], ended_at_s=s.get('ended_at_s'),
               stall_windows_ms=(s.get('executor_stall') or {}).get('windows_ms'),
               rig_stalled_ticks=(s.get('executor_stall') or {}).get('rig_stalled_ticks'),
               latch_mcu_ttl_expired_s=M.latch_time(art, 'mcu_ttl_expired'))
    if f_tick is None:
        row.update(restart=None, note='no F TTL line (the grid point did not expire the C TTL)')
        return row
    zero, increase = M.drive_increase_after(art, f_tick)
    late = M.drive_nonzero_after(art, f_tick + 1500)
    arms = M.h2m(art, 'ARM', since_ms=f_tick + 1)
    stops = [r['t_ms'] for r in M.h2m(art, 'STOP', since_ms=f_tick)]
    cs_after = [r for r in M.h2m(art, 'C', since_ms=f_tick) if not r['note'].endswith(':lost')]
    # the C lines that reached the MCU after the TTL (they clear TTL and would drive again after the 120 ms rest)
    row.update(drive_zero_tick=zero, drive_increase_tick=increase, drive_nonzero_after_1500_tick=late,
               arm_lines_after=len(arms), first_stop_after_ms=stops[0] if stops else None,
               c_lines_reaching_mcu_after=len(cs_after),
               first_c_after_ms=None if not cs_after else cs_after[0]['last_tick'],
               restart=bool(zero is None or increase is not None or late is not None or arms))
    return row


def _job(item):
    point, native_executable, build_dir = item
    started = time.perf_counter()
    e = entry(**point)
    result, art = A.run_script(e, None, None, native_executable, build_dir)
    row = dict(point, name=e['name'], wall_s=round(time.perf_counter() - started, 2))
    row.update(judge(art))
    return row


def points(grid=GRID):
    return [dict(zip(KEYS, values)) for values in itertools.product(*(grid[k] for k in KEYS))]


def sweep(out_path=None, grid=GRID, workers=1, limit=None, native_executable=None, build_dir=None):
    """Run the grid (module docstring); returns (and writes, with out_path) dict(summary, rows)."""
    started = time.perf_counter()
    todo = points(grid)
    if limit is not None:
        todo = todo[:int(limit)]
    items = [(p, native_executable, build_dir) for p in todo]
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=workers) as pool:
            rows = list(pool.map(_job, items))
    else:
        rows = [_job(item) for item in items]
    from steering import t1_rig as TR
    _, bc = TR.host_modules()
    summary = dict(runs=len(rows), with_ttl=sum(1 for r in rows if r['f_ttl_tick'] is not None),
                   restarts=sum(1 for r in rows if r.get('restart')),
                   latched=sum(1 for r in rows if r.get('latch_mcu_ttl_expired_s') is not None),
                   ends=_count(r['end_reason'] for r in rows),
                   host_code=os.path.abspath(getattr(bc, '__file__', '?')),
                   host_has_ttl_latch='mcu_ttl_expired' in getattr(bc, 'LATCH_NAMES', ()),
                   grid={k: list(v) for k, v in grid.items()}, wall_s=round(time.perf_counter() - started, 1))
    report = dict(schema=SCHEMA, result_kind=A.RESULT_KIND, field_validated=False, summary=summary, rows=rows)
    if out_path:
        raw = json.dumps(report, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False, default=str) + '\n'
        with open(out_path, 'wb') as fh:
            fh.write(raw.encode('utf-8'))
    return report


def _count(values):
    out = {}
    for v in values:
        out[str(v)] = out.get(str(v), 0) + 1
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--out', required=True)
    p.add_argument('--workers', type=int, default=1)
    p.add_argument('--quick', action='store_true', help='the small grid (QUICK_GRID)')
    p.add_argument('--limit', type=int, default=None)
    args = p.parse_args(argv)
    from steering.scene_spec import inside_repository
    out = os.path.abspath(args.out)
    if inside_repository(out):
        raise SystemExit('refused: %s lies inside the repository (runs live outside)' % out)
    report = sweep(out, QUICK_GRID if args.quick else GRID, args.workers, args.limit)
    print(json.dumps(report['summary'], sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
