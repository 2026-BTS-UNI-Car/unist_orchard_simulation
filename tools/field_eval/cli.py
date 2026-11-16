"""Command line of field_eval.

  python -B tools/field_eval/cli.py evaluate --map MAP --executor EXECUTOR.jsonl --scans SOURCE --out DIR
         [--vehicle VEHICLE.json] [--start-pose X,Y,YAW_DEG] [--settle-m 6] [--run N] [--header-stamps]
         [--trajectory REF.csv  (instead of --scans: an external reference trajectory)] [--no-figures]
  python -B tools/field_eval/cli.py map-info --map MAP
  python -B tools/field_eval/cli.py verify-sim --run-dir FSIM_RUN --out DIR [--start-error ALONG,LAT,YAW_DEG]
         [--map-noise-m SD] [--seed N]          (PC only: compares with the SIM truth of TRACE.npz)
(or python -B -m field_eval ... from tools/). The output folder must not lie inside the repository.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

from field_eval import evaluate as EV  # noqa: E402
from field_eval import refmap as RM  # noqa: E402


def _inside_repo(path):
    a = os.path.normcase(os.path.realpath(path))
    r = os.path.normcase(os.path.realpath(REPO))
    return a == r or a.startswith(r + os.sep)


def _pose(text):
    if text is None:
        return None
    parts = [float(v) for v in text.split(',')]
    if len(parts) != 3:
        raise SystemExit('a pose is X,Y,YAW_DEG')
    return tuple(parts)


def _print(obj):
    sys.stdout.buffer.write((json.dumps(EV.clean(obj), indent=1, sort_keys=True, ensure_ascii=False) + '\n')
                            .encode('utf-8'))


def main(argv=None):
    p = argparse.ArgumentParser(description='field_eval: lateral error of a run against a reference centre line')
    sub = p.add_subparsers(dest='command', required=True)
    e = sub.add_parser('evaluate')
    e.add_argument('--map', required=True)
    e.add_argument('--executor', required=True, help='executor JSONL file or folder')
    e.add_argument('--scans', help='F-SIM capture folder, recorder archive folder or rosbag2 folder')
    e.add_argument('--trajectory', help='external reference trajectory CSV (x_m, y_m[, t_s, yaw_deg])')
    e.add_argument('--out', required=True)
    e.add_argument('--vehicle')
    e.add_argument('--start-pose')
    e.add_argument('--settle-m', type=float, default=0.)
    e.add_argument('--run', type=int)
    e.add_argument('--header-stamps', action='store_true', help='scan time = header stamp (default: receipt)')
    e.add_argument('--no-figures', action='store_true')
    e.add_argument('--result-kind')
    m = sub.add_parser('map-info')
    m.add_argument('--map', required=True)
    v = sub.add_parser('verify-sim')
    v.add_argument('--run-dir', required=True)
    v.add_argument('--out', required=True)
    v.add_argument('--start-error', default='0,0,0')
    v.add_argument('--map-noise-m', type=float, default=0.)
    v.add_argument('--map-drop', type=float, default=0., help='fraction of map landmarks left out')
    v.add_argument('--seed', type=int, default=0)
    v.add_argument('--no-figures', action='store_true')
    args = p.parse_args(argv)
    if args.command == 'map-info':
        _print(RM.load_map(args.map).describe())
        return 0
    if _inside_repo(args.out):
        raise SystemExit('refused: %s lies inside the repository' % args.out)
    if args.command == 'evaluate':
        report = EV.evaluate(args.map, args.executor, args.scans, args.out, args.vehicle, _pose(args.start_pose),
                             args.settle_m, args.run, args.trajectory, args.result_kind,
                             True if args.header_stamps else None, figures=not args.no_figures)
        m = report['reference_metrics']
        loc = (report.get('localisation') or {}).get('in_drive') or {}
        _print(dict(result_kind=report['result_kind'], reference_metrics=m,
                    executor_estimate_error=report['executor_estimate_error'], localisation_in_drive=loc,
                    config_digest=report['config_digest'], out=args.out))
        return 0
    from field_eval import sim_verify as SV
    row = SV.verify(args.run_dir, args.out, start_error=_pose(args.start_error), map_noise_m=args.map_noise_m,
                    map_drop=args.map_drop, seed=args.seed, figures=not args.no_figures)
    _print(row)
    return 0


if __name__ == '__main__':
    sys.exit(main())
