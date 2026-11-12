#!/usr/bin/env python3
"""Rebuild every drive-response table and figure (PW-01 .. PW-05) from the recorded manual drives.

Usage (any working directory; nothing is written next to the recordings):
  python -B <this folder>/run_all.py [--config <config.json>] [--out <output folder>]
Defaults: --config = config.json next to this file (or $UBTS_DRIVE_RESPONSE_CONFIG), --out = ../out next to this
folder. Data locations come only from the config. tools/field_calib must sit next to this folder (as in the
repository: tools/drive_response + tools/field_calib).
Dependencies: Python >= 3.10, numpy, matplotlib (Malgun Gothic or NanumBarunpen font for the Korean labels).
All values describe the car BEFORE the front-wheel steering was fitted (differential drive, October 2026).
"""
import argparse
import os
import sys
import time

import common
import analysis
import figures
import tables


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--config', default=None)
    ap.add_argument('--out', default=os.path.join(common.TOOLS, 'out'))
    ap.add_argument('--no-figures', action='store_true')
    args = ap.parse_args(argv)
    t0 = time.perf_counter()
    cfg = common.Config(args.config)
    log = lambda *a: print(*a, file=sys.stderr, flush=True)  # noqa: E731
    log('config %s, data_root %s' % (cfg.path, cfg.root))
    R = analysis.run(cfg, log=log)
    out = os.path.abspath(args.out)
    tables.write_all(R, out)
    tables.write_results(R, out)
    if not args.no_figures:
        figures.write_all(R, out)
    log('done in %.1f s -> %s' % (time.perf_counter() - t0, out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
