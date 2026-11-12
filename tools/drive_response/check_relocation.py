#!/usr/bin/env python3
"""PW-02 check: copy every input file into another folder (same relative layout), point a copied config at it,
rebuild the tables and compare them byte for byte with an existing output folder.

Usage: python -B check_relocation.py --out <existing output folder> --work <empty scratch folder> [--keep]
Only the config's data_root changes; the copies are removed afterwards unless --keep. Nothing is written next to
the original data.
"""
import argparse
import filecmp
import json
import os
import shutil
import subprocess
import sys

import common


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--config', default=None)
    ap.add_argument('--out', required=True, help='output folder of a normal run (tables/, results.json, INPUTS.json)')
    ap.add_argument('--work', required=True, help='scratch folder for the copied data and the second output')
    ap.add_argument('--keep', action='store_true')
    args = ap.parse_args(argv)
    cfg = common.Config(args.config)
    if os.path.exists(args.work) and os.listdir(args.work):
        raise SystemExit('--work must be empty or missing')
    data = os.path.abspath(os.path.join(args.work, 'data'))
    with open(os.path.join(args.out, 'INPUTS.json'), 'r', encoding='utf-8') as fh:
        files = list(json.load(fh)['files'])
    extra = [cfg.raw['inventory_probe_output']]
    extra += [f.replace('motor.jsonl', 'metadata.json') for f in files if f.endswith('motor.jsonl')]
    for rel in files + extra:
        dst = os.path.join(data, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(os.path.join(cfg.root, rel), dst)
    moved = dict(cfg.raw, data_root=data.replace(os.sep, '/'))
    cfg2 = os.path.join(args.work, 'config_moved.json')
    common.write_json(cfg2, moved)
    out2 = os.path.join(args.work, 'out')
    subprocess.run([sys.executable, '-B', os.path.join(common.HERE, 'run_all.py'), '--config', cfg2, '--out', out2,
                    '--no-figures'], check=True)
    same, diff = [], []
    for sub in ('tables', '.'):
        for name in sorted(os.listdir(os.path.join(args.out, sub))):
            a, b = os.path.join(args.out, sub, name), os.path.join(out2, sub, name)
            if os.path.isdir(a):
                continue
            (same if os.path.isfile(b) and filecmp.cmp(a, b, shallow=False) else diff).append(name)
    print('copied %d files; identical %d; different %d %s' % (len(files) + len(extra), len(same), len(diff), diff))
    if not args.keep:
        shutil.rmtree(args.work)
    return 0 if not diff else 1


if __name__ == '__main__':
    sys.exit(main())
