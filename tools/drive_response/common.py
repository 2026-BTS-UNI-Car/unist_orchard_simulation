"""Shared helpers: configuration, path resolution, small statistics, LF/UTF-8 writers.

No data folder is named here: config.json (next to this file, or --config, or the environment variable
UBTS_DRIVE_RESPONSE_CONFIG) gives every location.
"""
import csv
import hashlib
import io
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
if TOOLS not in sys.path:            # tools/field_calib sits next to tools/drive_response
    sys.path.insert(0, TOOLS)

import numpy as np  # noqa: E402

CONFIG_ENV = 'UBTS_DRIVE_RESPONSE_CONFIG'
DEFAULT_CONFIG = os.path.join(HERE, 'config.json')
RECORDINGS_FILE = os.path.join(HERE, 'recordings.json')


class Config:
    def __init__(self, path=None):
        path = path or os.environ.get(CONFIG_ENV, '').strip() or DEFAULT_CONFIG
        with open(path, 'r', encoding='utf-8') as fh:
            self.raw = json.load(fh)
        self.path = os.path.abspath(path)
        self.root = self.raw['data_root']

    def path_of(self, key):
        value = self.raw[key]
        if os.path.isabs(value):
            return value
        return os.path.join(self.root, value)

    def relative(self, path):
        """Label of an input path for the output manifests: '<config key>/<rest>' when the path lies under one of the
        configured locations (the longest match wins), so no machine prefix and no data folder name is written.
        Paths outside every configured location fall back to the path relative to data_root."""
        absolute = os.path.abspath(path)
        full = os.path.normcase(absolute)
        best = None
        for key, value in self.raw.items():
            if key.startswith('_') or key == 'data_root' or not isinstance(value, str):
                continue
            base_abs = os.path.abspath(self.path_of(key))
            base = os.path.normcase(base_abs)
            if full == base or full.startswith(base.rstrip(os.sep) + os.sep):
                if best is None or len(base) > len(best[1]):
                    best = (key, base, base_abs)
        if best is not None:
            rest = os.path.relpath(absolute, best[2]) if full != best[1] else ''
            label = best[0] if rest in ('', '.') else best[0] + '/' + rest.replace(os.sep, '/')
            return label
        try:
            return os.path.relpath(path, self.root).replace(os.sep, '/')
        except ValueError:
            return path.replace(os.sep, '/')


def load_recordings():
    with open(RECORDINGS_FILE, 'r', encoding='utf-8') as fh:
        return json.load(fh)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def finite(values):
    v = np.asarray([x for x in values if x is not None], dtype=float)
    return v[np.isfinite(v)]


def pct(values, p):
    v = finite(values)
    return float(np.percentile(v, p)) if len(v) else math.nan


def q3(values, nd=3):
    """[p10, p50, p90] rounded, or None."""
    v = finite(values)
    if not len(v):
        return None
    return [round(float(np.percentile(v, p)), nd) for p in (10, 50, 90)]


def rnd(x, nd=3):
    if x is None:
        return None
    try:
        xf = float(x)
    except (TypeError, ValueError):
        return x
    return None if not math.isfinite(xf) else round(xf, nd)


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return None if not math.isfinite(float(value)) else float(value)
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def write_bytes(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as fh:
        fh.write(data)


def write_text(path, text):
    write_bytes(path, text.replace('\r\n', '\n').encode('utf-8'))


def write_json(path, obj):
    write_text(path, json.dumps(json_safe(obj), ensure_ascii=False, indent=1, sort_keys=False, allow_nan=False) + '\n')


def fmt(x, nd=3):
    if x is None:
        return ''
    if isinstance(x, float):
        if not math.isfinite(x):
            return ''
        return ('%.' + str(nd) + 'f') % x
    return str(x)


def write_csv(path, header, rows):
    buf = io.StringIO(newline='')
    w = csv.writer(buf, lineterminator='\n')
    w.writerow(header)
    for row in rows:
        w.writerow(['' if v is None else v for v in row])
    write_text(path, buf.getvalue())


def md_table(header, rows):
    out = ['| ' + ' | '.join(str(h) for h in header) + ' |', '| ' + ' | '.join('---' for _ in header) + ' |']
    for row in rows:
        out.append('| ' + ' | '.join('' if v is None else str(v).replace('|', '/') for v in row) + ' |')
    return '\n'.join(out)
