"""Executor JSONL reader (tools/row_executor/executor_main.py records; one file, several rotated files or a folder).

Tick records (k 'tick', 20 Hz) give per tick: t (executor clock: time.monotonic on the Jetson, the SIM clock in the
F-SIM), state, T (throttle %, the drive command), S (target steering mrad), steer.meas (measured steering mrad),
v (LiDAR speed m/s), sr.vt (target speed m/s), lat (online lateral offset m, left positive), head (online heading
error deg = corridor heading - vehicle heading, i.e. minus the vehicle heading against the row), yaw (yaw rate deg/s),
width, corridor. Events (k 'event') carry t and wall (time.time): their median wall - t maps ticks to the wall clock
of the recorder (rosbag2 receipt and the recorder sidecar use time.time on the same Jetson).
The 'config' event holds the whole configuration and config_digest; 'start' and 'run_end' bound a run.
"""
from __future__ import annotations

import gzip
import hashlib
import json
import math
import os

import numpy as np

TICK_FIELDS = ('t', 'T', 'S', 'meas', 'v', 'vt', 'lat', 'head', 'yaw', 'width', 'run', 'active', 'corridor')
VEHICLE_KEYS = ('lidar_x_in_base_m', 'lidar_y_in_base_m', 'lidar_yaw_offset_deg', 'footprint_length_m',
                'footprint_width_m', 'imu_yaw_sign')


def _num(value):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return math.nan
    return v if math.isfinite(v) else math.nan


def _files(path):
    if os.path.isdir(path):
        names = sorted(n for n in os.listdir(path) if n.endswith('.jsonl') or n.endswith('.jsonl.gz'))
        if not names:
            raise FileNotFoundError('no *.jsonl in %s' % path)
        return [os.path.join(path, n) for n in names]
    return [path]


def _open(path):
    return gzip.open(path, 'rb') if path.endswith('.gz') else open(path, 'rb')


class ExecutorLog:
    """Ticks as numpy columns, events as dicts, the config and its digest, file SHA-256s."""

    def __init__(self, path, run=None):
        self.files = _files(path)
        self.sha256 = {}
        ticks, events = [], []
        self.bad_lines = 0
        for name in self.files:
            h = hashlib.sha256()
            with _open(name) as fh:
                for line in fh:
                    h.update(line)
                    try:
                        rec = json.loads(line)
                    except ValueError:
                        self.bad_lines += 1
                        continue
                    if not isinstance(rec, dict):
                        continue
                    if rec.get('k') == 'tick':
                        ticks.append(rec)
                    elif rec.get('k') == 'event':
                        events.append(rec)
            self.sha256[os.path.basename(name)] = h.hexdigest()
        self.events = events
        configs = [e for e in events if e.get('event') == 'config']
        self.config = dict(configs[-1].get('config') or {}) if configs else {}
        self.config_digests = sorted({e.get('config_digest') for e in events
                                      if e.get('event') in ('config', 'start') and e.get('config_digest')})
        offsets = [float(e['wall']) - float(e['t']) for e in events
                   if isinstance(e.get('wall'), (int, float)) and isinstance(e.get('t'), (int, float))]
        self.wall_minus_t = float(np.median(offsets)) if offsets else None
        self.wall_offset_spread_s = float(np.ptp(offsets)) if len(offsets) > 1 else 0.
        self.runs = sorted({int(r['run']) for r in ticks if isinstance(r.get('run'), int)})
        driving = sorted({int(r['run']) for r in ticks if isinstance(r.get('run'), int) and (r.get('T') or 0) > 0})
        if run is None:
            run = driving[-1] if driving else (self.runs[-1] if self.runs else None)
        self.run = run
        self.run_end = next((e for e in events if e.get('event') == 'run_end' and e.get('run_id') == run), None)
        self.run_start = next((e for e in events if e.get('event') == 'start' and e.get('run_id') == run), None)
        self.ticks = self._columns(ticks)

    def _columns(self, ticks):
        cols = {k: [] for k in TICK_FIELDS}
        cols['state'] = []
        for r in ticks:
            steer = r.get('steer') if isinstance(r.get('steer'), dict) else {}
            sr = r.get('sr') if isinstance(r.get('sr'), dict) else {}
            cols['t'].append(_num(r.get('t')))
            cols['T'].append(_num(r.get('T')) if 'T' in r else _num(max(r.get('L') or 0, r.get('R') or 0)))
            cols['S'].append(_num(r.get('S')))
            cols['meas'].append(_num(steer.get('meas')))
            cols['v'].append(_num(r.get('v')))
            cols['vt'].append(_num(sr.get('vt')))
            cols['lat'].append(_num(r.get('lat')))
            cols['head'].append(_num(r.get('head')))
            cols['yaw'].append(_num(r.get('yaw')))
            cols['width'].append(_num(r.get('width')))
            cols['run'].append(r.get('run') if isinstance(r.get('run'), int) else -1)
            cols['active'].append(1. if r.get('state') == 'ACTIVE' else 0.)
            cols['corridor'].append(1. if r.get('corridor') else 0.)
            cols['state'].append(str(r.get('state')))
        out = {k: np.asarray(v, dtype=float) for k, v in cols.items() if k not in ('state', 'run')}
        out['run'] = np.asarray(cols['run'], dtype=int)
        out['state'] = np.asarray(cols['state'])
        return out

    def run_ticks(self):
        """Tick columns of the selected run (all ticks when the log has no run ids)."""
        if self.run is None:
            return dict(self.ticks)
        keep = self.ticks['run'] == self.run
        return {k: v[keep] for k, v in self.ticks.items()}

    def drive_interval(self):
        """(t_first, t_last) of the ticks of the run with a drive command T > 0 (executor clock), or None."""
        ticks = self.run_ticks()
        on = np.flatnonzero(np.nan_to_num(ticks['T']) > 0)
        if not len(on):
            return None
        return float(ticks['t'][on[0]]), float(ticks['t'][on[-1]])

    def vehicle(self):
        """Vehicle values the executor ran with (lidar mount, footprint, IMU sign); missing keys are absent."""
        return {k: self.config[k] for k in VEHICLE_KEYS if self.config.get(k) is not None}

    def describe(self):
        return dict(files=[os.path.basename(f) for f in self.files], sha256=self.sha256, ticks=len(self.ticks['t']),
                    events=len(self.events), bad_lines=self.bad_lines, runs=self.runs, run=self.run,
                    config_digest=self.config_digests, wall_minus_t_s=self.wall_minus_t,
                    wall_offset_spread_s=self.wall_offset_spread_s,
                    end_reason=None if self.run_end is None else self.run_end.get('end_reason'),
                    vehicle=self.vehicle())
