"""Virtual GNSS of the fast SIM T1 rig (plant side; design v2 V2-1 5, PC-12). SIM only.

A general (non-RTK) receiver at about 1 Hz, made from the TRUE antenna position (like the LiDAR and IMU synthesis; the
executor gets the fix only, never the pose): error = per-run bias + first-order Gauss-Markov walk + white noise, in a
local metric frame rotated by frame_rotation_deg against the SIM world (a real row is not aligned with east). The
reported horizontal accuracy is hacc_m (+ small jitter). No latitude / longitude anywhere: the fix is local metres
(east_m, north_m) of that frame; the executor's GNSS layer keeps them run-local.

Conditions (SIM seconds; design V2-1 / PC-12 condition names in steering.safety_scripts.perception_compare):
  dropouts      [[t0, t1], ...]       no fix in the window (silent receiver; 'no_fix' mode: fixes with fix_ok false)
  jumps         [{at_s, east_m, north_m, duration_s}, ...]   fixes with a stamp in [at_s, at_s + duration_s) get the
                offset added (a spike for duration_s <= 1 / rate_hz)
  hacc_windows  [[t0, t1, scale], ...] reported accuracy AND the walk / white noise x scale (gnss_hacc_x3)
Evaluation counters (report()): fixes emitted, suppressed by a dropout, carrying a jump, scaled. Placeholder values
(자리값): the 10/10 receiver reported 5.3-10 m (p50) and about 1 Hz; its short-term jitter was not measured.
"""
import json
import math

import numpy as np

DEFAULTS = dict(rate_hz=1., phase_ms=230, latency_s=.30, hacc_m=7., hacc_jitter_m=.3, bias_sigma_m=3.,
                walk_sigma_m=1.5, walk_tau_s=60., white_sigma_m=.4, frame_rotation_deg=30., frame_offset_m=(0., 0.),
                antenna_xy_m=None, dropouts=(), dropout_mode='silent', jumps=(), hacc_windows=())


def canonical(spec):
    """The rig configuration value of a GNSS spec: canonical JSON text (hashable, deterministic), or None."""
    if spec is None:
        return None
    if isinstance(spec, str):
        spec = json.loads(spec)
    unknown = sorted(set(spec) - set(DEFAULTS))
    if unknown:
        raise ValueError('unknown gnss keys: %s' % ', '.join(unknown))
    return json.dumps(spec, sort_keys=True, separators=(',', ':'), allow_nan=False)


class GnssEmulator:
    def __init__(self, spec, seed, antenna_xy_m=(0., 0.)):
        values = dict(DEFAULTS)
        values.update(json.loads(spec) if isinstance(spec, str) else dict(spec))
        self.v = values
        if not 0.1 <= float(values['rate_hz']) <= 10.:
            raise ValueError('gnss rate_hz outside [0.1, 10]')
        self.period_ms = int(round(1000. / float(values['rate_hz'])))
        if self.period_ms % 10:
            raise ValueError('gnss period %d ms is not on the 10 ms plant grid' % self.period_ms)
        self.phase_ms = int(values['phase_ms']) % self.period_ms
        if self.phase_ms % 10:
            raise ValueError('gnss phase_ms must be on the 10 ms grid')
        if values['dropout_mode'] not in ('silent', 'no_fix'):
            raise ValueError("gnss dropout_mode must be 'silent' or 'no_fix'")
        self.antenna = tuple(float(a) for a in (values['antenna_xy_m'] or antenna_xy_m))
        self.rng = np.random.default_rng(seed)
        self.bias = self.rng.normal(0., float(values['bias_sigma_m']), 2)
        self.walk = self.rng.normal(0., float(values['walk_sigma_m']), 2)
        rot = math.radians(float(values['frame_rotation_deg']))
        self.rot = (math.cos(rot), math.sin(rot))
        self.offset = tuple(float(o) for o in values['frame_offset_m'])
        self.last_t = None
        self.counts = dict(emitted=0, suppressed=0, jumped=0, scaled=0, no_fix=0)
        self.jump_stamps = []

    def _window(self, items, t):
        for item in items:
            if float(item[0]) <= t < float(item[1]):
                return item
        return None

    def sample(self, t_s, base_x, base_y, base_yaw):
        """A fix packet body for SIM time t_s from the TRUE base pose, or None (no fix due now / dropout)."""
        t_ms = int(round(t_s * 1000.))
        if t_ms % self.period_ms != self.phase_ms:
            return None
        v = self.v
        dt = self.period_ms / 1000. if self.last_t is None else t_s - self.last_t
        self.last_t = t_s
        scale = 1.
        window = self._window(v['hacc_windows'], t_s)
        if window is not None:
            scale = float(window[2])
        # the walk and the white noise run whether or not a fix is emitted (the receiver keeps tracking)
        a = math.exp(-dt / float(v['walk_tau_s']))
        self.walk = self.walk * a + self.rng.normal(0., float(v['walk_sigma_m']) * math.sqrt(1. - a * a), 2)
        white = self.rng.normal(0., float(v['white_sigma_m']), 2)
        jitter = float(self.rng.normal(0., float(v['hacc_jitter_m'])))
        dropped = self._window(v['dropouts'], t_s) is not None
        if dropped and v['dropout_mode'] == 'silent':
            self.counts['suppressed'] += 1
            return None
        c, s = math.cos(base_yaw), math.sin(base_yaw)
        ax = base_x + c * self.antenna[0] - s * self.antenna[1]
        ay = base_y + s * self.antenna[0] + c * self.antenna[1]
        error = self.bias + scale * (self.walk + white)
        east = self.rot[0] * ax - self.rot[1] * ay + self.offset[0] + float(error[0])
        north = self.rot[1] * ax + self.rot[0] * ay + self.offset[1] + float(error[1])
        for jump in v['jumps']:
            if float(jump['at_s']) <= t_s < float(jump['at_s']) + float(jump.get('duration_s', .5)):
                east += float(jump.get('east_m', 0.))
                north += float(jump.get('north_m', 0.))
                self.counts['jumped'] += 1
                self.jump_stamps.append(round(t_s, 3))
                break
        if scale != 1.:
            self.counts['scaled'] += 1
        hacc = max(.1, scale * (float(v['hacc_m']) + jitter))
        self.counts['emitted'] += 1
        if dropped:
            self.counts['no_fix'] += 1
            return dict(east_m=float('nan'), north_m=float('nan'), hacc_m=None, fix_ok=False,
                        latency_s=float(v['latency_s']))
        return dict(east_m=round(east, 3), north_m=round(north, 3), hacc_m=round(hacc, 2), fix_ok=True,
                    latency_s=float(v['latency_s']))

    def report(self):
        """Evaluation counters (never an executor input)."""
        return dict(counts=dict(self.counts), jump_stamps=list(self.jump_stamps), period_ms=self.period_ms,
                    spec=json.loads(canonical({k: self.v[k] for k in self.v if k in DEFAULTS
                                               and k != 'antenna_xy_m'} | {'antenna_xy_m': list(self.antenna)})))
