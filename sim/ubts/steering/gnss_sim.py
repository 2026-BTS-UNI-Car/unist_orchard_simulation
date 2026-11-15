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
RTK receiver model (key 'rtk', None = off: the general receiver above, packets and random streams unchanged; with it
every packet carries fix_type; docs/steering/GNSS_RTK_KO.md 5):
  rtk = {default_type, windows: [[t0, t1, type], ...], fixed: {...}, float: {...}}   type 'fixed' | 'float' | 'single'
                | 'none' (fix_ok false). Per type an error model (hacc_m reported, hacc_jitter_m, white_sigma_m,
                walk_sigma_m, walk_tau_s; 'single' = the general receiver's values above, its bias included). The walks
                of all types run at every fix (a solution change does not reset them).
  frame_mirror  true: east and north swapped (a left-handed frame: a wrong frame for the executor)
  frame_scale   [sx, sy] scale of the frame axes before the offset (a badly scaled local frame)
Placeholder RTK values (자리값; ZED-F9P class datasheet order of magnitude, the receiver under canopy never measured):
FIXED hacc 0.014 m, white 0.008 m, walk 0.006 m / 30 s; FLOAT hacc 0.25 m, white 0.06 m, walk 0.20 m / 20 s.
Evaluation counters (report()): fixes emitted, suppressed by a dropout, carrying a jump, scaled, per RTK type. Placeholder
values (자리값): the 10/10 receiver reported 5.3-10 m (p50) and about 1 Hz; its short-term jitter was not measured.
"""
import json
import math

import numpy as np

DEFAULTS = dict(rate_hz=1., phase_ms=230, latency_s=.30, hacc_m=7., hacc_jitter_m=.3, bias_sigma_m=3.,
                walk_sigma_m=1.5, walk_tau_s=60., white_sigma_m=.4, frame_rotation_deg=30., frame_offset_m=(0., 0.),
                antenna_xy_m=None, dropouts=(), dropout_mode='silent', jumps=(), hacc_windows=(),
                rtk=None, frame_mirror=False, frame_scale=(1., 1.))
NEW_KEYS = ('rtk', 'frame_mirror', 'frame_scale')     # left out of report() while at their defaults (older reports)
RTK_TYPES = ('fixed', 'float', 'single', 'none')
RTK_MODELS = dict(fixed=dict(hacc_m=.014, hacc_jitter_m=.002, white_sigma_m=.008, walk_sigma_m=.006, walk_tau_s=30.),
                  float=dict(hacc_m=.25, hacc_jitter_m=.05, white_sigma_m=.06, walk_sigma_m=.20, walk_tau_s=20.))
RTK_KEYS = ('default_type', 'windows', 'fixed', 'float')


def canonical(spec):
    """The rig configuration value of a GNSS spec: canonical JSON text (hashable, deterministic), or None."""
    if spec is None:
        return None
    if isinstance(spec, str):
        spec = json.loads(spec)
    unknown = sorted(set(spec) - set(DEFAULTS))
    if unknown:
        raise ValueError('unknown gnss keys: %s' % ', '.join(unknown))
    if spec.get('rtk') is not None:
        _rtk_config(spec['rtk'])
    return json.dumps(spec, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _rtk_config(rtk):
    """The RTK model of a spec with the placeholder defaults filled in (ValueError on an unknown key or type)."""
    unknown = sorted(set(rtk) - set(RTK_KEYS))
    if unknown:
        raise ValueError('unknown gnss rtk keys: %s' % ', '.join(unknown))
    out = dict(default_type=rtk.get('default_type', 'fixed'), windows=[list(w) for w in rtk.get('windows') or ()])
    for name in ('fixed', 'float'):
        model = dict(RTK_MODELS[name])
        extra = sorted(set(rtk.get(name) or {}) - set(model))
        if extra:
            raise ValueError('unknown gnss rtk %s keys: %s' % (name, ', '.join(extra)))
        model.update(rtk.get(name) or {})
        out[name] = model
    for kind in [out['default_type']] + [w[2] for w in out['windows']]:
        if kind not in RTK_TYPES:
            raise ValueError('gnss rtk type %r not in %s' % (kind, RTK_TYPES))
    return out


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
        self.mirror = bool(values['frame_mirror'])
        self.scale = tuple(float(v) for v in values['frame_scale'])
        self.rtk = None if values['rtk'] is None else _rtk_config(values['rtk'])
        if self.rtk is not None:
            # drawn after the general receiver's draws: the general streams stay as they were
            self.rtk_walk = {name: self.rng.normal(0., float(self.rtk[name]['walk_sigma_m']), 2)
                             for name in ('fixed', 'float')}
            self.counts.update({'rtk_' + kind: 0 for kind in RTK_TYPES})

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
        kind = None
        if self.rtk is not None:
            rtk_white, rtk_jitter = {}, {}
            for name in ('fixed', 'float'):
                model = self.rtk[name]
                ar = math.exp(-dt / float(model['walk_tau_s']))
                self.rtk_walk[name] = (self.rtk_walk[name] * ar
                                       + self.rng.normal(0., float(model['walk_sigma_m']) * math.sqrt(1. - ar * ar), 2))
                rtk_white[name] = self.rng.normal(0., float(model['white_sigma_m']), 2)
                rtk_jitter[name] = float(self.rng.normal(0., float(model['hacc_jitter_m'])))
            window = self._window(self.rtk['windows'], t_s)
            kind = self.rtk['default_type'] if window is None else window[2]
        dropped = self._window(v['dropouts'], t_s) is not None
        if dropped and v['dropout_mode'] == 'silent':
            self.counts['suppressed'] += 1
            return None
        c, s = math.cos(base_yaw), math.sin(base_yaw)
        ax = base_x + c * self.antenna[0] - s * self.antenna[1]
        ay = base_y + s * self.antenna[0] + c * self.antenna[1]
        if kind in ('fixed', 'float'):
            error = scale * (self.rtk_walk[kind] + rtk_white[kind])
        else:
            error = self.bias + scale * (self.walk + white)
        px, py = self.rot[0] * ax - self.rot[1] * ay, self.rot[1] * ax + self.rot[0] * ay
        px, py = self.scale[0] * px, self.scale[1] * py
        if self.mirror:
            px, py = py, px
        east = px + self.offset[0] + float(error[0])
        north = py + self.offset[1] + float(error[1])
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
        if kind in ('fixed', 'float'):
            model = self.rtk[kind]
            hacc = max(.005, scale * (float(model['hacc_m']) + rtk_jitter[kind]))
        self.counts['emitted'] += 1
        extra = {} if kind is None else dict(fix_type=kind)
        if dropped or kind == 'none':
            self.counts['no_fix'] += 1
            if kind is not None:
                self.counts['rtk_none'] += 1
            return dict(east_m=float('nan'), north_m=float('nan'), hacc_m=None, fix_ok=False,
                        latency_s=float(v['latency_s']), **({} if kind is None else dict(fix_type='none')))
        if kind is not None:
            self.counts['rtk_' + kind] += 1
        if kind in ('fixed', 'float'):
            return dict(east_m=round(east, 4), north_m=round(north, 4), hacc_m=round(hacc, 4), fix_ok=True,
                        latency_s=float(v['latency_s']), **extra)
        return dict(east_m=round(east, 3), north_m=round(north, 3), hacc_m=round(hacc, 2), fix_ok=True,
                    latency_s=float(v['latency_s']), **extra)

    def report(self):
        """Evaluation counters (never an executor input)."""
        keep = {k: self.v[k] for k in self.v if k in DEFAULTS and k != 'antenna_xy_m'
                and not (k in NEW_KEYS and self.v[k] == DEFAULTS[k])}
        return dict(counts=dict(self.counts), jump_stamps=list(self.jump_stamps), period_ms=self.period_ms,
                    spec=json.loads(canonical(keep | {'antenna_xy_m': list(self.antenna)})))
