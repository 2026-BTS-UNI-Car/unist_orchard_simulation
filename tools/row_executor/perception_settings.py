"""Settings of the executor's optional perception layers L2 (local grid) and L3 (GNSS) - standard library only.

config.py (stdlib only) reads the field specification and the rule checks from here; local_grid.py, gnss_layer.py and
perception_layers.py (numpy) take their settings objects from here. Design v2 V2-1 3 (GNSS), V2-3 (local grid); work
items PC-07, PC-12. Every value is a placeholder (SIM development value), not a field calibration.

ExecutorConfig keys (blocks local_grid_* and gnss_*; an untouched block is left out of to_dict() / digest(), so existing
configurations keep their digest; both layers are OFF by default and the Runner is then bit-identical):
  local_grid_enabled            build the grid of every run (display, fit, optional fusion)
  local_grid_cell_m / _length_m / _width_m / _every_n_scans / _free_beam_step / _min_hits   GridSettings
  local_grid_fusion_gain        > 0: the estimator gets the grid lines as corridor observations of source 'local_grid'
                                (EstimatorConfig external_corridor_gain / _sources); 0 = display and log only
  local_grid_fuse_width         also pull the band width (default False: the grid lines lie a trunk radius further out)
  local_grid_publish            put a bounded grid serialisation into the executor status (operator map)
  local_grid_publish_period_s / _publish_max_bytes   at most every period, text size cap
  gnss_mode                     'off' | 'log_only' | 'geofence' (design open choice 4 proposes log_only in the field
                                file; the dataclass default stays 'off' so existing configurations are unchanged)
  gnss_max_hacc_m / _max_age_s / _jump_gate_m / _fence_margin_m / _fence_confirm / _course_baseline_m /
  _course_tolerance_deg         GnssSettings
"""
from dataclasses import dataclass
import math

GNSS_MODES = ('off', 'log_only', 'geofence')
_REAL = 'real'                 # the config.py field-spec kind of a finite number


@dataclass(frozen=True)
class GridSettings:
    cell_m: float = .10
    length_m: float = 20.
    width_m: float = 10.
    behind_m: float = 6.
    every_n_scans: int = 2
    max_range_m: float = 12.
    free_beam_step: int = 0
    min_hits: int = 3
    free_ratio: float = 1.
    recentre_m: float = 1.
    fit_behind_m: float = 4.
    fit_ahead_m: float = 10.
    fit_cross_min_m: float = .5
    fit_cross_max_m: float = 3.5
    fit_window_deg: float = 15.
    fit_track_deg: float = 4.
    fit_step_deg: float = .5
    fit_track_s: float = 2.
    fit_along_bin_m: float = .5
    fit_cross_bin_m: float = .1
    fit_min_coverage: int = 5
    fit_full_coverage: int = 8
    fit_gate_m: float = .25
    fit_trim_m: float = .18
    fit_min_cells: int = 6
    width_min_m: float = 1.2
    width_max_m: float = 8.

    def problems(self):
        """Range and relation problems (empty = valid)."""
        p = _range_problems(self, (
            ('cell_m', .02, .5), ('length_m', 4., 60.), ('width_m', 2., 30.), ('behind_m', 0., 30.),
            ('every_n_scans', 1, 10), ('max_range_m', 2., 30.), ('free_beam_step', 0, 64), ('min_hits', 1, 50),
            ('free_ratio', 0., 10.), ('recentre_m', .1, 10.), ('fit_behind_m', 0., 20.), ('fit_ahead_m', 1., 30.),
            ('fit_cross_min_m', 0., 2.), ('fit_cross_max_m', 1., 15.), ('fit_window_deg', 1., 60.),
            ('fit_track_deg', .5, 30.), ('fit_step_deg', .05, 5.), ('fit_track_s', 0., 10.),
            ('fit_along_bin_m', .1, 3.), ('fit_cross_bin_m', .02, .5), ('fit_min_coverage', 1, 100),
            ('fit_full_coverage', 1, 200), ('fit_gate_m', .05, 1.), ('fit_trim_m', .05, 1.), ('fit_min_cells', 2, 500),
            ('width_min_m', .5, 10.), ('width_max_m', 1., 20.)))
        if p:
            return p
        if self.behind_m >= self.length_m:
            p.append('behind_m must be below length_m')
        if self.fit_cross_min_m >= self.fit_cross_max_m:
            p.append('fit_cross_min_m must be below fit_cross_max_m')
        if self.fit_cross_max_m > self.width_m / 2:
            p.append('fit_cross_max_m must lie inside the window (width_m / 2)')
        if self.fit_behind_m > self.behind_m or self.fit_ahead_m > self.length_m - self.behind_m:
            p.append('the fit region must lie inside the window')
        if self.width_min_m >= self.width_max_m:
            p.append('width_min_m must be below width_max_m')
        if self.fit_full_coverage < self.fit_min_coverage:
            p.append('fit_full_coverage must be >= fit_min_coverage')
        if self.length_m / self.cell_m * self.width_m / self.cell_m > 4e6:
            p.append('too many cells (length x width / cell^2 > 4e6)')
        return p


@dataclass(frozen=True)
class GnssSettings:
    mode: str = 'off'
    max_hacc_m: float = 10.          # design V2-1 3: horizontal accuracy <= 10 m
    max_age_s: float = 2.5           # ... and age <= 2.5 s
    jump_gate_m: float = 3.          # innovation against the dead-reckoned last accepted fix (see gnss_layer)
    jump_max_gap_s: float = 5.       # a gap longer than this restarts the track instead of gating
    reanchor_after: int = 3          # consecutive mutually consistent rejected fixes restart the track
    fence_margin_m: float = 10.      # design V2-1 3: margin max(10 m, 3 x hacc)
    fence_hacc_factor: float = 3.
    fence_confirm: int = 3           # consecutive accepted fixes outside the fence before it trips
    course_baseline_m: float = 15.   # odometry path before the alignment and of one course window
    course_tolerance_deg: float = 45.  # a 15 m window of a 7 m-class fix is +-10-30 deg off (SIM): gross check only
    disagree_factor: float = 3.      # position disagreement flag above max(fence_margin_m, factor x hacc)

    def problems(self):
        p = []
        if self.mode not in GNSS_MODES:
            p.append('mode: must be one of %s' % (GNSS_MODES,))
        p += _range_problems(self, (
            ('max_hacc_m', .5, 50.), ('max_age_s', .2, 10.), ('jump_gate_m', .5, 50.), ('jump_max_gap_s', 1., 60.),
            ('reanchor_after', 1, 20), ('fence_margin_m', 2., 100.), ('fence_hacc_factor', 1., 10.),
            ('fence_confirm', 1, 20), ('course_baseline_m', 2., 100.), ('course_tolerance_deg', 5., 90.),
            ('disagree_factor', 1., 10.)))
        return p


def _range_problems(obj, checks):
    p = []
    for name, lo, hi in checks:
        value = getattr(obj, name)
        if isinstance(lo, int) and isinstance(hi, int):
            if type(value) is not int:
                p.append('%s: must be an integer' % name)
                continue
        elif type(value) not in (int, float) or not math.isfinite(value):
            p.append('%s: must be a finite number' % name)
            continue
        if not lo <= value <= hi:
            p.append('%s: %r outside [%g, %g]' % (name, value, lo, hi))
    return p


# --- ExecutorConfig block (config.py) ------------------------------------------------------------------------------------
# Field specification in the config.py format (kind, lo, hi[, 'nullable']); the defaults are the dataclass defaults of
# ExecutorConfig (config.py) and must equal GridSettings / GnssSettings.
CONFIG_SPEC = {
    'local_grid_enabled': (bool,), 'local_grid_cell_m': (_REAL, .02, .5), 'local_grid_length_m': (_REAL, 4., 60.),
    'local_grid_width_m': (_REAL, 2., 30.), 'local_grid_every_n_scans': (int, 1, 10),
    'local_grid_free_beam_step': (int, 0, 64), 'local_grid_min_hits': (int, 1, 50),
    'local_grid_fusion_gain': (_REAL, 0., .5), 'local_grid_fuse_width': (bool,), 'local_grid_publish': (bool,),
    'local_grid_publish_period_s': (_REAL, .2, 10.), 'local_grid_publish_max_bytes': (int, 1024, 262144),
    'gnss_mode': (str,), 'gnss_max_hacc_m': (_REAL, .5, 50.), 'gnss_max_age_s': (_REAL, .2, 10.),
    'gnss_jump_gate_m': (_REAL, .5, 50.), 'gnss_fence_margin_m': (_REAL, 2., 100.), 'gnss_fence_confirm': (int, 1, 20),
    'gnss_course_baseline_m': (_REAL, 2., 100.), 'gnss_course_tolerance_deg': (_REAL, 5., 90.),
}


def grid_settings(cfg):
    """GridSettings of an ExecutorConfig (the window and fit values not in the config keep their defaults; behind_m
    keeps the 30 % share of the length, the fit region is clipped into the window)."""
    length, width = float(cfg.local_grid_length_m), float(cfg.local_grid_width_m)
    base = GridSettings()
    behind = round(length * base.behind_m / base.length_m, 6)
    return GridSettings(cell_m=float(cfg.local_grid_cell_m), length_m=length, width_m=width, behind_m=behind,
                        every_n_scans=int(cfg.local_grid_every_n_scans),
                        free_beam_step=int(cfg.local_grid_free_beam_step), min_hits=int(cfg.local_grid_min_hits),
                        fit_behind_m=min(base.fit_behind_m, behind), fit_ahead_m=min(base.fit_ahead_m, length - behind),
                        fit_cross_max_m=min(base.fit_cross_max_m, width / 2))


def gnss_settings(cfg):
    return GnssSettings(mode=cfg.gnss_mode, max_hacc_m=float(cfg.gnss_max_hacc_m), max_age_s=float(cfg.gnss_max_age_s),
                        jump_gate_m=float(cfg.gnss_jump_gate_m), fence_margin_m=float(cfg.gnss_fence_margin_m),
                        fence_confirm=int(cfg.gnss_fence_confirm), course_baseline_m=float(cfg.gnss_course_baseline_m),
                        course_tolerance_deg=float(cfg.gnss_course_tolerance_deg))


def enabled(cfg):
    """True when any perception layer is on (the Runner builds PerceptionLayers only then)."""
    return bool(getattr(cfg, 'local_grid_enabled', False)) or getattr(cfg, 'gnss_mode', 'off') != 'off'


def rule_problems(cfg):
    """config.py _rule_problems part of the two blocks."""
    p = []
    if cfg.local_grid_fusion_gain > 0. and not cfg.local_grid_enabled:
        p.append('local_grid_fusion_gain > 0 needs local_grid_enabled')
    if (cfg.local_grid_publish or cfg.local_grid_fuse_width) and not cfg.local_grid_enabled:
        p.append('local_grid_publish / local_grid_fuse_width need local_grid_enabled')
    if cfg.local_grid_enabled:
        p += ['local_grid: ' + text for text in grid_settings(cfg).problems()]
    if cfg.gnss_mode not in GNSS_MODES:
        p.append('gnss_mode: must be one of %s' % (GNSS_MODES,))
    elif cfg.gnss_mode != 'off':
        p += ['gnss: ' + text for text in gnss_settings(cfg).problems()]
    return p


def estimator_fields(cfg):
    """EstimatorConfig overrides of the grid fusion (empty while the gain is 0: the estimator is then unchanged)."""
    gain = float(cfg.local_grid_fusion_gain)
    if gain <= 0.:
        return {}
    return dict(external_corridor_gain=gain, external_corridor_sources=('local_grid',))
