"""The executor's optional perception layers of design v2: L2 local grid (PC-07) and L3 GNSS (PC-12) for one Runner.

The Runner builds a PerceptionLayers only when a layer is on (perception_settings.enabled: local_grid_enabled or
gnss_mode != 'off'); with both off nothing here runs and the Runner is bit-identical to the version without it.

  new_run(t, row_length_m)   a new run: a new grid (the estimator, and so the odometry frame, is new too) and a new
                             GNSS run state (the fence row length)
  on_scan(est, stamp, ...)   after every processed scan (the converted ranges the estimator got): the grid update every
                             local_grid_every_n_scans scans; with local_grid_fusion_gain > 0 the new fit goes to
                             est.on_corridor_observation(stamp, lateral, heading, width or None, quality, 'local_grid')
                             (the estimator's own gates decide; a refused observation changes nothing)
  on_gnss(t, msg, est)       a GNSS message received at executor time t (gnss_layer.fix_from_message)
  tick(now, est, snapshot)   once per Runner tick: the GNSS layer's pose bookkeeping; mode weak_constraint: the RTK
                             constraint's pending fixes and its rate-limited correction (gnss_rtk.py)
  controller_snapshot(s)     the snapshot the controller gets: s itself, or (weak_constraint, correction not 0) a copy
                             with the bounded RTK lateral correction
  end_reason()               'gnss_geofence' (mode geofence / weak_constraint, fence tripped) or None
  summary(now, est)          {'local_grid': ..., 'gnss': ...} blocks of the Runner summary (proc_ms = wall clock); the
                             'gnss' block gets 'rtk' in mode weak_constraint
  export(now, est)           the bounded grid serialisation for the operator map (rate-limited), or None

The grid sees the estimator only through its public outputs (base_pose, odom_valid, config.lidar_x_in_base_m, the band
heading of the snapshot); the GNSS layer and the RTK constraint never write into the estimator.
"""
import math
import time

from . import perception_settings as PS
from .gnss_layer import GnssLayer, LocalFrame, fix_from_message
from .gnss_rtk import RtkConstraint, fix_type_from_message
from .local_grid import SOURCE_NAME, LocalGrid


class PerceptionLayers:
    def __init__(self, cfg):
        self.grid_settings = PS.grid_settings(cfg) if cfg.local_grid_enabled else None
        self.fusion_gain = float(cfg.local_grid_fusion_gain)
        self.fuse_width = bool(cfg.local_grid_fuse_width)
        self.publish = bool(cfg.local_grid_publish)
        self.publish_period_s = float(cfg.local_grid_publish_period_s)
        self.publish_max_bytes = int(cfg.local_grid_publish_max_bytes)
        self.gnss_settings = PS.gnss_settings(cfg) if cfg.gnss_mode != 'off' else None
        self.gnss = None if self.gnss_settings is None else GnssLayer(self.gnss_settings)
        # RTK weak constraint (mode weak_constraint only; None otherwise and nothing below calls it)
        self.rtk = RtkConstraint(PS.rtk_settings(cfg)) if cfg.gnss_mode == 'weak_constraint' else None
        self.frame = LocalFrame()           # latitude / longitude -> local metres (origin private, never exported)
        self.grid = None
        self.fusion = {}                    # outcome -> count of the fused observations of this run
        self.gnss_unreadable = 0
        self._export = None
        self._export_t = None
        self._band_heading = None

    def new_run(self, t, row_length_m):
        self.grid = None if self.grid_settings is None else LocalGrid(self.grid_settings)
        self.fusion = {}
        self._export, self._export_t = None, None
        self._band_heading = None
        if self.gnss is not None:
            self.gnss.new_run(row_length_m)
        if self.rtk is not None:
            self.rtk.new_run()

    # --- inputs --------------------------------------------------------------------------------------------------
    def on_scan(self, est, stamp, ranges, angle_min, increment):
        grid = self.grid
        if grid is None:
            return None
        base = est.base_pose(stamp)
        lever = float(est.config.lidar_x_in_base_m)
        laser = (base[0] + lever * math.cos(base[2]), base[1] + lever * math.sin(base[2]), base[2])
        band = est.band
        prior = lines = None
        if band is not None:
            prior = float(band.heading)
            lines = (float(band.center) + .5 * float(band.width), float(band.center) - .5 * float(band.width))
        fit = grid.on_scan(stamp, ranges, angle_min, increment, laser, bool(est.odom_valid), base, prior, lines)
        if fit is None or self.fusion_gain <= 0.:
            return fit
        lateral, heading = fit.observation(base)
        outcome = est.on_corridor_observation(stamp, lateral, heading, fit.width if self.fuse_width else None,
                                              fit.quality, SOURCE_NAME)
        key = str(outcome)
        self.fusion[key] = self.fusion.get(key, 0) + 1
        return fit

    def on_gnss(self, t, msg, est):
        if self.gnss is None:
            return None
        fix = fix_from_message(msg, self.frame)
        if fix is None:
            self.gnss_unreadable += 1
            return 'unreadable'
        base = None if est is None else est.base_pose(float(t))
        band = None if est is None or est.band is None else float(est.band.heading)
        if self.rtk is None:
            return self.gnss.on_fix(t, fix[0], fix[1], fix[2], fix[3], base, band)
        flags = sum(self.gnss.flags.values())
        outcome = self.gnss.on_fix(t, fix[0], fix[1], fix[2], fix[3], base, band)
        self.rtk.on_fix(t, fix[0], fix[1], fix[2], fix_type_from_message(msg), outcome,
                        sum(self.gnss.flags.values()) > flags)
        return outcome

    def tick(self, now, est, snapshot=None):
        if self.gnss is not None and est is not None:
            self.gnss.note_pose(est.base_pose(float(now)))
        if self.rtk is not None and est is not None and snapshot is not None:
            self.rtk.tick(now, snapshot, est.base_pose(float(now)))

    def controller_snapshot(self, s):
        """The controller's snapshot (weak_constraint: lateral_offset + the bounded RTK correction; otherwise s)."""
        return s if self.rtk is None else self.rtk.apply(s)

    # --- outputs -------------------------------------------------------------------------------------------------
    def end_reason(self):
        return None if self.gnss is None else self.gnss.end_reason()

    def grid_age_s(self, now):
        """Age of the fit the fusion would use (A_map of design V2-4), None without a fit."""
        if self.grid is None or self.grid.fit is None:
            return None
        return float(now) - self.grid.fit.stamp

    def summary(self, now, est=None):
        out = {}
        if self.grid is not None:
            block = self.grid.summary(now)
            if self.grid.fit is not None and est is not None:
                lateral, heading = self.grid.fit.observation(est.base_pose(float(now)))
                block['fit'].update(lateral_m=round(lateral, 4), heading_deg=round(math.degrees(heading), 3))
            if self.fusion_gain > 0.:
                block['fusion'] = dict(gain=self.fusion_gain, outcomes=dict(sorted(self.fusion.items())))
            out['local_grid'] = block
        elif self.grid_settings is not None:
            out['local_grid'] = dict(updates=0, reason='no_run')
        if self.gnss is not None:
            block = self.gnss.summary(now)
            if self.gnss_unreadable:
                block['unreadable'] = self.gnss_unreadable
            if self.rtk is not None:
                block['rtk'] = self.rtk.summary(now)
            out['gnss'] = block
        return out

    def export(self, now, est=None):
        if not self.publish or self.grid is None:
            return None
        now = float(now)
        if self._export is not None and self._export_t is not None and now - self._export_t < self.publish_period_s:
            return self._export
        t0 = time.perf_counter()
        base = None if est is None else est.base_pose(now)
        data = self.grid.export(now, base, max_bytes=self.publish_max_bytes)
        data['proc_ms'] = round((time.perf_counter() - t0) * 1e3, 3)
        self._export, self._export_t = data, now
        return data
