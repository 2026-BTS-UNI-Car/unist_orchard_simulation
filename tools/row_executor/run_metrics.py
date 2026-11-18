"""Run metrics of the row executor: the vehicle's path against the reference centreline (2026-11-19 field request).

Standard library only, incremental sums (a few float operations per tick: the Jetson is CPU-limited). Display, status and
log only: nothing here reaches the estimator, the controller or a command.

One RunMetrics per run (Runner._begin makes a new one); it keeps its values after the run ended until the next Start, so
the status shows the last run's numbers. A sample is one executor tick of a row run in state ACTIVE with a drive command
(throttle > 0: the vehicle is driven; the standstill alignment and the stops are not samples).

References (neither is ground truth: both come from the vehicle's own LiDAR):
  estimator  the corridor centreline of the estimator band, i.e. the snapshot's lateral_offset / heading_error (the
             values the controller steers on; 'lat' / 'head' of the tick log). Sampled when the corridor is valid.
  grid       the centreline of the local occupancy grid's two-line fit (local_grid.GridFit: the midline of the two
             fitted row lines / corridor sides) seen from the same base pose; only with local_grid_enabled and a fit at
             most perception_layers.GRID_REFERENCE_MAX_AGE_S old. The grid keeps several scans (persistence), so it is
             a slower, partly independent check of the fast estimator band.
Lateral: + = base left of the centreline [m]; heading error: corridor axis - vehicle heading [deg] (estimator sign; the
tick log's head < 0 = nose LEFT). Distances: the estimator's resolved path distance between samples (>= 0).

summary() fields (status.metrics, run_end event 'metrics'):
  ref                 'estimator' (always) or 'estimator+grid' (when the grid reference had samples)
  samples, distance_m, time_s          driving ticks, path and time over them
  lat_rms_m, lat_mean_m, lat_max_abs_m, within_0_05_pct, within_0_10_pct, head_rms_deg, head_max_abs_deg
                      estimator reference (within_*: share of the samples with |lat| <= 0.05 / 0.10 m, percent)
  grid_samples, grid_lat_rms_m, grid_lat_mean_m, grid_lat_max_abs_m, grid_within_0_05_pct, grid_within_0_10_pct,
  grid_head_rms_deg, grid_head_max_abs_deg   the same against the grid centreline (None without grid samples)
  ref_diff_lat_rms_m, ref_diff_head_rms_deg  RMS of (estimator - grid) over the ticks with both references
  last                {lat_m, head_deg, grid_lat_m, grid_head_deg} of the newest sample
"""
import math

WITHIN_M = (.05, .10)


class _Stats:
    __slots__ = ('n', 'lat_sum', 'lat_sq', 'lat_max', 'head_sq', 'head_max', 'within')

    def __init__(self):
        self.n = 0
        self.lat_sum = self.lat_sq = self.lat_max = 0.
        self.head_sq = self.head_max = 0.
        self.within = [0] * len(WITHIN_M)

    def add(self, lat, head_deg):
        self.n += 1
        self.lat_sum += lat
        self.lat_sq += lat * lat
        a = abs(lat)
        if a > self.lat_max:
            self.lat_max = a
        self.head_sq += head_deg * head_deg
        h = abs(head_deg)
        if h > self.head_max:
            self.head_max = h
        for i, limit in enumerate(WITHIN_M):
            if a <= limit:
                self.within[i] += 1

    def fields(self, prefix=''):
        n = self.n
        if not n:
            return {prefix + k: None for k in ('lat_rms_m', 'lat_mean_m', 'lat_max_abs_m', 'within_0_05_pct',
                                               'within_0_10_pct', 'head_rms_deg', 'head_max_abs_deg')}
        return {prefix + 'lat_rms_m': round(math.sqrt(self.lat_sq / n), 4),
                prefix + 'lat_mean_m': round(self.lat_sum / n, 4),
                prefix + 'lat_max_abs_m': round(self.lat_max, 4),
                prefix + 'within_0_05_pct': round(100. * self.within[0] / n, 1),
                prefix + 'within_0_10_pct': round(100. * self.within[1] / n, 1),
                prefix + 'head_rms_deg': round(math.sqrt(self.head_sq / n), 3),
                prefix + 'head_max_abs_deg': round(self.head_max, 3)}


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class RunMetrics:
    """Path-against-centreline numbers of one run (module docstring)."""

    def __init__(self, run_id=None):
        self.run_id = run_id
        self.est = _Stats()
        self.grid = _Stats()
        self.diff_n = 0
        self.diff_lat_sq = self.diff_head_sq = 0.
        self.distance_m = 0.
        self.time_s = 0.
        self._last_t = None
        self._last_d = None
        self.last = None

    def update(self, t, snapshot, driving, grid_ref=None):
        """One tick. snapshot: the Runner's estimator snapshot (lateral_offset, heading_error, corridor_valid,
        distance_m); driving: a drive command this tick (throttle > 0, state ACTIVE); grid_ref: (lateral_m, heading_rad)
        against the grid centreline or None. Returns True when the tick was a sample."""
        distance = snapshot.get('distance_m') if isinstance(snapshot, dict) else None
        if not driving or not isinstance(snapshot, dict):
            self._last_t = None
            self._last_d = distance if _finite(distance) else None
            return False
        t = float(t)
        if self._last_t is not None and t > self._last_t:
            self.time_s += min(t - self._last_t, .5)          # a stall longer than 0.5 s is not counted as driving
        self._last_t = t
        if _finite(distance):
            if self._last_d is not None and distance > self._last_d:
                self.distance_m += distance - self._last_d
            self._last_d = distance
        lat, head = snapshot.get('lateral_offset'), snapshot.get('heading_error')
        est = snapshot.get('corridor_valid') is True and _finite(lat) and _finite(head)
        if est:
            self.est.add(float(lat), math.degrees(float(head)))
        g = None
        if grid_ref is not None and _finite(grid_ref[0]) and _finite(grid_ref[1]):
            g = (float(grid_ref[0]), math.degrees(float(grid_ref[1])))
            self.grid.add(g[0], g[1])
            if est:
                self.diff_n += 1
                self.diff_lat_sq += (float(lat) - g[0]) ** 2
                self.diff_head_sq += (math.degrees(float(head)) - g[1]) ** 2
        if est or g is not None:
            self.last = dict(lat_m=round(float(lat), 3) if est else None,
                             head_deg=round(math.degrees(float(head)), 2) if est else None,
                             grid_lat_m=None if g is None else round(g[0], 3),
                             grid_head_deg=None if g is None else round(g[1], 2))
        return est or g is not None

    def summary(self):
        out = dict(run_id=self.run_id, ref='estimator+grid' if self.grid.n else 'estimator', samples=self.est.n,
                   distance_m=round(self.distance_m, 3), time_s=round(self.time_s, 2))
        out.update(self.est.fields())
        out['grid_samples'] = self.grid.n
        out.update(self.grid.fields('grid_'))
        out['ref_diff_lat_rms_m'] = round(math.sqrt(self.diff_lat_sq / self.diff_n), 4) if self.diff_n else None
        out['ref_diff_head_rms_deg'] = round(math.sqrt(self.diff_head_sq / self.diff_n), 3) if self.diff_n else None
        out['last'] = None if self.last is None else dict(self.last)
        return out
