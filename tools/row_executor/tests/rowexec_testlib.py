"""Shared helpers for the row_executor tests (path setup, synthetic scenes, a fake-time driver).

Runs under pytest and plain unittest (WSL without pytest). No bytecode is written.
"""
import math
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

# Real RPLIDAR S2 scan geometry (10/10 recordings: 3240 beams, angle_min = float32(-pi)).
RAW_AMIN = -3.1415927410125732
RAW_INC = .0019398535368964076
BEAMS = 3240
OFFSET_DEG = 179.08
LIDAR_X = .18


def dash_dir():
    """The read-only dashboard folder: row_executor.sim_paths 'dashboard_dir' (UBTS_DASH_DIR, the path config file, or
    this repository's dashboard/ folder, same rover_control_core.py bytes as the real dashboard). RuntimeError when it
    holds no scripts/rover_control_core.py (steering branch: no built-in machine path)."""
    from row_executor import sim_paths
    try:
        root = str(sim_paths.configured_path('dashboard_dir', 'dir'))
    except FileNotFoundError as exc:
        raise RuntimeError(str(exc)) from None
    if not os.path.isfile(os.path.join(root, 'scripts', 'rover_control_core.py')):
        raise RuntimeError('%s has no scripts/rover_control_core.py (set UBTS_DASH_DIR)' % root)
    return root


def import_dash(name):
    """Import scripts.<name> from the read-only dashboard folder (dash_dir(); its scripts/ package, never the top-level
    scripts/ folder of this repository)."""
    import importlib
    root = dash_dir()
    if root in sys.path:
        sys.path.remove(root)
    sys.path.insert(0, root)
    package = sys.modules.get('scripts')
    if package is not None:
        origin = os.path.realpath(os.path.dirname(getattr(package, '__file__', '') or ''))
        if origin != os.path.realpath(os.path.join(root, 'scripts')):
            for key in [k for k in sys.modules if k == 'scripts' or k.startswith('scripts.')]:
                del sys.modules[key]
    module = importlib.import_module('scripts.' + name)
    if os.path.realpath(os.path.dirname(module.__file__)) != os.path.realpath(os.path.join(root, 'scripts')):
        raise RuntimeError('scripts.%s resolved outside dashboard-live-raw: %s' % (name, module.__file__))
    return module


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def body_bearing_to_raw_index(bearing_rad, offset_deg=OFFSET_DEG):
    raw = wrap(bearing_rad - math.radians(offset_deg))
    return int(round((raw - RAW_AMIN) / RAW_INC)) % BEAMS


def orchard_trunks(row_y=1.75, spacing=.5, x0=-8., x1=14., radius=.12, extra=()):
    """Trunk circles (x, y, radius) in the estimator frame at the LiDAR (+x = travel, +y = left)."""
    trunks = []
    x = x0
    while x <= x1 + 1e-9:
        trunks.append((x, row_y, radius))
        trunks.append((x, -row_y, radius))
        x += spacing
    return trunks + list(extra)


def raw_scan(trunks, self_returns=(), offset_deg=OFFSET_DEG, beams=BEAMS, zero_no_return=True):
    """Raw-order ranges (LaserScan.ranges) of a static scene; 0 = no return like the real driver.

    self_returns: ((body_bearing_rad, range_m, half_width_beams), ...) e.g. a mount bracket.
    """
    import numpy as np
    amin_body = wrap(RAW_AMIN + math.radians(offset_deg))
    angles = amin_body + np.arange(beams) * RAW_INC
    dx, dy = np.cos(angles), np.sin(angles)
    r = np.full(beams, np.inf)
    for cx, cy, rho in trunks:
        b = dx * cx + dy * cy
        disc = b * b - (cx * cx + cy * cy - rho * rho)
        ok = disc >= 0
        t = np.where(ok, b - np.sqrt(np.where(ok, disc, 0.)), np.inf)
        t[t <= 0] = np.inf
        r = np.minimum(r, t)
    for bearing, rng, half in self_returns:
        centre = body_bearing_to_raw_index(bearing, offset_deg)
        for k in range(centre - half, centre + half + 1):
            r[k % beams] = rng
    r[r >= 30.] = np.inf
    if zero_no_return:
        r = np.where(np.isfinite(r), r, 0.)
    return r.astype(np.float32).astype(np.float64)


class FakeTimeDriver:
    """Feeds a Runner at 33 Hz IMU / 10 Hz scans and ticks it at 20 Hz on a fake monotonic clock."""

    def __init__(self, runner, scan_fn, t0=1000., acc=(0., 0., 9.81)):
        self.runner = runner
        self.scan_fn = scan_fn          # t -> ranges (raw order) or None (no scan delivered)
        self.t = t0
        self.acc = acc
        self.imu_on = True
        self.scan_on = True
        self.next_imu = t0
        self.next_scan = t0 + .013
        self.log = []

    def run(self, duration, hold=False):
        """Advance by duration seconds; hold is a bool or a function of t. Returns the decisions."""
        out = []
        end = self.t + duration - 1e-9
        while self.t + .05 <= end + 1e-6:
            target = self.t + .05
            while min(self.next_imu, self.next_scan) <= target:
                if self.next_imu <= self.next_scan:
                    if self.imu_on:
                        self.runner.feed_imu(self.next_imu, (0., 0., 0.), self.acc)
                    self.next_imu += 1 / 33.
                else:
                    if self.scan_on:
                        ranges = self.scan_fn(self.next_scan)
                        if ranges is not None:
                            self.runner.feed_scan(self.next_scan, ranges, RAW_AMIN, RAW_INC, 30., .05)
                    self.next_scan += .1
            self.t = target
            h = hold(self.t) if callable(hold) else hold
            decision = self.runner.tick(self.t, h)
            out.append((self.t, decision))
        self.log.extend(out)
        return out


def first(decisions, predicate):
    for t, d in decisions:
        if predicate(d):
            return t, d
    return None, None
