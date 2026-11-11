"""Steering episode evaluator (design 7.1, 6.5, 6.6, 7.2 A21 evaluator part; 8.3 P2).

Evaluates a fast_sim trace (steering/fast_sim.py, schema ubts.steer_sim.fast_sim_trace.v1) with the metrics of
the single-row pre-registration (single-row-20261102/PREREG.json 'metrics', 'episode_termination'; the code is
closed_loop/raster_corridor.evaluate_episode) and the steering additions of design 7.1. The PREREG helpers and
constants are imported from closed_loop.raster_corridor (unchanged): _stats, _runs, stop_category, time_cap_s,
ROW_END_WINDOWS, REST_*, MISSED_MARGIN_M, LEFT_FIELD_*, NO_PROGRESS_*, the cell classes and CLEARANCE_GROUPS.

Truth: this module is evaluation code (A13 allowlist 'evaluator'). It reads the truth_* trace columns; nothing
here feeds an executor, an estimator or a controller.

Frames (design 3.1): base = footprint centre; field = corridor frame (W2: s along the centreline, d left of
travel positive, psi = wrap(yaw - tangent)); W0 and a world-less call use the world frame as the field
(s = x, d = y). Points (rear axle, front bumper, footprint) are projected into the field with the corridor frame
(closed_loop.raster_corridor.CurvilinearFrame.project_many), so curved centrelines are exact.

Footprint (design 3.2, 7.1): geometry.SteeringGeometry.footprint_polygons(delta_L, delta_R) with the bracket
margin = body rectangle + the two steered front wheels at their TRUE wheel angles (trace truth_delta_left_rad /
truth_delta_right_rad) + the rear wheels. 'swept' metrics use these polygons; 'body' metrics use the body
rectangle alone (with the placeholder profile it is the PREREG 1.22 x 0.70 m rectangle centred on the base).
Wheel angles are rounded to 1e-5 rad for the footprint cache (< 1e-6 m change of any corner).

Rules (one per name; every result carries its definition in 'definitions'):
  lateral / heading       PREREG: base d and psi in the in-row window (first motion .. terminal rest, s in
                          [0, s_E]); RMSE, p95 |d|, max; also rear-axle and front-bumper d (report) and
                          distance-weighted RMSE / p95 of the base d (single_row_eval rule, report)
  safety band             PREREG form |d_base| <= band_m, band_m = 3.7/2 - 0.60 - half width - 0.30 (0.60 m with the
                          placeholder profile), every sample from first motion to rest; steering form (design 7.1):
                          the swept footprint's reach in the corridor normal stays within 3.7/2 - 0.60 - 0.30 =
                          0.95 m of the centreline (equal to the PREREG form at heading 0 with the wheels inside the
                          body); both reported, PREREG_STEER fixes which one is judged
  departure               CORRIDOR_DEPARTURE, steering form (design 7.1): the swept footprint reaches a row line,
                          max d > spacing(s)/2 or min d < -spacing(s)/2 (spacing = left - right row d of the catalogue
                          profile at the base s); PREREG body form |d_base| > spacing/2 - half width is reported
  contacts                swept footprint vs the world: raster cells (conservative full-cell enclosure: cell-centre
                          signed distance to the polygons - res/sqrt 2, PREREG contact_accounting) or W0 posts
                          (exact circle); groups = closed_loop.raster_corridor.CLEARANCE_GROUPS; body-only values too
  termination             PREREG episode_termination applied post hoc in sample order (COLLISION with the swept
                          footprint, MISSED_ROW_END, CORRIDOR_DEPARTURE (steering form), LEFT_FIELD, TIME_CAP,
                          NO_PROGRESS), else TERMINAL_STOP at the final rest of >= 1 s, else OPEN; a rule needs its
                          inputs (s_E, spacing, a world) and is skipped without them ('rules_evaluated')
  stop position           e = s_front at rest - s_E, s_front = s of base + front_bumper_x_in_base_m along the heading;
                          FALSE / EARLY / OK / LATE (raster_corridor.ROW_END_WINDOWS) / MISSED; overshoot
  stop reason             registered table (raster_corridor.stop_category) and, for the new steering reasons of
                          design 6.6, PROPOSED_STEERING_STOP_TABLE (a proposal; PREREG_STEER fixes it)
  steering saturation     |delta_cmd| >= 0.95 x the equivalent soft limit of that side, over driving samples
                          (forward drive command, command defined): fraction and longest run; rate-limited fraction
                          (true wheel-side rate >= 0.95 x the rate limit, moving or standstill); command-rate fraction
                          (servo target changes faster than the moving rate limit)
  steering activity       total variation of delta_cmd per metre (deg/m), sign changes per 10 m (hysteresis
                          +- sign_threshold, default the servo deadband), p95 of the command and the true rates
  steering tracking       p95 |servo target - measured| and p95 |measured - true wheel-side angle| (SIM only)
  wheel limits (A21)      samples with a true wheel angle beyond its mechanical limit (per wheel, per direction);
                          largest wheel sweep beyond the body half width (with the bracket margin)
  REVERSE                 any drive command < 0 (design 7.1: PIVOT_OR_REVERSE -> REVERSE)
  AUTO_RESTART            drive based: a non-zero drive command after the vehicle rested with drive command 0 for
                          >= 0.5 s after first motion (steering commands do not count)
  STEER_AFTER_TERMINAL    after a terminal state (terminal_t_s given; else every STOP command (stop mode in effect)
                          after first motion; else the start of the terminal rest) and until the drive restarts:
                          (a) the commanded steering (servo target) moves beyond the hold tolerance max(1 deg, 2 x
                          servo deadband) from the command at the terminal row, or (b) the stop moves the command by
                          more than the tolerance to somewhere other than the steering position at the stop
                          (hold reference, hold_reference(): truth_hold_position_rad = the MCU filtered potentiometer
                          position a hold_measured STOP holds (executor / T1 rig traces, R13-2 / R13-3), else the
                          SteeredPlant hold_measured target recomputed from pot_raw (fast_sim, a row per plant step),
                          else truth_servo_view_rad, steer_measured_rad; hold_measured holding the
                          position is allowed, the centre policy is not); the potentiometer decode is not a reference
                          of (a) (sag / noise, review R1) (design 6.5); 'auto_centre' marks a move toward 0 (A22); the
                          true wheel-side drift after the terminal moment is reported (A17 input)
  rest clearance          PREREG obstacle_wall_clearance: the swept footprint's (and the body's) smallest clearance
                          to ADDED_HARD cells over the terminal rest rows with the 6 m reach (raster worlds)
  stop response           per stop trigger (stop mode in effect, or the drive command falling to 0): time to drive
                          output 0 and to rest, distance, sideways shift relative to the heading at the trigger and
                          the true steering change during the stop, per mode (report)
  corridor metrics        lateral, heading and the safety bands need a corridor: a W2 world or a closed-loop driver
                          on W0 (its line y = 0); evaluate_run marks them not applicable for open-loop W0 maneuvers

Evaluation grid: the PREREG metrics use 20 Hz samples (t multiple of sample_dt_s = 0.05 s, like the harness control
ticks); a trace whose rows do not give that grid (trace_every not 1 or 5) is refused unless require_grid=False, and
the result records the spacing used; STEER_AFTER_TERMINAL, REVERSE, wheel limits and the stop response use every
trace row.

Every result from the placeholder profile is a placeholder-based SIM result (자리값 기반 SIM 결과), not field
validation. CLI (from sim/ubts): python -B -m steering.evaluate_steer run --run <fast_sim run folder> [--out FILE]
                                 python -B -m steering.evaluate_steer a21 --out <folder outside the repository>
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

from closed_loop import raster_corridor as rc
from . import geometry as geo
from .vehicle_profile import load_profile

SCHEMA = 'ubts.steer_sim.steer_evaluation.v1'
RUN_SCHEMA = 'ubts.steer_sim.steer_evaluation_run.v1'
A21_SCHEMA = 'ubts.steer_sim.a21_evaluator.v1'
RESULT_KIND_FALLBACK = '자리값 기반 SIM 결과'
DEFAULT_SAMPLE_DT_S = .05            # PREREG 20 Hz samples (harness control ticks)
ROW_HALF_SPACING_M = 3.7 / 2         # PREREG safety_band_derivation: lower typical row half spacing
CANOPY_REACH_M = .60                 # PREREG: conservative canopy reach at the 0.6 m scan height
CLEARANCE_MARGIN_M = .30             # PREREG: clearance margin (same as the obstacle / wall stop clearance)
SATURATION_FRACTION = .95            # design 7.1 / profile steering_authority_insufficient.limit_fraction
IDLE_BEFORE_RESTART_S = .5           # PREREG forbidden_commands AUTO_RESTART
HOLD_TOLERANCE_FLOOR_DEG = 1.        # design 6.5: max(1 deg, 2 x servo deadband)
AUTHORITY_FRACTION_FLAG, AUTHORITY_RUN_FLAG_S = .20, 3.   # PREREG pwm_saturation AUTHORITY_MARGIN_LOW (report)
CONTACT_REACH_M = 2.                 # raster_corridor.footprint_clearances default reach
FOOTPRINT_DIGITS = 5                 # wheel-angle rounding of the footprint cache (rad)
STOP_NAMES = {0: None, 1: 'release', 2: 'brake', 3: 'coast'}   # fast_sim STOP_CODES
PREREG_FAIL_CODES = ('COLLISION', 'CONTACT_MOVING', 'CORRIDOR_DEPARTURE', 'MISSED_ROW_END', 'LEFT_FIELD',
                     'NO_PROGRESS', 'TIME_CAP')
STEERING_FAIL_CODES = ('REVERSE', 'AUTO_RESTART', 'STEER_AFTER_TERMINAL', 'WHEEL_LIMIT', 'SAFETY_BAND',
                       'SAFETY_BAND_SWEPT')
# Design 6.6 new stop reasons -> stop category. A PROPOSAL of this evaluator: the steering pre-registration
# (PREREG_STEER) fixes the table before the first evaluation run and may only make entries less favourable.
# Actuator-chain faults are treated like the registered DRIVE sub-label (CONTROL_AUTHORITY); an unconfirmed STOP
# is a safety-system fault.
PROPOSED_STEERING_STOP_TABLE = {
    'steering_feedback_invalid': 'CONTROL_AUTHORITY',
    'steering_feedback_stale': 'CONTROL_AUTHORITY',
    'steering_not_following': 'CONTROL_AUTHORITY',
    'steering_stalled': 'CONTROL_AUTHORITY',
    'steering_ineffective': 'CONTROL_AUTHORITY',
    'steering_uncalibrated': 'CONTROL_AUTHORITY',
    'steering_polarity_fault': 'CONTROL_AUTHORITY',
    'steering_driver_fault': 'CONTROL_AUTHORITY',
    'steering_authority_insufficient': 'CONTROL_AUTHORITY',     # also in the registered table (same category)
    'stop_unconfirmed': 'SYSTEM_FAULT',
    # executor link / session faults of the front-steer executor (row_executor config.STEER_LINK_FAULT_REASONS, P4)
    'mcu_reset': 'SYSTEM_FAULT', 'link_watchdog': 'SYSTEM_FAULT', 'link_bad_lines': 'SYSTEM_FAULT',
    'link_ttl_expired': 'SYSTEM_FAULT', 'link_command_rejected': 'SYSTEM_FAULT', 'link_hardware_fault': 'SYSTEM_FAULT',
    'mcu_state_mismatch': 'SYSTEM_FAULT', 'arm_unconfirmed': 'SYSTEM_FAULT', 'arm_rejected': 'SYSTEM_FAULT',
    'link_identity_mismatch': 'SYSTEM_FAULT', 'bridge_internal_error': 'SYSTEM_FAULT', 'mcu_disarmed': 'SYSTEM_FAULT',
}
PROPOSED_TABLE_SOURCE = 'evaluate_steer.PROPOSED_STEERING_STOP_TABLE (proposal; PREREG_STEER fixes it)'
REQUIRED_COLUMNS = ('t_s', 'truth_x_m', 'truth_y_m', 'truth_yaw_rad', 'truth_v_m_s', 'truth_delta_left_rad',
                    'truth_delta_right_rad')
# STEER_AFTER_TERMINAL rule (b) hold reference (R13-2 / R13-3; hold_reference()): the executor / T1 rig trace column
# truth_hold_position_rad (the MCU filtered potentiometer position: what a hold_measured STOP holds); else, for a trace
# with a row every plant step (fast_sim trace_every 1), the SteeredPlant hold_measured target recomputed from the
# potentiometer columns (median of the last pot_median_of pot_raw samples, decoded: 'measured_hold_recomputed', review
# P4R-03); else the columns below, first present wins: the fast_sim servo view (traces before R13-2, or thinned rows),
# the potentiometer decode (older traces).
HOLD_POSITION_COLUMNS = (('truth_hold_position_rad', 'truth_hold_position'), ('truth_servo_view_rad', 'truth_servo_view'),
                         ('steer_measured_rad', 'measured_fallback'))


class EvaluationError(ValueError):
    """The trace or the evaluation inputs cannot be evaluated."""


# --- configuration -------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class EvalConfig:
    """Profile-derived constants of the evaluator (build with EvalConfig.from_profile)."""

    geometry: geo.SteeringGeometry
    soft_limits_eq_rad: tuple            # (left, right) equivalent soft limits, positive (rad)
    rate_moving_rad_s: float             # steer_rate_max_deg_s
    rate_standstill_rad_s: float         # steer_rate_standstill_deg_s
    servo_deadband_rad: float
    hold_tolerance_rad: float            # max(1 deg, 2 x servo deadband)
    sign_threshold_rad: float            # hysteresis of the sign-change count (default: the servo deadband)
    profile_id: str = ''
    profile_sha256: str = ''
    result_kind: str = RESULT_KIND_FALLBACK
    row_half_spacing_m: float = ROW_HALF_SPACING_M
    canopy_reach_m: float = CANOPY_REACH_M
    clearance_margin_m: float = CLEARANCE_MARGIN_M
    saturation_fraction: float = SATURATION_FRACTION
    sample_dt_s: float = DEFAULT_SAMPLE_DT_S
    rest_v_m_s: float = rc.REST_V_M_S
    rest_w_rad_s: float = rc.REST_W_RAD_S
    rest_hold_s: float = rc.REST_HOLD_S
    idle_before_restart_s: float = IDLE_BEFORE_RESTART_S
    contact_reach_m: float = CONTACT_REACH_M
    pot_median_of: int = 5               # pot_filter.median_of: the hold_measured median of SteeredPlant (R13-2)

    @classmethod
    def from_profile(cls, profile=None, **changes):
        """From a vehicle_profile.VehicleProfile (None: the placeholder profile). Use the scene's profile view
        (fast_sim.scene_params(scene).profile) so the Ackermann ratio and the limits match the plant."""
        profile = load_profile() if profile is None else profile
        geometry = geo.SteeringGeometry.from_profile(profile)
        deadband = math.radians(float(profile.value('servo_deadband_deg')))
        label = profile.result_label() if hasattr(profile, 'result_label') else {}
        values = dict(geometry=geometry, soft_limits_eq_rad=tuple(geometry.soft_limits_eq()),
                      pot_median_of=int(profile.value('pot_filter')['median_of']),
                      rate_moving_rad_s=math.radians(float(profile.value('steer_rate_max_deg_s'))),
                      rate_standstill_rad_s=math.radians(float(profile.value('steer_rate_standstill_deg_s'))),
                      servo_deadband_rad=deadband,
                      hold_tolerance_rad=max(math.radians(HOLD_TOLERANCE_FLOOR_DEG), 2. * deadband),
                      sign_threshold_rad=deadband, profile_id=str(getattr(profile, 'profile_id', '')),
                      profile_sha256=str(getattr(profile, 'sha256', '')),
                      result_kind=str(label.get('result_kind', RESULT_KIND_FALLBACK)))
        values.update(changes)
        return cls(**values)

    @property
    def body_half_width_m(self):
        return self.geometry.footprint_width_m / 2.

    @property
    def band_m(self):
        """PREREG safety band re-derived with the profile's body half width (0.60 m with the placeholder)."""
        return self.row_half_spacing_m - self.canopy_reach_m - self.body_half_width_m - self.clearance_margin_m

    @property
    def band_edge_m(self):
        """|d| of the band edge for the swept footprint itself (band + half width = 0.95 m)."""
        return self.row_half_spacing_m - self.canopy_reach_m - self.clearance_margin_m

    def describe(self):
        g = self.geometry
        return dict(profile_id=self.profile_id, profile_sha256=self.profile_sha256, result_kind=self.result_kind,
                    soft_limits_eq_deg=[round(math.degrees(v), 4) for v in self.soft_limits_eq_rad],
                    wheel_mech_limits_deg={k: round(math.degrees(v), 4) for k, v in g.wheel_limits().items()},
                    rate_moving_deg_s=round(math.degrees(self.rate_moving_rad_s), 4),
                    rate_standstill_deg_s=round(math.degrees(self.rate_standstill_rad_s), 4),
                    servo_deadband_deg=round(math.degrees(self.servo_deadband_rad), 4),
                    hold_tolerance_deg=round(math.degrees(self.hold_tolerance_rad), 4),
                    sign_threshold_deg=round(math.degrees(self.sign_threshold_rad), 4),
                    band_m=round(self.band_m, 4), band_edge_m=round(self.band_edge_m, 4),
                    band_derivation='%.2f - %.2f (canopy) - %.3f (half width) - %.2f (margin)' % (
                        self.row_half_spacing_m, self.canopy_reach_m, self.body_half_width_m, self.clearance_margin_m),
                    footprint=dict(length_m=g.footprint_length_m, width_m=g.footprint_width_m,
                                   front_bumper_x_in_base_m=g.front_bumper_x_in_base_m,
                                   rear_axle_x_in_base_m=g.rear_axle_x_in_base_m,
                                   hardware_margin_m=g.hardware_margin_m),
                    saturation_fraction=self.saturation_fraction, sample_dt_s=self.sample_dt_s,
                    rest=dict(v_m_s=self.rest_v_m_s, w_rad_s=self.rest_w_rad_s, hold_s=self.rest_hold_s),
                    idle_before_restart_s=self.idle_before_restart_s, contact_reach_m=self.contact_reach_m)


# --- small helpers ----------------------------------------------------------------------------------------------------
def plain(value):
    """JSON-safe copy: numpy scalars to Python, non-finite floats to None, tuples to lists."""
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, (float, np.floating)):
        value = float(value)
        return value if math.isfinite(value) else None
    if isinstance(value, np.ndarray):
        return plain(value.tolist())
    if isinstance(value, Path):
        return str(value)
    return value


def _round(value, digits=4):
    if value is None:
        return None
    value = float(value)
    return round(value, digits) if math.isfinite(value) else None


def weighted_percentile(values, weights, q):
    """The rule of closed_loop.single_row_eval._weighted_percentile (copied: that module imports the harness, which
    needs scipy, so it cannot be imported on every supported Python; a test compares the two where it imports)."""
    values = np.asarray(values, dtype=float)
    weights = np.asarray(weights, dtype=float)
    order = np.argsort(values)
    values, weights = values[order], weights[order]
    cumulative = np.cumsum(weights)
    if cumulative[-1] <= 0:
        return float(np.percentile(values, q))
    return float(values[min(len(values) - 1, int(np.searchsorted(cumulative, q / 100. * cumulative[-1])))])


def _first(mask):
    idx = np.flatnonzero(mask)
    return int(idx[0]) if len(idx) else None


def _column(arrays, name, required=False):
    if name in arrays:
        return np.asarray(arrays[name])
    if required:
        raise EvaluationError('trace_column_missing:%s' % name)
    return None


def grid_indices(t, sample_dt_s):
    """Indices of the rows on the evaluation grid (t a multiple of sample_dt_s within 1e-6 s); all rows when
    sample_dt_s is None or when no row lies on the grid."""
    t = np.asarray(t, dtype=float)
    if sample_dt_s is None or not len(t):
        return np.arange(len(t))
    k = t / float(sample_dt_s)
    on = np.abs(k - np.round(k)) * float(sample_dt_s) < 1e-6
    return np.flatnonzero(on) if on.any() else np.arange(len(t))


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


# --- footprint geometry -----------------------------------------------------------------------------------------------
class FootprintCache:
    """Body-frame footprint polygons (numpy, counter-clockwise) and their hull for rounded wheel angles."""

    NAMES = ('body', 'front_left', 'front_right', 'rear_left', 'rear_right')

    def __init__(self, geometry, include_hardware_margin=True, digits=FOOTPRINT_DIGITS):
        self.geometry = geometry
        self.include_hardware_margin = bool(include_hardware_margin)
        self.digits = int(digits)
        self._polygons, self._hulls, self._sweeps = {}, {}, {}
        self.body = _ccw(np.asarray(geometry.body_polygon(), dtype=float))

    def key(self, delta_left, delta_right):
        return round(float(delta_left), self.digits), round(float(delta_right), self.digits)

    def polygons(self, delta_left, delta_right):
        key = self.key(delta_left, delta_right)
        if key not in self._polygons:
            raw = self.geometry.footprint_polygons(key[0], key[1], self.include_hardware_margin)
            self._polygons[key] = {name: _ccw(np.asarray(raw[name], dtype=float)) for name in self.NAMES}
        return self._polygons[key]

    def hull(self, delta_left, delta_right):
        key = self.key(delta_left, delta_right)
        if key not in self._hulls:
            points = np.concatenate(list(self.polygons(*key).values()))
            self._hulls[key] = _ccw(np.asarray(geo.convex_hull(points.tolist()), dtype=float))
        return self._hulls[key]

    def sweeps(self, delta_left, delta_right):
        """(left wheel, right wheel) reach beyond the body half width, with the bracket margin (m)."""
        key = self.key(delta_left, delta_right)
        if key not in self._sweeps:
            g = self.geometry
            self._sweeps[key] = (g.wheel_sweep_beyond_body(geo.LEFT, key[0], self.include_hardware_margin),
                                 g.wheel_sweep_beyond_body(geo.RIGHT, key[1], self.include_hardware_margin))
        return self._sweeps[key]


def _ccw(points):
    """The polygon in counter-clockwise order (reversed if its signed area is negative)."""
    x, y = points[:, 0], points[:, 1]
    area = float(np.dot(x[:-1], y[1:]) - np.dot(x[1:], y[:-1]) + x[-1] * y[0] - x[0] * y[-1])
    return points[::-1].copy() if area < 0 else points


def to_world(points, x, y, yaw):
    """Body-frame points (N, 2) -> world (N, 2) for the base pose (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    px, py = points[:, 0], points[:, 1]
    return np.column_stack((x + c * px - s * py, y + s * px + c * py))


def signed_distance_convex(px, py, polygon):
    """Signed distance of points to a counter-clockwise convex polygon: > 0 outside (Euclidean distance to the
    boundary), <= 0 inside (minus the distance to the nearest edge line)."""
    px = np.asarray(px, dtype=float).reshape(-1, 1)
    py = np.asarray(py, dtype=float).reshape(-1, 1)
    ax, ay = polygon[:, 0][None, :], polygon[:, 1][None, :]
    bx, by = np.roll(polygon[:, 0], -1)[None, :], np.roll(polygon[:, 1], -1)[None, :]
    ex, ey = bx - ax, by - ay
    length2 = ex * ex + ey * ey
    length = np.sqrt(length2)
    valid = length > 0
    line = np.where(valid, -(ex * (py - ay) - ey * (px - ax)) / np.where(valid, length, 1.), -np.inf)
    inside = np.all(line <= 0., axis=1)
    t = np.clip(((px - ax) * ex + (py - ay) * ey) / np.where(valid, length2, 1.), 0., 1.)
    seg = np.hypot(px - (ax + t * ex), py - (ay + t * ey))
    seg = np.where(valid, seg, np.hypot(px - ax, py - ay))
    return np.where(inside, line.max(axis=1), seg.min(axis=1))


# --- world access -----------------------------------------------------------------------------------------------------
def world_kind(world):
    if world is None:
        return 'none'
    if hasattr(world, 'map') and hasattr(world, 'frame'):
        return 'raster'
    if hasattr(world, 'posts'):
        return 'posts'
    raise EvaluationError('world_kind_unknown:%s' % type(world).__name__)


class FieldFrame:
    """Projection of world points into the field: a CurvilinearFrame (W2) or the identity (W0 / no world)."""

    def __init__(self, curvilinear=None):
        self.curvilinear = curvilinear

    def project(self, xs, ys):
        xs = np.asarray(xs, dtype=float).ravel()
        ys = np.asarray(ys, dtype=float).ravel()
        if self.curvilinear is None:
            return xs.copy(), ys.copy()
        return self.curvilinear.project_many(xs, ys)

    def heading_error(self, xs, ys, yaws):
        """psi = wrap(yaw - tangent of the closest segment) (raster_corridor.CurvilinearFrame.to_field)."""
        yaws = np.asarray(yaws, dtype=float).ravel()
        if self.curvilinear is None:
            return wrap(yaws)
        return np.array([self.curvilinear.to_field(float(x), float(y), float(a))[2]
                         for x, y, a in zip(np.ravel(xs), np.ravel(ys), yaws)])


def corridor_spacing_function(world, spacing_m=None):
    """spacing(s) of the departure rule: the catalogue profile of a raster world (left - right row d, linear in s),
    else the constant spacing_m, else None (rule not evaluated)."""
    config = getattr(world, 'config', None)
    if config is not None and getattr(config, 'profile_s_m', None):
        profile_s = np.asarray(config.profile_s_m, dtype=float)
        left = np.asarray(config.left_row_d_m, dtype=float)
        right = np.asarray(config.right_row_d_m, dtype=float)
        return lambda s: np.interp(s, profile_s, left) - np.interp(s, profile_s, right)
    if spacing_m is not None:
        value = float(spacing_m)
        return lambda s: np.full(np.shape(s), value, dtype=float)
    return None


def _raster_cells(rmap, x, y, radius):
    """(cx, cy, class) of the non-FREE cells within the square of half side `radius` around (x, y); cells outside
    the raster count as UNOBSERVED (the cell gathering of raster_corridor.footprint_clearances)."""
    res = rmap.resolution
    r_lo = int(math.floor((y - radius) / res)) - rmap.row0
    r_hi = int(math.floor((y + radius) / res)) - rmap.row0 + 1
    c_lo = int(math.floor((x - radius) / res)) - rmap.col0
    c_hi = int(math.floor((x + radius) / res)) - rmap.col0 + 1
    sub = np.full((r_hi - r_lo, c_hi - c_lo), rc.UNOBSERVED, dtype=np.uint8)
    a0, a1 = max(r_lo, 0), min(r_hi, rmap.shape[0])
    b0, b1 = max(c_lo, 0), min(c_hi, rmap.shape[1])
    if a1 > a0 and b1 > b0:
        sub[a0 - r_lo:a1 - r_lo, b0 - c_lo:b1 - c_lo] = rmap.classes[a0:a1, b0:b1]
    rows, cols = np.nonzero(sub != rc.FREE)
    cls = sub[rows, cols]
    cx = (cols + c_lo + rmap.col0 + .5) * res
    cy = (rows + r_lo + rmap.row0 + .5) * res
    return cx, cy, cls


_GROUP_CLASSES = (('hard', (rc.OCCUPIED, rc.ADDED_HARD)), ('added_hard', (rc.ADDED_HARD,)),
                  ('branch', (rc.ADDED_BRANCH,)), ('uncertain', (rc.UNCERTAIN,)),
                  ('uncertain_occ', (rc.UNCERTAIN_OCC,)), ('unobserved', (rc.UNOBSERVED,)))


def polygon_clearances(world, polygons_world, x, y, reach_m=CONTACT_REACH_M):
    """{'swept': {group: clearance}, 'body': {...}} of world polygons {name: (K, 2)} around the base (x, y).

    raster world: conservative full-cell enclosure (cell-centre signed distance - res / sqrt 2), the cells within
    the polygons' radius + reach_m of the base; inf = nothing of that group within reach (PREREG accounting).
    posts (W0): exact distance from the post circle to the polygons, group 'hard'; other groups inf."""
    kind = world_kind(world)
    out = {key: {name: math.inf for name in rc.CLEARANCE_GROUPS} for key in ('swept', 'body')}
    if kind == 'none':
        return out
    if kind == 'posts':
        posts = np.asarray(world.posts, dtype=float).reshape(-1, 3)
        if not len(posts):
            return out
        sd = {name: signed_distance_convex(posts[:, 0], posts[:, 1], poly) - posts[:, 2]
              for name, poly in polygons_world.items()}
        out['swept']['hard'] = float(np.min([v.min() for v in sd.values()]))
        out['body']['hard'] = float(sd['body'].min())
        return out
    rmap = world.map
    res = rmap.resolution
    radius = max(float(np.max(np.hypot(p[:, 0] - x, p[:, 1] - y))) for p in polygons_world.values()) \
        + reach_m + res
    cx, cy, cls = _raster_cells(rmap, x, y, radius)
    if not len(cx):
        return out
    sd = {name: signed_distance_convex(cx, cy, poly) for name, poly in polygons_world.items()}
    enclosure = res / math.sqrt(2.)
    swept = np.min(np.vstack(list(sd.values())), axis=0) - enclosure
    body = sd['body'] - enclosure
    for name, classes in _GROUP_CLASSES:
        mask = np.isin(cls, classes)
        if mask.any():
            out['swept'][name] = float(swept[mask].min())
            out['body'][name] = float(body[mask].min())
    return out


def world_for_scene(scene):
    """The evaluation world of a fast_sim scene: W0 -> flat_world.FlatWorld with the scene posts; W2 -> the raster
    corridor world (closed_loop.raster_corridor; needs the configured maps and catalogue)."""
    return world_for_spec(scene.world)


def world_for_spec(world):
    """The evaluation world of a scene_spec.WorldSpec (or its dict form): W0 or W2 as world_for_scene."""
    from .scene_spec import WorldSpec
    spec = world if isinstance(world, WorldSpec) else WorldSpec.from_dict(world)
    if spec.kind == 'W0':
        from . import flat_world as fw
        return fw.FlatWorld(fw.FlatWorldConfig(markers=spec.markers))
    if spec.kind != 'W2':
        raise EvaluationError('world_kind_not_supported_here: %s' % spec.kind)
    from .scene_spec import sealed_status
    if sealed_status(spec.record, spec.corridor_id, spec.direction) is not None and not spec.allow_sealed:
        raise EvaluationError('sealed_corridor_refused (reserved for the steering pre-registration)')
    modifiers = tuple(rc.modifier_from_dict(m) for m in spec.modifiers)
    config = rc.corridor_world_config(spec.record, spec.corridor_id, spec.direction, modifiers=modifiers)
    return rc.build_raster_world(config)


# --- stop reasons -----------------------------------------------------------------------------------------------------
def steer_stop_category(reason):
    """(category, table label, table source) of a terminal reason: the registered table first
    (raster_corridor.stop_category), then the proposed steering table for the design 6.6 reasons."""
    category, raw, source = rc.stop_category(reason)
    if category == 'UNCLASSIFIED' and reason in PROPOSED_STEERING_STOP_TABLE:
        return PROPOSED_STEERING_STOP_TABLE[reason], PROPOSED_STEERING_STOP_TABLE[reason], PROPOSED_TABLE_SOURCE
    return category, raw, source


# --- the evaluator ----------------------------------------------------------------------------------------------------
def _stats_deg(values):
    stats = rc._stats(np.degrees(values), 3)
    return {k: v for k, v in stats.items() if k != 'p95'}


def _distance_weighted(d, s, mask):
    """Distance-weighted RMSE and p95 of |d| over consecutive grid samples inside the mask (single_row_eval)."""
    idx = np.flatnonzero(mask)
    if len(idx) < 2:
        return dict(rmse=None, p95=None)
    pairs = idx[1:][np.diff(idx) == 1]
    if not len(pairs):
        return dict(rmse=None, p95=None)
    values = np.abs(d[pairs])
    weights = np.abs(s[pairs] - s[pairs - 1])
    total = float(weights.sum())
    if total <= 0:
        return dict(rmse=None, p95=None)
    return dict(rmse=round(float(np.sqrt(np.sum(weights * values ** 2) / total)), 4),
                p95=round(weighted_percentile(values, weights, 95), 4))


def _sign_changes(values, threshold):
    count, state = 0, 0
    for value in values:
        if not math.isfinite(value):
            continue
        new = 1 if value > threshold else (-1 if value < -threshold else state)
        if state and new != state:
            count += 1
        state = new
    return count


def _terminal_starts(t, stop_code, first_motion, rest_start, terminal_t_s):
    """Row indices where a terminal window starts (see STEER_AFTER_TERMINAL) and the source of each."""
    if terminal_t_s is not None:
        i = _first(t >= float(terminal_t_s) - 1e-9)
        return [] if i is None else [(i, 'terminal_t_s')]
    starts = []
    if stop_code is not None and first_motion is not None:
        active = stop_code != 0
        edges = np.flatnonzero(active & ~np.concatenate(([False], active[:-1])))
        starts = [(int(i), 'stop_command') for i in edges if t[i] >= t[first_motion] - 1e-9]
    if not starts and rest_start is not None:
        starts = [(int(rest_start), 'terminal_rest')]
    return starts


def measured_hold_rows(pot_raw, decoded, median_of):
    """Per row j: the SteeredPlant hold_measured target after the plant step of row j (R13-2), i.e. the decode of the
    sorted element median_of // 2 of the pot_raw samples of rows j - median_of + 1 .. j (SteeredPlant.measured_hold_rad;
    in the first median_of - 1 rows the first row refills the missing samples, where the plant also has the sample of
    its reset). decoded[k] is the nominal decode of pot_raw[k] (fast_sim steer_measured_rad), so the result is the
    decoded value of the row holding the chosen raw value. Needs a row per plant step (trace_every 1)."""
    raw = np.asarray(pot_raw).astype(np.int64)
    dec = np.asarray(decoded, dtype=float)
    n = int(median_of)
    if not len(raw):
        return dec.copy()
    index = np.maximum(np.arange(len(raw))[:, None] + np.arange(-n + 1, 1)[None, :], 0)
    order = np.argsort(raw[index], axis=1, kind='stable')
    return dec[index[np.arange(len(raw)), order[:, n // 2]]]


def hold_reference(arrays, meta, config, force=None):
    """(rows or None, source) of the STEER_AFTER_TERMINAL rule (b) hold reference (see HOLD_POSITION_COLUMNS).
    force: None (automatic), 'truth_hold_position', 'measured_hold_recomputed', 'truth_servo_view' or
    'measured_fallback' (e.g. 'truth_servo_view' for a fast_sim trace made before R13-2)."""
    choices = []
    if _column(arrays, 'truth_hold_position_rad') is not None:
        choices.append(('truth_hold_position', lambda: np.asarray(arrays['truth_hold_position_rad'], dtype=float)))
    if (_column(arrays, 'pot_raw') is not None and _column(arrays, 'steer_measured_rad') is not None
            and meta.get('trace_every') == 1):
        choices.append(('measured_hold_recomputed', lambda: measured_hold_rows(
            arrays['pot_raw'], arrays['steer_measured_rad'], config.pot_median_of)))
    for name, source in HOLD_POSITION_COLUMNS[1:]:
        if _column(arrays, name) is not None:
            choices.append((source, (lambda n=name: np.asarray(arrays[n], dtype=float))))
    for source, make in choices:
        if force is None or source == force:
            return make(), source
    if force is not None:
        raise EvaluationError('hold reference %r is not available in this trace' % force)
    return None, 'command_before'


def steer_after_terminal(t, target, drive_cmd, stop_code, v, starts, tolerance, rest_v, *, hold_position=None,
                         hold_source='command_before', true_eq=None):
    """STEER_AFTER_TERMINAL windows (every trace row; design 6.5, A22 'no commanded steering move after a terminal
    state'). The commanded steering is the servo target. Per window from row i (j0 = i - 1) to the drive restart:

      (a) the command moves after the terminal row: max |target[k] - target[i]| > tolerance;
      (b) the stop itself moves the command: |target[i] - target[j0]| > tolerance AND the new command is not the
          steering position at the stop (|target[i] - hold_position[j0]| > tolerance). Holding the position where it
          is (the hold_measured policy) is allowed; the centre policy or a new command is not.
          hold_position = hold_reference(): truth_hold_position_rad (executor / T1 rig traces: the MCU filtered
          potentiometer position, the target a hold_measured STOP holds; R13-2 / R13-3), the SteeredPlant hold_measured
          target recomputed from the pot_raw rows (fast_sim with a row per plant step; review P4R-03),
          truth_servo_view_rad (fast_sim: the servo-side angle; traces before R13-2 or thinned rows),
          steer_measured_rad (the potentiometer decode: sag and noise can exceed the tolerance, older traces only),
          else the command before the stop (then any change counts).

    The potentiometer decode is never the reference of (a): under the L1 model it differs from the servo target by
    the supply sag and the noise with an unchanged command (review R1 false positives). auto_centre (A22): a move of
    kind (b) toward 0, or a command move toward 0 at rest after the terminal row. Report only (A17 input, judged in
    P4): the true wheel-side drift after the terminal moment (true_eq = truth_delta_eq_rad) and whether it exceeds the
    tolerance. Returns the rule result dict."""
    windows = []
    for i, source in starts:
        j0 = max(i - 1, 0)
        restart = None
        if drive_cmd is not None:
            after = np.flatnonzero((np.arange(len(t)) > i) & (drive_cmd != 0)
                                   & ((stop_code == 0) if stop_code is not None else True))
            restart = int(after[0]) if len(after) else None
        end = restart if restart is not None else len(t)
        segment = slice(i, end)
        commanded = float(target[i])
        before = float(target[j0])
        position = float(hold_position[j0]) if hold_position is not None else before
        seg_target = target[segment]
        dev_cmd = float(np.max(np.abs(seg_target - commanded))) if len(seg_target) else 0.
        stop_change = commanded - before
        hold_gap = abs(commanded - position)
        stop_moved = abs(stop_change) > tolerance and hold_gap > tolerance
        at_rest = np.abs(v[segment]) < rest_v
        toward_zero_after = np.abs(seg_target) < abs(commanded) - tolerance
        stop_centred = stop_moved and abs(commanded) < min(abs(before), abs(position)) - tolerance
        auto_centre = bool(stop_centred or np.any(at_rest & toward_zero_after))
        violated = stop_moved or dev_cmd > tolerance
        first_violation = None
        if stop_moved:
            first_violation = round(float(t[i]), 4)
        elif violated:
            k = _first(np.abs(seg_target - commanded) > tolerance)
            first_violation = None if k is None else round(float(t[i + k]), 4)
        drift = None
        if true_eq is not None and end > i:
            drift = float(np.max(np.abs(true_eq[segment] - true_eq[j0])))
        windows.append(dict(start_t_s=round(float(t[i]), 4), source=source,
                            end_t_s=round(float(t[end - 1]), 4), ended_by_restart=restart is not None,
                            command_before_deg=round(math.degrees(before), 4),
                            command_at_start_deg=round(math.degrees(commanded), 4),
                            hold_position_deg=round(math.degrees(position), 4), hold_position_source=hold_source,
                            stop_change_deg=round(math.degrees(stop_change), 4),
                            stop_hold_gap_deg=round(math.degrees(hold_gap), 4), stop_moved_command=bool(stop_moved),
                            max_dev_from_command_deg=round(math.degrees(dev_cmd), 4),
                            auto_centre=auto_centre, violated=bool(violated), first_violation_t_s=first_violation,
                            true_drift_deg=None if drift is None else round(math.degrees(drift), 4),
                            true_drift_over_tolerance=None if drift is None else bool(drift > tolerance)))
    return dict(evaluated=True, tolerance_deg=round(math.degrees(tolerance), 4), windows=windows,
                hold_position_source=hold_source,
                flag=any(w['violated'] for w in windows), auto_centre=any(w['auto_centre'] for w in windows),
                true_drift_over_tolerance=any(bool(w['true_drift_over_tolerance']) for w in windows))


def stop_response(t, v, x, y, yaw, drive_cmd, drive_applied, stop_code, delta_eq, field_d, first_motion, rest_v):
    """Per stop trigger after first motion: stop mode in effect (0 -> mode) or the drive command falling to 0."""
    triggers = []
    if first_motion is None:
        return triggers
    n = len(t)
    stop_edge = np.zeros(n, dtype=bool)
    if stop_code is not None:
        stop_edge[1:] = (stop_code[1:] != 0) & (stop_code[:-1] == 0)
    zero_edge = np.zeros(n, dtype=bool)
    if drive_cmd is not None:
        zero_edge[1:] = (drive_cmd[1:] == 0) & (drive_cmd[:-1] > 0)
        if stop_code is not None:
            zero_edge &= stop_code == 0
    candidates = np.flatnonzero((np.arange(n) >= max(first_motion, 1)) & (stop_edge | zero_edge))
    for i in (int(k) for k in candidates):
        mode = STOP_NAMES.get(int(stop_code[i]), 'unknown') if stop_edge[i] else 'throttle_zero'
        j0 = i - 1
        zero_drive = _first(drive_applied[i:] == 0.) if drive_applied is not None else None
        rest = _first(np.abs(v[i:]) < rest_v)
        entry = dict(t0_s=round(float(t[j0]), 4), mode=mode, v0_m_s=round(float(v[j0]), 4),
                     time_to_drive_zero_s=None if zero_drive is None else round(float(t[i + zero_drive] - t[j0]), 4),
                     completed=rest is not None)
        if rest is not None:
            j = i + rest
            dx, dy = float(x[j] - x[j0]), float(y[j] - y[j0])
            c, s = math.cos(float(yaw[j0])), math.sin(float(yaw[j0]))
            entry.update(time_to_rest_s=round(float(t[j] - t[j0]), 4), distance_m=round(math.hypot(dx, dy), 4),
                         sideways_shift_m=round(-s * dx + c * dy, 4),
                         field_d_change_m=None if field_d is None else round(float(field_d[j] - field_d[j0]), 4),
                         steer_change_deg=round(math.degrees(float(delta_eq[j] - delta_eq[j0])), 4))
        triggers.append(entry)
    return triggers


def grid_spacing(t, rows):
    """Spacing (s) of the evaluation rows: the median difference, and whether every difference is equal (1e-6 s)."""
    diffs = np.diff(np.asarray(t, dtype=float)[rows])
    if not len(diffs):
        return None, True
    median = float(np.median(diffs))
    return median, bool(np.all(np.abs(diffs - median) <= 1e-6))


def evaluate_trace(arrays, config=None, *, world=None, s_end=None, spacing_m=None, s_start=None, final_reason=None,
                   terminal_t_s=None, prereg_termination=True, class_limits=None, include_hardware_margin=True,
                   corridor_metrics=True, episode=None, require_grid=True, hold_reference_source=None):
    """Evaluate one fast_sim trace (arrays dict as fast_sim.load_trace returns). See the module docstring.
    hold_reference_source: None (automatic, hold_reference()) or one source name to force (e.g. 'truth_servo_view' for
    a fast_sim trace made before R13-2, whose hold_measured stop held the servo view).

    config        EvalConfig (None: from the placeholder profile)
    world         None, a W0 flat_world.FlatWorld (posts) or a raster corridor world (raster_corridor)
    s_end         row end s_E (default: world.s_end of a raster world; W0: None, no row-end rules)
    spacing_m     constant row spacing for the departure rule when the world has no catalogue profile
    s_start       start s for TIME_CAP (default: the first sample)
    final_reason  terminal pause / end reason of the executor or controller (stop category); None: not judged
    terminal_t_s  time of the terminal state (executor PAUSED / ENDED); None: STOP commands / terminal rest
    class_limits  optional dict lateral_rmse_max_m, lateral_abs_p95_max_m, lateral_abs_max_m, heading_rmse_max_deg,
                  heading_abs_max_deg (PREREG classes); None: no class judgement
    corridor_metrics  False for traces without a corridor (open-loop maneuvers on W0): lateral, heading and the
                  safety bands are reported as not applicable and give no fail code
    episode       optional identity dict copied into the result
    require_grid  True (default): refuse a trace whose rows do not give the evaluation grid (sample_dt_s, the PREREG
                  20 Hz) - a trace_every that is not a divisor of 5 (plant 100 Hz) thins the grid to 10, 6.7, 5, 2.9 Hz
                  ... (review R11). False evaluates on the coarser grid; the result records the spacing either way
    """
    config = EvalConfig.from_profile() if config is None else config
    for name in REQUIRED_COLUMNS:
        _column(arrays, name, required=True)
    kind = world_kind(world)
    g = config.geometry
    t_all = np.asarray(arrays['t_s'], dtype=float)
    if len(t_all) < 2:
        raise EvaluationError('trace_too_short')
    rows = grid_indices(t_all, config.sample_dt_s)
    spacing_s, spacing_even = grid_spacing(t_all, rows)
    grid_matches = (config.sample_dt_s is None or spacing_s is None
                    or (spacing_even and abs(spacing_s - float(config.sample_dt_s)) <= 1e-6))
    if require_grid and not grid_matches:
        raise EvaluationError('trace_rows_do_not_give_the_evaluation_grid: row spacing %.6g s (even %s), the PREREG '
                              'grid is %.6g s; record every plant step or every 5th (trace_every 1 or 5)'
                              % (spacing_s, spacing_even, config.sample_dt_s))
    pick = (lambda name, required=False: None if _column(arrays, name, required) is None
            else np.asarray(arrays[name])[rows])
    t = t_all[rows]
    x, y, yaw = pick('truth_x_m', True).astype(float), pick('truth_y_m', True).astype(float), \
        pick('truth_yaw_rad', True).astype(float)
    yaw_u = pick('truth_yaw_unwrapped_rad')
    yaw_u = np.unwrap(yaw) if yaw_u is None else yaw_u.astype(float)
    v = pick('truth_v_m_s', True).astype(float)
    dl = pick('truth_delta_left_rad', True).astype(float)
    dr = pick('truth_delta_right_rad', True).astype(float)
    deq = pick('truth_delta_eq_rad')
    deq = (.5 * (dl + dr)) if deq is None else deq.astype(float)
    frame = FieldFrame(getattr(world, 'frame', None) if kind == 'raster' else None)
    meta = json.loads(bytes(arrays['meta_json']).decode('utf-8')) if _column(arrays, 'meta_json') is not None else {}
    trace_world = meta.get('world')
    if trace_world is None:            # T1 rig / executor traces: meta.rig.world = {'kind': ...}
        trace_world = ((meta.get('rig') or {}).get('world') or {}).get('kind')
    curvilinear_needed = kind != 'raster' and trace_world == 'W2'
    if curvilinear_needed:
        raise EvaluationError('w2_trace_needs_its_raster_world (pass world=world_for_scene(scene))')
    s, d = frame.project(x, y)
    psi = frame.heading_error(x, y, yaw)
    if kind == 'raster':
        s_end = float(world.s_end) if s_end is None else float(s_end)
    spacing_fn = corridor_spacing_function(world if kind == 'raster' else None, spacing_m)
    n = len(t)
    # rest, motion, windows (PREREG)
    w = np.gradient(yaw_u, t) if n > 1 else np.zeros(n)
    at_rest = (np.abs(v) < config.rest_v_m_s) & (np.abs(w) < config.rest_w_rad_s)
    moving = np.abs(v) > config.rest_v_m_s
    first_motion = _first(moving)
    s0 = float(s[0]) if s_start is None else float(s_start)
    rest = None
    runs = rc._runs(at_rest)
    if runs and first_motion is not None and runs[-1][1] == n and runs[-1][0] > first_motion:
        a, b = runs[-1]
        if t[b - 1] - t[a] >= config.rest_hold_s - 1e-6:
            rest = int(a)
    # rear axle, front bumper, footprint (field)
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    rear_s, rear_d = frame.project(x + g.rear_axle_x_in_base_m * cos_y, y + g.rear_axle_x_in_base_m * sin_y)
    front_s, front_d = frame.project(x + g.front_bumper_x_in_base_m * cos_y, y + g.front_bumper_x_in_base_m * sin_y)
    cache = FootprintCache(g, include_hardware_margin)
    hull_world, owners = [], []
    for i in range(n):
        points = to_world(cache.hull(dl[i], dr[i]), x[i], y[i], yaw[i])
        hull_world.append(points)
        owners.append(np.full(len(points), i))
    hull_world = np.concatenate(hull_world)
    owners = np.concatenate(owners)
    _, hd = frame.project(hull_world[:, 0], hull_world[:, 1])
    d_max = np.full(n, -np.inf)
    d_min = np.full(n, np.inf)
    np.maximum.at(d_max, owners, hd)
    np.minimum.at(d_min, owners, hd)
    sweeps = np.array([cache.sweeps(dl[i], dr[i]) for i in range(n)])
    # contacts
    clear = {key: {name: np.full(n, math.inf) for name in rc.CLEARANCE_GROUPS} for key in ('swept', 'body')}
    obstacles = kind == 'raster' or (kind == 'posts' and len(np.asarray(world.posts).reshape(-1, 3)) > 0)
    if obstacles:
        for i in range(n):
            polys = {name: to_world(poly, x[i], y[i], yaw[i]) for name, poly in cache.polygons(dl[i], dr[i]).items()}
            values = polygon_clearances(world, polys, x[i], y[i], config.contact_reach_m)
            for key in ('swept', 'body'):
                for name, value in values[key].items():
                    clear[key][name][i] = value
    # departure and bands
    spacing = spacing_fn(s) if spacing_fn is not None else None
    if spacing is not None:
        margin_left = spacing / 2. - d_max
        margin_right = spacing / 2. + d_min
        departed = (margin_left < 0.) | (margin_right < 0.)
        departed_body = np.abs(d) > spacing / 2. - config.body_half_width_m
    else:
        margin_left = margin_right = departed = departed_body = None
    # termination (PREREG, post hoc)
    rules = ['COLLISION'] if kind != 'none' else []
    if s_end is not None:
        rules += ['MISSED_ROW_END', 'LEFT_FIELD', 'TIME_CAP']
    if spacing is not None:
        rules.append('CORRIDOR_DEPARTURE')
    rules.append('NO_PROGRESS')
    events = []
    if prereg_termination:
        limit = n if rest is None else rest + 1
        cap = rc.time_cap_s(s_end, s0) if s_end is not None else None
        for i in range(limit):
            if 'COLLISION' in rules and clear['swept']['hard'][i] <= 0:
                events.append(('COLLISION', i))
            elif s_end is not None and front_s[i] > s_end + rc.MISSED_MARGIN_M:
                events.append(('MISSED_ROW_END', i))
            elif departed is not None and departed[i]:
                events.append(('CORRIDOR_DEPARTURE', i))
            elif s_end is not None and s[i] < -rc.LEFT_FIELD_BEFORE_M:
                events.append(('LEFT_FIELD', i))
            elif s_end is not None and s[i] > s_end + rc.LEFT_FIELD_BEYOND_M:
                events.append(('MISSED_ROW_END', i))
            elif cap is not None and t[i] > cap:
                events.append(('TIME_CAP', i))
            elif t[i] - t[0] >= rc.NO_PROGRESS_WINDOW_S:
                j = int(np.searchsorted(t, t[i] - rc.NO_PROGRESS_WINDOW_S))
                if s[i] - s[j] < rc.NO_PROGRESS_MIN_M:
                    events.append(('NO_PROGRESS', i))
            if events:
                break
    if events:
        prereg_end, end_index = events[0]
    elif rest is not None:
        prereg_end, end_index = 'TERMINAL_STOP', n - 1
    else:
        prereg_end, end_index = 'OPEN', n - 1
    index = np.arange(n)
    upto = index <= end_index
    stop_index = rest if prereg_end == 'TERMINAL_STOP' else None
    window = upto.copy()
    if first_motion is not None:
        window &= index >= first_motion
    if stop_index is not None:
        window &= index <= stop_index
    in_row = window & (s >= 0.) & ((s <= s_end) if s_end is not None else True)
    span = window if first_motion is not None else upto
    # stop category and stop position
    category = raw_label = table_source = None
    if final_reason is not None:
        category, raw_label, table_source = steer_stop_category(final_reason)
    if prereg_end != 'TERMINAL_STOP':
        category = None
    e_stop = float(front_s[stop_index] - s_end) if stop_index is not None and s_end is not None else None
    outcome = None
    if prereg_end == 'MISSED_ROW_END':
        outcome = 'MISSED'
    elif category == 'ROW_END' and e_stop is not None:
        outcome = next((name for name, lo, hi in rc.ROW_END_WINDOWS
                        if (lo <= e_stop < hi if name != 'OK' else lo <= e_stop <= hi)), None)
        outcome = outcome or ('MISSED' if e_stop > rc.MISSED_MARGIN_M else None)
    # contacts summary
    moving_any = (np.abs(v) > config.rest_v_m_s) | (np.abs(w) > config.rest_w_rad_s)
    hard_contact = clear['swept']['hard'] <= 0
    any_contact = hard_contact | (clear['swept']['branch'] <= 0)
    body_hard = clear['body']['hard'] <= 0
    unknown = {name: int(np.sum((clear['swept'][name] <= 0) & upto)) for name in rc.UNKNOWN_GROUPS}

    def finite_min(values, mask):
        sel = values[mask]
        return _round(sel.min(), 4) if sel.size and np.isfinite(sel.min()) else None
    contacts = dict(
        evaluated=kind != 'none', world=kind,
        collision_samples=int(np.sum(hard_contact & upto)),
        moving_contact_samples=int(np.sum(any_contact & moving_any & upto)),
        rest_contact_samples=int(np.sum(any_contact & ~moving_any & upto)),
        branch_contact_samples=int(np.sum((clear['swept']['branch'] <= 0) & upto)),
        unknown_overlap_samples=unknown, unknown_overlap_total=int(sum(unknown.values())),
        min_hard_clearance_m=finite_min(clear['swept']['hard'], upto),
        min_branch_clearance_m=finite_min(clear['swept']['branch'], upto),
        body_only=dict(collision_samples=int(np.sum(body_hard & upto)),
                       min_hard_clearance_m=finite_min(clear['body']['hard'], upto)),
        wheel_sweep_only_collision_samples=int(np.sum(hard_contact & ~body_hard & upto)),
        first_collision_t_s=None if _first(hard_contact & upto) is None
        else round(float(t[_first(hard_contact & upto)]), 4))
    # lateral / heading / bands
    band = config.band_m
    reach_left = d_max - d
    reach_right = d - d_min
    swept_excess = np.maximum(d_max - config.band_edge_m, -config.band_edge_m - d_min)
    lateral = dict(base=rc._stats(d[in_row]), rear_axle=rc._stats(rear_d[in_row]),
                   front_bumper=rc._stats(front_d[in_row]),
                   base_distance_weighted=_distance_weighted(d, s, in_row), samples=int(in_row.sum()))
    heading = _stats_deg(psi[in_row])
    safety_band = dict(band_m=round(band, 4), max_abs_d_m=_round(np.abs(d[span]).max()) if span.any() else None,
                       exceeded=bool(span.any() and np.abs(d[span]).max() > band))
    safety_band_swept = dict(band_edge_m=round(config.band_edge_m, 4),
                             max_excess_m=_round(swept_excess[span].max()) if span.any() else None,
                             max_reach_left_m=_round(reach_left[span].max()) if span.any() else None,
                             max_reach_right_m=_round(reach_right[span].max()) if span.any() else None,
                             exceeded=bool(span.any() and swept_excess[span].max() > 0.))
    if spacing is not None:
        margins = np.minimum(margin_left, margin_right)
        first_dep = _first(departed & upto)
        departure = dict(evaluated=True, min_margin_m=_round(margins[upto].min()),
                         min_margin_side='left' if margin_left[upto].min() <= margin_right[upto].min() else 'right',
                         departed=first_dep is not None,
                         first_t_s=None if first_dep is None else round(float(t[first_dep]), 4),
                         prereg_body_form_departed=bool(np.any(departed_body & upto)),
                         prereg_body_form_min_margin_m=_round(
                             (spacing / 2. - config.body_half_width_m - np.abs(d))[upto].min()))
    else:
        departure = dict(evaluated=False, reason='no row spacing (W0 without spacing_m, or no world profile)')
    # steering: saturation, activity, tracking
    req = pick('cmd_steer_target_mrad')
    req = None if req is None else req.astype(float) / 1000.
    target = pick('steer_target_rad')
    target = None if target is None else target.astype(float)
    measured = pick('steer_measured_rad')
    measured = None if measured is None else measured.astype(float)
    drive = pick('drive_request_pct')
    drive = pick('cmd_throttle_pct') if drive is None else drive
    drive = None if drive is None else drive.astype(float)
    command = req if req is not None else target
    left_limit, right_limit = config.soft_limits_eq_rad
    steering = {}
    if command is not None:
        defined = np.isfinite(command)
        motion = span & ((drive > 0.) if drive is not None else moving)      # driving samples (forward command)
        driving = motion & defined                                          # ... with a steering command
        limit = np.where(command >= 0., left_limit, right_limit)
        saturated = driving & (np.abs(np.where(defined, command, 0.)) >= config.saturation_fraction * limit)
        dt = np.diff(t)
        longest = max((float(t[b - 1] - t[a]) + (float(np.median(dt)) if len(dt) else 0.)
                       for a, b in rc._runs(saturated)), default=0.)
        rate_true = np.abs(np.diff(deq)) / np.where(dt > 0, dt, np.nan)
        rate_limit = np.where(np.abs(v[:-1]) < config.rest_v_m_s, config.rate_standstill_rad_s,
                              config.rate_moving_rad_s)
        pair_driving = motion[:-1] & motion[1:]
        rate_limited = pair_driving & (rate_true >= config.saturation_fraction * rate_limit)
        cmd_rate = None
        over = np.zeros(len(dt), dtype=bool)
        if target is not None:
            cmd_rate = np.abs(np.diff(target)) / np.where(dt > 0, dt, np.nan)
            over = pair_driving & (cmd_rate > config.rate_moving_rad_s * (1. + 1e-9))
        fraction = float(saturated.sum() / max(int(driving.sum()), 1))
        steering['saturation'] = dict(
            driving_samples=int(motion.sum()), commanded_driving_samples=int(driving.sum()),
            saturated_fraction=round(fraction, 4),
            longest_saturated_run_s=round(longest, 3),
            rate_limited_fraction=round(float(rate_limited.sum() / max(int(pair_driving.sum()), 1)), 4),
            command_rate_over_limit_fraction=round(float(over.sum() / max(int(pair_driving.sum()), 1)), 4),
            authority_margin_low=bool(prereg_end == 'TERMINAL_STOP' and (fraction > AUTHORITY_FRACTION_FLAG
                                                                         or longest > AUTHORITY_RUN_FLAG_S)))
        dist = np.asarray(arrays['truth_distance_m'])[rows].astype(float) if 'truth_distance_m' in arrays else None
        travelled = float(dist[span].max() - dist[span].min()) if dist is not None and span.any() else None
        span_pairs = span[:-1] & span[1:]
        both = span_pairs & np.isfinite(command[:-1]) & np.isfinite(command[1:])
        tv_cmd = float(np.sum(np.abs(np.diff(np.where(defined, command, 0.)))[both]))
        tv_true = float(np.sum(np.abs(np.diff(deq))[span_pairs]))
        cmd_rate_all = np.abs(np.diff(np.where(defined, command, 0.))) / np.where(dt > 0, dt, np.nan)
        changes = _sign_changes(command[span], config.sign_threshold_rad)
        per_m = (lambda total: round(math.degrees(total) / travelled, 4) if travelled else None)
        steering['activity'] = dict(
            travelled_m=_round(travelled), total_variation_cmd_deg_per_m=per_m(tv_cmd),
            total_variation_true_deg_per_m=per_m(tv_true), sign_changes=changes,
            sign_changes_per_10m=round(changes / (travelled / 10.), 4) if travelled else None,
            rate_cmd_p95_deg_s=_round(math.degrees(float(np.percentile(cmd_rate_all[both], 95)))) if both.any()
            else None,
            rate_true_p95_deg_s=_round(math.degrees(float(np.nanpercentile(rate_true[span_pairs], 95))))
            if span_pairs.any() else None)
        tracking = {}
        if target is not None and measured is not None and span.any():
            tracking['target_minus_measured_p95_deg'] = _round(
                math.degrees(float(np.percentile(np.abs(target - measured)[span], 95))))
        if measured is not None and span.any():
            tracking['measured_minus_true_p95_deg'] = _round(
                math.degrees(float(np.percentile(np.abs(measured - deq)[span], 95))))
        steering['tracking'] = tracking
    else:
        steering['note'] = 'no steering command columns'
    # A21: wheel limits (every row) and sweep
    lim = g.wheel_limits()
    dl_all = np.asarray(arrays['truth_delta_left_rad'], dtype=float)
    dr_all = np.asarray(arrays['truth_delta_right_rad'], dtype=float)
    excess_l = np.maximum(dl_all - lim['left_wheel_toward_left'], -dl_all - lim['left_wheel_toward_right'])
    excess_r = np.maximum(dr_all - lim['right_wheel_toward_left'], -dr_all - lim['right_wheel_toward_right'])
    excess = np.maximum(excess_l, excess_r)
    steering['wheel_limits'] = dict(violation_samples=int(np.sum(excess > 1e-9)),
                                    max_excess_deg=round(math.degrees(max(0., float(excess.max()))), 6),
                                    max_abs_left_deg=round(math.degrees(float(np.abs(dl_all).max())), 4),
                                    max_abs_right_deg=round(math.degrees(float(np.abs(dr_all).max())), 4))
    steering['sweep'] = dict(max_left_wheel_beyond_body_m=_round(sweeps[:, 0].max()),
                             max_right_wheel_beyond_body_m=_round(sweeps[:, 1].max()),
                             include_hardware_margin=bool(include_hardware_margin))
    # forbidden events (every row)
    t_rows = t_all
    v_rows = np.asarray(arrays['truth_v_m_s'], dtype=float)
    drive_rows = _column(arrays, 'drive_request_pct')
    drive_rows = _column(arrays, 'cmd_throttle_pct') if drive_rows is None else drive_rows
    drive_rows = None if drive_rows is None else np.asarray(drive_rows, dtype=float)
    throttle_rows = _column(arrays, 'cmd_throttle_pct')
    stop_rows = _column(arrays, 'cmd_stop_mode')
    stop_rows = None if stop_rows is None else np.asarray(stop_rows).astype(np.int64)
    forbidden = {}
    if drive_rows is not None:
        negative = drive_rows < 0.
        if throttle_rows is not None:
            negative = negative | (np.asarray(throttle_rows, dtype=float) < 0.)
        k = _first(negative)
        forbidden['reverse'] = dict(flag=bool(negative.any()), samples=int(negative.sum()),
                                    first_t_s=None if k is None else round(float(t_rows[k]), 4))
        # AUTO_RESTART (drive based) on the evaluation grid like PREREG
        restart_t = None
        if first_motion is not None and drive is not None:
            idle = at_rest & (drive == 0.)
            for a, b in rc._runs(idle):
                if a < first_motion or t[b - 1] - t[a] < config.idle_before_restart_s - 1e-6:
                    continue
                later = np.flatnonzero(drive[b:] != 0.)
                if len(later):
                    restart_t = round(float(t[b + later[0]]), 4)
                    break
        forbidden['auto_restart'] = dict(flag=restart_t is not None, t_s=restart_t)
    else:
        forbidden['reverse'] = dict(flag=False, evaluated=False)
        forbidden['auto_restart'] = dict(flag=False, evaluated=False)
    target_rows = _column(arrays, 'steer_target_rad')
    if target_rows is not None:
        target_rows = np.asarray(target_rows, dtype=float)
        hold_rows, hold_source = hold_reference(arrays, meta, config, force=hold_reference_source)
        true_eq_rows = (np.asarray(arrays['truth_delta_eq_rad'], dtype=float) if 'truth_delta_eq_rad' in arrays
                        else .5 * (dl_all + dr_all))
        fm_rows = _first(np.abs(v_rows) > config.rest_v_m_s)
        rest_row = int(rows[rest]) if rest is not None else None
        starts = _terminal_starts(t_rows, stop_rows, fm_rows, rest_row, terminal_t_s)
        forbidden['steer_after_terminal'] = steer_after_terminal(
            t_rows, target_rows, drive_rows, stop_rows, v_rows, starts, config.hold_tolerance_rad, config.rest_v_m_s,
            hold_position=hold_rows, hold_source=hold_source, true_eq=true_eq_rows)
    else:
        forbidden['steer_after_terminal'] = dict(flag=False, evaluated=False, reason='no steer_target_rad column')
    # stop response (every row)
    responses = stop_response(
        t_rows, v_rows, np.asarray(arrays['truth_x_m'], dtype=float), np.asarray(arrays['truth_y_m'], dtype=float),
        np.asarray(arrays['truth_yaw_rad'], dtype=float), drive_rows,
        None if _column(arrays, 'drive_applied_pct') is None else np.asarray(arrays['drive_applied_pct'], float),
        stop_rows, np.asarray(arrays['truth_delta_eq_rad'], dtype=float) if 'truth_delta_eq_rad' in arrays
        else .5 * (dl_all + dr_all),
        None if 'truth_field_y_m' not in arrays else np.asarray(arrays['truth_field_y_m'], dtype=float),
        _first(np.abs(v_rows) > config.rest_v_m_s), config.rest_v_m_s)
    by_mode = {}
    for entry in responses:
        block = by_mode.setdefault(entry['mode'], dict(n=0, time_to_rest_s=[], distance_m=[], sideways_shift_m=[]))
        block['n'] += 1
        if entry['completed']:
            block['time_to_rest_s'].append(entry['time_to_rest_s'])
            block['distance_m'].append(entry['distance_m'])
            block['sideways_shift_m'].append(entry['sideways_shift_m'])
    for block in by_mode.values():
        for key in ('time_to_rest_s', 'distance_m', 'sideways_shift_m'):
            values = block.pop(key)
            block[key + '_median'] = _round(float(np.median(values))) if values else None
            block[key + '_max_abs'] = _round(float(np.max(np.abs(values)))) if values else None
    # obstacle / wall clearance at rest (PREREG metrics.obstacle_wall_clearance, END_STRUCTURE / OBSTACLE classes)
    # and the pre-stop speed ratio (PREREG report_only), on the evaluation grid
    rest_rows = None if stop_index is None else np.arange(stop_index, end_index + 1)
    rest_swept, rest_body = rest_clearances(world, cache, x, y, yaw, dl, dr, rest_rows)
    pre_stop = pre_stop_speed_ratio(v, drive, s, window, moving, stop_index, s0)
    # fail codes
    codes = []
    if events:
        codes.append(prereg_end)
    if contacts['moving_contact_samples'] and 'CONTACT_MOVING' not in codes:
        codes.append('CONTACT_MOVING')
    if not corridor_metrics:
        absent = dict(evaluated=False, reason='no corridor (open-loop maneuver on W0)')
        lateral, heading, safety_band, safety_band_swept = dict(absent), dict(absent), dict(absent), dict(absent)
    if safety_band.get('exceeded'):
        codes.append('SAFETY_BAND')
    if safety_band_swept.get('exceeded'):
        codes.append('SAFETY_BAND_SWEPT')
    if forbidden['reverse'].get('flag'):
        codes.append('REVERSE')
    if forbidden['auto_restart'].get('flag'):
        codes.append('AUTO_RESTART')
    if forbidden['steer_after_terminal'].get('flag'):
        codes.append('STEER_AFTER_TERMINAL')
    if steering['wheel_limits']['violation_samples']:
        codes.append('WHEEL_LIMIT')
    if outcome in ('FALSE', 'EARLY', 'LATE'):
        codes.append('ROW_END_' + outcome)
    class_result = None
    if class_limits is not None and corridor_metrics:
        class_result = _judge_class(lateral['base'], heading, class_limits)
        codes.extend(class_result['fail_codes'])
    result = dict(
        schema=SCHEMA, definitions='docs/steering/EVALUATE_STEER_KO.md (design 7.1 on PREREG metrics)',
        episode=plain(episode) if episode else None, config=config.describe(), world=kind,
        samples=int(n), grid_dt_s=None if spacing_s is None else round(spacing_s, 6),
        grid_dt_requested_s=config.sample_dt_s, grid_matches=bool(grid_matches), trace_rows=int(len(t_all)),
        prereg_termination=bool(prereg_termination), rules_evaluated=rules if prereg_termination else [],
        corridor_metrics=bool(corridor_metrics),
        prereg_end=prereg_end, end_t_s=round(float(t[end_index]), 4), s_E_m=_round(s_end, 3), s_start_m=round(s0, 3),
        final_reason=final_reason, stop_category=category, stop_table_label=raw_label,
        stop_table_source=table_source, lateral_m=lateral, heading_deg=heading, safety_band=safety_band,
        safety_band_swept=safety_band_swept, departure=departure,
        progress=dict(s_rest_m=_round(s[stop_index], 3) if stop_index is not None else None,
                      s_final_m=round(float(s[end_index]), 3),
                      progress_m=round(float((s[stop_index] if stop_index is not None else s[end_index]) - s0), 3),
                      fraction_of_row=round(float((s[stop_index] if stop_index is not None else s[end_index])
                                                  / s_end), 4) if s_end else None),
        stop_position_e_m=_round(e_stop, 3), overshoot_m=_round(max(0., float(front_s[upto].max() - s_end)), 3)
        if s_end is not None else None, row_end_outcome=outcome,
        false_row_end_detection=bool(category == 'ROW_END' and e_stop is not None and e_stop < -2.),
        missed_row_end=prereg_end == 'MISSED_ROW_END', contacts=contacts, steering=steering, forbidden=forbidden,
        obstacle_wall_clearance_at_rest_m=rest_swept, obstacle_wall_clearance_at_rest_body_m=rest_body,
        obstacle_wall_reach_m=rc.AT_REST_REACH_M,
        stop_response=dict(triggers=responses, by_mode=by_mode), class_judgement=class_result, fail_codes=codes,
        report_only=dict(mean_speed_m_s=_round(float(np.mean(v[span])), 3) if span.any() else None,
                         pre_stop_speed_ratio=pre_stop,
                         first_motion_t_s=None if first_motion is None else round(float(t[first_motion]), 4),
                         max_front_s_minus_end_m=_round(float(front_s.max() - s_end), 3) if s_end is not None
                         else None),
        result_kind=config.result_kind, field_validated=False, physical_truth=False)
    return plain(result)


def rest_clearances(world, cache, x, y, yaw, dl, dr, rest_rows, reach_m=None):
    """PREREG metrics.obstacle_wall_clearance at rest (raster_corridor._added_hard_at_rest) with the steering
    footprint: the smallest clearance of the swept footprint (and of the body rectangle alone) to the ADDED_HARD cells
    (scenario obstacle box / wall) over the terminal rest rows, with the AT_REST_REACH_M reach (6 m; the 2 m contact
    reach reported a wall 3 m ahead of a row-end stop as 'none'). (swept, body) in m; None without a raster world,
    without a terminal rest or with nothing added within the reach. Rest poses repeat: each distinct one once."""
    reach = rc.AT_REST_REACH_M if reach_m is None else float(reach_m)
    if world_kind(world) != 'raster' or rest_rows is None or not len(rest_rows):
        return None, None
    seen = {}
    swept, body = math.inf, math.inf
    for i in rest_rows:
        key = tuple(round(float(value), 6) for value in (x[i], y[i], yaw[i], dl[i], dr[i]))
        if key in seen:
            continue
        polys = {name: to_world(poly, x[i], y[i], yaw[i]) for name, poly in cache.polygons(dl[i], dr[i]).items()}
        values = polygon_clearances(world, polys, x[i], y[i], reach)
        seen[key] = True
        swept = min(swept, values['swept']['added_hard'])
        body = min(body, values['body']['added_hard'])
    return (_round(swept, 3) if math.isfinite(swept) else None), (_round(body, 3) if math.isfinite(body) else None)


def pre_stop_speed_ratio(v, drive, s, window, moving, stop_index, s0):
    """PREREG report_only pre_stop_speed_ratio (raster_corridor.evaluate_episode): |v| when the drive command fell to
    0 before the terminal rest (the first row of the last drive-zero run before the rest) over the median |v| of the
    cruise (window, moving, s in [s_start + 5, s_rest - 5]). None without a rest, a drive column or a cruise."""
    if stop_index is None or drive is None:
        return None
    zero = drive == 0.
    k = stop_index
    while k > 0 and zero[k - 1]:
        k -= 1
    cruise = window & moving & (s >= s0 + 5.) & (s <= s[stop_index] - 5.)
    if not cruise.any():
        return None
    return round(float(abs(v[k]) / max(float(np.median(np.abs(v[cruise]))), 1e-9)), 3)


def _judge_class(lateral, heading, limits):
    checks = (('LATERAL_RMSE', lateral.get('rmse'), limits.get('lateral_rmse_max_m')),
              ('LATERAL_P95', lateral.get('p95'), limits.get('lateral_abs_p95_max_m')),
              ('LATERAL_MAX', lateral.get('max'), limits.get('lateral_abs_max_m')),
              ('HEADING_RMSE', heading.get('rmse'), limits.get('heading_rmse_max_deg')),
              ('HEADING_MAX', heading.get('max'), limits.get('heading_abs_max_deg')))
    codes = [code for code, value, limit in checks if limit is not None and value is not None and value > limit]
    return dict(limits=dict(limits), fail_codes=codes)


# --- run folders ----------------------------------------------------------------------------------------------------
def evaluate_run(run_dir, out_path=None, class_limits=None, only=None):
    """Evaluate every ok episode of a fast_sim run folder (only: scene names to keep). Writes the report to out_path
    (default <run_dir>/STEER_EVALUATION.json) and returns it."""
    from . import fast_sim as fs
    run_dir = Path(run_dir)
    out_path = run_dir / 'STEER_EVALUATION.json' if out_path is None else Path(out_path)
    specs = fs.run_scene_specs(run_dir)
    worlds, rows = {}, []
    for record in fs.read_episodes(run_dir):
        if record.get('status') != 'ok' or record['scene_name'] not in specs:
            continue
        if only is not None and record['scene_name'] not in only:
            continue
        scene = specs[record['scene_name']]
        arrays, _ = fs.load_trace(run_dir / record['trace']['file'])
        key = json.dumps(scene.world.to_dict(), sort_keys=True)
        if key not in worlds:
            worlds[key] = world_for_scene(scene)
        config = EvalConfig.from_profile(fs.scene_params(scene).profile)
        harness = record.get('harness') or {}
        # The PREREG termination rules and the corridor metrics are row-episode rules: open-loop maneuvers on W0
        # (circles, sweeps, stops) have no corridor.
        prereg = scene.world.kind == 'W2' or scene.controller is not None
        ev = evaluate_trace(arrays, config, world=worlds[key], s_start=scene.start.x_m if scene.world.kind == 'W2'
                            else None, final_reason=harness.get('final_reason'), class_limits=class_limits,
                            prereg_termination=prereg, corridor_metrics=prereg,
                            episode=dict(episode_id=record['episode_id'], scene_name=record['scene_name'],
                                         seed=record['seed'], trace_content_sha256=record['trace']['content_sha256'],
                                         end_reason=record['end']['end_reason']))
        rows.append(ev)
    report = dict(schema=RUN_SCHEMA, run=str(run_dir), episodes=len(rows), summary=summarise(rows), rows=rows,
                  result_kinds=sorted({r['result_kind'] for r in rows}) or [RESULT_KIND_FALLBACK],
                  field_validated=False)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    _write_json(out_path, report)
    return report


EXECUTOR_RUN_SCHEMA = 'ubts.steer_sim.steer_evaluation_executor_run.v1'


def load_npz(path):
    """Arrays of an npz trace (no pickle)."""
    with np.load(str(path), allow_pickle=False) as data:
        return {name: data[name] for name in data.files}


def evaluate_executor_trace(arrays, spec, summary, config=None, world=None, class_limits=None):
    """Evaluate one executor-in-the-loop trace (tools/row_executor/sim_in_loop_steer.py: the T1 rig trace with the ctrl_*
    executor columns and truth_hold_position_rad). spec: the episode spec (world, controller); summary: the result's
    'summary' (ended_at_s: the executor's terminal moment, end_reason). The PREREG termination and the corridor metrics
    apply to W2 worlds and to the straight-hold controller (a corridor-following driver); open-loop maneuvers on W0 are
    evaluated without them (as evaluate_run does for fast_sim maneuvers)."""
    world_spec = spec.get('world') or {'kind': 'W0'}
    world = world_for_spec(world_spec) if world is None else world
    row_episode = world_spec.get('kind') == 'W2' or (spec.get('controller') or {}).get('kind') == 'straight_hold'
    start = spec.get('start_pose') or (0., 0., 0.)
    terminal = summary.get('ended_at_s')
    return evaluate_trace(arrays, config, world=world, s_start=float(start[0]) if world_spec.get('kind') == 'W2' else None,
                          final_reason=summary.get('end_reason'), terminal_t_s=terminal, class_limits=class_limits,
                          prereg_termination=row_episode and world_spec.get('kind') == 'W2',
                          corridor_metrics=row_episode and world_spec.get('kind') == 'W2',
                          episode=dict(name=spec.get('name'), seed=spec.get('seed'),
                                       end_reason=summary.get('end_reason')))


def evaluate_executor_run(run_dir, out_path=None, class_limits=None):
    """Evaluate the episode folders of a sim_in_loop_steer run (every sub-folder with RESULT.json and TRACE.npz, or the
    folder itself). Writes <run_dir>/STEER_EVALUATION_EXECUTOR.json (or out_path) and returns the report."""
    run_dir = Path(run_dir)
    folders = [run_dir] if (run_dir / 'RESULT.json').is_file() else sorted(
        p for p in run_dir.iterdir() if (p / 'RESULT.json').is_file() and (p / 'TRACE.npz').is_file())
    rows = []
    for folder in folders:
        result = json.loads((folder / 'RESULT.json').read_text(encoding='utf-8'))
        arrays = load_npz(folder / 'TRACE.npz')
        ev = evaluate_executor_trace(arrays, result['spec'], result['summary'], class_limits=class_limits)
        ev['episode']['folder'] = folder.name
        ev['episode']['trace_content_sha256'] = result['digests']['trace_content_sha256']
        rows.append(ev)
    report = dict(schema=EXECUTOR_RUN_SCHEMA, run=str(run_dir), episodes=len(rows), summary=summarise(rows), rows=rows,
                  result_kinds=sorted({r['result_kind'] for r in rows}) or [RESULT_KIND_FALLBACK],
                  field_validated=False)
    out_path = run_dir / 'STEER_EVALUATION_EXECUTOR.json' if out_path is None else Path(out_path)
    _write_json(out_path, report)
    return report


def summarise(rows):
    """Counts of fail codes, ends and contacts over evaluation rows."""
    codes, ends = {}, {}
    for row in rows:
        ends[row['prereg_end']] = ends.get(row['prereg_end'], 0) + 1
        for code in row['fail_codes']:
            codes[code] = codes.get(code, 0) + 1
    return dict(episodes=len(rows), prereg_ends=dict(sorted(ends.items())), fail_codes=dict(sorted(codes.items())),
                collisions=sum(1 for r in rows if r['contacts']['collision_samples']),
                wheel_sweep_only_collisions=sum(1 for r in rows if r['contacts']['wheel_sweep_only_collision_samples']))


def _write_json(path, value):
    path = Path(path)
    raw = (json.dumps(plain(value), sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False) + '\n')
    tmp = path.with_name(path.name + '.tmp%d' % os.getpid())
    tmp.write_bytes(raw.encode('utf-8'))
    os.replace(tmp, path)


# --- A21 evaluator case -----------------------------------------------------------------------------------------------
A21_STEER_DEG = 30.      # beyond the equivalent soft limit: the plant clamps the target (A21 limit)


def a21_scene(side='left', name=None):
    """W0, L0 servo, standstill steering from 0 to the soft limit (target 30 deg, clamped by the plant): the
    'start with wheels turned next to a trunk' case of design 7.1."""
    from . import scene_spec as ss
    sign = 1. if side == 'left' else -1.
    return ss.scene(name or 'a21_standstill_steer_' + side, backend='L0', seeds=[0], duration_s=4.,
                    maneuver=dict(kind='step', params=dict(at_s=.5, throttle_pct=0., steer_deg=[0., sign * A21_STEER_DEG])))


def a21_post_at_wheel(arrays, geometry, side, inset_m=.004, radius_m=.01):
    """A post (x, y, r) that the steered front wheel of `side` really reaches at its largest angle: its centre lies
    inset_m inside the bare wheel outline (no bracket margin) at the outermost wheel corner, and it is outside the
    body rectangle. Raises when the wheel does not stick out of the body far enough."""
    deltas = np.asarray(arrays['truth_delta_left_rad' if side == 'left' else 'truth_delta_right_rad'], dtype=float)
    i = int(np.argmax(np.abs(deltas)))
    dl, dr = float(arrays['truth_delta_left_rad'][i]), float(arrays['truth_delta_right_rad'][i])
    corners = np.asarray(geometry.front_wheel_corners(side, dl if side == 'left' else dr, False), dtype=float)
    sign = 1. if side == 'left' else -1.
    k = int(np.argmax(sign * corners[:, 1]))          # outermost corner of the bare wheel
    cx, cy = corners[k]
    centre = np.array([cx, cy - sign * inset_m])     # straight inward from the corner: inside the wheel outline
    body_edge = sign * geometry.footprint_width_m / 2.
    if sign * (centre[1] - body_edge) - radius_m <= 0.:
        raise EvaluationError('a21_wheel_does_not_stick_out_beyond_the_post_radius')
    world = to_world(centre[None, :], float(arrays['truth_x_m'][i]), float(arrays['truth_y_m'][i]),
                     float(arrays['truth_yaw_rad'][i]))[0]
    return float(world[0]), float(world[1]), float(radius_m), i


def a21_evaluator_case(out_dir=None):
    """A21 evaluator part: for each side, run the standstill large-steering scene (fast_sim.run_episode, L0), place a
    post that the steered wheel really reaches, evaluate with the post: the body-only footprint must not touch it,
    the swept footprint must. Returns the report (and writes A21_EVALUATOR.json into out_dir when given)."""
    from . import fast_sim as fs
    from . import flat_world as fw
    cases = []
    for side in ('left', 'right'):
        scene = a21_scene(side)
        arrays, _, record = fs.run_episode(scene, 0)
        profile = fs.scene_params(scene).profile
        config = EvalConfig.from_profile(profile)
        px, py, pr, index = a21_post_at_wheel(arrays, config.geometry, side)
        world = fw.FlatWorld(fw.FlatWorldConfig(markers=((px, py, pr),)))
        ev = evaluate_trace(arrays, config, world=world, prereg_termination=False,
                            episode=dict(episode_id=record['episode_id'], scene_name=scene.name))
        bare = evaluate_trace(arrays, config, world=world, prereg_termination=False, include_hardware_margin=False)
        body_min = ev['contacts']['body_only']['min_hard_clearance_m']
        swept_min = ev['contacts']['min_hard_clearance_m']
        cases.append(dict(
            side=side, scene=scene.to_dict(), scene_sha256=scene.sha256(),
            trace_content_sha256=fs.content_sha256(arrays), post_world_xyr=[round(px, 6), round(py, 6), pr],
            max_wheel_angle_sample=index, max_abs_wheel_deg=dict(left=ev['steering']['wheel_limits']['max_abs_left_deg'],
                                                                  right=ev['steering']['wheel_limits']['max_abs_right_deg']),
            soft_limits_eq_deg=ev['config']['soft_limits_eq_deg'], wheel_limit_violations=ev['steering']['wheel_limits'],
            sweep=ev['steering']['sweep'], body_only_min_clearance_m=body_min, swept_min_clearance_m=swept_min,
            swept_without_bracket_margin_min_clearance_m=bare['contacts']['min_hard_clearance_m'],
            body_only_collision_samples=ev['contacts']['body_only']['collision_samples'],
            swept_collision_samples=ev['contacts']['collision_samples'],
            first_swept_contact_t_s=ev['contacts']['first_collision_t_s'],
            passed=bool(body_min is not None and body_min > 0. and swept_min is not None and swept_min <= 0.
                        and bare['contacts']['min_hard_clearance_m'] <= 0.
                        and ev['steering']['wheel_limits']['violation_samples'] == 0)))
    report = dict(schema=A21_SCHEMA, check='A21(F evaluator)',
                  criterion='the footprint check reflects the steered-wheel sweep per steering angle: a post the '
                            'steered wheel really reaches is touched by the swept footprint and not by the body-only '
                            'footprint; no true wheel angle beyond its mechanical limit (SIM_ACCEPTANCE A21)',
                  cases=cases, passed=all(c['passed'] for c in cases), result_kind=RESULT_KIND_FALLBACK,
                  field_validated=False)
    if out_dir is not None:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        _write_json(Path(out_dir) / 'A21_EVALUATOR.json', report)
    return plain(report)


# --- CLI ------------------------------------------------------------------------------------------------------------
def _repository_root():
    return Path(__file__).resolve().parents[3]


def _outside_repository(path):
    """Outside this work tree and every other work tree of the same repository (scene_spec.inside_repository)."""
    from . import scene_spec as ss
    return not ss.inside_repository(path)


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python -B -m steering.evaluate_steer', description=__doc__.split('\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)
    run = sub.add_parser('run', help='evaluate every ok episode of a fast_sim run folder')
    run.add_argument('--run', required=True)
    run.add_argument('--out', default=None, help='report file (default <run>/STEER_EVALUATION.json; outside the '
                                                 'repository)')
    run.add_argument('--only', nargs='+', help='scene names to keep')
    case = sub.add_parser('a21', help='A21 evaluator case (standstill large steering next to a post)')
    case.add_argument('--out', required=True, help='folder outside the repository')
    exe = sub.add_parser('executor', help='evaluate a sim_in_loop_steer run folder (executor-in-the-loop traces)')
    exe.add_argument('--run', required=True)
    exe.add_argument('--out', default=None)
    args = parser.parse_args(argv)
    if args.command == 'executor':
        out = args.out or str(Path(args.run) / 'STEER_EVALUATION_EXECUTOR.json')
        if not (_outside_repository(args.run) and _outside_repository(out)):
            raise SystemExit('the run folder and the report must be outside the repository')
        report = evaluate_executor_run(args.run, out)
        print(json.dumps(report['summary'], indent=1, ensure_ascii=False))
        return 0
    if args.command == 'run':
        out = args.out or str(Path(args.run) / 'STEER_EVALUATION.json')
        if not (_outside_repository(args.run) and _outside_repository(out)):
            raise SystemExit('the run folder and the report must be outside the repository')
        report = evaluate_run(args.run, out, only=set(args.only) if args.only else None)
        print(json.dumps(report['summary'], indent=1, ensure_ascii=False))
        return 0
    if not _outside_repository(args.out):
        raise SystemExit('--out must be outside the repository')
    report = a21_evaluator_case(args.out)
    print(json.dumps(dict(passed=report['passed'], cases=[{k: c[k] for k in (
        'side', 'body_only_min_clearance_m', 'swept_min_clearance_m', 'swept_without_bracket_margin_min_clearance_m',
        'passed')} for c in report['cases']]), indent=1, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
