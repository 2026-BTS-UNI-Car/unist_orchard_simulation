"""A24 start-condition feasibility (design 5.3, 7.2 A24; 8.3 P2).

For every development corridor x start condition, the IDEAL controller is simulated on the kinematic bicycle
(rear-axle reference, design 3.2; no slip, efficiency 1, no understeer, no wander, no actuator lag, no zero offset)
and the condition is marked INFEASIBLE when its trajectory meets the departure judgement. Such a condition's
failure is not counted against a controller (design 5.3). The list is fixed before the steering pre-registration.

Ideal controller ("steer at the maximum steering rate straight to the maximum equivalent angle", A24), bang-bang:
  turn        from the start wheel angle delta0 the wheel-side equivalent angle moves at the moving steering-rate
              limit (steer_rate_max_deg_s) toward sigma x the equivalent soft limit of that side and holds it
  straighten  as soon as straightening now (delta -> 0 at the same rate) would end parallel to the corridor
              (predicted final heading error psi + sign(delta) v / (L r) (-ln cos delta) reaches 0), delta moves
              back to 0 at the rate limit; the window ends when delta is 0: the vehicle is parallel to the corridor
  sigma       both directions are simulated. At the start the predicted final heading of straightening at once is
              P0 = psi0 + sign(delta0) v / (L r) (-ln cos delta0); the direction sigma corrects when sigma P0 < 0
              (the heading must be turned that way to end parallel), or both do when |P0| <= 0.5 deg (nothing to
              correct); of the correcting directions the one with the larger departure margin is the ideal one
  heading     the controller's heading error is taken against the CHORD tangent of the centreline (direction from
              s - 1 m to s + 1 m at the base s): the catalogue polyline (0.25 m step) has kinks of up to about 1.5
              deg between segments, so "parallel" to one 0.25 m segment is not parallel to the corridor
The vehicle moves at the constant speed SPEED_M_S (0.4 m/s, design 6.3 target) from t = 0 (no steering at
standstill before the start; with a kinematic model only the ratio rate / speed matters: 30 deg/s at 0.4 m/s =
75 deg/m). Integration: exact circular arcs per 0.01 s step with the mean curvature of the step.

Departure judgement (design 7.1, the steering form of the PREREG CORRIDOR_DEPARTURE rule): the footprint (body
rectangle + steered front wheels at the wheel angles of delta (geometry.wheel_angles, the profile's Ackermann
ratio) + rear wheels, with the bracket margin) reaches a row line: max d of the footprint > spacing(s)/2 or min
d < -spacing(s)/2, spacing = left - right row d of the catalogue profile at the base s, every 0.01 s sample of the
window including the start pose. margin = min over the window of (spacing/2 - max d, spacing/2 + min d);
INFEASIBLE when margin < 0. Reported besides (not the verdict): the PREREG body form (|d_base| > spacing/2 - half
width), the swept-footprint hard-cell clearance along the ideal path (conservative full-cell enclosure, 20 Hz,
needs the 10/10 maps) and the verdict at the slow steering rate 10 deg/s (condition steer_rate_slow_10).

Start pose (single-row PREREG corridor_frame_and_labels.start_pose): base at s = 1.0 m, d = d0, heading = tangent +
psi0, wheels at delta0. Conditions (design 5.3 start conditions; nominal as the reference): see CONDITIONS.

Corridors: development set only (design 7.3): the frozen 8 directions (single-row CORRIDOR_FREEZE.json) and the
other included directions of the catalogue (external_paths 'corridor_catalogue'), WITHOUT the three sealed
directions (scene_spec.SEALED_DIRECTIONS, single-row amendment A1) and without the reverse directions of the sealed
row pairs (design 7.3: the same row pair reversed is not development evidence); the rule is scene_spec.sealed_status,
the same one that refuses such scenes. Their A24 entries are deferred to the steering pre-registration step and are
never simulated here.

Output (run folder outside the repository): A24_ENTRY_FEASIBILITY.json (list, parameters, identities, deferred
directions) and A24_TABLE.txt (markdown table of the margins). No raster coordinates are written: only corridor
frame values (s, d, heading error) and margins. Placeholder profile: placeholder-based SIM results (자리값 기반 SIM
결과), not field validation.

CLI (from sim/ubts): python -B -m steering.entry_feasibility --out <folder outside the repository> [--workers N]
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
import platform
import sys
import time
import types

import numpy as np

from closed_loop import raster_corridor as rc
from . import evaluate_steer as es
from . import geometry as geo
from . import scene_spec as ss
from .vehicle_profile import load_profile

SCHEMA = 'ubts.steer_sim.a24_entry_feasibility.v1'
S_START_M = 1.0                  # PREREG start pose s
SPEED_M_S = .4                   # design 6.3 target speed
DT_S = .01
HEADING_TOL_DEG = .5             # |P0| <= this: nothing to correct (both directions correct)
CHORD_HALF_M = 1.               # heading reference: chord of the centreline from s - 1 m to s + 1 m
MAX_LENGTH_M = 30.               # safety stop of the integration (never reached by a correcting direction)
CONTACT_EVERY = 5                # hard-cell clearance every 5th step (20 Hz at DT_S 0.01)
SLOW_RATE_DEG_S = 10.            # condition steer_rate_slow_10 (the profile range lower bound)
RESULT_KIND_FALLBACK = '자리값 기반 SIM 결과'
MAX_WORKERS = 12


@dataclass(frozen=True)
class Condition:
    name: str
    d0_m: float
    psi0_deg: float
    delta0_deg: float
    note: str


# Design 5.3 start conditions (+ nominal). Signs: d left of travel +, heading CCW +, steering left +.
CONDITIONS = (
    Condition('nominal', 0., 0., 0., 'centred start (reference)'),
    Condition('entry_lat_p04', .40, 0., 0., 'start 0.40 m left'),
    Condition('entry_lat_m04', -.40, 0., 0., 'start 0.40 m right'),
    Condition('entry_hdg_p08', 0., 8., 0., 'heading +8 deg'),
    Condition('entry_hdg_m08', 0., -8., 0., 'heading -8 deg'),
    Condition('entry_hdg_p12', 0., 12., 0., 'heading +12 deg'),
    Condition('entry_hdg_m12', 0., -12., 0., 'heading -12 deg'),
    Condition('entry_div_pos', .40, 8., 0., 'diverging: 0.40 m left and +8 deg'),
    Condition('entry_div_neg', -.40, -8., 0., 'diverging: 0.40 m right and -8 deg'),
    Condition('start_wheels_turned_p10', 0., 0., 10., 'wheels turned +10 deg at the start'),
    Condition('start_wheels_turned_m10', 0., 0., -10., 'wheels turned -10 deg at the start'),
)
CONDITION_NAMES = tuple(c.name for c in CONDITIONS)
# single-row-20261102/CORRIDOR_FREEZE.json 'selected' (SHA-256 of its SELECTION.json 313d4bf0...): the frozen 8.
FROZEN_DIRECTIONS = (('035952', 'c04', 'rev'), ('035952', 'c05', 'rev'), ('040927', 'c02', 'fwd'),
                     ('040927', 'c08', 'fwd'), ('042301', 'c02', 'fwd'), ('042301', 'c05', 'fwd'),
                     ('045553', 'c04', 'fwd'), ('040927', 'c06', 'rev'))
SEALED_DIRECTIONS = ss.SEALED_DIRECTIONS
OTHER = dict(fwd='rev', rev='fwd')


def corridor_key(record, corridor_id, direction):
    return '%s:%s:%s' % (record, corridor_id, direction)


def split_directions(catalogue):
    """(development, deferred) from the catalogue's included directions. development: list of dict(record,
    corridor_id, direction, role 'frozen' | 'included'); deferred: list of dict(..., reason). Sealed directions and
    the reverse of a sealed row pair are deferred (never simulated)."""
    development, deferred = [], []
    for entry in catalogue['corridors']:
        for direction in ('fwd', 'rev'):
            if entry['directions'][direction]['status'] != 'included':
                continue
            key = (entry['record'], entry['corridor_id'], direction)
            item = dict(record=key[0], corridor_id=key[1], direction=direction, key=corridor_key(*key))
            status = ss.sealed_status(*key)          # the scene_spec rule (one place for scenes and A24)
            if status is not None:
                deferred.append(dict(item, sealed_status=status,
                                     reason='%s: A24 list deferred to the steering pre-registration step'
                                     % ss.SEALED_STATUS_REASONS[status]))
            else:
                development.append(dict(item, role='frozen' if key in FROZEN_DIRECTIONS else 'included'))
    missing = [corridor_key(*k) for k in FROZEN_DIRECTIONS
               if not any((d['record'], d['corridor_id'], d['direction']) == k for d in development)]
    if missing:
        raise ValueError('a24_frozen_directions_missing_from_the_catalogue: %s' % missing)
    return development, deferred


@dataclass(frozen=True)
class IdealParams:
    speed_m_s: float
    rate_rad_s: float
    limit_left_rad: float
    limit_right_rad: float
    dt_s: float = DT_S
    heading_tol_rad: float = math.radians(HEADING_TOL_DEG)
    max_length_m: float = MAX_LENGTH_M

    def limit(self, sigma):
        return self.limit_left_rad if sigma > 0 else self.limit_right_rad


def ideal_params(profile=None, rate_deg_s=None, speed_m_s=SPEED_M_S):
    profile = load_profile() if profile is None else profile
    geometry = geo.SteeringGeometry.from_profile(profile)
    left, right = geometry.soft_limits_eq()
    rate = float(profile.value('steer_rate_max_deg_s')) if rate_deg_s is None else float(rate_deg_s)
    return geometry, IdealParams(speed_m_s=float(speed_m_s), rate_rad_s=math.radians(rate), limit_left_rad=left,
                                 limit_right_rad=right)


def chord_tangent(frame, s, half=CHORD_HALF_M):
    """Direction of the centreline chord from s - half to s + half (the end segments extend beyond the ends)."""
    x0, y0 = frame.point(s - half)
    x1, y1 = frame.point(s + half)
    return math.atan2(y1 - y0, x1 - x0)


def chord_heading_error(frame, x, y, yaw, half=CHORD_HALF_M):
    """wrap(yaw - chord tangent at the projection s of (x, y))."""
    s = frame.project(x, y)[0]
    error = yaw - chord_tangent(frame, s, half)
    return math.atan2(math.sin(error), math.cos(error))


def predicted_final_heading(psi, delta, gain):
    """Heading error after straightening delta -> 0 at once at the rate limit: psi + sign(delta) gain (-ln cos delta),
    gain = v / (L r)."""
    return psi + math.copysign(1., delta) * gain * (-math.log(math.cos(delta))) if delta else psi


def simulate_ideal(geometry, frame, start_pose, delta0, sigma, params):
    """Kinematic bicycle under the ideal bang-bang controller toward sigma (+1 left, -1 right).

    start_pose: base (x, y, yaw) in the corridor frame's plane; frame: CurvilinearFrame (to_field(x, y, yaw) ->
    (s, d, psi)). Returns dict of arrays (t, base x / y / yaw, delta, phase codes 0 turn 1 straighten) and the end
    state."""
    v, r, dt = params.speed_m_s, params.rate_rad_s, params.dt_s
    length = geometry.wheelbase_m
    x_rear = geometry.rear_axle_x_in_base_m
    xb, yb, yaw = (float(value) for value in start_pose)
    xr, yr = xb + x_rear * math.cos(yaw), yb + x_rear * math.sin(yaw)
    delta = float(delta0)
    gain = v / (length * r)
    limit = sigma * params.limit(sigma)
    phase, t, travelled = 0, 0., 0.
    initial_predicted = None
    out = dict(t=[0.], x=[xb], y=[yb], yaw=[yaw], delta=[delta], phase=[0])
    converged = True
    while True:
        psi = chord_heading_error(frame, xb, yb, yaw)
        if phase == 0:
            predicted = predicted_final_heading(psi, delta, gain)
            if t == 0.:
                initial_predicted = predicted
            if sigma * predicted >= 0.:
                phase = 1
        target = limit if phase == 0 else 0.
        if phase == 1 and delta == 0.:
            break
        step = max(-r * dt, min(r * dt, target - delta))
        new_delta = delta + step
        if phase == 1 and abs(new_delta) < 1e-12:
            new_delta = 0.
        kappa = .5 * (math.tan(delta) + math.tan(new_delta)) / length
        ds = v * dt
        if abs(kappa) < 1e-12:
            xr += ds * math.cos(yaw)
            yr += ds * math.sin(yaw)
        else:
            yaw2 = yaw + kappa * ds
            xr += (math.sin(yaw2) - math.sin(yaw)) / kappa
            yr -= (math.cos(yaw2) - math.cos(yaw)) / kappa
            yaw = yaw2
        delta = new_delta
        t += dt
        travelled += ds
        xb, yb = xr - x_rear * math.cos(yaw), yr - x_rear * math.sin(yaw)
        for key, value in (('t', t), ('x', xb), ('y', yb), ('yaw', yaw), ('delta', delta), ('phase', phase)):
            out[key].append(value)
        if travelled > params.max_length_m:
            converged = False
            break
    arrays = {key: np.asarray(values, dtype=float) for key, values in out.items()}
    s_end, d_end, psi_local = frame.to_field(xb, yb, yaw)
    return dict(arrays=arrays, sigma=int(sigma), converged=converged, length_m=travelled,
                initial_predicted_heading_deg=math.degrees(initial_predicted),
                end=dict(s_m=s_end, d_m=d_end, psi_deg=math.degrees(chord_heading_error(frame, xb, yb, yaw)),
                         psi_local_segment_deg=math.degrees(psi_local)),
                peak_abs_delta_deg=float(np.degrees(np.abs(arrays['delta']).max())),
                reached_limit=bool(np.any(np.abs(arrays['delta']) >= params.limit(sigma) - 1e-9)))


def departure_margins(geometry, frame, spacing_fn, arrays, cache=None):
    """Per sample: (margin_left, margin_right, base s, base d) of the swept footprint (design 7.1 departure)."""
    cache = cache or es.FootprintCache(geometry)
    xs, ys, yaws, deltas = arrays['x'], arrays['y'], arrays['yaw'], arrays['delta']
    points, owners = [], []
    for i in range(len(xs)):
        left, right = geometry.wheel_angles(float(deltas[i]))
        hull = es.to_world(cache.hull(left, right), float(xs[i]), float(ys[i]), float(yaws[i]))
        points.append(hull)
        owners.append(np.full(len(hull), i))
    points = np.concatenate(points)
    owners = np.concatenate(owners)
    _, d_points = frame.project_many(points[:, 0], points[:, 1])
    n = len(xs)
    d_max, d_min = np.full(n, -np.inf), np.full(n, np.inf)
    np.maximum.at(d_max, owners, d_points)
    np.minimum.at(d_min, owners, d_points)
    s_base, d_base = frame.project_many(xs, ys)
    half = spacing_fn(s_base) / 2.
    return half - d_max, half + d_min, s_base, d_base, half


def _candidate(geometry, frame, spacing_fn, pose, delta0, sigma, params, cache):
    sim = simulate_ideal(geometry, frame, pose, delta0, sigma, params)
    left, right, s_base, d_base, half = departure_margins(geometry, frame, spacing_fn, sim['arrays'], cache)
    margin = np.minimum(left, right)
    k = int(np.argmin(margin))
    body_margin = half - geometry.footprint_width_m / 2. - np.abs(d_base)
    sim.update(margin_left=left, margin_right=right, s_base=s_base, d_base=d_base,
               min_margin_m=float(margin[k]), min_margin_side='left' if left[k] <= right[k] else 'right',
               min_margin_s_m=float(s_base[k]), min_margin_t_s=float(sim['arrays']['t'][k]),
               prereg_body_min_margin_m=float(body_margin.min()))
    p0 = math.radians(sim['initial_predicted_heading_deg'])
    sim['corrects'] = bool(sim['converged'] and (sigma * p0 < 0. or abs(p0) <= params.heading_tol_rad))
    return sim


def choose(candidates):
    """The ideal direction: among the correcting candidates the largest margin (ties: sigma +1 first)."""
    correcting = [c for c in candidates if c['corrects']]
    pool = correcting or list(candidates)
    best = max(pool, key=lambda c: (round(c['min_margin_m'], 9), c['sigma']))
    return best, bool(correcting)


def _hard_contact(world, geometry, arrays, cache, every=CONTACT_EVERY):
    """Swept and body-only minimum hard-cell clearance along an ideal path (raster world), every `every` samples."""
    if world is None:
        return None
    n = len(arrays['t'])
    picks = sorted(set(range(0, n, every)) | {n - 1})
    swept, body, start = math.inf, math.inf, None
    for i in picks:
        left, right = geometry.wheel_angles(float(arrays['delta'][i]))
        polys = {name: es.to_world(poly, float(arrays['x'][i]), float(arrays['y'][i]), float(arrays['yaw'][i]))
                 for name, poly in cache.polygons(left, right).items()}
        values = es.polygon_clearances(world, polys, float(arrays['x'][i]), float(arrays['y'][i]))
        swept = min(swept, values['swept']['hard'])
        body = min(body, values['body']['hard'])
        if i == 0:
            start = values['swept']['hard']
    return dict(samples=len(picks), swept_min_hard_clearance_m=es._round(swept),
                body_min_hard_clearance_m=es._round(body), contact=bool(swept <= 0.),
                start_swept_hard_clearance_m=es._round(start), start_contact=bool(start is not None and start <= 0.))


def check_entry(geometry, frame, spacing_fn, condition, params, *, world=None, slow_params=None, cache=None):
    """A24 row of one corridor frame and one Condition. world: raster world for the hard-cell report (or None)."""
    cache = cache or es.FootprintCache(geometry)
    pose = frame.pose(S_START_M, condition.d0_m, math.radians(condition.psi0_deg))
    delta0 = math.radians(condition.delta0_deg)
    candidates = [_candidate(geometry, frame, spacing_fn, pose, delta0, sigma, params, cache) for sigma in (1, -1)]
    best, corrected = choose(candidates)
    other = [c for c in candidates if c is not best][0]
    row = dict(
        condition=condition.name, d0_m=condition.d0_m, psi0_deg=condition.psi0_deg, delta0_deg=condition.delta0_deg,
        verdict='INFEASIBLE' if best['min_margin_m'] < 0. else 'FEASIBLE', sigma=best['sigma'],
        initial_predicted_heading_deg=es._round(best['initial_predicted_heading_deg'], 3),
        a_direction_corrects=corrected, min_margin_m=es._round(best['min_margin_m']),
        min_margin_side=best['min_margin_side'], min_margin_s_m=es._round(best['min_margin_s_m'], 3),
        min_margin_t_s=es._round(best['min_margin_t_s'], 3),
        start_margin_m=es._round(min(best['margin_left'][0], best['margin_right'][0])),
        prereg_body_min_margin_m=es._round(best['prereg_body_min_margin_m']),
        window_length_m=es._round(best['length_m'], 3), peak_abs_delta_deg=es._round(best['peak_abs_delta_deg'], 3),
        reached_limit=best['reached_limit'], end_s_m=es._round(best['end']['s_m'], 3),
        end_d_m=es._round(best['end']['d_m'], 4), end_psi_deg=es._round(best['end']['psi_deg'], 3),
        end_psi_local_segment_deg=es._round(best['end']['psi_local_segment_deg'], 3),
        max_abs_d_base_m=es._round(float(np.abs(best['d_base']).max())),
        other_direction=dict(sigma=other['sigma'], corrects=other['corrects'],
                             min_margin_m=es._round(other['min_margin_m'])))
    row['hard_cells'] = _hard_contact(world, geometry, best['arrays'], cache)
    if slow_params is not None:
        slow = [_candidate(geometry, frame, spacing_fn, pose, delta0, sigma, slow_params, cache) for sigma in (1, -1)]
        slow_best, slow_corrected = choose(slow)
        row['slow_rate'] = dict(rate_deg_s=round(math.degrees(slow_params.rate_rad_s), 3),
                                verdict='INFEASIBLE' if slow_best['min_margin_m'] < 0. else 'FEASIBLE',
                                min_margin_m=es._round(slow_best['min_margin_m']), sigma=slow_best['sigma'],
                                a_direction_corrects=slow_corrected,
                                window_length_m=es._round(slow_best['length_m'], 3),
                                hard_cells=_hard_contact(world, geometry, slow_best['arrays'], cache))
    return row


_CATALOGUES = {}


def _catalogue(path):
    if path not in _CATALOGUES:
        _CATALOGUES[path] = rc.load_catalogue(path)
    return _CATALOGUES[path]


def _corridor_job(job):
    """One corridor (spawn safe): rows for all conditions."""
    catalogue = _catalogue(job['catalogue_path'])
    profile = load_profile()
    geometry, params = ideal_params(profile)
    _, slow_params = ideal_params(profile, rate_deg_s=job['slow_rate_deg_s'])
    config = rc.corridor_world_config(job['record'], job['corridor_id'], job['direction'], catalogue=catalogue)
    frame = rc.CurvilinearFrame(config.polyline, config.s_origin_m)
    spacing_fn = es.corridor_spacing_function(types.SimpleNamespace(config=config))
    world, world_note = None, None
    if job['hard_cells']:
        try:
            world = rc.build_raster_world(config)
        except (OSError, ValueError) as exc:
            world_note = '%s: %s' % (type(exc).__name__, exc)
    cache = es.FootprintCache(geometry)
    started = time.perf_counter()
    rows = [check_entry(geometry, frame, spacing_fn, condition, params, world=world, slow_params=slow_params,
                        cache=cache) for condition in CONDITIONS if condition.name in job['conditions']]
    spacing_start = float(spacing_fn(np.array([S_START_M]))[0])
    return dict(key=job['key'], record=job['record'], corridor_id=job['corridor_id'], direction=job['direction'],
                role=job['role'], end_s_m=round(float(config.end_s_m), 3), spacing_median_m=round(
                    float(config.row_width_m), 3), spacing_at_start_m=round(spacing_start, 3), rows=rows,
                hard_cells_world=None if world is None else dict(pgm_sha256=world.map.info['pgm_sha256']),
                hard_cells_note=world_note, wall_s=round(time.perf_counter() - started, 3))


def _file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _inside_repository(path):
    """Inside this work tree or any other work tree of the same repository (scene_spec.inside_repository)."""
    return ss.inside_repository(path)


def run_a24(out_dir, *, workers=1, conditions=CONDITION_NAMES, hard_cells=True, slow_rate_deg_s=SLOW_RATE_DEG_S,
            catalogue_path=None, only=None, progress=None):
    """Run A24 on the development directions into out_dir (outside the repository). Returns the report."""
    out_dir = Path(out_dir).resolve()
    if _inside_repository(out_dir):
        raise ValueError('a24_out_folder_inside_the_repository: %s' % out_dir)
    if not 1 <= int(workers) <= MAX_WORKERS:
        raise ValueError('a24_workers_out_of_range (1..%d)' % MAX_WORKERS)
    unknown = sorted(set(conditions) - set(CONDITION_NAMES))
    if unknown:
        raise ValueError('a24_unknown_conditions: %s' % unknown)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = rc.catalogue_path(catalogue_path)
    catalogue = rc.load_catalogue(path)
    development, deferred = split_directions(catalogue)
    if only:
        development = [d for d in development if d['key'] in set(only)]
        refused = sorted(set(only) - {d['key'] for d in development})
        if refused:
            raise ValueError('a24_not_development_directions: %s' % refused)
    jobs = [dict(d, catalogue_path=str(path), conditions=list(conditions), hard_cells=bool(hard_cells),
                 slow_rate_deg_s=float(slow_rate_deg_s)) for d in development]
    started = time.perf_counter()
    for variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
        os.environ.setdefault(variable, '1')
    if int(workers) == 1:
        corridors = []
        for job in jobs:
            corridors.append(_corridor_job(job))
            if progress is not None:
                progress('%s %.2f s' % (job['key'], corridors[-1]['wall_s']))
    else:
        context = multiprocessing.get_context('spawn')
        with ProcessPoolExecutor(max_workers=int(workers), mp_context=context) as pool:
            corridors = list(pool.map(_corridor_job, jobs))
    profile = load_profile()
    geometry, params = ideal_params(profile)
    infeasible = [dict(corridor=c['key'], condition=r['condition'], min_margin_m=r['min_margin_m'])
                  for c in corridors for r in c['rows'] if r['verdict'] == 'INFEASIBLE']
    slow_infeasible = [dict(corridor=c['key'], condition=r['condition'], min_margin_m=r['slow_rate']['min_margin_m'])
                       for c in corridors for r in c['rows'] if r.get('slow_rate', {}).get('verdict') == 'INFEASIBLE']
    contact = [dict(corridor=c['key'], condition=r['condition'],
                    swept_min_hard_clearance_m=r['hard_cells']['swept_min_hard_clearance_m'])
               for c in corridors for r in c['rows'] if r.get('hard_cells') and r['hard_cells']['contact']]
    code = {rel: _file_sha256(Path(__file__).resolve().parents[1] / rel)
            for rel in ('steering/entry_feasibility.py', 'steering/evaluate_steer.py', 'steering/geometry.py',
                        'steering/vehicle_profile.py', 'closed_loop/raster_corridor.py')}
    report = dict(
        schema=SCHEMA, check='A24(F)',
        criterion='ideal control (max steering rate, straight to the max equivalent angle) on the kinematic bicycle; '
                  'INFEASIBLE when the trajectory meets the departure judgement (SIM_ACCEPTANCE A24)',
        parameters=dict(speed_m_s=params.speed_m_s, rate_deg_s=round(math.degrees(params.rate_rad_s), 4),
                        slow_rate_deg_s=float(slow_rate_deg_s),
                        soft_limits_eq_deg=[round(math.degrees(params.limit_left_rad), 4),
                                            round(math.degrees(params.limit_right_rad), 4)],
                        dt_s=params.dt_s, heading_tol_deg=HEADING_TOL_DEG, chord_half_m=CHORD_HALF_M,
                        s_start_m=S_START_M,
                        footprint=dict(length_m=geometry.footprint_length_m, width_m=geometry.footprint_width_m,
                                       wheelbase_m=geometry.wheelbase_m, ackermann_ratio=geometry.ackermann_ratio,
                                       hardware_margin_m=geometry.hardware_margin_m),
                        departure='swept footprint max d > spacing(s)/2 or min d < -spacing(s)/2 (design 7.1)'),
        conditions=[dict(name=c.name, d0_m=c.d0_m, psi0_deg=c.psi0_deg, delta0_deg=c.delta0_deg, note=c.note)
                    for c in CONDITIONS if c.name in conditions],
        profile=dict(profile_id=profile.profile_id, profile_sha256=profile.sha256),
        catalogue=dict(sha256=catalogue['_sha256']), code_sha256=code,
        environment=dict(python=platform.python_version(), numpy=np.__version__, platform=sys.platform),
        development_directions=len(corridors), deferred_directions=deferred, corridors=corridors,
        infeasible=infeasible, infeasible_at_slow_rate=slow_infeasible, ideal_path_hard_contacts=contact,
        wall_s=round(time.perf_counter() - started, 2), workers=int(workers),
        result_kind=profile.result_label().get('result_kind', RESULT_KIND_FALLBACK), field_validated=False)
    es._write_json(out_dir / 'A24_ENTRY_FEASIBILITY.json', report)
    (out_dir / 'A24_TABLE.txt').write_bytes(table_text(report).encode('utf-8'))
    return report


def _cell(row):
    mark = '**INF** ' if row['verdict'] == 'INFEASIBLE' else ''
    return '%s%.2f' % (mark, row['min_margin_m'])


def table_text(report):
    """Markdown table: corridors x conditions, departure margin (m) of the ideal path; INF = INFEASIBLE."""
    names = [c['name'] for c in report['conditions']]
    short = {name: name.replace('entry_', '').replace('start_wheels_turned_', 'wheels_') for name in names}
    lines = ['| corridor | role | spacing at start (m) | ' + ' | '.join(short[n] for n in names) + ' |',
             '|' + '---|' * (3 + len(names))]
    for c in report['corridors']:
        cells = {r['condition']: r for r in c['rows']}
        lines.append('| %s | %s | %.2f | %s |' % (c['key'], c['role'], c['spacing_at_start_m'],
                                                   ' | '.join(_cell(cells[n]) for n in names)))
    return '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python -B -m steering.entry_feasibility', description=__doc__.split('\n')[0])
    parser.add_argument('--out', required=True, help='run folder outside the repository')
    parser.add_argument('--workers', type=int, default=1)
    parser.add_argument('--conditions', nargs='+', default=list(CONDITION_NAMES))
    parser.add_argument('--only', nargs='+', help='development corridor keys record:cid:dir')
    parser.add_argument('--no-hard-cells', action='store_true', help='skip the hard-cell report (no maps needed)')
    args = parser.parse_args(argv)
    report = run_a24(args.out, workers=args.workers, conditions=args.conditions, hard_cells=not args.no_hard_cells,
                     only=args.only, progress=lambda text: print(text, flush=True))
    print(json.dumps(dict(development_directions=report['development_directions'],
                          deferred=[d['key'] for d in report['deferred_directions']],
                          infeasible=report['infeasible'], infeasible_at_slow_rate=report['infeasible_at_slow_rate'],
                          ideal_path_hard_contacts=len(report['ideal_path_hard_contacts']), wall_s=report['wall_s']),
                     indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
