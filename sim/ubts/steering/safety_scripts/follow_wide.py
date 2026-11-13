"""Wide single-row following matrix of the row-following controller (bundle S1c: PC-11 condition injector, PC-14
pre-registration, PC-15 development iterations, PC-19 judgement run; PW-08 drive table checks).

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_V2_KO.md and PREREG_STEER_FOLLOW_V2.json (= prereg() of this
module; the test sim/ubts/test_steering_follow_wide.py compares them). Do not change CONDITIONS, CRITERIA, seeds or the
measuring rules after a run; an amendment is written in the pre-registration document first.

Simulator: the T1 rig with the executor in the loop (tools/row_executor/sim_in_loop_steer.py), the same chain as
follow_matrix.py: the real executor loop and Runner (front_steer, MOTOR, single-row mode, one-press start) with the
controller slot {'kind': 'row_follow'} <-> bridge client <-> host bridge <-> MCU firmware and actuator model <-> steered
plant <-> world <-> S2 scan, IMU and (GNSS conditions) the virtual GNSS.

Condition injector (PC-11), everything on the SIM side (plant, world, sensors, operator emulation); the executor only
sees its usual inputs:
  row world       W0 posts on two row lines from a RowGeometry: row width, tree spacing, a constant curvature (a curved
                  row: the lines are arcs), missing trees (gaps), end posts beyond the last tree, extra posts (obstacle)
  W2              a development corridor of the 10/10 catalogue (raster world; sealed corridors are refused by the rig)
  start           (s, d, heading) in the row frame
  vehicle         profile overrides (curvature wander mean = side pull, steering zero offset), profile v0.3 with the
                  steering-axis disturbance model (steer_torque injections, terrain impact / push processes)
  drive table     the plant's steady speed = the field tuning file's speed table (the file the executor reads) x factor
                  at duty x duty_scale - duty_offset_pct (steered_plant.drive_table_spec; RigConfig.drive_table)
  sensors         rig faults sensor_blackout (lidar / imu), the virtual GNSS with dropouts / jumps (gnss_mode on)
  operator        the one-press screen stops its heartbeats (action screen_hidden)
  speed           the controller spec's cruise_speed_m_s (the field tuning 'target speed')
Slope: the fast SIM has no slope model (the plant is planar, the IMU has no tilt input); not injected (model absent).

Metrics come from the truth columns of the rig trace and the world geometry (evaluation only, A13 role 'evaluator'):
lateral error d and heading error of the footprint centre in the row frame (straight: y and yaw; arc: distance to the
centre arc and yaw minus the arc tangent; W2: the corridor frame columns truth_field_*), over the drive interval (first
to last tick with a drive command; offset starts: RMSE / p95 after SETTLE_M of travel, maxima from the start), the row
end stop position e = s(front bumper) - s(last structure on the row lines: the last tree or the end post; decision R50),
the old e (against the last tree) for the record, the smallest footprint clearance to a post (W2: the raster's hard
clearance), the end reason, pauses after the drive started, the A22 rule after the terminal moment, steering fault lines
of the MCU and host bridge latches, the GNSS layer block, the operator-link stop time and stale stop times. Every number
is a placeholder-based SIM result (자리값 기반 SIM 결과), not field performance.

CLI (from sim/ubts):
  python -B -m steering.safety_scripts.follow_wide --out <folder outside the repository> --seeds dev|judge
         [--workers N (<= 4)] [--only NAME ...] [--iteration K]
  python -B -m steering.safety_scripts.follow_wide --out <folder> --pw08-cell     (PW-08 table cell check)
  python -B -m steering.safety_scripts.follow_wide --prereg                         (print the JSON)
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
import os
import sys
import time

import numpy as np

from . import acceptance as A
from . import follow_matrix as FM
from . import measure as M

SCHEMA = 'ubts.steer_sim.follow_wide.v1'
PREREG_SCHEMA = 'ubts.steer_sim.prereg_follow_wide.v1'
PREREG_ID = 'steer_follow_wide_v2'
DEV_SEEDS = (100, 101, 102, 103, 104)
JUDGE_SEEDS = (3000, 3001, 3002, 3003, 3004)
SEED_SETS = dict(dev=DEV_SEEDS, judge=JUDGE_SEEDS)
MAX_DEV_ITERATIONS = 3
FRONT_BUMPER_M = FM.FRONT_BUMPER_M
HALF_WIDTH_M = FM.HALF_WIDTH_M
FOOTPRINT_LENGTH_M, FOOTPRINT_WIDTH_M = 2. * FRONT_BUMPER_M, 2. * HALF_WIDTH_M
FAULT_AT_S = 20.                     # sensor / operator faults: SIM time
DIST_AT_S = 6.                       # steering disturbances: SIM time (DISTURBANCE_CONDITIONS_FOR_S1C, ST-11 t0)
ROW_DURATION_S = 200.                # runs that should end at the row end (0.2 m/s needs ~125 s)
PAUSE_DURATION_S = 60.               # runs that should pause at 20 s and never restart (40 s of watching)
SETTLE_M = FM.SETTLE_M
PROFILE_V03 = 'front_steer_v0_3_placeholder'
STEER_FAULTS = ('POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK', 'STALL', 'TRACK', 'THERMAL')
ABOVE_SIZE_FAULTS = ('STALL', 'TRACK', 'DRV_STUCK', 'POLARITY')     # SIM_ACCEPTANCE amendment 43 (5)
# Registered criteria (sources: the pre-registration document 4). Do not change after a run.
CRITERIA = dict(
    lateral_rmse_m=.20, lateral_p95_m=.35, lateral_max_m=.50, heading_rmse_deg=5., heading_max_deg=15.,
    row_end_error_m=1.0, speed_max_m_s=.50, speed_max_fast_m_s=.55, min_clearance_m=0., after_terminal_window_s=3.,
    stale_drive_zero_s=.1, lidar_max_age_s=.45, imu_max_age_s=.40, link_drive_zero_s=.55, rest_speed_m_s=.02,
    steer_fault_within_s=3., pw08_cell_ratio_tolerance=.01)


# --- row geometry ---------------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class RowGeometry:
    """Two row lines of posts (trees) at d = +-half_width_m around a centre line through the origin along +x: straight
    (curvature 0) or an arc of constant curvature (+ = turning left). Trees every spacing_m from start_s_m to end_s_m
    (the last tree line), missing trees, end posts, extra posts. The row frame: s along the centre line, d to the left."""

    half_width_m: float = FM.ROW_HALF_WIDTH_M
    spacing_m: float = FM.POST_SPACING_M
    post_radius_m: float = FM.POST_RADIUS_M
    start_s_m: float = FM.ROW_START_X_M
    end_s_m: float = FM.ROW_END_X_M
    curvature_1_m: float = 0.
    gaps: tuple = ()                 # ((side 'L' / 'R' / 'B', s of the first missing tree, count), ...)
    end_post_m: object = None        # None, or end posts this far beyond the last tree on both lines
    end_post_radius_m: float = .06   # G-SIM end post: a 0.10 m square (gsim_follow.post_positions)
    extra: tuple = ()                # ((s, d, radius), ...) other posts (an obstacle on the centre line)

    def centre(self, s):
        k = self.curvature_1_m
        if k == 0.:
            return float(s), 0., 0.
        return math.sin(k * s) / k, (1. - math.cos(k * s)) / k, k * s

    def point(self, s, d):
        x, y, heading = self.centre(s)
        return x - d * math.sin(heading), y + d * math.cos(heading)

    def tree_s(self):
        count = int(math.floor((self.end_s_m - self.start_s_m) / self.spacing_m + 1e-9)) + 1
        return [self.start_s_m + self.spacing_m * k for k in range(count)]

    def missing(self):
        """{(side, k)} of the removed trees (k: index in tree_s())."""
        trees = self.tree_s()
        out = set()
        for side, first_s, count in self.gaps:
            k0 = int(np.argmin([abs(s - first_s) for s in trees]))
            for k in range(k0, k0 + int(count)):
                for name in ('L', 'R') if side == 'B' else (side,):
                    out.add((name, k))
        return out

    def posts(self):
        """W0 markers [[x, y, radius], ...] (mm rounding, as the rig's world spec keeps them)."""
        out, gone = [], self.missing()
        for k, s in enumerate(self.tree_s()):
            for name, d in (('L', self.half_width_m), ('R', -self.half_width_m)):
                if (name, k) not in gone:
                    x, y = self.point(s, d)
                    out.append([round(x, 3), round(y, 3), self.post_radius_m])
        if self.end_post_m is not None:
            for d in (self.half_width_m, -self.half_width_m):
                x, y = self.point(self.end_s_m + float(self.end_post_m), d)
                out.append([round(x, 3), round(y, 3), self.end_post_radius_m])
        for s, d, radius in self.extra:
            x, y = self.point(s, d)
            out.append([round(x, 3), round(y, 3), float(radius)])
        return out

    def last_tree_s(self):
        trees, gone = self.tree_s(), self.missing()
        return max(s for k, s in enumerate(trees) if ('L', k) not in gone or ('R', k) not in gone)

    def last_structure_s(self):
        """s of the last structure on the row lines (decision R50): the end post if there is one, else the last tree."""
        last = self.last_tree_s()
        return last if self.end_post_m is None else max(last, self.end_s_m + float(self.end_post_m))

    def frame(self, x, y, yaw):
        """(s, d, heading error) arrays of world poses (base or any point)."""
        x, y, yaw = (np.asarray(v, dtype=float) for v in (x, y, yaw))
        k = self.curvature_1_m
        if k == 0.:
            return x, y, np.arctan2(np.sin(yaw), np.cos(yaw))
        radius = 1. / k
        sign = 1. if k > 0. else -1.
        phi = np.arctan2(x, sign * (radius - y))
        s = phi / abs(k)
        d = sign * (abs(radius) - np.hypot(x, y - radius))
        err = yaw - k * s
        return s, d, np.arctan2(np.sin(err), np.cos(err))

    def start_pose(self, s, d, heading_deg):
        """World (x, y, yaw_deg) of the row-frame start (s, d, heading error)."""
        x, y = self.point(s, d)
        return round(x, 4), round(y, 4), round(math.degrees(self.centre(s)[2]) + float(heading_deg), 4)

    def to_dict(self):
        out = asdict(self)
        out['gaps'] = [list(g) for g in self.gaps]
        out['extra'] = [list(e) for e in self.extra]
        return out


def geometry(changes=None):
    values = dict(changes or {})
    for key in ('gaps', 'extra'):
        if key in values:
            values[key] = tuple(tuple(v) for v in values[key])
    return RowGeometry(**values)


# --- conditions -------------------------------------------------------------------------------------------------------------
def _gnss_jumps(size_m):
    """Three one-fix spikes at 30, 45, 60 s (perception_compare's gnss_jump_* rule)."""
    return [dict(at_s=t, east_m=round(size_m * math.cos(k), 6), north_m=round(size_m * math.sin(k), 6), duration_s=.5)
            for k, t in enumerate((30., 45., 60.))]


def _blackout(which, duration_s):
    return [dict(kind='sensor_blackout', at_s=FAULT_AT_S, params=dict(duration_s=duration_s, which=which))]


def _torque(torque_nm, duration_s):
    return [dict(kind='steer_torque', at_s=DIST_AT_S, params=dict(torque_nm=float(torque_nm),
                                                                  duration_s=float(duration_s)))]


TERRAIN = dict(steer_terrain_impacts=dict(rate_per_m=1.0, torque_sd_nm=1.0, duration_s=.05),
               steer_terrain_push=dict(sd_nm=.5, corr_length_m=2.0))
TERRAIN_X2 = dict(steer_terrain_impacts=dict(rate_per_m=1.0, torque_sd_nm=2.0, duration_s=.05),
                  steer_terrain_push=dict(sd_nm=1.0, corr_length_m=2.0))

# name, family, expect, changes. Every condition runs the registered seed set (dev or judge).
CONDITIONS = (
    # A. row geometry
    dict(name='nominal', family='geometry', expect='row_end'),
    dict(name='curve_l60', family='geometry', expect='row_end', world=dict(curvature_1_m=1. / 60.)),
    dict(name='curve_r60', family='geometry', expect='row_end', world=dict(curvature_1_m=-1. / 60.)),
    dict(name='curve_l30', family='geometry', expect='row_end', world=dict(curvature_1_m=1. / 30.)),
    dict(name='curve_r30', family='geometry', expect='row_end', world=dict(curvature_1_m=-1. / 30.)),
    dict(name='gap1_left', family='geometry', expect='row_end', world=dict(gaps=[['L', 12.0, 1]])),
    dict(name='gap2_right', family='geometry', expect='row_end', world=dict(gaps=[['R', 12.0, 2]])),
    dict(name='gap3_left', family='geometry', expect='row_end', world=dict(gaps=[['L', 10.5, 3]])),
    dict(name='gap1_both', family='geometry', expect='row_end', world=dict(gaps=[['B', 12.0, 1]])),
    dict(name='gap3_both', family='geometry', expect='row_end', world=dict(gaps=[['B', 10.5, 3]])),
    dict(name='endpost_0p6', family='geometry', expect='row_end', world=dict(end_post_m=.6)),
    dict(name='endpost_1p2', family='geometry', expect='row_end', world=dict(end_post_m=1.2)),
    dict(name='endpost_2p0', family='geometry', expect='row_end', world=dict(end_post_m=2.0)),
    dict(name='width_3p0', family='geometry', expect='row_end', world=dict(half_width_m=1.5)),
    dict(name='width_4p0', family='geometry', expect='row_end', world=dict(half_width_m=2.0)),
    dict(name='spacing_1p0', family='geometry', expect='row_end', world=dict(spacing_m=1.0)),
    dict(name='spacing_3p0', family='geometry', expect='row_end', world=dict(spacing_m=3.0)),
    # W2: 10/10 development corridors (the rig refuses the sealed ones)
    dict(name='w2_042301_c03', family='w2', expect='row_end', w2=dict(record='042301', corridor_id='c03')),
    dict(name='w2_040927_c06', family='w2', expect='row_end', w2=dict(record='040927', corridor_id='c06')),
    dict(name='w2_045553_c03', family='w2', expect='row_end', w2=dict(record='045553', corridor_id='c03')),
    # B. start offsets (RMSE / p95 after SETTLE_M)
    dict(name='lateral_p040', family='start', expect='row_end', start=(0., .40, 0.), settle_m=SETTLE_M),
    dict(name='lateral_n040', family='start', expect='row_end', start=(0., -.40, 0.), settle_m=SETTLE_M),
    dict(name='heading_p10', family='start', expect='row_end', start=(0., 0., 10.), settle_m=SETTLE_M),
    dict(name='heading_n10', family='start', expect='row_end', start=(0., 0., -10.), settle_m=SETTLE_M),
    # C. vehicle: side pull (drive left / right difference as the curvature wander mean, the profile range end) and
    # steering zero offset
    dict(name='pull_left', family='vehicle', expect='row_end', wander_mean_deg_per_m=.6),
    dict(name='pull_right', family='vehicle', expect='row_end', wander_mean_deg_per_m=-.6),
    dict(name='steer_zero_p2', family='vehicle', expect='row_end', profile={'steer_zero_offset_deg': 2.0}),
    dict(name='steer_zero_n2', family='vehicle', expect='row_end', profile={'steer_zero_offset_deg': -2.0}),
    # D. target speed (0.4 is nominal)
    dict(name='speed_0p2', family='speed', expect='row_end', controller=dict(cruise_speed_m_s=.2)),
    dict(name='speed_0p5', family='speed', expect='row_end', controller=dict(cruise_speed_m_s=.5),
         speed_max_key='speed_max_fast_m_s'),
    # E. drive table mismatch (PW-08 five steps, battery, start threshold; the plant = the tuning file table x ...)
    dict(name='table_x070', family='drive_table', expect='row_end', drive_table=dict(factor=.70)),
    dict(name='table_x085', family='drive_table', expect='row_end', drive_table=dict(factor=.85)),
    dict(name='table_x100', family='drive_table', expect='row_end', drive_table=dict(factor=1.00)),
    dict(name='table_x115', family='drive_table', expect='row_end', drive_table=dict(factor=1.15)),
    dict(name='table_x130', family='drive_table', expect='row_end', drive_table=dict(factor=1.30)),
    dict(name='battery_0p85', family='drive_table', expect='row_end', drive_table=dict(duty_scale=.85)),
    dict(name='start_threshold_p15', family='drive_table', expect='row_end', drive_table=dict(duty_offset_pct=15.)),
    # F. steering-axis disturbance (profile v0.3; DISTURBANCE_CONDITIONS_FOR_S1C; executor outer loop on)
    dict(name='dist_none_v03', family='disturbance', expect='row_end', profile_id=PROFILE_V03, no_steer_fault=True),
    dict(name='dist_step_p2p5', family='disturbance', expect='row_end', profile_id=PROFILE_V03, faults=_torque(2.5, 3.),
         no_steer_fault=True),
    dict(name='dist_step_n2p5', family='disturbance', expect='row_end', profile_id=PROFILE_V03,
         faults=_torque(-2.5, 3.), no_steer_fault=True),
    dict(name='dist_push_2p0', family='disturbance', expect='row_end', profile_id=PROFILE_V03,
         faults=_torque(2.0, 10.), no_steer_fault=True),
    dict(name='dist_push_1p0', family='disturbance', expect='row_end', profile_id=PROFILE_V03,
         faults=_torque(1.0, 20.), no_steer_fault=True),
    dict(name='dist_terrain', family='disturbance', expect='row_end', profile_id=PROFILE_V03, profile=TERRAIN,
         no_steer_fault=True),
    dict(name='dist_terrain_x2', family='disturbance', expect='row_end_or_steer_fault', profile_id=PROFILE_V03,
         profile=TERRAIN_X2),
    dict(name='dist_above_5', family='disturbance', expect='steer_fault', profile_id=PROFILE_V03,
         faults=_torque(5.0, 5.)),
    # G. sensors
    dict(name='lidar_dropout_0p2', family='sensor', expect='row_end', faults=_blackout('lidar', .2)),
    dict(name='lidar_stale_1p0', family='sensor', expect='stale_lidar', faults=_blackout('lidar', 1.)),
    dict(name='imu_dropout_0p2', family='sensor', expect='row_end', faults=_blackout('imu', .2)),
    dict(name='imu_stale_1p0', family='sensor', expect='stale_imu', faults=_blackout('imu', 1.)),
    # H. GNSS layer on (geofence) with the virtual GNSS
    dict(name='gnss_nominal', family='gnss', expect='row_end', config=dict(gnss_mode='geofence'), gnss={}),
    dict(name='gnss_dropout_60s', family='gnss', expect='row_end', config=dict(gnss_mode='geofence'),
         gnss=dict(dropouts=[[10., 70.]])),
    dict(name='gnss_jump_5m', family='gnss', expect='row_end', config=dict(gnss_mode='geofence'),
         gnss=dict(jumps=_gnss_jumps(5.))),
    dict(name='gnss_jump_15m', family='gnss', expect='row_end', config=dict(gnss_mode='geofence'),
         gnss=dict(jumps=_gnss_jumps(15.))),
    # I. operator link, J. obstacle
    dict(name='operator_link_lost', family='operator', expect='operator_link_lost',
         actions=[dict(op='screen_hidden', at_s=FAULT_AT_S)]),
    dict(name='obstacle', family='obstacle', expect='obstacle_ahead', world=dict(extra=[[12.0, 0.0, FM.POST_RADIUS_M]])),
)
EXPECTS = ('row_end', 'stale_lidar', 'stale_imu', 'operator_link_lost', 'obstacle_ahead', 'steer_fault',
           'row_end_or_steer_fault')


def condition(name):
    for c in CONDITIONS:
        if c['name'] == name:
            return c
    raise KeyError(name)


def duration(c):
    if c.get('w2'):
        return 300.
    if c['expect'] in ('stale_lidar', 'stale_imu', 'operator_link_lost', 'obstacle_ahead'):
        return PAUSE_DURATION_S
    return ROW_DURATION_S


def prereg():
    """The machine-readable pre-registration (docs/steering/PREREG_STEER_FOLLOW_V2.json)."""
    return dict(schema=PREREG_SCHEMA, id=PREREG_ID, result_kind='placeholder-based SIM result',
                simulator='F-SIM T1 rig, executor in the loop (tools/row_executor/sim_in_loop_steer.py)',
                controller=dict(FM.CONTROLLER), seeds=dict(dev=list(DEV_SEEDS), judge=list(JUDGE_SEEDS)),
                max_dev_iterations=MAX_DEV_ITERATIONS, criteria=CRITERIA, settle_m=SETTLE_M, fault_at_s=FAULT_AT_S,
                disturbance_at_s=DIST_AT_S, footprint=dict(front_bumper_m=FRONT_BUMPER_M, half_width_m=HALF_WIDTH_M),
                default_row=RowGeometry().to_dict(), steer_faults=list(STEER_FAULTS),
                above_size_faults=list(ABOVE_SIZE_FAULTS),
                conditions=[dict(json.loads(json.dumps(c)), duration_s=duration(c)) for c in CONDITIONS],
                runs_per_seed_set=len(CONDITIONS) * len(DEV_SEEDS),
                pw08_cell=dict(duties=list(PW08_DUTIES), cell=list(PW08_CELL), cell_speed_factor=PW08_FACTOR,
                               distance_m=PW08_DISTANCE_M, window=list(PW08_WINDOW_S), seed=PW08_SEED,
                               tolerance=CRITERIA['pw08_cell_ratio_tolerance']))


# --- tuning file -> plant drive table ------------------------------------------------------------------------------------------
def tuning_table(path=None):
    """(speed table points, file SHA-256, path) of a field tuning file (None = the package default), read by the
    executor's own reader (row_executor.field_tuning)."""
    A._sim_in_loop()                                  # puts tools/ on sys.path (row_executor)
    from row_executor import field_tuning as FT
    tfile = FT.load(path)
    return [list(p) for p in tfile.values['speed_table']], tfile.sha256, tfile.path


def drive_table(changes, path=None):
    """RigConfig.drive_table of a condition: the tuning file's table with factor / duty_scale / duty_offset_pct."""
    points, sha, _ = tuning_table(path)
    return dict(points=points, factor=float(changes.get('factor', 1.)), duty_scale=float(changes.get('duty_scale', 1.)),
                duty_offset_pct=float(changes.get('duty_offset_pct', 0.)), source='field_tuning sha256 ' + sha)


# --- episodes ---------------------------------------------------------------------------------------------------------------
def spec(c, seed, name=None):
    """Episode spec of one run (a profile override is an object: the spec is not JSON-able then)."""
    from steering.safety_scripts import catalogue as CAT
    rig = {}
    from steering.vehicle_profile import PROFILES_DIR, load_profile
    base = load_profile() if not c.get('profile_id') else load_profile(PROFILES_DIR / (c['profile_id'] + '.yaml'))
    overrides = dict(c.get('profile') or {})
    if 'wander_mean_deg_per_m' in c:
        wander = dict(base.value('curvature_wander'))
        wander['mean_deg_per_m'] = float(c['wander_mean_deg_per_m'])
        overrides['curvature_wander'] = wander
    if overrides:
        rig['profile'] = base.with_overrides(overrides, label=c['name'])
    elif c.get('profile_id'):
        rig['profile'] = base
    if c.get('gnss') is not None:
        rig['gnss'] = json.loads(json.dumps(c['gnss']))
    if c.get('drive_table'):
        rig['drive_table'] = drive_table(c['drive_table'])
    if c.get('w2'):
        world = dict(kind='W2', record=c['w2']['record'], corridor_id=c['w2']['corridor_id'],
                     direction=c['w2'].get('direction', 'fwd'))
        start = (1., 0., 0.)
    else:
        geo = geometry(c.get('world'))
        world = dict(kind='W0', markers=geo.posts())
        start = geo.start_pose(*c.get('start', (0., 0., 0.)))
    controller = dict(FM.CONTROLLER, **(c.get('controller') or {}))
    return CAT.episode(name or 'wide_%s_s%d' % (c['name'], seed), controller=controller, world=world,
                       faults=[dict(f) for f in c.get('faults') or ()], actions=[dict(a) for a in c.get('actions') or ()],
                       rig=rig, config=dict(c.get('config') or {}), duration_s=duration(c), seed=int(seed),
                       start_pose=start, auto_repress=False)


def printable(episode, c):
    """The episode spec for RESULT.json: the profile object replaced by its identity and overrides."""
    out = dict(episode, rig=dict(episode['rig']))
    if 'profile' in out['rig']:
        out['rig']['profile'] = dict(profile_id=c.get('profile_id') or 'default', overrides=c.get('profile') or {},
                                     wander_mean_deg_per_m=c.get('wander_mean_deg_per_m'))
    return out


# --- metrics ----------------------------------------------------------------------------------------------------------------
def _w2_frame(world):
    def frame(x, y, yaw):
        rows = np.array([world.to_field(float(a), float(b), float(c)) for a, b, c in zip(x, y, yaw)],
                        dtype=float).reshape(-1, 3)
        return rows[:, 0], rows[:, 1], rows[:, 2]
    return frame


def _w2_clearance(world, x, y, yaw):
    from closed_loop import raster_corridor as rc
    best = math.inf
    for a, b, c in zip(x, y, yaw):
        best = min(best, float(rc.footprint_clearances(world.map, float(a), float(b), float(c), FOOTPRINT_LENGTH_M,
                                                       FOOTPRINT_WIDTH_M)['hard']))
    return best


def steer_fault_lines(art):
    """[(code, t_s)] of the MCU F lines of steering faults (STEER_FAULTS), SIM time from the queued tick (1 ms)."""
    out = []
    for text, queued, _ in M.mcu_lines(art, 'F'):
        parts = text.split('*')[0].split(' ')
        if len(parts) > 1 and parts[1] in STEER_FAULTS:
            out.append((parts[1], round(queued / 1000., 3)))
    return out


def measure(c, arrays, ticks, summary, world, markers, art, gnss_report=None):
    """Metrics of one run (truth: evaluation only)."""
    t = np.asarray(arrays['t_s'], dtype=float)
    x, y = np.asarray(arrays['truth_x_m'], dtype=float), np.asarray(arrays['truth_y_m'], dtype=float)
    yaw = np.asarray(arrays['truth_yaw_rad'], dtype=float)
    v = np.asarray(arrays['truth_v_m_s'], dtype=float)
    throttle = np.asarray(arrays['ctrl_throttle_pct'], dtype=float)
    bx, by = x + FRONT_BUMPER_M * np.cos(yaw), y + FRONT_BUMPER_M * np.sin(yaw)
    if c.get('w2'):
        s, d, h = (np.asarray(arrays[k], dtype=float) for k in ('truth_field_x_m', 'truth_field_y_m',
                                                               'truth_field_heading_rad'))
        bumper_s = float(_w2_frame(world)(bx[-1:], by[-1:], yaw[-1:])[0][0])
        truth = world.corridor_truth(0)
        last_structure, last_tree = float(max(truth['line_end_x'])), float(max(truth['line_end_x']))
        clearance = _w2_clearance(world, x[::25], y[::25], yaw[::25])
    else:
        geo = geometry(c.get('world'))
        s, d, h = geo.frame(x, y, yaw)
        bumper_s = float(geo.frame(bx[-1:], by[-1:], yaw[-1:])[0][0])
        last_structure, last_tree = geo.last_structure_s(), geo.last_tree_s()
        clearance = FM.clearance(x[::5], y[::5], yaw[::5], markers)
    driving = np.flatnonzero(throttle > 0)
    faults = steer_fault_lines(art)
    out = dict(end_reason=summary.get('end_reason'), ended_at_s=summary.get('ended_at_s'),
               final_state=ticks[-1].get('state') if ticks else None,
               final_reason=ticks[-1].get('reason') if ticks else None, drove=bool(len(driving)),
               min_clearance_m=round(clearance, 4), rest_speed_m_s=round(float(abs(v[-1])), 4),
               rest_s_m=round(float(s[-1]), 3), rest_d_m=round(float(d[-1]), 3),
               rest_heading_deg=round(float(np.degrees(h[-1])), 3),
               last_structure_s_m=round(last_structure, 3), last_tree_s_m=round(last_tree, 3),
               row_end_error_m=round(bumper_s - last_structure, 3), row_end_error_last_tree_m=round(bumper_s - last_tree, 3),
               steer_fault_lines=[list(f) for f in faults],
               host_latches=[list(item) for item in summary.get('host_latches') or ()],
               pauses_after_start=[], first_pause=None)
    gnss = next((r['gnss'] for r in reversed(ticks) if 'gnss' in r), None)
    if gnss is not None:
        out['gnss'] = dict(counts=gnss.get('counts'), flags=gnss.get('flags'), fence_tripped=gnss.get('fence_tripped'),
                           aligned=gnss.get('aligned'))
    if gnss_report is not None:
        out['gnss_receiver'] = dict(counts=gnss_report['counts'], jump_stamps=gnss_report['jump_stamps'])
    if not len(driving):
        return out
    a, b = int(driving[0]), int(driving[-1])
    t_a = float(t[a])
    pauses = [(float(e[0]), e[2]) for e in summary.get('events', ()) if e[1] == 'PAUSED' and float(e[0]) > t_a]
    out['pauses_after_start'] = sorted({reason for _, reason in pauses})
    out['first_pause'] = None if not pauses else [round(pauses[0][0], 3), pauses[0][1]]
    path = np.concatenate(([0.], np.cumsum(np.hypot(np.diff(x[a:b + 1]), np.diff(y[a:b + 1])))))
    k = a + int(np.searchsorted(path, float(c.get('settle_m', 0.))))
    out.update(drive_start_s=round(t_a, 2), drive_stop_s=round(float(t[b]), 2), distance_m=round(float(path[-1]), 2),
               lateral_max_m=round(float(np.max(np.abs(d[a:b + 1]))), 4),
               heading_max_deg=round(float(np.degrees(np.max(np.abs(h[a:b + 1])))), 3),
               speed_mean_m_s=round(float(np.mean(v[a:b + 1])), 3), speed_max_m_s=round(float(np.max(v[a:b + 1])), 3),
               settle_m=float(c.get('settle_m', 0.)))
    if k < b:
        out.update(lateral_rmse_m=round(float(np.sqrt(np.mean(d[k:b + 1] ** 2))), 4),
                   lateral_p95_m=round(float(np.percentile(np.abs(d[k:b + 1]), 95)), 4),
                   heading_rmse_deg=round(float(np.degrees(np.sqrt(np.mean(h[k:b + 1] ** 2)))), 3))
    # speed error of the cruise phase (report): true speed - the controller's target speed at the executor ticks
    cruise = [r for r in ticks if r.get('state') == 'ACTIVE' and (r.get('T') or 0) > 0
              and (r.get('sr') or {}).get('cap') == 'cruise' and (r.get('sr') or {}).get('vt') is not None]
    if cruise:
        err = [float(np.interp(r['t'], t, v)) - float(r['sr']['vt']) for r in cruise]
        out['cruise_speed_error_m_s'] = dict(mean=round(float(np.mean(err)), 4),
                                             p95_abs=round(float(np.percentile(np.abs(err), 95)), 4), ticks=len(err))
    # stopping distance (report): travelled from the controller's stop request to rest
    stop = next((r for r in ticks if r['t'] >= t_a and (r.get('ctl') or [None])[0] == 'STOP_REQUESTED'), None)
    if stop is not None:
        j = int(np.searchsorted(t, stop['t']))
        out['stop_request_s'] = round(float(stop['t']), 3)
        out['stop_distance_m'] = round(float(np.sum(np.hypot(np.diff(x[j:]), np.diff(y[j:])))), 3)
    for which, key, limit in (('lidar', 'lidar_age', CRITERIA['lidar_max_age_s']),
                              ('imu', 'imu_age', CRITERIA['imu_max_age_s'])):
        stale = next((r for r in ticks if (r.get(key) or 0.) > limit and r['t'] >= t_a), None)
        if stale is not None:
            zero = next((r for r in ticks if r['t'] >= stale['t'] and not (r.get('T') or 0)), None)
            out['%s_stale_first_tick_s' % which] = stale['t']
            out['%s_stale_drive_zero_s' % which] = None if zero is None else round(zero['t'] - stale['t'], 3)
            out['%s_restart_after_stale' % which] = bool(zero is None or any((r.get('T') or 0) > 0 for r in ticks
                                                                             if r['t'] > zero['t']))
    hidden = next((a for a in c.get('actions') or () if a['op'] == 'screen_hidden'), None)
    if hidden is not None:
        # the screen beats once per executor tick while visible; the action runs before the beat of its tick, so the
        # last heartbeat is the tick before the action. Drive 0 = the first executor tick (its sent command, the
        # amendment 31 'drive-0 request sent') with T = 0 at or after the last heartbeat.
        beats = [r['t'] for r in ticks if r['t'] < float(hidden['at_s']) - 1e-9]
        last_beat = max(beats) if beats else None
        zero = None if last_beat is None else next((r for r in ticks if r['t'] >= last_beat and not (r.get('T') or 0)),
                                                   None)
        out['link_last_beat_s'] = last_beat
        out['link_drive_zero_s'] = None if zero is None else round(zero['t'] - last_beat, 3)
        out['link_restart'] = bool(zero is None or any((r.get('T') or 0) > 0 for r in ticks if r['t'] > zero['t']))
        trace_zero = None if last_beat is None else next((float(tt) for tt, th in zip(t, throttle)
                                                          if tt >= last_beat and th <= 0.), None)
        out['link_trace_zero_s'] = None if trace_zero is None else round(trace_zero - last_beat, 3)   # report
    return out


def judge(c, m):
    """Registered checks of one run: list of dict(check, ok, measured, limit)."""
    crit = CRITERIA
    checks = []

    def add(name, ok, measured, limit):
        checks.append(dict(check=name, ok=bool(ok), measured=measured, limit=limit))

    def below(name, key, unit, limit_key=None):
        value, limit = m.get(key), crit[limit_key or key]
        add(name, value is not None and value <= limit, value, '<= %g %s' % (limit, unit))

    def row_end_checks(prefix=''):
        add(prefix + 'ends at the row end', m['end_reason'] == 'row_end', m['end_reason'], 'row_end')
        add(prefix + 'no pause after the drive started', not m['pauses_after_start'], m['pauses_after_start'], 'none')
        below(prefix + 'lateral RMSE', 'lateral_rmse_m', 'm')
        below(prefix + 'lateral p95', 'lateral_p95_m', 'm')
        below(prefix + 'lateral max', 'lateral_max_m', 'm')
        below(prefix + 'heading RMSE', 'heading_rmse_deg', 'deg')
        below(prefix + 'heading max', 'heading_max_deg', 'deg')
        below(prefix + 'true speed max', 'speed_max_m_s', 'm/s', c.get('speed_max_key'))
        add(prefix + 'row end stop position e (last structure)', abs(m['row_end_error_m']) <= crit['row_end_error_m'],
            m['row_end_error_m'], '[-%g, +%g] m' % (crit['row_end_error_m'], crit['row_end_error_m']))

    def steer_fault_checks(prefix=''):
        lines = m.get('steer_fault_lines') or []
        first = lines[0] if lines else None
        add(prefix + 'first steering fault is a registered one', first is not None and first[0] in ABOVE_SIZE_FAULTS,
            first, 'one of %s' % ', '.join(ABOVE_SIZE_FAULTS))
        onsets = [f['at_s'] for f in c.get('faults') or () if f['kind'] == 'steer_torque']
        if onsets:                    # a torque step has an onset; the terrain processes act all the time
            onset = min(onsets)
            add(prefix + 'steering fault within %g s of the onset' % crit['steer_fault_within_s'],
                first is not None and onset - 1e-9 <= first[1] <= onset + crit['steer_fault_within_s'],
                None if first is None else round(first[1] - onset, 3), '[0, %g] s' % crit['steer_fault_within_s'])
        add(prefix + 'not a row end', m['end_reason'] != 'row_end', m['end_reason'], 'not row_end')
        below(prefix + 'lateral max', 'lateral_max_m', 'm')

    add('no contact', m['min_clearance_m'] > crit['min_clearance_m'], m['min_clearance_m'],
        '> %g m' % crit['min_clearance_m'])
    add('at rest at the end', m['rest_speed_m_s'] <= crit['rest_speed_m_s'], m['rest_speed_m_s'],
        '<= %g m/s' % crit['rest_speed_m_s'])
    add('no guard violation', not str(m['end_reason']).startswith('guard_violation'), m['end_reason'], 'none')
    after = m.get('after_terminal') or {}
    add('no commanded steering move after the terminal moment (A22 rule)', after.get('passed') is True,
        dict((k, after.get(k)) for k in ('passed', 'complete', 'max_target_move_deg', 'executor_command_move_deg')),
        'passed')
    expect = c['expect']
    if expect == 'row_end':
        row_end_checks()
    elif expect == 'row_end_or_steer_fault':
        if m.get('steer_fault_lines'):
            steer_fault_checks('(fault branch) ')
        else:
            row_end_checks('(row end branch) ')
    elif expect == 'steer_fault':
        steer_fault_checks()
    elif expect in ('stale_lidar', 'stale_imu'):
        which = expect.split('_')[1]
        first = m.get('first_pause')
        add('first pause is %s' % expect, bool(first) and first[1] == expect, first, expect)
        value = m.get('%s_stale_drive_zero_s' % which)
        add('drive command 0 after the %s input is stale' % which, value is not None
            and value <= crit['stale_drive_zero_s'], value, '<= %g s' % crit['stale_drive_zero_s'])
        add('no restart', m.get('%s_restart_after_stale' % which) is False, m.get('%s_restart_after_stale' % which),
            False)
    elif expect == 'operator_link_lost':
        first = m.get('first_pause')
        add('first pause is operator_link_lost', bool(first) and first[1] == 'operator_link_lost', first,
            'operator_link_lost')
        value = m.get('link_drive_zero_s')
        add('drive command 0 after the last heartbeat', value is not None and value <= crit['link_drive_zero_s'], value,
            '<= %g s' % crit['link_drive_zero_s'])
        add('no restart', m.get('link_restart') is False, m.get('link_restart'), False)
    else:
        add('ends with %s' % expect, m['end_reason'] == expect, m['end_reason'], expect)
    if c.get('no_steer_fault'):
        add('no steering fault line of the MCU', not m.get('steer_fault_lines'), m.get('steer_fault_lines'), 'none')
        add('no host bridge latch', not m.get('host_latches'), m.get('host_latches'), 'none')
    if c.get('gnss') is not None:
        gnss = m.get('gnss') or {}
        add('GNSS fence not tripped', gnss.get('fence_tripped') is False, gnss.get('fence_tripped'), False)
        injected = len((c['gnss'] or {}).get('jumps') or ())
        if injected:
            rejected = (gnss.get('counts') or {}).get('jump')
            jumped = ((m.get('gnss_receiver') or {}).get('counts') or {}).get('jumped')
            add('GNSS jumps rejected = fixes carrying a jump', rejected is not None and rejected == jumped,
                dict(rejected=rejected, fixes_with_jump=jumped), 'equal')
    return checks


# --- run ------------------------------------------------------------------------------------------------------------------
def run_one(item):
    cond, seed, out_root = item
    S = A._sim_in_loop()
    started = time.perf_counter()
    episode = spec(cond, seed)
    result, facade = S.run_episode(episode, None, keep=True)
    try:
        if out_root is not None:
            result = facade.write(os.path.join(out_root, episode['name']), printable(episode, cond))
        arrays = facade.trace_arrays()
        ticks = [r for r in facade.log.records if r.get('k') == 'tick']
        art = M.artefacts(facade, printable(episode, cond))
        metrics = measure(cond, arrays, ticks, facade.summary(), facade.rig.world,
                          episode['world'].get('markers') or (), art, facade.rig.gnss_report())
        metrics['after_terminal'] = FM.terminal_window(art, metrics)
        summary = facade.summary()
    finally:
        facade.rig.close()
    checks = judge(cond, metrics)
    return dict(condition=cond['name'], family=cond['family'], seed=seed, name=episode['name'], expect=cond['expect'],
                ok=all(ch['ok'] for ch in checks), failed=[ch['check'] for ch in checks if not ch['ok']], checks=checks,
                metrics=metrics, digests=result['digests'], config_digest=result['config_digest'],
                tuning=summary.get('tuning'), wall_s=round(time.perf_counter() - started, 1))


def run_all(out_dir, seed_set, workers=1, only=None, iteration=None, seeds=None):
    started = time.perf_counter()
    chosen = tuple(seeds) if seeds else SEED_SETS[seed_set]
    items = [(c, seed, os.path.join(out_dir, 'runs')) for c in CONDITIONS for seed in chosen
             if not only or c['name'] in only]
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=min(4, workers)) as pool:
            rows = list(pool.map(run_one, items))
    else:
        rows = [run_one(item) for item in items]
    by = {}
    for r in rows:
        entry = by.setdefault(r['condition'], dict(family=r['family'], runs=0, passed=0, failed_checks={}))
        entry['runs'] += 1
        entry['passed'] += int(r['ok'])
        for name in r['failed']:
            entry['failed_checks'][name] = entry['failed_checks'].get(name, 0) + 1
    report = dict(schema=SCHEMA, prereg=PREREG_ID, result_kind=A.RESULT_KIND, field_validated=False,
                  seed_set=seed_set, seeds=list(chosen), iteration=iteration,
                  complete=not only and not seeds, criteria=CRITERIA, runs=len(rows),
                  passed=sum(1 for r in rows if r['ok']), ok=bool(rows) and all(r['ok'] for r in rows),
                  conditions=by, rows=rows, workers=workers, wall_s=round(time.perf_counter() - started, 1))
    _write_json(os.path.join(out_dir, 'FOLLOW_WIDE.json'), report)
    return report


CONDENSED_METRICS = ('end_reason', 'ended_at_s', 'first_pause', 'pauses_after_start', 'drive_start_s', 'drive_stop_s',
                     'distance_m', 'lateral_rmse_m', 'lateral_p95_m', 'lateral_max_m', 'heading_rmse_deg',
                     'heading_max_deg', 'speed_mean_m_s', 'speed_max_m_s', 'row_end_error_m',
                     'row_end_error_last_tree_m', 'min_clearance_m', 'rest_speed_m_s', 'steer_fault_lines',
                     'host_latches', 'lidar_stale_drive_zero_s', 'imu_stale_drive_zero_s', 'link_drive_zero_s',
                     'link_trace_zero_s', 'cruise_speed_error_m_s', 'stop_distance_m')


def condense(report, rows=True):
    """A short form of a FOLLOW_WIDE.json report for the repository result file: per condition the passed runs and the
    failed checks; with rows, per run the main metrics, the failed checks and the three SHA-256 digests."""
    out = dict(seed_set=report['seed_set'], seeds=report['seeds'], iteration=report.get('iteration'),
               complete=report['complete'], runs=report['runs'], passed=report['passed'], ok=report['ok'],
               conditions=report['conditions'])
    if rows:
        out['rows'] = []
        for r in report['rows']:
            m = r['metrics']
            gnss = m.get('gnss') or {}
            out['rows'].append(dict(
                condition=r['condition'], seed=r['seed'], ok=r['ok'], failed=r['failed'],
                metrics={k: m.get(k) for k in CONDENSED_METRICS if m.get(k) is not None},
                gnss=None if not gnss else dict(fence_tripped=gnss.get('fence_tripped'),
                                                jump=(gnss.get('counts') or {}).get('jump')),
                digests=r['digests']))
    return out


def _write_json(path, value):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    raw = json.dumps(value, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False, default=str) + '\n'
    with open(path, 'wb') as fh:
        fh.write(raw.encode('utf-8'))


# --- PW-08: one table cell changed -> the steady speed of the executor's table and of the fast SIM plant -------------------
PW08_DUTIES = (25, 50)               # duties at which the steady speed is compared (under the duty cap)
PW08_CELL = (0, 1)                   # (point index, field index) of the changed cell: the first point's speed
PW08_FACTOR = 1.10                   # that cell's speed x 1.10
PW08_DISTANCE_M = 30.
PW08_WINDOW_S = (12., 22.)           # steady window after the start of the drive (SIM s)
PW08_SEED = 3100


def pw08_files(out_dir):
    """The package tuning file and a copy with one table cell changed (written to out_dir). Returns both paths."""
    _, _, base_path = tuning_table()
    with open(base_path, 'rb') as fh:
        raw = fh.read()
    data = json.loads(raw.decode('utf-8'))
    table = data['values']['advanced']['speed_table']
    k, j = PW08_CELL
    table[k][j] = round(float(table[k][j]) * PW08_FACTOR, 4)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'field_tuning.cell_changed.json')
    with open(path, 'wb') as fh:
        fh.write((json.dumps(data, indent=1, ensure_ascii=False) + '\n').encode('utf-8'))
    return base_path, path


def _table_speed(points, duty):
    from steering import steered_plant as SP
    return SP.drive_table_speed(SP.drive_table_spec(points), duty)


def pw08_one(path, duty, out_dir=None):
    """Straight hold at a duty in the default row with the plant on the file's table; the executor reads the same
    file. Returns the steady plant speed (truth mean over PW08_WINDOW_S after the drive started), the executor's table
    speed at that duty and both records' file SHA-256. World: the default row (W0 posts, as the matrix); the first run
    of 2026-11-13 used the empty floor and never started (no corridor for the one-press start)."""
    from steering.safety_scripts import catalogue as CAT
    S = A._sim_in_loop()
    points, sha, _ = tuning_table(path)
    name = 'pw08_%s_d%d' % ('changed' if path and 'cell_changed' in str(path) else 'base', duty)
    controller = {'kind': 'straight_hold', 'drive_pct': int(duty), 'distance_m': PW08_DISTANCE_M}
    # the default row world: a go needs an acquired corridor (one-press start checks); an empty floor never starts
    episode = CAT.episode(name, controller=controller, world={'kind': 'W0', 'markers': RowGeometry().posts()},
                          rig=dict(drive_table=dict(points=points, factor=1., duty_scale=1., duty_offset_pct=0.,
                                                    source='field_tuning sha256 ' + sha)),
                          config=dict(tuning_file=str(path)), duration_s=PW08_WINDOW_S[1] + 8., seed=PW08_SEED,
                          stream_key='pw08_cell')
    result, facade = S.run_episode(episode, None, keep=True)
    try:
        if out_dir is not None:
            facade.write(os.path.join(out_dir, name), episode)
        arrays = facade.trace_arrays()
        cfg = facade.cfg
        identity = facade.rig.identity(include_files=False)
        tuning = facade.summary().get('tuning') or {}
    finally:
        facade.rig.close()
    t, v = np.asarray(arrays['t_s'], dtype=float), np.asarray(arrays['truth_v_m_s'], dtype=float)
    thr = np.asarray(arrays['ctrl_throttle_pct'], dtype=float)
    start = float(t[np.flatnonzero(thr > 0)[0]])
    window = (t >= start + PW08_WINDOW_S[0]) & (t < start + PW08_WINDOW_S[1])
    executor_points = [list(p) for p in (cfg.prior_speed_table or ())]
    return dict(duty=int(duty), file=os.path.basename(str(path)), file_sha256=sha,
                plant_steady_speed_m_s=round(float(np.mean(v[window])), 5), window_rows=int(window.sum()),
                executor_table_speed_m_s=None if not executor_points else round(
                    _table_speed(executor_points, duty) * float(cfg.prior_speed_scale or 1.), 5),
                file_table_speed_m_s=round(_table_speed(points, duty), 5),
                executor_record_sha256=tuning.get('file_sha256') or tuning.get('sha256'),
                rig_record_source=(identity.get('drive_table') or {}).get('source'),
                trace_content_sha256=result['digests']['trace_content_sha256'])


def pw08_cell(out_dir):
    base, changed = pw08_files(out_dir)
    rows = []
    for duty in PW08_DUTIES:
        a = pw08_one(base, duty, os.path.join(out_dir, 'runs'))
        b = pw08_one(changed, duty, os.path.join(out_dir, 'runs'))
        expected = b['file_table_speed_m_s'] / a['file_table_speed_m_s']
        plant = b['plant_steady_speed_m_s'] / a['plant_steady_speed_m_s']
        executor = None if None in (a['executor_table_speed_m_s'], b['executor_table_speed_m_s']) else \
            b['executor_table_speed_m_s'] / a['executor_table_speed_m_s']
        tol = CRITERIA['pw08_cell_ratio_tolerance']
        checks = [dict(check='fast SIM plant ratio', ok=abs(plant / expected - 1.) <= tol,
                       measured=round(plant, 5), limit='%.5f +- %g %%' % (expected, 100 * tol)),
                  dict(check='executor table ratio', ok=executor is not None and abs(executor / expected - 1.) <= tol,
                       measured=None if executor is None else round(executor, 5),
                       limit='%.5f +- %g %%' % (expected, 100 * tol)),
                  dict(check='same file in the executor and rig records (changed run)',
                       ok=bool(b['executor_record_sha256']) and b['rig_record_source'] ==
                       'field_tuning sha256 ' + b['executor_record_sha256'],
                       measured=[b['executor_record_sha256'], b['rig_record_source']], limit='equal SHA-256')]
        rows.append(dict(duty=duty, expected_ratio=round(expected, 5), base=a, changed=b, checks=checks,
                         ok=all(ch['ok'] for ch in checks), gazebo='not run in this bundle (Docker stack: S2 / S5)'))
    report = dict(schema=SCHEMA + '.pw08_cell', prereg=PREREG_ID, result_kind=A.RESULT_KIND, field_validated=False,
                  cell=list(PW08_CELL), factor=PW08_FACTOR, rows=rows, ok=all(r['ok'] for r in rows))
    _write_json(os.path.join(out_dir, 'PW08_CELL.json'), report)
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--out')
    p.add_argument('--seeds', choices=sorted(SEED_SETS))
    p.add_argument('--seed', type=int, nargs='*', help='explicit seeds (a partial run, not a judgement)')
    p.add_argument('--workers', type=int, default=1)
    p.add_argument('--only', nargs='*')
    p.add_argument('--iteration', type=int)
    p.add_argument('--prereg', action='store_true')
    p.add_argument('--pw08-cell', action='store_true')
    args = p.parse_args(argv)
    if args.prereg:
        sys.stdout.buffer.write((json.dumps(prereg(), sort_keys=True, indent=1) + '\n').encode('utf-8'))   # LF
        return 0
    if not args.out:
        raise SystemExit('--out is required')
    from steering.scene_spec import inside_repository
    if inside_repository(args.out):
        raise SystemExit('refused: %s lies inside the repository (runs live outside)' % args.out)
    if args.pw08_cell:
        report = pw08_cell(args.out)
        for row in report['rows']:
            print('%s duty %d expected %.4f %s' % ('ok  ' if row['ok'] else 'FAIL', row['duty'], row['expected_ratio'],
                                                   '; '.join('%s %s' % (c['check'], c['measured']) for c in row['checks'])))
        return 0 if report['ok'] else 1
    if not args.seeds and not args.seed:
        raise SystemExit('--seeds dev|judge (or --seed N ... for a partial run) is required')
    report = run_all(args.out, args.seeds or 'explicit', args.workers, args.only, args.iteration, args.seed)
    for row in report['rows']:
        m = row['metrics']
        print('%-4s %-20s s%-4d end %s e %s rmse %s max %s head %s v %s clear %s %s' % (
            'ok' if row['ok'] else 'FAIL', row['condition'], row['seed'], m.get('end_reason') or m.get('first_pause'),
            m.get('row_end_error_m'), m.get('lateral_rmse_m'), m.get('lateral_max_m'), m.get('heading_max_deg'),
            m.get('speed_max_m_s'), m.get('min_clearance_m'), '' if row['ok'] else 'failed: ' + '; '.join(row['failed'])))
    print('%s: %d of %d runs pass%s, %.0f s' % ('PASS' if report['ok'] else 'FAIL', report['passed'], report['runs'],
                                                '' if report['complete'] else ' (partial)', report['wall_s']))
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
