"""Single-row SIM runner: the REAL executor (mode 'single_row') in the loop on the 10/10 raster corridors (SIM only).

No hardware, no Jetson, no ROS, no Docker, no motor. Per episode (deterministic for (plant, condition, corridor, seed)):
  sim/ubts/closed_loop/harness.run_episode (100 Hz FieldPlant, 10 Hz SIM LiDAR, 20 Hz raw IMU, 20 Hz control) with the
  raster-world hooks of closed_loop/raster_corridor.py (10/10 Cartographer offline maps, SLAM reconstructions, not physical
  truth) and its estimator / controller slots filled by ONE facade around the REAL executor_main.ExecutorLoop
  (sim_in_loop.ExecutorInTheLoop: real S2 scan format 3240 beams at -179.08 deg, ScanStamper, newest scan only,
  NearConfirmFilter, Runner with the vendored estimator and closed_loop/single_row.SingleRowController (the SIM-tree
  modules, byte-identical to the vendor manifest), geometry guard, Backstop, OutputGuard, hold rule) -> BridgeClient
  (deadman) -> the dashboard's REAL RoverControl (read-only <dashboard folder>/scripts/rover_control_core.py, sim_paths
  'dashboard_dir') -> fake motor -> harness command latency -> MCU ramp -> plant.
  Localisation inputs: synthesised 2D LiDAR + raw IMU only (SENSOR_ONLY_SINGLE_ROW). No GPS, no camera, no map, no
  pose, no PWM odometry, no virtual encoder reaches the executor. SIM truth is used only to synthesise the sensors, to
  score the episode (closed_loop.raster_corridor.evaluate_episode) and to END the SIM episode after the post-stop
  observation (evaluation side; see SingleRowInTheLoop).

Plants (PREREG plants): P_fit1003 (primary; sim_in_loop.P_FIT1003 on the 10/10 defaults) and field1010 (the 10/10
  FieldResponseParams defaults, SIM_ASSUMED 50 % deadband). Condition plant_overrides win for the names they set.
  SidedGainPlant (PREREG plants.sided_gain_extension): the MCU-applied duty of each side (after ramp and reversal wait)
  times its gain before the UNCHANGED FieldPlant response; no other change. Nothing weakens the plant.
Executor configuration (one per plant, never per condition / corridor / seed): single_row_executor_config(); SIM
  calibration values (speed at the start base, yaw authority, stop distance) from the plant profile's NOMINAL
  parameters only, as sim_in_loop.executor_config. The inner-wheel rule (breakaway 20 + margin 10 = 30, the config minimum
  and the SIM convention) binds the speed loop's LOWEST base: base_min - max_differential >= 30, so P_fit1003 runs at
  base 50-60 % (about 0.42-0.51 m/s at the nominal per-run speed scale) and cannot reach the 0.20 m/s approach target;
  field1010 runs at base 60-75 %.
Operator emulation (PREREG operator_policy): Start at the first tick (row length: --row-length none = no length, the
  default for this tool; l_op = the PREREG L_op = s_E - s_start + 5.0 m), Arm at the next, ONE hold press, heartbeat
  kept, never released, no STOP, no re-press. --press-policy prereg_fixed (default) presses at t = 1.2 s whatever the
  page says; page_ready presses at the first tick >= 1.2 s at which the page would accept a press (inputs fresh for
  fresh_recover_s + 0.05 s). The first executor PAUSE (other than await_hold) or END after the press is the episode's
  stop (stop_trigger), its category the stop category. A press the Runner ignores ('press_ignored_inputs_not_fresh': the
  inputs were not fresh for 1.0 s at 1.2 s, e.g. 0.2-0.3 s LiDAR latency) is such a PAUSE (UNCLASSIFIED; reported).
Episode end (PREREG episode_termination): after the stop trigger the facade reports the terminal HOLD to the harness once
  the TRUE plant has been at rest (|v| < 0.01 m/s, |body yaw rate incl. a scenario's external yaw| < 0.01 rad/s) for
  1.0 + 3.0 s (rest + post-stop observation for AUTO_RESTART) and no scenario yaw burst starts within the next 1.2 s, or
  20 s after the trigger; the harness then needs 1.0 s more of rest. Collisions end at once (harness);
  MISSED_ROW_END / CORRIDOR_DEPARTURE / LEFT_FIELD / NO_PROGRESS / TIME_CAP are applied post hoc by evaluate_episode;
  max_time_s = TIME_CAP + 5 s.
Verdict (prereg_verdict): the PREREG classes (acceptable terminal outcomes, lateral / heading limits on completions and on
  OBSTACLE / END_STRUCTURE stops, the field1010 bracket rule), all-class rules and fail codes; run summary with R1-R8
  (meaningful for a final run only; a smoke / dev run reports them as not a verdict). Reports in Korean, keys in English.
Outputs (--out-dir): RUN_MANIFEST.json, EPISODES.jsonl (one row per episode, appended as they finish; a re-run skips the
  episode ids already there), SUMMARY.json, REPORT_KO.txt, traces/<episode>.json.gz (harness trace in the corridor frame,
  executor tick log, raw step times), videos/<episode>.mp4 for --video (single_row_video.render_video).
  Corridor-frame values only: no raster or GNSS coordinates.

Usage (from the worktree root; at most 12 workers; waits while the HOLD_MEASUREMENT file of the configured hold folder
exists, sim/ubts/closed_loop/external_paths.py 'hold_dir'):
  python -B tools/row_executor/single_row_sim.py --out-dir <dir> --run-kind smoke \
      --episode P_fit1003:nominal:042301:c03:fwd:0 --episode field1010:nominal:042301:c03:rev:0 --video all --workers 2
  python -B tools/row_executor/single_row_sim.py --out-dir <dir> --run-kind dev --prereg \
      --selection <SELECTION.json of CORRIDOR_FREEZE.json> --seed-set dev --workers 12
Holdout seeds (7, 8) are refused unless --freeze names an existing FREEZE.json (PREREG seeds.holdout_policy).
External locations (steering branch, docs/steering/EXECUTOR_IMPORT_KO.md): PREREG.json from sim_paths
('single_row_prereg': UBTS_SINGLE_ROW_PREREG or the path config file), the corridor catalogue and the 10/10 maps from
sim/ubts/closed_loop/external_paths.py. On the steering branch the sealed corridors (042301 c01 / c04 / c07, both
directions; sim/ubts/steering/scene_spec.sealed_status) are reserved for the steering pre-registration: do not run them.
"""
import argparse
import concurrent.futures
import gzip
import hashlib
import json
import math
import os
import platform
import statistics
import subprocess
import sys
import time
import traceback
from dataclasses import replace

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
SIM_DIR = os.path.join(REPO, 'sim', 'ubts')
for _path in (SIM_DIR, TOOLS):
    if _path not in sys.path:
        sys.path.insert(0, _path)
sys.dont_write_bytecode = True

import numpy as np  # noqa: E402

from closed_loop import harness as SH  # noqa: E402  (SIM tree; the executor verifies it against the vendor manifest)
from closed_loop import raster_corridor as RC  # noqa: E402
from closed_loop.harness import EpisodeConfig  # noqa: E402
from field_response import forward_curve  # noqa: E402
from row_executor import config as XC  # noqa: E402
from row_executor import core as XCORE  # noqa: E402
from row_executor import sim_in_loop as SIL  # noqa: E402
from row_executor import sim_paths as XP  # noqa: E402  (PREREG.json location)

SCHEMA_ROW = 'ubts.single_row.sim_episode.v1'
SCHEMA_MANIFEST = 'ubts.single_row.sim_run_manifest.v1'
SCHEMA_SUMMARY = 'ubts.single_row.sim_summary.v1'
PREREG_REGISTERED_SHA256 = '9c4ff770adc805a49236b4e12a99f33fad6f7e68e671987cb99adfd08bc943a6'
PLANTS = {'P_fit1003': 'p_fit1003', 'field1010': 'field1010'}       # PREREG plant -> sim_in_loop plant profile
START_S_M = 1.0                      # PREREG start pose: base centre at s = 1.0 m
L_OP_EXTRA_M = 5.0                   # PREREG L_op = s_E - s_start + 5.0 m
PRESS_AT_S = 1.2                     # PREREG operator_policy
PRESS_READY_MARGIN_S = .05           # page_ready: inputs fresh for fresh_recover_s + this
REST_V_M_S, REST_W_RAD_S = .01, .01  # PREREG rest
REST_THEN_OBSERVE_S = 1. + 3.        # rest 1.0 s, then 3.0 s post-stop observation (AUTO_RESTART)
STOP_SIGNAL_FALLBACK_S = 20.         # terminal HOLD to the harness at the latest this long after the stop trigger
HARNESS_REST_S = 1.                  # harness.run_episode: terminal HOLD + 1.0 s of rest ends the episode
STEP_TIMING_SKIP_S = 2.              # PREREG processing latency excludes the first 2.0 s of SIM time
TIME_CAP_MARGIN_S = 5.
STEP_HIST_EDGES_MS = np.round(10. ** np.linspace(-2., 4., 241), 6)   # 0.01 ms .. 10 s, 40 bins per decade
SAFETY_BAND_M = .6
SENSOR_STOP_WINDOW_AFTER_S = 5.      # SENSOR blackout conditions: stop within [blackout start, end + 5 s]
EXPRESSIONS = {                      # PREREG condition parameters given as text (checked verbatim)
    'yaw_disturbances': '((15 + 20 k, 15.5 + 20 k, 24 * (-1)**k) for k = 0, 1, ...)',
    'occlusions': '((0, T_cap, 75, 105),) + ((10 + 15 k, 13 + 15 k, -150, -30) for k = 0, 1, ...)',
    'obstacle_s': 'max(8.0, 0.5 s_E)',
}
CLASS_CATEGORIES = {                 # acceptable stop categories per PREREG class (field1010 bracket rule added below)
    'NOMINAL': ('ROW_END',), 'ENTRY': ('ROW_END',), 'DRIFT': ('ROW_END',), 'ACTUATION': ('ROW_END',),
    'DISTURBANCE': ('ROW_END',), 'CANOPY_GAP': ('ROW_END',), 'WEAK_STEERING': ('ROW_END', 'CONTROL_AUTHORITY'),
    'SENSOR': ('ROW_END', 'PERCEPTION_OR_LOCALISATION'), 'END_STRUCTURE': ('ROW_END', 'END_STRUCTURE'),
    'OBSTACLE': ('OBSTACLE',),
}
ZERO_TOLERANCE = ('COLLISION', 'CONTACT_MOVING', 'CORRIDOR_DEPARTURE', 'MISSED_ROW_END', 'LEFT_FIELD',
                  'PIVOT_OR_REVERSE', 'AUTO_RESTART', 'SYSTEM_FAULT', 'UNCLASSIFIED')
# Per plant: the frozen single-row executor settings (SIM; the base range is bound by the inner-wheel rule, see the doc).
SINGLE_ROW_PLANT_SETTINGS = {
    'P_fit1003': dict(base_duty_pct=50, reduce_duty_pct=50, single_row_base_min_pct=50, single_row_base_max_pct=60,
                      max_differential_pct=20, duty_cap_pct=80, diff_cap_pct=40, single_row_bias_max_pct=12.),
    'field1010': dict(base_duty_pct=72, reduce_duty_pct=72, single_row_base_min_pct=60, single_row_base_max_pct=75,
                      max_differential_pct=20, duty_cap_pct=95, diff_cap_pct=40, single_row_bias_max_pct=12.),
}


# --- PREREG -------------------------------------------------------------------------------------------------------
def prereg_path():
    """The single-row PREREG.json as configured (sim_paths 'single_row_prereg': UBTS_SINGLE_ROW_PREREG or the path config
    file; there is no built-in path). FileNotFoundError (PathNotConfigured when nothing is configured, or naming the
    source of a value that is not a file)."""
    XP.configured_path('single_row_prereg', 'file')
    return XP.lookup('single_row_prereg')[0]


def load_prereg(path=None):
    path = path or prereg_path()
    raw = open(path, 'rb').read()
    data = json.loads(raw.decode('utf-8'))
    data['_sha256'] = hashlib.sha256(raw).hexdigest()
    data['_path'] = path
    data['_amendments'] = os.path.join(os.path.dirname(path), 'PREREG_AMENDMENTS.json')
    return data


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for block in iter(lambda: fh.read(1 << 16), b''):
            h.update(block)
    return h.hexdigest()


# --- plants --------------------------------------------------------------------------------------------------------
ORIGINAL_FIELD_PLANT = SIL.ORIGINAL_FIELD_PLANT


class EvalPlant(ORIGINAL_FIELD_PLANT):
    """The unchanged FieldPlant plus an evaluation-only handle (cls.last) for the episode-end rule; never read by the
    executor. SidedGainPlant subclasses multiply the applied duties (PREREG sided_gain_extension)."""
    last = None
    gain_left = 1.
    gain_right = 1.

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        type(self).last = self


class SidedGainPlant(EvalPlant):
    """MCU-applied duty of each side x its gain, then the UNCHANGED FieldPlant response (mean -> speed, difference ->
    yaw). PREREG plants.sided_gain_extension; no other change."""

    def _targets(self, left, right, dt):
        return super()._targets(left * self.gain_left, right * self.gain_right, dt)


def plant_class(sided_gain=None):
    """A fresh subclass per episode (its own cls.last): EvalPlant, or SidedGainPlant with the given gains."""
    if sided_gain is None:
        return type('EvalPlant_episode', (EvalPlant,), dict(last=None))
    left, right = float(sided_gain['left']), float(sided_gain['right'])
    if not (0. < left <= 1.5 and 0. < right <= 1.5):
        raise ValueError('sided gain outside (0, 1.5]')
    return type('SidedGainPlant_%g_%g' % (left, right), (SidedGainPlant,), dict(last=None, gain_left=left,
                                                                                gain_right=right))


# --- executor configuration (one per plant) -------------------------------------------------------------------------
def nominal_plant_params(plant):
    return SH.plant_params(EpisodeConfig(plant_overrides=SIL.plant_spec(PLANTS[plant])['overrides']))


def single_row_executor_config(plant, overrides=None):
    """The frozen mode-'single_row' executor config of one plant (validate()d; motor output on for the SIM bridge).

    sim_in_loop.executor_config for the plant profile (SIM 'measured' values from the NOMINAL parameters: breakaway 20,
    yaw authority 2 x nominal arc gain x max differential, max yaw 0.6 x that, C7 per profile) + the single-row settings
    of SINGLE_ROW_PLANT_SETTINGS + v_measured = the nominal forward curve at the START base (C3) + resolved_polyline,
    require_native_navigation false (SENSOR_ONLY_SINGLE_ROW). overrides: development only (recorded in the manifest)."""
    if plant not in PLANTS:
        raise ValueError('unknown plant %r' % (plant,))
    values = dict(SINGLE_ROW_PLANT_SETTINGS[plant], mode=XC.SINGLE_ROW_MODE, require_native_navigation=False,
                  distance_mode='resolved_polyline')
    values.update(overrides or {})
    params = nominal_plant_params(plant)
    authority = 2. * values['max_differential_pct'] * params.arc_yaw_deg_s_per_pct_diff
    values.setdefault('arc_yaw_deg_s_per_pct_diff', params.arc_yaw_deg_s_per_pct_diff)
    values.setdefault('v_measured_m_s', round(forward_curve(values['base_duty_pct'], params), 4))
    values.setdefault('measured_yaw_at_max_diff_deg_s', round(authority, 4))
    values.setdefault('max_yaw_rate_deg_s', XC.max_yaw_for_authority(authority))
    return SIL.executor_config(values, plant_name=PLANTS[plant])


# --- episodes ------------------------------------------------------------------------------------------------------
def corridor_label(entry, direction):
    lines = entry['lines']
    return '%s:L%d-R%d:%s' % (entry['record'], lines['left']['group_id'], lines['right']['group_id'], direction)


def resolve_corridor(catalogue, spec):
    """'<record>:<cNN>:<dir>' or '<record>:L<a>-R<b>:<dir>' -> (catalogue entry, direction, label)."""
    parts = spec.split(':')
    if len(parts) != 3 or parts[2] not in ('fwd', 'rev'):
        raise ValueError('corridor %r: expected <record>:<cNN or La-Rb>:<fwd|rev>' % (spec,))
    record, ident, direction = parts
    for entry in catalogue['corridors']:
        if entry['record'] != record:
            continue
        if entry['corridor_id'] == ident or corridor_label(entry, direction).split(':')[1] == ident:
            return entry, direction, corridor_label(entry, direction)
    raise KeyError('corridor %s not in the catalogue' % spec)


def label_digest(entry, direction):
    """The PREREG label digest of one corridor direction (prereg_select.py, recomputed from the catalogue)."""
    geo, dd = entry['geometry'], entry['directions'][direction]
    label = dict(key=corridor_label(entry, direction).rsplit(':', 1)[0], direction=direction,
                 start_s_fwd_m=dd['start_s_fwd_m'], end_s_fwd_m=dd['end_s_fwd_m'], length_m=dd['length_m'],
                 s_origin_fwd_m=geo['s_origin_fwd_m'],
                 polyline_fwd=[[round(float(x), 3), round(float(y), 3)] for x, y in geo['polyline_fwd']])
    return hashlib.sha256(json.dumps(label, sort_keys=True, separators=(',', ':')).encode('utf-8')).hexdigest()


def yaw_bursts(t_cap):
    out, k = [], 0
    while 15. + 20. * k < t_cap:
        out.append((15. + 20. * k, 15.5 + 20. * k, 24. * (-1) ** k))
        k += 1
    return tuple(out)


def occlusion_sectors(t_cap):
    out, k = [(0., t_cap, 75., 105.)], 0
    while 10. + 15. * k < t_cap:
        out.append((10. + 15. * k, 13. + 15. * k, -150., -30.))
        k += 1
    return tuple(out)


def condition_modifiers(params, s_end):
    mods = []
    if 'gap' in params:
        g = params['gap']
        mods += RC.prereg_gap(tuple(g['sides']), s_end, float(g['length_m']), float(g['centre_fraction']))
    if 'branches' in params:
        mods += RC.prereg_branches(s_end, **params['branches'])
    if 'wall' in params:
        mods += RC.prereg_wall(float(params['wall']['distance_m']))
    if 'obstacle' in params:
        o = params['obstacle']
        if o.get('s') != EXPRESSIONS['obstacle_s']:
            raise ValueError('obstacle s expression changed: %r' % (o.get('s'),))
        mods += RC.prereg_obstacle(s_end, tuple(o['size_m']), float(o.get('d', 0.)))
    return tuple(mods)


KNOWN_PARAMS = ('start_d_m', 'start_heading_deg', 'plant_overrides', 'sided_gain', 'command_latency_s',
                'lidar_latency_s', 'lidar_latency_jitter_s', 'imu_latency_s', 'scan_drop_probability', 'blackouts',
                'occlusions', 'yaw_disturbances', 'gap', 'branches', 'wall', 'obstacle')


def build_episode(prereg, catalogue, plant, condition, corridor_spec, seed):
    """(EpisodeConfig, meta) of one PREREG episode. Condition parameters come from PREREG.json itself."""
    cond = prereg['conditions'].get(condition)
    if cond is None:
        raise KeyError('condition %r not in the PREREG' % (condition,))
    if plant not in cond['plants']:
        raise ValueError('condition %s is not registered for plant %s' % (condition, plant))
    params = dict(cond['params'])
    unknown = sorted(set(params) - set(KNOWN_PARAMS))
    if unknown:
        raise ValueError('condition %s: unknown parameters %s' % (condition, unknown))
    for key in ('yaw_disturbances', 'occlusions'):
        if key in params and params[key] != EXPRESSIONS[key]:
            raise ValueError('condition %s: %s expression changed: %r' % (condition, key, params[key]))
    entry, direction, label = resolve_corridor(catalogue, corridor_spec)
    probe = RC.corridor_world_config(entry['record'], entry['corridor_id'], direction, catalogue=catalogue)
    s_end = probe.end_s_m
    t_cap = RC.time_cap_s(s_end, START_S_M)
    world = RC.corridor_world_config(entry['record'], entry['corridor_id'], direction, catalogue=catalogue,
                                     modifiers=condition_modifiers(params, s_end))
    plant_pairs = dict(SIL.plant_spec(PLANTS[plant])['overrides'])
    plant_pairs.update(params.get('plant_overrides', {}))                 # the condition wins for the names it sets
    defaults = prereg['condition_defaults']
    ep = EpisodeConfig(
        name=condition, world=world, mode='continuous', seed=int(seed), stream_key='%s|%s' % (label, condition),
        corridor_index=0, start_x_m=START_S_M, start_y_m=float(params.get('start_d_m', defaults['start_d_m'])),
        start_yaw_deg=float(params.get('start_heading_deg', defaults['start_heading_deg'])),
        plant_overrides=tuple(sorted(plant_pairs.items())),
        command_latency_s=float(params.get('command_latency_s', defaults['command_latency_s'])),
        lidar_latency_s=float(params.get('lidar_latency_s', defaults['lidar_latency_s'])),
        lidar_latency_jitter_s=float(params.get('lidar_latency_jitter_s', defaults['lidar_latency_jitter_s'])),
        imu_latency_s=float(params.get('imu_latency_s', defaults['imu_latency_s'])),
        scan_drop_probability=float(params.get('scan_drop_probability', defaults['scan_drop_probability'])),
        blackouts=tuple(tuple(b) for b in params.get('blackouts', ())),
        occlusions=occlusion_sectors(t_cap) if 'occlusions' in params else (),
        yaw_disturbances=yaw_bursts(t_cap) if 'yaw_disturbances' in params else (),
        max_time_s=t_cap + TIME_CAP_MARGIN_S)
    seeds = prereg['seeds']
    seed_set = 'DEV' if seed in seeds['DEV'] else ('HOLDOUT' if seed in seeds['HOLDOUT'] else 'OTHER')
    meta = dict(plant=plant, condition=condition, cls=cond['cls'], corridor=label, record=entry['record'],
                catalogue_id=entry['corridor_id'], direction=direction, seed=int(seed), seed_set=seed_set,
                stream_key=ep.stream_key, s_E_m=round(s_end, 3), time_cap_s=round(t_cap, 2),
                label_digest=label_digest(entry, direction), sided_gain=params.get('sided_gain'),
                blackouts=[list(b) for b in ep.blackouts],
                wall_distance_m=None if 'wall' not in params else float(params['wall']['distance_m']))
    return ep, meta


def episode_id(meta):
    return '%s__%s__%s_%s_%s__s%d' % (meta['plant'], meta['condition'], meta['record'], meta['catalogue_id'],
                                      meta['direction'], meta['seed'])


# --- the facade ----------------------------------------------------------------------------------------------------
class SingleRowInTheLoop(SIL.ExecutorInTheLoop):
    """sim_in_loop's REAL-executor facade with the single-row operator policy (module docstring) and the PREREG episode
    end. Truth enters only the episode-end signal to the harness (rest of the plant after the stop trigger)."""

    def __init__(self, episode, cfg, row_len_m, real_format, rover_control, plant_cls, press_policy='prereg_fixed'):
        if press_policy not in ('prereg_fixed', 'page_ready'):
            raise ValueError('press_policy')
        super().__init__(episode, cfg, 0. if row_len_m is None else row_len_m, real_format, rover_control)
        self.row_len_m = None if row_len_m is None else float(row_len_m)
        self.press_policy = press_policy
        self.plant_cls = plant_cls
        self.press_t = None
        self.stop = None                 # first executor PAUSE (other than await_hold) / END after the press
        self.rest_since = None
        self.terminal_signal_t = None
        self.step_ms = []                # wall ms of ExecutorLoop.step for SIM t >= STEP_TIMING_SKIP_S
        self.tick_log = []               # executor-side values per tick (traces / video; no truth)
        step = self.loop.step

        def timed_step(now=None):
            started = time.perf_counter()
            decision = step(now)
            if now is not None and now >= STEP_TIMING_SKIP_S:
                self.step_ms.append((time.perf_counter() - started) * 1e3)
            return decision
        self.loop.step = timed_step

    def snapshot(self, now):
        s = super().snapshot(now)
        runner = self.loop.runner
        decision = self.loop._last_decision
        sr = (decision.summary.get('single_row') if decision is not None else None) or {}
        self.tick_log.append(dict(
            t=round(now, 2), rs=runner.state, rr=runner.reason, ph=sr.get('phase'),
            L=None if decision is None else decision.left, R=None if decision is None else decision.right,
            lat=sr.get('lateral_error_m'), hdg=sr.get('heading_error_deg'), lref=sr.get('lateral_reference_m'),
            conf=(sr.get('confidence') or {}).get('level'), diff=sr.get('differential_pct'), sat=sr.get('saturated'),
            auth=sr.get('authority_state'), bias=sr.get('bias_pct'), base=sr.get('base_duty_pct'),
            vt=sr.get('target_speed_m_s'), v=sr.get('speed_m_s'), dist=sr.get('distance_m'),
            end=sr.get('row_end_ahead_m'), rem=sr.get('end_remaining_m'), cat=sr.get('stop_category'),
            width=_round(s.get('corridor_width')) if s.get('corridor_valid') else None,
            sides=[_round(v) for v in s.get('side_ends_ahead_m') or ()] if s.get('corridor_valid') else None))
        return s

    def _operator(self, now):
        loop, runner = self.loop, self.loop.runner
        if self.phase == 'start':
            self.future = loop.submit('start', row_len_m=self.row_len_m)
            self.phase = 'started'
        elif self.phase == 'started' and self.future.done():
            result = self.future.result()
            if not result.get('ok'):
                self.start_error, self.phase = result, 'failed'
            else:
                self.future, self.phase = loop.submit('arm'), 'arming'
        elif self.phase == 'arming' and self.future.done():
            result = self.future.result()
            if not result.get('ok'):
                self.start_error, self.phase = result, 'failed'
            else:
                self.phase = 'armed'
        elif self.phase == 'armed' and now >= PRESS_AT_S - 1e-9:
            if self.press_policy == 'prereg_fixed' or self._resume_ready(now):
                self._new_press(now)
                self.press_t = now
        elif self.phase == 'holding':
            if self.stop is None and (runner.state == XCORE.ENDED or (
                    runner.state == XCORE.PAUSED and runner.reason != 'await_hold')):
                self.stop = dict(t=round(now, 3), state=runner.state,
                                 reason=runner.end_reason if runner.state == XCORE.ENDED else runner.reason)
                if runner.reason.startswith('press_ignored'):
                    self.press_ignored += 1
            loop.hold_beat(self.press)               # heartbeat kept, never released, no re-press

    def _facade_state(self, now):
        runner = self.loop.runner
        term = False
        if self.stop is not None:
            # evaluation side only: the TRUE body rate includes a scenario's external yaw (a burst at rest is not rest)
            plant = self.plant_cls.last if self.plant_cls is not None else None
            at_rest = (plant is not None and abs(plant.v) < REST_V_M_S
                       and abs(plant.w + plant.external_yaw_rate) < REST_W_RAD_S)
            self.rest_since = (self.rest_since if self.rest_since is not None else now) if at_rest else None
            # the harness then needs 1.0 s more of rest: no scenario yaw burst may start inside that last second
            quiet = not any(t0 <= now + HARNESS_REST_S + .2 and t1 >= now for t0, t1, _ in self.ep.yaw_disturbances)
            if self.terminal_signal_t is None and (
                    (self.rest_since is not None and now - self.rest_since >= REST_THEN_OBSERVE_S - 1e-9 and quiet)
                    or now - self.stop['t'] >= STOP_SIGNAL_FALLBACK_S):
                self.terminal_signal_t = round(now, 3)
            if self.terminal_signal_t is not None:
                state, reason, term = 'HOLD', self.stop['reason'], True
            else:
                state, reason = ('STOP_OBSERVATION', self.stop['reason'])
        elif runner.state == XCORE.ENDED:
            state, reason = 'ENDED', runner.end_reason
        elif runner.state == XCORE.PAUSED:
            state, reason = 'PAUSED', runner.reason
        elif runner.state == XCORE.ACTIVE and runner.controller is not None:
            ctl = runner.controller
            if not any(c is ctl for c in self._controllers):
                self._controllers.append(ctl)
            state, reason = ctl.state, ctl.reason
        else:
            state, reason = 'REACQUIRE', 'start'
        if (state, reason) != (self.state, self.reason):
            self.events.append((round(now, 3), state, reason))
        self.state, self.reason, self.hold_terminal = state, reason, term


# --- one episode ---------------------------------------------------------------------------------------------------
_PROCESS_CACHE = {}


def _cached(key, factory):
    if key not in _PROCESS_CACHE:
        _PROCESS_CACHE[key] = factory()
    return _PROCESS_CACHE[key]


def _round(value, digits=3):
    return round(float(value), digits) if isinstance(value, (int, float)) and math.isfinite(value) else None


def _percentiles(values, digits=3):
    if not values:
        return dict(n=0, p50=None, p99=None, max=None)
    a = np.asarray(values, dtype=float)
    return dict(n=int(a.size), p50=round(float(np.percentile(a, 50)), digits),
                p99=round(float(np.percentile(a, 99)), digits), max=round(float(a.max()), digits))


def _histogram(values):
    counts, _ = np.histogram(np.clip(np.asarray(values, dtype=float), STEP_HIST_EDGES_MS[0], STEP_HIST_EDGES_MS[-1]),
                             bins=STEP_HIST_EDGES_MS)
    return {str(i): int(c) for i, c in enumerate(counts) if c}


def run_job(job):
    """One episode -> EPISODES.jsonl row (never raises: an exception is a SYSTEM_FAULT row)."""
    started = time.perf_counter()
    options = job['options']
    row = dict(schema=SCHEMA_ROW, run_id=options['run_id'], episode_id=job['episode_id'], **job['meta'])
    try:
        row.update(_run_episode(job, options))
    except Exception as exc:                                  # PREREG EXCEPTION = SYSTEM_FAULT
        row.update(end_kind='SYSTEM_FAULT', verdict='FAIL', fail_codes=['SYSTEM_FAULT'], error=repr(exc)[:500],
                   trace_tail=traceback.format_exc()[-3000:])
    finally:
        SH.FieldPlant = ORIGINAL_FIELD_PLANT
    row['wall_s'] = round(time.perf_counter() - started, 2)
    return row


def _run_episode(job, options):
    meta = job['meta']
    prereg = _cached(('prereg', options['prereg_path']), lambda: load_prereg(options['prereg_path']))
    catalogue = _cached(('catalogue', options['catalogue_path']), lambda: RC.load_catalogue(options['catalogue_path']))
    plant = meta['plant']
    cfg = _cached(('cfg', plant, json.dumps(options['config_overrides'].get(plant, {}), sort_keys=True)),
                  lambda: single_row_executor_config(plant, options['config_overrides'].get(plant)))
    ep, meta2 = build_episode(prereg, catalogue, plant, meta['condition'], meta['corridor_spec'], meta['seed'])
    if options.get('max_time_s') is not None:                 # development / tests only (refused in a final run)
        ep = replace(ep, max_time_s=min(ep.max_time_s, float(options['max_time_s'])))
    row_len = (meta2['s_E_m'] - START_S_M + L_OP_EXTRA_M) if options['row_length'] == 'l_op' else None
    plant_cls = plant_class(meta2['sided_gain'])
    rover = _cached(('rover',), SIL.rover_control_class)
    SH.FieldPlant = plant_cls
    hooks = RC.RasterHooks()
    facade = SingleRowInTheLoop(ep, cfg, row_len, SIL.RealFormat(cfg.lidar_yaw_offset_deg), rover, plant_cls,
                                press_policy=options['press_policy'])
    try:
        result = SH.run_episode(ep, record_trace=True, estimator_factory=lambda _c: facade,
                                controller_factory=lambda _c: facade, **hooks.kwargs())
    finally:
        facade.loop.shutdown('sim_episode_end')
    world = hooks.world
    params = SH.plant_params(ep)
    evaluation = RC.evaluate_episode(result, world, footprint_length_m=params.footprint_length_m,
                                     footprint_width_m=params.footprint_width_m, s_start=ep.start_x_m,
                                     duty_caps=(cfg.diff_cap_pct, cfg.duty_cap_pct))
    executor = facade.summary()
    runner = facade.loop.runner
    described = world.describe()
    out = dict(
        executor_config_digest=cfg.digest(), localisation_path=XC.SINGLE_ROW_LOCALISATION,
        row_length_mode=options['row_length'], row_len_m=None if row_len is None else round(row_len, 3),
        press_policy=options['press_policy'], press_t_s=facade.press_t, max_time_s=round(ep.max_time_s, 2),
        stop_trigger=facade.stop, stop_trigger_time_s=None if facade.stop is None else facade.stop['t'],
        terminal_signal_t_s=facade.terminal_signal_t,
        end_kind=evaluation['prereg_end'], harness_end=result['end'], executor_reason=result['final_reason'],
        executor_end_reason=runner.end_reason, executor_end_note=runner.end_note,
        stop_category=evaluation['stop_category'], stop_table_label=evaluation['stop_table_label'],
        e_stop_m=evaluation['stop_position_e_m'], row_end_outcome=evaluation['row_end_outcome'],
        overshoot_m=evaluation['overshoot_m'], s_base_at_rest_m=evaluation['progress']['s_rest_m'],
        progress_m=evaluation['progress']['progress_m'], progress_fraction=evaluation['progress']['fraction_of_row'],
        time_s=result['time_s'], mean_speed_m_s=evaluation['report_only']['mean_speed_m_s'],
        lateral_rmse_m=evaluation['lateral_m']['rmse'], lateral_p95_m=evaluation['lateral_m']['p95'],
        lateral_max_m=evaluation['lateral_m']['max'], safety_band_max_m=evaluation['safety_band']['max_abs_d_m'],
        heading_rmse_deg=evaluation['heading_deg']['rmse'], heading_max_deg=evaluation['heading_deg']['max'],
        collisions=evaluation['contacts']['collision_samples'],
        contacts_moving=evaluation['contacts']['moving_contact_samples'],
        contacts_rest=evaluation['contacts']['rest_contact_samples'],
        unknown_overlap_samples=evaluation['contacts']['unknown_overlap_total'],
        min_hard_clearance_m=evaluation['contacts']['min_hard_clearance_m'],
        min_branch_clearance_m=evaluation['contacts']['min_branch_clearance_m'],
        obstacle_clearance_m=evaluation['obstacle_wall_clearance_at_rest_m'] if meta2['cls'] == 'OBSTACLE' else None,
        wall_clearance_m=evaluation['obstacle_wall_clearance_at_rest_m'] if meta2['cls'] == 'END_STRUCTURE' else None,
        sat_fraction=(evaluation['pwm_saturation'] or {}).get('saturated_fraction'),
        sat_longest_s=(evaluation['pwm_saturation'] or {}).get('longest_saturated_run_s'),
        authority_margin_low=(evaluation['pwm_saturation'] or {}).get('authority_margin_low'),
        step_ms=_percentiles(facade.step_ms), step_ms_hist=_histogram(facade.step_ms),
        scan_ms=executor['scan_proc_ms'], pre_stop_speed_ratio=evaluation['report_only']['pre_stop_speed_ratio'],
        forbidden=evaluation['forbidden'], row_end_first_estimate=result['row_end_first_estimate'],
        row_end_last_estimate=result['row_end_last_estimate'],
        events=[list(e) for e in facade.events[-24:]],
        single_row_final=_final_telemetry(facade),
        executor=dict((k, executor[k]) for k in ('end_reason', 'end_note', 'pauses', 'resumes', 'presses',
                                                 'press_ignored', 'scans_processed', 'scans_dropped', 'sends',
                                                 'bridge_errors', 'bridge_state', 'deadman_false', 'guard_violations',
                                                 'max_abs_diff_sent', 'first_drive_t', 'start_error', 'backstop')),
        raster_world=dict((k, described[k]) for k in ('pgm_sha256', 'yaml_sha256', 'catalogue_sha256', 'end_s_m',
                                                      'walls_s_m', 'modifiers', 'lidar', 'contact_samples')),
        evaluation=dict((k, evaluation[k]) for k in ('prereg_end', 'end_t_s', 'safety_band', 'contacts',
                                                     'pwm_saturation', 'forbidden', 'report_only', 'progress')))
    full = dict(meta2, **out)
    full.update(prereg_verdict(prereg, full))
    if options['trace_dir']:
        _write_trace(options['trace_dir'], job['episode_id'], result, facade, evaluation, full)
    if options['video'] and (options['video'] == 'all' or job['episode_id'] in options['video']):
        full['video'] = _render(options['video_dir'], job['episode_id'], result, world, facade, evaluation, full,
                                options['run_kind'])
    full.pop('sided_gain', None)
    return full


def _final_telemetry(facade):
    decision = facade.loop._last_decision
    sr = (decision.summary.get('single_row') if decision is not None else None) or {}
    keys = ('phase', 'bias_pct', 'base_duty_pct', 'authority_state', 'saturation_fraction', 'stop_reason',
            'stop_category', 'first_stop', 'phase_events', 'distance_m')
    return {k: sr.get(k) for k in keys}


def _write_trace(trace_dir, eid, result, facade, evaluation, row):
    os.makedirs(trace_dir, exist_ok=True)
    payload = dict(schema='ubts.single_row.sim_trace.v1', episode_id=eid, frame='corridor (s along the labelled '
                   'centreline, d left of travel; truth, evaluation only) + executor estimates (no raster / GNSS '
                   'coordinates)', harness_trace=result['trace'], executor_ticks=facade.tick_log,
                   step_ms=[round(v, 4) for v in facade.step_ms], evaluation=evaluation,
                   verdict=row.get('verdict'), fail_codes=row.get('fail_codes'))
    path = os.path.join(trace_dir, eid + '.json.gz')
    with gzip.open(path, 'wt', encoding='utf-8', newline='\n') as fh:
        json.dump(payload, fh, separators=(',', ':'), default=str)


def _render(video_dir, eid, result, world, facade, evaluation, row, run_kind):
    from row_executor import single_row_video as V
    snapshots = [dict(t=r['t'], corridor_width=r['width'], row_end_ahead_m=r['end'], side_ends_ahead_m=r['sides'])
                 for r in facade.tick_log]
    title = 'SIM %s: executor single_row, %s %s %s, %s seed %d -> %s / %s (raster SLAM map, not physical truth)' % (
        run_kind.upper(), row['plant'], row['corridor'], row['condition'], row['seed_set'], row['seed'],
        row.get('end_kind'), row.get('stop_category'))
    path = os.path.join(video_dir, eid + '.mp4')
    try:
        return V.render_video(result, world, path, snapshots=snapshots, evaluation=evaluation, title=title)
    except Exception as exc:                                  # a video never changes the episode result
        return dict(error=repr(exc)[:300])


# --- verdict (PREREG classes, all-class rules, fail codes) ------------------------------------------------------------
def prereg_verdict(prereg, row):
    """PREREG per-episode verdict: dict(verdict 'PASS'|'FAIL', fail_codes, notes, completed)."""
    cls_name, plant = row['cls'], row['plant']
    cls = prereg['classes'][cls_name]
    limits = cls['limits']
    codes, notes = [], []
    end, cat, e = row['end_kind'], row['stop_category'], row['e_stop_m']
    terminal = end == 'TERMINAL_STOP'
    for kind, code in (('COLLISION', 'COLLISION'), ('CORRIDOR_DEPARTURE', 'CORRIDOR_DEPARTURE'),
                       ('MISSED_ROW_END', 'MISSED_ROW_END'), ('LEFT_FIELD', 'LEFT_FIELD'),
                       ('NO_PROGRESS', 'NO_PROGRESS'), ('TIME_CAP', 'TIME_CAP')):
        if end == kind:
            codes.append(code)
    if end == 'OPEN':
        codes.append('OPEN_END')              # runner-level code: no terminal stop and no PREREG end (always FAIL)
    if row['collisions'] and 'COLLISION' not in codes:
        codes.append('COLLISION')
    if row['contacts_moving']:
        codes.append('CONTACT_MOVING')
    if row['forbidden'].get('pivot_or_reverse'):
        codes.append('PIVOT_OR_REVERSE')
    if row['forbidden'].get('auto_restart'):
        codes.append('AUTO_RESTART')
    if row['evaluation']['safety_band'].get('exceeded'):
        codes.append('SAFETY_BAND')
    acceptable = CLASS_CATEGORIES[cls_name] + (('CONTROL_AUTHORITY',) if plant == 'field1010'
                                               and cls_name != 'NOMINAL' else ())
    completed = bool(terminal and cat == 'ROW_END' and e is not None and -1. <= e <= 1.)
    limits_apply = completed
    if terminal:
        if cat in ('OPERATOR_OR_BACKSTOP', 'SYSTEM_FAULT'):
            codes.append('CATEGORY_NOT_ACCEPTABLE')
        elif cat == 'UNCLASSIFIED':
            codes.append('UNCLASSIFIED')
        elif cat not in acceptable:
            codes.append('CATEGORY_NOT_ACCEPTABLE')
        if cat == 'ROW_END' and e is not None and not completed:
            codes.append('ROW_END_FALSE' if e < -2. else 'ROW_END_EARLY' if e < -1. else 'ROW_END_LATE')
        if cls_name == 'SENSOR' and cat == 'PERCEPTION_OR_LOCALISATION' and row['blackouts']:
            t0 = min(b[0] for b in row['blackouts'])
            t1 = max(b[1] for b in row['blackouts'])
            t_stop = row['stop_trigger_time_s']
            if t_stop is None or not t0 <= t_stop <= t1 + SENSOR_STOP_WINDOW_AFTER_S:
                codes.append('PERCEPTION_STOP_WINDOW')
        if cls_name == 'END_STRUCTURE':
            D = row['wall_distance_m']
            if cat == 'END_STRUCTURE':
                limits_apply = True
                if e is None or not -1. <= e <= D - .30:
                    codes.append('END_STRUCTURE_POSITION')
            clearance = row['wall_clearance_m']
            if clearance is None:
                # evaluate_episode measures it with a 6 m reach (raster_corridor.AT_REST_REACH_M): None = no wall cell
                # within 6 m of the footprint at rest, i.e. clearance > 6 m >= 0.30 m (dev round 1 evaluator fix: the
                # 2 m contact reach had turned every correct row-end stop 3 m before the wall into a fail)
                notes.append('WALL_BEYOND_%g_M' % RC.AT_REST_REACH_M)
            elif clearance < .30:
                codes.append('WALL_CLEARANCE')
            notes.append('ORDER_UNVERIFIED (the executor reports no row_end_evidence_time_s / front_structure_time_s)')
        if cls_name == 'OBSTACLE' and cat == 'OBSTACLE':
            limits_apply = True
            clearance = row['obstacle_clearance_m']
            if clearance is None or not .30 <= clearance <= 3.:
                codes.append('OBSTACLE_CLEARANCE')
    if limits_apply:
        for value, limit, code in ((row['lateral_rmse_m'], limits['lateral_rmse_max_m'], 'LATERAL_RMSE'),
                                   (row['lateral_p95_m'], limits['lateral_abs_p95_max_m'], 'LATERAL_P95'),
                                   (row['lateral_max_m'], limits['lateral_abs_max_m'], 'LATERAL_MAX'),
                                   (row['heading_rmse_deg'], limits['heading_rmse_max_deg'], 'HEADING_RMSE'),
                                   (row['heading_max_deg'], limits['heading_abs_max_deg'], 'HEADING_MAX')):
            if value is None or value > limit + 1e-9:
                codes.append(code)
    codes = list(dict.fromkeys(codes))
    return dict(verdict='FAIL' if codes else 'PASS', fail_codes=codes, verdict_notes=notes, completed=completed,
                acceptable_categories=list(acceptable))


# --- summary -------------------------------------------------------------------------------------------------------
def _stats(values):
    vals = [v for v in values if v is not None and isinstance(v, (int, float)) and math.isfinite(v)]
    if not vals:
        return dict(n=0, median=None, p95=None, max=None)
    a = np.asarray(vals, dtype=float)
    return dict(n=int(a.size), median=round(float(np.median(a)), 4), p95=round(float(np.percentile(a, 95)), 4),
                max=round(float(a.max()), 4))


def _pooled_percentile(rows, q):
    counts = np.zeros(len(STEP_HIST_EDGES_MS) - 1, dtype=np.int64)
    for r in rows:
        for k, c in (r.get('step_ms_hist') or {}).items():
            counts[int(k)] += c
    total = int(counts.sum())
    if not total:
        return None
    index = int(np.searchsorted(np.cumsum(counts), q / 100. * total))
    return round(float(STEP_HIST_EDGES_MS[min(index + 1, len(STEP_HIST_EDGES_MS) - 1)]), 3)   # upper bin edge


def _group(rows):
    out = dict(n=len(rows), passed=sum(r.get('verdict') == 'PASS' for r in rows))
    out['rate'] = round(out['passed'] / out['n'], 4) if out['n'] else None
    out['stop_categories'] = _hist(r.get('stop_category') for r in rows)
    out['row_end_outcomes'] = _hist(r.get('row_end_outcome') for r in rows)
    out['end_kinds'] = _hist(r.get('end_kind') for r in rows)
    out['fail_codes'] = _hist(code for r in rows for code in r.get('fail_codes', ()))
    for key in ('lateral_rmse_m', 'lateral_p95_m', 'lateral_max_m', 'heading_rmse_deg', 'heading_max_deg', 'e_stop_m',
                'overshoot_m', 'mean_speed_m_s', 'sat_fraction'):
        out[key] = _stats(r.get(key) for r in rows)
    return out


def _hist(items):
    out = {}
    for item in items:
        out[str(item)] = out.get(str(item), 0) + 1
    return dict(sorted(out.items()))


def summarise(rows, prereg, run_kind, expected_rows=None):
    by = {}
    for r in rows:
        by.setdefault((r['plant'], r['cls'], r['condition'], r['seed_set']), []).append(r)
    groups = [dict(plant=k[0], cls=k[1], condition=k[2], seed_set=k[3], **_group(v)) for k, v in sorted(by.items())]
    plants = sorted({r['plant'] for r in rows})
    rules = {}
    for plant in plants:
        mine = [r for r in rows if r['plant'] == plant]
        nominal = [r for r in mine if r['cls'] == 'NOMINAL']
        other = [r for r in mine if r['cls'] != 'NOMINAL']
        conditions = {}
        for r in mine:
            conditions.setdefault(r['condition'], []).append(r)
        rules[plant] = dict(
            R1_nominal_pass=dict(n=len(nominal), passed=sum(r['verdict'] == 'PASS' for r in nominal),
                                 ok=bool(nominal) and all(r['verdict'] == 'PASS' for r in nominal)),
            R2_non_nominal_pass=dict(n=len(other), passed=sum(r['verdict'] == 'PASS' for r in other),
                                     ok=bool(other) and sum(r['verdict'] == 'PASS' for r in other) >= .95 * len(other)),
            R3_per_condition={c: dict(n=len(v), passed=sum(r['verdict'] == 'PASS' for r in v),
                                      ok=sum(r['verdict'] == 'PASS' for r in v) >= .75 * len(v))
                              for c, v in sorted(conditions.items())})
    zero = {code: [r['episode_id'] for r in rows if code in r.get('fail_codes', ())] for code in ZERO_TOLERANCE}
    false_ends = [r['episode_id'] for r in rows if 'ROW_END_FALSE' in r.get('fail_codes', ())]
    sensor = [r for r in rows if r['plant'] == 'P_fit1003' and r['cls'] == 'SENSOR']
    p99, p999 = _pooled_percentile(rows, 99), _pooled_percentile(rows, 99.9)
    r = dict(
        R1=all(rules[p]['R1_nominal_pass']['ok'] for p in plants) if plants else None,
        R2=all(rules[p]['R2_non_nominal_pass']['ok'] for p in plants) if plants else None,
        R3=all(all(v['ok'] for v in rules[p]['R3_per_condition'].values()) for p in plants) if plants else None,
        R4=dict(events={k: v for k, v in zero.items() if v}, ok=not any(zero.values())),
        R5=dict(false_row_end=len(false_ends), limit=.01 * len(rows), ok=len(false_ends) <= .01 * len(rows)),
        R6=dict(n=len(sensor), completed=sum(bool(x.get('completed')) for x in sensor),
                ok=bool(sensor) and sum(bool(x.get('completed')) for x in sensor) >= .5 * len(sensor)),
        R7=dict(step_ms_p99=p99, step_ms_p999=p999, ok=p99 is not None and p99 <= 50. and p999 <= 200.),
        R8=dict(rows=len(rows), expected=expected_rows, ok=expected_rows is not None and len(rows) == expected_rows))
    verdict_valid = run_kind == 'final'
    overall = (all([r['R1'], r['R2'], r['R3'], r['R4']['ok'], r['R5']['ok'], r['R6']['ok'], r['R7']['ok'],
                    r['R8']['ok']]) if verdict_valid else None)
    return dict(schema=SCHEMA_SUMMARY, run_kind=run_kind, rows=len(rows), prereg_sha256=prereg['_sha256'],
                verdict_is_prereg_verdict=verdict_valid,
                overall='PASS' if overall else ('FAIL' if overall is False else 'NOT_A_VERDICT (%s run)' % run_kind),
                groups=groups, per_plant_rules=rules, rules=r,
                timing_pooled=dict(step_ms_p50=_pooled_percentile(rows, 50), step_ms_p99=p99, step_ms_p999=p999,
                                   step_ms_max=max((x['step_ms']['max'] for x in rows if x.get('step_ms', {}).get('max')
                                                    is not None), default=None),
                                   note='pooled from per-episode histograms (upper bin edge), this PC, parallel '
                                        'workers; not a Jetson timing'),
                failed=[dict(episode_id=x['episode_id'], fail_codes=x.get('fail_codes'), stop=x.get('executor_reason'),
                             e_stop_m=x.get('e_stop_m')) for x in rows if x.get('verdict') != 'PASS'])


def report_ko(summary, manifest, rows):
    lines = ['단일열 SIM 실행 보고 (%s)' % manifest['run_id'], '=' * 60, '']
    kind = manifest['run_kind']
    if kind != 'final':
        lines += ['* 이 실행은 %s 실행이다. PREREG 최종 판정이 아니다(평가 행렬·holdout·FREEZE 없음).' % kind, '']
    lines += ['실행 경로: 실제 실행기(ExecutorLoop, mode single_row, SENSOR_ONLY_SINGLE_ROW) + 실제 대시보드 RoverControl',
              '(가짜 모터) + 10/10 raster 회랑(SLAM 재구성 지도, 물리적 참값 아님). GPS·카메라·PWM 오도메트리 없음.',
              '열 길이 입력: %s, 누름 정책: %s, 워커 %d개, 플랫폼 %s.' % (
                  manifest['row_length'], manifest['press_policy'], manifest['workers'], manifest['platform']['machine']),
              '', '에피소드:']
    for r in rows:
        lines.append('  %s  %s  끝=%s 정지분류=%s e=%s m  횡 RMSE/p95/최대 %s/%s/%s m  heading RMSE/최대 %s/%s 도  '
                     '판정 %s %s  (%.0f s SIM, wall %.0f s)' % (
                         r['episode_id'], r.get('executor_reason'), r.get('end_kind'), r.get('stop_category'),
                         r.get('e_stop_m'), r.get('lateral_rmse_m'), r.get('lateral_p95_m'), r.get('lateral_max_m'),
                         r.get('heading_rmse_deg'), r.get('heading_max_deg'), r.get('verdict'),
                         ','.join(r.get('fail_codes') or []), r.get('time_s') or 0, r.get('wall_s') or 0))
    t = summary['timing_pooled']
    lines += ['', 'ExecutorLoop.step 처리 시간(이 PC, 처음 2 s 제외): p50 %s ms, p99 %s ms, p99.9 %s ms, 최대 %s ms' % (
        t['step_ms_p50'], t['step_ms_p99'], t['step_ms_p999'], t['step_ms_max']),
        '종합: %s' % summary['overall'], '',
        '한계: SIM 전용. 실차 조향 권한·저 duty 응답·정지 거리는 측정값이 아니다. 지도는 SLAM 재구성이다.',
        'P_fit1003에서는 실행기 inner-wheel 규칙(breakaway 20 + margin 10)이 base 하한을 50 %로 묶어 0.20 m/s 감속 목표에',
        '도달할 수 없다(약 0.42 m/s 이상). 장착 보정·모터·Jetson 작업은 없다.']
    return '\n'.join(lines) + '\n'


# --- run -----------------------------------------------------------------------------------------------------------
def _git(*args):
    try:
        return subprocess.run(('git',) + args, capture_output=True, text=True, cwd=REPO, timeout=20).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def hold_present():
    return SIL.hold_present()


def parse_episode(spec):
    """'<plant>:<condition>:<record>:<corridor>:<dir>:<seed>'."""
    parts = spec.split(':')
    if len(parts) != 6:
        raise ValueError('episode %r: expected plant:condition:record:corridor:dir:seed' % (spec,))
    return dict(plant=parts[0], condition=parts[1], corridor_spec=':'.join(parts[2:5]), seed=int(parts[5]))


def load_selection(path, catalogue):
    with open(path, 'r', encoding='utf-8') as fh:
        data = json.load(fh)
    out = []
    for item in data['selected']:
        entry, direction, label = resolve_corridor(catalogue, item['corridor'])
        digest = label_digest(entry, direction)
        if item.get('label_digest') and item['label_digest'] != digest:
            raise ValueError('label digest of %s differs from the catalogue (%s != %s)' % (label, item['label_digest'],
                                                                                          digest))
        out.append(label)
    return out, hashlib.sha256(open(path, 'rb').read()).hexdigest()


def plan_jobs(args, prereg, catalogue):
    seeds = prereg['seeds']
    if args.prereg:
        if not args.selection:
            raise SystemExit('--prereg needs --selection (the frozen corridor selection)')
        corridors, _ = load_selection(args.selection, catalogue)
        if args.seed_set == 'dev':
            seed_list = list(seeds['DEV'])
        elif args.seed_set == 'holdout':
            seed_list = list(seeds['HOLDOUT'])
        else:
            seed_list = list(seeds['DEV']) + list(seeds['HOLDOUT'])
        plants = args.plants or list(PLANTS)
        specs = []
        for plant in plants:
            for condition, cond in prereg['conditions'].items():
                if plant not in cond['plants'] or (args.conditions and condition not in args.conditions):
                    continue
                for corridor in corridors:
                    for seed in seed_list:
                        specs.append(dict(plant=plant, condition=condition, corridor_spec=corridor, seed=seed))
    else:
        specs = [parse_episode(s) for s in (args.episode or [])]
        if args.corridors:
            for plant in args.plants or ['P_fit1003']:
                for condition in args.conditions or ['nominal']:
                    for corridor in args.corridors:
                        for seed in args.seeds or [0]:
                            specs.append(dict(plant=plant, condition=condition, corridor_spec=corridor, seed=seed))
    if any(s['seed'] in seeds['HOLDOUT'] for s in specs):
        if not (args.freeze and os.path.isfile(args.freeze)):
            raise SystemExit('holdout seeds %s need --freeze <existing FREEZE.json> (PREREG seeds.holdout_policy)'
                             % seeds['HOLDOUT'])
    jobs = []
    for s in specs:
        _, meta = build_episode(prereg, catalogue, s['plant'], s['condition'], s['corridor_spec'], s['seed'])
        meta['corridor_spec'] = s['corridor_spec']
        meta.pop('sided_gain', None)
        jobs.append(dict(episode_id=episode_id(meta), meta=meta))
    ids = [j['episode_id'] for j in jobs]
    if len(ids) != len(set(ids)):
        raise SystemExit('duplicate episodes in the plan')
    return jobs


def source_hashes(prereg, catalogue_path):
    files = {
        'tools/row_executor/single_row_sim.py': os.path.join(HERE, 'single_row_sim.py'),
        'tools/row_executor/sim_in_loop.py': os.path.join(HERE, 'sim_in_loop.py'),
        'tools/row_executor/SHA256SUMS': os.path.join(HERE, 'SHA256SUMS'),
        'tools/row_executor/vendor/SOURCE_MANIFEST.json': os.path.join(HERE, 'vendor', 'SOURCE_MANIFEST.json'),
        'sim/ubts/closed_loop/harness.py': os.path.join(SIM_DIR, 'closed_loop', 'harness.py'),
        'sim/ubts/closed_loop/raster_corridor.py': os.path.join(SIM_DIR, 'closed_loop', 'raster_corridor.py'),
        'sim/ubts/closed_loop/raster_catalogue.py': os.path.join(SIM_DIR, 'closed_loop', 'raster_catalogue.py'),
        'sim/ubts/closed_loop/single_row.py': os.path.join(SIM_DIR, 'closed_loop', 'single_row.py'),
        'sim/ubts/closed_loop/estimator.py': os.path.join(SIM_DIR, 'closed_loop', 'estimator.py'),
        'sim/ubts/closed_loop/controllers.py': os.path.join(SIM_DIR, 'closed_loop', 'controllers.py'),
        'sim/ubts/field_response.py': os.path.join(SIM_DIR, 'field_response.py'),
    }
    out = {k: sha256_file(v) for k, v in files.items() if os.path.isfile(v)}
    dash_path = SIL.rover_control_class()[2]
    out['dashboard rover_control_core.py'] = sha256_file(dash_path)
    out['CATALOGUE.json'] = sha256_file(catalogue_path)
    out['PREREG.json'] = prereg['_sha256']
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    p.add_argument('--out-dir', required=True)
    p.add_argument('--run-kind', choices=('smoke', 'dev', 'final'), required=True)
    p.add_argument('--episode', action='append', help='plant:condition:record:corridor:dir:seed (repeatable)')
    p.add_argument('--corridors', nargs='+', help='record:cNN:dir or record:La-Rb:dir (with --plants/--conditions/--seeds)')
    p.add_argument('--plants', nargs='+', choices=sorted(PLANTS))
    p.add_argument('--conditions', nargs='+')
    p.add_argument('--seeds', type=int, nargs='+')
    p.add_argument('--prereg', action='store_true', help='the PREREG matrix (plants x conditions x --selection x seeds)')
    p.add_argument('--selection', help='SELECTION.json named by CORRIDOR_FREEZE.json (label digests are re-checked)')
    p.add_argument('--seed-set', choices=('dev', 'holdout', 'all'), default='dev')
    p.add_argument('--freeze', help='FREEZE.json (required for holdout seeds)')
    p.add_argument('--row-length', choices=('none', 'l_op'), default=None,
                   help="none (no row length at Start; this tool's default) or l_op (PREREG L_op = s_E - 1 + 5 m; the "
                        "default with --prereg)")
    p.add_argument('--press-policy', choices=('prereg_fixed', 'page_ready'), default='prereg_fixed')
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--video', default='none', help="'none', 'all' or comma-separated episode ids")
    p.add_argument('--no-traces', action='store_true')
    p.add_argument('--max-time-s', type=float, default=None, help='development only: cap the SIM time (not in a final run)')
    p.add_argument('--prereg-path', default=None)
    p.add_argument('--catalogue', default=None)
    p.add_argument('--config-override', action='append', default=[],
                   help='PLANT:KEY=JSON executor config override (development only; recorded in the manifest)')
    args = p.parse_args(argv)
    if not 1 <= args.workers <= 12:
        raise SystemExit('workers must be 1..12')
    SIL.hold_file()                  # the hold folder must be configured (external_paths 'hold_dir'): else no run at all
    if args.run_kind == 'final' and (not args.prereg or args.max_time_s is not None or args.config_override):
        raise SystemExit('a final run is the PREREG matrix (--prereg), without --max-time-s / --config-override')
    row_length = args.row_length or ('l_op' if args.prereg else 'none')
    prereg = load_prereg(args.prereg_path)
    catalogue_path = str(RC.catalogue_path(args.catalogue))
    catalogue = RC.load_catalogue(catalogue_path)
    overrides = {}
    for item in args.config_override:
        head, value = item.split('=', 1)
        plant, key = head.split(':', 1)
        overrides.setdefault(plant, {})[key] = json.loads(value)
    jobs = plan_jobs(args, prereg, catalogue)
    if not jobs:
        raise SystemExit('nothing to run')
    configs = {plant: single_row_executor_config(plant, overrides.get(plant))
               for plant in sorted({j['meta']['plant'] for j in jobs})}
    os.makedirs(args.out_dir, exist_ok=True)
    run_id = os.path.basename(os.path.normpath(args.out_dir))
    video = 'all' if args.video == 'all' else (None if args.video == 'none' else set(args.video.split(',')))
    options = dict(run_id=run_id, run_kind=args.run_kind, prereg_path=prereg['_path'], catalogue_path=catalogue_path,
                   row_length=row_length, press_policy=args.press_policy, config_overrides=overrides,
                   trace_dir=None if args.no_traces else os.path.join(args.out_dir, 'traces'),
                   video=video, video_dir=os.path.join(args.out_dir, 'videos'), max_time_s=args.max_time_s)
    rows_path = os.path.join(args.out_dir, 'EPISODES.jsonl')
    done = {}
    if os.path.exists(rows_path):
        with open(rows_path, 'r', encoding='utf-8') as fh:
            for line in fh:
                if line.strip():
                    row = json.loads(line)
                    done[row['episode_id']] = row
    todo = [j for j in jobs if j['episode_id'] not in done]
    started_utc = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    manifest = dict(
        schema=SCHEMA_MANIFEST, run_id=run_id, run_kind=args.run_kind,
        purpose={'smoke': 'pipeline smoke only, NOT evaluation', 'dev': 'development (DEV seeds), not the verdict',
                 'final': 'PREREG final run'}[args.run_kind],
        created_utc=started_utc, prereg=dict(path=prereg['_path'], sha256=prereg['_sha256'],
                                             registered_sha256=PREREG_REGISTERED_SHA256,
                                             amendments_present=os.path.exists(prereg['_amendments'])),
        corridor_freeze=None, freeze=dict(path=args.freeze, sha256=sha256_file(args.freeze)) if args.freeze else None,
        selection=None if not args.selection else dict(path=args.selection, sha256=sha256_file(args.selection)),
        git_head=_git('rev-parse', 'HEAD'), git_status_short=_git('status', '--short'),
        sources_sha256=source_hashes(prereg, catalogue_path),
        executor_configs={plant: dict(digest=cfg.digest(), config=cfg.to_dict(),
                                      controller_settings=cfg.controller_settings()[1], feasibility=cfg.feasibility())
                          for plant, cfg in configs.items()},
        config_overrides=overrides, row_length=row_length, max_time_s_override=args.max_time_s,
        l_op_statement='L_op (when used) drives only the executor distance / time backstop; the row-end decision uses '
                       'the estimator row-end evidence only (single_row.py), never L_op, elapsed time or distance',
        press_policy=args.press_policy, localisation_path=XC.SINGLE_ROW_LOCALISATION, gps='none', camera='none',
        pwm_saturation_caps='(D_max, U_max) = (diff_cap_pct, duty_cap_pct) of the frozen config',
        workers=args.workers, platform=dict(system=platform.system(), release=platform.release(),
                                            machine=platform.machine(), processor=platform.processor(),
                                            python=sys.version.split()[0], cpu_count=os.cpu_count()),
        planned=len(jobs), already_done=len(done), to_run=len(todo), hold_waits=0,
        episodes=[j['episode_id'] for j in jobs])
    if args.prereg:
        manifest['corridor_freeze'] = 'see --selection (CORRIDOR_FREEZE.json names it)'
    manifest_path = os.path.join(args.out_dir, 'RUN_MANIFEST.json')
    if done and os.path.exists(manifest_path):
        # a resumed run must be the same run: same kind, policies, PREREG, selection and executor configs
        with open(manifest_path, 'r', encoding='utf-8') as fh:
            previous = json.load(fh)
        for key in ('run_kind', 'row_length', 'press_policy', 'max_time_s_override', 'config_overrides'):
            if previous.get(key) != manifest[key]:
                raise SystemExit('resume refused: %s differs from the existing RUN_MANIFEST.json' % key)
        if previous['prereg']['sha256'] != manifest['prereg']['sha256'] or previous.get('selection') != manifest['selection']:
            raise SystemExit('resume refused: PREREG or selection differs from the existing RUN_MANIFEST.json')
        if {k: v['digest'] for k, v in previous['executor_configs'].items()} !=                 {k: v['digest'] for k, v in manifest['executor_configs'].items()}:
            raise SystemExit('resume refused: executor config digests differ from the existing RUN_MANIFEST.json')
        manifest['resumed_from'] = dict(created_utc=previous.get('created_utc'), rows_before=len(done))
    _write_json(manifest_path, manifest)
    print('single-row SIM %s: %d planned, %d done, %d to run, %d workers' % (args.run_kind, len(jobs), len(done),
                                                                           len(todo), args.workers), flush=True)
    t0 = time.time()
    with concurrent.futures.ProcessPoolExecutor(max_workers=args.workers) as pool, \
            open(rows_path, 'a', encoding='utf-8', newline='\n') as out:
        pending, queue = {}, list(todo)
        while queue or pending:
            while queue and len(pending) < args.workers:
                if hold_present():
                    if pending:
                        break
                    manifest['hold_waits'] += 1
                    print('HOLD_MEASUREMENT present, waiting 60 s', flush=True)
                    time.sleep(60)
                    continue
                job = queue.pop(0)
                pending[pool.submit(run_job, dict(job, options=options))] = job
            if not pending:
                continue
            complete, _ = concurrent.futures.wait(set(pending), return_when=concurrent.futures.FIRST_COMPLETED)
            for future in complete:
                job = pending.pop(future)
                try:
                    row = future.result()
                except Exception as exc:                      # worker died: recorded, not marked done (re-run retries)
                    print('ERROR %s: %r' % (job['episode_id'], exc), flush=True)
                    continue
                done[row['episode_id']] = row
                out.write(json.dumps(row, separators=(',', ':'), default=str) + '\n')
                out.flush()
                print('%-70s %s %s e=%s %s (%.0f s)' % (row['episode_id'], row.get('end_kind'), row.get('stop_category'),
                                                         row.get('e_stop_m'), row.get('verdict'), row.get('wall_s', 0)),
                      flush=True)
    rows = [done[j['episode_id']] for j in jobs if j['episode_id'] in done]
    expected = len(jobs) if args.run_kind == 'final' else None
    summary = summarise(rows, prereg, args.run_kind, expected)
    manifest.update(finished_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), wall_s=round(time.time() - t0, 1),
                    rows=len(rows))
    _write_json(manifest_path, manifest)
    _write_json(os.path.join(args.out_dir, 'SUMMARY.json'), summary)
    with open(os.path.join(args.out_dir, 'REPORT_KO.txt'), 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(report_ko(summary, manifest, rows))
    print('overall: %s; rows %d; wall %.0f s' % (summary['overall'], len(rows), time.time() - t0), flush=True)
    return 0


def _write_json(path, data):
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        json.dump(data, fh, indent=1, default=str)
        fh.write('\n')


if __name__ == '__main__':
    sys.exit(main())
