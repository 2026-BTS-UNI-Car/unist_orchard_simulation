"""2D-harness comparison: SingleRowController vs the plain continuous RowController (SIM only).

    cd sim/ubts
    python -B -m closed_loop.single_row_eval --out <file.json> --seeds 0 1 2 3 --workers 12

Each episode is harness.run_episode (unchanged: 100 Hz FieldPlant, 10 Hz SIM LiDAR, 20 Hz raw IMU,
truth only for sensor synthesis and scoring) with the SAME EpisodeConfig (hence the same world,
plant, noise and latency draws) for both controllers. Scenes reuse matrix.SCENARIOS unchanged
(same name = the identical matrix episode) plus single-row scenes built on the nominal world with
plant overrides / yaw disturbances (drift that changes along the row, weak arc, start offsets). The
plant itself is never weakened: overrides only make drift or steering harder than the defaults.

Development seeds only (matrix.SEEDS[:4] = 0-3); tuning happened on these. Nothing here is a holdout.
Labels are truth-based and evaluation-only; the controllers never see truth.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import statistics
import subprocess
import time

import numpy as np

from . import external_paths
from .controllers import config_for
from .harness import EpisodeConfig, run_episode, stream_seed
from .matrix import SCENARIOS as MATRIX_SCENARIOS
from .single_row import SingleRowController, make_single_row_config, stop_category
from .world import WorldConfig

DEV_SEEDS = (0, 1, 2, 3)
# The HOLD_MEASUREMENT file is looked for in the configured hold folder (UBTS_HOLD_DIR, else 'hold_dir' of the
# UBTS_STEER_PATHS file; external_paths.hold_file). main() stops with an error when it is not configured.

# expectation kinds: 'row_end' (completion = row-end stop at the true end), 'wall_end' (row end; a stop
# at the end wall is also safe), 'end_wall' (end structure or row end), 'midrow_wall' (front stop at the
# wall, never a row end), 'authority' (completion, or a safe CONTROL_AUTHORITY stop)
_NOMINAL = dict(start_yaw_deg=5.)
SCENES = {
    # --- matrix scenes, unchanged ------------------------------------------------------------------
    'nominal_open_end': ('row_end', MATRIX_SCENARIOS['nominal_open_end'].overrides),
    'missing_trunk_gap': ('row_end', MATRIX_SCENARIOS['missing_trunk_gap'].overrides),
    'dense_branches': ('row_end', MATRIX_SCENARIOS['dense_branches'].overrides),
    'long_row_50m': ('row_end', MATRIX_SCENARIOS['long_row_50m'].overrides),
    'steering_ineffective': ('authority', MATRIX_SCENARIOS['steering_ineffective'].overrides),
    'wall_beyond_rows': ('wall_end', MATRIX_SCENARIOS['wall_beyond_rows'].overrides),
    'wall_beyond_6m': ('wall_end', MATRIX_SCENARIOS['wall_beyond_6m'].overrides),
    'narrow_headland': ('end_wall', MATRIX_SCENARIOS['narrow_headland'].overrides),
    'wall_across_before_end': ('midrow_wall', MATRIX_SCENARIOS['wall_across_before_end'].overrides),
    # --- single-row scenes on the nominal world (new names = their own random streams) --------------
    # Drift: the 10/10 straight-line curvature is an OU process in distance (ESTIMATED: mean -0.15,
    # sd 0.95 deg/m, correlation 8 m SIM_ASSUMED). These push it to the measured p10/p90 tails and beyond.
    'drift_right_strong': ('row_end', dict(_NOMINAL, plant_overrides=(('wander_mean_deg_per_m', -1.3),
                                                                      ('wander_sd_deg_per_m', .5)))),
    'drift_left_strong': ('row_end', dict(_NOMINAL, plant_overrides=(('wander_mean_deg_per_m', 1.0),
                                                                     ('wander_sd_deg_per_m', .5)))),
    'drift_varying_fast': ('row_end', dict(_NOMINAL, plant_overrides=(('wander_sd_deg_per_m', 1.5),
                                                                      ('wander_corr_length_m', 4.)))),
    # A drift that reverses mid-row: external body yaw rate (gyro-sensed, like a slope / skid), +-0.6 deg/s
    # = +-1.5 deg/m at 0.4 m/s on top of the default wander.
    'drift_reversal': ('row_end', dict(_NOMINAL, yaw_disturbances=((4., 22., .6), (22., 60., -.6)))),
    # Prior dev1010_bias scene (field10-autonomy-integration v6): gyro bias 0.4 deg/s + a 2 deg/s skid.
    'gyro_bias_skid': ('row_end', dict(start_yaw_deg=5., start_y_m=.1, yaw_disturbances=((10., 12., 2.),),
                                       gyro_bias_deg_s=.4)),
    'weak_arc_half': ('authority', dict(start_yaw_deg=8., plant_overrides=(('arc_yaw_deg_s_per_pct_diff', .06),))),
    'weak_arc_drift': ('authority', dict(start_yaw_deg=5., plant_overrides=(('arc_yaw_deg_s_per_pct_diff', .04),
                                                                            ('wander_mean_deg_per_m', -1.0),
                                                                            ('wander_sd_deg_per_m', .5)))),
    'start_offset_left': ('row_end', dict(start_y_m=.4, start_yaw_deg=10.)),
    'start_offset_right': ('row_end', dict(start_y_m=-.4, start_yaw_deg=-10.)),
    # Over-actuation: arc gain 2.5x the nominal 0.12 the feed-forward assumes (sim_in_loop bracket arc030).
    'strong_arc': ('row_end', dict(start_yaw_deg=8., plant_overrides=(('arc_yaw_deg_s_per_pct_diff', .30),))),
    # --- sensor scenes from the matrix, unchanged ------------------------------------------------------
    'latency_jitter': ('row_end', MATRIX_SCENARIOS['latency_jitter'].overrides),
    'lidar_blackout_dropouts': ('row_end', MATRIX_SCENARIOS['lidar_blackout_dropouts'].overrides),
    'imu_noise_vibration': ('row_end', MATRIX_SCENARIOS['imu_noise_vibration'].overrides),
}


# Optional plant preset under every scene (the scene's own plant_overrides win for the names they set; the
# random streams do not depend on it). p_fit1003 = tools/row_executor/sim_in_loop.P_FIT1003 (WEEKEND_PLAN 4.2:
# 40/55/70/88 % -> 0.33/0.47/0.61/0.78 m/s, arc 0.15 deg/s per %diff, vibration 0.7, pivot min 55); SIM only.
PLANT_PRESETS = dict(field1010=(), p_fit1003=(('speed_at_90_m_s', .80), ('forward_deadband_pct', 5.),
                                              ('arc_yaw_deg_s_per_pct_diff', .15), ('drive_vibration_rms_m_s2', .7),
                                              ('pivot_min_duty_pct', 55.)))


def episode(name, seed, mode='continuous', plant='field1010'):
    """EpisodeConfig built exactly like matrix.episodes (world seed from the episode's own stream)."""
    _, overrides = SCENES[name]
    world = overrides.get('world', WorldConfig())
    config = EpisodeConfig(name=name, mode=mode, seed=seed, **{k: v for k, v in overrides.items() if k != 'world'})
    config = replace(config, world=replace(world, seed=stream_seed(config, 'world')))
    preset = dict(PLANT_PRESETS[plant])
    if preset:
        own = dict(config.plant_overrides)
        config = replace(config, plant_overrides=tuple(sorted({**preset, **own}.items())))
    return config


# Row-end window of the BASE position relative to the last trunk (truth): RowController aims the base at the
# observed end (row_end_stop_m 0; matrix window -0.5..+1.5), SingleRowController the FRONT (base 0.61 m before
# it); one window covers both design targets: -0.61 - 0.6 .. +1.5. A ROW_END stop before it is a false
# (early) end, after it a late one. Evaluation choice of this comparison, not a pre-registered criterion.
ROW_END_WINDOW_M = (-1.21, 1.5)
HALF_LENGTH_M = .61                                   # CATALOGUE 1.22 m footprint (plant default)


def judge(kind, result, category):
    """Truth-based labels (evaluation only). category = stop category of the final reason."""
    collision = result['collisions'] > 0
    left = result['left_corridor']
    rel = result['final_x_rel_row_end_m']
    at_end = ROW_END_WINDOW_M[0] <= rel <= ROW_END_WINDOW_M[1]
    gap = result['wall_gap_m']
    at_wall = gap is not None and .1 <= gap <= 1.2 and result['progress_m'] >= 1.
    ok = False
    safe = False
    if kind in ('row_end', 'authority'):
        ok = category == 'ROW_END' and at_end
        safe = kind == 'authority' and category == 'CONTROL_AUTHORITY'
    elif kind == 'wall_end':
        ok = category == 'ROW_END' and at_end
        safe = category in ('END_STRUCTURE', 'OBSTACLE') and at_wall
    elif kind == 'end_wall':
        ok = (category == 'ROW_END' and at_end) or (category in ('END_STRUCTURE', 'OBSTACLE') and at_wall)
    elif kind == 'midrow_wall':
        ok = category in ('END_STRUCTURE', 'OBSTACLE') and at_wall
    completed = ok and not collision and not left
    terminal = result['end'] == 'terminal_hold'
    return dict(completed=completed, acceptable=completed or (safe and not collision and not left and terminal),
                collision=collision, false_row_end=bool(category == 'ROW_END' and rel < ROW_END_WINDOW_M[0]),
                late_row_end=bool(category == 'ROW_END' and rel > ROW_END_WINDOW_M[1]),
                matrix_window_ok=bool(category == 'ROW_END' and -.5 <= rel <= 1.5),
                row_end_missed=bool(kind in ('row_end', 'wall_end') and category != 'ROW_END' and not collision))


def _weighted_percentile(values, weights, q):
    order = np.argsort(values)
    values, weights = values[order], weights[order]
    cumulative = np.cumsum(weights)
    if cumulative[-1] <= 0:
        return float(np.percentile(values, q))
    return float(values[min(len(values) - 1, int(np.searchsorted(cumulative, q / 100. * cumulative[-1])))])


def _row_metrics(trace, row_end_x):
    """Time-weighted (20 Hz samples) and DISTANCE-weighted (|dx| between samples) in-row errors: a controller
    that slows down at a large error is not penalised for the extra samples there (per metre of row)."""
    inside = [(p, q) for p, q in zip(trace[1:], trace[:-1]) if 0. <= p['fx'] <= row_end_x]
    if not inside:
        inside = [(trace[-1], trace[-1])]
    lateral = np.abs([p['fy'] for p, _ in inside])
    heading = np.abs([p['fyaw_deg'] for p, _ in inside])
    weights = np.abs([p['fx'] - q['fx'] for p, q in inside])
    total = float(weights.sum())

    def weighted_rms(values, digits):
        return round(float(np.sqrt(np.sum(weights * values ** 2) / total)), digits) if total > 0 else None
    moving = [p for p in trace if p['left'] or p['right']]
    diffs = [p['right'] - p['left'] for p in moving]
    return dict(lateral_rmse_m=round(float(np.sqrt(np.mean(lateral ** 2))), 4),
                lateral_p95_m=round(float(np.percentile(lateral, 95)), 4),
                lateral_max_in_row_m=round(float(lateral.max()), 4),
                lateral_rmse_dist_m=weighted_rms(lateral, 4),
                lateral_p95_dist_m=round(_weighted_percentile(lateral, weights, 95), 4),
                heading_rmse_deg=round(float(np.sqrt(np.mean(heading ** 2))), 3),
                heading_rmse_dist_deg=weighted_rms(heading, 3),
                heading_p95_deg=round(float(np.percentile(heading, 95)), 3),
                abs_diff_p95_pct=round(float(np.percentile(np.abs(diffs), 95)), 1) if diffs else 0.,
                pivot_or_reverse_commands=sum(1 for p in trace if p['left'] < 0 or p['right'] < 0))


def run_job(job):
    name, seed, controller, overrides = job[:4]
    plant = job[4] if len(job) > 4 else 'field1010'
    kind = SCENES[name][0]
    config = episode(name, seed, plant=plant)
    holder = {}
    if controller == 'single':
        def factory(_control):
            holder['c'] = SingleRowController(make_single_row_config(**overrides))
            return holder['c']
        kwargs = dict(controller_factory=factory)
    elif controller == 'plain':
        if overrides:
            config = replace(config, controller_overrides=tuple(sorted(overrides.items())))
        kwargs = {}
    else:
        raise ValueError(controller)
    started = time.perf_counter()
    result = run_episode(config, record_trace=True, **kwargs)
    wall = time.perf_counter() - started
    trace = result.pop('trace')
    row_end_x = result['final_x_rel_row_end_m']
    row_end_x = result['progress_m'] + config.start_x_m - row_end_x      # truth end x (field frame)
    category = stop_category(result['final_reason']) if result['end'] == 'terminal_hold' else None
    row = dict(scene=name, kind=kind, seed=seed, controller=controller, end=result['end'],
               final_reason=result['final_reason'], category=category, time_s=result['time_s'],
               progress_m=result['progress_m'], final_x_rel_row_end_m=result['final_x_rel_row_end_m'],
               wall_gap_m=result['wall_gap_m'], mean_speed_m_s=result['mean_speed_m_s'],
               lateral_rms_m=result['lateral_rms_m'], lateral_max_m=result['lateral_max_m'],
               heading_rms_deg=result['heading_rms_deg'], left_corridor=result['left_corridor'],
               collisions=result['collisions'], min_clearance_m=result['min_clearance_m'],
               min_branch_clearance_m=result['min_branch_clearance_m'],
               branch_contact_samples_moving=result['branch_contact_samples_moving'],
               chatter_flips=result['chatter_flips'], chatter_per_min=result['chatter_per_min'],
               rapid_flips=result['rapid_flips'], stops=result['stops'],
               intermediate_stops=result['intermediate_stops'], holds=result['holds'],
               row_end_first_estimate=result['row_end_first_estimate'],
               row_end_last_estimate=result['row_end_last_estimate'], wall_s=round(wall, 2),
               state_time_s=result['state_time_s'], **_row_metrics(trace, row_end_x))
    row['front_x_rel_row_end_m'] = round(row['final_x_rel_row_end_m'] + HALF_LENGTH_M, 3)
    row.update(judge(kind, row, category))
    if controller == 'single':
        c = holder['c']
        telemetry = c.telemetry()
        # Row-end slow zone: from the (last) APPROACH_END entry to the stop request that followed it (controller
        # events), distance from the truth trace (evaluation only).
        entries = [t for t, name, _ in c.phase_events if name == 'APPROACH_END']
        entry = entries[-1] if entries else None
        stop = next((t for t, state, _ in c.events if state == 'STOP_REQUESTED' and entry is not None and t >= entry),
                    None)
        along = {p['t']: p['fx'] for p in trace}
        approach_time = approach_distance = None
        if entry is not None and stop is not None:
            approach_time = round(stop - entry, 2)
            x0, x1 = along.get(round(entry, 2)), along.get(round(stop, 2))
            approach_distance = None if x0 is None or x1 is None else round(x1 - x0, 3)
        row.update(saturation_fraction=round(telemetry['saturation_fraction'], 4), bias_pct=round(c.bias_pct, 3),
                   base_duty_final_pct=round(c.base_duty, 2), authority_state=c.authority_state,
                   phase_events=c.phase_events[:12], counts=dict(c.counts),
                   approach_time_s=approach_time, approach_distance_m=approach_distance)
    return row


def summarise(rows):
    def mean(values):
        values = [v for v in values if v is not None and math.isfinite(v)]
        return round(statistics.fmean(values), 4) if values else None
    out = {}
    for controller in sorted({r['controller'] for r in rows}):
        mine = [r for r in rows if r['controller'] == controller]
        by_scene = {}
        for scene in SCENES:
            part = [r for r in mine if r['scene'] == scene]
            if not part:
                continue
            by_scene[scene] = dict(
                runs=len(part), completed=sum(r['completed'] for r in part),
                acceptable=sum(r['acceptable'] for r in part), collisions=sum(r['collision'] for r in part),
                categories={k: sum(str(r['category']) == k for r in part)
                            for k in sorted({str(r['category']) for r in part})},
                final_reasons=sorted(r['final_reason'] for r in part),
                lateral_rmse_m=mean([r['lateral_rmse_m'] for r in part]),
                lateral_p95_m=mean([r['lateral_p95_m'] for r in part]),
                lateral_rmse_dist_m=mean([r['lateral_rmse_dist_m'] for r in part]),
                lateral_p95_dist_m=mean([r['lateral_p95_dist_m'] for r in part]),
                lateral_max_m=round(max(r['lateral_max_in_row_m'] for r in part), 3),
                heading_rmse_deg=mean([r['heading_rmse_deg'] for r in part]),
                heading_rmse_dist_deg=mean([r['heading_rmse_dist_deg'] for r in part]),
                final_x_rel_row_end_m=[r['final_x_rel_row_end_m'] for r in part],
                progress_m=mean([r['progress_m'] for r in part]), time_s=mean([r['time_s'] for r in part]),
                chatter_flips=sum(r['chatter_flips'] for r in part), rapid_flips=sum(r['rapid_flips'] for r in part),
                saturation_fraction=mean([r.get('saturation_fraction') for r in part]),
                min_branch_clearance_m=round(min(r['min_branch_clearance_m'] for r in part), 3))
        out[controller] = dict(runs=len(mine), completed=sum(r['completed'] for r in mine),
                               acceptable=sum(r['acceptable'] for r in mine),
                               collisions=sum(r['collision'] for r in mine),
                               false_row_end=sum(r['false_row_end'] for r in mine),
                               late_row_end=sum(r['late_row_end'] for r in mine),
                               row_end_missed=sum(r['row_end_missed'] for r in mine),
                               matrix_window_ok=sum(r['matrix_window_ok'] for r in mine),
                               lateral_rmse_m=mean([r['lateral_rmse_m'] for r in mine]),
                               lateral_p95_m=mean([r['lateral_p95_m'] for r in mine]),
                               lateral_rmse_dist_m=mean([r['lateral_rmse_dist_m'] for r in mine]),
                               lateral_p95_dist_m=mean([r['lateral_p95_dist_m'] for r in mine]),
                               lateral_max_m=round(max(r['lateral_max_in_row_m'] for r in mine), 3),
                               heading_rmse_deg=mean([r['heading_rmse_deg'] for r in mine]),
                               heading_rmse_dist_deg=mean([r['heading_rmse_dist_deg'] for r in mine]),
                               chatter_flips=sum(r['chatter_flips'] for r in mine),
                               rapid_flips=sum(r['rapid_flips'] for r in mine),
                               mean_time_s=mean([r['time_s'] for r in mine]),
                               pivot_or_reverse_commands=sum(r['pivot_or_reverse_commands'] for r in mine),
                               by_scene=by_scene)
    return out


def _git(*args):
    try:
        return subprocess.run(('git',) + args, capture_output=True, text=True, cwd=Path(__file__).parent,
                              timeout=20).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--out', required=True)
    p.add_argument('--seeds', type=int, nargs='+', default=list(DEV_SEEDS))
    p.add_argument('--scenes', nargs='+', default=None)
    p.add_argument('--controllers', nargs='+', default=['plain', 'single'])
    p.add_argument('--workers', type=int, default=8)
    p.add_argument('--single', default='{}', help='JSON overrides of SingleRowConfig')
    p.add_argument('--plain', default='{}', help='JSON controller_overrides of the plain continuous RowController')
    p.add_argument('--plant', default='field1010', choices=sorted(PLANT_PRESETS),
                   help='plant preset under every scene (scene overrides win)')
    args = p.parse_args(argv)
    if args.workers > 12:
        raise SystemExit('at most 12 worker processes')
    hold_file = external_paths.hold_file()
    while hold_file.exists():
        print('waiting: %s exists' % hold_file, flush=True)
        time.sleep(60)
    single = json.loads(args.single)
    plain = json.loads(args.plain)
    make_single_row_config(**single)                 # validate before any work
    config_for('continuous', **plain)
    scenes = args.scenes or list(SCENES)
    jobs = [(scene, seed, controller, single if controller == 'single' else plain, args.plant)
            for scene in scenes for seed in args.seeds for controller in args.controllers]
    started = time.perf_counter()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(run_job, jobs))
    payload = dict(schema='ubts.single_row.harness_comparison.v1', seeds=args.seeds, development_seeds_only=True,
                   git_head=_git('rev-parse', 'HEAD'), git_status=_git('status', '--short'),
                   single_row_config=asdict(make_single_row_config(**single)),
                   plain_config=asdict(config_for('continuous', **plain)), plant=args.plant,
                   plant_preset_overrides=dict(PLANT_PRESETS[args.plant]),
                   plant_note='harness FieldResponseParams (field_20261010) + preset + scene overrides (scene wins)',
                   wall_s=round(time.perf_counter() - started, 1),
                   scenes={k: dict(kind=v[0], overrides=repr(v[1])) for k, v in SCENES.items() if k in scenes},
                   summary=summarise(rows), rows=rows)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, 'w', encoding='utf-8', newline='\n') as handle:
        json.dump(payload, handle, indent=1, default=str)
    for controller, block in payload['summary'].items():
        print(controller, json.dumps({k: v for k, v in block.items() if k != 'by_scene'}))
    for scene in scenes:
        line = [scene]
        for controller in args.controllers:
            b = payload['summary'][controller]['by_scene'].get(scene)
            if b:
                line.append('%s ok %d/%d latD %.3f/%.3f max %.3f hdgD %.2f t %.0f end %s cat %s chat %d sat %s' % (
                    controller[:6], b['completed'], b['runs'], b['lateral_rmse_dist_m'], b['lateral_p95_dist_m'],
                    b['lateral_max_m'], b['heading_rmse_dist_deg'], b['time_s'], b['final_x_rel_row_end_m'],
                    b['categories'], b['chatter_flips'], b['saturation_fraction']))
        print(' | '.join(line))


if __name__ == '__main__':
    main()
