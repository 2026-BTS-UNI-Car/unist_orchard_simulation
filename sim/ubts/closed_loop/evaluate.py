"""Run the seeded A/B/C matrix and write a JSON result (SIM only).

    cd sim/ubts
    python -m closed_loop.evaluate --out dev.json --workers 4   (docs JSON is assembled from several runs)
    python -m closed_loop.evaluate --out holdout.json --seeds 8 9 10 11 --workers 4
    python -m closed_loop.evaluate --out side.json --extra-scenarios --scenarios aisle_2p2m
        --modes hybrid hybrid_side --pairs hybrid:hybrid_side      (one command; opt-in side rule/scenes)
    python -m closed_loop.evaluate --out rowend.json --row-end-position-confirm-m 0.5
        (opt-in estimator variant on the SAME episodes as a run without the flag; compare the two files
         run by run, e.g. with compare_runs.py; recorded as estimator_overrides + the effective estimator)
    python -m closed_loop.evaluate --out continuation.json --row-end-continuation-m 12
        (opt-in line-continuation veto, pair rule v2: a two-sided row end whose BOTH line extensions show
         separated trunk-like objects within 12 m is not an end; --row-end-continuation-rule per_side runs the
         frozen v1 rule; recorded the same way; may be combined with --row-end-position-confirm-m)
    python -m closed_loop.evaluate --out headland.json --extra-scenarios headland_clutter --row-end-continuation-m 12
        (opt-in scene groups: a bare --extra-scenarios = side_clearance as before; headland_clutter = review F5)
    python -m closed_loop.evaluate --out fit.json --modes continuous_side
        --plant-override speed_at_90_m_s=0.80 --plant-override forward_deadband_pct=5
        --control-override base_duty_pct=55 --control-override max_differential_pct=20
        (opt-in matrix-wide plant / controller variant, SIM only: every NAME is a field_response.
         FieldResponseParams / controllers.ControlConfig field (tuple fields: comma-separated values; bool:
         true/false). A scenario's OWN plant_overrides / controller_overrides keep precedence for the names they
         set (e.g. the 78 % deadband scene stays at 78 %). Absent by default; recorded as plant_overrides /
         controller_overrides, the effective plant/controllers and, per scenario, its own overrides and the
         superseded global names. The random streams do not depend on them: the same (scenario, mode, seed) of a
         run without the flags shares every draw.)

Default seeds are matrix.SEEDS (0-7, development set); HOLDOUT_SEEDS (8-11)
are a confirmation set. Each episode is independent and deterministic for its
(scenario, seed); episodes run in worker processes. The JSON contains per-run
metrics, per-mode and per-scenario summaries with 95% Wilson intervals, the
paired A-vs-C comparison, every scenario's full configuration with the plant
values it overrides (SCENARIO_OVERRIDE_OF_*), the plant provenance table and
controller/estimator settings.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, fields, replace
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import time

from field_response import FieldResponseParams, provenance
from .controllers import ControlConfig, config_for
from .estimator import ROW_END_POSITION_MAX_GAP_FACTOR, EstimatorConfig
from .harness import describe_config, plant_params, run_episode
from .matrix import (ALL_MODES, EXTRA_GROUPS, HOLDOUT_SEEDS, MODES, MODE_LABEL, SCENARIOS, SEEDS, episodes,
                     extra_groups, judge, scenario_pool, scenario_spec)


def _run(config):
    started = time.perf_counter()
    result = run_episode(config)
    result['wall_s'] = round(time.perf_counter() - started, 2)
    result['events'] = result['events'][-14:]
    result.update(judge(result, config.name))
    return result


def _mean(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    return round(statistics.fmean(values), 4) if values else None


def wilson(k, n, z=1.96):
    if n == 0:
        return None
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return [round(max(0., centre - half), 3), round(min(1., centre + half), 3)]


def mcnemar_exact(b, c):
    """Two-sided exact McNemar p for discordant counts b, c."""
    n = b + c
    if n == 0:
        return 1.
    k = min(b, c)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n
    return round(min(1., 2 * tail), 4)


def block(rows):
    holds, finals = {}, {}
    for row in rows:
        for reason in row['holds']:
            holds[reason] = holds.get(reason, 0) + 1
        finals[row['final_reason']] = finals.get(row['final_reason'], 0) + 1
    n = len(rows)
    completed = sum(r['completed'] for r in rows)
    acceptable = sum(r['acceptable'] for r in rows)
    summary = dict(
        runs=n,
        completed=completed, completed_rate_ci95=wilson(completed, n),
        acceptable=acceptable, acceptable_rate_ci95=wilson(acceptable, n),
        expected_holds=sum(r['expected_hold'] for r in rows),
        unexpected_safe_holds=sum(r['safe_hold'] and not r['expected_hold'] for r in rows),
        collisions=sum(r['collision'] for r in rows),
        left_corridor_runs=sum(r['left_corridor'] for r in rows),
        branch_contact_runs=sum(r['branch_contact_samples'] > 0 for r in rows),
        invented_row_end=sum(r['invented_row_end'] for r in rows),
        early_row_end=sum(r['early_row_end'] for r in rows),
        row_end_recognised=sum(r['row_end_recognised'] for r in rows),
        time_limit_runs=sum(r['end'] == 'time_limit' for r in rows),
        lateral_rms_m_mean=_mean([r['lateral_rms_m'] for r in rows]),
        lateral_max_m_max=round(max(r['lateral_max_m'] for r in rows), 3),
        min_clearance_m=round(min(r['min_clearance_m'] for r in rows), 3),
        intermediate_stops_per_run=_mean([r['intermediate_stops'] for r in rows]),
        stops_without_turn_per_run=_mean([r['stops_without_turn'] for r in rows]),
        pivots_per_run=_mean([r['turns'] for r in rows]),
        stall_aborts=sum(r['stall_aborts'] for r in rows),
        plant_pivot_stalls=sum(r['plant_pivot_stalls'] for r in rows),
        sign_flips_per_run=_mean([r['chatter_flips'] for r in rows]),
        sign_flips_per_min=_mean([r['chatter_per_min'] for r in rows]),
        rapid_flips_total=sum(r['rapid_flips'] for r in rows),
        progress_m_mean=_mean([r['progress_m'] for r in rows]),
        mean_speed_m_s=_mean([r['mean_speed_m_s'] for r in rows]),
        time_s_mean=_mean([r['time_s'] for r in rows]),
        last_row=dict(correct=sum(r['last_row'] == 'correct' for r in rows),
                      undecided=sum(r['last_row'] == 'undecided' for r in rows),
                      wrong=sum(r['last_row'] == 'wrong' for r in rows)),
        final_reasons=dict(sorted(finals.items())),
        hold_reasons=dict(sorted(holds.items())),
        # --- added (side-clearance evaluation; truth metrics; .get: rows of older runs lack them) ---
        start_contact_runs=sum((r.get('start_branch_clearance_m') is not None and r['start_branch_clearance_m'] <= 0)
                               for r in rows),
        moving_contact_runs=sum(r.get('branch_contact_samples_moving', 0) > 0 for r in rows),
        moving_contact_samples=sum(r.get('branch_contact_samples_moving', 0) for r in rows),
        max_intrusion_increase_m=round(max(r.get('branch_intrusion_increase_m', 0.) for r in rows), 4),
        min_branch_clearance_m=round(min(r['min_branch_clearance_m'] for r in rows), 3),
        side_blocked_runs=sum(r['final_reason'] == 'side_blocked' for r in rows),
        side_stops=sum(r.get('side_stops', 0) for r in rows),
        side_inhibit_ticks=sum(r.get('side_inhibit_ticks', 0) for r in rows),
    )
    variant = [r['row_end_position']['counts'] for r in rows if 'row_end_position' in r]
    if variant:      # opt-in row-end position bridge only: the default summary keys are unchanged
        summary['row_end_position_attempts'] = {key: sum(c.get(key, 0) for c in variant)
                                                for key in sorted({k for c in variant for k in c})}
        summary['row_end_position_bridged_runs'] = sum(c.get('bridged', 0) > 0 for c in variant)
    continuation = [r['row_end_continuation']['counts'] for r in rows if 'row_end_continuation' in r]
    if continuation:  # opt-in line-continuation veto only: the default summary keys are unchanged
        summary['row_end_continuation_counts'] = {key: sum(c.get(key, 0) for c in continuation)
                                                  for key in sorted({k for c in continuation for k in c})}
        summary['row_end_continuation_cancelled_runs'] = sum(c.get('cancelled', 0) > 0 for c in continuation)
    return summary


def paired(runs, a='continuous', b='hybrid'):
    """Paired completion comparison per (scenario, seed): common random numbers."""
    index = {(r['name'], r['seed'], r['mode']): r['completed'] for r in runs}
    only_a, only_b, both, neither, detail = 0, 0, 0, 0, dict(only_a=[], only_b=[])
    for (name, seed, mode), done in index.items():
        if mode != a or (name, seed, b) not in index:
            continue
        other = index[(name, seed, b)]
        if done and other:
            both += 1
        elif done:
            only_a += 1
            detail['only_a'].append([name, seed])
        elif other:
            only_b += 1
            detail['only_b'].append([name, seed])
        else:
            neither += 1
    return dict(a=MODE_LABEL[a], b=MODE_LABEL[b], both=both, neither=neither, only_a=only_a, only_b=only_b,
                mcnemar_exact_p=mcnemar_exact(only_a, only_b), discordant=detail)


def summarise(runs, scenarios=SCENARIOS):
    by_mode = {MODE_LABEL[m]: block([r for r in runs if r['mode'] == m]) for m in ALL_MODES if any(r['mode'] == m for r in runs)}
    by_scenario = {}
    for name in scenarios:
        rows = [r for r in runs if r['name'] == name]
        if rows:
            by_scenario[name] = {MODE_LABEL[m]: block([r for r in rows if r['mode'] == m])
                                 for m in ALL_MODES if any(r['mode'] == m for r in rows)}
    by_category = {}
    for name, spec in scenarios.items():
        by_category.setdefault(spec.category, []).append(name)
    category = {cat: {MODE_LABEL[m]: block([r for r in runs if r['mode'] == m and r['name'] in names])
                      for m in ALL_MODES if any(r['mode'] == m and r['name'] in names for r in runs)}
                for cat, names in by_category.items() if any(r['name'] in names for r in runs)}
    return by_mode, by_scenario, category


def _git_head():
    try:
        return subprocess.run(['git', '-c', 'safe.directory=*', 'rev-parse', 'HEAD'], capture_output=True,
                              text=True, check=True, cwd=Path(__file__).resolve().parent).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _git_dirty():
    try:
        out = subprocess.run(['git', '-c', 'safe.directory=*', 'status', '--porcelain', '--', '.'],
                             capture_output=True, text=True, check=True,
                             cwd=Path(__file__).resolve().parents[1]).stdout
        return bool(out.strip())
    except (OSError, subprocess.CalledProcessError):
        return None


# --- opt-in matrix-wide plant / controller overrides (--plant-override / --control-override) ---------------

PLANT_FIELDS = {f.name: getattr(FieldResponseParams(), f.name) for f in fields(FieldResponseParams)}
CONTROL_FIELDS = {f.name: getattr(ControlConfig(), f.name) for f in fields(ControlConfig) if f.name != 'mode'}


def _number(text):
    value = float(text)
    if not math.isfinite(value):
        raise ValueError('not finite')
    return value


def parse_override(text, defaults, kind):
    """'NAME=VALUE' -> (name, typed value). The type follows the field's default: tuple -> comma-separated
    finite floats, bool -> true/false, int -> integer, float -> finite float. Raises ValueError."""
    name, sep, raw = text.partition('=')
    name, raw = name.strip(), raw.strip()
    if not sep or not name or not raw:
        raise ValueError('%s override %r is not NAME=VALUE' % (kind, text))
    if name not in defaults:
        raise ValueError('unknown %s override %r (known: %s)' % (kind, name, ', '.join(sorted(defaults))))
    default = defaults[name]
    try:
        if isinstance(default, bool):
            if raw.lower() not in ('true', 'false'):
                raise ValueError('expected true/false')
            value = raw.lower() == 'true'
        elif isinstance(default, int):
            value = int(raw)
        elif isinstance(default, tuple):
            value = tuple(_number(part) for part in raw.split(','))
        else:
            value = _number(raw)
    except ValueError as error:
        raise ValueError('%s override %s=%r: %s' % (kind, name, raw, error)) from None
    return name, value


def parse_overrides(texts, defaults, kind):
    pairs = [parse_override(text, defaults, kind) for text in texts or ()]
    names = [name for name, _ in pairs]
    duplicated = sorted({name for name in names if names.count(name) > 1})
    if duplicated:
        raise ValueError('%s override given more than once: %s' % (kind, duplicated))
    return tuple(pairs)


def apply_overrides(configs, plant_overrides=(), controller_overrides=()):
    """Attach matrix-wide overrides to every episode. A scenario's OWN plant_overrides / controller_overrides
    take precedence for the names it sets (the global value is dropped for that scenario, not stacked).
    Without global overrides the configurations are returned unchanged (identical objects)."""
    if not plant_overrides and not controller_overrides:
        return list(configs)
    result = []
    for config in configs:
        own_plant = {name for name, _ in config.plant_overrides}
        own_control = {name for name, _ in config.controller_overrides}
        result.append(replace(
            config,
            plant_overrides=tuple((k, v) for k, v in plant_overrides if k not in own_plant)
            + tuple(config.plant_overrides),
            controller_overrides=tuple((k, v) for k, v in controller_overrides if k not in own_control)
            + tuple(config.controller_overrides)))
    return result


def override_record(own, plant_overrides, controller_overrides):
    """Per-scenario record of its own overrides and the global names it supersedes."""
    own_plant, own_control = dict(own.plant_overrides), dict(own.controller_overrides)
    return dict(scenario_plant_overrides={k: list(v) if isinstance(v, tuple) else v for k, v in own_plant.items()},
                scenario_controller_overrides=own_control,
                superseded_global_plant_overrides=sorted(k for k, _ in plant_overrides if k in own_plant),
                superseded_global_controller_overrides=sorted(k for k, _ in controller_overrides if k in own_control))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--seeds', type=int, nargs='*', default=list(SEEDS))
    parser.add_argument('--scenarios', nargs='*')
    parser.add_argument('--modes', nargs='*', default=list(MODES))
    parser.add_argument('--extra-scenarios', nargs='*', default=None, metavar='GROUP',
                        help='also offer opt-in scene groups (matrix.EXTRA_GROUPS: %s); a bare flag = '
                        'side_clearance (matrix.EXTRA_SCENARIOS, as before); not part of the default matrix'
                        % ', '.join(sorted(EXTRA_GROUPS)))
    parser.add_argument('--pairs', nargs='*', default=[], metavar='A:B',
                        help='additional paired completion comparisons, e.g. hybrid:hybrid_side')
    parser.add_argument('--row-end-position-confirm-m', type=float, default=None, metavar='M',
                        help='opt-in estimator variant: bridge two-sided row-end observations older than '
                        'row_end_valid_s whose side end points agree within M (EstimatorConfig.'
                        'row_end_position_confirm_m; default OFF)')
    parser.add_argument('--row-end-position-max-gap-s', type=float, default=None, metavar='S',
                        help='maximum bridged gap for --row-end-position-confirm-m (default: the '
                        'EstimatorConfig value; finite, > row_end_valid_s and <= %g x row_end_valid_s)'
                        % ROW_END_POSITION_MAX_GAP_FACTOR)
    parser.add_argument('--row-end-continuation-m', type=float, default=None, metavar='L',
                        help='opt-in estimator variant: a row end whose line extensions visibly continue between '
                        'end + the required free gap and end + L is not an end (EstimatorConfig.'
                        'row_end_continuation_m; default OFF; finite, > end_free_m and <= free_check_range_m)')
    parser.add_argument('--row-end-continuation-rule', choices=('pair', 'per_side'), default=None,
                        help='rule of --row-end-continuation-m (default: the EstimatorConfig value, pair = v2: both '
                        'line extensions must show separated trunk-like objects; per_side = the frozen v1 rule)')
    parser.add_argument('--plant-override', action='append', default=[], metavar='NAME=VALUE',
                        help='opt-in matrix-wide SIM plant value (field_response.FieldResponseParams field; tuple '
                        'fields comma-separated); repeatable; a scenario\'s own plant override of the same name wins '
                        '(default: none)')
    parser.add_argument('--control-override', action='append', default=[], metavar='NAME=VALUE',
                        help='opt-in matrix-wide controller value (controllers.ControlConfig field except mode; '
                        'applied on top of config_for(mode)); repeatable; a scenario\'s own controller override of '
                        'the same name wins (default: none)')
    args = parser.parse_args(argv)
    try:
        plant_global = parse_overrides(args.plant_override, PLANT_FIELDS, 'plant')
        control_global = parse_overrides(args.control_override, CONTROL_FIELDS, 'controller')
    except ValueError as error:
        parser.error(str(error))
    overrides = []
    if args.row_end_position_confirm_m is not None:
        overrides.append(('row_end_position_confirm_m', args.row_end_position_confirm_m))
    if args.row_end_position_max_gap_s is not None:
        if args.row_end_position_confirm_m is None:
            parser.error('--row-end-position-max-gap-s needs --row-end-position-confirm-m')
        overrides.append(('row_end_position_max_gap_s', args.row_end_position_max_gap_s))
    if args.row_end_continuation_m is not None:
        overrides.append(('row_end_continuation_m', args.row_end_continuation_m))
    if args.row_end_continuation_rule is not None:
        if args.row_end_continuation_m is None:
            parser.error('--row-end-continuation-rule needs --row-end-continuation-m')
        overrides.append(('row_end_continuation_rule', args.row_end_continuation_rule))
    try:
        # EstimatorConfig validates the variants (finite values, tolerance < end_free_m, row_end_valid_s <
        # max gap <= ROW_END_POSITION_MAX_GAP_FACTOR x row_end_valid_s, end_free_m < continuation length <=
        # free_check_range_m); argparse accepts 'inf'/'nan'.
        estimator = replace(EstimatorConfig(), **dict(overrides))
    except ValueError as error:
        parser.error('invalid estimator variant %s: %s' % (dict(overrides), error))
    extra = False if args.extra_scenarios is None else tuple(args.extra_scenarios) or True
    try:
        groups = extra_groups(extra)
    except ValueError as error:
        parser.error(str(error))
    scenarios = scenario_pool(extra)
    unknown = sorted(set(args.scenarios or ()) - set(scenarios))
    if unknown:
        parser.error('unknown scenarios %s (extra scenarios need --extra-scenarios [GROUP ...])' % unknown)
    if set(args.modes) - set(ALL_MODES):
        parser.error('unknown modes %s' % sorted(set(args.modes) - set(ALL_MODES)))
    pairs = [tuple(p.split(':')) for p in args.pairs]
    if any(len(p) != 2 or not set(p) <= set(args.modes) for p in pairs):
        parser.error('--pairs needs A:B with both modes in --modes')
    configs = episodes(seeds=tuple(args.seeds), modes=tuple(args.modes), names=args.scenarios,
                       extra=extra, estimator_overrides=tuple(overrides))
    own_configs = {config.name: config for config in configs}     # before the matrix-wide overrides
    configs = apply_overrides(configs, plant_global, control_global)
    if plant_global or control_global:
        try:
            # FieldResponseParams / ControlConfig validate every merged episode before any worker starts.
            replace(FieldResponseParams(), **dict(plant_global))
            for config in configs:
                plant_params(config)
                config_for(config.mode, **dict(config.controller_overrides))
        except (TypeError, ValueError) as error:
            parser.error('invalid plant/controller override %s %s: %s'
                         % (dict(plant_global), dict(control_global), error))
    started = time.perf_counter()
    with ProcessPoolExecutor(args.workers) as pool:
        runs = list(pool.map(_run, configs, chunksize=1))
    by_mode, by_scenario, by_category = summarise(runs, scenarios)
    scenario_configs = {}
    for config in configs:
        if config.name not in scenario_configs:
            record = describe_config(config)
            record.pop('seed', None)
            record.pop('mode', None)
            record['world'].pop('seed', None)
            spec = scenario_spec(config.name)
            scenario_configs[config.name] = dict(description=spec.description, category=spec.category,
                                                 expected=list(spec.expected),
                                                 acceptable_holds=list(spec.acceptable_holds),
                                                 wall_stop=spec.wall_stop, config=record)
            if plant_global or control_global:     # opt-in only: the default scenario records are unchanged
                scenario_configs[config.name].update(override_record(own_configs[config.name], plant_global,
                                                                     control_global))
    payload = dict(
        schema='ubts.sim.field_response_control.v2',
        generated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        base_git_head=_git_head(), sim_tree_dirty=_git_dirty(),
        real_vehicle_validated=False,
        truth_used_for='sensor synthesis and evaluation metrics only',
        runtime_s=round(time.perf_counter() - started, 1), workers=args.workers,
        seeds=list(args.seeds), development_seeds=list(SEEDS), holdout_seeds=list(HOLDOUT_SEEDS),
        seed_set='holdout' if set(args.seeds) <= set(HOLDOUT_SEEDS) else
                 'development' if set(args.seeds) <= set(SEEDS) else 'mixed',
        plant=provenance(),
        estimator=asdict(estimator),       # effective estimator settings (EstimatorConfig() without a variant)
        controllers={MODE_LABEL[m]: asdict(config_for(m)) for m in ALL_MODES} if not control_global else
        {MODE_LABEL[m]: asdict(config_for(m, **dict(control_global))) for m in args.modes},
        scenarios=scenario_configs,
        summary_by_mode=by_mode,
        paired_A_vs_C=paired(runs) if {'continuous', 'hybrid'} <= set(args.modes) else None,
        paired_C_vs_C2=paired(runs, 'hybrid', 'hybrid_fallback') if {'hybrid', 'hybrid_fallback'} <= set(args.modes) else None,
        summary_by_category=by_category,
        summary_by_scenario=by_scenario,
        runs=runs,
    )
    if groups:
        payload['extra_scenarios'] = sorted(set(scenarios) - set(SCENARIOS))
        if extra is not True:       # explicit group names (the bare flag keeps the original payload)
            payload['extra_scenario_groups'] = list(groups)
    if overrides:    # opt-in estimator variant (absent by default); also in every scenario config record
        payload['estimator_overrides'] = dict(overrides)
    if plant_global or control_global:
        # Opt-in matrix-wide variant (absent by default). 'controllers' then holds the effective settings of the
        # run modes (config_for(mode, **controller_overrides)); 'plant' stays the default provenance table and
        # 'plant_effective' is the table with the global plant overrides (before scenario-own overrides, which
        # every scenario record lists with the global names it supersedes).
        payload['plant_overrides'] = {k: list(v) if isinstance(v, tuple) else v for k, v in plant_global}
        payload['controller_overrides'] = dict(control_global)
        payload['override_precedence'] = 'scenario_own_overrides_win'
        payload['plant_effective'] = provenance(replace(FieldResponseParams(), **dict(plant_global)))
    if pairs:
        payload['paired_extra'] = {'%s:%s' % (a, b): paired(runs, a, b) for a, b in pairs}
    Path(args.out).write_text(json.dumps(payload, indent=1, ensure_ascii=False, allow_nan=False, default=float) + '\n',
                              encoding='utf-8', newline='\n')
    print(json.dumps(dict(by_mode={k: {x: v[x] for x in ('runs', 'completed', 'acceptable', 'collisions',
                                                          'left_corridor_runs', 'intermediate_stops_per_run',
                                                          'pivots_per_run')} for k, v in by_mode.items()},
                          paired=payload['paired_A_vs_C'] and {k: v for k, v in payload['paired_A_vs_C'].items()
                                                               if k != 'discordant'}), indent=1, default=float))


if __name__ == '__main__':
    main()
