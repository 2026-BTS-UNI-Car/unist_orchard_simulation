"""evaluate --plant-override / --control-override (opt-in matrix-wide SIM variants; default OFF).

Contracts: values are typed by the field default; unknown names, 'mode', duplicates and non-finite values are
refused before any episode runs; a scenario's OWN override wins for the names it sets; without the flags the
episode configurations and the payload are unchanged; the random streams never depend on the overrides.
"""
from dataclasses import replace
import json

import pytest

from closed_loop.evaluate import (CONTROL_FIELDS, PLANT_FIELDS, apply_overrides, main as evaluate_main,
                                  parse_override, parse_overrides)
from closed_loop.harness import PURPOSES, plant_params, stream_seed
from closed_loop.controllers import config_for
from closed_loop.matrix import episodes

FIT = (('speed_at_90_m_s', .8), ('forward_deadband_pct', 5.), ('arc_yaw_deg_s_per_pct_diff', .15),
       ('drive_vibration_rms_m_s2', .7), ('pivot_min_duty_pct', 55.))
W1 = (('base_duty_pct', 55), ('reduce_duty_pct', 45), ('max_differential_pct', 20),
      ('arc_yaw_deg_s_per_pct_diff', .15))


def test_values_follow_the_field_type():
    assert parse_override('speed_at_90_m_s=0.80', PLANT_FIELDS, 'plant') == ('speed_at_90_m_s', .8)
    assert parse_override('pivot_breakaway_delay_s=0.2,0.6', PLANT_FIELDS, 'plant') == \
        ('pivot_breakaway_delay_s', (.2, .6))
    assert parse_override('base_duty_pct=55', CONTROL_FIELDS, 'controller') == ('base_duty_pct', 55)
    assert isinstance(parse_override('base_duty_pct=55', CONTROL_FIELDS, 'controller')[1], int)
    assert parse_override('side_clearance_check=true', CONTROL_FIELDS, 'controller') == \
        ('side_clearance_check', True)
    assert parse_override(' lookahead_m = 2.5 ', CONTROL_FIELDS, 'controller') == ('lookahead_m', 2.5)


@pytest.mark.parametrize('text,fields_', [
    ('speed_at_90_m_s', PLANT_FIELDS), ('speed_at_90_m_s=', PLANT_FIELDS), ('nope=1', PLANT_FIELDS),
    ('speed_at_90_m_s=nan', PLANT_FIELDS), ('speed_at_90_m_s=inf', PLANT_FIELDS),
    ('pivot_rate_events_deg_s=5,x', PLANT_FIELDS), ('mode=hybrid', CONTROL_FIELDS),
    ('base_duty_pct=55.5', CONTROL_FIELDS), ('side_clearance_check=1', CONTROL_FIELDS),
    ('lookahead_m=inf', CONTROL_FIELDS)])
def test_bad_values_are_refused(text, fields_):
    with pytest.raises(ValueError):
        parse_override(text, fields_, 'x')


def test_duplicates_are_refused():
    with pytest.raises(ValueError):
        parse_overrides(['base_duty_pct=55', 'base_duty_pct=60'], CONTROL_FIELDS, 'controller')


def test_no_overrides_leave_the_configurations_unchanged():
    configs = episodes(seeds=(0, 13), modes=('continuous', 'hybrid_side'), extra=('side_clearance',
                                                                                 'headland_clutter'))
    same = apply_overrides(configs)
    assert same == configs and all(a is b for a, b in zip(same, configs))


def test_scenario_own_overrides_win_and_draws_are_shared():
    configs = episodes(seeds=(0, 12), modes=('continuous_side',))
    merged = apply_overrides(configs, FIT, W1)
    by_name = {}
    for base, fit in zip(configs, merged):
        assert replace(fit, plant_overrides=base.plant_overrides,
                       controller_overrides=base.controller_overrides) == base
        assert all(stream_seed(base, p) == stream_seed(fit, p) for p in PURPOSES)
        by_name[fit.name] = fit
    # Scene-own deadband 78 % (and 60 / 35 %) and vibration 3.0 stay; the other global values still apply.
    p = plant_params(by_name['forward_deadband_above_base'])
    assert p.forward_deadband_pct == 78. and p.speed_at_90_m_s == .8 and p.pivot_min_duty_pct == 55.
    assert plant_params(by_name['low_duty_weak_unknown']).forward_deadband_pct == 60.
    assert plant_params(by_name['low_duty_strong_unknown']).forward_deadband_pct == 35.
    assert plant_params(by_name['imu_noise_vibration']).drive_vibration_rms_m_s2 == 3.
    assert plant_params(by_name['steering_ineffective']).arc_yaw_deg_s_per_pct_diff == .02
    nominal = plant_params(by_name['nominal_open_end'])
    assert (nominal.forward_deadband_pct, nominal.arc_yaw_deg_s_per_pct_diff, nominal.drive_vibration_rms_m_s2) == \
        (5., .15, .7)
    # Plant overrides never stack: one entry per name.
    names = [k for k, _ in by_name['forward_deadband_above_base'].plant_overrides]
    assert len(names) == len(set(names))
    control = config_for('continuous_side', **dict(by_name['nominal_open_end'].controller_overrides))
    assert (control.base_duty_pct, control.reduce_duty_pct, control.max_differential_pct,
            control.arc_yaw_deg_s_per_pct_diff, control.side_clearance_check) == (55, 45, 20, .15, True)


def test_scenario_controller_override_wins():
    (config,) = episodes(seeds=(0,), modes=('continuous',), names=['nominal_open_end'])
    config = replace(config, controller_overrides=(('base_duty_pct', 70),))
    (merged,) = apply_overrides([config], (), W1)
    assert dict(merged.controller_overrides)['base_duty_pct'] == 70
    assert [k for k, _ in merged.controller_overrides].count('base_duty_pct') == 1


@pytest.mark.parametrize('argv', [
    ['--control-override', 'base_duty_pct=90'],                          # 90 + 25 > 100: no headroom
    ['--control-override', 'mode=stop_turn'],
    ['--plant-override', 'forward_deadband_pct=95'],                     # FieldResponseParams speed curve
    ['--plant-override', 'speed_at_90_m_s=0.95'],                        # >= speed_at_100
    ['--plant-override', 'speed_at_90_m_s=0.8', '--plant-override', 'speed_at_90_m_s=0.7']])
def test_evaluate_refuses_invalid_overrides_before_running(tmp_path, argv):
    with pytest.raises(SystemExit):
        evaluate_main(['--out', str(tmp_path / 'x.json'), '--seeds', '0', '--modes', 'continuous',
                       '--scenarios', 'nominal_open_end'] + argv)
    assert not (tmp_path / 'x.json').exists()


def test_evaluate_records_global_and_scenario_overrides(tmp_path):
    # forward_deadband_above_base: W1 (base 55) under its own 78 % deadband cannot move -> short episode.
    argv = ['--seeds', '0', '--modes', 'continuous', '--workers', '1', '--scenarios', 'forward_deadband_above_base']
    flags = [x for k, v in FIT for x in ('--plant-override', '%s=%s' % (k, v))]
    flags += [x for k, v in W1 for x in ('--control-override', '%s=%s' % (k, v))]
    evaluate_main(['--out', str(tmp_path / 'fit.json')] + argv + flags)
    fit = json.loads((tmp_path / 'fit.json').read_text(encoding='utf-8'))
    assert fit['plant_overrides'] == dict(FIT) and fit['controller_overrides'] == dict(W1)
    assert fit['override_precedence'] == 'scenario_own_overrides_win'
    assert fit['plant_effective']['parameters']['forward_deadband_pct']['value'] == 5.
    assert fit['plant']['parameters']['forward_deadband_pct']['value'] == 50.      # default table unchanged
    assert fit['controllers'] == {'A_continuous': fit['controllers']['A_continuous']}
    assert fit['controllers']['A_continuous']['base_duty_pct'] == 55
    scene = fit['scenarios']['forward_deadband_above_base']
    assert scene['scenario_plant_overrides'] == {'forward_deadband_pct': 78.}
    assert scene['superseded_global_plant_overrides'] == ['forward_deadband_pct']
    assert scene['superseded_global_controller_overrides'] == []
    assert dict((k, v) for k, v in scene['config']['plant_overrides'])['forward_deadband_pct'] == 78.
    assert scene['config']['plant_provenance_overrides']['speed_at_90_m_s']['value'] == .8
    (run,) = fit['runs']
    assert run['final_reason'] == 'drive_no_response' and run['acceptable']
    # Without the flags: no override keys, the default controller table.
    evaluate_main(['--out', str(tmp_path / 'base.json')] + argv)
    base = json.loads((tmp_path / 'base.json').read_text(encoding='utf-8'))
    for key in ('plant_overrides', 'controller_overrides', 'override_precedence', 'plant_effective'):
        assert key not in base
    assert 'scenario_plant_overrides' not in base['scenarios']['forward_deadband_above_base']
    assert base['controllers']['A_continuous']['base_duty_pct'] == 75 and 'C_hybrid' in base['controllers']
