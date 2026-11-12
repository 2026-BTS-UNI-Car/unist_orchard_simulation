"""steering.steer_hold_checks (SIM_ACCEPTANCE amendments 40-42): case tables, the thermal window, the judgement rules
and one short T1 rig episode. Placeholder-based SIM results; the judgement runs themselves are not run here.

Run from sim/ubts: python -B -m pytest -q -p no:cacheprovider test_steering_hold_checks.py
"""
from __future__ import annotations

import os

import pytest

from steering import steer_hold_checks as shc
from steering import vehicle_profile as vp
from steering.native import build as native_build


def test_case_tables_follow_the_registration():
    v02 = vp.load_profile(shc.PROFILE_V02)
    dist = shc.cases_of('disturbance', v02)
    assert sorted(dist) == ['D1', 'D2', 'D3', 'D4', 'D5', 'D6', 'R1', 'R2', 'R3', 'R4']
    assert {n: c['group'] for n, c in dist.items() if c['group'] != 'report'} == dict(
        D1='within', D2='within', D3='within', D4='within', D5='above', D6='above')
    assert dist['D1']['model'][0] == dict(kind='steer_torque', at_s=6.0, params=dict(torque_nm=2.5, duration_s=3.0))
    assert dist['D5']['expect'] == ('STALL', 'TRACK', 'DRV_STUCK') and shc.ABOVE_DEFINED_41 == ('STALL', 'TRACK')
    assert dist['R3']['model'][1]['kind'] == 'pot_slip'
    assert 'R3' not in shc.cases_of('disturbance', vp.load_profile())      # a v2 profile has no belt model
    assert sorted(shc.cases_of('normal', v02)) == ['N1', 'N2', 'N3', 'N4', 'N5', 'N6']
    assert sorted(shc.cases_of('faults', v02)) == ['G1', 'G2', 'G3']
    assert shc.SEEDS['disturbance']['judgement'] == (1000, 1001, 1002, 1003, 1004)
    assert shc.SEEDS['normal']['judgement'] == tuple(range(1100, 1110))


def test_segment_commands():
    n4 = shc.cases_of('normal', vp.load_profile(shc.PROFILE_V02))['N4']['cmd']
    assert n4(1.0) == (0, 0.) and n4(2.1) == (0, 5.) and n4(7.1) == (0, -5.) and n4(20.) == (0, 0.)


def test_hot_window():
    changes = [(0, 0), (100, 220), (1100, 0), (40000, 210), (40500, 0)]
    assert shc._hot_window_ms(changes, 204, 60000) == 1000
    assert shc._hot_window_ms(changes, 215, 60000) == 1000
    assert shc._hot_window_ms(changes, 230, 60000) == 0
    assert shc._hot_window_ms([(0, 255)], 204, 59999, window_ms=30000) == 30000


def _ep(case, group, **kw):
    out = dict(case=case, seed=1000, group=group, f_lines=[], latches=[], armed=True, recovery_s=.2)
    out.update(kw)
    return out


def test_check_rules():
    ok = dict(seed_kind='judgement', episodes=[
        _ep('D1', 'within'), _ep('D5', 'above', first_steer_fault=['DRV_STUCK', 6230], tick_diff=0),
        _ep('R1', 'report', f_lines=[['TRACK', 7000]])])
    passed, items = shc.check(ok)
    assert passed
    original = [i for i in items if i['group'] == 'above_original_41']
    assert original and not original[0]['passed'] and original[0]['report_only']
    for bad in (_ep('D1', 'within', recovery_s=2.5), _ep('D1', 'within', latches=['steering_not_following']),
                _ep('D1', 'within', recovery_s=None), _ep('D6', 'above', first_steer_fault=['POLARITY', 6266],
                                                          tick_diff=0),
                _ep('D6', 'above', first_steer_fault=['STALL', 6500], tick_diff=2),
                _ep('N5', 'normal', recovery_s=None, latches=['steering_not_following']),
                _ep('N2', 'normal', recovery_s=None, f_lines=[['POLARITY', 5000]])):
        passed, _ = shc.check(dict(seed_kind='judgement', episodes=[bad]))
        assert not passed, bad
    assert not shc.check(dict(seed_kind='list', episodes=[_ep('D1', 'within')]))[0]   # only judgement seeds judge


def test_check_rules_of_amendment_43():
    pol = _ep('D6', 'above', first_steer_fault=['POLARITY', 6266], tick_diff=0)
    assert not shc.check(dict(seed_kind='judgement', amendment=41, episodes=[pol]))[0]
    passed, items = shc.check(dict(seed_kind='judgement43', amendment=43, episodes=[pol]))
    assert passed and all(i['group'] != 'above_original_41' for i in items)
    soft = _ep('D6', 'above', first_steer_fault=['SOFTLIMIT', 6266], tick_diff=0)
    assert not shc.check(dict(seed_kind='judgement43', amendment=43, episodes=[soft]))[0]
    assert shc.SEEDS['disturbance']['judgement43'] == (1500, 1501, 1502, 1503, 1504)
    assert shc.SEEDS['normal']['judgement43'] == tuple(range(1300, 1310))
    assert shc.SEEDS['faults']['judgement43'] == (1400, 1401, 1402)


@pytest.fixture(scope='module')
def native(tmp_path_factory):
    try:
        native_build.find_compiler()
    except native_build.NativeBuildError as exc:
        pytest.skip('no C++ compiler for the native steering model: %s' % exc)
    build_dir = os.environ.get(native_build.ENV_BUILD_DIR) or str(tmp_path_factory.mktemp('steer_native_build'))
    return native_build.ensure_built(build_dir=build_dir)


def test_one_rig_episode(native):
    ep = shc.run_episode('disturbance', 'D3', 0, None, os.path.dirname(native.path), with_trace=True)
    assert ep['armed'] and ep['f_lines'] == [] and ep['latches'] == []
    assert ep['identity']['profile_id'] == 'front_steer_v0_2_placeholder'
    assert 0. < ep['max_steer_dev_deg'] < 3. and ep['recovery_s'] is not None
    assert len(ep['trace']['t_s']) == len(ep['trace']['truth_y_m']) > 100
