"""Profile schema v3 (front_steer_v0_2_placeholder), the steer_ext startup block and the native belt / torque terms
(ST-10, ST-11; docs/steering/STEER_EXT_MODEL_KO.md, STEER_PROFILE_V02_KO.md).

All values are placeholders; these tests check the model and the loader rules, not field behaviour.
Run from sim/ubts: python -B -m pytest -q -p no:cacheprovider test_steering_profile_v3.py
(the native tests need a C++ compiler and UBTS_STEER_BUILD_DIR outside the work tree, or they build in a temp folder).
"""
from __future__ import annotations

import copy
import math
import os
from pathlib import Path

import pytest

from steering import backend_api as ba
from steering import vehicle_profile as vp
from steering.actuator_link import NativeSteeringBackend
from steering.native import build as native_build

PROFILES = Path(vp.__file__).resolve().parent / 'profiles'
V02 = PROFILES / 'front_steer_v0_2_placeholder.yaml'
V01 = PROFILES / 'front_steer_v0_1_placeholder.yaml'
FIXED = {'steer_zero_offset_deg': 0.0, 'pot_cal_residual_deg': 0.0}


@pytest.fixture(scope='module')
def v02():
    return vp.load_profile(V02)


@pytest.fixture(scope='module')
def v01():
    return vp.load_profile(V01)


# --- loader --------------------------------------------------------------------------------------------------------------
def test_v02_loads_as_schema_v3(v02):
    assert v02.schema == vp.SCHEMA_V3 and v02.schema_version == 3
    assert v02.names() == list(vp.PARAMETER_NAMES_V3)
    assert v02.warnings() == []
    assert v02.provenance_grade() == 'placeholder'
    assert v02.value('belt_tooth_step_deg') == pytest.approx(18.0)
    assert v02.value('pot_travel_deg') == pytest.approx(54.19, abs=.01)
    assert v02.value('pot_cal_half_width_counts') == pytest.approx(95.18, abs=.01)
    for name in ('belt_type', 'belt_pulley_teeth_pot', 'belt_pot_play_deg', 'steer_limit_switch',
                 'steer_supply_shared_with_drive'):
        assert v02.provenance(name) == 'SPEC_PENDING'


def test_v2_files_load_unchanged(v01):
    assert v01.schema == vp.SCHEMA and v01.schema_version == 2
    assert v01.names() == list(vp.PARAMETER_NAMES)
    assert not v01.has('belt_type') and v01.has('wheelbase_m')
    assert ba.ext_startup_block(v01) is None
    assert 'steer_ext' not in ba.native_startup_config(v01, 1)


def test_v02_consistency_changes_against_v01(v01, v02):
    """The values the consistency revision changed (R18 decision 3) and nothing else of the v2 parameters."""
    changed = {name for name in vp.PARAMETER_NAMES if v01.value(name) != v02.value(name)}
    assert changed == {'servo_min_pwm_pct', 'polarity_fault', 'driver_stuck_fault', 'thermal_proxy',
                       'host_track_mismatch'}
    assert v02.value('servo_min_pwm_pct') - v02.value('steer_min_pwm_standstill_pct') >= 10.
    assert v02.param('pot_to_steer_ratio')['range'] == [0.5, 5.0]


def _ratio(profile, ratio, **kwargs):
    return profile.with_overrides({'pot_to_steer_ratio': ratio}, label='belt_ratio', **kwargs)


@pytest.mark.parametrize('ratio', [0.55, 1.0, 2.0, 4.0])
def test_belt_ratios_inside_the_pot_window_load(v02, ratio):
    view = _ratio(v02, ratio)
    assert view.value('pot_cal_half_width_counts') >= vp.MARK_HALF_WIDTH_MIN
    assert view.value('belt_tooth_step_deg') == pytest.approx(360. / 20. / ratio)


@pytest.mark.parametrize('ratio,code', [(0.3, 'out_of_range'), (5.5, 'out_of_range'),
                                        (0.5, 'pot_cal_half_width_small'), (4.5, 'pot_range_exceeded'),
                                        (5.0, 'pot_range_exceeded')])
def test_belt_ratios_refused(v02, ratio, code):
    """Below 1 is allowed by the range (design v2 V2-6.5); 0.5 with the 28 deg placeholder limits gives marks 47.6
    counts from the centre (< 50), above about 4.2 the end stops leave the calibration window."""
    with pytest.raises(vp.ProfileError) as info:
        _ratio(v02, ratio)
    assert info.value.code == code


def test_out_of_range_override_turns_the_pot_checks_into_warnings(v02):
    view = _ratio(v02, 5.0, allow_out_of_range=True)
    codes = {w['code'] for w in view.warnings()}
    assert 'pot_end_stops_outside_cal_window' in codes


def _v02_text(**replacements):
    text = V02.read_bytes().decode('utf-8')
    for old, new in replacements.items():
        assert text.count(old) == 1, old
        text = text.replace(old, new)
    return text.encode('utf-8')


def test_file_with_a_ratio_outside_the_window_is_refused():
    data = _v02_text(**{'pot_to_steer_ratio: {value: 1.0,': 'pot_to_steer_ratio: {value: 5.0,'})
    with pytest.raises(vp.ProfileError) as info:
        vp.parse_profile(data)
    assert info.value.code == 'pot_range_exceeded'


def test_limit_switches_are_refused_until_modelled():
    data = _v02_text(**{'steer_limit_switch: {value: "none",': 'steer_limit_switch: {value: "both",'})
    with pytest.raises(vp.ProfileError) as info:
        vp.parse_profile(data)
    assert info.value.code == 'not_modelled'


def test_tooth_skips_need_a_timing_belt(v02):
    with pytest.raises(vp.ProfileError):
        v02.with_overrides({'belt_type': 'flat', 'belt_skip_per_10000deg': 1.0}, label='t')
    assert v02.with_overrides({'belt_type': 'flat', 'belt_creep_sd_deg_per_100deg': .2}, label='t')


def test_schema_parameter_sets_are_exact():
    v3_only = _v02_text(**{'schema: ubts.vehicle_profile.v3': 'schema: ubts.vehicle_profile.v2'})
    with pytest.raises(vp.ProfileError) as info:
        vp.parse_profile(v3_only)
    assert info.value.code == 'parameter_unknown'
    v1 = V01.read_bytes().decode('utf-8').replace('schema: ubts.vehicle_profile.v2', 'schema: ubts.vehicle_profile.v3')
    with pytest.raises(vp.ProfileError) as info:
        vp.parse_profile(v1.encode('utf-8'))
    assert info.value.code == 'parameter_missing'


def test_identify_accepts_the_new_questions():
    assert vp.IDENTIFY_TEST_PATTERN.match('Q29') and vp.IDENTIFY_TEST_PATTERN.match('Q22')
    assert not vp.IDENTIFY_TEST_PATTERN.match('Q30')


# --- startup block and injections ------------------------------------------------------------------------------------------
def test_ext_block_values(v02):
    ext = ba.ext_startup_block(v02)
    assert ext['belt_play_rad'] == pytest.approx(math.radians(.3))
    assert ext['belt_tooth_step_rad'] == pytest.approx(math.radians(18.))
    assert ext['supply_rate_scale'] == 1.0 and ext['backdrive_coast_gain'] == 3.0
    assert ext['impact_rate_per_m'] == 0. and ext['push_sd_nm'] == 0.
    low = v02.with_overrides({'battery_voltage_scale': .9}, label='battery')
    assert ba.ext_startup_block(low)['supply_rate_scale'] == .9
    alone = low.with_overrides({'steer_supply_shared_with_drive': False}, label='own_supply')
    assert ba.ext_startup_block(alone)['supply_rate_scale'] == 1.0
    assert ba.native_startup_config(v02, 1)['steer_ext'] == ext


def test_belt_injection_helpers(v02, v01):
    skip = ba.belt_skip_injection(v02, 2.0, teeth=-1)
    assert skip == dict(kind='pot_slip', at_s=2.0, params=dict(offset_rad=math.radians(-18.)))
    assert ba.belt_break_injection(1.5) == dict(kind='pot_stuck', at_s=1.5, params={})
    with pytest.raises(ValueError):
        ba.belt_skip_injection(v01, 1.0)
    with pytest.raises(ValueError):
        ba.belt_skip_injection(v02, 1.0, teeth=0)
    ok = ba.validate_injections([dict(kind='steer_torque', at_s=1., params=dict(torque_nm=2.5, duration_s=.5))])
    assert ok[0]['params'] == dict(torque_nm=2.5, duration_s=.5)
    for bad in (dict(torque_nm=2.5, duration_s=0.), dict(torque_nm=2000., duration_s=1.), dict(torque_nm=1.)):
        with pytest.raises(ValueError):
            ba.validate_injections([dict(kind='steer_torque', at_s=1., params=bad)])


# --- native model ---------------------------------------------------------------------------------------------------------
@pytest.fixture(scope='module')
def native(tmp_path_factory):
    try:
        native_build.find_compiler()
    except native_build.NativeBuildError as exc:
        pytest.skip('no C++ compiler for the native steering model: %s' % exc)
    build_dir = os.environ.get(native_build.ENV_BUILD_DIR) or str(tmp_path_factory.mktemp('steer_native_build'))
    return native_build.ensure_built(build_dir=build_dir)


def _backend(native, profile, mode='direct_pwm', seed=3, injections=()):
    return NativeSteeringBackend(profile, seed, mode=mode, injections=injections, executable=native.path)


def _run(backend, n, dt=.01, **kw):
    kw.setdefault('mode', backend.mode)
    out = []
    for _ in range(n):
        out.append(backend.step(ba.SteerStepInput(dt_s=dt, **kw)))
    return out


def _deg(x):
    return math.degrees(x)


def _neutral(v02):
    """v0.2 with every steer_ext term off and the v0.1 values of the consistency revision."""
    return v02.with_overrides({'belt_pot_play_deg': 0.0}, label='neutral_ext')


def test_neutral_ext_block_is_bit_identical(native, v01):
    """A startup line with a steer_ext block whose terms are all off gives the same response bytes as without the
    block (the v2 profiles keep their results)."""
    import subprocess
    cfg = ba.native_startup_config(v01, 7)
    ext = dict(belt_play_rad=0., belt_creep_sd_rad_per_sqrt_rad=0., belt_skip_per_rad=0., belt_tooth_step_rad=0.,
               supply_rate_scale=1., backdrive_coast_gain=1., self_lock_breakaway_nm=0., impact_rate_per_m=0.,
               impact_torque_sd_nm=0., impact_duration_s=0., push_sd_nm=0., push_corr_length_m=1.)
    frames = [ba.encode_reset_request(1, 0.0)]
    seq = 2
    for k in range(300):
        pwm = .6 if k < 100 else (-.5 if k < 200 else 0.)
        frames.append(ba.encode_step_request(seq, ba.SteerStepInput(dt_s=.01, steer_pwm=pwm, vehicle_speed_m_s=.4,
                                                                    vehicle_stopped=False)))
        seq += 1
    body = b''.join(frames)
    outs = []
    for config in (cfg, dict(cfg, steer_ext=ext)):
        proc = subprocess.run([native.path], input=ba.startup_line(config) + body, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, timeout=60)
        assert proc.returncode == 0, proc.stderr
        outs.append(proc.stdout)
    assert outs[0] == outs[1]   # READY (same build id), the reset and every step response, byte for byte


def test_belt_play_lags_the_pot_on_reversal(native, v02):
    view = v02.with_overrides(dict(FIXED, belt_pot_play_deg=2.0, steer_backlash_deg=0.0, pot_noise_sd_counts=0.0,
                                   pot_supply_sag_per_pwm=0.0),
                              label='belt_play', allow_out_of_range=True)
    b = _backend(native, view)
    move = dict(vehicle_speed_m_s=0., vehicle_stopped=True)   # standing: no release drift while settling
    try:
        b.reset(0.0)
        _run(b, 50, steer_pwm=.7, **move)
        top = _run(b, 40, steer_pwm=0., **move)[-1]          # settled after the lag
        back = None
        for _ in range(100):                                  # reverse until the motor side went back 1.2 deg
            back = _run(b, 1, dt=.002, steer_pwm=-.7, **move)[-1]
            if _deg(top.motor_side_angle_rad - back.motor_side_angle_rad) >= 1.2:
                break
        assert abs(back.pot_raw - top.pot_raw) == 0            # the pot shaft has not moved yet (play 2 deg)
        while _deg(top.motor_side_angle_rad - back.motor_side_angle_rad) < 4.0:
            back = _run(b, 1, dt=.002, steer_pwm=-.7, **move)[-1]
        # 4 deg back, 2 deg of it in the play: about 2 deg x 3.79 counts / deg
        assert top.pot_raw - back.pot_raw == pytest.approx(2.0 * 1024. / 270., abs=1.5)
    finally:
        b.close()


def test_tooth_skips_move_the_pot_zero_by_whole_teeth(native, v02):
    view = v02.with_overrides(dict(FIXED, belt_pot_play_deg=0.0, belt_skip_per_10000deg=5.0,
                                   pot_noise_sd_counts=1.0), label='skips')
    view = view.with_overrides({'belt_pulley_teeth_pot': 60}, label='small_step')   # 6 deg per tooth
    b = _backend(native, view, seed=11)
    try:
        b.reset(0.0)
        # sweep back and forth about 40 deg x 30 = 1200 deg of travel: expect about 0.6 skips; repeat until one
        skips = 0
        for k in range(120):
            outs = _run(b, 60, steer_pwm=(.9 if k % 2 == 0 else -.9))
            raw_expected = round(512 + 1024. / math.radians(270.) * outs[-1].motor_side_angle_rad)
            offset_counts = outs[-1].pot_raw - raw_expected
            skips = round(offset_counts / (6.0 * 1024. / 270.))
            if skips:
                break
        assert skips != 0
        assert abs(offset_counts - skips * 6.0 * 1024. / 270.) < 6   # whole teeth, noise and rounding aside
    finally:
        b.close()


def _torque_case(native, profile, torque, mode='direct_pwm', moving=True, en=True, pwm=0., n=150, seed=5):
    inj = [dict(kind='steer_torque', at_s=.2, params=dict(torque_nm=torque, duration_s=60.))]
    b = _backend(native, profile, mode=mode, seed=seed, injections=inj)
    try:
        b.reset(0.0)
        outs = _run(b, n, steer_pwm=pwm, steer_en=en, vehicle_speed_m_s=.4 if moving else 0.,
                    vehicle_stopped=not moving)
    finally:
        b.close()
    return outs


def test_torque_below_the_friction_only_closes_the_play(native, v02):
    view = v02.with_overrides(dict(FIXED, steer_backlash_wander={'sd_deg': 0.0, 'corr_length_m': 5.0}),
                              label='still_play')
    outs = _torque_case(native, view, 1.0)      # d = 0.2 < moving start duty 0.25
    assert _deg(outs[-1].motor_side_angle_rad) == pytest.approx(0.0, abs=1e-9)
    assert _deg(outs[-1].delta_eq_rad) == pytest.approx(1.0, abs=.02)   # half the 2 deg backlash


def test_torque_back_drives_brake_and_coast(native, v02):
    view = v02.with_overrides(FIXED, label='fixed')
    brake = _torque_case(native, view, 2.5, en=True)   # d = 0.5: (0.5 - 0.25) / 0.75 x 30 = 10 deg/s
    coast = _torque_case(native, view, 2.5, en=False)  # x 3
    t = 1.3                                             # 1.1 s after the onset, before the end stop
    k = int(t / .01) - 1
    brake_rate = _deg(brake[k].motor_side_angle_rad - brake[k - 50].motor_side_angle_rad) / .5
    coast_rate = _deg(coast[k].motor_side_angle_rad - coast[k - 20].motor_side_angle_rad) / .2
    assert brake_rate == pytest.approx(10.0, rel=.05)
    assert coast[-1].delta_eq_rad >= brake[-1].delta_eq_rad
    assert coast_rate > 2.0 * brake_rate or 'END_STOP_LEFT' in coast[k].flags()


def test_drive_against_the_torque_holds(native, v02):
    view = v02.with_overrides(FIXED, label='fixed')
    held = _torque_case(native, view, 2.5, pwm=-.45)   # u_eff = -0.45 + 0.5 = 0.05 < 0.25: static friction
    assert abs(_deg(held[-1].motor_side_angle_rad - held[70].motor_side_angle_rad)) < .01   # lag decayed
    pushed = _torque_case(native, view, 5.0, pwm=-.45)  # u_eff = 0.55: pushed toward the left
    assert pushed[-1].motor_side_angle_rad > pushed[30].motor_side_angle_rad + math.radians(3.)


def test_self_locking_holds_below_the_breakaway(native, v02):
    view = v02.with_overrides(dict(FIXED, steer_self_locking=True), label='self_lock')
    below = _torque_case(native, view, 6.0, en=False)   # 6 < 8 N m breakaway
    above = _torque_case(native, view, 12.0, en=False)  # (12 - 8) / 5 = 0.8
    assert below[-1].motor_side_angle_rad == pytest.approx(0.0, abs=1e-12)
    assert above[-1].motor_side_angle_rad > math.radians(5.)


def test_torque_needs_l1_and_a_stall_torque(native, v02):
    import subprocess
    cfg = ba.native_startup_config(v02.with_overrides(FIXED, label='f'), 1, mode='test_servo',
                                   injections=[dict(kind='steer_torque', at_s=0., params=dict(torque_nm=1.,
                                                                                               duration_s=1.))])
    for edit in (lambda c: c['actuator'].update(level='L0'), lambda c: c['actuator'].update(stall_torque_nm=0.)):
        bad = copy.deepcopy(cfg)
        edit(bad)
        proc = subprocess.run([native.path], input=ba.startup_line(bad), stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, timeout=30)
        assert proc.returncode == ba.EXIT_BAD_STARTUP, proc.stderr


def test_terrain_impacts_only_while_moving_and_deterministic(native, v02):
    view = v02.with_overrides(dict(FIXED, steer_terrain_impacts={'rate_per_m': 2.0, 'torque_sd_nm': 4.0,
                                                                 'duration_s': .1}), label='impacts')

    def run(moving, seed):
        b = _backend(native, view, mode='test_servo', seed=seed)
        try:
            b.reset(0.0)
            return [o.delta_eq_rad for o in _run(b, 500, target_delta_eq_rad=0., vehicle_speed_m_s=.4 if moving else 0.,
                                                  vehicle_stopped=not moving)]
        finally:
            b.close()
    still = run(False, 4)
    assert max(abs(x) for x in still) < math.radians(.01)
    a, b2 = run(True, 4), run(True, 4)
    assert a == b2
    assert max(abs(x) for x in a) > math.radians(1.)          # impacts move the wheels (play and beyond)
    assert run(True, 5) != a


def test_supply_rate_scale(native, v02):
    view = v02.with_overrides(dict(FIXED, battery_voltage_scale=.9), label='battery')
    full = v02.with_overrides(FIXED, label='fixed')
    rates = []
    for prof in (full, view):
        b = _backend(native, prof)
        try:
            b.reset(0.0)
            outs = _run(b, 60, steer_pwm=1.0, vehicle_speed_m_s=.4, vehicle_stopped=False)
            rates.append(_deg(outs[-1].motor_side_angle_rad - outs[-21].motor_side_angle_rad) / .2)
        finally:
            b.close()
    assert rates[0] == pytest.approx(30.0, rel=.02)
    assert rates[1] == pytest.approx(27.0, rel=.02)
