"""Steering-SIM drive values (design 3.5, 3.6): curve roles and sample points, reverse placeholder,
battery scale, G-SIM DC motor line derivation, stop modes and the stop-event table. The release stop
distance is a closed form here (A4(F) plant run is a later step). Placeholder-based SIM values only.
"""
from dataclasses import replace
import math

import pytest

from field_response import PARAMETERS, FieldResponseParams, forward_curve
from steering import drive as dr
from steering import vehicle_profile as vp

P_FIT1003 = replace(FieldResponseParams(), speed_at_90_m_s=.80, forward_deadband_pct=5.)


@pytest.fixture(scope='module')
def profile():
    return vp.load_profile()


@pytest.fixture(scope='module')
def profile_v0():
    """Profile v0 (tau_stall 1.6 N*m). The default is v0.1 (DECISIONS R14); the v0 derivation stays checked."""
    return vp.load_profile(vp.PROFILES_DIR / 'front_steer_v0_placeholder.yaml')


def test_curve_roles(profile):
    assert dr.curve_roles(profile) == {'main': 'P_fit1003', 'bracket': 'field1010'}
    assert dr.resolve_curve(profile, 'main') == 'P_fit1003' and dr.resolve_curve(profile, 'bracket') == 'field1010'
    assert dr.resolve_curve(profile, 'field1010') == 'field1010'
    with pytest.raises(ValueError):
        dr.resolve_curve(profile, 'fast')


def test_main_curve_sample_points(profile):
    """A4(F) curve part: P_fit1003 40/55/70/88 % -> 0.33/0.47/0.61/0.78 m/s within 1 %."""
    for duty, speed in ((40., .33), (55., .47), (70., .61), (88., .78)):
        assert dr.steady_speed_m_s(profile, duty) == pytest.approx(speed, rel=.01)
    assert dr.steady_speed_m_s(profile, 90.) == pytest.approx(.80)
    assert dr.steady_speed_m_s(profile, 100.) == pytest.approx(.90)
    assert dr.steady_speed_m_s(profile, 5.) == 0. and dr.steady_speed_m_s(profile, 6.) > 0.


def test_bracket_curve_sample_points(profile):
    """field1010: 90/100 % measured 0.72/0.90 m/s, 55 % -> 0.09 m/s (PREREG text), deadband 50 %."""
    for duty, speed in ((90., .72), (100., .90), (55., .09)):
        assert dr.steady_speed_m_s(profile, duty, 'bracket') == pytest.approx(speed, rel=.01)
    assert dr.steady_speed_m_s(profile, 50., 'bracket') == 0.


def test_curves_are_field_response_forward_curve(profile):
    for duty in range(0, 101, 7):
        assert dr.steady_speed_m_s(profile, duty) == forward_curve(duty, P_FIT1003)
        assert dr.steady_speed_m_s(profile, duty, 'field1010') == forward_curve(duty, FieldResponseParams())


def test_reverse_curve_placeholder(profile):
    assert profile.value('reverse_curve') == .8
    for duty in (20., 40., 88., 100.):
        assert dr.steady_speed_m_s(profile, -duty) == pytest.approx(-.8 * dr.steady_speed_m_s(profile, duty))


def test_battery_scale_scales_the_effective_duty(profile):
    assert dr.steady_speed_m_s(profile, 40., battery_scale=.9) == pytest.approx(forward_curve(36., P_FIT1003))
    assert dr.steady_speed_m_s(profile, 40., battery_scale=.9) < dr.steady_speed_m_s(profile, 40.)
    low = profile.with_overrides({'battery_voltage_scale': .85}, label='battery_low')
    assert dr.steady_speed_m_s(low, 60.) == pytest.approx(forward_curve(51., P_FIT1003))


def test_duty_checks(profile):
    for bad in (101., -100.5, float('nan'), True):
        with pytest.raises(ValueError):
            dr.steady_speed_m_s(profile, bad)
    with pytest.raises(ValueError):
        dr.steady_speed_m_s(profile, 40., battery_scale=0.)


def test_dc_line_main_curve(profile_v0):
    """Design 3.6: P_fit1003 line slope 0.00937 m/s/%, deadband 4.8 % -> r*omega0 0.937 m/s, omega0 6.7 rad/s."""
    profile = profile_v0
    line = dr.dc_motor_line(profile)
    assert line.curve == 'P_fit1003' and line.wheel_radius_m == .14
    assert line.slope_m_s_per_pct == pytest.approx(.00937, abs=1e-5)
    assert line.r_omega0_m_s == pytest.approx(.937, abs=.001)
    assert line.omega0_rad_s == pytest.approx(6.7, abs=.01)
    assert line.deadband_frac == pytest.approx(.048, abs=.0005)
    assert line.omega0_rad_s == pytest.approx(profile.value('drive_omega0_rad_s'))
    assert line.deadband_frac == pytest.approx(profile.value('drive_deadband_frac'))
    assert line.tau_stall_nm == 1.6 and line.cmd_max_nm == 1.6 and line.cmd_min_nm == -1.6
    assert line.p_gain == pytest.approx(1.6 / line.omega0_rad_s)
    assert line.coulomb_friction_nm == pytest.approx(line.deadband_frac * 1.6)
    assert line.joint_velocity_target_rad_s(.5) == pytest.approx(.5 * line.omega0_rad_s)
    assert line.torque_nm(.5, 0.) == pytest.approx(.8)
    assert line.torque_nm(.5, .5 * line.omega0_rad_s) == pytest.approx(0., abs=1e-12)
    assert line.torque_nm(0., 2.) == pytest.approx(-1.6 * 2. / line.omega0_rad_s)   # duty 0, EN on: braking torque
    assert line.torque_nm(1., -50.) == pytest.approx(1.6)                            # clipped at cmd_max
    for duty in (.40, .55, .70, .88):
        assert line.steady_speed_m_s(duty) == pytest.approx(.9372 * (duty - .04834), abs=2e-4)
    assert line.steady_speed_m_s(.04) == 0.
    with pytest.raises(ValueError):
        line.joint_velocity_target_rad_s(1.5)


def test_dc_line_bracket_curve(profile):
    line = dr.dc_motor_line(profile, 'bracket')
    assert line.r_omega0_m_s == pytest.approx(1.8)
    assert line.omega0_rad_s == pytest.approx(12.86, abs=.01)
    assert line.deadband_frac == pytest.approx(.5)


def test_dc_line_battery_scale(profile_v0):
    profile = profile_v0
    nominal = dr.dc_motor_line(profile, battery_scale=1.)
    low = dr.dc_motor_line(profile, battery_scale=.85)
    assert low.omega0_rad_s == pytest.approx(.85 * nominal.omega0_rad_s)
    assert low.tau_stall_nm == pytest.approx(.85 * 1.6)
    assert low.p_gain == pytest.approx(nominal.p_gain)
    assert low.coulomb_friction_nm == pytest.approx(nominal.coulomb_friction_nm)
    # the same effective-duty rule as the F-SIM curve: v(d; s) = r*omega0*(s*d - d0)
    assert low.steady_speed_m_s(.6) == pytest.approx(nominal.steady_speed_m_s(.6 * .85))


def test_stall_torque_placeholder_and_stability(profile_v0):
    """Design 3.6 (profile v0): tau_stall about 1.6 N*m per wheel (1.5 m/s^2, 30 kg, r 0.14 m); p_gain*dt/J about 0.13."""
    profile = profile_v0
    assert dr.vehicle_mass_kg(profile) == pytest.approx(30.)
    tau = dr.tau_stall_from_acceleration(dr.vehicle_mass_kg(profile), 1.5, .14)
    assert tau == pytest.approx(1.575) and round(tau, 1) == profile.value('drive_tau_stall_nm')
    assert profile.value('wheel_inertia_kgm2') == pytest.approx(.0147)
    assert dr.gsim_stability_number(profile) == pytest.approx(.13, abs=.01)
    assert dr.gsim_stability_number(profile, tau_stall_nm=tau) == pytest.approx(.133, abs=.001)
    with pytest.raises(ValueError):
        dr.tau_stall_from_acceleration(30., 1.5, .14, driven_wheels=0)


def _start_time_constant_s(profile):
    """G-SIM DC line start time constant T = r^2 * omega0 * m_eff / (n * tau_stall), m_eff = m + n * J / r^2."""
    line = dr.dc_motor_line(profile)
    r, n = line.wheel_radius_m, 4
    m_eff = dr.vehicle_mass_kg(profile) + n * profile.value('wheel_inertia_kgm2') / r ** 2
    return r * r * line.omega0_rad_s * m_eff / (n * line.tau_stall_nm)


def test_stall_torque_v0_1_matches_the_fast_sim_time_constant(profile, profile_v0):
    """DECISIONS R14: v0.1 changes only drive_tau_stall_nm (1.6 -> 4.4 N*m) so that the G-SIM start time constant
    is about the F-SIM drive_accel_tau_s 0.25 s (an unmeasured start-delay placeholder; the F-SIM release stop
    distance comes from the measured stop_release, DECISIONS R17)."""
    assert profile.profile_id == vp.DEFAULT_PROFILE_ID == 'front_steer_v0_1_placeholder'
    assert profile.value('drive_tau_stall_nm') == 4.4 and profile_v0.value('drive_tau_stall_nm') == 1.6
    assert profile.provenance('drive_tau_stall_nm') == 'SPEC_PENDING'
    rows = {r['name']: r for r in profile.provenance_table()}
    rows_v0 = {r['name']: r for r in profile_v0.provenance_table()}
    assert sorted(rows) == sorted(rows_v0)
    assert [k for k in rows if rows[k] != rows_v0[k]] == ['drive_tau_stall_nm']
    assert _start_time_constant_s(profile) == pytest.approx(profile.value('drive_accel_tau_s'), rel=.05)
    assert _start_time_constant_s(profile_v0) == pytest.approx(.68, abs=.02)
    assert dr.gsim_stability_number(profile) == pytest.approx(.3726, abs=.001)
    assert dr.gsim_stability_number(profile) < 1.


@pytest.mark.parametrize('wheels, radius', [('front2', .10), ('rear2', .16), ('4wd_lr_pairs', .16)])
def test_wheel_inertia_is_the_driven_wheel(profile, wheels, radius):
    """F7: J and the DC line use the same (driven) wheel radius; front2 drives the front wheels."""
    view = profile.with_overrides({'drive_wheels': wheels, 'wheel_radius_front_m': .10, 'wheel_radius_rear_m': .16},
                                  label='driven_wheel')
    line = dr.dc_motor_line(view)
    assert line.wheel_radius_m == radius
    assert view.value('wheel_inertia_kgm2') == pytest.approx(.5 * view.value('wheel_mass_kg') * radius ** 2)
    assert dr.gsim_stability_number(view) == pytest.approx(line.p_gain * (1. / 120.) / (.75 * radius ** 2))


def test_stop_modes(profile):
    modes = dr.stop_modes(profile)
    assert list(modes) == ['release', 'brake', 'coast']
    release, brake, coast = modes['release'], modes['brake'], modes['coast']
    assert (release.onset_s, release.decel_median_m_s2, release.decel_log_sd) == (.25, 1.3, .15)
    assert (brake.onset_s, brake.decel_median_m_s2, brake.decel_log_sd) == (.10, 2., 0.)
    assert (coast.onset_s, coast.decel_median_m_s2, coast.decel_log_sd) == (.05, .6, 0.)
    assert brake.decel_median_range_m_s2 == (1.3, 4.) and coast.decel_median_range_m_s2 == (.3, 1.3)
    assert release.provenance == 'MEASURED_20261010_OLD_DRIVETRAIN'
    assert brake.provenance == coast.provenance == 'SIM_ASSUMED'
    # old-drivetrain release target: 0.56..0.60 m from 0.90..0.95 m/s (closed form, no ramp fade before the onset)
    for speed in (.925, .95):
        assert .555 <= release.distance_m(speed) <= .60
    for speed in (.2, .4, .9):
        assert brake.distance_m(speed) < min(release.distance_m(speed), coast.distance_m(speed))
        assert brake.time_s(speed) < min(release.time_s(speed), coast.time_s(speed))
    assert release.decel_m_s2(0.) == pytest.approx(1.3)
    assert release.decel_m_s2(1.) == pytest.approx(1.3 * math.exp(.15))
    assert brake.decel_m_s2(2.) == pytest.approx(2.)
    assert brake.time_s(.4) == pytest.approx(.30)
    assert brake.distance_m(.4, decel_m_s2=4.) == pytest.approx(.04 + .02)
    with pytest.raises(ValueError):
        brake.distance_m(.4, decel_m_s2=0.)


def test_stop_event_table(profile):
    assert dr.drive_stop_mode(profile, 'operator_stop') == 'brake'
    assert dr.drive_stop_mode(profile, 'disarm') == 'release'
    assert dr.drive_stop_mode(profile, 'throttle_zero') == 'release'
    assert dr.drive_stop_mode(profile, 'link_watchdog') == 'brake'
    assert dr.drive_stop_mode(profile, 'thermal') == 'release'
    assert dr.drive_stop_mode(profile, 'mcu_reset') == 'coast' and dr.drive_stop_mode(profile, 'power_cut') == 'coast'
    assert dr.drive_stop_mode(profile, 'ttl_expiry') == profile.value('ttl_expiry_drive_mode') == 'brake'
    release_ttl = profile.with_overrides({'ttl_expiry_drive_mode': 'release'}, label='ttl_release')
    assert dr.drive_stop_mode(release_ttl, 'ttl_expiry') == 'release'
    assert set(dr.STOP_EVENT_DRIVE_MODE.values()) - {None} <= set(dr.STOP_MODE_NAMES)
    with pytest.raises(ValueError):
        dr.drive_stop_mode(profile, 'meteor')


def test_mcu_ramp_matches_field_response(profile):
    ramp = dr.mcu_ramp(profile)
    assert ramp.pwm_per_s == 255 and ramp.pct_per_s == pytest.approx(100.)
    assert ramp.reversal_dwell_s == pytest.approx(.12)
    assert ramp.pct_per_s == pytest.approx(PARAMETERS['mcu_ramp_pct_per_s'][0])
    assert ramp.reversal_dwell_s == pytest.approx(PARAMETERS['mcu_reversal_wait_s'][0])


def test_profile_copies_match_field_response(profile):
    assert dr.check_profile_against_field_response(profile) == []
    changed = profile.with_overrides({'lateral_slip_sd_m_s': .05}, label='slip')
    problems = dr.check_profile_against_field_response(changed)
    assert len(problems) == 1 and 'lateral_slip_sd_m_s' in problems[0]


def test_drive_vibration(profile):
    params = FieldResponseParams()
    for duty in (40., 89., 90., 95., 100.):
        assert dr.drive_vibration_m_s2(profile, duty) == pytest.approx(params.drive_vibration(duty))
    assert dr.drive_vibration_m_s2(profile, 95.) == pytest.approx(2.35)
