"""The field response plant reproduces the 2026-10-10 manual-drive statistics.

Targets are the MEASURED statistics re-derived from the field evidence (LiDAR
scan matching, raw IMU, ~2 Hz MCU status); tolerances reflect record
resolution and the finite number of seeded plant runs. Plant statistics are
computed with the evidence's own sample construction where it matters (stop
from the release with a 10 Hz/0.5 s stillness rule, 0.5 s gyro std windows).
Passing these tests does NOT validate the model below 90% duty or on a real
vehicle.
"""
from dataclasses import fields, replace
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from field_response import (CATALOGUE, DASHBOARD, ESTIMATED, FIRMWARE, MEASURED, PIVOT_EVENT_RATES_DEG_S,
                            SIM_ASSUMED, FieldPlant, FieldResponseParams, McuRamp, manual_mix, provenance,
                            provenance_digest)
from actuator_response import ActuatorResponse

DT = .01
HERE = Path(__file__).resolve().parent


def drive(command, seconds, seed, params=None, surface='normal'):
    """Hold a duty command with a 10 Hz keepalive (the MCU watchdog is 2 s)."""
    plant = FieldPlant(params, seed=seed, surface=surface)
    rows = []
    for k in range(int(round(seconds / DT))):
        if k % 10 == 0:
            plant.command(*command)
        plant.step(DT)
        rows.append((plant.time_s, plant.x, plant.y, plant.yaw, plant.v, plant.w))
    return plant, np.array(rows)


def windows(rows, start, end, column):
    return [rows[(rows[:, 0] > k) & (rows[:, 0] <= k + 1), column].mean() for k in range(start, end)]


def test_every_parameter_has_explicit_provenance():
    table = provenance()
    assert table['pwm_is_wheel_speed'] is False and table['encoder'] is False
    assert table['real_vehicle_validated'] is False
    allowed = {MEASURED, ESTIMATED, SIM_ASSUMED, CATALOGUE, FIRMWARE, DASHBOARD}
    for item in fields(FieldResponseParams):
        entry = table['parameters'][item.name]
        assert entry['provenance'] in allowed, item.name
        assert entry['basis']
    p = table['parameters']
    assert p['speed_at_100_m_s']['provenance'] == MEASURED
    assert p['mcu_ramp_pct_per_s']['provenance'] == MEASURED
    # firmware/dashboard constants are not "measured from the records"
    assert p['mcu_reversal_wait_s']['provenance'] == FIRMWARE
    assert p['mcu_watchdog_s']['provenance'] == FIRMWARE
    assert p['mixer_outside_ratio']['provenance'] == DASHBOARD
    assert p['forward_deadband_pct']['provenance'] == SIM_ASSUMED      # below 90% unmeasured
    assert p['arc_speed_loss_per_pct_diff']['provenance'] == SIM_ASSUMED and p['arc_speed_loss_per_pct_diff']['value'] == 0
    assert p['pivot_icr_behind_lidar_m']['provenance'] == ESTIMATED
    assert p['pivot_stall_probability_stall_prone']['provenance'] == ESTIMATED
    assert p['stall_late_breakaway_probability']['provenance'] == SIM_ASSUMED
    assert p['stall_late_breakaway_probability']['value'] == 0       # only instance: human handling
    assert p['gyro_still_noise_sd_deg_s']['provenance'] == MEASURED
    assert p['drive_vibration_at_100_m_s2']['provenance'] == MEASURED
    assert p['pivot_rate_events_deg_s']['provenance'] == MEASURED
    assert p['footprint_length_m']['provenance'] == CATALOGUE
    assert sum(table['counts'].values()) == len(p)
    changed = provenance(replace(FieldResponseParams(), forward_deadband_pct=35.))
    assert changed['parameters']['forward_deadband_pct']['provenance'].startswith('SCENARIO_OVERRIDE')


def test_manual_mixer_matches_dashboard_rule():
    assert manual_mix(30, 1, 0) == (30, 30)
    assert manual_mix(30, 1, -1) == (30, 45)        # W+A: right is outside
    assert manual_mix(30, 1, 1) == (45, 30)         # W+D
    assert manual_mix(80, 1, 1) == (100, 80)        # outside clipped at 100
    assert manual_mix(100, 1, -1) == manual_mix(100, 1, 0) == (100, 100)
    assert manual_mix(30, -1, 1) == (-45, -30)      # S+D keeps the inside at speed
    assert manual_mix(30, 0, -1) == (-30, 30)       # A alone: opposite-duty pivot
    with pytest.raises(ValueError):
        manual_mix(30, 2, 0)


def test_mcu_ramp_reversal_and_integer_contract():
    mcu = McuRamp()
    t = 0.
    mcu.command(100, 100, t)
    while mcu.applied[0] < 100:
        t += DT
        mcu.step(t, DT)
    assert t == pytest.approx(1.0, abs=.015)
    start = t
    while mcu.applied[0] > -100:
        t += DT
        mcu.command(-100, -100, t)
        mcu.step(t, DT)
    assert t - start == pytest.approx(2.12, abs=.03)         # fade 1.0 + 0.12 break + ramp 1.0 (records 2.06 p50)
    mcu.command(60, 60, t)
    mcu.hard_stop()
    assert mcu.applied == [0., 0.] and mcu.target == [0., 0.]
    with pytest.raises(ValueError):
        mcu.command(50.5, 50, t)


def test_mcu_watchdog_zeroes_immediately_like_the_firmware():
    mcu = McuRamp()
    t = 0.
    mcu.command(100, 100, t)
    for _ in range(150):
        t += DT
        mcu.step(t, DT)
    assert mcu.applied == [100., 100.]
    for _ in range(int(.6 / DT)):                            # command stopped at t=0: trips at 2.0 s
        t += DT
        mcu.step(t, DT)
    assert mcu.watchdog_tripped and mcu.applied == [0., 0.]  # stopDirections(): no fade


def test_mcu_break_applies_to_any_restart_from_zero():
    mcu = McuRamp()
    t = 0.
    mcu.command(20, 20, t)
    while mcu.applied[0] < 20:
        t += DT
        mcu.step(t, DT)
    mcu.command(0, 0, t)
    while mcu.applied[0] > 0:
        t += DT
        mcu.step(t, DT)
    zero_at = t
    mcu.command(50, 50, t)                                   # same direction again
    while mcu.applied[0] == 0:
        t += DT
        mcu.step(t, DT)
    assert t - zero_at >= .12 - 1e-9                         # firmware BREAK_MS after reaching zero


def test_straight_speed_matches_measured_100_and_90_percent():
    fast = [v for seed in range(60) for v in windows(drive((100, 100), 8, seed)[1], 3, 8, 4)]
    slow = [v for seed in range(30) for v in windows(drive((90, 90), 8, seed)[1], 3, 8, 4)]
    p10, p50, p90 = np.percentile(fast, [10, 50, 90])
    assert p50 == pytest.approx(.90, abs=.03)
    assert p10 == pytest.approx(.727, abs=.05) and p90 == pytest.approx(.999, abs=.04)
    assert np.median(slow) == pytest.approx(.72, abs=.04)


def test_arc_90_100_is_a_weak_turn():
    yaw, speed = [], []
    for seed in range(60):
        _, rows = drive((90, 100), 10, seed)
        yaw += [math.degrees(v) for v in windows(rows, 3, 10, 5)]
        speed += windows(rows, 3, 10, 4)
    assert np.median(yaw) == pytest.approx(.96, abs=.3)       # LiDAR 0.96, IMU 0.98 deg/s (radius ~45 m)
    assert np.percentile(yaw, 90) < 2.6                       # LiDAR p90 2.38 (0.4 s windows)
    # No arc speed loss is modelled (SIM_ASSUMED 0): 95% mean duty runs at the straight curve.
    assert np.median(speed) == pytest.approx(.81, abs=.04)


def _stop_from_release(seed, params=None):
    """Clean fade-to-zero stop measured like the evidence: from the release, 10 Hz positions,
    stop = first scan whose next-0.5 s speed is below 0.03 m/s."""
    plant = FieldPlant(params, seed=seed)
    rows = []
    for k in range(600 + seed % 10):
        if k % 10 == 0:
            plant.command(100, 100)
        plant.step(DT)
    t0, v0, x0, y0, yaw0 = plant.time_s, plant.v, plant.x, plant.y, plant.yaw
    plant.command(0, 0)
    for _ in range(500):
        plant.step(DT)
        rows.append((plant.time_s, plant.x, plant.y))
    r = np.array(rows)
    scans = np.arange(t0 + .05 * (seed % 2), t0 + 4.5, .1)
    p = np.column_stack((np.interp(scans, r[:, 0], r[:, 1]), np.interp(scans, r[:, 0], r[:, 2])))
    axis = np.array((math.cos(yaw0), math.sin(yaw0)))
    for j in range(len(scans) - 5):
        if np.linalg.norm(p[j + 5] - p[j]) / .5 < .03:
            return v0, float((p[j] - (x0, y0)) @ axis), float(scans[j] - t0)
    return v0, math.nan, math.nan


def test_stop_from_release_reproduces_the_clean_fades():
    rows = [_stop_from_release(seed) for seed in range(160)]
    fast = [(d, t) for v, d, t in rows if .88 <= v <= .97]
    assert len(fast) >= 40
    d50 = np.median([d for d, _ in fast])
    t50 = np.median([t for _, t in fast])
    assert d50 == pytest.approx(.566, abs=.05)                # measured 0.559/0.566/0.595 m from 0.90-0.95 m/s
    assert t50 == pytest.approx(.895, abs=.12)                # measured 0.89-0.90 s
    # The previous fit (decel 1.45, no onset) is clearly rejected by the same procedure.
    old = replace(FieldResponseParams(), coast_onset_s=0., coast_decel_median_m_s2=1.45, coast_decel_log_sd=.45)
    old_fast = [d for v, d, _ in (_stop_from_release(seed, old) for seed in range(80)) if .88 <= v <= .97]
    assert np.median(old_fast) < .45


def test_straight_wander_matches_per_metre_statistics():
    per_metre, offsets, right = [], [], []
    for seed in range(400):
        rng = np.random.default_rng(1000 + seed)
        _, rows = drive((100, 100), 20, seed)
        t = 2.
        while t < 17:
            length = rng.uniform(3, 8)
            segment = rows[(rows[:, 0] >= t) & (rows[:, 0] < t + length)]
            t += length
            if len(segment) < 300:
                break
            turn = math.degrees(math.atan2(math.sin(segment[-1, 3] - segment[0, 3]),
                                           math.cos(segment[-1, 3] - segment[0, 3])))
            chord = segment[-1, 1:3] - segment[0, 1:3]
            per_metre.append(turn / np.linalg.norm(chord))
            right.append(turn < 0)
            unit = chord / np.linalg.norm(chord)
            rel = segment[:, 1:3] - segment[0, 1:3]
            offsets.append(np.max(np.abs(rel[:, 0] * unit[1] - rel[:, 1] * unit[0])))
    p10, p50, p90 = np.percentile(per_metre, [10, 50, 90])
    # LiDAR runs: p10/p50/p90 -1.33/-0.23/+0.90 deg/m, right share 0.60 (IMU median -0.11, 0.565)
    assert p10 == pytest.approx(-1.33, abs=.25) and p90 == pytest.approx(.90, abs=.25)
    assert -.30 <= p50 <= -.08                                 # between the LiDAR and IMU medians
    assert .55 <= np.mean(right) <= .66
    assert np.median(offsets) == pytest.approx(.041, abs=.015)
    assert np.percentile(offsets, 90) == pytest.approx(.103, abs=.03)


def test_pivot_rates_follow_the_measured_event_distribution_without_clipping():
    rates = []
    for seed in range(240):
        plant, rows = drive((-100, 100), 4, seed)
        if not plant.pivot.stalled:
            rates.append(math.degrees(rows[rows[:, 0] > 2.5, 5].mean()))
    lo, hi = min(PIVOT_EVENT_RATES_DEG_S), max(PIVOT_EVENT_RATES_DEG_S)
    assert np.median(rates) == pytest.approx(22.1, abs=3.)    # LiDAR events p50 22.1 deg/s
    assert np.mean(np.array(rates) < 17.) >= .12              # measured: 4/16 below 17 (slow tail)
    assert np.mean(np.array(rates) > 26.) >= .2               # measured: 6/16 above 26
    assert min(rates) < lo + 1. and max(rates) > hi - 1.


def test_pivot_stall_shares_by_surface():
    normal = [drive((-100, 100), 3, seed)[0].pivot.stalled for seed in range(200)]
    prone = [drive((-100, 100), 3, seed, surface='stall_prone')[0].pivot.stalled for seed in range(80)]
    assert np.mean(normal) == pytest.approx(.05, abs=.04)
    assert np.mean(prone) == pytest.approx(.75, abs=.12)
    rotating = [math.degrees(rows[rows[:, 0] > 2.5, 5].mean()) for seed in range(80)
                for plant, rows in [drive((-100, 100), 4, seed, surface='stall_prone')] if not plant.pivot.stalled]
    assert rotating and max(rotating) < 9.                    # 035952 rotating events 6.5-7.3 deg/s


def test_pivot_breakaway_needs_the_minimum_duty_and_the_delay():
    plant, rows = drive((-60, 60), 3, 2)                      # below the SIM_ASSUMED 80 % minimum
    assert plant.regime == 'pivot_below_breakaway' and np.all(rows[:, 5] == 0.)
    plant = FieldPlant(seed=3)
    delay = None
    for k in range(300):
        if k % 10 == 0:
            plant.command(-100, 100)
        plant.step(DT)
        if plant.pivot is not None and min(map(abs, plant.mcu.applied)) >= 80 and delay is None:
            delay = plant.pivot.breakaway_delay_s
            loaded_from = plant.time_s
        if delay is not None and plant.time_s - loaded_from < delay - 2 * DT:
            assert plant.w == 0.                                # no rotation before the breakaway delay
    assert not plant.pivot.stalled and abs(plant.w) > 0.


def test_late_breakaway_is_off_by_default_and_works_when_overridden():
    stalled = [plant for seed in range(80) for plant in [drive((-100, 100), 8, seed, surface='stall_prone')[0]]
               if plant.pivot.stalled]
    assert stalled and all(plant.w == 0. for plant in stalled)   # default probability 0: never breaks away
    late = replace(FieldResponseParams(), stall_late_breakaway_probability=1.)
    released = [drive((-100, 100), 8, seed, params=late, surface='stall_prone')[0] for seed in range(20)]
    assert any(p.pivot.stalled and p.pivot.late_breakaway_s is not None and abs(p.w) > 0 for p in released)


def test_pivot_outcome_does_not_depend_on_what_happened_before():
    """Dedicated pivot stream: the k-th pivot event is the same whatever the timing before it."""
    draws = []
    for warmup in (0, 137, 1450):
        plant = FieldPlant(seed=(5, 77, 1))
        for k in range(warmup):
            if k % 10 == 0:
                plant.command(100, 100)
            plant.step(DT)
        for k in range(150):
            if k % 10 == 0:
                plant.command(-100, 100)
            plant.step(DT)
        draws.append(plant.pivot_log[0])
    assert draws[0] == draws[1] == draws[2]


def test_rotation_stops_after_outputs_reach_zero():
    plant, _ = drive((-100, 100), 4, 3)
    assert not plant.pivot.stalled and abs(plant.w) > math.radians(4)
    plant.command(0, 0)
    while max(map(abs, plant.mcu.applied)) > 0:
        plant.step(DT)
    for _ in range(50):
        plant.step(DT)
    assert abs(math.degrees(plant.w)) < .5                     # settles p50/p90 0.03/0.31-0.54 s


def _gyro_std(command, seeds=40):
    stds, vibs = [], []
    for seed in range(seeds):
        plant = FieldPlant(seed=seed)
        samples = []
        for k in range(int(6 / DT)):
            if k % 10 == 0:
                plant.command(*command)
            plant.step(DT)
            if (k + 1) % 5 == 0 and plant.time_s > 3:
                s = plant.imu_sample()
                samples.append((math.degrees(s['gyro_z_rad_s']), np.linalg.norm(np.subtract(s['accel_m_s2'], (0, 0, 9.81)))))
        g = np.array(samples)
        stds += [g[i:i + 10, 0].std() for i in range(0, len(g) - 10, 10)]
        vibs += [math.sqrt(np.mean(g[i:i + 10, 1] ** 2)) for i in range(0, len(g) - 10, 10)]
    return np.median(stds), np.median(vibs)


def test_imu_noise_and_vibration_match_the_measured_regimes():
    std, vib = _gyro_std((100, 100))
    assert std == pytest.approx(4.11, abs=.7) and vib == pytest.approx(3.01, abs=.5)   # straight 100/100
    std, vib = _gyro_std((0, 0))
    assert std < .3 and vib < .3                                                     # stop 0.09 / 0.03
    std, vib = _gyro_std((100, -100), seeds=30)
    assert std == pytest.approx(6.24, abs=1.5)                                       # pivot right


def test_zero_output_never_moves_and_runs_are_seeded():
    plant = FieldPlant(seed=9)
    for _ in range(500):
        plant.command(0, 0)
        plant.step(DT)
    assert (plant.x, plant.y, plant.yaw) == (0., 0., 0.)
    a = drive((80, 95), 5, 4)[1]
    b = drive((80, 95), 5, 4)[1]
    np.testing.assert_array_equal(a, b)


def test_actuator_profile_selection_keeps_qualitative_default():
    old = ActuatorResponse({'enabled': True})
    assert old.status()['profile'] == 'qualitative_stress' and old.ramp_step([0, 0], [1, 1], .02, 1.) is None
    with pytest.raises(ValueError):
        ActuatorResponse({'enabled': True, 'profile': 'made_up'})


def test_field_profile_wheel_targets_differ_from_the_qualitative_profile():
    field = ActuatorResponse({'enabled': True, 'profile': 'field_20261010'}, track_m=.56)
    qualitative = ActuatorResponse({'enabled': True})
    status = field.status()
    assert status['profile'] == 'field_20261010' and status['calibrated_speed_curve'] is False
    assert status['provenance_sha256'] == provenance_digest(field.field.params)
    assert len(str(status)) < 1200                             # compact: no full provenance table per message
    # MEASURED ramp: 100 %/s per side, so 0.02 s moves 2 % of the 1 m/s maximum.
    assert field.ramp_step([0., 0.], [1., 1.], .02, 1.) == pytest.approx([.02, .02])
    assert field.wheel_targets(0., 0., 1., 1.) == (0., 0.)
    scale = field.field.plant.speed_run_scale
    left, right = field.wheel_targets(1., 1., 1., 1.02)
    assert (left + right) / 2 == pytest.approx(.90 * scale, abs=1e-3)     # 0.90 m/s x run scale
    # SIM_ASSUMED 50 % deadband: 40 % duty does not move (qualitative profile does).
    assert field.wheel_targets(.4, .4, 1., 1.04) == (0., 0.)
    assert sum(qualitative.wheel_targets(.4, .4, 1., 1.04)) > 0.
    # Pivot: no yaw before the breakaway, then a measured-distribution event rate (or 0 if stalled).
    first = field.wheel_targets(-1., 1., 1., 1.06)
    assert first == pytest.approx((0., 0.)) or field.field.plant.pivot.stalled
    for k in range(60):
        last = field.wheel_targets(-1., 1., 1., 1.08 + .02 * k)
    w = math.degrees((last[1] - last[0]) / .56)
    if not field.field.plant.pivot.stalled:
        assert 3. < w < 40.
    q_left, q_right = qualitative.wheel_targets(-1., 1., 1., 1.)
    assert abs(math.degrees((q_right - q_left) / .56)) > 100.        # qualitative: far faster


def test_shipped_rover_yaml_keeps_the_qualitative_default_and_a_loadable_field_block():
    config = yaml.safe_load((HERE / 'rover.yaml').read_text(encoding='utf-8'))
    response = config['vehicle']['actuator_response']
    assert response['profile'] == 'qualitative_stress'
    switched = dict(response, profile='field_20261010')
    field = ActuatorResponse(switched, track_m=.56)
    assert field.field is not None and field.field.plant.surface == response['field_20261010']['surface']
    # The shipped seed's first pivot rotates (a first Gazebo trial should not start with the rare stall).
    for k in range(80):
        field.wheel_targets(-1., 1., 1., .02 * k)
    assert not field.field.plant.pivot.stalled
