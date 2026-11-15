"""Executor configuration: frozen dataclass, strict JSON loading, validate() and feasibility rules.

No numpy, no ROS. Every key is typed and bounded; unknown keys are refused. Motor output stays
OFF unless motor_output_enabled is true AND every rule that needs a field measurement is met
(the caller additionally requires its own CLI flag and an operator Arm; that is not done here).

Profiles
  'field'  the only profile for the orchard. A bench pattern (fixed duty sequence that bypasses
           the controller) can NOT be configured in it: bench_pattern must be empty and
           bench_ack absent, and load_config(path, expected_profile='field') refuses a file whose
           profile is anything else.
  'bench'  wheels-off-ground bench tests only. Requires bench_ack == 'WHEELS_OFF_GROUND'. The
           pattern still goes through the same hold gating, freshness rule and OutputGuard.

Feasibility rules (WEEKEND_PLAN 3.3 + FEASIBILITY_CRITIQUE H1), checked whenever the measured
breakaway duty is known and the inner-wheel rule is enabled:
  inner wheel in FOLLOW   base_duty - max_differential  >= breakaway + inner_margin
  inner wheel in REDUCE   reduce_duty - max_differential >= breakaway + inner_margin
  (REDUCE is never entered by the allowed continuous modes; the default reduce duty equals the
  base duty so no configuration can put the inner wheel below breakaway there either.)
With motor output enabled the measured values are required: breakaway (if the inner rule is on),
the measured speed at the base duty (time backstop, speed limit) and the measured yaw rate at the
configured full differential (yaw authority).

Day-1 review fixes (Fix A; review_safety_day1 S5/S6, review_correctness_day1 F2):
  rolling threshold (F2)  rolling_breakaway_duty_pct (C3b, nullable): the duty at which a wheel that
     is ALREADY turning keeps turning. Measured -> the inner-wheel rule becomes two rules:
       moving (FOLLOW)       base/reduce - max_differential >= rolling + inner_margin
       start (standstill)    base/reduce >= breakaway + inner_margin
     and the Runner limits the differential symmetrically (inner wheel raised to breakaway + margin,
     outer lowered by the same amount, mean kept) until the estimator reports sustained motion
     (core.Runner._inner_wheel_limit). Not measured -> unchanged: base/reduce - max_differential >=
     breakaway + margin, and the guard minimum is breakaway + margin at all times.
  body mask (S5)  every masked beam: max_range <= distance from the LiDAR along that beam to the
     footprint boundary + 0.05 m; beams within 35 deg of the travel axis: max_range <= distance along
     the beam to the front footprint edge + 0.05 m. A mask can hide the vehicle, not the corridor.
  bind_host (S6)  an IPv4 address literal (ipaddress), never unspecified/wildcard ('', '0', '0.0.0.0',
     '::', '::ffff:0.0.0.0'), multicast or broadcast; names are refused ('0' binds 0.0.0.0 on Linux).
  motor output, field profile (S6), in addition to the measurements above:
     guard_heading_deg in [10, 30] deg and guard_lateral_margin_m in [0, 0.3] m (null = guard off: refused);
     stop model (front_margin_m, stop_decel_m_s2, reaction_s, stop_onset_s) not looser than the
     vendored defaults (0.35 m, 1.07 m/s^2, 0.15 s, 0.25 s) unless measured_stop_distance_m (C7) is
     present; whenever it is present the model at v_measured_m_s must be at least as conservative:
     model distance >= measured AND model distance + front_margin >= measured + 0.35 m;
     side_hard/soft/exit margins not below 0.03/0.15/0.20 m, side_confirm_scans not above 2;
     lidar_max_age_s <= 0.45 s, imu_max_age_s <= 0.40 s.

Day-2 review fixes (review_safety_day2 N1/N2/N8, review_correctness_day2 R2; block at the end of this file):
  N1  motor output (row and calibration) also refuses: near-return confirmation looser than 2 scans / 2.0 m /
      0.35 m + 0.10 m/scan, range_min_floor_m > 0.05, backstop margin < 2 m or time factor > 1.15, max_speed_m_s >
      0.55, stall/stale/bridge/hold timings and repeat limits above the defaults, inner_margin_pct < 10,
      yaw_authority_fraction > 0.6, min_yaw_authority_deg_s < 2, a footprint below 1.0 x 0.55 m or a LiDAR outside it,
      lidar_yaw_offset_deg more than 10 deg from 179.08, imu_yaw_sign -1, breakaway/rolling below 20 %, and controller
      overrides that delay a stop or a no-response decision (corridor_lost_s, drive_grace_s, drive_no_response_s,
      drive_min_speed_m_s, still_confirm_s, max_stops_per_window, reacquire_updates) beyond the vendored defaults.
  N2  calibration output: base_duty_pct <= 60, duty_cap_pct <= 75, max_speed_m_s <= 0.5 (no C3 speed is required).
  N8  row motor output needs measured_stop_distance_m (C7, the executor's own 0/0 stop; calibration run kind 'stop').
  R2  onsite_update: a lower differential scales the measured yaw authority (x new / file differential) and lowers
      max_yaw_rate_deg_s with it; a differential whose authority falls below the minimum is refused.

Calibration runs (calib_runs.py; review_correctness_day1 F2: C5 can not be measured with the dashboard mixer):
  calib_motor_output_enabled  motor output for the 'arc' (C5) and 'rolldown' (C3b) calibration runs only, field
     profile, with --motor-output and an Arm per run; row runs still need motor_output_enabled (and every
     measurement). The S6 motor rules apply; C2/C3/C5 measurements are not required (C5 is what the runs measure).
  calib_distance_cap_m / calib_time_cap_s   upper bounds of one calibration run (the operator may only lower them).

Field defaults from the C1 gate (field autonomy work folder of 2026-10-22, gate/C1_GATE_REPORT.txt section 5;
open-loop replay of the six 10/10 records, flying runs, continuous_side/weekend):
  P1 near_confirm_scans=2 (2 m, 0.35 m + 0.10 m/scan)  front_blocked 72 -> 5, side_blocked 2 -> 0;
     strict premature stops 1 per 21 m -> 1 per 127 m. Cost: a new near object is accepted 0.2 s
     later (9 cm at 0.45 m/s, inside the 0.35 m front margin).
  P2 imu_max_age_s 0.30 -> 0.40   real 20 Hz IMU receipt gaps up to 0.365 s; stale_imu stops 2 -> 0.
  P3 guard_heading_deg=25, guard_lateral_margin_m=0.0   continuous FOLLOW has no large-error limit and
     the tracked band stayed 'valid' at heading errors up to 180 deg / |lateral| 17.9 m after the
     vehicle turned (F4); terminal stop instead of a saturated arc across the rows.
  P4 scan_stamp_mode='header_offset'   header + running median(receipt - header) - latency, on the
     receipt (monotonic) clock: bursty delivery (10/03: receive gaps of 0 ms) no longer explodes the
     estimator speed (false front_blocked at clearance -133 m); 10/10 decisions unchanged (36 -> 35 stops).
  P5 corridor_every_n_scans 2 -> 3   replay decisions equal or better (mid-straight stops 16 -> 14),
     processing tail lower (parallel p99 65 -> 50 ms); with a 3x slower CPU n=2 gave stale/stall stops
     (1 per 87 m) and n=3 did not (1 per 132 m). To be confirmed on the Jetson (V5); 2 is allowed.

Assisted headland turn (TurnAssist, turn_assist.py; opt-in, default DISABLED): the turn_* / pivot_* /
yaw_disagree_* / handoff_* keys. turn_assist_enabled false (default) means no turn run may command
output; turn output additionally needs motor_output_enabled and every turn measurement
(turn_feasibility()['output_allowed']: C6 pivot duties on this headland surface, C2 breakaway, C3 speed;
checked by TurnAssist.start, a missing one never invalidates the row config), pivot caps above
90 % need an explicit user acknowledgement, and the field profile still refuses bench patterns. An
untouched turn block is left out of to_dict()/digest(), so existing configs keep their digest. (Steering branch:
turn_assist.py is not imported; the turn keys and their rules stay so that configs and digests are unchanged.)

Field decision rules (unified calibration 2026-10-22; block at the end of this file = the ONE place of the rule
parameters, used on the single-row branch by tools/field_calib/recommend.py and its final config generator
make_field_config.py; neither was imported to the steering branch):
  decision table   max_differential d = min(20, base - (R + inner_margin), 100 - base) with the C3b rolling threshold
                   R, else with the C2 breakaway B (base 55, margin 10: d = min(20, 45 - R) / min(20, 45 - B));
                   max_yaw_rate = min(4.5, 0.6 x measured authority) rounded down to 0.1 deg/s; C7 stop model check.
  steering GO rule G3/G3b (SIM steering-authority bracket): weaker-side C5 gain >= 0.10 deg/s per % of (R - L),
                   d >= 10 and the yaw at R - L = 2 d >= 4.0 deg/s.
  Row MOTOR output (field profile, the continuous modes = every allowed mode) therefore also refuses
  max_differential_pct outside [10, 20] (SIM evidence) and a measured authority below
  max(min_yaw_authority_deg_s, 4.0): the minimum authority of row motor output is raised from 2.0 to 4.0 deg/s.
  The dataclass default min_yaw_authority_deg_s stays 2.0, so SHADOW configs, their digest and the calibration
  runs (they measure C5) are unaffected.
  Pivot rotation centre (pivot_icr_*): ONE convention, see icr_base_from_lidar().

Single-row mode (mode 'single_row', 2026-11-02; goal document SINGLE_ROW_GOAL_20261102.md): ONE row along the
centreline, continuous forward differential correction of a drift that changes along the row, row-end detection ->
slow down -> stop; no pivot, no headland turn, no next row. The controller is the vendored
closed_loop/single_row.SingleRowController (core.Runner); the single_row_* keys below are its executor-facing settings and
an untouched single-row block is left out of to_dict()/digest() (existing configs keep their digest).
  localisation  the executor's own LiDAR + raw IMU estimator (corridor-relative lateral / heading errors, scan-matching
                speed and the resolved_polyline distance), labelled SENSOR_ONLY_SINGLE_ROW. A native navigation
                provider is not required (require_native_navigation must be set false explicitly; it is never silently
                ignored); when one is present it is an optional cross-check (executor_main). Every freshness, ownership,
                hold, obstacle (front / side / geometry guard / OutputGuard) and STOP rule of the row executor stays.
  speed         PWM is not speed: the controller adapts the base duty inside [single_row_base_min_pct,
                single_row_base_max_pct] toward single_row_target_speed_m_s from the LiDAR speed (start at base_duty_pct);
                the inner-wheel rules therefore hold at the LOWEST base (base_min - max_differential >= threshold +
                margin; base_min >= breakaway + margin for a start from standstill) and base_max + max_differential must
                fit duty_cap_pct.
  row length    optional at Start: none -> distance cap single_row_max_row_length_m and a motion-time cap
                backstop_time_factor x cap / min(single_row_min_speed_m_s, single_row_approach_speed_m_s); an entered
                length adds the existing L - backstop_margin_m distance bound (the smaller one ends the run). A backstop
                end is 'row_distance_backstop' / 'row_time_backstop' (OPERATOR_OR_BACKSTOP), never a row end.
  refused       turn_assist_enabled (no headland turn), require_native_navigation true (provider path), a
                controller_overrides.row_end_stop_m (single_row_end_stop_offset_m is the row-end stop setting).
  motor output  additionally: single_row_end_stop_offset_m >= footprint_length_m / 2 (the front stops at or before the
                observed end of the row support) and the steering GO rule of the continuous modes (max_differential 10-20,
                measured authority >= 4.0 deg/s).
"""
from dataclasses import MISSING, asdict, dataclass, field, fields, replace
import hashlib
import ipaddress
import json
import math
import posixpath

from . import perception_settings as PS   # stdlib only (local_grid_* / gnss_* blocks)

SCHEMA = 'ubts.row_executor.config.v1'
PROFILES = ('field', 'bench')
BENCH_ACK = 'WHEELS_OFF_GROUND'
# Forward-only modes of the vendored controllers: no pivot (TURN) and no REDUCE state. 'single_row' = the vendored
# closed_loop/single_row.SingleRowController (continuous FOLLOW + row-end slow zone; side rule ON).
SINGLE_ROW_MODE = 'single_row'
CONTROLLER_MODES = ('continuous', 'continuous_side', SINGLE_ROW_MODE)
SCAN_STAMP_MODES = ('receipt', 'header_offset')
CONTROL_HZ = 20.          # the vendored controller's PI integration assumes 20 Hz ticks
MAX_BENCH_STEP_S = 10.
MAX_BENCH_TOTAL_S = 60.
MAX_BENCH_STEPS = 32
MAX_MASK_SEGMENTS = 512

# Allowed ControlConfig overrides (name -> (type, low, high)). Everything that could enable a
# pivot, a turn, a fallback or change the tick rate is deliberately absent.
CONTROLLER_OVERRIDE_LIMITS = {
    'heading_gain': (float, .1, 3.),
    'lookahead_m': (float, 1., 10.),
    'yaw_kp_pct_per_deg_s': (float, 0., 10.),
    'yaw_ki_pct_per_deg': (float, 0., 10.),
    'enter_error_deg': (float, .5, 10.),
    'exit_error_deg': (float, .1, 9.),
    'urgent_error_deg': (float, 1., 30.),
    'recorrect_interval_s': (float, 0., 10.),
    'min_hold_s': (float, 1., 5.),
    'still_confirm_s': (float, .2, 2.),
    'drive_grace_s': (float, .5, 3.),
    'drive_no_response_s': (float, .5, 3.),
    'drive_min_speed_m_s': (float, .01, .2),
    'corridor_lost_s': (float, .5, 5.),
    'corridor_wait_s': (float, .5, 10.),
    'reacquire_timeout_s': (float, 1., 20.),
    'reacquire_updates': (int, 1, 10),
    'front_margin_m': (float, .2, 2.),
    'stop_onset_s': (float, 0., 1.),
    'stop_decel_m_s2': (float, .2, 5.),
    'reaction_s': (float, 0., 1.),
    'stop_timeout_s': (float, 1., 10.),
    'row_end_stop_m': (float, 0., 3.),
    'max_stops_per_window': (int, 1, 10),
    'side_hard_margin_m': (float, .01, .5),
    'side_soft_margin_m': (float, .02, .8),
    'side_soft_exit_m': (float, .03, 1.),
    'side_confirm_scans': (int, 1, 10),
}


class ConfigError(ValueError):
    """One or more configuration problems; .problems lists every one of them."""

    def __init__(self, problems):
        self.problems = list(problems)
        super().__init__('; '.join(self.problems))


@dataclass(frozen=True)
class ExecutorConfig:
    schema: str = SCHEMA
    profile: str = 'field'
    # --- output enable (fail-safe default) -------------------------------------------------
    motor_output_enabled: bool = False
    require_native_navigation: bool = True  # latest AUTO: missing native map/pose/cost -> WAIT, never raw fallback
    bench_ack: str = ''                    # bench profile only: must equal BENCH_ACK
    bench_pattern: tuple = ()              # bench profile only: ((duration_s, left, right), ...)
    # --- controller ------------------------------------------------------------------------
    mode: str = 'continuous_side'
    base_duty_pct: int = 55                # 10/3: 55 % ~ 0.46 m/s (GNSS, 3 segments); C2/C3 decide
    reduce_duty_pct: int = 55              # REDUCE is unused by continuous modes; = base (H1)
    max_differential_pct: int = 20         # outputs base +- 20
    arc_yaw_deg_s_per_pct_diff: float = .15
    max_yaw_rate_deg_s: float = 4.5
    controller_overrides: dict = field(default_factory=dict)
    # --- output guard + feasibility ----------------------------------------------------------
    duty_cap_pct: int = 75                 # >= base + max_differential
    diff_cap_pct: int = 40                 # >= 2 * max_differential
    breakaway_duty_pct: object = None      # measured B_fwd (C2); int or None (unknown)
    inner_rule_enabled: bool = True        # guard: min(L, R) >= breakaway + inner_margin when driving
    inner_margin_pct: int = 10
    v_measured_m_s: object = None          # measured speed at base duty (C3); float or None
    max_speed_m_s: float = .5
    measured_yaw_at_max_diff_deg_s: object = None   # measured yaw at +-max_differential (C5)
    min_yaw_authority_deg_s: float = 2.
    yaw_authority_fraction: float = .6     # max_yaw_rate <= fraction * measured authority
    # --- sensor adaptation -------------------------------------------------------------------
    lidar_yaw_offset_deg: float = 179.08   # travel axis is -179.08 deg in the scan frame (estimate)
    imu_yaw_sign: int = 1
    imu_up_tau_s: float = 1.
    scan_latency_s: float = .08            # scan stamp = receipt - latency (replay shift 0.005-0.15)
    range_min_floor_m: float = .05
    beam_grid_count: int = 3240            # body-mask grid = the real S2 scan grid
    beam_grid_angle_min_rad: float = -3.1415927410125732
    beam_grid_increment_rad: float = .0019398535368964076
    body_mask: tuple = ()                  # ((first_beam, last_beam, max_range_m), ...) raw beam order
    body_mask_limit_m: float = 1.2         # a mask never hides anything farther than this
    max_scan_trim_beams: int = 4
    # --- estimator (EstimatorConfig overrides) ------------------------------------------------
    lidar_x_in_base_m: float = .18
    footprint_length_m: float = 1.22
    footprint_width_m: float = .70
    corridor_every_n_scans: int = 3        # C1 gate P5 (SIM default 2; Jetson V5 decides)
    distance_mode: str = 'resolved_polyline'  # sensor-only path; scan_norm_legacy is explicit development compatibility
    row_end_continuation_m: object = None  # None (OFF) or 12.0 with the 'pair' rule
    # --- near-return confirmation (C1 gate P1) --------------------------------------------------
    near_confirm_scans: int = 2            # 0 = off. A return nearer than near_confirm_range_m survives
    near_confirm_range_m: float = 2.       # only if each of the previous N processed scans of the run
    near_confirm_tol_m: float = .35        # had a return within tol (+ tol_step per older scan), same
    near_confirm_tol_step_m: float = .10   # rotated laser frame (one-scan near clusters = false stops)
    # --- geometry guard (C1 gate P3) -------------------------------------------------------------
    guard_heading_deg: object = 25.        # None = off. Driving + valid band + |heading_error| > this: END
    guard_lateral_margin_m: object = 0.    # None = off. |lateral| > width/2 - footprint/2 + margin: END
    # --- scan stamps (C1 gate P4) ----------------------------------------------------------------
    scan_stamp_mode: str = 'header_offset' # 'receipt' = receipt - scan_latency_s
    scan_stamp_window: int = 50            # scans in the running median of (receipt - header)
    scan_stamp_reset_s: float = .5         # |offset - median| above this (clock step): restart the median
    # --- freshness, pauses ---------------------------------------------------------------------
    control_hz: float = CONTROL_HZ
    lidar_max_age_s: float = .45
    imu_max_age_s: float = .4              # C1 gate P2 (SIM 0.3; real IMU receipt gaps up to 0.365 s)
    fresh_recover_s: float = 1.            # inputs fresh this long before a NEW hold press resumes
    max_stale_events: int = 3              # stale pauses per run; one more ends the run
    stall_guard_s: float = .35
    max_stall_guard_pauses: int = 3
    hold_timeout_s: float = .5
    send_hz: float = 10.
    terminal_stop_wait_s: float = 1.5
    # --- backstop (MVP-1 stop rule) -------------------------------------------------------------
    # The run ENDS at row_len - margin of counted distance (counted only while the estimator reports
    # motion, review F4); the vehicle then rests one stopping distance later: about row_len - 2 m + 0.28 m
    # at 0.45 m/s (stop model) = row_len - 1.7 m, +- odometry error (guard.py module doc).
    backstop_margin_m: float = 2.          # stop at row_len - 2 m (last 2 m and headland are manual)
    backstop_time_factor: float = 1.15     # time >= 1.15 * (row_len - 2) / v_measured
    min_run_distance_m: float = 1.
    max_row_len_m: float = 300.
    # --- bridge / operator interface ---------------------------------------------------------
    socket_path: str = '/run/ubts-rover/control.sock'
    bridge_timeout_s: float = .6
    bind_host: str = '10.77.0.1'
    bind_port: int = 8097
    # --- day-1 review fixes (Fix A): field measurements, nullable --------------------------------
    rolling_breakaway_duty_pct: object = None   # C3b: duty that keeps an already turning wheel turning (F2)
    measured_stop_distance_m: object = None     # C7: p90 distance from the zero command to rest at base duty
    # --- calibration runs 'arc' (C5) / 'rolldown' (C3b), calib_runs.py (field profile, MOTOR only) --------
    calib_motor_output_enabled: bool = False    # motor output for CALIBRATION runs only (row runs need
                                                # motor_output_enabled); the S6 motor safety rules must hold
    calib_distance_cap_m: float = 15.           # counted distance of one calibration run (operator may lower it)
    calib_time_cap_s: float = 45.               # elapsed time of one calibration run from the press (may lower it)

    # --- single-row mode (mode 'single_row'; vendored closed_loop/single_row.SingleRowController) -------------
    # Read only in mode 'single_row'; an untouched block is left out of to_dict()/digest(). Speeds are LiDAR
    # scan-matching speed targets of the controller's speed loop (PWM is not speed), not calibration values.
    single_row_target_speed_m_s: float = .40    # cruise target (SingleRowConfig.cruise_speed_m_s)
    single_row_min_speed_m_s: float = .20       # target under a large lateral error / authority SLOW (min_speed_m_s,
                                                # authority_slow_speed_m_s); with the approach speed: the time-cap speed
    single_row_approach_speed_m_s: float = .20  # row-end slow zone target (approach_speed_m_s)
    single_row_base_min_pct: int = 45           # base duty range of the speed loop (base_duty_pct = the start base);
    single_row_base_max_pct: int = 55           # inner-wheel rules at base_min, base_max + max_differential <= duty cap
    single_row_bias_max_pct: float = 12.        # limit of the persistent drift-bias estimate [% differential]
    single_row_start_max_deg: float = 25.       # start / re-acquisition alignment limit (<= guard_heading_deg)
    single_row_end_stop_offset_m: float = .61   # the base stops this far before the observed row end (front at the end)
    single_row_max_row_length_m: float = 120.   # distance safety cap of one run (the row length is optional at Start)
    # vendored estimator options of the single-row mode (EstimatorConfig; None = the original rule). SIM development
    # values (single-row-20261102 DEV round 1 analysis), not field calibrations:
    single_row_acquire_behind_m: object = 2.5   # first corridor acquisition: returns from this far behind the LiDAR
                                                # to 8 m ahead (headland structures behind the start are not the row)
    single_row_acquire_window_deg: float = 30.  # first acquisition axis search +- this (EstimatorConfig
                                                # acquire_axis_window_deg, 50 elsewhere): a start is refused beyond
                                                # single_row_start_max_deg anyway; diagonal tree alignments of the
                                                # grid at 36-50 deg won the dominance test in sparse rows
    single_row_end_segment_span_m: object = 2.  # row end: a side's support is cut at its first free gap unless the
                                                # support beyond is a continuing row on both sides (cross gap)
    single_row_end_last_trunk: bool = True      # row end: a small last trunk trimmed by the 97th percentile stays the end
    single_row_end_stagger_gap_m: object = 4.   # row end: both sides ended but one end > 2 m beyond the other: the far
                                                # side ends before its >= 4 m free gap that starts at the other side's
                                                # end when the support beyond it is not row-like (an isolated post /
                                                # bin on one line extension is not row support); needs the segment span
    single_row_band_about_vehicle: bool = True  # one-sided corridor updates turn the band about the vehicle, not about
                                                # the run's start (EstimatorConfig band_rotate_about_vehicle; a 0.7 deg
                                                # step 36 m from the start had moved the corridor 0.45 m at the vehicle)
    single_row_initial_press_grace_s: float = 1.  # the FIRST press of a run (await_hold) made while the inputs are not
                                                  # yet fresh for fresh_recover_s is kept this long while it stays
                                                  # held (input warm-up after Start / Arm); 0 = ignored at once

    # --- front-wheel steering (vehicle_profile 'front_steer'; steering branch P4, design 6.5-6.7; block at the end of
    # this file). Read only with vehicle_profile 'front_steer'; an untouched block is left out of to_dict()/digest()
    # (existing configs keep their digest). Every value below is a placeholder until the steering measurements
    # (B0-B5, F1) exist; the SIM configurations fill the 'measured' ones from the vehicle profile (SIM_ASSUMED).
    vehicle_profile: str = 'differential'        # 'differential' (left / right duty, history) or 'front_steer'
    steer_limit_left_mrad: int = 409             # equivalent (bicycle) soft limit toward the left [mrad]: the host
    steer_limit_right_mrad: int = 409            # profile steer_limit (computed from the per-wheel soft limits) / right
    steer_rate_limit_deg_s: float = 24.          # command slew limit while the drive command is not 0 (design 6.2:
                                                 # 0.8 x the measured steering rate B5; MCU CFG rmov)
    steer_rate_limit_standstill_deg_s: float = 12.   # ... while the drive command is 0 (0.8 x B5 standstill; MCU rstd)
    steer_measured_rate_deg_s: object = None     # B5 measured steering rate while driving [deg/s]
    steer_measured_rate_standstill_deg_s: object = None   # B5 / F5 measured steering rate at standstill [deg/s]
    steer_rate_min_deg_s: float = 8.             # steering GO rule: B5 >= this (placeholder, design 6.6; P8 fixes it)
    steer_range_measured: bool = False           # steering GO rule: B2/B3 measured marks in the calibration record
    steer_wheel_soft_margin_deg: object = None   # per-wheel soft limit = mechanical limit - this (GO: >= 2 deg)
    steer_min_pwm_pct: object = None             # B4 measured steering motor start PWM [%]
    steer_zero_offset_deg: object = None         # F1 measured zero offset [deg] (GO: |offset| <= 1.5)
    steer_servo_deadband_deg: object = None      # servo deadband of the MCU record [deg] (GO: <= 1.0)
    steer_polarity_checked: bool = False         # steering GO rule: B0 polarity check passed (MCU record pol=ok too)
    steer_cal_id: object = None                  # calibration set id the bridge must report (arm precheck)
    steer_allow_sim: bool = False                # True only in a SIM configuration: the device must report sim=1
                                                 # (False: it must report sim=0; arm precheck sim_identity_mismatch)
    steer_status_max_age_s: float = .3           # steering status older than this while ACTIVE: steering_feedback_stale
    steer_authority_fraction: float = .95        # steering_authority_insufficient (profile D2 placeholder): |command|
    steer_authority_s: float = 1.5               # >= fraction x soft limit for longer than this and |lateral| not falling
    steer_rotation_centre_x_m: float = -.38      # rotation centre of the side-sweep rule: the rear axle in the base frame
    steer_rotation_centre_sd_m: float = .05      # (profile rear_axle_x_in_base_m, design 6.7) and its uncertainty
    steer_row_end_curve_fit: bool = True         # front_steer estimator: curved row ends (EstimatorConfig
                                                 # row_end_curve_fit; False = the straight-band rule). S1c 2026-11-13
    steer_row_end_agree_m: object = 1.0          # front_steer estimator: a row end needs two consecutive full updates
                                                 # whose reported ends agree within this (EstimatorConfig
                                                 # row_end_agree_m; None = the original streak). S1c round 2, R53
    # The control bridge the executor talks to (docs/steering/EXECUTOR_ESP32_KO.md, 2026-11-16). 'v3': the V3 steering
    # bridge (steer_rover_bridge, the SIM and the design). 'esp32_abs_hold': the ESP32-S3 bridge of the real board
    # (dashboard scripts/esp32_steer_bridge.py, firmware ABS-HOLD-8-REBOUND-LIMITS) through bridge_client
    # Esp32AbsHoldAdapter: the same control socket and client rules; the adapter converts the steering scale, decides
    # when a new ANGLE target goes (the SIM host policy abs_hold_host) and runs the host monitors the firmware lacks.
    steer_bridge_kind: str = 'v3'
    steer_esp32_mark_deg: object = None          # equivalent wheel angle [deg] at the firmware command +-20 (the CAL
                                                 # marks). SIM: mechanical limit - 1 deg (abs_hold.mark_angle_deg);
                                                 # field: the B3 measurement. Needed with steer_bridge_kind esp32_abs_hold
    steer_esp32_cal: object = None               # [RIGHT, CENTER, LEFT, MAX_COMMAND] of the firmware CAL line the bridge
                                                 # must report (the calibration identity; the V3 bridge has steer_cal_id)
    steer_esp32_require_fw_watchdog: bool = True  # arm only when the firmware reports a drive watchdog (FW DRIVE_WD=):
                                                 # the board firmware of 2026-11-16 has none (EXECUTOR_ESP32_KO.md 4)
    steer_wheelbase_m: object = None             # wheelbase of the row-follow controller and of the ESP32 adapter's
                                                 # steering_ineffective rule (None: the controller default 0.76 m)

    # --- autonomous start and the operator link (design v2 V2-2, decision R32; docs/steering/OPERATOR_API_KO.md). An
    # untouched block is left out of to_dict()/digest() (existing configs keep their digest).
    start_mode: object = None                    # None = the profile's default (effective_start_mode): 'one_press' for
                                                 # front_steer, 'hold_to_run' for the differential profile (history);
                                                 # or explicitly 'one_press' / 'hold_to_run'
    operator_timeout_s: float = 1.               # one_press: last operator heartbeat + this -> PAUSED operator_link_lost
                                                 # (locked: config file only; design v2 V2-2 item 3, A17 amendment 31;
                                                 # 1.0 s from 0.5 s by decision R60, A17 amendment 60)
    tuning_file: object = None                   # front_steer: the field tuning file (field_tuning.py, design v2 V2-5);
                                                 # None = the package default field_tuning.default.json
    tuning_values: dict = field(default_factory=dict)   # front_steer: the steering controller values of the loaded field
                                                 # tuning (written by the executor loop from tuning_file, never by hand:
                                                 # with_tuning); {} = the controller's own defaults

    # --- actuator priors and estimator outputs of design v2 (V2-1 item 2, layer L1; front_steer only). Opt-in: with the
    # defaults nothing is fed and nothing changes; an untouched block is left out of to_dict()/digest() (existing
    # configs keep their digest). PWM is not an encoder: the prior is bounded and LiDAR + raw IMU stay the main input.
    # Every value is a placeholder until the steered vehicle is measured (the field tuning file replaces the table).
    prior_enabled: object = None                 # feed the applied drive duty and the measured steering angle of the
                                                 # steering status to the estimator; snapshot / log block 'actuator_prior'.
                                                 # None = the profile default (prior_active): on for front_steer (with
                                                 # weight 0 only the record and the online-scale monitor of PW-10, the
                                                 # decisions are unchanged), off for the differential profile; or
                                                 # explicitly true / false
    prior_weight: float = 0.                     # 0 = record and monitor only; (0, 1] = scan-match seed blend and the
                                                 # share of the bounded prediction (a failed match, a scan gap)
    prior_speed_table: tuple = ()                # ((duty_pct, speed_m_s), ...) rising; () = the vendored placeholder table
    prior_speed_scale: float = 1.                # k_field: table speed x scale = expected speed (field tuning 0.5-1.3)
    prior_wheelbase_m: object = None             # yaw-rate prior v tan(steer) / L; None = no yaw-rate prior
    prior_predict_max_s: float = 1.              # prediction bound since the last scan-matched pose (locked values):
    prior_predict_max_m: float = .5              # whichever comes first; then PAUSED 'prediction_limit'
    prior_mismatch_fraction: float = .2          # |table speed / LiDAR speed - 1| above this over the monitor window:
                                                 # table_mismatch (shown and logged; never a stop)
    corridor_quality: bool = False               # estimator 'corridor_quality' block (sides, support, ages, spread)

    # --- perception layers L2 local grid / L3 GNSS (design v2 V2-3, V2-1 3; PC-07, PC-12; perception_settings.py) ------
    # Both OFF by default; an untouched block is left out of to_dict()/digest() (existing configs keep their digest) and
    # the Runner builds no layer then (bit-identical). Placeholders (SIM development values), not field calibrations.
    local_grid_enabled: bool = False             # rolling occupancy grid in the estimator's odometry frame (local_grid.py)
    local_grid_cell_m: float = .10               # cell size [m]
    local_grid_length_m: float = 20.             # window along the grid axis [m] (30 % of it behind the vehicle)
    local_grid_width_m: float = 10.              # window across [m]
    local_grid_every_n_scans: int = 2            # grid update (insert + row-line fit) every N processed scans
    local_grid_free_beam_step: int = 0           # 0 = hits only; N = every N-th beam also marks free cells
    local_grid_min_hits: int = 3                 # occupied = hit in >= this many updates (persistence)
    local_grid_fusion_gain: float = 0.           # > 0: grid lines -> estimator corridor observations (source local_grid)
    local_grid_fuse_width: bool = False          # fusion also pulls the band width
    local_grid_publish: bool = False             # bounded grid serialisation in the executor status (operator map)
    local_grid_publish_period_s: float = .5      # ... at most this often
    local_grid_publish_max_bytes: int = 16384    # ... text size cap (the bitmap is pooled until it fits)
    gnss_mode: str = 'off'                       # 'off' | 'log_only' | 'geofence' (gnss_layer.py; never a steering input)
    gnss_max_hacc_m: float = 10.                 # quality gate: horizontal accuracy <= this
    gnss_max_age_s: float = 2.5                  # ... and age <= this (older = GNSS absent, a normal state)
    gnss_jump_gate_m: float = 3.                 # jump rejection: innovation against the dead-reckoned last fix
    gnss_fence_margin_m: float = 10.             # fence margin max(this, 3 x hacc) around the row rectangle
    gnss_fence_confirm: int = 3                  # consecutive accepted fixes outside before the fence trips
    gnss_course_baseline_m: float = 15.          # odometry path of a course window (alignment, drift)
    gnss_course_tolerance_deg: float = 45.       # course drift flag (a gross check: GNSS course is not a heading)

    # --- assisted headland turn (TurnAssist; opt-in, default DISABLED; TURN_DESIGN.txt 3.10) ------
    # With the defaults below no turn run can command any output (observe / dry runs only) and the
    # row executor is unchanged; an untouched turn block is left out of to_dict()/digest().
    turn_assist_enabled: bool = False
    pivot_duty_start_pct: object = None    # C6: lowest duty that rotated 3/3 on the headland surface + 5
    pivot_duty_cap_pct: object = None      # C6: duty that rotated 6/6; > 90 needs pivot_high_duty_ack
    pivot_high_duty_ack: str = ''          # PIVOT_HIGH_DUTY_ACK to allow a cap above 90 % (user decision)
    pivot_surface_ok: bool = False         # C6 passed on THIS headland surface (6/6 rotated, no stall)
    pivot_duty_step_pct: int = 10          # breakaway ladder step ...
    pivot_step_interval_s: float = 1.      # ... at most once per this interval
    pivot_breakaway_grace_s: float = 1.    # no rotation this long after the MCU ramp -> next step / stall
    pivot_rotating_deg_s: float = 5.       # rotating = smoothed gyro >= this for confirm_s ...
    pivot_rotating_confirm_s: float = .25
    pivot_rotating_lidar_deg: float = 1.5  # ... and scan-matching rotation >= this (odometry valid)
    pivot_stall_window_s: float = 1.       # mid-pivot stall: < stall_min_deg in this window
    pivot_stall_min_deg: float = 1.5
    pivot_rate_max_deg_s: float = 45.      # measured max 42 deg/s
    pivot_wrong_direction_deg: float = 3.
    pivot_coast_s_hi: float = .35          # initial coast time at d >= 90 % (measured 0.29-0.55 s at 100 %)
    pivot_coast_s_lo: float = .25          # initial coast time at d <= 70 %
    pivot_accept_error_deg: float = 8.
    pivot_topup_max: int = 1
    pivot_topup_max_deg: float = 25.
    pivot_overshoot_max_deg: float = 20.
    pivot_icr_ahead_left_m: object = None  # C6 rotation centre of LEFT pivots relative to the base centre
    pivot_icr_left_left_m: object = None   # (ahead, left) [m]; None = no lateral pre-correction
    pivot_icr_ahead_right_m: object = None # same for RIGHT pivots
    pivot_icr_left_right_m: object = None
    yaw_disagree_warn_deg: float = 5.      # |gyro - scan matching| heading at the same stamp
    yaw_disagree_abort_deg: float = 10.    # real 10/10 turns: p95 4.5 deg, max 7.3 deg (63 episodes)
    turn_drive_duty_pct: object = None     # creep / traverse duty; None = base_duty_pct
    turn_drive_diff_cap_pct: int = 10      # heading-hold differential |L - R|
    turn_heading_kp_pct_per_deg: float = 1.
    turn_clear_radius_m: float = 1.28      # pivot disc before a pivot (estimator 1.03 + 0.25)
    turn_clear_min_during_m: float = 1.    # pivot disc while rotating (measured swept radius <= 1.0 m)
    turn_clear_scans: int = 3
    turn_creep_max_m: float = 6.
    turn_side_drift_max_m: float = .5      # creep: lateral drift in the turn frame
    turn_side_min_m: float = .3            # traverse: clearance on the row side
    turn_side_hard_m: float = .03          # creep / traverse: either side
    turn_line_band_m: float = .6           # trunk-line band half width (turn frame)
    turn_line_gap_m: float = 4.            # a line continues across gaps up to this (next block beyond)
    turn_imu_gap_abort_s: float = .4       # IMU receipt gap while pivoting -> end (real 10/10: up to 0.365 s)
    turn_imu_dt_median_max_s: float = .04  # live IMU 33 Hz
    turn_max_pauses_per_step: int = 3
    turn_pivot_total_s: float = 30.        # pivot command time per turn
    turn_stall_loaded_total_s: float = 4.5 # pivot commanded, not turning (< stall floor), per step
    turn_bias_max_deg_s: float = .3
    turn_bias_sd_max_deg_s: float = 1.     # per-sample sd in the still window (+ standard error <= 0.1)
    turn_precheck_still_s: float = 2.
    turn_precheck_vibration_max_m_s2: float = .3
    turn_precheck_timeout_s: float = 10.
    turn_settle_still_s: float = .5
    turn_settle_vibration_max_m_s2: float = .8
    turn_settle_timeout_s: float = 4.
    turn_shape_change_extra_s: float = .32 # zero time before forward <-> pivot: |d|/100 + this
    turn_shape_change_still_s: float = .4
    turn_uncommanded_rate_deg_s: float = 5.
    turn_uncommanded_move_m: float = .1
    turn_acquire_still_s: float = 1.
    turn_acquire_timeout_s: float = 6.
    handoff_heading_max_deg: float = 10.
    handoff_lateral_max_m: float = .4
    handoff_width_tol_m: float = .5

    # ------------------------------------------------------------------------------------------
    def validate(self):
        problems = _type_problems(self)
        if not problems:
            problems = _rule_problems(self)
        if problems:
            raise ConfigError(problems)
        return self

    def feasibility(self):
        return feasibility(self)

    def to_dict(self):
        data = asdict(self)
        data['bench_pattern'] = [list(step) for step in self.bench_pattern]
        data['body_mask'] = [list(seg) for seg in self.body_mask]
        data['controller_overrides'] = dict(sorted(self.controller_overrides.items()))
        data['tuning_values'] = {k: _plain(v) for k, v in sorted(self.tuning_values.items())}
        if all(data[k] == _TURN_DEFAULTS[k] for k in TURN_KEYS):
            for k in TURN_KEYS:        # untouched turn block: digest and dump of existing configs unchanged
                del data[k]
        if all(data[k] == _SINGLE_ROW_DEFAULTS[k] for k in SINGLE_ROW_KEYS):
            for k in SINGLE_ROW_KEYS:  # untouched single-row block: the same (existing configs keep their digest)
                del data[k]
        if all(data[k] == _STEER_DEFAULTS[k] for k in STEER_KEYS):
            for k in STEER_KEYS:       # untouched steering block: the same (existing configs keep their digest)
                del data[k]
        if all(data[k] == _AUTONOMY_DEFAULTS[k] for k in AUTONOMY_KEYS):
            for k in AUTONOMY_KEYS:    # untouched autonomy block (design v2): the same
                del data[k]
        data['prior_speed_table'] = [list(point) for point in self.prior_speed_table]
        if all(getattr(self, k) == _PRIOR_DEFAULTS[k] for k in PRIOR_KEYS):
            for k in PRIOR_KEYS:       # untouched design v2 block: the same
                del data[k]
        for keys, defaults in ((LOCAL_GRID_KEYS, _LOCAL_GRID_DEFAULTS), (GNSS_KEYS, _GNSS_DEFAULTS)):
            if all(data[k] == defaults[k] for k in keys):
                for k in keys:         # untouched perception block: the same (existing configs keep their digest)
                    del data[k]
        return data

    def digest(self):
        text = json.dumps(self.to_dict(), sort_keys=True, separators=(',', ':'), allow_nan=False)
        return hashlib.sha256(text.encode('utf-8')).hexdigest()

    def estimator_overrides(self):
        overrides = dict(lidar_x_in_base_m=float(self.lidar_x_in_base_m),
                         footprint_length_m=float(self.footprint_length_m),
                         footprint_width_m=float(self.footprint_width_m),
                         corridor_every_n_scans=int(self.corridor_every_n_scans),
                         distance_mode=self.distance_mode,
                         row_end_continuation_m=(None if self.row_end_continuation_m is None
                                                 else float(self.row_end_continuation_m)))
        if self.mode == SINGLE_ROW_MODE:
            # single-row estimator options (other modes: the EstimatorConfig defaults, i.e. the original rules)
            overrides.update(single_row_estimator_fields(self))
        if is_front_steer(self) and self.steer_row_end_curve_fit:
            # front_steer: curved row ends (S1c; the differential history keeps the straight-band rule)
            overrides['row_end_curve_fit'] = True
        if is_front_steer(self) and self.steer_row_end_agree_m is not None:
            # front_steer: the two confirming full updates must agree on the end (S1c round 2, decision R53)
            overrides['row_end_agree_m'] = float(self.steer_row_end_agree_m)
        overrides.update(PS.estimator_fields(self))   # local grid fusion (empty while the gain is 0)
        return overrides

    def actuator_prior_settings(self):
        """Keyword arguments of the vendored ActuatorPriorConfig (closed_loop/actuator_prior.py), or None while
        the prior is not active (prior_active). The status age limit is the steering status one (design v2 V2-1)."""
        if not prior_active(self):
            return None
        settings = dict(speed_scale=float(self.prior_speed_scale),
                        wheelbase_m=None if self.prior_wheelbase_m is None else float(self.prior_wheelbase_m),
                        status_max_age_s=float(self.steer_status_max_age_s), weight=float(self.prior_weight),
                        predict_max_s=float(self.prior_predict_max_s), predict_max_m=float(self.prior_predict_max_m),
                        mismatch_fraction=float(self.prior_mismatch_fraction))
        if self.prior_speed_table:
            settings['speed_table'] = tuple((float(d), float(v)) for d, v in self.prior_speed_table)
        return settings

    def controller_settings(self):
        """(mode, overrides) for the vendored config_for(); freshness is shared with the Runner.

        Mode 'single_row': (SINGLE_ROW_MODE, keyword arguments of the vendored single_row.SingleRowConfig) - the
        same base / differential / arc gain / yaw / freshness values plus single_row_controller_fields()."""
        overrides = dict(base_duty_pct=int(self.base_duty_pct),
                         reduce_duty_pct=int(self.reduce_duty_pct),
                         max_differential_pct=int(self.max_differential_pct),
                         arc_yaw_deg_s_per_pct_diff=float(self.arc_yaw_deg_s_per_pct_diff),
                         max_yaw_rate_deg_s=float(self.max_yaw_rate_deg_s),
                         lidar_max_age_s=float(self.lidar_max_age_s),
                         imu_max_age_s=float(self.imu_max_age_s),
                         control_hz=float(self.control_hz))
        if self.mode == SINGLE_ROW_MODE:
            overrides.update(single_row_controller_fields(self))
        for name, value in self.controller_overrides.items():
            overrides[name] = CONTROLLER_OVERRIDE_LIMITS[name][0](value)
        return self.mode, overrides

    def guard_inner_min_pct(self):
        """Minimum duty of each wheel while driving, or None when the rule cannot/does not apply."""
        if not self.inner_rule_enabled or self.breakaway_duty_pct is None:
            return None
        return int(self.breakaway_duty_pct) + int(self.inner_margin_pct)

    def guard_rolling_inner_min_pct(self):
        """Minimum duty of each wheel once the vehicle is moving (review F2): None unless the inner rule is on
        and BOTH the static and the rolling threshold are measured."""
        if self.guard_inner_min_pct() is None or self.rolling_breakaway_duty_pct is None:
            return None
        return int(self.rolling_breakaway_duty_pct) + int(self.inner_margin_pct)


TURN_KEYS = tuple(f.name for f in fields(ExecutorConfig))[
    [f.name for f in fields(ExecutorConfig)].index('turn_assist_enabled'):]
_TURN_DEFAULTS = {f.name: f.default for f in fields(ExecutorConfig) if f.name in TURN_KEYS}
SINGLE_ROW_KEYS = tuple(f.name for f in fields(ExecutorConfig) if f.name.startswith('single_row_'))
_SINGLE_ROW_DEFAULTS = {f.name: f.default for f in fields(ExecutorConfig) if f.name in SINGLE_ROW_KEYS}
STEER_KEYS = ('vehicle_profile',) + tuple(f.name for f in fields(ExecutorConfig) if f.name.startswith('steer_'))
_STEER_DEFAULTS = {f.name: f.default for f in fields(ExecutorConfig) if f.name in STEER_KEYS}
AUTONOMY_KEYS = ('start_mode', 'operator_timeout_s', 'tuning_file', 'tuning_values')
_AUTONOMY_DEFAULTS = {f.name: (f.default_factory() if f.default_factory is not MISSING else f.default)
                      for f in fields(ExecutorConfig) if f.name in AUTONOMY_KEYS}
ONE_PRESS, HOLD_TO_RUN = 'one_press', 'hold_to_run'
START_MODES = (ONE_PRESS, HOLD_TO_RUN)
PRIOR_KEYS = tuple(f.name for f in fields(ExecutorConfig) if f.name.startswith('prior_')) + ('corridor_quality',)
_PRIOR_DEFAULTS = {f.name: f.default for f in fields(ExecutorConfig) if f.name in PRIOR_KEYS}
MAX_SPEED_TABLE_POINTS = 32
LOCAL_GRID_KEYS = tuple(f.name for f in fields(ExecutorConfig) if f.name.startswith('local_grid_'))
_LOCAL_GRID_DEFAULTS = {f.name: f.default for f in fields(ExecutorConfig) if f.name in LOCAL_GRID_KEYS}
GNSS_KEYS = tuple(f.name for f in fields(ExecutorConfig) if f.name.startswith('gnss_'))
_GNSS_DEFAULTS = {f.name: f.default for f in fields(ExecutorConfig) if f.name in GNSS_KEYS}


# --- field specification -----------------------------------------------------------------------
_REAL = 'real'
_SPEC = {
    'schema': (str,), 'profile': (str,), 'motor_output_enabled': (bool,), 'require_native_navigation': (bool,), 'bench_ack': (str,),
    'bench_pattern': ('pattern',), 'mode': (str,),
    'base_duty_pct': (int, 1, 100), 'reduce_duty_pct': (int, 1, 100), 'max_differential_pct': (int, 0, 50),
    'arc_yaw_deg_s_per_pct_diff': (_REAL, .01, 1.), 'max_yaw_rate_deg_s': (_REAL, .1, 20.),
    'controller_overrides': ('overrides',),
    'duty_cap_pct': (int, 1, 100), 'diff_cap_pct': (int, 0, 100),
    'breakaway_duty_pct': (int, 0, 100, 'nullable'), 'inner_rule_enabled': (bool,),
    'inner_margin_pct': (int, 0, 50),
    'v_measured_m_s': (_REAL, .05, 2., 'nullable'), 'max_speed_m_s': (_REAL, .05, 1.5),
    'measured_yaw_at_max_diff_deg_s': (_REAL, 0., 90., 'nullable'),
    'min_yaw_authority_deg_s': (_REAL, 0., 30.), 'yaw_authority_fraction': (_REAL, .1, 1.),
    'lidar_yaw_offset_deg': (_REAL, -180., 180.), 'imu_yaw_sign': (int, -1, 1),
    'imu_up_tau_s': (_REAL, .1, 10.), 'scan_latency_s': (_REAL, 0., .3),
    'range_min_floor_m': (_REAL, 0., .5), 'beam_grid_count': (int, 100, 4096),
    'beam_grid_angle_min_rad': (_REAL, -2 * math.pi, 2 * math.pi),
    'beam_grid_increment_rad': (_REAL, 1e-4, .1), 'body_mask': ('segments',),
    'body_mask_limit_m': (_REAL, .05, 2.), 'max_scan_trim_beams': (int, 0, 16),
    'lidar_x_in_base_m': (_REAL, -1., 1.), 'footprint_length_m': (_REAL, .3, 3.),
    'footprint_width_m': (_REAL, .3, 2.), 'corridor_every_n_scans': (int, 2, 3),
    'distance_mode': (str,),
    'row_end_continuation_m': (_REAL, 2., 20., 'nullable'),
    'near_confirm_scans': (int, 0, 4), 'near_confirm_range_m': (_REAL, .3, 5.),
    'near_confirm_tol_m': (_REAL, .05, 1.), 'near_confirm_tol_step_m': (_REAL, 0., .5),
    'guard_heading_deg': (_REAL, 5., 90., 'nullable'), 'guard_lateral_margin_m': (_REAL, -.3, 1., 'nullable'),
    'scan_stamp_mode': (str,), 'scan_stamp_window': (int, 5, 500), 'scan_stamp_reset_s': (_REAL, .05, 5.),
    'control_hz': (_REAL, CONTROL_HZ, CONTROL_HZ),
    'lidar_max_age_s': (_REAL, .1, 1.), 'imu_max_age_s': (_REAL, .05, 1.),
    'fresh_recover_s': (_REAL, .2, 10.), 'max_stale_events': (int, 0, 20),
    'stall_guard_s': (_REAL, .1, 2.), 'max_stall_guard_pauses': (int, 0, 20),
    'hold_timeout_s': (_REAL, .1, 2.), 'send_hz': (_REAL, 2., 20.),
    'terminal_stop_wait_s': (_REAL, .2, 5.),
    'backstop_margin_m': (_REAL, 0., 20.), 'backstop_time_factor': (_REAL, 1., 3.),
    'min_run_distance_m': (_REAL, .1, 50.), 'max_row_len_m': (_REAL, 1., 1000.),
    'socket_path': (str,), 'bridge_timeout_s': (_REAL, .1, 5.),
    'bind_host': (str,), 'bind_port': (int, 1024, 65535),
    # assisted headland turn (opt-in)
    'turn_assist_enabled': (bool,), 'pivot_duty_start_pct': (int, 30, 100, 'nullable'),
    'pivot_duty_cap_pct': (int, 30, 100, 'nullable'), 'pivot_high_duty_ack': (str,), 'pivot_surface_ok': (bool,),
    'pivot_duty_step_pct': (int, 1, 20), 'pivot_step_interval_s': (_REAL, .5, 3.),
    'pivot_breakaway_grace_s': (_REAL, .5, 3.), 'pivot_rotating_deg_s': (_REAL, 2., 15.),
    'pivot_rotating_confirm_s': (_REAL, .1, 1.), 'pivot_rotating_lidar_deg': (_REAL, .5, 5.),
    'pivot_stall_window_s': (_REAL, .5, 3.), 'pivot_stall_min_deg': (_REAL, .5, 5.),
    'pivot_rate_max_deg_s': (_REAL, 20., 90.), 'pivot_wrong_direction_deg': (_REAL, 1., 10.),
    'pivot_coast_s_hi': (_REAL, .1, .8), 'pivot_coast_s_lo': (_REAL, .1, .8),
    'pivot_accept_error_deg': (_REAL, 2., 15.), 'pivot_topup_max': (int, 0, 2),
    'pivot_topup_max_deg': (_REAL, 5., 45.), 'pivot_overshoot_max_deg': (_REAL, 0., 30.),
    'pivot_icr_ahead_left_m': (_REAL, -.8, .8, 'nullable'), 'pivot_icr_left_left_m': (_REAL, -.8, .8, 'nullable'),
    'pivot_icr_ahead_right_m': (_REAL, -.8, .8, 'nullable'), 'pivot_icr_left_right_m': (_REAL, -.8, .8, 'nullable'),
    'yaw_disagree_warn_deg': (_REAL, 1., 15.), 'yaw_disagree_abort_deg': (_REAL, 3., 30.),
    'turn_drive_duty_pct': (int, 1, 100, 'nullable'), 'turn_drive_diff_cap_pct': (int, 0, 20),
    'turn_heading_kp_pct_per_deg': (_REAL, 0., 5.), 'turn_clear_radius_m': (_REAL, .8, 3.),
    'turn_clear_min_during_m': (_REAL, .5, 3.), 'turn_clear_scans': (int, 1, 10),
    'turn_creep_max_m': (_REAL, .5, 10.), 'turn_side_drift_max_m': (_REAL, .1, 1.5),
    'turn_side_min_m': (_REAL, .05, 1.), 'turn_side_hard_m': (_REAL, .01, .5),
    'turn_line_band_m': (_REAL, .2, 1.5), 'turn_line_gap_m': (_REAL, 1., 8.),
    'turn_imu_gap_abort_s': (_REAL, .05, .6), 'turn_imu_dt_median_max_s': (_REAL, .01, .1),
    'turn_max_pauses_per_step': (int, 0, 10), 'turn_pivot_total_s': (_REAL, 5., 60.),
    'turn_stall_loaded_total_s': (_REAL, 1., 10.), 'turn_bias_max_deg_s': (_REAL, .05, 2.),
    'turn_bias_sd_max_deg_s': (_REAL, .05, 2.), 'turn_precheck_still_s': (_REAL, .5, 10.),
    'turn_precheck_vibration_max_m_s2': (_REAL, .05, 3.), 'turn_precheck_timeout_s': (_REAL, 2., 60.),
    'turn_settle_still_s': (_REAL, .2, 3.), 'turn_settle_vibration_max_m_s2': (_REAL, .1, 3.),
    'turn_settle_timeout_s': (_REAL, 1., 15.), 'turn_shape_change_extra_s': (_REAL, .12, 2.),
    'turn_shape_change_still_s': (_REAL, .2, 3.), 'turn_uncommanded_rate_deg_s': (_REAL, 1., 30.),
    'turn_uncommanded_move_m': (_REAL, .02, 1.), 'turn_acquire_still_s': (_REAL, .2, 5.),
    'turn_acquire_timeout_s': (_REAL, 1., 30.), 'handoff_heading_max_deg': (_REAL, 2., 30.),
    'handoff_lateral_max_m': (_REAL, .1, 1.), 'handoff_width_tol_m': (_REAL, .1, 2.),
}
# Day-1 review fixes (Fix A): separate block so that keys added by other branches merge cleanly.
_SPEC.update({
    'rolling_breakaway_duty_pct': (int, 0, 100, 'nullable'),
    'measured_stop_distance_m': (_REAL, .05, 5., 'nullable'),
})
# Calibration runs (calib_runs.py): separate block for the same reason.
_SPEC.update({
    'calib_motor_output_enabled': (bool,),
    'calib_distance_cap_m': (_REAL, 2., 20.),
    'calib_time_cap_s': (_REAL, 5., 90.),
})
# Single-row mode (block at the end of this file): separate block for the same reason.
_SPEC.update({
    'single_row_target_speed_m_s': (_REAL, .05, 1.), 'single_row_min_speed_m_s': (_REAL, .05, 1.),
    'single_row_approach_speed_m_s': (_REAL, .05, 1.),
    'single_row_base_min_pct': (int, 1, 100), 'single_row_base_max_pct': (int, 1, 100),
    'single_row_bias_max_pct': (_REAL, 0., 50.), 'single_row_start_max_deg': (_REAL, 1., 29.),
    'single_row_end_stop_offset_m': (_REAL, 0., 3.), 'single_row_max_row_length_m': (_REAL, 2., 1000.),
    'single_row_acquire_behind_m': (_REAL, 0., 7.5, 'nullable'),
    'single_row_acquire_window_deg': (_REAL, 10., 50.),
    'single_row_initial_press_grace_s': (_REAL, 0., 3.),
    'single_row_band_about_vehicle': (bool,),
    'single_row_end_stagger_gap_m': (_REAL, 1.5, 20., 'nullable'),
    'single_row_end_last_trunk': (bool,),
    'single_row_end_segment_span_m': (_REAL, .5, 10., 'nullable'),
})
# Front-wheel steering block (end of this file): separate block for the same reason.
_SPEC.update({
    'vehicle_profile': (str,),
    'steer_limit_left_mrad': (int, 1, 1570), 'steer_limit_right_mrad': (int, 1, 1570),
    'steer_rate_limit_deg_s': (_REAL, .5, 120.), 'steer_rate_limit_standstill_deg_s': (_REAL, .5, 120.),
    'steer_measured_rate_deg_s': (_REAL, 0., 200., 'nullable'),
    'steer_measured_rate_standstill_deg_s': (_REAL, 0., 200., 'nullable'),
    'steer_rate_min_deg_s': (_REAL, 1., 60.), 'steer_range_measured': (bool,),
    'steer_wheel_soft_margin_deg': (_REAL, 0., 15., 'nullable'), 'steer_min_pwm_pct': (_REAL, 0., 100., 'nullable'),
    'steer_zero_offset_deg': (_REAL, -10., 10., 'nullable'), 'steer_servo_deadband_deg': (_REAL, 0., 10., 'nullable'),
    'steer_polarity_checked': (bool,), 'steer_cal_id': (int, 1, 65535, 'nullable'), 'steer_allow_sim': (bool,),
    'steer_status_max_age_s': (_REAL, .05, 1.),
    'steer_row_end_curve_fit': (bool,),
    'steer_row_end_agree_m': (_REAL, .1, 5., 'nullable'),
    'steer_authority_fraction': (_REAL, .5, 1.), 'steer_authority_s': (_REAL, .2, 10.),
    'steer_rotation_centre_x_m': (_REAL, -1.5, 1.5), 'steer_rotation_centre_sd_m': (_REAL, 0., .5),
    'steer_bridge_kind': (str,), 'steer_esp32_mark_deg': (_REAL, 5., 40., 'nullable'),
    'steer_esp32_cal': ('esp32_cal',), 'steer_esp32_require_fw_watchdog': (bool,),
    'steer_wheelbase_m': (_REAL, .2, 3., 'nullable'),
})
# Autonomous start and operator link (design v2 V2-2): separate block for the same reason.
_SPEC.update({
    'start_mode': (str, None, None, 'nullable'), 'operator_timeout_s': (_REAL, .5, 3.),
    'tuning_file': (str, None, None, 'nullable'), 'tuning_values': ('tuning',),
})
# Actuator priors and estimator outputs (design v2 V2-1): separate block for the same reason.
_SPEC.update({
    'prior_enabled': (bool, None, None, 'nullable'), 'prior_weight': (_REAL, 0., 1.), 'prior_speed_table': ('speed_table',),
    'prior_speed_scale': (_REAL, .5, 1.3), 'prior_wheelbase_m': (_REAL, .2, 3., 'nullable'),
    'prior_predict_max_s': (_REAL, .1, 3.), 'prior_predict_max_m': (_REAL, .05, 2.),
    'prior_mismatch_fraction': (_REAL, .05, .9), 'corridor_quality': (bool,),
})

# Perception layers (local_grid_* / gnss_*, perception_settings.py): separate block for the same reason.
_SPEC.update(PS.CONFIG_SPEC)

# Vendored ControlConfig defaults (closed_loop/controllers.py; tests compare them with the vendored file).
# With motor output the field profile may not configure anything looser (review S6).
STOP_MODEL_DEFAULTS = dict(front_margin_m=.35, stop_decel_m_s2=1.07, reaction_s=.15, stop_onset_s=.25)
SIDE_RULE_DEFAULTS = dict(side_hard_margin_m=.03, side_soft_margin_m=.15, side_soft_exit_m=.20, side_confirm_scans=2)
MOTOR_GUARD_HEADING_DEG = (10., 30.)      # C1 gate P3 (25 deg); null switches the guard off
MOTOR_GUARD_LATERAL_M = (0., .3)
BODY_MASK_TOLERANCE_M = .05               # S5: a mask reaches at most this far outside the footprint
BODY_MASK_FORWARD_SECTOR_DEG = 35.        # S5: |bearing| from the travel axis that only the front edge bounds


def stopping_distance_model(speed_m_s, stop_decel_m_s2=1.07, reaction_s=.15, stop_onset_s=.25):
    """The vendored RowController.stopping_distance formula (the front margin is added separately)."""
    v = max(0., float(speed_m_s))
    return v * (reaction_s + stop_onset_s) + v * v / (2 * stop_decel_m_s2)


def _is_real(value):
    return type(value) in (int, float) and math.isfinite(value)


def _check_scalar(name, value, spec):
    kind = spec[0]
    nullable = len(spec) > 3 and spec[3] == 'nullable'
    if value is None:
        return [] if nullable else ['%s: null not allowed' % name]
    if kind is bool:
        return [] if type(value) is bool else ['%s: must be true/false' % name]
    if kind is str:
        return [] if type(value) is str and len(value) <= 256 else ['%s: must be a string' % name]
    if kind is int:
        if type(value) is not int:
            return ['%s: must be an integer' % name]
    elif kind == _REAL:
        if not _is_real(value):
            return ['%s: must be a finite number' % name]
    lo, hi = spec[1], spec[2]
    if not lo <= value <= hi:
        return ['%s: %r outside [%g, %g]' % (name, value, lo, hi)]
    return []


def _type_problems(cfg):
    problems = []
    for f in fields(cfg):
        name, value = f.name, getattr(cfg, f.name)
        spec = _SPEC[name]
        kind = spec[0]
        if kind == 'pattern':
            problems += _pattern_shape(value)
        elif kind == 'segments':
            problems += _segments_shape(value)
        elif kind == 'overrides':
            problems += _overrides_problems(value)
        elif kind == 'speed_table':
            problems += _speed_table_shape(value)
        elif kind == 'tuning':
            problems += _tuning_shape(value)
        elif kind == 'esp32_cal':
            problems += _esp32_cal_shape(value)
        else:
            problems += _check_scalar(name, value, spec)
    return problems


def _esp32_cal_shape(value):
    """steer_esp32_cal: None or [RIGHT, CENTER, LEFT, MAX_COMMAND] ints of the firmware CAL line (12-bit counts; the pot
    decreases toward the right: RIGHT < CENTER < LEFT)."""
    if value is None:
        return []
    if (not isinstance(value, (list, tuple)) or len(value) != 4 or not all(type(v) is int for v in value)):
        return ['steer_esp32_cal: must be null or [RIGHT, CENTER, LEFT, MAX_COMMAND] integers']
    right, center, left, max_command = value
    if not (0 <= right < center < left <= 4095 and 1 <= max_command <= 90):
        return ['steer_esp32_cal: need 0 <= RIGHT < CENTER < LEFT <= 4095 and 1 <= MAX_COMMAND <= 90']
    return []


def _plain(value):
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    return value


def _tuning_shape(value):
    """tuning_values: {} or the controller values of field_tuning.controller_fields (every key known, finite numbers, the
    speed table a list of [duty, speed]); the ranges and rules are field_tuning's (the loop loads the file)."""
    from .field_tuning import controller_field_names
    if not isinstance(value, dict):
        return ['tuning_values: must be an object']
    problems = ['tuning_values: %r is not a tuning value' % k for k in sorted(set(value) - controller_field_names())]
    for name, v in value.items():
        if name == 'speed_table':
            if not isinstance(v, (list, tuple)) or not all(isinstance(p, (list, tuple)) and len(p) == 2
                                                           and all(_is_real(x) for x in p) for p in v):
                problems.append('tuning_values.speed_table: a list of [duty, speed]')
        elif not _is_real(v):
            problems.append('tuning_values.%s: must be a finite number' % name)
    return problems


def with_tuning(cfg, values):
    """The executor config with the resolved field tuning values (field_tuning): tuning_values (the steering controller)
    and the actuator prior's table and scale. Locked values that live in the executor config must equal it."""
    from . import field_tuning as T
    problems = T.lock_problems(values, cfg)
    if problems:
        raise ConfigError(problems)
    prior = T.config_fields(values)
    # an explicit prior table / scale of the executor config wins (a SIM experiment, e.g. prior_checks' table x 1.3)
    if cfg.prior_speed_table:
        del prior['prior_speed_table']
    if cfg.prior_speed_scale != _PRIOR_DEFAULTS['prior_speed_scale']:
        del prior['prior_speed_scale']
    return replace(cfg, tuning_values=T.controller_fields(values), **prior).validate()


def _speed_table_shape(value):
    """prior_speed_table: () or 2 .. MAX_SPEED_TABLE_POINTS (duty_pct, speed_m_s) points, the duty rising inside
    (0, 100], the speed >= 0 and never falling with the duty (the vendored SpeedTable rule), the last speed > 0."""
    if not isinstance(value, tuple) or len(value) > MAX_SPEED_TABLE_POINTS:
        return ['prior_speed_table: must be a tuple of at most %d points' % MAX_SPEED_TABLE_POINTS]
    if not value:
        return []
    problems = []
    if len(value) < 2:
        problems.append('prior_speed_table: needs at least two points')
    previous = None
    for k, point in enumerate(value):
        if not isinstance(point, tuple) or len(point) != 2 or not _is_real(point[0]) or not _is_real(point[1]):
            problems.append('prior_speed_table[%d]: must be (duty_pct, speed_m_s)' % k)
            previous = None
            continue
        if not (0 < point[0] <= 100 and 0 <= point[1] <= 3.):
            problems.append('prior_speed_table[%d]: duty outside (0, 100] or speed outside [0, 3] m/s' % k)
        if previous is not None and not (point[0] > previous[0] and point[1] >= previous[1]):
            problems.append('prior_speed_table[%d]: the duty must rise and the speed must not fall with the duty' % k)
        previous = point
    if not problems and not value[-1][1] > 0:
        problems.append('prior_speed_table: the last speed must be > 0')
    return problems


def _pattern_shape(value):
    if not isinstance(value, tuple) or len(value) > MAX_BENCH_STEPS:
        return ['bench_pattern: must be a tuple of at most %d steps' % MAX_BENCH_STEPS]
    problems = []
    for k, step in enumerate(value):
        if (not isinstance(step, tuple) or len(step) != 3 or not _is_real(step[0])
                or type(step[1]) is not int or type(step[2]) is not int):
            problems.append('bench_pattern[%d]: must be (duration_s, int left, int right)' % k)
        elif not 0 < step[0] <= MAX_BENCH_STEP_S:
            problems.append('bench_pattern[%d]: duration outside (0, %g] s' % (k, MAX_BENCH_STEP_S))
    return problems


def _segments_shape(value):
    if not isinstance(value, tuple) or len(value) > MAX_MASK_SEGMENTS:
        return ['body_mask: must be a tuple of at most %d segments' % MAX_MASK_SEGMENTS]
    problems = []
    for k, seg in enumerate(value):
        if (not isinstance(seg, tuple) or len(seg) != 3 or type(seg[0]) is not int
                or type(seg[1]) is not int or not _is_real(seg[2])):
            problems.append('body_mask[%d]: must be (int first, int last, max_range_m)' % k)
    return problems


def _overrides_problems(value):
    if not isinstance(value, dict):
        return ['controller_overrides: must be an object']
    problems = []
    for name, v in value.items():
        if name not in CONTROLLER_OVERRIDE_LIMITS:
            problems.append('controller_overrides.%s: not an allowed override' % name)
            continue
        kind, lo, hi = CONTROLLER_OVERRIDE_LIMITS[name]
        ok = type(v) is int if kind is int else _is_real(v)
        if not ok or not lo <= v <= hi:
            problems.append('controller_overrides.%s: %r outside [%g, %g]' % (name, v, lo, hi))
    return problems


def _rule_problems(cfg):
    p = []
    if cfg.schema != SCHEMA:
        p.append('schema: expected %s' % SCHEMA)
    if cfg.profile not in PROFILES:
        p.append('profile: must be one of %s' % (PROFILES,))
    if cfg.profile == 'field' and (cfg.bench_pattern or cfg.bench_ack):
        p.append('profile field: bench_pattern/bench_ack are not allowed (bench patterns bypass the controller)')
    if cfg.profile == 'bench' and cfg.bench_ack != BENCH_ACK:
        p.append('profile bench: bench_ack must be %r' % BENCH_ACK)
    if cfg.mode not in CONTROLLER_MODES:
        p.append('mode: only forward-only continuous modes %s are allowed' % (CONTROLLER_MODES,))
    if cfg.distance_mode not in ('scan_norm_legacy', 'resolved_polyline'):
        p.append('distance_mode: must be scan_norm_legacy or resolved_polyline')
    if cfg.scan_stamp_mode not in SCAN_STAMP_MODES:
        p.append('scan_stamp_mode: must be one of %s' % (SCAN_STAMP_MODES,))
    if cfg.imu_yaw_sign not in (-1, 1):
        p.append('imu_yaw_sign: must be -1 or +1')
    base, diff, red = cfg.base_duty_pct, cfg.max_differential_pct, cfg.reduce_duty_pct
    if base + diff > cfg.duty_cap_pct:
        p.append('duty_cap_pct %d < base + max_differential = %d (guard would trip on normal output)'
                 % (cfg.duty_cap_pct, base + diff))
    if red + diff > cfg.duty_cap_pct:
        p.append('duty_cap_pct %d < reduce + max_differential = %d' % (cfg.duty_cap_pct, red + diff))
    if 2 * diff > cfg.diff_cap_pct:
        p.append('diff_cap_pct %d < 2 * max_differential = %d' % (cfg.diff_cap_pct, 2 * diff))
    if base - diff < 1 or red - diff < 1:
        p.append('base/reduce - max_differential must leave the inner wheel driving (>= 1)')
    feas = feasibility(cfg)
    p += feas['problems']
    for k, (first, last, rng) in enumerate(cfg.body_mask):
        if not 0 <= first <= last < cfg.beam_grid_count:
            p.append('body_mask[%d]: beam range outside [0, %d)' % (k, cfg.beam_grid_count))
        if not 0 < rng <= cfg.body_mask_limit_m:
            p.append('body_mask[%d]: max_range %r outside (0, body_mask_limit_m]' % (k, rng))
    p += _body_mask_footprint_problems(cfg)
    p += calib_rule_problems(cfg)
    if cfg.row_end_continuation_m is not None and cfg.row_end_continuation_m <= 1.5:
        p.append('row_end_continuation_m: must exceed the 1.5 m end_free gap')
    side = dict(hard=.03, soft=.15, exit=.20)
    for key, name in (('hard', 'side_hard_margin_m'), ('soft', 'side_soft_margin_m'), ('exit', 'side_soft_exit_m')):
        side[key] = cfg.controller_overrides.get(name, side[key])
    if not side['hard'] < side['soft'] < side['exit']:
        p.append('side margins: need hard < soft < exit')
    enter = cfg.controller_overrides.get('enter_error_deg', 2.5)
    exit_ = cfg.controller_overrides.get('exit_error_deg', 1.)
    if exit_ >= enter:
        p.append('exit_error_deg must be below enter_error_deg')
    problem = _bind_host_problem(cfg.bind_host)
    if problem:
        p.append(problem)
    if not cfg.socket_path.startswith('/'):
        p.append('socket_path: must be absolute')
    if cfg.max_row_len_m <= cfg.backstop_margin_m + cfg.min_run_distance_m:
        p.append('max_row_len_m too small for backstop_margin_m + min_run_distance_m')
    if cfg.hold_timeout_s >= .75:
        p.append('hold_timeout_s must stay below the bridge input TTL 0.75 s')
    if 1. / cfg.send_hz >= .75 / 2:
        p.append('send_hz too low for the bridge input TTL 0.75 s')
    guard_min = cfg.guard_inner_min_pct()
    for k, (dur, left, right) in enumerate(cfg.bench_pattern):
        reason = _bench_step_problem(cfg, left, right, guard_min)
        if reason:
            p.append('bench_pattern[%d]: %s' % (k, reason))
    if sum(step[0] for step in cfg.bench_pattern) > MAX_BENCH_TOTAL_S:
        p.append('bench_pattern: total duration above %g s' % MAX_BENCH_TOTAL_S)
    p += _turn_rule_problems(cfg)
    p += single_row_rule_problems(cfg)
    p += steer_rule_problems(cfg)
    p += autonomy_rule_problems(cfg)
    p += prior_rule_problems(cfg)
    p += PS.rule_problems(cfg)
    return p


def _bench_step_problem(cfg, left, right, guard_min):
    if left < 0 or right < 0:
        return 'negative duty (reverse/pivot) not allowed'
    if max(left, right) > cfg.duty_cap_pct:
        return 'duty above duty_cap_pct'
    if abs(left - right) > cfg.diff_cap_pct:
        return 'differential above diff_cap_pct'
    if guard_min is not None and (left or right) and min(left, right) < guard_min:
        return 'inner wheel below breakaway + inner_margin'
    return None


C7_UNKNOWN = 'measured_stop_distance_m unknown (C7: executor 0/0 stop, calibration run kind stop)'


def feasibility(cfg):
    """Static parameter feasibility: status 'ok' / 'unknown' / 'infeasible' with every problem.

    'unknown' (shadow only): a measurement needed to judge is missing. With motor output enabled
    every missing measurement is a problem and the status is 'infeasible'.
    """
    problems, unknown = [], []
    base, diff, red = cfg.base_duty_pct, cfg.max_differential_pct, cfg.reduce_duty_pct
    steer = cfg.vehicle_profile == FRONT_STEER
    if steer:
        # front-wheel steering (design 6.6 table): no inner wheel (equal duty both sides), no C5 arc authority; the
        # drive-side measurements stay (C2 breakaway, C3 speed, C7 stop) and the steering GO rule replaces the yaw rule
        if cfg.breakaway_duty_pct is None:
            unknown.append('breakaway_duty_pct unknown (C2)')
        go_problems, go_unknown = front_steer_go_problems(cfg)
        problems += go_problems
        unknown += go_unknown
    elif cfg.inner_rule_enabled:
        if cfg.breakaway_duty_pct is None:
            unknown.append('breakaway_duty_pct unknown (C2)')
        elif cfg.rolling_breakaway_duty_pct is not None:
            problems += _rolling_inner_problems(cfg)
        else:
            need = cfg.breakaway_duty_pct + cfg.inner_margin_pct
            if base - diff < need:
                problems.append('inner wheel base-max_differential = %d < breakaway+margin = %d'
                                % (base - diff, need))
            if red - diff < need:
                problems.append('inner wheel reduce-max_differential = %d < breakaway+margin = %d'
                                % (red - diff, need))
        if cfg.breakaway_duty_pct is not None and cfg.mode == SINGLE_ROW_MODE:
            problems += single_row_inner_problems(cfg)
    if cfg.v_measured_m_s is None:
        unknown.append('v_measured_m_s unknown (C3)')
    elif cfg.v_measured_m_s > cfg.max_speed_m_s:
        problems.append('v_measured_m_s %.3f above max_speed_m_s %.3f' % (cfg.v_measured_m_s, cfg.max_speed_m_s))
    yaw = cfg.measured_yaw_at_max_diff_deg_s
    if steer:
        pass                                       # no differential yaw authority on a steered vehicle
    elif yaw is None:
        unknown.append('measured_yaw_at_max_diff_deg_s unknown (C5)')
    else:
        if yaw < cfg.min_yaw_authority_deg_s:
            problems.append('yaw authority %.2f deg/s below %.2f' % (yaw, cfg.min_yaw_authority_deg_s))
        if cfg.max_yaw_rate_deg_s > cfg.yaw_authority_fraction * yaw + 1e-9:
            problems.append('max_yaw_rate_deg_s %.2f above %.2f x measured authority %.2f'
                            % (cfg.max_yaw_rate_deg_s, cfg.yaw_authority_fraction, yaw))
    if cfg.measured_stop_distance_m is None:
        unknown.append(C7_UNKNOWN)
    if cfg.motor_output_enabled and cfg.profile == 'field':
        problems += ['motor_output_enabled needs ' + u for u in unknown]
        if not cfg.inner_rule_enabled and not steer:
            problems.append('motor_output_enabled needs inner_rule_enabled (no driving below breakaway)')
        problems += _motor_field_problems(cfg)
        problems += motor_steering_problems(cfg)    # steering GO rule, row output only (field decision rules block)
                                                    # (the differential G3 / G3b; [] with vehicle_profile front_steer)
        problems += single_row_motor_problems(cfg)  # single-row block (end of this file); [] in the other modes
        unknown = []
    # profile 'bench' (wheels off the ground, fixed duty pattern, no controller): the field measurements are not
    # needed; the pattern still passes the OutputGuard caps, the hold gating and the freshness rule.
    status = 'infeasible' if problems else ('unknown' if unknown else 'ok')
    return dict(status=status, problems=problems, unknown=unknown)


# --- day-1 review fixes (Fix A) --------------------------------------------------------------------
_FIELD_DEFAULTS = {f.name: f.default for f in fields(ExecutorConfig)}


def _rolling_inner_problems(cfg):
    """F2: inner-wheel rules when the rolling threshold is measured (the static one is measured too)."""
    static, rolling, margin = cfg.breakaway_duty_pct, cfg.rolling_breakaway_duty_pct, cfg.inner_margin_pct
    if rolling > static:
        return ['rolling_breakaway_duty_pct %d above breakaway_duty_pct %d (a turning wheel can not need more than '
                'a standing one: re-measure C2 / C3b)' % (rolling, static)]
    problems = []
    for name, duty in (('base', cfg.base_duty_pct), ('reduce', cfg.reduce_duty_pct)):
        if duty - cfg.max_differential_pct < rolling + margin:
            problems.append('inner wheel %s-max_differential = %d < rolling_breakaway+margin = %d'
                            % (name, duty - cfg.max_differential_pct, rolling + margin))
        if duty < static + margin:
            problems.append('%s_duty_pct %d < breakaway+margin = %d (a start from standstill needs both wheels at '
                            'the static threshold)' % (name, duty, static + margin))
    return problems


def _motor_field_problems(cfg):
    """S6: with motor output in the field profile no safety setting may be switched off or loosened."""
    problems = []
    for name, (lo, hi) in (('guard_heading_deg', MOTOR_GUARD_HEADING_DEG),
                           ('guard_lateral_margin_m', MOTOR_GUARD_LATERAL_M)):
        value = getattr(cfg, name)
        if value is None:
            problems.append('motor_output_enabled needs %s (geometry guard, C1 gate P3; null switches it off)' % name)
        elif not lo <= value <= hi:
            problems.append('motor_output_enabled: %s %r outside [%g, %g]' % (name, value, lo, hi))
    overrides, d = cfg.controller_overrides, STOP_MODEL_DEFAULTS
    model = {k: overrides.get(k, v) for k, v in d.items()}
    looser = [k for k, worse in (('front_margin_m', model['front_margin_m'] < d['front_margin_m']),
                                 ('stop_decel_m_s2', model['stop_decel_m_s2'] > d['stop_decel_m_s2']),
                                 ('reaction_s', model['reaction_s'] < d['reaction_s']),
                                 ('stop_onset_s', model['stop_onset_s'] < d['stop_onset_s'])) if worse]
    measured, v = cfg.measured_stop_distance_m, cfg.v_measured_m_s
    if looser and measured is None:
        problems.append('motor_output_enabled: stop model %s looser than the defaults needs measured_stop_distance_m '
                        '(C7)' % '/'.join(looser))
    if measured is not None and v is not None:
        dist = stopping_distance_model(v, model['stop_decel_m_s2'], model['reaction_s'], model['stop_onset_s'])
        if dist + 1e-9 < measured:
            problems.append('motor_output_enabled: stop model %.3f m at v_measured %.2f m/s is shorter than the '
                            'measured C7 stop distance %.3f m' % (dist, v, measured))
        if dist + model['front_margin_m'] + 1e-9 < measured + d['front_margin_m']:
            problems.append('motor_output_enabled: stop model + front_margin_m = %.3f m is below the measured C7 stop '
                            'distance + %.2f m = %.3f m' % (dist + model['front_margin_m'], d['front_margin_m'],
                                                            measured + d['front_margin_m']))
    for name, default in SIDE_RULE_DEFAULTS.items():
        value = overrides.get(name, default)
        if (value > default) if name == 'side_confirm_scans' else (value < default):
            problems.append('motor_output_enabled: %s %r looser than the default %g' % (name, value, default))
    for name in ('lidar_max_age_s', 'imu_max_age_s'):
        if getattr(cfg, name) > _FIELD_DEFAULTS[name]:
            problems.append('motor_output_enabled: %s %r looser than the default %g'
                            % (name, getattr(cfg, name), _FIELD_DEFAULTS[name]))
    problems += day2_motor_problems(cfg)          # review_safety_day2 N1 (day-2 block below)
    return problems


def _bind_host_problem(host):
    """None for a specific IPv4 unicast address literal, else the problem (review S6)."""
    if not isinstance(host, str) or not host.strip():
        return 'bind_host: refuse wildcard binds (empty; the operator page must stay on the AP / LAN address)'
    try:
        addr = ipaddress.ip_address(host)
    except ValueError:
        return ("bind_host: %r is not an IP address literal (names and shorthand forms are refused; '0' is the "
                "wildcard 0.0.0.0 on Linux)" % host)
    mapped = getattr(addr, 'ipv4_mapped', None)
    if addr.is_unspecified or (mapped is not None and mapped.is_unspecified):
        return 'bind_host: refuse wildcard binds (%s; the operator page must stay on the AP / LAN address)' % host
    if addr.version != 4:
        return 'bind_host: %s is not IPv4 (the operator page server is IPv4 only)' % host
    if addr.is_multicast or addr == ipaddress.IPv4Address('255.255.255.255'):
        return 'bind_host: %s is not a unicast address' % host
    return None


def footprint_exit_range(bearing_rad, lidar_x_m, length_m, width_m):
    """Distance from the LiDAR at base (lidar_x_m, 0) along a beam of the given bearing (0 = travel axis, + = left)
    to where it leaves the footprint rectangle |x| <= length/2, |y| <= width/2 (the estimator's base frame:
    origin at the footprint centre); 0 when the beam does not cross the footprint."""
    c, s = math.cos(bearing_rad), math.sin(bearing_rad)
    t_in, t_out = -math.inf, math.inf
    for origin, direction, half in ((lidar_x_m, c, .5 * length_m), (0., s, .5 * width_m)):
        if abs(direction) < 1e-12:
            if not -half <= origin <= half:
                return 0.
            continue
        a, b = (-half - origin) / direction, (half - origin) / direction
        t_in, t_out = max(t_in, min(a, b)), min(t_out, max(a, b))
    if t_out < max(t_in, 0.):
        return 0.
    return t_out


def body_mask_beam_limits(cfg):
    """S5, per beam of the mask grid: (largest range allowed by the footprint boundary rule, largest range allowed by
    the forward-sector rule (inf outside the sector), bearing from the travel axis in deg)."""
    offset = math.radians(cfg.lidar_yaw_offset_deg)
    front = max(0., .5 * cfg.footprint_length_m - cfg.lidar_x_in_base_m)
    boundary, forward, bearings = [], [], []
    for i in range(cfg.beam_grid_count):
        a = cfg.beam_grid_angle_min_rad + i * cfg.beam_grid_increment_rad + offset
        bearing = math.atan2(math.sin(a), math.cos(a))
        boundary.append(footprint_exit_range(bearing, cfg.lidar_x_in_base_m, cfg.footprint_length_m,
                                             cfg.footprint_width_m) + BODY_MASK_TOLERANCE_M)
        in_sector = abs(math.degrees(bearing)) <= BODY_MASK_FORWARD_SECTOR_DEG
        forward.append(front / math.cos(bearing) + BODY_MASK_TOLERANCE_M if in_sector else math.inf)
        bearings.append(math.degrees(bearing))
    return boundary, forward, bearings


def _body_mask_footprint_problems(cfg):
    if not cfg.body_mask:
        return []
    boundary, forward, bearings = body_mask_beam_limits(cfg)
    problems = []
    for k, (first, last, rng) in enumerate(cfg.body_mask):
        if not 0 <= first <= last < cfg.beam_grid_count:
            continue                                     # reported by the beam range rule
        for limits, what in ((boundary, 'reaches beyond the footprint boundary + %.2f m' % BODY_MASK_TOLERANCE_M),
                             (forward, 'hides the forward sector (|bearing| <= %g deg) beyond the front footprint '
                                       'edge + %.2f m' % (BODY_MASK_FORWARD_SECTOR_DEG, BODY_MASK_TOLERANCE_M))):
            part = limits[first:last + 1]
            low = min(part)
            if rng > low + 1e-9:
                i = first + part.index(low)
                problems.append('body_mask[%d]: max_range %.3f m %s (%.3f m at beam %d, bearing %.1f deg)'
                                % (k, rng, what, low, i, bearings[i]))
    return problems


# --- calibration runs (calib_runs.py) ---------------------------------------------------------------
def calib_rule_problems(cfg):
    """calib_motor_output_enabled: field profile only, and every S6 motor safety rule (geometry guard, stop model,
    side rule, freshness) must hold, as for row output. The calibration-specific limits (minimum duty = measured
    breakaway or 40, duty/diff caps, distance/time caps, yaw/heading/overspeed checks) are in calib_runs.make_plan
    and CalibRun. The measurements row output needs (C2/C3/C5) are NOT required: C5 is measured by these runs."""
    if not cfg.calib_motor_output_enabled:
        return []
    if cfg.profile != 'field':
        return ['calib_motor_output_enabled: field profile only (the bench has its own pattern)']
    return (['calib_motor_output_enabled (same rule as motor output): ' + problem for problem in _motor_field_problems(cfg)]
            + calib_day2_problems(cfg))


def calib_output_problems(cfg):
    """Problems that refuse an Arm for a calibration run (empty = allowed)."""
    if not cfg.calib_motor_output_enabled:
        return ['calib_motor_output_enabled is false in the config']
    return calib_rule_problems(cfg)


# --- on-site parameter changes (operator page, AP-mode field loop) ------------------------------
ONSITE_KEYS = ('base_duty_pct', 'reduce_duty_pct', 'max_differential_pct')
ONSITE_LIMITS = dict(base_duty_pct=(30, 70), reduce_duty_pct=(30, 70), max_differential_pct=(0, 25))


def onsite_update(file_cfg, current_cfg, params):
    """current_cfg with the operator's base/reduce/differential, validated; ConfigError lists every problem.

    Bounds: ONSITE_LIMITS, the configured caps (duty_cap_pct / diff_cap_pct are never raised here)
    and every validate()/feasibility rule (inner wheel >= breakaway + margin, ...). With motor output
    enabled the values may only stay at or below the FILE's base duty and differential: the measured
    speed (v_measured_m_s: time backstop, speed limit) and yaw authority were measured there, so a lower
    base keeps the speed at or below the measured one (the time backstop can then only stop early) and a
    lower differential never commands more steering than was measured. Raising them needs a new
    measurement and a config file edit (read-only mount) + restart. The same holds for a config that enables
    calibration output (calib_motor_output_enabled): the calibration tables are built around the base duty.
    """
    if not isinstance(params, dict) or not params:
        raise ConfigError(['onsite: a non-empty object of %s is required' % (ONSITE_KEYS,)])
    problems = ['onsite: %s is not an on-site parameter' % k for k in sorted(set(params) - set(ONSITE_KEYS))]
    for key in ONSITE_KEYS:
        if key not in params:
            continue
        value, (lo, hi) = params[key], ONSITE_LIMITS[key]
        if type(value) is not int or not lo <= value <= hi:
            problems.append('onsite %s: %r outside [%d, %d]' % (key, value, lo, hi))
    if not problems and (file_cfg.motor_output_enabled or file_cfg.calib_motor_output_enabled):
        for key in ('base_duty_pct', 'max_differential_pct'):
            if params.get(key, getattr(current_cfg, key)) > getattr(file_cfg, key):
                problems.append('onsite %s: motor output allows only <= the measured file value %d'
                                % (key, getattr(file_cfg, key)))
    if problems:
        raise ConfigError(problems)
    if current_cfg.motor_output_enabled != file_cfg.motor_output_enabled or current_cfg.profile != file_cfg.profile:
        raise ConfigError(['onsite: current config does not derive from the file config'])
    new = replace(current_cfg, **{k: params[k] for k in ONSITE_KEYS if k in params})
    return onsite_yaw_authority(file_cfg, new).validate()      # review_correctness_day2 R2 (day-2 block below)

# --- assisted headland turn (opt-in) --------------------------------------------------------------
PIVOT_HIGH_DUTY_ACK = 'USER_APPROVED_HIGH_PIVOT_DUTY'


def turn_drive_duty(cfg):
    """Forward duty of the turn's creep / traverse steps (turn_drive_duty_pct, else base_duty_pct)."""
    return int(cfg.turn_drive_duty_pct if cfg.turn_drive_duty_pct is not None else cfg.base_duty_pct)


def _turn_rule_problems(cfg):
    """Static range rules of the turn block (independent of turn_assist_enabled)."""
    p = []
    start, cap = cfg.pivot_duty_start_pct, cfg.pivot_duty_cap_pct
    if start is not None and cap is not None and start > cap:
        p.append('pivot_duty_start_pct %d above pivot_duty_cap_pct %d' % (start, cap))
    if cap is not None and cap > 90 and cfg.pivot_high_duty_ack != PIVOT_HIGH_DUTY_ACK:
        p.append('pivot_duty_cap_pct above 90 needs pivot_high_duty_ack = %r (user decision)' % PIVOT_HIGH_DUTY_ACK)
    if cfg.pivot_high_duty_ack not in ('', PIVOT_HIGH_DUTY_ACK):
        p.append('pivot_high_duty_ack: must be empty or %r' % PIVOT_HIGH_DUTY_ACK)
    if cfg.pivot_coast_s_lo > cfg.pivot_coast_s_hi:
        p.append('pivot_coast_s_lo must not exceed pivot_coast_s_hi')
    if not cfg.yaw_disagree_warn_deg < cfg.yaw_disagree_abort_deg:
        p.append('yaw_disagree_warn_deg must be below yaw_disagree_abort_deg')
    if not cfg.pivot_accept_error_deg < cfg.pivot_topup_max_deg:
        p.append('pivot_accept_error_deg must be below pivot_topup_max_deg')
    if cfg.pivot_overshoot_max_deg < cfg.pivot_accept_error_deg:
        p.append('pivot_overshoot_max_deg must not be below pivot_accept_error_deg')
    if cfg.turn_clear_min_during_m > cfg.turn_clear_radius_m:
        p.append('turn_clear_min_during_m must not exceed turn_clear_radius_m')
    if cfg.pivot_breakaway_grace_s < cfg.pivot_step_interval_s:
        p.append('pivot_breakaway_grace_s must be >= pivot_step_interval_s (one ladder step per grace)')
    for side in ('left', 'right'):
        ahead, left = getattr(cfg, 'pivot_icr_ahead_%s_m' % side), getattr(cfg, 'pivot_icr_left_%s_m' % side)
        if (ahead is None) != (left is None):
            p.append('pivot_icr_*_%s_m: set both (ahead, left) or neither' % side)
    if cfg.turn_assist_enabled and cfg.profile != 'field':
        p.append('turn_assist_enabled: only in the field profile (never with a bench pattern)')
    # Turn measurements do NOT invalidate the config (the row executor stays usable when a headland
    # surface fails C6): turn_feasibility()['output_allowed'] gates every turn output at TurnAssist.start.
    return p


BODY_MASK_FOOTPRINT_TOL_M = .02


def body_mask_blind_beams(cfg, tol_m=BODY_MASK_FOOTPRINT_TOL_M):
    """Grid beams whose body mask hides returns OUTSIDE the footprint rectangle (+ tol_m).

    A masked beam turns every return nearer than its mask range into 'no return' (ScanAdapter). The
    LiDAR (lidar_x_in_base_m ahead of the base centre) is inside the footprint, so the hidden stretch of
    a beam leaves the footprint exactly when the end of the mask range lies outside it. Base-frame beam
    direction = grid angle + lidar_yaw_offset_deg (the mount the adapter undoes). Empty mask -> [].
    """
    out = set()
    half_l, half_w = cfg.footprint_length_m / 2. + tol_m, cfg.footprint_width_m / 2. + tol_m
    offset = math.radians(cfg.lidar_yaw_offset_deg)
    for first, last, max_range in cfg.body_mask:
        for i in range(int(first), int(last) + 1):
            a = cfg.beam_grid_angle_min_rad + i * cfg.beam_grid_increment_rad + offset
            x = cfg.lidar_x_in_base_m + max_range * math.cos(a)
            y = max_range * math.sin(a)
            if abs(x) > half_l or abs(y) > half_w:
                out.add(i)
    return sorted(out)


def turn_feasibility(cfg):
    """Turn feasibility: 'ok' / 'unknown' (observe and dry runs only) / 'infeasible'.

    With turn_assist_enabled AND motor_output_enabled every unknown is a problem: the C6 pivot values
    on this headland surface, the C2 breakaway (inner wheel of the heading hold) and the C3 speed
    (stop model / traverse time backstop) must be measured. output_allowed is True only then and with no
    problem; TurnAssist.start(motor_output=True) refuses otherwise (the config itself stays valid).
    """
    problems, unknown = [], []
    if cfg.pivot_duty_start_pct is None or cfg.pivot_duty_cap_pct is None:
        unknown.append('pivot_duty_start_pct / pivot_duty_cap_pct unknown (C6)')
    if not cfg.pivot_surface_ok:
        unknown.append('pivot_surface_ok false (C6 on this headland surface)')
    if cfg.breakaway_duty_pct is None:
        unknown.append('breakaway_duty_pct unknown (C2)')
    else:
        need = cfg.breakaway_duty_pct + cfg.inner_margin_pct
        low = turn_drive_duty(cfg) - cfg.turn_drive_diff_cap_pct // 2
        if low < need:
            problems.append('turn inner wheel %d < breakaway + margin %d' % (low, need))
    if cfg.v_measured_m_s is None:
        unknown.append('v_measured_m_s unknown (C3)')
    drive, half = turn_drive_duty(cfg), cfg.turn_drive_diff_cap_pct // 2
    if drive - half < 1:
        problems.append('turn drive duty - turn_drive_diff_cap_pct/2 must leave both wheels driving')
    if drive + cfg.turn_drive_diff_cap_pct > cfg.duty_cap_pct:
        problems.append('turn drive duty + turn_drive_diff_cap_pct %d above duty_cap_pct %d'
                        % (drive + cfg.turn_drive_diff_cap_pct, cfg.duty_cap_pct))
    if not cfg.inner_rule_enabled:
        unknown.append('inner_rule_enabled false')
    blind = body_mask_blind_beams(cfg)
    if blind:
        # The pivot disc (turn_clear_radius_m / turn_clear_min_during_m) is checked on the masked scan:
        # a mask that hides returns outside the footprint makes part of the disc invisible (review fix).
        problems.append('body_mask hides %d beams beyond the footprint (pivot disc partly blind)' % len(blind))
    enabled = cfg.turn_assist_enabled and cfg.motor_output_enabled
    if enabled:
        problems += ['turn output needs ' + u for u in unknown]
        unknown = []
    status = 'infeasible' if problems else ('unknown' if unknown else 'ok')
    return dict(status=status, problems=problems, unknown=unknown, output_allowed=bool(enabled and not problems))


def icr_base_from_lidar(along_lidar_m, left_lidar_m, lidar_x_in_base_m):
    """Pivot rotation centre (ICR) in the ONE convention of pivot_icr_ahead_<side>_m / pivot_icr_left_<side>_m.

    Frame: the estimator's base frame, origin = footprint centre, x = travel direction (ahead +), y = left (+),
    z up (right-handed). <side> = the pivot direction: 'left' = CCW = +yaw (left wheels backwards, right wheels
    forwards, TurnGuard direction +1), 'right' = CW. turn_assist._pivot2_shift rotates the base about this point.
    tools/field_calib (steps.icr_along_left) measures the centre relative to the LiDAR origin on the travel axis:
    along_lidar_m ahead of the LiDAR (+), left_lidar_m to the left (+; a mirrored scan frame is already undone there).
    The LiDAR sits lidar_x_in_base_m ahead of the base centre on the travel axis, so the base-frame centre is
    (along_lidar_m + lidar_x_in_base_m, left_lidar_m). (The SIM plants of turn_sim_in_loop / the tests use their own
    internal 'ICR behind the LiDAR' distance; it is never written to a config.)"""
    return float(along_lidar_m) + float(lidar_x_in_base_m), float(left_lidar_m)


# --- JSON -------------------------------------------------------------------------------------
def from_dict(data):
    if not isinstance(data, dict):
        raise ConfigError(['config: top level must be an object'])
    names = {f.name for f in fields(ExecutorConfig)}
    unknown = sorted(set(data) - names)
    if unknown:
        raise ConfigError(['unknown key: %s' % k for k in unknown])
    values = dict(data)
    problems = []
    if 'bench_pattern' in values:
        values['bench_pattern'] = _tuple_of_tuples(values['bench_pattern'], 'bench_pattern', problems)
    if 'body_mask' in values:
        values['body_mask'] = _tuple_of_tuples(values['body_mask'], 'body_mask', problems)
    if 'prior_speed_table' in values:
        values['prior_speed_table'] = _tuple_of_tuples(values['prior_speed_table'], 'prior_speed_table', problems)
    if 'controller_overrides' in values and isinstance(values['controller_overrides'], dict):
        values['controller_overrides'] = dict(values['controller_overrides'])
    if 'tuning_values' in values and isinstance(values['tuning_values'], dict):
        values['tuning_values'] = dict(values['tuning_values'])
    if isinstance(values.get('steer_esp32_cal'), list):
        values['steer_esp32_cal'] = tuple(values['steer_esp32_cal'])
    if problems:
        raise ConfigError(problems)
    return ExecutorConfig(**values).validate()


def _tuple_of_tuples(value, name, problems):
    if not isinstance(value, (list, tuple)):
        problems.append('%s: must be a list' % name)
        return ()
    out = []
    for item in value:
        if not isinstance(item, (list, tuple)):
            problems.append('%s: every entry must be a list' % name)
            return ()
        out.append(tuple(item))
    return tuple(out)


def _reject_constant(token):
    raise ConfigError(['config: non-finite number %s' % token])


def _reject_duplicates(pairs):
    seen = {}
    for key, value in pairs:
        if key in seen:
            raise ConfigError(['config: duplicate key %s' % key])
        seen[key] = value
    return seen


def loads(text):
    try:
        data = json.loads(text, parse_constant=_reject_constant, object_pairs_hook=_reject_duplicates)
    except json.JSONDecodeError as exc:
        raise ConfigError(['config: invalid JSON (%s)' % exc]) from None
    return from_dict(data)


def load_config(path, expected_profile='field'):
    """Load + validate; the caller states which profile it is running (field refuses bench)."""
    with open(path, 'r', encoding='utf-8') as fh:
        cfg = loads(fh.read())
    if cfg.profile != expected_profile:
        raise ConfigError(['profile %r loaded where %r was expected' % (cfg.profile, expected_profile)])
    return cfg


def dumps(cfg):
    return json.dumps(cfg.to_dict(), indent=1, sort_keys=True, allow_nan=False) + '\n'


# --- day-2 review fixes (review_safety_day2 N1/N2, review_correctness_day2 R2; separate block for clean merges) -------
# N1: with motor output (row runs and calibration runs) no setting below may be loosened beyond the evaluated field
# default. Each entry: (name, direction, bound); 'max' = the value may not exceed bound, 'min' = not below it.
DAY2_MOTOR_LIMITS = (
    # near-return confirmation (C1 gate P1): more scans / a longer range / a tighter tolerance hide real obstacles
    # (review probe F2b: tol 0.05, step 0, 4 scans -> a static obstacle invisible in every scan under 1.5 m)
    ('near_confirm_scans', 'max', 2), ('near_confirm_range_m', 'max', 2.), ('near_confirm_tol_m', 'min', .35),
    ('near_confirm_tol_step_m', 'min', .10),
    ('range_min_floor_m', 'max', .05),                  # 0.5 deletes every return of the vehicle's own side zone
    ('backstop_margin_m', 'min', 2.), ('backstop_time_factor', 'max', 1.15),   # MVP-1 stop rule L-2 m, 1.15 x
    ('max_speed_m_s', 'max', .55),                      # decision table O5 is the largest considered (0.55 m/s)
    ('stall_guard_s', 'max', .35), ('max_stall_guard_pauses', 'max', 3), ('max_stale_events', 'max', 3),
    ('bridge_timeout_s', 'max', .6), ('hold_timeout_s', 'max', .5), ('fresh_recover_s', 'min', 1.),
    ('inner_margin_pct', 'min', 10), ('yaw_authority_fraction', 'max', .6), ('min_yaw_authority_deg_s', 'min', 2.),
    ('footprint_length_m', 'min', 1.), ('footprint_width_m', 'min', .55),      # catalog 1.22 x 0.70 (tape at C0)
)
# vendored ControlConfig defaults that bound a stop or a no-response decision (tests compare them with the vendored file)
DAY2_CONTROLLER_LIMITS = (
    ('corridor_lost_s', 'max', 2.), ('drive_grace_s', 'max', 1.), ('drive_no_response_s', 'max', 1.),
    ('drive_min_speed_m_s', 'min', .03), ('still_confirm_s', 'min', .4), ('max_stops_per_window', 'max', 3),
    ('reacquire_updates', 'min', 2),
)
DAY2_LIDAR_YAW_OFFSET_DEG = (179.08, 10.)           # LIDAR_RESPONSE.json estimate +- 10 deg (S0/V6 check the sign)
DAY2_THRESHOLD_MIN_PCT = 20                         # breakaway / rolling below 20 % is a typo (the C2 ladder starts at 20)
# N2: calibration runs (no C3 needed) are bounded directly: base, cap and the overspeed reference
CALIB_MAX_BASE_PCT = 60
CALIB_MAX_DUTY_CAP_PCT = 75
CALIB_MAX_SPEED_M_S = .5


def _limit_problem(prefix, name, value, direction, bound):
    if value is None:
        return None
    if (direction == 'max' and value > bound + 1e-9) or (direction == 'min' and value < bound - 1e-9):
        return '%s: %s %r %s the evaluated field limit %g' % (prefix, name, value,
                                                              'above' if direction == 'max' else 'below', bound)
    return None


def day2_motor_problems(cfg):
    """review_safety_day2 N1: motor output (row or calibration) refuses any loosened safety-relevant setting."""
    prefix = 'motor_output_enabled'
    problems = []
    for name, direction, bound in DAY2_MOTOR_LIMITS:
        if name.startswith('near_confirm_') and name != 'near_confirm_scans' and cfg.near_confirm_scans == 0:
            continue                                     # filter off: every near return reaches the estimator
        problem = _limit_problem(prefix, name, getattr(cfg, name), direction, bound)
        if problem:
            problems.append(problem)
    for name, direction, bound in DAY2_CONTROLLER_LIMITS:
        problem = _limit_problem(prefix, 'controller_overrides.' + name, cfg.controller_overrides.get(name),
                                 direction, bound)
        if problem:
            problems.append(problem)
    nominal, tol = DAY2_LIDAR_YAW_OFFSET_DEG
    off = (cfg.lidar_yaw_offset_deg - nominal + 180.) % 360. - 180.
    if abs(off) > tol + 1e-9:
        problems.append('%s: lidar_yaw_offset_deg %r is %.1f deg from the measured mount %.2f (> %g; reversed or '
                        'wrong mount)' % (prefix, cfg.lidar_yaw_offset_deg, off, nominal, tol))
    if cfg.imu_yaw_sign != 1:
        problems.append('%s: imu_yaw_sign must be +1 (gravity-projected yaw, replay correlation 0.74-0.95)' % prefix)
    if not abs(cfg.lidar_x_in_base_m) < .5 * cfg.footprint_length_m:
        problems.append('%s: lidar_x_in_base_m %r puts the LiDAR outside the footprint (length %g m)'
                        % (prefix, cfg.lidar_x_in_base_m, cfg.footprint_length_m))
    for name in ('breakaway_duty_pct', 'rolling_breakaway_duty_pct'):
        value = getattr(cfg, name)
        if value is not None and value < DAY2_THRESHOLD_MIN_PCT:
            problems.append('%s: %s %d below %d %% is not a plausible measurement (typo?)'
                            % (prefix, name, value, DAY2_THRESHOLD_MIN_PCT))
    return problems


def calib_day2_problems(cfg):
    """review_safety_day2 N2: a calibration config needs no measured speed, so its duty and speed are bounded here."""
    problems = []
    for name, value, bound in (('base_duty_pct', cfg.base_duty_pct, CALIB_MAX_BASE_PCT),
                               ('duty_cap_pct', cfg.duty_cap_pct, CALIB_MAX_DUTY_CAP_PCT),
                               ('max_speed_m_s', cfg.max_speed_m_s, CALIB_MAX_SPEED_M_S)):
        if value > bound + 1e-9:
            problems.append('calib_motor_output_enabled: %s %r above %g (calibration runs need no C3 speed; '
                            'they are bounded directly)' % (name, value, bound))
    return problems


def onsite_yaw_authority(file_cfg, cfg):
    """review_correctness_day2 R2: the measured yaw authority belongs to the FILE's differential. An on-site
    differential d scales it linearly (measured x d / file d, the arc gain model of calib_runs.arc_outcome) and
    max_yaw_rate_deg_s is lowered (0.1 deg/s steps, never above the file value) to yaw_authority_fraction x the scaled
    authority. The usual rules then apply (authority >= min_yaw_authority_deg_s: a too small differential is refused).
    Back at the file differential the file values return."""
    measured, file_diff = file_cfg.measured_yaw_at_max_diff_deg_s, file_cfg.max_differential_pct
    if measured is None:
        return cfg
    if cfg.max_differential_pct == file_diff:
        return replace(cfg, measured_yaw_at_max_diff_deg_s=measured, max_yaw_rate_deg_s=file_cfg.max_yaw_rate_deg_s)
    if file_diff <= 0:
        return replace(cfg, measured_yaw_at_max_diff_deg_s=None)       # nothing measured to scale from
    scaled = round(float(measured) * cfg.max_differential_pct / file_diff, 3)
    limit = math.floor(cfg.yaw_authority_fraction * scaled * 10. + 1e-9) / 10.
    max_yaw = min(file_cfg.max_yaw_rate_deg_s, limit) if limit >= .1 else file_cfg.max_yaw_rate_deg_s
    return replace(cfg, measured_yaw_at_max_diff_deg_s=scaled, max_yaw_rate_deg_s=max_yaw)


# --- field decision rules (unified calibration 2026-10-22; separate block for clean merges) ---------------------------
# The ONE place of the rule parameters: row MOTOR feasibility here (motor_steering_problems), tools/field_calib/recommend.py
# (recording stage and, with the executor's calibration runs, the final config) and make_field_config.py import them.
# Sources: decision table DAY2_STATUS.txt 7 / WEEKEND_PLAN 3.3; steering GO rule (field autonomy work folder of
# 2026-10-22, steer_rule/RULE_KO.txt; continuous_side, default 45 scenes, development seeds 0-7: every tested setting with gain >=
# 0.10 and authority >= 4.0 deg/s at max_differential 10/16/20 passed, every one below 4.0 at max_differential < 20
# failed; max_yaw = 0.6 x authority in all of them). SIM plant != vehicle: a SIM-derived threshold, not a proof.
STEER_GAIN_MIN_DPS_PER_PCT = .10      # G3: weaker-side C5 least-squares gain, deg/s per % of (R - L)
STEER_AUTHORITY_MIN_DPS = 4.0         # G3b: yaw at R - L = 2 x max_differential (row MOTOR minimum authority)
STEER_MIN_DIFFERENTIAL_PCT = 10       # G3b: smallest max_differential with SIM evidence at >= 4.0 deg/s
STEER_MAX_DIFFERENTIAL_PCT = 20       # decision table cap (WEEKEND_PLAN 3.3: outputs base +- 20); largest SIM evidence
STEER_AUTHORITY_FRACTION = .6         # the SIM evidence used max_yaw = 0.6 x authority (= yaw_authority_fraction)
STEER_MARGINAL_FACTOR = 1.15          # a GO below 1.15 x 4.0 = 4.6 deg/s is reported as 'steering marginal'
STEER_MAX_YAW_DPS = 4.5               # WEEKEND_PLAN 3.3 ceiling of max_yaw_rate_deg_s
STEER_MIN_SEGMENTS_PER_SIDE = 3       # C5: left (R > L) and right each >= 3 segments; one side never stands in
STEER_DIRECT_WINDOW_PCT = 5           # direct C5 segments: 2 d - 5 <= |R - L| <= 2 d (never above 2 d)
STEER_RULE_MODES = CONTROLLER_MODES   # the continuous row-following modes (every allowed mode); a pivot mode needs its own
DECISION_BASE_DUTY_PCT = 55           # base of the decision table (= the default base_duty_pct; others: user decision O5)


def decision_max_differential(base_pct, inner_margin_pct, breakaway_pct, rolling_pct=None):
    """Decision table: (max_differential, wheel threshold used, source).

    d = min(20, base - (threshold + inner_margin), 100 - base), at least 0. threshold = the C3b rolling threshold R
    when it is measured (feasibility() still needs base >= B + inner_margin for the start from standstill), else the
    C2 breakaway B. Without B there is no inner-wheel limit: d = min(20, 100 - base) (the config then refuses motor
    output for the missing B)."""
    base = int(base_pct)
    if breakaway_pct is None:
        return max(0, min(STEER_MAX_DIFFERENTIAL_PCT, 100 - base)), None, 'breakaway unknown (C2): no inner-wheel limit'
    if rolling_pct is not None:
        threshold, source = int(rolling_pct), 'rolling_breakaway_duty_pct (C3b)'
    else:
        threshold, source = int(breakaway_pct), 'breakaway_duty_pct (C2)'
    d = min(STEER_MAX_DIFFERENTIAL_PCT, base - (threshold + int(inner_margin_pct)), 100 - base)
    return max(0, int(d)), threshold, source


def _median(values):
    v = sorted(values)
    n = len(v)
    return float(v[n // 2]) if n % 2 else .5 * (v[n // 2 - 1] + v[n // 2])


def steering_authority(segments, max_differential_pct, side_gain, side_n):
    """Yaw authority at R - L = 2 d for the steering GO rule (G3b): dict(yaw, method, direct, gain_weak).

    segments  C5 arc rows: side ('left' = R > L = +yaw / 'right'), diff = R - L [%], yaw_net_dps (deg/s, straight
              bias removed) - tools/field_calib steps.c5_arc rows and/or calib_analysis.c5_segments rows.
    side_gain / side_n  per side: least-squares gain and segment count (the caller's pool).
    gain_weak = the smaller side gain, only when BOTH sides have >= 3 segments (else yaw None: G3/G3b unknown).
    yaw: direct segments with 2d - 5 <= |R - L| <= 2d, signed in the commanded direction (a segment that turned the
    wrong way is negative authority), median per side when a side has >= 3, the smaller side when both have; else
    gain_weak x 2d, lowered by a single side's direct median. Never above 2d: yaw grows with |R - L|, so a wider window
    over-states the authority (review_steer_gate: a false GO at the edge). Clipped at 0."""
    enough = all(side_n.get(k, 0) >= STEER_MIN_SEGMENTS_PER_SIDE and side_gain.get(k) is not None
                 for k in ('left', 'right'))
    gain_weak = min(side_gain['left'], side_gain['right']) if enough else None
    out = dict(yaw=None, method=None, direct={}, gain_weak=gain_weak)
    d = int(max_differential_pct or 0)
    if gain_weak is None or d <= 0:
        return out
    want = 2 * d
    for side in ('left', 'right'):
        ys = [x['yaw_net_dps'] * (1. if x['diff'] > 0 else -1.) for x in segments
              if x.get('side') == side and x.get('yaw_net_dps') is not None
              and want - STEER_DIRECT_WINDOW_PCT <= abs(x['diff']) <= want]
        if len(ys) >= STEER_MIN_SEGMENTS_PER_SIDE:
            out['direct'][side] = _median(ys)
    if len(out['direct']) == 2:
        out['yaw'] = max(0., min(out['direct'].values()))
        out['method'] = 'direct C5 segments with %d <= |R-L| <= %d, >= %d per side, signed (smaller side)' % (
            want - STEER_DIRECT_WINDOW_PCT, want, STEER_MIN_SEGMENTS_PER_SIDE)
        return out
    est = gain_weak * want
    max_abs = max((abs(x['diff']) for x in segments if x.get('yaw_net_dps') is not None), default=0)
    method = 'smaller side gain %.3f x %d (measured up to |R-L| %s)' % (gain_weak, want, max_abs)
    for side, y in out['direct'].items():
        if y < est:
            est = y
            method = 'direct C5 segments of the %s side only (%d <= |R-L| <= %d, below the gain-based %.2f)' % (
                side, want - STEER_DIRECT_WINDOW_PCT, want, gain_weak * want)
    out['yaw'], out['method'] = max(0., est), method
    return out


def steering_go_problems(gain_weak, max_differential_pct, yaw):
    """Steering GO rule G3 + G3b on the measured values: [] = GO, else every reason (None = not measured)."""
    p = []
    if gain_weak is None:
        p.append('weaker-side C5 gain not measured (both sides >= %d segments)' % STEER_MIN_SEGMENTS_PER_SIDE)
    elif gain_weak < STEER_GAIN_MIN_DPS_PER_PCT - 1e-9:
        p.append('weaker-side C5 gain %.3f < %.2f deg/s per %%' % (gain_weak, STEER_GAIN_MIN_DPS_PER_PCT))
    if max_differential_pct is None or max_differential_pct < STEER_MIN_DIFFERENTIAL_PCT:
        p.append('max_differential %s < %d (inner-wheel rule; no SIM evidence below)' % (
            max_differential_pct, STEER_MIN_DIFFERENTIAL_PCT))
    if yaw is None:
        p.append('yaw authority at R-L = 2 x max_differential not measured')
    elif yaw < STEER_AUTHORITY_MIN_DPS - 1e-9:
        p.append('yaw authority %.2f < %.1f deg/s' % (yaw, STEER_AUTHORITY_MIN_DPS))
    return p


def max_yaw_for_authority(yaw_dps, fraction=STEER_AUTHORITY_FRACTION):
    """max_yaw_rate_deg_s for a measured authority: min(4.5, fraction x yaw) rounded down to 0.1, at least 0.1."""
    return round(max(.1, min(STEER_MAX_YAW_DPS, math.floor(10. * fraction * float(yaw_dps) + 1e-6) / 10.)), 1)


def guard_caps(cfg, base_pct, max_differential_pct):
    """(duty_cap_pct, diff_cap_pct) of a field config: the tight OutputGuard caps base + d and 2 d. Raised only when the
    opt-in turn is enabled in cfg (turn_feasibility needs turn drive duty + turn_drive_diff_cap_pct <= duty_cap)."""
    duty_cap, diff_cap = int(base_pct) + int(max_differential_pct), 2 * int(max_differential_pct)
    if getattr(cfg, 'turn_assist_enabled', False):
        drive = cfg.turn_drive_duty_pct if cfg.turn_drive_duty_pct is not None else int(base_pct)
        duty_cap = max(duty_cap, int(drive) + int(cfg.turn_drive_diff_cap_pct))
    return min(100, duty_cap), min(100, diff_cap)


def motor_min_yaw_authority(cfg):
    """Minimum measured yaw authority of row MOTOR output: max(min_yaw_authority_deg_s, 4.0) in the continuous modes."""
    floor = STEER_AUTHORITY_MIN_DPS if cfg.mode in STEER_RULE_MODES else 0.
    return max(float(cfg.min_yaw_authority_deg_s), floor)


def motor_steering_problems(cfg):
    """Row MOTOR output (field profile): max_differential_pct in [10, 20] and the measured authority >= 4.0 deg/s
    (steering GO rule). Called from feasibility() only with motor_output_enabled; SHADOW and calibration configs are
    not affected (the calibration runs measure C5). Vehicle profile front_steer: [] (front_steer_go_problems is its GO
    rule; there is no differential)."""
    if cfg.mode not in STEER_RULE_MODES or cfg.vehicle_profile == FRONT_STEER:
        return []
    problems = []
    d = cfg.max_differential_pct
    if not STEER_MIN_DIFFERENTIAL_PCT <= d <= STEER_MAX_DIFFERENTIAL_PCT:
        problems.append('motor_output_enabled: max_differential_pct %d outside [%d, %d] (steering GO rule: SIM evidence '
                        'only for differentials %d-%d)' % (d, STEER_MIN_DIFFERENTIAL_PCT, STEER_MAX_DIFFERENTIAL_PCT,
                                                           STEER_MIN_DIFFERENTIAL_PCT, STEER_MAX_DIFFERENTIAL_PCT))
    yaw, need = cfg.measured_yaw_at_max_diff_deg_s, motor_min_yaw_authority(cfg)
    if yaw is not None and need > cfg.min_yaw_authority_deg_s + 1e-9 and yaw < need - 1e-9:
        problems.append('motor_output_enabled: yaw authority %.2f deg/s below %.2f (steering GO rule G3b: row motor '
                        'output needs >= %.1f deg/s at R-L = 2 x max_differential, min_yaw_authority_deg_s %.2f is raised)'
                        % (yaw, need, STEER_AUTHORITY_MIN_DPS, cfg.min_yaw_authority_deg_s))
    return problems


# --- single-row mode (mode 'single_row', 2026-11-02; separate block for clean merges) -----------------------------
# The executor-facing settings of the vendored closed_loop/single_row.SingleRowController (module docstring, 'Single-row
# mode'). Rules here: the static shape of the block (single_row_rule_problems, every profile), the inner-wheel rules at the
# lowest base of the speed loop (single_row_inner_problems, from feasibility() whenever the breakaway is known) and the
# motor-output additions (single_row_motor_problems). Nothing here relaxes a rule of the other modes.
SINGLE_ROW_LOCALISATION = 'SENSOR_ONLY_SINGLE_ROW'
SINGLE_ROW_YAW_AUTHORITY_FRACTION = .6     # SingleRowConfig: max_yaw_rate <= 0.6 x nominal 2 x arc gain x max differential


def single_row_controller_fields(cfg):
    """SingleRowConfig keyword arguments of the single_row_* keys (+ the footprint shared with the estimator)."""
    low = float(cfg.single_row_min_speed_m_s)
    return dict(cruise_speed_m_s=float(cfg.single_row_target_speed_m_s), min_speed_m_s=low,
                authority_slow_speed_m_s=low, approach_speed_m_s=float(cfg.single_row_approach_speed_m_s),
                base_min_pct=int(cfg.single_row_base_min_pct), base_max_pct=int(cfg.single_row_base_max_pct),
                bias_max_pct=float(cfg.single_row_bias_max_pct), start_max_deg=float(cfg.single_row_start_max_deg),
                end_stop_offset_m=float(cfg.single_row_end_stop_offset_m),
                footprint_length_m=float(cfg.footprint_length_m), footprint_width_m=float(cfg.footprint_width_m))


def single_row_estimator_fields(cfg):
    """EstimatorConfig keyword arguments of the single-row estimator options (None = the original rule)."""
    behind, span = cfg.single_row_acquire_behind_m, cfg.single_row_end_segment_span_m
    return dict(acquire_behind_m=None if behind is None else float(behind),
                acquire_axis_window_deg=float(cfg.single_row_acquire_window_deg),
                row_end_segment_span_m=None if span is None else float(span),
                band_rotate_about_vehicle=bool(cfg.single_row_band_about_vehicle),
                row_end_stagger_gap_m=None if cfg.single_row_end_stagger_gap_m is None
                else float(cfg.single_row_end_stagger_gap_m),
                row_end_last_trunk=bool(cfg.single_row_end_last_trunk))


def single_row_time_cap_speed(cfg):
    """Reference speed of the single-row motion-time cap: the lowest target speed the controller ever drives toward
    (a vehicle slower than this over the whole cap distance is not following the row)."""
    return min(float(cfg.single_row_min_speed_m_s), float(cfg.single_row_approach_speed_m_s))


def single_row_rule_problems(cfg):
    """Static rules of mode 'single_row' (every profile); [] in the other modes."""
    if cfg.mode != SINGLE_ROW_MODE:
        return []
    p = []
    if cfg.require_native_navigation:
        p.append('single_row: localisation is the executor LiDAR + raw IMU estimator (%s); set '
                 'require_native_navigation false explicitly (a native provider is an optional cross-check)'
                 % SINGLE_ROW_LOCALISATION)
    if cfg.turn_assist_enabled:
        p.append('single_row: turn_assist_enabled is refused (no headland turn / next row in single-row mode)')
    if 'row_end_stop_m' in cfg.controller_overrides:
        p.append('single_row: controller_overrides.row_end_stop_m is not used (single_row_end_stop_offset_m is the '
                 'row-end stop setting)')
    base, low, high, diff = (cfg.base_duty_pct, cfg.single_row_base_min_pct, cfg.single_row_base_max_pct,
                             cfg.max_differential_pct)
    # vehicle_profile front_steer: these rules stay. The Runner builds the vendored SingleRowConfig in every profile (its
    # stop model and timing constants: still_confirm_s, drive_no_response_s, ...; SingleRowConfig.__post_init__ checks
    # the same base / differential / yaw relations), although a front_steer run never commands its differential.
    if not low <= base <= high:
        p.append('single_row: base_duty_pct %d (start base) outside [single_row_base_min_pct %d, '
                 'single_row_base_max_pct %d]' % (base, low, high))
    if high + diff > cfg.duty_cap_pct:
        p.append('single_row: duty_cap_pct %d < single_row_base_max_pct + max_differential = %d (guard would trip on '
                 'normal output)' % (cfg.duty_cap_pct, high + diff))
    if high + diff > 100:
        p.append('single_row: single_row_base_max_pct + max_differential = %d above 100 (no differential headroom)'
                 % (high + diff))
    if low - diff < 1:
        p.append('single_row: single_row_base_min_pct - max_differential must leave the inner wheel driving (>= 1)')
    target = cfg.single_row_target_speed_m_s
    if cfg.single_row_min_speed_m_s > target or cfg.single_row_approach_speed_m_s > target:
        p.append('single_row: min / approach speed must not exceed single_row_target_speed_m_s')
    if target > cfg.max_speed_m_s:
        p.append('single_row: single_row_target_speed_m_s %.2f above max_speed_m_s %.2f' % (target, cfg.max_speed_m_s))
    if cfg.single_row_bias_max_pct > diff:
        p.append('single_row: single_row_bias_max_pct %.1f above max_differential_pct %d'
                 % (cfg.single_row_bias_max_pct, diff))
    if cfg.guard_heading_deg is not None and cfg.single_row_start_max_deg > cfg.guard_heading_deg:
        p.append('single_row: single_row_start_max_deg %.1f above guard_heading_deg %.1f (a start the geometry guard '
                 'would end at once)' % (cfg.single_row_start_max_deg, cfg.guard_heading_deg))
    if cfg.single_row_acquire_window_deg < cfg.single_row_start_max_deg:
        p.append('single_row: single_row_acquire_window_deg %.1f below single_row_start_max_deg %.1f (the start check, '
                 'not the acquisition window, refuses a misaligned start)'
                 % (cfg.single_row_acquire_window_deg, cfg.single_row_start_max_deg))
    if cfg.single_row_end_stagger_gap_m is not None and cfg.single_row_end_segment_span_m is None:
        p.append('single_row: single_row_end_stagger_gap_m needs single_row_end_segment_span_m (the row-like test of '
                 'the support beyond the gap)')
    nominal = 2. * cfg.arc_yaw_deg_s_per_pct_diff * diff
    if not cfg.max_yaw_rate_deg_s <= SINGLE_ROW_YAW_AUTHORITY_FRACTION * nominal + 1e-9:
        p.append('single_row: max_yaw_rate_deg_s %.2f above %.1f x nominal authority 2 x arc gain x max_differential '
                 '= %.2f (SingleRowConfig rule)' % (cfg.max_yaw_rate_deg_s, SINGLE_ROW_YAW_AUTHORITY_FRACTION, nominal))
    cap = cfg.single_row_max_row_length_m
    if cap > cfg.max_row_len_m:
        p.append('single_row: single_row_max_row_length_m %.1f above max_row_len_m %.1f' % (cap, cfg.max_row_len_m))
    if cap <= cfg.min_run_distance_m:
        p.append('single_row: single_row_max_row_length_m must exceed min_run_distance_m')
    return p


def single_row_inner_problems(cfg):
    """Inner-wheel rules at the LOWEST base of the speed loop (breakaway known; feasibility() problems).

    Moving: single_row_base_min_pct - max_differential >= (rolling threshold if measured, else breakaway) + margin.
    Start from standstill (equal duty at the current base, which may be base_min after a corridor wait):
    single_row_base_min_pct >= breakaway + margin."""
    if cfg.mode != SINGLE_ROW_MODE or not cfg.inner_rule_enabled or cfg.breakaway_duty_pct is None:
        return []
    static, margin, diff = cfg.breakaway_duty_pct, cfg.inner_margin_pct, cfg.max_differential_pct
    low = cfg.single_row_base_min_pct
    rolling = cfg.rolling_breakaway_duty_pct
    moving, name = (rolling, 'rolling_breakaway') if rolling is not None else (static, 'breakaway')
    problems = []
    if low - diff < moving + margin:
        problems.append('single_row inner wheel single_row_base_min_pct-max_differential = %d < %s+margin = %d'
                        % (low - diff, name, moving + margin))
    if low < static + margin:
        problems.append('single_row_base_min_pct %d < breakaway+margin = %d (a start from standstill at the lowest '
                        'base)' % (low, static + margin))
    return problems


def single_row_motor_problems(cfg):
    """Row MOTOR output in mode 'single_row' (feasibility(), field profile): the front stops at or before the observed
    end of the row support (single_row_end_stop_offset_m >= footprint_length_m / 2)."""
    if cfg.mode != SINGLE_ROW_MODE:
        return []
    problems = []
    if cfg.single_row_end_stop_offset_m < .5 * cfg.footprint_length_m - 1e-9:
        problems.append('motor_output_enabled: single_row_end_stop_offset_m %.2f below footprint_length_m / 2 = %.2f '
                        '(the front would stop beyond the observed row end)'
                        % (cfg.single_row_end_stop_offset_m, .5 * cfg.footprint_length_m))
    return problems


# --- front-wheel steering (vehicle_profile 'front_steer'; steering branch P4, design 6.5-6.7; separate block) ----------
# The steered vehicle drives both sides with the same duty (host drive_lr_mode equal_duty) and steers the front wheels
# through the V3 bridge (throttle %, steer_mrad). Rules here: the static shape of the block (steer_rule_problems, every
# profile), the steering GO rule of design 6.6 (front_steer_go_problems, from feasibility(): unknown in SHADOW, problems
# with motor output) and the stop-reason table of the steering monitors (STEER_FAULT_REASONS, used by core.Runner and
# bridge_client.SteerBridgeClient). Nothing here changes a rule of the differential profile. Every threshold is a
# placeholder (design 6.6 table [proposal]; the steering pre-registration P8 fixes them).
DIFFERENTIAL = 'differential'
FRONT_STEER = 'front_steer'
VEHICLE_PROFILES = (DIFFERENTIAL, FRONT_STEER)
STEER_RATE_FRACTION = .8              # design 6.2: command slew limit <= 0.8 x the measured steering rate (B5)
STEER_GO_SOFT_MARGIN_MIN_DEG = 2.     # design 6.6: per-wheel soft limit <= mechanical limit - 2 deg
STEER_GO_ZERO_OFFSET_MAX_DEG = 1.5    # design 6.6: |zero offset F1| <= 1.5 deg
STEER_GO_DEADBAND_MAX_DEG = 1.        # design 6.6: servo deadband <= 1.0 deg
# Control socket folders of the two bridge roles (HOST_BRIDGE_V3_KO.md 2: the host wrapper binds its role to its folder).
# A front_steer configuration binds steer_allow_sim to the SIM folder (review RS-14): SIM allowed only with a socket under
# STEER_SIM_SOCKET_DIR, a vehicle configuration never on it.
STEER_SIM_SOCKET_DIR = '/run/ubts-steer/'
STEER_VEHICLE_SOCKET_DIR = '/run/ubts-rover/'
# Control bridges (steer_bridge_kind). The ESP32-S3 bridge's steering API: int steer_mrad (left +) on the firmware command
# scale, +-COMMAND_LIMIT_DEG (dashboard scripts/esp32_abs_hold.py COMMAND_LIMIT_DEG 20, LIMIT_MRAD 349).
STEER_BRIDGE_KINDS = ('v3', 'esp32_abs_hold')
ESP32_BRIDGE_KIND = 'esp32_abs_hold'
ESP32_COMMAND_LIMIT_DEG = 20.
ESP32_BRIDGE_LIMIT_MRAD = 349
# Design 6.6 stop reasons. The executor's own monitors (core.SteerMonitor) give steering_feedback_stale /
# steering_feedback_invalid / steering_uncalibrated / steering_authority_insufficient; every other reason comes from the
# detector that owns the condition, read from the steering status: MCU fault names (status fl, esp_motor_v3
# FAULT_BIT_NAMES) and host latches (steer_bridge_core LATCH_NAMES). First match in this order wins: the steering faults
# (STEER_FAULT_REASONS) before the link / session faults (STEER_LINK_FAULT_REASONS: executor-only reasons of category
# SYSTEM_FAULT, a proposal; a C TTL expiry ends the run because the next C would restart the drive without the operator).
# Status bits that are no fault (BOOT, SOFTLIMIT) and host latches without a table entry give no reason (bridge_error /
# bridge_latched then, with the fault names in the end note).
STEER_FAULT_REASONS = (
    ('POLARITY', 'steering_polarity_fault'),
    ('DRV_STUCK', 'steering_driver_fault'),
    ('THERMAL', 'steering_driver_fault'),
    ('STALL', 'steering_stalled'),
    ('TRACK', 'steering_not_following'),
    ('steering_not_following', 'steering_not_following'),     # host latch (requested vs measured, 3 deg 1.5 s)
    ('steering_ineffective', 'steering_ineffective'),         # host latch (gyro vs v tan(delta) / L, op motion)
    ('POT_OPEN', 'steering_feedback_invalid'),
    ('POT_SHORT', 'steering_feedback_invalid'),
    ('POT_JUMP', 'steering_feedback_invalid'),
    ('POT_NOISE', 'steering_feedback_invalid'),
    ('NOCAL', 'steering_uncalibrated'),
    ('calibration_mismatch', 'steering_uncalibrated'),
    ('steering_status_stale', 'steering_feedback_stale'),     # host latch (status gap 150 ms)
    ('pot_invalid', 'steering_feedback_invalid'),             # ESP32 bridge: STATUS INVALID / hold cancel INVALID POT
    ('status_seq_gap', 'steering_feedback_stale'),
    ('stop_unconfirmed', 'stop_unconfirmed'),
)
STEER_LINK_FAULT_REASONS = (
    ('mcu_reset_detected', 'mcu_reset'),
    ('WATCHDOG', 'link_watchdog'),
    ('CRC', 'link_bad_lines'),
    ('TTL', 'link_ttl_expired'),
    ('mcu_ttl_expired', 'link_ttl_expired'),                  # host latch (R18 decision 1: the host stops on the TTL)
    ('c_rejected', 'link_command_rejected'),
    ('hardware_fault', 'link_hardware_fault'),
    ('mcu_state_mismatch', 'mcu_state_mismatch'),
    ('arm_unconfirmed', 'arm_unconfirmed'),
    ('arm_rejected', 'arm_rejected'),
    ('sim_identity_mismatch', 'link_identity_mismatch'),
    ('hardware_profile_mismatch', 'link_identity_mismatch'),
    ('internal_error', 'bridge_internal_error'),
    ('mcu_disarmed', 'mcu_disarmed'),
    ('firmware_mismatch', 'link_identity_mismatch'),          # ESP32 bridge: another firmware on the board
    ('mcu_drive_watchdog', 'link_watchdog'),                  # ESP32 bridge: the (proposed) firmware drive watchdog fired
)
STEER_STOP_REASONS = ('steering_feedback_invalid', 'steering_feedback_stale', 'steering_not_following',
                      'steering_stalled', 'steering_ineffective', 'steering_uncalibrated', 'steering_polarity_fault',
                      'steering_driver_fault', 'steering_authority_insufficient', 'stop_unconfirmed')
STEER_LINK_STOP_REASONS = tuple(sorted(set(reason for _, reason in STEER_LINK_FAULT_REASONS)))


def is_front_steer(cfg):
    return getattr(cfg, 'vehicle_profile', DIFFERENTIAL) == FRONT_STEER


def effective_start_mode(cfg):
    """The start mode of a configuration (design v2 V2-2): the explicit start_mode, else 'one_press' for front_steer and
    'hold_to_run' for the differential profile."""
    mode = getattr(cfg, 'start_mode', None)
    if mode is not None:
        return mode
    return ONE_PRESS if is_front_steer(cfg) else HOLD_TO_RUN


def autonomy_rule_problems(cfg):
    """Static rules of the autonomy block: a known start mode. The operator timeout range is the field spec (0.5-3.0 s,
    locked: config file only)."""
    if cfg.start_mode is not None and cfg.start_mode not in START_MODES:
        return ['start_mode: must be null or one of %s' % (START_MODES,)]
    return []


def steer_fault_reason(names):
    """The stop reason of MCU fault names / host latch names: the first STEER_FAULT_REASONS match (design 6.6), else the
    first STEER_LINK_FAULT_REASONS match, else None."""
    present = set(names or ())
    for table in (STEER_FAULT_REASONS, STEER_LINK_FAULT_REASONS):
        for name, reason in table:
            if name in present:
                return reason
    return None


def steer_rule_problems(cfg):
    """Static rules of the steering block (every profile); [] for the differential profile."""
    if cfg.vehicle_profile not in VEHICLE_PROFILES:
        return ['vehicle_profile: must be one of %s' % (VEHICLE_PROFILES,)]
    p = []
    if cfg.vehicle_profile != FRONT_STEER:
        return p
    if cfg.profile == 'bench':
        p.append('front_steer: the bench profile is refused (bench patterns are left / right duties; the steering bench '
                 'is the MCU BENCH mode, P9)')
    if cfg.turn_assist_enabled:
        p.append('front_steer: turn_assist_enabled is refused (no pivot, no headland turn)')
    if cfg.calib_motor_output_enabled:
        p.append('front_steer: calib_motor_output_enabled is refused (the calibration runs C5 / C3b / C7 are left / right '
                 'duty tables; the steering identification is sim/ubts/steering/ident with the drive parts in '
                 'tools/field_calib, IDENT_KO.md)')
    socket = posixpath.normpath(cfg.socket_path) if cfg.socket_path.startswith('/') else cfg.socket_path
    in_sim = (socket + '/').startswith(STEER_SIM_SOCKET_DIR)
    if cfg.steer_allow_sim and not in_sim:
        p.append('front_steer: steer_allow_sim True needs the SIM control socket under %s (socket_path %s%s; review '
                 'RS-14)' % (STEER_SIM_SOCKET_DIR, cfg.socket_path,
                             ': the vehicle socket' if (socket + '/').startswith(STEER_VEHICLE_SOCKET_DIR) else ''))
    if not cfg.steer_allow_sim and in_sim:
        p.append('front_steer: socket_path %s is a SIM control socket; a vehicle configuration (steer_allow_sim False) '
                 'never uses it (review RS-14)' % cfg.socket_path)
    if cfg.steer_rate_limit_standstill_deg_s > cfg.steer_rate_limit_deg_s + 1e-9:
        p.append('front_steer: steer_rate_limit_standstill_deg_s %.2f above steer_rate_limit_deg_s %.2f (the actuator is '
                 'not faster at standstill)' % (cfg.steer_rate_limit_standstill_deg_s, cfg.steer_rate_limit_deg_s))
    reply = 1. / cfg.send_hz + 1. / cfg.control_hz
    if cfg.steer_status_max_age_s < reply - 1e-9:
        p.append('front_steer: steer_status_max_age_s %.3f below one drive reply period + one tick = %.3f s (the status '
                 'would never be fresh)' % (cfg.steer_status_max_age_s, reply))
    if not abs(cfg.steer_rotation_centre_x_m) < .5 * cfg.footprint_length_m:
        p.append('front_steer: steer_rotation_centre_x_m %r outside the footprint (length %g m)'
                 % (cfg.steer_rotation_centre_x_m, cfg.footprint_length_m))
    p += esp32_rule_problems(cfg)
    return p


def esp32_scale(cfg):
    """Firmware command degrees per equivalent wheel degree of the ESP32 bridge (ESP32_COMMAND_LIMIT_DEG /
    steer_esp32_mark_deg), or None (not the ESP32 bridge, or no mark angle)."""
    if getattr(cfg, 'steer_bridge_kind', 'v3') != ESP32_BRIDGE_KIND or cfg.steer_esp32_mark_deg is None:
        return None
    return ESP32_COMMAND_LIMIT_DEG / float(cfg.steer_esp32_mark_deg)


def esp32_rule_problems(cfg):
    """Static rules of the bridge choice (front_steer): a known kind; the ESP32 bridge needs the mark angle, the
    wheelbase, no SIM identity and soft limits that stay inside the board's command range."""
    p = []
    if cfg.steer_bridge_kind not in STEER_BRIDGE_KINDS:
        return ['steer_bridge_kind: must be one of %s' % (STEER_BRIDGE_KINDS,)]
    if cfg.steer_bridge_kind != ESP32_BRIDGE_KIND:
        if cfg.steer_esp32_mark_deg is not None or cfg.steer_esp32_cal is not None:
            p.append('steer_esp32_*: only with steer_bridge_kind %s' % ESP32_BRIDGE_KIND)
        return p
    if cfg.steer_allow_sim:
        p.append('steer_bridge_kind %s: steer_allow_sim must be false (the ESP32 bridge has no SIM identity)'
                 % ESP32_BRIDGE_KIND)
    if cfg.steer_esp32_mark_deg is None:
        p.append('steer_bridge_kind %s needs steer_esp32_mark_deg (equivalent wheel angle of the command +-20)'
                 % ESP32_BRIDGE_KIND)
    if cfg.steer_wheelbase_m is None:
        p.append('steer_bridge_kind %s needs steer_wheelbase_m (controller and steering_ineffective rule)'
                 % ESP32_BRIDGE_KIND)
    scale = esp32_scale(cfg)
    if scale is not None:
        for name, limit in (('steer_limit_left_mrad', cfg.steer_limit_left_mrad),
                            ('steer_limit_right_mrad', cfg.steer_limit_right_mrad)):
            if limit * scale > ESP32_BRIDGE_LIMIT_MRAD + 1e-9:
                p.append('%s %d mrad is %.0f command mrad, beyond the ESP32 bridge limit %d (command +-%g deg)'
                         % (name, limit, limit * scale, ESP32_BRIDGE_LIMIT_MRAD, ESP32_COMMAND_LIMIT_DEG))
    return p


def prior_active(cfg):
    """Whether the actuator prior layer runs (design v2 V2-1, PW-10): prior_enabled true / false, or with None (the
    default) the profile default: on for front_steer (weight 0 = record and online-scale monitor only, the same
    decisions), off for the differential profile (it has no steering status)."""
    enabled = getattr(cfg, 'prior_enabled', None)
    if enabled is None:
        return is_front_steer(cfg)
    return bool(enabled)


def prior_rule_problems(cfg):
    """Static rules of the design v2 block: the priors need the steering status (front_steer) and a weight needs
    an active prior."""
    p = []
    if cfg.prior_enabled is True and cfg.vehicle_profile != FRONT_STEER:
        p.append('prior_enabled: needs vehicle_profile front_steer (the applied duty and the steering angle come from '
                 'the steering status)')
    if cfg.prior_weight > 0 and not prior_active(cfg):
        p.append('prior_weight %r: needs prior_enabled' % (cfg.prior_weight,))
    return p


def front_steer_go_problems(cfg):
    """Steering GO rule of design 6.6 (vehicle_profile front_steer): (problems, unknown). An unknown is a missing
    measurement (SHADOW: status 'unknown'; with motor output feasibility() turns every unknown into a problem). Kept from
    the differential table: the drive-side C2 / C3 / C7 measurements (feasibility()), imu_yaw_sign +1 and the LiDAR
    mount within 10 deg (day2_motor_problems). Firmware V3, cal_id match, the polarity flag of the MCU record and a
    fresh steering status are checked at the arm (bridge_client.SteerBridgeClient.arm_precheck)."""
    problems, unknown = [], []
    if not cfg.steer_range_measured:
        unknown.append('steer_range_measured false (B2 / B3: measured steering range in the calibration record)')
    rates = (('steer_rate_limit_deg_s', cfg.steer_rate_limit_deg_s, cfg.steer_measured_rate_deg_s, 'B5'),
             ('steer_rate_limit_standstill_deg_s', cfg.steer_rate_limit_standstill_deg_s,
              cfg.steer_measured_rate_standstill_deg_s, 'B5 / F5 at standstill'))
    for name, limit, measured, label in rates:
        if measured is None:
            unknown.append('%s: measured steering rate unknown (%s)' % (name, label))
        elif limit > STEER_RATE_FRACTION * measured + 1e-9:
            problems.append('%s %.2f deg/s above %.1f x the measured steering rate %.2f deg/s (design 6.2)'
                            % (name, limit, STEER_RATE_FRACTION, measured))
    measured = cfg.steer_measured_rate_deg_s
    if measured is not None and measured < cfg.steer_rate_min_deg_s - 1e-9:
        problems.append('measured steering rate %.2f deg/s below %.1f deg/s (steering GO rule for the lookahead; '
                        'placeholder, P8 fixes it)' % (measured, cfg.steer_rate_min_deg_s))
    margin = cfg.steer_wheel_soft_margin_deg
    if margin is None:
        unknown.append('steer_wheel_soft_margin_deg unknown (per-wheel soft limit against the mechanical limit, B2 / B3)')
    elif margin < STEER_GO_SOFT_MARGIN_MIN_DEG - 1e-9:
        problems.append('per-wheel soft limit only %.2f deg inside the mechanical limit (steering GO rule: >= %g deg)'
                        % (margin, STEER_GO_SOFT_MARGIN_MIN_DEG))
    if cfg.steer_min_pwm_pct is None:
        unknown.append('steer_min_pwm_pct unknown (B4: steering motor start PWM)')
    zero = cfg.steer_zero_offset_deg
    if zero is None:
        unknown.append('steer_zero_offset_deg unknown (F1)')
    elif abs(zero) > STEER_GO_ZERO_OFFSET_MAX_DEG + 1e-9:
        problems.append('zero offset %.2f deg beyond +-%g deg (steering GO rule, F1)'
                        % (zero, STEER_GO_ZERO_OFFSET_MAX_DEG))
    deadband = cfg.steer_servo_deadband_deg
    if deadband is None:
        unknown.append('steer_servo_deadband_deg unknown (MCU record)')
    elif deadband > STEER_GO_DEADBAND_MAX_DEG + 1e-9:
        problems.append('servo deadband %.2f deg above %g deg (steering GO rule)' % (deadband, STEER_GO_DEADBAND_MAX_DEG))
    if not cfg.steer_polarity_checked:
        unknown.append('steer_polarity_checked false (B0 polarity check)')
    if getattr(cfg, 'steer_bridge_kind', 'v3') == ESP32_BRIDGE_KIND:
        if cfg.steer_esp32_cal is None:
            unknown.append('steer_esp32_cal unknown (the firmware CAL line the ESP32 bridge must report)')
    elif cfg.steer_cal_id is None:
        unknown.append('steer_cal_id unknown (the calibration set id the bridge must report)')
    return problems, unknown
