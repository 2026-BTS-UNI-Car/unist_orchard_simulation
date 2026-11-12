"""Executor core shared by the live node and the offline replay (no ROS, no motor I/O).

Pipeline per tick (single thread, deterministic for a given input order):
  feed_imu / feed_scan (receipt monotonic time t)  ->  vendored SensorEstimator
  tick(now, hold_active)  ->  snapshot(now)  ->  Backstop  ->  hold / freshness rule
                          ->  RowController.step  ->  OutputGuard  ->  Decision(left, right, ...)

Input conversion (same as the validated replay, resume-audit-20261013/sim-review/sim2real/replay.py):
  * scan: angle_min += lidar_yaw_offset_deg (travel axis is -179.08 deg in the scan frame; the
    array is not reordered), wrapped to (-pi, pi]; 0 / NaN / < range_min / >= range_max -> inf;
    near returns inside the per-beam body mask -> inf; the scan is trimmed by a few tail beams
    when n * inc exceeds 2 pi + 1.01 inc (scan_corridor would refuse the whole scan otherwise).
  * IMU: yaw rate = sign * gyro . up, up = normalised EMA (tau 1 s) of the specific force; the
    raw acceleration is passed unchanged (vibration RMS).
  * clocks: every stamp is the receipt time.monotonic(); scan stamp = receipt - scan_latency_s,
    IMU stamp = receipt. Header stamps are diagnostics only (a GNSS time step of the system
    clock can not move the estimator clock).

Run life cycle (one run = one row pass from an operator Start):
  start_run(t, row_len_m)  new estimator, Backstop armed, state PAUSED ('await_hold')
  PAUSED  -> ACTIVE  only on a NEW hold press (rising edge) at a moment when both inputs have
                     been fresh for >= fresh_recover_s. A new RowController is built at every
                     resume (so a pause can never be read as 'drive_no_response').
  ACTIVE  -> PAUSED  hold released, stale input, loop stall (stall_guard) or an external pause();
                     corridor lost (see below).
  any     -> ENDED   terminal controller HOLD (row_end_candidate, front_blocked, side_blocked,
                     corridor_*, drive_no_response, ...), guard violation, backstop, too many
                     stale events / stall-guard pauses, clock regression, operator end_run().
                     An ENDED run never restarts; the operator must Start a new run.

Terminal decisions are never lost (review S1 / F1): the vendored controller decides a terminal stop
(row_end_candidate, front_blocked, side_blocked, drive_no_response, stop_repeat_limit, ...) by
entering STOP_REQUESTED with a pending ('hold' | 'front', reason) and only reaches HOLD after it saw
the vehicle stop (0.6-3 s later). A pause request in that window (hold released, stale input, loop
stall, watchdog) ENDS the run with the pending reason (end_note 'terminal_stop_pending_at_<why>')
instead of pausing, so the next press can not build a fresh controller that drives on. Any other
pending kind except 'wait_corridor' (and STOP_REQUESTED without a pending decision) is treated as
terminal too (fail closed).

Corridor lost (review S2 / F7): the vendored controller would stop, wait in REACQUIRE and drive on by
itself once the corridor is back while the button is still held. The Runner instead PAUSES (reason
'corridor_lost', controller discarded) the moment the controller starts that stop (STOP_REQUESTED with
pending 'wait_corridor'; any REACQUIRE other than the initial 'start' is handled the same way). A
resume then needs a NEW press made while the inputs are fresh AND the estimator reports standstill
(still for still_confirm_s); a press while the vehicle still coasts is ignored
('press_ignored_not_standstill'). The controller's limit is kept: a corridor loss when
max_stops_per_window (3) losses already happened within turn_window_m (5 m) of counted distance ENDS
the run ('corridor_lost_repeat_limit'; the controller's own count would restart with every new
controller).

Bench interlock (review S4): a bench run (wheels off the ground, profile 'bench' only) ENDS at once
('bench_motion_detected') when the estimator reports motion (|speed| > 0.1 m/s, or more than 0.2 m of
distance counted while it reports motion) and ('bench_front_blocked') when the front clearance is
below the stop model (vendored required_front(speed)), in PAUSED as well as ACTIVE.

Rolling inner-wheel threshold (review F2, only with rolling_breakaway_duty_pct measured): while the
vehicle has not been seen moving since the resume (estimator not still, odometry valid, speed >= 0.10
m/s; dropped again at still / invalid odometry / speed < 0.05 m/s) the inner wheel must reach the
static breakaway + margin: the Runner lowers the differential symmetrically (inner raised, outer
lowered, mean kept) before the OutputGuard checks it with the static minimum; once moving the
controller's output passes unchanged and the guard checks the rolling minimum.

Calibration runs (calib_runs.py, field profile, MOTOR only): start_calibration(t, plan) begins a run of kind
'arc' (C5), 'rolldown' (C3b) or 'stop' (C7): a bounded duty table instead of the row controller. Same Start -> press ->
fresh-input rule; the first press starts the pattern and ANY pause after it (hold released, stale input, loop
stall) ENDS the run ('calib_<why>'), so a calibration never restarts from standstill in the middle of its table.
Every output passes the run's own OutputGuard (caps, forward only, explicit minimum duty = measured breakaway or
40 instead of the inner-wheel rule); the run ends on its terminal checks (front stop model, side rule, overspeed,
yaw rate, heading change, no response, odometry, distance/time caps; calib_runs docstring). After a calibration run
ended, every tick hands the snapshot to CalibRun.observe_stop: the stop that follows the run's own 0/0 (C7).

Day-2 review fixes (review_safety_day2):
  N9  repeated short presses can not push a stuck vehicle without end: commanded non-zero output while the sensors see
      no motion (the vendored drive_no_response test: still for still_confirm_s, or odometry valid and speed below
      drive_min_speed_m_s) accumulates over the whole row run, across pauses and new controllers, and is reset only by
      observed motion; above NO_RESPONSE_CUMULATIVE_FACTOR x (mcu_ramp + drive_grace + drive_no_response) = 6 s the run
      ENDS ('drive_no_response_repeat'). (Each new controller restarts its own 3 s test at every resume.)
  N6  bench: while ACTIVE, odometry invalid (LiDAR scan matching failed) for more than BENCH_ODOM_LOST_S ends the run
      ('bench_odometry_lost'): the motion interlock needs scan matching (a featureless place hid 6.7 m of motion).

Stale-input rule (single rule, FEASIBILITY_CRITIQUE H4e): LiDAR older than lidar_max_age_s or
IMU older than imu_max_age_s while ACTIVE -> output 0/0 and PAUSED; motion restarts only after a
NEW hold press made while the inputs have been fresh for fresh_recover_s. Holding the button
through the recovery does not restart anything. The vendored controller's own stale recovery
(REACQUIRE after 1 s fresh + still) is never reached: the controller is not stepped with stale
input, and a recoverable HOLD it might still return is handled by this same rule. More than
max_stale_events stale pauses in one run end it ('stale_repeat_limit').

drive_no_response: the vendored controller declares it ~3 s after the first forward command if
the sensors see no motion (MCU ramp 1 s + grace 1 s + 1 s), then stops (terminal).

Executor-level rules added after the C1 gate (open-loop replay of the real 10/10 records; the
vendored estimator/controller are unchanged, gate/C1_GATE_REPORT.txt section 5):
  NearConfirmFilter (P1)  per run: a return nearer than near_confirm_range_m reaches the estimator
                          only if each of the previous near_confirm_scans processed scans had a return
                          within near_confirm_tol_m (+ tol_step per older scan) in the same rotated frame.
  geometry guard (P3)     while the controller drives with a valid band: |heading_error| >
                          guard_heading_deg -> END 'guard_heading'; |lateral_offset| > corridor_width/2 -
                          footprint_width/2 + guard_lateral_margin_m -> END 'guard_lateral'.
  ScanStamper (P4)        estimator scan stamps on the receipt clock from the header spacing (used by the
                          caller, e.g. executor_main; Runner.feed_scan(stamp=...)).

Single-row mode (config mode 'single_row', 2026-11-02): the controller is the vendored
closed_loop/single_row.SingleRowController (single_row_module(): imported next to the verified controllers module, its
file hash-checked against the vendor manifest), built from config.controller_settings() (SingleRowConfig). Everything
above stays: hold / fresh-input rule, terminal decisions never lost, corridor lost -> pause + standstill press, geometry
guard, rolling inner-wheel limit, N9 no-response budget, OutputGuard. Differences:
  start_run(t, row_len_m=None)  the row length is optional: Backstop.start_single_row (distance cap
                single_row_max_row_length_m, or min(cap, L - margin) with an entered length; motion-time cap from the
                controller's lowest target speed). A backstop end is OPERATOR_OR_BACKSTOP, never a row end.
  first press   the initial press of a run (reason 'await_hold', no resume yet) made while the inputs have not yet been
                fresh for fresh_recover_s is kept for single_row_initial_press_grace_s while it stays HELD: the run
                starts once the inputs complete their fresh time inside the grace (input warm-up after Start / Arm,
                e.g. LiDAR latency 0.2-0.3 s; SIM dev round 1 lidar_latency). A release, or the grace running out,
                drops it ('press_ignored_inputs_not_fresh': release and press again). Every resume after a pause still
                needs a NEW press made while the inputs are fresh (the stale-input rule is unchanged).
  terminal holds  the controller's own reasons (row_end, obstacle_ahead, end_structure_ahead,
                steering_authority_insufficient, start_misaligned, ...) end the run like every non-recoverable HOLD.
  reporting     Decision.summary['single_row'] (every tick): phase, estimated lateral / heading error, confidence,
                L/R duty, differential / saturation / authority, bias, base duty, row-end distance, distance, speed,
                stop reason and category (single_row_stop_category: the vendored table, DRIVE / SENSOR as sub-labels,
                plus the executor-only reasons). single_row_geometry(): the estimator-frame corridor estimate, base pose
                and a bounded recent path for the operator page (display only; nothing feeds back into control).

Front-wheel steering (config vehicle_profile 'front_steer', steering branch P4, design 6.5-6.7,
docs/steering/EXECUTOR_STEER_KO.md): the output is Decision(throttle, steer_mrad) (both sides get the throttle duty; left /
right carry the throttle as the host bridge's requested.left / right do). Everything above stays (hold / fresh-input rule,
terminal decisions never lost, corridor lost, geometry guard, N9 no-response budget, backstop) except:
  controller    a steering controller is required (controller_factory: the P8 steering controller or a SIM scripted
                controller, sim/ubts/steering/scripted_control.py); its commands carry throttle and steer_mrad (None =
                keep the steering). Only a driving state (FOLLOW / REDUCE) may change the steering; any other state must
                keep it (guard_violation:steer_in_non_driving_state).
  guard         SteerOutputGuard (guard.py): throttle int 0..duty cap, steer_mrad int inside the equivalent soft limit and
                within the slew limit (moving / standstill); no inner-wheel rule, no rolling limit.
  steering hold the steering target of every PAUSED / ENDED decision is the held one: the last command of the run, or the
                measured steering of the first steering status before any command (design 6.5: no steering change after
                PAUSED / ENDED, no centring at standstill). None until a steering status was seen (SHADOW).
  steering feedback  SteerMonitor: the bridge's steering block of every reply (ExecutorLoop -> feed_steering). While
                feedback is required (MOTOR, armed): ACTIVE needs it fresh (steer_status_max_age_s), calibrated and valid,
                else the run ENDS (steering_feedback_stale / steering_uncalibrated / steering_feedback_invalid); a press
                is ignored while it is not usable ('press_ignored_steering_feedback'); a steering fault of the MCU or a
                host latch in it ends the run in PAUSED or ACTIVE with its design 6.6 reason (config.STEER_FAULT_REASONS).
                steering_authority_insufficient (profile D2 placeholder) is the Runner's own rule (corridor valid only).
  snapshot      the side-sweep rotation centre is the rear axle (steer_rotation_centre_x_m, design 6.7); the estimator's
                pivot values (pivot_clearance_m, pivot_radius_required_m) are removed (off: a steered vehicle does not
                pivot); steer_measured_mrad / steer_status_age_s / steer_command_mrad are added.
  refused       bench runs and calibration runs (left / right duty tables).
"""
import collections
from dataclasses import dataclass, field
import hashlib
import json
import math
import os
import sys
import time

import numpy as np

from . import calib_runs as CR
from .config import FRONT_STEER, SINGLE_ROW_MODE, single_row_time_cap_speed, steer_fault_reason
from .guard import Backstop, MotionOdometer, OutputGuard, SteerOutputGuard, sensor_reports_motion

VENDOR_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vendor')
# The vendored SIM-tree packages are subpackages of this package's vendor folder (a namespace package, never on sys.path):
# SIM-tree name -> vendor subpackage. Their own names never shadow the SIM tree's closed_loop / orchard_perception
# (check_pins, A15; review P4R-02).
VENDOR_PACKAGES = {'closed_loop': 'closed_loop_copy', 'orchard_perception': 'orchard_perception_copy'}
IDLE, PAUSED, ACTIVE, ENDED = 'IDLE', 'PAUSED', 'ACTIVE', 'ENDED'
SCAN_MAX_POINTS = 4096          # orchard_perception.scan_corridor CorridorConfig.max_scan_points
SCAN_MAX_RANGE_M = 30.          # scan_corridor refuses range_max > 30
DRIVING_CONTROLLER_STATES = ('FOLLOW', 'REDUCE')
# review S4: a bench run (wheels off the ground) ends when the estimator sees the vehicle move
BENCH_MAX_SPEED_M_S = .1
BENCH_MAX_DISTANCE_M = .2
BENCH_ODOM_LOST_S = 1.          # review_safety_day2 N6
NO_RESPONSE_CUMULATIVE_FACTOR = 2.   # review_safety_day2 N9: x the single-press budget (ramp + grace + no response)
# review F2: sustained-motion detection for the rolling inner-wheel threshold (hysteresis)
ROLLING_ENTER_SPEED_M_S = .10
ROLLING_EXIT_SPEED_M_S = .05
CALIB_KINDS = CR.KINDS          # calibration run kinds 'arc' (C5) / 'rolldown' (C3b), calib_runs.py
# vendored modules by their SIM-tree source path (vendor/SOURCE_MANIFEST.json source_path; stable over vendor renames)
VENDOR_SOURCES = dict(estimator='sim/ubts/closed_loop/estimator.py', controllers='sim/ubts/closed_loop/controllers.py',
                      single_row='sim/ubts/closed_loop/single_row.py',
                      scan_corridor='sim/ubts/orchard_perception/scan_corridor.py',
                      actuator_prior='sim/ubts/closed_loop/actuator_prior.py')
SINGLE_ROW_VENDOR_PATH = VENDOR_SOURCES['single_row']
# single-row stop categories (single_row_stop_category): the vendored table first; these executor-only pause / end reasons
# are not in it. Display / log only - the SIM evaluation uses the vendored table (PREREG); nothing re-maps a table reason.
SINGLE_ROW_EXECUTOR_CATEGORIES = {
    'operator_stop': 'OPERATOR_OR_BACKSTOP',                 # page / dashboard STOP (emergency_stop)
    'reacquire_needs_press': 'PERCEPTION_OR_LOCALISATION',   # corridor re-acquisition wait (Runner pause)
    'stall_guard': 'SENSOR',                                 # loop stall pause (the repeat limit is SENSOR in the table)
    'native_cross_check_cost_occupied': 'OBSTACLE',          # optional native cross-check veto (executor_main)
    'bridge_error': 'SYSTEM_FAULT', 'bridge_latched': 'SYSTEM_FAULT', 'bridge_arm_failed': 'SYSTEM_FAULT',
    'shutdown': 'SYSTEM_FAULT', 'loop_exception': 'SYSTEM_FAULT',
    # front_steer (design 6.6): the categories of evaluate_steer.PROPOSED_STEERING_STOP_TABLE (a proposal; the steering
    # pre-registration fixes the table). steering_authority_insufficient is in the vendored table already.
    'steering_feedback_invalid': 'CONTROL_AUTHORITY', 'steering_feedback_stale': 'CONTROL_AUTHORITY',
    'steering_not_following': 'CONTROL_AUTHORITY', 'steering_stalled': 'CONTROL_AUTHORITY',
    'steering_ineffective': 'CONTROL_AUTHORITY', 'steering_uncalibrated': 'CONTROL_AUTHORITY',
    'steering_polarity_fault': 'CONTROL_AUTHORITY', 'steering_driver_fault': 'CONTROL_AUTHORITY',
    'stop_unconfirmed': 'SYSTEM_FAULT',
    # front_steer link / session faults (config.STEER_LINK_FAULT_REASONS; executor-only, a proposal)
    'mcu_reset': 'SYSTEM_FAULT', 'link_watchdog': 'SYSTEM_FAULT', 'link_bad_lines': 'SYSTEM_FAULT',
    'link_ttl_expired': 'SYSTEM_FAULT', 'link_command_rejected': 'SYSTEM_FAULT', 'link_hardware_fault': 'SYSTEM_FAULT',
    'mcu_state_mismatch': 'SYSTEM_FAULT', 'arm_unconfirmed': 'SYSTEM_FAULT', 'arm_rejected': 'SYSTEM_FAULT',
    'link_identity_mismatch': 'SYSTEM_FAULT', 'bridge_internal_error': 'SYSTEM_FAULT', 'mcu_disarmed': 'SYSTEM_FAULT',
}
SINGLE_ROW_SUB_LABELS = dict(DRIVE='CONTROL_AUTHORITY', SENSOR='PERCEPTION_OR_LOCALISATION')   # PREREG sub-labels
SINGLE_ROW_NOT_STOPS = ('await_hold', 'resumed', 'no_run', 'press_ignored_inputs_not_fresh',
                        'press_ignored_not_standstill', 'press_ignored_steering_feedback')
# front_steer (design 6.7): estimator snapshot values of a differential pivot about the ICR behind the LiDAR; removed
PIVOT_SNAPSHOT_KEYS = ('pivot_clearance_m', 'pivot_radius_required_m')
STEER_INVALID_FACTOR = 1.5      # front_steer: |measured| beyond 1.5 x the soft limit is no steering angle (decode fault)
# front_steer (review RS-01): steering_host counters of MCU condition events no status bit or latch may keep, and the
# MCU fault name each one stands for (its reason comes from config.steer_fault_reason)
LINK_EVENT_COUNTERS = (('mcu_fault_TTL', 'TTL'),)
SR_PATH_STEP_M = .2             # operator-page path: one base point per 0.2 m of estimator motion ...
SR_PATH_POINTS = 150            # ... at most 150 points (30 m)
SR_PHASE_EVENTS = 6             # last controller phase events in the status


class VendorError(ImportError):
    pass


def _inside(path, root):
    try:
        return os.path.commonpath([os.path.realpath(path), os.path.realpath(root)]) == os.path.realpath(root)
    except ValueError:
        return False


def vendor_manifest(vendor_dir=VENDOR_DIR):
    with open(os.path.join(vendor_dir, 'SOURCE_MANIFEST.json'), 'r', encoding='utf-8') as fh:
        return json.load(fh)


def verify_vendor(vendor_dir=VENDOR_DIR):
    """Recompute every vendored file's sha256; VendorError on any mismatch or missing file."""
    manifest = vendor_manifest(vendor_dir)
    if manifest.get('modified') is not False:
        raise VendorError('vendor manifest does not declare unmodified sources')
    if not manifest.get('files'):
        raise VendorError('vendor manifest lists no files')
    for entry in manifest['files']:
        path = os.path.join(vendor_dir, *entry['vendored_path'].split('/'))
        try:
            with open(path, 'rb') as fh:
                data = fh.read()
        except OSError as exc:
            raise VendorError('vendored file missing: %s (%s)' % (entry['vendored_path'], exc)) from None
        if len(data) != entry['bytes'] or hashlib.sha256(data).hexdigest() != entry['sha256']:
            raise VendorError('vendored file modified: %s' % entry['vendored_path'])
    return manifest


_MISSING = object()


class _SimTreeNames:
    """While the vendored estimator is imported: the SIM-tree module names it imports absolutely
    (orchard_perception.scan_corridor) point at the vendored copies in sys.modules; afterwards the previous entries
    (the SIM tree's, or none) are put back. Not thread-safe: load_vendor() runs at start-up."""

    def __init__(self, aliases):
        self.aliases = aliases
        self.saved = {}

    def __enter__(self):
        for name, module in self.aliases.items():
            self.saved[name] = sys.modules.get(name, _MISSING)
            sys.modules[name] = module
        return self

    def __exit__(self, *exc):
        for name, previous in self.saved.items():
            if previous is _MISSING:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous
        return False


def _vendor_hashes():
    """{source_path: sha256} of the vendor manifest (after verify_vendor)."""
    return {e['source_path']: e['sha256'] for e in verify_vendor()['files']}


def _check_module_bytes(module, source_path, want):
    path = getattr(module, '__file__', None)
    if path is None:
        raise VendorError('module without a file for %s' % source_path)
    if source_path not in want:
        raise VendorError('vendor manifest lists no %s' % source_path)
    with open(path, 'rb') as fh:
        if hashlib.sha256(fh.read()).hexdigest() != want[source_path]:
            raise VendorError('module %s is not byte-identical to the vendored %s' % (path, source_path))


def load_vendor():
    """Import the vendored estimator/controller after verifying their hashes; (estimator, controllers).

    The copies are subpackages of this package's vendor folder (row_executor.vendor.closed_loop_copy and
    .orchard_perception_copy, VENDOR_PACKAGES): the vendor folder is never put on sys.path and the SIM-tree names
    closed_loop / orchard_perception are never registered, so the SIM tree and the vendor can be imported in one process
    (no import shadowing, check_pins / A15). The vendored estimator imports orchard_perception.scan_corridor by its
    SIM-tree name: during that one import the name points at the vendored copy (_SimTreeNames), so the estimator binds
    the vendored detector. Every imported file is checked against the manifest sha256 again."""
    manifest = verify_vendor()
    want = {e['source_path']: e['sha256'] for e in manifest['files']}
    from .vendor import closed_loop_copy as loop_package       # noqa: F401  (vendored, hash-verified)
    from .vendor import orchard_perception_copy as corridor_package
    from .vendor.orchard_perception_copy import scan_corridor as corridor
    from .vendor.closed_loop_copy import controllers
    with _SimTreeNames({'orchard_perception': corridor_package, 'orchard_perception.scan_corridor': corridor}):
        from .vendor.closed_loop_copy import estimator
    for module, source in ((controllers, VENDOR_SOURCES['controllers']), (estimator, VENDOR_SOURCES['estimator']),
                           (corridor, VENDOR_SOURCES['scan_corridor']),
                           (_prior_module(estimator), VENDOR_SOURCES['actuator_prior'])):
        if not _inside(module.__file__, VENDOR_DIR):
            raise VendorError('vendored module resolved outside the vendor dir: %s' % module.__file__)
        _check_module_bytes(module, source, want)
    if getattr(estimator, 'observe_scan_corridor', None) is not corridor.observe_scan_corridor:
        raise VendorError('the vendored estimator does not use the vendored scan corridor detector')
    estimator.VENDOR_COMMIT = manifest['source_commit']
    return estimator, controllers


def verified_modules(estimator, controllers):
    """Accept ALREADY IMPORTED estimator/controllers modules (e.g. the SIM tree's, for the SIM-in-the-loop
    test where the SIM harness owns the names closed_loop/orchard_perception) only if their files and the
    scan_corridor module the estimator uses are byte-identical to the vendored manifest. Field runs never
    use this: they import the vendored copies through load_vendor()."""
    manifest = verify_vendor()
    want = {e['source_path']: e['sha256'] for e in manifest['files']}
    corridor = sys.modules.get(getattr(estimator, 'observe_scan_corridor').__module__)
    for module, source in ((estimator, VENDOR_SOURCES['estimator']), (controllers, VENDOR_SOURCES['controllers']),
                           (corridor, VENDOR_SOURCES['scan_corridor']),
                           (_prior_module(estimator), VENDOR_SOURCES['actuator_prior'])):
        _check_module_bytes(module, source, want)
    estimator.VENDOR_COMMIT = manifest['source_commit']
    return estimator, controllers


def _prior_module(estimator):
    """The actuator prior module the given estimator module imported (closed_loop/actuator_prior.py next to it)."""
    module = sys.modules.get(getattr(getattr(estimator, 'ActuatorPrior', None), '__module__', None))
    if module is None:
        raise VendorError('the estimator module has no actuator prior module')
    return module


def single_row_module(controllers):
    """closed_loop.single_row from the SAME package as the given (verified) controllers module: the vendor package (field
    runs, after load_vendor: row_executor.vendor.closed_loop_copy) or the SIM tree (verified_modules, SIM-in-the-loop).
    Its file must be byte-identical to the vendor manifest entry and its SingleRowController must derive from that
    controllers module's RowController."""
    import importlib
    package = controllers.__name__.rsplit('.', 1)[0]
    module = importlib.import_module(package + '.single_row')
    want = _vendor_hashes()
    if SINGLE_ROW_VENDOR_PATH not in want:
        raise VendorError('vendor manifest lists no %s' % SINGLE_ROW_VENDOR_PATH)
    path = getattr(module, '__file__', None)
    if path is None:
        raise VendorError('single_row module without a file')
    with open(path, 'rb') as fh:
        if hashlib.sha256(fh.read()).hexdigest() != want[SINGLE_ROW_VENDOR_PATH]:
            raise VendorError('module %s is not byte-identical to the vendored %s' % (path, SINGLE_ROW_VENDOR_PATH))
    if not issubclass(module.SingleRowController, controllers.RowController):
        raise VendorError('single_row module does not belong to the given controllers module')
    return module


def single_row_stop_category(module, reason):
    """(category, table label, source) of a single-row pause / end reason; (None, None, None) when it is not a stop.

    The vendored module's stop_category() table first (sub-labels DRIVE -> CONTROL_AUTHORITY, SENSOR ->
    PERCEPTION_OR_LOCALISATION, as the PREREG enum), then SINGLE_ROW_EXECUTOR_CATEGORIES for executor-only reasons;
    anything else is UNCLASSIFIED (fail closed: shown, never a row end)."""
    if reason is None or reason in SINGLE_ROW_NOT_STOPS:
        return None, None, None
    label = module.stop_category(reason) if module is not None else None
    if label is not None:
        return SINGLE_ROW_SUB_LABELS.get(label, label), label, 'single_row.stop_category'
    label = SINGLE_ROW_EXECUTOR_CATEGORIES.get(reason)
    if label is not None:
        return SINGLE_ROW_SUB_LABELS.get(label, label), label, 'executor'
    return 'UNCLASSIFIED', None, 'none'


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _finite_or_none(value, digits=4):
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return round(value, digits) if math.isfinite(value) else None


# --- scan ---------------------------------------------------------------------------------------
class ScanRejected(ValueError):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


class ScanAdapter:
    """Raw LaserScan fields -> estimator scan (see module docstring). Never edits the caller's array."""

    def __init__(self, yaw_offset_deg=179.08, body_mask=(), grid_count=3240,
                 grid_angle_min_rad=-3.1415927410125732, grid_increment_rad=.0019398535368964076,
                 mask_limit_m=1.2, range_min_floor_m=.05, max_trim_beams=4):
        self.offset = math.radians(float(yaw_offset_deg))
        if type(grid_count) is not int or grid_count < 3 or not grid_increment_rad > 0:
            raise ValueError('invalid beam grid')
        self.grid_count = grid_count
        self.grid_min = float(grid_angle_min_rad)
        self.grid_inc = float(grid_increment_rad)
        table = np.zeros(grid_count, dtype=np.float64)
        for first, last, max_range in body_mask:
            if not (0 <= first <= last < grid_count and 0 < max_range <= mask_limit_m):
                raise ValueError('invalid body mask segment %r' % ((first, last, max_range),))
            table[first:last + 1] = np.maximum(table[first:last + 1], float(max_range))
        self.mask_table = table
        self.range_min_floor = float(range_min_floor_m)
        self.max_trim = int(max_trim_beams)
        self._index_key = None
        self._index = None

    @classmethod
    def from_config(cls, cfg):
        return cls(cfg.lidar_yaw_offset_deg, cfg.body_mask, cfg.beam_grid_count, cfg.beam_grid_angle_min_rad,
                   cfg.beam_grid_increment_rad, cfg.body_mask_limit_m, cfg.range_min_floor_m,
                   cfg.max_scan_trim_beams)

    def _grid_index(self, n, angle_min, increment):
        key = (n, angle_min, increment)
        if key != self._index_key:
            angles = angle_min + np.arange(n, dtype=np.float64) * increment
            index = np.rint((angles - self.grid_min) / self.grid_inc).astype(np.int64) % self.grid_count
            self._index_key, self._index = key, index
        return self._index

    def convert(self, ranges, angle_min, increment, range_max, range_min=0.):
        try:
            amin, inc, rmax, rmin = float(angle_min), float(increment), float(range_max), float(range_min)
        except (TypeError, ValueError):
            raise ScanRejected('invalid_scan_geometry') from None
        if not all(math.isfinite(v) for v in (amin, inc, rmax, rmin)) or inc <= 0 or abs(amin) > 2 * math.pi:
            raise ScanRejected('invalid_scan_geometry')
        try:
            r = np.array(ranges, dtype=np.float64)
        except (TypeError, ValueError):
            raise ScanRejected('invalid_ranges') from None
        if r.ndim != 1:
            raise ScanRejected('invalid_ranges')
        n = int(r.shape[0])
        if n > SCAN_MAX_POINTS:
            raise ScanRejected('scan_capacity_exceeded')
        limit = 2 * math.pi + inc * 1.01        # the scan_corridor geometry rule
        keep = min(n, int(math.floor(limit / inc)))
        while keep > 0 and keep * inc > limit:
            keep -= 1
        trimmed = n - keep
        if keep < 3:
            raise ScanRejected('too_few_beams')
        if trimmed > self.max_trim:
            raise ScanRejected('invalid_scan_geometry')
        r = r[:keep]
        rmax_eff = min(rmax, SCAN_MAX_RANGE_M)
        rmin_eff = max(rmin, self.range_min_floor)
        if not rmin_eff < rmax_eff:
            raise ScanRejected('invalid_scan_geometry')
        with np.errstate(invalid='ignore'):
            valid = np.isfinite(r) & (r >= rmin_eff) & (r < rmax_eff)
        r[~valid] = np.inf
        mask_r = self.mask_table[self._grid_index(keep, amin, inc)]
        masked = valid & (r <= mask_r)
        r[masked] = np.inf
        stats = dict(beams_in=n, beams=keep, trimmed=trimmed, invalid=int(keep - np.count_nonzero(valid)),
                     masked=int(np.count_nonzero(masked)),
                     near_0p5=int(np.count_nonzero(np.isfinite(r) & (r < .5))))
        return r, wrap(amin + self.offset), inc, rmax_eff, stats


class NearConfirmFilter:
    """C1 gate P1 (same algorithm as replay_adapters.NearTransientFilter, the evaluated variant).

    A return nearer than near_m survives only if EACH of the previous confirm_scans scans (unfiltered
    near returns, newest first) had a return within tol_m + age * tol_step_m of it, in the same rotated
    laser frame. The 10/10 false front stops were one-scan clusters of 5-14 identical near returns.
    Real obstacles persist (0.1 s per scan); a new one is accepted confirm_scans scans later. Edits r in
    place (-> inf) and returns the number removed. One instance per run (state = recent scans).
    """

    def __init__(self, confirm_scans=2, near_m=2., tol_m=.35, tol_step_m=.1):
        from scipy.spatial import cKDTree       # scipy is already an estimator dependency
        self._tree = cKDTree
        self.confirm_scans, self.near_m = int(confirm_scans), float(near_m)
        self.tol_m, self.tol_step_m = float(tol_m), float(tol_step_m)
        self.history = []
        self.removed = 0

    @classmethod
    def from_config(cls, cfg):
        if cfg.near_confirm_scans <= 0:
            return None
        return cls(cfg.near_confirm_scans, cfg.near_confirm_range_m, cfg.near_confirm_tol_m,
                   cfg.near_confirm_tol_step_m)

    def apply(self, r, angle_min, increment):
        idx = np.nonzero(np.isfinite(r) & (r < self.near_m))[0]
        a = angle_min + idx * increment
        pts = np.column_stack((r[idx] * np.cos(a), r[idx] * np.sin(a)))
        removed = 0
        if len(self.history) >= self.confirm_scans and len(idx):
            keep = np.ones(len(idx), dtype=bool)
            for age, prev in enumerate(self.history[:self.confirm_scans]):
                if not len(prev):
                    keep[:] = False
                    break
                d, _ = self._tree(prev).query(pts, distance_upper_bound=self.tol_m + age * self.tol_step_m)
                keep &= np.isfinite(d)
            removed = int((~keep).sum())
            r[idx[~keep]] = np.inf
        # (fewer than confirm_scans scans so far: every near return is kept, as in the evaluated variant)
        self.history.insert(0, pts)
        del self.history[self.confirm_scans:]
        self.removed += removed
        return removed


class ScanStamper:
    """Estimator scan stamps on the receipt (time.monotonic) clock.

    'receipt'        stamp = receipt - latency (WEEKEND_PLAN 1.3).
    'header_offset'  (C1 gate P4) stamp = header + median over the last `window` scans of
                     (receipt - header) - latency, never later than the receipt. The header keeps the
                     driver's regular 0.1 s spacing, so a burst of late deliveries (several scans received
                     within a few ms) does not produce near-zero estimator dt. A system-clock step of the
                     header (GNSS time sync) changes the offset at once: |offset - median| > reset_s
                     restarts the median at the new offset (counted). A missing/non-finite header falls
                     back to receipt - latency (counted).
    Every received scan must be observed (also the ones the caller then drops), in receipt order.
    """

    def __init__(self, mode='header_offset', latency_s=.08, window=50, reset_s=.5):
        if mode not in ('receipt', 'header_offset'):
            raise ValueError('unknown scan stamp mode')
        self.mode, self.latency_s = mode, float(latency_s)
        self.window, self.reset_s = int(window), float(reset_s)
        self.offsets = collections.deque(maxlen=self.window)
        self.counts = dict(stamped=0, resets=0, header_missing=0, capped=0)
        self.last_offset = None

    @classmethod
    def from_config(cls, cfg):
        return cls(cfg.scan_stamp_mode, cfg.scan_latency_s, cfg.scan_stamp_window, cfg.scan_stamp_reset_s)

    def median(self):
        return float(np.median(self.offsets)) if self.offsets else None

    def stamp(self, receipt, header=None):
        receipt = float(receipt)
        self.counts['stamped'] += 1
        if self.mode == 'receipt':
            return receipt - self.latency_s
        try:
            header = None if header is None else float(header)
        except (TypeError, ValueError):
            header = None
        if header is None or not math.isfinite(header):
            self.counts['header_missing'] += 1
            return receipt - self.latency_s
        offset = receipt - header
        self.last_offset = offset
        if self.offsets and abs(offset - self.median()) > self.reset_s:
            self.offsets.clear()
            self.counts['resets'] += 1
        self.offsets.append(offset)
        stamp = header + self.median() - self.latency_s
        if stamp > receipt:
            self.counts['capped'] += 1
            stamp = receipt
        return stamp

    def summary(self):
        median = self.median()
        return dict(mode=self.mode, median_offset_s=None if median is None else round(median, 4),
                    last_offset_s=None if self.last_offset is None else round(self.last_offset, 4), **self.counts)


# --- IMU ----------------------------------------------------------------------------------------
class ImuYawRate:
    """Gravity-projected yaw rate (causal EMA 'up'), identical to the replay's gravity_projected_yaw."""

    def __init__(self, tau_s=1., sign=1):
        if sign not in (-1, 1) or not tau_s > 0:
            raise ValueError('invalid IMU yaw settings')
        self.tau_s, self.sign = float(tau_s), int(sign)
        self.state = None
        self.last_t = None
        self.rejected = 0

    def update(self, t, gyro_xyz, acc_xyz):
        """(yaw_rate [rad/s], (ax, ay, az)) or None for an unusable sample (counted in .rejected)."""
        try:
            t = float(t)
            g = tuple(float(v) for v in gyro_xyz)
            a = tuple(float(v) for v in acc_xyz)
        except (TypeError, ValueError):
            self.rejected += 1
            return None
        if len(g) != 3 or len(a) != 3 or not all(math.isfinite(v) for v in g + a + (t,)):
            self.rejected += 1
            return None
        if self.last_t is not None and t < self.last_t:
            self.rejected += 1
            return None
        dt = .05 if self.state is None else min(.2, max(1e-3, t - self.last_t))
        if self.state is None:
            self.state = list(a)
        k = 1 - math.exp(-dt / self.tau_s)
        self.state = [s + (v - s) * k for s, v in zip(self.state, a)]
        norm = math.sqrt(sum(v * v for v in self.state))
        self.last_t = t
        if not norm > 1e-6:
            self.rejected += 1
            return None
        yaw = self.sign * sum(gv * sv for gv, sv in zip(g, self.state)) / norm
        return yaw, a


# --- runner -------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Decision:
    left: int
    right: int
    state: str          # Runner state: IDLE / PAUSED / ACTIVE / ENDED
    reason: str
    terminal: bool      # True once the run has ended (ENDED)
    summary: dict = field(default_factory=dict)
    # front_steer only (None in the differential profile): the drive duty of both sides [%] and the equivalent steering
    # target [mrad] (None: no target known yet, SHADOW without steering feedback). left = right = throttle then.
    throttle: object = None
    steer_mrad: object = None


class SteerMonitor:
    """Steering status input of the front_steer Runner (design 6.6; config block 'front-wheel steering').

    feed(t, block, host)  the steering block of a bridge status (design 4.2 keys, steer_bridge_core STEERING_BLOCK_KEYS)
                          and its steering_host block, received at executor time t (a drive / keepalive / arm / status
                          reply); the executor never reads the MCU or the plant otherwise
    age(now)              now - t + block age_ms / 1000 (inf without a block, or with a missing or negative age_ms)
    measured_mrad()       the block's measured_mrad when it is a finite number, else None
    confirmed()           the host has the MCU's ARM confirmation (steering_host arm_state 'confirmed'); the faults of a
                          block from before the confirmation are history (the confirmation clears them)
    link_events()         MCU condition events of this arm that no status bit or latch may show (LINK_EVENT_COUNTERS:
                          an F TTL line counted by the host, steering_host counters mcu_fault_TTL, above its value before
                          the ARM confirmation). A C line that reaches the MCU right after a TTL brake clears the TTL bit
                          before the next status line, so the bit alone can miss the event (review RS-01)
    fault()               the design 6.6 reason of the block's faults (MCU fl names + host latches + link_events(),
                          config.steer_fault_reason), None before the confirmation or without a steering fault
    check(now)            the ACTIVE rule, first problem: steering_feedback_stale (no block, or older than
                          steer_status_max_age_s), steering_uncalibrated (calibrated is not True), steering_feedback_invalid
                          (measured not a finite number, or beyond STEER_INVALID_FACTOR x the soft limit)
    usable(now)           confirmed, check() None and fault() None (a resume press needs this)
    authority(now, steer, throttle, s)  steering_authority_insufficient, the profile D2 placeholder rule: while driving
                          (throttle > 0) with a valid corridor, |steer| >= steer_authority_fraction x the soft limit of that
                          side for longer than steer_authority_s and the |lateral error| at the end not below its value at
                          the start; None otherwise. Without a corridor there is no lateral error: not evaluated.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.last = None            # (t, block, host)
        self.fed = 0
        self.rejected = 0
        self._sat = None            # (t, |lateral|) of the saturation start
        self.authority_saturated_s = 0.
        self._link_base = {}        # host counter -> value before the ARM confirmation of this arm
        self._link_events = set()   # LINK_EVENT_COUNTERS names seen since the confirmation

    def reset_run(self):
        self._sat = None
        self.authority_saturated_s = 0.

    def feed(self, t, block, host=None):
        if not isinstance(block, dict):
            self.rejected += 1
            return False
        host = dict(host) if isinstance(host, dict) else {}
        self.last = (float(t), dict(block), host)
        self.fed += 1
        self._note_link_events(host)
        return True

    def _note_link_events(self, host):
        counters = host.get('counters')
        if not isinstance(counters, dict):
            return                                   # a host without counters: the status bits only
        confirmed = host.get('arm_state') == 'confirmed'
        for key, name in LINK_EVENT_COUNTERS:
            value = counters.get(key, 0)             # the host counts from the first event (an absent key is 0)
            if type(value) is not int:
                continue
            if not confirmed:
                self._link_base[key] = value         # before the confirmation of this arm: the baseline
                self._link_events.discard(name)
            elif value > self._link_base.setdefault(key, value):
                self._link_events.add(name)

    def link_events(self):
        return sorted(self._link_events)

    def age(self, now):
        if self.last is None:
            return math.inf
        t, block, _ = self.last
        age_ms = block.get('age_ms')
        if type(age_ms) is not int or age_ms < 0:
            return math.inf
        return max(0., float(now) - t) + age_ms / 1000.

    def measured_mrad(self):
        if self.last is None:
            return None
        value = self.last[1].get('measured_mrad')
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            return None
        return float(value)

    def faults(self):
        if self.last is None:
            return []
        faults = self.last[1].get('faults')
        return [str(f) for f in faults] if isinstance(faults, (list, tuple)) else []

    def confirmed(self):
        return self.last is not None and self.last[2].get('arm_state') == 'confirmed'

    def fault(self):
        if not self.confirmed():
            return None
        return steer_fault_reason(self.faults() + self.link_events())

    def check(self, now):
        cfg = self.cfg
        if self.last is None or self.age(now) > cfg.steer_status_max_age_s:
            return 'steering_feedback_stale'
        if self.last[1].get('calibrated') is not True:
            return 'steering_uncalibrated'
        measured = self.measured_mrad()
        limit = STEER_INVALID_FACTOR * max(cfg.steer_limit_left_mrad, cfg.steer_limit_right_mrad)
        if measured is None or abs(measured) > limit:
            return 'steering_feedback_invalid'
        return None

    def usable(self, now):
        return self.confirmed() and self.check(now) is None and self.fault() is None

    def authority(self, now, steer_mrad, throttle, s):
        cfg = self.cfg
        lateral = s.get('lateral_offset') if s.get('corridor_valid') else None
        if (type(steer_mrad) is not int or not throttle or lateral is None
                or not isinstance(lateral, (int, float)) or not math.isfinite(lateral)):
            self._sat = None
            return None
        limit = cfg.steer_limit_left_mrad if steer_mrad >= 0 else cfg.steer_limit_right_mrad
        if abs(steer_mrad) < cfg.steer_authority_fraction * limit - 1e-9:
            self._sat = None
            return None
        if self._sat is None:
            self._sat = (float(now), abs(float(lateral)))
            return None
        held = float(now) - self._sat[0]
        self.authority_saturated_s = max(self.authority_saturated_s, held)
        if held > cfg.steer_authority_s and abs(float(lateral)) >= self._sat[1] - 1e-9:
            return 'steering_authority_insufficient'
        return None

    def summary(self, now):
        if self.last is None:
            return dict(available=False, fed=self.fed)
        t, block, host = self.last
        age = self.age(now)
        return dict(available=True, fed=self.fed, age_s=None if not math.isfinite(age) else round(age, 3),
                    measured_mrad=_finite_or_none(self.measured_mrad(), 1), calibrated=block.get('calibrated'),
                    servo_state=block.get('servo_state'), faults=self.faults(), link_events=self.link_events(),
                    arm_state=host.get('arm_state'), stop_state=host.get('stop_state'), fault_reason=self.fault())


class Runner:
    """Not thread-safe: the caller serialises feed_*/tick/pause/end_run (one lock in the node)."""

    def __init__(self, config, estimator_factory=None, controller_factory=None, vendor_modules=None):
        config.validate()
        self.config = config
        estimator, controllers = load_vendor() if vendor_modules is None else verified_modules(*vendor_modules)
        self.estimator_config = estimator.EstimatorConfig(**config.estimator_overrides())
        mode, overrides = config.controller_settings()
        self.single_row = None              # the vendored single_row module in mode 'single_row', else None
        if mode == SINGLE_ROW_MODE:
            single_row = single_row_module(controllers)
            self.single_row = single_row
            self.controller_config = single_row.SingleRowConfig(**overrides)
            if not self.controller_config.side_clearance_check:
                raise ValueError('single_row keeps the side clearance rule ON')
            default_controller = lambda: single_row.SingleRowController(self.controller_config)   # noqa: E731
        else:
            self.controller_config = controllers.config_for(mode, **overrides)
            if self.controller_config.side_clearance_check != (mode == 'continuous_side'):
                raise ValueError('side clearance rule does not match the configured mode')
            default_controller = lambda: controllers.RowController(self.controller_config)          # noqa: E731
        if self.controller_config.mode != 'continuous' or self.controller_config.pivot_failure_fallback:
            raise ValueError('controller must be the forward-only continuous mode')
        self.terminal_holds = tuple(controllers.TERMINAL_HOLDS) + (
            tuple(h for h in self.single_row.TERMINAL_HOLDS if h not in controllers.TERMINAL_HOLDS)
            if self.single_row is not None else ())
        self.recoverable_holds = tuple(controllers.RECOVERABLE_HOLDS)
        # The vendored stop model (stopping_distance / required_front) for the bench interlock and the
        # backstop report; this instance is never stepped.
        self.stop_model = controllers.RowController(self.controller_config)
        self.vendor_commit = estimator.VENDOR_COMMIT
        self._new_estimator = estimator_factory or (lambda: estimator.SensorEstimator(self.estimator_config))
        self._new_controller = controller_factory or default_controller
        self.scan_adapter = ScanAdapter.from_config(config)
        self.imu = ImuYawRate(config.imu_up_tau_s, config.imu_yaw_sign)
        self.guard = OutputGuard.from_config(config)
        # front_steer (module docstring): a steering controller, SteerOutputGuard, the steering feedback monitor
        self.steering = config.vehicle_profile == FRONT_STEER
        self.steer_monitor = None
        self.steer_required = False         # front_steer: set by the caller; steering feedback expected (MOTOR, armed)
        self._tick_now = None
        if self.steering:
            if controller_factory is None:
                raise ValueError('front_steer needs a steering controller (controller_factory: the P8 steering '
                                 'controller or a SIM scripted controller); the vendored controllers command left / right')
            self.guard = SteerOutputGuard.from_config(config)
            self.steer_monitor = SteerMonitor(config)
        self.stats = dict(imu_in=0, imu_rejected=0, imu_stamp_drop=0, scans_in=0, scans_rejected={},
                          scan_stamp_drop=0, last_scan_proc_s=None, max_scan_proc_s=0.)
        self.run_id = 0
        self.state = IDLE
        self.run_kind = None
        self.estimator = None
        self.near_filter = None
        self.controller = None
        self.backstop = None
        self.calib = None
        self.last_snapshot = None
        self.reason = 'no_run'
        self.end_reason = None
        self.end_note = None
        self.last_receipt = dict(scan=None, imu=None)
        self._reset_run_state()

    # --- run state ----------------------------------------------------------------------------
    def _reset_run_state(self):
        self._prev_hold = True          # a press must be seen AFTER the run starts
        self._fresh_since = None
        self._last_tick = None
        self._last_output = (0, 0)
        self._last_imu_stamp = None
        self._last_scan_stamp = None
        self.stale_events = 0
        self.stall_guard_pauses = 0
        self.pauses = 0
        self.resumes = 0
        self.controllers_built = 0
        self.last_command = None
        self.last_violation = None
        self._bench_elapsed = 0.
        self._bench_last = None
        self._bench_odometer = None
        self._controller_t0 = None
        # review fixes (Fix A)
        self._press_needs_standstill = False      # corridor-lost pause: a resume press needs standstill
        self._press_pending_since = None          # single-row initial press held through the input warm-up
        self._corridor_lost_at = []               # counted distance at every corridor-lost pause of the run
        self.corridor_lost_pauses = 0
        self._rolling = False                     # sustained motion since the resume (rolling threshold)
        self.inner_limited_ticks = 0
        # day-2 review fixes
        self.no_motion_drive_s = 0.               # N9: commanded output without observed motion, whole run
        self._bench_odom_bad_since = None         # N6
        # single-row reporting (mode 'single_row'; display / log only)
        self._sr_telemetry = {}                   # last SingleRowController.telemetry() of this run
        self._sr_path = collections.deque(maxlen=SR_PATH_POINTS)   # base (x, y) in the estimator frame
        self._sr_events = []                      # last controller phase events of this run
        self._sr_events_seen = (None, 0)          # (controller, phase events already copied)
        self._sr_first_stop = None                # the first pause / end of the run after a resume
        # front_steer: the steering target of every zero-throttle decision (module docstring 'steering hold')
        self._held_steer = None
        if getattr(self, 'steer_monitor', None) is not None:
            self.steer_monitor.reset_run()

    def _begin(self, t, kind):
        self.run_id += 1
        self.run_kind = kind
        self.estimator = self._new_estimator()
        self.near_filter = NearConfirmFilter.from_config(self.config)
        self.controller = None
        self.calib = None
        self.last_snapshot = None
        self.end_reason = None
        self.end_note = None
        self._reset_run_state()
        self._last_tick = float(t)
        self.state, self.reason = PAUSED, 'await_hold'
        return self.run_id

    def start_run(self, t, row_len_m=None):
        """A row run. row_len_m: operator-measured length from the start pose to the row end (required except in mode
        'single_row', where it is optional: Backstop.start_single_row)."""
        if self.state in (PAUSED, ACTIVE):
            raise RuntimeError('run_in_progress')
        backstop = Backstop.from_config(self.config)
        if self.single_row is not None:                                # raises before any state change
            backstop.start_single_row(t, 0., self.config.single_row_max_row_length_m,
                                      single_row_time_cap_speed(self.config), row_len_m)
        else:
            backstop.start(t, 0., row_len_m, self.config.v_measured_m_s)   # raises before any state change
        run_id = self._begin(t, 'row')
        self.backstop = backstop
        return run_id

    def start_bench(self, t):
        if self.steering:
            raise RuntimeError('front_steer_has_no_bench_pattern')
        if self.config.profile != 'bench' or not self.config.bench_pattern:
            raise RuntimeError('bench_pattern_requires_bench_profile')
        if self.state in (PAUSED, ACTIVE):
            raise RuntimeError('run_in_progress')
        self.backstop = None
        return self._begin(t, 'bench')

    def start_calibration(self, t, plan):
        """A calibration run (calib_runs.CalibPlan from make_plan; field profile only)."""
        if self.steering:
            raise RuntimeError('front_steer_calibration_runs_are_left_right_tables')
        if self.config.profile != 'field':
            raise RuntimeError('calibration_needs_field_profile')
        if not isinstance(plan, CR.CalibPlan) or plan.kind not in CALIB_KINDS:
            raise RuntimeError('calibration_plan_required')
        if self.state in (PAUSED, ACTIVE):
            raise RuntimeError('run_in_progress')
        self.backstop = None
        run_id = self._begin(t, plan.kind)
        self.calib = CR.CalibRun(plan, self.stop_model, self.controller_config)
        return run_id

    def _end(self, reason, note=None):
        pending = self._pending_terminal()
        if note is None and pending is not None and pending != reason:
            note = 'controller_pending:' + pending       # e.g. a backstop ended the run during a terminal stop
        self.state, self.reason, self.end_reason = ENDED, reason, reason
        self.end_note = note
        self._sr_note_stop(ENDED, reason)
        self.controller = None
        if self.calib is not None:
            self.calib.end(reason)          # idempotent: keeps a reason the run decided itself
        self._last_output = (0, 0)

    def end_run(self, t, why='operator_end', note=None):
        if self.state in (PAUSED, ACTIVE):
            self._end(str(why), note)

    def _pending_terminal(self):
        """The reason of a terminal stop the controller has decided but not yet completed, else None.

        STOP_REQUESTED with pending ('hold' | 'front', reason): terminal ('front' ends in HOLD front_blocked in
        the continuous modes). 'wait_corridor' is the corridor-lost stop (handled as a pause). Anything else,
        including a missing pending decision, is treated as terminal (fail closed).
        """
        c = self.controller
        if c is None or getattr(c, 'state', None) != 'STOP_REQUESTED':
            return None
        pending = getattr(c, 'pending', None)
        if not pending:
            return 'stop_without_decision'
        kind, reason = pending[0], pending[1]
        if kind == 'wait_corridor':
            return None
        return str(reason)

    def _pause(self, why):
        if self.state != ACTIVE:
            return
        if self.run_kind in CALIB_KINDS:
            # a calibration run never resumes mid-table: any pause ends it (the results so far are kept)
            self._end('calib_' + str(why))
            return
        pending = self._pending_terminal()
        if pending is not None:
            # review S1 / F1: a terminal decision outranks any pause (never PAUSED, never lost)
            self._end(pending, note='terminal_stop_pending_at_' + str(why))
            return
        self.state, self.reason = PAUSED, why
        self._sr_note_stop(PAUSED, why)
        self.controller = None
        self._last_output = (0, 0)
        self.pauses += 1
        if why == 'stall_guard':
            self.stall_guard_pauses += 1
            if self.stall_guard_pauses > self.config.max_stall_guard_pauses:
                self._end('stall_guard_repeat_limit')
        elif why in ('stale_lidar', 'stale_imu'):
            self.stale_events += 1
            if self.stale_events > self.config.max_stale_events:
                self._end('stale_repeat_limit')

    def pause(self, t, why):
        """External pause (e.g. the loop-stall watchdog with why='stall_guard')."""
        self._pause(str(why))

    def _resume(self, now):
        self.state, self.reason = ACTIVE, 'resumed'
        self.resumes += 1
        self._press_needs_standstill = False
        self._rolling = False
        if self.run_kind == 'row':
            self.controller = self._new_controller()
            self._controller_t0 = now
            self.controllers_built += 1
        elif self.run_kind in CALIB_KINDS:
            self.calib.begin(now)
        else:
            self._bench_last = now

    # --- inputs -------------------------------------------------------------------------------
    def _run_open(self):
        # The estimator keeps running after the run ENDED (until the next Start replaces it), so the stop
        # after an end decision is observed (stillness for the bridge stop, overrun distance in the log).
        return self.state in (PAUSED, ACTIVE, ENDED) and self.estimator is not None

    def feed_imu(self, t, gyro_xyz, acc_xyz):
        self.stats['imu_in'] += 1
        out = self.imu.update(t, gyro_xyz, acc_xyz)
        if out is None:
            self.stats['imu_rejected'] += 1
            return False
        self.last_receipt['imu'] = float(t)
        if not self._run_open():
            return True
        if self._last_imu_stamp is not None and t <= self._last_imu_stamp:
            self.stats['imu_stamp_drop'] += 1
            return False
        self._last_imu_stamp = float(t)
        self.estimator.on_imu(float(t), float(out[0]), out[1])
        return True

    def feed_scan(self, t, ranges, angle_min, increment, range_max, range_min=0., stamp=None):
        """t = receipt time; stamp = estimator stamp (default t - scan_latency_s, see ScanStamper)."""
        self.stats['scans_in'] += 1
        try:
            r, amin, inc, rmax, stats = self.scan_adapter.convert(ranges, angle_min, increment, range_max, range_min)
        except ScanRejected as exc:
            rejected = self.stats['scans_rejected']
            rejected[exc.reason] = rejected.get(exc.reason, 0) + 1
            return dict(accepted=False, reason=exc.reason)
        self.last_receipt['scan'] = float(t)
        stats['accepted'] = True
        if not self._run_open():
            return stats
        stamp = float(t) - self.config.scan_latency_s if stamp is None else float(stamp)
        if not math.isfinite(stamp) or stamp > float(t):
            self.stats['scan_stamp_drop'] += 1
            stats['accepted'] = False
            stats['reason'] = 'stamp_invalid'
            return stats
        if self._last_scan_stamp is not None and stamp <= self._last_scan_stamp:
            self.stats['scan_stamp_drop'] += 1
            stats['accepted'] = False
            stats['reason'] = 'stamp_not_increasing'
            return stats
        self._last_scan_stamp = stamp
        c0 = time.perf_counter()
        if self.near_filter is not None:
            stats['near_unconfirmed'] = self.near_filter.apply(r, amin, inc)
        self.estimator.on_scan(stamp, r, amin, inc, rmax)
        proc = time.perf_counter() - c0
        self.stats['last_scan_proc_s'] = proc
        self.stats['max_scan_proc_s'] = max(self.stats['max_scan_proc_s'], proc)
        stats['proc_s'] = proc
        return stats

    def feed_steering(self, t, block, host=None):
        """front_steer: the steering block (and steering_host block) of a bridge reply received at executor time t.
        Before the first command of a run the held steering target is the first measured steering (design 6.5: the
        steering stays where it is). False in the differential profile or for a block that is not a dict."""
        if self.steer_monitor is None:
            return False
        ok = self.steer_monitor.feed(t, block, host)
        if ok and self._held_steer is None:
            measured = self.steer_monitor.measured_mrad()
            if measured is not None:
                cfg = self.config
                self._held_steer = int(max(-cfg.steer_limit_right_mrad, min(cfg.steer_limit_left_mrad,
                                                                            int(round(measured)))))
        return ok

    def _steer_snapshot(self, s, now):
        """front_steer snapshot (design 6.7): rotation centre of the side-sweep rule = the rear axle, the pivot values of
        the estimator removed (off), the steering feedback and the held command added."""
        if not self.steering:
            return s
        s['rotation_centre_x_m'] = float(self.config.steer_rotation_centre_x_m)
        s['rotation_centre_sd_m'] = float(self.config.steer_rotation_centre_sd_m)
        for key in PIVOT_SNAPSHOT_KEYS:
            s.pop(key, None)
        age = self.steer_monitor.age(now)
        s.update(steer_measured_mrad=self.steer_monitor.measured_mrad(),
                 steer_status_age_s=age if math.isfinite(age) else None, steer_command_mrad=self._held_steer)
        return s

    def _steer_ok(self, now):
        """front_steer: may a press resume now? (steering feedback usable whenever it is required)"""
        return not self.steering or not self.steer_required or self.steer_monitor.usable(now)

    # --- tick ---------------------------------------------------------------------------------
    def _decision(self, left, right, snapshot=None, extra=None):
        summary = self._summary(snapshot)
        if extra:
            summary.update(extra)
        if 'single_row' in summary:
            summary['single_row'].update(left_pct=int(left), right_pct=int(right))     # the executor's output
        if self.steering:
            # every caller passes the zero output: throttle 0 and the held steering target (no change after a pause /
            # end); the guard records it, so the next command is rate-checked against what was sent
            if self._tick_now is not None:
                self.guard.hold(self._tick_now, self._held_steer)
            if 'single_row' in summary:
                summary['single_row'].update(throttle_pct=int(left), steer_mrad=self._held_steer)
            return Decision(int(left), int(right), self.state, self.reason, self.state == ENDED, summary,
                            throttle=int(left), steer_mrad=self._held_steer)
        return Decision(int(left), int(right), self.state, self.reason, self.state == ENDED, summary)

    def _steer_decision(self, throttle, steer_mrad, snapshot, extra):
        """front_steer: an output that passed the guard (throttle on both sides, the new held steering target)."""
        summary = self._summary(snapshot)
        summary.update(extra)
        if 'single_row' in summary:
            summary['single_row'].update(left_pct=int(throttle), right_pct=int(throttle), throttle_pct=int(throttle),
                                         steer_mrad=int(steer_mrad))
        return Decision(int(throttle), int(throttle), self.state, self.reason, False, summary, throttle=int(throttle),
                        steer_mrad=int(steer_mrad))

    def tick(self, now, hold_active):
        decision = self._tick(now, hold_active)
        if self.calib is not None and self.state == ENDED and self.last_snapshot is not None:
            self.calib.observe_stop(float(now), self.last_snapshot)      # C7: the stop after the run's 0/0
            if decision.summary.get('calib') is not None:
                decision.summary['calib'] = self.calib.status()
        return decision

    def _tick(self, now, hold_active):
        now = float(now)
        self._tick_now = now
        hold = hold_active is True
        if self.state == ENDED and self.estimator is not None:
            self._prev_hold = hold
            if self._last_tick is None or now >= self._last_tick:
                self._last_tick = now
                self.last_snapshot = self._steer_snapshot(self.estimator.snapshot(now), now)
                if self.single_row is not None:
                    self._sr_track_path()               # the stop after the end is shown on the operator page
            return self._decision(0, 0, self.last_snapshot)
        if self.state in (IDLE, ENDED) or self.estimator is None:
            self._prev_hold = hold
            return self._decision(0, 0)
        if self._last_tick is not None and now < self._last_tick:
            self._end('clock_regression')
            return self._decision(0, 0)
        if self.state == ACTIVE and self._last_tick is not None and now - self._last_tick > self.config.stall_guard_s:
            self._pause('stall_guard')
        dt_tick = 0. if self._last_tick is None else max(0., now - self._last_tick)
        self._last_tick = now
        s = self._steer_snapshot(self.estimator.snapshot(now), now)
        self.last_snapshot = s
        if self.single_row is not None:
            self._sr_track_path()
        fresh = (s['lidar_age_s'] <= self.config.lidar_max_age_s and s['imu_age_s'] <= self.config.imu_max_age_s)
        if not fresh:
            self._fresh_since = None
        elif self._fresh_since is None:
            self._fresh_since = now
        if self.state == ENDED:
            return self._decision(0, 0, s)
        if self.run_kind == 'bench':
            why = self._bench_interlock(s, now)
            if why is not None:
                self._prev_hold = hold
                self._end(why)
                return self._decision(0, 0, s)
        if self.backstop is not None:
            moving = self._last_output != (0, 0) or not s.get('still', False)
            reason = self.backstop.check(now, s.get('distance_m'), moving, sensor_reports_motion(s))
            if reason is not None:
                self._end(reason)
                return self._decision(0, 0, s)
        if self.steering and self.steer_required:
            # a steering fault of the MCU or a host latch in the steering status: its design 6.6 reason, PAUSED or ACTIVE
            why = self.steer_monitor.fault()
            if why is not None:
                self._prev_hold = hold
                self._end(why, note='steering_status_faults:' + ','.join(self.steer_monitor.faults()
                                                                         + self.steer_monitor.link_events()))
                return self._decision(0, 0, s)
        rising = hold and not self._prev_hold
        self._prev_hold = hold
        ready = fresh and now - self._fresh_since >= self.config.fresh_recover_s
        steer_ok = self._steer_ok(now)
        if self._press_pending_since is not None and (not hold or self.state != PAUSED):
            self._press_pending_since = None          # released: the held initial press lapses (no later start)
        if self.state == PAUSED and rising:
            if ((not ready or not steer_ok) and self._initial_press_grace_s() > 0. and self.reason == 'await_hold'
                    and not self.resumes):
                # single-row initial start only: the press is kept while it stays HELD and the inputs complete
                # their fresh_recover_s within the grace (input warm-up right after Start / Arm, e.g. LiDAR latency)
                self._press_pending_since = now
            elif not ready:
                # Ignored: the operator must release and press again once the inputs are fresh.
                self.reason = 'press_ignored_inputs_not_fresh'
            elif not steer_ok:
                # front_steer: the steering status is required and not usable (stale, uncalibrated, invalid, not
                # confirmed after the arm): release and press again
                self.reason = 'press_ignored_steering_feedback'
            elif self._press_needs_standstill and not self._standstill(s):
                # corridor-lost pause: the press must come after the vehicle was seen at rest
                self.reason = 'press_ignored_not_standstill'
            else:
                self._resume(now)
        elif self.state == PAUSED and self._press_pending_since is not None:
            if ready and steer_ok:
                self._press_pending_since = None
                self._resume(now)
            elif now - self._press_pending_since > self._initial_press_grace_s():
                self._press_pending_since = None      # the grace ran out: as an ignored press (release, press again)
                self.reason = 'press_ignored_inputs_not_fresh' if not ready else 'press_ignored_steering_feedback'
        if self.state == PAUSED:
            self._last_output = (0, 0)
            return self._decision(0, 0, s)
        # ACTIVE
        if not hold:
            self._pause('hold_released')
            return self._decision(0, 0, s)
        if not fresh:
            self._pause('stale_lidar' if s['lidar_age_s'] > self.config.lidar_max_age_s else 'stale_imu')
            return self._decision(0, 0, s)
        if self.steering:
            return self._steer_active(now, s, dt_tick)
        rolling = False
        guard = self.guard
        if self.run_kind in CALIB_KINDS:
            left, right, why = self.calib.step(now, s)
            if why is not None:
                self._end(why)
                return self._decision(0, 0, s)
            ctrl_state, ctrl_reason = 'CALIB', 'calib_%s_%s' % (self.run_kind, self.calib.phase)
            guard = self.calib.guard        # explicit minimum duty instead of the inner-wheel rule
        elif self.run_kind == 'bench':
            left, right, why = self._bench_command(now)
            if why is not None:
                self._end(why)
                return self._decision(0, 0, s)
            ctrl_state, ctrl_reason = 'BENCH', 'bench_pattern'
        else:
            # The vendored RowController starts with since = 0.0 (SIM episodes start at t = 0):
            # stepped on the raw monotonic clock its REACQUIRE timeout (6 s) would fire at once.
            # It therefore runs on a clock that starts at 0 at its construction (resume). The
            # snapshot holds only ages/durations, so nothing else depends on the time origin.
            command = self.controller.step(now - self._controller_t0, s)
            self.last_command = command
            if self.single_row is not None:
                self._sr_collect(self.controller)
            ctrl_state, ctrl_reason = command.state, command.reason
            ctrl = dict(controller_state=ctrl_state, controller_reason=ctrl_reason)
            if command.state == 'HOLD':
                if command.reason in self.recoverable_holds:
                    self._pause(command.reason)      # same single stale rule
                else:
                    self._end(command.reason)        # terminal holds (and unknown reasons) end the run
                return self._decision(0, 0, s, ctrl)
            left, right = command.left, command.right
            if (left or right) and command.state not in DRIVING_CONTROLLER_STATES:
                self.last_violation = 'output_in_non_driving_state'
                self._end('guard_violation:output_in_non_driving_state')
                return self._decision(0, 0, s)
            if self._controller_waits_for_corridor(command):
                self._corridor_lost(s, command)
                return self._decision(0, 0, s, ctrl)
            if command.state in DRIVING_CONTROLLER_STATES:
                why = self.geometry_guard(s)
                if why is not None:
                    self._end(why)
                    return self._decision(0, 0, s, dict(ctrl, raw_left=left, raw_right=right))
                rolling = self._update_rolling(s)
                left, right = self._inner_wheel_limit(left, right, rolling)
            why = self._no_response_budget(s, dt_tick)
            if why is not None:
                self._end(why)
                return self._decision(0, 0, s, ctrl)
        out_left, out_right, violation = guard.apply(left, right, rolling=rolling)
        if violation is not None:
            if guard is not self.guard:
                self.guard.violations += 1
            self.last_violation = violation
            self._end('guard_violation:' + violation)
            return self._decision(0, 0, s, dict(raw_left=_int_or_repr(left), raw_right=_int_or_repr(right)))
        self.reason = ctrl_reason
        self._last_output = (out_left, out_right)
        return self._decision(out_left, out_right, s, dict(controller_state=ctrl_state, controller_reason=ctrl_reason,
                                                           rolling=rolling))

    def _steer_active(self, now, s, dt_tick):
        """ACTIVE tick of a front_steer row run (module docstring): the steering feedback rule, the steering controller,
        the corridor / geometry / steering authority / no-response rules and SteerOutputGuard. The hold / fresh-input
        rules ran before (_tick)."""
        if self.steer_required:
            why = self.steer_monitor.check(now)
            if why is not None:
                self._end(why)
                return self._decision(0, 0, s)
        command = self.controller.step(now - self._controller_t0, s)
        self.last_command = command
        if self.single_row is not None:
            self._sr_collect(self.controller)
        ctrl_state, ctrl_reason = command.state, command.reason
        ctrl = dict(controller_state=ctrl_state, controller_reason=ctrl_reason)
        if command.state == 'HOLD':
            if command.reason in self.recoverable_holds:
                self._pause(command.reason)      # same single stale rule
            else:
                self._end(command.reason)        # terminal holds (and unknown reasons) end the run
            return self._decision(0, 0, s, ctrl)
        throttle = getattr(command, 'throttle', None)
        steer = getattr(command, 'steer_mrad', None)
        steer = self._held_steer if steer is None else steer
        driving = command.state in DRIVING_CONTROLLER_STATES
        if throttle and not driving:
            self.last_violation = 'output_in_non_driving_state'
            self._end('guard_violation:output_in_non_driving_state')
            return self._decision(0, 0, s)
        if not driving and steer != self._held_steer:
            # design 6.5: the steering changes only in a driving state (a stop keeps it where it is)
            self.last_violation = 'steer_in_non_driving_state'
            self._end('guard_violation:steer_in_non_driving_state')
            return self._decision(0, 0, s, dict(raw_steer=_int_or_repr(steer)))
        if self._controller_waits_for_corridor(command):
            self._corridor_lost(s, command)
            return self._decision(0, 0, s, ctrl)
        if driving:
            why = self.geometry_guard(s)
            if why is not None:
                self._end(why)
                return self._decision(0, 0, s, dict(ctrl, raw_throttle=_int_or_repr(throttle),
                                                    raw_steer=_int_or_repr(steer)))
            why = self.steer_monitor.authority(now, steer, throttle, s)
            if why is not None:
                self._end(why)
                return self._decision(0, 0, s, ctrl)
        why = self._no_response_budget(s, dt_tick)
        if why is not None:
            self._end(why)
            return self._decision(0, 0, s, ctrl)
        out_throttle, out_steer, violation = self.guard.apply(now, throttle, steer)
        if violation is not None:
            self.last_violation = violation
            self._end('guard_violation:' + violation)
            return self._decision(0, 0, s, dict(raw_throttle=_int_or_repr(throttle), raw_steer=_int_or_repr(steer)))
        self.reason = ctrl_reason
        self._last_output = (out_throttle, out_throttle)
        self._held_steer = out_steer
        return self._steer_decision(out_throttle, out_steer, s, ctrl)

    def _initial_press_grace_s(self):
        """Single-row mode: how long an initial press (await_hold, no resume yet) is kept while held and the inputs
        are not yet fresh for fresh_recover_s; 0 in the other modes (a press must come after the inputs are fresh)."""
        if self.single_row is None:
            return 0.
        return float(self.config.single_row_initial_press_grace_s)

    def geometry_guard(self, s):
        """C1 gate P3: None, 'guard_heading' or 'guard_lateral' for a driving tick with a valid band."""
        cfg = self.config
        if not s.get('corridor_valid'):
            return None
        heading, lateral, width = s.get('heading_error'), s.get('lateral_offset'), s.get('corridor_width')
        if cfg.guard_heading_deg is not None and heading is not None and math.isfinite(heading):
            if abs(math.degrees(heading)) > cfg.guard_heading_deg:
                return 'guard_heading'
        if (cfg.guard_lateral_margin_m is not None and lateral is not None and width is not None
                and math.isfinite(lateral) and math.isfinite(width)):
            half = .5 * width - .5 * cfg.footprint_width_m
            if abs(lateral) > half + cfg.guard_lateral_margin_m:
                return 'guard_lateral'
        return None

    # --- review fixes (Fix A) --------------------------------------------------------------------
    def _standstill(self, s):
        """Sensor-observed stillness for the controller's still_confirm_s (missing still_s = not still)."""
        still_s = s.get('still_s', 0.)
        return (s.get('still') is True and isinstance(still_s, (int, float)) and math.isfinite(still_s)
                and still_s >= self.controller_config.still_confirm_s)

    def _controller_waits_for_corridor(self, command):
        """True when the controller started its corridor-lost stop (STOP_REQUESTED, pending 'wait_corridor') or is in
        any REACQUIRE other than the initial 'start' (from which it would drive on without a new press)."""
        if command.state == 'STOP_REQUESTED':
            pending = getattr(self.controller, 'pending', None)
            return bool(pending) and pending[0] == 'wait_corridor'
        return command.state == 'REACQUIRE' and command.reason != 'start'

    def _corridor_lost(self, s, command):
        """review S2 / F7: pause (a NEW press after standstill resumes) or end at the repeat limit."""
        cfg = self.controller_config
        position = self.backstop.travelled_m if self.backstop is not None else float(s.get('distance_m') or 0.)
        recent = [p for p in self._corridor_lost_at if position - p <= cfg.turn_window_m]
        self._corridor_lost_at.append(position)
        self.corridor_lost_pauses += 1
        if len(recent) >= cfg.max_stops_per_window:
            self._end('corridor_lost_repeat_limit',
                      note='%d corridor losses within %g m' % (len(recent) + 1, cfg.turn_window_m))
            return
        why = 'corridor_lost' if command.state == 'STOP_REQUESTED' or command.reason == 'corridor_lost_wait' \
            else 'reacquire_needs_press'
        self._pause(why)                # the pending 'wait_corridor' is dropped with the controller
        self._press_needs_standstill = True

    def _no_response_budget(self, s, dt):
        """review_safety_day2 N9: None, or 'drive_no_response_repeat' when the commanded output (previous tick non-zero)
        has seen no motion for more than the cumulative budget since the last observed motion (whole run)."""
        cc = self.controller_config
        speed = s.get('speed')
        still = s.get('still') is True and isinstance(s.get('still_s'), (int, float)) and s['still_s'] >= cc.still_confirm_s
        slow = (s.get('odom_valid', True) is True and isinstance(speed, (int, float)) and math.isfinite(speed)
                and speed < cc.drive_min_speed_m_s)
        if not (still or slow):
            self.no_motion_drive_s = 0.
            return None
        if self._last_output != (0, 0):
            self.no_motion_drive_s += dt
        budget = NO_RESPONSE_CUMULATIVE_FACTOR * (cc.mcu_ramp_s + cc.drive_grace_s + cc.drive_no_response_s)
        if self.no_motion_drive_s > budget:
            return 'drive_no_response_repeat'
        return None

    def _bench_interlock(self, s, now=None):
        """review S4: None, 'bench_motion_detected', 'bench_odometry_invalid' or 'bench_front_blocked';
        day-2 N6: 'bench_odometry_lost' (ACTIVE and scan matching invalid for more than BENCH_ODOM_LOST_S)."""
        if self.state == ACTIVE and now is not None and s.get('odom_valid') is not True:
            if self._bench_odom_bad_since is None:
                self._bench_odom_bad_since = float(now)
            elif float(now) - self._bench_odom_bad_since > BENCH_ODOM_LOST_S:
                return 'bench_odometry_lost'
        else:
            self._bench_odom_bad_since = None
        if self._bench_odometer is None:
            distance0 = s.get('distance_m')
            try:
                self._bench_odometer = MotionOdometer(0. if distance0 is None else distance0)
            except (TypeError, ValueError):
                return 'bench_odometry_invalid'
        if not self._bench_odometer.update(s.get('distance_m', self._bench_odometer.last_m),
                                           sensor_reports_motion(s)):
            return 'bench_odometry_invalid'
        speed = s.get('speed', 0.)
        speed = 0. if speed is None else speed
        if not (isinstance(speed, (int, float)) and math.isfinite(speed)) or abs(speed) > BENCH_MAX_SPEED_M_S:
            return 'bench_motion_detected'
        if self._bench_odometer.counted_m > BENCH_MAX_DISTANCE_M:
            return 'bench_motion_detected'
        front = s.get('front_clearance_m')
        if front is not None and not front >= self.stop_model.required_front(max(0., speed)):
            return 'bench_front_blocked'
        return None

    def _update_rolling(self, s):
        """review F2: sustained motion since the resume, with hysteresis (fail-safe: anything odd = not rolling)."""
        speed = s.get('speed')
        ok = (s.get('still') is False and s.get('odom_valid', True) is True
              and isinstance(speed, (int, float)) and math.isfinite(speed))
        if not ok:
            self._rolling = False
        elif self._rolling:
            self._rolling = speed >= ROLLING_EXIT_SPEED_M_S
        else:
            self._rolling = speed >= ROLLING_ENTER_SPEED_M_S
        return self._rolling

    def _inner_wheel_limit(self, left, right, rolling):
        """review F2 (only when a rolling threshold is configured): before sustained motion the inner wheel must
        reach the static breakaway + margin; lower the differential symmetrically (sum kept). Anything that does
        not fit (non-int, negative, mean below the minimum) is left to the OutputGuard."""
        if self.guard.rolling_inner_min_pct is None or rolling:
            return left, right
        need = self.guard.inner_min_pct
        if (need is None or type(left) is not int or type(right) is not int or left < 0 or right < 0
                or not (left or right) or min(left, right) >= need or left + right < 2 * need):
            return left, right
        self.inner_limited_ticks += 1
        total = left + right
        return (need, total - need) if left < right else (total - need, need)

    def _bench_command(self, now):
        steps = self.config.bench_pattern
        if self._bench_last is not None:
            self._bench_elapsed += max(0., now - self._bench_last)
        self._bench_last = now
        elapsed = 0.
        for duration, left, right in steps:
            elapsed += duration
            if self._bench_elapsed < elapsed:
                return left, right, None
        return 0, 0, 'bench_pattern_complete'

    # --- single-row reporting (mode 'single_row'; display / log only, nothing feeds back into control) ----------------
    def _sr_note_stop(self, state, reason):
        """Record the first pause / end of the run (the operator policy of the SIM evaluation: the first stop counts)."""
        if self.single_row is None or self._sr_first_stop is not None:
            return
        category, label, source = single_row_stop_category(self.single_row, reason)
        if category is None:
            return
        self._sr_first_stop = dict(t=None if self._last_tick is None else round(self._last_tick, 3), state=state,
                                   reason=reason, category=category, label=label, source=source)

    def _sr_collect(self, controller):
        """Copy the controller's telemetry and its new phase events (bounded) after a step."""
        telemetry = getattr(controller, 'telemetry', None)
        self._sr_telemetry = telemetry() if callable(telemetry) else {}
        events = getattr(controller, 'phase_events', None)
        if not isinstance(events, list):
            return
        owner, seen = self._sr_events_seen
        if owner is not controller:
            seen = 0
        for event in events[seen:]:
            self._sr_events.append(list(event))
        del self._sr_events[:-SR_PHASE_EVENTS]
        self._sr_events_seen = (controller, len(events))

    def _sr_base_pose(self):
        """Base (footprint centre) pose (x, y, yaw) in the estimator's odometry frame at the last scan, or None."""
        est = self.estimator
        pose = getattr(est, 'pose', None)
        if pose is None or getattr(est, 'last_scan_stamp', None) is None:
            return None
        try:
            x, y, yaw = (float(v) for v in pose[:3])
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return None
        lever = self.estimator_config.lidar_x_in_base_m
        return x - lever * math.cos(yaw), y - lever * math.sin(yaw), yaw

    def _sr_track_path(self):
        base = self._sr_base_pose()
        if base is None:
            return
        if not self._sr_path or math.hypot(base[0] - self._sr_path[-1][0], base[1] - self._sr_path[-1][1]) >= SR_PATH_STEP_M:
            self._sr_path.append((round(base[0], 3), round(base[1], 3)))

    def _sr_block(self, s):
        """Decision.summary['single_row']: the single-row telemetry of this tick (left / right added by _decision)."""
        t = self._sr_telemetry or {}
        s = s if s is not None else (self.last_snapshot or {})
        cfg = self.config
        lidar_age, imu_age = s.get('lidar_age_s'), s.get('imu_age_s')
        fresh = (isinstance(lidar_age, (int, float)) and lidar_age <= cfg.lidar_max_age_s
                 and isinstance(imu_age, (int, float)) and imu_age <= cfg.imu_max_age_s)
        valid = s.get('corridor_valid') is True
        corridor_age = s.get('corridor_age_s') if valid else None
        odom = s.get('odom_valid') is True
        trust = t.get('gyro_trust')
        if not fresh or not valid:
            level = 'NONE'
        elif (corridor_age is not None and corridor_age > .6) or not odom or (trust is not None and trust < 1.):
            level = 'LOW'
        else:
            level = 'OK'
        if self.state == ENDED:
            stop_reason = self.end_reason
        elif self.state == PAUSED and self.reason not in SINGLE_ROW_NOT_STOPS:
            stop_reason = self.reason
        else:
            stop_reason = None
        category, label, source = single_row_stop_category(self.single_row, stop_reason)
        controller = self.controller
        if controller is not None and callable(getattr(controller, 'current_phase', None)):
            phase = controller.current_phase()
        else:
            phase = {ENDED: 'STOPPED', PAUSED: 'PAUSED', ACTIVE: 'START'}.get(self.state, 'IDLE')
        heading = s.get('heading_error') if valid else None
        end = s.get('row_end_ahead_m') if valid else None
        remaining = None if end is None else end - cfg.single_row_end_stop_offset_m
        return dict(
            phase=phase, localisation='SENSOR_ONLY_SINGLE_ROW',
            lateral_error_m=_finite_or_none(s.get('lateral_offset'), 3) if valid else None,
            heading_error_deg=None if heading is None else _finite_or_none(math.degrees(heading), 2),
            lateral_reference_m=_finite_or_none(t.get('lateral_reference_m'), 3),
            lookahead_error_deg=_finite_or_none(t.get('lookahead_error_deg'), 2),
            confidence=dict(level=level, inputs_fresh=bool(fresh), corridor_valid=valid,
                            corridor_age_s=_finite_or_none(corridor_age, 3),
                            corridor_updates=s.get('corridor_updates'), odom_valid=odom,
                            gyro_trust=_finite_or_none(trust, 3)),
            differential_pct=_finite_or_none(t.get('differential_pct'), 2),
            raw_differential_pct=_finite_or_none(t.get('raw_differential_pct'), 2),
            differential_limit_pct=_finite_or_none(t.get('differential_limit_pct'), 2),
            saturated=bool(t.get('saturated', False)),
            saturation_fraction=_finite_or_none(t.get('saturation_fraction'), 4),
            authority_state=t.get('authority_state'), bias_pct=_finite_or_none(t.get('bias_pct'), 3),
            base_duty_pct=_finite_or_none(t.get('base_duty_pct'), 2),
            target_speed_m_s=_finite_or_none(t.get('target_speed_m_s'), 3),
            target_yaw_rate_deg_s=_finite_or_none(t.get('target_yaw_rate_deg_s'), 3),
            measured_yaw_rate_deg_s=_finite_or_none(math.degrees(s.get('yaw_rate', 0.) or 0.), 3),
            row_end_ahead_m=_finite_or_none(end, 3), end_remaining_m=_finite_or_none(remaining, 3),
            distance_m=_finite_or_none(s.get('distance_m'), 3), speed_m_s=_finite_or_none(s.get('speed'), 3),
            stop_reason=stop_reason, stop_category=category, stop_label=label, stop_category_source=source,
            first_stop=None if self._sr_first_stop is None else dict(self._sr_first_stop),
            phase_events=[list(e) for e in self._sr_events])

    def single_row_geometry(self):
        """Estimator-frame corridor estimate for the operator page canvas (display only; None outside single-row runs).

        frame: the estimator's odometry frame of this run (origin = the start pose, no map, no GNSS). base = footprint
        centre (x, y, yaw) at the last scan; band = tracked corridor axis heading, centreline offset along the axis'
        left normal, width and the along-axis coordinates of each side's observed support end; path = recent base
        positions (one per 0.2 m, at most 30 m)."""
        if self.single_row is None or self.estimator is None:
            return None
        base = self._sr_base_pose()
        band = getattr(self.estimator, 'band', None)
        band_out = None
        if band is not None:
            try:
                band_out = dict(heading_rad=round(float(band.heading), 4), centre_m=round(float(band.center), 3),
                                width_m=round(float(band.width), 3),
                                end_left_m=None if band.end_left is None else round(float(band.end_left), 3),
                                end_right_m=None if band.end_right is None else round(float(band.end_right), 3))
            except (AttributeError, TypeError, ValueError):
                band_out = None
            if band_out is not None and not all(math.isfinite(band_out[k]) for k in ('heading_rad', 'centre_m',
                                                                                   'width_m')):
                band_out = None
        return dict(frame='estimator_odometry_run_start',
                    base=None if base is None else [round(base[0], 3), round(base[1], 3), round(base[2], 4)],
                    band=band_out, path=[list(p) for p in self._sr_path],
                    footprint_m=[self.config.footprint_length_m, self.config.footprint_width_m])

    # --- reporting ----------------------------------------------------------------------------
    def _summary(self, s):
        out = dict(run_id=self.run_id, run_kind=self.run_kind, stale_events=self.stale_events,
                   stall_guard_pauses=self.stall_guard_pauses, pauses=self.pauses, resumes=self.resumes,
                   controllers_built=self.controllers_built, corridor_lost_pauses=self.corridor_lost_pauses)
        if self.end_note:
            out['end_note'] = self.end_note
        if self.inner_limited_ticks:
            out['inner_limited_ticks'] = self.inner_limited_ticks
        if self.no_motion_drive_s:
            out['no_motion_drive_s'] = round(self.no_motion_drive_s, 2)
        if self._press_pending_since is not None:
            out['press_pending'] = True               # single-row initial press held, waiting for fresh inputs
        if self._fresh_since is not None and self._last_tick is not None:
            out['fresh_for_s'] = round(self._last_tick - self._fresh_since, 3)
        if self.backstop is not None:
            out['backstop'] = self.backstop_summary()
        if self.calib is not None:
            out['calib'] = self.calib.status()
            if self.state == ACTIVE and self.calib.last_tick:
                out['calib_tick'] = dict(self.calib.last_tick)
        if self.last_violation:
            out['guard_violation'] = self.last_violation
        if self.single_row is not None and self.run_kind == 'row':
            out['single_row'] = self._sr_block(s)
        if self.steering:
            now = self._tick_now if self._tick_now is not None else 0.
            out['steering'] = dict(self.steer_monitor.summary(now), required=bool(self.steer_required),
                                   command_mrad=self._held_steer,
                                   authority_saturated_s=round(self.steer_monitor.authority_saturated_s, 3))
        if s is None:
            return out
        side_l = s.get('side_clearance_left_m')
        side_r = s.get('side_clearance_right_m')
        heading = s.get('heading_error')
        out.update(lidar_age_s=_finite_or_none(s.get('lidar_age_s'), 3),
                   imu_age_s=_finite_or_none(s.get('imu_age_s'), 3),
                   still=bool(s.get('still')), speed_m_s=_finite_or_none(s.get('speed'), 3),
                   distance_m=_finite_or_none(s.get('distance_m'), 3),
                   yaw_rate_deg_s=_finite_or_none(math.degrees(s.get('yaw_rate', 0.)), 3),
                   corridor_valid=bool(s.get('corridor_valid')), corridor_reason=s.get('corridor_reason'),
                   lateral_offset_m=_finite_or_none(s.get('lateral_offset'), 3),
                   heading_error_deg=None if heading is None else _finite_or_none(math.degrees(heading), 2),
                   corridor_width_m=_finite_or_none(s.get('corridor_width'), 3),
                   row_end_ahead_m=_finite_or_none(s.get('row_end_ahead_m'), 3),
                   front_clearance_m=_finite_or_none(s.get('front_clearance_m'), 3),
                   side_clearance_left_m=_finite_or_none(side_l, 3),
                   side_clearance_right_m=_finite_or_none(side_r, 3),
                   vibration_rms=_finite_or_none(s.get('vibration_rms'), 3))
        if self.controller is not None:
            out['controller_state'] = self.controller.state
            out['controller_reason'] = self.controller.reason
        return out

    def backstop_summary(self):
        """Backstop.summary() + expected_rest_m: where the vehicle comes to rest after the distance rule ends the
        run, from the Start pose = limit + vendored stopping distance at v_measured_m_s (None if unknown)."""
        out = self.backstop.summary()
        v = self.config.v_measured_m_s
        out['expected_rest_m'] = (None if v is None or out.get('limit_m') is None
                                  else round(out['limit_m'] + self.stop_model.stopping_distance(v), 3))
        return out

    def status(self):
        out = dict(state=self.state, reason=self.reason, end_reason=self.end_reason, end_note=self.end_note,
                   run_id=self.run_id,
                   run_kind=self.run_kind, config_digest=self.config.digest(),
                   vendor_commit=self.vendor_commit, feasibility=self.config.feasibility(),
                   stats=dict(self.stats, scans_rejected=dict(self.stats['scans_rejected'])),
                   guard_violations=self.guard.violations)
        if self.steering:
            out.update(vehicle_profile=self.config.vehicle_profile, steer_command_mrad=self._held_steer,
                       steer_feedback_fed=self.steer_monitor.fed)
        return out


def _int_or_repr(value):
    return value if type(value) is int else repr(value)
