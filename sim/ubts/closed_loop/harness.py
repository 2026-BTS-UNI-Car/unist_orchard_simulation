"""One headless closed-loop episode: plant -> sensors -> estimator -> controller.

Timing: plant 100 Hz; LiDAR 10 Hz (ScanSynthesizer gating); raw IMU 20 Hz;
controller 20 Hz; integer duty commands reach the MCU after a command
latency. LiDAR/IMU packets keep their sensor stamps and are delivered after
the configured latency (+ optional per-scan jitter); dropouts/blackouts/sector
occlusion remove data.

Randomness: every stream (plant run constants/process/pivot events/sensor
noise, scene and canopy, LiDAR range noise, scan drops, latency jitter) is
derived from (seed, scenario stream key, purpose), so scenarios with the same
seed do NOT share draws, while modes A/B/C of one scenario and seed do (common
random numbers for the paired comparison).

Truth (plant pose, scene geometry) is used for sensor synthesis and metrics
only. The estimator gets its NOMINAL calibration (EstimatorConfig defaults +
explicit estimator_overrides), never the plant's true lever arm/footprint.

Optional hooks (run_episode keywords; None = the procedural world.build_world,
world.make_lidar and world.footprint_clearance, looked up at call time exactly as
before): world_factory(config.world) -> world, lidar_factory(world, seed) ->
(caster, ScanSynthesizer), clearance_evaluator(world, x, y, yaw, length, width)
-> (hard, branch). E.g. closed_loop.raster_corridor (10/10 reconstructed rows);
they change sensor synthesis and truth metrics only, never the estimator inputs.

Plant and actuation hooks (run_episode keywords; both None = the differential
FieldPlant driven by the controller's left/right duty pair, with results and
traces bit-identical to the version before these hooks):
  plant_factory(params, *, surface, seed, pose, config) -> plant. Default
    field_plant_factory = FieldPlant(params, surface=surface, seed=seed, pose=pose).
  actuation: an object with
    values(command) -> tuple         plant command values from the controller command
    apply(plant, values)             called when the delayed command reaches the plant
    is_reversal(values, last_values) -> bool   (the reversal_commands counter)
    trace_fields(values) -> dict     trace keys written in place of left/right
    initial_values() -> tuple        the values before the first command
    and optionally turn_sign(values) -> -1/0/+1 for the chatter counters; without
    it chatter_flips, chatter_per_min and rapid_flips are None (not computed).
    Default DifferentialActuation: values (command.left, command.right),
    apply = plant.command(left, right), reversal = a sign flip of left or right.
  A steered plant takes (throttle_pct, steer_target_mrad) through its actuation
  (apply -> plant.command_actuation); its command() refuses differential
  commands, so actuation=None with such a plant fails at the first command.
The loop reads from any plant: step(dt), imu_sample(), lidar_pose(), x, y, yaw,
v, w, distance_m, external_yaw_rate, forward_blocked, gyro_bias, pivot_count,
pivot_stalls and pivot_log.
"""
from dataclasses import asdict, dataclass, replace
import heapq
import math
import zlib

import numpy as np

from field_response import FieldPlant, FieldResponseParams, provenance
from .controllers import RowController, config_for
from .estimator import EstimatorConfig, SensorEstimator
from .world import WorldConfig, build_world, footprint_clearance, make_lidar, yaw_quaternion

PURPOSES = dict(plant=1, lidar_noise=2, scan_drop=3, latency=4, world=5)
ACTUATION_METHODS = ('values', 'apply', 'is_reversal', 'trace_fields', 'initial_values')


class DifferentialActuation:
    """Default actuation: the controller's integer left/right duty pair to FieldPlant.command."""

    def values(self, command):
        return (command.left, command.right)

    def apply(self, plant, values):
        plant.command(*values)

    def is_reversal(self, values, last_values):
        return (values[0] * last_values[0] < 0) or (values[1] * last_values[1] < 0)

    def trace_fields(self, values):
        return dict(left=values[0], right=values[1])

    def initial_values(self):
        return (0, 0)

    def turn_sign(self, values):
        diff = values[1] - values[0]
        return 0 if abs(diff) < 3 else (1 if diff > 0 else -1)


def field_plant_factory(params, *, surface, seed, pose, config=None):
    """Default plant_factory: the differential FieldPlant (config is not used). FieldPlant is looked up in this
    module at call time, so a tool that replaces harness.FieldPlant keeps working with plant_factory=None."""
    return FieldPlant(params, surface=surface, seed=seed, pose=pose)


def _checked_actuation(actuation):
    actuation = DifferentialActuation() if actuation is None else actuation
    missing = [name for name in ACTUATION_METHODS if not callable(getattr(actuation, name, None))]
    turn_sign = getattr(actuation, 'turn_sign', None)
    if turn_sign is not None and not callable(turn_sign):
        missing.append('turn_sign (not callable)')
    if missing:
        raise TypeError('actuation %r lacks %s' % (type(actuation).__name__, ', '.join(missing)))
    return actuation, turn_sign


@dataclass(frozen=True)
class EpisodeConfig:
    name: str = 'nominal'
    world: WorldConfig = WorldConfig()
    mode: str = 'hybrid'
    seed: int = 0
    stream_key: str | None = None    # defaults to name: randomness is per (scenario, seed)
    corridor_index: int = 0          # corridor between rows k and k+1 (field y = k * row width)
    start_x_m: float = 2.5           # field frame: row axis +x, corridor centre at y offset 0
    start_y_m: float = 0.            # relative to the start corridor centre
    start_yaw_deg: float = 0.
    surface: str = 'normal'
    plant_overrides: tuple = ()      # ((name, value), ...) SIM sensitivity only
    estimator_overrides: tuple = ()  # nominal calibration changes (never copied from the plant)
    lidar_latency_s: float = 0.
    lidar_latency_jitter_s: float = 0.   # + U(0, jitter) per scan, in-order delivery (one serial stream)
    imu_latency_s: float = .02
    command_latency_s: float = .05   # SIM_ASSUMED bridge/serial latency
    scan_drop_probability: float = 0.
    blackouts: tuple = ()            # ((t0, t1, 'lidar'|'imu'|'both'), ...)
    occlusions: tuple = ()           # ((t0, t1, lo_deg, hi_deg), ...) laser frame, no return
    yaw_disturbances: tuple = ()     # ((t0, t1, deg_s), ...) external body yaw rate (skid)
    forward_blocks: tuple = ()       # ((t0, t1), ...) forward motion blocked (root below the scan plane)
    gyro_noise_scale: float = 1.
    gyro_bias_deg_s: float | None = None
    max_time_s: float = 110.
    controller_overrides: tuple = ()
    next_row_side: str = 'left'      # mission plan: side of the next corridor to be judged (last row)


def stream_seed(config, purpose):
    """Deterministic 32-bit seed for (seed, scenario stream key, purpose)."""
    key = zlib.crc32((config.stream_key or config.name).encode('utf-8'))
    return int(np.random.SeedSequence([config.seed, key, PURPOSES[purpose]]).generate_state(1)[0])


def plant_params(config):
    params = FieldResponseParams()
    overrides = dict(config.plant_overrides)
    if config.gyro_noise_scale != 1.:
        overrides['gyro_still_noise_sd_deg_s'] = params.gyro_still_noise_sd_deg_s * config.gyro_noise_scale
        overrides['gyro_vibration_deg_s_per_m_s2'] = params.gyro_vibration_deg_s_per_m_s2 * config.gyro_noise_scale
    return replace(params, **overrides)


def estimator_config(config):
    return replace(EstimatorConfig(), **dict(config.estimator_overrides))


def describe_config(config):
    """JSON-safe record of the episode configuration and the plant provenance it used."""
    record = asdict(config)
    record['plant_provenance_overrides'] = {
        k: v for k, v in provenance(plant_params(config))['parameters'].items()
        if v['provenance'].startswith('SCENARIO_OVERRIDE')}
    return record


def _finite(value, digits=3):
    """Rounded truth value for the added JSON keys (None when no object is within reach)."""
    return round(float(value), digits) if math.isfinite(value) else None


def run_episode(config, *, record_trace=False, estimator_factory=SensorEstimator, controller_factory=RowController,
                world_factory=None, lidar_factory=None, clearance_evaluator=None, plant_factory=None, actuation=None):
    actuation, turn_sign = _checked_actuation(actuation)        # before any world is built
    world = (world_factory or build_world)(config.world)
    clearance = clearance_evaluator or footprint_clearance
    params = plant_params(config)
    width = config.world.row_width_m
    k = config.corridor_index
    centre_y = k * width
    pose = world.pose_in_field(config.start_x_m, centre_y + config.start_y_m, math.radians(config.start_yaw_deg))
    plant = (plant_factory or field_plant_factory)(params, surface=config.surface, seed=(config.seed, zlib.crc32(
        (config.stream_key or config.name).encode('utf-8')), PURPOSES['plant']), pose=pose, config=config)
    # Truth at the start pose (evaluation only): separates a spawn overlap from a contact made while driving.
    start_hard, start_branch = clearance(world, *pose, params.footprint_length_m,
                                         params.footprint_width_m)
    if config.gyro_bias_deg_s is not None:
        plant.gyro_bias = math.radians(config.gyro_bias_deg_s)
    estimator = estimator_factory(estimator_config(config))
    control = config_for(config.mode, **dict(config.controller_overrides))
    controller = controller_factory(control)
    caster, lidar = (lidar_factory or make_lidar)(world, seed=stream_seed(config, 'lidar_noise'))
    drop_rng = np.random.default_rng(stream_seed(config, 'scan_drop'))
    jitter_rng = np.random.default_rng(stream_seed(config, 'latency'))
    truth = world.corridor_truth(k)
    row_end = truth['end_x']
    walls_ahead = truth['cross_walls_x']
    half_free = width / 2 - params.footprint_width_m / 2
    dt = .01
    steps = int(round(config.max_time_s / dt))
    packets, commands, sequence = [], [], 0
    trace = []
    metrics = dict(lateral=[], heading=[], min_hard=math.inf, min_branch=math.inf,
                   collisions=0, branch_contacts=0, chatter_flips=0, rapid_flips=0, sign=0,
                   sign_time=-math.inf,
                   commands=0, reversal_commands=0, state_time={}, first_collision_s=None,
                   max_lateral_m=0., first_row_end_estimate=None, last_row_end_estimate=None,
                   branch_contacts_moving=0, side_trigger=None)
    last_values = tuple(actuation.initial_values())
    last_scan_arrival = -math.inf
    stopped_since = None
    end_reason = 'time_limit'
    snapshot = None

    def blacked(kind, now):
        return any(t0 <= now < t1 and (kind_ == kind or kind_ == 'both') for t0, t1, kind_ in config.blackouts)

    now = 0.
    for step_index in range(steps):
        now = (step_index + 1) * dt
        while commands and commands[0][0] <= now - dt + 1e-9:
            _, _, due = heapq.heappop(commands)
            actuation.apply(plant, due)
        plant.external_yaw_rate = sum(math.radians(rate) for t0, t1, rate in config.yaw_disturbances
                                      if t0 <= now < t1)
        plant.forward_blocked = any(t0 <= now < t1 for t0, t1 in config.forward_blocks)
        plant.step(dt)
        if (step_index + 1) % 10 == 0:          # LiDAR 10 Hz
            caster.occluded_sectors = tuple((math.radians(lo), math.radians(hi))
                                            for t0, t1, lo, hi in config.occlusions if t0 <= now < t1)
            lx, ly, lyaw = plant.lidar_pose()
            frame = lidar.sample(int(round(now * 1e9)), (lx, ly, .6), yaw_quaternion(lyaw))
            if (frame is not None and not blacked('lidar', now)
                    and drop_rng.random() >= config.scan_drop_probability):
                sequence += 1
                latency = config.lidar_latency_s + (jitter_rng.uniform(0, config.lidar_latency_jitter_s)
                                                    if config.lidar_latency_jitter_s > 0 else 0.)
                # One serial sensor stream: jitter delays scans but never reorders them.
                arrival = max(now + latency, last_scan_arrival)
                last_scan_arrival = arrival
                heapq.heappush(packets, (arrival, sequence, 'scan', now, frame.ranges))
        if (step_index + 1) % 5 == 0:           # raw IMU 20 Hz
            sample = plant.imu_sample()
            if not blacked('imu', now):
                sequence += 1
                heapq.heappush(packets, (now + config.imu_latency_s, sequence, 'imu', now, sample))
        while packets and packets[0][0] <= now + 1e-9:
            _, _, kind, stamp, payload = heapq.heappop(packets)
            if kind == 'scan':
                estimator.on_scan(stamp, payload, lidar.angle_min, lidar.angle_increment, 30.)
            else:
                estimator.on_imu(stamp, payload['gyro_z_rad_s'], payload['accel_m_s2'])
        if (step_index + 1) % 5 == 0:           # controller 20 Hz
            snapshot = estimator.snapshot(now)
            command = controller.step(now, snapshot)
            values = tuple(actuation.values(command))
            # (arrival, push index, values): the index orders equal arrivals without comparing the values.
            heapq.heappush(commands, (now + config.command_latency_s, metrics['commands'], values))
            metrics['commands'] += 1
            if turn_sign is not None:
                sign = turn_sign(values)
                if sign and metrics['sign'] and sign != metrics['sign']:
                    metrics['chatter_flips'] += 1
                    if now - metrics['sign_time'] < 2.:
                        metrics['rapid_flips'] += 1
                if sign:
                    metrics['sign'] = sign
                    metrics['sign_time'] = now
            if actuation.is_reversal(values, last_values):
                metrics['reversal_commands'] += 1
            last_values = values
            metrics['state_time'][controller.state] = metrics['state_time'].get(controller.state, 0.) + .05
            fx, fy, fyaw = world.to_field(plant.x, plant.y, plant.yaw)
            fy -= centre_y
            if snapshot.get('row_end_ahead_m') is not None:     # evaluation only: where it put the end
                estimate = round(fx + snapshot['row_end_ahead_m'] - row_end, 3)
                if metrics['first_row_end_estimate'] is None:
                    metrics['first_row_end_estimate'] = (round(now, 2), estimate)
                metrics['last_row_end_estimate'] = (round(now, 2), estimate)
            hard, branch = clearance(world, plant.x, plant.y, plant.yaw,
                                     params.footprint_length_m, params.footprint_width_m)
            metrics['min_hard'] = min(metrics['min_hard'], hard)
            metrics['min_branch'] = min(metrics['min_branch'], branch)
            if branch <= 0:
                metrics['branch_contacts'] += 1
                if abs(plant.v) > .01 or abs(plant.w) > .01:
                    metrics['branch_contacts_moving'] += 1
            if metrics['side_trigger'] is None and (getattr(command, 'reason', None) == 'side_swing_inhibit'
                                                    or controller.reason == 'side_blocked'):
                # First firing of the opt-in side rule and the truth clearance then (true/false trigger).
                metrics['side_trigger'] = dict(t=round(now, 2), reason=('side_blocked' if controller.reason ==
                                                                         'side_blocked' else command.reason),
                                               branch_m=_finite(branch), hard_m=_finite(hard))
            if 0. <= fx <= row_end:
                metrics['lateral'].append(fy)
                metrics['heading'].append(math.atan2(math.sin(fyaw), math.cos(fyaw)))
                metrics['max_lateral_m'] = max(metrics['max_lateral_m'], abs(fy))
            if record_trace:
                trace.append(dict(t=round(now, 2), fx=round(fx, 3), fy=round(fy, 3),
                                  fyaw_deg=round(math.degrees(fyaw), 2), v=round(plant.v, 3),
                                  **actuation.trace_fields(values), state=controller.state,
                                  reason=controller.reason,
                                  est_heading_deg=None if not snapshot['corridor_valid'] else round(math.degrees(snapshot['heading_error']), 2),
                                  est_lateral=None if not snapshot['corridor_valid'] else round(snapshot['lateral_offset'], 3),
                                  corridor_reason=snapshot['corridor_reason']))
            if hard <= 0:
                metrics['collisions'] += 1
                metrics['first_collision_s'] = now
                end_reason = 'collision'
                break
            if controller.state == 'HOLD' and controller.hold_terminal:
                if abs(plant.v) < .01 and abs(plant.w) < .01:
                    stopped_since = now if stopped_since is None else stopped_since
                    if now - stopped_since >= 1.:
                        end_reason = 'terminal_hold'
                        break
                else:
                    stopped_since = None
            if fx > row_end + config.world.headland_depth_m + 6.:
                end_reason = 'left_field'
                break
    fx, fy, fyaw = world.to_field(plant.x, plant.y, plant.yaw)
    fy -= centre_y
    lateral = np.asarray(metrics['lateral']) if metrics['lateral'] else np.zeros(1)
    heading = np.degrees(np.asarray(metrics['heading'])) if metrics['heading'] else np.zeros(1)
    duration = min(now, config.max_time_s)
    holds = [reason for _, state, reason in controller.events if state == 'HOLD']
    events = controller.events
    # A stop is intermediate when driving or turning resumes after it (the final stop is not).
    intermediate = sum(1 for i, (_, state, _) in enumerate(events) if state == 'STOP_REQUESTED'
                       and any(later in ('FOLLOW', 'REDUCE', 'TURN') for _, later, _ in events[i + 1:]))
    final_reason = controller.reason if controller.state == 'HOLD' else controller.state
    row_end_claimed = final_reason == 'row_end_candidate'
    front_x = fx + params.footprint_length_m / 2 * math.cos(fyaw)
    ahead = [w - front_x for w in walls_ahead if w > fx]
    # Last row (estimator-only claim vs scene truth); judged on the mission's next-row side.
    claims = estimator.next_corridor()
    side = config.next_row_side
    claim = dict(PRESENT='NOT_LAST', ABSENT='LAST', UNKNOWN='UNKNOWN')[claims[side]]
    expected = truth['next_' + side]
    trigger = metrics['side_trigger']
    known = [] if trigger is None else [v for v in (trigger['branch_m'], trigger['hard_m']) if v is not None]
    trigger_truth = min(known) if known else None    # truth clearance when the side rule first fired
    result = dict(
        name=config.name, mode=config.mode, seed=config.seed, scenario=config.world.scenario,
        end=end_reason, final_state=controller.state, final_reason=final_reason,
        time_s=round(duration, 2), progress_m=round(fx - config.start_x_m, 3),
        final_x_rel_row_end_m=round(fx - row_end, 3),
        mean_speed_m_s=round((fx - config.start_x_m) / max(duration, 1e-9), 3),
        lateral_rms_m=round(float(np.sqrt(np.mean(lateral ** 2))), 4),
        lateral_max_m=round(float(metrics['max_lateral_m']), 4),
        left_corridor=bool(metrics['max_lateral_m'] > half_free),
        heading_rms_deg=round(float(np.sqrt(np.mean(heading ** 2))), 3),
        min_clearance_m=round(float(metrics['min_hard']), 3),
        min_branch_clearance_m=round(float(metrics['min_branch']), 3),
        wall_gap_m=round(min(ahead), 3) if ahead else None,
        collisions=metrics['collisions'], branch_contact_samples=metrics['branch_contacts'],
        stops=controller.counts['stops'], intermediate_stops=intermediate, turns=controller.counts['turns'],
        stops_without_turn=controller.counts['stops_without_turn'],
        stall_aborts=controller.counts['stalls'], holds=holds,
        chatter_flips=metrics['chatter_flips'] if turn_sign is not None else None,
        chatter_per_min=(round(metrics['chatter_flips'] / max(duration / 60, 1e-9), 2) if turn_sign is not None
                         else None),
        rapid_flips=metrics['rapid_flips'] if turn_sign is not None else None,
        reversal_commands=metrics['reversal_commands'],
        state_time_s={key: round(v, 2) for key, v in sorted(metrics['state_time'].items())},
        row_end_claimed=row_end_claimed,
        invented_row_end=bool(row_end_claimed and fx < row_end - 2.),
        early_row_end=bool(row_end_claimed and fx < row_end - .5),
        row_end_first_estimate=metrics['first_row_end_estimate'],
        row_end_last_estimate=metrics['last_row_end_estimate'],
        next_corridor_claims=claims, next_corridor_truth=dict(left=truth['next_left'], right=truth['next_right']),
        last_row_claim=claim, last_row_expected=expected,
        plant_pivots=plant.pivot_count, plant_pivot_stalls=plant.pivot_stalls,
        plant_pivot_rates=[r for _, r, _ in plant.pivot_log][:20],
        odom_failures=estimator.odom_failures, estimated_distance_m=round(estimator.distance, 3),
        truth_distance_m=round(plant.distance_m, 3),
        corridor_reason=snapshot['corridor_reason'] if snapshot else None,
        events=controller.events[:60],
        # --- added keys (side-clearance evaluation; truth metrics only) ---
        start_clearance_m=_finite(start_hard), start_branch_clearance_m=_finite(start_branch),
        branch_contact_samples_moving=metrics['branch_contacts_moving'],
        branch_intrusion_increase_m=round(max(0., min(0., start_branch) - metrics['min_branch']), 4),
        side_stops=controller.counts.get('side_stops', 0),
        side_inhibit_ticks=controller.counts.get('side_inhibit', 0),
        side_trigger_truth_m=trigger_truth, side_trigger=trigger,
    )
    if estimator_config(config).row_end_position_confirm_m is not None:
        # Opt-in row-end position bridge only (absent by default): estimator-side outcome counts of every
        # bridge attempt and the first bridges. No truth involved.
        result['row_end_position'] = dict(counts=dict(getattr(estimator, 'end_position_counts', {})),
                                          bridges=list(getattr(estimator, 'end_position_events', [])))
    if estimator_config(config).row_end_continuation_m is not None:
        # Opt-in line-continuation veto only (absent by default): estimator-side counts (ended sides examined,
        # vetoed, two-sided ends of the original rule cancelled) and the first cancellations. No truth involved.
        result['row_end_continuation'] = dict(counts=dict(getattr(estimator, 'continuation_counts', {})),
                                              cancellations=list(getattr(estimator, 'continuation_events', [])))
    if record_trace:
        result['trace'] = trace
    return result
