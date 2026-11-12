"""T1 executor-in-the-loop F-SIM of the front-steer vehicle (design 2.5 T1, 8.3 P4 'sim_in_loop_steer'; SIM only).

What runs (one process, one SIM clock, deterministic for (episode spec, seed) on one platform and build):
  executor   the REAL executor_main.ExecutorLoop + core.Runner with config vehicle_profile 'front_steer' in MOTOR mode
             (single-row mode settings, SIM 'measured' values from the vehicle profile: SIM_ASSUMED, never field values);
             its controller slot holds a SIM scripted controller (sim/ubts/steering/scripted_control.py; the row
             controller is P8)
  client     the REAL bridge_client.SteerBridgeClient whose transport is the T1 rig's control API (t1_rig.RigClient:
             request dict -> reply dict; a queued request advances the rig until the bridge answers, like a socket
             client waiting for the bridge loop)
  below      steering/t1_rig.T1Rig: SteerBridgeCore + SteerControl (dashboard/scripts, injected SIM clock), EspSteerV3 on a
             MemoryTransport, the firmware V3 MCU + C++ actuator / potentiometer model (lockstep v2 firmware mode),
             SteeredPlant (RigPlant), the W0 flat world or a W2 raster corridor (development corridors only) and sensors
Sensor path (the same conversion as sim_in_loop): the rig's SIM 3200-beam scan becomes the real S2 LaserScan through
steering.s2_format.RealFormat (3240 beams, angle_min float32(-pi), increment 0.0019398535 rad, raw order, the
lidar_yaw_offset_deg mount convention, 0 = no return, 1 mm), header = the SIM sensor stamp, receipt = the packet's arrival
time; IMU angular_velocity (0, 0, gyro_z) and linear_acceleration (sensor axes), header = stamp, receipt = arrival.
Clock: the executor steps at 20 Hz on the rig's 10 ms grid (t = 0, 50, 100 ms ...; a late tick is skipped, never caught
up), with the rig clock as its clock (ExecutorLoop clock = rig.clock, wall = rig.wall_clock; SteerBridgeClient too).
Operator emulation (the executor's start mode, config.effective_start_mode):
  hold_to_run  Start (row length optional, single-row mode) at the first tick, Arm at ARM_AT_S, press hold at PRESS_AT_S
               and keep the heartbeat; after a pause (not await_hold) release and press again with a new press id once the
               Runner would accept it (REPRESS_WAIT_S later), at most MAX_REPRESSES; never after an end; never an
               automatic re-arm.
  one_press    (design v2 V2-2, the front_steer default) the operator screen is open from the first tick: one heartbeat of
               SESSION per tick while it is 'visible'; Start at the first tick, Arm at ARM_AT_S, one go at PRESS_AT_S or
               the first tick after it on which the executor's start checks pass (ExecutorLoop._go_blockers: the button
               of the screen is on); after a pause a new go (auto_repress: REPRESS_WAIT_S later and once the checks pass,
               at most MAX_REPRESSES); never after an end.
Episode actions (episode spec 'actions', times in SIM s; the safety scripts use them): stop (executor emergency_stop:
the page STOP), release (the operator lets the hold go: deadman loss), hold_lost (heartbeats stop without a release:
the hold times out), press, end, ack, arm, start, executor_stall (no executor tick for duration_s), client2_arm /
client2_drive / client2_stop (a second screen or the CLI on the same bridge), late_drive (a STOP and, queued right
after it, a drive with the executor's lease: 09-19 'late drive after STOP').
Rig faults: episode spec 'faults' [{kind, at_s, params}] (t1_rig RigFault; model faults at construction). The rig fault
executor_stall (params duration_s, after None or 'f_ttl', delay_s) stops the executor ticks here while the rig says so
(rig.executor_stalled()); with after 'f_ttl' the window starts when an F TTL line reaches the host (+ delay_s).
Truth isolation (A13): the executor and the client see only RigClient replies, the rig clock and the converted sensor
messages; the trace's truth_* columns and the MCU channels are written by the rig / this harness for evaluation only.
Outputs (write()): TRACE.npz (the rig trace with truth_hold_position_rad: the MCU filtered potentiometer position, the
hold_measured STOP target, R13-3; + the ctrl_* executor columns), SERIAL.jsonl (rig serial log), EXECUTOR.jsonl (the
executor's JSONL records), RESULT.json (identity, end, events, timings, SHA-256 of the three files' canonical contents).
Bridge outage (rig fault bridge_restart): the rig's control socket does not answer; RigSocket turns that into the
socket transport's error (TransportError 'socket_error:...'), as UnixSocketTransport does for a refused connection.

CLI (from the work tree root):
  python -B tools/row_executor/sim_in_loop_steer.py episode --spec <spec.json> --out <folder outside the repository>
  python -B tools/row_executor/sim_in_loop_steer.py checks --out <folder> [--only NAME ...] [--workers N]
  (checks: the safety scripts and the A5 / A8 / A11 / A13 / A17 / A19 / A22 checks of sim/ubts/steering/safety_scripts)
Env: UBTS_STEER_BUILD_DIR (native build folder outside the repository), UBTS_STEER_CXX (Windows g++), UBTS_STEER_PATHS (W2).
Every number comes from the placeholder profile: results are placeholder-based SIM results (자리값 기반 SIM 결과), not
field validation.
"""
import argparse
import collections
import hashlib
import json
import math
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
SIM_DIR = os.path.join(REPO, 'sim', 'ubts')
for _path in (SIM_DIR, TOOLS):
    if _path not in sys.path:
        sys.path.insert(0, _path)
sys.dont_write_bytecode = True

import numpy as np  # noqa: E402

from closed_loop import controllers as SC  # noqa: E402  (SIM tree; byte-identical to the vendored copy)
from closed_loop import estimator as SE  # noqa: E402
from row_executor import config as XC  # noqa: E402
from row_executor import core as XCORE  # noqa: E402
from row_executor import executor_main as XM  # noqa: E402
from row_executor.bridge_client import SteerBridgeClient, TransportError  # noqa: E402
from steering import drive as SD  # noqa: E402
from steering import scripted_control as SCRIPT  # noqa: E402
from steering import t1_rig as TR  # noqa: E402
from steering.s2_format import RAW_AMIN, RAW_INC, RealFormat  # noqa: E402
from steering.scene_spec import WorldSpec  # noqa: E402

SCHEMA = 'ubts.steer_sim.executor_in_the_loop.v1'
RESULT_KIND = '자리값 기반 SIM 결과'
TICK_MS = 50                      # executor 20 Hz on the 10 ms rig grid
ARM_AT_S = .5                     # the host bridge is ready (session STOP confirmed) about 0.15 s after the boot
PRESS_AT_S = 1.2
REPRESS_WAIT_S = .3
MAX_REPRESSES = 20
OWNER = 'row-executor:sim'
SIM_SOCKET = '/run/ubts-steer/control.sock'   # the SIM control socket (T1 talks to the rig, the path binds the role)
STATE_CODES = {XCORE.IDLE: 0, XCORE.PAUSED: 1, XCORE.ACTIVE: 2, XCORE.ENDED: 3}
# Estimator columns, written only when the executor config has prior_enabled (the default trace is unchanged): the
# estimator's latency-compensated base pose in its own odometry frame, the time it has run on the actuator prior since
# the last scan-matched pose and the scan-match flag (evaluation of the prediction; never fed back).
EST_COLUMNS = ('ctrl_est_x_m', 'ctrl_est_y_m', 'ctrl_est_yaw_rad', 'ctrl_est_predict_s', 'ctrl_est_odom_valid')
CTRL_COLUMNS = ('ctrl_throttle_pct', 'ctrl_steer_mrad', 'ctrl_state', 'ctrl_hold', 'ctrl_steer_required',
                'ctrl_steer_age_s')
# executor JSONL fields that hold wall-clock (perf_counter) values: dropped from the deterministic log digest
WALL_FIELDS = ('loop_ms', 'proc_ms', 'max_scan_proc_s', 'last_scan_proc_s')
ACTIONS = ('stop', 'release', 'hold_lost', 'press', 'end', 'ack', 'arm', 'start', 'executor_stall', 'client2_arm',
           'client2_drive', 'client2_stop', 'late_drive', 'foreign_arm', 'go', 'screen_hidden', 'screen_visible',
           'other_screen')
SESSION = 'sim-screen-0001'       # one_press: the operator screen's session id
OTHER_SESSION = 'sim-screen-0002'  # one_press: a second screen (action other_screen)
EXECUTOR_EVENTS_KEPT = ('start', 'request', 'emergency_stop', 'bridge_error', 'bridge_stop', 'run_end', 'stop_confirmed',
                        'stop_unconfirmed', 'stop_watch_timeout', 'watchdog_stall')


# --- executor configuration -------------------------------------------------------------------------------------------
def steer_executor_config(rig, overrides=None, base_duty_pct=55):
    """The SIM executor configuration of a rig: vehicle_profile front_steer, single-row mode, MOTOR, and the 'measured'
    values from the rig's vehicle profile and host profile (SIM_ASSUMED placeholders: profile nominal values, never the
    plant's per-run draws). overrides: ExecutorConfig fields on top (validate() must pass)."""
    profile, host = rig.profile, rig.hp
    value = profile.value
    speed = float(SD.steady_speed_m_s(profile, base_duty_pct))
    values = dict(
        vehicle_profile=XC.FRONT_STEER, mode=XC.SINGLE_ROW_MODE, require_native_navigation=False,
        motor_output_enabled=True, base_duty_pct=int(base_duty_pct),
        # the vendored SingleRowConfig (stop model, timing constants) is built in every profile and checks its own
        # base / differential / yaw relations; a front_steer run never commands this differential
        max_yaw_rate_deg_s=3.6,
        # drive side (C2 / C3 / C7): SIM-assumed, inside the stop model (0.4 m/s -> model 0.235 m)
        breakaway_duty_pct=20, v_measured_m_s=round(min(speed, .5), 3), max_speed_m_s=.55,
        measured_stop_distance_m=.2,
        # steering (B0-B5, F1): the profile's placeholders
        steer_limit_left_mrad=int(host.limit_left_mrad), steer_limit_right_mrad=int(host.limit_right_mrad),
        steer_rate_limit_deg_s=round(XC.STEER_RATE_FRACTION * float(value('steer_rate_max_deg_s')), 3),
        steer_rate_limit_standstill_deg_s=round(XC.STEER_RATE_FRACTION * float(value('steer_rate_standstill_deg_s')),
                                                3),
        steer_measured_rate_deg_s=float(value('steer_rate_max_deg_s')),
        steer_measured_rate_standstill_deg_s=float(value('steer_rate_standstill_deg_s')),
        steer_range_measured=True, steer_wheel_soft_margin_deg=float(value('steer_soft_margin_deg')),
        steer_min_pwm_pct=float(value('steer_min_pwm_pct')), steer_zero_offset_deg=0.,
        steer_servo_deadband_deg=float(value('servo_deadband_deg')), steer_polarity_checked=True,
        steer_cal_id=int(rig.config.cal_id), steer_allow_sim=True, socket_path=SIM_SOCKET,
        steer_rotation_centre_x_m=float(value('rear_axle_x_in_base_m')),
        lidar_yaw_offset_deg=float(value('lidar_yaw_offset_deg')), lidar_x_in_base_m=float(value('lidar_x_in_base_m')),
        footprint_length_m=float(value('footprint_length_m')), footprint_width_m=float(value('footprint_width_m')))
    values.update(overrides or {})
    if values.get('prior_enabled'):
        # actuator priors (design v2 V2-1): the SIM 'measured' duty -> speed table and wheelbase are the profile's
        # nominal values (SIM_ASSUMED placeholders, never the per-run speed draw), unless the overrides give them
        values.setdefault('prior_wheelbase_m', float(value('wheelbase_m')))
        if not values.get('prior_speed_table'):
            values['prior_speed_table'] = profile_speed_table(profile)
    from dataclasses import replace
    return replace(XC.ExecutorConfig(), **values).validate()


def profile_speed_table(profile, duties=(10, 20, 30, 40, 50, 60, 70, 80, 90, 100), factor=1.):
    """The prior's (duty %, speed m/s) table from the profile's nominal drive curve (steering.drive.steady_speed_m_s:
    profile values, not a run's draw) x factor. Points with no speed (inside the deadband) are left out."""
    points = []
    for duty in duties:
        speed = round(float(SD.steady_speed_m_s(profile, duty)) * float(factor), 4)
        if speed > 0. and (not points or speed >= points[-1][1]):
            points.append((float(duty), speed))
    return tuple(points)


# --- log capture ------------------------------------------------------------------------------------------------------
class CaptureLog:
    """The executor's JSONL log kept in memory (no disk): put() keeps every record; canonical_bytes() is the JSON line
    form without the wall-clock fields (WALL_FIELDS), the A8 digest."""

    dropped = 0
    ticks_suppressed = 0
    paths = ()

    def __init__(self):
        self.records = []

    def put(self, record, tick=False):
        self.records.append(record)

    def health(self):
        return dict(state='ok', dropped=0, write_errors=0, last_error=None, failing_since=None, recoveries=0,
                    pending=0, lines=len(self.records), bytes=0, ticks_suppressed=0, writer_alive=True, file=None)

    def close(self, timeout=0.):
        pass

    def events(self, names=None):
        return [r for r in self.records if r.get('k') == 'event' and (names is None or r.get('event') in names)]

    @staticmethod
    def _strip(value):
        if isinstance(value, dict):
            return {k: CaptureLog._strip(v) for k, v in value.items() if k not in WALL_FIELDS}
        if isinstance(value, list):
            return [CaptureLog._strip(v) for v in value]
        return value

    def canonical_bytes(self):
        lines = [json.dumps(self._strip(r), sort_keys=True, separators=(',', ':'), allow_nan=False, default=str)
                 for r in self.records]
        return ('\n'.join(lines) + '\n').encode('utf-8')

    def jsonl_bytes(self):
        lines = [json.dumps(r, sort_keys=True, separators=(',', ':'), allow_nan=False, default=str) for r in self.records]
        return ('\n'.join(lines) + '\n').encode('utf-8')


def _ns(**kw):
    return types.SimpleNamespace(**kw)


def _header(t):
    sec = math.floor(t)
    return _ns(stamp=_ns(sec=int(sec), nanosec=int(round((t - sec) * 1e9))), frame_id='laser')


def scan_message(packet, real_format):
    """A rig scan packet -> the real S2 LaserScan fields (RealFormat; frame laser, raw order, 0 = no return)."""
    return _ns(header=_header(packet['stamp_s']), ranges=real_format.convert(packet['ranges']), angle_min=RAW_AMIN,
               angle_increment=RAW_INC, range_min=.05, range_max=30., scan_time=.1, time_increment=0.)


def imu_message(packet):
    """A rig IMU packet -> sensor_msgs/Imu-like fields (the raw IMU axes; the executor projects the gyro itself)."""
    acc = packet['accel_m_s2']
    header = _header(packet['stamp_s'])
    header.frame_id = 'imu_link'
    return _ns(header=header, angular_velocity=_ns(x=0., y=0., z=float(packet['gyro_z_rad_s'])),
               linear_acceleration=_ns(x=float(acc[0]), y=float(acc[1]), z=float(acc[2])))


class RigSocket:
    """The executor's transport in T1: the RigClient's requests; a request while the rig has no bridge process
    (bridge_restart) raises the socket transport's TransportError, like UnixSocketTransport on a refused connection.
    on_motion(t, yaw_rate_rad_s, speed_m_s): called with every op motion sample the bridge accepted (evaluation of the
    host steering_ineffective time; the executor never sees it)."""

    def __init__(self, client, on_motion=None):
        self._client = client
        self._on_motion = on_motion

    def __call__(self, request):
        try:
            reply = self._client(request)
        except TR.BridgeUnavailable as exc:
            raise TransportError('socket_error:%s' % exc) from None
        if (self._on_motion is not None and isinstance(request, dict) and request.get('op') == 'motion'
                and isinstance(reply, dict) and reply.get('ok') is True):
            self._on_motion(request.get('yaw_rate_rad_s'), request.get('speed_m_s'))
        return reply


# --- the facade ------------------------------------------------------------------------------------------------------
class SteerExecutorInTheLoop:
    """One episode: the executor on a T1 rig (module docstring). run(duration_s) advances; summary() / write()."""

    def __init__(self, rig, cfg, controller_spec, actions=(), auto_repress=True, capture_inputs=0, timing=True,
                 operator=True):
        self.rig = rig
        self.cfg = cfg
        self.controller_spec = dict(controller_spec)
        self.client = rig.client(OWNER)
        self.motion_log = []             # (t, yaw rate, speed) of the op motion samples the bridge took (evaluation)
        self.socket = RigSocket(self.client, on_motion=lambda yaw, speed: self.motion_log.append(
            (round(rig.now, 4), float(yaw), float(speed))))
        self.log = CaptureLog()
        SCRIPT.make_factory(self.controller_spec, cfg)           # refuses a bad spec before anything starts
        # the controller factory gets the loop's effective config (the field tuning values the loop loaded, PW-08)
        self.executor = XM.ExecutorLoop(
            cfg, motor_output_flag=True, clock=rig.clock, wall=rig.wall_clock, log=self.log,
            bridge_factory=lambda: SteerBridgeClient.from_config(cfg, transport=self.socket, owner=OWNER,
                                                                 clock=rig.clock, wall=rig.wall_clock),
            stop_transport=self.socket,
            runner_factory=lambda c: XCORE.Runner(c, vendor_modules=(SE, SC),
                                                  controller_factory=SCRIPT.make_factory(self.controller_spec, c)))
        self.cfg = cfg = self.executor.config
        self.real_format = RealFormat(cfg.lidar_yaw_offset_deg)
        self.actions = sorted((dict(a) for a in actions), key=lambda a: (float(a['at_s']), a['op']))
        for action in self.actions:
            if action['op'] not in ACTIONS:
                raise ValueError('unknown action %r (known: %s)' % (action['op'], ', '.join(ACTIONS)))
        self._next_action = 0
        self.auto_repress = bool(auto_repress)
        self.operator = bool(operator)
        self.capture_inputs = int(capture_inputs)
        self.inputs = dict(scans=[], imus=[])
        self.input_counts = collections.Counter()
        self.phase, self.press, self.presses, self.represses = 'start', None, 0, 0
        self.one_press = XC.effective_start_mode(cfg) == XC.ONE_PRESS
        self.go_refused = []             # one_press: refused gos (t, error)
        self.other_screen = []           # one_press: replies to the second screen (action other_screen)
        self.other_future = None
        self.repress_at = None
        self.future = None
        self.start_error = None
        self.press_ignored = 0
        self.beating = True
        self.stall_until_ms = None
        self.rig_stalled_ticks = 0       # ticks skipped by the rig fault executor_stall
        self.next_tick_ms = 0
        self.ticks = 0
        self.skipped_ticks = 0
        self.decision = None
        self.events = []                 # (t, kind, detail) of state changes of the executor
        self._state = None
        self.ended_at = None
        self.first_active_at = None
        self.client2 = None
        self.client2_replies = []
        self.late_replies = []
        self.latch_log = []              # host bridge latches as they appear: (name, since s, detail) (evaluation)
        self._latch_seen = set()
        self.timing = bool(timing)
        self.tick_wall_ms, self.step_wall_ms = [], []
        self._inner_s = 0.
        if self.timing:
            self._wrap_rig_timers()
        self.mcu_step_ms = []
        rig.add_trace_extras(CTRL_COLUMNS, self._ctrl_row)
        self.est_columns = bool(cfg.prior_enabled)
        if self.est_columns:
            rig.add_trace_extras(EST_COLUMNS, self._est_row)

    # -- timing (A11): the rig's work inside an executor tick is subtracted from the executor step latency
    def _wrap_rig_timers(self):
        rig = self.rig
        bridge_step, mcu_step = rig._bridge_step, rig._mcu_and_plant_step
        fw = rig.fw
        fw_step = fw.step

        def timed(fn):
            def inner(*args, **kwargs):
                t0 = time.perf_counter()
                try:
                    return fn(*args, **kwargs)
                finally:
                    self._inner_s += time.perf_counter() - t0
            return inner

        def timed_fw(*args, **kwargs):
            t0 = time.perf_counter()
            try:
                return fw_step(*args, **kwargs)
            finally:
                self.mcu_step_ms.append((time.perf_counter() - t0) * 1e3)

        rig._bridge_step = timed(bridge_step)
        rig._mcu_and_plant_step = timed(mcu_step)
        fw.step = timed_fw

    # -- trace columns of the executor (written by the rig at every trace row)
    def _ctrl_row(self):
        d = self.decision
        runner = self.executor.runner
        monitor = getattr(runner, 'steer_monitor', None)
        age = monitor.age(self.rig.now) if monitor is not None else math.inf
        if d is None:
            return (0., float('nan'), float(STATE_CODES.get(runner.state, -1)), 0., 0., float('nan'))
        steer = float('nan') if d.steer_mrad is None else float(d.steer_mrad)
        if self.one_press:
            link = self.executor.link
            level = link.go_active and link.last_beat is not None and self.rig.clock() - link.last_beat <= link.timeout_s
        else:
            level = self.executor.hold.active(self.rig.clock())
        return (float(d.throttle or 0), steer, float(STATE_CODES.get(d.state, -1)),
                float(bool(level)), float(bool(runner.steer_required)),
                age if math.isfinite(age) else float('nan'))

    def _est_row(self):
        """EST_COLUMNS of this trace row (NaN before the first scan of a run)."""
        est = self.executor.runner.estimator
        if est is None or est.last_scan_stamp is None:
            return (float('nan'),) * 5
        now = self.rig.now
        x, y, yaw = est.base_pose(now)
        block = est.prior.snapshot(now, est.last_scan_stamp, est.speed) if est.prior is not None else None
        return (x, y, yaw, float('nan') if block is None else float(block['predict_s']), float(bool(est.odom_valid)))

    # -- sensors -> executor
    def _feed_sensors(self):
        for packet in self.rig.poll_sensors():
            receipt = float(packet['arrival_s'])
            if packet['kind'] == 'scan':
                msg = scan_message(packet, self.real_format)
                self.input_counts['scans'] += 1
                if len(self.inputs['scans']) < self.capture_inputs:
                    self.inputs['scans'].append(dict(receipt=receipt, header=packet['stamp_s'], msg=msg))
                self.executor.on_scan_msg(receipt, msg)
            else:
                msg = imu_message(packet)
                self.input_counts['imus'] += 1
                if len(self.inputs['imus']) < self.capture_inputs:
                    self.inputs['imus'].append(dict(receipt=receipt, header=packet['stamp_s'],
                                                    gyro_z=float(packet['gyro_z_rad_s']),
                                                    accel=list(packet['accel_m_s2'])))
                self.executor.on_imu_msg(receipt, msg)

    # -- operator
    def _new_press(self):
        self.presses += 1
        if self.one_press:
            self.future, self.phase = self.executor.submit('go', session=SESSION), 'going'
            return
        self.press = 'sim-press-%04d' % self.presses
        self.executor.hold_beat(self.press)
        self.phase = 'holding'

    def _resume_ready(self, now):
        runner = self.executor.runner
        if runner.state != XCORE.PAUSED:
            return False
        fresh_since = runner._fresh_since
        if fresh_since is None or now - fresh_since < self.cfg.fresh_recover_s + .05:
            return False
        if not runner._steer_ok(now):
            return False
        if runner._press_needs_standstill:
            s = runner.last_snapshot
            return s is not None and runner._standstill(s)
        return True

    def _go_ready(self, now):
        return not self.executor._go_blockers(now, SESSION)

    def _operator(self, now):
        executor, runner = self.executor, self.executor.runner
        if not self.operator:
            return
        if self.one_press and self.beating:
            executor.operator_heartbeat(SESSION)          # the screen is visible: one heartbeat per tick
        if self.one_press and self.phase in ('armed', 'going', 'holding', 'released'):
            self._operator_one_press(now)
            return
        if self.phase == 'start':
            self.future = executor.submit('start', row_len_m=None)
            self.phase = 'started'
        elif self.phase == 'started' and self.future.done():
            result = self.future.result()
            if not result.get('ok'):
                self.start_error, self.phase = result, 'failed'
            else:
                self.phase = 'wait_arm'
        elif self.phase == 'wait_arm' and now >= ARM_AT_S - 1e-9:
            self.future, self.phase = executor.submit('arm'), 'arming'
        elif self.phase == 'arming' and self.future.done():
            result = self.future.result()
            if not result.get('ok'):
                self.start_error, self.phase = result, 'failed'
            else:
                self.phase = 'armed'
        elif self.phase == 'armed' and now >= PRESS_AT_S - 1e-9:
            self._new_press()
        elif self.phase == 'holding':
            if runner.state == XCORE.ENDED:
                executor.hold_release(self.press)
                self.phase = 'done'
            elif runner.state == XCORE.PAUSED and runner.reason != 'await_hold':
                if str(runner.reason).startswith('press_ignored'):
                    self.press_ignored += 1
                executor.hold_release(self.press)
                self.represses += 1
                allowed = self.auto_repress and self.represses <= MAX_REPRESSES
                self.phase = 'released' if allowed else 'gave_up'
                self.repress_at = now + REPRESS_WAIT_S
            elif self.beating:
                executor.hold_beat(self.press)
        elif self.phase == 'released' and runner.state == XCORE.ENDED:
            self.phase = 'done'
        elif self.phase == 'released' and now >= self.repress_at and self._resume_ready(now):
            self._new_press()

    def _operator_one_press(self, now):
        executor, runner = self.executor, self.executor.runner
        if self.phase == 'armed' and now >= PRESS_AT_S - 1e-9 and self._go_ready(now):
            self._new_press()
        elif self.phase == 'going' and self.future.done():
            result = self.future.result()
            if result.get('ok'):
                self.phase = 'holding'
            else:
                self.go_refused.append((round(now, 3), result.get('error')))
                self.phase = 'armed' if runner.state == XCORE.PAUSED else 'done'
        elif self.phase == 'holding':
            if runner.state == XCORE.ENDED:
                self.phase = 'done'
            elif runner.state == XCORE.PAUSED and not executor.link.go_active:
                if str(runner.reason).startswith('press_ignored'):
                    self.press_ignored += 1
                self.represses += 1
                allowed = self.auto_repress and self.represses <= MAX_REPRESSES
                self.phase = 'released' if allowed else 'gave_up'
                self.repress_at = now + REPRESS_WAIT_S
        elif self.phase == 'released':
            if runner.state == XCORE.ENDED:
                self.phase = 'done'
            elif now >= self.repress_at and self._go_ready(now):
                self._new_press()

    def _action(self, action, now):
        op = action['op']
        executor = self.executor
        self.events.append((round(now, 3), 'action', op))
        if op == 'stop':
            executor.emergency_stop(action.get('source', 'page'))
        elif op == 'release':
            if self.one_press:
                self.beating = False                  # one_press has no release: the screen stops its heartbeats
            elif self.press is not None:
                executor.hold_release(self.press)
            self.phase = 'operator_released'
        elif op in ('hold_lost', 'screen_hidden'):
            self.beating = False
        elif op == 'screen_visible':
            self.beating = True
        elif op in ('press', 'go'):
            self.beating = True
            self._new_press()
        elif op == 'other_screen':
            # one_press: a second screen opens: a heartbeat and a go of another session (refused while the first is fresh)
            beat = executor.operator_heartbeat(OTHER_SESSION)
            self.other_screen.append(dict(t=round(now, 3), op='heartbeat', ok=bool(beat[0]), error=beat[1]))
            self.other_future = executor.submit('go', session=OTHER_SESSION)
        elif op == 'end':
            executor.submit('end')
        elif op == 'ack':
            executor.submit('ack')
        elif op == 'arm':
            self.future, self.phase = executor.submit('arm'), 'arming'
        elif op == 'start':
            self.future, self.phase = executor.submit('start', row_len_m=None), 'started'
        elif op == 'executor_stall':
            self.stall_until_ms = self.rig.ms + int(round(float(action.get('duration_s', 1.)) * 1000.))
        elif op in ('client2_arm', 'client2_drive', 'client2_stop'):
            if self.client2 is None:
                self.client2 = self.rig.client(action.get('owner', 'dashboard-screen-2'))
            client = self.client2
            if op == 'client2_arm':
                reply = client.arm(precheck=False)
            elif op == 'client2_stop':
                reply = client.stop('brake', reason=action.get('reason', 'cli_stop'))
            else:
                reply = client.request({'op': 'drive', 'owner': client.owner, 'lease': 'not-a-lease', 'ticket': 'x',
                                        'seq': 1, 'deadman': True, 'throttle': int(action.get('throttle', 30)),
                                        'steer_mrad': int(action.get('steer_mrad', 0))})
            self.client2_replies.append(dict(t=round(now, 3), op=op, ok=bool(reply.get('ok')),
                                             error=reply.get('error')))
        elif op == 'foreign_arm':
            # another writer on the serial line (a CLI) arms the MCU with the current boot id (the rig's host link knows
            # it from the banner): the MCU is ARMED without a host lease
            self.rig.fault('foreign_write', text='ARM ' + str(self.rig.link.boot_id), crc=True)
        elif op == 'late_drive':
            bridge = executor.bridge
            lease = getattr(bridge, 'lease', None)
            ticket, seq = getattr(bridge, 'ticket', None), getattr(bridge, 'seq', 0)
            self.rig.submit({'op': 'stop', 'reason': 'late_drive_after_stop'})
            late = {'op': 'drive', 'owner': OWNER, 'lease': lease, 'ticket': ticket, 'seq': seq + 1, 'deadman': True,
                    'throttle': int(action.get('throttle', 40)), 'steer_mrad': int(action.get('steer_mrad', 0))}
            self.late_replies.append(dict(t=round(now, 3), future=self.rig.submit(late)))

    # -- one executor tick
    def tick(self):
        rig = self.rig
        now = rig.now
        while self._next_action < len(self.actions) and float(self.actions[self._next_action]['at_s']) <= now + 1e-9:
            self._action(self.actions[self._next_action], now)
            self._next_action += 1
        if self.stall_until_ms is not None and rig.ms < self.stall_until_ms:
            self.skipped_ticks += 1
            return None
        self.stall_until_ms = None
        if rig.executor_stalled():
            # rig fault executor_stall (e.g. right after an F TTL line: the window of DECISIONS R18 decision 1)
            self.skipped_ticks += 1
            self.rig_stalled_ticks += 1
            return None
        t0 = time.perf_counter()
        inner0 = self._inner_s
        self._feed_sensors()
        self._operator(now)
        s0 = time.perf_counter()
        self.decision = self.executor.step(now)
        if self.other_future is not None and self.other_future.done():
            reply = self.other_future.result()
            self.other_screen.append(dict(t=round(now, 3), op='go', ok=bool(reply.get('ok')), error=reply.get('error')))
            self.other_future = None
        s1 = time.perf_counter()
        self.ticks += 1
        wall = (time.perf_counter() - t0) * 1e3
        inner = (self._inner_s - inner0) * 1e3
        self.tick_wall_ms.append(wall)
        self.step_wall_ms.append(max(0., (s1 - s0) * 1e3 - inner))
        self._note_state(now)
        self._note_latches()
        return self.decision

    def _note_latches(self):
        """Host bridge latches (evaluation record; the executor sees them only in the steering status)."""
        for name, entry in self.rig.core.latches.items():
            key = (name, entry.get('since'))
            if key not in self._latch_seen:
                self._latch_seen.add(key)
                self.latch_log.append((name, None if entry.get('since') is None else round(float(entry['since']), 4),
                                       entry.get('detail')))

    def _note_state(self, now):
        runner = self.executor.runner
        key = (runner.state, runner.reason)
        if key != self._state:
            self._state = key
            self.events.append((round(now, 3), runner.state, runner.reason))
        if runner.state == XCORE.ACTIVE and self.first_active_at is None:
            self.first_active_at = now
        if runner.state == XCORE.ENDED and self.ended_at is None:
            self.ended_at = now

    def run(self, duration_s, post_end_s=None):
        """Advance to SIM time duration_s (from the rig start); with post_end_s, stop that long after the run ended."""
        rig = self.rig
        end_ms = int(round(float(duration_s) * 1000.))
        while rig.ms < end_ms:
            if self.ended_at is not None and post_end_s is not None and rig.now >= self.ended_at + post_end_s - 1e-9:
                break
            if rig.ms >= self.next_tick_ms:
                while self.next_tick_ms + TICK_MS <= rig.ms:
                    self.next_tick_ms += TICK_MS          # a late tick: the missed ones are skipped, never caught up
                    self.skipped_ticks += 1
                if rig.ms == self.next_tick_ms:
                    self.tick()
                    self.next_tick_ms += TICK_MS
                    continue
                self.next_tick_ms += TICK_MS
                continue
            rig.step(TR.PLANT_DT_S)
        self._note_latches()
        for item in self.late_replies:
            future = item.pop('future', None)
            if future is not None:
                item['reply'] = future.result() if future.done() else {'ok': False, 'error': 'not_answered'}
        return self

    # -- results
    def summary(self):
        runner, executor = self.executor.runner, self.executor
        bridge = executor.bridge

        def pct(values, q):
            return None if not values else round(float(np.percentile(np.asarray(values), q)), 4)

        return dict(
            end_reason=runner.end_reason, end_note=runner.end_note, runner_state=runner.state, runner_reason=runner.reason,
            ended_at_s=None if self.ended_at is None else round(self.ended_at, 3),
            first_active_at_s=None if self.first_active_at is None else round(self.first_active_at, 3),
            pauses=runner.pauses, resumes=runner.resumes, presses=self.presses, represses=self.represses,
            press_ignored=self.press_ignored, start_error=self.start_error, guard_violations=runner.guard.violations,
            bridge_state=None if bridge is None else bridge.state,
            bridge_latch=None if bridge is None else bridge.latch_reason,
            bridge_counts=None if bridge is None else dict(bridge.counts), executor_counts=dict(executor.counts),
            steer_feedback_fed=None if runner.steer_monitor is None else runner.steer_monitor.fed,
            ticks=self.ticks, skipped_ticks=self.skipped_ticks, inputs=dict(self.input_counts),
            executor_stall=dict(rig_stalled_ticks=self.rig_stalled_ticks,
                                windows_ms=[list(w) for w in getattr(self.rig, 'executor_stall_windows', ())]),
            events=[list(e) for e in self.events], client2=self.client2_replies,
            start_mode=XC.effective_start_mode(self.cfg), go_refused=[list(g) for g in self.go_refused],
            tuning=None if self.executor.tuning is None else self.executor.tuning.brief(),
            other_screen=list(self.other_screen),
            host_latches=[list(item) for item in self.latch_log],
            late_drive=[dict(t=item['t'], reply=dict((k, item.get('reply', {}).get(k)) for k in ('ok', 'error')))
                        for item in self.late_replies],
            executor_events=[dict((k, v) for k, v in r.items() if k not in ('wall',))
                             for r in self.log.events(EXECUTOR_EVENTS_KEPT)],
            timing=dict(step_ms=dict(p50=pct(self.step_wall_ms, 50), p99=pct(self.step_wall_ms, 99),
                                     p999=pct(self.step_wall_ms, 99.9), max=pct(self.step_wall_ms, 100), n=len(self.step_wall_ms)),
                        tick_ms=dict(p50=pct(self.tick_wall_ms, 50), p99=pct(self.tick_wall_ms, 99),
                                     p999=pct(self.tick_wall_ms, 99.9), n=len(self.tick_wall_ms)),
                        mcu_step_ms=dict(p50=pct(self.mcu_step_ms, 50), p99=pct(self.mcu_step_ms, 99),
                                         p999=pct(self.mcu_step_ms, 99.9), mean=None if not self.mcu_step_ms
                                         else round(float(np.mean(self.mcu_step_ms)), 4), n=len(self.mcu_step_ms)),
                        note='wall-clock values (perf_counter): step_ms = ExecutorLoop.step without the rig work done '
                             'inside it (bridge and MCU / plant steps of a blocking request); tick_ms includes the sensor '
                             'conversion and that rig work; mcu_step_ms = one FW_STEP (10 MCU ticks) of the native '
                             'firmware process incl. the pipe round trip. Not part of any digest.'))

    def trace_arrays(self):
        """The rig trace (with truth_hold_position_rad: the MCU filtered potentiometer position by the host calibration
        table, the hold_measured STOP target; R13-3, written by the rig) + the ctrl_* executor columns + meta."""
        arrays = self.rig.trace_arrays()
        meta = json.loads(bytes(arrays['meta_json']).decode('utf-8'))
        meta.update(schema_executor=SCHEMA,
                    executor_columns=list(CTRL_COLUMNS) + (list(EST_COLUMNS) if self.est_columns else []),
                    ctrl_state_codes={k: v for k, v in STATE_CODES.items()})
        arrays['meta_json'] = np.frombuffer(json.dumps(meta, sort_keys=True, separators=(',', ':'), ensure_ascii=True,
                                                       allow_nan=False).encode('utf-8'), dtype=np.uint8).copy()
        return arrays

    def digests(self, arrays=None):
        arrays = self.trace_arrays() if arrays is None else arrays
        return dict(trace_content_sha256=TR.content_sha256({k: v for k, v in arrays.items() if k != 'meta_json'}),
                    serial_log_sha256=self.rig.serial_log_sha256(),
                    executor_log_sha256=hashlib.sha256(self.log.canonical_bytes()).hexdigest())

    def write(self, out_dir, spec=None):
        os.makedirs(out_dir, exist_ok=True)
        arrays = self.trace_arrays()
        TR.write_npz(os.path.join(out_dir, 'TRACE.npz'), arrays)
        self.rig.write_serial_log(os.path.join(out_dir, 'SERIAL.jsonl'))
        with open(os.path.join(out_dir, 'EXECUTOR.jsonl'), 'wb') as fh:
            fh.write(self.log.jsonl_bytes())
        result = self.result(spec, arrays)
        _write_json(os.path.join(out_dir, 'RESULT.json'), result)
        return result

    def result(self, spec=None, arrays=None):
        return dict(schema=SCHEMA, result_kind=RESULT_KIND, field_validated=False, spec=spec,
                    rig=self.rig.identity(include_files=True), config_digest=self.cfg.digest(),
                    controller_spec_sha256=SCRIPT.spec_digest(self.controller_spec),
                    summary=self.summary(), digests=self.digests(arrays))


# --- episodes -----------------------------------------------------------------------------------------------------------
EPISODE_KEYS = ('name', 'seed', 'world', 'start_pose', 'start_steer_deg', 'controller', 'faults', 'actions', 'rig',
                'config', 'duration_s', 'post_end_s', 'auto_repress', 'capture_inputs', 'operator', 'stream_key')


def normalise_spec(spec):
    unknown = sorted(set(spec) - set(EPISODE_KEYS))
    if unknown:
        raise ValueError('unknown episode keys: %s' % ', '.join(unknown))
    out = dict(name=str(spec.get('name', 'episode')), seed=int(spec.get('seed', 0)),
               world=dict(spec.get('world') or {'kind': 'W0'}), start_pose=list(spec.get('start_pose', (0., 0., 0.))),
               start_steer_deg=float(spec.get('start_steer_deg', 0.)),
               controller=dict(spec.get('controller') or {'kind': 'hold_still', 'duration_s': 5.}),
               faults=[dict(f) for f in spec.get('faults') or ()], actions=[dict(a) for a in spec.get('actions') or ()],
               rig=dict(spec.get('rig') or {}), config=dict(spec.get('config') or {}),
               duration_s=float(spec.get('duration_s', 10.)), post_end_s=spec.get('post_end_s'),
               auto_repress=bool(spec.get('auto_repress', True)), capture_inputs=int(spec.get('capture_inputs', 0)),
               operator=bool(spec.get('operator', True)), stream_key=spec.get('stream_key'))
    return out


def build_rig(spec, native_executable=None, build_dir=None):
    world = WorldSpec.from_dict(spec['world'])
    x, y, yaw_deg = (float(v) for v in spec['start_pose'])
    faults = tuple(TR.RigFault(f['kind'], float(f.get('at_s', 0.)), dict(f.get('params') or {})) for f in spec['faults'])
    changes = dict(spec['rig'])
    for key in ('record_overrides', 'host_overrides'):
        if key in changes and isinstance(changes[key], dict):
            changes[key] = tuple(sorted(changes[key].items()))
    config = TR.RigConfig(seed=spec['seed'], stream_key=spec.get('stream_key') or 'steer_exec_' + spec['name'],
                          world=world, start_pose=(x, y, math.radians(yaw_deg)),
                          start_steer_rad=math.radians(spec['start_steer_deg']), faults=faults, **changes)
    return TR.T1Rig(config, native_executable=native_executable, build_dir=build_dir)


def run_episode(spec, out_dir=None, native_executable=None, build_dir=None, keep=False):
    """Run one episode spec (normalise_spec keys). Returns the result dict (and the facade with keep=True; the caller
    then closes facade.rig)."""
    spec = normalise_spec(spec)
    started = time.perf_counter()
    rig = build_rig(spec, native_executable, build_dir)
    try:
        cfg = steer_executor_config(rig, spec['config'])
        facade = SteerExecutorInTheLoop(rig, cfg, spec['controller'], spec['actions'], spec['auto_repress'],
                                        spec['capture_inputs'], operator=spec['operator'])
        facade.run(spec['duration_s'], spec['post_end_s'])
        result = facade.write(out_dir, spec) if out_dir else facade.result(spec)
        result['wall_s'] = round(time.perf_counter() - started, 3)
    finally:
        if not keep:
            rig.close()
    return (result, facade) if keep else result


def _job(item):
    spec, out_dir, native_executable, build_dir = item
    return run_episode(spec, out_dir, native_executable, build_dir)


def run_episodes(specs, out_root=None, workers=1, native_executable=None, build_dir=None):
    """Run episode specs (one rig each); workers > 1 uses a process pool. Results in the order of specs."""
    items = [(spec, None if out_root is None else os.path.join(out_root, normalise_spec(spec)['name']),
              native_executable, build_dir) for spec in specs]
    if workers <= 1:
        return [_job(item) for item in items]
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(_job, items))


def _write_json(path, value):
    raw = json.dumps(value, sort_keys=True, indent=1, ensure_ascii=False, allow_nan=False, default=str) + '\n'
    tmp = path + '.tmp%d' % os.getpid()
    with open(tmp, 'wb') as fh:
        fh.write(raw.encode('utf-8'))
    os.replace(tmp, path)


def _outside_repository(path):
    from steering.scene_spec import inside_repository
    if inside_repository(path):
        raise SystemExit('refused: %s lies inside the repository (runs live outside)' % path)
    return path


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = p.add_subparsers(dest='command', required=True)
    one = sub.add_parser('episode', help='run one episode spec (JSON)')
    one.add_argument('--spec', required=True)
    one.add_argument('--out', required=True)
    checks = sub.add_parser('checks', help='safety scripts and the P4 acceptance checks')
    checks.add_argument('--out', required=True)
    checks.add_argument('--only', nargs='*')
    checks.add_argument('--workers', type=int, default=1)
    checks.add_argument('--repeat', type=int, default=None, help='A19 rate seeds (default: the catalogue value)')
    args = p.parse_args(argv)
    out = _outside_repository(os.path.abspath(args.out))
    if args.command == 'episode':
        with open(args.spec, 'r', encoding='utf-8') as fh:
            spec = json.load(fh)
        result = run_episode(spec, out)
        print(json.dumps(dict(end=result['summary']['end_reason'], digests=result['digests'], wall_s=result['wall_s'])))
        return 0
    from steering.safety_scripts import acceptance
    report = acceptance.run_all(out, only=args.only, workers=args.workers, repeat=args.repeat)
    print(json.dumps(report['verdicts'], indent=1, sort_keys=True))
    return 0 if report['all_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
