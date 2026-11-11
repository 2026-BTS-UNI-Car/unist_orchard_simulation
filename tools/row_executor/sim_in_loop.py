"""SIM-in-the-loop CLOSED-LOOP test of the executor code path (SIM only; no hardware, no ROS, no Docker).

What runs (per episode, deterministic for (scenario, seed)):
  sim/ubts/closed_loop/harness.run_episode (unchanged: 100 Hz FieldPlant, 10 Hz SIM LiDAR, 20 Hz IMU, scene truth
  only for sensor synthesis and scoring) with its estimator/controller slots filled by ONE facade around the REAL
  executor_main.ExecutorLoop:
      SIM scan --(real S2 format)--> ExecutorLoop.on_scan_msg (receipt = harness delivery time, header = SIM stamp)
      SIM IMU  --(Imu fields)------> ExecutorLoop.on_imu_msg
      ExecutorLoop.step(now) at the harness' 20 Hz: ScanStamper (header_offset), newest scan only, ScanAdapter
      (+179.08 deg, 0 -> no return), NearConfirmFilter, vendored estimator/controller (via Runner), geometry guard,
      Backstop, OutputGuard, hold rule -> BridgeClient (deadman:true, 10 Hz + immediate zero) -> the dashboard's
      REAL RoverControl (scripts/rover_control_core.py, read-only checkout, in-process, fake motor) -> the duty
      the fake motor holds is what the plant receives (after the harness' command latency, then the MCU ramp).
  Real S2 format: 3240 beams, angle_min float32(-pi), increment 0.0019398535 rad; real beam j looks at body angle
  angle_min + j*inc + 179.08 deg (the mount estimate the executor undoes); nearest SIM beam (3200 over 2 pi);
  no return -> 0.0; ranges quantised to 1 mm; range_min 0.05, range_max 30.
  Stamps: the harness delivers a scan at the first 10 ms step after sensor stamp + latency (+ jitter); the facade
  reproduces that time exactly (same latency RNG stream, in-order delivery) and uses it as the receipt. The SIM
  header is the SIM sensor stamp (true scan time); scan_latency_s 0.08 then makes the executor treat nominal
  scans as 80 ms older than they are (inside the real 0.005-0.15 s uncertainty; deliberately not corrected).
  Operator emulation: Start (row length) at the first tick, Arm on the next, press hold at 1.2 s and keep the
  heartbeat. After any pause (stale input, stall, corridor lost, ...) the operator releases and presses again with
  a NEW press id once the page would accept it: at least 0.3 s later, inputs fresh for fresh_recover_s and, after a
  corridor-lost pause, the vehicle seen at rest (review S2 / Fix A rule). Every resume press is counted (at most 20
  per episode); a press the Runner still ignores is counted separately (press_ignored).
Variants
  plain       the SIM stack as validated (SIM estimator defaults incl. corridor_every_n_scans 2, SIM 3200-beam
              scans, no executor) with the SAME controller settings as the executor (cfg.controller_settings()).
  exec_field  the executor, backstop row length = operator-measured length from the start pose to the last
              trunk line: MVP-1 stop = min(row_end_candidate, distance >= L-2 m, time >= 1.15 (L-2)/v_measured).
  exec_cone   the executor with the distance backstop at the +1.5 m cone (row length L+3.5 m) to compare the row-end
              recognition and the controller behaviour with plain (the SIM judge needs row_end_candidate).
  exec_cone_simlike  exec_cone with P1 (near confirm), P3 (geometry guard), P4 (header stamps), P5 (n=3) OFF:
              separates the executor path itself (format, stamps, loop, bridge, hold) from the gate rules.
Plant: P_fit1003 (WEEKEND_PLAN 4.2; 40/55/70/88 % -> 0.33/0.47/0.61/0.78 m/s, arc 0.15 deg/s per %diff, vibration
0.7, pivot min 55) under every scenario's own plant overrides (the scenario wins for the names it sets).
Plant brackets (--plant NAME, or 'plant' of a matrix entry; PLANTS below): arc006 / arc030 (arc gain 0.06 / 0.30),
db30 (forward deadband 30 %: 55 % -> 0.33 m/s) and stickSSrRR = P_fit1003 + a SIM_ASSUMED per-side breakaway
(StictionPlant: a side whose applied duty is below the static threshold SS does not drive from rest; once driving it
keeps driving down to the rolling threshold RR, below which it stops driving (its duty counts as 0 in the response
model: the vehicle arcs toward that side and slows); a side that is dragged along (vehicle moving) re-engages at
RR). Nothing in the plant brackets is measured; they bracket the unknown low-duty response (field C2/C3b/C5).
Executor config: the field default (config.ExecutorConfig()) made motor-feasible for this plant with SIM 'measured'
values (breakaway 20 % (day 1-2: 10 %; the day-2 config refuses < 20; the guard minimum 30 never binds at base 55 / diff
20), v 0.47 m/s at 55 %, C7 stop 0.25 m, yaw 6.0 deg/s at +-20 = 0.15 x 40) and max_yaw_rate_deg_s 3.6
(= 0.6 x 6.0, the feasibility rule; the plan's 4.5 would be refused for this plant), changed by --config-override
or the 'config' of a matrix entry (every config must pass validate(), i.e. the motor feasibility rules).
Time-backstop trace (exec variants): the motion time and counted distance of the Backstop joined with the truth
progress along the row on a 1 s motion-time grid ('tb_trace': [motion_time_s, counted_m, truth_progress_m]) plus the
open-loop reference speed of the run (P_fit forward curve at the base duty x the plant's per-run speed scale), for
the counterfactual 'which v_measured would have been early / late' analysis.

Usage (from the worktree root):
  python -B tools/row_executor/sim_in_loop.py --out <file.json> --seeds 0 1 --workers 6 [--variants ...]
         [--scenarios ...] [--plant NAME] [--config-override KEY=JSON ...]
  python -B tools/row_executor/sim_in_loop.py --matrix <matrix.json> --out <file.json> --workers 6
         matrix.json: {"seeds": [0, 1], "scenarios": null | [...], "entries": [{"label": "P0_d10",
         "plant": "p_fit1003", "variants": ["exec_field", "exec_cone"], "config": {"max_differential_pct": 10}}]}
         Rows are appended to <out>.jsonl as they finish (a re-run skips the rows already there); <out> gets the
         per-entry summaries. New work is not submitted while the HOLD_MEASUREMENT file of the configured hold folder
         exists (sim/ubts/closed_loop/external_paths.py, key 'hold_dir'; not configured -> error, nothing runs).
External locations (steering branch, docs/steering/EXECUTOR_IMPORT_KO.md): the dashboard folder of RoverControl comes
from sim_paths.py ('dashboard_dir': UBTS_DASH_DIR, a config file, or this repository's dashboard/ folder); the real S2
scan format (RealFormat, RAW_AMIN, RAW_INC, BEAMS, SIM_BEAMS) lives in sim/ubts/steering/s2_format.py and is re-exported
here unchanged.
"""
import argparse
import json
import math
import os
import re
import statistics
import sys
import time
import types
import zlib
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
from dataclasses import replace

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
SIM_DIR = os.path.join(REPO, 'sim', 'ubts')
for path in (SIM_DIR, TOOLS):
    if path not in sys.path:
        sys.path.insert(0, path)
sys.dont_write_bytecode = True

import numpy as np  # noqa: E402

from closed_loop import controllers as SC  # noqa: E402  (SIM tree; byte-identical to the vendored copy)
from closed_loop import estimator as SE  # noqa: E402
from closed_loop import external_paths as SIM_PATHS  # noqa: E402  (HOLD_MEASUREMENT folder)
from closed_loop import harness as SH  # noqa: E402
from closed_loop import matrix as SM  # noqa: E402
from closed_loop.world import build_world  # noqa: E402
from field_response import FieldResponseParams, forward_curve, provenance  # noqa: E402
from row_executor import config as XC  # noqa: E402
from row_executor import core as XCORE  # noqa: E402
from row_executor import executor_main as XM  # noqa: E402
from row_executor import sim_paths as XP  # noqa: E402  (dashboard folder)
from row_executor.bridge_client import BridgeClient  # noqa: E402
# real S2 scan format (shared with the G-SIM S2 format node); re-exported under the old names
from steering.s2_format import BEAMS, RAW_AMIN, RAW_INC, SIM_BEAMS, RealFormat  # noqa: E402,F401

GRID = .01                                   # harness step
P_FIT1003 = (('speed_at_90_m_s', .80), ('forward_deadband_pct', 5.), ('arc_yaw_deg_s_per_pct_diff', .15),
             ('drive_vibration_rms_m_s2', .7), ('pivot_min_duty_pct', 55.))
# breakaway 20 (day-2 N1: the config refuses < 20 %; the SIM plant's deadband is 5 %, so 20 never binds at diff 20 and
# base 55), measured_stop_distance_m 0.25 (day-2 N8: row motor needs C7; the model at 0.47 m/s is 0.29 m)
SIM_MEASURED = dict(motor_output_enabled=True, breakaway_duty_pct=20, v_measured_m_s=.47,
                    measured_yaw_at_max_diff_deg_s=6., max_yaw_rate_deg_s=3.6, measured_stop_distance_m=.25)
PRESS_AT_S = 1.2
REPRESS_WAIT_S = .3
MAX_REPRESSES = 20
TB_GRID_S = 1.                               # time-backstop trace resolution (motion time)
VARIANTS = ('plain', 'exec_field', 'exec_cone', 'exec_cone_simlike')
# exec_cone_simlike: the executor path with the C1-gate rules P1/P3/P4/P5 switched off (attribution only).
SIMLIKE = dict(corridor_every_n_scans=2, near_confirm_scans=0, guard_heading_deg=None, guard_lateral_margin_m=None,
               scan_stamp_mode='receipt')
# Plant brackets on top of P_FIT1003 (SIM only, none measured): name -> (extra overrides, stiction or None).
PLANTS = {
    # Preserve the complete 10/10 plant defaults. No P_FIT1003 coefficients are
    # merged into this profile; unmeasured low-duty settings remain SIM_ASSUMED.
    'field1010': ((), None),
    'p_fit1003': ((), None),
    'arc006': ((('arc_yaw_deg_s_per_pct_diff', .06),), None),
    'arc030': ((('arc_yaw_deg_s_per_pct_diff', .30),), None),
    'db30': ((('forward_deadband_pct', 30.),), None),
}
STICK_NAME = re.compile(r'^stick(\d{1,2})r(\d{1,2})$')


# --- plants ----------------------------------------------------------------------------------------------
ORIGINAL_FIELD_PLANT = SH.FieldPlant


class StictionPlant(ORIGINAL_FIELD_PLANT):
    """FieldPlant + SIM_ASSUMED per-side breakaway (static_pct from rest, rolling_pct once driving).

    The applied MCU duty of each side passes through an engaged/disengaged state: disengaged -> engaged when
    |duty| >= static_pct, or >= rolling_pct while the vehicle moves (the side's wheels are being dragged);
    engaged -> disengaged when |duty| < rolling_pct. A disengaged side contributes duty 0 to the unchanged
    FieldPlant response (mean -> speed, difference -> yaw): an inner wheel below the rolling threshold makes the
    vehicle arc toward that side and slow down; both sides below it stop the vehicle (coast). Not measured.
    """
    static_pct = 40.
    rolling_pct = 30.
    moving_m_s = .02
    last = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.engaged = [False, False]
        self.stiction = dict(one_side_s=0., none_s=0., drop_events=0, drop_events_moving=0)
        type(self).last = self

    def _targets(self, left, right, dt):
        moving = abs(self.v) > self.moving_m_s
        effective = []
        for side, duty in enumerate((left, right)):
            magnitude = abs(duty)
            engaged = self.engaged[side]
            if engaged and magnitude < self.rolling_pct:
                engaged = False
                if magnitude > 0:
                    self.stiction['drop_events'] += 1
                    self.stiction['drop_events_moving'] += moving
            elif not engaged and magnitude > 0 and (magnitude >= self.static_pct
                                                     or (moving and magnitude >= self.rolling_pct)):
                engaged = True
            self.engaged[side] = engaged
            effective.append(duty if engaged else 0.)
        commanded = (left != 0, right != 0)
        if all(commanded):
            if sum(self.engaged) == 1:
                self.stiction['one_side_s'] += dt
            elif not any(self.engaged):
                self.stiction['none_s'] += dt
        return super()._targets(effective[0], effective[1], dt)


def stiction_class(static_pct, rolling_pct):
    if not 0 < rolling_pct <= static_pct < 100:
        raise ValueError('stiction needs 0 < rolling <= static < 100')
    return type('StictionPlant_%d_%d' % (static_pct, rolling_pct), (StictionPlant,),
                dict(static_pct=float(static_pct), rolling_pct=float(rolling_pct), last=None))


def plant_spec(name, extra=(), stiction=None):
    """(name, overrides tuple on top of P_FIT1003, stiction (static, rolling) or None)."""
    if name in PLANTS:
        overrides, stick = PLANTS[name]
    else:
        match = STICK_NAME.match(name or '')
        if not match:
            raise ValueError('unknown plant %r (known: %s, stickSSrRR)' % (name, sorted(PLANTS)))
        overrides, stick = (), (float(match.group(1)), float(match.group(2)))
    if stiction is not None:
        stick = tuple(float(v) for v in stiction)
    if stick is not None:
        stiction_class(*stick)                    # validate
    merged = {} if name == 'field1010' else dict(P_FIT1003)
    merged.update(dict(overrides))
    merged.update(dict(extra))
    return dict(name=name, overrides=tuple(sorted(merged.items())), stiction=stick)


# --- the dashboard bridge core (read-only dashboard folder) ----------------------------------------------------
def rover_control_class():
    """(RoverControl, ControlError, path) from <dashboard folder>/scripts/rover_control_core.py. The folder comes from
    sim_paths 'dashboard_dir' (UBTS_DASH_DIR, the path config file, or this repository's dashboard/ folder, whose copy
    has the same bytes as the real dashboard checkout). RuntimeError when it holds no rover_control_core.py."""
    import importlib.util
    try:
        root = XP.configured_path('dashboard_dir', 'dir')
    except FileNotFoundError as exc:
        raise RuntimeError(str(exc)) from None
    path = os.path.join(str(root), 'scripts', 'rover_control_core.py')
    if not os.path.isfile(path):
        raise RuntimeError('%s has no scripts/rover_control_core.py (set UBTS_DASH_DIR; see '
                           'docs/steering/EXECUTOR_IMPORT_KO.md)' % root)
    spec = importlib.util.spec_from_file_location('dash_rover_control_core', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RoverControl, module.ControlError, path


class SimMotor:
    """EspMotor stand-in for RoverControl: the duty it holds is what the SIM plant receives."""
    firmware = 'UBTS_WHEEL_PWM_V2'
    error = None

    def __init__(self):
        self.duty = (0, 0)
        self.outputs = {}
        self.hard_stops = 0

    @property
    def ready(self):
        return True

    @property
    def stopped(self):
        return self.duty == (0, 0)

    def enable(self):
        pass

    def drive(self, left, right):
        self.duty = (int(left), int(right))

    def release(self):
        self.duty = (0, 0)

    def hard_stop(self):
        self.duty = (0, 0)
        self.hard_stops += 1


class CoreTransport:
    """Bridge.request semantics on top of the real RoverControl (status / op stop / dispatch)."""

    def __init__(self, control, control_error):
        self.control, self.control_error = control, control_error
        self.ops = dict(status=0, arm=0, drive=0, stop=0, other=0)
        self.deadman_false = 0

    def __call__(self, request):
        op = request.get('op')
        self.ops[op if op in self.ops else 'other'] += 1
        if op == 'drive' and request.get('deadman') is not True:
            self.deadman_false += 1
        if op == 'status':
            self.control.tick()
            return dict(self.control.status(), updated_at=time.time(), mode='manual')
        if op == 'stop':
            self.control.stop(str(request.get('reason', 'operator_stop'))[:80])
            return {'ok': True, 'state': 'stop_requested'}
        try:
            return self.control.dispatch(request)
        except (self.control_error, ValueError) as exc:
            return {'ok': False, 'error': str(exc)}


class DiscardLog:
    dropped = 0
    ticks_suppressed = 0
    paths = ()

    def put(self, record, tick=False):
        pass

    def health(self):
        return dict(state='ok', dropped=0, write_errors=0, last_error=None, failing_since=None, recoveries=0,
                    pending=0, lines=0, bytes=0, ticks_suppressed=0, writer_alive=True, file=None)

    def close(self, timeout=0.):
        pass


def ns(**kw):
    return types.SimpleNamespace(**kw)


def header(t):
    sec = math.floor(t)
    return ns(stamp=ns(sec=int(sec), nanosec=int(round((t - sec) * 1e9))))


def grid_ceil(t):
    return math.ceil((t - 1e-9) / GRID - 1e-9) * GRID


# RealFormat (SIM 3200-beam body-frame scan -> real S2 LaserScan ranges) moved to sim/ubts/steering/s2_format.py
# unchanged; it is imported above under the same name.


# --- the facade ---------------------------------------------------------------------------------------------
class ExecutorInTheLoop:
    """Fills the harness' estimator AND controller slots with the real ExecutorLoop (see the module docstring)."""

    def __init__(self, episode, cfg, row_len_m, real_format, rover_control, navigation_gate=None,
                 navigation_test_mode=False):
        RoverControl, ControlError, _ = rover_control
        self.ep = episode
        self.cfg = cfg
        self.now = 0.
        self.motor = SimMotor()
        self.control = RoverControl(self.motor, lambda: self.now)
        self.transport = CoreTransport(self.control, ControlError)
        self.loop = XM.ExecutorLoop(
            cfg, motor_output_flag=True, clock=lambda: self.now, wall=lambda: 0., log=DiscardLog(),
            bridge_factory=lambda: BridgeClient(owner='row-executor:sim', transport=self.transport),
            stop_transport=self.transport,
            navigation_gate=navigation_gate,
            navigation_test_mode=navigation_test_mode,
            runner_factory=lambda c: XCORE.Runner(c, vendor_modules=(SE, SC)))
        self.row_len_m = float(row_len_m)
        self.fmt = real_format
        self.jitter = np.random.default_rng(SH.stream_seed(episode, 'latency'))
        self.last_arrival = -math.inf
        self.phase, self.future, self.press, self.presses, self.represses = 'start', None, None, 0, 0
        self.repress_at = None
        self.release_t = None
        self.state, self.reason, self.hold_terminal = 'REACQUIRE', 'start', False
        self.events, self._controllers = [], []
        self.proc_ms, self.max_abs_diff, self.sent = [], 0, 0
        self.start_error = None
        self.press_ignored = 0
        self.pause_reasons = {}
        self.first_drive_t = None
        self.bs_trace = []                  # (now, motion_time_s, counted_m) while the backstop is armed

    # --- estimator slot ------------------------------------------------------------------------------
    def on_scan(self, stamp, ranges, angle_min, increment, range_max):
        if abs(angle_min + math.pi) > 1e-9 or abs(increment - 2 * math.pi / SIM_BEAMS) > 1e-12:
            raise ValueError('unexpected SIM scan geometry')
        latency = self.ep.lidar_latency_s
        if self.ep.lidar_latency_jitter_s > 0:
            latency += self.jitter.uniform(0, self.ep.lidar_latency_jitter_s)
        arrival = max(stamp + latency, self.last_arrival)
        self.last_arrival = arrival
        msg = ns(header=header(stamp), ranges=self.fmt.convert(ranges), angle_min=RAW_AMIN, angle_increment=RAW_INC,
                 range_min=.05, range_max=30.)
        self.loop.on_scan_msg(grid_ceil(arrival), msg)

    def on_imu(self, stamp, gyro_z, accel):
        msg = ns(header=header(stamp), angular_velocity=ns(x=0., y=0., z=float(gyro_z)),
                 linear_acceleration=ns(x=float(accel[0]), y=float(accel[1]), z=float(accel[2])))
        self.loop.on_imu_msg(grid_ceil(stamp + self.ep.imu_latency_s), msg)

    def snapshot(self, now):
        self.now = now
        self._operator(now)
        decision = self.loop.step(now)
        self.control.tick()
        info = self.loop._last_scan_info
        if info.get('proc_ms') is not None and info.get('received'):
            self.proc_ms.append(info['proc_ms'])
        self._facade_state(now)
        runner = self.loop.runner
        if (decision.left or decision.right) and self.first_drive_t is None:
            self.first_drive_t = now
        backstop = runner.backstop
        if backstop is not None and backstop.active and runner.state != XCORE.IDLE:
            self.bs_trace.append((now, backstop.motion_time_s, backstop.travelled_m))
        s = runner.last_snapshot
        return s if s is not None else dict(corridor_valid=False, corridor_reason='no_run', row_end_ahead_m=None)

    def next_corridor(self):
        est = self.loop.runner.estimator
        return est.next_corridor() if est is not None else dict(left='UNKNOWN', right='UNKNOWN')

    @property
    def odom_failures(self):
        est = self.loop.runner.estimator
        return 0 if est is None else est.odom_failures

    @property
    def distance(self):
        est = self.loop.runner.estimator
        return 0. if est is None else est.distance

    # --- controller slot -----------------------------------------------------------------------------
    def step(self, now, snapshot):
        left, right = self.motor.duty
        self.max_abs_diff = max(self.max_abs_diff, abs(left - right))
        runner = self.loop.runner
        reason = self.reason
        if runner.state == XCORE.ACTIVE and runner.last_command is not None:
            reason = runner.last_command.reason
        return ns(left=left, right=right, state=self.state, reason=reason)

    @property
    def counts(self):
        out = dict(stops=0, turns=0, stalls=0, holds=0, stops_without_turn=0, fallbacks=0, side_stops=0,
                   side_inhibit=0)
        for ctl in self._controllers:
            for key in out:
                out[key] += ctl.counts.get(key, 0)
        return out

    # --- internals -------------------------------------------------------------------------------------
    def _new_press(self, now):
        self.presses += 1
        self.press = 'sim-press-%04d' % self.presses
        self.loop.hold_beat(self.press)
        self.phase = 'holding'

    def _resume_ready(self, now):
        """What the operator page shows before a resume press: inputs fresh long enough and, after a corridor-lost
        pause, the vehicle seen at rest (the Runner rules; an operator presses only when the page says so)."""
        runner = self.loop.runner
        if runner.state != XCORE.PAUSED:
            return False
        fresh_since = runner._fresh_since
        if fresh_since is None or now - fresh_since < self.cfg.fresh_recover_s + .05:
            return False
        if runner._press_needs_standstill:
            s = runner.last_snapshot
            return s is not None and runner._standstill(s)
        return True

    def _operator(self, now):
        loop, runner = self.loop, self.loop.runner
        if self.phase == 'start':
            self.future = loop.submit('start', row_len_m=self.row_len_m)
            self.phase = 'started'
        elif self.phase == 'started' and self.future.done():
            result = self.future.result()
            if not result.get('ok'):
                self.start_error, self.phase = result, 'failed'
            else:
                self.future, self.phase = loop.submit('arm'), 'arming'
        elif self.phase == 'arming' and self.future.done():
            result = self.future.result()
            if not result.get('ok'):
                self.start_error, self.phase = result, 'failed'
            else:
                self.phase = 'armed'
        elif self.phase == 'armed' and now >= PRESS_AT_S:
            self._new_press(now)
        elif self.phase == 'holding':
            if runner.state == XCORE.ENDED:
                loop.hold_release(self.press)
                self.phase = 'done'
            elif runner.state == XCORE.PAUSED and runner.reason != 'await_hold':
                if runner.reason.startswith('press_ignored'):
                    self.press_ignored += 1
                loop.hold_release(self.press)
                self.represses += 1
                self.phase = 'released' if self.represses <= MAX_REPRESSES else 'gave_up'
                self.repress_at = now + REPRESS_WAIT_S
            else:
                loop.hold_beat(self.press)
        elif self.phase == 'released' and runner.state == XCORE.ENDED:
            self.phase = 'done'
        elif self.phase == 'released' and now >= self.repress_at and self._resume_ready(now):
            self._new_press(now)

    def _facade_state(self, now):
        runner = self.loop.runner
        term = False
        if runner.state == XCORE.ENDED:
            state, reason, term = 'HOLD', runner.end_reason, True
        elif runner.state == XCORE.PAUSED:
            state, reason = 'PAUSED', runner.reason
        elif runner.state == XCORE.ACTIVE and runner.controller is not None:
            ctl = runner.controller
            if not any(c is ctl for c in self._controllers):
                self._controllers.append(ctl)
            state, reason = ctl.state, ctl.reason
        else:
            state, reason = 'REACQUIRE', 'start'
        if (state, reason) != (self.state, self.reason):
            self.events.append((round(now, 3), state, reason))
            if state == 'PAUSED' and reason not in ('await_hold',) and not str(reason).startswith('press_ignored'):
                self.pause_reasons[reason] = self.pause_reasons.get(reason, 0) + 1
        self.state, self.reason, self.hold_terminal = state, reason, term

    def summary(self):
        runner, loop = self.loop.runner, self.loop
        proc = np.asarray(self.proc_ms) if self.proc_ms else np.zeros(1)
        bridge = loop.bridge
        return dict(end_reason=runner.end_reason, end_note=runner.end_note, runner_state=runner.state,
                    stale_events=runner.stale_events,
                    stall_guard_pauses=runner.stall_guard_pauses, pauses=runner.pauses, resumes=runner.resumes,
                    pause_reasons=dict(sorted(self.pause_reasons.items())),
                    corridor_lost_pauses=runner.corridor_lost_pauses, inner_limited_ticks=runner.inner_limited_ticks,
                    controllers_built=runner.controllers_built, presses=self.presses, represses=self.represses,
                    press_ignored=self.press_ignored,
                    near_unconfirmed=None if runner.near_filter is None else runner.near_filter.removed,
                    scans_processed=loop.counts['scans_processed'], scans_dropped=loop.counts['scans_dropped'],
                    sends=loop.counts['sends'], bridge_errors=loop.counts['bridge_errors'],
                    bridge_state=None if bridge is None else bridge.state,
                    bridge_ops=dict(self.transport.ops), deadman_false=self.transport.deadman_false,
                    motor_hard_stops=self.motor.hard_stops, max_abs_diff_sent=self.max_abs_diff,
                    guard_violations=runner.guard.violations, start_error=self.start_error,
                    first_drive_t=self.first_drive_t,
                    backstop=None if runner.backstop is None else runner.backstop_summary(),
                    stamper=loop.stamper.summary(),
                    scan_proc_ms=dict(p50=round(float(np.percentile(proc, 50)), 2),
                                      p99=round(float(np.percentile(proc, 99)), 2), max=round(float(proc.max()), 2)))


# --- episodes -----------------------------------------------------------------------------------------------
def executor_config(overrides=None, plant_name='p_fit1003'):
    # The historical SIM_MEASURED name describes a fake calibrated controller,
    # NOT C2/C3/C5/C7 measurements. The 10/10 profile uses its nominal response
    # for speed/yaw rather than the old 10/3 0.47 m/s and 0.15 deg/s/% figures.
    # Never read a realised plant seed, pose, speed scale, or sensor truth here.
    # This historical SIL deliberately tests the sensor-only path. The latest
    # production config defaults to native navigation REQUIRED and cannot use
    # this explicit development-mode exception as a missing-provider fallback.
    values = dict(SIM_MEASURED, require_native_navigation=False,
                  distance_mode='scan_norm_legacy')
    if plant_name == 'field1010':
        nominal = FieldResponseParams()
        requested = replace(XC.ExecutorConfig(), **(overrides or {}))
        authority = 2 * requested.max_differential_pct * nominal.arc_yaw_deg_s_per_pct_diff
        values.update(v_measured_m_s=forward_curve(requested.base_duty_pct, nominal),
                      distance_mode='resolved_polyline',
                      arc_yaw_deg_s_per_pct_diff=nominal.arc_yaw_deg_s_per_pct_diff,
                      measured_yaw_at_max_diff_deg_s=authority,
                      max_yaw_rate_deg_s=min(requested.max_yaw_rate_deg_s,
                                             requested.yaw_authority_fraction * authority),
                      # No low-duty C7 was measured. This explicit SIM bracket
                      # keeps the strict C7 gate and a more conservative stop
                      # predictor; it is never an installable calibration.
                      measured_stop_distance_m=.05,
                      controller_overrides={'reaction_s': .4})
    cfg = replace(XC.ExecutorConfig(), **values)
    if overrides:
        cfg = replace(cfg, **overrides)
    return cfg.validate()


def merge_overrides(own, global_pairs):
    names = {k for k, _ in own}
    return tuple((k, v) for k, v in global_pairs if k not in names) + tuple(own)


def episode_configs(seeds, names=None, cfg=None, plant_pairs=P_FIT1003):
    cfg = cfg or executor_config()
    mode, ctl = cfg.controller_settings()
    ctl_pairs = tuple(sorted(ctl.items()))
    out = []
    for ep in SM.episodes(seeds=seeds, modes=(mode,), names=names):
        out.append(replace(ep, plant_overrides=merge_overrides(ep.plant_overrides, plant_pairs),
                           controller_overrides=merge_overrides(ep.controller_overrides, ctl_pairs)))
    return out


def true_row_length(ep):
    world = build_world(ep.world)
    return float(world.corridor_truth(ep.corridor_index)['end_x']) - ep.start_x_m


def mvp1_outcome(result, variant):
    """Scenario-independent label for the MVP-1 stop rule (truth position of the final stop)."""
    if result['collisions'] > 0:
        return 'collision'
    if result['left_corridor']:
        return 'corridor_departure'
    if result['end'] == 'left_field':
        return 'left_field'
    rel = result['final_x_rel_row_end_m']
    reason = result['final_reason']
    if rel > 1.5:
        return 'overrun_beyond_cone'
    if result['end'] != 'terminal_hold':
        return 'time_limit'
    if reason in ('row_end_candidate', 'row_distance_backstop', 'row_time_backstop') and rel >= -3.5:
        return 'stopped_at_row_end'
    return 'early_safe_stop'


def open_loop_speed(ep, base_duty):
    """Steady straight speed of this run's plant at the base duty (forward curve x per-run speed scale; the
    AR(1) fluctuation is zero mean). An equal-duty start with both sides above the breakaway is assumed."""
    params = SH.plant_params(ep)
    twin = ORIGINAL_FIELD_PLANT(params, surface=ep.surface, seed=(ep.seed, zlib.crc32(
        (ep.stream_key or ep.name).encode('utf-8')), SH.PURPOSES['plant']))
    nominal = forward_curve(base_duty, params)
    return nominal, nominal * twin.speed_run_scale, twin.speed_run_scale


def tb_trace(facade, trace, start_x):
    """Backstop motion time / counted distance joined with the truth progress, on a TB_GRID_S grid + last point."""
    progress = {round(row['t'], 2): row['fx'] - start_x for row in trace}
    out, next_t = [], 0.
    joined = [(mt, counted, progress.get(round(now, 2))) for now, mt, counted in facade.bs_trace]
    joined = [row for row in joined if row[2] is not None]
    for mt, counted, prog in joined:
        if mt >= next_t:
            out.append([round(mt, 2), round(counted, 3), round(prog, 3)])
            next_t = (math.floor(mt / TB_GRID_S) + 1) * TB_GRID_S
    if joined and (not out or out[-1][0] != round(joined[-1][0], 2)):
        mt, counted, prog = joined[-1]
        out.append([round(mt, 2), round(counted, 3), round(prog, 3)])
    return out


def run_job(job):
    variant, ep, cfg_overrides = job[:3]
    plant = job[3] if len(job) > 3 and job[3] else plant_spec('p_fit1003')
    label = job[4] if len(job) > 4 else None
    started = time.perf_counter()
    plant_cls = stiction_class(*plant['stiction']) if plant['stiction'] else None
    SH.FieldPlant = plant_cls or ORIGINAL_FIELD_PLANT
    try:
        if variant == 'plain':
            result = SH.run_episode(ep)
        else:
            cfg = executor_config(dict(cfg_overrides or {}, **(SIMLIKE if variant == 'exec_cone_simlike' else {})),
                                  plant_name=plant['name'])
            length = true_row_length(ep)
            row_len = length if variant == 'exec_field' else length + cfg.backstop_margin_m + 1.5
            fmt = RealFormat(cfg.lidar_yaw_offset_deg)
            facade = ExecutorInTheLoop(ep, cfg, row_len, fmt, rover_control_class())
            result = SH.run_episode(ep, record_trace=True, estimator_factory=lambda _c: facade,
                                    controller_factory=lambda _c: facade)
            trace = result.pop('trace')
            result['executor'] = facade.summary()
            result['executor']['row_len_m'] = round(row_len, 3)
            result['executor']['true_row_len_m'] = round(length, 3)
            nominal, v_open, scale = open_loop_speed(ep, cfg.base_duty_pct)
            result['executor']['speed_ref'] = dict(v_config=cfg.v_measured_m_s, v_nominal=round(nominal, 4),
                                                   v_open=round(v_open, 4), run_scale=round(scale, 4),
                                                   time_factor=cfg.backstop_time_factor,
                                                   margin_m=cfg.backstop_margin_m)
            result['executor']['tb_trace'] = tb_trace(facade, trace, ep.start_x_m)
        if plant_cls is not None and plant_cls.last is not None:
            result['stiction'] = {k: (round(v, 2) if isinstance(v, float) else v)
                                  for k, v in plant_cls.last.stiction.items()}
    finally:
        SH.FieldPlant = ORIGINAL_FIELD_PLANT
    result['wall_s'] = round(time.perf_counter() - started, 2)
    result['events'] = result['events'][-16:]
    result['variant'] = variant
    result['plant'] = plant['name']
    if plant['name'] == 'field1010':
        result['actuator_profile_authority'] = dict(
            profile='field_20261010', evidence_population='20261010_manual_field_records',
            response_parameters=provenance(SH.plant_params(ep)),
            controller_calibration='SIM_NOMINAL_ASSUMPTIONS_NOT_REAL_C2_C3_C5_C7',
            pose_truth_or_PWM_used_for_localization=False,
            low_duty_response_measured=False, hardware_output_or_deployment_qualified=False,
            fresh_holdout=False, scope='generated closed loop, not recorded counterfactual replay')
    if label is not None:
        result['label'] = label
    result.update(SM.judge(result, ep.name))
    result['mvp1'] = mvp1_outcome(result, variant)
    return result


def _pct(values, q):
    return round(float(np.percentile(np.asarray(values, dtype=float), q)), 3) if values else None


def summarise(rows):
    out = {}
    for variant in sorted({r['variant'] for r in rows}):
        rs = [r for r in rows if r['variant'] == variant]
        finals, mvp = {}, {}
        for r in rs:
            finals[r['final_reason']] = finals.get(r['final_reason'], 0) + 1
            mvp[r['mvp1']] = mvp.get(r['mvp1'], 0) + 1
        ex = [r['executor'] for r in rs if 'executor' in r]
        block = dict(runs=len(rs), completed=sum(r['completed'] for r in rs),
                     acceptable=sum(r['acceptable'] for r in rs),
                     collisions=sum(r['collision'] for r in rs), corridor_departures=sum(r['left_corridor'] for r in rs),
                     left_field=sum(r['end'] == 'left_field' for r in rs),
                     time_limit=sum(r['end'] == 'time_limit' for r in rs),
                     branch_contact_runs=sum(r['branch_contact_samples'] > 0 for r in rs),
                     moving_contact_runs=sum(r.get('branch_contact_samples_moving', 0) > 0 for r in rs),
                     invented_row_end=sum(r['invented_row_end'] for r in rs),
                     row_end_recognised=sum(r['row_end_recognised'] for r in rs),
                     lateral_max_m=round(max(r['lateral_max_m'] for r in rs), 3),
                     lateral_rms_mean_m=round(statistics.fmean(r['lateral_rms_m'] for r in rs), 4),
                     mean_speed_m_s=round(statistics.fmean(r['mean_speed_m_s'] for r in rs), 3),
                     min_clearance_m=round(min(r['min_clearance_m'] for r in rs), 3),
                     final_reasons=dict(sorted(finals.items())), mvp1_outcomes=dict(sorted(mvp.items())),
                     wall_s_mean=round(statistics.fmean(r['wall_s'] for r in rs), 2))
        if ex:
            v_cl = [e['tb_trace'][-1][2] / e['tb_trace'][-1][0] for e in ex
                    if e.get('tb_trace') and e['tb_trace'][-1][0] > 5.]
            block['executor'] = dict(
                guard_violations=sum(e['guard_violations'] for e in ex),
                deadman_false=sum(e['deadman_false'] for e in ex),
                bridge_errors=sum(e['bridge_errors'] for e in ex),
                stale_events=sum(e['stale_events'] for e in ex), represses=sum(e['represses'] for e in ex),
                press_ignored=sum(e.get('press_ignored', 0) for e in ex),
                corridor_lost_pauses=sum(e.get('corridor_lost_pauses', 0) for e in ex),
                start_errors=sum(e['start_error'] is not None for e in ex),
                max_abs_diff_sent=max(e['max_abs_diff_sent'] for e in ex),
                near_unconfirmed=sum(e['near_unconfirmed'] or 0 for e in ex),
                scans_dropped=sum(e['scans_dropped'] for e in ex),
                scan_proc_ms_p99_max=max(e['scan_proc_ms']['p99'] for e in ex),
                scan_proc_ms_max=max(e['scan_proc_ms']['max'] for e in ex),
                time_backstop=sum(e['end_reason'] == 'row_time_backstop' for e in ex),
                distance_backstop=sum(e['end_reason'] == 'row_distance_backstop' for e in ex),
                closed_loop_speed_p10_p50_p90=[_pct(v_cl, 10), _pct(v_cl, 50), _pct(v_cl, 90)],
                bridge_end_states={s: sum(e['bridge_state'] == s for e in ex)
                                   for s in sorted({str(e['bridge_state']) for e in ex})})
        out[variant] = block
    return out


def paired(rows, a='plain', b='exec_cone'):
    """Per (scenario, seed): completion changes between two variants."""
    by = {}
    for r in rows:
        by.setdefault((r['name'], r['seed']), {})[r['variant']] = r
    gained, lost, changed = [], [], []
    for key, pair in sorted(by.items()):
        if a in pair and b in pair:
            ca, cb = pair[a]['completed'], pair[b]['completed']
            if cb and not ca:
                gained.append(key)
            elif ca and not cb:
                lost.append(key)
            if pair[a]['final_reason'] != pair[b]['final_reason']:
                changed.append(dict(scenario=key[0], seed=key[1], **{a: pair[a]['final_reason'],
                                                                    b: pair[b]['final_reason']}))
    return dict(pair='%s vs %s' % (a, b), b_completed_a_not=gained, a_completed_b_not=lost,
                final_reason_changed=changed)


def parse_kv(items):
    out = {}
    for item in items or ():
        key, value = item.split('=', 1)
        out[key] = json.loads(value)
    return out


def hold_file():
    """The HOLD_MEASUREMENT file of the configured hold folder (closed_loop/external_paths.py 'hold_dir'; the old code
    used the parent folder of the work tree). PathNotConfigured when no hold folder is configured."""
    return str(SIM_PATHS.hold_file())


def hold_present():
    return os.path.exists(hold_file())


def wait_for_hold():
    while hold_present():
        print('HOLD_MEASUREMENT present, waiting 60 s', flush=True)
        time.sleep(60)


# --- matrix mode ---------------------------------------------------------------------------------------------
def load_matrix(path):
    with open(path, 'r', encoding='utf-8') as fh:
        spec = json.load(fh)
    seeds = spec.get('seeds', [0, 1])
    names = spec.get('scenarios')
    entries, jobs = [], []
    for entry in spec['entries']:
        label = entry['label']
        if any(e['label'] == label for e in entries):
            raise ValueError('duplicate label %s' % label)
        plant = plant_spec(entry.get('plant', 'p_fit1003'), tuple(sorted(entry.get('plant_overrides', {}).items())),
                           entry.get('stiction'))
        overrides = dict(entry.get('config', {}))
        cfg = executor_config(overrides, plant_name=plant['name'])  # original feasibility still applies
        variants = entry.get('variants', ['exec_field', 'exec_cone'])
        for v in variants:
            if v not in VARIANTS:
                raise ValueError('unknown variant %s' % v)
        eps = episode_configs(seeds, names, cfg, plant['overrides'])
        entries.append(dict(label=label, plant=plant, variants=variants, config_overrides=overrides,
                            config_digest=cfg.digest(), controller_settings=dict(cfg.controller_settings()[1]),
                            feasibility=cfg.feasibility(), note=entry.get('note', '')))
        for ep in eps:
            for v in variants:
                jobs.append((v, ep, overrides, plant, label))
    return seeds, names, entries, jobs


def row_key(row):
    return '%s|%s|%s|%s' % (row.get('label'), row['variant'], row['name'], row['seed'])


def job_key(job):
    variant, ep, _, _, label = job
    return '%s|%s|%s|%s' % (label, variant, ep.name, ep.seed)


def run_matrix(args):
    seeds, names, entries, jobs = load_matrix(args.matrix)
    rover_control_class()
    jsonl = args.out + 'l' if args.out.endswith('.json') else args.out + '.jsonl'
    done = {}
    if os.path.exists(jsonl):
        with open(jsonl, 'r', encoding='utf-8') as fh:
            for line in fh:
                line = line.strip()
                if line:
                    row = json.loads(line)
                    done[row_key(row)] = row
    todo = [job for job in jobs if job_key(job) not in done]
    print('matrix: %d entries, %d jobs, %d already done, %d to run' % (len(entries), len(jobs), len(done),
                                                                        len(todo)), flush=True)
    started = time.time()
    finished = 0
    with ProcessPoolExecutor(max_workers=args.workers) as pool, \
            open(jsonl, 'a', encoding='utf-8', newline='\n') as out:
        pending, queue, errors = {}, list(todo), []
        while queue or pending:
            while queue and len(pending) < args.workers:
                if hold_present():
                    if pending:
                        break
                    wait_for_hold()
                job = queue.pop(0)
                pending[pool.submit(run_job, job)] = job
            if not pending:
                continue
            complete, _ = wait(set(pending), return_when=FIRST_COMPLETED)
            for future in complete:
                job = pending.pop(future)
                try:
                    row = future.result()
                except Exception as exc:            # recorded, not marked done: a re-run retries it
                    errors.append(dict(job=job_key(job), error=repr(exc)))
                    print('ERROR %s: %r' % (job_key(job), exc), flush=True)
                    continue
                done[row_key(row)] = row
                out.write(json.dumps(row, separators=(',', ':'), default=str) + '\n')
                out.flush()
                finished += 1
                if finished % 20 == 0 or not (queue or pending):
                    print('%d/%d %.0f s' % (finished, len(todo), time.time() - started), flush=True)
    summary = {}
    for entry in entries:
        rows = [done[job_key(j)] for j in jobs if j[4] == entry['label'] and job_key(j) in done]
        summary[entry['label']] = dict(entry, plant=dict(entry['plant'], overrides=dict(entry['plant']['overrides'])),
                                       summary=summarise(rows))
    result = dict(schema='ubts.row_executor.sim_in_loop.matrix.v1',
                  created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), seeds=seeds,
                  scenarios=names, rows_file=os.path.basename(jsonl), wall_s=round(time.time() - started, 1),
                  errors=errors,
                  base_config=executor_config().to_dict(), entries=summary)
    with open(args.out, 'w', encoding='utf-8', newline='\n') as fh:
        json.dump(result, fh, indent=1, default=str)
        fh.write('\n')
    for label, block in summary.items():
        for variant, s in block['summary'].items():
            print('%-14s %-11s %3d runs completed %3d acceptable %3d col %d dep %d lat %.2f clr %.2f %s'
                  % (label, variant, s['runs'], s['completed'], s['acceptable'], s['collisions'],
                     s['corridor_departures'], s['lateral_max_m'], s['min_clearance_m'], s['final_reasons']))
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    p.add_argument('--out', required=True)
    p.add_argument('--matrix', help='matrix JSON (see the module doc); the other selection options are ignored')
    p.add_argument('--seeds', type=int, nargs='+', default=[0, 1])
    p.add_argument('--variants', nargs='+', default=list(VARIANTS), choices=VARIANTS)
    p.add_argument('--scenarios', nargs='*')
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--plant', default='p_fit1003', help='plant bracket: %s or stickSSrRR' % ', '.join(sorted(PLANTS)))
    p.add_argument('--config-override', action='append', help='ExecutorConfig KEY=JSON (exec variants)')
    args = p.parse_args(argv)
    wait_for_hold()
    if args.matrix:
        return run_matrix(args)
    overrides = parse_kv(args.config_override)
    cfg = executor_config(overrides, plant_name=args.plant)
    plant = plant_spec(args.plant)
    rover_control_class()                          # fail early if the dashboard checkout is missing
    eps = episode_configs(args.seeds, args.scenarios, cfg, plant['overrides'])
    jobs = [(v, ep, overrides, plant, None) for ep in eps for v in args.variants]
    started = time.time()
    rows = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for k, row in enumerate(pool.map(run_job, jobs, chunksize=1)):
            rows.append(row)
            if (k + 1) % 20 == 0:
                print('%d/%d %.0f s' % (k + 1, len(jobs), time.time() - started), flush=True)
    result = dict(schema='ubts.row_executor.sim_in_loop.v1', created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                  seeds=args.seeds, variants=args.variants, scenarios=sorted({ep.name for ep in eps}),
                  plant=dict(plant, overrides=dict(plant['overrides'])), plant_overrides=dict(plant['overrides']),
                  executor_config=cfg.to_dict(), config_digest=cfg.digest(),
                  controller_settings=dict(cfg.controller_settings()[1]), mode=cfg.controller_settings()[0],
                  wall_s=round(time.time() - started, 1), summary=summarise(rows),
                  paired=[paired(rows, 'plain', 'exec_cone'), paired(rows, 'exec_cone', 'exec_field'),
                          paired(rows, 'plain', 'exec_cone_simlike'), paired(rows, 'exec_cone_simlike', 'exec_cone')],
                  runs=rows)
    with open(args.out, 'w', encoding='utf-8', newline='\n') as fh:
        json.dump(result, fh, indent=1, default=str)
        fh.write('\n')
    print(json.dumps(result['summary'], indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
