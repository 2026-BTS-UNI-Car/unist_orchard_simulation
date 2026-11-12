"""T1 rig: host bridge + V3 link + firmware-mode MCU + SteeredPlant + world + sensors on ONE SIM clock.

Design 2.3.1, 2.5 (T1), 4.1, 4.2 and 8.3 P4 (the part below the executor in 'sim_in_loop_steer'). Every piece is the
real code of its tier:

  host     dashboard/scripts: SteerBridgeCore + SteerControl (control API) + EspSteerV3 + MemoryTransport
  MCU      firmware/ubts_steer_v3 in the native lockstep v2 process (firmware mode, FW_STEP), with the C++ steering
           actuator and potentiometer model in the same process (actuator_link.FirmwareSteeringBackend)
  plant    steered_plant.SteeredPlant (drive line, kinematic bicycle, IMU model); its steering backend is the
           firmware process and its drive duty is the MCU's drive pins (RigPlant, below)
  world    W0 flat world (flat_world.py) or a W2 raster corridor (closed_loop.raster_corridor); LiDAR from the world
           at the plant's LiDAR pose (cpu_lidar.ScanSynthesizer), IMU from the plant

Clock and order (all times are SIM time; 1 ms = 1 MCU tick; T1Rig.ms = ticks run since RESET):
  plant step k covers the ticks [t, t + 10) with t = 10 k:
    1. callbacks and scheduled faults due at or before t
    2. bridge step, when t is on the bridge grid (bridge_phase_ms + j * bridge_period_ms, default every 20 ms):
       the host clock reads t / 1000; the core reads every MCU byte delivered at or before t, answers at most one
       queued request and writes (STOP first, then C / PING). Host bytes written at t reach the MCU by the UART rule
       of LOCKSTEP_PROTOCOL_KO.md 11.7 (115200 baud: the first byte at tick t + 1, a 32 B C line's LF at t + 3)
    3. MCU: one FW_STEP of 10 ticks with the plant speed and standstill flag at t (supply scale from the drive duty
       at t); events and host bytes at their ticks
    4. MCU -> host: an MCU line is delivered when its LF has left the wire: at (LF done tick + 1) ms, i.e. a status
       line queued at tick n (85 B at most) is readable from about n + 8 and read by the next bridge step after that
    5. plant: SteeredPlant.step(0.01) with the MCU's drive pins at the last tick of the step (equal duty: mean of
       left and right), the drive EN (D1 low = coast) and a brake edge when a duty fell to 0 at once (more than one
       ramp step, so not the 255/s ramp); the true steering comes from the firmware process
    6. sensors at t + 10: a LiDAR scan when t + 10 is on the LiDAR grid (profile lidar_hz), an IMU sample at the
       first step end at or after each IMU due time (profile imu_rate_hz: 33 Hz gives 30 / 30 / 40 ms spacing on
       the 10 ms grid); packets become readable at stamp + latency (poll_sensors)
    7. trace row (every trace_every steps)
  A control-API request is answered by the first bridge step at or after the time it is submitted (0 .. 20 ms);
  RigClient blocks (advances the rig) until then, like a socket client waits for the bridge loop.

Truth isolation: an executor or estimator may use only poll_sensors(), the RigClient replies and clock(). plant,
truth(), mcu_channels() and the truth_* / mcu_* trace columns are evaluation data. The MCU's internal servo values
(report_servo channels) are evaluation data too.

Model simplifications kept from P3 (FIRMWARE_V3_KO.md 7, LOCKSTEP 11.5): the lockstep TX side is the 16384 B
simplification (the ESP8266 core 3.1.2 TX FIFO model is a unit-test option only), the hardware RX FIFO is not
modelled, partial EEPROM failures are not modelled. MCU -> host bytes are delivered per line (the host acts on whole
lines only). The supply scale of a step uses the drive duty at its start.

Every number comes from the placeholder profile: results are placeholder-based SIM results (자리값 기반 SIM 결과),
not field validation. Usage: see docs/steering/T1_RIG_KO.md.
"""
from __future__ import annotations

import collections
from dataclasses import dataclass, field, replace
import hashlib
import importlib
import io
import json
import math
import os
from pathlib import Path
import sys
import zipfile
import zlib

import numpy as np

from . import backend_api as ba
from . import flat_world as fw
from . import host_profile_export as hpe
from . import steered_plant as sp
from .actuator_link import FirmwareSteeringBackend
from .native import build as native_build
from .native import fw_eeprom
from .native import lockstep_v2 as v2
from .vehicle_profile import load_profile

RIG_SCHEMA = 'ubts.steer_sim.t1_rig.v1'
TRACE_SCHEMA = 'ubts.steer_sim.t1_rig_trace.v1'
SERIAL_LOG_SCHEMA = 'ubts.steer_sim.t1_rig_serial.v1'
RESULT_KIND = '자리값 기반 SIM 결과'
PLANT_STEP_MS = 10
PLANT_DT_S = .01
BRIDGE_PERIOD_MS = 20                     # steer_rover_bridge loop period
REQUEST_TIMEOUT_S = .4                    # SteerBridgeCore.REQUEST_TIMEOUT_S (the wrapper's socket wait)
STANDSTILL_SPEED_M_S = sp.STANDSTILL_SPEED_M_S
RAMP_STEP_MAX = 13                        # largest MCU ramp step (255/s x 50 ms cap): a bigger drop to 0 is a brake
WALL_EPOCH_S = 1.7e9                      # wall clock = epoch + SIM time (+ wall_clock_jump faults)
LIDAR_Z_M = .6                            # closed_loop.harness scan height (the casters ignore z)
VEHICLE_PROFILE = 'front_steer'
PURPOSES = dict(plant=1, lidar_noise=2, mcu_boot=201, mcu_chip=202)   # plant / lidar as closed_loop.harness
ENV_DASHBOARD_DIR = 'UBTS_STEER_DASHBOARD_DIR'
REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_DASHBOARD_DIR = REPO_ROOT / 'dashboard'
STOP_CODES = {None: 0, 'release': 1, 'brake': 2, 'coast': 3}
DRIVE_EVENT_CODES = {'none': 0, 'ramp_down': 1, 'brake': 2, 'coast': 3, 'en_on': 4}
ARM_STATE_CODES = {'none': 0, 'pending': 1, 'confirmed': 2}
STOP_STATE_CODES = {'none': 0, 'pending': 1, 'unconfirmed': 2, 'confirmed': 3}
HOST_STATE_CODES = {'starting': 0, 'disarmed': 1, 'armed_manual': 2, 'fault': 3}   # snapshot 'state' (-1: other)
# v1 actuator / potentiometer injections (construction time; LOCKSTEP 9) and rig aliases
MODEL_FAULTS = ('jam', 'slow', 'driver_dead', 'driver_stuck_on', 'polarity_reversed', 'pot_open', 'pot_short',
                'pot_stuck', 'pot_noise_x10', 'pot_jump', 'pot_slip', 'pot_drift', 'pot_reversed')
# 'thermal': an overloaded (slow) steering actuator keeps the servo output above the THERMAL proxy level; at 0.3 x the
# rate it still moves more than the STALL / TRACK thresholds (spec 6.7), so the MCU's THERMAL proxy is what trips.
MODEL_FAULT_ALIASES = {'stall': ('jam', {}), 'pot_noise': ('pot_noise_x10', {'scale': 10.}),
                       'thermal': ('slow', {'rate_scale': .3})}
BOOT_DELAY_TICKS = 70                     # reset -> setup() (ESP protocol 1: real chip about 70 ms [placeholder])
RUNTIME_FAULTS = ('mcu_reset', 'power_cut', 'port_open_reset', 'motor_power_off', 'motor_power_on', 'mcu_stall',
                  'adc_set', 'adc_model', 'link_loss', 'usb_unplug', 'half_frame', 'crc_error', 'range_error',
                  'status_loss', 'bridge_stall', 'bridge_restart', 'host_defect', 'late_drive_after_stop',
                  'wall_clock_jump', 'foreign_write', 'sensor_blackout', 'executor_stall')
EXECUTOR_STALL_TRIGGERS = (None, 'f_ttl')   # executor_stall: at its time, or when the next F TTL line reaches the host
DEFAULT_BRIDGE_DOWN_S = 2.                # bridge_restart: the time without a bridge process
# truth_mcu_pos_rad: the MCU filtered potentiometer position P[n] (servo_pos channel, host calibration table, rad).
# The rig also writes it as truth_hold_position_rad: the target a hold_measured STOP holds (R13-2 / R13-3), the hold
# reference of evaluate_steer (it is not the fast_sim servo view; review P4R-16).
TRUTH_FIELDS = (
    'truth_x_m', 'truth_y_m', 'truth_yaw_rad', 'truth_yaw_unwrapped_rad', 'truth_v_m_s', 'truth_w_rad_s',
    'truth_external_yaw_rate_rad_s', 'truth_distance_m', 'truth_curvature_1_m', 'truth_steer_curvature_1_m',
    'truth_wander_deg_per_m', 'truth_lateral_slip_m_s', 'truth_delta_eq_rad', 'truth_delta_left_rad',
    'truth_delta_right_rad', 'truth_motor_side_rad', 'truth_applied_steer_pwm', 'truth_mcu_pos_rad',
    'truth_mcu_cmd_rad', 'truth_mcu_tgt_rad')
FLOAT_FIELDS = ('t_s', 'cmd_throttle_pct', 'cmd_steer_target_mrad', 'steer_target_rad', 'drive_request_pct',
                'drive_applied_pct', 'steer_measured_rad', 'host_status_age_ms') + TRUTH_FIELDS
INT_FIELDS = (('cmd_stop_mode', np.int8), ('phase', np.int16), ('pot_raw', np.int16), ('fault_bits', np.int32),
              ('drive_en', np.int8), ('steer_en', np.int8), ('truth_model_flags', np.int32),
              ('truth_decelerating', np.int8), ('truth_regime', np.int8), ('truth_mcu_state', np.int16),
              ('truth_steer_mode', np.int8), ('truth_mcu_fault_bits', np.int32), ('truth_mcu_steer_pwm', np.int16),
              ('truth_mcu_drive_l', np.int16), ('truth_mcu_drive_r', np.int16), ('truth_drive_event', np.int8),
              ('host_state', np.int8), ('host_arm_state', np.int8), ('host_stop_state', np.int8),
              ('host_lease', np.int8), ('host_latch_mask', np.int32), ('host_status_seq', np.int32),
              ('host_status_st', np.int8), ('host_status_tgt_permille', np.int16),
              ('host_status_pos_permille', np.int16), ('host_last_q', np.int32), ('host_c_sent', np.int32))
REGIME_CODES = {'still': 0, 'drive': 1, 'stopping': 2, 'blocked': 3}
STEER_TARGET_NOTE = ('steer_target_rad is the MCU servo goal clamp(tgt, soft_r, soft_l) while the steering servo is '
                     'ACTIVE or HOLD (the last one is held while the servo is off); a hold_measured stop holds the '
                     'MCU filtered potentiometer position P[t0] (truth_mcu_pos_rad = truth_hold_position_rad at the '
                     'stop). permille -> rad by the host calibration table (marks), not the tyre angle')
HOLD_POSITION_NOTE = ('truth_hold_position_rad: the MCU filtered potentiometer position P[n] (servo_pos channel, host '
                      'calibration table, rad), the same values as truth_mcu_pos_rad; a hold_measured STOP holds P at '
                      'the STOP tick (R13-2 / R13-3); evaluation only')
UNITS = {'_s': 's', '_pct': '% duty', '_mrad': 'mrad', '_rad': 'rad', '_m': 'm', '_m_s': 'm/s', '_rad_s': 'rad/s',
         '_1_m': '1/m', '_deg_per_m': 'deg/m', '_ms': 'ms', '_permille': 'permille', 'pot_raw': 'ADC count',
         'fault_bits': 'FaultBit mask', 'truth_model_flags': 'ModelFlag mask',
         'truth_applied_steer_pwm': 'signed fraction of full PWM'}


class RigError(RuntimeError):
    """The rig cannot be built or was used wrongly."""


class RigDependencyError(RigError):
    """A world needs external data or an optional library that is missing or broken here (W2: the corridor
    catalogue and maps under UBTS_STEER_PATHS, and the raster LiDAR's library). Tests skip on this error only."""


class BridgeUnavailable(OSError):
    """No bridge process (bridge_restart fault): a control-API request fails at once, like a socket connect to a
    control socket nobody listens on. The executor harness maps it to the socket transport's error."""


# --- small helpers ------------------------------------------------------------------------------------------------
def stream_seed(seed, stream_key, purpose):
    """Deterministic 32-bit seed for (seed, stream key, purpose) (closed_loop.harness.stream_seed rule)."""
    key = zlib.crc32(stream_key.encode('utf-8'))
    return int(np.random.SeedSequence([int(seed), key, int(purpose)]).generate_state(1)[0])


def plant_seed(seed, stream_key):
    """The seed closed_loop.harness and fast_sim give the plant: (seed, CRC-32 of the stream key, 1)."""
    return (int(seed), zlib.crc32(stream_key.encode('utf-8')), PURPOSES['plant'])


def yaw_quaternion(yaw):
    return (0., 0., math.sin(yaw / 2.), math.cos(yaw / 2.))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def write_npz(path, arrays):
    """npz whose bytes depend only on the arrays (fast_sim.write_npz rule: fixed member time, sorted members)."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for name in sorted(arrays):
            data = io.BytesIO()
            np.lib.format.write_array(data, np.ascontiguousarray(arrays[name]), allow_pickle=False)
            info = zipfile.ZipInfo(name + '.npy', date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.create_system = 3
            info.external_attr = 0o644 << 16
            archive.writestr(info, data.getvalue())
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp%d' % os.getpid())
    tmp.write_bytes(buffer.getvalue())
    os.replace(tmp, path)


def content_sha256(arrays):
    """SHA-256 over (name, dtype, shape, C-order bytes) of every array, sorted by name (fast_sim.content_sha256)."""
    digest = hashlib.sha256()
    for name in sorted(arrays):
        array = np.ascontiguousarray(arrays[name])
        digest.update(name.encode('utf-8') + b'\0' + array.dtype.str.encode('ascii') + b'\0'
                      + repr(tuple(array.shape)).encode('ascii') + b'\0')
        digest.update(array.tobytes())
    return digest.hexdigest()


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True, allow_nan=False)


_HOST_STATE = {}


def host_modules(dashboard_dir=None):
    """(esp_motor_v3, steer_bridge_core) of the dashboard of this work tree (or env UBTS_STEER_DASHBOARD_DIR).

    Another 'scripts' package may already be on the path (an image can carry an older dashboard copy): the host
    modules are imported with this dashboard first, then sys.path and the earlier 'scripts*' modules are put back.
    The host never imports sim code; this rig imports the host. Raises RigError when the files are missing.
    """
    root = Path(dashboard_dir or os.environ.get(ENV_DASHBOARD_DIR) or DEFAULT_DASHBOARD_DIR)
    key = str(root.resolve())
    if key in _HOST_STATE:
        return _HOST_STATE[key]
    if not (root / 'scripts' / 'steer_bridge_core.py').is_file():
        raise RigError('host code not found: %s/scripts/steer_bridge_core.py (set %s)' % (root, ENV_DASHBOARD_DIR))
    saved_path = list(sys.path)
    saved = {name: module for name, module in sys.modules.items() if name == 'scripts' or name.startswith('scripts.')}
    for name in saved:
        del sys.modules[name]
    try:
        sys.path.insert(0, str(root))
        v3 = importlib.import_module('scripts.esp_motor_v3')
        bc = importlib.import_module('scripts.steer_bridge_core')
    finally:
        sys.path[:] = saved_path
        for name in [n for n in sys.modules if n == 'scripts' or n.startswith('scripts.')]:
            del sys.modules[name]
        sys.modules.update(saved)
    _HOST_STATE[key] = (v3, bc)
    return v3, bc


def resolve_native(native_executable=None, build_dir=None, compiler=None):
    """Path of a native program with firmware mode: the given file, else a build into build_dir (or env
    UBTS_STEER_BUILD_DIR), which must lie outside every work tree of the repository (builds live outside)."""
    if native_executable is not None:
        path = str(native_executable)
        if not os.path.isfile(path):
            raise RigError('native program not found: %s' % path)
        return path
    if not native_build.firmware_present():
        raise RigError(native_build.missing_firmware_message())
    folder = build_dir or os.environ.get(native_build.ENV_BUILD_DIR)
    if not folder:
        raise RigError('set %s (or build_dir) to a folder outside the repository for the native build'
                       % native_build.ENV_BUILD_DIR)
    from .scene_spec import inside_repository
    if inside_repository(folder):
        raise RigError('native build folder %s lies inside the repository; use a runs folder' % folder)
    result = native_build.ensure_built(compiler=compiler, build_dir=folder)
    if result.firmware_build_id is None:
        raise RigError('the native program %s was built without the firmware sources' % result.path)
    return result.path


# --- configuration --------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class RigFault:
    """One fault. kind: a model fault (MODEL_FAULTS or an alias of MODEL_FAULT_ALIASES; construction time only,
    passed to the native as a v1 injection) or a runtime fault (RUNTIME_FAULTS). at_s: SIM time from the start.
    params: the injection params (model faults) or the runtime fault's options (T1_RIG_KO.md 6)."""

    kind: str
    at_s: float = 0.
    params: dict = field(default_factory=dict)


@dataclass(frozen=True)
class RigConfig:
    """Everything one rig run needs (all values placeholders unless a caller sets measured ones).

    world            None or a scene_spec.WorldSpec (W0 markers, W2 record / corridor / direction / modifiers)
    start_pose       (x, y, yaw_rad) of the base: W0 world frame, W2 corridor frame (s, d, heading error)
    start_steer_rad  true wheel-side equivalent angle at the start
    profile          VehicleProfile (None: the default placeholder profile); its steer_actuator_level must be L1
    bridge_phase_ms  first bridge step time (0 or 10 with the 20 ms period)
    imu_rate_hz / lidar_hz   None: the profile values (imu_rate_hz, lidar_hz)
    record_overrides CFG values written into the EEPROM record (fw_eeprom CFG keys); wd / ttlmax / sthz are mirrored
                     into the host profile 'link' unless host_overrides sets that section
    host_overrides   {section: {key: value}} on top of host_profile_export.build()
    host_hw_profile  None (the exported profile's hw_profile) or another hardware profile name the host expects in
                     the banner (A19: hardware_profile_mismatch)
    host_simulated   True: the host link expects a SIM device (banner sim=1); False: a vehicle link (A14 / A19:
                     sim_identity_mismatch against the SIM MCU)
    eeprom_blank     True: blank EEPROM (NOCAL at boot)
    faults           RigFault tuple (model faults go into the startup line, runtime faults are scheduled)
    """

    seed: int = 0
    stream_key: str = 't1_rig'
    world: object = None
    start_pose: tuple = (0., 0., 0.)
    start_steer_rad: float = 0.
    profile: object = None
    curve: str = 'main'
    gyro_noise_scale: float = 1.
    cal_id: int = 1
    host_cal_id: object = 'same'
    mdir: int = 1
    pol_ok: int = 1
    eeprom_blank: bool = False
    record_overrides: tuple = ()
    host_overrides: tuple = ()
    host_hw_profile: object = None
    host_simulated: bool = True
    boot_delay_ticks: int = BOOT_DELAY_TICKS
    boot_garbage_hex: str = ''
    port_open_reset: bool = False
    bridge_period_ms: int = BRIDGE_PERIOD_MS
    bridge_phase_ms: int = 0
    lidar_latency_s: float = 0.
    imu_latency_s: float = .02
    imu_rate_hz: object = None
    lidar_hz: object = None
    faults: tuple = ()
    trace_every: int = 1
    serial_log: bool = True
    report_servo: bool = True
    report_adc: bool = False          # evaluation channel adc_read (raw ADC per tick; P4 A19 POT_NOISE timing check)

    def __post_init__(self):
        for name in ('record_overrides', 'host_overrides'):
            value = getattr(self, name)
            if isinstance(value, dict):
                object.__setattr__(self, name, tuple(sorted(value.items())))
        object.__setattr__(self, 'faults', tuple(self.faults))
        if isinstance(self.seed, bool) or not isinstance(self.seed, int) or not 0 <= self.seed < 2 ** 32:
            raise RigError('rig_seed_must_be_u32')
        if self.bridge_period_ms % PLANT_STEP_MS or self.bridge_period_ms <= 0:
            raise RigError('bridge_period_ms must be a positive multiple of %d' % PLANT_STEP_MS)
        if self.bridge_phase_ms % PLANT_STEP_MS or not 0 <= self.bridge_phase_ms < self.bridge_period_ms:
            raise RigError('bridge_phase_ms must be a multiple of %d below the period' % PLANT_STEP_MS)
        if isinstance(self.trace_every, bool) or not isinstance(self.trace_every, int) or not 1 <= self.trace_every:
            raise RigError('trace_every must be a positive int')
        if self.host_hw_profile is not None and (not isinstance(self.host_hw_profile, str) or not self.host_hw_profile):
            raise RigError('host_hw_profile must be None or a non-empty string')
        if not isinstance(self.host_simulated, bool):
            raise RigError('host_simulated must be a bool')
        for item in self.faults:
            if not isinstance(item, RigFault):
                raise RigError('faults must be RigFault items')
            kind = item.kind
            if kind not in MODEL_FAULTS and kind not in MODEL_FAULT_ALIASES and kind not in RUNTIME_FAULTS:
                raise RigError('unknown rig fault %r' % (kind,))

    def record_override_dict(self):
        return dict(self.record_overrides)

    def host_override_dict(self):
        return {section: dict(values) for section, values in self.host_overrides}


def rig_config_from_scene(scene, seed, **changes):
    """A RigConfig from a scene_spec.SceneSpec: its world, start (pose, steering angle), profile view (file, SHA pin,
    overrides; the backend level must allow L1), curve, gyro noise scale, stream key and trace_every. The scene's
    driver (maneuver / controller) is not used: the rig is driven through the control API."""
    if scene.world.kind == 'W2' and scene.world.allow_sealed:
        raise RigError('sealed corridors are reserved for the steering pre-registration; the rig does not open them')
    if scene.profile.geometry_check:
        raise RigError('geometry_check sets the L0 ideal servo; the firmware rig needs the L1 actuator model')
    if scene.backend == 'L0':
        raise RigError('scene backend L0: the firmware mode runs the L1 actuator model')
    from . import fast_sim
    profile = fast_sim.scene_profile(scene)
    start = scene.start
    data = dict(seed=int(seed), stream_key=scene.stream_key, world=scene.world,
                start_pose=(start.x_m, start.y_m, math.radians(start.yaw_deg)),
                start_steer_rad=math.radians(start.steer_deg), profile=profile, curve=scene.profile.curve,
                gyro_noise_scale=scene.gyro_noise_scale, trace_every=scene.trace_every)
    data.update(changes)
    return RigConfig(**data)


# --- plant in firmware mode --------------------------------------------------------------------------------------------
class _PinDrive:
    """Stand-in for SteeredPlant's MCU ramp: the firmware ramps, the plant takes the MCU drive pins (duty %)."""

    def __init__(self):
        self.applied = 0.
        self.target = 0.
        self.hold_until = -math.inf

    def step(self, now_s, dt):
        return self.applied

    def command(self, duty_pct):
        raise RigError('firmware mode: the drive duty comes from the MCU pins (use the control API)')

    def hard_stop(self, now_s):
        raise RigError('firmware mode: the MCU brakes (use the control API)')


class FirmwareLink(ba.SteeringBackend):
    """SteeringBackend handed to SteeredPlant: configure() starts the firmware process for the plant's draw seed;
    step() returns the output of the FW_STEP the rig ran for this plant step (prime())."""

    LEVEL = 'L1'
    MODES = ('firmware',)

    def __init__(self, rig_options):
        self.options = rig_options
        self.fw = None
        self.cal = None
        self.image = None
        self.startup = None
        self._pending = None
        self.step_inputs = 0

    def configure(self, profile, seed):
        o = self.options
        startup = v2.firmware_startup_config(profile, seed)
        self.startup = startup
        if o['eeprom_blank']:
            image, cal = None, None
        else:
            image, cal = fw_eeprom.calibrated_eeprom(startup, profile, cal_id=o['cal_id'], mdir=o['mdir'],
                                                     pol_ok=o['pol_ok'])
            if o['record_overrides']:
                params = dict(cal['params'], **o['record_overrides'])
                cal = dict(cal, params=params)
                image = fw_eeprom.image_hex(fw_eeprom.encode_record(
                    o['cal_id'], cal['raw_l'], cal['raw_c'], cal['raw_r'], cal['soft_l'], cal['soft_r'],
                    pol_ok=o['pol_ok'], mdir=o['mdir'], params=params))
        self.image, self.cal = image, cal
        firmware = dict(boot_seed=o['boot_seed'], chip_id=o['chip_id'], eeprom_hex=image, adc_mode='model',
                        boot_delay_ticks=o['boot_delay_ticks'], boot_garbage_hex=o['boot_garbage_hex'],
                        initial_delta_eq_rad=o['start_steer_rad'], report_servo=o['report_servo'])
        if o.get('report_adc'):
            firmware['report_adc'] = True       # only when asked: the default startup line is unchanged
        self.fw = FirmwareSteeringBackend(profile, seed, firmware=firmware, injections=o['injections'],
                                          executable=o['executable'])

    def reset(self, delta_eq_rad=0.):
        return self.fw.reset(delta_eq_rad)

    def prime(self, out):
        self._pending = out

    def step(self, inp):
        out, self._pending = self._pending, None
        if out is None:
            raise RigError('the rig plant was stepped outside T1Rig (no FW_STEP output for this step)')
        self.step_inputs += 1
        return out

    def close(self):
        if self.fw is not None:
            self.fw.close()

    def info(self):
        fw_ = self.fw
        return dict(backend='FirmwareLink', level=self.LEVEL, firmware_build=None if fw_ is None else fw_.firmware_build,
                    build_id=None if fw_ is None else fw_.build_id,
                    executable_sha256=None if fw_ is None else fw_.executable_sha256,
                    eeprom_blank=self.image is None)


class RigPlant(sp.SteeredPlant):
    """SteeredPlant whose drive duty is the MCU's pins and whose steering backend is the firmware process.

    The plant's own ramp, stop() and command_*() are not used (the MCU ramps, brakes and steers); apply_pins() sets
    the duty (%), the drive EN and a brake / coast edge before every step. steer_status() is refused: the steering
    state an executor may read is the MCU status line through the bridge (A13)."""

    def __init__(self, params, *, seed, pose, link, initial_steer_rad):
        super().__init__(params, surface='normal', seed=seed, pose=pose, backend=link,
                         initial_steer_rad=initial_steer_rad)
        self._ramp = _PinDrive()
        self.yaw_start = self.yaw
        self.yaw_unwrapped = self.yaw
        self.drive_event = 'none'

    def apply_pins(self, duty_pct, drive_en, event):
        self._ramp.applied = self._ramp.target = float(duty_pct)
        self.drive_event = event
        if event == 'coast' and self.drive_en:
            self.drive_en = False
            self._stop_request = 'coast'
            self.stop_mode = 'coast'
        elif drive_en and not self.drive_en:
            self.drive_en = True
            self.stop_mode = None
        if event == 'brake' and self.drive_en:
            self._stop_request = 'brake'
            self.stop_mode = 'brake'
        elif event == 'ramp_down':
            self.stop_mode = 'release'
        elif duty_pct != 0. and self.drive_en:
            self.stop_mode = None
        self.throttle_request_pct = float(duty_pct)

    def step(self, dt):
        last = self.yaw
        super().step(dt)
        self.yaw_unwrapped += wrap(self.yaw - last)

    def command_actuation(self, throttle_pct, steer_target_mrad):
        raise RigError('firmware mode: drive and steer through the control API (RigClient)')

    def command_drive(self, throttle_pct):
        raise RigError('firmware mode: drive through the control API (RigClient)')

    def command_steer_pwm(self, pwm, steer_en=True):
        raise RigError('firmware mode: the MCU drives the steering')

    def stop(self, mode='brake', steer=None):
        raise RigError('firmware mode: stop through the control API (RigClient.stop)')

    def steer_status(self):
        raise RigError('firmware mode: the steering state is the MCU status line (RigClient.steering)')

    def status(self):
        return dict(applied_pct=self._ramp.applied, drive_en=self.drive_en, drive_event=self.drive_event)


# --- the rig ----------------------------------------------------------------------------------------------------------
class T1Rig:
    """T1 rig (module docstring). T1Rig(config=None, *, native_executable=None, build_dir=None, dashboard_dir=None).

    Time: ms (int, SIM ms since RESET), now (s), clock() (s; the injected clock of the host and the executor).
    Advance: step(dt_s), run_until(t_s), run_while(predicate, limit_s). Control API: client(owner) -> RigClient,
    request(request), submit(request). Sensors: poll_sensors(). Faults: RigConfig.faults, fault(kind, at_s=None,
    **params), at(t_s, fn). Logs: serial_log(), serial_log_bytes(), serial_log_sha256(). Trace: trace_arrays(),
    trace_sha256(), write_trace(path), add_trace_extras(names, fn), set_phase(name). Evaluation: truth(),
    mcu_channel(name), mcu_channel_at(name, tick), mcu_lines(). close() ends the native process.
    """

    def __init__(self, config=None, *, native_executable=None, build_dir=None, dashboard_dir=None, compiler=None):
        self.config = config = RigConfig() if config is None else config
        self.ms = 0
        self._closed = False
        self._bridge_done_ms = None
        self._wall_offset_s = 0.
        self.v3, self.bc = host_modules(dashboard_dir)
        profile = load_profile() if config.profile is None else config.profile
        level = profile.value('steer_actuator_level')
        if level != 'L1':
            raise RigError('the firmware rig runs the L1 actuator model; the profile view has %s' % level)
        self.profile = profile
        self.params = sp.SteeredPlantParams.from_profile(profile, curve=config.curve, geometry_check=False,
                                                         steer_mode='direct_pwm',
                                                         gyro_noise_scale=config.gyro_noise_scale)
        self.executable = resolve_native(native_executable, build_dir, compiler)
        self.seed = config.seed
        self.stream_key = config.stream_key
        model_faults, runtime_faults = self._split_faults(config.faults)
        self.injections = model_faults
        # world and start pose
        self.world, self.world_kind, self.caster, self.lidar = self._make_world(config.world)
        x, y, yaw = (float(v) for v in config.start_pose)
        pose = (x, y, yaw) if self.world_kind == 'W0' else self.world.pose_in_field(x, y, yaw)
        # MCU + plant
        options = dict(boot_seed=stream_seed(self.seed, self.stream_key, PURPOSES['mcu_boot']),
                       chip_id=stream_seed(self.seed, self.stream_key, PURPOSES['mcu_chip']) & 0xFFFFFF,
                       cal_id=config.cal_id, mdir=config.mdir, pol_ok=config.pol_ok, eeprom_blank=config.eeprom_blank,
                       record_overrides=config.record_override_dict(), boot_delay_ticks=config.boot_delay_ticks,
                       boot_garbage_hex=config.boot_garbage_hex, start_steer_rad=float(config.start_steer_rad),
                       injections=model_faults, executable=self.executable, report_servo=config.report_servo,
                       report_adc=config.report_adc)
        self.link_backend = FirmwareLink(options)
        try:
            self._init_after_link(profile, config, pose, runtime_faults)
        except BaseException:
            self.close()                                 # no native process outlives a failed constructor
            raise

    def _init_after_link(self, profile, config, pose, runtime_faults):
        self.plant = RigPlant(self.params, seed=plant_seed(self.seed, self.stream_key), pose=pose,
                              link=self.link_backend, initial_steer_rad=float(config.start_steer_rad))
        self.fw = self.link_backend.fw
        self.cal = self.link_backend.cal
        self._build_host(profile, config)
        # sensors
        self.lidar_hz = float(profile.value('lidar_hz') if config.lidar_hz is None else config.lidar_hz)
        self.imu_rate_hz = float(profile.value('imu_rate_hz') if config.imu_rate_hz is None else config.imu_rate_hz)
        self.lidar_period_ms = int(round(1000. / self.lidar_hz))
        if self.lidar_period_ms % PLANT_STEP_MS:
            raise RigError('lidar period %d ms is not on the 10 ms plant grid' % self.lidar_period_ms)
        self._imu_next_ms = 1000. / self.imu_rate_hz
        self._packets = []
        self._sensor_seq = 0
        self._last_scan_arrival = -math.inf
        # bookkeeping
        self._h2m_log, self._m2h_log = [], []
        self._mcu_line = bytearray()
        self._mcu_line_first = None
        self._mcu_line_garbage = False
        self._duty_state = dict(drive_l=0, drive_r=0, drive_en=0)
        self._last_servo_goal_rad = None
        self._callbacks = []
        self._cb_order = 0
        self._windows = collections.defaultdict(list)    # kind -> [(t0_ms, t1_ms, params)]
        self._corrupt_next = []                           # pending host-line corruptions ('crc', 'range')
        self._rows_f, self._rows_i = [], []
        self._phase_names, self._phase_index, self._phase = [], {}, None
        self.set_phase('run')
        self._extras, self._extra_names = [], []
        self._plant_steps = 0
        self.counters = collections.Counter()
        self.default_client = None
        self._bridge_down_until = None                    # bridge_restart: SIM ms when the new bridge process starts
        self._bridge_restart_reset = False
        self.bridge_events = []                           # (kind, t_ms) of bridge_restart faults (evaluation)
        self._stall_triggers = []                         # executor_stall after='f_ttl', not started yet
        self.executor_stall_windows = []                  # (t0_ms, t1_ms, trigger) of executor_stall faults
        for item in runtime_faults:
            self.fault(item.kind, at_s=item.at_s, **item.params)

    # -- construction helpers
    def _split_faults(self, faults):
        model, runtime = [], []
        for item in faults:
            kind = item.kind
            if kind in MODEL_FAULT_ALIASES:
                real, defaults = MODEL_FAULT_ALIASES[kind]
                params = dict(defaults, **item.params)
                model.append({'kind': real, 'at_s': float(item.at_s), 'params': params})
            elif kind in MODEL_FAULTS:
                model.append({'kind': kind, 'at_s': float(item.at_s), 'params': dict(item.params)})
            else:
                runtime.append(item)
        return ba.validate_injections(model), runtime

    def _make_world(self, spec):
        lidar_seed = stream_seed(self.seed, self.stream_key, PURPOSES['lidar_noise'])
        noise = float(self.profile.value('lidar_noise_sd_m'))
        if spec is None or spec.kind == 'W0':
            markers = () if spec is None else spec.markers
            world = fw.build_flat_world(fw.FlatWorldConfig(markers=tuple(markers)))
            caster, lidar = fw.make_flat_lidar(world, lidar_seed, noise_sd=noise)
            return world, 'W0', caster, lidar
        from .scene_spec import sealed_status
        if spec.allow_sealed or sealed_status(spec.record, spec.corridor_id, spec.direction) is not None:
            raise RigError('sealed corridors are reserved for the steering pre-registration; the rig does not open them')
        from closed_loop import raster_corridor as rc
        modifiers = tuple(rc.modifier_from_dict(m) for m in spec.modifiers)
        try:
            config = rc.corridor_world_config(spec.record, spec.corridor_id, spec.direction, modifiers=modifiers)
            world = rc.build_raster_world(config)
            caster, lidar = rc.make_raster_lidar(world, lidar_seed, noise_sd=noise)
        except (ImportError, OSError) as exc:      # external data not configured / unreadable, library broken
            raise RigDependencyError('W2 %s %s %s is not available here: %s: %s'
                                     % (spec.record, spec.corridor_id, spec.direction, type(exc).__name__, exc)) from exc
        return world, 'W2', caster, lidar

    def _build_host(self, profile, config):
        host_cal = config.cal_id if config.host_cal_id == 'same' else config.host_cal_id
        data = hpe.build(profile, cal_id=host_cal)
        overrides = config.host_override_dict()
        record = config.record_override_dict()
        if 'link' not in overrides:
            mirror = {}
            for cfg_key, host_key in (('wd', 'watchdog_ms'), ('ttlmax', 'c_ttl_max_ms'), ('sthz', 'status_hz')):
                if cfg_key in record:
                    mirror[host_key] = record[cfg_key]
            if mirror:
                overrides['link'] = mirror
        for section, values in overrides.items():
            data[section] = dict(data[section], **values)
        if config.host_hw_profile is not None:
            data['hw_profile'] = config.host_hw_profile
        raw = hpe.to_bytes(data)
        self.host_profile_bytes = raw
        self.hp = self.bc.HostProfile.from_bytes(raw, origin='t1_rig')
        self._host_simulated = config.host_simulated
        self._new_bridge_process(config.port_open_reset)

    def _new_bridge_process(self, port_open_reset):
        """The host objects of one bridge process: a MemoryTransport on the MCU's line, the V3 link, the core (host
        profile self.hp). A bridge_restart builds them again (the new process opens the port: a port-open reset when
        asked)."""
        self.transport = self.v3.MemoryTransport(clock=self.clock)
        if port_open_reset:
            self.transport.on_open.append(lambda: self.fw.event('port_open_reset', tick=self.ms))
        self.link = self.v3.EspSteerV3(self.transport, self.clock, simulated=self._host_simulated,
                                       expect_hw=self.hp.hw_profile, banner_timeout_s=self.hp.banner_timeout_s)
        self.core = self.bc.SteerBridgeCore(self.link, self.hp, clock=self.clock, wall_clock=self.wall_clock)
        self.bridge_processes = getattr(self, 'bridge_processes', 0) + 1

    # -- clocks
    def clock(self):
        """The injected SIM clock (seconds): host bridge, control API and executor."""
        return self.ms / 1000.

    @property
    def now(self):
        return self.ms / 1000.

    def wall_clock(self):
        """Wall clock of the snapshot updated_at only: epoch + SIM time + wall_clock_jump faults."""
        return WALL_EPOCH_S + self.ms / 1000. + self._wall_offset_s

    # -- advancing
    def _require_open(self):
        if self._closed:
            raise RigError('rig closed')

    def step(self, dt_s=PLANT_DT_S):
        """Advance by dt_s (a multiple of 10 ms)."""
        steps = int(round(float(dt_s) * 1000.)) // PLANT_STEP_MS
        if steps * PLANT_STEP_MS != int(round(float(dt_s) * 1000.)) or steps < 0:
            raise RigError('step dt must be a non-negative multiple of 10 ms: %r' % (dt_s,))
        for _ in range(steps):
            self._plant_step()

    def run_until(self, t_s):
        """Advance until SIM time t_s (rounded up to the 10 ms grid)."""
        target = int(math.ceil(float(t_s) * 1000. - 1e-6))
        while self.ms < target:
            self._plant_step()

    def run_while(self, predicate, limit_s):
        """Advance while predicate() is true, at most limit_s; returns the predicate value at the end."""
        end = self.ms + int(round(limit_s * 1000.))
        while self.ms < end and predicate():
            self._plant_step()
        return predicate()

    def run_until_true(self, predicate, limit_s):
        """Advance until predicate() is true (checked before every plant step), at most limit_s."""
        end = self.ms + int(round(limit_s * 1000.))
        while self.ms < end:
            if predicate():
                return True
            self._plant_step()
        return bool(predicate())

    def _bridge_due(self):
        c = self.config
        return (self.ms - c.bridge_phase_ms) % c.bridge_period_ms == 0 and self._bridge_done_ms != self.ms

    def _plant_step(self):
        self._require_open()
        self._run_callbacks()
        if self._bridge_due():
            self._bridge_step()
        self._mcu_and_plant_step()

    def _run_callbacks(self):
        while self._callbacks and self._callbacks[0][0] <= self.ms:
            _, _, fn = self._callbacks.pop(0)
            fn()

    def at(self, t_s, fn):
        """Run fn() at the start of the first plant step at or after SIM time t_s (before its bridge step)."""
        t_ms = int(math.ceil(float(t_s) * 1000. - 1e-6))
        self._callbacks.append((max(t_ms, self.ms), self._cb_order, fn))
        self._cb_order += 1
        self._callbacks.sort(key=lambda item: (item[0], item[1]))

    def _window_active(self, kind, t_ms):
        return any(t0 <= t_ms < t1 for t0, t1, _ in self._windows.get(kind, ()))

    def _window_params(self, kind, t_ms):
        for t0, t1, params in self._windows.get(kind, ()):
            if t0 <= t_ms < t1:
                return params
        return None

    # -- bridge step and host -> MCU bytes
    def _bridge_step(self):
        t = self.ms
        self._bridge_done_ms = t
        if self._bridge_down_until is not None:
            if t < self._bridge_down_until:
                self.counters['bridge_steps_down'] += 1
                return
            self._bridge_down_until = None                 # a new bridge process starts and opens the port
            self._new_bridge_process(self._bridge_restart_reset)
            self.bridge_events.append(('start', t))
            self.counters['bridge_restarts'] += 1
        if self._window_active('bridge_stall', t):
            self.counters['bridge_steps_stalled'] += 1
            return
        self.core.step()
        self.counters['bridge_steps'] += 1
        for written, data in self.transport.device.take_timed():
            self._to_mcu(int(round(written * 1000.)), bytes(data), 'bridge')

    def _to_mcu(self, t_ms, data, source):
        note = source
        if source == 'bridge' and self._corrupt_next:
            data, note = self._corrupt(data)
        loss = self._window_params('link_loss', t_ms)
        if loss is not None and loss.get('direction', 'both') in ('both', 'to_mcu'):
            self._log_h2m(t_ms, None, None, data, note + ':lost')
            self.counters['h2m_lost_bytes'] += len(data)
            return
        arrivals = self.fw.send(data, tick=t_ms, pace='uart')
        self._log_h2m(t_ms, arrivals[0] if arrivals else None, arrivals[-1] if arrivals else None, data, note)

    def _corrupt(self, data):
        """Apply the next pending corruption to a bridge line that it fits.

        'range': a C line only; S becomes 2000 (outside -1000..1000) with a correct CRC, so it passes the CRC check
        and the MCU refuses it as N <q> range. 'crc': a line with a CRC field gets one content byte flipped (the CRC
        kept); a line without one (PING, INFO) gets a wrong CRC field, so every corrupted line is a bad line for
        the MCU's burst counter (a good PING in between would reset it, spec 6.6). STOP is never changed. A line
        that does not fit goes out unchanged and the corruption waits for the next one."""
        kind = self._corrupt_next[0]
        if not data.endswith(b'\n') or data.startswith(b'STOP'):
            return data, 'bridge'
        if kind == 'range':
            if not data.startswith(b'C '):
                return data, 'bridge'
            self._corrupt_next.pop(0)
            parts = data[:-1].split(b'*')[0].split(b' ')
            parts[5] = b'2000'
            self.counters['range_errors'] += 1
            return self.v3.frame(b' '.join(parts).decode('ascii')), 'bridge:range_error'
        body, star, crc = data[:-1].partition(b'*')
        if len(body) < 2:
            return data, 'bridge'
        self._corrupt_next.pop(0)
        self.counters['crc_errors'] += 1
        if not star:
            wrong = (self.v3.crc8(body) + 1) & 0xFF
            return body + b'*%02X\n' % wrong, 'bridge:crc_error'
        index = len(body) // 2
        value = body[index] ^ 0x01
        flipped = bytes([value]) if 0x21 <= value <= 0x7E and value != 0x2A else b'#'
        return body[:index] + flipped + body[index + 1:] + b'*' + crc + b'\n', 'bridge:crc_error'

    def _log_h2m(self, t_ms, first, last, data, note):
        if self.config.serial_log:
            self._h2m_log.append((t_ms, first, last, bytes(data), note))

    # -- MCU step, MCU -> host bytes, plant, sensors
    def _supply_scale(self):
        duty = abs(self.plant._ramp.applied) / 100.
        return max(1e-6, 1. - self.params.pot_supply_sag_per_pwm * duty)

    def _mcu_and_plant_step(self):
        t = self.ms
        plant = self.plant
        speed = plant.v
        out_base = self.fw.step(ba.SteerStepInput(dt_s=PLANT_DT_S, mode='firmware', steer_en=True,
                                                  vehicle_speed_m_s=speed,
                                                  vehicle_stopped=abs(speed) < STANDSTILL_SPEED_M_S,
                                                  supply_scale=self._supply_scale()))
        out = self.fw.last
        self._deliver_mcu_bytes(out.tx)
        duty_pct, drive_en, event = self._drive_from_pins(out, t)
        base = replace(out_base, serial_out=b'')
        self.link_backend.prime(base)
        plant.external_yaw_rate = 0.
        plant.apply_pins(duty_pct, drive_en, event)
        plant.step(PLANT_DT_S)
        self.ms = t + PLANT_STEP_MS
        self._plant_steps += 1
        self._sensors()
        if (self._plant_steps - 1) % self.config.trace_every == 0:
            self._record_row(out)

    def _drive_from_pins(self, out, t):
        state = self._duty_state
        event = 'none'
        for change in out.changes:
            name = change.channel
            if name not in ('drive_l', 'drive_r', 'drive_en'):
                continue
            previous = state[name]
            state[name] = change.value
            if name == 'drive_en':
                if previous and not change.value:
                    event = 'coast'
                elif change.value and not previous and event == 'none':
                    event = 'en_on'
            elif change.value == 0 and previous != 0:
                if abs(previous) > RAMP_STEP_MAX and state['drive_en'] and event != 'coast':
                    event = 'brake'
                elif event == 'none':
                    event = 'ramp_down'
        left, right = state['drive_l'], state['drive_r']
        if left != right:
            self.counters['unequal_duty_steps'] += 1
        duty_pct = (left + right) / 2. / 255. * 100.
        return duty_pct, bool(state['drive_en']), event

    def _deliver_mcu_bytes(self, tx):
        for entry in tx:
            if self._mcu_line_first is None:
                self._mcu_line_first = entry.queued_tick
            self._mcu_line.append(entry.byte)
            self._mcu_line_garbage = self._mcu_line_garbage or entry.boot_garbage
            if entry.byte != 0x0A:
                continue
            data = bytes(self._mcu_line)
            first, garbage = self._mcu_line_first, self._mcu_line_garbage
            self._mcu_line, self._mcu_line_first, self._mcu_line_garbage = bytearray(), None, False
            at_ms = entry.done_tick + 1
            note = 'garbage+line' if garbage else ''
            loss = self._window_params('link_loss', at_ms)
            if loss is not None and loss.get('direction', 'both') in ('both', 'to_host'):
                self._log_m2h(at_ms, first, entry.done_tick, data, 'lost')
                self.counters['m2h_lost_lines'] += 1
                continue
            if self._window_active('status_loss', at_ms) and data.startswith(b'S '):
                self._log_m2h(at_ms, first, entry.done_tick, data, 'status_lost')
                self.counters['status_lines_lost'] += 1
                continue
            if not self.transport.is_open:
                self._log_m2h(at_ms, first, entry.done_tick, data, 'port_closed')
                self.counters['m2h_port_closed_lines'] += 1
                continue
            self.transport.device.send(data, at=at_ms / 1000.)
            self._log_m2h(at_ms, first, entry.done_tick, data, note)
            if self._stall_triggers and data.startswith(b'F TTL'):
                self._trigger_executor_stall(first, at_ms)

    # -- executor stall (fault executor_stall: evaluation of the executor-loop harness)
    def _add_executor_stall(self, t0_ms, duration_ms, trigger):
        window = (int(t0_ms), int(t0_ms) + int(duration_ms))
        self._windows['executor_stall'].append(window + ({'trigger': trigger},))
        self.executor_stall_windows.append(window + (trigger,))
        self.counters['executor_stall_windows'] += 1

    def _trigger_executor_stall(self, queued_tick, readable_ms):
        for item in list(self._stall_triggers):
            if queued_tick is not None and queued_tick >= item['since_ms']:
                self._stall_triggers.remove(item)
                self._add_executor_stall(readable_ms + item['delay_ms'], item['duration_ms'], 'f_ttl')

    def executor_stalled(self, t_ms=None):
        """True while an executor_stall fault window covers SIM ms t_ms (default now): the executor-loop harness runs
        no executor tick then (a late tick is skipped, as on the vehicle when the executor process stalls)."""
        return self._window_active('executor_stall', self.ms if t_ms is None else int(t_ms))

    def _log_m2h(self, at_ms, first, done, data, note):
        if self.config.serial_log:
            self._m2h_log.append((at_ms, first, done, bytes(data), note))

    def _sensors(self):
        t = self.ms
        plant = self.plant
        if t % self.lidar_period_ms == 0:
            lx, ly, lyaw = plant.lidar_pose()
            frame = self.lidar.sample(t * 1000000, (lx, ly, LIDAR_Z_M), yaw_quaternion(lyaw))
            if frame is not None and not self._blacked('lidar', t):
                self._sensor_seq += 1
                arrival = max(t / 1000. + self.config.lidar_latency_s, self._last_scan_arrival)
                self._last_scan_arrival = arrival
                self._packets.append(dict(kind='scan', seq=self._sensor_seq, stamp_s=t / 1000., arrival_s=arrival,
                                          ranges=frame.ranges, angle_min=self.lidar.angle_min,
                                          angle_increment=self.lidar.angle_increment, range_max=self.lidar.maximum))
        if t + 1e-9 >= self._imu_next_ms:
            while self._imu_next_ms <= t + 1e-9:
                self._imu_next_ms += 1000. / self.imu_rate_hz
            sample = plant.imu_sample()
            if not self._blacked('imu', t):
                self._sensor_seq += 1
                self._packets.append(dict(kind='imu', seq=self._sensor_seq, stamp_s=t / 1000.,
                                          arrival_s=t / 1000. + self.config.imu_latency_s,
                                          gyro_z_rad_s=sample['gyro_z_rad_s'], accel_m_s2=tuple(sample['accel_m_s2'])))

    def _blacked(self, which, t_ms):
        params = self._window_params('sensor_blackout', t_ms)
        return params is not None and params.get('which', 'both') in ('both', which)

    def poll_sensors(self):
        """Sensor packets readable now (arrival <= now), in arrival order, each returned once. Scan: kind 'scan',
        stamp_s (SIM sensor time), arrival_s, ranges (float32, SIM beams, inf = no return), angle_min,
        angle_increment, range_max. IMU: kind 'imu', stamp_s, arrival_s, gyro_z_rad_s, accel_m_s2 (sensor frame)."""
        now = self.now + 1e-9
        ready = [p for p in self._packets if p['arrival_s'] <= now]
        if ready:
            self._packets = [p for p in self._packets if p['arrival_s'] > now]
            ready.sort(key=lambda p: (p['arrival_s'], p['seq']))
        return ready

    # -- control API
    def bridge_down(self):
        """True while a bridge_restart fault has no bridge process (between the exit and the next process)."""
        return self._bridge_down_until is not None

    def submit(self, request):
        """Queue a request at the bridge core; the next bridge step answers it. Returns a Future. Without a bridge
        process (bridge_restart) BridgeUnavailable at once (nobody listens on the control socket)."""
        self._require_open()
        if self._bridge_down_until is not None:
            self.counters['requests_without_bridge'] += 1
            raise BridgeUnavailable('no bridge process (control socket not listening)')
        return self.core.submit(request)

    def request(self, request, timeout_s=REQUEST_TIMEOUT_S):
        """submit() and advance the rig until the bridge answered (at most timeout_s SIM time; then the future is
        cancelled, the bridge stops and control_busy_stopped is returned, like SteerBridgeCore.request)."""
        future = self.submit(request)
        core = self.core
        end = self.ms + int(round(timeout_s * 1000.))
        while not future.done() and self.ms <= end:
            self._run_callbacks()
            if core is not self.core or self._bridge_down_until is not None:
                break                                   # the bridge process ended (bridge_restart) before answering
            if self._bridge_due():
                self._bridge_step()
                continue
            if self.ms >= end:
                break
            self._mcu_and_plant_step()
        if future.done():
            return future.result()
        future.cancel()
        if core is not self.core or self._bridge_down_until is not None:
            self.counters['requests_lost_with_bridge'] += 1
            raise BridgeUnavailable('the bridge process ended before it answered (connection reset)')
        self.core.emergency.set()
        self.counters['request_timeouts'] += 1
        return {'ok': False, 'error': 'control_busy_stopped'}

    def client(self, owner='t1-rig-client', expect_cal_id=None):
        """A RigClient (control-API client handle) for this rig."""
        return RigClient(self, owner, expect_cal_id=self.hp.cal_id if expect_cal_id is None else expect_cal_id)

    def observe_motion(self, yaw_rate_rad_s, speed_m_s):
        """Feed a sensor-derived motion sample straight into the core (RigClient.observe_motion uses the op)."""
        self.core.observe_motion(yaw_rate_rad_s, speed_m_s)

    # -- faults
    def fault(self, kind, at_s=None, **params):
        """Schedule a runtime fault at SIM time at_s (default now). See T1_RIG_KO.md 6 for each kind's params."""
        if kind in MODEL_FAULTS or kind in MODEL_FAULT_ALIASES:
            raise RigError('%s is an actuator / potentiometer model fault: give it in RigConfig.faults (it goes into '
                           'the native startup line)' % kind)
        if kind not in RUNTIME_FAULTS:
            raise RigError('unknown rig fault %r' % (kind,))
        t_s = self.now if at_s is None else float(at_s)
        t_ms = int(math.ceil(t_s * 1000. - 1e-6))
        duration_ms = int(round(float(params.get('duration_s', 0.)) * 1000.))
        if kind == 'executor_stall':
            # The executor loop (outside the rig: sim_in_loop_steer) runs no tick while executor_stalled() is true;
            # the rig only keeps the window. after None: from at_s; after 'f_ttl': from the time the first F TTL line
            # queued at or after at_s becomes readable by the host, + delay_s (the executor misses the TTL event in
            # its drive replies for that long: the window of DECISIONS R18 decision 1).
            after = params.get('after')
            if after not in EXECUTOR_STALL_TRIGGERS:
                raise RigError('executor_stall after must be one of %s' % (EXECUTOR_STALL_TRIGGERS,))
            delay_ms = int(round(float(params.get('delay_s', 0.)) * 1000.))
            if duration_ms <= 0 or delay_ms < 0:
                raise RigError('executor_stall needs duration_s > 0 and delay_s >= 0')
            if after is None:
                self._add_executor_stall(t_ms + delay_ms, duration_ms, 'at')
            else:
                self._stall_triggers.append(dict(since_ms=t_ms, delay_ms=delay_ms, duration_ms=duration_ms))
            return
        if kind in ('link_loss', 'status_loss', 'bridge_stall', 'sensor_blackout'):
            if duration_ms <= 0:
                raise RigError('%s needs duration_s > 0' % kind)
            self._windows[kind].append((t_ms, t_ms + duration_ms, dict(params)))
            return
        self.at(t_ms / 1000., lambda: self._apply_fault(kind, params))

    def _apply_fault(self, kind, params):
        t = self.ms
        fw_ = self.fw
        self.counters['fault_' + kind] += 1
        if kind == 'mcu_reset':
            fw_.event('mcu_reset', tick=t, arg=int(params.get('reason', 4)))
        elif kind == 'power_cut':
            fw_.event('power_cycle', tick=t, arg=int(round(float(params.get('off_s', .5)) * 1000.)))
        elif kind == 'port_open_reset':
            fw_.event('port_open_reset', tick=t)
        elif kind == 'motor_power_off':
            fw_.event('motor_power', tick=t, arg=0)
        elif kind == 'motor_power_on':
            fw_.event('motor_power', tick=t, arg=1)
        elif kind == 'mcu_stall':
            fw_.event('mcu_stall', tick=t, arg=int(round(float(params.get('duration_s', .5)) * 1000.)))
        elif kind == 'adc_set':
            fw_.event('adc_set', tick=t, arg=int(params['raw']))
        elif kind == 'adc_model':
            fw_.event('adc_model', tick=t)
        elif kind == 'usb_unplug':
            self.transport.device.unplug()
            duration = float(params.get('duration_s', 1.))
            self.at(t / 1000. + duration, self.transport.device.replug)
        elif kind == 'half_frame':
            text = params.get('text', 'C 1 300 0 0 0')
            data = text.encode('ascii') if isinstance(text, str) else bytes(text)
            self._to_mcu(t, data[:max(1, len(data) // 2)], 'half_frame')
        elif kind in ('crc_error', 'range_error'):
            self._corrupt_next.extend(['crc' if kind == 'crc_error' else 'range'] * int(params.get('count', 1)))
        elif kind == 'late_drive_after_stop':
            client = params.get('client') or self.default_client
            if client is None:
                raise RigError('late_drive_after_stop needs a client (params client=RigClient)')
            client.submit_late_drive_after_stop(int(params.get('throttle', 40)), int(params.get('steer_mrad', 0)))
        elif kind == 'bridge_restart':
            # The bridge process is killed: its port closes (the OS closes the file; nothing is written), its control
            # socket stops answering (requests get BridgeUnavailable) and its queue, lease and latches are gone. After
            # down_s a new process starts with new host objects (no lease, new session: banner, session STOP) and
            # opens the port; port_open_reset True (default: RigConfig.port_open_reset) resets the MCU on that open.
            down_ms = int(round(float(params.get('down_s', DEFAULT_BRIDGE_DOWN_S)) * 1000.))
            if down_ms <= 0:
                raise RigError('bridge_restart needs down_s > 0')
            self._bridge_restart_reset = bool(params.get('port_open_reset', self.config.port_open_reset))
            self._bridge_down_until = t + down_ms
            try:
                if self.link.state != 'closed':
                    self.link.close('bridge_process_killed')
            except OSError:
                pass
            self.bridge_events.append(('exit', t))
        elif kind == 'host_defect':
            # A code defect in the host bridge (A19 internal_error): the next bridge step raises once inside the core
            # loop (the core latches internal_error and stops; a deliberate SIM injection, never a real defect)
            core = self.core
            original = core._drive_link
            state = dict(raised=False)

            def broken(now, _original=original, _state=state):
                if not _state['raised']:
                    _state['raised'] = True
                    raise ValueError('injected host defect (T1 fault host_defect)')
                return _original(now)
            core._drive_link = broken
        elif kind == 'wall_clock_jump':
            self._wall_offset_s += float(params.get('jump_s', 3600.))
        elif kind == 'foreign_write':
            text = params.get('text')
            data = text.encode('ascii') if isinstance(text, str) else bytes(params.get('data', b''))
            if params.get('crc', False):
                data = self.v3.frame(data.decode('ascii'))
            elif not data.endswith(b'\n') and params.get('lf', True):
                data += b'\n'
            self._to_mcu(t, data, 'foreign')

    # -- trace
    def set_phase(self, name):
        """Label of the following trace rows (phase / phase_names columns)."""
        index = self._phase_index.get(name)
        if index is None:
            index = self._phase_index[name] = len(self._phase_names)
            self._phase_names.append(name)
        self._phase = index

    def add_trace_extras(self, names, fn):
        """Extra float columns: fn() -> values (one per name) at every trace row (e.g. executor ctrl_* values).
        Columns added after rows exist are refused (every row must have every column)."""
        if self._rows_f:
            raise RigError('add trace extras before the first trace row')
        names = tuple(names)
        for name in names:
            if name in FLOAT_FIELDS or name in dict(INT_FIELDS) or name.startswith('truth_'):
                raise RigError('extra column %r clashes with a rig column' % name)
        self._extras.append((names, fn))
        self._extra_names.extend(names)

    def permille_to_rad(self, permille):
        """MCU permille -> bicycle equivalent angle (rad) by the host calibration table (exact, not rounded)."""
        hp = self.hp
        if permille >= 0:
            return permille * hp.mark_left_mrad / 1.0e6
        return permille * hp.mark_right_mrad / 1.0e6

    def _servo_goal(self, tgt):
        cal = self.link.cal
        if cal is not None and cal.get('soft_set'):
            return min(cal['soft_l'], max(cal['soft_r'], tgt))
        if self.cal is not None:
            return min(self.cal['soft_l'], max(self.cal['soft_r'], tgt))
        return tgt

    def _host_view(self):
        core, control, link = self.core, self.core.control, self.core.link
        arm = 'pending' if core._arm_pending_t is not None else ('confirmed' if core._arm_confirmed else 'none')
        if core._stop is not None:
            stop = 'unconfirmed' if core._stop['given_up'] else 'pending'
        else:
            stop = 'confirmed' if core._stop_confirmed else 'none'
        mask = 0
        for index, name in enumerate(self.bc.LATCH_NAMES):
            if name in core.latches:
                mask |= 1 << index
        return arm, stop, mask

    def _record_row(self, out):
        plant, fw_, tick = self.plant, self.fw, self.ms - 1
        control, link = self.core.control, self.core.link
        ch = {name: fw_.channel_at(name, tick) for name in v2.CHANNELS}
        mode = ch['steer_mode']
        servo_on = ch['mcu_state'] != v2.MCU_NOT_RUNNING and mode in (1, 2)
        if servo_on or self._last_servo_goal_rad is None:
            self._last_servo_goal_rad = self.permille_to_rad(self._servo_goal(ch['servo_tgt']))
        status = link.status
        status_t = link.status_t
        arm, stop, mask = self._host_view()
        down = self._bridge_down_until is not None      # bridge_restart: no bridge process, no host view, no lease
        leased = bool(control.lease) and not down
        steer_cmd = control.steer_mrad if leased else None
        state = 'no_process' if down else self.core.snapshot.get('state', 'starting')
        st = plant._steer
        row_f = (self.now, float(control.throttle if leased else 0),
                 float('nan') if steer_cmd is None else float(steer_cmd), self._last_servo_goal_rad,
                 float(control.throttle if leased else 0), plant._ramp.applied,
                 float('nan') if status is None else self.permille_to_rad(status['pos']),
                 float('nan') if status_t is None else (self.clock() - status_t) * 1000.,
                 plant.x, plant.y, plant.yaw, plant.yaw_unwrapped, plant.v, plant.w, plant.external_yaw_rate,
                 plant.distance_m, plant.curvature, plant.steer_curvature, plant.wander_deg_per_m, plant.lateral,
                 st.delta_eq_rad, st.delta_left_rad, st.delta_right_rad, st.motor_side_angle_rad, st.applied_steer_pwm,
                 self.permille_to_rad(ch['servo_pos']), self.permille_to_rad(ch['servo_cmd']),
                 self.permille_to_rad(ch['servo_tgt']))
        for _, fn in self._extras:
            row_f = row_f + tuple(float(v) for v in fn())
        last_c = self.core._last_c
        row_i = (STOP_CODES.get(self.core._stop_mode, 0), self._phase, out.base.pot_raw,
                 0 if status is None else status['fl'], int(plant.drive_en), int(ch['steer_en']),
                 st.model_flags, int(plant.decelerating), REGIME_CODES[plant.regime], ch['mcu_state'], mode,
                 ch['fault_bits'], ch['steer_pwm'], ch['drive_l'], ch['drive_r'], DRIVE_EVENT_CODES[plant.drive_event],
                 HOST_STATE_CODES.get(state, -1), ARM_STATE_CODES[arm], STOP_STATE_CODES[stop],
                 int(leased), mask, -1 if status is None else status['n'],
                 -1 if status is None else status['st'], 0 if status is None else status['tgt'],
                 0 if status is None else status['pos'], -1 if last_c is None else last_c['q'],
                 self.core.counters['c_sent'])
        self._rows_f.append(row_f)
        self._rows_i.append(row_i)

    def trace_arrays(self):
        """Trace arrays (fast_sim trace member names where they exist, plus mcu / host columns and meta_json)."""
        names = FLOAT_FIELDS + tuple(self._extra_names)
        floats = np.array(self._rows_f, dtype=np.float64).reshape(-1, len(names))
        ints = np.array(self._rows_i, dtype=np.int64).reshape(-1, len(INT_FIELDS))
        out = {name: floats[:, k].copy() for k, name in enumerate(names)}
        for k, (name, dtype) in enumerate(INT_FIELDS):
            out[name] = ints[:, k].astype(dtype)
        out['truth_hold_position_rad'] = out['truth_mcu_pos_rad'].copy()      # the hold reference (R13-3, P4R-16)
        x_rear = self.params.geometry.rear_axle_x_in_base_m
        yaw = out['truth_yaw_rad']
        out['truth_rear_x_m'] = out['truth_x_m'] + x_rear * np.cos(yaw)
        out['truth_rear_y_m'] = out['truth_y_m'] + x_rear * np.sin(yaw)
        if self.world_kind == 'W0':
            out['truth_field_x_m'] = out['truth_x_m'].copy()
            out['truth_field_y_m'] = out['truth_y_m'].copy()
            out['truth_field_heading_rad'] = np.arctan2(np.sin(yaw), np.cos(yaw))
        else:
            rows = [self.world.to_field(float(x), float(y), float(a))
                    for x, y, a in zip(out['truth_x_m'], out['truth_y_m'], yaw)]
            arr = np.array(rows, dtype=np.float64).reshape(-1, 3)
            out['truth_field_x_m'], out['truth_field_y_m'] = arr[:, 0].copy(), arr[:, 1].copy()
            out['truth_field_heading_rad'] = arr[:, 2].copy()
        out['phase_names'] = np.array(self._phase_names)
        out['meta_json'] = np.frombuffer(_canonical(self.trace_meta(out)).encode('utf-8'), dtype=np.uint8).copy()
        return out

    def trace_meta(self, arrays=None):
        names = sorted(k for k in (arrays or {}) if k not in ('meta_json',))
        units = {}
        for name in names:
            for suffix, unit in UNITS.items():
                if name.endswith(suffix) or name == suffix:
                    units[name] = unit
        return dict(schema=TRACE_SCHEMA, result_kind=RESULT_KIND, field_validated=False, rig=self.identity(),
                    dt_s=PLANT_DT_S * self.config.trace_every, plant_dt_s=PLANT_DT_S,
                    truth_keys=[n for n in names if n.startswith('truth_')], units=units,
                    steer_target_note=STEER_TARGET_NOTE, hold_position_note=HOLD_POSITION_NOTE,
                    bridge_events=[list(e) for e in self.bridge_events],
                    executor_stall_windows=[list(w) for w in self.executor_stall_windows],
                    columns_note=('cmd_*: host bridge request in effect (throttle %, steer_mrad); host_*: host bridge '
                                  'view (status line fields as the host last read them); truth_*: evaluation only '
                                  '(plant truth and the MCU RAM servo channels)'))

    def trace_sha256(self):
        arrays = self.trace_arrays()
        return content_sha256({k: v for k, v in arrays.items() if k != 'meta_json'})

    def write_trace(self, path):
        """Write the trace npz (deterministic bytes); returns (path, content SHA-256 without meta_json)."""
        arrays = self.trace_arrays()
        write_npz(path, arrays)
        return str(path), content_sha256({k: v for k, v in arrays.items() if k != 'meta_json'})

    # -- serial log
    def serial_log(self):
        """Both directions in time order: dicts (dir 'h2m' | 'm2h', t_ms, first_tick, last_tick, data hex, text,
        note). h2m: t_ms = host write time, ticks = arrival ticks of the first / last byte (None: lost). m2h: t_ms =
        host-readable time (LF done tick + 1), ticks = queued tick of the first byte and the LF done tick."""
        records = [('h2m',) + r for r in self._h2m_log] + [('m2h',) + r for r in self._m2h_log]
        records.sort(key=lambda r: (r[1], 0 if r[0] == 'm2h' else 1))
        out = []
        for direction, t_ms, first, last, data, note in records:
            out.append(dict(dir=direction, t_ms=t_ms, first_tick=first, last_tick=last, data=data.hex(),
                            text=data.decode('ascii', 'replace').rstrip('\n'), note=note))
        return out

    def serial_log_bytes(self):
        """Canonical serial log: one JSON object per line (sorted keys), schema header first."""
        lines = [_canonical(dict(schema=SERIAL_LOG_SCHEMA, rig=self.identity(include_files=False)))]
        lines.extend(_canonical(item) for item in self.serial_log())
        return ('\n'.join(lines) + '\n').encode('ascii')

    def serial_log_sha256(self):
        return hashlib.sha256(self.serial_log_bytes()).hexdigest()

    def write_serial_log(self, path):
        data = self.serial_log_bytes()
        Path(path).write_bytes(data)
        return str(path), hashlib.sha256(data).hexdigest()

    # -- evaluation views (never executor inputs)
    def truth(self):
        out = self.plant.truth()
        tick = self.ms - 1
        out['mcu'] = {name: self.fw.channel_at(name, tick) for name in v2.CHANNELS}
        out['t_s'] = self.now
        return out

    def mcu_channel(self, name):
        return self.fw.channel(name)

    def mcu_channel_at(self, name, tick):
        return self.fw.channel_at(name, tick)

    def mcu_lines(self, kinds=None, since_tick=0):
        return [line for line in self.fw.lines() if line.queued_tick >= since_tick
                and (kinds is None or line.kind in kinds)]

    def host_writes(self, since_ms=0, head=None):
        """[(t_ms, first arrival tick, last arrival tick, bytes, note)] of host -> MCU writes."""
        out = []
        for t_ms, first, last, data, note in self._h2m_log:
            if t_ms < since_ms:
                continue
            if head is not None and data.split(b'*')[0].split(b' ')[0].strip() != head.encode('ascii'):
                continue
            out.append((t_ms, first, last, data, note))
        return out

    def identity(self, include_files=True):
        profile = self.profile
        world = None
        if self.world_kind == 'W0':
            world = dict(kind='W0', markers=[list(m) for m in self.world.config.markers])
        else:
            spec = self.config.world
            world = dict(kind='W2', record=spec.record, corridor_id=spec.corridor_id, direction=spec.direction,
                         modifiers=[dict(m) for m in spec.modifiers])
        out = dict(schema=RIG_SCHEMA, seed=self.seed, stream_key=self.stream_key, world=world,
                   profile_id=profile.profile_id, profile_sha256=profile.sha256,
                   profile_label=profile.result_label().get('label') if hasattr(profile, 'result_label') else None,
                   firmware_build=self.fw.firmware_build, native_build_id=self.fw.build_id,
                   host_profile_sha256=self.hp.sha256, cal_id=self.config.cal_id,
                   host_simulated=self.config.host_simulated,
                   start_pose=[float(v) for v in self.config.start_pose],
                   start_steer_rad=float(self.config.start_steer_rad), curve=self.params.curve,
                   bridge_period_ms=self.config.bridge_period_ms, bridge_phase_ms=self.config.bridge_phase_ms,
                   lidar_hz=self.lidar_hz, imu_rate_hz=self.imu_rate_hz, lidar_latency_s=self.config.lidar_latency_s,
                   imu_latency_s=self.config.imu_latency_s, injections=[dict(i) for i in self.injections],
                   record_overrides=self.config.record_override_dict(), host_overrides=self.config.host_override_dict(),
                   result_kind=RESULT_KIND)
        if include_files:
            out['native_executable_sha256'] = self.fw.executable_sha256
        return out

    # -- lifecycle
    def close(self):
        """End the native process (idempotent)."""
        if self._closed:
            return
        self._closed = True
        try:
            plant = getattr(self, 'plant', None)
            if plant is not None:
                plant.close()
            elif getattr(self, 'link_backend', None) is not None:
                self.link_backend.close()
        finally:
            link = getattr(self, 'link', None)
            if link is not None and link.state != 'closed':
                try:
                    link.close('shutdown')
                except OSError:
                    pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


# --- control-API client handle ---------------------------------------------------------------------------------------
class RigClient:
    """Client of the bridge core in the rig: exactly the operations the executor's BridgeClient uses.

    __call__(request) is the BridgeClient transport protocol (request dict -> reply dict, blocking: the rig advances
    until the bridge answers). The helpers keep owner / lease / ticket / seq like a dashboard or executor client:
    status(), steering(), arm_precheck(), arm(), wait_armed(), drive(throttle, steer_mrad), keepalive(), release(),
    stop(mode='brake' | 'release'), observe_motion(yaw_rate_rad_s, speed_m_s). Every reply is returned as the bridge
    gave it (ok false included); nothing here re-arms by itself.
    The rig itself is private (_rig): an executor that gets this client as its transport reaches the rig only through
    requests, never its evaluation channels (A13; truth_isolation flags rig names in decision code, review RS-05).
    During a bridge_restart fault (no bridge process) a request gets a transport error like a socket client
    (BridgeTransportError, an OSError).
    """

    def __init__(self, rig, owner, expect_cal_id=None):
        if not isinstance(owner, str) or not owner:
            raise RigError('owner must be a non-empty string')
        self._rig = rig
        self.owner = owner
        self.expect_cal_id = expect_cal_id
        self.lease = self.ticket = None
        self.seq = 0
        self.last_reply = None
        self.late_futures = []
        self.counts = collections.Counter()
        if rig.default_client is None:
            rig.default_client = self

    def __call__(self, request):
        return self.request(request)

    def request(self, request, timeout_s=REQUEST_TIMEOUT_S):
        self.counts[str(request.get('op'))] += 1
        reply = self._rig.request(request, timeout_s=timeout_s)
        self.last_reply = reply
        return reply

    def status(self):
        return self.request({'op': 'status'})

    def steering(self):
        """The steering block of the status (design 4.2) or None."""
        return self.status().get('steering')

    def arm_precheck(self, status=None, allow_sim=True):
        """None when the bridge status allows an arm (design 8.3 P4 rules), else the first problem found."""
        status = self.status() if status is None else status
        if not isinstance(status, dict) or status.get('ok') is not True:
            return 'status_not_ok'
        if status.get('firmware') != 'UBTS_STEER_V3' or status.get('protocol') != 3:
            return 'firmware_v3_required'
        if status.get('vehicle_profile') != VEHICLE_PROFILE or status.get('profile') != VEHICLE_PROFILE:
            return 'vehicle_profile_mismatch'
        if status.get('simulated') is not allow_sim:
            return 'sim_identity_mismatch'
        steering = status.get('steering') or {}
        if not steering.get('calibrated'):
            return 'steering_uncalibrated'
        if self.expect_cal_id is not None and status.get('cal_id') != self.expect_cal_id:
            return 'calibration_mismatch'
        faults = [name for name in steering.get('faults') or () if name not in ('BOOT', 'TTL', 'SOFTLIMIT')]
        if 'POLARITY' in faults or 'DRV_STUCK' in faults:
            return 'polarity_check_missing'
        if faults:
            return 'steering_fault:' + ','.join(faults)
        host = status.get('steering_host') or {}
        if host.get('stale', True):
            return 'steering_status_stale'
        if host.get('stop_state') != 'confirmed':
            return 'stop_unconfirmed'
        if status.get('armed') is not False or status.get('state') != 'disarmed':
            return 'bridge_armed_by_other_client'
        if status.get('moving') is not False:
            return 'bridge_reports_moving'
        return None

    def arm(self, precheck=True):
        if precheck:
            problem = self.arm_precheck()
            if problem:
                return {'ok': False, 'error': 'arm_precheck:' + problem}
        reply = self.request({'op': 'arm', 'owner': self.owner, 'vehicle_profile': VEHICLE_PROFILE})
        self._take(reply)
        return reply

    def wait_armed(self, limit_s=.3):
        """Advance until the MCU confirmed the ARM (steering_host arm_state confirmed); False if not or lease lost."""
        core = self._rig.core
        ok = self._rig.run_until_true(lambda: core._arm_confirmed or not core.control.lease, limit_s)
        return bool(ok and core._arm_confirmed and core.control.lease == self.lease)

    def _owned(self, op, **extra):
        if self.lease is None:
            return {'ok': False, 'error': 'client_has_no_lease'}
        request = dict(op=op, owner=self.owner, lease=self.lease, ticket=self.ticket, seq=self.seq + 1, **extra)
        reply = self.request(request)
        self._take(reply)
        return reply

    def _take(self, reply):
        if reply.get('ok') and reply.get('lease'):
            self.lease, self.ticket, self.seq = reply['lease'], reply['ticket'], reply['seq']
        elif not reply.get('ok') and reply.get('error') in ('control_not_owned', 'stale_input_rearm_required',
                                                             'invalid_sequence', 'invalid_drive', 'stop_barrier',
                                                             'differential_command_rejected'):
            self.lease = self.ticket = None

    def drive(self, throttle, steer_mrad):
        return self._owned('drive', deadman=True, throttle=throttle, steer_mrad=steer_mrad)

    def keepalive(self):
        return self._owned('keepalive')

    def release(self):
        return self._owned('release')

    def stop(self, mode='brake', reason='t1_rig_client'):
        """mode 'brake': op stop (MCU STOP: drive brake, steering hold_measured; the lease is revoked).
        mode 'release': op release (throttle 0 with the MCU ramp, the steering target kept; lease kept)."""
        if mode == 'brake':
            reply = self.request({'op': 'stop', 'reason': reason})
            self.lease = self.ticket = None
            return reply
        if mode == 'release':
            return self.release()
        raise RigError('stop mode must be brake or release: %r' % (mode,))

    def observe_motion(self, yaw_rate_rad_s, speed_m_s):
        """Sensor-derived motion sample for the bridge's steering_ineffective monitor (op motion, lease holder)."""
        if self.lease is None:
            return {'ok': False, 'error': 'client_has_no_lease'}
        reply = self.request({'op': 'motion', 'owner': self.owner, 'lease': self.lease,
                              'yaw_rate_rad_s': float(yaw_rate_rad_s), 'speed_m_s': float(speed_m_s)})
        if reply.get('error') == 'control_not_owned':     # the bridge no longer holds our lease (a stop happened)
            self.lease = self.ticket = None
        return reply

    def submit_late_drive_after_stop(self, throttle, steer_mrad):
        """Non-blocking: op stop and, queued right after it, a drive with the lease the STOP revokes (09-19 test
        'late drive after STOP'). The drive's Future is appended to late_futures; the next bridge step answers it."""
        lease, ticket, seq = self.lease, self.ticket, self.seq
        self._rig.submit({'op': 'stop', 'reason': 'late_drive_after_stop'})
        late = {'op': 'drive', 'owner': self.owner, 'lease': lease, 'ticket': ticket, 'seq': seq + 1, 'deadman': True,
                'throttle': throttle, 'steer_mrad': steer_mrad}
        self.late_futures.append(self._rig.submit(late))
        self.lease = self.ticket = None

    def late_drive_after_stop(self, throttle, steer_mrad):
        """Blocking form of submit_late_drive_after_stop(): returns the late drive's reply."""
        self.submit_late_drive_after_stop(throttle, steer_mrad)
        future = self.late_futures[-1]
        self._rig.run_until_true(future.done, REQUEST_TIMEOUT_S)
        return future.result() if future.done() else {'ok': False, 'error': 'not_answered'}
