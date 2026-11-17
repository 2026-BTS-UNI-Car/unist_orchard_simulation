"""Clock-injected host bridge core for the front-steer vehicle (ESP protocol V3, design 4.1-4.2).

Same meaning as the fixed V2 Bridge (scripts/rover_control_bridge.py, not imported, not changed): a queue of 8
requests, request expiry 0.25 s, busy -> stop, status and stop bypass the queue, one request per tick, a status
snapshot after each tick. On top of that:
  * C resend at c_resend_hz while the operator input is valid, ttl = min(c_ttl_max_ms, MCU ttlmax, remaining input
    deadline in ms); a new q for every C line (spec 4.8)
  * PING at ping_hz while the link is up (watchdog only; it never extends motion, spec 4.4)
  * STOP confirmation: a status line with st DISARMED and L = R = 0 within stop_confirm_periods status periods,
    else STOP again every stop_resend_interval_ms (a fixed grid from the first STOP) up to stop_resend_max times,
    then the stop_unconfirmed latch
  * an MCU reset seen only in the status lines (ms back, n restarted) or in an ARM refused with N - boot latches
    mcu_reset_detected, stops and asks for the banner with INFO; arm waits for that banner (the boot id). No INFO /
    CAL GET while a lease is held
  * ARM confirmation by the next status lines (st ARMED); an N line for ARM or no confirmation stops and latches
    (arm_rejected, arm_unconfirmed). A STOP written while an ARM is unconfirmed is confirmed only by a status line
    made at least two status periods after the last one seen (an older line may predate the ARM)
  * host faults latched until an operator re-arm: steering_status_stale, mcu_reset_detected, calibration_mismatch,
    hardware_profile_mismatch, sim_identity_mismatch, steering_not_following, steering_ineffective,
    status_seq_gap, stop_unconfirmed, mcu_disarmed, mcu_state_mismatch, c_rejected, hardware_fault, arm_rejected,
    arm_unconfirmed, internal_error, mcu_ttl_expired. The re-arm clears them only when the MCU confirms the ARM
    (st ARMED); a refused or unconfirmed ARM keeps them
  * C TTL latch (DECISIONS R18 decision 1): an MCU C TTL expiry (an F TTL line, or the TTL bit of an ARMED status
    line) while the host holds a lease and still had a valid operator input when the MCU's last C ran out (C lines
    were lost or late) stops at once and latches mcu_ttl_expired: no further C line until an operator re-arm, so the
    next C can never restart the drive 120 ms after the TTL brake (spec 6.3, 8.2) without the operator. The TTL at the
    end of the host's own input deadline (throttle 0 at standstill: lease kept, no C resend, spec 6.3) is the expected
    end of the command and does not latch
  * transport errors never escape step(): they latch hardware_fault, close the link and reopen it later
    (no automatic re-arm). Any other exception in a step or a request latches internal_error and stops
  * op motion: the lease holder (the executor) gives sensor-derived motion samples (gyro yaw rate, estimated
    speed) for the steering_ineffective monitor; answered at once like status, never a motion input

No ROS, socket, serial, fcntl or termios imports. The clock is injected (time.monotonic on the vehicle, SIM time
in T1 and in the G-SIM SIM clock mode); wall_clock is only used for the updated_at field of the snapshot.
The steering block of the status follows design 4.2. All numbers are placeholders.
"""
import collections
import concurrent.futures
import hashlib
import json
import math
import queue
import threading
import time

from scripts.esp_motor_v3 import FIRMWARE, PROTOCOL, fault_names
from scripts.rover_control_core import ControlError
from scripts.steer_control_core import VEHICLE_PROFILE, SteerControl

HOST_PROFILE_SCHEMA = 'ubts.steer_host_profile.v1'
STEER_STATE_SCHEMA = 'ubts.steer_state.v1'
MOTION_FRESH_S = 0.2          # steering_ineffective needs a motion sample at most this old [proposal]
MOTION_LIMITS = (10.0, 10.0)  # op motion: |yaw_rate_rad_s|, |speed_m_s| at most these (a sanity range) [proposal]

# Host latch names (cleared only by an operator re-arm whose preconditions pass). New names go at the end (the T1 rig
# trace keeps one bit per name in this order).
LATCH_NAMES = ('steering_status_stale', 'mcu_reset_detected', 'calibration_mismatch', 'hardware_profile_mismatch',
               'sim_identity_mismatch', 'steering_not_following', 'steering_ineffective', 'status_seq_gap',
               'stop_unconfirmed', 'mcu_disarmed', 'mcu_state_mismatch', 'c_rejected', 'hardware_fault',
               'arm_rejected', 'arm_unconfirmed', 'internal_error', 'mcu_ttl_expired')
TTL_BIT = 0x0800                       # fl TTL (spec 5.3): set by a C TTL expiry, cleared by the next C or DISARMED
# mcu_ttl_expired (R18 decision 1): the operator input deadline the host had TTL_INPUT_LAG_S before it read the TTL
# event decides whether the expiry was the host's own (deadline passed: no latch) or lost / late C lines (latch). The lag
# keeps a fresh request that the host answered between the MCU's expiry and the read of its F line (line time, USB
# latency, one loop period: about 35 ms at most) from turning the expected deadline TTL into a latch [proposal].
TTL_INPUT_LAG_S = 0.05
# Error codes the steer bridge adds to the RoverControl / Bridge ones (design 4.2 list first).
ERROR_CODES = ('firmware_v3_required', 'steering_uncalibrated', 'calibration_mismatch', 'hardware_profile_mismatch',
               'polarity_check_missing', 'steer_profile_required', 'differential_command_rejected',
               'steering_fault_latched', 'steering_status_stale', 'stop_unconfirmed',
               # added by this implementation (proposed):
               'sim_identity_mismatch', 'link_config_mismatch', 'hardware_fault', 'config_unsaved', 'internal_error',
               'invalid_motion')
# N reasons that only an ARM line can get from the bridge's own traffic -> stop reason.
ARM_REJECT_STOP_REASON = {'boot': 'mcu_reset_detected', 'nocal': 'steering_uncalibrated',
                          'cal': 'calibration_mismatch', 'pol': 'polarity_check_missing',
                          'fault': 'steering_fault_latched', 'busy': 'steering_busy', 'state': 'arm_rejected_state',
                          'syntax': 'arm_rejected_syntax', 'nocrc': 'arm_rejected_nocrc', 'cmd': 'arm_rejected_cmd'}
POLARITY_LATCH_BITS = 0x0020 | 0x0040   # fl POLARITY, DRV_STUCK (spec 5.3)
STEERING_BLOCK_KEYS = ('measured_mrad', 'measured_permille', 'raw', 'target_permille', 'cmd_permille', 'pwm',
                       'mcu_ms', 'seq', 'applied_q', 'age_ms', 'ttl_remaining_ms', 'drive_en', 'steer_en',
                       'hold_remaining_ms', 'stop_mode', 'faults', 'servo_state', 'saturated', 'calibrated')


class HostProfileError(ValueError):
    pass


def round_half_away(x):
    """Round half away from zero (spec 7.3 rounding) to int."""
    return int(math.floor(abs(x) + 0.5)) * (1 if x >= 0 else -1)


def duty_from_throttle(throttle):
    """Signed drive duty for a throttle percent, the V2 rule (pct * 255 + 50) // 100 (motor_pwm derived PWM)."""
    if type(throttle) is not int or not -100 <= throttle <= 100:
        raise ValueError('throttle_must_be_int_-100..100')
    duty = (abs(throttle) * 255 + 50) // 100
    return duty if throttle >= 0 else -duty


# --- host profile ----------------------------------------------------------------------------------------------------
_NUM = (int, float)
# section -> key -> (types, lo, hi); lo/hi None = no bound; a tuple of allowed values instead of bounds for strings.
_HOST_PROFILE_SPEC = {
    'steer_limit': {'left_mrad': (int, 1, 1570), 'right_mrad': (int, 1, 1570), 'left_rad': (_NUM, 0.0, 1.5708),
                    'right_rad': (_NUM, 0.0, 1.5708)},
    'calibration': {'cal_id': ('cal_id', 1, 65535), 'mark_left_mrad': (_NUM, 1.0, 1570.0),
                    'mark_right_mrad': (_NUM, 1.0, 1570.0), 'permille_per_deg': (_NUM, 1.0, 1000.0),
                    'expected_soft_permille': ('soft', None, None), 'rule': (str, None, None)},
    'link': {'baud': (int, 115200, 230400), 'status_hz': (int, 10, 50), 'c_ttl_max_ms': (int, 50, 1000),
             'c_resend_hz': (_NUM, 1.0, 100.0), 'ping_hz': (_NUM, 1.0, 100.0), 'watchdog_ms': (int, 100, 5000),
             'banner_timeout_s': (_NUM, 0.1, 10.0)},
    'stop': {'confirm_periods': (int, 1, 10), 'resend_interval_ms': (_NUM, 1.0, 1000.0), 'resend_max': (int, 1, 20),
             'steer_policy': (str, ('hold_measured', 'hold_target', 'center'), None),
             'hold_after_stop_ms': (int, 0, 10000), 'hold_after_disarm_ms': (int, 0, 10000),
             'hold_after_loss_ms': (int, 0, 10000), 'watchdog_steer_mode': (str, ('hold', 'center', 'free'), None),
             'ttl_expiry_drive_mode': (str, ('brake', 'release'), None)},
    'arm': {'confirm_ms': (_NUM, 10.0, 5000.0)},
    'host_faults': {'status_gap_ms': (_NUM, 10.0, 10000.0), 'seq_gap_missing': (int, 0, 1000),
                    'seq_gap_window_s': (_NUM, 0.1, 60.0), 'track_error_deg': (_NUM, 0.1, 90.0),
                    'track_duration_s': (_NUM, 0.01, 60.0), 'ineffective': ('ineffective', None, None)},
    'servo': {'deadband_permille': (int, 0, 1000), 'max_pwm': (int, 1, 255)},
    'drive': {'lr_mode': (str, ('equal_duty',), None)},
    'vehicle': {'wheelbase_m': (_NUM, 0.1, 5.0)},
}
# Optional keys (absent = the value in brackets; an older host profile keeps its bytes): host_faults.track_rule
# ['v3_ramp_gate'] 'esp32_adapter' = steering_not_following as the vehicle's ESP32 stack counts it (the executor's
# Esp32AbsHoldAdapter: |sent target - measured| without the MCU ramp gate; vehicle profile steer_monitor_stack esp32_r5,
# docs/steering/SMOOTH_R5_DISTURBANCE_KO.md).
TRACK_RULES = ('v3_ramp_gate', 'esp32_adapter')
_HOST_PROFILE_OPTIONAL = {'host_faults': {'track_rule': (str, TRACK_RULES, None)}}
_INEFFECTIVE_KEYS = ('min_speed_m_s', 'min_steer_deg', 'rate_error_deg_s', 'rate_error_frac', 'duration_s')
_TOP_KEYS = ('schema', 'vehicle_profile', 'hw_profile', 'source_profile', 'provenance') + tuple(_HOST_PROFILE_SPEC)
_SOURCE_KEYS = ('profile_id', 'profile_sha256', 'provenance_grade', 'result_kind', 'field_validated', 'overrides')


def _is_num(value, types):
    return isinstance(value, types) and not isinstance(value, bool) and (not isinstance(value, float)
                                                                         or math.isfinite(value))


def _check_value(where, value, spec):
    types, lo, hi = spec
    if types == 'cal_id':
        if value is not None and not (type(value) is int and 1 <= value <= 65535):
            raise HostProfileError('%s must be null or an int 1..65535' % where)
        return
    if types == 'soft':
        if not isinstance(value, dict) or set(value) != {'left', 'right'} or not (
                type(value['left']) is int and 1 <= value['left'] <= 1000 and type(value['right']) is int
                and -1000 <= value['right'] <= -1):
            raise HostProfileError('%s must be {"left": 1..1000, "right": -1000..-1}' % where)
        return
    if types == 'ineffective':
        if not isinstance(value, dict) or set(value) != set(_INEFFECTIVE_KEYS) or not all(
                _is_num(value[k], _NUM) and value[k] >= 0 for k in _INEFFECTIVE_KEYS) or value['duration_s'] <= 0:
            raise HostProfileError('%s must have %s (non-negative numbers)' % (where, ', '.join(_INEFFECTIVE_KEYS)))
        return
    if types is str:
        if not isinstance(value, str) or not value or (lo is not None and value not in lo):
            raise HostProfileError('%s must be one of %s' % (where, lo) if lo else '%s must be a string' % where)
        return
    if types is int:
        if type(value) is not int:
            raise HostProfileError('%s must be an int' % where)
    elif not _is_num(value, types):
        raise HostProfileError('%s must be a finite number' % where)
    if lo is not None and not lo <= value <= hi:
        raise HostProfileError('%s=%r outside %r..%r' % (where, value, lo, hi))


def _reject_constant(name):
    raise HostProfileError('host profile JSON must not contain %s' % name)


def _check_timing_relations(data):
    """Values that pass one by one but not together (spec HOST_BRIDGE 6): a PING at least twice per MCU watchdog
    window, a C resend before the C ttl ends, a status gap and an ARM confirmation of at least two status periods."""
    link, faults, arm = data['link'], data['host_faults'], data['arm']
    status_period_ms = 1000.0 / link['status_hz']
    for ok, text in (
            (1000.0 / link['ping_hz'] <= link['watchdog_ms'] / 2.0,
             'link.ping_hz: the PING period must be at most half of link.watchdog_ms'),
            (1000.0 / link['c_resend_hz'] < link['c_ttl_max_ms'],
             'link.c_resend_hz: the C resend period must be shorter than link.c_ttl_max_ms'),
            (faults['status_gap_ms'] >= 2.0 * status_period_ms,
             'host_faults.status_gap_ms must be at least two status periods (1000 / link.status_hz ms)'),
            (arm['confirm_ms'] >= 2.0 * status_period_ms,
             'arm.confirm_ms must be at least two status periods (1000 / link.status_hz ms)')):
        if not ok:
            raise HostProfileError(text)


class HostProfile:
    """Validated host profile JSON (schema ubts.steer_host_profile.v1, written by
    sim/ubts/steering/host_profile_export.py from a vehicle profile; the host never imports sim code)."""

    def __init__(self, data, sha256=None, origin='<memory>'):
        if not isinstance(data, dict):
            raise HostProfileError('host profile must be a JSON object')
        keys = set(data)
        if keys != set(_TOP_KEYS):
            missing, unknown = sorted(set(_TOP_KEYS) - keys), sorted(keys - set(_TOP_KEYS))
            raise HostProfileError('host profile keys: missing %s, unknown %s' % (missing, unknown))
        if data['schema'] != HOST_PROFILE_SCHEMA:
            raise HostProfileError('schema %r (need %r)' % (data['schema'], HOST_PROFILE_SCHEMA))
        if data['vehicle_profile'] != VEHICLE_PROFILE:
            raise HostProfileError('vehicle_profile %r (this bridge is for %r)' % (data['vehicle_profile'],
                                                                                 VEHICLE_PROFILE))
        if not isinstance(data['hw_profile'], str) or not data['hw_profile']:
            raise HostProfileError('hw_profile must be a non-empty string')
        source = data['source_profile']
        if not isinstance(source, dict) or set(source) != set(_SOURCE_KEYS):
            raise HostProfileError('source_profile needs exactly %s' % (_SOURCE_KEYS,))
        if not isinstance(source['profile_id'], str) or not isinstance(source['profile_sha256'], str) or len(
                source['profile_sha256']) != 64:
            raise HostProfileError('source_profile profile_id / profile_sha256 malformed')
        if not isinstance(data['provenance'], dict):
            raise HostProfileError('provenance must be an object')
        for section, spec in _HOST_PROFILE_SPEC.items():
            block = data[section]
            optional = _HOST_PROFILE_OPTIONAL.get(section, {})
            if not isinstance(block, dict) or not set(spec) <= set(block) <= set(spec) | set(optional):
                raise HostProfileError('%s needs exactly %s' % (section, sorted(spec)) + (
                    ' (optional %s)' % sorted(optional) if optional else ''))
            for key, rule in spec.items():
                _check_value('%s.%s' % (section, key), block[key], rule)
            for key, rule in optional.items():
                if key in block:
                    _check_value('%s.%s' % (section, key), block[key], rule)
        _check_timing_relations(data)
        self.data = data
        self.sha256 = sha256
        self.origin = origin
        lim, cal, link, stop = data['steer_limit'], data['calibration'], data['link'], data['stop']
        self.vehicle_profile = data['vehicle_profile']
        self.hw_profile = data['hw_profile']
        self.cal_id = cal['cal_id']
        self.limit_left_mrad, self.limit_right_mrad = lim['left_mrad'], lim['right_mrad']
        self.mark_left_mrad, self.mark_right_mrad = float(cal['mark_left_mrad']), float(cal['mark_right_mrad'])
        if lim['left_mrad'] > lim['left_rad'] * 1000.0 + 1e-6 or lim['right_mrad'] > lim['right_rad'] * 1000.0 + 1e-6:
            raise HostProfileError('steer_limit mrad must not exceed the exact limit')
        if self.permille_from_mrad(lim['left_mrad']) > 1000 or self.permille_from_mrad(-lim['right_mrad']) < -1000:
            raise HostProfileError('steer limit beyond the calibration marks (|S| > 1000)')
        self.expected_soft = (cal['expected_soft_permille']['left'], cal['expected_soft_permille']['right'])
        self.status_hz = link['status_hz']
        self.status_period_s = 1.0 / link['status_hz']
        self.c_ttl_max_ms = link['c_ttl_max_ms']
        self.c_resend_period_s = 1.0 / link['c_resend_hz']
        self.ping_period_s = 1.0 / link['ping_hz']
        self.watchdog_ms = link['watchdog_ms']
        self.banner_timeout_s = float(link['banner_timeout_s'])
        self.baud = link['baud']
        self.stop_confirm_s = stop['confirm_periods'] * self.status_period_s
        self.stop_resend_interval_s = stop['resend_interval_ms'] / 1000.0
        self.stop_resend_max = stop['resend_max']
        self.steer_policy = stop['steer_policy']
        self.hold_s = {'stop': stop['hold_after_stop_ms'] / 1000.0, 'disarm': stop['hold_after_disarm_ms'] / 1000.0,
                       'loss': stop['hold_after_loss_ms'] / 1000.0}
        self.watchdog_steer_mode = stop['watchdog_steer_mode']
        self.ttl_expiry_drive_mode = stop['ttl_expiry_drive_mode']
        self.arm_confirm_s = data['arm']['confirm_ms'] / 1000.0
        faults = data['host_faults']
        self.status_gap_s = faults['status_gap_ms'] / 1000.0
        self.seq_gap_missing, self.seq_gap_window_s = faults['seq_gap_missing'], float(faults['seq_gap_window_s'])
        self.track_error_mrad = math.radians(faults['track_error_deg']) * 1000.0
        self.track_duration_s = float(faults['track_duration_s'])
        self.track_rule = faults.get('track_rule', 'v3_ramp_gate')
        self.ineffective = dict(faults['ineffective'])
        self.deadband_permille = data['servo']['deadband_permille']
        self.servo_max_pwm = data['servo']['max_pwm']
        self.wheelbase_m = float(data['vehicle']['wheelbase_m'])

    @classmethod
    def from_bytes(cls, raw, origin='<memory>'):
        raw = bytes(raw)
        try:
            data = json.loads(raw.decode('utf-8'), parse_constant=_reject_constant)
        except (UnicodeDecodeError, ValueError) as exc:
            if isinstance(exc, HostProfileError):
                raise
            raise HostProfileError('host profile is not UTF-8 JSON: %s' % exc) from None
        return cls(data, sha256=hashlib.sha256(raw).hexdigest(), origin=origin)

    @classmethod
    def load(cls, path):
        with open(path, 'rb') as handle:
            return cls.from_bytes(handle.read(), origin=str(path))

    def permille_from_mrad(self, mrad):
        """Bicycle equivalent angle (mrad, left +) -> MCU permille (calibration marks, linear per side)."""
        if mrad >= 0:
            return round_half_away(1000.0 * mrad / self.mark_left_mrad)
        return -round_half_away(1000.0 * -mrad / self.mark_right_mrad)

    def mrad_from_permille(self, permille):
        """MCU position permille -> bicycle equivalent angle in mrad (int). Pot reading, not the tyre angle."""
        if permille >= 0:
            return round_half_away(permille * self.mark_left_mrad / 1000.0)
        return -round_half_away(-permille * self.mark_right_mrad / 1000.0)

    def identity(self):
        source = self.data['source_profile']
        return dict(host_profile_sha256=self.sha256, profile_id=source['profile_id'],
                    profile_sha256=source['profile_sha256'], provenance_grade=source['provenance_grade'],
                    result_kind=source['result_kind'], field_validated=source['field_validated'], origin=self.origin)


# --- motor adapter: the RoverControl motor interface on top of the V3 link ---------------------------------------
class V3MotorAdapter:
    """What SteerControl calls. It never writes C lines itself; the core derives them from the control state."""

    def __init__(self, core):
        self.core = core

    @property
    def ready(self):
        return self.core.link.ready

    @property
    def stopped(self):
        core = self.core
        s, t = core.link.status, core.link.status_t
        return s is not None and core.clock() - t <= core.hp.status_gap_s and s['L'] == 0 and s['R'] == 0

    @property
    def firmware(self):
        return FIRMWARE if self.core.link.handshake_done else None

    @property
    def outputs(self):
        return self.core.pin_outputs()

    @property
    def error(self):
        core = self.core
        if core.last_error:
            return core.last_error
        return next(iter(core.latches), None)

    def hard_stop(self, for_arm=False):
        self.core._hard_stop(for_arm)

    def enable(self):
        self.core._enable()

    def release(self):
        self.core.action_log.append('release')
        self.core._command_dirty = True

    def drive_steer(self, throttle, steer_mrad):
        self.core.action_log.append(('drive', throttle, steer_mrad))
        self.core._command_dirty = True

    def drive(self, left, right):
        raise RuntimeError('differential_command_rejected')

    def arm_blocker(self):
        return self.core._arm_blocker()

    def clear_latches(self):
        self.core._clear_latches()


# --- the core --------------------------------------------------------------------------------------------------------
class SteerBridgeCore:
    """Bridge core. Threaded use (wrapper): request() from socket threads, step() every 20 ms in the loop thread.
    Single-thread use (T1): submit() returns a Future that the next step() resolves."""

    QUEUE_SIZE = 8
    REQUEST_EXPIRY_S = 0.25
    REQUEST_TIMEOUT_S = 0.4
    REOPEN_INTERVAL_S = 1.0
    CAL_QUERY_INTERVAL_S = 1.0
    # The calibration line follows the banner (about 11 ms at 115200 baud): ask with CAL GET only when it is still
    # missing this long after the banner [proposal; spec 12.1-60].
    CAL_QUERY_GRACE_S = 0.1
    TIME_EPS_S = 1e-6           # float slack of the scheduled times (STOP resend grid)

    def __init__(self, link, host_profile, clock=time.monotonic, wall_clock=time.time, auto_reopen=True):
        self.link = link
        self.hp = host_profile
        self.clock = clock
        self.wall_clock = wall_clock
        self.auto_reopen = auto_reopen
        if link.expect_hw != host_profile.hw_profile:
            raise ValueError('link expect_hw differs from the host profile hw_profile')
        self.motor = V3MotorAdapter(self)
        self.control = SteerControl(self.motor, clock, steer_limit_left_mrad=host_profile.limit_left_mrad,
                                    steer_limit_right_mrad=host_profile.limit_right_mrad,
                                    status_extra=self._status_extra)
        self.pending = queue.Queue(maxsize=self.QUEUE_SIZE)
        self.emergency = threading.Event()
        self.latches = collections.OrderedDict()
        self.action_log = collections.deque(maxlen=512)
        self.mcu_events = collections.deque(maxlen=32)
        self.counters = collections.Counter()
        self.last_error = None
        self.snapshot = {'ok': True, 'connected': False, 'state': 'starting'}
        self._stop = None
        self._stop_confirmed = False
        self._arm_pending_t = None
        self._arm_confirmed = False
        self._command_dirty = False
        self._next_c_due = None
        self._last_c = None
        self._next_ping_due = None
        self._status_stale = False
        self._seq_window = collections.deque()
        self._track_since = None
        self._ineffective_since = None
        self._motion = None
        self._hold = None
        self._stop_mode = None
        self._next_reopen = None
        self._last_cal_query = None
        self._banner_wanted = False   # an MCU reset was seen in the status lines, the new banner is not here yet
        self._next_info_t = None
        self._clear_on_confirm = False   # an operator re-arm clears the latches when the MCU confirms the ARM
        # (poll time, operator input deadline or None without a lease) of the recent link polls: the reference of the
        # mcu_ttl_expired rule (R18 decision 1, _ttl_event)
        self._input_history = collections.deque(maxlen=16)

    # -- requests (same rules as the V2 Bridge)
    def submit(self, request):
        future = concurrent.futures.Future()
        if not isinstance(request, dict):
            future.set_result({'ok': False, 'error': 'object_required'})
            return future
        op = request.get('op')
        if op is not None and not isinstance(op, str):
            future.set_result({'ok': False, 'error': 'unknown_operation'})
            return future
        if op == 'status':
            future.set_result(dict(self.snapshot))
            return future
        if op == 'stop':
            self.emergency.set()
            future.set_result({'ok': True, 'state': 'stop_requested'})
            return future
        if op == 'motion':
            future.set_result(self._motion_request(request))
            return future
        try:
            self.pending.put_nowait((self.clock(), request, future))
        except queue.Full:
            self.emergency.set()
            future.set_result({'ok': False, 'error': 'control_busy_stopped'})
        return future

    def request(self, request, timeout=None):
        future = self.submit(request)
        try:
            return future.result(timeout=self.REQUEST_TIMEOUT_S if timeout is None else timeout)
        except concurrent.futures.TimeoutError:
            future.cancel()
            self.emergency.set()
            return {'ok': False, 'error': 'control_busy_stopped'}

    def observe_motion(self, yaw_rate_rad_s, speed_m_s):
        """Sensor-derived motion (gyro yaw rate, LiDAR speed) for steering_ineffective. Never plant truth."""
        for value in (yaw_rate_rad_s, speed_m_s):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('motion values must be finite numbers')
        self._motion = (self.clock(), float(yaw_rate_rad_s), float(speed_m_s))

    def _motion_request(self, request):
        """op motion {owner, lease, yaw_rate_rad_s, speed_m_s}: a motion sample of the lease holder (the executor's
        gyro yaw rate and its sensor speed estimate) for steering_ineffective. Answered at once (no queue, no ticket,
        no seq): it is an observation, never a motion input, and it does not extend any deadline. Another client's
        sample is refused with control_not_owned (nothing stops); a value outside MOTION_LIMITS or not a finite
        number with invalid_motion."""
        control = self.control
        lease = control.lease
        if not lease or request.get('owner') != control.owner or request.get('lease') != lease:
            self.counters['motion_refused'] += 1
            return {'ok': False, 'error': 'control_not_owned'}
        yaw, speed = request.get('yaw_rate_rad_s'), request.get('speed_m_s')
        for value, limit in zip((yaw, speed), MOTION_LIMITS):
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                    or abs(value) > limit):
                self.counters['motion_invalid'] += 1
                return {'ok': False, 'error': 'invalid_motion'}
        self.observe_motion(yaw, speed)
        self.counters['motion_samples'] += 1
        return {'ok': True}

    # -- loop
    def open(self):
        try:
            self.link.open()
        except (OSError, RuntimeError) as exc:
            self.last_error = 'open failed: %s' % exc
            self._next_reopen = self.clock() + self.REOPEN_INTERVAL_S
            return False
        self._session_reset()
        return True

    def close(self, reason='shutdown'):
        self._call(self.control.stop, reason)
        try:
            self.link.close()
        except OSError:
            pass

    def step(self):
        """One loop tick. Never raises: an unexpected exception latches internal_error and stops."""
        try:
            self._step()
        except Exception as exc:   # a defect, not a transport error (those are handled inside)
            self._internal_error('step', exc)
            try:
                self.snapshot = self._build_snapshot()
            except Exception as again:
                self.counters['snapshot_errors'] += 1
                self.last_error = 'internal_error: snapshot: %s: %s' % (type(again).__name__, again)

    def _step(self):
        if self.emergency.is_set():
            self._call(self.control.stop, 'operator_stop')
            while not self.pending.empty():
                _, _, future = self.pending.get_nowait()
                if not future.done():
                    future.set_result({'ok': False, 'error': 'stop_barrier'})
            self.emergency.clear()
        if self.link.state == 'closed' and self.auto_reopen and (
                self._next_reopen is None or self.clock() >= self._next_reopen):
            self.open()
        self._call(self.control.tick)
        self._poll_link()
        self._call(self.control.tick)
        # One bounded request per tick, after stop, expiry and serial feedback.
        if not self.pending.empty():
            received, request, future = self.pending.get_nowait()
            if not future.cancelled():
                if self.clock() - received > self.REQUEST_EXPIRY_S:
                    result = {'ok': False, 'error': 'request_expired'}
                else:
                    result = self._dispatch(request)
                try:
                    future.set_result(result)
                except concurrent.futures.InvalidStateError:   # cancelled by the socket thread after 0.4 s
                    self.counters['answers_after_cancel'] += 1
        self._drive_link(self.clock())
        self.snapshot = self._build_snapshot()

    def _dispatch(self, request):
        try:
            return self.control.dispatch(request)
        except (ControlError, ValueError) as exc:
            return {'ok': False, 'error': str(exc)}
        except (OSError, RuntimeError) as exc:
            self._link_failure(exc)
            return {'ok': False, 'error': 'hardware_fault'}
        except Exception as exc:   # a defect: stop and answer, never leave the loop
            return self._internal_error('dispatch', exc)

    def _internal_error(self, where, exc):
        """An exception that is not a transport error (a defect): latch internal_error, stop, keep running."""
        detail = '%s: %s: %s' % (where, type(exc).__name__, exc)
        self.counters['internal_errors'] += 1
        self.last_error = 'internal_error: ' + detail
        self._latch('internal_error', detail[:200])
        self._arm_pending_t = None
        self._clear_on_confirm = False
        try:
            self._assert_stop('internal_error')
        except Exception:          # the stop path failed too: stop again at the start of the next step
            self.counters['internal_stop_errors'] += 1
            self.emergency.set()
        return {'ok': False, 'error': 'internal_error'}

    def _call(self, fn, *args):
        try:
            fn(*args)
        except (OSError, RuntimeError) as exc:
            self._link_failure(exc)

    def _send(self, fn, *args):
        """Write through the link; transport errors latch hardware_fault and close the link (never raise)."""
        if self.link.state == 'closed':
            return None
        try:
            return fn(*args)
        except OSError as exc:
            self._link_failure(exc)
            return None

    def _link_failure(self, exc):
        self.last_error = 'hardware_fault: %s' % (exc,)
        self._latch('hardware_fault', str(exc))
        if self.link.state != 'closed':
            try:
                self.link.close('hardware_fault')
            except OSError:
                pass
        self._arm_pending_t = None
        self._arm_confirmed = False
        self._clear_on_confirm = False
        if self.control.lease:
            self.control.stop('hardware_fault')   # the adapter sees the closed link and writes nothing
        self._next_reopen = self.clock() + self.REOPEN_INTERVAL_S

    def _latch(self, name, detail=None):
        entry = self.latches.get(name)
        if entry is None:
            self.latches[name] = {'since': self.clock(), 'detail': detail, 'count': 1}
        else:
            entry['count'] += 1
            entry['detail'] = detail
        self.counters['latch_' + name] += 1

    def _clear_latches(self):
        """Operator re-arm (all arm conditions passed): the latches go when the MCU confirms the ARM (first status
        line with st ARMED). An N line, no confirmation or a write error keeps them (spec 12.1-75)."""
        self._clear_on_confirm = True
        self.last_error = None

    def _stop_control(self, reason):
        self._call(self.control.stop, reason)

    def _assert_stop(self, reason):
        """Stop with a lease (revokes it, reason recorded); without a lease only STOP + confirmation, so the
        control reason (e.g. startup, operator_stop) is not overwritten by a housekeeping STOP."""
        if self.control.lease:
            self._stop_control(reason)
        else:
            self._call(self._hard_stop, False)

    def _session_reset(self):
        self._stop = None
        self._stop_confirmed = False
        self._arm_pending_t = None
        self._arm_confirmed = False
        self._command_dirty = False
        self._next_c_due = None
        self._next_ping_due = None
        self._status_stale = False
        self._seq_window.clear()
        self._track_since = self._ineffective_since = None
        self._hold = None
        self._last_cal_query = None
        self._banner_wanted = False
        self._next_info_t = None
        self._clear_on_confirm = False
        self._input_history.clear()

    def _want_banner(self, now):
        """After an MCU reset seen only in the status lines (ms went back, n restarted), the boot banner may have
        been lost: a partial line before the reset or boot garbage without LF merges with it on the host. Ask with
        INFO now and every banner_timeout_s until a banner arrives; arm is refused until then (spec 12.1-61)."""
        if not self._banner_wanted:
            self._banner_wanted = True
            self._next_info_t = now

    # -- motor adapter actions
    def _hard_stop(self, for_arm):
        self.action_log.append('STOP')
        now = self.clock()
        arm_in_flight = self._arm_pending_t is not None
        self._arm_pending_t = None
        self._arm_confirmed = False
        self._command_dirty = False
        self._next_c_due = None
        status = self.link.status
        if not for_arm:
            self._clear_on_confirm = False   # a STOP ends a pending operator re-arm: its latches stay
            # A STOP while one is still unconfirmed keeps the first t0 (the latch deadline does not move).
            if self._stop is None:
                self._stop = {'t0': now, 'last': now, 'resends': 0, 'given_up': False, 'min_n': None}
            if arm_in_flight and status is not None:
                # The ARM may not have reached the MCU yet: a line made before it shows DISARMED and zero duty too.
                # Only a line two status periods after the last one seen confirms this STOP (spec 12.1-77).
                self._stop['min_n'] = (status['n'] + 2) % 65536
            self._stop_confirmed = False
            if status is not None and status['st'] == 2:
                self._hold = (now, self.hp.hold_s['stop'])
            self._stop_mode = 'brake'
        self.counters['stop_sent'] += 1
        self._send(self.link.send_stop)

    def _enable(self):
        self.action_log.append('enable')
        self.link.send_arm()            # errors propagate: arm fails before a lease is granted
        self._arm_pending_t = self.clock()
        self._arm_confirmed = False
        self.counters['arm_sent'] += 1

    def _arm_blocker(self):
        link, hp = self.link, self.hp
        now = self.clock()
        if not link.is_open or not link.handshake_done or link.banner is None:
            return 'firmware_v3_required'
        if link.identity_error:
            return link.identity_error
        if self._banner_wanted:
            return 'mcu_reset_detected'   # the boot id of the running MCU is not known yet
        banner = link.banner
        if banner['hw'] != hp.hw_profile:
            return 'hardware_profile_mismatch'
        if (banner['wd'], banner['ttl'], banner['st']) != (hp.watchdog_ms, hp.c_ttl_max_ms, hp.status_hz):
            return 'link_config_mismatch'
        if link.status is None or now - link.status_t > hp.status_gap_s:
            return 'steering_status_stale'
        status = link.status
        if self._stop is not None or status['st'] != 1 or status['L'] or status['R']:
            return 'stop_unconfirmed'
        cal = link.cal
        if cal is None or cal['cal_id'] is None or banner['cal_id'] is None or None in (
                cal['left'], cal['center'], cal['right']) or not cal['soft_set']:
            return 'steering_uncalibrated'
        if cal['cal_id'] != banner['cal_id'] or cal['caldirty'] or (hp.cal_id is not None and cal['cal_id'] != hp.cal_id):
            return 'calibration_mismatch'
        if cal['cfgdirty']:
            # CFG values in MCU RAM that differ from the stored record (thresholds, watchdog, ...): the MCU would ARM,
            # the vehicle path does not (spec 12.1-74). The bridge sends no CFG and holds the port, so the cal line
            # of this session shows them.
            return 'config_unsaved'
        if not cal['pol_ok'] or status['fl'] & POLARITY_LATCH_BITS:
            # a POLARITY / DRV_STUCK latch needs a passing CAL POL before the MCU accepts ARM (spec 12.1-71)
            return 'polarity_check_missing'
        return None

    # -- receive side
    def _poll_link(self):
        link = self.link
        if link.state == 'closed':
            return
        # the operator input deadline as this poll sees it, before this step answers a request (_ttl_event)
        control = self.control
        self._input_history.append((self.clock(), control.input_until if control.lease else None))
        for event in link.poll():
            kind = event['type']
            if kind == 'link_error':
                self._link_failure(OSError(event['detail']))
                return
            handler = getattr(self, '_on_' + kind, None)
            if handler is not None:
                handler(event)
            else:
                self.counters['events_' + kind] += 1
        self._check_timers(self.clock())

    def _on_banner(self, event):
        line, link = event['line'], self.link
        self._banner_wanted = False
        self._next_info_t = None
        if event['new_session']:
            self.counters['sessions'] += 1
            if self.control.lease:
                self._stop_control('link_reopened')
            self._session_reset()
            self._next_ping_due = event['t']
        elif event['boot_changed']:
            self._latch('mcu_reset_detected', 'new boot id %s' % line['boot'])
            if self.control.lease:
                self._stop_control('mcu_reset_detected')
            self._session_reset()
            self._next_ping_due = event['t']
        if event['identity']:
            self._latch(event['identity'], 'banner sim=%d' % int(line['sim']))
            if self.control.lease:
                self._stop_control(event['identity'])
            return
        if link.hw_mismatch:
            self._latch('hardware_profile_mismatch', 'banner hw=%s' % line['hw'])
            if self.control.lease:
                self._stop_control('hardware_profile_mismatch')
        if hp_cal_mismatch(self.hp, line['cal_id']):
            self._latch('calibration_mismatch', 'banner cal=%s' % line['cal_id'])
            if self.control.lease:
                self._stop_control('calibration_mismatch')
        if event['new_session'] or event['boot_changed']:
            # Initial safe state of every session and boot: STOP, confirmed by the status lines.
            self._call(self._hard_stop, False)

    def _on_cal(self, event):
        line = event['line']
        if hp_cal_mismatch(self.hp, line['cal_id']):
            self._latch('calibration_mismatch', 'cal id=%s' % line['cal_id'])
            if self.control.lease:
                self._stop_control('calibration_mismatch')

    def _on_status(self, event):
        s, t = event['line'], event['t']
        if event['ms_regressed'] or event['n_bad']:
            # A new boot restarts ms and n (spec 5.3); its banner may have been lost on the host side.
            if event['n_bad']:
                self.counters['status_n_anomaly'] += 1
                self._seq_window.clear()
            what = ('status ms went back to %d' % s['ms'] if event['ms_regressed'] else
                    'status n %d does not follow the previous line' % s['n'])
            self._latch('mcu_reset_detected', what)
            self._assert_stop('mcu_reset_detected')
            self._want_banner(t)
        if event['n_missing']:
            self.counters['status_missing'] += event['n_missing']
            self._seq_window.append((t, event['n_missing']))
        while self._seq_window and t - self._seq_window[0][0] > self.hp.seq_gap_window_s:
            self._seq_window.popleft()
        missing = sum(n for _, n in self._seq_window)
        if missing > self.hp.seq_gap_missing:
            self._seq_window.clear()
            self._latch('status_seq_gap', '%d status lines missing within %.1f s' % (missing,
                                                                                     self.hp.seq_gap_window_s))
            if self.control.lease:
                self._stop_control('status_seq_gap')
        # A line read in the step that wrote the STOP may confirm it: DISARMED with zero duty is already the state
        # the STOP makes, and only an ARM (never sent before the confirmation) can leave it (spec 12.1-62). After a
        # STOP written while an ARM was unconfirmed, only a line two periods later counts (12.1-77); a new boot
        # (ms back, n restarted) is DISARMED with zero duty and counts at once.
        if self._stop is not None and s['st'] == 1 and s['L'] == 0 and s['R'] == 0 and (
                event['ms_regressed'] or event['n_bad'] or self._stop_line_late_enough(s['n'])):
            self._stop = None
            self._stop_confirmed = True
            self.counters['stop_confirmed'] += 1
        if self._arm_pending_t is not None:
            if s['st'] == 2:
                self._arm_pending_t = None
                self._arm_confirmed = True
                self._hold = None
                self._command_dirty = True
                self.counters['arm_confirmed'] += 1
                if self._clear_on_confirm:      # the operator re-arm is complete: now the latches go (12.1-75)
                    self._clear_on_confirm = False
                    self.latches.clear()
        elif self._arm_confirmed and self.control.lease and s['st'] != 2:
            self._latch('mcu_disarmed', self._recent_mcu_cause())
            self._stop_control('mcu_disarmed')
        if s['st'] != 1 and not self.control.lease and self._arm_pending_t is None and self._stop is None:
            self._latch('mcu_state_mismatch', 'MCU %s without a host lease' % s['state'])
            self._assert_stop('mcu_state_mismatch')
        if s['st'] == 2 and s['fl'] & TTL_BIT:
            # the F TTL line itself may be lost; the bit stays in the ARMED status lines until a C reaches the MCU
            self._ttl_event(t, 'status line TTL bit', count=False)
        if self._hold is not None and not s['en'] & 2 and t > self._hold[0]:
            self._hold = None   # the MCU ended the HOLD (steering EN 0)
        self._track_monitor(t, s)
        self._ineffective_monitor(t, s)

    def _stop_line_late_enough(self, n):
        floor = self._stop.get('min_n')
        return floor is None or (n - floor) % 65536 <= 32767

    def _on_reject(self, event):
        line = event['line']
        self.counters['rejects_' + line['reason']] += 1
        self.mcu_events.append(dict(t=event['t'], kind='N', q=line['q'], reason=line['reason']))
        if line['q'] is None:
            if self._arm_pending_t is not None and line['reason'] in ARM_REJECT_STOP_REASON:
                reason = line['reason']
                self._arm_pending_t = None
                self._clear_on_confirm = False
                self._latch('arm_rejected', 'ARM refused: N - %s' % reason)      # the old latches stay (12.1-75)
                if reason == 'boot':
                    # The MCU booted again and the banner with the new boot id did not reach the host (it merged
                    # with a cut line or boot garbage): ask for it with INFO; arm waits for it (spec 12.1-76).
                    self._latch('mcu_reset_detected', 'ARM refused with N - boot: boot id %s is stale'
                                % self.link.boot_id)
                    self._want_banner(event['t'])
                self.last_error = 'arm_rejected:' + reason
                self._stop_control(ARM_REJECT_STOP_REASON[reason])
            return
        if line['reason'] in ('old', 'range', 'syntax', 'nocrc'):
            self._latch('c_rejected', 'C q %d rejected: %s' % (line['q'], line['reason']))
            self.link.resync_q()   # take last_q from the next status line
            self._stop_control('c_rejected')

    def _on_fault(self, event):
        line = event['line']
        self.mcu_events.append(dict(t=event['t'], kind='F', code=line['code'], arg=line['arg'], ms=line['ms']))
        self.counters['mcu_fault_' + line['code']] += 1
        if line['code'] == 'TTL':
            self._stop_mode = 'brake' if self.hp.ttl_expiry_drive_mode == 'brake' else 'release'
            self._ttl_event(event['t'], 'F TTL line')
        elif line['code'] == 'CRC':
            self._stop_mode = 'brake'
            self._hold = (event['t'], self.hp.hold_s['loss'])
        elif line['code'] != 'NOCAL':
            self._stop_mode = 'release' if line['code'] == 'THERMAL' else 'brake'
            self._hold = None

    def _ttl_reference(self, t):
        """The operator input deadline (None: no lease) of the latest poll at least TTL_INPUT_LAG_S before t; the
        oldest poll kept when none is that old; None when no poll is kept."""
        limit = t - TTL_INPUT_LAG_S + self.TIME_EPS_S
        for poll_t, until in reversed(self._input_history):
            if poll_t <= limit:
                return until
        return self._input_history[0][1] if self._input_history else None

    def _ttl_event(self, t, source, count=True):
        """An MCU C TTL expiry seen at t (an F TTL line, or the TTL bit of an ARMED status line).

        DECISIONS R18 decision 1: with a lease and a confirmed ARM, an expiry the host did not cause stops at once and
        latches mcu_ttl_expired; the lease goes with the stop, so no C line leaves the host until an operator re-arm
        and the next C cannot restart the drive after the TTL brake (spec 6.3: a new C clears TTL and drives again
        after the 120 ms rest of 8.2). 'Did not cause': the operator input was still valid TTL_INPUT_LAG_S before
        this read (the host would have sent C lines: they were lost or late). An expiry at the end of the host's own
        input deadline (the lease kept at throttle 0, spec 6.3, HOST_BRIDGE_V3_KO.md 2) is the expected end of the
        command: counted (ttl_expected), not latched. No reference (no lease at that poll: the lease is newer than
        the lag, so this ARM's C lines cannot have run out yet) is not the expected end: it latches."""
        control = self.control
        if not (control.lease and self._arm_confirmed):
            return
        until = self._ttl_reference(t)
        if until is not None and until <= t + self.TIME_EPS_S:
            if count:
                self.counters['ttl_expected'] += 1
            return
        valid = 'an unknown time' if until is None else '%d ms' % int(round((until - t) * 1000.0))
        self._latch('mcu_ttl_expired', '%s while the operator input was valid for %s more (C lines lost or late)'
                    % (source, valid))
        self.last_error = 'mcu_ttl_expired'
        self._stop_control('mcu_ttl_expired')

    def _on_watchdog(self, event):
        self.mcu_events.append(dict(t=event['t'], kind='W', ms=event['line']['ms']))
        self.counters['mcu_watchdog'] += 1
        self._stop_mode = 'brake'
        mode = self.hp.watchdog_steer_mode
        self._hold = None if mode == 'free' else (event['t'], self.hp.hold_s['loss'])

    def _on_ack(self, event):
        self.mcu_events.append(dict(t=event['t'], kind='A', line=event['line']))

    def _on_pin(self, event):
        self.mcu_events.append(dict(t=event['t'], kind='P', line=event['line']))

    def _on_bad_line(self, event):
        self.counters['rx_bad_lines'] += 1

    def _on_identity(self, event):
        self.counters['identity_events'] += 1

    def _recent_mcu_cause(self):
        for item in reversed(self.mcu_events):
            if item['kind'] == 'W':
                return 'MCU watchdog'
            if item['kind'] == 'F' and item['code'] not in ('TTL', 'NOCAL'):
                return 'MCU fault ' + item['code']
        status = self.link.status
        return 'MCU state %s' % (status['state'] if status else 'unknown')

    def _marks_valid(self):
        cal = self.link.cal
        return cal is not None and None not in (cal['left'], cal['center'], cal['right'])

    def _ramp_goal(self, s, cal):
        """The goal of the MCU servo ramp: the target of the status line, clipped to the soft limits of the
        calibration (spec 7.1; the same goal as the servo state 'slewing')."""
        goal = s['tgt']
        if cal is not None and cal['soft_set']:
            goal = min(cal['soft_l'], max(cal['soft_r'], goal))
        return goal

    def _track_monitor(self, t, s):
        """steering_not_following: |requested - measured| > track_error for track_duration, counted only while the
        MCU servo ramp has reached its goal (status cmd == the soft-limited tgt). While the MCU still slews toward
        the request at its configured rate (rmov / rstd), the lag is the designed one and the MCU's own TRACK rule
        (progress-aware, spec 6.7, decision 12.1-69) watches it; a large standstill step (about 40 deg at 12 deg/s)
        used to latch here (SIM_ACCEPTANCE amendment 40, normal script N5; amendment 43).

        track_rule 'esp32_adapter' (host profile option; the vehicle's ESP32 stack, whose board has no servo ramp and
        no MCU TRACK rule): requested = the target the board was sent (status cmd, the steering law's last ANGLE),
        counted without the ramp gate, as the executor's Esp32AbsHoldAdapter._track does."""
        control = self.control
        if not (control.lease and self._arm_confirmed and s['st'] == 2 and control.steer_mrad is not None
                and self._marks_valid()):
            self._track_since = None
            return
        if self.hp.track_rule == 'esp32_adapter':
            requested = self.hp.mrad_from_permille(s['cmd'])
        elif s['cmd'] != self._ramp_goal(s, self.link.cal):
            self._track_since = None
            return
        else:
            requested = control.steer_mrad
        measured = self.hp.mrad_from_permille(s['pos'])
        if abs(requested - measured) > self.hp.track_error_mrad:
            if self._track_since is None:
                self._track_since = t
            elif t - self._track_since >= self.hp.track_duration_s:
                self._track_since = None
                self._latch('steering_not_following', 'requested %d mrad, measured %d mrad' % (requested, measured))
                self._stop_control('steering_not_following')
        else:
            self._track_since = None

    def _ineffective_monitor(self, t, s):
        control, cfg, motion = self.control, self.hp.ineffective, self._motion
        if not (control.lease and self._arm_confirmed and s['st'] == 2 and motion is not None
                and t - motion[0] <= MOTION_FRESH_S and self._marks_valid()):
            self._ineffective_since = None
            return
        delta = self.hp.mrad_from_permille(s['pos']) / 1000.0
        yaw_rate, speed = motion[1], motion[2]
        if speed <= cfg['min_speed_m_s'] or abs(delta) <= math.radians(cfg['min_steer_deg']):
            self._ineffective_since = None
            return
        predicted = speed * math.tan(delta) / self.hp.wheelbase_m
        limit = max(math.radians(cfg['rate_error_deg_s']), cfg['rate_error_frac'] * abs(predicted))
        if abs(yaw_rate - predicted) > limit:
            if self._ineffective_since is None:
                self._ineffective_since = t
            elif t - self._ineffective_since >= cfg['duration_s']:
                self._ineffective_since = None
                self._latch('steering_ineffective', 'gyro %.3f rad/s, v tan(delta)/L %.3f rad/s' % (yaw_rate,
                                                                                                   predicted))
                self._stop_control('steering_ineffective')
        else:
            self._ineffective_since = None

    def _check_timers(self, now):
        link, hp = self.link, self.hp
        if not link.handshake_done:
            return
        last = link.status_t if link.status_t is not None else link.banner_t
        if last is not None and now - last > hp.status_gap_s:
            if not self._status_stale:
                self._status_stale = True
                self._latch('steering_status_stale', 'no status line for %d ms' % int((now - last) * 1000))
                if self.control.lease:
                    self._stop_control('steering_status_stale')
        else:
            self._status_stale = False
        if self._arm_pending_t is not None and now - self._arm_pending_t > hp.arm_confirm_s:
            self._arm_pending_t = None
            self._clear_on_confirm = False
            self._latch('arm_unconfirmed', 'no status line with st ARMED within %d ms of the ARM'
                        % int(round(hp.arm_confirm_s * 1000)))
            self.last_error = 'arm_unconfirmed'
            self._stop_control('arm_unconfirmed')
        # INFO and CAL GET answers delay the next status line (303 B / 129 B of MCU output): never while the host
        # holds a lease (spec 12.1-60). A reset detection has revoked the lease before INFO is sent.
        quiet = not self.control.lease and self._arm_pending_t is None
        if self._banner_wanted and link.ready and quiet and now >= self._next_info_t - self.TIME_EPS_S:
            self._next_info_t = now + hp.banner_timeout_s
            self.counters['info_after_reset'] += 1
            self._send(link.send_info)
        if (link.ready and quiet and link.cal is None and not self._banner_wanted
                and (link.banner_t is None or now - link.banner_t >= self.CAL_QUERY_GRACE_S - self.TIME_EPS_S)
                and (self._last_cal_query is None or now - self._last_cal_query >= self.CAL_QUERY_INTERVAL_S)):
            self._last_cal_query = now
            self._send(link.send_cal_get)

    # -- transmit side
    def _desired_command(self, now):
        control = self.control
        if (not control.lease or not self._arm_confirmed or control.steer_mrad is None or now >= control.input_until
                or self._stop is not None):
            return None
        duty = duty_from_throttle(control.requested['left'])   # equal duty both sides (drive_lr_mode equal_duty)
        return duty, duty, self.hp.permille_from_mrad(control.steer_mrad), control.input_until

    def _drive_link(self, now):
        link, hp = self.link, self.hp
        stop = self._stop
        if stop is not None and not stop['given_up']:
            # Fixed grid from the first STOP (spec 12.1-62): resend k (k = 0 .. resend_max - 1) at
            # t0 + confirm + k * interval, the latch one confirm window after the last scheduled resend
            # (t0 + 100, 150, 200, 250, 300 ms and t0 + 400 ms with the placeholder values). A host loop of 20 ms
            # rounds each one up to its next tick but does not stretch the grid.
            first = stop['t0'] + hp.stop_confirm_s
            if stop['resends'] < hp.stop_resend_max:
                due = first + stop['resends'] * hp.stop_resend_interval_s
                if now >= due - self.TIME_EPS_S:
                    stop['resends'] += 1
                    stop['last'] = now
                    self.counters['stop_resent'] += 1
                    self._send(link.send_stop)
            elif now >= first + (hp.stop_resend_max - 1) * hp.stop_resend_interval_s + hp.stop_confirm_s \
                    - self.TIME_EPS_S:
                stop['given_up'] = True
                self._latch('stop_unconfirmed', 'no DISARMED status with zero duty after %d STOP resends'
                            % stop['resends'])
                self.last_error = 'stop_unconfirmed'
        if not link.ready:
            return
        if self._next_ping_due is None or now >= self._next_ping_due:
            self._send(link.send_ping)
            due = (now if self._next_ping_due is None else self._next_ping_due) + hp.ping_period_s
            self._next_ping_due = due if due > now else now + hp.ping_period_s
        command = self._desired_command(now)
        if command is None:
            self._next_c_due = None
            return
        left, right, steer, deadline = command
        remaining_ms = int(math.floor((deadline - now) * 1000.0))
        ttl = min(hp.c_ttl_max_ms, link.banner['ttl'], remaining_ms)
        if ttl < 1:
            return
        dirty = self._command_dirty
        if not (dirty or self._next_c_due is None or now >= self._next_c_due):
            return
        q = self._send(link.send_c, ttl, left, right, steer)
        if q is None:
            return
        self._last_c = dict(t=now, q=q, ttl=ttl, L=left, R=right, S=steer)
        self._command_dirty = False
        self._stop_mode = None if (left or right) else (self._stop_mode or 'release')
        if dirty or self._next_c_due is None:
            # A new request restarts the resend grid; periodic resends keep it (average c_resend_hz).
            self._next_c_due = now + hp.c_resend_period_s
        else:
            due = self._next_c_due + hp.c_resend_period_s
            self._next_c_due = due if due > now else now + hp.c_resend_period_s
        self.counters['c_sent'] += 1

    # -- status
    def pin_outputs(self):
        s = self.link.status
        if s is None:
            return {}
        left, right, sp, en = s['L'], s['R'], s['sp'], s['en']
        return {'D8': max(left, 0), 'D5': max(-left, 0), 'D3': max(right, 0), 'D0': max(-right, 0),
                'D1': 255 if en & 1 else 0, 'D2': 255 if en & 2 else 0, 'D6': max(sp, 0), 'D7': max(-sp, 0)}

    def applied(self):
        """/rover/control/applied: MCU-reported pin values (V2 pin keys) plus the V3 fields."""
        s = self.link.status
        out = dict(self.pin_outputs())
        if s is not None:
            out.update(drive_l=s['L'], drive_r=s['R'], drive_en=bool(s['en'] & 1), steer_pwm=s['sp'],
                       steer_en=bool(s['en'] & 2), applied_q=s['q'], mcu_ms=s['ms'],
                       age_ms=int(round((self.clock() - self.link.status_t) * 1000)))
        return out

    def _servo_state(self, s, cal):
        if not s['en'] & 2:
            return 'off'
        if s['st'] == 1:
            return 'hold'
        if s['st'] != 2:
            return 'off'
        if s['cmd'] != self._ramp_goal(s, cal):
            return 'slewing'
        return 'tracking' if s['sp'] else 'settled'

    def steering_block(self):
        now = self.clock()
        link, cal, s = self.link, self.link.cal, self.link.status
        host = list(self.latches)
        calibrated = bool(cal is not None and cal['cal_id'] is not None and self._marks_valid() and cal['soft_set'])
        if s is None:
            block = dict.fromkeys(STEERING_BLOCK_KEYS)
            block.update(faults=host, servo_state='unknown', saturated=False, calibrated=calibrated,
                         stop_mode=self._stop_mode)
            return block
        hold = None
        if self._hold is not None and s['st'] == 1 and s['en'] & 2:
            hold = max(0, int(round((self._hold[0] + self._hold[1] - now) * 1000)))
        saturated = abs(s['sp']) >= self.hp.servo_max_pwm
        if cal is not None and cal['soft_set'] and (s['tgt'] > cal['soft_l'] or s['tgt'] < cal['soft_r']):
            saturated = True
        return {'measured_mrad': self.hp.mrad_from_permille(s['pos']) if self._marks_valid() else None,
                'measured_permille': s['pos'], 'raw': s['raw'], 'target_permille': s['tgt'],
                'cmd_permille': s['cmd'], 'pwm': s['sp'], 'mcu_ms': s['ms'], 'seq': s['n'], 'applied_q': s['q'],
                'age_ms': int(round((now - link.status_t) * 1000)), 'ttl_remaining_ms': s['tl'],
                'drive_en': bool(s['en'] & 1), 'steer_en': bool(s['en'] & 2), 'hold_remaining_ms': hold,
                'stop_mode': self._stop_mode, 'faults': fault_names(s['fl']) + host,
                'servo_state': self._servo_state(s, cal), 'saturated': saturated, 'calibrated': calibrated}

    def steering_host(self):
        """Host-side diagnostics next to the design 4.2 block (not part of it)."""
        now = self.clock()
        s, link = self.link.status, self.link
        if self._arm_pending_t is not None:
            arm_state = 'pending'
        elif self._arm_confirmed:
            arm_state = 'confirmed'
        else:
            arm_state = 'none'
        if self._stop is not None:
            stop_state = 'unconfirmed' if self._stop['given_up'] else 'pending'
        else:
            stop_state = 'confirmed' if self._stop_confirmed else 'none'
        return {'arm_state': arm_state, 'stop_state': stop_state,
                'mcu_state': s['state'] if s else None, 'fl': '%04X' % s['fl'] if s else None,
                'stale': s is None or now - link.status_t > self.hp.status_gap_s,
                'target_mrad': self.control.steer_mrad,
                'latches': {name: dict(entry) for name, entry in self.latches.items()},
                'last_c': dict(self._last_c) if self._last_c else None,
                'next_q': link.next_q, 'link': link.describe(), 'counters': dict(sorted(self.counters.items())),
                'host_profile': self.hp.identity(),
                'expected_soft_permille': list(self.hp.expected_soft),
                'calibration_soft_matches': (None if link.cal is None or not link.cal['soft_set'] else
                                             (link.cal['soft_l'], link.cal['soft_r']) == self.hp.expected_soft)}

    def _status_extra(self):
        link = self.link
        banner = link.banner or {}
        cal = link.cal
        extra = {'firmware': FIRMWARE if link.handshake_done else None,
                 'protocol': PROTOCOL if link.handshake_done else None,
                 'boot_id': link.boot_id, 'device_id': banner.get('dev'), 'reset_reason': banner.get('rst'),
                 'hw_profile': banner.get('hw'), 'simulated': banner['sim'] if banner else link.simulated,
                 'profile': self.hp.vehicle_profile,
                 'cal_id': cal['cal_id'] if cal is not None else banner.get('cal_id'),
                 'steering': self.steering_block(), 'steering_host': self.steering_host()}
        if self.latches and not self.control.lease:
            extra['state'] = 'fault'
        return extra

    def _build_snapshot(self):
        transport = self.link.transport
        return {**self.control.status(), 'updated_at': self.wall_clock(),
                'transport': 'simulation' if self.link.simulated else 'esp_usb',
                'transport_kind': getattr(transport, 'kind', None), 'serial_port': getattr(transport, 'path', None),
                'serial_open': self.link.is_open, 'mode': 'manual',
                'device_identity': getattr(transport, 'identity', None)}

    def steering_state(self, line=None, received=None):
        """/rover/steering/state (JSON, one per MCU status line, nominal 20 Hz): a status line (default: the last
        one), its host receive time and the calibrated angle. The wrapper gives every line it took from the link."""
        now = self.clock()
        link = self.link
        s, banner, cal = link.status, link.banner or {}, link.cal
        status_t = link.status_t
        if line is not None:
            s, status_t = line, received
        received_at = None if s is None else self.wall_clock() - (now - status_t)
        state = dict(schema=STEER_STATE_SCHEMA, source_kind='simulation' if link.simulated else 'mcu_status',
                     received_at=received_at, received_monotonic=status_t,
                     stale=s is None or now - status_t > self.hp.status_gap_s,
                     boot_id=link.boot_id, reset_reason=banner.get('rst'),
                     cal_id=cal['cal_id'] if cal else banner.get('cal_id'), target_mrad=self.control.steer_mrad,
                     stop_mode=self._stop_mode, host_faults=list(self.latches),
                     host_profile_sha256=self.hp.sha256)
        if s is None:
            state.update(dict.fromkeys(('mcu_ms', 'mcu_seq', 'state', 'raw_adc', 'pos_permille', 'angle_mrad',
                                        'target_permille', 'cmd_permille', 'steer_pwm', 'drive_l', 'drive_r',
                                        'drive_en', 'steer_en', 'applied_q', 'ttl_remaining_ms', 'fl')))
            state['faults'] = list(self.latches)
            return state
        state.update(mcu_ms=s['ms'], mcu_seq=s['n'], state=s['state'], raw_adc=s['raw'], pos_permille=s['pos'],
                     angle_mrad=self.hp.mrad_from_permille(s['pos']) if self._marks_valid() else None,
                     target_permille=s['tgt'], cmd_permille=s['cmd'], steer_pwm=s['sp'], drive_l=s['L'],
                     drive_r=s['R'], drive_en=bool(s['en'] & 1), steer_en=bool(s['en'] & 2), applied_q=s['q'],
                     ttl_remaining_ms=s['tl'], fl='%04X' % s['fl'], faults=fault_names(s['fl']) + list(self.latches))
        return state


def hp_cal_mismatch(hp, cal_id):
    """A reported calibration id that differs from the host profile one (None reported = uncalibrated, not a
    mismatch; hp.cal_id None = accept any id)."""
    return cal_id is not None and hp.cal_id is not None and cal_id != hp.cal_id
