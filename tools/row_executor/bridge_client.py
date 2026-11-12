"""Client for the existing ESP bridge socket (no change to the bridge, dashboard or firmware).

Protocol facts this client relies on (DASH = dashboard-live-raw, read-only reference):
  * /run/ubts-rover/control.sock, one JSON object per line, ONE request per connection: the
    bridge Handler reads a single line, answers it and closes (scripts/rover_control_bridge.py
    Handler.handle). Every call here opens a new connection.
  * status / stop never need a lease; op stop works for any client and clears the current lease
    (scripts/rover_control_core.py dispatch, Bridge.request emergency path).
  * arm {owner} -> {lease, ticket, seq=0}; refused with control_in_use while another client
    (e.g. dashboard manual driving) holds the lease, or verified_esp_v2_required when the ESP is
    not ready.
  * drive {owner, lease, ticket, seq=previous+1, deadman=true, left, right} -> new ticket
    (TTL 1.5 s). deadman MUST be true on EVERY drive, including 0/0: a drive without it is an
    invalid_drive (hard stop + lease removed). 0/0 keeps the lease: moving is False so the 0.75 s
    input TTL does not expire, and every drive renews the 30 s owner TTL. "Hold to run" is the
    executor's own operator signal and is NOT the bridge deadman field.
  * a non-zero drive must be renewed within 0.75 s or the bridge stops (input_expired) and
    removes the lease.

Client states: DISARMED -> arm() -> ARMED -> stop() -> DISARMED; any error -> LATCHED.
LATCHED on any error (transport error/timeout, malformed answer, ok:false such as
control_not_owned after somebody else's STOP, stale_input_rearm_required, invalid_sequence,
request_expired, control_busy_stopped, stop_barrier, a bridge status that is not ok). When the
client held a lease at that moment it sends a best-effort op stop, EXCEPT for the answers that
say the lease is already gone (LEASE_GONE_ERRORS: control_not_owned, stop_barrier): somebody
else's STOP (dashboard, observer phone, input expiry) already stopped the motors and removed our
lease, and the next lease may already belong to the human who armed manual driving on the
dashboard; an op stop from us would wipe that new lease (review S7). The client NEVER re-arms by
itself: arm() refuses while LATCHED until acknowledge_latch() is called from an explicit
operator action, and drive() never sends anything unless ARMED.

Forward-only by design: drive() refuses negative or non-integer duties (that is a bug upstream;
it latches and stops).

Recovery (field): see RECOVERY_PROCEDURE and recovery_hint(status).

Front-steer vehicle (steering branch P4, docs/steering/EXECUTOR_STEER_KO.md): SteerBridgeClient (end of this file) is the
client of the V3 steering bridge (dashboard/scripts/steer_rover_bridge.py + steer_bridge_core.py; same socket rules):
drive(throttle, steer_mrad), keepalive(), motion(yaw_rate, speed), the steering status of every reply and the steering
arm precheck (steer_arm_precheck). The V2 client above is unchanged.
"""
import json
import socket
import threading
import time
import uuid

FIRMWARE = 'UBTS_WHEEL_PWM_V2'
DISARMED, ARMED, LATCHED = 'DISARMED', 'ARMED', 'LATCHED'
MAX_REQUEST_BYTES = 4096        # the bridge Handler refuses longer lines
MAX_RESPONSE_BYTES = 65536
BRIDGE_CONTAINER = 'unist-orachard-ros2-esp-rover-bridge'   # container_name in the dashboard compose file
BRIDGE_FIND = "docker ps -a --filter name=esp-rover-bridge --format '{{.Names}} {{.Status}}'"
# ok:false answers after which the bridge no longer holds OUR lease (it was removed by an op stop / expiry / other
# owner). Latching on them must not send an op stop: it could only hit a lease the human took in the meantime.
LEASE_GONE_ERRORS = ('control_not_owned', 'stop_barrier')

RECOVERY_PROCEDURE = """\
ESP bridge recovery (Jetson, operator with SSH or a local terminal; motors must be safe first):
 1. Keep the vehicle stopped: physical E-stop pressed or wheels off the ground. Expect a possible
    short output glitch while the bridge re-opens the serial port (ESP reset, unverified).
 2. Find the ACTUAL bridge container name and state (expected %(c)s; use the name printed here):
      %(find)s
    and cat ~/unist_orachard_dashboard/runtime/rover_bridge_status.json
 3. Restart ONLY that bridge container:  docker restart <name from step 2>
    NEVER use docker compose up/down/restart for any dashboard service: the compose files pin an
    old dashboard image and recreate containers (workspace guide, 2026-10-13 table).
 4. Confirm: status shows connected=true, firmware=%(f)s, state=disarmed, last_error=null.
 5. Executor: acknowledge the latch in the operator page and press Start again (never automatic).
Why a restart is required:
 - MCU watchdog event: EspMotor.error = 'mcu_watchdog_rearm_required' is never cleared in-process
   (scripts/esp_motor.py event/ready) -> the bridge reports connected=false and refuses arm.
 - RuntimeError from the wheel controller (e.g. 'PC heartbeat expired; wheels disabled') escapes
   Bridge.step's except (ControlError, ValueError) and puts the bridge loop into state 'fault'
   for good (scripts/rover_control_bridge.py main loop): requests then time out as
   control_busy_stopped.
 - If the restart cannot open the device (USB re-enumerated after an unplug), do not recreate
   the container with compose; stop and escalate (device mapping is fixed at container creation).
""" % dict(c=BRIDGE_CONTAINER, f=FIRMWARE, find=BRIDGE_FIND)


class BridgeError(RuntimeError):
    def __init__(self, reason, response=None):
        super().__init__(reason)
        self.reason = reason
        self.response = response


class TransportError(BridgeError):
    pass


class UnixSocketTransport:
    """One request per connection, newline-terminated JSON both ways, bounded by one deadline."""

    def __init__(self, path, timeout=.6):
        self.path = str(path)
        self.timeout = float(timeout)
        self.connections = 0

    def __call__(self, request):
        data = json.dumps(request, allow_nan=False, separators=(',', ':')).encode('utf-8') + b'\n'
        if len(data) > MAX_REQUEST_BYTES:
            raise TransportError('request_too_large')
        deadline = time.monotonic() + self.timeout
        buffer = b''
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        except (AttributeError, OSError) as exc:
            raise TransportError('socket_unavailable:%s' % exc) from None
        try:
            sock.settimeout(self.timeout)
            sock.connect(self.path)
            self.connections += 1
            sock.sendall(data)
            while b'\n' not in buffer:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TransportError('timeout')
                sock.settimeout(remaining)
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buffer += chunk
                if len(buffer) > MAX_RESPONSE_BYTES:
                    raise TransportError('response_too_large')
        except socket.timeout:
            raise TransportError('timeout') from None
        except OSError as exc:
            raise TransportError('socket_error:%s' % (exc.strerror or exc)) from None
        finally:
            sock.close()
        line = buffer.split(b'\n', 1)[0]
        if not line:
            raise TransportError('empty_response')
        try:
            response = json.loads(line)
        except ValueError:
            raise TransportError('malformed_response') from None
        if not isinstance(response, dict):
            raise TransportError('malformed_response')
        return response


class BridgeClient:
    def __init__(self, path='/run/ubts-rover/control.sock', owner=None, timeout=.6, transport=None,
                 snapshot_max_age_s=2.):
        self.owner = owner or 'row-executor:%s' % uuid.uuid4().hex[:12]
        if not isinstance(self.owner, str) or not self.owner:
            raise ValueError('owner must be a non-empty string')
        self.transport = transport if transport is not None else UnixSocketTransport(path, timeout)
        self.snapshot_max_age_s = float(snapshot_max_age_s)
        self._lock = threading.RLock()
        self.state = DISARMED
        self.lease = self.ticket = None
        self.seq = 0
        self.latch_reason = None
        self.last_status = None
        self.last_response = None
        self.last_ok_monotonic = None
        self.counts = dict(requests=0, drives=0, errors=0, latches=0, stops=0, latch_stop_skipped=0)

    # --- internals -----------------------------------------------------------------------------
    def _send(self, request):
        self.counts['requests'] += 1
        response = self.transport(request)
        if not isinstance(response, dict):
            raise TransportError('malformed_response')
        self.last_response = response
        return response

    def _latch(self, reason, send_stop):
        """Enter LATCHED; best-effort op stop if a lease was held (and send_stop). Never re-arms."""
        held = self.lease is not None
        self.state = LATCHED
        self.latch_reason = str(reason)
        self.counts['latches'] += 1
        self.lease = self.ticket = None
        if held and not send_stop:
            self.counts['latch_stop_skipped'] += 1
        if send_stop and held:
            try:
                self.counts['stops'] += 1
                self._send({'op': 'stop', 'reason': ('row_executor_latched:' + str(reason))[:80]})
            except BridgeError:
                pass

    def _fail(self, reason, response=None, send_stop=True):
        self.counts['errors'] += 1
        self._latch(reason, send_stop)
        raise BridgeError(reason, response)

    # --- operations ----------------------------------------------------------------------------
    def status(self):
        """Bridge snapshot dict. Transport errors or ok:false latch (stop sent if a lease was held)."""
        with self._lock:
            try:
                response = self._send({'op': 'status'})
            except BridgeError as exc:
                self._fail('status_failed:' + exc.reason)
            self.last_status = response
            if response.get('ok') is not True:
                self._fail('bridge_status_not_ok:%s:%s' % (response.get('state'), response.get('last_error')),
                           response)
            return response

    def arm(self):
        """Explicit operator action only. Precheck the bridge, then take the lease."""
        with self._lock:
            if self.state == LATCHED:
                raise BridgeError('latched:%s' % self.latch_reason)
            if self.state == ARMED:
                raise BridgeError('already_armed')
            status = self.status()
            problem = arm_precheck(status, self.snapshot_max_age_s)
            if problem:
                self._fail('arm_precheck:' + problem, status, send_stop=False)
            try:
                response = self._send({'op': 'arm', 'owner': self.owner})
            except BridgeError as exc:
                self._fail('arm_failed:' + exc.reason, send_stop=False)
            if response.get('ok') is not True:
                # e.g. control_in_use: we never held the lease, so no stop is sent to the other owner.
                self._fail('arm_refused:%s' % response.get('error'), response, send_stop=False)
            lease, ticket, seq = response.get('lease'), response.get('ticket'), response.get('seq')
            if not isinstance(lease, str) or not lease or not isinstance(ticket, str) or not ticket or seq != 0 \
                    or type(seq) is not int:
                self.lease = lease if isinstance(lease, str) and lease else None
                self._fail('arm_malformed_response', response)
            self.lease, self.ticket, self.seq = lease, ticket, 0
            self.state = ARMED
            self.latch_reason = None
            self.last_ok_monotonic = time.monotonic()
            return response

    def drive(self, left, right):
        """Send one drive with deadman:true (also for 0/0). Requires ARMED; never sends otherwise."""
        with self._lock:
            if self.state != ARMED:
                raise BridgeError('not_armed:%s' % self.state)
            for value in (left, right):
                if type(value) is not int or not 0 <= value <= 100:
                    self._fail('invalid_drive_values:%r/%r' % (left, right))
            seq = self.seq + 1
            request = {'op': 'drive', 'owner': self.owner, 'lease': self.lease, 'ticket': self.ticket,
                       'seq': seq, 'deadman': True, 'left': left, 'right': right}
            try:
                response = self._send(request)
            except BridgeError as exc:
                self._fail('drive_failed:' + exc.reason)
            if response.get('ok') is not True:
                error = response.get('error')
                # lease already gone (somebody else's STOP / expiry): latch silently, never stop a newer lease
                self._fail('drive_refused:%s' % error, response, send_stop=error not in LEASE_GONE_ERRORS)
            ticket = response.get('ticket')
            if (not isinstance(ticket, str) or not ticket or response.get('seq') != seq
                    or type(response.get('seq')) is not int or response.get('lease') != self.lease):
                self._fail('drive_malformed_response', response)
            self.ticket, self.seq = ticket, seq
            self.counts['drives'] += 1
            self.last_ok_monotonic = time.monotonic()
            return response

    def stop(self, reason='row_executor_stop', latch=False):
        """op stop (lease not needed). Normal end -> DISARMED; latch=True or any error -> LATCHED.

        Returns True when the bridge acknowledged the stop request. The acknowledgement means
        'stop requested', not 'outputs are zero': confirm with status()['outputs'].
        """
        with self._lock:
            self.counts['stops'] += 1
            self.lease = self.ticket = None
            try:
                response = self._send({'op': 'stop', 'reason': str(reason)[:80]})
            except BridgeError as exc:
                self.counts['errors'] += 1
                self._latch('stop_failed:' + exc.reason, send_stop=False)
                return False
            if response.get('ok') is not True:
                self.counts['errors'] += 1
                self._latch('stop_not_ok:%s' % response.get('error'), send_stop=False)
                return False
            if latch:
                self._latch('operator_stop:' + str(reason), send_stop=False)
            elif self.state != LATCHED:
                self.state = DISARMED
            return True

    def acknowledge_latch(self):
        """Explicit operator acknowledgement: LATCHED -> DISARMED. Does not arm."""
        with self._lock:
            if self.state == LATCHED:
                self.state = DISARMED
                self.latch_reason = None
            return self.state

    def snapshot(self):
        with self._lock:
            return dict(state=self.state, owner=self.owner, seq=self.seq, has_lease=self.lease is not None,
                        latch_reason=self.latch_reason, counts=dict(self.counts),
                        last_ok_monotonic=self.last_ok_monotonic)


def arm_precheck(status, snapshot_max_age_s=2., now=None):
    """None when the bridge snapshot allows an arm, else the first problem found."""
    if not isinstance(status, dict) or status.get('ok') is not True:
        return 'status_not_ok'
    if status.get('connected') is not True:
        return 'esp_not_connected'
    if status.get('firmware') != FIRMWARE:
        return 'firmware_not_v2'
    if status.get('last_error'):
        return 'bridge_error:%s' % status.get('last_error')
    if status.get('armed') is not False or status.get('state') != 'disarmed':
        return 'bridge_armed_by_other_client'
    if status.get('moving') is not False:
        return 'bridge_reports_moving'
    updated = status.get('updated_at')
    if updated is not None:
        now = time.time() if now is None else now
        if not isinstance(updated, (int, float)) or abs(now - updated) > snapshot_max_age_s:
            return 'bridge_snapshot_stale'
    return None


def recovery_hint(status=None, error=None):
    """Short operator hint for a failed status/arm/drive; the full text is RECOVERY_PROCEDURE."""
    restart = ('restart the ESP bridge: docker restart %s (never docker compose; check the actual name first: '
               'docker ps -a --filter name=esp-rover-bridge)' % BRIDGE_CONTAINER)
    if error is not None and str(error).startswith(('status_failed', 'drive_failed', 'arm_failed', 'stop_failed')):
        return 'bridge socket unreachable or timing out: check docker ps; ' + restart
    if not isinstance(status, dict):
        return 'no bridge status: check docker ps; ' + restart
    if status.get('state') == 'fault' or status.get('ok') is False:
        return 'bridge loop in fault state (%s): %s' % (status.get('last_error'), restart)
    if status.get('last_error') == 'mcu_watchdog_rearm_required':
        return 'MCU watchdog latched the ESP (permanent until restart): ' + restart
    if status.get('connected') is not True:
        return 'ESP not connected/verified: check USB and power; ' + restart
    if status.get('armed'):
        return 'another client holds the lease (dashboard manual?): press STOP on the dashboard, then Start again'
    return 'bridge looks ready: acknowledge the latch and press Start (never automatic)'


# --- front-steer V3 bridge (steering branch P4; design 4.2, 8.3 P4 'BridgeClient') ------------------------------------
STEER_FIRMWARE = 'UBTS_STEER_V3'
STEER_PROTOCOL = 3
STEER_VEHICLE_PROFILE = 'front_steer'
# Status bits that are not faults at an arm (V3 spec 5.3): the BOOT marker, an expired C TTL, a clamped target.
STEER_PRECHECK_IGNORED_BITS = ('BOOT', 'TTL', 'SOFTLIMIT')
# Host latches (steer_bridge_core LATCH_NAMES) that an operator re-arm clears (spec 12.1-75): after acknowledge_latch()
# the precheck lets the arm go to the bridge, which refuses it with the code while the condition is still there.
STEER_REARM_STATES = ('disarmed', 'fault')
# Host latches that may stand while NO client holds the lease and that the operator's Arm press itself clears (the first
# Arm of the G-SIM runs of 2026-11-12, docs/steering/OPERATOR_API_KO.md): steering_status_stale is latched by the bridge
# on a status-line gap even when nobody drives (G-SIM: a Gazebo clock stall of 0.11-0.19 s wall slowed the MCU
# emulator's status lines while the stack stood idle). With no lease no output was affected; the arm still needs the
# status to be fresh NOW (the rearm precheck checks stale, age_ms, the STOP confirmation, no lease, no motion), and the
# bridge's arm clears the latch.
STEER_IDLE_CLEARABLE_LATCHES = ('steering_status_stale',)


def idle_latch_only(status):
    """The idle-clearable host latches of a status whose ONLY faults they are (no MCU fault bit, no other latch) and
    which nobody holds (not armed, no lease, state disarmed / fault); () otherwise."""
    if not isinstance(status, dict):
        return ()
    steering, host = status.get('steering'), status.get('steering_host')
    if not isinstance(steering, dict) or not isinstance(host, dict):
        return ()
    latches = host.get('latches')
    if not isinstance(latches, dict) or not latches or not set(latches) <= set(STEER_IDLE_CLEARABLE_LATCHES):
        return ()
    faults = {str(name) for name in steering.get('faults') or () if name not in STEER_PRECHECK_IGNORED_BITS}
    if not faults or not faults <= set(latches):
        return ()
    if status.get('armed') is not False or status.get('state') not in STEER_REARM_STATES or host.get('arm_state') == 'pending':
        return ()
    return tuple(sorted(latches))


def steer_arm_precheck(status, *, expect_cal_id, allow_sim, limit_left_mrad, limit_right_mrad, status_max_age_s=.3,
                       snapshot_max_age_s=2., now=None, rearm=False):
    """None when a steer-bridge snapshot allows an executor arm (design 8.3 P4), else the first problem found.

    Checked: the status is ok and connected; firmware UBTS_STEER_V3 and protocol 3; vehicle_profile front_steer (the
    control rules and the host profile); the device identity sim flag equals allow_sim (True only in a SIM
    configuration: a SIM executor never arms a real vehicle, a field executor never arms a SIM device); the bridge's
    equivalent soft limits equal the configured ones; a steering block that is calibrated and has a finite measured
    angle; the calibration set id the configuration expects (none configured: refused); no POLARITY / DRV_STUCK bit (a
    passing CAL POL is needed, spec 12.1-71); no other fault bit or host latch (rearm=True, an operator re-arm after an
    acknowledged latch: the latches and latched MCU bits are what the arm clears, the bridge refuses the arm while the
    condition is still there); a fresh steering status (steering_host not stale, age_ms <= status_max_age_s) and a
    confirmed STOP; nobody holds the lease and nothing moves; the snapshot itself is recent (updated_at, wall clock).
    """
    if not isinstance(status, dict) or status.get('ok') is not True:
        return 'status_not_ok'
    if status.get('connected') is not True:
        return 'esp_not_connected'
    if status.get('firmware') != STEER_FIRMWARE or status.get('protocol') != STEER_PROTOCOL:
        return 'firmware_v3_required'
    if status.get('vehicle_profile') != STEER_VEHICLE_PROFILE or status.get('profile') != STEER_VEHICLE_PROFILE:
        return 'vehicle_profile_mismatch'
    if status.get('simulated') is not bool(allow_sim):
        return 'sim_identity_mismatch'
    limits = status.get('steer_limit_mrad')
    if not isinstance(limits, dict) or limits.get('left') != limit_left_mrad or limits.get('right') != limit_right_mrad:
        return 'steer_limit_mismatch'
    steering = status.get('steering')
    if not isinstance(steering, dict):
        return 'steering_status_missing'
    if steering.get('calibrated') is not True:
        return 'steering_uncalibrated'
    if expect_cal_id is None:
        return 'calibration_id_not_configured'
    if status.get('cal_id') != expect_cal_id:
        return 'calibration_mismatch'
    faults = [str(name) for name in steering.get('faults') or () if name not in STEER_PRECHECK_IGNORED_BITS]
    if 'POLARITY' in faults or 'DRV_STUCK' in faults:
        return 'polarity_check_missing'
    if faults and not rearm:
        return 'steering_fault:' + ','.join(faults)
    host = status.get('steering_host')
    if not isinstance(host, dict) or host.get('stale', True) is not False:
        return 'steering_status_stale'
    age = steering.get('age_ms')
    if type(age) is not int or age < 0 or age > status_max_age_s * 1000.:
        return 'steering_status_stale'
    measured = steering.get('measured_mrad')
    if isinstance(measured, bool) or not isinstance(measured, (int, float)) or measured != measured \
            or measured in (float('inf'), float('-inf')):
        return 'steering_feedback_invalid'
    if host.get('stop_state') != 'confirmed':
        return 'stop_unconfirmed'
    state = status.get('state')
    if status.get('armed') is not False or state not in (STEER_REARM_STATES if rearm else ('disarmed',)):
        return 'bridge_armed_by_other_client' if status.get('armed') else 'bridge_state_%s' % state
    if status.get('moving') is not False:
        return 'bridge_reports_moving'
    updated = status.get('updated_at')
    if updated is not None:
        now = time.time() if now is None else now
        if not isinstance(updated, (int, float)) or abs(now - updated) > snapshot_max_age_s:
            return 'bridge_snapshot_stale'
    return None


class SteerBridgeClient(BridgeClient):
    """Client of the front-steer V3 bridge: the BridgeClient socket, latch and no-automatic-re-arm rules plus

      arm()                 steer_arm_precheck(status) first, then {op arm, owner, vehicle_profile 'front_steer'}
      drive(throttle, steer_mrad)  {op drive, owner, lease, ticket, seq + 1, deadman true, throttle, steer_mrad}: throttle
                            an int 0..100 (forward only; both sides get it), steer_mrad an int inside the configured
                            equivalent soft limit; anything else latches and stops (a bug upstream)
      keepalive()           {op keepalive, owner, lease, ticket, seq + 1}: lease and ticket only (no motion, no C line);
                            the executor uses it while it has no steering target yet
      motion(yaw, speed)    {op motion, owner, lease, yaw_rate_rad_s, speed_m_s}: the executor's sensor-derived motion
                            sample (gyro yaw rate, LiDAR scan-matching speed) for the bridge's steering_ineffective
                            monitor; answered at once; never latches (a refusal is counted)
      steering()            (receipt time, steering block, steering_host block) of the last reply that carried a status
                            (arm, drive, keepalive, status), or None
      fault_reason(status)  the design 6.6 stop reason of a status' steering faults (config.steer_fault_reason) or None
    clock: the executor clock (receipt times); wall: the wall clock of the snapshot age check (updated_at).
    """

    def __init__(self, path='/run/ubts-rover/control.sock', owner=None, timeout=.6, transport=None,
                 snapshot_max_age_s=2., *, limit_left_mrad, limit_right_mrad, expect_cal_id, allow_sim=False,
                 status_max_age_s=.3, clock=time.monotonic, wall=time.time):
        super().__init__(path, owner, timeout, transport, snapshot_max_age_s)
        for name, value in (('limit_left_mrad', limit_left_mrad), ('limit_right_mrad', limit_right_mrad)):
            if type(value) is not int or not 0 < value < 1571:
                raise ValueError('%s must be a positive int mrad' % name)
        if expect_cal_id is not None and (type(expect_cal_id) is not int or not 1 <= expect_cal_id <= 65535):
            raise ValueError('expect_cal_id must be None or an int 1..65535')
        self.limit_left_mrad, self.limit_right_mrad = limit_left_mrad, limit_right_mrad
        self.expect_cal_id = expect_cal_id
        self.allow_sim = bool(allow_sim)
        self.status_max_age_s = float(status_max_age_s)
        self.clock, self.wall = clock, wall
        self.steering_seen = None
        self.operator_ack = False
        self.last_precheck = None
        self.counts.update(keepalives=0, motion=0, motion_refused=0, steering_replies=0)

    @classmethod
    def from_config(cls, cfg, transport=None, owner=None, clock=time.monotonic, wall=time.time):
        return cls(cfg.socket_path, owner=owner, timeout=cfg.bridge_timeout_s, transport=transport,
                   limit_left_mrad=int(cfg.steer_limit_left_mrad), limit_right_mrad=int(cfg.steer_limit_right_mrad),
                   expect_cal_id=cfg.steer_cal_id, allow_sim=cfg.steer_allow_sim,
                   status_max_age_s=cfg.steer_status_max_age_s, clock=clock, wall=wall)

    # --- internals -----------------------------------------------------------------------------
    def _keep(self, response):
        """Keep the steering status of a reply (a drive / keepalive / arm reply carries it under 'status')."""
        status = response if 'steering' in response else response.get('status')
        if isinstance(status, dict) and isinstance(status.get('steering'), dict):
            host = status.get('steering_host')
            host = dict(host) if isinstance(host, dict) else {}
            pins = status.get('outputs')
            if isinstance(pins, dict):
                # the drive pins the MCU reported in the same status line (V2 pin keys, 0..255): the signed APPLIED duty
                # of each side for the estimator's actuator prior (core.Runner._feed_actuator; never the request)
                values = [pins.get(key) for key in ('D8', 'D5', 'D3', 'D0')]
                if all(type(v) is int for v in values):
                    host['drive_l'], host['drive_r'] = values[0] - values[1], values[2] - values[3]
            self.steering_seen = (self.clock(), dict(status['steering']), host)
            self.counts['steering_replies'] += 1
            return status
        return None

    def _owned_request(self, op, **fields):
        seq = self.seq + 1
        request = dict(op=op, owner=self.owner, lease=self.lease, ticket=self.ticket, seq=seq, **fields)
        try:
            response = self._send(request)
        except BridgeError as exc:
            self._fail('%s_failed:%s' % (op, exc.reason))
        if response.get('ok') is not True:
            error = response.get('error')
            self._fail('%s_refused:%s' % (op, error), response, send_stop=error not in LEASE_GONE_ERRORS)
        ticket = response.get('ticket')
        if (not isinstance(ticket, str) or not ticket or response.get('seq') != seq
                or type(response.get('seq')) is not int or response.get('lease') != self.lease):
            self._fail('%s_malformed_response' % op, response)
        self.ticket, self.seq = ticket, seq
        self.last_ok_monotonic = self.clock()
        self._keep(response)
        return response

    # --- operations ----------------------------------------------------------------------------
    def status(self):
        response = super().status()
        self._keep(response)
        return response

    def arm(self):
        """Explicit operator action only. Steering precheck, then take the lease (vehicle_profile front_steer)."""
        with self._lock:
            if self.state == LATCHED:
                raise BridgeError('latched:%s' % self.latch_reason)
            if self.state == ARMED:
                raise BridgeError('already_armed')
            status = self.status()
            rearm = self.operator_ack

            def check(rearm):
                return steer_arm_precheck(status, expect_cal_id=self.expect_cal_id, allow_sim=self.allow_sim,
                                          limit_left_mrad=self.limit_left_mrad, limit_right_mrad=self.limit_right_mrad,
                                          status_max_age_s=self.status_max_age_s,
                                          snapshot_max_age_s=self.snapshot_max_age_s, now=self.wall(), rearm=rearm)
            problem = check(rearm)
            self.last_precheck = dict(problem=problem, rearm=rearm)
            idle = idle_latch_only(status) if problem is not None and not rearm else ()
            if idle and check(True) is None:
                # a latch the bridge set while nobody held the lease (STEER_IDLE_CLEARABLE_LATCHES): this Arm press is
                # the operator action that clears it; everything else of the precheck still holds (rearm rules)
                self.last_precheck = dict(problem=None, rearm=True, idle_latches=list(idle), first_problem=problem)
                self.counts['idle_latch_rearms'] = self.counts.get('idle_latch_rearms', 0) + 1
                problem = None
            if problem:
                self._fail('arm_precheck:' + problem, status, send_stop=False)
            try:
                response = self._send({'op': 'arm', 'owner': self.owner, 'vehicle_profile': STEER_VEHICLE_PROFILE})
            except BridgeError as exc:
                self._fail('arm_failed:' + exc.reason, send_stop=False)
            if response.get('ok') is not True:
                self._fail('arm_refused:%s' % response.get('error'), response, send_stop=False)
            lease, ticket, seq = response.get('lease'), response.get('ticket'), response.get('seq')
            if not isinstance(lease, str) or not lease or not isinstance(ticket, str) or not ticket or seq != 0 \
                    or type(seq) is not int:
                self.lease = lease if isinstance(lease, str) and lease else None
                self._fail('arm_malformed_response', response)
            self.lease, self.ticket, self.seq = lease, ticket, 0
            self.state = ARMED
            self.latch_reason = None
            self.operator_ack = False
            self.last_ok_monotonic = self.clock()
            self._keep(response)
            return response

    def drive(self, throttle, steer_mrad):
        """One drive (deadman true): throttle int 0..100 on both sides, steer_mrad int inside the soft limit."""
        with self._lock:
            if self.state != ARMED:
                raise BridgeError('not_armed:%s' % self.state)
            if (type(throttle) is not int or not 0 <= throttle <= 100 or type(steer_mrad) is not int
                    or not -self.limit_right_mrad <= steer_mrad <= self.limit_left_mrad):
                self._fail('invalid_drive_values:%r/%r' % (throttle, steer_mrad))
            response = self._owned_request('drive', deadman=True, throttle=throttle, steer_mrad=steer_mrad)
            self.counts['drives'] += 1
            return response

    def keepalive(self):
        """Lease and ticket only (no motion): before the executor has a steering target."""
        with self._lock:
            if self.state != ARMED:
                raise BridgeError('not_armed:%s' % self.state)
            response = self._owned_request('keepalive')
            self.counts['keepalives'] += 1
            return response

    def motion(self, yaw_rate_rad_s, speed_m_s):
        """Sensor-derived motion sample for the bridge's steering_ineffective monitor; True when the bridge took it.
        Never latches: a refusal (e.g. control_not_owned right after a stop) or a transport error is only counted."""
        with self._lock:
            if self.state != ARMED or self.lease is None:
                return False
            try:
                response = self._send({'op': 'motion', 'owner': self.owner, 'lease': self.lease,
                                       'yaw_rate_rad_s': float(yaw_rate_rad_s), 'speed_m_s': float(speed_m_s)})
            except BridgeError:
                self.counts['motion_refused'] += 1
                return False
            if response.get('ok') is not True:
                self.counts['motion_refused'] += 1
                return False
            self.counts['motion'] += 1
            return True

    def acknowledge_latch(self):
        """Explicit operator acknowledgement: LATCHED -> DISARMED; the next arm is an operator re-arm (the bridge's
        latches are what it clears, steer_arm_precheck rearm). Does not arm."""
        with self._lock:
            self.operator_ack = True
            return super().acknowledge_latch()

    def steering(self):
        with self._lock:
            return self.steering_seen

    def fault_reason(self, status=None):
        """The design 6.6 stop reason of a status' steering faults and host latches (None: none of them is a steering
        stop, or no status). status None: a fresh status() request (errors give None, the client stays as it is)."""
        from .config import steer_fault_reason
        if status is None:
            try:
                status = self._send({'op': 'status'})
            except BridgeError:
                return None
            self._keep(status)
        steering = status.get('steering') if isinstance(status, dict) else None
        if not isinstance(steering, dict):
            return None
        names = list(steering.get('faults') or ())
        host = status.get('steering_host')
        if isinstance(host, dict) and isinstance(host.get('latches'), dict):
            names += list(host['latches'])
        return steer_fault_reason(names)

    def snapshot(self):
        out = super().snapshot()
        with self._lock:
            seen = self.steering_seen
            out.update(vehicle_profile=STEER_VEHICLE_PROFILE, operator_ack=self.operator_ack,
                       last_precheck=self.last_precheck,
                       steering_age_s=None if seen is None else round(self.clock() - seen[0], 3))
        return out
