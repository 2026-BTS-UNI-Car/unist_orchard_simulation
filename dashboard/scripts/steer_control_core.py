"""Manual front-steer control rules on top of RoverControl (design 4.2). No hardware, socket or ROS imports.

Kept from RoverControl (scripts/rover_control_core.py, not modified): lease, one-use ticket, seq + 1, deadman,
input deadline 0.75 s, ticket 1.5 s, idle owner 30 s, keepalive that never extends the motion deadline,
release with a 1.1 s completion bound, stop that revokes the lease, no automatic re-arm.

Changed for the steered vehicle:
  * arm needs "vehicle_profile": "front_steer" (steer_profile_required). The motor adapter may refuse arm with
    a steering error code (firmware_v3_required, steering_uncalibrated, calibration_mismatch, ...).
  * drive takes throttle (int -100..100) and steer_mrad (int, inside the computed equivalent soft limit);
    anything else stops with invalid_drive. A drive with left/right (differential format) stops with
    differential_command_rejected, before the ownership checks, so a differential client can never move it.
  * steer-only commands (throttle 0) still need the lease, the ticket, seq, deadman and the armed state.
  * requested.left/right carry the throttle percent so the pinned motor_pwm.py still records the drive duty.

There is no pivot and no differential mixing: both sides always get the same duty (drive_lr_mode equal_duty).
"""
import secrets
import time

from scripts.rover_control_core import ControlError, RoverControl

VEHICLE_PROFILE = 'front_steer'


def _strict_int(value):
    return type(value) is int


class SteerControl(RoverControl):
    """RoverControl for the front-steer vehicle.

    motor must provide the RoverControl motor interface (ready, stopped, firmware, outputs, error, hard_stop(),
    enable(), release()) plus arm_blocker() -> error code or None, clear_latches() and
    drive_steer(throttle, steer_mrad). status_extra() (optional) returns fields merged into status().
    """

    RELEASE_TTL = 1.1   # the literal of RoverControl.dispatch release

    def __init__(self, motor, clock=time.monotonic, *, steer_limit_left_mrad, steer_limit_right_mrad,
                 status_extra=None):
        super().__init__(motor, clock)
        for name, value in (('steer_limit_left_mrad', steer_limit_left_mrad),
                            ('steer_limit_right_mrad', steer_limit_right_mrad)):
            if not _strict_int(value) or not 0 < value < 1571:
                raise ValueError(name + '_must_be_a_positive_int_mrad')
        self.steer_limit_left_mrad = steer_limit_left_mrad
        self.steer_limit_right_mrad = steer_limit_right_mrad
        self.status_extra = status_extra
        self.throttle = 0
        self.steer_mrad = None

    # -- state
    def stop(self, reason='operator_stop'):
        self.throttle = 0
        self.steer_mrad = None
        super().stop(reason)

    def status(self):
        status = super().status()
        status['requested'] = {'throttle': self.throttle, 'steer_mrad': self.steer_mrad,
                               'left': self.requested['left'], 'right': self.requested['right']}
        status['vehicle_profile'] = VEHICLE_PROFILE
        status['steer_limit_mrad'] = {'left': self.steer_limit_left_mrad, 'right': self.steer_limit_right_mrad}
        if self.status_extra is not None:
            status.update(self.status_extra())
        return status

    # -- requests
    def dispatch(self, request):
        op = request.get('op')
        if op == 'drive' and ('left' in request or 'right' in request):
            self.tick()
            self.stop('differential_command_rejected')
            raise ControlError('differential_command_rejected')
        if op == 'arm':
            return self._arm(request)
        if op == 'drive':
            return self._drive(request)
        if op == 'release':
            return self._release(request)
        return super().dispatch(request)

    def _check_owner(self, request):
        owner = request.get('owner')
        if not isinstance(owner, str) or not owner:
            raise ControlError('operator_required')
        return owner

    def _check_owned(self, request):
        """The ownership, ticket and sequence rules of RoverControl.dispatch, in the same order."""
        owner = self._check_owner(request)
        if not self.lease or owner != self.owner or request.get('lease') != self.lease:
            raise ControlError('control_not_owned')
        if request.get('ticket') != self.ticket or self.clock() >= self.ticket_until:
            self.stop('stale_input')
            raise ControlError('stale_input_rearm_required')
        seq = request.get('seq')
        if type(seq) is not int or seq != self.seq + 1:
            self.stop('invalid_sequence')
            raise ControlError('invalid_sequence')
        return seq

    def _arm(self, request):
        self.tick()
        owner = self._check_owner(request)
        if request.get('vehicle_profile') != VEHICLE_PROFILE:
            raise ControlError('steer_profile_required')
        if self.lease:
            raise ControlError('control_in_use')
        blocker = self.motor.arm_blocker()
        if blocker:
            raise ControlError(blocker)
        # Operator re-arm (all arm conditions passed): the host latches go when the MCU confirms this ARM, and
        # only then (spec 12.1-75); a refused or unconfirmed ARM keeps them.
        self.motor.clear_latches()
        self.motor.hard_stop(for_arm=True)
        self.motor.enable()
        self.owner, self.lease = owner, secrets.token_urlsafe(24)
        self.owner_until = self.clock() + self.OWNER_TTL
        self.seq = 0
        self.throttle = 0
        self.steer_mrad = None
        self.requested = {'left': 0, 'right': 0}
        self.moving = False
        self.reason = 'operator_armed'
        return self._grant_ticket()

    def _release(self, request):
        """RoverControl release (MCU ramp to zero, 1.1 s completion bound); the steering target is kept."""
        self.tick()
        seq = self._check_owned(request)
        self.motor.release()
        self.throttle = 0
        self.requested = {'left': 0, 'right': 0}
        self.moving = True
        self.input_until = self.clock() + self.RELEASE_TTL
        self.seq = seq
        self.owner_until = self.clock() + self.OWNER_TTL
        return self._grant_ticket()

    def _drive(self, request):
        self.tick()
        seq = self._check_owned(request)
        throttle, steer = request.get('throttle'), request.get('steer_mrad')
        if (request.get('deadman') is not True or not _strict_int(throttle) or not -100 <= throttle <= 100
                or not _strict_int(steer) or not -self.steer_limit_right_mrad <= steer <= self.steer_limit_left_mrad):
            self.stop('invalid_drive')
            raise ControlError('invalid_drive')
        self.motor.drive_steer(throttle, steer)
        self.throttle, self.steer_mrad = throttle, steer
        self.requested = {'left': throttle, 'right': throttle}
        self.moving = bool(throttle)
        self.input_until = self.clock() + self.INPUT_TTL
        self.seq = seq
        self.owner_until = self.clock() + self.OWNER_TTL
        return self._grant_ticket()
