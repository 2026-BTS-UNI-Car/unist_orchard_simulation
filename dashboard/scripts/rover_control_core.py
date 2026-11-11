"""Manual ownership and monotonic input expiry. No hardware or ROS imports."""
import secrets
import time


class ControlError(ValueError):
    pass


class RoverControl:
    INPUT_TTL = 0.75
    TICKET_TTL = 1.5
    OWNER_TTL = 30.0

    def __init__(self, motor, clock=time.monotonic):
        self.motor, self.clock = motor, clock
        self.lease = self.owner = self.ticket = None
        self.ticket_until = self.input_until = self.owner_until = 0.0
        self.seq = 0
        self.moving = False
        self.reason = "startup"
        self.requested = {"left": 0, "right": 0}

    def stop(self, reason="operator_stop"):
        self.lease = self.owner = self.ticket = None
        self.moving = False
        self.requested = {"left": 0, "right": 0}
        self.reason = reason
        self.motor.hard_stop()

    def tick(self):
        now = self.clock()
        if self.moving and not any(self.requested.values()) and self.motor.stopped:
            self.moving = False
        if self.lease and not self.motor.ready:
            self.stop("device_unavailable")
        elif self.moving and now >= self.input_until:
            self.stop("input_expired")
        elif self.lease and now >= self.owner_until:
            self.stop("operator_expired")

    def status(self):
        return {"ok": True, "state": "armed_manual" if self.lease else "disarmed",
                "connected": bool(self.motor.ready), "firmware": self.motor.firmware,
                "armed": bool(self.lease), "moving": self.moving,
                "requested": dict(self.requested), "reason": self.reason,
                "input_ttl_s": self.INPUT_TTL, "automatic_input_enabled": False,
                "outputs": dict(self.motor.outputs), "last_error": self.motor.error}

    def _grant_ticket(self):
        self.ticket = secrets.token_urlsafe(24)
        self.ticket_until = self.clock() + self.TICKET_TTL
        return {"ok": True, "lease": self.lease, "ticket": self.ticket, "seq": self.seq,
                "ticket_ttl_s": self.TICKET_TTL, "status": self.status()}

    def dispatch(self, request):
        self.tick()
        op = request.get("op")
        if op == "status":
            return self.status()
        if op == "stop":
            self.stop(str(request.get("reason", "operator_stop"))[:80])
            return self.status()
        owner = request.get("owner")
        if not isinstance(owner, str) or not owner:
            raise ControlError("operator_required")
        if op == "arm":
            if self.lease:
                raise ControlError("control_in_use")
            if not self.motor.ready:
                raise ControlError("verified_esp_v2_required")
            self.motor.hard_stop()
            self.motor.enable()
            self.owner, self.lease = owner, secrets.token_urlsafe(24)
            self.owner_until = self.clock() + self.OWNER_TTL
            self.seq = 0
            self.reason = "operator_armed"
            return self._grant_ticket()
        if not self.lease or owner != self.owner or request.get("lease") != self.lease:
            raise ControlError("control_not_owned")
        if request.get("ticket") != self.ticket or self.clock() >= self.ticket_until:
            self.stop("stale_input")
            raise ControlError("stale_input_rearm_required")
        seq = request.get("seq")
        if type(seq) is not int or seq != self.seq + 1:
            self.stop("invalid_sequence")
            raise ControlError("invalid_sequence")
        if op not in {"drive", "keepalive", "release"}:
            raise ControlError("unknown_operation")
        left, right = request.get("left"), request.get("right")
        if op == "drive":
            if request.get("deadman") is not True or any(type(v) is not int or not -100 <= v <= 100 for v in (left, right)):
                self.stop("invalid_drive")
                raise ControlError("invalid_drive")
            self.motor.drive(left, right)
            self.requested = {"left": left, "right": right}
            self.moving = bool(left or right)
            self.input_until = self.clock() + self.INPUT_TTL
        elif op == "release":
            self.motor.release()
            self.requested = {"left": 0, "right": 0}
            self.moving = True
            self.input_until = self.clock() + 1.1
        self.seq = seq
        self.owner_until = self.clock() + self.OWNER_TTL
        return self._grant_ticket()
