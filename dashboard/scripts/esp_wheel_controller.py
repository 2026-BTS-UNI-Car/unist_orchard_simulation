"""Controller from rover_control_test/remote_helper.py, V2 baseline."""
import time
PIN_GPIO = {"D0": 16, "D1": 5, "D2": 4, "D3": 0, "D4": 2, "D5": 14, "D6": 12, "D7": 13, "D8": 15}
_UNSET = object()
HEARTBEAT_TIMEOUT_MS = 2000

def emit(kind, **values):
    pass

class PulseController:
    """Timed or held outputs; both stop when the PC heartbeat is lost."""

    def __init__(self, write, event=emit, clock=time.monotonic):
        self.write = write
        self.event = event
        self.clock = clock
        self.active = {}
        self.last_heartbeat = clock()
        self.last_heartbeat_gap_ms = 0
        self.max_heartbeat_gap_ms = 0
        self.heartbeat_count = 0

    def set_pwm(self, pin, value, duration_ms):
        if pin not in PIN_GPIO:
            raise ValueError("Unknown NodeMCU pin")
        if type(value) is not int or not 0 <= value <= 255:
            raise ValueError("PWM must be an integer in 0..255")
        if duration_ms is not None and (type(duration_ms) is not int or not 100 <= duration_ms <= 5000):
            raise ValueError("Duration must be null (hold) or an integer in 100..5000 ms")
        if value:
            self.active[pin] = None if duration_ms is None else self.clock() + duration_ms / 1000.0
        # A partial write can still reach the MCU; track it before writing so
        # final cleanup will attempt a zero even when the write raises.
        self._send(pin, value, "user")
        if not value:
            self.active.pop(pin, None)

    def _send(self, pin, value, reason):
        self.write(f"{pin} {value}\n".encode("ascii"))
        self.event("tx", pin=pin, value=value, reason=reason)

    def heartbeat(self):
        # Expired output must be stopped before a delayed heartbeat can renew
        # the lease; this also covers multiple messages in one stdin batch.
        gap_ms = round((self.clock() - self.last_heartbeat) * 1000)
        self.last_heartbeat_gap_ms = gap_ms
        self.max_heartbeat_gap_ms = max(self.max_heartbeat_gap_ms, gap_ms)
        self.heartbeat_count += 1
        self.check_heartbeat()
        self.last_heartbeat = self.clock()

    def heartbeat_health(self):
        current_gap_ms = round((self.clock() - self.last_heartbeat) * 1000)
        return {"gap_ms": self.last_heartbeat_gap_ms,
                "current_gap_ms": current_gap_ms,
                "max_gap_ms": max(self.max_heartbeat_gap_ms, current_gap_ms),
                "count": self.heartbeat_count, "limit_ms": HEARTBEAT_TIMEOUT_MS}

    def check_heartbeat(self):
        gap_ms = round((self.clock() - self.last_heartbeat) * 1000)
        if self.active and (self.clock() - self.last_heartbeat) * 1000 > HEARTBEAT_TIMEOUT_MS:
            try:
                self.stop("heartbeat_timeout")
            finally:
                self.event("watchdog", gap_ms=gap_ms, limit_ms=HEARTBEAT_TIMEOUT_MS, source="pc_heartbeat",
                           message=f"PC heartbeat expired ({gap_ms} ms gap); active pins set to zero")
            return True
        return False

    def stop(self, reason="stop"):
        pins = list(self.active)
        # Keep failed pins active so a subsequent best-effort stop can retry.
        errors = []
        for pin in pins:
            try:
                self._send(pin, 0, reason)
                self.active.pop(pin, None)
            except OSError as exc:
                errors.append(exc)
        if errors:
            raise errors[0]

    def tick(self):
        now = self.clock()
        if self.check_heartbeat():
            return
        for pin, deadline in list(self.active.items()):
            if deadline is not None and now >= deadline:
                self._send(pin, 0, "pulse_complete")
                self.active.pop(pin, None)


class WheelController:
    """Single serial owner, with break-before-make wheel direction changes."""

    DIRECTION_PINS = {"left": ("D8", "D5"), "right": ("D3", "D0")}
    POWER_PINS = ("D1", "D2", "D6", "D7")
    CHANGE_DELAY = 0.120

    def __init__(self, controller):
        self.controller = controller
        self.event = controller.event
        self.clock = controller.clock
        self.enabled = False
        self.engaged = False
        self.powered = False
        self.supports_pwm = False
        self.supports_ramp = False
        self.firmware_name = None
        self._mcu_watchdog_latched = False
        self.on_value = 100
        self.pending = {}

    def observe_firmware(self, line):
        """Negotiate actual PWM support from the connected MCU's response."""
        name = next((name for name in ("UBTS_WHEEL_PWM_V2", "UBTS_WHEEL_PWM_V1", "UBTS_PWM_V1")
                     if line.startswith((name + " ").encode("ascii"))), None)
        if name is None:
            return False
        self.supports_pwm = True
        self.supports_ramp = name == "UBTS_WHEEL_PWM_V2"
        self.on_value = 255
        if self.firmware_name != name:
            self.firmware_name = name
            self.event("firmware", name=name, supports_pwm=True, **({"supports_ramp": True} if self.supports_ramp else {}))
        if line.rstrip(b"\r") in (b"UBTS_WHEEL_PWM_V1 WATCHDOG stopped", b"UBTS_WHEEL_PWM_V2 WATCHDOG stopped"):
            if not self._mcu_watchdog_latched:
                # Set the latch before writes: zero acknowledgements and a
                # repeated notification must not generate another stop cycle.
                self._mcu_watchdog_latched = True
                try:
                    self.stop_all("mcu_watchdog")
                finally:
                    self.event("watchdog", message="ESP watchdog stopped the wheels; control disabled")
        return True

    def _zero_side(self, side, reason):
        self.pending.pop(side, None)
        errors = []
        for pin in self.DIRECTION_PINS[side]:
            # Track even an unconfirmed zero, so final cleanup retries it.
            self.controller.active[pin] = None
            try:
                self.controller._send(pin, 0, reason)
                self.controller.active.pop(pin, None)
            except OSError as exc:
                errors.append(exc)
        if errors:
            raise errors[0]

    def enable(self, enabled, on_value=100):
        if type(enabled) is not bool:
            raise ValueError("Wheel enable must be true or false")
        if type(on_value) is not int or on_value not in (100, 255):
            raise ValueError("Wheel ON value must be 100 or 255")
        if not enabled:
            self.stop("wheel_disabled", disable=True)
            return
        if (self.clock() - self.controller.last_heartbeat) * 1000 > HEARTBEAT_TIMEOUT_MS:
            raise RuntimeError("Refresh the PC heartbeat before enabling wheels")
        self.enabled = False
        self.pending.clear()
        self.engaged = True
        try:
            # Cancel every old raw output/timer before taking ownership.
            self.controller.stop("wheel_enable")
            self.stop("wheel_enable")
            # The client cannot assert capabilities by choosing ON=255.
            self.on_value = 255 if self.supports_pwm else 100
            for pin in self.POWER_PINS:
                self.controller._send(pin, self.on_value, "wheel_power")
            self.powered = True
            self.enabled = True
            self._mcu_watchdog_latched = False
            self.event("wheel_mode", enabled=True, reason="wheel_enable")
        except (OSError, RuntimeError):
            self.enabled = False
            self.pending.clear()
            try:
                self.stop_all("wheel_enable_failed")
            except (OSError, RuntimeError):
                # Keep the original enable failure as the reported error.
                pass
            self.event("wheel_mode", enabled=False, reason="wheel_enable_failed")
            raise

    def set_pwm(self, pin, value, duration_ms):
        if self.enabled:
            raise RuntimeError("Disable wheel control before sending individual pin commands")
        self.controller.set_pwm(pin, value, duration_ms)
        if value:
            self._mcu_watchdog_latched = False

    def command(self, left=None, right=None, left_speed=_UNSET, right_speed=_UNSET):
        requested = {side: value for side, value in (("left", left), ("right", right))
                     if value is not None}
        if not requested or any(value not in ("forward", "reverse", "stop")
                                for value in requested.values()):
            raise ValueError("Wheel direction must be forward, reverse, or stop")
        speeds = {}
        for side, speed in (("left", left_speed), ("right", right_speed)):
            if speed is not _UNSET and (type(speed) is not int or not 0 <= speed <= 100):
                raise ValueError("Wheel speed must be an integer in 0..100 percent")
            if side not in requested:
                if speed is not _UNSET:
                    raise ValueError("Wheel speed requires a direction for the same wheel")
                continue
            speed = 100 if speed is _UNSET else speed
            if requested[side] == "stop" or speed == 0:
                requested[side] = "stop"
                speed = 0
            elif speed != 100 and not self.supports_pwm:
                raise RuntimeError("Variable wheel speed requires verified PWM firmware")
            speeds[side] = speed
        moving = any(value != "stop" for value in requested.values())
        if moving and not self.enabled:
            raise RuntimeError("Enable wheel control first")
        if moving and (self.clock() - self.controller.last_heartbeat) * 1000 > HEARTBEAT_TIMEOUT_MS:
            self.stop_all("heartbeat_timeout")
            raise RuntimeError("PC heartbeat expired; wheels disabled")
        if moving:
            self._mcu_watchdog_latched = False
        self.engaged = True
        try:
            for side, direction in requested.items():
                if self.supports_ramp:
                    # The MCU performs the actual ramp-to-zero and direction
                    # interlock. Do not insert a host-side zero/delay on every
                    # same-direction speed update.
                    self.pending.pop(side, None)
                    if direction == "stop":
                        self._zero_side(side, "wheel_soft_stop")
                    else:
                        forward, reverse = self.DIRECTION_PINS[side]
                        pin, opposite = (forward, reverse) if direction == "forward" else (reverse, forward)
                        self.controller._send(opposite, 0, "wheel_target")
                        self.controller.active.pop(opposite, None)
                        self.controller.active[pin] = None
                        self.controller._send(pin, (speeds[side] * 255 + 50) // 100, "wheel_target")
                    continue
                # Both low, including when a queued reversal is replaced.
                self._zero_side(side, "wheel_break")
                if direction != "stop":
                    self.pending[side] = (self.clock() + self.CHANGE_DELAY, direction, speeds[side])
            phase = "waiting" if moving and not self.supports_ramp else "applied"
            self.event("wheel_command", phase=phase, **requested,
                       **{side + "_speed": speed for side, speed in speeds.items()})
        except (OSError, RuntimeError):
            self.pending.clear()
            self.enabled = False
            self.event("wheel_mode", enabled=False, reason="wheel_write_failed")
            try:
                self.stop_all("wheel_write_failed")
            except (OSError, RuntimeError):
                pass
            raise

    def stop(self, reason="wheel_stop", disable=False):
        # Clear every delayed make before attempting any serial writes.
        self.pending.clear()
        self.engaged = True
        if disable:
            self.enabled = False
            self.event("wheel_mode", enabled=False, reason=reason)
        errors = []
        if self.supports_ramp and (disable or reason != "wheel_stop"):
            try:
                # STOP bypasses a fading target: emergency/loss/close must not
                # keep coasting through the ordinary deceleration interval.
                self.controller.write(b"STOP\n")
            except (OSError, RuntimeError) as exc:
                errors.append(exc)
        for side in self.DIRECTION_PINS:
            try:
                self._zero_side(side, reason)
            except (OSError, RuntimeError) as exc:
                errors.append(exc)
        if errors:
            self.enabled = False
            self.event("wheel_mode", enabled=False, reason="wheel_stop_failed")
            raise errors[0]
        self.event("wheel_stopped", reason=reason, **({"soft": True} if self.supports_ramp and not disable and reason == "wheel_stop" else {}))

    def stop_all(self, reason="stop"):
        errors = []
        if self.engaged or self.enabled or self.pending or self.supports_ramp:
            try:
                self.stop(reason, disable=True)
            except (OSError, RuntimeError) as exc:
                errors.append(exc)
        else:
            # A queued enable can be canceled client-side before it reaches us.
            # Always acknowledge disabled mode, without touching unused pins.
            self.enabled = False
            self.pending.clear()
            self.event("wheel_mode", enabled=False, reason=reason)
        try:
            self.controller.stop(reason)
        except (OSError, RuntimeError) as exc:
            errors.append(exc)
        if errors:
            raise errors[0]

    def reset_connection(self):
        self.pending.clear()
        self.enabled = self.engaged = self.powered = False
        self.supports_pwm = False
        self.supports_ramp = False
        self.firmware_name = None
        self._mcu_watchdog_latched = False
        self.on_value = 100

    def heartbeat(self):
        self.check_heartbeat()
        self.controller.heartbeat()

    def check_heartbeat(self):
        # A pending change has no active high pin yet, so PulseController's
        # active-only timeout is insufficient: cancel pending work first.
        gap_ms = round((self.clock() - self.controller.last_heartbeat) * 1000)
        if ((self.enabled or self.pending or (self.supports_ramp and self.controller.active))
                and (self.clock() - self.controller.last_heartbeat) * 1000 > HEARTBEAT_TIMEOUT_MS):
            try:
                self.stop_all("heartbeat_timeout")
            finally:
                self.event("watchdog", gap_ms=gap_ms, limit_ms=HEARTBEAT_TIMEOUT_MS, source="pc_heartbeat",
                           message=f"PC heartbeat expired ({gap_ms} ms gap); wheels stopped and disabled")
            return True
        return self.controller.check_heartbeat()

    def tick(self):
        if self.check_heartbeat():
            return
        self.controller.tick()
        if not self.enabled:
            return
        for side, (deadline, direction, speed) in list(self.pending.items()):
            if self.clock() < deadline:
                continue
            self.pending.pop(side, None)
            forward_pin, reverse_pin = self.DIRECTION_PINS[side]
            pin = forward_pin if direction == "forward" else reverse_pin
            self.controller.active[pin] = None
            try:
                value = (speed * 255 + 50) // 100 if self.supports_pwm else 100
                self.controller._send(pin, value, "wheel_make")
            except (OSError, RuntimeError):
                self.pending.clear()
                self.enabled = False
                self.event("wheel_mode", enabled=False, reason="wheel_write_failed")
                try:
                    self.stop_all("wheel_write_failed")
                except (OSError, RuntimeError):
                    pass
                raise
            self.event("wheel_command", phase="applied", **{side: direction, side + "_speed": speed})
