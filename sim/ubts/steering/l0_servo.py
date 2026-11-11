"""L0 ideal steering servo backend (design 3.3 level L0) behind the SteeringBackend contract.

L0 is the comparison baseline of the steering actuator models (design 3.3): the steering
follows the commanded equivalent angle with a rate limit and a first-order lag. It is used
for the first checks (A1, A2 for rate and lag, A21, A12). It is NOT the default actuator
model of the design (L1): SteeredPlant takes it by default only when the profile's
steer_actuator_level is L0 (geometry_check sets L0); otherwise it gives the native L1
backend. Results with it carry level L0 in SteeredPlant.result_label().

Mode: test_servo only (protocol v1, docs/steering/LOCKSTEP_PROTOCOL_KO.md section 6: "L0 is
only possible in test_servo mode"). The plant gives target_delta_eq_rad each step.

State and angles (protocol v1 section 7, all on the steering-equivalent scale, rad, + left):
  theta     pot-side angle = motor-side angle = link-side angle (L0 has no backlash)
  m         T(theta), the link table output; the test servo controls m (it does not know
            the zero offset, protocol section 6)
  delta_eq  m * (1 + lr * sign(m)) + zero_offset, the true wheel-side equivalent angle
  end stop  delta_eq in [-eq_mech_limit_right, +eq_mech_limit_left] (geometry.py); at the
            stop theta is held, so neither wheel passes its own mechanical limit (A21)

Dynamics: d(theta)/dt = clip((theta_target - theta) / tau, -rate, +rate) with
theta_target = T^-1(target), tau = steer_motor_tau_s and rate = steer_rate_max_deg_s while
moving or steer_rate_standstill_deg_s when vehicle_stopped. Each step integrates exactly
over ticks / mcu_tick_hz (cumulative tick rule of the protocol, section 5): first the
rate-limited part, then the exponential part. No deadband, no backlash, no motor model, no
faults, no fault injections. With steer_en False the angle stays where it is (L0 has no
release model; the non-self-locking release of design 3.3 belongs to the L1 backend).

Potentiometer: error free (no noise, gain or linearity error, drift, supply sag or
calibration residual). pot_raw = clamp(round_half_away(center_raw + g * theta), 0, 2^bits - 1)
with g = 2^bits * pot_to_steer_ratio / pot_range (counts per rad), the nominal mapping of
protocol section 7 with every error term zero. pot_location motor and wheel read the same
angle because L0 has no backlash.

applied_steer_pwm: L0 has no motor, so this is an equivalent effort at the end of the step,
clip((theta_target - theta) / tau, -rate, rate) / rate (so +-1 while rate limited and also
while pushing against an end stop); 0 with EN off. The native L0 level reports the last
tick's step over its rate limit, which is close to this but not bit-identical.

The mechanical mapping (link table, left/right asymmetry, per-run zero offset, end stops)
is the same as in protocol section 7, so the plant sees the same geometry with L0 and with
the native backend. The zero offset is the per-run draw profile.sample(seed) of
steer_zero_offset_deg (the native startup line uses the same rule), or the plain value when
a scenario override fixed it.

Pure Python (math only). Placeholder values; results are placeholder-based SIM results.
"""
from __future__ import annotations

import math

from . import backend_api as ba
from . import geometry as geo

LEVEL = 'L0'
_U64_MAX = 0xFFFFFFFFFFFFFFFF
_END_STOP_TOLERANCE_RAD = 1e-12


def round_half_away(value):
    """C/C++ llround(): halves go away from zero (Python round() would go to even)."""
    return int(math.floor(value + .5)) if value >= 0. else -int(math.floor(-value + .5))


def per_run_value(profile, seed, name):
    """The per-run value of a profile parameter: the seeded draw for a distribution, the plain value otherwise
    (VehicleProfile.per_run_value; the native startup line uses the same rule)."""
    return profile.per_run_value(seed, name)


def approach(theta, target, rate, tau, span):
    """Exact solution of d(theta)/dt = clip((target - theta) / tau, -rate, rate) after `span` seconds.

    tau <= 0 gives a pure rate limit. rate must be positive.
    """
    error = target - theta
    if error == 0. or span <= 0.:
        return theta
    sign = 1. if error > 0. else -1.
    size = abs(error)
    if tau <= 0.:
        return target if size <= rate * span else theta + sign * rate * span
    knee = rate * tau                      # below this error the motion is exponential
    if size > knee:
        rate_time = (size - knee) / rate   # time spent at the rate limit
        if rate_time >= span:
            return theta + sign * rate * span
        span -= rate_time
        size = knee
    return target - sign * size * math.exp(-span / tau)


class L0ServoBackend(ba.SteeringBackend):
    """L0 ideal steering servo; implements backend_api.SteeringBackend (test_servo mode only).

    L0ServoBackend(mode='test_servo', mcu_tick_hz=None): mcu_tick_hz None uses the profile
    sim_mcu_tick_hz. configure(profile, seed) must be called before reset() and step().
    """

    LEVEL = LEVEL
    MODES = ('test_servo',)

    def __init__(self, mode='test_servo', mcu_tick_hz=None):
        if mode not in self.MODES:
            raise ValueError('l0_backend_supports_only_test_servo_mode:%r' % (mode,))
        if mcu_tick_hz is not None and (isinstance(mcu_tick_hz, bool) or not isinstance(mcu_tick_hz, int)
                                        or not 100 <= mcu_tick_hz <= 100000):
            raise ValueError('l0_tick_hz_invalid')
        self.mode = mode
        self._tick_override = mcu_tick_hz
        self._configured = False
        self._closed = False

    # -- SteeringBackend -------------------------------------------------------------------------------------------
    def configure(self, profile, seed):
        if self._closed:
            raise RuntimeError('l0_backend_closed')
        if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed <= _U64_MAX:
            raise ValueError('l0_seed_must_be_u64')
        v = profile.value
        tick = v('sim_mcu_tick_hz') if self._tick_override is None else self._tick_override
        if isinstance(tick, bool) or not isinstance(tick, int) or not 100 <= tick <= 100000:
            raise ValueError('l0_tick_hz_invalid')
        geometry = geo.SteeringGeometry.from_profile(profile)
        self.geometry = geometry
        self.link = geometry.link_table
        self.tick_hz = tick
        self.mech_left, self.mech_right = geometry.mech_limits_eq()
        self.rate_moving = math.radians(v('steer_rate_max_deg_s'))
        self.rate_standstill = math.radians(v('steer_rate_standstill_deg_s'))
        self.tau = float(v('steer_motor_tau_s'))
        if self.rate_moving <= 0. or self.rate_standstill <= 0. or self.tau < 0.:
            raise ValueError('l0_rate_or_tau_invalid')
        self.lr = float(v('steer_lr_asymmetry'))
        if not -1. < self.lr < 1.:
            raise ValueError('l0_lr_asymmetry_out_of_range')
        self.self_locking = bool(v('steer_self_locking'))
        self.pot_location = v('pot_location')
        bits = v('adc_bits')
        self.adc_top = 2 ** bits - 1
        self.pot_gain = 2. ** bits * v('pot_to_steer_ratio') / math.radians(v('pot_range_deg'))
        self.pot_center = float(v('pot_center_raw'))
        self.zero_offset = math.radians(per_run_value(profile, seed, 'steer_zero_offset_deg'))
        self.profile_level = v('steer_actuator_level')
        self.profile_id = profile.profile_id
        self.profile_sha256 = profile.sha256
        self.seed = seed
        self.theta_max = self._theta_for_eq(self.mech_left)
        self.theta_min = self._theta_for_eq(-self.mech_right)
        if not self.theta_min < self.theta_max:
            raise ValueError('l0_zero_offset_outside_the_mechanical_range')
        self._configured = True
        self.reset(0.)          # protocol: the state after configuration equals RESET(0.0)

    def reset(self, delta_eq_rad=0.):
        self._require()
        if isinstance(delta_eq_rad, bool) or not isinstance(delta_eq_rad, (int, float)):
            raise ValueError('l0_reset_delta_not_a_number')
        delta = float(delta_eq_rad)
        if not math.isfinite(delta) or not (-self.mech_right - _END_STOP_TOLERANCE_RAD <= delta
                                            <= self.mech_left + _END_STOP_TOLERANCE_RAD):
            raise ValueError('l0_reset_delta_outside_the_mechanical_limits')
        self.theta = min(self.theta_max, max(self.theta_min, self._theta_for_eq(delta)))
        self.plant_time = 0.
        self.ticks_total = 0
        self.pwm = 0.
        self.steer_en = False
        return self._output(0)

    def step(self, inp):
        self._require()
        if not isinstance(inp, ba.SteerStepInput):
            raise TypeError('l0_step_needs_a_SteerStepInput')
        inp.validate()
        if inp.mode != self.mode:
            raise ValueError('l0_step_mode_%s_but_backend_mode_%s' % (inp.mode, self.mode))
        self.plant_time += inp.dt_s
        target_ticks = round_half_away(self.plant_time * self.tick_hz)
        ticks = target_ticks - self.ticks_total
        self.ticks_total = target_ticks
        self.steer_en = inp.steer_en
        if ticks <= 0:
            return self._output(0)
        span = ticks / float(self.tick_hz)
        rate = self.rate_standstill if inp.vehicle_stopped else self.rate_moving
        if inp.steer_en:
            target = self.link.pot_from_wheel(inp.target_delta_eq_rad)
            new = approach(self.theta, target, rate, self.tau, span)
            new = min(self.theta_max, max(self.theta_min, new))
            error = target - new
            if self.tau > 0.:
                self.pwm = max(-1., min(1., error / self.tau / rate))
            else:
                self.pwm = 0. if error == 0. else math.copysign(1., error)
        else:
            new = self.theta
            self.pwm = 0.
        self.theta = new
        return self._output(ticks)

    def close(self):
        self._closed = True

    # -- information -----------------------------------------------------------------------------------------------
    def info(self):
        """Backend identity and per-run values (for run records; not an executor input)."""
        self._require()
        return dict(backend='L0ServoBackend', level=LEVEL, mode=self.mode, mcu_tick_hz=self.tick_hz,
                    seed=self.seed, profile_id=self.profile_id, profile_sha256=self.profile_sha256,
                    profile_actuator_level=self.profile_level, zero_offset_rad=self.zero_offset,
                    eq_mech_limit_left_rad=self.mech_left, eq_mech_limit_right_rad=self.mech_right,
                    potentiometer='error free (nominal mapping)',
                    not_modelled=['deadband', 'backlash', 'motor', 'release', 'faults', 'injections',
                                  'pot noise and errors'])

    # -- internals -------------------------------------------------------------------------------------------------
    def _require(self):
        if self._closed:
            raise RuntimeError('l0_backend_closed')
        if not self._configured:
            raise RuntimeError('l0_backend_not_configured')

    def _eq_from_theta(self, theta):
        m = self.link.wheel_from_pot(theta)
        sign = 1. if m > 0. else (-1. if m < 0. else 0.)
        return m * (1. + self.lr * sign) + self.zero_offset

    def _theta_for_eq(self, delta):
        u = delta - self.zero_offset
        m = u / (1. + self.lr) if u >= 0. else u / (1. - self.lr)
        return self.link.pot_from_wheel(m)

    def _output(self, ticks):
        delta = min(self.mech_left, max(-self.mech_right, self._eq_from_theta(self.theta)))
        left, right = self.geometry.wheel_angles(delta)
        flags = 0
        if self.theta >= self.theta_max - _END_STOP_TOLERANCE_RAD:
            flags |= ba.ModelFlag.END_STOP_LEFT
        if self.theta <= self.theta_min + _END_STOP_TOLERANCE_RAD:
            flags |= ba.ModelFlag.END_STOP_RIGHT
        if not self.steer_en and not self.self_locking:
            flags |= ba.ModelFlag.FREE_WHEELING
        raw = round_half_away(self.pot_center + self.pot_gain * self.theta)
        if raw <= 0 or raw >= self.adc_top:
            raw = min(self.adc_top, max(0, raw))
            flags |= ba.ModelFlag.POT_CLIPPED
        return ba.SteerStepOutput(sim_time_s=self.ticks_total / float(self.tick_hz), ticks=ticks,
                                  delta_left_rad=left, delta_right_rad=right, delta_eq_rad=delta,
                                  motor_side_angle_rad=self.theta, pot_raw=raw, applied_steer_pwm=self.pwm,
                                  steer_en=self.steer_en, fault_bits=0, model_flags=flags)
