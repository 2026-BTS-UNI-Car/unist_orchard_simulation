"""Simulation-only autonomous yaw-rate servo using IMU-derived feedback.

Corrects measured yaw-rate tracking error using IMU-derived yaw rate only.
The planner still owns path selection, obstacle checks, authorization and STOP.
These gains are simulation assumptions and require physical Gazebo validation.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class YawCommand:
    linear: float
    angular: float
    left_m_s: float
    right_m_s: float
    valid: bool
    reason: str
    saturated: bool = False


class YawRateTracker:
    def __init__(self, track=.56, max_wheel=1., kp=2., ki=1.,
                 integral_limit=.8, max_yaw_command=1.2, stale_s=.10,
                 max_step_s=.25, future_tolerance_s=1/30):
        parameters=(track,max_wheel,kp,ki,integral_limit,max_yaw_command,stale_s,max_step_s,future_tolerance_s)
        if not all(math.isfinite(v) and v > 0 for v in parameters):
            raise ValueError('invalid_yaw_tracker_configuration')
        self.track,self.max_wheel,self.kp,self.ki=track,max_wheel,kp,ki
        self.integral_limit,self.max_yaw_command=integral_limit,max_yaw_command
        self.stale_s,self.max_step_s=stale_s,max_step_s
        self.future_tolerance_s=future_tolerance_s
        self.reset()

    def reset(self):
        self.integral=0.
        self.last_time=None
        self.last_desired=0.
        self.fault=None

    def stop(self, reason='stopped'):
        self.integral=0.
        self.last_time=None
        self.last_desired=0.
        return YawCommand(0.,0.,0.,0.,True,reason)

    def _fail(self, reason):
        self.stop(reason)
        self.fault=reason
        return YawCommand(0.,0.,0.,0.,False,reason)

    def reject_measurement(self, reason):
        """Latch an external arrival-time fault through the same zero path."""
        return self._fail(reason)

    def step(self, linear, desired_yaw_rate, measured_yaw_rate,
             sim_time, measurement_stamp, *, authorized):
        if self.fault:
            return YawCommand(0.,0.,0.,0.,False,self.fault)
        if not authorized:
            return self._fail('authorization_lost')
        values=(linear,desired_yaw_rate,measured_yaw_rate,sim_time,measurement_stamp)
        if not all(math.isfinite(v) for v in values) or sim_time < 0:
            return self._fail('invalid_yaw_sample')
        # The upstream NumPy path tracker returns NumPy scalars. Normalize at
        # this boundary so status flags and values always serialize as JSON.
        linear,desired_yaw_rate,measured_yaw_rate,sim_time,measurement_stamp=map(float,values)
        age=sim_time-measurement_stamp
        if age < -self.future_tolerance_s-1e-9:
            return self._fail('future_yaw_rate')
        if age > self.stale_s:
            return self._fail('stale_yaw_rate')
        # Separate bridge callbacks may deliver a 30 Hz odometry message
        # before its 120 Hz /clock update (observed skew: one 8.33 ms tick).
        # Bound that delivery skew to one odometry period, then integrate at
        # the newest observed physics stamp. Catch-up clock callbacks cannot
        # rewind or double-count the PI interval. Larger future offsets fail.
        control_time=max(sim_time,measurement_stamp)
        if abs(linear) < 1e-9 and abs(desired_yaw_rate) < 1e-9:
            # An explicit stop never fights residual motion with reverse torque.
            return self.stop('zero_command')
        dt=0. if self.last_time is None else control_time-self.last_time
        if dt < 0:
            return self._fail('clock_reset')
        if dt > self.max_step_s:
            return self._fail('control_time_gap')
        if (desired_yaw_rate*self.last_desired < 0 and
                abs(desired_yaw_rate) > .02 and abs(self.last_desired) > .02):
            self.integral=0.
        if abs(desired_yaw_rate) < .015 and abs(measured_yaw_rate) < .03:
            self.integral=0.
        self.last_time,self.last_desired=control_time,desired_yaw_rate
        velocity=max(-self.max_wheel,min(self.max_wheel,linear))
        # Preserve requested forward speed and use only the remaining wheel
        # authority for yaw. Both wheels stay within the physical command bound.
        limit=min(self.max_yaw_command,2*(self.max_wheel-abs(velocity))/self.track)
        error=desired_yaw_rate-measured_yaw_rate
        proposed=max(-self.integral_limit,min(self.integral_limit,self.integral+self.ki*error*dt))
        unconstrained=desired_yaw_rate+self.kp*error+proposed
        # Conditional integration prevents growing the integral into saturation;
        # integration that unwinds an existing saturation remains allowed.
        if not (abs(unconstrained) > limit and unconstrained*error > 0):
            self.integral=proposed
        unconstrained=desired_yaw_rate+self.kp*error+self.integral
        angular=max(-limit,min(limit,unconstrained))
        left,right=velocity-angular*self.track/2,velocity+angular*self.track/2
        saturated=bool(abs(angular-unconstrained) > 1e-9 or velocity != linear)
        return YawCommand(velocity,angular,left,right,True,'tracking',saturated)
