"""Planar wheel-distance / IMU-heading estimator; no truth or GNSS inputs.

Wheel odometry's own orientation is used only to recover signed travel from
its XY increments. It never supplies the output heading. Position remains
dead reckoning and cannot correct forward slip or lateral skid.
"""
from collections import deque
from dataclasses import dataclass
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def attitude(quaternion):
    if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
        raise ValueError('invalid_quaternion')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if norm < 1e-8 or abs(norm-1.) > .05:
        raise ValueError('non_unit_quaternion')
    x,y,z,w = (v/norm for v in quaternion)
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return roll,pitch,yaw


@dataclass(frozen=True)
class PlanarPose:
    stamp: float
    x: float
    y: float
    yaw: float
    forward_velocity: float
    yaw_rate: float


class PlanarOdometry:
    HISTORY_DEPTH = 8

    def __init__(self, max_imu_age=.10, max_wheel_gap=.25, max_sync_wait=.2):
        self.max_imu_age = max_imu_age
        self.max_wheel_gap = max_wheel_gap
        self.max_sync_wait = max_sync_wait
        self.reset()

    def reset(self):
        self.imu = deque(maxlen=64)
        self.initial_heading = None
        self.last_wheel = None
        self.pose = None
        self.fault = None
        self.distance = 0.
        self.pending_wheel = None
        self.pending_wheels = deque()
        self.pending_since = None
        self.wheel_receipts = deque(maxlen=self.HISTORY_DEPTH)
        self.imu_receipts = deque(maxlen=self.HISTORY_DEPTH)
        self.fault_context = None
        self.startup_stable_since = None
        self.startup_previous_receipt = None
        self.startup_previous_stamp = None
        self.startup_motion_detected = False
        self.startup_recoveries = 0
        self.startup_unpaired_discarded = 0

    def _latch_fault(self, reason, wall_time=None):
        if self.fault is None:
            self.fault=reason
            self.fault_context={'reason':reason,'imu_stamp':self.imu[-1][0] if self.imu else None,
                                'wheel_stamp':self.last_wheel[0] if self.last_wheel else None,
                                'pending_stamp':self.pending_wheel[0] if self.pending_wheel else None,
                                'pending_wait_s':wall_time-self.pending_since
                                    if wall_time is not None and self.pending_since is not None else None,
                                'pending_stamps':[sample[0] for sample in self.pending_wheels],
                                'wheel_receipts':list(self.wheel_receipts),
                                'imu_receipts':list(self.imu_receipts)}
        return self.fault

    def add_imu(self, stamp, quaternion, angular_velocity, wall_time=None):
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError('invalid_imu_timestamp')
        if self.imu and stamp <= self.imu[-1][0]:
            return False
        roll,pitch,yaw = attitude(quaternion)
        if len(angular_velocity) != 3 or not all(math.isfinite(v) for v in angular_velocity):
            raise ValueError('invalid_imu_angular_velocity')
        if abs(math.cos(pitch)) < .2:
            raise ValueError('imu_heading_singularity')
        # Body gyro to yaw derivative (Euler ZYX), including small terrain tilt.
        rate = (angular_velocity[1]*math.sin(roll)+angular_velocity[2]*math.cos(roll))/math.cos(pitch)
        if self.initial_heading is None:
            self.initial_heading = yaw
            relative = 0.
        else:
            previous = self.imu[-1][1]
            relative = previous+wrap(yaw-self.initial_heading-previous)
        self.imu.append((stamp,relative,rate))
        self.imu_receipts.append({'stamp':stamp,'receipt_monotonic':wall_time})
        return True

    def heading_at(self, stamp):
        if not self.imu or stamp < self.imu[0][0]-1e-9:
            return None
        for sample in reversed(self.imu):
            if abs(stamp-sample[0]) < 1e-9:
                return sample[1]
        for first,second in zip(self.imu,list(self.imu)[1:]):
            if first[0] <= stamp <= second[0]:
                if second[0]-first[0] > self.max_imu_age:
                    # Latest-only delivery can drop an old IMU frame during
                    # scheduling stalls. Use the nearest actual orientation
                    # plus its gyro only within the same 100 ms prediction
                    # bound; do not interpolate across the entire missing gap.
                    nearest=min((first,second),key=lambda value:abs(stamp-value[0]))
                    delta=stamp-nearest[0]
                    return nearest[1]+delta*nearest[2] if abs(delta)<=self.max_imu_age else None
                fraction = (stamp-first[0])/(second[0]-first[0])
                return first[1]+fraction*(second[1]-first[1])
        latest,yaw,rate = self.imu[-1]
        age = stamp-latest
        if age < -1e-9 or age > self.max_imu_age:
            return None
        # Odom 30 Hz and IMU 20 Hz are asynchronous. Bound gyro prediction to
        # at most 100 ms; never fall back to encoder yaw on stale IMU data.
        return yaw+max(0.,age)*rate

    def queue_wheel(self, stamp, x, y, quaternion, wall_time):
        """Retain measured increments while their bounded IMU history catches up."""
        if self.fault:return None
        if not all(math.isfinite(v) for v in (stamp,x,y,wall_time)) or stamp < 0:
            raise ValueError('invalid_wheel_sample')
        attitude(quaternion)
        if self.last_wheel and stamp <= self.last_wheel[0]:
            return None
        if self.pending_wheels and stamp <= self.pending_wheels[-1][0]:
            return None
        self.wheel_receipts.append({'stamp':stamp,'receipt_monotonic':wall_time})
        # Never silently overwrite an unintegrated increment. Saturation is a
        # fault, not permission to interpolate over an unobserved trajectory.
        if len(self.pending_wheels) >= self.HISTORY_DEPTH:
            if self.pose is None:
                # No origin/increment exists before the first paired sample.
                # Keep bounded startup observations without inventing motion.
                self.pending_wheels.popleft();self.startup_unpaired_discarded+=1
            else:
                self._latch_fault('wheel_history_overflow',wall_time)
                return None
        self.pending_wheels.append((stamp,x,y,quaternion))
        self.pending_wheel=self.pending_wheels[0]
        if self.pending_since is None:
            self.pending_since=wall_time
        return self.flush_wheel(wall_time)

    def expire_pending(self, wall_time):
        if self.fault:return self.fault
        if (self.pose is not None and self.pending_since is not None and
                wall_time-self.pending_since > self.max_sync_wait):
            self._latch_fault('imu_heading_stale',wall_time)
        return self.fault

    def flush_wheel(self, wall_time):
        if self.fault or self.pending_wheel is None:
            return None
        # A late IMU cannot renew the original acquisition deadline, even if
        # it now supplies a heading for the oldest queued wheel message.
        if self.expire_pending(wall_time):
            return None
        result=None
        while self.pending_wheels:
            self.pending_wheel=self.pending_wheels[0]
            if self.pose is None and self.imu and self.pending_wheel[0]<self.imu[0][0]-1e-9:
                # An earlier wheel cannot be paired with an IMU reference that
                # did not exist yet. Initialize only at a real later pair.
                self.pending_wheels.popleft();self.startup_unpaired_discarded+=1
                continue
            if self.heading_at(self.pending_wheel[0]) is None:
                break
            # Each real measured increment still obeys the original250ms
            # limit and the original100ms orientation prediction bound.
            initializing=self.pose is None
            result=self.update_wheel(*self.pending_wheel)
            if self.fault:
                return None
            if initializing:self.pending_since=wall_time
            self.pending_wheels.popleft()
        self.pending_wheel=self.pending_wheels[0] if self.pending_wheels else None
        if self.pending_wheel is None:self.pending_since=None
        return result

    def control_pair_ready(self, sim_time):
        """Queued history may update pose, but stale history cannot authorize motion."""
        return (self.fault is None and self.pose is not None and self.pending_wheel is None
                and math.isfinite(sim_time)
                and -1/30-1e-9 <= sim_time-self.pose.stamp <= self.max_imu_age)

    def recover_stationary_startup(self, stamp, x, y, quaternion, wall_time, *, allowed):
        """Recover startup scheduling loss only; never reset a moving frame.

        Caller must establish never-armed, zero output, fresh stationary joints.
        One second of consecutive fresh wheel/IMU observations is required.
        The initial reference and existing XY remain unchanged.
        """
        if (not allowed or self.fault not in ('imu_heading_stale','wheel_time_gap')
                or self.pose is None or self.last_wheel is None or not self.imu):
            self.startup_stable_since=None;self.startup_previous_receipt=None;self.startup_previous_stamp=None
            return None
        if not all(math.isfinite(v) for v in (stamp,x,y,wall_time)):
            return None
        _,_,wheel_yaw=attitude(quaternion)
        heading=self.heading_at(stamp)
        if heading is None or stamp<=self.last_wheel[0]:
            self.startup_stable_since=None
            return None
        if (self.distance>1e-4 or math.hypot(x-self.last_wheel[1],y-self.last_wheel[2])>1e-4
                or abs(wrap(wheel_yaw-self.last_wheel[3]))>.001
                or abs(wrap(heading-self.pose.yaw))>.01 or abs(self.imu[-1][2])>.02):
            self.startup_motion_detected=True
        if self.startup_motion_detected:return None
        if self.startup_previous_stamp is not None and stamp<=self.startup_previous_stamp:
            self.startup_stable_since=None
            return None
        self.startup_previous_stamp=stamp
        if (self.startup_previous_receipt is None or
                wall_time-self.startup_previous_receipt>.2 or wall_time<self.startup_previous_receipt):
            self.startup_stable_since=wall_time
        self.startup_previous_receipt=wall_time
        if self.startup_stable_since is None:self.startup_stable_since=wall_time
        if wall_time-self.startup_stable_since<1.:return None
        self.last_wheel=(stamp,x,y,wheel_yaw)
        self.pose=PlanarPose(stamp,self.pose.x,self.pose.y,wrap(heading),0.,self.imu[-1][2])
        self.pending_wheels.clear();self.pending_wheel=None;self.pending_since=None;self.fault=None
        self.startup_stable_since=None;self.startup_previous_receipt=None;self.startup_previous_stamp=None
        self.startup_recoveries+=1
        return self.pose

    def update_wheel(self, stamp, x, y, quaternion):
        if self.fault:
            return None
        if not all(math.isfinite(v) for v in (stamp,x,y)) or stamp < 0:
            raise ValueError('invalid_wheel_sample')
        _,_,wheel_yaw = attitude(quaternion)
        if self.last_wheel and stamp <= self.last_wheel[0]:
            return None
        heading = self.heading_at(stamp)
        if heading is None:
            if self.pose is not None:
                self._latch_fault('imu_heading_stale')
            return None
        current = (stamp,x,y,wheel_yaw)
        if self.last_wheel is None:
            self.last_wheel = current
            self.pose = PlanarPose(stamp,0.,0.,wrap(heading),0.,0.)
            return self.pose
        old_stamp,old_x,old_y,old_yaw = self.last_wheel
        dt = stamp-old_stamp
        if dt > self.max_wheel_gap:
            self._latch_fault('wheel_time_gap')
            return None
        encoder_turn = wrap(wheel_yaw-old_yaw)
        encoder_middle = old_yaw+encoder_turn/2
        chord = (x-old_x)*math.cos(encoder_middle)+(y-old_y)*math.sin(encoder_middle)
        # Recover signed arc travel, including reverse, from the encoder pose.
        sinc = math.sin(encoder_turn/2)/(encoder_turn/2) if abs(encoder_turn) > 1e-8 else 1.
        travel = chord/sinc
        turn = wrap(heading-self.pose.yaw)
        middle = self.pose.yaw+turn/2
        actual_sinc = math.sin(turn/2)/(turn/2) if abs(turn) > 1e-8 else 1.
        distance = travel*actual_sinc
        self.pose = PlanarPose(stamp,self.pose.x+distance*math.cos(middle),
                               self.pose.y+distance*math.sin(middle),wrap(heading),
                               travel/dt,turn/dt)
        self.last_wheel = current
        self.distance += abs(travel)
        return self.pose

    def status(self):
        return {'source':'wheel_distance_imu_heading', 'simulated':True,
                'position':'dead_reckoning_no_slip_correction',
                'heading':'imu_orientation_relative_to_initial_with_bounded_gyro_prediction',
                'uses_ground_truth':False, 'uses_gps':False,
                'state':'fault' if self.fault else ('ready' if self.pose else 'waiting_for_imu_and_wheels'),
                'fault':self.fault, 'distance_m':self.distance,
                'last_fault':self.fault_context,'startup_recoveries':self.startup_recoveries,
                'startup_unpaired_discarded':self.startup_unpaired_discarded,
                'startup_motion_detected':self.startup_motion_detected,
                'pending_wheel_stamp':self.pending_wheel[0] if self.pending_wheel else None,
                'stamp':self.pose.stamp if self.pose else None}
