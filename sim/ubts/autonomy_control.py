"""Simulation-only autonomous lease and wall-clock watchdog."""
import json
import math
from pathlib import Path
import secrets
import time

from autonomy import OrchardAutonomy
from yaw_rate_tracker import YawRateTracker
from occupancy_lookup import OccupancyLookup
from scripts.rover_control_core import ControlError
from scripts.sim_clock import freshness as clock_freshness


class AutonomousControl:
    HEARTBEAT_S = 1.0

    def __init__(self, core, config, metadata, clock=time.monotonic,
                 map_path='/run/ubts-sim/mapping/planning_map.json', sim_clock=None):
        self.core, self.clock = core, clock
        self.sim_clock = sim_clock or clock
        self.mission = OrchardAutonomy(config, metadata)
        vehicle=config['vehicle']
        self.yaw_tracker=YawRateTracker(track=float(vehicle['track_m']),
                                       max_wheel=float(vehicle['max_wheel_speed_m_s']),
                                       **config.get('autonomy',{}).get('yaw_rate_feedback',{}))
        self.yaw_status={}
        self.map_path = Path(map_path)
        self.owner = self.token = self.grant = None
        self.deadline = 0.; self.next_step = 0.; self.next_map_read = 0.
        self.next_feedback_check = 0.
        self.next_wait_zero = 0.
        self.map = {}; self.occupied = set()
        self._map_text = None
        self.occupancy = OccupancyLookup(.1,self.occupied)
        self.map_freshness = {'clock': 'CLOCK_MONOTONIC_RAW', 'fresh': False, 'reason': 'not_read', 'read_error': None}
        self.scan = {}; self.odom = {}; self.yaw_measurement = {}
        self.sensor_wait_reason = None; self.sensor_wait_since = None
        self.feedback_check = None
        self.diagnostic_observer = None
        self.planning_deferrals = 0
        self.last_planning_deferred = False

    def _observe(self, name, function, *args, **kwargs):
        # Optional main-thread diagnostic, installed by the SIM adapter only.
        # The observer must preserve the callable's result and exceptions.
        if self.diagnostic_observer is None:
            return function(*args, **kwargs)
        return self.diagnostic_observer(name, function, *args, **kwargs)

    @property
    def active(self):
        return self.token is not None

    def _map_ready(self):
        now = self.clock()
        if now >= self.next_map_read:
            self.next_map_read = now+.2
            try:
                text = self.map_path.read_text()
                # The mapper writes at2Hz; retain one successfully validated
                # payload between our5Hz reads. Compare complete text, never
                # version/mtime, and still check both RAW ages on every call.
                # This private payload is not returned by status() or mutated
                # by any reader; status contains separate freshness metadata.
                if text != self._map_text:
                    value = json.loads(text)
                    if not isinstance(value, dict): raise ValueError('map_object_required')
                    resolution = float(value.get('resolution', 0))
                    if not .01 <= resolution <= 1.:
                        raise ValueError('invalid_map_metadata')
                    occupied = {tuple(p) for p in value.get('occupied', [])}
                    if any(len(p) != 2 or not all(isinstance(v, int) for v in p) for p in occupied):
                        raise ValueError('invalid_map_cells')
                    value['resolution'] = resolution
                    lookup=self.occupancy
                    if lookup.resolution!=resolution or lookup.occupied!=occupied:
                        lookup=OccupancyLookup(resolution,occupied)
                    self.map, self.occupied, self.occupancy = value, occupied, lookup
                    self._map_text = text
                self.map_freshness['read_error'] = None
            except (OSError, ValueError, TypeError) as error:
                self.map = {}; self.occupied = set()
                self._map_text = None
                self.occupancy = OccupancyLookup(.1,self.occupied)
                self.map_freshness['read_error'] = type(error).__name__
        # File timestamps use RAW; self.clock remains the existing MONOTONIC
        # scheduler / lease clock and must never be subtracted from RAW data.
        self.map_freshness = {**clock_freshness(self.map, 2.5),
                              'active': self.map.get('active') is True,
                              'read_error': self.map_freshness.get('read_error'),
                              'version': self.map.get('version') if isinstance(self.map.get('version'), int) else None}
        source = clock_freshness(self.map.get('source_scan_clock'), 2.5)
        self.map_freshness['source_scan'] = source
        if self.map_freshness['fresh'] and not source['fresh']:
            self.map_freshness.update(fresh=False, reason='source_scan_'+source['reason'])
        return self.map_freshness['active'] and self.map_freshness['fresh']

    def stop(self, reason):
        # Cleanup and repeated operator STOP must not replace the first stopped
        # mission's evidence, or touch a later manual owner's lease.
        if not self.active and self.mission.state == 'STOPPED':
            return self.status()
        self.mission.stop(reason)
        self.yaw_tracker.stop(reason)
        self.sensor_wait_since = None
        self.yaw_status.update(state='stopped',reason=reason,applied_angular=0.)
        if self.active:
            self.core.stop(reason)
        self.owner = self.token = self.grant = None
        return self.status()

    def status(self, include_path=False):
        result = self.mission.snapshot(include_path)
        result.update(ok=True, active=self.active, message=result.get('reason', ''),
                      heartbeat_s=self.HEARTBEAT_S,yaw_tracking=dict(self.yaw_status),
                      map_freshness=dict(self.map_freshness),
                      planning_dispatch={'deferred_due_ticks':self.planning_deferrals,
                                         'last_due_tick_deferred':self.last_planning_deferred,
                                         'resume_requires_empty_burst':True})
        return result

    def _track_yaw(self, result, odom, now=None):
        sample=self.yaw_measurement
        measured=float(sample['yaw_rate'])
        stamp=float(sample['sim_stamp'])
        wall_stamp=float(sample['wall_stamp'])
        sim_time=float(self.sim_clock())
        wall_age=(self.clock() if now is None else now)-wall_stamp
        if not all(math.isfinite(v) for v in (measured,stamp,wall_stamp,sim_time,wall_age)):
            raise ValueError('invalid_yaw_measurement')
        if wall_age < 0 or wall_age > .2:
            command=self.yaw_tracker.reject_measurement('stale_imu_arrival' if wall_age >= 0 else 'future_imu_arrival')
        else:
            command=self.yaw_tracker.step(result['linear'],result['angular'],measured,
                                          sim_time,stamp,authorized=True)
        self.yaw_status={'source':'imu_gyro_world_yaw_derivative','state':'tracking' if command.valid else 'fault',
                         'reason':command.reason,'measurement_sim_stamp':stamp,
                         'control_sim_stamp':sim_time,'measurement_sim_age_s':sim_time-stamp,
                         'measurement_wall_age_s':wall_age,
                         'pose_sim_age_s':sim_time-float(odom['sim_stamp']),
                         'pose_wall_age_s':self.clock()-float(odom['wall_stamp']),
                         'desired_angular':float(result['angular']),'measured_angular':measured,
                         'corrected_angular':command.angular,'applied_angular':command.angular,
                         'integral':self.yaw_tracker.integral,'saturated':command.saturated,
                         'left_m_s':command.left_m_s,'right_m_s':command.right_m_s}
        return command

    def _wait_for_feedback(self, odom, now):
        """Zero output during a bounded inter-topic synchronization gap."""
        # Re-evaluate the adapter's clock/paired-pose age after expensive work,
        # including at the final command boundary, not just before planning.
        if self.feedback_check is not None:self.feedback_check(now)
        sample=self.yaw_measurement
        stamp=float(sample['sim_stamp']);wall_stamp=float(sample['wall_stamp'])
        rate=float(sample['yaw_rate']);sim_time=float(self.sim_clock())
        if not all(math.isfinite(v) for v in (stamp,wall_stamp,rate,sim_time)):
            raise ValueError('invalid_yaw_measurement')
        age=sim_time-stamp;wall_age=now-wall_stamp
        # Independent topic callbacks can expose a newer IMU before /clock.
        # Never actuate beyond the tracker's existing future tolerance: hold
        # zero while clock catches up under the same fixed acquisition limit.
        waiting=self.sensor_wait_reason
        # Recovery must not discard the original deadline for a malformed or
        # stale pose. These are the existing paired-pose and mission freshness
        # limits; the adapter still checks integration/clock receipt readiness.
        if self.sensor_wait_since is not None:
            pose_stamp=float(odom['sim_stamp']);pose_wall=float(odom['wall_stamp'])
            if not all(math.isfinite(v) for v in (*self.mission._pose(odom),pose_stamp,pose_wall)):
                raise ValueError('invalid_odometry')
            pose_age=sim_time-pose_stamp
            if (not self.mission._fresh(odom,now) or
                    pose_age < -self.yaw_tracker.future_tolerance_s-1e-9 or
                    pose_age > self.yaw_tracker.stale_s):
                waiting=waiting or 'waiting_for_wheel_imu_pair'
        if not waiting:
            if age < -self.yaw_tracker.future_tolerance_s-1e-9:
                waiting='waiting_for_clock'
            elif age>self.yaw_tracker.stale_s or wall_age<0 or wall_age>.2:
                waiting='waiting_for_imu'
        if not waiting:
            # feedback_check or scheduling can cross a deadline after the
            # caller's cached `now`. Re-read the actual authorization/wait
            # clock before discarding the only record of an existing wait.
            if self.sensor_wait_since is not None and not self.check_watchdogs():
                return True
            self.sensor_wait_since=None
            return False
        first_wait=self.sensor_wait_since is None
        if first_wait:self.sensor_wait_since=now
        self.yaw_status={'source':'imu_gyro_world_yaw_derivative','state':'waiting','reason':waiting,
                         'measurement_sim_stamp':stamp,'control_sim_stamp':sim_time,
                         'measurement_sim_age_s':age,'measurement_wall_age_s':wall_age,
                         'pose_sim_age_s':sim_time-float(odom['sim_stamp']),
                         'pose_wall_age_s':now-float(odom['wall_stamp']),
                         'applied_angular':0.,'left_m_s':0.,'right_m_s':0.}
        # The stale sample triggers zero output immediately; the acquisition
        # window starts at that detection, not at the already-old sample.
        # Fresh clock/wheel callbacks cannot extend this fixed wall deadline.
        if wall_age<0 or now-self.sensor_wait_since>.2:
            self.stop('imu_synchronization_timeout')
            return True
        # Readiness is cheap and checked every recovery loop. Only repeated
        # ZERO transmissions are limited to50ms; each new wait zeros at once.
        if first_wait or now>=self.next_wait_zero:
            self.next_wait_zero=now+.05
            self.yaw_tracker.stop(waiting)
            if self._drive_wheels(0.,0.):self.core.motor.hard_stop()
        return True

    def _drive_wheels(self, left, right):
        try:
            self.grant = self.core.dispatch({
                'op': 'drive', 'owner': self.owner, 'lease': self.grant['lease'],
                'ticket': self.grant['ticket'], 'seq': self.grant['seq']+1,
                'deadman': True,
                'left': max(-100, min(100, round(left/self.yaw_tracker.max_wheel*100))),
                'right': max(-100, min(100, round(right/self.yaw_tracker.max_wheel*100)))})
            return True
        except ControlError:
            self.stop('autonomy_control_lost')
            return False

    def dispatch(self, payload):
        op, owner = payload.get('op'), payload.get('owner')
        if op == 'autonomy_status': return self.status(include_path=True)
        if op == 'autonomy_stop': return self.stop(payload.get('reason', 'autonomy_operator_stop'))
        if op == 'autonomy_speed':
            percent = payload.get('speed_percent')
            if isinstance(percent, bool) or not isinstance(percent, (int, float)) or not math.isfinite(percent) or not 0 <= percent <= 100:
                raise ControlError('invalid_autonomy_speed')
            if not owner: raise ControlError('operator_required')
            if self.active:
                if owner != self.owner or payload.get('token') != self.token:
                    raise ControlError('autonomy_not_owned')
                if not self.check_watchdogs():
                    raise ControlError('autonomy_control_expired')
            self.mission.set_speed_percent(float(percent))
            # A speed change never grants control or extends the watchdog.
            # Discard a cached trajectory command before the next planning tick.
            self.next_step = 0.
            if percent == 0 and self.active:
                return self.stop('autonomy_speed_zero')
            self.yaw_tracker.reset()
            if self.active:
                # A busy adapter may defer the next planner tick. Remove its
                # preceding nonzero command now, under the same owned lease;
                # only a fresh guarded plan may resume at the new speed.
                self._drive_wheels(0.,0.)
            return self.status()
        if op == 'autonomy_start':
            if self.mission.speed_percent <= 0:
                raise ControlError('autonomy_speed_zero')
            if self.active or self.core.lease:
                raise ControlError('stop_existing_control_before_autonomy')
            if not owner: raise ControlError('operator_required')
            if self.sensor_wait_reason:raise ControlError(self.sensor_wait_reason)
            if not self._map_ready(): raise ControlError('start_live_mapping_first')
            now = self.clock()
            # Plan before arming; malformed prior or alignment never moves a wheel.
            result = self.mission.start(self.odom, now)
            if not result['active']:
                raise ControlError(result['reason'])
            # Validate current scan and swept footprint before granting a lease.
            # Planning may overlap fresh sensor callbacks. Snapshot the inputs
            # before reading the check time so new samples are not "future".
            scan, odom = self.scan, self.odom
            now = self.clock()
            result = self.mission.step(scan, odom, self.occupancy, now)
            if not result['active']:
                raise ControlError(result['reason'])
            self.yaw_tracker.reset()
            try:command=self._track_yaw(result,odom)
            except (KeyError,TypeError,ValueError):
                self.mission.stop('invalid_yaw_measurement')
                self.yaw_status={'state':'fault','reason':'invalid_yaw_measurement'}
                raise ControlError('invalid_yaw_measurement')
            if not command.valid:
                self.mission.stop(command.reason)
                raise ControlError(command.reason)
            blocked=self.mission.command_guard(scan,odom,self.occupancy,command.linear,command.angular)
            if blocked:
                self.mission.stop(blocked)
                raise ControlError(blocked)
            try:
                self.grant = self.core.dispatch({'op': 'arm', 'owner': owner})
            except Exception:
                self.mission.stop('arming_failed'); raise
            self.owner, self.token = owner, secrets.token_urlsafe(24)
            self.deadline = self.clock()+self.HEARTBEAT_S
            return {**self.status(include_path=True), 'token': self.token}
        if op == 'autonomy_heartbeat':
            if not self.active or owner != self.owner or payload.get('token') != self.token:
                raise ControlError('autonomy_not_owned')
            if self.clock() >= self.deadline:
                self.stop('autonomy_heartbeat_expired')
                raise ControlError('autonomy_heartbeat_expired')
            self.deadline = self.clock()+self.HEARTBEAT_S
            return self.status()
        raise ControlError('unknown_autonomy_operation')

    def check_watchdogs(self):
        """Cheap authorization/deadline check; no planning, PI or map reads."""
        if not self.active:return False
        now = self.clock()
        if now >= self.deadline:
            self.stop('autonomy_heartbeat_expired'); return False
        if self.core.lease != self.grant['lease'] or not self.core.motor.ready:
            self.stop('autonomy_control_lost'); return False
        # A queued fresh callback may not revive an already expired wait.
        if self.sensor_wait_since is not None and now-self.sensor_wait_since>.2:
            self.stop('imu_synchronization_timeout');return False
        return True

    def step(self, *, allow_planning=True):
        if not self.check_watchdogs():return
        now = self.clock()
        # A valid pair can appear briefly between queued topic callbacks.
        # Confirm recovery before either cadence gate, including waits entered
        # inside a guard whose planning deadline is still in the future.
        if self.sensor_wait_since is not None:
            try:
                if self._wait_for_feedback(self.odom,now):return
            except (KeyError,TypeError,ValueError):
                self.stop('invalid_yaw_measurement');return
        if now < self.next_step: return
        # A busy dispatch cannot spend the planner's due slot: the next empty
        # burst must be able to plan immediately. Rate-limit only the cheap
        # feedback path while deferred. Existing waits were checked above.
        if not allow_planning:
            if now < self.next_feedback_check:return
            self.next_feedback_check=now+.05
        self.last_planning_deferred=not allow_planning
        if not allow_planning:self.planning_deferrals+=1
        # Stale feedback must zero motion before map parsing or either swept
        # collision guard can spend CPU. The owner watchdog above stays first.
        try:
            if self._wait_for_feedback(self.odom,now):return
        except (KeyError,TypeError,ValueError):
            self.stop('invalid_yaw_measurement');return
        # An exhausted dispatch burst may still have measured wheel/IMU
        # history ready. Keep the original50ms feedback/zero checks, but do
        # not insert map parsing or collision work ahead of that history.
        # No command/keepalive is sent on a fresh deferred tick: the existing
        # input TTL also bounds how long an old nonzero command can survive.
        if not allow_planning:return
        self.next_step=now+.05
        self.next_feedback_check=self.next_step
        if not self._observe('map_ready',self._map_ready):
            self.stop('mapping_stopped_or_stale'); return
        result = self._observe('mission_step',self.mission.step,self.scan,self.odom,self.occupancy,now)
        if not result['active']:
            self.stop(result['reason']); return
        try:
            feedback_now=self.clock()
            if self._wait_for_feedback(self.odom,feedback_now):return
            command=self._observe('yaw_track',self._track_yaw,result,self.odom,now=feedback_now)
        except (KeyError,TypeError,ValueError):
            self.stop('invalid_yaw_measurement');return
        if not command.valid:
            self.stop(command.reason);return
        blocked=self._observe('corrected_guard',self.mission.command_guard,self.scan,self.odom,self.occupancy,command.linear,command.angular)
        if blocked:
            self.stop(blocked);return
        # The corrected-command guard can itself be delayed by host load.
        # Recheck authorization and freshness at the actual actuation boundary
        # without applying the PI integral a second time.
        actuation_now=self.clock()
        if actuation_now>=self.deadline:
            self.stop('autonomy_heartbeat_expired');return
        if self.core.lease!=self.grant['lease'] or not self.core.motor.ready:
            self.stop('autonomy_control_lost');return
        try:
            if self._wait_for_feedback(self.odom,actuation_now):return
        except (KeyError,TypeError,ValueError):
            self.stop('invalid_yaw_measurement');return
        self._observe('drive_wheels',self._drive_wheels,command.left_m_s,command.right_m_s)
