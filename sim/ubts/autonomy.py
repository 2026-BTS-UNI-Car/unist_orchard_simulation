"""Bounded, simulation-only 2D orchard traversal.

The unchanged upstream Dubins planner and Pure Pursuit tracker provide the row
and headland path. Lane/end coordinates are an explicit *mission prior*, not
detected trees. Only wheel odometry, LaserScan and the live 2D occupancy map are
used at runtime. There is no ROS publisher, serial path, ground-truth obstacle
input, or automatic arming here; the adapter owns the existing owner/deadman
lease. A stop is latched until the user explicitly starts a new mission.

This is a planar simulator controller, not 3D SLAM or a real-car autonomy stack.
Objects above/below the 60 cm scan plane remain unobservable.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys
from typing import Callable, Mapping

import numpy as np
from occupancy_lookup import OccupancyLookup
from lidar_broadphase import prepare, all_outside

UPSTREAM = Path(__file__).resolve().parents[2] / 'ros2_ws/src/orchard_navigation'
if str(UPSTREAM) not in sys.path:
    sys.path.insert(0, str(UPSTREAM))
from orchard_navigation.headland import PathTracker, dubins_path


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class OrchardAutonomy:
    """``step(scan, odom, occupancy, now, authorized=True)`` returns a command.

    scan: dict of ranges, angle_min, angle_increment, range_min/max, wall_stamp.
    odom: dict of x, y, yaw, wall_stamp, in the same frame as the live grid.
    occupancy: callable ``(x_m, y_m) -> -1 unknown / 0..100 occupied``.
    now and wall_stamp are monotonic wall seconds, never simulation time.

    Default odometry origin is the generated world's spawn pose. Set metadata
    ``odom_origin_world=[x,y,yaw]`` explicitly if a different origin is used.
    ``start`` does not grant authorization. The caller must separately retain
    the armed owner lease, browser keepalive and explicit autonomy selection.
    """

    def __init__(self, config: Mapping, metadata: Mapping):
        self.config, self.metadata = config, metadata
        vehicle, lidar = config.get('vehicle', {}), config.get('lidar', {})
        options = config.get('autonomy', {})
        self.max_wheel = float(vehicle.get('max_wheel_speed_m_s', .5))
        self.track = float(vehicle.get('track_m', .56))
        self.speed = min(float(options.get('speed_m_s', .4)), self.max_wheel * .75)
        self.turn_speed = min(float(options.get('turn_speed_m_s', .3)), self.speed)
        self.cruise_limit = self.speed
        self.turn_limit = self.turn_speed
        self.speed_percent = 100.
        self.acceleration = max(.05, float(vehicle.get('acceleration_m_s2', .4)))
        self.half_length = float(vehicle.get('original_length_m', 1.22)) / 2 + .10
        self.half_width = float(vehicle.get('original_width_m', .70)) / 2 + .10
        self.lidar_xy = (float(lidar.get('x_m', .18)), float(lidar.get('y_m', 0.)))
        self.stale_s = float(options.get('sensor_stale_s', 1.5))
        self.exit_margin = float(options.get('exit_margin_m', 1.2))
        self.lookahead = float(options.get('lookahead_m', .9))
        self.max_deviation = float(options.get('max_deviation_m', .65))
        self.grid_step = .15  # occupancy collision footprint sampling, metres
        self.state, self.reason = 'IDLE', 'not_started'
        self.tracker = None
        self.path = np.empty((0, 3))
        self._snapshot_path_key = None
        self._snapshot_path = ()
        self.phases = []
        self.waypoints = []
        self.lane_indices = []
        self.last_now = None
        self._last_result = None
        self._scan_context = {}
        self._planned_command = None
        self.obstacle_diagnostic = None
        self._footprint = np.array([(x, y) for x in np.linspace(-self.half_length, self.half_length, math.ceil(2*self.half_length/self.grid_step)+1)
                                    for y in np.linspace(-self.half_width, self.half_width, math.ceil(2*self.half_width/self.grid_step)+1)])

    @staticmethod
    def _pose(odom):
        return tuple(float(odom[key]) for key in ('x', 'y', 'yaw'))

    def _origin(self):
        if 'odom_origin_world' in self.metadata:
            return tuple(float(v) for v in self.metadata['odom_origin_world'])
        spawn = self.metadata.get('spawn', [0., 0., 0.])
        yaw = float(self.metadata.get('config', {}).get('spawn_yaw', 0.))
        return float(spawn[0]), float(spawn[1]), yaw

    def _world_pose(self, pose):
        ox, oy, heading = self._origin()
        x, y, yaw = pose
        c, s = math.cos(heading), math.sin(heading)
        return ox+c*x-s*y, oy+s*x+c*y, wrap(yaw+heading)

    def _odom_path(self, path):
        ox, oy, heading = self._origin()
        c, s = math.cos(heading), math.sin(heading)
        arr = np.asarray(path, dtype=float).copy()
        dx, dy = arr[:, 0]-ox, arr[:, 1]-oy
        arr[:, 0], arr[:, 1] = c*dx+s*dy, -s*dx+c*dy
        arr[:, 2] = np.arctan2(np.sin(arr[:, 2]-heading), np.cos(arr[:, 2]-heading))
        return arr

    def start(self, odom: Mapping, now: float) -> dict:
        """Plan from a near-centred/aligned lane pose, retaining no old command."""
        self.obstacle_diagnostic = None
        self._planned_command = None
        self.stop('planning')
        try:
            pose = self._pose(odom)
            if not all(math.isfinite(v) for v in (*pose, now)):
                raise ValueError('invalid_odometry')
            if not self._fresh(odom, now):
                raise ValueError('stale_odometry')
            lanes = sorted(float(y) for y in self.metadata.get('lane_centers_y', []))
            limits = self.metadata.get('row_x_range', [])
            if len(lanes) < 1 or len(limits) != 2:
                raise ValueError('orchard_mission_prior_missing')
            x, y, yaw = self._world_pose(pose)
            lane = min(range(len(lanes)), key=lambda k: abs(lanes[k]-y))
            direction = 1 if math.cos(yaw) >= 0 else -1
            heading = 0. if direction > 0 else math.pi
            if abs(y-lanes[lane]) > .65 or abs(wrap(yaw-heading)) > math.radians(35):
                raise ValueError('align_with_lane_before_start')
            lo, hi = sorted(float(v) for v in limits)
            if x < lo-5. or x > hi+5.:
                raise ValueError('outside_orchard_start_area')
            lane_limit = int(self.config.get('autonomy', {}).get('max_lanes', len(lanes)))
            self.lane_indices = list(range(lane, min(len(lanes), lane+max(1, lane_limit))))
            parts, phases, waypoints = [], [], []
            current = (x, y, yaw)
            for index, k in enumerate(self.lane_indices):
                end = hi+self.exit_margin if direction > 0 else lo-self.exit_margin
                if direction*(end-current[0]) <= .25:
                    raise ValueError('start_before_row_exit')
                target = (end, lanes[k], heading)
                # A straight row segment softly centres the initial <=65 cm offset.
                n = max(2, math.ceil(math.hypot(end-current[0], lanes[k]-current[1])/.05)+1)
                segment = np.column_stack((np.linspace(current[0], end, n),
                                           np.linspace(current[1], lanes[k], n),
                                           np.full(n, heading)))
                parts.append(segment if not parts else segment[1:])
                phases.extend([('FOLLOW_ROW', k)] * len(parts[-1]))
                waypoints.append({'x': end, 'y': lanes[k], 'yaw': heading, 'kind': 'row_exit', 'lane': k})
                if index+1 < len(self.lane_indices):
                    nk = self.lane_indices[index+1]
                    next_heading = math.pi if direction > 0 else 0.
                    goal = (end, lanes[nk], next_heading)
                    turn = dubins_path(target, goal, abs(lanes[nk]-lanes[k])/2, step=.05)
                    parts.append(turn[1:]); phases.extend([('TURN', k)] * (len(turn)-1))
                    waypoints.append({'x': end, 'y': lanes[nk], 'yaw': next_heading, 'kind': 'row_entry', 'lane': nk})
                    current, direction, heading = goal, -direction, next_heading
            self.path = self._odom_path(np.vstack(parts))
            self.phases = phases
            self.waypoints = []
            for point in waypoints:
                px, py, pyaw = self._odom_path([(point['x'], point['y'], point['yaw'])])[0]
                self.waypoints.append({**point, 'x': float(px), 'y': float(py), 'yaw': float(pyaw)})
            self.tracker = PathTracker(self.path, lookahead=self.lookahead, max_curvature=1/.9)
            self.state, self.reason = 'FOLLOW_ROW', 'mission_prior_scan_and_grid_guarded'
            self.last_now = now
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.state, self.reason = 'STOPPED', str(exc)
        return self._result(0., 0.)

    def stop(self, reason='user_stop') -> dict:
        self.state, self.reason = 'STOPPED', reason
        return self._result(0., 0.)

    def _fresh(self, sample, now):
        stamp = float(sample.get('wall_stamp', -math.inf))
        return math.isfinite(stamp) and 0 <= now-stamp < self.stale_s

    def _scan_points(self, scan):
        ranges = np.asarray(scan['ranges'], dtype=float)
        angle_min, inc = float(scan['angle_min']), float(scan['angle_increment'])
        near, far = float(scan.get('range_min', .05)), float(scan.get('range_max', 30.))
        if ranges.ndim != 1 or len(ranges) < 180 or not .01 < abs(inc)*len(ranges) <= 2*math.pi+.1:
            raise ValueError('invalid_scan_geometry')
        if abs(inc)*(len(ranges)-1) < math.radians(350):
            raise ValueError('full_scan_required')
        if not all(math.isfinite(v) for v in (angle_min, inc, near, far)) or far <= near:
            raise ValueError('invalid_scan_geometry')
        angles = angle_min + np.arange(len(ranges))*inc
        # Positive infinity is the LaserScan convention for no return. Other
        # out-of-range values must not silently count as observed free space.
        valid = np.isposinf(ranges) | (np.isfinite(ranges) & (ranges >= near) & (ranges <= far))
        front = np.cos(angles) > .5
        if valid.mean() < .8 or not front.any() or valid[front].mean() < .9:
            raise ValueError('scan_coverage_insufficient')
        hits = valid & np.isfinite(ranges) & (ranges < far*.995)
        self._scan_context = {'sim_stamp': scan.get('sim_stamp'), 'wall_stamp': scan.get('wall_stamp'),
                              'angle_min': angle_min, 'angle_increment': inc,
                              'hit_beam_indices': np.flatnonzero(hits)}
        return np.column_stack((ranges[hits]*np.cos(angles[hits])+self.lidar_xy[0],
                                ranges[hits]*np.sin(angles[hits])+self.lidar_xy[1]))

    def _blocked(self, points, pose, occupancy: Callable, linear, angular, stage='planner'):
        """Test stopping horizon swept vehicle against raw returns and map cells."""
        self.obstacle_diagnostic = None
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        world_hits = points @ np.array([[c, s], [-s, c]]) + (x, y)
        broadphase_bound=prepare(world_hits,self.half_length,self.half_width)
        # Include response/deceleration and a small buffer in the checked horizon.
        distance = max(1.2, linear*.8 + linear*linear/(2*self.acceleration) + self.half_length)
        curvature = angular/linear if linear > 1e-5 else 0.
        travels=np.arange(0., distance+.075, .15)
        frames=None;batch_hit=None
        # Only the known immutable occupied-set lookup is safe to evaluate
        # early. Generic callbacks may have side effects or custom thresholds.
        if type(occupancy) is OccupancyLookup and occupancy.grid is not None:
            try:
                with np.errstate(all='raise'):
                    frames=[self._sweep_frame(travel,x,y,yaw,c,s,curvature) for travel in travels]
                    cells=np.concatenate([self._footprint @ np.array([[pc,ps],[-ps,pc]])+(px,py)
                                          for px,py,pyaw,pc,ps in frames])
                    batch_hit=occupancy.first_occupied(cells)
            except (ValueError,TypeError,OverflowError,FloatingPointError,MemoryError):
                # A later invalid sample must not preempt an earlier LiDAR or
                # map hit. Re-evaluate in the original short-circuit order.
                frames=None
        for horizon_index,travel in enumerate(travels):
            px,py,pyaw,pc,ps=(frames[horizon_index] if frames is not None else
                              self._sweep_frame(travel,x,y,yaw,c,s,curvature))
            if not all_outside(world_hits,px,py,pc,ps,broadphase_bound):
                local = (world_hits-(px, py)) @ np.array([[pc, -ps], [ps, pc]])
                inside = (np.abs(local[:, 0]) <= self.half_length) & (np.abs(local[:, 1]) <= self.half_width)
                if len(local) and np.any(inside):
                    index = int(np.flatnonzero(inside)[0])
                    self.obstacle_diagnostic = self._guard_diagnostic(stage, linear, angular, distance,
                        float(travel), pose, (px, py, pyaw))
                    point = points[index]
                    laser = point-np.asarray(self.lidar_xy)
                    beam_indices = self._scan_context.get('hit_beam_indices', [])
                    beam = int(beam_indices[index]) if index < len(beam_indices) else None
                    self.obstacle_diagnostic.update(reason='lidar_obstacle', blocking_point={
                        'beam_index': beam, 'range_m': float(np.linalg.norm(laser)),
                        'angle_rad': (self._scan_context['angle_min']+beam*self._scan_context['angle_increment'])
                                     if beam is not None else math.atan2(float(laser[1]), float(laser[0])),
                        'body_xy': point.tolist(), 'odom_xy': world_hits[index].tolist(),
                        'swept_body_xy': local[index].tolist(),
                        'inside_margin_m': float(min(self.half_length-abs(local[index, 0]),
                                                     self.half_width-abs(local[index, 1])))})
                    return 'lidar_obstacle'
            hit=None
            if frames is not None:
                if batch_hit is not None and batch_hit[0]//len(self._footprint)==horizon_index:
                    index,value=batch_hit
                    hit=(cells[index],value)
            else:
                # Generic occupancy callbacks retain scalar evaluation order.
                footprint=self._footprint @ np.array([[pc, ps], [-ps, pc]]) + (px, py)
                for gx,gy in footprint:
                    value=occupancy(float(gx),float(gy))
                    if value is not None and float(value)>=60:
                        hit=((gx,gy),value);break
            if hit is not None:
                (gx,gy),value=hit
                self.obstacle_diagnostic = self._guard_diagnostic(stage, linear, angular, distance,
                    float(travel), pose, (px, py, pyaw))
                self.obstacle_diagnostic.update(reason='map_obstacle', blocking_map_sample={
                    'odom_xy': [float(gx), float(gy)], 'occupancy': float(value)})
                return 'map_obstacle'
        return None

    @staticmethod
    def _sweep_frame(travel,x,y,yaw,c,s,curvature):
        if abs(curvature)<1e-5:
            px,py,pyaw=x+travel*c,y+travel*s,yaw
        else:
            pyaw=yaw+travel*curvature
            px=x+(math.sin(pyaw)-s)/curvature
            py=y+(-math.cos(pyaw)+c)/curvature
        return px,py,pyaw,math.cos(pyaw),math.sin(pyaw)

    def _guard_diagnostic(self, stage, linear, angular, horizon, travel, pose, swept_pose):
        """Freeze only the selected rejection evidence, never a full scan."""
        return {'stage': stage, 'scan_sim_stamp': self._scan_context.get('sim_stamp'),
                'scan_wall_stamp': self._scan_context.get('wall_stamp'),
                'planned_command': self._planned_command,
                'checked_command': {'linear_m_s': float(linear), 'angular_rad_s': float(angular)},
                'lidar_body_xy_m': list(self.lidar_xy), 'odom_pose': list(pose),
                'footprint_half_extents_m': [self.half_length, self.half_width],
                'footprint_padding_m': .10, 'horizon_m': float(horizon),
                'first_blocked_travel_m': travel, 'swept_odom_pose': list(swept_pose)}

    def command_guard(self, scan, odom, occupancy, linear, angular):
        """Recheck the corrected actuator command without weakening planner guards."""
        return self._blocked(self._scan_points(scan),self._pose(odom),occupancy,linear,angular,
                             stage='yaw_corrected')

    def step(self, scan: Mapping, odom: Mapping, occupancy: Callable, now: float,
             authorized: bool = True) -> dict:
        if self.state in ('IDLE', 'STOPPED', 'DONE') or self.tracker is None:
            return self._result(0., 0.)
        if not authorized:
            return self.stop('control_authorization_lost')
        if not math.isfinite(now) or (self.last_now is not None and now < self.last_now):
            return self.stop('clock_reset')
        self.last_now = now
        try:
            if not self._fresh(scan, now) or not self._fresh(odom, now):
                return self.stop('stale_scan_or_odometry')
            if not callable(occupancy):
                return self.stop('occupancy_map_unavailable')
            pose = self._pose(odom)
            if not all(math.isfinite(v) for v in pose):
                return self.stop('invalid_odometry')
            points = self._scan_points(scan)
            state, lane = self.phases[self.tracker.i]
            speed = self.turn_speed if state == 'TURN' else self.speed
            linear, angular = self.tracker.command(*pose, speed)
            self._planned_command = {'linear_m_s': float(linear), 'angular_rad_s': float(angular)}
            if self.tracker.cross_track > self.max_deviation:
                return self.stop('path_deviation')
            if self.tracker.remaining <= .20 and math.hypot(pose[0]-self.path[-1, 0], pose[1]-self.path[-1, 1]) < .35:
                self.state, self.reason = 'DONE', 'planned_lanes_completed'
                return self._result(0., 0.)
            blocked = self._blocked(points, pose, occupancy, linear, angular)
            if blocked:
                return self.stop(blocked)
            self.state, self.reason = self.phases[self.tracker.i][0], 'tracking'
            return self._result(linear, angular)
        except (KeyError, TypeError, ValueError, OverflowError):
            return self.stop('invalid_sensor_or_map_input')

    def _result(self, linear, angular):
        left, right = linear-angular*self.track/2, linear+angular*self.track/2
        scale = max(1., abs(left)/self.max_wheel, abs(right)/self.max_wheel)
        left, right = left/scale, right/scale
        result = {'linear': (left+right)/2, 'angular': (right-left)/self.track,
                  'left_pct': left/self.max_wheel*100, 'right_pct': right/self.max_wheel*100,
                  'active': self.state not in ('IDLE', 'STOPPED', 'DONE'),
                  'state': self.state, 'reason': self.reason,
                  'lane': self.phases[self.tracker.i][1] if self.tracker is not None else None,
                  'remaining_m': float(self.tracker.remaining) if self.tracker is not None else 0.,
                  'path_index': int(self.tracker.i) if self.tracker is not None else 0,
                  'planner': 'upstream_dubins_pure_pursuit_2d_guarded',
                  'mission_source': 'known_orchard_layout', 'simulated': True}
        self._last_result = result
        return result

    def path_snapshot(self, *, borrow=False):
        """Sampled rounded XY; normal callers receive detached mutable lists.

        Internal status/visualization callers may retain the immutable borrowed
        tuple across writer threads. Compare CONTENT on every capture: tests
        and replanning may replace or mutate the control ndarray in place.
        This cache never changes that ndarray or the tracker's reference.
        """
        rounded = self.path[::5, :2].round(3)
        if not borrow:
            return rounded.tolist()
        key = (rounded.shape, rounded.dtype.str, rounded.tobytes())
        if key != self._snapshot_path_key:
            self._snapshot_path = tuple(tuple(point) for point in rounded.tolist())
            self._snapshot_path_key = key
        return self._snapshot_path

    def snapshot(self, include_path=False):
        out = dict(self._last_result or self._result(0., 0.))
        out.update(speed_percent=self.speed_percent, cruise_speed_m_s=self.speed,
                   turn_speed_m_s=self.turn_speed, cruise_limit_m_s=self.cruise_limit)
        out['obstacle_diagnostic'] = self.obstacle_diagnostic
        if include_path:
            out['waypoints'] = self.waypoints
            out['path'] = self.path_snapshot()
        return out

    def set_speed_percent(self, percent):
        self.speed_percent = percent
        self.speed = self.cruise_limit * percent / 100.
        self.turn_speed = self.turn_limit * percent / 100.
        # A deferred planner must not reuse the preceding, faster command.
        # Snapshot fallback yields zero until a fresh planning tick replaces it.
        self._last_result = None
        self._planned_command = None
