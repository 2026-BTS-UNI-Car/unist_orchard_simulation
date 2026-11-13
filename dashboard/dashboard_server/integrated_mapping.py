"""Shared raw LiDAR/IMU registration + soft GNSS map rebuild. No wheel input."""
import math
import threading
import time
import numpy as np
from dashboard_server.raw_mapping.raw_localization import RawLocalization, source_stamp
from dashboard_server.raw_mapping.gnss_constraints import UniqueGNSS, robust_rigid_alignment, smooth_position_constraints
from dashboard_server.tile_transport import StreamingOccupancyMap


def integrate(grid, scan, pose, t_s=None):
    ranges = []; angles = []
    for index, value in enumerate(scan['ranges']):
        if value is not None and math.isfinite(value) and scan['range_min'] <= value < scan['range_max']:
            ranges.append(value); angles.append(scan['angle_min'] + index*scan['angle_increment'])
    return grid.update(dict(stamp=source_stamp(scan),recording_t_s=t_s,
        preserve_hit_endpoints=True,
        pose=dict(x=float(pose[0]), y=float(pose[1]), yaw=float(pose[2])),
        localization='source_native_lidar_IMU_soft_GNSS',pose_source='encoderless_scan_registration',
        scan=dict(ranges=ranges,angles=angles,angle_min=0.,angle_increment=1.,
                  range_min=scan['range_min'],range_max=scan['range_max'])))


def corrected_poses(records, factors):
    stamps=np.array([r['stamp'] for r in records]);local=np.array([r['pose'] for r in records])
    factors=[f for f in factors if stamps[0] <= f.stamp <= stamps[-1]]
    pairs=np.array([[np.interp(f.stamp,stamps,local[:,axis]) for axis in range(2)] for f in factors]).reshape((-1,2))
    fit=robust_rigid_alignment(pairs,factors)
    if not fit['accepted']:
        return local, dict(state='local_frame',reason=fit['reason'],epochs=len(factors)), None
    world=local.copy();world[:,:2]=local[:,:2]@fit['rotation'].T+fit['translation'];world[:,2]+=fit['yaw_rad']
    keys=[0]
    for i in range(1,len(stamps)):
        if stamps[i]-stamps[keys[-1]] >= 1.: keys.append(i)
    if keys[-1] != len(stamps)-1: keys.append(len(stamps)-1)
    corrected, proof=smooth_position_constraints(stamps[keys],world[keys,:2],factors)
    shift=corrected-world[keys,:2]
    for axis in range(2): world[:,axis]+=np.interp(stamps,stamps[keys],shift[:,axis])
    return world, dict(state='GNSS_position_constrained',epochs=len(factors),**proof), dict(
        rotation=fit['rotation'],translation=fit['translation'],yaw=fit['yaw_rad'],last_shift=shift[-1])


class IntegratedMapping:
    def __init__(self, background_gnss=True, max_records=36000):
        self.lock=threading.RLock()
        self.localizer=RawLocalization(point_support_fallback=True)
        self.gnss=UniqueGNSS();self.records=[];self.segment=-1;self.generation=0
        self.map=StreamingOccupancyMap(max_beams=450,laser_x=0.,laser_y=0.)
        self.old_segments=[];self.frame='isolated_local_segment';self.basis=None
        self.diagnostic={};self.gps_status=dict(state='no_fix',epochs=0)
        self.background_gnss=background_gnss;self.max_records=max_records
        self.optimizing=False;self.last_optimization_stamp=0.;self.last_optimization_epochs=0
        self.input_scans=0;self.accepted=0;self.rejected=0
        self.closed=False

    def gps(self, sample):
        with self.lock:
            factor=self.gnss.add(sample)
            if not factor and not self.gnss.accepted:
                self.gps_status=dict(state='no_usable_fix',epochs=0)
            return factor

    def world_pose(self, pose):
        p=pose.copy()
        if self.basis:
            p[:2]=p[:2]@self.basis['rotation'].T+self.basis['translation']+self.basis['last_shift']
            p[2]+=self.basis['yaw']
        return p

    def add(self, scan, imu, draw=True, t_s=None):
        started=time.perf_counter()
        with self.lock:
            if self.closed:raise ValueError('mapping_engine_closed')
            if self.accepted>=self.max_records:raise ValueError('retained_scan_limit_new_map_required')
            self.input_scans+=1
            result=self.localizer.add(scan,imu)
            if not result['accepted']:
                self.rejected+=1;self.diagnostic=dict(reason=result['reason'],accepted=False,
                    elapsed_ms=(time.perf_counter()-started)*1000)
                return result
            self.accepted+=1
            if result['segment'] != self.segment:
                if self.records:
                    self.old_segments.append(dict(segment=self.segment,map=self.map,records=self.records,
                                                  frame=self.frame,gnss=dict(self.gps_status)))
                self.segment=result['segment'];self.records=[];self.basis=None
                self.map=StreamingOccupancyMap(max_beams=450,laser_x=0.,laser_y=0.)
                self.generation+=1;self.frame='isolated_local_segment'
                self.gps_status=dict(state='waiting_for_motion_baseline',epochs=len(self.gnss.accepted))
                self.last_optimization_stamp=0.;self.last_optimization_epochs=0
            if len(self.records)>=self.max_records:
                raise ValueError('retained_scan_limit_new_map_required')
            # ROS LaserScan ranges are float32. Retain exactly that precision
            # rather than thousands of boxed Python floats per historical scan.
            retained_scan={**scan,'ranges':np.asarray(scan['ranges'],dtype=np.float32)}
            self.records.append(dict(stamp=result['stamp'],pose=result['pose'].copy(),scan=retained_scan,t_s=t_s,draw=draw))
            if draw: integrate(self.map,scan,self.world_pose(result['pose']),t_s)
            self.diagnostic=dict(reason=result['reason'],accepted=True,
                registration_ms=float(result['elapsed_ms']),total_ms=(time.perf_counter()-started)*1000,
                input_scans=self.input_scans,accepted_scans=self.accepted,rejected_scans=self.rejected)
            if (self.background_gnss and not self.optimizing and len(self.gnss.accepted)>=8
                    and result['stamp']-self.last_optimization_stamp>=20
                    and len(self.gnss.accepted)-self.last_optimization_epochs>=8):
                self.optimizing=True;self.last_optimization_stamp=result['stamp']
                self.last_optimization_epochs=len(self.gnss.accepted)
                threading.Thread(target=self.optimize,name='map-GNSS-rebuild',daemon=True).start()
            return result

    def optimize(self):
        try:
            with self.lock:
                segment=self.segment;records=list(self.records);factors=list(self.gnss.accepted)
            world,proof,basis=corrected_poses(records,factors)
            if basis is None:
                with self.lock:
                    if self.segment==segment:self.gps_status=proof
                return
            grid=StreamingOccupancyMap(max_beams=450,laser_x=0.,laser_y=0.)
            for r,p in zip(records,world):
                if self.closed or not self.background_gnss:return
                if r['draw']:integrate(grid,r['scan'],p,r['t_s'])
            # Catch up outside the hot lock. Commit only a complete generation;
            # accepted scans arriving during rebuild are never lost or frozen.
            caught=len(records)
            while True:
                with self.lock:
                    if self.closed or not self.background_gnss or self.segment!=segment:return
                    tail=list(self.records[caught:])
                    if not tail:
                        self.map=grid;self.basis=basis;self.gps_status=proof
                        self.frame='fixed_ENU_GNSS_position_constrained';self.generation+=1
                        return
                for r in tail:
                    p=r['pose'].copy();p[:2]=p[:2]@basis['rotation'].T+basis['translation']+basis['last_shift'];p[2]+=basis['yaw']
                    if r['draw']:integrate(grid,r['scan'],p,r['t_s'])
                caught+=len(tail)
        except Exception as error:
            with self.lock:self.gps_status=dict(state='correction_failed_local_map_preserved',error=str(error)[:160])
        finally:
            with self.lock:self.optimizing=False

    def info(self):
        return dict(segment=self.segment,segments=len(self.old_segments)+int(bool(self.records)),
            coordinate_frame=self.frame,localization='source_native_lidar_IMU_soft_GNSS',
            pose_source='encoderless_scan_registration',source='original_LaserScan_and_raw_IMU',
            runtime_authority=False,autonomy_eligible=False,pose_accuracy_validated=False,
            mount_height_m=.60,mount_rotation_calibrated=False,deskew_applied=False,
            gnss={**self.gps_status, 'rebuilding':self.optimizing}, diagnostic=dict(self.diagnostic),
            no_encoder=True,PWM_used_as_odometry=False,camera_used_for_mapping=False,
            note='GPS·IMU·LiDAR 통합 · 센서 품질에 따라 보정 · 위치 추정 공백은 별도 지도로 보존')
