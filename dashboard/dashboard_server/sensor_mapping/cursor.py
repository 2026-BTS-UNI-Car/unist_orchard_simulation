"""Unchanged causal cursor and raster integration from deployed actual mapper."""
import bisect,json,math
from .raw_mapping.continuous_localization import ContinuousLocalization
from .raw_mapping.raw_localization import source_stamp
from .raw_mapping.gnss_constraints import UniqueGNSS,robust_rigid_alignment
class SourceCursorLocalizer:
    """Ordered raw prefix only; independent of raster/storage and HTTP work."""
    def __init__(self):
        self.engine=ContinuousLocalization();self.gnss=UniqueGNSS()
        self.imus=[];self.stamps=[];self.pending_imus=[];self.pairs=[];self.basis=None
        self.input_scans=0;self.note=dict(state='no_usable_fix')

    def consume(self,batch):
        engine=self.engine;result=None
        for row in batch:
            topic=row['topic'];message=row['message']
            if topic=='/imu/data_raw':
                self.pending_imus.append(message)
                i=bisect.bisect_right(self.stamps,source_stamp(message))
                self.imus.insert(i,message);self.stamps.insert(i,source_stamp(message))
                if len(self.imus)>64:self.imus.pop(0);self.stamps.pop(0)
            elif topic=='/gps/data':
                sample=json.loads(message['data']);factor=self.gnss.add(sample)
                if factor:engine.gps(sample)
                if factor and engine.previous:
                    self.pairs.append((engine.pose[:2].copy(),factor))
                    if self.basis is None and len(self.pairs)>=8:
                        fit=robust_rigid_alignment(np_array([p for p,f in self.pairs]),[f for p,f in self.pairs])
                        if fit['accepted']:self.basis=fit
                    if self.basis:
                        local=(np_array(factor.xy)-self.basis['translation'])@self.basis['rotation']
                        scale=math.hypot(factor.scale_m,2*float(np_array(local).dot(np_array(local)))**.5*self.basis['heading_uncertainty_lower_bound_rad'])
                        engine.gps_prior(factor.stamp,local,scale)
                    self.note=dict(state='position_consistency_validation' if self.basis else 'waiting_for_motion_baseline',epochs=len(self.gnss.accepted),reported_hacc_m=factor.reported_hacc_m)
            elif topic=='/scan':
                stamp=source_stamp(message);index=bisect.bisect_right(self.stamps,stamp)-1
                self.pending_imus.sort(key=source_stamp)
                while self.pending_imus and source_stamp(self.pending_imus[0])<=stamp:engine.imu(self.pending_imus.pop(0))
                result=engine.add(message,self.imus[index] if index>=0 else None);self.input_scans+=1
        return result,dict(input_scans=self.input_scans,rejected_scans=engine.rejected,
                          motion_prior_scans=engine.motion_prior_count),self.note

def np_array(values):
    import numpy as np
    return np.asarray(values)

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
