"""Conservative offline GNSS factors; no body heading, motor, ROS or TF output.

ENU origin is fixed per archive. Host sensor stamps are the alignment clock;
receiver UTC identifies independent navigation epochs, including old host/UTC
offsets. Accuracy is a conservative factor scale, not a claimed calibrated
covariance. A caller must supply continuous, independently accepted LiDAR poses.
"""
from dataclasses import dataclass
from datetime import datetime
import math
import numpy as np


def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def ecef(latitude,longitude,altitude=0.):
    lat,lon=math.radians(latitude),math.radians(longitude)
    a=6378137.;e2=6.6943799901413165e-3
    n=a/math.sqrt(1-e2*math.sin(lat)**2)
    return np.array([(n+altitude)*math.cos(lat)*math.cos(lon),
                     (n+altitude)*math.cos(lat)*math.sin(lon),
                     (n*(1-e2)+altitude)*math.sin(lat)])


class FixedENU:
    def __init__(self,latitude,longitude):
        self.latitude,self.longitude=latitude,longitude
        self.origin=ecef(latitude,longitude)
        lat,lon=math.radians(latitude),math.radians(longitude)
        self.rotation=np.array([[-math.sin(lon),math.cos(lon),0],
            [-math.sin(lat)*math.cos(lon),-math.sin(lat)*math.sin(lon),math.cos(lat)],
            [math.cos(lat)*math.cos(lon),math.cos(lat)*math.sin(lon),math.sin(lat)]])
    def xy(self,latitude,longitude):
        return (self.rotation@(ecef(latitude,longitude)-self.origin))[:2]


@dataclass(frozen=True)
class GNSSFactor:
    stamp: float
    receiver_epoch: str
    xy: tuple
    scale_m: float
    reported_hacc_m: float
    fix_type: str
    def __post_init__(self):
        if (not finite(self.stamp) or self.stamp<0 or not isinstance(self.receiver_epoch,str) or not self.receiver_epoch
                or len(self.xy)!=2 or not all(finite(v) for v in self.xy)
                or not finite(self.scale_m) or self.scale_m<.1
                or not finite(self.reported_hacc_m) or self.reported_hacc_m<=0):
            raise ValueError('invalid_GNSS_factor')


class UniqueGNSS:
    def __init__(self):
        self.origin=None;self.seen=set();self.accepted=[];self.rejected={}
    def reject(self,reason):
        self.rejected[reason]=self.rejected.get(reason,0)+1
        return None
    def add(self,sample):
        if sample.get('valid') is not True:return self.reject('invalid_fix')
        lat,lon,hacc=(sample.get(k) for k in ('latitude','longitude','h_acc_m'))
        if not all(finite(v) for v in (lat,lon,hacc)) or not -90<=lat<=90 or not -180<=lon<=180 or not 0<hacc<=50:
            return self.reject('missing_or_invalid_position_accuracy')
        header=sample.get('stamp') or {}
        sec,nsec=header.get('sec'),header.get('nanosec')
        if not all(finite(v) for v in (sec,nsec)) or sec<0 or not 0<=nsec<1e9:
            return self.reject('invalid_host_source_stamp')
        source=sec+nsec*1e-9
        try:
            utc=datetime.fromisoformat(sample['gps_time'])
            if utc.tzinfo is None:raise ValueError('UTC timezone missing')
            epoch=utc.isoformat()
        except (KeyError,TypeError,ValueError):return self.reject('missing_receiver_epoch')
        # UTC identifies the solution epoch, regardless of differing publication
        # sequence/host delivery stamps. Never shift source time to receiver UTC.
        if epoch in self.seen:return self.reject('duplicate_receiver_epoch')
        if self.accepted and source<=self.accepted[-1].stamp:return self.reject('host_clock_nonmonotonic')
        self.seen.add(epoch)
        if self.origin is None:self.origin=FixedENU(lat,lon)
        fix=str(sample.get('fix_type') or 'unknown')
        # Unknown antenna lever arm and uncalibrated receiver error remain in
        # the factor. Even RTK labels cannot create a zero-variance constraint.
        floor=1. if fix in ('autonomous','unknown') else .10
        scale=math.hypot(max(floor,hacc),.50)
        factor=GNSSFactor(source,epoch,tuple(self.origin.xy(lat,lon)),scale,hacc,fix)
        self.accepted.append(factor)
        return factor


def robust_rigid_alignment(local_xy,factors,minimum_span_m=8.):
    """Align one CONTINUOUS local segment without using GNSS course as yaw.

    This is a diagnostic frame fit, not yaw calibration. Short/noisy trajectories
    return an explicit rejection, preserving local odometry. No scale is fitted.
    """
    p=np.asarray(local_xy,dtype=float)
    if p.shape!=(len(factors),2) or not np.all(np.isfinite(p)):
        raise ValueError('invalid_corresponding_local_poses')
    if len(factors)<8:return dict(accepted=False,reason='too_few_independent_epochs')
    q=np.asarray([f.xy for f in factors]);sigma=np.asarray([f.scale_m for f in factors])
    weights=1/sigma**2;rotation=np.eye(2);translation=np.zeros(2)
    for _ in range(8):
        weights=weights/max(weights.sum(),1e-12)
        pc=(p*weights[:,None]).sum(axis=0);qc=(q*weights[:,None]).sum(axis=0)
        cross=(p-pc).T@((q-qc)*weights[:,None]);u,_,vt=np.linalg.svd(cross)
        rotation=vt.T@u.T
        if np.linalg.det(rotation)<0:
            vt[-1]*=-1;rotation=vt.T@u.T
        translation=qc-rotation@pc
        residual=np.linalg.norm(p@rotation.T+translation-q,axis=1)
        weights=(1/sigma**2)*np.minimum(1.,2.5*sigma/np.maximum(residual,1e-9))
    centered=np.linalg.norm(p-pc,axis=1)
    span=float(np.quantile(centered,.95)*2)
    required=max(minimum_span_m,5*float(np.median(sigma)))
    if span<required:return dict(accepted=False,reason='insufficient_motion_baseline_for_GNSS_heading',span_m=span,required_span_m=required)
    normalized=residual/sigma
    if np.median(normalized)>2.5 or np.mean(normalized<=3)<.7:
        return dict(accepted=False,reason='GNSS_local_trajectory_disagreement',median_normalized_residual=float(np.median(normalized)))
    return dict(accepted=True,reason='diagnostic_rigid_alignment',rotation=rotation,translation=translation,
                yaw_rad=math.atan2(rotation[1,0],rotation[0,0]),span_m=span,
                median_residual_m=float(np.median(residual)),
                heading_uncertainty_lower_bound_rad=math.atan2(math.sqrt(2)*float(np.median(sigma)),span),
                physical_heading_calibrated=False,autonomy_eligible=False)


def smooth_position_constraints(stamps,local_world_xy,factors,drift_scale_m_per_s=.15):
    """Sparse offline positional graph with soft, unique-epoch GNSS factors.

    Keyframe increments stay in a fixed already aligned frame. Corrected poses
    require rebuilding retained scans/submaps; never move only the rover marker
    over an uncorrected occupancy raster. This does not solve yaw/loop closure.
    """
    from scipy.sparse import coo_matrix
    from scipy.sparse.linalg import lsqr
    t=np.asarray(stamps,dtype=float);p=np.asarray(local_world_xy,dtype=float)
    if len(t)<2 or p.shape!=(len(t),2) or not np.all(np.isfinite(t)) or not np.all(np.isfinite(p)) or np.any(np.diff(t)<=0):
        raise ValueError('invalid_continuous_keyframe_poses')
    if not finite(drift_scale_m_per_s) or drift_scale_m_per_s<=0:raise ValueError('invalid_drift_scale')
    # Dedupe again at this boundary so arbitrary callers cannot overweight GPS.
    independent={}
    for factor in factors:
        if t[0]<=factor.stamp<=t[-1]:independent.setdefault(factor.receiver_epoch,factor)
    factors=list(independent.values())
    if not factors:return p.copy(),dict(used_factors=0,corrected=False,reason='no_GNSS_in_segment')
    delta=np.diff(p,axis=0);dt=np.diff(t)
    edge_scale=np.maximum(.02,drift_scale_m_per_s*np.sqrt(dt)+.01*np.linalg.norm(delta,axis=1))
    result=p.copy();robust=np.ones(len(factors))
    for _ in range(5):
        rows=[];cols=[];values=[];rhs=[];row_index=0
        for i,(step,scale) in enumerate(zip(delta,edge_scale)):
            rows.extend([row_index,row_index]);cols.extend([i,i+1]);values.extend([-1/scale,1/scale]);rhs.append(step/scale);row_index+=1
        # A weak local origin prior keeps the system bounded without dominating
        # actual GNSS accuracy. Linear interpolation uses real keyframe stamps.
        rows.append(row_index);cols.append(0);values.append(.1);rhs.append(p[0]*.1);row_index+=1
        for k,factor in enumerate(factors):
            j=min(len(t)-2,max(0,int(np.searchsorted(t,factor.stamp)-1)))
            fraction=(factor.stamp-t[j])/(t[j+1]-t[j]);weight=math.sqrt(robust[k])/factor.scale_m
            rows.extend([row_index,row_index]);cols.extend([j,j+1]);values.extend([(1-fraction)*weight,fraction*weight]);rhs.append(np.asarray(factor.xy)*weight);row_index+=1
        matrix=coo_matrix((values,(rows,cols)),shape=(row_index,len(t))).tocsr();rhs=np.asarray(rhs)
        solutions=[lsqr(matrix,rhs[:,axis],atol=1e-8,btol=1e-8,iter_lim=max(200,10*len(t))) for axis in range(2)]
        if any(solution[1] not in (0,1,2) for solution in solutions):raise ValueError('GNSS_graph_solver_not_converged')
        result=np.column_stack([solution[0] for solution in solutions])
        residual=np.asarray([math.hypot(*(np.array([np.interp(f.stamp,t,result[:,axis]) for axis in range(2)])-f.xy))/f.scale_m for f in factors])
        robust=np.minimum(1.,2.5/np.maximum(residual,1e-9))
    return result,dict(used_factors=len(factors),corrected=True,median_normalized_GNSS_residual=float(np.median(residual)),
                       maximum_correction_m=float(np.max(np.linalg.norm(result-p,axis=1))),
                       factor_scale='reported_accuracy_with_floor_and_unknown_antenna_lever_arm',
                       covariance_calibrated=False,poses_require_map_rebuild=True,
                       yaw_optimized=False,autonomy_eligible=False)
