"""Experimental full-source planar registration with IMU quality checks.

No wheel/PWM/known-orchard/truth/camera input. Unknown IMU-to-laser rotation is
not replaced with a fabricated extrinsic: relative quaternion rotation bounds
LiDAR rotation, while no raw absolute yaw/roll/pitch rotates the map. This is an
offline diagnostic candidate, without motion authority or accuracy guarantees.
Rejected gaps remain discontinuities; they are never zero-velocity updates.
"""
from dataclasses import dataclass
import math,time
import numpy as np
from scipy.spatial import cKDTree
from collections import Counter
from .fit_workers import evaluate

MAX_BEAMS=4096  # Recorded S2 bags have 3,240 beams, not exactly 3,200.


def source_stamp(message):
    s=message['header']['stamp']
    return s['sec']+s['nanosec']*1e-9


def rotate(theta):
    c,s=math.cos(theta),math.sin(theta)
    return np.array([[c,-s],[s,c]])


def quaternion(message):
    if (message.get('orientation_covariance') or [0])[0]<0:raise ValueError('orientation_unavailable')
    q=np.array([message['orientation'][k] for k in 'xyzw'],dtype=float)
    norm=np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or not .9<norm<1.1:raise ValueError('invalid_quaternion')
    return q/norm


def angular_change(first,second):
    return 2*math.acos(float(np.clip(abs(np.dot(first,second)),0,1)))


def scan_points(message):
    ranges=np.asarray(message['ranges'],dtype=float)
    if not 24<=len(ranges)<=MAX_BEAMS:raise ValueError('bounded_scan_size_exceeded')
    amin,increment=float(message['angle_min']),float(message['angle_increment'])
    if not math.isfinite(amin) or not math.isfinite(increment) or increment<=0 or len(ranges)*increment>2*math.pi+2*increment:
        raise ValueError('invalid_scan_angles')
    valid=np.isfinite(ranges)&(ranges>=max(.05,message['range_min']))&(ranges<min(20.,message['range_max']))
    angles=amin+np.nonzero(valid)[0]*increment
    points=np.column_stack((np.cos(angles)*ranges[valid],np.sin(angles)*ranges[valid]))
    if len(points)<120:raise ValueError('insufficient_returns')
    return points


@dataclass
class Surfaces:
    points: np.ndarray
    normals: np.ndarray
    tree: object
    denominator: int
    dense: object = None
    dense_tree: object = None


def surfaces(dense,maximum=480):
    dense=np.asarray(dense,dtype=float)
    anchors=dense[np.linspace(0,len(dense)-1,min(maximum,len(dense)),dtype=int)]
    tree=cKDTree(dense);distances,indices=tree.query(anchors,k=8,workers=1)
    neighborhoods=dense[indices];center=neighborhoods.mean(axis=1)
    delta=neighborhoods-center[:,None,:]
    covariance=np.einsum('nki,nkj->nij',delta,delta)/8
    eigen,vectors=np.linalg.eigh(covariance)
    supported=(distances[:,-1]<.35)&(eigen[:,1]>1e-6)&(eigen[:,0]<eigen[:,1]*.25)
    p=anchors[supported];normals=vectors[supported,:,0]
    if len(p)<60:raise ValueError('insufficient_observed_surfaces')
    return Surfaces(p,normals,cKDTree(p),len(anchors),dense,tree)


def fit_surfaces(target,current,theta,initial_xy,max_translation,max_yaw,failures=None,yaw_prior=None):
    def failed(reason):
        if failures is not None:failures[reason]+=1
        return None
    xy=np.asarray(initial_xy,dtype=float).copy()
    for _ in range(15):
        moved=current.points@rotate(theta).T+xy
        distance,index=target.tree.query(moved,workers=1)
        normals=target.normals[index];difference=target.points[index]-moved
        residual=np.einsum('ni,ni->n',difference,normals)
        aligned=np.abs(np.einsum('ni,ni->n',current.normals@rotate(theta).T,normals))>.7
        good=(distance<.30)&(np.abs(residual)<.15)&aligned
        if good.sum()<60:return failed('iteration_insufficient_correspondences')
        p=moved[good];n=normals[good];r=residual[good]
        jac=np.column_stack((n[:,0],n[:,1],-n[:,0]*p[:,1]+n[:,1]*p[:,0]))
        weight=np.minimum(1.,.04/np.maximum(np.abs(r),1e-9))
        matrix=jac*np.sqrt(weight[:,None]);rhs=r*np.sqrt(weight)
        if yaw_prior is not None:
            expected,sigma=yaw_prior
            # Point-to-line residuals are metres, the gyro residual is radians.
            # Convert the prior to the same residual scale (30 mm sensor floor).
            # Using 1/sigma beside unwhitened metre residuals made a noisy gyro
            # overpower observed geometry and smear distant objects into lines.
            prior_weight=.03/sigma
            matrix=np.vstack((matrix,[0.,0.,prior_weight]));rhs=np.r_[rhs,(expected-theta)*prior_weight]
        delta=np.linalg.lstsq(matrix,rhs,rcond=1e-6)[0]
        # Each step applies a left SE(2) correction, including the translated
        # scan's rotational lever arm, rather than independently adding yaw.
        theta+=float(delta[2]);xy=rotate(float(delta[2]))@xy+delta[:2]
        if np.linalg.norm(xy)>max_translation:return failed('search_translation_bound')
        if abs(theta)>max_yaw:return failed('search_rotation_bound')
        if np.linalg.norm(delta[:2])<.0005 and abs(delta[2])<.0001:break
    moved=current.points@rotate(theta).T+xy
    distance,index=target.tree.query(moved,workers=1);normals=target.normals[index]
    residual=np.einsum('ni,ni->n',target.points[index]-moved,normals)
    aligned=np.abs(np.einsum('ni,ni->n',current.normals@rotate(theta).T,normals))>.7
    good=(distance<.25)&(np.abs(residual)<.12)&aligned
    coverage=int(good.sum())/current.denominator
    if good.sum()<60:return failed('final_insufficient_correspondences')
    if coverage<.4:return failed('final_insufficient_overlap')
    p=moved[good];n=normals[good]
    jac=np.column_stack((n[:,0],n[:,1],(-n[:,0]*p[:,1]+n[:,1]*p[:,0])/max(1.,np.median(np.linalg.norm(p,axis=1)))))
    information=jac.T@jac/len(p);eigen=np.linalg.eigvalsh(information)
    xy_eigen=np.linalg.eigvalsh(n.T@n/len(n))
    rmse=float(np.sqrt(np.mean(residual[good]**2)))
    if xy_eigen[0]<.06:return failed('translation_geometrically_degenerate')
    if eigen[0]<.002 or eigen[-1]/eigen[0]>200:return failed('pose_information_degenerate')
    if rmse>.08:return failed('excessive_point_to_line_residual')
    return dict(xy=xy,yaw=theta,coverage=coverage,rmse_m=rmse,inliers=int(good.sum()),information_eigenvalues=eigen.tolist())


def registration(target,current,imu_rotation,dt,velocity=None):
    # Experimental search bounds, not maximum vehicle speed or safety limits.
    max_translation=min(1.,.10+3.*dt);max_yaw=math.radians(1.5+120.*dt)
    prediction=np.zeros(2) if velocity is None else np.asarray(velocity)*dt
    seeds=[(0.,np.zeros(2)),(0.,prediction)]
    if imu_rotation>math.radians(.15):seeds.extend([(imu_rotation,prediction),(-imu_rotation,prediction)])
    results=[];failures=Counter()
    def solve(seed):
        theta,xy=seed;diagnostic=Counter()
        return fit_surfaces(target,current,theta,xy,max_translation,max_yaw,diagnostic),diagnostic
    unique=[]
    for seed in seeds:
        if not any(seed[0]==other[0] and np.array_equal(seed[1],other[1]) for other in unique):unique.append(seed)
    for candidate,diagnostic in evaluate(solve,unique):
        failures.update(diagnostic)
        if candidate:
            if abs(candidate['yaw'])<=imu_rotation+math.radians(3):results.append(candidate)
            else:failures['IMU_relative_rotation_disagreement']+=1
    if not results:return dict(accepted=False,reason='unsupported_or_inconsistent_registration',fit_rejection_counts=dict(failures),current_supported_points=len(current.points),sampled_point_denominator=current.denominator)
    results.sort(key=lambda r:(-r['coverage'],r['rmse_m']));best=results[0]
    for other in results[1:]:
        separated=np.linalg.norm(other['xy']-best['xy'])>.08 or abs(other['yaw']-best['yaw'])>math.radians(1)
        comparable=other['coverage']>=best['coverage']-.03 and other['rmse_m']<=max(.015,best['rmse_m']*1.3)
        if separated and comparable:return dict(accepted=False,reason='ambiguous_registration')
    return dict(accepted=True,reason='candidate_planar_fit',**best)


class RawLocalization:
    def __init__(self,point_support_fallback=False,reference_history=False):
        self.segment=-1;self.previous=None;self.pose=np.zeros(3);self.velocity=None
        self.last_seen=None;self.previous_imu=None;self.rejections=0
        self.point_support_fallback=point_support_fallback
        self.reference=None
        if reference_history:
            from raw_reference import RecentGeometry
            self.reference=RecentGeometry()
    def reject(self,stamp,reason,diagnostic=None):
        self.rejections+=1
        return dict(stamp=stamp,accepted=False,reason=reason,segment=self.segment,pose=None,diagnostic=diagnostic or {},autonomy_eligible=False)
    def add(self,scan,imu):
        stamp=source_stamp(scan);started=time.perf_counter()
        if not math.isfinite(stamp) or stamp<=0:return self.reject(stamp,'invalid_scan_stamp')
        if self.last_seen is not None and stamp<=self.last_seen:return self.reject(stamp,'scan_clock_nonmonotonic')
        self.last_seen=stamp
        try:
            if imu is None or abs(source_stamp(imu)-stamp)>.10:raise ValueError('waiting_for_source_aligned_IMU')
            q=quaternion(imu)
            if (imu.get('linear_acceleration_covariance') or [0])[0]<0:
                raise ValueError('linear_acceleration_unavailable')
            acceleration=np.array([imu['linear_acceleration'][k] for k in 'xyz'])
            if not np.all(np.isfinite(acceleration)) or abs(np.linalg.norm(acceleration)-9.80665)>6.:
                raise ValueError('IMU_shock_or_invalid_acceleration')
            points=scan_points(scan);current=surfaces(points)
        except (ValueError,KeyError,TypeError) as error:return self.reject(stamp,str(error))
        gap=None if self.previous is None else stamp-self.previous[0]
        if self.previous is None or gap>1.:
            # Unknown lost travel cannot be drawn as a chord or zero odometry.
            self.segment+=1;self.pose=np.zeros(3);self.velocity=None
            if self.reference:self.reference.clear()
            result=dict(accepted=True,reason='initial_segment' if self.previous is None else 'new_segment_after_unknown_gap')
        else:
            target=self.previous[1];reference_points=0
            result=registration(target,current,angular_change(self.previous_imu,q),gap,self.velocity)
            if not result['accepted'] and self.point_support_fallback:
                from .raw_point_support import point_support_registration
                alternative=point_support_registration(target,current,angular_change(self.previous_imu,q),gap,self.velocity)
                if alternative['accepted']:result=alternative
                else:result['point_support_rejection']=alternative
            # A fused reference has different occlusion and angular density.
            # It must not replace a successful adjacent-scan registration.
            # Try it only for a rejected fit, keeping the same observability,
            # residual, ambiguity and IMU gates (not relaxing them for trees).
            if not result['accepted'] and self.reference:
                retained=self.reference.points(self.pose)
                if retained is not None:
                    try:
                        reference=surfaces(retained)
                        alternative=registration(reference,current,angular_change(self.previous_imu,q),gap,self.velocity)
                        if not alternative['accepted'] and self.point_support_fallback:
                            from .raw_point_support import point_support_registration
                            alternative=point_support_registration(reference,current,angular_change(self.previous_imu,q),gap,self.velocity)
                        if alternative['accepted']:
                            result=alternative;reference_points=len(retained)
                            result['reason']='experimental_recent_reference_fallback'
                    except ValueError:pass
            if not result['accepted']:return self.reject(stamp,result['reason'],{k:v for k,v in result.items() if k not in ('accepted','reason')})
            self.pose[:2]+=rotate(self.pose[2])@result['xy'];self.pose[2]+=result['yaw']
            self.velocity=rotate(-result['yaw'])@(result['xy']/gap)
            result['recent_reference_points']=reference_points
        if self.reference:self.reference.add(points,self.pose)
        self.previous=(stamp,current);self.previous_imu=q
        return dict(stamp=stamp,segment=self.segment,pose=self.pose.copy(),
                    elapsed_ms=(time.perf_counter()-started)*1000,imu_role='source_pairing_shock_and_relative_rotation_bound',
                    deskew_applied=False,mount_rotation_calibrated=False,autonomy_eligible=False,**result)
