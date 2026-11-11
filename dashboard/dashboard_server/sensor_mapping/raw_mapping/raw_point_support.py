"""Experimental dense-point proposal with a separate normal observability gate.

Curved/compact returns need not themselves look like planar wall surfaces. Fit
the complete sampled current scan, then check separately measured surface
support in both XY directions. Full-scan coverage is not renormalized after
filtering. This is not semantic trunk recognition or proof of a static scene.
"""
import math
import numpy as np
from .raw_localization import rotate
from .fit_workers import evaluate


def proposal(target,current,theta,initial_xy,translation_bound,rotation_bound,
             min_coverage=.7,max_rmse=.11,min_normal_support=60,min_normal_eigen=.06,max_distance=.2):
    source=current.dense[np.linspace(0,len(current.dense)-1,min(480,len(current.dense)),dtype=int)]
    xy=np.asarray(initial_xy).copy()
    for _ in range(15):
        moved=source@rotate(theta).T+xy
        distance,index=target.dense_tree.query(moved,workers=1)
        limit=min(.3,max(.04,float(np.quantile(distance,.8))))
        good=distance<=limit
        if good.sum()<100:return None
        p=moved[good];q=target.dense[index[good]];r=distance[good]
        weight=np.minimum(1.,.05/np.maximum(r,1e-9));weight/=weight.sum()
        pc=(p*weight[:,None]).sum(axis=0);qc=(q*weight[:,None]).sum(axis=0)
        u,_,vt=np.linalg.svd((p-pc).T@((q-qc)*weight[:,None]));rotation=vt.T@u.T
        if np.linalg.det(rotation)<0:vt[-1]*=-1;rotation=vt.T@u.T
        shift=qc-rotation@pc
        correction=math.atan2(rotation[1,0],rotation[0,0]);theta+=correction;xy=rotation@xy+shift
        if np.linalg.norm(xy)>translation_bound or abs(theta)>rotation_bound:return None
        if np.linalg.norm(shift)<.0005 and abs(correction)<.0001:break
    moved=source@rotate(theta).T+xy;distance,index=target.dense_tree.query(moved,workers=1)
    good=distance<max_distance;coverage=float(good.mean())
    if coverage<min_coverage:return None
    rmse=float(np.sqrt(np.mean(distance[good]**2)))
    if rmse>max_rmse:return None
    # Full-density point fits alone can invent translation along a single wall.
    # Validate a separate normal-supported subset at this candidate transform.
    support=current.points@rotate(theta).T+xy
    distance,indices=target.tree.query(support,workers=1)
    normals=target.normals[indices]
    aligned=np.abs(np.einsum('ni,ni->n',current.normals@rotate(theta).T,normals))>.7
    accepted=(distance<.25)&aligned
    if accepted.sum()<min_normal_support:return None
    n=normals[accepted];p=support[accepted]
    residual=np.einsum('ni,ni->n',target.points[indices[accepted]]-p,n)
    if float(np.sqrt(np.mean(residual**2)))>.08:return None
    eigen=np.linalg.eigvalsh(n.T@n/len(n))
    if eigen[0]<min_normal_eigen:return None
    radius=max(1.,float(np.median(np.linalg.norm(p,axis=1))))
    jac=np.column_stack((n[:,0],n[:,1],(-n[:,0]*p[:,1]+n[:,1]*p[:,0])/radius))
    information=np.linalg.eigvalsh(jac.T@jac/len(n))
    if information[0]<.002 or information[-1]/information[0]>200:return None
    return dict(xy=xy,yaw=theta,coverage=coverage,rmse_m=rmse,normal_support_count=int(accepted.sum()),normal_information_eigenvalues=information.tolist())


def point_support_registration(target,current,imu_rotation,dt,velocity=None):
    prediction=np.zeros(2) if velocity is None else np.asarray(velocity)*dt
    seeds=[(0.,np.zeros(2)),(0.,prediction)]
    if imu_rotation>math.radians(.15):seeds.extend([(imu_rotation,prediction),(-imu_rotation,prediction)])
    candidates=[]
    def solve(seed):
        yaw,xy=seed
        return proposal(target,current,yaw,xy,min(1.,.1+3*dt),math.radians(1.5+120*dt))
    unique=[]
    for seed in seeds:
        if not any(seed[0]==other[0] and np.array_equal(seed[1],other[1]) for other in unique):unique.append(seed)
    for value in evaluate(solve,unique):
        if value and abs(value['yaw'])<=imu_rotation+math.radians(3):candidates.append(value)
    if not candidates:return dict(accepted=False,reason='point_and_normal_support_insufficient')
    candidates.sort(key=lambda v:(-v['coverage'],v['rmse_m']));best=candidates[0]
    for other in candidates[1:]:
        different=np.linalg.norm(other['xy']-best['xy'])>.08 or abs(other['yaw']-best['yaw'])>math.radians(1)
        similar=other['coverage']>=best['coverage']-.03 and other['rmse_m']<=max(.015,best['rmse_m']*1.3)
        if different and similar:return dict(accepted=False,reason='ambiguous_point_support_registration')
    return dict(accepted=True,reason='experimental_dense_points_with_separate_normal_support',**best)
