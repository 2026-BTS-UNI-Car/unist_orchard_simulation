"""Offline fixed-IMU-yaw compact-surface translation proposals.

No ROS, wheel/command input or execution authority. ``shadow_eligible`` is a
bounded diagnostic contract for a separately isolated shadow consumer, not
motion accuracy. A caller's evidence must be established independently; a small
fit residual or this module's correspondences cannot establish static support.
All original ranges remain attached to the result for separate collision use.
Features are observed surfaces, never semantic trunk labels or free space.
"""
from dataclasses import dataclass, field
import math
from numbers import Integral, Real

import numpy as np


MAX_RAW_BEAMS = 3200
MAX_FEATURES = 128
MAX_PAIR_INTERVAL_NS = 150_000_000
_FIT_CACHE_CAPACITY = 512


def _integer(value, minimum=0, maximum=2**63-1):
    return (isinstance(value, Integral) and not isinstance(value, bool)
            and minimum <= int(value) <= maximum)


def _finite(value):
    try:
        return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)
    except OverflowError:
        return False


def _label(value):
    return isinstance(value, str) and bool(value.strip()) and len(value) <= 256


@dataclass(frozen=True)
class CompactScan:
    epoch: str
    frame_id: str
    stamp_ns: int
    ranges: tuple
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    time_increment: float

    def __post_init__(self):
        if (not _label(self.epoch) or not _label(self.frame_id) or not _integer(self.stamp_ns)
                or not isinstance(self.ranges, tuple) or not 24 <= len(self.ranges) <= MAX_RAW_BEAMS
                or not all(v is None or (isinstance(v, Real) and not isinstance(v, bool)) for v in self.ranges)
                or not all(_finite(v) for v in (self.angle_min, self.angle_increment,
                                               self.range_min, self.range_max, self.time_increment))
                or not 0 < self.range_min < self.range_max <= 30.001
                or self.time_increment < 0 or self.angle_increment <= 0
                or abs(self.angle_min) > 2*math.pi
                or len(self.ranges)*self.angle_increment > 2*math.pi+self.angle_increment*1.01):
            raise ValueError('invalid_bounded_compact_scan')
        # NumPy unsigned/signed scalars can promote mixed subtraction or bounds
        # comparisons to float above 2**53. Preserve ns exactly before arithmetic.
        object.__setattr__(self,'stamp_ns',int(self.stamp_ns))
        # Reject integers that cannot safely become bounded float observations.
        if any(isinstance(v, Integral) and abs(int(v)) > 2**63-1 for v in self.ranges if v is not None):
            raise ValueError('unbounded_range_integer')
        try:
            for value in self.ranges:
                if value is not None:
                    float(value)
        except (TypeError,ValueError,OverflowError) as error:
            raise ValueError('unrepresentable_range_number') from error


@dataclass(frozen=True)
class CompactConfig:
    """Experimental compact-specific gates, not field-calibrated guarantees."""
    cluster_gap_m: float = .25
    min_cluster_points: int = 3
    max_cluster_width_m: float = 1.
    max_hypotheses: int = 256
    max_iterations: int = 15
    max_translation_m: float = .6
    association_distance_m: float = .25
    convergence_m: float = 1e-4
    min_associations: int = 6
    min_matched_returns: int = 24
    min_overlap: float = .45
    max_rmse_m: float = .05
    min_spread_eigenvalue_m2: float = .05
    max_spread_condition: float = 20.
    ambiguity_distance_m: float = .08
    ambiguity_support_margin: int = 1

    def __post_init__(self):
        for name, minimum, maximum in (
                ('min_cluster_points',3,100),('max_hypotheses',1,512),('max_iterations',1,15),
                ('min_associations',6,MAX_FEATURES),('min_matched_returns',24,MAX_RAW_BEAMS),
                ('ambiguity_support_margin',1,10)):
            if not _integer(getattr(self,name),minimum,maximum):
                raise ValueError('invalid_compact_count_bound')
        for name, maximum in (('cluster_gap_m',.25),('max_cluster_width_m',1.),
                ('max_translation_m',.6),('association_distance_m',.25),('convergence_m',1e-4),
                ('max_rmse_m',.05),('ambiguity_distance_m',.08)):
            if not _finite(getattr(self,name)) or not 0 < getattr(self,name) <= maximum:
                raise ValueError('invalid_or_relaxed_compact_distance')
        if (not _finite(self.min_overlap) or not .45 <= self.min_overlap <= 1
                or not _finite(self.min_spread_eigenvalue_m2) or self.min_spread_eigenvalue_m2 < .05
                or not _finite(self.max_spread_condition) or not 1 <= self.max_spread_condition <= 20):
            raise ValueError('invalid_or_relaxed_compact_quality')


@dataclass(frozen=True)
class ShadowEvidence:
    """Caller assertions with explicit provenance; never inferred from this fit.

    A known-flat-fixture profile may establish static geometry externally. A
    field profile additionally requires independently tracked associations;
    reusing this pair's residual as that evidence is circular and unsupported.
    ``temporal_pairs`` indexes deterministic feature lists in this exact pair.
    This object is not cryptographic attestation or proof of its own assertions.
    """
    profile_id: str
    provenance: str  # known_flat_fixture or field_verified_assumptions
    evidence_ref: str
    epoch: str
    frame_id: str
    previous_stamp_ns: int
    stamp_ns: int
    instantaneous_verified: bool
    calibrated_mount_verified: bool
    imu_yaw_verified: bool
    planar_verified: bool
    height_change_excluded: bool
    independent_static_majority_verified: bool
    temporal_evidence_ref: str | None = None
    temporal_pairs: tuple[tuple[int, int], ...] = ()

    def __post_init__(self):
        # Invalid evidence is reported by the result, while valid integer clocks
        # are canonicalized before exact scan/evidence context comparisons.
        for name in ('previous_stamp_ns','stamp_ns'):
            value=getattr(self,name)
            if _integer(value):
                object.__setattr__(self,name,int(value))


@dataclass(frozen=True)
class CompactFeature:
    center_xy: tuple[float, float]
    width_m: float
    beam_indices: tuple[int, ...]


@dataclass(frozen=True)
class FeatureSet:
    features: tuple[CompactFeature, ...]
    raw_group_count: int
    finite_return_count: int
    tiny_group_count: int
    wide_group_count: int
    reason: str


@dataclass(frozen=True)
class CompactMode:
    delta_xy: tuple[float, float]
    matches: tuple[tuple[int, int], ...]
    overlap: float
    rmse_m: float
    matched_returns: tuple[int, int]
    spread_eigenvalues_m2: tuple[float, float]
    spread_condition: float
    converged: bool


@dataclass(frozen=True)
class CompactResult:
    reason: str
    proposed_delta_xy: tuple[float, float] | None
    shadow_eligible: bool
    ineligible_reasons: tuple[str, ...]
    previous: CompactScan
    current: CompactScan
    previous_features: FeatureSet | None = None
    current_features: FeatureSet | None = None
    modes: tuple[CompactMode, ...] = ()
    hypothesis_count: int = 0
    ambiguous: bool = False
    evidence_provenance: str | None = None
    execution_eligible: bool = field(default=False, init=False)
    runtime_authority: bool = field(default=False, init=False)


def _group_indices(points,valid,gap,full_circle):
    indices=np.flatnonzero(valid)
    if not len(indices):return []
    differences=points[indices[1:]]-points[indices[:-1]]
    distances=np.linalg.norm(differences,axis=1)
    consecutive=np.diff(indices)==1
    # Vector reductions and the original scalar dot-product norm can round
    # differently right at <=gap. Retain that original decision near the
    # boundary; ordinary adjacency is computed in one bounded array operation.
    tolerance=16*np.finfo(float).eps*np.maximum(1.,np.maximum(distances,abs(gap)))
    near=consecutive&(np.abs(distances-gap)<=tolerance)
    for index in np.flatnonzero(near):
        distances[index]=np.linalg.norm(differences[index])
    splits=np.flatnonzero(~consecutive|(distances>gap))+1
    groups=[part.tolist() for part in np.split(indices,splits)]
    if (full_circle and len(groups)>1 and groups[0][0]==0 and groups[-1][-1]==len(points)-1
            and np.linalg.norm(points[0]-points[-1])<=gap):
        groups[0]=groups[-1]+groups[0]
        groups.pop()
    return groups


def extract_features(scan, config=CompactConfig()):
    """Visit every raw group (<=3200); reject >128 eligible, never truncate.

    All groups and beam indices survive in diagnostics or the original scan;
    excluded large surfaces and missing beams must not disappear from collision
    processing merely because they cannot support these compact correspondences.
    """
    if not isinstance(scan, CompactScan) or not isinstance(config, CompactConfig):
        raise ValueError('expected_scan_and_config')
    ranges=np.asarray([math.nan if v is None else v for v in scan.ranges],dtype=float)
    valid=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<scan.range_max)
    angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
    safe=np.where(valid,ranges,0.)
    points=np.column_stack((np.cos(angles)*safe,np.sin(angles)*safe))
    full_circle=abs(len(ranges)*scan.angle_increment-2*math.pi)<=scan.angle_increment*1.01
    groups=_group_indices(points,valid,config.cluster_gap_m,full_circle)
    features=[]; tiny=wide=0
    for indices in groups:
        if len(indices)<config.min_cluster_points:
            tiny+=1
            continue
        selected=points[indices]
        width=float(np.linalg.norm(np.ptp(selected,axis=0)))
        if width>config.max_cluster_width_m:
            wide+=1
            continue
        features.append(CompactFeature(tuple(map(float,selected.mean(axis=0))),width,tuple(indices)))
    reason='eligible_feature_capacity_exceeded' if len(features)>MAX_FEATURES else 'features_extracted'
    return FeatureSet(tuple(features),len(groups),int(valid.sum()),tiny,wide,reason)


def _mutual(previous,current,config):
    square=np.sum((previous[:,None,:]-current[None,:,:])**2,axis=2)
    pi=square.argmin(axis=0); ci=square.argmin(axis=1)
    indices=np.arange(len(current))
    keep=(ci[pi]==indices)&(square[pi,indices]<=config.association_distance_m**2)
    return pi[keep],indices[keep]


def _fit_features(previous,current,yaw,config):
    if max(len(previous),len(current))>MAX_FEATURES:
        return (),0,'eligible_feature_capacity_exceeded'
    if not previous or not current:
        return (),0,'no_compact_features'
    p=np.array([f.center_xy for f in previous]); c=np.array([f.center_xy for f in current])
    co,si=math.cos(yaw),math.sin(yaw)
    rotated=c@np.array(((co,si),(-si,co)))
    seeds=(p[:,None,:]-rotated[None,:,:]).reshape(-1,2)
    seeds=seeds[np.linalg.norm(seeds,axis=1)<=config.max_translation_m]
    if len(seeds)>config.max_hypotheses:
        return (),len(seeds),'hypothesis_capacity_exceeded'
    # All caches are local to this exact p/rotated/config pair. Keys preserve
    # floating-point bits and correspondence order; no quantization or skipped
    # seeds/iterations is allowed. Once full, new keys use the original work.
    mutual_cache={}; update_cache={}; mode_cache={}
    def mutual_at(delta):
        key=delta.tobytes()
        cached=mutual_cache.get(key)
        if cached is not None:
            return cached
        result=_mutual(p,rotated+delta,config)
        if len(mutual_cache)<_FIT_CACHE_CAPACITY:
            mutual_cache[key]=result
        return result
    def median_update(pi,ci):
        key=(pi.tobytes(),ci.tobytes())
        cached=update_cache.get(key)
        if cached is not None:
            return cached
        result=np.median(p[pi]-rotated[ci],axis=0)
        if len(update_cache)<_FIT_CACHE_CAPACITY:
            update_cache[key]=result
        return result
    modes=[]
    for seed in seeds:
        delta=seed.copy(); converged=False
        for _ in range(config.max_iterations):
            pi,ci=mutual_at(delta)
            if not len(pi):
                break
            update=median_update(pi,ci)
            if np.linalg.norm(update-delta)<=config.convergence_m:
                delta=update; converged=True
                break
            delta=update
        if np.linalg.norm(delta)>config.max_translation_m:
            continue
        mode_key=(delta.tobytes(),converged)
        mode=mode_cache.get(mode_key)
        if mode is None:
            pi,ci=mutual_at(delta)
            if not len(pi):
                continue
            residual=p[pi]-rotated[ci]-delta
            centered=p[pi]-p[pi].mean(axis=0)
            eigen=np.linalg.eigvalsh(centered.T@centered/len(pi))
            condition=float(eigen[-1]/eigen[0]) if eigen[0]>0 else math.inf
            mode=CompactMode(tuple(map(float,delta)),tuple((int(a),int(b)) for a,b in zip(pi,ci)),
                len(pi)/max(len(p),len(c)),float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))),
                (sum(len(previous[a].beam_indices) for a in pi),sum(len(current[b].beam_indices) for b in ci)),
                tuple(map(float,eigen)),condition,converged)
            if len(mode_cache)<_FIT_CACHE_CAPACITY:
                mode_cache[mode_key]=mode
        duplicate=next((i for i,m in enumerate(modes)
                        if np.linalg.norm(np.array(m.delta_xy)-delta)<=config.convergence_m),None)
        if duplicate is None:
            modes.append(mode)
        elif not modes[duplicate].converged and converged:
            modes[duplicate]=mode
    modes.sort(key=lambda m:(-len(m.matches),m.rmse_m,*m.delta_xy))
    return tuple(modes),len(seeds),'proposal_only' if modes else 'no_bounded_correspondence'


def _evidence_reasons(evidence,previous,current,best,config):
    if not isinstance(evidence,ShadowEvidence):
        return ['missing_independent_evidence']
    if (not all(_label(v) for v in (evidence.profile_id,evidence.evidence_ref,evidence.epoch,evidence.frame_id))
            or evidence.provenance not in ('known_flat_fixture','field_verified_assumptions')
            or not _integer(evidence.previous_stamp_ns) or not _integer(evidence.stamp_ns)):
        return ['invalid_evidence_identity']
    if ((evidence.epoch,evidence.frame_id,evidence.previous_stamp_ns,evidence.stamp_ns)
            != (previous.epoch,previous.frame_id,previous.stamp_ns,current.stamp_ns)):
        return ['evidence_pair_context_mismatch']
    reasons=[]
    for name in ('instantaneous_verified','calibrated_mount_verified','imu_yaw_verified','planar_verified',
                 'height_change_excluded','independent_static_majority_verified'):
        if getattr(evidence,name) is not True:
            reasons.append('unverified_'+name)
    pairs=evidence.temporal_pairs
    if (not isinstance(pairs,tuple) or len(pairs)>MAX_FEATURES
            or any(not isinstance(pair,tuple) or len(pair)!=2
                   or not all(_integer(index,0,MAX_FEATURES-1) for index in pair) for pair in pairs)
            or len(set(pairs))!=len(pairs)):
        reasons.append('invalid_temporal_support_pairs')
    elif evidence.provenance=='field_verified_assumptions':
        if not _label(evidence.temporal_evidence_ref):
            reasons.append('missing_independent_temporal_evidence')
        if len(set(pairs)&set(best.matches))<config.min_associations:
            reasons.append('insufficient_independent_temporal_associations')
    return reasons


def propose_translation(previous,current,delta_yaw_rad,*,evidence=None,config=CompactConfig()):
    """Preserve raw scans, propose XY, and separately qualify an offline shadow.

    Convention: previous_xy = R(measured_yaw_delta) @ current_xy + translation.
    Input IMU yaw quality/freshness and extrinsics require independent validation.
    No source/receipt clocks are fabricated here. A future consumer must implement
    freshness, continuity, stop/rearm and collision contracts before any actuation.
    """
    if not isinstance(previous,CompactScan) or not isinstance(current,CompactScan) or not isinstance(config,CompactConfig):
        raise ValueError('expected_scan_and_config')
    def invalid(reason):
        return CompactResult(reason,None,False,(reason,),previous,current)
    if previous.epoch!=current.epoch or previous.frame_id!=current.frame_id:
        return invalid('scan_context_mismatch')
    if not 0<current.stamp_ns-previous.stamp_ns<=MAX_PAIR_INTERVAL_NS:
        return invalid('invalid_scan_pair_interval')
    if not _finite(delta_yaw_rad) or abs(delta_yaw_rad)>math.pi:
        return invalid('invalid_imu_yaw')
    before,after=extract_features(previous,config),extract_features(current,config)
    modes,count,reason=_fit_features(before.features,after.features,delta_yaw_rad,config)
    if not modes:
        return CompactResult(reason,None,False,(reason,),previous,current,before,after,hypothesis_count=count)
    best=modes[0]
    reasons=[]
    if not best.converged:reasons.append('not_converged')
    if len(best.matches)<config.min_associations:reasons.append('insufficient_associations')
    if min(best.matched_returns)<config.min_matched_returns:reasons.append('insufficient_matched_returns')
    if best.overlap<config.min_overlap:reasons.append('insufficient_feature_overlap')
    if best.rmse_m>config.max_rmse_m:reasons.append('excessive_full_residual')
    if best.spread_eigenvalues_m2[0]<config.min_spread_eigenvalue_m2 or best.spread_condition>config.max_spread_condition:
        reasons.append('insufficient_two_dimensional_spread')
    ambiguous=any(len(mode.matches)>=len(best.matches)-config.ambiguity_support_margin
                  and mode.rmse_m<=config.max_rmse_m
                  and np.linalg.norm(np.array(mode.delta_xy)-best.delta_xy)>=config.ambiguity_distance_m
                  for mode in modes[1:])
    if ambiguous:reasons.append('ambiguous_translation')
    if previous.time_increment!=0 or current.time_increment!=0:reasons.append('rolling_scan_unsupported')
    reasons.extend(_evidence_reasons(evidence,previous,current,best,config))
    return CompactResult('shadow_eligible' if not reasons else 'proposal_ineligible',best.delta_xy,
        not reasons,tuple(reasons),previous,current,before,after,modes,count,ambiguous,
        evidence.provenance if isinstance(evidence,ShadowEvidence) else None)
