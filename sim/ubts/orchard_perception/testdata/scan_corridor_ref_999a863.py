"""Bounded, observation-only parallel surface bands in a LaserScan.

Unlike compact trunk candidates, this also accepts fragmented branch/wall
surfaces. A band is NOT a tree row. It grants no free-space, row-end, forward
direction or motor authority. All coordinates are in the uncalibrated laser
frame; the caller keeps the full raw scan for collision/visibility checks.
"""
from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np


@dataclass(frozen=True)
class CorridorConfig:
    max_scan_points: int = 4096
    support_range_m: float = 10.
    heading_step_deg: float = 2.
    spatial_cell_m: float = .10
    band_tolerance_m: float = .22
    min_side_offset_m: float = .5
    min_width_m: float = 1.
    max_width_m: float = 10.
    longitudinal_bin_m: float = .75
    min_occupied_bins: int = 6
    min_overlap_m: float = 3.
    max_band_peaks_per_side: int = 8
    ambiguity_heading_deg: float = 8.
    ambiguity_offset_m: float = .6
    ambiguity_score_ratio: float = .95

    def __post_init__(self):
        for name in ('max_scan_points','min_occupied_bins','max_band_peaks_per_side'):
            if type(getattr(self,name)) is not int or getattr(self,name)<=0:
                raise ValueError(name)
        if self.max_scan_points>4096 or self.max_band_peaks_per_side>8:
            raise ValueError('unbounded_work')
        for name in ('support_range_m','heading_step_deg','spatial_cell_m','band_tolerance_m',
                     'min_side_offset_m','min_width_m','max_width_m','longitudinal_bin_m',
                     'min_overlap_m','ambiguity_heading_deg','ambiguity_offset_m','ambiguity_score_ratio'):
            value=getattr(self,name)
            if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
                raise ValueError(name)
        if (self.support_range_m>30 or not 1<=self.heading_step_deg<=10 or
            self.spatial_cell_m<.05 or self.band_tolerance_m<.05 or
            self.longitudinal_bin_m<.25 or self.min_width_m>self.max_width_m or
            self.max_width_m>20 or not .5<=self.ambiguity_score_ratio<=1):
            raise ValueError('invalid_limits')


@dataclass(frozen=True)
class CorridorObservation:
    valid: bool
    reason: str
    heading_axis_laser_rad: float | None = None
    left_offset_m: float | None = None
    right_offset_m: float | None = None
    overlap_along_m: tuple[float,float] | None = None
    interior_returns: int = 0
    diagnostics: dict = field(default_factory=dict)
    semantic_tree_rows_verified: bool = False
    free_space_verified: bool = False
    forward_direction_verified: bool = False
    last_row: str = 'UNKNOWN'
    runtime_authority: bool = False

    @property
    def width_m(self):
        return self.left_offset_m-self.right_offset_m if self.valid else None


def axis_difference(a,b):
    """An unoriented line axis is periodic over pi, not two pi."""
    return abs((a-b+math.pi/2)%math.pi-math.pi/2)


def _finite_median(values):
    # All support coordinates are already finite. The general nan/axis
    # dispatch in numpy.median dominated thousands of tiny band queries.
    ordered=np.sort(values);middle=len(ordered)//2
    return float(ordered[middle] if len(ordered)%2 else (ordered[middle-1]+ordered[middle])*.5)


def _finite_span(values):
    ordered=np.sort(values);result=[]
    for fraction in (.05,.95):
        index=(len(ordered)-1)*fraction;lower=int(index);upper=min(lower+1,len(ordered)-1)
        weight=index-lower;a=float(ordered[lower]);b=float(ordered[upper]);difference=b-a
        # Same linear percentile interpolation, including the stable upper
        # branch used by NumPy. This changes dispatch cost, not support limits.
        result.append(a+difference*weight if weight<.5 else b-difference*(1-weight))
    return result


def observe_scan_corridor(scan:Any,config:CorridorConfig|None=None)->CorridorObservation:
    config=config or CorridorConfig();diagnostic={}
    def reject(reason):return CorridorObservation(False,reason,diagnostics=diagnostic)
    try:
        n=len(scan.ranges)
        if n>config.max_scan_points:return reject('scan_capacity_exceeded')
        amin,inc,rmin,rmax=[float(getattr(scan,k)) for k in ('angle_min','angle_increment','range_min','range_max')]
        if (n<3 or not all(math.isfinite(v) for v in (amin,inc,rmin,rmax)) or
            inc<=0 or abs(amin)>2*math.pi or rmin<0 or not rmin<rmax<=30 or
            n*inc>2*math.pi+inc*1.01):return reject('invalid_scan_geometry')
        ranges=np.asarray(scan.ranges,dtype=float)
        if ranges.shape!=(n,):return reject('invalid_scan_geometry')
    except (AttributeError,TypeError,ValueError,OverflowError):return reject('invalid_scan_geometry')
    valid=np.isfinite(ranges)&(ranges>=rmin)&(ranges<rmax)
    support=valid&(ranges<=config.support_range_m)
    diagnostic.update(scan_points=n,valid_returns=int(valid.sum()),support_returns=int(support.sum()),
                      outside_support_returns=int((valid&~support).sum()))
    if int(support.sum())<config.min_occupied_bins*2:return reject('insufficient_returns')
    a=amin+np.arange(n)[support]*inc;r=ranges[support]
    raw=np.column_stack((r*np.cos(a),r*np.sin(a)))
    # One support vote per spatial cell prevents repeated/compensated driver
    # beams or close dense surfaces from winning by ray count. This perception
    # view never edits the caller's scan or removes obstacles from visibility.
    _,indices=np.unique(np.floor(raw/config.spatial_cell_m).astype(np.int32),axis=0,return_index=True)
    points=raw[np.sort(indices)]
    diagnostic['spatial_support_cells']=len(points)
    bin_m=config.band_tolerance_m
    radius=int(math.ceil(config.support_range_m/bin_m))+1
    bins=radius*2+1
    candidates=[];capacity_rejections=0;heading_count=0
    for deg in np.arange(-90.,90.,config.heading_step_deg):
        heading_count+=1;theta=math.radians(float(deg))
        direction=np.array((math.cos(theta),math.sin(theta)))
        normal=np.array((-direction[1],direction[0]))
        cross=points@normal;along=points@direction
        cross_bin=np.rint(cross/bin_m).astype(int)+radius
        along_bin=np.floor(along/config.longitudinal_bin_m).astype(int)
        along_radius=int(math.ceil(config.support_range_m/config.longitudinal_bin_m))+1
        along_n=along_radius*2+1
        packed=cross_bin*along_n+(along_bin+along_radius)
        votes=np.bincount(packed,minlength=bins*along_n).reshape(bins,along_n)>0
        coverage=votes.copy()
        coverage[1:]|=votes[:-1];coverage[:-1]|=votes[1:]
        occupied=coverage.sum(axis=1)
        bands=[]
        # Local maxima of longitudinal coverage, not all point-pair line fits.
        seeds=np.flatnonzero(occupied>=config.min_occupied_bins)
        for seed in sorted(seeds,key=lambda i:(-occupied[i],abs(i-radius),i)):
            estimate=(seed-radius)*bin_m
            if abs(estimate)<config.min_side_offset_m:continue
            if any(abs(estimate-b['offset'])<=2*bin_m for b in bands):continue
            selected=np.abs(cross-estimate)<=config.band_tolerance_m
            if selected.sum()<config.min_occupied_bins:continue
            offset=_finite_median(cross[selected])
            selected=np.abs(cross-offset)<=config.band_tolerance_m
            occupied_bins=np.unique(along_bin[selected])
            if len(occupied_bins)<config.min_occupied_bins:continue
            lo,hi=_finite_span(along[selected])
            if hi-lo<config.min_overlap_m:continue
            bands.append(dict(offset=offset,lo=float(lo),hi=float(hi),count=len(occupied_bins),
                rms=float(np.sqrt(np.mean((cross[selected]-offset)**2)))))
        left=[b for b in bands if b['offset']>=config.min_side_offset_m]
        right=[b for b in bands if b['offset']<=-config.min_side_offset_m]
        if len(left)>config.max_band_peaks_per_side or len(right)>config.max_band_peaks_per_side:
            capacity_rejections+=1;continue
        for l in left:
            for rr in right:
                width=l['offset']-rr['offset']
                lo,hi=max(l['lo'],rr['lo']),min(l['hi'],rr['hi'])
                if not config.min_width_m<=width<=config.max_width_m or hi-lo<config.min_overlap_m:continue
                # An observed inner parallel band contradicts selecting an
                # outer aisle. A lone interior obstacle is reported separately.
                if any(rr['offset']+2*bin_m<b['offset']<l['offset']-2*bin_m and
                       min(hi,b['hi'])-max(lo,b['lo'])>=config.min_overlap_m for b in bands):continue
                score=min(l['count'],rr['count'])+(hi-lo)/config.longitudinal_bin_m
                candidates.append(dict(heading=theta,left=l['offset'],right=rr['offset'],lo=lo,hi=hi,
                    score=score,residual=l['rms']+rr['rms']))
    diagnostic.update(heading_hypotheses=heading_count,band_capacity_rejected_headings=capacity_rejections,
                      pair_hypotheses=len(candidates))
    if not candidates:return reject('no_parallel_surface_bands')
    candidates.sort(key=lambda c:(-c['score'],c['residual']))
    best=candidates[0];diagnostic['best_score']=best['score']
    # Headings near +/-90 are the same axis; align normals before comparing
    # offsets. Similar but materially different corridors remain ambiguous.
    for other in candidates[1:]:
        if other['score']<best['score']*config.ambiguity_score_ratio:continue
        heading=axis_difference(other['heading'],best['heading'])
        flipped=abs(other['heading']-best['heading'])>math.pi/2
        left=-other['right'] if flipped else other['left']
        right=-other['left'] if flipped else other['right']
        if (heading>math.radians(config.ambiguity_heading_deg) or
            abs(left-best['left'])>config.ambiguity_offset_m or
            abs(right-best['right'])>config.ambiguity_offset_m):return reject('ambiguous_parallel_surface_bands')
    theta=best['heading'];direction=np.array((math.cos(theta),math.sin(theta)))
    normal=np.array((-direction[1],direction[0]));cross=raw@normal;along=raw@direction
    interior=(cross>best['right']+config.band_tolerance_m)&(cross<best['left']-config.band_tolerance_m)
    # A front wall can sit just beyond the observed side-band overlap. Keeping
    # only the overlap would hide exactly that blocking return. Report ALL raw
    # ROI returns between the bands; this is still not a visibility/drive test.
    in_overlap=(along>=best['lo'])&(along<=best['hi'])
    diagnostic['interior_returns_in_overlap']=int((interior&in_overlap).sum())
    diagnostic['interior_returns_outside_overlap']=int((interior&~in_overlap).sum())
    return CorridorObservation(True,'observed_parallel_surface_bands',theta,best['left'],best['right'],
        (best['lo'],best['hi']),int(interior.sum()),diagnostic)
