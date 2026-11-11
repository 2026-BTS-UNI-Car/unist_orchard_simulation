"""Portable raw-scan and independent feature-level negative controls."""
from dataclasses import replace
import math
import unittest
import warnings

import numpy as np

try:
    from .compact_registration import (CompactConfig,CompactFeature,CompactScan,ShadowEvidence,
                                       extract_features,propose_translation,_fit_features,_group_indices)
except ImportError:
    from compact_registration import (CompactConfig,CompactFeature,CompactScan,ShadowEvidence,
                                      extract_features,propose_translation,_fit_features,_group_indices)


CENTERS=np.array(((-4.2,-2.),(-3.1,2.1),(-1.4,-2.2),(.9,1.8),(2.7,-1.9),(4.5,2.3),
                  (-4.1,4.2),(-2.8,-4.4),(-.8,4.1),(1.3,-4.2),(3.1,4.5),(4.8,-4.3)))


def scan(centers=CENTERS,stamp=1_000_000_001,*,ranges=None,**changes):
    increment=2*math.pi/3200
    if ranges is None:
        ranges=[math.inf]*3200
        for x,y in centers:
            index=round((math.atan2(y,x)+math.pi)/increment)%3200
            for offset in range(-2,3):
                target=(index+offset)%3200
                # Nearest surface wins overlapping angular samples, as in a
                # scan; an occluded feature is not silently made visible.
                ranges[target]=min(ranges[target],math.hypot(x,y))
    value=CompactScan('epoch-a','laser',stamp,tuple(ranges),-math.pi,increment,.05,30.,0.)
    return replace(value,**changes)


def moved(points=CENTERS,delta=(.07,-.03),yaw=.04):
    # Independent world-to-current frame construction; true pose only in tests.
    c,s=math.cos(yaw),math.sin(yaw)
    return (np.asarray(points)-delta)@np.array(((c,-s),(s,c)))


def evidence(previous,current,**changes):
    value=ShadowEvidence('flat-fixture-v1','known_flat_fixture','independent-fixture-hash',
        previous.epoch,previous.frame_id,previous.stamp_ns,current.stamp_ns,
        True,True,True,True,True,True)
    return replace(value,**changes)


def features(points):
    return tuple(CompactFeature(tuple(p),.2,tuple(range(5*i,5*i+5))) for i,p in enumerate(points))


def feature_fit(previous,current,yaw=.04):
    return _fit_features(features(previous),features(current),yaw,CompactConfig())


def scalar_groups(points,valid,gap,full_circle):
    """Independent retained scalar adjacency definition for differential tests."""
    groups=[]
    for index in np.flatnonzero(valid):
        if (groups and index==groups[-1][-1]+1
                and np.linalg.norm(points[index]-points[groups[-1][-1]])<=gap):
            groups[-1].append(int(index))
        else:groups.append([int(index)])
    if (full_circle and len(groups)>1 and groups[0][0]==0 and groups[-1][-1]==len(points)-1
            and np.linalg.norm(points[0]-points[-1])<=gap):
        groups[0]=groups[-1]+groups[0];groups.pop()
    return groups


class AdjacencyParityTests(unittest.TestCase):
    def test_vector_groups_match_scalar_for_bounded_nonuniform_scans_and_seam(self):
        rng=np.random.default_rng(92471)
        for count in (24,128,3200):
            for full_circle in (False,True):
                for _ in range(5):
                    angles=np.linspace(-math.pi,math.pi,count,endpoint=False)
                    ranges=rng.uniform(.05,30.,count)
                    # Include a long smooth arc and multiple missing-return cuts.
                    ranges[count//4:count//2]=3.
                    points=np.column_stack((np.cos(angles)*ranges,np.sin(angles)*ranges))
                    valid=rng.random(count)>.15
                    valid[0]=valid[-1]=True;points[-1]=points[0]+(.1,0.)
                    self.assertEqual(_group_indices(points,valid,.25,full_circle),
                                     scalar_groups(points,valid,.25,full_circle))

    def test_threshold_and_adjacent_float_values_keep_scalar_inclusive_decision(self):
        points=[];valid=[]
        for angle in np.linspace(-math.pi,math.pi,97):
            for radius in (np.nextafter(.25,0.),.25,np.nextafter(.25,math.inf)):
                points.extend(((0.,0.),(math.cos(angle)*radius,math.sin(angle)*radius),(99.,99.)))
                valid.extend((True,True,False))
        points=np.array(points);valid=np.array(valid)
        self.assertEqual(_group_indices(points,valid,.25,False),scalar_groups(points,valid,.25,False))
        self.assertEqual(_group_indices(np.array(((0.,0.),(.25,0.))),np.ones(2,dtype=bool),.25,False),[[0,1]])

    def test_empty_single_and_invalid_return_splits_preserve_order(self):
        points=np.array(((0.,0.),(.1,0.),(.2,0.),(.3,0.)))
        for valid in (np.zeros(4,dtype=bool),np.array((False,True,False,False)),
                      np.array((True,False,True,True)),np.ones(4,dtype=bool)):
            for circle in (False,True):
                self.assertEqual(_group_indices(points,valid,.25,circle),scalar_groups(points,valid,.25,circle))


class CompactRegistrationTests(unittest.TestCase):
    def assert_no_execution(self,result):
        self.assertFalse(result.execution_eligible)
        self.assertFalse(result.runtime_authority)

    def test_raw_known_transform_and_independent_fixture_allow_shadow_only(self):
        previous,current=scan(),scan(moved(),stamp=1_100_000_001)
        result=propose_translation(previous,current,.04,evidence=evidence(previous,current))
        self.assertTrue(result.shadow_eligible,result.ineligible_reasons)
        np.testing.assert_allclose(result.proposed_delta_xy,(.07,-.03),atol=.008)
        self.assertEqual(result.ineligible_reasons,())
        self.assert_no_execution(result)

    def test_identity_does_not_shortcut_profile_or_quality(self):
        previous,current=scan(),scan(stamp=1_100_000_001)
        result=propose_translation(previous,current,0.)
        self.assertEqual(result.proposed_delta_xy,(0.,0.))
        self.assertFalse(result.shadow_eligible)
        self.assertIn('missing_independent_evidence',result.ineligible_reasons)
        self.assert_no_execution(result)

    def test_rolling_height_planar_calibration_and_static_majority_each_block(self):
        previous,current=scan(),scan(stamp=1_100_000_001)
        for name in ('instantaneous_verified','calibrated_mount_verified','imu_yaw_verified',
                     'planar_verified','height_change_excluded','independent_static_majority_verified'):
            for value in (False,1,'yes'):
                with self.subTest(name=name,value=value):
                    result=propose_translation(previous,current,0.,evidence=evidence(previous,current,**{name:value}))
                    self.assertFalse(result.shadow_eligible)
                    self.assertIn('unverified_'+name,result.ineligible_reasons)
        rolling=replace(current,time_increment=30e-6)
        result=propose_translation(previous,rolling,0.,evidence=evidence(previous,rolling))
        self.assertIn('rolling_scan_unsupported',result.ineligible_reasons)
        self.assertFalse(result.shadow_eligible)

    def test_evidence_must_bind_exact_integer_pair_epoch_and_frame(self):
        previous,current=scan(),scan(stamp=1_100_000_001)
        for changes in ({'epoch':'old'},{'frame_id':'other'},
                        {'previous_stamp_ns':previous.stamp_ns-1},{'stamp_ns':current.stamp_ns+1},
                        {'stamp_ns':float(current.stamp_ns)},{'stamp_ns':True},
                        {'provenance':'self_fit_residual'},{'evidence_ref':''}):
            with self.subTest(changes=changes):
                result=propose_translation(previous,current,0.,evidence=evidence(previous,current,**changes))
                self.assertFalse(result.shadow_eligible)
                self.assertEqual(result.proposed_delta_xy,(0.,0.))

    def test_field_profile_requires_independent_temporal_pair_support(self):
        previous,current=scan(),scan(stamp=1_100_000_001)
        profile=evidence(previous,current,provenance='field_verified_assumptions')
        missing=propose_translation(previous,current,0.,evidence=profile)
        self.assertFalse(missing.shadow_eligible)
        self.assertIn('missing_independent_temporal_evidence',missing.ineligible_reasons)
        actual_pairs=missing.modes[0].matches
        supported=replace(profile,temporal_evidence_ref='external-track-evidence',temporal_pairs=actual_pairs)
        result=propose_translation(previous,current,0.,evidence=supported)
        self.assertTrue(result.shadow_eligible,result.ineligible_reasons)
        self.assertEqual(result.evidence_provenance,'field_verified_assumptions')
        self.assert_no_execution(result)
        for pairs in (actual_pairs[:5],((127,127),),actual_pairs+(actual_pairs[0],),([0,0],)):
            rejected=propose_translation(previous,current,0.,evidence=replace(supported,temporal_pairs=pairs))
            self.assertFalse(rejected.shadow_eligible)

    def test_no_clock_repair_for_repeated_old_cross_epoch_or_cross_frame_inputs(self):
        previous=scan()
        for current in (scan(stamp=previous.stamp_ns),scan(stamp=previous.stamp_ns-1),
                        scan(stamp=previous.stamp_ns+150_000_001),
                        scan(stamp=previous.stamp_ns+100_000_000,epoch='new'),
                        scan(stamp=previous.stamp_ns+100_000_000,frame_id='other')):
            result=propose_translation(previous,current,0.)
            self.assertIsNone(result.proposed_delta_xy)
            self.assertFalse(result.shadow_eligible)
            self.assertIs(result.previous,previous)
            self.assertIs(result.current,current)

    def test_mixed_large_integer_clocks_preserve_interval_boundary_to_one_ns(self):
        start=1_800_000_000_123_456_789
        for previous_type in (int,np.uint64,np.int64):
            for current_type in (int,np.uint64,np.int64):
                for interval in (149_999_999,150_000_000,150_000_001):
                    with self.subTest(previous_type=previous_type,current_type=current_type,interval=interval):
                        previous=scan(stamp=previous_type(start))
                        current=scan(stamp=current_type(start+interval))
                        self.assertIs(type(previous.stamp_ns),int)
                        self.assertIs(type(current.stamp_ns),int)
                        self.assertEqual(current.stamp_ns-previous.stamp_ns,interval)
                        result=propose_translation(previous,current,0.)
                        self.assertEqual(result.reason=='invalid_scan_pair_interval',interval>150_000_000)
                        if interval<=150_000_000:
                            self.assertEqual(result.proposed_delta_xy,(0.,0.))

    def test_mixed_large_reversed_clocks_do_not_overflow_or_wrap(self):
        start=2**63-151_000_000
        with warnings.catch_warnings():
            warnings.simplefilter('error',RuntimeWarning)
            for previous_type in (int,np.uint64,np.int64):
                for current_type in (int,np.uint64,np.int64):
                    for interval in (-150_000_001,-150_000_000,-149_999_999,-1,0):
                        previous=scan(stamp=previous_type(start))
                        current=scan(stamp=current_type(start+interval))
                        result=propose_translation(previous,current,0.)
                        self.assertEqual(result.reason,'invalid_scan_pair_interval')
                        self.assertIsNone(result.proposed_delta_xy)
        with self.assertRaises(ValueError):scan(stamp=np.uint64(2**63))

    def test_large_mixed_evidence_clocks_cannot_round_away_one_ns_mismatch(self):
        start=1_800_000_000_123_456_789
        previous,current=scan(stamp=np.uint64(start)),scan(stamp=np.int64(start+100_000_000))
        exact=evidence(previous,current,previous_stamp_ns=np.uint64(start),stamp_ns=np.int64(current.stamp_ns))
        self.assertIs(type(exact.previous_stamp_ns),int)
        self.assertIs(type(exact.stamp_ns),int)
        self.assertTrue(propose_translation(previous,current,0.,evidence=exact).shadow_eligible)
        for changes in ({'previous_stamp_ns':np.uint64(start+1)},
                        {'stamp_ns':np.uint64(current.stamp_ns-1)},
                        {'stamp_ns':np.uint64(2**63)}):
            result=propose_translation(previous,current,0.,evidence=replace(exact,**changes))
            self.assertFalse(result.shadow_eligible)
            self.assertTrue(any('evidence' in reason for reason in result.ineligible_reasons))

    def test_all_raw_returns_and_unknown_values_remain_available_for_collision(self):
        raw=list(scan().ranges)
        raw[2:7]=[None,math.nan,54.8,.01,30.]
        # A long visible surface is excluded from compact support, not erased.
        raw[1000:1300]=[10.]*300
        previous=scan(ranges=raw); current=replace(previous,stamp_ns=previous.stamp_ns+100_000_000)
        result=propose_translation(previous,current,0.)
        self.assertIs(result.previous.ranges,previous.ranges)
        self.assertEqual(len(result.previous.ranges),3200)
        self.assertIsNone(result.previous.ranges[2])
        self.assertTrue(math.isnan(result.previous.ranges[3]))
        self.assertEqual(result.previous.ranges[4:7],(54.8,.01,30.))
        self.assertEqual(result.previous.ranges[1100],10.)
        self.assertGreater(result.previous_features.wide_group_count,0)

    def test_more_than128_raw_groups_are_scanned_without_dropping_tiny_groups(self):
        raw=[math.inf]*3200
        for i in range(150):raw[4*i]=5.
        for start in (900,1100,1300,1700,1900,2100):raw[start:start+5]=[5.]*5
        extracted=extract_features(scan(ranges=raw))
        self.assertEqual(extracted.raw_group_count,156)
        self.assertEqual(extracted.tiny_group_count,150)
        self.assertEqual(len(extracted.features),6)
        self.assertEqual(extracted.reason,'features_extracted')
        self.assertEqual(extracted.finite_return_count,180)

    def test_129_eligible_features_are_explicitly_rejected_without_truncation(self):
        raw=[math.inf]*3200
        for i in range(129):raw[6*i:6*i+3]=[5.]*3
        previous=scan(ranges=raw);current=replace(previous,stamp_ns=previous.stamp_ns+100_000_000)
        result=propose_translation(previous,current,0.,evidence=evidence(previous,current))
        self.assertEqual(result.reason,'eligible_feature_capacity_exceeded')
        self.assertEqual(len(result.previous_features.features),129)
        self.assertIsNone(result.proposed_delta_xy)
        self.assertEqual(result.previous.ranges,tuple(raw))

    def test_hypothesis_bound_rejects_instead_of_sampling_convenient_modes(self):
        previous,current=scan(),scan(stamp=1_100_000_001)
        result=propose_translation(previous,current,0.,config=CompactConfig(max_hypotheses=1))
        self.assertEqual(result.reason,'hypothesis_capacity_exceeded')
        self.assertGreater(result.hypothesis_count,1)
        self.assertEqual(result.modes,())
        self.assertIsNone(result.proposed_delta_xy)

    def test_seam_cluster_retains_original_angular_indices(self):
        raw=[math.inf]*3200
        for index in (3198,3199,0,1,2):raw[index]=2.
        feature=extract_features(scan(ranges=raw)).features[0]
        self.assertEqual(feature.beam_indices,(3198,3199,0,1,2))

    def test_invalid_numeric_and_work_bounds_are_rejected(self):
        for changes in ({'stamp_ns':True},{'stamp_ns':1.},{'ranges':tuple([1.]*3201)},
                        {'ranges':tuple([True]*3200)},{'ranges':tuple([10**1000]*3200)},
                        {'angle_increment':0.},{'range_max':math.inf},{'time_increment':-.01}):
            with self.subTest(changes=str(changes)[:50]),self.assertRaises(ValueError):
                replace(scan(),**changes)
        for changes in ({'max_hypotheses':513},{'max_iterations':16},{'max_translation_m':.7},
                        {'max_rmse_m':.06},{'min_associations':1},{'min_overlap':.1}):
            with self.subTest(changes=changes),self.assertRaises(ValueError):CompactConfig(**changes)
        for yaw in (True,math.nan,math.inf,4.):
            result=propose_translation(scan(),scan(stamp=1_100_000_001),yaw)
            self.assertEqual(result.reason,'invalid_imu_yaw')


class IndependentNegativeControlTests(unittest.TestCase):
    """Labels below are scoring descriptions only, never feature-fit inputs."""
    def test_static_branches_with_occlusion_still_offer_localization_features(self):
        indices=[0,1,3,4,6,7,9,10]
        modes,_,reason=feature_fit(CENTERS,moved()[indices])
        self.assertEqual(reason,'proposal_only')
        np.testing.assert_allclose(modes[0].delta_xy,(.07,-.03),atol=1e-12)
        self.assertEqual(len(modes[0].matches),8)

    def test_minority_ground_fragments_cannot_pull_coordinate_median(self):
        current=moved()
        c,s=math.cos(.04),math.sin(.04)
        current[:3]+=np.array((.20,.05))@np.array(((c,-s),(s,c)))
        modes,_,_=feature_fit(CENTERS,current)
        np.testing.assert_allclose(modes[0].delta_xy,(.07,-.03),atol=1e-12)
        self.assertGreater(modes[0].rmse_m,.05)  # Full residual is not robustly hidden.
        previous,now=scan(),scan(current,stamp=1_100_000_001)
        result=propose_translation(previous,now,.04,evidence=evidence(previous,now))
        self.assertFalse(result.shadow_eligible)
        self.assertIn('excessive_full_residual',result.ineligible_reasons)

    def test_majority_ground_and_changed_branch_view_can_fit_wrong_with_low_residual(self):
        for description in ('ground_fragment_majority','changed_branch_view_majority'):
            current=moved()
            c,s=math.cos(.04),math.sin(.04)
            current[:10]+=np.array((.12,0.))@np.array(((c,-s),(s,c)))
            modes,_,_=feature_fit(CENTERS,current)
            with self.subTest(description=description):
                self.assertGreater(np.linalg.norm(np.array(modes[0].delta_xy)-(.07,-.03)),.1)
                self.assertLess(modes[0].rmse_m,.05)
                previous,now=scan(),scan(current,stamp=1_100_000_001)
                result=propose_translation(previous,now,.04)
                self.assertIsNotNone(result.proposed_delta_xy)
                self.assertFalse(result.shadow_eligible)
                self.assertIn('missing_independent_evidence',result.ineligible_reasons)
                self.assertFalse(result.execution_eligible)

    def test_periodic_identity_and_partial_occlusion_retain_aliases(self):
        periodic=np.column_stack((np.arange(14)*.3-1.8,np.ones(14)*2.))
        for points in (periodic,periodic[2:]):
            previous,current=scan(periodic),scan(points,stamp=1_100_000_001)
            result=propose_translation(previous,current,0.,evidence=evidence(previous,current))
            self.assertTrue(result.ambiguous,result.ineligible_reasons)
            self.assertFalse(result.shadow_eligible)
            self.assertIn('ambiguous_translation',result.ineligible_reasons)
            self.assertGreater(len(result.modes),1)

    def test_single_unrelated_replacement_perfect_fit_is_ineligible(self):
        previous=scan(np.array(((1.,1.),)))
        current=scan(np.array(((.9,1.),)),stamp=1_100_000_001)
        result=propose_translation(previous,current,0.,evidence=evidence(previous,current))
        self.assertGreater(np.linalg.norm(result.proposed_delta_xy),.09)
        self.assertLess(result.modes[0].rmse_m,1e-12)
        self.assertIn('insufficient_associations',result.ineligible_reasons)
        self.assertFalse(result.shadow_eligible)

    def test_total_occlusion_is_missing_motion_not_zero_velocity(self):
        result=propose_translation(scan(),scan(np.empty((0,2)),stamp=1_100_000_001),.04)
        self.assertEqual(result.reason,'no_compact_features')
        self.assertIsNone(result.proposed_delta_xy)
        self.assertFalse(result.shadow_eligible)

    def test_low_overlap_and_collinear_support_are_separate_geometric_rejections(self):
        previous,current=scan(),scan(CENTERS[:3],stamp=1_100_000_001)
        result=propose_translation(previous,current,0.,evidence=evidence(previous,current))
        self.assertIn('insufficient_feature_overlap',result.ineligible_reasons)
        self.assertFalse(result.shadow_eligible)
        # Irregular spacing removes the periodic alias; collinearity alone still
        # cannot meet this conservative two-dimensional support contract.
        points=np.column_stack(([-3.4,-2.1,-.5,1.4,3.2,5.5],np.ones(6)*2.))
        previous,current=scan(points),scan(points,stamp=1_100_000_001)
        result=propose_translation(previous,current,0.,evidence=evidence(previous,current))
        self.assertIn('insufficient_two_dimensional_spread',result.ineligible_reasons)
        self.assertFalse(result.shadow_eligible)


if __name__=='__main__':unittest.main()
