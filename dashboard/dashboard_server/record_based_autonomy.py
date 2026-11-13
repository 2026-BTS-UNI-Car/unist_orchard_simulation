"""Native-sensor row-following proposals for record replay / observation only.

This module owns no IO, clocks, threads, leases or devices. Percentage response,
laser-forward convention, footprint, timing and braking margins are conservative
estimates, not calibration. No proposal can arm or apply a motor command. Original
scan and causal IMU timestamps must be supplied; caller flags are not certificates.
"""
from dataclasses import dataclass
import math
import threading


SCHEMA = 'ubts.record_based_autonomy_proposal.v1'
MAX_NS = 2**63 - 1
MAX_BEAMS = 4096
CONTEXT_KEYS = ('source_session', 'generation', 'source_clock_domain', 'receipt_clock_domain')


def _finite(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _ns(value):
    return type(value) is int and 0 <= value <= MAX_NS


def _name(value):
    return type(value) is str and 0 < len(value) <= 128


def _context(value):
    if type(value) is not dict or not all(key in value for key in CONTEXT_KEYS):
        return None
    if not _ns(value['generation']) or not all(_name(value[key]) for key in CONTEXT_KEYS if key != 'generation'):
        return None
    return tuple(value[key] for key in CONTEXT_KEYS)


def _clip(value, low, high):
    return min(high, max(low, value))


@dataclass(frozen=True)
class ObservationEstimates:
    """Explicit unverified estimates, never usable as a physical brake profile.

    The record audits establish unequal response / vibration and coarse settling;
    they do not identify a safe gain, deadzone or speed. These defaults are small
    bounded proposal settings rather than fits to the October records.
    """
    body_length_m: float = 1.22
    body_width_m: float = .70
    footprint_margin_m: float = .20
    front_lookahead_m: float = 1.0
    cruise_percent: float = 15.0
    max_percent: float = 30.0
    linear_ff_percent_per_m_s: float = 100.0
    linear_p_percent_per_m_s: float = 12.0
    yaw_ff_percent_per_rad_s: float = 8.0
    yaw_p_percent_per_rad_s: float = 6.0
    yaw_damping_percent_per_rad_s: float = 2.0
    center_gain_rad_s_per_m: float = .35
    heading_gain_s_inverse: float = .7
    max_yaw_rad_s: float = .25
    slew_percent_per_s: float = 20.0
    source_age_ns: int = 250_000_000
    receipt_age_ns: int = 200_000_000
    imu_age_ns: int = 150_000_000
    max_pair_interval_ns: int = 500_000_000
    clock_jump_ns: int = 50_000_000
    support_scans: int = 3
    support_duration_ns: int = 150_000_000

    def valid(self):
        positive = (self.body_length_m, self.body_width_m, self.footprint_margin_m,
            self.front_lookahead_m, self.max_percent, self.linear_ff_percent_per_m_s,
            self.center_gain_rad_s_per_m, self.heading_gain_s_inverse,
            self.max_yaw_rad_s, self.slew_percent_per_s)
        nonnegative = (self.linear_p_percent_per_m_s, self.yaw_ff_percent_per_rad_s,
            self.yaw_p_percent_per_rad_s, self.yaw_damping_percent_per_rad_s)
        return (all(_finite(v) and v > 0 for v in positive) and
            all(_finite(v) and v >= 0 for v in nonnegative) and
            _finite(self.cruise_percent) and 0 <= self.cruise_percent <= self.max_percent <= 30 and
            all(_ns(v) and v > 0 for v in (self.source_age_ns, self.receipt_age_ns,
                self.imu_age_ns, self.max_pair_interval_ns, self.clock_jump_ns, self.support_duration_ns)) and
            type(self.support_scans) is int and 3 <= self.support_scans <= 8)


def _fit_side(points):
    """Bounded deterministic consensus then orthogonal TLS; no learned row prior.

    Each side needs >=8 inliers spanning >=2m and >=65% of its candidate returns.
    Isolated branch points are never removed from the separate front obstacle test.
    """
    if len(points) < 8:
        return None
    # At most 16 anchors / 120 candidate lines, independent of raw beam count.
    ordered = sorted(points)
    anchors = [ordered[(i*(len(ordered)-1))//15] for i in range(16)]
    best = None
    for i, p in enumerate(anchors):
        for q in anchors[i+1:]:
            if q[0]-p[0] < .75:
                continue
            slope = (q[1]-p[1])/(q[0]-p[0])
            if abs(math.atan(slope)) > .35:
                continue
            intercept = p[1]-slope*p[0]
            norm = math.hypot(slope, 1.)
            inliers = [v for v in ordered if abs(v[1]-slope*v[0]-intercept)/norm <= .12]
            if len(inliers) < 8:
                continue
            span = inliers[-1][0]-inliers[0][0]
            score = (len(inliers), span)
            if best is None or score > best[0]:
                best = (score, inliers)
    if best is None or best[0][1] < 2.0 or best[0][0]/len(ordered) < .65:
        return None
    inliers = best[1]
    mx = sum(p[0] for p in inliers)/len(inliers)
    my = sum(p[1] for p in inliers)/len(inliers)
    xx = sum((p[0]-mx)**2 for p in inliers)
    yy = sum((p[1]-my)**2 for p in inliers)
    xy = sum((p[0]-mx)*(p[1]-my) for p in inliers)
    angle = .5*math.atan2(2*xy, xx-yy)
    if abs(angle) > .35:
        return None
    slope = math.tan(angle)
    intercept = my-slope*mx
    residuals = [abs(p[1]-slope*p[0]-intercept)/math.hypot(slope, 1.) for p in inliers]
    if max(residuals) > .18:
        return None
    return dict(slope=slope, intercept=intercept, heading_rad=angle,
        support=len(inliers), candidate_returns=len(ordered), span_m=best[0][1],
        residual_rms_m=math.sqrt(sum(v*v for v in residuals)/len(residuals)))


class RecordBasedAutonomy:
    """Stateful pure controller: explicit reset, zero first, persistent two sides.

    Source discontinuities / reversal / Stop latch zero until explicit reset.
    A temporary geometric dropout holds zero and requires persistence again.
    A front return inside the estimated swept rectangle latches STOP. There is no
    end-turn planner: vanished sides / unknown exit cannot authorize a blind turn.
    The input frame is the original laser frame; +X is only an estimated forward
    convention. A valid proposal is not independently validated body feedback.
    """
    def __init__(self, estimates=None, *, allow_record_replay_estimates=False):
        self.estimates = estimates or ObservationEstimates()
        if type(self.estimates) is not ObservationEstimates or not self.estimates.valid():
            raise ValueError('bounded_observation_estimates_required')
        if type(allow_record_replay_estimates) is not bool:
            raise ValueError('explicit_record_replay_estimates_bool_required')
        self.allow_record_replay_estimates = allow_record_replay_estimates
        self._replay_active = False
        self._source_unverified = True
        self._context = None
        self._lock = threading.RLock()
        self._stopping = threading.Event()
        self._fault = 'reset_required'
        self._last = None
        self._first = True
        self._output = (0., 0.)
        self._support_count = 0
        self._support_start = None
        self._row = None

    def _answer(self, decision, reason, *, proposed=(0., 0.), trajectory=(), geometry=None, motion=None):
        if self._stopping.is_set():
            decision, reason, proposed, trajectory = 'STOP', self._fault or 'stop_requested', (0., 0.), ()
        return dict(schema=SCHEMA, mode='OBSERVE_ONLY', decision=decision, reason=str(reason)[:256],
            record_replay_estimate=self._replay_active,
            source_provenance_unverified=self._source_unverified,
            proposed_left_percent=proposed[0], proposed_right_percent=proposed[1],
            applied_left_percent=0., applied_right_percent=0., trajectory=list(trajectory),
            row_geometry=geometry, laser_plane_motion=motion,
            temporal_support_scans=self._support_count,
            estimates=dict(footprint_length_m=self.estimates.body_length_m,
                footprint_width_m=self.estimates.body_width_m,
                footprint_margin_m=self.estimates.footprint_margin_m,
                calibrated=False, mount_verified=False, motor_response_verified=False,
                braking_verified=False, body_velocity_verified=False,
                timing_physically_validated=False, frame='estimated_laser_forward'),
            runtime_authority=False, motor_command_sent=False, autonomy_output_enabled=False,
            physical_autonomy_qualified=False, execution_eligible=False,
            PWM_used_as_odometry=False, wheel_speed_observed=False)

    def _hold(self, reason):
        self._output = (0., 0.)
        self._support_count = 0
        self._support_start = self._row = None
        return self._answer('HOLD', reason)

    def stop(self, reason='operator_stop'):
        # Cancellation is visible even while bounded geometric work holds lock.
        self._stopping.set()
        with self._lock:
            self._fault = str(reason)[:256]
            self._last = None
            self._first = True
            self._hold(reason)
            return self._answer('STOP', self._fault)

    def manual_takeover(self):
        return self.stop('manual_takeover')

    def reset(self, context):
        value = _context(context)
        if value is None:
            raise ValueError('explicit_native_source_context_required')
        with self._lock:
            self._context = value
            self._fault = None
            self._last = None
            self._first = True
            self._stopping.clear()
            self._replay_active = False
            self._source_unverified = True
            return self._hold('first_zero_required')

    def reanchor_after_expiry(self, context):
        """Drop skipped-pair state after caller-proven ordinary positive expiry.

        The caller must retain source/order/freshness gates and supply a fresh
        NEW scan. This is not a reset of an operator, geometric or timing STOP;
        neither the fault nor cancellation event is ever cleared here.
        """
        value = _context(context)
        with self._lock:
            if (value is None or value != self._context or self._fault is not None or
                    self._stopping.is_set()):
                return False
            self._last = None
            self._first = True
            self._output = (0., 0.)
            self._support_count = 0
            self._support_start = self._row = None
            # Stop can set its event before waiting for this lock. Never clear
            # it, even when it races with this small state-only reanchor.
            return not self._stopping.is_set() and self._fault is None

    def __call__(self, snapshot, cruise_preference_mps):
        if not _finite(cruise_preference_mps) or cruise_preference_mps < 0:
            return self.stop('invalid_cruise_preference')
        # A preference is not measured vehicle speed; only the estimated FF scale
        # converts it to a proposal. Direct recorded percentages use plan().
        percent = cruise_preference_mps*self.estimates.linear_ff_percent_per_m_s
        return self.plan(snapshot, cruise_percent=percent)

    def plan(self, snapshot, cruise_percent=None):
        with self._lock:
            return self._plan(snapshot, cruise_percent)

    def _plan(self, snapshot, cruise_percent):
        e = self.estimates
        if self._fault is not None:
            return self._answer('STOP', self._fault)
        if self._stopping.is_set():
            return self._answer('STOP', 'stop_requested')
        if type(snapshot) is not dict or _context(snapshot.get('context')) != self._context:
            return self.stop('source_context_changed_or_missing')
        self._replay_active = (self.allow_record_replay_estimates and
            snapshot.get('replay_estimate') is True and
            snapshot['context']['source_clock_domain'] == 'archive_ros_header_unverified' and
            snapshot['context']['receipt_clock_domain'] == 'synthetic_offline_replay')
        scan, motion, imu = (snapshot.get(name) for name in ('scan', 'motion', 'imu'))
        if not all(type(value) is dict for value in (scan, motion, imu)):
            return self._hold('native_scan_motion_IMU_required')
        native = scan.get('original_native') is True and imu.get('original_native') is True
        self._source_unverified = self._replay_active or not native
        replay_input = (self._replay_active and scan.get('original_native') is False and
                        imu.get('original_native') is False)
        if not native and not replay_input:
            return self._hold('original_native_provenance_unverified')
        now, receipt_now = snapshot.get('source_now_ns'), snapshot.get('receipt_now_ns')
        ints = (now, receipt_now, scan.get('stamp_ns'), scan.get('receipt_ns'), scan.get('sequence'),
                motion.get('previous_scan_stamp_ns'), motion.get('current_scan_stamp_ns'),
                motion.get('receipt_ns'), imu.get('stamp_ns'), imu.get('receipt_ns'))
        if not all(_ns(v) for v in ints) or not _name(scan.get('frame_id')):
            return self.stop('typed_original_timestamps_required')
        stamp, receipt, seq = scan['stamp_ns'], scan['receipt_ns'], scan['sequence']
        if not (0 <= now-stamp <= e.source_age_ns and
                0 <= receipt_now-receipt <= e.receipt_age_ns and
                0 <= receipt_now-motion['receipt_ns'] <= e.receipt_age_ns and
                0 <= receipt_now-imu['receipt_ns'] <= e.receipt_age_ns and
                0 <= stamp-imu['stamp_ns'] <= e.imu_age_ns):
            return self.stop('source_or_receipt_stale_future_or_IMU_noncausal')
        if motion['current_scan_stamp_ns'] != stamp:
            return self.stop('native_pair_scan_endpoint_mismatch')
        pair_ns = stamp-motion['previous_scan_stamp_ns']
        if not 0 < pair_ns <= e.max_pair_interval_ns:
            return self.stop('native_pair_time_discontinuity')
        if self._last is not None:
            previous_stamp, previous_receipt, previous_seq, previous_frame, previous_now, previous_receipt_now = self._last
            if (stamp <= previous_stamp or receipt <= previous_receipt or seq <= previous_seq or
                    scan['frame_id'] != previous_frame or
                    motion['previous_scan_stamp_ns'] != previous_stamp):
                return self.stop('source_order_or_native_pair_discontinuity')
            if (now < previous_now or receipt_now < previous_receipt_now or
                    abs((now-previous_now)-(receipt_now-previous_receipt_now)) > e.clock_jump_ns):
                return self.stop('source_receipt_clock_reversal_or_jump')
        self._last = (stamp, receipt, seq, scan['frame_id'], now, receipt_now)
        if not all(_finite(motion.get(key)) for key in ('dx_m', 'dy_m', 'dyaw_rad')) or not _finite(imu.get('yaw_rate_rad_s')):
            return self.stop('finite_native_motion_required')
        if (motion.get('accepted') is not True or motion.get('map_eligible') is not True or
                motion.get('path_break') is not False or motion.get('motion_prior') is not False):
            return self._hold('accepted_geometric_native_pair_required')
        interval_s = pair_ns/1e9
        forward = motion['dx_m']/interval_s
        lateral = motion['dy_m']/interval_s
        yaw_rate = motion['dyaw_rad']/interval_s
        if abs(motion['dyaw_rad']) >= math.pi or forward < -.02:
            return self.stop('reversal_or_rotation_branch_unknown')
        if abs(forward) > 1.5 or abs(lateral) > .5 or abs(yaw_rate) > 1.5 or abs(imu['yaw_rate_rad_s']) > 1.5:
            return self._hold('laser_plane_motion_outside_observation_envelope')
        if abs(yaw_rate-imu['yaw_rate_rad_s']) > .5:
            return self._hold('native_laser_IMU_yaw_disagreement')
        preference = e.cruise_percent if cruise_percent is None else cruise_percent
        if not _finite(preference) or not 0 <= preference <= e.max_percent:
            return self.stop('bounded_forward_cruise_percent_required')
        ranges = scan.get('ranges')
        fields = (scan.get('angle_min'), scan.get('angle_increment'), scan.get('range_min'), scan.get('range_max'))
        if (type(ranges) not in (list, tuple) or not 24 <= len(ranges) <= MAX_BEAMS or
                not all(_finite(v) for v in fields) or fields[1] == 0 or
                abs(fields[0]) > 2*math.pi or abs(fields[1])*len(ranges) > 2*math.pi+.1 or
                not 0 <= fields[2] < fields[3] <= 1000):
            return self._hold('invalid_full_raw_scan')
        # A verified original-CDR adapter may preserve nonfinite range slots as
        # JSON null. Only its exact native marker (or explicit archive replay)
        # admits null as UNKNOWN, never as a free ray or recovered CDR value.
        null_unknown = (self._replay_active or
                        (scan.get('original_native') is True and
                         scan.get('nonfinite_ranges_as_unknown') is True))
        if not all(type(r) in (int, float) or (null_unknown and r is None) for r in ranges):
            return self._hold('invalid_full_raw_scan')
        left, right = [], []
        front_total = front_known = 0
        front_min = front_max = None
        half_width = e.body_width_m/2+e.footprint_margin_m
        front_x = e.body_length_m/2+e.front_lookahead_m
        angle_limit = math.atan2(half_width, front_x)
        for i, r in enumerate(ranges):
            angle = fields[0]+i*fields[1]
            angle = math.atan2(math.sin(angle), math.cos(angle))
            front = abs(angle) <= angle_limit
            front_total += int(front)
            if front:
                front_min = angle if front_min is None else min(front_min, angle)
                front_max = angle if front_max is None else max(front_max, angle)
            valid = _finite(r) and fields[2] < r < fields[3]
            if not valid:
                continue
            front_known += int(front)
            x, y = r*math.cos(angle), r*math.sin(angle)
            if -.5*e.body_length_m <= x <= front_x and abs(y) <= half_width:
                return self.stop('front_return_inside_estimated_swept_space')
            if .6 <= x <= 6.0 and half_width < abs(y) <= 3.0:
                (left if y > 0 else right).append((x, y))
        # Discrete rays need both frontal edges within one original beam step;
        # this never fills a missing/invalid ray with synthetic free evidence.
        edge_step = abs(fields[1])
        if (front_total < 3 or front_known != front_total or
                front_min > -angle_limit+edge_step or front_max < angle_limit-edge_step):
            return self._hold('front_space_unobserved')
        a, b = _fit_side(left), _fit_side(right)
        if self._stopping.is_set():
            return self._answer('STOP', self._fault or 'stop_requested')
        if a is None or b is None:
            return self._hold('row_end_or_two_sided_support_unknown')
        heading = (a['heading_rad']+b['heading_rad'])/2
        width = a['intercept']-b['intercept']
        center = (a['intercept']+b['intercept'])/2
        if (abs(a['heading_rad']-b['heading_rad']) > .12 or
                width < 2*half_width+.1 or width > 5.5 or abs(center) > width/2-half_width):
            return self._hold('nonparallel_or_narrow_row_geometry')
        row = (center, heading, width)
        if self._row is not None and (abs(center-self._row[0]) > .25 or
                abs(heading-self._row[1]) > .12 or abs(width-self._row[2]) > .4):
            return self._hold('row_geometry_not_temporally_stable')
        self._row = row
        if self._support_start is None:
            self._support_start = stamp
        self._support_count = min(8, self._support_count+1)
        if self._first:
            self._first = False
            return self._answer('HOLD', 'first_zero_required')
        if self._support_count < e.support_scans or stamp-self._support_start < e.support_duration_ns:
            return self._answer('HOLD', 'two_sided_support_persistence_pending')
        geometry = dict(center_m=center, heading_rad=heading, width_m=width,
            left_support=a['support'], right_support=b['support'],
            left_residual_rms_m=a['residual_rms_m'], right_residual_rms_m=b['residual_rms_m'])
        measured = dict(forward_m_s=forward, lateral_m_s=lateral, yaw_rad_s=yaw_rate,
            IMU_yaw_rad_s=imu['yaw_rate_rad_s'], frame='original_laser_plane', body_validated=False)
        if preference == 0:
            self._output = (0., 0.)
            return self._answer('HOLD', 'zero_cruise_preference', geometry=geometry, motion=measured)
        desired_yaw = _clip(e.center_gain_rad_s_per_m*center+e.heading_gain_s_inverse*heading,
                            -e.max_yaw_rad_s, e.max_yaw_rad_s)
        desired_speed_estimate = preference/e.linear_ff_percent_per_m_s
        base = _clip(preference+e.linear_p_percent_per_m_s*(desired_speed_estimate-forward), 0., e.max_percent)
        turn = (e.yaw_ff_percent_per_rad_s*desired_yaw+
                e.yaw_p_percent_per_rad_s*(desired_yaw-yaw_rate)-
                e.yaw_damping_percent_per_rad_s*imu['yaw_rate_rad_s'])
        # No reversing/pivot: differential corrections always remain forward.
        target = (_clip(base-turn, 0., e.max_percent), _clip(base+turn, 0., e.max_percent))
        slew = e.slew_percent_per_s*interval_s
        proposal = tuple(_clip(v, old-slew, old+slew) for v, old in zip(target, self._output))
        proposal = tuple(_clip(v, 0., e.max_percent) for v in proposal)
        self._output = proposal
        trajectory = []
        # Estimated centerline visualization only; neither a cleared swept arc
        # nor a trajectory executable on this uncalibrated physical vehicle.
        for i in range(1, 17):
            x = i*e.front_lookahead_m/16
            y = center+math.tan(heading)*x
            trajectory.append(dict(x_m=x, y_m=y, yaw_rad=heading))
        return self._answer('PROPOSE_FORWARD', 'persistent_native_two_sided_row_estimate',
            proposed=proposal, trajectory=trajectory, geometry=geometry, motion=measured)
