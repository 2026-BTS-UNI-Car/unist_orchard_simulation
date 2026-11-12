"""Sensor-only state for encoderless row following (no truth, no PWM, no wheels).

Inputs are delayed sensor packets only: LaserScan-like ranges with their
sensor stamp, and raw-IMU-like gyro/accel samples with their stamp. The
configuration holds NOMINAL calibration values (mount lever arm, footprint,
pivot centre); the harness never copies the plant's true values into it.

* odometry: bounded keyframe point-to-point ICP (scipy cKDTree), seeded by
  the integrated gyro. This stands in for the native LiDAR scan matcher; it is
  a harness simplification, not the deployed matcher. Stillness is decided
  from keyframe-relative poses (non-accumulating noise) plus the gyro; the
  snapshot reports how long it has lasted. A failed match keeps the gyro
  rotation and never invents translation.
* corridor acquisition: an axis search restricted to +-acquire_axis_window_deg
  around the current travel direction (or around the last tracked band after a
  loss) that pairs the NEAREST parallel support band on each side and vetoes a
  pair with any intervening band (gapped rows), so cross-row trunk columns and
  rows across a gap are not chosen. Ambiguous best axes are refused.
* corridor tracking: orchard_perception.scan_corridor.observe_scan_corridor on
  every second scan (5 Hz), refined by a shared-slope least-squares line fit
  and tracked as ONE world-fixed band in the odometry frame. When the detector
  is ambiguous or disagrees, returns near the predicted lines are fitted
  instead (gated tracking). One-sided updates keep the learnt width; the
  learnt width also survives a band loss.
* row end: per-side forward support extents excluding any structure that spans
  the aisle core (a wall/fence across both line extensions is not row
  support). A side has ENDED only if observed free space of at least
  end_free_m (and more than the largest in-row gap when a structure closes the
  view) follows its support. Both sides must end, in 2 consecutive two-sided
  updates (temporal confirmation), and the evidence expires after 2.5 s; the
  corridor ends at the later of the two ends. A one-sided view never creates
  or extends an end, and a one-sided gap is never an end.
  Opt-in (row_end_position_confirm_m, default None = OFF and bit-identical):
  a two-sided ended update may also continue the streak across a gap longer
  than row_end_valid_s (up to row_end_position_max_gap_s) when both side end
  points agree with the previous two-sided ended observation (same band, no
  failed scan match, a net LiDAR viewpoint displacement along the band larger
  than the tolerance and no contradicting one-sided view in between). The
  final freshness rule is unchanged.
  Opt-in (row_end_continuation_m, default None = OFF and bit-identical): a
  two-sided end of the rule above is NOT an end when BOTH row lines visibly
  continue (pair rule, v2, the default rule): each line extension (a band
  widening with the LiDAR along distance) holds >= row_end_continuation_min_objects
  separated trunk-sized clusters between end + the required free gap and end +
  row_end_continuation_m, inside free_check_range_m and before any cross-aisle
  structure (a cross alley seen through, not a row end). Fences, walls and bins
  near an extension are not trunk-sized; one-sided evidence and one-sided views
  never cancel. Positive evidence only: an occluded or unreturned extension
  (UNKNOWN) and an observed free one (ABSENT) keep the original decision; the
  variant can only remove an end, never create or require one. The frozen v1
  per-side rule (row_end_continuation_rule='per_side') stays selectable to
  reproduce its evaluation.
* next corridor (last-row evidence): per side, a parallel line one learnt width
  beyond the side line is PRESENT, ABSENT (rays observed through that zone and
  no line) or UNKNOWN (zone not observed, e.g. an opaque barrier).
* front and pivot clearances come from the newest raw scan, including branch
  returns. Gyro bias is learnt only while sensor-observed still.

Opt-in inputs and outputs of design v2 (V2-1, V2-3; every one OFF by default and
bit-identical then: the state they use is None and no original value is touched):
* actuator priors (actuator_prior, layer L1; closed_loop/actuator_prior.py):
  on_actuator(stamp, duty_left_pct, duty_right_pct, steer_rad) takes the APPLIED
  drive duty and the calibrated steering angle of an MCU status line. PWM is not
  an encoder: with weight 0 the prior is only recorded and compared with the
  LiDAR speed (online scale, table mismatch); with a weight it seeds the scan
  match and predicts across a failed match or a scan gap for at most
  predict_max_s / predict_max_m. The snapshot gets the 'actuator_prior' block.
* corridor quality (corridor_quality): the snapshot gets 'corridor_quality':
  which sides the newest accepted update saw, the support returns per side, the
  ages and the innovation spread. Nothing reads it back.
* external corridor observations (external_corridor_gain > 0): the provider slot
  on_corridor_observation(stamp, lateral_offset_m, heading_error_rad, width_m,
  quality, source) of a camera or of the executor's local grid. A gated, low
  gain pull of the tracked band; it never creates a band, never keeps one alive
  and never touches the row-end evidence.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from orchard_perception.scan_corridor import observe_scan_corridor

from .actuator_prior import ActuatorPrior, ActuatorPriorConfig   # opt-in layer L1 (OFF: never instantiated)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _first_cell_indices(cells):
    """Ascending indices of the first point in each distinct (x, y) cell.

    Same set as np.sort(np.unique(cells, axis=0, return_index=True)[1]) (numpy's
    return_index sorts stably, i.e. first occurrence); an int64 key is a bijection
    of the cell pair, so the grouping is identical."""
    if not len(cells):
        return np.zeros(0, dtype=np.intp)
    x = cells[:, 0].astype(np.int64)
    y = cells[:, 1].astype(np.int64)
    ymin = y.min()
    _, index = np.unique((x - x.min()) * (int(y.max() - ymin) + 1) + (y - ymin), return_index=True)
    index.sort()
    return index


@dataclass(frozen=True)
class EstimatorConfig:
    # --- nominal calibration (NOT the plant's truth) -------------------------
    lidar_x_in_base_m: float = .18       # SIM_ASSUMED mount (rover.yaml); unmeasured
    footprint_length_m: float = 1.22     # CATALOGUE
    footprint_width_m: float = .70       # CATALOGUE
    pivot_icr_behind_lidar_m: float = .18   # ESTIMATED pooled median 0.18 m (incl. slip)
    pivot_icr_uncertainty_m: float = .25    # ESTIMATED per-event sd 0.25 m
    # --- odometry -------------------------------------------------------------
    icp_max_range_m: float = 12.
    icp_cell_m: float = .12
    icp_iterations: int = 12
    icp_min_pairs: int = 25
    # Legacy bit-exact replay remains explicit. The row executor selects the
    # sensor-only spatially resolved polyline; this is not native SDK odometry.
    distance_mode: str = 'scan_norm_legacy'
    # --- corridor ---------------------------------------------------------------
    corridor_every_n_scans: int = 2
    band_heading_gate_deg: float = 8.
    band_offset_gate_m: float = .35
    band_width_gate_m: float = .5
    band_side_gate_m: float = .3
    band_gain: float = .4
    band_center_gain: float = .25
    band_init_count: int = 3
    band_timeout_s: float = 2.
    band_min_width_m: float = 1.2
    band_max_width_m: float = 8.
    known_width_forget_m: float = 10.        # learnt width expires after this odometry distance without a band
    line_gate_m: float = .3
    line_min_points: int = 8
    acquire_axis_window_deg: float = 50.     # first acquisition: +- around the travel direction
    reacquire_axis_window_deg: float = 20.   # after a loss: +- around the last band axis
    acquire_axis_step_deg: float = 2.
    acquire_cross_bin_m: float = .2
    acquire_along_bin_m: float = .75
    acquire_along_range_m: float = 8.
    acquire_min_coverage: int = 4            # along bins (0.75 m) with returns on a band
    acquire_veto_coverage: int = 2           # returns in >= this many along bins between the lines veto a pair
    acquire_branch_margin_m: float = .7      # canopy zone next to a line, ignored by the veto ...
    acquire_inner_fraction: float = .3       # ... or this fraction of the pair width if larger
    acquire_ambiguity_deg: float = 8.
    acquire_ambiguity_ratio: float = .95
    # --- row end / next corridor -------------------------------------------------
    side_extent_tolerance_m: float = .35
    support_range_m: float = 10.         # matches CorridorConfig.support_range_m
    free_check_range_m: float = 20.      # support resuming within this range cancels an end
    row_end_confirm_updates: int = 2     # consecutive full updates with both sides ended ...
    row_end_valid_s: float = 2.5         # ... each within this age; ends expire after it
    # Opt-in position-consistency bridge (None = OFF = original streak rule). With a value [m]: a
    # two-sided ended update whose previous two-sided ended update is older than row_end_valid_s still
    # continues the streak if, for BOTH sides, the end point agrees with the previous one within this
    # distance along AND across the band, the gap is <= row_end_position_max_gap_s, the band object is
    # the same, no scan match failed in between, the NET LiDAR viewpoint displacement projected on the
    # current band axis exceeds this distance (not the accumulated odometry path) and no one-sided view
    # in between contradicted the remembered end of its side.
    # SIM tuning: 0.5 m is the 10/13 proposal (seed 15 ends 20.68/20.67 m).
    row_end_position_confirm_m: float | None = None
    # 6 s = the 4.6 s two-sided gap of missing_trunk_gap seed 15 + 30 %; with a 0.05 m/s odometry speed
    # bias (real 10/10 LiDAR speed |error| median 0.04-0.05 m/s) the gap costs <= 0.3 m of the 0.5 m budget.
    # Must be finite, > row_end_valid_s and <= ROW_END_POSITION_MAX_GAP_FACTOR * row_end_valid_s.
    row_end_position_max_gap_s: float = 6.
    # Opt-in line-continuation veto (None = OFF = original rule, bit-identical). With a length L [m]: a side
    # that the original rule calls ended is NOT ended when its row line visibly continues beyond the end:
    # returns inside the continuation band around the line extension, in >= row_end_continuation_min_bins
    # distinct row_end_continuation_bin_m along bins between end + needed (the rule's own required free gap;
    # nearer returns are the end tree's canopy that the p97 end trims) and end + L, at a range below
    # free_check_range_m and before any cross-aisle structure (a wall is not row support). Positive evidence
    # only: UNKNOWN (occluded/no return) and ABSENT (observed free) keep the original decision.
    # SIM derivation (development seeds 0-7 only, taskE_crossalley/DIAGNOSIS.txt): the cross-alley free
    # length behind a false end was 5.9-9.5 m (p5-p95), so L must exceed ~9.5 m (8 m left 9 of 18 false runs);
    # 12 m keeps a margin and bounds how far a resuming line (e.g. a next block) can cancel an end.
    row_end_continuation_m: float | None = None
    # Band half width at the LiDAR (= side_extent_tolerance_m) + slope per metre of LiDAR along distance (about
    # 2 deg of line axis error), at most band_max: the line offset jitter (~0.2 m) and axis error extrapolated
    # 9-15 m ahead put the resuming trunks outside the fixed 0.35 m band (6 of 18 false runs kept with it).
    row_end_continuation_band_m: float = .35
    row_end_continuation_band_slope: float = .035
    row_end_continuation_band_max_m: float = .8   # must stay below band_min_width_m (never the opposite line)
    row_end_continuation_bin_m: float = .5
    row_end_continuation_min_bins: int = 2        # one stray return / branch tip does not cancel an end
    # Which rule row_end_continuation_m switches on (read only while it is set):
    #   'pair' (v2): a two-sided end of the original rule is cancelled only when BOTH line extensions show
    #       >= row_end_continuation_min_objects separated trunk-like objects (a cross alley resumes both lines;
    #       one-sided evidence - a fence, post, bin or wall near one extension - never cancels an end, and a side
    #       the original rule did not end is never examined). Objects are 2-D single-linkage clusters of the
    #       returns around the band; a cluster counts only if it has a band return inside the zone, >=
    #       object_min_points returns and a trunk-sized extent (<= object_along_m along, <= object_cross_m across,
    #       measured over a context as large as these limits beyond the zone, so a longer structure cannot hide
    #       behind the context edge): continuous or band-crossing structures (parallel fences, oblique walls, bin
    #       faces) are excluded; objects closer than object_sep_m along count once (one tree's trunk and canopy
    #       pieces). The v1 bin settings are not read.
    #   'per_side' (v1, frozen cf2ed7f semantics, kept only to reproduce that evaluation): each ended side with
    #       >= min_bins occupied band bins is not ended. Review F1: one-sided clutter cancels true row ends.
    row_end_continuation_rule: str = 'pair'
    row_end_continuation_link_m: float = .35
    row_end_continuation_object_along_m: float = 1.2
    row_end_continuation_object_cross_m: float = .8
    row_end_continuation_object_min_points: int = 3
    row_end_continuation_object_sep_m: float = 1.5
    row_end_continuation_min_objects: int = 2
    core_margin_m: float = .6            # aisle core = between the lines minus this canopy margin
    cross_structure_min_points: int = 6
    cross_structure_min_span: float = .5  # fraction of the aisle core width
    end_free_m: float = 1.5
    end_gap_margin_m: float = .5
    # Opt-in support segmentation (None = OFF = original rule, bit-identical). With a span L [m]: a side's support
    # (line band, inside support_range_m) is cut at its FIRST free gap that reaches ahead of the LiDAR and is at
    # least row_end_segment_min_gap_m and the side's own required free gap (max(end_free_m, p75 of the in-row gaps
    # before it + end_gap_margin_m)): the side ends at the support before that gap, the returns beyond it bound the
    # observed free space (the minimum gap keeps sparse last trunks 3-5 m apart in the row: 040927 L6-R8 rev). The
    # original 97th percentile over ALL support inside the range took any structure that entered the range on the
    # line extension beyond the row end (a fence, post, bush or bin 7-10 m beyond it) for row support, so a
    # confirmed end vanished again in the approach (SIM single-row dev round 1: 035952 L0-R2 rev, 042301 L4-R1 fwd,
    # 045553 L3-R2 fwd). A cross gap is not cut: when the support beyond each side's end (its cut, or its original
    # end) up to free_check_range_m and before a cross-aisle structure is row-like on BOTH sides (returns in >=
    # row_end_segment_min_bins distinct row_end_segment_bin_m along bins, >= 2 returns each, spanning >= L), the
    # original rule applies to both sides (dev round 2: a 5 m two-sided gap had the far side's resumed trunks just
    # beyond the 10 m range on one line and inside it on the other). One-sided gaps never end the corridor.
    row_end_segment_span_m: float | None = None
    row_end_segment_min_gap_m: float = 6.
    row_end_segment_bin_m: float = .5
    row_end_segment_min_bins: int = 4
    # Opt-in one-sided band update about the vehicle (False = OFF = original rule, bit-identical). The band is a line pair
    # in the odometry frame: heading + the centre offset along its normal measured from the odometry ORIGIN (the run's
    # start). A full update re-anchors the centre at the vehicle after its heading step, but a one-sided update kept the
    # origin-referenced centre (only blended by band_center_gain), so its heading step g/2 x (observed - band) turned
    # the band about the start point: at 36 m from the start a 0.7 deg step moved the corridor by 0.45 m at the vehicle
    # (SIM single-row dev round 1: 040927 L1-R2 fwd lateral estimate jumps 0.4-1.0 m, then obstacle_ahead / guard_lateral;
    # the jump grows with the distance driven). With True the one-sided update keeps the centre's offset from the
    # vehicle through the heading step (rotation about the vehicle) before blending toward the observed side.
    band_rotate_about_vehicle: bool = False
    # Opt-in staggered-end check (None = OFF = original rule, bit-identical; needs row_end_segment_span_m for the
    # row-like test). With a gap G [m]: when BOTH sides ended in this update but one side's end lies more than
    # row_end_stagger_window_m beyond the other's, the farther side's support (line band, inside support_range_m) is
    # searched for a free gap of at least G and the side's own required gap that reaches ahead of the LiDAR and starts
    # within row_end_stagger_window_m of the nearer side's end; when the support beyond that gap (up to the farther
    # end) is not row-like (_row_like), the farther side ends at its support before the gap. Both rows end at the same
    # place there; an isolated structure on one line extension 4-6 m beyond the row end (a post, bin or bush in the
    # headland, inside the 6 m segmentation gap) is not row support (SIM single-row dev round 2: 045553 L3-R2 fwd, the
    # right row ended at the true end, a 0.6 m object 4.9 m beyond the left row's last trunk became the left end, the
    # row end moved onto it -> MISSED_ROW_END in 59 of 84 episodes). The window keeps a two-sided gap (both ends at the
    # gap) and a stagger whose far side continues as a row unchanged. Limitation: a row that really continues alone
    # for one isolated last trunk beyond a gap >= G that starts at the other row's end is cut there (no such stagger
    # among the labelled units of the 8 DEV corridors: their unit gaps within 14 m of the end are <= 3.9 m).
    row_end_stagger_gap_m: float | None = None
    row_end_stagger_window_m: float = 2.
    # Opt-in last-trunk end (False = OFF = original rule, bit-identical). With True a side's support end is at least the
    # median of the support's last cluster (>= 2 returns, 0.3 m linkage) within 2 m beyond the 97th percentile: a small
    # last trunk (a few returns) that the percentile trims stays the end (SIM single-row dev round 2: 042301 L3-R11 fwd,
    # the right row's last unit is a single 5 cm cell; trimmed, the end fell 1.0 m short -> ROW_END_EARLY e -1.0..-1.3).
    row_end_last_trunk: bool = False
    # Opt-in initial acquisition window (None = OFF = original symmetric +-acquire_along_range_m). With a value B [m]:
    # the FIRST corridor acquisition of a run (no band learnt yet) uses only returns from B behind the LiDAR to
    # acquire_along_range_m ahead. A single-row start is made at the row entry facing into the row; headland
    # structures behind the vehicle (a fence or wall across the aisle, bins, bushes) are not the corridor to follow,
    # but their returns between the line extensions vetoed every pair (SIM single-row dev round 1: 040927 L6-R8 rev,
    # 042301 L3-R11 fwd, 042301 L4-R1 fwd: corridor_not_acquired at the start). Re-acquisition after a loss keeps
    # the symmetric window (the corridor then lies behind as well as ahead).
    acquire_behind_m: float | None = None
    next_line_gate_m: float = .35
    next_min_coverage: int = 4
    next_min_observed_rays: int = 40
    next_present_votes: int = 3
    next_absent_votes: int = 5
    # --- stillness / IMU ---------------------------------------------------------
    gyro_lowpass_s: float = .25
    vibration_window_s: float = .5
    still_yaw_deg_s: float = 1.5
    still_translation_m: float = .025
    still_rotation_deg: float = .6
    keyframe_translation_m: float = .4
    keyframe_rotation_deg: float = 8.
    # --- design v2 opt-in inputs / outputs (all OFF = original rules, bit-identical) ---------------------------------
    # Actuator priors, layer L1 (None = OFF: on_actuator is ignored). An ActuatorPriorConfig: speed table, scale,
    # wheelbase, weight (0 = record and monitor only) and the prediction bound; closed_loop/actuator_prior.py.
    actuator_prior: ActuatorPriorConfig | None = None
    # Corridor quality block in the snapshot (False = OFF: the snapshot keeps its original keys).
    corridor_quality: bool = False
    # External corridor observation slot (0 = OFF: on_corridor_observation is ignored). With a gain g in (0, 1]: an
    # observation of an accepted source, not older than external_corridor_max_age_s, with quality >=
    # external_corridor_min_quality and inside the band gates (band_heading_gate_deg, band_offset_gate_m,
    # band_width_gate_m) pulls the tracked band by g (heading about the vehicle, then the centre, then the width).
    external_corridor_gain: float = 0.
    external_corridor_sources: tuple = ()       # accepted source names, e.g. ('local_grid',); empty = none
    external_corridor_max_age_s: float = .5
    external_corridor_min_quality: float = .5

    def __post_init__(self):
        if self.actuator_prior is not None and not isinstance(self.actuator_prior, ActuatorPriorConfig):
            raise ValueError('actuator_prior_invalid: an ActuatorPriorConfig or None')
        if not (isinstance(self.external_corridor_gain, (int, float)) and 0. <= self.external_corridor_gain <= 1.):
            raise ValueError('external_corridor_invalid: gain must be in [0, 1]')
        if self.external_corridor_gain > 0. and not (
                isinstance(self.external_corridor_sources, tuple) and self.external_corridor_sources
                and all(isinstance(name, str) and name for name in self.external_corridor_sources)
                and math.isfinite(self.external_corridor_max_age_s) and self.external_corridor_max_age_s > 0.
                and math.isfinite(self.external_corridor_min_quality)
                and 0. <= self.external_corridor_min_quality <= 1.):
            raise ValueError('external_corridor_invalid: a gain needs source names, max_age_s > 0 and '
                             'min_quality in [0, 1]')
        if self.distance_mode not in ('scan_norm_legacy', 'resolved_polyline'):
            raise ValueError('unknown_sensor_path_distance_mode')
        if self.row_end_segment_span_m is not None and not (
                math.isfinite(self.row_end_segment_span_m) and self.row_end_segment_span_m > 0.
                and math.isfinite(self.row_end_segment_bin_m) and self.row_end_segment_bin_m > 0.
                and math.isfinite(self.row_end_segment_min_gap_m) and self.row_end_segment_min_gap_m >= self.end_free_m
                and _count_ok(self.row_end_segment_min_bins)):
            raise ValueError('row_end_segment_invalid: span_m and bin_m must be finite and > 0, min_gap_m finite and '
                             '>= end_free_m, min_bins an int >= 1')
        if self.acquire_behind_m is not None and not (
                math.isfinite(self.acquire_behind_m) and 0. <= self.acquire_behind_m < self.acquire_along_range_m):
            raise ValueError('acquire_behind_invalid: must be finite and in [0, acquire_along_range_m)')
        if self.row_end_stagger_gap_m is not None and not (
                self.row_end_segment_span_m is not None and math.isfinite(self.row_end_stagger_gap_m)
                and self.row_end_stagger_gap_m >= self.end_free_m and math.isfinite(self.row_end_stagger_window_m)
                and self.row_end_stagger_window_m > 0.):
            raise ValueError('row_end_stagger_invalid: gap_m finite and >= end_free_m, window_m finite and > 0, '
                             'row_end_segment_span_m set (row-like test)')
        if self.row_end_continuation_m is not None:
            self._check_continuation()      # OFF: the continuation settings are not read (default unchanged)
        tolerance = self.row_end_position_confirm_m
        if tolerance is None:
            return                          # OFF: the variant settings are not read (default unchanged)
        gap = self.row_end_position_max_gap_s
        if not (math.isfinite(tolerance) and 0. < tolerance < self.end_free_m):
            # A tolerance as large as the free-space requirement could link two different ends.
            raise ValueError('row_end_position_invalid: confirm_m must be finite, > 0 and < end_free_m')
        if not (math.isfinite(gap) and self.row_end_valid_s < gap
                <= ROW_END_POSITION_MAX_GAP_FACTOR * self.row_end_valid_s):
            # A maximum gap not above row_end_valid_s would never bridge anything; an unbounded one
            # (inf) would keep a two-sided observation as evidence for ever (see the factor below).
            raise ValueError('row_end_position_invalid: max_gap_s must be finite, > row_end_valid_s and '
                             '<= %g * row_end_valid_s' % ROW_END_POSITION_MAX_GAP_FACTOR)

    def _check_continuation(self):
        length = self.row_end_continuation_m
        if not (math.isfinite(length) and self.end_free_m < length <= self.free_check_range_m):
            # The checked zone starts at end + needed >= end + end_free_m; returns beyond free_check_range_m are
            # outside the range the original rule treats as observed.
            raise ValueError('row_end_continuation_invalid: continuation_m must be finite, > end_free_m and '
                             '<= free_check_range_m')
        band, slope, top = (self.row_end_continuation_band_m, self.row_end_continuation_band_slope,
                            self.row_end_continuation_band_max_m)
        if not (math.isfinite(band) and math.isfinite(slope) and math.isfinite(top)
                and 0. < band <= top < self.band_min_width_m and slope >= 0.):
            # A band as wide as the narrowest accepted corridor would take the opposite line as continuation.
            raise ValueError('row_end_continuation_invalid: need 0 < band_m <= band_max_m < band_min_width_m '
                             'and band_slope >= 0 (all finite)')
        rule = self.row_end_continuation_rule
        if rule not in CONTINUATION_RULES:
            raise ValueError('row_end_continuation_invalid: rule must be one of %s' % (CONTINUATION_RULES,))
        if rule == 'per_side':
            bins = self.row_end_continuation_min_bins
            if not (math.isfinite(self.row_end_continuation_bin_m) and self.row_end_continuation_bin_m > 0.
                    and _count_ok(bins)):
                raise ValueError('row_end_continuation_invalid: bin_m must be finite and > 0, min_bins an int >= 1')
            return
        lengths = (self.row_end_continuation_link_m, self.row_end_continuation_object_along_m,
                   self.row_end_continuation_object_cross_m, self.row_end_continuation_object_sep_m)
        if not (all(math.isfinite(v) for v in lengths) and self.row_end_continuation_link_m > 0.
                and self.row_end_continuation_object_along_m > 0. and self.row_end_continuation_object_cross_m > 0.
                and self.row_end_continuation_object_sep_m >= 0.):
            raise ValueError('row_end_continuation_invalid: pair rule needs finite link_m > 0, object_along_m > 0, '
                             'object_cross_m > 0, object_sep_m >= 0')
        if not (_count_ok(self.row_end_continuation_object_min_points)
                and _count_ok(self.row_end_continuation_min_objects)):
            raise ValueError('row_end_continuation_invalid: object_min_points and min_objects must be ints >= 1')


def _last_trunk_end(a, end):
    """Opt-in row_end_last_trunk: the side end is at least the median of the support's LAST cluster (1-D single
    linkage 0.3 m, >= 2 returns, at or before `end` + the trimmed tail): a small last trunk that the 97th percentile
    trims is kept at its centre; a dense canopy tail (its median before the percentile) leaves the end unchanged."""
    tail = a[a <= end + 2.]               # never beyond a cut / free gap: only the trimmed tail of this support
    if len(tail) < 2:
        return end
    groups = np.split(tail, np.flatnonzero(np.diff(tail) > .3) + 1)
    for group in reversed(groups):
        if len(group) >= 2:
            return max(end, float(np.median(group)))
    return end


def _count_ok(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 1


CONTINUATION_RULES = ('pair', 'per_side')
CONTINUATION_KEYS = ('checked', 'vetoed', 'cancelled')   # ended sides examined / cancelled; two-sided ends cancelled
# Pair rule (v2): the same keys (sides examined = 2 per two-sided end of the original rule; sides vetoed = 2 per
# cancellation) + 'one_sided': two-sided ends with continuation evidence on exactly one side (kept as ends).
CONTINUATION_PAIR_KEYS = CONTINUATION_KEYS + ('one_sided',)


def _components(xy, link):
    """Labels of the single-linkage clusters of 2-D points (pairs not farther apart than link are connected)."""
    n = len(xy)
    if n < 2:
        return np.zeros(n, dtype=np.intp)
    pairs = cKDTree(xy).query_pairs(link, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
    return connected_components(graph, directed=False)[1]


def continuation_objects(config, along, cross, rng, offset, end, needed, front):
    """Pair rule (v2): number of separated trunk-like objects on one side's line extension (laser frame).

    Zone: along in [end + needed, end + row_end_continuation_m), range < free_check_range_m, before a
    cross-aisle structure (front - 0.3), |cross - offset| < band(along) (v1 band: band_m + band_slope x along,
    at most band_max_m). Context: the zone box widened by object_cross_m laterally and object_along_m along (any
    range, also beyond the front), i.e. at least the largest accepted object, so a longer structure is never cut
    to an object-sized piece by the context edge. Context returns are clustered (single linkage, link_m). A
    cluster is an object if it has a zone return, >= object_min_points returns and ALL its context returns span
    <= object_along_m along and <= object_cross_m across (trunk sized: a fence parallel to the line, a wall
    crossing the band or a bin face is longer). Object position = median along of its zone returns; positions
    closer than object_sep_m to the previous counted object (nearest first) count once (one tree's trunk and
    canopy pieces)."""
    cfg = config
    start, stop = end + needed, end + cfg.row_end_continuation_m
    along_margin, cross_margin = cfg.row_end_continuation_object_along_m, cfg.row_end_continuation_object_cross_m
    rel = cross - offset
    width = np.minimum(cfg.row_end_continuation_band_max_m,
                       cfg.row_end_continuation_band_m + cfg.row_end_continuation_band_slope * np.maximum(0., along))
    context = (np.abs(rel) < width + cross_margin) & (along >= start - along_margin) & (along < stop + along_margin)
    if not context.any():
        return 0
    a, r = along[context], rel[context]
    zone = (np.abs(r) < width[context]) & (a >= start) & (a < stop) & (rng[context] < cfg.free_check_range_m)
    if front is not None:
        zone &= a < front - .3                  # as the original rule: a wall is not row support
    if not zone.any():
        return 0
    labels = _components(np.column_stack((a, r)), cfg.row_end_continuation_link_m)
    positions = []
    for label in np.unique(labels[zone]):
        member = labels == label
        if member.sum() < cfg.row_end_continuation_object_min_points:
            continue                            # a stray return / branch tip
        if (np.ptp(a[member]) > cfg.row_end_continuation_object_along_m
                or np.ptp(r[member]) > cfg.row_end_continuation_object_cross_m):
            continue                            # a structure, not a trunk
        positions.append(float(np.median(a[member & zone])))
    count, last = 0, -math.inf
    for position in sorted(positions):
        if position - last >= cfg.row_end_continuation_object_sep_m:
            count, last = count + 1, position
    return count


# Upper bound of row_end_position_max_gap_s in units of row_end_valid_s (10 s with the default 2.5 s).
# The bridge links two observations through the LiDAR odometry; a speed bias of 0.05 m/s (real 10/10
# LiDAR speed |error| median 0.04-0.05 m/s) accumulates 0.5 m in 10 s, i.e. the whole 0.5 m proposal
# tolerance, after which agreement within the tolerance no longer separates one end from another. The
# bound also keeps the 10 s diagnostic sensitivity setting of the 19eafed evaluation valid.
ROW_END_POSITION_MAX_GAP_FACTOR = 4.

END_POSITION_REASONS = ('bridged', 'no_memory', 'gap', 'odometry', 'travel', 'along', 'lateral', 'contradicted')


@dataclass
class Band:
    heading: float          # world (odometry) axis, oriented with travel
    center: float           # centreline offset along the left normal
    width: float
    stamp: float
    full_updates: int = 0
    end_left: float | None = None    # along-axis world coordinate of observed support end
    end_right: float | None = None
    end_stamp: float | None = None


class ResolvedPathLength:
    """Monotonic resolved SE(2) base-translation polyline, not per-scan norms.

    A stationary pose cloud contributes its maximum anchor-relative extent
    once, not every scan. Spatially resolved segments are committed at the
    existing keyframe translation resolution. A return by at least one ICP
    cell along the last segment commits the observed extreme as a reversal;
    forward/back travel is then counted separately. Excursions below that
    resolution and high-curvature paths are not claimed exact. A match gap
    breaks the polyline: unobserved translation is never invented.
    """
    def __init__(self, segment_m, reversal_m):
        if not (math.isfinite(segment_m) and math.isfinite(reversal_m)
                and 0 < reversal_m <= segment_m):
            raise ValueError('positive_sensor_path_resolution')
        self.segment_m, self.reversal_m = float(segment_m), float(reversal_m)
        self.anchor = self.extreme = None
        self.extent_m = self.committed_m = 0.
        self.segments = self.reversals = self.gaps = 0

    @property
    def distance(self):return self.committed_m + self.extent_m

    def break_segment(self):
        self.committed_m += self.extent_m
        self.extent_m = 0.
        self.anchor = self.extreme = None
        self.gaps += 1
        return self.distance

    def update(self, x_m, y_m):
        if not (math.isfinite(x_m) and math.isfinite(y_m)):
            raise ValueError('finite_sensor_path_pose')
        point = (float(x_m), float(y_m))
        if self.anchor is None:
            self.anchor = self.extreme = point
            return self.distance
        if self.extent_m >= self.reversal_m:
            axis = ((self.extreme[0] - self.anchor[0]) / self.extent_m,
                    (self.extreme[1] - self.anchor[1]) / self.extent_m)
            returned = ((point[0] - self.extreme[0]) * axis[0]
                        + (point[1] - self.extreme[1]) * axis[1])
            if returned <= -self.reversal_m:
                self.committed_m += self.extent_m
                self.anchor = self.extreme
                self.extent_m = 0.
                self.reversals += 1
        radius = math.hypot(point[0] - self.anchor[0], point[1] - self.anchor[1])
        if radius > self.extent_m:
            self.extent_m, self.extreme = radius, point
        if radius >= self.segment_m:
            self.committed_m += self.extent_m
            self.anchor = self.extreme = point
            self.extent_m = 0.
            self.segments += 1
        return self.distance


QUALITY_GAIN = .2            # EMA gain of the squared innovations in CorridorQuality
EXTERNAL_REASONS = ('fused', 'no_band', 'source', 'quality', 'invalid', 'order', 'stale', 'future', 'no_pose',
                    'heading_gate', 'offset_gate', 'width_gate')


class CorridorQuality:
    """Opt-in corridor_quality: what the accepted corridor updates saw (bookkeeping only, nothing reads it back).

    sides          'both' / 'left' / 'right' of the newest accepted update, 'none' without a band
    source         'init', 'candidate' (detector lines) or 'guided' (returns near the predicted lines)
    support        returns within line_gate_m of the left / right line of that update (|along| < 8 m)
    stamps         of the newest update, the newest two-sided update and the newest update of each side
    one_sided_run  accepted one-sided updates since the last two-sided one
    rejected_run   corridor updates that were refused since the last accepted one
    innovation     EMA (gain QUALITY_GAIN) of the squared innovations (observed - predicted) of the accepted updates:
                   lateral offset [m], heading [rad], width [m] (two-sided updates only); reported as sd
    """

    def __init__(self):
        self.sides = 'none'
        self.source = None
        self.support = [0, 0]
        self.stamp = self.full_stamp = None
        self.side_stamp = [None, None]
        self.one_sided_run = 0
        self.rejected_run = 0
        self.variance = dict(lateral=None, heading=None, width=None)

    def _innovation(self, name, value):
        if value is None:
            return
        previous = self.variance[name]
        square = value * value
        self.variance[name] = square if previous is None else previous + QUALITY_GAIN * (square - previous)

    def accepted(self, stamp, sides, source, support, lateral=None, heading=None, width=None):
        self.sides, self.source, self.stamp = sides, source, stamp
        self.support = list(support)
        self.rejected_run = 0
        if sides == 'both':
            self.full_stamp = stamp
            self.side_stamp = [stamp, stamp]
            self.one_sided_run = 0
        else:
            self.side_stamp[0 if sides == 'left' else 1] = stamp
            self.one_sided_run += 1
        for name, value in (('lateral', lateral), ('heading', heading), ('width', width)):
            self._innovation(name, value)

    def rejected(self):
        self.rejected_run += 1

    def lost(self):
        self.sides, self.source = 'none', None
        self.support = [0, 0]

    def snapshot(self, now, band):
        def age(stamp):
            return None if stamp is None else now - stamp

        def sd(name):
            value = self.variance[name]
            return None if value is None else math.sqrt(value)
        return dict(sides=self.sides if band is not None else 'none', source=self.source,
                    support_left=self.support[0], support_right=self.support[1], age_s=age(self.stamp),
                    full_age_s=age(self.full_stamp), left_age_s=age(self.side_stamp[0]),
                    right_age_s=age(self.side_stamp[1]), one_sided_run=self.one_sided_run,
                    rejected_run=self.rejected_run, lateral_sd_m=sd('lateral'), heading_sd_rad=sd('heading'),
                    width_sd_m=sd('width'))


class ExternalCorridor:
    """Opt-in external_corridor_gain: bookkeeping of the provider slot (last observation and outcome counts)."""

    def __init__(self):
        self.counts = dict.fromkeys(EXTERNAL_REASONS, 0)
        self.last_stamp = None           # stamp of the newest observation that was looked at (order check)
        self.fused_stamp = None
        self.fused_source = None
        self.last_reason = None

    def note(self, reason):
        self.counts[reason] += 1
        self.last_reason = reason
        return reason

    def snapshot(self, now):
        return dict(fused=self.counts['fused'], refused=sum(v for k, v in self.counts.items() if k != 'fused'),
                    last_reason=self.last_reason, source=self.fused_source,
                    age_s=None if self.fused_stamp is None else now - self.fused_stamp)


class SensorEstimator:
    def __init__(self, config=EstimatorConfig()):
        self.config = config
        self.imu = deque()
        self.last_imu_stamp = None
        self.gyro_lp = 0.
        self.gyro_bias = 0.
        self.bias_samples = 0
        self.gyro_heading = 0.           # bias-corrected raw gyro integral (not a map yaw)
        self.last_scan_stamp = None
        self.scan_count = 0
        self.keyframe = None             # (points, pose)
        self.history = deque(maxlen=12)  # (stamp, pose, odom_valid) of recent scans
        self.pose = np.zeros(3)          # laser pose in odometry frame at last scan
        self.speed = 0.
        self.speed_samples = deque(maxlen=5)
        self.odom_valid = False
        self.odom_failures = 0
        self.distance = 0.
        if config.distance_mode == 'resolved_polyline':
            self.path_length = ResolvedPathLength(config.keyframe_translation_m, config.icp_cell_m)
        self.still_since = None
        self.band = None
        self.candidates = []
        self.known_width = None          # learnt row width survives a band loss ...
        self.band_lost_at_m = None       # ... until the vehicle has moved known_width_forget_m
        self.last_band_heading = None    # learnt axis survives a band loss (re-acquisition prior)
        self.accepted_updates = 0        # monotonic count of accepted corridor updates
        self.front_clearance = math.inf
        self.pivot_clearance = math.inf
        self.side_clearance = math.inf
        self.side_left = self.side_right = math.inf     # per-side split of side_clearance (same band)
        self.side_left_x = self.side_right_x = None     # base x of the nearest return on that side
        self.corridor_reason = 'waiting'
        self.reference_heading = None
        self.end_free = [False, False]
        self.end_streak = 0              # consecutive full updates with both sides ended
        self.next_votes = dict(left=[0, 0], right=[0, 0])   # [present, absent]
        # Opt-in row_end_position_* only (never written while OFF): the last two-sided ended
        # observation, the outcome of every bridge attempt and the first bridges (evaluation).
        self.end_position_memory = None
        self.end_position_counts = dict.fromkeys(END_POSITION_REASONS, 0)
        self.end_position_events = []
        # Opt-in row_end_continuation_m only (never written while OFF): the veto detail of the latest
        # _side_extents call, counts over full updates and the first cancelled two-sided ends (evaluation).
        self.continuation_last = None
        pair = config.row_end_continuation_m is not None and config.row_end_continuation_rule == 'pair'
        self.continuation_counts = dict.fromkeys(CONTINUATION_PAIR_KEYS if pair else CONTINUATION_KEYS, 0)
        self.continuation_events = []
        # Design v2 opt-in layers (None while OFF and never touched then): actuator priors (L1), the corridor quality
        # bookkeeping and the external corridor observation slot.
        self.prior = None if config.actuator_prior is None else ActuatorPrior(config.actuator_prior)
        self.quality = CorridorQuality() if config.corridor_quality else None
        self.external = ExternalCorridor() if config.external_corridor_gain > 0. else None

    # --- opt-in actuator status (never stored while OFF) -------------------------------------------------
    def on_actuator(self, stamp, duty_left_pct, duty_right_pct, steer_rad=None):
        """Opt-in actuator_prior: one MCU status line on the estimator clock - the APPLIED drive duty of each side
        [% of full scale, signed, + forward] and the calibrated steering angle [rad, + left; None = not known].
        Not the request and not a wheel speed. Returns False while OFF or for a refused sample."""
        if self.prior is None:
            return False
        return self.prior.update(stamp, duty_left_pct, duty_right_pct, steer_rad)

    # --- IMU -------------------------------------------------------------
    def on_imu(self, stamp, gyro, accel):
        if self.last_imu_stamp is not None and stamp <= self.last_imu_stamp:
            return
        dt = .05 if self.last_imu_stamp is None else min(.2, stamp - self.last_imu_stamp)
        self.last_imu_stamp = stamp
        self.imu.append((stamp, gyro, accel))
        self.gyro_heading += (gyro - self.gyro_bias) * dt
        while self.imu and stamp - self.imu[0][0] > 3.:
            self.imu.popleft()
        a = 1 - math.exp(-dt / self.config.gyro_lowpass_s)
        self.gyro_lp += (gyro - self.gyro_lp) * a
        if self.is_still(stamp, minimum_s=.5):
            self.bias_samples = min(self.bias_samples + 1, 200)
            self.gyro_bias += (gyro - self.gyro_bias) / self.bias_samples

    def gyro_integral(self, t0, t1):
        total, previous = 0., None
        for stamp, gyro, _ in self.imu:
            if t0 < stamp <= t1:
                step = .05 if previous is None else min(.1, stamp - previous)
                total += (gyro - self.gyro_bias) * step
            previous = stamp
        return total

    def vibration_rms(self, now):
        window = [a for s, _, a in self.imu if now - s <= self.config.vibration_window_s]
        if len(window) < 3:
            return None
        values = np.asarray(window)
        return float(np.sqrt(np.mean(np.sum((values - values.mean(axis=0)) ** 2, axis=1))))

    def yaw_rate(self):
        return self.gyro_lp - self.gyro_bias

    # --- scans -----------------------------------------------------------
    @staticmethod
    def _points(ranges, angle_min, increment, range_max):
        r = np.asarray(ranges, dtype=float)
        angles = angle_min + np.arange(len(r)) * increment
        valid = np.isfinite(r) & (r > 0) & (r < range_max)
        return np.column_stack((r[valid] * np.cos(angles[valid]), r[valid] * np.sin(angles[valid])))

    def _icp(self, reference, current, guess):
        cfg = self.config
        tree = cKDTree(reference)
        x, y, t = guess
        limit = .5
        for _ in range(cfg.icp_iterations):
            c, s = math.cos(t), math.sin(t)
            moved = current @ np.array(((c, s), (-s, c))) + (x, y)
            distance, index = tree.query(moved, distance_upper_bound=limit)
            mask = np.isfinite(distance)
            if mask.sum() < cfg.icp_min_pairs:
                return None
            a, b = current[mask], reference[index[mask]]
            ma, mb = a.mean(axis=0), b.mean(axis=0)
            h = (a - ma).T @ (b - mb)
            t = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
            c, s = math.cos(t), math.sin(t)
            x, y = mb - np.array(((c, -s), (s, c))) @ ma
            limit = max(.15, limit * .8)
        c, s = math.cos(t), math.sin(t)
        distance, _ = tree.query(current @ np.array(((c, s), (-s, c))) + (x, y), distance_upper_bound=.15)
        return (float(x), float(y), float(t)), float(np.isfinite(distance).mean())

    def on_scan(self, stamp, ranges, angle_min, increment, range_max):
        cfg = self.config
        if self.last_scan_stamp is not None and stamp <= self.last_scan_stamp:
            return
        points = self._points(ranges, angle_min, increment, range_max)
        near = points[np.hypot(points[:, 0], points[:, 1]) < cfg.icp_max_range_m]
        sparse = near[_first_cell_indices(np.floor(near / cfg.icp_cell_m).astype(np.int32))]
        if self.last_scan_stamp is not None and self.keyframe is not None:
            dt = stamp - self.last_scan_stamp
            rotation = self.gyro_integral(self.last_scan_stamp, stamp)
            c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
            # Opt-in actuator prior (a): a blended translation seed; None (OFF, weight 0 or no status over the interval)
            # keeps the original seed below, bit for bit.
            seed = None if self.prior is None else self.prior.seed(self.last_scan_stamp, stamp, self.speed)
            if seed is None:
                predicted = np.array((self.pose[0] + c * self.speed * dt, self.pose[1] + s * self.speed * dt,
                                      wrap(self.pose[2] + rotation)))
            else:
                predicted = np.array((self.pose[0] + c * seed, self.pose[1] + s * seed, wrap(self.pose[2] + rotation)))
            key_points, key_pose = self.keyframe
            # A scan gap of 0.5 s or more is matched only with a prior seed (the LiDAR speed alone is no seed then).
            result = (self._icp(key_points, sparse, _relative(key_pose, predicted))
                      if len(sparse) >= cfg.icp_min_pairs and (dt < .5 or seed is not None) else None)
            previous = self.pose
            if result is not None and result[1] >= .5:
                self.pose = _compose(key_pose, result[0])
                self.odom_valid = True
                if self.prior is not None:
                    self.prior.matched()
            else:
                # Degraded: gyro rotation only; translation is NOT invented.
                self.pose = np.array((previous[0], previous[1], wrap(previous[2] + rotation)))
                self.odom_valid = False
                self.odom_failures += 1
                if self.prior is not None:
                    # Opt-in actuator prior (b): a bounded prediction along the mid heading (0 with weight 0, without
                    # a status line over the interval and beyond predict_max_s / predict_max_m).
                    moved = self.prior.predict(self.last_scan_stamp, stamp)
                    if moved:
                        mid = wrap(previous[2] + .5 * rotation)
                        self.pose = np.array((previous[0] + math.cos(mid) * moved,
                                              previous[1] + math.sin(mid) * moved, self.pose[2]))
            step = _relative(previous, self.pose)
            if dt > 0 and self.odom_valid:
                self.speed_samples.append(step[0] / dt)
                self.speed = float(np.median(self.speed_samples))
                if cfg.distance_mode == 'scan_norm_legacy':
                    self.distance += math.hypot(step[0], step[1])
                if self.prior is not None:
                    self.prior.observe(self.last_scan_stamp, stamp, step[0])     # (c) monitor, (e) distance
            relative = _relative(key_pose, self.pose)
            if (not self.odom_valid or math.hypot(relative[0], relative[1]) > cfg.keyframe_translation_m
                    or abs(relative[2]) > math.radians(cfg.keyframe_rotation_deg)):
                self.keyframe = (sparse, self.pose.copy())
        else:
            self.keyframe = (sparse, self.pose.copy())
        if cfg.distance_mode == 'resolved_polyline':
            if self.odom_valid or self.last_scan_stamp is None:
                # Nominal measured-frame geometry only. No truth or applied
                # duty enters this calculation; rotation about the base does
                # not count the LiDAR lever-arm arc as base translation.
                bx = self.pose[0] - cfg.lidar_x_in_base_m * math.cos(self.pose[2])
                by = self.pose[1] - cfg.lidar_x_in_base_m * math.sin(self.pose[2])
                self.distance = self.path_length.update(bx, by)
            else:
                self.distance = self.path_length.break_segment()
        self.history.append((stamp, self.pose.copy(), self.odom_valid))
        self._update_stillness(stamp)
        self.last_scan_stamp = stamp
        self.scan_count += 1
        if self.reference_heading is None:
            self.reference_heading = float(self.pose[2])
        self._clearances(points)
        if self.band is None or self.scan_count % cfg.corridor_every_n_scans == 0:
            self._corridor(stamp, _Scan(ranges, angle_min, increment, range_max), points)
        if self.band is not None and stamp - self.band.stamp > cfg.band_timeout_s:
            self.last_band_heading = self.band.heading
            self.band_lost_at_m = self.distance
            self.band = None
            self.candidates = []
            self.corridor_reason = 'corridor_lost'
            if self.quality is not None:
                self.quality.lost()

    def _update_stillness(self, stamp):
        """Still when every recent pose stays within a few cm / tenths of a degree."""
        cfg = self.config
        # At low duty, 0.06 m/s travels only 2.4 cm in the old 0.4 s
        # pose span, so the legacy 2.5 cm cloud rule falsely reports still.
        # Preserve that exact legacy behavior, but require substantially longer
        # sensor evidence for the resolved low-speed profile. Spatial/yaw
        # tolerances and the controller's no-response/Stop rules are unchanged.
        horizon = 1. if cfg.distance_mode == 'resolved_polyline' else .45
        window = [(t, pose, ok) for t, pose, ok in self.history if stamp - t <= horizon]
        current = self.history[-1][1]
        covered = (cfg.distance_mode == 'scan_norm_legacy' or
                   (len(window) >= 2 and window[-1][0] - window[0][0] >= .8))
        steady = (covered and len(window) >= 4 and all(ok for _, _, ok in window) and all(
            math.hypot(pose[0] - current[0], pose[1] - current[1]) <= cfg.still_translation_m
            and abs(wrap(pose[2] - current[2])) <= math.radians(cfg.still_rotation_deg)
            for _, pose, _ in window))
        if not steady:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = window[0][0]

    def _clearances(self, points):
        cfg = self.config
        bx = points[:, 0] + cfg.lidar_x_in_base_m
        by = points[:, 1]
        half_l, half_w = cfg.footprint_length_m / 2, cfg.footprint_width_m / 2
        ahead = (bx > half_l - .05) & (np.abs(by) <= half_w + .10)
        self.front_clearance = float(bx[ahead].min() - half_l) if ahead.any() else math.inf
        beside = (np.abs(bx) <= half_l) & (np.abs(by) > half_w - .05)
        self.side_clearance = float(np.abs(by[beside]).min() - half_w) if beside.any() else math.inf
        px = points[:, 0] + cfg.pivot_icr_behind_lidar_m
        self.pivot_clearance = float(np.hypot(px, points[:, 1]).min()) if len(points) else math.inf
        # Per-side split of the SAME beside band: min(left, right) == side_clearance exactly.
        self.side_left, self.side_left_x = _nearest_side(by, bx, beside & (by > 0), half_w)
        self.side_right, self.side_right_x = _nearest_side(-by, bx, beside & (by < 0), half_w)

    def pivot_radius_required(self, margin=.10):
        """Swept radius of the footprint about the uncertain pivot centre (nominal calibration)."""
        cfg = self.config
        icr_x = cfg.lidar_x_in_base_m - cfg.pivot_icr_behind_lidar_m
        reach = max(abs(cfg.footprint_length_m / 2 - icr_x), abs(-cfg.footprint_length_m / 2 - icr_x))
        return math.hypot(reach + cfg.pivot_icr_uncertainty_m, cfg.footprint_width_m / 2) + margin

    # --- corridor acquisition ------------------------------------------------
    def _acquire(self, points):
        """Constrained axis search: nearest band on each side, no intervening band.

        Returns (axis_laser, left, right) or None; sets corridor_reason.
        """
        cfg = self.config
        if self.last_band_heading is not None:
            prior, window = wrap(self.last_band_heading - self.pose[2]), cfg.reacquire_axis_window_deg
        else:
            prior, window = 0., cfg.acquire_axis_window_deg     # laser +x = current travel direction
        pts = points[np.hypot(points[:, 0], points[:, 1]) < cfg.support_range_m]
        if len(pts):
            pts = pts[_first_cell_indices(np.floor(pts / .1).astype(np.int32))]
        if len(pts) < 2 * cfg.acquire_min_coverage:
            self.corridor_reason = 'insufficient_returns'
            return None
        candidates, strengths = [], []
        for offset in np.arange(-window, window + 1e-9, cfg.acquire_axis_step_deg):
            axis = prior + math.radians(float(offset))
            found, strength = self._axis_pair(pts, axis)
            strengths.append((strength, axis))
            if found is not None:
                candidates.append(found)
        if not candidates:
            self.corridor_reason = 'no_nearest_band_pair_near_travel_axis'
            return None
        # The dominant (sharpest) line axis comes first; another clearly different axis
        # that is nearly as sharp is ambiguous. The pair must lie on the dominant axis:
        # a pair found only at a skewed axis (e.g. while the sharpest lines are vetoed
        # by a gapped row, or a diagonal of a square grid) is refused.
        gate = math.radians(cfg.acquire_ambiguity_deg)
        top, dominant = max(strengths)
        if any(strength >= cfg.acquire_ambiguity_ratio * top and abs(wrap(2 * (axis - dominant))) / 2 > gate
               for strength, axis in strengths):     # line axes compare modulo pi
            self.corridor_reason = 'ambiguous_corridor_axis'
            return None
        near = [c for c in candidates if abs(wrap(c['axis'] - dominant)) <= gate]
        if not near:
            self.corridor_reason = 'pair_not_on_dominant_axis'
            return None
        best = min(near, key=lambda c: (-c['score'], c['residual']))
        return best['axis'], best['left'], best['right']

    def _axis_pair(self, pts, axis):
        cfg = self.config
        c, s = math.cos(axis), math.sin(axis)
        along = pts @ np.array((c, s))
        cross = pts @ np.array((-s, c))
        sel = np.abs(along) <= cfg.acquire_along_range_m
        if cfg.acquire_behind_m is not None and self.last_band_heading is None:
            sel &= along >= -cfg.acquire_behind_m     # opt-in: first acquisition of a run, the corridor ahead
        along, cross = along[sel], cross[sel]
        if len(cross) < 2 * cfg.acquire_min_coverage:
            return None, 0
        cb = np.floor(cross / cfg.acquire_cross_bin_m).astype(int)
        ab = np.floor(along / cfg.acquire_along_bin_m).astype(int)
        lo = cb.min()
        # Occupancy grid (cross bin x along bin) instead of sorting unique keys: the
        # same distinct along-bin counts per cross bin, as exact integers.
        occupied = np.zeros((cb.max() - lo + 1, ab.max() - ab.min() + 1), dtype=bool)
        occupied[cb - lo, ab - ab.min()] = True
        cov = occupied.sum(axis=1).astype(int)
        # coverage of a 2-bin (0.4 m) wide band = distinct along bins in bin j-1 or j,
        # i.e. a band centred on the lower edge of bin j
        union = occupied.copy()
        union[1:] |= occupied[:-1]
        cov2 = union.sum(axis=1).astype(int)
        centres = (np.arange(len(cov)) + lo) * cfg.acquire_cross_bin_m

        def nearest(sign):
            # centres strictly increase, so argsort(sign * centres) is ascending (+1) or
            # descending (-1) index order: the first qualifying bin is the lowest/highest.
            qualifying = np.flatnonzero((sign * centres >= .5) & (cov2 >= cfg.acquire_min_coverage))
            for i in qualifying[:1] if sign > 0 else qualifying[-1:]:
                x = centres[i]
                # Snap outwards to the strongest band within 0.6 m (trunk line, not the
                # canopy fringe on the aisle side of it; not a fence just outside the row).
                near = (sign * centres >= sign * x) & (sign * centres <= sign * x + .6)
                j = int(np.flatnonzero(near)[np.argmax(cov2[near])])
                values = cross[np.abs(cross - centres[j]) <= .25]
                # np.median for finite data == middle of the sorted values ((a+b)/2 == (a+b)*.5);
                # the std is only needed for a returned pair (residual), so it is deferred.
                ordered = np.sort(values)
                middle = len(ordered) // 2
                median = ordered[middle] if len(ordered) % 2 else (ordered[middle - 1] + ordered[middle]) * .5
                return float(median), int(cov2[j]), values
            return None
        side_strength = [int(cov2[mask].max()) if mask.any() else 0
                         for mask in (centres >= .5, centres <= -.5)]
        strength = side_strength[0] + side_strength[1]
        left = nearest(1)
        if left is None:            # the right side is not needed (nearest has no side effects)
            return None, strength
        right = nearest(-1)
        if right is None:
            return None, strength
        width = left[0] - right[0]
        if not cfg.band_min_width_m <= width <= cfg.band_max_width_m:
            return None, strength
        # Veto: an intervening band (e.g. a gapped row) between the chosen pair.
        margin = max(cfg.acquire_branch_margin_m, cfg.acquire_inner_fraction * width)
        inner = (cross > right[0] + margin) & (cross < left[0] - margin)
        inner_bins = ab[inner]
        if (np.count_nonzero(np.bincount(inner_bins - inner_bins.min())) if len(inner_bins) else 0) >= cfg.acquire_veto_coverage:
            return None, strength   # returns between the lines (e.g. a gapped row): not this pair
        return dict(axis=axis, left=left[0], right=right[0], score=left[1] + right[1],
                    residual=float(np.std(left[2])) + float(np.std(right[2]))), strength

    # --- corridor tracking ---------------------------------------------------
    def _line_fit(self, points, axis, left, right):
        """Shared-slope LS fit of the returns near two laser-frame lines.

        Returns (axis, left, right); a side without enough support is None.
        Branch/trunk returns are only *selected* here, never deleted.
        """
        cfg = self.config
        c, s = math.cos(axis), math.sin(axis)
        along = points @ np.array((c, s))
        cross = points @ np.array((-s, c))
        near = np.abs(along) < 8.
        masks = [(np.abs(cross - offset) < cfg.line_gate_m) & near if offset is not None
                 else np.zeros(len(points), bool) for offset in (left, right)]
        sides = [k for k, m in enumerate(masks) if m.sum() >= cfg.line_min_points]
        if not sides:
            return axis, None, None
        a = np.concatenate([along[masks[k]] for k in sides])
        y = np.concatenate([cross[masks[k]] for k in sides])
        columns = [a] + [np.concatenate([np.full(int(masks[j].sum()), float(j == k)) for j in sides])
                         for k in sides]
        design = np.column_stack(columns)
        solution = None
        for _ in range(2):
            solution, *_ = np.linalg.lstsq(design, y, rcond=None)
            keep = np.abs(design @ solution - y) < .18
            if keep.sum() < cfg.line_min_points or any(design[keep, 1 + i].sum() < 4 for i in range(len(sides))):
                return axis, None, None
            design, y = design[keep], y[keep]
        slope = float(solution[0])
        if abs(slope) > math.tan(math.radians(4)):
            return axis, None, None
        angle = math.atan(slope)
        offsets = [None, None]
        for index, k in enumerate(sides):
            offsets[k] = float(solution[1 + index] * math.cos(angle))
        return axis + angle, offsets[0], offsets[1]

    def _side_extents(self, points, axis, left, right):
        """Per-side support end (laser along) and whether observed free space follows it.

        The end is the 97th percentile of line support inside support_range_m,
        excluding any structure that spans the aisle core. The side has ENDED
        ("free") only if the space after it, up to the next support on the same
        line, a cross-aisle structure, or the free_check_range_m visibility limit,
        is longer than end_free_m and than the robust (p75) in-row gap + margin.
        """
        cfg = self.config
        c, s = math.cos(axis), math.sin(axis)
        along = points @ np.array((c, s))
        cross = points @ np.array((-s, c))
        rng = np.hypot(points[:, 0], points[:, 1])
        inside = rng < cfg.support_range_m
        far = rng < cfg.free_check_range_m
        front = None
        if left is not None and right is not None:
            centre, half = (left + right) / 2, (left - right) / 2
            core_half = max(.3, half - cfg.core_margin_m)
            core = far & (np.abs(cross - centre) < core_half) & (along > .3)
            if core.sum() >= cfg.cross_structure_min_points:
                a_core, x_core = along[core], cross[core]
                # Same scan in ascending start order: the slab [start, start + .6] is the
                # sorted range [searchsorted(start, left), searchsorted(start + .6, right));
                # its ptp is max - min over that range (order free).
                order = np.argsort(a_core)
                a_sorted, x_sorted = a_core[order], np.append(x_core[order], 0.)
                first = np.searchsorted(a_sorted, a_sorted, 'left')
                last = np.searchsorted(a_sorted, a_sorted + .6, 'right')
                bounds = np.empty(2 * len(first), dtype=np.intp)
                bounds[0::2], bounds[1::2] = first, last
                spread = (np.maximum.reduceat(x_sorted, bounds)[0::2]
                          - np.minimum.reduceat(x_sorted, bounds)[0::2])
                hit = np.flatnonzero((last - first >= cfg.cross_structure_min_points)
                                     & (spread >= cfg.cross_structure_min_span * 2 * core_half))
                if len(hit):
                    front = float(a_sorted[hit[0]])
        continuation = cfg.row_end_continuation_m is not None
        per_side = continuation and cfg.row_end_continuation_rule == 'per_side'
        if continuation:
            # per side: v1 continuation bins of a side the rule ended / v2 objects of a two-sided end
            self.continuation_last = [None, None]
        cuts = (self._segment_cuts(along, cross, inside, far, front, (left, right))
                if cfg.row_end_segment_span_m is not None else (None, None))
        ends, free, needs = [], [], []
        for offset in (left, right):
            if offset is None:
                ends.append(None)
                free.append(False)
                needs.append(None)
                continue
            line = np.abs(cross - offset) < cfg.side_extent_tolerance_m
            if front is not None:
                line &= along < front - .3        # a cross-aisle structure is not row support
            mask = line & inside
            if mask.sum() < 4:
                ends.append(None)
                free.append(False)
                needs.append(None)
                continue
            a = np.sort(along[mask])
            cut = cuts[len(ends)]
            end = float(np.percentile(a, 97)) if cut is None else cut
            if cfg.row_end_last_trunk:
                end = _last_trunk_end(a, end)
            resumed = line & far & (along > end + .3)
            limit = math.sqrt(max(0., cfg.free_check_range_m ** 2 - offset ** 2))
            stop_at = min(limit, float(along[resumed].min()) if resumed.any() else math.inf,
                          front if front is not None else math.inf)
            gap = stop_at - end
            behind = a[(a >= -cfg.acquire_along_range_m) & (a <= end)]
            gaps = np.diff(behind)
            gaps = gaps[gaps > .3]
            # Open view: a robust (p75) in-row gap, so one missing trunk does not veto
            # every end. View closed by a cross-aisle structure (fence/wall): ambiguous
            # with a mid-row wall after missing trunks, so the gap must beat the
            # LARGEST in-row gap (a false row end is worse than a missed one).
            if not len(gaps):
                in_row = 0.
            elif front is not None and stop_at == front:
                in_row = float(gaps.max())
            else:
                in_row = float(np.percentile(gaps, 75))
            needed = max(cfg.end_free_m, in_row + cfg.end_gap_margin_m)
            ended = gap >= needed
            if ended and per_side:
                ended = not self._line_continues(len(ends), along, cross, far, offset, end, needed, front)
            ends.append(end)
            free.append(ended)
            needs.append(needed)
        if cfg.row_end_stagger_gap_m is not None and all(free) and None not in ends:
            ends = self._stagger_cut(along, cross, inside, front, (left, right), ends, needs)
        if continuation and not per_side and free[0] and free[1]:
            # Pair rule (v2): only a two-sided end of the original rule is examined, both sides together; it is
            # cancelled only when both line extensions continue (a one-sided view is never examined).
            objects = [continuation_objects(cfg, along, cross, rng, offset, end, needed, front)
                       for offset, end, needed in zip((left, right), ends, needs)]
            self.continuation_last = objects
            if min(objects) >= cfg.row_end_continuation_min_objects:
                free = [False, False]
        return ends, free, front

    def _line_continues(self, side, along, cross, far, offset, end, needed, front):
        """Opt-in row_end_continuation_m: does the row line of an ENDED side visibly continue beyond it?

        Returns inside a band around the line extension (half width band_m + band_slope x LiDAR along, at
        most band_max_m) with along in [end + needed, end + row_end_continuation_m), range below
        free_check_range_m and before a cross-aisle structure. They must fill >= min_bins distinct bin_m
        along bins (bins counted from end + needed). Only positive evidence counts: no returns there
        (occluded, not returned or observed free) keeps the original decision. Records the bin count."""
        cfg = self.config
        start, stop = end + needed, end + cfg.row_end_continuation_m
        band = np.minimum(cfg.row_end_continuation_band_max_m,
                          cfg.row_end_continuation_band_m + cfg.row_end_continuation_band_slope
                          * np.maximum(0., along))
        on = far & (np.abs(cross - offset) < band) & (along >= start) & (along < stop)
        if front is not None:
            on &= along < front - .3                  # as the original rule: a wall is not row support
        bins = len(np.unique(np.floor((along[on] - start) / cfg.row_end_continuation_bin_m))) if on.any() else 0
        self.continuation_last[side] = bins
        return bins >= cfg.row_end_continuation_min_bins

    # --- opt-in row_end_stagger_gap_m (never called while OFF) -----------------------------------------
    def _stagger_cut(self, along, cross, inside, front, offsets, ends, needs):
        """Both sides ended, one end more than row_end_stagger_window_m beyond the other: the farther side ends at its
        support before the first free gap of >= max(row_end_stagger_gap_m, its required gap) that reaches ahead of the
        LiDAR and starts within row_end_stagger_window_m of the nearer side's end, when the support beyond that gap (up
        to the farther end) is not row-like. Otherwise the ends are returned unchanged."""
        cfg = self.config
        far = 0 if ends[0] > ends[1] else 1
        near_end, far_end = ends[1 - far], ends[far]
        if far_end - near_end <= cfg.row_end_stagger_window_m:
            return ends
        line = np.abs(cross - offsets[far]) < cfg.side_extent_tolerance_m
        if front is not None:
            line &= along < front - .3            # a cross-aisle structure is not row support
        a = np.sort(along[line & inside & (along <= far_end + .3)])
        needed = max(cfg.row_end_stagger_gap_m, needs[far])
        for i in range(len(a) - 1):
            if a[i + 1] - a[i] < needed or a[i + 1] <= 0. or abs(a[i] - near_end) > cfg.row_end_stagger_window_m:
                continue
            if self._row_like(a[i + 1:]):
                return ends                       # the far line continues as a row beyond the gap: unchanged
            end = float(np.percentile(a[:i + 1], 97))
            if cfg.row_end_last_trunk:
                end = _last_trunk_end(a[:i + 1], end)
            out = list(ends)
            out[far] = end
            return out
        return ends

    # --- opt-in row_end_segment_span_m (never called while OFF) ---------------------------------------
    def _segment_cuts(self, along, cross, inside, far, front, offsets):
        """Per side: the support end before the side's first qualifying free gap, or None (original rule).

        A gap qualifies when it reaches ahead of the LiDAR (its far return has along > 0) and is at least
        row_end_segment_min_gap_m and the side's required free gap from the in-row gaps before it (the original
        needed rule: max(end_free_m, p75 + end_gap_margin_m)); the end is the 97th percentile of the support before it.
        A side without such a gap keeps the original end (97th percentile of its support). When the support beyond
        the end of BOTH sides (cut or original, up to free_check_range_m) is row-like (_row_like), it is a cross gap:
        no cut on either side."""
        cfg = self.config
        found = []
        for offset in offsets:
            if offset is None:
                found.append(None)
                continue
            line = np.abs(cross - offset) < cfg.side_extent_tolerance_m
            if front is not None:
                line &= along < front - .3        # a cross-aisle structure is not row support
            a = np.sort(along[line & inside])
            if len(a) < 4:
                found.append(None)
                continue
            cut, end, prior = None, float(np.percentile(a, 97)), []
            for i in range(len(a) - 1):
                gap = float(a[i + 1] - a[i])
                if gap <= .3:
                    continue
                if a[i + 1] > 0.:
                    in_row = float(np.percentile(prior, 75)) if prior else 0.
                    if gap >= max(cfg.row_end_segment_min_gap_m, cfg.end_free_m, in_row + cfg.end_gap_margin_m):
                        cut = end = float(np.percentile(a[:i + 1], 97))
                        break
                prior.append(gap)
            beyond = np.sort(along[line & far & (along > end + .3)])
            found.append((cut, self._row_like(beyond)))
        if all(f is not None and f[1] for f in found):
            return None, None                     # both rows continue beyond: a cross gap, the original rule
        return tuple(None if f is None else f[0] for f in found)

    def _row_like(self, alongs):
        """Along positions beyond an end look like a continuing row line: >= row_end_segment_min_bins distinct
        row_end_segment_bin_m along bins with >= 2 returns, the first to the last of them spanning >=
        row_end_segment_span_m. A post, bush, bin or a fence crossing the line band fills one or two bins."""
        cfg = self.config
        if len(alongs) < 2 * cfg.row_end_segment_min_bins:
            return False
        bins, counts = np.unique(np.floor(np.asarray(alongs) / cfg.row_end_segment_bin_m).astype(np.int64),
                                 return_counts=True)
        filled = bins[counts >= 2]
        if len(filled) < cfg.row_end_segment_min_bins:
            return False
        return float(filled[-1] - filled[0] + 1) * cfg.row_end_segment_bin_m >= cfg.row_end_segment_span_m

    def _next_corridor_votes(self, points, axis, left, right, width):
        """Per side: is a parallel line one width beyond the side line observed?"""
        cfg = self.config
        c, s = math.cos(axis), math.sin(axis)
        along = points @ np.array((c, s))
        cross = points @ np.array((-s, c))
        rng = np.hypot(points[:, 0], points[:, 1])
        for name, offset, sign in (('left', left, 1.), ('right', right, -1.)):
            z0, z1 = sign * offset + .5 * width, sign * offset + 1.5 * width   # distance beyond, signed side
            sc = sign * cross
            # Only alongside the observed part of THIS corridor's line: beyond the
            # row start/end the absence of a next row says nothing.
            own = (np.abs(cross - offset) < cfg.side_extent_tolerance_m) & (rng < cfg.support_range_m)
            if own.sum() < 4:
                continue
            lo = max(-cfg.acquire_along_range_m, float(along[own].min()))
            hi = min(cfg.acquire_along_range_m, float(along[own].max()))
            if hi - lo < 2.:
                continue
            zone = (sc > z0) & (sc < z1) & (along >= lo) & (along <= hi) & (rng < cfg.support_range_m)
            present = False
            if zone.sum() >= cfg.next_min_coverage:
                expected = sign * offset + width
                near = zone & (np.abs(sc - expected) < .5 * width - .1)
                if near.sum():
                    peak = float(np.median(sc[near]))
                    band = zone & (np.abs(sc - peak) < cfg.next_line_gate_m)
                    present = len(np.unique(np.floor(along[band] / .75))) >= cfg.next_min_coverage
            if present:
                self.next_votes[name][0] += 1
                continue
            # Observed through the zone: rays (incl. no-return) that reach its far edge.
            angles = self._scan_angles
            ranges = self._scan_ranges
            direction = sign * np.sin(angles - axis)
            usable = direction > .2
            reach = z1 / np.maximum(direction, 1e-6)
            along_at = reach * np.cos(angles - axis)
            through = usable & ((~np.isfinite(ranges)) | (ranges <= 0) | (ranges >= reach)) \
                & (along_at >= lo) & (along_at <= hi) & (reach < cfg.support_range_m + 2)
            zone_hits = int(zone.sum())
            if int(through.sum()) >= cfg.next_min_observed_rays and zone_hits < cfg.next_min_coverage:
                self.next_votes[name][1] += 1

    def _world(self, axis, left, right):
        """Laser-frame lines -> world heading oriented with travel; offsets follow."""
        heading = wrap(self.pose[2] + axis)
        reference = self.band.heading if self.band is not None else self.reference_heading
        if abs(wrap(heading - reference)) > math.pi / 2:
            heading, axis = wrap(heading + math.pi), wrap(axis + math.pi)
            left, right = (None if right is None else -right), (None if left is None else -left)
        return heading, axis, left, right

    def _detected_lines(self, observation, points):
        axis, left, right = self._line_fit(points, observation.heading_axis_laser_rad,
                                           observation.left_offset_m, observation.right_offset_m)
        if left is None or right is None:
            axis, left, right = (observation.heading_axis_laser_rad, observation.left_offset_m,
                                 observation.right_offset_m)
        return self._world(axis, left, right)

    def _corridor(self, stamp, scan, points):
        r = np.asarray(scan.ranges, dtype=float)
        self._scan_ranges = r
        self._scan_angles = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        if self.band is None:
            found = self._acquire(points)
            if found is None:
                self.candidates = []
                return
            axis, left, right = self._line_fit(points, *found)
            if left is None or right is None:
                axis, left, right = found
            self._initialise(stamp, *self._world(axis, left, right), points)
            return
        observation = observe_scan_corridor(scan)
        if observation.valid and self._apply(stamp, *self._detected_lines(observation, points),
                                             points, 'candidate'):
            return
        # Detector ambiguous/inconsistent: fit raw returns near the PREDICTED lines.
        band = self.band
        x, y, theta = self.pose
        normal = np.array((-math.sin(band.heading), math.cos(band.heading)))
        lateral = float(np.array((x, y)) @ normal)
        axis, left, right = self._line_fit(points, wrap(band.heading - theta),
                                           band.center + band.width / 2 - lateral,
                                           band.center - band.width / 2 - lateral)
        if left is None and right is None:
            self.corridor_reason = 'no_support_near_tracked_lines'
            if self.quality is not None:
                self.quality.rejected()
            return
        if not self._apply(stamp, wrap(theta + axis), axis, left, right, points, 'guided'):
            self.corridor_reason = 'inconsistent_corridor_rejected'
            if self.quality is not None:
                self.quality.rejected()

    def _support_counts(self, points, axis, left, right):
        """Opt-in corridor_quality: returns within line_gate_m of each laser-frame line (|along| < 8 m)."""
        c, s = math.cos(axis), math.sin(axis)
        along = points @ np.array((c, s))
        cross = points @ np.array((-s, c))
        near = np.abs(along) < 8.
        gate = self.config.line_gate_m
        return [0 if offset is None else int(np.count_nonzero((np.abs(cross - offset) < gate) & near))
                for offset in (left, right)]

    def _apply(self, stamp, heading, axis, left, right, points, source):
        """Gated full (both sides) or one-sided update of the tracked band."""
        cfg, band = self.config, self.band
        x, y, _ = self.pose
        if abs(wrap(heading - band.heading)) >= math.radians(cfg.band_heading_gate_deg):
            return False
        normal = np.array((-math.sin(band.heading), math.cos(band.heading)))
        lateral = float(np.array((x, y)) @ normal)
        pred_left = band.center + band.width / 2 - lateral
        pred_right = band.center - band.width / 2 - lateral
        along0 = float(np.array((x, y)) @ np.array((math.cos(heading), math.sin(heading))))
        g, gc = cfg.band_gain, cfg.band_center_gain
        if left is not None and right is not None:
            width = left - right
            offset_pred = -(pred_left + pred_right) / 2
            offset_obs = -(left + right) / 2
            if (abs(offset_obs - offset_pred) < cfg.band_offset_gate_m
                    and abs(width - band.width) < cfg.band_width_gate_m
                    and cfg.band_min_width_m <= width <= cfg.band_max_width_m):   # no slow drift to 0
                if self.quality is not None:
                    self.quality.accepted(stamp, 'both', source, self._support_counts(points, axis, left, right),
                                          lateral=offset_obs - offset_pred, heading=wrap(heading - band.heading),
                                          width=width - band.width)
                band.heading = wrap(band.heading + g * wrap(heading - band.heading))
                band.width += g * (width - band.width)
                normal = np.array((-math.sin(band.heading), math.cos(band.heading)))
                lateral = float(np.array((x, y)) @ normal)
                band.center = lateral - ((1 - gc) * offset_pred + gc * offset_obs)
                band.stamp = stamp
                band.full_updates += 1
                self.accepted_updates += 1
                self.known_width = band.width
                self.last_band_heading = band.heading
                self._update_ends(band, self._side_extents(points, axis, left, right), along0, stamp,
                                  (heading, left, right))
                self._next_corridor_votes(points, axis, left, right, band.width)
                self.corridor_reason = 'tracked_full_' + source
                return True
        # One-sided: the matched line keeps the learnt width.
        for observed, predicted, sign in ((left, pred_left, -1), (right, pred_right, 1)):
            if observed is not None and abs(observed - predicted) < cfg.band_side_gate_m:
                if self.quality is not None:
                    self.quality.accepted(stamp, 'left' if sign < 0 else 'right', source,
                                          self._support_counts(points, axis, left, right),
                                          lateral=observed - predicted, heading=wrap(heading - band.heading))
                relative = band.center - lateral          # opt-in: the centre offset from the vehicle before the turn
                band.heading = wrap(band.heading + g / 2 * wrap(heading - band.heading))
                normal = np.array((-math.sin(band.heading), math.cos(band.heading)))
                lateral = float(np.array((x, y)) @ normal)
                center_obs = lateral + observed + sign * band.width / 2
                if cfg.band_rotate_about_vehicle:
                    band.center = lateral + relative      # rotated about the vehicle, as the full update does
                band.center += gc * (center_obs - band.center)
                band.stamp = stamp
                self.accepted_updates += 1
                # A one-sided view never yields or extends a row end (the other side is
                # unknown); the last two-sided evidence simply ages out (row_end_valid_s).
                if cfg.row_end_position_confirm_m is not None and self.end_position_memory is not None:
                    # Opt-in: it may only CONTRADICT the remembered end of its side (never confirm).
                    self._one_sided_end_check(band, heading, axis, observed, 0 if sign < 0 else 1, points)
                self.corridor_reason = 'tracked_one_side_' + source
                return True
        return False

    def _update_ends(self, band, extents, along0, stamp, lines=None):
        """Row-end evidence comes ONLY from full (two-sided) updates.

        lines = (world heading, laser left offset, laser right offset) of this update; it is read only
        by the opt-in row_end_position_* bridge.
        """
        cfg = self.config
        ends, free, _ = extents
        if cfg.row_end_continuation_m is not None:
            self._count_continuation(ends, free, stamp)
        recent = band.end_stamp is not None and stamp - band.end_stamp <= cfg.row_end_valid_s
        both = all(free) and None not in ends
        if cfg.row_end_position_confirm_m is not None and both and self.end_streak and not recent:
            # end_streak > 0: the previous full update was two-sided ended (any other full update resets
            # it), but older than row_end_valid_s. Bridge only on position consistency (guards inside).
            recent = self._end_position_bridge(band, ends, stamp, lines)
        self.end_streak = (self.end_streak + 1 if recent else 1) if both else 0
        band.end_left = None if ends[0] is None else along0 + ends[0]
        band.end_right = None if ends[1] is None else along0 + ends[1]
        band.end_stamp = stamp
        self.end_free = list(free)
        if cfg.row_end_position_confirm_m is not None:
            # pose = the LiDAR viewpoint (odometry frame) of this observation: the travel guard reads the
            # NET viewpoint displacement, not the accumulated path (self.distance), which also grows with
            # scan-match jitter while stopped and with the LiDAR arc of a pivot.
            self.end_position_memory = (dict(band=band, stamp=stamp, points=self._end_points(lines, ends),
                                             odom_failures=self.odom_failures,
                                             pose=(float(self.pose[0]), float(self.pose[1])))
                                        if both and lines is not None else None)

    # --- opt-in row_end_continuation_m (never called while OFF) ------------------------------------
    def _count_continuation(self, ends, free, stamp):
        """Evaluation counts of one full update (no truth): sides the original rule ended and that were
        examined, sides vetoed, and two-sided ends of the original rule that the veto cancelled."""
        bins, self.continuation_last = self.continuation_last, None   # from this update's _side_extents
        if bins is None:
            return
        examined = [b is not None for b in bins]
        vetoed = [b is not None and not f for b, f in zip(bins, free)]
        counts = self.continuation_counts
        counts['checked'] += sum(examined)
        counts['vetoed'] += sum(vetoed)
        pair = self.config.row_end_continuation_rule == 'pair'
        if pair and all(examined) and not any(vetoed):
            # Pair rule: a kept two-sided end with continuation evidence on exactly one side (v1 would veto it).
            counts['one_sided'] += int(sum(b >= self.config.row_end_continuation_min_objects for b in bins) == 1)
        if all(examined) and any(vetoed):            # the original rule had both sides ended here
            counts['cancelled'] += 1
            if len(self.continuation_events) < 20:
                self.continuation_events.append(dict(stamp=round(stamp, 3), ends=[round(e, 3) for e in ends],
                                                     **{'objects' if pair else 'bins': list(bins)}))

    # --- opt-in row_end_position_* bridge (never called while OFF) ---------------------------------
    def _end_points(self, lines, ends):
        """World (odometry-frame) end point of each side: on its line, at its along end."""
        heading, *offsets = lines
        c, s = math.cos(heading), math.sin(heading)
        x, y = float(self.pose[0]), float(self.pose[1])
        return tuple(None if end is None or offset is None else (x + c * end - s * offset, y + s * end + c * offset)
                     for end, offset in zip(ends, offsets))

    def _end_disagreement(self, band, point, remembered):
        """(|along|, |across|) difference of two end points, on the CURRENT band axis."""
        c, s = math.cos(band.heading), math.sin(band.heading)
        dx, dy = point[0] - remembered[0], point[1] - remembered[1]
        return abs(c * dx + s * dy), abs(-s * dx + c * dy)

    def _viewpoint_travel(self, band, remembered):
        """|net LiDAR viewpoint displacement since the remembered observation . CURRENT band axis| [m].

        Net, not the odometry path: path accrues with scan-match jitter while stopped and along the arc
        of the LiDAR in a pivot without changing the viewpoint. Along the band axis, not euclidean: the
        compared quantity is the along-row end position; a sideways move (centring, a pivot arc across the
        aisle) adds no along-row viewpoint change. The projection is never larger than the euclidean
        displacement, so this guard is the stricter one.
        Scope (review of 2cb0f4e): this only excludes agreement WITHOUT along-row viewpoint change
        (stopped, jitter, sideways). It does NOT discriminate visibility artefacts in general: an end is
        the 97th-percentile along-row support point, i.e. it sits on the last visible trunk or on an
        occluder surface and stays fixed in the world until another trunk appears or disappears, so an
        occlusion- or range-limit-made false end can still agree after > tolerance travel (reproduced in
        taskD_rowend/review_fix). Such ends also pass the original 2.5 s rule."""
        c, s = math.cos(band.heading), math.sin(band.heading)
        return abs(c * (float(self.pose[0]) - remembered[0]) + s * (float(self.pose[1]) - remembered[1]))

    def _end_position_bridge(self, band, ends, stamp, lines):
        cfg = self.config
        memory = self.end_position_memory
        tolerance = cfg.row_end_position_confirm_m
        detail = None
        if memory is None or memory['band'] is not band or lines is None:
            reason = 'no_memory'            # a new band starts without a previous observation
        elif stamp - memory['stamp'] > cfg.row_end_position_max_gap_s:
            reason = 'gap'
        elif self.odom_failures != memory['odom_failures']:
            reason = 'odometry'             # a failed scan match in the gap (incl. this scan): frame link weak
        elif self._viewpoint_travel(band, memory['pose']) <= tolerance:
            # Agreement counts only from a viewpoint that moved more than the tolerance ALONG the band
            # (excludes agreement while stopped/jittering/moving sideways; it does not discriminate
            # occlusion or range-limit ends that stay fixed in the world, see _viewpoint_travel).
            reason = 'travel'
        else:
            differences = [self._end_disagreement(band, point, previous)
                           for point, previous in zip(self._end_points(lines, ends), memory['points'])]
            along = max(d[0] for d in differences)
            across = max(d[1] for d in differences)
            reason = 'along' if along > tolerance else 'lateral' if across > tolerance else 'bridged'
            detail = (along, across)
        self.end_position_counts[reason] += 1
        if reason == 'bridged' and len(self.end_position_events) < 20:
            self.end_position_events.append(dict(stamp=round(stamp, 3), gap_s=round(stamp - memory['stamp'], 3),
                                                 along_m=round(detail[0], 4), across_m=round(detail[1], 4)))
        return reason == 'bridged'

    def _one_sided_end_check(self, band, heading, axis, offset, side, points):
        """A one-sided view whose side support continues, or ends elsewhere, clears the memory."""
        memory = self.end_position_memory
        if memory['band'] is not band:
            return
        offsets = (offset, None) if side == 0 else (None, offset)
        ends, free, _ = self._side_extents(points, axis, *offsets)
        if ends[side] is None:
            return                          # no support of this side in range: says nothing
        point = self._end_points((heading,) + offsets, ends)[side]
        along, across = self._end_disagreement(band, point, memory['points'][side])
        tolerance = self.config.row_end_position_confirm_m
        if not free[side] or along > tolerance or across > tolerance:
            self.end_position_memory = None
            self.end_position_counts['contradicted'] += 1

    def _initialise(self, stamp, heading, axis, left, right, points):
        cfg = self.config
        width = left - right
        if (self.known_width is not None and self.band_lost_at_m is not None
                and self.distance - self.band_lost_at_m > cfg.known_width_forget_m):
            self.known_width = None      # another area: rows may have a different width
            self.last_band_heading = None
        if not cfg.band_min_width_m <= width <= cfg.band_max_width_m or (
                self.known_width is not None and abs(width - self.known_width) > cfg.band_width_gate_m):
            self.candidates = []
            self.corridor_reason = 'implausible_initial_width'
            return
        x, y, _ = self.pose
        normal = np.array((-math.sin(heading), math.cos(heading)))
        center = float(np.array((x, y)) @ normal) + (left + right) / 2
        if self.candidates:
            h, c, w, _ = self.candidates[-1]
            if (abs(wrap(heading - h)) > math.radians(cfg.band_heading_gate_deg)
                    or abs(center - c) > .4 or abs(width - w) > cfg.band_width_gate_m):
                self.candidates = []
        self.candidates.append((heading, center, width, stamp))
        self.corridor_reason = 'initialising'
        if len(self.candidates) >= cfg.band_init_count:
            headings = [h for h, *_ in self.candidates]
            mean_heading = math.atan2(sum(map(math.sin, headings)), sum(map(math.cos, headings)))
            self.band = Band(mean_heading, float(np.mean([c for _, c, _, _ in self.candidates])),
                             float(np.mean([w for _, _, w, _ in self.candidates])), stamp, 1)
            self.known_width = self.band.width
            self.last_band_heading = self.band.heading
            self.accepted_updates += 1
            along0 = float(np.array((x, y)) @ np.array((math.cos(heading), math.sin(heading))))
            self._update_ends(self.band, self._side_extents(points, axis, left, right), along0, stamp,
                              (heading, left, right))
            self.candidates = []
            self.corridor_reason = 'tracked_full_init'
            if self.quality is not None:
                self.quality.accepted(stamp, 'both', 'init', self._support_counts(points, axis, left, right))

    # --- opt-in external corridor observations (never called into while OFF) --------------------------------
    def _base_pose_at(self, stamp):
        """Base pose (x, y, yaw) in the odometry frame at `stamp`, or None: interpolated between the recent scan poses
        (about the last second), or advanced from the newest scan with the gyro and the LiDAR speed."""
        if self.last_scan_stamp is None or not self.history:
            return None
        cfg = self.config
        if stamp >= self.last_scan_stamp:
            theta = wrap(self.pose[2] + self.gyro_integral(self.last_scan_stamp, stamp))
            lx, ly = advance(self.pose, theta, self.speed, stamp - self.last_scan_stamp)
        else:
            entries = list(self.history)
            if stamp < entries[0][0]:
                return None
            k = max(i for i, entry in enumerate(entries) if entry[0] <= stamp)
            (t0, p0, _), (t1, p1, _) = entries[k], entries[min(k + 1, len(entries) - 1)]
            u = 0. if t1 <= t0 else (stamp - t0) / (t1 - t0)
            lx, ly = p0[0] + u * (p1[0] - p0[0]), p0[1] + u * (p1[1] - p0[1])
            theta = wrap(p0[2] + u * wrap(p1[2] - p0[2]))
        return (lx - cfg.lidar_x_in_base_m * math.cos(theta), ly - cfg.lidar_x_in_base_m * math.sin(theta), theta)

    def on_corridor_observation(self, stamp, lateral_offset_m, heading_error_rad, width_m=None, quality=1.,
                                source='external'):
        """Opt-in provider slot (external_corridor_gain > 0): a corridor observation made at `stamp` (estimator clock)
        in the snapshot's own convention - lateral_offset_m = base offset from the corridor centre along the corridor's
        left normal, heading_error_rad = corridor axis - vehicle heading, width_m (optional), quality in [0, 1] and the
        provider name. Returns the outcome (EXTERNAL_REASONS: 'fused' or why it was not used); None while OFF.

        A used observation pulls the tracked band by the gain: the heading about the vehicle, then the centre, then
        the width. It never creates a band, never refreshes the band's own stamp (the band still times out without
        LiDAR support), never counts as a corridor update and never touches the row-end evidence."""
        book = self.external
        if book is None:
            return None
        cfg = self.config
        if source not in cfg.external_corridor_sources:
            return book.note('source')
        values = (stamp, lateral_offset_m, heading_error_rad, quality) + (() if width_m is None else (width_m,))
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
            return book.note('invalid')
        if book.last_stamp is not None and stamp <= book.last_stamp:
            return book.note('order')
        book.last_stamp = stamp
        if quality < cfg.external_corridor_min_quality:
            return book.note('quality')
        stamps = [t for t in (self.last_scan_stamp, self.last_imu_stamp) if t is not None]
        if not stamps or self.band is None:
            return book.note('no_band')
        newest = max(stamps)
        if newest - stamp > cfg.external_corridor_max_age_s:
            return book.note('stale')
        if stamp - newest > cfg.external_corridor_max_age_s:
            return book.note('future')
        pose = self._base_pose_at(stamp)
        if pose is None:
            return book.note('no_pose')
        band, gain = self.band, cfg.external_corridor_gain
        bx, by, theta = pose
        turn = wrap(wrap(theta + heading_error_rad) - band.heading)
        if abs(turn) >= math.radians(cfg.band_heading_gate_deg):
            return book.note('heading_gate')
        lateral = bx * -math.sin(band.heading) + by * math.cos(band.heading)
        if abs((lateral - band.center) - lateral_offset_m) >= cfg.band_offset_gate_m:
            return book.note('offset_gate')
        if width_m is not None and not (abs(width_m - band.width) < cfg.band_width_gate_m
                                        and cfg.band_min_width_m <= width_m <= cfg.band_max_width_m):
            return book.note('width_gate')
        relative = band.center - lateral                 # the centre's offset from the vehicle before the turn
        band.heading = wrap(band.heading + gain * turn)
        lateral = bx * -math.sin(band.heading) + by * math.cos(band.heading)
        band.center = lateral + relative                 # rotated about the vehicle
        band.center += gain * ((lateral - lateral_offset_m) - band.center)
        if width_m is not None:
            band.width += gain * (width_m - band.width)
        book.fused_stamp, book.fused_source = stamp, source
        return book.note('fused')

    # --- outputs ------------------------------------------------------------
    def still_duration(self, now):
        """Seconds of sensor-observed stillness (0 when moving, stale or not steady)."""
        if self.still_since is None or self.last_scan_stamp is None or now - self.last_scan_stamp >= .5:
            return 0.
        if abs(self.gyro_lp - self.gyro_bias) > math.radians(self.config.still_yaw_deg_s):
            return 0.
        return max(0., self.last_scan_stamp - self.still_since)

    def is_still(self, now, minimum_s=.4):
        if self.still_since is None or self.last_scan_stamp is None:
            return False
        return self.still_duration(now) >= minimum_s and now - self.last_scan_stamp < .5

    def next_corridor(self):
        cfg = self.config
        result = {}
        for side, (present, absent) in self.next_votes.items():
            if present >= cfg.next_present_votes:
                result[side] = 'PRESENT'
            elif absent >= cfg.next_absent_votes and present == 0:
                result[side] = 'ABSENT'
            else:
                result[side] = 'UNKNOWN'
        return result

    def base_pose(self, now):
        """Latency-compensated base pose (x, y, yaw) in the odometry frame at `now`: the pose the snapshot's corridor
        values refer to (the newest scan pose advanced by the gyro and by the LiDAR speed, or by the bounded actuator
        prior). Display and evaluation only; nothing in the estimator reads it back."""
        cfg = self.config
        theta = self.pose[2]
        if self.last_scan_stamp is not None:
            theta = wrap(theta + self.gyro_integral(self.last_scan_stamp, now))
        horizon = 0. if self.last_scan_stamp is None else max(0., now - self.last_scan_stamp)
        ahead = (None if self.prior is None or self.last_scan_stamp is None
                 else self.prior.extrapolate(self.last_scan_stamp, now, self.speed))
        if ahead is None:
            lx, ly = advance(self.pose, theta, self.speed, horizon)
        else:
            lx, ly = advance(self.pose, theta, ahead[0], 1.)
        return (float(lx - cfg.lidar_x_in_base_m * math.cos(theta)),
                float(ly - cfg.lidar_x_in_base_m * math.sin(theta)), float(theta))

    def snapshot(self, now):
        cfg = self.config
        lidar_age = math.inf if self.last_scan_stamp is None else now - self.last_scan_stamp
        imu_age = math.inf if self.last_imu_stamp is None else now - self.last_imu_stamp
        theta = self.pose[2]
        if self.last_scan_stamp is not None:
            theta = wrap(theta + self.gyro_integral(self.last_scan_stamp, now))
        # Latency compensation: advance the scan pose with gyro and LiDAR speed
        # along the wrapped mid-heading (no naive mean across +-pi).
        horizon = 0. if self.last_scan_stamp is None else max(0., now - self.last_scan_stamp)
        # Opt-in actuator prior (b): the bounded, blended travel since the newest scan; None (OFF, weight 0 or no
        # status over that time) keeps the original LiDAR-speed rule below, bit for bit.
        ahead = (None if self.prior is None or self.last_scan_stamp is None
                 else self.prior.extrapolate(self.last_scan_stamp, now, self.speed))
        if ahead is None:
            lx, ly = advance(self.pose, theta, self.speed, horizon)
            front_clearance = self.front_clearance - max(0., self.speed) * horizon
        else:
            lx, ly = advance(self.pose, theta, ahead[0], 1.)
            front_clearance = self.front_clearance - max(0., ahead[0])
        bx = lx - cfg.lidar_x_in_base_m * math.cos(theta)
        by = ly - cfg.lidar_x_in_base_m * math.sin(theta)
        still_s = self.still_duration(now)
        result = dict(lidar_age_s=lidar_age, imu_age_s=imu_age, yaw_rate=self.yaw_rate(),
                      gyro_heading=self.gyro_heading, gyro_bias=self.gyro_bias,
                      speed=self.speed, odom_valid=self.odom_valid, lidar_heading=float(self.pose[2]),
                      distance_m=self.distance, still=still_s > 0., still_s=still_s,
                      vibration_rms=self.vibration_rms(now),
                      front_clearance_m=front_clearance,
                      side_clearance_m=self.side_clearance,
                      side_clearance_left_m=self.side_left, side_clearance_right_m=self.side_right,
                      side_nearest_x_left_m=self.side_left_x, side_nearest_x_right_m=self.side_right_x,
                      rotation_centre_x_m=cfg.lidar_x_in_base_m - cfg.pivot_icr_behind_lidar_m,   # NOMINAL
                      rotation_centre_sd_m=cfg.pivot_icr_uncertainty_m, scan_count=self.scan_count,
                      pivot_clearance_m=self.pivot_clearance,
                      pivot_radius_required_m=self.pivot_radius_required(),
                      corridor_valid=False, corridor_reason=self.corridor_reason,
                      corridor_updates=self.accepted_updates, next_corridor=self.next_corridor())
        band = self.band
        if band is not None:
            normal = (-math.sin(band.heading), math.cos(band.heading))
            direction = (math.cos(band.heading), math.sin(band.heading))
            lateral = bx * normal[0] + by * normal[1] - band.center
            along = bx * direction[0] + by * direction[1]
            ends = [None if e is None else e - along for e in (band.end_left, band.end_right)]
            # A side is "ended" only when its support stops inside the sensed
            # range and observed free space follows it (not a cross-aisle wall),
            # on BOTH sides; the corridor ends where the LATER of the two ends
            # (staggered rows legitimately end a part-spacing apart).
            limit = cfg.support_range_m - 1.5 - cfg.lidar_x_in_base_m
            fresh = band.end_stamp is not None and now - band.end_stamp <= cfg.row_end_valid_s
            both_end = (fresh and self.end_streak >= cfg.row_end_confirm_updates
                        and all(e is not None and e < limit and f for e, f in zip(ends, self.end_free)))
            result.update(corridor_valid=True, heading_error=wrap(band.heading - theta),
                          lateral_offset=lateral, corridor_width=band.width,
                          corridor_age_s=now - band.stamp, corridor_full_updates=band.full_updates,
                          side_ends_ahead_m=ends, side_end_free=list(self.end_free),
                          row_end_ahead_m=max(ends) if both_end else None,
                          end_age_s=None if band.end_stamp is None else now - band.end_stamp)
        if self.prior is not None:
            result['actuator_prior'] = self.prior.snapshot(now, self.last_scan_stamp, self.speed)
        if self.quality is not None:
            result['corridor_quality'] = self.quality.snapshot(now, band)
        if self.external is not None:
            result['external_corridor'] = self.external.snapshot(now)
        return result


def _nearest_side(lateral, bx, mask, half_width):
    """Nearest return of one side of the beside band: (clearance [m], base x [m]) or (inf, None)."""
    if not mask.any():
        return math.inf, None
    lateral, bx = lateral[mask], bx[mask]
    i = int(np.argmin(lateral))
    return float(lateral[i] - half_width), float(bx[i])


def advance(pose, theta, speed, horizon):
    """Advance the scan pose by speed*horizon along the WRAPPED mid-heading (pose yaw -> theta)."""
    mid = wrap(pose[2] + .5 * wrap(theta - pose[2]))
    return pose[0] + math.cos(mid) * speed * horizon, pose[1] + math.sin(mid) * speed * horizon


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array((a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])))


def _relative(a, b):
    """Pose b expressed in the frame of pose a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2]))


class _Scan:
    __slots__ = ('ranges', 'angle_min', 'angle_increment', 'range_min', 'range_max')

    def __init__(self, ranges, angle_min, increment, range_max):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = increment
        self.range_min = .05
        self.range_max = range_max
