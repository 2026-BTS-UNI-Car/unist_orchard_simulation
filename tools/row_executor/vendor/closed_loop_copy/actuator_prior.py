"""Actuator priors of the row estimator (layer L1; design v2 V2-1 item 2, decision R31). Opt-in, default OFF.

Inputs, nothing else: the applied drive duty and the calibrated steering angle that the MCU reported in a status line,
with the time of that line on the estimator clock. Never the request alone and never a simulator value. PWM is not an
encoder and the potentiometer measures the steering link, not a wheel: both give PRIORS that are bounded in time and
distance. LiDAR and raw IMU stay the main input of the corridor position and of the row end.

  speed prior      v_p = table(|duty|) x speed_scale, signed like the duty. table = (duty %, m/s) points, piecewise
                   linear, through (0, 0) below the first point, held above the last one. The duty is the mean of the
                   two sides (the steered vehicle drives both with the same duty).
  yaw-rate prior   r_p = v_p x tan(steering angle) / wheelbase; None without a steering angle or a wheelbase.
  hold             a status sample counts from its stamp until the next sample, at most status_max_age_s: an older
                   status line gives no prior and the estimator falls back to LiDAR + IMU only.
Uses (applied by the estimator; weight 0 = monitor and record only, the estimate itself is then unchanged):
  (a) seed         scan-match translation seed = (1 - weight) x LiDAR speed x dt + weight x prior displacement. The
                   rotation seed stays the gyro.
  (b) prediction   a failed scan match, a scan gap and the time since the newest scan: weight x prior displacement,
                   for at most predict_max_s since the last scan-matched pose or predict_max_m, whichever comes first.
                   After that nothing more is added and 'limit' is reported (for the executor's prediction_limit pause, PC-06).
  (c) monitor      online scale = LiDAR step / prior displacement over the last monitor_window_s of driving. The table
                   is flagged (table_mismatch) when the table error |prior / LiDAR - 1| exceeds mismatch_fraction with
                   at least monitor_min_s of samples in the window. The flag alone never stops anything.
                   Only a scan-matched interval enters (the LiDAR speed's quality check: the estimator calls observe()
                   after a match with fitness >= 0.5 over a scan gap < 0.5 s); a failed match, a scan gap, no status
                   line or a prior below monitor_min_speed_m_s leaves the window as it is. The scale is then HELD
                   (PW-10): the snapshot reports online_scale_frozen and the reason online_scale_hold when the newest
                   window entry is older than FROZEN_AFTER_S (lidar_stale: no new scan; lidar_unmatched: the newest
                   scan did not match; not_driving: the prior is below the monitor speed; no_status: no status line
                   over the interval).
  (e) distance     cumulative prior displacement over the scan intervals, reported next to the LiDAR distance.
Never: steering on the prior alone beyond the bound, a row end from the prior, a position accumulated without LiDAR.

DEFAULT_SPEED_TABLE is a PLACEHOLDER: the duty -> speed relation of the vehicle BEFORE the steering gear was fitted
(docs/drive_response/README_KO.md: the fitted line v = 0.00855 x duty + 0.025 up to 80 %, and the two 10/10 sample
medians 90 % 0.715 m/s and 100 % 0.899 m/s). Below 40 % nothing was measured. The field tuning file replaces it
(design v2 V2-5) and the steered vehicle has to be measured again. Standard library only.
"""
from collections import deque
from dataclasses import dataclass
import math

# (duty %, speed m/s). PLACEHOLDER, see the module docstring: line samples at 25 and 80 %, then the 10/10 medians.
DEFAULT_SPEED_TABLE = ((25., .239), (80., .709), (90., .715), (100., .899))
KEEP_S = 4.                         # status samples older than this are dropped (longest interval ever integrated)
PREDICTING_AFTER_S = .25            # newest scan older than this (2.5 scan periods): the pose is a prediction
FROZEN_AFTER_S = .5                 # (c) newest window entry older than this: the online scale is held (5 scan periods,
                                    # the scan gap the estimator no longer matches on the LiDAR speed alone; a shorter
                                    # value flickers when a slow host skips scans)
HOLD_REASONS = ('lidar_stale', 'lidar_unmatched', 'not_driving', 'no_status')


def _finite(*values):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)


class SpeedTable:
    """Duty [% of full scale] -> steady forward speed [m/s] of the prior. Points: duty strictly increasing in (0, 100],
    speed >= 0 and never decreasing with the duty (a table that falls with the duty is refused)."""

    def __init__(self, points):
        try:
            pts = tuple((float(d), float(v)) for d, v in points)
        except (TypeError, ValueError):
            raise ValueError('speed_table_invalid: points must be (duty_pct, speed_m_s) pairs') from None
        if len(pts) < 2:
            raise ValueError('speed_table_invalid: at least two points')
        for k, (duty, speed) in enumerate(pts):
            if not (math.isfinite(duty) and math.isfinite(speed) and 0. < duty <= 100. and speed >= 0.):
                raise ValueError('speed_table_invalid: point %d must have 0 < duty <= 100 and a finite speed >= 0' % k)
            if k and not (duty > pts[k - 1][0] and speed >= pts[k - 1][1]):
                raise ValueError('speed_table_invalid: point %d: duty must increase, speed must not fall' % k)
        if not pts[-1][1] > 0.:
            raise ValueError('speed_table_invalid: the last speed must be > 0')
        self.points = pts

    def speed(self, duty_pct):
        """Signed speed prior of a signed duty (before the scale)."""
        duty = abs(float(duty_pct))
        sign = -1. if duty_pct < 0. else 1.
        d0, v0 = 0., 0.
        for d1, v1 in self.points:
            if duty <= d1:
                return sign * (v0 + (v1 - v0) * (duty - d0) / (d1 - d0))
            d0, v0 = d1, v1
        return sign * v0

    def duty(self, speed_m_s):
        """Smallest duty [%] whose table speed reaches speed_m_s >= 0 (None above the table)."""
        target = float(speed_m_s)
        if not target >= 0.:
            raise ValueError('speed_table_invalid: the inverse needs a speed >= 0')
        d0, v0 = 0., 0.
        for d1, v1 in self.points:
            if target <= v1:
                return d0 if v1 == v0 else d0 + (d1 - d0) * (target - v0) / (v1 - v0)
            d0, v0 = d1, v1
        return None


@dataclass(frozen=True)
class ActuatorPriorConfig:
    speed_table: tuple = DEFAULT_SPEED_TABLE   # PLACEHOLDER (module docstring)
    speed_scale: float = 1.              # k_field: table speed x scale = expected speed (field tuning, 0.5 .. 1.3)
    wheelbase_m: float | None = None     # None: no yaw-rate prior
    status_max_age_s: float = .3         # = the steering status age limit of the executor
    weight: float = 0.                   # 0 = monitor and record only; (0, 1] = seed blend and prediction share
    predict_max_s: float = 1.            # (b) bound since the last scan-matched pose (locked value, placeholder)
    predict_max_m: float = .5
    monitor_window_s: float = 20.        # (c) driving time in the online scale
    monitor_min_s: float = 10.           # ... at least this much before the table is judged
    monitor_min_speed_m_s: float = .1    # samples with a slower prior are not driving
    mismatch_fraction: float = .2        # table error |prior / LiDAR - 1| above this: table_mismatch (placeholder)

    def __post_init__(self):
        SpeedTable(self.speed_table)
        if not (_finite(self.speed_scale) and .1 <= self.speed_scale <= 3.):
            raise ValueError('actuator_prior_invalid: speed_scale must be in [0.1, 3]')
        if self.wheelbase_m is not None and not (_finite(self.wheelbase_m) and self.wheelbase_m > 0.):
            raise ValueError('actuator_prior_invalid: wheelbase_m must be > 0 or None')
        if not (_finite(self.weight) and 0. <= self.weight <= 1.):
            raise ValueError('actuator_prior_invalid: weight must be in [0, 1]')
        if not (_finite(self.status_max_age_s, self.predict_max_s, self.predict_max_m) and self.status_max_age_s > 0.
                and 0. < self.predict_max_s <= KEEP_S - 1. and self.predict_max_m > 0.):
            raise ValueError('actuator_prior_invalid: status_max_age_s > 0, 0 < predict_max_s <= %g, predict_max_m > 0'
                             % (KEEP_S - 1.))
        if not (_finite(self.monitor_window_s, self.monitor_min_s, self.monitor_min_speed_m_s, self.mismatch_fraction)
                and 0. < self.monitor_min_s <= self.monitor_window_s and self.monitor_min_speed_m_s > 0.
                and 0. < self.mismatch_fraction < 1.):
            raise ValueError('actuator_prior_invalid: need 0 < monitor_min_s <= monitor_window_s, '
                             'monitor_min_speed_m_s > 0 and 0 < mismatch_fraction < 1')


class ActuatorPrior:
    """State of the prior layer of ONE estimator (one run). The estimator calls it; nothing here reads a sensor."""

    def __init__(self, config):
        self.config = config
        self.table = SpeedTable(config.speed_table)
        self.samples = deque()           # (stamp, speed prior, yaw-rate prior | None, duty %, steering angle | None)
        self.accepted = 0
        self.rejected = 0
        self.predict_s = 0.              # (b) predicted since the last scan-matched pose: time ...
        self.predict_m = 0.              # ... and distance
        self.limit = False               # the bound was reached; cleared by the next scan-matched pose
        self.predicted_scans = 0         # failed scan matches that got a predicted translation (whole run)
        self.distance = 0.               # (e) cumulative |prior displacement| over scan-matched intervals
        self.window = deque()            # (c) (stamp, dt, LiDAR step, prior step) of driving samples
        self.window_s = 0.               # driving time in the window
        self.last_entry = None           # (c) end stamp of the newest window entry (PW-10: the scale's update time)
        self.last_skip = None            # (c) (end stamp, reason in HOLD_REASONS) of the newest interval left out
        self.unmatched = 0               # failed scan matches seen by predict() (whole run; also with weight 0)

    # --- input ------------------------------------------------------------------------------------------------
    def update(self, stamp, duty_left_pct, duty_right_pct, steer_rad=None):
        """One status line. False (counted) for a value that is not finite, a duty beyond +-100 %, a steering angle
        beyond +-pi/2 or a stamp that does not increase."""
        if not _finite(stamp, duty_left_pct, duty_right_pct) or max(abs(duty_left_pct), abs(duty_right_pct)) > 100.:
            self.rejected += 1
            return False
        if steer_rad is not None and not (_finite(steer_rad) and abs(steer_rad) < math.pi / 2):
            self.rejected += 1
            return False
        if self.samples and stamp <= self.samples[-1][0]:
            self.rejected += 1
            return False
        duty = .5 * (float(duty_left_pct) + float(duty_right_pct))
        speed = self.table.speed(duty) * self.config.speed_scale
        yaw = None
        if steer_rad is not None and self.config.wheelbase_m is not None:
            yaw = speed * math.tan(float(steer_rad)) / self.config.wheelbase_m
        self.samples.append((float(stamp), speed, yaw, duty, None if steer_rad is None else float(steer_rad)))
        self.accepted += 1
        while self.samples and stamp - self.samples[0][0] > KEEP_S:
            self.samples.popleft()
        return True

    def latest(self, now):
        """The newest sample when it is not older than status_max_age_s at `now` (nor from the future), else None."""
        if not self.samples:
            return None
        sample = self.samples[-1]
        return sample if 0. <= now - sample[0] <= self.config.status_max_age_s else None

    def displacement(self, t0, t1):
        """(prior displacement [m] over (t0, t1], complete): zero-order hold of the samples, each for at most
        status_max_age_s. complete is False when a part of the interval has no valid sample."""
        if not t1 > t0:
            return 0., True
        total = covered = 0.
        hold = self.config.status_max_age_s
        samples = list(self.samples)
        for k, (stamp, speed, _, _, _) in enumerate(samples):
            end = min(stamp + hold, samples[k + 1][0] if k + 1 < len(samples) else math.inf)
            lo, hi = max(stamp, t0), min(end, t1)
            if hi > lo:
                total += speed * (hi - lo)
                covered += hi - lo
        return total, covered >= (t1 - t0) - 1e-9

    # --- (a) seed -----------------------------------------------------------------------------------------------
    def seed(self, t0, t1, lidar_speed):
        """Translation seed of the scan interval (t0, t1], or None for the original rule (weight 0, or the prior does
        not cover the interval)."""
        weight = self.config.weight
        if not weight > 0.:
            return None
        step, complete = self.displacement(t0, t1)
        if not complete:
            return None
        return (1. - weight) * lidar_speed * (t1 - t0) + weight * step

    # --- (b) prediction -------------------------------------------------------------------------------------------
    def matched(self):
        """A scan-matched pose: the prediction bound starts again."""
        self.predict_s = self.predict_m = 0.
        self.limit = False

    def predict(self, t0, t1):
        """Translation [m] to add for a FAILED scan match over (t0, t1] (0 while weight 0, without a complete prior or
        beyond the bound). Counts the interval against the bound."""
        self.unmatched += 1
        self.last_skip = (float(t1), 'lidar_unmatched')     # (c) the monitor sees no LiDAR speed for this interval
        weight = self.config.weight
        if not weight > 0.:
            return 0.
        cfg = self.config
        room_s = cfg.predict_max_s - self.predict_s
        self.predict_s += t1 - t0
        if room_s <= 0. or self.limit:
            self.limit = True
            return 0.
        end = min(t1, t0 + room_s)
        step, complete = self.displacement(t0, end)
        if not complete:
            self.limit = self.limit or self.predict_s > cfg.predict_max_s
            return 0.
        step *= weight
        room_m = cfg.predict_max_m - self.predict_m
        if abs(step) >= room_m:
            step = math.copysign(max(0., room_m), step)
            self.limit = True
        if self.predict_s > cfg.predict_max_s:
            self.limit = True
        self.predict_m += abs(step)
        if step:
            self.predicted_scans += 1
        return step

    def extrapolate(self, t0, now, lidar_speed):
        """Travel [m] since the newest scan (stamp t0) for the snapshot at `now`: (travel, limit), or None for the
        original rule (weight 0, or the prior does not cover the time). Does not change the state."""
        weight = self.config.weight
        if not weight > 0.:
            return None
        cfg = self.config
        horizon = max(0., now - t0)
        room_s = max(0., cfg.predict_max_s - self.predict_s)
        limit = self.limit or horizon > room_s
        span = min(horizon, room_s)
        step, complete = self.displacement(t0, t0 + span)
        if not complete:
            return None
        travel = (1. - weight) * lidar_speed * span + weight * step
        room_m = max(0., cfg.predict_max_m - self.predict_m)
        if abs(travel) >= room_m and horizon > 0.:
            travel = math.copysign(room_m, travel)
            limit = True
        return travel, limit

    # --- (c) monitor, (e) distance ----------------------------------------------------------------------------
    def observe(self, t0, t1, lidar_step):
        """A scan-matched interval (t0, t1] with the LiDAR forward step [m]."""
        step, complete = self.displacement(t0, t1)
        dt = t1 - t0
        if not dt > 0.:
            return
        if not complete:
            self.last_skip = (float(t1), 'no_status')
            return
        self.distance += abs(step)
        if abs(step) < self.config.monitor_min_speed_m_s * dt:
            self.last_skip = (float(t1), 'not_driving')
            return
        self.window.append((t1, dt, float(lidar_step), step))
        self.window_s += dt
        self.last_entry = float(t1)
        popped = False
        while self.window and self.window_s - self.window[0][1] >= self.config.monitor_window_s:
            self.window_s -= self.window.popleft()[1]
            popped = True
        if popped:
            self.window_s = math.fsum(entry[1] for entry in self.window)     # no drift of the running sum

    def online_scale(self):
        """LiDAR distance / prior distance of the monitor window (None before monitor_min_s of driving)."""
        if self.window_s < self.config.monitor_min_s:
            return None
        prior = math.fsum(entry[3] for entry in self.window)
        if not abs(prior) > 1e-6:
            return None
        return math.fsum(entry[2] for entry in self.window) / prior

    def scale_hold(self, now, scan_stamp):
        """(updating, hold reason | None) of the online scale at `now` (PW-10). updating: the newest window entry is at
        most FROZEN_AFTER_S old. Otherwise the reason is lidar_stale when the newest scan is older than FROZEN_AFTER_S
        (or there is none), else the reason of the newest interval left out (lidar_unmatched, not_driving, no_status),
        else lidar_stale (no interval since the newest entry)."""
        if self.last_entry is not None and 0. <= now - self.last_entry <= FROZEN_AFTER_S:
            return True, None
        if scan_stamp is None or now - scan_stamp > FROZEN_AFTER_S:
            return False, 'lidar_stale'
        skip = self.last_skip
        if skip is not None and (self.last_entry is None or skip[0] > self.last_entry):
            return False, skip[1]
        return False, 'lidar_stale'

    def table_mismatch(self):
        scale = self.online_scale()
        if scale is None:
            return False
        if not scale > 0.:
            return True                  # the LiDAR saw no forward motion (or the other direction) for the whole window
        return abs(1. / scale - 1.) > self.config.mismatch_fraction

    # --- output -------------------------------------------------------------------------------------------------
    def snapshot(self, now, scan_stamp, lidar_speed):
        """The 'actuator_prior' block of the estimator snapshot (plain values)."""
        cfg = self.config
        sample = self.latest(now)
        newest = self.samples[-1] if self.samples else None
        horizon = 0. if scan_stamp is None else max(0., now - scan_stamp)
        ahead = None if scan_stamp is None else self.extrapolate(scan_stamp, now, lidar_speed)
        predict_s = self.predict_s + horizon if cfg.weight > 0. else 0.
        predict_m = self.predict_m + (abs(ahead[0]) if ahead is not None else 0.) if cfg.weight > 0. else 0.
        scale = self.online_scale()
        updating, hold = self.scale_hold(now, scan_stamp)
        return dict(
            fresh=sample is not None, age_s=None if newest is None else now - newest[0],
            duty_pct=None if sample is None else sample[3], steer_rad=None if sample is None else sample[4],
            speed_m_s=None if sample is None else sample[1], yaw_rate_rad_s=None if sample is None else sample[2],
            weight=cfg.weight, predict_s=predict_s, predict_m=predict_m,
            predicting=bool(cfg.weight > 0. and (self.predict_s > 0. or horizon > PREDICTING_AFTER_S)),
            limit=bool(self.limit or (ahead is not None and ahead[1])),
            predicted_scans=self.predicted_scans, distance_m=self.distance, online_scale=scale,
            monitor_s=self.window_s, table_mismatch=self.table_mismatch(), samples=self.accepted,
            rejected=self.rejected,
            # PW-10: the scale's update state. frozen = a scale exists and it was not updated for FROZEN_AFTER_S.
            online_scale_age_s=None if self.last_entry is None else now - self.last_entry,
            online_scale_frozen=bool(scale is not None and not updating), online_scale_hold=hold,
            monitor_window_s=cfg.monitor_window_s, monitor_min_s=cfg.monitor_min_s, unmatched_scans=self.unmatched)

