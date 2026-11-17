// SIM model of the SMOOTH hold of the board firmware r5 and the SIM host ANGLE policy for it (see smooth_r5.h).
#include "smooth_r5.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "fw_cal.h"    // firmware/ubts_steer_v3: Cal
#include "fw_steer.h"  // firmware/ubts_steer_v3: steer_law_hook

namespace ubts_steer {

namespace {

const int kRightDirection = 1;  // steerRightDirection: F turns right, the POT value falls toward the right

int imin(int a, int b) { return a < b ? a : b; }
int imax(int a, int b) { return a > b ? a : b; }

}  // namespace

// ---- board ------------------------------------------------------------------------------------------------------
SmoothBoard::SmoothBoard() { configure(SmoothLawConfig()); }

void SmoothBoard::configure(const SmoothLawConfig &cfg) {
  cfg_ = cfg;
  reset(0, -1);
}

void SmoothBoard::reset(uint32_t now, int counts) {
  sample_started_ = false;
  sample_ms_ = 0;
  for (int i = 0; i < 9; ++i) window_[i] = 0;
  window_count_ = window_index_ = 0;
  candidate_ = confirmed_ = -1;
  confirmed_ms_ = 0;
  en_ = false;
  pwm_f_ = pwm_r_ = 0;
  direction_ = 0;
  steer_off_ms_ = now - static_cast<uint32_t>(cfg_.reverse_pause_ms);  // off long ago (a board that has been idle)
  target_active_ = false;
  target_ = cfg_.cal_center;  // setup(): steerTarget = steerCenter
  sm_phase_ = sm_dir_ = sm_pwm_ = floor_ = 0;
  run_start_ = limit_start_ = tick_ms_ = progress_ms_ = settle_start_ = drive_ms_ = 0;
  progress_ref_ = run_start_pot_ = peak_ = 0;
  runs_ = 0;
  fresh_ = braking_ = false;
  quiet_pot_ = -1;
  quiet_ms_ = 0;
  quiet_pending_ = false;
  cancels_ = 0;
  last_cancel_ = kSmoothCancelNone;
  if (counts >= 0 && plausible(counts)) {  // the filter as a board that has been sampling would have it
    for (int i = 0; i < cfg_.median_of; ++i) window_[i] = counts;
    window_count_ = cfg_.median_of;
    window_index_ = 0;
    candidate_ = confirmed_ = counts;
    confirmed_ms_ = now;
    sample_started_ = true;
    sample_ms_ = now;
  }
}

bool SmoothBoard::plausible(int v) const {  // plausiblePotValue(): the hard guards +- POT_LIMIT_MARGIN
  return v > 100 && v < 4000 && v > cfg_.hard_right_min - cfg_.limit_margin &&
         v < cfg_.hard_left_max + cfg_.limit_margin;
}

void SmoothBoard::reset_filter() {
  candidate_ = -1;
  window_count_ = 0;
  window_index_ = 0;
}

int SmoothBoard::read_pot(uint32_t now, int counts) {  // readPotValue()
  if (!sample_started_ || static_cast<uint32_t>(now - sample_ms_) >= static_cast<uint32_t>(cfg_.sample_ms)) {
    sample_started_ = true;
    sample_ms_ = now;
    const int raw = counts;
    if (!plausible(raw)) {
      reset_filter();
    } else {
      window_[window_index_] = raw;
      window_index_ = (window_index_ + 1) % cfg_.median_of;
      if (window_count_ < cfg_.median_of) ++window_count_;
      if (window_count_ == cfg_.median_of) {
        int sorted[9];
        for (int i = 0; i < cfg_.median_of; ++i) sorted[i] = window_[i];
        for (int i = 1; i < cfg_.median_of; ++i) {
          const int item = sorted[i];
          int j = i - 1;
          while (j >= 0 && sorted[j] > item) {
            sorted[j + 1] = sorted[j];
            --j;
          }
          sorted[j + 1] = item;
        }
        const int median = sorted[cfg_.median_of / 2];
        if (candidate_ >= 0 && std::abs(median - candidate_) <= cfg_.pair_tol) {
          int next = (median + candidate_) / 2;
          // r4: never follow faster than slew_per_ms counts/ms (at least slew_min) while the last confirmation is fresh
          if (confirmed_ >= 0 && static_cast<uint32_t>(now - confirmed_ms_) <= static_cast<uint32_t>(cfg_.stale_ms)) {
            const int limit = imax(cfg_.slew_min, static_cast<int>(static_cast<uint32_t>(cfg_.slew_per_ms) *
                                                                    static_cast<uint32_t>(now - confirmed_ms_)));
            if (next > confirmed_ + limit) {
              next = confirmed_ + limit;
            } else if (next < confirmed_ - limit) {
              next = confirmed_ - limit;
            }
          }
          confirmed_ = next;
          confirmed_ms_ = now;
        }
        candidate_ = median;
      }
    }
  }
  if (confirmed_ < 0 || static_cast<uint32_t>(now - confirmed_ms_) > static_cast<uint32_t>(cfg_.stale_ms)) return -1;
  return confirmed_;
}

int SmoothBoard::output() const {
  if (!en()) return 0;
  if (pwm_f_ > 0) return -pwm_f_;  // F = right = toward - permille
  if (pwm_r_ > 0) return pwm_r_;
  return 0;
}

void SmoothBoard::deenergize(uint32_t now) {  // deenergizeMotor(0)
  en_ = false;
  pwm_f_ = pwm_r_ = 0;
  if (direction_ != 0) steer_off_ms_ = now;
  direction_ = 0;
}

void SmoothBoard::stop_motor(uint32_t now) {  // stopMotor(0)
  deenergize(now);
  target_active_ = false;
  sm_phase_ = 0;
  runs_ = 0;
  fresh_ = false;
  braking_ = false;
  quiet_pending_ = false;
}

void SmoothBoard::stop(uint32_t now) { stop_motor(now); }

void SmoothBoard::cancel(uint32_t now, int reason) {  // smCancel()
  stop_motor(now);
  ++cancels_;
  last_cancel_ = reason;
}

void SmoothBoard::write_pwm(int direction, int pwm) {
  if (direction == 1) {
    pwm_f_ = pwm;
  } else {
    pwm_r_ = pwm;
  }
}

void SmoothBoard::start_run(int direction, int value, uint32_t now) {  // smStartRun()
  en_ = false;
  pwm_f_ = pwm_r_ = 0;
  braking_ = false;
  quiet_pot_ = value;
  quiet_ms_ = now;
  sm_dir_ = direction;
  floor_ = cfg_.pwm_min;
  sm_pwm_ = cfg_.pwm_min;
  write_pwm(direction, sm_pwm_);
  direction_ = direction;
  en_ = true;
  run_start_ = limit_start_ = tick_ms_ = progress_ms_ = now;
  progress_ref_ = run_start_pot_ = peak_ = value;
  ++runs_;
  sm_phase_ = 1;
  quiet_pending_ = false;
}

// smQuietSample(): the firmware switches EN low for quiet_us and keeps the median of five reads; the SIM switched EN
// low for the servo period before this one and takes this period's ADC value (see smooth_r5.h).
void SmoothBoard::quiet_sample(uint32_t now, int counts) {
  int v = counts;
  if (plausible(v)) {
    if (quiet_pot_ >= 0) {
      const int limit = imax(cfg_.slew_min, static_cast<int>(static_cast<uint32_t>(cfg_.slew_per_ms) *
                                                              static_cast<uint32_t>(now - quiet_ms_)));
      if (v > quiet_pot_ + limit) {
        v = quiet_pot_ + limit;
      } else if (v < quiet_pot_ - limit) {
        v = quiet_pot_ - limit;
      }
    }
    quiet_pot_ = v;
  }
  quiet_ms_ = now;
}

void SmoothBoard::end_run(uint32_t now, int value) {  // smEndRun()
  drive_ms_ += static_cast<uint32_t>(now - run_start_);
  if (cfg_.brake_ms > 0) {
    // dynamic brake: both PWM 0 with EN high (both low-side switches); ends after brake_ms or at any stop
    pwm_f_ = pwm_r_ = 0;
    steer_off_ms_ = now;
    direction_ = 0;
    braking_ = true;
  } else {
    deenergize(now);
  }
  sm_phase_ = 2;
  settle_start_ = now;
  peak_ = value;
  quiet_pending_ = false;
}

bool SmoothBoard::angle(uint32_t now, float deg, int counts) {  // moveSteeringToAngle(deg < 0, fabsf(deg))
  const bool left = deg < 0.0f;
  const float degrees = std::fabs(deg);
  if (!(degrees >= 0.0f && degrees <= 20.0f)) return false;  // ERROR: angle must be 0..20 degrees (also NaN)
  const int value = read_pot(now, counts);
  if (!plausible(value)) {
    stop_motor(now);  // STEER STOP: INVALID POT
    return false;
  }
  const int span = left ? cfg_.cal_left - cfg_.cal_center : cfg_.cal_right - cfg_.cal_center;
  const int target = cfg_.cal_center +
                     static_cast<int>(lroundf(static_cast<float>(span) * degrees /
                                              static_cast<float>(cfg_.command_max_deg)));
  if (target_active_ && target_ == target) return true;  // OK: SAME ABSOLUTE TARGET (budget unchanged)
  const int want = target < value ? kRightDirection : -kRightDirection;
  if (target_active_ && sm_phase_ == 1 && direction_ == sm_dir_ && want == sm_dir_ &&
      std::abs(target - value) > cfg_.stop_tol) {
    // same direction while driving: take the new target over without stopping
    target_ = target;
    runs_ = 1;
    fresh_ = true;
    limit_start_ = now;
    return true;
  }
  stop_motor(now);  // a reversal then waits reverse_pause_ms unpowered
  target_ = target;
  target_active_ = true;
  fresh_ = true;
  drive_ms_ = 0;
  return true;
}

void SmoothBoard::loop(uint32_t now, int counts) {
  // checkSteerTimeout(): the 1000 ms steering timeout does not apply to the smooth hold with a target; the steering
  // motor runs only inside the hold here (no manual 1 F / 1 R).
  if (target_active_) update(now, counts);
}

void SmoothBoard::update(uint32_t now, int counts) {  // updateSmoothHold()
  const int value = read_pot(now, counts);
  if (!plausible(value)) {
    cancel(now, kSmoothCancelInvalid);  // STEER HOLD CANCELLED: ADC STALE/INVALID
    return;
  }
  const int error = target_ - value;  // > 0: the target is to the LEFT (POT increases)
  const bool run_right = sm_dir_ == kRightDirection;
  if (sm_phase_ == 1) {
    if (cfg_.quiet_ms > 0) {
      if (quiet_pending_) {
        quiet_sample(now, counts);  // EN went low one servo period ago
        quiet_pending_ = false;
      } else if (static_cast<uint32_t>(now - quiet_ms_) >= static_cast<uint32_t>(cfg_.quiet_ms)) {
        quiet_pending_ = true;  // EN low for this servo period; the read and the run rules follow on the next one
        return;
      }
    }
    const int pos = (cfg_.quiet_ms > 0 && quiet_pot_ >= 0) ? quiet_pot_ : value;
    const int run_error = target_ - pos;
    peak_ = run_right ? imin(peak_, pos) : imax(peak_, pos);
    // never chase a reading past a CAL mark (the pulse hold's guard)
    if ((run_right && pos <= cfg_.cal_right - cfg_.guard) || (!run_right && pos >= cfg_.cal_left + cfg_.guard)) {
      end_run(now, pos);
      return;
    }
    const int along = run_right ? -run_error : run_error;  // remaining distance in the running direction
    if (along <= cfg_.stop_tol) {                           // reached, or passed: brake, settle
      end_run(now, pos);
      return;
    }
    const int progress = run_right ? progress_ref_ - pos : pos - progress_ref_;
    if (progress >= cfg_.progress_counts) {
      progress_ref_ = pos;
      progress_ms_ = now;
    } else if (static_cast<uint32_t>(now - progress_ms_) >= static_cast<uint32_t>(cfg_.stall_ms)) {
      if (sm_pwm_ >= cfg_.pwm_max) {
        drive_ms_ += static_cast<uint32_t>(now - run_start_);
        cancel(now, kSmoothCancelStall);  // STALL (no force retry)
        return;
      }
      floor_ = imin(cfg_.pwm_max, floor_ + cfg_.boost);  // battery voltage and load change the PWM that moves it
      progress_ms_ = now;
      progress_ref_ = pos;
    }
    if (static_cast<uint32_t>(now - limit_start_) >= static_cast<uint32_t>(cfg_.max_run_ms)) {
      drive_ms_ += static_cast<uint32_t>(now - run_start_);
      cancel(now, kSmoothCancelRunTime);  // RUN TIME LIMIT
      return;
    }
    if (static_cast<uint32_t>(now - tick_ms_) >= static_cast<uint32_t>(cfg_.ramp_tick_ms)) {
      tick_ms_ = now;
      const float scale = std::min(1.0f, std::max(0.0f, static_cast<float>(along - cfg_.stop_tol) /
                                                            static_cast<float>(cfg_.slow)));
      const int want = floor_ + static_cast<int>(lroundf(static_cast<float>(cfg_.pwm_max - floor_) * scale));
      sm_pwm_ = want > sm_pwm_ ? imin(want, sm_pwm_ + cfg_.ramp) : want;
      write_pwm(sm_dir_, sm_pwm_);
    }
    return;
  }
  if (sm_phase_ == 2) {
    if (braking_ && static_cast<uint32_t>(now - settle_start_) >= static_cast<uint32_t>(cfg_.brake_ms)) {
      en_ = false;  // EN low: the brake ends
      braking_ = false;
    }
    // early unpowered travel and the filter latency count toward the rebound peak
    if (static_cast<uint32_t>(now - settle_start_) < static_cast<uint32_t>(cfg_.rebound_window_ms)) {
      peak_ = run_right ? imin(peak_, value) : imax(peak_, value);
    }
    if (static_cast<uint32_t>(now - settle_start_) < static_cast<uint32_t>(cfg_.settle_ms)) return;
    sm_phase_ = 0;
    fresh_ = false;  // after a settled run only drift above restart_tol starts another one
    const int rebound = run_right ? value - peak_ : peak_ - value;
    if (std::abs(error) > cfg_.restart_tol && rebound >= cfg_.rebound_max) {
      cancel(now, kSmoothCancelRebound);  // REBOUND (no force retry)
      return;
    }
  }
  if (std::abs(error) <= cfg_.stop_tol || (!fresh_ && std::abs(error) <= cfg_.restart_tol)) {
    fresh_ = false;
    runs_ = 0;
    return;  // target reached: keep monitoring, no powered preload
  }
  if (runs_ >= cfg_.max_runs) {
    cancel(now, kSmoothCancelNoConvergence);  // NO CONVERGENCE (no force retry)
    return;
  }
  const bool right = error < 0;
  if ((right && value <= cfg_.cal_right) || (!right && value >= cfg_.cal_left)) {
    cancel(now, kSmoothCancelQuietLimit);  // QUIET OPERATING LIMIT
    return;
  }
  const int direction = right ? kRightDirection : -kRightDirection;
  // a reversal waits until the driver has been off for reverse_pause_ms (the motor coasts down first)
  if (direction != sm_dir_ &&
      static_cast<uint32_t>(now - steer_off_ms_) < static_cast<uint32_t>(cfg_.reverse_pause_ms)) {
    return;
  }
  start_run(direction, value, now);
}

// ---- SIM session: ADC path, host policy, hook ----------------------------------------------------------------------
namespace {

struct Host {
  bool started;
  uint32_t last_poll, last_angle;
  double sent_deg;      // command degrees of the last ANGLE (+ right)
  uint32_t cancels_seen;
  bool blocked;         // the last ANGLE was cancelled: not sent again until the request changes
  double blocked_deg;
};

struct Session {
  SmoothLawConfig cfg;
  SmoothBoard board;
  Host host;
  int last_out;  // output of the previous servo period (the ADC value of this period was read with it)
};

Session g_smooth;

void reset_session(uint32_t now, int counts) {
  Session &s = g_smooth;
  s.board.configure(s.cfg);
  s.board.reset(now, counts);
  s.host.started = false;
  s.host.last_poll = s.host.last_angle = 0;
  s.host.sent_deg = 0.0;
  s.host.cancels_seen = 0;
  s.host.blocked = false;
  s.host.blocked_deg = 0.0;
  s.last_out = 0;
}

// The V3 10-bit model ADC through the V3 calibration marks onto the board's 12-bit command scale (as abs_hold8.cc).
int counts_from_raw(uint16_t raw, const ubts_fw::Cal &cal) {
  const SmoothLawConfig &c = g_smooth.cfg;
  const double d = static_cast<double>(raw) - static_cast<double>(cal.raw_c);
  const double left_span = static_cast<double>(cal.raw_l) - static_cast<double>(cal.raw_c);
  const double right_span = static_cast<double>(cal.raw_r) - static_cast<double>(cal.raw_c);
  double p;  // + toward the left mark (1 at the left mark, -1 at the right mark)
  if (d * left_span >= 0.0) {
    p = left_span != 0.0 ? d / left_span : 0.0;
  } else {
    p = right_span != 0.0 ? -d / right_span : 0.0;
  }
  const double counts = p >= 0.0 ? c.cal_center + p * (c.cal_left - c.cal_center)
                                 : c.cal_center + p * (c.cal_center - c.cal_right);
  return static_cast<int>(std::floor(counts + 0.5));
}

// SIM ADC path placeholders: while the steering driver was powered over the period of this ADC value, the reading
// lags the motion by adc_powered_offset counts and current bursts add adc_burst_counts for adc_burst_ms of every
// adc_burst_period_ms (host test r4/r5 models; field: 2026-11-16 NOTES, the size is not measured).
int adc_counts(uint32_t now, uint16_t raw, const ubts_fw::Cal &cal) {
  const Session &s = g_smooth;
  int counts = counts_from_raw(raw, cal);
  if (s.last_out != 0) {
    // last_out < 0: driving right (counts fall) -> the reading stays higher; > 0: left -> lower
    counts += s.last_out < 0 ? s.cfg.adc_powered_offset : -s.cfg.adc_powered_offset;
    if (s.cfg.adc_burst_counts != 0 &&
        now % static_cast<uint32_t>(s.cfg.adc_burst_period_ms) < static_cast<uint32_t>(s.cfg.adc_burst_ms)) {
      counts += s.cfg.adc_burst_counts;
    }
  }
  return counts;
}

double quantize(double deg, double step, double limit) {  // the bridge's quantize_command: half away from zero
  double v = deg > limit ? limit : (deg < -limit ? -limit : deg);
  if (step > 0.0) {
    const double q = std::floor(std::fabs(v) / step + 0.5) * step;
    v = v < 0.0 ? -q : q;
  }
  if (v > limit) v = limit;
  if (v < -limit) v = -limit;
  return v == 0.0 ? 0.0 : v;
}

void send_angle(uint32_t now, double deg, int counts) {
  Session &s = g_smooth;
  s.host.sent_deg = deg;
  s.host.last_angle = now;
  s.board.angle(now, static_cast<float>(deg), counts);
}

// SIM host: the ANGLE the bridge would write for this goal (every poll_ms).
void host_policy(uint32_t now, int32_t goal, int counts) {
  Session &s = g_smooth;
  const SmoothLawConfig &c = s.cfg;
  const double want = quantize(-static_cast<double>(goal) * c.command_max_deg / 1000.0, c.host_step_deg,
                               c.command_max_deg);
  if (!s.host.started) {
    s.host.started = true;
    s.host.last_poll = now;
    s.host.cancels_seen = s.board.cancels();
    send_angle(now, want, counts);
    return;
  }
  if (static_cast<uint32_t>(now - s.host.last_poll) < static_cast<uint32_t>(c.host_poll_ms)) return;
  s.host.last_poll = now;
  if (s.board.cancels() != s.host.cancels_seen) {  // a hold cancel since the last poll
    s.host.cancels_seen = s.board.cancels();
    s.host.blocked = true;
    s.host.blocked_deg = s.host.sent_deg;
  }
  if (s.host.blocked) {
    if (std::fabs(want - s.host.blocked_deg) < 1e-9) return;  // the firmware cancelled this target: no force retry
    s.host.blocked = false;
  }
  if (std::fabs(want - s.host.sent_deg) < c.host_deadband_deg - 1e-9) return;
  const uint32_t since = now - s.host.last_angle;
  const int value = s.board.read_pot(now, counts);
  const bool reached = s.board.target_active() && s.board.phase() == 0 && value >= 0 &&
                       std::abs(s.board.target() - value) <= c.stop_tol;
  const bool idle = !s.board.target_active() || reached;
  if (since >= static_cast<uint32_t>(c.host_min_interval_ms) || idle) send_angle(now, want, counts);
}

}  // namespace

void smooth_r5_install(const SmoothLawConfig &cfg) {
  if (!cfg.present) return;
  g_smooth.cfg = cfg;
  reset_session(0, -1);
  ubts_fw::steer_law_hook = &smooth_r5_law;
}

void smooth_r5_remove() { ubts_fw::steer_law_hook = 0; }

int16_t smooth_r5_law(uint32_t now, int32_t goal, uint16_t raw, bool fresh, const ubts_fw::Cal &cal,
                      int32_t *cmd_permille, uint8_t *release) {
  Session &s = g_smooth;
  if (fresh) reset_session(now, counts_from_raw(raw, cal));  // a new ACTIVE / HOLD period: no target, as after STOP
  const int counts = adc_counts(now, raw, cal);
  host_policy(now, goal, counts);
  s.board.loop(now, counts);
  const int out = s.board.output();
  s.last_out = out;
  *cmd_permille = static_cast<int32_t>(std::lround(-s.host.sent_deg * 1000.0 / s.cfg.command_max_deg));
  *release = s.board.en() ? 0 : 1;  // EN high with both PWM 0 = the short brake (V3 BRAKE pins); EN low = released
  return static_cast<int16_t>(out);
}

}  // namespace ubts_steer
