// SIM model of the ABS-HOLD-8-REBOUND-LIMITS steering law and the SIM host ANGLE policy (see abs_hold8.h).
#include "abs_hold8.h"

#include <cmath>
#include <cstdlib>

#include "fw_cal.h"    // firmware/ubts_steer_v3: Cal
#include "fw_steer.h"  // firmware/ubts_steer_v3: steer_law_hook

namespace ubts_steer {

namespace {

const int kWindowMax = 9;

struct Law {
  LawConfig cfg;
  // potentiometer filter (readPotValue)
  bool sample_started;
  uint32_t sample_ms;
  int window[kWindowMax];
  int window_count, window_index;
  int candidate, confirmed;
  uint32_t confirmed_ms;
  // hold state (updateSteeringHold)
  bool target_active;
  int target;
  int phase;  // 0 idle, 1 pulse, 2 release / settle
  uint32_t pulse_start, pulse_ms, settle_start, drive_ms;
  int pulse_before, pulse_peak, pulse_dir;  // pulse_dir +1 toward the left (counts rise), -1 toward the right
  int no_progress;
  bool pulse_pending;
  // host policy
  bool host_started;
  uint32_t last_poll, last_angle;
  double sent_deg;  // command degrees of the last ANGLE (+ right)
};

Law g_law;

void reset_law() {
  Law &w = g_law;
  w.sample_started = false;
  w.sample_ms = 0;
  for (int i = 0; i < kWindowMax; ++i) w.window[i] = 0;
  w.window_count = w.window_index = 0;
  w.candidate = w.confirmed = -1;
  w.confirmed_ms = 0;
  w.target_active = false;
  w.target = w.cfg.cal_center;
  w.phase = 0;
  w.pulse_start = w.pulse_ms = w.settle_start = w.drive_ms = 0;
  w.pulse_before = w.pulse_peak = w.cfg.cal_center;
  w.pulse_dir = 0;
  w.no_progress = 0;
  w.pulse_pending = false;
  w.host_started = false;
  w.last_poll = w.last_angle = 0;
  w.sent_deg = 0.0;
}

// The V3 10-bit model ADC through the V3 calibration marks onto the board's 12-bit command scale.
int counts_from_raw(uint16_t raw, const ubts_fw::Cal &cal) {
  const LawConfig &c = g_law.cfg;
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

bool plausible(int v) {
  const LawConfig &c = g_law.cfg;
  return v > 100 && v < 4000 && v > c.cal_right - c.limit_margin && v < c.cal_left + c.limit_margin;
}

void reset_filter() {
  g_law.candidate = -1;
  g_law.window_count = 0;
  g_law.window_index = 0;
}

// The board's filter runs all the time; the hook runs only in the V3 servo modes. At the start of a servo period the
// filter is primed with the present value as a board that has been sampling would have it.
void prime_filter(uint32_t now, uint16_t raw, const ubts_fw::Cal &cal) {
  Law &w = g_law;
  const int value = counts_from_raw(raw, cal);
  if (!plausible(value)) {
    reset_filter();
    w.confirmed = -1;
    return;
  }
  for (int i = 0; i < w.cfg.median_of; ++i) w.window[i] = value;
  w.window_count = w.cfg.median_of;
  w.window_index = 0;
  w.candidate = value;
  w.confirmed = value;
  w.confirmed_ms = now;
  w.sample_started = true;
  w.sample_ms = now;
}

// readPotValue(): one sample every sample_ms, median of median_of, pair confirmation; -1 when stale.
int read_pot(uint32_t now, uint16_t raw, const ubts_fw::Cal &cal) {
  Law &w = g_law;
  const LawConfig &c = w.cfg;
  if (!w.sample_started || static_cast<uint32_t>(now - w.sample_ms) >= static_cast<uint32_t>(c.sample_ms)) {
    w.sample_started = true;
    w.sample_ms = now;
    const int value = counts_from_raw(raw, cal);
    if (!plausible(value)) {
      reset_filter();
    } else {
      w.window[w.window_index] = value;
      w.window_index = (w.window_index + 1) % c.median_of;
      if (w.window_count < c.median_of) ++w.window_count;
      if (w.window_count == c.median_of) {
        int sorted[kWindowMax];
        for (int i = 0; i < c.median_of; ++i) sorted[i] = w.window[i];
        for (int i = 1; i < c.median_of; ++i) {
          const int item = sorted[i];
          int j = i - 1;
          while (j >= 0 && sorted[j] > item) {
            sorted[j + 1] = sorted[j];
            --j;
          }
          sorted[j + 1] = item;
        }
        const int median = sorted[c.median_of / 2];
        if (w.candidate >= 0 && std::abs(median - w.candidate) <= c.pair_tol) {
          w.confirmed = (median + w.candidate) / 2;
          w.confirmed_ms = now;
        }
        w.candidate = median;
      }
    }
  }
  if (w.confirmed < 0 || static_cast<uint32_t>(now - w.confirmed_ms) > static_cast<uint32_t>(c.stale_ms)) return -1;
  return w.confirmed;
}

void stop_motor() {  // stopMotor(0) for the steering: de-energise, target off, phase idle
  Law &w = g_law;
  w.target_active = false;
  w.phase = 0;
  w.pulse_pending = false;
  w.drive_ms = 0;
  w.no_progress = 0;
}

void cancel() { stop_motor(); }

int target_of(double deg) {  // moveSteeringToAngle(): absolute target from the fixed centre
  const LawConfig &c = g_law.cfg;
  const bool left = deg < 0.0;
  const double magnitude = std::fabs(deg);
  const int span = left ? c.cal_left - c.cal_center : c.cal_right - c.cal_center;
  return c.cal_center + static_cast<int>(std::lround(span * magnitude / c.command_max_deg));
}

void send_angle(uint32_t now, double deg) {
  Law &w = g_law;
  w.sent_deg = deg;
  w.last_angle = now;
  const int target = target_of(deg);
  if (w.target_active && w.target == target) return;  // same absolute target: budget unchanged
  stop_motor();
  w.target = target;
  w.target_active = true;
  w.phase = 2;
  w.settle_start = now;
}

void finish_pulse(uint32_t now) {
  Law &w = g_law;
  w.drive_ms += static_cast<uint32_t>(now - w.pulse_start);
  w.phase = 2;
  w.settle_start = now;
  w.pulse_pending = true;
}

// updateSteeringHold(); returns the output (+ toward the left, the fw.u sign) of this servo tick.
int update_hold(uint32_t now, int value) {
  Law &w = g_law;
  const LawConfig &c = w.cfg;
  if (value < 0) {
    cancel();  // ADC stale / invalid
    return 0;
  }
  if (w.phase == 1) {
    const bool right = w.pulse_dir < 0;
    w.pulse_peak = right ? (value < w.pulse_peak ? value : w.pulse_peak) : (value > w.pulse_peak ? value : w.pulse_peak);
    if (static_cast<uint32_t>(now - w.pulse_start) >= w.pulse_ms || (right && value <= c.cal_right - c.guard) ||
        (!right && value >= c.cal_left + c.guard)) {
      finish_pulse(now);
      return 0;
    }
    return w.pulse_dir * c.pulse_pwm;
  }
  if (w.phase == 2) {
    if (w.pulse_pending && static_cast<uint32_t>(now - w.settle_start) < static_cast<uint32_t>(c.rebound_window_ms)) {
      const bool right = w.pulse_dir < 0;
      w.pulse_peak = right ? (value < w.pulse_peak ? value : w.pulse_peak)
                           : (value > w.pulse_peak ? value : w.pulse_peak);
    }
    if (static_cast<uint32_t>(now - w.settle_start) < static_cast<uint32_t>(c.settle_ms)) return 0;
    w.phase = 0;
  }
  const int error = w.target - value;
  if (w.pulse_pending) {
    const bool right = w.pulse_dir < 0;
    const int progress = right ? w.pulse_before - value : value - w.pulse_before;
    const int rebound = right ? value - w.pulse_peak : w.pulse_peak - value;
    w.pulse_pending = false;
    if (std::abs(error) > c.target_tol) {
      if (rebound >= c.rebound_max) {
        cancel();
        return 0;
      }
      w.no_progress = progress < c.progress_min ? w.no_progress + 1 : 0;
      if (w.no_progress >= c.no_progress_max) {
        cancel();
        return 0;
      }
    }
  }
  if (std::abs(error) <= c.target_tol) {
    w.drive_ms = 0;
    w.no_progress = 0;
    return 0;
  }
  if (std::abs(error) <= c.restart_tol && w.drive_ms == 0) return 0;
  if (w.drive_ms + 20u >= static_cast<uint32_t>(c.budget_ms)) {
    cancel();
    return 0;
  }
  const bool right = error < 0;
  if ((right && value <= c.cal_right) || (!right && value >= c.cal_left)) {
    cancel();  // quiet operating limit
    return 0;
  }
  w.pulse_dir = right ? -1 : 1;
  w.pulse_before = value;
  w.pulse_peak = value;
  uint32_t ms = static_cast<uint32_t>(std::abs(error) > c.long_above ? c.long_ms : c.short_ms);
  const uint32_t left_budget = static_cast<uint32_t>(c.budget_ms) - w.drive_ms;
  w.pulse_ms = ms < left_budget ? ms : left_budget;
  w.pulse_start = now;
  w.phase = 1;
  return w.pulse_dir * c.pulse_pwm;
}

// SIM host: the ANGLE it would send for this goal, every poll_ms.
void host_policy(uint32_t now, int32_t goal, int value) {
  Law &w = g_law;
  const LawConfig &c = w.cfg;
  double want = -static_cast<double>(goal) * c.command_max_deg / 1000.0;
  want = std::floor(want / c.host_step_deg + 0.5) * c.host_step_deg;
  if (want > c.command_max_deg) want = c.command_max_deg;
  if (want < -c.command_max_deg) want = -c.command_max_deg;
  if (!w.host_started) {
    w.host_started = true;
    w.last_poll = now;
    send_angle(now, want);
    return;
  }
  if (static_cast<uint32_t>(now - w.last_poll) < static_cast<uint32_t>(c.host_poll_ms)) return;
  w.last_poll = now;
  const uint32_t since = now - w.last_angle;
  const bool reached = w.target_active && w.phase == 0 && value >= 0 && std::abs(w.target - value) <= c.target_tol;
  const bool idle = !w.target_active || reached;
  if (std::fabs(want - w.sent_deg) >= c.host_deadband_deg - 1e-9) {
    if (since >= static_cast<uint32_t>(c.host_min_interval_ms) || idle) send_angle(now, want);
    return;
  }
  if (!w.target_active && value >= 0 && std::abs(target_of(w.sent_deg) - value) > c.restart_tol) {
    send_angle(now, w.sent_deg);  // re-send after a cancelled hold
  }
}

}  // namespace

void abs_hold8_install(const LawConfig &law) {
  g_law.cfg = law;
  reset_law();
  ubts_fw::steer_law_hook = law.present ? &abs_hold8_law : 0;
}

void abs_hold8_remove() { ubts_fw::steer_law_hook = 0; }

int16_t abs_hold8_law(uint32_t now, int32_t goal, uint16_t raw, bool fresh, const ubts_fw::Cal &cal,
                      int32_t *cmd_permille, uint8_t *release) {
  Law &w = g_law;
  if (fresh) {  // a new ACTIVE / HOLD period: the board starts without a target (as after STOP)
    reset_law();
    prime_filter(now, raw, cal);
  }
  const int value = read_pot(now, raw, cal);
  host_policy(now, goal, value);
  const int out = w.target_active ? update_hold(now, value) : 0;
  *cmd_permille = static_cast<int32_t>(std::lround(-w.sent_deg * 1000.0 / w.cfg.command_max_deg));
  *release = 1;
  return static_cast<int16_t>(out);
}

}  // namespace ubts_steer
