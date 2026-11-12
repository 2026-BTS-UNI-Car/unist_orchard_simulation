// Potentiometer and ADC model (see potentiometer.h).
#include "potentiometer.h"

#include <cmath>
#include <stdexcept>

namespace ubts_steer {

namespace {

const double kTwoPi = 6.283185307179586;

// Round half away from zero, exact for |x| < 2^52.
double round_half_away(double x) {
  if (x < 0.0) return -round_half_away(-x);
  const double whole = std::floor(x);
  return x - whole >= 0.5 ? whole + 1.0 : whole;
}

}  // namespace

Potentiometer::Potentiometer()
    : n_(1024),
      g_(0.0),
      captured_(false),
      captured_clipped_(false),
      clipped_(false),
      captured_raw_(0),
      belt_on_(false),
      belt_started_(false),
      belt_half_play_(0.0),
      belt_creep_(0.0),
      belt_skip_(0.0),
      belt_step_(0.0),
      belt_shaft_(0.0),
      belt_offset_(0.0) {}

void Potentiometer::configure(const PotConfig &cfg) {
  cfg_ = cfg;
  n_ = 1 << cfg.adc_bits;
  g_ = static_cast<double>(n_) * cfg.to_steer_ratio * (1.0 + cfg.gain_error) / cfg.range_rad;
}

void Potentiometer::configure_belt(const ExtConfig &ext) {
  belt_half_play_ = 0.5 * ext.belt_play_rad;
  belt_creep_ = ext.belt_creep_sd_rad_per_sqrt_rad;
  belt_skip_ = ext.belt_skip_per_rad;
  belt_step_ = ext.belt_tooth_step_rad;
  belt_on_ = belt_half_play_ > 0.0 || belt_creep_ > 0.0 || belt_skip_ > 0.0;
}

void Potentiometer::reset(uint64_t seed) {
  noise_.seed(seed, kStreamPotNoise);
  belt_rng_.seed(seed, kStreamBeltSlip);
  belt_started_ = false;
  belt_shaft_ = 0.0;
  belt_offset_ = 0.0;
  captured_ = false;
  captured_clipped_ = false;
  captured_raw_ = 0;
  clipped_ = false;
}

int Potentiometer::sample(double motor_angle, double link_angle, double t, double applied_pwm, double supply_scale,
                          const PotFaults &faults) {
  const double n = static_cast<double>(n_);
  double theta = cfg_.at_motor ? motor_angle : link_angle;
  if (belt_on_) {
    if (!belt_started_) {
      belt_started_ = true;
      belt_shaft_ = theta;
    }
    const double before = belt_shaft_;
    if (belt_shaft_ < theta - belt_half_play_) belt_shaft_ = theta - belt_half_play_;
    if (belt_shaft_ > theta + belt_half_play_) belt_shaft_ = theta + belt_half_play_;
    const double travel = std::fabs(belt_shaft_ - before);
    if (travel > 0.0) {
      if (belt_creep_ > 0.0) belt_offset_ += belt_creep_ * std::sqrt(travel) * belt_rng_.normal();
      if (belt_skip_ > 0.0 && belt_rng_.uniform() < -std::expm1(-belt_skip_ * travel)) {
        belt_offset_ += belt_rng_.uniform() < 0.5 ? -belt_step_ : belt_step_;
      }
    }
    theta = belt_shaft_ + belt_offset_;
  }
  theta += cfg_.drift_rad_per_s * t;
  if (faults.reversed) theta = -theta;
  theta += faults.slip_rad + faults.drift_rad;
  const double center = static_cast<double>(cfg_.center_raw);
  const double r0 = center + g_ * theta;
  const double r1 = cfg_.linearity_frac != 0.0 ? r0 + cfg_.linearity_frac * n * std::sin(kTwoPi * (r0 - center) / n) : r0;
  const double s_v = supply_scale * (1.0 - cfg_.supply_sag_per_pwm * std::fabs(applied_pwm));
  const double r2 = cfg_.ratiometric ? r1 : s_v * r1;
  const double eps = noise_.normal() * cfg_.noise_sd_counts * faults.noise_scale;
  double value = r2 + eps;
  // Guard before the int conversion (a NaN there is undefined behaviour); exit code 9 via main().
  if (!(value == value)) throw std::runtime_error("potentiometer: raw value is not a number");
  if (value > 1e9) value = 1e9;
  if (value < -1e9) value = -1e9;
  value = round_half_away(value) + static_cast<double>(faults.jump);
  int raw;
  bool clipped = false;
  if (value < 0.0) {
    raw = 0;
    clipped = true;
  } else if (value > n - 1.0) {
    raw = n_ - 1;
    clipped = true;
  } else {
    raw = static_cast<int>(value);
  }
  if (faults.stuck) {
    if (!captured_) {
      captured_ = true;
      captured_raw_ = raw;
      captured_clipped_ = clipped;
    }
    raw = captured_raw_;
    clipped = captured_clipped_;
  }
  if (faults.shorted) {
    raw = n_ - 1;
    clipped = false;
  }
  if (faults.open) {
    raw = 0;
    clipped = false;
  }
  clipped_ = clipped;
  return raw;
}

}  // namespace ubts_steer
