// Steering actuator model (see actuator.h).
#include "actuator.h"

#include <cmath>

namespace ubts_steer {

namespace {

double clamp(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

}  // namespace

double DistanceOu::advance(double ds) {
  if (ds != cached_ds_) {
    rho_ = std::exp(-ds / corr_);
    gain_ = sd_ * std::sqrt(-std::expm1(-2.0 * ds / corr_));
    cached_ds_ = ds;
  }
  const double next = rho_ * x_ + gain_ * rng_.normal();
  const double increment = next - x_;
  x_ = next;
  return increment;
}

Actuator::Actuator()
    : link_min_(0.0),
      link_max_(0.0),
      theta_straight_(0.0),
      dt_(1e-3),
      half_play_(0.0),
      e_tau_(0.0),
      one_minus_e_tau_(1.0),
      tau_one_minus_e_tau_(0.0),
      centre_k_per_m_(0.0),
      cached_ds_(-1.0),
      centre_factor_(1.0),
      theta_m_(0.0),
      omega_m_(0.0),
      theta_l_(0.0),
      applied_(0.0),
      free_(false) {}

void Actuator::configure(const ActuatorConfig &cfg, double link_min, double link_max, double theta_straight,
                         double tick_dt) {
  cfg_ = cfg;
  link_min_ = link_min;
  link_max_ = link_max;
  theta_straight_ = theta_straight;
  dt_ = tick_dt;
  half_play_ = 0.5 * cfg.backlash_rad;
  one_minus_e_tau_ = -std::expm1(-tick_dt / cfg.tau_s);
  e_tau_ = 1.0 - one_minus_e_tau_;
  tau_one_minus_e_tau_ = cfg.tau_s * one_minus_e_tau_;
  centre_k_per_m_ = cfg.release_scale * cfg.release_centering_per_s_per_m_s;
  wander_.configure(cfg.wander_sd_rad, cfg.wander_corr_length_m);
  release_.configure(cfg.release_noise_sd_rad, cfg.release_noise_corr_length_m);
  cached_ds_ = -1.0;
}

void Actuator::reset(double link_angle, uint64_t seed) {
  theta_l_ = clamp(link_angle, link_min_, link_max_);
  theta_m_ = theta_l_;
  omega_m_ = 0.0;
  applied_ = 0.0;
  free_ = !cfg_.self_locking;
  wander_.reset(seed, kStreamBacklashWander);
  release_.reset(seed, kStreamReleaseNoise);
  cached_ds_ = -1.0;
}

bool Actuator::in_backlash() const {
  if (!(half_play_ > 0.0)) return false;
  return std::fabs(theta_m_ - theta_l_) < half_play_ * (1.0 - 1e-9);
}

void Actuator::update_distance_terms(double ds) {
  if (ds != cached_ds_) {
    centre_factor_ = std::exp(-centre_k_per_m_ * ds);
    cached_ds_ = ds;
  }
}

void Actuator::tick(double command_pwm, bool enable, bool stopped, double speed_abs, const ActuatorFaults &faults,
                    double l0_target_pot, bool pot_at_motor) {
  const double h = half_play_;

  // Driver output (before the wiring polarity).
  double applied;
  if (faults.driver_dead) {
    applied = 0.0;
  } else if (faults.stuck_on) {
    applied = enable ? faults.stuck_pwm : 0.0;
  } else {
    applied = enable ? command_pwm : 0.0;
  }
  const double u = faults.polarity_reversed ? -applied : applied;
  const double rate = (stopped ? cfg_.rate_standstill_rad_s : cfg_.rate_max_rad_s) * faults.rate_scale;
  const bool l0 = cfg_.level == kLevelL0;
  const bool driving = l0 ? enable : applied != 0.0;

  // Motor side.
  if (faults.jam) {
    omega_m_ = 0.0;
    if (l0) applied = 0.0;
  } else if (l0 && driving) {
    const double pot_side = pot_at_motor ? theta_m_ : theta_l_;
    const double limit = rate * dt_;
    const double d = clamp((l0_target_pot - pot_side) * one_minus_e_tau_, -limit, limit);
    theta_m_ += d;
    omega_m_ = d / dt_;
    applied = limit > 0.0 ? clamp(d / limit, -1.0, 1.0) : 0.0;
  } else {
    // TODO(L1b): torque-speed line of the steering motor against the moving / standstill load
    // torques (design 3.3 L1b). Needs the motor no-load speed and the load model (Q11); the
    // startup refuses level L1b until then.
    double w_ss = 0.0;
    if (!l0 && driving) {
      const double min_pwm = stopped ? cfg_.min_pwm_standstill : cfg_.min_pwm;
      const double au = std::fabs(u);
      if (au > min_pwm) w_ss = (u > 0.0 ? rate : -rate) * (au - min_pwm) / (1.0 - min_pwm);
    }
    if (l0) applied = 0.0;
    const double dw = omega_m_ - w_ss;
    theta_m_ += w_ss * dt_ + dw * tau_one_minus_e_tau_;
    omega_m_ = w_ss + dw * e_tau_;
  }
  if (theta_m_ > link_max_ + h) {
    theta_m_ = link_max_ + h;
    if (omega_m_ > 0.0) omega_m_ = 0.0;
  } else if (theta_m_ < link_min_ - h) {
    theta_m_ = link_min_ - h;
    if (omega_m_ < 0.0) omega_m_ = 0.0;
  }

  // Link side.
  const bool free = !driving && !cfg_.self_locking && !faults.jam;
  theta_l_ = clamp(theta_l_, theta_m_ - h, theta_m_ + h);  // the motor side drags the link
  const double ds = stopped ? 0.0 : speed_abs * dt_;
  if (ds > 0.0) {
    // Both processes advance with the distance in every mode so that their streams stay aligned.
    const double wander = wander_.advance(ds);
    const double noise = release_.advance(ds);
    if (free) {
      update_distance_terms(ds);
      theta_l_ = theta_straight_ + (theta_l_ - theta_straight_) * centre_factor_ + noise;
    } else {
      theta_l_ = clamp(theta_l_ + wander, theta_m_ - h, theta_m_ + h);
    }
  }
  theta_l_ = clamp(theta_l_, link_min_, link_max_);
  if (free) theta_m_ = clamp(theta_m_, theta_l_ - h, theta_l_ + h);  // back-driven through the play

  applied_ = applied;
  free_ = free;
}

}  // namespace ubts_steer
