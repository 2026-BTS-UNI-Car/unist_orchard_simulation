// Steering actuator model (see actuator.h).
#include "actuator.h"

#include <cmath>

namespace ubts_steer {

namespace {

double clamp(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

const double kMaxBackdriveRateFactor = 4.0;  // cap of the torque-driven speed (x the rate limit)

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
      free_(false),
      torque_(0.0),
      impact_torque_(0.0),
      impact_left_s_(0.0) {}

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

void Actuator::configure_ext(const ExtConfig &ext) {
  ext_ = ext;
  push_.configure(ext.push_sd_nm, ext.push_corr_length_m);
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
  torque_ = 0.0;
  impact_torque_ = 0.0;
  impact_left_s_ = 0.0;
  if (ext_.present) {  // the extension streams exist only with the block (no draw otherwise)
    impact_rng_.seed(seed, kStreamImpact);
    if (ext_.push_sd_nm > 0.0) push_.reset(seed, kStreamPush);
  }
}

double Actuator::terrain_torque(double ds) {
  // Ground impacts: one rectangular pulse at a time, started by a Poisson event in distance; a new event replaces
  // the running pulse. Pushing torque: OU process in distance. Both only while the vehicle moves (ds > 0).
  if (impact_left_s_ > 0.0) {
    impact_left_s_ -= dt_;
    if (!(impact_left_s_ > 1e-12)) {
      impact_left_s_ = 0.0;
      impact_torque_ = 0.0;
    }
  }
  if (ds > 0.0 && ext_.impact_rate_per_m > 0.0) {
    const double p = -std::expm1(-ext_.impact_rate_per_m * ds);
    if (impact_rng_.uniform() < p) {
      impact_torque_ = ext_.impact_torque_sd_nm * impact_rng_.normal();
      impact_left_s_ = ext_.impact_duration_s;
    }
  }
  double tau = impact_torque_;
  if (ds > 0.0 && ext_.push_sd_nm > 0.0) push_.advance(ds);
  if (ext_.push_sd_nm > 0.0) tau += push_.value();
  return tau;
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
  double rate = (stopped ? cfg_.rate_standstill_rad_s : cfg_.rate_max_rad_s) * faults.rate_scale;
  if (ext_.present && ext_.supply_rate_scale != 1.0) rate *= ext_.supply_rate_scale;
  const bool l0 = cfg_.level == kLevelL0;
  const bool driving = l0 ? enable : applied != 0.0;
  const double ds = stopped ? 0.0 : speed_abs * dt_;

  // External torque (steer_ext and steer_torque injections; L1 only, refused with L0 at startup).
  double tau = faults.torque_nm;
  if (ext_.present) tau += terrain_torque(ds);
  torque_ = tau;
  // Equivalent duty of tau and the speed gain (coast: no electrical braking).
  double d_tau = 0.0, torque_gain = 1.0;
  if (!l0 && tau != 0.0 && !faults.jam) {
    const bool coast = !enable || faults.driver_dead;
    if (driving || !cfg_.self_locking) {
      d_tau = tau / cfg_.stall_torque_nm;
    } else {
      const double breakaway = ext_.self_lock_breakaway_nm;
      const double excess = std::fabs(tau) - breakaway;
      if (excess > 0.0) d_tau = (tau > 0.0 ? excess : -excess) / cfg_.stall_torque_nm;
    }
    if (!driving && coast && ext_.present) torque_gain = ext_.backdrive_coast_gain;
  }

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
    if (!l0 && driving && d_tau == 0.0) {
      const double min_pwm = stopped ? cfg_.min_pwm_standstill : cfg_.min_pwm;
      const double au = std::fabs(u);
      if (au > min_pwm) w_ss = (u > 0.0 ? rate : -rate) * (au - min_pwm) / (1.0 - min_pwm);
    } else if (!l0 && d_tau != 0.0) {
      // motor torque (if any) plus the external torque against the static friction (the start duty)
      const double min_pwm = stopped ? cfg_.min_pwm_standstill : cfg_.min_pwm;
      const double ue = (driving ? u : 0.0) + d_tau;
      const double au = std::fabs(ue);
      if (au > min_pwm) {
        const double cap = kMaxBackdriveRateFactor * rate;
        const double w = clamp(rate * torque_gain * (au - min_pwm) / (1.0 - min_pwm), 0.0, cap);
        w_ss = ue > 0.0 ? w : -w;
      }
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
  if (tau != 0.0 && !l0) theta_l_ = tau > 0.0 ? theta_m_ + h : theta_m_ - h;  // the torque closes the play
  theta_l_ = clamp(theta_l_, link_min_, link_max_);
  if (free) theta_m_ = clamp(theta_m_, theta_l_ - h, theta_l_ + h);  // back-driven through the play

  applied_ = applied;
  free_ = free;
}

}  // namespace ubts_steer
