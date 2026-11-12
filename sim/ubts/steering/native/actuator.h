// Steering actuator model (design 3.3, protocol section 7). Plant side: truth only.
//
// State on the steering equivalent-angle scale (rad, + left):
//   theta_m  motor-side angle (gearbox output, before the backlash)
//   omega_m  motor-side speed (rad/s)
//   theta_l  link-side angle (after the backlash); |theta_m - theta_l| <= b/2
//
// Level L1 (default): steady speed w_ss = sign(u) * rate * (|u| - min) / (1 - min) when
// |u| > min, else 0; omega_m follows w_ss with a first-order lag tau (exact discretisation).
// rate/min are the moving or standstill values (vehicle_stopped), rate times the product of
// active `slow` scales. u is the driver output after the wiring polarity.
// Level L0 (test_servo only): ideal servo. The motor moves the pot-side angle toward the
// pot-side target with a first-order lag tau and the rate limit; applied_pwm reports the
// normalised effort (fraction of the rate limit), not a PWM.
// Level L1b is not implemented (TODO, startup refuses it).
//
// Tick order: driver output -> motor update -> motor end stop -> link update -> link end stops.
// Modes of the link (protocol 7, decisions recorded in the native notes):
//   held     (drive torque present, or no torque but self-locking, or jam): the motor side
//            drags the link at the ends of the play (play operator); while the vehicle moves
//            the link wanders inside the play by the increments of a distance-based OU
//            process (backlash_wander_sd_rad, backlash_wander_corr_length_m).
//   free     (no drive torque, not self-locking, no jam): while the vehicle moves the link
//            relaxes toward the straight-wheel link angle at rate k*|v| (k = release_scale *
//            release_centering) plus the increments of a second OU process (release noise);
//            the link drags the motor side back through the play. Standing still: nothing.
// "No drive torque" (L1) means driver output 0: EN off, PWM 0, driver_dead.
// End stops: the link is limited to the link angles whose wheel-side equivalent angle is
// at -eq_mech_limit_right / +eq_mech_limit_left; the motor side to those plus b/2 (stall,
// speed set to 0).
// External torque (optional steer_ext block, profile schema v3; docs/steering/STEER_EXT_MODEL_KO.md): tau (N m on the
// steering axis, + left) = active steer_torque injections + ground impacts (Poisson in distance, one rectangular pulse
// at a time) + a pushing torque (distance OU). It enters the L1 speed law as an equivalent duty d = tau / stall_torque:
//   driving (driver output u != 0):  u_eff = u + d
//   brake   (EN on, output 0):       u_eff = d   (non-self-locking; self-locking: d minus the breakaway torque)
//   coast   (EN off / driver dead):  u_eff = d, speed x backdrive_coast_gain (no electrical braking)
// w_ss = sign(u_eff) * rate * (|u_eff| - min) / (1 - min) when |u_eff| > min (min = the moving / standstill start
// duty, the static friction), capped at kMaxBackdriveRateFactor * rate. A non-zero tau keeps the play closed toward
// tau (the link at the play end on the side of tau). With tau = 0 every step is the same as without the block.
#ifndef UBTS_STEER_ACTUATOR_H
#define UBTS_STEER_ACTUATOR_H

#include <stdint.h>

#include "steer_config.h"
#include "steer_rng.h"

namespace ubts_steer {

// Actuator fault injections active in one tick.
struct ActuatorFaults {
  ActuatorFaults()
      : jam(false),
        driver_dead(false),
        polarity_reversed(false),
        stuck_on(false),
        stuck_pwm(0.0),
        rate_scale(1.0),
        torque_nm(0.0) {}
  bool jam;                // motor-side angle frozen
  bool driver_dead;        // driver output always 0
  bool polarity_reversed;  // motor wiring reversed
  bool stuck_on;           // driver output stuck at stuck_pwm while EN is on
  double stuck_pwm;
  double rate_scale;       // product of active `slow` scales
  double torque_nm;        // sum of active steer_torque injections (N m, + left)
};

// Ornstein-Uhlenbeck process in travelled distance: x <- rho x + sd sqrt(1 - rho^2) N(0, 1),
// rho = exp(-ds / corr_length). Started from its stationary distribution at reset.
class DistanceOu {
 public:
  DistanceOu() : sd_(0.0), corr_(1.0), x_(0.0), cached_ds_(-1.0), rho_(1.0), gain_(0.0) {}
  void configure(double sd, double corr_length_m) {
    sd_ = sd;
    corr_ = corr_length_m;
    cached_ds_ = -1.0;
  }
  void reset(uint64_t seed, uint64_t tag) {
    rng_.seed(seed, tag);
    x_ = sd_ * rng_.normal();
    cached_ds_ = -1.0;
  }
  // Advance by the distance ds > 0 and return the increment of the process.
  double advance(double ds);
  double value() const { return x_; }

 private:
  double sd_, corr_, x_, cached_ds_, rho_, gain_;
  Stream rng_;
};

class Actuator {
 public:
  Actuator();
  // link_min/link_max: link-side end stops; theta_straight: link angle with straight wheels.
  void configure(const ActuatorConfig &cfg, double link_min, double link_max, double theta_straight, double tick_dt);
  // Optional steer_ext block (call after configure; default: off).
  void configure_ext(const ExtConfig &ext);
  // Motor side = link side (centre of the play), speed 0, noise streams restarted from the seed.
  void reset(double link_angle, uint64_t seed);
  // One MCU tick. command_pwm: driver command in [-1, 1] (direct_pwm request or test servo);
  // l0_target_pot: pot-side target (L0 only); pot_at_motor: where the pot measures (L0 only).
  void tick(double command_pwm, bool enable, bool stopped, double speed_abs, const ActuatorFaults &faults,
            double l0_target_pot, bool pot_at_motor);

  double motor_angle() const { return theta_m_; }
  double link_angle() const { return theta_l_; }
  double motor_speed() const { return omega_m_; }
  double applied_pwm() const { return applied_; }
  bool free_wheeling() const { return free_; }
  bool in_backlash() const;
  bool at_left_stop() const { return theta_l_ >= link_max_; }
  bool at_right_stop() const { return theta_l_ <= link_min_; }
  double external_torque() const { return torque_; }  // tau of the last tick (N m)

 private:
  void update_distance_terms(double ds);

  ActuatorConfig cfg_;
  double link_min_, link_max_, theta_straight_, dt_, half_play_;
  double e_tau_, one_minus_e_tau_, tau_one_minus_e_tau_;
  double centre_k_per_m_;
  double cached_ds_, centre_factor_;
  double theta_m_, omega_m_, theta_l_, applied_;
  bool free_;
  DistanceOu wander_, release_;
  // steer_ext
  ExtConfig ext_;
  double torque_, impact_torque_, impact_left_s_;
  DistanceOu push_;
  Stream impact_rng_;
  double terrain_torque(double ds);
};

}  // namespace ubts_steer

#endif  // UBTS_STEER_ACTUATOR_H
