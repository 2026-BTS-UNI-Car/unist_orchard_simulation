// Potentiometer and ADC model (design 3.4, protocol section 7). One call = one ADC sample.
//
//   theta_p = (motor-side angle if location motor, link-side angle if wheel) + drift * t
//   pot_reversed flips the sign of theta_p; pot_slip adds offsets; pot_drift adds rate * (t - at_s)
//   N = 2^bits, g = N * to_steer_ratio * (1 + gain_error) / range
//   r0 = center + g * theta_p
//   r1 = r0 + linearity * N * sin(2 pi (r0 - center) / N)
//   s_v = supply_scale * (1 - sag * |applied PWM|); r2 = s_v * r1 (r1 if ratiometric)
//   raw = clamp(round_half_away(r2 + eps) + jump counts, 0, N - 1), eps ~ N(0, sd * noise scale)
// Belt (optional steer_ext block, profile schema v3): before all of that the measured angle passes the belt. The pot
// shaft angle theta_s (steering scale) follows theta_meas through a play of width belt_play (play operator), and a
// slip offset is added: a friction-belt creep (random walk, sd belt_creep x sqrt(travel)) and tooth skips (Poisson in
// pot-side travel, +-one tooth step each). Without the block theta_s = theta_meas and no draw is made.
// Overrides after that: pot_stuck holds the first sample at or after at_s, pot_short gives
// N - 1, pot_open gives 0 (open wins). One noise draw per sample, always, so the stream does
// not depend on the fault state.
#ifndef UBTS_STEER_POTENTIOMETER_H
#define UBTS_STEER_POTENTIOMETER_H

#include <stdint.h>

#include "steer_config.h"
#include "steer_rng.h"

namespace ubts_steer {

struct PotFaults {
  PotFaults()
      : open(false), shorted(false), stuck(false), reversed(false), noise_scale(1.0), jump(0), slip_rad(0.0),
        drift_rad(0.0) {}
  bool open, shorted, stuck, reversed;
  double noise_scale;  // product of active pot_noise_x10 scales
  long jump;           // sum of active pot_jump counts
  double slip_rad;     // sum of active pot_slip offsets
  double drift_rad;    // sum of active pot_drift rate * (t - at_s)
};

class Potentiometer {
 public:
  Potentiometer();
  void configure(const PotConfig &cfg);
  void configure_belt(const ExtConfig &ext);  // optional steer_ext belt terms (after configure)
  void reset(uint64_t seed);  // new noise stream, stuck capture cleared, belt state restarted
  double belt_offset() const { return belt_offset_; }  // accumulated belt slip (rad, steering scale)
  int sample(double motor_angle, double link_angle, double t, double applied_pwm, double supply_scale,
             const PotFaults &faults);
  bool clipped() const { return clipped_; }
  int full_scale() const { return n_ - 1; }

 private:
  PotConfig cfg_;
  int n_;
  double g_;
  Stream noise_;
  bool captured_, captured_clipped_, clipped_;
  int captured_raw_;
  // steer_ext belt
  bool belt_on_, belt_started_;
  double belt_half_play_, belt_creep_, belt_skip_, belt_step_, belt_shaft_, belt_offset_;
  Stream belt_rng_;
};

}  // namespace ubts_steer

#endif  // UBTS_STEER_POTENTIOMETER_H
