// Potentiometer and ADC model (design 3.4, protocol section 7). One call = one ADC sample.
//
//   theta_p = (motor-side angle if location motor, link-side angle if wheel) + drift * t
//   pot_reversed flips the sign of theta_p; pot_slip adds offsets; pot_drift adds rate * (t - at_s)
//   N = 2^bits, g = N * to_steer_ratio * (1 + gain_error) / range
//   r0 = center + g * theta_p
//   r1 = r0 + linearity * N * sin(2 pi (r0 - center) / N)
//   s_v = supply_scale * (1 - sag * |applied PWM|); r2 = s_v * r1 (r1 if ratiometric)
//   raw = clamp(round_half_away(r2 + eps) + jump counts, 0, N - 1), eps ~ N(0, sd * noise scale)
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
  void reset(uint64_t seed);  // new noise stream, stuck capture cleared
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
};

}  // namespace ubts_steer

#endif  // UBTS_STEER_POTENTIOMETER_H
