// Native steering plant-side model: actuator + potentiometer + test servo + fault injections
// + the tick clock of the lockstep protocol (sections 5 to 7, 9). No MCU firmware logic (P3).
#ifndef UBTS_STEER_MODEL_H
#define UBTS_STEER_MODEL_H

#include <stdint.h>

#include <vector>

#include "actuator.h"
#include "potentiometer.h"
#include "steer_config.h"
#include "steer_geometry.h"

namespace ubts_steer {

// model_flags bits (protocol section 9, backend_api.ModelFlag).
const uint16_t kFlagEndStopLeft = 1u << 0;
const uint16_t kFlagEndStopRight = 1u << 1;
const uint16_t kFlagInBacklash = 1u << 2;
const uint16_t kFlagFreeWheeling = 1u << 3;
const uint16_t kFlagInjectionActive = 1u << 4;
const uint16_t kFlagPotClipped = 1u << 5;

struct StepInput {
  double dt_s, steer_pwm, target_rad, speed_m_s, supply_scale;
  bool steer_en, stopped;
};

struct StepOutput {
  double sim_time_s, delta_left_rad, delta_right_rad, delta_eq_rad, motor_angle_rad, applied_pwm;
  uint32_t ticks;
  uint16_t pot_raw, fault_bits, model_flags;
  bool steer_en;
};

class SteerModel {
 public:
  explicit SteerModel(const Config &cfg);
  // RESET angle rule: finite, |delta| < pi/2 and inside the equivalent mechanical limits.
  bool reset_angle_valid(double delta_eq) const;
  void reset(double delta_eq);
  // Runs the ticks of one plant step (cumulative tick rule); returns the tick count.
  uint32_t step(const StepInput &in);
  void output(uint32_t ticks, StepOutput *out) const;
  bool finite_state() const;

  // Firmware mode (protocol v2, section 11): the session runs the ticks one by one.
  int64_t ticks_total() const { return ticks_total_; }
  // Tick count N of a step of dt_s by the cumulative rule; the state does not change.
  int64_t ticks_for(double dt_s) const;
  // After the N ticks of that step: plant_time += dt_s, ticks_total = the rounded target (same rule as step()).
  void advance_clock(double dt_s);
  // One tick n: actuator with the driver command (D6 - D7) / 255 and EN (D2), then the ADC sample n + 1.
  // driver_off: steering motor supply off, the driver output is 0 whatever the pins.
  void fw_tick(int64_t n, double command, bool enable, bool driver_off, bool stopped, double speed_abs,
               double supply_scale);
  int pot_raw() const { return pot_raw_; }

 private:
  struct Timed {
    Injection injection;
    int64_t k_at;   // first tick index (actuator) or sample index (pot) at or after at_s
    int64_t k_end;  // pot_jump: first sample index at or after at_s + duration_s
  };

  int64_t first_index_at(double t) const;
  ActuatorFaults actuator_faults(int64_t tick_index) const;
  PotFaults pot_faults(int64_t sample_index) const;
  bool injection_active(int64_t index) const;
  double servo_law(double target) const;
  void take_sample(int64_t sample_index, double supply_scale);

  Config cfg_;
  LinkMap map_;
  Actuator actuator_;
  Potentiometer pot_;
  std::vector<Timed> timed_;
  double link_min_, link_max_, tick_dt_;
  int64_t ticks_per_servo_;
  double plant_time_;
  int64_t ticks_total_;
  double servo_cmd_;
  bool steer_en_;
  int pot_raw_;
};

}  // namespace ubts_steer

#endif  // UBTS_STEER_MODEL_H
