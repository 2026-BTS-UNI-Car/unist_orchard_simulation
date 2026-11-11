// Native steering plant-side model (see steer_model.h).
#include "steer_model.h"

#include <cmath>

namespace ubts_steer {

namespace {

const double kHalfPi = 1.57079632679489661923;

double clamp(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

bool is_finite(double x) { return x == x && x <= 1.7976931348623157e308 && x >= -1.7976931348623157e308; }

// llround for x >= 0 (round half away from zero), exact for x < 2^53.
int64_t round_nonnegative(double x) {
  const double whole = std::floor(x);
  int64_t r = static_cast<int64_t>(whole);
  if (x - whole >= 0.5) ++r;
  return r;
}

}  // namespace

SteerModel::SteerModel(const Config &cfg)
    : cfg_(cfg),
      link_min_(0.0),
      link_max_(0.0),
      tick_dt_(1.0 / cfg.tick_hz),
      ticks_per_servo_(cfg.tick_hz / cfg.servo.servo_hz),
      plant_time_(0.0),
      ticks_total_(0),
      servo_cmd_(0.0),
      steer_en_(false),
      pot_raw_(0) {
  const GeometryConfig &g = cfg_.geometry;
  map_.set(g.link_pot_rad, g.link_wheel_rad, cfg_.actuator.lr_asymmetry, cfg_.actuator.zero_offset_rad);
  link_max_ = map_.link_from_delta_eq(g.eq_mech_left_rad);
  link_min_ = map_.link_from_delta_eq(-g.eq_mech_right_rad);
  const double straight = clamp(map_.link_from_delta_eq(0.0), link_min_, link_max_);
  actuator_.configure(cfg_.actuator, link_min_, link_max_, straight, tick_dt_);
  pot_.configure(cfg_.pot);
  for (size_t i = 0; i < cfg_.injections.size(); ++i) {
    Timed t;
    t.injection = cfg_.injections[i];
    t.k_at = first_index_at(t.injection.at_s);
    t.k_end = t.injection.kind == kInjPotJump ? first_index_at(t.injection.at_s + t.injection.duration_s)
                                               : static_cast<int64_t>(0x7fffffffffffffffLL);
    timed_.push_back(t);
  }
}

int64_t SteerModel::first_index_at(double t) const {
  // smallest k with k / tick_hz >= t (a tolerance of 1e-9 tick absorbs decimal rounding)
  double k = std::ceil(t * static_cast<double>(cfg_.tick_hz) - 1e-9);
  if (k < 0.0) k = 0.0;
  if (k > 9.0e15) k = 9.0e15;
  return static_cast<int64_t>(k);
}

bool SteerModel::reset_angle_valid(double delta_eq) const {
  return is_finite(delta_eq) && std::fabs(delta_eq) < kHalfPi && delta_eq <= cfg_.geometry.eq_mech_left_rad &&
         delta_eq >= -cfg_.geometry.eq_mech_right_rad;
}

void SteerModel::reset(double delta_eq) {
  plant_time_ = 0.0;
  ticks_total_ = 0;
  servo_cmd_ = 0.0;
  steer_en_ = false;
  actuator_.reset(clamp(map_.link_from_delta_eq(delta_eq), link_min_, link_max_), cfg_.seed);
  pot_.reset(cfg_.seed);
  take_sample(0, 1.0);
}

ActuatorFaults SteerModel::actuator_faults(int64_t tick_index) const {
  ActuatorFaults f;
  double stuck_at = -1.0;
  for (size_t i = 0; i < timed_.size(); ++i) {
    const Timed &t = timed_[i];
    if (tick_index < t.k_at) continue;
    switch (t.injection.kind) {
      case kInjJam: f.jam = true; break;
      case kInjSlow: f.rate_scale *= t.injection.value; break;
      case kInjDriverDead: f.driver_dead = true; break;
      case kInjDriverStuckOn:
        if (t.injection.at_s >= stuck_at) {  // the latest activated one wins
          stuck_at = t.injection.at_s;
          f.stuck_on = true;
          f.stuck_pwm = t.injection.value;
        }
        break;
      case kInjPolarityReversed: f.polarity_reversed = true; break;
      default: break;
    }
  }
  return f;
}

PotFaults SteerModel::pot_faults(int64_t sample_index) const {
  PotFaults f;
  const double t = static_cast<double>(sample_index) * tick_dt_;
  for (size_t i = 0; i < timed_.size(); ++i) {
    const Timed &x = timed_[i];
    if (sample_index < x.k_at) continue;
    switch (x.injection.kind) {
      case kInjPotOpen: f.open = true; break;
      case kInjPotShort: f.shorted = true; break;
      case kInjPotStuck: f.stuck = true; break;
      case kInjPotNoiseX10: f.noise_scale *= x.injection.value; break;
      case kInjPotJump:
        if (sample_index < x.k_end) f.jump += x.injection.counts;
        break;
      case kInjPotSlip: f.slip_rad += x.injection.value; break;
      case kInjPotDrift: f.drift_rad += x.injection.value * (t - x.injection.at_s); break;
      case kInjPotReversed: f.reversed = true; break;
      default: break;
    }
  }
  return f;
}

bool SteerModel::injection_active(int64_t index) const {
  for (size_t i = 0; i < timed_.size(); ++i) {
    if (index >= timed_[i].k_at && index < timed_[i].k_end) return true;
  }
  return false;
}

double SteerModel::servo_law(double target) const {
  // Test-only P servo (protocol section 6): measured angle = T(noise-free pot-side truth).
  const double pot_side = cfg_.pot.at_motor ? actuator_.motor_angle() : actuator_.link_angle();
  const double error = target - map_.table(pot_side);
  const double magnitude = std::fabs(error);
  if (magnitude <= cfg_.servo.deadband_rad) return 0.0;
  const double pwm = clamp(cfg_.servo.kp_per_rad * magnitude, cfg_.servo.min_pwm, cfg_.servo.max_pwm);
  return error > 0.0 ? pwm : -pwm;
}

void SteerModel::take_sample(int64_t sample_index, double supply_scale) {
  const double t = static_cast<double>(sample_index) * tick_dt_;
  pot_raw_ = pot_.sample(actuator_.motor_angle(), actuator_.link_angle(), t, actuator_.applied_pwm(), supply_scale,
                         pot_faults(sample_index));
}

uint32_t SteerModel::step(const StepInput &in) {
  plant_time_ += in.dt_s;
  const int64_t target = round_nonnegative(plant_time_ * static_cast<double>(cfg_.tick_hz));
  const int64_t ticks = target - ticks_total_;
  const double speed_abs = std::fabs(in.speed_m_s);
  const bool servo = cfg_.mode == kModeTestServo;
  const double l0_target = servo ? map_.table_inverse(in.target_rad) : 0.0;
  for (int64_t i = 0; i < ticks; ++i) {
    const int64_t n = ticks_total_ + i;
    // (1) command
    if (servo && n % ticks_per_servo_ == 0) servo_cmd_ = servo_law(in.target_rad);
    const double command = servo ? servo_cmd_ : in.steer_pwm;
    // (2) integrate one tick
    actuator_.tick(command, in.steer_en, in.stopped, speed_abs, actuator_faults(n), l0_target, cfg_.pot.at_motor);
    steer_en_ = in.steer_en;
    // (3) one ADC sample at the end of the tick
    take_sample(n + 1, in.supply_scale);
  }
  if (ticks > 0) ticks_total_ = target;
  return static_cast<uint32_t>(ticks);
}

int64_t SteerModel::ticks_for(double dt_s) const {
  const double plant_time = plant_time_ + dt_s;
  return round_nonnegative(plant_time * static_cast<double>(cfg_.tick_hz)) - ticks_total_;
}

void SteerModel::advance_clock(double dt_s) {
  plant_time_ += dt_s;
  const int64_t target = round_nonnegative(plant_time_ * static_cast<double>(cfg_.tick_hz));
  if (target > ticks_total_) ticks_total_ = target;
}

void SteerModel::fw_tick(int64_t n, double command, bool enable, bool driver_off, bool stopped, double speed_abs,
                         double supply_scale) {
  ActuatorFaults faults = actuator_faults(n);
  if (driver_off) faults.driver_dead = true;  // steering motor supply off: no output and no torque
  actuator_.tick(command, enable, stopped, speed_abs, faults, 0.0, cfg_.pot.at_motor);
  steer_en_ = enable;
  take_sample(n + 1, supply_scale);
}

void SteerModel::output(uint32_t ticks, StepOutput *out) const {
  const GeometryConfig &g = cfg_.geometry;
  out->sim_time_s = static_cast<double>(ticks_total_) / static_cast<double>(cfg_.tick_hz);
  const double delta = clamp(map_.delta_eq(actuator_.link_angle()), -g.eq_mech_right_rad, g.eq_mech_left_rad);
  double left = 0.0, right = 0.0;
  wheel_angles(delta, g.linkage, g.wheelbase_m, g.kingpin_width_m, g.ackermann_ratio, &left, &right);
  // Rounding guard only: the equivalent limits already keep both wheels inside their limits.
  out->delta_left_rad = clamp(left, -g.wheel_limit_rad[kLeftTowardRight], g.wheel_limit_rad[kLeftTowardLeft]);
  out->delta_right_rad = clamp(right, -g.wheel_limit_rad[kRightTowardRight], g.wheel_limit_rad[kRightTowardLeft]);
  out->delta_eq_rad = delta;
  out->motor_angle_rad = actuator_.motor_angle();
  out->applied_pwm = actuator_.applied_pwm();
  out->ticks = ticks;
  out->pot_raw = static_cast<uint16_t>(pot_raw_);
  out->fault_bits = 0;
  uint16_t flags = 0;
  if (actuator_.at_left_stop()) flags |= kFlagEndStopLeft;
  if (actuator_.at_right_stop()) flags |= kFlagEndStopRight;
  if (actuator_.in_backlash()) flags |= kFlagInBacklash;
  if (actuator_.free_wheeling()) flags |= kFlagFreeWheeling;
  if (injection_active(ticks_total_)) flags |= kFlagInjectionActive;
  if (pot_.clipped()) flags |= kFlagPotClipped;
  out->model_flags = flags;
  out->steer_en = steer_en_;
}

bool SteerModel::finite_state() const {
  return is_finite(actuator_.motor_angle()) && is_finite(actuator_.link_angle()) &&
         is_finite(actuator_.motor_speed()) && is_finite(actuator_.applied_pwm()) && is_finite(plant_time_);
}

}  // namespace ubts_steer
