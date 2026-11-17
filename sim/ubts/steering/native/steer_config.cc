// Startup configuration of the native steering process (see steer_config.h).
#include "steer_config.h"

#include <stdio.h>

#include <cmath>
#include <string>

#include "steer_geometry.h"
#include "steer_json.h"

namespace ubts_steer {

namespace {

const double kPi = 3.14159265358979323846;
const double kHalfPi = 1.57079632679489661923;
const double kMaxPotGainCountsPerRad = 1e9;

const char *const kInjectionNames[kInjKindCount] = {
    "jam",      "slow",      "driver_dead",   "driver_stuck_on", "polarity_reversed", "pot_open",    "pot_short",
    "pot_stuck", "pot_noise_x10", "pot_jump", "pot_slip",        "pot_drift",         "pot_reversed",
    "steer_torque"};

std::string format_double(double x) {
  char buf[40];
  snprintf(buf, sizeof buf, "%.17g", x);
  return buf;
}

bool is_finite(double x) { return x == x && x <= 1.7976931348623157e308 && x >= -1.7976931348623157e308; }

// Read access to one JSON object with an exact key set; every error names the key path.
class ObjectView {
 public:
  ObjectView(const json::Document &doc, int index, const std::string &path) : doc_(doc), index_(index), path_(path) {
    if (doc.node(index).kind != json::kObject) throw ConfigError(path_ + ": must be an object");
  }

  void require_keys(const char *const *names, size_t count) const {
    const json::Node &node = doc_.node(index_);
    for (size_t i = 0; i < node.keys.size(); ++i) {
      bool known = false;
      for (size_t j = 0; j < count; ++j) known = known || node.keys[i] == names[j];
      if (!known) throw ConfigError(key_path(node.keys[i]) + ": unknown key");
    }
    for (size_t j = 0; j < count; ++j) {
      bool present = false;
      for (size_t i = 0; i < node.keys.size(); ++i) present = present || node.keys[i] == names[j];
      if (!present) throw ConfigError(key_path(names[j]) + ": missing key");
    }
  }

  int child(const char *name) const {
    const json::Node &node = doc_.node(index_);
    for (size_t i = 0; i < node.keys.size(); ++i) {
      if (node.keys[i] == name) return node.items[i];
    }
    throw ConfigError(key_path(name) + ": missing key");
  }

  const json::Node &node(const char *name) const { return doc_.node(child(name)); }

  std::string key_path(const std::string &name) const { return path_.empty() ? name : path_ + "." + name; }

  double number(const char *name, double lo, double hi, bool lo_open = false, bool hi_open = false) const {
    const json::Node &n = node(name);
    if (n.kind != json::kNumber) throw ConfigError(key_path(name) + ": must be a number");
    const double v = n.number;
    if (!is_finite(v)) throw ConfigError(key_path(name) + ": must be finite");
    const bool below = lo_open ? !(v > lo) : !(v >= lo);
    const bool above = hi_open ? !(v < hi) : !(v <= hi);
    if (below || above) {
      throw ConfigError(key_path(name) + ": " + format_double(v) + " outside " + (lo_open ? "(" : "[") +
                        format_double(lo) + ", " + format_double(hi) + (hi_open ? ")" : "]"));
    }
    return v;
  }

  int64_t integer(const char *name, int64_t lo, int64_t hi) const {
    const json::Node &n = node(name);
    int64_t v = 0;
    if (!json::to_int64(n, &v)) throw ConfigError(key_path(name) + ": must be an integer");
    if (v < lo || v > hi) throw ConfigError(key_path(name) + ": " + n.text + " out of range");
    return v;
  }

  bool boolean(const char *name) const {
    const json::Node &n = node(name);
    if (n.kind != json::kBool) throw ConfigError(key_path(name) + ": must be true or false");
    return n.boolean;
  }

  std::string string(const char *name) const {
    const json::Node &n = node(name);
    if (n.kind != json::kString) throw ConfigError(key_path(name) + ": must be a string");
    return n.text;
  }

  ObjectView object(const char *name) const { return ObjectView(doc_, child(name), key_path(name)); }

  // True when the object has the key with this string value (no error otherwise).
  bool has_string(const char *name, const char *value) const {
    const json::Node &node = doc_.node(index_);
    for (size_t i = 0; i < node.keys.size(); ++i) {
      if (node.keys[i] == name) {
        const json::Node &n = doc_.node(node.items[i]);
        return n.kind == json::kString && n.text == value;
      }
    }
    return false;
  }

  const json::Document &doc() const { return doc_; }

 private:
  const json::Document &doc_;
  int index_;
  std::string path_;
};

void parse_geometry(const ObjectView &g, GeometryConfig *out) {
  static const char *const keys[] = {"steer_type",
                                     "wheelbase_m",
                                     "kingpin_width_m",
                                     "ackermann_ratio",
                                     "wheel_mech_limit_rad",
                                     "eq_mech_limit_left_rad",
                                     "eq_mech_limit_right_rad",
                                     "eq_soft_limit_left_rad",
                                     "eq_soft_limit_right_rad",
                                     "link_table_rad"};
  g.require_keys(keys, sizeof keys / sizeof keys[0]);
  const std::string type = g.string("steer_type");
  if (type == "linkage") {
    out->linkage = true;
  } else if (type == "axle_pivot") {
    out->linkage = false;
  } else {
    throw ConfigError(g.key_path("steer_type") + ": must be linkage or axle_pivot");
  }
  out->wheelbase_m = g.number("wheelbase_m", 0.0, 10.0, true);
  out->kingpin_width_m = g.number("kingpin_width_m", 0.0, 10.0, true);
  out->ackermann_ratio = g.number("ackermann_ratio", 0.0, 1.0);
  const ObjectView limits = g.object("wheel_mech_limit_rad");
  static const char *const limit_keys[] = {"left_wheel_toward_left", "left_wheel_toward_right",
                                           "right_wheel_toward_left", "right_wheel_toward_right"};
  limits.require_keys(limit_keys, 4);
  for (int i = 0; i < 4; ++i) out->wheel_limit_rad[i] = limits.number(limit_keys[i], 0.0, kHalfPi, true, true);
  out->eq_mech_left_rad = g.number("eq_mech_limit_left_rad", 0.0, kHalfPi, true, true);
  out->eq_mech_right_rad = g.number("eq_mech_limit_right_rad", 0.0, kHalfPi, true, true);
  out->eq_soft_left_rad = g.number("eq_soft_limit_left_rad", 0.0, out->eq_mech_left_rad, true);
  out->eq_soft_right_rad = g.number("eq_soft_limit_right_rad", 0.0, out->eq_mech_right_rad, true);

  const json::Document &doc = g.doc();
  const json::Node &table = g.node("link_table_rad");
  const std::string path = g.key_path("link_table_rad");
  if (table.kind != json::kArray) throw ConfigError(path + ": must be an array of [pot, wheel] rows");
  if (table.items.size() < 2 || table.items.size() > 64) throw ConfigError(path + ": needs 2..64 rows");
  out->link_pot_rad.clear();
  out->link_wheel_rad.clear();
  for (size_t r = 0; r < table.items.size(); ++r) {
    const json::Node &row = doc.node(table.items[r]);
    if (row.kind != json::kArray || row.items.size() != 2) throw ConfigError(path + ": each row must be [pot, wheel]");
    double v[2];
    for (int c = 0; c < 2; ++c) {
      const json::Node &cell = doc.node(row.items[static_cast<size_t>(c)]);
      if (cell.kind != json::kNumber || !is_finite(cell.number) || std::fabs(cell.number) > kPi) {
        throw ConfigError(path + ": row values must be finite numbers within [-pi, pi]");
      }
      v[c] = cell.number;
    }
    if (r > 0 && (!(v[0] > out->link_pot_rad.back()) || !(v[1] > out->link_wheel_rad.back()))) {
      throw ConfigError(path + ": both columns must be strictly increasing");
    }
    out->link_pot_rad.push_back(v[0]);
    out->link_wheel_rad.push_back(v[1]);
  }
}

void check_geometry_consistency(const GeometryConfig &g) {
  // The equivalent mechanical limits must keep both wheels inside their own limits, and one
  // wheel must touch its limit (they are the largest such angles, design 3.2).
  const double inside_tol = 1e-9, touch_tol = 1e-7;
  double left = 0.0, right = 0.0;
  wheel_angles(g.eq_mech_left_rad, g.linkage, g.wheelbase_m, g.kingpin_width_m, g.ackermann_ratio, &left, &right);
  const double over_left = std::fmax(left - g.wheel_limit_rad[kLeftTowardLeft], right - g.wheel_limit_rad[kRightTowardLeft]);
  if (over_left > inside_tol || over_left < -touch_tol) {
    throw ConfigError("geometry.eq_mech_limit_left_rad: inconsistent with wheel_mech_limit_rad (wheel margin " +
                      format_double(-over_left) + " rad)");
  }
  wheel_angles(-g.eq_mech_right_rad, g.linkage, g.wheelbase_m, g.kingpin_width_m, g.ackermann_ratio, &left, &right);
  const double over_right =
      std::fmax(-left - g.wheel_limit_rad[kLeftTowardRight], -right - g.wheel_limit_rad[kRightTowardRight]);
  if (over_right > inside_tol || over_right < -touch_tol) {
    throw ConfigError("geometry.eq_mech_limit_right_rad: inconsistent with wheel_mech_limit_rad (wheel margin " +
                      format_double(-over_right) + " rad)");
  }
}

void parse_actuator(const ObjectView &a, int mode, ActuatorConfig *out) {
  static const char *const keys[] = {"level",
                                     "rate_max_rad_s",
                                     "rate_standstill_rad_s",
                                     "tau_s",
                                     "min_pwm",
                                     "min_pwm_standstill",
                                     "backlash_rad",
                                     "backlash_wander_sd_rad",
                                     "backlash_wander_corr_length_m",
                                     "self_locking",
                                     "release_scale",
                                     "release_centering_per_s_per_m_s",
                                     "release_noise_sd_rad",
                                     "release_noise_corr_length_m",
                                     "lr_asymmetry",
                                     "stall_torque_nm",
                                     "load_moving_nm",
                                     "load_standstill_nm",
                                     "zero_offset_rad"};
  a.require_keys(keys, sizeof keys / sizeof keys[0]);
  const std::string level = a.string("level");
  if (level == "L0") {
    out->level = kLevelL0;
    if (mode != kModeTestServo) throw ConfigError("actuator.level: L0 (ideal servo) works only in mode test_servo");
  } else if (level == "L1") {
    out->level = kLevelL1;
  } else if (level == "L1b") {
    throw ConfigError(
        "actuator.level: L1b (torque model) is not implemented in v1 (TODO: needs the steering motor no-load speed "
        "and a load-torque model, design 3.3 and Q11)");
  } else {
    throw ConfigError("actuator.level: must be L0, L1 or L1b");
  }
  out->rate_max_rad_s = a.number("rate_max_rad_s", 0.0, 100.0, true);
  out->rate_standstill_rad_s = a.number("rate_standstill_rad_s", 0.0, 100.0, true);
  out->tau_s = a.number("tau_s", 0.0, 10.0, true);
  out->min_pwm = a.number("min_pwm", 0.0, 1.0, false, true);
  out->min_pwm_standstill = a.number("min_pwm_standstill", 0.0, 1.0, false, true);
  // design 3.3 L1: a standing car steers slower and needs more PWM to start the motor
  if (out->rate_standstill_rad_s > out->rate_max_rad_s) {
    throw ConfigError("actuator.rate_standstill_rad_s: " + format_double(out->rate_standstill_rad_s) +
                      " must not exceed rate_max_rad_s " + format_double(out->rate_max_rad_s));
  }
  if (out->min_pwm_standstill < out->min_pwm) {
    throw ConfigError("actuator.min_pwm_standstill: " + format_double(out->min_pwm_standstill) +
                      " must not be below min_pwm " + format_double(out->min_pwm));
  }
  out->backlash_rad = a.number("backlash_rad", 0.0, 0.5);
  out->wander_sd_rad = a.number("backlash_wander_sd_rad", 0.0, 0.5);
  out->wander_corr_length_m = a.number("backlash_wander_corr_length_m", 0.0, 1e6, true);
  out->self_locking = a.boolean("self_locking");
  out->release_scale = a.number("release_scale", 0.0, 100.0);
  out->release_centering_per_s_per_m_s = a.number("release_centering_per_s_per_m_s", 0.0, 1000.0);
  out->release_noise_sd_rad = a.number("release_noise_sd_rad", 0.0, 0.5);
  out->release_noise_corr_length_m = a.number("release_noise_corr_length_m", 0.0, 1e6, true);
  out->lr_asymmetry = a.number("lr_asymmetry", -0.9, 0.9);
  out->stall_torque_nm = a.number("stall_torque_nm", 0.0, 1e4);
  out->load_moving_nm = a.number("load_moving_nm", 0.0, 1e4);
  out->load_standstill_nm = a.number("load_standstill_nm", 0.0, 1e4);
  out->zero_offset_rad = a.number("zero_offset_rad", -kHalfPi, kHalfPi, true, true);
}

void parse_pot(const ObjectView &p, PotConfig *out) {
  static const char *const keys[] = {"location",         "range_rad",          "to_steer_ratio",  "center_raw",
                                     "adc_bits",         "ratiometric",        "supply_sag_per_pwm", "noise_sd_counts",
                                     "linearity_frac",   "gain_error",         "drift_rad_per_s", "cal_residual_rad"};
  p.require_keys(keys, sizeof keys / sizeof keys[0]);
  const std::string location = p.string("location");
  if (location == "motor") {
    out->at_motor = true;
  } else if (location == "wheel") {
    out->at_motor = false;
  } else {
    throw ConfigError("pot.location: must be motor or wheel");
  }
  out->range_rad = p.number("range_rad", 0.0, 4.0 * kPi, true);
  out->to_steer_ratio = p.number("to_steer_ratio", 0.0, 100.0, true);
  // pot_raw is 0..1023 on the wire (backend_api.POT_RAW_MAX), so at most 10 bits.
  out->adc_bits = static_cast<int>(p.integer("adc_bits", 1, 10));
  out->center_raw = static_cast<int>(p.integer("center_raw", 0, (1 << out->adc_bits) - 1));
  out->ratiometric = p.boolean("ratiometric");
  out->supply_sag_per_pwm = p.number("supply_sag_per_pwm", 0.0, 1.0, false, true);
  out->noise_sd_counts = p.number("noise_sd_counts", 0.0, 1000.0);
  out->linearity_frac = p.number("linearity_frac", -0.5, 0.5);
  out->gain_error = p.number("gain_error", -1.0, 10.0, true);
  out->drift_rad_per_s = p.number("drift_rad_per_s", -1.0, 1.0);
  out->cal_residual_rad = p.number("cal_residual_rad", -kPi, kPi);
  // ADC counts per rad; bounded so that center + gain * angle stays finite (no NaN reaches the int conversion)
  const double gain = static_cast<double>(1 << out->adc_bits) * out->to_steer_ratio * (1.0 + out->gain_error) /
                      out->range_rad;
  if (!is_finite(gain) || !(gain > 0.0) || gain > kMaxPotGainCountsPerRad) {
    throw ConfigError("pot: gain 2^adc_bits * to_steer_ratio * (1 + gain_error) / range_rad = " + format_double(gain) +
                      " counts/rad must be finite and in (0, " + format_double(kMaxPotGainCountsPerRad) + "]");
  }
}

void parse_servo(const ObjectView &s, int tick_hz, ServoConfig *out) {
  static const char *const keys[] = {"servo_hz", "kp_per_rad", "deadband_rad", "min_pwm", "max_pwm"};
  s.require_keys(keys, sizeof keys / sizeof keys[0]);
  out->servo_hz = static_cast<int>(s.integer("servo_hz", 1, tick_hz));
  if (tick_hz % out->servo_hz != 0) throw ConfigError("mcu_tick_hz: must be a multiple of test_servo.servo_hz");
  out->kp_per_rad = s.number("kp_per_rad", 0.0, 1e6);
  out->deadband_rad = s.number("deadband_rad", 0.0, kPi);
  out->min_pwm = s.number("min_pwm", 0.0, 1.0);
  out->max_pwm = s.number("max_pwm", 0.0, 1.0);
  if (out->min_pwm > out->max_pwm) throw ConfigError("test_servo.min_pwm: must not exceed max_pwm");
}

void parse_injections(const json::Document &doc, int index, int level, std::vector<Injection> *out) {
  const json::Node &list = doc.node(index);
  if (list.kind != json::kArray) throw ConfigError("injections: must be an array");
  if (list.items.size() > 64) throw ConfigError("injections: at most 64 items");
  for (size_t i = 0; i < list.items.size(); ++i) {
    char path[48];
    snprintf(path, sizeof path, "injections[%lu]", static_cast<unsigned long>(i));
    const ObjectView item(doc, list.items[i], path);
    static const char *const keys[] = {"kind", "at_s", "params"};
    item.require_keys(keys, 3);
    Injection inj;
    const std::string kind = item.string("kind");
    inj.kind = -1;
    for (int k = 0; k < kInjKindCount; ++k) {
      if (kind == kInjectionNames[k]) inj.kind = k;
    }
    if (inj.kind < 0) throw ConfigError(item.key_path("kind") + ": unknown injection kind " + kind);
    inj.at_s = item.number("at_s", 0.0, 1e9);
    const ObjectView params = item.object("params");
    switch (inj.kind) {
      case kInjSlow: {
        static const char *const p[] = {"rate_scale"};
        params.require_keys(p, 1);
        inj.value = params.number("rate_scale", 0.0, 1.0, true);
        break;
      }
      case kInjDriverStuckOn: {
        static const char *const p[] = {"pwm"};
        params.require_keys(p, 1);
        inj.value = params.number("pwm", -1.0, 1.0);
        break;
      }
      case kInjPotNoiseX10: {
        static const char *const p[] = {"scale"};
        params.require_keys(p, 1);
        inj.value = params.number("scale", 0.0, 1e6, true);
        break;
      }
      case kInjPotJump: {
        static const char *const p[] = {"counts", "duration_s"};
        params.require_keys(p, 2);
        inj.counts = static_cast<long>(params.integer("counts", -65535, 65535));
        inj.duration_s = params.number("duration_s", 0.0, 1e9, true);
        break;
      }
      case kInjPotSlip: {
        static const char *const p[] = {"offset_rad"};
        params.require_keys(p, 1);
        inj.value = params.number("offset_rad", -2.0 * kPi, 2.0 * kPi);
        break;
      }
      case kInjPotDrift: {
        static const char *const p[] = {"rate_rad_per_s"};
        params.require_keys(p, 1);
        inj.value = params.number("rate_rad_per_s", -100.0, 100.0);
        break;
      }
      case kInjSteerTorque: {
        static const char *const p[] = {"torque_nm", "duration_s"};
        params.require_keys(p, 2);
        inj.value = params.number("torque_nm", -1000.0, 1000.0);
        inj.duration_s = params.number("duration_s", 0.0, 1e9, true);
        break;
      }
      default: {
        params.require_keys(0, 0);
        break;
      }
    }
    if (level == kLevelL0 && inj.kind == kInjSteerTorque) {
      throw ConfigError(item.key_path("kind") + ": steer_torque needs the PWM actuator (L1); L0 is an ideal servo");
    }
    if (level == kLevelL0 &&
        (inj.kind == kInjDriverDead || inj.kind == kInjDriverStuckOn || inj.kind == kInjPolarityReversed)) {
      throw ConfigError(item.key_path("kind") + ": driver fault " + kind +
                        " needs the PWM actuator (L1); L0 is an ideal servo");
    }
    out->push_back(inj);
  }
}

// Optional block "steer_ext" (profile schema v3). Every key is required inside the block.
void parse_ext(const ObjectView &e, const ActuatorConfig &act, ExtConfig *out) {
  static const char *const keys[] = {"belt_play_rad",          "belt_creep_sd_rad_per_sqrt_rad", "belt_skip_per_rad",
                                     "belt_tooth_step_rad",    "supply_rate_scale",              "backdrive_coast_gain",
                                     "self_lock_breakaway_nm", "impact_rate_per_m",              "impact_torque_sd_nm",
                                     "impact_duration_s",      "push_sd_nm",                     "push_corr_length_m"};
  e.require_keys(keys, sizeof keys / sizeof keys[0]);
  out->present = true;
  out->belt_play_rad = e.number("belt_play_rad", 0.0, 0.5);
  out->belt_creep_sd_rad_per_sqrt_rad = e.number("belt_creep_sd_rad_per_sqrt_rad", 0.0, 1.0);
  out->belt_skip_per_rad = e.number("belt_skip_per_rad", 0.0, 1000.0);
  out->belt_tooth_step_rad = e.number("belt_tooth_step_rad", 0.0, kPi);
  out->supply_rate_scale = e.number("supply_rate_scale", 0.0, 2.0, true);
  out->backdrive_coast_gain = e.number("backdrive_coast_gain", 1.0, 100.0);
  out->self_lock_breakaway_nm = e.number("self_lock_breakaway_nm", 0.0, 1e4);
  out->impact_rate_per_m = e.number("impact_rate_per_m", 0.0, 1000.0);
  out->impact_torque_sd_nm = e.number("impact_torque_sd_nm", 0.0, 1e4);
  out->impact_duration_s = e.number("impact_duration_s", 0.0, 10.0);
  out->push_sd_nm = e.number("push_sd_nm", 0.0, 1e4);
  out->push_corr_length_m = e.number("push_corr_length_m", 0.0, 1e6, true);
  if (out->belt_skip_per_rad > 0.0 && !(out->belt_tooth_step_rad > 0.0)) {
    throw ConfigError("steer_ext.belt_tooth_step_rad: must be > 0 when belt_skip_per_rad > 0");
  }
  if (out->impact_rate_per_m > 0.0 && !(out->impact_duration_s > 0.0)) {
    throw ConfigError("steer_ext.impact_duration_s: must be > 0 when impact_rate_per_m > 0");
  }
  const bool torque_model = out->impact_rate_per_m > 0.0 || out->push_sd_nm > 0.0;
  if (torque_model && !(act.stall_torque_nm > 0.0)) {
    throw ConfigError("actuator.stall_torque_nm: must be > 0 with the steer_ext torque model");
  }
}

// Optional block "steer_law" (profile schema v4, firmware mode only). Every key is required inside the block.
void parse_law(const ObjectView &b, LawConfig *out) {
  static const char *const keys[] = {
      "kind",          "cal_right",         "cal_center",          "cal_left",        "command_max_deg",
      "pulse_pwm",     "pulse_long_ms",     "pulse_short_ms",      "pulse_long_above_counts", "settle_ms",
      "rebound_window_ms", "target_tol_counts", "restart_tol_counts", "rebound_max_counts", "progress_min_counts",
      "no_progress_max", "drive_budget_ms", "guard_counts",        "sample_ms",       "median_of",
      "pair_tol_counts", "stale_ms",        "limit_margin_counts", "host_step_deg",   "host_deadband_deg",
      "host_min_interval_ms", "host_poll_ms"};
  b.require_keys(keys, sizeof keys / sizeof keys[0]);
  if (b.string("kind") != "abs_hold_8") throw ConfigError(b.key_path("kind") + ": only abs_hold_8 is modelled");
  out->present = true;
  out->cal_right = static_cast<int>(b.integer("cal_right", 1, 4095));
  out->cal_center = static_cast<int>(b.integer("cal_center", 1, 4095));
  out->cal_left = static_cast<int>(b.integer("cal_left", 1, 4095));
  if (!(out->cal_right < out->cal_center && out->cal_center < out->cal_left)) {
    throw ConfigError("steer_law: need cal_right < cal_center < cal_left");
  }
  out->command_max_deg = b.number("command_max_deg", 1.0, 90.0);
  out->pulse_pwm = static_cast<int>(b.integer("pulse_pwm", 1, 255));
  out->long_ms = static_cast<int>(b.integer("pulse_long_ms", 1, 1000));
  out->short_ms = static_cast<int>(b.integer("pulse_short_ms", 1, 1000));
  out->long_above = static_cast<int>(b.integer("pulse_long_above_counts", 0, 4095));
  out->settle_ms = static_cast<int>(b.integer("settle_ms", 1, 10000));
  out->rebound_window_ms = static_cast<int>(b.integer("rebound_window_ms", 0, 10000));
  out->target_tol = static_cast<int>(b.integer("target_tol_counts", 0, 4095));
  out->restart_tol = static_cast<int>(b.integer("restart_tol_counts", 0, 4095));
  out->rebound_max = static_cast<int>(b.integer("rebound_max_counts", 1, 4095));
  out->progress_min = static_cast<int>(b.integer("progress_min_counts", 0, 4095));
  out->no_progress_max = static_cast<int>(b.integer("no_progress_max", 1, 1000));
  out->budget_ms = static_cast<int>(b.integer("drive_budget_ms", 21, 100000));
  out->guard = static_cast<int>(b.integer("guard_counts", 0, 4095));
  out->sample_ms = static_cast<int>(b.integer("sample_ms", 1, 1000));
  out->median_of = static_cast<int>(b.integer("median_of", 1, 9));
  if (out->median_of % 2 != 1) throw ConfigError(b.key_path("median_of") + ": must be odd");
  out->pair_tol = static_cast<int>(b.integer("pair_tol_counts", 0, 4095));
  out->stale_ms = static_cast<int>(b.integer("stale_ms", 1, 10000));
  out->limit_margin = static_cast<int>(b.integer("limit_margin_counts", 0, 4095));
  out->host_step_deg = b.number("host_step_deg", 0.0, 90.0, true);
  out->host_deadband_deg = b.number("host_deadband_deg", 0.0, 90.0);
  out->host_min_interval_ms = static_cast<int>(b.integer("host_min_interval_ms", 1, 100000));
  out->host_poll_ms = static_cast<int>(b.integer("host_poll_ms", 1, 100000));
  if (out->target_tol > out->restart_tol) throw ConfigError("steer_law: need target_tol_counts <= restart_tol_counts");
  if (out->short_ms > out->long_ms) throw ConfigError("steer_law: need pulse_short_ms <= pulse_long_ms");
}

// Optional block "steer_law" with kind smooth_r5 (profile schema v5, firmware mode only; smooth_r5.h). Every key is
// required. The SM parameter ranges are the firmware's SM_LIMITS (SM SET refuses values outside them). Kind
// smooth_r6p (profile schema v6, the SIM proposal r6p) needs the four proposal keys too.
void parse_smooth_law(const ObjectView &b, SmoothLawConfig *out, bool proposal) {
  static const char *const keys[] = {
      "kind",              "cal_right",          "cal_center",          "cal_left",          "command_max_deg",
      "pwm_min",           "pwm_max",            "slow_counts",         "stop_tol_counts",   "restart_tol_counts",
      "ramp_per_tick",     "settle_ms",          "stall_ms",            "boost",             "max_run_ms",
      "brake_ms",          "quiet_ms",           "quiet_us",            "progress_counts",   "max_runs",
      "reverse_pause_ms",  "ramp_tick_ms",       "rebound_max_counts",  "rebound_window_ms", "guard_counts",
      "sample_ms",         "median_of",          "pair_tol_counts",     "stale_ms",          "slew_min_counts",
      "slew_counts_per_ms", "hard_right_min",    "hard_left_max",       "limit_margin_counts",
      "adc_powered_offset_counts", "adc_burst_counts", "adc_burst_ms",  "adc_burst_period_ms",
      "host_step_deg",     "host_deadband_deg",  "host_min_interval_ms", "host_poll_ms",
      // kind smooth_r6p only (the SIM proposal r6p)
      "hold_pwm_max",      "push_counts",        "rest_brake",          "hold_no_cancel"};
  const size_t count = sizeof keys / sizeof keys[0];
  b.require_keys(keys, proposal ? count : count - 4);
  out->present = true;
  out->cal_right = static_cast<int>(b.integer("cal_right", 1, 4095));
  out->cal_center = static_cast<int>(b.integer("cal_center", 1, 4095));
  out->cal_left = static_cast<int>(b.integer("cal_left", 1, 4095));
  if (!(out->cal_right < out->cal_center && out->cal_center < out->cal_left)) {
    throw ConfigError("steer_law: need cal_right < cal_center < cal_left");
  }
  out->command_max_deg = b.number("command_max_deg", 1.0, 90.0);
  out->pwm_min = static_cast<int>(b.integer("pwm_min", 20, 150));
  out->pwm_max = static_cast<int>(b.integer("pwm_max", 40, 160));
  out->slow = static_cast<int>(b.integer("slow_counts", 20, 400));
  out->stop_tol = static_cast<int>(b.integer("stop_tol_counts", 3, 30));
  out->restart_tol = static_cast<int>(b.integer("restart_tol_counts", 5, 60));
  out->ramp = static_cast<int>(b.integer("ramp_per_tick", 1, 100));
  out->settle_ms = static_cast<int>(b.integer("settle_ms", 40, 1000));
  out->stall_ms = static_cast<int>(b.integer("stall_ms", 80, 1000));
  out->boost = static_cast<int>(b.integer("boost", 1, 60));
  out->max_run_ms = static_cast<int>(b.integer("max_run_ms", 300, 8000));
  out->brake_ms = static_cast<int>(b.integer("brake_ms", 0, 200));
  out->quiet_ms = static_cast<int>(b.integer("quiet_ms", 0, 50));
  out->quiet_us = static_cast<int>(b.integer("quiet_us", 200, 3000));
  if (out->pwm_min > out->pwm_max) throw ConfigError("steer_law: need pwm_min <= pwm_max (PWM_MIN ABOVE PWM_MAX)");
  if (out->restart_tol <= out->stop_tol) {
    throw ConfigError("steer_law: need restart_tol_counts > stop_tol_counts (RESTART_TOL NOT ABOVE STOP_TOL)");
  }
  out->progress_counts = static_cast<int>(b.integer("progress_counts", 1, 4095));
  out->max_runs = static_cast<int>(b.integer("max_runs", 1, 255));
  out->reverse_pause_ms = static_cast<int>(b.integer("reverse_pause_ms", 0, 10000));
  out->ramp_tick_ms = static_cast<int>(b.integer("ramp_tick_ms", 1, 1000));
  out->rebound_max = static_cast<int>(b.integer("rebound_max_counts", 1, 4095));
  out->rebound_window_ms = static_cast<int>(b.integer("rebound_window_ms", 0, 10000));
  out->guard = static_cast<int>(b.integer("guard_counts", 0, 4095));
  out->sample_ms = static_cast<int>(b.integer("sample_ms", 1, 1000));
  out->median_of = static_cast<int>(b.integer("median_of", 1, 9));
  if (out->median_of % 2 != 1) throw ConfigError(b.key_path("median_of") + ": must be odd");
  out->pair_tol = static_cast<int>(b.integer("pair_tol_counts", 0, 4095));
  out->stale_ms = static_cast<int>(b.integer("stale_ms", 1, 10000));
  out->slew_min = static_cast<int>(b.integer("slew_min_counts", 0, 4095));
  out->slew_per_ms = static_cast<int>(b.integer("slew_counts_per_ms", 0, 4095));
  out->hard_right_min = static_cast<int>(b.integer("hard_right_min", 1, 4095));
  out->hard_left_max = static_cast<int>(b.integer("hard_left_max", 1, 4095));
  out->limit_margin = static_cast<int>(b.integer("limit_margin_counts", 0, 4095));
  if (!(out->hard_right_min <= out->cal_right && out->cal_left <= out->hard_left_max)) {
    throw ConfigError("steer_law: need hard_right_min <= cal_right and cal_left <= hard_left_max");
  }
  out->adc_powered_offset = static_cast<int>(b.integer("adc_powered_offset_counts", -4095, 4095));
  out->adc_burst_counts = static_cast<int>(b.integer("adc_burst_counts", -4095, 4095));
  out->adc_burst_ms = static_cast<int>(b.integer("adc_burst_ms", 0, 100000));
  out->adc_burst_period_ms = static_cast<int>(b.integer("adc_burst_period_ms", 0, 100000));
  if (out->adc_burst_counts != 0 && !(out->adc_burst_period_ms > 0 && out->adc_burst_ms > 0 &&
                                      out->adc_burst_ms <= out->adc_burst_period_ms)) {
    throw ConfigError("steer_law: adc_burst_counts needs 0 < adc_burst_ms <= adc_burst_period_ms");
  }
  out->host_step_deg = b.number("host_step_deg", 0.0, 90.0, true);
  out->host_deadband_deg = b.number("host_deadband_deg", 0.0, 90.0);
  out->host_min_interval_ms = static_cast<int>(b.integer("host_min_interval_ms", 1, 100000));
  out->host_poll_ms = static_cast<int>(b.integer("host_poll_ms", 1, 100000));
  out->proposal = proposal;
  if (proposal) {
    out->hold_pwm_max = static_cast<int>(b.integer("hold_pwm_max", 40, 255));
    if (out->hold_pwm_max < out->pwm_max) throw ConfigError("steer_law: need hold_pwm_max >= pwm_max");
    out->push_counts = static_cast<int>(b.integer("push_counts", 0, 4095));
    out->rest_brake = static_cast<int>(b.integer("rest_brake", 0, 1));
    out->hold_no_cancel = static_cast<int>(b.integer("hold_no_cancel", 0, 1));
  }
}

const char *const kProtocolV1 = "ubts.steer_lockstep.v1";
const char *const kProtocolV2 = "ubts.steer_lockstep.v2";

int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Hex text -> bytes; false on an odd length or a non-hex character.
bool hex_bytes(const std::string &text, std::vector<uint8_t> *out) {
  if (text.size() % 2 != 0) return false;
  out->clear();
  for (size_t i = 0; i < text.size(); i += 2) {
    const int hi = hex_value(text[i]), lo = hex_value(text[i + 1]);
    if (hi < 0 || lo < 0) return false;
    out->push_back(static_cast<uint8_t>((hi << 4) | lo));
  }
  return true;
}

void parse_firmware(const ObjectView &f, const GeometryConfig &g, FirmwareConfig *out) {
  static const char *const keys[] = {"expect_build", "boot_seed",       "chip_id",          "eeprom_hex",
                                     "initial_delta_eq_rad", "boot_delay_ticks", "millis_start", "boot_garbage_hex",
                                     "adc_mode",     "adc_script_raw",  "motor_power",      "report_adc",
                                     "report_servo"};
  f.require_keys(keys, sizeof keys / sizeof keys[0]);
  const json::Node &build = f.node("expect_build");
  if (build.kind == json::kNull) {
    out->expect_build_set = false;
  } else {
    bool ok = build.kind == json::kString && build.text.size() == 8;
    for (size_t i = 0; ok && i < build.text.size(); ++i) {
      const char c = build.text[i];
      ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F');
    }
    if (!ok) throw ConfigError(f.key_path("expect_build") + ": must be null or 8 uppercase hex digits");
    out->expect_build_set = true;
    out->expect_build = build.text;
  }
  if (!json::to_uint64(f.node("boot_seed"), &out->boot_seed)) {
    throw ConfigError(f.key_path("boot_seed") + ": must be an integer 0..2^64-1");
  }
  out->chip_id = static_cast<uint32_t>(f.integer("chip_id", 0, 16777215));
  const json::Node &eeprom = f.node("eeprom_hex");
  if (eeprom.kind == json::kNull) {
    out->eeprom_set = false;
    for (int i = 0; i < 256; ++i) out->eeprom[i] = 0xFF;
  } else {
    std::vector<uint8_t> bytes;
    if (eeprom.kind != json::kString || eeprom.text.size() != 512 || !hex_bytes(eeprom.text, &bytes)) {
      throw ConfigError(f.key_path("eeprom_hex") + ": must be null or 512 hex digits (256 bytes)");
    }
    out->eeprom_set = true;
    for (int i = 0; i < 256; ++i) out->eeprom[i] = bytes[static_cast<size_t>(i)];
  }
  const double delta = f.number("initial_delta_eq_rad", -kHalfPi, kHalfPi, true, true);
  if (delta > g.eq_mech_left_rad || delta < -g.eq_mech_right_rad) {
    throw ConfigError(f.key_path("initial_delta_eq_rad") + ": outside the equivalent mechanical limits");
  }
  out->initial_delta_eq_rad = delta;
  out->boot_delay_ticks = static_cast<uint32_t>(f.integer("boot_delay_ticks", 0, 1000));
  out->millis_start = static_cast<uint32_t>(f.integer("millis_start", 0, 4294967295LL));
  const std::string garbage = f.string("boot_garbage_hex");
  if (garbage.size() > 512 || !hex_bytes(garbage, &out->boot_garbage)) {
    throw ConfigError(f.key_path("boot_garbage_hex") + ": must be an even number of hex digits, at most 512");
  }
  const std::string adc = f.string("adc_mode");
  if (adc == "model") {
    out->adc_model = true;
  } else if (adc == "script") {
    out->adc_model = false;
  } else {
    throw ConfigError(f.key_path("adc_mode") + ": must be model or script");
  }
  out->adc_script_raw = static_cast<uint16_t>(f.integer("adc_script_raw", 0, 1023));
  out->motor_power = f.boolean("motor_power");
  out->report_adc = f.boolean("report_adc");
  out->report_servo = f.boolean("report_servo");
}

}  // namespace

const char *injection_kind_name(int kind) {
  return kind >= 0 && kind < kInjKindCount ? kInjectionNames[kind] : "unknown";
}

Config parse_config(const std::string &line) {
  try {
    json::Document parsed(line);
    const ObjectView top(parsed, parsed.root(), "");
    // The protocol string selects the key set: v2 adds "firmware" (section 11.2).
    bool v2 = false;
    {
      const json::Node &root = parsed.node(parsed.root());
      for (size_t i = 0; i < root.keys.size(); ++i) {
        if (root.keys[i] == "protocol") {
          const json::Node &value = parsed.node(root.items[i]);
          v2 = value.kind == json::kString && value.text == kProtocolV2;
        }
      }
    }
    static const char *const keys_v1[] = {"protocol",      "mode",       "seed",     "mcu_tick_hz", "profile_id",
                                          "profile_sha256", "profile_overrides", "geometry", "actuator", "pot",
                                          "test_servo",    "mcu",        "injections"};
    static const char *const keys_v2[] = {"protocol",      "mode",       "seed",     "mcu_tick_hz", "profile_id",
                                          "profile_sha256", "profile_overrides", "geometry", "actuator", "pot",
                                          "test_servo",    "mcu",        "injections", "firmware"};
    // "steer_ext" (profile schema v3) is optional in both protocols: without it every extension term is off.
    bool has_ext = false, has_law = false;
    {
      const json::Node &root = parsed.node(parsed.root());
      for (size_t i = 0; i < root.keys.size(); ++i) {
        has_ext = has_ext || root.keys[i] == "steer_ext";
        has_law = has_law || root.keys[i] == "steer_law";
      }
    }
    std::vector<const char *> key_set;
    if (v2) {
      key_set.assign(keys_v2, keys_v2 + sizeof keys_v2 / sizeof keys_v2[0]);
    } else {
      key_set.assign(keys_v1, keys_v1 + sizeof keys_v1 / sizeof keys_v1[0]);
    }
    if (has_ext) key_set.push_back("steer_ext");
    // "steer_law" (profile schema v4) is optional and needs protocol v2 in firmware mode (checked below).
    if (has_law) key_set.push_back("steer_law");
    top.require_keys(&key_set[0], key_set.size());
    Config cfg;
    const std::string protocol = top.string("protocol");
    if (protocol == kProtocolV1) {
      cfg.protocol = 1;
    } else if (protocol == kProtocolV2) {
      cfg.protocol = 2;
    } else {
      throw ConfigError("protocol: expected ubts.steer_lockstep.v1 or ubts.steer_lockstep.v2, got " + protocol);
    }
    const std::string mode = top.string("mode");
    if (mode == "direct_pwm") {
      cfg.mode = kModeDirectPwm;
    } else if (mode == "test_servo") {
      cfg.mode = kModeTestServo;
    } else if (mode == "firmware") {
      if (cfg.protocol != 2) {
        throw ConfigError("mode: firmware needs protocol ubts.steer_lockstep.v2; v1 runs direct_pwm and test_servo");
      }
      cfg.mode = kModeFirmware;
    } else {
      throw ConfigError("mode: unknown mode " + mode);
    }
    if (!json::to_uint64(top.node("seed"), &cfg.seed)) throw ConfigError("seed: must be an integer 0..2^64-1");
    cfg.tick_hz = static_cast<int>(top.integer("mcu_tick_hz", 100, 100000));
    cfg.profile_id = top.string("profile_id");
    cfg.profile_sha256 = top.string("profile_sha256");
    const json::Node &overrides = top.node("profile_overrides");
    if (overrides.kind != json::kNull && overrides.kind != json::kObject) {
      throw ConfigError("profile_overrides: must be null or an object");
    }
    if (top.node("mcu").kind != json::kObject) throw ConfigError("mcu: must be an object");

    parse_geometry(top.object("geometry"), &cfg.geometry);
    check_geometry_consistency(cfg.geometry);

    parse_actuator(top.object("actuator"), cfg.mode, &cfg.actuator);
    const double offset = cfg.actuator.zero_offset_rad;
    if (!(offset < cfg.geometry.eq_mech_left_rad) || !(offset > -cfg.geometry.eq_mech_right_rad)) {
      throw ConfigError("actuator.zero_offset_rad: must lie strictly inside the equivalent mechanical limits");
    }
    parse_pot(top.object("pot"), &cfg.pot);
    parse_servo(top.object("test_servo"), cfg.tick_hz, &cfg.servo);
    parse_injections(parsed, top.child("injections"), cfg.actuator.level, &cfg.injections);
    if (has_ext) parse_ext(top.object("steer_ext"), cfg.actuator, &cfg.ext);
    for (size_t i = 0; i < cfg.injections.size(); ++i) {
      if (cfg.injections[i].kind == kInjSteerTorque && !(cfg.actuator.stall_torque_nm > 0.0)) {
        throw ConfigError("actuator.stall_torque_nm: must be > 0 with a steer_torque injection");
      }
    }
    if (cfg.protocol == 2) {
      const json::Node &firmware = top.node("firmware");
      if (cfg.mode == kModeFirmware) {
        if (firmware.kind != json::kObject) throw ConfigError("firmware: must be an object in mode firmware");
        // MCU tick = 1 ms = millis() + 1 (section 11.2); the firmware reads only EEPROM, compile constants and
        // the serial bytes, so of the mcu block only the baud is used here.
        if (cfg.tick_hz != 1000) throw ConfigError("mcu_tick_hz: mode firmware needs 1000");
        const ObjectView mcu = top.object("mcu");
        const json::Node &baud = mcu.node("baud");
        int64_t value = 0;
        if (!json::to_int64(baud, &value) || value < 1 || value > 5760000) {
          throw ConfigError("mcu.baud: must be an integer baud rate");
        }
        cfg.mcu_baud = static_cast<uint32_t>(value);
        parse_firmware(top.object("firmware"), cfg.geometry, &cfg.fw);
        if (has_law) {
          const ObjectView law = top.object("steer_law");
          if (law.has_string("kind", "smooth_r5")) {
            parse_smooth_law(law, &cfg.smooth, false);  // profile schema v5, steer_fw_mode smooth_r5
          } else if (law.has_string("kind", "smooth_r6p")) {
            parse_smooth_law(law, &cfg.smooth, true);  // profile schema v6, steer_fw_mode smooth_r6p (SIM proposal)
          } else {
            parse_law(law, &cfg.law);  // every other block: the abs_hold_8 rules and messages as before
          }
        }
      } else if (firmware.kind != json::kNull) {
        throw ConfigError("firmware: must be null unless the mode is firmware");
      }
    }
    if (has_law && !cfg.law.present && !cfg.smooth.present) {
      throw ConfigError("steer_law: only in mode firmware (protocol v2)");
    }
    return cfg;
  } catch (const json::ParseError &e) {
    throw ConfigError(std::string("startup line is not valid JSON: ") + e.what());
  }
}

}  // namespace ubts_steer
