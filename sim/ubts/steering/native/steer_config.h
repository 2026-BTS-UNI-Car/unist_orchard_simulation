// Startup configuration of the native steering process (protocol section 2).
//
// parse_config() reads the one-line JSON object written by
// backend_api.native_startup_config()/startup_line(), checks the exact key sets, types and
// ranges, and returns the values in SI units. Any problem is a ConfigError (exit code 2).
// The "mcu" block and the content of "profile_overrides" are type-checked only (v1).
#ifndef UBTS_STEER_CONFIG_H
#define UBTS_STEER_CONFIG_H

#include <stdint.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace ubts_steer {

enum Mode { kModeDirectPwm = 0, kModeTestServo = 1, kModeFirmware = 2 };
enum Level { kLevelL0 = 0, kLevelL1 = 1, kLevelL1b = 2 };
enum InjectionKind {
  kInjJam,
  kInjSlow,
  kInjDriverDead,
  kInjDriverStuckOn,
  kInjPolarityReversed,
  kInjPotOpen,
  kInjPotShort,
  kInjPotStuck,
  kInjPotNoiseX10,
  kInjPotJump,
  kInjPotSlip,
  kInjPotDrift,
  kInjPotReversed,
  kInjKindCount
};

// Index order of GeometryConfig::wheel_limit_rad (geometry.WHEEL_LIMIT_KEYS).
enum WheelLimit { kLeftTowardLeft = 0, kLeftTowardRight = 1, kRightTowardLeft = 2, kRightTowardRight = 3 };

struct GeometryConfig {
  bool linkage;  // steer_type linkage (true) or axle_pivot (false)
  double wheelbase_m, kingpin_width_m, ackermann_ratio;
  double wheel_limit_rad[4];  // positive magnitudes, WheelLimit order
  double eq_mech_left_rad, eq_mech_right_rad, eq_soft_left_rad, eq_soft_right_rad;
  std::vector<double> link_pot_rad, link_wheel_rad;  // link table columns
};

struct ActuatorConfig {
  int level;  // Level
  double rate_max_rad_s, rate_standstill_rad_s, tau_s, min_pwm, min_pwm_standstill, backlash_rad;
  double wander_sd_rad, wander_corr_length_m;
  bool self_locking;
  double release_scale, release_centering_per_s_per_m_s, release_noise_sd_rad, release_noise_corr_length_m;
  double lr_asymmetry, stall_torque_nm, load_moving_nm, load_standstill_nm, zero_offset_rad;
};

struct PotConfig {
  bool at_motor;  // location motor (true, before the backlash) or wheel (false, link side)
  double range_rad, to_steer_ratio;
  int center_raw, adc_bits;
  bool ratiometric;
  double supply_sag_per_pwm, noise_sd_counts, linearity_frac, gain_error, drift_rad_per_s, cal_residual_rad;
};

struct ServoConfig {
  int servo_hz;
  double kp_per_rad, deadband_rad, min_pwm, max_pwm;
};

struct Injection {
  Injection() : kind(0), at_s(0.0), value(0.0), duration_s(0.0), counts(0) {}
  int kind;           // InjectionKind
  double at_s;        // activation time after RESET (s)
  double value;       // rate_scale, pwm, scale, offset_rad or rate_rad_per_s (by kind)
  double duration_s;  // pot_jump only
  long counts;        // pot_jump only
};

// Startup "firmware" object of protocol v2 (section 11.2), mode firmware only.
struct FirmwareConfig {
  FirmwareConfig()
      : expect_build_set(false),
        boot_seed(0),
        chip_id(0),
        eeprom_set(false),
        initial_delta_eq_rad(0.0),
        boot_delay_ticks(0),
        millis_start(0),
        adc_model(false),
        adc_script_raw(0),
        motor_power(true),
        report_adc(false),
        report_servo(false) {
    for (int i = 0; i < 256; ++i) eeprom[i] = 0xFF;
  }
  bool expect_build_set;
  std::string expect_build;  // 8 uppercase hex digits when set
  uint64_t boot_seed;
  uint32_t chip_id;
  bool eeprom_set;
  uint8_t eeprom[256];       // all 0xFF when eeprom_hex is null
  double initial_delta_eq_rad;
  uint32_t boot_delay_ticks;
  uint32_t millis_start;
  std::vector<uint8_t> boot_garbage;
  bool adc_model;            // adc_mode "model" (true) or "script"
  uint16_t adc_script_raw;
  bool motor_power;
  bool report_adc;
  bool report_servo;         // servo state channels 11..14 (SIM evaluation only, LOCKSTEP 11.6)
};

struct Config {
  Config() : protocol(1), mode(0), seed(0), tick_hz(0), mcu_baud(0) {}
  int protocol;  // 1 or 2 (startup protocol string; frame header version)
  int mode;      // Mode
  uint64_t seed;
  int tick_hz;
  std::string profile_id, profile_sha256;
  GeometryConfig geometry;
  ActuatorConfig actuator;
  PotConfig pot;
  ServoConfig servo;
  std::vector<Injection> injections;
  uint32_t mcu_baud;     // v2 firmware mode: mcu.baud (checked against the compiled firmware baud)
  FirmwareConfig fw;     // v2 firmware mode
};

class ConfigError : public std::runtime_error {
 public:
  explicit ConfigError(const std::string &what) : std::runtime_error(what) {}
};

// Protocol v1 ("ubts.steer_lockstep.v1": exact v1 key set, firmware mode refused) or v2
// ("ubts.steer_lockstep.v2": v1 keys + "firmware"; section 11.2).
Config parse_config(const std::string &line);
const char *injection_kind_name(int kind);

}  // namespace ubts_steer

#endif  // UBTS_STEER_CONFIG_H
