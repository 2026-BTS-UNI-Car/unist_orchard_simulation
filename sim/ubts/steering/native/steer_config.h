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
  kInjSteerTorque,  // external torque on the steering axis (steer_ext disturbance model, ST-11)
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
  double value;       // rate_scale, pwm, scale, offset_rad, rate_rad_per_s or torque_nm (by kind)
  double duration_s;  // pot_jump and steer_torque only
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

// Optional startup block "steer_ext" (vehicle profile schema v3: belt potentiometer, steering supply, external torque;
// docs/steering/STEER_EXT_MODEL_KO.md). Absent: every term is off and the model is bit-identical to a run without it.
struct ExtConfig {
  ExtConfig()
      : present(false),
        belt_play_rad(0.0),
        belt_creep_sd_rad_per_sqrt_rad(0.0),
        belt_skip_per_rad(0.0),
        belt_tooth_step_rad(0.0),
        supply_rate_scale(1.0),
        backdrive_coast_gain(1.0),
        self_lock_breakaway_nm(0.0),
        impact_rate_per_m(0.0),
        impact_torque_sd_nm(0.0),
        impact_duration_s(0.0),
        push_sd_nm(0.0),
        push_corr_length_m(1.0) {}
  bool present;
  double belt_play_rad;                   // measurement-side play between the measured member and the pot shaft
  double belt_creep_sd_rad_per_sqrt_rad;  // friction-belt creep: random walk of the pot zero per sqrt(travel)
  double belt_skip_per_rad;               // tooth skips per rad of pot-side travel (Poisson)
  double belt_tooth_step_rad;             // zero shift of one tooth skip (steering scale)
  double supply_rate_scale;               // steering rate x this (shared battery voltage scale)
  double backdrive_coast_gain;            // back-driven speed x this with the driver off (no electrical braking)
  double self_lock_breakaway_nm;          // self-locking gear: external torque needed to back-drive it
  double impact_rate_per_m, impact_torque_sd_nm, impact_duration_s;  // ground impacts (Poisson in distance)
  double push_sd_nm, push_corr_length_m;                             // pushing torque (OU in distance)
};

// Optional startup block "steer_law" with kind abs_hold_8 (vehicle profile schema v4, steer_fw_mode abs_hold_8;
// firmware mode only):
// the steering law of the real board firmware ABS-HOLD-8-REBOUND-LIMITS replaces the V3 servo law (abs_hold8.h).
// Absent: the V3 law runs and the session is bit-identical to one without the block. Counts of a 12-bit ADC, ms.
struct LawConfig {
  LawConfig()
      : present(false),
        cal_right(1681),
        cal_center(1900),
        cal_left(2180),
        command_max_deg(20.0),
        pulse_pwm(100),
        long_ms(80),
        short_ms(50),
        long_above(30),
        settle_ms(300),
        rebound_window_ms(100),
        target_tol(10),
        restart_tol(16),
        rebound_max(20),
        progress_min(3),
        no_progress_max(3),
        budget_ms(1000),
        guard(15),
        sample_ms(5),
        median_of(5),
        pair_tol(20),
        stale_ms(100),
        limit_margin(100),
        host_step_deg(0.5),
        host_deadband_deg(1.0),
        host_min_interval_ms(1000),
        host_poll_ms(200) {}
  bool present;
  int cal_right, cal_center, cal_left;
  double command_max_deg;
  int pulse_pwm, long_ms, short_ms, long_above, settle_ms, rebound_window_ms;
  int target_tol, restart_tol, rebound_max, progress_min, no_progress_max, budget_ms, guard;
  int sample_ms, median_of, pair_tol, stale_ms, limit_margin;
  double host_step_deg, host_deadband_deg;
  int host_min_interval_ms, host_poll_ms;
};

// Optional startup block "steer_law" with kind smooth_r5 (vehicle profile schema v5, steer_fw_mode smooth_r5;
// firmware mode only): the SMOOTH hold of the board firmware r5 replaces the V3 servo law (smooth_r5.h). Counts of a
// 12-bit ADC, ms. The defaults are the firmware's compile-time defaults (SM_DEFAULTS) and constants.
struct SmoothLawConfig {
  SmoothLawConfig()
      : present(false),
        cal_right(1681),
        cal_center(1900),
        cal_left(2180),
        command_max_deg(20.0),
        pwm_min(60),
        pwm_max(100),
        slow(120),
        stop_tol(6),
        restart_tol(16),
        ramp(12),
        settle_ms(120),
        stall_ms(200),
        boost(15),
        max_run_ms(5000),
        brake_ms(40),
        quiet_ms(15),
        quiet_us(1500),
        progress_counts(4),
        max_runs(4),
        reverse_pause_ms(60),
        ramp_tick_ms(10),
        rebound_max(20),
        rebound_window_ms(100),
        guard(15),
        sample_ms(5),
        median_of(5),
        pair_tol(20),
        stale_ms(100),
        slew_min(10),
        slew_per_ms(2),
        hard_right_min(1300),
        hard_left_max(2600),
        limit_margin(100),
        adc_powered_offset(0),
        adc_burst_counts(0),
        adc_burst_ms(0),
        adc_burst_period_ms(0),
        host_step_deg(0.25),
        host_deadband_deg(0.25),
        host_min_interval_ms(200),
        host_poll_ms(200) {}
  bool present;
  int cal_right, cal_center, cal_left;
  double command_max_deg;
  // SM parameters (NVS steer_sm)
  int pwm_min, pwm_max, slow, stop_tol, restart_tol, ramp, settle_ms, stall_ms, boost, max_run_ms, brake_ms, quiet_ms,
      quiet_us;
  // compile-time constants of the smooth hold
  int progress_counts, max_runs, reverse_pause_ms, ramp_tick_ms, rebound_max, rebound_window_ms, guard;
  // potentiometer filter (readPotValue, r4 slew limit) and validity window
  int sample_ms, median_of, pair_tol, stale_ms, slew_min, slew_per_ms, hard_right_min, hard_left_max, limit_margin;
  // SIM ADC path placeholders while the steering driver is powered (0 = off)
  int adc_powered_offset, adc_burst_counts, adc_burst_ms, adc_burst_period_ms;
  // SIM host ANGLE policy
  double host_step_deg, host_deadband_deg;
  int host_min_interval_ms, host_poll_ms;
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
  ExtConfig ext;         // optional steer_ext block
  uint32_t mcu_baud;     // v2 firmware mode: mcu.baud (checked against the compiled firmware baud)
  FirmwareConfig fw;     // v2 firmware mode
  LawConfig law;         // optional steer_law block (firmware mode), kind abs_hold_8
  SmoothLawConfig smooth;  // optional steer_law block (firmware mode), kind smooth_r5
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
