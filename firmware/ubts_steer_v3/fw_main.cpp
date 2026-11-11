// Entry points and the loop() tick order (see fw_main.h). One processed tick per millisecond: on the real chip
// loop() runs more often and returns early inside the same millisecond (only the TX pump runs); in the SIM
// loop() is called exactly once per 1 ms tick.
#include "fw_main.h"

#include "fw_cal.h"
#include "fw_config.h"
#include "fw_drive.h"
#include "fw_hw.h"
#include "fw_link.h"
#include "fw_monitor.h"
#include "fw_params.h"
#include "fw_state.h"
#include "fw_steer.h"

namespace ubts_fw {

namespace {

uint32_t status_period() { return 1000u / fw.w_par[kPSthz]; }

void load_calibration() {
  uint8_t rec[kRecordBytes];
  for (uint8_t i = 0; i < kRecordBytes; ++i) rec[i] = hw::eeprom_read(i);
  uint16_t id = 0;
  uint32_t crc = 0;
  Cal cal;
  uint16_t par[kParamCount];
  if (record_decode(rec, &id, &cal, par, &crc)) {
    fw.r_valid = 1;
    fw.r_cal_id = id;
    fw.r_crc = crc;
    fw.r_cal = fw.w_cal = cal;
    for (int i = 0; i < kParamCount; ++i) fw.r_par[i] = fw.w_par[i] = par[i];
  } else {
    fw.r_valid = 0;
    cal_clear(&fw.r_cal);
    cal_clear(&fw.w_cal);
    params_defaults(fw.r_par);
    params_defaults(fw.w_par);
  }
}

// Step 3: link watchdog, before the receive step so that a late PING cannot save it (spec 6.2).
void watchdog_step() {
  if (fw.state != kStArmed && fw.state != kStBench) return;
  if (static_cast<uint32_t>(fw.now - fw.last_input) <= fw.w_par[kPWd]) return;
  drive_brake(fw.now);
  if (fw.state == kStArmed) {
    switch (fw.w_par[kPWdmode]) {
      case 1: steer_hold(kHoldCenter, fw.w_par[kPHloss]); break;
      case 2: steer_off(); break;
      default: steer_hold(kHoldMeasured, fw.w_par[kPHloss]); break;
    }
    clear_c_state();
  }
  fw.state = kStDisarmed;
  fw.latched = static_cast<uint16_t>(fw.latched | kFWatchdog);
  emit_watchdog_line();
}

// Step 5: C TTL, after the receive step so that a C received in this tick continues the motion (spec 6.3).
void ttl_step() {
  if (fw.state != kStArmed || !fw.c_valid) return;
  if (static_cast<uint32_t>(fw.now - fw.c_ms) < fw.c_ttl) return;
  if (fw.w_par[kPTtlmode] == 1) {
    drive_release();
  } else {
    drive_brake(fw.now);
  }
  fw.c_valid = 0;
  fw.ttl_bit = 1;
  fw.ttl_count = static_cast<uint16_t>(fw.ttl_count + 1u);
  emit_fault_line(kFTtl, fw.ttl_count);
}

// Step 6: fault detectors and the actions of newly latched faults (spec 6.4, 6.7).
void fault_step() {
  const uint16_t fresh = monitor_faults();
  if (fresh == 0) {
    // A THERMAL latch acts once, at its first trip. While its condition is true again (a CAL JOG or CAL POL that
    // started after the count fell below tha), the steering output stops in every tick; the F line is not sent
    // again (spec 6.7, decision 12.1-68).
    if ((fw.latched & kFThermal) && monitor_thermal_active() && fw.mode != kModeOff) steer_off();
    return;
  }
  if ((fresh & static_cast<uint16_t>(~kFThermal)) != 0) {
    drive_brake(fw.now);
  } else {
    drive_release();  // THERMAL alone
  }
  steer_off();
  if (fw.state == kStArmed) clear_c_state();
  if (fw.state == kStArmed || fw.state == kStBench) fw.state = kStDisarmed;
  fw.latched = static_cast<uint16_t>(fw.latched | fresh);
  for (int bit = 0; bit < 16; ++bit) {
    const uint16_t mask = static_cast<uint16_t>(1u << bit);
    if (fresh & mask) emit_fault_line(mask, monitor_fault_arg(mask));
  }
}

// Step 10: status lines at b + k * P_st (spec 5.4).
void status_step() {
  if (static_cast<int32_t>(fw.now - fw.next_status) < 0) return;
  emit_status_line();
  fw.next_status += status_period();
  if (static_cast<int32_t>(fw.now - fw.next_status) >= 0) fw.next_status = fw.now + status_period();
}

void write_pwm_if(uint8_t gpio, uint8_t value, uint8_t *written, bool zero_pass) {
  if (value == *written || (value == 0) != zero_pass) return;
  hw::write_pwm(gpio, value);
  *written = value;
}

// Step 11: write only the pins that changed (spec 8.4). Outputs going to 0 first, the steering EN low before and
// high after the PWM pins. Before the TX step, so a STOP or a fault stop reaches the pins first (12.1-66).
void pins_step() {
  drive_pin_values(&fw.pin_lf, &fw.pin_lr, &fw.pin_rf, &fw.pin_rr);
  if (fw.pin_sen == 0 && fw.out_sen != 0) {
    hw::write_level(kPinSteerEn, false);
    fw.out_sen = 0;
  }
  for (int pass = 0; pass < 2; ++pass) {
    const bool zero_pass = pass == 0;
    write_pwm_if(kPinLeftFwd, fw.pin_lf, &fw.out_lf, zero_pass);
    write_pwm_if(kPinLeftRev, fw.pin_lr, &fw.out_lr, zero_pass);
    write_pwm_if(kPinRightFwd, fw.pin_rf, &fw.out_rf, zero_pass);
    write_pwm_if(kPinRightRev, fw.pin_rr, &fw.out_rr, zero_pass);
    write_pwm_if(kPinSteerA, fw.pin_sa, &fw.out_sa, zero_pass);
    write_pwm_if(kPinSteerB, fw.pin_sb, &fw.out_sb, zero_pass);
  }
  if (fw.pin_sen != 0 && fw.out_sen == 0) {
    hw::write_level(kPinSteerEn, true);
    fw.out_sen = 1;
  }
  fw.steer_en_prev = fw.pin_sen;
  monitor_end_of_tick();
}

}  // namespace

uint16_t fault_mask() {
  uint16_t m = fw.latched;
  if (fw.softlimit_bit) m = static_cast<uint16_t>(m | kFSoftlimit);
  if (fw.ttl_bit) m = static_cast<uint16_t>(m | kFTtl);
  return m;
}

void clear_c_state() {
  fw.c_valid = 0;
  fw.ttl_bit = 0;
}

void firmware_setup() {
  state_reset();
  hw::begin();  // writes all PWM pins 0, steering EN LOW, drive EN HIGH (out_* = 0 after state_reset)
  uint32_t id = hw::random32();
  for (int i = 0; i < 16 && id == 0; ++i) id = hw::random32();  // boot_id is never 0
  fw.boot_id = id != 0 ? id : 1u;
  fw.chip_id = hw::chip_id() & 0xFFFFFFu;
  fw.rst_reason = hw::reset_reason();
  load_calibration();
  const uint32_t now = hw::now_ms();
  fw.now = fw.boot_ms = fw.last_ms = now;
  fw.state = kStDisarmed;
  fw.latched = static_cast<uint16_t>(kFBoot | (fw.r_valid ? 0u : kFNocal));
  fw.last_input = now;
  drive_boot(now);
  steer_boot(now);
  monitor_boot();
  link_boot();
  fw.next_status = now + status_period();
  emit_banner();
  emit_cal_line();
}

void firmware_loop() {
  const uint32_t now = hw::now_ms();  // step 1
  if (fw.ticked && now == fw.last_ms) {
    link_tx_pump();
    return;
  }
  if (fw.ticked) ++fw.tick;
  fw.ticked = 1;
  fw.last_ms = now;
  fw.now = now;
  monitor_adc();       // 2
  watchdog_step();     // 3
  link_rx();           // 4
  ttl_step();          // 5
  fault_step();        // 6
  steer_mode_clock();  // 7
  drive_ramp(now);     // 8
  steer_output();      // 9
  status_step();       // 10
  pins_step();         // 11 (pins, run counters, thermal ring)
  link_tx_pump();      // 12 (never waits for the UART, spec 5.10)
}

uint8_t debug_state() { return fw.state; }

uint16_t debug_fault_bits() { return fault_mask(); }

}  // namespace ubts_fw
