// Firmware mode of the lockstep protocol v2 (docs/steering/LOCKSTEP_PROTOCOL_KO.md section 11).
//
// The firmware sources (firmware/ubts_steer_v3, through the Arduino-style shim), the actuator model and the
// potentiometer model run in this process. One FW_STEP runs N ticks of 1 ms (cumulative tick rule); per tick:
//   (1) events of the tick (resets, power, motor supply, stall, ADC source) in list order
//   (2) host bytes of the tick -> the RX ring of 255 B (lost when the MCU is off or before setup(); a full ring
//       drops its oldest byte, as the ESP8266 core 3.1.2 does)
//   (3) setup() + loop() at the setup tick, else loop() when set up and not stalled
//   (4) actuator tick with (D6 - D7) / 255 and D2 (0 and 0 when not running; driver output 0 without motor supply)
//   (5) ADC sample at the end of the tick (the next analogRead in model mode)
//   (6) bytes written this tick are queued at tick n; the UART rule gives their done ticks
// Drive duties are outputs only (the plant runs the drive): left (D8 - D5) / 255, right (D3 - D0) / 255, EN D1.
#ifndef UBTS_STEER_FW_SESSION_H
#define UBTS_STEER_FW_SESSION_H

#include <stdint.h>

#include <vector>

#include "arduino_shim/sim_mcu.h"
#include "steer_config.h"
#include "steer_model.h"

namespace ubts_steer {

// true when the firmware sources are linked in (fw_session.cc); false in a program that build.py built
// without them (fw_session_absent.cc): lockstep_main.cc then refuses a firmware-mode startup line (exit 2).
extern const bool kFirmwareLinked;

enum FwEventKind {
  kEvMcuReset = 1,       // arg: reset reason 1..6
  kEvPowerCycle = 2,     // arg: off ticks 1..60000, power-on reset (reason 0) after them
  kEvPortOpenReset = 3,  // arg 0: reset reason 6
  kEvMotorPower = 4,     // arg 0 off, 1 on
  kEvMcuStall = 5,       // arg: ticks 1..60000 without loop()
  kEvAdcSet = 6,         // arg: raw 0..1023 (script value from this tick)
  kEvAdcModel = 7        // arg 0: model value from this tick
};

enum FwChannel {
  kChDriveL = 1,
  kChDriveR = 2,
  kChDriveEn = 3,
  kChSteerPwm = 4,
  kChSteerEn = 5,
  kChMcuState = 6,
  kChFaultBits = 7,
  kChGpioWrites = 8,
  kChEepromCommits = 9,
  kChAdcRead = 10,
  // SIM evaluation channels (startup firmware.report_servo; LOCKSTEP 11.6): the firmware's own servo values,
  // read from its RAM after the tick (not visible on the real device except through the status line).
  kChServoPos = 11,   // P[n], permille (fw.pos)
  kChServoTgt = 12,   // servo target tgt, permille
  kChServoCmd = 13,   // rate-limited internal target cmd, permille
  kChSteerMode = 14,  // steering mode 0 OFF, 1 ACTIVE, 2 HOLD, 3 JOG, 4 POL
  kChannelCount = 15  // index 0 unused
};

struct FwEvent {
  uint16_t offset;
  uint16_t kind;
  uint32_t arg;
};

struct FwRx {
  uint16_t offset;
  uint8_t byte;
};

struct FwStepInput {
  double dt_s, speed_m_s, supply_scale;
  bool stopped;
  std::vector<FwEvent> events;
  std::vector<FwRx> rx;
};

struct FwChange {
  uint16_t offset;
  uint8_t channel;
  int32_t value;
};

struct FwStepOutput {
  StepOutput base;  // v1 block; drive and serial fields below
  double drive_left, drive_right;
  bool drive_en;
  std::vector<FwChange> changes;
  std::vector<ubts_sim::TxByte> tx;
  uint8_t mcu_state;  // 0..3, 255 when not running
  uint8_t run_flags;  // bit0 powered, bit1 setup done, bit2 stalled, bit3 motor supply
  uint16_t boot_count;
  uint32_t mcu_millis;  // 0xFFFFFFFF when not running
  uint32_t rx_lost_total;
};

class FirmwareSession {
 public:
  FirmwareSession(const Config &cfg, SteerModel *model);
  // RESET_REQUEST: v1 reset of the model, EEPROM = startup image, RNG = boot_seed, counters 0,
  // MCU power-on reset at tick 0 (setup() at tick boot_delay_ticks).
  void reset(double delta_eq, FwStepOutput *out);
  // Tick count of a step (cumulative rule), for the request checks.
  int64_t ticks_for(double dt_s) const { return model_->ticks_for(dt_s); }
  void step(const FwStepInput &in, FwStepOutput *out);

 private:
  bool running() const { return powered_ && setup_done_; }
  void reset_mcu(int64_t tick, uint32_t reason);
  void apply_event(int64_t tick, const FwEvent &event);
  void run_tick(int64_t n, bool stopped, double speed_abs, double supply_scale);
  void channel_values(int32_t *values) const;
  void fill_output(uint32_t ticks, FwStepOutput *out) const;

  const Config &cfg_;
  SteerModel *model_;
  bool powered_, setup_done_;
  int64_t reset_tick_, setup_tick_, stall_until_, power_on_tick_;
  uint16_t boot_count_;
  uint32_t rx_lost_;
  bool adc_model_;
  uint16_t adc_script_;
  bool motor_power_;
  int32_t channels_[kChannelCount];
  int32_t adc_read_value_;
  uint32_t last_millis_;
  bool last_stalled_;
};

}  // namespace ubts_steer

#endif  // UBTS_STEER_FW_SESSION_H
