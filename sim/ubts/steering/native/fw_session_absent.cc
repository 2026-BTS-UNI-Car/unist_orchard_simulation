// FirmwareSession of a native program built without the firmware sources.
//
// build.py compiles this file instead of fw_session.cc, fw_sketch.cc and arduino_shim/sim_mcu.cc when
// firmware/ubts_steer_v3 is missing (for example in an image that copies only sim/ubts). The v1 modes and the v2
// direct_pwm and test_servo modes are unchanged. lockstep_main.cc refuses a firmware-mode startup line before a
// session is used (kFirmwareLinked is false), so reset() and step() are never reached.
#include "fw_session.h"

#include <stdexcept>

namespace ubts_steer {

const bool kFirmwareLinked = false;

FirmwareSession::FirmwareSession(const Config &cfg, SteerModel *model)
    : cfg_(cfg),
      model_(model),
      powered_(false),
      setup_done_(false),
      reset_tick_(0),
      setup_tick_(0),
      stall_until_(0),
      power_on_tick_(-1),
      boot_count_(0),
      rx_lost_(0),
      adc_model_(true),
      adc_script_(0),
      motor_power_(true),
      adc_read_value_(0),
      last_millis_(0xFFFFFFFFu),
      last_stalled_(false) {
  for (int i = 0; i < kChannelCount; ++i) channels_[i] = 0;
  channels_[kChMcuState] = 255;
}

void FirmwareSession::reset(double, FwStepOutput *) {
  throw std::logic_error("firmware mode is not built into this program");
}

void FirmwareSession::step(const FwStepInput &, FwStepOutput *) {
  throw std::logic_error("firmware mode is not built into this program");
}

}  // namespace ubts_steer
