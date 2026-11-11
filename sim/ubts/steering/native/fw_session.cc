// Firmware mode of the lockstep protocol v2 (see fw_session.h).
#include "fw_session.h"

#include <cmath>

#include "fw_main.h"   // firmware/ubts_steer_v3: read-only state view for the change channels
#include "fw_state.h"  // firmware RAM, read only for the servo evaluation channels (report_servo)

namespace ubts_steer {

const bool kFirmwareLinked = true;

using ubts_sim::g_mcu;

namespace {

// GPIO numbers of the firmware pin plan (ESP protocol 8.3, plan A).
const int kGpioLeftFwd = 15;   // D8
const int kGpioLeftRev = 14;   // D5
const int kGpioRightFwd = 0;   // D3
const int kGpioRightRev = 16;  // D0
const int kGpioDriveEn = 5;    // D1
const int kGpioSteerEn = 4;    // D2
const int kGpioSteerA = 12;    // D6
const int kGpioSteerB = 13;    // D7

}  // namespace

FirmwareSession::FirmwareSession(const Config &cfg, SteerModel *model)
    : cfg_(cfg),
      model_(model),
      powered_(true),
      setup_done_(false),
      reset_tick_(0),
      setup_tick_(0),
      stall_until_(0),
      power_on_tick_(-1),
      boot_count_(0),
      rx_lost_(0),
      adc_model_(cfg.fw.adc_model),
      adc_script_(cfg.fw.adc_script_raw),
      motor_power_(cfg.fw.motor_power),
      adc_read_value_(0),
      last_millis_(0xFFFFFFFFu),
      last_stalled_(false) {
  for (int i = 0; i < kChannelCount; ++i) channels_[i] = 0;
  channels_[kChMcuState] = 255;
  if (cfg.mode == kModeFirmware) g_mcu.configure(cfg.mcu_baud, cfg.fw.chip_id, cfg.fw.boot_seed, cfg.fw.eeprom);
}

void FirmwareSession::reset_mcu(int64_t tick, uint32_t reason) {
  setup_done_ = false;
  reset_tick_ = tick;
  setup_tick_ = tick + static_cast<int64_t>(cfg_.fw.boot_delay_ticks);
  stall_until_ = 0;  // a reset ends a stall
  g_mcu.reset_hardware(static_cast<uint32_t>(tick), reason);
  if (!cfg_.fw.boot_garbage.empty()) {
    g_mcu.queue_boot_garbage(static_cast<uint32_t>(tick), &cfg_.fw.boot_garbage[0], cfg_.fw.boot_garbage.size());
  }
}

void FirmwareSession::reset(double delta_eq, FwStepOutput *out) {
  model_->reset(delta_eq);
  g_mcu.session_reset();
  powered_ = true;
  power_on_tick_ = -1;
  boot_count_ = 0;
  rx_lost_ = 0;
  adc_model_ = cfg_.fw.adc_model;
  adc_script_ = cfg_.fw.adc_script_raw;
  motor_power_ = cfg_.fw.motor_power;
  for (int i = 0; i < kChannelCount; ++i) channels_[i] = 0;
  channels_[kChMcuState] = 255;
  adc_read_value_ = 0;
  last_millis_ = 0xFFFFFFFFu;
  last_stalled_ = false;
  reset_mcu(0, 0);  // power-on reset at tick 0
  out->changes.clear();
  out->tx.clear();
  fill_output(0, out);
}

void FirmwareSession::apply_event(int64_t tick, const FwEvent &event) {
  switch (event.kind) {
    case kEvMcuReset:
      if (powered_) reset_mcu(tick, event.arg);
      break;
    case kEvPortOpenReset:
      if (powered_) reset_mcu(tick, 6);
      break;
    case kEvPowerCycle:
      powered_ = false;
      setup_done_ = false;
      stall_until_ = 0;
      power_on_tick_ = tick + static_cast<int64_t>(event.arg);
      g_mcu.reset_hardware(static_cast<uint32_t>(tick), 0);  // pins 0, RX lost, no TX while off
      break;
    case kEvMotorPower:
      motor_power_ = event.arg != 0;
      break;
    case kEvMcuStall:
      if (powered_) stall_until_ = tick + static_cast<int64_t>(event.arg);
      break;
    case kEvAdcSet:
      adc_model_ = false;
      adc_script_ = static_cast<uint16_t>(event.arg);
      break;
    case kEvAdcModel:
      adc_model_ = true;
      break;
    default:
      break;  // the request check refuses unknown kinds
  }
}

void FirmwareSession::run_tick(int64_t n, bool stopped, double speed_abs, double supply_scale) {
  const bool stalled = n < stall_until_;
  bool called = false;
  if (powered_) {
    const bool setup_tick = !setup_done_ && n == setup_tick_;
    if (setup_tick || (setup_done_ && !stalled)) {
      const uint32_t millis_value = static_cast<uint32_t>(static_cast<uint64_t>(n - reset_tick_) +
                                                          static_cast<uint64_t>(cfg_.fw.millis_start));
      const uint16_t adc = adc_model_ ? static_cast<uint16_t>(model_->pot_raw()) : adc_script_;
      g_mcu.begin_tick(static_cast<uint32_t>(n), millis_value, adc);
      if (setup_tick) {
        ::setup();
        setup_done_ = true;
        ++boot_count_;
      }
      if (!stalled) ::loop();
      called = true;
    }
  }
  const bool run = running();
  if (!run) {
    adc_read_value_ = 0;
  } else if (called && g_mcu.adc_was_read()) {
    adc_read_value_ = g_mcu.adc_value();
  }
  last_stalled_ = run && stalled;
  last_millis_ = run ? static_cast<uint32_t>(static_cast<uint64_t>(n - reset_tick_) +
                                             static_cast<uint64_t>(cfg_.fw.millis_start))
                     : 0xFFFFFFFFu;
  double command = 0.0;
  bool enable = false;
  if (run) {
    command = static_cast<double>(g_mcu.pin_value(kGpioSteerA) - g_mcu.pin_value(kGpioSteerB)) / 255.0;
    enable = g_mcu.pin_level(kGpioSteerEn) != 0;
  }
  model_->fw_tick(n, command, enable, !motor_power_, stopped, speed_abs, supply_scale);
}

void FirmwareSession::channel_values(int32_t *v) const {
  for (int i = 0; i < kChannelCount; ++i) v[i] = 0;
  if (running()) {
    v[kChDriveL] = g_mcu.pin_value(kGpioLeftFwd) - g_mcu.pin_value(kGpioLeftRev);
    v[kChDriveR] = g_mcu.pin_value(kGpioRightFwd) - g_mcu.pin_value(kGpioRightRev);
    v[kChDriveEn] = g_mcu.pin_level(kGpioDriveEn);
    v[kChSteerPwm] = g_mcu.pin_value(kGpioSteerA) - g_mcu.pin_value(kGpioSteerB);
    v[kChSteerEn] = g_mcu.pin_level(kGpioSteerEn);
    v[kChMcuState] = ubts_fw::debug_state();
    v[kChFaultBits] = ubts_fw::debug_fault_bits();
    v[kChServoPos] = ubts_fw::fw.pos;
    v[kChServoTgt] = ubts_fw::fw.tgt;
    v[kChServoCmd] = ubts_fw::fw.cmd;
    v[kChSteerMode] = ubts_fw::fw.mode;
  } else {
    v[kChMcuState] = 255;
  }
  v[kChGpioWrites] = static_cast<int32_t>(g_mcu.gpio_writes());
  v[kChEepromCommits] = static_cast<int32_t>(g_mcu.eeprom_commits());
  v[kChAdcRead] = adc_read_value_;
}

void FirmwareSession::fill_output(uint32_t ticks, FwStepOutput *out) const {
  model_->output(ticks, &out->base);
  out->base.pot_raw = adc_model_ ? static_cast<uint16_t>(model_->pot_raw()) : adc_script_;
  out->base.fault_bits = static_cast<uint16_t>(channels_[kChFaultBits]);
  out->drive_left = static_cast<double>(channels_[kChDriveL]) / 255.0;
  out->drive_right = static_cast<double>(channels_[kChDriveR]) / 255.0;
  out->drive_en = channels_[kChDriveEn] != 0;
  out->mcu_state = static_cast<uint8_t>(channels_[kChMcuState]);
  out->run_flags = static_cast<uint8_t>((powered_ ? 1u : 0u) | (setup_done_ ? 2u : 0u) | (last_stalled_ ? 4u : 0u) |
                                        (motor_power_ ? 8u : 0u));
  out->boot_count = boot_count_;
  out->mcu_millis = last_millis_;
  out->rx_lost_total = rx_lost_;
}

void FirmwareSession::step(const FwStepInput &in, FwStepOutput *out) {
  const int64_t a = model_->ticks_total();
  const int64_t ticks = model_->ticks_for(in.dt_s);
  const double speed_abs = std::fabs(in.speed_m_s);
  out->changes.clear();
  out->tx.clear();
  size_t ev = 0, rx = 0;
  int32_t values[kChannelCount];
  for (int64_t i = 0; i < ticks; ++i) {
    const int64_t n = a + i;
    if (!powered_ && power_on_tick_ >= 0 && n >= power_on_tick_) {
      powered_ = true;
      power_on_tick_ = -1;
      reset_mcu(n, 0);  // power-on reset after a power cycle
    }
    while (ev < in.events.size() && in.events[ev].offset == i) apply_event(n, in.events[ev++]);
    while (rx < in.rx.size() && in.rx[rx].offset == i) {
      const uint8_t byte = in.rx[rx++].byte;
      if (!running() || !g_mcu.rx_push(byte)) ++rx_lost_;
    }
    run_tick(n, in.stopped, speed_abs, in.supply_scale);
    channel_values(values);
    for (int ch = 1; ch < kChannelCount; ++ch) {
      if (ch == kChAdcRead && !cfg_.fw.report_adc) continue;
      if (ch >= kChServoPos && !cfg_.fw.report_servo) continue;
      if (values[ch] != channels_[ch]) {
        FwChange change;
        change.offset = static_cast<uint16_t>(i);
        change.channel = static_cast<uint8_t>(ch);
        change.value = values[ch];
        out->changes.push_back(change);
        channels_[ch] = values[ch];
      }
    }
    if (!cfg_.fw.report_adc) channels_[kChAdcRead] = values[kChAdcRead];
    if (!cfg_.fw.report_servo) {
      for (int ch = kChServoPos; ch < kChannelCount; ++ch) channels_[ch] = values[ch];
    }
  }
  model_->advance_clock(in.dt_s);
  g_mcu.take_tx_done_before(static_cast<uint32_t>(a + ticks), &out->tx);
  fill_output(static_cast<uint32_t>(ticks), out);
}

}  // namespace ubts_steer
