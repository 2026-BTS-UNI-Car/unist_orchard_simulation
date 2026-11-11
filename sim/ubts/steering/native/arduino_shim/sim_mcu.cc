// Simulated ESP8266 hardware and the Arduino-style API on top of it (see sim_mcu.h).
#include "sim_mcu.h"

#include <string.h>

#include <cstddef>

#include "Arduino.h"
#include "EEPROM.h"
#include "ESP8266WiFi.h"

namespace ubts_sim {

SimMcu g_mcu;

uint64_t splitmix64(uint64_t *state) {
  uint64_t z = (*state += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

SimMcu::SimMcu()
    : byte_units_(50),
      chip_id_(0),
      boot_seed_(0),
      rng_state_(0),
      ram_size_(0),
      ram_dirty_(false),
      eeprom_commits_(0),
      tick_(0),
      millis_(0),
      adc_value_(0),
      adc_read_(false),
      gpio_writes_(0),
      pwm_range_(1023),
      pwm_freq_(1000),
      serial_baud_(0),
      rx_head_(0),
      rx_count_(0),
      tx_head_(0),
      line_end_u_(0),
      tx_fifo_model_(false),
      commit_fail_(false),
      cpu_units_(0),
      tx_block_events_(0),
      tx_block_units_(0) {
  memset(pin_write_u_, 0, sizeof pin_write_u_);
  memset(image_, 0xFF, sizeof image_);
  memset(flash_, 0xFF, sizeof flash_);
  memset(ram_, 0, sizeof ram_);
  memset(&reset_info_, 0, sizeof reset_info_);
  memset(value_, 0, sizeof value_);
  memset(level_, 0, sizeof level_);
  memset(rx_, 0, sizeof rx_);
}

void SimMcu::configure(uint32_t baud, uint32_t chip_id, uint64_t boot_seed, const uint8_t *eeprom_image) {
  byte_units_ = baud > 0 ? static_cast<uint32_t>(5760000u / baud) : 50u;  // 10 bits per byte, in u = 1/576000 s
  chip_id_ = chip_id;
  boot_seed_ = boot_seed;
  if (eeprom_image != 0) {
    memcpy(image_, eeprom_image, sizeof image_);
  } else {
    memset(image_, 0xFF, sizeof image_);
  }
  session_reset();
}

void SimMcu::session_reset() {
  memcpy(flash_, image_, sizeof flash_);
  rng_state_ = boot_seed_;
  eeprom_commits_ = 0;
  gpio_writes_ = 0;
  tx_.clear();
  tx_head_ = 0;
  line_end_u_ = 0;
  tx_block_events_ = 0;
  tx_block_units_ = 0;
  reset_hardware(0, 0);
}

void SimMcu::reset_hardware(uint32_t tick, uint32_t reason) {
  memset(value_, 0, sizeof value_);
  memset(level_, 0, sizeof level_);
  rx_head_ = rx_count_ = 0;
  memset(ram_, 0, sizeof ram_);
  ram_size_ = 0;
  ram_dirty_ = false;
  pwm_range_ = 1023;  // core defaults before setup()
  pwm_freq_ = 1000;
  serial_baud_ = 0;
  adc_read_ = false;
  reset_info_.reason = reason;
  // unsent bytes are lost with the UART state; done ticks are non-decreasing, so cut the tail
  size_t keep = tx_head_;
  while (keep < tx_.size() && tx_[keep].done_tick < tick) ++keep;
  tx_.resize(keep);
  line_end_u_ = static_cast<uint64_t>(tick) * kUnitsPerTick;
  tick_ = tick;
  cpu_units_ = 0;
  memset(pin_write_u_, 0, sizeof pin_write_u_);
}

void SimMcu::queue_boot_garbage(uint32_t tick, const uint8_t *bytes, size_t n) {
  for (size_t i = 0; i < n; ++i) schedule(bytes[i], 1, tick);
}

void SimMcu::begin_tick(uint32_t tick, uint32_t millis_value, uint16_t adc_value) {
  tick_ = tick;
  millis_ = millis_value;
  adc_value_ = adc_value;
  adc_read_ = false;
  cpu_units_ = 0;
}

bool SimMcu::rx_push(uint8_t byte) {
  bool kept = true;
  if (rx_count_ >= kRxBufferBytes - 1) {  // full ring (255 B): the core 3.1.2 ISR drops the oldest byte
    rx_head_ = (rx_head_ + 1) % kRxBufferBytes;
    --rx_count_;
    kept = false;
  }
  rx_[(rx_head_ + rx_count_) % kRxBufferBytes] = byte;
  ++rx_count_;
  return kept;
}

void SimMcu::schedule(uint8_t byte, uint8_t flags, uint32_t tick) {
  schedule_at(byte, flags, tick, static_cast<uint64_t>(tick) * kUnitsPerTick);
}

void SimMcu::schedule_at(uint8_t byte, uint8_t flags, uint32_t tick, uint64_t start_u) {
  const uint64_t end = (line_end_u_ > start_u ? line_end_u_ : start_u) + byte_units_;
  line_end_u_ = end;
  TxByte b;
  b.queued_tick = tick;
  b.done_tick = static_cast<uint32_t>((end + kUnitsPerTick - 1) / kUnitsPerTick - 1);  // ceil(E / 576) - 1
  b.byte = byte;
  b.flags = flags;
  b.end_u = end;
  tx_.push_back(b);
}

int SimMcu::tx_fifo_fill() const {
  // A pending byte is in the FIFO while its start (end - one byte time) is still ahead; the byte being shifted out
  // is not counted. Bytes are scheduled in order, so count from the newest.
  const uint64_t now = now_units();
  int fill = 0;
  for (size_t i = tx_.size(); i > tx_head_; --i) {
    if (tx_[i - 1].end_u - byte_units_ <= now) break;
    ++fill;
  }
  return fill;
}

void SimMcu::take_tx_done_before(uint32_t tick_limit, std::vector<TxByte> *out) {
  while (tx_head_ < tx_.size() && tx_[tx_head_].done_tick < tick_limit) out->push_back(tx_[tx_head_++]);
  if (tx_head_ == tx_.size()) {
    tx_.clear();
    tx_head_ = 0;
  } else if (tx_head_ > 4096) {
    tx_.erase(tx_.begin(), tx_.begin() + static_cast<std::ptrdiff_t>(tx_head_));
    tx_head_ = 0;
  }
}

void SimMcu::pin_mode(uint8_t pin, uint8_t mode) {
  (void)pin;
  (void)mode;
}

void SimMcu::digital_write(uint8_t pin, uint8_t val) {
  ++gpio_writes_;
  if (pin >= kGpioCount) return;
  level_[pin] = val ? 1 : 0;
  value_[pin] = val ? 255 : 0;
  pin_write_u_[pin] = now_units();
}

int SimMcu::digital_read(uint8_t pin) const { return pin < kGpioCount ? level_[pin] : 0; }

int SimMcu::analog_read(uint8_t pin) {
  if (pin != A0) return 0;
  adc_read_ = true;
  return adc_value_;
}

void SimMcu::analog_write(uint8_t pin, int val) {
  ++gpio_writes_;
  if (pin >= kGpioCount) return;
  if (val < 0) val = 0;
  if (val > 255) val = 255;
  value_[pin] = static_cast<uint8_t>(val);
  level_[pin] = val > 0 ? 1 : 0;
  pin_write_u_[pin] = now_units();
}

int SimMcu::serial_read() {
  if (rx_count_ == 0) return -1;
  const uint8_t c = rx_[rx_head_];
  rx_head_ = (rx_head_ + 1) % kRxBufferBytes;
  --rx_count_;
  return c;
}

int SimMcu::serial_peek() const { return rx_count_ == 0 ? -1 : rx_[rx_head_]; }

int SimMcu::serial_room() const {
  if (tx_fifo_model_) return kTxFifoBytes - tx_fifo_fill();  // core 3.1.2 uart_tx_free()
  const size_t pending = tx_pending();
  return pending >= kTxCapacity ? 0 : static_cast<int>(kTxCapacity - pending);
}

size_t SimMcu::serial_write(uint8_t c) {
  if (tx_fifo_model_) {
    if (tx_fifo_fill() >= kTxFifoFull) {  // the core would busy-wait here: add the wait to the CPU time
      ++tx_block_events_;
      const uint64_t start = now_units();
      while (tx_fifo_fill() >= kTxFifoFull) {
        // the oldest byte still in the FIFO enters the shift register at its start time
        const size_t first = tx_.size() - static_cast<size_t>(tx_fifo_fill());
        cpu_units_ += tx_[first].end_u - byte_units_ - now_units();
      }
      tx_block_units_ += now_units() - start;
    }
    schedule_at(c, 0, tick_, now_units());
    return 1;
  }
  if (serial_room() <= 0) return 0;
  schedule(c, 0, tick_);
  return 1;
}

void SimMcu::eeprom_begin(size_t size) {
  ram_size_ = size < kEepromFlashBytes ? size : kEepromFlashBytes;
  memcpy(ram_, flash_, ram_size_);
  ram_dirty_ = false;
}

uint8_t SimMcu::eeprom_read(int address) const {
  if (address < 0 || static_cast<size_t>(address) >= ram_size_) return 0;
  return ram_[address];
}

void SimMcu::eeprom_write(int address, uint8_t val) {
  if (address < 0 || static_cast<size_t>(address) >= ram_size_) return;
  if (ram_[address] != val) {
    ram_[address] = val;
    ram_dirty_ = true;
  }
}

bool SimMcu::eeprom_commit() {
  if (ram_size_ == 0) return false;
  if (!ram_dirty_) return true;  // ESP8266 core: nothing to write
  if (commit_fail_) return false;  // test option: the flash write fails, the RAM copy stays dirty
  memcpy(flash_, ram_, ram_size_);
  ram_dirty_ = false;
  ++eeprom_commits_;  // the SIM counts a commit as 0 ticks (spec 12.1 item 26)
  return true;
}

uint32_t SimMcu::random32() {
  for (;;) {
    const uint32_t v = static_cast<uint32_t>(splitmix64(&rng_state_) & 0xFFFFFFFFu);
    if (v != 0) return v;  // zero outputs are skipped (spec 10.1)
  }
}

}  // namespace ubts_sim

// --- Arduino API ------------------------------------------------------------------------------------------------
using ubts_sim::g_mcu;

unsigned long millis() { return g_mcu.millis_now(); }
unsigned long micros() { return g_mcu.millis_now() * 1000ul; }
void delay(unsigned long ms) { (void)ms; }
void yield() {}
void pinMode(uint8_t pin, uint8_t mode) { g_mcu.pin_mode(pin, mode); }
void digitalWrite(uint8_t pin, uint8_t val) { g_mcu.digital_write(pin, val); }
int digitalRead(uint8_t pin) { return g_mcu.digital_read(pin); }
int analogRead(uint8_t pin) { return g_mcu.analog_read(pin); }
void analogWrite(uint8_t pin, int val) { g_mcu.analog_write(pin, val); }
void analogWriteFreq(uint32_t freq) { g_mcu.analog_write_freq(freq); }
void analogWriteRange(uint32_t range) { g_mcu.analog_write_range(range); }

HardwareSerial Serial;
void HardwareSerial::begin(unsigned long baud) { g_mcu.serial_begin(baud); }
int HardwareSerial::available() { return g_mcu.serial_available(); }
int HardwareSerial::read() { return g_mcu.serial_read(); }
int HardwareSerial::peek() { return g_mcu.serial_peek(); }
int HardwareSerial::availableForWrite() { return g_mcu.serial_room(); }
size_t HardwareSerial::write(uint8_t c) { return g_mcu.serial_write(c); }
size_t HardwareSerial::write(const uint8_t *buffer, size_t size) {
  size_t n = 0;
  while (n < size && g_mcu.serial_write(buffer[n]) == 1) ++n;
  return n;
}
void HardwareSerial::flush() {}

EEPROMClass EEPROM;
void EEPROMClass::begin(size_t size) { g_mcu.eeprom_begin(size); }
uint8_t EEPROMClass::read(int const address) { return g_mcu.eeprom_read(address); }
void EEPROMClass::write(int const address, uint8_t const val) { g_mcu.eeprom_write(address, val); }
bool EEPROMClass::commit() { return g_mcu.eeprom_commit(); }
bool EEPROMClass::end() { return g_mcu.eeprom_commit(); }

EspClass ESP;
uint32_t EspClass::getChipId() { return g_mcu.chip_id(); }
uint32_t EspClass::random() { return g_mcu.random32(); }
struct rst_info *EspClass::getResetInfoPtr() { return g_mcu.reset_info(); }

ESP8266WiFiClass WiFi;
bool ESP8266WiFiClass::mode(WiFiMode_t m) {
  (void)m;
  return true;
}
