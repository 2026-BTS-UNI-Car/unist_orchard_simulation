// Simulated ESP8266 hardware behind the Arduino-style shim (LOCKSTEP_PROTOCOL_KO.md section 11).
//
// One SimMcu per process (g_mcu): GPIO values, the UART RX ring (256 B, holds 255, drops the oldest byte), the UART
// TX line with the lockstep timing rule, the EEPROM flash (survives MCU resets inside a session) and its RAM copy,
// the deterministic ESP.random() stream (splitmix64), chip id and reset reason, and the tick clock of millis().
// The lockstep session (fw_session.h) and the native unit test drive it; the firmware sees only the Arduino API.
//
// Test options (native unit test only; the lockstep session never sets them):
//   * set_tx_fifo_model(true): the ESP8266 core 3.1.2 UART0 TX rules instead of the lockstep simplification:
//     availableForWrite() = 128 - FIFO fill, and Serial.write() of a byte waits while the fill is 127 or more
//     (uart.cpp: uart_tx_free, uart_tx_fifo_full). The wait is not run: it is added to a CPU time inside the
//     tick, counted in tx_block_events() / tx_block_units(), and later pin writes of the tick carry that time
//     (pin_write_units()). A byte that goes out while the line is idle enters the shift register at once.
//   * set_eeprom_commit_fail(true): EEPROM.commit() of changed bytes fails (flash untouched, RAM copy stays dirty).
#ifndef UBTS_SIM_MCU_H
#define UBTS_SIM_MCU_H

#include <stddef.h>
#include <stdint.h>

#include <vector>

#include "user_interface.h"

// The sketch entry points (firmware/ubts_steer_v3/ubts_steer_v3.ino).
void setup();
void loop();

namespace ubts_sim {

const uint32_t kUnitsPerTick = 576;      // time unit u = 1/576000 s, one tick = 1 ms
const size_t kRxBufferBytes = 256;       // ESP8266 core RX ring; it holds kRxBufferBytes - 1 = 255 bytes
const size_t kTxCapacity = 16384;        // SIM UART queue; no 128 B FIFO limit (protocol 11.5 simplification)
const int kTxFifoBytes = 128;            // ESP8266 UART0 TX FIFO (the optional FIFO model)
const int kTxFifoFull = 127;             // core 3.1.2 uart_tx_fifo_full(): fill >= 0x7f
const size_t kEepromFlashBytes = 256;
const int kGpioCount = 18;               // GPIO 0..16 and A0 (17)

struct TxByte {
  uint32_t queued_tick;  // tick in which the firmware wrote the byte (or the reset tick for boot garbage)
  uint32_t done_tick;    // tick in which its stop bit ends
  uint8_t byte;
  uint8_t flags;         // bit0: boot garbage (not written by the firmware)
  uint64_t end_u;        // end of its stop bit in u (SIM bookkeeping; not sent in FW_STEP_RESPONSE)
};

class SimMcu {
 public:
  SimMcu();

  // Session (lockstep RESET): flash = image, RNG = seed, counters 0, RX/TX empty, pins 0, line idle at tick 0.
  void configure(uint32_t baud, uint32_t chip_id, uint64_t boot_seed, const uint8_t *eeprom_image);
  void session_reset();

  // MCU reset or power off at `tick`: pins 0 (floating, pull-downs), RX buffer empty, EEPROM RAM copy lost,
  // TX bytes not finished before `tick` are discarded and the line is idle from the start of `tick`.
  void reset_hardware(uint32_t tick, uint32_t reason);
  void queue_boot_garbage(uint32_t tick, const uint8_t *bytes, size_t n);

  // Tick context, set before setup()/loop() of a tick.
  void begin_tick(uint32_t tick, uint32_t millis_value, uint16_t adc_value);
  bool adc_was_read() const { return adc_read_; }
  uint16_t adc_value() const { return adc_value_; }

  // Host byte arriving. A full ring (255 B) drops its oldest byte to take the new one, like the core 3.1.2
  // UART ISR (UART_DISCARD_NEWEST is not defined there). Returns false when a byte was lost.
  bool rx_push(uint8_t byte);
  void take_tx_done_before(uint32_t tick_limit, std::vector<TxByte> *out);
  size_t tx_pending() const { return tx_.size() - tx_head_; }

  int pin_value(int gpio) const { return gpio >= 0 && gpio < kGpioCount ? value_[gpio] : 0; }
  int pin_level(int gpio) const { return gpio >= 0 && gpio < kGpioCount ? level_[gpio] : 0; }
  uint32_t gpio_writes() const { return gpio_writes_; }
  uint32_t eeprom_commits() const { return eeprom_commits_; }
  const uint8_t *eeprom_flash() const { return flash_; }

  // Test options (see the file comment). They survive configure() and session_reset().
  void set_tx_fifo_model(bool on) { tx_fifo_model_ = on; }
  void set_eeprom_commit_fail(bool on) { commit_fail_ = on; }
  uint32_t tx_block_events() const { return tx_block_events_; }  // Serial.write calls that would have waited
  uint64_t tx_block_units() const { return tx_block_units_; }    // total wait, u
  uint64_t pin_write_units(int gpio) const {                     // time of the last write of a pin, u
    return gpio >= 0 && gpio < kGpioCount ? pin_write_u_[gpio] : 0;
  }
  uint64_t now_units() const { return static_cast<uint64_t>(tick_) * kUnitsPerTick + cpu_units_; }

  // --- backend of the Arduino API (sim_mcu.cc) ---
  unsigned long millis_now() const { return millis_; }
  void pin_mode(uint8_t pin, uint8_t mode);
  void digital_write(uint8_t pin, uint8_t val);
  int digital_read(uint8_t pin) const;
  int analog_read(uint8_t pin);
  void analog_write(uint8_t pin, int val);
  void analog_write_range(uint32_t range) { pwm_range_ = range; }
  void analog_write_freq(uint32_t freq) { pwm_freq_ = freq; }
  uint32_t pwm_range() const { return pwm_range_; }
  uint32_t pwm_freq() const { return pwm_freq_; }
  void serial_begin(unsigned long baud) { serial_baud_ = baud; }
  unsigned long serial_baud() const { return serial_baud_; }
  int serial_available() const { return static_cast<int>(rx_count_); }
  int serial_read();
  int serial_peek() const;
  int serial_room() const;
  size_t serial_write(uint8_t c);
  void eeprom_begin(size_t size);
  uint8_t eeprom_read(int address) const;
  void eeprom_write(int address, uint8_t val);
  bool eeprom_commit();
  uint32_t chip_id() const { return chip_id_; }
  uint32_t random32();
  rst_info *reset_info() { return &reset_info_; }

 private:
  void schedule(uint8_t byte, uint8_t flags, uint32_t tick);
  void schedule_at(uint8_t byte, uint8_t flags, uint32_t tick, uint64_t start_u);
  int tx_fifo_fill() const;  // FIFO model: bytes waiting behind the shift register at now_units()

  uint32_t byte_units_;
  uint32_t chip_id_;
  uint64_t boot_seed_, rng_state_;
  uint8_t image_[kEepromFlashBytes];
  uint8_t flash_[kEepromFlashBytes];
  uint8_t ram_[kEepromFlashBytes];
  size_t ram_size_;
  bool ram_dirty_;
  uint32_t eeprom_commits_;
  rst_info reset_info_;
  uint32_t tick_;
  unsigned long millis_;
  uint16_t adc_value_;
  bool adc_read_;
  uint8_t value_[kGpioCount];
  uint8_t level_[kGpioCount];
  uint32_t gpio_writes_;
  uint32_t pwm_range_, pwm_freq_;
  unsigned long serial_baud_;
  uint8_t rx_[kRxBufferBytes];
  size_t rx_head_, rx_count_;
  std::vector<TxByte> tx_;
  size_t tx_head_;
  uint64_t line_end_u_;  // end of the last scheduled stop bit, in u
  bool tx_fifo_model_, commit_fail_;
  uint64_t cpu_units_;   // FIFO model: CPU time spent in waits inside this tick, u
  uint32_t tx_block_events_;
  uint64_t tx_block_units_;
  uint64_t pin_write_u_[kGpioCount];
};

extern SimMcu g_mcu;

// splitmix64 step (the SIM "hardware" RNG, ESP protocol 10.1).
uint64_t splitmix64(uint64_t *state);

}  // namespace ubts_sim

#endif  // UBTS_SIM_MCU_H
