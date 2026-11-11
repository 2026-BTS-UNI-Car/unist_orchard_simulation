// Hardware access of the steering firmware V3: the only part that calls the Arduino / ESP8266 core API.
// The control logic calls these functions only. The SIM build compiles the same file against the Arduino-style
// shim in sim/ubts/steering/native/arduino_shim/.
#ifndef UBTS_STEER_V3_FW_HW_H
#define UBTS_STEER_V3_FW_HW_H

#include <stddef.h>
#include <stdint.h>

namespace ubts_fw {
namespace hw {

// setup() hardware order (spec 8.3): PWM range/frequency, drive direction pins LOW/OUTPUT/0, steering PWM pins
// the same, steering EN LOW/OUTPUT, drive EN HIGH/OUTPUT; then Wi-Fi off, UART, EEPROM.begin(256).
void begin();

uint32_t now_ms();                         // millis() as u32
uint16_t adc();                            // analogRead(A0), clamped to 0..1023
void write_pwm(uint8_t gpio, uint8_t v);   // analogWrite
void write_level(uint8_t gpio, bool high); // digitalWrite

int rx_available();
int rx_read();                             // -1 when empty
int tx_room();                             // Serial.availableForWrite()
size_t tx_write(const uint8_t *data, size_t n);

uint8_t eeprom_read(int address);
void eeprom_write(int address, uint8_t value);
bool eeprom_commit();

uint32_t random32();                       // ESP.random()
uint32_t chip_id();                        // ESP.getChipId()
uint32_t reset_reason();                   // ESP.getResetInfoPtr()->reason

}  // namespace hw
}  // namespace ubts_fw

#endif  // UBTS_STEER_V3_FW_HW_H
