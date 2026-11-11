// Hardware access (see fw_hw.h). Real chip: ESP8266 Arduino core. SIM: the Arduino-style shim.
#include "fw_hw.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
extern "C" {
#include <user_interface.h>
}

#include "fw_config.h"

namespace ubts_fw {
namespace hw {

namespace {

void init_pwm_pin(uint8_t gpio) {
  digitalWrite(gpio, LOW);
  pinMode(gpio, OUTPUT);
  analogWrite(gpio, 0);
}

}  // namespace

void begin() {
  analogWriteRange(255);
  analogWriteFreq(1000);
  init_pwm_pin(kPinLeftFwd);
  init_pwm_pin(kPinLeftRev);
  init_pwm_pin(kPinRightFwd);
  init_pwm_pin(kPinRightRev);
  init_pwm_pin(kPinSteerA);
  init_pwm_pin(kPinSteerB);
  digitalWrite(kPinSteerEn, LOW);
  pinMode(kPinSteerEn, OUTPUT);
  digitalWrite(kPinDriveEn, HIGH);  // the firmware never lowers it again (spec 8.3)
  pinMode(kPinDriveEn, OUTPUT);
  WiFi.mode(WIFI_OFF);              // as the V2 firmware: radio off
  Serial.begin(kBaud);
  EEPROM.begin(kEepromBytes);
}

uint32_t now_ms() { return static_cast<uint32_t>(millis()); }

uint16_t adc() {
  int v = analogRead(A0);
  if (v < 0) v = 0;
  if (v > 1023) v = 1023;
  return static_cast<uint16_t>(v);
}

void write_pwm(uint8_t gpio, uint8_t v) { analogWrite(gpio, v); }

void write_level(uint8_t gpio, bool high) { digitalWrite(gpio, high ? HIGH : LOW); }

int rx_available() { return Serial.available(); }

int rx_read() { return Serial.read(); }

int tx_room() { return Serial.availableForWrite(); }

size_t tx_write(const uint8_t *data, size_t n) { return Serial.write(data, n); }

uint8_t eeprom_read(int address) { return EEPROM.read(address); }

void eeprom_write(int address, uint8_t value) { EEPROM.write(address, value); }

bool eeprom_commit() { return EEPROM.commit(); }

uint32_t random32() { return ESP.random(); }

uint32_t chip_id() { return ESP.getChipId(); }

uint32_t reset_reason() {
  const struct rst_info *info = ESP.getResetInfoPtr();
  return info != 0 ? info->reason : 0xFFFFFFFFu;
}

}  // namespace hw
}  // namespace ubts_fw
