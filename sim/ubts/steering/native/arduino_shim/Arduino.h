// Arduino / ESP8266 core API subset for the native SIM build of firmware/ubts_steer_v3 (decision R1).
//
// Only what the firmware uses. Every call goes to the simulated MCU of this process (sim_mcu.h): millis() is the
// lockstep tick clock, analogRead(A0) the potentiometer model or the script value, analogWrite/digitalWrite are
// captured as driver inputs, Serial is the lockstep UART. Signatures follow the ESP8266 Arduino core 3.x.
// Extends the idea of the V2 native harness (rover_control_test/tests/native/Arduino.h); V2 files are untouched.
#ifndef UBTS_SIM_ARDUINO_H
#define UBTS_SIM_ARDUINO_H

#include <stddef.h>
#include <stdint.h>

#define LOW 0x0
#define HIGH 0x1
#define INPUT 0x00
#define OUTPUT 0x01

// ESP8266 analog input and the NodeMCU pin labels (variants/nodemcu/pins_arduino.h).
static const uint8_t A0 = 17;
static const uint8_t D0 = 16;
static const uint8_t D1 = 5;
static const uint8_t D2 = 4;
static const uint8_t D3 = 0;
static const uint8_t D4 = 2;
static const uint8_t D5 = 14;
static const uint8_t D6 = 12;
static const uint8_t D7 = 13;
static const uint8_t D8 = 15;

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);  // never used by the firmware loop (it would block); a no-op here
void yield();

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);
void analogWriteFreq(uint32_t freq);
void analogWriteRange(uint32_t range);

class HardwareSerial {
 public:
  void begin(unsigned long baud);
  int available();
  int read();
  int peek();
  int availableForWrite();
  size_t write(uint8_t c);
  size_t write(const uint8_t *buffer, size_t size);
  void flush();
};

extern HardwareSerial Serial;

struct rst_info;  // complete type in user_interface.h

class EspClass {
 public:
  uint32_t getChipId();
  uint32_t random();
  struct rst_info *getResetInfoPtr();
};

extern EspClass ESP;

#endif  // UBTS_SIM_ARDUINO_H
