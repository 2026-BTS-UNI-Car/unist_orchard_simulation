// ESP8266 EEPROM library subset (flash emulation) for the native SIM build. The emulated flash survives MCU
// resets inside one lockstep session; RESET loads the startup image (sim_mcu.h).
#ifndef UBTS_SIM_EEPROM_H
#define UBTS_SIM_EEPROM_H

#include <stddef.h>
#include <stdint.h>

class EEPROMClass {
 public:
  void begin(size_t size);
  uint8_t read(int const address);
  void write(int const address, uint8_t const val);
  bool commit();
  bool end();
};

extern EEPROMClass EEPROM;

#endif  // UBTS_SIM_EEPROM_H
