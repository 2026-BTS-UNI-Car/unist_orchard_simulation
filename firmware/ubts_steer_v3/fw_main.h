// Entry points of the steering firmware V3 and the loop() tick order (spec 7.1).
#ifndef UBTS_STEER_V3_FW_MAIN_H
#define UBTS_STEER_V3_FW_MAIN_H

#include <stdint.h>

namespace ubts_fw {

void firmware_setup();  // Arduino setup(): RAM to power-on values, pins, UART, EEPROM, boot_id, banner
void firmware_loop();   // Arduino loop(): the 12 steps once per millisecond

uint16_t fault_mask();  // status line fl: latches, BOOT and the SOFTLIMIT / TTL conditions
void clear_c_state();   // no valid C, TTL bit 0 (STOP, DISARM, leaving ARMED)

// Read-only view for the SIM and the conformance runner (lockstep change channels). No control effect.
uint8_t debug_state();        // McuState (0 before setup)
uint16_t debug_fault_bits();  // == fault_mask()

}  // namespace ubts_fw

#endif  // UBTS_STEER_V3_FW_MAIN_H
