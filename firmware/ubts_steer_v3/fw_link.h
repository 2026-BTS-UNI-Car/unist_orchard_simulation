// Serial link: host line reader and parser, commands (spec 2, 4), MCU lines and the 512 B TX ring (spec 5).
#ifndef UBTS_STEER_V3_FW_LINK_H
#define UBTS_STEER_V3_FW_LINK_H

#include <stdint.h>

namespace ubts_fw {

void link_boot();     // empty line buffer (normal state), empty TX ring
void link_rx();       // loop step 4: at most 64 received bytes, each line runs at its LF
void link_tx_pump();  // loop step 12: TX ring -> UART, at most availableForWrite() - 1 bytes (never waits)

void emit_banner();
void emit_cal_line();
void emit_status_line();
void emit_fault_line(uint16_t bit, int32_t arg);  // F <CODE> <arg> <ms>
void emit_watchdog_line();                        // W <ms>
void emit_pol_result(bool ok, int32_t d1, int32_t d2);

}  // namespace ubts_fw

#endif  // UBTS_STEER_V3_FW_LINK_H
