// Build constants and the pin plan of the steering firmware V3 (docs/steering/ESP_PROTOCOL_V3_DRAFT_KO.md).
//
// Compile-time defines (all optional; the defaults are the real-chip values):
//   UBTS_STEER_SIM     1 only in the native SIM build (banner sim=1), default 0 (spec 10.4)
//   UBTS_STEER_HW      hardware profile token, default front_steer_v0 (spec 10.4, placeholder P16)
//   UBTS_STEER_BAUD    UART baud, default 115200 (spec 10.6, placeholder P13)
//   UBTS_STEER_FW_SRC  firmware source id token, 8 uppercase hex digits (spec 10.5); the build tool computes it,
//                      without it the banner shows build=00000000
// The defines change only the banner and the hardware layer (fw_hw.cpp), never the control logic.
#ifndef UBTS_STEER_V3_FW_CONFIG_H
#define UBTS_STEER_V3_FW_CONFIG_H

#include <stdint.h>

#ifndef UBTS_STEER_SIM
#define UBTS_STEER_SIM 0
#endif
#ifndef UBTS_STEER_HW
#define UBTS_STEER_HW front_steer_v0
#endif
#ifndef UBTS_STEER_BAUD
#define UBTS_STEER_BAUD 115200
#endif
#ifndef UBTS_STEER_FW_SRC
#define UBTS_STEER_FW_SRC 00000000
#endif

#define UBTS_FW_STR2(x) #x
#define UBTS_FW_STR(x) UBTS_FW_STR2(x)

namespace ubts_fw {

static const char kFwBuild[] = UBTS_FW_STR(UBTS_STEER_FW_SRC);
static const char kHwProfile[] = UBTS_FW_STR(UBTS_STEER_HW);
static const uint32_t kBaud = UBTS_STEER_BAUD;
static const uint8_t kSimBuild = UBTS_STEER_SIM ? 1 : 0;

// Pin plan A (spec 8.3, placeholder Q5). GPIO numbers; NodeMCU labels in the comments.
static const uint8_t kPinLeftFwd = 15;   // D8 left forward PWM
static const uint8_t kPinLeftRev = 14;   // D5 left reverse PWM
static const uint8_t kPinRightFwd = 0;   // D3 right forward PWM
static const uint8_t kPinRightRev = 16;  // D0 right reverse PWM
static const uint8_t kPinDriveEn = 5;    // D1 drive enable (4 EN lines tied, pull-downs), HIGH after setup
static const uint8_t kPinSteerEn = 4;    // D2 steering enable
static const uint8_t kPinSteerA = 12;    // D6 steering PWM A (+ side with mdir +1)
static const uint8_t kPinSteerB = 13;    // D7 steering PWM B
// D4 (GPIO2) is not used and not configured. A0 is the potentiometer wiper.

static const uint16_t kEepromBytes = 256;  // EEPROM.begin(256), record at offset 0 (spec 9.1)

}  // namespace ubts_fw

#endif  // UBTS_STEER_V3_FW_CONFIG_H
