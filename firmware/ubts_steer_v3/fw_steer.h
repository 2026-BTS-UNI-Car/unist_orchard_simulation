// Steering: modes OFF / ACTIVE / HOLD / JOG / POL, the integer servo law with rate limit, deadband brake hold and
// reversal pause (spec 7.4 .. 7.10), and the steering pin values (spec 7.7).
#ifndef UBTS_STEER_V3_FW_STEER_H
#define UBTS_STEER_V3_FW_STEER_H

#include <stdint.h>

#include "fw_cal.h"

namespace ubts_fw {

// Optional steering-law hook (2026-11-16, docs/steering/ABS_HOLD8_SIM_KO.md). Null by default: the V3 servo law runs.
// The native SIM session sets it to emulate another board firmware's steering law (ABS-HOLD-8 of the ESP32-S3 board);
// the firmware itself never sets it, so on the device the logic is always the V3 law. When set, servo_update() calls
// it in place of the servo law with: millis() of the servo tick, the goal in permille (tgt clamped to the soft
// limits), the raw ADC of this tick, fresh = first servo update of an ACTIVE / HOLD period, the working calibration.
// It returns the output -255..255 (the sign of fw.u: + pushes toward + permille), sets *cmd_permille (the target the
// emulated firmware holds; reported as cmd) and *release (1: the driver de-energised, EN low, instead of the V3
// deadband BRAKE).
typedef int16_t (*SteerLawHook)(uint32_t now, int32_t goal, uint16_t raw, bool fresh, const Cal &cal,
                                int32_t *cmd_permille, uint8_t *release);
extern SteerLawHook steer_law_hook;

void steer_boot(uint32_t now);     // servo clock: next_servo = b, last_servo = b - P_sv
void steer_off();                  // mode OFF, output 0 at once
void steer_arm();                  // ACTIVE with tgt = cmd = pos (also from HOLD)
void steer_hold(uint8_t policy, uint32_t hold_ms);  // HOLD (from ACTIVE) with a HoldPolicy target
void steer_jog(int16_t pwm, uint16_t ms);
void steer_pol();
bool steer_busy();                 // HOLD, JOG or POL
bool steer_servo_mode();           // ACTIVE or HOLD

void steer_mode_clock();           // loop step 7: HOLD end, JOG end, POL phases and result
void steer_output();               // loop step 9: servo clock and law, steering pin values

}  // namespace ubts_fw

#endif  // UBTS_STEER_V3_FW_STEER_H
