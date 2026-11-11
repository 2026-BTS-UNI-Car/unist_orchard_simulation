// Steering: modes OFF / ACTIVE / HOLD / JOG / POL, the integer servo law with rate limit, deadband brake hold and
// reversal pause (spec 7.4 .. 7.10), and the steering pin values (spec 7.7).
#ifndef UBTS_STEER_V3_FW_STEER_H
#define UBTS_STEER_V3_FW_STEER_H

#include <stdint.h>

namespace ubts_fw {

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
