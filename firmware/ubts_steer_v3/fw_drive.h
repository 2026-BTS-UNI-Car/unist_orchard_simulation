// Drive outputs: V2 ramp 255/s on a 10 ms grid, 120 ms zero dwell, brake and release (spec 8.2), BENCH pin
// requests (spec 4.12) and the "moving" flag (spec 7.5).
#ifndef UBTS_STEER_V3_FW_DRIVE_H
#define UBTS_STEER_V3_FW_DRIVE_H

#include <stdint.h>

namespace ubts_fw {

void drive_boot(uint32_t now);      // both sides 0, ramp grid and zero dwell start at boot
void drive_ramp(uint32_t now);      // loop step 8 (both sides, then the moving bookkeeping)
void drive_brake(uint32_t now);     // V2 stopDirections: duty and target 0 in this tick
void drive_release();               // targets 0, the ramp brings the duty down
void drive_set_targets(int16_t left, int16_t right);
bool drive_busy();                  // an applied duty is not 0
bool drive_moving(uint32_t now);    // nonzero duty now or within movms

// BENCH D<n> (n = 8, 5, 3, 0): V2 requestDuty on the signed target of that side.
void drive_bench_request(uint8_t pin_digit, uint8_t value);
// Magnitudes of that pin after a request (reply P D<n> <duty> <target>).
uint8_t drive_pin_duty(uint8_t pin_digit);
uint8_t drive_pin_target(uint8_t pin_digit);

// Direction pin values from the applied duties (left fwd, left rev, right fwd, right rev).
void drive_pin_values(uint8_t *lf, uint8_t *lr, uint8_t *rf, uint8_t *rr);

}  // namespace ubts_fw

#endif  // UBTS_STEER_V3_FW_DRIVE_H
