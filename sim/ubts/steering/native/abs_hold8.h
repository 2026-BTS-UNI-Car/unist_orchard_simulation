// SIM model of the steering law of the real board firmware ABS-HOLD-8-REBOUND-LIMITS (ESP32-S3,
// BTS7960_Serial_Control.ino, received 2026-11-16) and of the SIM host's ANGLE policy
// (docs/steering/ABS_HOLD8_SIM_KO.md).
//
// Firmware mode only: the startup block "steer_law" (kind abs_hold_8; profile schema v4, steer_fw_mode abs_hold_8)
// installs abs_hold8_law() as the steering-law hook of the firmware V3 (fw_steer.h steer_law_hook). The V3
// firmware keeps the link, arming, TTL, watchdog, STOP, drive ramp and fault monitors; only its servo law is replaced.
// Without the block the hook stays null and the session is the V3 session (bit-identical).
//
// Real firmware rules kept (counts of the board's 12-bit ADC, ms):
//   ANGLE a (command degrees, + right): target = CENTER + lroundf(span * |a| / 20), span = LEFT - CENTER (left) or
//     RIGHT - CENTER (right). The same target while holding is ignored; a new one stops the motor, resets the drive
//     budget and the no-progress count and waits settle_ms before the first decision.
//   Decisions on quiet samples: pot = median of 5 samples every 5 ms, confirmed when two medians agree within 20
//     (their mean); no confirmation for more than 100 ms is a fault (hold cancelled).
//   After the settle: |error| <= 10 holds (budget reset); |error| <= 16 with no correction running is ignored; else a
//     pulse of PWM 100 for 80 ms (|error| > 30) or 50 ms, cut at the guard 15 counts beyond an operating limit; then
//     the driver is released (EN low) for 300 ms. The first 100 ms of the release count toward the rebound peak.
//     Outside the target tolerance: rebound >= 20 cancels; 3 pulses in a row with < 3 counts retained progress cancel;
//     1000 ms of drive in one correction cancels; a pulse that would start at or beyond an operating limit cancels.
// Simplifications: the decision loop runs on the V3 servo grid (500 Hz placeholder) instead of every loop pass; the
//   12-bit value is made from the V3 10-bit model ADC through the V3 calibration marks (+1000 permille = the left
//   operating reference, -1000 = the right one), so the SIM pot noise and resolution are those of the 10-bit model.
//
// SIM host policy (the Jetson bridge for the new board is not written yet): every poll_ms the host compares the
// executor's goal, as command degrees a = -goal_permille * command_max / 1000 rounded to step_deg, with the target it
// last sent. It sends a new ANGLE when they differ by >= deadband_deg and either min_interval_ms has passed since the
// last ANGLE or the firmware is idle (target reached or hold cancelled). After a cancel it sends the same ANGLE again
// when the error is still above the restart tolerance.
#ifndef UBTS_STEER_ABS_HOLD8_H
#define UBTS_STEER_ABS_HOLD8_H

#include <stdint.h>

#include "steer_config.h"

namespace ubts_fw {
struct Cal;
}

namespace ubts_steer {

// Install (cfg.law.present) or remove the hook; resets the law state.
void abs_hold8_install(const LawConfig &law);
void abs_hold8_remove();

// The hook (fw_steer.h SteerLawHook signature).
int16_t abs_hold8_law(uint32_t now, int32_t goal, uint16_t raw, bool fresh, const ubts_fw::Cal &cal,
                      int32_t *cmd_permille, uint8_t *release);

}  // namespace ubts_steer

#endif  // UBTS_STEER_ABS_HOLD8_H
