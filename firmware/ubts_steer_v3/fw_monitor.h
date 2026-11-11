// Potentiometer input and fault monitors: ADC median filter and position (loop step 2), history rings,
// fault detectors (loop step 6), run counters and the thermal bit ring (loop step 11). Spec 6.7, 7.2.
#ifndef UBTS_STEER_V3_FW_MONITOR_H
#define UBTS_STEER_V3_FW_MONITOR_H

#include <stdint.h>

namespace ubts_fw {

void monitor_boot();          // the first ADC sample fills the median buffer; empty history
void monitor_adc();           // loop step 2
void monitor_refill_median(); // CFG med: the buffer is filled with this tick's raw
void monitor_thermal_window_changed();  // CFG thw: recount the thermal window
uint16_t monitor_faults();    // loop step 6: run_track, SOFTLIMIT bit; returns the tripped latched bits
                              // (not yet latched ones); the arguments are kept for monitor_fault_arg()
int32_t monitor_fault_arg(uint16_t bit);
bool monitor_thermal_active();  // the THERMAL condition (window count > tha)
// Active conditions that refuse ARM, CAL JOG and CAL POL with fault (f outside pvmin..pvmax, the THERMAL
// condition; spec 6.7, decision 12.1-68).
bool monitor_arm_blocked();
void monitor_end_of_tick();   // loop step 11: run counters and the thermal ring

}  // namespace ubts_fw

#endif  // UBTS_STEER_V3_FW_MONITOR_H
