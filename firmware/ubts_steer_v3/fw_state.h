// RAM state of the steering firmware V3. One plain struct so that setup() can set all of it to the
// power-on value (a reset loses RAM; spec 6.1). No heap, fixed-size rings.
#ifndef UBTS_STEER_V3_FW_STATE_H
#define UBTS_STEER_V3_FW_STATE_H

#include <stdint.h>

#include "fw_cal.h"
#include "fw_params.h"

namespace ubts_fw {

enum McuState { kStBoot = 0, kStDisarmed = 1, kStArmed = 2, kStBench = 3 };
enum SteerMode { kModeOff = 0, kModeActive = 1, kModeHold = 2, kModeJog = 3, kModePol = 4 };
enum HoldPolicy { kHoldMeasured = 0, kHoldTarget = 1, kHoldCenter = 2 };
enum FrameError { kFrameOk = 0, kFrameLong = 1, kFrameCtrl = 2 };
// DRV_STUCK observation run (spec 6.7, decision 12.1-70): none, started by a BRAKE tick, drift under BRAKE seen.
enum StuckPhase { kStuckIdle = 0, kStuckBrake = 1, kStuckDrift = 2 };

// Fault mask bits (status line fl, spec 5.3).
static const uint16_t kFNocal = 1u << 0;
static const uint16_t kFPotOpen = 1u << 1;
static const uint16_t kFPotShort = 1u << 2;
static const uint16_t kFPotJump = 1u << 3;
static const uint16_t kFPotNoise = 1u << 4;
static const uint16_t kFPolarity = 1u << 5;
static const uint16_t kFDrvStuck = 1u << 6;
static const uint16_t kFSoftlimit = 1u << 7;
static const uint16_t kFStall = 1u << 8;
static const uint16_t kFTrack = 1u << 9;
static const uint16_t kFThermal = 1u << 10;
static const uint16_t kFTtl = 1u << 11;
static const uint16_t kFWatchdog = 1u << 12;
static const uint16_t kFCrc = 1u << 13;
static const uint16_t kFBoot = 1u << 14;
// Bits that a received ARM clears (the latches and BOOT; spec 6.8).
static const uint16_t kFArmClears = kFNocal | kFPotOpen | kFPotShort | kFPotJump | kFPotNoise | kFPolarity |
                                    kFDrvStuck | kFStall | kFTrack | kFThermal | kFWatchdog | kFCrc | kFBoot;
// Hardware latches: ARM is refused with pol while one is set; a passing CAL POL (or a reset) clears them
// (decision 12.1-71).
static const uint16_t kFNeedsPolCheck = kFPolarity | kFDrvStuck;

static const uint16_t kPRing = 1024;          // positions, power of two (windows up to 1000 ms)
static const uint16_t kFRing = 128;           // filtered and raw ADC values (jmpw, nsw <= 100)
static const uint32_t kThermalTicks = 60000;  // one bit per tick (thw <= 60 s)
static const uint16_t kTxRing = 512;          // MCU -> host bytes (spec 5.10)
static const uint8_t kLineMax = 63;           // host line content (spec 2.2)
static const uint8_t kMedMax = 9;

struct DriveSide {
  int16_t d;         // applied duty, signed
  int16_t g;         // target duty, signed
  uint16_t rem;      // ramp remainder 0..999
  uint32_t ramp_at;  // ms of the last 10 ms ramp grid point
  uint32_t zero_at;  // ms when the duty last became 0
};

struct FwState {
  // clock
  uint32_t now;           // millis() of this tick (loop step 1)
  uint32_t boot_ms;       // millis() at setup()
  uint32_t last_ms;       // millis() of the last processed tick
  uint32_t tick;          // processed ticks since setup (0 = the boot tick); wraps at 2^32
  uint32_t ticks_seen;    // processed ticks whose step 11 is done, saturates at 2^32 - 1 (window guards)
  uint8_t ticked;         // at least one tick processed since setup
  // identity
  uint32_t boot_id;
  uint32_t chip_id;
  uint32_t rst_reason;
  // state machine and faults
  uint8_t state;          // McuState
  uint16_t latched;       // latched fault bits and BOOT
  uint8_t ttl_bit;        // TTL condition bit
  uint8_t softlimit_bit;  // SOFTLIMIT condition bit
  // link
  uint32_t last_input;    // ms of the last watchdog refresh
  uint16_t last_q;        // q of the last accepted C (0 after boot)
  uint8_t c_valid;        // the last C is still valid (TTL running)
  uint32_t c_ms;
  uint16_t c_ttl;
  uint16_t ttl_count;     // TTL expiries since boot (u16, wraps)
  uint8_t bad_run;        // bad lines in a row
  // drive
  DriveSide side[2];      // 0 left, 1 right
  uint8_t drive_ever;     // a nonzero duty happened since boot
  uint32_t last_drive_nonzero;
  // steering
  uint8_t mode;           // SteerMode
  int32_t tgt, cmd;       // servo target and rate-limited internal target, permille
  int32_t rrem;           // rate-limit remainder
  int32_t in_db;          // ms inside the deadband
  uint8_t braked;         // deadband brake hold
  int8_t last_sign;       // sign of the last nonzero servo output
  uint32_t last_nz_ms;    // ms of the last nonzero servo output
  int16_t u;              // position-space servo output -255..255 (0 outside ACTIVE/HOLD)
  uint8_t servo_live;     // ACTIVE/HOLD pins follow u (after the first servo update)
  uint32_t hold_t0, hold_h;
  int16_t jog_pwm;
  uint16_t jog_ms;
  uint32_t jog_t0;
  uint32_t pol_t0;
  uint16_t pol_f0, pol_f500;
  uint8_t pol_have500;
  uint32_t next_servo, last_servo;
  // status line clock
  uint32_t next_status;
  uint16_t status_n;
  uint16_t drop;          // MCU lines dropped because the TX ring was full (saturates)
  // ADC
  uint16_t med_buf[kMedMax];
  uint8_t med_pos;
  uint8_t med_refill;     // fill the median buffer with the next sample
  uint16_t raw, f;
  int16_t pos;            // P[n], permille
  // history (index = tick modulo the ring size)
  int16_t p_ring[kPRing];
  uint16_t f_ring[kFRing];
  uint16_t raw_ring[kFRing];
  uint8_t thermal_bits[kThermalTicks / 8];
  uint32_t thermal_idx;   // bit index of the current tick, 0 .. kThermalTicks - 1 (own wrap; 2^32 % 60000 != 0)
  uint32_t thermal_cnt;   // set bits among the last thermal_window ticks
  uint32_t thermal_window;
  uint16_t run_pol, run_stall, run_stuck, run_track;
  int8_t run_pol_sign;
  uint8_t stuck_phase;    // StuckPhase
  int16_t stuck_ref;      // P at the BRAKE tick that started the DRV_STUCK run
  // calibration and parameters: working copy W and stored record R
  Cal w_cal, r_cal;
  uint16_t w_par[kParamCount], r_par[kParamCount];
  uint8_t r_valid;
  uint16_t r_cal_id;
  uint32_t r_crc;
  uint8_t commits;        // EEPROM commits since boot
  // pins: values chosen this tick and values last written
  uint8_t pin_lf, pin_lr, pin_rf, pin_rr, pin_sa, pin_sb, pin_sen;
  uint8_t out_lf, out_lr, out_rf, out_rr, out_sa, out_sb, out_sen;
  uint8_t steer_en_prev;  // steering EN pin of the previous tick
  // host line reader
  char line[kLineMax + 1];
  uint8_t line_len;
  uint8_t line_err;       // FrameError
  uint8_t line_cr;        // CR seen, the next byte must be LF
  // TX ring
  uint8_t tx[kTxRing];
  uint16_t tx_head, tx_len;
};

extern FwState fw;

// Power-on RAM: every field 0.
void state_reset();

inline int32_t sgn32(int32_t v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }
inline int32_t abs32(int32_t v) { return v < 0 ? -v : v; }
inline uint16_t sat_inc16(uint16_t v) { return v == 0xFFFFu ? v : static_cast<uint16_t>(v + 1u); }

}  // namespace ubts_fw

#endif  // UBTS_STEER_V3_FW_STATE_H
