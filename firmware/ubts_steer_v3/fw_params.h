// The 41 CFG parameters (spec 4.10). Order = EEPROM record order (spec 9.2).
//
// Compiled defaults come from the placeholder vehicle profile front_steer_v0_placeholder (D and D2 tables)
// through the spec 4.10 conversion with the placeholder scale s = 40 permille/deg (1 deg ~ 40 permille).
// All values are placeholders.
#ifndef UBTS_STEER_V3_FW_PARAMS_H
#define UBTS_STEER_V3_FW_PARAMS_H

#include <stdint.h>

namespace ubts_fw {

enum ParamId {
  kPKp = 0,   // PWM per permille x 256
  kPDb,       // servo deadband, permille
  kPMinp,     // servo minimum PWM outside the deadband
  kPMaxp,     // servo maximum PWM
  kPDwell,    // deadband dwell before the brake hold, ms
  kPRev,      // servo reversal pause, ms
  kPRmov,     // command rate limit while moving, permille/s
  kPRstd,     // command rate limit at standstill, permille/s
  kPTtlmax,   // largest C ttl, ms
  kPWd,       // link watchdog, ms
  kPTtlmode,  // 0 brake, 1 release at C TTL expiry
  kPWdmode,   // 0 hold, 1 center, 2 free steering after the watchdog
  kPHloss,    // steering hold after link loss (watchdog, bad-line burst), ms
  kPHstop,    // steering hold after STOP, ms
  kPHdis,     // steering hold after DISARM, ms
  kPStoppol,  // 0 hold_measured, 1 hold_target, 2 center after STOP
  kPSthz,     // status line rate, Hz (10, 20, 25, 40, 50)
  kPMed,      // median filter length (1, 3, 5, 7, 9)
  kPPvmin,    // valid filtered raw, low end
  kPPvmax,    // valid filtered raw, high end
  kPCwmin,    // calibration mark window, low end
  kPCwmax,    // calibration mark window, high end
  kPJmp,      // pot jump counts
  kPJmpw,     // pot jump window, ms
  kPNsd,      // pot noise sd x 10, counts
  kPNsw,      // pot noise window, samples (= ms)
  kPPolp,     // polarity fault: motion away from the output, permille
  kPPolt,     // polarity fault window, ms
  kPStp,      // stall: PWM level
  kPStt,      // stall window, ms
  kPStm,      // stall: smallest motion, permille
  kPTrk,      // track fault error, permille
  kPTrt,      // track fault duration, ms
  kPDsm,      // driver stuck: motion, permille
  kPDst,      // driver stuck window, ms
  kPThp,      // thermal proxy PWM level
  kPThw,      // thermal window, s
  kPTha,      // thermal accumulated time, ms
  kPCrcn,     // bad-line burst length
  kPMovms,    // "moving" holds this long after the last nonzero drive duty, ms
  kPSvhz,     // servo rate, Hz (500, 1000)
  kParamCount
};

enum ParamSet { kSetRange = 0, kSetSthz = 1, kSetMed = 2, kSetSvhz = 3 };

struct ParamInfo {
  const char *name;
  uint16_t def;
  uint16_t lo;
  uint16_t hi;
  uint8_t set;  // ParamSet
};

extern const ParamInfo kParams[kParamCount];

void params_defaults(uint16_t *p);
// Index of a key (exact text), or -1.
int param_find(const char *s, uint8_t n);
// The value is allowed for this key on its own (range or value set).
bool param_value_ok(int id, uint32_t v);
// The bound pairs hold: minp <= maxp, rstd <= rmov, pvmin < pvmax, cwmin < cwmax, tha <= thw * 1000.
bool params_cross_ok(const uint16_t *p);
// Every value allowed and the bound pairs hold.
bool params_all_ok(const uint16_t *p);
bool params_equal(const uint16_t *a, const uint16_t *b);

}  // namespace ubts_fw

#endif  // UBTS_STEER_V3_FW_PARAMS_H
