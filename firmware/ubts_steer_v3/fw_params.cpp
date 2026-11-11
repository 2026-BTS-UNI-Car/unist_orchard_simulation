// CFG parameter table (see fw_params.h).
#include "fw_params.h"

#include "fw_text.h"

namespace ubts_fw {

// name, default, low, high, value set. Defaults: placeholder profile, spec 4.10 conversion, s = 40 permille/deg.
const ParamInfo kParams[kParamCount] = {
    {"kp", 245, 1, 4096, kSetRange},       // round(15 %/deg / 100 * 255 / 40 * 256)
    {"db", 20, 0, 200, kSetRange},         // 0.5 deg
    {"minp", 102, 0, 255, kSetRange},      // 40 %
    {"maxp", 255, 1, 255, kSetRange},      // 100 %
    {"dwell", 200, 0, 2000, kSetRange},    // 0.2 s
    {"rev", 20, 0, 500, kSetRange},        // 20 ms
    {"rmov", 960, 1, 5000, kSetRange},     // 0.8 * 30 deg/s
    {"rstd", 480, 1, 5000, kSetRange},     // 0.8 * 15 deg/s
    {"ttlmax", 300, 50, 1000, kSetRange},  // c_ttl_max_ms
    {"wd", 500, 100, 5000, kSetRange},     // watchdog_ms
    {"ttlmode", 0, 0, 1, kSetRange},       // brake
    {"wdmode", 0, 0, 2, kSetRange},        // hold
    {"hloss", 2000, 0, 10000, kSetRange},  // 2 s
    {"hstop", 2000, 0, 10000, kSetRange},  // 2 s
    {"hdis", 2000, 0, 10000, kSetRange},   // 2 s
    {"stoppol", 0, 0, 2, kSetRange},       // hold_measured
    {"sthz", 20, 10, 50, kSetSthz},        // status_hz
    {"med", 5, 1, 9, kSetMed},             // pot_filter.median_of
    {"pvmin", 20, 0, 1023, kSetRange},     // pot_valid_raw
    {"pvmax", 1003, 0, 1023, kSetRange},
    {"cwmin", 60, 0, 1023, kSetRange},     // cal_end_raw_window
    {"cwmax", 963, 0, 1023, kSetRange},
    {"jmp", 8, 1, 1023, kSetRange},        // pot_jump_counts
    {"jmpw", 2, 1, 100, kSetRange},
    {"nsd", 50, 1, 1000, kSetRange},       // pot_noise_counts sd 5.0
    {"nsw", 50, 2, 100, kSetRange},
    {"polp", 40, 1, 1000, kSetRange},      // polarity_fault 1 deg
    {"polt", 100, 1, 1000, kSetRange},     // 0.1 s
    {"stp", 153, 0, 255, kSetRange},       // stall_detect 60 %
    {"stt", 500, 1, 1000, kSetRange},      // 0.5 s
    {"stm", 20, 0, 1000, kSetRange},       // 0.5 deg
    {"trk", 120, 1, 2000, kSetRange},      // track_fault 3 deg
    {"trt", 1000, 1, 10000, kSetRange},    // 1.0 s
    {"dsm", 80, 1, 1000, kSetRange},       // driver_stuck_fault 2 deg
    {"dst", 300, 1, 1000, kSetRange},      // 0.3 s
    {"thp", 204, 0, 255, kSetRange},       // thermal_proxy 80 %
    {"thw", 30, 1, 60, kSetRange},         // 30 s
    {"tha", 5000, 1, 60000, kSetRange},    // 5 s
    {"crcn", 5, 1, 50, kSetRange},         // crc_burst_n
    {"movms", 300, 0, 2000, kSetRange},    // design 4.4 "last 0.3 s"
    {"svhz", 500, 500, 1000, kSetSvhz},    // servo_hz
};

void params_defaults(uint16_t *p) {
  for (int i = 0; i < kParamCount; ++i) p[i] = kParams[i].def;
}

int param_find(const char *s, uint8_t n) {
  for (int i = 0; i < kParamCount; ++i) {
    if (token_is(s, n, kParams[i].name)) return i;
  }
  return -1;
}

bool param_value_ok(int id, uint32_t v) {
  if (id < 0 || id >= kParamCount) return false;
  const ParamInfo &info = kParams[id];
  switch (info.set) {
    case kSetSthz: return v == 10 || v == 20 || v == 25 || v == 40 || v == 50;
    case kSetMed: return v == 1 || v == 3 || v == 5 || v == 7 || v == 9;
    case kSetSvhz: return v == 500 || v == 1000;
    default: return v >= info.lo && v <= info.hi;
  }
}

bool params_cross_ok(const uint16_t *p) {
  return p[kPMinp] <= p[kPMaxp] && p[kPRstd] <= p[kPRmov] && p[kPPvmin] < p[kPPvmax] && p[kPCwmin] < p[kPCwmax] &&
         static_cast<uint32_t>(p[kPTha]) <= static_cast<uint32_t>(p[kPThw]) * 1000u;
}

bool params_all_ok(const uint16_t *p) {
  for (int i = 0; i < kParamCount; ++i) {
    if (!param_value_ok(i, p[i])) return false;
  }
  return params_cross_ok(p);
}

bool params_equal(const uint16_t *a, const uint16_t *b) {
  for (int i = 0; i < kParamCount; ++i) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

}  // namespace ubts_fw
