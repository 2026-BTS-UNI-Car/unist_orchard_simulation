// Little-endian field access for the lockstep frames (docs/steering/LOCKSTEP_PROTOCOL_KO.md).
// Values are moved byte by byte; nothing depends on struct layout or host byte order
// (doubles are IEEE-754 binary64 on every supported target).
#ifndef UBTS_STEER_WIRE_H
#define UBTS_STEER_WIRE_H

#include <stdint.h>
#include <string.h>

namespace ubts_steer {

const uint16_t kWireVersion = 1;
const uint16_t kFrameStepRequest = 0x0001;
const uint16_t kFrameResetRequest = 0x0002;
const uint16_t kFrameReady = 0x8000;
const uint16_t kFrameStepResponse = 0x8001;
const uint16_t kFrameResetResponse = 0x8002;
const uint32_t kHeaderBytes = 16;
const uint32_t kStepRequestBytes = 48;
const uint32_t kResetRequestBytes = 8;
const uint32_t kResponseBytes = 80;
const uint32_t kReadyBytes = 8;
const uint32_t kMaxSerialBytes = 4096;
const uint32_t kMaxBuildIdBytes = 256;
const size_t kMaxStartupLineBytes = 65536;  // including the final '\n'

// Protocol v2 (section 11): header version 2; firmware mode frames.
const uint16_t kWireVersionV2 = 2;
const uint16_t kFrameFwStepRequest = 0x0003;
const uint16_t kFrameFwStepResponse = 0x8003;
const uint32_t kFwStepRequestBytes = 32;   // + 8 per event + 4 per rx byte
const uint32_t kFwStepResponseBytes = 96;  // + 8 per change + 12 per tx byte
const uint32_t kFwEventBytes = 8;
const uint32_t kFwRxBytes = 4;
const uint32_t kFwChangeBytes = 8;
const uint32_t kFwTxBytes = 12;
const uint32_t kMaxFwEvents = 64;
const uint32_t kMaxFwRx = 4096;
const uint32_t kMaxFwChanges = 4096;
const uint32_t kMaxFwTx = 4096;
const double kMaxFwDtS = 0.1;

inline uint16_t get_u16(const unsigned char *p) {
  return static_cast<uint16_t>(p[0] | (static_cast<unsigned>(p[1]) << 8));
}

inline uint32_t get_u32(const unsigned char *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

inline double get_f64(const unsigned char *p) {
  const uint64_t u = static_cast<uint64_t>(get_u32(p)) | (static_cast<uint64_t>(get_u32(p + 4)) << 32);
  double d;
  memcpy(&d, &u, sizeof d);
  return d;
}

inline void put_u16(unsigned char *p, uint16_t v) {
  p[0] = static_cast<unsigned char>(v & 0xffu);
  p[1] = static_cast<unsigned char>((v >> 8) & 0xffu);
}

inline void put_u32(unsigned char *p, uint32_t v) {
  for (int i = 0; i < 4; ++i) p[i] = static_cast<unsigned char>((v >> (8 * i)) & 0xffu);
}

inline void put_f64(unsigned char *p, double d) {
  uint64_t u;
  memcpy(&u, &d, sizeof u);
  put_u32(p, static_cast<uint32_t>(u & 0xffffffffULL));
  put_u32(p + 4, static_cast<uint32_t>(u >> 32));
}

}  // namespace ubts_steer

#endif  // UBTS_STEER_WIRE_H
