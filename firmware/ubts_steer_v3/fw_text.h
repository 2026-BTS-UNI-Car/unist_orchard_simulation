// Bounded text helpers: the MCU line builder (spec 5) and the strict number grammar of the host lines (spec 2.3).
#ifndef UBTS_STEER_V3_FW_TEXT_H
#define UBTS_STEER_V3_FW_TEXT_H

#include <stdint.h>

namespace ubts_fw {

// Longest MCU line is the banner, 174 B with the LF (spec 5.1).
static const uint8_t kOutLineCap = 184;

// One MCU line under construction. finish() appends "*HH\n" with the CRC-8 of the bytes so far
// (uppercase hex). Text past the capacity is cut (a programming error; lines are bounded by design).
struct OutLine {
  char buf[kOutLineCap];
  uint8_t len;

  void clear() { len = 0; }
  void ch(char c);
  void str(const char *s);
  void u32(uint32_t v);
  void i32(int32_t v);
  void hex(uint32_t v, uint8_t digits);  // uppercase, zero padded to `digits`
  void finish();
};

int hex_digit(char c);  // 0..15, or -1

// Unsigned grammar 0|[1-9][0-9]{0,8}; false on any other text.
bool parse_uint(const char *s, uint8_t n, uint32_t *out);
// Signed grammar: unsigned, or -[1-9][0-9]{0,8} ("-0" is refused).
bool parse_int(const char *s, uint8_t n, int32_t *out);
// Exactly `digits` hex digits, any case.
bool parse_hex(const char *s, uint8_t n, uint8_t digits, uint32_t *out);
// Token equals a NUL-terminated word.
bool token_is(const char *s, uint8_t n, const char *word);

}  // namespace ubts_fw

#endif  // UBTS_STEER_V3_FW_TEXT_H
