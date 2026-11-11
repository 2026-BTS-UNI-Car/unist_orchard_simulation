// Text helpers (see fw_text.h).
#include "fw_text.h"

#include "fw_crc.h"

namespace ubts_fw {

namespace {
const char kHexDigits[] = "0123456789ABCDEF";
}  // namespace

void OutLine::ch(char c) {
  if (len < kOutLineCap) buf[len++] = c;
}

void OutLine::str(const char *s) {
  while (*s) ch(*s++);
}

void OutLine::u32(uint32_t v) {
  char tmp[10];
  uint8_t n = 0;
  do {
    tmp[n++] = static_cast<char>('0' + v % 10u);
    v /= 10u;
  } while (v != 0 && n < sizeof tmp);
  while (n > 0) ch(tmp[--n]);
}

void OutLine::i32(int32_t v) {
  if (v < 0) {
    ch('-');
    u32(static_cast<uint32_t>(-(v + 1)) + 1u);  // no overflow at INT32_MIN
  } else {
    u32(static_cast<uint32_t>(v));
  }
}

void OutLine::hex(uint32_t v, uint8_t digits) {
  for (int shift = 4 * (digits - 1); shift >= 0; shift -= 4) ch(kHexDigits[(v >> shift) & 0xFu]);
}

void OutLine::finish() {
  const uint8_t crc = crc8(reinterpret_cast<const uint8_t *>(buf), len);
  ch('*');
  hex(crc, 2);
  ch('\n');
}

int hex_digit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool parse_uint(const char *s, uint8_t n, uint32_t *out) {
  if (n == 0 || n > 9) return false;
  if (s[0] == '0' && n != 1) return false;  // no leading zero
  uint32_t v = 0;
  for (uint8_t i = 0; i < n; ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
    v = v * 10u + static_cast<uint32_t>(s[i] - '0');
  }
  *out = v;
  return true;
}

bool parse_int(const char *s, uint8_t n, int32_t *out) {
  if (n >= 1 && s[0] == '-') {
    uint32_t v = 0;
    if (n < 2 || s[1] == '0' || !parse_uint(s + 1, static_cast<uint8_t>(n - 1), &v)) return false;
    *out = -static_cast<int32_t>(v);
    return true;
  }
  uint32_t v = 0;
  if (!parse_uint(s, n, &v)) return false;
  *out = static_cast<int32_t>(v);
  return true;
}

bool parse_hex(const char *s, uint8_t n, uint8_t digits, uint32_t *out) {
  if (n != digits || digits == 0 || digits > 8) return false;
  uint32_t v = 0;
  for (uint8_t i = 0; i < n; ++i) {
    const int d = hex_digit(s[i]);
    if (d < 0) return false;
    v = (v << 4) | static_cast<uint32_t>(d);
  }
  *out = v;
  return true;
}

bool token_is(const char *s, uint8_t n, const char *word) {
  uint8_t i = 0;
  for (; i < n; ++i) {
    if (word[i] == 0 || word[i] != s[i]) return false;
  }
  return word[i] == 0;
}

}  // namespace ubts_fw
