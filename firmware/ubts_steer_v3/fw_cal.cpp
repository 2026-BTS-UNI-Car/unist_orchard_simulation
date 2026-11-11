// Calibration and the EEPROM record (see fw_cal.h).
#include "fw_cal.h"

#include "fw_crc.h"

namespace ubts_fw {

namespace {

int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }

uint16_t get16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8)); }

void put16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFFu);
  p[1] = static_cast<uint8_t>(v >> 8);
}

uint32_t get32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

bool magic_is(const uint8_t *p, uint8_t last) { return p[0] == 'U' && p[1] == 'S' && p[2] == 'C' && p[3] == last; }

}  // namespace

void cal_clear(Cal *c) {
  c->raw_l = c->raw_c = c->raw_r = kMarkUnset;
  c->soft_l = c->soft_r = 0;
  c->pol_ok = 0;
  c->mdir = 1;
}

bool cal_equal(const Cal &a, const Cal &b) {
  return a.raw_l == b.raw_l && a.raw_c == b.raw_c && a.raw_r == b.raw_r && a.soft_l == b.soft_l &&
         a.soft_r == b.soft_r && a.pol_ok == b.pol_ok && a.mdir == b.mdir;
}

bool cal_any_set(const Cal &c) {
  Cal empty;
  cal_clear(&empty);
  return !cal_equal(c, empty);
}

bool marks_valid(const Cal &c) {
  if (c.raw_l == kMarkUnset || c.raw_c == kMarkUnset || c.raw_r == kMarkUnset) return false;
  if (c.raw_l > 1023 || c.raw_c > 1023 || c.raw_r > 1023) return false;
  const int32_t l = c.raw_l, m = c.raw_c, r = c.raw_r;
  const bool up = l > m && m > r;
  const bool down = l < m && m < r;
  if (!up && !down) return false;
  return iabs32(l - m) >= 50 && iabs32(m - r) >= 50;
}

bool cal_complete(const Cal &c) { return marks_valid(c) && c.soft_l >= 1 && c.soft_r <= -1; }

bool marks_in_window(const Cal &c, uint16_t lo, uint16_t hi) {
  return c.raw_l >= lo && c.raw_l <= hi && c.raw_c >= lo && c.raw_c <= hi && c.raw_r >= lo && c.raw_r <= hi;
}

int16_t raw_to_permille(const Cal &c, uint16_t f) {
  if (!marks_valid(c)) return 0;
  const int32_t dir = c.raw_l > c.raw_c ? 1 : -1;
  const int32_t d = (static_cast<int32_t>(f) - static_cast<int32_t>(c.raw_c)) * dir;
  int32_t p;
  if (d >= 0) {
    const int32_t span = iabs32(static_cast<int32_t>(c.raw_l) - static_cast<int32_t>(c.raw_c));
    p = (2000 * d + span) / (2 * span);
  } else {
    const int32_t span = iabs32(static_cast<int32_t>(c.raw_c) - static_cast<int32_t>(c.raw_r));
    p = -((2000 * (-d) + span) / (2 * span));
  }
  if (p > 9999) p = 9999;
  if (p < -9999) p = -9999;
  return static_cast<int16_t>(p);
}

uint32_t record_crc(const uint8_t *in) { return crc32(in, kRecordCrcOffset); }

void record_encode(uint8_t *out, bool tombstone, uint16_t cal_id, const Cal &c, const uint16_t *params) {
  out[0] = 'U';
  out[1] = 'S';
  out[2] = 'C';
  out[3] = tombstone ? 'X' : '3';
  out[4] = 1;
  out[5] = kRecordBytes;
  put16(out + 6, cal_id);
  put16(out + 8, c.raw_l);
  put16(out + 10, c.raw_c);
  put16(out + 12, c.raw_r);
  put16(out + 14, static_cast<uint16_t>(c.soft_l));
  put16(out + 16, static_cast<uint16_t>(c.soft_r));
  out[18] = c.pol_ok ? 1 : 0;
  out[19] = c.mdir < 0 ? 0xFF : 0x01;
  for (int i = 0; i < kParamCount; ++i) put16(out + 20 + 2 * i, params[i]);
  const uint32_t crc = record_crc(out);
  for (int i = 0; i < 4; ++i) out[kRecordCrcOffset + i] = static_cast<uint8_t>((crc >> (8 * i)) & 0xFFu);
}

bool record_decode(const uint8_t *in, uint16_t *cal_id, Cal *c, uint16_t *params, uint32_t *crc) {
  if (!magic_is(in, '3') || in[4] != 1 || in[5] != kRecordBytes) return false;
  const uint32_t stored = get32(in + kRecordCrcOffset);
  if (record_crc(in) != stored) return false;
  const uint16_t id = get16(in + 6);
  if (id < 1) return false;
  if (in[18] > 1 || (in[19] != 0x01 && in[19] != 0xFF)) return false;
  Cal cal;
  cal.raw_l = get16(in + 8);
  cal.raw_c = get16(in + 10);
  cal.raw_r = get16(in + 12);
  cal.soft_l = static_cast<int16_t>(get16(in + 14));
  cal.soft_r = static_cast<int16_t>(get16(in + 16));
  cal.pol_ok = in[18];
  cal.mdir = in[19] == 0xFF ? -1 : 1;
  uint16_t p[kParamCount];
  for (int i = 0; i < kParamCount; ++i) p[i] = get16(in + 20 + 2 * i);
  if (!marks_valid(cal) || cal.soft_l < 1 || cal.soft_l > 1000 || cal.soft_r < -1000 || cal.soft_r > -1) return false;
  if (!params_all_ok(p)) return false;
  if (!marks_in_window(cal, p[kPCwmin], p[kPCwmax])) return false;
  *cal_id = id;
  *c = cal;
  for (int i = 0; i < kParamCount; ++i) params[i] = p[i];
  *crc = stored;
  return true;
}

bool record_tombstone_id(const uint8_t *in, uint16_t *cal_id) {
  if (!magic_is(in, 'X')) return false;
  if (record_crc(in) != get32(in + kRecordCrcOffset)) return false;
  *cal_id = get16(in + 6);
  return true;
}

}  // namespace ubts_fw
