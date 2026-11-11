// Calibration fields, raw -> permille mapping (spec 7.3) and the EEPROM record (spec 9.2 .. 9.4).
#ifndef UBTS_STEER_V3_FW_CAL_H
#define UBTS_STEER_V3_FW_CAL_H

#include <stdint.h>

#include "fw_params.h"

namespace ubts_fw {

static const uint16_t kMarkUnset = 0xFFFFu;
static const uint8_t kRecordBytes = 106;
static const uint8_t kRecordCrcOffset = 102;

// Calibration fields of the working copy W or the stored record R.
struct Cal {
  uint16_t raw_l, raw_c, raw_r;  // marks, kMarkUnset when not set
  int16_t soft_l, soft_r;        // soft limits in permille, 0 and 0 when not set
  uint8_t pol_ok;                // polarity check passed
  int8_t mdir;                   // +1: D6 pushes the position toward +, -1: reversed
};

void cal_clear(Cal *c);                 // marks unset, soft 0,0, pol 0, mdir +1
bool cal_equal(const Cal &a, const Cal &b);
bool cal_any_set(const Cal &c);         // any field differs from cal_clear()
bool marks_valid(const Cal &c);         // 3 marks, strictly monotonic, both spans >= 50
bool cal_complete(const Cal &c);        // marks valid, soft_l >= 1, soft_r <= -1
bool marks_in_window(const Cal &c, uint16_t lo, uint16_t hi);

// Filtered raw -> position in permille (+1000 left mark, 0 center, -1000 right mark), clamped to +-9999;
// 0 when the marks are not valid.
int16_t raw_to_permille(const Cal &c, uint16_t f);

// 106-byte record: magic, version 1, length 106, cal_id, marks, soft, pol, mdir, 41 params, CRC-32 (LE).
void record_encode(uint8_t *out, bool tombstone, uint16_t cal_id, const Cal &c, const uint16_t *params);
// Full load check of spec 9.4; on success fills the outputs.
bool record_decode(const uint8_t *in, uint16_t *cal_id, Cal *c, uint16_t *params, uint32_t *crc);
// "USCX" tombstone with a matching CRC-32: its cal_id.
bool record_tombstone_id(const uint8_t *in, uint16_t *cal_id);
uint32_t record_crc(const uint8_t *in);  // CRC-32 over bytes 0..101

}  // namespace ubts_fw

#endif  // UBTS_STEER_V3_FW_CAL_H
