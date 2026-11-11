// CRC-8 of the serial lines (spec 3) and CRC-32 of the EEPROM record (spec 9.3). Bitwise, no tables (RAM).
#ifndef UBTS_STEER_V3_FW_CRC_H
#define UBTS_STEER_V3_FW_CRC_H

#include <stddef.h>
#include <stdint.h>

namespace ubts_fw {

// Polynomial 0x07, init 0x00, no reflection, xorout 0x00 (check "123456789" -> 0xF4).
uint8_t crc8(const uint8_t *data, size_t n);

// CRC-32/ISO-HDLC: reflected 0xEDB88320, init and xorout 0xFFFFFFFF (check "123456789" -> 0xCBF43926).
uint32_t crc32(const uint8_t *data, size_t n);

}  // namespace ubts_fw

#endif  // UBTS_STEER_V3_FW_CRC_H
