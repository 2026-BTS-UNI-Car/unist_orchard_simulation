// CRC-8 and CRC-32 (see fw_crc.h).
#include "fw_crc.h"

namespace ubts_fw {

uint8_t crc8(const uint8_t *data, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; ++i) {
    crc = static_cast<uint8_t>(crc ^ data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80u) ? static_cast<uint8_t>((crc << 1) ^ 0x07u) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

uint32_t crc32(const uint8_t *data, size_t n) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
  }
  return crc ^ 0xFFFFFFFFu;
}

}  // namespace ubts_fw
