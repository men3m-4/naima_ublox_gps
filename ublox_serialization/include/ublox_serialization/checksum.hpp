

#ifndef UBLOX_SERIALIZATION_CHECKSUM_HPP
#define UBLOX_SERIALIZATION_CHECKSUM_HPP

#include <cstdint>

namespace ublox {

/**
 * @brief calculate the checksum of a u-blox_message
 * @param data the start of the u-blox message
 * @param data the size of the u-blox message
 * @param ck_a the checksum a output
 * @param ck_b the checksum b output
 */
static inline void calculateChecksum(const uint8_t *data,
                                     uint32_t size,
                                     uint8_t &ck_a,
                                     uint8_t &ck_b) {
  ck_a = 0; ck_b = 0;
  for(uint32_t i = 0; i < size; ++i)
  {
    ck_a = ck_a + data[i];
    ck_b = ck_b + ck_a;
  }
}

/**
 * @brief calculate the checksum of a u-blox_message.
 * @param data the start of the u-blox message
 * @param data the size of the u-blox message
 * @param checksum the checksum output
 * @return the checksum
 */
static inline uint16_t calculateChecksum(const uint8_t *data,
                                         uint32_t size,
                                         uint16_t &checksum) {
  uint8_t *byte = reinterpret_cast<uint8_t *>(&checksum);
  calculateChecksum(data, size, byte[0], byte[1]);
  return checksum;
}

}  // namespace ublox

#endif  // UBLOX_SERIALIZATION_CHECKSUM_HPP
