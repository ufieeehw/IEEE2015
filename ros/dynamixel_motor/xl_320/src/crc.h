#ifndef CRC_H
#define CRC_H

#include <stdint.h>

extern const uint16_t CRC_TABLE[256];

extern inline uint16_t UpdateCRC(uint16_t crc_accum, uint8_t* data_blk_ptr, uint16_t data_blk_size) {
  uint16_t i, j;

  for(j = 0; j < data_blk_size; j++) {
    i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ CRC_TABLE[i];
  }

  return crc_accum;
}

#endif // CRC_H
