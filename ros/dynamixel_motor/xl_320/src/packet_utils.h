#ifndef PACKET_UTILS_H
#define PACKET_UTILS_H

#include <stdint.h>

static const int MAX_PACKET_BYTES = 256;
static const int HEADER_SIZE = 3;
static const uint8_t HEADER[3] = {0xFF, 0xFF, 0xFD}; 
static const int PREFIX_SIZE = 8;
static const int CHECKSUM_SIZE = 2;
static const int BASE_PACKET_SIZE = 8 + 2; // Prefix + checksum

static const int ID_ADDR = 4;
static const int LENGTH_L_ADDR = 5;
static const int LENGTH_H_ADDR = 6;
static const int INST_ADDR = 7;


/*
 * Packet Construction utilities.
 *
 * Unlike the "high level commands" these are supposed to be general across Dynamixel SDK 2.0
 * That is, they are supposed to work on both XL-320 and DXL-PRO
 * Haven't test on a DXL-PRO though...
 */

extern inline uint8_t LowBits(uint16_t word) {
  return word & 0xFF;
}

extern inline uint8_t HighBits(uint16_t word) {
  return (word >> 8) & 0xFF;
}

extern inline uint16_t MakeWord(uint8_t low_bits, uint8_t high_bits) {
  return (high_bits << 8) + low_bits;
}

void SetPacketPrefix(uint8_t* data, uint8_t dxl_id, uint8_t instruction, uint16_t num_parameters);
void FinalPacketConsistencyCheck(uint8_t* data, uint16_t num_bytes);

// Adds the checksum. This should be the final piece of the packet, so this also
// returns the total number of bytes in the final packet.
void SetChecksum(uint8_t* data, uint16_t num_parameters);

void SetByteParam(uint8_t* data, uint16_t param_byte_offset, uint8_t value);
void SetWordParam(uint8_t* data, uint16_t param_byte_offset, uint16_t value);

uint8_t GetByteParam(uint8_t* data, uint16_t param_byte_offset);
uint16_t GetWordParam(uint8_t* data, uint16_t param_byte_offset);


void PrintPacket(uint8_t* data, uint16_t num_bytes); 
uint16_t MakePingPacket(uint8_t* data, uint8_t dxl_id); 

/*
  HIGHER LEVEL PACKET CONSTRUCTION
*/

uint16_t MakePingPacket(uint8_t* data, uint8_t dxl_id);
uint16_t MakeReadPacket(uint8_t* data, uint8_t dxl_id, uint16_t start_addr, uint16_t num_bytes);
uint16_t MakeReadBytePacket(uint8_t* data, uint8_t dxl_id, uint16_t addr);
uint16_t MakeReadWordPacket(uint8_t* data, uint8_t dxl_id, uint16_t start_addr);

uint16_t MakeWritePacket(uint8_t* data, uint8_t dxl_id, uint16_t start_addr, uint8_t* write_data, uint16_t num_write_bytes);
uint16_t MakeWriteBytePacket(uint8_t* data, uint8_t dxl_id, uint16_t addr, uint8_t write_data);
uint16_t MakeWriteWordPacket(uint8_t* data, uint8_t dxl_id, uint16_t start_addr, uint16_t write_data);

#endif // PACKET_UTILS_H
