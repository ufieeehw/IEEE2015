#include <stdint.h>
#include <assert.h>
#include <stdio.h>

#include "packet_utils.h"
#include "crc.h"

/*
 * Packet Construction utilities.
 *
 * Unlike the "high level commands" these are supposed to be general across Dynamixel SDK 2.0
 * That is, they are supposed to work on both XL-320 and DXL-PRO
 * Haven't test on a DXL-PRO though...
 */
inline uint8_t LowBits(uint16_t word);

inline uint8_t HighBits(uint16_t word);

inline uint16_t MakeWord(uint8_t low_bits, uint8_t high_bits);

void SetPacketPrefix(uint8_t* data,
                     uint8_t dxl_id,
                     uint8_t instruction,
                     uint16_t num_parameters) {
  // Header
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFD;

  // Reserved
  // At the time of this writing, this register is inconsequential.
  // data[3] = 0x00;

  // The dynamixel ID to target. 0xFE is broadcast
  data[4] = dxl_id;

  // Packet Length
  // == Length after packet length field
  // == Number of parameters + 3
  int packet_length = num_parameters + 3;

  assert(packet_length <= UINT16_MAX);       // No 16bit overflow.
  data[5] = LowBits(packet_length);         // Low order bits of packet length.
  data[6] = HighBits(packet_length);  // High order bits of packet length.

  // Instruction byte.
  data[7] = instruction;
}

/*
 * Adds the CRC checksum. The entire data packet up until the 
 * CRC checksum must already be set, because the value is a function
 * of all the preceding bytes.
 * Returns the total number of bytes in the packet.
 */
void SetChecksum(uint8_t* data, uint16_t num_parameters) {
  // The prefix size is 8 bytes. Check overflow.
  assert(num_parameters <= MAX_PACKET_BYTES - PREFIX_SIZE);
  uint16_t num_bytes_before_checksum = num_parameters + PREFIX_SIZE;
  uint16_t crc = UpdateCRC(0, data, num_bytes_before_checksum);
  data[num_bytes_before_checksum] = LowBits(crc);
  data[num_bytes_before_checksum + 1] = HighBits(crc);
}


void SetByteParam(uint8_t* data, uint16_t param_byte_offset, uint8_t value) {
  data[PREFIX_SIZE + param_byte_offset] = value;
}

void SetWordParam(uint8_t* data, uint16_t param_byte_offset, uint16_t value) {
  data[PREFIX_SIZE + param_byte_offset] = LowBits(value);
  data[PREFIX_SIZE + param_byte_offset + 1] = HighBits(value);
}

uint8_t GetByteParam(uint8_t* data, uint16_t param_byte_offset) {
  return data[PREFIX_SIZE + param_byte_offset];
}

uint16_t GetWordParam(uint8_t* data, uint16_t param_byte_offset) {
  return MakeWord(data[PREFIX_SIZE + param_byte_offset],
                  data[PREFIX_SIZE + param_byte_offset + 1]);
}

void PrintPacket(uint8_t* data, uint16_t num_bytes) {
  int i;
  if (num_bytes >= PREFIX_SIZE) {
    printf("Prefix: ");
    for (i = 0; i < PREFIX_SIZE; i++) {
      printf("%02x | ", data[i]);
    }

    printf("Payload: ");
    for (; i < num_bytes; i++) {
      printf("%02x | ", data[i]);
    }
    printf("\n");
  } else {
    printf("Prefix not complete: ");
    for (i = 0; i < num_bytes; i++) {
      printf("%02x | ", data[i]);
    }
  }
}

void FinalPacketConsistencyCheck(uint8_t* data, uint16_t num_bytes) {
  assert(num_bytes >= BASE_PACKET_SIZE);
  // Check header
  int i;
  for (i = 0; i < HEADER_SIZE; i++) {
    assert(data[i] == HEADER[i]);
  }
  // Check packet length setting
  assert(MakeWord(data[LENGTH_L_ADDR], data[LENGTH_H_ADDR]) == num_bytes - PREFIX_SIZE + 1);

  // Check Instruction
  assert(data[INST_ADDR] > 0 && data[INST_ADDR] <= 11);

  // CRC Check
  uint16_t expected_crc = UpdateCRC(0, data, num_bytes - CHECKSUM_SIZE);
  assert(MakeWord(data[num_bytes - 2], data[num_bytes - 1]) == expected_crc);
  
}


// Higher Level Packet Construction
// Returns number of parameter bytes added.
uint16_t MakePingPacket(uint8_t* data, uint8_t dxl_id) {
  uint8_t instruction = 1;
  uint16_t num_parameters_tx = 0;
  SetPacketPrefix(data, dxl_id, instruction, num_parameters_tx);
  return num_parameters_tx;
}

// Returns number of parameter bytes added.
uint16_t MakeReadPacket(uint8_t* data, uint8_t dxl_id, uint16_t start_addr, uint16_t num_bytes) {
  uint8_t instruction = 2;
  uint16_t num_parameters_tx = 4;
  SetPacketPrefix(data, dxl_id, instruction, num_parameters_tx);
  SetWordParam(data, 0, start_addr); 
  SetWordParam(data, 2, num_bytes); 
  return num_parameters_tx;
}

// For reading only a single byte
uint16_t MakeReadBytePacket(uint8_t* data, uint8_t dxl_id, uint16_t addr) {
  uint8_t instruction = 2;
  uint16_t num_parameters_tx = 4;
  SetPacketPrefix(data, dxl_id, instruction, num_parameters_tx);
  SetWordParam(data, 0, addr); 
  SetWordParam(data, 2, 1); 
  return num_parameters_tx;
}

// For reading two consecutive bytes
uint16_t MakeReadWordPacket(uint8_t* data, uint8_t dxl_id, uint16_t start_addr) {
  uint8_t instruction = 2;
  uint16_t num_parameters_tx = 4;
  SetPacketPrefix(data, dxl_id, instruction, num_parameters_tx);
  SetWordParam(data, 0, start_addr); 
  SetWordParam(data, 2, 2); 
  return num_parameters_tx;
}

// Returns number of parameter bytes added.
uint16_t MakeWritePacket(uint8_t* data, uint8_t dxl_id, uint16_t start_addr, uint8_t* write_data, uint16_t num_write_bytes) {
  uint8_t instruction = 3;
  uint16_t num_parameters_tx = 2 + num_write_bytes; // Start addr and data
  SetPacketPrefix(data, dxl_id, instruction, num_parameters_tx);
  SetWordParam(data, 0, start_addr); 
  int i;
  for (i = 0; i < num_write_bytes; i++) {
    SetByteParam(data, 2 + i, write_data[i]); 
  }
  return num_parameters_tx;
}

uint16_t MakeWriteBytePacket(uint8_t* data, uint8_t dxl_id, uint16_t addr, uint8_t write_data) {
  uint8_t instruction = 3;
  uint16_t num_parameters_tx = 2 + 1; // Start addr and data
  SetPacketPrefix(data, dxl_id, instruction, num_parameters_tx);
  SetWordParam(data, 0, addr); 
  SetByteParam(data, 2, write_data); 
  return num_parameters_tx;
}

uint16_t MakeWriteWordPacket(uint8_t* data, uint8_t dxl_id, uint16_t start_addr, uint16_t write_data) {
  uint8_t instruction = 3;
  uint16_t num_parameters_tx = 2 + 2; // Start addr and data
  SetPacketPrefix(data, dxl_id, instruction, num_parameters_tx);
  SetWordParam(data, 0, start_addr); 
  SetWordParam(data, 2, write_data); 
  return num_parameters_tx;
}
// TODO Reg write
// TODO Action
// TODO Factory reset
// TODO Reboot
// TODO Sync read
// TODO Sync write
// TODO Bulk read
// TODO Bulk write
