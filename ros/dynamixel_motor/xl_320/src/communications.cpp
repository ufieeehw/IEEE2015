#include "communications.h"

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "dxl_hal.h"
#include "crc.h"

#include "packet_utils.h"

int GetBaudRate(int baud_num) {
  switch(baud_num) {
  case 0:
    return 9600;
  case 1:
    return 57600;
  case 2:
    return 115200;
  case 3:
    return 1000000;
  default:
    printf("Invalid dynamixel baud number: %d\n", baud_num);
    assert(false);
  }
  return -1;
}

int InitDXL(int deviceIndex, int baudnum ) {
  if(dxl_hal_open(deviceIndex, GetBaudRate(baudnum)) == 0) {
    return 0;
  }

  return 1;
}

void TerminateDXL(void) {
  dxl_hal_close();
}

// Sends the data over opened DXL serial line.
// Returns false if not all the bytes were sent.
bool TXPacket(uint8_t* data, int num_bytes) {
  assert(num_bytes < MAX_PACKET_BYTES);
  // PrintPacket(data, num_bytes);
  FinalPacketConsistencyCheck(data, num_bytes);
 
  int num_transmitted_bytes = dxl_hal_tx(data, num_bytes);
  if(num_bytes != num_transmitted_bytes) {
    return false;
  } else {
    return true;
  }
}


// Caller is resonsible for the received_data buffer's appropriate size.
// Returns the total numberSet bytes received for packet.
// If receive timed out, value of num_bytes_received is undefined.
bool RXPacket(uint8_t* received_data, int packet_length, int* num_bytes_received) {
  dxl_hal_set_timeout(packet_length);

  // Repeatedly attempt to read until either we have all of the bytes
  // requested or the timeout expires.
  // TODO replace this logic with something that isn't so CPU intensive.
  // like interrupt driven...
  int bytes_read = 0; 
  while (bytes_read < packet_length) {
    if (dxl_hal_timeout() == 1) {
      return false;
    }
    assert(packet_length < MAX_PACKET_BYTES);
    bytes_read += dxl_hal_rx(&(received_data[bytes_read]), packet_length);
  }

  // PrintPacket(data, num_bytes);

  // Now attempt to match the header to the data received.
  int header_match_count = 0;
  int buffer_index = 0;
  while (header_match_count < HEADER_SIZE) {
    if (buffer_index >= bytes_read) {
      return false;
    }
    if (received_data[buffer_index++] == HEADER[header_match_count]) {
      header_match_count++;
    } else {
      header_match_count = 0;
    }
  }

  // Now, the header is matched
  // Move bytes down to the beginning of the data buffer.
  int packet_start_index = buffer_index - HEADER_SIZE;
  bytes_read -= packet_start_index;
  memmove(received_data, &(received_data[packet_start_index]), bytes_read);

  // Now read the remaining bytes if necessary.
  while (bytes_read < packet_length) {
    assert((packet_length - bytes_read) < MAX_PACKET_BYTES);
    bytes_read += dxl_hal_rx(&(received_data[bytes_read]), packet_length - bytes_read);
    if (dxl_hal_timeout() == 1) {
      return false;
    }
  }

  uint16_t expected_crc = UpdateCRC(0, received_data, bytes_read - CHECKSUM_SIZE);
  uint16_t received_crc = MakeWord(received_data[bytes_read - 2], received_data[bytes_read - 1]);
  if (expected_crc != received_crc) {
    // CRC check failed.
    return false;
  }

  *num_bytes_received = bytes_read;
  return true;
}

// Error within the status packet is considered to be "parameter 0"
// So Pings receive 4 bytes of parameters.
// Also, data sent is overwritten with returned data. The buffer is reused.
bool TXRXPacket(uint8_t* data, int num_params_sending, int num_params_receiving) {
  SetChecksum(data, num_params_sending);
  // TRANSMIT PACKET
  bool result = TXPacket(data, BASE_PACKET_SIZE + num_params_sending);
  if (!result) {
    // printf("Couldn't send packet successfully.\n");
    return false;
  }

  // If broadcast, don't wait for reply.
  if (data[ID_ADDR] == BROADCAST_ID) {
    return true;
  }

  // RECEIVE PACKET
  int num_bytes_received;
  result = RXPacket(data, BASE_PACKET_SIZE + num_params_receiving, &num_bytes_received);
  if (!result) {
    // printf("Couldn't receive packet successfully.\n");
    return false;
  }
  return true;
}
