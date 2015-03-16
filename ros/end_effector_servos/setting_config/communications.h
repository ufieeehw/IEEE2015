#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

#include <stdbool.h>
#include <stdint.h>


static const uint8_t BROADCAST_ID = 0xFE;

// Convert from DXL baud numbers used in the dxl registers
// to actual baud rates.
int GetBaudRate(int baud_num);

// Opening the USB serial device (USB2AX)
int InitDXL(int deviceIndex, int baudnum);
void TerminateDXL(void);

// Sends the data over opened DXL serial line.
// Returns false if not all the bytes were sent.
bool TXPacket(uint8_t* data, int num_bytes);

// Caller is resonsible for the received_data buffer's appropriate size.
// Returns the total number of bytes received for packet.
// If receive timed out, value of num_bytes_received is undefined.
bool RXPacket(uint8_t* received_data, int packet_length, int* num_bytes_received);

// Error within the status packet is considered to be "parameter 0"
// So Pings receive 4 bytes of parameter.
bool TXRXPacket(uint8_t* data, int num_params_sending, int num_params_receiving);

#endif // COMMUNICATIONS_H
