#ifndef UART_H_
#define UART_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

void uart_init();
bool uart_put(char ch);
int uart_get();
bool uart_send_msg_block(uint8_t type, char* msg, uint8_t len);
bool uart_send_msg_noblock(uint8_t type, char* msg, uint8_t len);
char* uart_get_msg(void);
bool uart_get_msg_status();
void uart_set_msg_status(bool status);
float uart_int32_to_float(char* data);
float uart_int16_to_float(char* data);
float uart_int8_to_float(char* data);
void uart_float_to_char32(char* buffer, float data);
void uart_float_to_char16(char* buffer, float data);
void uart_float_to_char8(char* buffer, float data);

// uart Handler Functions
void uart_echo_request(char* message, uint8_t len);
void uart_echo_reply(char* message, uint8_t len);

typedef enum RxStates_enum {
	WaitingForStart,
	GrabLength,
	ReceivingMessage,
	Processing
} RxStates_t;

typedef enum MessageTypes_enum {
	ACKValid = 0x00,
	ACKInvalid = 0x01,
	EchoRequest = 0x02,
	EchoReply = 0x03,
	PIDsetSpeed = 0x04,
	PIDgetOdometry = 0x05,
	PIDsetMultiplier = 0x06,
	PIDgetMultiplier = 0x07,
	PIDgetSpeed = 0x08,
	TCSGetRawData = 0x09

}MessageTypes_t;


#endif /* UART_H_ */
