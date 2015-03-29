#include "uart.h"
#include <stdlib.h>
#include <math.h>


// Configuration Values
static USART_t &uart_usb = USARTC0;
#define RXVEC_USB USARTC0_RXC_vect
#define TXVEC_USB USARTC0_DRE_vect
static PORT_t &uart_port_usb = PORTC;
static const uint8_t txpin_usb = 3;
static const uint8_t rxpin_usb = 2;
static const uint16_t bsel_usb = 3301;		// 19200 baud
static const uint8_t bscale_usb = 0b1011;
static bool echo_enabled = true;
static const uint8_t max_message_len = 64;
static const char start_char = '^';
// flags
volatile static bool waiting_for_msg_ACK = false;
static bool no_ACK_after_3x = false;
static uint8_t start_char_count = 0;
static uint8_t rx_message_len = 0;
static uint8_t tx_message_len = 0;
static RxStates_t rx_state = WaitingForStart;
bool message_to_be_handled = false;



// Buffer to contain data from UART.
struct UARTData {
	char buffer[max_message_len];
	volatile uint8_t buf_pos;
};

static UARTData uart_data_usb;


// Functions
void uart_init() {
	uart_port_usb.OUTSET = _BV(txpin_usb);
	uart_port_usb.DIRSET = _BV(txpin_usb);
	
	uart_usb.CTRLA = USART_RXCINTLVL_MED_gc; // Set the interrupt priority of the receive flag, but not the DREIF, since the DREIF will fire as soon as the data register is empty.
	uart_usb.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	uart_usb.CTRLC = USART_CHSIZE_8BIT_gc;
	uart_usb.BAUDCTRLA = bsel_usb & 0xFF;
	uart_usb.BAUDCTRLB = (bscale_usb << USART_BSCALE_gp) | (bsel_usb >> 8);	
}

// Places a character in the buffer, only if there is room to do so, and enables the transmit interrupt
// so that the transmit ISR will be hit.
bool uart_put(char ch) {
	// First check to make sure that the buffer pointer isn't pointing to the end of the buffer already.
	if (uart_data_usb.buf_pos >= sizeof(uart_data_usb.buffer))
		return false;
	
	// Since it's not, we'll make sure the DREIF IF is disabled, so that we're not interrupted while adding data to the buffer and updating the pointer location.
	uart_usb.CTRLA &= ~USART_DREINTLVL_gm;
	uart_data_usb.buffer[uart_data_usb.buf_pos++] = ch;
	uart_usb.CTRLA |= USART_DREINTLVL_MED_gc; // Re-enable the interrupt, so that it'll transmit what was just placed in teh buffer
	return true;
}

// Gets a character from the data register in the uart and puts it into the input buffer.

static void receive() {
	cli();
	
	// First get the data from the UART.
	uint8_t byte = uart_usb.DATA;
	
	switch(rx_state){
		case WaitingForStart:
			if(message_to_be_handled == false){
				if(byte==start_char){
					start_char_count++;
				}
			
				if(start_char_count == 3){
					rx_state = GrabLength;
					start_char_count = 0;
				}
			}
		break;

		case GrabLength:
			rx_message_len = byte;
			uart_data_usb.buffer[0] = rx_message_len; // Grab the message length and put it into the first position in teh buffer
			uart_data_usb.buf_pos = 1;
			rx_state = ReceivingMessage;
			//CRCInit();
		break;
		
		case ReceivingMessage:
			if(waiting_for_msg_ACK == true){
				if(byte == ACKValid){
					waiting_for_msg_ACK = false;
					rx_state = WaitingForStart;
					break;
				}
				else if(byte == ACKInvalid){
					//retransmit();
					break;
				}	
			}
			
			
			else if(rx_message_len-- > 0){
				uart_data_usb.buffer[uart_data_usb.buf_pos++] = byte;
			}else {
				message_to_be_handled = true;
				rx_state = WaitingForStart;
				uart_data_usb.buf_pos = 0;
			}
		break;
		
		default:
		break;
	
	}
	
	sei();
}

// Gets a character from the output buffer and puts it into uart to be transmitted
static void transmit() {
	// First check if there's untransmitted message data in the buffer.
	if(uart_data_usb.buf_pos < tx_message_len) {
		// If there is, then send the first untransmitted character in the buffer.
		uart_usb.DATA = uart_data_usb.buffer[uart_data_usb.buf_pos];
		uart_data_usb.buffer[uart_data_usb.buf_pos] = 0; // This'll clear out the buffer as the characters are transmitted, so disable if you want to keep the message.
		uart_data_usb.buf_pos++;
		
	} else {
		//If there isn't any data in the buffer, then disable the DREIF so that it doesn't continue interrupting.
		uart_usb.CTRLA &= ~USART_DREINTLVL_gm;
		uart_data_usb.buf_pos = 0;  // This sets the buffer back to the beginning when finished transmitting, which is necessary if you've zeroed out the buffer.
	}
}

bool uart_send_msg_noblock(uint8_t type, char* msg, uint8_t len){
	//len++; // Include line to print null character at the end of the message
	
	tx_message_len = len + 4;
	if(tx_message_len > max_message_len){
		return false;
	}
	
	//if anyone has any reason why this transmission should not occur, speak now or forever --ah, you know the rest
	
	
	// we'll make sure the DREIF IF is disabled, so that we're not interrupted while adding data to the buffer and updating the pointer location.
	uart_usb.CTRLA &= ~USART_DREINTLVL_gm;
	
	//fill first 3 bytes of buffer with start character (^)
	memset(uart_data_usb.buffer, '^', 3);
	//copy the len(gth) into the next byte of the buffer, 
	uart_data_usb.buffer[3] = len;
	//...followed by the type, 
	uart_data_usb.buffer[4] = type;
	//...and len-1 bytes of the msg (type + msg = len bytes long)
	for(int i = 0; i < (len-1); i++){
		uart_data_usb.buffer[i+5] = *msg; //copy the data at address pointed to by msg into buffer 
										  //(start at 5 positions after start because
		msg++;
	}
	uart_data_usb.buf_pos = 0; 
	
	//len+4 = number of characters from first ^ to null + 1 to make the buf_pos point to the first empty position?
	
	//make sure that the TXISR is activated
	uart_usb.CTRLA |= USART_DREINTLVL_MED_gc; // Re-enable the interrupt, so that it'll transmit what was just placed in teh buffer

	//set the waiting_for_msg_ACK flag
	waiting_for_msg_ACK = true;
	
	
	return true; 
}

bool uart_send_msg_block( uint8_t type, char* msg, uint8_t len)
{ 
	
	if(uart_send_msg_noblock(type, msg, len) == false){
		return false;
	}
	
	while(waiting_for_msg_ACK);
	
	if(no_ACK_after_3x){
		return false;
	}
	
	return true;
}

char* uart_get_msg(void){
	return (char*)&uart_data_usb.buffer;
}

bool uart_get_msg_status() {
	return message_to_be_handled;
}

void uart_set_msg_status(bool status) {
	message_to_be_handled = status;
}

void uart_echo_request(char* message, uint8_t len) {
	char *messageCopy = (char*)calloc(len, 1);
	memcpy((void*)messageCopy, (void*)message, len);
	uart_echo_reply(messageCopy, len);
	free(messageCopy);
}

void uart_echo_reply(char* message, uint8_t len) {
	uart_send_msg_block(EchoReply, message, len);
}

float uart_int32_to_float(char* data) {
	int32_t haxor32 = (static_cast<uint32_t>(data[3]) << 24) |
	                           (static_cast<uint32_t>(data[2]) << 16) |
							   (static_cast<uint32_t>(data[1]) <<  8) |
							   (static_cast<uint32_t>(data[0]) <<  0);
	float flaxor = (float)(haxor32/1000.0);
	return flaxor;
}
// Redo the functions in here to reflect the one above.
float uart_int16_to_float(char* data) {
	int16_t haxor16 = (int16_t)(data[1] << 8 | data[0]);
	float flaxor = (float)(haxor16)/1000.0;
	return flaxor;
}

float uart_int8_to_float(char* data) {
	int16_t haxor8 = (int8_t)(data[0]);
	float flaxor = (float)(haxor8)/1000.0;
	return flaxor;
}

void uart_float_to_char32(char* buffer, float data) {
	int32_t haxor32 = roundf(data*1000.);
	uint32_t tmp = static_cast<uint32_t>(haxor32);
	buffer[0] = tmp >> 0;
	buffer[1] = tmp >> 8;
	buffer[2] = tmp >> 16;
	buffer[3] = tmp >> 24;

	/*buffer[0] = (char)((haxor32     ) & 0xFF);
	buffer[1] = (char)((haxor32 >> 8) & 0xFF);
	buffer[2] = (char)((haxor32 >> 16) & 0xFF);
	buffer[3] = (char)((haxor32 >> 24) & 0xFF);*/
	
}

void uart_float_to_char16(char* buffer, float data) {
	int16_t haxor16 = data*1000;
	for(int i = 0; i < 2; i++){
		buffer[i] = (char)((haxor16 >> i*8) & 0x00FF);
	}
}

void uart_float_to_char8(char* buffer, float data) {
	int8_t haxor8 = data*1000;
	for(int i = 0; i < 1; i++){
		buffer[i] = (char)((haxor8 >> i*8) & 0xFF);
	}
}



ISR(TXVEC_USB) {
	transmit();
}

ISR(RXVEC_USB) {
	receive();
}

