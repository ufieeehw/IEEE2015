/*

UART protocol for UF 2015 IEEE end effector
Created on: 10-February-2015
Created by: Zach Goins

*/

#include "end_effector.h"
 
// Configures USART and Baud Rate
void UART_init(void)
{
	// Not necessary; initialize anyway
	DDRD |= (1<<1);
	DDRD &= (0<<0);
 
	// Set baud rate - lower byte and top nibble
	UBRR0H = ((SCALER) & 0xF00);
	UBRR0L = (uint8_t) ((SCALER) & 0xFF);
 
	TX_START();
	RX_START();
	RX_INTEN();
	TX_INTEN();
 
	// Set frame format = 8-N-1
	UCSR0C = (0x03 << UCSZ00);
 
}
 
//Returns a byte from the serial buffer
uint8_t get_byte(void)
{
	// Check to see if something was received
	while (!(UCSR0A & _BV(RXC0)));
	return (uint8_t) UDR0;
}
 
// Transmits a byte
void put_byte(unsigned char data)
{
	// Stay here until data buffer is empty
	while (!(UCSR0A & _BV(UDRE0)));
	UDR0 = (unsigned char) data;
 
}

// Receive multiple Bytes before taking action
void get_string(int length)
{
	// length specifies length of incoming message
	for (int x = 0; x < length; x++)
	{
		recv_bytes[x] = get_byte();
	}	
}

// Depending on received bytes, activates solenoids 
void solenoid(){
	
	if (recv_bytes[1] == SOL_COM_IN)
	{
		SOL_IN;
		putByte(0x00);
	}
	else if (recv_bytes[1] == SOL_COM_OUT)
	{
		SOL_OUT;
		putByte(0x01);
	}
	
}
 
int main(void)
{ 
	cli();				// disable interrupts
	UART_init();	
	sei();				// enable interrupts
	SOL_INIT			// enable solenoid output 
	
    while(1);
}

ISR(USART_RX_vect)
{	
	get_string(2);
	
	if ((recv_bytes[0] == SOL)
	{
		solenoid();
	}
	if ((recv_bytes[0] == SERVO)
	{
		
	}
	
	put_byte(TRANS_COMPLT);
}

ISR(USART_TX_vect)
{
	
}
