/* 

UART protocol for UF 2015 IEEE end effector
Created on: 10-February-2015
Created by: Zach Goins

*/

#define F_CPU         16000000UL

#define DESIRED_BAUD	9600						          	// Baud rate (9600 is default)
#define SCALER			  (F_CPU/16)/(DESIRED_BAUD - 1)	// Used for UBRRL and UBRRH
#define TX_START()		UCSR0B |= (1<<TXEN0)	  		// Enable TX
#define RX_START()		UCSR0B |= (1<<RXEN0)		  	// Enable RX
#define RX_INTEN()		UCSR0B |= (1<<RXCIE0)			  // Enable interrupt on RX complete
#define TX_INTEN()		UCSR0B |= (1<<TXCIE0)			  // Enable interrupt on TX complete

#define SOL_INIT	  	DDRB = 0b00011000;				    // Solenoid output to H-bridge on Pin 3 and 4
#define SOL			    	0x6A							              // This is a message about solenoids
#define SOL_COM_IN		0x6B						          	// This message is telling us to pull solenoids in
#define SOL_COM_OUT		0x6C							          // This message is telling us to push solenoids out
#define	SOL_IN			  PORTB = 0b00001000;				    // Pull solenoid in
#define SOL_OUT			  PORTB = 0b00010000;				    // Release magnet and push out

#define SERVO			    0x7A							              // This is a message about solenoids

#define TRANS_COMPLT	0xFE							          // Transmission complete

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

uint8_t recv_bytes[5];
unsigned int servo_position;

void UART_init(void);
uint8_t get_byte(void);
void put_byte(unsigned char data);
void get_string(int length);
void solenoid();
