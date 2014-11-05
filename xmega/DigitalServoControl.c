/*
 * DigitalServoControl.c
 *
 * Created: 11/1/2014 3:35:52 PM
 *  Author: Zach Goins
 */ 

// Instruction Packet Format  0xFF  -- 0xFF -- SERVO_ID(hex) -- LENGTH(Parameters+2) -- INSTRUCTION(0x03 = Write, 0x02 = Read) -- PARAMTERS -- CHECKSUM
// Status Packet Format	      0xFF  -- 0xFF -- SERVO_ID(hex) -- LENGTH(Parameters+2) -- ERROR_CODE -- PARAMETERS -- CHECKSUM


// Port D Pin 0 is assumed TX/RX input
// Port D Pin 1 is assumed Logic Controller
// Logic control is HIGH for allowing TX and LOW for allowing RX


#define F_CPU			16000000UL
#define Baud_Rate		9600	
#define TX_Direction		PORTD_DIRSET
#define TX_Port			PORTD_DIR
#define TXoffRXon     	        TX_Port |= (1<<1), TX_Port &= ~(1<<2)
#define TXonRXoff		TX_Port |= (1<<1), TX_Port &= ~(1<<2)
#define Read			0x02	
#define Write			0x03
#define Broadcast_ID		0xFE			// Specific to the XL-320
#define CHG_ID			0x0F

#include <avr/io.h>
#include "uart.h"
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile unsigned char recieved_Byte;
unsigned char Parameters[30];				// Array to hold packet arguments

//---------------------------------------------------------------------------------------------------------------------------------------------
// Function used to transmit packet to one servo
// CAN ONLY BE USED ON ONE SERVO BECAUSE id IS BEING SENT

void Transmit_Write(unsigned char ID, unsigned char Instruction, unsigned char ParamNum){
	
	unsigned char length = (ParamNum + 2);
	unsigned char checkSum = ID + Instruction + length;
	
	TXonRXoff;
	
	uart_putc(0xFF);
	uart_putc(0xFF);
	uart_putc(ID);
	uart_putc(ParamNum + 2);
	uart_putc(Instruction);
	for(int x = 0; x < ParamNum; x++){
		uart_putc(Parameters[x]);
		checkSum += Parameters[x];
	}
	uart_putc(~checkSum);
	
}

//-------------------------------------------------------------------------------------------------------------------------------------------

int main(void){
	
	TX_Direction = 0b00000011;							// Set Port D as output
	
	int init_Buad_Rate  = UART_BAUD_SELECT(Baud_Rate, F_CPU);			// Normal UART Speed
	int init_Buad_Rate_Double = UART_BAUD_SELECT_DOUBLE_SPEED(Baud_Rate, F_CPU);	// Double UART Speed
	
	uart_init(init_Buad_Rate);							// Initialize UART Transmitting
	
	sei();										// Set Global Variables 
	
	// Test While loop for right now
	// Used until I can test with logic analyzer
	
    while(1)																		
    {
		Parameters[0] = CHG_ID;
		Parameters[1] = 0x01;
		Transmit_Write(Broadcast_ID, Write, 2);
		_delay_ms(1000);
    }
}

// ------------------------------------------------------------------------------------------------------------------------------------------

ISR (USARTD0_RXC_vect)		// Interrupt to catch when data is received back from servo 
{
	 recieved_Byte = USARTD0_DATA; 
}

ISR (USARTD0_TXC_vect)
{
	TXoffRXon;
}
