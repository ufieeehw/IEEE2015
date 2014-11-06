/*
 * DigitalServoControl.c
 *
 * Created: 11/1/2014 3:35:52 PM
 *  Author: Zach Goins
 */ 

// Instruction Packet  0xFF  -- 0xFF -- SERVO_ID(hfex) -- LENGTH(Parameters+2) -- INSTRUCTION(0x03 = Write, 0x02 = Read) -- PARAMTERS -- CHECKSUM
// Status Packet	   0xFF  -- 0xFF -- SERVO_ID(hex) -- LENGTH(Parameters+2) -- ERROR_CODE -- PARAMETERS -- CHECKSUM

// Port D Pin 0 is TX/RX in
// Port D Pin 1 is Logic Controller
// Logic control is HIGH for allowing TX and LOW for allowing RX


#define F_CPU				16000000UL
#define Baud_Rate			28800
#define TX_Direction			PORTD_DIR
#define TX_Port				PORTC_OUT
#define TXoffRXon			TX_Port = (0b00000010)
#define TXonRXoff			TX_Port |= (0b00000001)
#define Read				0x02	
#define Write				0x03
#define Broadcast_ID_320XL		0xFE 
#define CHG_ID				0x0F

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile unsigned char recieved_Byte;
unsigned char Parameters[30];
unsigned int Bsel = 428;
signed int BScale = -7;

//----------------------------------------------------------------------------------------------
// Function used to transmit packet to one servo
// CAN ONLY BE USED ON ONE SERVO BECAUSE id IS BEING SENT - Unless using the broadcast IDs to set servo ID's initially

void Transmit_Write(unsigned char ID, unsigned char Instruction, unsigned char ParamNum){
	
	unsigned char length = (ParamNum + 2);
	unsigned char checkSum = ID + Instruction + length;
	
	USARTC0.DATA = 0xFF;
	USARTC0.DATA = 0xFF;
	USARTC0.DATA = ID;
	USARTC0.DATA = (ParamNum + 2);
	USARTC0.DATA = Instruction;
	for(int x = 0; x < ParamNum; x++){
		USARTC0.DATA = Parameters[x];
		checkSum += Parameters[x];
	}
	USARTC0.DATA = ~checkSum;
	
}

//---------------------------------------------------------------------------------------------------

int main(void){
	
		// UART setup with interrupts 
	
		PORTC_DIRCLR = 0x04;
		PORTC_DIRSET = 0x08;
		
		USARTC0.CTRLA = USART_RXCINTLVL_MED_gc;
		USARTC0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
		USARTC0.CTRLC = USART_CHSIZE_8BIT_gc;
		
		USARTC0_BAUDCTRLA = (Bsel & 0xFF);
		USARTC0_BAUDCTRLB = (BScale << 4 & 0xF0) | ((Bsel >> 8) & 0x0F);
		
		PMIC.CTRL |= PMIC_MEDLVLEN_bm;
		sei();
		
		// This is for sending the broadcast id and setting an id on connected servos
		// refer to Packet settings at top
		
		Parameters[0] = CHG_ID // Change ID parameter
		Parameters[1] = 0x01;  // pass id = 1 to connected servos
		
		Transmit_Write(Broadcast_ID_320XL, Write, 2);
	
    while(1) 
    {    
		asm("nop");
    }
}

ISR (USARTC0_RXC_vect)
{
	TXonRXoff;
	USARTC0.DATA = USARTC0.DATA; 
}

ISR (USARTC0_TXC_vect)
{
	TXoffRXon;
}
