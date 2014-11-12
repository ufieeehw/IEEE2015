/*
 * DigitalServoControl.c
 *
 * Created: 11/1/2014 3:35:52 PM
 *  Author: Zach Goins
 */ 

// Instruction Packet  0xFF  -- 0xFF -- 0xFD -- 0x00-- SERVO_ID(hex) -- LENGTH(Parameters+3) -- INSTRUCTION -- PARAMETERS -- CHECKSUM1 -- CHECKSUM2
// Status Packet	   0xFF  -- 0xFF -- SERVO_ID(hex) -- LENGTH(Parameters+2) -- ERROR_CODE -- PARAMETERS -- CHECKSUM

// Port D Pin 0 is TX/RX in
// Port D Pin 1 is Logic Controller
// Logic control is HIGH for allowing TX and LOW for allowing RX


#define F_CPU				2000000UL
#define Baud_Rate			28800
#define Main_Port			PORTD
#define UART_Port			USARTD0
#define TX_Direction			Main_Port.DIR
#define TX_Port				Main_Port.OUT
#define TXoffRXon			TX_Port = (0b10000010)
#define TXonRXoff			TX_Port |= (0b00001000)
#define Read				0x02	
#define Write				0x03
#define RegWrite			0x04
#define Broadcast_ID_320XL		0xFE 
#define CHG_ID				0x0F
#define Header_1			0xFF
#define Header_2			0xFD
#define	Reserved			0x00
#define LED				0x19

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

volatile unsigned char recieved_Byte;
unsigned char Parameters[30];
unsigned int Bsel = 27;
signed int BScale = -3;

//----------------------------------------------------------------------------------------------

void send_Char(uint8_t x){

	// If the buffer has not overflowed and
	// If the transmit buffer has room for data
	
	while (!(UART_Port.STATUS & USART_DREIF_bm));
	UART_Port.DATA = x;
	
}

// Function used to transmit packet to one servo
// CAN ONLY BE USED ON ONE SERVO BECAUSE id IS BEING SENT - Unless using the broadcast IDs to set servo ID's initially

void Transmit_Write(uint8_t ID, uint8_t Instruction, unsigned char ParamNum){
	
	unsigned char length = (ParamNum + 3);
	unsigned char data_blk_size = (3 + 1 + 1) + length + ParamNum - 2; 
	unsigned char CRC_I = (data_blk_size & 0x00FF);
	unsigned char CRC_H = ((data_blk_size >> 8) & 0x00FF);
	
	TXonRXoff;
	
	send_Char(Header_1);
	send_Char(Header_1);
	send_Char(Header_2);
	send_Char(Reserved);
	send_Char(ID);
	send_Char(0x00);
	send_Char(length);
	send_Char(Instruction);
	for(int x = 0; x < ParamNum; x++){
		send_Char(Parameters[x]);
	}
	send_Char(CRC_I);
	send_Char(CRC_H);

	// UART test 
	
	/*send_Char('a');
	send_Char('b');
	send_Char('c');
	send_Char('d');*/
	
}

 void uart_interupt_setup(){
	 
	 Main_Port.DIRCLR = 0x04;
	 Main_Port.DIRSET = 0x08;
	 
	 UART_Port.CTRLA = USART_RXCINTLVL_MED_gc;
	 UART_Port.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	 UART_Port.CTRLC = USART_CHSIZE_8BIT_gc;
	 
	 UART_Port.BAUDCTRLA = (Bsel & 0xFF);
	 UART_Port.BAUDCTRLB = (BScale << 4 & 0xF0) | ((Bsel >> 8) & 0x0F);
	 
	 PMIC.CTRL |= PMIC_MEDLVLEN_bm;
	 sei();
	 
 }
 
 void set_ID(){
	 
	 // For sending the broadcast id and setting an id on connected servos
	 // Refer to Packet settings at top
	 
	 Parameters[0] = CHG_ID; // Change ID parameter
	 Parameters[1] = 0x01;  // pass id = 1 to connected servos - Change up to 254
	 
	 Transmit_Write(Broadcast_ID_320XL, Write, 2);
 }

//---------------------------------------------------------------------------------------------------

int main(void){
	
	uart_interupt_setup();
	
    while(1) 
    {    
		asm("nop");
    }
}

ISR (USARTC0_RXC_vect)
{	
	Parameters[0] = LED;
	Parameters[1] = 0b00000111;
	
	Transmit_Write(Broadcast_ID_320XL, RegWrite, 2);
	
	// For testing
	// USARTC0.DATA = USARTC0.DATA;
}

ISR (USARTC0_TXC_vect)
{
	//TXoffRXon;
}
