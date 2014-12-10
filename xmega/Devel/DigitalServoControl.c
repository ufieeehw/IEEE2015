/*
 * DigitalServoControl.c
 *
 * Created: 11/1/2014 3:35:52 PM
 *  Author: Zach Goins
 */ 

// Instruction Packet  0xFF  -- 0xFF -- SERVO_ID(hex) -- LENGTH(Parameters+2) -- INSTRUCTION(0x03 = Write, 0x02 = Read) -- PARAMTERS -- CHECKSUM
// Status Packet	   0xFF  -- 0xFF -- SERVO_ID(hex) -- LENGTH(Parameters+2) -- ERROR_CODE -- PARAMETERS -- CHECKSUM

// Port D Pin 0 is TX/RX in
// Port D Pin 1 is Logic Controller
// Logic control is HIGH for allowing TX and LOW for allowing RX


#define F_CPU				32000000UL
#define Baud_Rate			1000000
#define Main_Port			PORTC
#define UART_Port			USARTC0
#define TX_Direction			Main_Port.DIR
#define TX_Port				Main_Port.OUT
#define TXoffRXon		    	TX_Port = (0b00000000)
#define TXonRXoff			TX_Port |= (0b00011100)
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
unsigned int Bsel = 1;
signed int BScale = 0;

	
//----------------------------------------------------------------------------------------------
// Function used to transmit packet to one servo
// CAN ONLY BE USED ON ONE SERVO BECAUSE id IS BEING SENT - Unless using the broadcast IDs to set servo ID's initially

void send_Char(uint8_t x){

	// If the buffer has not overflowed and
	// If the transmit buffer has room for data
	
	while (!(UART_Port.STATUS & USART_DREIF_bm));
	UART_Port.DATA = x;
	
}

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size){
	unsigned short i, j;
	unsigned short crc_table[256] = { 
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,\
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,\
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,\
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,\
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,\
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,\
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,\
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,\
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,\
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,\
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,\
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,\
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,\
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,\
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,\
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,\
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,\
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,\
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,\
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,\
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,\
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,\
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,\
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,\
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,\
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,\
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,\
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,\
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,\
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,\
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,\
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202 \
	};
	
	for(j = 0; j < data_blk_size; j++){
		i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}
	
	return crc_accum;
}

void Transmit_Write_320XL(uint8_t ID, uint8_t Instruction, unsigned char ParamNum){
	
	unsigned char length = (ParamNum + 3);
	unsigned char data_blk_size = (3 + 1 + 1) + length + ParamNum - 2; 
	
	unsigned short TestPacket[] = {0xFF ,0xFF, 0xFD, 0x00, 0x01, 0x00, 0x05, 0x03, 0x19, 0x07};
	unsigned short C;

	unsigned char CRC_L = (data_blk_size & 0x00FF);
	unsigned char CRC_H = ((data_blk_size >> 8) & 0x00FF);
	
	TXonRXoff;
	
	for(int x = 0; x < ParamNum; x++){
		send_Char(TestPacket[x]);
	}

	/*send_Char(Header_1);
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
	send_Char(CRC_L);
	send_Char(CRC_H);*/

	// UART test 
	
	/*send_Char('a');
	send_Char('b');
	send_Char('c');
	send_Char('d');*/
	
}

 void uart_interupt_setup(){
	 
	 PORTC_DIRCLR = 0x04;
	 PORTC_DIRSET = 0x08;
	 
	 USARTC0.CTRLA = USART_RXCINTLVL_MED_gc;
	 USARTC0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	 USARTC0.CTRLC = USART_CHSIZE_8BIT_gc;
	 
	 USARTC0.BAUDCTRLA = (Bsel & 0xFF);
	 USARTC0.BAUDCTRLB = (BScale << 4 & 0xF0) | ((Bsel >> 8) & 0x0F);
	 
	 PMIC.CTRL |= PMIC_MEDLVLEN_bm;
	 sei();
	 
 }
 
 void set_ID(){
	 
	 // For sending the broadcast id and setting an id on connected servos
	 // Refer to Packet settings at top
	 
	 Parameters[0] = CHG_ID; // Change ID parameter
	 Parameters[1] = 0x01;  // pass id = 1 to connected servos - Change up to 254
	 
	 Transmit_Write_320XL(Broadcast_ID_320XL, Write, 2);
 }

//---------------------------------------------------------------------------------------------------

int main(void){
	
	OSC.CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm;
		while (!(OSC.STATUS & OSC_RC32MEN_bm)){
		CPU_CCP = CCP_IOREG_gc;
		CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
		}
	
	uart_interupt_setup();
	
    while(1) 
    {    
		asm("nop");
    }
}

ISR (USARTC0_RXC_vect)
{	
	Parameters[0] = LED;
	Parameters[1] = 0b00000111; //0x07
	
	Transmit_Write_320XL(Broadcast_ID_320XL, RegWrite, 2);
	
	// For testing
	// USARTC0.DATA = USARTC0.DATA;
}

ISR (USARTC0_TXC_vect)
{
	//TXoffRXon;
}
