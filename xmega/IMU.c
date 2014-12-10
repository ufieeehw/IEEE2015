
#include <string.h>
#include "IMU.h"
#define ACC_X 0b11101000
#define ACC_Y 0b11101010
#define ACC_Z 0b11101100
#define M_X 0b11001000
#define M_Y 0b11001010
#define M_Z 0b11001100
#include <avr/io.h>
void IMU_init(){
	uint8_t SPI_Wait_Counter;
	//Step 1: Initialize the Port E as an SPI master
	PORTD.DIRSET = 0xB0;
	SPID.CTRL = 0b01011100;
	
	PORTD.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;
	PORTD.OUTSET = PIN4_bm;
	
	//Step 2: Set the control register's sampling frequency
	uint8_t IMU_Control_Data[8] = {0b00100000, 0b01100111, 0x24, 0x94, 0x25, 0x00, 0x26, 0x00}; //This is what will be written to the IMU to set the sampling frequency
	for(int i=0; i<8; i++){
	PORTD.OUTCLR = PIN4_bm; //Set the SS line low
	SPID_DATA = IMU_Control_Data[2*i]; //Load SPID_DATA with the first byte, the address of the IMU we are accessing
	SPI_Wait_Counter = 0;
	while((SPID_STATUS!=0x80)||(SPI_Wait_Counter<10)){
		SPI_Wait_Counter+=1;
		}
	//This waits for the SPID status to throw up a flag to indicate the last transfer is complete
	//The act of checking the flag, and then accessing the SPID_DATA register, clears the flag
	//for the next transfer
	
	SPID_DATA = IMU_Control_Data[2*i+1]; //Transmits the next byte, which selects sampling frequency
	//Next, wait until the transfer is complete before ending the transfer
	SPI_Wait_Counter = 0;
	while((SPID_STATUS!=0x80)||(SPI_Wait_Counter<10)){
		SPI_Wait_Counter+=1;
		}
	PORTD.OUTSET = PIN4_bm;  //Set the SS line high to complete the transfer
	}
}

int IMU_get_data(Message m){
  uint8_t SPI_Wait_Counter;
  Message out = get_msg(IMU_DATA_TYPE, 12);
  //Step 3:
		 uint8_t IMU_Addresses[6] = {ACC_X, ACC_Y, ACC_Z, M_X, M_Y, M_Z}; //These are the addresses where target data is stored
		 uint8_t IMU_Received_Data[12];  //This is a list to store the data the the IMU returns
		 //Step 4: Transmit data
		 int IMU_Address_Index; 
		 while(1){
		 for(IMU_Address_Index=0;IMU_Address_Index<6;IMU_Address_Index++){
			 PORTD.OUTCLR = PIN4_bm;  //SS line held low again
			SPID_DATA = IMU_Addresses[IMU_Address_Index];  //Load SPID_DAta with the first address
			//Wait for SPID_STATUS to say the byte has been transferred
			SPI_Wait_Counter = 0;
			while((SPID_STATUS!=0x80)||(SPI_Wait_Counter<10)){
				SPI_Wait_Counter+=1;
				}
			//Load the SPID data with zeroes, since all that matters now is the two bytes coming from the IMU
			SPID_DATA = 0x00;
			//Wait more
			while((SPID_STATUS!=0x80)||(SPI_Wait_Counter<10)){{
				SPI_Wait_Counter+=1;
				}
			IMU_Received_Data[2*IMU_Address_Index] = SPID_DATA; //Take the received data and put it in the array.  Now you have the low byte
			//Repeat for the high byte
			SPID_DATA = 0x00;
			while((SPID_STATUS!=0x80)||(SPI_Wait_Counter<10)){
				SPI_Wait_Counter+=1;
				}
			IMU_Received_Data[2*IMU_Address_Index+1] = SPID_DATA;
			PORTD.OUTSET = PIN4_bm;
		 }
  
  
  //you can either create a buffer and load it directly, or copy to the allocated memory with memcpy
  memcpy(out.data, IMU_Received_Data, out.size);
  queue_push(out, OUT_QUEUE);
  return OK;
		 }
		 }
}

//{ACCX_H, ACCX_L, ACCY_H, ACCY_L, ACCZ_H, ACCZ_L, MX_H, MX_L, MY_H, MY_L, MZ_H, MZ_L}
//L, H