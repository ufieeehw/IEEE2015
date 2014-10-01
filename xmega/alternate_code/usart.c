/* program definitions for USART handling functions */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "types.h"
#include "message.h"
#include "usart.h"

//variables for internal use only
static Message* m_in;     //message storage
static Message* m_out;    //output message cache
static int out_count;     //outgoing byte count
static int in_count;      //incoming byte count
int usart_busy_flag = 0;  //define the global flag

//Function to configure USART
//TODO: Confirm system architecture
void initialize_usart(){
  //setup TX (pin 3) to output and set default value to 1
	PORTC.DIR = PIN3_bm;
	PORTC.OUT = PIN3_bm;
	
	//Store the values for BSEL (A[7-0] and B[3-0]) and BSCALE(B[7-4])
	USARTC0.BAUDCTRLA = (BSEL & 0xFF);
	USARTC0.BAUDCTRLB = ((BSCALE << 4) & 0xF0) | ((BSEL >> 8) & 0x0F);

	// USART is Asynchronous, no parity, 1 stop bit, 8-bit mode (00-00-0-011)
	USARTC0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	
	//configure the usart to generate interrupts on recieve (high) and data ready (mid)
	USARTC0.CTRLA = USART_RXCINTLVL_HI_gc || USART_DREINTLVL_MED_gc;	//0x32

	//Finally, Turn on TX (bit 3) and RX (bit 4)
	USARTC0.CTRLB = PIN3_bm | PIN4_bm;
  
  usart_busy_flag = 0;  //usart is not busy
  in_count = 0;         //no data yet
  out_count = 0;
}

//outside handle for sending data, only needs to be called 
// if queue was empty prior to this send (interrupt normally gets it)
void notify_data(){
  if(!usart_busy_flag){ //don't interfere if we're already running
    usart_busy_flag = 1;  //usart is now busy
    USARTC0.CTRLA = USART_RXCINTLVL_HI_gc || USART_DREINTLVL_MED_gc;	//enable the interrupt
  }
}

ISR(USARTC0_RXC_vect){
	//grab the data
  in_count++; //new byte, increment count
  
  //preform different actions based on the byte count
  if(1 == in_count){  //type field
    m_in = (Message*) calloc(1,sizeof(Message));  //allocate space for the incoming message
    m_in->type = USARTC0.DATA;  //get the data
    if((m_in->type & DATA_MASK ) == NO_DATA_TYPE){
      m_in->size = 0;
    } else if((m_in->type & DATA_MASK ) == DATA_1B_TYPE){
      m_in->size = 1;
      m_in->data = (uint8_t*) malloc(1);  //allocate 1B for data
    } else if((m_in->type & DATA_MASK ) == DATA_2B_TYPE){
      m_in->size = 2;
      m_in->data = (uint8_t*) malloc(2);  //allocate 2B for data
    } //do nothing for arbitrary size since next byte is size
  } else if (2 == in_count && ((m_in->type & DATA_MASK ) == DATA_NB_TYPE)){
    m_in->size = USARTC0.DATA;  //get the size
    m_in->data = (uint8_t*) malloc(m_in->size); //allocate the requested amount of data
  } else {
    int offset = ((m_in->type & DATA_MASK ) == DATA_NB_TYPE)? in_count - 2: in_count - 1;
    *((m_in->data)+offset) = USARTC0.DATA;  //store the byte
  }
  
  //check if we're done recieving this message
  if(m_in->size == in_count+1 || (((m_in->type & DATA_MASK ) == DATA_NB_TYPE) && m_in->size == in_count + 2)){
    queue_push(m_in,IN_QUEUE);   //push the incoming message
    in_count = 0; //reset counter for next message
  }
}

ISR(USARTC0_DRE_vect){
  if(in_queue || m_out){ //check if there's a message to send
    if(0 == out_count){
      queue_pop(m_out,OUT_QUEUE); //get the next queue
      USARTC0.DATA = m_out->type;
    }
    
    ++out_count;
    if(((m_in->type & DATA_MASK ) == DATA_NB_TYPE)){  //arbitrary length buffers
      if(m_out->size + 2 == out_count){ //we're done jim
        free(m_out);    //free the memory
        m_out = 0;      //point to null
        out_count = 0;  //stop sending data
      }
    } else {  //normal data
      if(m_out->size + 1 == out_count){ //we're done jim
        free(m_out);    //free the memory
        m_out = 0;      //point to null
        out_count = 0;  //stop sending data
      }
    }
  } else {  //nothing to send, shutdown
    usart_busy_flag = 0;  //we're no longer busy
    USARTC0.CTRLA = USART_RXCINTLVL_HI_gc; //disable the DRIEF interrupt
  }
}