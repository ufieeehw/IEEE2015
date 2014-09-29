/* program definitions for USART handling functions */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "types.h"
#include "message.h"
#include "usart.h"

int usart_busy_flag = 0;    //define the global flag
Buffer in_buffer;           //define the buffers
Buffer out_buffer;
static int error = OK;  //flag for error handling

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

//add data to the start of the buffer
int buffer_push(Buffer* b, uint8_t data){
  if((b->end+1)%MAX_BUFFER_SIZE == b->start) return BUFFER_FULL_TYPE;
  b->data[end] = data;  //add to end of list
  end = (end+1)%MAX_BUFFER_SIZE;  //increment end index
  return OK;
}

//remove data from start of buffer
int buffer_pop(Buffer* b, uint8_t* data){
  if(b->end == b->start) return BUFFER_EMPTY_TYPE;
  data* = b->data[start]; //get the first element and increment index
  start = (start+1)%MAX_BUFFER_SIZE;  //increment start index
  return OK;
}

/* function will alternate between buffers until both are full/empty, 
 * then will block until more are avaliable. Will also handle error flags.
 * Should be in its own thread, somehow. */
void resolve_buffers(){
  int turn = IN_QUEUE;  //keep track of who's turn it is

  while(true){  //this goes forever man
    //go until in buffer is empty and output buffer is full or there are no more messages
    if(in_buffer->start != in_buffer->end || ((out_buffer->start != (out_buffer->end+1)%MAX_BUFFER_SIZE) && !in_queue)){
      //decide who's turn it is (alternate unless somebody is full/empty)
      if(in_buffer->start == in_buffer->end) turn = OUT_QUEUE; //if in_buffer is empty, it's out buffer's turn
      else if((out_buffer->start != (out_buffer->end+1)%MAX_BUFFER_SIZE) && !in_queue) turn  = IN_QUEUE; //and vice versa
      else turn = (turn+1)%2; //otherwise alternate

      //throw to individualized methods
      if(turn == IN_QUEUE) resolve_single_input();
      else  resolve_single_output();
    } else {
      block();  //wait for work
    }
    if(error != OK){  //check error status
      Message* m = calloc(1,sizeof(Message)); //create a message
      m->type = (uint8_t) error; //set the type to the error
      queue_push(m,IN_QUEUE); //handle internally?
      error = OK; //reset the flag
    }
  }
}

/* function will resolve a single input message from the buffer */
// Might be worth recombining into resolve_buffers for better blocking efficency
void reslove_single_input(){
  int in_count = 0; //count the bytes
  Message * m_in = (Message*) calloc(1,sizeof(Message));  //allocate space for message
  uint8_t data;
  
  do{
    //wait for byte to become avaliable if its still being sent
    while(buffer_pop(in_buffer, &data)){ //wait for successfull buffer read
      block(); //TODO: figure this out (throw to main program?)
    } 
  
    in_count++; //new byte, increment count
  
    //preform different actions based on the byte count
    if(1 == in_count){  //type field
      m_in->type = data;  //get the data
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
      m_in->size = data;  //get the size
      m_in->data = (uint8_t*) malloc(m_in->size); //allocate the requested amount of data
    } else {
      int offset = ((m_in->type & DATA_MASK ) == DATA_NB_TYPE)? in_count - 2: in_count - 1;
      *((m_in->data)+offset) = data;  //store the byte
    }
  } while(m_in->size == in_count+1 || (((m_in->type & DATA_MASK ) == DATA_NB_TYPE) && m_in->size == in_count + 2));
  queue_push(m_in,IN_QUEUE);   //push the incoming message to the queue
}

/* function will place a single output message in the buffer */
void resolve_single_output{
  Message* m_out;
  queue_pop(m_out,OUT_QUEUE); //get the next message in the queue
  uint8_t data;
  int bytes, out_count = 0;
  if((m_out->type & DATA_MASK ) == DATA_NB_TYPE)  bytes = 2 + m_out->size;
  else bytes = 1 + m_out->size;
  
  
  do{
    ++out_count;
    if(1 == out_count) data = m_out->type;  //send the type field
    else if((m_out->type & DATA_MASK ) == DATA_NB_TYPE) && out_count == 2) data = m_out->size; //send the size
    else{ //send the data
      int offset = (m_out->type & DATA_MASK ) == DATA_NB_TYPE)? out_count - 2 : out_count - 1;
      data = m_out->data[offset];
    }
    
    //wait for space in buffer to send data
    while(buffer_push(out_buffer, data)){ //wait for successfull buffer read
      block(); //TODO: figure this out
    }
    
  }while(out_count < bytes);
  free(m_out);  //free the message memory
}

ISR(USARTC0_RXC_vect){
	int status = buffer_push(in_buffer,USARTC0.DATA);
  if(status != OK) error = status; //report errors
  unblock();  //TODO
}

ISR(USARTC0_DRE_vect){
  int status = buffer_pop(out_buffer,&USARTC0.DATA);
  if(status != OK) error = status; //report errors
  unblock();  //TODO
}