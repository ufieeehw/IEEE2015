/* main program for the IEEE robot */
/* add your initialization calls to the init function */

//includes, add more as needed
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "types.h"
#include "message.h"
#include "usart.h"
#include "table.h"

//Quantum definitions (how many buffer operations per function call)
#define BUFFER_ALLOWED  16

void debug();

/* initialization function for the robot, add your own code */
void init(){
  //change the clock mode
  OSC.CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm; // enable 32 Mhz clock (don't disable 2 Mhz)
	while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { } // wait for it to stabilize
	CPU_CCP = CCP_IOREG_gc; // enable access to protected registers
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to the 32 Mhz clock
  
  //do component initializations
  //INSERT FUNCTION CALLS HERE
  
  //initialize communications
  initialize_usart();
  
  debug();
  
  //finally, enable interrupts
  PMIC.CTRL = 7; //enable all interrupt levels
  sei();  //global interrupt enable
}

/* main function, don't change without consulting Josh */
int main(){
  init(); //call initializations
  while(1){  //don't break out of loop
    resolve_buffers(BUFFER_ALLOWED); //resolve some of the buffer
    
    if(in_queue){ //if there is a message, throw to the handler
      Message m; //set pointer to null
      int status = VECTOR_ERROR_TYPE;	//default to error
      queue_pop(&m, IN_QUEUE); //get incoming message
      int index = m.type & 0x3F; //index is last 6 bits of message type
      switch(m.type >> 6){ //determine data type (first 2 bits)
        case 0: //no data type
          if(index < NO_DATA_ARRAY_SIZE) status = (*no_data_func[index])(m);
          break;
        case 1: //1b data type
          if(index < DATA_1B_ARRAY_SIZE) status = (*data_1b_func[index])(m);
          break;
        case 2: //2b data type
          if(index < DATA_2B_ARRAY_SIZE) status = (*data_2b_func[index])(m);
          break;
        case 3: //nb data type
          if(index < DATA_NB_ARRAY_SIZE) status = (*data_nb_func[index])(m);
          break;
      }
      if(VECTOR_ERROR_TYPE == status) status = no_func(m); //report bad vectors
      if((m.type & DATA_MASK) && m.size) free(m.data); //free data memory if it exists
      if(status != OK && status < 0x40){ //report single byte errors
        Message err; //create a message
        err.type = (uint8_t) status; //set the type to the error code
        err.size = 0;
        queue_push(err,OUT_QUEUE);  //report errors to host computer
      } 
    }
  }
}

//fill buffers with things to see what happens
void debug(){
  //load some messages into the output buffer (should output on console immediatley)
  buffer_push(out_buffer, '0'); //0x30, no_data message
  buffer_push(out_buffer, 'a'); //0x70, 1b_type message part 1
  buffer_push(out_buffer, 'b'); //0x71, 1b_type message part 2
  
  //write some output messages (should output after working through buffer)
  Message m; //create pointer
  m.type = '1';  //0x31, no_data message
  queue_push(m,OUT_QUEUE);
  m.type = 'A';  //0x41, 1b_type message part 1
  m.size = 1;
  m.data = (uint8_t*) malloc(sizeof(uint8_t));  //create the data buffer
  *(m.data) = 'B';  //0x42, 1b_type message part 2
  queue_push(m,OUT_QUEUE);  //copy to output queue
  
  //write some input messages (should get bounced to input queue after error tagging)
  m.type = '2';  //0x32, no_data message
  queue_push(m,IN_QUEUE);
  m.type = 'p';  //0x41, 1b_type message part 1
  m.size = 1;
  m.data = (uint8_t*) malloc(sizeof(uint8_t));  //create the data buffer
  *(m.data) = 'q';  //0x42, 1b_type message part 2
  queue_push(m,IN_QUEUE);  //copy to output queue
  
  //add some messages to the input queue, if everything works, these should get outputted last
  buffer_push(in_buffer, '3'); //0x33, no_data message
  buffer_push(in_buffer, 'P'); //0x50, 1b_type message part 1
  buffer_push(in_buffer, 'Q'); //0x51, 1b_type message part 2
}