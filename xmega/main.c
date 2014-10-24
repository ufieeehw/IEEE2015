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
#include "meta.h"

//Quantum definitions (how many buffer operations per function call)
#define BUFFER_ALLOWED  16

/* initialization (and reset) function for the robot, add your own code */
void init(){
  //change the clock mode
  OSC.CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm; // enable 32 Mhz clock (don't disable 2 Mhz)
	while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { } // wait for it to stabilize
	CPU_CCP = CCP_IOREG_gc; // enable access to protected registers
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to the 32 Mhz clock
  
  //do component initializations
  meta_init();  //initialize meta functions (should come first)
  
  //initialize communications
  initialize_usart();
}

/* main function, don't change without consulting Josh */
int main(){
  init(); //call initializations
    
  //enable interrupts
  PMIC.CTRL = 7; //enable all interrupt levels
  sei();  //global interrupt enable
  
  while(1){  //don't break out of loop
    resolve_buffers(BUFFER_ALLOWED); //resolve some of the buffer
    
    if(in_queue){ //if there is a message, throw to the handler
      Message m; //set pointer to null
      int status = VECTOR_ERROR_TYPE;	//default to error
      queue_pop(&m, IN_QUEUE); //get incoming message
      
      //wait until allowed to start
      if(!start_ok && m.type != START_TYPE){
        free_msg(m);
        continue; 
      }
      
      //send to receiver
      uint8_t index = m.type & 0x3F; //index is last 6 bits of message type
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
      
      free_msg(m);  //free data memory
      
      if(status != OK && status < 0x40){ //report single byte errors
        Message err; //create a message
        err.type = (uint8_t) status; //set the type to the error code
        err.size = 0;
        queue_push(err,OUT_QUEUE);  //report errors to host computer
      } 
    }
  }
}
