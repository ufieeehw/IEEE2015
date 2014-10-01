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

/* initialization function for the robot, add your own code */
void init(){
  //change the clock mode
  CLK.CTRL = 0x01;  //switch to the 32MHz clock
  
  //do component initializations
  //INSERT FUNCTION CALLS HERE
  
  //initialize communications
  initialize_usart();

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
      Message *m = 0; //set pointer to null
      int status = VECTOR_ERROR_TYPE;	//default to error
      queue_pop(m, IN_QUEUE); //get incoming message
      int index = m->type & 0x3F; //index is last 6 bits of message type
      switch(m->type >> 6){ //determine data type (first 2 bits)
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
      if(status != OK){ //create an error message and ship it off
        Message* m = calloc(1,sizeof(Message)); //create a message
        m->type = (uint8_t) status; //set the type to the error code
        queue_push(m,OUT_QUEUE); //report errors to host computer
      }
    }
  }
}
