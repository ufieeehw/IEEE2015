/* Function definitions for meta operations (Kill, Debug, Start) */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "types.h"
#include "message.h"
#include "usart.h"
#include "meta.h"

int start_ok; //default to not starting

void meta_init(void){
  //Start
  start_ok = 0;  //have not started yet
  
  //Debug
  PORTQ.DIRSET = 0xF;  //set led direction to output
}

int debug_msg(Message m){
  PORTQ.OUTTGL = 0xF;  //toggle led state
  
  //send outgoing message
  Message out = get_msg(DEBUG_TYPE, 1);
  out.data[0] = 'a';
  return queue_push(out, OUT_QUEUE);
}

int start_msg(Message m){
  wipe_queue(OUT_QUEUE); //delete old messages
  wipe_out_buffer();      //delete all pending data
  start_ok = 1; //begin listening and talking
  
  //send ACK message
  Message out = get_msg(START_TYPE, 0);
  return queue_push(out, OUT_QUEUE)  ;
}

int kill_msg(Message m){
  cli();  //no more interrupts till we're done
  
  init(); //reset all components (called twice, but is more secure)
  
  wipe_queue(OUT_QUEUE);  //clear all pending messages
  wipe_queue(IN_QUEUE);
  wipe_in_buffer();       //delete all pending data
  wipe_out_buffer();       
  
  //initiate the reset
  CPU_CCP = CCP_IOREG_gc; //enable access to protected registers
  RST.CTRL = 0x01;        //call the reset
  
  return OK;  //code will not get here
}