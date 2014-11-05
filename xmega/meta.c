/* Function definitions for meta operations (Kill, Debug, Start) */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "types.h"
#include "message.h"
#include "meta.h"

int start_ok; //initialize flag

void meta_init(void){
  //Start
  start_ok = 0;  //have not started yet
  
  //Keep_Alive
  CCP = CCP_IOREG_gc; //enable protected access
  WDT.CTRL = WDT_CEN_bm; //disable the timer
  
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

//tell programt to start listening for messages
int start_msg(Message m){
  wipe_queue(OUT_QUEUE); //delete old messages
  start_ok = 1;   //begin listening/sending messages
  
  //start the watchdog timer (1 second timeout)
  CCP = CCP_IOREG_gc; //enable protected access
  WDT.CTRL = WDT_ENABLE_bm | WDT_CEN_bm | 0x1C;
  
  //send ACK message
  Message out = get_msg(START_TYPE, 0);
  return queue_push(out, OUT_QUEUE);
}

int kill_msg(Message m){
  cli();  //no more interrupts till we're done
  
  //initiate the reset
  CPU_CCP = CCP_IOREG_gc; //enable access to protected registers
  RST.CTRL = 0x01;        //initiate reset
  
  return OK;  //code will not get here
}

int keep_alive_msg(Message m){
  //reset the timer
  asm volatile("wdr");
  
  Message out = get_msg(KEEP_ALIVE_TYPE,0);
  queue_push(out,OUT_QUEUE);  //send ack
  
  return OK;
}