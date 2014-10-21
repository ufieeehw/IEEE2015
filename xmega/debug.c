#include "types.h"
#include "message.h"
#include <avr/io.h>
#include <stdlib.h>

void debug_init(void){
  PORTQ.DIRSET = 0xF0;  //set led direction to output
}

int debug_msg(Message m){
  PORTQ.OUTTGL = 0xF0;  //toggle led state
  
  //prepare outgoing message
  Message out;
  out.type = DEBUG_TYPE;
  out.size = 1;
  out.data = malloc(out.size);
  out.data[0] = 'a';
  return queue_push(out, OUT_QUEUE)  ;
}