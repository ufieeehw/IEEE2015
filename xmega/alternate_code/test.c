/* quick test to see if system can handle the archetecture */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "message.h"
#include "usart.h"
#include "types.h"

int main(){
  CLK.CTRL = 0x01;  //switch to the 32MHz clock
  initialize_usart(); 
  PMIC.CTRL = 7;  //enable all interrupts
  sei();  //globally enable interrupts
  
  Message* m = 0; //pointer for messages
  while(1){ //loop forever
    if(in_queue){
      queue_pop(m,IN_QUEUE);
      queue_push(m,OUT_QUEUE);
      if(!usart_busy_flag) notify_data();
    }
  }
  
  return 0;
}