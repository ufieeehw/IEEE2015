/* function definitions for message handling */
#include "message.h"
#include "types.h"

//initialize the global variables (no messages and null pointers)
int message_count = 0;
Message* in_queue = 0;
Message* out_queue = 0;
Message* in_queue_end = 0;
Message* out_queue_end = 0;


/* Function will remove message from proper queue (1 for out/priority)
 * Function gets the next message, places it in m, and deletes its reference
 * Returns 0 (OK) if successful */
int queue_pop(Message* m, int direction){
  //determine the proper queue
  if(IN_QUEUE == direction){               //in queue
    if(!in_queue) return MESSAGE_ERROR_TYPE;  //queue is empty
    m = in_queue;               //add the node
    in_queue = in_queue->next;  //redefine the head
  } else{                       //out queue
    if(!out_queue) return MESSAGE_ERROR_TYPE;
    m = out_queue;
    out_queue = out_queue->next;
  }
  return OK;
}

/* Function will add message to selected queue
 * Returns 0 if successfull */
int queue_push(Message* m, int direction){
  if(MAX_MESSAGE <= message_count) return MESSAGE_ERROR_TYPE;  //no more space
  
  m->next = 0;  //set the next pointer to null
  
  //determine the proper queue
  if(IN_QUEUE == direction){     //incoming
    if(!in_queue){ //special case if queue is empty
      in_queue = m;
      in_queue_end = m;
    } else {  //normal execution   
      in_queue_end->next = m;     //add the node
      in_queue_end = m;           //redefine the tail
    }
  } else {                      //out
    if(!out_queue){ //check if queue is empty
      out_queue = m;
      out_queue_end = m;
    } else {  //normal execution   
      out_queue_end->next = m;     //add the node
      out_queue_end = m;           //redefoute the tail
    }
  }
  return OK;
}


