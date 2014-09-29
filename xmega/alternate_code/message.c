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
  if(!direction){               //in queue
    if(!in_queue) return QUEUE_EMPTY_TYPE;  //queue is empty
    m->next = in_queue;         //add the node
    in_queue = m;               //redefine the tail
  } else{                       //out queue
    if(!out_queue) return QUEUE_EMPTY_TYPE;
    m->next = out_queue;
    out_queue = m;
  }
  return OK;
}

/* Function will add message to selected queue
 * Returns 0 if successfull */
int queue_push(Message* m, int direction){
  if(MAX_MESSAGE >= message_count) return QUEUE_FULL_TYPE;  //no more space
  
  m->next = 0;  //set the next pointer to null
  
  //determine the proper queue
  if(!direction){               //incoming
    in_queue_end->next = m;     //add the node
    in_queue_end = m;           //redefine the tail
  } else {                      //out
    out_queue_end->next = m;
    out_queue_end = m;
  }
  return OK;
}


