/* function definitions for message handling */
#include <stdlib.h>
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
    *m = *in_queue;               //copy the node
    Message *next = in_queue->next; //store next node
    free(in_queue);  //free the memory
    in_queue = next;  //advance queue
  } else{                       //out queue
    if(!out_queue) return MESSAGE_ERROR_TYPE;
    *m = *out_queue;               //add the node
    Message *next = out_queue->next; //store next node
    free(out_queue);  //free the memory
    out_queue = next;  //advance queue
  }
  m->next = 0;  //don't let return function see pointer
  message_count--;
  return OK;
}

/* Function will add message to selected queue
 * Returns 0 if successfull */
int queue_push(Message m, int direction){
  if(MAX_MESSAGE <= message_count) return MESSAGE_ERROR_TYPE;  //no more space
  
  Message* new_msg = malloc(sizeof(Message));  //create Message storage
  *new_msg = m;  //copy the data
  new_msg->next = 0;  //make sure pointer is null
  
  //deternew_msgine the proper queue
  if(IN_QUEUE == direction){     //inconew_msging
    if(!in_queue){ //special case if queue is enew_msgpty
      in_queue = new_msg;
      in_queue_end = new_msg;
    } else {  //nornew_msgal execution   
      in_queue_end->next = new_msg;     //add the node
      in_queue_end = new_msg;           //redefine the tail
    }
  } else {                      //out
    if(!out_queue){ //check if queue is enew_msgpty
      out_queue = new_msg;
      out_queue_end = new_msg;
    } else {  //nornew_msgal execution   
      out_queue_end->next = new_msg;     //add the node
      out_queue_end = new_msg;           //redefoute the tail
    }
  }
  message_count++;
  return OK;
}

//wrapper for message creation, size doesnt matter unless you're NB typed
Message get_msg(uint8_t type, uint8_t size){
  //TODO: message caching scheme
  
  Message m;
  m.type = type;  //get the type
  
  switch (type & DATA_MASK){
    case NO_DATA_TYPE: m.size = 0; break;
    case DATA_1B_TYPE: m.size = 1; break;
    case DATA_2B_TYPE: m.size = 2; break;
    case DATA_NB_TYPE: m.size = size; break;
  }

  if(m.size) m.data = (uint8_t*) malloc(m.size);  //allocate the storage
  else m.data = 0;
  
  return m; //return the message
}

//wrapper for freeing message data, don't call this (check for caching)
void free_msg(Message m){
  if((m.type & DATA_MASK) && m.size) free(m.data);
}

//remove all messages from a queue (start and kill call this)
void wipe_queue(int direction){
  Message *p = (IN_QUEUE == direction)? in_queue : out_queue;
  
  while(p){
    free_msg(*p);  //free the data
    Message *next = p->next;  //get the next pointer
    free(p);  //free the memory
    p = next; //advance list
  }
  
  //wipe the end pointer
  if(IN_QUEUE == direction) in_queue_end = 0;
  else out_queue_end = 0;
}