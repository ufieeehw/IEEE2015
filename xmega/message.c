/* function definitions for message handling */
#include <stdlib.h>
#include <string.h>
#include "message.h"
#include "types.h"

//initialize the global variables (no messages and null pointers)
int message_count = 0;
Message* in_queue = 0;
Message* out_queue = 0;
Message* in_queue_end = 0;
Message* out_queue_end = 0;

//define cache sizes and associations
#define MSG_CACHE_SIZE  32
#define MSG_CACHE_MASK  3
#define DATA_CACHE_SIZE 32
#define DATA_CACHE_NUM  8
#define DATA_CACHE_1B   0
#define DATA_CACHE_2B   1
#define DATA_CACHE_4B   2
#define DATA_CACHE_6B   3
#define DATA_CACHE_12B  6

/* create the data caches */
uint8_t data_cache_1B[DATA_CACHE_SIZE];
uint8_t data_cache_2B[DATA_CACHE_SIZE];
uint8_t data_cache_4B[DATA_CACHE_SIZE];
uint8_t data_cache_6B[DATA_CACHE_SIZE];
uint8_t data_cache_12B[DATA_CACHE_SIZE];  //IMU uses this

//setup the pointers to the data caches
uint8_t* data_cache_toplvl[DATA_CACHE_NUM] = {data_cache_1B, data_cache_2B, data_cache_4B, data_cache_6B, 0, 0, data_cache_12B, 0};

//32-bit map for used/unused words (1st bit is 1 if word is free)
uint32_t data_cache_map[DATA_CACHE_NUM];  //data cache avaliability tracker

/* create the message cache */
Message msg_cache[MSG_CACHE_SIZE] = {{0}};  //initialize all cache members to 0
uint32_t msg_cache_map; //msg cache bit tracker
Message* get_node();           //get a message node
void free_node(Message* m);   //clear the message node 

//message to initialize queues and caches
void init_msg_queue(){
  //wipe the message queues (mostly for resets)
  wipe_queue(OUT_QUEUE);
  wipe_queue(IN_QUEUE);
  
  message_count = 0;  //reset the count
  
  //initalize the caches
  data_cache_map[DATA_CACHE_1B] = 0x7FFFFFFF; //first bit is null
  data_cache_map[DATA_CACHE_2B] = 0xAAAAAAAA;
  data_cache_map[DATA_CACHE_4B] = 0x88888888;
  data_cache_map[DATA_CACHE_4B] = 0x82082080;
  data_cache_map[DATA_CACHE_12B] = 0x80080000;
  msg_cache_map = 0xFFFFFFFF;
}

/* Function will remove message from proper queue (1 for out/priority)
 * Function gets the next message, places it in m, and deletes its reference
 * Returns 0 (OK) if successful */
int queue_pop(Message* m, int direction){
  //determine the proper queue
  if(IN_QUEUE == direction){               //in queue
    if(!in_queue) return MESSAGE_ERROR_TYPE;  //queue is empty
    *m = *in_queue;               //copy the node
    Message *next = in_queue->next; //store next node
    free_node(in_queue);  //free the memory
    in_queue = next;  //advance queue
  } else{                       //out queue
    if(!out_queue) return MESSAGE_ERROR_TYPE;
    *m = *out_queue;               //add the node
    Message *next = out_queue->next; //store next node
    free_node(out_queue);  //free the memory
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
  
  Message *new_msg = get_node();  //get the pointer
  new_msg->type = m.type;   //copy the data fields
  new_msg->size = m.size;
  new_msg->data = m.data;
  new_msg->cache_tag = m.cache_tag;
  
  //deternew_msgine the proper queue
  if(IN_QUEUE == direction){     //incming message
    if(!in_queue){ //special case if queue is enew_msgpty
      in_queue = new_msg;
      in_queue_end = new_msg;
    } else {  //nornew_msgal execution   
      in_queue_end->next = new_msg;     //add the node
      in_queue_end = new_msg;           //redefine the tail
    }
  } else {                      //out
    if(!out_queue){ //check if queue is empty
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
// will try to get data storage from cache if avaliable
Message get_msg(uint8_t type, uint8_t size){  
  Message m;
  m.type = type;  //get the type
  
  switch (type & DATA_MASK){
    case NO_DATA_TYPE: m.size = 0; break;
    case DATA_1B_TYPE: m.size = 1; break;
    case DATA_2B_TYPE: m.size = 2; break;
    case DATA_NB_TYPE: m.size = size; break;
  }

  if(!m.size){  //no_data types don't need caching
    m.data = 0;
    m.cache_tag = 0;
    return m;
  }
  
  int cache_num = -1;
  switch(m.size){
    case 1: cache_num = DATA_CACHE_1B; break;
    case 2: cache_num = DATA_CACHE_2B; break;
    case 4: cache_num = DATA_CACHE_4B; break;
    case 6: cache_num = DATA_CACHE_4B; break;
    case 12: cache_num = DATA_CACHE_12B; break;
  }
  
  //make sure cache exists and isn't full (map is empty)
  if(-1 != cache_num && data_cache_map[cache_num]){
    for(int i=0; i<DATA_CACHE_SIZE; i+=m.size){
      if(data_cache_map[cache_num] & (1 << (DATA_CACHE_SIZE - i - 1))){  //check cache
        m.data = data_cache_toplvl[cache_num] + i;  //get the pointer
        m.cache_tag = ((0xE0 & (cache_num << 5)) | (0x1F & i)); //tag (size)[7-5](index)[4-0]
        data_cache_map[cache_num] &= ~(1 << (DATA_CACHE_SIZE - i - 1));  //clear cache bit
        return m; //we're done, return message
      }
    }  
  } else {
    m.data = (uint8_t*) malloc(m.size);  //allocate the storage
    m.cache_tag = 0;  //no tag
  }
  
  return m; //return the message
}

//wrapper for freeing message data, don't call this (handles caching)
Message free_msg(Message m){
  if(!(m.type & DATA_MASK)) return m; //no_data, nothing to do
  
  if(!m.cache_tag && m.data) free(m.data); //no caching, free data
  else {  //cache exists, free the location
    data_cache_map[(m.cache_tag >> 5) & 7] |= 1 << (m.cache_tag & 0x1F);
    m.cache_tag = 0;  //no more cache
  }
  m.data = 0; //reset the data pointer
  return m;
}

//free the data associated with a linked-list node
void free_node(Message* m){
  if(m->msg_tag){ //message is valid, free cache
    uint32_t bit = 1 << (m->msg_tag & 0x1F); //convert index to a bit
    msg_cache_map |= bit;  //set the bit to indicate avaliability
  } else free(m); //otherwise free the data, was not cached
}

//get a linked-list node
Message* get_node(){
  Message* m;
  
  if(msg_cache_map)  //if location is avaliable
    for(int i=0; i<MSG_CACHE_SIZE; i+=1)
      if(msg_cache_map & (1 << (MSG_CACHE_SIZE - i - 1))){  //check cache
        m = &msg_cache[i]; //get the pointer
        memset(m, 0, sizeof(Message)); //clear the message (just in case)
        m->msg_tag = (0xC0 | (0x1F & i)); //tag (110)[7-5](index)[4-0]
        msg_cache_map &= ~(1 << (MSG_CACHE_SIZE - i - 1));  //clear cache bit
        return m; //we're done, return message
      }
  m = (Message*) calloc(1,sizeof(Message));  //cache unavaliable, allocate msg
  
  return m;
}

//remove all messages from a queue (start and kill call this)
void wipe_queue(int direction){
  Message *p = (IN_QUEUE == direction)? in_queue : out_queue;
  
  while(p){
    free_msg(*p);  //free the data
    Message *next = p->next;  //get the next pointer
    free_node(p);  //free the memory
    p = next; //advance list
  }
  
  //wipe the end pointer
  if(IN_QUEUE == direction) in_queue_end = 0;
  else out_queue_end = 0;
}

