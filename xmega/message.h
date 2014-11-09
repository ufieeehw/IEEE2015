/* Message Handling definitions for IEEE robot */
#ifndef  _IEEE_MESSAGE_H_
#define _IEEE_MESSAGE_H_

#include <stdint.h>

//message definition, linked list storage
typedef struct Message Message;
struct Message{
  uint8_t  type;      //message type ID
  uint8_t  size;      //size of the data field
  uint8_t* data;      //message data
  uint8_t  cache_tag; //used internally for data caching
  uint8_t  msg_tag;   //used internally for msg caching
  Message* next;      //used internally for queueing
};

//define maximum queue sizes (256 is 1.5 KB, out of 4KB total storage)
#define MAX_MESSAGE 256
extern int message_count;

//define the queue heads (don't interact without using methods)
extern Message* in_queue;
extern Message* out_queue;

//define the queue tails
extern Message* in_queue_end;
extern Message* out_queue_end;

//directional definitions to make things easier
#define OUT_QUEUE     0x01
#define IN_QUEUE      0x00

//queue interaction functions (in = 0)
int queue_pop(Message* m, int direction); //get a message from the queue
int queue_push(Message m, int direction); //add a message to the queue
Message get_msg(uint8_t type, uint8_t size);  //get a message struct with data allocation

//functions that you shouldn't call
void wipe_queue(int direction);   //wipe a queue
Message free_msg(Message m);      //free the memory in a message
void init_msg_queue();            //initialize queues

#endif

