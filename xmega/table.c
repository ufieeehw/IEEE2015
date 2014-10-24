/* Function pointer definitions for each messgae type. */
/* Each function takes exactly one message as an argument, and returns an error type (int). */
/* YOU MUST REGISTER YOUR SYSTEM HERE AND IN TYPES.H (use the same value)*/
/* Please increase the constant array size in table.h if you expand the array */
/* DO NOT CREATE MORE THAN 48 ENTRY's PER ARRAY (6 bits - 16 error types)*/
 
#include <stdlib.h>
#include <string.h>
#include "types.h"
#include "message.h"
#include "table.h"
#include "meta.h"
 
//NO_DATA_TYPE function pointers (messages with only a type field)
int (*no_data_func[NO_DATA_ARRAY_SIZE]) (Message m) = {
  no_func,      //0x00 - reserved (OK)
  kill_msg,     //0x01 - kill
  start_msg,    //0x02 - start
  no_func       //0x03 - unused
};
 
//DATA_1B_TYPE function pointers (messages with 1 byte of data)
int (*data_1b_func[DATA_1B_ARRAY_SIZE]) (Message m) = {
  debug_msg,  //0x40 - debug
  no_func,    //0x41 - unused
  no_func,    //0x42 - unused
  no_func     //0x43 - unused
};


//DATA_2B_TYPE function pointers (messaegs with 2 bytes of data)
int (*data_2b_func[DATA_2B_ARRAY_SIZE]) (Message m) = {
  no_func,    //0x80 - unused
  no_func,    //0x81 - unused
  no_func,    //0x82 - unused
  no_func     //0x83 - unused
};


//DATA_NB_TYPE function pointers (arbitrary length messages)
int (*data_nb_func[DATA_NB_ARRAY_SIZE]) (Message m) = {
  no_func,    //0xC0 - unused
  no_func,    //0xC1 - unused
  no_func,    //0xC2 - unused
  no_func     //0xC3 - unused
};
 
 
//placeholder function will report errors, returns queue status (also good example function)
int no_func(Message m){
  if(IS_ERROR_TYPE(m.type)){  //misdirected error, bounce to out_queue
    //need to copy the data buffer otherwise it will be deleted on return
    Message m_out = m;  //create a copy of the message
    if((m.type & DATA_MASK) != NO_DATA_TYPE){
      m_out.data = malloc(m.size);  //allocate new buffer space
      memcpy(m_out.data, m.data, m.size); //copy data
    }
    return queue_push(m_out,OUT_QUEUE); //bounce to outgoing queue
  }
  
  //otherwise pack new error message
  Message m_out;
  m_out.type = VECTOR_ERROR_TYPE;  //set type
  m_out.size = ((m.type & DATA_MASK) == DATA_NB_TYPE)? m.size+2 : m.size+1;  //get the size
  m_out.data = malloc(m_out.size);  //allocate space
  m_out.data[0] = m.type;  //copy the type field
  if((m.type & DATA_MASK) == DATA_NB_TYPE){  //NB_types send a size field
    m_out.data[1] = m.size;
    memcpy(m_out.data+2,m.data,m_out.size-2);
  } else {  //other types don't send size field
    memcpy(m_out.data+1,m.data,m_out.size-1);
  }
  
  return queue_push(m_out,OUT_QUEUE);  //add the message to the queue
}