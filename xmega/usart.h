/* Usart definitions and functions for the IEEE robot */
#ifndef  _IEEE_USART_H_
#define  _IEEE_USART_H_

//Baud rate is 256000Hz
#define BSEL    872
#define BSCALE  -7

//data buffer definitions
#define MAX_BUFFER_LENGTH  64
typedef struct Buffer Buffer;
struct Buffer{
  uint8_t data[MAX_BUFFER_LENGTH];  //buffer data array
  int     start;  //start index
  int     end;    //end index
};

//delcare the buffers
extern Buffer* in_buffer;
extern Buffer* out_buffer;


//function will configure the USART to prepare it to run
void initialize_usart();

//functions for manipulating buffers (internal use only, don't actually call these)
int buffer_push(Buffer* b, uint8_t data);
int buffer_pop(Buffer* b, uint8_t* data);
void wipe_in_buffer();     //clear the buffers
void wipe_out_buffer();
void resolve_buffers(int bytes);  //resolve bytes of buffered data
void resolve_single_input();      //resolve one input byte
void resolve_single_output();     //resolve one output byte

//flag indicates weather the system is busy
extern int usart_busy_flag;

#endif
