/* Usart definitions and functions for the IEEE robot */
#ifndef  _IEEE_USART_H_
#define  _IEEE_USART_H_

//Baud rate is 256000Hz
#define BSEL    872
#define BSCALE  -7

//function will configure the USART to prepare it to run
void initialize_usart();

//function to alert the UART to continue sending data
// should onle need to be called if queue was previously stopped
void notify_data();

//flag indicates weather the system is busy
extern int usart_busy_flag;

#endif
