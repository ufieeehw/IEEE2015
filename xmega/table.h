/* header file for message type function pointers */
#ifndef _IEEE_TABLE_H_
#define _IEEE_TABLE_H_

//pointer array size definitions
#define NO_DATA_ARRAY_SIZE	4
#define DATA_1B_ARRAY_SIZE	4
#define DATA_2B_ARRAY_SIZE	4
#define DATA_NB_ARRAY_SIZE	4

//function array definitions
extern int (*no_data_func[NO_DATA_ARRAY_SIZE]) (Message m);
extern int (*data_1b_func[DATA_1B_ARRAY_SIZE]) (Message m);
extern int (*data_2b_func[DATA_2B_ARRAY_SIZE]) (Message m);
extern int (*data_nb_func[DATA_NB_ARRAY_SIZE]) (Message m);

//no_func prototype
int no_func(Message m);

#endif

