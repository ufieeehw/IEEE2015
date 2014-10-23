/* Function definitions for meta operations (Kill, Debug, Start) */

#ifndef __IEEE_DEBUB_H__
#define __IEEE_DEBUG_H__

//define prototypes
void meta_init(void);
int debug_msg(Message m);
int start_msg(Message m);
int kill_msg(Message m);

//prototypes from main.c
void init(void);

//flag definition for starting
extern int start_ok;

#endif