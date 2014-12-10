#ifndef PID_H_
#define PID_H_
#include "message.h"

// PID definitions
#define LEFT_FRONT_MOTOR  0
#define LEFT_REAR_MOTOR   1
#define RIGHT_FRONT_MOTOR 2
#define RIGHT_REAR_MOTOR  3

// PID queue definitions
#define ERROR_QUEUE_SIZE    64
typedef struct {
  int16_t* data;    // pointer to data buffer
  uint8_t start;    // first index with data
} Error_History;

// Init functions
void pid_init(void);
void pid_set_tunings(float Kp, float Ki, float Kd, uint8_t motor);

// jOS.h set desiered speed callback
int pid_speed_msg(Message msg);

// jOS.h razzmatazz
void error_history_push(int16_t data, uint8_t motor);  // push an error sample
int16_t error_history_at(int8_t index, uint8_t motor); // get a history entry
void error_history_batch(int16_t* buffer, uint8_t size, uint8_t motor); // return batched history entries
void update_pid(void); // call a pid update

// PID functions
int16_t pid_measure_error(uint8_t motor);
uint16_t pid_compute(uint8_t motor);

// Black magic used for software encoders
unsigned int gray_to_binary(unsigned int num);

#endif /* PID_H_ */