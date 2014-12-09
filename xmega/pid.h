
#ifndef PID_H_
#define PID_H_

typedef enum{
	WHEEL1,
	WHEEL2,
	WHEEL3,
	WHEEL4
} wheelNum;

//PID definitions
#define LEFT_FRONT_MOTOR  0
#define LEFT_REAR_MOTOR   1
#define RIGHT_FRONT_MOTOR 2
#define RIGHT_REAR_MOTOR  3

//pid queue definitions
#define ERROR_QUEUE_SIZE    64
typedef struct {
  int16_t* data;   //pointer to data buffer
  uint8_t start;    //first index with data
} Error_History;

void pid_init(void);
void error_history_push(int16_t data, uint8_t motor); //push an error sample
int16_t error_history_at(int8_t index, uint8_t motor); //get a history entry
void error_history_batch(int16_t* buffer, uint8_t size, uint8_t motor); //return batched history entries
void update_pid(void); //call a pid update
void pid_setTunings(float Kp, float Ki, float Kd, wheelNum num);
float pid_getSpeed(wheelNum num);
void pid_setSpeed(float speed, wheelNum num);
void pid_set_speed_handler(float speed);
void pid_get_speed_handler(char* message, uint8_t len);
void pid_get_odometry_handler(char* message, uint8_t len);
void pid_get_speed_multiplier_handler(char* message, uint8_t len);
void pid_set_speed_multiplier_handler(char* message, uint8_t len);
#endif /* PID_H_ */