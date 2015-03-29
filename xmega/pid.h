#ifndef PID_H_
#define PID_H_

#include <avr/io.h>

#include "pololu.h"

enum wheelNum{
	WHEEL_LF,
	WHEEL_LR,
	WHEEL_RF,
	WHEEL_RR
};

void pid_init();
void pid_setTunings(float Kp, float Ki, float Kd, wheelNum num);
float pid_getSpeed(wheelNum num);
void pid_setSpeed(float speed, wheelNum num);
void pid_set_speed_handler(char* message, uint8_t len);
void pid_get_speed_handler(char* message, uint8_t len);
void pid_get_odometry_handler(char* message, uint8_t len);
void pid_get_speed_multiplier_handler(char* message, uint8_t len);
void pid_set_speed_multiplier_handler(char* message, uint8_t len);
void pid_set_tuning_handler(char* message, uint8_t len);


#endif /* PID_H_ */
