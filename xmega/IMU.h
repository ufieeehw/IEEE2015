#ifndef __IMU_IEEE_H__
#define __IMU_IEEE_H__
#include "message.h"
#include "types.h"
void IMU_init();
int IMU_get_data(Message m);
#endif
