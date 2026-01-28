#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H

#include "state_define.h"

void IMU_task(void * argument);
void gyroscope_outline_detection(void);

extern float gyro[3], accel[3], temp;
extern float roll,pitch,yaw;

#endif // IMU_TEMP_CTRL_H
