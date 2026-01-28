//
// Created by WXR on 2025/12/26.
//

#ifndef SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_LQR_H
#define SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_LQR_H

#include "panel.h"
#include "my_maths.h"
#include "BMI088/App/imu_temp_ctrl.h"
#include "remoter.h"

void lqr(void);
void yaw_pid (void);
void roll_pid(void);
void lqr_pid_parallel (void);

#endif //SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_LQR_H
