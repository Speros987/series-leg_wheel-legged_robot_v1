//
// Created by WXR on 2025/12/26.
//

#ifndef SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_LQR_H
#define SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_LQR_H

#include "panel.h"
#include "my_maths.h"
#include "BMI088/App/imu_temp_ctrl.h"

void lqr();
void yaw_pid ();
void roll_pid();
void lqr_pid_parallel ();

#endif //SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_LQR_H