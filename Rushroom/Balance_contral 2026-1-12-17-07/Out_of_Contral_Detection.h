//
// Created by WXR on 2025/12/27.
//

#ifndef SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_WHEEL_ACCELERATATION_DETECTION_H
#define SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_WHEEL_ACCELERATATION_DETECTION_H

#include "state_define.h"
#include "panel.h"
#include "DM_can_IO.h"
#include <math.h>
#include "DM_can_IO.h"
#include "imu_temp_ctrl.h"

void out_of_contral_detection ();
void left_wheel_acceleration_detection ();
void right_wheel_acceleration_detection ();
void fall_detection ();
void stable_detection ();

#endif //SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_WHEEL_ACCELERATATION_DETECTION_H