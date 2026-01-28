//
// Created by WXR on 2025/12/26.
//

#ifndef SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_VMC_H
#define SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_VMC_H

#include "panel.h"
#include "my_maths.h"
#include "DM_can_IO.h"
#include "can.h"
#include <math.h>
#include "state_define.h"
#include "dsp/fast_math_functions.h"
#include "Virtual_Chassis_Control.h"

void vmc();
void left_vmc();
void right_vmc();

#endif //SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_VMC_H