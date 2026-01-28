//
// Created by WXR on 2026/1/20.
//

#ifndef SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_LIE_DOWN_H
#define SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_LIE_DOWN_H

#include "struct_typedef.h"
#include "remoter.h"

extern uint8_t lie_down_state;
extern uint16_t lie_down_tick;
extern uint16_t lie_down_time;

void Lie_Down(void);

#endif //SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_LIE_DOWN_H