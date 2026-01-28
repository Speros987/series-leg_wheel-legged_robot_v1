//
// Created by WXR on 2025/12/26.
//

#include "state_define.h"

chassis_state_Def chassis_state = chassis_error;
controller_state_Def controller_state = controller_error;
manual_state_Def manual_state = manual_error;
gyroscope_state_Def gyroscope_state = gyroscope_error;
balance_state_Def balance_state = unbalanced;
skid_state_Def left_skid_state = not_skid;
skid_state_Def right_skid_state = not_skid;
height_finish_state_Def height_finish_state = height_finished;
height_state_Def height_state = height_low;
off_ground_state_Def left_off_ground_state = not_off_ground;
off_ground_state_Def right_off_ground_state = not_off_ground;
jump_state_Def jump_state = idle;
stumble_state_Def left_stumble_state = not_stumble;
stumble_state_Def rightstumble_state = not_stumble;
