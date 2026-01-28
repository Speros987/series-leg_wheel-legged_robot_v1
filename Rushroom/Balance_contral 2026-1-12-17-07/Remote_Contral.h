//
// Created by WXR on 2026/1/7.
//

#ifndef SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_REMOTE_CONTRAL_H
#define SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_REMOTE_CONTRAL_H

#include "DR16.h"

typedef enum
{
    User_define_is_pressed,
    User_define_not_pressed,
    User_define_is_pressed_to_not_pressed,
    User_define_not_pressed_to_is_pressed,
}button_Def;

typedef struct
{
    char User_define_x;
    char User_define_y;
}joystick_Def;

typedef enum
{
    User_define_shang,
    User_define_zhong,
    User_define_xia,
}toggle_Def;

extern button_Def zuo_shang_button ;
extern button_Def zuo_xia_button ;
extern button_Def zuo_zuo_button ;
extern button_Def zuo_you_button ;
extern button_Def you_shang_button ;
extern button_Def you_xia_button ;
extern button_Def you_zuo_button ;
extern button_Def you_you_button ;

extern joystick_Def left_joystick ;
extern joystick_Def right_joystick;

extern toggle_Def left_toggle ;
extern toggle_Def right_toggle;

void DR16_Data_Process(void);

#endif //SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_REMOTE_CONTRAL_H