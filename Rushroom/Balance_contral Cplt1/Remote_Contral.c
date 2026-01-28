//
// Created by WXR on 2026/1/7.
//

#include "Remote_Contral.h"

button_Def zuo_shang_button ;
button_Def zuo_xia_button ;
button_Def zuo_zuo_button ;
button_Def zuo_you_button ;
button_Def you_shang_button ;
button_Def you_xia_button ;
button_Def you_zuo_button ;
button_Def you_you_button ;

joystick_Def left_joystick = {0,0};
joystick_Def right_joystick = {0,0};

toggle_Def left_toggle ;
toggle_Def right_toggle ;


void DR16_Data_Process(void)
{
    //
    if (DR16_Data.Left_Switch == shang)
    {
        manual_state = manual_error;
    }
    else
    {
        manual_state = manual_OK;
    }
}