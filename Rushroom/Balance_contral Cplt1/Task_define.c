//
// Created by WXR on 2025/12/26.
//

#include "Task_define.h"
#include "panel.h"


void task_1ms(void const * argument)
{
    for(;;)
    {
        input_total();
        lqr_pid_parallel ();
        vmc();
        speed_calculate ();
        out_of_contral_detection ();
        jump_control ();
        main_output();
        osDelay(1);
    }
}

void task_100ms(void const * argument)
{
    for(;;)
    {
        remote_controller_outline_detection();
        can_outline_detection();
        gyroscope_outline_detection();
        emergency();
        osDelay(100);
    }
}