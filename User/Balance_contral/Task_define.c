//
// Created by WXR on 2025/12/26.
//

#include "Task_define.h"
#include "panel.h"

char task1 = 1 ;
// char buzzer1 = 0 ;
char lie_down  =0;
void task_1ms(void const * argument)
{
    for(;;)
    {
        if (lie_down == 0)
        {
            if (task1 == 1)
            {
                input_total();
                vmc();
                lqr_pid_parallel ();
                out_of_contral_detection();
                main_output();
            }
            if (task1 == 2)
            {
                // input_total();
                // vmc();
                // main_output();
            }
            if (task1 == 3)
            {
                x.left_x_lun =0;       //单位：m
                x.left_d_x_lun =0;     //单位：m/s
                x.left_theta =0;       //单位：rad
                //保留原始数据
                x.left_sin_theta =0;
                x.left_cos_theta =0;
                x.left_pre_theta =0;
                x.left_d_theta =0;     //单位：rad/s

                x.right_x_lun =0;       //单位：m
                x.right_d_x_lun =0;     //单位：m/s
                x.right_theta =0;       //单位：rad
                //保留原始数据
                x.right_sin_theta =0;
                x.right_cos_theta =0;
                x.right_pre_theta =0;
                x.right_d_theta =0;     //单位：rad/s

                x.x_lun =0;       // 误差距离 单位：m
                x.d_x_lun =0;     // 误差速度 单位：m/s
                x.x_target =0;
                x.d_x_target =0;
                x.theta =0;       //单位：rad
                //保留原始数据
                x.sin_theta =0;
                x.cos_theta =0;
                x.pre_theta =0;
                x.d_theta =0;     //单位：rad/s

                x.alpha =0;       //单位：rad
                x.pre_alpha =0;
                x.d_alpha =0;     //单位：rad/s

                hip_torque.left_front_hip_torque = 0 ;
                hip_torque.left_back_hip_torque = 0 ;
                hip_torque.right_back_hip_torque = 0 ;
                hip_torque.right_front_hip_torque = 0 ;

                wheel_torque.left = 0;
                wheel_torque.right = 0;

                left_u.tao_lun = 0;
                left_u.tao_shen = 0;
                right_u.tao_lun = 0;
                right_u.tao_shen = 0;
            }
        }

        if (lie_down == 1)
        {
            Lie_Down();
        }
        osDelay(1);
    }
}

void task_100ms(void const * argument)
{
    for(;;)
    {
        can_outline_detection();
        gyroscope_outline_detection();
        emergency();
        osDelay(100);
        HT_10A_outline_Detection();
    }
}
