//
// Created by WXR on 2025/12/26.
//

#include "Virtual_Chassis_Control.h"

/**
 * @attention 严查lqr矩阵是否正确
 */
//extern char output_test2 ;
// char test1 = 3;
// char lqr_lun_test = 1;
void lqr()
{
    // if (lqr_lun_test == 0)
    // {
    //     x.x_lun = 0;
    //     x.d_x_lun = 0;
    // }
    // if (test1 == 1)
    // {
    //     left_u.tao_lun = - k.k21 * x.left_theta - k.k22 * x.left_d_theta - k.k23 * x.x_lun - k.k24 * x.d_x_lun - k.k25 * x.alpha - k.k26 * x.d_alpha ;
    //     left_u.tao_shen = - k.k11 * x.left_theta - k.k12 * x.left_d_theta - k.k13 * x.x_lun - k.k14 * x.d_x_lun - k.k15 * x.alpha - k.k16 * x.d_alpha ;
    // }
    // if (test1 == 2)
    // {
    //     right_u.tao_lun = - k.k21 * x.right_theta - k.k22 * x.right_d_theta - k.k23 * x.x_lun - k.k24 * x.d_x_lun - k.k25 * x.alpha - k.k26 * x.d_alpha ;
    //     right_u.tao_shen = - k.k11 * x.right_theta - k.k12 * x.right_d_theta - k.k13 * x.x_lun - k.k14 * x.d_x_lun - k.k15 * x.alpha - k.k16 * x.d_alpha ;
    // }
    // if (test1 == 3)
    // {
        left_u.tao_lun = - k.k21 * x.left_theta - k.k22 * x.left_d_theta - k.k23 * x.x_lun - k.k24 * x.d_x_lun - k.k25 * x.alpha - k.k26 * x.d_alpha ;
        left_u.tao_shen = - k.k11 * x.left_theta - k.k12 * x.left_d_theta - k.k13 * x.x_lun - k.k14 * x.d_x_lun - k.k15 * x.alpha - k.k16 * x.d_alpha ;
        right_u.tao_lun = - k.k21 * x.right_theta - k.k22 * x.right_d_theta - k.k23 * x.x_lun - k.k24 * x.d_x_lun - k.k25 * x.alpha - k.k26 * x.d_alpha ;
        right_u.tao_shen = - k.k11 * x.right_theta - k.k12 * x.right_d_theta - k.k13 * x.x_lun - k.k14 * x.d_x_lun - k.k15 * x.alpha - k.k16 * x.d_alpha ;
    // }
}

float error_angle = 0;
float yaw_state = 0;
float calculate_yaw = 0;

void yaw_pid()
{
    if (yaw_target > 180)
    {
        yaw_target = -180 + (yaw_target - 180);
    }
    if (yaw_target < -180)
    {
        yaw_target = 180 + (yaw_target + 180);
    }

    if (yaw_target > 0 && yaw < 0 && yaw < (yaw_target - 180))
    {
        error_angle = (180 - yaw_target) + (yaw + 180);
    }
    else if (yaw_target < 0 && yaw > 0 && yaw > (yaw_target + 180))
    {
        error_angle = -((180+yaw_target) + (180 - yaw));
    }
    else
    {
        error_angle = yaw - yaw_target ;
    }

    if (d_yaw > 10 || d_yaw < -10)
    {
        yaw_state = 1;
    }
    if (d_yaw > -1 && d_yaw < 1 && yaw_state == 1)
    {
        yaw_state = 0;
        yaw_target = yaw ;
    }

    calculate_yaw = (CAN_1.velocity[2]+CAN_2.velocity[2]);
    //预留偏航控制函数
    location_pid(Kp_yaw,Ki_yaw,Kd_yaw,0,error_angle,&yaw_controller,&yaw_error,&yaw_accumulated_error,&yaw_pre_error,yaw_I_MAX,yaw_controller_MAX);


}

void roll_pid()
{
    location_pid(Kp_roll,Ki_roll,Kd_roll,roll_target,-pitch,&roll_controller,&roll_error,&roll_accumulated_error,&roll_pre_error,roll_I_MAX,roll_controller_MAX);
}

/**
 * @brief pid与lqr并联
 * @attention 正负符号需确认
 */
// char linshi = 0 ;
void lqr_pid_parallel ()
{
    if (lie_down == 0)
    {
        lqr();
    }
    yaw_pid();


    // if (linshi == 0)
    // {
        wheel_torque.left = left_u.tao_lun - yaw_controller ;
        wheel_torque.right = right_u.tao_lun + yaw_controller ;
    // }
    // if (linshi == 1)
    // {
    //     wheel_torque.left = left_u.tao_lun - yaw_controller ;
    //     wheel_torque.right = right_u.tao_lun + yaw_controller ;
    // }
    // if (linshi == 2)
    // {
    //     wheel_torque.left = -left_u.tao_lun + yaw_controller ;
    //     wheel_torque.right = -right_u.tao_lun - yaw_controller ;
    // }
    // if (linshi == 3)
    // {
    //     wheel_torque.left = -left_u.tao_lun - yaw_controller ;
    //     wheel_torque.right = -right_u.tao_lun + yaw_controller ;
    // }
}
