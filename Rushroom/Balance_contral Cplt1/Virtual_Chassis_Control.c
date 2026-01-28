//
// Created by WXR on 2025/12/26.
//

#include "Virtual_Chassis_Control.h"

/**
 * @attention 严查lqr矩阵是否正确
 */
char test1 = 3;
void lqr()
{
    if (test1 == 1)
    {
        left_u.tao_lun = k.k21 * x.left_theta + k.k22 * x.left_d_theta + k.k23 * x.left_x_lun + k.k24 * x.left_d_x_lun+ k.k25 * x.alpha + k.k26 * x.d_alpha ;
        left_u.tao_shen = k.k11 * x.left_theta + k.k12 * x.left_d_theta + k.k13 * x.left_x_lun + k.k14 * x.left_d_x_lun+ k.k15 * x.alpha + k.k16 * x.d_alpha ;
    }
    if (test1 == 2)
    {
        right_u.tao_lun = k.k21 * x.right_theta + k.k22 * x.right_d_theta + k.k23 * x.right_x_lun + k.k24 * x.right_d_x_lun+ k.k25 * x.alpha + k.k26 * x.d_alpha ;
        right_u.tao_shen = k.k11 * x.right_theta + k.k12 * x.right_d_theta + k.k13 * x.right_x_lun + k.k14 * x.right_d_x_lun+ k.k15 * x.alpha + k.k16 * x.d_alpha ;
    }
    if (test1 == 3)
    {
        left_u.tao_lun = k.k21 * x.left_theta + k.k22 * x.left_d_theta + k.k23 * x.left_x_lun + k.k24 * x.left_d_x_lun+ k.k25 * x.alpha + k.k26 * x.d_alpha ;
        left_u.tao_shen = k.k11 * x.left_theta + k.k12 * x.left_d_theta + k.k13 * x.left_x_lun + k.k14 * x.left_d_x_lun+ k.k15 * x.alpha + k.k16 * x.d_alpha ;
        right_u.tao_lun = k.k21 * x.right_theta + k.k22 * x.right_d_theta + k.k23 * x.right_x_lun + k.k24 * x.right_d_x_lun+ k.k25 * x.alpha + k.k26 * x.d_alpha ;
        right_u.tao_shen = k.k11 * x.right_theta + k.k12 * x.right_d_theta + k.k13 * x.right_x_lun + k.k14 * x.right_d_x_lun+ k.k15 * x.alpha + k.k16 * x.d_alpha ;
    }
}

void yaw_pid()
{
    //预留偏航控制函数
    location_pid(Kp_yaw,Ki_yaw,Kd_yaw,yaw_target,yaw,&yaw_controller,&yaw_error,&yaw_accumulated_error,&yaw_pre_error);
}

void roll_pid()
{
    location_pid(Kp_roll,Ki_roll,Kd_roll,roll_target,roll,&roll_controller,&roll_error,&roll_accumulated_error,&roll_pre_error);
}

/**
 * @brief pid与lqr并联
 * @attention 正负符号需确认
 */
char linshi = 0 ;
void lqr_pid_parallel ()
{
    lqr();
    yaw_pid();

    if (linshi == 0)
    {
        wheel_torque.left = left_u.tao_lun + yaw_controller ;
        wheel_torque.right = right_u.tao_lun - yaw_controller ;
    }
    if (linshi == 1)
    {
        wheel_torque.left = left_u.tao_lun - yaw_controller ;
        wheel_torque.right = right_u.tao_lun + yaw_controller ;
    }
    if (linshi == 2)
    {
        wheel_torque.left = -left_u.tao_lun + yaw_controller ;
        wheel_torque.right = -right_u.tao_lun - yaw_controller ;
    }
    if (linshi == 3)
    {
        wheel_torque.left = -left_u.tao_lun - yaw_controller ;
        wheel_torque.right = -right_u.tao_lun + yaw_controller ;
    }
}