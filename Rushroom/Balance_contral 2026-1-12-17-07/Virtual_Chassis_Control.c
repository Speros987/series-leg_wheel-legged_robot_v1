//
// Created by WXR on 2025/12/26.
//

#include "Virtual_Chassis_Control.h"

/**
 * @attention 严查lqr矩阵是否正确
 */
void lqr()
{
    u.tao_lun = k.k21 * x.theta + k.k22 * x.d_theta + k.k23 * x.x_lun + k.k24 * x.d_x_lun+ k.k25 * x.alpha + k.k26 * x.d_alpha ;
    u.tao_shen = k.k11 * x.theta + k.k12 * x.d_theta + k.k13 * x.x_lun + k.k14 * x.d_x_lun+ k.k15 * x.alpha + k.k16 * x.d_alpha ;
}

void yaw_control ()
{
    //预留偏航控制函数
    location_pid(Kp_yaw,Ki_yaw,Kd_yaw,yaw_target,yaw,&yaw_controller,&yaw_error,&yaw_accumulated_error,&yaw_pre_error);
}

/**
 * @brief pid与lqr并联
 * @attention 正负符号需确认
 */
void lqr_pid_parallel ()
{
    lqr();
    yaw_control ();
    wheel_torque.left = u.tao_lun + yaw_controller ;
    wheel_torque.right = u.tao_lun - yaw_controller ;
}