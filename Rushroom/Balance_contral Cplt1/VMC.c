//
// Created by WXR on 2025/12/26.
//

/***********************************************************************
 *
 * @brief 左右腿VMC算法
 * @bug 力矩未限幅
 *      高度调低时可能撞地，调pd
 *
 ***********************************************************************/

#include "VMC.h"

/**
 * vmc总函数
 * @brief 左右腿分别调用vmc函数
 */
void vmc()
{
    roll_pid();
    left_vmc();
    right_vmc();
    x.x_lun = (x.left_x_lun + x.right_x_lun)/2 ;
    x.d_x_lun = (x.d_x_lun + x.d_x_lun)/2 ;
    x.theta = (x.left_theta + x.right_theta)/2 ;
    x.pre_theta = (x.left_pre_theta + x.right_pre_theta)/2 ;
    x.d_theta = (x.left_d_theta + x.right_d_theta)/2;
}

/**
 * 左腿vmc
 * @brief 运动学正解解算j矩阵，按照期望B点力算出两髋关节力矩
 * @attention 没加d_theta的解算
 */
float left_sin_theta1_theta2 ;
float left_sin_alpha ;
float left_cos_alpha ;
float left_sin_beta  ;
float left_cos_beta  ;
float left_a ;
float left_b ;
float left_L_ac ;
float left_c ;
float left_tan_theta1_1_of_2 ;
float left_OB,left_sin_AOB,left_cos_AOB ;
float left_F_x,left_F_y ;
float left_d_OB ;
float left_cos_alpha_beta_2 ; //cos((α-β）/2)

void left_vmc()
{
    //通过运动学正解算出雅可比矩阵
    left_sin_alpha = arm_sin_f32(left_leg.alpha) ;
    left_cos_alpha = arm_cos_f32(left_leg.alpha) ;
    left_sin_beta = arm_sin_f32(left_leg.beta) ;
    left_cos_beta = arm_cos_f32(left_leg.beta) ;

    left_leg.x_a = l_1 * left_cos_alpha ;
    left_leg.y_a = l_1 * left_sin_alpha ;
    left_leg.x_c = l_5 + l_4 * left_cos_beta ;
    left_leg.y_c = l_4 * left_sin_beta ;

    left_a = 2 * (left_leg.x_a - left_leg.x_c) * l_2 ; //中间量
    left_b = 2* (left_leg.y_a - left_leg.y_c) * l_2 ;  //中间量
    left_L_ac = sqrtf(( left_leg.x_a - left_leg.x_c) * ( left_leg.x_a - left_leg.x_c) + ( left_leg.y_a - left_leg.y_c) * ( left_leg.y_a - left_leg.y_c)) ;
    left_c = l_3 * l_3 - l_2 * l_2 - left_L_ac * left_L_ac ;     //中间量

    left_tan_theta1_1_of_2 = (left_b + sqrtf(left_a*left_a + left_b*left_b - left_c*left_c))/(left_a + left_c);     //有错误的可能，没具体检查  tan(theta/2)
    left_leg.sin_theta_1 = 2*left_tan_theta1_1_of_2/(1 + left_tan_theta1_1_of_2*left_tan_theta1_1_of_2) ;
    left_leg.cos_theta_1 = (1 - left_tan_theta1_1_of_2*left_tan_theta1_1_of_2)/(1 + left_tan_theta1_1_of_2*left_tan_theta1_1_of_2) ;

    //中断一下，算b点
    left_leg.x_b = l_1*left_cos_alpha + l_2*left_leg.cos_theta_1 ;
    left_leg.y_b = l_1*left_sin_alpha + l_2*left_leg.sin_theta_1 ;

    left_leg.sin_theta_2 = (left_leg.x_c - left_leg.x_b) / l_3 ;
    left_leg.cos_theta_2 = (left_leg.y_c - left_leg.y_b) / l_3 ;

    //继续通过运动学正解算出雅可比矩阵
    left_sin_theta1_theta2 = left_leg.sin_theta_1*left_leg.cos_theta_2 - left_leg.sin_theta_2*left_leg.cos_theta_1 ;   //sin(theta1-theta2)

    J_one = l_1*left_leg.sin_theta_2*(left_sin_alpha*left_leg.cos_theta_1 - left_leg.sin_theta_1*left_cos_alpha)/left_sin_theta1_theta2;
    J_two = - 1*l_4*left_leg.sin_theta_1*(left_sin_beta*left_leg.cos_theta_2 - left_leg.sin_theta_2*left_cos_beta)/left_sin_theta1_theta2;
    J_three = - 1*l_1*left_leg.cos_theta_2*(left_sin_alpha*left_leg.cos_theta_1 - left_leg.sin_theta_1*left_cos_alpha)/left_sin_theta1_theta2;
    J_four = l_4*left_leg.cos_theta_1*(left_sin_beta*left_leg.cos_theta_2 - left_leg.sin_theta_2*left_cos_beta)/left_sin_theta1_theta2;

    //五连杆虚拟化，算Fx,Fy

    left_cos_alpha_beta_2 = arm_cos_f32((left_leg.alpha-left_leg.beta)/2) ;
    left_leg.pre_L_tui = left_leg.L_tui ;
    left_leg.L_tui = l_1*left_cos_alpha_beta_2 + sqrtf(l_1*l_1*left_cos_alpha_beta_2*left_cos_alpha_beta_2 - (l_1*l_1 - l_2*l_2)) ;

    x.left_pre_theta = x.left_theta ;
    x.left_theta = 3.1415926f/2 - left_leg.beta/2 - left_leg.alpha/2 ;
    x.left_d_theta = - left_leg.d_alpha/2 - left_leg.d_beta/2 ;
    x.left_cos_theta = arm_cos_f32(x.left_theta) ;
    x.left_sin_theta = arm_sin_f32(x.left_theta) ;

    if (jump_state != start)
    {
        viscoelastic_force_left = Kp_vmc * ( left_leg.L_tui_target - left_leg.L_tui ) + Kd_vmc * ( left_leg.L_tui - left_leg.pre_L_tui ) + roll_controller ;
    }
    if (jump_state == start)
    {
        viscoelastic_force_left = viscoelastic_force_max ;
    }

    left_F_x = left_u.tao_shen*x.left_cos_theta/left_leg.L_tui + viscoelastic_force_left*x.left_sin_theta ;
    left_F_y = -left_u.tao_shen*x.left_sin_theta/left_leg.L_tui +viscoelastic_force_left*x.left_cos_theta ;

    //Fx,Fy映射到髋关节电机力矩
    hip_torque.left_front_hip_torque = J_one*left_F_x + J_two*left_F_y ;
    hip_torque.left_back_hip_torque = J_three*left_F_x + J_four*left_F_y ;
}

/**
 * 右腿vmc
 * @brief 运动学正解解算j矩阵，按照期望B点力算出两髋关节力矩
 * @attention 没加d_theta的解算
 */
float right_sin_theta1_theta2 ;
float right_sin_alpha ;
float right_cos_alpha ;
float right_sin_beta  ;
float right_cos_beta  ;
float right_a ;
float right_b ;
float right_L_ac ;
float right_c ;
float right_tan_theta1_1_of_2 ;
float right_OB,right_sin_AOB,right_cos_AOB ;
float right_F_x,right_F_y ;
float right_d_OB ;
float right_cos_alpha_beta_2 ; //cos((α-β）/2)

void right_vmc()
{
    //通过运动学正解算出雅可比矩阵
    right_sin_alpha = arm_sin_f32(right_leg.alpha) ;
    right_cos_alpha = arm_cos_f32(right_leg.alpha) ;
    right_sin_beta = arm_sin_f32(right_leg.beta) ;
    right_cos_beta = arm_cos_f32(right_leg.beta) ;

    right_leg.x_a = l_1 * right_cos_alpha ;
    right_leg.y_a = l_1 * right_sin_alpha ;
    right_leg.x_c = l_5 + l_4 * right_cos_beta ;
    right_leg.y_c = l_4 * right_sin_beta ;

    right_a = 2 * (right_leg.x_a - right_leg.x_c) * l_2 ; //中间量
    right_b = 2* (right_leg.y_a - right_leg.y_c) * l_2 ;  //中间量
    right_L_ac = sqrtf(( right_leg.x_a - right_leg.x_c) * ( right_leg.x_a - right_leg.x_c) + ( right_leg.y_a - right_leg.y_c) * ( right_leg.y_a - right_leg.y_c)) ;
    right_c = l_3 * l_3 - l_2 * l_2 - right_L_ac * right_L_ac ;     //中间量

    right_tan_theta1_1_of_2 = (right_b + sqrtf(right_a*right_a + right_b*right_b - right_c*right_c))/(right_a + right_c);     //有错误的可能，没具体检查  tan(theta/2)
    right_leg.sin_theta_1 = 2*right_tan_theta1_1_of_2/(1 + right_tan_theta1_1_of_2*right_tan_theta1_1_of_2) ;
    right_leg.cos_theta_1 = (1 - right_tan_theta1_1_of_2*right_tan_theta1_1_of_2)/(1 + right_tan_theta1_1_of_2*right_tan_theta1_1_of_2) ;

    //中断一下，算b点
    right_leg.x_b = l_1*right_cos_alpha + l_2*right_leg.cos_theta_1 ;
    right_leg.y_b = l_1*right_sin_alpha + l_2*right_leg.sin_theta_1 ;

    right_leg.sin_theta_2 = (right_leg.x_c - right_leg.x_b) / l_3 ;
    right_leg.cos_theta_2 = (right_leg.y_c - right_leg.y_b) / l_3 ;

    //继续通过运动学正解算出雅可比矩阵
    right_sin_theta1_theta2 = right_leg.sin_theta_1*right_leg.cos_theta_2 - right_leg.sin_theta_2*right_leg.cos_theta_1 ;   //sin(theta1-theta2)

    J_one = l_1*right_leg.sin_theta_2*(right_sin_alpha*right_leg.cos_theta_1 - right_leg.sin_theta_1*right_cos_alpha)/right_sin_theta1_theta2;
    J_two = - 1*l_4*right_leg.sin_theta_1*(right_sin_beta*right_leg.cos_theta_2 - right_leg.sin_theta_2*right_cos_beta)/right_sin_theta1_theta2;
    J_three = - 1*l_1*right_leg.cos_theta_2*(right_sin_alpha*right_leg.cos_theta_1 - right_leg.sin_theta_1*right_cos_alpha)/right_sin_theta1_theta2;
    J_four = l_4*right_leg.cos_theta_1*(right_sin_beta*right_leg.cos_theta_2 - right_leg.sin_theta_2*right_cos_beta)/right_sin_theta1_theta2;

    //五连杆虚拟化，算Fx,Fy

    right_cos_alpha_beta_2 = arm_cos_f32((right_leg.alpha-right_leg.beta)/2) ;
    right_leg.pre_L_tui = right_leg.L_tui ;
    right_leg.L_tui = l_1*right_cos_alpha_beta_2 + sqrtf(l_1*l_1*right_cos_alpha_beta_2*right_cos_alpha_beta_2 - (l_1*l_1 - l_2*l_2)) ;

    x.right_pre_theta = x.right_theta ;
    x.right_theta = 3.1415926f/2 - right_leg.beta/2 - right_leg.alpha/2 ;
    x.right_d_theta = - right_leg.d_alpha/2 - right_leg.d_beta/2 ;
    x.right_cos_theta = arm_cos_f32(x.right_theta) ;
    x.right_sin_theta = arm_sin_f32(x.right_theta) ;

    if (jump_state != start)
    {
        viscoelastic_force_right = Kp_vmc * ( right_leg.L_tui_target - right_leg.L_tui ) + Kd_vmc * ( right_leg.L_tui - right_leg.pre_L_tui ) - roll_controller ;
    }
    if (jump_state == start)
    {
        viscoelastic_force_right = viscoelastic_force_max ;
    }

    right_F_x = right_u.tao_shen*x.right_cos_theta/right_leg.L_tui + viscoelastic_force_right*x.right_sin_theta ;
    right_F_y = -right_u.tao_shen*x.right_sin_theta/right_leg.L_tui +viscoelastic_force_right*x.right_cos_theta ;

    //Fx,Fy映射到髋关节电机力矩
    hip_torque.right_front_hip_torque = -(J_one*right_F_x + J_two*right_F_y) ;
    hip_torque.right_back_hip_torque = -(J_three*right_F_x + J_four*right_F_y) ;
}