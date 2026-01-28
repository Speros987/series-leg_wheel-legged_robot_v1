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
    left_vmc();
    right_vmc();
}

/**
 * 左腿vmc
 * @brief 运动学正解解算j矩阵，按照期望B点力算出两髋关节力矩
 * @attention 没加d_theta的解算
 */
float sin_theta1_theta2 ;
float sin_alpha ;
float cos_alpha ;
float sin_beta  ;
float cos_beta  ;
float a ;
float b ;
float L_ac ;
float c ;
float tan_theta1_1_of_2 ;
float OB,sin_AOB,cos_AOB ;
float F_x,F_y ;
float d_OB ;

void left_vmc()
{
    //通过运动学正解算出雅可比矩阵
    sin_alpha = arm_sin_f32(left_leg.alpha) ;
    cos_alpha = arm_cos_f32(left_leg.alpha) ;
    sin_beta = arm_sin_f32(left_leg.beta) ;
    cos_beta = arm_cos_f32(left_leg.beta) ;

    left_leg.x_a = l_1 * cos_alpha ;
    left_leg.y_a = l_1 * sin_alpha ;
    left_leg.x_c = l_5 + l_4 * cos_beta ;
    left_leg.y_c = l_4 * sin_beta ;

    a = 2 * (left_leg.x_a - left_leg.x_c) * l_2 ; //中间量
    b = 2* (left_leg.y_a - left_leg.y_c) * l_2 ;  //中间量
    L_ac = sqrtf(( left_leg.x_a - left_leg.x_c) * ( left_leg.x_a - left_leg.x_c) + ( left_leg.y_a - left_leg.y_c) * ( left_leg.y_a - left_leg.y_c)) ;
    c = l_3 * l_3 - l_2 * l_2 - L_ac * L_ac ;     //中间量

    tan_theta1_1_of_2 = (b + sqrtf(a*a + b*b - c*c))/(a + c);     //有错误的可能，没具体检查  tan(theta/2)
    left_leg.sin_theta_1 = 2*tan_theta1_1_of_2/(1 + tan_theta1_1_of_2*tan_theta1_1_of_2) ;
    left_leg.cos_theta_1 = (1 - tan_theta1_1_of_2*tan_theta1_1_of_2)/(1 + tan_theta1_1_of_2*tan_theta1_1_of_2) ;

    //中断一下，算b点
    left_leg.x_b = l_1*cos_alpha + l_2*left_leg.cos_theta_1 ;
    left_leg.y_b = l_1*sin_alpha + l_2*left_leg.sin_theta_1 ;

    left_leg.sin_theta_2 = (left_leg.x_c - left_leg.x_b) / l_3 ;
    left_leg.cos_theta_2 = (left_leg.y_c - left_leg.y_b) / l_3 ;

    //继续通过运动学正解算出雅可比矩阵
    sin_theta1_theta2 = left_leg.sin_theta_1*left_leg.cos_theta_2 - left_leg.sin_theta_2*left_leg.cos_theta_1 ;   //sin(theta1-theta2)

    J_one = l_1*left_leg.sin_theta_2*(sin_alpha*left_leg.cos_theta_1 - left_leg.sin_theta_1*cos_alpha)/sin_theta1_theta2;
    J_two = - 1*l_4*left_leg.sin_theta_1*(sin_beta*left_leg.cos_theta_2 - left_leg.sin_theta_2*cos_beta)/sin_theta1_theta2;
    J_three = - 1*l_1*left_leg.cos_theta_2*(sin_alpha*left_leg.cos_theta_1 - left_leg.sin_theta_1*cos_alpha)/sin_theta1_theta2;
    J_four = l_4*left_leg.cos_theta_1*(sin_beta*left_leg.cos_theta_2 - left_leg.sin_theta_2*cos_beta)/sin_theta1_theta2;

    //五连杆虚拟化，算Fx,Fy
    OB= 0 ;
    sin_AOB= 0 ;
    cos_AOB = 0 ;

    OB = sqrtf(l_1*l_1+l_2*l_2+2*(left_leg.cos_theta_1*cos_alpha+left_leg.sin_theta_1*sin_alpha)*l_1*l_2) ;
    cos_AOB = (OB*OB + l_1*l_1 - l_2*l_2)/(2*l_1*OB) ;
    sin_AOB = -(left_leg.sin_theta_1*cos_alpha - left_leg.cos_theta_1*sin_alpha)*l_2/OB ;
    left_leg.L_tui = sqrtf(l_5*l_5/4 + OB*OB -l_5*OB*(cos_alpha*cos_AOB+sin_alpha*sin_AOB)) ;

    x.theta = 3.1415926f/2 - left_leg.beta/2 - left_leg.alpha/2 ;
    x.cos_theta = arm_cos_f32(x.theta) ;
    x.sin_theta = arm_sin_f32(x.theta) ;

    if (jump_state != start)
    {
        viscoelastic_force_left = Kp_vmc * ( left_leg.L_tui_target - left_leg.L_tui ) + Kd_vmc * ( left_leg.L_tui - left_leg.pre_L_tui ) ;
    }
    if (jump_state == start)
    {
        viscoelastic_force_left = viscoelastic_force_max ;
    }

    F_x = u.tao_shen*x.cos_theta/left_leg.L_tui + viscoelastic_force_left*x.sin_theta ;
    F_y = -u.tao_shen*x.sin_theta/left_leg.L_tui +viscoelastic_force_left*x.cos_theta ;

    //Fx,Fy映射到髋关节电机力矩
    hip_torque.left_front_hip_torque = J_one*F_x + J_two*F_y ;
    hip_torque.left_back_hip_torque = J_three*F_x + J_four*F_y ;

    //计算d_theta
    left_leg.d_theta_1 = (l_1*CAN_1.velocity[0] *(sin_alpha*left_leg.cos_theta_2 - left_leg.sin_theta_2*cos_alpha) + l_4*CAN_1.velocity[1]*(sin_beta*left_leg.cos_theta_1 - left_leg.sin_theta_1*cos_beta))/(l_2*sin_theta1_theta2) ;
    d_OB = (l_1*l_2*(left_leg.d_theta_1*(left_leg.cos_theta_1*sin_alpha - left_leg.sin_theta_1*cos_alpha) + CAN_1.velocity[0]*(left_leg.sin_theta_1*cos_alpha - sin_alpha*left_leg.cos_theta_1)))/(sqrt(l_1*l_1+l_2*l_2+2*(left_leg.cos_theta_1*cos_alpha+left_leg.sin_theta_1*sin_alpha)*l_1*l_2)) ;
    left_leg.d_L_tui = 1/(2*sqrt(l_5*l_5 + OB*OB - l_5*OB*(cos_alpha*cos_AOB)))*(2*OB*d_OB - l_5*(d_OB*(cos_alpha*cos_AOB) + OB*(sin_alpha*CAN_1.velocity[0]*cos_AOB + cos_alpha*(2*OB*d_OB*2*l_1*OB - (OB*OB + l_1*l_1 - l_2*l_2)*2*l_1*OB*OB)/(4*l_1*l_1*OB*OB)))) ;
    x.d_theta = (l_5*left_leg.L_tui*(2*left_leg.L_tui*left_leg.d_L_tui - 2*OB*d_OB) - (left_leg.L_tui*left_leg.L_tui - OB*OB)*l_5*left_leg.d_L_tui)/(l_5*l_5*left_leg.L_tui*left_leg.L_tui*x.cos_theta) ;
}

/**
 * 右腿vmc
 * @brief 运动学正解解算j矩阵，按照期望B点力算出两髋关节力矩
 * @attention 没加d_theta的解算
 */
/*
void right_vmc()
{
    //计算当前雅可比矩阵
    float sin_alpha = arm_sin_f32(right_leg.alpha) ;
    float cos_alpha = arm_cos_f32(right_leg.alpha) ;
    float sin_beta = arm_sin_f32(right_leg.beta) ;
    float cos_beta = arm_cos_f32(right_leg.beta) ;

    right_leg.x_a = l_1 * cos_alpha ;
    right_leg.y_a = l_1 * sin_alpha ;
    right_leg.x_c = l_5 + l_4 * cos_beta ;
    right_leg.y_c = l_4 * sin_beta ;

    float a = 2 * (right_leg.x_a - right_leg.x_c) * l_2 ;
    float b = 2* (right_leg.y_a - right_leg.y_c) * l_2 ;
    float L_ac = sqrtf(( right_leg.x_a - right_leg.x_c) * ( right_leg.x_a - right_leg.x_c) + ( right_leg.y_a - right_leg.y_c) * ( right_leg.y_a - right_leg.y_c)) ;
    float c = l_3 * l_3 - l_2 * l_2 - L_ac * L_ac ;

    float tan_theta1_1_of_2 = (b + sqrtf(a*a + b*b - c*c))/(a + c);     //有错误的可能，没具体检查
    right_leg.sin_theta_1 = 2*tan_theta1_1_of_2/(1 + tan_theta1_1_of_2*tan_theta1_1_of_2) ;
    right_leg.cos_theta_1 = (1 - tan_theta1_1_of_2*tan_theta1_1_of_2)/(1 + tan_theta1_1_of_2*tan_theta1_1_of_2) ;

    right_leg.x_b = l_1*cos_alpha + l_2*right_leg.cos_theta_1 ;
    right_leg.y_b = l_1*sin_alpha + l_2*right_leg.sin_theta_1 ;

    right_leg.sin_theta_2 = (right_leg.x_c - right_leg.x_b) / l_3 ;
    right_leg.cos_theta_2 = (right_leg.y_c - right_leg.y_b) / l_3 ;

    float sin_theta1_theta2 = right_leg.sin_theta_1*right_leg.cos_theta_2 - right_leg.sin_theta_2*right_leg.cos_theta_1 ;

    J_one = l_1*right_leg.sin_theta_2*(sin_alpha*right_leg.cos_theta_1 - right_leg.sin_theta_1*cos_alpha)/sin_theta1_theta2;
    J_two = - 1*l_4*right_leg.sin_theta_1*(sin_beta*right_leg.cos_theta_2 - right_leg.sin_theta_2*cos_beta)/sin_theta1_theta2;
    J_three = - 1*l_1*right_leg.cos_theta_2*(sin_alpha*right_leg.cos_theta_1 - right_leg.sin_theta_1*cos_alpha)/sin_theta1_theta2;
    J_four = l_4*right_leg.cos_theta_1*(sin_beta*right_leg.cos_theta_2 - right_leg.sin_theta_2*cos_beta)/sin_theta1_theta2;

    //运动学正解
    float OB,sin_AOB,cos_AOB = 0 ;
    OB = sqrtf(l_1*l_1+l_2*l_2+2*(right_leg.cos_theta_1*cos_alpha+right_leg.sin_theta_1*sin_alpha)*l_1*l_2) ;
    cos_AOB = (OB*OB + l_1*l_1 - l_2*l_2)/(2*l_1*OB) ;
    sin_AOB = -(right_leg.sin_theta_1*cos_alpha - right_leg.cos_theta_1*sin_alpha)*l_2/OB ;
    right_leg.L_tui = sqrtf(l_5*l_5/4 + OB*OB -l_5*OB*(cos_alpha*cos_AOB+sin_alpha*sin_AOB)) ;      //算腿长

    x.cos_theta = (sin_alpha*cos_AOB - sin_AOB*cos_alpha)*OB/right_leg.L_tui ;
    x.sin_theta = (l_5*l_5/4 + right_leg.L_tui*right_leg.L_tui - OB*OB)/(l_5*right_leg.L_tui) ;

    //极坐标
    x.theta = asinf(x.sin_theta) ;                                                                  //算腿角度

    if (jump_state != start)
    {
        viscoelastic_force_right = Kp_vmc * ( right_leg.L_tui_target - right_leg.L_tui ) + Kd_vmc * ( right_leg.L_tui - right_leg.pre_L_tui ) ;     //算力
    }
    if (jump_state == start)
    {
        viscoelastic_force_right = viscoelastic_force_max ;
    }

    //极坐标转换直角坐标
    float F_x,F_y ;
    F_x = u.tao_shen*x.cos_theta/right_leg.L_tui + viscoelastic_force_right*x.sin_theta ;
    F_y = -u.tao_shen*x.sin_theta/right_leg.L_tui +viscoelastic_force_right*x.cos_theta ;

    //直角坐标转换hip力矩
    hip_torque.right_front_hip_torque = J_one*F_x + J_two*F_y ;
    hip_torque.right_back_hip_torque = J_three*F_x + J_four*F_y ;
}
*/