#include "panel.h"

float DT = 0.001f ;   //控制周期

//常量参数
float m_lun = 1.0f ;       //轮子质量
float m_tui = 1.0f ;       //腿质量
float m_shen = 1.0f ;      //身体质量
float R_lun = 0.1f ;       //轮子半径
float L_luntui = 1.0f ;    //轮子到腿重心距离
float L_shentui = 1.0f ;   //腿到身体重心距离
float L_shen = 1.0f ;      //身体重心到腿连接点距离
float I_lun = 1.0f ;       //轮子转动惯量
float I_tui = 1.0f ;       //腿转动惯量
float I_shen = 1.0f ;      //身体转动惯量
float g = 9.8f;           //重力加速度
float l_1 = 0.1f ;
float l_2 = 0.13f ;
float l_3 =0.13f ;
float l_4 = 0.1f ;
float l_5 =0.0f ;
float L_leg_max = 0.2f ;    //最大高度
float L_leg_min = 0.1f ;    //最小高度
float viscoelastic_force_max = 10.0f ;

float stumble_target = 360.0f ;   //摔跤检测阈值

/**
* 雅可比矩阵量
*/
float J_one = 0 ;
float J_two = 0 ;
float J_three = 0 ;
float J_four = 0 ;

/**
* 变量
*/
float viscoelastic_force_left = 0.0f ;       //vmc控制的左腿弹簧-阻尼力
float viscoelastic_force_right = 0.0f ;      //vmc控制的右腿弹簧-阻尼力
float body_speed_y = 0.0f ;                  //身体y方向速度

/**
* 向量
*/
State_Vector_Def x ={0,0,0,0,0,0,0,0,};                    //状态向量
d_State_Vector_Def d_x ={0,0,0,0,0,0,};                //无扰动趋势向量
Total_d_State_Vector_Def total_d_x ={0,0,0,0,0,0,};    //总趋势向量
Input_Vector_Def left_u ={0,0,};                    //模拟输入向量
Input_Vector_Def right_u ={0,0,};                    //模拟输入向量
Hip_Torque_Def hip_torque ={0,0,0,0,};             //髋关节力矩向量
Wheel_Torque_Def wheel_torque ={0,0,};       //轮子力矩向量
J_Matrix_Def J ={0,0,0,0};                        //雅可比矩阵
K_Matrix_Def k ={3.6205f,1.0699f,0.1166f,0.4486f,9.8959f,3.0320f,-13.4947f,-3.3962f,-0.9932f,-3.7394f,4.1013f,1.0180f,};     //k矩阵，未修改
Leg_State_Vector_Def left_leg = {0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.15f,0.0f,0.0f,0.0f}  ;
Leg_State_Vector_Def right_leg = {0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.15f,0.0f,0.0f,0.0f}  ;
Off_Ground_Detection_State_Vector_Def left_leg_off_ground_detection_state_vector = {0} ;
Off_Ground_Detection_State_Vector_Def right_leg_off_ground_detection_state_vector = {0} ;

/**
 * 手柄控制
 */
float target_d_displace = 0.0f ;
float target_yaw = 0.0f ;


/**
 * pid控制参数及寄存器
 */
float Kp_yaw = 0.0f ;     //yaw控制参数
float Ki_yaw = 0.0f ;
float Kd_yaw = 0.0f ;
float yaw_target = 0.0f;
float yaw_controller;
float yaw_error;
float yaw_accumulated_error;
float yaw_pre_error;

float Kp_roll = 0.0f ;     //roll控制参数
float Ki_roll = 0.0f ;
float Kd_roll = 0.0f ;
float roll_target = 0.0f;
float roll_controller;
float roll_error;
float roll_accumulated_error;
float roll_pre_error;

float Kp_vmc = 6000 ;          //vmc控制参数
float Kd_vmc = -2000 ;

/**
 * @brief 总输入函数
 * @bug 正反方向未确定，角度转换未确定，未标定零点
 */
void input_total(void)
{
    left_leg.alpha = 3.1415926f + CAN_1.position[1] ;
    left_leg.beta = CAN_1.position[0] ;
    left_leg.d_alpha = CAN_1.velocity[1] ;
    left_leg.d_beta = CAN_1.velocity[0] ;
    right_leg.alpha = 3.1415926f - CAN_2.position[1] ;
    right_leg.beta = -CAN_2.position[0] ;
    right_leg.d_alpha = -CAN_2.velocity[1] ;
    right_leg.d_beta = -CAN_2.velocity[0] ;


    x.d_x_lun  = (CAN_1.velocity[2] - CAN_2.velocity[2])/2 ;
    x.x_lun += x.d_x_lun*DT;
    x.pre_alpha = x.alpha ;
    x.alpha = roll ;
    x.d_alpha = gyro[1] ;
}