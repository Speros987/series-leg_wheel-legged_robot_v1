#include "panel.h"

float DT = 0.001f ;   //控制周期

//常量参数
float m_lun = 0.386f ;       //轮子质量
float m_tui = 0.43484f ;       //腿质量
float m_shen = 1.50836f ;      //身体质量
float R_lun = 0.375f ;       //轮子半径
float L_luntui = 0.04397f ;    //轮子到腿重心距离
float L_shentui = 0.07603f ;   //腿到身体重心距离
float L_shen = 0.00439f ;      //身体重心到腿连接点距离
float I_lun = 0.000178f ;       //轮子转动惯量
float I_tui = 0.003478f ;       //腿转动惯量
float I_shen = 0.00812693445f ;      //身体转动惯量
float g = 9.81f;           //重力加速度
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
State_Vector_Def x ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};                    //状态向量
d_State_Vector_Def d_x ={0,0,0,0,0,0,};                //无扰动趋势向量
Total_d_State_Vector_Def total_d_x ={0,0,0,0,0,0,};    //总趋势向量
Input_Vector_Def left_u ={0,0,};                    //模拟输入向量
Input_Vector_Def right_u ={0,0,};                    //模拟输入向量
Hip_Torque_Def hip_torque ={0,0,0,0,};             //髋关节力矩向量
Wheel_Torque_Def wheel_torque ={0,0,};       //轮子力矩向量
J_Matrix_Def J ={0,0,0,0};                        //雅可比矩阵
K_Matrix_Def k ={39.6182f,8.8756f,10.1111f,49.8532f,0.0866f,-0.2025f,-4.5613f,-0.7230f,-0.6320f,-3.1730f,1.1102f,0.1760f,};     //k矩阵
Leg_State_Vector_Def left_leg = {0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.15f,0.0f,0.15f,0.0f}  ;
Leg_State_Vector_Def right_leg = {0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f , 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.15f,0.0f,0.15f,0.0f}  ;
Off_Ground_Detection_State_Vector_Def left_leg_off_ground_detection_state_vector = {0} ;
Off_Ground_Detection_State_Vector_Def right_leg_off_ground_detection_state_vector = {0} ;



/**
 * pid控制参数及寄存器
 */
float Kp_yaw = 0.008f ;     //yaw控制参数
float Ki_yaw = 0.00001f ;
float Kd_yaw = 0.02f ;

float yaw_target = 0.0f;
float yaw_controller;
float yaw_error;
float yaw_accumulated_error;
float yaw_pre_error;
float yaw_I_MAX = 1.0f ;
float yaw_controller_MAX = 0.2f;

float Kp_roll = 0.0f ;     //roll控制参数
float Ki_roll = 0.0f ;
float Kd_roll = 0.0f ;
float roll_target = 0.0f;
float roll_controller;
float roll_error;
float roll_accumulated_error;
float roll_pre_error;
float roll_I_MAX = 0.0f ;
float roll_controller_MAX = 0.19f;

float Kp_vmc = 16000 ;          //vmc控制参数
float Kd_vmc = -1000 ;

/**
 * @brief 总输入函数
 * @bug 正反方向未确定，角度转换未确定，未标定零点
 */
float x_lun ;
float d_x_lun ;
float d_lun_filter = 0.01f;
float d_yaw ;
float d_yaw_filter = 0.1f ;
void input_total(void)
{
    left_leg.alpha = 3.141593f + CAN_1.position[1] ;
    left_leg.beta = CAN_1.position[0] ;
    left_leg.d_alpha = CAN_1.velocity[1] ;
    left_leg.d_beta = CAN_1.velocity[0] ;
    right_leg.alpha = 3.141593f - CAN_2.position[1] ;
    right_leg.beta = -CAN_2.position[0] ;
    right_leg.d_alpha = -CAN_2.velocity[1] ;
    right_leg.d_beta = -CAN_2.velocity[0] ;

    d_x_lun = ((CAN_1.velocity[2] - CAN_2.velocity[2])/2)*d_lun_filter*6.283185f/360.0f*0.375f + x.d_x_lun*(1-d_lun_filter) ;   //轮子速度&低通滤波
    x.d_x_lun = -x.d_x_target + d_x_lun ;    //轮速误差
    x_lun += d_x_lun*DT;  //所在世界坐标
    x.x_lun = -x.x_target + x_lun ;  //轮子位置误差
    x.pre_alpha = x.alpha ;
    x.alpha = roll*6.283185f/360.0f ;
    x.d_alpha = (x.alpha - x.pre_alpha)*1000 ;

    d_yaw = gyro[2]*d_yaw_filter + d_yaw*(1-d_yaw_filter);
}
