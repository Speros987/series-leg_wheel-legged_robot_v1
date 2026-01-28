#ifndef PANEL_H
#define PANEL_H

#include "DM_can_IO.h"
#include "imu_temp_ctrl.h"

extern float DT ;   //控制周期

/**
 * 常数参数
 */
extern float m_lun ;
extern float m_tui ;
extern float m_shen ;
extern float R_lun ;
extern float L_luntui ;
extern float L_shentui ;
extern float L_shen ;
extern float I_lun ;
extern float I_tui ;
extern float I_shen ;
extern float g ;
extern float l_1 ;
extern float l_2 ;
extern float l_3 ;
extern float l_4 ;
extern float l_5 ;
extern float L_leg_max ;
extern float L_leg_min ;
extern float viscoelastic_force_max ;

extern float stumble_target ;

/**
 * 手柄控制
 */
extern float target_d_displace ;
extern float target_yaw ;

/**
 * pid控制参数及寄存器
 */
extern float Kp_yaw ;
extern float Ki_yaw ;
extern float Kd_yaw ;
extern float yaw_target ;
extern float yaw_controller;
extern float yaw_error;
extern float yaw_accumulated_error;
extern float yaw_pre_error;
extern float yaw_I_MAX ;
extern float yaw_controller_MAX ;

extern float Kp_roll ;
extern float Ki_roll ;
extern float Kd_roll ;
extern float roll_target ;
extern float roll_controller;
extern float roll_error;
extern float roll_accumulated_error;
extern float roll_pre_error;
extern float roll_I_MAX ;
extern float roll_controller_MAX ;

extern float Kp_vmc ;
extern float Kd_vmc ;

/**
* 变量
*/
extern float viscoelastic_force_left ;
extern float viscoelastic_force_right ;
extern float body_speed_y ;

/**
* 雅可比矩阵量
*/
extern float J_one ;
extern float J_two ;
extern float J_three ;
extern float J_four ;

/**
 * 状态向量
 */
typedef struct
{
    float left_x_lun ;       //单位：m
    float left_d_x_lun ;     //单位：m/s
    float left_theta ;       //单位：rad
        //保留原始数据
        float left_sin_theta ;
        float left_cos_theta ;
    float left_pre_theta ;
    float left_d_theta ;     //单位：rad/s

    float right_x_lun ;       //单位：m
    float right_d_x_lun ;     //单位：m/s
    float right_theta ;       //单位：rad
    //保留原始数据
    float right_sin_theta ;
    float right_cos_theta ;
    float right_pre_theta ;
    float right_d_theta ;     //单位：rad/s

    float x_lun ;       // 误差距离 单位：m
    float d_x_lun ;     // 误差速度 单位：m/s
    float x_target ;
    float d_x_target ;
    float theta ;       //单位：rad
    //保留原始数据
    float sin_theta ;
    float cos_theta ;
    float pre_theta ;
    float d_theta ;     //单位：rad/s

    float alpha ;       //单位：rad
    float pre_alpha ;
    float d_alpha ;     //单位：rad/s
} State_Vector_Def ;


/**
 * 无干扰趋势向量
 */
typedef struct
{
    float d_x_lun ;     //单位：m/s
    float dd_x_lun ;    //单位：m/s^2
    float d_theta ;     //单位：rad/s
    float dd_theta ;    //单位：rad/s^2
    float d_alpha ;     //单位：rad/s
    float dd_alpha ;    //单位：rad/s^2
} d_State_Vector_Def ;

/**
 * 总趋势向量
 */
typedef struct
{
    float d_x_lun ;     //单位：m/s
    float dd_x_lun ;    //单位：m/s^2
    float d_theta ;     //单位：rad/s
    float dd_theta ;    //单位：rad/s^2
    float d_alpha ;     //单位：rad/s
    float dd_alpha ;    //单位：rad/s^2
} Total_d_State_Vector_Def ;

/**
 * 腿五连杆状态向量
 */
typedef struct
{
    //原始参数
    float alpha ;
    float d_alpha ;
    float beta ;
    float d_beta ;

    //中间参数
    float x_a ;
    float y_a ;
    float x_b ;
    float y_b ;
    float x_c ;
    float y_c ;
    float sin_theta_1 ;
    float cos_theta_1 ;
    float theta_1 ;
    float d_theta_1 ;
    float pre_theta_1 ;
    float sin_theta_2 ;
    float cos_theta_2 ;
    float theta_2 ;
    float d_theta_2 ;
    float pre_theta_2 ;

    //目标参数
    float L_tui_target ;
    float L_tui ;         //腿长
    float pre_L_tui ;
    float d_L_tui ;
} Leg_State_Vector_Def;

/**
 * 模拟输入向量
 */
typedef struct
{
    float tao_lun ;         //单位：N?m
    float tao_shen;         //单位：N?m
} Input_Vector_Def ;

/**
 * 髋关节力矩
 */
typedef struct
{
    float left_front_hip_torque ;       //单位：N?m
    float left_back_hip_torque ;        //单位：N?m
    float right_front_hip_torque ;      //单位：N?m
    float right_back_hip_torque ;       //单位：N?m
} Hip_Torque_Def ;

typedef struct
{
    float left ;
    float right ;
}Wheel_Torque_Def;

/**
 * 雅可比矩阵
 */
typedef struct
{
    float one ;
    float two ;
    float three ;
    float four ;
} J_Matrix_Def ;

/**
 * K矩阵
 * @attention 还未修改
 */
typedef struct
{
    float k11 ;
    float k12 ;
    float k13 ;
    float k14 ;
    float k15 ;
    float k16 ;
    float k21 ;
    float k22 ;
    float k23 ;
    float k24 ;
    float k25 ;
    float k26 ;
} K_Matrix_Def ;

/**
 * 离地检测状态向量
 */
typedef struct
{
    float x_lun ;
    float y_lun ;
    float x_shen ;
    float y_shen ;
    float pre_x_lun ;
    float pre_y_lun ;

    float d_x_lun ;
    float d_y_lun ;
    float pre_d_x_lun ;
    float pre_d_y_lun ;

    float dd_x_lun ;
    float dd_y_lun ;
} Off_Ground_Detection_State_Vector_Def ;

extern State_Vector_Def x ;
extern d_State_Vector_Def d_x ;
extern Total_d_State_Vector_Def total_d_x ;
extern Input_Vector_Def left_u ;
extern Input_Vector_Def right_u ;
extern Hip_Torque_Def hip_torque ;
extern Wheel_Torque_Def wheel_torque ;
extern J_Matrix_Def J ;
extern K_Matrix_Def k ;
extern Leg_State_Vector_Def left_leg ;
extern Leg_State_Vector_Def right_leg ;
extern Off_Ground_Detection_State_Vector_Def left_leg_off_ground_detection_state_vector ;
extern Off_Ground_Detection_State_Vector_Def right_leg_off_ground_detection_state_vector ;
extern float x_lun ;
extern float d_x_lun ;
extern float d_lun_filter ;
extern float d_yaw ;
extern float d_yaw_filter ;

void input_total(void);

#endif 

