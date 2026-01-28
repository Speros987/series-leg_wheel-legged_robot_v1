//
// Created by WXR on 2026/1/20.
//

#include "Lie_Down.h"

#include "my_maths.h"
#include "panel.h"

uint8_t lie_down_state = 0;
uint16_t lie_down_tick = 0;
uint16_t lie_down_time = 500;
float lie_down_theta = 0.0f;    //卧倒腿角
float lie_down_p = 0.0f;
float lie_down_i = 0.0f;
float lie_down_d = 0.0f;

float lie_down_controller;
float lie_down_error;
float lie_down_accumulated_error;
float lie_down_pre_error;
float lie_down_I_MAX  = 0.5f;
float lie_down_controller_MAX = 3.0f;
float lie_down_alpha = 0.0f ;   //卧倒身角

float lie_down_v_p,lie_down_v_i,lie_down_v_d,lie_down_v_error, lie_down_v_accumulated_error, lie_down_v_pre_error, lie_down_v_I_MAX,lie_down_v_controller_MAX = 0.8f;

float lie_down_v_target = 0;

float tao_lun;
float yaw_p;
float yaw_i;
float yaw_d;
float yaw_accelerate;
float yaw_tao;
float yaw_error_1;
float yaw_accumulated_error_1;
float yaw_pre_error_1;
float yaw_I_MAX_1;
float yaw_controller_MAX_1;
float pitch_p = 0.05f;
float pitch_i;
float pitch_d = 0.01f;
float pitch_target = 0;
float pitch_tao;
float pitch_error_1;
float pitch_accumulated_error_1;
float pitch_pre_error_1;
float pitch_I_MAX_1;
float pitch_controller_MAX_1 = 3;
float can11;
float can12;
float can13;
float can21;
float can22;
float can23;
float accelerate = 0.5f;
float yaw_p_accelerate = 1.0f;

void Lie_Down(void)
{
    x.x_target = 0.01f;
    x.d_x_target = 0.01f;
    x.alpha = 0.01f;
    x.d_alpha = 0.01f;
    x.x_lun = 0.01f;
    x.d_x_lun = 0.01f;
    x.alpha = 0.01f;
    x.d_alpha = 0.01f;
    x.x_lun = 0.01f;

    k.k11 = 0.01f;
    k.k12 = 0.01f;
    k.k13 = 0.01f;
    k.k14 = 0.01f;
    k.k15 = 0.01f;
    k.k16 = 0.01f;
    k.k21 = 0.01f;
    k.k22 = 0.01f;
    k.k23 = 0.01f;
    k.k24 = 0.01f;
    k.k25 = 0.01f;
    k.k26 = 0.01f;

    Kp_roll = 0.000001f ;     //roll控制参数
    Ki_roll = 0.000001f ;
    Kd_roll = 0.000001f ;
    roll_target = 0.000001f;
    roll_controller = 0.000001f;
    roll_error = 0.000001f;
    roll_accumulated_error = 0.000001f;
    roll_pre_error = 0.000001f;
    roll_I_MAX = 0.000001f ;
    roll_controller_MAX = 0.000001f;

    Kp_vmc = 0 ;          //vmc控制参数
    Kd_vmc = 0 ;


    //解算v_qian

    //状态
    if (lie_down_tick >= lie_down_time)
    {
        lie_down_state = 1 ;
    }

    if (lie_down_state == 0)
    {
        Disable_Motor(&hcan1,1);
        Disable_Motor(&hcan1,2);
        Disable_Motor(&hcan2,1);
        Disable_Motor(&hcan2,2);

        //前
        tao_lun = 0.8f ;
        tao_lun = 0.8f ;
        lie_down_tick++ ;

        //pid控yaw
        // location_pid(yaw_p, yaw_i, yaw_d, HT_10A.right_y , gyro[2], &yaw_tao,
        //              &yaw_error_1, &yaw_accumulated_error_1, &yaw_pre_error_1, yaw_I_MAX_1,
        //              yaw_controller_MAX_1);
        // yaw_tao=HT_10A.right_y*yaw_p;
        
        //解算加并联得到结果
        can13 = tao_lun + yaw_tao ;
        can23 = tao_lun - yaw_tao ;
    }
    else
    {
        lie_down_tick++ ;
        if (lie_down_tick < (lie_down_time+10))
        {
            Enable_Motor(&hcan1,1);
            Enable_Motor(&hcan1,2);
            Enable_Motor(&hcan2,1);
            Enable_Motor(&hcan2,2);
        }
        
        //前
        // location_pid(lie_down_v_p, lie_down_v_i, lie_down_v_d, HT_10A.left_y*accelerate , accel[1] , &tao_lun,
        //              &lie_down_v_error, &lie_down_v_accumulated_error, &lie_down_v_pre_error, lie_down_v_I_MAX,
        //              lie_down_v_controller_MAX);
        tao_lun = HT_10A.left_y*accelerate;
        //yaw
        // location_pid(yaw_p, yaw_i, yaw_d, HT_10A.right_x*yaw_accelerate , (CAN_1.velocity[2] - CAN_2.velocity[2]), &yaw_tao,
        //              &yaw_error_1, &yaw_accumulated_error_1, &yaw_pre_error_1, yaw_I_MAX_1,
        //              yaw_controller_MAX_1);
        yaw_tao = HT_10A.right_x*yaw_p_accelerate;
        //pitch
        location_pid(pitch_p, pitch_i, pitch_d, pitch_target, -roll, &pitch_tao,
                     &pitch_error_1, &pitch_accumulated_error_1, &pitch_pre_error_1, pitch_I_MAX_1,
                     pitch_controller_MAX_1);

        //并联解算
        //lun
        can13 = tao_lun + yaw_tao ;
        can23 = tao_lun - yaw_tao ;
        
        can12 = pitch_tao;
        can22 = pitch_tao ;
        can11 = 0;
        can21 = 0;
    }

    if (manual_state == manual_OK)
    {
        MIT_CtrlMotor(&hcan1,1,0,0,0,0, can11);
        MIT_CtrlMotor(&hcan1,2,0,0,0,0, can12);
        MIT_CtrlMotor(&hcan1,3,0,0,0,0, can13);
        MIT_CtrlMotor(&hcan2,1,0,0,0,0, -can21);
        MIT_CtrlMotor(&hcan2,2,0,0,0,0, -can22);
        MIT_CtrlMotor(&hcan2,3,0,0,0,0, -can23);
    }
    else
    {
        Disable_Motor(&hcan1, 1);
        Disable_Motor(&hcan1, 2);
        Disable_Motor(&hcan1, 3);
        Disable_Motor(&hcan2, 1);
        Disable_Motor(&hcan2, 2);
        Disable_Motor(&hcan2, 3);
    }
}
