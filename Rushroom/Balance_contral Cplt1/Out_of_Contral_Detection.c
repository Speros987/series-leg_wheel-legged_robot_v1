//
// Created by WXR on 2025/12/27.
//

/***********************************************************************
 *
 * @brief 失控检测
 * @bug 离地检测未加滤波，易受噪声影响
 *      只写了左轮，没写右轮
 *      轮重力没乘倾斜角
 *
 ***********************************************************************/

#include "Out_of_Contral_Detection.h"

/**
 * @brief 失控检测
 */
void out_of_contral_detection ()
{
    left_wheel_acceleration_detection ();
    right_wheel_acceleration_detection ();
    fall_detection();
}


/**
 * @brief 轮子加速度检测
 * @bug 离地检测未加滤波，易受噪声影响
 *      只写了左轮，没写右轮
 *      轮重力没乘倾斜角
 */
void left_wheel_acceleration_detection ()
{
    int x_flag = 0 , y_flag = 0 ,I_flag = 0;
    //预留轮子加速度检测函数
    float F1 = 0, F2 = 0;
    F2 = left_u.tao_lun/(L_luntui + L_shentui) ;
    F1 = viscoelastic_force_right ;

    float P = 0,N = 0;
    P = F1*x.cos_theta - F2*x.sin_theta ;
    N = F1*x.sin_theta + F2*x.cos_theta ;

    left_leg_off_ground_detection_state_vector.pre_x_lun = left_leg_off_ground_detection_state_vector.x_lun ;
    left_leg_off_ground_detection_state_vector.pre_y_lun = left_leg_off_ground_detection_state_vector.y_lun ;
    left_leg_off_ground_detection_state_vector.pre_d_x_lun = left_leg_off_ground_detection_state_vector.d_x_lun ;
    left_leg_off_ground_detection_state_vector.pre_d_y_lun = left_leg_off_ground_detection_state_vector.d_y_lun ;

    left_leg_off_ground_detection_state_vector.x_lun = left_leg_off_ground_detection_state_vector.x_shen + L_shen*sinf(x.alpha) - left_leg.L_tui*x.sin_theta ;
    left_leg_off_ground_detection_state_vector.y_lun = left_leg_off_ground_detection_state_vector.y_shen - L_shen*cosf(x.alpha) - left_leg.L_tui*x.cos_theta ;
    left_leg_off_ground_detection_state_vector.d_x_lun = (left_leg_off_ground_detection_state_vector.x_lun - left_leg_off_ground_detection_state_vector.pre_x_lun)/DT ;
    left_leg_off_ground_detection_state_vector.d_y_lun = (left_leg_off_ground_detection_state_vector.y_lun - left_leg_off_ground_detection_state_vector.pre_y_lun)/DT ;
    left_leg_off_ground_detection_state_vector.dd_x_lun = (left_leg_off_ground_detection_state_vector.d_x_lun - left_leg_off_ground_detection_state_vector.pre_d_x_lun)/DT ;
    left_leg_off_ground_detection_state_vector.dd_y_lun = (left_leg_off_ground_detection_state_vector.d_y_lun - left_leg_off_ground_detection_state_vector.pre_d_y_lun)/DT ;

    if((left_leg_off_ground_detection_state_vector.dd_x_lun -  (-N/m_lun) < (-N/m_lun)*0.1) || (left_leg_off_ground_detection_state_vector.dd_x_lun - (-N/m_lun) > -((-N)/m_lun)*0.1))
    {
        x_flag = 1 ;
    }
    else
    {
        x_flag = 0 ;
    }

    if((left_leg_off_ground_detection_state_vector.dd_y_lun - ((P + m_lun*g)/m_lun) < ((P + m_lun*g)/m_lun)*0.1) || (left_leg_off_ground_detection_state_vector.dd_y_lun - ((P + m_lun*g)/m_lun) > -((P + m_lun*g)/m_lun)*0.1))
    {
        y_flag = 1 ;
    }
    else
    {
        y_flag = 0 ;
    }

    static float pre_velocity = 0;
    pre_velocity = CAN_1.velocity[3] ;
    float d_velocity = (CAN_1.velocity[3] - pre_velocity)/DT ;
    if (((d_velocity - (CAN_1.torque[3]/I_lun)) < (CAN_1.torque[3]/I_lun)*0.1) || ((d_velocity - (CAN_1.torque[3]/I_lun)) > -(CAN_1.torque[3]/I_lun)*0.1))
    {
        I_flag = 1 ;
    }

    if (x_flag == 1 && y_flag == 1 && I_flag == 1)
    {
        left_off_ground_state = is_off_ground ;
    }
    else
    {
        left_off_ground_state = not_off_ground ;
    }

    if (x_flag == 1 && y_flag == 0 && I_flag == 1)
    {
        left_skid_state = is_skid ;
    }
    else
    {
        left_skid_state = not_skid ;
    }
}

/**
 * @brief 轮子加速度检测
 * @bug 离地检测未加滤波，易受噪声影响
 *      轮重力没乘倾斜角
 */
void right_wheel_acceleration_detection ()
{
    int x_flag = 0 , y_flag = 0 ,I_flag = 0;
    //预留轮子加速度检测函数
    float F1 = 0, F2 = 0;
    F2 = left_u.tao_lun/(L_luntui + L_shentui) ;
    F1 = viscoelastic_force_right ;

    float P = 0,N = 0;
    P = F1*x.cos_theta - F2*x.sin_theta ;
    N = F1*x.sin_theta + F2*x.cos_theta ;

    right_leg_off_ground_detection_state_vector.pre_x_lun = right_leg_off_ground_detection_state_vector.x_lun ;
    right_leg_off_ground_detection_state_vector.pre_y_lun = right_leg_off_ground_detection_state_vector.y_lun ;
    right_leg_off_ground_detection_state_vector.pre_d_x_lun = right_leg_off_ground_detection_state_vector.d_x_lun ;
    right_leg_off_ground_detection_state_vector.pre_d_y_lun = right_leg_off_ground_detection_state_vector.d_y_lun ;

    right_leg_off_ground_detection_state_vector.x_lun = right_leg_off_ground_detection_state_vector.x_shen + L_shen*sinf(x.alpha) - right_leg.L_tui*x.sin_theta ;
    right_leg_off_ground_detection_state_vector.y_lun = right_leg_off_ground_detection_state_vector.y_shen - L_shen*cosf(x.alpha) - right_leg.L_tui*x.cos_theta ;
    right_leg_off_ground_detection_state_vector.d_x_lun = (right_leg_off_ground_detection_state_vector.x_lun - right_leg_off_ground_detection_state_vector.pre_x_lun)/DT ;
    right_leg_off_ground_detection_state_vector.d_y_lun = (right_leg_off_ground_detection_state_vector.y_lun - right_leg_off_ground_detection_state_vector.pre_y_lun)/DT ;
    right_leg_off_ground_detection_state_vector.dd_x_lun = (right_leg_off_ground_detection_state_vector.d_x_lun - right_leg_off_ground_detection_state_vector.pre_d_x_lun)/DT ;
    right_leg_off_ground_detection_state_vector.dd_y_lun = (right_leg_off_ground_detection_state_vector.d_y_lun - right_leg_off_ground_detection_state_vector.pre_d_y_lun)/DT ;

    if((right_leg_off_ground_detection_state_vector.dd_x_lun -  (-N/m_lun) < (-N/m_lun)*0.1) || (right_leg_off_ground_detection_state_vector.dd_x_lun - (-N/m_lun) > -((-N)/m_lun)*0.1))
    {
        x_flag = 1 ;
    }
    else
    {
        x_flag = 0 ;
    }

    if((right_leg_off_ground_detection_state_vector.dd_y_lun - ((P + m_lun*g)/m_lun) < ((P + m_lun*g)/m_lun)*0.1) || (right_leg_off_ground_detection_state_vector.dd_y_lun - ((P + m_lun*g)/m_lun) > -((P + m_lun*g)/m_lun)*0.1))
    {
        y_flag = 1 ;
    }
    else
    {
        y_flag = 0 ;
    }

    static float pre_velocity = 0;
    pre_velocity = CAN_2.velocity[3] ;
    float d_velocity = (CAN_2.velocity[3] - pre_velocity)/DT ;
    if (((d_velocity - (CAN_2.torque[3]/I_lun)) < (CAN_2.torque[3]/I_lun)*0.1) || ((d_velocity - (CAN_2.torque[3]/I_lun)) > -(CAN_2.torque[3]/I_lun)*0.1))
    {
        I_flag = 1 ;
    }

    if (x_flag == 1 && y_flag == 1 && I_flag == 1)
    {
        right_off_ground_state = is_off_ground ;
    }
    else
    {
        right_off_ground_state = not_off_ground ;
    }

    if (x_flag == 1 && y_flag == 0 && I_flag == 1)
    {
        right_skid_state = is_skid ;
    }
    else
    {
        right_skid_state = not_skid ;
    }
}

/**
 * @brief 摔跤检测
 * @attention 选择垂直的陀螺仪轴作为检测轴
 *            仅仅为角加速度检测
 */
void fall_detection ()
{
    static int stumble_flag = 0;
    //预留摔跤检测函数
    if (gyro[2] > stumble_target || gyro[2] < -stumble_target)
    {
        stumble_flag ++;
    }
    else
    {
        stumble_flag = 0;
    }

    //五次累加检测
    if (stumble_flag > 5)
    {
        rightstumble_state = is_stumble ;
    }
    else
    {
        stable_detection ();
    }
}

/**
 * @brief 摔跤后稳定检测
 * @attention 摔跤后才调用
 */
void stable_detection ()
{
    if ( jump_state == stumble)
    {
        static int stable_flag = 0;
        if (fabsf(gyro[2]) < 50.0f && fabsf(accel[0]) < 0.5f && fabsf(accel[1]) < 0.5f && fabsf(accel[2]) < 0.5f)
        {
            stable_flag ++;
        }
        else
        {
            stable_flag = 0;
        }

        //十次累加检测
        if (stable_flag > 10)
        {
            rightstumble_state = not_stumble ;
        }
    }
}