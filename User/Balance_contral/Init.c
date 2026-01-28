//
// Created by WXR on 2025/12/26.
//

/***********************************************************************
 *
 * @brief 初始化
 *
 ***********************************************************************/

#include "Init.h"

/**
 * @brief 整车初始化函数
 */
void car_init ()
{
    motor_init();
}

/**
 * @brief 电机重启或初始化函数
 */
void motor_init()
{
    Enable_Motor(&hcan1,0x01);
    Enable_Motor(&hcan1,0x02);
    Enable_Motor(&hcan1,0x03);
    Enable_Motor(&hcan2,0x01);
    Enable_Motor(&hcan2,0x02);
    Enable_Motor(&hcan2,0x03);
}

// /**
//  * @brief 遥控器初始化函数
//  */
// void remote_controller_init()
// {
//
// }
