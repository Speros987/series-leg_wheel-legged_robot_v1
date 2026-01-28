//
// Created by WXR on 2025/12/27.
//

/***********************************************************************
 *
 * @brief 紧急情况处理函数
 * @attention 最高优先级
 *
 ***********************************************************************/

#include "Emergency.h"

#include "Init.h"

void emergency ()
{
    //底盘错误处理
    if ( chassis_state == chassis_error)
    {
        motor_init();
    }

    //遥控器错误处理
    if ( controller_state == controller_error)
    {

    }

    //手刹
    if ( manual_state == manual_error)
    {
        Disable_Motor(&hcan1, 1);
        Disable_Motor(&hcan1, 2);
        Disable_Motor(&hcan1, 3);
        Disable_Motor(&hcan2, 1);
        Disable_Motor(&hcan2, 2);
        Disable_Motor(&hcan2, 3);
    }
}
