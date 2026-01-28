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
    else
    {
        Enable_Motor(&hcan1, 1);
        Enable_Motor(&hcan1, 2);
        Enable_Motor(&hcan1, 3);
        Enable_Motor(&hcan2, 1);
        Enable_Motor(&hcan2, 2);
        Enable_Motor(&hcan2, 3);
    }

    if (car_state != 0)
    {
        while (1)
        {
            Disable_Motor(&hcan1, 1);
            Disable_Motor(&hcan1, 2);
            Disable_Motor(&hcan1, 3);
            Disable_Motor(&hcan2, 1);
            Disable_Motor(&hcan2, 2);
            Disable_Motor(&hcan2, 3);
        }
    }
}
