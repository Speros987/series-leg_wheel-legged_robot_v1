//
// Created by WXR on 2026/1/10.
//

#include "Main_Output.h"

// char output_test1 = 8 , output_test2 = 4 ;

void main_output()
{

#pragma region 髋关节输出


    //限幅
    if (hip_torque.left_front_hip_torque > 3.0f)
    {
        hip_torque.left_front_hip_torque = 3.0f;
    }
    if (hip_torque.left_front_hip_torque < -3.0f)
    {
        hip_torque.left_front_hip_torque = -3.0f;
    }

    if (hip_torque.left_back_hip_torque > 3.0f)
    {
        hip_torque.left_back_hip_torque = 3.0f;
    }
    if (hip_torque.left_back_hip_torque < -3.0f)
    {
        hip_torque.left_back_hip_torque = -3.0f;
    }

    if (hip_torque.right_front_hip_torque > 3.0f)
    {
        hip_torque.right_front_hip_torque = 3.0f;
    }
    if (hip_torque.right_front_hip_torque < -3.0f)
    {
        hip_torque.right_front_hip_torque = -3.0f;
    }

    if (hip_torque.right_back_hip_torque > 3.0f)
    {
        hip_torque.right_back_hip_torque = 3.0f;
    }
    if (hip_torque.right_back_hip_torque < -3.0f)
    {
        hip_torque.right_back_hip_torque = -3.0f;
    }
    //
    // if (output_test1 == 1)
    // {
    //     //髋关节输出
    //     if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    //     {
    //         MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, hip_torque.left_front_hip_torque);
    //         MIT_CtrlMotor(&hcan1, 0x02, 0, 0, 0, 0, -hip_torque.left_back_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x01, 0, 0, 0, 0, hip_torque.right_front_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x02, 0, 0, 0, 0, -hip_torque.right_back_hip_torque);
    //     }
    //     else
    //     {
    //         Disable_Motor(&hcan1, 0x01);
    //         Disable_Motor(&hcan1, 0x02);
    //         Disable_Motor(&hcan2, 0x01);
    //         Disable_Motor(&hcan2, 0x02);
    //     }
    // }
    // if (output_test1 == 2)
    // {
    //     //髋关节输出
    //     if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    //     {
    //         MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, hip_torque.left_front_hip_torque);
    //         MIT_CtrlMotor(&hcan1, 0x02, 0, 0, 0, 0, -hip_torque.left_back_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x01, 0, 0, 0, 0, -hip_torque.right_front_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x02, 0, 0, 0, 0, hip_torque.right_back_hip_torque);
    //     }
    //     else
    //     {
    //         Disable_Motor(&hcan1, 0x01);
    //         Disable_Motor(&hcan1, 0x02);
    //         Disable_Motor(&hcan2, 0x01);
    //         Disable_Motor(&hcan2, 0x02);
    //     }
    // }
    // if (output_test1 == 3)
    // {
    //     //髋关节输出
    //     if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    //     {
    //         MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, -hip_torque.left_front_hip_torque);
    //         MIT_CtrlMotor(&hcan1, 0x02, 0, 0, 0, 0, hip_torque.left_back_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x01, 0, 0, 0, 0, hip_torque.right_front_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x02, 0, 0, 0, 0, -hip_torque.right_back_hip_torque);
    //     }
    //     else
    //     {
    //         Disable_Motor(&hcan1, 0x01);
    //         Disable_Motor(&hcan1, 0x02);
    //         Disable_Motor(&hcan2, 0x01);
    //         Disable_Motor(&hcan2, 0x02);
    //     }
    // }
    // if (output_test1 == 4)
    // {
    //     //髋关节输出
    //     if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    //     {
    //         MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, -hip_torque.left_front_hip_torque);
    //         MIT_CtrlMotor(&hcan1, 0x02, 0, 0, 0, 0, hip_torque.left_back_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x01, 0, 0, 0, 0, -hip_torque.right_front_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x02, 0, 0, 0, 0, hip_torque.right_back_hip_torque);
    //     }
    //     else
    //     {
    //         Disable_Motor(&hcan1, 0x01);
    //         Disable_Motor(&hcan1, 0x02);
    //         Disable_Motor(&hcan2, 0x01);
    //         Disable_Motor(&hcan2, 0x02);
    //     }
    // }
    // if (output_test1 == 4)
    // {
    //     //髋关节输出
    //     if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    //     {
    //         MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, -hip_torque.left_front_hip_torque);
    //         MIT_CtrlMotor(&hcan1, 0x02, 0, 0, 0, 0, hip_torque.left_back_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x01, 0, 0, 0, 0, -hip_torque.right_front_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x02, 0, 0, 0, 0, hip_torque.right_back_hip_torque);
    //     }
    //     else
    //     {
    //         Disable_Motor(&hcan1, 0x01);
    //         Disable_Motor(&hcan1, 0x02);
    //         Disable_Motor(&hcan2, 0x01);
    //         Disable_Motor(&hcan2, 0x02);
    //     }
    // }
    // if (output_test1 == 5)
    // {
    //     //髋关节输出
    //     if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    //     {
    //         MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, hip_torque.left_front_hip_torque);
    //         MIT_CtrlMotor(&hcan1, 0x02, 0, 0, 0, 0, hip_torque.left_back_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x01, 0, 0, 0, 0, -hip_torque.right_front_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x02, 0, 0, 0, 0, -hip_torque.right_back_hip_torque);
    //     }
    //     else
    //     {
    //         Disable_Motor(&hcan1, 0x01);
    //         Disable_Motor(&hcan1, 0x02);
    //         Disable_Motor(&hcan2, 0x01);
    //         Disable_Motor(&hcan2, 0x02);
    //     }
    // }
    // if (output_test1 == 6)
    // {
    //     //髋关节输出
    //     if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    //     {
    //         MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, -hip_torque.left_front_hip_torque);
    //         MIT_CtrlMotor(&hcan1, 0x02, 0, 0, 0, 0, -hip_torque.left_back_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x01, 0, 0, 0, 0, hip_torque.right_front_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x02, 0, 0, 0, 0, hip_torque.right_back_hip_torque);
    //     }
    //     else
    //     {
    //         Disable_Motor(&hcan1, 0x01);
    //         Disable_Motor(&hcan1, 0x02);
    //         Disable_Motor(&hcan2, 0x01);
    //         Disable_Motor(&hcan2, 0x02);
    //     }
    // }
    // if (output_test1 == 7)
    // {
    //     //髋关节输出
    //     if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    //     {
    //         MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, hip_torque.left_front_hip_torque);
    //         MIT_CtrlMotor(&hcan1, 0x02, 0, 0, 0, 0, hip_torque.left_back_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x01, 0, 0, 0, 0, hip_torque.right_front_hip_torque);
    //         MIT_CtrlMotor(&hcan2, 0x02, 0, 0, 0, 0, hip_torque.right_back_hip_torque);
    //     }
    //     else
    //     {
    //         Disable_Motor(&hcan1, 0x01);
    //         Disable_Motor(&hcan1, 0x02);
    //         Disable_Motor(&hcan2, 0x01);
    //         Disable_Motor(&hcan2, 0x02);
    //     }
    // }
    // if (output_test1 == 8)
    // {
        //髋关节输出

    if (lie_down == 1 && lie_down_state == 0)
    {
        MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, 0);
        MIT_CtrlMotor(&hcan1, 0x02, 0, 0, 0, 0, 0);
        MIT_CtrlMotor(&hcan2, 0x01, 0, 0, 0, 0, 0);
        MIT_CtrlMotor(&hcan2, 0x02, 0, 0, 0, 0, 0);
    }
    else
    {
        if (manual_state == manual_OK && gyroscope_state == gyroscope_OK && car_state == 0)
        {
            MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, -hip_torque.left_front_hip_torque);
            MIT_CtrlMotor(&hcan1, 0x02, 0, 0, 0, 0, -hip_torque.left_back_hip_torque);
            MIT_CtrlMotor(&hcan2, 0x01, 0, 0, 0, 0, -hip_torque.right_front_hip_torque);
            MIT_CtrlMotor(&hcan2, 0x02, 0, 0, 0, 0, -hip_torque.right_back_hip_torque);
        }
        else
        {
            Disable_Motor(&hcan1, 0x01);
            Disable_Motor(&hcan1, 0x02);
            Disable_Motor(&hcan2, 0x01);
            Disable_Motor(&hcan2, 0x02);
        }
    }
    // }

#pragma endregion 髋关节输出

#pragma region 轮毂输出

    if (wheel_torque.left > 1.0f)
    {
        wheel_torque.left = 1.0f;
    }
    if (wheel_torque.left < -1.0f)
    {
        wheel_torque.left = -1.0f;
    }

    if (wheel_torque.right > 1.0f)
    {
        wheel_torque.right = 1.0f;
    }
    if (wheel_torque.right < -1.0f)
    {
        wheel_torque.right = -1.0f;
    }


    // if (output_test2 == 1)
    // {
    //     //轮毂输出
    //     if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    //     {
    //         MIT_CtrlMotor(&hcan1, 0x03, 0, 0, 0, 0, wheel_torque.left);
    //         MIT_CtrlMotor(&hcan2, 0x03, 0, 0, 0, 0, wheel_torque.right);
    //     }
    //     else
    //     {
    //         Disable_Motor(&hcan1, 0x03);
    //         Disable_Motor(&hcan2, 0x03);
    //     }
    // }
    //
    // if (output_test2 == 2)
    // {
    //     //轮毂输出
    //     if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    //     {
    //         MIT_CtrlMotor(&hcan1, 0x03, 0, 0, 0, 0, -wheel_torque.left);
    //         MIT_CtrlMotor(&hcan2, 0x03, 0, 0, 0, 0, -wheel_torque.right);
    //     }
    //     else
    //     {
    //         Disable_Motor(&hcan1, 0x03);
    //         Disable_Motor(&hcan2, 0x03);
    //     }
    // }
    //
    // if (output_test2 == 3)
    // {
    //     //轮毂输出
    //     if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    //     {
    //         MIT_CtrlMotor(&hcan1, 0x03, 0, 0, 0, 0, -wheel_torque.left);
    //         MIT_CtrlMotor(&hcan2, 0x03, 0, 0, 0, 0, wheel_torque.right);
    //     }
    //     else
    //     {
    //         Disable_Motor(&hcan1, 0x03);
    //         Disable_Motor(&hcan2, 0x03);
    //     }
    // }
    //
    // if (output_test2 == 4)
    // {
        //轮毂输出
        if (manual_state == manual_OK && gyroscope_state == gyroscope_OK && car_state == 0)
        {
            MIT_CtrlMotor(&hcan1, 0x03, 0, 0, 0, 0, wheel_torque.left);
            MIT_CtrlMotor(&hcan2, 0x03, 0, 0, 0, 0, -wheel_torque.right);
        }
        else
        {
            Disable_Motor(&hcan1, 0x03);
            Disable_Motor(&hcan2, 0x03);
        }
    // }
    // if (output_test2 == 5)
    // {
    //     MIT_CtrlMotor(&hcan1, 0x03, 0, 0, 0, 0, 0);
    //     MIT_CtrlMotor(&hcan2, 0x03, 0, 0, 0, 0, 0);
    // }
#pragma endregion 轮毂输出

}
