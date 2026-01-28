//
// Created by WXR on 2026/1/10.
//

#include "Main_Output.h"

void main_output()
{
    //限幅
    if (hip_torque.left_front_hip_torque > 1.0f)
    {
        hip_torque.left_front_hip_torque = 1.0f;
    }
    if (hip_torque.left_front_hip_torque < -1.0f)
    {
        hip_torque.left_front_hip_torque = -1.0f;
    }

    if (hip_torque.left_back_hip_torque > 1.0f)
    {
        hip_torque.left_back_hip_torque = 1.0f;
    }
    if (hip_torque.left_back_hip_torque < -1.0f)
    {
        hip_torque.left_back_hip_torque = -1.0f;
    }

    if (hip_torque.right_front_hip_torque > 1.0f)
    {
        hip_torque.right_front_hip_torque = 1.0f;
    }
    if (hip_torque.right_front_hip_torque < -1.0f)
    {
        hip_torque.right_front_hip_torque = -1.0f;
    }

    if (hip_torque.right_back_hip_torque > 1.0f)
    {
        hip_torque.right_back_hip_torque = 1.0f;
    }
    if (hip_torque.right_back_hip_torque < -1.0f)
    {
        hip_torque.right_back_hip_torque = -1.0f;
    }

    if (wheel_torque.left > 0.5f)
    {
        wheel_torque.left = 0.5f;
    }
    if (wheel_torque.left < -0.5f)
    {
        wheel_torque.left = -0.5f;
    }

    if (wheel_torque.right > 0.5f)
    {
        wheel_torque.right = 0.5f;
    }
    if (wheel_torque.right < -0.5f)
    {
        wheel_torque.right = -0.5f;
    }

    //髋关节输出
    if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    {
        MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, hip_torque.left_front_hip_torque);
        MIT_CtrlMotor(&hcan1, 0x02, 0, 0, 0, 0, hip_torque.left_back_hip_torque);
        MIT_CtrlMotor(&hcan2, 0x01, 0, 0, 0, 0, hip_torque.right_front_hip_torque);
        MIT_CtrlMotor(&hcan2, 0x02, 0, 0, 0, 0, hip_torque.right_back_hip_torque);
    }
    else
    {
        MIT_CtrlMotor(&hcan1, 0x01, 0, 0, 0, 0, 0);
        MIT_CtrlMotor(&hcan1, 0x02, 0, 0, 0, 0, 0);
        MIT_CtrlMotor(&hcan2, 0x01, 0, 0, 0, 0, 0);
        MIT_CtrlMotor(&hcan2, 0x02, 0, 0, 0, 0, 0);
    }

    //轮毂输出
    if (manual_state == manual_OK && gyroscope_state == gyroscope_OK)
    {
        MIT_CtrlMotor(&hcan1, 0x03, 0, 0, 0, 0, wheel_torque.left);
        MIT_CtrlMotor(&hcan2, 0x03, 0, 0, 0, 0, wheel_torque.right);
    }
    else
    {
        MIT_CtrlMotor(&hcan1, 0x03, 0, 0, 0, 0, 0);
        MIT_CtrlMotor(&hcan2, 0x03, 0, 0, 0, 0, 0);
    }
}