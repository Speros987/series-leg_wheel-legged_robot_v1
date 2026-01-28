#ifndef DM_CAN_IO_H
#define DM_CAN_IO_H

#include "main.h"
#include "panel.h"

#define P_MIN (-3.14159)		//位置最小值
#define P_MAX 3.14159		//位置最大值
#define V_MIN (-30)			//速度最小值
#define V_MAX 30			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN (-10)			//转矩最大值
#define T_MAX 10			//转矩最小值

typedef struct
{
    int p_int[8],v_int[8],t_int[8];						//这里可根据电机数目自行修改，读取三个电机的位置、速度、转矩
    float position[8],velocity[8],torque[8];	        //三个电机的位置、速度、转矩解析存储
    uint8_t  Tx_Data[8];								//数据发送存储
    uint8_t  RxData[8];									//数据接收存储
    CAN_RxHeaderTypeDef Rx_pHeader;
    uint8_t CAN_flag[8];
    uint8_t CAN_pre_flag[8];
    uint8_t CAN_state[8];
}CANx_t;

extern CANx_t CAN_1,CAN_2;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
void Enable_Motor(CAN_HandleTypeDef* hcan,uint16_t id);
void Disable_Motor(CAN_HandleTypeDef* hcan,uint16_t id);
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq);
void can_outline_detection (void) ;

#endif
