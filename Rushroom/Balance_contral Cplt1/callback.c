/**
 * @file    callback.c
 * @brief   UART（遥控器）接收完成回调函数
 */


#include "usart.h"
#include "DR16.h"
#include "struct_typedef.h"
#include "Remote_Contral.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UART_RxCpltCallback(18);
	DR16_Data_Process();
}