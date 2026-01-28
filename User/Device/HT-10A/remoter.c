#include "remoter.h"


uint8_t Rx_Data[BUFF_SIZE];

DMA_HandleTypeDef hdma_uart3_rx;

int Remoter_disconnected_times;
uint8_t Error_state = 0;

SBUS_CH_Struct SBUS_CH;
HT_10A_Def HT_10A;

unsigned char a[4] = {0};
unsigned char b[4] = {0};
float c ;
float d ;

float x_error = -0.244 ;
float x_error_forecast = 0.02f;

//目前没加模式改变瞬间
void Sbus_Data_Count(uint8_t *buf)
{
    if (buf[23] == 0 && buf[0] == 0x0F)
    {
		Remoter_disconnected_times = 0;
        SBUS_CH.ConnectState = 1;
        SBUS_CH.CH1 = ((int16_t)buf[ 1] >> 0 | ((int16_t)buf[ 2] << 8 )) & 0x07FF;
        SBUS_CH.CH2 = ((int16_t)buf[ 2] >> 3 | ((int16_t)buf[ 3] << 5 )) & 0x07FF;
        SBUS_CH.CH3 = ((int16_t)buf[ 3] >> 6 | ((int16_t)buf[ 4] << 2 ) | (int16_t)buf[ 5] << 10 ) & 0x07FF;
        SBUS_CH.CH4 = ((int16_t)buf[ 5] >> 1 | ((int16_t)buf[ 6] << 7 )) & 0x07FF;
        SBUS_CH.CH5 = ((int16_t)buf[ 6] >> 4 | ((int16_t)buf[ 7] << 4 )) & 0x07FF;
        SBUS_CH.CH6 = ((int16_t)buf[ 7] >> 7 | ((int16_t)buf[ 8] << 1 ) | (int16_t)buf[9] << 9 ) & 0x07FF;
        SBUS_CH.CH7 = ((int16_t)buf[ 9] >> 2 | ((int16_t)buf[10] << 6 )) & 0x07FF;
        SBUS_CH.CH8 = ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3 )) & 0x07FF;
        SBUS_CH.CH9 = ((int16_t)buf[12] << 0 | ((int16_t)buf[13] << 8 )) & 0x07FF;
        SBUS_CH.CH10 = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5 )) & 0x07FF;
        SBUS_CH.CH11 = ((int16_t)buf[14] >> 6 | ((int16_t)buf[15] << 2 ) | (int16_t)buf[16] << 10 ) & 0x07FF;
        SBUS_CH.CH12 = ((int16_t)buf[16] >> 1 | ((int16_t)buf[17] << 7 )) & 0x07FF;
        SBUS_CH.CH13 = ((int16_t)buf[17] >> 4 | ((int16_t)buf[18] << 4 )) & 0x07FF;
        SBUS_CH.CH14 = ((int16_t)buf[18] >> 7 | ((int16_t)buf[19] << 1 ) | (int16_t)buf[20] << 9 ) & 0x07FF;
        SBUS_CH.CH15 = ((int16_t)buf[20] >> 2 | ((int16_t)buf[21] << 6 )) & 0x07FF;
        SBUS_CH.CH16 = ((int16_t)buf[21] >> 5 | ((int16_t)buf[22] << 3 )) & 0x07FF;

    	HT_10A.left_x = (float)(SBUS_CH.CH4 - 992) / 800 ;
    	HT_10A.left_y = (float)(SBUS_CH.CH3 - 992) / 800 ;
    	HT_10A.right_x = (float)(SBUS_CH.CH1 - 992) / 800 ;
    	HT_10A.right_y = (float)(SBUS_CH.CH2 - 992) / 800 ;
    	switch (SBUS_CH.CH5)
    	{
    		case 192:
    			HT_10A.switch_left_3 = shang_3;
    			break;
    		case 992:
    			HT_10A.switch_left_3 = zhong_3;
    			break;
    		case 1792:
    			HT_10A.switch_left_3 = xia_3;
    			break;
    		default:
    			HT_10A.switch_left_3 = error_3;
    			break;
    	}
    	switch (SBUS_CH.CH6)
    	{
    		case 192:
    			HT_10A.switch_left_2 = shang_2;
    			break;
    		case 1792:
    			HT_10A.switch_left_2 = xia_2;
    			break;
    		default:
    			HT_10A.switch_left_2 = error_2;
    			break;
    	}
    	switch (SBUS_CH.CH7)
    	{
    		case 192:
    			HT_10A.switch_right_2 = shang_2;
    			break;
    		case 1792:
    			HT_10A.switch_right_2 = xia_2;
    			break;
    		default:
    			HT_10A.switch_right_2 = error_2;
    			break;
    	}
    	switch (SBUS_CH.CH8)
    	{
    		case 192:
    			HT_10A.switch_right_3 = shang_3;
    			break;
    		case 992:
    			HT_10A.switch_right_3 = zhong_3;
    			break;
    		case 1792:
    			HT_10A.switch_right_3 = xia_3;
    			break;
    		default:
    			HT_10A.switch_right_3 = error_3;
    			break;
    	}
    	HT_10A.left_knob = SBUS_CH.CH9 - 192;
    	HT_10A.right_knob = SBUS_CH.CH10 - 192;
    }
    else
    {
		Remoter_disconnected_times ++;
		if(Remoter_disconnected_times >= 50)
      	SBUS_CH.ConnectState = 0;
    }
}

float x_acceleration = 0.0008f;
float yaw_acceleration = -0.7f;
char x_state = 0;
char x_stop = 0;
char control_state = 0;
char ai_x_accelerate = 0;
void control_code()
{
	if (SBUS_CH.ConnectState == 0 || HT_10A.switch_left_2 == error_2 || HT_10A.switch_left_2 == shang_2)
	{
		manual_state = manual_error;
	}
	else
	{
		manual_state = manual_OK;
	}
	if (HT_10A.switch_left_3 == zhong_3 )
	{
		task1 = 1;
	}
	if (HT_10A.switch_left_3 == shang_3 )
	{
		task1 = 3;
	}
	if (HT_10A.switch_left_3 ==xia_3 && (x.d_x_lun < 0.001f && x.d_x_lun > -0.001f))
	{
		lie_down = 1;
	}

	x.d_x_target = HT_10A.left_y * x_acceleration ;
	if (x.d_x_target > 0.3f)
	{
		x.d_x_target = 0.3f;
	}
	if (x.d_x_target < -0.3f)
	{
		x.d_x_target = -0.3f;
	}

	if ((HT_10A.left_y < 0.05f) && (HT_10A.left_y > -0.05f))
	{
		x_state = 0 ;
	}
	else if (HT_10A.left_y > 0.05f)
	{
		x_state = 1;
		x_lun = x_error-x_error_forecast;
		x_stop = 0;
	}
	else if (HT_10A.left_y < -0.05f)
	{
		x_state = 1;
		x_lun = x_error+x_error_forecast;
		x_stop = 0;
	}

	if ((x_state == 0 && (x.d_x_lun < 0.001f && x.d_x_lun > -0.001f)) && x_stop == 0)
	{
		x_lun = x_error;
		x_stop = 1;
	}
	yaw_target += HT_10A.right_x * yaw_acceleration ;
}


void Remoter_Init()
{
		SBUS_CH.CH1 = 992;
		SBUS_CH.CH2 = 992;
		SBUS_CH.CH3 = 992;
		SBUS_CH.CH4 = 992;
		SBUS_CH.CH5 = 992;
		SBUS_CH.CH6 = 192;
		SBUS_CH.CH7 = 1792;
		SBUS_CH.CH8 = 992;
		SBUS_CH.CH9 = 192;
		SBUS_CH.CH10 = 992;
		SBUS_CH.SW3 = 1;
	
		Error_state = 0;
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rx_Data, BUFF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_uart3_rx, DMA_IT_HT);
}

uint16_t HT_10A_flag = 0;
uint16_t HT_10A_pre_flag = 0;
uint8_t HT_10A_state = 0;
//下板单独调试遥控器接收
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART3)
	{
		// 接收完毕后重启
		HAL_UART_Transmit_DMA(&huart6, Rx_Data, Size);
		Sbus_Data_Count(Rx_Data);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rx_Data, BUFF_SIZE);
		control_code();
		HT_10A_flag++;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == USART3)
	{
		__HAL_UNLOCK(huart);
		memset(Rx_Data, 0, BUFF_SIZE);							   // 清除接收缓存		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rx_Data, BUFF_SIZE);// 接收发生错误后重启
	}
}

void HT_10A_outline_Detection()
{
	if (HT_10A_flag != HT_10A_pre_flag)
	{
		HT_10A_state = 1;
	}
	else
	{
		HT_10A_state = 0;
		if (manual_state == manual_OK)
		{
			car_state=1 ;
		}

		Disable_Motor(&hcan1, 1);
		Disable_Motor(&hcan1, 2);
		Disable_Motor(&hcan1, 3);
		Disable_Motor(&hcan2, 1);
		Disable_Motor(&hcan2, 2);
		Disable_Motor(&hcan2, 3);
	}

	HT_10A_pre_flag = HT_10A_flag;

	if (SBUS_CH.ConnectState == 0)
	{
		HT_10A_state = 0;
		car_state=1 ;

		Disable_Motor(&hcan1, 1);
		Disable_Motor(&hcan1, 2);
		Disable_Motor(&hcan1, 3);
		Disable_Motor(&hcan2, 1);
		Disable_Motor(&hcan2, 2);
		Disable_Motor(&hcan2, 3);
	}
}