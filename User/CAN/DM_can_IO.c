#include "DM_can_IO.h"

CANx_t CAN_1,CAN_2;

/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/// converts unsigned int to float, given range and number of bits ///
	 float span = x_max - x_min;
	 float offset = x_min;
	 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{
	 /// Converts a float to an unsigned int, given range and number of bits///
	 float span = x_max - x_min;
	 float offset = x_min;
	 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief  电机使能
 */
void Enable_Motor(CAN_HandleTypeDef* hcan,uint16_t id)
{

	static CAN_TxHeaderTypeDef   Tx_Header;

	Tx_Header.StdId=id;
	Tx_Header.IDE=CAN_ID_STD;
	Tx_Header.RTR=CAN_RTR_DATA;
	Tx_Header.DLC=0x08;

	CANx_t CAN ;
	if (hcan->Instance==CAN1)
	{
		CAN = CAN_1;
	}
	if (hcan->Instance==CAN2)
	{
		CAN = CAN_2;
	}
	CAN.Tx_Data[0] = 0xFF;
	CAN.Tx_Data[1] = 0xFF;
	CAN.Tx_Data[2] = 0xFF;
	CAN.Tx_Data[3] = 0xFF;
	CAN.Tx_Data[4] = 0xFF;
	CAN.Tx_Data[5] = 0xFF;
	CAN.Tx_Data[6] = 0xFF;
	CAN.Tx_Data[7] = 0xFC;

	//寻空邮箱发送数据
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN.Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN.Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN.Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
		}
	}
}

/**
 * @brief  电机失能
 */
void Disable_Motor(CAN_HandleTypeDef* hcan,uint16_t id)
{
	static CAN_TxHeaderTypeDef   Tx_Header;

	Tx_Header.StdId=id;
	Tx_Header.IDE=CAN_ID_STD;
	Tx_Header.RTR=CAN_RTR_DATA;
	Tx_Header.DLC=0x08;

	CANx_t CAN ;
	if (hcan->Instance==CAN1)
	{
		CAN = CAN_1;
	}
	if (hcan->Instance==CAN2)
	{
		CAN = CAN_2;
	}
	CAN.Tx_Data[0] = 0xFF;
	CAN.Tx_Data[1] = 0xFF;
	CAN.Tx_Data[2] = 0xFF;
	CAN.Tx_Data[3] = 0xFF;
	CAN.Tx_Data[4] = 0xFF;
	CAN.Tx_Data[5] = 0xFF;
	CAN.Tx_Data[6] = 0xFF;
	CAN.Tx_Data[7] = 0xFD;

	//寻空邮箱发送数据
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN.Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN.Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN.Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
		}
	}
}


/**
 * @brief  MIT模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  id     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 * @param  _KP    位置比例系数
 * @param  _KD    位置微分系数
 * @param  _torq  转矩给定值
 */
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq)
{
	static CAN_TxHeaderTypeDef   Tx_Header;
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	Tx_Header.StdId=id;
	Tx_Header.IDE=CAN_ID_STD;
	Tx_Header.RTR=CAN_RTR_DATA;
	Tx_Header.DLC=0x08;

	CANx_t CAN ;
	if (hcan->Instance==CAN1)
	{
		CAN = CAN_1;
	}
	if (hcan->Instance==CAN2)
	{
		CAN = CAN_2;
	}
	CAN.Tx_Data[0] = (pos_tmp >> 8);
	CAN.Tx_Data[1] = pos_tmp;
	CAN.Tx_Data[2] = (vel_tmp >> 4);
	CAN.Tx_Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	CAN.Tx_Data[4] = kp_tmp;
	CAN.Tx_Data[5] = (kd_tmp >> 4);
	CAN.Tx_Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	CAN.Tx_Data[7] = tor_tmp;

	//寻空邮箱发送数据
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN.Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN.Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN.Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
		}
	}
}


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/**
 * CAN中断接受回调函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CANx_t *CAN ;

	if(hcan->Instance==CAN1)
	{
		CAN = &CAN_1;
	}
	if(hcan->Instance==CAN2)
	{
		CAN = &CAN_2;
	}

	if(HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&CAN->Rx_pHeader,CAN->RxData)==HAL_OK)//获取数据
	{
		int i = 0;
		switch (CAN->Rx_pHeader.StdId)
		{
			case 0X011:
			case 0X012:
			case 0X013:
			case 0X014:
			case 0X015:
			case 0X016:
			case 0X017:
			case 0X018:
				i = CAN->Rx_pHeader.StdId - 0X011;
				CAN->p_int[i]=(CAN->RxData[1]<<8)|CAN->RxData[2];
				CAN->v_int[i]=(CAN->RxData[3]<<4)|(CAN->RxData[4]>>4);
				CAN->t_int[i]=((CAN->RxData[4]&0xF)<<8)|CAN->RxData[5];
				CAN->position[i] = uint_to_float(CAN->p_int[i], P_MIN, P_MAX, 16); // (-12.5,12.5)
				CAN->velocity[i] = uint_to_float(CAN->v_int[i], V_MIN, V_MAX, 12); // (-45.0,45.0)
				CAN->torque[i] = uint_to_float(CAN->t_int[i], T_MIN, T_MAX, 12); // (-18.0,18.0)
				CAN->CAN_flag[i]++ ;
				break;
			default:
				break;
		}
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);	//激活CAN中断通知
	}

}

void can_outline_detection ()
{
	//预留CAN通信检测函数
	for (int i = 0; i < 3; i++)
	{
		if (CAN_1.CAN_flag[i] != CAN_1.CAN_pre_flag[i])
		{
			CAN_1.CAN_state[i] = 1;
		}
		else
		{
			CAN_1.CAN_state[i] = 0;
		}
		CAN_1.CAN_pre_flag[i] = CAN_1.CAN_flag[i];

		if (CAN_2.CAN_flag[i] != CAN_2.CAN_pre_flag[i])
		{
			CAN_2.CAN_state[i] = 1;
		}
		else
		{
			CAN_2.CAN_state[i] = 0;
		}
		CAN_2.CAN_pre_flag[i] = CAN_2.CAN_flag[i];
	}

	if (CAN_1.CAN_state[0] == 0 || CAN_1.CAN_state[1] == 0 || CAN_1.CAN_state[2] == 0 || CAN_2.CAN_state[0] == 0 || CAN_2.CAN_state[1] == 0 || CAN_2.CAN_state[2] == 0)
	{
		Disable_Motor(&hcan1, 1);
		Disable_Motor(&hcan1, 2);
		Disable_Motor(&hcan1, 3);
		Disable_Motor(&hcan2, 1);
		Disable_Motor(&hcan2, 2);
		Disable_Motor(&hcan2, 3);

		if (manual_state == manual_OK)
		{
			car_state=1 ;
		}
	}
}
