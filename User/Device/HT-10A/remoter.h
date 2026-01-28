#ifndef REMOTER_H
#define REMOTER_H

#include "main.h"
#include "state_define.h"
#include "panel.h"
#include "usart.h"
#include "string.h"
#include "Task_define.h"
#include "Lie_Down.h"
#include <stdint.h>
#include <stdio.h>

#define BUFF_SIZE	100

typedef enum
{
	shang_to_xia_3 =31,
	xia_3 =1,
	xia_to_zhong_3 =12,
	zhong_to_xia_3 =21,
	zhong_3 =2,
	zhong_to_shang_3 =23,
	shang_to_zhong_3 =32,
	shang_3 =3,
	xia_to_shang_3 =13,
	error_3 =0,
}Switch_3_Def;

typedef enum
{
	shang_to_xia_2 =21,
	xia_2 =1,
	shang_2 =2,
	xia_to_shang_2 =12,
	error_2 =0,
}Switch_2_Def;

typedef struct
{
	uint16_t CH1;	//右x 192-992-1792
	uint16_t CH2;	//右y
	uint16_t CH3;	//左y
	uint16_t CH4;	//左x
	uint16_t CH5;	//最左拨杆（上中下）
	uint16_t CH6;	//左拨杆（上下）
	uint16_t CH7;	//右拨杆（上下）
	uint16_t CH8;	//最右拨杆（上中下）
	uint16_t CH9;	//左旋钮（遥控器1000->192；2000->1792）
	uint16_t CH10;	//右旋钮（遥控器1000->192；2000->1792）
	uint16_t CH11;
	uint16_t CH12;
	uint16_t CH13;
	uint16_t CH14;
	uint16_t CH15;
	uint16_t CH16;
	uint8_t ConnectState;//遥控器与接收器连接状态 0=未连接，1=正常连接
	uint8_t SW1;
	uint8_t SW2;
	uint8_t SW3;
	uint8_t SW4;
	
}SBUS_CH_Struct;

typedef struct
{
	float left_x ;
	float left_y ;
	float right_x ;
	float right_y ;
	Switch_3_Def switch_left_3;
	Switch_2_Def switch_left_2;
	Switch_2_Def switch_right_2;
	Switch_3_Def switch_right_3;
	uint16_t left_knob ;
	uint16_t right_knob ;
}HT_10A_Def;

extern SBUS_CH_Struct SBUS_CH;
extern HT_10A_Def HT_10A;
extern uint8_t Rx_Data[BUFF_SIZE];
extern uint8_t up_board_data[128];

void control_code(void);
void Remoter_Init(void);
void HT_10A_outline_Detection(void);

#endif
