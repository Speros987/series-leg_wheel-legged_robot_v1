//
// Created by WXR on 2025/12/26.
//

#ifndef SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_STATE_DEFINE_H
#define SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_STATE_DEFINE_H

typedef enum
{
    manual_error =0,
    manual_OK =1,
}manual_state_Def;

typedef enum
{
    controller_error =0,
    controller_OK =1,
}controller_state_Def;

typedef enum
{
    chassis_error =0,
    chassis_OK =1,
}chassis_state_Def;

typedef enum
{
    gyroscope_error =0,
    gyroscope_OK =1,
}gyroscope_state_Def;

typedef enum
{
    unbalanced =0,
    balanced =1,
}balance_state_Def;

typedef enum
{
    is_skid  =0,
    not_skid =1,
}skid_state_Def;

typedef enum
{
    height_finished  =0,
    height_unfinished =1,
} height_finish_state_Def;

typedef enum
{
    height_high  =0,
    height_low =1,
} height_state_Def;

typedef enum
{
    is_off_ground =0,
    not_off_ground =1,
}off_ground_state_Def;

typedef enum
{
    idle,
    ready,
    start,
    jump,
    land,

    stumble,
    turtle,
}jump_state_Def;

typedef enum
{
    is_stumble =0,
    not_stumble =1,
}stumble_state_Def;

extern controller_state_Def controller_state ;
extern chassis_state_Def chassis_state ;
extern manual_state_Def manual_state ;
extern gyroscope_state_Def gyroscope_state ;
extern balance_state_Def balance_state ;
extern skid_state_Def left_skid_state ;
extern skid_state_Def right_skid_state ;
extern height_finish_state_Def height_finish_state ;
extern height_state_Def height_state ;
extern off_ground_state_Def left_off_ground_state ;
extern off_ground_state_Def right_off_ground_state ;
extern jump_state_Def jump_state ;
extern stumble_state_Def left_stumble_state ;
extern stumble_state_Def rightstumble_state ;

#endif //SERIES_LEG_WHEEL_LEGGED_ROBOT_V1_STATE_DEFINE_H