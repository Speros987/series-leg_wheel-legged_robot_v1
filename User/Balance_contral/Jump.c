// //
// // Created by WXR on 2025/12/27.
// //
//
// /***********************************************************************
//  *
//  * @brief 跳跃控制
//  *
//  ***********************************************************************/
//
// #include "Jump.h"
// int jump_state_jump_time = 0;
//
// void jump_control ()
// {
//     //预留跳跃控制函数
//
//     static int body_speed_down_finish_flag = 0;
//
//     {//跳跃状态机
//         if (jump_state == idle && height_finish_state == height_finished && left_skid_state == not_skid && height_state == height_low && left_off_ground_state == not_off_ground && balance_state == balanced)
//         {   //条件：处于跳跃空闲状态、低高度、高度调整完成、未滑行、未离地、已平衡
//             jump_state = ready ;     //跳跃状态进入跳跃准备状态，等待松开跳跃键
//         }
//
//         if (jump_state == ready && you_shang_button == User_define_not_pressed_to_is_pressed)
//         {   //条件：处于跳跃准备状态、跳跃按钮从松开到按下（仅触发一次）
//             jump_state = start ;     //跳跃状态进入跳跃启动状态，开始起跳（VMC中腿部模拟力调整为最大）
//         }
//
//         if(jump_state == start && left_off_ground_state == is_off_ground)
//         {   //条件：处于跳跃启动状态、已离地
//             jump_state = jump ;      //跳跃状态进入跳跃中状态，此时离地
//         }
//
//         {//若不摔跤
//             if (jump_state == jump && (left_off_ground_state == not_off_ground || left_skid_state == not_skid))
//             {   //条件：处于跳跃中状态、（已着地或滑行）
//                 jump_state = land ;      //跳跃状态进入跳跃着陆状态，开始减速
//                 jump_state_jump_time = 0;
//             }
//
//             if (jump_state == land && body_speed_down_finish_flag > 10)
//             {   //条件：处于跳跃着陆状态、速度小于5cm/s且滤波
//                 jump_state = idle ;      //跳跃状态进入跳跃空闲状态，完成跳跃，开始调整高度
//             }
//         }
//
//         {//若摔跤
//             if (jump_state == jump && rightstumble_state == is_stumble)
//             {   //条件：处于跳跃中状态、摔跤检测触发
//                 jump_state = stumble ;
//             }
//
//             //翻车
//             if (jump_state == stumble && rightstumble_state == not_stumble && left_off_ground_state == is_off_ground)
//             {
//                 jump_state = turtle ;
//             }
//
//             //恢复
//             if ((jump_state == stumble || jump_state == turtle) && rightstumble_state == not_stumble && left_off_ground_state == not_off_ground)
//             {
//                 jump_state = idle ;
//             }
//         }
//     }
//
//
//     {//不同状态的行为控制
//         if (jump_state == jump)  //跳跃腾空
//         {   //机身上升收腿，下降伸腿
//             //时间顺序检测
//             static int body_down_flag = 0;
//             jump_state_jump_time ++;    //跳跃中时间计数多0.001s
//
//             //跳跃腿长初始化
//             if (jump_state_jump_time == 1)
//             {
//                 right_leg.L_tui_target = L_leg_min ;        //腿长设置为短
//                 left_leg.L_tui_target = L_leg_min ;
//                 body_down_flag = 0;                 //重置下降标志
//             }
//
//             //下降检测滤波
//             if (body_speed_y < 0)
//             {
//                 body_down_flag ++;
//             }
//             else
//             {
//                 body_down_flag = 0;
//             }
//
//             if (body_down_flag > 10)
//             {
//                 right_leg.L_tui_target = L_leg_max ;       //腿长设置为长
//                 left_leg.L_tui_target = L_leg_max ;
//             }
//             /*
//                     char body_speed_y_flag_zheng = 0 ;
//                     char body_speed_y_flag_fu = 0 ;
//
//                 //五次滤波
//                     if (body_speed_y > 0)
//                     {
//                         if (body_speed_y_flag_zheng < 5)
//                         {
//                             body_speed_y_flag_zheng ++;
//                         }
//                         body_speed_y_flag_fu =0;
//                     }
//                     else
//                     {
//                         if (body_speed_y_flag_fu < 5)
//                         {
//                             body_speed_y_flag_fu ++;
//                         }
//                         body_speed_y_flag_zheng =0;
//                     }
//
//
//                     if (body_speed_y_flag_zheng == 5)
//                     {
//                         right_leg.L_tui_target = 5 ;        //腿长设置为短
//                         left_leg.L_tui_target = 5 ;
//                     }
//                     if (body_speed_y_flag_fu == 5)
//                     {
//                         right_leg.L_tui_target = 20 ;       //腿长设置为长
//                         left_leg.L_tui_target = 20 ;
//                     }
//             */
//         }
//
//
//
//         if (jump_state == land)  //着陆状态
//         {
//             right_leg.L_tui_target = 20 ;       //腿长设置为长，弹性缓冲
//             left_leg.L_tui_target = 20 ;
//
//             //机身速度减速完成 累加滤波
//             if (body_speed_y < 0.05f && body_speed_y > -0.05f)
//             {
//                 body_speed_down_finish_flag ++ ;
//             }
//             else
//             {
//                 body_speed_down_finish_flag = 0 ;
//             }
//         }
//
//
//         if (jump_state == idle)  //跳跃空闲状态
//         {
//             right_leg.L_tui_target = L_leg_min ;       //腿长设置为短
//             left_leg.L_tui_target = L_leg_min ;
//         }
//
//
//         if (jump_state == stumble || jump_state == turtle)   //摔跤或翻车状态
//         {
//             right_leg.L_tui_target = L_leg_min ;       //腿长设置为短
//             left_leg.L_tui_target = L_leg_min ;
//         }
//     }
// }
