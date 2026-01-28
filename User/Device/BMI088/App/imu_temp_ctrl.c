#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "imu_temp_ctrl.h"
#include "MahonyAHRS.h"
#include "pid.h"
#define cheat TRUE  //浣�寮�妯″� �绘��杈�灏���gyro�?
#define correct_Time_define 1000    //涓��靛�?0椋? 1000娆″��骞冲��
#define temp_times 300       //�㈡�娓╁害���?

pid_type_def Temperature_PID={0};
float Temperature_PID_Para[3]={1600,50,40};


float gyro[3], accel[3], temp;
float gyro_correct[3]={0};
float RefTemp = 40;   //Destination
float roll,pitch,yaw=0;
uint8_t attitude_flag=0;
uint32_t correct_times=0;


void INS_Init(void)
{
	
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f,0); //ekf���?���
//		PID_init(&Temperature_PID, PID_POSITION,Temperature_PID_Para,2000,200); //����pidlimit
		Mahony_Init(1000);  //mahony濮挎��瑙ｇ����濮���
    // imu heat init
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init());  //���轰华���?���
}

uint32_t temp_temperature=0;
void IMU_Temperature_Ctrl()
{
	  PID_calc(&Temperature_PID, temp, RefTemp); //娓╁害pid  //��瑕�璋�涓�涓�pid浣垮�娓╁害�?40掳宸���
		temp_temperature=(uint32_t)Temperature_PID.out; 
		if(Temperature_PID.out<0)
		{
			temp_temperature=0;
		}
    htim10.Instance->CCR4 = temp_temperature;
}

int gyroscope_flag = 0;

/***
 * @brief: INS_TASK(void const * argument)
 * @param: argument - 浠诲�″����
 * @retval: void
 * @details: IMU濮挎���у�朵换�″�芥�?
 
*/
static uint8_t first_mahony=0; 
void INS_Task(void)
{
    static uint32_t count = 0;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_read(gyro, accel, &temp);
				if(first_mahony==0)
				{
					first_mahony++;
					MahonyAHRSinit(accel[0],accel[1],accel[2],0,0,0);  
				}
				if(attitude_flag==2)  //ekf��濮挎��瑙ｇ�?
				{
					gyro[0]-=gyro_correct[0];   //���婚���轰华0椋?
					gyro[1]-=gyro_correct[1];
					gyro[2]-=gyro_correct[2];
					
					#if cheat              //浣�寮� �?浠ヨ?�yaw寰�绋冲�? �绘��姣�杈�灏����?
						if(fabsf(gyro[2])<0.003f)
							gyro[2]=0;
					#endif
					//===========================================================================
						//ekf濮挎��瑙ｇ���ㄥ�?
					//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
					IMU_QuaternionEKF_Update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2]);
					//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
					//===============================================================================	
						
					//=================================================================================
					//mahony濮挎��瑙ｇ���ㄥ�?
					//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
					Mahony_update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],0,0,0);
					Mahony_computeAngles(); //瑙�搴�璁＄��   绉绘?��板�?��骞冲�伴��瑕��挎�㈡��瀵瑰���arm_atan2_f32 �? arm_asin
					//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
					//=============================================================================
					//ekf�峰��濮挎���?�搴��芥��
					pitch=Get_Pitch(); //�峰�pitch
					roll=Get_Roll();//�峰�roll
					yaw=Get_Yaw();//�峰�yaw
					//==============================================================================
				}
				else if(attitude_flag==1)   //�舵�?1 寮�濮?1000娆＄�����轰华0椋���濮���
				{
						//gyro correct
						gyro_correct[0]+=	gyro[0];
						gyro_correct[1]+=	gyro[1];
						gyro_correct[2]+=	gyro[2];
						correct_times++;
						if(correct_times>=correct_Time_define)
						{
							gyro_correct[0]/=correct_Time_define;
							gyro_correct[1]/=correct_Time_define;
							gyro_correct[2]/=correct_Time_define;
							attitude_flag=2; //go to 2 state
						}
				}
    }

    // temperature control
    if ((count % 10) == 0)
    {
        // 100hz ��娓╁害�у��pid
        IMU_Temperature_Ctrl();
				
				static uint32_t temp_Ticks=0;
//				if((fabsf(temp-RefTemp)<0.5f)&&attitude_flag==0) //�ヨ�棰�瀹�娓╁害涔�宸�灏�浜�0.5掳 寮�濮��?℃��
				{
					temp_Ticks++;
					if(temp_Ticks>temp_times)   //璁℃�拌揪�颁�瀹��?℃�板�? ��杩��?0椋���濮��� 璇存��娓╁害宸茬�杈惧�扮�?�?
					{
						attitude_flag=1;  //go to correct state
					}
				}
    }
    count++;

	gyroscope_flag++;
}

int pre_gyroscope_flag = 0;

void gyroscope_outline_detection()
{
	if (gyroscope_flag ==pre_gyroscope_flag)
		gyroscope_state = gyroscope_error;
	else
	{
		gyroscope_state = gyroscope_OK;
	}
	pre_gyroscope_flag = gyroscope_flag;
}

void IMU_task(void  * argument)
{
    INS_Init();
    /* Infinite loop */
    for (;;)
    {
        INS_Task();
        osDelay(1);
    }
}


/**
************************************************************************
* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 瑙���涓?�?��GPIO寮���
* @retval:     	void
* @details:    	GPIO澶��ㄤ�?�?��璋��芥�帮��?�������搴�璁″�����轰华涓?�?
************************************************************************
**/
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if(GPIO_Pin == ACC_INT_Pin)
//    {
//    }
//    else if(GPIO_Pin == GYRO_INT_Pin)
//    {

//    }
//}
