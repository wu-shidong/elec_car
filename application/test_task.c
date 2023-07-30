/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "CAN_receive.h"
/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const * argument)
{	
	Motor_enable();
	can_filter_init();
	PosSpeed_CtrlMotor(&GIMBAL_CAN,&gimbal[0],0x105,4,10);
	PosSpeed_CtrlMotor(&GIMBAL_CAN,&gimbal[1],0x106,4,10);
	while(1)
	{
		PosSpeed_CtrlMotor(&GIMBAL_CAN,&gimbal[0],0x105,4,10);
		PosSpeed_CtrlMotor(&GIMBAL_CAN,&gimbal[1],0x106,4,10);
		osDelay(10);
	}
}



