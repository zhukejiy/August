/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "motor_chassis.h"
#include "can.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
Motor_Chassis_t Chassis = {0};
float Chassis_Out_Limit = 16000.0f; /*< 输出限幅，3508最大接收16384*/
/* function ------------------------------------------------------------------*/

/**
  * @brief  底盘电机CAN
  * @param  void
  * @retval void
  * @attention 放中断里面
  */
void Motor_Chassis_CanTransmit(void)
{
    uint8_t i;
    for(i=0;i<4;i++)
		{
        /* 输出限幅 */
        Chassis.M3508[i].LPf.Output = constrain_int16_t(Chassis.M3508[i].LPf.Output,
                                               -Chassis_Out_Limit, Chassis_Out_Limit);
            
        /* CAN 赋值 */
        Chassis.CanData[2*i]=(uint8_t)(Chassis.M3508[i].LPf.Output>>8);
        Chassis.CanData[2*i+1]=(uint8_t)(Chassis.M3508[i].LPf.Output);
    }
    CAN1_Transmit(0x200,Chassis.CanData);
}

int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	return amt<low? low:(amt>high? high:amt); /*< 问号表达式运行比if else快 */
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
