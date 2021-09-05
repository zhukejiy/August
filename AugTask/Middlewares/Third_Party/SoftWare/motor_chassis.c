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
#include "message.h"
#include "motor_chassis.h"
#include "can.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
Motor_Chassis_t Chassis = {0};

/* 速度环 */
float Chassis_Speed_Kp   = 12.0f; /*< 速度环P参数 */
float Chassis_Speed_Ki   = 0.15f; /*< 速度环I参数 */
float Chassis_Speed_Kd   = 0.0f; /*< 速度环D参数 */
float Chassis_Speed_Inc_Limit = 16000.0f; /*< 速度环增量限幅 */

/* 电流环 */
float Chassis_Current_Kp = 1.5f; /*< 电流环P参数 */
float Chassis_Current_Ki = 0.5f; /*< 电流环I参数 */
float Chassis_Current_Kd = 0.03f; /*< 电流环D参数 */
float Chassis_Current_Inc_Limit = 16000.0f; /*< 电流环增量限幅 */

float Chassis_Out_Limit = 16000.0f; /*< 输出限幅，3508最大接收16384*/
/* function ------------------------------------------------------------------*/

/**
  * @brief  遥控器控制速度解算
  * @param  遥控器消息结构体
  * @retval 
  * @attention 
  */
void Remote_Control_GetMoveData(RemoteData_t RDMsg)
{
	 uint8_t i;
   for(i=0;i<4;i++)
   {
		   Chassis.M3508[i].TarSpeed = RDMsg.Ch0;
	 }
}


/**
  * @brief  底盘电机PID初始化
  * @param  void
  * @retval void
  * @attention 
  */
void Motor_Chassis_Init(void)
{
    uint8_t i;
	
    for(i=0;i<4;i++)
    {
        pid_init_increment(&Chassis.M3508[i].PidSpeed,Chassis_Speed_Kp,
                           Chassis_Speed_Ki,Chassis_Speed_Kd,Chassis_Speed_Inc_Limit);
    }
    for(i=0;i<4;i++)
    {
        pid_init_increment(&Chassis.M3508[i].PidCurrent,Chassis_Current_Kp,
                           Chassis_Current_Ki,Chassis_Current_Kd,Chassis_Current_Inc_Limit);
    }
}


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

/**
  * @brief  底盘电机PID输出
  * @param  void
  * @retval void
  * @attention （通过速度环（外环）所得值再计算电流环，通过电流坏（内环）直接控制输出）（rx,lpf???）
  */
void Motor_Chassis_PidRun(void)
{
    uint8_t i;

    #ifdef CHASSIS_WAVE
        UART2_SendWave(5, 2, &Chassis.M3508[0].TarSpeed, &Chassis.M3508[0].Rx.Speed,
                             &Chassis.M3508[0].TarCurrent, &Chassis.M3508[0].Rx.Current,
                             &Chassis.M3508[0].LPf.Output);
    #endif

    #ifdef CHASSIS_DEBUG
        Chassis_Init();
    #endif

    for (i = 0; i < 4; i++)     //rx speed lpf
    {
        Chassis.M3508[i].LPf.Speed = 0.8 * Chassis.M3508[i].Rx.Speed + 0.2 * Chassis.M3508[i].LPf.Speed;
    }   
    for (i = 0; i < 4; i++)     //speed loop
    {
        Chassis.M3508[i].TarCurrent = pid_increment_update(Chassis.M3508[i].TarSpeed, Chassis.M3508[i].LPf.Speed, &Chassis.M3508[i].PidSpeed);
    }
    for (i = 0; i < 4; i++)     //tar current lpf 
    {
        Chassis.M3508[i].LPf.TarCurrent = 0.8 * Chassis.M3508[i].TarCurrent + 0.2 * Chassis.M3508[i].LPf.TarCurrent;
    }
    
    for (i = 0; i < 4; i++)     //rx current lpf 
    {
        Chassis.M3508[i].LPf.Current = 0.8 * Chassis.M3508[i].Rx.Current + 0.2 * Chassis.M3508[i].LPf.Current;
    }
    
    for (i = 0; i < 4; i++)     //current loop
    {
        Chassis.M3508[i].Output = pid_increment_update(Chassis.M3508[i].LPf.TarCurrent, Chassis.M3508[i].LPf.Current, &Chassis.M3508[i].PidCurrent);
    }
    
    for (i = 0; i < 4; i++)     //out lpf
    {
        Chassis.M3508[i].LPf.Output = 0.8 * Chassis.M3508[i].Output + 0.2 * Chassis.M3508[i].LPf.Output;
    }
}

/**
  * @brief  底盘进程
  * @param  控制指令结构体
  * @retval void
* @attention 
  */
void Chassis_Process(RemoteData_t RDMsg)
{
    Remote_Control_GetMoveData(RDMsg);
    Motor_Chassis_PidRun();
}


/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
