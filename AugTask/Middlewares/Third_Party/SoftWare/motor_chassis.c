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

/* �ٶȻ� */
float Chassis_Speed_Kp   = 12.0f; /*< �ٶȻ�P���� */
float Chassis_Speed_Ki   = 0.15f; /*< �ٶȻ�I���� */
float Chassis_Speed_Kd   = 0.0f; /*< �ٶȻ�D���� */
float Chassis_Speed_Inc_Limit = 16000.0f; /*< �ٶȻ������޷� */

/* ������ */
float Chassis_Current_Kp = 1.5f; /*< ������P���� */
float Chassis_Current_Ki = 0.5f; /*< ������I���� */
float Chassis_Current_Kd = 0.03f; /*< ������D���� */
float Chassis_Current_Inc_Limit = 16000.0f; /*< �����������޷� */

float Chassis_Out_Limit = 16000.0f; /*< ����޷���3508������16384*/
/* function ------------------------------------------------------------------*/

/**
  * @brief  ң���������ٶȽ���
  * @param  ң������Ϣ�ṹ��
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
  * @brief  ���̵��PID��ʼ��
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
  * @brief  ���̵��CAN
  * @param  void
  * @retval void
  * @attention ���ж�����
  */
void Motor_Chassis_CanTransmit(void)
{
    uint8_t i;
    for(i=0;i<4;i++)
		{
        /* ����޷� */
        Chassis.M3508[i].LPf.Output = constrain_int16_t(Chassis.M3508[i].LPf.Output,
                                               -Chassis_Out_Limit, Chassis_Out_Limit);
            
        /* CAN ��ֵ */
        Chassis.CanData[2*i]=(uint8_t)(Chassis.M3508[i].LPf.Output>>8);
        Chassis.CanData[2*i+1]=(uint8_t)(Chassis.M3508[i].LPf.Output);
    }
    CAN1_Transmit(0x200,Chassis.CanData);
}

int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	return amt<low? low:(amt>high? high:amt); /*< �ʺű��ʽ���б�if else�� */
}

/**
  * @brief  ���̵��PID���
  * @param  void
  * @retval void
  * @attention ��ͨ���ٶȻ����⻷������ֵ�ټ����������ͨ�����������ڻ���ֱ�ӿ����������rx,lpf???��
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
  * @brief  ���̽���
  * @param  ����ָ��ṹ��
  * @retval void
* @attention 
  */
void Chassis_Process(RemoteData_t RDMsg)
{
    Remote_Control_GetMoveData(RDMsg);
    Motor_Chassis_PidRun();
}


/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
