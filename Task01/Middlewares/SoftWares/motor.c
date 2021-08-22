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
#include "motor.h"
#include "message.h"
#include "can.h"
/* typedef -------------------------------------------------------------------*/
typedef struct
{
	CAN_TxHeaderTypeDef*         TxMsg;
	uint8_t                      Data[8];
} CAN_TxTypeDef;

typedef int16_t 	s16;
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
/* function ------------------------------------------------------------------*/
#define CIRCLE_FASTEST 1000
void Circle_Continue(Mcircle_t *Mc, uint16_t angle)
{
    if ((angle < CIRCLE_FASTEST) && (Mc->Angle > 8192 - CIRCLE_FASTEST))
    {
        Mc->Circle++;
    }
    else if ((angle > 8192 - CIRCLE_FASTEST) && (Mc->Angle < CIRCLE_FASTEST))
    {
        Mc->Circle--;
    }
    Mc->Angle = angle;
}

void set_moto_current(CAN_TxTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){

	hcan->TxMsg->StdId = 0x200;
	hcan->TxMsg->IDE = CAN_ID_STD;
	hcan->TxMsg->RTR = CAN_RTR_DATA;
	hcan->TxMsg->DLC = 0x08;
	hcan->Data[0] = iq1 >> 8;
	hcan->Data[1] = iq1;
	hcan->Data[2] = iq2 >> 8;
	hcan->Data[3] = iq2;
	hcan->Data[4] = iq3 >> 8;
	hcan->Data[5] = iq3;
	hcan->Data[6] = iq4 >> 8;
	hcan->Data[7] = iq4;
	
}	


/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
