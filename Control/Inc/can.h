/**
  *@file can.h
  *@date 2018-10-7
  *@author Vacuo.W
  *@brief 
  */
  
  
#ifndef _CAN_H
#define _CAN_H

#include "stm32f4xx_hal.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;

extern CAN_TxHeaderTypeDef  Tx1Message;
extern CAN_RxHeaderTypeDef  Rx1Message;
static uint8_t aData[8];

void CAN1_Init();
void CAN_Getdata(CAN_HandleTypeDef *hcan,CAN_RxHeaderTypeDef *pHeader,uint8_t aData[]);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void underpan_motor_output(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);
void cloud_motor_output(int16_t iq1,int16_t iq2,int16_t iq3);


#endif
