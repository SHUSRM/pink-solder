/**
  *@file test_can.h
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
  
#ifndef _TEST__CAN_H
#define _TEST__CAN_H

#include "stm32f4xx_HAL.h"


void CanFilter_Init(CAN_HandleTypeDef* hcan);
void underpan_motor_output(CAN_HandleTypeDef* hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);
void cloud_motor_output(CAN_HandleTypeDef* hcan,int16_t iq1,int16_t iq2,int16_t iq3);

#endif

