/**
  *@file test_uart.c
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
  
#ifndef _TEST__UART_H
#define _TEST__UART_H

#include "stm32f4xx_HAL.h"


extern int16_t output_data[8];

extern void sendware(void *wareaddr, uint32_t waresize);
#endif

