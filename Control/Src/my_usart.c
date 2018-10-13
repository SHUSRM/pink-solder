/**
  *@file usart.c
  *@date 2018-10-11
  *@author Vacuo.W
  *@brief 
  */

#include "my_usart.h"
#include "control.h"


//struct CAMERA camera;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

void sendware(void *wareaddr, uint32_t waresize)
{
	#define CMD_WARE     3
	uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    
	uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};   
	HAL_UART_Transmit(&huart4, (uint8_t *)cmdf, sizeof(cmdf), 5000);
	HAL_UART_Transmit(&huart4, (uint8_t *)wareaddr, waresize ,5000);
	HAL_UART_Transmit(&huart4, (uint8_t *)cmdr, sizeof(cmdr), 5000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    
	/************小电脑串口数据处理*************/
    if (huart->Instance==USART2)
    {
        switch (camera.count)
        {
            case 0:
                if (camera.recieve[0]=='&') camera.count=1;
                else camera.count=0;
                break;
            case 1:
                if (camera.recieve[0]=='%') camera.count=2;
                else camera.count=0;
                break;
            case 2:
                camera.sum = '%'+'&';
                camera.x += camera.recieve[0]<<8;
                camera.sum += camera.recieve[0];
                camera.count=3;
                break;
            case 3:
                camera.x += camera.recieve[0];
                camera.sum += camera.recieve[0];
                camera.count=4;
                break;
            case 4:
                if (camera.recieve[0]=='-') camera.x = -camera.x;
                camera.sum += camera.recieve[0];
                camera.count=5;
                break;
            case 5:
                camera.y += camera.recieve[0]<<8;
                camera.sum += camera.recieve[0];
                camera.count=6;
                break;
            case 6:
                camera.y += camera.recieve[0];
                camera.sum += camera.recieve[0];
                camera.count=7;
                break;
            case 7:
                if (camera.recieve[0]=='-') camera.y = -camera.y;
                camera.sum += camera.recieve[0];
                camera.count=8;
                break;
            case 8:
                if (camera.sum==camera.recieve[0])
                {
                    camera.transmit[0]='R';
                    HAL_UART_Transmit(&huart2,camera.transmit,1,1000);
                }
                else {
                    camera.x=0;
                    camera.y=0;
                    camera.transmit[0]='W';
                    HAL_UART_Transmit(&huart2,camera.transmit,1,1000);
                }
                camera.count=0;
                break;
        }
    }   
	
	/******************裁判系统串口数据处理********************/
	else if (huart->Instance==USART6)
	{
		
		
		
		
	}
	

    
 
}