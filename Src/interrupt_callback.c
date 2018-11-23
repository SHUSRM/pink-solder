#include "interrupt_callback.h"
#include "control.h"
#include "usart.h"
#include "tim.h"
#include "can.h"
//定时器溢出中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
	}
	if (htim->Instance == TIM6)
	{
//		get_mpu_data();
		underpanPID();
		CAN_SetUnderpanMotorCurrent(underpan[0].CurrentOutput, underpan[1].CurrentOutput,
									underpan[2].CurrentOutput, underpan[3].CurrentOutput);
//		cloudPitchPID();
//		cloudYawPID();
//		CAN_SetCloudMotorCurrent(cloudPitch.CurrentOutput, cloudYaw.CurrentOutput, 1);
	}
}
//void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
//{
//	if (hcan->Instance == CAN1)
//	{
//		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx1Message, canRxDataBuf);
//		CAN_GetMotoData(&Rx1Message, canRxDataBuf);
//	}
//}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx1Message, canRxDataBuf);
		CAN_GetMotoData(&Rx1Message, canRxDataBuf);
	}
}

float _f;
char charBuf[4];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/************遥控器数据处理*************/
	if (huart->Instance == USART1)
	{
		telecontroller_data();
	}
	/*************小电脑串口数据处理************/
	if (huart->Instance == USART2)
	{
		switch (camera.Count)
		{
		case 0:
			if (camera.Recieve[0] == '&')
				camera.Count = 1;
			else
				camera.Count = 0;
			break;
		case 1:
			if (camera.Recieve[0] == '%')
				camera.Count = 2;
			else
				camera.Count = 0;
			break;
		case 2:
			camera.Sum = '%' + '&';
			camera.X += camera.Recieve[0] << 8;
			camera.Sum += camera.Recieve[0];
			camera.Count = 3;
			break;
		case 3:
			camera.X += camera.Recieve[0];
			camera.Sum += camera.Recieve[0];
			camera.Count = 4;
			break;
		case 4:
			if (camera.Recieve[0] == '-')
				camera.X = -camera.X;
			camera.Sum += camera.Recieve[0];
			camera.Count = 5;
			break;
		case 5:
			camera.Y += camera.Recieve[0] << 8;
			camera.Sum += camera.Recieve[0];
			camera.Count = 6;
			break;
		case 6:
			camera.Y += camera.Recieve[0];
			camera.Sum += camera.Recieve[0];
			camera.Count = 7;
			break;
		case 7:
			if (camera.Recieve[0] == '-')
				camera.Y = -camera.Y;
			camera.Sum += camera.Recieve[0];
			camera.Count = 8;
			break;
		case 8:
			if (camera.Sum == camera.Recieve[0])
			{
				camera.Transmit[0] = 'R';
				HAL_UART_Transmit(&huart2, camera.Transmit, 1, 1000);
			}
			else
			{
				camera.X = 0;
				camera.Y = 0;
				camera.Transmit[0] = 'W';
				HAL_UART_Transmit(&huart2, camera.Transmit, 1, 1000);
			}
			camera.Count = 0;
			break;
		}
	}
	/*************PID参数串口数据处理***********/
	else if (huart->Instance == UART4)
	{
		RX_PID_Buf[RX_PID_Count & 0x7f] = pidReadBuf;
		//是否开始接收
		if ((RX_PID_Count & 0x7f) == 0 && RX_PID_Buf[0] != '$')
			return;

		RX_PID_Count++;

		if ((RX_PID_Count & 0x7f) == 8)
		{
			//接收正确
			if (RX_PID_Sum == pidReadBuf)
			{
				for (int i = 0; i < 4; i++)
					charBuf[i] = RX_PID_Buf[i + 3];

				switch (RX_PID_Buf[1])
				{
				case 'p':
					memcpy(&pidAdjust->p, charBuf, 4);
					if (RX_PID_Buf[2] == '-')
						pidAdjust->p = -pidAdjust->p;
					break;
				case 'i':
					memcpy(&pidAdjust->i, charBuf, 4);
					if (RX_PID_Buf[2] == '-')
						pidAdjust->i = -pidAdjust->i;
					break;
				case 'd':
					memcpy(&pidAdjust->d, charBuf, 4);
					if (RX_PID_Buf[2] == '-')
						pidAdjust->d = -pidAdjust->d;
					break;
				}
				RX_PID_Sum = 0;
				RX_PID_Count = 0;
				printf("%.2f \n", cloudYaw.AnglePID.d);
			}
			else
			{
				RX_PID_Sum = 0;
				RX_PID_Count = 0;
			}
		}
		else
			RX_PID_Sum += pidReadBuf;
	}
	/******************裁判系统串口数据处理********************/
	else if (huart->Instance == USART6)
	{
	}
}

//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{

//}
