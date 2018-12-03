#include "interrupt_callback.h"
#include "control.h"
#include "usart.h"
#include "tim.h"
#include "can.h"
#include "math.h"

u8 flag = 0;
//��ʱ������ж�
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
	}
	if (htim->Instance == TIM6)
	{
		flag = !flag;
		if (flag)
		{
			get_mpu_data();
			//�������˲�
			KalmanFilter(atan2(sensor.Acc.Origin.x,sensor.Acc.Origin.z)*180/PI,-sensor.Gyro.Origin.y/16.4);						
		}
		get_mpu_data();
		//�������˲�
		KalmanFilter(atan2(sensor.Acc.Origin.x,sensor.Acc.Origin.z)*180/PI,-sensor.Gyro.Origin.y/16.4);				
//		MOTO_UnderpanPID();
//		CAN_SetUnderpanMotorCurrent(underpan[0].CurrentOutput, underpan[1].CurrentOutput,
//									underpan[2].CurrentOutput, underpan[3].CurrentOutput);
//		MOTO_CloudPitchPID();
//		MOTO_CloudYawPID();
//		CAN_SetCloudMotorCurrent(cloudPitch.CurrentOutput,cloudYaw.CurrentOutput,0);
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

char charBuf[4];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/************ң�������ݴ���*************/
	if (huart->Instance == USART1)
	{
		TelecontrollerData();
	}
	/*************С���Դ������ݴ���************/
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
	else if(huart->Instance == USART3)
	{
	#if 0
		if ((sensor.RxCount & 0x8000) == 0) //����δ���
		{
			sensor.RxBuf[sensor.RxCount & 0X3FFF] = sensor.mpuReadBuf;

			if ((sensor.RxCount & 0X3FFF) == 0 && sensor.RxBuf[0] != 0x55)
				return; //�� 0 �����ݲ���֡ͷ������
			if ((sensor.RxCount & 0X3FFF) == 1 && sensor.RxBuf[1] != 0x52)
			{
				sensor.RxCount = 0;
				return; //�� 2 �����ݲ��ǽǶȣ�����
			}
			
			sensor.RxCount++;

			if ((sensor.RxCount & 0X3FFF) == 11)
			{
				for (int i = 0; i < 10; i++)
					sensor.RxSum += sensor.RxBuf[i];

				if (sensor.RxSum == sensor.RxBuf[10])
				{
					sensor.RxCount |= 0x8000;
					
					sensor.Auto.GyroX = ((short)(sensor.RxBuf[3] << 8 | sensor.RxBuf[2])) / 32768.0 * 2000;
					sensor.Auto.GyroY = ((short)((sensor.RxBuf[5] << 8) | sensor.RxBuf[4])) / 32768.0 * 2000;
					sensor.Auto.GyroZ = ((short)((sensor.RxBuf[7] << 8) | sensor.RxBuf[6])) / 32768.0 * 2000;
					sensor.RxCount = 0;
					sensor.RxSum = 0;
				}
				
				else
				{
					sensor.RxCount = 0;
					sensor.RxSum = 0;
				}
			}
		}
	#endif
		if(sensor.RxBuf[11] == 0x55 && sensor.RxBuf[12] == 0x52)
		{
			for (int i = 0; i < 10; i++)
					sensor.RxSum += sensor.RxBuf[i+11];
			if (sensor.RxSum == sensor.RxBuf[21])
			{
				sensor.Auto.GyroX = ((short)(sensor.RxBuf[14] << 8 | sensor.RxBuf[13])) / 32768.0 * 2000;
				sensor.Auto.GyroY = ((short)((sensor.RxBuf[16] << 8) | sensor.RxBuf[15])) / 32768.0 * 2000;
				sensor.Auto.GyroZ = ((short)((sensor.RxBuf[18] << 8) | sensor.RxBuf[17])) / 32768.0 * 2000;
				sensor.RxSum = 0;
			}	
		}
		else
		{
			IMU_Init();
			HAL_UART_Receive_DMA(&huart3,sensor.RxBuf,sizeof(sensor.RxBuf));
		}
	}
	/*************PID�����������ݴ���***********/
	else if (huart->Instance == UART4)
	{
		rxPID.Buf[rxPID.Count & 0x7f] = rxPID.pidReadBuf;
		//�Ƿ�ʼ����
		if ((rxPID.Count & 0x7f) == 0 && rxPID.Buf[0] != '$')
			return;

		rxPID.Count++;

		if ((rxPID.Count & 0x7f) == 8)
		{
			//������ȷ
			if (rxPID.Sum == rxPID.pidReadBuf)
			{
				for (int i = 0; i < 4; i++)
					charBuf[i] = rxPID.Buf[i + 3];

				switch (rxPID.Buf[1])
				{
				case 'p':
					memcpy(&rxPID.pidAdjust->p, charBuf, 4);
					if (rxPID.Buf[2] == '-')
						rxPID.pidAdjust->p = -rxPID.pidAdjust->p;
					break;
				case 'i':
					memcpy(&rxPID.pidAdjust->i, charBuf, 4);
					if (rxPID.Buf[2] == '-')
						rxPID.pidAdjust->i = -rxPID.pidAdjust->i;
					break;
				case 'd':
					memcpy(&rxPID.pidAdjust->d, charBuf, 4);
					if (rxPID.Buf[2] == '-')
						rxPID.pidAdjust->d = -rxPID.pidAdjust->d;
					break;
				}
				rxPID.Sum = 0;
				rxPID.Count = 0;
			}
			else
			{
				rxPID.Sum = 0;
				rxPID.Count = 0;
			}
		}
		else
			rxPID.Sum += rxPID.pidReadBuf;
	}
	/******************����ϵͳ�������ݴ���********************/
	else if (huart->Instance == USART6)
	{
	}
}

//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->ErrorCode == HAL_UART_ERROR_ORE)
//	{
//		
//	}
//}
