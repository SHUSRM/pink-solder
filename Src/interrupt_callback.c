#include "interrupt_callback.h"
#include "control.h"
#include "usart.h"
#include "tim.h"
#include "can.h"
#include "math.h"
#include "string.h"


#define PERIOD  1000
#define IDENTIFICATION	0
#define SINE			0
#define PITCH			1
#if PITCH
	#define YAW				0
#else
	#define YAW				1
#endif
u16 timeCount 	= 0;
u8 	periodCount = 0;
int	periodIndex = 70;
u8 	setSpeedData[PERIOD*20]; //143040
u8 	realSpeedData[PERIOD*20];
u16 Period[64] = {1000,667,500,400,333,286,250,222,200,182,167,154,143,
				133,125,118,111,105,100,95,91,87,83,80,77,74,71,69,67,65,
				63,61,59,57,56,54,53,51,50,49,48,47,45,42,38,36,33,31,29,28,
				26,25,20,17,14,13,11,10,9,8,5,4,3,2};

//定时器溢出中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
	}
	if (htim->Instance == TIM6)
	{
	#if IDENTIFICATION
		#if YAW
		#if SINE		
		if(periodIndex < 64 && periodIndex >=0)
		{
			cloudYaw.SetSpeed = 700*sin(2*PI* timeCount / Period[periodIndex]);	
			MPU6050_GetData();
			cloudYaw.CurrentOutput = PID_Calc(&(cloudYaw.SpeedPID),
												   mpu6050.Gyro.Origin.x, cloudYaw.SetSpeed);
			
			MOTO_CloudPitchPID(timeCount);
			CAN_SetCloudMotorCurrent(cloudPitch.CurrentOutput,cloudYaw.CurrentOutput,0);		
			
			if(timeCount < Period[periodIndex])
				timeCount ++;
			else
			{
				timeCount = 0;
				periodCount++;
			}
			if(periodCount == 20)
			{
				periodCount = 0;	
				periodIndex ++;
			}
		}
		else
		{
			cloudYaw.SetSpeed = 0;//700*sin(2*PI*timeCount/PERIOD);	
			MPU6050_GetData();
			cloudYaw.CurrentOutput = PID_Calc(&(cloudYaw.SpeedPID),
												   mpu6050.Gyro.Origin.x, cloudYaw.SetSpeed);
			MOTO_CloudPitchPID(timeCount);
			CAN_SetCloudMotorCurrent(cloudPitch.CurrentOutput,cloudYaw.CurrentOutput,0);
			if(timeCount < PERIOD)
				timeCount ++;
			else
			{
				timeCount = 0;
				periodCount++;
			}
			if(periodCount == 20)
			{
				periodCount = 0;	
				periodIndex ++;
			}
			if(periodIndex >1000)
			{
				periodIndex = 70;
			}
		}
		#else
//		switch(periodCount)
//		{
//		case 0:
//			cloudYaw.SetSpeed = 0;
//			break;
//		case 1:
//			cloudYaw.SetSpeed = 700;
//			break;
//		case 2:
//			cloudYaw.SetSpeed = 0;
//			break;
//		case 3:
//			cloudYaw.SetSpeed = -700;
//			break;
//		}
		switch(periodCount)
		{
		case 0:
			cloudYaw.SetAngle = 1000;
			break;
		case 1:
			cloudYaw.SetAngle = 1700;
			break;
		case 2:
			cloudYaw.SetAngle = 1000;
			break;
		case 3:
			cloudYaw.SetAngle = 300;
			break;
		}
		MPU6050_GetData();
		cloudYaw.SetSpeed = PID_SpecialCalc(&(cloudYaw.AnglePID),
											 cloudYaw.Angle, cloudYaw.SetAngle, mpu6050.Gyro.Radian.x);
		cloudYaw.CurrentOutput = PID_Calc(&(cloudYaw.SpeedPID),
											   mpu6050.Gyro.Origin.x, cloudYaw.SetSpeed);
		MOTO_CloudPitchPID(timeCount);
		CAN_SetCloudMotorCurrent(cloudPitch.CurrentOutput,cloudYaw.CurrentOutput,0);
		if(timeCount < PERIOD)
			timeCount ++;
		else
		{
			timeCount = 0;
			periodCount ++;
		}
		if(periodCount == 4)
		{
			periodCount = 0;
		}
		
		#endif
		#endif
		#if PITCH
		#if SINE		
		if(periodIndex < 64 && periodIndex >=0)
		{
			cloudPitch.SetSpeed = 700*sin(2*PI* timeCount / Period[periodIndex]);	
			MPU6050_GetData();
			cloudPitch.CurrentOutput = PID_Calc(&(cloudPitch.SpeedPID),
												   mpu6050.Gyro.Origin.y, cloudPitch.SetSpeed);
			
			MOTO_CloudYawPID(timeCount);
			CAN_SetCloudMotorCurrent(cloudPitch.CurrentOutput,cloudYaw.CurrentOutput,0);		
			
			if(timeCount < Period[periodIndex])
				timeCount ++;
			else
			{
				timeCount = 0;
				periodCount++;
			}
			if(periodCount == 20)
			{
				periodCount = 0;	
				periodIndex ++;
			}
		}
		else
		{
			cloudPitch.SetSpeed = -500;//700*sin(2*PI*timeCount/PERIOD);	
			MPU6050_GetData();
			cloudPitch.CurrentOutput = PID_Calc(&(cloudPitch.SpeedPID),
												   mpu6050.Gyro.Origin.y, cloudPitch.SetSpeed);
			MOTO_CloudYawPID(timeCount);
			CAN_SetCloudMotorCurrent(cloudPitch.CurrentOutput,cloudYaw.CurrentOutput,0);
			if(timeCount < PERIOD)
				timeCount ++;
			else
			{
				timeCount = 0;
				periodCount++;
			}
			if(periodCount == 20)
			{
				periodCount = 0;	
				periodIndex ++;
			}
			if(periodIndex >1000)
			{
				periodIndex = 70;
			}
		}
		#else
//		switch(periodCount)
//		{
//		case 0:
//			cloudPitch.SetSpeed = 0;
//			break;
//		case 1:
//			cloudPitch.SetSpeed = 700;
//			break;
//		case 2:
//			cloudPitch.SetSpeed = 0;
//			break;
//		case 3:
//			cloudPitch.SetSpeed = -700;
//			break;
//		}
		switch(periodCount)
		{
		case 0:
			cloudPitch.SetAngle = PITCH_MID;
			break;
		case 1:
			cloudPitch.SetAngle = PITCH_MID + 300;
			break;
		case 2:
			cloudPitch.SetAngle = PITCH_MID;
			break;
		case 3:
			cloudPitch.SetAngle = PITCH_MID - 300;
			break;
		}
		MPU6050_GetData();
		cloudPitch.SetSpeed = PID_SpecialCalc(&(cloudPitch.AnglePID),
											   cloudPitch.Angle, cloudPitch.SetAngle, mpu6050.Gyro.Origin.y);
		cloudPitch.CurrentOutput = PID_Calc(&(cloudPitch.SpeedPID),
											mpu6050.Gyro.Origin.y, cloudPitch.SetSpeed);
		MOTO_CloudYawPID(timeCount);
		CAN_SetCloudMotorCurrent(cloudPitch.CurrentOutput,cloudYaw.CurrentOutput,0);
		if(timeCount < PERIOD)
			timeCount ++;
		else
		{
			timeCount = 0;
			periodCount ++;
		}
		if(periodCount == 4)
		{
			periodCount = 0;
		}
		
		#endif
		#endif
	#else
		MPU6050_GetData();
		//位置环10ms控制一次,速度环1ms控制一次
		MOTO_CloudPitchPID(timeCount);
		MOTO_CloudYawPID(timeCount);
		CAN_SetCloudMotorCurrent(cloudPitch.CurrentOutput,cloudYaw.CurrentOutput,0);		
			
		if(timeCount % 5 == 0)
		{
//			MPU6050_GetData();
//			KalmanFilter(atan2(mpu6050.Acc.Origin.x,mpu6050.Acc.Origin.z)*180/PI,-mpu6050.Gyro.Origin.y/16.4,&mpu6050.PitchK);		
			
//			MPU6500_GetData();	
		}
		if(timeCount % 10 == 0)
		{
//			MOTO_UnderpanPID();
//			CAN_SetUnderpanMotorCurrent(underpan[0].CurrentOutput, underpan[1].CurrentOutput,
//										underpan[2].CurrentOutput, underpan[3].CurrentOutput);
		}	
		
		if(timeCount < 1000)
			timeCount ++;
		else
			timeCount = 0;
	#endif	
	}
}

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
	/************遥控器数据处理*************/
	if (huart->Instance == USART1)
	{
		TelecontrollerData();
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
	/************串口陀螺仪数据处理************/
	else if(huart->Instance == USART3)
	{
	#if 0
		if ((mpu6050.RxCount & 0x8000) == 0) //接收未完成
		{
			mpu6050.RxBuf[mpu6050.RxCount & 0X3FFF] = mpu6050.mpuReadBuf;

			if ((mpu6050.RxCount & 0X3FFF) == 0 && mpu6050.RxBuf[0] != 0x55)
				return; //第 0 号数据不是帧头，跳过
			if ((mpu6050.RxCount & 0X3FFF) == 1 && mpu6050.RxBuf[1] != 0x52)
			{
				mpu6050.RxCount = 0;
				return; //第 2 号数据不是角度，跳过
			}
			
			mpu6050.RxCount++;

			if ((mpu6050.RxCount & 0X3FFF) == 11)
			{
				for (int i = 0; i < 10; i++)
					mpu6050.RxSum += mpu6050.RxBuf[i];

				if (mpu6050.RxSum == mpu6050.RxBuf[10])
				{
					mpu6050.RxCount |= 0x8000;
					
					mpu6050.Auto.GyroX = ((short)(mpu6050.RxBuf[3] << 8 | mpu6050.RxBuf[2])) / 32768.0 * 2000;
					mpu6050.Auto.GyroY = ((short)((mpu6050.RxBuf[5] << 8) | mpu6050.RxBuf[4])) / 32768.0 * 2000;
					mpu6050.Auto.GyroZ = ((short)((mpu6050.RxBuf[7] << 8) | mpu6050.RxBuf[6])) / 32768.0 * 2000;
					mpu6050.RxCount = 0;
					mpu6050.RxSum = 0;
				}
				
				else
				{
					mpu6050.RxCount = 0;
					mpu6050.RxSum = 0;
				}
			}
		}
	#endif
		if(mpu6050.RxBuf[11] == 0x55 && mpu6050.RxBuf[12] == 0x52)
		{
			for (int i = 0; i < 10; i++)
					mpu6050.RxSum += mpu6050.RxBuf[i+11];
			if (mpu6050.RxSum == mpu6050.RxBuf[21])
			{
				mpu6050.Auto.GyroX = ((short)(mpu6050.RxBuf[14] << 8 | mpu6050.RxBuf[13])) / 32768.0 * 2000;
				mpu6050.Auto.GyroY = ((short)((mpu6050.RxBuf[16] << 8) | mpu6050.RxBuf[15])) / 32768.0 * 2000;
				mpu6050.Auto.GyroZ = ((short)((mpu6050.RxBuf[18] << 8) | mpu6050.RxBuf[17])) / 32768.0 * 2000;
				mpu6050.RxSum = 0;
			}	
		}
		else
		{
			IMU_Init();
			HAL_UART_Receive_DMA(&huart3,mpu6050.RxBuf,sizeof(mpu6050.RxBuf));
		}
	}
	/*************PID参数串口数据处理***********/
	else if (huart->Instance == UART4)
	{
		rxPID.Buf[rxPID.Count & 0x7f] = rxPID.pidReadBuf;
		//是否开始接收
		if ((rxPID.Count & 0x7f) == 0 && rxPID.Buf[0] != '$')
			return;

		rxPID.Count++;

		if ((rxPID.Count & 0x7f) == 8)
		{
			//接收正确
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
	/******************裁判系统串口数据处理********************/
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
