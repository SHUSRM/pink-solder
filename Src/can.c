/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "control.h"

u8 txData[8];
u8 canRxDataBuf[8];
uint32_t pTxMailbox;
CAN_TxHeaderTypeDef  Tx1Message;		//发送配置参数
CAN_RxHeaderTypeDef  Rx1Message;		//发送配置参数
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void CAN1_FilterInit()
{
	CAN_FilterTypeDef canfilter;

	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

	//  //filtrate any ID you want here
	canfilter.FilterIdHigh = 0x0000;
	canfilter.FilterIdLow = 0x0000;
	canfilter.FilterMaskIdHigh = 0x0000;
	canfilter.FilterMaskIdLow = 0x0000;

	canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
	canfilter.FilterActivation = ENABLE;
	canfilter.SlaveStartFilterBank = 14;
	//use different filter for can1&can2
	canfilter.FilterBank = 0;
	//    canfilter.FilterNumber = 0;
	//    hcan1.pTxMsg = &Tx1Message;
	//    hcan1.pRxMsg = &Rx1Message;

	HAL_CAN_ConfigFilter(&hcan1, &canfilter);

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_CAN_Start(&hcan1);
}
/***************************************
底盘电调id：201到204
云台电调id：205到206
拨弹电调id：207
***************************************/
void CAN_GetMotoData(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	switch (pHeader->StdId)
	{
	case 0x201:
		underpan[0].Angle = aData[0] << 8 | aData[1];
		underpan[0].Speed = aData[2] << 8 | aData[3];
		underpan[0].Current = aData[4] << 8 | aData[5];
		underpan[0].Temperature = aData[6];
	
		underpan[0].CurrentSum += underpan[0].Current;

		if (underpan[0].CurrentFlag == 1)
			underpan[0].CurrentSum -= underpan[0].CurrentStore[underpan[0].CurrentCount];

		underpan[0].CurrentStore[underpan[0].CurrentCount] = underpan[0].Current;
		underpan[0].CurrentCount++;
		if (underpan[0].CurrentCount > 9)
		{
			underpan[0].CurrentCount = 0;
			underpan[0].CurrentFlag = 1;
		}
		underpan[0].AverageCurrent = underpan[0].CurrentSum / 10;
		break;

	case 0x202:
		underpan[1].Angle = aData[0] << 8 | aData[1];
		underpan[1].Speed = aData[2] << 8 | aData[3];
		underpan[1].Current = aData[4] << 8 | aData[5];
		underpan[1].Temperature = aData[6];
		underpan[1].CurrentSum += underpan[1].Current;
	
		if (underpan[1].CurrentFlag == 1)
			underpan[1].CurrentSum -= underpan[1].CurrentStore[underpan[1].CurrentCount];
		underpan[1].CurrentStore[underpan[1].CurrentCount] = underpan[1].Current;
		underpan[1].CurrentCount++;
		if (underpan[1].CurrentCount > 9)
		{
			underpan[1].CurrentCount = 0;
			underpan[1].CurrentFlag = 1;
		}
		underpan[1].AverageCurrent = underpan[1].CurrentSum / 10;
		break;

	case 0x203:
		underpan[2].Angle = aData[0] << 8 | aData[1];
		underpan[2].Speed = aData[2] << 8 | aData[3];
		underpan[2].Current = aData[4] << 8 | aData[5];
		underpan[2].Temperature = aData[6];
		underpan[2].CurrentSum += underpan[2].Current;

		if (underpan[2].CurrentFlag == 1)
			underpan[2].CurrentSum -= underpan[2].CurrentStore[underpan[2].CurrentCount];
		underpan[2].CurrentStore[underpan[2].CurrentCount] = underpan[2].Current;
		underpan[2].CurrentCount++;
		if (underpan[2].CurrentCount > 9)
		{
			underpan[2].CurrentCount = 0;
			underpan[2].CurrentFlag = 1;
		}
		underpan[2].AverageCurrent = underpan[2].CurrentSum / 10;
		break;

	case 0x204:
		underpan[3].Angle = aData[0] << 8 | aData[1];
		underpan[3].Speed = aData[2] << 8 | aData[3];
		underpan[3].Current = aData[4] << 8 | aData[5];
		underpan[3].Temperature = aData[6];

		underpan[3].CurrentSum += underpan[3].Current;
		if (underpan[3].CurrentFlag == 1)
			underpan[3].CurrentSum -= underpan[3].CurrentStore[underpan[3].CurrentCount];
		underpan[3].CurrentStore[underpan[3].CurrentCount] = underpan[3].Current;
		underpan[3].CurrentCount++;
		if (underpan[3].CurrentCount > 9)
		{
			underpan[3].CurrentCount = 0;
			underpan[3].CurrentFlag = 1;
		}
		underpan[3].AverageCurrent = underpan[3].CurrentSum / 10;
		break;
	//pitch
	case 0x205:
		cloudPitch.Angle = aData[0] << 8 | aData[1];
		cloudPitch.Speed = aData[2] << 8 | aData[3];
		cloudPitch.Current = aData[4] << 8 | aData[5];

		if (cloudPitch.Angle > 4096)
			cloudPitch.Angle = cloudPitch.Angle - 8192;

		break;
	//yaw
	case 0x206:
		cloudYaw.Angle = aData[0] << 8 | aData[1];
		cloudYaw.Speed = aData[2] << 8 | aData[3];
		cloudYaw.Current = aData[4] << 8 | aData[5];

		if (cloudYaw.Angle > 4096)
			cloudYaw.Angle = cloudYaw.Angle - 8192;
		break;

	case 0x207:
		dan_para[0].Angle = aData[0] << 8 | aData[1];
		dan_para[0].Speed = aData[2] << 8 | aData[3];
		break;
	}
}


//发送数据
//底盘发送数据时，标识符为0x200
void CAN_SetUnderpanMotorCurrent(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
	
	Tx1Message.StdId = 0x200;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	txData[0] = iq1 >> 8;
	txData[1] = iq1;
	txData[2] = iq2 >> 8;
	txData[3] = iq2;
	txData[4] = iq3 >> 8;
	txData[5] = iq3;
	txData[6] = iq4 >> 8;
	txData[7] = iq4;
	
	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message, txData, &pTxMailbox);
}
//发送数据
//底盘发送数据时，标识符为0x1ff
void CAN_SetCloudMotorCurrent(int16_t iq1,int16_t iq2,int16_t iq3)
{
	Tx1Message.StdId = 0x1ff;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	txData[0] = iq1 >> 8;
	txData[1] = iq1;
	txData[2] = iq2 >> 8;
	txData[3] = iq2;
	txData[4] = iq3 >> 8;
	txData[5] = iq3;

	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message, txData, &pTxMailbox);
}

#
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
