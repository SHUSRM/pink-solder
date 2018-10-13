/**
  *@file test_can.c
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */

#include "test_can.h"
#include "can.h"
#include "usart.h"
#include "test_control.h"

struct CAMERA camera;


//can filter must be initialized before use
void CanFilter_Init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterConfTypeDef canfilter;
  
  //create memory to save the message, if not will raise error
  static CanTxMsgTypeDef  Tx1Message;
  static CanRxMsgTypeDef  Rx1Message;
  //static CanTxMsgTypeDef  Tx2Message;
  //static CanRxMsgTypeDef  Rx2Message;
  
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  
  //filtrate any ID you want here
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;
  
  canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
  canfilter.FilterActivation = ENABLE;
  canfilter.BankNumber = 14;
  
  //use different filter for can1&can2
  if(hcan == &hcan1)
  {
    canfilter.FilterNumber = 0;
    hcan->pTxMsg = &Tx1Message;
    hcan->pRxMsg = &Rx1Message;
  }
 /* if(hcan == &hcan2)
  {
    canfilter.FilterNumber = 14;
    hcan->pTxMsg = &Tx2Message;
    hcan->pRxMsg = &Rx2Message;
  }*/
  
  HAL_CAN_ConfigFilter(hcan, &canfilter);
  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);	
}

//it will be auto callback when can receive msg completely
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{	
  switch(hcan->pRxMsg->StdId)
  {
    case 0x201:
    {
			underpan_para[0].mechanical_angle=hcan->pRxMsg->Data[0]<<8|hcan->pRxMsg->Data[1];
			underpan_para[0].rotation_rate=hcan->pRxMsg->Data[2]<<8|hcan->pRxMsg->Data[3];
			underpan_para[0].motor_current=hcan->pRxMsg->Data[4]<<8|hcan->pRxMsg->Data[5];
			underpan_para[0].motor_temperature=hcan->pRxMsg->Data[6];		
						
			sum_current[0]+=underpan_para[0].motor_current;
			if(current_flag[0]==1)sum_current[0]-=current_store[0][current_count0];
			current_store[0][current_count0]=underpan_para[0].motor_current;
			current_count0++;
			if(current_count0>9){current_count0=0;current_flag[0]=1;}			
    }break;
    case 0x202:
    {
			underpan_para[1].mechanical_angle=hcan->pRxMsg->Data[0]<<8|hcan->pRxMsg->Data[1];
			underpan_para[1].rotation_rate=hcan->pRxMsg->Data[2]<<8|hcan->pRxMsg->Data[3];
			underpan_para[1].motor_current=hcan->pRxMsg->Data[4]<<8|hcan->pRxMsg->Data[5];
			underpan_para[1].motor_temperature=hcan->pRxMsg->Data[6];
			
			sum_current[1]+=underpan_para[1].motor_current;
			if(current_flag[1]==1)sum_current[1]-=current_store[1][current_count1];
			current_store[1][current_count1]=underpan_para[1].motor_current;
			current_count1++;
			if(current_count1>9){current_count1=0;current_flag[1]=1;}
    }break;
    case 0x203:
    {
			underpan_para[2].mechanical_angle=hcan->pRxMsg->Data[0]<<8|hcan->pRxMsg->Data[1];
			underpan_para[2].rotation_rate=hcan->pRxMsg->Data[2]<<8|hcan->pRxMsg->Data[3];
			underpan_para[2].motor_current=hcan->pRxMsg->Data[4]<<8|hcan->pRxMsg->Data[5];
			underpan_para[2].motor_temperature=hcan->pRxMsg->Data[6];		
			
			sum_current[2]+=underpan_para[2].motor_current;
			if(current_flag[2]==1)sum_current[2]-=current_store[2][current_count2];
			current_store[2][current_count2]=underpan_para[2].motor_current;
			current_count2++;
			if(current_count2>9){current_count2=0;current_flag[2]=1;}
    }break;
    case 0x204:
    {
			underpan_para[3].mechanical_angle=hcan->pRxMsg->Data[0]<<8|hcan->pRxMsg->Data[1];
			underpan_para[3].rotation_rate=hcan->pRxMsg->Data[2]<<8|hcan->pRxMsg->Data[3];
			underpan_para[3].motor_current=hcan->pRxMsg->Data[4]<<8|hcan->pRxMsg->Data[5];
			underpan_para[3].motor_temperature=hcan->pRxMsg->Data[6];	
			
			sum_current[3]+=underpan_para[3].motor_current;
			if(current_flag[3]==1)sum_current[3]-=current_store[3][current_count3];
			current_store[3][current_count3]=underpan_para[3].motor_current;
			current_count3++;
			if(current_count3>9){current_count3=0;current_flag[3]=1;}
    }break;	
    case 0x205:
		  cloud_para[0].mechanical_angle=hcan->pRxMsg->Data[0]<<8|hcan->pRxMsg->Data[1];
			cloud_para[0].torque=hcan->pRxMsg->Data[2]<<8|hcan->pRxMsg->Data[3];
			cloud_para[0].torque_current=hcan->pRxMsg->Data[4]<<8|hcan->pRxMsg->Data[5];
		  if(cloud_para[0].mechanical_angle<4096) cloud_para[0].Bmechanical_angle=cloud_para[0].mechanical_angle;
		  if(cloud_para[0].mechanical_angle>4096) cloud_para[0].Bmechanical_angle=cloud_para[0].mechanical_angle-8192;
			//cloud_para[0].motor_temperature=hcan->pRxMsg->Data[6];						
		break;
		case 0x206:
			cloud_para[1].mechanical_angle=hcan->pRxMsg->Data[0]<<8|hcan->pRxMsg->Data[1];
			cloud_para[1].torque=hcan->pRxMsg->Data[2]<<8|hcan->pRxMsg->Data[3];
			cloud_para[1].torque_current=hcan->pRxMsg->Data[4]<<8|hcan->pRxMsg->Data[5];
		  if(cloud_para[1].mechanical_angle<4096) cloud_para[1].Bmechanical_angle=cloud_para[1].mechanical_angle;
		  if(cloud_para[1].mechanical_angle>4096) cloud_para[1].Bmechanical_angle=cloud_para[1].mechanical_angle-8192;
			//cloud_para[1].motor_temperature=hcan->pRxMsg->Data[6];	
		break;	
    case 0x207:
 		dan_para[0].mechanical_angle=hcan->pRxMsg->Data[0]<<8|hcan->pRxMsg->Data[1];
		dan_para[0].speed=hcan->pRxMsg->Data[2]<<8|hcan->pRxMsg->Data[3];
		//dan_para[0].torque_current=hcan->pRxMsg->Data[4]<<8|hcan->pRxMsg->Data[5];
		//cloud_para[1].motor_temperature=hcan->pRxMsg->Data[6];						
		break;						
  }
  __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
}

//CAN send message test
void underpan_motor_output(CAN_HandleTypeDef* hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
	hcan->pTxMsg->StdId = 0x200;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[0] = iq1 >> 8;
	hcan->pTxMsg->Data[1] = iq1;
	hcan->pTxMsg->Data[2] = iq2 >> 8;
	hcan->pTxMsg->Data[3] = iq2;
	hcan->pTxMsg->Data[4] = iq3 >> 8;
	hcan->pTxMsg->Data[5] = iq3;
	hcan->pTxMsg->Data[6] = iq4 >> 8;
	hcan->pTxMsg->Data[7] = iq4;

	HAL_CAN_Transmit(hcan, 10);
}

void cloud_motor_output(CAN_HandleTypeDef* hcan,int16_t iq1,int16_t iq2,int16_t iq3)
{
	hcan->pTxMsg->StdId = 0x1ff;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[0] = iq1 >> 8;
	hcan->pTxMsg->Data[1] = iq1;
	hcan->pTxMsg->Data[2] = iq2 >> 8;
	hcan->pTxMsg->Data[3] = iq2;
	hcan->pTxMsg->Data[4] = iq3 >> 8;
	hcan->pTxMsg->Data[5] = iq3;

	HAL_CAN_Transmit(hcan, 10);
}

void HAL_UART_RxCpltaCallback(UART_HandleTypeDef *huart)
{
    
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
    
 
}

