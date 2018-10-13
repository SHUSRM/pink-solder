/**
  *@file can.c
  *@date 2018-10-7
  *@author Vacuo.W
  *@brief 
  */

#include "can.h"
#include "control.h"

uint8_t TxData[8];
uint32_t pTxMailbox;

CAN_TxHeaderTypeDef  Tx1Message;		//发送配置参数
CAN_RxHeaderTypeDef  Rx1Message;		//接收配置参数

void CAN1_Init()						
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
	canfilter.FilterBank=0;
//    canfilter.FilterNumber = 0;
//    hcan1.pTxMsg = &Tx1Message;
//    hcan1.pRxMsg = &Rx1Message;
  

	HAL_CAN_ConfigFilter(&hcan1,&canfilter);
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_CAN_Start(&hcan1);
}


/***************************************
底盘电调id：201到204
云台电调id：205到206
拨弹电调id：207
***************************************/

void CAN_Getdata(CAN_HandleTypeDef *hcan,CAN_RxHeaderTypeDef *pHeader,uint8_t aData[])
{
	switch(pHeader->StdId)
  {
    case 0x201:
    {
			underpan_para[0].mechanical_angle=aData[0]<<8|aData[1];
			underpan_para[0].rotation_rate=aData[2]<<8|aData[3];
			underpan_para[0].motor_current=aData[4]<<8|aData[5];
			underpan_para[0].motor_temperature=aData[6];		
						
			underpan_para[0].sum_current+=underpan_para[0].motor_current;
			if(underpan_para[0].current_flag==1)underpan_para[0].sum_current-=underpan_para[0].current_store[underpan_para[0].current_count];
			underpan_para[0].current_store[underpan_para[0].current_count]=underpan_para[0].motor_current;
			underpan_para[0].current_count++;
			if(underpan_para[0].current_count>9){underpan_para[0].current_count=0;underpan_para[0].current_flag=1;}	
			underpan_para[0].average_current=underpan_para[0].sum_current/10;			
    }break;
    case 0x202:
    {
			underpan_para[1].mechanical_angle=aData[0]<<8|aData[1];
			underpan_para[1].rotation_rate=aData[2]<<8|aData[3];
			underpan_para[1].motor_current=aData[4]<<8|aData[5];
			underpan_para[1].motor_temperature=aData[6];
			
			underpan_para[1].sum_current+=underpan_para[1].motor_current;
			if(underpan_para[1].current_flag==1)underpan_para[1].sum_current-=underpan_para[1].current_store[underpan_para[1].current_count];
			underpan_para[1].current_store[underpan_para[1].current_count]=underpan_para[1].motor_current;
			underpan_para[1].current_count++;
			if(underpan_para[1].current_count>9){underpan_para[1].current_count=0;underpan_para[1].current_flag=1;}	
			underpan_para[1].average_current=underpan_para[1].sum_current/10;
    }break;
    case 0x203:
    {
			underpan_para[2].mechanical_angle=aData[0]<<8|aData[1];
			underpan_para[2].rotation_rate=aData[2]<<8|aData[3];
			underpan_para[2].motor_current=aData[4]<<8|aData[5];
			underpan_para[2].motor_temperature=aData[6];		
			
			underpan_para[2].sum_current+=underpan_para[2].motor_current;
			if(underpan_para[2].current_flag==1)underpan_para[2].sum_current-=underpan_para[2].current_store[underpan_para[2].current_count];
			underpan_para[2].current_store[underpan_para[2].current_count]=underpan_para[2].motor_current;
			underpan_para[2].current_count++;
			if(underpan_para[2].current_count>9){underpan_para[2].current_count=0;underpan_para[2].current_flag=1;}	
			underpan_para[2].average_current=underpan_para[2].sum_current/10;
    }break;
    case 0x204:
    {
			underpan_para[3].mechanical_angle=aData[0]<<8|aData[1];
			underpan_para[3].rotation_rate=aData[2]<<8|aData[3];
			underpan_para[3].motor_current=aData[4]<<8|aData[5];
			underpan_para[3].motor_temperature=aData[6];	
			
			underpan_para[3].sum_current+=underpan_para[3].motor_current;
			if(underpan_para[3].current_flag==1)underpan_para[3].sum_current-=underpan_para[3].current_store[underpan_para[3].current_count];
			underpan_para[3].current_store[underpan_para[3].current_count]=underpan_para[3].motor_current;
			underpan_para[3].current_count++;
			if(underpan_para[3].current_count>9){underpan_para[3].current_count=0;underpan_para[3].current_flag=1;}	
			underpan_para[3].average_current=underpan_para[3].sum_current/10;
    }break;	
    case 0x205:
		  cloud_para[0].mechanical_angle=aData[0]<<8|aData[1];
			cloud_para[0].torque=aData[2]<<8|aData[3];
			cloud_para[0].torque_current=aData[4]<<8|aData[5];
		  if(cloud_para[0].mechanical_angle<4096) cloud_para[0].Bmechanical_angle=cloud_para[0].mechanical_angle;
		  if(cloud_para[0].mechanical_angle>4096) cloud_para[0].Bmechanical_angle=cloud_para[0].mechanical_angle-8192;
						
		break;
		case 0x206:
			cloud_para[1].mechanical_angle=aData[0]<<8|aData[1];
			cloud_para[1].torque=aData[2]<<8|aData[3];
			cloud_para[1].torque_current=aData[4]<<8|aData[5];
		  if(cloud_para[1].mechanical_angle<4096) cloud_para[1].Bmechanical_angle=cloud_para[1].mechanical_angle;
		  if(cloud_para[1].mechanical_angle>4096) cloud_para[1].Bmechanical_angle=cloud_para[1].mechanical_angle-8192;
	
		break;	
    case 0x207:
 		dan_para[0].mechanical_angle=aData[0]<<8|aData[1];
		dan_para[0].speed=aData[2]<<8|aData[3];
					
		break;					
	}
	
}


/*发送数据
底盘发送数据时，标识符为0x200*/
void underpan_motor_output(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
	
	Tx1Message.StdId = 0x200;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	TxData[0] = iq1 >> 8;
	TxData[1] = iq1;
	TxData[2] = iq2 >> 8;
	TxData[3] = iq2;
	TxData[4] = iq3 >> 8;
	TxData[5] = iq3;
	TxData[6] = iq4 >> 8;
	TxData[7] = iq4;
	
	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,  TxData, &pTxMailbox);
}

/*发送数据
底盘发送数据时，标识符为0x1ff*/
void cloud_motor_output(int16_t iq1,int16_t iq2,int16_t iq3)
{
	Tx1Message.StdId = 0x1ff;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	TxData[0] = iq1 >> 8;
	TxData[1] = iq1;
	TxData[2] = iq2 >> 8;
	TxData[3] = iq2;
	TxData[4] = iq3 >> 8;
	TxData[5] = iq3;

	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,  TxData, &pTxMailbox);
}





