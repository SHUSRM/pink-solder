/**
  *@file test_app.c
  *@date 2016-12-13
  *@author Albert.D
  *@brief 
  */
  
#include "test_app.h"
#include "test_beep.h"
#include "test_can.h"
#include "test_uart.h"
#include "test_imu.h"
#include "mpu6500_reg.h"
#include "can.h"
#include "usart.h"
#include "test_control.h"
#include "32mpu6050.h"

uint8_t Timetick2ms = 0;

/*total task used to check all,it will be used in timer 6 interrupt*/


void Test_task(void)
{
	Timetick2ms++;
	
	switch(Timetick2ms)
	{
		case 1:
//			if(!flag_once)
//			{		
//				camera_count++;
//				if(camera_count>4)
//				{
//					camera_count=0;
//					flag_once=1;
//				}
//			}
			bodan_control();				
			break;
		 
		case 2:
			cloud_p_v_pid();
			cloud_motor_output(&hcan1,cloud_para[0].motor_output,cloud_para[1].motor_output,dan_para[0].motor_output);
			break;
		
		case 3:
			if ((q==1&&e==1)||(tele_data.s1==1)) cloud_y_s_pid();
			else cloud_y_v_pid();
		  //cloud_motor_output(&hcan1,cloud_para[0].motor_output,cloud_para[1].motor_output,dan_para[0].motor_output);
			break;

		default:			
			underpan_pid();
			underpan_motor_output(&hcan1,underpan_para[0].i_output,underpan_para[1].i_output,underpan_para[2].i_output,underpan_para[3].i_output);				
		  Timetick2ms=0;
			break;
	}

}


