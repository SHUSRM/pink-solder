/**
  *@file timer.c
  *@date 2018-10-7
  *@author Vacuo.W
  *@brief 
  */

#include "timer.h"
#include "control.h"
#include "can.h"

uint8_t Timetick2ms = 0;

void Test_task(void)
{
	Timetick2ms++;
	
	switch(Timetick2ms)
	{
		case 1:

							
			break;
		 
		case 2:
			cloud_p_v_pid();

			break;
		
		case 3:
			cloud_y_v_pid();
			cloud_motor_output(cloud_para[0].motor_output,cloud_para[1].motor_output,dan_para[0].motor_output);

			break;

		default:			
			underpan_pid();						//µ×ÅÌµç»úpidÔËËã

			underpan_motor_output(underpan_para[0].i_output,underpan_para[1].i_output,
								  underpan_para[2].i_output,underpan_para[3].i_output);
			Timetick2ms=0;
			break;
	}
	
	
	
}
