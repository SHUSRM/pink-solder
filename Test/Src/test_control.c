#include "test_control.h"
#include "test_can.h"
#include "32mpu6050.h"
#include "test_uart.h"

/*
float underpan_P=10;
float underpan_I=0.15;
float underpan_D=0.2;
*/
struct underpan_parameter underpan_para[4];
float underpan_P;//ok
float underpan_I;//ok
float IOUT_P;
float IOUT_I;

uint8_t teledata_rx[18];
struct telecon_data tele_data;
uint8_t tele_count;
int16_t tele_buff[4][7];

struct cloud_parameter cloud_para[2];
int16_t pitch;
int16_t yaw;
int16_t pitch_error;
int16_t yaw_error;
int16_t pitch_error_ex;
int16_t yaw_error_ex;
float cloud_p_v_P,cloud_p_v_I,cloud_p_v_D;
float cloud_y_v_P,cloud_y_v_I,cloud_y_v_D;

struct dan_parameter dan_para[1];
uint16_t bodan_count;
uint8_t bodan_flag;
int16_t bodan_speed;
float dan_P,dan_I,dan_D;



//uint8_t flag_once;
//uint8_t camera_count;
uint8_t caipan_rx[40];
float powerbuff;
int32_t sum_current[4];
int16_t current_store[4][10];
uint8_t current_count0;
uint8_t current_count1;
uint8_t current_count2;
uint8_t current_count3;
int16_t average_current[4];
uint8_t current_flag[4]={0};
uint8_t control;

int16_t err_last;
float cloud_y_s_P;
float cloud_y_s_I;
float cloud_y_s_D;
uint8_t flag_yawdir;


void cloud_p_v_pid(void)
{
	int16_t err;
	float output;
	
	err=cloud_para[0].Bmechanical_angle-pitch;
	cloud_para[0].iout+=cloud_p_v_I*err;
	if(cloud_para[0].iout>1000)cloud_para[0].iout=1000;
	if(cloud_para[0].iout<-1000)cloud_para[0].iout=-1000;	
	output=cloud_p_v_P*err+cloud_para[0].iout+cloud_p_v_D*sensor.gyro.radian.y;
	cloud_para[0].motor_output=(int16_t)output;
	if(cloud_para[0].motor_output>5000)cloud_para[0].motor_output=5000;
	if(cloud_para[0].motor_output<-5000)cloud_para[0].motor_output=-5000;
}


void cloud_y_v_pid(void)
{	
	int16_t err;
	float output;
	
	err=cloud_para[1].Bmechanical_angle-yaw;
	cloud_para[1].iout+=cloud_y_v_I*err;
	if(cloud_para[1].iout>1000)cloud_para[1].iout=1000;
	if(cloud_para[1].iout<-1000)cloud_para[1].iout=-1000;	
	output=cloud_y_v_P*err+cloud_para[1].iout+cloud_y_v_D*sensor.gyro.radian.x;
	cloud_para[1].motor_output=(int16_t)output;
	if(cloud_para[1].motor_output>5000)cloud_para[1].motor_output=5000;
	if(cloud_para[1].motor_output<-5000)cloud_para[1].motor_output=-5000;		
}


void Bodan_pid(void)
{
	int16_t err[2];
	float output;

	dan_para[0].error[2]=dan_para[0].error[1];			
	dan_para[0].error[1]=dan_para[0].error[0];		
	dan_para[0].error[0]=bodan_speed-dan_para[0].speed;
	err[0]=dan_para[0].error[0]-dan_para[0].error[1];
	err[1]=dan_para[0].error[0]-2*dan_para[0].error[1]+dan_para[0].error[2];

	output=dan_P*err[0]+dan_I*dan_para[0].error[0]+dan_D*err[1];
	dan_para[0].motor_output+=output;
	if(dan_para[0].motor_output>8000)dan_para[0].motor_output=8000;
	if(dan_para[0].motor_output<-8000)dan_para[0].motor_output=-8000;	

}


void bodan_control(void)
{
	if(tele_data.press_l==1&&bodan_flag==0)
	{
		bodan_speed=-660;
		bodan_flag=1;
	}

    	
    if(control==1&&bodan_flag==0)
	{
		bodan_speed=60;
		bodan_flag=1;
	}
	if(bodan_flag)
	{
		bodan_count++;
	}
	if(bodan_count==20)
	{
		bodan_flag=0;
		bodan_speed=0;
		bodan_count=0;
	}
    
        
    //if (tele_data.press_l&&tele_data.press_r==1) bodan_speed=60;
	Bodan_pid();
}


void underpan_pid(void)
{
	register uint8_t i;
	int16_t error;
	
	for(i=0;i<4;i++)
	{
		error=underpan_para[i].set_speed-underpan_para[i].rotation_rate;
		underpan_para[i].i_interg+=underpan_I*error;
		if(underpan_para[i].i_interg>1000)underpan_para[i].i_interg=1000;
		if(underpan_para[i].i_interg<-1000)underpan_para[i].i_interg=-1000;
		underpan_para[i].motor_output=underpan_P*error+underpan_para[i].i_interg;
		if(underpan_para[i].motor_output>CURRENT_LIM)underpan_para[i].motor_output=CURRENT_LIM;
		if(underpan_para[i].motor_output<-CURRENT_LIM)underpan_para[i].motor_output=-CURRENT_LIM;
		
		error=underpan_para[i].motor_output-average_current[i];
		underpan_para[i].s_interg+=IOUT_I*error;
		if(underpan_para[i].s_interg>8000)underpan_para[i].s_interg=8000;
		if(underpan_para[i].s_interg<-8000)underpan_para[i].s_interg=-8000;
		underpan_para[i].i_output=IOUT_P*error+underpan_para[i].s_interg;
		if(underpan_para[i].i_output>CURRENT_MAX)underpan_para[i].i_output=CURRENT_MAX;
		if(underpan_para[i].i_output<-CURRENT_MAX)underpan_para[i].i_output=-CURRENT_MAX;
	}		
}


uint8_t q,e;
/*****接收遥控器数据*****/
void telecontroller_data(void)
{	
	uint8_t w,a,s,d,press_l,shift;
	int16_t speed;

	tele_data.ch0=((teledata_rx[0]| (teledata_rx[1] << 8)) & 0x07ff)-1024;
	tele_data.ch1=(((teledata_rx[1] >> 3) | (teledata_rx[2] << 5)) & 0x07ff)-1024;
	tele_data.ch2=(((teledata_rx[2] >> 6) | (teledata_rx[3] << 2) | (teledata_rx[4] << 10)) & 0x07ff)-1024;
	tele_data.ch3=(((teledata_rx[4] >> 1) | (teledata_rx[5] << 7)) & 0x07ff)-1024; 
	tele_data.s1=((teledata_rx[5] >> 4)& 0x000C) >> 2;
	tele_data.s2=((teledata_rx[5] >> 4)& 0x0003);
	tele_data.x=teledata_rx[6] | (teledata_rx[7] << 8);
	tele_data.y=teledata_rx[8] | (teledata_rx[9] << 8); 
	tele_data.z=teledata_rx[10] | (teledata_rx[11] << 8);
	tele_data.press_l=teledata_rx[12];
	tele_data.press_r=teledata_rx[13];
	tele_data.key=teledata_rx[14] | (teledata_rx[15] << 8);
	tele_data.resv=teledata_rx[16]|(teledata_rx[17]<<8);
	
	w = (tele_data.key&0x01);
	s = (tele_data.key&0x02)>>1;
	a = (tele_data.key&0x04)>>2;
	d = (tele_data.key&0x08)>>3;
	shift = (tele_data.key&0x10)>>4;
	control = (tele_data.key&0x20)>>5;
	e = (tele_data.key&0x40)>>6;
	q = (tele_data.key&0x80)>>7;
	press_l = tele_data.press_l;

#if defined(COMPUTER)
    TIM4->CCR1=2500;
	if(shift==1)
	{
		speed=1700;
        TIM4->CCR1=1500;
	}
    
	else speed=3000;
    if (tele_data.press_r==1) {yaw=yaw_mid; pitch=pitch_mid;}
	underpan_para[0].set_speed= ((w-s)+(d-a)+(q-e))*speed;
	underpan_para[1].set_speed= ((w-s)-(d-a)+(q-e))*speed;
	underpan_para[2].set_speed= -((w-s)+(d-a)-(q-e))*speed;
	underpan_para[3].set_speed= -((w-s)-(d-a)-(q-e))*speed;
	pitch-=0.1*tele_data.y;
	if(q==0||e==0)yaw-=0.3*tele_data.x;
	if(yaw>yaw_mid+770||yaw<yaw_mid-770||tele_data.press_r==1)
	{
		underpan_para[0].set_speed+=20*tele_data.x;
		underpan_para[1].set_speed+=20*tele_data.x;
		underpan_para[2].set_speed+=20*tele_data.x;
		underpan_para[3].set_speed+=20*tele_data.x;
	}
#else 	
	underpan_para[0].set_speed=(int16_t)(1.0*(tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
	underpan_para[1].set_speed=(int16_t)(1.0*(tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
	underpan_para[2].set_speed=(int16_t)(1.0*(-tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
	underpan_para[3].set_speed=(int16_t)(1.0*(-tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);	

		pitch=pitch_mid+1.0*tele_data.ch1;
		yaw=yaw_mid-1.0*tele_data.ch0;
	
    if (tele_data.s2==1) {TIM4->CCR1=1500;bodan_speed=0;}
    if (tele_data.s2==3) {TIM4->CCR1=2500;bodan_speed=0;}
    if (tele_data.s2==2) {TIM4->CCR1=2500;bodan_speed=-700;}

#endif
	if(pitch<(pitch_mid-600))pitch=pitch_mid-600;
	if(pitch>(pitch_mid+600))pitch=pitch_mid+600;
	if(yaw<(yaw_mid-800))yaw=yaw_mid-800;
	if(yaw>(yaw_mid+800))yaw=yaw_mid+800;		
	if(underpan_para[0].set_speed>SPEED_MAX)underpan_para[0].set_speed=SPEED_MAX;
	if(underpan_para[1].set_speed>SPEED_MAX)underpan_para[1].set_speed=SPEED_MAX;
	if(underpan_para[2].set_speed>SPEED_MAX)underpan_para[2].set_speed=SPEED_MAX;
	if(underpan_para[3].set_speed>SPEED_MAX)underpan_para[3].set_speed=SPEED_MAX;


}


//void camera_data(void)
//{		
//	pitch_error_ex=pitch_error;
//	yaw_error_ex=yaw_error;
//	
//	if(camera_rx[0]=='-')
//	{
//		yaw_error=(int16_t)(1.0*22.8*((camera_rx[1]-0x30)*10+(camera_rx[2]-0x30)+(camera_rx[3]-0x30)*0.1));
//	}
//	if(camera_rx[0]=='+')
//	{
//		yaw_error=-((int16_t)(1.0*22.8*((camera_rx[1]-0x30)*10+(camera_rx[2]-0x30)+(camera_rx[3]-0x30)*0.1)));
//	}
//	if(camera_rx[4]=='-')
//	{
//		pitch_error=(int16_t)(1.0*22.8*((camera_rx[5]-'0')*10+(camera_rx[6]-'0')+0.1*(camera_rx[7]-'0')));
//	}
//	if(camera_rx[4]=='+')
//	{
//		pitch_error=-((int16_t)(1.0*22.8*((camera_rx[5]-'0')*10+(camera_rx[6]-'0')+0.1*(camera_rx[7]-'0'))));
//	}
//	
//	if(yaw_error>500||yaw_error<-500)yaw_error=0;
//	if(pitch_error>500||pitch_error<-500)pitch_error=0;	
//	
//	if(yaw_error!=yaw_error_ex)yaw=yaw+yaw_error;
//	if(pitch_error!=pitch_error_ex)pitch=pitch+pitch_error;
//	
//	if(pitch<(pitch_mid-600))pitch=pitch_mid-600;
//	if(pitch>(pitch_mid+600))pitch=pitch_mid+600;
//	if(yaw<(yaw_mid-800))yaw=yaw_mid-800;
//	if(yaw>(yaw_mid+800))yaw=yaw_mid+800;		
//}


void current_cal(void)
{
	average_current[0]=sum_current[0]/10;
	average_current[1]=sum_current[1]/10;
	average_current[2]=sum_current[2]/10;
	average_current[3]=sum_current[3]/10;
}


void caipan_data(void)
{
	
	
}


void cloud_y_s_pid(void)
{
	int16_t err,speed;
    int8_t spe_cou;
	float output;
	
	err_last=err;
    if(flag_yawdir) spe_cou=65;
    else spe_cou=-70;
	err=sensor.gyro.radian.x+0.1*tele_data.ch0+2.5*tele_data.x+spe_cou;
	cloud_para[1].iout+=cloud_y_s_I*err;
	if(cloud_para[1].iout>1000)cloud_para[1].iout=1000;
	if(cloud_para[1].iout<-1000)cloud_para[1].iout=-1000;	
	output=cloud_y_s_P*err+cloud_para[1].iout+cloud_y_s_D*(err_last-err);
	cloud_para[1].motor_output=(int16_t)output;
	if(cloud_para[1].motor_output>5000)cloud_para[1].motor_output=5000;
	if(cloud_para[1].motor_output<-5000)cloud_para[1].motor_output=-5000;
    
	if (cloud_para[1].Bmechanical_angle<yaw_mid-600) flag_yawdir=0;
	if (cloud_para[1].Bmechanical_angle>yaw_mid+600) flag_yawdir=1;	
	if(flag_yawdir)speed=-1900;
	else speed=1900;
	yaw=cloud_para[1].Bmechanical_angle;
	underpan_para[0].set_speed= speed;
	underpan_para[1].set_speed= speed;
	underpan_para[2].set_speed= speed;
	underpan_para[3].set_speed= speed;	

}

void para_init(void)
{
	tele_count=0;
	underpan_P=2;//ok
	underpan_I=0.035;//ok
	IOUT_P=1.8;
	IOUT_I=0.02;
	pitch=pitch_mid;
	yaw=yaw_mid;
	pitch_error=0;
	yaw_error=0;
	pitch_error_ex=0;
	yaw_error_ex=0;
	cloud_p_v_P=5;
	cloud_p_v_I=0.01;
	cloud_p_v_D=-1.3;
	cloud_y_v_P=5;
	cloud_y_v_I=0.01;
	cloud_y_v_D=2.7;
	bodan_count=0;
	bodan_flag=0;
	bodan_speed=0;
	dan_P=10;
	dan_I=0.2;
	dan_D=0.1;
//	flag_once=1;
//	camera_count=0;
	cloud_y_s_P=4;
	cloud_y_s_I=0.6;
	cloud_y_s_D=0.5;
	flag_yawdir=0;
	underpan_para[0].set_speed=0;
	underpan_para[1].set_speed=0;
	underpan_para[2].set_speed=0;
	underpan_para[3].set_speed=0;
	underpan_para[0].rotation_rate=0;
	underpan_para[1].rotation_rate=0;	
	underpan_para[2].rotation_rate=0;	
	underpan_para[3].rotation_rate=0;
}
