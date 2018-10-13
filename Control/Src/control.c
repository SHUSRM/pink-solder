/**
  *@file control.c
  *@date 2018-10-7
  *@author Vacuo.W
  *@brief 
  */

#include "control.h"
#include "32mpu6050.h"


/************TELE***************/
uint8_t teledata_rx[18];
struct telecon_data tele_data;

/************UNDERPAN***************/
struct underpan_parameter underpan_para[4];
float underpan_P;//ok
float underpan_I;//ok
float IOUT_P;
float IOUT_I;

/************CLOUD***************/
struct cloud_parameter cloud_para[2];
int16_t pitch;
int16_t yaw;
int16_t pitch_error;
int16_t yaw_error;
int16_t pitch_error_ex;
int16_t yaw_error_ex;
float cloud_p_v_P,cloud_p_v_I,cloud_p_v_D;
float cloud_y_v_P,cloud_y_v_I,cloud_y_v_D;

/************DAN***************/
struct dan_parameter dan_para[1];
uint16_t bodan_count;
uint8_t bodan_flag;
int16_t bodan_speed;
float dan_P,dan_I,dan_D;

/************CAMERA***************/
struct CAMERA camera;

//裁判系统
struct JUDGE judge;


/****************************************/
/****************function****************/
/****************************************/

/*****接收遥控器数据*****/
void telecontroller_data(void)
{	
//	uint8_t w,a,s,d,press_l,press_r,shift;
//	int16_t speed;

	tele_data.ch0=((teledata_rx[0]| (teledata_rx[1] << 8)) & 0x07ff)-1024;						//右摇杆：左右
	tele_data.ch1=(((teledata_rx[1] >> 3) | (teledata_rx[2] << 5)) & 0x07ff)-1024;				//右摇杆：上下
	tele_data.ch2=(((teledata_rx[2] >> 6) | (teledata_rx[3] << 2) | (teledata_rx[4] << 10)) & 0x07ff)-1024;			//左：左右
	tele_data.ch3=(((teledata_rx[4] >> 1) | (teledata_rx[5] << 7)) & 0x07ff)-1024; 				//左：上下
	tele_data.s1=((teledata_rx[5] >> 4)& 0x000C) >> 2;					//左上开关：上中下对应132
	tele_data.s2=((teledata_rx[5] >> 4)& 0x0003);						//右上
	tele_data.x=teledata_rx[6] | (teledata_rx[7] << 8);
	tele_data.y=teledata_rx[8] | (teledata_rx[9] << 8); 
	tele_data.z=teledata_rx[10] | (teledata_rx[11] << 8);
	tele_data.press_l=teledata_rx[12];
	tele_data.press_r=teledata_rx[13];
	tele_data.key=teledata_rx[14] | (teledata_rx[15] << 8);
	tele_data.resv=teledata_rx[16]|(teledata_rx[17]<<8);
}
	

/*****底盘pid控制程序*****************
包含速度pid（内环）与电流pid控制（外环）
***************************************/
void underpan_pid(void)
{
	register uint8_t i;
	int16_t error;
	
	/********将遥控器数据接收********/
	underpan_para[0].set_speed=(int16_t)(1.0*(tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
	underpan_para[1].set_speed=(int16_t)(1.0*(tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
	underpan_para[2].set_speed=(int16_t)(1.0*(-tele_data.ch3-tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);
	underpan_para[3].set_speed=(int16_t)(1.0*(-tele_data.ch3+tele_data.ch2+tele_data.ch0)/660*SPEED_MAX);	
	
	
	/*******做4次pid计算********/
	for(i=0;i<4;i++)
	{
		error=underpan_para[i].set_speed-underpan_para[i].rotation_rate;
		underpan_para[i].i_interg+=underpan_I*error;
		if(underpan_para[i].i_interg>1000)underpan_para[i].i_interg=1000;
		if(underpan_para[i].i_interg<-1000)underpan_para[i].i_interg=-1000;
		underpan_para[i].motor_output=underpan_P*error+underpan_para[i].i_interg;
		if(underpan_para[i].motor_output>CURRENT_LIM)underpan_para[i].motor_output=CURRENT_LIM;
		if(underpan_para[i].motor_output<-CURRENT_LIM)underpan_para[i].motor_output=-CURRENT_LIM;
		
		error=underpan_para[i].motor_output-underpan_para[i].average_current;
		underpan_para[i].s_interg+=IOUT_I*error;
		if(underpan_para[i].s_interg>8000)underpan_para[i].s_interg=8000;
		if(underpan_para[i].s_interg<-8000)underpan_para[i].s_interg=-8000;
		underpan_para[i].i_output=IOUT_P*error+underpan_para[i].s_interg;
		if(underpan_para[i].i_output>CURRENT_MAX)underpan_para[i].i_output=CURRENT_MAX;
		if(underpan_para[i].i_output<-CURRENT_MAX)underpan_para[i].i_output=-CURRENT_MAX;
	}	
}
	
/**************云台yaw方向pid控制***************/
void cloud_y_v_pid(void)
{	
	int16_t err;
	float output;
	
	pitch=pitch_mid+1.0*tele_data.ch1;
	yaw=yaw_mid-1.0*tele_data.ch0;
	
	if(pitch<(pitch_mid-600))pitch=pitch_mid-600;
	if(pitch>(pitch_mid+600))pitch=pitch_mid+600;
	if(yaw<(yaw_mid-800))yaw=yaw_mid-800;
	if(yaw>(yaw_mid+800))yaw=yaw_mid+800;	
	
	err=cloud_para[1].Bmechanical_angle-yaw;
	cloud_para[1].iout+=cloud_y_v_I*err;
	if(cloud_para[1].iout>1000)cloud_para[1].iout=1000;
	if(cloud_para[1].iout<-1000)cloud_para[1].iout=-1000;	
	output=cloud_y_v_P*err+cloud_para[1].iout+cloud_y_v_D*sensor.gyro.radian.x;
	cloud_para[1].motor_output=(int16_t)output;
	if(cloud_para[1].motor_output>5000)cloud_para[1].motor_output=5000;
	if(cloud_para[1].motor_output<-5000)cloud_para[1].motor_output=-5000;		
}


/**************云台pitch方向pid控制***************/
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

/**************拨弹轮速度pid控制***************/
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


/*参数统一初始化*/	
void para_init(void)
{
	
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
//	cloud_y_s_P=4;
//	cloud_y_s_I=0.6;
//	cloud_y_s_D=0.5;
//	flag_yawdir=0;
	underpan_para[0].set_speed=0;
	underpan_para[1].set_speed=0;
	underpan_para[2].set_speed=0;
	underpan_para[3].set_speed=0;
	underpan_para[0].rotation_rate=0;
	underpan_para[1].rotation_rate=0;	
	underpan_para[2].rotation_rate=0;	
	underpan_para[3].rotation_rate=0;
}

	
	
	