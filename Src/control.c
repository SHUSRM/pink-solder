#include "control.h"
/************TELE***************/
uint8_t teledata_rx[18];
TeleconData tele_data;

/************CLOUD***************/
int16_t pitch_error;
int16_t yaw_error;
int16_t pitch_error_ex;
int16_t yaw_error_ex;

/************DAN***************/
DanParameter dan_para[1];
uint16_t bodan_count;
uint8_t bodan_flag;
int16_t bodan_speed;
float dan_P, dan_I, dan_D;

/************CAMERA***************/
Camera camera;

//裁判系统
Judge judge;
//串口调节pid
RxPID rxPID;

MotorPara underpan[4];
MotorPara cloudPitch;
MotorPara cloudYaw;
//PID_t *pidAdjust;

void MOTO_ControlInit()
{
	cloudPitch.SetAngle = PITCH_MID;
	cloudYaw.SetAngle = YAW_MID;
	
	//PID_StructInit(PID_t *pid,uint32_t mode,uint32_t maxout,uint32_t intergral_limit,float kp,float ki,float kd)
	for (int i = 0; i < 4; i++)
	{
		PID_StructInit(&(underpan[i].SpeedPID), POSITION_PID, CURRENT_LIM, 1000,
					   1.5,0.1,0); //0.035f, 0.0f);
		PID_StructInit(&(underpan[i].CurrentPID), POSITION_PID, CURRENT_LIM, 1000,
					   1.8f,0,0);// 0.02f, 0.0f);
	}
	
	PID_StructInit(&(underpan[2].AnglePID), POSITION_PID, CURRENT_LIM, 1000,
				   0,0,0); //0.035f, 0.0f);
	PID_StructInit(&(cloudPitch.AnglePID), POSITION_PID, 5000, 1000,
				   -5.3,-0.1,0.46);//5, 0.01f, -1.3f);
	PID_StructInit(&(cloudPitch.SpeedPID), POSITION_PID, 1000, 500,
				   -20.4,0,0);//5.2,0.08,0);// 5, 0.01f, -1.3f);
	PID_StructInit(&(cloudYaw.AnglePID), POSITION_PID, 5000, 1000,
				   -0,0,0);//5, -0.01f, -2.7f);
}

/*****接收遥控器数据*****/
void TelecontrollerData(void)
{
	//	uint8_t w,a,s,d,press_l,press_r,shift;
	//	int16_t speed;

	tele_data.ch0 = ((teledata_rx[0] | (teledata_rx[1] << 8)) & 0x07ff) - 1024;									//右摇杆：左右
	tele_data.ch1 = (((teledata_rx[1] >> 3) | (teledata_rx[2] << 5)) & 0x07ff) - 1024;							//右摇杆：上下
	tele_data.ch2 = (((teledata_rx[2] >> 6) | (teledata_rx[3] << 2) | (teledata_rx[4] << 10)) & 0x07ff) - 1024; //左：左右
	tele_data.ch3 = (((teledata_rx[4] >> 1) | (teledata_rx[5] << 7)) & 0x07ff) - 1024;							//左：上下
	tele_data.s1 = ((teledata_rx[5] >> 4) & 0x000C) >> 2;														//左上开关：上中下对应132
	tele_data.s2 = ((teledata_rx[5] >> 4) & 0x0003);															//右上
	tele_data.x = teledata_rx[6] | (teledata_rx[7] << 8);
	tele_data.y = teledata_rx[8] | (teledata_rx[9] << 8);
	tele_data.z = teledata_rx[10] | (teledata_rx[11] << 8);
	tele_data.press_l = teledata_rx[12];
	tele_data.press_r = teledata_rx[13];
	tele_data.key = teledata_rx[14] | (teledata_rx[15] << 8);
	tele_data.resv = teledata_rx[16] | (teledata_rx[17] << 8);
}
/*****底盘pid控制程序*****************
包含速度pid（内环）与电流pid控制（外环）
***************************************/
void MOTO_UnderpanPID()
{
	u8 i;
	/********将遥控器数据接收********/
	underpan[0].SetSpeed = 0;//(int16_t)(1.0 * (tele_data.ch3 + tele_data.ch2 + tele_data.ch0) / 660 * SPEED_MAX);
	underpan[1].SetSpeed = 0;//(int16_t)(1.0 * (tele_data.ch3 - tele_data.ch2 + tele_data.ch0) / 660 * SPEED_MAX);
	//underpan[2].SetSpeed = 0;//(int16_t)(1.0 * (-tele_data.ch3 - tele_data.ch2 + tele_data.ch0) / 660 * SPEED_MAX);
	underpan[3].SetSpeed = 0;//(int16_t)(1.0 * (-tele_data.ch3 + tele_data.ch2 + tele_data.ch0) / 660 * SPEED_MAX);

	for (i = 2; i < 3; i++)
	{
//		underpan[i].SetSpeed = PID_Calc(&(underpan[i].AnglePID),
//											 underpan[i].Angle, 4696);
		underpan[i].CurrentOutput = PID_Calc(&(underpan[i].SpeedPID),
											 underpan[i].Speed, underpan[i].SetSpeed);
//		underpan[i].CurrentOutput = PID_Calc(&(underpan[i].CurrentPID),
//											 underpan[i].Current, underpan[i].CurrentOutput);
	}
}
/**************云台pitch方向pid控制***************/
void MOTO_CloudPitchPID()
{
	cloudPitch.SetAngle = PITCH_MID + 1.0 * tele_data.ch1;
	if (cloudPitch.SetAngle < (PITCH_MID - 600))
		cloudPitch.SetAngle = PITCH_MID - 600;
	if (cloudPitch.SetAngle > (PITCH_MID + 600))
		cloudPitch.SetAngle = PITCH_MID + 600;
	
//	cloudPitch.SetAngle = 90;

	//cloudPitch.SetAngle = 7300;
//	cloudPitch.CurrentOutput = PID_Calc(&(cloudPitch.AnglePID),
//											   cloudPitch.Angle, cloudPitch.SetAngle);

	cloudPitch.AnglePID.i = -0.02*(600-ABS(cloudPitch.Angle-7300))/600;
	cloudPitch.CurrentOutput = PID_SpecialCalc(&(cloudPitch.AnglePID),
											   cloudPitch.Angle, cloudPitch.SetAngle, sensor.Gyro.Origin.y);
//	cloudPitch.SetSpeed = PID_SpecialCalc(&(cloudPitch.AnglePID),
//											   angleKal, cloudPitch.SetAngle, sensor.Gyro.Origin.y);
//	cloudPitch.SetSpeed =1;
//	cloudPitch.CurrentOutput = PID_Calc(&(cloudPitch.SpeedPID),
//											   gyroKal, cloudPitch.SetSpeed);

}
/**************云台yaw方向pid控制***************/
void MOTO_CloudYawPID()
{
	cloudYaw.SetAngle = YAW_MID - 1.0 * tele_data.ch0;
	if (cloudYaw.SetAngle < (YAW_MID - 800))
		cloudYaw.SetAngle = YAW_MID - 800;
	if (cloudYaw.SetAngle > (YAW_MID + 800))
		cloudYaw.SetAngle = YAW_MID + 800;
	
	//cloudYaw.AnglePID.i = -0.01*(800-ABS(cloudYaw.Angle-YAW_MID))/800;
	cloudYaw.CurrentOutput = PID_SpecialCalc(&(cloudYaw.AnglePID),
											 cloudYaw.Angle, cloudYaw.SetAngle, sensor.Gyro.Origin.x);
}
