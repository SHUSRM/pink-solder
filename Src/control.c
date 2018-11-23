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

//����ϵͳ
Judge judge;

MotorPara underpan[4];
MotorPara cloudPitch;
MotorPara cloudYaw;
PID_t *pidAdjust;

void MOTO_ControlInit()
{
	//ѡ����λ��PID���ζ���
	pidAdjust = &(underpan[2].SpeedPID);

	cloudPitch.SetAngle = PITCH_MID;
	cloudYaw.SetAngle = YAW_MID;
	
	//PID_StructInit(PID_t *pid,uint32_t mode,uint32_t maxout,uint32_t intergral_limit,float kp,float ki,float kd)
	for (int i = 0; i < 4; i++)
	{
		PID_StructInit(&(underpan[i].SpeedPID), POSITION_PID, CURRENT_LIM, 1000,
					   0,0,0); //0.035f, 0.0f);
		PID_StructInit(&(underpan[i].CurrentPID), POSITION_PID, CURRENT_LIM, 1000,
					   1.8f,0,0);// 0.02f, 0.0f);
	}
	PID_StructInit(&(cloudPitch.AnglePID), POSITION_PID, 5000, 1000,
				   5, 0.01f, -1.3f);
	PID_StructInit(&(cloudYaw.AnglePID), POSITION_PID, 5000, 1000,
				   5, 0.01f, 2.7f);
}

/*****����ң��������*****/
void telecontroller_data(void)
{
	//	uint8_t w,a,s,d,press_l,press_r,shift;
	//	int16_t speed;

	tele_data.ch0 = ((teledata_rx[0] | (teledata_rx[1] << 8)) & 0x07ff) - 1024;									//��ҡ�ˣ�����
	tele_data.ch1 = (((teledata_rx[1] >> 3) | (teledata_rx[2] << 5)) & 0x07ff) - 1024;							//��ҡ�ˣ�����
	tele_data.ch2 = (((teledata_rx[2] >> 6) | (teledata_rx[3] << 2) | (teledata_rx[4] << 10)) & 0x07ff) - 1024; //������
	tele_data.ch3 = (((teledata_rx[4] >> 1) | (teledata_rx[5] << 7)) & 0x07ff) - 1024;							//������
	tele_data.s1 = ((teledata_rx[5] >> 4) & 0x000C) >> 2;														//���Ͽ��أ������¶�Ӧ132
	tele_data.s2 = ((teledata_rx[5] >> 4) & 0x0003);															//����
	tele_data.x = teledata_rx[6] | (teledata_rx[7] << 8);
	tele_data.y = teledata_rx[8] | (teledata_rx[9] << 8);
	tele_data.z = teledata_rx[10] | (teledata_rx[11] << 8);
	tele_data.press_l = teledata_rx[12];
	tele_data.press_r = teledata_rx[13];
	tele_data.key = teledata_rx[14] | (teledata_rx[15] << 8);
	tele_data.resv = teledata_rx[16] | (teledata_rx[17] << 8);
}
/*****����pid���Ƴ���*****************
�����ٶ�pid���ڻ��������pid���ƣ��⻷��
***************************************/
void underpanPID()
{
	u8 i;
	/********��ң�������ݽ���********/
	underpan[0].SetSpeed = 0;//(int16_t)(1.0 * (tele_data.ch3 + tele_data.ch2 + tele_data.ch0) / 660 * SPEED_MAX);
	underpan[1].SetSpeed = 0;//(int16_t)(1.0 * (tele_data.ch3 - tele_data.ch2 + tele_data.ch0) / 660 * SPEED_MAX);
	underpan[2].SetSpeed = 500;//(int16_t)(1.0 * (-tele_data.ch3 - tele_data.ch2 + tele_data.ch0) / 660 * SPEED_MAX);
	underpan[3].SetSpeed = 0;//(int16_t)(1.0 * (-tele_data.ch3 + tele_data.ch2 + tele_data.ch0) / 660 * SPEED_MAX);

	for (i = 2; i < 3; i++)
	{
		underpan[i].CurrentOutput = PID_Calc(&(underpan[i].SpeedPID),
											 underpan[i].RotateSpeed, underpan[i].SetSpeed);
//		underpan[i].CurrentOutput = PID_Calc(&(underpan[i].CurrentPID),
//											 underpan[i].TorqueCurrent, underpan[i].CurrentOutput);
	}
}
/**************��̨pitch����pid����***************/
void cloudPitchPID()
{
	cloudPitch.SetAngle = PITCH_MID + 1.0 * tele_data.ch1;
	if (cloudPitch.SetAngle < (PITCH_MID - 600))
		cloudPitch.SetAngle = PITCH_MID - 600;
	if (cloudPitch.SetAngle > (PITCH_MID + 600))
		cloudPitch.SetAngle = PITCH_MID + 600;

	cloudPitch.CurrentOutput = PID_SpecialCalc(&(cloudPitch.AnglePID),
											   cloudPitch.MechanicalAngle, cloudPitch.SetAngle, sensor.Gyro.Origin.y);
}
/**************��̨yaw����pid����***************/
void cloudYawPID()
{
	cloudYaw.SetAngle = YAW_MID - 1.0 * tele_data.ch0;
	if (cloudYaw.SetAngle < (YAW_MID - 800))
		cloudYaw.SetAngle = YAW_MID - 800;
	if (cloudYaw.SetAngle > (YAW_MID + 800))
		cloudYaw.SetAngle = YAW_MID + 800;

	cloudYaw.CurrentOutput = PID_SpecialCalc(&(cloudYaw.AnglePID),
											 cloudPitch.MechanicalAngle, cloudPitch.SetAngle, sensor.Gyro.Origin.x);
}
