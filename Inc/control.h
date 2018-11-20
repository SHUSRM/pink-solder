#ifndef _CONTROL_H
#define _CONTROL_H

#include "stm32f4xx_HAL.h"
#include "sys.h"
#include "pid.h"
#include "mpu6050.h"
#include "beep.h"

#define CURRENT_LIM 4000				//电流最小值
#define CURRENT_MAX 16000				//电流最大值
#define SPEED_MAX 5000
#define PITCH_MID  800					//云台pitch轴初值
#define YAW_MID 300						//云台yaw轴初值
 

enum{
    SPEED = 0,
    CURRENT 	,
    POSITION 	,
};

/*****teler*******/
typedef struct 
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	uint8_t s1;
	uint8_t s2;
	
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;
	
	uint16_t key;   

	uint16_t resv;
}TeleconData;

typedef struct _moto
{
	uint16_t MechanicalAngle;		//转子机械角度
	int16_t RotateSpeed;			//转子转速
	int16_t TorqueCurrent;			//转矩电流
	uint16_t MotorTemperature;		//电机温度
	
	uint8_t CurrentFlag;
	int16_t CurrentStore[10];
	uint8_t CurrentCount;
	int16_t AverageCurrent;
	int32_t CurrentSum;
	int16_t SetSpeed;
	int16_t SetAngle;
	
	PID_t SpeedPID;
	PID_t CurrentPID;
	PID_t AnglePID;
//	int16_t MotorOutput;	
	int16_t CurrentOutput;

}MotorPara;
	
	
typedef struct
{
	uint16_t MechanicalAngle;//机械角度
	
	int16_t Speed;//转矩电流测量
	int16_t TorqueCurrent;//转矩电流给定
	
	int16_t Error[3];
	int16_t MotorOutput;
} DanParameter;

typedef struct
{
	uint16_t MechanicalAngle;		//机械角度
	int16_t B_MechanicalAngle;		//变换后角度
	int16_t RotateSpeed;			//转速
	int16_t TorqueCurrent;			//转矩电流
	int16_t iOut;

	int16_t MotorOutput;
	int16_t SetSpeed;
} CloudParameter;


typedef struct
{
    uint8_t Recieve[1];
    uint8_t Count;
    uint8_t Transmit[1];
    uint16_t X;
    uint16_t Y;
    uint8_t Sum;
} Camera;

typedef struct
{
    uint8_t Recieve[1];
    uint8_t Count;
    uint8_t Transmit[1];
//    uint16_t x;
//    uint16_t y;
//    uint8_t sum;
} Judge;



extern uint8_t teledata_rx[18];
extern TeleconData tele_data;

extern PID_t* pidAdjust;
extern MotorPara underpan[4];
extern MotorPara cloudPitch;
extern MotorPara cloudYaw;

extern DanParameter dan_para[1];
extern Camera camera;
extern Judge judge;

void telecontroller_data(void);
void MOTO_ControlInit(void);
void underpanPID(void);
void cloudPitchPID(void);
void cloudYawPID(void);



#endif