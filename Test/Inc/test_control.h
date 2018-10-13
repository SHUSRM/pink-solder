#ifndef _TEST__CONTROL_H
#define _TEST__CONTROL_H

#include "stm32f4xx_HAL.h"


#define pitch_mid  800
#define yaw_mid 300
#define CURRENT_MAX 16000
#define SPEED_MAX 5000
#define CURRENT_LIM 4000

/*****teler*******/
struct telecon_data
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
};	

struct underpan_parameter 
{
	uint16_t mechanical_angle;
	int16_t rotation_rate;
	int16_t motor_current;
	uint16_t motor_temperature;
	int16_t motor_output;
	int16_t set_speed;
	int16_t i_output;
	int16_t i_interg;
	int16_t s_interg;
};

typedef struct cloud_parameter 
{
	uint16_t mechanical_angle;//机械角度
	int16_t Bmechanical_angle;//变换后角度
	int16_t torque;//转矩电流测量
	int16_t torque_current;//转矩电流给定
	int16_t iout;

	int16_t motor_output;
	int16_t set_speed;
} cloud_parameter;

typedef struct dan_parameter 
{
	uint16_t mechanical_angle;//机械角度
	
	int16_t speed;//转矩电流测量
	int16_t torque_current;//转矩电流给定
	
	int16_t error[3];
	int16_t motor_output;
} dan_parameter;

typedef struct CAMERA
{
    uint8_t recieve[1];
    uint8_t count;
    uint8_t transmit[1];
    uint16_t x;
    uint16_t y;
    uint8_t sum;
} CAMERA;

extern int16_t yaw;
extern int16_t pitch;
extern uint8_t teledata_rx[18];
extern uint8_t camera_rx[1];
//extern uint8_t flag_once;
//extern uint8_t camera_count;

extern int16_t pitch_error;
extern int16_t yaw_error;
extern int16_t pitch_error_ex;
extern int16_t yaw_error_ex;
extern uint8_t caipan_rx[40];
extern struct CAMERA camera;
extern struct telecon_data tele_data;
extern struct underpan_parameter underpan_para[4];
extern struct cloud_parameter cloud_para[2];
extern struct dan_parameter dan_para[1];
extern int16_t average_current[4];
extern int32_t sum_current[4];
extern uint8_t current_count0;
extern uint8_t current_count1;
extern uint8_t current_count2;
extern uint8_t current_count3;
extern int16_t current_store[4][10];
extern uint8_t current_flag[4];
extern int16_t bodan_speed;
extern uint8_t q;
extern uint8_t e;


void Bodan_pid(void);
void bodan_control(void);
void telecontroller_data(void);
void camera_data(void);
void caipan_data(void);
void underpan_pid(void);
void cloud_p_v_pid(void);
void cloud_y_v_pid(void);
void current_cal(void);
void para_init(void);

#endif
