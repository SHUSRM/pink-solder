
#include "stm32f4xx_hal.h"

#define RtA 		57.324841		//  180/3.1415  角度制 转化为弧度制		
#define AtR    	0.0174533		//  1/RtA             RtA倒数		
#define Acc_G 	0.0011963		//  1/32768/4/9.8     加速度量程为4G		
#define Gyro_G 	0.03051756	//  1/32768/1000      陀螺仪量程为 +—1000			
#define Gyro_Gr	0.0005326   //  1/32768/1000/57.3 



void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);




struct _imu_angle{
        float ipitch;
				float roll;
        float iyaw;};


extern struct _imu_angle IMangle;













