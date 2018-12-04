#ifndef __FILTER_H
#define __FILTER_H

#define PI 3.1415926f

#include "sys.h"
typedef struct
{
	float Gyro;
	float Angle; 
	
	float gyro_bias;
	float angle_err;
	
	float QangleKal;// 陀螺仪噪声的协方差 过程噪声的协方差 
	float QgyroKal;//陀螺仪漂移噪声的协方差 过程噪声的协方差 为一个一行两列矩阵
	float RangleKal;// 加速度测量噪声的协方差 既测量偏差
	
	float dt;// 积分时间 滤波器采样时间5ms  

	char  H_0;// H矩阵一个数

	float k0,k1, E; //中间变量

	float K_0, K_1;  //K是卡尔曼增益 Kg

	float PP[4];  //计算P矩阵的中间变量
	float P[2][2];  //P矩阵  X 的协方差
	
}Kalman;


extern float angleKal, gyroKal; 	
void KalmanFilter(float accelAngle,float Gyro, Kalman *kal);		

#endif
