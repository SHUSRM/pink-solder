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
	
	float QangleKal;// ������������Э���� ����������Э���� 
	float QgyroKal;//������Ư��������Э���� ����������Э���� Ϊһ��һ�����о���
	float RangleKal;// ���ٶȲ���������Э���� �Ȳ���ƫ��
	
	float dt;// ����ʱ�� �˲�������ʱ��5ms  

	char  H_0;// H����һ����

	float k0,k1, E; //�м����

	float K_0, K_1;  //K�ǿ��������� Kg

	float PP[4];  //����P������м����
	float P[2][2];  //P����  X ��Э����
	
}Kalman;


extern float angleKal, gyroKal; 	
void KalmanFilter(float accelAngle,float Gyro, Kalman *kal);		

#endif
