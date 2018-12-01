#ifndef __FILTER_H
#define __FILTER_H



extern float angleKal, gyroKal; 	
float KalmanFilter(float Accel,float Gyro);		

#endif
