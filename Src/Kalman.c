#include "Kalman.h"

//float K1 =0.02; 
//float angleKal, gyroKal; 

//float QangleKal=0.001;// 陀螺仪噪声的协方差 过程噪声的协方差 
//float QgyroKal=0.003;//陀螺仪漂移噪声的协方差 过程噪声的协方差 为一个一行两列矩阵
//float RangleKal=0.7;//0.5;// 加速度测量噪声的协方差 既测量偏差

//float dt=0.005;// 积分时间 滤波器采样时间5ms  

//char  H_0 = 1;// H矩阵一个数

//float gyro_bias, angle_err; //陀螺仪漂移,

//float k0,k1, E; //中间变量

//float K_0, K_1;  //K是卡尔曼增益 Kg

//float PP[4] ={0,0,0,0};  //计算P矩阵的中间变量
//float P[2][2] = { { 1, 0 },{ 0, 1 } };  //P矩阵  X 的协方差


void KalmanInit(Kalman *kal)
{
	u8 i;
	kal->QangleKal=0.001;// 陀螺仪噪声的协方差 过程噪声的协方差 
	kal->QgyroKal=0.003;//陀螺仪漂移噪声的协方差 过程噪声的协方差 为一个一行两列矩阵
	kal->RangleKal=0.5;// 加速度测量噪声的协方差 既测量偏差
	kal->dt=0.005;// 积分时间 滤波器采样时间5ms  
	for(i = 0; i<4;i++)
		kal->PP[i] = 0;
	kal->P[0][0] = 1;
	kal->P[0][1] = 0;
	kal->P[1][0] = 0;
	kal->P[1][1] = 1;
}

//简易卡尔曼滤波
void KalmanFilter(float accelAngle,float Gyro, Kalman *kal)		
{
	//X(k|k-1)=AX(k-1|k-1)+BU(k)……….先验估计
	//|angle    | = |1  -dt||angle    | + |dt|gryo
	//|gyro_bias|	|0   1 ||gyro_bias|   |0 |
	kal->Angle +=(Gyro - kal->gyro_bias) * kal->dt; 
	
/*	P(k|k-1)=A P(k-1| k-1) AT+Q……….误差协方差
	Q = |cov(angle,angle) cov(angle,gyro)| = |QangleKal     0 |
	    |cov(gyro,angle)   cov(gyro,gyro)|	 |0       QgyroKal|
	P(k-1| k-1) = |a   b|
	              |c   d|
	P(k|k-1) = |a-c*dt-b*dt+d*dt*dt   b-d*dt| + |QangleKal     0 |  (dt*dt 太小可以省略）
	           |      c-d*dt            d   |   |0       QgyroKal|
	
		    = |-c-b     -d| *dt+ |a   b| + |QangleKal     0 | 
			  | -d      0 |      |c   d|   |0       QgyroKal|
*/
	kal->PP[0] = -kal->P[0][1] - kal->P[1][0]; 
	kal->PP[1] = -kal->P[1][1];
	kal->PP[2] = -kal->P[1][1];
	kal->PP[3] = 0;
	
	kal->P[0][0] = kal->PP[0] * kal->dt + kal->P[0][0] + kal->QangleKal;   
	kal->P[0][1] = kal->PP[1] * kal->dt + kal->P[0][1];  
	kal->P[1][0] = kal->PP[2] * kal->dt + kal->P[1][0];
	kal->P[1][1] = kal->PP[3] * kal->dt + kal->P[1][1] + kal->QgyroKal;
/*
	Kg(k)= P(k|k-1)*HT / (H*P(k|k-1) HT + R)……….计算卡尔曼增益
	Kg 对应angle和gyro_bias的卡尔曼增益 |K_0|
										|K_1|
	H = |1   0|   （观测变量Z 可由测量值accelAngle直接得出，gyro_bias和accelAngle无关	
	R为加速度计测量出的角度值的噪声
	P(k|k-1)*HT = |a   b| * | 1 | = | a |
	              |c   d|   | 0 |   | c |
	H*P(k|k-1)*HT = |1  0| * |a   b| * | 1 | = a
                             |c   d|   | 0 |
*/
	kal->k0 = kal->P[0][0];  
	kal->k1 = kal->P[1][0];
	
	kal->E = kal->RangleKal + kal->P[0][0];  // 分母 
	
	kal->K_0 = kal->k0 / kal->E;
	kal->K_1 = kal->k1 / kal->E;
	

/*	X(k|k)= X(k|k-1) + Kg(k)(Z(k) - H*X(k|k-1))……….修正估计
	Z = accelAngle
	H*X(k|k-1)) = |1   0| * |angle    | = angle
	                        |gyro_bias|
	X(k|k) = |angle    | + |K_0|  * angle
	         |gyro_bias|   |K_1|
*/
	kal->angle_err = accelAngle - kal->Angle;	//zk-先验估计
	kal->Angle += kal->K_0 * kal->angle_err;	 //后验估计
	kal->gyro_bias += kal->K_1 * kal->angle_err;	 //后验估计
	kal->Gyro = Gyro - kal->gyro_bias;	 //输出值(后验估计)的微分=角速度
/*
	P(k|k)=（ I-Kg(k) H） P(k|k-1)……….更新误差协方差
	I = |1  0|
	    |0  1|
	Kg(k) H P(k|k-1) = |K_0| * |1  0| * |a   b| = |K_0*a   K_0*b|
                       |K_1|            |c   d|   |K_1*a   K_1*b|

*/
	
	kal->P[0][0] -= kal->K_0 * kal->P[0][0];		 //后验估计误差协方差
	kal->P[0][1] -= kal->K_0 * kal->P[0][1];
	kal->P[1][0] -= kal->K_1 * kal->P[0][0];
	kal->P[1][1] -= kal->K_1 * kal->P[0][1];
	
}

