#include "Kalman.h"

float K1 =0.02; 
float angleKal, gyroKal; 

float QangleKal=0.001;// ������������Э���� ����������Э���� 
float QgyroKal=0.003;//������Ư��������Э���� ����������Э���� Ϊһ��һ�����о���
float RangleKal=0.7;//0.5;// ���ٶȲ���������Э���� �Ȳ���ƫ��

float dt=0.005;// ����ʱ�� �˲�������ʱ��5ms  

char  H_0 = 1;// H����һ����

float gyro_bias, angle_err; //������Ư��,

float k0,k1, E; //�м����

float K_0, K_1;  //K�ǿ��������� Kg

float PP[4] ={0,0,0,0};  //����P������м����
float P[2][2] = { { 1, 0 },{ 0, 1 } };  //P����  X ��Э����


//���׿������˲�

float KalmanFilter(float accelAngle,float Gyro)		
{
	//X(k|k-1)=AX(k-1|k-1)+BU(k)������.�������
	//|angle    | = |1  -dt||angle    | + |dt|gryo
	//|gyro_bias|	|0   1 ||gyro_bias|   |0 |
	angleKal+=(Gyro - gyro_bias) * dt; 
	
/*	P(k|k-1)=A P(k-1| k-1) AT+Q������.���Э����
	Q = |cov(angle,angle) cov(angle,gyro)| = |QangleKal     0 |
	    |cov(gyro,angle)   cov(gyro,gyro)|	 |0       QgyroKal|
	P(k-1| k-1) = |a   b|
	              |c   d|
	P(k|k-1) = |a-c*dt-b*dt+d*dt*dt   b-d*dt| + |QangleKal     0 |  (dt*dt ̫С����ʡ�ԣ�
	           |      c-d*dt            d   |   |0       QgyroKal|
	
		    = |-c-b     -d| *dt+ |a   b| + |QangleKal     0 | 
			  | -d      0 |      |c   d|   |0       QgyroKal|
*/
	PP[0] = -P[0][1] - P[1][0]; 
	PP[1] = -P[1][1];
	PP[2] = -P[1][1];
	PP[3] = 0;
	
	P[0][0] = PP[0] * dt + P[0][0] + QangleKal;   
	P[0][1] = PP[1] * dt + P[0][1];  
	P[1][0] = PP[2] * dt + P[1][0];
	P[1][1] = PP[3] * dt + P[1][1] + QgyroKal;
/*
	Kg(k)= P(k|k-1)*HT / (H*P(k|k-1) HT + R)������.���㿨��������
	Kg ��Ӧangle��gyro_bias�Ŀ��������� |K_0|
										|K_1|
	H = |1   0|   ���۲����Z ���ɲ���ֵaccelAngleֱ�ӵó���gyro_bias��accelAngle�޹�	
	RΪ���ٶȼƲ������ĽǶ�ֵ������
	P(k|k-1)*HT = |a   b| * | 1 | = | a |
	              |c   d|   | 0 |   | c |
	H*P(k|k-1)*HT = |1  0| * |a   b| * | 1 | = a
                             |c   d|   | 0 |
*/
	k0 = P[0][0];  
	k1 = P[1][0];
	
	E = RangleKal + P[0][0];  // ��ĸ 
	
	K_0 = k0 / E;
	K_1 = k1 / E;
	

/*	X(k|k)= X(k|k-1) + Kg(k)(Z(k) - H*X(k|k-1))������.��������
	Z = accelAngle
	H*X(k|k-1)) = |1   0| * |angle    | = angle
	                        |gyro_bias|
	X(k|k) = |angle    | + |K_0|  * angle
	         |gyro_bias|   |K_1|
*/
	angle_err = accelAngle - angleKal;	//zk-�������
	angleKal += K_0 * angle_err;	 //�������
	gyro_bias += K_1 * angle_err;	 //�������
	gyroKal = Gyro - gyro_bias;	 //���ֵ(�������)��΢��=���ٶ�
/*
	P(k|k)=�� I-Kg(k) H�� P(k|k-1)������.�������Э����
	I = |1  0|
	    |0  1|
	Kg(k) H P(k|k-1) = |K_0| * |1  0| * |a   b| = |K_0*a   K_0*b|
                       |K_1|            |c   d|   |K_1*a   K_1*b|

*/
	
	P[0][0] -= K_0 * P[0][0];		 //����������Э����
	P[0][1] -= K_0 * P[0][1];
	P[1][0] -= K_1 * P[0][0];
	P[1][1] -= K_1 * P[0][1];
	
	return angleKal;
}

