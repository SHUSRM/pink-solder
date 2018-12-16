#include "Kalman.h"

//float K1 =0.02; 
//float angleKal, gyroKal; 

//float QangleKal=0.001;// ������������Э���� ����������Э���� 
//float QgyroKal=0.003;//������Ư��������Э���� ����������Э���� Ϊһ��һ�����о���
//float RangleKal=0.7;//0.5;// ���ٶȲ���������Э���� �Ȳ���ƫ��

//float dt=0.005;// ����ʱ�� �˲�������ʱ��5ms  

//char  H_0 = 1;// H����һ����

//float gyro_bias, angle_err; //������Ư��,

//float k0,k1, E; //�м����

//float K_0, K_1;  //K�ǿ��������� Kg

//float PP[4] ={0,0,0,0};  //����P������м����
//float P[2][2] = { { 1, 0 },{ 0, 1 } };  //P����  X ��Э����


void KalmanInit(Kalman *kal)
{
	u8 i;
	kal->QangleKal=0.001;// ������������Э���� ����������Э���� 
	kal->QgyroKal=0.003;//������Ư��������Э���� ����������Э���� Ϊһ��һ�����о���
	kal->RangleKal=0.5;// ���ٶȲ���������Э���� �Ȳ���ƫ��
	kal->dt=0.005;// ����ʱ�� �˲�������ʱ��5ms  
	for(i = 0; i<4;i++)
		kal->PP[i] = 0;
	kal->P[0][0] = 1;
	kal->P[0][1] = 0;
	kal->P[1][0] = 0;
	kal->P[1][1] = 1;
}

//���׿������˲�
void KalmanFilter(float accelAngle,float Gyro, Kalman *kal)		
{
	//X(k|k-1)=AX(k-1|k-1)+BU(k)������.�������
	//|angle    | = |1  -dt||angle    | + |dt|gryo
	//|gyro_bias|	|0   1 ||gyro_bias|   |0 |
	kal->Angle +=(Gyro - kal->gyro_bias) * kal->dt; 
	
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
	kal->PP[0] = -kal->P[0][1] - kal->P[1][0]; 
	kal->PP[1] = -kal->P[1][1];
	kal->PP[2] = -kal->P[1][1];
	kal->PP[3] = 0;
	
	kal->P[0][0] = kal->PP[0] * kal->dt + kal->P[0][0] + kal->QangleKal;   
	kal->P[0][1] = kal->PP[1] * kal->dt + kal->P[0][1];  
	kal->P[1][0] = kal->PP[2] * kal->dt + kal->P[1][0];
	kal->P[1][1] = kal->PP[3] * kal->dt + kal->P[1][1] + kal->QgyroKal;
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
	kal->k0 = kal->P[0][0];  
	kal->k1 = kal->P[1][0];
	
	kal->E = kal->RangleKal + kal->P[0][0];  // ��ĸ 
	
	kal->K_0 = kal->k0 / kal->E;
	kal->K_1 = kal->k1 / kal->E;
	

/*	X(k|k)= X(k|k-1) + Kg(k)(Z(k) - H*X(k|k-1))������.��������
	Z = accelAngle
	H*X(k|k-1)) = |1   0| * |angle    | = angle
	                        |gyro_bias|
	X(k|k) = |angle    | + |K_0|  * angle
	         |gyro_bias|   |K_1|
*/
	kal->angle_err = accelAngle - kal->Angle;	//zk-�������
	kal->Angle += kal->K_0 * kal->angle_err;	 //�������
	kal->gyro_bias += kal->K_1 * kal->angle_err;	 //�������
	kal->Gyro = Gyro - kal->gyro_bias;	 //���ֵ(�������)��΢��=���ٶ�
/*
	P(k|k)=�� I-Kg(k) H�� P(k|k-1)������.�������Э����
	I = |1  0|
	    |0  1|
	Kg(k) H P(k|k-1) = |K_0| * |1  0| * |a   b| = |K_0*a   K_0*b|
                       |K_1|            |c   d|   |K_1*a   K_1*b|

*/
	
	kal->P[0][0] -= kal->K_0 * kal->P[0][0];		 //����������Э����
	kal->P[0][1] -= kal->K_0 * kal->P[0][1];
	kal->P[1][0] -= kal->K_1 * kal->P[0][0];
	kal->P[1][1] -= kal->K_1 * kal->P[0][1];
	
}

