#include "mpu6500.h"
#include "spi.h"
#include "gpio.h"
#include "Kalman.h"
#include <math.h>

uint8_t MPU_id = 0;
MPU6050 mpu6500;

//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
	static uint8_t MPU_Rx, MPU_Tx;

	MPU6500_NSS = 0;

	MPU_Tx = reg & 0x7f;
	HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
	MPU_Tx = data;
	HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);

	MPU6500_NSS = 1;
	return 0;
}
//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
	static uint8_t MPU_Rx, MPU_Tx;

	MPU6500_NSS = 0;

	MPU_Tx = reg | 0x80;
	HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
	HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);

	MPU6500_NSS = 1;
	return MPU_Rx;
}
//Read registers from MPU6500,address begin with regAddr
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
	static uint8_t MPU_Rx, MPU_Tx, MPU_Tx_buff[14] = {0xff};
	MPU6500_NSS = 0;

	MPU_Tx = regAddr | 0x80;
	MPU_Tx_buff[0] = MPU_Tx;
	HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
	HAL_SPI_TransmitReceive(&hspi5, MPU_Tx_buff, pData, len, 55);

	MPU6500_NSS = 1;
	return 0;
}
//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr << 3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr << 3);
}
//Initialize the MPU6500
uint8_t MPU6500_Init(void)
{
	uint8_t index = 0;
	uint8_t MPU6500_Init_Data[10][2] =
		{
			{MPU6500_PWR_MGMT_1, 0x80},		// Reset Device
			{MPU6500_PWR_MGMT_1, 0x03},		// Clock Source - Gyro-Z
			{MPU6500_PWR_MGMT_2, 0x00},		// Enable Acc & Gyro
			{MPU6500_CONFIG, 0x02},			// LPF 98Hz
			{MPU6500_GYRO_CONFIG, 0x18},	// +-2000dps
			{MPU6500_ACCEL_CONFIG, 0x10},   // +-8G
			{MPU6500_ACCEL_CONFIG_2, 0x02}, // enable LowPassFilter  Set Acc LPF
			{MPU6500_USER_CTRL, 0x20},		// Enable AUX
		};

	HAL_Delay(100);
	MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I); //read id of device,check if MPU6500 or not

	for (index = 0; index < 10; index++)
	{
		MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
		HAL_Delay(1);
	}
	
	KalmanInit(&mpu6500.PitchK);
	KalmanInit(&mpu6500.RollK);

	return 0;
}

void MPU6500_GetData(void)
{
	uint8_t mpu_buff[14];
	MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
	
	mpu6500.AccX = mpu_buff[0] << 8 | mpu_buff[1];
	mpu6500.AccY = mpu_buff[2] << 8 | mpu_buff[3];
	mpu6500.AccZ = mpu_buff[4] << 8 | mpu_buff[5];
	
	mpu6500.Temp = mpu_buff[6] << 8 | mpu_buff[7];
	
	mpu6500.GyroX = mpu_buff[8] << 8 | mpu_buff[9];
	mpu6500.GyroY = mpu_buff[10] << 8 | mpu_buff[11];
	mpu6500.GyroZ = mpu_buff[12] << 8 | mpu_buff[13];
	
	KalmanFilter(atan2(mpu6500.AccX,mpu6500.AccZ)*180/PI,-mpu6500.GyroY/16.4,&mpu6500.PitchK);
	//KalmanFilter(atan2(mpu6500.AccY,mpu6500.AccZ)*180/PI,-mpu6500.GyroX/16.4,&mpu6500.RollK);
	
}

