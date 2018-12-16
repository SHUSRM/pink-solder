#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"

//IO方向设置
#define SDA_IN()                         \
	{                                    \
		GPIOE->MODER &= ~(3 << (4 * 2)); \
		GPIOE->MODER |= 0 << 4 * 2;      \
	} //PB9输入模式
#define SDA_OUT()                        \
	{                                    \
		GPIOE->MODER &= ~(3 << (4 * 2)); \
		GPIOE->MODER |= 1 << 4 * 2;      \
	} //PB9输出模式
//IO操作函数
#define IIC_SCL PEout(5) //SCL
#define IIC_SDA PEout(4) //SDA
#define READ_SDA PEin(4) //输入SDA

//IIC所有操作函数
void IIC_Init(void);					  //初始化IIC的IO口
void IIC_Start(void);					  //发送IIC开始信号
void IIC_Stop(void);					  //发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);		  //IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack); //IIC读取一个字节
uint8_t IIC_Wait_Ack(void);				  //IIC等待ACK信号
void IIC_Ack(void);						  //IIC发送ACK信号
void IIC_NAck(void);					  //IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr, uint8_t addr, uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr, uint8_t addr);
#endif
