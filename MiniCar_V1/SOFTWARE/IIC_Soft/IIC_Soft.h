#ifndef __IIC_H__
#define __IIC_H__
#include "sys.h"
#include "delay.h"

//SWIIC GPIO定义和时钟线定义
#define IIC_SCL_GPIO	GPIOB
#define IIC_SCL_Pin		GPIO_Pin_9
#define IIC_SCL_GPIOCLK	RCC_APB2Periph_GPIOB
#define IIC_SDA_GPIO	GPIOB
#define IIC_SDA_Pin		GPIO_Pin_8
#define IIC_SDA_GPIOCLK	RCC_APB2Periph_GPIOB
//IO方向设置(仅适用于F1)
#define SDA_IN() {unsigned char n = 0;while(IIC_SDA_Pin>>n != 1){n++;}IIC_SDA_GPIO->CRL&=~(0xF<<(n*4));IIC_SDA_GPIO->CRL|=(u32)8<<(n*4);} //将GPIOx的GPIO_Pin设置为上拉输入模式
#define SDA_OUT() {unsigned char n = 0;while(IIC_SDA_Pin>>n != 1){n++;}IIC_SDA_GPIO->CRL&=~(0xF<<(n*4));IIC_SDA_GPIO->CRL|=(u32)3<<(n*4);} //将GPIOx的GPIO_Pin设置为推挽输出模式

//IO操作函数
#define IIC_SCL_High    GPIO_SetBits(IIC_SCL_GPIO,IIC_SCL_Pin) //SCL拉高
#define IIC_SCL_Low		GPIO_ResetBits(IIC_SCL_GPIO,IIC_SCL_Pin) //SCL拉低
#define IIC_SDA_High    GPIO_SetBits(IIC_SDA_GPIO,IIC_SDA_Pin) //SDA拉高
#define IIC_SDA_Low		GPIO_ResetBits(IIC_SDA_GPIO,IIC_SDA_Pin) //SDA拉低
#define READ_SDA   		GPIO_ReadInputDataBit(IIC_SDA_GPIO,IIC_SDA_Pin)//读取SDA输入电频

void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(u8 Byte);
u8 IIC_Read_Byte(u8 ack);
u8 IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
#endif
