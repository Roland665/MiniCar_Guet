#include "IIC_Hard.h"


//初始化硬件IIC的GPIO复用
void IIC_Init(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	    	//TestLED端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	//复用开漏
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);	  	//根据设定参数初始化
	
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 400000;//最快400Khz
	I2C_InitStructure.I2C_DutyCycle =  I2C_DutyCycle_2;//这里选择传输数据时低电频占空比2:1,但是要I2C_ClockSpeed>100khz时这个占空比配置才会生效，否则硬件占空比还是在1:1
	I2C_InitStructure.I2C_Ack = ENABLE;//默认启用应答
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//7位地址模式
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;//自身地址，单主机模式下给啥都行
	I2C_Init(I2C1,&I2C_InitStructure);

	I2C_Cmd(I2C1,ENABLE);//使能I2C
}

void IIC_Start(){
	I2C_GenerateSTART(I2C1,ENABLE); 
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
}

void IIC_Send_Address_Write(u8 address){
	I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter); 
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);
}
void IIC_Send_Byte(u8 byte){
	I2C_SendData(I2C1,byte); 
	while(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);
}
void IIC_Stop(){
	I2C_GenerateSTOP(I2C1,ENABLE);
}
