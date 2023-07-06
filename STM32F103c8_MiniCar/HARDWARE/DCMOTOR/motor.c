#include "motor.h"
#include "timer.h"

void MTS_Init(void){
	TIM3_PWM_Init(100-1,7200-1);//72M/7200=1Mhz的计数频率,重装载值100，所以PWM频率为 1Mhz/100=20hz.(周期为0.05s)
    MT_DirctionIO_Init();
	Go();
}

void MT_DirctionIO_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(MTLDIR0_Clock, ENABLE);	 		//使能端口时钟
	GPIO_InitStructure.GPIO_Pin = MTLDIR0_Pin;				//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(MTLDIR0_GPIO, &GPIO_InitStructure);			//根据设定参数初始化

	RCC_APB2PeriphClockCmd(MTLDIR1_Clock, ENABLE);	 		//使能端口时钟
	GPIO_InitStructure.GPIO_Pin = MTLDIR1_Pin;				//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(MTLDIR1_GPIO, &GPIO_InitStructure);			//根据设定参数初始化


	RCC_APB2PeriphClockCmd(MTRDIR1_Clock, ENABLE);	 		//使能端口时钟
	GPIO_InitStructure.GPIO_Pin = MTRDIR1_Pin;				//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(MTRDIR1_GPIO, &GPIO_InitStructure);			//根据设定参数初始化

	RCC_APB2PeriphClockCmd(MTRDIR0_Clock, ENABLE);	 		//使能端口时钟
	GPIO_InitStructure.GPIO_Pin = MTRDIR0_Pin;				//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(MTRDIR0_GPIO, &GPIO_InitStructure);			//根据设定参数初始化
}


void Go(void){
	MTLDIR0 = 0;
	MTRDIR0 = 0;
	MTLDIR1 = 1;
	MTRDIR1 = 1;
}

void Back(void){
	MTLDIR0 = 1;
	MTRDIR0 = 1;
	MTLDIR1 = 0;
	MTRDIR1 = 0;
}
