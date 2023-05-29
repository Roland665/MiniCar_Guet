#include "beep.h"   

void BEEP_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(BEEP_Clock, ENABLE);	 		//使能端口时钟
	
	GPIO_InitStructure.GPIO_Pin = BEEP_Pin;				//LED1端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(BEEP_GPIO, &GPIO_InitStructure);			//根据设定参数初始化
	GPIO_ResetBits(BEEP_GPIO,BEEP_Pin);					//LED1输出高
}
